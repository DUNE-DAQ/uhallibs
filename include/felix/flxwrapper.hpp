#pragma once

#include "flxcard/FlxCard.h"
#include "flxcard/FlxException.h"

namespace felix
{
namespace core
{

// Forward declarations
class FlxSimpleDMA;
class FlxCircularBufferDMA;


class CmemBuffer
{
public:
  CmemBuffer(size_t size, const char* title = "FelixCore") : size_(size)
  {
    int ret = CMEM_Open();
    if (!ret) ret = CMEM_GFPBPASegmentAllocate(size, (char*)title, &cmem_handle);
    if (!ret) ret = CMEM_SegmentPhysicalAddress(cmem_handle, &paddr);
    if (!ret) ret = CMEM_SegmentVirtualAddress(cmem_handle, &vaddr);
  }

  ~CmemBuffer()
  {
    CMEM_SegmentFree(cmem_handle);
    CMEM_Close();
  }

  CmemBuffer(const CmemBuffer& other) = delete;
  CmemBuffer(const CmemBuffer&& other) = delete;
  CmemBuffer& operator=(const CmemBuffer& other) = delete;
  CmemBuffer& operator=(const CmemBuffer&& other) = delete;

  u_long phys_address() const { return paddr; }
  u_long virt_address() const { return vaddr; }
  uint64_t* access_ptr() const { return (uint64_t*)virt_address(); }
  size_t size() const { return size_; }

private:
  int cmem_handle;
  u_long paddr, vaddr;
  size_t size_;
};


class Flx
{
public:
  Flx(unsigned device, unsigned lock_mask) : device(device) { flxcard.card_open(device, lock_mask); }
  ~Flx() { flxcard.card_close(); }

  Flx(const Flx& other) = delete;
  Flx(const Flx&& other) = delete;
  Flx& operator=(const Flx& other) = delete;
  Flx& operator=(const Flx&& other) = delete;

private:
  unsigned device;
  FlxCard flxcard;

  friend class FlxSimpleDMA;
  friend class FlxCircularBufferDMA;
};


class FlxSimpleDMA
{
public:
  FlxSimpleDMA(Flx& flx, unsigned dma_id) : flx(flx), dma_id(dma_id) { stop(); }
  ~FlxSimpleDMA() { stop(); }

  void transfer_to_host(CmemBuffer& buffer, size_t offset, size_t size) { flx.flxcard.dma_to_host(dma_id, buffer.phys_address()+offset, size, 0); }
  void transfer_from_host(CmemBuffer& buffer, size_t offset, size_t size) { flx.flxcard.dma_from_host(dma_id, buffer.phys_address()+offset, size, 0); }
  void wait() { flx.flxcard.dma_wait(dma_id); }
  void stop() { flx.flxcard.dma_stop(dma_id); }

private:
  Flx& flx;
  unsigned dma_id;
};


class FlxCircularBufferDMA
{
public:
  enum direction_t { TO_HOST, FROM_HOST } direction;

  FlxCircularBufferDMA(Flx& flx, unsigned dma_id, CmemBuffer& buffer, direction_t direction)
  : direction(direction), flx(flx), dma_id(dma_id), buffer(buffer)
  {
    if(direction == TO_HOST)
    {
      flx.flxcard.dma_to_host(dma_id, buffer.phys_address(), buffer.size(), FLX_DMA_WRAPAROUND);
    }
    else if(direction == FROM_HOST)
    {
      flx.flxcard.dma_from_host(dma_id, buffer.phys_address(), buffer.size(), FLX_DMA_WRAPAROUND);
    }
  }

  ~FlxCircularBufferDMA() { flx.flxcard.dma_stop(dma_id); }

  size_t bytes_available()
  {
    u_long ofw = offset_fw();
    u_long osw = offset_sw();
    return (ofw > osw ? ofw-osw : buffer.size()-osw+ofw);
  }

  void advance(size_t bytes)
  {
    flx.flxcard.dma_advance_ptr(dma_id, buffer.phys_address(), buffer.size(), bytes);
  }

  uint8_t* current_pos()
  {
    uint8_t* ptr = (uint8_t*)buffer.virt_address();
    ptr += offset_sw();
    return ptr;
  }


private:
  Flx& flx;
  unsigned dma_id;
  CmemBuffer& buffer;

  u_long get_current_address() { return flx.flxcard.dma_get_current_address(dma_id); }
  u_long offset_fw() { return (get_current_address() - buffer.phys_address()) % buffer.size(); }
  u_long offset_sw() { return (flx.flxcard.m_bar0->DMA_DESC[dma_id].read_ptr - buffer.phys_address()) % buffer.size(); }

};


template <typename T>
class CMEMBaseQueue
{
protected:
  CMEMBaseQueue(Flx& flx, unsigned dma_id, size_t size, FlxCircularBufferDMA::direction_t direction, const char* title)
  : flx(flx), cmembuf(size*sizeof(T), title), circularbuf(flx, dma_id, cmembuf, direction)
  {}

public:
  size_t size()
  {
    return cmembuf.size() / sizeof(T);
  }

  size_t available()
  {
    return circularbuf.bytes_available() / sizeof(T);
  }

protected:
  Flx& flx;
  CmemBuffer cmembuf;
  FlxCircularBufferDMA circularbuf;
};



template <typename T>
class CMEMReadQueue : public CMEMBaseQueue<T>
{
public:
  CMEMReadQueue(Flx& flx, unsigned dma_id, size_t size)
  : CMEMBaseQueue<T>(flx, dma_id, size, FlxCircularBufferDMA::direction_t::TO_HOST, "FelixCore CMEMReadQueue")
  {}

  bool try_pop(T* destination)
  {
    if(this->available() > 0)
    {
      memcpy((void*)destination, this->circularbuf.current_pos(), sizeof(T));
      this->circularbuf.advance(sizeof(T));
      return true;
    }
    else
    {
      return false;
    }
  }
};


template <typename T>
class CMEMWriteQueue : public CMEMBaseQueue<T>
{
public:
  CMEMWriteQueue(Flx& flx, unsigned dma_id, size_t size)
  :  CMEMBaseQueue<T>(flx, dma_id, size, FlxCircularBufferDMA::direction_t::FROM_HOST, "FelixCore CMEMWriteQueue")
  {}

  bool try_push(T* source)
  {
    if(this->available() > 0)
    {
      memcpy((void*)this->circularbuf.current_pos(), source, sizeof(T));
      this->circularbuf.advance(sizeof(T));
      return true;
    }
    else
    {
      return false;
    }
  }
};


}
}