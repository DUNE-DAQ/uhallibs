/*******************************************************************/
/*                                                                 */
/* This is the C++ source code of the flx-dma-test application     */
/*                                                                 */
/* Author: Markus Joos, CERN                                       */
/*                                                                 */
/**C 2015 Ecosoft - Made from at least 80% recycled source code*****/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>

#include "cmem_rcc/cmem_rcc.h"
#include "flxcard/FlxCard.h"
#include "flxcard/FlxException.h"


#define APPLICATION_NAME    "flx-fromhost-dma-test"
#define BUFSIZE (1024)
#define DMA_ID (0)

#ifndef BF_GBT_EMU_ENA_TOHOST
#define BF_GBT_EMU_ENA_TOHOST BF_FE_EMU_ENA_EMU_TOHOST
#endif
#ifndef BF_GBT_EMU_ENA_TOFRONTEND
#define BF_GBT_EMU_ENA_TOFRONTEND BF_FE_EMU_ENA_EMU_TOFRONTEND
#endif


//Globals
FlxCard flxCard;

int dma_softw_ptr;

class CmemBuffer
{
public:
  CmemBuffer(size_t size, const char* title = "FlxThroughput") : size_(size)
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
  uint8_t* access_ptr() const { return (uint8_t*)virt_address(); }
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

  void transfer_to_host(CmemBuffer& buffer, size_t offset, size_t size) { flx.flxcard.dma_to_host(dma_id, buffer.phys_address()+offset, size, FLX_DMA_WRAPAROUND); }
  void transfer_from_host(CmemBuffer& buffer, size_t offset, size_t size) { flx.flxcard.dma_from_host(dma_id, buffer.phys_address()+offset, size, FLX_DMA_WRAPAROUND); }
  void wait() { flx.flxcard.dma_wait(dma_id); }
  void stop() { flx.flxcard.dma_stop(dma_id); }

private:
  Flx& flx;
  unsigned dma_id;
};




/*****************/
void display_help()
/*****************/
{
  printf("Usage: %s [OPTIONS]\n", APPLICATION_NAME);
  printf("Initializes DMA transfers and the DMA memory on the screen in 1s intervals.\n\n");
  printf("Options:\n");
  printf("  -d NUMBER      Use card indicated by NUMBER. Default: 0.\n");
  printf("  -h             Display help.\n");
}


/********************************/
void dump_buffer(u_long virt_addr)
/********************************/
{
  u_char *buf = (u_char *)virt_addr;
  int i;

  for(i = 0; i < BUFSIZE; i++)
  {
    if(i % 32 == 0)
      printf("\n0x  ");
    printf("%02x ", *buf++);
  }
  printf("\n");
}


/*****************************/
int main(int argc, char **argv)
/*****************************/
{
  int i, loop, ret, device_number = 0, opt, handle, debuglevel;
  u_long baraddr0, vaddr, vaddr1, paddr1, paddr, board_id, bsize, opt_emu_ena_to_host, opt_emu_ena_to_frontend;
  flxcard_bar0_regs_t *bar0;

  while((opt = getopt(argc, argv, "hd:D:V")) != -1)
  {
    switch (opt)
    {
      case 'd':
        device_number = atoi(optarg);
        break;

      case 'h':
        display_help();
        exit(0);
        break;

      default:
        fprintf(stderr, "Usage: %s COMMAND [OPTIONS]\nTry %s -h for more information.\n", APPLICATION_NAME, APPLICATION_NAME);
        exit(-1);
    }
  }

  try
  {
    flxCard.card_open(device_number, 0);

    // save current state
    opt_emu_ena_to_host     = flxCard.cfg_get_option(BF_GBT_EMU_ENA_TOHOST);
    opt_emu_ena_to_frontend = flxCard.cfg_get_option(BF_GBT_EMU_ENA_TOFRONTEND);

    for(loop = 0; loop < 8; loop++)
      flxCard.dma_stop(loop);

    flxCard.dma_reset();
    flxCard.soft_reset();
    //flxCard.dma_fifo_flush(); MJ: Method disabled (requsted by Frans)
    flxCard.cfg_set_option(BF_GBT_EMU_ENA_TOFRONTEND, 0);
    flxCard.cfg_set_option(BF_GBT_EMU_ENA_TOHOST, 1);

    board_id = flxCard.cfg_get_option(REG_GIT_TAG); 

    u_long loop;
    char git_tag[8];
    for(loop = 0; loop < 8; loop++)
      git_tag[loop] = (board_id >> (8 * loop)) & 0xff;      
    printf("Board ID (GIT): %s\n", git_tag);   

   // ret = CMEM_Open();
   //bsize = 1;
   // if (!ret)
   //   ret = CMEM_SegmentAllocate(bsize, (char *)"FlxThroughput", &handle);
//
   // if (!ret)
   //   ret = CMEM_SegmentPhysicalAddress(handle, &paddr);

//    if (!ret)
  //    ret = CMEM_SegmentVirtualAddress(handle, &vaddr);

//    if (ret)
  //  {
   //   rcc_error_print(stdout, ret);
   //   exit(-1);
    //}

    //bsize = BUFSIZE;
    printf("0x%016lx\n", BUFSIZE);

    Flx flx(0, LOCK_DMA0);
    CmemBuffer cmembuf0(BUFSIZE, "FlxThroughput");
    //CmemBuffer cmembuf1(bsize, "test1");

    uint8_t* buf0 = cmembuf0.access_ptr();
    //uint8_t* buf1 = cmembuf1.access_ptr();
  
    //buf0[0] = 0xFE;
    //buf0[1] = 0xED;
    //buf0[2] = 0xF0;
    //buf0[3] = 0x0D;

    vaddr = cmembuf0.virt_address();
    paddr = cmembuf0.phys_address();

    //vaddr1 = cmembuf1.virt_address();
    //paddr1 = cmembuf1.phys_address();

    printf("\nBuffer before DMA write:\n");
    dump_buffer(vaddr);
    //dump_buffer(vaddr1);

    printf("\nInitiate Simple DMA write:\n");
    FlxSimpleDMA dma(flx, 0);

    dma.transfer_from_host(cmembuf0, 0, BUFSIZE);
    //dma.transfer_from_host(cmembuf1, 0, 4096);

   
  
//    vaddr = cmembuf0.virt_address();
//    paddr = cmembuf0.phys_address();

//    vaddr1 = cmembuf1.virt_address();
//    paddr1 = cmembuf1.phys_address();


    //dma.transfer_to_host(cmembuf1, 0, 4096);


    //flxCard.dma_from_host(DMA_ID, paddr, BUFSIZE, FLX_DMA_WRAPAROUND);
    //flx.flxcard.dma_to_host(DMA_ID, buffer.phys_address()+offset, size, 0);



//    printf("softw_ptr:  0x%016lx\n", dma_softw_ptr);

    //flxCard.dma_wait(DMA_ID);

    baraddr0 = flxCard.openBackDoor(0);
    bar0 = (flxcard_bar0_regs_t *)baraddr0;

    printf("Start Ptr:   0x%016lx\n", bar0->DMA_DESC[0].start_address);
    printf("End Ptr:     0x%016lx\n", bar0->DMA_DESC[0].end_address);
    printf("Enable:      0x%0x\n", bar0->DMA_DESC_ENABLE);
    printf("Read Ptr:    0x%016lx\n", bar0->DMA_DESC[0].read_ptr);
    printf("Write Ptr:   0x%016lx\n", bar0->DMA_DESC_STATUS[0].current_address);
    printf("Descriptor done DMA0: 0x%lx\n", bar0->DMA_DESC_STATUS[0].descriptor_done);
    printf("Even Addr. DMA  DMA0: 0x%lx\n", bar0->DMA_DESC_STATUS[0].even_addr_dma);
    printf("Even Addr. PC   DMA0: 0x%lx\n", bar0->DMA_DESC_STATUS[0].even_addr_pc);
    printf("Descriptor done DMA1: 0x%lx\n", bar0->DMA_DESC_STATUS[1].descriptor_done);
    printf("Even Addr. DMA  DMA1: 0x%lx\n", bar0->DMA_DESC_STATUS[1].even_addr_dma);
    printf("Even Addr. PC   DMA1: 0x%lx\n", bar0->DMA_DESC_STATUS[1].even_addr_pc);

    printf("Start Addr: %016lx\nEnd Addr:  %016lx\nRead Ptr: %016lx\n", bar0->DMA_DESC[0].start_address, bar0->DMA_DESC[0].end_address, bar0->DMA_DESC[0].read_ptr);


    printf("\nBuffer after DMA write:\n");
    dump_buffer(vaddr);
    //dump_buffer(vaddr1);

    for(i = 0; ; i++)
    {
      printf("\n--------------------\n  %d:\n", i);
      flxCard.dma_advance_ptr(DMA_ID, paddr, BUFSIZE, 512);
      //flxCard.dma_wait(DMA_ID);

      printf("Read Ptr:    0x%016lx\n", bar0->DMA_DESC[0].read_ptr);
      printf("Write Ptr:   0x%016lx\n", bar0->DMA_DESC_STATUS[0].current_address);
      printf("Descriptor done DMA0: 0x%lx\n", bar0->DMA_DESC_STATUS[0].descriptor_done);
      printf("Even Addr. DMA  DMA0: 0x%lx\n", bar0->DMA_DESC_STATUS[0].even_addr_dma);
      printf("Even Addr. PC   DMA0: 0x%lx\n", bar0->DMA_DESC_STATUS[0].even_addr_pc);
	

      dump_buffer(vaddr);
      //dump_buffer(vaddr1);
      sleep(1);
    }

    // reset to initial state
    flxCard.cfg_set_option(BF_GBT_EMU_ENA_TOHOST, opt_emu_ena_to_host);
    flxCard.cfg_set_option(BF_GBT_EMU_ENA_TOFRONTEND, opt_emu_ena_to_frontend);

    ret = CMEM_SegmentFree(handle);
    if (!ret)
      ret = CMEM_Close();
    if (ret)
      rcc_error_print(stdout, ret);

    flxCard.card_close();
  }
  catch(FlxException &ex)
  {
    std::cout << "ERROR. Exception thrown: " << ex.what() << std:: endl;
    exit(-1);
  }

  exit(0);
}
