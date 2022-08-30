/*
---------------------------------------------------------------------------

    This file is part of uHAL.

    uHAL is a hardware access library and programming framework
    originally developed for upgrades of the Level-1 trigger of the CMS
    experiment at CERN.

    uHAL is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    uHAL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with uHAL.  If not, see <http://www.gnu.org/licenses/>.


      Andrew Rose, Imperial College, London
      email: awr01 <AT> imperial.ac.uk

      Marc Magrans de Abril, CERN
      email: marc.magrans.de.abril <AT> cern.ch

      Tom Williams, Rutherford Appleton Laboratory, Oxfordshire
      email: tom.williams <AT> cern.ch

---------------------------------------------------------------------------
*/

/**
    @file
    @author Tom Williams
    @date September 2017
*/

#ifndef _DUNEDAQ_UHALLIBS_PROTOCOLAXI4LITE_HPP_
#define _DUNEDAQ_UHALLIBS_PROTOCOLAXI4LITE_HPP_

#include <stddef.h>  // for size_t
#include <stdint.h>  // for uint32_t, uint8_t

#include <chrono>
#include <deque>  // for deque
#include <memory>
#include <mutex>
#include <string>   // for string
#include <utility>  // for pair
#include <vector>   // for vector

#include "uhal/ClientInterface.hpp"
#include "uhal/ProtocolIPbus.hpp"
#include "uhal/log/exception.hpp"
#include "uhallibs/formatters.hpp"
#include "uhallibs/ipc/RobustMutex.hpp"
#include "uhallibs/ipc/SharedMemObject.hpp"

namespace uhal {
  struct URI;
  class Buffers;
}

namespace uhallibs {

namespace exception {
//! Exception class to handle the case in which the Axi4Lite connection timed
//! out.
UHAL_DEFINE_DERIVED_EXCEPTION_CLASS(
  Axi4LiteTimeout, uhal::exception::ClientTimeout,
  "Exception class to handle the case in "
  "which the Axi4Lite connection timed out."
  )
//! Exception class to handle a failure to read from the specified device files
//! during initialisation
UHAL_DEFINE_DERIVED_EXCEPTION_CLASS(
    Axi4LiteInitialisationError, uhal::exception::TransportLayerError,
    "Exception class to handle a failure to read from the specified device "
    "files during initialisation."
    )
//! Exception class to handle a low-level seek/read/write error after
//! initialisation
UHAL_DEFINE_DERIVED_EXCEPTION_CLASS(
    Axi4LiteCommunicationError, uhal::exception::TransportLayerError,
    "Exception class to handle a low-level seek/read/write error after "
    "initialisation."
    )
}  // namespace exception

//! Transport protocol to transfer an IPbus buffer over Axi4Lite mapped in a 32b address space
class Axi4Lite : public uhal::IPbus<2, 0> {
 public:
  class MappedFile {
   public:
    MappedFile(const std::string& aPath, size_t aLength, int aProtFlags = PROT_WRITE);
    ~MappedFile();

    const std::string& getPath() const;

    void setPath(const std::string& aPath);

    void setLength(size_t);

    //! Open bus file and map it to memory
    void open();
    //! Unmap and close file
    void close();

    //! Create a local buffer
    void createBuffer(const size_t aNrBytes);

    void read(const uint32_t aAddr, const uint32_t aNrWords,
              std::vector<uint32_t>& aValues);

    void write(const uint32_t aAddr, const std::vector<uint32_t>& aValues);

    void write(const uint32_t aAddr, const uint8_t* const aPtr,
               const size_t aNrBytes);

    void write(const uint32_t aAddr,
               const std::vector<std::pair<const uint8_t*, size_t> >& aData);

    bool haveLock() const;

    void lock();

    void unlock();

   private:

    std::string mPath;
    int mFd;
    uint32_t* mBar;
    size_t mLength;
    int mProtFlags;
    bool mLocked;
    size_t mBufferSize;
    char* mBuffer;
  };

  /**
    Constructor
    @param aId the uinique identifier that the client will be given.
    @param aUri a struct containing the full URI of the target.
  */
  Axi4Lite(const std::string& aId, const uhal::URI& aUri);

  Axi4Lite ( const Axi4Lite& ) = delete;
  Axi4Lite& operator= ( const Axi4Lite& ) = delete;

  //! Destructor
  virtual ~Axi4Lite();





 private:
  typedef ipc::RobustMutex IPCMutex_t;
  typedef std::unique_lock<IPCMutex_t> IPCScopedLock_t;

  typedef IPbus< 2 , 0 > InnerProtocol;
  typedef std::chrono::steady_clock SteadyClock_t;

  /**
    Send the IPbus buffer to the target, read back the response and call the packing-protocol's validate function
    @param aBuffers the buffer object wrapping the send and recieve buffers that are to be transported
    If multithreaded, adds buffer to the dispatch queue and returns. If single-threaded, calls the dispatch-worker dispatch function directly and blocks until the response is validated.
  */
  void implementDispatch ( std::shared_ptr< uhal::Buffers > aBuffers );

  //! Concrete implementation of the synchronization function to block until all buffers have been sent, all replies received and all data validated
  virtual void Flush( );

  //! Function which tidies up this protocol layer in the event of an exception
  virtual void dispatchExceptionHandler();

  static std::string getDevicePath(const uhal::URI& aUri);

  static std::string getSharedMemName(const std::string& );

  /**
    Return the maximum size to be sent based on the buffer size in the target
    @return the maximum size to be sent
  */
  uint32_t getMaxSendSize();

  /**
    Return the maximum size of reply packet based on the buffer size in the target
    @return the maximum size of reply packet
  */
  uint32_t getMaxReplySize();
  //! Set up the connection to the device
  void connect();

  //! Set up the connection to the device
  void connect( IPCScopedLock_t& );

  //! Close the connection to the device
  void disconnect();

  //! Write request packet to next page in host-to-FPGA device file 
  void write(const std::shared_ptr<uhal::Buffers>& aBuffers);

  //! Read next pending reply packet from appropriate page of FPGA-to-host device file, and validate contents
  void read();

  bool mConnected;

  MappedFile mMappedFile;

  ipc::SharedMemObject<IPCMutex_t> mIPCMutex;
  bool mIPCExternalSessionActive;
  uint64_t mIPCSessionCount;

  std::chrono::microseconds mSleepDuration;

  uint32_t mNumberOfPages, mMaxInFlight, mPageSize, mMaxPacketSize, mIndexNextPage, mPublishedReplyPageCount, mReadReplyPageCount;

  //! The list of buffers still awaiting a reply
  std::deque < std::shared_ptr< uhal::Buffers > > mReplyQueue;
};

}  // namespace uhal

#endif /* _DUNEDAQ_UHALLIBS_PROTOCOLAXI4LITE_HPP_ */
