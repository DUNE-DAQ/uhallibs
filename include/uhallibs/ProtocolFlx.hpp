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

#ifndef _DUNEDAQ_UHALLIBS_PROTOCOLFLX_V0_HPP_
#define _DUNEDAQ_UHALLIBS_PROTOCOLFLX_V0_HPP_


#include <deque>                           // for deque
#include <istream>                         // for istream
#include <stddef.h>                        // for size_t
#include <stdint.h>                        // for uint32_t, uint8_t
#include <string>                          // for string
#include <utility>                         // for pair
#include <vector>                          // for vector

#include <chrono>                          // for steady_clock

#include "uhal/ClientInterface.hpp"
#include "uhal/log/exception.hpp"
#include "uhal/ProtocolIPbus.hpp"

#include "flxcard/FlxCard.h"

#include "uhallibs/ipc/RobustMutex.hpp"
#include "uhallibs/ipc/SharedMemObject.hpp"

namespace boost
{
  template <class Y> class shared_ptr;
}

namespace uhal {
  struct URI;
  class Buffers;
}

namespace uhallibs
{

  namespace exception
  {
    //! Exception class to handle the case in which the PCIe connection timed out.
    UHAL_DEFINE_DERIVED_EXCEPTION_CLASS(
      FlxInvalidDevice, 
      uhal::exception::ClientTimeout, 
      "Exception class to handle the case in which the Felix device is invalid."
    )
    //! Exception class to handle the case in which the PCIe connection timed out.
    UHAL_DEFINE_DERIVED_EXCEPTION_CLASS(
      FlxTimeout, 
      uhal::exception::ClientTimeout, 
      "Exception class to handle the case in which the Felix connection timed out."
    )
    //! Exception class to handle a failure to read from the specified device files during initialisation
    UHAL_DEFINE_DERIVED_EXCEPTION_CLASS (
      FlxInitialisationError ,
      uhal::exception::TransportLayerError ,
      "Exception class to handle a failure to read from the specified device files during initialisation."
    )
    //! Exception class to handle a low-level seek/read/write error after initialisation
    UHAL_DEFINE_DERIVED_EXCEPTION_CLASS (
      FlxCommunicationError ,
      uhal::exception::TransportLayerError ,
      "Exception class to handle a low-level seek/read/write error after initialisation." 
    )
  }

  //! Transport protocol to transfer an IPbus buffer via device file, using mmap
  class Flx : public uhal::IPbus< 2 , 0 >
  {
    private:
      class Card {
      public:
        Card(const std::string& aPath, u_int aLockMask);
        ~Card();

        const std::string& getPath() const;

        int getDeviceId() const;

        void open();

        void close();

        void read(const uint32_t aAddr, const uint32_t aNrWords, std::vector<uint32_t>& aValues);

        void write(const uint32_t aAddr, const std::vector<std::pair<const uint8_t*, size_t> >& aData);

        bool haveLock() const;

        void lock();

        void unlock();

      private:
        
        static regmap_register_t* find_reg( const std::string& aName );


        std::string mPath;
        int mDeviceId;
        u_int mLockMask;
        
        FlxCard mFlxCard;
        bool mIsOpen;

        int mFd;
        bool mLocked;
        

      };

      template <typename T>
      struct HexTo {
        T value;
        operator T() const {return value;}
        friend std::istream& operator>>(std::istream& in, HexTo& out)
        {
          in >> std::hex >> out.value;
          return in;
        }
      };

      Flx ( const Flx& aFlx );

      Flx& operator= ( const Flx& aFlx );

    public:
      /**
        Constructor
        @param aId the uinique identifier that the client will be given.
        @param aUri a struct containing the full URI of the target.
      */
      Flx ( const std::string& aId, const uhal::URI& aUri );

      //!	Destructor
      virtual ~Flx();

    private:
      typedef ipc::RobustMutex IPCMutex_t;
      typedef std::unique_lock<IPCMutex_t> IPCScopedLock_t;

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

      static std::string getSharedMemName(const std::string& aPath);

      typedef IPbus< 2 , 0 > InnerProtocol;

      typedef std::chrono::steady_clock SteadyClock_t;

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

      Card mDeviceFile;

      ipc::SharedMemObject<IPCMutex_t> mIPCMutex;
      bool mIPCExternalSessionActive;
      uint64_t mIPCSessionCount;


      std::chrono::microseconds mSleepDuration;

      uint32_t mNumberOfPages, mPageSize, mIndexNextPage, mPublishedReplyPageCount, mReadReplyPageCount;

      //! The list of buffers still awaiting a reply
      std::deque < std::shared_ptr< uhal::Buffers > > mReplyQueue;

      /**
        A pointer to an exception object for passing exceptions from the worker thread to the main thread.
        Exceptions must always be created on the heap (i.e. using `new`) and deletion will be handled in the main thread
      */
      uhal::exception::exception* mAsynchronousException;
  };


}


#endif /* _DUNEDAQ_UHALLIBS_PROTOCOLFLX_V0_HPP_ */
