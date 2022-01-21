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

#include "uhal/ProtocolPCIe4.hpp"


#include <algorithm>                                        // for min
#include <assert.h>
#include <chrono>
#include <cstdlib>
#include <errno.h>
#include <fcntl.h>
#include <iomanip>                                          // for operator<<
#include <iostream>                                         // for operator<<
#include <sys/file.h>
#include <sys/stat.h>
#include <stdlib.h>                                         // for size_t, free
#include <stdio.h>
#include <string.h>                                         // for memcpy
#include <thread>
#include <unistd.h>

#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>  // for time_dura...

#include "uhal/grammars/URI.hpp"                            // for URI
#include "uhal/log/LogLevels.hpp"                           // for BaseLogLevel
#include "uhal/log/log_inserters.integer.hpp"               // for Integer
#include "uhal/log/log_inserters.quote.hpp"                 // for Quote
#include "uhal/log/log.hpp"
#include "uhal/Buffers.hpp"
#include "uhal/ClientFactory.hpp"

UHAL_REGISTER_EXTERNAL_CLIENT(uhal::PCIe4, "ipbuspci4-2.0", "AXI4 Lite IPBus client")
// Syntax: ipbus-qdma-axi4l-2.0:///sys/bus/pci/devices/0000:<bus>:<dev>.<func>/resource<bar#>
// Syntax: ipbus-qdma-axi4l-2.0://<bus>:<dev>.<func>/resource<bar#>

namespace uhal {


PCIe4::PacketFmt::PacketFmt(const uint8_t* const aPtr, const size_t aNrBytes) :
  mData(1, std::pair<const uint8_t*, size_t>(aPtr, aNrBytes))
{}


PCIe4::PacketFmt::PacketFmt(const std::vector< std::pair<const uint8_t*, size_t> >& aData) :
  mData(aData)
{}


PCIe4::PacketFmt::~PacketFmt()
{}


std::ostream& operator<<(std::ostream& aStream, const PCIe4::PacketFmt& aPacket)
{
  std::ios::fmtflags lOrigFlags( aStream.flags() );

  size_t lNrBytesWritten = 0;
  for (size_t i = 0; i < aPacket.mData.size(); i++) {
    for (const uint8_t* lPtr = aPacket.mData.at(i).first; lPtr != (aPacket.mData.at(i).first + aPacket.mData.at(i).second); lPtr++, lNrBytesWritten++) {
      if ((lNrBytesWritten & 3) == 0)
        aStream << std::endl << "   @ " << std::setw(3) << std::dec << (lNrBytesWritten >> 2) << " :  x";
      aStream << std::setw(2) << std::hex << uint16_t(*lPtr) << " ";
    }
  }

  aStream.flags( lOrigFlags );
  return aStream;
}

PCIe4::DMAFmt::DMAFmt(const uint8_t* const aPtr, const size_t aNrBytes) :
  mData(1, std::pair<const uint8_t*, size_t>(aPtr, aNrBytes))
{}

PCIe4::DMAFmt::~DMAFmt()
{}

std::ostream& operator<<(std::ostream& aStream, const PCIe4::DMAFmt& aPacket)
{
  std::ios::fmtflags lOrigFlags( aStream.flags() );

  size_t lNrBytesWritten = 0;
  for (size_t i = 0; i < aPacket.mData.size(); i++) {
    for (const uint8_t* lPtr = aPacket.mData.at(i).first; lPtr != (aPacket.mData.at(i).first + aPacket.mData.at(i).second); lPtr++, lNrBytesWritten++) {
      if ((lNrBytesWritten & 63) == 0)
        aStream << std::endl << "   @ " << std::setw(3) << std::dec << (lNrBytesWritten >> 2) << " :  x";
      aStream << std::setw(2) << std::hex << uint16_t(*lPtr) << " ";
    }
  }

  aStream.flags( lOrigFlags );
  return aStream;
}



PCIe4::File::File(const std::string& aPath, int aFlags) :
  mPath(aPath),
  mFd(-1),
  mFlags(aFlags),
  mLocked(false),
  mBufferSize(0),
  mBuffer(NULL)
{
}


PCIe4::File::~File()
{
  free(mBuffer);
  close();
}


const std::string& PCIe4::File::getPath() const
{
  return mPath;
}


void PCIe4::File::setPath(const std::string& aPath)
{
  mPath = aPath;
}


void PCIe4::File::open()
{
  if (mFd != -1)
    return;

  mFd = ::open(mPath.c_str(), mFlags);
  if ( mFd == -1 ) {
    exception::PCIe4InitialisationError lExc;
    log(lExc, "Failed to open device file ", Quote(mPath), "; errno=", Integer(errno), ", meaning ", Quote (strerror(errno)));
    throw lExc;
  }
}


void PCIe4::File::close()
{
  if (mFd != -1) {
    if (haveLock())
      unlock();
    int rc = ::close(mFd);
    mFd = -1;
    if (rc == -1)
      log (Error(), "Failed to close file ", Quote(mPath), "; errno=", Integer(errno), ", meaning ", Quote (strerror(errno)));
  }
}



void PCIe4::File::createBuffer(const size_t aNrBytes)
{
  if (mBuffer != NULL) {
    if (mBufferSize >= aNrBytes)
      return;
    else {
      free(mBuffer);
      mBuffer = NULL;
      mBufferSize = 0;
    }
  }

  posix_memalign((void**)&mBuffer, 4096/*alignment*/, aNrBytes + 4096);
  if (mBuffer == NULL) {
    exception::PCIe4CommunicationError lExc;
    log(lExc, "Failed to allocate ", Integer(aNrBytes + 4096), " bytes in PCIe4::File::createBuffer");
    throw lExc;
  }

  mBufferSize=aNrBytes+4096;
}


void PCIe4::File::read(const uint32_t aAddr, const uint32_t aNrWords, std::vector<uint32_t>& aValues)
{
  if (mFd == -1)
    open();

  // Here's the tricky bit. In Gen4x16 datawords are 512b wide. But the firmware is considering only the first 64b.
  // So every 2 32b words, a block of 512b has to be reserved.
  // 16 = 512/32
  uint32_t lNrWordsBuffer = (aNrWords/2 + aNrWords%2)*16;
  createBuffer(4 * lNrWordsBuffer);

  /* select AXI MM address */
  // The address needs also changing to accommodate for the jumps
  uint32_t lAddrBuffer = (aAddr/2)*16 + aAddr%2;
  off_t off = lseek(mFd, 4*lAddrBuffer, SEEK_SET);
  if ( off != off_t(4 * lAddrBuffer)) {
    exception::PCIe4CommunicationError lExc;
    log(lExc, "Offset returned by lseek, ", Integer(off), ", does not match that requested, ", Integer(4*lAddrBuffer), " (in preparation for read of ", Integer(lNrWordsBuffer), " words)");
    throw lExc;
  }

  /* read data from AXI MM into buffer using SGDMA */
  int rc = ::read(mFd, mBuffer, 4*lNrWordsBuffer);
  if (rc == -1) {
    exception::PCIe4CommunicationError lExc;
    log(lExc, "Read of ", Integer(4*lNrWordsBuffer), " bytes at address ", Integer(4 * lAddrBuffer), " failed! errno=", Integer(errno), ", meaning ", Quote (strerror(errno)));
    throw lExc;
  }
  else if (size_t(rc) < 4*lNrWordsBuffer) {
    exception::PCIe4CommunicationError lExc;
    log(lExc, "Only ", Integer(rc), " bytes transferred in read of ", Integer(4*lNrWordsBuffer), " bytes at address ", Integer(4 * lAddrBuffer));
    throw lExc;
  }

  log (Debug(), "Reading " , Integer(4*lNrWordsBuffer), " 8-bit words at address " , Integer(4*lAddrBuffer), " ... ", DMAFmt((const uint8_t* const)mBuffer, 4*lNrWordsBuffer));

  // Pick the wbitsords corresponding to the first 2 32b words each 512b
  auto lBuffer32 = reinterpret_cast<uint32_t*>(mBuffer);
  for( size_t i(0); i<aNrWords; ++i) {
    uint32_t iBuff = (i/2)*16 + i%2;
    aValues.push_back(lBuffer32[iBuff]);
  }
  // aValues.insert(aValues.end(), reinterpret_cast<uint32_t*>(mBuffer), reinterpret_cast<uint32_t*>(mBuffer)+lNrWords);
}


void PCIe4::File::write(const uint32_t aAddr, const std::vector<uint32_t>& aValues)
{
  write(4 * aAddr, reinterpret_cast<const uint8_t*>(aValues.data()), 4 * aValues.size());
}


void PCIe4::File::write(const uint32_t aAddr, const uint8_t* const aPtr, const size_t aNrBytes)
{
  if (mFd == -1)
    open();

  assert((aNrBytes % 4) == 0);

  uint32_t lNrWordsData = aNrBytes/4;
  uint32_t lNrWordsBuffer = (lNrWordsData/2 + lNrWordsData%2)*16;
  uint32_t lNrBytesBuffer = lNrWordsBuffer*4;
  createBuffer(4 * lNrWordsBuffer);

  // data to write to register address
  // memcpy(mBuffer, aPtr, lNrWordsBuffer);
  auto lBuffer32 = reinterpret_cast<uint32_t*>(mBuffer);
  auto lPtr32 = reinterpret_cast<const uint32_t*>(aPtr);
  for( size_t i(0); i<lNrWordsData; ++i) {
    uint32_t iBuff = (i/2)*16 + i%2;
    lBuffer32[iBuff] = lPtr32[i];
  }

  /* select AXI MM address */
  uint32_t lAddrBuffer = (aAddr/2)*16 + aAddr%2;
  off_t off = lseek(mFd, 4*lAddrBuffer, SEEK_SET);
  if ( off != off_t(lAddrBuffer)) {
    struct stat st;
    if (fstat(mFd, &st) or (not S_ISFIFO(st.st_mode))) {
      exception::PCIe4CommunicationError lExc;
      log(lExc, "Offset returned by lseek, ", Integer(off), ", does not match that requested, ", Integer(lAddrBuffer), " (in preparation for write of ", Integer(lNrBytesBuffer), " bytes)");
      throw lExc;
    }
  }

  log (Debug(), "Writing " , Integer(lNrBytesBuffer), " 8-bit words at address " , Integer(4*lAddrBuffer), " ... ", DMAFmt((const uint8_t* const)mBuffer, lNrBytesBuffer));

  /* write buffer to AXI MM address using SGDMA */
  int rc = ::write(mFd, mBuffer, lNrBytesBuffer);
  if (rc == -1) {
    exception::PCIe4CommunicationError lExc;
    log(lExc, "Write of ", Integer(lNrBytesBuffer), " bytes at address ", Integer(lAddrBuffer), " failed! errno=", Integer(errno), ", meaning ", Quote (strerror(errno)));
    throw lExc;
  }
  else if (size_t(rc) < lNrBytesBuffer) {
    exception::PCIe4CommunicationError lExc;
    log(lExc, "Only ", Integer(rc), " bytes transferred in write of ", Integer(lNrBytesBuffer), " bytes at address ", Integer(lAddrBuffer));
    throw lExc;
  }
}


void PCIe4::File::write(const uint32_t aAddr, const std::vector<std::pair<const uint8_t*, size_t> >& aData)
{
  if (mFd == -1)
    open();

  size_t lNrBytes = 0;
  for (size_t i = 0; i < aData.size(); i++)
    lNrBytes += aData.at(i).second;

  assert((lNrBytes % 4) == 0);

  uint32_t lNrWordsData = lNrBytes/4;
  uint32_t lNrWordsBuffer = (lNrWordsData/2 + lNrWordsData%2)*16;
  uint32_t lNrBytesBuffer = lNrWordsBuffer*4;
  createBuffer(4 * lNrWordsBuffer);

  // data to write to register address
  size_t lOffset = 0;
  for (size_t i = 0; i < aData.size(); ++i) {
    for (size_t j = 0; j < aData.at(i).second; ++j) {
      mBuffer[lOffset] = aData.at(i).first[j];
      ++lOffset;
      // Jump ahead to the next 512b offset (64 bytes)
      // |--------************...
      // |wwwwwwwwxxxxxxxxxxxxxx...

      if ( lOffset%64 == 8) {
        lOffset += 56;
      }
    }
  }

  /* select AXI MM address */
  uint32_t lAddrBuffer = (aAddr/2)*16 + aAddr%2;
  off_t off = lseek(mFd, 4*lAddrBuffer, SEEK_SET);
  if ( off != off_t(aAddr)) {
    struct stat st;
    if (fstat(mFd, &st) or (not S_ISFIFO(st.st_mode))) {
      exception::PCIe4CommunicationError lExc;
      log(lExc, "Offset returned by lseek, ", Integer(off), ", does not match that requested, ", Integer(aAddr), " (in preparation for write of ", Integer(lNrBytesBuffer), " bytes)");
      throw lExc;
    }
  }

  log (Debug(), "Writing " , Integer(lNrBytesBuffer), " 8-bit words at address " , Integer(4*lAddrBuffer), " ... ", DMAFmt((const uint8_t* const)mBuffer, lNrBytesBuffer));

  /* write buffer to AXI MM address using SGDMA */
  int rc = ::write(mFd, mBuffer, lNrBytesBuffer);
  if (rc == -1) {
    exception::PCIe4CommunicationError lExc;
    log(lExc, "Write of ", Integer(lNrBytesBuffer), " bytes at address ", Integer(aAddr), " failed! errno=", Integer(errno), ", meaning ", Quote (strerror(errno)));
    throw lExc;
  }
  else if (size_t(rc) < lNrBytesBuffer) {
    exception::PCIe4CommunicationError lExc;
    log(lExc, "Only ", Integer(rc), " bytes transferred in write of ", Integer(lNrBytesBuffer), " bytes at address ", Integer(aAddr));
    throw lExc;
  }
}


bool PCIe4::File::haveLock() const
{
  return mLocked;
}


void PCIe4::File::lock()
{
  if ( flock(mFd, LOCK_EX) == -1 ) {
    exception::MutexError lExc;
    log(lExc, "Failed to lock device file ", Quote(mPath), "; errno=", Integer(errno), ", meaning ", Quote (strerror(errno)));
    throw lExc;
  }
  mLocked = true;
}


void PCIe4::File::unlock()
{
  if ( flock(mFd, LOCK_UN) == -1 ) {
    log(Warning(), "Failed to unlock device file ", Quote(mPath), "; errno=", Integer(errno), ", meaning ", Quote (strerror(errno)));
  }
  else
    mLocked = false;
}




PCIe4::RobustMutex::RobustMutex() :
  mCount(0),
  mSessionActive(false)
{
  pthread_mutexattr_t lAttr;

  int s = pthread_mutexattr_init(&lAttr);
  if (s != 0) {
    exception::MutexError lExc;
    log(lExc, "Error code ", Integer(s), " (", strerror(s), ") returned in mutex attr initialisation");
    throw lExc;
  }

  s = pthread_mutexattr_setpshared(&lAttr, PTHREAD_PROCESS_SHARED);
  if (s != 0) {
    exception::MutexError lExc;
    log(lExc, "Error code ", Integer(s), " (", strerror(s), ") returned by pthread_mutexattr_setpshared");
    throw lExc;
  }

  s = pthread_mutexattr_setrobust(&lAttr, PTHREAD_MUTEX_ROBUST);
  if (s != 0) {
    exception::MutexError lExc;
    log(lExc, "Error code ", Integer(s), " (", strerror(s), ") returned by pthread_mutexattr_setrobust");
    throw lExc;
  }

  s = pthread_mutex_init(&mMutex, &lAttr);
  if (s != 0) {
    exception::MutexError lExc;
    log(lExc, "Error code ", Integer(s), " (", strerror(s), ") returned in mutex initialisation");
    throw lExc;
  }
}


PCIe4::RobustMutex::~RobustMutex()
{
}


void PCIe4::RobustMutex::lock()
{
  int s = pthread_mutex_lock(&mMutex);
  bool lLastOwnerDied = (s == EOWNERDEAD);
  if (lLastOwnerDied)
    s = pthread_mutex_consistent(&mMutex);

  if (s != 0) {
    exception::MutexError lExc;
    log(lExc, "Error code ", Integer(s), " (", strerror(s), ") returned when ", lLastOwnerDied ? "making mutex state consistent" : "locking mutex");
    throw lExc;
  }
}


void PCIe4::RobustMutex::unlock()
{
  int s = pthread_mutex_unlock(&mMutex);
  if (s != 0)
    log(Error(), "Error code ", Integer(s), " (", strerror(s), ") returned when unlocking mutex");
}


uint64_t PCIe4::RobustMutex::getCounter() const
{
  return mCount;
}


bool PCIe4::RobustMutex::isActive() const
{
  return mSessionActive;
}


void PCIe4::RobustMutex::startSession()
{
  mCount++;
  mSessionActive = true;
}

void PCIe4::RobustMutex::endSession()
{
  mSessionActive = false;
}




template <class T>
PCIe4::SharedObject<T>::SharedObject(const std::string& aName) :
  mName(aName),
  mSharedMem(boost::interprocess::open_or_create, aName.c_str(), 1024, 0x0, boost::interprocess::permissions(0666)),
  mObj(mSharedMem.find_or_construct<T>(boost::interprocess::unique_instance)())
{
}

template <class T>
PCIe4::SharedObject<T>::~SharedObject()
{
  // boost::interprocess::shared_memory_object::remove(mName.c_str());
}

template <class T>
T* PCIe4::SharedObject<T>::operator->()
{
  return mObj;
}

template <class T>
T& PCIe4::SharedObject<T>::operator*()
{
  return *mObj;
}




std::string PCIe4::getSharedMemName(const std::string& aPath)
{
  std::string lSanitizedPath(aPath);
  std::replace(lSanitizedPath.begin(), lSanitizedPath.end(), '/', ':');

  return "/uhal::ipbuspcie4-2.0::" + lSanitizedPath;
}


PCIe4::PCIe4 ( const std::string& aId, const URI& aUri ) :
  IPbus< 2 , 0 > ( aId , aUri ),
  mConnected(false),
  mDeviceFileHostToFPGA(aUri.mHostname.substr(0, aUri.mHostname.find(",")), O_RDWR ),
  mDeviceFileFPGAToHost(aUri.mHostname.substr(aUri.mHostname.find(",")+1), O_RDWR | O_NONBLOCK  /* for read might need O_RDWR | O_NONBLOCK */),
  mDeviceFileFPGAEvent("", O_RDONLY),
  mIPCMutex(getSharedMemName(mDeviceFileHostToFPGA.getPath())),
  mXdma7seriesWorkaround(false),
  mUseInterrupt(false),
  mNumberOfPages(0),
  mMaxInFlight(0),
  mPageSize(0),
  mMaxPacketSize(0),
  mIndexNextPage(0),
  mPublishedReplyPageCount(0),
  mReadReplyPageCount(0)
{
  if ( aUri.mHostname.find(",") == std::string::npos ) {
    exception::PCIe4InitialisationError lExc;
    log(lExc, "No comma found in hostname of PCIe4 client URI '" + uri() + "'; cannot construct 2 paths for device files");
    throw lExc;
  }
  else if ( aUri.mHostname.find(",") == 0 || aUri.mHostname.find(",") == aUri.mHostname.size()-1) {
    exception::PCIe4InitialisationError lExc;
    log(lExc, "Hostname of PCIe4 client URI '" + uri() + "' starts/ends with a comma; cannot construct 2 paths for device files");
    throw lExc;
  }

  mSleepDuration = std::chrono::microseconds(mUseInterrupt ? 0 : 50);

  for (const auto& lArg: aUri.mArguments) {
    if (lArg.first == "events") {
      if (mUseInterrupt) {
        exception::PCIe4InitialisationError lExc;
        log(lExc, "PCIe4 client URI ", Quote(uri()), ": 'events' attribute is specified multiple times");
        throw lExc;
      }

      mUseInterrupt = true;
      mDeviceFileFPGAEvent.setPath(lArg.second);
      log (Info() , "PCIe4 client with URI ", Quote (uri()), " is configured to use interrupts");
    }
    else if (lArg.first == "sleep") {
      mSleepDuration = std::chrono::microseconds(boost::lexical_cast<size_t>(lArg.second));
      log (Notice() , "PCIe4 client with URI ", Quote (uri()), " : Inter-poll-/-interrupt sleep duration set to ", boost::lexical_cast<size_t>(lArg.second), " us by URI 'sleep' attribute");
    }
    else if (lArg.first == "max_in_flight") {
      mMaxInFlight = boost::lexical_cast<size_t>(lArg.second);
      log (Notice() , "PCIe4 client with URI ", Quote (uri()), " : 'Maximum number of packets in flight' set to ", boost::lexical_cast<size_t>(lArg.second), " by URI 'max_in_flight' attribute");
    }
    else if (lArg.first == "max_packet_size") {
      mMaxPacketSize = boost::lexical_cast<size_t>(lArg.second);
      log (Notice() , "PCIe4 client with URI ", Quote (uri()), " : 'Maximum packet size (in 32-bit words) set to ", boost::lexical_cast<size_t>(lArg.second), " by URI 'max_packet_size' attribute");
    }
    else if (lArg.first == "xdma_7series_workaround") {
      mXdma7seriesWorkaround = true;
      log (Notice() , "PCIe4 client with URI ", Quote (uri()), " : Adjusting size of PCIe4 reads to a few fixed sizes as workaround for 7-series xdma firmware bug");
    }
    else
      log (Warning() , "Unknown attribute ", Quote (lArg.first), " used in URI ", Quote(uri()));
  }
}


PCIe4::~PCIe4()
{
  disconnect();
}


void PCIe4::implementDispatch ( std::shared_ptr< Buffers > aBuffers )
{
  log(Debug(), "PCIe4 client (URI: ", Quote(uri()), ") : implementDispatch method called");

  if ( ! mConnected )
    connect();

  if ( mReplyQueue.size() == mMaxInFlight )
    read();
  write(aBuffers);
}


void PCIe4::Flush( )
{
  log(Debug(), "PCIe4 client (URI: ", Quote(uri()), ") : Flush method called");
  while ( !mReplyQueue.empty() )
    read();

  mDeviceFileHostToFPGA.unlock();

  IPCScopedLock_t lLockGuard(*mIPCMutex);
  mIPCMutex->endSession();
}


void PCIe4::dispatchExceptionHandler()
{
  log(Notice(), "PCIe4 client ", Quote(id()), " (URI: ", Quote(uri()), ") : closing device files since exception detected");

  ClientInterface::returnBufferToPool ( mReplyQueue );

  mDeviceFileHostToFPGA.unlock();

  disconnect();

  InnerProtocol::dispatchExceptionHandler();
}


uint32_t PCIe4::getMaxSendSize()
{
  if ( ! mConnected )
    connect();

  return mMaxPacketSize * 4;
}


uint32_t PCIe4::getMaxReplySize()
{
  if ( ! mConnected )
    connect();

  return mMaxPacketSize * 4;
}


void PCIe4::connect()
{
  IPCScopedLock_t lLockGuard(*mIPCMutex);
  connect(lLockGuard);
}


void PCIe4::connect(IPCScopedLock_t& aGuard)
{
  // Read current value of session counter when reading status info from FPGA
  // (So that can check whether this info is up-to-date later on, when sending next request packet)
  mIPCExternalSessionActive = mIPCMutex->isActive() and (not mDeviceFileHostToFPGA.haveLock());
  mIPCSessionCount = mIPCMutex->getCounter();

  log ( Debug() , "PCIe4 client is opening device file " , Quote ( mDeviceFileFPGAToHost.getPath() ) , " (device-to-client)" );
  mDeviceFileFPGAToHost.open();

  std::vector<uint32_t> lValues;
  mDeviceFileFPGAToHost.read(0x0, 4, lValues);
  aGuard.unlock();
  log ( Debug(), "Read status info (", Integer(lValues.at(0)), ", ", Integer(lValues.at(1)), ", ", Integer(lValues.at(2)), ", ", Integer(lValues.at(3)), "): ", PacketFmt((const uint8_t*)lValues.data(), 4 * lValues.size()));

  mNumberOfPages = lValues.at(0);
  if ( (mMaxInFlight == 0) or (mMaxInFlight > mNumberOfPages) )
    mMaxInFlight = mNumberOfPages;
  mPageSize = lValues.at(1);
  if ( (mMaxPacketSize == 0) or (mMaxPacketSize >= mPageSize) )
    mMaxPacketSize = mPageSize - 1;
  mIndexNextPage = lValues.at(2);
  mPublishedReplyPageCount = lValues.at(3);
  mReadReplyPageCount = mPublishedReplyPageCount;

  if (lValues.at(1) > 0xFFFF) {
    exception::PCIe4InitialisationError lExc;
    log (lExc, "Invalid page size, ", Integer(lValues.at(1)), ", reported in device file ", Quote(mDeviceFileFPGAToHost.getPath()));
    throw lExc;
  }

  if (mIndexNextPage >= mNumberOfPages) {
    exception::PCIe4InitialisationError lExc;
    log (lExc, "Next page index, ", Integer(mIndexNextPage), ", reported in device file ", Quote(mDeviceFileFPGAToHost.getPath()), " is inconsistent with number of pages, ", Integer(mNumberOfPages));
    throw lExc;
  }

  log ( Debug() , "PCIe4 client is opening device file " , Quote ( mDeviceFileHostToFPGA.getPath() ) , " (client-to-device)" );
  mDeviceFileHostToFPGA.open();

  mDeviceFileFPGAToHost.createBuffer(4 * mPageSize);
  mDeviceFileHostToFPGA.createBuffer(4 * mPageSize);

  if (mUseInterrupt)
    mDeviceFileFPGAEvent.open();

  mConnected = true;
  log ( Info() , "PCIe4 client connected to device at ", Quote(mDeviceFileHostToFPGA.getPath()), ", ", Quote(mDeviceFileFPGAToHost.getPath()), "; FPGA has ", Integer(mNumberOfPages), " pages, each of size ", Integer(mPageSize), " words, index ", Integer(mIndexNextPage), " should be filled next" );
}


void PCIe4::disconnect()
{
  mDeviceFileHostToFPGA.close();
  mDeviceFileFPGAToHost.close();
  mDeviceFileFPGAEvent.close();
  mConnected = false;
}


void PCIe4::write(const std::shared_ptr<Buffers>& aBuffers)
{
  if (not mDeviceFileHostToFPGA.haveLock()) {
    mDeviceFileHostToFPGA.lock();

    IPCScopedLock_t lGuard(*mIPCMutex);
    mIPCMutex->startSession();
    mIPCSessionCount++;

    // If these two numbers don't match, another client/process has sent packets
    // more recently than this client has, so must re-read status info
    if (mIPCExternalSessionActive or (mIPCMutex->getCounter() != mIPCSessionCount)) {
      connect(lGuard);
    }
  }

  log (Info(), "PCIe4 client ", Quote(id()), " (URI: ", Quote(uri()), ") : writing ", Integer(aBuffers->sendCounter() / 4), "-word packet to page ", Integer(mIndexNextPage), " in ", Quote(mDeviceFileHostToFPGA.getPath()));

  const uint32_t lHeaderWord = (0x10000 | (((aBuffers->sendCounter() / 4) - 1) & 0xFFFF));
  std::vector<std::pair<const uint8_t*, size_t> > lDataToWrite;
  lDataToWrite.push_back( std::make_pair(reinterpret_cast<const uint8_t*>(&lHeaderWord), sizeof lHeaderWord) );
  lDataToWrite.push_back( std::make_pair(aBuffers->getSendBuffer(), aBuffers->sendCounter()) );

  IPCScopedLock_t lGuard(*mIPCMutex);
  mDeviceFileHostToFPGA.write(mIndexNextPage * 4 * mPageSize, lDataToWrite);
  log (Debug(), "Wrote " , Integer((aBuffers->sendCounter() / 4) + 1), " 32-bit words at address " , Integer(mIndexNextPage * 4 * mPageSize), " ... ", PacketFmt(lDataToWrite));

  mIndexNextPage = (mIndexNextPage + 1) % mNumberOfPages;
  mReplyQueue.push_back(aBuffers);
}


void PCIe4::read()
{
  const size_t lPageIndexToRead = (mIndexNextPage - mReplyQueue.size() + mNumberOfPages) % mNumberOfPages;
  SteadyClock_t::time_point lStartTime = SteadyClock_t::now();

  if (mReadReplyPageCount == mPublishedReplyPageCount)
  {
    if (mUseInterrupt)
    {
      std::vector<uint32_t> lRxEvent;
      // wait for interrupt; read events file node to see if user interrupt has come
      while (true) {
        mDeviceFileFPGAEvent.read(0, 1, lRxEvent);
        if (lRxEvent.at(0) == 1) {
          break;
        }
        lRxEvent.clear();

        if (SteadyClock_t::now() - lStartTime > std::chrono::microseconds(getBoostTimeoutPeriod().total_microseconds())) {
          exception::PCIe4Timeout lExc;
          log(lExc, "Next page (index ", Integer(lPageIndexToRead), " count ", Integer(mPublishedReplyPageCount+1), ") of PCIe4 device '" + mDeviceFileHostToFPGA.getPath() + "' is not ready after timeout period");
          throw lExc;
        }

        log(Debug(), "PCIe4 client ", Quote(id()), " (URI: ", Quote(uri()), ") : Waiting for interrupt; sleeping for ", mSleepDuration.count(), "us");
        if (mSleepDuration > std::chrono::microseconds(0))
          std::this_thread::sleep_for( mSleepDuration );

      } // end of while (true)

      log(Info(), "PCIe4 client ", Quote(id()), " (URI: ", Quote(uri()), ") : Reading page ", Integer(lPageIndexToRead), " (interrupt received)");
    }
    else
    {
      uint32_t lHwPublishedPageCount = 0x0;

      std::vector<uint32_t> lValues;
      while ( true ) {
        // FIXME : Improve by simply adding fileWrite method that takes uint32_t ref as argument (or returns uint32_t)
        IPCScopedLock_t lGuard(*mIPCMutex);
        mDeviceFileFPGAToHost.read(0, (mXdma7seriesWorkaround ? 8 : 4), lValues);
        lHwPublishedPageCount = lValues.at(3);
        log (Debug(), "Read status info from addr 0 (", Integer(lValues.at(0)), ", ", Integer(lValues.at(1)), ", ", Integer(lValues.at(2)), ", ", Integer(lValues.at(3)), "): ", PacketFmt((const uint8_t*)lValues.data(), 4 * lValues.size()));

        if (lHwPublishedPageCount != mPublishedReplyPageCount) {
          mPublishedReplyPageCount = lHwPublishedPageCount;
          break;
        }
        // FIXME: Throw if published page count is invalid number

        if (SteadyClock_t::now() - lStartTime > std::chrono::microseconds(getBoostTimeoutPeriod().total_microseconds())) {
          exception::PCIe4Timeout lExc;
          log(lExc, "Next page (index ", Integer(lPageIndexToRead), " count ", Integer(mPublishedReplyPageCount+1), ") of PCIe4 device '" + mDeviceFileHostToFPGA.getPath() + "' is not ready after timeout period");
          throw lExc;
        }

        log(Debug(), "PCIe4 client ", Quote(id()), " (URI: ", Quote(uri()), ") : Trying to read page index ", Integer(lPageIndexToRead), " = count ", Integer(mReadReplyPageCount+1), "; published page count is ", Integer(lHwPublishedPageCount), "; sleeping for ", mSleepDuration.count(), "us");
        if (mSleepDuration > std::chrono::microseconds(0))
          std::this_thread::sleep_for( mSleepDuration );
        lValues.clear();
      }

      log(Info(), "PCIe4 client ", Quote(id()), " (URI: ", Quote(uri()), ") : Reading page ", Integer(lPageIndexToRead), " (published count ", Integer(lHwPublishedPageCount), ", surpasses required, ", Integer(mReadReplyPageCount + 1), ")");
    }
  }
  mReadReplyPageCount++;
  
  // PART 1 : Read the page
  std::shared_ptr<Buffers> lBuffers = mReplyQueue.front();
  mReplyQueue.pop_front();

  uint32_t lNrWordsToRead(lBuffers->replyCounter() >> 2);
  if(mXdma7seriesWorkaround and (lNrWordsToRead % 32 == 0 || lNrWordsToRead % 32 == 28 || lNrWordsToRead < 4))
    lNrWordsToRead += 4;
  lNrWordsToRead += 1;
 
  std::vector<uint32_t> lPageContents;
  IPCScopedLock_t lGuard(*mIPCMutex);
  mDeviceFileFPGAToHost.read(4 + lPageIndexToRead * mPageSize, lNrWordsToRead , lPageContents);
  lGuard.unlock();
  log (Debug(), "Read " , Integer(lNrWordsToRead), " 32-bit words from address " , Integer(4 + lPageIndexToRead * 4 * mPageSize), " ... ", PacketFmt((const uint8_t*)lPageContents.data(), 4 * lPageContents.size()));

  // PART 2 : Transfer to reply buffer
  const std::deque< std::pair< uint8_t* , uint32_t > >& lReplyBuffers ( lBuffers->getReplyBuffer() );
  size_t lNrWordsInPacket = (lPageContents.at(0) >> 16) + (lPageContents.at(0) & 0xFFFF);
  if (lNrWordsInPacket != (lBuffers->replyCounter() >> 2))
    log (Warning(), "Expected reply packet to contain ", Integer(lBuffers->replyCounter() >> 2), " words, but it actually contains ", Integer(lNrWordsInPacket), " words");

  size_t lNrBytesCopied = 0;
  for (const auto& lBuffer: lReplyBuffers)
  {
    // Don't copy more of page than was written to, for cases when less data received than expected
    if ( lNrBytesCopied >= 4*lNrWordsInPacket)
      break;

    size_t lNrBytesToCopy = std::min( lBuffer.second , uint32_t(4*lNrWordsInPacket - lNrBytesCopied) );
    memcpy ( lBuffer.first, &lPageContents.at(1 + (lNrBytesCopied / 4)), lNrBytesToCopy );
    lNrBytesCopied += lNrBytesToCopy;
  }


  // PART 3 : Validate the packet contents
  uhal::exception::exception* lExc = NULL;
  try
  {
    lExc = ClientInterface::validate ( lBuffers );
  }
  catch ( exception::exception& aExc )
  {
    exception::ValidationError lExc2;
    log ( lExc2 , "Exception caught during reply validation for PCIe4 device with URI " , Quote ( this->uri() ) , "; what returned: " , Quote ( aExc.what() ) );
    throw lExc2;
  }

  if (lExc != NULL)
    lExc->throwAsDerivedType();
}


} // end ns uhal
