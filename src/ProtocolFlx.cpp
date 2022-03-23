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
	@author Alessandro Thea
	@date September 2019
*/

#include "uhallibs/ProtocolFlx.hpp"


#include <algorithm>                                        // for min
#include <assert.h>
#include <cstdlib>
#include <fcntl.h>
#include <iomanip>                                          // for operator<<
#include <iostream>                                         // for operator<<
#include <sys/mman.h>
#include <sys/stat.h>
#include <stdlib.h>                                         // for size_t, free
#include <string.h>                                         // for memcpy
#include <unistd.h>

#include <chrono>                                           // for operator>
#include <thread>                                           // for sleep_for
#include <filesystem>

#include "uhal/grammars/URI.hpp"                            // for URI
#include "uhal/log/LogLevels.hpp"                           // for BaseLogLevel
#include "uhal/log/log_inserters.integer.hpp"               // for uhal::Integer
#include "uhal/log/log_inserters.quote.hpp"                 // for uhal::Quote
#include "uhal/log/log.hpp"
#include "uhal/Buffers.hpp"
#include "uhal/ClientFactory.hpp"

#include "uhallibs/formatters.hpp"

#include "regmap/regmap-struct.h"

UHAL_REGISTER_EXTERNAL_CLIENT(uhallibs::Flx, "ipbusflx-2.0", "A client description")


namespace uhallibs {


//-----------------------------------------------------------------------------
regmap_register_t* Flx::Card::find_reg( const std::string& aName ) {

  regmap_register_t *reg;
  for (reg = regmap_registers; reg->name != NULL; reg++) {
    if (aName == reg->name) {
      return reg;
    }
  }

  return nullptr;
}

//-----------------------------------------------------------------------------
Flx::Card::Card(const std::string& aPath, int aEndpoint, u_int aLockMask) :
  mPath(aPath),
  mEndpoint(aEndpoint),
  mLockMask(aLockMask),
  mFlxCard(),
  mIsOpen(false) {

    // Can be made static
    // mWriteAddress = find_reg("IPBUS_WRITE_ADDRESS")->address;
    // mWriteData = find_reg("IPBUS_WRITE_DATA")->address;
    // mReadAddress = find_reg("IPBUS_READ_ADDRESS")->address;
    // mReadData = find_reg("IPBUS_READ_DATA")->address;
    // mWriteAddress = 0xC800;
    // mWriteData = 0xC810;
    // mReadAddress = 0xC820;
    // mReadData = 0xC830;

}


//-----------------------------------------------------------------------------
Flx::Card::~Card() {
}


//-----------------------------------------------------------------------------
void Flx::Card::open() {
  
  log(uhal::Debug(), "Flx::Card client Opening felix endpoint ", mEndpoint, " on ", mPath);

  if( access( mPath.c_str(), F_OK ) == -1 ) {

    exception::FlxInitialisationError lExc;
    log(lExc, "Failed to open device file ", uhal::Quote(mPath), "; errno=", uhal::Integer(errno), ", meaning ", uhal::Quote (strerror(errno)));
    throw lExc;
  }

  mFlxCard.card_open(mEndpoint, mLockMask);

  mIsOpen = true;
}


//-----------------------------------------------------------------------------
void Flx::Card::close() {

  if (mIsOpen)
    mFlxCard.card_close();
}


//-----------------------------------------------------------------------------
const std::string& Flx::Card::getPath() const {
  return mPath;
}

//-----------------------------------------------------------------------------
int Flx::Card::getEndpoint() const {
  return mEndpoint;
}

//-----------------------------------------------------------------------------
void Flx::Card::read(const uint32_t aAddr, const uint32_t aNrWords, std::vector<uint32_t>& aValues) {

  if (!mIsOpen)
    open();

  // BBB uint64_t lBaseAddr = mFlxCard.openBackDoor(2);
  flxcard_bar2_regs_t *bar2 = (flxcard_bar2_regs_t *) mFlxCard.openBackDoor( 2 );
  
  //BBB uint64_t *lReadAddrPtr = (uint64_t *)(lBaseAddr + mReadAddress);
  //BBB uint64_t *lReadDataPtr = (uint64_t *)(lBaseAddr + mReadData);

  // +1 is ceiling rounding in integers
  uint32_t lNrReads64b = (aNrWords+1)/2;
  uint32_t lAddr = aAddr/2;

  for ( uint32_t i(0); i<lNrReads64b; i++) {
      // *lReadAddrPtr = lAddr+i;
      bar2->IPBUS_READ_ADDRESS = lAddr+i;
      // uint64_t lDataWord = *lReadDataPtr;
      uint64_t lDataWord = bar2->IPBUS_READ_DATA;
      // Split the 64b word in 32b chunks
      aValues.push_back(lDataWord & 0xffffffff);
      if ( 2*i+1 < aNrWords ) 
        aValues.push_back(lDataWord >> 32);
  }

  log(uhal::Debug(), "Flx::Card::read, ", aNrWords, " requested, ", aValues.size(), " read");

}


//-----------------------------------------------------------------------------
void Flx::Card::write(const uint32_t aAddr, const std::vector<std::pair<const uint8_t*, size_t> >& aData)
{

  if (!mIsOpen)
    open();

  flxcard_bar2_regs_t *bar2 = (flxcard_bar2_regs_t *) mFlxCard.openBackDoor( 2 );

  size_t lNrBytes = 0;
  for (size_t i = 0; i < aData.size(); i++)
    lNrBytes += aData.at(i).second;

  assert((lNrBytes % 4) == 0);

  char *allocated = NULL;
  posix_memalign((void **)&allocated, 4096/*alignment*/, lNrBytes + 4096);
  if (allocated == NULL) {
    exception::FlxCommunicationError lExc;
    log(lExc, "Failed to allocate ", uhal::Integer(lNrBytes + 4096), " bytes in File::write/2 function");
    throw lExc;
  }

  // data to write to register address
  char* buffer = allocated;
  size_t lNrBytesCopied = 0;
  for (size_t i = 0; i < aData.size(); i++) {
    memcpy(buffer + lNrBytesCopied, aData.at(i).first, aData.at(i).second);
    lNrBytesCopied += aData.at(i).second;
  }

  lNrBytesCopied = 0;
  uint32_t lAddr = aAddr/2;

  while (lNrBytesCopied < lNrBytes) {
    bar2->IPBUS_WRITE_ADDRESS = lAddr;
    char* lSrcPtr = buffer + lNrBytesCopied;
    if ((lNrBytes - lNrBytesCopied) >= 8) {
      bar2->IPBUS_WRITE_DATA.DATA = *(uint64_t*) lSrcPtr;
      lNrBytesCopied += 8;
    }
    else if ((lNrBytes - lNrBytesCopied) >= 4) {
      bar2->IPBUS_WRITE_DATA.DATA = uint64_t(*(uint32_t*) lSrcPtr);
      lNrBytesCopied += 4;
    }

    ++lAddr;
  }

  free(allocated);
}




Flx::Flx ( const std::string& aId, const uhal::URI& aUri ) :
  IPbus< 2 , 0 > ( aId , aUri ),
  mConnected(false),
  mDeviceFile(aUri.mHostname, std::stoul(aUri.mPort), LOCK_NONE),
  mNumberOfPages(0),
  mPageSize(0),
  mIndexNextPage(0),
  mPublishedReplyPageCount(0),
  mReadReplyPageCount(0),
  mAsynchronousException ( NULL )
{
  mSleepDuration = std::chrono::microseconds(50);

  for (uhal::NameValuePairVectorType::const_iterator lIt = aUri.mArguments.begin(); lIt != aUri.mArguments.end(); lIt++) {
    if (lIt->first == "sleep") {
      mSleepDuration = std::chrono::microseconds(std::stoul(lIt->second));
      log (uhal::Notice() , "flx client with URI ", uhal::Quote (uri()), " : Inter-poll-/-interrupt sleep duration set to ", std::stoul(lIt->second), " us by URI 'sleep' attribute");
    }
    // else if (lIt->first == "offset") {
    //   const bool lIsHex = (lIt->second.find("0x") == 0) or (lIt->second.find("0X") == 0);
    //   const size_t lOffset = (lIsHex ? std::lexical_cast<HexTo<size_t> >(lIt->second) : std::stoul(lIt->second));
    //   mDeviceFile.setOffset(lOffset);
    //   log (uhal::Notice(), "flx client with URI ", uhal::Quote (uri()), " : Address offset set to ", uhal::Integer(lOffset, IntFmt<hex>()));
    // }
    else {
      log (uhal::Warning() , "Unknown attribute ", uhal::Quote (lIt->first), " used in URI ", uhal::Quote(uri()));
    }
  }
}


Flx::~Flx()
{
  disconnect();
}


void Flx::implementDispatch ( std::shared_ptr< uhal::Buffers > aBuffers )
{
  log(uhal::Debug(), "flx client (URI: ", uhal::Quote(uri()), ") : implementDispatch method called");

  if ( ! mConnected )
    connect();

  if ( mReplyQueue.size() == mNumberOfPages )
    read();
  write(aBuffers);
}


void Flx::Flush( )
{
  log(uhal::Debug(), "flx client (URI: ", uhal::Quote(uri()), ") : Flush method called");
  while ( !mReplyQueue.empty() )
    read();

}


void Flx::dispatchExceptionHandler()
{
  // FIXME: Adapt to PCIe implementation
  // log(uhal::Notice(), "flx client ", uhal::Quote(id()), " (URI: ", uhal::Quote(uri()), ") : closing device files since exception detected");

  // ClientInterface::returnBufferToPool ( mReplyQueue );
  // disconnect();

  InnerProtocol::dispatchExceptionHandler();
}


uint32_t Flx::getMaxSendSize()
{
  if ( ! mConnected )
    connect();

  return (mPageSize - 1) * 4;
}


uint32_t Flx::getMaxReplySize()
{
  if ( ! mConnected )
    connect();

  return (mPageSize - 1) * 4;
}


void Flx::connect()
{
  log ( uhal::Debug() , "flx client is opening device file " , uhal::Quote ( mDeviceFile.getPath() ) );
  std::vector<uint32_t> lValues;
  mDeviceFile.read(0x0, 4, lValues);
  log (uhal::Debug(), "Read status info from addr 0 (", uhal::Integer(lValues.at(0)), ", ", uhal::Integer(lValues.at(1)), ", ", uhal::Integer(lValues.at(2)), ", ", uhal::Integer(lValues.at(3)), "): ", PacketFmt((const uint8_t*)lValues.data(), 4 * lValues.size()));

  mNumberOfPages = lValues.at(0);
  // mPageSize = std::min(uint32_t(4096), lValues.at(1));
  mPageSize = lValues.at(1);
  mIndexNextPage = lValues.at(2);
  mPublishedReplyPageCount = lValues.at(3);
  mReadReplyPageCount = mPublishedReplyPageCount;

  if (lValues.at(1) > 0xFFFF) {
    exception::FlxInitialisationError lExc;
    log (lExc, "Invalid page size, ", uhal::Integer(lValues.at(1)), ", reported in device file ", uhal::Quote(mDeviceFile.getPath()));
    throw lExc;
  }

  if (mIndexNextPage >= mNumberOfPages) {
    exception::FlxInitialisationError lExc;
    log (lExc, "Next page index, ", uhal::Integer(mIndexNextPage), ", reported in device file ", uhal::Quote(mDeviceFile.getPath()), " is inconsistent with number of pages, ", uhal::Integer(mNumberOfPages));
    throw lExc;
  }

  mConnected = true;
  log ( uhal::Info() , "flx client connected to device at ", uhal::Quote(mDeviceFile.getPath()), "; FPGA has ", uhal::Integer(mNumberOfPages), " pages, each of size ", uhal::Integer(mPageSize), " words, index ", uhal::Integer(mIndexNextPage), " should be filled next" );
}


void Flx::disconnect()
{
  mDeviceFile.close();
  mConnected = false;
}


void Flx::write(const std::shared_ptr<uhal::Buffers>& aBuffers)
{
  log (uhal::Info(), "flx client ", uhal::Quote(id()), " (URI: ", uhal::Quote(uri()), ") : writing ", uhal::Integer(aBuffers->sendCounter() / 4), "-word packet to page ", uhal::Integer(mIndexNextPage), " in ", uhal::Quote(mDeviceFile.getPath()));

  const uint32_t lHeaderWord = (0x10000 | (((aBuffers->sendCounter() / 4) - 1) & 0xFFFF));
  std::vector<std::pair<const uint8_t*, size_t> > lDataToWrite;
  lDataToWrite.push_back( std::make_pair(reinterpret_cast<const uint8_t*>(&lHeaderWord), sizeof lHeaderWord) );
  lDataToWrite.push_back( std::make_pair(aBuffers->getSendBuffer(), aBuffers->sendCounter()) );
  // mDeviceFile.write(mIndexNextPage * 4 * mPageSize, lDataToWrite);
  mDeviceFile.write(mIndexNextPage * mPageSize, lDataToWrite);

  log (uhal::Debug(), "Wrote " , uhal::Integer((aBuffers->sendCounter() / 4) + 1), " 32-bit words at address " , uhal::Integer(mIndexNextPage * 4 * mPageSize), " ... ", PacketFmt(lDataToWrite));

  mIndexNextPage = (mIndexNextPage + 1) % mNumberOfPages;
  mReplyQueue.push_back(aBuffers);
}


void Flx::read()
{
  const size_t lPageIndexToRead = (mIndexNextPage - mReplyQueue.size() + mNumberOfPages) % mNumberOfPages;
  SteadyClock_t::time_point lStartTime = SteadyClock_t::now();

  if (mReadReplyPageCount == mPublishedReplyPageCount)
  {
    uint32_t lHwPublishedPageCount = 0x0;

    while ( true ) {
      std::vector<uint32_t> lValues;
      // FIXME : Improve by simply adding dmaWrite method that takes uint32_t ref as argument (or returns uint32_t)
      mDeviceFile.read(0, 4, lValues);
      lHwPublishedPageCount = lValues.at(3);
      log (uhal::Debug(), "Read status info from addr 0 (", uhal::Integer(lValues.at(0)), ", ", uhal::Integer(lValues.at(1)), ", ", uhal::Integer(lValues.at(2)), ", ", uhal::Integer(lValues.at(3)), "): ", PacketFmt((const uint8_t*)lValues.data(), 4 * lValues.size()));

      if (lHwPublishedPageCount != mPublishedReplyPageCount) {
        mPublishedReplyPageCount = lHwPublishedPageCount;
        break;
      }
      // FIXME: Throw if published page count is invalid number

      if (SteadyClock_t::now() - lStartTime > std::chrono::microseconds(getBoostTimeoutPeriod().total_microseconds())) {
        exception::FlxTimeout lExc;
        log(lExc, "Next page (index ", uhal::Integer(lPageIndexToRead), " count ", uhal::Integer(mPublishedReplyPageCount+1), ") of flx device '" + mDeviceFile.getPath() + "' is not ready after timeout period");
        throw lExc;
      }

      log(uhal::Debug(), "flx client ", uhal::Quote(id()), " (URI: ", uhal::Quote(uri()), ") : Trying to read page index ", uhal::Integer(lPageIndexToRead), " = count ", uhal::Integer(mReadReplyPageCount+1), "; published page count is ", uhal::Integer(lHwPublishedPageCount), "; sleeping for ", mSleepDuration.count(), "us");
      if (mSleepDuration > std::chrono::microseconds(0))
        std::this_thread::sleep_for( mSleepDuration );
    }

    log(uhal::Info(), "flx client ", uhal::Quote(id()), " (URI: ", uhal::Quote(uri()), ") : Reading page ", uhal::Integer(lPageIndexToRead), " (published count ", uhal::Integer(lHwPublishedPageCount), ", surpasses required, ", uhal::Integer(mReadReplyPageCount + 1), ")");
  }
  mReadReplyPageCount++;
  
  // PART 1 : Read the page
  std::shared_ptr<uhal::Buffers> lBuffers = mReplyQueue.front();
  mReplyQueue.pop_front();

  uint32_t lNrWordsToRead(lBuffers->replyCounter() >> 2);
  lNrWordsToRead += 1;
 
  std::vector<uint32_t> lPageContents;
  mDeviceFile.read(4 + lPageIndexToRead * mPageSize, lNrWordsToRead , lPageContents);
  log (uhal::Debug(), "Read " , uhal::Integer(lNrWordsToRead), " 32-bit words from address " , uhal::Integer(4 + lPageIndexToRead * 4 * mPageSize), " ... ", PacketFmt((const uint8_t*)lPageContents.data(), 4 * lPageContents.size()));

  // PART 2 : Transfer to reply buffer
  const std::deque< std::pair< uint8_t* , uint32_t > >& lReplyBuffers ( lBuffers->getReplyBuffer() );
  size_t lNrWordsInPacket = (lPageContents.at(0) >> 16) + (lPageContents.at(0) & 0xFFFF);
  if (lNrWordsInPacket != (lBuffers->replyCounter() >> 2))
    log (uhal::Warning(), "Expected reply packet to contain ", uhal::Integer(lBuffers->replyCounter() >> 2), " words, but it actually contains ", uhal::Integer(lNrWordsInPacket), " words");

  size_t lNrBytesCopied = 0;
  for ( std::deque< std::pair< uint8_t* , uint32_t > >::const_iterator lIt = lReplyBuffers.begin() ; lIt != lReplyBuffers.end() ; ++lIt )
  {
    // Don't copy more of page than was written to, for cases when less data received than expected
    if ( lNrBytesCopied >= 4*lNrWordsInPacket)
      break;

    size_t lNrBytesToCopy = std::min( lIt->second , uint32_t(4*lNrWordsInPacket - lNrBytesCopied) );
    memcpy ( lIt->first, &lPageContents.at(1 + (lNrBytesCopied / 4)), lNrBytesToCopy );
    lNrBytesCopied += lNrBytesToCopy;
  }


  // PART 3 : Validate the packet contents
  try
  {
    if ( uhal::exception::exception* lExc = ClientInterface::validate ( lBuffers ) ) //Control of the pointer has been passed back to the client interface
    {
      mAsynchronousException = lExc;
    }
  }
  catch ( uhal::exception::exception& aExc )
  {
    mAsynchronousException = new uhal::exception::ValidationError ();
    log ( *mAsynchronousException , "Exception caught during reply validation for flx device with URI " , uhal::Quote ( this->uri() ) , "; what returned: " , uhal::Quote ( aExc.what() ) );
  }

  if ( mAsynchronousException )
  {
    mAsynchronousException->throwAsDerivedType();
  }
}


} // end ns uhal
