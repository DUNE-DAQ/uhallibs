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

#include "uhal/ProtocolFlx.hpp"


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
#include "uhal/log/log_inserters.integer.hpp"               // for Integer
#include "uhal/log/log_inserters.quote.hpp"                 // for Quote
#include "uhal/log/log.hpp"
#include "uhal/Buffers.hpp"
#include "uhal/ClientFactory.hpp"

#include "regmap/regmap-struct.h"


UHAL_REGISTER_EXTERNAL_CLIENT(uhal::Flx, "ipbusflx-2.0", "A client description")


namespace uhal {


//-----------------------------------------------------------------------------
Flx::PacketFmt::PacketFmt(const uint8_t* const aPtr, const size_t aNrBytes) :
  mData(1, std::pair<const uint8_t*, size_t>(aPtr, aNrBytes))
{
}


//-----------------------------------------------------------------------------
Flx::PacketFmt::PacketFmt(const std::vector< std::pair<const uint8_t*, size_t> >& aData) :
  mData(aData)
{}


//-----------------------------------------------------------------------------
Flx::PacketFmt::~PacketFmt()
{}


//-----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& aStream, const Flx::PacketFmt& aPacket)
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


//-----------------------------------------------------------------------------
regmap_register_t* Flx::File::find_reg( const std::string& aName ) {

  regmap_register_t *reg;
  for (reg = regmap_registers; reg->name != NULL; reg++) {
    if (aName == reg->name) {
      return reg;
    }
  }

  return nullptr;
}

//-----------------------------------------------------------------------------
Flx::File::File(const std::string& aPath, int aEndpoint, u_int aLockMask) :
  mPath(aPath),
  mEndpoint(aEndpoint),
  mLockMask(aLockMask),
  mFlxCard(),
  mIsOpen(false) {

}


//-----------------------------------------------------------------------------
Flx::File::~File() {
}


//-----------------------------------------------------------------------------
void Flx::File::open() {
  
  log(Debug(), "Flx::File client Opening felix endpoint ", mEndpoint, " on ", mPath);

  if( access( mPath.c_str(), F_OK ) == -1 ) {

    exception::FlxInitialisationError lExc;
    log(lExc, "Failed to open device file ", Quote(mPath), "; errno=", Integer(errno), ", meaning ", Quote (strerror(errno)));
    throw lExc;
  }

  mFlxCard.card_open(mEndpoint, mLockMask);

  mIsOpen = true;
}


//-----------------------------------------------------------------------------
void Flx::File::close() {

  if (mIsOpen)
    mFlxCard.card_close();
}


//-----------------------------------------------------------------------------
const std::string& Flx::File::getPath() const {
  return mPath;
}

//-----------------------------------------------------------------------------
int Flx::File::getEndpoint() const {
  return mEndpoint;
}

//-----------------------------------------------------------------------------
void Flx::File::read(const uint32_t aAddr, const uint32_t aNrWords, std::vector<uint32_t>& aValues) {

  if (!mIsOpen)
    open();

  // uint64_t lBaseAddr = mFlxCard.openBackDoor(2);
  flxcard_bar2_regs_t *bar2 = (flxcard_bar2_regs_t *) mFlxCard.openBackDoor( 2 );
  
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

  log(Debug(), "Flx::File::read, ", aNrWords, " requested, ", aValues.size(), " read");

}


//-----------------------------------------------------------------------------
void Flx::File::write(const uint32_t aAddr, const std::vector<std::pair<const uint8_t*, size_t> >& aData)
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
    log(lExc, "Failed to allocate ", Integer(lNrBytes + 4096), " bytes in File::write/2 function");
    throw lExc;
  }

  // data to write to register address
  char* buffer = allocated;
  size_t lNrBytesCopied = 0;
  for (size_t i = 0; i < aData.size(); i++) {
    memcpy(buffer + lNrBytesCopied, aData.at(i).first, aData.at(i).second);
    lNrBytesCopied += aData.at(i).second;
  }

//  memcpy(aFlxBaseAddress + aAddr, buffer, lNrBytes);
  lNrBytesCopied = 0;
  uint32_t lAddr = aAddr/2;

  while (lNrBytesCopied < lNrBytes) {
    // *lWriteAddrPtr = lAddr;
    bar2->IPBUS_WRITE_ADDRESS = lAddr;
    char* lSrcPtr = buffer + lNrBytesCopied;
    if ((lNrBytes - lNrBytesCopied) >= 8) {
      // *lWriteDataPtr = *(uint64_t*) lSrcPtr;
      bar2->IPBUS_WRITE_DATA.DATA = *(uint64_t*) lSrcPtr;
      lNrBytesCopied += 8;
    }
    else if ((lNrBytes - lNrBytesCopied) >= 4) {
      // *lWriteDataPtr = uint64_t(*(uint32_t*) lSrcPtr);
      bar2->IPBUS_WRITE_DATA.DATA = uint64_t(*(uint32_t*) lSrcPtr);
      lNrBytesCopied += 4;
    }

    ++lAddr;
  }

  free(allocated);
}




Flx::Flx ( const std::string& aId, const URI& aUri ) :
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

  for (NameValuePairVectorType::const_iterator lIt = aUri.mArguments.begin(); lIt != aUri.mArguments.end(); lIt++) {
    if (lIt->first == "sleep") {
      mSleepDuration = std::chrono::microseconds(std::stoul(lIt->second));
      log (Notice() , "flx client with URI ", Quote (uri()), " : Inter-poll-/-interrupt sleep duration set to ", std::stoul(lIt->second), " us by URI 'sleep' attribute");
    }
    // else if (lIt->first == "offset") {
    //   const bool lIsHex = (lIt->second.find("0x") == 0) or (lIt->second.find("0X") == 0);
    //   const size_t lOffset = (lIsHex ? std::lexical_cast<HexTo<size_t> >(lIt->second) : std::stoul(lIt->second));
    //   mDeviceFile.setOffset(lOffset);
    //   log (Notice(), "flx client with URI ", Quote (uri()), " : Address offset set to ", Integer(lOffset, IntFmt<hex>()));
    // }
    else {
      log (Warning() , "Unknown attribute ", Quote (lIt->first), " used in URI ", Quote(uri()));
    }
  }
}


Flx::~Flx()
{
  disconnect();
}


void Flx::implementDispatch ( std::shared_ptr< Buffers > aBuffers )
{
  log(Debug(), "flx client (URI: ", Quote(uri()), ") : implementDispatch method called");

  if ( ! mConnected )
    connect();

  if ( mReplyQueue.size() == mNumberOfPages )
    read();
  write(aBuffers);
}


void Flx::Flush( )
{
  log(Debug(), "flx client (URI: ", Quote(uri()), ") : Flush method called");
  while ( !mReplyQueue.empty() )
    read();

}


void Flx::dispatchExceptionHandler()
{
  // FIXME: Adapt to PCIe implementation
  // log(Notice(), "flx client ", Quote(id()), " (URI: ", Quote(uri()), ") : closing device files since exception detected");

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
  log ( Debug() , "flx client is opening device file " , Quote ( mDeviceFile.getPath() ) );
  std::vector<uint32_t> lValues;
  mDeviceFile.read(0x0, 4, lValues);
  log (Debug(), "Read status info from addr 0 (", Integer(lValues.at(0)), ", ", Integer(lValues.at(1)), ", ", Integer(lValues.at(2)), ", ", Integer(lValues.at(3)), "): ", PacketFmt((const uint8_t*)lValues.data(), 4 * lValues.size()));

  mNumberOfPages = lValues.at(0);
  // mPageSize = std::min(uint32_t(4096), lValues.at(1));
  mPageSize = lValues.at(1);
  mIndexNextPage = lValues.at(2);
  mPublishedReplyPageCount = lValues.at(3);
  mReadReplyPageCount = mPublishedReplyPageCount;

  if (lValues.at(1) > 0xFFFF) {
    exception::FlxInitialisationError lExc;
    log (lExc, "Invalid page size, ", Integer(lValues.at(1)), ", reported in device file ", Quote(mDeviceFile.getPath()));
    throw lExc;
  }

  if (mIndexNextPage >= mNumberOfPages) {
    exception::FlxInitialisationError lExc;
    log (lExc, "Next page index, ", Integer(mIndexNextPage), ", reported in device file ", Quote(mDeviceFile.getPath()), " is inconsistent with number of pages, ", Integer(mNumberOfPages));
    throw lExc;
  }

  mConnected = true;
  log ( Info() , "flx client connected to device at ", Quote(mDeviceFile.getPath()), "; FPGA has ", Integer(mNumberOfPages), " pages, each of size ", Integer(mPageSize), " words, index ", Integer(mIndexNextPage), " should be filled next" );
}


void Flx::disconnect()
{
  mDeviceFile.close();
  mConnected = false;
}


void Flx::write(const std::shared_ptr<Buffers>& aBuffers)
{
  log (Info(), "flx client ", Quote(id()), " (URI: ", Quote(uri()), ") : writing ", Integer(aBuffers->sendCounter() / 4), "-word packet to page ", Integer(mIndexNextPage), " in ", Quote(mDeviceFile.getPath()));

  const uint32_t lHeaderWord = (0x10000 | (((aBuffers->sendCounter() / 4) - 1) & 0xFFFF));
  std::vector<std::pair<const uint8_t*, size_t> > lDataToWrite;
  lDataToWrite.push_back( std::make_pair(reinterpret_cast<const uint8_t*>(&lHeaderWord), sizeof lHeaderWord) );
  lDataToWrite.push_back( std::make_pair(aBuffers->getSendBuffer(), aBuffers->sendCounter()) );
  // mDeviceFile.write(mIndexNextPage * 4 * mPageSize, lDataToWrite);
  mDeviceFile.write(mIndexNextPage * mPageSize, lDataToWrite);

  log (Debug(), "Wrote " , Integer((aBuffers->sendCounter() / 4) + 1), " 32-bit words at address " , Integer(mIndexNextPage * 4 * mPageSize), " ... ", PacketFmt(lDataToWrite));

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
      log (Debug(), "Read status info from addr 0 (", Integer(lValues.at(0)), ", ", Integer(lValues.at(1)), ", ", Integer(lValues.at(2)), ", ", Integer(lValues.at(3)), "): ", PacketFmt((const uint8_t*)lValues.data(), 4 * lValues.size()));

      if (lHwPublishedPageCount != mPublishedReplyPageCount) {
        mPublishedReplyPageCount = lHwPublishedPageCount;
        break;
      }
      // FIXME: Throw if published page count is invalid number

      if (SteadyClock_t::now() - lStartTime > std::chrono::microseconds(getBoostTimeoutPeriod().total_microseconds())) {
        exception::FlxTimeout lExc;
        log(lExc, "Next page (index ", Integer(lPageIndexToRead), " count ", Integer(mPublishedReplyPageCount+1), ") of flx device '" + mDeviceFile.getPath() + "' is not ready after timeout period");
        throw lExc;
      }

      log(Debug(), "flx client ", Quote(id()), " (URI: ", Quote(uri()), ") : Trying to read page index ", Integer(lPageIndexToRead), " = count ", Integer(mReadReplyPageCount+1), "; published page count is ", Integer(lHwPublishedPageCount), "; sleeping for ", mSleepDuration.count(), "us");
      if (mSleepDuration > std::chrono::microseconds(0))
        std::this_thread::sleep_for( mSleepDuration );
    }

    log(Info(), "flx client ", Quote(id()), " (URI: ", Quote(uri()), ") : Reading page ", Integer(lPageIndexToRead), " (published count ", Integer(lHwPublishedPageCount), ", surpasses required, ", Integer(mReadReplyPageCount + 1), ")");
  }
  mReadReplyPageCount++;
  
  // PART 1 : Read the page
  std::shared_ptr<Buffers> lBuffers = mReplyQueue.front();
  mReplyQueue.pop_front();

  uint32_t lNrWordsToRead(lBuffers->replyCounter() >> 2);
  lNrWordsToRead += 1;
 
  std::vector<uint32_t> lPageContents;
  mDeviceFile.read(4 + lPageIndexToRead * mPageSize, lNrWordsToRead , lPageContents);
  log (Debug(), "Read " , Integer(lNrWordsToRead), " 32-bit words from address " , Integer(4 + lPageIndexToRead * 4 * mPageSize), " ... ", PacketFmt((const uint8_t*)lPageContents.data(), 4 * lPageContents.size()));

  // PART 2 : Transfer to reply buffer
  const std::deque< std::pair< uint8_t* , uint32_t > >& lReplyBuffers ( lBuffers->getReplyBuffer() );
  size_t lNrWordsInPacket = (lPageContents.at(0) >> 16) + (lPageContents.at(0) & 0xFFFF);
  if (lNrWordsInPacket != (lBuffers->replyCounter() >> 2))
    log (Warning(), "Expected reply packet to contain ", Integer(lBuffers->replyCounter() >> 2), " words, but it actually contains ", Integer(lNrWordsInPacket), " words");

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
  catch ( exception::exception& aExc )
  {
    mAsynchronousException = new exception::ValidationError ();
    log ( *mAsynchronousException , "Exception caught during reply validation for flx device with URI " , Quote ( this->uri() ) , "; what returned: " , Quote ( aExc.what() ) );
  }

  if ( mAsynchronousException )
  {
    mAsynchronousException->throwAsDerivedType();
  }
}


} // end ns uhal
