#include "uhallibs/ProtocolAxi4Lite.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <unistd.h>
#include <thread>

#include "uhallibs/formatters.hpp"

#include "uhal/ClientFactory.hpp"
#include "uhal/Buffers.hpp"

using uhal::Integer;

UHAL_REGISTER_EXTERNAL_CLIENT(uhallibs::Axi4Lite, "ipbusaxi4lite-2.0", "AXI4 Lite IPBus client")
// Syntax: ipbus-qdma-axi4l-2.0:///sys/bus/pci/devices/0000:<bus>:<dev>.<func>/resource<bar#>
// Syntax: ipbusaxi4lite-2.0://<bus>:<dev>.<func>/resource<bar#>
// 
namespace uhallibs {


Axi4Lite::MappedFile::MappedFile(const std::string& aPath, size_t aLength, int aProtFlags)
    : mPath(aPath),
      mFd(-1),
      mBar(nullptr),
      mLength(aLength),
      mProtFlags(aProtFlags),
      mLocked(false),
      mBufferSize(0),
      mBuffer(nullptr) {}

Axi4Lite::MappedFile::~MappedFile() {
  if (mBuffer != nullptr) free(mBuffer);
  close();
}

const std::string& Axi4Lite::MappedFile::getPath() const { return mPath; }

void Axi4Lite::MappedFile::setPath(const std::string& aPath) { mPath = aPath; }
void Axi4Lite::MappedFile::setLength(size_t aLength) { mLength = aLength; }

void Axi4Lite::MappedFile::open() {
  if (mBar != nullptr) return;

  mFd = ::open(mPath.c_str(), (mProtFlags & PROT_WRITE) ? O_RDWR : O_RDONLY);
  if (mFd < 0) {
    return;
  }

  void* lBar = mmap(nullptr, 4*mLength, mProtFlags, MAP_SHARED, mFd, 0);
  mBar = (lBar == MAP_FAILED ? nullptr : (uint32_t*)lBar);
}

void Axi4Lite::MappedFile::close() {
  if (mBar != nullptr) munmap(mBar, mLength);
  mBar = nullptr;

  if (mFd != -1) ::close(mFd);
  mFd = -1;
}

void Axi4Lite::MappedFile::createBuffer(const size_t aNrBytes) {
  if (mBuffer != NULL) {
    if (mBufferSize >= aNrBytes)
      return;
    else {
      free(mBuffer);
      mBuffer = NULL;
      mBufferSize = 0;
    }
  }

  posix_memalign((void**)&mBuffer, 4096 /*alignment*/, aNrBytes + 4096);
  if (mBuffer == NULL) {
    exception::Axi4LiteCommunicationError lExc;
    log(lExc, "Failed to allocate ", Integer(aNrBytes + 4096),
        " bytes in Axi4Lite::MappedFile::createBuffer");
    throw lExc;
  }

  mBufferSize = aNrBytes + 4096;
}

void Axi4Lite::MappedFile::read(const uint32_t aAddr, const uint32_t aNrWords,
                std::vector<uint32_t>& aValues) {
  if (mBar == nullptr) open();

  for (size_t i(0); i < aNrWords; ++i) {
    aValues.push_back(le32toh(mBar[aAddr + i]));
  }
}

void Axi4Lite::MappedFile::write(const uint32_t aAddr, const std::vector<uint32_t>& aValues) {
  write(aAddr, reinterpret_cast<const uint8_t*>(aValues.data()),
        4 * aValues.size());
}

void Axi4Lite::MappedFile::write(const uint32_t aAddr, const uint8_t* const aPtr,
                 const size_t aNrBytes) {
  if (mBar == nullptr) open();

  assert((aNrBytes % 4) == 0);
  uint32_t lNrWordsData = aNrBytes / 4;

  auto lPtr32 = reinterpret_cast<const uint32_t*>(aPtr);
  for (size_t i(0); i < lNrWordsData; ++i) {
    mBar[aAddr + i] = lPtr32[i];
  }
}

void Axi4Lite::MappedFile::write(const uint32_t aAddr,
                 const std::vector<std::pair<const uint8_t*, size_t> >& aData) {
  if (mBar == nullptr) open();

  size_t lNrBytes = 0;
  for (size_t i = 0; i < aData.size(); i++) lNrBytes += aData.at(i).second;

  assert((lNrBytes % 4) == 0);
  size_t lNrWords = lNrBytes/4;

  createBuffer(lNrBytes);

  size_t k(0);
  for (size_t i = 0; i < aData.size(); ++i) {
    for (size_t j = 0; j < aData.at(i).second; ++j) {
      mBuffer[k] = aData.at(i).first[j];
      ++k;
    }
  }

  auto mBuffer32b = reinterpret_cast<const uint32_t*>(mBuffer);

  for (size_t i(0); i<lNrWords; ++i) {
    mBar[aAddr + i] = htole32(mBuffer32b[i]);
  }
  // std::memcpy(mBar + aAddr, mBuffer, lNrBytes);

}

bool Axi4Lite::MappedFile::haveLock() const { return mLocked; }

void Axi4Lite::MappedFile::lock() {
  if (flock(mFd, LOCK_EX) == -1) {
    ipc::exception::MutexError lExc;
    log(lExc, "Failed to lock device file ", uhal::Quote(mPath),
        "; errno=", Integer(errno), ", meaning ", uhal::Quote(strerror(errno)));
    throw lExc;
  }
  mLocked = true;
}

void Axi4Lite::MappedFile::unlock() {
  if (flock(mFd, LOCK_UN) == -1) {
    log(uhal::Warning(), "Failed to unlock device file ", uhal::Quote(mPath),
        "; errno=", Integer(errno), ", meaning ", uhal::Quote(strerror(errno)));
  } else
    mLocked = false;
}

// Axi4Lite Transport
std::string Axi4Lite::getSharedMemName(const std::string& aPath)
{
  std::string lSanitizedPath(aPath);
  std::replace(lSanitizedPath.begin(), lSanitizedPath.end(), '/', ':');

  return "/uhal::ipbusaxi4lite-2.0::" + lSanitizedPath;
}

std::string Axi4Lite::getDevicePath(const uhal::URI& aUri) {

  std::string lPath = aUri.mHostname;

  auto it = std::find_if( aUri.mArguments.begin(), aUri.mArguments.end(),
    [](const std::pair<std::string, std::string>& element){ return element.first == "dev";} );

  if ( it != aUri.mArguments.end() ) {
    lPath += "/"+it->second;
  }

  return lPath;
}

Axi4Lite::Axi4Lite(const std::string& aId, const uhal::URI& aUri)
    : IPbus<2, 0>(aId, aUri),
      mConnected(false),
      mMappedFile(getDevicePath(aUri), 64, PROT_WRITE),
      mIPCMutex(getSharedMemName(mMappedFile.getPath())),
      mNumberOfPages(0),
      mMaxInFlight(0),
      mPageSize(0),
      mMaxPacketSize(0),
      mIndexNextPage(0),
      mPublishedReplyPageCount(0),
      mReadReplyPageCount(0) {

      }

Axi4Lite::~Axi4Lite() {

}



void Axi4Lite::implementDispatch ( std::shared_ptr< uhal::Buffers > aBuffers )
{
  log(uhal::Debug(), "Axi4Lite client (URI: ", uhal::Quote(uri()), ") : implementDispatch method called");

  if ( ! mConnected )
    connect();

  if ( mReplyQueue.size() == mMaxInFlight )
    read();
  write(aBuffers);
}


void Axi4Lite::Flush( )
{
  log(uhal::Debug(), "Axi4Lite client (URI: ", uhal::Quote(uri()), ") : Flush method called");
  while ( !mReplyQueue.empty() )
    read();

  mMappedFile.unlock();

  IPCScopedLock_t lLockGuard(*mIPCMutex);
  mIPCMutex->endSession();
}


void Axi4Lite::dispatchExceptionHandler()
{
  log(uhal::Notice(), "Axi4Lite client ", uhal::Quote(id()), " (URI: ", uhal::Quote(uri()), ") : closing device files since exception detected");

  ClientInterface::returnBufferToPool ( mReplyQueue );

  mMappedFile.unlock();

  disconnect();

  InnerProtocol::dispatchExceptionHandler();
}


uint32_t Axi4Lite::getMaxSendSize()
{
  if ( ! mConnected )
    connect();

  return mMaxPacketSize * 4;
}


uint32_t Axi4Lite::getMaxReplySize()
{
  if ( ! mConnected )
    connect();

  return mMaxPacketSize * 4;
}


void Axi4Lite::connect()
{
  IPCScopedLock_t lLockGuard(*mIPCMutex);
  connect(lLockGuard);
}

void Axi4Lite::connect(IPCScopedLock_t& aGuard)
{
  // Read current value of session counter when reading status info from FPGA
  // (So that can check whether this info is up-to-date later on, when sending next request packet)
  mIPCExternalSessionActive = mIPCMutex->isActive() and (not mMappedFile.haveLock());
  mIPCSessionCount = mIPCMutex->getCounter();

  log ( uhal::Debug() , "Axi4Lite client is opening device file " , uhal::Quote ( mMappedFile.getPath() ) , " (device-to-client)" );

  // Minimal mapping to read the 
  mMappedFile.setLength(0x10);
  mMappedFile.open();
  std::vector<uint32_t> lStats;
  mMappedFile.read(0, 4, lStats);
  mMappedFile.close();
  aGuard.unlock();

  mNumberOfPages = lStats.at(0);
  if ( (mMaxInFlight == 0) or (mMaxInFlight > mNumberOfPages) )
    mMaxInFlight = mNumberOfPages;
  mPageSize = lStats.at(1);
  if ( (mMaxPacketSize == 0) or (mMaxPacketSize >= mPageSize) )
    mMaxPacketSize = mPageSize - 1;
  mIndexNextPage = lStats.at(2);
  mPublishedReplyPageCount = lStats.at(3);
  mReadReplyPageCount = mPublishedReplyPageCount;

  // 
  constexpr uint32_t lSafetyMargin(4096);
  // Set the memory mapping range to a value commensurate to the memory available in firmware
  mMappedFile.setLength(mNumberOfPages*mPageSize+4+lSafetyMargin);
  mMappedFile.open();

  mConnected=true;

  log ( uhal::Info() , "Axi4Lite client connected to device at ", uhal::Quote(mMappedFile.getPath()), ", FPGA has ", Integer(mNumberOfPages), " pages, each of size ", Integer(mPageSize), " words, index ", Integer(mIndexNextPage), " should be filled next" );

}

void Axi4Lite::disconnect()
{
  mMappedFile.close();
  mConnected = false;
}

void Axi4Lite::write(const std::shared_ptr<uhal::Buffers>& aBuffers)
{
  if (not mMappedFile.haveLock()) {
    mMappedFile.lock();

    IPCScopedLock_t lGuard(*mIPCMutex);
    mIPCMutex->startSession();
    mIPCSessionCount++;

    // If these two numbers don't match, another client/process has sent packets
    // more recently than this client has, so must re-read status info
    if (mIPCExternalSessionActive or (mIPCMutex->getCounter() != mIPCSessionCount)) {
      connect(lGuard);
    }
  }

  log (uhal::Info(), "Axi4Lite client ", uhal::Quote(id()), " (URI: ", uhal::Quote(uri()), ") : writing ", Integer(aBuffers->sendCounter() / 4), "-word packet to page ", Integer(mIndexNextPage), " in ", uhal::Quote(mMappedFile.getPath()));

  const uint32_t lHeaderWord = (0x10000 | (((aBuffers->sendCounter() / 4) - 1) & 0xFFFF));
  std::vector<std::pair<const uint8_t*, size_t> > lDataToWrite;
  lDataToWrite.push_back( std::make_pair(reinterpret_cast<const uint8_t*>(&lHeaderWord), sizeof lHeaderWord) );
  lDataToWrite.push_back( std::make_pair(aBuffers->getSendBuffer(), aBuffers->sendCounter()) );

  IPCScopedLock_t lGuard(*mIPCMutex);
  mMappedFile.write(mIndexNextPage * mPageSize, lDataToWrite);
  log (uhal::Debug(), "Wrote " , Integer((aBuffers->sendCounter() / 4) + 1), " 32-bit words at address " , Integer(mIndexNextPage * mPageSize), " ... ", PacketFmt(lDataToWrite));

  mIndexNextPage = (mIndexNextPage + 1) % mNumberOfPages;
  mReplyQueue.push_back(aBuffers);
}


void Axi4Lite::read()
{
  const size_t lPageIndexToRead = (mIndexNextPage - mReplyQueue.size() + mNumberOfPages) % mNumberOfPages;
  SteadyClock_t::time_point lStartTime = SteadyClock_t::now();

  if (mReadReplyPageCount == mPublishedReplyPageCount)
  {
    uint32_t lHwPublishedPageCount = 0x0;

    std::vector<uint32_t> lValues;
    while ( true ) {
      // FIXME : Improve by simply adding fileWrite method that takes uint32_t ref as argument (or returns uint32_t)
      IPCScopedLock_t lGuard(*mIPCMutex);
      mMappedFile.read(0, 4, lValues);
      lHwPublishedPageCount = lValues.at(3);
      log (uhal::Debug(), "Read status info from addr 0 (", Integer(lValues.at(0)), ", ", Integer(lValues.at(1)), ", ", Integer(lValues.at(2)), ", ", Integer(lValues.at(3)), "): ", PacketFmt((const uint8_t*)lValues.data(), 4 * lValues.size()));

      if (lHwPublishedPageCount != mPublishedReplyPageCount) {
        mPublishedReplyPageCount = lHwPublishedPageCount;
        break;
      }
      // FIXME: Throw if published page count is invalid number

      if (SteadyClock_t::now() - lStartTime > std::chrono::microseconds(getBoostTimeoutPeriod().total_microseconds())) {
        exception::Axi4LiteTimeout lExc;
        log(lExc, "Next page (index ", Integer(lPageIndexToRead), " count ", Integer(mPublishedReplyPageCount+1), ") of Axi4Lite device '" + mMappedFile.getPath() + "' is not ready after timeout period");
        throw lExc;
      }

      log(uhal::Debug(), "Axi4Lite client ", uhal::Quote(id()), " (URI: ", uhal::Quote(uri()), ") : Trying to read page index ", Integer(lPageIndexToRead), " = count ", Integer(mReadReplyPageCount+1), "; published page count is ", Integer(lHwPublishedPageCount), "; sleeping for ", mSleepDuration.count(), "us");
      if (mSleepDuration > std::chrono::microseconds(0))
        std::this_thread::sleep_for( mSleepDuration );
      lValues.clear();
    }

    log(uhal::Info(), "Axi4Lite client ", uhal::Quote(id()), " (URI: ", uhal::Quote(uri()), ") : Reading page ", Integer(lPageIndexToRead), " (published count ", Integer(lHwPublishedPageCount), ", surpasses required, ", Integer(mReadReplyPageCount + 1), ")");

  }
  mReadReplyPageCount++;
  
  // PART 1 : Read the page
  std::shared_ptr<uhal::Buffers> lBuffers = mReplyQueue.front();
  mReplyQueue.pop_front();

  uint32_t lNrWordsToRead(lBuffers->replyCounter() >> 2);
  lNrWordsToRead += 1;
 
  std::vector<uint32_t> lPageContents;
  IPCScopedLock_t lGuard(*mIPCMutex);
  mMappedFile.read(4 + lPageIndexToRead * mPageSize, lNrWordsToRead , lPageContents);
  lGuard.unlock();
  log (uhal::Debug(), "Read " , Integer(lNrWordsToRead), " 32-bit words from address " , Integer(4 + lPageIndexToRead * 4 * mPageSize), " ... ", PacketFmt((const uint8_t*)lPageContents.data(), 4 * lPageContents.size()));

  // PART 2 : Transfer to reply buffer
  const std::deque< std::pair< uint8_t* , uint32_t > >& lReplyBuffers ( lBuffers->getReplyBuffer() );
  size_t lNrWordsInPacket = (lPageContents.at(0) >> 16) + (lPageContents.at(0) & 0xFFFF);
  if (lNrWordsInPacket != (lBuffers->replyCounter() >> 2))
    log (uhal::Warning(), "Expected reply packet to contain ", Integer(lBuffers->replyCounter() >> 2), " words, but it actually contains ", Integer(lNrWordsInPacket), " words");

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
  catch ( uhal::exception::exception& aExc )
  {
    uhal::exception::ValidationError lExc2;
    log ( lExc2 , "Exception caught during reply validation for Axi4Lite device with URI " , uhal::Quote ( this->uri() ) , "; what returned: " , uhal::Quote ( aExc.what() ) );
    throw lExc2;
  }

  if (lExc != NULL)
    lExc->throwAsDerivedType();
}

} // namespace uhal
