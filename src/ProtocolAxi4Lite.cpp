#include "uhal/ProtocolAxi4Lite.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <unistd.h>

#include "uhal/ClientFactory.hpp"

// UHAL_REGISTER_EXTERNAL_CLIENT(uhal::Axi4Lite, "ipbusaxi4lite-2.0", "AXI4 Lite IPBus client")

namespace uhal {


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

void Axi4Lite::MappedFile::open() {
  if (mBar != nullptr) return;

  mFd = ::open(mPath.c_str(), (mProtFlags & PROT_WRITE) ? O_RDWR : O_RDONLY);
  if (mFd < 0) {
    return;
  }

  void* lBar = mmap(nullptr, mLength, mProtFlags, MAP_SHARED, mFd, 0);
  mBar = (lBar == MAP_FAILED ? nullptr : (uint32_t*)lBar);
}

void Axi4Lite::MappedFile::close() {
  if (mBar != nullptr) munmap(mBar, mLength);

  if (mFd != -1) ::close(mFd);
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
    log(lExc, "Failed to lock device file ", Quote(mPath),
        "; errno=", Integer(errno), ", meaning ", Quote(strerror(errno)));
    throw lExc;
  }
  mLocked = true;
}

void Axi4Lite::MappedFile::unlock() {
  if (flock(mFd, LOCK_UN) == -1) {
    log(Warning(), "Failed to unlock device file ", Quote(mPath),
        "; errno=", Integer(errno), ", meaning ", Quote(strerror(errno)));
  } else
    mLocked = false;
}

Axi4Lite::Axi4Lite(const std::string& aId, const URI& aUri) : 
IPbus< 2 , 0 > ( aId , aUri )
// mConnected(false), mMapped(aUri.mHostname)
{

}


Axi4Lite::~Axi4Lite() {

}

} // namespace uhal