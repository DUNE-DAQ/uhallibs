#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "uhal/ClientInterface.hpp"
#include "uhal/ProtocolIPbus.hpp"
#include "uhal/log/exception.hpp"
#include "uhal/log/log.hpp"

namespace uhal {

namespace exception {

UHAL_DEFINE_DERIVED_EXCEPTION_CLASS(
    PCIe4CommunicationError, TransportLayerError,
    "Exception class to handle a low-level seek/read/write error after "
    "initialisation.")
UHAL_DEFINE_DERIVED_EXCEPTION_CLASS(
    MutexError, TransportLayerError,
    "Exception class to handle errors from pthread mutex-related functions.")

}  // namespace exception

class File {
 public:
  File(const std::string& aPath, size_t aLength, int aProtFlags);
  ~File();

  const std::string& getPath() const;
  void setPath(const std::string& aPath);

  void open();
  void close();

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

File::File(const std::string& aPath, size_t aLength, int aProtFlags)
    : mPath(aPath),
      mFd(-1),
      mBar(nullptr),
      mLength(aLength),
      mProtFlags(aProtFlags),
      mLocked(false),
      mBufferSize(0),
      mBuffer(nullptr) {}

File::~File() {
  if (mBuffer != nullptr) free(mBuffer);
  close();
}

const std::string& File::getPath() const { return mPath; }

void File::setPath(const std::string& aPath) { mPath = aPath; }

void File::open() {
  if (mBar != nullptr) return;

  mFd = ::open(mPath.c_str(), (mProtFlags & PROT_WRITE) ? O_RDWR : O_RDONLY);
  if (mFd < 0) {
    return;
  }

  void* lBar = mmap(nullptr, mLength, mProtFlags, MAP_SHARED, mFd, 0);
  mBar = (lBar == MAP_FAILED ? nullptr : (uint32_t*)lBar);
}

void File::close() {
  if (mBar != nullptr) munmap(mBar, mLength);

  if (mFd != -1) ::close(mFd);
}

void File::createBuffer(const size_t aNrBytes) {
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
    exception::PCIe4CommunicationError lExc;
    log(lExc, "Failed to allocate ", Integer(aNrBytes + 4096),
        " bytes in File::createBuffer");
    throw lExc;
  }

  mBufferSize = aNrBytes + 4096;
}

void File::read(const uint32_t aAddr, const uint32_t aNrWords,
                std::vector<uint32_t>& aValues) {
  if (mBar == nullptr) open();

  for (size_t i(0); i < aNrWords; ++i) {
    aValues.push_back(le32toh(mBar[aAddr + i]));
  }
}

void File::write(const uint32_t aAddr, const std::vector<uint32_t>& aValues) {
  write(4 * aAddr, reinterpret_cast<const uint8_t*>(aValues.data()),
        4 * aValues.size());
}

void File::write(const uint32_t aAddr, const uint8_t* const aPtr,
                 const size_t aNrBytes) {
  if (mBar == nullptr) open();

  assert((aNrBytes % 4) == 0);
  uint32_t lNrWordsData = aNrBytes / 4;

  auto lPtr32 = reinterpret_cast<const uint32_t*>(aPtr);
  for (size_t i(0); i < lNrWordsData; ++i) {
    mBar[aAddr/4 + i] = lPtr32[i];
  }
}

void File::write(const uint32_t aAddr,
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
    mBar[aAddr/4 + i] = htole32(mBuffer32b[i]);
  }
  // std::memcpy(mBar + aAddr, mBuffer, lNrBytes);

}

bool File::haveLock() const { return mLocked; }

void File::lock() {
  if (flock(mFd, LOCK_EX) == -1) {
    exception::MutexError lExc;
    log(lExc, "Failed to lock device file ", Quote(mPath),
        "; errno=", Integer(errno), ", meaning ", Quote(strerror(errno)));
    throw lExc;
  }
  mLocked = true;
}

void File::unlock() {
  if (flock(mFd, LOCK_UN) == -1) {
    log(Warning(), "Failed to unlock device file ", Quote(mPath),
        "; errno=", Integer(errno), ", meaning ", Quote(strerror(errno)));
  } else
    mLocked = false;
}

}  // namespace uhal


int main(int argc, char const* argv[]) {
  /* code */

  std::cout << "hello world" << std::endl;

  uint32_t n_pages;
  uint32_t page_size;
  uint32_t next_write_index;
  uint32_t read_counts;

  std::string lBarFile = "/sys/bus/pci/devices/0000:41:00.0/resource2";
  uhal::File f(lBarFile, 0x10000, PROT_WRITE);

  std::vector<uint32_t> lStats;
  f.open();
  f.lock();

  // Inject an ipbys read transation
  std::cout << "--- Stats ---" << std::endl;
  lStats.clear();
  f.read(0, 4, lStats);

  for (uint64_t i(0); i < lStats.size(); ++i) {
    std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << i
              << "   " << std::setw(8) << std::setfill('0') << lStats[i]
              << std::endl;
  }

  n_pages = lStats[0];
  page_size = lStats[1];
  next_write_index = lStats[2];
  read_counts = lStats[3];

  std::cout << "n_pages " << n_pages << std::endl;
  std::cout << "page_size " << page_size << std::endl;
  std::cout << "next_write_index " << next_write_index << std::endl;
  std::cout << "read_counts " << read_counts << std::endl;
  std::cout << "-------------" << std::endl;


  uint64_t write_base = next_write_index * page_size;
  std::cout << "write base : 0x" << std::hex << std::setw(8)
            << std::setfill('0') << write_base << std::endl;

  std::vector<uint32_t> lWriteVal = {
    0x00010002,
    0x200001F0,
    0x2001010F,
    0x00000001};

  for (uint64_t i(0); i < lWriteVal.size(); ++i) {
    std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << i+write_base
              << "   " << std::setw(8) << std::setfill('0') << lWriteVal[i]
              << std::endl;
  }

  f.write(write_base, lWriteVal);

  // Read back
  std::cout << "--- Stats ---" << std::endl;
  lStats.clear();
  f.read(0, 4, lStats);

  for (uint64_t i(0); i < lStats.size(); ++i) {
    std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << i
              << "   " << std::setw(8) << std::setfill('0') << lStats[i]
              << std::endl;
  }

  n_pages = lStats[0];
  page_size = lStats[1];
  next_write_index = lStats[2];
  read_counts = lStats[3];

  std::cout << "n_pages " << n_pages << std::endl;
  std::cout << "page_size " << page_size << std::endl;
  std::cout << "next_write_index " << next_write_index << std::endl;
  std::cout << "read_counts " << read_counts << std::endl;
  std::cout << "-------------" << std::endl;

  const size_t next_read_index = (next_write_index - 1 + n_pages) % n_pages;
  const size_t read_base = 4 + (next_read_index * page_size);
  std::cout << "read base : 0x" << std::hex << std::setw(8) << std::setfill('0')
            << read_base << std::endl;

  std::vector<uint32_t> lReadVal;
  f.read(read_base, 8, lReadVal);

  for (uint64_t i(0); i < lReadVal.size(); ++i) {
    std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << i+read_base
              << "   " << std::setw(8) << std::setfill('0') << lReadVal[i]
              << std::endl;
  }

  f.unlock();
  f.close();

  return 0;
}