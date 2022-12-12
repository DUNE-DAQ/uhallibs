/**

Low-level ipbus-over-axi4lite test

Small test application to test low-level ipbus communication over axi4 lite using the
uhal::Axi4Lite::MappedFile and manually injecting read and write ipbus packets.

**/
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

#include "uhallibs/ProtocolAxi4Lite.hpp"

int
main(int /* argc */, char const* /* argv[] */)
{
  /* code */

  std::string lBarFile = "/sys/bus/pci/devices/0000:41:00.0/resource0";

  std::cout << "Starting ipbus-axi4lite test on " << lBarFile << std::endl;

  uint32_t n_pages;
  uint32_t page_size;
  uint32_t next_write_index;
  uint32_t read_counts;

  uhallibs::Axi4Lite::MappedFile f(lBarFile, 0x40, PROT_WRITE);

  std::vector<uint32_t> lStats;
  f.open();
  f.lock();

  // Inject an ipbus read transation
  // Read the the status of the transport interface at address 0x0
  std::cout << "--- Stats ---" << std::endl;
  lStats.clear();
  f.read(0, 4, lStats);

  for (uint64_t i(0); i < lStats.size(); ++i) {
    std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << i << "   " << std::setw(8)
              << std::setfill('0') << lStats[i] << std::endl;
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
  f.unlock();
  f.close();

  // Re-open the interface file
  uint32_t map_size = n_pages * page_size + 4;
  f.setLength(map_size);
  f.open();
  f.lock();

  uint64_t write_base = next_write_index * page_size;
  std::cout << "write base : 0x" << std::hex << std::setw(8) << std::setfill('0') << write_base << std::endl;

  std::vector<uint32_t> lWriteVal = { 0x00010002, 0x200001F0, 0x2001010F, 0x00000001 };

  for (uint64_t i(0); i < lWriteVal.size(); ++i) {
    std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << i + write_base << "   " << std::setw(8)
              << std::setfill('0') << lWriteVal[i] << std::endl;
  }

  f.write(write_base, lWriteVal);

  // Read back
  std::cout << "--- Stats ---" << std::endl;
  lStats.clear();
  f.read(0, 4, lStats);

  for (uint64_t i(0); i < lStats.size(); ++i) {
    std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << i << "   " << std::setw(8)
              << std::setfill('0') << lStats[i] << std::endl;
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
  std::cout << "read base : 0x" << std::hex << std::setw(8) << std::setfill('0') << read_base << std::endl;

  std::vector<uint32_t> lReadVal;
  f.read(read_base, 8, lReadVal);

  for (uint64_t i(0); i < lReadVal.size(); ++i) {
    std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << i + read_base << "   " << std::setw(8)
              << std::setfill('0') << lReadVal[i] << std::endl;
  }

  f.unlock();
  f.close();

  return 0;
}
