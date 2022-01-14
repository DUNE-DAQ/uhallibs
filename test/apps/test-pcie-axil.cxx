#include <iostream>
#include <endian.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <iomanip>
#include <vector>


/* /sys/bus/pci/devices/0000:<bus>:<dev>.<func>/resource<bar#> */
#define get_syspath_bar_mmap(s, bus,dev,func,bar) \
	snprintf(s, sizeof(s), \
		"/sys/bus/pci/devices/0000:%02x:%02x.%x/resource%u", \
		bus, dev, func, bar)

static uint32_t *mmap_bar(char *fname, size_t len, int prot)
{
	int fd;
	void *bar;

	fd = open(fname, (prot & PROT_WRITE) ? O_RDWR : O_RDONLY);
	if (fd < 0)
		return NULL;

	bar = mmap(NULL, len, prot, MAP_SHARED, fd, 0);
	close(fd);

	return bar == MAP_FAILED ? NULL : (uint32_t*)bar;
}



uint32_t reg_read(uint64_t raddr) {
    char fname[256];
    uint32_t* bar;

	get_syspath_bar_mmap(fname, 0x41, 0, 0, 2);

    // std::cout << fname << std::endl;

	bar = mmap_bar(fname, raddr*4+4, PROT_READ);

	uint32_t rval = le32toh(bar[raddr]);
	munmap(bar, raddr+4);

    return rval;
}

static int32_t reg_write(uint64_t waddr, uint32_t wval)
{
	uint32_t *bar;
	char fname[256];

	std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << waddr << "   " << std::setw(8) << std::setfill('0') << wval << std::endl;

	get_syspath_bar_mmap(fname, 0x41, 0, 0, 2);

	bar = mmap_bar(fname, waddr*4+4, PROT_WRITE);

	bar[waddr] = htole32(wval);
	munmap(bar, waddr+4);
	return 0;
}

int main() {
    std::cout << "Hello World" << std::endl;

	// char fname[256];
    // uint32_t* bar;

    // uint64_t raddr = 0x0;
	// get_syspath_bar_mmap(fname, 0x41, 0, 0, 2);

    // std::cout << fname << std::endl;

	// bar = mmap_bar(fname, raddr+4, PROT_READ);

	// uint32_t rval = le32toh(bar[raddr]);
	// munmap(bar, raddr+4);


	uint32_t n_pages;
	uint32_t page_size;
	uint32_t next_write_index;
	uint32_t read_counts;

	std::vector<uint32_t> rd_buf(4);

	for ( uint64_t i(0); i<4; ++i) {
    	uint32_t rval = reg_read(i);
		rd_buf[i] = rval;
		std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << rval << std::endl;

	}

	n_pages = rd_buf[0];
	page_size = rd_buf[1];
	next_write_index = rd_buf[2];
	read_counts = rd_buf[3];

	rd_buf.resize(page_size);


	std::cout << "n_pages " << n_pages << std::endl;
	std::cout << "page_size " << page_size << std::endl;
	std::cout << "next_write_index " << next_write_index << std::endl;
	std::cout << "read_counts " << read_counts << std::endl;

	uint64_t write_base = next_write_index * page_size;
	std::cout << "write base : 0x" << std::hex << std::setw(8) << std::setfill('0') <<  write_base << std::endl;
	reg_write(write_base+0, 0x00010002);
	reg_write(write_base+1, 0x200001F0);
	reg_write(write_base+2, 0x2001010F);
	reg_write(write_base+3, 0x00000001);



	for ( uint64_t i(0); i<4; ++i) {
    	uint32_t rval = reg_read(i);
		rd_buf[i] = rval;
		std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << i << "   " << std::setw(8) << std::setfill('0') << rval << std::endl;

	}

	n_pages = rd_buf[0];
	page_size = rd_buf[1];
	next_write_index = rd_buf[2];
	read_counts = rd_buf[3];
	
	std::cout << "n_pages " << std::dec << n_pages << std::endl;
	std::cout << "page_size " << std::dec << page_size << std::endl;
	std::cout << "next_write_index " << std::dec << next_write_index << std::endl;
	std::cout << "read_counts " << std::dec << read_counts << std::endl;

	const size_t next_read_index = (next_write_index-1+n_pages) % n_pages;
	const size_t read_base = 4 + (next_read_index * page_size);
	std::cout << "read base : 0x" << std::hex << std::setw(8) << std::setfill('0') <<  read_base << std::endl;

	for ( uint64_t i(0); i<8; ++i) {
    	uint32_t rval = reg_read(read_base+i);
		// rd_buf[i] = rval;
		std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << read_base+i << "   " << std::setw(8) << std::setfill('0') << rval << std::endl;
	}


	for ( uint64_t i(0); i<4; ++i) {
    	uint32_t rval = reg_read(i);
		std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << i << "   " << std::setw(8) << std::setfill('0') << rval << std::endl;
	}

	// std::cout << "---------------------" << std::endl;
	// for ( uint64_t i(0); i<8192; ++i) {
    // 	reg_write(i,i);
	// 	// std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << i << "   " << std::setw(8) << std::setfill('0') << rvail << std::endl;
	// }

	// std::cout << "---------------------" << std::endl;
	// for ( uint64_t i(4); i<(8192+4); ++i) {
    // 	uint32_t rval = reg_read(i);
	// 	// rd_buf[i] = rval;
	// 	std::cout << "0x" << std::hex << std::setw(8) << std::setfill('0') << i << "   " << std::setw(8) << std::setfill('0') << rval << std::endl;
	// }
}