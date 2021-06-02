#include <iostream>
#include "felix/flxwrapper.hpp"

using namespace felix::core;

int main() {
    std::cout << "Hello!" << std::endl;

    Flx flx(0, 0);
    CmemBuffer cmembuf0(4096, "test0");
    CmemBuffer cmembuf1(4096, "test1");

    uint64_t* buf0 = cmembuf0.access_ptr();

    buf0[0] = 0xFF;
    buf0[1] = 0xAB;
    buf0[2] = 0xBC;
    buf0[3] = 0xDE;

    FlxSimpleDMA dma(flx, 2);
    dma.transfer_from_host(cmembuf0, 0, 4096);
    dma.wait();
    //   dma.transfer_to_host(cmembuf1, 0, 4096);


}