#include "uhal/ConnectionManager.hpp"

#include "uhal/log/log.hpp"

#include <iostream>

int main(int /* argc */, char const * /* argv[] */)
{

    uhal::setLogLevelTo(uhal::Debug());

    uhal::ConnectionManager cm("file://${UHALLIBS_SHARE}/config/c.xml", {"ipbusflx-2.0"});

    uhal::HwInterface flx = cm.getDevice("flx-0-ipb");

    flx.getNode("reg").write(0xbbbb);
    flx.dispatch();
    auto v = flx.getNode("reg").read();
    flx.dispatch();

    std::cout << "hex(v) " << std::hex << v << std::endl;



    std::cout << "Done " << std::endl;
    /* code */
    return 0;
}
