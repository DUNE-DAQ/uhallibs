#include "uhal/ConnectionManager.hpp"

#include "uhal/log/log.hpp"

#include <iostream>

int main(int argc, char const *argv[])
{

    uhal::setLogLevelTo(uhal::Debug());

    uhal::ConnectionManager cm("file://${WUPPER_TOYBOX_SHARE}/config/c.xml", {"ipbusflx-2.0", "ipbusaxi4lite-2.0"});

    uhal::HwInterface u50 = cm.getDevice("u50-axi4lite");

    u50.getNode("reg").write(0xbbbb);
    u50.dispatch();
    auto v = u50.getNode("reg").read();
    u50.dispatch();

    std::cout << "hex(v) " << std::hex << v << std::endl;



    std::cout << "Done " << std::endl;
    /* code */
    return 0;
}