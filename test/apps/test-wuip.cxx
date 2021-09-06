#include "boost/program_options.hpp"

#include "uhal/ConnectionManager.hpp"

#include "uhal/log/log.hpp"

#include <iostream>

namespace bpo = boost::program_options;

int main(int argc, char const *argv[])
{
    std::ostringstream descstr;
    descstr << *argv
            << " known arguments (additional arguments will be stored and "
               "passed on)";
    bpo::options_description desc(descstr.str());
    desc.add_options()(
      "device,d", bpo::value<std::string>()->default_value("flx-0-ipb"), "Device id")(
      "help,h", "produce help message");

    bpo::variables_map vm;
    try {
    //   auto parsed = bpo::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
      auto parsed = bpo::command_line_parser(argc, argv).options(desc).run();

    //   output.other_options = bpo::collect_unrecognized(parsed.options, bpo::include_positional);
      bpo::store(parsed, vm);
    } catch (bpo::error const& e) {
        std::cerr << e.what() << std::endl;
    }

    if (vm.count("help")) {
      std::cout << desc << std::endl; // NOLINT
      return 0;
    }

    try {
      bpo::notify(vm);
    } catch (bpo::error const& e) {
        std::cerr << e.what() << std::endl;
    }

    uhal::setLogLevelTo(uhal::Debug());

    uhal::ConnectionManager cm("file://${WUPPER_TOYBOX_SHARE}/config/c.xml", {"ipbusflx-2.0"});

    uhal::HwInterface flx = cm.getDevice(vm["device"].as<std::string>());

    flx.getNode("reg").write(0xbbbb);
    flx.dispatch();
    auto v = flx.getNode("reg").read();
    flx.dispatch();

    std::cout << "hex(v) " << std::hex << v << std::endl;



    std::cout << "Done " << std::endl;
    /* code */
    return 0;
}