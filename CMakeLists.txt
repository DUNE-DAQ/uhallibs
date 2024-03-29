cmake_minimum_required(VERSION 3.12)
project(uhallibs VERSION 1.2.0)

find_package(daq-cmake REQUIRED)

daq_setup_environment()

find_package(felix REQUIRED)
find_package(uhal REQUIRED)
find_package(Boost COMPONENTS filesystem date_time REQUIRED)

##############################################################################
# Main library
daq_add_library(*.cpp LINK_LIBRARIES felix::regmap felix::FlxCard uhal::uhal Boost::date_time)
target_compile_definitions(uhallibs PUBLIC REGMAP_VERSION=0x0500)

##############################################################################
# Plugins

##############################################################################
# Integration tests
daq_add_application( toy-tohost-dma-test toy-tohost-dma-test.cxx TEST LINK_LIBRARIES felix::FlxCard felix::rcc_error felix::cmem_rcc )
daq_add_application( test-wuip test-wuip.cxx TEST LINK_LIBRARIES uhallibs Boost::filesystem)
daq_add_application( test-uhal-axi4lite test-uhal-axi4lite.cxx TEST LINK_LIBRARIES uhallibs Boost::filesystem)
daq_add_application( test-pcie-axil test-pcie-axil.cxx TEST LINK_LIBRARIES uhallibs Boost::filesystem)
daq_add_application( test-axi4l-if test-axi4l-if.cxx TEST LINK_LIBRARIES uhallibs Boost::filesystem)

##############################################################################
# Test configurations
# file(COPY test/felix_wib_readout.json DESTINATION test)
# file(COPY test/felix_wib2_readout.json DESTINATION test)

##############################################################################
# Installation
daq_install()
