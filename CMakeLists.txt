cmake_minimum_required(VERSION 3.12)
project(wupper-toybox VERSION 1.0.0)

find_package(daq-cmake REQUIRED)

daq_setup_environment()

find_package(felix REQUIRED)

# daq_codegen(*.jsonnet TEMPLATES Structs.hpp.j2 Nljs.hpp.j2 )

##############################################################################
# Dependency sets
# set(DUNEDAQ_DEPENDENCIES readout::readout dataformats::dataformats)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mtune=native")

set(FELIX_DEPENDENCIES 
  felix::regmap
  felix::packetformat
  felix::drivers_rcc
  felix::getinput
  #felix::io_rcc
  felix::rcc_error
  felix::cmem_rcc
  felix::DFDebug
  felix::FlxCard
)

##############################################################################
# Main library
# daq_add_library(DefaultParserImpl.cpp CardWrapper.cpp LINK_LIBRARIES appfwk::appfwk ${FELIX_DEPENDENCIES} ${DUNEDAQ_DEPENDENCIES})

##############################################################################
# Plugins

##############################################################################
# Integration tests
daq_add_application( toy-tohost-dma-test toy-tohost-dma-test.cxx LINK_LIBRARIES ${FELIX_DEPENDENCIES})
daq_add_application( toy-fromhost-dma-test toy-fromhost-dma-test.cxx LINK_LIBRARIES ${FELIX_DEPENDENCIES})

##############################################################################
# Test configurations
# file(COPY test/felix_wib_readout.json DESTINATION test)
# file(COPY test/felix_wib2_readout.json DESTINATION test)

##############################################################################
# Installation
daq_install()