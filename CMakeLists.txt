cmake_minimum_required(VERSION 3.12)
project(wupper-toybox VERSION 1.0.0)

find_package(daq-cmake REQUIRED)

daq_setup_environment()

find_package(felix REQUIRED)
find_package(Boost 1.73.0 COMPONENTS filesystem program_options REQUIRED)

# daq_codegen(*.jsonnet TEMPLATES Structs.hpp.j2 Nljs.hpp.j2 )
set(LIST_OF_UHAL_LIBS $ENV{UHAL_LIB}/libcactus_uhal_log.so $ENV{UHAL_LIB}/libcactus_uhal_uhal.so) 

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

add_compile_definitions(REGMAP_VERSION=0x0500)

##############################################################################
# Main library
daq_add_library(*.cpp LINK_LIBRARIES ${FELIX_DEPENDENCIES} ${LIST_OF_UHAL_LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC $ENV{UHAL_INC})

##############################################################################
# Plugins

##############################################################################
# Integration tests
daq_add_application( toy-tohost-dma-test toy-tohost-dma-test.cxx LINK_LIBRARIES ${FELIX_DEPENDENCIES})
daq_add_application( test-wuip test-wuip.cxx TEST LINK_LIBRARIES wupper-toybox ${LIST_OF_UHAL_LIBS} Boost::filesystem Boost::program_options )

##############################################################################
# Test configurations
# file(COPY test/felix_wib_readout.json DESTINATION test)
# file(COPY test/felix_wib2_readout.json DESTINATION test)

##############################################################################
# Installation
daq_install()
