# Copyright 2011,2012,2016,2018,2019 Free Software Foundation, Inc.
#
# This file was generated by gr_modtool, a tool from the GNU Radio framework
# This file is a part of gr-difi
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX
list(APPEND difi_sources
    difi_source_cpp_impl.cc
    difi_sink_cpp_impl.cc
    synchronize_impl.cc
    tcp_server.h
    tcp_server.cc
    tcp_client.h
    tcp_client.cc
    udp_socket.h
    udp_socket.cc )

set(difi_sources "${difi_sources}" PARENT_SCOPE)
if(NOT difi_sources)
    MESSAGE(STATUS "No C++ sources... skipping lib/")
    return()
endif(NOT difi_sources)

add_library(gnuradio-difi SHARED ${difi_sources})
target_link_libraries(gnuradio-difi gnuradio::gnuradio-runtime)
target_include_directories(gnuradio-difi
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    PUBLIC $<INSTALL_INTERFACE:include>
  )
set_target_properties(gnuradio-difi PROPERTIES DEFINE_SYMBOL "gnuradio_difi_EXPORTS")

if(APPLE)
    set_target_properties(gnuradio-difi PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

########################################################################
# Install built library files
########################################################################
include(GrMiscUtils)
GR_LIBRARY_FOO(gnuradio-difi)

########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Building for version: ${VERSION} / ${LIBVER}")

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

# If your unit tests require special include paths, add them here
#include_directories()
# List all files that contain Boost.UTF unit tests here
list(APPEND test_difi_sources
)
# Anything we need to link to for the unit tests go here
list(APPEND GR_TEST_TARGET_DEPS gnuradio-difi)

if(NOT test_difi_sources)
    MESSAGE(STATUS "No C++ unit tests... skipping")
    return()
endif(NOT test_difi_sources)

foreach(qa_file ${test_difi_sources})
    GR_ADD_CPP_TEST("difi_${qa_file}"
        ${CMAKE_CURRENT_SOURCE_DIR}/${qa_file}
    )
endforeach(qa_file)
