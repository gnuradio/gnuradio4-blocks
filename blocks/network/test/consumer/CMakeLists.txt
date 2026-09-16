cmake_minimum_required(VERSION 3.27)

# A separate project on purpose: the exported interface of gnuradio4::gr-network is read by nothing this tree builds,
# because inside the tree the targets it names are the real ones rather than the imported ones. Only a consumer that
# finds the installed package sees what was written into the export, so only a consumer can catch an interface naming a
# target no installed config file defines. Configuring this project is the whole test — CMake resolves a target's link
# closure while generating, and a name it cannot resolve ends the run there.
project(gnuradio4_network_consumer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(gnuradio4Blocks CONFIG REQUIRED)

if(NOT TARGET gnuradio4::gr-network)
  message(FATAL_ERROR "the installed package defines no gnuradio4::gr-network; the network family was not part of the"
                      " install this consumer was pointed at")
endif()

# main.cpp is never compiled: ctest runs the configure step and stops. It exists because a link closure is resolved for
# a target that has one, and an INTERFACE library does not.
add_executable(gnuradio4_network_consumer main.cpp)
target_link_libraries(gnuradio4_network_consumer PRIVATE gnuradio4::gr-network gnuradio4::GrNetworkBlocksShared)
