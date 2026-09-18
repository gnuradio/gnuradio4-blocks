cmake_minimum_required(VERSION 3.27)

# A separate project on purpose: the exported interface of gnuradio4::gr-audio is read by nothing this tree builds,
# because inside the tree the targets it names are the real ones rather than the imported ones. Only a consumer that
# finds the installed package sees what was written into the export, so only a consumer can catch an interface that
# names a target no installed config file defines, or that carries no directory for a header the family includes.
# Configuring is the first half of the test — CMake resolves a target's link closure while generating, and a name it
# cannot resolve ends the run there — and building main.cpp is the second.
project(gnuradio4_audio_consumer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(gnuradio4Blocks CONFIG REQUIRED)

if(NOT TARGET gnuradio4::gr-audio)
  message(FATAL_ERROR "the installed package defines no gnuradio4::gr-audio; the audio family was not part of the"
                      " install this consumer was pointed at")
endif()

# An executable rather than a library: a link closure is resolved for a target that has one, and an INTERFACE library
# does not. It is built as well as configured, which is what puts the exported include directories in front of a
# compiler and the exported libraries on a link line.
add_executable(gnuradio4_audio_consumer main.cpp)
target_link_libraries(gnuradio4_audio_consumer PRIVATE gnuradio4::gr-audio gnuradio4::GrAudioBlocksShared)
