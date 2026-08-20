cmake_minimum_required(VERSION 3.28)

project(gnuradio4_blocks_consumer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(gnuradio4Blocks CONFIG REQUIRED)

add_executable(gnuradio4_blocks_consumer main.cpp)
target_link_libraries(gnuradio4_blocks_consumer PRIVATE gnuradio4::GrBasicBlocksStatic)
