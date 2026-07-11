cmake_minimum_required(VERSION 3.27)

project(
  gnuradio4Blocks
  VERSION 4.0.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)
include(FetchContent)
find_package(PkgConfig QUIET)

option(ENABLE_EXAMPLES "Enable example builds" ON)
option(ENABLE_TESTING "Enable test builds" ON)
option(GR4_ENABLE_AUDIO "Build the audio block family" OFF)
option(GR4_ENABLE_SDR "Build the SDR block family" OFF)
option(GR4_ENABLE_HTTP_TESTS "Build HTTP integration tests" OFF)
option(GR_USE_FETCHCONTENT_DEPS "Allow CMake to download selected third-party dependencies with FetchContent" OFF)
option(USE_CCACHE "Use ccache if available" ON)
set(GR4_PKG_VERSION "${PROJECT_VERSION}-git" CACHE STRING "Version string used in gnuradio4Blocks.pc")
set(GR_TEST_HELPER_LIBRARIES "")
set(PYTHON_AVAILABLE OFF)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM AND USE_CCACHE)
  message(STATUS "ccache found and will be used")
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
else()
  message(STATUS "ccache will not be used")
endif()

if(ENABLE_TESTING
   AND (UNIX
        OR APPLE
        OR WIN32))
  include(CTest)
  enable_testing()
endif()

find_package(gnuradio4 CONFIG REQUIRED)
find_package(gnuradio4Library CONFIG REQUIRED)
find_package(GnuRadioBlockLib CONFIG REQUIRED)

if(NOT TARGET gnuradio-core AND TARGET gnuradio4::gnuradio-core)
  add_library(gnuradio-core ALIAS gnuradio4::gnuradio-core)
endif()
if(NOT TARGET gnuradio-algorithm AND TARGET gnuradio4::gnuradio-algorithm)
  add_library(gnuradio-algorithm ALIAS gnuradio4::gnuradio-algorithm)
endif()
if(NOT TARGET gnuradio-options AND TARGET gnuradio4::gnuradio-options)
  add_library(gnuradio-options ALIAS gnuradio4::gnuradio-options)
endif()
if(NOT TARGET gnuradio-blocklib-core AND TARGET gnuradio4::gnuradio-blocklib-core)
  add_library(gnuradio-blocklib-core ALIAS gnuradio4::gnuradio-blocklib-core)
endif()
if(NOT TARGET gnuradio-plugin AND TARGET gnuradio4::gnuradio-plugin)
  add_library(gnuradio-plugin ALIAS gnuradio4::gnuradio-plugin)
endif()
if(NOT TARGET ut-benchmark AND TARGET gnuradio4::ut-benchmark)
  add_library(ut-benchmark ALIAS gnuradio4::ut-benchmark)
endif()
set(BOOST_UT_DISABLE_MODULE
    ON
    CACHE BOOL "Disable UT Module Support" FORCE)
add_compile_definitions(BOOST_UT_DISABLE_MODULE)
if(EMSCRIPTEN OR GR_USE_FETCHCONTENT_DEPS)
  FetchContent_Declare(
    ut
    GIT_REPOSITORY https://github.com/boost-ext/ut.git
    GIT_TAG 53e17f25119598c6458d30351b260193096ba67e
    EXCLUDE_FROM_ALL)
else()
  find_package(ut CONFIG QUIET)
  if(TARGET Boost::ut AND NOT TARGET ut)
    add_library(ut ALIAS Boost::ut)
  elseif(TARGET boost::ut AND NOT TARGET ut)
    add_library(ut ALIAS boost::ut)
  endif()
  if(NOT TARGET ut)
    find_path(GR_BOOST_UT_INCLUDE_DIR boost/ut.hpp)
    if(NOT GR_BOOST_UT_INCLUDE_DIR)
      message(FATAL_ERROR "Boost.UT was not found. Install libboost-ext-ut-dev, ut-devel, or a source-built Boost.UT.")
    endif()
    add_library(ut INTERFACE)
    target_include_directories(ut INTERFACE ${GR_BOOST_UT_INCLUDE_DIR})
  endif()
endif()
if(EMSCRIPTEN OR GR_USE_FETCHCONTENT_DEPS)
  FetchContent_MakeAvailable(ut)
endif()
if(NOT TARGET Boost::ut AND TARGET ut)
  add_library(Boost::ut ALIAS ut)
endif()

find_package(httplib CONFIG QUIET)
if(NOT TARGET httplib::httplib)
  pkg_search_module(
    CPP_HTTPLIB
    QUIET
    IMPORTED_TARGET
    cpp-httplib
    httplib)
  if(CPP_HTTPLIB_FOUND)
    add_library(httplib::httplib INTERFACE IMPORTED)
    target_link_libraries(httplib::httplib INTERFACE PkgConfig::CPP_HTTPLIB)
  endif()
endif()

if(GR4_ENABLE_SDR)
  find_package(SoapySDR CONFIG QUIET)
endif()

if(ENABLE_TESTING)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/GnuRadioTestHelpers.cmake)
endif()

add_subdirectory(bench)
add_subdirectory(blocks)

set(GR4_EXPORT_NAME gnuradio4Targets)
set(GR4_CMAKE_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/gnuradio4Blocks")

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gnuradio4BlocksConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4BlocksConfig.cmake"
  INSTALL_DESTINATION "${GR4_CMAKE_INSTALL_DIR}")

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4BlocksConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion)

configure_file(cmake/gnuradio4Blocks.pc.in "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4Blocks.pc" @ONLY)

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4BlocksConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4BlocksConfigVersion.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4Blocks.pc"
  DESTINATION "${GR4_CMAKE_INSTALL_DIR}")

install(EXPORT ${GR4_EXPORT_NAME} NAMESPACE gnuradio4:: DESTINATION "${GR4_CMAKE_INSTALL_DIR}")
