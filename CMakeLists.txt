cmake_minimum_required(VERSION 3.27)

project(
  gnuradio4Blocks
  VERSION 4.0.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# No target here declares a C++20 module, so dependency scanning would only add a preprocessor pass per translation
# unit; disable it until a module exists to discover.
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

# Block libraries are opened with dlopen, so a dependency outside the system prefix must be on the installed RPATH for
# the loader to resolve it. CMake applies the variable to every target created after this point, which makes it a
# project-wide default rather than a setting of one plugin; a value supplied on the command line or by an enclosing
# project is kept.
option(CMAKE_INSTALL_RPATH_USE_LINK_PATH "Add the directories of linked libraries to the installed RPATH" ON)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)
include(FetchContent)
find_package(PkgConfig QUIET)

option(ENABLE_EXAMPLES "Enable example builds" ON)
option(ENABLE_TESTING "Enable test builds" ON)
option(GR4_ENABLE_AUDIO "Build the audio block family" OFF)
option(GR4_ENABLE_SDR "Build the SDR block family" OFF)
option(GR4_ENABLE_NETWORK "Build the network block family" OFF)
option(GR4_ENABLE_ZEROMQ "Build the ZeroMQ block family" OFF)
option(GR4_ENABLE_ZEROMQ_EXAMPLES "Build the ZeroMQ examples" OFF)
option(GR4_ENABLE_HTTP_TESTS "Build HTTP integration tests" OFF)
option(GR_USE_FETCHCONTENT_DEPS "Allow CMake to download selected third-party dependencies with FetchContent" OFF)
option(USE_CCACHE "Use ccache if available" ON)
option(WARNINGS_AS_ERRORS "Enable -Werror flags" ON)

# A prefix this tree has already been installed into. What an export writes into an installed config file is read by
# consumers and by nothing inside the build, so the legs that check it configure a separate project against a real
# install; empty means the machine has none and those legs are not registered.
set(GR4_INSTALLED_PREFIX
    ""
    CACHE PATH "Prefix this tree is installed into; enables the legs that configure a consumer against the export")

# Test executables check correctness, not performance; a lower optimization level roughly halves their compile time and
# peak RSS. Benchmarks pin their own level and are unaffected. Empty keeps the build type's default.
set(GR_QA_OPTIMIZATION_LEVEL
    "-O1"
    CACHE STRING "Optimization level for test executables in optimized build types (empty = build-type default)")

# Bound the concurrency of the template-heaviest compilations, whose peak memory dominates the build. The pool limits
# only targets assigned to it; other Ninja work remains parallel. Width one keeps the heaviest translation units from
# compiling side by side: each test TU instantiates whole graphs, so any two of them next to ordinary lanes can exceed a
# bounded memory budget. Every unit-test executable and the packet-link example applications compile through the pool;
# the generated block-definition units are lighter and stay in the free lanes.
set(GR_HEAVY_COMPILE_JOBS
    "1"
    CACHE STRING "Concurrent compile jobs for the largest template-heavy TUs")
if(CMAKE_GENERATOR MATCHES "Ninja")
  set_property(GLOBAL APPEND PROPERTY JOB_POOLS gr4_heavy_compile=${GR_HEAVY_COMPILE_JOBS})
endif()

set(GR4_PKG_VERSION
    "${PROJECT_VERSION}-git"
    CACHE STRING "Version string used in gnuradio4Blocks.pc")
set(GR_TEST_HELPER_LIBRARIES "")
set(PYTHON_AVAILABLE OFF)

# limit parallel compile/link jobs so template-heavy TUs cannot exhaust memory, override with
# -DGR_BUILD_PARALLEL_LEVEL=<N> (mirrors gnuradio4-core). The default is a conservative floor rather than a constant:
# the safe level follows the host's free memory and what else is running, so measure before raising it.
set(GR_BUILD_PARALLEL_LEVEL
    "4"
    CACHE STRING "Maximum parallel compile/link jobs")
if(CMAKE_GENERATOR MATCHES "Ninja")
  set_property(GLOBAL APPEND PROPERTY JOB_POOLS compile_pool=${GR_BUILD_PARALLEL_LEVEL} link_pool=2)
  set(CMAKE_JOB_POOL_COMPILE compile_pool)
  set(CMAKE_JOB_POOL_LINK link_pool)
endif()

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM AND USE_CCACHE)
  message(STATUS "ccache found and will be used")
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
elseif(USE_CCACHE)
  message(WARNING "USE_CCACHE is ON but ccache was not found on PATH — builds will not be cached")
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
# the installed gnuradio4::gnuradio-options keeps its warning flags behind $<BUILD_INTERFACE:...>, so they do not reach
# this project through the export; recreate them on a local wrapper target so this project's own targets compile with
# -Wall/-Werror & co. — gr_add_block_library object libraries pick this target up automatically. The battery sits on an
# interface target of its own, listed after the imported one, because a target's own options precede the ones it
# inherits: carried directly on the wrapper, a local suppression would land ahead of the flags an SDK built from an
# older core still exports, and have no effect.
if(NOT TARGET gnuradio-options AND TARGET gnuradio4::gnuradio-options)
  include(cmake/CompilerWarnings.cmake)
  add_library(gnuradio-blocks-warnings INTERFACE)
  set_project_warnings(gnuradio-blocks-warnings)
  add_library(gnuradio-options INTERFACE)
  target_link_libraries(gnuradio-options INTERFACE gnuradio4::gnuradio-options gnuradio-blocks-warnings)
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

if(EMSCRIPTEN AND TARGET ut)
  # Boost.UT upstream adds -fwasm-exceptions which conflicts with -fexceptions, so replace it
  get_target_property(_ut_copts ut INTERFACE_COMPILE_OPTIONS)
  get_target_property(_ut_lopts ut INTERFACE_LINK_OPTIONS)
  if(_ut_copts)
    list(REMOVE_ITEM _ut_copts -fwasm-exceptions)
    list(APPEND _ut_copts -fexceptions)
    set_target_properties(ut PROPERTIES INTERFACE_COMPILE_OPTIONS "${_ut_copts}")
  endif()
  if(_ut_lopts)
    list(REMOVE_ITEM _ut_lopts -fwasm-exceptions)
    list(APPEND _ut_lopts -fexceptions)
    set_target_properties(ut PROPERTIES INTERFACE_LINK_OPTIONS "${_ut_lopts}")
  endif()
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

set(GR4_EXPORT_NEEDS_SOAPYSDR OFF)
if(GR4_ENABLE_SDR)
  find_package(SoapySDR CONFIG QUIET)
  if(TARGET SoapySDR)
    # the sdr family links SoapySDR's imported target and the export writes that name into the installed targets file;
    # record it so the package config requires the same dependency of a consumer.
    set(GR4_EXPORT_NEEDS_SOAPYSDR ON)
  endif()
endif()

set(GR4_EXPORT_NEEDS_ZEROMQ OFF)
set(GR4_ZEROMQ_CPPZMQ_PROVIDER "")
if(GR4_ENABLE_ZEROMQ)
  find_package(cppzmq CONFIG QUIET)
  if(TARGET cppzmq)
    set(GR4_CPPZMQ_TARGET cppzmq)
    set(GR4_ZEROMQ_CPPZMQ_PROVIDER "cmake")
  else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(
      CPPZMQ
      REQUIRED
      IMPORTED_TARGET
      cppzmq)
    set(GR4_CPPZMQ_TARGET PkgConfig::CPPZMQ)
    set(GR4_ZEROMQ_CPPZMQ_PROVIDER "pkgconfig")
  endif()
  set(GR4_EXPORT_NEEDS_ZEROMQ ON)
endif()

if(GR4_ENABLE_NETWORK)
  # libzmq through pkg-config and the header-only cppzmq binding beside it: typed socket options are what keep
  # ZMQ_MAXMSGSIZE, an int64_t among options that are mostly int, from being set at the wrong width.
  pkg_check_modules(
    GR4_LIBZMQ
    QUIET
    IMPORTED_TARGET
    libzmq)
  # cppzmq as a package before cppzmq as a loose header: a package carries the directory zmq.hpp sits in, and a target
  # is the only form of that directory an installed config file can re-create for a consumer. The bare header search
  # stays as the last resort, for a tree that has zmq.hpp and no package files for it; the directory it answers with
  # belongs to this machine and is no statement about any other, so it is the provider rather than the directory that is
  # recorded, and the installed config file searches the consumer's own machine.
  set(GR4_NETWORK_CPPZMQ_TARGET "")
  set(GR4_NETWORK_CPPZMQ_PROVIDER "")
  find_package(cppzmq CONFIG QUIET)
  if(TARGET cppzmq)
    set(GR4_NETWORK_CPPZMQ_TARGET cppzmq)
    set(GR4_NETWORK_CPPZMQ_PROVIDER "cmake")
  else()
    pkg_check_modules(
      GR4_CPPZMQ
      QUIET
      IMPORTED_TARGET
      cppzmq)
    if(TARGET PkgConfig::GR4_CPPZMQ)
      set(GR4_NETWORK_CPPZMQ_TARGET PkgConfig::GR4_CPPZMQ)
      set(GR4_NETWORK_CPPZMQ_PROVIDER "pkgconfig")
    else()
      find_path(GR4_CPPZMQ_INCLUDE_DIR zmq.hpp)
      if(GR4_CPPZMQ_INCLUDE_DIR)
        set(GR4_NETWORK_CPPZMQ_PROVIDER "header")
      endif()
    endif()
  endif()
endif()

set(GR4_EXPORT_NEEDS_LIBSOUNDIO OFF)
if(GR4_ENABLE_AUDIO AND NOT EMSCRIPTEN)
  # the audio family's backend links gnuradio4::gr-libsoundio, which nothing else defines. libsoundio ships no
  # pkg-config file, so it is found by library and header and wrapped here; under Emscripten the family uses its own
  # audio worklet backend and needs none of this.
  find_library(GR4_LIBSOUNDIO_LIBRARY soundio)
  find_path(GR4_LIBSOUNDIO_INCLUDE_DIR soundio/soundio.h)
  if(GR4_LIBSOUNDIO_LIBRARY AND GR4_LIBSOUNDIO_INCLUDE_DIR)
    add_library(gr-libsoundio INTERFACE)
    add_library(gnuradio4::gr-libsoundio ALIAS gr-libsoundio)
    # Both are what a search answered on this machine and are a statement about no other, so neither is exported: a
    # consumer that reads a library path and an include directory out of an installed targets file resolves them only
    # where the same paths exist. The install interface says that the dependency is needed and nothing more, and the
    # installed config file runs the same two searches on the consumer's own machine and attaches what they find to the
    # imported target.
    target_include_directories(gr-libsoundio SYSTEM INTERFACE $<BUILD_INTERFACE:${GR4_LIBSOUNDIO_INCLUDE_DIR}>)
    target_link_libraries(gr-libsoundio INTERFACE $<BUILD_INTERFACE:${GR4_LIBSOUNDIO_LIBRARY}>)
    set(GR4_EXPORT_NEEDS_LIBSOUNDIO ON)
    install(TARGETS gr-libsoundio EXPORT gnuradio4Targets)
  else()
    message(WARNING "GR4_ENABLE_AUDIO is ON but libsoundio was not found — the audio block family is not built")
    set(GR4_ENABLE_AUDIO OFF)
  endif()
endif()

if(ENABLE_TESTING)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/GnuRadioTestHelpers.cmake)
  add_test(NAME qa_BlockNamespaceConsistency COMMAND ${CMAKE_COMMAND} -DSOURCE_ROOT=${CMAKE_CURRENT_SOURCE_DIR} -P
                                                     ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CheckBlockNamespaces.cmake)

  # the sweep reads dynamic symbol tables, so it needs nm and a build that produces shared libraries
  find_program(GR4_NM_EXECUTABLE NAMES nm)
  set(_gr4_sweep_plugin_symbols ON)
  if(NOT GR4_NM_EXECUTABLE OR EMSCRIPTEN)
    set(_gr4_sweep_plugin_symbols OFF)
  endif()
  if(DEFINED GR_BLOCKLIB_BUILD_LIBS AND NOT GR_BLOCKLIB_BUILD_LIBS)
    set(_gr4_sweep_plugin_symbols OFF)
  endif()
  if(_gr4_sweep_plugin_symbols)
    add_test(
      NAME qa_PluginInitSymbols
      COMMAND
        ${CMAKE_COMMAND} -DPLUGIN_ROOT=${CMAKE_BINARY_DIR}/blocks -DNM_EXECUTABLE=${GR4_NM_EXECUTABLE}
        -DLIBRARY_SUFFIX=${CMAKE_SHARED_LIBRARY_SUFFIX} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CheckPluginSymbols.cmake)
  endif()
endif()

add_subdirectory(bench)
add_subdirectory(blocks)

set(GR4_EXPORT_NAME gnuradio4Targets)
set(GR4_CMAKE_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/gnuradio4Blocks")

# What the network family recorded about the libzmq and the cppzmq its exported interface depends on. Substituted into
# the config file as literal conditions, so each must always be one of the two constants and never the empty string.
get_property(GR4_EXPORT_NEEDS_LIBZMQ GLOBAL PROPERTY GR4_EXPORT_NEEDS_LIBZMQ)
if(NOT GR4_EXPORT_NEEDS_LIBZMQ)
  set(GR4_EXPORT_NEEDS_LIBZMQ FALSE)
endif()
get_property(GR4_EXPORT_NEEDS_CPPZMQ GLOBAL PROPERTY GR4_EXPORT_NEEDS_CPPZMQ)
if(NOT GR4_EXPORT_NEEDS_CPPZMQ)
  set(GR4_EXPORT_NEEDS_CPPZMQ FALSE)
endif()

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gnuradio4BlocksConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4BlocksConfig.cmake" INSTALL_DESTINATION "${GR4_CMAKE_INSTALL_DIR}")

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4BlocksConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion)

configure_file(cmake/gnuradio4Blocks.pc.in "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4Blocks.pc" @ONLY)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4BlocksConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4BlocksConfigVersion.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/gnuradio4Blocks.pc" DESTINATION "${GR4_CMAKE_INSTALL_DIR}")

install(
  EXPORT ${GR4_EXPORT_NAME}
  NAMESPACE gnuradio4::
  DESTINATION "${GR4_CMAKE_INSTALL_DIR}")
