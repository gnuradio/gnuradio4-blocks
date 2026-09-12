add_library(gr-sdr INTERFACE)
add_library(gnuradio4::gr-sdr ALIAS gr-sdr)
target_include_directories(gr-sdr INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                            $<INSTALL_INTERFACE:include/>)
target_link_libraries(gr-sdr INTERFACE gnuradio4::gnuradio-core gnuradio4::gr-common gnuradio4::gnuradio-algorithm)
install(TARGETS gr-sdr EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/sdr"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

if(TARGET SoapySDR)
  # the name reaches the installed targets file through the export, so the package config has to find SoapySDR again
  # before a consumer can link gnuradio4::gr-sdr.
  target_link_libraries(gr-sdr INTERFACE SoapySDR)

  # loopback device: static library for tests + optional shared module for system-wide use
  add_library(gr-sdr-loopback-static STATIC src/SoapyLoopbackModule.cpp)
  target_link_libraries(gr-sdr-loopback-static PUBLIC gr-sdr SoapySDR)
  target_link_libraries(gr-sdr-loopback-static PRIVATE gnuradio-options)
  set_target_properties(gr-sdr-loopback-static PROPERTIES POSITION_INDEPENDENT_CODE ON)

  add_library(gr-sdr-loopback MODULE src/SoapyLoopbackModule.cpp)
  target_link_libraries(gr-sdr-loopback PRIVATE gr-sdr SoapySDR gnuradio-options)
  set_target_properties(gr-sdr-loopback PROPERTIES PREFIX "") # SoapySDR expects no lib prefix

  # the registration units the headers' markers ask for. Without them a radio has no registry key, so a graph read from
  # a description cannot name one. The Soapy blocks need SoapySDR to compile, which is what puts the whole library
  # behind the find; gr-sdr above stays for consumers that instantiate the blocks directly.
  gr_add_block_library(
    GrSdrBlocks
    MAKE_SHARED_LIBRARY
    ${GR4_BLOCK_STATIC_LIBRARY}
    HEADERS
    include/gnuradio-4.0/sdr/RTL2832Source.hpp
    include/gnuradio-4.0/sdr/SoapySink.hpp
    include/gnuradio-4.0/sdr/SoapySource.hpp
    LINK_LIBRARIES
    gr-sdr
    gnuradio4::gnuradio-algorithm)

  if(TARGET GrSdrBlocksObject AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # These are the first units to instantiate SoapySink<std::uint8_t>. Inlining libstdc++'s vector growth into
    # rememberLastTransmitted() there loses the guard that makes the old size zero, and GCC reports the move of the old
    # elements as a write past the one-byte buffer it just allocated. The write does not happen.
    target_compile_options(GrSdrBlocksObject PRIVATE -Wno-stringop-overflow)
  endif()
endif()

if(ENABLE_TESTING)
  add_subdirectory(test)
endif()

if(ENABLE_EXAMPLES)
  add_subdirectory(src)
endif()
