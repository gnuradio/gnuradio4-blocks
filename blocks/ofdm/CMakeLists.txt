add_library(gr-ofdm INTERFACE)
add_library(gnuradio4::gr-ofdm ALIAS gr-ofdm)
target_link_libraries(gr-ofdm INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm)
target_include_directories(gr-ofdm INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                             $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-ofdm EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/ofdm"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

gr_add_block_library(
  GrOfdmBlocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  include/gnuradio-4.0/ofdm/CarrierAllocator.hpp
  include/gnuradio-4.0/ofdm/CyclicPrefix.hpp
  LINK_LIBRARIES
  gr-ofdm)
# The precompiled header reaches the vendored PocketFFT through CyclicPrefix.hpp and fourier/fft.hpp;
# cmake/CompilerWarnings.cmake turns -Wnull-dereference off for it under GCC 14.
set_target_properties(GrOfdmBlocksObject PROPERTIES GR_PCH_INCLUDES_POCKETFFT ON)

if(TARGET gnuradio4::GrOfdmBlocksShared AND ENABLE_TESTING)
  add_subdirectory(test)
endif()
