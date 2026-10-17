add_library(gr-ofdm INTERFACE)
add_library(gnuradio4::gr-ofdm ALIAS gr-ofdm)
target_link_libraries(gr-ofdm INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm gnuradio4::gr-digital)
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
  include/gnuradio-4.0/ofdm/ChannelEqualizer.hpp
  include/gnuradio-4.0/ofdm/CyclicPrefix.hpp
  include/gnuradio-4.0/ofdm/SchmidlCoxSync.hpp
  LINK_LIBRARIES
  gr-ofdm)

if(TARGET gnuradio4::GrOfdmBlocksShared AND ENABLE_TESTING)
  add_subdirectory(test)
endif()

if(ENABLE_EXAMPLES)
  add_subdirectory(benchmarks)
endif()
