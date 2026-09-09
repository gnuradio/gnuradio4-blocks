add_library(gr-adsb INTERFACE)
add_library(gnuradio4::gr-adsb ALIAS gr-adsb)
target_link_libraries(gr-adsb INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm)
target_include_directories(gr-adsb INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                             $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-adsb EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/adsb"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

gr_add_block_library(
  GrAdsbBlocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  include/gnuradio-4.0/adsb/PpmFramer.hpp
  include/gnuradio-4.0/adsb/ModeS.hpp
  include/gnuradio-4.0/adsb/AdsbPrinter.hpp
  LINK_LIBRARIES
  gr-adsb)

if(TARGET gnuradio4::GrAdsbBlocksShared AND ENABLE_TESTING)
  add_subdirectory(test)
endif()

if(ENABLE_EXAMPLES)
  add_subdirectory(benchmarks)
endif()
