add_library(gr-channel INTERFACE)
add_library(gnuradio4::gr-channel ALIAS gr-channel)
target_link_libraries(gr-channel INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm)
target_include_directories(gr-channel INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                                $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-channel EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/channel"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

gr_add_block_library(
  GrChannelBlocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  include/gnuradio-4.0/channel/Awgn.hpp
  include/gnuradio-4.0/channel/CarrierImpairments.hpp
  include/gnuradio-4.0/channel/Nonlinearity.hpp
  include/gnuradio-4.0/channel/Quantizer.hpp
  include/gnuradio-4.0/channel/Multipath.hpp
  LINK_LIBRARIES
  gr-channel)

if(TARGET gnuradio4::GrChannelBlocksShared AND ENABLE_TESTING)
  add_subdirectory(test)
endif()

if(ENABLE_EXAMPLES)
  add_subdirectory(benchmarks)
endif()
