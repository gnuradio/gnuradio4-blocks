add_library(gr-sync INTERFACE)
add_library(gnuradio4::gr-sync ALIAS gr-sync)
target_link_libraries(gr-sync INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm)
target_include_directories(gr-sync INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                             $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-sync EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/sync"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

gr_add_block_library(
  GrSyncBlocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  include/gnuradio-4.0/sync/CostasLoop.hpp
  include/gnuradio-4.0/sync/Pll.hpp
  LINK_LIBRARIES
  gr-sync)

if(TARGET gnuradio4::GrSyncBlocksShared AND ENABLE_TESTING)
  add_subdirectory(test)
endif()

if(ENABLE_EXAMPLES)
  add_subdirectory(benchmarks)
endif()
