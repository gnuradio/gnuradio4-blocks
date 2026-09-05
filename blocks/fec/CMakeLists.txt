add_library(gr-fec INTERFACE)
add_library(gnuradio4::gr-fec ALIAS gr-fec)
target_link_libraries(gr-fec INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm)
target_include_directories(gr-fec INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                            $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-fec EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/fec"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")
set(GrFecBlocks_HDRS
    include/gnuradio-4.0/fec/ConvBlocks.hpp
    include/gnuradio-4.0/fec/FecBlocks.hpp
    include/gnuradio-4.0/fec/RsBlocks.hpp)
gr_add_block_library(
  GrFecBlocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  ${GrFecBlocks_HDRS}
  LINK_LIBRARIES
  gr-fec)

if(TARGET gnuradio4::GrFecBlocksShared AND ENABLE_TESTING)
  add_subdirectory(test)
endif()
