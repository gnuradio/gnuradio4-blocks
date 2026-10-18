add_library(gr-measurement INTERFACE)
add_library(gnuradio4::gr-measurement ALIAS gr-measurement)
target_link_libraries(gr-measurement INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm)
target_include_directories(gr-measurement INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                                    $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-measurement EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/measurement"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

gr_add_block_library(
  GrMeasurementBlocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  include/gnuradio-4.0/measurement/SpectralEstimate.hpp
  LINK_LIBRARIES
  gr-measurement)

if(TARGET gnuradio4::GrMeasurementBlocksShared AND ENABLE_TESTING)
  add_subdirectory(test)
endif()
