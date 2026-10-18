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
  include/gnuradio-4.0/measurement/Detectors.hpp
  include/gnuradio-4.0/measurement/Kurtosis.hpp
  include/gnuradio-4.0/measurement/OccupiedBandwidth.hpp
  include/gnuradio-4.0/measurement/PhaseUnwrap.hpp
  include/gnuradio-4.0/measurement/PolarizationCombiner.hpp
  include/gnuradio-4.0/measurement/SpectralEstimate.hpp
  include/gnuradio-4.0/measurement/SpectralKurtosis.hpp
  LINK_LIBRARIES
  gr-measurement)
# The precompiled header reaches the vendored PocketFFT through SpectralEstimate.hpp and fourier/fft.hpp;
# cmake/CompilerWarnings.cmake turns -Wnull-dereference off for it under GCC 14.
set_target_properties(GrMeasurementBlocksObject PROPERTIES GR_PCH_INCLUDES_POCKETFFT ON)

if(TARGET gnuradio4::GrMeasurementBlocksShared AND ENABLE_TESTING)
  add_subdirectory(test)
endif()

if(ENABLE_EXAMPLES)
  add_subdirectory(benchmarks)
endif()
