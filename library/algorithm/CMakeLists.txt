add_library(gnuradio-algorithm INTERFACE)
add_library(gnuradio4::gnuradio-algorithm ALIAS gnuradio-algorithm)
target_include_directories(gnuradio-algorithm INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                                        $<INSTALL_INTERFACE:include/>)
# fourier/fft.hpp includes the vendored PocketFFT at its installed path below gnuradio-4.0/. That path cannot collide
# with a pocketfft installed on the system. It resolves from the include root alone. A consumer that finds this project
# through pkg-config needs no include directory beyond -I${includedir}. The source tree mirrors the installed layout
# under third_party/include/. That directory is a system include because the vendored sources do not pass this project's
# warning flags.
target_include_directories(gnuradio-algorithm SYSTEM
                           INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../third_party/include/>)
install(
  TARGETS gnuradio-algorithm
  EXPORT ${GR4_EXPORT_NAME}
  COMPONENT Devel)
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/algorithm"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0"
  COMPONENT Devel)
install(
  FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/meta/UncertainValue.hpp"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0/meta"
  COMPONENT Devel)
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/include/gnuradio-4.0/third_party/pocketfft"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0/third_party"
  COMPONENT Devel)

# gnuradio-core carries MemoryAllocators/Message/HistoryBuffer/Tensor/DataSet, which the public headers of this target
# include directly
target_link_libraries(
  gnuradio-algorithm
  INTERFACE gnuradio4::gnuradio-options
            gnuradio4::gnuradio-core
            gnuradio4::gnuradio-meta
            gnuradio4::vir
            gnuradio4::magic_enum)

if(GR_HTTP_ENABLED AND NOT EMSCRIPTEN)
  target_link_libraries(gnuradio-algorithm INTERFACE cpr::cpr)
endif()
target_compile_definitions(gnuradio-algorithm INTERFACE GR_HTTP_ENABLED=$<BOOL:${GR_HTTP_ENABLED}>)

if(ENABLE_EXAMPLES)
  add_subdirectory(benchmarks)
  add_subdirectory(src)
endif()

if(ENABLE_TESTING)
  add_subdirectory(test)
endif()
