# the math headers without the expression library, for a module that uses Unary.hpp alone
add_library(gr-math-headers INTERFACE)
add_library(gnuradio4::gr-math-headers ALIAS gr-math-headers)
target_link_libraries(gr-math-headers INTERFACE gnuradio4::gnuradio-core)
target_include_directories(gr-math-headers INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                                     $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-math-headers EXPORT gnuradio4Targets)

add_library(gr-math INTERFACE)
add_library(gnuradio4::gr-math ALIAS gr-math)
target_link_libraries(gr-math INTERFACE gr-math-headers gnuradio4::gnuradio-algorithm gnuradio4::exprtk)
install(TARGETS gr-math EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/math"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

gr_add_block_library(
  GrMathBlocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  include/gnuradio-4.0/math/ExpressionBlocks.hpp
  include/gnuradio-4.0/math/Math.hpp
  include/gnuradio-4.0/math/Rotator.hpp
  include/gnuradio-4.0/math/Unary.hpp
  LINK_LIBRARIES
  gr-math)

if(TARGET gnuradio4::GrMathBlocksShared AND ENABLE_TESTING)
  add_subdirectory(test)
endif()

if(ENABLE_EXAMPLES)
  add_subdirectory(benchmarks)
endif()
