# The benchmark harness, benchmark.hpp and gnuradio4::ut-benchmark, is core's: core exports it from a build with its
# tests enabled. Only the benchmarks use it, and the library's and the block families' benchmarks are built with
# ENABLE_EXAMPLES.
if(ENABLE_EXAMPLES AND NOT TARGET gnuradio4::ut-benchmark)
  message(FATAL_ERROR "ENABLE_EXAMPLES builds the benchmarks, which need gnuradio4::ut-benchmark from a gnuradio4 core "
                      "built with ENABLE_TESTING. Turn ENABLE_EXAMPLES off, or build against such a core.")
endif()

function(add_benchmark BM_NAME)
  add_executable(${BM_NAME} ${BM_NAME}.cpp)
  target_link_libraries(${BM_NAME} PRIVATE gnuradio-options ut gnuradio4::ut-benchmark)
endfunction()
