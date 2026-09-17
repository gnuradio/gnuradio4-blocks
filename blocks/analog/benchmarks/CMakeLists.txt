function(add_analog_benchmark BM_NAME)
  add_benchmark(${BM_NAME})
  target_link_libraries(
    ${BM_NAME}
    PRIVATE gnuradio-options
            gnuradio-core
            gr-analog
            gnuradio4::gr-testing
            ut
            ut-benchmark)
  target_compile_options(${BM_NAME} PRIVATE -O3)
endfunction()

add_analog_benchmark(bm_Agc)
add_analog_benchmark(bm_FmEmphasis)
add_analog_benchmark(bm_QuadratureDemod)
