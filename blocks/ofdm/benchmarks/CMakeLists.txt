function(add_ofdm_benchmark BM_NAME)
  add_benchmark(${BM_NAME})
  target_link_libraries(
    ${BM_NAME}
    PRIVATE gnuradio-options
            gnuradio-core
            gr-ofdm
            gnuradio4::gr-testing
            ut
            ut-benchmark)
  target_compile_options(${BM_NAME} PRIVATE -O3)
endfunction()

add_ofdm_benchmark(bm_Ofdm)
