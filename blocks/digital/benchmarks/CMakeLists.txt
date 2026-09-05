function(add_digital_benchmark BM_NAME)
  add_benchmark(${BM_NAME})
  target_link_libraries(
    ${BM_NAME}
    PRIVATE gnuradio-options
            gnuradio-core
            gr-digital
            gnuradio4::gr-testing
            ut
            ut-benchmark)
  target_compile_options(${BM_NAME} PRIVATE -O3)
endfunction()

add_digital_benchmark(bm_BitPacking)
add_digital_benchmark(bm_ConstellationBlocks)
add_digital_benchmark(bm_CpmModulate)
add_digital_benchmark(bm_DifferentialCoding)
add_digital_benchmark(bm_LinearEqualizer)
add_digital_benchmark(bm_Scrambler)
add_digital_benchmark(bm_SignalQuality)
