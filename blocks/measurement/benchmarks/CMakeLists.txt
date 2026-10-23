function(add_measurement_benchmark BM_NAME)
  add_benchmark(${BM_NAME})
  target_link_libraries(
    ${BM_NAME}
    PRIVATE gnuradio-options
            gnuradio-core
            gr-measurement
            gnuradio4::gr-testing
            ut
            ut-benchmark)
  target_compile_options(${BM_NAME} PRIVATE -O3)
endfunction()

add_measurement_benchmark(bm_Autocorrelation)
add_measurement_benchmark(bm_Detectors)
add_measurement_benchmark(bm_Kurtosis)
add_measurement_benchmark(bm_PhaseUnwrap)
add_measurement_benchmark(bm_PolarizationCombiner)
add_measurement_benchmark(bm_SpectralEstimate)
add_measurement_benchmark(bm_SpectralKurtosis)
