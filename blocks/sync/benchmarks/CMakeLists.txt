function(add_sync_benchmark BM_NAME)
  add_benchmark(${BM_NAME})
  target_link_libraries(
    ${BM_NAME}
    PRIVATE gnuradio-options
            gnuradio-core
            gr-sync
            gnuradio4::gr-testing
            ut
            ut-benchmark)
  target_compile_options(${BM_NAME} PRIVATE -O3)
endfunction()

add_sync_benchmark(bm_CostasLoop)
add_sync_benchmark(bm_FllBandEdge)
add_sync_benchmark(bm_Pll)
add_sync_benchmark(bm_PreambleTiming)
add_sync_benchmark(bm_SymbolSync)
