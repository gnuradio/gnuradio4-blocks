add_ut_test(qa_SpectralEstimate)
# A test that reaches SimdFFT builds at -O2. Its always_inline helpers need that inliner depth.
gr_pin_test_optimization(qa_SpectralEstimate -O2)
target_link_libraries(qa_SpectralEstimate PRIVATE gnuradio4::gr-measurement gnuradio4::gr-testing)
