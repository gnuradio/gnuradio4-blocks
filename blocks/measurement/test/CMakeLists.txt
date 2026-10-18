add_ut_test(qa_SpectralEstimate)
# A test that reaches SimdFFT builds at -O2. Its always_inline helpers need that inliner depth.
gr_pin_test_optimization(qa_SpectralEstimate -O2)
target_link_libraries(qa_SpectralEstimate PRIVATE gnuradio4::gr-measurement gnuradio4::gr-testing)

add_ut_test(qa_Detectors)
target_link_libraries(qa_Detectors PRIVATE gnuradio4::gr-measurement gnuradio4::gr-testing)

add_ut_test(qa_Kurtosis)
target_link_libraries(qa_Kurtosis PRIVATE gnuradio4::gr-measurement gnuradio4::gr-testing)

add_ut_test(qa_SpectralKurtosis)
gr_pin_test_optimization(qa_SpectralKurtosis -O2)
target_link_libraries(qa_SpectralKurtosis PRIVATE gnuradio4::gr-measurement gnuradio4::gr-testing)

add_ut_test(qa_PhaseUnwrap)
target_link_libraries(qa_PhaseUnwrap PRIVATE gnuradio4::gr-measurement gnuradio4::gr-testing)

add_ut_test(qa_PolarizationCombiner)
target_link_libraries(qa_PolarizationCombiner PRIVATE gnuradio4::gr-measurement gnuradio4::gr-testing)
