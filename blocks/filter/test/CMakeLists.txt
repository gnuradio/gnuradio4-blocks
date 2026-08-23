add_ut_test(qa_filter)
target_link_libraries(qa_filter PRIVATE gr-filter gr-testing)

add_ut_test(qa_FrequencyEstimator)
# SimdFFT's always_inline helpers need the inliner depth -O2 provides; at -O1 the unresolved indirect call is a hard
# error.
gr_pin_test_optimization(qa_FrequencyEstimator -O2)
target_link_libraries(qa_FrequencyEstimator PRIVATE gr-filter gr-testing)

add_ut_test(qa_SvdDenoiser)
target_link_libraries(qa_SvdDenoiser PRIVATE gr-filter gr-testing)

add_ut_test(qa_SavitzkyGolayFilter)
target_link_libraries(qa_SavitzkyGolayFilter PRIVATE gr-filter gr-testing)
