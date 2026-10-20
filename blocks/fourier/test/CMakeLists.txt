add_ut_test(qa_fourier)
# SimdFFT's always_inline helpers need the inliner depth -O2 provides; at -O1 the unresolved indirect call is a hard
# error.
gr_pin_test_optimization(qa_fourier -O2)
target_link_libraries(qa_fourier PRIVATE gr-fourier gr-testing)

add_ut_test(qa_FrequencyCompressor)
target_compile_options(qa_FrequencyCompressor PRIVATE $<$<NOT:$<CONFIG:Debug>>:-O2>)
target_link_libraries(qa_FrequencyCompressor PRIVATE gr-fourier gr-testing)
