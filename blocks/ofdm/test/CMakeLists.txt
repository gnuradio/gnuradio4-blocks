add_ut_test(qa_OfdmCarrierAllocator)
target_link_libraries(qa_OfdmCarrierAllocator PRIVATE gnuradio4::gr-ofdm gnuradio4::gr-testing)

add_ut_test(qa_OfdmCyclicPrefix)
# Reaches SimdFFT, whose always_inline helpers need the inliner depth -O2 provides.
gr_pin_test_optimization(qa_OfdmCyclicPrefix -O2)
target_link_libraries(qa_OfdmCyclicPrefix PRIVATE gnuradio4::gr-ofdm gnuradio4::gr-testing)

add_ut_test(qa_OfdmSchmidlCoxSync)
# Reaches SimdFFT and the phasor, whose always_inline helpers need the inliner depth -O2 provides.
gr_pin_test_optimization(qa_OfdmSchmidlCoxSync -O2)
target_link_libraries(qa_OfdmSchmidlCoxSync PRIVATE gnuradio4::gr-ofdm gnuradio4::gr-channel gnuradio4::gr-testing)

add_ut_test(qa_OfdmChannelEqualizer)
# Reaches SimdFFT, whose always_inline helpers need the inliner depth -O2 provides.
gr_pin_test_optimization(qa_OfdmChannelEqualizer -O2)
target_link_libraries(
  qa_OfdmChannelEqualizer
  PRIVATE gnuradio4::gr-ofdm
          gnuradio4::gr-channel
          gnuradio4::gr-digital
          gnuradio4::gr-filter
          gnuradio4::gr-testing)
