add_ut_test(qa_PpmFramer)
target_link_libraries(qa_PpmFramer PRIVATE gr-adsb gr-testing)

add_ut_test(qa_ModeSDecode)
target_link_libraries(qa_ModeSDecode PRIVATE gr-adsb gr-basic gr-testing)
