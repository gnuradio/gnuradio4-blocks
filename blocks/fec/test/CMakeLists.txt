add_ut_test(qa_FecBlocks)
target_link_libraries(qa_FecBlocks PRIVATE gr-fec)

add_ut_test(qa_ConvBlocks)
target_link_libraries(qa_ConvBlocks PRIVATE gr-fec)

add_ut_test(qa_PunctureBlocks)
target_link_libraries(qa_PunctureBlocks PRIVATE gr-fec)

add_ut_test(qa_InterleaveBlocks)
target_link_libraries(qa_InterleaveBlocks PRIVATE gr-fec)
