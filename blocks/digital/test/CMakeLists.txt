add_ut_test(qa_ConstellationBlocks)
target_link_libraries(qa_ConstellationBlocks PRIVATE gr-digital gr-testing)

add_ut_test(qa_DifferentialCoding)
target_link_libraries(qa_DifferentialCoding PRIVATE gr-digital gr-testing)
