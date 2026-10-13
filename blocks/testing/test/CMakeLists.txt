add_ut_test(qa_NullSources)
target_link_libraries(qa_NullSources PRIVATE gr-testing)
add_ut_test(qa_TestingFlowBlocks)
target_link_libraries(qa_TestingFlowBlocks PRIVATE gr-testing)
add_ut_test(qa_SimCompute)
target_link_libraries(qa_SimCompute PRIVATE gr-testing)

add_ut_test(qa_BuiltinTestBlocks)
target_link_libraries(qa_BuiltinTestBlocks PRIVATE gr-testing)

add_ut_test(qa_TestSpans)
target_link_libraries(qa_TestSpans PRIVATE gr-testing)

add_ut_test(qa_UI_Integration)
target_link_libraries(qa_UI_Integration PRIVATE gr-testing gr-basic gnuradio-algorithm)

add_ut_test(qa_GraphTestBlocks)
target_link_libraries(qa_GraphTestBlocks PRIVATE gr-testing)

gr_add_graph_test(graph_testing_vector_expect graphs/vector-expect.yaml)
gr_add_graph_test(
  graph_testing_vector_expect_mismatch
  graphs/vector-expect-mismatch.yaml
  PASS_REGULAR_EXPRESSION
  "sample 2 is 3, expected 3\\.5")
gr_add_graph_test(graph_testing_vector_expect_tolerance graphs/vector-expect-tolerance.yaml)
gr_add_graph_test(
  graph_testing_vector_expect_tolerance_mismatch
  graphs/vector-expect-tolerance-mismatch.yaml
  PASS_REGULAR_EXPRESSION
  "sample 0 is 100, expected 102")
gr_add_graph_test(graph_testing_vector_expect_tags graphs/vector-expect-tags.yaml)
gr_add_graph_test(
  graph_testing_vector_expect_tags_mismatch
  graphs/vector-expect-tags-mismatch.yaml
  PASS_REGULAR_EXPRESSION
  "tags with key 'mark' at \\[1\\], expected at \\[2\\]")
