add_ut_test(qa_NullSources)
target_link_libraries(qa_NullSources PRIVATE gr-basic gr-testing)
add_ut_test(qa_TestingFlowBlocks)
target_link_libraries(qa_TestingFlowBlocks PRIVATE gr-basic gr-testing)
add_ut_test(qa_SimCompute)
target_link_libraries(qa_SimCompute PRIVATE gr-basic gr-testing)

add_ut_test(qa_BuiltinTestBlocks)
target_link_libraries(qa_BuiltinTestBlocks PRIVATE gr-testing)

add_ut_test(qa_TestSpans)
target_link_libraries(qa_TestSpans PRIVATE gr-testing)

add_ut_test(qa_UI_Integration)
target_link_libraries(qa_UI_Integration PRIVATE gr-testing gr-basic gnuradio-algorithm)

add_ut_test(qa_GraphTestBlocks)
target_link_libraries(qa_GraphTestBlocks PRIVATE gr-basic gr-testing)

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
gr_add_graph_test(
  graph_testing_vector_expect_large_mismatch
  graphs/vector-expect-large-mismatch.yaml
  PASS_REGULAR_EXPRESSION
  "sample 0 is -1\\.7e\\+308, expected 1\\.7e\\+308")
gr_add_graph_test(graph_testing_vector_expect_large_mismatch_exit graphs/vector-expect-large-mismatch.yaml WILL_FAIL)
gr_add_graph_test(
  graph_testing_vector_expect_unit_mismatch
  graphs/vector-expect-large-mismatch.yaml
  PASS_REGULAR_EXPRESSION
  "sample 0 is -1, expected 1"
  SET
  "source.values=[-1.0]"
  "sink.expected=[1.0]")
gr_add_graph_test(
  graph_testing_vector_expect_unit_mismatch_exit
  graphs/vector-expect-large-mismatch.yaml
  WILL_FAIL
  SET
  "source.values=[-1.0]"
  "sink.expected=[1.0]")
gr_add_graph_test(graph_testing_vector_expect_tags_exact graphs/vector-expect-tags-exact.yaml)
gr_add_graph_test(
  graph_testing_vector_expect_tags_value_mismatch
  graphs/vector-expect-tags-value-mismatch.yaml
  PASS_REGULAR_EXPRESSION
  "tag 'level' at sample 1 is 3, expected 4")
gr_add_graph_test(graph_testing_vector_expect_tags_value_mismatch_exit graphs/vector-expect-tags-value-mismatch.yaml
                  WILL_FAIL)
gr_add_graph_test(
  graph_testing_vector_expect_tags_missing
  graphs/vector-expect-tags-missing.yaml
  PASS_REGULAR_EXPRESSION
  "tag 'mark' at sample 2 is missing, expected \"frame\"")
gr_add_graph_test(graph_testing_vector_expect_tags_missing_exit graphs/vector-expect-tags-missing.yaml WILL_FAIL)
gr_add_graph_test(
  graph_testing_vector_expect_tags_extra
  graphs/vector-expect-tags-extra.yaml
  PASS_REGULAR_EXPRESSION
  "tag 'level' at sample 1 is 3, expected no such tag")
gr_add_graph_test(graph_testing_vector_expect_tags_extra_exit graphs/vector-expect-tags-extra.yaml WILL_FAIL)

gr_add_graph_test(graph_testing_dataset_loopback graphs/dataset-loopback.yaml)
gr_add_graph_test(graph_testing_dataset_types graphs/dataset-types.yaml)
gr_add_failing_graph_test(graph_testing_dataset_value graphs/dataset-expect-mismatch.yaml
                          "record 0 field 'signal_values' value 1 is 2, expected 9")
gr_add_failing_graph_test(
  graph_testing_dataset_value_count
  graphs/dataset-expect-mismatch.yaml
  "record 0 field 'signal_values' holds 3 values, expected 2"
  SET
  "sink.records={0: {signal_values: [1, 2]}, 1: {}}")
gr_add_failing_graph_test(
  graph_testing_dataset_extents
  graphs/dataset-expect-mismatch.yaml
  "record 0 field 'extents' is \\[3\\], expected \\[4\\]"
  SET
  "sink.records={0: {extents: [4]}, 1: {}}")
gr_add_failing_graph_test(
  graph_testing_dataset_names
  graphs/dataset-expect-mismatch.yaml
  "record 0 field 'signal_names' is \\[payload\\], expected \\[other\\]"
  SET
  "sink.records={0: {signal_names: [other]}, 1: {}}")
gr_add_failing_graph_test(
  graph_testing_dataset_timestamp
  graphs/dataset-expect-mismatch.yaml
  "record 0 field 'timestamp' is 5, expected 6"
  SET
  "sink.records={0: {timestamp: 6}, 1: {}}")
gr_add_failing_graph_test(
  graph_testing_dataset_meta_count
  graphs/dataset-expect-mismatch.yaml
  "record 0 field 'meta_information' holds 1 maps, expected 2"
  SET
  "sink.records={0: {meta_information: [{}, {}]}, 1: {}}")
gr_add_failing_graph_test(
  graph_testing_dataset_meta_missing
  graphs/dataset-expect-mismatch.yaml
  "record 0 field 'meta_information' key 'gone' is missing, expected 1"
  SET
  "sink.records={0: {meta_information: {gone: 1}}, 1: {}}")
gr_add_failing_graph_test(
  graph_testing_dataset_meta_null
  graphs/dataset-expect-mismatch.yaml
  "record 0 field 'meta_information' key 'count' is 3, expected no such key"
  SET
  "sink.records={0: {meta_information: {count: null}}, 1: {}}")
gr_add_failing_graph_test(
  graph_testing_dataset_meta_absent
  graphs/dataset-expect-mismatch.yaml
  "record 0 field 'meta_absent' key 'who' is \"src\", expected no such key"
  SET
  "sink.records={0: {meta_absent: [who]}, 1: {}}")
gr_add_failing_graph_test(
  graph_testing_dataset_meta_exact
  graphs/dataset-expect-mismatch.yaml
  "record 0 field 'meta_information' key 'who' is \"src\", expected no such key"
  SET
  "sink.meta_exact=true"
  "sink.records={0: {meta_information: {count: 3}}, 1: {}}")
gr_add_failing_graph_test(
  graph_testing_dataset_meta_present
  graphs/dataset-expect-mismatch.yaml
  "record 0 field 'meta_present' key 'gone' is missing"
  SET
  "sink.records={0: {meta_present: [gone]}, 1: {}}")
gr_add_failing_graph_test(
  graph_testing_dataset_meta_present_none
  graphs/dataset-expect-mismatch.yaml
  "record 1 field 'meta_present' key 'count' is missing, the DataSet has no meta map"
  SET
  "sink.records={0: {}, 1: {meta_present: [count]}}")
gr_add_failing_graph_test(
  graph_testing_dataset_axis_names
  graphs/dataset-expect-mismatch.yaml
  "record 0 field 'axis_names' is \\[time\\], expected \\[other\\]"
  SET
  "sink.records={0: {axis_names: [other]}, 1: {}}")
gr_add_failing_graph_test(
  graph_testing_dataset_events
  graphs/dataset-expect-mismatch.yaml
  "record 0 field 'timing_events' is \\[\\[\\(2, {\"mark\": \"x\"}\\)\\]\\], expected \\[\\[\\(3, {\"mark\": \"x\"}\\)\\]\\]"
  SET
  "sink.records={0: {timing_events: [{3: {mark: x}}]}, 1: {}}")
gr_add_failing_graph_test(
  graph_testing_dataset_events_exact
  graphs/dataset-expect-mismatch.yaml
  "record 0 field 'timing_events' is \\[\\[\\(2, {\"mark\": \"x\"}\\)\\]\\], expected \\[\\[\\(2, {}\\)\\]\\]"
  SET
  "sink.meta_exact=true"
  "sink.records={0: {timing_events: [{2: {}}]}, 1: {}}")
gr_add_failing_graph_test(
  graph_testing_dataset_beyond
  graphs/dataset-expect-mismatch.yaml
  "record 1 is beyond the 1 expected records"
  SET
  "sink.records={0: {}}")
gr_add_failing_graph_test(
  graph_testing_dataset_too_few
  graphs/dataset-expect-mismatch.yaml
  "the stream ended after 2 DataSets, expected 3"
  SET
  "sink.records={0: {}, 1: {}, 2: {}}")
gr_add_failing_graph_test(
  graph_testing_dataset_records_field
  graphs/dataset-expect-mismatch.yaml
  "ExpectDataSetSink[^']*' reports an error on 'start': record 0 field 'bogus' is not a record field"
  SET
  "sink.records={0: {bogus: 1}, 1: {}}")
gr_add_failing_graph_test(
  graph_testing_dataset_records_meta_absent
  graphs/dataset-expect-mismatch.yaml
  "ExpectDataSetSink[^']*' reports an error on 'start': record 0 field 'meta_absent' is 5, expected a list of strings"
  SET
  "sink.records={0: {meta_absent: 5}, 1: {}}")
gr_add_failing_graph_test(graph_testing_dataset_meta_uint32 graphs/dataset-meta-types.yaml
                          "key 'count' is 3 \\(uint32\\), expected 4 \\(int64\\)")
gr_add_failing_graph_test(
  graph_testing_dataset_meta_uint64
  graphs/dataset-meta-types.yaml
  "key 'big' is 1099511627776 \\(uint64\\), expected 6 \\(int64\\)"
  SET
  "sink.records={0: {meta_information: {big: 6}}}")
gr_add_failing_graph_test(
  graph_testing_dataset_meta_string
  graphs/dataset-meta-types.yaml
  "key 'who' is \"src\" \\(string\\), expected 7 \\(int64\\)"
  SET
  "sink.records={0: {meta_information: {who: 7}}}")
gr_add_failing_graph_test(
  graph_testing_dataset_meta_bool
  graphs/dataset-meta-types.yaml
  "key 'flag' is true \\(bool\\), expected 1 \\(int64\\)"
  SET
  "sink.records={0: {meta_information: {flag: 1}}}")
