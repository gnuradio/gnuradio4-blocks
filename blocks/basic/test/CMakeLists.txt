function(add_basic_test TEST_NAME)
  add_ut_test(${TEST_NAME})
  target_link_libraries(${TEST_NAME} PRIVATE gr-basic gr-testing)
endfunction()

add_basic_test(qa_ClockSourceStop)
add_basic_test(qa_Converter)
add_basic_test(qa_Selector)
add_basic_test(qa_sources)
add_basic_test(qa_DataSink)
# the case ends at process exit, where a graph of static storage duration destroys its registered sinks
add_basic_test(qa_DataSinkExitOrder)
# qa_DataSinkRegistry looks up sinks that the shared block library creates. Under Emscripten that library is a static
# archive linked into the program, and the lookup has no second shared object to cross.
if(NOT EMSCRIPTEN)
  add_basic_test(qa_DataSinkRegistry)
  target_link_libraries(qa_DataSinkRegistry PRIVATE gnuradio4::GrBasicBlocksShared)
endif()
add_basic_test(qa_MessageDebug)
add_basic_test(qa_SampleDelay)
add_basic_test(qa_SkipHead)
add_basic_test(qa_TagDebug)
add_basic_test(qa_TriggerBlocks)
add_basic_test(qa_Throttle)
set_property(TEST qa_Throttle PROPERTY RESOURCE_LOCK rt)
# the rate case runs on the wall clock behind a Throttle; its tolerance is derived from the lateness it observes
add_basic_test(qa_RateProbe)
set_property(TEST qa_RateProbe PROPERTY RESOURCE_LOCK rt)

if(GR_ENABLE_BLOCK_REGISTRY AND INTERNAL_ENABLE_BLOCK_PLUGINS)
  add_basic_test(qa_BasicAvailableBlocks)
  target_link_libraries(qa_BasicAvailableBlocks PRIVATE gnuradio4::GrBasicBlocksShared)
endif()
add_basic_test(qa_StreamToDataSet)
add_basic_test(qa_DataSetToStream)
add_basic_test(qa_DataSetToPacket)
# the round-trip case runs the framing and CRC chain into the block, so it needs those headers too
target_link_libraries(qa_DataSetToPacket PRIVATE gr-digital)
add_basic_test(qa_PacketToDataSet)
# the chain case runs the converted records into CrcCheck, so it needs the digital headers too
target_link_libraries(qa_PacketToDataSet PRIVATE gr-digital)
add_basic_test(qa_SyncBlock)
add_basic_test(qa_VectorSink)
add_basic_test(qa_VectorSource)

if(PYTHON_AVAILABLE)
  add_basic_test(qa_PythonBlock)
  # the block library compiles PythonBlock<int32_t> and PythonBlock<float> in two units, and a case runs both
  target_link_libraries(qa_PythonBlock PRIVATE Python3::Python Python3::NumPy gnuradio4::GrBasicBlocksShared)
endif()

if(TARGET GrBasicBlocksShared)
  gr_add_graph_test(
    graph_basic_skip_head
    graphs/skip-head.yaml
    PLUGIN_DIRS
    "$<TARGET_FILE_DIR:GrBasicBlocksShared>")
endif()

gr_add_graph_test(
  graph_basic_rate_probe
  graphs/rate-probe.yaml
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrBasicBlocksShared>)

gr_add_graph_test(graph_basic_null_head_expect graphs/null-head-expect.yaml)
gr_add_graph_test(graph_basic_vector_source_expect graphs/vector-source-expect.yaml)
gr_add_graph_test(graph_basic_vector_source_sink graphs/vector-source-sink.yaml)

gr_add_failing_graph_test(graph_basic_dataset_records_key graphs/dataset-records-invalid.yaml
                          "records key 'x' is not a record index")
gr_add_failing_graph_test(
  graph_basic_dataset_records_gap
  graphs/dataset-records-invalid.yaml
  "records lists no record 1"
  SET
  "source.records={0: {}, 2: {}}")
gr_add_failing_graph_test(
  graph_basic_dataset_records_spelling
  graphs/dataset-records-invalid.yaml
  "records keys '0?1' and '0?1' both name 1"
  SET
  "source.records={1: {}, 01: {}, 0: {}}")
gr_add_failing_graph_test(
  graph_basic_dataset_records_entry
  graphs/dataset-records-invalid.yaml
  "records entry at 0 is 5, expected a map of fields"
  SET
  "source.records={0: 5}")
gr_add_failing_graph_test(
  graph_basic_dataset_records_field
  graphs/dataset-records-invalid.yaml
  "record 0 field 'signal_value' is not a record field"
  SET
  "source.records={0: {signal_value: [1]}}")
gr_add_failing_graph_test(
  graph_basic_dataset_records_meta_absent
  graphs/dataset-records-invalid.yaml
  "record 0 field 'meta_absent' is not a record field"
  SET
  "source.records={0: {meta_absent: [a]}}")
gr_add_failing_graph_test(
  graph_basic_dataset_records_values
  graphs/dataset-records-invalid.yaml
  "record 0 field 'signal_values' element 300 does not convert to uint8"
  SET
  "source.records={0: {signal_values: [300]}}")
gr_add_failing_graph_test(
  graph_basic_dataset_records_names
  graphs/dataset-records-invalid.yaml
  "record 0 field 'signal_names' holds 1, expected a string"
  SET
  "source.records={0: {signal_names: [1]}}")
gr_add_failing_graph_test(
  graph_basic_dataset_records_timestamp
  graphs/dataset-records-invalid.yaml
  "record 0 field 'timestamp' cannot parse 'abc' as an integer"
  SET
  "source.records={0: {timestamp: abc}}")
gr_add_failing_graph_test(
  graph_basic_dataset_records_meta
  graphs/dataset-records-invalid.yaml
  "record 0 field 'meta_information' is 5, expected a map or a list of maps"
  SET
  "source.records={0: {meta_information: 5}}")
gr_add_failing_graph_test(
  graph_basic_dataset_records_events
  graphs/dataset-records-invalid.yaml
  "record 0 field 'timing_events' list 0 key 'x' is not an event index"
  SET
  "source.records={0: {timing_events: [{x: {}}]}}")
gr_add_failing_graph_test(
  graph_basic_dataset_records_events_twice
  graphs/dataset-records-invalid.yaml
  "record 0 field 'timing_events' list 0 keys '0?2' and '0?2' both name 2"
  SET
  "source.records={0: {timing_events: [{2: {}, 02: {}}]}}")
gr_add_failing_graph_test(
  graph_basic_dataset_records_extents
  graphs/dataset-records-invalid.yaml
  "record 0 field 'extents' has no default for 3 values in 2 signals"
  SET
  "source.records={0: {signal_values: [1, 2, 3], signal_names: [a, b]}}")

# the graph names a basic block, and the case loads the basic plugin beside the testing one
if(TARGET GrBasicBlocksShared)
  gr_add_graph_test(
    graph_basic_TagDebug
    graphs/tag-debug.yaml
    PLUGIN_DIRS
    $<TARGET_FILE_DIR:GrBasicBlocksShared>)
endif()

gr_add_graph_test(
  graph_basic_converter_ports
  graphs/converter-ports.yaml
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrBasicBlocksShared>)
