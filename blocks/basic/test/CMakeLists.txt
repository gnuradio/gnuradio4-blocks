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
