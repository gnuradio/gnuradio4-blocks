add_ut_test(qa_AccessCodeCorrelator)
target_link_libraries(qa_AccessCodeCorrelator PRIVATE gr-digital gr-basic gr-testing)

add_ut_test(qa_BitPacking)
target_link_libraries(qa_BitPacking PRIVATE gr-digital gr-testing)

add_ut_test(qa_ConstellationBlocks)
target_link_libraries(qa_ConstellationBlocks PRIVATE gr-digital gr-testing)

add_ut_test(qa_CpmModulate)
target_link_libraries(
  qa_CpmModulate
  PRIVATE gr-digital
          gr-analog
          gr-filter
          gr-sync
          gr-channel
          gr-testing)

add_ut_test(qa_CrcBlocks)
target_link_libraries(qa_CrcBlocks PRIVATE gr-digital gr-testing)

add_ut_test(qa_DelimiterExtractor)
target_link_libraries(qa_DelimiterExtractor PRIVATE gr-digital gr-testing)

add_ut_test(qa_DelimiterFramer)
target_link_libraries(qa_DelimiterFramer PRIVATE gr-digital gr-testing)

add_ut_test(qa_DifferentialCoding)
target_link_libraries(qa_DifferentialCoding PRIVATE gr-digital gr-testing)

add_ut_test(qa_LengthHeaderFramer)
target_link_libraries(qa_LengthHeaderFramer PRIVATE gr-digital gr-testing)

add_ut_test(qa_LinearEqualizer)
target_link_libraries(qa_LinearEqualizer PRIVATE gr-digital gr-channel gr-testing)

add_ut_test(qa_Manchester)
target_link_libraries(qa_Manchester PRIVATE gr-digital gr-testing)

add_ut_test(qa_ManchesterCombine)
target_link_libraries(qa_ManchesterCombine PRIVATE gr-digital gr-testing)

add_ut_test(qa_PacketFramer)
target_link_libraries(qa_PacketFramer PRIVATE gr-digital gr-testing)

add_ut_test(qa_PamSymbols)
target_link_libraries(qa_PamSymbols PRIVATE gr-digital gr-testing)

add_ut_test(qa_PreambleCorrelator)
target_link_libraries(qa_PreambleCorrelator PRIVATE gr-digital gr-testing)

add_ut_test(qa_Scrambler)
target_link_libraries(qa_Scrambler PRIVATE gr-digital gr-testing)

add_ut_test(qa_SignalQuality)
target_link_libraries(qa_SignalQuality PRIVATE gr-digital gr-channel gr-testing)

add_ut_test(qa_ModemDemo)
target_link_libraries(
  qa_ModemDemo
  PRIVATE gr-digital
          gr-channel
          gr-filter
          gr-sync
          gr-testing)

# The digital family's graph cases are run by rungraph. Each case loads the digital plugin beside the testing and basic
# plugins.
function(gr_digital_graph_test TEST_NAME GRAPH)
  gr_add_graph_test(
    ${TEST_NAME}
    ${GRAPH}
    PLUGIN_DIRS
    $<TARGET_FILE_DIR:GrDigitalBlocksShared>
    ${ARGN})
endfunction()

# Each graph runs to the end when every sink receives what it expects.
gr_digital_graph_test(graph_digital_crc_round_trip graphs/crc-round-trip.yaml)
gr_digital_graph_test(graph_digital_crc_append graphs/crc-append.yaml)
gr_digital_graph_test(graph_digital_crc_corruption graphs/crc-corruption.yaml)
gr_digital_graph_test(graph_digital_crc_guard graphs/crc-guard.yaml)
gr_digital_graph_test(graph_digital_crc_skip graphs/crc-skip.yaml)
gr_digital_graph_test(graph_digital_crc_records graphs/crc-records.yaml)
gr_digital_graph_test(graph_digital_crc_settings graphs/crc-settings.yaml)

# The settings graphs with settings changed that the block accepts.
gr_digital_graph_test(
  graph_digital_accepted_crc_append_width_8
  graphs/crc-settings.yaml
  SET
  "append_block.width=!!uint32 8"
  "append_block.poly=!!uint64 7")
gr_digital_graph_test(
  graph_digital_accepted_crc_append_width_16
  graphs/crc-settings.yaml
  SET
  "append_block.width=!!uint32 16"
  "append_block.poly=!!uint64 7")
gr_digital_graph_test(
  graph_digital_accepted_crc_append_width_24
  graphs/crc-settings.yaml
  SET
  "append_block.width=!!uint32 24"
  "append_block.poly=!!uint64 7")
gr_digital_graph_test(
  graph_digital_accepted_crc_append_width_32
  graphs/crc-settings.yaml
  SET
  "append_block.width=!!uint32 32"
  "append_block.poly=!!uint64 7")
gr_digital_graph_test(
  graph_digital_accepted_crc_append_width_64
  graphs/crc-settings.yaml
  SET
  "append_block.width=!!uint32 64"
  "append_block.poly=!!uint64 7")

# The settings graphs with settings changed that the block refuses, on the block's reason.
gr_add_failing_graph_test(
  graph_digital_refused_crc_append_width_0
  graphs/crc-settings.yaml
  "CrcAppend[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 0"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "append_block.width=!!uint32 0")
gr_add_failing_graph_test(
  graph_digital_refused_crc_append_width_1
  graphs/crc-settings.yaml
  "CrcAppend[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 1"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "append_block.width=!!uint32 1")
gr_add_failing_graph_test(
  graph_digital_refused_crc_append_width_2
  graphs/crc-settings.yaml
  "CrcAppend[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 2"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "append_block.width=!!uint32 2")
gr_add_failing_graph_test(
  graph_digital_refused_crc_append_width_3
  graphs/crc-settings.yaml
  "CrcAppend[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 3"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "append_block.width=!!uint32 3")
gr_add_failing_graph_test(
  graph_digital_refused_crc_append_width_5
  graphs/crc-settings.yaml
  "CrcAppend[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 5"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "append_block.width=!!uint32 5")
gr_add_failing_graph_test(
  graph_digital_refused_crc_append_width_6
  graphs/crc-settings.yaml
  "CrcAppend[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 6"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "append_block.width=!!uint32 6")
gr_add_failing_graph_test(
  graph_digital_refused_crc_append_width_12
  graphs/crc-settings.yaml
  "CrcAppend[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 12"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "append_block.width=!!uint32 12")
gr_add_failing_graph_test(
  graph_digital_refused_crc_append_width_65
  graphs/crc-settings.yaml
  "CrcAppend[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 65"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "append_block.width=!!uint32 65")
gr_add_failing_graph_test(
  graph_digital_refused_crc_append_width_100
  graphs/crc-settings.yaml
  "CrcAppend[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 100"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "append_block.width=!!uint32 100")
gr_add_failing_graph_test(
  graph_digital_refused_crc_append_width_1000
  graphs/crc-settings.yaml
  "CrcAppend[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 1000"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "append_block.width=!!uint32 1000")
gr_add_failing_graph_test(
  graph_digital_refused_crc_check_width_0
  graphs/crc-settings.yaml
  "CrcCheck[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 0"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "check_block.width=!!uint32 0")
gr_add_failing_graph_test(
  graph_digital_refused_crc_check_width_1
  graphs/crc-settings.yaml
  "CrcCheck[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 1"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "check_block.width=!!uint32 1")
gr_add_failing_graph_test(
  graph_digital_refused_crc_check_width_2
  graphs/crc-settings.yaml
  "CrcCheck[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 2"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "check_block.width=!!uint32 2")
gr_add_failing_graph_test(
  graph_digital_refused_crc_check_width_3
  graphs/crc-settings.yaml
  "CrcCheck[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 3"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "check_block.width=!!uint32 3")
gr_add_failing_graph_test(
  graph_digital_refused_crc_check_width_5
  graphs/crc-settings.yaml
  "CrcCheck[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 5"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "check_block.width=!!uint32 5")
gr_add_failing_graph_test(
  graph_digital_refused_crc_check_width_6
  graphs/crc-settings.yaml
  "CrcCheck[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 6"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "check_block.width=!!uint32 6")
gr_add_failing_graph_test(
  graph_digital_refused_crc_check_width_12
  graphs/crc-settings.yaml
  "CrcCheck[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 12"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "check_block.width=!!uint32 12")
gr_add_failing_graph_test(
  graph_digital_refused_crc_check_width_65
  graphs/crc-settings.yaml
  "CrcCheck[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 65"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "check_block.width=!!uint32 65")
gr_add_failing_graph_test(
  graph_digital_refused_crc_check_width_100
  graphs/crc-settings.yaml
  "CrcCheck[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 100"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "check_block.width=!!uint32 100")
gr_add_failing_graph_test(
  graph_digital_refused_crc_check_width_1000
  graphs/crc-settings.yaml
  "CrcCheck[^\n]* throws: width must be a multiple of 8 in .8, 64. because the block appends whole bytes, got 1000"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "check_block.width=!!uint32 1000")
gr_add_failing_graph_test(
  graph_digital_refused_crc_append_byte_order
  graphs/crc-settings.yaml
  "CrcAppend[^\n]* throws: crc_byte_order must be 'big' or 'little', got 'network'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "append_block.crc_byte_order=network")
gr_add_failing_graph_test(
  graph_digital_refused_crc_check_byte_order
  graphs/crc-settings.yaml
  "CrcCheck[^\n]* throws: crc_byte_order must be 'big' or 'little', got ''"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "check_block.crc_byte_order=\"\"")

# A record the block refuses, on the block's reason.
gr_add_failing_graph_test(
  graph_digital_refused_crc_multi_signal
  graphs/crc-multi-signal.yaml
  "CrcAppend[^\n]* reports an error on 'invokeProcessBulk': a CRC covers one flat byte array. this DataSet carries 2 signals"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>)
