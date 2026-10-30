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
gr_digital_graph_test(graph_digital_delimiter_extractor_records graphs/delimiter-extractor-records.yaml)
gr_digital_graph_test(graph_digital_delimiter_extractor_refusals graphs/delimiter-extractor-refusals.yaml)
gr_digital_graph_test(graph_digital_delimiter_extractor_recovery graphs/delimiter-extractor-recovery.yaml)
gr_digital_graph_test(graph_digital_delimiter_extractor_triggers graphs/delimiter-extractor-triggers.yaml)
gr_digital_graph_test(graph_digital_delimiter_extractor_sequence graphs/delimiter-extractor-sequence.yaml)
gr_digital_graph_test(graph_digital_delimiter_extractor_events graphs/delimiter-extractor-events.yaml)
gr_digital_graph_test(graph_digital_delimiter_extractor_stuffing graphs/delimiter-extractor-stuffing.yaml)
gr_digital_graph_test(graph_digital_delimiter_extractor_crc graphs/delimiter-extractor-crc.yaml)
gr_digital_graph_test(graph_digital_delimiter_extractor_settings graphs/delimiter-extractor-settings.yaml)
gr_digital_graph_test(graph_digital_delimiter_framer_hdlc graphs/delimiter-framer-hdlc.yaml)
gr_digital_graph_test(graph_digital_delimiter_framer_loopback graphs/delimiter-framer-loopback.yaml)
gr_digital_graph_test(graph_digital_delimiter_framer_fcs graphs/delimiter-framer-fcs.yaml)
gr_digital_graph_test(graph_digital_delimiter_framer_ones graphs/delimiter-framer-ones.yaml)
gr_digital_graph_test(graph_digital_delimiter_framer_records graphs/delimiter-framer-records.yaml)
gr_digital_graph_test(graph_digital_delimiter_framer_settings graphs/delimiter-framer-settings.yaml)
gr_digital_graph_test(graph_digital_pam_slicer graphs/pam-slicer.yaml)
gr_digital_graph_test(graph_digital_pam_settings graphs/pam-settings.yaml)

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
gr_add_failing_graph_test(
  graph_digital_refused_extractor_character
  graphs/delimiter-extractor-settings.yaml
  "gr::digital::configureDelimiter: endDelimiter must be '0' and '1' characters only, got 'O' in \\\"110000O0\\\""
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_extractor.end_delimiter=\"110000O0\"")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_register
  graphs/delimiter-extractor-settings.yaml
  "gr::digital::configureDelimiter: endDelimiter is 65 bits. the match register is one 64-bit word"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "bit_extractor.end_delimiter=\"00000000000000000000000000000000000000000000000000000000000000000\"")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_whole_items
  graphs/delimiter-extractor-settings.yaml
  "gr::digital::configureDelimiter: endDelimiter is 7 bits, which is not a whole number of 8-bit items"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_extractor.end_delimiter=\"1100000\"")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_bits_per_item
  graphs/delimiter-extractor-settings.yaml
  "gr::digital::configureDelimiter: bitsPerItem must be in .1, 8., got 9"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_extractor.bits_per_item=!!uint32 9")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_frame_open
  graphs/delimiter-extractor-settings.yaml
  "frame_open must be 'delimiter', 'start' or 'immediate', got 'whenever'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "bit_extractor.end_delimiter=\"11000000\""
  "bit_extractor.frame_open=whenever")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_start_missing
  graphs/delimiter-extractor-settings.yaml
  "gr::digital::configureDelimiter: frameOpen 'start' needs a startDelimiter"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "bit_extractor.end_delimiter=\"11000000\""
  "bit_extractor.frame_open=start")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_start_unused
  graphs/delimiter-extractor-settings.yaml
  "gr::digital::configureDelimiter: frameOpen 'delimiter' consults no startDelimiter"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "bit_extractor.end_delimiter=\"11000000\""
  "bit_extractor.start_delimiter=\"00100100\"")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_transparency
  graphs/delimiter-extractor-settings.yaml
  "transparency must be 'none', 'bit_stuffing' or 'byte_escape', got 'magic'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "bit_extractor.end_delimiter=\"11000000\""
  "bit_extractor.transparency=magic")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_stuffing_outside
  graphs/delimiter-extractor-settings.yaml
  "stuff_after_ones and abort_ones belong to 'bit_stuffing' and transparency is 'none'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "bit_extractor.stuff_after_ones=!!uint32 4")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_escape_outside
  graphs/delimiter-extractor-settings.yaml
  "escape_item and escape_map belong to 'byte_escape' and transparency is 'none'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "bit_extractor.escape_item=!!uint32 219")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_stuffing_bytes
  graphs/delimiter-extractor-settings.yaml
  "gr::digital::configureDelimiter: bit stuffing is a bit-level code and needs bitsPerItem 1, got 8"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "bit_extractor.transparency=bit_stuffing"
  "bit_extractor.bits_per_item=!!uint32 8")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_escape_bits
  graphs/delimiter-extractor-settings.yaml
  "gr::digital::configureDelimiter: byte escaping needs a byte, so bitsPerItem must be 8, got 1"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "bit_extractor.end_delimiter=\"11000000\""
  "bit_extractor.transparency=byte_escape")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_stuff_after_one
  graphs/delimiter-extractor-settings.yaml
  "gr::digital::configureDelimiter: stuffAfterOnes must be in .2, 62., got 1"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "bit_extractor.transparency=bit_stuffing"
  "bit_extractor.stuff_after_ones=!!uint32 1")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_abort_early
  graphs/delimiter-extractor-settings.yaml
  "gr::digital::configureDelimiter: abortOnes 5 must exceed stuffAfterOnes 5"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "bit_extractor.transparency=bit_stuffing"
  "bit_extractor.abort_ones=!!uint32 5")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_escape_map_odd
  graphs/delimiter-extractor-settings.yaml
  "escape_map is a flat list of \\(escaped, original\\) pairs and must have an even length, got 1"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_extractor.transparency=byte_escape"
  "byte_extractor.escape_item=!!uint32 219"
  "byte_extractor.escape_map=!!uint32 [220]")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_escape_map_repeat
  graphs/delimiter-extractor-settings.yaml
  "gr::digital::configureDelimiter: the escape map lists the escaped value 220 twice"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_extractor.transparency=byte_escape"
  "byte_extractor.escape_item=!!uint32 219"
  "byte_extractor.escape_map=!!uint32 [220, 192, 220, 219]")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_escape_item_wide
  graphs/delimiter-extractor-settings.yaml
  "escape_item is an item value and must be in .0, 255., got 300"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_extractor.transparency=byte_escape"
  "byte_extractor.escape_item=!!uint32 300")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_floor_zero
  graphs/delimiter-extractor-settings.yaml
  "gr::digital::configureDelimiter: minPayloadItems must be at least 1"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_extractor.min_payload_items=!!uint32 0")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_floor_above
  graphs/delimiter-extractor-settings.yaml
  "gr::digital::configureDelimiter: minPayloadItems 5 exceeds maxPayloadItems 4"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_extractor.max_payload_items=!!uint32 4"
  "byte_extractor.min_payload_items=!!uint32 5")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_bound_extent
  graphs/delimiter-extractor-settings.yaml
  "gr::digital::configureDelimiter: maxPayloadItems is 2147483648. a record states its extent as a signed 32-bit count"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_extractor.max_payload_items=!!uint32 2147483648")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_pack_wide
  graphs/delimiter-extractor-settings.yaml
  "gr::digital::configureDelimiter: packBits must be 0, for no packing, or in .1, 8., got 9"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "bit_extractor.payload_pack_bits=!!uint32 9")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_pack_bytes
  graphs/delimiter-extractor-settings.yaml
  "gr::digital::configureDelimiter: the pack stage assembles bits into items and needs bitsPerItem 1, got 8"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_extractor.payload_pack_bits=!!uint32 8")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_destuffing
  graphs/delimiter-extractor-settings.yaml
  "gr::digital::configureDelimiter: coded data may carry runs of up to 5 consecutive ones and \\\"0111110\\\" carries at most 5"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "bit_extractor.end_delimiter=\"0111110\""
  "bit_extractor.transparency=bit_stuffing")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_soft_bytes
  graphs/delimiter-extractor-settings.yaml
  "a soft item carries one sliced bit and nothing else, so bits_per_item must be 1 on a floating-point stream, got 8"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "float_extractor.end_delimiter=\"11000000\""
  "float_extractor.bits_per_item=!!uint32 8")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_soft_pack
  graphs/delimiter-extractor-settings.yaml
  "the pack stage assembles bits into bytes and needs byte items. this stream is floating point"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "float_extractor.payload_pack_bits=!!uint32 8")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_required_both
  graphs/delimiter-extractor-settings.yaml
  "DelimiterExtractor[^\n]* throws: end_delimiter is required and has no default"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "bit_extractor.end_delimiter=\"\""
  "bit_extractor.max_payload_items=!!uint32 0")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_required_bound
  graphs/delimiter-extractor-settings.yaml
  "DelimiterExtractor[^\n]* throws: max_payload_items is required and has no default"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_extractor.max_payload_items=!!uint32 0")
gr_add_failing_graph_test(
  graph_digital_refused_extractor_required_delimiter
  graphs/delimiter-extractor-settings.yaml
  "DelimiterExtractor[^\n]* throws: end_delimiter is required and has no default"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "bit_extractor.end_delimiter=\"\"")
gr_add_failing_graph_test(
  graph_digital_refused_framer_introducer_unmapped
  graphs/delimiter-framer-settings.yaml
  "gr::digital::framerConfigure: the escape introducer is 219, which the escape map does not list as an original value"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "slip_framer.escape_map=!!uint32 [220, 192]")
gr_add_failing_graph_test(
  graph_digital_refused_framer_delimiter_unmapped
  graphs/delimiter-framer-settings.yaml
  "gr::digital::framerConfigure: an item of the end delimiter is 192, which the escape map does not list as an original value"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "slip_framer.escape_map=!!uint32 [221, 219]")
gr_add_failing_graph_test(
  graph_digital_refused_framer_pack_bytes
  graphs/delimiter-framer-settings.yaml
  "gr::digital::configureDelimiter: the pack stage assembles bits into items and needs bitsPerItem 1, got 8"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "slip_framer.payload_pack_bits=!!uint32 8")
gr_add_failing_graph_test(
  graph_digital_refused_framer_stuff_after_one
  graphs/delimiter-framer-settings.yaml
  "gr::digital::configureDelimiter: stuffAfterOnes must be in .2, 62., got 1"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "stuffed_framer.stuff_after_ones=!!uint32 1")
gr_add_failing_graph_test(
  graph_digital_refused_framer_destuffing
  graphs/delimiter-framer-settings.yaml
  "gr::digital::configureDelimiter: coded data may carry runs of up to 5 consecutive ones and \\\"0111110\\\" carries at most 5"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "stuffed_framer.end_delimiter=\"0111110\"")
gr_add_failing_graph_test(
  graph_digital_refused_framer_required_both
  graphs/delimiter-framer-settings.yaml
  "DelimiterFramer[^\n]* throws: end_delimiter is required and has no default"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_framer.end_delimiter=\"\""
  "byte_framer.bits_per_item=!!uint32 1"
  "byte_framer.max_payload_items=!!uint32 0")
gr_add_failing_graph_test(
  graph_digital_refused_framer_required_bound
  graphs/delimiter-framer-settings.yaml
  "DelimiterFramer[^\n]* throws: max_payload_items is required and has no default"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_framer.max_payload_items=!!uint32 0")
gr_add_failing_graph_test(
  graph_digital_refused_framer_required_delimiter
  graphs/delimiter-framer-settings.yaml
  "DelimiterFramer[^\n]* throws: end_delimiter is required and has no default"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_framer.end_delimiter=\"\""
  "byte_framer.bits_per_item=!!uint32 1")
gr_add_failing_graph_test(
  graph_digital_refused_slicer_labels
  graphs/pam-settings.yaml
  "PamSlicer[^\n]* throws: labels holds 2 entries for 4 levels. one per level rank"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "slicer_block.labels=!!uint32 [0, 1]")
gr_add_failing_graph_test(
  graph_digital_refused_slicer_odd_levels
  graphs/pam-settings.yaml
  "PamSlicer[^\n]* throws: n_levels must be even and at least 2, got 3"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "slicer_block.n_levels=!!uint32 3"
  "slicer_block.labels=!!uint32 [0, 1, 2]")
gr_add_failing_graph_test(
  graph_digital_refused_tracker_odd_levels
  graphs/pam-settings.yaml
  "LevelTracker[^\n]* throws: n_levels must be even and at least 2, got 3"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "tracker_block.n_levels=!!uint32 3")

# A record the block refuses, on the block's reason.
gr_add_failing_graph_test(
  graph_digital_refused_crc_multi_signal
  graphs/crc-multi-signal.yaml
  "CrcAppend[^\n]* reports an error on 'invokeProcessBulk': a CRC covers one flat byte array. this DataSet carries 2 signals"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>)

# The line a block prints at stop, for the counts no output record carries.
gr_digital_graph_test(
  graph_digital_counters_overrun
  graphs/delimiter-extractor-refusals.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::DelimiterExtractor 'overrun_extractor': frames emitted: 1, overrun frames: 1,")
gr_digital_graph_test(
  graph_digital_counters_survive
  graphs/delimiter-extractor-refusals.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::DelimiterExtractor 'survive_extractor': frames emitted: 1, overrun frames: 100,")
gr_digital_graph_test(
  graph_digital_counters_undersize
  graphs/delimiter-extractor-refusals.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::DelimiterExtractor 'undersize_extractor': undersize frames: 1(\n|, items)")
gr_digital_graph_test(
  graph_digital_counters_idle
  graphs/delimiter-extractor-refusals.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::DelimiterExtractor 'idle_extractor': frames emitted: 1, idle delimiters: 1")
gr_digital_graph_test(
  graph_digital_counters_unaligned
  graphs/delimiter-extractor-refusals.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::DelimiterExtractor 'unaligned_extractor': unaligned frames: 1,")
gr_digital_graph_test(
  graph_digital_counters_abort
  graphs/delimiter-extractor-recovery.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::DelimiterExtractor 'abort_extractor': frames emitted: 1, aborts: 1,")
gr_digital_graph_test(
  graph_digital_counters_patient
  graphs/delimiter-extractor-recovery.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::DelimiterExtractor 'patient_extractor': frames emitted: 2, items discarded")
gr_digital_graph_test(
  graph_digital_counters_restart
  graphs/delimiter-extractor-recovery.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::DelimiterExtractor 'restart_extractor': frames emitted: 1, restarts: 1,")
gr_digital_graph_test(
  graph_digital_counters_detector
  graphs/delimiter-extractor-triggers.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::DelimiterExtractor 'detector_extractor': frames emitted: 2, trigger resets: 1,")
gr_digital_graph_test(
  graph_digital_counters_mid_frame
  graphs/delimiter-extractor-triggers.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::DelimiterExtractor 'mid_frame_extractor': frames emitted: 1, trigger resets: 1,")
gr_digital_graph_test(
  graph_digital_counters_passive
  graphs/delimiter-extractor-triggers.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::DelimiterExtractor 'passive_extractor': frames emitted: 2, items discarded")
gr_digital_graph_test(
  graph_digital_counters_interior
  graphs/delimiter-extractor-events.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::DelimiterExtractor 'interior_extractor': [^\n]*, tags outside frames: 1, tags on removed items: 1"
)
gr_digital_graph_test(
  graph_digital_counters_edge
  graphs/delimiter-extractor-events.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::DelimiterExtractor 'edge_extractor': [^\n]*, dropped timing events: 1")
gr_digital_graph_test(
  graph_digital_counters_hdlc
  graphs/delimiter-framer-hdlc.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::DelimiterFramer 'hdlc_framer': records framed: 1, items emitted: 34, stuffed bits: 2\n")
gr_digital_graph_test(
  graph_digital_counters_drops
  graphs/delimiter-framer-records.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::DelimiterFramer 'drops_framer': records framed: 2, items emitted: 8, records refused: 2")
gr_digital_graph_test(
  graph_digital_counters_forgery
  graphs/delimiter-framer-records.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::DelimiterFramer 'forgery_framer': records framed: 1, items emitted: 7, forged delimiters: 2")
gr_digital_graph_test(
  graph_digital_counters_metadata
  graphs/delimiter-framer-records.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::DelimiterFramer 'metadata_framer': records framed: 3, items emitted: 13, escaped items: 2")
