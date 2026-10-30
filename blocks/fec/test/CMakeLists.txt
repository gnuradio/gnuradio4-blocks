add_ut_test(qa_PunctureBlocks)
target_link_libraries(qa_PunctureBlocks PRIVATE gr-fec)

add_ut_test(qa_InterleaveBlocks)
target_link_libraries(qa_InterleaveBlocks PRIVATE gr-fec)

# The fec family's tests are graph files run by rungraph. Each case loads the fec plugin beside the testing and basic
# plugins.
function(gr_fec_graph_test TEST_NAME GRAPH)
  gr_add_graph_test(
    ${TEST_NAME}
    ${GRAPH}
    PLUGIN_DIRS
    $<TARGET_FILE_DIR:GrFecBlocksShared>
    ${ARGN})
endfunction()

# Each graph runs to the end when every sink receives what it expects.
gr_fec_graph_test(graph_fec_binary_round_trip graphs/fec-binary-round-trip.yaml)
gr_fec_graph_test(graph_fec_binary_anchors graphs/fec-binary-anchors.yaml)
gr_fec_graph_test(graph_fec_binary_errors graphs/fec-binary-errors.yaml)
gr_fec_graph_test(graph_fec_binary_records graphs/fec-binary-records.yaml)
gr_fec_graph_test(graph_fec_rs_round_trip graphs/fec-rs-round-trip.yaml)
gr_fec_graph_test(graph_fec_rs_errors graphs/fec-rs-errors.yaml)
gr_fec_graph_test(graph_fec_settings graphs/fec-settings.yaml)
gr_fec_graph_test(graph_fec_conv_encode graphs/conv-encode.yaml)
gr_fec_graph_test(graph_fec_conv_decode graphs/conv-decode.yaml)
gr_fec_graph_test(graph_fec_conv_records graphs/conv-records.yaml)
gr_fec_graph_test(graph_fec_conv_settings graphs/conv-settings.yaml)

# The settings graphs with one setting changed that the block accepts.
gr_fec_graph_test(
  graph_fec_accepted_rs_pad_last
  graphs/fec-settings.yaml
  SET
  "rs_decoder_block.roots=!!uint32 16"
  "rs_decoder_block.pad=!!uint32 46")
gr_fec_graph_test(
  graph_fec_accepted_rs_gf64_depth_7
  graphs/fec-settings.yaml
  SET
  "rs_decoder_block.interleave=!!uint32 7")
gr_fec_graph_test(
  graph_fec_accepted_ccsds_depth_8
  graphs/fec-settings.yaml
  SET
  "ccsds_encoder_block.interleave=!!uint32 8")
gr_fec_graph_test(
  graph_fec_accepted_ccsds_roots_agree
  graphs/fec-settings.yaml
  SET
  "ccsds_encoder_block.roots=!!uint32 32")
gr_fec_graph_test(
  graph_fec_accepted_ccsds_pad_last
  graphs/fec-settings.yaml
  SET
  "ccsds_decoder_block.pad=!!uint32 238")
gr_fec_graph_test(
  graph_fec_accepted_conv_ccsds_k7
  graphs/conv-settings.yaml
  SET
  "ccsds_encoder_block.constraint_length=!!uint32 7")
gr_fec_graph_test(
  graph_fec_accepted_conv_ccsds_invert_agree
  graphs/conv-settings.yaml
  SET
  "ccsds_encoder_block.invert_outputs=!!uint32 2")
gr_fec_graph_test(
  graph_fec_accepted_conv_soft_open
  graphs/conv-settings.yaml
  SET
  "soft_decoder_block.termination=open")

# The settings graphs with one setting changed that the block refuses, on the block's reason.
gr_add_failing_graph_test(
  graph_fec_refused_fec_code_empty
  graphs/fec-settings.yaml
  "code has no default and must be set to 'golay24'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "encoder_block.code=\"\"")
gr_add_failing_graph_test(
  graph_fec_refused_fec_code_unknown
  graphs/fec-settings.yaml
  "code must be 'golay24', [^\n]*, got 'golay25'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "decoder_block.code=golay25")
gr_add_failing_graph_test(
  graph_fec_refused_rs_roots
  graphs/fec-settings.yaml
  "roots must be 8, 12 or 16, [^\n]*, got 10"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "rs_encoder_block.roots=!!uint32 10")
gr_add_failing_graph_test(
  graph_fec_refused_rs_pad
  graphs/fec-settings.yaml
  "pad is the shortening in leading zero symbols and must be below 47, [^\n]*, got 47"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "rs_decoder_block.roots=!!uint32 16"
  "rs_decoder_block.pad=!!uint32 47")
gr_add_failing_graph_test(
  graph_fec_refused_rs_code_unknown
  graphs/fec-settings.yaml
  "code must be empty, 'ccsds_255_223' or 'ccsds_255_239', got 'rs_experimental'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "rs_encoder_block.code=rs_experimental")
gr_add_failing_graph_test(
  graph_fec_refused_rs_gf64_basis
  graphs/fec-settings.yaml
  "the GF\\(64\\) family has none. got 'dual'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "rs_encoder_block.basis=dual")
gr_add_failing_graph_test(
  graph_fec_refused_rs_gf64_depth
  graphs/fec-settings.yaml
  "interleave must be 1 to 8, got 9"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "rs_decoder_block.interleave=!!uint32 9")
gr_add_failing_graph_test(
  graph_fec_refused_ccsds_basis_empty
  graphs/fec-settings.yaml
  "basis has no default under a CCSDS profile"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "ccsds_encoder_block.basis=\"\"")
gr_add_failing_graph_test(
  graph_fec_refused_ccsds_depth_6
  graphs/fec-settings.yaml
  "interleave must be one of 4\\.3\\.5\\.1's depths 1, 2, 3, 4, 5 or 8, got 6"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "ccsds_encoder_block.interleave=!!uint32 6")
gr_add_failing_graph_test(
  graph_fec_refused_ccsds_depth_7
  graphs/fec-settings.yaml
  "interleave must be one of 4\\.3\\.5\\.1's depths 1, 2, 3, 4, 5 or 8, got 7"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "ccsds_encoder_block.interleave=!!uint32 7")
gr_add_failing_graph_test(
  graph_fec_refused_ccsds_roots
  graphs/fec-settings.yaml
  "code 'ccsds_255_223' carries 32 parity symbols and roots says 16"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "ccsds_encoder_block.roots=!!uint32 16")
gr_add_failing_graph_test(
  graph_fec_refused_ccsds_pad
  graphs/fec-settings.yaml
  "pad is the shortening in leading zero symbols and must be below 239, [^\n]*, got 239"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "ccsds_decoder_block.pad=!!uint32 239")
gr_add_failing_graph_test(
  graph_fec_refused_conv_code_empty
  graphs/conv-settings.yaml
  "constraint_length and polynomials have no defaults"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "encoder_block.constraint_length=!!uint32 0"
  "encoder_block.polynomials=!!uint32 []")
gr_add_failing_graph_test(
  graph_fec_refused_conv_k2
  graphs/conv-settings.yaml
  "constraint_length must be 3 to 9, [^\n]*. got constraint_length 2,"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "decoder_block.constraint_length=!!uint32 2"
  "decoder_block.polynomials=!!uint32 [3, 2]")
gr_add_failing_graph_test(
  graph_fec_refused_conv_k10
  graphs/conv-settings.yaml
  "constraint_length must be 3 to 9, [^\n]*. got constraint_length 10,"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "decoder_block.constraint_length=!!uint32 10"
  "decoder_block.polynomials=!!uint32 [617, 1003]")
gr_add_failing_graph_test(
  graph_fec_refused_conv_one_polynomial
  graphs/conv-settings.yaml
  "polynomials names one generator per coded bit and must hold 2 to 4 of them, got 1"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "encoder_block.polynomials=!!uint32 [121]")
gr_add_failing_graph_test(
  graph_fec_refused_conv_polynomial_twice
  graphs/conv-settings.yaml
  "every polynomial must be nonzero, no wider than constraint_length bits and named once[^\n]*. got constraint_length 7,"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "encoder_block.polynomials=!!uint32 [121, 121]")
gr_add_failing_graph_test(
  graph_fec_refused_conv_polynomial_wide
  graphs/conv-settings.yaml
  "every polynomial must be nonzero, no wider than constraint_length bits and named once[^\n]*. got constraint_length 7,"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "encoder_block.polynomials=!!uint32 [121, 219]")
gr_add_failing_graph_test(
  graph_fec_refused_conv_polynomial_zero
  graphs/conv-settings.yaml
  "every polynomial must be nonzero, no wider than constraint_length bits and named once[^\n]*. got constraint_length 7,"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "encoder_block.polynomials=!!uint32 [121, 0]")
gr_add_failing_graph_test(
  graph_fec_refused_conv_ccsds_k5
  graphs/conv-settings.yaml
  "code 'ccsds' supplies constraint_length 7 and the explicit setting says 5"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "ccsds_encoder_block.constraint_length=!!uint32 5")
gr_add_failing_graph_test(
  graph_fec_refused_conv_ccsds_polynomials
  graphs/conv-settings.yaml
  "code 'ccsds' supplies its own polynomials and the explicit setting disagrees"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "ccsds_encoder_block.polynomials=!!uint32 [121, 91]")
gr_add_failing_graph_test(
  graph_fec_refused_conv_ccsds_invert
  graphs/conv-settings.yaml
  "code 'ccsds' supplies invert_outputs 0b10 and the explicit setting says 0b1:"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "ccsds_encoder_block.invert_outputs=!!uint32 1")
gr_add_failing_graph_test(
  graph_fec_refused_conv_code_unknown
  graphs/conv-settings.yaml
  "code must name one of 'ccsds', [^\n]*, got 'voyager'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "decoder_block.code=voyager")
gr_add_failing_graph_test(
  graph_fec_refused_conv_termination
  graphs/conv-settings.yaml
  "termination must be 'terminated' or 'open', got 'running'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "decoder_block.termination=running")

# Each registration below matches the line a block prints at stop, for the totals of a run and for the counts no output
# record carries. A block leaves out a counter at zero, so an expression that ends at the line break also requires each
# counter it does not name to be zero.

# The binary codes. Each round trip decodes four codewords, three for the length-15 BCH codes, and corrects nothing.
# Each inside chain corrects t errors in each of three codewords, where t is the code's radius. Each beyond chain counts
# one uncorrectable codeword for the codes that detect its pattern, and none for the others.
set(_fecCodes
    golay24
    golay23
    golay18
    hamming15
    hamming10
    bch63
    bch15_11
    bch15_7
    bch15_5)
set(_fecRadii
    3
    3
    3
    1
    1
    11
    1
    2
    3)
set(_fecDetects
    1
    0
    1
    0
    1
    1
    0
    0
    1)
foreach(
  _code
  _radius
  _detects
  IN
  ZIP_LISTS
  _fecCodes
  _fecRadii
  _fecDetects)
  if(_code MATCHES "^bch15_")
    set(_roundTripWords 3)
    set(_beyondWords 3)
  else()
    set(_roundTripWords 4)
    set(_beyondWords 2)
  endif()
  math(EXPR _corrected "3 * ${_radius}")
  set(_refused "")
  if(_detects)
    set(_refused ", uncorrectable codewords: 1")
  endif()
  gr_fec_graph_test(
    graph_fec_counters_round_trip_${_code}
    graphs/fec-binary-round-trip.yaml
    PASS_REGULAR_EXPRESSION
    "FecDecode '${_code}_decoder': records: 1, codewords: ${_roundTripWords}\n")
  gr_fec_graph_test(
    graph_fec_counters_inside_${_code}
    graphs/fec-binary-errors.yaml
    PASS_REGULAR_EXPRESSION
    "FecDecode '${_code}_inside_decoder': records: 1, codewords: 3, corrected errors: ${_corrected}\n")
  gr_fec_graph_test(
    graph_fec_counters_beyond_${_code}
    graphs/fec-binary-errors.yaml
    PASS_REGULAR_EXPRESSION
    "FecDecode '${_code}_beyond_decoder': records: 1, codewords: ${_beyondWords}(, corrected errors: [0-9]+)?${_refused}\n"
  )
endforeach()

# The crossed chain decodes eight codewords and counts none uncorrectable.
gr_fec_graph_test(
  graph_fec_counters_crossed
  graphs/fec-binary-errors.yaml
  PASS_REGULAR_EXPRESSION
  "FecDecode 'crossed_decoder': records: 1, codewords: 8(, corrected errors: [0-9]+)?\n")

# The ordered chains carry three records of two golay24 codewords. The decoder corrects the one flipped bit of each
# codeword.
set(_orderedRecords 3)
math(EXPR _orderedWords "${_orderedRecords} * 2")
gr_fec_graph_test(
  graph_fec_counters_ordered_encode
  graphs/fec-binary-records.yaml
  PASS_REGULAR_EXPRESSION
  "FecEncode 'ordered_encode_coder': records: ${_orderedRecords}, codewords: ${_orderedWords}\n")
gr_fec_graph_test(
  graph_fec_counters_ordered_decode
  graphs/fec-binary-records.yaml
  PASS_REGULAR_EXPRESSION
  "FecDecode 'ordered_decode_decoder': records: ${_orderedRecords}, codewords: ${_orderedWords}, corrected errors: ${_orderedWords}\n"
)

# The gf64 Reed-Solomon codes. Each unshortened round trip decodes two codewords and corrects nothing. Each inside chain
# corrects half the root count in each of two codewords. The beyond chain counts its one codeword uncorrectable.
foreach(_roots 8 12 16)
  math(EXPR _corrected "2 * (${_roots} / 2)")
  gr_fec_graph_test(
    graph_fec_counters_rs_round_trip_${_roots}
    graphs/fec-rs-round-trip.yaml
    PASS_REGULAR_EXPRESSION
    "RsDecode 'gf64_${_roots}_0_decoder': records: 1, codewords: 2\n")
  gr_fec_graph_test(
    graph_fec_counters_rs_inside_${_roots}
    graphs/fec-rs-errors.yaml
    PASS_REGULAR_EXPRESSION
    "RsDecode 'inside_${_roots}_decoder': records: 1, codewords: 2, corrected errors: ${_corrected}\n")
endforeach()
gr_fec_graph_test(
  graph_fec_counters_rs_beyond
  graphs/fec-rs-errors.yaml
  PASS_REGULAR_EXPRESSION
  "RsDecode 'beyond_decoder': records: 1, codewords: 1, uncorrectable codewords: 1\n")

# The CCSDS arms. Each encodes one codeblock of I codewords, where I is the depth. Its burst chain corrects E errors in
# each codeword, where E is the profile's radius.
set(_ccsdsArms 255_223_conventional_4_200 255_223_dual_1_0 255_239_conventional_2_200)
set(_ccsdsRadii 16 16 8)
set(_ccsdsDepths 4 1 2)
foreach(
  _arm
  _radius
  _depth
  IN
  ZIP_LISTS
  _ccsdsArms
  _ccsdsRadii
  _ccsdsDepths)
  math(EXPR _corrected "${_radius} * ${_depth}")
  gr_fec_graph_test(
    graph_fec_counters_ccsds_encode_${_arm}
    graphs/fec-rs-round-trip.yaml
    PASS_REGULAR_EXPRESSION
    "RsEncode 'ccsds_${_arm}_coder': records: 1, codewords: ${_depth}\n")
  gr_fec_graph_test(
    graph_fec_counters_ccsds_burst_${_arm}
    graphs/fec-rs-errors.yaml
    PASS_REGULAR_EXPRESSION
    "RsDecode 'burst_${_arm}_decoder': records: 1, codewords: ${_depth}, corrected errors: ${_corrected}\n")
endforeach()

# The refused records, the shortened gf64 round trips and the pad corruption.
gr_fec_graph_test(
  graph_fec_counters_misaligned_golay24
  graphs/fec-binary-records.yaml
  PASS_REGULAR_EXPRESSION
  "FecEncode 'misaligned_golay24_coder': records: 1, codewords: 2, records refused: 2")
gr_fec_graph_test(
  graph_fec_counters_misaligned_bch15_11
  graphs/fec-binary-records.yaml
  PASS_REGULAR_EXPRESSION
  "FecEncode 'misaligned_bch15_11_coder': records: 1, codewords: 2, records refused: 2")
gr_fec_graph_test(
  graph_fec_counters_truncated
  graphs/fec-binary-records.yaml
  PASS_REGULAR_EXPRESSION
  "FecDecode 'truncated_decoder': records: 1, codewords: 1, records refused: 1")
gr_fec_graph_test(
  graph_fec_counters_shortened_8
  graphs/fec-rs-round-trip.yaml
  PASS_REGULAR_EXPRESSION
  "RsDecode 'gf64_8_39_decoder': records: 1, codewords: 2\n")
gr_fec_graph_test(
  graph_fec_counters_shortened_12
  graphs/fec-rs-round-trip.yaml
  PASS_REGULAR_EXPRESSION
  "RsDecode 'gf64_12_39_decoder': records: 1, codewords: 2\n")
gr_fec_graph_test(
  graph_fec_counters_shortened_16
  graphs/fec-rs-round-trip.yaml
  PASS_REGULAR_EXPRESSION
  "RsDecode 'gf64_16_27_decoder': records: 1, codewords: 2\n")
gr_fec_graph_test(
  graph_fec_counters_pad
  graphs/fec-rs-errors.yaml
  PASS_REGULAR_EXPRESSION
  "RsDecode 'pad_decoder': records: 1, codewords: 1, corrected errors: 1, uncorrectable codewords: 1, pad corrupted: 1")
gr_fec_graph_test(
  graph_fec_counters_rs_truncated
  graphs/fec-rs-errors.yaml
  PASS_REGULAR_EXPRESSION
  "RsDecode 'truncated_decoder': records: 1, codewords: 1, records refused: 1")
# The convolutional code. The hard and soft round trips each decode one frame of 100 information bits and correct
# nothing. The ordered chains carry three frames of 100 bits, and the decoder corrects the three flipped bits of each.
gr_fec_graph_test(
  graph_fec_counters_conv_hard
  graphs/conv-decode.yaml
  PASS_REGULAR_EXPRESSION
  "ViterbiDecode 'hard_decoder': frames: 1, information bits: 100\n")
gr_fec_graph_test(
  graph_fec_counters_conv_soft
  graphs/conv-decode.yaml
  PASS_REGULAR_EXPRESSION
  "ViterbiDecodeSoft 'soft_decoder': frames: 1, information bits: 100\n")
set(_orderedFrames 3)
math(EXPR _orderedBits "${_orderedFrames} * 100")
math(EXPR _orderedCorrected "${_orderedFrames} * 3")
gr_fec_graph_test(
  graph_fec_counters_conv_ordered_encode
  graphs/conv-records.yaml
  PASS_REGULAR_EXPRESSION
  "ConvEncode 'ordered_encode_coder': frames: ${_orderedFrames}, information bits: ${_orderedBits}\n")
gr_fec_graph_test(
  graph_fec_counters_conv_ordered_decode
  graphs/conv-records.yaml
  PASS_REGULAR_EXPRESSION
  "ViterbiDecode 'ordered_decode_decoder': frames: ${_orderedFrames}, information bits: ${_orderedBits}, corrected errors: ${_orderedCorrected}\n"
)
gr_fec_graph_test(
  graph_fec_counters_refused_encode
  graphs/conv-records.yaml
  PASS_REGULAR_EXPRESSION
  "ConvEncode 'refused_encode_coder': frames: 1, information bits: 50, records refused: 1")
gr_fec_graph_test(
  graph_fec_counters_refused_hard
  graphs/conv-records.yaml
  PASS_REGULAR_EXPRESSION
  "ViterbiDecode 'refused_hard_decoder': frames: 1, information bits: 20, records refused: 2")
gr_fec_graph_test(
  graph_fec_counters_refused_soft
  graphs/conv-records.yaml
  PASS_REGULAR_EXPRESSION
  "ViterbiDecodeSoft 'refused_soft_decoder': frames: 1, information bits: 20, records refused: 1")
gr_fec_graph_test(
  graph_fec_counters_refused_step
  graphs/conv-records.yaml
  PASS_REGULAR_EXPRESSION
  "ViterbiDecode 'refused_step_decoder': records refused: 1")
