# the socket transports need no message library; the ZMQ suite and the two-process acceptance pair build only with the
# dependency they exercise
add_ut_test(qa_SocketPacketIO)
target_link_libraries(qa_SocketPacketIO PRIVATE gr-network gr-testing)
set_tests_properties(qa_SocketPacketIO PROPERTIES TIMEOUT 180)

add_ut_test(qa_TcpByteIo)
target_link_libraries(qa_TcpByteIo PRIVATE gr-network gr-testing)
# a passing run is seconds; the ceiling has to clear the sum of the watchdog deadlines a wholly broken transport would
# spend instead, which is what the socket suites next door are set for
set_tests_properties(qa_TcpByteIo PROPERTIES TIMEOUT 180)

if(GR4_NETWORK_HAVE_LIBZMQ)
  add_ut_test(qa_ZmqPacketIO)
  target_link_libraries(qa_ZmqPacketIO PRIVATE gr-network gr-testing)

  add_ut_test(qa_ZmqStreamIO)
  target_link_libraries(qa_ZmqStreamIO PRIVATE gr-network gr-testing)
  set_tests_properties(qa_ZmqStreamIO PROPERTIES TIMEOUT 120)

  add_ut_test(qa_StreamPacketIO)
  target_link_libraries(qa_StreamPacketIO PRIVATE gr-network gr-testing)
  set_tests_properties(qa_StreamPacketIO PROPERTIES TIMEOUT 120)

  add_ut_test(qa_MessagePacketIO)
  target_link_libraries(qa_MessagePacketIO PRIVATE gr-network gr-testing)
  set_tests_properties(qa_MessagePacketIO PROPERTIES TIMEOUT 120)

  # the example graphs are loaded by id through the global registry, so the shared library of every family they name has
  # to be loaded for those ids to resolve
  add_ut_test(qa_StreamPacketGraphs)
  target_link_libraries(
    qa_StreamPacketGraphs
    PRIVATE gnuradio-core
            gnuradio4::gr-basic
            gnuradio4::GrBasicBlocksShared
            gnuradio4::GrNetworkBlocksShared)
  target_compile_definitions(qa_StreamPacketGraphs
                             PRIVATE EXAMPLE_GRAPHS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../examples")
  set_tests_properties(qa_StreamPacketGraphs PROPERTIES TIMEOUT 120)

  # the acceptance driver launches the two executables in ../src as real, separate processes, so it is told where they
  # were built and is rebuilt whenever they are
  add_ut_test(qa_PacketLink)
  target_link_libraries(qa_PacketLink PRIVATE gr-network)
  add_dependencies(qa_PacketLink packet_link_tx packet_link_rx)
  target_compile_definitions(qa_PacketLink PRIVATE GR4_PACKET_LINK_TX="$<TARGET_FILE:packet_link_tx>"
                                                   GR4_PACKET_LINK_RX="$<TARGET_FILE:packet_link_rx>")
  set_tests_properties(qa_PacketLink PROPERTIES TIMEOUT 120)
endif()
