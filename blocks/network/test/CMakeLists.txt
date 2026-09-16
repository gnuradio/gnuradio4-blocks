# the socket transports need no message library; the ZMQ suite and the two-process acceptance pair build only with the
# dependency they exercise
add_ut_test(qa_SocketPacketIO)
target_link_libraries(qa_SocketPacketIO PRIVATE gr-network gr-testing)
set_tests_properties(qa_SocketPacketIO PROPERTIES TIMEOUT 180)

if(GR4_NETWORK_HAVE_LIBZMQ)
  add_ut_test(qa_ZmqPacketIO)
  target_link_libraries(qa_ZmqPacketIO PRIVATE gr-network gr-testing)

  # the acceptance driver launches the two executables in ../src as real, separate processes, so it is told where they
  # were built and is rebuilt whenever they are
endif()
