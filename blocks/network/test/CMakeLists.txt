if(GR4_NETWORK_HAVE_LIBZMQ)
  add_ut_test(qa_ZmqPacketIO)
  target_link_libraries(qa_ZmqPacketIO PRIVATE gr-network gr-testing)
endif()
