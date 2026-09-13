set(GR4_ZEROMQ_EXAMPLES
    zmq_loopback
    zmq_push_pull
    zmq_pub_sub
    zmq_req_rep
    zmq_pmt_payload)

foreach(example ${GR4_ZEROMQ_EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} PRIVATE gnuradio4::gnuradio-core gnuradio4::gnuradio-blocklib-core
                                           gnuradio4::gr-zeromq)
endforeach()

install(TARGETS ${GR4_ZEROMQ_EXAMPLES} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(PROGRAMS gr3_zmq_tag_loopback.py DESTINATION ${CMAKE_INSTALL_BINDIR})
