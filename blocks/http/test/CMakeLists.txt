if(NOT APPLE)
  add_ut_test(qa_HttpBlock)
  target_link_libraries(qa_HttpBlock PRIVATE gr-http gr-testing httplib::httplib)
  # qa_FileIo also serves HTTP on port 8080. The lock keeps the two tests apart.
  set_tests_properties(qa_HttpBlock PROPERTIES RESOURCE_LOCK http_port_8080)

  if(EMSCRIPTEN)
    target_link_options(
      qa_HttpBlock
      PRIVATE
      --pre-js=${CMAKE_CURRENT_SOURCE_DIR}/pre.js
      --emrun)
  endif()
endif()
