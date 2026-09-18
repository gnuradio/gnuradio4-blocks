add_ut_test(qa_Audio)
target_link_libraries(qa_Audio PRIVATE gr-audio gr-testing httplib::httplib)

# The exported interface is only ever read from an install, so these legs need one; GR4_INSTALLED_PREFIX names a prefix
# this tree has already been installed into and they are registered only where one is named. --fresh keeps a cache left
# by an earlier run from answering for the install as it is now.
#
# Two steps, because the halves of an exported interface fail in different places. A target name the installed config
# file does not define ends the configure; an include directory the interface never carried is not missed until a
# translation unit asks for a header, which is what the second step compiles. ctest runs the configure for whoever
# selects the compile and does not run the compile when the configure failed.
if(GR4_EXPORT_NEEDS_LIBSOUNDIO AND GR4_INSTALLED_PREFIX)
  add_test(
    NAME qa_AudioExportedTarget
    COMMAND
      ${CMAKE_COMMAND} --fresh -S ${CMAKE_CURRENT_SOURCE_DIR}/consumer -B ${CMAKE_CURRENT_BINARY_DIR}/consumer -G
      ${CMAKE_GENERATOR} -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_PREFIX_PATH=${GR4_INSTALLED_PREFIX})
  set_tests_properties(qa_AudioExportedTarget PROPERTIES TIMEOUT 120)
  set_tests_properties(qa_AudioExportedTarget PROPERTIES FIXTURES_SETUP gr4_audio_consumer)

  add_test(NAME qa_AudioExportedInclude COMMAND ${CMAKE_COMMAND} --build ${CMAKE_CURRENT_BINARY_DIR}/consumer)
  # one translation unit of framework templates, which is minutes on a slow runner and seconds on a warm one
  set_tests_properties(qa_AudioExportedInclude PROPERTIES TIMEOUT 600)
  set_tests_properties(qa_AudioExportedInclude PROPERTIES FIXTURES_REQUIRED gr4_audio_consumer)
endif()
