if(NOT EMSCRIPTEN AND TARGET gnuradio4::GrBasicBlocksShared)
  set(_gr4_address_sanitizer_enabled OFF)
  if(CMAKE_CXX_FLAGS MATCHES "(^| )-fsanitize=address($| )"
     OR CMAKE_EXE_LINKER_FLAGS MATCHES "(^| )-fsanitize=address($| )"
     OR CMAKE_SHARED_LINKER_FLAGS MATCHES "(^| )-fsanitize=address($| )")
    set(_gr4_address_sanitizer_enabled ON)
  endif()

  if(NOT _gr4_address_sanitizer_enabled)
    add_ut_test(qa_KnownSharedLibBlocks)
    set(_known_shared_lib_targets
        gnuradio-core
        gnuradio4::GrTestingBlocksShared
        gnuradio4::GrFileIoBlocksShared
        gnuradio4::GrBasicBlocksShared
        gnuradio4::GrElectricalBlocksShared
        gnuradio4::GrHttpBlocksShared
        gnuradio4::GrFilterBlocksShared
        gnuradio4::GrFourierBlocksShared)

    if(TARGET gnuradio4::GrAudioBlocksShared)
      list(APPEND _known_shared_lib_targets gnuradio4::GrAudioBlocksShared)
    endif()

    target_link_libraries(qa_KnownSharedLibBlocks
                          PRIVATE ${_known_shared_lib_targets})
  endif()

  add_executable(qa_apptest_KnownStaticLibBlocks
                 qa_apptest_KnownStaticLibBlocks.cpp)
  setup_test(qa_apptest_KnownStaticLibBlocks)
  target_link_libraries(qa_apptest_KnownStaticLibBlocks
                        PRIVATE gnuradio-core gnuradio4::GrBasicBlocksStatic)

  add_executable(qa_apptest_LoadingPlainBlocklibs
                 qa_apptest_LoadingPlainBlocklibs.cpp)
  setup_test(qa_apptest_LoadingPlainBlocklibs)
  target_link_libraries(qa_apptest_LoadingPlainBlocklibs
                        PRIVATE gnuradio-core gnuradio-blocklib-core)
  set_property(
    TEST qa_apptest_LoadingPlainBlocklibs
    PROPERTY
      ENVIRONMENT_MODIFICATION
      "GNURADIO4_PLUGIN_DIRECTORIES=set:${CMAKE_CURRENT_BINARY_DIR}/../../../blocks/basic"
  )

endif()
