# Test translation units are dominated by the Block/Graph/Scheduler closure that the module object libraries already
# precompile. A precompiled header takes roughly as long to build as it saves across a handful of translation units, so
# one header per test target, and even one per test directory, would largely pay for itself and no more. The test tier
# shares a single header instead, held by an anchor target that carries the compile options add_ut_test gives its tests.
option(GR_QA_PCH "Share one precompiled header across the test executables" ON)

# Builds the anchor target that owns the shared precompiled header. It compiles one trivial translation unit, so the
# header is its only real output. Its options mirror what add_ut_test gives a test target, because GCC refuses a
# precompiled header built under different settings -- the optimization level among them -- and CMake pairs the header
# with -Winvalid-pch, so under -Werror the refusal stops the build.
#
# The anchor is an executable rather than an object library for the same reason: under CMAKE_POSITION_INDEPENDENT_CODE
# CMake gives a library -fPIC and an executable -fPIE, and the tests reading this header are executables. It is left out
# of the default targets, so the link its main() allows is one nothing normally asks for.
function(gr_qa_add_pch_anchor ANCHOR)
  set(_stub "${CMAKE_CURRENT_BINARY_DIR}/${ANCHOR}.cpp")
  file(
    CONFIGURE
    OUTPUT
    "${_stub}"
    CONTENT
    "// Anchor for the shared test precompiled header; the header is the point, not this unit.\nint main() { return 0; }\n"
  )
  add_executable(${ANCHOR} EXCLUDE_FROM_ALL "${_stub}")
  if(GR_QA_OPTIMIZATION_LEVEL)
    set_target_properties(${ANCHOR} PROPERTIES GR_QA_REDUCED_OPTIMIZATION ON)
    target_compile_options(${ANCHOR} PRIVATE $<$<NOT:$<CONFIG:Debug>>:${GR_QA_OPTIMIZATION_LEVEL}>)
  endif()
  if(CMAKE_GENERATOR MATCHES "Ninja")
    set_property(TARGET ${ANCHOR} PROPERTY JOB_POOL_COMPILE gr4_heavy_compile)
  endif()
  target_include_directories(${ANCHOR} PRIVATE ${CMAKE_BINARY_DIR}/include ${CMAKE_CURRENT_FUNCTION_LIST_DIR})
  target_link_libraries(
    ${ANCHOR}
    PRIVATE gnuradio-options
            gnuradio-core
            gnuradio-blocklib-core
            ut
            ${GR_TEST_HELPER_LIBRARIES})
  gnuradio4_precompile_headers(${ANCHOR})
endfunction()

if(GR_QA_PCH AND NOT CMAKE_DISABLE_PRECOMPILE_HEADERS)
  gr_qa_add_pch_anchor(gr_qa_pch)
endif()

# Options that only say how a precompiled header is produced. The anchor carries them because it is the target the
# header is built on; a test reusing that header must not be refused for lacking them.
set(GR_QA_PCH_ONLY_OPTIONS -fpch-instantiate-templates)

# Gives the shared header to the tests of the directory that has just been read, skipping any whose compile options no
# longer match the ones it was built under. A test adjusts its own options after add_ut_test returns, so the check has
# to be deferred to the end of the directory to see them; gr_pin_test_optimization is one such caller and a bare
# target_compile_options is another. The check is worth making rather than leaving to the compiler, because CMake passes
# -Winvalid-pch and this project builds -Werror, so a mismatch stops the build instead of merely compiling slower.
# Options a target inherits from a linked library are not visible here, and would still reach the compiler.
function(gr_qa_attach_shared_pch)
  get_target_property(_anchorOptions gr_qa_pch COMPILE_OPTIONS)
  list(REMOVE_ITEM _anchorOptions ${GR_QA_PCH_ONLY_OPTIONS})
  get_property(
    _candidates
    DIRECTORY
    PROPERTY GR_QA_PCH_CANDIDATES)
  foreach(_test IN LISTS _candidates)
    get_target_property(_testOptions ${_test} COMPILE_OPTIONS)
    list(REMOVE_ITEM _testOptions ${GR_QA_PCH_ONLY_OPTIONS})
    if("${_testOptions}" STREQUAL "${_anchorOptions}")
      set_target_properties(${_test} PROPERTIES PRECOMPILE_HEADERS_REUSE_FROM gr_qa_pch)
    endif()
  endforeach()
endfunction()

function(setup_test_no_asan TEST_NAME)
  target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_BINARY_DIR}/include ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(
    ${TEST_NAME}
    PRIVATE gnuradio-options
            gnuradio-core
            gnuradio-blocklib-core
            ut
            ${GR_TEST_HELPER_LIBRARIES})
  add_test(NAME ${TEST_NAME} COMMAND ${CMAKE_CROSSCOMPILING_EMULATOR} ${CMAKE_CURRENT_BINARY_DIR}/${TEST_NAME})
endfunction()

function(setup_test TEST_NAME)
  setup_test_no_asan(${TEST_NAME})
endfunction()

function(add_ut_test TEST_NAME)
  add_executable(${TEST_NAME} ${TEST_NAME}.cpp)
  if(GR_QA_OPTIMIZATION_LEVEL)
    # GCC's null-dereference analysis false-positives in libstdc++'s inlined string code below -O2; the warning battery
    # reads this property.
    set_target_properties(${TEST_NAME} PROPERTIES GR_QA_REDUCED_OPTIMIZATION ON)
    target_compile_options(${TEST_NAME} PRIVATE $<$<NOT:$<CONFIG:Debug>>:${GR_QA_OPTIMIZATION_LEVEL}>)
  endif()
  if(CMAKE_GENERATOR MATCHES "Ninja")
    # Test TUs instantiate whole graphs per file and are the build's memory-heavy class; the width-limited pool keeps
    # their peaks from stacking.
    set_property(TARGET ${TEST_NAME} PROPERTY JOB_POOL_COMPILE gr4_heavy_compile)
  endif()
  if(TARGET gr_qa_pch)
    get_property(
      _candidates
      DIRECTORY
      PROPERTY GR_QA_PCH_CANDIDATES)
    if(NOT _candidates)
      cmake_language(DEFER CALL gr_qa_attach_shared_pch)
    endif()
    set_property(
      DIRECTORY
      APPEND
      PROPERTY GR_QA_PCH_CANDIDATES ${TEST_NAME})
  endif()
  setup_test(${TEST_NAME})
  set_property(TEST ${TEST_NAME} PROPERTY ENVIRONMENT_MODIFICATION
                                          "GNURADIO4_PLUGIN_DIRECTORIES=set:${CMAKE_CURRENT_BINARY_DIR}/plugins")
  target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_CURRENT_FUNCTION_LIST_DIR})
endfunction()

# Pin a test above GR_QA_OPTIMIZATION_LEVEL and restore the diagnostics that only hold at the higher level: the warning
# battery reads GR_QA_REDUCED_OPTIMIZATION to decide whether GCC's null-dereference analysis is trustworthy for this
# target. Pinning also takes the target off the shared precompiled header, which was built at GR_QA_OPTIMIZATION_LEVEL:
# gr_qa_attach_shared_pch sees the changed options and leaves the target parsing the headers, which is cheaper than a
# header of its own for the few tests that pin a level.
function(gr_pin_test_optimization TEST_NAME LEVEL)
  target_compile_options(${TEST_NAME} PRIVATE $<$<NOT:$<CONFIG:Debug>>:${LEVEL}>)
  set_target_properties(${TEST_NAME} PROPERTIES GR_QA_REDUCED_OPTIMIZATION OFF)
endfunction()
