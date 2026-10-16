# The compile option add_ut_test gives a test target for GR_QA_OPTIMIZATION_LEVEL. gr_pin_test_optimization removes this
# exact item to leave a target at the build type's own level.
set(GR_QA_OPTIMIZATION_OPTION "$<$<NOT:$<CONFIG:Debug>>:${GR_QA_OPTIMIZATION_LEVEL}>")

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
    target_compile_options(${TEST_NAME} PRIVATE ${GR_QA_OPTIMIZATION_OPTION})
  endif()
  setup_test(${TEST_NAME})
  set_property(TEST ${TEST_NAME} PROPERTY ENVIRONMENT_MODIFICATION
                                          "GNURADIO4_PLUGIN_DIRECTORIES=set:${CMAKE_CURRENT_BINARY_DIR}/plugins")
  target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_CURRENT_FUNCTION_LIST_DIR})
endfunction()

# Pin a test above GR_QA_OPTIMIZATION_LEVEL and restore the diagnostics that only hold at the higher level: the warning
# battery reads GR_QA_REDUCED_OPTIMIZATION to decide whether GCC's null-dereference analysis is trustworthy for this
# target.
#
# LEVEL is a compiler option such as -O2, or BUILD_TYPE for the build type's own level: the target drops
# GR_QA_OPTIMIZATION_LEVEL and adds no level of its own. A cost test of a header-only algorithm pins BUILD_TYPE, so the
# figure it prints describes the algorithm as the build type compiles it; its bound is loose enough to hold at any
# level. A Debug build leaves every pinned test unoptimized.
function(gr_pin_test_optimization TEST_NAME LEVEL)
  if(LEVEL STREQUAL "BUILD_TYPE")
    get_target_property(_options ${TEST_NAME} COMPILE_OPTIONS)
    if(_options)
      list(REMOVE_ITEM _options "${GR_QA_OPTIMIZATION_OPTION}")
      set_target_properties(${TEST_NAME} PROPERTIES COMPILE_OPTIONS "${_options}")
    endif()
  else()
    target_compile_options(${TEST_NAME} PRIVATE $<$<NOT:$<CONFIG:Debug>>:${LEVEL}>)
  endif()
  set_target_properties(${TEST_NAME} PROPERTIES GR_QA_REDUCED_OPTIMIZATION OFF)
endfunction()
