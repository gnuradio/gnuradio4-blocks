if(NOT TARGET ut-benchmark)
  add_library(ut-benchmark INTERFACE benchmark.hpp)
  if(NOT TARGET gnuradio4::ut-benchmark)
    add_library(gnuradio4::ut-benchmark ALIAS ut-benchmark)
  endif()
  target_include_directories(ut-benchmark INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  target_link_libraries(ut-benchmark INTERFACE ut)
  # benchmark.hpp includes Boost.UT as a header, so whoever links this needs the module form switched off as well
  target_compile_definitions(ut-benchmark INTERFACE BOOST_UT_DISABLE_MODULE)
  # the installed gr-testing links this target, so when core did not export one it is installed with the blocks
  install(TARGETS ut-benchmark EXPORT gnuradio4Targets)
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/benchmark.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()

function(add_benchmark BM_NAME)
  add_executable(${BM_NAME} ${BM_NAME}.cpp)
  target_link_libraries(${BM_NAME} PRIVATE gnuradio-options ut gnuradio4::ut-benchmark)
endfunction()

add_benchmark(bm_example)
