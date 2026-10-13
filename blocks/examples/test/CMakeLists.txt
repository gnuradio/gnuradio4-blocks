# The source-tree cases read the graphs from a header generated here, which embeds the text of every graph file. A graph
# that changes reruns the configure step and regenerates the header.
file(
  GLOB
  _graphFiles
  CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/../graphs/*.yaml")
list(SORT _graphFiles)
set(GR4_EXAMPLE_GRAPH_ENTRIES "")
foreach(_graphFile IN LISTS _graphFiles)
  set_property(
    DIRECTORY
    APPEND
    PROPERTY CMAKE_CONFIGURE_DEPENDS "${_graphFile}")
  get_filename_component(_graphName "${_graphFile}" NAME)
  file(READ "${_graphFile}" _graphText)
  string(FIND "${_graphText}" ")gr4yaml\"" _delimiterAt)
  if(NOT
     _delimiterAt
     EQUAL
     -1)
    message(FATAL_ERROR "${_graphFile} holds the raw-string delimiter )gr4yaml\"")
  endif()
  string(APPEND GR4_EXAMPLE_GRAPH_ENTRIES "    ExampleGraph{\"${_graphName}\", R\"gr4yaml(${_graphText})gr4yaml\"},\n")
endforeach()
configure_file(ExampleGraphs.hpp.in "${CMAKE_CURRENT_BINARY_DIR}/ExampleGraphs.hpp" @ONLY)

add_ut_test(qa_ExampleGraphs)
target_link_libraries(
  qa_ExampleGraphs
  PRIVATE gnuradio-core
          gnuradio4::gr-basic
          gnuradio4::GrAnalogBlocksShared
          gnuradio4::GrBasicBlocksShared
          gnuradio4::GrFilterBlocksShared
          gnuradio4::GrMathBlocksShared)

# The install-layout case reads the files where the install rule puts them: a setup test installs this directory's
# component into a prefix inside the build tree, and ctest runs it before the qa. The case is left out where the host
# file system is not readable (Emscripten) and where an absolute data directory puts the files outside that prefix.
if(EMSCRIPTEN)
  message(STATUS "example graphs: the install-layout case is left out under Emscripten")
elseif(IS_ABSOLUTE "${CMAKE_INSTALL_DATADIR}")
  message(STATUS "example graphs: the install-layout case is left out, CMAKE_INSTALL_DATADIR is absolute")
else()
  set(GR4_EXAMPLE_GRAPHS_TEST_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/install-layout")
  add_test(NAME qa_ExampleGraphsInstall COMMAND ${CMAKE_COMMAND} --install ${CMAKE_BINARY_DIR} --prefix
                                                ${GR4_EXAMPLE_GRAPHS_TEST_PREFIX} --component gr4-example-graphs)
  set_tests_properties(qa_ExampleGraphsInstall PROPERTIES FIXTURES_SETUP gr4_example_graphs_installed)
  set_tests_properties(qa_ExampleGraphsInstall PROPERTIES TIMEOUT 60)
  set_tests_properties(qa_ExampleGraphs PROPERTIES FIXTURES_REQUIRED gr4_example_graphs_installed)
  target_compile_definitions(
    qa_ExampleGraphs
    PRIVATE INSTALLED_EXAMPLE_GRAPHS_PATH="${GR4_EXAMPLE_GRAPHS_TEST_PREFIX}/${GR4_EXAMPLE_GRAPHS_INSTALL_DIR}")
endif()
set_tests_properties(qa_ExampleGraphs PROPERTIES TIMEOUT 60)
