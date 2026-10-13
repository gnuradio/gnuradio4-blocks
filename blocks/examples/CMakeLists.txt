# The example graphs are data: files in the GRC YAML form that gr::loadGrc reads. They install beside the other example
# graphs, so a graph editor or a host application pointed at the data directory finds them. The graphs are not compiled.
set(GR4_EXAMPLE_GRAPHS_INSTALL_DIR "${CMAKE_INSTALL_DATADIR}/gnuradio-4.0/examples")
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/graphs/"
  DESTINATION "${GR4_EXAMPLE_GRAPHS_INSTALL_DIR}"
  COMPONENT gr4-example-graphs)

# the qa loads the graphs by id, so it needs the shared library of every family they name
if(ENABLE_TESTING
   AND TARGET gnuradio4::GrAnalogBlocksShared
   AND TARGET gnuradio4::GrBasicBlocksShared
   AND TARGET gnuradio4::GrFilterBlocksShared
   AND TARGET gnuradio4::GrMathBlocksShared)
  add_subdirectory(test)
endif()
