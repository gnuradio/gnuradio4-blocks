# The example graphs are GRC YAML files that gr::loadGrc reads. They install as data to share/gnuradio-4.0/examples,
# beside the network family's example graphs.
set(GR4_EXAMPLE_GRAPHS_INSTALL_DIR "${CMAKE_INSTALL_DATADIR}/gnuradio-4.0/examples")
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/graphs/"
  DESTINATION "${GR4_EXAMPLE_GRAPHS_INSTALL_DIR}"
  COMPONENT gr4-example-graphs)

# the qa constructs each block the graphs name by id and links the shared library of each family
if(ENABLE_TESTING
   AND TARGET gnuradio4::GrAnalogBlocksShared
   AND TARGET gnuradio4::GrBasicBlocksShared
   AND TARGET gnuradio4::GrFilterBlocksShared
   AND TARGET gnuradio4::GrMathBlocksShared)
  add_subdirectory(test)
endif()
