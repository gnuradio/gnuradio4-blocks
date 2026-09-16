if(TARGET PkgConfig::GR4_LIBZMQ AND GR4_CPPZMQ_INCLUDE_DIR)
else()
  message(
    STATUS
      "GR4_ENABLE_NETWORK is ON but libzmq or the cppzmq header was not found — the ZMQ transports are not built; the TCP/UDP transports are"
  )
endif()

add_library(gr-network INTERFACE)
add_library(gnuradio4::gr-network ALIAS gr-network)
target_include_directories(gr-network INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                                $<INSTALL_INTERFACE:include/>)
if(GR4_NETWORK_HAVE_LIBZMQ)
  # cppzmq is a stray header rather than a package, so it carries no include directory of its own
  target_include_directories(gr-network SYSTEM INTERFACE $<BUILD_INTERFACE:${GR4_CPPZMQ_INCLUDE_DIR}>)
endif()
install(TARGETS gr-network EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/network"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")
if(GR4_NETWORK_HAVE_LIBZMQ)
endif()

gr_add_block_library(
  GrNetworkBlocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  LINK_LIBRARIES
  gr-network)

if(TARGET gnuradio4::GrNetworkBlocksShared AND ENABLE_TESTING)
  if(GR4_NETWORK_HAVE_LIBZMQ)
  endif()
  add_subdirectory(test)
endif()

add_subdirectory(bench)
