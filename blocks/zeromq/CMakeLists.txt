add_library(gr-zeromq INTERFACE)
add_library(gnuradio4::gr-zeromq ALIAS gr-zeromq)
target_link_libraries(gr-zeromq INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm ${GR4_CPPZMQ_TARGET})
target_include_directories(gr-zeromq INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                               $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-zeromq EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/zeromq"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

gr_add_block_library(
  GrZeromqBlocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  include/gnuradio-4.0/zeromq/ZmqPubSink.hpp
  include/gnuradio-4.0/zeromq/ZmqPullSource.hpp
  include/gnuradio-4.0/zeromq/ZmqPushSink.hpp
  include/gnuradio-4.0/zeromq/ZmqRepSink.hpp
  include/gnuradio-4.0/zeromq/ZmqReqSource.hpp
  include/gnuradio-4.0/zeromq/ZmqSubSource.hpp
  LINK_LIBRARIES
  gr-zeromq)

if(TARGET gnuradio4::GrZeromqBlocksShared AND ENABLE_TESTING)
  add_subdirectory(test)
endif()

if(ENABLE_EXAMPLES AND GR4_ENABLE_ZEROMQ_EXAMPLES)
  add_subdirectory(examples)
endif()
