add_executable(packet_link_tx packet_link_tx.cpp)
target_link_libraries(
  packet_link_tx
  PRIVATE gnuradio-options
          gnuradio-core
          gnuradio-blocklib-core
          gr-network
          gr-basic
          gr-digital
          gr-testing)

add_executable(packet_link_rx packet_link_rx.cpp)
target_link_libraries(
  packet_link_rx
  PRIVATE gnuradio-options
          gnuradio-core
          gnuradio-blocklib-core
          gr-network
          gr-basic
          gr-digital
          gr-testing)

if(CMAKE_GENERATOR MATCHES "Ninja")
  # Each application TU instantiates the full tx or rx chain, the same weight class as the heavy test TUs; they compile
  # through the width-limited pool.
  set_property(TARGET packet_link_tx packet_link_rx PROPERTY JOB_POOL_COMPILE gr4_heavy_compile)
endif()
