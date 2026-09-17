// The consumer project beside this file is configured and then built. Configuring proves that every target the
// exported interface of gnuradio4::gr-network names can be defined again by whoever finds the installed package;
// compiling proves that what the interface carries is enough to use the family's headers. The two are different
// claims: a missing target name ends the configure, while a missing include directory is not noticed until a
// translation unit asks for a header. `<zmq.hpp>` is the header that asks, because cppzmq is its own package and its
// directory is the one an install cannot assume.

#include <cstdint>

#include <gnuradio-4.0/network/ZmqPacketIO.hpp>

int main() {
    const gr::blocks::network::ZmqPacketSink<std::uint8_t> sink;
    return sink.endpoint.value.empty() ? 0 : 1;
}
