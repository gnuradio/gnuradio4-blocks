#include <array>
#include <chrono>
#include <cstdint>
#include <format>
#include <print>
#include <string>
#include <vector>

#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/YamlPmt.hpp>

#include <gnuradio-4.0/network/MessagePacketIO.hpp>
#include <gnuradio-4.0/network/PacketEnvelope.hpp>

// What the message pair costs per message on the scheduler's thread, and therefore how many messages a second one
// link sustains. A message plane is not a sample rate: what a host application asks of it is a few messages a second
// at rest and a burst when a graph is reconfigured, so the figure that matters is messages per second at a stated
// message size, not nanoseconds per byte.
//
// The scope is bm_ZmqPacketIO's: this measures the encode and decode paths the blocks run, not the socket. The
// message size is stated as the serialized metadata frame, which is the whole of what a message packet puts on a
// wire -- its payload carries no items at all.

namespace {

using namespace std::chrono;

constexpr std::size_t kRepeats = 5000UZ;

/// @brief A message with @p nKeys payload entries, the first four being what a settings exchange actually carries.
[[nodiscard]] gr::Message messageOf(std::size_t nKeys) {
    gr::property_map data;
    if (nKeys > 0UZ) {
        data.insert_or_assign(gr::property_map::key_type("sample_rate"), gr::pmt::Value(2000000.0f));
    }
    if (nKeys > 1UZ) {
        data.insert_or_assign(gr::property_map::key_type("signal_name"), gr::pmt::Value(std::string("link_iq")));
    }
    if (nKeys > 2UZ) {
        data.insert_or_assign(gr::property_map::key_type("frequency"), gr::pmt::Value(101500000.0));
    }
    if (nKeys > 3UZ) {
        data.insert_or_assign(gr::property_map::key_type("crc_ok"), gr::pmt::Value(true));
    }
    for (std::size_t i = 4UZ; i < nKeys; ++i) {
        data.insert_or_assign(gr::property_map::key_type(std::format("setting_{:02}", i)), gr::pmt::Value(static_cast<std::uint64_t>(i)));
    }

    gr::Message message;
    message.protocol        = gr::message::defaultClientProtocol;
    message.cmd             = gr::message::Command::Set;
    message.serviceName     = "gr::blocks::filter::FirFilter<float32>#3";
    message.clientRequestID = "request-1042";
    message.endpoint        = "Settings";
    message.rbac            = "";
    message.data            = std::move(data);
    return message;
}

template<typename F>
[[nodiscard]] double nanosecondsPerCall(F&& body) {
    const auto started = steady_clock::now();
    for (std::size_t i = 0UZ; i < kRepeats; ++i) {
        body();
    }
    const auto elapsed = duration_cast<nanoseconds>(steady_clock::now() - started).count();
    return static_cast<double>(elapsed) / static_cast<double>(kRepeats);
}

} // namespace

int main() {
    using namespace gr::blocks::network::detail::messagepacket;
    constexpr std::array<std::size_t, 4> keyCounts{0UZ, 4UZ, 20UZ, 100UZ};

    std::println("payload keys  frame bytes   sink ns/msg   source ns/msg   pair ns/msg   messages/s");
    for (const std::size_t nKeys : keyCounts) {
        const gr::Message      message  = messageOf(nKeys);
        const gr::property_map map      = encode(message, 42ULL);
        const std::string      metadata = gr::pmt::yaml::serialize(map);

        const double sinkNs   = nanosecondsPerCall([&message] {
            volatile const std::uint64_t n = envelopeOf(encode(message, 42ULL)).bytes();
            (void)n;
        });
        const double sourceNs = nanosecondsPerCall([&metadata] {
            const auto  parsed = gr::pmt::yaml::deserialize(metadata);
            std::size_t fields = 0UZ;
            if (parsed.has_value()) {
                if (const auto entry = parsed->find(gr::property_map::key_type(kMessageKey)); entry != parsed->end()) {
                    if (const auto* fieldMap = entry->second.get_if<gr::property_map>(); fieldMap != nullptr) {
                        const auto decoded = decode(*fieldMap);
                        fields             = decoded.has_value() ? decoded->endpoint.size() : 0UZ;
                    }
                }
            }
            volatile const std::size_t n = fields;
            (void)n;
        });

        const double pairNs = sinkNs + sourceNs;
        std::println("{:12} {:12}  {:12.1f}  {:14.1f}  {:12.1f}  {:11.0f}", nKeys, metadata.size(), sinkNs, sourceNs, pairNs, 1e9 / pairNs);
    }
    return 0;
}
