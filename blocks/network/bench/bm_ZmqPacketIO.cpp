#include <algorithm>
#include <chrono>
#include <complex>
#include <cstdint>
#include <cstring>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/YamlPmt.hpp>

#include <gnuradio-4.0/network/PacketEnvelope.hpp>

// What the packet transport costs per packet and per payload byte, and where the crossover between the two lies.
//
// The reasoning the specification records is that metadata serialization dominates at small payloads and the copy at
// large ones: the serializer sorts the map, formats every value through an ostream and builds a std::string, which is
// hundreds of instructions per key, against roughly one byte per cycle for a memcpy. Nothing measured that, so this
// benchmark exists to settle it, and it reports the serializer in isolation and a plain vector copy as the floor so
// the share of each is visible rather than inferred.
//
// It measures the per-packet work the blocks do on the scheduler's thread -- build the map, serialize it, build the
// header, copy the payload, and the reverse -- and not the socket. That is the deliberate scope: zmq_msg_send on an
// inproc pair measures libzmq, which this program does not own and cannot change, while the encode and decode paths
// are what a rate envelope for these blocks has to account for. A socket-level figure belongs to a separate
// measurement against a peer process.
//
// The target is EXCLUDE_FROM_ALL, so it is built by name when a measurement is wanted and never by the default build:
// the build budget these blocks were written under does not carry a benchmark on every build.

namespace {

using namespace std::chrono;

constexpr std::size_t kRepeats = 2000UZ;

/// @brief A metadata map with @p nKeys entries, the first four being the ones a framed chain actually carries.
[[nodiscard]] gr::property_map metadataOf(std::size_t nKeys) {
    gr::property_map map;
    if (nKeys > 0UZ) {
        map.insert_or_assign(gr::property_map::key_type("sequence"), gr::pmt::Value(static_cast<std::uint64_t>(42)));
    }
    if (nKeys > 1UZ) {
        map.insert_or_assign(gr::property_map::key_type("sample_start"), gr::pmt::Value(static_cast<std::uint64_t>(4096)));
    }
    if (nKeys > 2UZ) {
        map.insert_or_assign(gr::property_map::key_type("protocol"), gr::pmt::Value(std::string("ax25")));
    }
    if (nKeys > 3UZ) {
        map.insert_or_assign(gr::property_map::key_type("crc_ok"), gr::pmt::Value(true));
    }
    for (std::size_t i = 4UZ; i < nKeys; ++i) {
        map.insert_or_assign(gr::property_map::key_type(std::format("private_{:02}", i)), gr::pmt::Value(static_cast<std::uint64_t>(i)));
    }
    return map;
}

struct Encoded {
    std::array<std::uint8_t, gr::network::kHeaderBytesV1> header{};
    std::string                                           metadata{};
    std::vector<std::uint8_t>                             payload{};
};

/// @brief The sink's per-packet work: serialize the map, build the header, copy the payload.
[[nodiscard]] Encoded encodeOne(const gr::property_map& map, std::span<const std::uint8_t> items) {
    Encoded encoded;
    encoded.metadata = gr::pmt::yaml::serialize(map);
    encoded.payload.resize(items.size());
    std::memcpy(encoded.payload.data(), items.data(), items.size());

    gr::network::EnvelopeHeader header;
    header.item_type     = gr::network::kItemTypeCode<std::uint8_t>;
    header.item_size     = 1U;
    header.item_count    = static_cast<std::uint32_t>(items.size());
    header.payload_bytes = static_cast<std::uint32_t>(items.size());
    header.meta_bytes    = static_cast<std::uint32_t>(encoded.metadata.size());
    encoded.header       = gr::network::encodeHeader(header);
    return encoded;
}

/// @brief The source's per-envelope work: validate the header, parse the map, copy the payload.
[[nodiscard]] std::size_t decodeOne(const Encoded& encoded) {
    const auto header = gr::network::decodeHeader(encoded.header);
    if (!header.has_value()) {
        return 0UZ;
    }
    gr::property_map map;
    if (header->meta_bytes != 0U) {
        const auto parsed = gr::pmt::yaml::deserialize(encoded.metadata);
        if (parsed.has_value()) {
            map = *parsed;
        }
    }
    std::vector<std::uint8_t> items(header->item_count);
    if (!items.empty()) {
        std::memcpy(items.data(), encoded.payload.data(), items.size());
    }
    return items.size() + map.size();
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
    constexpr std::array<std::size_t, 5> payloadLengths{1UZ, 16UZ, 256UZ, 1500UZ, 65536UZ};
    constexpr std::array<std::size_t, 3> keyCounts{0UZ, 4UZ, 20UZ};

    std::println("payload  keys   encode ns/pkt  encode ns/B   decode ns/pkt  decode ns/B   serialize ns   copy ns");
    for (const std::size_t nKeys : keyCounts) {
        const gr::property_map map      = metadataOf(nKeys);
        const double           serialNs = nanosecondsPerCall([&map] {
            volatile const std::size_t n = gr::pmt::yaml::serialize(map).size();
            (void)n;
        });
        for (const std::size_t nBytes : payloadLengths) {
            std::vector<std::uint8_t> items(nBytes);
            for (std::size_t i = 0UZ; i < nBytes; ++i) {
                items[i] = static_cast<std::uint8_t>(i);
            }
            const Encoded encoded = encodeOne(map, items);

            const double encodeNs = nanosecondsPerCall([&map, &items] {
                volatile const std::size_t n = encodeOne(map, items).payload.size();
                (void)n;
            });
            const double decodeNs = nanosecondsPerCall([&encoded] {
                volatile const std::size_t n = decodeOne(encoded);
                (void)n;
            });
            const double copyNs   = nanosecondsPerCall([&items] {
                volatile const std::size_t n = std::vector<std::uint8_t>(items.begin(), items.end()).size();
                (void)n;
            });

            std::println("{:7} {:5}   {:13.1f}  {:11.3f}   {:13.1f}  {:11.3f}   {:12.1f}  {:8.1f}", nBytes, nKeys, encodeNs, encodeNs / static_cast<double>(nBytes), decodeNs, decodeNs / static_cast<double>(nBytes), serialNs, copyNs);
        }
    }
    return 0;
}
