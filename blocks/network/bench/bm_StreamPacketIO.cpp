#include <algorithm>
#include <chrono>
#include <complex>
#include <cstdint>
#include <cstring>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/YamlPmt.hpp>

#include <gnuradio-4.0/network/PacketEnvelope.hpp>

// What the stream pair costs per sample on the scheduler's thread, at the shape a radio link runs at: 2 MS/s
// complex<float>, max_items 4096, one tag per packet.
//
// The scope is the same as bm_ZmqPacketIO's next door and for the same reason: this measures the per-packet work the
// two blocks do where the scheduler can see it -- build the metadata map with its carried tag, serialize it, build
// the header, copy the payload, and the reverse -- and not the socket. zmq_msg_send on an inproc pair measures
// libzmq, which this program does not own and cannot change, while the encode and decode paths are what a rate
// envelope for these blocks has to account for.
//
// The number that matters is nanoseconds per sample against the 500 ns per sample a 2 MS/s stream leaves, so the
// last column is the share of one sample period the link costs. Built by name (EXCLUDE_FROM_ALL) and run pinned.

namespace {

using namespace std::chrono;
using Sample = std::complex<float>;

constexpr std::size_t kRepeats = 2000UZ;

struct Encoded {
    std::array<std::uint8_t, gr::network::kHeaderBytesV1> header{};
    std::string                                           metadata{};
    std::vector<std::uint8_t>                             payload{};
};

/// @brief One tag as the stream pair carries it: an offset from the chunk's first sample and the map standing there.
[[nodiscard]] gr::property_map carriedTags(std::size_t nTags) {
    gr::property_map map;
    map.insert_or_assign(gr::property_map::key_type("sequence"), gr::pmt::Value(std::uint64_t{42ULL}));
    map.insert_or_assign(gr::property_map::key_type("stream_position"), gr::pmt::Value(std::uint64_t{172032ULL}));
    map.insert_or_assign(gr::property_map::key_type("sample_rate"), gr::pmt::Value(2000000.0f));
    if (nTags == 0UZ) {
        return map;
    }
    gr::Tensor<gr::pmt::Value> tags;
    for (std::size_t i = 0UZ; i < nTags; ++i) {
        gr::property_map inner;
        inner.insert_or_assign(gr::property_map::key_type("trigger_name"), gr::pmt::Value(std::string("burst")));
        gr::property_map entry;
        entry.insert_or_assign(gr::property_map::key_type("offset"), gr::pmt::Value(static_cast<std::uint64_t>(i)));
        entry.insert_or_assign(gr::property_map::key_type("map"), gr::pmt::Value(std::move(inner)));
        tags.push_back(gr::pmt::Value(std::move(entry)));
    }
    map.insert_or_assign(gr::property_map::key_type("packet_tags"), gr::pmt::Value(std::move(tags)));
    return map;
}

/// @brief The sink's per-chunk work: serialize the map, build the header, copy the payload.
[[nodiscard]] Encoded encodeOne(const gr::property_map& map, std::span<const Sample> items) {
    Encoded encoded;
    encoded.metadata = gr::pmt::yaml::serialize(map);
    const auto raw   = std::as_bytes(items);
    encoded.payload.resize(raw.size());
    std::memcpy(encoded.payload.data(), raw.data(), raw.size());

    gr::network::EnvelopeHeader header;
    header.item_type     = gr::network::kItemTypeCode<Sample>;
    header.item_size     = static_cast<std::uint8_t>(sizeof(Sample));
    header.item_count    = static_cast<std::uint32_t>(items.size());
    header.payload_bytes = static_cast<std::uint32_t>(raw.size());
    header.meta_bytes    = static_cast<std::uint32_t>(encoded.metadata.size());
    encoded.header       = gr::network::encodeHeader(header);
    return encoded;
}

/// @brief The source's per-chunk work: validate the header, parse the map, read the tags out, copy the payload.
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
    std::size_t tagsRead = 0UZ;
    if (const auto entry = map.find(gr::property_map::key_type("packet_tags")); entry != map.end()) {
        if (const auto* list = entry->second.get_if<gr::Tensor<gr::pmt::Value>>(); list != nullptr) {
            for (const auto& element : *list) {
                if (const auto* inner = element.get_if<gr::property_map>(); inner != nullptr) {
                    tagsRead += inner->size();
                }
            }
        }
    }
    std::vector<Sample> items(header->item_count);
    if (!items.empty()) {
        std::memcpy(items.data(), encoded.payload.data(), header->payload_bytes);
    }
    return items.size() + map.size() + tagsRead;
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
    constexpr std::array<std::size_t, 3> chunkSizes{1024UZ, 4096UZ, 16384UZ};
    constexpr std::array<std::size_t, 2> tagCounts{0UZ, 1UZ};
    constexpr double                     kSamplePeriodNs = 500.0; // one sample at 2 MS/s

    std::println("max_items  tags   sink ns/pkt  sink ns/sample   source ns/pkt  source ns/sample   pair % of 2 MS/s");
    for (const std::size_t nTags : tagCounts) {
        const gr::property_map map = carriedTags(nTags);
        for (const std::size_t nItems : chunkSizes) {
            std::vector<Sample> items(nItems);
            for (std::size_t i = 0UZ; i < nItems; ++i) {
                items[i] = Sample{static_cast<float>(i), -static_cast<float>(i)};
            }
            const Encoded encoded = encodeOne(map, items);

            const double sinkNs   = nanosecondsPerCall([&map, &items] {
                volatile const std::size_t n = encodeOne(map, items).payload.size();
                (void)n;
            });
            const double sourceNs = nanosecondsPerCall([&encoded] {
                volatile const std::size_t n = decodeOne(encoded);
                (void)n;
            });

            const double perSampleSink   = sinkNs / static_cast<double>(nItems);
            const double perSampleSource = sourceNs / static_cast<double>(nItems);
            std::println("{:9} {:5}   {:11.1f}  {:14.3f}   {:13.1f}  {:16.3f}   {:15.2f}", nItems, nTags, sinkNs, perSampleSink, sourceNs, perSampleSource, 100.0 * (perSampleSink + perSampleSource) / kSamplePeriodNs);
        }
    }
    return 0;
}
