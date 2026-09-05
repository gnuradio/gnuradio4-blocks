#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/digital/DelimiterExtractor.hpp>

namespace {

using gr::blocks::digital::DelimiterExtractor;
using Record = gr::DataSet<std::uint8_t>;

constexpr std::size_t kItemsPerCall = 65536UZ;
constexpr std::size_t kRepeats      = 9UZ;

/// The reader and writer spans a `processBulk` needs, with no tag handling: the measurement is the item path.
struct TagSpan : std::span<const gr::Tag> {
    using value_type = gr::Tag;
    bool consume(std::size_t) const noexcept { return true; }
};

struct TagWriteSpan : std::span<gr::Tag> {
    using value_type = gr::Tag;
    void publish(std::size_t) const noexcept {}
};

template<typename T>
struct ReaderSpan : std::span<const T> {
    using value_type = T;

    TagSpan     rawTags{};
    std::size_t streamIndex = 0UZ;
    std::size_t consumed    = 0UZ;
    bool        isConnected = true;
    bool        isSync      = true;

    explicit ReaderSpan(std::span<const T> items) : std::span<const T>(items) {}

    bool consume(std::size_t nItems) noexcept {
        consumed = nItems;
        return true;
    }
    void consumeTags(std::size_t) noexcept {}

    [[nodiscard]] std::vector<std::pair<std::ptrdiff_t, std::reference_wrapper<const gr::property_map>>> tags() const { return {}; }
    [[nodiscard]] std::vector<std::pair<std::ptrdiff_t, std::reference_wrapper<const gr::property_map>>> tags(std::size_t) const { return {}; }
};

template<typename T>
struct WriterSpan : std::span<T> {
    using value_type = T;

    TagWriteSpan tags{};
    std::size_t  streamIndex = 0UZ;
    std::size_t  count       = 0UZ;
    bool         isConnected = true;
    bool         isSync      = true;

    explicit WriterSpan(std::span<T> items, bool connected = true) : std::span<T>(items), isConnected(connected) {}

    void publish(std::size_t nItems) noexcept { count = nItems; }
    void publishTag(const gr::property_map&, std::size_t = 0UZ) noexcept {}
};

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
};

/// The bit-stuffing encoder, so that the bit-stuffing arm is fed a stream the decoder actually removes items from.
[[nodiscard]] std::vector<std::uint8_t> stuff(std::span<const std::uint8_t> bits, unsigned k) {
    std::vector<std::uint8_t> coded;
    coded.reserve(bits.size() + bits.size() / k + 1UZ);
    unsigned ones = 0U;
    for (const std::uint8_t bit : bits) {
        coded.push_back(bit);
        if (bit != 0U) {
            ++ones;
            if (ones == k) {
                coded.push_back(0U);
                ones = 0U;
            }
        } else {
            ones = 0U;
        }
    }
    return coded;
}

[[nodiscard]] std::vector<std::uint8_t> flagItems() { return {0U, 1U, 1U, 1U, 1U, 1U, 1U, 0U}; }

/// A bit stream of frames of @p frameItems payload bits each, stuffed and flag delimited, cut to one call's worth.
[[nodiscard]] std::vector<std::uint8_t> stuffedStream(std::size_t frameItems) {
    Rng                       rng;
    std::vector<std::uint8_t> wire = flagItems();
    while (wire.size() < kItemsPerCall) {
        std::vector<std::uint8_t> payload(frameItems);
        for (std::uint8_t& bit : payload) {
            bit = static_cast<std::uint8_t>(rng.next() & 1ULL);
        }
        const std::vector<std::uint8_t> coded = stuff(std::span<const std::uint8_t>(payload), 5U);
        wire.insert(wire.end(), coded.begin(), coded.end());
        const std::vector<std::uint8_t> flag = flagItems();
        wire.insert(wire.end(), flag.begin(), flag.end());
    }
    wire.resize(kItemsPerCall);
    return wire;
}

/// The same frame lengths over raw bits, with no transparency and no coded expansion.
[[nodiscard]] std::vector<std::uint8_t> plainBitStream(std::size_t frameItems, std::size_t delimiterBits) {
    Rng                       rng;
    std::vector<std::uint8_t> wire;
    wire.reserve(kItemsPerCall + delimiterBits);
    while (wire.size() < kItemsPerCall) {
        for (std::size_t k = 0UZ; k < delimiterBits; ++k) {
            wire.push_back(static_cast<std::uint8_t>(k + 1UZ == delimiterBits ? 0U : 1U));
        }
        for (std::size_t k = 0UZ; k < frameItems; ++k) {
            wire.push_back(static_cast<std::uint8_t>(rng.next() & 1ULL));
        }
    }
    wire.resize(kItemsPerCall);
    return wire;
}

/// Byte items with `0xC0` between frames, escaped so that the payload cannot carry the delimiter.
[[nodiscard]] std::vector<std::uint8_t> escapedByteStream(std::size_t frameItems) {
    Rng                       rng;
    std::vector<std::uint8_t> wire;
    wire.reserve(kItemsPerCall + 2UZ);
    while (wire.size() < kItemsPerCall) {
        wire.push_back(0xC0U);
        for (std::size_t k = 0UZ; k < frameItems; ++k) {
            const auto byte = static_cast<std::uint8_t>(rng.next() & 0xFFULL);
            if (byte == 0xC0U) {
                wire.push_back(0xDBU);
                wire.push_back(0xDCU);
            } else if (byte == 0xDBU) {
                wire.push_back(0xDBU);
                wire.push_back(0xDDU);
            } else {
                wire.push_back(byte);
            }
        }
    }
    wire.resize(kItemsPerCall);
    return wire;
}

[[nodiscard]] std::string onesDelimiter(std::size_t bits) {
    std::string text(bits, '1');
    if (!text.empty()) {
        text.back() = '0';
    }
    return text;
}

} // namespace

int main() {
    const std::vector<std::size_t> kFrameLengths{8UZ, 64UZ, 1024UZ};
    const std::vector<std::size_t> kDelimiterBits{1UZ, 8UZ, 64UZ};

    std::vector<Record>       records(4096UZ);
    std::vector<Record>       rejects(4096UZ);
    std::vector<std::uint8_t> copyOut(kItemsPerCall);

    // the floor every arm is read against: one span copy of the same item count
    std::vector<gr::blocks::testing::bench::Arm> arms;
    const std::vector<std::uint8_t>              plainFloor = plainBitStream(64UZ, 8UZ);
    arms.push_back({"span copy, the floor", [&] {
                        std::ranges::copy(plainFloor, copyOut.begin());
                        return static_cast<double>(copyOut[kItemsPerCall / 2UZ]);
                    }});

    // transparency 'none' at one bit an item, over the three delimiter lengths and the three frame lengths
    std::vector<std::vector<std::uint8_t>>        plainStreams;
    std::vector<DelimiterExtractor<std::uint8_t>> plainBlocks;
    std::vector<std::string>                      plainLabels;
    for (const std::size_t delimiterBits : kDelimiterBits) {
        for (const std::size_t frameItems : kFrameLengths) {
            plainStreams.push_back(plainBitStream(frameItems, delimiterBits));
            plainBlocks.push_back(make<DelimiterExtractor<std::uint8_t>>({{"end_delimiter", onesDelimiter(delimiterBits)}, {"max_payload_items", gr::Size_t{4096}}}));
            plainLabels.push_back(std::format("none, 1 bit/item, d={}, frame={}", delimiterBits, frameItems));
        }
    }
    for (std::size_t which = 0UZ; which < plainBlocks.size(); ++which) {
        arms.push_back({plainLabels[which], [&, which] {
                            ReaderSpan<std::uint8_t> inSpan{std::span<const std::uint8_t>(plainStreams[which])};
                            WriterSpan<Record>       outSpan{std::span<Record>(records)};
                            WriterSpan<Record>       rejectSpan{std::span<Record>(rejects)};
                            std::ignore = plainBlocks[which].processBulk(inSpan, outSpan, rejectSpan);
                            return static_cast<double>(inSpan.consumed);
                        }});
    }

    // bit stuffing, which the operation table predicts at about fifty per cent over 'none'
    std::vector<std::vector<std::uint8_t>>        stuffedStreams;
    std::vector<DelimiterExtractor<std::uint8_t>> stuffedBlocks;
    for (const std::size_t frameItems : kFrameLengths) {
        stuffedStreams.push_back(stuffedStream(frameItems));
        stuffedBlocks.push_back(make<DelimiterExtractor<std::uint8_t>>({{"end_delimiter", std::string("01111110")}, {"transparency", std::string("bit_stuffing")}, {"max_payload_items", gr::Size_t{4096}}}));
    }
    for (std::size_t which = 0UZ; which < stuffedBlocks.size(); ++which) {
        arms.push_back({std::format("bit_stuffing, 1 bit/item, d=8, frame={}", kFrameLengths[which]), [&, which] {
                            ReaderSpan<std::uint8_t> inSpan{std::span<const std::uint8_t>(stuffedStreams[which])};
                            WriterSpan<Record>       outSpan{std::span<Record>(records)};
                            WriterSpan<Record>       rejectSpan{std::span<Record>(rejects)};
                            std::ignore = stuffedBlocks[which].processBulk(inSpan, outSpan, rejectSpan);
                            return static_cast<double>(inSpan.consumed);
                        }});
    }

    // byte escaping at eight bits an item, where the delay ring degenerates and the escape table stays in L1
    std::vector<std::vector<std::uint8_t>>        escapedStreams;
    std::vector<DelimiterExtractor<std::uint8_t>> escapedBlocks;
    for (const std::size_t frameItems : kFrameLengths) {
        escapedStreams.push_back(escapedByteStream(frameItems));
        escapedBlocks.push_back(make<DelimiterExtractor<std::uint8_t>>({{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}, {"transparency", std::string("byte_escape")}, //
            {"escape_item", gr::Size_t{0xDB}}, {"escape_map", std::vector<gr::Size_t>{0xDCU, 0xC0U, 0xDDU, 0xDBU}}, {"max_payload_items", gr::Size_t{4096}}}));
    }
    for (std::size_t which = 0UZ; which < escapedBlocks.size(); ++which) {
        arms.push_back({std::format("byte_escape, 8 bits/item, d=1, frame={}", kFrameLengths[which]), [&, which] {
                            ReaderSpan<std::uint8_t> inSpan{std::span<const std::uint8_t>(escapedStreams[which])};
                            WriterSpan<Record>       outSpan{std::span<Record>(records)};
                            WriterSpan<Record>       rejectSpan{std::span<Record>(rejects)};
                            std::ignore = escapedBlocks[which].processBulk(inSpan, outSpan, rejectSpan);
                            return static_cast<double>(inSpan.consumed);
                        }});
    }

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kItemsPerCall, kRepeats);
}
