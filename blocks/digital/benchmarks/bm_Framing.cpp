#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <functional>
#include <numbers>
#include <span>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/digital/AccessCodeCorrelator.hpp>
#include <gnuradio-4.0/digital/PacketFramer.hpp>
#include <gnuradio-4.0/digital/PreambleCorrelator.hpp>

namespace {

using gr::blocks::digital::AccessCodeCorrelator;
using gr::blocks::digital::PacketFramer;
using gr::blocks::digital::PreambleCorrelator;
using CF     = std::complex<float>;
using Packet = gr::DataSet<std::uint8_t>;

constexpr std::size_t kItemsPerCall = 65536UZ;
constexpr std::size_t kRepeats      = 9UZ;

/// The reader and writer spans a detector's processBulk needs.
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

    /// The pair form the framework presents, empty here: the benchmark measures the search path, not tag handling.
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

[[nodiscard]] std::string codeString(std::uint64_t word, std::size_t bits) {
    std::string text(bits, '0');
    for (std::size_t i = 0UZ; i < bits; ++i) {
        text[i] = ((word >> (bits - 1UZ - i)) & 1ULL) != 0ULL ? '1' : '0';
    }
    return text;
}

[[nodiscard]] std::vector<std::uint8_t> bits() {
    std::vector<std::uint8_t> stream(kItemsPerCall);
    std::uint64_t             state = 0x9e3779b97f4a7c15ULL;
    for (std::uint8_t& bit : stream) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        bit = static_cast<std::uint8_t>(state & 1ULL);
    }
    return stream;
}

[[nodiscard]] std::vector<CF> samples() {
    std::vector<CF> stream(kItemsPerCall);
    for (std::size_t i = 0UZ; i < stream.size(); ++i) {
        const double phase = 2.0 * std::numbers::pi * 0.037 * static_cast<double>(i);
        stream[i]          = CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
    }
    return stream;
}

[[nodiscard]] std::vector<float> sequenceOf(std::size_t length) {
    std::vector<float> flat(2UZ * length);
    std::uint64_t      state = 0x243f6a8885a308d3ULL;
    for (std::size_t k = 0UZ; k < length; ++k) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        const double angle  = std::numbers::pi * 0.25 + std::numbers::pi * 0.5 * static_cast<double>(state & 3ULL);
        flat[2UZ * k]       = static_cast<float>(std::cos(angle));
        flat[2UZ * k + 1UZ] = static_cast<float>(std::sin(angle));
    }
    return flat;
}

} // namespace

int main() {
    constexpr std::array<std::size_t, 3UZ> kCodeLengths{16UZ, 32UZ, 64UZ};
    constexpr std::array<std::size_t, 4UZ> kSequenceLengths{16UZ, 32UZ, 64UZ, 128UZ};
    constexpr std::uint64_t                kSyncWord = 0xACDDA4E2F28C20FCULL;

    const std::vector<std::uint8_t> bitStream    = bits();
    const std::vector<CF>           sampleStream = samples();

    std::vector<std::uint8_t> byteOut(kItemsPerCall);
    std::vector<CF>           complexOut(kItemsPerCall);
    std::vector<float>        magnitudeOut(kItemsPerCall);
    std::vector<Packet>       packetOut(64UZ);

    std::vector<AccessCodeCorrelator<std::uint8_t>> correlators;
    for (const std::size_t length : kCodeLengths) {
        correlators.push_back(make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", codeString(kSyncWord >> (64UZ - length), length)}, {"max_errors", static_cast<gr::Size_t>(length / 8UZ)}}));
    }
    std::vector<PreambleCorrelator<float>> preambles;
    for (const std::size_t length : kSequenceLengths) {
        preambles.push_back(make<PreambleCorrelator<float>>({{"sequence", sequenceOf(length)}, {"threshold", 0.9f}}));
    }
    PacketFramer<std::uint8_t> framer = make<PacketFramer<std::uint8_t>>({{"max_payload_items", 4095U}});

    // a distributed sync word holds one register per residue class, so the comparison is the contiguous one plus a
    // modular index; the arms at the two AO-40 strides are what turn that claim into a number
    constexpr std::array<std::size_t, 3UZ>          kStrides{1UZ, 51UZ, 80UZ};
    std::vector<AccessCodeCorrelator<std::uint8_t>> strided;
    for (const std::size_t spacing : kStrides) {
        strided.push_back(make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", codeString(kSyncWord, 52UZ)}, {"max_errors", static_cast<gr::Size_t>(14)}, {"stride", static_cast<gr::Size_t>(spacing)}}));
    }
    AccessCodeCorrelator<std::uint8_t> lagged = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", codeString(kSyncWord, 52UZ)}, {"max_errors", static_cast<gr::Size_t>(14)}, //
        {"stride", static_cast<gr::Size_t>(51)}, {"tag_at", std::string("code_start")}});

    // the AO-40 long form's 65 items spill into the register's second limb, which every shorter code skips; this arm
    // is the price of the wide path, read against the n=52 arm at the same stride
    AccessCodeCorrelator<std::uint8_t> longForm = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", codeString(kSyncWord, 52UZ) + codeString(kSyncWord, 13UZ)}, //
        {"max_errors", static_cast<gr::Size_t>(16)}, {"stride", static_cast<gr::Size_t>(80)}});

    std::vector<gr::blocks::testing::bench::Arm> arms;
    for (std::size_t which = 0UZ; which < kCodeLengths.size(); ++which) {
        arms.push_back({std::format("AccessCodeCorrelator, n={}", kCodeLengths[which]), [&, which] {
                            ReaderSpan<std::uint8_t> inSpan{std::span<const std::uint8_t>(bitStream)};
                            WriterSpan<std::uint8_t> outSpan{std::span<std::uint8_t>(byteOut)};
                            std::ignore = correlators[which].processBulk(inSpan, outSpan);
                            return static_cast<double>(byteOut[kItemsPerCall / 2UZ]);
                        }});
    }
    for (std::size_t which = 0UZ; which < kSequenceLengths.size(); ++which) {
        arms.push_back({std::format("PreambleCorrelator, N={}", kSequenceLengths[which]), [&, which] {
                            ReaderSpan<CF>    inSpan{std::span<const CF>(sampleStream)};
                            WriterSpan<CF>    outSpan{std::span<CF>(complexOut)};
                            WriterSpan<float> corrSpan{std::span<float>(magnitudeOut)};
                            std::ignore = preambles[which].processBulk(inSpan, outSpan, corrSpan);
                            return static_cast<double>(magnitudeOut[kItemsPerCall / 2UZ]);
                        }});
    }
    for (std::size_t which = 0UZ; which < kStrides.size(); ++which) {
        arms.push_back({std::format("AccessCodeCorrelator, n=52, stride={}", kStrides[which]), [&, which] {
                            ReaderSpan<std::uint8_t> inSpan{std::span<const std::uint8_t>(bitStream)};
                            WriterSpan<std::uint8_t> outSpan{std::span<std::uint8_t>(byteOut)};
                            std::ignore = strided[which].processBulk(inSpan, outSpan);
                            return static_cast<double>(byteOut[kItemsPerCall / 2UZ]);
                        }});
    }
    arms.push_back({"AccessCodeCorrelator, n=52, stride=51, tag_at=code_start", [&] {
                        ReaderSpan<std::uint8_t> inSpan{std::span<const std::uint8_t>(bitStream)};
                        WriterSpan<std::uint8_t> outSpan{std::span<std::uint8_t>(byteOut)};
                        std::ignore = lagged.processBulk(inSpan, outSpan);
                        return static_cast<double>(byteOut[kItemsPerCall / 2UZ]);
                    }});
    arms.push_back({"AccessCodeCorrelator, n=65, stride=80 (two limbs)", [&] {
                        ReaderSpan<std::uint8_t> inSpan{std::span<const std::uint8_t>(bitStream)};
                        WriterSpan<std::uint8_t> outSpan{std::span<std::uint8_t>(byteOut)};
                        std::ignore = longForm.processBulk(inSpan, outSpan);
                        return static_cast<double>(byteOut[kItemsPerCall / 2UZ]);
                    }});
    arms.push_back({"PacketFramer, searching", [&] {
                        ReaderSpan<std::uint8_t> inSpan{std::span<const std::uint8_t>(bitStream)};
                        WriterSpan<Packet>       outSpan{std::span<Packet>(packetOut)};
                        std::ignore = framer.processBulk(inSpan, outSpan);
                        return static_cast<double>(inSpan.consumed);
                    }});

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kItemsPerCall, kRepeats);
}
