#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/algorithm/digital/BitPacking.hpp>
#include <gnuradio-4.0/digital/Manchester.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::digital::ManchesterDecoder;
using gr::blocks::digital::ManchesterEncoder;

/// A tag key nobody reserved, so a rate changer's filtered forwarding drops it.
constexpr std::string_view kCustomKey = "manchester_probe";

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

template<typename TBlock>
void restage(TBlock& block, gr::property_map settings) {
    std::ignore = block.settings().set(std::move(settings));
    std::ignore = block.settings().activateContext();
    std::ignore = block.settings().applyStagedParameters();
}

[[nodiscard]] gr::property_map encoderSettings(std::string_view convention, gr::Size_t bitsPerItem = 8U, std::string_view order = "MsbFirst") { //
    return {{"convention", std::string(convention)}, {"bits_per_item", bitsPerItem}, {"bit_order", std::string(order)}};
}

[[nodiscard]] gr::property_map decoderSettings(std::string_view convention, gr::Size_t bitsPerItem = 8U, std::string_view order = "MsbFirst", gr::Size_t chipPhase = 0U) { return {{"convention", std::string(convention)}, {"bits_per_item", bitsPerItem}, {"bit_order", std::string(order)}, {"chip_phase", chipPhase}}; }

/// @brief The chips @p data encodes to, driven through the block one @p chunk of input items at a time.
[[nodiscard]] std::vector<std::uint8_t> encoded(gr::property_map settings, std::span<const std::uint8_t> data, std::size_t chunk = 0UZ) {
    auto block = make<ManchesterEncoder>(std::move(settings));
    return gr::blocks::testing::span::run<std::uint8_t>(block, data, chunk, 2UZ).samples;
}

/// @brief The two output streams of one decoder run.
struct Decoded {
    std::vector<std::uint8_t> bits{};
    std::vector<std::uint8_t> flags{};
};

[[nodiscard]] Decoded decodedBy(ManchesterDecoder& block, std::span<const std::uint8_t> chips, std::size_t chunk = 0UZ, std::span<const gr::Tag> tags = {}, bool flagsConnected = true) {
    std::vector<std::uint8_t> flags;
    auto                      seen = gr::blocks::testing::span::run3<std::uint8_t, std::uint8_t>(block, chips, chunk, flags, flagsConnected, tags);
    return {std::move(seen.samples), std::move(flags)};
}

[[nodiscard]] Decoded decoded(gr::property_map settings, std::span<const std::uint8_t> chips, std::size_t chunk = 0UZ, std::span<const gr::Tag> tags = {}, bool flagsConnected = true) {
    auto block = make<ManchesterDecoder>(std::move(settings));
    return decodedBy(block, chips, chunk, tags, flagsConnected);
}

/// @brief What `processEpilogue` published for @p trailing, given @p room output items to publish into.
[[nodiscard]] Decoded epilogueOf(ManchesterDecoder& block, std::span<const std::uint8_t> trailing, std::size_t room) {
    std::vector<std::uint8_t>                      bits(room, 0xEEU);
    std::vector<std::uint8_t>                      flags(room, 0xEEU);
    gr::blocks::testing::span::InputSpan<uint8_t>  inSpan{trailing};
    gr::blocks::testing::span::OutputSpan<uint8_t> outSpan{std::span<std::uint8_t>(bits)};
    gr::blocks::testing::span::OutputSpan<uint8_t> flagSpan{std::span<std::uint8_t>(flags)};

    std::ignore = block.processEpilogue(inSpan, outSpan, flagSpan);
    bits.resize(outSpan.count);
    flags.resize(flagSpan.count);
    return {std::move(bits), std::move(flags)};
}

/// @brief The chip stream `(out, violation)` reconstructs, which is the whole of the lossless claim.
[[nodiscard]] std::vector<std::uint8_t> rebuiltChips(std::span<const std::uint8_t> bits, std::span<const std::uint8_t> flags, unsigned k) {
    std::vector<std::uint8_t> chips(2UZ * bits.size());
    for (std::size_t m = 0UZ; m < bits.size(); ++m) {
        const unsigned head  = static_cast<unsigned>(bits[m]) ^ k;
        chips[2UZ * m]       = static_cast<std::uint8_t>(head);
        chips[2UZ * m + 1UZ] = static_cast<std::uint8_t>(flags[m] != 0U ? head : head ^ 1U);
    }
    return chips;
}

/// @brief The bit stream of @p items: each item's low @p width bits, traversed in the declared order.
[[nodiscard]] std::vector<std::uint8_t> bitsOf(std::span<const std::uint8_t> items, unsigned width, bool msbFirst) {
    std::vector<std::uint8_t> bits;
    bits.reserve(items.size() * width);
    for (const std::uint8_t item : items) {
        for (unsigned i = 0U; i < width; ++i) {
            const unsigned shift = msbFirst ? width - 1U - i : i;
            bits.push_back(static_cast<std::uint8_t>((static_cast<unsigned>(item) >> shift) & 1U));
        }
    }
    return bits;
}

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }

    [[nodiscard]] std::vector<std::uint8_t> items(std::size_t count, unsigned width) {
        const auto                mask = static_cast<std::uint8_t>((1U << width) - 1U);
        std::vector<std::uint8_t> data(count);
        for (std::uint8_t& item : data) {
            item = static_cast<std::uint8_t>(next() & mask);
        }
        return data;
    }
};

/// @brief What a sink saw at the far end of `source -> block -> sink`, with @p incoming planted at the source.
struct Seen {
    std::vector<std::uint8_t> samples{};
    std::vector<gr::Tag>      tags{};
};

template<typename TBlock>
[[nodiscard]] Seen throughGraph(gr::property_map settings, std::span<const std::uint8_t> stream, std::span<const gr::Tag> incoming = {}) {
    using gr::testing::ProcessFunction;
    using gr::testing::TagSink;
    using gr::testing::TagSource;

    gr::Graph  graph;
    const auto values = gr::Tensor<std::uint8_t>(stream.begin(), stream.end());
    auto&      source = graph.emplaceBlock<TagSource<std::uint8_t, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(stream.size())}, {"values", values}});
    source._tags.assign(incoming.begin(), incoming.end());
    std::ranges::sort(source._tags, std::ranges::less{}, &gr::Tag::index);

    auto& block = graph.emplaceBlock<TBlock>(std::move(settings));
    auto& sink  = graph.emplaceBlock<TagSink<std::uint8_t, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

    boost::ut::expect(graph.connect<"out", "in">(source, block).has_value());
    boost::ut::expect(graph.connect<"out", "in">(block, sink).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value());

    return {std::vector<std::uint8_t>(sink._samples.begin(), sink._samples.end()), sink._tags};
}

/// @brief The offsets at which a tag carrying @p key reached the sink.
[[nodiscard]] std::vector<std::size_t> offsetsOf(std::span<const gr::Tag> tags, std::string_view key) {
    std::vector<std::size_t> where;
    for (const gr::Tag& tag : tags) {
        if (tag.map.contains(gr::property_map::key_type(key))) {
            where.push_back(tag.index);
        }
    }
    return where;
}

/// @brief The `sample_rate` a tag carries, or a value no rate takes.
[[nodiscard]] float sampleRateOf(const gr::Tag& tag) {
    const auto entry = tag.map.find(gr::property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()));
    if (entry == tag.map.end()) {
        return -1.0F;
    }
    const auto* rate = entry->second.get_if<float>();
    return rate == nullptr ? -2.0F : *rate;
}

/// @brief Transitions between adjacent chips of @p chips, which is what the waveform properties count.
[[nodiscard]] std::size_t transitions(std::span<const std::uint8_t> chips) {
    std::size_t count = 0UZ;
    for (std::size_t i = 1UZ; i < chips.size(); ++i) {
        count += chips[i] != chips[i - 1UZ] ? 1UZ : 0UZ;
    }
    return count;
}

/// @brief What the two sinks of `source -> decoder -> {out, violation}` saw, with the flag port optionally left open.
struct SeenPair {
    std::vector<std::uint8_t> bits{};
    std::vector<std::uint8_t> flags{};
    std::vector<gr::Tag>      tags{};
};

[[nodiscard]] SeenPair decoderGraph(gr::property_map settings, std::span<const std::uint8_t> chips, std::span<const gr::Tag> incoming = {}, bool connectFlags = true) {
    using gr::testing::ProcessFunction;
    using gr::testing::TagSink;
    using gr::testing::TagSource;

    gr::Graph  graph;
    const auto values = gr::Tensor<std::uint8_t>(chips.begin(), chips.end());
    auto&      source = graph.emplaceBlock<TagSource<std::uint8_t, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(chips.size())}, {"values", values}});
    source._tags.assign(incoming.begin(), incoming.end());
    std::ranges::sort(source._tags, std::ranges::less{}, &gr::Tag::index);

    auto& block = graph.emplaceBlock<ManchesterDecoder>(std::move(settings));
    auto& bits  = graph.emplaceBlock<TagSink<std::uint8_t, ProcessFunction::USE_PROCESS_ONE>>({{"name", "bits"}});
    boost::ut::expect(graph.connect<"out", "in">(source, block).has_value());
    boost::ut::expect(graph.connect<"out", "in">(block, bits).has_value());

    TagSink<std::uint8_t, ProcessFunction::USE_PROCESS_ONE>* flags = nullptr;
    if (connectFlags) {
        flags = &graph.emplaceBlock<TagSink<std::uint8_t, ProcessFunction::USE_PROCESS_ONE>>({{"name", "flags"}});
        boost::ut::expect(graph.connect<"violation", "in">(block, *flags).has_value());
    }

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value());

    SeenPair seen;
    seen.bits.assign(bits._samples.begin(), bits._samples.end());
    seen.tags = bits._tags;
    if (flags != nullptr) {
        seen.flags.assign(flags->_samples.begin(), flags->_samples.end());
    }
    return seen;
}

constexpr std::array<std::string_view, 2> kConventions{"ieee802_3", "ge_thomas"};
constexpr std::array<std::string_view, 2> kOrders{"MsbFirst", "LsbFirst"};

} // namespace

const boost::ut::suite<"Manchester encoder"> manchesterEncoderTests = [] {
    using namespace boost::ut;
    using gr::blocks::testing::span::InputSpan;
    using gr::blocks::testing::span::OutputSpan;

    "known vector, ieee802_3"_test = [] {
        // anchor A, the ieee802_3 column: input byte, then the two chip bytes
        constexpr std::array<std::array<std::uint8_t, 3UZ>, 7UZ> kTable{{{0x00U, 0xAAU, 0xAAU}, {0xFFU, 0x55U, 0x55U}, {0x0FU, 0xAAU, 0x55U}, {0x55U, 0x99U, 0x99U}, {0xAAU, 0x66U, 0x66U}, {0xA5U, 0x66U, 0x99U}, {0x47U, 0x9AU, 0x95U}}};
        for (const auto& row : kTable) {
            const std::array<std::uint8_t, 1UZ> data{row[0UZ]};
            const auto                          chips = encoded(encoderSettings("ieee802_3"), std::span<const std::uint8_t>(data));
            expect(eq(chips.size(), 2UZ));
            expect(eq(chips[0UZ], row[1UZ])) << std::format("0x{:02X} encodes to 0x{:02X} 0x{:02X}", row[0UZ], row[1UZ], row[2UZ]);
            expect(eq(chips[1UZ], row[2UZ])) << std::format("0x{:02X} encodes to 0x{:02X} 0x{:02X}", row[0UZ], row[1UZ], row[2UZ]);
        }

        // anchor B: IEEE 802.3's preamble octet 0x55 and delimiter 0xD5, both taken least-significant-bit first,
        // which is the bit sequence 1 0 1 0 1 0 1 0 and 1 0 1 0 1 0 1 1
        const std::array<std::uint8_t, 1UZ> lsbPreamble{0x55U};
        const std::array<std::uint8_t, 1UZ> lsbDelimiter{0xD5U};
        const auto                          preambleLsb  = encoded(encoderSettings("ieee802_3", 8U, "LsbFirst"), std::span<const std::uint8_t>(lsbPreamble));
        const auto                          delimiterLsb = encoded(encoderSettings("ieee802_3", 8U, "LsbFirst"), std::span<const std::uint8_t>(lsbDelimiter));
        expect(that % (preambleLsb == std::vector<std::uint8_t>{0x66U, 0x66U}));
        expect(that % (delimiterLsb == std::vector<std::uint8_t>{0x66U, 0xA6U}));

        // the same two bit sequences carried most-significant-bit first are the items 0xAA and 0xAB
        const std::array<std::uint8_t, 1UZ> msbPreamble{0xAAU};
        const std::array<std::uint8_t, 1UZ> msbDelimiter{0xABU};
        const auto                          preambleMsb  = encoded(encoderSettings("ieee802_3"), std::span<const std::uint8_t>(msbPreamble));
        const auto                          delimiterMsb = encoded(encoderSettings("ieee802_3"), std::span<const std::uint8_t>(msbDelimiter));
        expect(that % (preambleMsb == std::vector<std::uint8_t>{0x66U, 0x66U}));
        expect(that % (delimiterMsb == std::vector<std::uint8_t>{0x66U, 0x65U}));

        // the two waveform properties the standard's own description states, asserted separately: the preamble is a
        // square wave with one transition per bit time, and the delimiter breaks it at exactly one place
        const auto preambleChips  = bitsOf(std::span<const std::uint8_t>(preambleLsb), 8U, false);
        const auto delimiterChips = bitsOf(std::span<const std::uint8_t>(delimiterLsb), 8U, false);
        expect(eq(preambleChips.size(), 16UZ));
        expect(eq(transitions(preambleChips), 8UZ)) << "one transition per bit time over eight bits";
        expect(eq(transitions(delimiterChips), 9UZ)) << "the delimiter adds exactly one boundary transition";
        for (std::size_t m = 0UZ; m < 8UZ; ++m) {
            expect(neq(preambleChips[2UZ * m], preambleChips[2UZ * m + 1UZ])) << "a transition at every bit center";
        }
    };

    "known vector, ge_thomas"_test = [] {
        // anchor A, the ge_thomas column
        constexpr std::array<std::array<std::uint8_t, 3UZ>, 7UZ> kTable{{{0x00U, 0x55U, 0x55U}, {0xFFU, 0xAAU, 0xAAU}, {0x0FU, 0x55U, 0xAAU}, {0x55U, 0x66U, 0x66U}, {0xAAU, 0x99U, 0x99U}, {0xA5U, 0x99U, 0x66U}, {0x47U, 0x65U, 0x6AU}}};
        for (const auto& row : kTable) {
            const std::array<std::uint8_t, 1UZ> data{row[0UZ]};
            const auto                          chips = encoded(encoderSettings("ge_thomas"), std::span<const std::uint8_t>(data));
            expect(eq(chips.size(), 2UZ));
            expect(eq(chips[0UZ], row[1UZ]));
            expect(eq(chips[1UZ], row[2UZ]));
        }

        // anchor C, MIL-STD-1553's own statement: a logic one is 1/0 and a logic zero is 0/1
        const std::array<std::uint8_t, 2UZ> oneThenZero{1U, 0U};
        const auto                          pairs = encoded(encoderSettings("ge_thomas", 1U), std::span<const std::uint8_t>(oneThenZero));
        expect(that % (pairs == std::vector<std::uint8_t>{1U, 0U, 0U, 1U})) << "chipPair(1) == (1,0) and chipPair(0) == (0,1)";

        // the identity that makes the table cheap to check: the two columns are bitwise complements, over all 256 items
        std::vector<std::uint8_t> all(256UZ);
        for (std::size_t value = 0UZ; value < all.size(); ++value) {
            all[value] = static_cast<std::uint8_t>(value);
        }
        const auto  ieee    = encoded(encoderSettings("ieee802_3"), std::span<const std::uint8_t>(all));
        const auto  thomas  = encoded(encoderSettings("ge_thomas"), std::span<const std::uint8_t>(all));
        std::size_t matched = 0UZ;
        for (std::size_t i = 0UZ; i < ieee.size(); ++i) {
            matched += static_cast<std::uint8_t>(~ieee[i]) == thomas[i] ? 1UZ : 0UZ;
        }
        expect(eq(matched, 512UZ)) << "every ge_thomas chip item is the complement of its ieee802_3 neighbor";
    };

    "code properties, over any input"_test = [] {
        Rng        rng;
        const auto data = rng.items(512UZ, 8U);
        for (const std::string_view convention : kConventions) {
            const auto chips = encoded(encoderSettings(convention), std::span<const std::uint8_t>(data));
            const auto bits  = bitsOf(std::span<const std::uint8_t>(data), 8U, true);
            const auto chip  = bitsOf(std::span<const std::uint8_t>(chips), 8U, true);
            expect(eq(chip.size(), 2UZ * bits.size()));

            std::size_t ones     = 0UZ;
            std::size_t worstRun = 0UZ;
            std::size_t run      = 1UZ;
            for (std::size_t j = 0UZ; j < chip.size(); ++j) {
                ones += chip[j];
                run      = j > 0UZ && chip[j] == chip[j - 1UZ] ? run + 1UZ : 1UZ;
                worstRun = std::max(worstRun, run);
            }
            expect(eq(ones, bits.size())) << std::format("{}: exactly 50 % ones in the chip stream", convention);
            expect(eq(worstRun, 2UZ)) << std::format("{}: the longest run of like chips is two", convention);

            int         disparity  = 0;
            std::size_t balanced   = 0UZ;
            std::size_t centers    = 0UZ;
            std::size_t boundaries = 0UZ;
            std::size_t equalPairs = 0UZ;
            for (std::size_t m = 0UZ; m < bits.size(); ++m) {
                disparity += chip[2UZ * m] == 1U ? 1 : -1;
                disparity += chip[2UZ * m + 1UZ] == 1U ? 1 : -1;
                balanced += disparity == 0 ? 1UZ : 0UZ;
                centers += chip[2UZ * m] != chip[2UZ * m + 1UZ] ? 1UZ : 0UZ;
                if (m + 1UZ < bits.size()) {
                    equalPairs += bits[m] == bits[m + 1UZ] ? 1UZ : 0UZ;
                    boundaries += (chip[2UZ * m + 1UZ] != chip[2UZ * m + 2UZ]) == (bits[m] == bits[m + 1UZ]) ? 1UZ : 0UZ;
                }
            }
            expect(eq(balanced, bits.size())) << std::format("{}: the running disparity is zero at every bit boundary", convention);
            expect(eq(centers, bits.size())) << std::format("{}: a transition at every bit center", convention);
            expect(eq(boundaries, bits.size() - 1UZ)) << std::format("{}: a boundary transition exactly where adjacent bits are equal", convention);
            expect(gt(equalPairs, 0UZ)) << "the sample must actually contain equal adjacent bits";
        }
    };

    "rate, state and tag placement"_test = [] {
        Rng        rng;
        const auto data = rng.items(2048UZ, 8U);

        // anchor K: 1 input item to 2 output items, and no state carried between runs
        auto       block = make<ManchesterEncoder>(encoderSettings("ieee802_3"));
        const auto first = gr::blocks::testing::span::run<std::uint8_t>(block, std::span<const std::uint8_t>(data), 0UZ, 2UZ);
        expect(eq(first.samples.size(), 2UZ * data.size()));
        expect(eq(first.consumed, data.size()));
        const auto again = gr::blocks::testing::span::run<std::uint8_t>(block, std::span<const std::uint8_t>(data), 7UZ, 2UZ);
        expect(that % (again.samples == first.samples)) << "the encoder is a pure function of its input: no warm-up, no carried state";

        // The encoder half: a tag at input item t arrives at output item 2t
        for (const std::size_t t : {0UZ, 1UZ, 7UZ, 8UZ, 1000UZ}) {
            const std::array<gr::Tag, 1UZ> planted{gr::Tag{t, {{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), std::string("mark")}}}};
            const auto                     seen = throughGraph<ManchesterEncoder>(encoderSettings("ieee802_3"), std::span<const std::uint8_t>(data), std::span<const gr::Tag>(planted));
            expect(that % (offsetsOf(seen.tags, gr::tag::TRIGGER_NAME.shortKey()) == std::vector<std::size_t>{2UZ * t})) << std::format("a tag at input item {} belongs at output item {}", t, 2UZ * t);
            expect(eq(seen.samples.size(), 2UZ * data.size()));
        }

        // two tags at one input item stay two tags, in order, rather than merging
        const std::array<gr::Tag, 2UZ> pair{gr::Tag{4UZ, {{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), std::string("first")}}}, gr::Tag{5UZ, {{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), std::string("second")}}}};
        const auto                     both = throughGraph<ManchesterEncoder>(encoderSettings("ieee802_3"), std::span<const std::uint8_t>(data), std::span<const gr::Tag>(pair));
        expect(that % (offsetsOf(both.tags, gr::tag::TRIGGER_NAME.shortKey()) == std::vector<std::size_t>{8UZ, 10UZ}));

        // the core doubles a forwarded sample_rate, and the block owns no such setting
        const std::array<gr::Tag, 1UZ> rated{gr::Tag{0UZ, {{gr::property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()), 1.0e6F}}}};
        const auto                     scaled  = throughGraph<ManchesterEncoder>(encoderSettings("ieee802_3"), std::span<const std::uint8_t>(data), std::span<const gr::Tag>(rated));
        bool                           sawRate = false;
        for (const gr::Tag& seenTag : scaled.tags) {
            if (sampleRateOf(seenTag) > 0.0F) {
                sawRate = true;
                expect(eq(sampleRateOf(seenTag), 2.0e6F)) << "1 MHz in, 2 MHz out: the chip rate is twice the bit rate";
            }
        }
        expect(sawRate) << "the sample_rate tag reached the sink";
    };

    "bit-order interoperability"_test = [] {
        // anchor I: unpacking to one bit an item, coding, and packing back is the same byte stream as coding whole
        // bytes. The packing is the bit-packing kernel's own, which is where this family's item model lives.
        Rng        rng;
        const auto data = rng.items(256UZ, 8U);
        for (const std::string_view convention : kConventions) {
            for (const std::string_view order : kOrders) {
                const gr::digital::BitOrder mode = order == "MsbFirst" ? gr::digital::BitOrder::MsbFirst : gr::digital::BitOrder::LsbFirst;

                gr::digital::BitRepack unpack;
                gr::digital::BitRepack pack;
                gr::digital::configureRepack(unpack, 8U, 1U, mode, mode);
                gr::digital::configureRepack(pack, 1U, 8U, mode, mode);

                std::vector<std::uint8_t> unpacked(8UZ * data.size());
                gr::digital::repack(unpack, std::span<const std::uint8_t>(data), std::span<std::uint8_t>(unpacked));
                const auto chips = encoded(encoderSettings(convention, 1U, order), std::span<const std::uint8_t>(unpacked));

                std::vector<std::uint8_t> packed(chips.size() / 8UZ);
                gr::digital::repack(pack, std::span<const std::uint8_t>(chips), std::span<std::uint8_t>(packed));
                const auto direct = encoded(encoderSettings(convention, 8U, order), std::span<const std::uint8_t>(data));
                expect(that % (packed == direct)) << std::format("{}/{}: one bit an item composes to the byte-wide coding", convention, order);
            }
        }

        // and the negative half: the two orders are not interchangeable
        const auto  msb       = encoded(encoderSettings("ieee802_3", 8U, "MsbFirst"), std::span<const std::uint8_t>(data));
        const auto  lsb       = encoded(encoderSettings("ieee802_3", 8U, "LsbFirst"), std::span<const std::uint8_t>(data));
        std::size_t differing = 0UZ;
        for (std::size_t i = 0UZ; i < msb.size(); ++i) {
            differing += msb[i] != lsb[i] ? 1UZ : 0UZ;
        }
        expect(gt(differing, msb.size() / 2UZ)) << "the two orders differ on far more than half the items";
    };

    "tag policy"_test = [] {
        // the compile-time half, asserted as a predicate: a rate changer does not get blind pass-through
        static_assert(!gr::block::kUnfilteredTagPropagationAdmissible<ManchesterEncoder>, "a 1:2 block must not claim UnfilteredTagPropagation");
        expect(!gr::block::kUnfilteredTagPropagationAdmissible<ManchesterEncoder>);

        // the runtime half: a reserved key survives, a custom key does not
        Rng                            rng;
        const auto                     data = rng.items(64UZ, 8U);
        const std::array<gr::Tag, 1UZ> mixed{gr::Tag{8UZ, {{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), std::string("kept")}, {gr::property_map::key_type(kCustomKey), std::string("dropped")}}}};
        const auto                     seen = throughGraph<ManchesterEncoder>(encoderSettings("ieee802_3"), std::span<const std::uint8_t>(data), std::span<const gr::Tag>(mixed));
        expect(that % (offsetsOf(seen.tags, gr::tag::TRIGGER_NAME.shortKey()) == std::vector<std::size_t>{16UZ}));
        expect(that % (offsetsOf(seen.tags, kCustomKey) == std::vector<std::size_t>{})) << "a custom-key protocol does not survive a rate changer";
    };

    "a stream value cannot stop the graph"_test = [] {
        constexpr std::size_t           kItems = 100000UZ;
        const std::vector<std::uint8_t> ones(kItems, 0xFFU);
        const std::vector<std::uint8_t> low(kItems, 0x07U);
        const auto                      fromOnes = encoded(encoderSettings("ieee802_3", 3U), std::span<const std::uint8_t>(ones));
        const auto                      fromLow  = encoded(encoderSettings("ieee802_3", 3U), std::span<const std::uint8_t>(low));
        expect(that % (fromOnes == fromLow)) << "only the low bits_per_item bits are read";
        expect(eq(fromOnes.size(), 2UZ * kItems));
        expect(std::ranges::all_of(fromOnes, [](std::uint8_t item) { return item < 8U; })) << "no output item carries a bit above bits_per_item";
    };

    "chunk independence"_test = [] {
        Rng rng;
        for (const gr::Size_t width : {gr::Size_t{1}, gr::Size_t{8}}) {
            const auto data  = rng.items(4096UZ, static_cast<unsigned>(width));
            const auto whole = encoded(encoderSettings("ieee802_3", width), std::span<const std::uint8_t>(data));
            for (const std::size_t chunk : {2UZ, 4UZ, 18UZ, 4096UZ}) {
                const auto chunked = encoded(encoderSettings("ieee802_3", width), std::span<const std::uint8_t>(data), chunk);
                expect(that % (chunked == whole)) << std::format("bits_per_item {}, chunk {}", width, chunk);
            }
        }
    };

    "validation rejects, and the previous configuration survives"_test = [] {
        expect(throws([] {
            auto block = make<ManchesterEncoder>({{"convention", std::string("")}}); // the empty spelling is the default, so the batch moves nothing
            block.start();
        })) << "there is no default convention";
        expect(throws([] { std::ignore = make<ManchesterEncoder>({{"convention", std::string("ieee802.3")}}); })) << "a typo must not silently become a convention";
        expect(throws([] { std::ignore = make<ManchesterEncoder>(encoderSettings("ieee802_3", 0U)); }));
        expect(throws([] { std::ignore = make<ManchesterEncoder>(encoderSettings("ieee802_3", 9U)); }));
        expect(throws([] { std::ignore = make<ManchesterEncoder>(encoderSettings("ieee802_3", 8U, "big_endian")); }));
        expect(nothrow([] { std::ignore = make<ManchesterEncoder>(encoderSettings("ge_thomas", 1U, "LsbFirst")); }));

        const std::array<std::uint8_t, 1UZ> data{0x47U};
        auto                                block  = make<ManchesterEncoder>(encoderSettings("ieee802_3"));
        const auto                          before = gr::blocks::testing::span::run<std::uint8_t>(block, std::span<const std::uint8_t>(data), 0UZ, 2UZ);
        expect(throws([&block] { restage(block, {{"bits_per_item", gr::Size_t{9}}}); }));
        const auto after = gr::blocks::testing::span::run<std::uint8_t>(block, std::span<const std::uint8_t>(data), 0UZ, 2UZ);
        expect(that % (after.samples == before.samples)) << "a rejected setting leaves the previous configuration whole";
    };

    "an unconfigured block is inert"_test = [] {
        ManchesterEncoder block;
        block.settings().init();

        const std::array<std::uint8_t, 4UZ> data{1U, 2U, 3U, 4U};
        std::array<std::uint8_t, 8UZ>       room{};
        InputSpan<std::uint8_t>             inSpan{std::span<const std::uint8_t>(data)};
        OutputSpan<std::uint8_t>            outSpan{std::span<std::uint8_t>(room)};
        expect(that % (block.processBulk(inSpan, outSpan) == gr::work::Status::ERROR));
        expect(eq(inSpan.consumed, 0UZ));
        expect(eq(outSpan.count, 0UZ));
    };
};

const boost::ut::suite<"Manchester decoder"> manchesterDecoderTests = [] {
    using namespace boost::ut;

    "the wrong convention is silent"_test = [] {
        // anchor D: chips encoded under one convention and decoded under the other, aligned
        constexpr std::array<std::array<std::uint8_t, 4UZ>, 4UZ> kTable{{{0x47U, 0x9AU, 0x95U, 0xB8U}, {0x00U, 0xAAU, 0xAAU, 0xFFU}, {0xAAU, 0x66U, 0x66U, 0x55U}, {0xA5U, 0x66U, 0x99U, 0x5AU}}};
        for (const auto& row : kTable) {
            const std::array<std::uint8_t, 2UZ> chips{row[1UZ], row[2UZ]};
            const auto                          right = decoded(decoderSettings("ieee802_3"), std::span<const std::uint8_t>(chips));
            const auto                          wrong = decoded(decoderSettings("ge_thomas"), std::span<const std::uint8_t>(chips));

            expect(that % (right.bits == std::vector<std::uint8_t>{row[0UZ]}));
            expect(that % (wrong.bits == std::vector<std::uint8_t>{row[3UZ]})) << std::format("0x{:02X} read under the other convention is 0x{:02X}", row[0UZ], row[3UZ]);
            expect(neq(right.bits[0UZ], wrong.bits[0UZ])) << "the two readings differ";
            expect(eq(static_cast<std::uint8_t>(right.bits[0UZ] ^ wrong.bits[0UZ]), std::uint8_t{0xFFU})) << "and differ by an exact complement";

            // the half that is the point: neither reading raises anything at all
            expect(that % (right.flags == std::vector<std::uint8_t>{0x00U}));
            expect(that % (wrong.flags == std::vector<std::uint8_t>{0x00U})) << "the wrong convention flags nothing, which is why it has no default";
        }
    };

    "round trip"_test = [] {
        // anchor J: 20 of 20, exact from item 0 with an all-zero violation stream
        Rng         rng;
        std::size_t exact = 0UZ;
        for (const gr::Size_t width : {gr::Size_t{1}, gr::Size_t{2}, gr::Size_t{3}, gr::Size_t{4}, gr::Size_t{8}}) {
            for (const std::string_view order : kOrders) {
                for (const std::string_view convention : kConventions) {
                    const auto data  = rng.items(4096UZ, static_cast<unsigned>(width));
                    const auto chips = encoded(encoderSettings(convention, width, order), std::span<const std::uint8_t>(data));
                    const auto back  = decoded(decoderSettings(convention, width, order), std::span<const std::uint8_t>(chips));
                    const bool ok    = back.bits == data && std::ranges::all_of(back.flags, [](std::uint8_t flag) { return flag == 0U; });
                    exact += ok ? 1UZ : 0UZ;
                    expect(ok) << std::format("{} bits, {}, {}", width, order, convention);
                }
            }
        }
        expect(eq(exact, 20UZ));
    };

    "violations are reported and not absorbed"_test = [] {
        // the MIL-STD-1553 command sync's shape -- three bit times, positive for the first one and a half and
        // negative for the following one and a half -- planted between legal pairs, one chip an item
        const std::vector<std::uint8_t> chips{0U, 1U, 1U, 0U, 1U, 1U, 1U, 0U, 0U, 0U, 0U, 1U, 1U, 0U};
        auto                            block = make<ManchesterDecoder>(decoderSettings("ieee802_3", 1U));
        const auto                      seen  = decodedBy(block, std::span<const std::uint8_t>(chips));

        expect(that % (seen.flags == std::vector<std::uint8_t>{0U, 0U, 1U, 0U, 1U, 0U, 0U})) << "flagged at exactly the invalid pairs and nowhere else";
        expect(that % (seen.bits == std::vector<std::uint8_t>{1U, 0U, 0U, 0U, 1U, 1U, 0U}));
        expect(eq(seen.bits.size(), chips.size() / 2UZ)) << "nothing is dropped: one item per two chips";
        expect(eq(seen.bits[2UZ], std::uint8_t{0U})) << "a flagged position carries the common chip corrected by the convention";
        expect(eq(seen.bits[4UZ], std::uint8_t{1U}));

        expect(eq(block._coder.gridParity, std::uint8_t{0U})) << "a violation does not move the pairing grid";
        expect(eq(block.nOrphanItems(), 0ULL));
        expect(eq(block.nDroppedChips(), 0ULL));
    };

    "(out, violation) is lossless"_test = [] {
        Rng  rng;
        auto chips = rng.items(8192UZ, 1U); // arbitrary chips, violations and all
        for (const std::string_view convention : kConventions) {
            const auto seen = decoded(decoderSettings(convention, 1U), std::span<const std::uint8_t>(chips));
            const auto k    = static_cast<unsigned>(gr::digital::conventionXor(gr::digital::conventionFromName(convention)));
            expect(that % (rebuiltChips(seen.bits, seen.flags, k) == chips)) << std::format("{}: the chip stream comes back exactly", convention);
        }
    };

    "single-chip errors"_test = [] {
        // anchor F, all four clauses
        const std::array<std::uint8_t, 1UZ> data{0xA5U};
        const auto                          clean = encoded(encoderSettings("ieee802_3", 1U), std::span<const std::uint8_t>(data)); // 16 chips, one an item
        const auto                          truth = decoded(decoderSettings("ieee802_3", 1U), std::span<const std::uint8_t>(clean));

        std::size_t flagged = 0UZ;
        for (std::size_t j = 0UZ; j < clean.size(); ++j) {
            auto broken = clean;
            broken[j] ^= 1U;
            const auto seen = decoded(decoderSettings("ieee802_3", 1U), std::span<const std::uint8_t>(broken));
            flagged += seen.flags[j / 2UZ] != 0U ? 1UZ : 0UZ;
            if (j % 2UZ == 0UZ) {
                expect(neq(seen.bits[j / 2UZ], truth.bits[j / 2UZ])) << "flipping the first chip inverts the decoded bit";
            } else {
                expect(eq(seen.bits[j / 2UZ], truth.bits[j / 2UZ])) << "flipping the second chip leaves the decoded bit correct";
            }
        }
        expect(eq(flagged, clean.size())) << "every single-chip error makes its pair invalid, so every one of them is flagged";

        // and the negative result: both chips of one pair is the smallest undetectable error
        for (std::size_t m = 0UZ; m < clean.size() / 2UZ; ++m) {
            auto broken = clean;
            broken[2UZ * m] ^= 1U;
            broken[2UZ * m + 1UZ] ^= 1U;
            const auto seen = decoded(decoderSettings("ieee802_3", 1U), std::span<const std::uint8_t>(broken));
            expect(eq(seen.flags[m], std::uint8_t{0U})) << "a both-chip flip stays a legal pair and is not detected";
            expect(neq(seen.bits[m], truth.bits[m])) << "and it inverts the bit silently";
        }
    };

    "misalignment"_test = [] {
        // anchor E: the same chips read under the right convention, one chip out of phase
        constexpr std::array<std::array<std::uint8_t, 5UZ>, 3UZ> kTable{{{0x47U, 0x9AU, 0x95U, 0x5CU, 0xE4U}, {0x00U, 0xAAU, 0xAAU, 0x7FU, 0x80U}, {0xAAU, 0x66U, 0x66U, 0xAAU, 0xFFU}}};
        for (const auto& row : kTable) {
            const std::array<std::uint8_t, 2UZ> chips{row[1UZ], row[2UZ]};
            const auto                          seen = decoded(decoderSettings("ieee802_3", 8U, "MsbFirst", 1U), std::span<const std::uint8_t>(chips));
            expect(that % (seen.bits == std::vector<std::uint8_t>{row[3UZ]})) << std::format("0x{:02X} misaligned decodes to 0x{:02X}", row[0UZ], row[3UZ]);
            expect(that % (seen.flags == std::vector<std::uint8_t>{row[4UZ]})) << std::format("0x{:02X} misaligned flags 0x{:02X}", row[0UZ], row[4UZ]);
        }

        // the general rule as a property, one chip an item so that a bit index is an item index
        Rng        rng;
        const auto bits  = rng.items(4096UZ, 1U);
        const auto chips = encoded(encoderSettings("ieee802_3", 1U), std::span<const std::uint8_t>(bits));
        const auto seen  = decoded(decoderSettings("ieee802_3", 1U, "MsbFirst", 1U), std::span<const std::uint8_t>(chips));
        expect(eq(seen.bits.size(), bits.size()));

        std::size_t agreed = 0UZ;
        std::size_t marked = 0UZ;
        for (std::size_t m = 1UZ; m < bits.size(); ++m) {
            const bool differ = bits[m - 1UZ] != bits[m];
            marked += seen.flags[m] != 0U ? 1UZ : 0UZ;
            agreed += (seen.flags[m] != 0U) == differ && seen.bits[m] == static_cast<std::uint8_t>(bits[m - 1UZ] ^ 1U) ? 1UZ : 0UZ;
        }
        expect(eq(agreed, bits.size() - 1UZ)) << "flagged exactly where adjacent data bits differ, and the bit is the complement of the earlier one";
        const double fraction = static_cast<double>(marked) / static_cast<double>(bits.size() - 1UZ);
        expect(fraction > 0.45 && fraction < 0.55) << std::format("uniform random data flags about half the positions, got {:.3f}", fraction);

        // the negative half: a constant stream is silent apart from the orphan, and reads as the wrong convention would
        const std::vector<std::uint8_t> constant(1024UZ, 0U);
        const auto                      flat    = encoded(encoderSettings("ieee802_3", 1U), std::span<const std::uint8_t>(constant));
        const auto                      skewed  = decoded(decoderSettings("ieee802_3", 1U, "MsbFirst", 1U), std::span<const std::uint8_t>(flat));
        const auto                      swapped = decoded(decoderSettings("ge_thomas", 1U), std::span<const std::uint8_t>(flat));
        expect(eq(skewed.flags[0UZ], std::uint8_t{1U})) << "the orphan is flagged";
        expect(std::ranges::all_of(std::span<const std::uint8_t>(skewed.flags).subspan(1UZ), [](std::uint8_t flag) { return flag == 0U; })) << "and nothing else is";
        expect(std::ranges::equal(std::span<const std::uint8_t>(skewed.bits).subspan(1UZ), std::span<const std::uint8_t>(swapped.bits).subspan(1UZ))) << "on a constant stream misalignment and the wrong convention are the same output";
    };

    "rate, tag placement and sample_rate"_test = [] {
        Rng        rng;
        const auto chips = encoded(encoderSettings("ieee802_3"), std::span<const std::uint8_t>(rng.items(1024UZ, 8U)));

        // anchor K: two input items to one output item and one violation item, one chip of carried state
        auto       block = make<ManchesterDecoder>(decoderSettings("ieee802_3"));
        const auto seen  = decodedBy(block, std::span<const std::uint8_t>(chips));
        expect(eq(seen.bits.size(), chips.size() / 2UZ));
        expect(eq(seen.flags.size(), chips.size() / 2UZ));
        expect(!block._coder.hasCarry) << "an aligned grid holds nothing between calls";

        // The decoder half: a tag at input item t arrives at output item floor(t/2), both parities.
        // At bits_per_item = 8 the request a tag can make is parity 0, which is the grid already in force.
        for (const std::size_t t : {0UZ, 1UZ, 6UZ, 7UZ, 1000UZ}) {
            const std::array<gr::Tag, 1UZ> planted{gr::Tag{t, {{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), std::string("mark")}}}};
            const auto                     graphed = decoderGraph(decoderSettings("ieee802_3"), std::span<const std::uint8_t>(chips), std::span<const gr::Tag>(planted));
            expect(that % (offsetsOf(graphed.tags, gr::tag::TRIGGER_NAME.shortKey()) == std::vector<std::size_t>{t / 2UZ})) << std::format("a tag at input item {} belongs at output item {}", t, t / 2UZ);
        }

        // the two input items of one chunk map to one output item, and neither of their tags is dropped on the way
        const std::array<gr::Tag, 2UZ> pair{gr::Tag{4UZ, {{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), std::string("first")}}}, gr::Tag{5UZ, {{gr::property_map::key_type(gr::tag::TRIGGER_OFFSET.shortKey()), 3.0F}}}};
        const auto                     both = decoderGraph(decoderSettings("ieee802_3"), std::span<const std::uint8_t>(chips), std::span<const gr::Tag>(pair));
        expect(that % (offsetsOf(both.tags, gr::tag::TRIGGER_NAME.shortKey()) == std::vector<std::size_t>{2UZ}));
        expect(that % (offsetsOf(both.tags, gr::tag::TRIGGER_OFFSET.shortKey()) == std::vector<std::size_t>{2UZ})) << "the second tag lands on the same output item rather than being dropped";

        // the core halves a forwarded sample_rate, and the block owns no such setting
        const std::array<gr::Tag, 1UZ> rated{gr::Tag{0UZ, {{gr::property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()), 2.0e6F}}}};
        const auto                     scaled  = decoderGraph(decoderSettings("ieee802_3"), std::span<const std::uint8_t>(chips), std::span<const gr::Tag>(rated));
        bool                           sawRate = false;
        for (const gr::Tag& seenTag : scaled.tags) {
            if (sampleRateOf(seenTag) > 0.0F) {
                sawRate = true;
                expect(eq(sampleRateOf(seenTag), 1.0e6F)) << "2 MHz of chips in, 1 MHz of bits out";
            }
        }
        expect(sawRate) << "the sample_rate tag reached the sink";
    };

    "alignment contract"_test = [] {
        Rng        rng;
        const auto bits  = rng.items(4096UZ, 1U);
        const auto chips = encoded(encoderSettings("ieee802_3", 1U), std::span<const std::uint8_t>(bits));

        { // (a) a chip_phase = 1 run begins with exactly one flagged orphan and nothing else is ever spent
            auto       skewed  = make<ManchesterDecoder>(decoderSettings("ieee802_3", 1U, "MsbFirst", 1U));
            const auto shifted = decodedBy(skewed, std::span<const std::uint8_t>(chips));
            expect(eq(shifted.flags[0UZ], std::uint8_t{1U})) << "the first output item is the orphan";
            expect(eq(skewed.nOrphanItems(), 1ULL)) << "exactly one, at item 0";
            expect(eq(skewed.nDroppedChips(), 0ULL)) << "steady phase 1 costs one chip of state and no items at all";

            const auto aligned = decoded(decoderSettings("ieee802_3", 1U), std::span<const std::uint8_t>(chips).subspan(1UZ, chips.size() - 2UZ));
            expect(std::ranges::equal(std::span<const std::uint8_t>(shifted.bits).subspan(1UZ), aligned.bits)) << "from item 1 it is an aligned decode of the same chips shifted by one";
            expect(std::ranges::equal(std::span<const std::uint8_t>(shifted.flags).subspan(1UZ), aligned.flags));
        }

        { // (b) a tag re-grids to the parity of its own chip index, effective at the next chunk boundary
            for (const std::size_t t : {8UZ, 9UZ}) {
                const std::array<gr::Tag, 1UZ> planted{gr::Tag{t, {{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), std::string("sync")}}}};
                auto                           tagged = make<ManchesterDecoder>(decoderSettings("ieee802_3", 1U));
                const auto                     seen   = decodedBy(tagged, std::span<const std::uint8_t>(chips), 2UZ, std::span<const gr::Tag>(planted));
                const std::size_t              at     = ((t + 1UZ) / 2UZ) * 2UZ; // the chunk boundary the change lands on
                const auto                     fresh  = decoded(decoderSettings("ieee802_3", 1U, "MsbFirst", static_cast<gr::Size_t>(t % 2UZ)), std::span<const std::uint8_t>(chips).subspan(at));
                expect(std::ranges::equal(std::span<const std::uint8_t>(seen.bits).subspan(at / 2UZ), fresh.bits)) << std::format("t = {}: the output from the boundary is a freshly gridded decode", t);
                expect(std::ranges::equal(std::span<const std::uint8_t>(seen.flags).subspan(at / 2UZ), fresh.flags));
                expect(eq(tagged._coder.gridParity, static_cast<std::uint8_t>(t % 2UZ)));
            }

            // at an even bits_per_item the product t * bits_per_item is always even, so a tag can only ask for parity 0
            const std::array<gr::Tag, 1UZ> odd{gr::Tag{7UZ, {{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), std::string("sync")}}}};
            auto                           wide = make<ManchesterDecoder>(decoderSettings("ieee802_3", 8U, "MsbFirst", 1U));
            std::ignore                         = decodedBy(wide, std::span<const std::uint8_t>(chips), 2UZ, std::span<const gr::Tag>(odd));
            expect(eq(wide._coder.gridParity, std::uint8_t{0U})) << "the request was parity 0 and it was taken";
        }

        { // (c) value filtering, a non-string value, and disabling
            const std::array<gr::Tag, 1UZ> other{gr::Tag{9UZ, {{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), std::string("other")}}}};
            const std::array<gr::Tag, 1UZ> number{gr::Tag{9UZ, {{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), gr::Size_t{7}}}}};
            const std::array<gr::Tag, 1UZ> right{gr::Tag{9UZ, {{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), std::string("sync")}}}};

            gr::property_map filtered   = decoderSettings("ieee802_3", 1U);
            filtered["align_tag_value"] = std::string("sync");
            const auto plain            = decoded(decoderSettings("ieee802_3", 1U), std::span<const std::uint8_t>(chips), 2UZ);

            expect(that % (decoded(filtered, std::span<const std::uint8_t>(chips), 2UZ, std::span<const gr::Tag>(other)).bits == plain.bits)) << "a different value at the key does not re-grid";
            expect(that % (decoded(filtered, std::span<const std::uint8_t>(chips), 2UZ, std::span<const gr::Tag>(number)).bits == plain.bits)) << "a non-string value at the key does not re-grid";
            expect(that % (decoded(filtered, std::span<const std::uint8_t>(chips), 2UZ, std::span<const gr::Tag>(right)).bits != plain.bits)) << "the value it was told to wait for does";

            gr::property_map disabled = decoderSettings("ieee802_3", 1U);
            disabled["align_tag_key"] = std::string("");
            expect(that % (decoded(disabled, std::span<const std::uint8_t>(chips), 2UZ, std::span<const gr::Tag>(right)).bits == plain.bits)) << "an empty key ignores every tag";
        }

        { // (d) the tag is read and forwarded, never consumed
            const std::array<gr::Tag, 1UZ> planted{gr::Tag{8UZ, {{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), std::string("sync")}}}};
            const auto                     graphed = decoderGraph(decoderSettings("ieee802_3", 1U), std::span<const std::uint8_t>(chips), std::span<const gr::Tag>(planted));
            expect(that % (offsetsOf(graphed.tags, gr::tag::TRIGGER_NAME.shortKey()) == std::vector<std::size_t>{4UZ})) << "a downstream extractor still needs it";
        }

        { // (e) the grid-change budget of anchor G, in both directions and in neither
            const std::array<gr::Tag, 1UZ> toOne{gr::Tag{9UZ, {{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), std::string("sync")}}}};
            const std::array<gr::Tag, 1UZ> toZero{gr::Tag{8UZ, {{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), std::string("sync")}}}};

            auto       rising = make<ManchesterDecoder>(decoderSettings("ieee802_3", 1U));
            const auto up     = decodedBy(rising, std::span<const std::uint8_t>(chips), 2UZ, std::span<const gr::Tag>(toOne));
            expect(eq(up.bits.size(), chips.size() / 2UZ)) << "every call still publishes one item per two input items";
            expect(eq(rising.nOrphanItems(), 1ULL)) << "0 -> 1 fabricates exactly one flagged orphan";
            expect(eq(rising.nDroppedChips(), 0ULL)) << "and loses no chip";

            auto       falling = make<ManchesterDecoder>(decoderSettings("ieee802_3", 1U, "MsbFirst", 1U));
            const auto down    = decodedBy(falling, std::span<const std::uint8_t>(chips), 2UZ, std::span<const gr::Tag>(toZero));
            expect(eq(down.bits.size(), chips.size() / 2UZ));
            expect(eq(falling.nDroppedChips(), 1ULL)) << "1 -> 0 drops exactly the held chip";
            expect(eq(falling.nOrphanItems(), 1ULL)) << "and fabricates nothing beyond the run's own opening orphan";

            auto       steady = make<ManchesterDecoder>(decoderSettings("ieee802_3", 1U));
            const auto same   = decodedBy(steady, std::span<const std::uint8_t>(chips), 2UZ, std::span<const gr::Tag>(toZero));
            expect(eq(same.bits.size(), chips.size() / 2UZ));
            expect(eq(steady.nOrphanItems(), 0ULL)) << "requesting the parity already in force costs nothing";
            expect(eq(steady.nDroppedChips(), 0ULL));
        }

        {                                                    // (f) violations never realign the decoder, however many of them arrive
            std::vector<std::uint8_t> violating(4096UZ, 1U); // every pair is 11
            auto                      block = make<ManchesterDecoder>(decoderSettings("ieee802_3", 1U));
            const auto                seen  = decodedBy(block, std::span<const std::uint8_t>(violating));
            expect(std::ranges::all_of(seen.flags, [](std::uint8_t flag) { return flag == 1U; })) << "every pair is reported";
            expect(eq(block.chip_phase.value, gr::Size_t{0})) << "and the setting is untouched";
            expect(eq(block._coder.gridParity, std::uint8_t{0U}));
            expect(eq(block.nOrphanItems(), 0ULL));
            expect(eq(block.nDroppedChips(), 0ULL));
        }
    };

    "tag policy"_test = [] {
        static_assert(!gr::block::kUnfilteredTagPropagationAdmissible<ManchesterDecoder>, "a 2:1 block must not claim UnfilteredTagPropagation");
        expect(!gr::block::kUnfilteredTagPropagationAdmissible<ManchesterDecoder>);

        Rng                            rng;
        const auto                     chips = encoded(encoderSettings("ieee802_3"), std::span<const std::uint8_t>(rng.items(64UZ, 8U)));
        const std::array<gr::Tag, 1UZ> mixed{gr::Tag{8UZ, {{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), std::string("kept")}, {gr::property_map::key_type(kCustomKey), std::string("dropped")}}}};
        const auto                     seen = decoderGraph(decoderSettings("ieee802_3"), std::span<const std::uint8_t>(chips), std::span<const gr::Tag>(mixed));
        expect(that % (offsetsOf(seen.tags, gr::tag::TRIGGER_NAME.shortKey()) == std::vector<std::size_t>{4UZ}));
        expect(that % (offsetsOf(seen.tags, kCustomKey) == std::vector<std::size_t>{})) << "a custom-key protocol does not survive a rate changer";
    };

    "a stream value cannot stop the graph"_test = [] {
        constexpr std::size_t           kItems = 100000UZ;
        const std::vector<std::uint8_t> ones(kItems, 0xFFU);
        const std::vector<std::uint8_t> low(kItems, 0x07U);
        const auto                      fromOnes = decoded(decoderSettings("ieee802_3", 3U), std::span<const std::uint8_t>(ones));
        const auto                      fromLow  = decoded(decoderSettings("ieee802_3", 3U), std::span<const std::uint8_t>(low));
        expect(that % (fromOnes.bits == fromLow.bits)) << "only the low bits_per_item bits are read";
        expect(that % (fromOnes.flags == fromLow.flags));
        expect(eq(fromOnes.bits.size(), kItems / 2UZ));
        expect(std::ranges::all_of(fromOnes.bits, [](std::uint8_t item) { return item < 8U; }));
        expect(std::ranges::all_of(fromOnes.flags, [](std::uint8_t item) { return item < 8U; }));
    };

    "chunk independence"_test = [] {
        Rng rng;
        for (const gr::Size_t width : {gr::Size_t{1}, gr::Size_t{8}}) {
            const auto                     data  = rng.items(4096UZ, static_cast<unsigned>(width));
            const auto                     chips = encoded(encoderSettings("ieee802_3", width), std::span<const std::uint8_t>(data));
            const std::array<gr::Tag, 1UZ> planted{gr::Tag{101UZ, {{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), std::string("sync")}}}};
            const auto                     whole = decoded(decoderSettings("ieee802_3", width), std::span<const std::uint8_t>(chips), 0UZ, std::span<const gr::Tag>(planted));
            for (const std::size_t chunk : {2UZ, 4UZ, 18UZ, 4096UZ}) {
                const auto chunked = decoded(decoderSettings("ieee802_3", width), std::span<const std::uint8_t>(chips), chunk, std::span<const gr::Tag>(planted));
                expect(that % (chunked.bits == whole.bits)) << std::format("bits_per_item {}, chunk {}", width, chunk);
                expect(that % (chunked.flags == whole.flags)) << std::format("bits_per_item {}, chunk {}, violations", width, chunk);
            }
        }
    };

    "validation rejects, and the previous configuration survives"_test = [] {
        expect(throws([] {
            auto block = make<ManchesterDecoder>({{"convention", std::string("")}}); // the empty spelling is the default, so the batch moves nothing
            block.start();
        })) << "there is no default convention";
        expect(throws([] { std::ignore = make<ManchesterDecoder>({{"convention", std::string("thomas")}}); }));
        expect(throws([] { std::ignore = make<ManchesterDecoder>(decoderSettings("ieee802_3", 0U)); }));
        expect(throws([] { std::ignore = make<ManchesterDecoder>(decoderSettings("ieee802_3", 9U)); }));
        expect(throws([] { std::ignore = make<ManchesterDecoder>(decoderSettings("ieee802_3", 8U, "little_endian")); }));
        expect(throws([] { std::ignore = make<ManchesterDecoder>(decoderSettings("ieee802_3", 8U, "MsbFirst", 2U)); })) << "a chip phase is 0 or 1";
        expect(nothrow([] { std::ignore = make<ManchesterDecoder>(decoderSettings("ge_thomas", 1U, "LsbFirst", 1U)); }));

        const std::array<std::uint8_t, 2UZ> chips{0x9AU, 0x95U};
        auto                                block  = make<ManchesterDecoder>(decoderSettings("ieee802_3"));
        const auto                          before = decodedBy(block, std::span<const std::uint8_t>(chips));
        expect(throws([&block] { restage(block, {{"chip_phase", gr::Size_t{2}}}); }));
        const auto after = decodedBy(block, std::span<const std::uint8_t>(chips));
        expect(that % (after.bits == before.bits)) << "a rejected setting leaves the previous configuration whole";
        expect(that % (after.flags == before.flags));
    };

    "a settings change re-grids only when it moves the grid"_test = [] {
        const std::array<std::uint8_t, 2UZ> chips{0x9AU, 0x95U};
        auto                                block = make<ManchesterDecoder>(decoderSettings("ieee802_3", 1U, "MsbFirst", 1U));
        std::ignore                               = decodedBy(block, std::span<const std::uint8_t>(chips));
        expect(block._coder.hasCarry) << "phase 1 holds a chip between calls";

        restage(block, {{"flush_partial", false}});
        expect(block._coder.hasCarry) << "a tail rule does not move the grid, so the held chip is still the chip it was";

        restage(block, {{"chip_phase", gr::Size_t{0}}});
        expect(!block._coder.hasCarry) << "a chip_phase change re-grids and clears the carry";
    };

    "end of stream"_test = [] {
        { // width 8, phase 0: four pairs fill the first four positions and the rest is pad
            auto                                block = make<ManchesterDecoder>(decoderSettings("ieee802_3"));
            const std::array<std::uint8_t, 1UZ> trailing{0x9AU};
            const auto                          tail = epilogueOf(block, std::span<const std::uint8_t>(trailing), 1UZ);
            expect(that % (tail.bits == std::vector<std::uint8_t>{0x40U}));
            expect(that % (tail.flags == std::vector<std::uint8_t>{0x0FU})) << "the four positions no chip reached are flagged, and zero in out";
            expect(eq(block.nDroppedChips(), 0ULL));
        }

        { // width 8, phase 1: the held chip pairs with the first trailing chip and the last one is a flagged orphan
            auto                                block = make<ManchesterDecoder>(decoderSettings("ieee802_3", 8U, "MsbFirst", 1U));
            const std::array<std::uint8_t, 2UZ> body{0x9AU, 0x95U};
            std::ignore = decodedBy(block, std::span<const std::uint8_t>(body));
            expect(block._coder.hasCarry);
            const std::array<std::uint8_t, 1UZ> trailing{0x9AU};
            const auto                          tail = epilogueOf(block, std::span<const std::uint8_t>(trailing), 1UZ);
            expect(eq(tail.bits.size(), 1UZ));
            expect(eq(static_cast<unsigned>(tail.flags[0UZ]) & 0x07U, 0x07U)) << "the three positions no chip reached are flagged";
            expect(eq(static_cast<unsigned>(tail.bits[0UZ]) & 0x07U, 0U)) << "and are zero in out";
            expect(eq(block.nOrphanItems(), 2ULL)) << "the run's opening orphan, and the final unpaired chip";
            expect(!block._coder.hasCarry);
        }

        { // width 1, phase 0: a lone trailing chip leaves as a flagged orphan
            auto                                block = make<ManchesterDecoder>(decoderSettings("ieee802_3", 1U));
            const std::array<std::uint8_t, 1UZ> trailing{1U};
            const auto                          tail = epilogueOf(block, std::span<const std::uint8_t>(trailing), 1UZ);
            expect(that % (tail.bits == std::vector<std::uint8_t>{0U}));
            expect(that % (tail.flags == std::vector<std::uint8_t>{1U}));
            expect(eq(block.nOrphanItems(), 1ULL));
        }

        { // width 1, phase 1: the held chip and the trailing chip are a pair, so nothing is fabricated
            auto                                block = make<ManchesterDecoder>(decoderSettings("ieee802_3", 1U, "MsbFirst", 1U));
            const std::array<std::uint8_t, 2UZ> body{1U, 0U};
            std::ignore = decodedBy(block, std::span<const std::uint8_t>(body));
            expect(block._coder.hasCarry);
            const std::array<std::uint8_t, 1UZ> trailing{1U};
            const auto                          tail = epilogueOf(block, std::span<const std::uint8_t>(trailing), 1UZ);
            expect(eq(tail.bits.size(), 1UZ));
            expect(eq(tail.flags[0UZ], std::uint8_t{0U})) << "the carry and the trailing chip differ, so the pair is legal";
            expect(eq(block.nOrphanItems(), 1ULL)) << "only the run's opening orphan";
        }

        { // flush_partial = false drops the tail, counts the chips and emits nothing
            for (const gr::Size_t width : {gr::Size_t{1}, gr::Size_t{8}}) {
                gr::property_map settings                 = decoderSettings("ieee802_3", width);
                settings["flush_partial"]                 = false;
                auto                                block = make<ManchesterDecoder>(std::move(settings));
                const std::array<std::uint8_t, 1UZ> trailing{0x9AU};
                const auto                          tail = epilogueOf(block, std::span<const std::uint8_t>(trailing), 1UZ);
                expect(tail.bits.empty()) << "no item is emitted";
                expect(tail.flags.empty());
                expect(eq(block.nDroppedChips(), static_cast<std::uint64_t>(width))) << "the trailing chips are counted rather than lost quietly";
            }
        }

        // and the framework half: an odd input item count leaves one trailing item, the epilogue runs once, and what
        // it explicitly published reaches the sink
        Rng        rng;
        const auto chips   = encoded(encoderSettings("ieee802_3"), std::span<const std::uint8_t>(rng.items(65UZ, 8U))); // 130 chip items
        const auto odd     = std::span<const std::uint8_t>(chips).subspan(0UZ, 129UZ);
        const auto flushed = decoderGraph(decoderSettings("ieee802_3"), odd);
        expect(eq(flushed.bits.size(), 65UZ)) << "64 paired items and one flushed tail";
        expect(eq(flushed.flags.size(), 65UZ));
        expect(eq(flushed.flags[64UZ], std::uint8_t{0x0FU})) << "the tail's pad positions are flagged";

        gr::property_map dropping = decoderSettings("ieee802_3");
        dropping["flush_partial"] = false;
        const auto truncated      = decoderGraph(std::move(dropping), odd);
        expect(eq(truncated.bits.size(), 64UZ)) << "with no flush the trailing item never becomes one";
    };

    "an unconnected violation port"_test = [] {
        Rng        rng;
        const auto chips = encoded(encoderSettings("ieee802_3"), std::span<const std::uint8_t>(rng.items(2048UZ, 8U)));

        const auto connected = decoded(decoderSettings("ieee802_3"), std::span<const std::uint8_t>(chips));
        const auto open      = decoded(decoderSettings("ieee802_3"), std::span<const std::uint8_t>(chips), 0UZ, {}, false);
        expect(that % (open.bits == connected.bits)) << "a chain that does not care about violations decodes the same";
        expect(open.flags.empty()) << "and nothing is published on the port it left open";

        const auto graphed = decoderGraph(decoderSettings("ieee802_3"), std::span<const std::uint8_t>(chips), {}, false);
        expect(that % (graphed.bits == connected.bits)) << "and the graph runs rather than stalling on a port nobody terminated";
        expect(graphed.flags.empty());

        // an open port keeps the default buffer the graph never resized, so past a few thousand items its reserved
        // span comes back short of what the connected port was given. Leaving it out of the count is what keeps that
        // from throttling the block to nothing.
        const auto wide  = encoded(encoderSettings("ieee802_3"), std::span<const std::uint8_t>(rng.items(20000UZ, 8U)));
        const auto large = decoderGraph(decoderSettings("ieee802_3"), std::span<const std::uint8_t>(wide), {}, false);
        expect(eq(large.bits.size(), wide.size() / 2UZ)) << "a chunk wider than the open port's own buffer still decodes in full";
    };
};

int main() { /* not needed for UT */ }
