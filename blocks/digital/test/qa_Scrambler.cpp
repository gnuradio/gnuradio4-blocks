#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <bit>
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
#include <gnuradio-4.0/digital/Scrambler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using AdditiveScrambler = gr::blocks::digital::AdditiveScrambler<std::uint8_t>;
using gr::blocks::digital::MultiplicativeDescrambler;
using gr::blocks::digital::MultiplicativeScrambler;
namespace spans = gr::blocks::testing::span;

// The three profiles below are functions and not namespace-scope constants: boost::ut runs a test body from the
// runner's destructor at exit, by which point a constant with dynamic storage in this unit is already destroyed.

/// @brief CCSDS 131.0-B's pseudo-randomizer, named as an interoperability constant and not as a default of the block.
[[nodiscard]] gr::property_map ccsdsSettings() { return {{"taps", std::string("1,3,5,8")}, {"seed", std::string("11111111")}}; }

/// @brief IEEE Std 802.11's frame-synchronous scrambler, whose 127-bit sequence the standard publishes.
[[nodiscard]] gr::property_map ieee80211Settings() { return {{"taps", std::string("4,7")}, {"seed", std::string("0000111")}}; }

/// @brief ITU-T V-series form A, V.32's `GPC`, which the GSTN modems use self-synchronizingly.
[[nodiscard]] gr::property_map ituFormASettings() { return {{"taps", std::string("18,23")}, {"seed", std::string(23UZ, '1')}}; }

/// @brief The delay sets the standards name for a self-synchronizing scrambler, with `degree` beside each.
struct SelfSyncEntry {
    std::string_view taps;
    std::size_t      degree;
    std::string_view what;
};

constexpr std::array<SelfSyncEntry, 5UZ> kSelfSync{{
    {"18,23", 23UZ, "ITU-T V-series form A, V.32's GPC"},
    {"5,23", 23UZ, "ITU-T V-series form B, V.32's GPA"},
    {"14,17", 17UZ, "ITU-T V.22 and V.22bis"},
    {"4,7", 7UZ, "IEEE 802.11 DSSS and HR/DSSS"},
    {"39,58", 58UZ, "IEEE 802.3 10GBASE-R 64B/66B"},
}};

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

[[nodiscard]] gr::property_map with(gr::property_map settings, gr::property_map extra) {
    for (auto& [key, value] : extra) {
        settings.insert_or_assign(key, value);
    }
    return settings;
}

/// @brief What @p call complains about, which is where a rejection has to name the value that caused it.
template<typename TCall>
[[nodiscard]] std::string complaint(TCall&& call) {
    try {
        call();
    } catch (const std::exception& error) {
        return std::string(error.what());
    }
    return std::string{};
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

[[nodiscard]] std::vector<std::uint8_t> randomItems(std::size_t count, std::uint32_t bound, std::uint64_t seed = 0x243f6a8885a308d3ULL) {
    Rng                       rng{seed};
    std::vector<std::uint8_t> items(count);
    for (std::uint8_t& item : items) {
        item = static_cast<std::uint8_t>(rng.next() % bound);
    }
    return items;
}

template<typename TBlock>
[[nodiscard]] std::vector<std::uint8_t> drive(gr::property_map settings, std::span<const std::uint8_t> input, std::size_t chunkSize = 0UZ, std::span<const gr::Tag> tags = {}) {
    TBlock block = make<TBlock>(std::move(settings));
    return spans::run<std::uint8_t>(block, input, chunkSize, 1UZ, tags).samples;
}

/// @brief What one epoch of @p settings looks like over @p input, which is what a freshly reset register produces.
template<typename TBlock>
[[nodiscard]] std::vector<std::uint8_t> epoch(const gr::property_map& settings, std::span<const std::uint8_t> input) {
    return drive<TBlock>(settings, input);
}

/// @brief @p input cut into epochs of the stated lengths, each converted from a freshly reset register.
template<typename TBlock>
[[nodiscard]] std::vector<std::uint8_t> epochs(const gr::property_map& settings, std::span<const std::uint8_t> input, std::span<const std::size_t> lengths) {
    std::vector<std::uint8_t> expected;
    std::size_t               at = 0UZ;
    for (const std::size_t length : lengths) {
        const std::size_t               count = std::min(length, input.size() - at);
        const std::vector<std::uint8_t> part  = epoch<TBlock>(settings, input.subspan(at, count));
        expected.insert(expected.end(), part.begin(), part.end());
        at += count;
    }
    return expected;
}

/// @brief @p input through @p first and then through @p second, which is what a transmit and receive pair is.
template<typename TFirst, typename TSecond>
[[nodiscard]] std::vector<std::uint8_t> throughPair(const gr::property_map& firstSettings, const gr::property_map& secondSettings, std::span<const std::uint8_t> input) {
    const std::vector<std::uint8_t> channel = drive<TFirst>(firstSettings, input);
    return drive<TSecond>(secondSettings, std::span<const std::uint8_t>(channel));
}

/// @brief The fraction of @p items that are ones, bit by bit, which is what whitening is measured in.
[[nodiscard]] double onesFraction(std::span<const std::uint8_t> items) {
    std::size_t ones = 0UZ;
    for (const std::uint8_t item : items) {
        ones += static_cast<std::size_t>(std::popcount(item));
    }
    return static_cast<double>(ones) / static_cast<double>(8UZ * items.size());
}

/// @brief Run @p stream through the chain @p build assembles, with @p incoming planted at the source, and report the
/// tags the sink saw. Only a graph exercises the framework's own forwarder, which is what tag propagation means here.
template<typename TBuild>
[[nodiscard]] std::vector<gr::Tag> throughGraph(std::span<const std::uint8_t> stream, std::span<const gr::Tag> incoming, TBuild&& build) {
    using gr::testing::ProcessFunction;
    using gr::testing::TagSink;
    using gr::testing::TagSource;

    gr::Graph  graph;
    const auto values = gr::Tensor<std::uint8_t>(stream.begin(), stream.end());
    auto&      source = graph.emplaceBlock<TagSource<std::uint8_t, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(stream.size())}, {"values", values}});
    source._tags.assign(incoming.begin(), incoming.end());
    std::ranges::sort(source._tags, std::ranges::less{}, &gr::Tag::index);

    auto& sink = graph.emplaceBlock<TagSink<std::uint8_t, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});
    build(graph, source, sink);

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value());
    return sink._tags;
}

/// @brief Where @p key carrying @p value reached the sink.
[[nodiscard]] std::vector<std::size_t> offsetsOf(std::span<const gr::Tag> seen, std::string_view key, const gr::pmt::Value& value) {
    std::vector<std::size_t> where;
    for (const gr::Tag& tag : seen) {
        if (const auto found = tag.map.find(gr::property_map::key_type(key)); found != tag.map.end() && found->second == value) {
            where.push_back(tag.index);
        }
    }
    return where;
}

/// @brief A generator named rather than spelled out, which is the whole of what a profile setting stages.
[[nodiscard]] gr::property_map kProfile(std::string_view name) { return gr::property_map{{"profile", std::string(name)}}; }

/// @brief A mask supplied outright, in the hexadecimal spelling a whitening sequence is published in.
[[nodiscard]] gr::property_map kSequence(std::string_view hex, bool repeat) { return gr::property_map{{"sequence", std::string(hex)}, {"sequence_repeat", repeat}}; }

[[nodiscard]] gr::Tag resetTag(std::size_t at, std::string label = std::string("frame")) { return gr::Tag{at, gr::property_map{{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), std::move(label)}}}; }

} // namespace

const boost::ut::suite<"additive scrambler"> additiveTests = [] {
    using namespace boost::ut;

    "the CCSDS sequence reaches the port as the standard publishes it"_test = [] {
        // an all-zero input emits the sequence itself, whose first 40 bits are the standard's check value
        const std::vector<std::uint8_t> zeros(8UZ, std::uint8_t{0});
        const std::vector<std::uint8_t> want{0xFFU, 0x48U, 0x0EU, 0xC0U, 0x9AU, 0x0DU, 0x70U, 0xBCU};
        expect(that % (drive<AdditiveScrambler>(ccsdsSettings(), std::span<const std::uint8_t>(zeros)) == want));

        const std::vector<std::uint8_t> plain{0x47U, 0x53U, 0x4FU, 0x43U, 0x21U, 0x21U, 0x21U, 0x21U};
        const std::vector<std::uint8_t> scrambled{0xB8U, 0x1BU, 0x41U, 0x83U, 0xBBU, 0x2CU, 0x51U, 0x9DU};
        expect(that % (drive<AdditiveScrambler>(ccsdsSettings(), std::span<const std::uint8_t>(plain)) == scrambled)) << "the worked example, scrambling";
        expect(that % (drive<AdditiveScrambler>(ccsdsSettings(), std::span<const std::uint8_t>(scrambled)) == plain)) << "and descrambling, because XOR is an involution";

        // the reciprocal reading of the same polynomial is a different sequence, so a reversed delay list is caught
        const std::vector<std::uint8_t> reciprocal{0xFFU, 0x1AU, 0xAFU, 0x66U, 0x52U, 0x23U, 0x1EU, 0x10U};
        expect(that % (drive<AdditiveScrambler>(with(ccsdsSettings(), {{"taps", std::string("3,5,7,8")}}), std::span<const std::uint8_t>(zeros)) == reciprocal));
        expect(that % (reciprocal != want));
    };

    "there is no default polynomial and no default seed"_test = [] {
        expect(throws([] {
            AdditiveScrambler block{}; // no settings at all, so nothing is staged and nothing is validated until start
            block.start();
        })) << "a block with no taps refuses to start";
        expect(throws([] { std::ignore = make<AdditiveScrambler>({{"taps", std::string("1,3,5,8")}}); })) << "and one with no seed does too";

        { // until it is configured it is inert, so nothing downstream mistakes a pass-through for a scrambled stream
            AdditiveScrambler               block{};
            const std::vector<std::uint8_t> zeros(32UZ, std::uint8_t{0});
            std::vector<std::uint8_t>       out(32UZ);
            std::vector<gr::Tag>            published;
            spans::InputSpan<std::uint8_t>  inSpan{std::span<const std::uint8_t>(zeros)};
            spans::OutputSpan<std::uint8_t> outSpan{std::span<std::uint8_t>(out), 0UZ, &published};
            expect(that % (block.processBulk(inSpan, outSpan) == gr::work::Status::ERROR));
            expect(eq(inSpan.consumed, 0UZ));
        }
    };

    "every rejection names the value that caused it"_test = [] {
        const auto refuses = [](gr::property_map settings, std::string_view offender, std::string_view why) {
            const std::string message = complaint([&settings] { std::ignore = make<AdditiveScrambler>(settings); });
            expect(!message.empty()) << why;
            expect(message.contains(offender)) << std::format("{}: \"{}\" is missing from \"{}\"", why, offender, message);
        };

        refuses(with(ccsdsSettings(), {{"taps", std::string("1,0,5")}}), "1,0,5", "a delay of zero has no meaning");
        refuses(with(ccsdsSettings(), {{"taps", std::string("1,65")}}), "1,65", "a delay above 64 does not fit the register");
        refuses(with(ccsdsSettings(), {{"taps", std::string("3,3")}}), "3,3", "a repeated delay is a typo, not a doubled tap");
        refuses(with(ccsdsSettings(), {{"taps", std::string("1,3;5")}}), "1,3;5", "a stray character must not silently become a different polynomial");
        refuses(with(ccsdsSettings(), {{"seed", std::string("1111")}}), "1111", "a seed must be exactly degree bits");
        refuses(with(ccsdsSettings(), {{"seed", std::string("11112111")}}), "11112111", "a seed is '0' and '1' characters");
        refuses(with(ccsdsSettings(), {{"seed", std::string("00000000")}}), "00000000", "an all-zero additive register is a dead generator");
        refuses(with(ccsdsSettings(), {{"bits_per_item", 0U}}), "0", "zero significant bits is not an item");
        refuses(with(ccsdsSettings(), {{"bits_per_item", 9U}}), "9", "nine significant bits do not fit a byte");

        expect(nothrow([] { std::ignore = make<AdditiveScrambler>(with(ccsdsSettings(), {{"bits_per_item", 1U}, {"bit_order", std::string("LsbFirst")}})); })) << "the boundaries of both ranges are legal";
    };

    "a rejected setting leaves the configuration it was applied to intact"_test = [] {
        const std::vector<std::uint8_t> data = randomItems(64UZ, 256U);
        const std::vector<std::uint8_t> want = epoch<AdditiveScrambler>(ccsdsSettings(), std::span<const std::uint8_t>(data));

        AdditiveScrambler block = make<AdditiveScrambler>(ccsdsSettings());
        expect(throws([&block] { restage(block, {{"seed", std::string("1111")}}); }));
        block.reset();
        expect(that % (spans::run<std::uint8_t>(block, std::span<const std::uint8_t>(data)).samples == want)) << "a half-applied configuration would leave the generator somewhere neither setting names";
    };

    "a reset tag makes the tagged item the first item of a new epoch"_test = [] {
        constexpr std::size_t           kAt  = 37UZ;
        const std::vector<std::uint8_t> data = randomItems(200UZ, 256U);
        const std::vector<gr::Tag>      tags{resetTag(kAt)};
        const std::vector<std::uint8_t> plain = epoch<AdditiveScrambler>(ccsdsSettings(), std::span<const std::uint8_t>(data));
        const std::vector<std::uint8_t> seen  = drive<AdditiveScrambler>(ccsdsSettings(), std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(tags));

        const std::vector<std::uint8_t> after = epoch<AdditiveScrambler>(ccsdsSettings(), std::span<const std::uint8_t>(data).subspan(kAt));
        expect(that % std::equal(seen.begin(), seen.begin() + static_cast<std::ptrdiff_t>(kAt), plain.begin())) << "the items before the tag belong to the epoch that was already running";
        expect(that % std::equal(seen.begin() + static_cast<std::ptrdiff_t>(kAt), seen.end(), after.begin())) << "and the tagged item is sequence bit 0, not the item after it";
        expect(that % (seen != plain)) << "the reset has to be observable at all";
    };

    "a narrowed reset tag answers to one detector only"_test = [] {
        constexpr std::size_t           kAt      = 40UZ;
        const std::vector<std::uint8_t> data     = randomItems(120UZ, 256U);
        const gr::property_map          narrowed = with(ccsdsSettings(), {{"reset_tag_value", std::string("frame")}});
        const std::vector<std::uint8_t> plain    = epoch<AdditiveScrambler>(ccsdsSettings(), std::span<const std::uint8_t>(data));

        const std::vector<gr::Tag> matching{resetTag(kAt, std::string("frame"))};
        const std::vector<gr::Tag> other{resetTag(kAt, std::string("preamble"))};
        const std::vector<gr::Tag> typed{gr::Tag{kAt, gr::property_map{{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), gr::Size_t{7}}}}};

        expect(that % (drive<AdditiveScrambler>(narrowed, std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(matching)) != plain)) << "the value it names does reset";
        expect(that % (drive<AdditiveScrambler>(narrowed, std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(other)) == plain)) << "another detector's label does not";
        expect(that % (drive<AdditiveScrambler>(narrowed, std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(typed)) == plain)) << "and a value that is not a string never matches a narrowing value";
        expect(that % (drive<AdditiveScrambler>(ccsdsSettings(), std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(typed)) != plain)) << "though an empty reset_tag_value asks only that the key be present";

        const gr::property_map deaf = with(ccsdsSettings(), {{"reset_tag_key", std::string("")}});
        expect(that % (drive<AdditiveScrambler>(deaf, std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(matching)) == plain)) << "an empty key disables the mechanism for a chain that must not follow a detector";
    };

    "the periodic reset gives a constant input a period of its own"_test = [] {
        constexpr std::size_t           kPeriod = 23UZ;
        constexpr std::size_t           kEpochs = 12UZ;
        const std::vector<std::uint8_t> zeros(kPeriod * kEpochs, std::uint8_t{0});
        const std::vector<std::uint8_t> seen = drive<AdditiveScrambler>(with(ccsdsSettings(), {{"reset_period", static_cast<gr::Size_t>(kPeriod)}}), std::span<const std::uint8_t>(zeros));

        std::size_t matching = 0UZ;
        for (std::size_t i = 0UZ; i < seen.size(); ++i) {
            matching += seen[i] == seen[i % kPeriod] ? 1UZ : 0UZ;
        }
        expect(eq(matching, seen.size())) << "every epoch repeats the sequence from bit 0";
        expect(that % (seen[0UZ] != seen[kPeriod - 1UZ])) << "and the sequence is not constant, so the check means something";
    };

    "a tag re-anchors the period rather than adding a second reset"_test = [] {
        constexpr std::size_t           kPeriod  = 16UZ;
        const std::vector<std::uint8_t> data     = randomItems(96UZ, 256U);
        const gr::property_map          periodic = with(ccsdsSettings(), {{"reset_period", static_cast<gr::Size_t>(kPeriod)}});

        { // a tag off the grid moves the grid with it: the next boundary is P items after the tag, not after 32
            const std::vector<gr::Tag>      tags{resetTag(40UZ)};
            const std::vector<std::size_t>  lengths{16UZ, 16UZ, 8UZ, 16UZ, 16UZ, 16UZ, 8UZ};
            const std::vector<std::uint8_t> want = epochs<AdditiveScrambler>(ccsdsSettings(), std::span<const std::uint8_t>(data), std::span<const std::size_t>(lengths));
            expect(that % (drive<AdditiveScrambler>(periodic, std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(tags)) == want));

            const std::vector<std::size_t>  unmoved{16UZ, 16UZ, 8UZ, 8UZ, 16UZ, 16UZ, 16UZ};
            const std::vector<std::uint8_t> wrong = epochs<AdditiveScrambler>(ccsdsSettings(), std::span<const std::uint8_t>(data), std::span<const std::size_t>(unmoved));
            expect(that % (want != wrong)) << "a counter left un-anchored would put the next boundary at 48";
        }
        { // and a tag exactly on a boundary is one reset, so the epoch it opens is a whole period long
            const std::vector<gr::Tag>      tags{resetTag(32UZ)};
            const std::vector<std::uint8_t> want = drive<AdditiveScrambler>(periodic, std::span<const std::uint8_t>(data));
            expect(that % (drive<AdditiveScrambler>(periodic, std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(tags)) == want));
        }
    };

    "the output does not depend on how the stream is cut into calls"_test = [] {
        const std::vector<std::uint8_t> data = randomItems(4096UZ, 256U);
        const std::vector<gr::Tag>      tags{resetTag(5UZ), resetTag(1000UZ), resetTag(1001UZ), resetTag(3777UZ)};

        for (const gr::Size_t width : {gr::Size_t{1}, gr::Size_t{3}, gr::Size_t{8}}) {
            const gr::property_map          settings = with(ccsdsSettings(), {{"bits_per_item", width}, {"reset_period", gr::Size_t{97}}});
            const std::vector<std::uint8_t> whole    = drive<AdditiveScrambler>(settings, std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(tags));
            for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 4096UZ}) {
                const std::vector<std::uint8_t> cut = drive<AdditiveScrambler>(settings, std::span<const std::uint8_t>(data), chunk, std::span<const gr::Tag>(tags));
                expect(that % (cut == whole)) << std::format("bits_per_item {} at chunk {}: a register not written back at the end of a call is right for large chunks and wrong for small ones", width, chunk);
            }
        }
    };

    "a reconfiguring change is a discontinuity and a boundary setting is not"_test = [] {
        const std::vector<std::uint8_t>     data = randomItems(64UZ, 256U);
        const std::span<const std::uint8_t> first(std::span<const std::uint8_t>(data).subspan(0UZ, 32UZ));
        const std::span<const std::uint8_t> second(std::span<const std::uint8_t>(data).subspan(32UZ));

        { // selecting a different boundary does not move the one the register is on
            AdditiveScrambler         block = make<AdditiveScrambler>(ccsdsSettings());
            std::vector<std::uint8_t> seen  = spans::run<std::uint8_t>(block, first).samples;
            restage(block, {{"reset_tag_value", std::string("frame")}});
            const std::vector<std::uint8_t> tail = spans::run<std::uint8_t>(block, second).samples;
            seen.insert(seen.end(), tail.begin(), tail.end());
            expect(that % (seen == epoch<AdditiveScrambler>(ccsdsSettings(), std::span<const std::uint8_t>(data)))) << "the stream runs on across a change that only names a boundary";
        }
        { // a new polynomial cannot inherit the old register, so the change opens an epoch
            AdditiveScrambler block = make<AdditiveScrambler>(ccsdsSettings());
            std::ignore             = spans::run<std::uint8_t>(block, first);
            restage(block, ieee80211Settings());
            expect(that % (spans::run<std::uint8_t>(block, second).samples == epoch<AdditiveScrambler>(ieee80211Settings(), second)));
        }
    };

    "a stream value cannot stop the graph"_test = [] {
        constexpr std::size_t           kItems = 20000UZ;
        const std::vector<std::uint8_t> ones(kItems, std::uint8_t{0xFF});
        const std::vector<std::uint8_t> low(kItems, std::uint8_t{0x07});
        const gr::property_map          settings = with(ccsdsSettings(), {{"bits_per_item", gr::Size_t{3}}});

        const std::vector<std::uint8_t> seen = drive<AdditiveScrambler>(settings, std::span<const std::uint8_t>(ones));
        expect(that % (seen == drive<AdditiveScrambler>(settings, std::span<const std::uint8_t>(low)))) << "an item's unused high bits are masked, not rejected";
        expect(that % std::ranges::all_of(seen, [](std::uint8_t item) { return item < 8U; })) << "and an output item's unused high bits are zero";
    };

    "every tag key passes through at its own offset"_test = [] {
        static_assert(gr::block::kUnfilteredTagPropagationAdmissible<AdditiveScrambler>, "the block is 1:1 on synchronous ports, declares no resampling or stride and supplies no forwardTags()");

        const gr::pmt::Value            carried{std::string("carried")};
        const gr::pmt::Value            label{std::string("frame")};
        const std::vector<std::uint8_t> stream = randomItems(600UZ, 256U);

        std::vector<gr::Tag> incoming;
        incoming.emplace_back(11UZ, gr::property_map{{gr::property_map::key_type("private_key"), carried}});
        incoming.emplace_back(400UZ, gr::property_map{{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), label}});

        const std::vector<gr::Tag> seen = throughGraph(std::span<const std::uint8_t>(stream), std::span<const gr::Tag>(incoming), [](gr::Graph& graph, auto& source, auto& sink) {
            auto& first  = graph.emplaceBlock<AdditiveScrambler>(ccsdsSettings());
            auto& second = graph.emplaceBlock<AdditiveScrambler>(ccsdsSettings());
            boost::ut::expect(graph.connect<"out", "in">(source, first).has_value());
            boost::ut::expect(graph.connect<"out", "in">(first, second).has_value());
            boost::ut::expect(graph.connect<"out", "in">(second, sink).has_value());
        });

        expect(that % (offsetsOf(std::span<const gr::Tag>(seen), "private_key", carried) == std::vector<std::size_t>{11UZ})) << "a key the auto-forward set does not name survives two blocks";
        expect(that % (offsetsOf(std::span<const gr::Tag>(seen), gr::tag::TRIGGER_NAME.shortKey(), label) == std::vector<std::size_t>{400UZ})) << "and the reset tag is read and forwarded, never consumed";
    };

    "sample_rate leaves with the value it arrived with"_test = [] {
        constexpr float                 kRate  = 12345.0F;
        const std::vector<std::uint8_t> stream = randomItems(300UZ, 256U);
        const std::vector<gr::Tag>      incoming{gr::Tag{50UZ, gr::property_map{{gr::property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()), kRate}}}};

        const std::vector<gr::Tag> seen = throughGraph(std::span<const std::uint8_t>(stream), std::span<const gr::Tag>(incoming), [](gr::Graph& graph, auto& source, auto& sink) {
            auto& block = graph.emplaceBlock<AdditiveScrambler>(ccsdsSettings());
            boost::ut::expect(graph.connect<"out", "in">(source, block).has_value());
            boost::ut::expect(graph.connect<"out", "in">(block, sink).has_value());
        });

        expect(that % (offsetsOf(std::span<const gr::Tag>(seen), gr::tag::SAMPLE_RATE.shortKey(), gr::pmt::Value(kRate)) == std::vector<std::size_t>{50UZ})) << "the block is 1:1 and declares no sample_rate of its own, so there is nothing to rewrite and nothing to substitute";
    };
};

const boost::ut::suite<"multiplicative scrambler"> multiplicativeScramblerTests = [] {
    using namespace boost::ut;

    "the feedback comes from the block's own output, not from a sequence beside it"_test = [] {
        const std::vector<std::uint8_t> data = randomItems(512UZ, 256U);
        expect(that % (drive<MultiplicativeScrambler>(ituFormASettings(), std::span<const std::uint8_t>(data)) != drive<AdditiveScrambler>(ituFormASettings(), std::span<const std::uint8_t>(data)))) << "the two families share a polynomial and a seed and are not the same transformation";
    };

    "an empty seed is a legitimate history rather than a missing setting"_test = [] {
        const std::vector<std::uint8_t> data = randomItems(256UZ, 256U);
        const gr::property_map          empty{{"taps", std::string("18,23")}};
        const gr::property_map          spelled{{"taps", std::string("18,23")}, {"seed", std::string(23UZ, '0')}};

        expect(nothrow([&empty] { std::ignore = make<MultiplicativeScrambler>(empty); })) << "the seed reaches only the first degree bits of an epoch, so it has a default the additive block cannot have";
        expect(that % (drive<MultiplicativeScrambler>(empty, std::span<const std::uint8_t>(data)) == drive<MultiplicativeScrambler>(spelled, std::span<const std::uint8_t>(data)))) << "and the empty spelling names all zeros";
        expect(throws([] { std::ignore = make<MultiplicativeScrambler>({{"taps", std::string("18,23")}, {"seed", std::string("101")}}); })) << "a seed that is given must still be exactly degree bits";
    };

    "the whitening depends on the data, and its fixed point is the reason to say so"_test = [] {
        constexpr std::size_t           kItems = 4096UZ;
        const std::vector<std::uint8_t> zeros(kItems, std::uint8_t{0});

        const std::vector<std::uint8_t> dead = drive<MultiplicativeScrambler>({{"taps", std::string("18,23")}}, std::span<const std::uint8_t>(zeros));
        expect(that % std::ranges::all_of(dead, [](std::uint8_t item) { return item == 0U; })) << "an all-zero register carrying an all-zero input emits zeros for ever, which no additive scrambler does";

        const double fraction = onesFraction(std::span<const std::uint8_t>(drive<MultiplicativeScrambler>(ituFormASettings(), std::span<const std::uint8_t>(zeros))));
        expect(ge(fraction, 0.45)) << std::format("a seeded register whitens the same input: {:.4f} ones", fraction);
        expect(le(fraction, 0.55)) << std::format("a seeded register whitens the same input: {:.4f} ones", fraction);
    };

    "every epoch is an independently seeded run"_test = [] {
        // the reset machinery is shared with AdditiveScrambler, where the narrowing and the disabling halves are
        // pinned; what is asserted per block is that the mode reaches it and that the boundaries land where they are
        constexpr std::size_t           kAt  = 37UZ;
        const std::vector<std::uint8_t> data = randomItems(200UZ, 256U);
        const std::vector<gr::Tag>      tags{resetTag(kAt)};
        const std::vector<std::uint8_t> plain = epoch<MultiplicativeScrambler>(ituFormASettings(), std::span<const std::uint8_t>(data));
        const std::vector<std::uint8_t> seen  = drive<MultiplicativeScrambler>(ituFormASettings(), std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(tags));
        const std::vector<std::uint8_t> after = epoch<MultiplicativeScrambler>(ituFormASettings(), std::span<const std::uint8_t>(data).subspan(kAt));

        expect(that % std::equal(seen.begin(), seen.begin() + static_cast<std::ptrdiff_t>(kAt), plain.begin())) << "the items before the tag belong to the epoch that was already running";
        expect(that % std::equal(seen.begin() + static_cast<std::ptrdiff_t>(kAt), seen.end(), after.begin())) << "and the tagged item opens a run seeded exactly as a fresh block is";
        expect(that % (seen != plain)) << "the reset has to be observable at all";
    };

    "a tag re-anchors the period rather than adding a second reset"_test = [] {
        const std::vector<std::uint8_t> data     = randomItems(96UZ, 256U);
        const gr::property_map          periodic = with(ituFormASettings(), {{"reset_period", gr::Size_t{16}}});

        const std::vector<gr::Tag>      tags{resetTag(40UZ)};
        const std::vector<std::size_t>  lengths{16UZ, 16UZ, 8UZ, 16UZ, 16UZ, 16UZ, 8UZ};
        const std::vector<std::uint8_t> want = epochs<MultiplicativeScrambler>(ituFormASettings(), std::span<const std::uint8_t>(data), std::span<const std::size_t>(lengths));
        expect(that % (drive<MultiplicativeScrambler>(periodic, std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(tags)) == want));

        const std::vector<std::uint8_t> onGrid = drive<MultiplicativeScrambler>(periodic, std::span<const std::uint8_t>(data));
        const std::vector<gr::Tag>      aligned{resetTag(32UZ)};
        expect(that % (drive<MultiplicativeScrambler>(periodic, std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(aligned)) == onGrid)) << "a tag exactly on a boundary is one reset, so the epoch it opens is a whole period long";
    };

    "the output does not depend on how the stream is cut into calls"_test = [] {
        const std::vector<std::uint8_t> data = randomItems(4096UZ, 256U);
        const std::vector<gr::Tag>      tags{resetTag(5UZ), resetTag(1000UZ), resetTag(1001UZ), resetTag(3777UZ)};

        for (const gr::Size_t width : {gr::Size_t{1}, gr::Size_t{3}, gr::Size_t{8}}) {
            const gr::property_map          settings = with(ituFormASettings(), {{"bits_per_item", width}, {"reset_period", gr::Size_t{97}}});
            const std::vector<std::uint8_t> whole    = drive<MultiplicativeScrambler>(settings, std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(tags));
            for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 4096UZ}) {
                expect(that % (drive<MultiplicativeScrambler>(settings, std::span<const std::uint8_t>(data), chunk, std::span<const gr::Tag>(tags)) == whole)) << std::format("bits_per_item {} at chunk {}: the register has to survive a call boundary", width, chunk);
            }
        }
    };

    "every tag key passes through at its own offset"_test = [] {
        static_assert(gr::block::kUnfilteredTagPropagationAdmissible<MultiplicativeScrambler>, "the block is 1:1 on synchronous ports, declares no resampling or stride and supplies no forwardTags()");

        const gr::pmt::Value            carried{std::string("carried")};
        const gr::pmt::Value            label{std::string("frame")};
        const std::vector<std::uint8_t> stream = randomItems(600UZ, 256U);

        std::vector<gr::Tag> incoming;
        incoming.emplace_back(11UZ, gr::property_map{{gr::property_map::key_type("private_key"), carried}});
        incoming.emplace_back(400UZ, gr::property_map{{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), label}});

        const std::vector<gr::Tag> seen = throughGraph(std::span<const std::uint8_t>(stream), std::span<const gr::Tag>(incoming), [](gr::Graph& graph, auto& source, auto& sink) {
            auto& block = graph.emplaceBlock<MultiplicativeScrambler>(ituFormASettings());
            boost::ut::expect(graph.connect<"out", "in">(source, block).has_value());
            boost::ut::expect(graph.connect<"out", "in">(block, sink).has_value());
        });

        expect(that % (offsetsOf(std::span<const gr::Tag>(seen), "private_key", carried) == std::vector<std::size_t>{11UZ}));
        expect(that % (offsetsOf(std::span<const gr::Tag>(seen), gr::tag::TRIGGER_NAME.shortKey(), label) == std::vector<std::size_t>{400UZ})) << "the reset tag is read and forwarded, never consumed";
    };
};

const boost::ut::suite<"multiplicative descrambler"> multiplicativeDescramblerTests = [] {
    using namespace boost::ut;

    "the pair inverts exactly, from item 0, at every entry the standards name"_test = [] {
        const std::vector<std::uint8_t> bytes = randomItems(1000UZ, 256U);
        const std::vector<std::uint8_t> bits  = randomItems(8000UZ, 2U);

        for (const SelfSyncEntry& entry : kSelfSync) {
            const gr::property_map settings{{"taps", std::string(entry.taps)}, {"seed", std::string(entry.degree, '1')}};
            expect(that % (throughPair<MultiplicativeScrambler, MultiplicativeDescrambler>(settings, settings, std::span<const std::uint8_t>(bytes)) == bytes)) << std::format("{} at eight bits per item", entry.what);

            const gr::property_map perBit = with(settings, {{"bits_per_item", gr::Size_t{1}}});
            expect(that % (throughPair<MultiplicativeScrambler, MultiplicativeDescrambler>(perBit, perBit, std::span<const std::uint8_t>(bits)) == bits)) << std::format("{} at one bit per item", entry.what);
        }
    };

    "a wrong seed costs the first degree bits and nothing after them"_test = [] {
        const std::vector<std::uint8_t> bits = randomItems(4000UZ, 2U);
        Rng                             rng{0x13198a2e03707344ULL};

        for (const SelfSyncEntry& entry : kSelfSync) {
            const gr::property_map          sent{{"taps", std::string(entry.taps)}, {"seed", std::string(entry.degree, '1')}, {"bits_per_item", gr::Size_t{1}}};
            const std::vector<std::uint8_t> channel = drive<MultiplicativeScrambler>(sent, std::span<const std::uint8_t>(bits));

            for (std::size_t trial = 0UZ; trial < 8UZ; ++trial) {
                std::string wrong(entry.degree, '0');
                for (char& bit : wrong) {
                    bit = (rng.next() & 1ULL) != 0ULL ? '1' : '0';
                }
                const gr::property_map          received = with(sent, {{"seed", wrong}});
                const std::vector<std::uint8_t> seen     = drive<MultiplicativeDescrambler>(received, std::span<const std::uint8_t>(channel));

                expect(that % std::equal(seen.begin() + static_cast<std::ptrdiff_t>(entry.degree), seen.end(), bits.begin() + static_cast<std::ptrdiff_t>(entry.degree))) //
                    << std::format("{}: every bit from index {} on is data whatever the register began as", entry.what, entry.degree);
            }
        }
    };

    "a mid-stream join converges within degree bits of where it started"_test = [] {
        constexpr std::size_t           kJoin = 501UZ;
        const std::vector<std::uint8_t> bits  = randomItems(4000UZ, 2U);

        for (const SelfSyncEntry& entry : kSelfSync) {
            const gr::property_map          sent{{"taps", std::string(entry.taps)}, {"seed", std::string(entry.degree, '1')}, {"bits_per_item", gr::Size_t{1}}};
            const std::vector<std::uint8_t> channel = drive<MultiplicativeScrambler>(sent, std::span<const std::uint8_t>(bits));

            // a descrambler switched on part way through the stream, with a seed that names none of what preceded it
            const std::vector<std::uint8_t> seen = drive<MultiplicativeDescrambler>(with(sent, {{"seed", std::string(entry.degree, '0')}}), std::span<const std::uint8_t>(channel).subspan(kJoin));
            expect(that % std::equal(seen.begin() + static_cast<std::ptrdiff_t>(entry.degree), seen.end(), bits.begin() + static_cast<std::ptrdiff_t>(kJoin + entry.degree))) //
                << std::format("{}: correct from bit {} of the stream", entry.what, kJoin + entry.degree);
        }
    };

    "a reset the descrambler never gets costs it degree bits an epoch, not the epoch"_test = [] {
        constexpr std::size_t           kPeriod = 400UZ;
        constexpr std::size_t           kDegree = 23UZ;
        const std::vector<std::uint8_t> bits    = randomItems(kPeriod * 12UZ, 2U);
        const gr::property_map          perBit  = with(ituFormASettings(), {{"bits_per_item", gr::Size_t{1}}});
        const gr::property_map          framed  = with(perBit, {{"reset_period", static_cast<gr::Size_t>(kPeriod)}});

        expect(that % (throughPair<MultiplicativeScrambler, MultiplicativeDescrambler>(framed, framed, std::span<const std::uint8_t>(bits)) == bits)) << "both ends resetting together is exact over twelve epochs";

        const std::vector<std::uint8_t> seen = throughPair<MultiplicativeScrambler, MultiplicativeDescrambler>(framed, perBit, std::span<const std::uint8_t>(bits));
        expect(that % (seen != bits)) << "a descrambler that never resets is not simply right, or the positive half would mean nothing";

        std::size_t wrongInside = 0UZ;
        std::size_t wrongAfter  = 0UZ;
        for (std::size_t i = 0UZ; i < bits.size(); ++i) {
            if (seen[i] != bits[i]) {
                ((i % kPeriod) < kDegree ? wrongInside : wrongAfter) += 1UZ;
            }
        }
        expect(eq(wrongAfter, 0UZ)) << "and the damage is bounded to the first degree bits of an epoch, which is the whole self-synchronizing claim";
        expect(gt(wrongInside, 0UZ)) << "while inside that window it is genuinely wrong";
    };

    "every epoch is an independently seeded run"_test = [] {
        constexpr std::size_t           kAt  = 37UZ;
        const std::vector<std::uint8_t> data = randomItems(200UZ, 256U);
        const std::vector<gr::Tag>      tags{resetTag(kAt)};
        const std::vector<std::uint8_t> plain = epoch<MultiplicativeDescrambler>(ituFormASettings(), std::span<const std::uint8_t>(data));
        const std::vector<std::uint8_t> seen  = drive<MultiplicativeDescrambler>(ituFormASettings(), std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(tags));
        const std::vector<std::uint8_t> after = epoch<MultiplicativeDescrambler>(ituFormASettings(), std::span<const std::uint8_t>(data).subspan(kAt));

        expect(that % std::equal(seen.begin(), seen.begin() + static_cast<std::ptrdiff_t>(kAt), plain.begin())) << "the items before the tag belong to the epoch that was already running";
        expect(that % std::equal(seen.begin() + static_cast<std::ptrdiff_t>(kAt), seen.end(), after.begin())) << "and the tagged item opens a run seeded exactly as a fresh block is";
        expect(that % (seen != plain)) << "the reset has to be observable at all";
    };

    "a tag re-anchors the period rather than adding a second reset"_test = [] {
        const std::vector<std::uint8_t> data     = randomItems(96UZ, 256U);
        const gr::property_map          periodic = with(ituFormASettings(), {{"reset_period", gr::Size_t{16}}});

        const std::vector<gr::Tag>      tags{resetTag(40UZ)};
        const std::vector<std::size_t>  lengths{16UZ, 16UZ, 8UZ, 16UZ, 16UZ, 16UZ, 8UZ};
        const std::vector<std::uint8_t> want = epochs<MultiplicativeDescrambler>(ituFormASettings(), std::span<const std::uint8_t>(data), std::span<const std::size_t>(lengths));
        expect(that % (drive<MultiplicativeDescrambler>(periodic, std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(tags)) == want));

        const std::vector<gr::Tag> aligned{resetTag(32UZ)};
        expect(that % (drive<MultiplicativeDescrambler>(periodic, std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(aligned)) == drive<MultiplicativeDescrambler>(periodic, std::span<const std::uint8_t>(data)))) << "a tag exactly on a boundary is one reset, so the epoch it opens is a whole period long";
    };

    "the output does not depend on how the stream is cut into calls"_test = [] {
        const std::vector<std::uint8_t> data = randomItems(4096UZ, 256U);
        const std::vector<gr::Tag>      tags{resetTag(5UZ), resetTag(1000UZ), resetTag(1001UZ), resetTag(3777UZ)};

        for (const gr::Size_t width : {gr::Size_t{1}, gr::Size_t{3}, gr::Size_t{8}}) {
            const gr::property_map          settings = with(ituFormASettings(), {{"bits_per_item", width}, {"reset_period", gr::Size_t{97}}});
            const std::vector<std::uint8_t> whole    = drive<MultiplicativeDescrambler>(settings, std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(tags));
            for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 4096UZ}) {
                expect(that % (drive<MultiplicativeDescrambler>(settings, std::span<const std::uint8_t>(data), chunk, std::span<const gr::Tag>(tags)) == whole)) << std::format("bits_per_item {} at chunk {}: the window of received bits has to survive a call boundary", width, chunk);
            }
        }
    };
};

const boost::ut::suite<"the family's cross-cuts"> familyTests = [] {
    using namespace boost::ut;

    "all three are admissible for the pass-all tag policy"_test = [] {
        static_assert(gr::block::kUnfilteredTagPropagationAdmissible<AdditiveScrambler>);
        static_assert(gr::block::kUnfilteredTagPropagationAdmissible<MultiplicativeScrambler>);
        static_assert(gr::block::kUnfilteredTagPropagationAdmissible<MultiplicativeDescrambler>, "each is 1:1 on synchronous ports, declares no resampling or stride and supplies no forwardTags()");
        expect(true) << "the compile-time half of the policy, which costs nothing to keep asserted";
    };

    "a key rides a whole transmit and receive chain at its own offset"_test = [] {
        const gr::pmt::Value            carried{std::string("carried")};
        const gr::pmt::Value            label{std::string("frame")};
        const std::vector<std::uint8_t> stream = randomItems(600UZ, 256U);

        std::vector<gr::Tag> incoming;
        incoming.emplace_back(11UZ, gr::property_map{{gr::property_map::key_type("private_key"), carried}});
        incoming.emplace_back(400UZ, gr::property_map{{gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), label}});

        const std::vector<gr::Tag> seen = throughGraph(std::span<const std::uint8_t>(stream), std::span<const gr::Tag>(incoming), [](gr::Graph& graph, auto& source, auto& sink) {
            auto& scrambler   = graph.emplaceBlock<MultiplicativeScrambler>(ituFormASettings());
            auto& descrambler = graph.emplaceBlock<MultiplicativeDescrambler>(ituFormASettings());
            boost::ut::expect(graph.connect<"out", "in">(source, scrambler).has_value());
            boost::ut::expect(graph.connect<"out", "in">(scrambler, descrambler).has_value());
            boost::ut::expect(graph.connect<"out", "in">(descrambler, sink).has_value());
        });

        expect(that % (offsetsOf(std::span<const gr::Tag>(seen), "private_key", carried) == std::vector<std::size_t>{11UZ})) << "a key the auto-forward set does not name survives the pair";
        expect(that % (offsetsOf(std::span<const gr::Tag>(seen), gr::tag::TRIGGER_NAME.shortKey(), label) == std::vector<std::size_t>{400UZ})) << "and so does the reserved key the blocks read their epoch boundary from";
    };

    "sample_rate crosses all three unchanged"_test = [] {
        constexpr float                 kRate  = 12345.0F;
        const std::vector<std::uint8_t> stream = randomItems(300UZ, 256U);
        const std::vector<gr::Tag>      incoming{gr::Tag{50UZ, gr::property_map{{gr::property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()), kRate}}}};

        const std::vector<gr::Tag> seen = throughGraph(std::span<const std::uint8_t>(stream), std::span<const gr::Tag>(incoming), [](gr::Graph& graph, auto& source, auto& sink) {
            auto& additive    = graph.emplaceBlock<AdditiveScrambler>(ccsdsSettings());
            auto& scrambler   = graph.emplaceBlock<MultiplicativeScrambler>(ituFormASettings());
            auto& descrambler = graph.emplaceBlock<MultiplicativeDescrambler>(ituFormASettings());
            boost::ut::expect(graph.connect<"out", "in">(source, additive).has_value());
            boost::ut::expect(graph.connect<"out", "in">(additive, scrambler).has_value());
            boost::ut::expect(graph.connect<"out", "in">(scrambler, descrambler).has_value());
            boost::ut::expect(graph.connect<"out", "in">(descrambler, sink).has_value());
        });

        expect(that % (offsetsOf(std::span<const gr::Tag>(seen), gr::tag::SAMPLE_RATE.shortKey(), gr::pmt::Value(kRate)) == std::vector<std::size_t>{50UZ})) //
            << "none of the three changes a rate and none declares a sample_rate of its own, so there is nothing to rescale and nothing to substitute";
    };
};

const boost::ut::suite<"named profiles"> profileTests = [] {
    using namespace boost::ut;

    "a profile fills the generator its own standard publishes"_test = [] {
        const std::vector<std::uint8_t> zeros(16UZ, std::uint8_t{0});

        // CCSDS 131.0-B's pseudo-randomizer, whose first 40 bits are the value the standard prints
        const std::vector<std::uint8_t> ccsds{0xFFU, 0x48U, 0x0EU, 0xC0U, 0x9AU, 0x0DU, 0x70U, 0xBCU};
        const std::vector<std::uint8_t> byName = drive<AdditiveScrambler>(kProfile("ccsds131"), std::span<const std::uint8_t>(zeros).first(8UZ));
        expect(that % (byName == ccsds)) << "the name resolves to the delay set and the seed, not to something near them";
        expect(that % (byName == drive<AdditiveScrambler>(ccsdsSettings(), std::span<const std::uint8_t>(zeros).first(8UZ)))) << "and to exactly what the two settings spelled out reach";

        // the CC11xx / SX12xx whitening sequence: taps, seed and an LSB-first traversal, confirmed together
        const std::vector<std::uint8_t> pn9{0xFFU, 0xE1U, 0x1DU, 0x9AU, 0xEDU, 0x85U, 0x33U, 0x24U, 0xEAU, 0x7AU, 0xD2U, 0x39U, 0x70U, 0x97U, 0x57U, 0x0AU};
        expect(that % (drive<AdditiveScrambler>(kProfile("pn9"), std::span<const std::uint8_t>(zeros)) == pn9)) << "bit_order comes from the profile: the block's own default would give the MSB-first reading instead";
        expect(that % (drive<AdditiveScrambler>(with(kProfile("pn9"), {{"bits_per_item", 8U}, {"bit_order", std::string("MsbFirst")}}), std::span<const std::uint8_t>(zeros)) == pn9)) //
            << "and it wins over a staged bits_per_item and bit_order, which carry defaults and cannot be told from a choice";
    };

    "a profile is never a default"_test = [] {
        expect(throws([] {
            AdditiveScrambler block{}; // profile is present and empty, which selects nothing at all
            block.start();
        })) << "an unnamed generator is still no generator";

        AdditiveScrambler               block{};
        const std::vector<std::uint8_t> zeros(32UZ, std::uint8_t{0});
        std::vector<std::uint8_t>       out(32UZ);
        std::vector<gr::Tag>            published;
        spans::InputSpan<std::uint8_t>  inSpan{std::span<const std::uint8_t>(zeros)};
        spans::OutputSpan<std::uint8_t> outSpan{std::span<std::uint8_t>(out), 0UZ, &published};
        expect(that % (block.processBulk(inSpan, outSpan) == gr::work::Status::ERROR));
        expect(eq(inSpan.consumed, 0UZ));
    };

    "naming a profile and a setting it fills is a refusal and not a merge"_test = [] {
        const auto refuses = [](gr::property_map settings, std::string_view offender, std::string_view why) {
            const std::string message = complaint([&settings] { std::ignore = make<AdditiveScrambler>(settings); });
            expect(!message.empty()) << why;
            expect(message.contains(offender)) << std::format("{}: \"{}\" is missing from \"{}\"", why, offender, message);
        };

        refuses(with(kProfile("ccsds131"), {{"taps", std::string("4,7")}}), "taps", "two tap sets are two answers to one question");
        refuses(with(kProfile("ccsds131"), {{"taps", std::string("4,7")}}), "4,7", "and the refusal names the value that caused it");
        refuses(with(kProfile("ccsds131"), {{"seed", std::string("00001111")}}), "seed", "a profile's seed is part of the agreement it names");
        refuses(with(kProfile("ccsds131"), {{"sequence", std::string("FF E1")}}), "sequence", "a supplied mask replaces the generator a profile would build");
        refuses({{"taps", std::string("1,3,5,8")}, {"seed", std::string("11111111")}, {"sequence", std::string("FF E1")}}, "sequence", "taps and sequence are alternative mask sources");

        const std::string unknown = complaint([] { std::ignore = make<AdditiveScrambler>(kProfile("ccsds132")); });
        expect(unknown.contains("ccsds132")) << unknown;
        for (const std::string_view name : {"ccsds131", "ccsds131_17", "pn9", "si4463", "g3ruh"}) {
            expect(unknown.contains(name)) << std::format("an unknown name lists the ones there are, and \"{}\" is missing from \"{}\"", name, unknown);
        }

        expect(nothrow([] { std::ignore = make<AdditiveScrambler>(kProfile("si4463")); })) << "si4463's constants are recorded, so the profile applies";
    };

    "a profile of the wrong family is refused naming the family"_test = [] {
        const std::string additive = complaint([] { std::ignore = make<AdditiveScrambler>(kProfile("g3ruh")); });
        expect(additive.contains("g3ruh")) << additive;
        expect(additive.contains("multiplicative")) << "G3RUH is self-synchronizing, and running it additively is a silent wrong answer rather than an error: " + additive;

        for (const std::string_view name : {"ccsds131", "pn9"}) {
            const std::string onScrambler   = complaint([name] { std::ignore = make<MultiplicativeScrambler>(kProfile(name)); });
            const std::string onDescrambler = complaint([name] { std::ignore = make<MultiplicativeDescrambler>(kProfile(name)); });
            expect(onScrambler.contains(name) && onScrambler.contains("additive")) << onScrambler;
            expect(onDescrambler.contains(name) && onDescrambler.contains("additive")) << onDescrambler;
        }
    };

    "the g3ruh pair inverts exactly, and the block reads its taps from the name"_test = [] {
        const std::vector<std::uint8_t> data = randomItems(512UZ, 256U);
        expect(that % (throughPair<MultiplicativeScrambler, MultiplicativeDescrambler>(kProfile("g3ruh"), kProfile("g3ruh"), std::span<const std::uint8_t>(data)) == data));

        const gr::property_map spelled{{"taps", std::string("12,17")}, {"seed", std::string(17UZ, '0')}};
        expect(that % (drive<MultiplicativeScrambler>(kProfile("g3ruh"), std::span<const std::uint8_t>(data)) == drive<MultiplicativeScrambler>(spelled, std::span<const std::uint8_t>(data)))) //
            << "the delay reading is {12,17}, the reciprocal {5,17} being a different transmitted stream with the same period";
    };

    "both counters are readable on all three blocks"_test = [] {
        AdditiveScrambler         additive    = make<AdditiveScrambler>(kProfile("ccsds131"));
        MultiplicativeScrambler   scrambler   = make<MultiplicativeScrambler>(kProfile("g3ruh"));
        MultiplicativeDescrambler descrambler = make<MultiplicativeDescrambler>(kProfile("g3ruh"));
        expect(eq(additive.nUnscrambledItems(), 0ULL));
        expect(eq(additive.nForcedTransitions(), 0ULL)) << "structurally zero on the additive block, which has no feedback for the rule to act on";
        expect(eq(scrambler.nForcedTransitions(), 0ULL));
        expect(eq(scrambler.nUnscrambledItems(), 0ULL)) << "structurally zero on the multiplicative pair, whose recursion never runs out";
        expect(eq(descrambler.nForcedTransitions(), 0ULL));
        expect(eq(descrambler.nUnscrambledItems(), 0ULL));
    };
};

const boost::ut::suite<"the explicit sequence"> explicitSequenceTests = [] {
    using namespace boost::ut;

    "a supplied mask is XORed into the stream and tiles the epoch"_test = [] {
        const std::vector<std::uint8_t> mask{0xFFU, 0xE1U, 0x1DU, 0x9AU};
        const std::vector<std::uint8_t> data = randomItems(41UZ, 256U);
        const std::vector<std::uint8_t> seen = drive<AdditiveScrambler>(kSequence("FF E1 1D 9A", true), std::span<const std::uint8_t>(data));

        std::vector<std::uint8_t> want(data.size());
        for (std::size_t i = 0UZ; i < data.size(); ++i) {
            want[i] = static_cast<std::uint8_t>(data[i] ^ mask[i % mask.size()]);
        }
        expect(that % (seen == want)) << "a period-S sequence tiles exactly as a period-S recursion would, past the end of a whole number of repetitions as well as inside one";
        expect(that % (drive<AdditiveScrambler>(kSequence("ffe11d9a", true), std::span<const std::uint8_t>(data)) == want)) << "the spacing and the case of the hexadecimal are spelling and not meaning";
    };

    "a non-repeating sequence covers its first S bits and counts every item past them"_test = [] {
        constexpr std::size_t           kExtra = 7UZ;
        const std::vector<std::uint8_t> mask{0xFFU, 0xE1U, 0x1DU, 0x9AU};
        const std::vector<std::uint8_t> data = randomItems(mask.size() + kExtra, 256U);

        AdditiveScrambler               block = make<AdditiveScrambler>(kSequence("FF E1 1D 9A", false));
        const std::vector<std::uint8_t> seen  = spans::run<std::uint8_t>(block, std::span<const std::uint8_t>(data)).samples;

        std::vector<std::uint8_t> want(data);
        for (std::size_t i = 0UZ; i < mask.size(); ++i) {
            want[i] = static_cast<std::uint8_t>(data[i] ^ mask[i]);
        }
        expect(that % (seen == want)) << "the items past the sequence leave unscrambled rather than XORed with something arbitrary";
        expect(eq(block.nUnscrambledItems(), static_cast<std::uint64_t>(kExtra))) << "a 1:1 block cannot drop an item, so what it does about an overrun is count it";
        expect(eq(block.nForcedTransitions(), 0ULL));
    };

    "a periodic epoch known in advance to outlast the sequence is refused at start()"_test = [] {
        const auto started = [](gr::property_map settings) {
            AdditiveScrambler block = make<AdditiveScrambler>(std::move(settings));
            return complaint([&block] { block.start(); });
        };

        // 4 bytes is 32 bits, and 8 items of 8 bits is 64, so the last 32 bits of every epoch would leave unscrambled
        const std::string refused = started(with(kSequence("FF E1 1D 9A", false), {{"reset_period", static_cast<gr::Size_t>(8)}}));
        expect(!refused.empty()) << "the shortfall is arithmetic here rather than a property of the stream";
        for (const std::string_view figure : {"32", "8"}) {
            expect(refused.contains(figure)) << std::format("the refusal names S, the period and the shortfall, and \"{}\" is missing from \"{}\"", figure, refused);
        }

        expect(started(with(kSequence("FF E1 1D 9A", false), {{"reset_period", static_cast<gr::Size_t>(4)}})).empty()) << "an epoch the sequence covers exactly is not an overrun";
        expect(started(with(kSequence("FF E1 1D 9A", true), {{"reset_period", static_cast<gr::Size_t>(8)}})).empty()) << "and a repeating sequence has no end to run past";
        expect(started(kSequence("FF E1 1D 9A", false)).empty()) << "without a period the overrun is a stream property and is the counter's, not start()'s";
    };

    "the mask's spelling is read rather than guessed at"_test = [] {
        const auto refuses = [](std::string_view spelling, std::string_view why) {
            const std::string message = complaint([spelling] { std::ignore = make<AdditiveScrambler>(kSequence(spelling, true)); });
            expect(!message.empty()) << why;
            expect(message.contains(spelling)) << std::format("{}: \"{}\" is missing from \"{}\"", why, spelling, message);
        };

        refuses("FF E", "an odd number of digits is half a byte and half a typo");
        refuses("FF G1", "a character that is not a hexadecimal digit must not silently become a different mask");
    };
};

const boost::ut::suite<"the forced-transition group"> forcedTransitionTests = [] {
    using namespace boost::ut;

    /// @brief The V-series-family shape: a degree-20 recursion, the output inverted, and the monitor on the two
    /// nearest transmitted bits. `force_transition_after` fires at 2^-N per item, so N is small enough here for a
    /// four-thousand-item run to reach it.
    const auto forced = [](gr::Size_t after) { return gr::property_map{{"taps", std::string("3,20")}, {"seed", std::string(20UZ, '1')}, {"invert_output", true}, {"force_transition_after", after}, {"monitor_delays", std::string("1,2")}}; };

    "the pair inverts exactly with the rule running, and both ends count the same firings"_test = [forced] {
        const std::vector<std::uint8_t> data = randomItems(4000UZ, 256U);

        MultiplicativeScrambler   scrambler   = make<MultiplicativeScrambler>(forced(8U));
        MultiplicativeDescrambler descrambler = make<MultiplicativeDescrambler>(forced(8U));

        const std::vector<std::uint8_t> channel   = spans::run<std::uint8_t>(scrambler, std::span<const std::uint8_t>(data)).samples;
        const std::vector<std::uint8_t> recovered = spans::run<std::uint8_t>(descrambler, std::span<const std::uint8_t>(channel)).samples;

        expect(that % (recovered == data)) << "the monitor reads only the transmitted stream, so both ends compute the same counter and the descrambler stays the exact inverse";
        expect(that % (channel != data));
        expect(eq(scrambler.nForcedTransitions(), descrambler.nForcedTransitions())) << "the two counters agreeing is the property the rule's invertibility rests on";
        expect(gt(scrambler.nForcedTransitions(), 0ULL)) << "at N = 8 the rule fires about once in 256 bits, so 32000 bits reach it and the count is not vacuous";
    };

    "the rule costs nothing when it is off, and the group refuses to describe a mechanism that is"_test = [] {
        const std::vector<std::uint8_t> data  = randomItems(600UZ, 256U);
        const gr::property_map          plain = {{"taps", std::string("3,20")}, {"seed", std::string(20UZ, '1')}};

        MultiplicativeScrambler off = make<MultiplicativeScrambler>(with(plain, {{"force_transition_after", static_cast<gr::Size_t>(0)}}));
        expect(that % (spans::run<std::uint8_t>(off, std::span<const std::uint8_t>(data)).samples == drive<MultiplicativeScrambler>(plain, std::span<const std::uint8_t>(data))));
        expect(eq(off.nForcedTransitions(), 0ULL));

        const auto refuses = [&plain](gr::property_map extra, std::string_view why) { expect(!complaint([&plain, &extra] { std::ignore = make<MultiplicativeScrambler>(with(plain, extra)); }).empty()) << why; };
        refuses({{"monitor_delays", std::string("1,2")}}, "a setting that describes a disabled mechanism is a settings error rather than a no-op");
        refuses({{"force_transition_after", static_cast<gr::Size_t>(8)}}, "and a counter with nothing to monitor is the same error the other way round");
        refuses({{"force_transition_after", static_cast<gr::Size_t>(1)}, {"monitor_delays", std::string("1,2")}}, "a modulus of one fires at every item");
        refuses({{"force_transition_after", static_cast<gr::Size_t>(8)}, {"monitor_delays", std::string("1,2,3")}}, "the monitor compares two delays, not three");
        refuses({{"force_transition_after", static_cast<gr::Size_t>(8)}, {"monitor_delays", std::string("2")}}, "nor one");
    };

    "the two ends must carry the same inversion"_test = [forced] {
        const std::vector<std::uint8_t> data     = randomItems(400UZ, 256U);
        const gr::property_map          straight = {{"taps", std::string("3,20")}, {"seed", std::string(20UZ, '1')}};

        expect(that % (throughPair<MultiplicativeScrambler, MultiplicativeDescrambler>(forced(32U), forced(32U), std::span<const std::uint8_t>(data)) == data));
        expect(that % (drive<MultiplicativeScrambler>(forced(32U), std::span<const std::uint8_t>(data)) != drive<MultiplicativeScrambler>(straight, std::span<const std::uint8_t>(data)))) //
            << "the inversion has to be observable at all";
        expect(that % (throughPair<MultiplicativeScrambler, MultiplicativeDescrambler>(forced(32U), straight, std::span<const std::uint8_t>(data)) != data)) //
            << "a descrambler that does not know about the inversion recovers something else, which is why the group is a setting on both blocks";
    };
};

const boost::ut::suite<"the record carrier"> recordCarrierTests = [] {
    using namespace boost::ut;
    using Record          = gr::DataSet<std::uint8_t>;
    using RecordScrambler = gr::blocks::digital::AdditiveScrambler<Record>;

    const auto recordOf = [](std::vector<std::uint8_t> values) {
        Record r;
        r.signal_values = std::move(values);
        r.extents.push_back(static_cast<std::int32_t>(r.signal_values.size()));
        r.signal_names.emplace_back("digital");
        r.timing_events.resize(1UZ);
        r.meta_information.resize(1UZ);
        r.meta_information[0UZ]["origin"] = std::string("qa");
        return r;
    };

    "the record carrier equals the stream carrier, record after record"_test = [&] {
        const std::vector<std::uint8_t> bits   = randomItems(200UZ, 2U);
        const std::vector<std::uint8_t> stream = epoch<AdditiveScrambler>(with(ccsdsSettings(), {{"bits_per_item", gr::Size_t{1U}}}), std::span<const std::uint8_t>(bits));

        RecordScrambler           block   = make<RecordScrambler>(with(ccsdsSettings(), {{"bits_per_item", gr::Size_t{1U}}}));
        const std::vector<Record> records = {recordOf(bits), recordOf(bits)};
        const auto                seen    = spans::run<Record>(block, std::span<const Record>(records));
        expect(eq(seen.samples.size(), 2UZ));
        for (const Record& r : seen.samples) {
            expect(std::ranges::equal(r.signal_values, stream)) << "each record is one epoch, item for item the stream's fresh register";
            expect(!r.meta_information.empty());
            if (!r.meta_information.empty()) {
                const auto entry = r.meta_information[0UZ].find(gr::property_map::key_type("origin"));
                expect(that % (entry != r.meta_information[0UZ].end())) << "the record's facts cross verbatim";
            }
        }
    };

    "the record form reproduces the published opening"_test = [&] {
        // the degree-8 sequence's first forty bits, 10.4.3 note 2's value, out of a zero record
        RecordScrambler           block = make<RecordScrambler>({{"profile", std::string("ccsds131")}});
        const std::vector<Record> zero  = {recordOf(std::vector<std::uint8_t>(5UZ, 0U))};
        const auto                seen  = spans::run<Record>(block, std::span<const Record>(zero));
        expect(eq(seen.samples.size(), 1UZ));
        if (!seen.samples.empty()) {
            const std::vector<std::uint8_t> want{0xFFU, 0x48U, 0x0EU, 0xC0U, 0x9AU};
            expect(std::ranges::equal(seen.samples[0UZ].signal_values, want)) << "FF 48 0E C0 9A from the record form as well";
        }
    };

    "the stream resets are inert on the record carrier"_test = [&] {
        const std::vector<std::uint8_t> bits  = randomItems(64UZ, 2U);
        const gr::property_map          plain = with(ccsdsSettings(), {{"bits_per_item", gr::Size_t{1U}}});
        const gr::property_map          noisy = with(plain, {{"reset_period", gr::Size_t{3U}}, {"reset_tag_key", std::string("some_key")}});

        RecordScrambler quiet   = make<RecordScrambler>(plain);
        RecordScrambler decoyed = make<RecordScrambler>(noisy);

        const std::vector<Record> records = {recordOf(bits)};
        const auto                a       = spans::run<Record>(quiet, std::span<const Record>(records));
        const auto                b       = spans::run<Record>(decoyed, std::span<const Record>(records));
        expect(eq(a.samples.size(), 1UZ));
        expect(eq(b.samples.size(), 1UZ));
        if (!a.samples.empty() && !b.samples.empty()) {
            expect(std::ranges::equal(a.samples[0UZ].signal_values, b.samples[0UZ].signal_values)) << "reset_period and reset_tag_key change nothing: the record boundary is the only epoch";
        }
    };
};

int main() { /* not needed for UT */ }
