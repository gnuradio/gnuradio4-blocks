#include <boost/ut.hpp>

#include <bit>
#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/digital/Scrambler.hpp>

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

/// @brief The fraction of @p items that are ones, bit by bit, which is what whitening is measured in.
[[nodiscard]] double onesFraction(std::span<const std::uint8_t> items) {
    std::size_t ones = 0UZ;
    for (const std::uint8_t item : items) {
        ones += static_cast<std::size_t>(std::popcount(item));
    }
    return static_cast<double>(ones) / static_cast<double>(8UZ * items.size());
}

/// @brief A generator named rather than spelled out, which is the whole of what a profile setting stages.
[[nodiscard]] gr::property_map kProfile(std::string_view name) { return gr::property_map{{"profile", std::string(name)}}; }

/// @brief A mask supplied outright, in the hexadecimal spelling a whitening sequence is published in.
[[nodiscard]] gr::property_map kSequence(std::string_view hex, bool repeat) { return gr::property_map{{"sequence", std::string(hex)}, {"sequence_repeat", repeat}}; }

[[nodiscard]] gr::Tag resetTag(std::size_t at, std::string label = std::string("frame")) { return gr::Tag{at, gr::property_map{{gr::property_map::key_type(gr::tag::TRIGGER_NAME.key()), std::move(label)}}}; }

} // namespace

const boost::ut::suite<"additive scrambler"> additiveTests = [] {
    using namespace boost::ut;

    "until it is configured the block is inert"_test = [] {
        // nothing downstream mistakes a pass-through for a scrambled stream
        AdditiveScrambler               block{};
        const std::vector<std::uint8_t> zeros(32UZ, std::uint8_t{0});
        std::vector<std::uint8_t>       out(32UZ);
        std::vector<gr::Tag>            published;
        spans::InputSpan<std::uint8_t>  inSpan{std::span<const std::uint8_t>(zeros)};
        spans::OutputSpan<std::uint8_t> outSpan{std::span<std::uint8_t>(out), 0UZ, &published};
        expect(that % (block.processBulk(inSpan, outSpan) == gr::work::Status::ERROR));
        expect(eq(inSpan.consumed, 0UZ));
    };

    "a rejected setting leaves the configuration it was applied to intact"_test = [] {
        const std::vector<std::uint8_t> data = randomItems(64UZ, 256U);
        const std::vector<std::uint8_t> want = epoch<AdditiveScrambler>(ccsdsSettings(), std::span<const std::uint8_t>(data));

        AdditiveScrambler block = make<AdditiveScrambler>(ccsdsSettings());
        expect(throws([&block] { restage(block, {{"seed", std::string("1111")}}); }));
        block.reset();
        expect(that % (spans::run<std::uint8_t>(block, std::span<const std::uint8_t>(data)).samples == want)) << "a half-applied configuration would leave the generator somewhere neither setting names";
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
};

const boost::ut::suite<"multiplicative scrambler"> multiplicativeScramblerTests = [] {
    using namespace boost::ut;

    "a seeded register whitens a zero input"_test = [] {
        const std::vector<std::uint8_t> zeros(4096UZ, std::uint8_t{0});
        const double                    fraction = onesFraction(std::span<const std::uint8_t>(drive<MultiplicativeScrambler>(ituFormASettings(), std::span<const std::uint8_t>(zeros))));
        expect(ge(fraction, 0.45)) << std::format("a seeded register whitens the same input: {:.4f} ones", fraction);
        expect(le(fraction, 0.55)) << std::format("a seeded register whitens the same input: {:.4f} ones", fraction);
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
};

const boost::ut::suite<"multiplicative descrambler"> multiplicativeDescramblerTests = [] {
    using namespace boost::ut;

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

    "all three are admissible for forwarding every key"_test = [] {
        static_assert(AdditiveScrambler::keepsTagOffsets());
        static_assert(MultiplicativeScrambler::keepsTagOffsets());
        static_assert(MultiplicativeDescrambler::keepsTagOffsets(), "each is 1:1 on synchronous ports, declares no resampling or stride and supplies no forwardTags()");
        expect(true) << "the compile-time half of the every-key default, which costs nothing to keep asserted";
    };
};

const boost::ut::suite<"named profiles"> profileTests = [] {
    using namespace boost::ut;

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

    "a non-repeating sequence fires no forced transition"_test = [] {
        const std::vector<std::uint8_t> data  = randomItems(11UZ, 256U);
        AdditiveScrambler               block = make<AdditiveScrambler>(kSequence("FF E1 1D 9A", false));
        std::ignore                           = spans::run<std::uint8_t>(block, std::span<const std::uint8_t>(data));
        expect(eq(block.nForcedTransitions(), 0ULL)) << "the forced-transition rule belongs to the multiplicative pair";
    };
};

const boost::ut::suite<"the forced-transition group"> forcedTransitionTests = [] {
    using namespace boost::ut;

    "the rule fires no transition when it is off"_test = [] {
        const std::vector<std::uint8_t> data  = randomItems(600UZ, 256U);
        const gr::property_map          plain = {{"taps", std::string("3,20")}, {"seed", std::string(20UZ, '1')}};

        MultiplicativeScrambler off = make<MultiplicativeScrambler>(with(plain, {{"force_transition_after", static_cast<gr::Size_t>(0)}}));
        std::ignore                 = spans::run<std::uint8_t>(off, std::span<const std::uint8_t>(data));
        expect(eq(off.nForcedTransitions(), 0ULL));
    };
};

int main() { /* not needed for UT */ }
