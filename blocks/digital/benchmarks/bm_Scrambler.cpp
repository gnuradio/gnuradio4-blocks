#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/digital/Scrambler.hpp>

// The cost of a scrambler is proportional to the bits it moves and not to the items carrying them, so every arm is
// reported per item and per bit, and one item of eight bits sits beside eight items of one bit for comparison. Each
// configured block is measured against the definition it implements — the bit-at-a-time reference loop, one feedback
// per bit — so a fast form that is not fast can be deleted with evidence rather than defended with an argument. Three
// further arms isolate what the block adds to the kernel: the epoch scan over an empty tag list, the same with the
// scan turned off, and a periodic reset that splits every call into epochs.
//
// Two later arms measure the mask sources and the forced-transition rule. An explicit sequence is the table form with
// the table supplied instead of walked, so it sits beside the table entries above it; the forced-transition pair is
// measured with the rule off and at N = 32, and the off arm has to land within noise of the plain multiplicative arm
// of the same tap set, or the rule is not additive to the family's cost.

namespace {

using AdditiveScrambler = gr::blocks::digital::AdditiveScrambler<std::uint8_t>;
using gr::blocks::digital::MultiplicativeDescrambler;
using gr::blocks::digital::MultiplicativeScrambler;
using gr::blocks::testing::bench::Arm;
using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;

constexpr std::size_t kItemsPerCall = 65536UZ;
constexpr std::size_t kRepeats      = 9UZ;

/// @brief A named polynomial with the register width that fixes its seed length.
struct Entry {
    std::string_view label;
    std::string_view taps;
    std::size_t      degree;
};

// The additive entries are three: two that reach the table form and one that cannot, the table entries
// being the smallest and the largest the threshold admits.
constexpr std::array<Entry, 3UZ>    kAdditive{Entry{"CCSDS 1,3,5,8", "1,3,5,8", 8UZ}, Entry{"DVB 14,15", "14,15", 15UZ}, Entry{"ITU 18,23", "18,23", 23UZ}};
constexpr std::array<Entry, 2UZ>    kMultiplicative{Entry{"ITU 18,23", "18,23", 23UZ}, Entry{"802.11 4,7", "4,7", 7UZ}};
constexpr std::array<unsigned, 2UZ> kWidths{1U, 8U};

/// @brief The DVB seed names the sequence its standard's figure prints; the other entries take all ones.
[[nodiscard]] std::string seedFor(const Entry& entry) { //
    return entry.taps == "14,15" ? std::string("000000111111011") : std::string(entry.degree, '1');
}

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
    return block;
}

template<typename TBlock>
[[nodiscard]] TBlock configured(const Entry& entry, unsigned bitsPerItem) {
    return make<TBlock>({{"taps", std::string(entry.taps)}, {"seed", seedFor(entry)}, {"bits_per_item", static_cast<gr::Size_t>(bitsPerItem)}});
}

/// @brief The kernel the block of the same configuration holds, for the reference loop to run against.
[[nodiscard]] gr::digital::ScramblerConfig kernelFor(const Entry& entry, gr::digital::ScramblerMode mode, unsigned bitsPerItem) {
    gr::digital::ScramblerConfig config;
    gr::digital::configureScrambler(config, gr::digital::tapsFromDelayList(entry.taps), gr::digital::seedFromBitString(seedFor(entry), static_cast<std::uint8_t>(entry.degree)), mode, bitsPerItem, gr::digital::BitOrder::MsbFirst);
    return config;
}

/// @brief A 64-byte mask in the hexadecimal spelling `sequence` reads, which is 512 bits and so covers the whole of a
/// 65536-item call by tiling and nothing else.
[[nodiscard]] std::string maskHex() {
    std::string   text;
    std::uint64_t state = 0x2545f4914f6cdd1dULL;
    for (std::size_t byte = 0UZ; byte < 64UZ; ++byte) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        std::format_to(std::back_inserter(text), "{:02X}", static_cast<unsigned>(state & 0xFFULL));
    }
    return text;
}

/// @brief The V-series-family shape: a degree-20 recursion, the output inverted, and the monitor on the
/// two nearest transmitted bits, at the stated modulus. `after` of zero stages the group and leaves it disabled, which
/// is the arm that has to land on the plain one beside it.
[[nodiscard]] gr::property_map forcedSettings(gr::Size_t after) {
    gr::property_map settings{{"taps", std::string("3,20")}, {"seed", std::string(20UZ, '1')}, {"invert_output", after != 0U}, {"force_transition_after", after}};
    if (after != 0U) {
        settings.insert_or_assign("monitor_delays", std::string("1,2"));
    }
    return settings;
}

[[nodiscard]] std::vector<std::uint8_t> randomItems(unsigned bitsPerItem) {
    std::vector<std::uint8_t> data(kItemsPerCall);
    std::uint64_t             state = 0x9e3779b97f4a7c15ULL;
    const std::uint8_t        mask  = static_cast<std::uint8_t>((1U << bitsPerItem) - 1U);
    for (std::uint8_t& item : data) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        item = static_cast<std::uint8_t>(state & mask);
    }
    return data;
}

} // namespace

int main() {
    const std::array<std::vector<std::uint8_t>, kWidths.size()> inputs{randomItems(kWidths[0UZ]), randomItems(kWidths[1UZ])};
    std::vector<std::uint8_t>                                   output(kItemsPerCall);
    std::vector<std::uint8_t>                                   copied(kItemsPerCall);

    // everything is built before anything is timed, so an arm times the sample path and never a configuration. Each
    // family holds one entry per (width, polynomial) pair, in the order the arms below read them.
    std::vector<AdditiveScrambler>            additive;
    std::vector<MultiplicativeScrambler>      scramblers;
    std::vector<MultiplicativeDescrambler>    descramblers;
    std::vector<gr::digital::ScramblerConfig> additiveKernels;
    std::vector<gr::digital::ScramblerConfig> scramblerKernels;
    std::vector<gr::digital::ScramblerConfig> descramblerKernels;
    for (const unsigned bits : kWidths) {
        for (const Entry& entry : kAdditive) {
            additive.push_back(configured<AdditiveScrambler>(entry, bits));
            additiveKernels.push_back(kernelFor(entry, gr::digital::ScramblerMode::Additive, bits));
        }
        for (const Entry& entry : kMultiplicative) {
            scramblers.push_back(configured<MultiplicativeScrambler>(entry, bits));
            scramblerKernels.push_back(kernelFor(entry, gr::digital::ScramblerMode::MultiplicativeScramble, bits));
            descramblers.push_back(configured<MultiplicativeDescrambler>(entry, bits));
            descramblerKernels.push_back(kernelFor(entry, gr::digital::ScramblerMode::MultiplicativeDescramble, bits));
        }
    }
    AdditiveScrambler unscanned = make<AdditiveScrambler>({{"taps", std::string("1,3,5,8")}, {"seed", std::string("11111111")}, {"reset_tag_key", std::string("")}});
    AdditiveScrambler periodic  = make<AdditiveScrambler>({{"taps", std::string("1,3,5,8")}, {"seed", std::string("11111111")}, {"reset_period", static_cast<gr::Size_t>(1024)}});
    AdditiveScrambler supplied  = make<AdditiveScrambler>({{"sequence", maskHex()}, {"sequence_repeat", true}});
    AdditiveScrambler profiled  = make<AdditiveScrambler>({{"profile", std::string("ccsds131")}});

    MultiplicativeScrambler   plainPair   = make<MultiplicativeScrambler>({{"taps", std::string("3,20")}, {"seed", std::string(20UZ, '1')}});
    MultiplicativeScrambler   forcedOff   = make<MultiplicativeScrambler>(forcedSettings(0U));
    MultiplicativeScrambler   forcedOn    = make<MultiplicativeScrambler>(forcedSettings(32U));
    MultiplicativeDescrambler unforcedOff = make<MultiplicativeDescrambler>(forcedSettings(0U));
    MultiplicativeDescrambler unforcedOn  = make<MultiplicativeDescrambler>(forcedSettings(32U));

    std::vector<Arm> arms;

    /// @brief A block arm: one call over the whole input, published in one span, at @p bits bits per item.
    const auto block = [&arms, &inputs, &output](std::string label, auto& scrambler, unsigned bits) {
        arms.push_back({std::move(label),
            [&scrambler, &inputs, &output, bits] {
                InputSpan<std::uint8_t>  inSpan{std::span<const std::uint8_t>(inputs[bits == 1U ? 0UZ : 1UZ])};
                OutputSpan<std::uint8_t> outSpan{std::span<std::uint8_t>(output)};
                std::ignore = scrambler.processBulk(inSpan, outSpan);
                return static_cast<double>(output[kItemsPerCall / 2UZ]);
            },
            kItemsPerCall * bits});
    };

    /// @brief The comparison arm for the block above it: the same conversion, one feedback per bit.
    const auto reference = [&arms, &inputs, &output](gr::digital::ScramblerConfig& config, unsigned bits) {
        arms.push_back({"  the same, reference loop",
            [&config, &inputs, &output, bits] {
                gr::digital::detail::scrambleReference(config, std::span<const std::uint8_t>(inputs[bits == 1U ? 0UZ : 1UZ]), std::span<std::uint8_t>(output));
                return static_cast<double>(output[kItemsPerCall / 2UZ]);
            },
            kItemsPerCall * bits});
    };

    for (std::size_t w = 0UZ; w < kWidths.size(); ++w) {
        const unsigned bits = kWidths[w];
        for (std::size_t i = 0UZ; i < kAdditive.size(); ++i) {
            const std::size_t at = w * kAdditive.size() + i;
            block(std::format("AdditiveScrambler {}, {} bit/item", kAdditive[i].label, bits), additive[at], bits);
            reference(additiveKernels[at], bits);
        }
        for (std::size_t i = 0UZ; i < kMultiplicative.size(); ++i) {
            const std::size_t at = w * kMultiplicative.size() + i;
            block(std::format("MultiplicativeScrambler {}, {} bit/item", kMultiplicative[i].label, bits), scramblers[at], bits);
            reference(scramblerKernels[at], bits);
            block(std::format("MultiplicativeDescrambler {}, {} bit/item", kMultiplicative[i].label, bits), descramblers[at], bits);
            reference(descramblerKernels[at], bits);
        }
    }

    block("AdditiveScrambler CCSDS, no epoch scan", unscanned, 8U);
    block("AdditiveScrambler CCSDS, 64 epochs a call", periodic, 8U);
    block("AdditiveScrambler CCSDS by profile name", profiled, 8U);
    block("AdditiveScrambler 512-bit supplied sequence", supplied, 8U);
    block("MultiplicativeScrambler 3,20, no rule staged", plainPair, 8U);
    block("MultiplicativeScrambler 3,20, rule staged off", forcedOff, 8U);
    block("MultiplicativeScrambler 3,20, rule at N = 32", forcedOn, 8U);
    block("MultiplicativeDescrambler 3,20, rule off", unforcedOff, 8U);
    block("MultiplicativeDescrambler 3,20, rule at N = 32", unforcedOn, 8U);
    arms.push_back({"a plain span copy, the floor",
        [&inputs, &copied] {
            std::ranges::copy(inputs[1UZ], copied.begin());
            return static_cast<double>(copied[kItemsPerCall / 2UZ]);
        },
        kItemsPerCall * 8UZ});

    gr::blocks::testing::bench::report(std::span<Arm>(arms), kItemsPerCall, kRepeats);
}
