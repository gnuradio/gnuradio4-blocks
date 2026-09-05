#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/digital/ManchesterCombine.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

/*
 * The block reads a Manchester chip stream that has already been detected, so what this file builds is the detector's
 * output rather than the air: a BPSK waveform at four samples a chip, decimated at a stated sampling phase and
 * differentially detected, which is the arithmetic a noncoherent demodulator runs. The negative control is the fixed
 * two-tap combine the same chain would otherwise use, and it is shown to fail at BOTH pairings rather than at one,
 * because the operation and not the grid is what separates them.
 */
namespace {

using gr::blocks::digital::ManchesterCombine;
using gr::testing::ProcessFunction;
using gr::testing::TagSink;
using gr::testing::TagSource;

using C = std::complex<double>;

constexpr std::size_t kSamplesPerChip   = 4UZ;
constexpr std::size_t kSamplingPhase    = 2UZ;  ///< mid-chip, the instant a timing loop settles on
constexpr double      kCarrierPhase     = 0.4;  ///< an arbitrary carrier phase, which the differential product removes
constexpr double      kCyclesPerChip    = 0.01; ///< a residual frequency error, which the product leaves as a scale
constexpr std::size_t kBits             = 4000UZ;
constexpr std::size_t kAveragingSymbols = 256UZ; ///< the block's own defaults, which the derived bounds below read
constexpr std::size_t kHoldSymbols      = 64UZ;

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
};

/// @brief @p count data bits of `+1` and `-1`, exactly half of each, so that the levels the negative control produces
/// are equiprobable by construction and its mean magnitude over root mean square is `1/sqrt(2)` to more places than a
/// draw would give.
[[nodiscard]] std::vector<int> dataBits(std::size_t count, std::uint64_t seed) {
    Rng              rng{seed};
    std::vector<int> bits(count, 1);
    for (std::size_t k = 0UZ; k < count / 2UZ; ++k) {
        bits[k] = -1;
    }
    for (std::size_t k = count; k > 1UZ; --k) { // Fisher-Yates, so the halves are interleaved rather than blocked
        std::swap(bits[k - 1UZ], bits[rng.next() % k]);
    }
    return bits;
}

/**
 * @brief The chip stream a Manchester DBPSK transmitter puts on the air, given the data bits @p data.
 *
 * The data is differentially encoded first, because a differential detector recovers the product of adjacent symbols
 * rather than a symbol: `b[k] = b[k-1] a[k]` makes that product `a[k]`, so what comes back is the data and not a
 * running product of it. Each `b[k]` is then two chips of opposite sign, which is the line code.
 */
[[nodiscard]] std::vector<int> manchesterChips(std::span<const int> data) {
    std::vector<int> chips;
    chips.reserve(2UZ * data.size());
    int running = 1;
    for (const int bit : data) {
        running *= bit;
        chips.push_back(running);
        chips.push_back(-running);
    }
    return chips;
}

/**
 * @brief What a differential detector produces from @p chips: a BPSK waveform at four samples a chip, decimated at
 * `kSamplingPhase`, and `Re(z[n] conj(z[n-1]))` over the result.
 *
 * The carrier phase and a residual frequency error are carried through, so the stream is the one a noncoherent
 * receiver actually holds. The product cancels the phase and leaves the cosine of one chip's frequency error as a
 * scale on every item alike, which is what makes the detector noncoherent and what makes every magnitude equal.
 */
[[nodiscard]] std::vector<float> detected(std::span<const int> chips) {
    std::vector<C> waveform(chips.size() * kSamplesPerChip);
    for (std::size_t n = 0UZ; n < waveform.size(); ++n) {
        const double angle = kCarrierPhase + 2. * std::numbers::pi * kCyclesPerChip * static_cast<double>(n) / static_cast<double>(kSamplesPerChip);
        waveform[n]        = C(std::cos(angle), std::sin(angle)) * static_cast<double>(chips[n / kSamplesPerChip]);
    }

    std::vector<C> sampled;
    sampled.reserve(chips.size());
    for (std::size_t k = 0UZ; k < chips.size(); ++k) {
        sampled.push_back(waveform[k * kSamplesPerChip + kSamplingPhase]);
    }

    std::vector<float> soft(sampled.size(), 0.f); // item 0 has no predecessor, which is the one a detector cannot decide
    for (std::size_t n = 1UZ; n < sampled.size(); ++n) {
        soft[n] = static_cast<float>(std::real(sampled[n] * std::conj(sampled[n - 1UZ])));
    }
    return soft;
}

/// @brief The chip decisions themselves, which is the other stream the block reads.
[[nodiscard]] std::vector<float> chipDecisions(std::span<const int> data) {
    std::vector<float> chips;
    chips.reserve(2UZ * data.size());
    for (const int bit : data) {
        chips.push_back(static_cast<float>(bit));
        chips.push_back(static_cast<float>(-bit));
    }
    return chips;
}

/// @brief The fixed two-tap decimating combine at the pairing @p parity: half the difference of an adjacent pair.
[[nodiscard]] std::vector<float> twoTapCombine(std::span<const float> soft, std::size_t parity) {
    std::vector<float> bits;
    for (std::size_t n = parity; n + 1UZ < soft.size(); n += 2UZ) {
        bits.push_back(0.5f * (soft[n] - soft[n + 1UZ]));
    }
    return bits;
}

/// @brief The mean magnitude over the root mean square: 1 for a clean antipodal stream, `1/sqrt(2)` for a `{0, 1}` one.
[[nodiscard]] double meanOverRms(std::span<const float> values) {
    double magnitude = 0.;
    double square    = 0.;
    for (const float value : values) {
        magnitude += std::abs(static_cast<double>(value));
        square += static_cast<double>(value) * static_cast<double>(value);
    }
    const double count = static_cast<double>(std::max<std::size_t>(values.size(), 1UZ));
    return square > 0. ? magnitude / (count * std::sqrt(square / count)) : 0.;
}

[[nodiscard]] double mean(std::span<const float> values) {
    double sum = 0.;
    for (const float value : values) {
        sum += static_cast<double>(value);
    }
    return values.empty() ? 0. : sum / static_cast<double>(values.size());
}

/// @brief The three streams one graph run through the block produced, and the tags that reached the bit sink.
struct Combined {
    std::vector<float>   bits{};
    std::vector<float>   phase{};
    std::vector<float>   confidence{};
    std::vector<gr::Tag> tags{};
};

[[nodiscard]] Combined through(gr::property_map settings, std::span<const float> soft, std::span<const gr::Tag> incoming = {}) {
    gr::Graph  flow;
    const auto values = gr::Tensor<float>(soft.begin(), soft.end());
    auto&      source = flow.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(soft.size())}, {"values", values}, {"sample_rate", 800.f}});
    source._tags.assign(incoming.begin(), incoming.end());
    std::ranges::sort(source._tags, std::ranges::less{}, &gr::Tag::index);

    auto& block      = flow.emplaceBlock<ManchesterCombine>(std::move(settings));
    auto& bits       = flow.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "bits"}});
    auto& phase      = flow.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "phase"}});
    auto& confidence = flow.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "confidence"}});

    boost::ut::expect(flow.connect<"out", "in">(source, block).has_value());
    boost::ut::expect(flow.connect<"out", "in">(block, bits).has_value());
    boost::ut::expect(flow.connect<"chip_phase", "in">(block, phase).has_value());
    boost::ut::expect(flow.connect<"confidence", "in">(block, confidence).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(flow)).has_value()) << boost::ut::fatal;
    boost::ut::expect(scheduler.runAndWait().has_value()) << boost::ut::fatal;

    return {std::vector<float>(bits._samples.begin(), bits._samples.end()), std::vector<float>(phase._samples.begin(), phase._samples.end()), std::vector<float>(confidence._samples.begin(), confidence._samples.end()), bits._tags};
}

/**
 * @brief Output items from @p from on whose decision is not `sign * data[k + offset]`.
 *
 * @p sign is the overall inversion the chain carries and @p offset the place the pairing puts a bit at; both are
 * stated by the caller from the arithmetic rather than searched for, so a shift is a failure and not a fit.
 */
[[nodiscard]] std::size_t disagreements(std::span<const float> bits, std::span<const int> data, std::size_t from, std::size_t offset, int sign) {
    std::size_t wrong = 0UZ;
    for (std::size_t k = from; k < bits.size() && k + offset < data.size(); ++k) {
        wrong += (bits[k] >= 0.f ? sign : -sign) == data[k + offset] ? 0UZ : 1UZ;
    }
    return wrong;
}

/// @brief The first output item at which `chip_phase` reads @p wanted, or the stream's length if it never does.
[[nodiscard]] std::size_t settlesAt(std::span<const float> phase, float wanted) {
    std::size_t at = 0UZ;
    while (at < phase.size() && phase[at] != wanted) {
        ++at;
    }
    return at;
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

} // namespace

const boost::ut::suite<"ManchesterCombine"> manchesterCombineTests = [] {
    using namespace boost::ut;

    // The transmitted bits come back exactly, from both starting chip phases
    "a noiseless differential stream decodes at either pairing, pinned"_test = [] {
        const std::vector<int>   data  = dataBits(kBits, 20260903ULL);
        const std::vector<int>   chips = manchesterChips(std::span<const int>(data));
        const std::vector<float> even  = detected(std::span<const int>(chips));
        // the same air read one chip late, so a pair now starts at an odd chip index and the first bit is a[1]
        const std::vector<float> odd(even.begin() + 1, even.end());

        // inside a bit the detector multiplies two antipodal chips, so the recovered bit carries an overall minus
        const auto pinnedAt = [&data](std::string_view label, const std::vector<float>& soft, std::int32_t pinned, std::size_t from, std::size_t offset) {
            const Combined run = through({{"input", std::string("differential")}, {"phase", pinned}}, std::span<const float>(soft));
            expect(eq(run.bits.size(), soft.size() / 2UZ)) << label << ": one bit for two chips";
            expect(eq(disagreements(std::span<const float>(run.bits), std::span<const int>(data), from, offset, -1), 0UZ)) << label << ": every transmitted bit comes back";
            expect(gt(meanOverRms(std::span<const float>(run.bits).subspan(from)), 0.9999)) << label << ": the decisions are antipodal, so the mean magnitude is the root mean square";
            expect(std::ranges::all_of(run.phase, [pinned](float value) { return value == static_cast<float>(pinned); })) << label << ": the pinned grid is what the port reports";
        };
        pinnedAt("phase 0", even, 0, 1UZ, 0UZ);
        pinnedAt("phase 1", odd, 1, 0UZ, 1UZ);
    };

    // The estimating arm: the same stream with the pairing left to the block
    "the estimate finds the pairing on its own at either phase"_test = [] {
        const std::vector<int>   data  = dataBits(kBits, 20260903ULL);
        const std::vector<int>   chips = manchesterChips(std::span<const int>(data));
        const std::vector<float> even  = detected(std::span<const int>(chips));
        const std::vector<float> odd(even.begin() + 1, even.end());

        const auto estimatesAt = [&data](std::string_view label, const std::vector<float>& soft, float expected, std::size_t offset) {
            const Combined run = through({{"input", std::string("differential")}}, std::span<const float>(soft));
            expect(eq(run.phase.size(), run.bits.size())) << label;

            // The two means start equal and separate as the constant accumulates, so the estimate is right within a
            // few symbols; the grid then waits out `hold_symbols` of unbroken disagreement, which is what acquisition
            // costs a stream that opens on the other parity. A chain that cannot spend it pins `phase`.
            const std::size_t settled = settlesAt(std::span<const float>(run.phase), expected);
            std::println("[combine] {} estimating: settles at symbol {}, mean confidence {:.3f}, final {:.3f}", label, settled, mean(std::span<const float>(run.confidence)), run.confidence.empty() ? 0.f : run.confidence.back());
            expect(le(settled, kHoldSymbols + 8UZ)) << std::format("{}: the grid settles after {} symbols, against a hold of {}", label, settled, kHoldSymbols);
            expect(eq(disagreements(std::span<const float>(run.bits), std::span<const int>(data), settled + 1UZ, offset, -1), 0UZ)) << label << ": every bit after the grid settles";
            expect(gt(mean(std::span<const float>(run.confidence)), 0.8)) << label << ": one parity's mean is a constant and the other's is nothing";
        };
        estimatesAt("phase 0", even, 0.f, 0UZ);
        estimatesAt("phase 1", odd, 1.f, 1UZ);
    };

    // The negative control fails at BOTH pairings, and the arithmetic says why
    "the fixed two-tap combine is unipolar at either pairing"_test = [] {
        const std::vector<int>   data  = dataBits(kBits, 7ULL);
        const std::vector<int>   chips = manchesterChips(std::span<const int>(data));
        const std::vector<float> soft  = detected(std::span<const int>(chips));

        const Combined selected  = through({{"input", std::string("differential")}, {"phase", std::int32_t{0}}}, std::span<const float>(soft));
        const double   antipodal = meanOverRms(std::span<const float>(selected.bits).subspan(1UZ));
        expect(gt(antipodal, 0.9999)) << "the selected parity is antipodal";

        for (const std::size_t parity : {0UZ, 1UZ}) {
            // 0.5 (1 - b b') takes two values and one of them is zero, so the mean magnitude is 0.5 and the root
            // mean square 1/sqrt(2) -- the ratio a stream of clean decisions cannot produce
            const std::vector<float> combined = twoTapCombine(std::span<const float>(soft).subspan(1UZ), parity);
            const double             figure   = meanOverRms(std::span<const float>(combined));
            const std::size_t        atZero   = static_cast<std::size_t>(std::ranges::count_if(combined, [](float value) { return std::abs(value) < 1e-6f; }));
            std::println("[combine] two-tap pairing {}: mean over rms {:.4f} against selection's {:.4f}, {:.3f} of its decisions on the threshold", parity, figure, antipodal, static_cast<double>(atZero) / static_cast<double>(combined.size()));
            expect(lt(std::abs(figure - 0.70710678), 0.001)) << std::format("pairing {}: the two-tap combine reads 1/sqrt(2), not 1", parity);
            expect(lt(std::abs(static_cast<double>(atZero) / static_cast<double>(combined.size()) - 0.5), 0.01)) << std::format("pairing {}: half its decisions sit on the slicer's threshold", parity);
        }
    };

    // A chip-phase step costs at most hold_symbols bits, and the port says the grid moved
    "a chip-phase step is held against and then taken"_test = [] {
        constexpr std::size_t    kStep = 1000UZ; ///< the bit at which one chip is lost, which moves the pairing
        const std::vector<int>   data  = dataBits(kBits, 99ULL);
        const std::vector<int>   chips = manchesterChips(std::span<const int>(data));
        const std::vector<float> whole = detected(std::span<const int>(chips));

        // dropping one chip IS a step of the chip phase: everything after it pairs on the other parity
        std::vector<float> stepped(whole.begin(), whole.begin() + static_cast<std::ptrdiff_t>(2UZ * kStep));
        stepped.insert(stepped.end(), whole.begin() + static_cast<std::ptrdiff_t>(2UZ * kStep + 1UZ), whole.end());

        const Combined run = through({{"input", std::string("differential")}}, std::span<const float>(stepped));
        expect(eq(disagreements(std::span<const float>(run.bits).first(kStep), std::span<const int>(data), 1UZ, 0UZ, -1), 0UZ)) << "every bit before the step";

        std::size_t moved = kStep;
        while (moved < run.phase.size() && run.phase[moved] == run.phase[kStep - 1UZ]) {
            ++moved;
        }
        // What a step costs is the averaging window turning over plus the hold: the mean the step made stale decays
        // as `exp(-n / averaging_symbols)` while the other grows as `1 - exp(-n / averaging_symbols)`, so they cross
        // at `averaging_symbols ln 2` symbols and the grid then waits out `hold_symbols` of unbroken disagreement.
        const std::size_t crossover = static_cast<std::size_t>(std::ceil(static_cast<double>(kAveragingSymbols) * std::numbers::ln2));
        std::println("[combine] chip-phase step at symbol {}: the grid moves {} symbols later, against {} of window crossover and {} of hold", kStep, moved - kStep, crossover, kHoldSymbols);
        expect(neq(moved, run.phase.size())) << "the chip_phase port reports the move";
        expect(ge(moved - kStep, kHoldSymbols)) << "no move is taken before the hold has run";
        expect(le(moved - kStep, crossover + kHoldSymbols + 8UZ)) << std::format("the grid moves {} symbols after the step", moved - kStep);

        // one chip is gone, so from the move on the bits are the data one place further on
        expect(eq(disagreements(std::span<const float>(run.bits), std::span<const int>(data), moved + 1UZ, 1UZ, -1), 0UZ)) << "and the stream recovers whole";
    };

    // the `chips` arm: chip decisions, where a bit IS half the difference of a pair
    "chip decisions combine by difference, at the pairing the products name"_test = [] {
        const std::vector<int>   data  = dataBits(kBits, 31ULL);
        const std::vector<float> chips = chipDecisions(std::span<const int>(data));
        const std::vector<float> shifted(chips.begin() + 1, chips.end());

        const auto combinesAt = [&data](std::string_view label, const std::vector<float>& soft, float expected) {
            const Combined    run     = through({{"input", std::string("chips")}}, std::span<const float>(soft));
            const std::size_t settled = settlesAt(std::span<const float>(run.phase), expected);
            std::println("[combine] chips {}: settles at symbol {}, mean confidence {:.3f}", label, settled, mean(std::span<const float>(run.confidence)));
            expect(le(settled, kHoldSymbols + 8UZ)) << std::format("{}: the products separate after {} symbols, against a hold of {}", label, settled, kHoldSymbols);
            expect(eq(disagreements(std::span<const float>(run.bits), std::span<const int>(data), settled + 1UZ, 0UZ, 1), 0UZ)) << label << ": half the difference of a pair is the bit";
            expect(gt(meanOverRms(std::span<const float>(run.bits).subspan(settled + 1UZ)), 0.9999)) << label << ": and it is antipodal";
        };
        combinesAt("phase 0", chips, 0.f);
        combinesAt("phase 1", shifted, 1.f);
    };

    // The block is exactly 2:1, so the core halves a forwarded sample_rate
    "rate and sample_rate"_test = [] {
        const std::vector<int>         data  = dataBits(512UZ, 5ULL);
        const std::vector<int>         chips = manchesterChips(std::span<const int>(data));
        const std::vector<float>       soft  = detected(std::span<const int>(chips));
        const std::array<gr::Tag, 1UZ> rated{gr::Tag{0UZ, {{gr::property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()), 800.0F}}}};

        const Combined run = through({{"input", std::string("differential")}}, std::span<const float>(soft), std::span<const gr::Tag>(rated));
        expect(eq(run.bits.size(), soft.size() / 2UZ)) << "two chips in, one bit out";

        bool sawRate = false;
        for (const gr::Tag& seenTag : run.tags) {
            if (sampleRateOf(seenTag) > 0.0F) {
                sawRate = true;
                expect(eq(sampleRateOf(seenTag), 400.0F)) << "800 chips a second in, 400 bits a second out";
            }
        }
        expect(sawRate) << "the sample_rate tag reached the sink";
    };

    "the settings that have no meaning are refused"_test = [] {
        const auto build = [](gr::property_map settings) {
            ManchesterCombine block(std::move(settings));
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
            block.start();
        };
        expect(throws([&build] { build({{"input", std::string("soft")}}); })) << "a stream this block cannot read is named, not guessed at";
        expect(throws([&build] { build({{"phase", std::int32_t{2}}}); })) << "a pairing grid has two parities";
        expect(throws([&build] { build({{"averaging_symbols", gr::Size_t{0}}}); })) << "a mean over no symbols is not a mean";
        expect(nothrow([&build] { build({{"input", std::string("chips")}, {"phase", std::int32_t{1}}}); }));
    };
};

int main() { /* tests are statically registered */ }
