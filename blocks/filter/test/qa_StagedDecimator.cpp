#include <boost/ut.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <numbers>
#include <print>
#include <span>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/filter/FirFilter.hpp>
#include <gnuradio-4.0/filter/StagedDecimator.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::filter::FirFilter;
using gr::blocks::filter::StagedDecimator;
using CF       = std::complex<float>;
namespace test = gr::blocks::testing::span;

/// A 64-bit stream offset narrowed to an index; a template, so the cast stands where `std::size_t` is 32 bits and is
/// not a useless cast where the two are the same 64-bit type.
template<std::integral To, std::integral From>
[[nodiscard]] constexpr To narrowIndex(From value) noexcept {
    return static_cast<To>(value);
}

template<typename T>
[[nodiscard]] StagedDecimator<T> makeBlock(gr::property_map settings) {
    StagedDecimator<T> block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
    return block;
}

/// @brief Drive the block over @p input in calls of @p chunkOutputs outputs, which is what the framework hands it.
template<typename T>
[[nodiscard]] auto run(StagedDecimator<T>& block, const std::vector<T>& input, std::size_t chunkOutputs = 0UZ, std::span<const gr::Tag> tags = {}) {
    const std::size_t d = static_cast<std::size_t>(block.decimation);
    return test::runDecimating<T>(block, std::span<const T>(input), chunkOutputs * d, d, tags);
}

/// @brief A deterministic stand-in for randomness: the same numbers on every machine and every run.
struct Noise {
    std::uint64_t state = 0x2545F4914F6CDD1DULL;

    [[nodiscard]] float next() noexcept {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return static_cast<float>(static_cast<double>(state >> 11) / 9007199254740992.0 * 2.0 - 1.0);
    }
};

template<typename T>
[[nodiscard]] std::vector<T> noise(std::size_t n, std::uint64_t seed) {
    Noise          source{seed};
    std::vector<T> out(n);
    for (T& v : out) {
        if constexpr (std::same_as<T, CF>) {
            v = CF{source.next(), source.next()};
        } else {
            v = source.next();
        }
    }
    return out;
}

[[nodiscard]] std::vector<CF> tone(std::size_t n, double frequency) {
    std::vector<CF> x(n);
    for (std::size_t i = 0UZ; i < n; ++i) {
        const double angle = 2.0 * std::numbers::pi * frequency * static_cast<double>(i);
        x[i]               = CF{static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle))};
    }
    return x;
}

/// @brief The largest magnitude over the settled tail, the transient and the group delay left behind.
[[nodiscard]] double settledPeak(std::span<const CF> y) {
    double peak = 0.0;
    for (std::size_t k = y.size() / 2UZ; k < y.size(); ++k) {
        peak = std::max(peak, static_cast<double>(std::abs(y[k])));
    }
    return peak;
}

/// Where a frequency lands after a decimation by @p d, in cycles per sample of the slower rate.
[[nodiscard]] double foldedBy(double f, std::size_t d) {
    const double scaled = f * static_cast<double>(d);
    return std::abs(scaled - std::round(scaled));
}

/// The charge the design tables are quoted under: half the length plus the center. It is the true live
/// count only where `(N-1)/2` is even, so the two are carried separately and the gap stays visible.
template<typename T>
[[nodiscard]] double tabulatedMacsPerInput(const StagedDecimator<T>& block) {
    double      macs   = 0.0;
    std::size_t stride = 1UZ;
    for (std::size_t i = 0UZ; i < block.stages(); ++i) {
        const std::size_t d      = block.stageDecimation(i);
        const std::size_t n      = block.stageTaps(i).size();
        const std::size_t charge = d == 2UZ ? n / 2UZ + 1UZ : n;
        macs += static_cast<double>(charge) / static_cast<double>(stride * d);
        stride *= d;
    }
    return macs;
}

template<typename T>
[[nodiscard]] std::vector<std::size_t> ladderOf(const StagedDecimator<T>& block) {
    std::vector<std::size_t> out;
    for (std::size_t i = 0UZ; i < block.stages(); ++i) {
        out.push_back(block.stageDecimation(i));
    }
    return out;
}

/// A key per tag, so a tag that survives is visible even where several of them share an output offset.
[[nodiscard]] gr::property_map tagKey(std::size_t which) {
    gr::property_map map;
    map.insert_or_assign(gr::property_map::key_type{std::format("tag{}", which)}, static_cast<gr::Size_t>(which));
    return map;
}

[[nodiscard]] std::size_t countOwnKeys(const gr::Tag& tag) {
    return static_cast<std::size_t>(std::ranges::count_if(tag.map, [](const auto& entry) { return std::string_view(entry.first).starts_with("tag"); }));
}

[[nodiscard]] std::string join(const std::vector<std::size_t>& values) {
    std::string out;
    for (const std::size_t v : values) {
        out += std::format("{}{}", out.empty() ? "" : ", ", v);
    }
    return out;
}

/**
 * @brief The delay of the running path in input samples, read off the block's ladder: `((N_i - 1)/2) * p_{i-1}` summed over
 * the stages, less one sample of each halving stage's input rate over complex samples, where the halfband cascade
 * samples a halving's output early.
 */
template<typename T>
[[nodiscard]] std::uint64_t pathDelay(const StagedDecimator<T>& block) {
    std::uint64_t delay  = 0ULL;
    std::uint64_t stride = 1ULL;
    for (std::size_t s = 0UZ; s < block.stages(); ++s) {
        delay += ((block.stageTaps(s).size() - 1ULL) / 2ULL) * stride;
        if constexpr (std::same_as<T, CF>) {
            if (block.stageDecimation(s) == 2UZ) {
                delay -= stride;
            }
        }
        stride *= block.stageDecimation(s);
    }
    return delay;
}

/// @brief The index of the sample of largest magnitude, the first of several equal ones.
template<typename T>
[[nodiscard]] std::size_t peakIndex(std::span<const T> y) {
    std::size_t at = 0UZ;
    for (std::size_t k = 1UZ; k < y.size(); ++k) {
        if (std::abs(y[k]) > std::abs(y[at])) {
            at = k;
        }
    }
    return at;
}

/// The tables are quoted to two decimal places, so a value ending in `.xx5` sits exactly on the rounding
/// boundary and half a quantum is not enough slack.
constexpr double kTabulated = 0.0051;

[[nodiscard]] bool longRun() { return std::getenv("ENABLE_LONG_TESTS") != nullptr; }

} // namespace

const boost::ut::suite<"staged decimator"> stagedDecimatorTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::testing;
    using gr::filter::mapResampledOffset;

    "n*D samples in yields exactly n out, from the first call"_test = [] {
        for (const gr::Size_t d : {1U, 2U, 3U, 8U, 16U, 25U, 32U, 64U}) {
            StagedDecimator<CF>   block = makeBlock<CF>({{"decimation", d}});
            const std::size_t     n     = 1000UZ;
            const std::vector<CF> x     = noise<CF>(n * d, 0xBF58476D1CE4E5B9ULL);

            for (const std::size_t chunkOutputs : {1UZ, 7UZ, 0UZ}) {
                const auto y = run(block, x, chunkOutputs);
                expect(eq(y.samples.size(), n)) << "D = " << d << " in calls of " << chunkOutputs << " outputs";
                expect(eq(y.consumed, n * d));
            }
        }
    };

    "a decimation of one is a bit-exact pass-through, tags included"_test = [] {
        StagedDecimator<CF> block = makeBlock<CF>({{"decimation", 1U}});
        expect(eq(block.stages(), 0UZ)) << "no taps are designed";
        expect(eq(block.groupDelaySamples(), 0ULL));
        expect(eq(block.macsPerInput(), 0.0));

        const std::vector<CF>      x = noise<CF>(500UZ, 0x3C79AC492BA7B653ULL);
        const std::vector<gr::Tag> tags{gr::Tag{7UZ, tagKey(0)}, gr::Tag{300UZ, tagKey(1)}};
        const auto                 y = run(block, x, 100UZ, std::span<const gr::Tag>(tags));

        expect(that % (y.samples == x)) << "bit for bit";
        expect(that % (y.offsetsOf("tag0") == std::vector<std::size_t>{7UZ}));
        expect(that % (y.offsetsOf("tag1") == std::vector<std::size_t>{300UZ}));
    };

    "the block's ladder is halvings first, then the odd factors largest first"_test = [] {
        const auto ladder = [](gr::Size_t d, bool factorOdd) { return ladderOf(makeBlock<CF>({{"decimation", d}, {"factor_odd", factorOdd}})); };

        expect(that % (ladder(1U, true) == std::vector<std::size_t>{}));
        expect(that % (ladder(2U, true) == std::vector<std::size_t>{2UZ}));
        expect(that % (ladder(8U, true) == std::vector<std::size_t>{2UZ, 2UZ, 2UZ}));
        expect(that % (ladder(25U, true) == std::vector<std::size_t>{5UZ, 5UZ}));
        expect(that % (ladder(25U, false) == std::vector<std::size_t>{25UZ}));
        expect(that % (ladder(50U, true) == std::vector<std::size_t>{2UZ, 5UZ, 5UZ}));
        expect(that % (ladder(50U, false) == std::vector<std::size_t>{2UZ, 25UZ}));
        expect(that % (ladder(64U, true) == std::vector<std::size_t>{2UZ, 2UZ, 2UZ, 2UZ, 2UZ, 2UZ}));
    };

    "the acceptance table, read off the block"_test = [] {
        struct Row {
            gr::Size_t decimation;
            double     worstStopbandDb;
            double     rippleSumDb;
        };
        // The worst stage is the least attenuated one, which is the one a fold path can cross, and in every
        // ladder ending in a halving stage that is the last: the narrowest transition, designed closest to
        // its target. D = 25 as one stage of 25 is its own worst stage.
        constexpr Row kRows[] = {{8U, -85.39, 0.0019}, {16U, -85.39, 0.0021}, {25U, -85.09, 0.0009}, {32U, -85.39, 0.0023}, {64U, -85.39, 0.0023}, {128U, -85.39, 0.0024}};

        for (const Row& row : kRows) {
            if (row.decimation > 64U && !longRun()) {
                continue; // D = 128 is seven searches
            }
            StagedDecimator<CF> block = makeBlock<CF>({{"decimation", row.decimation}, {"passband_width", 0.90f}, {"factor_odd", false}});
            expect(approx(block.worstStopbandDb(), row.worstStopbandDb, 0.5)) << "D = " << row.decimation << " worst stage";
            expect(approx(block.rippleSumDb(), row.rippleSumDb, 0.001)) << "D = " << row.decimation << " ripple sum";
            expect(block.worstStopbandDb() <= -85.0) << "D = " << row.decimation << " clears the 85 dB target";
        }
    };

    "the MAC tables, read off the block"_test = [] {
        struct Row {
            gr::Size_t decimation;
            bool       factorOdd;
            double     tabulated90;
            double     true90;
            double     tabulated80;
            double     true80;
        };
        // `macsPerInput()` reports what the cascade pays; the tabulated charge recomputed from the same tap
        // counts is what the design tables quote, and the two differ wherever a halfband's length is 3 mod 4.
        constexpr Row kRows[] = {
            {8U, true, 15.38, 16.125, 11.88, 12.625},    //
            {25U, false, 53.88, 53.88, 27.88, 27.88},    //
            {25U, true, 17.84, 17.84, 12.64, 12.64},     //
            {64U, true, 8.42, 8.640625, 7.98, 8.203125}, //
        };

        for (const Row& row : kRows) {
            StagedDecimator<CF> wide   = makeBlock<CF>({{"decimation", row.decimation}, {"passband_width", 0.90f}, {"factor_odd", row.factorOdd}});
            StagedDecimator<CF> narrow = makeBlock<CF>({{"decimation", row.decimation}, {"passband_width", 0.80f}, {"factor_odd", row.factorOdd}});

            expect(approx(tabulatedMacsPerInput(wide), row.tabulated90, kTabulated)) << "D = " << row.decimation << " at 0.90, tabulated charge";
            expect(approx(wide.macsPerInput(), row.true90, 1e-6)) << "D = " << row.decimation << " at 0.90, what the kernel pays";
            expect(approx(tabulatedMacsPerInput(narrow), row.tabulated80, kTabulated)) << "D = " << row.decimation << " at 0.80, tabulated charge";
            expect(approx(narrow.macsPerInput(), row.true80, 1e-6)) << "D = " << row.decimation << " at 0.80, what the kernel pays";
        }

        // The row that keeps the value claim honest: one polyphase stage of 25 reclaims exactly nothing,
        // because the ladder degenerates to the single filter it is compared with.
        StagedDecimator<CF> one      = makeBlock<CF>({{"decimation", 25U}, {"passband_width", 0.90f}, {"factor_odd", false}});
        StagedDecimator<CF> factored = makeBlock<CF>({{"decimation", 25U}, {"passband_width", 0.90f}, {"factor_odd", true}});
        expect(approx(one.macsPerInput() / factored.macsPerInput(), 3.02, 0.01)) << "factoring 25 into 5 and 5 is worth 3.02x";
    };

    "a halving stage's even-offset taps are zeros and its live count says so"_test = [] {
        StagedDecimator<CF> block = makeBlock<CF>({{"decimation", 64U}, {"passband_width", 0.90f}});
        expect(eq(block.stages(), 6UZ));

        for (std::size_t s = 0UZ; s < block.stages(); ++s) {
            const std::span<const float> taps = block.stageTaps(s);
            expect(eq(block.stageDecimation(s), 2UZ));

            const auto  n    = static_cast<int>(taps.size());
            const int   mid  = (n - 1) / 2;
            double      peak = 0.0;
            double      zero = 0.0;
            std::size_t live = 0UZ;
            for (int i = 0; i < n; ++i) {
                const double v      = std::abs(static_cast<double>(taps[static_cast<std::size_t>(i)]));
                const int    offset = i - mid;
                peak                = std::max(peak, v);
                if (offset != 0 && (offset % 2) == 0) {
                    zero = std::max(zero, v);
                } else {
                    ++live;
                }
            }

            // Not zero to the bit: the tap is `sin` of a rounded multiple of pi, 322 to 332 dB under the
            // peak, which is why the kernel skips it by index rather than by testing against zero.
            expect(20.0 * std::log10(std::max(zero, 1e-300) / peak) < -300.0) << n << " taps";
            expect(eq(live, 2UZ * ((static_cast<std::size_t>(mid) + 1UZ) / 2UZ) + 1UZ)) << n << " taps: the live count is 2*ceil((N-1)/4)+1";

            // Where `(N-1)/2` is odd the outermost pair sits at an odd offset from the center and lives, so
            // the tabulated `N/2 + 1` charge is one multiply short. At N of 19 and 23 it reads 10 and 12
            // against a true 11 and 13, which is the whole of this ladder's 8.42 against 8.64.
            const std::size_t tabled = static_cast<std::size_t>(n) / 2UZ + 1UZ;
            expect(eq(live, (mid % 2 == 0) ? tabled : tabled + 1UZ)) << n << " taps against the tabulated charge";
        }
    };

    "bit-identical output whatever the call size"_test = []<typename T>() {
        for (const gr::Size_t d : {8U, 25U, 64U}) {
            StagedDecimator<T>   block = makeBlock<T>({{"decimation", d}});
            const std::size_t    n     = 2048UZ;
            const std::vector<T> x     = noise<T>(n * d, 0xD6E8FEB86659FD93ULL);

            const auto whole = run(block, x);
            expect(eq(whole.samples.size(), n));

            for (const std::size_t chunkOutputs : {1UZ, 3UZ, 17UZ, static_cast<std::size_t>(d), static_cast<std::size_t>(d) + 1UZ, 512UZ}) {
                StagedDecimator<T> fresh = makeBlock<T>({{"decimation", d}});
                const auto         part  = run(fresh, x, chunkOutputs);

                expect(eq(part.samples.size(), n)) << "D = " << d << " in calls of " << chunkOutputs << " outputs";
                expect(std::ranges::equal(part.samples, whole.samples)) << "D = " << d << " in calls of " << chunkOutputs << " outputs, bit for bit";
            }
        }
    } | std::tuple<CF, float>{};

    "a frequency that folds into the kept band is stopped, and one inside it is not"_test = [] {
        constexpr gr::Size_t kD     = 8U;
        constexpr double     kWidth = 0.90;
        const double         half   = 0.5 * kWidth / static_cast<double>(kD); // W/2 at the input rate

        StagedDecimator<CF> block = makeBlock<CF>({{"decimation", kD}, {"passband_width", static_cast<float>(kWidth)}});

        for (const double f : {0.005, 0.02, 0.05}) {
            const auto y = run(block, tone(1UZ << 15, f));
            expect(approx(settledPeak(std::span<const CF>(y.samples)), 1.0, 0.01)) << "a tone at " << f << " is inside the kept band and comes through at unit gain";
        }

        // Every frequency here is above the kept band and folds back inside it, so it is exactly what the
        // acceptance rule bounds: one stopband crossed and several passbands.
        for (const double f : {0.13, 0.24, 0.26, 0.38, 0.49}) {
            expect(f > half) << "the probe frequency is outside the kept band to begin with";
            expect(foldedBy(f, kD) <= half * static_cast<double>(kD)) << "and folds back inside it";
            const auto   y  = run(block, tone(1UZ << 15, f));
            const double db = 20.0 * std::log10(std::max(settledPeak(std::span<const CF>(y.samples)), 1e-300));
            expect(db <= -84.0) << std::format("a tone at {} folds in at {:.2f} dB", f, db);
        }
    };

    "a decimation of two passes and stops the same way"_test = [] {
        // The single-stage shape: the whole ladder is the halfband cascade with one element and no
        // polyphase tail, which no other decimation reaches with a signal-level assertion.
        constexpr gr::Size_t kD     = 2U;
        constexpr double     kWidth = 0.90;
        const double         half   = 0.5 * kWidth / static_cast<double>(kD);

        StagedDecimator<CF> block = makeBlock<CF>({{"decimation", kD}, {"passband_width", static_cast<float>(kWidth)}});
        expect(eq(block.stages(), 1UZ));

        for (const double f : {0.005, 0.05, 0.20}) {
            const auto y = run(block, tone(1UZ << 15, f));
            expect(approx(settledPeak(std::span<const CF>(y.samples)), 1.0, 0.01)) << "a tone at " << f << " is inside the kept band and comes through at unit gain";
        }

        for (const double f : {0.30, 0.38, 0.45, 0.49}) {
            expect(f > half) << "the probe frequency is outside the kept band to begin with";
            expect(foldedBy(f, kD) <= half * static_cast<double>(kD)) << "and folds back inside it";
            const auto   y  = run(block, tone(1UZ << 15, f));
            const double db = 20.0 * std::log10(std::max(settledPeak(std::span<const CF>(y.samples)), 1e-300));
            expect(db <= -84.0) << std::format("a tone at {} folds in at {:.2f} dB", f, db);
        }
    };

    "the group delay is the stated sum, and the cascade sits one sample per halving ahead of it"_test = [] {
        // `HalfbandCascade` reads its taps forward over a window primed `N-2` deep, so its output `k` is the
        // filtered stream sampled at `2k+1` rather than at `2k` and each halving stage lands one sample of
        // its own rate early. Summed over the ladder that is `sum of the halving strides` input samples,
        // 63 of 2158 at `D = 64`: a fixed offset, aliasing-identical, and not a drift.
        for (const gr::Size_t d : {8U, 64U}) {
            StagedDecimator<CF> block = makeBlock<CF>({{"decimation", d}, {"passband_width", 0.90f}});

            std::uint64_t stated = 0ULL;
            std::uint64_t stride = 1ULL;
            std::uint64_t ahead  = 0ULL;
            for (std::size_t s = 0UZ; s < block.stages(); ++s) {
                stated += ((block.stageTaps(s).size() - 1ULL) / 2ULL) * stride;
                ahead += stride;
                stride *= block.stageDecimation(s);
            }
            expect(eq(block.groupDelaySamples(), stated)) << "D = " << d;
            expect(eq(ahead, static_cast<std::uint64_t>(d) - 1ULL)) << "D = " << d << ": every stage is a halving, so the strides sum to D - 1";

            std::vector<CF> x(1UZ << 16, CF{});
            x[0]         = CF{1.0f, 0.0f};
            const auto y = run(block, x);

            double energy = 0.0;
            double moment = 0.0;
            for (std::size_t k = 0UZ; k < y.samples.size(); ++k) {
                const double e = static_cast<double>(std::norm(y.samples[k]));
                energy += e;
                moment += e * static_cast<double>(k);
            }
            const double centroid  = moment / energy;
            const double statedOut = static_cast<double>(stated) / static_cast<double>(d);
            const double aheadOut  = static_cast<double>(stated - ahead) / static_cast<double>(d);

            expect(lt(std::abs(centroid - statedOut), 1.0)) << std::format("D = {}: the impulse centroid is {:.3f} output samples against a stated {:.3f}", d, centroid, statedOut);
            expect(lt(std::abs(centroid - aheadOut), std::abs(centroid - statedOut))) << std::format("D = {}: and sits nearer {:.3f}, the sum less the halving strides", d, aheadOut);
        }

        expect(eq(makeBlock<CF>({{"decimation", 64U}, {"passband_width", 0.90f}}).groupDelaySamples(), 2158ULL)) << "6*1 + 6*2 + 9*4 + 9*8 + 11*16 + 58*32";
    };

    "tags map from the total decimation, not stage by stage"_test = [] {
        constexpr std::size_t kD = 10UZ;

        // the FIR filter's worked tag-map row, which this block must reproduce because its map is the total one
        constexpr std::uint64_t kWorked[] = {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2};
        for (std::size_t i = 0UZ; i < std::size(kWorked); ++i) {
            expect(eq(mapResampledOffset(i, 1ULL, kD), kWorked[i])) << "input " << i;
        }

        // Composing floor(i/2 + 1/2) through the stages rounds once per stage and is a different function:
        // at D = 8 input 3 maps to 0 by the total rule and to 1 by the composed one.
        std::uint64_t composed = 3ULL;
        for (int stage = 0; stage < 3; ++stage) {
            composed = mapResampledOffset(composed, 1ULL, 2ULL);
        }
        expect(eq(mapResampledOffset(3ULL, 1ULL, 8ULL), 0ULL));
        expect(eq(composed, 1ULL)) << "which is why the block maps once from D and never through the stages";

        expect(eq(mapResampledOffset((1ULL << 24) + 1ULL, 1ULL, 1ULL), (1ULL << 24) + 1ULL));
        expect(eq(mapResampledOffset((1ULL << 53) + 1ULL, 1ULL, 1ULL), (1ULL << 53) + 1ULL));
        expect(eq(mapResampledOffset((1ULL << 53) + 1ULL, 1ULL, 2ULL), (1ULL << 52) + 1ULL));

        StagedDecimator<CF>  block = makeBlock<CF>({{"decimation", static_cast<gr::Size_t>(kD)}});
        std::vector<gr::Tag> tags;
        for (std::size_t i = 0UZ; i < kD; ++i) { // D tags on consecutive inputs, all of which belong at the offsets the map gives
            tags.emplace_back(i, tagKey(i));
        }
        const auto          y     = run(block, noise<CF>(200UZ * kD, 0xA24BAED4963EE407ULL), 4UZ, std::span<const gr::Tag>(tags));
        const std::uint64_t delay = pathDelay(block);

        expect(that % (y.offsetsOf("tag0") == std::vector<std::size_t>{narrowIndex<std::size_t>(mapResampledOffset(delay, 1ULL, kD))})) << "a tag at input 0 lands on the output of the delay alone";
        for (std::size_t i = 0UZ; i < kD; ++i) {
            expect(that % (y.offsetsOf(std::format("tag{}", i)) == std::vector<std::size_t>{narrowIndex<std::size_t>(mapResampledOffset(i + delay, 1ULL, kD))})) << "tag " << i;
        }
    };

    "a tag that states a property of the stream crosses unmoved, and a trigger moves by the delay"_test = [] {
        // the rate describes output 0 as it describes input 0; the trigger leaves on the output of the path's delay alone
        for (const gr::Size_t decimation : {8U, 10U}) {
            StagedDecimator<float> block = makeBlock<float>({{"decimation", decimation}});
            const std::uint64_t    delay = pathDelay(block);
            gr::property_map       opening; // a source's opening tag: the stream's rate and the trigger of its first sample
            opening.insert_or_assign(gr::property_map::key_type{"sample_rate"}, 48000.0f);
            opening.insert_or_assign(gr::property_map::key_type{"trigger_time"}, std::uint64_t{1000});
            const std::vector<gr::Tag> tags{gr::Tag{0UZ, opening}};
            const std::vector<float>   x(narrowIndex<std::size_t>(delay + 40ULL * decimation), 0.0f);
            const auto                 y = run(block, x, 4UZ, std::span<const gr::Tag>(tags));

            expect(that % (y.offsetsOf("sample_rate") == std::vector<std::size_t>{0UZ})) << std::format("D = {}: the rate stays on output 0", decimation);
            expect(that % (y.offsetsOf("trigger_time") == std::vector<std::size_t>{narrowIndex<std::size_t>(mapResampledOffset(delay, 1ULL, decimation))})) << std::format("D = {}: the trigger moves by the delay", decimation);
        }
    };

    "a tag leaves on the output that carries its sample's energy"_test = []<typename T>() {
        // Input i lands on output (i + d) / D, d being the path's delay. Each input below is the one in the second call's
        // last output group whose delayed position is a whole output, so the output it lands on belongs to a later call.
        constexpr std::size_t kChunkOutputs = 4UZ;
        for (const gr::Size_t decimation : {8U, 10U}) {
            const std::size_t   d     = static_cast<std::size_t>(decimation);
            StagedDecimator<T>  block = makeBlock<T>({{"decimation", decimation}});
            const std::uint64_t delay = pathDelay(block);

            const std::size_t callEnd = 2UZ * kChunkOutputs * d;
            std::size_t       at      = callEnd - d;
            while ((at + delay) % d != 0UZ) {
                ++at;
            }
            const std::size_t want = narrowIndex<std::size_t>((at + delay) / d);

            std::vector<T> x(narrowIndex<std::size_t>(callEnd + delay + 20UZ * d), T{});
            x[at] = T{1.0f};
            const std::vector<gr::Tag> tags{gr::Tag{at, tagKey(0)}};
            const auto                 y = run(block, x, kChunkOutputs, std::span<const gr::Tag>(tags));

            expect(ge(want, 2UZ * kChunkOutputs)) << "the delayed output belongs to a later call than the tag's input";
            expect(that % (y.offsetsOf("tag0") == std::vector<std::size_t>{want})) << std::format("D = {}: input {} leaves on output {} and on no other", d, at, want);
            expect(eq(peakIndex(std::span<const T>(y.samples)), want)) << std::format("D = {}: the impulse at input {} peaks on output {}", d, at, want);
        }
    } | std::tuple<float, CF>{};

    "a tag whose output falls past the current call is published when that output is produced"_test = [] {
        constexpr std::size_t kD    = 10UZ;
        StagedDecimator<CF>   block = makeBlock<CF>({{"decimation", static_cast<gr::Size_t>(kD)}});

        // input 995 maps past output 99, the last output of the call that carries it
        const std::vector<gr::Tag> tags{gr::Tag{995UZ, tagKey(0)}};
        const auto                 y    = run(block, noise<CF>(1010UZ * kD, 0x9E3779B97F4A7C15ULL), 1UZ, std::span<const gr::Tag>(tags));
        const std::size_t          want = narrowIndex<std::size_t>(mapResampledOffset(995ULL + pathDelay(block), 1ULL, kD));
        expect(gt(want, 99UZ));
        expect(that % (y.offsetsOf("tag0") == std::vector<std::size_t>{want})) << "held, then published at the offset the map gave it";
    };

    "tags land where the decimation puts them, through a graph"_test = [] {
        constexpr std::size_t kTags = 22UZ;
        constexpr std::size_t kD    = 10UZ;
        gr::Graph             graph;

        auto& source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 4000U}, {"mark_tag", false}});
        for (std::size_t i = 0UZ; i < kTags; ++i) {
            source._tags.emplace_back(i, tagKey(i));
        }
        auto& decimator = graph.emplaceBlock<StagedDecimator<float>>({{"decimation", static_cast<gr::Size_t>(kD)}});
        auto& sink      = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

        expect(graph.connect<"out", "in">(source, decimator).has_value());
        expect(graph.connect<"out", "in">(decimator, sink).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        const std::uint64_t      delay = pathDelay(decimator);
        std::vector<std::size_t> want;
        for (std::size_t i = 0UZ; i < kTags; ++i) {
            const std::size_t at = narrowIndex<std::size_t>(mapResampledOffset(i + delay, 1ULL, kD));
            if (want.empty() || want.back() != at) {
                want.push_back(at);
            }
        }

        std::vector<std::size_t> got;
        std::size_t              keysSeen = 0UZ;
        for (const gr::Tag& published : sink._tags) {
            const std::size_t mine = countOwnKeys(published);
            if (mine > 0UZ) {
                got.push_back(published.index);
                keysSeen += mine;
            }
        }
        expect(that % (got == want)) << std::format("output offsets [{}] against [{}]", join(got), join(want));
        expect(eq(keysSeen, kTags)) << "every tag survives, in input order, none merged away";
    };

    "a forwarded sample_rate is divided by the total decimation"_test = [] {
        constexpr std::size_t kD       = 10UZ;
        constexpr float       kRateIn  = 480000.f;
        constexpr float       kRateOut = kRateIn / static_cast<float>(kD);
        gr::Graph             graph;

        auto& source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 4000U}, {"mark_tag", false}});
        source._tags.emplace_back(200UZ, gr::property_map{{gr::property_map::key_type{gr::tag::SAMPLE_RATE.shortKey()}, gr::pmt::Value(kRateIn)}});

        auto& decimator = graph.emplaceBlock<StagedDecimator<float>>({{"decimation", static_cast<gr::Size_t>(kD)}});
        auto& sink      = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

        expect(graph.connect<"out", "in">(source, decimator).has_value());
        expect(graph.connect<"out", "in">(decimator, sink).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        std::vector<float> rates;
        for (const gr::Tag& published : sink._tags) {
            if (published.index == 0UZ) {
                continue; // the source announces its own rate at offset 0
            }
            if (const auto found = published.map.find(gr::property_map::key_type{gr::tag::SAMPLE_RATE.shortKey()}); found != published.map.end()) {
                rates.push_back(found->second.value_or(0.f));
            }
        }
        expect(that % (rates == std::vector<float>{kRateOut})) << "downstream reads the rate of the stream it is handed, not the one this block was fed";
    };

    "a decimation change rebuilds the ladder and re-origins the tag map"_test = [] {
        StagedDecimator<CF>   block = makeBlock<CF>({{"decimation", 8U}});
        const std::vector<CF> x     = noise<CF>(1600UZ, 0x8A5CD789635D2DFFULL);

        const std::vector<gr::Tag> early{gr::Tag{40UZ, tagKey(0)}};
        const auto                 head = run(block, x, 25UZ, std::span<const gr::Tag>(early));
        expect(eq(head.samples.size(), 200UZ));
        expect(that % (head.offsetsOf("tag0") == std::vector<std::size_t>{narrowIndex<std::size_t>(mapResampledOffset(40ULL + pathDelay(block), 1ULL, 8ULL))}));
        expect(eq(block.stages(), 3UZ));
        const std::uint64_t before = block.groupDelaySamples();

        std::ignore = block.settings().setStaged({{"decimation", 16U}});
        std::ignore = block.settings().applyStagedParameters();
        expect(eq(block.stages(), 4UZ)) << "the ladder is rebuilt, not adjusted";
        expect(block.groupDelaySamples() != before) << "and the delay changes with it";

        // The new origin is input 1600 / output 200, so input 1632 is two outputs past it before the delay.
        const std::vector<gr::Tag> late{gr::Tag{1632UZ, tagKey(1)}};
        const auto                 tail = test::runDecimating<CF>(block, std::span<const CF>(x), 16UZ * 25UZ, 16UZ, std::span<const gr::Tag>(late), 1600UZ, 200UZ);
        expect(eq(tail.samples.size(), 100UZ)) << "1600 further inputs at D = 16, from the first call after the rebuild";
        expect(that % (tail.offsetsOf("tag1") == std::vector<std::size_t>{200UZ + narrowIndex<std::size_t>(mapResampledOffset(32ULL + pathDelay(block), 1ULL, 16ULL))})) << "mapped from the new origin, not rescaled from zero";
    };

    "a rebuild at the same rate keeps the tag in flight"_test = [] {
        constexpr std::size_t kD    = 10UZ;
        StagedDecimator<CF>   block = makeBlock<CF>({{"decimation", static_cast<gr::Size_t>(kD)}});
        const std::vector<CF> x     = noise<CF>(2000UZ, 0x5D2DFF8A5CD78963ULL);

        // input 995 maps past output 99, the last output of the run that carries it
        const std::vector<gr::Tag> tags{gr::Tag{995UZ, tagKey(0)}};
        const auto                 head   = test::runDecimating<CF>(block, std::span<const CF>(x).first(1000UZ), kD, kD, std::span<const gr::Tag>(tags));
        const std::size_t          placed = narrowIndex<std::size_t>(mapResampledOffset(995ULL + pathDelay(block), 1ULL, kD));
        expect(eq(head.samples.size(), 100UZ));
        expect(that % (head.offsetsOf("tag0") == std::vector<std::size_t>{})) << "its output is not produced yet, so it is held";

        const std::size_t stages = block.stages();
        std::ignore              = block.settings().setStaged({{"ripple_db", 0.04f}}); // a rebuild key that leaves the rate, and so the tag map's origin, alone
        std::ignore              = block.settings().applyStagedParameters();
        expect(eq(block.stages(), stages));

        const auto tail = test::runDecimating<CF>(block, std::span<const CF>(x).subspan(1000UZ), kD, kD, {}, 1000UZ, 100UZ);
        expect(eq(tail.samples.size(), 100UZ));
        expect(that % (tail.offsetsOf("tag0") == std::vector<std::size_t>{placed})) << "a design change that moves no tag keeps the held one, at the offset it already had";
    };

    "a rebuild to a ladder already built designs nothing new"_test = [] {
        StagedDecimator<CF>             block = makeBlock<CF>({{"decimation", 16U}, {"passband_width", 0.90f}});
        std::vector<std::vector<float>> first;
        for (std::size_t s = 0UZ; s < block.stages(); ++s) {
            first.emplace_back(block.stageTaps(s).begin(), block.stageTaps(s).end());
        }

        for (const gr::Size_t d : {8U, 16U}) {
            std::ignore = block.settings().setStaged({{"decimation", d}});
            std::ignore = block.settings().applyStagedParameters();
        }

        expect(eq(block.stages(), first.size()));
        for (std::size_t s = 0UZ; s < block.stages(); ++s) {
            expect(that % (std::vector<float>(block.stageTaps(s).begin(), block.stageTaps(s).end()) == first[s])) << "stage " << s << " is the same tap set, to the bit";
        }
    };

    "degenerate parameters"_test = [] {
        expect(throws([] { std::ignore = makeBlock<CF>({{"decimation", 0U}}); }));
        expect(throws([] { std::ignore = makeBlock<CF>({{"decimation", 8U}, {"passband_width", 0.0f}}); }));
        expect(throws([] { std::ignore = makeBlock<CF>({{"decimation", 8U}, {"passband_width", 1.0f}}); }));
        expect(throws([] { std::ignore = makeBlock<CF>({{"decimation", 8U}, {"passband_width", -0.5f}}); }));

        StagedDecimator<CF> live = makeBlock<CF>({{"decimation", 8U}});
        expect(throws([&live] {
            std::ignore = live.settings().setStaged({{"decimation", 0U}});
            std::ignore = live.settings().applyStagedParameters();
        })) << "and on a live change";

        // An odd prime above the bound warns and works: refusing a legal decimation is worse than a long
        // last stage, and factoring cannot help a prime.
        StagedDecimator<CF>   oversized = makeBlock<CF>({{"decimation", 29U}, {"max_odd_factor", 25U}});
        const std::vector<CF> x         = noise<CF>(29UZ * 100UZ, 0x1D8E4E27C47D124FULL);
        expect(eq(oversized.stages(), 1UZ));
        expect(eq(run(oversized, x).samples.size(), 100UZ));
    };

    "nanoseconds per input sample against the single-stage filter"_test = [] {
        if (std::getenv("ENABLE_BENCHMARK_TESTS") == nullptr) {
            return; // opt-in: a timing assertion belongs to a controlled run, not to every ctest invocation
        }
        using Clock = std::chrono::steady_clock;

        const std::vector<CF> x = noise<CF>(1UZ << 17, 0x1D8E4E27C47D124FULL);

        struct Reading {
            double best   = 0.0;
            double spread = 0.0;
        };

        // The two arms run alternately inside one loop, so a frequency excursion or a migration lands on
        // both rather than on whichever was measured while it happened.
        const auto interleaved = []<typename TA, typename TB>(TA& a, TB& b, const std::vector<CF>& input, std::size_t d) {
            const std::span<const CF> whole(input.data(), (input.size() / d) * d); // whole calls, as the framework hands them
            const auto                n = static_cast<double>(whole.size());
            std::vector<CF>           ya(whole.size() / d);
            std::vector<CF>           yb(whole.size() / d);
            std::vector<double>       ta;
            std::vector<double>       tb;
            for (int repeat = 0; repeat < 8; ++repeat) {
                const auto t0 = Clock::now();
                std::ignore   = a.processBulk(whole, std::span<CF>(ya));
                const auto t1 = Clock::now();
                std::ignore   = b.processBulk(whole, std::span<CF>(yb));
                const auto t2 = Clock::now();
                if (repeat > 1) { // the first passes warm the stage buffers, which are sized on demand
                    ta.push_back(static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count()) / n);
                    tb.push_back(static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count()) / n);
                }
            }
            const auto read = [](std::vector<double>& v) {
                std::ranges::sort(v);
                return Reading{v.front(), (v.back() - v.front()) / v.front()};
            };
            return std::pair{read(ta), read(tb)};
        };

        struct Arm {
            gr::Size_t decimation;
            bool       factorOdd;
            double     tabulatedRatio;
        };
        constexpr Arm kArms[] = {{8U, true, 3.34}, {25U, false, 1.00}, {25U, true, 3.02}, {64U, true, 6.20}};

        for (const Arm& arm : kArms) {
            const std::size_t   d      = arm.decimation;
            StagedDecimator<CF> staged = makeBlock<CF>({{"decimation", arm.decimation}, {"passband_width", 0.90f}, {"factor_odd", arm.factorOdd}});

            const std::size_t                       single = arm.decimation;
            const gr::filter::StagedDecimatorDesign one    = gr::filter::designDecimatorLadder(std::span<const std::size_t>(&single, 1UZ), d, 0.90);
            FirFilter<CF, float>                    flat(gr::property_map{{"taps", one.stages[0UZ].taps}, {"decimation", arm.decimation}});
            flat.settings().init();
            std::ignore = flat.settings().applyStagedParameters();
            flat.start();

            const auto [ladder, plain] = interleaved(staged, flat, x, d);
            std::println("StagedDecimator D={:3} factor_odd={:5}: staged {:6.3f} ns/in (+{:.1f} %, {:6.1f} MS/s, {} stages, {:6.3f} MAC/in true, {:6.3f} tabled, {:.3f} ns/MAC), single {:7.3f} ns/in (+{:.1f} %, {} taps, {:5.2f} MAC/in, {:.3f} ns/MAC), {:.2f}x measured against {:.2f}x tabulated", //
                arm.decimation, arm.factorOdd, ladder.best, 100.0 * ladder.spread, 1000.0 / ladder.best, staged.stages(), staged.macsPerInput(), tabulatedMacsPerInput(staged), ladder.best / staged.macsPerInput(), plain.best, 100.0 * plain.spread, one.stages[0UZ].taps.size(), one.macsPerInput, plain.best / one.macsPerInput, plain.best / ladder.best, arm.tabulatedRatio);

            // The charge the ladder pays is never below the charge the tables quote: a halfband of length
            // 3 mod 4 keeps one tap more than `N/2 + 1`, and no stage keeps fewer.
            expect(ge(staged.macsPerInput(), tabulatedMacsPerInput(staged))) << "D = " << arm.decimation;
            if (arm.tabulatedRatio > 1.01) {
                expect(ge(plain.best / ladder.best, 0.7 * arm.tabulatedRatio)) << "D = " << arm.decimation << ": the staged arm beats the single stage";
            }
        }
    };
};

int main() { /* tests are automatically registered and run */ }
