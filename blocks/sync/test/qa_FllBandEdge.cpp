#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <limits>
#include <numbers>
#include <random>
#include <span>
#include <vector>

#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/sync/FllBandEdge.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

// The tap geometry, the detector gain and the real-tap identity are properties of gr::sync::BandEdgeFilter and are
// pinned by that header's own QA. What is tested here is the block: the first-order gain it designs, the sign its
// settled frequency carries, what the clamp does at the edge of the pull-in range, and that a setter which should be a
// no-op is one.

namespace {

using gr::blocks::sync::FllBandEdge;
namespace test = gr::blocks::testing::span;

using CF = std::complex<float>;
using CD = std::complex<double>;

constexpr double kPi = std::numbers::pi;

template<typename TBlock>
void configure(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
}

FllBandEdge makeFll(gr::property_map settings = {}) {
    FllBandEdge block(std::move(settings));
    configure(block);
    return block;
}

auto driveFll(FllBandEdge& block, std::span<const CF> input, std::size_t chunk = 0UZ, std::array<bool, 3UZ> wired = {true, true, true}, std::span<const gr::Tag> tags = {}) { return test::runPorts<3UZ, CF>(block, input, chunk, wired, tags); }

/// @brief The first-order loop gain: `beta = 2*pi*lambda/Kdet`, `lambda = 4*Bn*T/(1 + 2*Bn*T)`.
[[nodiscard]] double expectedBeta(double noiseBandwidth, double detectorGain) { return 2.0 * kPi * (4.0 * noiseBandwidth / (1.0 + 2.0 * noiseBandwidth)) / detectorGain; }

/// @brief Root-raised-cosine QPSK at `sps` samples per symbol, scaled to a mean power of `power` and offset in frequency.
[[nodiscard]] std::vector<CF> shapedQpsk(std::size_t nSamples, int sps, double rolloff, double power, double offset, std::uint32_t seed) {
    const std::vector<float> taps     = gr::filter::fir::design::rootRaisedCosine(static_cast<std::size_t>(8 * sps + 1), static_cast<double>(sps), rolloff);
    const std::size_t        nSymbols = nSamples / static_cast<std::size_t>(sps) + taps.size() / static_cast<std::size_t>(sps) + 4UZ;

    std::mt19937                       rng(seed);
    std::uniform_int_distribution<int> pick(0, 3);
    std::vector<CD>                    symbols(nSymbols);
    for (CD& symbol : symbols) {
        symbol = std::polar(1.0, 0.25 * kPi + 0.5 * kPi * static_cast<double>(pick(rng)));
    }

    std::vector<CD> shaped(nSamples, CD{});
    for (std::size_t k = 0UZ; k < nSymbols; ++k) {
        const std::size_t base = k * static_cast<std::size_t>(sps);
        for (std::size_t j = 0UZ; j < taps.size(); ++j) {
            if (base + j < nSamples) {
                shaped[base + j] += symbols[k] * static_cast<double>(taps[j]);
            }
        }
    }

    double energy = 0.0;
    for (std::size_t i = nSamples / 4UZ; i < nSamples; ++i) {
        energy += std::norm(shaped[i]);
    }
    const double scale = std::sqrt(power / (energy / static_cast<double>(nSamples - nSamples / 4UZ)));

    std::vector<CF> signal(nSamples);
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        signal[i] = static_cast<CF>(shaped[i] * scale * std::polar(1.0, 2.0 * kPi * offset * static_cast<double>(i)));
    }
    return signal;
}

[[nodiscard]] double meanOfTail(std::span<const float> values, std::size_t count) {
    double sum = 0.0;
    for (std::size_t i = values.size() - count; i < values.size(); ++i) {
        sum += static_cast<double>(values[i]);
    }
    return sum / static_cast<double>(count);
}

} // namespace

const boost::ut::suite<"FllBandEdge"> fllTests = [] {
    using namespace boost::ut;

    "the first-order gain is the exact inversion, tabulated"_test = [] {
        for (const auto& [sps, bandwidth, beta] : std::array<std::tuple<double, double, double>, 3UZ>{{{4.0, 0.01, 0.061600}, {2.0, 0.0628, 0.701109}, {8.0, 0.005, 0.015552}}}) {
            FllBandEdge block = makeFll({{"samples_per_symbol", sps}, {"noise_bandwidth", bandwidth}});
            expect(lt(std::abs(static_cast<double>(block.loop().beta()) - beta), 1e-6)) << "beta at sps=" << sps << " Bn*T=" << bandwidth;
            expect(lt(std::abs(static_cast<double>(block.loop().beta()) - expectedBeta(bandwidth, sps)), 1e-6)) << "and it is 2*pi*lambda/Kdet";
            expect(eq(block.loop().alpha(), 0.f)) << "a first-order loop has no proportional arm";
        }
    };

    "changing the bandwidth recomputes beta and leaves no proportional arm behind"_test = [] {
        FllBandEdge block = makeFll({{"samples_per_symbol", 4.0}, {"noise_bandwidth", 0.01}});
        expect(lt(std::abs(static_cast<double>(block.loop().beta()) - expectedBeta(0.01, 4.0)), 1e-6));

        std::ignore = block.settings().setStaged({{"noise_bandwidth", 0.05}});
        std::ignore = block.settings().applyStagedParameters();
        expect(eq(block.loop().alpha(), 0.f)) << "the bandwidth setter must not resurrect alpha";
        expect(lt(std::abs(static_cast<double>(block.loop().beta()) - expectedBeta(0.05, 4.0)), 1e-6)) << "and must recompute beta from the first-order form";
    };

    "setting samples_per_symbol to the value already in force changes nothing"_test = [] {
        FllBandEdge block = makeFll({{"samples_per_symbol", 4.0}, {"noise_bandwidth", 0.0628}});

        const float              beta  = block.loop().beta();
        const float              limit = block.loop().maxFrequency();
        const std::vector<float> taps  = block.filters().prototype;

        std::ignore = block.settings().setStaged({{"samples_per_symbol", 4.0}});
        std::ignore = block.settings().applyStagedParameters();

        expect(eq(block.loop().beta(), beta)) << "the gain must be bit-identical, not a factor of 2*pi away";
        expect(eq(block.loop().maxFrequency(), limit)) << "and so must the clamp";
        expect(std::ranges::equal(block.filters().prototype, taps)) << "and every tap";
    };

    "the clamp defaults to the discriminant's own pull-in range"_test = [] {
        for (const auto& [sps, rolloff] : std::array<std::pair<double, double>, 3UZ>{{{2.0, 0.35}, {4.0, 0.22}, {8.0, 0.50}}}) {
            FllBandEdge  block = makeFll({{"samples_per_symbol", sps}, {"rolloff", rolloff}});
            const double limit = kPi * (1.0 + rolloff) / sps;
            expect(lt(std::abs(static_cast<double>(block.loop().maxFrequency()) - limit), 1e-6)) << "upper clamp at sps=" << sps;
            expect(lt(std::abs(static_cast<double>(block.loop().minFrequency()) + limit), 1e-6)) << "lower clamp at sps=" << sps;
        }

        FllBandEdge explicitBounds = makeFll({{"max_frequency", 0.25f}, {"min_frequency", -0.125f}});
        expect(eq(explicitBounds.loop().maxFrequency(), 0.25f)) << "an explicit bound is taken as given";
        expect(eq(explicitBounds.loop().minFrequency(), -0.125f));
    };

    "a symmetric signal at zero offset produces no error"_test = [] {
        constexpr std::size_t kSamples = 20000UZ;
        const std::vector<CF> input    = shapedQpsk(kSamples, 4, 0.35, 1.0, 0.0, 3U);

        // Bounds pinned so tightly that the loop cannot move: what the error port carries is then the open-loop
        // discriminant at zero offset, which is what S(0) means.
        FllBandEdge block   = makeFll({{"samples_per_symbol", 4.0}, {"rolloff", 0.35}, {"filter_length", gr::Size_t(45)}, {"max_frequency", 1e-30f}, {"min_frequency", -1e-30f}});
        const auto  tracked = driveFll(block, std::span<const CF>(input));

        expect(lt(std::abs(meanOfTail(std::span<const float>(tracked.aux[2]), kSamples / 2UZ)), 1e-3)) << "S(0) must be zero";
    };

    "the settled frequency is the estimate of the offset, and it is positive above the carrier"_test = [] {
        constexpr std::size_t kSamples = 20000UZ;
        for (const double offset : {0.005, 0.02, 0.05}) {
            const std::vector<CF> input = shapedQpsk(kSamples, 4, 0.35, 1.0, offset, 4U);

            FllBandEdge block   = makeFll({{"samples_per_symbol", 4.0}, {"rolloff", 0.35}, {"filter_length", gr::Size_t(45)}, {"noise_bandwidth", 0.01}});
            const auto  tracked = driveFll(block, std::span<const CF>(input));

            const double settled  = meanOfTail(std::span<const float>(tracked.aux[0]), 15000UZ);
            const double expected = 2.0 * kPi * offset;

            // The sign is the whole of it: mixing this discriminant's positive Kdet with the other derotation is
            // positive feedback, and this assertion is what stops that combination shipping.
            expect(gt(settled, 0.0)) << "the estimate, not the correction: a signal above the carrier settles positive, got " << settled;

            // The loop is an integrator and has no steady-state error. What is left is data-dependent tracking
            // jitter, which is a constant number of rad/sample and not a fraction of the offset: measured under
            // 1e-3 rad/sample here, 0.1% of the pull-in range and 2.8% of the smallest of these three offsets.
            // Assert the absolute figure, and the relative one where it is tighter.
            expect(lt(std::abs(settled - expected), 2e-3)) << "settled " << settled << " against " << expected;
            if (offset >= 0.02) {
                expect(lt(std::abs(settled - expected) / expected, 0.01)) << "settled " << settled << " against " << expected;
            }
        }
    };

    "the loop pulls in from inside its own range and saturates outside it"_test = [] {
        constexpr std::size_t kSamples = 60000UZ;
        const double          edge     = (1.0 + 0.35) / (2.0 * 4.0); // (1+alpha)/(2*sps), the discriminant's pull-in range

        const std::vector<CF> inside  = shapedQpsk(kSamples, 4, 0.35, 1.0, 0.9 * edge, 6U);
        FllBandEdge           near    = makeFll({{"samples_per_symbol", 4.0}, {"rolloff", 0.35}, {"noise_bandwidth", 0.02}});
        const auto            pulled  = driveFll(near, std::span<const CF>(inside));
        const double          settled = meanOfTail(std::span<const float>(pulled.aux[0]), 8000UZ);
        expect(!near.loop().saturated()) << "an offset inside the range must not park the loop on a bound";
        expect(lt(std::abs(settled - 2.0 * kPi * 0.9 * edge) / (2.0 * kPi * 0.9 * edge), 0.05)) << "settled " << settled << " against " << 2.0 * kPi * 0.9 * edge;

        const std::vector<CF> outside = shapedQpsk(kSamples, 4, 0.35, 1.0, 1.5 * edge, 6U);
        FllBandEdge           far     = makeFll({{"samples_per_symbol", 4.0}, {"rolloff", 0.35}, {"noise_bandwidth", 0.02}});
        const auto            stuck   = driveFll(far, std::span<const CF>(outside));
        expect(far.loop().saturated()) << "an offset outside the range must saturate";
        expect(eq(stuck.aux[0][kSamples - 1UZ], far.loop().maxFrequency())) << "against the upper bound";
    };

    "the normalized discriminant is bounded and needs no gain of its own"_test = [] {
        constexpr std::size_t kSamples = 20000UZ;
        for (const double power : {0.25, 1.0, 4.0}) {
            const std::vector<CF> input = shapedQpsk(kSamples, 4, 0.35, power, 0.02, 8U);

            FllBandEdge block   = makeFll({{"samples_per_symbol", 4.0}, {"rolloff", 0.35}, {"noise_bandwidth", 0.01}, {"normalized_discriminant", true}, {"detector_gain", 46.0456}});
            const auto  tracked = driveFll(block, std::span<const CF>(input));

            for (const float e : tracked.aux[2]) {
                expect(le(std::abs(e), 1.0f)) << "the normalized error is bounded in [-1, 1]";
            }
            const double settled = meanOfTail(std::span<const float>(tracked.aux[0]), 4000UZ);
            expect(gt(settled, 0.0)) << "and it keeps the sign";
            expect(lt(std::abs(settled - 2.0 * kPi * 0.02) / (2.0 * kPi * 0.02), 0.05)) << "at input power " << power << ", settled " << settled;
        }
    };

    "degenerate settings are refused rather than designing something meaningless"_test = [] {
        expect(throws([] { std::ignore = makeFll({{"samples_per_symbol", 0.0}}); })) << "zero samples per symbol";
        expect(throws([] { std::ignore = makeFll({{"rolloff", 1.5}}); })) << "a rolloff outside [0, 1]";
        expect(throws([] { std::ignore = makeFll({{"filter_length", gr::Size_t(2)}}); })) << "two taps";
        expect(throws([] { std::ignore = makeFll({{"detector_gain", -1.0}}); })) << "a negative detector gain";
        expect(throws([] { std::ignore = makeFll({{"noise_bandwidth", 0.0}}); })) << "zero bandwidth";
        expect(throws([] { std::ignore = makeFll({{"min_frequency", 1.f}, {"max_frequency", -1.f}}); })) << "crossed bounds";
    };

    "tags ride through, the two loop tags do not, and a wild payload cannot hang the block"_test = [] {
        constexpr std::size_t      kSamples = 400UZ;
        const std::vector<CF>      input    = shapedQpsk(kSamples, 4, 0.35, 1.0, 0.01, 12U);
        const std::vector<gr::Tag> passenger{gr::Tag{40UZ, gr::property_map{{"passenger", 1.0}}}};

        FllBandEdge block   = makeFll();
        const auto  carried = driveFll(block, std::span<const CF>(input), 0UZ, {true, true, true}, std::span<const gr::Tag>(passenger));
        expect(eq(carried.offsetsOf("passenger").size(), 1UZ));
        expect(eq(carried.offsetsOf("passenger")[0], 40UZ));

        const std::vector<gr::Tag> steering{gr::Tag{10UZ, gr::property_map{{"phase_est", 0.25}, {"freq_est", 0.3}}}};
        FllBandEdge                steered = makeFll();
        const auto                 driven  = driveFll(steered, std::span<const CF>(input), 0UZ, {true, true, true}, std::span<const gr::Tag>(steering));
        expect(eq(driven.tags.size(), 0UZ)) << "the loop tags are consumed";
        expect(approx(driven.aux[1][10UZ], 0.25f, 1e-6f)) << "and are in force on the tagged sample";

        const std::vector<gr::Tag> wild{gr::Tag{5UZ, gr::property_map{{"phase_est", 1e9}}}};
        FllBandEdge                wilded = makeFll();
        const auto                 tamed  = driveFll(wilded, std::span<const CF>(input), 0UZ, {true, true, true}, std::span<const gr::Tag>(wild));
        expect(le(std::abs(tamed.aux[1][5UZ]), static_cast<float>(kPi))) << "a payload of 1e9 is wrapped, not walked to, and leaves a canonical angle";

        const std::vector<gr::Tag> bad{gr::Tag{5UZ, gr::property_map{{"phase_est", std::numeric_limits<double>::quiet_NaN()}, {"freq_est", std::numeric_limits<double>::infinity()}}}};
        FllBandEdge                rejected = makeFll();
        const auto                 guarded  = driveFll(rejected, std::span<const CF>(input), 0UZ, {true, true, true}, std::span<const gr::Tag>(bad));
        FllBandEdge                plain    = makeFll();
        const auto                 untagged = driveFll(plain, std::span<const CF>(input), 0UZ, {true, true, true});
        expect(eq(rejected.ignoredTagPayloads(), 2ULL));
        expect(std::ranges::equal(guarded.samples, untagged.samples));
    };

    "the output does not depend on chunking, the tap count, or which side ports are wired"_test = [] {
        const std::vector<CF> input = shapedQpsk(4000UZ, 4, 0.35, 1.0, 0.02, 21U);
        for (const gr::Size_t nTaps : {gr::Size_t(33), gr::Size_t(45), gr::Size_t(89)}) {
            const gr::property_map settings{{"samples_per_symbol", 4.0}, {"rolloff", 0.35}, {"filter_length", nTaps}, {"noise_bandwidth", 0.01}};

            FllBandEdge block     = makeFll(settings);
            const auto  reference = driveFll(block, std::span<const CF>(input));

            for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 4096UZ}) {
                for (const bool wired : {true, false}) {
                    FllBandEdge other = makeFll(settings);
                    expect(std::ranges::equal(driveFll(other, std::span<const CF>(input), chunk, {wired, wired, wired}).samples, reference.samples)) << "N=" << nTaps << " chunk " << chunk;
                }
            }
        }
    };
};

int main() { /* not needed for ut */ }
