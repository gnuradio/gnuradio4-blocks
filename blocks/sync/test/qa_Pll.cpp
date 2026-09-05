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

#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/sync/Pll.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

// Frequencies are radians per sample throughout, as they are in the blocks. The settling anchors quoted below were
// computed by running this loop, and they are asserted no tighter than that, because the block is written in float
// and the third one is at the edge of what a float integrator resolves.

namespace {

using gr::blocks::sync::PllCarrierTracking;
using gr::blocks::sync::PllFreqDet;
using gr::blocks::sync::PllRefOut;
namespace test = gr::blocks::testing::span;

using CF = std::complex<float>;

constexpr double kPi        = std::numbers::pi;
constexpr double kBandwidth = 0.01;

template<typename TBlock>
void configure(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
}

PllCarrierTracking makeCarrier(gr::property_map settings = {}) {
    PllCarrierTracking block(std::move(settings));
    configure(block);
    return block;
}

PllFreqDet makeFreqDet(gr::property_map settings = {}) {
    PllFreqDet block(std::move(settings));
    configure(block);
    return block;
}

PllRefOut makeRefOut(gr::property_map settings = {}) {
    PllRefOut block(std::move(settings));
    configure(block);
    return block;
}

std::vector<CF> tone(std::size_t nSamples, double cyclesPerSample, double amplitude = 1.0, double initialPhase = 0.0) {
    std::vector<CF> signal(nSamples);
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        const double phase = initialPhase + 2.0 * kPi * cyclesPerSample * static_cast<double>(i);
        signal[i]          = CF(static_cast<float>(amplitude * std::cos(phase)), static_cast<float>(amplitude * std::sin(phase)));
    }
    return signal;
}

auto driveCarrier(PllCarrierTracking& block, std::span<const CF> input, std::size_t chunk = 0UZ, std::array<bool, 3UZ> wired = {true, true, true}, std::span<const gr::Tag> tags = {}) { return test::runPorts<3UZ, CF>(block, input, chunk, wired, tags); }

auto driveFreqDet(PllFreqDet& block, std::span<const CF> input, std::size_t chunk = 0UZ, std::array<bool, 2UZ> wired = {true, true}, std::span<const gr::Tag> tags = {}) { return test::runPorts<2UZ, float>(block, input, chunk, wired, tags); }

auto driveRefOut(PllRefOut& block, std::span<const CF> input, std::size_t chunk = 0UZ, std::array<bool, 3UZ> wired = {true, true, true}, std::span<const gr::Tag> tags = {}) { return test::runPorts<3UZ, CF>(block, input, chunk, wired, tags); }

/// @brief Samples until `|arg(y)|` stays below @p bound for the rest of the run.
[[nodiscard]] std::size_t settlingLength(std::span<const CF> derotated, float bound) {
    std::size_t last = 0UZ;
    for (std::size_t i = 0UZ; i < derotated.size(); ++i) {
        if (std::abs(std::atan2(derotated[i].imag(), derotated[i].real())) >= bound) {
            last = i + 1UZ;
        }
    }
    return last;
}

[[nodiscard]] float largestResidual(std::span<const CF> derotated, std::size_t from) {
    float worst = 0.f;
    for (std::size_t i = from; i < derotated.size(); ++i) {
        worst = std::max(worst, std::abs(std::atan2(derotated[i].imag(), derotated[i].real())));
    }
    return worst;
}

} // namespace

const boost::ut::suite<"Pll"> pllTests = [] {
    using namespace boost::ut;

    "the derotated argument is the wrapped phase difference, for any phase"_test = [] {
        std::mt19937                           rng(20260821U);
        std::uniform_real_distribution<double> amplitude(0.01, 10.0);
        std::uniform_real_distribution<double> angle(-kPi, kPi);
        std::uniform_real_distribution<double> loopPhase(-50.0, 50.0);

        double worst = 0.0;
        for (std::size_t trial = 0UZ; trial < 10000UZ; ++trial) {
            const double a   = amplitude(rng);
            const double phi = angle(rng);
            const double p   = loopPhase(rng);

            const std::complex<double> x = std::polar(a, phi);
            const std::complex<double> y = x * std::polar(1.0, -p);
            worst                        = std::max(worst, std::abs(std::atan2(y.imag(), y.real()) - std::remainder(phi - p, 2.0 * kPi)));
        }
        expect(lt(worst, 1e-6)) << "arg(x*exp(-j*p)) must equal remainder(arg(x) - p, 2*pi); worst " << worst;
    };

    "a frequency step is tracked to its exact scaling with no residual phase error"_test = [] {
        constexpr std::size_t kSamples = 8000UZ;
        for (const double offset : {1e-4, 1e-3, 1e-2}) {
            const std::vector<CF> input = tone(kSamples, offset);

            PllFreqDet detector = makeFreqDet({{"noise_bandwidth", kBandwidth}});
            const auto measured = driveFreqDet(detector, std::span<const CF>(input));

            PllCarrierTracking carrier   = makeCarrier({{"noise_bandwidth", kBandwidth}});
            const auto         derotated = driveCarrier(carrier, std::span<const CF>(input));
            const auto         expected  = static_cast<float>(2.0 * kPi * offset);

            // A relative bound of 1e-5 is what the two larger offsets meet with room. At 1e-4 cycles
            // per sample the float phase accumulator sets the floor instead: adding an increment of 6.3e-4 to a phase
            // of order 1 rounds at 1.2e-7, and the integrator absorbs that rounding by reading about 2e-8 rad/sample
            // low — 3.5e-5 relative, and a property of the accumulator's precision rather than of the loop.
            const float tolerance = std::max(std::abs(expected) * 1e-5f, 1e-7f);
            for (std::size_t i = kSamples - 1000UZ; i < kSamples; ++i) {
                expect(approx(measured.samples[i], expected, tolerance)) << "settled frequency at offset " << offset;
            }
            expect(lt(largestResidual(std::span<const CF>(derotated.samples), kSamples - 5000UZ), 1e-4f)) << "second order leaves no steady-state phase error at offset " << offset;
        }
    };

    "the frequency estimate at index n has already seen sample n"_test = [] {
        const std::vector<CF> input = tone(16UZ, 0.0, 1.0, 0.5);

        PllFreqDet  detector = makeFreqDet({{"noise_bandwidth", kBandwidth}});
        const float beta     = detector.loop().beta();
        const auto  measured = driveFreqDet(detector, std::span<const CF>(input));

        expect(neq(measured.samples[0], 0.f)) << "the pre-update convention would emit exactly zero here";
        expect(approx(measured.samples[0], beta * 0.5f, std::abs(beta) * 1e-5f)) << "the first output is beta times the first error";
    };

    "the reference and the derotated output multiply back to the input"_test = [] {
        const std::vector<CF> input = tone(4000UZ, 0.003, 0.7);

        PllCarrierTracking carrier   = makeCarrier({{"noise_bandwidth", kBandwidth}});
        PllRefOut          reference = makeRefOut({{"noise_bandwidth", kBandwidth}});
        const auto         removed   = driveCarrier(carrier, std::span<const CF>(input));
        const auto         replica   = driveRefOut(reference, std::span<const CF>(input));

        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            const CF product = removed.samples[i] * replica.samples[i];
            expect(lt(std::abs(product - input[i]), 1e-5f)) << "identity at i=" << i;
            expect(approx(std::abs(replica.samples[i]), 1.0f, 1e-6f)) << "unit amplitude at i=" << i;
        }
    };

    "the reference is unit amplitude during acquisition and on silence"_test = [] {
        const std::vector<CF> silence(512UZ, CF{});

        PllRefOut  reference = makeRefOut({{"noise_bandwidth", kBandwidth}});
        const auto replica   = driveRefOut(reference, std::span<const CF>(silence));
        for (std::size_t i = 0UZ; i < silence.size(); ++i) {
            expect(approx(std::abs(replica.samples[i]), 1.0f, 1e-6f)) << "silence at i=" << i;
        }
    };

    "a stream of zeros coasts at whatever frequency the loop was given"_test = [] {
        constexpr std::size_t      kSamples = 256UZ;
        const std::vector<CF>      silence(kSamples, CF{});
        const std::vector<gr::Tag> tags{gr::Tag{0UZ, gr::property_map{{"freq_est", 0.1}}}};

        PllCarrierTracking carrier   = makeCarrier();
        PllFreqDet         detector  = makeFreqDet();
        PllRefOut          reference = makeRefOut();

        const auto removed  = driveCarrier(carrier, std::span<const CF>(silence), 0UZ, {true, true, true}, std::span<const gr::Tag>(tags));
        const auto measured = driveFreqDet(detector, std::span<const CF>(silence), 0UZ, {true, true}, std::span<const gr::Tag>(tags));
        const auto replica  = driveRefOut(reference, std::span<const CF>(silence), 0UZ, {true, true, true}, std::span<const gr::Tag>(tags));

        for (std::size_t i = 0UZ; i < kSamples; ++i) {
            expect(eq(removed.samples[i], CF{})) << "zero in, zero out at i=" << i;
            expect(eq(measured.samples[i], 0.1f)) << "the frequency is undisturbed at i=" << i;
            expect(approx(std::abs(replica.samples[i]), 1.0f, 1e-6f)) << "the replica keeps turning at i=" << i;
            expect(!std::isnan(measured.samples[i])) << "no NaN at i=" << i;
        }
        const float advance = std::arg(replica.samples[11] * std::conj(replica.samples[10]));
        expect(approx(advance, 0.1f, 1e-5f)) << "the replica advances at the loop frequency";
    };

    "nothing about the loop depends on the input amplitude"_test = [] {
        constexpr std::size_t        kSamples = 6000UZ;
        std::array<std::size_t, 3UZ> settling{};
        std::array<float, 3UZ>       settled{};

        std::size_t which = 0UZ;
        for (const double amplitude : {1.0 / 64.0, 1.0, 64.0}) {
            const std::vector<CF> input = tone(kSamples, 0.002, amplitude);

            PllFreqDet detector = makeFreqDet({{"noise_bandwidth", kBandwidth}});
            const auto measured = driveFreqDet(detector, std::span<const CF>(input));

            PllCarrierTracking carrier   = makeCarrier({{"noise_bandwidth", kBandwidth}});
            const auto         derotated = driveCarrier(carrier, std::span<const CF>(input));

            settled[which]  = measured.samples[kSamples - 1UZ];
            settling[which] = settlingLength(std::span<const CF>(derotated.samples), 0.01f);
            ++which;
        }

        expect(approx(settled[0], settled[1], std::abs(settled[1]) * 1e-5f)) << "1/64 scaling must not move the estimate";
        expect(approx(settled[2], settled[1], std::abs(settled[1]) * 1e-5f)) << "64x scaling must not move the estimate";
        for (const std::size_t which2 : {0UZ, 2UZ}) {
            const double ratio = static_cast<double>(settling[which2]) / static_cast<double>(settling[1]);
            expect(lt(std::abs(ratio - 1.0), 0.05)) << "settling length must not move by more than 5%, got ratio " << ratio;
        }
    };

    "the settled estimate carries no bias from the arctangent"_test = [] {
        constexpr std::size_t kSamples = 12000UZ;
        const std::vector<CF> input    = tone(kSamples, 0.004);

        PllFreqDet detector = makeFreqDet({{"noise_bandwidth", kBandwidth}});
        const auto measured = driveFreqDet(detector, std::span<const CF>(input));

        double mean = 0.0;
        for (std::size_t i = kSamples - 4000UZ; i < kSamples; ++i) {
            mean += static_cast<double>(measured.samples[i]);
        }
        mean /= 4000.0;
        const double expected = 2.0 * kPi * 0.004;
        expect(lt(std::abs(mean - expected) / expected, 1e-5)) << "mean of the settled tail " << mean << " against " << expected;
    };

    "the clamp is a hard limit, and a loop parked on it reports unlocked"_test = [] {
        constexpr std::size_t kSamples   = 4000UZ;
        const std::vector<CF> offsetTone = tone(kSamples, 0.05);

        // Bounds pinned together: the integrator cannot move, so saturation is all that is exercised.
        PllFreqDet pinned = makeFreqDet({{"noise_bandwidth", kBandwidth}, {"max_frequency", 0.f}, {"min_frequency", 0.f}});
        const auto held   = driveFreqDet(pinned, std::span<const CF>(offsetTone));
        for (std::size_t i = 0UZ; i < kSamples; ++i) {
            expect(eq(held.samples[i], 0.f)) << "a pinned loop must not move at i=" << i;
        }
        expect(pinned.loop().saturated()) << "saturated() must say so";
        expect(!pinned.locked) << "a saturated loop is not locked";

        // A bound below the offset: the loop pushes against it, reaches it, and never crosses it. It cannot hold
        // still there — an offset it cannot reach keeps the phase turning, so the error changes sign once a cycle —
        // and what the clamp guarantees is the bound, not stillness.
        PllFreqDet capped   = makeFreqDet({{"noise_bandwidth", kBandwidth}, {"max_frequency", 0.05f}, {"min_frequency", -0.05f}});
        const auto limited  = driveFreqDet(capped, std::span<const CF>(offsetTone));
        const auto extremes = std::ranges::minmax(limited.samples);
        expect(le(extremes.max, 0.05f)) << "the frequency must never cross the bound";
        expect(ge(extremes.min, -0.05f)) << "either bound";
        expect(eq(extremes.max, 0.05f)) << "and must reach it";
        expect(!capped.locked) << "a loop outside its clamp is not locked";

        expect(throws([] { std::ignore = makeFreqDet({{"min_frequency", 1.f}, {"max_frequency", -1.f}}); })) << "crossed bounds";
        expect(throws([] { std::ignore = makeFreqDet({{"noise_bandwidth", 0.0}}); })) << "zero bandwidth";
        expect(throws([] { std::ignore = makeFreqDet({{"damping", -1.0}}); })) << "negative damping";
    };

    "saturating the loop leaves no windup to unwind"_test = [] {
        constexpr std::size_t kHeld  = 1000UZ;
        constexpr std::size_t kAfter = 3000UZ;

        std::vector<CF> input(kHeld + kAfter);
        double          phase = 0.0;
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            input[i] = CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
            phase += i < kHeld ? 2.0 * kPi * 0.05 : 0.0;
        }

        PllFreqDet detector = makeFreqDet({{"noise_bandwidth", kBandwidth}, {"max_frequency", 0.05f}, {"min_frequency", -0.05f}});
        const auto measured = driveFreqDet(detector, std::span<const CF>(input));

        const float pressed = std::ranges::max(std::span<const float>(measured.samples).first(kHeld));
        expect(eq(pressed, 0.05f)) << "the loop must have been pressed against the bound while the offset was there";
        expect(lt(std::abs(measured.samples[kHeld + 2000UZ]), 1e-3f)) << "and must be back within 20/(Bn*T) samples, got " << measured.samples[kHeld + 2000UZ];
    };

    "the lock metric means the same thing at every input level"_test = [] {
        constexpr std::size_t  kSamples = 40000UZ;
        const gr::property_map settings{{"noise_bandwidth", kBandwidth}, {"lock_time_constant", 2000.0}};

        const auto metricOf = [&](std::span<const CF> input, gr::property_map extra) {
            gr::property_map merged = settings;
            for (const auto& [key, value] : extra) {
                merged[key] = value;
            }
            PllCarrierTracking carrier = makeCarrier(std::move(merged));
            std::ignore                = driveCarrier(carrier, input, 0UZ, {false, false, true});
            return std::pair{carrier.lockMetric(), static_cast<bool>(carrier.locked)};
        };

        const std::vector<CF> clean           = tone(kSamples, 0.001);
        const std::vector<CF> loud            = tone(kSamples, 0.001, 5.0);
        const auto [cleanMetric, cleanLocked] = metricOf(std::span<const CF>(clean), {});
        const auto [loudMetric, loudLocked]   = metricOf(std::span<const CF>(loud), {});
        expect(approx(cleanMetric, 1.0f, 0.02f)) << "a locked carrier at amplitude 1";
        expect(approx(loudMetric, 1.0f, 0.02f)) << "a locked carrier at amplitude 5";
        expect(lt(std::abs(loudMetric - cleanMetric), 1e-4f)) << "and the two must agree, since the metric is divided by the amplitude reference";
        expect(cleanLocked && loudLocked);

        // A carrier of unit amplitude in complex AWGN of total variance 0.2512, i.e. 6 dB of signal-to-noise ratio.
        std::mt19937                     rng(7U);
        std::normal_distribution<double> noise(0.0, std::sqrt(0.2512 / 2.0));
        std::vector<CF>                  noisy(kSamples);
        std::vector<CF>                  noiseOnly(kSamples);
        for (std::size_t i = 0UZ; i < kSamples; ++i) {
            const double phase = 2.0 * kPi * 0.001 * static_cast<double>(i);
            noisy[i]           = CF(static_cast<float>(std::cos(phase) + noise(rng)), static_cast<float>(std::sin(phase) + noise(rng)));
            noiseOnly[i]       = CF(static_cast<float>(noise(rng)), static_cast<float>(noise(rng)));
        }
        const auto [noisyMetric, noisyLocked] = metricOf(std::span<const CF>(noisy), {});
        const auto [noiseMetric, noiseLocked] = metricOf(std::span<const CF>(noiseOnly), {});
        expect(gt(noisyMetric, 0.85f)) << "a carrier 6 dB above the noise, got " << noisyMetric;
        expect(noisyLocked);
        expect(lt(noiseMetric, 0.10f)) << "noise alone, got " << noiseMetric;
        expect(!noiseLocked);

        const std::vector<CF> outside             = tone(kSamples, 0.05);
        const auto [outsideMetric, outsideLocked] = metricOf(std::span<const CF>(outside), {{"max_frequency", 0.05f}, {"min_frequency", -0.05f}});
        expect(lt(outsideMetric, 0.10f)) << "a tone outside the frequency clamp, got " << outsideMetric;
        expect(!outsideLocked) << "and the saturation test catches it even if the average does not";

        expect(gt(std::min(cleanMetric, noisyMetric), 0.7f) && lt(std::max(noiseMetric, outsideMetric), 0.7f)) << "a threshold of 0.7 separates every one of these";
    };

    "the lock time constant changes how fast the metric settles, not what it settles to"_test = [] {
        // From a cold start both accumulators rise together and their ratio is meaningful almost at once, so the
        // averaging length only shows itself in a transition: noise first, then a carrier of the same magnitude.
        constexpr std::size_t kNoise   = 20000UZ;
        constexpr std::size_t kCarrier = 30000UZ;

        std::mt19937                     rng(31U);
        std::normal_distribution<double> noise(0.0, std::sqrt(0.5));
        std::vector<CF>                  input(kNoise + kCarrier);
        for (std::size_t i = 0UZ; i < kNoise; ++i) {
            input[i] = CF(static_cast<float>(noise(rng)), static_cast<float>(noise(rng)));
        }
        for (std::size_t i = kNoise; i < input.size(); ++i) {
            input[i] = CF(1.f, 0.f);
        }

        const auto crossing = [&](double timeConstant) {
            PllCarrierTracking block   = makeCarrier({{"noise_bandwidth", kBandwidth}, {"lock_time_constant", timeConstant}});
            const auto         history = driveCarrier(block, std::span<const CF>(input), 0UZ, {false, false, true});
            std::size_t        at      = input.size();
            for (std::size_t i = kNoise; i < input.size(); ++i) {
                if (history.aux[2][i] > 0.5f) {
                    at = i - kNoise;
                    break;
                }
            }
            return std::pair{at, block.lockMetric()};
        };

        const auto [quickAt, quickSettled] = crossing(100.0);
        const auto [slowAt, slowSettled]   = crossing(5000.0);
        expect(gt(slowAt, 4UZ * quickAt)) << "the long constant must be far slower: " << quickAt << " against " << slowAt;
        expect(approx(quickSettled, slowSettled, 0.02f)) << "and both must arrive at the same place";
    };

    "the lock detector runs only where its answer is read"_test = [] {
        constexpr std::size_t  kSamples = 40000UZ;
        const gr::property_map settings{{"noise_bandwidth", kBandwidth}, {"lock_time_constant", 2000.0}};
        const std::vector<CF>  clean = tone(kSamples, 0.001);

        PllRefOut wired = makeRefOut(settings);
        std::ignore     = driveRefOut(wired, std::span<const CF>(clean), 0UZ, {false, false, true});
        expect(wired.locked) << "a clean carrier with the lock port connected";
        expect(gt(wired.lockMetric(), 0.9f));

        // the amplitude reference is a square root a sample and the only one in this family, so it is not paid for
        // where nothing reads it — and the metric is then not maintained
        PllRefOut bare = makeRefOut(settings);
        std::ignore    = driveRefOut(bare, std::span<const CF>(clean), 0UZ, {false, false, false});
        expect(!bare.locked) << "with the port unwired the detector never runs";
        expect(eq(bare.lockMetric(), 0.f));

        // the squelch reads the metric as much as the port does, so it keeps the detector running on its own
        PllCarrierTracking gated = makeCarrier({{"noise_bandwidth", kBandwidth}, {"lock_time_constant", 2000.0}, {"squelch_when_unlocked", true}});
        std::ignore              = driveCarrier(gated, std::span<const CF>(clean), 0UZ, {false, false, false});
        expect(gated.locked) << "the squelch is a reader too";
        expect(gt(gated.lockMetric(), 0.9f));
    };

    "tags ride through, the two loop tags do not, and a wild payload cannot hang the block"_test = [] {
        constexpr std::size_t      kSamples = 200UZ;
        const std::vector<CF>      input    = tone(kSamples, 0.001);
        const std::vector<gr::Tag> passenger{gr::Tag{40UZ, gr::property_map{{"passenger", 1.0}}}};

        PllCarrierTracking carrier = makeCarrier();
        const auto         carried = driveCarrier(carrier, std::span<const CF>(input), 0UZ, {true, true, true}, std::span<const gr::Tag>(passenger));
        expect(eq(carried.offsetsOf("passenger").size(), 1UZ)) << "the tag must arrive once";
        expect(eq(carried.offsetsOf("passenger")[0], 40UZ)) << "at the same offset";
        for (std::size_t side = 0UZ; side < 3UZ; ++side) {
            expect(eq(carried.auxTags[side].size(), 1UZ)) << "and on every connected port, side " << side;
        }

        const std::vector<gr::Tag> steering{gr::Tag{10UZ, gr::property_map{{"phase_est", 0.25}, {"freq_est", 0.01}}}};
        PllCarrierTracking         steered = makeCarrier();
        const auto                 driven  = driveCarrier(steered, std::span<const CF>(input), 0UZ, {true, true, true}, std::span<const gr::Tag>(steering));
        expect(eq(driven.tags.size(), 0UZ)) << "the loop tags are consumed, not forwarded";
        expect(approx(driven.aux[1][10UZ], 0.25f, 1e-6f)) << "the phase must be in force on the tagged sample";

        const std::vector<gr::Tag> wild{gr::Tag{5UZ, gr::property_map{{"phase_est", 1e9}}}};
        PllCarrierTracking         wilded = makeCarrier();
        const auto                 tamed  = driveCarrier(wilded, std::span<const CF>(input), 0UZ, {true, true, true}, std::span<const gr::Tag>(wild));
        expect(le(std::abs(tamed.aux[1][5UZ]), static_cast<float>(kPi))) << "a payload of 1e9 is wrapped, not walked to, and leaves a canonical angle";
        expect(!std::isnan(tamed.aux[1][5UZ]));

        for (const double payload : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
            const std::vector<gr::Tag> bad{gr::Tag{5UZ, gr::property_map{{"phase_est", payload}, {"freq_est", payload}}}};
            PllCarrierTracking         rejected = makeCarrier();
            const auto                 guarded  = driveCarrier(rejected, std::span<const CF>(input), 0UZ, {true, true, true}, std::span<const gr::Tag>(bad));
            PllCarrierTracking         plain    = makeCarrier();
            const auto                 untagged = driveCarrier(plain, std::span<const CF>(input), 0UZ, {true, true, true});
            expect(eq(rejected.ignoredTagPayloads(), 2ULL)) << "both payloads counted";
            expect(std::ranges::equal(guarded.samples, untagged.samples)) << "and the state must be what it would have been without the tag";
        }
    };

    "the output does not depend on chunking or on which side ports are wired"_test = [] {
        const std::vector<CF> input = tone(5000UZ, 0.0017, 1.3);

        PllCarrierTracking carrier      = makeCarrier({{"noise_bandwidth", kBandwidth}});
        PllFreqDet         detector     = makeFreqDet({{"noise_bandwidth", kBandwidth}});
        PllRefOut          reference    = makeRefOut({{"noise_bandwidth", kBandwidth}});
        const auto         carrierRef   = driveCarrier(carrier, std::span<const CF>(input));
        const auto         detectorRef  = driveFreqDet(detector, std::span<const CF>(input));
        const auto         referenceRef = driveRefOut(reference, std::span<const CF>(input));

        for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 4096UZ}) {
            for (const bool wired : {true, false}) {
                PllCarrierTracking a = makeCarrier({{"noise_bandwidth", kBandwidth}});
                PllFreqDet         b = makeFreqDet({{"noise_bandwidth", kBandwidth}});
                PllRefOut          c = makeRefOut({{"noise_bandwidth", kBandwidth}});
                expect(std::ranges::equal(driveCarrier(a, std::span<const CF>(input), chunk, {wired, wired, wired}).samples, carrierRef.samples)) << "PllCarrierTracking at chunk " << chunk;
                expect(std::ranges::equal(driveFreqDet(b, std::span<const CF>(input), chunk, {wired, wired}).samples, detectorRef.samples)) << "PllFreqDet at chunk " << chunk;
                expect(std::ranges::equal(driveRefOut(c, std::span<const CF>(input), chunk, {wired, wired, wired}).samples, referenceRef.samples)) << "PllRefOut at chunk " << chunk;
            }
        }
    };

    "a live bandwidth change moves no state and drops no sample"_test = [] {
        constexpr std::size_t kSplit   = 500UZ;
        constexpr std::size_t kSamples = 1500UZ;
        const std::vector<CF> input    = tone(kSamples, 0.002);

        PllFreqDet  detector  = makeFreqDet({{"noise_bandwidth", kBandwidth}});
        const auto  first     = driveFreqDet(detector, std::span<const CF>(input).first(kSplit));
        const float phase     = detector.loop().phase();
        const float frequency = detector.loop().frequency();

        std::ignore = detector.settings().setStaged({{"noise_bandwidth", 0.05}});
        std::ignore = detector.settings().applyStagedParameters();
        expect(eq(detector.loop().phase(), phase)) << "the phase must not move";
        expect(eq(detector.loop().frequency(), frequency)) << "nor the frequency";
        expect(neq(detector.loop().beta(), 0.f));

        const auto second = driveFreqDet(detector, std::span<const CF>(input).subspan(kSplit));
        expect(eq(first.samples.size() + second.samples.size(), kSamples)) << "no sample repeated or dropped";
        expect(eq(first.consumed + second.consumed, kSamples));
    };
};

int main() { /* not needed for ut */ }
