#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <format>
#include <limits>
#include <numbers>
#include <optional>
#include <random>
#include <span>
#include <string>
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

using gr::blocks::sync::detail::advanceCarrier;
using gr::blocks::sync::detail::applyCarrierSettings;
using gr::blocks::sync::detail::ArgumentTile;
using gr::blocks::sync::detail::CarrierLoop;
using gr::blocks::sync::detail::derotator;

/// @brief A tag the reference applies before sample `index`: the loop's phase, frequency, or both.
struct Steer {
    std::size_t          index;
    std::optional<float> phase;
    std::optional<float> frequency;
};

struct History {
    std::vector<CF>    derotated;
    std::vector<float> frequency;
    std::vector<float> phase;
};

[[nodiscard]] CarrierLoop makeLoop(double bandwidth, float bound = std::numbers::pi_v<float>) {
    CarrierLoop loop{bandwidth, std::numbers::sqrt2 / 2.0, 1.0};
    applyCarrierSettings(loop, bandwidth, std::numbers::sqrt2 / 2.0, -bound, bound);
    return loop;
}

/// @brief The error by the product `x * exp(-j*p)`: `arg(x * exp(-j*p))`, and zero for an exact zero.
[[nodiscard]] float referenceError(float phase, CF sample) {
    const CF y = sample * derotator(phase);
    return sample.real() == 0.f && sample.imag() == 0.f ? 0.f : std::atan2(y.imag(), y.real());
}

/// @brief The loop run sample by sample on `referenceError`, with its frequency clamped to `+-bound`.
[[nodiscard]] History referenceRun(std::span<const CF> input, std::span<const Steer> steers = {}, float bound = std::numbers::pi_v<float>) {
    CarrierLoop loop = makeLoop(kBandwidth, bound);
    History     history;
    std::size_t next = 0UZ;
    for (std::size_t i = 0UZ; i < input.size(); ++i) {
        for (; next < steers.size() && steers[next].index <= i; ++next) {
            if (steers[next].phase) {
                loop.setPhase(*steers[next].phase);
            }
            if (steers[next].frequency) {
                loop.setFrequency(*steers[next].frequency);
            }
        }
        const float phase = loop.phase();
        history.derotated.push_back(input[i] * derotator(phase));
        history.phase.push_back(phase);
        std::ignore = loop.step(referenceError(phase, input[i]));
        history.frequency.push_back(loop.frequency());
    }
    return history;
}

/// @brief The error `advanceCarrier` feeds the loop for one sample at a given loop phase, read back from a first step taken at zero frequency.
[[nodiscard]] float patchError(float phase, CF sample) {
    CarrierLoop loop = makeLoop(kBandwidth);
    loop.reset(phase, 0.f);
    ArgumentTile tile;
    std::ignore = advanceCarrier(loop, sample, tile.at(std::span<const CF>(&sample, 1UZ), 0UZ, 1UZ));
    return loop.frequency() / loop.beta();
}

[[nodiscard]] float referenceStepError(float phase, CF sample) {
    CarrierLoop loop = makeLoop(kBandwidth);
    loop.reset(phase, 0.f);
    std::ignore = loop.step(referenceError(loop.phase(), sample));
    return loop.frequency() / loop.beta();
}

/// @brief The two errors are the same number to rounding, or both sit on the `+-pi` cut from either side of it.
[[nodiscard]] bool sameError(float patch, float reference, float tolerance = 2e-6f) {
    if (std::isnan(patch) || std::isnan(reference)) {
        return std::isnan(patch) && std::isnan(reference);
    }
    const float pi = std::numbers::pi_v<float>;
    return std::abs(patch - reference) <= tolerance || (std::abs(patch) >= pi - tolerance && std::abs(reference) >= pi - tolerance);
}

[[nodiscard]] float largestDifference(std::span<const float> a, std::span<const float> b) {
    float worst = 0.f;
    for (std::size_t i = 0UZ; i < std::min(a.size(), b.size()); ++i) {
        worst = std::max(worst, std::abs(a[i] - b[i]));
    }
    return worst;
}

[[nodiscard]] float largestDifference(std::span<const CF> a, std::span<const CF> b, float scale = 1.f) {
    float worst = 0.f;
    for (std::size_t i = 0UZ; i < std::min(a.size(), b.size()); ++i) {
        worst = std::max(worst, std::abs(a[i] - b[i]) / scale);
    }
    return worst;
}

/// @brief A tone of `cyclesPerSample` from phase 0.7, with white noise of sigma `noiseSigma` on each part.
std::vector<CF> noisyTone(std::size_t nSamples, double cyclesPerSample, double noiseSigma, unsigned seed) {
    std::mt19937                     rng(seed);
    std::normal_distribution<double> noise(0.0, noiseSigma);
    std::vector<CF>                  signal(nSamples);
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        const double phase = 0.7 + 2.0 * kPi * cyclesPerSample * static_cast<double>(i);
        signal[i]          = CF(static_cast<float>(std::cos(phase) + noise(rng)), static_cast<float>(std::sin(phase) + noise(rng)));
    }
    return signal;
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

    "the error is the reference arg(x * exp(-j*p)) at every phase difference, including the wrap"_test = [] {
        // the loop open: the error read for samples at every angle against loop phases across the whole range
        std::vector<float> angles{-3.14159274f, 3.14159274f, -3.1415925f, 3.1415925f, -3.14f, 3.14f, -1.5707964f, 1.5707964f, 0.f, 1e-7f, -1e-7f, 2.5f, -2.5f};
        std::vector<float> phases{-3.14159274f, 3.14159274f, -3.14159f, 3.14159f, -3.f, 3.f, -1.f, 1.f, 0.f, 0.5f, -0.5f, 1e-7f};
        for (float a = -3.1f; a < 3.1f; a += 0.1237f) {
            angles.push_back(a);
            phases.push_back(a * 0.9f);
        }
        std::size_t cases = 0UZ;
        std::size_t bad   = 0UZ;
        std::string first;
        for (const float phase : phases) {
            for (const float angle : angles) {
                for (const float amplitude : {1.f, 1e-3f, 1e3f}) {
                    const CF    x{amplitude * std::cos(angle), amplitude * std::sin(angle)};
                    const float patch     = patchError(phase, x);
                    const float reference = referenceStepError(phase, x);
                    if (!sameError(patch, reference) || !(std::abs(patch) <= std::numbers::pi_v<float> + 1e-6f)) {
                        first = bad == 0UZ ? std::format("phase {} angle {} amplitude {}: {} against {}", phase, angle, amplitude, patch, reference) : first;
                        ++bad;
                    }
                    ++cases;
                }
            }
        }
        expect(eq(bad, 0UZ)) << "errors that are not the reference's, or not reduced to one turn; the first: " << first;
        expect(gt(cases, 1000UZ));

        // the difference at the cut itself: the argument of -1 with either sign of zero, against a loop phase that puts the difference on +-pi
        for (const float imag : {0.f, -0.f}) {
            for (const float phase : {0.f, 1e-7f, -1e-7f, 3.f, -3.f}) {
                expect(sameError(patchError(phase, CF{-1.f, imag}), referenceStepError(phase, CF{-1.f, imag}))) << "phase " << phase << " imag " << imag;
            }
        }
    };

    "an input of exactly zero feeds the loop zero error, and a stream of tone and zeros matches the reference"_test = [] {
        for (const CF zero : {CF{0.f, 0.f}, CF{-0.f, 0.f}, CF{0.f, -0.f}, CF{-0.f, -0.f}}) {
            for (const float phase : {0.f, 1.f, -3.f, 3.1f}) {
                expect(eq(patchError(phase, zero), 0.f)) << "an exact zero is no error at phase " << phase;
            }
        }

        std::vector<CF> input = tone(3000UZ, 0.01, 1.0, 0.4);
        for (const std::size_t at : {100UZ, 101UZ, 255UZ, 256UZ, 511UZ, 1500UZ, 1501UZ, 1502UZ}) {
            input[at] = CF{};
        }
        const History      expected = referenceRun(std::span<const CF>(input));
        PllCarrierTracking carrier  = makeCarrier({{"noise_bandwidth", kBandwidth}});
        PllFreqDet         detector = makeFreqDet({{"noise_bandwidth", kBandwidth}});
        const auto         removed  = driveCarrier(carrier, std::span<const CF>(input));
        const auto         measured = driveFreqDet(detector, std::span<const CF>(input));
        expect(lt(largestDifference(std::span<const CF>(removed.samples), std::span<const CF>(expected.derotated)), 1e-4f)) << "derotated output";
        expect(lt(largestDifference(std::span<const float>(measured.samples), std::span<const float>(expected.frequency)), 1e-6f)) << "frequency";
        expect(lt(largestDifference(std::span<const float>(removed.aux[1]), std::span<const float>(expected.phase)), 1e-4f)) << "phase";
        for (const std::size_t at : {100UZ, 255UZ, 256UZ, 1501UZ}) {
            expect(eq(removed.samples[at], CF{})) << "a zero in is a zero out at " << at;
        }
    };

    "a sample that is not finite gives NaN where the reference does, or the argument of its direction"_test = [] {
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const float inf = std::numeric_limits<float>::infinity();
        for (const float phase : {0.f, 1.f, -2.f}) {
            for (const CF sample : {CF{nan, 0.f}, CF{0.f, nan}, CF{nan, nan}, CF{1.f, nan}}) {
                expect(std::isnan(patchError(phase, sample))) << "a NaN part gives a NaN error, as the reference does";
                expect(std::isnan(referenceStepError(phase, sample)));
            }
            // an infinite part has the argument of its direction; the reference's product of an infinity with a rotor part of either sign
            // or of zero is not that argument in general, so the error is checked against the exact value instead
            for (const CF sample : {CF{inf, 0.f}, CF{-inf, 0.f}, CF{0.f, inf}, CF{0.f, -inf}, CF{inf, 1.f}, CF{1.f, -inf}}) {
                const double direction = std::atan2(std::isinf(sample.imag()) ? std::copysign(1.0, sample.imag()) : 0.0, std::isinf(sample.real()) ? std::copysign(1.0, sample.real()) : 0.0);
                const double exact     = std::remainder(direction - static_cast<double>(phase), 2.0 * kPi);
                const float  patch     = patchError(phase, sample);
                expect(std::isfinite(patch)) << "an infinite sample is a direction at phase " << phase;
                expect(lt(std::abs(std::remainder(static_cast<double>(patch) - exact, 2.0 * kPi)), 2e-6)) << "got " << patch << " expected " << exact << " at phase " << phase;
            }
        }

        // in a stream: the samples before a NaN are the reference's, and from it on the loop is NaN in both
        std::vector<CF> input  = tone(1000UZ, 0.01);
        input[300]             = CF{nan, 0.f};
        const History expected = referenceRun(std::span<const CF>(input));
        PllFreqDet    detector = makeFreqDet({{"noise_bandwidth", kBandwidth}});
        const auto    measured = driveFreqDet(detector, std::span<const CF>(input));
        expect(lt(largestDifference(std::span<const float>(measured.samples).first(300UZ), std::span<const float>(expected.frequency).first(300UZ)), 1e-6f)) << "before the NaN";
        for (std::size_t i = 300UZ; i < input.size(); ++i) {
            expect(std::isnan(measured.samples[i]) == std::isnan(expected.frequency[i])) << "NaN state at " << i;
        }
    };

    "chunkings that split a tile, tags at its edges, and a loop at its frequency bound match the reference"_test = [] {
        constexpr std::size_t      kSamples = 1300UZ;
        const std::vector<CF>      input    = noisyTone(kSamples, 0.013, 0.05, 3U);
        const std::vector<gr::Tag> tags{gr::Tag{255UZ, gr::property_map{{"phase_est", -2.0}}}, gr::Tag{256UZ, gr::property_map{{"freq_est", 0.05}, {"edge", 1.0}}}, gr::Tag{257UZ, gr::property_map{{"phase_est", 2.5}}}, gr::Tag{511UZ, gr::property_map{{"edge", 2.0}}}, gr::Tag{512UZ, gr::property_map{{"phase_est", 0.0}, {"freq_est", -0.02}}}, gr::Tag{700UZ, gr::property_map{{"phase_est", 10.0}}}, gr::Tag{701UZ, gr::property_map{{"phase_est", -10.0}}}, gr::Tag{1024UZ, gr::property_map{{"phase_est", 3.0}}}};
        const std::vector<Steer>   steers{{255UZ, -2.f, std::nullopt}, {256UZ, std::nullopt, 0.05f}, {257UZ, 2.5f, std::nullopt}, {512UZ, 0.f, -0.02f}, {700UZ, 10.f, std::nullopt}, {701UZ, -10.f, std::nullopt}, {1024UZ, 3.f, std::nullopt}};
        const History              reference = referenceRun(std::span<const CF>(input), std::span<const Steer>(steers));

        PllCarrierTracking whole     = makeCarrier({{"noise_bandwidth", kBandwidth}});
        const auto         all       = driveCarrier(whole, std::span<const CF>(input), 0UZ, {true, true, true}, std::span<const gr::Tag>(tags));
        const float        deviation = largestDifference(std::span<const CF>(all.samples), std::span<const CF>(reference.derotated));
        expect(lt(deviation, 1e-4f)) << "one call against the sequential reference";
        expect(lt(largestDifference(std::span<const float>(all.aux[0]), std::span<const float>(reference.frequency)), 1e-5f)) << "frequency";
        expect(lt(largestDifference(std::span<const float>(all.aux[1]), std::span<const float>(reference.phase)), 1e-4f)) << "phase";
        expect(eq(all.offsetsOf("edge").size(), 2UZ));
        expect(eq(all.offsetsOf("edge")[0], 256UZ)) << "a tag on the first sample of the second tile";
        expect(eq(all.offsetsOf("edge")[1], 511UZ)) << "a tag on the last sample of the first tile";

        for (const std::size_t chunk : {1UZ, 7UZ, 100UZ, 255UZ, 256UZ, 257UZ, 300UZ, 511UZ, 512UZ, 513UZ}) {
            PllCarrierTracking cut    = makeCarrier({{"noise_bandwidth", kBandwidth}});
            const auto         pieces = driveCarrier(cut, std::span<const CF>(input), chunk, {true, true, true}, std::span<const gr::Tag>(tags));
            expect(std::ranges::equal(pieces.samples, all.samples)) << "output at chunk " << chunk;
            expect(std::ranges::equal(pieces.aux[0], all.aux[0])) << "frequency at chunk " << chunk;
            expect(std::ranges::equal(pieces.aux[1], all.aux[1])) << "phase at chunk " << chunk;
            expect(eq(pieces.offsetsOf("edge").size(), 2UZ)) << "tags at chunk " << chunk;
        }

        // the loop parked at its frequency bound by a tone beyond it, so the error crosses the cut again and again
        const std::vector<CF> beyond  = tone(kSamples, 0.02, 1.0, 0.3);
        const History         parked  = referenceRun(std::span<const CF>(beyond), {}, 0.05f);
        PllCarrierTracking    bounded = makeCarrier({{"noise_bandwidth", kBandwidth}, {"max_frequency", 0.05f}, {"min_frequency", -0.05f}});
        const auto            held    = driveCarrier(bounded, std::span<const CF>(beyond), 100UZ);
        expect(lt(largestDifference(std::span<const float>(held.aux[0]), std::span<const float>(parked.frequency)), 1e-5f)) << "frequency at the bound";
        expect(lt(largestDifference(std::span<const CF>(held.samples), std::span<const CF>(parked.derotated)), 1e-3f)) << "derotated output at the bound";
        expect(eq(parked.frequency.back(), 0.05f)) << "the reference is parked at the bound";
    };
};

int main() { /* not needed for ut */ }
