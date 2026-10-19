#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <format>
#include <limits>
#include <numbers>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/analog/NoiseBlanker.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::analog::NoiseBlanker;
using gr::blocks::testing::ProcessFunction;
using gr::blocks::testing::TagSink;
using gr::blocks::testing::TagSource;

namespace test = gr::blocks::testing::span;

using CF = std::complex<float>;

constexpr double kPi   = std::numbers::pi;
constexpr float  kRate = 96000.f;

template<typename T>
[[nodiscard]] NoiseBlanker<T> make(gr::property_map settings = {}) {
    NoiseBlanker<T> block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

template<typename TBlock>
void apply(TBlock& block, gr::property_map settings) {
    std::ignore = block.settings().setStaged(std::move(settings));
    std::ignore = block.settings().applyStagedParameters();
}

struct Random {
    std::uint64_t state = 0x243f6a8885a308d3ULL;

    [[nodiscard]] double uniform() noexcept {
        state += 0x9e3779b97f4a7c15ULL;
        std::uint64_t z = state;
        z               = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z               = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        z ^= z >> 31;
        return static_cast<double>(z >> 11) * 0x1.0p-53;
    }

    [[nodiscard]] double gaussian() noexcept { return std::sqrt(-2.0 * std::log(uniform() + 1e-300)) * std::cos(2.0 * kPi * uniform()); }

    /// Circular complex Gaussian of unit mean power. Each component has variance one half.
    [[nodiscard]] CF sample() noexcept { return CF(static_cast<float>(gaussian() * 0.7071067811865476), static_cast<float>(gaussian() * 0.7071067811865476)); }
};

[[nodiscard]] std::vector<CF> noise(std::size_t count, std::uint64_t seed) {
    Random          rng{seed};
    std::vector<CF> samples(count);
    for (CF& sample : samples) {
        sample = rng.sample();
    }
    return samples;
}

/// A constant-magnitude tone. |x|^2 always equals its own mean, and a correct detector does not fire on it.
[[nodiscard]] std::vector<CF> tone(std::size_t count, double amplitude, double normalized = 0.013) {
    std::vector<CF> samples(count);
    for (std::size_t i = 0UZ; i < count; ++i) {
        const double phase = 2.0 * kPi * normalized * static_cast<double>(i);
        samples[i]         = CF(static_cast<float>(amplitude * std::cos(phase)), static_cast<float>(amplitude * std::sin(phase)));
    }
    return samples;
}

/// @brief A signal band-limited to @p bandwidth of the sample rate, as a sum of random-phase bins inside it.
[[nodiscard]] std::vector<CF> bandLimited(std::size_t count, double bandwidth, std::uint64_t seed) {
    Random               rng{seed};
    const std::ptrdiff_t half = static_cast<std::ptrdiff_t>(bandwidth * 0.5 * static_cast<double>(count));
    std::vector<CF>      samples(count, CF{});
    std::vector<double>  phase(static_cast<std::size_t>(2 * half + 1));
    for (double& value : phase) {
        value = 2.0 * kPi * rng.uniform();
    }
    for (std::ptrdiff_t bin = -half; bin <= half; ++bin) {
        const double turn  = 2.0 * kPi * static_cast<double>(bin) / static_cast<double>(count);
        const double start = phase[static_cast<std::size_t>(bin + half)];
        for (std::size_t i = 0UZ; i < count; ++i) {
            const double angle = start + turn * static_cast<double>(i);
            samples[i] += CF(static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle)));
        }
    }
    const double scale = 1.0 / std::sqrt(static_cast<double>(2 * half + 1));
    for (CF& sample : samples) {
        sample *= static_cast<float>(scale);
    }
    return samples;
}

/// @brief Energy of @p signal inside `|f| <= bandwidth/2`, by a direct transform of the bins that matter.
[[nodiscard]] double inBandEnergy(std::span<const CF> signal, double bandwidth) {
    const std::ptrdiff_t half  = static_cast<std::ptrdiff_t>(bandwidth * 0.5 * static_cast<double>(signal.size()));
    double               total = 0.0;
    for (std::ptrdiff_t bin = -half; bin <= half; ++bin) {
        const double turn = -2.0 * kPi * static_cast<double>(bin) / static_cast<double>(signal.size());
        double       re   = 0.0;
        double       im   = 0.0;
        for (std::size_t i = 0UZ; i < signal.size(); ++i) {
            const double angle = turn * static_cast<double>(i);
            const double c     = std::cos(angle);
            const double s     = std::sin(angle);
            re += static_cast<double>(signal[i].real()) * c - static_cast<double>(signal[i].imag()) * s;
            im += static_cast<double>(signal[i].real()) * s + static_cast<double>(signal[i].imag()) * c;
        }
        total += (re * re + im * im) / static_cast<double>(signal.size() * signal.size());
    }
    return total;
}

[[nodiscard]] double meanPower(std::span<const CF> samples) {
    double total = 0.0;
    for (const CF& sample : samples) {
        total += static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
    }
    return total / static_cast<double>(samples.size());
}

[[nodiscard]] bool longTestsEnabled() { return std::getenv("ENABLE_LONG_TESTS") != nullptr; }

/// A magnitude threshold `k` carried into this block's power units: `T = (pi/4) k^2`, i.e. 20log10(k) - 1.0492 dB.
[[nodiscard]] double thresholdFromMagnitudeRatio(double k) { return 20.0 * std::log10(k) - 1.0492; }

/**
 * @brief Where a censored tracker settles on unit-power noise, and the run rate that follows.
 *
 * An exact reference gives exp(-T) for the trigger rate. A censored tracker is not exact. It does not learn from the
 * samples it rejected. It settles on E[p | p <= T*m] and not on E[p], and the fixed point of that is below one. At
 * the default 9.32 dB the shift is 0.17 %, and exp(-T) is the answer. At 6 dB the shift is 10 %, and exp(-T) is not
 * the answer. This is a property of censoring and not of this implementation.
 */
[[nodiscard]] std::pair<double, double> censoredRunRate(double thresholdDb, std::size_t blankSamples) {
    const double threshold = std::pow(10.0, thresholdDb / 10.0);
    double       mean      = 1.0;
    for (int step = 0; step < 200; ++step) { // m = E[p | p <= T*m] for a unit-mean exponential p
        const double u    = threshold * mean;
        const double tail = std::exp(-u);
        mean              = (1.0 - (1.0 + u) * tail) / (1.0 - tail);
    }
    const double perSample = std::exp(-threshold * mean);
    return {mean, perSample / (1.0 + perSample * static_cast<double>(blankSamples - 1UZ))};
}

/**
 * @brief P(|A + n|^2 > T) for a planted amplitude with |A|^2 = @p planted and unit-power complex Gaussian noise n.
 *
 * |A + n|^2 is a Poisson mixture of Gamma(k + 1, 1) with weights exp(-planted) planted^k / k!. The tail of
 * Gamma(k + 1, 1) at T is exp(-T) times the sum of T^j / j! for j up to k.
 */
[[nodiscard]] double detectionProbability(double planted, double threshold) {
    double total  = 0.0;
    double weight = std::exp(-planted);
    double term   = std::exp(-threshold);
    double tail   = 0.0;
    for (std::size_t k = 0UZ; k < 400UZ; ++k) {
        tail += term;
        total += weight * tail;
        term *= threshold / static_cast<double>(k + 1UZ);
        weight *= planted / static_cast<double>(k + 1UZ);
    }
    return total;
}

} // namespace

const boost::ut::suite<"NoiseBlanker"> noiseBlankerTests = [] {
    using namespace boost::ut;

    "a clean signal comes out delayed and otherwise untouched"_test = [] {
        for (const char* rule : {"interpolate", "hold", "zero"}) {
            NoiseBlanker<CF>       block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"replacement", std::string(rule)}});
            const std::vector<CF>  x     = tone(40000UZ, 0.5);
            const test::Result<CF> got   = test::run(block, std::span<const CF>(x), 997UZ);

            expect(eq(got.samples.size(), x.size())) << "1:1";
            std::size_t firstDifference = got.samples.size();
            for (std::size_t i = 9UZ; i < x.size(); ++i) {
                if (got.samples[i] != x[i - 9UZ]) {
                    firstDifference = i;
                    break;
                }
            }
            expect(eq(firstDifference, got.samples.size())) << std::format("{}: nothing above the threshold, so the output is the input delayed by 9, bit for bit", rule);
            for (std::size_t i = 0UZ; i < 9UZ; ++i) {
                expect(eq(got.samples[i], CF{})) << "and the delay line starts zeroed";
            }
        }
    };

    "the delay is there when the block is off and does not move when it is toggled"_test = [] {
        NoiseBlanker<CF> off = make<CF>({{"sample_rate", kRate}});
        std::vector<CF>  ramp(4000UZ);
        for (std::size_t i = 0UZ; i < ramp.size(); ++i) {
            ramp[i] = CF(static_cast<float>(i), static_cast<float>(2UZ * i));
        }
        const test::Result<CF> passed = test::run(off, std::span<const CF>(ramp), 137UZ);
        expect(eq(passed.samples.size(), ramp.size()));
        bool exact = true;
        for (std::size_t i = 9UZ; i < ramp.size(); ++i) {
            exact = exact && passed.samples[i] == ramp[i - 9UZ];
        }
        expect(that % exact) << "disabled is the input delayed by delay_samples, bit for bit";

        // A mid-stream `enabled` change keeps the alignment. Every sample comes out once, in order.
        NoiseBlanker<CF>  toggled = make<CF>({{"sample_rate", kRate}, {"threshold_db", 60.0}});
        std::size_t       calls   = 0UZ;
        std::vector<CF>   out;
        const std::size_t stride = 137UZ;
        for (std::size_t base = 0UZ; base < ramp.size(); base += stride) {
            if (calls == 5UZ) {
                apply(toggled, {{"enabled", true}});
            } else if (calls == 17UZ) {
                apply(toggled, {{"enabled", false}});
            }
            ++calls;
            const std::size_t      count = std::min(stride, ramp.size() - base);
            const test::Result<CF> piece = test::run(toggled, std::span<const CF>(ramp).subspan(base, count), count);
            out.insert(out.end(), piece.samples.begin(), piece.samples.end());
        }
        expect(eq(out.size(), ramp.size())) << "a toggle does not change the number of output samples";
        bool continuous = true;
        for (std::size_t i = 9UZ; i < ramp.size(); ++i) {
            continuous = continuous && out[i] == ramp[i - 9UZ];
        }
        expect(that % continuous) << "and the ramp is continuous across it";
    };

    "the threshold is a power ratio and the conversion from decibels is exact"_test = [] {
        for (const double db : {0.0, 3.0, 9.32, 20.0}) {
            NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"threshold_db", db}});
            expect(approx(block._threshold, std::pow(10.0, db / 10.0), std::pow(10.0, db / 10.0) * 1e-9)) << std::format("threshold_db {}", db);

            // The marked sample reaches the output delay_samples later. The probe is therefore followed by enough
            // silence to flush the line. Silence is below any threshold and opens no window of its own.
            const auto fires = [db](double scale) {
                NoiseBlanker<CF> probe = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"threshold_db", db}, {"emit_tags", true}});
                probe._warmup          = 0UZ;
                probe._power           = 1.0;
                std::vector<CF> one(32UZ, CF{}); // the probe is the first sample, so p_bar is still exactly what was set
                one[0UZ] = CF(static_cast<float>(std::sqrt(probe._threshold * scale)), 0.f);
                return test::run(probe, std::span<const CF>(one), 32UZ).tags.size() == 1UZ;
            };
            expect(!fires(1.0 - 1e-5)) << std::format("threshold_db {}: just below T * p_bar does not trigger", db);
            expect(fires(1.0 + 1e-5)) << std::format("threshold_db {}: just above it does", db);
        }

        // Exactly at the threshold, in a case binary arithmetic represents exactly, with T = 4, p_bar = 1 and |x| = 2.
        NoiseBlanker<CF> exactly = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"threshold_db", 10.0 * std::log10(4.0)}, {"emit_tags", true}});
        exactly._warmup          = 0UZ;
        exactly._power           = 1.0;
        expect(eq(exactly._threshold, 4.0));
        std::vector<CF> atThreshold(32UZ, CF{});
        atThreshold[0UZ] = CF(2.f, 0.f);
        expect(eq(test::run(exactly, std::span<const CF>(atThreshold), 32UZ).tags.size(), 0UZ)) << "a sample at exactly T * p_bar does not trigger: the comparison is strict";

        // A magnitude threshold k against the mean magnitude of noise of power p_bar: |x| > k * E|x|, where
        // E|x| = sqrt(pi * p_bar) / 2. A block built from the helper's value fires at that magnitude.
        for (const double k : {2.5, 3.3}) {
            const auto firesAt = [k](double scale) {
                NoiseBlanker<CF> probe = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"threshold_db", thresholdFromMagnitudeRatio(k)}, {"emit_tags", true}});
                probe._warmup          = 0UZ;
                probe._power           = 1.0;
                std::vector<CF> one(32UZ, CF{});
                one[0UZ] = CF(static_cast<float>(k * std::sqrt(kPi) / 2.0 * scale), 0.f);
                return test::run(probe, std::span<const CF>(one), 32UZ).tags.size() == 1UZ;
            };
            expect(!firesAt(1.0 - 1e-3)) << std::format("k = {}: just below k times the mean magnitude does not trigger", k);
            expect(firesAt(1.0 + 1e-3)) << std::format("k = {}: just above it does", k);
        }
    };

    "the false-blank rate is the exponential tail"_test = [] {
        const std::size_t     samples    = longTestsEnabled() ? 4000000UZ : 400000UZ;
        std::vector<double>   thresholds = longTestsEnabled() ? std::vector<double>{6.0, 8.0, 9.32} : std::vector<double>{6.0};
        constexpr std::size_t kWarmUp    = 120000UZ;

        for (const double db : thresholds) {
            NoiseBlanker<CF>      block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"averaging_time", 100000.0 / static_cast<double>(kRate)}, {"threshold_db", db}, {"emit_tags", true}});
            const std::vector<CF> warm  = noise(kWarmUp, 0x13198a2e03707344ULL);
            std::ignore                 = test::run(block, std::span<const CF>(warm), 4096UZ);
            for (std::size_t pass = 0UZ; pass < 5UZ; ++pass) { // let the censored tracker reach its own fixed point
                std::ignore = test::run(block, std::span<const CF>(noise(kWarmUp, 0x452821e638d01377ULL + pass)), 4096UZ);
            }

            const std::vector<CF>  x   = noise(samples, 0xa4093822299f31d0ULL + static_cast<std::uint64_t>(db));
            const test::Result<CF> got = test::run(block, std::span<const CF>(x), 4096UZ);

            // The tail is tested against the reference the block holds. That is the exact statement of
            // P(false) = exp(-T * p_bar). The second assertion places that reference below one. A censored tracker
            // does not learn from the samples it rejected. Taking exp(-T) alone is right at the default 9.32 dB
            // (0.17 % apart) and 30 % out at 6 dB.
            // Two triggers less than four samples apart leave one contiguous replaced run and one marker. The
            // first-order merge term below accounts for that.
            const auto [modeled, modeledRate] = censoredRunRate(db, 7UZ);
            const double perSample            = std::exp(-std::pow(10.0, db / 10.0) * block._power);
            const double wantRate             = perSample / (1.0 + perSample * 6.0) * (1.0 - 3.0 * perSample);
            const double gotRate              = static_cast<double>(got.tags.size()) / static_cast<double>(samples);
            const double error                = std::sqrt(wantRate / static_cast<double>(samples));
            std::println("threshold {:.2f} dB: {} runs, rate {:.6e} against {:.6e} at a tracker of {:.4f} (model {:.4f}); exp(-T) alone is {:.6e}", db, got.tags.size(), gotRate, wantRate, block._power, modeled, std::exp(-std::pow(10.0, db / 10.0)));
            expect(lt(std::abs(gotRate - wantRate), 5.0 * error)) << std::format("threshold_db {}: the trigger rate is the exponential tail of |x|^2 against the tracked reference", db);
            expect(lt(block._power, 1.0)) << "and the reference sits below the true power, which is what censoring costs";
            expect(gt(block._power, 0.5)) << "but not far below it";
            std::ignore = modeledRate;
        }
    };

    "censoring keeps the reference where the noise is"_test = [] {
        constexpr std::size_t kSamples = 200000UZ;
        constexpr std::size_t kPeriod  = 50UZ;
        constexpr double      kInr     = 1000.0; // 30 dB

        std::vector<CF> x = noise(kSamples, 0x082efa98ec4e6c89ULL);
        for (std::size_t at = kPeriod; at < kSamples; at += kPeriod) {
            x[at] += CF(static_cast<float>(std::sqrt(kInr)), 0.f);
        }

        NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"averaging_time", 1000.0 / static_cast<double>(kRate)}});
        std::ignore            = test::run(block, std::span<const CF>(x), 4096UZ);

        double       uncensored = 0.0; // the desensitizing order, which updates from every sample and then compares
        const double alpha      = block._alpha;
        for (const CF& sample : x) {
            const double power = static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
            uncensored += alpha * (power - uncensored);
        }

        std::println("period 50, INR 30 dB: censored {:.4f}, uncensored {:.4f}", block._power, uncensored);
        expect(approx(block._power, 1.0, 0.05)) << "the censored tracker settles on the noise, which is what the threshold is taken against";
        expect(gt(uncensored, 15.0)) << "where an uncensored one is dragged up by the events it is meant to detect";
    };

    "the warm-up blanks nothing and then settles on the budget"_test = [] {
        constexpr std::size_t kTau  = 1000UZ;
        NoiseBlanker<CF>      block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"averaging_time", static_cast<double>(kTau) / static_cast<double>(kRate)}, {"emit_tags", true}});

        std::vector<CF> loud = noise(kTau, 0x2ff24fa2ee84e39cULL);
        for (CF& sample : loud) {
            sample *= 100.f; // loud enough that a fixed reference of 1.0 would blank nearly every sample
        }
        expect(eq(test::run(block, std::span<const CF>(loud), 128UZ).tags.size(), 0UZ)) << "not one blanked sample in the first time constant, whatever the level";

        // The tracker still climbs for a few time constants after the warm-up ends. The settled duty is read after
        // that climb. The cost of the warm-up is one time constant of no protection, plus the few more during which
        // the reference still climbs.
        std::vector<CF> settling = noise(10UZ * kTau, 0x452821e638d01377ULL);
        std::vector<CF> after    = noise(10UZ * kTau, 0xbe5466cf34e90c6cULL);
        for (CF& sample : settling) {
            sample *= 100.f;
        }
        for (CF& sample : after) {
            sample *= 100.f;
        }
        std::ignore                    = test::run(block, std::span<const CF>(settling), 128UZ);
        const test::Result<CF> got     = test::run(block, std::span<const CF>(after), 128UZ);
        const double           blanked = static_cast<double>(got.tags.size()) * 9.0 / static_cast<double>(after.size());
        const double           want    = std::exp(-std::pow(10.0, 9.32 / 10.0)) * 9.0;
        std::println("after the warm-up: blanked {:.5f} of the stream against a budget of {:.5f}", blanked, want);
        expect(lt(blanked, 3.0 * want)) << "and afterwards the duty is the budget, within a factor of three";
        expect(gt(blanked, want / 3.0)) << "from either side: the blanker still fires after the warm-up";
    };

    "detection probability follows the non-central chi-square"_test = [] {
        const std::size_t     kImpulses = longTestsEnabled() ? 100000UZ : 30000UZ;
        constexpr std::size_t kSpacing  = 20UZ;

        for (const double inrDb : {6.0, 10.0, 13.0}) {
            const double want = detectionProbability(std::pow(10.0, inrDb / 10.0), std::pow(10.0, 9.32 / 10.0));
            // The stream runs nine samples past the last impulse. Each impulse is looked for in the output, which is
            // the input delayed by nine.
            const double      amplitude = std::sqrt(std::pow(10.0, inrDb / 10.0));
            const std::size_t count     = kImpulses * kSpacing;
            std::vector<CF>   x         = noise(count + 9UZ, 0x9216d5d98979fb1bULL + static_cast<std::uint64_t>(inrDb));
            for (std::size_t k = 1UZ; k <= kImpulses; ++k) {
                const std::size_t at = k * kSpacing - 1UZ;
                x[at] += CF(static_cast<float>(amplitude), 0.f);
            }

            // The reference is held exact. A 5 % duty of impulses would otherwise raise it and cost detection. That
            // effect belongs to the scene and not to the statistic under test here.
            NoiseBlanker<CF> block     = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"averaging_time", 1e8 / static_cast<double>(kRate)}, {"replacement", std::string("zero")}});
            block._warmup              = 0UZ;
            block._power               = 1.0;
            const test::Result<CF> got = test::run(block, std::span<const CF>(x), 4096UZ);

            // Counts whether the planted sample was replaced. Counting run starts instead
            // undercounts by every window that met its neighbor, which at 6 dB is a sixth of them.
            std::size_t caught = 0UZ;
            std::size_t direct = 0UZ;
            for (std::size_t k = 1UZ; k <= kImpulses; ++k) {
                const std::size_t at    = k * kSpacing - 1UZ;
                const double      power = static_cast<double>(x[at].real()) * static_cast<double>(x[at].real()) + static_cast<double>(x[at].imag()) * static_cast<double>(x[at].imag());
                caught += got.samples[at + 9UZ] == CF{} ? 1UZ : 0UZ;
                direct += power > block._threshold ? 1UZ : 0UZ;
            }
            const double measured = static_cast<double>(caught) / static_cast<double>(kImpulses);
            const double error    = std::sqrt(want * (1.0 - want) / static_cast<double>(kImpulses));
            std::println("INR {:.0f} dB: blanked {:.5f}, statistic over threshold {:.5f}, closed form {:.5f}", inrDb, measured, static_cast<double>(direct) / static_cast<double>(kImpulses), want);
            expect(lt(std::abs(measured - want), 4.0 * error)) << std::format("INR {} dB against the non-central chi-square", inrDb);
            expect(ge(caught, direct)) << "and every sample whose statistic exceeds the threshold is replaced";
        }
    };

    "interpolation beats holding beats zeroing, by the measured margins"_test = [] {
        constexpr std::size_t kLength    = 16384UZ;
        constexpr double      kBandwidth = 0.01;
        constexpr double      kIsr       = 1000.0; // 30 dB in power

        const std::vector<CF> clean = bandLimited(kLength, kBandwidth, 0x3f84d5b5b5470917ULL);
        const double          rms   = std::sqrt(meanPower(std::span<const CF>(clean)));

        std::vector<CF> dirty = clean;
        for (std::size_t at = 2048UZ; at < kLength; at += 4096UZ) {
            dirty[at] += CF(static_cast<float>(rms * std::sqrt(kIsr)), 0.f);
        }

        struct Arm {
            const char* rule;
            double      bound;
        };
        // Each rule has a bound with margin for the generator, and interpolation beats zeroing by 40 dB.
        constexpr Arm       kArms[]{{"interpolate", -80.0}, {"hold", -55.0}, {"zero", -33.0}};
        std::vector<double> residual;

        for (const auto& [rule, bound] : kArms) {
            NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"averaging_time", 100000.0 / static_cast<double>(kRate)}, {"replacement", std::string(rule)}});
            block._warmup          = 0UZ;
            block._power           = rms * rms;

            const test::Result<CF> got = test::run(block, std::span<const CF>(dirty), 4096UZ);
            std::vector<CF>        error(kLength - 9UZ);
            for (std::size_t i = 0UZ; i < error.size(); ++i) {
                error[i] = got.samples[i + 9UZ] - clean[i];
            }
            const double db = 10.0 * std::log10(inBandEnergy(std::span<const CF>(error), kBandwidth) / inBandEnergy(std::span<const CF>(clean), kBandwidth));
            residual.push_back(db);
            std::println("{:<12} in-channel residual {:.2f} dB", rule, db);
            expect(lt(db, bound)) << std::format("{} leaves {:.2f} dB", rule, db);
        }
        expect(gt(residual[2UZ] - residual[0UZ], 40.0)) << "and interpolation beats zeroing by more than 40 dB";
    };

    "interpolation is exact on a straight line"_test = [] {
        constexpr std::size_t kLength = 400UZ;
        constexpr std::size_t kAt     = 200UZ;
        std::vector<CF>       x(kLength);
        for (std::size_t i = 0UZ; i < kLength; ++i) {
            x[i] = CF(1.f + 0.01f * static_cast<float>(i), -2.f + 0.03f * static_cast<float>(i));
        }
        std::vector<CF> planted = x;
        planted[kAt] += CF(500.f, 0.f);

        NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}});
        block._warmup          = 0UZ;
        block._power           = 20.0;

        const test::Result<CF> got = test::run(block, std::span<const CF>(planted), 64UZ);
        for (std::size_t i = kAt - 2UZ; i <= kAt + 6UZ; ++i) {
            const CF     made     = got.samples[i + 9UZ];
            const double relative = std::abs(static_cast<double>(made.real()) - static_cast<double>(x[i].real())) / std::abs(static_cast<double>(x[i].real()));
            expect(lt(relative, 1e-6)) << std::format("input {}: interpolating across the gap reproduces the line exactly, so the (j+1)/(W+1) weights are right", i);
        }
    };

    "the replaced set is exactly the nine samples the window names"_test = [] {
        constexpr std::size_t kAt = 500UZ;
        std::vector<CF>       x   = tone(2000UZ, 1.0);
        x[kAt] += CF(200.f, 0.f);

        NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"replacement", std::string("zero")}, {"emit_tags", true}});
        block._warmup          = 0UZ;
        block._power           = 1.0;

        const test::Result<CF>   got = test::run(block, std::span<const CF>(x), 251UZ);
        std::vector<std::size_t> replaced;
        for (std::size_t i = 9UZ; i < got.samples.size(); ++i) {
            if (got.samples[i] == CF{}) {
                replaced.push_back(i - 9UZ); // back to the input offset it came from
            }
        }
        std::vector<std::size_t> want;
        for (std::size_t i = kAt - 2UZ; i <= kAt + 6UZ; ++i) {
            want.push_back(i);
        }
        expect(that % (replaced == want)) << "inputs q-2 .. q+6, nine of them, and no others";
        expect(eq(got.tags.size(), 1UZ)) << "one marker for the run";
        if (!got.tags.empty()) {
            expect(eq(got.tags[0UZ].index, kAt + 6UZ + 9UZ)) << "on the run's last replaced sample";
            expect(got.tags[0UZ].map.at(gr::property_map::key_type{"noise_blanked"}) == gr::pmt::Value(gr::Size_t{9U})) << "stating its length";
        }
    };

    "a retriggering window is bounded and reopens"_test = [] {
        constexpr std::size_t kLength = 2000UZ;
        constexpr std::size_t kStart  = 500UZ;
        constexpr std::size_t kBurst  = 200UZ;

        std::vector<CF> x = tone(kLength, 1.0);
        for (std::size_t i = kStart; i < kStart + kBurst; ++i) {
            x[i] += CF(200.f, 0.f);
        }
        x[1500UZ] += CF(200.f, 0.f); // one more, well after the burst

        NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"retrigger", true}, {"replacement", std::string("zero")}, {"emit_tags", true}});
        block._warmup          = 0UZ;
        block._power           = 1.0;

        // Sample by sample, so the bound can be read at every point and need not be inferred from the output.
        std::uint32_t highest = 0U;
        std::size_t   made    = 0UZ;
        for (std::size_t i = 0UZ; i < kLength; ++i) {
            made += test::run(block, std::span<const CF>(x).subspan(i, 1UZ), 1UZ).samples.size();
            highest = std::max(highest, block._windowLength);
        }
        expect(eq(made, kLength)) << "the block never stops producing";
        expect(eq(highest, 28U)) << "a retriggering window stops extending at exactly max_window_samples, 4 * blank_samples";

        NoiseBlanker<CF> again     = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"retrigger", true}, {"replacement", std::string("zero")}, {"emit_tags", true}});
        again._warmup              = 0UZ;
        again._power               = 1.0;
        const test::Result<CF> got = test::run(again, std::span<const CF>(x), 97UZ);
        expect(eq(got.tags.size(), 2UZ)) << "the burst is one replaced run and the later impulse opens a second: a closed window reopens on the next detection";

        std::size_t longest = 0UZ;
        std::size_t run     = 0UZ;
        for (std::size_t i = 9UZ; i < got.samples.size(); ++i) {
            run     = got.samples[i] == CF{} ? run + 1UZ : 0UZ;
            longest = std::max(longest, run);
        }
        expect(le(longest, kBurst + 9UZ)) << "and a sustained burst is replaced for as long as it lasts and no longer";
    };

    "the output does not depend on the chunking"_test = [] {
        std::vector<CF> x = noise(60000UZ, 0xba3bbe38fb4c55d9ULL);
        for (std::size_t at = 777UZ; at < x.size(); at += 1013UZ) {
            x[at] += CF(60.f, 20.f);
        }

        for (const char* rule : {"interpolate", "hold", "zero"}) {
            NoiseBlanker<CF>       reference = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"replacement", std::string(rule)}});
            const test::Result<CF> want      = test::run(reference, std::span<const CF>(x), 4096UZ);

            for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 4096UZ}) {
                NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"replacement", std::string(rule)}});
                expect(that % (test::run(block, std::span<const CF>(x), chunk).samples == want.samples)) << std::format("{}, chunk {}", rule, chunk);
            }
        }
    };

    "reserved keys move with their samples, and the private one does not survive"_test = [] {
        struct Marker {
            const char*    key;
            std::size_t    at;
            gr::pmt::Value value;
        };
        const std::array<Marker, 7UZ> markers{{
            {"trigger_name", 0UZ, gr::pmt::Value(std::string("alpha"))},
            {"trigger_time", 1UZ, gr::pmt::Value(std::uint64_t{111})},
            {"trigger_offset", 1UZ, gr::pmt::Value(0.5f)},
            {"num_channels", 37UZ, gr::pmt::Value(gr::Size_t{3})},
            {"rx_overflow", 512UZ, gr::pmt::Value(true)},
            {"signal_name", 900UZ, gr::pmt::Value(std::string("nb"))},
            {"t0", 1200UZ, gr::pmt::Value(std::string("private"))},
        }};

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 4000U}, {"mark_tag", false}});
        for (const Marker& marker : markers) {
            source._tags.emplace_back(marker.at, gr::property_map{{gr::property_map::key_type{marker.key}, marker.value}});
        }
        auto& block = graph.emplaceBlock<NoiseBlanker<CF>>({{"sample_rate", kRate}});
        auto& sink  = graph.emplaceBlock<TagSink<CF, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});
        expect(graph.connect<"out", "in">(source, block).has_value());
        expect(graph.connect<"out", "in">(block, sink).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        constexpr std::size_t    kMissing = std::numeric_limits<std::size_t>::max();
        std::vector<std::size_t> offsets(markers.size(), kMissing);
        for (const gr::Tag& receivedTag : sink._tags) {
            for (std::size_t which = 0UZ; which < markers.size(); ++which) {
                const auto found = receivedTag.map.find(gr::property_map::key_type{markers[which].key});
                if (found != receivedTag.map.end() && found->second == markers[which].value) {
                    offsets[which] = receivedTag.index;
                }
            }
        }
        for (std::size_t which = 0UZ; which + 1UZ < markers.size(); ++which) {
            expect(eq(offsets[which], markers[which].at + 9UZ)) << std::format("{} moves by delay_samples with its sample", markers[which].key);
        }
        expect(eq(offsets[markers.size() - 1UZ], kMissing)) << "and the non-reserved key does not survive, which is the framework's rule and not this block's";
    };

    "degenerate settings"_test = [] {
        expect(throws([] { std::ignore = make<CF>({{"blank_samples", 0U}}); }));
        expect(throws([] { std::ignore = make<CF>({{"sample_rate", 0.f}}); }));
        expect(throws([] { std::ignore = make<CF>({{"sample_rate", -1.f}}); }));
        expect(throws([] { std::ignore = make<CF>({{"averaging_time", -1.0}}); }));
        expect(throws([] { std::ignore = make<CF>({{"replacement", std::string("taper")}}); })) << "'taper' is not one of the accepted replacement rules";

        NoiseBlanker<CF> block = make<CF>({{"sample_rate", kRate}});
        std::ignore            = block.settings().setStaged({{"blank_samples", 0U}});
        const auto staged      = block.settings().applyStagedParameters();
        expect(staged.failedParameters.contains(gr::property_map::key_type{"blank_samples"})) << "the limits refuse a live change too";
        expect(eq(block.delay_samples.value, 9U)) << "and the line keeps its accepted length";

        const double pole = block.alpha.value;
        expect(throws([&block] { apply(block, {{"averaging_time", 1e12}}); })) << "a time constant whose pole rounds to zero is refused";
        expect(eq(block.alpha.value, pole)) << "and alpha keeps the resolved pole";
        expect(nothrow([&block] { apply(block, {{"threshold_db", 12.0}}); })) << "the next application passes";
        expect(eq(block._threshold, std::pow(10.0, 1.2))) << "and the block takes its value";

        NoiseBlanker<CF>       silly = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"threshold_db", -60.0}});
        const std::vector<CF>  x     = noise(20000UZ, 0x21c66842f6e96c9aULL);
        const test::Result<CF> got   = test::run(silly, std::span<const CF>(x), 512UZ);
        expect(eq(got.samples.size(), x.size())) << "a threshold of -60 dB blanks essentially everything and the block still produces one output per input";

        NoiseBlanker<float>      real = make<float>({{"enabled", true}, {"sample_rate", kRate}});
        const std::vector<float> audio(5000UZ, 0.25f);
        expect(eq(test::run(real, std::span<const float>(audio), 333UZ).samples.size(), audio.size())) << "and the real port runs the same code on x^2";
    };

    "a level rise is replaced for one averaging time and then passes"_test = [] {
        constexpr std::size_t kSettle = 48000UZ;
        constexpr std::size_t kAfter  = 96000UZ;
        struct Scene {
            const char* what;
            double      carrier; // amplitude of a tone added from the change on
            std::size_t gap;     // zeros from the change on
            float       gain;    // noise amplitude after the gap
        };
        // After the noise rise, about one sample in twelve stays below the old threshold and breaks the replaced run.
        for (const auto& [what, carrier, gap, gain] : {Scene{"a carrier keys up 20 dB above the noise", 10.0, 0UZ, 1.f}, Scene{"the noise returns after 0.1 s of zeros", 0.0, 9600UZ, 1.f}, Scene{"the noise rises 20 dB", 0.0, 0UZ, 10.f}}) {
            std::vector<CF> x = noise(kSettle + kAfter, 0x5be0cd19137e2179ULL);
            std::fill_n(x.begin() + static_cast<std::ptrdiff_t>(kSettle), gap, CF{});
            const std::size_t     resume = kSettle + gap;
            const std::vector<CF> added  = tone(x.size() - resume, carrier);
            for (std::size_t i = resume; i < x.size(); ++i) {
                x[i] = x[i] * gain + added[i - resume];
            }
            const double             scenePower = carrier * carrier + static_cast<double>(gain * gain);
            std::vector<std::size_t> impulses; // 30 dB above the scene, from 0.1 s after the change on
            for (std::size_t at = resume + 9600UZ; at + 20UZ < x.size(); at += 997UZ) {
                x[at] += CF(static_cast<float>(std::sqrt(1000.0 * scenePower)), 0.f);
                impulses.push_back(at);
            }

            NoiseBlanker<CF>       block    = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"replacement", std::string("zero")}});
            const test::Result<CF> got      = test::run(block, std::span<const CF>(x), 4096UZ);
            const auto             replaced = [&got](std::size_t from, std::size_t to) {
                const auto zero = std::ranges::count(std::span<const CF>(got.samples).subspan(from + 9UZ, to - from), CF{});
                return static_cast<double>(zero) / static_cast<double>(to - from);
            };
            std::size_t caught = 0UZ;
            for (const std::size_t at : impulses) {
                caught += got.samples[at + 9UZ] == CF{} ? 1UZ : 0UZ;
            }
            std::println("{}: replaced {:.4f} in the first 9 ms, {:.4f} from 0.1 s on; {} of {} impulses; tracker {:.4g}", what, replaced(resume, resume + 864UZ), replaced(resume + 9600UZ, x.size() - 9UZ), caught, impulses.size(), block._power);
            expect(gt(replaced(resume, resume + 864UZ), 0.99)) << what << ": the rise is replaced while it is younger than one averaging time";
            expect(lt(replaced(resume + 9600UZ, x.size() - 9UZ), 0.05)) << what << ": then the tracker has learned the level and the scene passes";
            expect(eq(caught, impulses.size())) << what << ": and every impulse above the new level is replaced";
            expect(gt(block._power, 0.5 * scenePower)) << what << ": the tracker sits at the new level";
        }
    };

    "a run longer than the delay line holds, then interpolates, and its marker states its whole length"_test = [] {
        constexpr std::size_t kAt   = 500UZ;
        const std::vector<CF> clean = tone(2000UZ, 1.0);
        for (const char* rule : {"interpolate", "hold", "zero"}) {
            // A burst of b samples at +46 dB with retrigger off: windows of 7 follow one another while the burst lasts.
            for (const auto& [burst, run] : {std::pair{1UZ, 9UZ}, std::pair{12UZ, 16UZ}, std::pair{20UZ, 23UZ}, std::pair{40UZ, 44UZ}}) {
                std::vector<CF> x = clean;
                for (std::size_t i = kAt; i < kAt + burst; ++i) {
                    x[i] += CF(200.f, 0.f);
                }
                NoiseBlanker<CF> block     = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"replacement", std::string(rule)}, {"emit_tags", true}});
                block._warmup              = 0UZ;
                block._power               = 1.0;
                const test::Result<CF> got = test::run(block, std::span<const CF>(x), 251UZ);

                double worst = 0.0;
                for (std::size_t i = kAt; i < kAt + burst; ++i) {
                    worst = std::max(worst, static_cast<double>(std::abs(got.samples[i + 9UZ] - clean[i])));
                }
                expect(le(worst, 2.0)) << std::format("{}, burst {}: no output on the burst strays further than the tone's diameter, got {:.3f}", rule, burst, worst);
                expect(eq(got.tags.size(), 1UZ)) << std::format("{}, burst {}: one run, one marker", rule, burst);
                if (!got.tags.empty()) {
                    expect(eq(got.tags[0UZ].index, kAt - 2UZ + run - 1UZ + 9UZ)) << std::format("{}, burst {}: on the run's last replaced sample", rule, burst);
                    expect(got.tags[0UZ].map.at(gr::property_map::key_type{"noise_blanked"}) == gr::pmt::Value(static_cast<gr::Size_t>(run))) << std::format("{}, burst {}: stating the run's whole length, {}", rule, burst, run);
                }
            }
        }
    };

    "a sample whose power is not finite is replaced and leaves the tracker alone"_test = [] {
        std::vector<CF> x = noise(60000UZ, 0x3c6ef372fe94f82bULL);
        x[100UZ]          = CF(std::numeric_limits<float>::quiet_NaN(), 0.f); // inside the warm-up
        x[20000UZ]        = CF(std::numeric_limits<float>::quiet_NaN(), 0.f);
        x[30000UZ]        = CF(0.f, std::numeric_limits<float>::infinity());
        std::vector<std::size_t> impulses;
        for (std::size_t at = 21000UZ; at < 59000UZ; at += 1000UZ) {
            x[at] += CF(31.6f, 0.f);
            impulses.push_back(at);
        }

        NoiseBlanker<CF>       block  = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"replacement", std::string("zero")}});
        const test::Result<CF> got    = test::run(block, std::span<const CF>(x), 4096UZ);
        std::size_t            caught = 0UZ;
        for (const std::size_t at : impulses) {
            caught += got.samples[at + 9UZ] == CF{} ? 1UZ : 0UZ;
        }
        const auto notFinite = [](CF sample) { return !std::isfinite(sample.real()) || !std::isfinite(sample.imag()); };
        expect(eq(std::ranges::count_if(got.samples, notFinite), std::ptrdiff_t{0})) << "each sample that is not finite is replaced";
        expect(eq(caught, impulses.size())) << "and detection goes on after them";
        expect(approx(block._power, 1.0, 0.2)) << "the tracker never took one in";
    };

    "a batch the block refuses changes nothing it computes, and the limits refuse a NaN alone"_test = [] {
        const std::vector<CF> x     = noise(30000UZ, 0x9b05688c2b3e6c1fULL);
        NoiseBlanker<CF>      block = make<CF>({{"sample_rate", kRate}, {"replacement", std::string("zero")}});
        std::ignore                 = test::run(block, std::span<const CF>(x).first(5000UZ), 4096UZ);

        expect(throws([&block] { apply(block, {{"enabled", true}, {"replacement", std::string("taper")}}); })) << "an unknown rule refuses the whole batch";
        const std::optional<gr::pmt::Value> held = block.settings().get("enabled");
        const bool*                         kept = held.has_value() ? held->get_if<bool>() : nullptr;
        expect(kept != nullptr && *kept) << "the refused value stays in the settings";
        expect(nothrow([&block] { apply(block, {{"enabled", true}}); })) << "sent again alone it equals the member and passes unseen";
        for (int attempt = 0; attempt < 2; ++attempt) {
            std::ignore       = block.settings().setStaged({{"threshold_db", std::numeric_limits<double>::quiet_NaN()}});
            const auto staged = block.settings().applyStagedParameters();
            expect(staged.failedParameters.contains(gr::property_map::key_type{"threshold_db"})) << "a NaN is outside every limit, each time it is sent";
            expect(eq(block.threshold_db.value, 9.32)) << "and never reaches the member";
        }

        const test::Result<CF> got     = test::run(block, std::span<const CF>(x).subspan(5000UZ), 4096UZ);
        bool                   delayed = true;
        for (std::size_t i = 0UZ; i < got.samples.size(); ++i) {
            delayed = delayed && got.samples[i] == x[5000UZ + i - 9UZ];
        }
        expect(that % delayed) << "the block stays disabled: the output is the input delayed, bit for bit";
    };

    "a value outside its limits on a tag is refused, the run completes, and the block keeps its delay"_test = [] {
        static constexpr std::size_t kLength = 4000UZ;
        static constexpr std::size_t kAt     = 2000UZ;
        // A tag does not change a key named in the constructor's map. The map leaves out the keys the tag sets.
        const auto runWith = [](gr::property_map change) {
            gr::Graph graph;
            auto&     source = graph.emplaceBlock<TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(kLength)}, {"mark_tag", false}});
            source._tags.emplace_back(kAt, std::move(change));
            auto& block = graph.emplaceBlock<NoiseBlanker<CF>>({{"sample_rate", kRate}});
            auto& sink  = graph.emplaceBlock<TagSink<CF, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});
            boost::ut::expect(graph.connect<"out", "in">(source, block).has_value());
            boost::ut::expect(graph.connect<"out", "in">(block, sink).has_value());

            gr::scheduler::Simple scheduler;
            boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
            const bool      ran = scheduler.runAndWait().has_value();
            std::vector<CF> out;
            for (std::size_t i = 0UZ; i < sink._samples.size(); ++i) {
                out.push_back(sink._samples[i]);
            }
            return std::tuple{ran, out, block.blank_samples.value, block.threshold_db.value, block.delay_samples.value};
        };
        // The source counts: input i is i. Output n is n less the delay in force.
        const auto delayedBy = [](const std::vector<CF>& out, std::size_t from, std::size_t to, std::size_t delay) {
            bool exact = out.size() >= to;
            for (std::size_t n = from; exact && n < to; ++n) {
                exact = out[n] == CF(static_cast<float>(n - delay), 0.f);
            }
            return exact;
        };

        for (const gr::property_map& change : {gr::property_map{{"blank_samples", gr::Size_t{0U}}}, gr::property_map{{"threshold_db", std::numeric_limits<double>::quiet_NaN()}}}) {
            const auto& key                                = change.begin()->first;
            const auto [ran, out, blank, threshold, delay] = runWith(change);
            expect(ran) << std::format("{}: the refusal on a tag is a plain message, and the run completes", key);
            expect(eq(out.size(), kLength)) << std::format("{}: every sample comes out", key);
            expect(that % delayedBy(out, 9UZ, kLength, 9UZ)) << std::format("{}: the output is the input delayed by the accepted nine samples, after the tag too", key);
            expect(eq(blank, 7U)) << std::format("{}: the members keep the accepted values", key);
            expect(eq(threshold, 9.32));
            expect(eq(delay, 9U));
        }

        const auto [ran, out, blank, threshold, delay] = runWith(gr::property_map{{"blank_samples", gr::Size_t{3U}}});
        expect(ran);
        expect(eq(blank, 3U)) << "a value inside the limits applies";
        expect(eq(delay, 5U));
        expect(eq(threshold, 9.32));
        expect(that % delayedBy(out, 9UZ, kAt, 9UZ)) << "the input delayed by nine before the tag";
        expect(that % delayedBy(out, kAt, kLength, 5UZ)) << "and by five from the tag's sample on";
    };

    "a change of the delay keeps the newest inputs and moves their tags with them"_test = [] {
        constexpr std::size_t kLength = 4000UZ;
        constexpr std::size_t kPiece  = 137UZ;
        std::vector<CF>       ramp(kLength);
        for (std::size_t i = 0UZ; i < kLength; ++i) {
            ramp[i] = CF(static_cast<float>(i + 1UZ), 0.f);
        }
        const auto tagged = [](const char* key) { return gr::property_map{{gr::property_map::key_type{key}, gr::pmt::Value(std::string("x"))}}; };

        NoiseBlanker<CF>                                 block = make<CF>({{"sample_rate", kRate}});
        std::deque<CF>                                   line(9UZ, CF{}); // the model: the inputs still to leave, oldest first
        std::vector<CF>                                  want;
        std::vector<CF>                                  got;
        std::vector<std::pair<std::string, std::size_t>> tags;
        for (std::size_t base = 0UZ, piece = 0UZ; base < kLength; base += kPiece, ++piece) {
            if (piece == 5UZ || piece == 17UZ) {
                const gr::Size_t blank = piece == 5UZ ? 14U : 3U;
                apply(block, {{"blank_samples", blank}});
                while (line.size() < 2UZ + blank) {
                    line.push_front(CF{});
                }
                while (line.size() > 2UZ + blank) {
                    line.pop_front();
                }
            }
            const std::size_t    count = std::min(kPiece, kLength - base);
            std::vector<gr::Tag> marks;
            if (piece == 4UZ) {
                marks.emplace_back(count - 1UZ, tagged("trigger_name")); // in the line when it grows
            } else if (piece == 16UZ) {
                marks.emplace_back(count - 16UZ, tagged("signal_name")); // the oldest of sixteen waiting inputs, dropped when the line shrinks
                marks.emplace_back(count - 1UZ, tagged("trigger_name")); // the newest, kept
            }
            const test::Result<CF> result = test::run(block, std::span<const CF>(ramp).subspan(base, count), count, std::span<const gr::Tag>(marks));
            for (const gr::Tag& each : result.tags) {
                tags.emplace_back(std::string(each.map.begin()->first), got.size() + each.index);
            }
            got.insert(got.end(), result.samples.begin(), result.samples.end());
            for (std::size_t i = 0UZ; i < count; ++i) {
                line.push_back(ramp[base + i]);
                want.push_back(line.front());
                line.pop_front();
            }
        }
        expect(that % (got == want)) << "zeros enter where the line grows, the oldest inputs drop out where it shrinks, and the rest come out once, in order";
        expect(eq(block.delay_samples.value, 5U));

        const auto                                             placeOf = [&got](std::size_t input) { return static_cast<std::size_t>(std::ranges::find(got, CF(static_cast<float>(input + 1UZ), 0.f)) - got.begin()); };
        const std::vector<std::pair<std::string, std::size_t>> expected{{"trigger_name", placeOf(5UZ * kPiece - 1UZ)}, {"signal_name", placeOf(17UZ * kPiece - 5UZ)}, {"trigger_name", placeOf(17UZ * kPiece - 1UZ)}};
        expect(that % (tags == expected)) << "each kept tag leaves with its sample, and the tag of a dropped sample leaves with the oldest kept one";
    };

    "a shorter line moves the tags and markers of the inputs it drops to the next sample out"_test = [] {
        constexpr std::size_t kAt    = 200UZ;      // the impulse, whose run is inputs 198 to 206
        constexpr std::size_t kFirst = kAt + 10UZ; // the first call ends with inputs 201 to 209 waiting in the line
        std::vector<CF>       x      = tone(400UZ, 1.0);
        x[kAt] += CF(200.f, 0.f);
        const auto tagged  = [](const char* key) { return gr::property_map{{gr::property_map::key_type{key}, gr::pmt::Value(std::string("x"))}}; };
        const auto blanker = [](const char* rule) {
            NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"replacement", std::string(rule)}, {"emit_tags", true}});
            block._warmup          = 0UZ;
            block._power           = 1.0;
            return block;
        };

        // From nine to one: the line drops inputs 201 to 208, the rest of the run among them.
        NoiseBlanker<CF>           shrunk = blanker("zero");
        const std::vector<gr::Tag> marks{{kAt + 2UZ, tagged("trigger_name")}, {kAt + 8UZ, tagged("signal_name")}, {kAt + 9UZ, tagged("trigger_time")}};
        const test::Result<CF>     before = test::run(shrunk, std::span<const CF>(x).first(kFirst), kFirst, std::span<const gr::Tag>(marks));
        apply(shrunk, {{"lookback_samples", 0U}, {"blank_samples", 1U}});
        const test::Result<CF> after = test::run(shrunk, std::span<const CF>(x).subspan(kFirst), 64UZ);
        expect(before.tags.empty()) << "the tags and the run's end are still in the line when it shrinks";
        std::vector<std::string> keys;
        for (const gr::Tag& each : after.tags) {
            expect(eq(each.index, 0UZ)) << "each tag of a dropped input leaves on the next sample out";
            keys.emplace_back(each.map.begin()->first);
        }
        expect(that % (keys == std::vector<std::string>{"trigger_name", "noise_blanked", "signal_name", "trigger_time"})) << "in the order of the samples they belong to, the kept sample's own tag last";
        if (keys.size() == 4UZ) {
            expect(after.tags[1UZ].map.at(gr::property_map::key_type{"noise_blanked"}) == gr::pmt::Value(gr::Size_t{9U})) << "the marker states the run's nine samples, the dropped ones counted";
        }
        expect(after.samples.front() == x[kAt + 9UZ]) << "the next sample out is the input after the dropped ones";
        const std::ptrdiff_t zeros = std::ranges::count(std::span<const CF>(before.samples).subspan(9UZ), CF{}) + std::ranges::count(after.samples, CF{});
        expect(eq(zeros, std::ptrdiff_t{3})) << "and after the line's first fill, the three replaced samples that left before the change are the only zeros";

        // From nine to seven: the line drops inputs 201 and 202 from inside the run, and the run's end stays in it.
        NoiseBlanker<CF>       reference = blanker("interpolate");
        const test::Result<CF> whole     = test::run(reference, std::span<const CF>(x), kFirst);
        NoiseBlanker<CF>       cut       = blanker("interpolate");
        std::ignore                      = test::run(cut, std::span<const CF>(x).first(kFirst), kFirst);
        apply(cut, {{"lookback_samples", 0U}});
        const test::Result<CF> rest = test::run(cut, std::span<const CF>(x).subspan(kFirst), 64UZ);
        bool                   same = true;
        for (std::size_t input = kAt + 3UZ; input <= kAt + 6UZ; ++input) {
            same = same && rest.samples[input + 7UZ - kFirst] == whole.samples[input + 9UZ];
        }
        expect(that % same) << "each kept sample of the run takes the value it takes with no change";
        expect(that % (rest.offsetsOf("noise_blanked") == std::vector<std::size_t>{kAt + 6UZ + 7UZ - kFirst})) << "the marker stays on the run's last sample";
        if (rest.tags.size() == 1UZ) {
            expect(rest.tags[0UZ].map.at(gr::property_map::key_type{"noise_blanked"}) == gr::pmt::Value(gr::Size_t{9U})) << "and states its nine samples, the dropped ones counted";
        }
    };

    "the tags held at the end of the stream leave at the end-of-stream index, in order"_test = [] {
        constexpr std::size_t kLength = 100UZ;
        std::vector<CF>       x       = tone(kLength, 1.0);
        x[95UZ] += CF(200.f, 0.f); // its run, inputs 93 to 99, is still in the line when the stream ends
        const auto                 tagged = [](const char* key) { return gr::property_map{{gr::property_map::key_type{key}, gr::pmt::Value(std::string("x"))}}; };
        const std::vector<gr::Tag> marks{{40UZ, tagged("trigger_name")}, {94UZ, tagged("signal_name")}};

        NoiseBlanker<CF> block     = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"replacement", std::string("zero")}, {"emit_tags", true}});
        block._warmup              = 0UZ;
        block._power               = 1.0;
        const test::Result<CF> got = test::run(block, std::span<const CF>(x), 37UZ, std::span<const gr::Tag>(marks));
        expect(that % (got.offsetsOf("trigger_name") == std::vector<std::size_t>{49UZ})) << "a tag whose sample leaves is published with it";
        expect(eq(got.tags.size(), 1UZ)) << "and nothing else left during the stream";

        // The framework's end-of-stream call: spans of no samples, offset 0 at the end-of-stream index.
        gr::property_map ending = tagged("rx_overflow");
        ending.insert_or_assign(static_cast<std::pmr::string>(gr::tag::END_OF_STREAM), gr::pmt::Value(true));
        const std::vector<gr::Tag> last{{kLength, ending}};
        std::vector<gr::Tag>       published;
        test::InputSpan<CF>        endIn(std::span<const CF>{}, kLength, std::span<const gr::Tag>(last));
        test::OutputSpan<CF>       endOut(std::span<CF>{}, kLength, &published);
        auto                       inSpans  = std::tie(endIn);
        auto                       outSpans = std::tie(endOut);
        block.forwardTags(inSpans, outSpans, 0UZ);

        std::vector<std::string> keys;
        for (const gr::Tag& each : published) {
            expect(eq(each.index, kLength)) << "every held tag sits at the end-of-stream index";
            expect(!each.map.contains(static_cast<std::pmr::string>(gr::tag::END_OF_STREAM))) << "and the end_of_stream key stays behind";
            keys.emplace_back(each.map.begin()->first);
        }
        expect(that % (keys == std::vector<std::string>{"signal_name", "noise_blanked", "rx_overflow"})) << "in the order of the samples they belong to";
        if (published.size() == 3UZ) {
            expect(published[1UZ].map.at(gr::property_map::key_type{"noise_blanked"}) == gr::pmt::Value(gr::Size_t{7U})) << "the run's marker states its seven samples";
        }
    };

    "a reset to defaults restarts the line and the tracker, and the line's tags and markers leave on the next sample out"_test = [] {
        constexpr std::size_t kAt    = 200UZ;      // the impulse, whose run is inputs 198 to 206
        constexpr std::size_t kFirst = kAt + 10UZ; // the first call ends with inputs 201 to 209 waiting in the line
        std::vector<CF>       x      = tone(400UZ, 1.0);
        x[kAt] += CF(200.f, 0.f);
        const auto tagged = [](const char* key) { return gr::property_map{{gr::property_map::key_type{key}, gr::pmt::Value(std::string("x"))}}; };

        NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"replacement", std::string("zero")}, {"emit_tags", true}});
        block.settings().storeDefaults(); // the defaults a graph stores after the constructor's map
        block._warmup = 0UZ;
        block._power  = 1.0;
        const std::vector<gr::Tag> marks{{kAt + 2UZ, tagged("trigger_name")}, {kAt + 8UZ, tagged("signal_name")}};
        const test::Result<CF>     before = test::run(block, std::span<const CF>(x).first(kFirst), kFirst, std::span<const gr::Tag>(marks));
        expect(before.tags.empty()) << "the tags and the run's end are still in the line at the reset";

        block.settings().resetDefaults();
        expect(eq(block._warmup, block._warmupLength)) << "the reset restarts the warm-up";
        expect(eq(block._power, 0.0)) << "and the tracked power";

        const test::Result<CF>   after = test::run(block, std::span<const CF>(x).subspan(kFirst), 64UZ);
        std::vector<std::string> keys;
        for (const gr::Tag& each : after.tags) {
            expect(eq(each.index, 0UZ)) << "each tag of an input in the line leaves on the next sample out";
            keys.emplace_back(each.map.begin()->first);
        }
        expect(that % (keys == std::vector<std::string>{"trigger_name", "noise_blanked", "signal_name"})) << "in the order of the samples they belong to";
        if (keys.size() == 3UZ) {
            expect(after.tags[1UZ].map.at(gr::property_map::key_type{"noise_blanked"}) == gr::pmt::Value(gr::Size_t{9U})) << "the marker states the run's nine samples, the dropped ones counted";
        }
        bool restarted = after.samples.size() == x.size() - kFirst;
        for (std::size_t i = 0UZ; restarted && i < after.samples.size(); ++i) {
            restarted = after.samples[i] == (i < 9UZ ? CF{} : x[kFirst + i - 9UZ]);
        }
        expect(that % restarted) << "the line restarts with zeros, and the inputs after the reset come out delayed and untouched";
    };

    "start() empties the line and restarts the tracker"_test = [] {
        const std::vector<CF> first  = noise(5000UZ, 0x1f83d9abfb41bd6bULL);
        const std::vector<CF> second = noise(5000UZ, 0x5be0cd19137e2179ULL);

        NoiseBlanker<CF> reused = make<CF>({{"enabled", true}, {"sample_rate", kRate}});
        std::ignore             = test::run(reused, std::span<const CF>(first), 4096UZ);
        reused.start();
        NoiseBlanker<CF> fresh = make<CF>({{"enabled", true}, {"sample_rate", kRate}});
        fresh.start();
        expect(that % (test::run(reused, std::span<const CF>(second), 4096UZ).samples == test::run(fresh, std::span<const CF>(second), 4096UZ).samples)) << "a restarted block is a fresh one: no sample of the earlier run comes out";
    };

    "a run of exact zeros leaves both trackers at zero, not at a subnormal"_test = [] {
        // Both trackers keep a double, and the silence that walks one into the subnormals is long. At the default pole
        // (0.01 s at 96 kHz, alpha = 1.0411e-3) the power needs 680 061 samples. The duty, whose pole is ten times
        // slower, needs 6.7 million. Only the pole sets the count. The case therefore runs at alpha = 0.1, where the
        // power needs 6 724 samples and the duty, from a half, needs 70 416.
        constexpr std::size_t kZeros = 80000UZ;

        NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"averaging_time", 0.0}, {"alpha", 0.1}});
        block._warmup          = 0UZ;
        block._power           = 1.0; // where a settled tracker on unit-power noise sits
        block._duty            = 0.5; // and where a run of impulses leaves the duty

        const std::vector<CF>  zeros(kZeros, CF{});
        const test::Result<CF> got = test::run(block, std::span<const CF>(zeros), 4096UZ);

        expect(eq(std::fpclassify(block._power), FP_ZERO)) << "the tracked power is exactly zero, not the smallest subnormal";
        expect(eq(std::fpclassify(block._duty), FP_ZERO)) << "and so is the duty, ten times slower to get there and flushed all the same";
        const auto subnormal = [](CF sample) { return std::fpclassify(sample.real()) == FP_SUBNORMAL || std::fpclassify(sample.imag()) == FP_SUBNORMAL; };
        expect(eq(std::ranges::count_if(got.samples, subnormal), std::ptrdiff_t{0})) << "and no output on the run is subnormal";
    };

    "the flush changes nothing above the subnormal range"_test = [] {
        // The two recursions without the flush, run sample by sample beside the block over a step of unit magnitude
        // and its decay. The seeded reference power is exactly the step's power. Nothing is detected and no window
        // opens. Both updates run unconditionally, and the mirror is exact. At alpha = 0.5 the power passes 1e-300
        // after 997 zeros and cannot be flushed before 1022. The duty, at a tenth of that pole, passes 1e-300 at
        // sample 13 454 and cannot be flushed before 13 798.
        constexpr double      kAlpha = 0.5;
        constexpr std::size_t kStep  = 8UZ;
        constexpr std::size_t kTotal = 13454UZ;

        NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"averaging_time", 0.0}, {"alpha", kAlpha}});
        block._warmup          = 0UZ;
        block._power           = 1.0;
        block._duty            = 0.5;

        double      power        = block._power;
        double      duty         = block._duty;
        std::size_t powerSamples = 0UZ;
        bool        identical    = true;

        for (std::size_t i = 0UZ; i < kTotal && identical; ++i) {
            const CF     sample = i < kStep ? CF(1.f, 0.f) : CF{};
            const double p      = static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
            const bool   above  = power >= 1e-300;
            power += kAlpha * (p - power);
            duty += 0.1 * kAlpha * (0.0 - duty);

            CF                   made = CF(1.f, 1.f);
            test::InputSpan<CF>  inSpan(std::span<const CF>(&sample, 1UZ), i);
            test::OutputSpan<CF> outSpan(std::span<CF>(&made, 1UZ), i);
            std::ignore = block.processBulk(inSpan, outSpan);

            const CF want = i >= 9UZ && i < kStep + 9UZ ? CF(1.f, 0.f) : CF{}; // nothing is marked, so the output is the input delayed by nine
            identical     = identical && block._duty == duty && made == want;
            if (above) {
                identical    = identical && block._power == power;
                powerSamples = i + 1UZ;
            }
        }
        expect(identical) << "both states and the output agree bit for bit with the unflushed recursions at every sample";
        expect(lt(power, 1e-300) && lt(duty, 1e-300)) << "and the comparison ran until each state was below 1e-300";
        expect(eq(powerSamples, kStep + 997UZ)) << "the power crossing 1e-300 where its pole says, well before it could be flushed";
    };

    "the per-sample cost stays inside the recorded budget"_test = [] {
        if (std::getenv("ENABLE_BENCHMARK_TESTS") == nullptr) {
            return; // opt-in, since a throughput figure belongs to a controlled run and not to every ctest invocation
        }
        using Clock                   = std::chrono::steady_clock;
        constexpr std::size_t kLength = 1UZ << 22;
        constexpr std::size_t kChunk  = 4096UZ;
        constexpr int         kRuns   = 7;

        const std::vector<CF> x = noise(kLength, 0x24a19947b3916cf7ULL);
        std::vector<CF>       y(kLength);

        NoiseBlanker<CF> full = make<CF>({{"enabled", true}, {"sample_rate", kRate}});

        double best      = 1e300;
        double worst     = 0.0;
        double floorBest = 1e300;
        for (int repeat = 0; repeat <= kRuns; ++repeat) {
            const auto floorStart = Clock::now();
            std::copy_n(x.begin(), kLength, y.begin());
            const double floorNs = std::chrono::duration<double, std::nano>(Clock::now() - floorStart).count() / static_cast<double>(kLength);

            const auto start = Clock::now();
            for (std::size_t base = 0UZ; base < kLength; base += kChunk) {
                test::InputSpan<CF>  inSpan(std::span<const CF>(x).subspan(base, kChunk), base);
                test::OutputSpan<CF> outSpan(std::span<CF>(y).subspan(base, kChunk), base);
                std::ignore = full.processBulk(inSpan, outSpan);
            }
            const double ns = std::chrono::duration<double, std::nano>(Clock::now() - start).count() / static_cast<double>(kLength);
            if (repeat > 0) {
                best      = std::min(best, ns);
                worst     = std::max(worst, ns);
                floorBest = std::min(floorBest, floorNs);
            }
        }
        std::println("NoiseBlanker<complex<float>> {:.3f} ns/sample (spread {:.3f}), span copy {:.3f}", best, worst - best, floorBest);
        // The bound holds with margin for an optimized build of this test. It catches a change of shape.
        expect(lt(best, 22.0)) << std::format("the block reads {:.3f} ns/sample", best);
    };
};

int main() { /* not needed for UT */ }
