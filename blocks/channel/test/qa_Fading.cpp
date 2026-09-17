#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <format>
#include <numbers>
#include <print>
#include <span>
#include <vector>

#include <gnuradio-4.0/algorithm/channel/DelayProfile.hpp>
#include <gnuradio-4.0/channel/Multipath.hpp>

namespace {

using C = std::complex<float>;

template<typename Block>
[[nodiscard]] Block configured(gr::property_map settings) {
    Block block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

/// A constant unit input makes the output of a single-tap channel the tap process itself.
[[nodiscard]] std::vector<C> tapProcess(const gr::property_map& settings, std::size_t nSamples, std::size_t chunk) {
    auto                 block = configured<gr::blocks::channel::FadingChannel<C>>(settings);
    const std::vector<C> ones(nSamples, C(1.f, 0.f));
    std::vector<C>       out(nSamples);
    for (std::size_t i = 0UZ; i < nSamples; i += chunk) {
        const std::size_t n = std::min(chunk, nSamples - i);
        std::ignore         = block.processBulk(std::span<const C>(ones.data() + i, n), std::span<C>(out.data() + i, n));
    }
    return out;
}

[[nodiscard]] double meanPower(std::span<const C> x) {
    double total = 0.;
    for (const C v : x) {
        total += static_cast<double>(std::norm(v));
    }
    return total / static_cast<double>(x.size());
}

/// Normalized complex autocorrelation of the tap process at a lag in samples.
[[nodiscard]] double autocorrelationAt(std::span<const C> h, std::size_t lag) {
    std::complex<double> correlation{0., 0.};
    double               energy = 0.;
    for (std::size_t k = 0UZ; k + lag < h.size(); ++k) {
        const auto a = std::complex<double>(h[k + lag]);
        const auto b = std::complex<double>(h[k]);
        correlation += a * std::conj(b);
        energy += std::norm(b);
    }
    return std::abs(correlation) / energy;
}

/// J0 from its power series, sum_k (-1)^k (x/2)^{2k} / (k!)^2, summed until a term falls below 1e-16 of the
/// running sum. The test carries its own J0 because the C++17 special mathematical functions are optional in
/// practice: libc++ declares none of them. The arguments used here stay at or below pi, where fewer than
/// twenty terms reach the precision of a double.
[[nodiscard]] double besselJ0(double x) {
    const double quarterSquare = 0.25 * x * x;
    double       term          = 1.;
    double       sum           = 1.;
    for (int k = 1; k <= 64; ++k) {
        term *= -quarterSquare / static_cast<double>(k * k);
        sum += term;
        if (std::abs(term) <= 1e-16 * std::abs(sum)) {
            break;
        }
    }
    return sum;
}

/// The most a tap gain of unit amplitude can move in one sample. A unit vector turned by omega moves 2*sin(omega/2)
/// and never more than 2; a diffuse arm turns by at most omega = 2*pi*f_d/f_s and the line of sight by
/// `los_doppler_ratio` of that, and `tapGain` sums N arms scaled by sqrt(1/(K+1))/sqrt(N) and adds the line of
/// sight scaled by sqrt(K/(K+1)).
[[nodiscard]] double gainStepBound(double fd, double fs, std::size_t nSinusoids, double kFactor = 0., double losRatio = 0.) {
    const auto   chord    = [](double omega) { return 2. * std::sin(0.5 * std::min(std::abs(omega), std::numbers::pi_v<double>)); };
    const double omegaMax = 2. * std::numbers::pi_v<double> * std::abs(fd) / fs;
    return std::sqrt(1. / (1. + kFactor)) * std::sqrt(static_cast<double>(nSinusoids)) * chord(omegaMax) + std::sqrt(kFactor / (1. + kFactor)) * chord(omegaMax * losRatio);
}

/// The largest one-sample step of a tap process from `from` onward.
[[nodiscard]] double largestStep(std::span<const C> gains, std::size_t from) {
    double largest = 0.;
    for (std::size_t k = from; k + 1UZ < gains.size(); ++k) {
        largest = std::max(largest, std::abs(std::complex<double>(gains[k + 1UZ]) - std::complex<double>(gains[k])));
    }
    return largest;
}

struct LiveChange {
    std::vector<C>       gains{};        ///< the tap process; `gains[before]` is the first sample at the new settings
    std::complex<double> beforeUpdate{}; ///< the tap gain at position `before`, read at the old settings
    std::complex<double> afterUpdate{};  ///< the same gain, read once the new settings have been applied
};

/// Runs a single-tap channel for `before` samples, applies `change` to the block while it runs, and runs `after`
/// more. A constant unit input makes the output the tap process itself.
[[nodiscard]] LiveChange liveChange(const gr::property_map& settings, const gr::property_map& change, std::size_t before, std::size_t after) {
    auto block = configured<gr::blocks::channel::FadingChannel<C>>(settings);
    block.start(); // only a running block retunes; before that a settings change draws a new realization

    LiveChange result;
    result.gains.resize(before + after);
    const std::vector<C> ones(std::max(before, after), C(1.f, 0.f));
    std::ignore = block.processBulk(std::span<const C>(ones.data(), before), std::span<C>(result.gains.data(), before));

    result.beforeUpdate = block.currentGains().front();
    std::ignore         = block.settings().setStaged(change);
    std::ignore         = block.settings().applyStagedParameters();
    result.afterUpdate  = block.currentGains().front();

    std::ignore = block.processBulk(std::span<const C>(ones.data(), after), std::span<C>(result.gains.data() + before, after));
    return result;
}

} // namespace

const boost::ut::suite<"fading channel"> fadingTests = [] {
    using namespace boost::ut;
    using gr::blocks::channel::FadingChannel;
    using gr::channel::tapsFromProfile;

    constexpr double fs = 10'000.;
    constexpr double fd = 50.;

    const gr::property_map kSingleTap{{"sample_rate", static_cast<float>(fs)}, //
        {"delays", std::vector<gr::Size_t>{0U}}, {"powers_db", std::vector<double>{0.0}}, {"max_doppler", fd}, {"seed", std::uint64_t(4)}};

    // the profile states the average power of each path, so the realized process must carry it
    "tap power matches the profile"_test = [&] {
        const auto   h        = tapProcess(kSingleTap, 200'000UZ, 200'000UZ);
        const double measured = meanPower(h);
        expect(lt(std::abs(measured - 1.0), 0.05)) << std::format("mean |h|^2 = {:g}, profile says 1", measured);
    };

    // Clarke's isotropic scattering gives a J0 autocorrelation; this is what makes the process a fading
    // channel rather than merely a random one
    "autocorrelation follows the Bessel envelope"_test = [&] {
        const auto h = tapProcess(kSingleTap, 400'000UZ, 400'000UZ);
        for (const double lagFraction : {0.1, 0.25, 0.5}) {
            const auto   lag      = static_cast<std::size_t>(std::llround(lagFraction / fd * fs));
            const double measured = autocorrelationAt(std::span<const C>(h), lag);
            const double argument = 2. * std::numbers::pi_v<double> * fd * static_cast<double>(lag) / fs;
            const double bessel   = std::abs(besselJ0(argument));
#if __has_include(<cmath>) && defined(__cpp_lib_math_special_functions)
            expect(lt(std::abs(besselJ0(argument) - std::cyl_bessel_j(0., argument)), 1e-12)) << std::format("the J0 series differs from the standard function at x = {:g}", argument);
#endif
            std::println("Fading autocorrelation at lag {:.2f}/f_d: measured {:.4f}, J0 {:.4f}", lagFraction, measured, bessel);
            expect(lt(std::abs(measured - bessel), 0.12)) << std::format("lag {:.2f}/f_d: measured {:.4f}, J0 {:.4f}", lagFraction, measured, bessel);
        }
    };

    "a zero Doppler freezes the channel"_test = [&] {
        gr::property_map settings = kSingleTap;
        settings["max_doppler"]   = 0.0;
        const auto h              = tapProcess(settings, 4096UZ, 4096UZ);
        for (std::size_t k = 1UZ; k < h.size(); ++k) {
            expect(eq(h[k], h[0])) << std::format("a frozen channel must not vary, differs at {}", k);
        }
    };

    "the realization is bit-identical however the stream is split"_test = [&] {
        constexpr std::size_t nSamples  = 20'000UZ;
        const auto            reference = tapProcess(kSingleTap, nSamples, nSamples);
        for (const std::size_t chunk : {1UZ, 3UZ, 64UZ, 4095UZ}) {
            expect(tapProcess(kSingleTap, nSamples, chunk) == reference) << std::format("{}-sample calls", chunk);
        }
    };

    "the same seed reproduces and a different one does not"_test = [&] {
        gr::property_map other = kSingleTap;
        other["seed"]          = std::uint64_t(5);
        expect(tapProcess(kSingleTap, 4096UZ, 4096UZ) == tapProcess(kSingleTap, 4096UZ, 4096UZ));
        expect(tapProcess(kSingleTap, 4096UZ, 4096UZ) != tapProcess(other, 4096UZ, 4096UZ));
    };

    // The block promises that a setting which needs no new draw is applied to the taps already running without
    // disturbing the stream under them, so a rate change must leave the gain where the process had carried it.
    constexpr std::size_t kSinusoids = 16UZ;
    constexpr std::size_t kBefore    = 1000UZ; // not a multiple of the 256-sample phasor restore interval
    constexpr std::size_t kAfter     = 2000UZ; // carries the stream across the restores that follow the update
    // the tap carries unit mean power, so this absolute bound on the gain is a relative one on the process
    constexpr double kUnchanged = 1e-9;

    const gr::property_map kLiveTap{{"sample_rate", static_cast<float>(fs)}, {"delays", std::vector<gr::Size_t>{0U}}, //
        {"powers_db", std::vector<double>{0.0}}, {"max_doppler", fd}, {"n_sinusoids", static_cast<gr::Size_t>(kSinusoids)}, {"seed", std::uint64_t(4)}};

    // a Rician tap retunes its line of sight along with its diffuse arms, so both carry their phase across
    "a live Doppler change keeps the gain it had reached"_test = [&] {
        constexpr double losRatio = 0.7; // the block's default
        for (const double k : {0., 4.}) {
            gr::property_map settings     = kLiveTap;
            settings["k_factor"]          = k;
            settings["los_doppler_ratio"] = losRatio;

            const auto   run  = liveChange(settings, {{"max_doppler", 2. * fd}}, kBefore, kAfter);
            const double step = std::abs(run.afterUpdate - run.beforeUpdate);
            expect(lt(step, kUnchanged)) << std::format("K = {:.0f}: doubling max_doppler stepped the gain by {:g}", k, step);

            const double bound   = gainStepBound(2. * fd, fs, kSinusoids, k, losRatio);
            const double largest = largestStep(std::span<const C>(run.gains), kBefore - 1UZ);
            expect(lt(largest, bound)) << std::format("K = {:.0f}: largest one-sample step from the update on {:g}, the sinusoids can move {:g}", k, largest, bound);
        }
    };

    "a live sample-rate change keeps the gain it had reached"_test = [&] {
        // halving the rate doubles every phase increment
        const auto   run  = liveChange(kLiveTap, {{"sample_rate", static_cast<float>(0.5 * fs)}}, kBefore, kAfter);
        const double step = std::abs(run.afterUpdate - run.beforeUpdate);
        expect(lt(step, kUnchanged)) << std::format("halving sample_rate stepped the gain by {:g}", step);

        const double bound   = gainStepBound(fd, 0.5 * fs, kSinusoids);
        const double largest = largestStep(std::span<const C>(run.gains), kBefore - 1UZ);
        expect(lt(largest, bound)) << std::format("largest one-sample step from the update on {:g}, the sinusoids can move {:g}", largest, bound);
    };

    "a retuned Doppler is the rate the process then follows"_test = [&] {
        constexpr std::size_t kMeasured = 200'000UZ;
        constexpr double      twoPi     = 2. * std::numbers::pi_v<double>;
        const double          newFd     = 2. * fd;

        const auto               run = liveChange(kLiveTap, {{"max_doppler", newFd}}, kBefore, kMeasured);
        const std::span<const C> after(run.gains.data() + kBefore, kMeasured);

        const auto   lag      = static_cast<std::size_t>(std::llround(0.25 / newFd * fs));
        const double measured = autocorrelationAt(after, lag);
        const double atNew    = std::abs(besselJ0(twoPi * newFd * static_cast<double>(lag) / fs));
        const double atOld    = std::abs(besselJ0(twoPi * fd * static_cast<double>(lag) / fs));
        std::println("Fading after a live retune: autocorrelation {:.4f}, J0 at the new rate {:.4f}, at the old {:.4f}", measured, atNew, atOld);
        expect(lt(std::abs(measured - atNew), 0.12)) << std::format("measured {:.4f}, J0 at the new Doppler {:.4f}", measured, atNew);
        expect(gt(std::abs(measured - atOld), 0.12)) << std::format("measured {:.4f} still follows the old Doppler, whose J0 is {:.4f}", measured, atOld);
    };

    "a live Doppler change starts a frozen channel moving"_test = [&] {
        gr::property_map frozen = kLiveTap;
        frozen["max_doppler"]   = 0.0;

        const auto   run  = liveChange(frozen, {{"max_doppler", fd}}, kBefore, kAfter);
        const double step = std::abs(run.afterUpdate - run.beforeUpdate);
        expect(lt(step, kUnchanged)) << std::format("unfreezing the channel stepped the gain by {:g}", step);

        const double bound   = gainStepBound(fd, fs, kSinusoids);
        const double largest = largestStep(std::span<const C>(run.gains), kBefore - 1UZ);
        expect(lt(largest, bound)) << std::format("largest one-sample step from the update on {:g}, the sinusoids can move {:g}", largest, bound);

        double wandered = 0.;
        for (std::size_t k = kBefore; k < run.gains.size(); ++k) {
            wandered = std::max(wandered, std::abs(std::complex<double>(run.gains[k]) - run.afterUpdate));
        }
        expect(gt(wandered, 0.1)) << std::format("an unfrozen channel must vary, it moved {:g} over {} samples", wandered, kAfter);
    };

    // A line of sight does not shift the mean: it rotates at its own Doppler, so it averages away like the
    // diffuse part. What it changes is the steadiness of the envelope. For a Rayleigh tap |h|^2 is
    // exponential and its variance equals its mean squared; a Rician tap of factor K has variance
    // (1 + 2K)/(1 + K)^2, so a strong line flattens the deep fades out.
    "a Rician factor steadies the envelope by its stated law"_test = [&] {
        const auto varianceOfPower = [](std::span<const C> h) {
            const double mean = meanPower(h);
            double       sum  = 0.;
            for (const C v : h) {
                const double d = static_cast<double>(std::norm(v)) - mean;
                sum += d * d;
            }
            return sum / static_cast<double>(h.size());
        };

        for (const double k : {0., 4., 10.}) {
            gr::property_map settings = kSingleTap;
            settings["k_factor"]      = k;
            const auto   h            = tapProcess(settings, 400'000UZ, 400'000UZ);
            const double measured     = varianceOfPower(std::span<const C>(h));
            const double expected     = (1. + 2. * k) / ((1. + k) * (1. + k));
            std::println("Fading K = {:.0f}: var(|h|^2) measured {:.4f}, law {:.4f}", k, measured, expected);
            expect(lt(std::abs(measured - expected), 0.15)) << std::format("K = {:.0f}: measured {:.4f}, expected {:.4f}", k, measured, expected);
            expect(lt(std::abs(meanPower(h) - 1.0), 0.05)) << "a Rician tap still carries the profile's power";
        }
    };

    "profiles normalize to unit total power"_test = [&] {
        const gr::property_map twoTap{{"sample_rate", static_cast<float>(fs)}, //
            {"delays", std::vector<gr::Size_t>{0U, 3U}}, {"powers_db", std::vector<double>{0.0, -3.0}}, {"max_doppler", fd}, {"seed", std::uint64_t(9)}};

        auto                 block = configured<FadingChannel<C>>(twoTap);
        const std::vector<C> impulseFree(200'000UZ, C(1.f, 0.f));
        std::vector<C>       out(impulseFree.size());
        std::ignore = block.processBulk(std::span<const C>(impulseFree), std::span<C>(out));
        // both taps see the same constant input, so the output power is the sum of the tap powers plus their
        // cross terms, which average away over many coherence times
        expect(lt(std::abs(meanPower(std::span<const C>(out)) - 1.0), 0.1)) << std::format("normalized two-tap power {:g}", meanPower(std::span<const C>(out)));
    };

    "delays are refused unless strictly ascending, and the tables must match"_test = [&] {
        expect(throws([&] { std::ignore = configured<FadingChannel<C>>({{"delays", std::vector<gr::Size_t>{0U, 0U}}, {"powers_db", std::vector<double>{0.0, 0.0}}}); })) << "repeated delays";
        expect(throws([&] { std::ignore = configured<FadingChannel<C>>({{"delays", std::vector<gr::Size_t>{0U, 1U}}, {"powers_db", std::vector<double>{0.0}}}); })) << "mismatched table lengths";
    };

    "tapsFromProfile places each path at its rounded delay"_test = [] {
        // 0 us and 5 us at 1 MS/s land on samples 0 and 5
        const std::vector<double> delays{0.0, 5.0e-6};
        const std::vector<double> powers{0.0, -6.0};
        const auto                taps = tapsFromProfile(std::span<const double>(delays), std::span<const double>(powers), 1.0e6);

        expect(eq(taps.size(), 6UZ)) << "the span reaches the longest delay";
        expect(gt(std::abs(taps[0]), 0.f));
        expect(gt(std::abs(taps[5]), 0.f));
        for (const std::size_t empty : {1UZ, 2UZ, 3UZ, 4UZ}) {
            expect(eq(taps[empty], std::complex<float>(0.f, 0.f))) << std::format("no path at {}", empty);
        }

        double total = 0.;
        for (const auto tap : taps) {
            total += static_cast<double>(std::norm(tap));
        }
        expect(lt(std::abs(total - 1.0), 1e-6)) << std::format("normalized profile power {:g}", total);
        // -6 dB in power is an amplitude ratio of 10^(-6/20) = 0.501187; the round number would be -6.0206 dB
        const double wantedRatio = std::pow(10., -6. / 20.);
        expect(lt(std::abs(static_cast<double>(std::abs(taps[5]) / std::abs(taps[0])) - wantedRatio), 1e-6));
    };

    "paths rounding onto one sample sum in power"_test = [] {
        // two equal paths 0.1 us apart at 1 MS/s both round to sample 0
        const std::vector<double> delays{0.0, 1.0e-7};
        const std::vector<double> powers{0.0, 0.0};
        const auto                taps = tapsFromProfile(std::span<const double>(delays), std::span<const double>(powers), 1.0e6, false);
        expect(eq(taps.size(), 1UZ));
        expect(lt(std::abs(static_cast<double>(std::norm(taps[0])) - 2.0), 1e-6)) << "two unit-power paths on one sample carry two units";
    };

    "tapsFromProfile refuses a malformed profile"_test = [] {
        const std::vector<double> two{0.0, 1.0e-6};
        const std::vector<double> one{0.0};
        const std::vector<double> negative{-1.0e-6};
        expect(throws([&] { std::ignore = tapsFromProfile(std::span<const double>(two), std::span<const double>(one), 1.0e6); }));
        expect(throws([&] { std::ignore = tapsFromProfile(std::span<const double>(negative), std::span<const double>(one), 1.0e6); }));
        expect(throws([&] { std::ignore = tapsFromProfile(std::span<const double>(one), std::span<const double>(one), 0.0); }));
    };
};

int main() { /* not needed for UT */ }
