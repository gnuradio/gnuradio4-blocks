#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <print>
#include <span>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/algorithm/measurement/PolarizationCombiner.hpp>
#include <gnuradio-4.0/algorithm/rng/GaussianNoise.hpp>
#include <gnuradio-4.0/algorithm/rng/Xoshiro256pp.hpp>

using namespace boost::ut;
using gr::measurement::BranchCovariance;
using gr::measurement::kPolarizationSaturation;
using gr::measurement::polarizationCombine;
using gr::measurement::PolarizationEstimate;
using gr::measurement::PolarizationMode;
using gr::measurement::polarizationModeFrom;
using gr::measurement::PolarizationNormalization;
using gr::measurement::polarizationNormalizationFrom;
using gr::measurement::polarizationPassthrough;

namespace {

using Complex = std::complex<float>;

/// A two-branch scene. One signal passes through two channels, and each branch adds independent noise.
struct Scene {
    std::vector<Complex> signal0; ///< the signal component on branch 0, alone
    std::vector<Complex> signal1;
    std::vector<Complex> noise0; ///< the noise component on branch 0, alone
    std::vector<Complex> noise1;
    std::vector<Complex> branch0; ///< their sum, the estimator's input
    std::vector<Complex> branch1;
};

[[nodiscard]] Scene makeScene(gr::rng::GaussianNoise<float>& noise, std::complex<double> h0, std::complex<double> h1, double noisePower0, double noisePower1, std::size_t n) {
    Scene scene;
    scene.signal0.resize(n);
    scene.signal1.resize(n);
    scene.noise0.resize(n);
    scene.noise1.resize(n);
    scene.branch0.resize(n);
    scene.branch1.resize(n);

    const Complex g0 = static_cast<Complex>(h0);
    const Complex g1 = static_cast<Complex>(h1);
    const float   a0 = static_cast<float>(std::sqrt(noisePower0));
    const float   a1 = static_cast<float>(std::sqrt(noisePower1));

    noise.fillComplex(scene.noise0, a0);
    noise.fillComplex(scene.noise1, a1);
    for (std::size_t k = 0UZ; k < n; ++k) {
        const Complex s  = noise.complexSample();
        scene.signal0[k] = g0 * s;
        scene.signal1[k] = g1 * s;
        scene.branch0[k] = scene.signal0[k] + scene.noise0[k];
        scene.branch1[k] = scene.signal1[k] + scene.noise1[k];
    }
    return scene;
}

[[nodiscard]] double meanPower(std::span<const Complex> in) {
    double sum = 0.;
    for (const Complex value : in) {
        sum += static_cast<double>(value.real()) * static_cast<double>(value.real()) + static_cast<double>(value.imag()) * static_cast<double>(value.imag());
    }
    return sum / static_cast<double>(in.size());
}

/// The output signal-to-noise ratio of @p estimate, from the signal-only and noise-only streams through the same
/// weights.
[[nodiscard]] double realizedSnr(const Scene& scene, const PolarizationEstimate& estimate) {
    std::vector<Complex> signal(scene.signal0.size());
    std::vector<Complex> noise(scene.noise0.size());
    expect(polarizationCombine(scene.signal0, scene.signal1, estimate, signal).has_value());
    expect(polarizationCombine(scene.noise0, scene.noise1, estimate, noise).has_value());
    return meanPower(signal) / meanPower(noise);
}

[[nodiscard]] double toDb(double linear) { return 10. * std::log10(linear); }

} // namespace

const boost::ut::suite<"PolarizationCombiner"> _polarization = [] {
    "the mode and normalization names parse"_test = [] {
        expect(polarizationModeFrom("mrc").value() == PolarizationMode::mrc);
        expect(polarizationModeFrom("selection").value() == PolarizationMode::selection);
        expect(!polarizationModeFrom("mrc ").has_value());
        expect(polarizationNormalizationFrom("unit_noise").value() == PolarizationNormalization::unit_noise);
        expect(polarizationNormalizationFrom("unit_signal").value() == PolarizationNormalization::unit_signal);
        expect(!polarizationNormalizationFrom("unity").has_value());
    };

    "the branch ratios sum to the combined ratio on any data"_test = [] {
        // Seeded pairs with no channel model. `a - lambda_-` and `b - lambda_-` sum to `lambda_+ - lambda_-`, and the
        // two branch ratios sum to the combined ratio.
        gr::rng::Xoshiro256pp         rng(0xDECAFULL);
        gr::rng::GaussianNoise<float> noise(rng);

        double worst = 0.;
        for (std::size_t trial = 0UZ; trial < 20UZ; ++trial) {
            BranchCovariance covariance;
            for (std::size_t k = 0UZ; k < 4'096UZ; ++k) {
                const Complex a = noise.complexSample() + Complex{0.3f * static_cast<float>(trial), 0.1f};
                const Complex b = 0.7f * a + noise.complexSample();
                covariance.add(a, b);
            }
            const PolarizationEstimate estimate = covariance.solve().value();
            const double               sum      = estimate.branchSnr0 + estimate.branchSnr1;
            worst                               = std::max(worst, std::abs(sum - estimate.combinedSnr) / estimate.combinedSnr);
        }
        std::println("the additive identity on arbitrary data: worst relative departure {:.3e}", worst);
        expect(lt(worst, 1e-9)) << "SNR_out = SNR_0 + SNR_1 holds on the estimates";
    };

    "the phase and the amplitude ratio recover the channel without noise bias"_test = [] {
        constexpr std::size_t kWindow = 262'144UZ;
        constexpr double      kNoise  = 0.02;

        gr::rng::Xoshiro256pp         rng(0x1234ULL);
        gr::rng::GaussianNoise<float> noise(rng);
        const std::complex<double>    h1    = 0.5 * std::exp(std::complex<double>{0., 2.});
        const Scene                   scene = makeScene(noise, {1., 0.}, h1, kNoise, kNoise, kWindow);

        BranchCovariance covariance;
        expect(covariance.add(scene.branch0, scene.branch1).has_value());
        const PolarizationEstimate estimate = covariance.solve().value();

        const double tolerance = 4. / std::sqrt(static_cast<double>(kWindow));
        std::println("channel recovery at M = {}: relative phase {:.5f} against -2, amplitude ratio {:.5f} against 2, tolerance {:.5f}", kWindow, estimate.relativePhase, estimate.amplitudeRatio, tolerance);
        expect(std::abs(estimate.relativePhase + 2.) < tolerance) << "arg(h0) - arg(h1) is -2 rad";
        expect(std::abs(estimate.amplitudeRatio - 2.) < tolerance) << "|h0|/|h1| is 2";

        // The branch noise biases `sqrt(a/b)` toward 1.
        const double naive = std::sqrt(covariance.a() / covariance.b());
        std::println("sqrt(a/b) reads {:.5f}", naive);
        expect(lt(naive, 2.)) << "sqrt(a/b) reads below 2";
    };

    "maximal-ratio combining adds the two branch ratios within 0.05 dB"_test = [] {
        constexpr std::size_t kWindow = 262'144UZ;
        constexpr double      kNoise  = 0.1;
        // One power measurement over N samples spreads by 4.343/sqrt(N) dB, and four times that is 0.034 dB here. The
        // loss from estimating the weights is smaller at this window.
        constexpr double kEnvelopeDb = 0.05;

        struct Arm {
            std::complex<double> h1;
            double               expectedGainDb;
            std::string_view     label;
        };
        const Arm arms[] = {{{std::cos(0.7), std::sin(0.7)}, 3.0103, "equal branches"}, {0.5 * std::exp(std::complex<double>{0., 2.}), 0.9691, "branch 1 at -6 dB"}};

        for (const Arm& arm : arms) {
            gr::rng::Xoshiro256pp         rng(0xF00DULL);
            gr::rng::GaussianNoise<float> noise(rng);

            // The weights come from one window and apply to an independent one. The figure includes the estimation
            // loss.
            const Scene training = makeScene(noise, {1., 0.}, arm.h1, kNoise, kNoise, kWindow);
            const Scene applied  = makeScene(noise, {1., 0.}, arm.h1, kNoise, kNoise, kWindow);

            BranchCovariance covariance;
            expect(covariance.add(training.branch0, training.branch1).has_value());
            const PolarizationEstimate estimate = covariance.solve().value();

            const double combined = realizedSnr(applied, estimate);
            const double branch0  = meanPower(applied.signal0) / meanPower(applied.noise0);
            const double branch1  = meanPower(applied.signal1) / meanPower(applied.noise1);
            const double gainDb   = toDb(combined) - toDb(std::max(branch0, branch1));

            std::println("{}: branch ratios {:.3f} and {:.3f} dB, combined {:.3f} dB, gain {:.4f} dB against the predicted {:.4f}", arm.label, toDb(branch0), toDb(branch1), toDb(combined), gainDb, arm.expectedGainDb);
            expect(std::abs(gainDb - arm.expectedGainDb) < kEnvelopeDb) << arm.label << "misses 10*log10(1 + SNR_min/SNR_max)";
            expect(approx(toDb(combined), toDb(branch0 + branch1), kEnvelopeDb)) << "the output ratio is the sum of the two branch ratios";
        }
    };

    "the first weight lies on the positive real axis in every window"_test = [] {
        constexpr std::size_t kWindow = 8'192UZ;

        gr::rng::Xoshiro256pp         rng(0xABCDULL);
        gr::rng::GaussianNoise<float> noise(rng);
        const std::complex<double>    h1 = std::exp(std::complex<double>{0., 1.3});

        double worstStep = 0.;
        double previous  = 0.;
        for (std::size_t window = 0UZ; window < 8UZ; ++window) {
            const Scene      scene = makeScene(noise, {1., 0.}, h1, 0.05, 0.05, kWindow);
            BranchCovariance covariance;
            expect(covariance.add(scene.branch0, scene.branch1).has_value());
            const PolarizationEstimate estimate = covariance.solve().value();

            // An eigenvector is defined up to a unit-modulus scalar. solve() fixes it with arg(weight0) = 0.
            expect(estimate.weight0.imag() == 0.) << "arg(weight 0) is zero exactly, window" << window;
            expect(estimate.weight0.real() > 0.) << "weight 0 is positive";

            const double phase = std::arg(estimate.weight1);
            if (window > 0UZ) {
                worstStep = std::max(worstStep, std::abs(phase - previous));
            }
            previous = phase;
        }
        std::println("across eight windows the second weight's phase moves at most {:.6f} rad", worstStep);
        expect(lt(worstStep, 0.05)) << "the second weight's phase moves by less than 0.05 rad between windows";
    };

    "the orthogonal output nulls the signal"_test = [] {
        constexpr std::size_t kWindow = 65'536UZ;

        gr::rng::Xoshiro256pp         rng(0x5150ULL);
        gr::rng::GaussianNoise<float> noise(rng);
        const std::complex<double>    h1    = 0.8 * std::exp(std::complex<double>{0., -0.4});
        const Scene                   scene = makeScene(noise, {1., 0.}, h1, 0.01, 0.01, kWindow);

        BranchCovariance covariance;
        expect(covariance.add(scene.branch0, scene.branch1).has_value());
        const PolarizationEstimate estimate = covariance.solve().value();

        // `u = [-conj(w_1), conj(w_0)]` is orthogonal to `w`.
        const std::complex<double> inner = std::conj(estimate.weight0) * estimate.ortho0 + std::conj(estimate.weight1) * estimate.ortho1;
        expect(lt(std::abs(inner), 1e-15)) << "w^H u is zero to rounding, got" << std::abs(inner);

        std::vector<Complex> combined(kWindow);
        std::vector<Complex> ortho(kWindow);
        expect(polarizationCombine(scene.signal0, scene.signal1, estimate, combined, ortho).has_value());
        const double leak = toDb(meanPower(ortho) / meanPower(combined));
        std::println("the signal in the orthogonal output reads {:.1f} dB against the combined output", leak);
        expect(lt(leak, -40.)) << "the orthogonal output carries the signal at least 40 dB below the combined output";
    };

    "selection follows the stronger branch and switches once as the branches cross"_test = [] {
        constexpr std::size_t kWindow = 16'384UZ;

        gr::rng::Xoshiro256pp         rng(0x2468ULL);
        gr::rng::GaussianNoise<float> noise(rng);

        int         selected  = -1;
        std::size_t switches  = 0UZ;
        std::size_t mrcBetter = 0UZ;
        for (std::size_t step = 0UZ; step < 6UZ; ++step) {
            // Branch 1's amplitude crosses branch 0's halfway through.
            const double               magnitude = 0.5 + 0.25 * static_cast<double>(step);
            const std::complex<double> h1        = magnitude * std::exp(std::complex<double>{0., 0.9});
            const Scene                scene     = makeScene(noise, {1., 0.}, h1, 0.05, 0.05, kWindow);

            BranchCovariance covariance;
            expect(covariance.add(scene.branch0, scene.branch1).has_value());
            const PolarizationEstimate chosen = covariance.solve(PolarizationMode::selection).value();
            const PolarizationEstimate mrc    = covariance.solve(PolarizationMode::mrc).value();

            if (selected >= 0 && chosen.selectedBranch != selected) {
                ++switches;
            }
            selected = chosen.selectedBranch;

            const double bySelection = realizedSnr(scene, chosen);
            const double byMrc       = realizedSnr(scene, mrc);
            if (byMrc > bySelection) {
                ++mrcBetter;
            }
            expect(mrc.selectedBranch == -1) << "mrc names no branch";
        }
        expect(switches == 1UZ) << "the selection changes exactly once as the branches cross, got" << switches;
        expect(mrcBetter == 6UZ) << "maximal-ratio combining beats selection at every step";
    };

    "an unbounded ratio is flagged and reads the cap"_test = [] {
        constexpr std::size_t kWindow = 4'096UZ;

        // Both branches carry the same noiseless sequence. `a`, `b` and `|c|` are the same sums, and `lambda_-` is
        // exactly zero.
        BranchCovariance noiseless;
        for (std::size_t k = 0UZ; k < kWindow; ++k) {
            const Complex sample{static_cast<float>(std::cos(0.31 * static_cast<double>(k))), static_cast<float>(std::sin(0.31 * static_cast<double>(k)))};
            noiseless.add(sample, sample);
        }
        const PolarizationEstimate hot = noiseless.solve().value();
        expect(hot.saturatedSnr) << "an unbounded ratio is flagged";
        expect(hot.branchSnr0 == kPolarizationSaturation) << "and reads the 60 dB cap";
        expect(hot.combinedSnr == kPolarizationSaturation);
        expect(approx(hot.combinedSnrDb(), 60., 1e-9));

        // Branch 1 is all zeros. `lambda_+ - a` is exactly zero, and the amplitude ratio is unbounded. The weights
        // select branch 0.
        gr::rng::Xoshiro256pp         rng(0x9999ULL);
        gr::rng::GaussianNoise<float> noise(rng);
        std::vector<Complex>          only0(kWindow);
        const std::vector<Complex>    empty(kWindow);
        noise.fillComplex(only0);

        BranchCovariance lonely;
        expect(lonely.add(only0, empty).has_value());
        const PolarizationEstimate degenerate = lonely.solve().value();
        expect(degenerate.saturatedRatio) << "the unbounded amplitude ratio is flagged";
        expect(degenerate.amplitudeRatio == kPolarizationSaturation);
        expect(degenerate.weight0 == std::complex<double>{1., 0.});
        expect(degenerate.weight1 == std::complex<double>{0., 0.});

        std::vector<Complex> out(kWindow);
        expect(polarizationCombine(only0, empty, degenerate, out).has_value());
        expect(that % std::ranges::equal(out, only0)) << "and the output is branch 0, bit for bit";

        // A ratio of zero reads the decibel floor, and the gain stays finite.
        const PolarizationEstimate silent{};
        expect(eq(silent.combinedSnrDb(), gr::math::kDbFloor));
        expect(eq(silent.branchSnrDb(0UZ), gr::math::kDbFloor));
        expect(eq(silent.combiningGainDb(), 0.));
    };

    "with stated noise powers the amplitude ratio is |h0|/|h1|"_test = [] {
        constexpr std::size_t kWindow = 65'536UZ;
        constexpr double      kNoise0 = 1e-4;
        constexpr double      kNoise1 = 1e-3; // a one-to-ten noise imbalance

        gr::rng::Xoshiro256pp         rng(0x6060ULL);
        gr::rng::GaussianNoise<float> noise(rng);
        const std::complex<double>    h1     = 0.5 * std::exp(std::complex<double>{0., 2.});
        const Scene                   scene  = makeScene(noise, {1., 0.}, h1, kNoise0, kNoise1, kWindow);
        const std::vector<double>     powers = {kNoise0, kNoise1};

        BranchCovariance covariance;
        expect(covariance.add(scene.branch0, scene.branch1).has_value());
        const PolarizationEstimate estimate = covariance.solve(PolarizationMode::mrc, PolarizationNormalization::unit_noise, powers).value();

        // The whitened ratio is 2 * sqrt(N1/N0), 6.32. The branch ratios are 40 and 24 dB, and the estimate's spread is
        // far inside the tolerance.
        const double tolerance = 4. / std::sqrt(static_cast<double>(kWindow));
        std::println("with noise powers {} and {}: amplitude ratio {:.5f} against 2, relative phase {:.5f} against -2", kNoise0, kNoise1, estimate.amplitudeRatio, estimate.relativePhase);
        expect(!estimate.saturatedRatio && !estimate.saturatedSnr);
        expect(std::abs(estimate.amplitudeRatio - 2.) < tolerance) << "the amplitude ratio is |h0|/|h1| = 2, not the whitened ratio";
        expect(std::abs(estimate.relativePhase + 2.) < tolerance) << "whitening leaves the relative phase alone";
    };

    "under selection the figures are the selected branch's, and unit_signal gives it unit signal power"_test = [] {
        constexpr std::size_t kWindow = 65'536UZ;
        constexpr double      kNoise  = 0.1;

        gr::rng::Xoshiro256pp         rng(0x3131ULL);
        gr::rng::GaussianNoise<float> noise(rng);
        const std::complex<double>    h1       = 0.5 * std::exp(std::complex<double>{0., 0.7}); // branch 1 at -6 dB
        const Scene                   training = makeScene(noise, {1., 0.}, h1, kNoise, kNoise, kWindow);
        const Scene                   applied  = makeScene(noise, {1., 0.}, h1, kNoise, kNoise, kWindow);

        BranchCovariance covariance;
        expect(covariance.add(training.branch0, training.branch1).has_value());
        const PolarizationEstimate chosen = covariance.solve(PolarizationMode::selection, PolarizationNormalization::unit_signal).value();
        const PolarizationEstimate mrc    = covariance.solve(PolarizationMode::mrc, PolarizationNormalization::unit_signal).value();

        expect(eq(chosen.selectedBranch, 0)) << "branch 0 is the stronger";
        expect(eq(chosen.combinedSnr, chosen.branchSnr0)) << "the combined ratio is the selected branch's";
        expect(eq(chosen.combiningGainDb(), 0.)) << "selection gains nothing over the better branch";
        expect(eq(chosen.branchSnr0, mrc.branchSnr0) && eq(chosen.branchSnr1, mrc.branchSnr1)) << "the branch ratios do not depend on the mode";

        const double         realizedDb = toDb(realizedSnr(applied, chosen));
        std::vector<Complex> signal(kWindow);
        expect(polarizationCombine(applied.signal0, applied.signal1, chosen, signal).has_value());
        std::println("selection: reported {:.3f} dB, realized {:.3f} dB, output signal power {:.5f}", chosen.combinedSnrDb(), realizedDb, meanPower(signal));
        expect(approx(chosen.combinedSnrDb(), realizedDb, 0.15)) << "the reported ratio is the output's";
        expect(approx(meanPower(signal), 1., 0.03)) << "unit_signal sets the selected branch's signal power to 1";
    };

    "a silent branch reads a ratio of zero, and the other branch reads the cap"_test = [] {
        constexpr std::size_t kWindow = 4'096UZ;

        gr::rng::Xoshiro256pp         rng(0x5151ULL);
        gr::rng::GaussianNoise<float> noise(rng);
        const Scene                   scene = makeScene(noise, {1., 0.}, {0., 0.}, 0.1, 0.1, kWindow);
        const std::vector<Complex>    zeros(kWindow);

        BranchCovariance silent1;
        expect(silent1.add(scene.branch0, zeros).has_value());
        for (const PolarizationMode mode : {PolarizationMode::mrc, PolarizationMode::selection}) {
            const PolarizationEstimate estimate = silent1.solve(mode).value();
            std::println("branch 1 silent, {}: branch ratios {} and {}, combined {}, saturated {}", gr::measurement::polarizationModeName(mode), estimate.branchSnr0, estimate.branchSnr1, estimate.combinedSnr, estimate.saturatedSnr);
            expect(estimate.saturatedSnr) << "one branch alone gives no noise measurement";
            expect(eq(estimate.branchSnr1, 0.)) << "the silent branch carries no signal";
            expect(eq(estimate.branchSnr0, kPolarizationSaturation)) << "the other branch's ratio is unmeasured and reads the cap";
            expect(eq(estimate.combinedSnr, kPolarizationSaturation));
        }

        BranchCovariance silent0;
        expect(silent0.add(zeros, scene.branch0).has_value());
        const PolarizationEstimate mirrored = silent0.solve().value();
        expect(eq(mirrored.branchSnr0, 0.) && eq(mirrored.branchSnr1, kPolarizationSaturation)) << "the same holds with branch 0 silent";

        BranchCovariance both;
        expect(both.add(zeros, zeros).has_value());
        const PolarizationEstimate nothing = both.solve().value();
        expect(nothing.saturatedSnr && eq(nothing.branchSnr0, 0.) && eq(nothing.branchSnr1, 0.) && eq(nothing.combinedSnr, 0.)) << "two silent branches read zero throughout";
    };

    "a noiseless input reads the cap with its flag, and 80 dB is measured"_test = [] {
        constexpr std::size_t kWindow = 4'096UZ;

        // `r1 = g r0` in float. Rounding leaves `lambda_-` at zero or near 150 dB below the total power, of either sign.
        std::vector<Complex> r0(kWindow);
        std::vector<Complex> r1(kWindow);
        for (std::size_t k = 0UZ; k < kWindow; ++k) {
            const double phase = 2. * std::numbers::pi * 0.031 * static_cast<double>(k);
            r0[k]              = Complex{static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))};
        }
        for (const Complex g : {Complex{2.f, 0.f}, Complex{1.f, 0.f}, std::polar(0.5f, 0.7f), std::polar(0.3f, -1.1f), std::polar(1.f, 0.2f), std::polar(0.9f, 2.5f)}) {
            std::ranges::transform(r0, r1.begin(), [g](Complex value) { return g * value; });
            BranchCovariance covariance;
            expect(covariance.add(r0, r1).has_value());
            const PolarizationEstimate estimate = covariance.solve().value();
            std::println("noiseless g = {:.3f}{:+.3f}j: lambda- {:.3e}, saturated {}, combined {:.2f} dB", g.real(), g.imag(), estimate.lambdaMinus, estimate.saturatedSnr, estimate.combinedSnrDb());
            expect(estimate.saturatedSnr) << std::format("g = {}{:+}j: the rounding is not a noise measurement", g.real(), g.imag());
            expect(eq(estimate.combinedSnr, kPolarizationSaturation) && eq(estimate.branchSnr0, kPolarizationSaturation) && eq(estimate.branchSnr1, kPolarizationSaturation));
        }

        // Noise 80 dB below each branch's signal lies above the floor. The combined ratio is 83 dB.
        gr::rng::Xoshiro256pp         rng(0x8181ULL);
        gr::rng::GaussianNoise<float> noise(rng);
        const Scene                   quiet = makeScene(noise, {1., 0.}, std::exp(std::complex<double>{0., 0.4}), 1e-8, 1e-8, kWindow);
        BranchCovariance              covariance;
        expect(covariance.add(quiet.branch0, quiet.branch1).has_value());
        const PolarizationEstimate measured = covariance.solve().value();
        std::println("noise at -80 dB: combined {:.3f} dB against {:.3f} dB", measured.combinedSnrDb(), toDb(2. / 1e-8));
        expect(!measured.saturatedSnr) << "a ratio of 83 dB is measured";
        expect(approx(measured.combinedSnrDb(), toDb(2. / 1e-8), 0.5));
    };

    "a pair with a non-finite sample stays out of the covariance and is counted"_test = [] {
        constexpr std::size_t kWindow = 4'096UZ;

        gr::rng::Xoshiro256pp         rng(0x4141ULL);
        gr::rng::GaussianNoise<float> noise(rng);
        const Scene                   scene = makeScene(noise, {1., 0.}, {0.3, 0.2}, 0.01, 0.01, kWindow);

        BranchCovariance clean;
        expect(clean.add(scene.branch0, scene.branch1).has_value());

        BranchCovariance spoiled;
        const auto       half = static_cast<std::ptrdiff_t>(kWindow / 2UZ);
        expect(spoiled.add(std::span<const Complex>(scene.branch0).first(kWindow / 2UZ), std::span<const Complex>(scene.branch1).first(kWindow / 2UZ)).has_value());
        spoiled.add(Complex{std::numeric_limits<float>::quiet_NaN(), 0.f}, scene.branch1[0UZ]);
        spoiled.add(scene.branch0[0UZ], Complex{0.f, -std::numeric_limits<float>::infinity()});
        expect(spoiled.add(std::span<const Complex>(scene.branch0.begin() + half, scene.branch0.end()), std::span<const Complex>(scene.branch1.begin() + half, scene.branch1.end())).has_value());

        expect(eq(spoiled.nonFinite(), 2UZ)) << "a NaN pair and an infinite pair are counted";
        expect(eq(spoiled.count(), clean.count())) << "neither enters the sums";
        expect(spoiled.a() == clean.a() && spoiled.b() == clean.b() && spoiled.c() == clean.c()) << "the sums equal those of the finite pairs alone";
        const PolarizationEstimate estimate = spoiled.solve().value();
        expect(std::isfinite(std::abs(estimate.weight0)) && std::isfinite(std::abs(estimate.weight1))) << "the weights are finite";
        expect(eq(clean.nonFinite(), 0UZ));

        spoiled.reset();
        expect(eq(spoiled.nonFinite(), 0UZ)) << "reset() clears the count";
    };

    "whitening by the noise powers reaches the optimum under unequal branch noise"_test = [] {
        constexpr std::size_t kWindow = 262'144UZ;

        gr::rng::Xoshiro256pp         rng(0x77ULL);
        gr::rng::GaussianNoise<float> noise(rng);
        const std::complex<double>    h1     = std::exp(std::complex<double>{0., 0.4});
        const Scene                   scene  = makeScene(noise, {1., 0.}, h1, 0.02, 0.2, kWindow);
        const std::vector<double>     powers = {0.02, 0.2};

        BranchCovariance covariance;
        expect(covariance.add(scene.branch0, scene.branch1).has_value());

        const PolarizationEstimate whitened = covariance.solve(PolarizationMode::mrc, PolarizationNormalization::unit_noise, powers).value();
        const PolarizationEstimate assumed  = covariance.solve().value();

        const double branch0 = meanPower(scene.signal0) / meanPower(scene.noise0);
        const double branch1 = meanPower(scene.signal1) / meanPower(scene.noise1);
        const double best    = realizedSnr(scene, whitened);
        const double naive   = realizedSnr(scene, assumed);

        std::println("branch ratios {:.3f} and {:.3f} dB; whitened combination {:.3f} dB, the equal-noise assumption {:.3f} dB, the optimum {:.3f}", toDb(branch0), toDb(branch1), toDb(best), toDb(naive), toDb(branch0 + branch1));
        expect(approx(toDb(best), toDb(branch0 + branch1), 0.05)) << "whitening reaches the optimum with a ten-to-one noise imbalance";
        expect(gt(best, naive)) << "the equal-noise solution reads lower";

        const auto oneNoisePower = covariance.solve(PolarizationMode::mrc, PolarizationNormalization::unit_noise, std::vector<double>{1.});
        expect(fatal(!oneNoisePower.has_value())) << "two branches take two noise powers";
        expect(oneNoisePower.error().message.contains("1 noise powers")) << oneNoisePower.error().message;
        expect(!covariance.solve(PolarizationMode::mrc, PolarizationNormalization::unit_noise, std::vector<double>{1., 0.}).has_value()) << "a zero noise power is refused";
    };

    "any split of the stream gives the same covariance"_test = [] {
        constexpr std::size_t kWindow = 10'000UZ;

        gr::rng::Xoshiro256pp         rng(0x3333ULL);
        gr::rng::GaussianNoise<float> noise(rng);
        const Scene                   scene = makeScene(noise, {1., 0.}, {0.6, 0.3}, 0.05, 0.05, kWindow);

        BranchCovariance whole;
        expect(whole.add(scene.branch0, scene.branch1).has_value());

        for (const std::size_t chunk : {1UZ, 7UZ, 1'000UZ, 12'345UZ}) {
            BranchCovariance pieced;
            for (std::size_t at = 0UZ; at < kWindow; at += chunk) {
                const std::size_t n = std::min(chunk, kWindow - at);
                expect(pieced.add(std::span<const Complex>(scene.branch0).subspan(at, n), std::span<const Complex>(scene.branch1).subspan(at, n)).has_value());
            }
            expect(pieced.a() == whole.a()) << std::format("chunk {} changed the covariance", chunk);
            expect(pieced.b() == whole.b());
            expect(pieced.c() == whole.c());
            expect(pieced.count() == whole.count());
        }
    };

    "the pass-through estimate copies a branch, and mismatched spans are refused"_test = [] {
        const PolarizationEstimate first = polarizationPassthrough(0UZ);
        expect(first.weight0 == std::complex<double>{1., 0.});
        expect(first.weight1 == std::complex<double>{0., 0.});
        expect(first.selectedBranch == 0);

        const std::vector<Complex> in0{{1.f, 2.f}, {3.f, 4.f}};
        const std::vector<Complex> in1{{5.f, 6.f}, {7.f, 8.f}};
        std::vector<Complex>       out(2UZ);
        std::vector<Complex>       ortho(2UZ);
        expect(polarizationCombine(in0, in1, first, out, ortho).has_value());
        expect(that % std::ranges::equal(out, in0)) << "branch 0 passes through bit for bit";
        expect(that % std::ranges::equal(ortho, in1)) << "branch 1 goes to the orthogonal output";

        const PolarizationEstimate second = polarizationPassthrough(1UZ);
        expect(polarizationCombine(in0, in1, second, out, ortho).has_value());
        expect(that % std::ranges::equal(out, in1));

        const auto unpaired = polarizationCombine(in0, std::span<const Complex>(in1).first(1UZ), first, out);
        expect(fatal(!unpaired.has_value())) << "branches of unequal length are refused";
        expect(unpaired.error().message.contains("against 1 on branch 1")) << unpaired.error().message;
        expect(!polarizationCombine(in0, in1, first, std::span<Complex>(out).first(1UZ)).has_value()) << "one output short";
        expect(!polarizationCombine(in0, in1, first, out, std::span<Complex>(ortho).first(1UZ)).has_value()) << "one orthogonal output short";

        BranchCovariance  covariance;
        const std::size_t before  = covariance.count();
        const auto        refused = covariance.add(in0, std::span<const Complex>(in1).first(1UZ));
        expect(fatal(!refused.has_value()));
        expect(refused.error().message.contains("BranchCovariance::add")) << refused.error().message;
        expect(eq(covariance.count(), before)) << "a refused pair accumulates nothing";
        expect(covariance.count() == 0UZ);
        expect(covariance.a() == 0.) << "an empty covariance reads zero";
    };

    "unit_signal sets the output signal power to 1 and reports the scale"_test = [] {
        constexpr std::size_t kWindow = 65'536UZ;

        gr::rng::Xoshiro256pp         rng(0x8080ULL);
        gr::rng::GaussianNoise<float> noise(rng);
        const Scene                   scene = makeScene(noise, {1., 0.}, {0.7, 0.2}, 0.05, 0.05, kWindow);

        BranchCovariance covariance;
        expect(covariance.add(scene.branch0, scene.branch1).has_value());

        const PolarizationEstimate unitNoise  = covariance.solve(PolarizationMode::mrc, PolarizationNormalization::unit_noise).value();
        const PolarizationEstimate unitSignal = covariance.solve(PolarizationMode::mrc, PolarizationNormalization::unit_signal).value();

        expect(unitNoise.scale == 1.);
        expect(approx(std::norm(unitNoise.weight0) + std::norm(unitNoise.weight1), 1., 1e-12)) << "unit_noise gives a unit weight vector";

        // The covariance identifies `P|h|^2 = lambda_+ - lambda_-`. unit_signal scales the weights by
        // `1/sqrt(lambda_+ - lambda_-)`, and the combined signal power is 1.
        std::vector<Complex> signal(kWindow);
        expect(polarizationCombine(scene.signal0, scene.signal1, unitSignal, signal).has_value());
        std::println("unit_signal scale {:.6f}, realized output signal power {:.6f}", unitSignal.scale, meanPower(signal));
        expect(approx(meanPower(signal), 1., 0.02)) << "the combined output's signal power is 1";
        expect(approx(unitSignal.scale, 1. / std::sqrt(unitSignal.lambdaPlus - unitSignal.lambdaMinus), 1e-12));

        // The ratios do not depend on the scale.
        expect(approx(unitNoise.combinedSnr, unitSignal.combinedSnr, 1e-12));
    };
};

int main() { /* not needed for UT */ }
