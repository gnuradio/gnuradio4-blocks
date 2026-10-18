#include <boost/ut.hpp>

#include <algorithm>
#include <array>
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
#include <type_traits>
#include <vector>

#include <gnuradio-4.0/algorithm/measurement/Kurtosis.hpp>
#include <gnuradio-4.0/algorithm/rng/GaussianNoise.hpp>
#include <gnuradio-4.0/algorithm/rng/Xoshiro256pp.hpp>

// MomentSample admits float, double and their complex forms. MomentDomain marks the complex forms as complex.
static_assert(gr::measurement::MomentSample<float> && gr::measurement::MomentSample<double>);
static_assert(gr::measurement::MomentSample<std::complex<float>> && gr::measurement::MomentSample<std::complex<double>>);
static_assert(!gr::measurement::MomentSample<int> && !gr::measurement::MomentSample<long double> && !gr::measurement::MomentSample<std::complex<long double>>);
static_assert(!gr::measurement::detail::MomentDomain<float>::isComplex && !gr::measurement::detail::MomentDomain<double>::isComplex);
static_assert(gr::measurement::detail::MomentDomain<std::complex<float>>::isComplex && gr::measurement::detail::MomentDomain<std::complex<double>>::isComplex);

using namespace boost::ut;
using gr::measurement::gaussianFourthMoment;
using gr::measurement::KurtosisAccumulator;
using gr::measurement::kurtosisBias;
using gr::measurement::kurtosisSpread;
using gr::measurement::kurtosisVariance;
using gr::measurement::spectralKurtosis;
using gr::measurement::SpectralKurtosisAccumulator;
using gr::measurement::spectralKurtosisCw;
using gr::measurement::spectralKurtosisExpectation;
using gr::measurement::spectralKurtosisSpread;
using gr::measurement::spectralKurtosisVariance;
using gr::measurement::SpectrumResponse;

namespace {

constexpr double kTwoPi = 2. * std::numbers::pi_v<double>;
using Complex           = std::complex<float>;

/// The mean and the sample standard deviation of a run of readings.
struct Spread {
    double mean{0.};
    double sigma{0.};

    [[nodiscard]] double standardError(std::size_t n) const { return sigma / std::sqrt(static_cast<double>(n)); }
};

[[nodiscard]] Spread spreadOf(std::span<const double> values) {
    Spread out;
    for (const double value : values) {
        out.mean += value;
    }
    out.mean /= static_cast<double>(values.size());
    for (const double value : values) {
        out.sigma += (value - out.mean) * (value - out.mean);
    }
    out.sigma = std::sqrt(out.sigma / static_cast<double>(values.size() - 1UZ));
    return out;
}

/// The excess kurtosis of unit Gaussian noise `g` plus a Bernoulli(p) impulse of amplitude `A`, in closed form.
/// `E[x^2] = 1 + p A^2` and `E[x^4] = 3 + 6 p A^2 + p A^4`. The odd terms vanish because `E[g] = E[g^3] = 0`.
[[nodiscard]] constexpr double impulsiveExcess(double p, double amplitude) noexcept {
    const double a2 = amplitude * amplitude;
    const double m2 = 1. + p * a2;
    return (3. + 6. * p * a2 + p * a2 * a2) / (m2 * m2) - 3.;
}

/// The large-`M` limit `L` of `SK` at `d = 1` for a carrier of bin power `S`, present a fraction `p` of the time, over
/// noise of unit power, and the first-order coefficient `b1` of its finite-`M` bias, `E[SK] = L + b1/M`.
///
/// The bin power `u` is a mixture. With the carrier present it is noncentral, with raw moments `S + 1`,
/// `S^2 + 4S + 2` and `S^3 + 9S^2 + 18S + 6`. Without it `u` is exponential, with moments 1, 2 and 6. Write `m1`, `m2`
/// and `m3` for the mixture's moments and `v = m2 - m1^2`. Then `L = m2/m1^2 - 1`. The delta method on
/// `M*S2/S1^2 = A/B^2`, with `A` the mean of `u^2` and `B` the mean of `u`, gives
/// `E[A/B^2] = m2/m1^2 + (3 m2 v/m1^4 - 2 (m3 - m1 m2)/m1^3)/M` to first order. The leading factor `(M + 1)/(M - 1)`
/// adds `2L/M`. On noise alone `b1` is zero.
struct PulsedLimit {
    double limit{0.};
    double bias{0.}; ///< `b1`
};

[[nodiscard]] constexpr PulsedLimit pulsedLimit(double p, double signal) noexcept {
    const double s  = signal;
    const double m1 = p * (s + 1.) + (1. - p);
    const double m2 = p * (s * s + 4. * s + 2.) + (1. - p) * 2.;
    const double m3 = p * (s * s * s + 9. * s * s + 18. * s + 6.) + (1. - p) * 6.;
    const double v  = m2 - m1 * m1;
    const double l  = m2 / (m1 * m1) - 1.;
    return {.limit = l, .bias = 2. * l + 3. * m2 * v / (m1 * m1 * m1 * m1) - 2. * (m3 - m1 * m2) / (m1 * m1 * m1)};
}

/// One bin's power summed over `d` independent periodograms, Gamma of shape `d`. The statistic does not depend on
/// scale. The sum gives the same reading as the mean.
[[nodiscard]] float gammaPower(gr::rng::GaussianNoise<float>& noise, std::size_t shape) {
    float sum = 0.f;
    for (std::size_t i = 0UZ; i < shape; ++i) {
        const Complex z = noise.complexSample();
        sum += z.real() * z.real() + z.imag() * z.imag();
    }
    return sum;
}

} // namespace

const boost::ut::suite<"Kurtosis"> _kurtosis = [] {
    "the closed forms of the Gaussian value, the bias and the spread"_test = [] {
        expect(gaussianFourthMoment(false) == 3.);
        expect(gaussianFourthMoment(true) == 2.);
        expect(kurtosisBias(4'096UZ, true) == -2. / 4'096.);
        expect(kurtosisBias(4'096UZ, false) == -6. / 4'096.);
        expect(kurtosisVariance(4'096UZ, true) == 4. / 4'096.);
        expect(kurtosisVariance(4'096UZ, false) == 24. / 4'096.);
        expect(approx(kurtosisSpread(4'096UZ, true), 2. / std::sqrt(4'096.), 1e-12)) << "2/sqrt(N) at N = 4096";
        expect(approx(kurtosisSpread(4'096UZ, false), std::sqrt(24. / 4'096.), 1e-12)) << "sqrt(24/N) at N = 4096";
        expect(kurtosisBias(0UZ, true) == 0.) << "an empty window has zero bias";
    };

    "seeded Gaussian noise reads the closed-form bias and spread"_test = [] {
        constexpr std::size_t kWindows = 400UZ;
        constexpr std::size_t kWindow  = 4'096UZ;

        gr::rng::Xoshiro256pp         rng(0x5EEDU);
        gr::rng::GaussianNoise<float> noise(rng);
        std::vector<Complex>          complexWindow(kWindow);
        std::vector<float>            realWindow(kWindow);
        std::vector<double>           complexReadings(kWindows);
        std::vector<double>           realReadings(kWindows);

        for (std::size_t w = 0UZ; w < kWindows; ++w) {
            noise.fillComplex(complexWindow);
            KurtosisAccumulator<Complex> accumulator;
            accumulator.add(std::span<const Complex>(complexWindow));
            complexReadings[w] = accumulator.excess();

            noise.fill(realWindow);
            KurtosisAccumulator<float> real;
            real.add(std::span<const float>(realWindow));
            realReadings[w] = real.excess();
        }

        for (const bool complexDomain : {true, false}) {
            const Spread measured = spreadOf(complexDomain ? complexReadings : realReadings);
            const double bias     = kurtosisBias(kWindow, complexDomain);
            const double spread   = kurtosisSpread(kWindow, complexDomain);
            const double band     = 4. * spread / std::sqrt(static_cast<double>(kWindows));

            std::println("{} Gaussian at N = {}: mean excess {:+.6f} against the bias {:+.6f} (band +/-{:.6f}); spread {:.6f} against {:.6f}", complexDomain ? "complex" : "real   ", kWindow, measured.mean, bias, band, measured.sigma, spread);
            expect(std::abs(measured.mean - bias) < band) << "the mean matches the closed-form bias";
            expect(std::abs(measured.sigma - spread) < 0.1 * spread) << "and the per-window spread matches the closed form";
        }
    };

    "constant modulus, two equal tones and a real sinusoid read their closed values"_test = [] {
        constexpr std::size_t kSamples = 262'144UZ;
        // An increment that is not a simple fraction of a turn spreads the phase evenly over the window.
        constexpr double kIncrement = 0.30901699437494745;

        // Constant modulus. `E|z|^4 = A^4` and `(E|z|^2)^2 = A^4`. Then `K = 1`.
        KurtosisAccumulator<Complex> constantModulus;
        for (std::size_t k = 0UZ; k < kSamples; ++k) {
            const double phase = kTwoPi * kIncrement * static_cast<double>(k);
            constantModulus.add(Complex{static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))});
        }
        expect(approx(constantModulus.normalized(), 1., 1e-6)) << "a constant-modulus stream reads K = 1 to float rounding, got" << constantModulus.normalized();
        expect(approx(constantModulus.excess(), -1., 1e-6));

        // Two equal complex tones. `|z|^2 = 2 + 2 cos(D)` with `D` uniform. Then `E|z|^2 = 2`, `E|z|^4 = 6` and
        // `K = 3/2`.
        KurtosisAccumulator<Complex> twoTones;
        for (std::size_t k = 0UZ; k < kSamples; ++k) {
            const double a = kTwoPi * 0.1 * static_cast<double>(k);
            const double b = kTwoPi * (0.1 + kIncrement) * static_cast<double>(k);
            twoTones.add(Complex{static_cast<float>(std::cos(a) + std::cos(b)), static_cast<float>(std::sin(a) + std::sin(b))});
        }
        expect(approx(twoTones.normalized(), 1.5, 1e-4)) << "two equal complex tones read K = 3/2, got" << twoTones.normalized();
        expect(approx(twoTones.excess(), -0.5, 1e-4)) << "an excess of -0.5";

        // A real sinusoid. `m4 = 3A^4/8` and `m2 = A^2/2`. Then `K = 3/2` and the excess is -1.5.
        KurtosisAccumulator<float> sinusoid;
        for (std::size_t k = 0UZ; k < kSamples; ++k) {
            sinusoid.add(static_cast<float>(std::cos(kTwoPi * kIncrement * static_cast<double>(k))));
        }
        expect(approx(sinusoid.normalized(), 1.5, 1e-4)) << "a real sinusoid reads K = 3/2, got" << sinusoid.normalized();
        expect(approx(sinusoid.excess(), -1.5, 1e-4));

        expect(lt(constantModulus.excess(), 0.)) << "a bounded signal reads a negative excess";
        expect(lt(twoTones.excess(), 0.));
        expect(lt(sinusoid.excess(), 0.));
    };

    "an impulsive stream reads its closed-form positive excess"_test = [] {
        constexpr std::size_t kWindows = 16UZ;
        constexpr std::size_t kWindow  = 1UZ << 18;

        struct Point {
            double p;
            double amplitude;
        };
        constexpr std::array<Point, 3> points{{{0.001, 10.}, {0.01, 5.}, {0.001, 20.}}};

        for (const Point& point : points) {
            gr::rng::Xoshiro256pp         rng(0xC0FFEEU);
            gr::rng::GaussianNoise<float> noise(rng);
            std::vector<float>            window(kWindow);
            std::vector<double>           readings(kWindows);

            for (std::size_t w = 0UZ; w < kWindows; ++w) {
                noise.fill(window);
                for (float& sample : window) {
                    if (rng.uniform01<float>() < static_cast<float>(point.p)) {
                        sample += static_cast<float>(point.amplitude);
                    }
                }
                KurtosisAccumulator<float> accumulator;
                accumulator.add(std::span<const float>(window));
                readings[w] = accumulator.excess();
            }

            const Spread measured = spreadOf(readings);
            const double closed   = impulsiveExcess(point.p, point.amplitude);
            const double band     = 4. * measured.standardError(kWindows);
            std::println("impulsive p = {:.3f}, A = {:4.1f}: mean excess {:.4f} against the closed form {:.4f}, band +/-{:.4f}", point.p, point.amplitude, measured.mean, closed, band);
            expect(std::abs(measured.mean - closed) < band) << "the mean lies within four standard errors of the closed form";
            expect(gt(measured.mean, 0.)) << "and the excess is positive";
        }
    };

    "a degenerate window is flagged and reads zero"_test = [] {
        KurtosisAccumulator<Complex> accumulator;
        expect(accumulator.degenerate()) << "an empty accumulator is degenerate";
        expect(accumulator.normalized() == 0.) << "and reads zero";
        expect(accumulator.excess() == 0.);
        expect(accumulator.meanPower() == 0.);

        const std::vector<Complex> zeros(1'024UZ);
        accumulator.add(std::span<const Complex>(zeros));
        expect(accumulator.count() == 1'024UZ);
        expect(accumulator.degenerate()) << "an all-zero window is degenerate";
        expect(accumulator.normalized() == 0.);

        accumulator.reset();
        expect(accumulator.count() == 0UZ);
        expect(accumulator.sumPower() == 0.);
    };

    "a NaN or infinite sample makes the window degenerate, and every figure reads zero"_test = [] {
        const auto check = [](const auto& accumulator, std::string_view what) {
            expect(accumulator.degenerate()) << what << "is degenerate";
            expect(!accumulator.finite()) << what << "is not finite";
            expect(accumulator.normalized() == 0.) << what << "reads K = 0, got" << accumulator.normalized();
            expect(accumulator.excess() == 0.) << what << "reads an excess of 0, got" << accumulator.excess();
            expect(accumulator.meanPower() == 0.) << what << "reads a mean power of 0, got" << accumulator.meanPower();
        };
        std::vector<float> window(64UZ, 0.5f);
        window[10] = -1.f;

        KurtosisAccumulator<float> finiteWindow;
        finiteWindow.add(std::span<const float>(window));
        expect(finiteWindow.finite() && !finiteWindow.degenerate()) << "the window without a bad sample is finite";

        window[20] = std::numeric_limits<float>::quiet_NaN();
        KurtosisAccumulator<float> withNan;
        withNan.add(std::span<const float>(window));
        check(withNan, "a real window with a NaN sample");

        window[20] = std::numeric_limits<float>::infinity();
        KurtosisAccumulator<float> withInfinity;
        withInfinity.add(std::span<const float>(window));
        check(withInfinity, "a real window with an infinite sample");

        KurtosisAccumulator<Complex> complexNan;
        complexNan.add(Complex{0.5f, 0.25f});
        complexNan.add(Complex{0.f, std::numeric_limits<float>::quiet_NaN()});
        complexNan.add(Complex{-0.5f, 0.25f});
        check(complexNan, "a complex window with a NaN imaginary part");

        // |x|^4 overflows double above about 1.2e77.
        KurtosisAccumulator<double> overflow;
        overflow.add(1.);
        overflow.add(1e200);
        check(overflow, "a double window whose fourth power overflows");
    };

    "any split of the stream gives the same sums"_test = [] {
        gr::rng::Xoshiro256pp         rng(7ULL);
        gr::rng::GaussianNoise<float> noise(rng);
        std::vector<Complex>          window(4'096UZ);
        noise.fillComplex(window);

        KurtosisAccumulator<Complex> bulk;
        bulk.add(std::span<const Complex>(window));

        KurtosisAccumulator<Complex> chunked;
        for (const std::size_t chunk : {1UZ, 7UZ, 1'000UZ}) {
            chunked.reset();
            for (std::size_t at = 0UZ; at < window.size(); at += chunk) {
                chunked.add(std::span<const Complex>(window).subspan(at, std::min(chunk, window.size() - at)));
            }
            expect(chunked.sumPower() == bulk.sumPower()) << "the sums are bit-identical for every chunk size";
            expect(chunked.sumPowerSquared() == bulk.sumPowerSquared());
        }

        KurtosisAccumulator<Complex> scalar;
        for (const Complex sample : window) {
            scalar.add(sample);
        }
        expect(scalar.sumPower() == bulk.sumPower()) << "and the scalar add gives the same sums";
    };

    "spectral kurtosis reads 1 on noise at every shape"_test = [] {
        constexpr std::size_t kBins    = 2'000UZ;
        constexpr std::size_t kSpectra = 64UZ;

        for (const std::size_t shape : {1UZ, 4UZ, 16UZ}) {
            gr::rng::Xoshiro256pp         rng(0xA11CEULL + shape);
            gr::rng::GaussianNoise<float> noise(rng);

            SpectralKurtosisAccumulator accumulator(kBins);
            std::vector<float>          spectrum(kBins);
            for (std::size_t m = 0UZ; m < kSpectra; ++m) {
                for (float& bin : spectrum) {
                    bin = gammaPower(noise, shape);
                }
                expect(accumulator.accumulate(spectrum) == SpectrumResponse::accepted);
            }

            std::vector<double> sk(kBins);
            expect(accumulator.evaluate(static_cast<double>(shape), sk) == 0UZ) << "no bin is degenerate on noise";

            const Spread measured = spreadOf(sk);
            const double closed   = spectralKurtosisSpread(kSpectra, static_cast<double>(shape));
            const double band     = 4. * closed / std::sqrt(static_cast<double>(kBins));
            std::println("SK at M = {}, d = {:2}: mean {:.5f} against the exact 1 (band +/-{:.5f}); spread {:.5f} against {:.5f}", kSpectra, shape, measured.mean, band, measured.sigma, closed);
            expect(std::abs(measured.mean - spectralKurtosisExpectation()) < band) << "the mean lies within four standard errors of 1";
            expect(std::abs(measured.sigma - closed) < 0.1 * closed) << "and the spread matches the closed-form variance";
        }

        // The closed-form spread at four points.
        expect(approx(spectralKurtosisSpread(64UZ, 1.), 0.24251, 1e-4));
        expect(approx(spectralKurtosisSpread(64UZ, 4.), 0.19728, 1e-4));
        expect(approx(spectralKurtosisSpread(64UZ, 16.), 0.18321, 1e-4));
        expect(approx(spectralKurtosisSpread(16UZ, 1.), 0.44678, 1e-4));
    };

    "spectral kurtosis reads exactly 0 on a noiseless tone"_test = [] {
        constexpr std::size_t kBins = 4UZ;

        for (const std::size_t spectra : {8UZ, 64UZ, 1'024UZ}) {
            SpectralKurtosisAccumulator accumulator(kBins);
            const std::vector<float>    spectrum{1.f, 3.f, 0.25f, 128.f};
            for (std::size_t m = 0UZ; m < spectra; ++m) {
                expect(accumulator.accumulate(spectrum) == SpectrumResponse::accepted);
            }

            std::vector<double> sk(kBins);
            expect(accumulator.evaluate(1., sk).has_value());
            for (const double value : sk) {
                // Identical bin powers give `M*S2/S1^2 = 1` exactly, and the comparison is exact.
                expect(value == 0.) << "a noiseless tone reads SK = 0 at M =" << spectra;
            }
        }
    };

    "a carrier in noise reads its closed form, and a pulsed signal reads its limit with the first-order bias"_test = [] {
        constexpr std::size_t kBins    = 200UZ;
        constexpr std::size_t kSpectra = 1'024UZ;

        for (const double rho : {1., 2., 10., 100.}) {
            gr::rng::Xoshiro256pp         rng(0xBEEFULL + static_cast<std::uint64_t>(rho));
            gr::rng::GaussianNoise<float> noise(rng);

            SpectralKurtosisAccumulator accumulator(kBins);
            std::vector<float>          spectrum(kBins);
            const float                 amplitude = static_cast<float>(std::sqrt(rho));
            for (std::size_t m = 0UZ; m < kSpectra; ++m) {
                for (float& bin : spectrum) {
                    const Complex z = Complex{amplitude, 0.f} + noise.complexSample();
                    bin             = z.real() * z.real() + z.imag() * z.imag();
                }
                (void)accumulator.accumulate(spectrum);
            }

            std::vector<double> sk(kBins);
            expect(accumulator.evaluate(1., sk).has_value());
            const Spread measured = spreadOf(sk);
            const double closed   = spectralKurtosisCw(rho);
            std::println("SK on CW at rho = {:5.1f}: mean {:.5f} against (2*rho+1)/(rho+1)^2 = {:.5f}", rho, measured.mean, closed);
            expect(std::abs(measured.mean - closed) < 0.01) << "the carrier-in-noise closed form";
            expect(lt(measured.mean, 1.)) << "a steady carrier reads below 1";
        }

        // A pulsed signal of bin power `S`, present a fraction `p` of the time, over noise of unit power. At S = 100 the
        // large-M limit is 7.6116, 2.8491 and 1.0000 at p = 0.1, 0.25 and 0.5, and the first-order bias `b1/M` is
        // 65.0/M, 15.2/M and 3.8/M. The bins are independent readings. The bound is four standard errors of their
        // mean, from their own spread.
        expect(approx(pulsedLimit(0.5, 100.).limit, 1., 1e-12)) << "the limit is exactly 1 at one half";
        expect(approx(pulsedLimit(0., 100.).bias, 0., 1e-12)) << "noise alone has no first-order bias";
        for (const double p : {0.1, 0.25, 0.5}) {
            constexpr double  kSignal = 100.;
            const PulsedLimit closed  = pulsedLimit(p, kSignal);

            for (const std::size_t spectra : {1'024UZ, 8'192UZ}) {
                gr::rng::Xoshiro256pp         rng(0xFEEDULL + static_cast<std::uint64_t>(100. * p));
                gr::rng::GaussianNoise<float> noise(rng);

                SpectralKurtosisAccumulator accumulator(kBins);
                std::vector<float>          spectrum(kBins);
                for (std::size_t m = 0UZ; m < spectra; ++m) {
                    for (float& bin : spectrum) {
                        const bool    present = rng.uniform01<double>() < p;
                        const Complex z       = Complex{present ? static_cast<float>(std::sqrt(kSignal)) : 0.f, 0.f} + noise.complexSample();
                        bin                   = z.real() * z.real() + z.imag() * z.imag();
                    }
                    (void)accumulator.accumulate(spectrum);
                }

                std::vector<double> sk(kBins);
                expect(accumulator.evaluate(1., sk).has_value());
                const Spread measured = spreadOf(sk);
                const double expected = closed.limit + closed.bias / static_cast<double>(spectra);
                const double band     = 4. * measured.standardError(kBins);
                std::println("SK on a pulsed signal at p = {:.2f}, M = {:5}: mean {:.4f} against L + b1/M = {:.4f} (L {:.4f}), band +/-{:.4f}", p, spectra, measured.mean, expected, closed.limit, band);

                expect(std::abs(measured.mean - expected) < band) << "the mean lies within four standard errors of L + b1/M";
                if (p < 0.5) {
                    expect(gt(measured.mean, 1.)) << "a signal present less than half the time reads above 1";
                }
            }
        }
    };

    "the spectral accumulator refuses unusable spectra and counts bins with no power"_test = [] {
        SpectralKurtosisAccumulator accumulator(4UZ);
        expect(accumulator.bins() == 4UZ);
        expect(accumulator.count() == 0UZ);

        const std::vector<float> wrongSize(5UZ, 1.f);
        expect(accumulator.accumulate(wrongSize) == SpectrumResponse::wrongBinCount);
        expect(accumulator.count() == 0UZ) << "a refused spectrum folds nothing in";

        const std::vector<float> negative{1.f, -1.f, 1.f, 1.f};
        expect(accumulator.accumulate(negative) == SpectrumResponse::invalidBin);
        expect(accumulator.count() == 0UZ);
        expect(accumulator.nRefused() == 2ULL);

        const std::vector<float> infinite{1.f, std::numeric_limits<float>::infinity(), 1.f, 1.f};
        const std::vector<float> notANumber{1.f, 1.f, std::numeric_limits<float>::quiet_NaN(), 1.f};
        expect(accumulator.accumulate(infinite) != SpectrumResponse::accepted) << "an infinite bin is refused";
        expect(accumulator.accumulate(notANumber) != SpectrumResponse::accepted) << "a NaN bin is refused";
        expect(accumulator.count() == 0UZ) << "and neither folds anything in";
        expect(accumulator.nRefused() == 4ULL);

        const std::vector<float> dead{1.f, 0.f, 1.f, 1.f};
        for (std::size_t m = 0UZ; m < 16UZ; ++m) {
            expect(accumulator.accumulate(dead) == SpectrumResponse::accepted);
        }
        std::vector<double> sk(4UZ);
        expect(accumulator.evaluate(1., sk) == 1UZ) << "the bin with no power is counted";
        expect(sk[1] == 0.) << "and written as zero";

        std::vector<double> tooFew(3UZ);
        const auto          refused = accumulator.evaluate(1., tooFew);
        expect(fatal(!refused.has_value())) << "an output span of the wrong size is an error";
        expect(refused.error().message.contains("one entry per bin")) << refused.error().message;

        expect(spectralKurtosis(0., 0., 64UZ, 1.) == 0.) << "a bin with no power reads zero";
        expect(spectralKurtosisVariance(1UZ, 1.) == 0.) << "and the variance of one spectrum reads zero";
    };
};

int main() { /* not needed for UT */ }
