#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <format>
#include <limits>
#include <numeric>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/algorithm/rng/GaussianNoise.hpp>
#include <gnuradio-4.0/algorithm/rng/Xoshiro256pp.hpp>

using namespace boost::ut;

const boost::ut::suite noiseGeneratorTests = [] {
    using gr::rng::GaussianNoise;
    using gr::rng::Xoshiro256pp;

    "real Gaussian N(0,1) mean and variance"_test = [] {
        Xoshiro256pp          rng(42);
        GaussianNoise<double> gauss(rng);
        constexpr int         N   = 100'000;
        double                sum = 0.0;
        double                sq  = 0.0;
        for (int i = 0; i < N; ++i) {
            const double v = gauss();
            sum += v;
            sq += v * v;
        }
        const double mean     = sum / N;
        const double variance = sq / N - mean * mean;
        expect(approx(mean, 0.0, 0.02)) << std::format("mean {:.6f} not near 0", mean);
        expect(approx(variance, 1.0, 0.05)) << std::format("variance {:.6f} not near 1", variance);
    };

    "real Gaussian<float> mean and variance"_test = [] {
        Xoshiro256pp         rng(77);
        GaussianNoise<float> gauss(rng);
        constexpr int        N   = 100'000;
        double               sum = 0.0;
        double               sq  = 0.0;
        for (int i = 0; i < N; ++i) {
            const double v = static_cast<double>(gauss());
            sum += v;
            sq += v * v;
        }
        const double mean     = sum / N;
        const double variance = sq / N - mean * mean;
        expect(approx(mean, 0.0, 0.02)) << std::format("float Gaussian mean {:.6f}", mean);
        expect(approx(variance, 1.0, 0.05)) << std::format("float Gaussian variance {:.6f}", variance);
    };

    "Gaussian determinism"_test = [] {
        Xoshiro256pp          rng1(123);
        Xoshiro256pp          rng2(123);
        GaussianNoise<double> g1(rng1);
        GaussianNoise<double> g2(rng2);
        for (int i = 0; i < 1000; ++i) {
            expect(eq(g1(), g2())) << std::format("mismatch at {}", i);
        }
    };

    // the polar method yields variates in pairs, so a call that ends on an odd count leaves a spare. fill()
    // must resume from it: opening at 'false' while still writing the state back made an odd-sized split draw a
    // different sequence than one call, which every seeded model built on this generator depends on not happening
    "bulk fills are chunk independent"_test = [] {
        constexpr std::size_t nSamples = 64UZ;

        const auto viaFill = [](std::size_t chunk) {
            Xoshiro256pp         rng(12345);
            GaussianNoise<float> gauss(rng);
            std::vector<float>   out(nSamples);
            for (std::size_t i = 0UZ; i < nSamples; i += chunk) {
                gauss.fill(std::span<float>(out.data() + i, std::min(chunk, nSamples - i)));
            }
            return out;
        };
        const auto viaFillComplex = [](std::size_t chunk) {
            Xoshiro256pp                     rng(12345);
            GaussianNoise<float>             gauss(rng);
            std::vector<std::complex<float>> out(nSamples);
            for (std::size_t i = 0UZ; i < nSamples; i += chunk) {
                gauss.fillComplex(std::span<std::complex<float>>(out.data() + i, std::min(chunk, nSamples - i)));
            }
            return out;
        };

        const auto reference        = viaFill(nSamples);
        const auto referenceComplex = viaFillComplex(nSamples);
        for (const std::size_t chunk : {1UZ, 2UZ, 3UZ, 5UZ, 8UZ, 17UZ}) {
            expect(viaFill(chunk) == reference) << std::format("fill in {}-sample calls must match one call", chunk);
            expect(viaFillComplex(chunk) == referenceComplex) << std::format("fillComplex in {}-sample calls must match one call", chunk);
        }
    };

    // complexSample() takes the log through std::log. gr::meta's float log is within 2 ulp of the exact result, and
    // the test takes the C library's within 2. With an ulp at most 2^-23 of a value, the two logs differ by at most
    // 8 times 2^-24 of either. The divide adds 2, the square root halves the sum and adds 2, and the
    // products with u, 1/sqrt(2) and the amplitude add 2 each: 13 times 2^-24 a part.
    "fillComplex<float> follows complexSample() within the log's bound"_test = [] {
        constexpr double      kUnits   = 13.;
        constexpr std::size_t nSamples = 1000UZ;
        for (const float amplitude : {1.f, 0.3f}) {
            for (const std::size_t chunk : {nSamples, 257UZ, 7UZ, 1UZ}) {
                Xoshiro256pp                     rng(2024);
                Xoshiro256pp                     rngReference(2024);
                GaussianNoise<float>             gauss(rng);
                GaussianNoise<float>             reference(rngReference);
                std::vector<std::complex<float>> out(nSamples);
                for (std::size_t i = 0UZ; i < nSamples; i += chunk) {
                    gauss.fillComplex(std::span<std::complex<float>>(out.data() + i, std::min(chunk, nSamples - i)), amplitude);
                }
                std::size_t outside{0UZ};
                std::string firstOutside;
                for (std::size_t i = 0UZ; i < nSamples; ++i) {
                    const std::complex<float> want = reference.complexSample();
                    for (const auto& [got, part] : {std::pair{out[i].real(), want.real()}, std::pair{out[i].imag(), want.imag()}}) {
                        const float  expected = amplitude * part;
                        const double bound    = kUnits * std::ldexp(std::abs(static_cast<double>(expected)), -24) + 2. * static_cast<double>(std::numeric_limits<float>::denorm_min());
                        if (std::abs(static_cast<double>(got) - static_cast<double>(expected)) > bound && outside++ == 0UZ) {
                            firstOutside = std::format("sample {}: {} against {}", i, got, expected);
                        }
                    }
                }
                expect(eq(outside, 0UZ)) << std::format("amplitude {} chunk {}: {} parts outside the bound, first {}", amplitude, chunk, outside, firstOutside);
                expect(std::ranges::equal(rng._state, rngReference._state)) << std::format("amplitude {} chunk {}: the generator state follows the same draws", amplitude, chunk);
                expect(eq(gauss(), reference())) << std::format("amplitude {} chunk {}: the next real draw matches", amplitude, chunk);
            }
        }
    };

    "complex Gaussian Option B: E[|n|^2] ~ 1"_test = [] {
        Xoshiro256pp          rng(55);
        GaussianNoise<double> gauss(rng);
        constexpr int         N         = 100'000;
        double                powerSum  = 0.0;
        double                realSqSum = 0.0;
        double                imagSqSum = 0.0;
        for (int i = 0; i < N; ++i) {
            const auto sample = gauss.complexSample();
            powerSum += std::norm(sample); // |n|^2
            realSqSum += sample.real() * sample.real();
            imagSqSum += sample.imag() * sample.imag();
        }
        const double meanPower = powerSum / N;
        const double realVar   = realSqSum / N;
        const double imagVar   = imagSqSum / N;
        expect(approx(meanPower, 1.0, 0.05)) << std::format("E[|n|^2] = {:.6f}, expected ~1", meanPower);
        expect(approx(realVar, 0.5, 0.05)) << std::format("Var(real) = {:.6f}, expected ~0.5", realVar);
        expect(approx(imagVar, 0.5, 0.05)) << std::format("Var(imag) = {:.6f}, expected ~0.5", imagVar);
    };

    "complex Gaussian<float> Option B"_test = [] {
        Xoshiro256pp         rng(88);
        GaussianNoise<float> gauss(rng);
        constexpr int        N        = 100'000;
        double               powerSum = 0.0;
        for (int i = 0; i < N; ++i) {
            const auto sample = gauss.complexSample();
            powerSum += static_cast<double>(std::norm(sample));
        }
        const double meanPower = powerSum / N;
        expect(approx(meanPower, 1.0, 0.05)) << std::format("float complex E[|n|^2] = {:.6f}", meanPower);
    };

    "Gaussian samples are not all identical"_test = [] {
        Xoshiro256pp          rng(0);
        GaussianNoise<double> gauss(rng);
        const double          first     = gauss();
        bool                  different = false;
        for (int i = 0; i < 100; ++i) {
            if (gauss() != first) {
                different = true;
                break;
            }
        }
        expect(different) << "all Gaussian samples identical";
    };

    "uniform noise range and mean"_test = [] {
        Xoshiro256pp  rng(31);
        constexpr int N      = 100'000;
        double        sum    = 0.0;
        double        minVal = 0.0;
        double        maxVal = 0.0;
        for (int i = 0; i < N; ++i) {
            const double v = rng.uniformM11<double>();
            sum += v;
            minVal = std::min(minVal, v);
            maxVal = std::max(maxVal, v);
        }
        expect(ge(minVal, -1.0)) << "uniform below -1";
        expect(lt(maxVal, 1.0)) << "uniform above +1";
        expect(approx(sum / N, 0.0, 0.01)) << std::format("uniform mean {:.6f}", sum / N);
    };

    "triangular noise range and mean"_test = [] {
        Xoshiro256pp  rng(31);
        constexpr int N      = 100'000;
        double        sum    = 0.0;
        double        minVal = 0.0;
        double        maxVal = 0.0;
        for (int i = 0; i < N; ++i) {
            const double v = rng.triangularM11<double>();
            sum += v;
            minVal = std::min(minVal, v);
            maxVal = std::max(maxVal, v);
        }
        expect(ge(minVal, -1.0)) << "triangular below -1";
        expect(lt(maxVal, 1.0)) << "triangular above +1";
        expect(approx(sum / N, 0.0, 0.01)) << std::format("triangular mean {:.6f}", sum / N);
    };
};

int main() { /* not needed for UT */ }
