#include <benchmark.hpp>

#include <gnuradio-4.0/algorithm/measurement/Kurtosis.hpp>
#include <gnuradio-4.0/algorithm/rng/GaussianNoise.hpp>
#include <gnuradio-4.0/algorithm/rng/Xoshiro256pp.hpp>

#include <complex>
#include <cstddef>
#include <format>
#include <span>
#include <string_view>
#include <tuple>
#include <vector>

namespace {

using Complex = std::complex<float>;

constexpr std::size_t kSamples = 65'536UZ;
constexpr std::size_t kRepeats = 200UZ;

} // namespace

/// Times the two accumulators. Per sample, `KurtosisAccumulator` computes `|x|^2`, squares it and adds both to the
/// sums. `|x|^2` costs two multiplies and an add for complex input and one multiply for real input.
void benchKurtosis() {
    using namespace benchmark;
    using boost::ut::operator""_test;

    "KurtosisAccumulator: ns/sample"_test = [] {
        gr::rng::Xoshiro256pp         rng(1ULL);
        gr::rng::GaussianNoise<float> noise(rng);

        std::vector<Complex> complexIn(kSamples);
        std::vector<float>   realIn(kSamples);
        noise.fillComplex(complexIn);
        noise.fill(realIn);

        gr::measurement::KurtosisAccumulator<Complex> complexAccumulator;
        const auto                                    complexName                 = std::format("KurtosisAccumulator<complex<float>>, N={}", kSamples);
        ::benchmark::benchmark<kRepeats>(std::string_view(complexName), kSamples) = [&] {
            complexAccumulator.reset();
            complexAccumulator.add(std::span<const Complex>(complexIn));
            ::benchmark::force_to_memory(complexAccumulator);
        };

        gr::measurement::KurtosisAccumulator<float> realAccumulator;
        const auto                                  realName                   = std::format("KurtosisAccumulator<float>,           N={}", kSamples);
        ::benchmark::benchmark<kRepeats>(std::string_view(realName), kSamples) = [&] {
            realAccumulator.reset();
            realAccumulator.add(std::span<const float>(realIn));
            ::benchmark::force_to_memory(realAccumulator);
        };
        ::benchmark::results::add_separator();
    };

    "SpectralKurtosisAccumulator: ns/bin per record"_test = [] {
        constexpr std::size_t kBins = 4'096UZ;

        gr::rng::Xoshiro256pp         rng(2ULL);
        gr::rng::GaussianNoise<float> noise(rng);
        std::vector<Complex>          draw(kBins);
        noise.fillComplex(draw);

        std::vector<float> spectrum(kBins);
        for (std::size_t bin = 0UZ; bin < kBins; ++bin) {
            spectrum[bin] = draw[bin].real() * draw[bin].real() + draw[bin].imag() * draw[bin].imag();
        }

        gr::measurement::SpectralKurtosisAccumulator accumulator(kBins);
        const auto                                   name               = std::format("SpectralKurtosisAccumulator::accumulate, {} bins", kBins);
        ::benchmark::benchmark<kRepeats>(std::string_view(name), kBins) = [&] { (void)accumulator.accumulate(spectrum); };

        std::vector<double> sk(kBins);
        const auto          evaluate                                        = std::format("SpectralKurtosisAccumulator::evaluate,   {} bins", kBins);
        ::benchmark::benchmark<kRepeats>(std::string_view(evaluate), kBins) = [&] { std::ignore = accumulator.evaluate(1., sk); };
        ::benchmark::results::add_separator();
    };
}

inline const boost::ut::suite<"kurtosis benchmarks"> _kurtosis_bm = [] { benchKurtosis(); };

int main() { /* not needed by the UT framework */ }
