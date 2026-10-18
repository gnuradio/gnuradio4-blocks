#include <benchmark.hpp>

#include <gnuradio-4.0/algorithm/analysis/Autocorrelation.hpp>
#include <gnuradio-4.0/algorithm/rng/GaussianNoise.hpp>
#include <gnuradio-4.0/algorithm/rng/Xoshiro256pp.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <format>
#include <span>
#include <string_view>
#include <vector>

namespace {

using Complex = std::complex<float>;

/// Input for at least sixteen windows at every geometry of the grid.
constexpr std::size_t kSamples = 262'144UZ;
constexpr std::size_t kRepeats = 5UZ;

struct Geometry {
    std::size_t windowLength;
    std::size_t maxLag;
    std::size_t nAverages;
};

constexpr std::array<Geometry, 3> kGeometries{{{4096UZ, 1024UZ, 16UZ}, {8192UZ, 2048UZ, 16UZ}, {16384UZ, 4096UZ, 16UZ}}};
constexpr std::array<double, 2>   kOverlaps{0., 0.5};

[[nodiscard]] std::vector<Complex> noise(std::size_t n) {
    gr::rng::Xoshiro256pp         rng(0xACF0ULL);
    gr::rng::GaussianNoise<float> gauss(rng);
    std::vector<Complex>          out(n);
    for (Complex& sample : out) {
        sample = gauss.complexSample();
    }
    return out;
}

} // namespace

/**
 * @brief Nanoseconds per input sample over the estimator's own grid, and the transform's share of it.
 *
 * One window costs one transform pair of length `M = acfTransformLength(N, L)` plus `O(M)` for the taper, the
 * magnitude squaring and the accumulation. An input sample therefore costs about `2 * 5 M log2(M) / H` real
 * operations. The last rows time the transform pair alone at each `M`. A grid row minus its transform row is the cost
 * beyond the transform.
 */
void benchAutocorrelation() {
    using namespace benchmark;
    using boost::ut::operator""_test;

    "Autocorrelation: ns per input sample over the grid"_test = [] {
        const std::vector<Complex> input = noise(kSamples);

        for (const Geometry& geometry : kGeometries) {
            for (const gr::analysis::AcfKind kind : {gr::analysis::AcfKind::Complex, gr::analysis::AcfKind::Envelope}) {
                for (const gr::analysis::AcfNormalization normalization : {gr::analysis::AcfNormalization::Unbiased, gr::analysis::AcfNormalization::Biased}) {
                    for (const double overlap : kOverlaps) {
                        gr::analysis::AcfConfig config{};
                        config.windowLength  = geometry.windowLength;
                        config.maxLag        = geometry.maxLag;
                        config.kind          = kind;
                        config.normalization = normalization;
                        config.overlap       = overlap;
                        config.nAverages     = geometry.nAverages;

                        gr::analysis::Autocorrelation<Complex> kernel;
                        kernel.prepare(config);

                        double     lastPower = 0.;
                        const auto sink      = [&lastPower](const gr::analysis::AcfResult& result) { lastPower = result.zeroLag; };
                        const auto name      = std::format("N={:>5} L={:>4} K={} {:>8} {:>8} overlap={:.1f} M={:>5}", geometry.windowLength, geometry.maxLag, geometry.nAverages, gr::analysis::acfKindName(kind), gr::analysis::acfNormalizationName(normalization), overlap, kernel.transformLength());

                        ::benchmark::benchmark<kRepeats>(std::string_view(name), kSamples) = [&] {
                            kernel.reset();
                            std::size_t at = 0UZ;
                            while (at < input.size()) {
                                const std::size_t used = kernel.process(std::span<const Complex>(input).subspan(at), sink);
                                if (used == 0UZ) {
                                    break;
                                }
                                at += used;
                            }
                            ::benchmark::force_to_memory(lastPower);
                        };
                    }
                }
            }
            ::benchmark::results::add_separator();
        }
    };

    "the transform pair alone, at the same lengths and per the same input sample"_test = [] {
        for (const Geometry& geometry : kGeometries) {
            const std::size_t transformLength = gr::analysis::acfTransformLength(geometry.windowLength, geometry.maxLag);
            for (const double overlap : kOverlaps) {
                const std::size_t hop     = std::max(1UZ, static_cast<std::size_t>(std::llround(static_cast<double>(geometry.windowLength) * (1. - overlap))));
                const std::size_t windows = kSamples / hop;

                gr::algorithm::FFT<std::complex<double>, std::complex<double>, gr::algorithm::Direction::Forward>  forward{};
                gr::algorithm::FFT<std::complex<double>, std::complex<double>, gr::algorithm::Direction::Backward> backward{};
                std::vector<std::complex<double>, gr::allocator::Aligned<std::complex<double>>>                    work(transformLength);
                std::vector<std::complex<double>, gr::allocator::Aligned<std::complex<double>>>                    spectrum(transformLength);
                std::vector<std::complex<double>, gr::allocator::Aligned<std::complex<double>>>                    correlation(transformLength);
                for (std::size_t k = 0UZ; k < geometry.windowLength; ++k) {
                    work[k] = std::complex<double>(static_cast<double>(k % 17UZ) - 8., static_cast<double>(k % 23UZ) - 11.);
                }

                const auto name = std::format("transform pair alone     M={:>5} hop={:>5} overlap={:.1f}", transformLength, hop, overlap);

                ::benchmark::benchmark<kRepeats>(std::string_view(name), windows * hop) = [&] {
                    // The backward transform writes to its own buffer. Every iteration therefore transforms the
                    // same window. A correlation fed back as input would grow in scale by a factor of the transform
                    // length on each iteration.
                    for (std::size_t window = 0UZ; window < windows; ++window) {
                        forward.compute(work, spectrum);
                        for (std::complex<double>& bin : spectrum) {
                            bin = std::complex<double>(bin.real() * bin.real() + bin.imag() * bin.imag(), 0.);
                        }
                        backward.compute(spectrum, correlation);
                    }
                    ::benchmark::force_to_memory(correlation[0UZ]);
                };
            }
            ::benchmark::results::add_separator();
        }
    };
}

inline const boost::ut::suite<"autocorrelation benchmarks"> _autocorrelation_bm = [] { benchAutocorrelation(); };

int main() { /* not needed by the UT framework */ }
