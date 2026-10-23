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
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <magic_enum.hpp>

#include <gnuradio-4.0/algorithm/analysis/Autocorrelation.hpp>
#include <gnuradio-4.0/algorithm/rng/GaussianNoise.hpp>
#include <gnuradio-4.0/algorithm/rng/Xoshiro256pp.hpp>

using namespace boost::ut;
using gr::analysis::AcfConfig;
using gr::analysis::AcfKind;
using gr::analysis::acfLagTail;
using gr::analysis::acfMedianLowerBound;
using gr::analysis::acfNormalDeviate;
using gr::analysis::AcfNormalization;
using gr::analysis::acfPeakThresholdDb;
using gr::analysis::acfRealThreshold;
using gr::analysis::acfRecordThreshold;
using gr::analysis::AcfResult;
using gr::analysis::acfScatter;
using gr::analysis::acfThreshold;
using gr::analysis::acfTransformLength;

namespace {

using Complex           = std::complex<float>;
constexpr double kTwoPi = 2. * std::numbers::pi_v<double>;

/// One published record, copied out of the spans the kernel passes to its sink.
struct Record {
    std::vector<float> magnitude{};
    std::vector<float> phase{};
    double             zeroLag{};
    double             scatter{};
    double             threshold{};
    double             peakThresholdDb{};
    std::uint64_t      sampleStart{};
    std::size_t        nAveraged{};
    std::uint64_t      nDistinct{};
    double             nPairs{};
    std::size_t        scatterLag{};
    bool               realValued{};
};

[[nodiscard]] Record copyOf(const AcfResult& result) {
    return Record{
        .magnitude       = std::vector<float>(result.magnitude.begin(), result.magnitude.end()),
        .phase           = std::vector<float>(result.phase.begin(), result.phase.end()),
        .zeroLag         = result.zeroLag,
        .scatter         = result.scatter,
        .threshold       = result.threshold,
        .peakThresholdDb = result.peakThresholdDb,
        .sampleStart     = result.sampleStart,
        .nAveraged       = result.nAveraged,
        .nDistinct       = result.nDistinctSamples,
        .nPairs          = result.nPairs,
        .scatterLag      = result.scatterLag,
        .realValued      = result.realValued,
    };
}

/// @brief Drives a prepared kernel over one span, presenting the remainder again after each call.
template<typename T>
[[nodiscard]] std::vector<Record> drive(gr::analysis::Autocorrelation<T>& kernel, std::span<const T> in, bool flushTail = true) {
    std::vector<Record> records;
    const auto          sink = [&records](const AcfResult& result) { records.push_back(copyOf(result)); };
    std::size_t         at   = 0UZ;
    while (at < in.size()) {
        const std::size_t used = kernel.process(in.subspan(at), sink);
        if (used == 0UZ) {
            break;
        }
        at += used;
    }
    if (flushTail) {
        std::ignore = kernel.flush(sink);
    }
    return records;
}

/// @brief The whole estimate of one window by the direct `O(N L)` sum in double, the reference for the transform path.
///
/// It tapers the mean-removed sequence and sums the products at each lag. With the mean removed it adds
/// `r_w(tau) / (r_w(0) (N - 1))` of the zero lag to lag `tau`. It divides by the taper's own autocorrelation or by
/// its value at zero.
template<typename T>
[[nodiscard]] std::vector<double> directRatios(const AcfConfig& config, std::span<const T> window) {
    const std::size_t        n     = config.windowLength;
    const std::size_t        lags  = config.maxLag;
    const std::vector<float> taper = gr::algorithm::window::create<float>(config.window, n);

    std::vector<std::complex<double>> u(n);
    std::complex<double>              mean{};
    for (std::size_t k = 0UZ; k < n; ++k) {
        std::complex<double> value{};
        if constexpr (std::is_same_v<T, float>) {
            const double sample = static_cast<double>(window[k]);
            value               = config.kind == AcfKind::Envelope ? std::complex<double>(sample * sample, 0.) : std::complex<double>(sample, 0.);
        } else {
            const double re = static_cast<double>(window[k].real());
            const double im = static_cast<double>(window[k].imag());
            value           = config.kind == AcfKind::Envelope ? std::complex<double>(re * re + im * im, 0.) : std::complex<double>(re, im);
        }
        u[k] = value;
        mean += value;
    }
    mean /= static_cast<double>(n);
    if (!config.removeMean) {
        mean = std::complex<double>{};
    }
    for (std::size_t k = 0UZ; k < n; ++k) {
        u[k] = (u[k] - mean) * static_cast<double>(taper[k]);
    }

    std::vector<double>               ratios(lags + 1UZ, 0.);
    std::vector<double>               taperAcf(lags + 1UZ, 0.);
    std::vector<std::complex<double>> sums(lags + 1UZ);
    std::vector<double>               magnitudes(lags + 1UZ, 0.);
    for (std::size_t lag = 0UZ; lag <= lags; ++lag) {
        double rw{};
        for (std::size_t k = 0UZ; k + lag < n; ++k) {
            rw += static_cast<double>(taper[k]) * static_cast<double>(taper[k + lag]);
            sums[lag] += u[k + lag] * std::conj(u[k]);
        }
        taperAcf[lag] = rw;
    }
    for (std::size_t lag = 0UZ; lag <= lags; ++lag) {
        const double restored = config.removeMean && lag > 0UZ ? sums[0UZ].real() * taperAcf[lag] / (taperAcf[0UZ] * static_cast<double>(n - 1UZ)) : 0.;
        magnitudes[lag]       = std::abs(sums[lag] + restored);
    }
    for (std::size_t lag = 0UZ; lag <= lags; ++lag) {
        double ratio = magnitudes[0UZ] > 0. ? magnitudes[lag] / magnitudes[0UZ] : 0.;
        if (config.normalization == AcfNormalization::Unbiased) {
            ratio = taperAcf[lag] > 0. ? ratio * taperAcf[0UZ] / taperAcf[lag] : 0.;
        }
        ratios[lag] = ratio;
    }
    return ratios;
}

[[nodiscard]] std::vector<Complex> complexNoise(std::size_t n, std::uint64_t seed) {
    gr::rng::Xoshiro256pp         rng(seed);
    gr::rng::GaussianNoise<float> gauss(rng);
    std::vector<Complex>          out(n);
    for (Complex& sample : out) {
        sample = gauss.complexSample();
    }
    return out;
}

[[nodiscard]] std::vector<float> realNoise(std::size_t n, std::uint64_t seed) {
    gr::rng::Xoshiro256pp         rng(seed);
    gr::rng::GaussianNoise<float> gauss(rng);
    std::vector<float>            out(n);
    for (float& sample : out) {
        sample = gauss();
    }
    return out;
}

/// @brief The largest published ratio over lags `[first, last]`, and its lag.
[[nodiscard]] std::pair<std::size_t, double> peakOver(std::span<const float> magnitude, std::size_t first, std::size_t last) {
    std::size_t at   = first;
    double      best = -1.;
    for (std::size_t lag = first; lag <= last && lag < magnitude.size(); ++lag) {
        if (static_cast<double>(magnitude[lag]) > best) {
            best = static_cast<double>(magnitude[lag]);
            at   = lag;
        }
    }
    return {at, best};
}

/// @brief The scatter of each lag `0..L` of a record of rectangular windows `hop` apart. A product at lag `tau` carries
/// one unit of weight per window that holds it. The sum of the squared weights is then
/// `K (N - tau) + 2 sum_m (K - m) max(0, N - tau - m hop)`.
[[nodiscard]] std::vector<double> rectangularScatter(std::size_t n, std::size_t lags, std::size_t hop, std::size_t windows, AcfNormalization normalization) {
    std::vector<double> scatter(lags + 1UZ, 0.);
    for (std::size_t lag = 1UZ; lag <= lags; ++lag) {
        auto squares = static_cast<double>(windows * (n - lag));
        for (std::size_t m = 1UZ; m < windows && m * hop < n - lag; ++m) {
            squares += 2. * static_cast<double>(windows - m) * static_cast<double>(n - lag - m * hop);
        }
        const auto divisor = static_cast<double>(windows * (normalization == AcfNormalization::Biased ? n : n - lag));
        scatter[lag]       = std::sqrt(squares) / divisor;
    }
    return scatter;
}

[[nodiscard]] AcfConfig baseConfig(std::size_t n, std::size_t lags, AcfKind kind, AcfNormalization normalization) {
    AcfConfig config{};
    config.windowLength  = n;
    config.maxLag        = lags;
    config.kind          = kind;
    config.normalization = normalization;
    config.overlap       = 0.;
    config.nAverages     = 1UZ;
    return config;
}

} // namespace

const suite<"autocorrelation free functions"> _acfFunctions = [] {
    "the transform length is the next power of two at or above N + L"_test = [] {
        expect(eq(acfTransformLength(8192UZ, 2048UZ), 16384UZ));
        expect(eq(acfTransformLength(8192UZ, 4096UZ), 16384UZ));
        expect(eq(acfTransformLength(4096UZ, 1024UZ), 8192UZ));
        expect(eq(acfTransformLength(16384UZ, 2048UZ), 32768UZ));
        expect(eq(acfTransformLength(240UZ, 16UZ), 256UZ)) << "N + L exactly a power of two needs no further rounding";
        expect(eq(acfTransformLength(1024UZ, 1023UZ), 2048UZ));
        expect(eq(acfTransformLength(64UZ, 8UZ), 128UZ));
    };

    "the scatter is one over the root of the products behind the lag"_test = [] {
        expect(std::abs(acfScatter(4096UZ) - 1. / 64.) < 1e-15);
        expect(std::isinf(acfScatter(0UZ))) << "no products behind a lag give an infinite scatter";
    };

    "the detection multiple depends on the searched lags and the rate alone"_test = [] {
        // at one pair the threshold is the multiple sqrt(ln(L/p)) itself
        constexpr std::array<std::pair<std::size_t, double>, 4> kCases{{{256UZ, 3.186}, {1024UZ, 3.397}, {2048UZ, 3.497}, {4096UZ, 3.595}}};
        for (const auto& [lags, expected] : kCases) {
            const double multiple = acfThreshold(1UZ, lags, 1e-2);
            expect(std::abs(multiple - expected) < 1e-3) << std::format("L={} p=1e-2: multiple {:.4f} against {:.4f}", lags, multiple, expected);
        }
        constexpr std::array<std::pair<std::size_t, double>, 4> kTighter{{{256UZ, 3.529}, {1024UZ, 3.720}, {2048UZ, 3.812}, {4096UZ, 3.902}}};
        for (const auto& [lags, expected] : kTighter) {
            const double multiple = acfThreshold(1UZ, lags, 1e-3);
            expect(std::abs(multiple - expected) < 1e-3) << std::format("L={} p=1e-3: multiple {:.4f} against {:.4f}", lags, multiple, expected);
        }
        // and it scales as the scatter does
        expect(std::abs(acfThreshold(4096UZ, 1024UZ, 1e-3) - 3.720 / 64.) < 1e-4);
    };

    "the median's lower bound holds its tail by the exact binomial count"_test = [] {
        // With equal scatters the count of lags below a height is binomial. The median of the L lags and lag 0 lies
        // below the height when floor((L+1)/2) + 1 lags do.
        constexpr double scatter = 0.03;
        for (const std::size_t lags : {16UZ, 255UZ, 1024UZ}) {
            for (const bool real : {false, true}) {
                for (const double tail : {5e-3, 5e-2}) {
                    const std::vector<double> equal(lags, scatter);
                    const double              height = acfMedianLowerBound(equal, tail, real);
                    const double              below  = 1. - acfLagTail(height, scatter, real);
                    const auto                count  = static_cast<double>(lags);
                    double                    exact  = 0.;
                    for (std::size_t k = (lags + 1UZ) / 2UZ + 1UZ; k <= lags; ++k) {
                        const auto hits = static_cast<double>(k);
                        exact += std::exp(std::lgamma(count + 1.) - std::lgamma(hits + 1.) - std::lgamma(count - hits + 1.) + hits * std::log(below) + (count - hits) * std::log(1. - below));
                    }
                    std::println("median lower bound: L={:>4} {} tail {}: {:.4f} scatters, exact probability below it {:.3e}", lags, real ? "real" : "complex", tail, height / scatter, exact);
                    expect(exact <= tail) << std::format("L={} {} tail {}: exact {:.3e}", lags, real ? "real" : "complex", tail, exact);
                    const std::vector<double> doubled(lags, 2. * scatter);
                    expect(eq(acfMedianLowerBound(doubled, tail, real), 2. * height)) << "the bound scales with the scatter";
                }
            }
        }
        const std::vector<double> single{scatter};
        expect(eq(acfMedianLowerBound(single, 1e-3, false), 1.)) << "over one lag the median is lag 0 or above it";
        std::vector<double> mostlySilent(16UZ, 0.);
        mostlySilent[0UZ] = scatter;
        expect(eq(acfMedianLowerBound(mostlySilent, 1e-3, false), 0.)) << "a median of silent lags reads zero";
        expect(std::isinf(acfPeakThresholdDb(mostlySilent, 1e-3, false))) << "and no margin over it holds";
    };

    "a real estimate has a heavier tail and needs a higher threshold"_test = [] {
        // The two-sided normal deviate t solves erfc(t/sqrt2) = tail. The checks use its tabulated values.
        expect(std::abs(acfNormalDeviate(0.05) - 1.959964) < 1e-5) << std::format("{:.6f}", acfNormalDeviate(0.05));
        expect(std::abs(acfNormalDeviate(0.01) - 2.575829) < 1e-5) << std::format("{:.6f}", acfNormalDeviate(0.01));
        expect(std::abs(acfNormalDeviate(0.002699796) - 3.) < 1e-5) << std::format("{:.6f}", acfNormalDeviate(0.002699796));

        const double              rayleigh = acfThreshold(3072UZ, 1024UZ, 1e-3);
        const double              normal   = acfRealThreshold(3072UZ, 1024UZ, 1e-3);
        const std::vector<double> equal(1024UZ, acfScatter(3072UZ));
        expect(normal > rayleigh) << std::format("real {:.5f} against complex {:.5f}", normal, rayleigh);
        std::println("thresholds at nPairs=3072, L=1024, p=1e-3: complex {:.5f} ({:.3f} dB over the median), real {:.5f} ({:.3f} dB)", rayleigh, acfPeakThresholdDb(equal, 1e-3, false), normal, acfPeakThresholdDb(equal, 1e-3, true));
    };

    "over lags of equal scatter the record threshold is the closed form"_test = [] {
        const std::vector<double> equal(1024UZ, acfScatter(3072UZ));
        for (const double pFa : {1e-2, 1e-3}) {
            const double complexThreshold = acfRecordThreshold(equal, pFa, false);
            const double realThreshold    = acfRecordThreshold(equal, pFa, true);
            expect(std::abs(complexThreshold / acfThreshold(3072UZ, 1024UZ, pFa) - 1.) < 1e-12) << std::format("p={}: complex {:.9f}", pFa, complexThreshold);
            expect(std::abs(realThreshold / acfRealThreshold(3072UZ, 1024UZ, pFa) - 1.) < 1e-9) << std::format("p={}: real {:.9f}", pFa, realThreshold);
        }
        // a lag of zero scatter publishes zero and adds nothing to the rate
        std::vector<double> withSilent = equal;
        withSilent.push_back(0.);
        expect(std::abs(acfRecordThreshold(withSilent, 1e-3, false) / acfRecordThreshold(equal, 1e-3, false) - 1.) < 1e-12);
    };
};

const suite<"autocorrelation kernel"> _acfKernel = [] {
    "Wiener-Khinchin equals the direct sum"_test = [] {
        constexpr std::array<std::pair<std::size_t, std::size_t>, 4> kGeometries{{{64UZ, 8UZ}, {256UZ, 100UZ}, {2048UZ, 512UZ}, {1024UZ, 1023UZ}}};
        double                                                       worst = 0.;
        for (const auto& [n, lags] : kGeometries) {
            const std::vector<Complex> complexInput = complexNoise(n, 0x9E37ULL + n);
            const std::vector<float>   realInput    = realNoise(n, 0xC0FFEEULL + n);
            for (const AcfKind kind : {AcfKind::Complex, AcfKind::Envelope}) {
                for (const AcfNormalization normalization : {AcfNormalization::Unbiased, AcfNormalization::Biased}) {
                    const AcfConfig config = baseConfig(n, lags, kind, normalization);

                    gr::analysis::Autocorrelation<Complex> complexKernel;
                    complexKernel.prepare(config);
                    expect(eq(complexKernel.transformLength(), acfTransformLength(n, lags))) << "the kernel's transform length is acfTransformLength";
                    const std::vector<Record> complexRecords = drive(complexKernel, std::span<const Complex>(complexInput));
                    expect(eq(complexRecords.size(), 1UZ)) << "one window, one record";
                    const std::vector<double> complexReference = directRatios<Complex>(config, std::span<const Complex>(complexInput));

                    gr::analysis::Autocorrelation<float> realKernel;
                    realKernel.prepare(config);
                    const std::vector<Record> realRecords = drive(realKernel, std::span<const float>(realInput));
                    expect(eq(realRecords.size(), 1UZ));
                    const std::vector<double> realReference = directRatios<float>(config, std::span<const float>(realInput));

                    for (const auto& [records, reference, label] : {std::tuple{&complexRecords, &complexReference, "complex input"}, std::tuple{&realRecords, &realReference, "real input"}}) {
                        const std::vector<float>& magnitude = (*records)[0UZ].magnitude;
                        expect(eq(magnitude.size(), lags + 1UZ));
                        expect(magnitude[0UZ] == 1.f) << std::format("N={} L={} {}: lag zero is exactly one by construction", n, lags, label);
                        for (std::size_t lag = 0UZ; lag <= lags; ++lag) {
                            const double want     = (*reference)[lag];
                            const double relative = std::abs(static_cast<double>(magnitude[lag]) - want) / std::max(want, 1e-6);
                            worst                 = std::max(worst, relative);
                            expect(relative < 1e-5) << std::format("N={} L={} kind={} norm={} {} lag {}: {:.9f} against {:.9f}", n, lags, gr::analysis::acfKindName(kind), gr::analysis::acfNormalizationName(normalization), label, lag, magnitude[lag], want);
                        }
                    }
                }
            }
        }
        std::println("direct sum: worst relative disagreement over 4 geometries x 2 kinds x 2 normalizations x 2 input types: {:.3e}", worst);
    };

    "the scatter, per lag and per normalization"_test = [] {
        constexpr std::size_t                n            = 512UZ;
        constexpr std::size_t                lags         = 384UZ;
        constexpr std::size_t                realizations = 400UZ;
        constexpr std::array<std::size_t, 5> kLags{1UZ, 64UZ, 128UZ, 256UZ, 384UZ};

        for (const AcfNormalization normalization : {AcfNormalization::Biased, AcfNormalization::Unbiased}) {
            const AcfConfig                        config = baseConfig(n, lags, AcfKind::Complex, normalization);
            gr::analysis::Autocorrelation<Complex> kernel;
            kernel.prepare(config);

            std::array<double, kLags.size()> sumSquares{};
            for (std::size_t trial = 0UZ; trial < realizations; ++trial) {
                const std::vector<Complex> noise   = complexNoise(n, 0x51ED0000ULL + trial);
                const std::vector<Record>  records = drive(kernel, std::span<const Complex>(noise));
                expect(eq(records.size(), 1UZ));
                for (std::size_t k = 0UZ; k < kLags.size(); ++k) {
                    const double value = static_cast<double>(records[0UZ].magnitude[kLags[k]]);
                    sumSquares[k] += value * value;
                }
            }
            for (std::size_t k = 0UZ; k < kLags.size(); ++k) {
                const std::size_t lag      = kLags[k];
                const double      measured = std::sqrt(sumSquares[k] / static_cast<double>(realizations));
                const double      expected = normalization == AcfNormalization::Biased ? std::sqrt(static_cast<double>(n - lag)) / static_cast<double>(n) : 1. / std::sqrt(static_cast<double>(n - lag));
                std::println("scatter: {} lag {:>4}: rms {:.5f} against {:.5f} ({:+.1f}%)", gr::analysis::acfNormalizationName(normalization), lag, measured, expected, 100. * (measured / expected - 1.));
                expect(std::abs(kernel.lagScatter()[lag] / expected - 1.) < 1e-9) << std::format("{} lag {}: the modeled scatter {:.7f} is the closed form", gr::analysis::acfNormalizationName(normalization), lag, kernel.lagScatter()[lag]);
                expect(std::abs(measured / expected - 1.) < 0.10) << std::format("{} lag {}: {:.5f} against {:.5f}", gr::analysis::acfNormalizationName(normalization), lag, measured, expected);
            }
        }
    };

    "the record threshold holds its rate, and a 1/sqrt(N) threshold fires more often"_test = [] {
        constexpr std::size_t n       = 384UZ;
        constexpr std::size_t lags    = 128UZ;
        constexpr std::size_t windows = 2000UZ;
        constexpr double      pFa     = 1e-3;

        const AcfConfig                        config = baseConfig(n, lags, AcfKind::Complex, AcfNormalization::Unbiased);
        gr::analysis::Autocorrelation<Complex> kernel;
        kernel.prepare(config);

        const std::vector<double> closed  = rectangularScatter(n, lags, n, 1UZ, AcfNormalization::Unbiased);
        const double              derived = acfRecordThreshold(std::span<const double>(closed).subspan(1UZ), pFa, false);
        const double              naive   = acfThreshold(n, lags, pFa); // every lag at the scatter 1/sqrt(N)
        expect(derived < acfThreshold(n - lags, lags, pFa)) << "the threshold lies below that of every lag at the scatter of lag L";
        std::size_t firedDerived = 0UZ;
        std::size_t firedNaive   = 0UZ;
        for (std::size_t trial = 0UZ; trial < windows; ++trial) {
            const std::vector<Complex> noise   = complexNoise(n, 0xFA00000ULL + trial);
            const std::vector<Record>  records = drive(kernel, std::span<const Complex>(noise));
            const auto [at, best]              = peakOver(records[0UZ].magnitude, 1UZ, lags);
            std::ignore                        = at;
            firedDerived += best > derived ? 1UZ : 0UZ;
            firedNaive += best > naive ? 1UZ : 0UZ;
            expect(std::abs(records[0UZ].threshold / derived - 1.) < 1e-9) << "the record publishes the threshold of the closed-form scatter";
        }
        const double rateDerived = static_cast<double>(firedDerived) / static_cast<double>(windows);
        const double rateNaive   = static_cast<double>(firedNaive) / static_cast<double>(windows);
        const double spread      = 3. * std::sqrt(pFa * (1. - pFa) / static_cast<double>(windows));
        std::println("false alarms: derived threshold {:.5f} fires on {}/{} = {:.4f}; the 1/sqrt(N) threshold {:.5f} fires on {}/{} = {:.4f} ({:.1f} times)", derived, firedDerived, windows, rateDerived, naive, firedNaive, windows, rateNaive, rateNaive / std::max(rateDerived, 1. / static_cast<double>(windows)));
        expect(rateDerived <= pFa + spread) << std::format("derived rate {:.4f} against design {:.4f} + spread {:.4f}", rateDerived, pFa, spread);
        expect(rateNaive > rateDerived) << "the 1/sqrt(N) threshold fires more often than the derived one";

        // The positive arm is one longer window with a sinusoidal envelope modulation at a known period. The
        // envelope kind finds it.
        constexpr std::size_t positiveN    = 4096UZ;
        constexpr std::size_t positiveLags = 1024UZ;
        constexpr std::size_t period       = 64UZ;
        constexpr double      depth        = 0.9;
        std::vector<Complex>  modulated    = complexNoise(positiveN, 0x9005ULL);
        for (std::size_t k = 0UZ; k < positiveN; ++k) {
            const auto scale = static_cast<float>(std::sqrt(1. + depth * std::cos(kTwoPi * static_cast<double>(k) / static_cast<double>(period))));
            modulated[k] *= scale;
        }
        AcfConfig                              envelopeConfig = baseConfig(positiveN, positiveLags, AcfKind::Envelope, AcfNormalization::Unbiased);
        gr::analysis::Autocorrelation<Complex> envelopeKernel;
        envelopeKernel.prepare(envelopeConfig);
        const std::vector<Record> found = drive(envelopeKernel, std::span<const Complex>(modulated));
        // The autocorrelation of a cosine is a cosine, and the record is its magnitude. The anti-nodes at the odd
        // half-periods are therefore exactly as high as the peak at the period. The search band is centered on the
        // expected lag of one period. Inside it the magnitude has a single maximum. That maximum is quadratically
        // flat. One lag to either side of it, the cosine falls by 5e-3 of its height, far under the scatter. The
        // located lag is therefore asserted to a fraction of a period.
        const std::size_t tolerance = period / 8UZ;
        const auto [at, best]       = peakOver(found[0UZ].magnitude, period - period / 4UZ, period + period / 4UZ);
        std::println("positive arm: envelope modulation at {} samples found at lag {} reading {:.4f} against the record's threshold {:.4f}", period, at, best, found[0UZ].threshold);
        expect(at >= period - tolerance && at <= period + tolerance) << std::format("the injected period is where the maximum sits: lag {} against {} +/- {}", at, period, tolerance);
        expect(best > found[0UZ].threshold) << "and it clears the record's own threshold";
        expect(found[0UZ].realValued) << "an envelope estimate is real and takes the folded-normal threshold";
    };

    "the two normalizations differ by exactly the lag window"_test = [] {
        constexpr std::size_t n    = 1024UZ;
        constexpr std::size_t lags = 256UZ;
        for (const auto windowType : {gr::algorithm::window::Type::Rectangular, gr::algorithm::window::Type::Hann}) {
            const std::vector<Complex> input = complexNoise(n, 0xBEEFULL);

            AcfConfig unbiasedConfig   = baseConfig(n, lags, AcfKind::Complex, AcfNormalization::Unbiased);
            unbiasedConfig.window      = windowType;
            AcfConfig biasedConfig     = unbiasedConfig;
            biasedConfig.normalization = AcfNormalization::Biased;

            gr::analysis::Autocorrelation<Complex> unbiasedKernel;
            unbiasedKernel.prepare(unbiasedConfig);
            const std::vector<Record> unbiased = drive(unbiasedKernel, std::span<const Complex>(input));

            gr::analysis::Autocorrelation<Complex> biasedKernel;
            biasedKernel.prepare(biasedConfig);
            const std::vector<Record> biased = drive(biasedKernel, std::span<const Complex>(input));

            const std::span<const double> taperAcf = unbiasedKernel.taperAutocorrelation();
            for (std::size_t lag = 0UZ; lag <= lags; ++lag) {
                const double window = taperAcf[lag] / taperAcf[0UZ];
                const double want   = static_cast<double>(unbiased[0UZ].magnitude[lag]) * window;
                const double got    = static_cast<double>(biased[0UZ].magnitude[lag]);
                expect(std::abs(got - want) <= 1e-5 * std::max(want, 1e-6)) << std::format("{} lag {}: biased {:.9f} against unbiased x r_w(tau)/r_w(0) {:.9f}", magic_enum::enum_name(windowType), lag, got, want);
            }

            // the unbiased estimate divides the taper out and agrees with the direct sum
            const std::vector<double> reference = directRatios<Complex>(unbiasedConfig, std::span<const Complex>(input));
            for (std::size_t lag = 0UZ; lag <= lags; ++lag) {
                const double relative = std::abs(static_cast<double>(unbiased[0UZ].magnitude[lag]) - reference[lag]) / std::max(reference[lag], 1e-6);
                expect(relative < 1e-5) << std::format("{} unbiased lag {}: {:.9f} against {:.9f}", magic_enum::enum_name(windowType), lag, unbiased[0UZ].magnitude[lag], reference[lag]);
            }
        }
    };

    "averaging lowers the scatter, and half-overlapped windows over the same samples match it"_test = [] {
        constexpr std::size_t                n            = 240UZ;
        constexpr std::size_t                lags         = 16UZ;
        constexpr std::size_t                realizations = 200UZ;
        constexpr std::array<std::size_t, 3> kAverages{1UZ, 4UZ, 16UZ};

        for (const std::size_t averages : kAverages) {
            AcfConfig disjoint   = baseConfig(n, lags, AcfKind::Complex, AcfNormalization::Unbiased);
            disjoint.nAverages   = averages;
            AcfConfig overlapped = disjoint;
            overlapped.overlap   = 0.5;
            overlapped.nAverages = 2UZ * averages - 1UZ; // the same distinct samples, N + (K'-1)H == K N at H = N/2

            gr::analysis::Autocorrelation<Complex> disjointKernel;
            disjointKernel.prepare(disjoint);
            gr::analysis::Autocorrelation<Complex> overlappedKernel;
            overlappedKernel.prepare(overlapped);

            double        disjointSum   = 0.;
            double        overlappedSum = 0.;
            std::uint64_t disjointDistinct{};
            std::uint64_t overlappedDistinct{};
            for (std::size_t trial = 0UZ; trial < realizations; ++trial) {
                const std::vector<Complex> noise = complexNoise(n * averages, 0xA5A50000ULL + trial);
                const std::vector<Record>  a     = drive(disjointKernel, std::span<const Complex>(noise), false);
                const std::vector<Record>  b     = drive(overlappedKernel, std::span<const Complex>(noise), false);
                expect(ge(a.size(), 1UZ));
                expect(ge(b.size(), 1UZ));
                const double av = static_cast<double>(a[0UZ].magnitude[1UZ]);
                const double bv = static_cast<double>(b[0UZ].magnitude[1UZ]);
                disjointSum += av * av;
                overlappedSum += bv * bv;
                disjointDistinct   = a[0UZ].nDistinct;
                overlappedDistinct = b[0UZ].nDistinct;
                disjointKernel.reset();
                overlappedKernel.reset();
            }
            const double measuredDisjoint   = std::sqrt(disjointSum / static_cast<double>(realizations));
            const double measuredOverlapped = std::sqrt(overlappedSum / static_cast<double>(realizations));
            const double expected           = 1. / std::sqrt(static_cast<double>(averages * (n - 1UZ)));

            std::println("averaging: K={:>2} disjoint rms {:.5f} against 1/sqrt(K(N-1)) {:.5f}; overlapped K'={} rms {:.5f}; distinct samples {} and {}", averages, measuredDisjoint, expected, overlapped.nAverages, measuredOverlapped, disjointDistinct, overlappedDistinct);
            expect(std::abs(measuredDisjoint / expected - 1.) < 0.10) << std::format("K={}: disjoint rms {:.5f} against {:.5f}", averages, measuredDisjoint, expected);
            expect(std::abs(measuredOverlapped / measuredDisjoint - 1.) < 0.10) << std::format("K={}: the overlapped windows scatter as the disjoint ones, {:.5f} against {:.5f}", averages, measuredOverlapped, measuredDisjoint);
            expect(std::abs(measuredOverlapped / overlappedKernel.lagScatter()[1UZ] - 1.) < 0.10) << std::format("K={}: the overlapped rms {:.5f} against the modeled {:.5f}", averages, measuredOverlapped, overlappedKernel.lagScatter()[1UZ]);
            expect(eq(disjointDistinct, averages * n)) << "disjoint windows cover K N distinct samples";
            expect(eq(overlappedDistinct, averages * n)) << "half-overlapped windows over the same input: the same distinct samples";
        }
    };

    "mean removal, and a constant-modulus envelope with nothing to correlate"_test = [] {
        constexpr std::size_t n    = 4096UZ;
        constexpr std::size_t lags = 512UZ;

        std::vector<Complex> offset = complexNoise(n, 0xD1CEULL);
        for (Complex& sample : offset) {
            sample += Complex{1.f, 0.f}; // a DC offset equal to the rms
        }

        AcfConfig kept  = baseConfig(n, lags, AcfKind::Complex, AcfNormalization::Unbiased);
        kept.removeMean = false;
        gr::analysis::Autocorrelation<Complex> keptKernel;
        keptKernel.prepare(kept);
        const std::vector<Record> withPedestal = drive(keptKernel, std::span<const Complex>(offset));
        double                    lowest       = 1.;
        for (std::size_t lag = 1UZ; lag <= lags; ++lag) {
            lowest = std::min(lowest, static_cast<double>(withPedestal[0UZ].magnitude[lag]));
        }
        std::println("mean kept: an offset puts a pedestal of at least {:.4f} under every lag", lowest);
        expect(lowest >= 0.4) << std::format("every lag reads at least 0.4, lowest {:.4f}", lowest);

        AcfConfig removed  = kept;
        removed.removeMean = true;
        gr::analysis::Autocorrelation<Complex> removedKernel;
        removedKernel.prepare(removed);
        const std::vector<Record> withoutPedestal = drive(removedKernel, std::span<const Complex>(offset));
        const auto [at, best]                     = peakOver(withoutPedestal[0UZ].magnitude, 1UZ, lags);
        std::ignore                               = at;
        std::println("mean removed: the largest lag reads {:.4f} against a scatter of {:.4f}", best, withoutPedestal[0UZ].scatter);
        expect(best < 6. * withoutPedestal[0UZ].scatter) << "with the mean removed every lag stays under six scatter units";

        // A constant-modulus input has no envelope fluctuation. The estimator counts the window degenerate and
        // publishes no record.
        std::vector<Complex>  constant(n);
        gr::rng::Xoshiro256pp rng(0x1234ULL);
        for (std::size_t k = 0UZ; k < n; ++k) {
            const double phase = kTwoPi * rng.uniform01<double>();
            constant[k]        = Complex{static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))};
        }
        AcfConfig                              envelope = baseConfig(n, lags, AcfKind::Envelope, AcfNormalization::Unbiased);
        gr::analysis::Autocorrelation<Complex> envelopeKernel;
        envelopeKernel.prepare(envelope);
        const std::vector<Record> none = drive(envelopeKernel, std::span<const Complex>(constant));
        expect(none.empty()) << "a constant-modulus envelope publishes no record";
        expect(eq(envelopeKernel.nDegenerate, std::uint64_t{1ULL})) << "and the window is counted degenerate";
        expect(eq(envelopeKernel.nWindows, std::uint64_t{1ULL}));
        expect(eq(envelopeKernel.nRecords, std::uint64_t{0ULL}));
    };

    "the window counters add up, and the remainder stays with the caller"_test = [] {
        constexpr std::size_t n      = 256UZ;
        constexpr std::size_t lags   = 64UZ;
        AcfConfig             config = baseConfig(n, lags, AcfKind::Complex, AcfNormalization::Unbiased);
        config.overlap               = 0.5;
        config.nAverages             = 4UZ;

        gr::analysis::Autocorrelation<Complex> kernel;
        kernel.prepare(config);
        expect(eq(kernel.hop(), 128UZ));

        const std::vector<Complex> input = complexNoise(2000UZ, 0x777ULL);
        std::vector<Record>        records;
        const auto                 sink = [&records](const AcfResult& result) { records.push_back(copyOf(result)); };
        std::size_t                at   = 0UZ;
        while (at < input.size()) {
            const std::size_t used = kernel.process(std::span<const Complex>(input).subspan(at), sink);
            if (used == 0UZ) {
                break;
            }
            at += used;
        }
        const std::size_t tail = input.size() - at;
        expect(lt(tail, n)) << "the remainder is shorter than a window";
        expect(eq(kernel.nSamples, at));
        expect(eq(kernel.nWindows, config.nAverages * kernel.nRecords + kernel.windowsPending() + kernel.nDegenerate + kernel.nNonFinite));
        expect(eq(records.size(), kernel.nRecords));
        for (std::size_t k = 0UZ; k < records.size(); ++k) {
            expect(eq(records[k].nAveraged, config.nAverages));
            expect(eq(records[k].sampleStart, k * config.nAverages * kernel.hop()));
            expect(eq(records[k].nDistinct, n + (config.nAverages - 1UZ) * kernel.hop()));
        }
        std::ignore = kernel.flush(sink);
        expect(eq(kernel.nPartialFlushes, records.size() > 0UZ ? 1UZ : 0UZ));
    };

    "the scatter counts the products overlapping windows share"_test = [] {
        constexpr std::size_t      n        = 256UZ;
        constexpr std::size_t      lags     = 64UZ;
        constexpr std::size_t      averages = 4UZ;
        const std::vector<Complex> input    = complexNoise(4096UZ, 0x5CA7ULL);
        for (const double overlap : {0., 0.5, 0.75}) {
            for (const AcfNormalization normalization : {AcfNormalization::Unbiased, AcfNormalization::Biased}) {
                AcfConfig config = baseConfig(n, lags, AcfKind::Complex, normalization);
                config.overlap   = overlap;
                config.nAverages = averages;
                gr::analysis::Autocorrelation<Complex> kernel;
                kernel.prepare(config);
                const std::vector<Record> records = drive(kernel, std::span<const Complex>(input), false);
                expect(fatal(ge(records.size(), 1UZ)));
                const std::vector<double> closed = rectangularScatter(n, lags, kernel.hop(), averages, normalization);
                for (std::size_t lag = 1UZ; lag <= lags; ++lag) {
                    expect(std::abs(kernel.lagScatter()[lag] / closed[lag] - 1.) < 1e-9) << std::format("overlap {} {} lag {}: {:.8f} against {:.8f}", overlap, gr::analysis::acfNormalizationName(normalization), lag, kernel.lagScatter()[lag], closed[lag]);
                }
                const auto   noisiest = static_cast<std::size_t>(std::distance(closed.begin(), std::ranges::max_element(closed)));
                const double divisor  = static_cast<double>(normalization == AcfNormalization::Biased ? n : n - noisiest);
                const double pairs    = std::pow(static_cast<double>(n - noisiest) / (divisor * closed[noisiest]), 2.);
                expect(eq(records[0UZ].scatterLag, noisiest)) << std::format("overlap {} {}: the scatter holds for the noisiest lag", overlap, gr::analysis::acfNormalizationName(normalization));
                expect(std::abs(records[0UZ].scatter / closed[noisiest] - 1.) < 1e-9);
                expect(std::abs(records[0UZ].nPairs / pairs - 1.) < 1e-9) << std::format("{:.3f} effective products against {:.3f}", records[0UZ].nPairs, pairs);
                expect(eq(noisiest, normalization == AcfNormalization::Biased ? 1UZ : lags)) << "the biased scatter is largest at lag 1, the unbiased at lag L";
            }
        }
    };

    "the threshold holds the stated false-alarm rate under each taper, overlap and normalization"_test = [] {
        constexpr std::size_t n        = 192UZ;
        constexpr std::size_t lags     = 64UZ;
        constexpr std::size_t averages = 4UZ;
        constexpr std::size_t records  = 800UZ;
        constexpr double      pFa      = 0.25;
        std::uint64_t         seed     = 0xFA1A0000ULL;
        for (const auto taper : {gr::algorithm::window::Type::Rectangular, gr::algorithm::window::Type::Hann}) {
            for (const double overlap : {0., 0.5}) {
                for (const AcfNormalization normalization : {AcfNormalization::Unbiased, AcfNormalization::Biased}) {
                    AcfConfig config      = baseConfig(n, lags, AcfKind::Complex, normalization);
                    config.window         = taper;
                    config.overlap        = overlap;
                    config.nAverages      = averages;
                    config.falseAlarmRate = pFa;
                    gr::analysis::Autocorrelation<Complex> kernel;
                    kernel.prepare(config);

                    const std::vector<Complex> noise     = complexNoise(records * averages * kernel.hop() + n, ++seed);
                    std::size_t                fired     = 0UZ;
                    std::size_t                seen      = 0UZ;
                    double                     threshold = 0.;
                    const auto                 sink      = [&](const AcfResult& result) {
                        if (seen++ < records) {
                            threshold = result.threshold;
                            fired += *std::ranges::max_element(result.magnitude.subspan(1UZ)) > static_cast<float>(result.threshold) ? 1UZ : 0UZ;
                        }
                    };
                    std::size_t at = 0UZ;
                    while (at < noise.size()) {
                        const std::size_t used = kernel.process(std::span<const Complex>(noise).subspan(at), sink);
                        if (used == 0UZ) {
                            break;
                        }
                        at += used;
                    }
                    // Noise lags are close to independent. A record clears the threshold with a probability near
                    // 1 - prod(1 - tail) over its lags, a little below the stated rate the union bound sets. The lags
                    // share the zero lag and the window means, and the measured rate lies up to a few percent below
                    // that product. A correct threshold falls outside four binomial standard deviations about it in
                    // about 1 of 1000 runs of the eight arms.
                    double quiet = 1.;
                    for (const double scatter : kernel.lagScatter().subspan(1UZ)) {
                        quiet *= 1. - acfLagTail(threshold, scatter, false);
                    }
                    const double expected = 1. - quiet;
                    const double bound    = 4. * std::sqrt(expected * (1. - expected) / static_cast<double>(records));
                    const double measured = static_cast<double>(fired) / static_cast<double>(records);
                    std::println("false alarms: {} overlap {} {}: {} of {} records, rate {:.4f} against {:.4f} +/- {:.4f} (stated {:.4f})", magic_enum::enum_name(taper), overlap, gr::analysis::acfNormalizationName(normalization), fired, records, measured, expected, bound, pFa);
                    expect(fatal(ge(seen, records)));
                    expect(expected <= pFa) << "the union bound lies at or above the rate of independent lags";
                    expect(std::abs(measured - expected) <= bound) << std::format("{} overlap {} {}: rate {:.4f} against {:.4f} within {:.4f}", magic_enum::enum_name(taper), overlap, gr::analysis::acfNormalizationName(normalization), measured, expected, bound);
                }
            }
        }
    };

    "under long averaging a noise lag has zero mean, and the threshold holds its rate"_test = [] {
        // Mean removal alone gives each lag of white noise the mean -1/(N-1) of the zero lag under the unbiased
        // normalization and -r_w(tau)/(r_w(0) (N-1)) under the biased one. The scatter falls as 1/sqrt(K), and the
        // offset does not. At N = 64 and K = 64 the offset is about one scatter unit. The estimator adds the offset
        // back.
        constexpr std::size_t n        = 64UZ;
        constexpr std::size_t lags     = 16UZ;
        constexpr std::size_t averages = 64UZ;
        constexpr std::size_t records  = 100UZ;
        constexpr double      pFa      = 0.1;
        std::uint64_t         seed     = 0x0FF5E700ULL;
        for (const auto& [taper, overlap, normalization] : {std::tuple{gr::algorithm::window::Type::Rectangular, 0., AcfNormalization::Unbiased}, std::tuple{gr::algorithm::window::Type::Hann, 0.5, AcfNormalization::Biased}}) {
            AcfConfig config      = baseConfig(n, lags, AcfKind::Complex, normalization);
            config.window         = taper;
            config.overlap        = overlap;
            config.nAverages      = averages;
            config.falseAlarmRate = pFa;
            gr::analysis::Autocorrelation<Complex> kernel;
            kernel.prepare(config);
            const std::span<const double> taperAcf = kernel.taperAutocorrelation();
            const bool                    biased   = normalization == AcfNormalization::Biased;
            // the size of the offset mean removal alone gives lag `tau`
            const auto offset = [&taperAcf, biased](std::size_t lag) { return (biased ? taperAcf[lag] / taperAcf[0UZ] : 1.) / static_cast<double>(n - 1UZ); };

            const std::vector<Complex> noise     = complexNoise(records * averages * kernel.hop() + n, ++seed);
            std::size_t                fired     = 0UZ;
            std::size_t                seen      = 0UZ;
            double                     threshold = 0.;
            double                     relative  = 0.; // the sum of each lag's real part over its offset
            const auto                 sink      = [&](const AcfResult& result) {
                if (seen++ < records) {
                    threshold = result.threshold;
                    fired += *std::ranges::max_element(result.magnitude.subspan(1UZ)) > static_cast<float>(result.threshold) ? 1UZ : 0UZ;
                    for (std::size_t lag = 1UZ; lag <= lags; ++lag) {
                        relative += static_cast<double>(result.magnitude[lag]) * std::cos(static_cast<double>(result.phase[lag])) / offset(lag);
                    }
                }
            };
            std::size_t at = 0UZ;
            while (at < noise.size()) {
                const std::size_t used = kernel.process(std::span<const Complex>(noise).subspan(at), sink);
                if (used == 0UZ) {
                    break;
                }
                at += used;
            }
            expect(fatal(ge(seen, records)));

            // The real part of a lag scatters by s/sqrt(2) about its mean. The mean below averages `records * lags`
            // such parts, each over its lag's offset, and has the standard error below. Without the share the
            // estimator adds back, it would lie near -1.
            double quiet  = 1.;
            double spread = 0.;
            for (std::size_t lag = 1UZ; lag <= lags; ++lag) {
                const double scatter = kernel.lagScatter()[lag];
                quiet *= 1. - acfLagTail(threshold, scatter, false);
                spread += 0.5 * (scatter / offset(lag)) * (scatter / offset(lag));
            }
            const double mean          = relative / static_cast<double>(records * lags);
            const double standardError = std::sqrt(spread / static_cast<double>(records)) / static_cast<double>(lags);
            const double expected      = 1. - quiet;
            const double bound         = 4. * std::sqrt(expected * (1. - expected) / static_cast<double>(records));
            const double measured      = static_cast<double>(fired) / static_cast<double>(records);
            std::println("long averaging: {} overlap {} {} K={}: lag mean {:+.3f} offsets (standard error {:.3f}); {} of {} records, rate {:.4f} against {:.4f} +/- {:.4f} (stated {:.4f})", magic_enum::enum_name(taper), overlap, gr::analysis::acfNormalizationName(normalization), averages, mean, standardError, fired, records, measured, expected, bound, pFa);
            expect(std::abs(mean) <= 5. * standardError) << std::format("{} {}: the lags' mean is {:+.3f} offsets, within five standard errors of zero", magic_enum::enum_name(taper), gr::analysis::acfNormalizationName(normalization), mean);
            expect(std::abs(measured - expected) <= bound) << std::format("{} {}: rate {:.4f} against {:.4f} within {:.4f}", magic_enum::enum_name(taper), gr::analysis::acfNormalizationName(normalization), measured, expected, bound);
        }
    };

    "the envelope kind without mean removal publishes an infinite threshold and margin"_test = [] {
        // Without mean removal the envelope of complex Gaussian noise keeps its mean P, and its variance is P^2. Every
        // lag then reads near one half. The complex kind of zero-mean noise keeps lags of zero mean and a finite
        // threshold.
        constexpr std::size_t      n     = 1024UZ;
        constexpr std::size_t      lags  = 16UZ;
        const std::vector<Complex> noise = complexNoise(4UZ * n, 0xE5E5ULL);
        for (const AcfKind kind : {AcfKind::Envelope, AcfKind::Complex}) {
            AcfConfig config  = baseConfig(n, lags, kind, AcfNormalization::Unbiased);
            config.removeMean = false;
            config.nAverages  = 4UZ;
            gr::analysis::Autocorrelation<Complex> kernel;
            kernel.prepare(config);
            const std::vector<Record> records = drive(kernel, std::span<const Complex>(noise), false);
            expect(fatal(eq(records.size(), 1UZ)));
            const Record& record = records[0UZ];
            double        sum    = 0.;
            for (std::size_t lag = 1UZ; lag <= lags; ++lag) {
                sum += static_cast<double>(record.magnitude[lag]);
            }
            const double mean = sum / static_cast<double>(lags);
            std::println("mean kept: {} kind: lags 1..{} average {:.4f}; threshold {}, margin {} dB", gr::analysis::acfKindName(kind), lags, mean, record.threshold, record.peakThresholdDb);
            if (kind == AcfKind::Envelope) {
                expect(std::abs(mean - 0.5) < 0.1) << std::format("the envelope lags average {:.4f}, near one half", mean);
                expect(std::isinf(record.threshold) && record.threshold > 0.) << "the envelope kind publishes no finite threshold";
                expect(std::isinf(record.peakThresholdDb) && record.peakThresholdDb > 0.) << "and no finite margin";
            } else {
                expect(std::isfinite(record.threshold)) << "the complex kind keeps a finite threshold";
                expect(std::isfinite(record.peakThresholdDb)) << "and a finite margin";
            }
        }
    };

    "the decibel margin over a record's own median holds the stated rate at every max_lag"_test = [] {
        // A detector refers the margin to the median of the record's L + 1 values, lag 0 included, and fires at a
        // local maximum of lags 1..L-1 above it.
        const auto fires = [](std::span<const float> magnitude, double marginDb) {
            if (magnitude.empty()) {
                return false;
            }
            std::vector<float> sorted(magnitude.begin(), magnitude.end());
            const auto         middle = static_cast<std::ptrdiff_t>(sorted.size() / 2UZ);
            std::nth_element(sorted.begin(), sorted.begin() + middle, sorted.end());
            const double cutoff = static_cast<double>(sorted[static_cast<std::size_t>(middle)]) * std::pow(10., marginDb / 10.);
            for (std::size_t lag = 1UZ; lag + 1UZ < magnitude.size(); ++lag) {
                if (static_cast<double>(magnitude[lag]) > cutoff && magnitude[lag] >= magnitude[lag - 1UZ] && magnitude[lag] > magnitude[lag + 1UZ]) {
                    return true;
                }
            }
            return false;
        };
        constexpr std::size_t n       = 64UZ;
        constexpr std::size_t records = 3000UZ;
        constexpr double      pFa     = 0.02;
        // Four binomial standard deviations over the stated rate: a detector at exactly that rate exceeds the bound in
        // about 1 of 10000 seeds per arm.
        const double  bound = pFa + 4. * std::sqrt(pFa * (1. - pFa) / static_cast<double>(records));
        std::uint64_t seed  = 0xDB0000ULL;
        for (const auto taper : {gr::algorithm::window::Type::Rectangular, gr::algorithm::window::Type::Hann}) {
            for (const AcfNormalization normalization : {AcfNormalization::Unbiased, AcfNormalization::Biased}) {
                for (const std::size_t lags : {n / 16UZ, n / 4UZ, n / 2UZ, n - 1UZ}) {
                    AcfConfig config      = baseConfig(n, lags, AcfKind::Complex, normalization);
                    config.window         = taper;
                    config.falseAlarmRate = pFa;
                    gr::analysis::Autocorrelation<Complex> kernel;
                    kernel.prepare(config);

                    const std::vector<Complex> noise  = complexNoise(records * n, ++seed);
                    std::size_t                fired  = 0UZ;
                    double                     margin = 0.;
                    const auto                 sink   = [&](const AcfResult& result) {
                        margin = result.peakThresholdDb;
                        fired += fires(result.magnitude, result.peakThresholdDb) ? 1UZ : 0UZ;
                    };
                    expect(fatal(eq(kernel.process(std::span<const Complex>(noise), sink), noise.size())));
                    expect(fatal(eq(kernel.nRecords, std::uint64_t{records})));
                    const double measured = static_cast<double>(fired) / static_cast<double>(records);
                    std::println("median margin: {} {} L={:>3} of N={}: {:.3f} dB, {} of {} records, rate {:.4f} against at most {:.4f} (bound {:.4f})", magic_enum::enum_name(taper), gr::analysis::acfNormalizationName(normalization), lags, n, margin, fired, records, measured, pFa, bound);
                    expect(measured <= bound) << std::format("{} {} L={}: rate {:.4f} against {:.4f}", magic_enum::enum_name(taper), gr::analysis::acfNormalizationName(normalization), lags, measured, bound);
                }
            }
        }
    };

    "a lag whose noise model lies below the transform's resolution publishes zero and is left out of the threshold"_test = [] {
        // Under the unbiased normalization a Hann taper's correlation at the lags next to N is a product of its end
        // values, and its `sum c^2` lies far below the transform's rounding. The reference forms `sum c^2` and `r_w` by
        // direct sums over the record's windows.
        constexpr std::size_t    n     = 1024UZ;
        constexpr std::size_t    lags  = n - 1UZ;
        const std::vector<float> taper = gr::algorithm::window::create<float>(gr::algorithm::window::Type::Hann, n);
        for (const auto& [averages, overlap] : {std::pair{1UZ, 0.}, std::pair{4UZ, 0.5}}) {
            AcfConfig config = baseConfig(n, lags, AcfKind::Complex, AcfNormalization::Unbiased);
            config.window    = gr::algorithm::window::Type::Hann;
            config.overlap   = overlap;
            config.nAverages = averages;
            gr::analysis::Autocorrelation<Complex> kernel;
            kernel.prepare(config);
            const std::size_t          hop     = kernel.hop();
            const std::size_t          span    = n + (averages - 1UZ) * hop;
            const std::vector<Complex> noise   = complexNoise(2UZ * averages * hop + n, 0x2E50ULL + averages);
            const std::vector<Record>  records = drive(kernel, std::span<const Complex>(noise), false);
            expect(fatal(ge(records.size(), 2UZ)));

            std::vector<double> squares(lags + 1UZ, 0.);
            std::vector<double> direct(lags + 1UZ, 0.);
            std::vector<double> weights(span);
            for (std::size_t lag = 0UZ; lag <= lags; ++lag) {
                std::ranges::fill(weights, 0.);
                double overlapSum = 0.;
                for (std::size_t k = 0UZ; k + lag < n; ++k) {
                    const double product = static_cast<double>(taper[k]) * static_cast<double>(taper[k + lag]);
                    overlapSum += product;
                    for (std::size_t window = 0UZ; window < averages; ++window) {
                        weights[window * hop + k] += product;
                    }
                }
                for (const double weight : weights) {
                    squares[lag] += weight * weight;
                }
                direct[lag] = overlapSum > 0. ? std::sqrt(squares[lag]) / (static_cast<double>(averages) * overlapSum) : 0.;
            }

            std::vector<double> kept(lags, 0.);
            std::size_t         silent = 0UZ;
            for (std::size_t lag = 1UZ; lag <= lags; ++lag) {
                const double modeled = kernel.lagScatter()[lag];
                if (modeled > 0.) {
                    kept[lag - 1UZ] = direct[lag];
                    expect(std::abs(modeled / direct[lag] - 1.) < 1e-4) << std::format("K={} lag {}: modeled scatter {:.6g} against the direct sum {:.6g}", averages, lag, modeled, direct[lag]);
                } else {
                    ++silent;
                    for (const Record& record : records) {
                        expect(eq(record.magnitude[lag], 0.f)) << std::format("K={} lag {}: a lag without a model publishes zero", averages, lag);
                    }
                }
                if (squares[lag] > 1e-9 * squares[0UZ]) {
                    expect(modeled > 0.) << std::format("K={} lag {}: a lag well above the resolution keeps its model", averages, lag);
                }
            }
            const double threshold = acfRecordThreshold(kept, config.falseAlarmRate, false);
            std::println("resolution: Hann K={} L={} of N={}: {} lags without a model, noisiest lag {} on {:.2f} products, threshold {:.5f} against {:.5f} over the direct sums", averages, lags, n, silent, records[0UZ].scatterLag, records[0UZ].nPairs, records[0UZ].threshold, threshold);
            expect(ge(records[0UZ].nPairs, 1.)) << "a lag stands on at least one effective product";
            expect(std::abs(records[0UZ].threshold / threshold - 1.) < 1e-4) << "the threshold covers the lags that keep a model";
        }
    };

    "a record published after nAverages falls below the windows pending counts apart from full records"_test = [] {
        constexpr std::size_t n      = 64UZ;
        AcfConfig             config = baseConfig(n, 8UZ, AcfKind::Complex, AcfNormalization::Unbiased);
        config.nAverages             = 16UZ;
        gr::analysis::Autocorrelation<Complex> kernel;
        kernel.prepare(config);

        const std::vector<Complex>     input   = complexNoise(15UZ * n, 0x4A4AULL);
        const std::span<const Complex> samples = std::span<const Complex>(input);
        std::vector<Record>            records;
        const auto                     sink = [&records](const AcfResult& result) { records.push_back(copyOf(result)); };
        expect(eq(kernel.process(samples.first(10UZ * n), sink), 10UZ * n));
        expect(records.empty());
        kernel.setAveraging(4UZ);
        expect(eq(kernel.process(samples.subspan(10UZ * n, n), sink), n));
        expect(fatal(eq(records.size(), 1UZ)));
        expect(eq(records[0UZ].nAveraged, 11UZ)) << "the next window publishes every window pending";
        expect(eq(kernel.nLongRecords, std::uint64_t{1ULL})) << "and the record counts as a long one";
        expect(eq(kernel.nRecords, std::uint64_t{0ULL})) << "not as a record of nAverages windows";
        expect(eq(kernel.process(samples.subspan(11UZ * n), sink), 4UZ * n));
        expect(fatal(eq(records.size(), 2UZ)));
        expect(eq(records[1UZ].nAveraged, 4UZ));
        expect(eq(kernel.nRecords, std::uint64_t{1ULL}));
        expect(eq(kernel.nLongRecords, std::uint64_t{1ULL}));
    };

    "a flush counts its record by the windows it holds, after nAverages falls to or below them"_test = [] {
        constexpr std::size_t      n     = 64UZ;
        const std::vector<Complex> input = complexNoise(10UZ * n, 0x4A4BULL);
        struct FlushArm {
            std::size_t   averages;
            std::uint64_t full;
            std::uint64_t longer;
            std::uint64_t partial;
        };
        for (const auto& [averages, full, longer, partial] : {FlushArm{4UZ, 0ULL, 1ULL, 0ULL}, FlushArm{10UZ, 1ULL, 0ULL, 0ULL}, FlushArm{16UZ, 0ULL, 0ULL, 1ULL}}) {
            AcfConfig config = baseConfig(n, 8UZ, AcfKind::Complex, AcfNormalization::Unbiased);
            config.nAverages = 16UZ;
            gr::analysis::Autocorrelation<Complex> kernel;
            kernel.prepare(config);
            std::vector<Record> records;
            const auto          sink = [&records](const AcfResult& result) { records.push_back(copyOf(result)); };
            expect(eq(kernel.process(std::span<const Complex>(input), sink), 10UZ * n));
            expect(fatal(eq(kernel.windowsPending(), 10UZ)));
            kernel.setAveraging(averages);
            expect(kernel.flush(sink));
            expect(fatal(eq(records.size(), 1UZ)));
            expect(eq(records[0UZ].nAveraged, 10UZ)) << std::format("nAverages {}: the flush publishes the ten windows pending", averages);
            expect(eq(kernel.nRecords, full)) << std::format("nAverages {}: a record of exactly nAverages windows", averages);
            expect(eq(kernel.nLongRecords, longer)) << std::format("nAverages {}: a record of more", averages);
            expect(eq(kernel.nPartialFlushes, partial)) << std::format("nAverages {}: a record of fewer", averages);
            expect(eq(kernel.nPartialWindows, 10U * partial)) << std::format("nAverages {}: the windows of a record of fewer", averages);
        }
    };

    "a window holding a NaN or an infinity is counted apart and publishes nothing"_test = [] {
        constexpr std::size_t n    = 64UZ;
        constexpr std::size_t lags = 8UZ;
        for (const float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
            std::vector<Complex> input = complexNoise(3UZ * n, 0x0BADULL);
            input[n + 5UZ]             = Complex{bad, 0.f};
            gr::analysis::Autocorrelation<Complex> kernel;
            kernel.prepare(baseConfig(n, lags, AcfKind::Complex, AcfNormalization::Unbiased));
            const std::vector<Record> records = drive(kernel, std::span<const Complex>(input), false);
            expect(eq(kernel.nWindows, std::uint64_t{3ULL}));
            expect(eq(kernel.nNonFinite, std::uint64_t{1ULL})) << std::format("{}: the window holding it is counted non-finite", bad);
            expect(eq(kernel.nDegenerate, std::uint64_t{0ULL})) << "and not degenerate";
            expect(eq(records.size(), 2UZ));
            for (const Record& record : records) {
                expect(std::ranges::all_of(record.magnitude, [](float value) { return std::isfinite(value); }));
            }
        }
    };

    "prepare and the setters refuse invalid settings"_test = [] {
        gr::analysis::Autocorrelation<Complex> kernel;
        AcfConfig                              config = baseConfig(1024UZ, 256UZ, AcfKind::Complex, AcfNormalization::Unbiased);

        AcfConfig tooShort    = config;
        tooShort.windowLength = 8UZ;
        expect(throws<gr::exception>([&] { kernel.prepare(tooShort); }));
        AcfConfig zeroLag = config;
        zeroLag.maxLag    = 0UZ;
        expect(throws<gr::exception>([&] { kernel.prepare(zeroLag); }));
        AcfConfig lagAtWindow = config;
        lagAtWindow.maxLag    = lagAtWindow.windowLength;
        expect(throws<gr::exception>([&] { kernel.prepare(lagAtWindow); }));
        AcfConfig noAverages = config;
        noAverages.nAverages = 0UZ;
        expect(throws<gr::exception>([&] { kernel.prepare(noAverages); }));
        AcfConfig fullOverlap = config;
        fullOverlap.overlap   = 1.;
        expect(throws<gr::exception>([&] { kernel.prepare(fullOverlap); }));
        AcfConfig zeroRate      = config;
        zeroRate.falseAlarmRate = 0.;
        expect(throws<gr::exception>([&] { kernel.prepare(zeroRate); }));
        expect(nothrow([&] { kernel.prepare(config); }));
        expect(throws<gr::exception>([&] { kernel.setAveraging(0UZ); }));
        expect(throws<gr::exception>([&] { kernel.setFalseAlarmRate(1.); }));
    };

    "the segmenter refuses a taper the window cannot build"_test = [] {
        gr::analysis::WindowedSegmenter<float> segmenter;
        expect(throws<gr::exception>([&] { segmenter.prepare(1UZ, 0UZ, 0., gr::algorithm::window::Type::Kaiser); })) << "a one-sample Kaiser window";
        expect(nothrow([&] { segmenter.prepare(16UZ, 4UZ, 0., gr::algorithm::window::Type::Kaiser); }));
        expect(eq(segmenter.taper.size(), 16UZ));
    };
};

int main() { /* not needed for UT */ }
