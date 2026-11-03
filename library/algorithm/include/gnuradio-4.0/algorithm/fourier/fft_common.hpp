#ifndef GNURADIO_ALGORITHM_FFT_COMMON_HPP
#define GNURADIO_ALGORITHM_FFT_COMMON_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <iterator>
#include <numbers>
#include <utility>
#include <vector>

#include <format>
#include <ranges>

#include <gnuradio-4.0/meta/simd_math.hpp>

#include <gnuradio-4.0/algorithm/math/Decibel.hpp>

namespace gr::algorithm::fft {

namespace detail {
using FloatVector = vir::stdx::native_simd<float>;

/// @brief The real and the imaginary parts of the bins [k, k + FloatVector::size()) as two vectors.
inline std::pair<FloatVector, FloatVector> deinterleave(const std::complex<float>* bins, std::size_t k) noexcept {
    const float* parts = reinterpret_cast<const float*>(bins + k);
    return {FloatVector([parts](auto i) { return parts[2UZ * i]; }), FloatVector([parts](auto i) { return parts[2UZ * i + 1UZ]; })};
}

/// @brief Writes the scaled magnitudes, in dB if `outputInDb`, of the leading whole vectors of bins [0, n) of an
/// N-point transform to `out` and returns their count. The caller computes the remaining bins.
inline std::size_t magnitudeSpectrumVectors(const std::complex<float>* bins, float* out, std::size_t n, std::size_t N, bool halfSpectrum, bool outputInDb) noexcept {
    using V                   = FloatVector;
    const float       unit    = 1.f / static_cast<float>(N);
    const float       folded  = 2.f / static_cast<float>(N);
    const std::size_t nyquist = N / 2UZ;
    const V           floorAmplitude(static_cast<float>(gr::math::kDbFloorAmplitude));
    const V           floorDb(static_cast<float>(gr::math::kDbFloor));
    std::size_t       k = 0UZ;
    for (; k + V::size() <= n; k += V::size()) {
        const auto [re, im] = deinterleave(bins, k);
        V scale(halfSpectrum ? folded : unit);
        if (halfSpectrum && (k == 0UZ || (k <= nyquist && nyquist < k + V::size()))) {
            scale = V([k, nyquist, unit, folded](auto i) {
                const std::size_t bin = k + i;
                return bin == 0UZ || bin == nyquist ? unit : folded;
            });
        }
        V magnitude = gr::meta::hypot(re, im) * scale;
        if (outputInDb) {
            V level                                  = V(20.f) * gr::meta::log10(magnitude);
            where(magnitude < floorAmplitude, level) = floorDb;
            magnitude                                = level;
        }
        magnitude.copy_to(out + k, vir::stdx::element_aligned);
    }
    return k;
}

/// @brief Writes the phases of the leading whole vectors of bins [0, n) to `out` and returns their count. The caller
/// computes the remaining bins.
inline std::size_t phaseSpectrumVectors(const std::complex<float>* bins, float* out, std::size_t n) noexcept {
    using V       = FloatVector;
    std::size_t k = 0UZ;
    for (; k + V::size() <= n; k += V::size()) {
        const auto [re, im] = deinterleave(bins, k);
        gr::meta::atan2(im, re).copy_to(out + k, vir::stdx::element_aligned);
    }
    return k;
}

/// @brief True where the spectrum of `TIn` into `TOut` runs over float vectors.
template<typename TIn, typename TOut>
inline constexpr bool kSpectrumInVectors = std::is_same_v<std::ranges::range_value_t<TIn>, std::complex<float>> && std::ranges::contiguous_range<TIn> && std::ranges::contiguous_range<TOut> && std::is_same_v<std::ranges::range_value_t<TOut>, float>;
} // namespace detail

struct ConfigMagnitude {
    bool computeHalfSpectrum = false;
    bool outputInDb          = false;
    bool shiftSpectrum       = false;
};

/// @brief amplitude spectrum of an unnormalized DFT result.
/// Every bin of the full spectrum is scaled by 1/N. The half spectrum scales by 2/N to account for
/// the mirrored bin it drops, except at DC and Nyquist, which have no twin.
/// Float bins of a contiguous range are computed in simd vectors through `gr::meta::hypot` and `gr::meta::log10`,
/// each within 2 ulp of the exact result. The bins past the last whole vector and double bins call the C library.
template<std::ranges::input_range TContainerIn, std::ranges::output_range<typename TContainerIn::value_type::value_type> TContainerOut = std::vector<typename TContainerIn::value_type::value_type>, typename T = TContainerIn::value_type>
requires(std::is_same_v<T, std::complex<float>> || std::is_same_v<T, std::complex<double>>)
auto computeMagnitudeSpectrum(const TContainerIn& fftIn, TContainerOut&& magOut = {}, ConfigMagnitude config = {}) {
    const std::size_t N = fftIn.size();
    if (N == 0) {
        throw std::invalid_argument("fftIn cannot be empty.");
    }

    const std::size_t magSize = config.computeHalfSpectrum ? (N / 2UZ) : N;
    using PrecisionType       = typename T::value_type;
    auto computeMagnitude     = [N, halfSpectrum = config.computeHalfSpectrum, outputInDb = config.outputInDb](const auto& c, std::size_t k) {
        const bool folded = halfSpectrum && k != 0UZ && k != N / 2UZ;
        const auto scale{(folded ? PrecisionType(2.) : PrecisionType(1.)) / static_cast<PrecisionType>(N)};
        const auto mag{std::hypot(c.real(), c.imag()) * scale};
        if (outputInDb) {
            return gr::math::amplitudeToDb(mag);
        }
        return mag;
    };

    if constexpr (requires(std::size_t n) {
                      magOut.clear();
                      magOut.resize(n);
                  }) {
        magOut.clear();
        magOut.resize(magSize);
    } else {
        static_assert(std::tuple_size_v<TContainerIn> == std::tuple_size_v<TContainerOut>, "Size mismatch for fixed-size container.");
    }

    std::size_t k = 0UZ;
    if constexpr (detail::kSpectrumInVectors<const TContainerIn&, TContainerOut&>) {
        k = detail::magnitudeSpectrumVectors(std::ranges::data(fftIn), std::ranges::data(magOut), magSize, N, config.computeHalfSpectrum, config.outputInDb);
    }
    for (auto it = std::next(fftIn.begin(), static_cast<std::ptrdiff_t>(k)); k < magSize; ++k, ++it) {
        magOut[k] = computeMagnitude(*it, k);
    }

    if constexpr (std::is_same_v<T, std::complex<float>> || std::is_same_v<T, std::complex<double>>) {
        if (!config.computeHalfSpectrum && config.shiftSpectrum) {
            auto halfN = std::ssize(magOut) / 2;
            std::ranges::rotate(magOut, std::ranges::begin(magOut) + halfN); // rotate so that negative frequencies appear at the front
        }
    }

    return magOut;
}

template<std::ranges::input_range TContainerIn, typename T = TContainerIn::value_type>
requires(std::is_same_v<T, std::complex<float>> || std::is_same_v<T, std::complex<double>>)
auto computeMagnitudeSpectrum(const TContainerIn& fftIn, ConfigMagnitude config) {
    return computeMagnitudeSpectrum(fftIn, {}, config);
}

struct ConfigPhase {
    bool computeHalfSpectrum = false;
    bool outputInDeg         = false;
    bool unwrapPhase         = false;
    bool shiftSpectrum       = false;
};

template<std::ranges::input_range TContainerInOut, typename T = TContainerInOut::value_type>
requires(std::floating_point<T>)
void unwrapPhase(TContainerInOut& phase) {
    if (std::ranges::empty(phase)) {
        return;
    }

    const auto pi   = std::numbers::pi_v<T>;
    auto       prev = phase.front();
    std::transform(phase.begin() + 1, phase.end(), phase.begin() + 1, [&prev, pi](T& current) {
        T diff = current - prev;
        while (diff > pi) {
            current -= 2 * pi;
            diff = current - prev;
        }
        while (diff < -pi) {
            current += 2 * pi;
            diff = current - prev;
        }
        prev = current;
        return current;
    });
}

/// @brief phase spectrum of a DFT result, in radians or degrees.
/// Float bins of a contiguous range take their phase through `gr::meta::atan2`, within 2 ulp of the exact angle; the
/// rest call the C library.
template<std::ranges::input_range TContainerIn, std::ranges::output_range<typename TContainerIn::value_type::value_type> TContainerOut = std::vector<typename TContainerIn::value_type::value_type>, typename T = TContainerIn::value_type>
requires(std::is_same_v<T, std::complex<float>> || std::is_same_v<T, std::complex<double>>)
auto computePhaseSpectrum(const TContainerIn& fftIn, TContainerOut&& phaseOut = {}, ConfigPhase config = {}) {
    const std::size_t N = fftIn.size();
    if (N == 0) {
        throw std::invalid_argument("fftIn cannot be empty.");
    }

    std::size_t phaseSize = config.computeHalfSpectrum ? (N / 2) : N;
    if constexpr (requires(std::size_t n) {
                      phaseOut.clear();
                      phaseOut.resize(n);
                  }) {
        phaseOut.clear();
        phaseOut.resize(phaseSize);
    } else {
        static_assert(std::tuple_size_v<TContainerIn> == std::tuple_size_v<TContainerOut>, "Size mismatch for fixed-size container.");
    }

    std::size_t k = 0UZ;
    if constexpr (detail::kSpectrumInVectors<const TContainerIn&, TContainerOut&>) {
        k = detail::phaseSpectrumVectors(std::ranges::data(fftIn), std::ranges::data(phaseOut), phaseSize);
    }
    for (auto it = std::next(fftIn.begin(), static_cast<std::ptrdiff_t>(k)); k < phaseSize; ++k, ++it) {
        phaseOut[k] = std::atan2((*it).imag(), (*it).real());
    }

    if (config.unwrapPhase) {
        unwrapPhase(phaseOut);
    }

    if (config.outputInDeg) {
        std::ranges::transform(phaseOut, phaseOut.begin(), [](const auto& phase) { return phase * static_cast<typename T::value_type>(180.) * std::numbers::inv_pi_v<typename T::value_type>; });
    }

    if (!config.computeHalfSpectrum && config.shiftSpectrum) {
        auto halfN = std::ssize(phaseOut) / 2;
        std::ranges::rotate(phaseOut, phaseOut.begin() + halfN); // rotate so that negative frequencies appear at the front
    }

    return phaseOut;
}

template<std::ranges::input_range TContainerIn, typename T = TContainerIn::value_type>
requires(std::is_same_v<T, std::complex<float>> || std::is_same_v<T, std::complex<double>>)
auto computePhaseSpectrum(const TContainerIn& fftIn, ConfigPhase config) {
    return computePhaseSpectrum(fftIn, {}, config);
}

} // namespace gr::algorithm::fft
#endif // GNURADIO_ALGORITHM_FFT_COMMON_HPP
