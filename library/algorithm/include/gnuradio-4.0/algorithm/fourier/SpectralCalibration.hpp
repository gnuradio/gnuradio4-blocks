#ifndef GNURADIO_ALGORITHM_SPECTRAL_CALIBRATION_HPP
#define GNURADIO_ALGORITHM_SPECTRAL_CALIBRATION_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <concepts>
#include <limits>
#include <span>

#include <gnuradio-4.0/algorithm/math/Decibel.hpp>

namespace gr::algorithm::fft {

/**
 * @brief The scalars that turn an unnormalized transform into a calibrated power spectral density.
 *
 * The density is power per hertz, in the squared units of the input. Unit power reads 0 dB. For an unnormalized
 * forward transform `X[k] = sum_n x[n] w[n] exp(-j 2 pi k n / N)`,
 *
 *     psd[k] = |X[k]|^2 / (fs * sum(w^2))
 *
 * and the band integral is `sum_k psd[k] * fs / N`. By Parseval, `sum_k |X[k]|^2 = N * sum_n |x[n] w[n]|^2`, and
 * the integral reduces to `sum_n |x[n]|^2 w[n]^2 / sum(w^2)`. For a constant-envelope input of amplitude A, the
 * integral is `A^2` for any window. The integral therefore does not depend on the window. A full-scale complex tone
 * integrates to 1. A full-scale real sine has half that power and integrates to about 1/2, -3.01 dB.
 *
 * A tone read from its peak bin needs a second correction. The window spreads a tone over several bins, and the
 * peak density is the tone's power divided by the window's equivalent noise bandwidth. The peak density times
 * `enbwHz` is the tone's power. A noise floor reads correctly without this correction.
 */
template<std::floating_point F>
struct SpectralScale {
    F density{};  ///< multiplies |X[k]|^2 to give power per hertz
    F enbwBins{}; ///< the window's equivalent noise bandwidth, in bins
    F binWidthHz{};

    /// @brief The equivalent noise bandwidth in hertz. A peak density times this value is the tone's power.
    [[nodiscard]] constexpr F enbwHz() const noexcept { return enbwBins * binWidthHz; }
};

/// @brief Sum of a window's samples, the coherent gain before division by the length.
template<std::floating_point F>
[[nodiscard]] constexpr F windowSum(std::span<const F> window) noexcept {
    F total{0};
    for (const F value : window) {
        total += value;
    }
    return total;
}

/// @brief Sum of a window's squared samples, the incoherent gain and the density denominator.
template<std::floating_point F>
[[nodiscard]] constexpr F windowPower(std::span<const F> window) noexcept {
    F total{0};
    for (const F value : window) {
        total += value * value;
    }
    return total;
}

/// @brief `N * sum(w^2) / sum(w)^2`, the window's noise bandwidth in bins. One for a rectangle, NaN for a window
/// whose samples sum to zero.
template<std::floating_point F>
[[nodiscard]] constexpr F enbwBins(std::span<const F> window) noexcept {
    const F coherent = windowSum(window);
    if (coherent == F{0}) {
        return std::numeric_limits<F>::quiet_NaN();
    }
    return static_cast<F>(window.size()) * windowPower(window) / (coherent * coherent);
}

/// @brief The scalars for one window and sample rate. The transform length is the window's length.
///
/// A sample rate that is not positive, or a window of zero power, gives a NaN density. An empty window gives a NaN
/// bin width.
template<std::floating_point F>
[[nodiscard]] constexpr SpectralScale<F> spectralScale(std::span<const F> window, F sampleRate) noexcept {
    const F power = windowPower(window);
    return SpectralScale<F>{
        .density    = (sampleRate > F{0} && power > F{0}) ? F{1} / (sampleRate * power) : std::numeric_limits<F>::quiet_NaN(),
        .enbwBins   = enbwBins(window),
        .binWidthHz = window.empty() ? std::numeric_limits<F>::quiet_NaN() : sampleRate / static_cast<F>(window.size()),
    };
}

/**
 * @brief Adds `|X[k]|^2 * scale` to `out`, two-sided or one-sided.
 *
 * A one-sided spectrum keeps bins `0 .. N/2`, with N/2 rounded down. It doubles every bin except DC and, for an even
 * N, the Nyquist bin. An odd N has no Nyquist bin. The fold is applied in power. A one-sided density then integrates
 * to the same total as the two-sided density.
 *
 * The function adds to `out` and does not clear it. A caller averages segments by accumulating into one buffer.
 */
template<std::floating_point F>
constexpr void accumulatePowerSpectrum(std::span<const std::complex<F>> spectrum, F scale, bool oneSided, std::span<F> out) noexcept {
    const std::size_t n       = spectrum.size();
    const std::size_t bins    = std::min(out.size(), oneSided ? n / 2UZ + 1UZ : n);
    const std::size_t nyquist = n % 2UZ == 0UZ ? n / 2UZ : n;
    for (std::size_t k = 0UZ; k < bins; ++k) {
        const std::complex<F>& value  = spectrum[k];
        const F                power  = value.real() * value.real() + value.imag() * value.imag();
        const bool             folded = oneSided && k != 0UZ && k != nyquist;
        out[k] += (folded ? F{2} : F{1}) * power * scale;
    }
}

/// @brief A power ratio in decibels, limited below by `floorDb`. A power that is not positive, NaN included, gives
/// `floorDb`.
template<std::floating_point F>
[[nodiscard]] constexpr F powerToDb(F power, F floorDb = static_cast<F>(gr::math::kDbFloor)) noexcept {
    if (!(power > F{0})) {
        return floorDb;
    }
    const F db = F{10} * std::log10(power);
    return db < floorDb ? floorDb : db;
}

/// @brief The band integral of a density, `sum(psd) * binWidth`.
template<std::floating_point F>
[[nodiscard]] constexpr F integratePowerDensity(std::span<const F> density, F binWidthHz) noexcept {
    F total{0};
    for (const F value : density) {
        total += value;
    }
    return total * binWidthHz;
}

} // namespace gr::algorithm::fft

#endif // GNURADIO_ALGORITHM_SPECTRAL_CALIBRATION_HPP
