#ifndef GNURADIO_ALGORITHM_MATH_DECIBEL_HPP
#define GNURADIO_ALGORITHM_MATH_DECIBEL_HPP

#include <cmath>
#include <concepts>

namespace gr::math {

/// @brief The level, in dB, of every power ratio at or below `kDbFloorPower` and every amplitude at or below
/// `kDbFloorAmplitude`.
///
/// A zero, a negative or a negative-infinite argument reads as this level and never as an infinity. The floor is
/// -300 dB. The power floor 1e-30 is a normal number in `float` and in `double`. Both types floor at the same level.
inline constexpr double kDbFloor = -300.0;
/// @brief The power ratio whose level is `kDbFloor`.
inline constexpr double kDbFloorPower = 1e-30;
/// @brief The amplitude ratio whose level is `kDbFloor`.
inline constexpr double kDbFloorAmplitude = 1e-15;

/// @brief A power ratio in dB, `10 log10(linear)`, or `kDbFloor` below `kDbFloorPower`. A NaN stays a NaN.
template<std::floating_point F>
[[nodiscard]] inline F toDb(F linear) noexcept {
    if (linear < static_cast<F>(kDbFloorPower)) {
        return static_cast<F>(kDbFloor);
    }
    return F{10} * std::log10(linear);
}

/// @brief An amplitude ratio in dB, `20 log10(amplitude)`, or `kDbFloor` below `kDbFloorAmplitude`. A NaN stays a NaN.
template<std::floating_point F>
[[nodiscard]] inline F amplitudeToDb(F amplitude) noexcept {
    if (amplitude < static_cast<F>(kDbFloorAmplitude)) {
        return static_cast<F>(kDbFloor);
    }
    return F{20} * std::log10(amplitude);
}

} // namespace gr::math

#endif // GNURADIO_ALGORITHM_MATH_DECIBEL_HPP
