#ifndef GNURADIO_ANALOG_DETAIL_AVERAGER_HPP
#define GNURADIO_ANALOG_DETAIL_AVERAGER_HPP

#include <cmath>
#include <format>

#include <gnuradio-4.0/Block.hpp>

namespace gr::blocks::analog::detail {

/// @brief Resolve the averager coefficient, rejecting values that would stall or overshoot the average.
[[nodiscard]] inline double averagerCoefficient(double alpha, double averagingTimeSeconds, double sampleRate) {
    if (!(alpha > 0.0) || alpha > 1.0) {
        throw gr::exception(std::format("alpha must lie in (0, 1], got {}", alpha));
    }
    if (averagingTimeSeconds <= 0.0) {
        return alpha;
    }
    if (!(sampleRate > 0.0) || !std::isfinite(sampleRate)) {
        throw gr::exception(std::format("sample_rate must be positive and finite to derive alpha, got {}", sampleRate));
    }
    return 1.0 - std::exp(-1.0 / (sampleRate * averagingTimeSeconds));
}

} // namespace gr::blocks::analog::detail

#endif // GNURADIO_ANALOG_DETAIL_AVERAGER_HPP
