#ifndef GNURADIO_ANALOG_DETAIL_AVERAGER_HPP
#define GNURADIO_ANALOG_DETAIL_AVERAGER_HPP

#include <cmath>
#include <expected>
#include <format>

#include <gnuradio-4.0/Message.hpp>

namespace gr::blocks::analog::detail {

/// @brief Resolve the averager coefficient. The error names a value that would stall or overshoot the average.
[[nodiscard]] inline std::expected<double, gr::Error> averagerCoefficient(double alpha, double averagingTimeSeconds, double sampleRate) {
    if (!(alpha > 0.0) || alpha > 1.0) {
        return std::unexpected(gr::Error(std::format("alpha must lie in (0, 1], got {}", alpha)));
    }
    if (averagingTimeSeconds <= 0.0) {
        return alpha;
    }
    if (!(sampleRate > 0.0) || !std::isfinite(sampleRate)) {
        return std::unexpected(gr::Error(std::format("sample_rate must be positive and finite to derive alpha, got {}", sampleRate)));
    }
    const double samples = sampleRate * averagingTimeSeconds;
    const double pole    = 1.0 - std::exp(-1.0 / samples);
    if (!(pole > 0.0)) {
        return std::unexpected(gr::Error(std::format("a time constant of {} samples gives a pole that rounds to 0", samples)));
    }
    return pole;
}

} // namespace gr::blocks::analog::detail

#endif // GNURADIO_ANALOG_DETAIL_AVERAGER_HPP
