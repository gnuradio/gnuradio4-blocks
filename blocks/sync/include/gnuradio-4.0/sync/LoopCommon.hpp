#ifndef GNURADIO_SYNC_LOOP_COMMON_HPP
#define GNURADIO_SYNC_LOOP_COMMON_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <optional>
#include <string_view>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Tag.hpp>

#include <gnuradio-4.0/algorithm/sync/ControlLoop.hpp>

namespace gr::blocks::sync {

/// @brief Stream tag: sets the loop phase, in radians, before the tagged sample is processed.
inline constexpr std::string_view kPhaseEstKey = "phase_est";
/// @brief Stream tag: sets the loop frequency, in radians per sample, before the tagged sample is processed.
inline constexpr std::string_view kFreqEstKey = "freq_est";

namespace detail {

/// @brief The loop every block in this module holds by value: `float` state, phase reduced modulo `2*pi` every step.
using CarrierLoop = gr::sync::ControlLoop<float>;

/// @brief Validate the shared loop settings and hand them to the kernel without disturbing its two state variables.
inline void applyCarrierSettings(CarrierLoop& loop, double noiseBandwidth, double damping, float minFrequency, float maxFrequency) {
    if (!(noiseBandwidth > 0.0) || !std::isfinite(noiseBandwidth)) {
        throw gr::exception(std::format("noise_bandwidth is a normalized noise bandwidth and must be positive and finite, got {}", noiseBandwidth));
    }
    if (!(damping > 0.0) || !std::isfinite(damping)) {
        throw gr::exception(std::format("damping must be positive and finite, got {}", damping));
    }
    if (!(minFrequency <= maxFrequency)) {
        throw gr::exception(std::format("min_frequency ({}) must not exceed max_frequency ({}) — crossed bounds are a limit cycle, not a clamp", minFrequency, maxFrequency));
    }
    loop.setFrequencyLimits(minFrequency, maxFrequency);
    loop.setDamping(damping);
    loop.setNoiseBandwidth(noiseBandwidth);
}

/// @brief The samples a 1:1 call may produce: the shortest connected span, ignoring the optional ports left unwired.
[[nodiscard]] inline std::size_t syncCount(std::size_t nSamples, const auto& span) noexcept { return span.isConnected ? std::min(nSamples, span.size()) : nSamples; }

/// @brief `exp(-j*p)`. Both components come out of one argument reduction.
[[nodiscard]] inline std::complex<float> derotator(float phase) noexcept { return {std::cos(phase), -std::sin(phase)}; }

/// @brief A tag payload read as a real number, or nothing when it is a different type or is not finite.
[[nodiscard]] inline std::optional<float> finiteReal(const pmt::Value& value) noexcept {
    double number = 0.0;
    if (const auto* asDouble = value.get_if<double>(); asDouble != nullptr) {
        number = *asDouble;
    } else if (const auto* asFloat = value.get_if<float>(); asFloat != nullptr) {
        number = static_cast<double>(*asFloat);
    } else {
        return std::nullopt;
    }
    return std::isfinite(number) ? std::optional<float>(static_cast<float>(number)) : std::nullopt;
}

/**
 * @brief The family's stream-tag contract: `phase_est` and `freq_est` steer the loop and are consumed, everything else
 * rides through at its own offset.
 *
 * Payloads are validated before they reach the loop — an unchecked phase would reach a subtractive wrap
 * with an argument it cannot reduce — and a non-finite one is ignored and counted.
 */
template<typename TLoop, typename FPublish>
inline void routeTag(const property_map& map, TLoop& loop, std::uint64_t& ignored, FPublish&& publish) {
    const property_map::key_type phaseKey{kPhaseEstKey};
    const property_map::key_type freqKey{kFreqEstKey};

    const auto phase = map.find(phaseKey);
    const auto freq  = map.find(freqKey);
    if (phase == map.end() && freq == map.end()) {
        publish(map);
        return;
    }

    if (freq != map.end()) {
        if (const std::optional<float> value = finiteReal(freq->second); value.has_value()) {
            loop.setFrequency(*value);
        } else {
            ++ignored;
        }
    }
    if (phase != map.end()) {
        if (const std::optional<float> value = finiteReal(phase->second); value.has_value()) {
            loop.setPhase(*value);
        } else {
            ++ignored;
        }
    }

    property_map rest = map;
    rest.erase(phaseKey);
    rest.erase(freqKey);
    if (!rest.empty()) {
        publish(rest);
    }
}

} // namespace detail

} // namespace gr::blocks::sync

#endif // GNURADIO_SYNC_LOOP_COMMON_HPP
