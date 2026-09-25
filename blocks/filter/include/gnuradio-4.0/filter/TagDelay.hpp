#ifndef GNURADIO_FILTER_TAG_DELAY_HPP
#define GNURADIO_FILTER_TAG_DELAY_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Tag.hpp>

namespace gr::blocks::filter::detail {

/**
 * @brief Twice the delay of @p taps in samples of the rate they run at, from the centroid of their energy.
 *
 * The centroid `sum(k*|h[k]|^2) / sum(|h[k]|^2)` equals the group delay averaged over frequency with `|H|^2` as the
 * weight. A symmetric or antisymmetric set has the constant group delay `(N-1)/2`, and its centroid is exactly that. An
 * asymmetric set has no single group delay, and its centroid is the sample its energy arrives at: `k` for a lone tap at
 * `k`.
 *
 * The result is doubled because a linear-phase set of even length delays by a half sample. A centroid within a
 * thousandth of that half-sample grid is snapped to it; float rounding in a designed set moves the centroid by far
 * less. Any other centroid is rounded to the nearest whole sample. A set with no energy has no delay.
 */
template<typename TTap>
[[nodiscard]] std::uint64_t twiceTapDelay(std::span<const TTap> taps) noexcept {
    double moment = 0.0;
    double energy = 0.0;
    for (std::size_t k = 0UZ; k < taps.size(); ++k) {
        const double e = static_cast<double>(std::norm(taps[k]));
        moment += static_cast<double>(k) * e;
        energy += e;
    }
    if (!(energy > 0.0)) {
        return 0ULL;
    }
    const double twice = 2.0 * moment / energy;
    if (!std::isfinite(twice)) {
        return 0ULL;
    }
    const double grid = std::round(twice);
    if (std::abs(twice - grid) < 1e-3) {
        return static_cast<std::uint64_t>(grid);
    }
    return 2ULL * static_cast<std::uint64_t>(std::llround(0.5 * twice));
}

/**
 * @brief The output offset of input offset @p offset delayed by `twiceDelay / 2` samples at the interpolated rate.
 *
 * `floor((offset*L + twiceDelay/2) / M + 1/2)`: the delayed position is rounded once to the nearest output sample, a
 * half rounding up. At a zero delay this is `gr::filter::mapResampledOffset`, and it splits @p offset into `q*M + r`
 * the same way, so it holds wherever `2*M*L + twiceDelay` fits in 64 bits.
 */
[[nodiscard]] constexpr std::uint64_t mapDelayedOffset(std::uint64_t offset, std::uint64_t interpolation, std::uint64_t decimation, std::uint64_t twiceDelay) noexcept {
    const std::uint64_t q = offset / decimation;
    const std::uint64_t r = offset % decimation;
    return q * interpolation + (2ULL * r * interpolation + twiceDelay + decimation) / (2ULL * decimation);
}

/**
 * @brief The tag keys that state a property of the stream rather than mark a time position.
 *
 * A property describes the output from its first sample as it describes the input, so a key of this set crosses a
 * delaying block unmoved. Every other key marks a time position, such as a trigger, a burst edge or a time stamp, and
 * moves by the block's delay to the sample that carries the energy it marks.
 */
inline constexpr std::array<std::string_view, 13> kStreamPropertyKeys{"sample_rate", "signal_name", "num_channels", "signal_quantity", "signal_unit", "signal_min", "signal_max", "frequency", "context", "time", "reset_default", "store_default", "end_of_stream"};

[[nodiscard]] constexpr bool statesStreamProperty(std::string_view key) noexcept { return std::ranges::find(kStreamPropertyKeys, key) != kStreamPropertyKeys.end(); }

/// @brief Tags waiting for their output, as (output offset, tag), in output order.
using HeldTags = std::vector<std::pair<std::uint64_t, property_map>>;

/**
 * @brief Hold the entries of @p tag, which crossed at input position @p undelayed and delayed position @p delayed.
 *
 * The keys that state a property of the stream go to @p undelayed. The rest go to @p delayed, and never ahead of
 * @p latest, the position of the last delayed entry, which this advances; tags that mark a time position therefore
 * leave in the order they arrived. @p held stays in output order, and entries at one output stay in the order they
 * arrived.
 */
inline void holdTag(HeldTags& held, std::uint64_t& latest, std::uint64_t undelayed, std::uint64_t delayed, property_map tag) {
    property_map stated;
    for (auto it = tag.begin(); it != tag.end();) {
        if (statesStreamProperty(std::string_view(it->first))) {
            stated.emplace(it->first, it->second);
            it = tag.erase(it);
        } else {
            ++it;
        }
    }
    if (!stated.empty()) {
        const auto after = std::ranges::upper_bound(held, undelayed, std::ranges::less{}, &HeldTags::value_type::first);
        held.emplace(after, undelayed, std::move(stated));
    }
    if (!tag.empty()) {
        latest           = std::max(latest, delayed);
        const auto after = std::ranges::upper_bound(held, latest, std::ranges::less{}, &HeldTags::value_type::first);
        held.emplace(after, latest, std::move(tag));
    }
}

} // namespace gr::blocks::filter::detail

#endif // GNURADIO_FILTER_TAG_DELAY_HPP
