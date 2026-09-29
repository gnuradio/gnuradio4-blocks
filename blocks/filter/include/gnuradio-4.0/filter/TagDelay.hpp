#ifndef GNURADIO_FILTER_TAG_DELAY_HPP
#define GNURADIO_FILTER_TAG_DELAY_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <span>
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

/// @brief Tags waiting for their output, as (output offset, tag), in output order.
using HeldTags = std::vector<std::pair<std::uint64_t, property_map>>;

/**
 * @brief Hold @p tag for output @p delayed, never ahead of @p latest, the output of the last held tag, which this
 * advances.
 *
 * A tag describes the sample it sits on. Every key moves with it: a trigger, a burst edge and a time stamp as much
 * as a `sample_rate`, a `frequency` or a `context`. Tags therefore leave in the order they arrived, and @p held stays
 * in output order.
 */
inline void holdTag(HeldTags& held, std::uint64_t& latest, std::uint64_t delayed, property_map tag) {
    latest = std::max(latest, delayed);
    held.emplace_back(latest, std::move(tag));
}

} // namespace gr::blocks::filter::detail

#endif // GNURADIO_FILTER_TAG_DELAY_HPP
