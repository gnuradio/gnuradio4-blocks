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

/**
 * @brief The tags a delaying block holds for outputs it has not made yet, and the input it keeps back so that the end
 * of its stream finds an output to carry them.
 *
 * A tag leaves on the output that carries its input sample's energy, up to the block's delay past its input. The block
 * makes no output past its last input, so a tag on one of the last inputs lies past every output. While a tag is held
 * past the outputs of a call, the block keeps back its last input chunk and asks for two chunks a call. When the
 * stream ends, the framework hands the kept chunk to the block's epilogue, whose outputs are the stream's last, and
 * every tag still held leaves on the last of them. A block whose input is not connected keeps nothing back.
 */
struct TagDelayLine {
    HeldTags      held;
    std::uint64_t takenUntil     = 0ULL;  ///< the input offset below which every tag is held or published
    bool          keepBack       = false; ///< the current call keeps back its last input chunk
    std::size_t   freeMinSamples = 0UZ;   ///< the input port's `min_samples` while no tag is held past a call
    bool          captured       = false; ///< `freeMinSamples` holds the port's own value

    /// @brief Forget every held tag, and give @p in back the `min_samples` it had before any tag was held.
    template<typename TPort>
    void reset(TPort& in) {
        if (!captured) {
            freeMinSamples = in.min_samples;
            captured       = true;
        }
        in.min_samples = freeMinSamples;
        held.clear();
        takenUntil = 0ULL;
        keepBack   = false;
    }

    /**
     * @brief Hold the tags of the first @p processedIn samples of @p span that no earlier call took, each on the output
     * @p place returns for its input offset. @p place may rewrite the tag it is handed.
     */
    template<typename TSpan, typename TPlace>
    void take(TSpan& span, std::size_t processedIn, TPlace&& place) {
        std::uint64_t       latest = held.empty() ? 0ULL : held.back().first;
        const std::uint64_t first  = static_cast<std::uint64_t>(span.streamIndex);
        for (const auto& [relIndex, tagMap] : span.tags(processedIn)) {
            if (relIndex < 0) {
                continue;
            }
            const std::uint64_t at = first + static_cast<std::uint64_t>(relIndex);
            if (at < takenUntil) { // a kept-back sample, presented again
                continue;
            }
            property_map        tag(tagMap.get());
            const std::uint64_t output = place(at, tag);
            holdTag(held, latest, output, std::move(tag));
        }
        takenUntil = std::max(takenUntil, first + static_cast<std::uint64_t>(processedIn));
    }

    /**
     * @brief The outputs a call of @p chunks input chunks makes, from output @p outBase: every chunk's @p outChunk
     * outputs, less one chunk while a held tag lies past them and @p in is connected. A call of one chunk then makes
     * none, and the input waits for a second chunk or for the stream's end. Sets the `min_samples` of @p in for the
     * next call: two chunks of @p inChunk while a tag is held past this call.
     */
    template<typename TPort>
    [[nodiscard]] std::size_t outputsToMake(TPort& in, std::size_t chunks, std::size_t inChunk, std::size_t outChunk, std::uint64_t outBase) {
        std::size_t made    = chunks * outChunk;
        const bool  waiting = in.isConnected() && !held.empty() && held.back().first >= outBase + made;
        keepBack            = waiting && chunks > 0UZ;
        if (keepBack) {
            made -= outChunk;
        }
        in.min_samples = waiting ? std::max(freeMinSamples, 2UZ * inChunk) : freeMinSamples;
        return made;
    }

    /**
     * @brief Publish on @p span the held tags whose output lies among its first @p made outputs, and hold the rest.
     * With @p streamEnds every held tag leaves, one past those outputs on the last of them, or on the first output of
     * @p span when it makes none.
     */
    template<typename TSpan>
    void release(TSpan& span, std::size_t made, bool streamEnds) {
        if (held.empty()) {
            return;
        }
        const std::uint64_t base = static_cast<std::uint64_t>(span.streamIndex);
        const std::uint64_t end  = base + static_cast<std::uint64_t>(made);
        const std::uint64_t last = made > 0UZ ? end - 1ULL : base;
        HeldTags            deferred;
        for (auto& [output, tag] : held) {
            if (output >= end && !streamEnds) {
                deferred.emplace_back(output, std::move(tag));
                continue;
            }
            const std::uint64_t at = std::min(output, last);
            span.publishTag(tag, static_cast<std::size_t>(at > base ? at - base : 0ULL));
        }
        held = std::move(deferred);
    }
};

} // namespace gr::blocks::filter::detail

#endif // GNURADIO_FILTER_TAG_DELAY_HPP
