#ifndef GNURADIO_FILTER_TAG_DELAY_HPP
#define GNURADIO_FILTER_TAG_DELAY_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
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
 * @brief The tags a delaying block holds for outputs it has not made yet.
 *
 * A tag leaves on the output that carries its input sample's energy, up to the block's delay past its input. The block
 * makes no output past its last input. A tag on one of the last inputs therefore lies past every output. A block that
 * takes more than one input per output also meets a partial last chunk: the framework hands those inputs to the
 * block's epilogue alone, and they make no output. When the stream ends, every tag still held leaves at the
 * end-of-stream index, one past the last output, where the framework publishes its `end_of_stream` tag. No tag moves
 * onto an earlier output, and the block keeps no input back to make an output for it.
 */
struct TagDelayLine {
    HeldTags      held;
    std::uint64_t takenUntil = 0ULL; ///< the input offset below which every tag is held or published

    /// @brief Forget every held tag.
    void reset() noexcept {
        held.clear();
        takenUntil = 0ULL;
    }

    /**
     * @brief Hold the tags of the first @p processedIn samples of @p span that no earlier call took, each on the output
     * @p place returns for its input offset. @p place may rewrite the tag it is handed, and a tag it empties is dropped.
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
            if (at < takenUntil) { // taken by an earlier call
                continue;
            }
            property_map        tag(tagMap.get());
            const std::uint64_t output = place(at, tag);
            if (!tag.empty()) {
                holdTag(held, latest, output, std::move(tag));
            }
        }
        takenUntil = std::max(takenUntil, first + static_cast<std::uint64_t>(processedIn));
    }

    /// @brief The outputs a call of @p chunks input chunks of @p outChunk outputs each makes into a span of @p room
    /// outputs.
    [[nodiscard]] static constexpr std::size_t outputsToMake(std::size_t chunks, std::size_t room, std::size_t outChunk) noexcept { return std::min(chunks, room / outChunk) * outChunk; }

    /**
     * @brief Publish on @p span the held tags whose output lies among its first @p made outputs, and hold the rest.
     * With @p streamEnds every held tag leaves, and a tag whose output lies past those outputs leaves at the index after
     * them, the end-of-stream index.
     */
    template<typename TSpan>
    void release(TSpan& span, std::size_t made, bool streamEnds) {
        if (held.empty()) {
            return;
        }
        const std::uint64_t base = static_cast<std::uint64_t>(span.streamIndex);
        const std::uint64_t end  = base + static_cast<std::uint64_t>(made);
        HeldTags            deferred;
        for (auto& [output, tag] : held) {
            if (output >= end && !streamEnds) {
                deferred.emplace_back(output, std::move(tag));
                continue;
            }
            const std::uint64_t at = std::min(output, end);
            span.publishTag(tag, static_cast<std::size_t>(at > base ? at - base : 0ULL));
        }
        held = std::move(deferred);
    }
};

/**
 * @brief The tag placement of a synchronous block that filters sample by sample and decimates by one or more: a tag
 * leaves on the output that carries its input sample's energy, and a tag held past the stream's last output leaves at
 * the end-of-stream index.
 *
 * `TDerived` has the input port `in` and provides `tagDecimation()`, the input samples per output;
 * `twiceTagDelay()`, the delay in half input samples, or no value where the framework places the tags; and
 * `filterSamples(input, output)`, which filters whole input chunks into their outputs. A tag on input `i` leaves whole,
 * every key with it, on output `round((i + d) / M)`, a half rounding up, and keeps that output whatever delay or
 * decimation change follows. A `sample_rate` key is divided by `M` as the tag is taken in.
 */
template<typename TDerived, typename TIn, typename TOut>
struct DelayedTagFilter {
    TagDelayLine  _tags;
    std::uint64_t _inOrigin  = 0ULL;
    std::uint64_t _outOrigin = 0ULL;
    bool          _reorigin  = false;

    [[nodiscard]] TDerived& self() noexcept { return static_cast<TDerived&>(*this); }

    /// @brief Forget every held tag and start the map at the stream's first sample.
    void tagsStart() {
        _tags.reset();
        _inOrigin  = 0ULL;
        _outOrigin = 0ULL;
        _reorigin  = false;
    }

    /// @brief A decimation change applied between calls takes its new origin on the next call.
    void tagsMarkReorigin() noexcept { _reorigin = true; }

    /**
     * @brief Hold each tag for its delayed output and publish those this call makes. Without a delay, the framework's
     * own forwarding runs. The tags still held from a delay in force before leave on the call's first output, ahead of
     * every tag the framework forwards.
     */
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        const std::size_t                  decimation = self().tagDecimation();
        const std::optional<std::uint64_t> twiceDelay = self().twiceTagDelay();
        if (!twiceDelay.has_value()) {
            gr::for_each_writer_span(
                [this](auto& span) {
                    if (span.isSync && span.isConnected) {
                        _tags.release(span, 0UZ, true);
                    }
                },
                outputSpans);
            if (processedIn >= decimation && processedIn > 0UZ) { // the epilogue's partial chunk makes no output
                self().forwardInputTags(inputSpans, outputSpans, processedIn);
            }
            return;
        }

        if (_reorigin) {
            gr::for_each_reader_span(
                [this](auto& span) {
                    if (span.isSync && span.isConnected) {
                        _inOrigin = static_cast<std::uint64_t>(span.streamIndex);
                    }
                },
                inputSpans);
            gr::for_each_writer_span(
                [this](auto& span) {
                    if (span.isSync && span.isConnected) {
                        _outOrigin = static_cast<std::uint64_t>(span.streamIndex);
                    }
                },
                outputSpans);
            _reorigin = false;
        }

        gr::for_each_reader_span(
            [&](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                _tags.take(span, processedIn, [&](std::uint64_t at, property_map& tag) {
                    self().scaleSampleRateByChunkRatio(tag); // the ratio in force where the tag crossed, not where it is published
                    return _outOrigin + mapDelayedOffset(at - _inOrigin, 1ULL, decimation, *twiceDelay);
                });
            },
            inputSpans);

        gr::for_each_writer_span(
            [processedIn, decimation, this](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                const std::size_t made = TagDelayLine::outputsToMake(processedIn / decimation, span.size(), 1UZ);
                if (made > 0UZ) { // a call that makes nothing publishes nothing
                    _tags.release(span, made, false);
                }
            },
            outputSpans);
    }

    /// @brief The stream's last whole input chunks, and every held tag: a tag past their outputs leaves at the
    /// end-of-stream index. Without a delay the epilogue makes no output.
    template<InputSpanLike TInput, OutputSpanLike TOutput>
    [[nodiscard]] work::Status processEpilogue(TInput& input, TOutput& output) {
        std::size_t outputs = 0UZ;
        if (self().twiceTagDelay().has_value()) {
            outputs = std::min(input.size() / self().tagDecimation(), output.size());
            if (outputs > 0UZ) {
                std::ignore = processBulk(std::span<const TIn>(input.data(), outputs * self().tagDecimation()), std::span<TOut>(output.data(), outputs));
            }
            _tags.release(output, outputs, true);
        }
        output.publish(outputs);
        return work::Status::OK;
    }

    /// @brief Filter @p input, whole chunks of `tagDecimation()` samples, into one output per chunk.
    [[nodiscard]] work::Status processBulk(std::span<const TIn> input, std::span<TOut> output) {
        self().filterSamples(input, output);
        return work::Status::OK;
    }
};

} // namespace gr::blocks::filter::detail

#endif // GNURADIO_FILTER_TAG_DELAY_HPP
