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
 * @brief The tags a delaying block holds for outputs it has not made yet, and the input it keeps back for the end of
 * its stream.
 *
 * A tag leaves on the output that carries its input sample's energy, up to the block's delay past its input. The block
 * makes no output past its last input. A tag on one of the last inputs therefore lies past every output. A block that
 * takes more than one input per output also meets a partial last chunk: the framework hands those inputs to the
 * block's epilogue alone, and they make no output. The block keeps back its last input chunk on each call while a tag
 * is held past the call's outputs, and on every call where a chunk holds more than one input. It then asks for two
 * chunks a call. When the stream ends, the framework hands the kept chunk and any partial chunk to the epilogue. The
 * epilogue makes the stream's last output from the kept chunk, and every tag still held leaves on that output.
 *
 * A block over `Async` ports keeps back, on every call, the input samples from the one that completes its last output,
 * and asks for one sample more than it keeps. A block whose input is not connected, or whose input buffer cannot hold
 * two chunks, keeps nothing back. A tag held past the stream's last output then leaves at the index after that output,
 * the index of the stream's end.
 */
struct TagDelayLine {
    HeldTags      held;
    std::uint64_t takenUntil     = 0ULL;  ///< the input offset below which every tag is held or published
    bool          keepBack       = false; ///< the current call keeps back its last input chunk
    std::size_t   freeMinSamples = 0UZ;   ///< the input port's `min_samples` where a call keeps nothing back
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
            if (at < takenUntil) { // a kept-back sample, presented again
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

    /**
     * @brief The outputs a call of @p chunks input chunks makes, from output @p outBase into a span of @p room outputs:
     * the @p outChunk outputs of every chunk the span holds, less the last chunk's where the call keeps it back. A call
     * of one chunk then makes none, and the input waits for a second chunk or for the stream's end. Sets the
     * `min_samples` of @p in for the next call: two chunks of @p inChunk where calls keep back.
     */
    template<typename TPort>
    [[nodiscard]] std::size_t outputsToMake(TPort& in, std::size_t chunks, std::size_t room, std::size_t inChunk, std::size_t outChunk, std::uint64_t outBase) {
        const std::size_t made  = std::min(chunks, room / outChunk) * outChunk;
        const bool        keeps = keepsBack(in, inChunk, outBase + made);
        keepBack                = keeps && chunks > 0UZ;
        in.min_samples          = keeps ? std::max(freeMinSamples, 2UZ * inChunk) : freeMinSamples;
        return keepBack ? made - outChunk : made;
    }

    /**
     * @brief Whether a call keeps back its last input chunk of @p inChunk samples: on every call where a chunk holds
     * more than one input, and while a tag is held at or past output @p end. Only a connected input whose buffer holds
     * two chunks keeps anything back.
     */
    template<typename TPort>
    [[nodiscard]] bool keepsBack(const TPort& in, std::size_t inChunk, std::uint64_t end) const {
        return in.isConnected() && (inChunk > 1UZ || heldPast(end)) && 2UZ * inChunk <= in.bufferSize();
    }

    /// @brief Whether a held tag lies at or past output @p end.
    [[nodiscard]] bool heldPast(std::uint64_t end) const noexcept { return !held.empty() && held.back().first >= end; }

    /**
     * @brief Record that a call over `Async` ports keeps back @p kept input samples, and set the `min_samples` of @p in
     * to one more than those while it keeps any. The framework then ends the stream when only the kept samples remain.
     */
    template<typename TPort>
    void keepInputs(TPort& in, std::size_t kept) {
        keepBack       = kept > 0UZ;
        in.min_samples = keepBack ? std::max(freeMinSamples, kept + 1UZ) : freeMinSamples;
    }

    /**
     * @brief Publish on @p span the held tags whose output lies among its first @p made outputs, and hold the rest.
     * With @p streamEnds every held tag leaves, and a tag whose output lies past those outputs leaves on the last of
     * them. A @p span that makes no output takes the tags at its first index, the index after the last output published
     * before it.
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

/**
 * @brief The tag placement of a synchronous block that filters sample by sample and decimates by one or more: a tag
 * leaves on the output that carries its input sample's energy, and a tag held past the stream's last output leaves on
 * that output.
 *
 * `TDerived` has the input port `in` and provides `tagDecimation()`, the input samples per output;
 * `twiceTagDelay()`, the delay in half input samples, or no value where the framework places the tags; and
 * `filterSamples(input, output)`, which filters whole input chunks into their outputs. Each tag passes the framework's
 * key filter and setting substitution as it is taken in, so only its position differs from the framework's own
 * forwarding. A tag on input `i` leaves on output `round((i + d) / M)`, a half rounding up, and keeps that output
 * whatever delay or decimation change follows.
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
        _tags.reset(self().in);
        _inOrigin  = 0ULL;
        _outOrigin = 0ULL;
        _reorigin  = false;
    }

    /// @brief A decimation change applied between calls takes its new origin on the next call.
    void tagsMarkReorigin() noexcept { _reorigin = true; }

    /**
     * @brief Hold each tag for its delayed output and publish those this call makes. Without a delay, the framework's
     * own forwarding runs, and the tags still held leave on the call's outputs.
     */
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        const std::size_t                  decimation = self().tagDecimation();
        const std::optional<std::uint64_t> twiceDelay = self().twiceTagDelay();
        if (!twiceDelay.has_value()) {
            _tags.keepBack = false;
            if (_tags.captured) {
                self().in.min_samples = _tags.freeMinSamples;
            }
            gr::for_each_writer_span(
                [processedIn, decimation, this](auto& span) {
                    if (span.isSync && span.isConnected) {
                        _tags.release(span, processedIn / decimation, true);
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

        std::optional<property_map> cachedSettings;
        gr::for_each_reader_span(
            [&](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                _tags.take(span, processedIn, [&](std::uint64_t at, property_map& tag) {
                    tag = self().filterAndSubstituteTag(tag, cachedSettings);
                    return _outOrigin + mapDelayedOffset(at - _inOrigin, 1ULL, decimation, *twiceDelay);
                });
            },
            inputSpans);

        gr::for_each_writer_span(
            [processedIn, decimation, this](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                const std::size_t made = _tags.outputsToMake(self().in, processedIn / decimation, span.size(), decimation, 1UZ, static_cast<std::uint64_t>(span.streamIndex));
                if (made > 0UZ) { // a call that makes nothing publishes nothing
                    _tags.release(span, made, false);
                }
            },
            outputSpans);
    }

    /// @brief One call's samples, less the input chunk and its output that a held tag keeps back for the stream's end.
    template<InputSpanLike TInput, OutputSpanLike TOutput>
    [[nodiscard]] work::Status processBulk(TInput& input, TOutput& output) {
        if (!_tags.keepBack) {
            return processBulk(std::span<const TIn>(input.data(), input.size()), std::span<TOut>(output.data(), output.size()));
        }
        const std::size_t inputs  = input.size() - self().tagDecimation();
        const std::size_t outputs = output.size() - 1UZ;
        if (outputs == 0UZ) {
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        const work::Status status = processBulk(std::span<const TIn>(input.data(), inputs), std::span<TOut>(output.data(), outputs));
        std::ignore               = input.consume(inputs);
        output.publish(outputs);
        return status;
    }

    /// @brief The stream's last input chunks, the kept-back one among them, and every held tag on their last output.
    /// Without a delay the stream's partial last chunk makes no output.
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
        _tags.keepBack = false;
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
