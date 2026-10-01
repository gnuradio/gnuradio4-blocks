#ifndef GNURADIO_FILTER_TAG_DELAY_HPP
#define GNURADIO_FILTER_TAG_DELAY_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <string>
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
 * asymmetric set has no single group delay. Its centroid is the sample its energy arrives at, `k` for a lone tap at
 * `k`.
 *
 * The result is doubled because a linear-phase set of even length delays by a half sample. A centroid within a
 * thousandth of that half-sample grid is snapped to it. Float rounding in a designed set moves the centroid by far
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
 * The result is `floor((offset*L + twiceDelay/2) / M + 1/2)`. The delayed position is rounded once to the nearest
 * output sample, and a half rounds up. At a zero delay this is `gr::filter::mapResampledOffset`. It splits @p offset
 * into `q*M + r` the same way, so the result is exact wherever `2*M*L + twiceDelay` fits in 64 bits.
 */
[[nodiscard]] constexpr std::uint64_t mapDelayedOffset(std::uint64_t offset, std::uint64_t interpolation, std::uint64_t decimation, std::uint64_t twiceDelay) noexcept {
    const std::uint64_t q = offset / decimation;
    const std::uint64_t r = offset % decimation;
    return q * interpolation + (2ULL * r * interpolation + twiceDelay + decimation) / (2ULL * decimation);
}

/// @brief Tags waiting for their output, as (output offset, tag) pairs in output order.
using HeldTags = std::vector<std::pair<std::uint64_t, property_map>>;

/**
 * @brief Hold @p tag for output @p delayed, or for @p latest when that is later. @p latest is the output of the last
 * held tag, and this call advances it.
 *
 * A tag describes the sample it sits on, so all its keys move with it. A `sample_rate`, `frequency` or `context` key
 * moves the same way as a trigger, a burst edge or a time stamp. Tags leave in the order they arrived, and @p held
 * stays in output order.
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
 * takes more than one input per output can also end on a partial chunk. The framework passes that chunk to the block's
 * epilogue alone, and it makes no output. When the stream ends, every tag still held leaves at the end-of-stream index,
 * one past the last output. The framework publishes its `end_of_stream` tag at that index. The tags left on the input
 * past the last sample the block consumed leave there too. No tag moves onto an earlier output, and the block holds
 * back no input to make an output for a tag. A stop request ends no stream. At a stop request the framework stops the
 * block without running its epilogue, except in the two cases `dropAtStop` names. The held tags are dropped with their
 * samples.
 */
struct TagDelayLine {
    static constexpr std::uint64_t kStreamEnd = std::numeric_limits<std::uint64_t>::max(); ///< the output of a tag held for the end-of-stream index

    HeldTags      held;
    std::uint64_t takenUntil = 0ULL; ///< the input offset below which every tag is held or published

    /// @brief Forget every held tag.
    void reset() noexcept {
        held.clear();
        takenUntil = 0ULL;
    }

    /**
     * @brief Hold the tags of the first @p processedIn samples of @p span that no earlier call took, each on the output
     * @p place returns for its input offset. @p place may rewrite the tag. A tag it leaves empty is dropped.
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

    /**
     * @brief Hold for the end-of-stream index every tag in the tag ring of @p in at or past input offset @p from, each
     * rewritten by @p rewrite. The ring holds such a tag when no call consumed its sample. It also holds the tags past
     * the last input sample. The `end_of_stream` key is removed, since the framework publishes its own.
     */
    template<typename TPort, typename TRewrite>
    void takeRemainder(TPort& in, std::uint64_t from, TRewrite&& rewrite) {
        if (!in.isConnected()) {
            return;
        }
        const std::pmr::string endOfStream = static_cast<std::pmr::string>(gr::tag::END_OF_STREAM);
        for (const Tag& ringTag : in.tagReader().get()) {
            if (static_cast<std::uint64_t>(ringTag.index) < from) {
                continue;
            }
            property_map tag(ringTag.map);
            tag.erase(endOfStream);
            rewrite(tag);
            if (!tag.empty()) {
                held.emplace_back(kStreamEnd, std::move(tag));
            }
        }
    }

    /// @brief The outputs a call of @p chunks input chunks of @p outChunk outputs each makes into a span of @p room
    /// outputs.
    [[nodiscard]] static constexpr std::size_t outputsToMake(std::size_t chunks, std::size_t room, std::size_t outChunk) noexcept { return std::min(chunks, room / outChunk) * outChunk; }

    /**
     * @brief Publish on @p span the held tags whose output lies among its first @p made outputs, and hold the rest.
     * With @p streamEnds every held tag leaves. A tag whose output lies past those outputs then leaves at the index
     * after them, the end-of-stream index.
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
 * @brief Whether @p block runs its epilogue under a stop request. If so, drop every tag @p tags holds and withdraw the
 * tags already placed on @p output. Publish no output there.
 *
 * The framework stops a block at a stop request before any epilogue. It still runs the epilogue under one when the stop
 * arrives after the call's lifecycle check, or when no output is connected. The epilogue then publishes no sample, no
 * held tag and no tag placed on its span before it. That includes the forwarding's tags and the framework's forwarded
 * settings tag. A call's forwarding checks no state. Its tags leave with the outputs the framework publishes for the
 * call.
 */
template<typename TBlock, typename TOutput>
[[nodiscard]] bool dropAtStop(const TBlock& block, TagDelayLine& tags, TOutput& output) {
    if (!lifecycle::isShuttingDown(block.state())) {
        return false;
    }
    static_assert(requires { output.tagsPublished = 0UZ; }, "the epilogue's output span counts the tags it publishes in `tagsPublished`");
    tags.reset();
    output.tagsPublished = 0UZ;
    output.publish(0UZ);
    return true;
}

/**
 * @brief The tag placement of a synchronous block that filters sample by sample and decimates by `M >= 1`.
 *
 * A tag leaves on the output that carries its input sample's energy. A tag held past the stream's last output leaves at
 * the end-of-stream index.
 *
 * `TDerived` has the input port `in` and provides three functions. `tagDecimation()` returns the input samples per
 * output. `twiceTagDelay()` returns the delay in half input samples, or no value when the framework places the tags.
 * `filterSamples(input, output)` filters whole input chunks into their outputs. A tag on input `i` leaves on output
 * `round((i + d) / M)`, and a half rounds up. The tag keeps that output through any later delay or decimation change. A
 * `sample_rate` key is divided by `M` when the tag is taken in.
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
     * own forwarding runs. Tags still held from an earlier delay then leave on the call's first output, ahead of every
     * tag the framework forwards.
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

    /**
     * @brief Make as many of the outputs of the stream's last input as the output span holds, and publish every held
     * tag, at the end-of-stream index for a tag past the last output.
     *
     * Without a delay the epilogue makes no output. The tags that no call forwarded leave at the end-of-stream index
     * through the framework's key filter. The tags of a partial last chunk are among them.
     */
    template<InputSpanLike TInput, OutputSpanLike TOutput>
    [[nodiscard]] work::Status processEpilogue(TInput& input, TOutput& output) {
        if (dropAtStop(self(), _tags, output)) {
            return work::Status::OK;
        }
        const std::uint64_t past    = static_cast<std::uint64_t>(input.streamIndex) + static_cast<std::uint64_t>(input.size());
        std::size_t         outputs = 0UZ;
        if (self().twiceTagDelay().has_value()) {
            outputs = std::min(input.size() / self().tagDecimation(), output.size());
            if (outputs > 0UZ) {
                std::ignore = processBulk(std::span<const TIn>(input.data(), outputs * self().tagDecimation()), std::span<TOut>(output.data(), outputs));
            }
            _tags.takeRemainder(self().in, past, [this](property_map& tag) { self().scaleSampleRateByChunkRatio(tag); });
        } else {
            std::optional<property_map> cachedSettings;
            const auto                  forwarded = [&](property_map& tag) { tag = self().filterAndSubstituteTag(tag, cachedSettings); };
            if (input.size() < self().tagDecimation()) {
                _tags.take(input, input.size(), [&](std::uint64_t, property_map& tag) {
                    forwarded(tag);
                    return TagDelayLine::kStreamEnd;
                });
            }
            _tags.takeRemainder(self().in, past, forwarded);
        }
        _tags.release(output, outputs, true);
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
