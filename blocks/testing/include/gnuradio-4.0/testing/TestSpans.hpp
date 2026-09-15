#ifndef GNURADIO_TESTING_TEST_SPANS_HPP
#define GNURADIO_TESTING_TEST_SPANS_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>
#include <span>
#include <stdexcept>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Tag.hpp>

// Minimal ReaderSpanLike/WriterSpanLike pair, so a processBulk — or a forwardTags override — can be driven at an exact
// chunk size and from an exact absolute stream position without standing up a graph and a scheduler. `rawTags` plus
// `streamIndex` is the idiom SchmittTrigger reads; `tags(window)` yields the framework's own (relative index, map)
// pairs, including the negative index an unconsumed tag is presented at, and both are offered here because the blocks
// driven this way use one or the other.
//
// Tags are tracked by a cursor of their own rather than by the sample cursor. A call is handed every tag from that
// cursor up to the end of its sample window, so a tag an earlier call did not retire comes round again at a negative
// relative index; the cursor advances only at the end of a call, by the rule a released span applies (`TagCursor`).
// A call's tags are therefore not the tags of its samples: a retained tag, and a tag the block retired ahead of the
// samples it consumed, are the two transitions a tag-forwarding test exists to pin, and neither one is visible from
// the sample window alone.
namespace gr::blocks::testing::span {

struct TagReaderSpan : std::span<const gr::Tag> {
    using value_type          = gr::Tag;
    constexpr TagReaderSpan() = default;
    constexpr TagReaderSpan(std::span<const gr::Tag> tags) : std::span<const gr::Tag>(tags) {}
    constexpr bool consume(std::size_t) const noexcept { return true; }
};

struct TagWriterSpan : std::span<gr::Tag> {
    using value_type          = gr::Tag;
    constexpr TagWriterSpan() = default;
    constexpr TagWriterSpan(std::span<gr::Tag> tags) : std::span<gr::Tag>(tags) {}
    constexpr void publish(std::size_t) const noexcept {}
};

using TagView = std::pair<std::ptrdiff_t, std::reference_wrapper<const gr::property_map>>;

/// @brief What a call retires from the tag cursor when the block made no request of its own.
enum class TagRetirement {
    FirstTagOnly, ///< the tags at relative index <= 0, which is what every policy but backward propagation retires
    WholeWindow   ///< every tag below the consumed prefix, which is what backward propagation retires
};

template<typename T>
struct InputSpan : std::span<const T> {
    using value_type = T;

    TagReaderSpan rawTags{};
    std::size_t   streamIndex = 0UZ;
    std::size_t   consumed    = 0UZ;
    /// the local sample index the block asked to retire tags below; meaningful only while `tagsConsumeRequested`
    std::size_t tagsConsumed         = 0UZ;
    bool        consumeRequested     = false;
    bool        tagsConsumeRequested = false;
    bool        isConnected          = true;
    bool        isSync               = true;

    InputSpan(std::span<const T> items, std::size_t at = 0UZ, std::span<const gr::Tag> incoming = {}, bool sync = true) : std::span<const T>(items), rawTags(incoming), streamIndex(at), isSync(sync) {}

    constexpr bool consume(std::size_t nItems) noexcept {
        consumed         = nItems;
        consumeRequested = true;
        return true;
    }

    /// @brief Ask for the tags below local sample index @p untilLocalIndex; the cursor applies it at the end of the call.
    constexpr void consumeTags(std::size_t untilLocalIndex) noexcept {
        tagsConsumed         = untilLocalIndex;
        tagsConsumeRequested = true;
    }

    /// @brief The index of @p tag relative to the first sample of this call, negative for a tag behind that sample.
    [[nodiscard]] constexpr std::ptrdiff_t relativeIndex(const gr::Tag& tag) const noexcept { return tag.index >= streamIndex ? static_cast<std::ptrdiff_t>(tag.index - streamIndex) : -static_cast<std::ptrdiff_t>(streamIndex - tag.index); }

    [[nodiscard]] std::vector<TagView> tags() const {
        std::vector<TagView> view;
        view.reserve(rawTags.size());
        for (const gr::Tag& tag : rawTags) {
            view.emplace_back(relativeIndex(tag), std::cref(tag.map));
        }
        return view;
    }

    [[nodiscard]] std::vector<TagView> tags(std::size_t window) const {
        std::vector<TagView> view;
        for (const gr::Tag& tag : rawTags) {
            if (tag.index < streamIndex + window) {
                view.emplace_back(relativeIndex(tag), std::cref(tag.map));
            }
        }
        return view;
    }
};

/**
 * @brief The tag cursor of a driven port: which tags a call is handed, and which of them the call retires.
 *
 * A call is handed every tag from the cursor up to the end of its sample window, so a tag an earlier call did not
 * retire is handed over again at a negative relative index. The cursor advances at the end of the call, by the rule a
 * released span applies: the block's own `consumeTags` request when it made one; nothing at all when the block asked
 * to consume no samples; otherwise the rule the cursor was built with. A tag beyond the end of the call's window is
 * never retired, however wide the request.
 *
 * The tags are in index order, which is the order a tag buffer holds them in.
 */
class TagCursor {
public:
    explicit TagCursor(std::span<const gr::Tag> tags, TagRetirement rule = TagRetirement::FirstTagOnly) noexcept : _tags(tags), _rule(rule) {}

    /// @brief The tags a call over `[streamIndex, streamIndex + windowSize)` is handed.
    [[nodiscard]] std::span<const gr::Tag> window(std::size_t streamIndex, std::size_t windowSize) const {
        const std::span<const gr::Tag> pending = _tags.subspan(_retired);
        const auto                     end     = std::ranges::lower_bound(pending, streamIndex + windowSize, std::ranges::less{}, &gr::Tag::index);
        return pending.first(static_cast<std::size_t>(end - pending.begin()));
    }

    /// @brief Advance the cursor past what @p span retires, which is what a port does when the span is released.
    template<typename T>
    void retire(const InputSpan<T>& span) noexcept {
        const std::size_t visibleEnd = _retired + span.rawTags.size();
        if (span.tagsConsumeRequested) {
            retireBelow(span.streamIndex + span.tagsConsumed, visibleEnd);
            return;
        }
        if (span.empty() || (span.consumeRequested && span.consumed == 0UZ)) {
            return;
        }
        if (_rule == TagRetirement::FirstTagOnly) {
            retireBelow(span.streamIndex + 1UZ, visibleEnd);
        } else {
            const std::size_t prefix = span.consumeRequested ? span.consumed : (span.isSync ? span.size() : 0UZ);
            retireBelow(span.streamIndex + prefix, visibleEnd);
        }
    }

    /// @brief How many tags the cursor has retired.
    [[nodiscard]] std::size_t retired() const noexcept { return _retired; }

private:
    void retireBelow(std::size_t untilIndex, std::size_t visibleEnd) noexcept {
        while (_retired < visibleEnd && _tags[_retired].index < untilIndex) {
            ++_retired;
        }
    }

    std::span<const gr::Tag> _tags{};
    TagRetirement            _rule    = TagRetirement::FirstTagOnly;
    std::size_t              _retired = 0UZ;
};

template<typename T>
struct OutputSpan : std::span<T> {
    using value_type = T;

    std::vector<gr::Tag>* sink = nullptr;
    TagWriterSpan         tags{};
    std::size_t           streamIndex = 0UZ;
    std::size_t           count       = 0UZ;
    bool                  isConnected = true;
    bool                  isSync      = true;

    OutputSpan(std::span<T> items, std::size_t at = 0UZ, std::vector<gr::Tag>* published = nullptr, bool connected = true, bool sync = true) : std::span<T>(items), sink(published), streamIndex(at), isConnected(connected), isSync(sync) {}

    constexpr void publish(std::size_t nItems) noexcept { count = nItems; }

    void publishTag(const gr::property_map& tagData, std::size_t tagOffset = 0UZ) {
        if (!isConnected || sink == nullptr) {
            return;
        }
        sink->push_back(gr::Tag{streamIndex + tagOffset, tagData});
    }
};

/// @brief One run of a block: the main stream, `NAux` `float` side streams, and the tags each port published.
template<typename TOut, std::size_t NAux = 0UZ>
struct Capture {
    std::vector<TOut>                      samples{};
    std::array<std::vector<float>, NAux>   aux{};
    std::vector<gr::Tag>                   tags{};
    std::array<std::vector<gr::Tag>, NAux> auxTags{};
    std::size_t                            consumed = 0UZ;

    /// @brief The absolute output offsets of the published tags carrying @p key.
    [[nodiscard]] std::vector<std::size_t> offsetsOf(std::string_view key) const {
        const typename gr::property_map::key_type wanted{key};
        std::vector<std::size_t>                  offsets;
        for (const gr::Tag& tag : tags) {
            if (tag.map.contains(wanted)) {
                offsets.push_back(tag.index);
            }
        }
        return offsets;
    }
};

/// @brief The result of a run of a block whose output type is its input type.
template<typename T>
using Result = Capture<T>;

/**
 * @brief Drive a block whose output type is its input type over @p input in chunks of @p chunkSize.
 *
 * The rate is the block's own: it consumes what it takes and publishes what it makes, and the run follows what it
 * consumed rather than assuming a ratio. @p tags are injected at their absolute input offsets.
 */
template<typename TBlock, typename T>
[[nodiscard]] Capture<T> run(TBlock& block, std::span<const T> input, std::size_t chunkSize = 0UZ, std::span<const gr::Tag> tags = {}) {
    Capture<T>        result;
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    std::vector<T>    scratch(stride);
    TagCursor         cursor(tags);

    for (std::size_t base = 0UZ; base < input.size();) {
        const std::size_t count = std::min(stride, input.size() - base);

        InputSpan<T>  inSpan(input.subspan(base, count), base, cursor.window(base, count));
        OutputSpan<T> outSpan(std::span<T>(scratch.data(), count), result.samples.size(), &result.tags);
        std::ignore = block.processBulk(inSpan, outSpan);
        cursor.retire(inSpan);

        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        result.consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ) { // no progress: the chunk is shorter than the group the block requires
            break;
        }
        base += inSpan.consumed;
    }
    return result;
}

/**
 * @brief Drive a one-in one-out block over @p input in chunks of @p chunkSize, at a rate of @p outPerIn.
 *
 * @p outPerIn is the output items reserved per input item — 1 for a passthrough, `bitsPerSymbol()` for a soft
 * decoder. @p startOffset puts the whole run at an absolute stream position, which is what the tag-offset tests
 * need and what no scheduler-driven test can reach.
 */
template<typename TOut, typename TBlock, typename TIn>
[[nodiscard]] Capture<TOut> run(TBlock& block, std::span<const TIn> input, std::size_t chunkSize = 0UZ, std::size_t outPerIn = 1UZ, std::span<const gr::Tag> tags = {}, std::size_t startOffset = 0UZ) {
    Capture<TOut>     result;
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    std::vector<TOut> scratch(stride * outPerIn);
    TagCursor         cursor(tags);

    for (std::size_t base = 0UZ; base < input.size();) {
        const std::size_t count = std::min(stride, input.size() - base);

        InputSpan<TIn>   inSpan(input.subspan(base, count), startOffset + base, cursor.window(startOffset + base, count));
        OutputSpan<TOut> outSpan(std::span<TOut>(scratch.data(), count * outPerIn), outPerIn * (startOffset + base), &result.tags);

        std::ignore  = block.processBulk(inSpan, outSpan);
        auto inputs  = std::tie(inSpan);
        auto outputs = std::tie(outSpan);
        if constexpr (requires { block.forwardTags(inputs, outputs, count); }) {
            block.forwardTags(inputs, outputs, count);
        }
        cursor.retire(inSpan);

        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(count * outPerIn));
        result.consumed += count;
        base += count;
    }
    return result;
}

/**
 * @brief `run`, with a second output port beside the main one — the correlation-magnitude port of a detector.
 *
 * @p auxTags collects what the second port published, which is what a test asking whether a tag reached every port
 * needs. The call order here is the framework's — `forwardTags` first, then `processBulk` — because a block that
 * publishes on both paths has to keep one port's indices in order across the two.
 */
template<typename TOut, typename TAux, typename TBlock, typename TIn>
[[nodiscard]] Capture<TOut> run3(TBlock& block, std::span<const TIn> input, std::size_t chunkSize, std::vector<TAux>& aux, bool auxConnected = true, std::span<const gr::Tag> tags = {}, std::size_t startOffset = 0UZ, std::vector<gr::Tag>* auxTags = nullptr) {
    Capture<TOut>     result;
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    std::vector<TOut> scratch(stride);
    std::vector<TAux> auxScratch(stride);
    TagCursor         cursor(tags);
    aux.clear();

    for (std::size_t base = 0UZ; base < input.size();) {
        const std::size_t count = std::min(stride, input.size() - base);

        InputSpan<TIn>   inSpan(input.subspan(base, count), startOffset + base, cursor.window(startOffset + base, count));
        OutputSpan<TOut> outSpan(std::span<TOut>(scratch.data(), count), startOffset + base, &result.tags);
        OutputSpan<TAux> auxSpan(auxConnected ? std::span<TAux>(auxScratch.data(), count) : std::span<TAux>{}, startOffset + base, auxTags, auxConnected);

        auto inputs  = std::tie(inSpan);
        auto outputs = std::tie(outSpan, auxSpan);
        if constexpr (requires { block.forwardTags(inputs, outputs, count); }) {
            block.forwardTags(inputs, outputs, count);
        }
        std::ignore = block.processBulk(inSpan, outSpan, auxSpan);
        cursor.retire(inSpan);

        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        aux.insert(aux.end(), auxScratch.begin(), auxScratch.begin() + static_cast<std::ptrdiff_t>(auxSpan.count));
        result.consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ) {
            break;
        }
        base += inSpan.consumed;
    }
    return result;
}

/**
 * @brief Drive a variable-rate block: @p feed items arrive per call and everything not yet consumed is presented again.
 *
 * A block that needs a whole header before it can decide consumes nothing until it has one, and a scheduler answers
 * that by growing the window rather than by repeating a call it cannot serve. So a feed of one is one item arriving
 * per call, not a one-item window.
 */
template<typename TOut, typename TBlock, typename TIn>
[[nodiscard]] Capture<TOut> runVariable(TBlock& block, std::span<const TIn> input, std::size_t feed, std::size_t outRoom, std::span<const gr::Tag> tags = {}, std::size_t startOffset = 0UZ) {
    Capture<TOut>     result;
    const std::size_t arriving = feed == 0UZ ? std::max(input.size(), 1UZ) : feed;
    const std::size_t room     = outRoom == 0UZ ? std::max(input.size(), 1UZ) : outRoom;
    std::vector<TOut> scratch(room);
    TagCursor         cursor(tags);

    std::size_t consumed = 0UZ;
    std::size_t fed      = 0UZ;
    while (consumed < input.size()) {
        fed = std::min(input.size(), fed + arriving);

        InputSpan<TIn>   inSpan(input.subspan(consumed, fed - consumed), startOffset + consumed, cursor.window(startOffset + consumed, fed - consumed));
        OutputSpan<TOut> outSpan(std::span<TOut>(scratch.data(), room), result.samples.size(), &result.tags);

        std::ignore = block.processBulk(inSpan, outSpan);
        cursor.retire(inSpan);

        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            result.samples.push_back(std::move(scratch[k]));
        }
        result.consumed += inSpan.consumed;
        consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && fed == input.size()) {
            break;
        }
    }
    return result;
}

/// @brief Sentinel for `runDecimating`: the output position follows the input position through the decimation.
inline constexpr std::size_t kOutFollowsIn = static_cast<std::size_t>(-1);

/**
 * @brief Drive a decimating block over @p input in chunks of @p chunkSize inputs, which the framework hands whole.
 *
 * @p chunkSize is truncated to a multiple of @p decimation, because `input_chunk_size` is what the framework chunks by;
 * a non-zero @p chunkSize below @p decimation truncates to nothing and is rejected rather than looping forever. A
 * trailing chunk shorter than @p chunkSize is delivered as a final shorter call, truncated to a whole number of
 * outputs, so only a remainder too short to produce one output goes unprocessed.
 * @p startOffset puts the run at an absolute stream position, which is what the tag-offset tests need and what no
 * scheduler-driven test can reach; @p startOutOffset is the matching output position where the two do not divide —
 * after a mid-stream `decimation` change the outputs already produced are not `startOffset / decimation`.
 */
template<typename TOut, typename TBlock, typename TIn>
[[nodiscard]] Capture<TOut> runDecimating(TBlock& block, std::span<const TIn> input, std::size_t chunkSize, std::size_t decimation, std::span<const gr::Tag> tags = {}, std::size_t startOffset = 0UZ, std::size_t startOutOffset = kOutFollowsIn) {
    if (decimation == 0UZ) {
        throw std::invalid_argument("runDecimating: decimation must be non-zero");
    }
    const std::size_t stride = ((chunkSize == 0UZ ? input.size() : chunkSize) / decimation) * decimation;
    if (stride == 0UZ) {
        throw std::invalid_argument("runDecimating: chunkSize must be zero or at least decimation");
    }

    Capture<TOut>     result;
    const std::size_t outStart = startOutOffset == kOutFollowsIn ? startOffset / decimation : startOutOffset;
    std::vector<TOut> scratch(stride / decimation);
    TagCursor         cursor(tags);

    for (std::size_t base = 0UZ; base < input.size(); base += stride) {
        // the final chunk is whatever is left, less any remainder too short to make one output
        const std::size_t take = (std::min(stride, input.size() - base) / decimation) * decimation;
        if (take == 0UZ) {
            break;
        }

        InputSpan<TIn>   inSpan(input.subspan(base, take), startOffset + base, cursor.window(startOffset + base, take));
        OutputSpan<TOut> outSpan(std::span<TOut>(scratch.data(), take / decimation), outStart + base / decimation, &result.tags);

        auto inputs  = std::tie(inSpan);
        auto outputs = std::tie(outSpan);
        if constexpr (requires { block.forwardTags(inputs, outputs, take); }) {
            block.forwardTags(inputs, outputs, take);
        }
        std::ignore = block.processBulk(std::span<const TIn>(inSpan), std::span<TOut>(outSpan));
        cursor.retire(inSpan);

        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(take / decimation));
        result.consumed += take;
    }
    return result;
}

/**
 * @brief Drive a variable-rate block over `Async` ports: @p feed samples arrive per call and everything not yet
 * consumed is presented again, with @p outRoom output slots offered.
 *
 * That is what the framework does with an `Async` port — it hands over everything the reader holds and the block
 * consumes a prefix — so a feed of one is one sample arriving per call and not a one-sample window. It is also what
 * makes the tag path testable: a tag past the consumed prefix comes round again, and mapping it twice is the failure
 * the block's own bookkeeping exists to prevent.
 */
template<typename TOut, typename TBlock, typename TIn>
[[nodiscard]] Capture<TOut> runAsync(TBlock& block, std::span<const TIn> input, std::size_t feed, std::size_t outRoom, std::span<const gr::Tag> tags = {}, std::size_t startOffset = 0UZ) {
    Capture<TOut>     result;
    const std::size_t arriving = feed == 0UZ ? std::max(input.size(), 1UZ) : feed;
    const std::size_t room     = std::max(outRoom, 1UZ);
    std::vector<TOut> scratch(room);
    TagCursor         cursor(tags);

    std::size_t consumed = 0UZ;
    std::size_t fed      = 0UZ;
    while (consumed < input.size()) {
        fed = std::min(input.size(), fed + arriving);

        InputSpan<TIn>   inSpan(input.subspan(consumed, fed - consumed), startOffset + consumed, cursor.window(startOffset + consumed, fed - consumed), false);
        OutputSpan<TOut> outSpan(std::span<TOut>(scratch.data(), room), result.samples.size(), &result.tags, true, false);

        auto inputs  = std::tie(inSpan);
        auto outputs = std::tie(outSpan);
        if constexpr (requires { block.forwardTags(inputs, outputs, fed - consumed); }) {
            block.forwardTags(inputs, outputs, fed - consumed);
        }
        std::ignore = block.processBulk(inSpan, outSpan);
        cursor.retire(inSpan);

        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        result.consumed += inSpan.consumed;
        consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && fed == input.size()) {
            break;
        }
    }
    return result;
}

/// @brief Drive @p block over @p input in chunks of @p chunkSize, with @p connected selecting which side ports exist.
template<std::size_t NAux, typename TOut, typename TBlock, typename TIn>
[[nodiscard]] Capture<TOut, NAux> runPorts(TBlock& block, std::span<const TIn> input, std::size_t chunkSize, std::array<bool, NAux> connected, std::span<const gr::Tag> tags = {}) {
    Capture<TOut, NAux>                  result;
    const std::size_t                    stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    std::vector<TOut>                    scratch(stride);
    std::array<std::vector<float>, NAux> auxScratch;
    for (std::vector<float>& buffer : auxScratch) {
        buffer.resize(stride);
    }
    TagCursor cursor(tags);

    for (std::size_t base = 0UZ; base < input.size();) {
        const std::size_t count = std::min(stride, input.size() - base);

        InputSpan<TIn>   inSpan(input.subspan(base, count), base, cursor.window(base, count));
        OutputSpan<TOut> outSpan(std::span<TOut>(scratch.data(), count), result.samples.size(), &result.tags);

        std::array<OutputSpan<float>, NAux> auxSpans = [&]<std::size_t... I>(std::index_sequence<I...>) { return std::array<OutputSpan<float>, NAux>{OutputSpan<float>(connected[I] ? std::span<float>(auxScratch[I].data(), count) : std::span<float>{}, result.aux[I].size(), &result.auxTags[I], connected[I])...}; }(std::make_index_sequence<NAux>{});

        if constexpr (NAux == 2UZ) {
            std::ignore = block.processBulk(inSpan, outSpan, auxSpans[0], auxSpans[1]);
        } else {
            static_assert(NAux == 3UZ, "the tracking-loop blocks carry two or three optional side ports");
            std::ignore = block.processBulk(inSpan, outSpan, auxSpans[0], auxSpans[1], auxSpans[2]);
        }
        cursor.retire(inSpan);

        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        for (std::size_t which = 0UZ; which < NAux; ++which) {
            result.aux[which].insert(result.aux[which].end(), auxScratch[which].begin(), auxScratch[which].begin() + static_cast<std::ptrdiff_t>(auxSpans[which].count));
        }
        result.consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ) {
            break;
        }
        base += inSpan.consumed;
    }
    return result;
}

/**
 * @brief `runPorts` for a variable-rate block: @p feed samples arrive per call and everything not yet consumed is
 * presented again, with @p outRoom output slots offered.
 *
 * `runPorts` hands over a fixed window and moves past it, which is the right model for a block that consumes what it
 * is given. A block that needs an interpolation window consumes nothing until it has one, and a scheduler answers that
 * by growing the window rather than by repeating a call it cannot serve — so here a chunk size of one is one sample
 * arriving per call, not a one-sample window. @p startOffset puts the whole stream at an absolute input position.
 */
template<std::size_t NAux, typename TOut, typename TBlock, typename TIn>
[[nodiscard]] Capture<TOut, NAux> runPortsVariable(TBlock& block, std::span<const TIn> input, std::size_t feed, std::size_t outRoom, std::array<bool, NAux> connected, std::span<const gr::Tag> tags = {}, std::size_t startOffset = 0UZ) {
    Capture<TOut, NAux> result;
    const std::size_t   arriving = feed == 0UZ ? std::max(input.size(), 1UZ) : feed;
    const std::size_t   room     = outRoom == 0UZ ? std::max(input.size(), 1UZ) : outRoom;

    std::vector<TOut>                    scratch(room);
    std::array<std::vector<float>, NAux> auxScratch;
    for (std::vector<float>& buffer : auxScratch) {
        buffer.resize(room);
    }
    TagCursor cursor(tags);

    std::size_t consumed = 0UZ;
    std::size_t fed      = 0UZ;
    while (consumed < input.size()) {
        fed = std::min(input.size(), fed + arriving);

        InputSpan<TIn>   inSpan(input.subspan(consumed, fed - consumed), startOffset + consumed, cursor.window(startOffset + consumed, fed - consumed));
        OutputSpan<TOut> outSpan(std::span<TOut>(scratch.data(), room), result.samples.size(), &result.tags);

        std::array<OutputSpan<float>, NAux> auxSpans = [&]<std::size_t... I>(std::index_sequence<I...>) { return std::array<OutputSpan<float>, NAux>{OutputSpan<float>(connected[I] ? std::span<float>(auxScratch[I].data(), room) : std::span<float>{}, result.aux[I].size(), &result.auxTags[I], connected[I])...}; }(std::make_index_sequence<NAux>{});

        static_assert(NAux == 3UZ, "the variable-rate shim is written for the three optional ports a symbol synchronizer carries");
        std::ignore = block.processBulk(inSpan, outSpan, auxSpans[0], auxSpans[1], auxSpans[2]);
        cursor.retire(inSpan);

        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        for (std::size_t which = 0UZ; which < NAux; ++which) {
            result.aux[which].insert(result.aux[which].end(), auxScratch[which].begin(), auxScratch[which].begin() + static_cast<std::ptrdiff_t>(auxSpans[which].count));
        }
        result.consumed += inSpan.consumed;
        consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && fed == input.size()) {
            break;
        }
    }
    return result;
}

} // namespace gr::blocks::testing::span

#endif // GNURADIO_TESTING_TEST_SPANS_HPP
