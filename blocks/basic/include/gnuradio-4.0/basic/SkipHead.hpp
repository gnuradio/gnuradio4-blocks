#ifndef GNURADIO_SKIP_HEAD_HPP
#define GNURADIO_SKIP_HEAD_HPP

#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/basic/NamespaceCompatibility.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::SkipHead, [T], [ std::uint8_t, std::int16_t, std::int32_t, float, double, std::complex<float>, std::complex<double> ])

template<typename T>
struct SkipHead : Block<SkipHead<T>> {
    using Description = Doc<R""(
@brief Drops the first `n_skip` samples of the stream and passes every later sample through unchanged.

A tag on a dropped sample is dropped with it. A tag on a kept sample stays on that sample: the tag at input offset `t`
leaves at output offset `t - d`, where `d` is the number of samples dropped. `n_skip = 0` passes every sample and is
the default. A change of `n_skip` while
samples are still being dropped takes the new count, counted from the start of the stream. A count at or below the
samples already dropped ends the skip there. A change after the skip has completed has no effect; with `n_skip` 0 the
skip completes on the first call. A reset starts the count again. A stream that ends before `n_skip` samples have
arrived produces no samples, and the block ends with it.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "skip count", Doc<"samples to drop at the start of the stream">, Visible> n_skip = 0U;

    GR_MAKE_REFLECTABLE(SkipHead, in, out, n_skip);

    std::size_t _dropped  = 0UZ;   // samples consumed and not produced
    bool        _skipDone = false; // set once the count is reached; later changes of n_skip are ignored

    void reset() {
        _dropped  = 0UZ;
        _skipDone = false;
    }

    /// @brief The number of samples still to drop.
    [[nodiscard]] std::size_t remainingSkip() const noexcept {
        const std::size_t wanted = static_cast<std::size_t>(n_skip.value);
        return _skipDone || wanted <= _dropped ? 0UZ : wanted - _dropped;
    }

    /**
     * @brief Drop the tags of the samples this call drops and move the others back by the samples dropped before them.
     *
     * A tag at relative index `r` leaves at output offset `r - remainingSkip()` when `r` is at or past that count. When
     * the stream ends, the tags still on the input leave at the end-of-stream index, except those inside the skip.
     */
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        const std::size_t                                     toDrop = remainingSkip();
        std::vector<std::pair<std::size_t, gr::property_map>> kept;
        const auto                                            keep = [&kept, processedIn, toDrop](std::ptrdiff_t relIndex, const gr::property_map& tagMap) {
            if (relIndex < 0 || static_cast<std::size_t>(relIndex) < toDrop) {
                return;
            }
            gr::property_map forwarded = tagMap;
            forwarded.erase(static_cast<std::pmr::string>(gr::tag::END_OF_STREAM));
            if (!forwarded.empty()) {
                kept.emplace_back(processedIn == 0UZ ? 0UZ : static_cast<std::size_t>(relIndex) - toDrop, std::move(forwarded));
            }
        };
        std::size_t window = processedIn;
        gr::for_each_writer_span(
            [&window, toDrop](auto& span) {
                if (span.isSync && span.isConnected) { // the tags of an input tail this call cannot publish leave with the next call
                    window = std::min(window, toDrop + span.size());
                }
            },
            outputSpans);
        gr::for_each_reader_span(
            [&keep, processedIn, window](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                if (processedIn == 0UZ) {
                    for (const auto& [relIndex, tagMap] : span.tags()) {
                        keep(relIndex, tagMap.get());
                    }
                } else {
                    for (const auto& [relIndex, tagMap] : span.tags(window)) {
                        keep(relIndex, tagMap.get());
                    }
                }
            },
            inputSpans);

        if (kept.empty()) {
            return;
        }
        gr::for_each_writer_span(
            [&kept](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const auto& [offset, tagMap] : kept) {
                    span.publishTag(tagMap, offset);
                }
            },
            outputSpans);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t drop = std::min(remainingSkip(), inSpan.size());
        const std::size_t pass = std::min(inSpan.size() - drop, outSpan.size());
        _dropped += drop;
        _skipDone = remainingSkip() == 0UZ;

        std::copy_n(inSpan.begin() + static_cast<std::ptrdiff_t>(drop), pass, outSpan.begin());
        std::ignore = inSpan.consume(drop + pass);
        outSpan.publish(pass);
        return work::Status::OK;
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_SKIP_HEAD_HPP
