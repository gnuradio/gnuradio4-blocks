#ifndef GNURADIO_SAMPLE_DELAY_HPP
#define GNURADIO_SAMPLE_DELAY_HPP

#include <algorithm>
#include <complex>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/basic/NamespaceCompatibility.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::SampleDelay, [T], [ std::uint8_t, std::int16_t, std::int32_t, float, double, std::complex<float>, std::complex<double> ])

template<typename T>
struct SampleDelay : Block<SampleDelay<T>> {
    using Description = Doc<R""(
@brief `out[n] = in[n - delay]`: the stream preceded by `delay` zeros, at exactly one output per input.

Not `gr::blocks::testing::Delay`, which is a wall-clock start-up hold and does not delay the stream at all.
`delay = 0` is a bit-exact pass-through and is the default. The line costs `delay * sizeof(T)` bytes and has no upper
bound. A tag at input offset `t` is republished at output offset `t + delay`, so a marker stays with the sample it
marks.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "delay", Doc<"out[n] = in[n - delay]; a live change re-primes the line and stays 1:1">, Visible> delay = 0U;

    GR_MAKE_REFLECTABLE(SampleDelay, in, out, delay);

    std::vector<T>                                        _line{};
    std::size_t                                           _delay  = 0UZ;
    std::size_t                                           _cursor = 0UZ;
    std::vector<std::pair<std::size_t, gr::property_map>> _pendingTags{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        const std::size_t wanted = static_cast<std::size_t>(delay.value);
        if (wanted == _delay) {
            return;
        }
        const std::size_t kept = std::min(wanted, _delay);

        std::vector<T> primed(wanted, T{}); // allocate before installing, so a throw leaves the previous line intact
        for (std::size_t k = 0UZ; k < kept; ++k) {
            primed[wanted - kept + k] = _line[(_cursor + (_delay - kept) + k) % _delay];
        }

        remapPendingTags(wanted);
        _line   = std::move(primed);
        _cursor = 0UZ;
        _delay  = wanted;
    }

    void reset() {
        std::ranges::fill(_line, T{});
        _cursor = 0UZ;
        _pendingTags.clear();
    }

    /**
     * @brief Place every input tag `delay` samples later, holding the ones whose output this call does not reach.
     *
     * Replaces the framework's forwarding rather than adjusting it, and republishes what it saw rather than only the
     * reserved keys. A held tag's position is carried relative to the next output offset, so a delay change moves it
     * with its sample by moving that position.
     */
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        std::vector<std::pair<std::size_t, gr::property_map>> arriving;
        gr::for_each_reader_span(
            [&arriving, processedIn, this](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const auto& [relIndex, tagMap] : span.tags(processedIn)) {
                    if (relIndex < 0) { // from before this window: already placed, and the framework would duplicate it
                        continue;
                    }
                    arriving.emplace_back(static_cast<std::size_t>(relIndex) + _delay, tagMap.get());
                }
            },
            inputSpans);

        if (arriving.empty() && _pendingTags.empty()) {
            return;
        }

        std::vector<std::pair<std::size_t, gr::property_map>> deferred;
        gr::for_each_writer_span(
            [&](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                const auto place = [&](const std::pair<std::size_t, gr::property_map>& tag) {
                    if (tag.first >= processedIn) { // its output is not in this call: hold it rather than move it
                        deferred.emplace_back(tag.first - processedIn, tag.second);
                        return;
                    }
                    span.publishTag(tag.second, tag.first);
                };
                for (const auto& tag : _pendingTags) { // held tags are older than anything arriving now
                    place(tag);
                }
                for (const auto& tag : arriving) {
                    place(tag);
                }
            },
            outputSpans);

        _pendingTags = std::move(deferred);
    }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        const std::size_t nSamples = std::min(input.size(), output.size());
        if (_delay == 0UZ) {
            std::copy_n(input.begin(), nSamples, output.begin());
            return work::Status::OK;
        }

        if (nSamples >= _delay) {
            const std::size_t head = _delay - _cursor;
            std::copy_n(_line.begin() + static_cast<std::ptrdiff_t>(_cursor), head, output.begin());
            std::copy_n(_line.begin(), _cursor, output.begin() + static_cast<std::ptrdiff_t>(head));
            std::copy_n(input.begin(), nSamples - _delay, output.begin() + static_cast<std::ptrdiff_t>(_delay));
            std::copy_n(input.begin() + static_cast<std::ptrdiff_t>(nSamples - _delay), _delay, _line.begin());
            _cursor = 0UZ;
            return work::Status::OK;
        }

        for (std::size_t at = 0UZ; at < nSamples;) { // shorter than the line: two runs at most, never a shift
            const std::size_t run = std::min(nSamples - at, _delay - _cursor);
            std::copy_n(_line.begin() + static_cast<std::ptrdiff_t>(_cursor), run, output.begin() + static_cast<std::ptrdiff_t>(at));
            std::copy_n(input.begin() + static_cast<std::ptrdiff_t>(at), run, _line.begin() + static_cast<std::ptrdiff_t>(_cursor));
            _cursor = _cursor + run == _delay ? 0UZ : _cursor + run;
            at += run;
        }
        return work::Status::OK;
    }

private:
    void remapPendingTags(std::size_t wanted) {
        if (wanted > _delay) {
            const std::size_t delta = wanted - _delay;
            for (auto& [position, tagMap] : _pendingTags) {
                position += delta;
            }
            return;
        }
        const std::size_t delta = _delay - wanted;
        std::erase_if(_pendingTags, [delta](const auto& tag) { return tag.first < delta; }); // its sample was discarded
        for (auto& [position, tagMap] : _pendingTags) {
            position -= delta;
        }
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_SAMPLE_DELAY_HPP
