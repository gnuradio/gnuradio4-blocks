#ifndef GNURADIO_DECIMATORS_HPP
#define GNURADIO_DECIMATORS_HPP

#include <algorithm>
#include <complex>
#include <cstdint>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::basic {

namespace detail {

/**
 * @brief Republishes input tags at the output offsets where a decimator placed their samples.
 *
 * Each tag moves to the next item the block keeps. The mapping counts items in integers against the block's own phase
 * and is exact at any stream offset. A tag with no kept item left in the call waits for the next call. At the end of
 * the stream the block publishes every waiting tag at the end-of-stream index, one past its last output. Order and
 * multiplicity are preserved.
 */
struct TagRelay {
    std::vector<property_map> _carry{};
    std::size_t               _cursor = 0UZ;
    std::size_t               _count  = 0UZ;

    void reset() {
        _carry.clear();
        _cursor = 0UZ;
        _count  = 0UZ;
    }

    template<typename TInSpan>
    void begin(const TInSpan& inSpan) noexcept {
        _cursor = 0UZ;
        _count  = inSpan.rawTags.size();
    }

    template<typename TInSpan>
    void collect(const TInSpan& inSpan, std::size_t index) {
        const auto&       raw   = inSpan.rawTags;
        const std::size_t until = inSpan.streamIndex + index;
        while (_cursor < _count && raw[_cursor].index <= until) {
            if (raw[_cursor].index >= inSpan.streamIndex) {
                _carry.push_back(raw[_cursor].map);
            }
            ++_cursor;
        }
    }

    /// Holds every tag in the tag ring of `port` at or past input offset `from`, without its `end_of_stream` key.
    template<typename TPort>
    void collectPastEnd(TPort& port, std::size_t from) {
        if (!port.isConnected()) {
            return;
        }
        const std::pmr::string endOfStream = static_cast<std::pmr::string>(gr::tag::END_OF_STREAM);
        for (const Tag& ringTag : port.tagReader().get()) {
            if (ringTag.index < from) {
                continue;
            }
            property_map tag(ringTag.map);
            tag.erase(endOfStream);
            if (!tag.empty()) {
                _carry.push_back(std::move(tag));
            }
        }
    }

    template<typename TOutSpan>
    void release(TOutSpan& outSpan, std::size_t at) {
        for (const property_map& forwarded : _carry) {
            outSpan.publishTag(forwarded, at);
        }
        _carry.clear();
    }

    /// True when nothing waits and nothing is left to collect. A run of items can then be copied in one step instead of
    /// item by item. Meaningful only after `begin`.
    [[nodiscard]] bool idle() const noexcept { return _carry.empty() && _cursor >= _count; }
};

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::basic::KeepOneInN, [T], [ float, std::complex<float>, std::int16_t, std::uint8_t ])

template<typename T>
struct KeepOneInN : Block<KeepOneInN<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Keeps the last item of every group of `n` and drops the rest.

The block thins a stream without filtering it, for a display or a slow status stream. It takes items of one type and
gives the same type at `1/n` of the input rate. The first item out is the `n`-th input, then every `n`-th input after
it. A large `n` needs no buffer of `n` items.

A tag on a dropped item moves forward to the next kept item. A `sample_rate` tag leaves with its value divided by `n`.
At the end of the stream a tag still waiting for a kept item leaves at the end-of-stream index, one past the last
output.

`n` is at least one. A change of `n` restarts the group at once, and the next item out is the `n`-th input after the
change. Once the block has passed a `sample_rate` tag on, the next kept item after the change carries the last such
tag's input rate divided by the new `n`, ahead of the tags waiting for the item. `reset()` restarts the group the same
way and keeps the waiting tags.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "n", Doc<"group size with the last item of each group kept">, Limits<1U, std::numeric_limits<gr::Size_t>::max()>> n = 1U;

    GR_MAKE_REFLECTABLE(KeepOneInN, in, out, n);

    gr::Size_t       _n         = n.value; ///< the accepted `n`
    gr::Size_t       _counter   = n.value;
    float            _inputRate = 0.f; ///< the input rate of the last `sample_rate` tag published in this run, 0 before the first
    detail::TagRelay _relay{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (!newSettings.contains("n") || n.value == _n) {
            return;
        }
        _n       = n.value;
        _counter = _n;
        if (_inputRate > 0.f) {
            _relay._carry.insert(_relay._carry.begin(), property_map{{std::pmr::string(gr::tag::SAMPLE_RATE.shortKey()), pmt::Value(_inputRate)}});
        }
    }

    void start() {
        _inputRate = 0.f;
        _relay.reset();
    }

    void reset() { _counter = _n; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        outSpan.publish(keep(inSpan, outSpan));
        return work::Status::OK;
    }

    /// At the end of the stream the block keeps what the trailing input holds and publishes every waiting tag at the
    /// end-of-stream index. At a stop request it publishes nothing and drops the waiting tags.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (lifecycle::isShuttingDown(this->state())) {
            _relay.reset();
            outSpan.publish(0UZ);
            return work::Status::OK;
        }
        const std::size_t made = keep(inSpan, outSpan);
        _relay.collectPastEnd(in, inSpan.streamIndex + inSpan.size());
        releaseWaiting(outSpan, made);
        outSpan.publish(made);
        return work::Status::OK;
    }

private:
    [[nodiscard]] std::size_t keep(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t made   = 0UZ;
        std::size_t walked = 0UZ;
        _relay.begin(inSpan);

        for (; walked < inSpan.size() && made < outSpan.size(); ++walked) {
            _relay.collect(inSpan, walked);
            if (--_counter > 0U) {
                continue;
            }
            _counter      = _n;
            outSpan[made] = inSpan[walked];
            releaseWaiting(outSpan, made);
            ++made;
        }

        std::ignore = inSpan.consume(walked);
        return made;
    }

    void releaseWaiting(OutputSpanLike auto& outSpan, std::size_t at) {
        const std::pmr::string sampleRate(gr::tag::SAMPLE_RATE.shortKey());
        for (property_map& tag : _relay._carry) {
            if (auto found = tag.find(sampleRate); found != tag.end()) {
                if (const float* rate = found->second.template get_if<float>(); rate != nullptr) {
                    _inputRate    = *rate;
                    found->second = static_cast<float>(static_cast<double>(*rate) / static_cast<double>(_n));
                } else if (const double* wide = found->second.template get_if<double>(); wide != nullptr) {
                    _inputRate    = static_cast<float>(*wide);
                    found->second = *wide / static_cast<double>(_n);
                }
            }
        }
        _relay.release(outSpan, at);
    }
};

GR_REGISTER_BLOCK(gr::blocks::basic::KeepMInN, [T], [ float, std::complex<float>, std::int16_t, std::uint8_t ])

template<typename T>
struct KeepMInN : Block<KeepMInN<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Keeps `m` consecutive items out of every `n`, starting `offset` items into each group.

The block takes a fixed slice from a repeating frame, such as a snapshot of a few thousand items in every period or a
fixed window of a repeating burst. It gives items of the input's type. The kept items are indices `offset` to
`offset + m - 1` of every group of `n`, and the first group begins at the first input. A group or a slice may span any
number of calls, and the block holds no buffer of a group. At the end of the stream the block has emitted the part of
the last slice it saw.

Each tag moves forward to the first item the block keeps at or after its own. For an item past the end of a slice, that
is the first kept item of the next group. A `sample_rate` tag leaves unchanged and states the spacing of the items
inside a slice. Tags that wait past the last kept item leave at the end-of-stream index.

`m` and `n` are at least one, and the block refuses an `offset + m` above `n`. A change of `m`, `n` or `offset` restarts
the group, and the first group after the change begins at the next input item. `reset()` restarts it in the same way and
leaves the waiting tags in place.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "m", Doc<"items kept per group">, Limits<1U, std::numeric_limits<gr::Size_t>::max()>> m      = 1U;
    Annotated<gr::Size_t, "n", Doc<"group size">, Limits<1U, std::numeric_limits<gr::Size_t>::max()>>           n      = 1U;
    Annotated<gr::Size_t, "offset", Doc<"index in the group of the first kept item">>                           offset = 0U;

    GR_MAKE_REFLECTABLE(KeepMInN, in, out, m, n, offset);

    gr::Size_t       _m      = m.value;      ///< the accepted `m`
    gr::Size_t       _n      = n.value;      ///< the accepted `n`
    gr::Size_t       _offset = offset.value; ///< the accepted `offset`
    gr::Size_t       _phase  = 0U;           ///< index within the current group of the next input item
    detail::TagRelay _relay{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const gr::Size_t nextM      = newSettings.contains("m") ? m.value : _m;
        const gr::Size_t nextN      = newSettings.contains("n") ? n.value : _n;
        const gr::Size_t nextOffset = newSettings.contains("offset") ? offset.value : _offset;
        if (std::uint64_t{nextOffset} + std::uint64_t{nextM} > std::uint64_t{nextN}) {
            throw gr::exception(std::format("offset ({}) + m ({}) must not exceed n ({})", nextOffset, nextM, nextN));
        }
        if (nextM != _m || nextN != _n || nextOffset != _offset) {
            _m      = nextM;
            _n      = nextN;
            _offset = nextOffset;
            _phase  = 0U;
        }
    }

    void start() { _relay.reset(); }

    void reset() { _phase = 0U; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        outSpan.publish(keep(inSpan, outSpan));
        return work::Status::OK;
    }

    /// Ends the stream as `KeepOneInN::processEpilogue` does.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (lifecycle::isShuttingDown(this->state())) {
            _relay.reset();
            outSpan.publish(0UZ);
            return work::Status::OK;
        }
        const std::size_t made = keep(inSpan, outSpan);
        _relay.collectPastEnd(in, inSpan.streamIndex + inSpan.size());
        _relay.release(outSpan, made);
        outSpan.publish(made);
        return work::Status::OK;
    }

private:
    [[nodiscard]] std::size_t keep(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t group = static_cast<std::size_t>(_n);
        const std::size_t first = static_cast<std::size_t>(_offset);
        const std::size_t last  = first + static_cast<std::size_t>(_m); // exclusive, and at most the group

        std::size_t walked = 0UZ;
        std::size_t made   = 0UZ;
        _relay.begin(inSpan);

        while (walked < inSpan.size()) {
            const std::size_t phase = static_cast<std::size_t>(_phase);
            if (phase >= first && phase < last) { // inside the slice, copy the run that fits both spans
                const std::size_t run = std::min({last - phase, inSpan.size() - walked, outSpan.size() - made});
                if (run == 0UZ) {
                    break; // output full, the rest of the slice waits for the next call
                }
                if (_relay.idle()) {
                    std::copy_n(inSpan.begin() + static_cast<std::ptrdiff_t>(walked), run, outSpan.begin() + static_cast<std::ptrdiff_t>(made));
                } else {
                    for (std::size_t i = 0UZ; i < run; ++i) {
                        _relay.collect(inSpan, walked + i);
                        outSpan[made + i] = inSpan[walked + i];
                        _relay.release(outSpan, made + i);
                    }
                }
                walked += run;
                made += run;
                _phase = static_cast<gr::Size_t>(phase + run);
            } else { // outside the slice, skip to its start or to the next group in one step
                const std::size_t until = phase < first ? first : group;
                const std::size_t run   = std::min(until - phase, inSpan.size() - walked);
                _relay.collect(inSpan, walked + run - 1UZ); // one call carries every tag of the skipped run forward
                walked += run;
                _phase = static_cast<gr::Size_t>(phase + run);
            }
            if (_phase >= static_cast<gr::Size_t>(group)) {
                _phase = 0U;
            }
        }

        std::ignore = inSpan.consume(walked);
        return made;
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_DECIMATORS_HPP
