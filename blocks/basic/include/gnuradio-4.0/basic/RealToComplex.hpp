#ifndef GNURADIO_BASIC_REAL_TO_COMPLEX_HPP
#define GNURADIO_BASIC_REAL_TO_COMPLEX_HPP

#include <algorithm>
#include <complex>
#include <concepts>
#include <cstddef>
#include <expected>
#include <format>
#include <limits>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/algorithm/filter/RealToComplex.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::RealToComplex, [T], [float])

namespace detail {

/// @brief The validator of `sample_rate` and `transition_hz`: true above 0.
inline constexpr auto kRealToComplexPositive = [](float value) { return value > 0.f; };

} // namespace detail

template<typename T>
requires std::same_as<T, float>
struct RealToComplex : Block<RealToComplex<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Converts a real stream to a complex stream at half the rate, centered on a quarter of the input rate.

The block sits where a real stream enters a complex chain. It mixes the input down by `fs/4`, low-pass filters it at
`fs/4` and keeps every second sample, where `fs` is `sample_rate`. The output runs at `fs/2`. Its zero frequency is the
input's `fs/4`, and a tone at `f` leaves at `f - fs/4`. The band covered runs from `transition_hz/2` to
`fs/2 - transition_hz/2` of the input. Outside the transition the mirror image lies about 53 dB below the band. A real
tone of amplitude A leaves at magnitude A/2, 6 dB below its input level. The conversion keeps one of the two complex
exponentials that make up the tone.

The filter is a Hamming-windowed halfband low-pass of `N` taps, odd and at most 4095. A narrower `transition_hz` takes
more taps, down to a transition of `3.319 fs / 4095`, about 38.9 Hz at 48 kHz. A `transition_hz` below that gets 4095
taps and that wider transition. The band covered then starts at half of it, and the mirror of a tone below that edge is
attenuated less. At 48 kHz and a `transition_hz` of 1 Hz, the mirror of a 1 Hz tone lies about 1.6 dB below the tone.

Input `2n` makes output `n`, and the conversion delays the signal by `D = (N-1)/2` input samples. A tag on input `i`
leaves on output `floor((i + D + 1)/2)` with all its keys, `i` and its output counted from the latest restart of the
conversion. A tag's `sample_rate` key leaves stating the output rate, half the accepted `sample_rate`. A `double`
value stays a `double`, and any other value becomes a `float`. The first output of a run carries a `sample_rate` tag at
`fs/2`. A tag whose output lies past the stream's last output leaves at the end-of-stream index, one past that output.

`sample_rate` and `transition_hz` are positive, and the block refuses a `transition_hz` of half the `sample_rate` or
more. A change of either rebuilds the filter and restarts the conversion at the sample where the change applies. A tag
held at the restart leaves on the restart's first output. After a change of `sample_rate` that output carries the new
output rate, ahead of the tags waiting for it. `reset()` restarts the conversion at the accepted settings.

A NaN or infinite input sample makes the outputs that read it non-finite, at most `(N+1)/2` of them. The outputs after
them are finite again.
)"">;

    PortIn<T>                in;
    PortOut<std::complex<T>> out;

    Annotated<float, "sample_rate", Doc<"input sample rate in Hz">, Limits<0.f, std::numeric_limits<float>::max(), detail::kRealToComplexPositive>, Visible>         sample_rate   = 48'000.f;
    Annotated<float, "transition_hz", Doc<"filter transition width in Hz">, Limits<0.f, std::numeric_limits<float>::max(), detail::kRealToComplexPositive>, Visible> transition_hz = 200.f;

    GR_MAKE_REFLECTABLE(RealToComplex, in, out, sample_rate, transition_hz);

    float                                             _rate       = sample_rate.value;   ///< the accepted `sample_rate`
    float                                             _transition = transition_hz.value; ///< the accepted `transition_hz`
    std::optional<gr::filter::RealToComplex>          _conversion{};
    std::vector<std::pair<std::size_t, property_map>> _held{};           ///< tags and their input indices, in input order
    std::size_t                                       _inOrigin  = 0UZ;  ///< the input index the conversion started at
    std::size_t                                       _outOrigin = 0UZ;  ///< the output index of its first output
    bool                                              _reorigin  = true; ///< the next call starts the conversion's count
    bool                                              _restate   = true; ///< the next output carries the output rate

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const float nextRate       = newSettings.contains("sample_rate") ? sample_rate.value : _rate;
        const float nextTransition = newSettings.contains("transition_hz") ? transition_hz.value : _transition;
        if (!(nextTransition < 0.5f * nextRate)) {
            throw gr::exception(std::format("transition_hz ({}) must be below half of sample_rate ({})", nextTransition, nextRate));
        }
        if (nextRate == _rate && nextTransition == _transition) {
            return;
        }
        std::expected<gr::filter::RealToComplex, gr::Error> rebuilt = gr::filter::RealToComplex::make(static_cast<double>(nextRate), static_cast<double>(nextTransition));
        if (!rebuilt) {
            throw gr::exception(rebuilt.error().message);
        }
        _conversion = std::move(*rebuilt);
        _reorigin   = true;
        if (nextRate != _rate) {
            _restate = true;
            for (auto& [input, tag] : _held) {
                restateRate(tag, nextRate);
            }
        }
        _rate       = nextRate;
        _transition = nextTransition;
    }

    void start() {
        _held.clear();
        conversion().reset();
        _reorigin = true;
        _restate  = true;
    }

    void reset() {
        conversion().reset();
        _reorigin = true;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const auto [consumed, made] = convert(inSpan, outSpan);
        std::ignore                 = inSpan.consume(consumed);
        outSpan.publish(made);
        return work::Status::OK;
    }

    /// At the end of the stream the block converts what the trailing input holds and publishes every held tag at the
    /// end-of-stream index, with the tags on input it did not convert and past its last sample. At a stop request it
    /// publishes nothing and drops the held tags.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (lifecycle::isShuttingDown(this->state())) {
            _held.clear();
            outSpan.publish(0UZ);
            return work::Status::OK;
        }
        const auto [consumed, made] = convert(inSpan, outSpan);
        std::ignore                 = inSpan.consume(consumed);
        if (in.isConnected()) {
            const std::pmr::string endOfStream = static_cast<std::pmr::string>(gr::tag::END_OF_STREAM);
            const std::size_t      from        = inSpan.streamIndex + consumed;
            for (const Tag& ringTag : in.tagReader().get()) {
                if (ringTag.index < from) {
                    continue;
                }
                property_map tag(ringTag.map);
                tag.erase(endOfStream);
                restateRate(tag, _rate);
                if (!tag.empty()) {
                    _held.emplace_back(ringTag.index, std::move(tag));
                }
            }
        }
        for (const auto& [input, tag] : _held) {
            outSpan.publishTag(tag, made);
        }
        _held.clear();
        outSpan.publish(made);
        return work::Status::OK;
    }

private:
    [[nodiscard]] gr::filter::RealToComplex& conversion() {
        if (!_conversion) {
            _conversion = gr::filter::RealToComplex::make(static_cast<double>(_rate), static_cast<double>(_transition)).value(); // the accepted values always design
        }
        return *_conversion;
    }

    /// Converts what the output has room for, holds the tags of the converted input and publishes the held tags whose
    /// outputs this call makes. Returns the input consumed and the outputs made.
    [[nodiscard]] std::pair<std::size_t, std::size_t> convert(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        gr::filter::RealToComplex& filter = conversion();
        if (_reorigin) {
            _inOrigin  = inSpan.streamIndex;
            _outOrigin = outSpan.streamIndex;
            _reorigin  = false;
        }
        const std::size_t consumed = std::min(inSpan.size(), filter.inputsFor(outSpan.size()));
        const std::size_t made     = filter.process(std::span<const T>(inSpan.data(), consumed), std::span<std::complex<T>>(outSpan.data(), outSpan.size()));

        const std::size_t first = inSpan.streamIndex;
        for (const Tag& tag : inSpan.rawTags) {
            if (tag.index < first || tag.index >= first + consumed) {
                continue;
            }
            property_map moved(tag.map);
            restateRate(moved, _rate);
            _held.emplace_back(tag.index, std::move(moved));
        }

        if (made > 0UZ) {
            const std::size_t base = outSpan.streamIndex;
            if (_restate) {
                outSpan.publishTag(property_map{{std::pmr::string(gr::tag::SAMPLE_RATE.shortKey()), pmt::Value(0.5f * _rate)}}, 0UZ);
                _restate = false;
            }
            const std::size_t delay    = filter.groupDelay();
            std::size_t       released = 0UZ;
            for (; released < _held.size(); ++released) {
                const std::size_t output = outputOf(_held[released].first, delay);
                if (output >= base + made) {
                    break;
                }
                outSpan.publishTag(_held[released].second, std::max(output, base) - base);
            }
            _held.erase(_held.begin(), _held.begin() + static_cast<std::ptrdiff_t>(released));
        }
        return {consumed, made};
    }

    /// The output of the tag on input @p input under the conversion in force, whose delay is @p delay. An input ahead
    /// of the conversion's start maps to its first output. The map does not decrease with the input.
    [[nodiscard]] std::size_t outputOf(std::size_t input, std::size_t delay) const noexcept {
        if (input < _inOrigin) {
            return _outOrigin;
        }
        return _outOrigin + (input - _inOrigin + delay + 1UZ) / 2UZ;
    }

    /// Replaces a `sample_rate` key with half of @p inputRate, the output rate, keeping a `double` as a `double`.
    static void restateRate(property_map& tag, float inputRate) {
        const std::pmr::string key(gr::tag::SAMPLE_RATE.shortKey());
        if (auto found = tag.find(key); found != tag.end()) {
            if (found->second.get_if<double>() != nullptr) {
                found->second = 0.5 * static_cast<double>(inputRate);
            } else {
                found->second = 0.5f * inputRate;
            }
        }
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_BASIC_REAL_TO_COMPLEX_HPP
