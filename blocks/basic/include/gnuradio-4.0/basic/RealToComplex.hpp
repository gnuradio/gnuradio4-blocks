#ifndef GNURADIO_BASIC_REAL_TO_COMPLEX_HPP
#define GNURADIO_BASIC_REAL_TO_COMPLEX_HPP

#include <algorithm>
#include <complex>
#include <concepts>
#include <cstddef>
#include <expected>
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

/// @brief The validator of `transition`: true above 0 and below 0.5.
inline constexpr auto kRealToComplexTransition = [](float value) { return value > 0.f && value < 0.5f; };

} // namespace detail

template<typename T>
requires std::same_as<T, float>
struct RealToComplex : Block<RealToComplex<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Converts a real stream to a complex stream at half the rate, centered on a quarter of the input rate.

The block sits where a real stream enters a complex chain. It mixes the input down by `fs/4`, low-pass filters it at
`fs/4` and keeps every second sample, where `fs` is the input rate. The output runs at `fs/2`. Its zero frequency is
the input's `fs/4`, and a tone at `f` leaves at `f - fs/4`. `transition` is the filter's transition width as a fraction
of `fs`. The band covered runs from `transition fs/2` to `(1 - transition) fs/2` of the input. Outside the transition
the mirror image lies about 53 dB below the band. A real tone of amplitude A leaves at magnitude A/2, 6 dB below its
input level. The conversion keeps one of the two complex exponentials that make up the tone.

The filter is a Hamming-windowed halfband low-pass of `N` taps, odd and at most 4095, with `N` about `3.319/transition`.
The band loses half the transition at its low edge, the input's DC, and half at its top. The default of 0.02 takes 167
taps and loses 1% of `fs` at each edge, 480 Hz at 48 kHz. A narrow signal inside the band, an amateur-radio channel
among them, needs no narrower transition, and the filter stays short. Audible audio needs a narrower transition and more
taps: a band from 100 Hz at 48 kHz takes a `transition` of 0.0042 and 791 taps. The cap of 4095 taps is reached at a
`transition` of `3.319/4095`, about 8.1e-4. A `transition` below that gets 4095 taps and that wider transition. The band
covered then starts at half of it, and the mirror of a tone below that edge is attenuated less. At a `transition` of
2e-5, the mirror of a tone at `2e-5 fs` lies about 1.5 dB below the tone.

Input `2n` makes output `n`, and the conversion delays the signal by `D = (N-1)/2` input samples. A tag on input `i`
leaves on output `floor((i + D + 1)/2)` with all its keys, `i` and its output counted from the latest restart of the
conversion. A `gr:sample_rate` key leaves at half the value it arrived with. A `float` stays a `float` and a `double`
stays a `double`. A value of another type leaves as it arrived. The block publishes no rate of its own. A tag whose
output lies past the stream's last output leaves at the end-of-stream index, one past that output.

`transition` lies above 0 and below 0.5. A change rebuilds the filter and restarts the conversion at the sample where
the change applies. A tag held at the restart leaves on the restart's first output. `reset()` restarts the conversion
at the accepted `transition`.

A NaN or infinite input sample makes the outputs that read it non-finite, at most `(N+1)/2` of them. The outputs after
them are finite again.
)"">;

    PortIn<T>                in;
    PortOut<std::complex<T>> out;

    Annotated<float, "transition", Doc<"filter transition width as a fraction of the input rate">, Limits<0.f, 0.5f, detail::kRealToComplexTransition>, Visible> transition = 0.02f;

    GR_MAKE_REFLECTABLE(RealToComplex, in, out, transition);

    float                                             _transition = transition.value; ///< the accepted `transition`
    std::optional<gr::filter::RealToComplex>          _conversion{};
    std::vector<std::pair<std::size_t, property_map>> _held{};           ///< tags and their input indices, in input order
    std::size_t                                       _inOrigin  = 0UZ;  ///< the input index the conversion started at
    std::size_t                                       _outOrigin = 0UZ;  ///< the output index of its first output
    bool                                              _reorigin  = true; ///< the next call starts the conversion's count

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (!newSettings.contains("transition") || transition.value == _transition) {
            return;
        }
        std::expected<gr::filter::RealToComplex, gr::Error> rebuilt = gr::filter::RealToComplex::make(1.0, static_cast<double>(transition.value));
        if (!rebuilt) {
            throw gr::exception(rebuilt.error().message);
        }
        _conversion = std::move(*rebuilt);
        _reorigin   = true;
        _transition = transition.value;
    }

    void start() {
        _held.clear();
        conversion().reset();
        _reorigin = true;
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
                halveRate(tag);
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
            _conversion = gr::filter::RealToComplex::make(1.0, static_cast<double>(_transition)).value(); // an accepted transition always designs
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
            halveRate(moved);
            _held.emplace_back(tag.index, std::move(moved));
        }

        if (made > 0UZ) {
            const std::size_t base     = outSpan.streamIndex;
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

    /// Halves a `gr:sample_rate` key of @p tag, a `float` as a `float` and a `double` as a `double`. A value of another
    /// type stays as it is.
    static void halveRate(property_map& tag) {
        const std::pmr::string key(gr::tag::SAMPLE_RATE.key());
        if (auto found = tag.find(key); found != tag.end()) {
            if (const float* rate = found->second.get_if<float>(); rate != nullptr) {
                found->second = 0.5f * *rate;
            } else if (const double* wide = found->second.get_if<double>(); wide != nullptr) {
                found->second = 0.5 * *wide;
            }
        }
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_BASIC_REAL_TO_COMPLEX_HPP
