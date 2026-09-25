#ifndef GNURADIO_RATIONAL_RESAMPLER_HPP
#define GNURADIO_RATIONAL_RESAMPLER_HPP

#include <algorithm>
#include <complex>
#include <concepts>
#include <cstdint>
#include <format>
#include <numeric>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/filter/PolyphaseResampler.hpp>

#include <gnuradio-4.0/filter/NamespaceCompatibility.hpp>
#include <gnuradio-4.0/filter/TagDelay.hpp>

namespace gr::blocks::filter {

GR_REGISTER_BLOCK(gr::blocks::filter::RationalResampler, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct RationalResampler : Block<RationalResampler<T>, Resampling<1UZ, 1UZ, false>> {
    using TParent     = Block<RationalResampler<T>, Resampling<1UZ, 1UZ, false>>;
    using Description = Doc<R""(
@brief Changes the sample rate by the exact ratio `interpolation / decimation`, images and aliases suppressed.

`L` outputs for every `M` inputs. One filter does both jobs: its stopband starts at the Nyquist frequency of whichever
rate is lower. With `taps` empty the prototype is designed at the interpolated rate `L*fs_in` from `rolloff`,
`attenuation_db` and `max_ripple_db`, `L/M` reduced by its gcd first; supplied taps are used as given, at the
unreduced ratio and with no gain applied.

Single-stage only: 48 kHz to 44.1 kHz is `147/160` after reduction and costs 5880 taps. Changing `interpolation`,
`decimation` or `taps` rebuilds the filter and resets the phase and the history, with a discontinuity at the seam; a
ratio change also moves the tag map's origin, everything else leaving the alignment alone. A forwarded `sample_rate`
tag is multiplied by `L/M`, so downstream reads the rate of the stream this block hands it.

Every forwarded tag that marks a time position, such as a trigger, a burst edge or a time stamp, moves by the filter's
delay `d`, in samples of the interpolated rate: a tag on input `i` leaves on output `round((i*L + d) / M)`, the sample
that carries the energy of input `i`. A tag that states a property of the stream, such as `sample_rate`, `signal_name`
or `context`, crosses unmoved, to the output of input `i` itself. A designed or other symmetric prototype of `N` taps
delays by `(N-1)/2`. An asymmetric supplied prototype moves its tags by the centroid of its energy, rounded to the whole
interpolated sample. A tag keeps the output it was given when it crossed, whatever rebuild follows, and a tag whose
output lies past the end of the stream is not published. )"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "interpolation", Doc<"L: output samples produced per M inputs">, Visible>                                                             interpolation = 1U;
    Annotated<gr::Size_t, "decimation", Doc<"M: input samples consumed per L outputs">, Visible>                                                                decimation    = 1U;
    Annotated<std::vector<float>, "taps", Doc<"prototype at the interpolated rate L*fs_in; empty designs one. Supplied taps carry their own gain, L included">> taps{};
    Annotated<float, "rolloff", Doc<"fraction of the surviving band spent on the transition; designed taps only">>                                              rolloff        = 0.2f;
    Annotated<float, "attenuation_db", Unit<"dB">, Doc<"stopband target of the designed filter">>                                                               attenuation_db = 60.f;
    Annotated<float, "max_ripple_db", Unit<"dB">, Doc<"passband ripple target of the designed filter">>                                                         max_ripple_db  = 0.1f;

    GR_MAKE_REFLECTABLE(RationalResampler, in, out, interpolation, decimation, taps, rolloff, attenuation_db, max_ripple_db);

    std::optional<gr::filter::PolyphaseResampler<T>>    _resampler;
    std::uint64_t                                       _interpolation = 1ULL; /// after reduction, where the taps were designed
    std::uint64_t                                       _decimation    = 1ULL;
    std::uint64_t                                       _twiceDelay    = 0ULL; /// the prototype's delay in half interpolated samples, `twiceTapDelay`
    std::uint64_t                                       _inOrigin      = 0ULL;
    std::uint64_t                                       _outOrigin     = 0ULL;
    bool                                                _reorigin      = false;
    std::vector<std::pair<std::uint64_t, property_map>> _pendingTags;

    /// The kernel is built from the members, and a batch that moves no value never calls back, so a block
    /// constructed at its declared defaults is born with a kernel and the chunk sizes its ratio states.
    explicit RationalResampler(property_map init = {}) : TParent(std::move(init)) { rebuild(); }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        static constexpr std::array kRebuildKeys{"interpolation", "decimation", "taps", "rolloff", "attenuation_db", "max_ripple_db"};

        const bool live        = _resampler.has_value();
        const bool rateChanged = newSettings.contains("interpolation") || newSettings.contains("decimation");
        if (!live || std::ranges::any_of(kRebuildKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            rebuild();
        }
        _reorigin = _reorigin || (live && rateChanged);
    }

    void start() {
        rebuild();
        _pendingTags.clear();
        _inOrigin  = 0ULL;
        _outOrigin = 0ULL;
        _reorigin  = false;
    }

    /// @brief The delay every forwarded tag moves by, in input samples. Generally not an integer.
    [[nodiscard]] double groupDelaySamples() const noexcept { return static_cast<double>(_twiceDelay) / (2.0 * static_cast<double>(_interpolation)); }

    void rebuild() {
        if (interpolation < 1U || decimation < 1U) {
            throw gr::exception(std::format("interpolation ({}) and decimation ({}) must both be at least one", interpolation.value, decimation.value));
        }

        std::vector<float> prototype = taps;
        std::uint64_t      l         = interpolation;
        std::uint64_t      m         = decimation;

        // supplied taps were designed against L*fs_in, and a reduced L would be a different interpolated rate: only a
        // designed prototype reduces the ratio, and supplied taps at a reducible ratio cost gcd(L, M) times the branches
        if (prototype.empty()) {
            const std::uint64_t g = std::gcd(l, m);
            l /= g;
            m /= g;
            const gr::filter::ResamplerDesign design = gr::filter::designResampler(static_cast<std::size_t>(l), static_cast<std::size_t>(m), static_cast<double>(rolloff.value), static_cast<double>(attenuation_db.value), static_cast<double>(max_ripple_db.value));
            if (!design.ok) {
                throw gr::exception(std::format("no filter under the tap cap meets {} dB stopband and {} dB ripple for {}/{}", attenuation_db.value, max_ripple_db.value, l, m));
            }
            prototype = design.taps;
        }

        _resampler.emplace(static_cast<std::size_t>(l), static_cast<std::size_t>(m), std::span<const float>(prototype));
        _interpolation = l;
        _decimation    = m;
        _twiceDelay    = detail::twiceTapDelay(std::span<const float>(prototype));

        this->input_chunk_size  = static_cast<gr::Size_t>(m);
        this->output_chunk_size = static_cast<gr::Size_t>(l);
    }

    /**
     * @brief Place every input tag that marks a time position on the output sample that carries its input sample's
     * energy, from the current phase origin.
     *
     * Input `i` maps to output `round((i*L + d) / M)`, `d` being the prototype's delay at the interpolated rate and a
     * half rounding up. The keys that state a property of the stream (`detail::kStreamPropertyKeys`) go to output
     * `round(i*L / M)` instead. A tag whose output is not in this call is held and published by the call that produces that
     * output. A tag is placed once, when it crosses, under the delay and the ratio in force then. A rebuild moves no
     * held tag, and a tag that crosses after one is never placed ahead of a tag held from before it. Tags therefore
     * leave in the order they arrived. A held tag whose output the stream ends before is never published.
     *
     * This replaces the framework's forwarding rather than adjusting it: the default publishes a tag at the output
     * index matching its input index, which is only right at a ratio of one. It is also where a ratio change takes its
     * new origin: the change is applied on the settings path, between calls, where neither absolute offset is knowable.
     * A tag already held keeps its absolute output offset, which the new origin leaves alone, and the `sample_rate`
     * value it was scaled by on arrival, which is the ratio that was in force when it crossed.
     */
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
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

        std::uint64_t latest = _pendingTags.empty() ? 0ULL : _pendingTags.back().first;
        gr::for_each_reader_span(
            [&latest, processedIn, this](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const auto& [relIndex, tagMap] : span.tags(processedIn)) {
                    if (relIndex < 0) {
                        // A tag from before this window is one this block has already placed. The
                        // framework's own forwarding clamps it to offset zero and republishes it,
                        // which would duplicate the tag this block already placed.
                        continue;
                    }
                    const std::uint64_t at = static_cast<std::uint64_t>(span.streamIndex) + static_cast<std::uint64_t>(relIndex);
                    property_map        forwarded(tagMap.get());
                    this->scaleSampleRateByChunkRatio(forwarded); // the ratio in force where the tag crossed, not where it is published
                    detail::holdTag(_pendingTags, latest, _outOrigin + detail::mapDelayedOffset(at - _inOrigin, _interpolation, _decimation, 0ULL), _outOrigin + detail::mapDelayedOffset(at - _inOrigin, _interpolation, _decimation, _twiceDelay), std::move(forwarded));
                }
            },
            inputSpans);

        if (_pendingTags.empty()) {
            return;
        }

        std::vector<std::pair<std::uint64_t, property_map>> deferred;
        gr::for_each_writer_span(
            [&](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                const std::uint64_t base = static_cast<std::uint64_t>(span.streamIndex);
                const std::uint64_t end  = base + span.size();

                const auto place = [&](const std::pair<std::uint64_t, property_map>& tag) {
                    if (tag.first >= end) { // its output is not in this call: hold it rather than move it
                        deferred.push_back(tag);
                        return;
                    }
                    span.publishTag(tag.second, static_cast<std::size_t>(tag.first > base ? tag.first - base : 0ULL));
                };
                for (const auto& tag : _pendingTags) {
                    place(tag);
                }
            },
            outputSpans);

        _pendingTags = std::move(deferred);
    }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) {
        // The framework hands whole chunks — M in, L out — so the phase is zero at every call
        // boundary and the two counts agree exactly. Anything else is a contract violation.
        const std::size_t made = _resampler->process(input, output);
        if (made != output.size()) {
            throw gr::exception(std::format("{} input samples yield {} outputs, not the {} reserved", input.size(), made, output.size()));
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_RATIONAL_RESAMPLER_HPP
