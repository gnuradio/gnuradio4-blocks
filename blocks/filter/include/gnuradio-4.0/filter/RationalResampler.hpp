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
ratio change also moves the tag map's origin, everything else leaving the alignment alone. The group delay is stated,
not compensated. A forwarded `sample_rate` tag is multiplied by `L/M`, so downstream reads the rate of the stream this
block hands it.
)"">;

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
    std::size_t                                         _designLength  = 1UZ;
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

    /// @brief The filter's delay in input samples — stated, not compensated. Generally not an integer.
    [[nodiscard]] double groupDelaySamples() const noexcept { return static_cast<double>(_designLength - 1UZ) / (2.0 * static_cast<double>(_interpolation)); }

    void rebuild() {
        if (interpolation < 1U || decimation < 1U) {
            throw gr::exception(std::format("interpolation ({}) and decimation ({}) must both be at least one", interpolation.value, decimation.value));
        }

        std::vector<float> prototype = taps;
        std::uint64_t      l         = interpolation;
        std::uint64_t      m         = decimation;

        if (prototype.empty()) {
            const std::uint64_t g = std::gcd(l, m);
            l /= g;
            m /= g;
            const gr::filter::ResamplerDesign design = gr::filter::designResampler(static_cast<std::size_t>(l), static_cast<std::size_t>(m), static_cast<double>(rolloff.value), static_cast<double>(attenuation_db.value), static_cast<double>(max_ripple_db.value));
            if (!design.ok) {
                throw gr::exception(std::format("no filter under the tap cap meets {} dB stopband and {} dB ripple for {}/{}", attenuation_db.value, max_ripple_db.value, l, m));
            }
            prototype     = design.taps;
            _designLength = static_cast<std::size_t>(design.designLength);
        } else {
            // the taps were designed against L*fs_in, and a reduced L would be a different interpolated rate, so the
            // ratio is left unreduced: supplied taps at a reducible ratio cost gcd(L, M) times the branches
            _designLength = prototype.size();
        }

        _resampler.emplace(static_cast<std::size_t>(l), static_cast<std::size_t>(m), std::span<const float>(prototype));
        _interpolation = l;
        _decimation    = m;

        this->input_chunk_size  = static_cast<gr::Size_t>(m);
        this->output_chunk_size = static_cast<gr::Size_t>(l);
    }

    /**
     * @brief Place every input tag at the output offset the rate change puts it at, from the current phase origin.
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

        std::vector<std::pair<std::uint64_t, property_map>> arriving;
        gr::for_each_reader_span(
            [&arriving, processedIn, this](auto& span) {
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
                    arriving.emplace_back(_outOrigin + gr::filter::mapResampledOffset(at - _inOrigin, _interpolation, _decimation), std::move(forwarded));
                }
            },
            inputSpans);

        if (arriving.empty() && _pendingTags.empty()) {
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
                for (const auto& tag : arriving) {
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
