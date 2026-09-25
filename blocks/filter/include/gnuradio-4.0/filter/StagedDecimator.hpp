#ifndef GNURADIO_STAGED_DECIMATOR_HPP
#define GNURADIO_STAGED_DECIMATOR_HPP

#include <algorithm>
#include <array>
#include <complex>
#include <concepts>
#include <cstdint>
#include <optional>
#include <print>
#include <span>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/filter/HalfbandCascade.hpp>
#include <gnuradio-4.0/algorithm/filter/PolyphaseResampler.hpp>

#include <gnuradio-4.0/filter/NamespaceCompatibility.hpp>
#include <gnuradio-4.0/filter/TagDelay.hpp>

namespace gr::blocks::filter {

GR_REGISTER_BLOCK(gr::blocks::filter::StagedDecimator, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct StagedDecimator : Block<StagedDecimator<T>, Resampling<1UZ, 1UZ, false>> {
    using TParent     = Block<StagedDecimator<T>, Resampling<1UZ, 1UZ, false>>;
    using Description = Doc<R""(
@brief Decimates by `D` as a ladder of stages rather than as one filter, at a fraction of the multiplies.

`D = 2^k * m` runs as `k` halving stages followed by the odd part, every stage given the same passband in hertz rather
than the same fraction of its own rate, so the length migrates to the last stage. `factor_odd` splits the odd part
into primes and `max_odd_factor` bounds the closing stage. Measured: about 4.5x a single filter at `D = 64` and
`passband_width = 0.90`.

`passband_width` is a fraction of the output rate, not a frequency, so the block needs no `sample_rate` setting and
originates no rate tag; a forwarded `sample_rate` tag is divided by `D`, so downstream reads the rate of the stream
this block hands it. `D = 1` designs no taps and is a bit-exact pass-through. Changing `decimation` or any design parameter
rebuilds the ladder, with a discontinuity at the seam; only a `decimation` change moves the tag map's origin, a
redesign at the same rate leaving the alignment alone.

Every forwarded tag that marks a time position, such as a trigger, a burst edge or a time stamp, moves by the ladder's
delay `d`: a tag on input `i` leaves on output `round((i + d) / D)`, the sample that carries the energy of input `i`. A
tag that states a property of the stream, such as `sample_rate`, `signal_name` or `context`, crosses unmoved, to the
output of input `i` itself. Over real samples `d` is `groupDelaySamples()`. Over complex samples the halving stages run
as a halfband cascade, which samples each halving's output one sample of that stage's input rate early; there `d` is
`groupDelaySamples()` less `2^k - 1` for `k` halvings. A tag keeps the output it was given when it crossed, whatever
rebuild follows, and a tag whose output lies past the end of the stream is not published. )"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "decimation", Doc<"D: input samples consumed per output; at least one">, Visible>                                             decimation     = 1U;
    Annotated<float, "passband_width", Doc<"W as a fraction of the output rate, in (0, 1); not of the input rate, and not a frequency">, Visible>       passband_width = 0.8f;
    Annotated<float, "attenuation_db", Unit<"dB">, Doc<"per-stage stopband target">>                                                                    attenuation_db = 85.f;
    Annotated<float, "ripple_db", Unit<"dB">, Doc<"ripple budget for the whole cascade, divided evenly among the stages">>                              ripple_db      = 0.05f;
    Annotated<bool, "factor_odd", Doc<"factor the odd part into primes, largest first; false uses one polyphase stage which gains nothing at D = 25.">> factor_odd     = true;
    Annotated<gr::Size_t, "max_odd_factor", Doc<"largest odd factor placed in one stage; a larger one is placed anyway and warned about">>              max_odd_factor = 25U;

    GR_MAKE_REFLECTABLE(StagedDecimator, in, out, decimation, passband_width, attenuation_db, ripple_db, factor_odd, max_odd_factor);

    static constexpr bool kHalfbandKernel = std::same_as<T, std::complex<float>>;

    gr::filter::StagedDecimatorDesign                   _design;
    std::optional<gr::filter::HalfbandCascade>          _cascade;
    std::vector<gr::filter::PolyphaseResampler<T>>      _tail;
    std::uint64_t                                       _decimation = 1ULL;
    std::uint64_t                                       _inOrigin   = 0ULL;
    std::uint64_t                                       _outOrigin  = 0ULL;
    std::uint64_t                                       _twiceDelay = 0ULL; /// the path's delay in half input samples, which the tags move by
    bool                                                _live       = false;
    bool                                                _reorigin   = false;
    std::vector<T>                                      _front;
    std::vector<T>                                      _back;
    std::vector<std::pair<std::uint64_t, property_map>> _pendingTags;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        static constexpr std::array kRebuildKeys{"decimation", "passband_width", "attenuation_db", "ripple_db", "factor_odd", "max_odd_factor"};

        const bool rateChanged = _live && newSettings.contains("decimation");
        if (!_live || std::ranges::any_of(kRebuildKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            rebuild();
        }
        _reorigin = _reorigin || rateChanged;
    }

    void start() {
        rebuild();
        _pendingTags.clear();
        _inOrigin  = 0ULL;
        _outOrigin = 0ULL;
        _reorigin  = false;
    }

    [[nodiscard]] std::size_t stages() const noexcept { return _design.stages.size(); }

    [[nodiscard]] std::span<const float> stageTaps(std::size_t stage) const noexcept { return std::span<const float>(_design.stages[stage].taps); }

    [[nodiscard]] std::size_t stageDecimation(std::size_t stage) const noexcept { return _design.stages[stage].decimation; }

    /// @brief Multiplies per input sample summed over the ladder, counting a halfband stage at only the taps the cascade evaluates.
    [[nodiscard]] double macsPerInput() const noexcept { return _design.macsPerInput; }

    /// @brief The stopband of the least attenuated stage, which bounds the cascade's alias rejection.
    [[nodiscard]] double worstStopbandDb() const noexcept { return _design.worstStopbandDb; }

    [[nodiscard]] double rippleSumDb() const noexcept { return _design.rippleSumDb; }

    /// @brief `sum over stages of ((N_i - 1)/2) * p_{i-1}` input samples, the design's delay and the delay of the real-sample path.
    [[nodiscard]] std::uint64_t groupDelaySamples() const noexcept { return _design.groupDelaySamples; }

    void rebuild() {
        if (decimation < 1U) {
            throw gr::exception(std::format("decimation ({}) must be at least one", decimation.value));
        }
        if (!(passband_width > 0.f) || !(passband_width < 1.f)) {
            throw gr::exception(std::format("passband_width ({}) is a fraction of the output rate and must lie in (0, 1)", passband_width.value));
        }

        _design = gr::filter::designStagedDecimator(static_cast<std::size_t>(decimation), static_cast<double>(passband_width), static_cast<double>(attenuation_db), static_cast<double>(ripple_db), factor_odd, static_cast<std::size_t>(max_odd_factor));
        if (!_design.ok) {
            throw gr::exception(std::format("no ladder under the tap cap meets {} dB stopband and {} dB total ripple at a decimation of {} and a passband width of {}", attenuation_db.value, ripple_db.value, decimation.value, passband_width.value));
        }
        if (_design.oversizedOddFactor != 0UZ) {
            std::println(stderr, "gr::blocks::filter::StagedDecimator: a decimation of {} leaves an odd factor of {} above max_odd_factor ({}) — placed anyway, in one long closing stage", decimation.value, _design.oversizedOddFactor, max_odd_factor.value);
        }

        _cascade.reset();
        _tail.clear();

        if constexpr (kHalfbandKernel) {
            std::vector<std::vector<float>> halvings;
            for (const gr::filter::DecimatorStage& stage : _design.stages) {
                if (stage.halfband) {
                    halvings.push_back(stage.taps);
                }
            }
            if (!halvings.empty()) {
                _cascade.emplace(std::span<const std::vector<float>>(halvings));
                _cascade->primeWithSilence();
            }
        }

        for (const gr::filter::DecimatorStage& stage : _design.stages) {
            if (kHalfbandKernel && stage.halfband) {
                continue;
            }
            _tail.emplace_back(1UZ, stage.decimation, std::span<const float>(stage.taps));
        }

        std::uint64_t delay = _design.groupDelaySamples;
        if constexpr (kHalfbandKernel) {
            for (const gr::filter::DecimatorStage& stage : _design.stages) {
                if (stage.halfband) {
                    delay -= stage.stride; // the cascade samples a halving's output one sample of its input rate early
                }
            }
        }

        _decimation = decimation;
        _twiceDelay = 2ULL * delay;
        _live       = true;
        _front.clear();
        _back.clear();

        this->input_chunk_size  = static_cast<gr::Size_t>(decimation);
        this->output_chunk_size = 1U;
    }

    /**
     * @brief Place every input tag that marks a time position on the output sample that carries its input sample's
     * energy, from the current phase origin.
     *
     * Input `i` maps to output `round((i + d) / D)`, `d` being the path's delay and a half rounding up, from the total
     * decimation and never stage by stage. The keys that state a property of the stream (`detail::kStreamPropertyKeys`)
     * go to output `round(i / D)` instead. A tag whose output is not in this call is held and published by the call
     * that produces that output. A tag is placed once, when it crosses, under the delay and the decimation in force
     * then. A rebuild moves no held tag, and a tag that crosses after one is never placed ahead of a tag held from
     * before it. Tags therefore leave in the order they arrived. A held tag whose output the stream ends before is
     * never published.
     *
     * This replaces the framework's forwarding rather than adjusting it, and it is also where a `decimation` change takes
     * its new origin: the change is applied on the settings path, between calls, where neither absolute offset is
     * knowable. A `sample_rate` tag is divided by the total decimation as it is taken in, so a tag that crossed before a
     * rate change carries the ratio that was in force when it crossed.
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
                    if (relIndex < 0) { // a tag from before this window is one this block has already placed
                        continue;
                    }
                    const std::uint64_t at = static_cast<std::uint64_t>(span.streamIndex) + static_cast<std::uint64_t>(relIndex);
                    property_map        forwarded(tagMap.get());
                    this->scaleSampleRateByChunkRatio(forwarded); // the rate in force where the tag crossed, not where it is published
                    detail::holdTag(_pendingTags, latest, _outOrigin + detail::mapDelayedOffset(at - _inOrigin, 1ULL, _decimation, 0ULL), _outOrigin + detail::mapDelayedOffset(at - _inOrigin, 1ULL, _decimation, _twiceDelay), std::move(forwarded));
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
        std::span<const T> stream = input;

        if constexpr (kHalfbandKernel) {
            if (_cascade) {
                _front.clear();
                _cascade->push(input, _front);
                stream = std::span<const T>(_front);
            }
        }

        for (std::size_t i = 0UZ; i < _tail.size(); ++i) {
            const std::size_t made = _tail[i].outputsFor(stream.size());
            if (i + 1UZ == _tail.size() && made == output.size()) {
                std::ignore = _tail[i].process(stream, output);
                return work::Status::OK;
            }
            _back.resize(made);
            std::ignore = _tail[i].process(stream, std::span<T>(_back));
            _front.swap(_back);
            stream = std::span<const T>(_front);
        }

        if (stream.size() != output.size()) {
            throw gr::exception(std::format("{} input samples yield {} outputs, not the {} reserved", input.size(), stream.size(), output.size()));
        }
        std::ranges::copy(stream, output.begin());
        return work::Status::OK;
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_STAGED_DECIMATOR_HPP
