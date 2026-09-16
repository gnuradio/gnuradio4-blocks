#ifndef GNURADIO_BLOCKS_ARBITRARY_RESAMPLER_HPP
#define GNURADIO_BLOCKS_ARBITRARY_RESAMPLER_HPP

#include <algorithm>
#include <array>
#include <complex>
#include <concepts>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/filter/ArbitraryResampler.hpp>

#include <gnuradio-4.0/filter/NamespaceCompatibility.hpp>

namespace gr::blocks::filter {

GR_REGISTER_BLOCK(gr::blocks::filter::ArbitraryRateResampler, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct ArbitraryRateResampler : Block<ArbitraryRateResampler<T>> {
    using TKernel     = gr::filter::ArbitraryResampler<T>;
    using Description = Doc<R""(
@brief Changes the sample rate by a real number, realized exactly as a stated rational.

A caller whose ratio is a fraction of small terms wants `RationalResampler`, at half the arithmetic and no
interpolation error. What this block buys is a bounded filter: the prototype is designed once for a bank of `L` arms
and its length per arm never follows the ratio's arithmetic.

The rate is a `double` and the phase is fixed-point, so the realized ratio is a rational the block reports through
`realizedRate()`, within `1.2e-10` relative of the request. Lowering `rate` past what the prototype was cut for
rebuilds it; `min_rate` states that floor once and keeps the design search off the settings path. The ports are
`Async`, an arbitrary `double` ratio not being expressible as the framework's constant `L:M`. `rate = 1` is not a
pass-through.

A forwarded `sample_rate` tag is multiplied by `rate`, so downstream reads the rate of the stream this block hands it.
)"">;

    PortIn<T, Async>  in;
    PortOut<T, Async> out;

    Annotated<double, "rate", Doc<"output samples per input sample; must be positive. A ratio of small terms belongs in RationalResampler">>                    rate                = 1.0;
    Annotated<double, "min_rate", Doc<"lowest rate the prototype is cut for; 0 tracks the rate in force when it was last built">>                               min_rate            = 0.0;
    Annotated<gr::Size_t, "bank_size", Doc<"L: arms in the polyphase bank; 0 sizes it from attenuation_db and the interpolation order">>                        bank_size           = 0U;
    Annotated<gr::Size_t, "interpolation_order", Doc<"q: 0 nearest arm, 1 linear, 3 cubic Lagrange">>                                                           interpolation_order = 1U;
    Annotated<std::vector<float>, "taps", Doc<"prototype at the interpolated rate L*fs_in; empty designs one. Supplied taps carry their own gain, L included">> taps{};
    Annotated<float, "rolloff", Unit<"fraction">, Doc<"share of the surviving band spent on the transition; designed taps only">>                               rolloff        = 0.2f;
    Annotated<float, "attenuation_db", Unit<"dB">, Doc<"stopband target of the designed prototype, and what the bank is sized against">>                        attenuation_db = 60.f;
    Annotated<float, "max_ripple_db", Unit<"dB">, Doc<"passband ripple target of the designed prototype">>                                                      max_ripple_db  = 0.1f;

    GR_MAKE_REFLECTABLE(ArbitraryRateResampler, in, out, rate, min_rate, bank_size, interpolation_order, taps, rolloff, attenuation_db, max_ripple_db);

    std::optional<TKernel>                              _resampler;
    std::size_t                                         _bankSize    = 1UZ;
    double                                              _designedFor = 1.0; /// the `min(1, r)` the prototype was cut for
    std::uint64_t                                       _inOrigin    = 0ULL;
    std::uint64_t                                       _outOrigin   = 0ULL;
    std::uint64_t                                       _stepOrigin  = 0ULL;
    std::int64_t                                        _phaseOrigin = 0LL;
    bool                                                _reorigin    = false;
    std::vector<std::pair<std::uint64_t, property_map>> _pendingTags;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        static constexpr std::array kRebuildKeys{"bank_size", "interpolation_order", "taps", "rolloff", "attenuation_db", "max_ripple_db"};
        const bool                  live    = _resampler.has_value();
        const bool                  rebuilt = !live || std::ranges::any_of(kRebuildKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); });
        if (rebuilt) {
            rebuild();
        } else if (newSettings.contains("rate") || newSettings.contains("min_rate")) {
            retune();
        } else {
            return;
        }
        _reorigin = _reorigin || live;
    }

    void start() {
        rebuild();
        _pendingTags.clear();
        _inOrigin  = 0ULL;
        _outOrigin = 0ULL;
        _reorigin  = false;
    }

    /// @brief `L*2^32/step`, the rational the block actually runs at. Exact as a `double` while `step` is below `2^53`.
    [[nodiscard]] double realizedRate() const noexcept { return _resampler->realizedRate(); }
    /// @brief `B`, the taps one arm holds — the wrap term included, so a dot product is this long.
    [[nodiscard]] std::size_t tapsPerArm() const noexcept { return _resampler->tapsPerArm(); }
    [[nodiscard]] std::size_t bankSize() const noexcept { return _bankSize; }
    /// @brief `(N-1)/(2L)` input samples, stated and not compensated.
    [[nodiscard]] double         groupDelaySamples() const noexcept { return _resampler->groupDelaySamples(); }
    [[nodiscard]] std::size_t    outputsFor(std::size_t nInput) const noexcept { return _resampler->outputsFor(nInput); }
    [[nodiscard]] std::size_t    inputsFor(std::size_t nOutput) const noexcept { return _resampler->inputsFor(nOutput); }
    [[nodiscard]] const TKernel& kernel() const noexcept { return *_resampler; }

    void rebuild() {
        if (!(rate > 0.0)) {
            throw gr::exception(std::format("rate ({}) must be positive", rate.value));
        }
        const int order = static_cast<int>(interpolation_order);
        if (order != 0 && order != 1 && order != 3) {
            throw gr::exception(std::format("interpolation_order ({}) is 0, 1 or 3", interpolation_order.value));
        }

        _bankSize = bank_size > 0U ? static_cast<std::size_t>(bank_size) : gr::filter::arbitraryBankSize(static_cast<double>(attenuation_db.value), static_cast<double>(rolloff.value), order);

        std::vector<float> prototype = taps;
        if (prototype.empty()) {
            _designedFor                             = designTarget();
            const gr::filter::ResamplerDesign design = gr::filter::designArbitraryResampler(_bankSize, _designedFor, static_cast<double>(rolloff.value), static_cast<double>(attenuation_db.value), static_cast<double>(max_ripple_db.value));
            if (!design.ok) {
                throw gr::exception(std::format("no prototype under the tap cap meets {} dB stopband and {} dB ripple for a bank of {} at rates down to {}", attenuation_db.value, max_ripple_db.value, _bankSize, _designedFor));
            }
            prototype = design.taps;
        } else {
            _designedFor = 0.0; // supplied taps are never redesigned; the caller owns the rate they are valid for
        }

        _resampler.emplace(rate, _bankSize, order, std::span<const float>(prototype));
        _stepOrigin  = _resampler->step();
        _phaseOrigin = _resampler->phase();
    }

    /**
     * @brief Change the rate, keeping the position, and redesign only where the new rate would under-filter.
     *
     * The design law depends on the rate only through `min(1, r)`, so a rate that rises inside the designed band leaves
     * the prototype narrower than it needs to be, which costs bandwidth and never aliases; only a fall below what it
     * was cut for folds anything in.
     */
    void retune() {
        if (!(rate > 0.0)) {
            throw gr::exception(std::format("rate ({}) must be positive", rate.value));
        }
        const double wanted = designTarget();
        if (taps.value.empty() && wanted < _designedFor) {
            const gr::filter::ResamplerDesign design = gr::filter::designArbitraryResampler(_bankSize, wanted, static_cast<double>(rolloff.value), static_cast<double>(attenuation_db.value), static_cast<double>(max_ripple_db.value));
            if (!design.ok) {
                throw gr::exception(std::format("no prototype under the tap cap meets {} dB stopband for a bank of {} at rates down to {}", attenuation_db.value, _bankSize, wanted));
            }
            _resampler->setTaps(design.taps); // keeps the phase and as much of the window as the new B holds
            _designedFor = wanted;
        }
        _resampler->setRate(rate);
    }

    /**
     * @brief Take the origin of a new rate regime, and keep the framework's own forwarding off this block's tags.
     *
     * Defining this at all is what replaces the default forwarder, whose output index matches its input index and is
     * right only at a rate of one. The tags themselves are mapped in `processBulk`; what belongs here is the origin,
     * because this is the one hook the framework calls after a settings change has been applied and before a sample
     * of the call is processed, and the one that sees every port's span. A rate change is applied on the settings
     * path, between calls, where neither absolute offset is knowable.
     */
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t /*processedIn*/) {
        if (!_reorigin) {
            return;
        }
        gr::for_each_reader_span([this](auto& span) { _inOrigin = static_cast<std::uint64_t>(span.streamIndex); }, inputSpans);
        gr::for_each_writer_span([this](auto& span) { _outOrigin = static_cast<std::uint64_t>(span.streamIndex); }, outputSpans);
        _stepOrigin  = _resampler->step();
        _phaseOrigin = _resampler->phase();
        _reorigin    = false;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t room = outSpan.size();
        std::size_t       nIn  = inSpan.size();
        std::size_t       made = _resampler->outputsFor(nIn);
        if (made > room) {
            // the largest input count whose outputs still fit: `inputsFor` is the smallest count reaching a given
            // output, so one short of what reaches `room + 1` is exactly it
            nIn  = _resampler->inputsFor(room + 1UZ) - 1UZ;
            made = _resampler->outputsFor(nIn);
        }

        mapTags(inSpan, nIn);
        std::ignore = _resampler->process(std::span<const T>(inSpan.data(), nIn), std::span<T>(outSpan.data(), made));
        releaseTags(outSpan, made);

        std::ignore = inSpan.consume(nIn);
        inSpan.consumeTags(nIn);
        outSpan.publish(made);

        if (made == 0UZ && nIn == 0UZ) {
            // above unity one input sample can carry two outputs, so a room of one is not short of input: it is short
            // of room, and reporting that is what stops the scheduler re-offering the same window
            return _resampler->outputsFor(inSpan.size()) > 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    /// @brief Rewrite a forwarded rate by `rate`. The framework's chunk-ratio scaling covers only a declared `L:M`,
    /// which an arbitrary real ratio is not, so the same rule is applied here from the setting itself.
    void scaleSampleRate(property_map& parameters) const noexcept {
        const auto it = parameters.find(gr::tag::SAMPLE_RATE.shortKey());
        if (it == parameters.end()) {
            return;
        }
        if (const float* inputRate = it->second.get_if<float>(); inputRate != nullptr) {
            it->second = static_cast<float>(rate * static_cast<double>(*inputRate));
        }
    }

    /// @brief The `min(1, r)` the prototype has to cover: the stated floor where there is one, and the current rate otherwise.
    [[nodiscard]] double designTarget() const noexcept {
        const double floorRate = min_rate > 0.0 ? std::min(1.0, static_cast<double>(min_rate)) : 1.0;
        return std::min(floorRate, std::min(1.0, static_cast<double>(rate)));
    }

    /**
     * @brief Place the tags of the samples this call consumes at their output offsets, under the regime in force now.
     *
     * A tag's mapping is committed here, where its sample is consumed, and not where the tag first becomes visible.
     * An `Async` port is presented every sample it holds and the block consumes a prefix of them, so a tag past that
     * prefix is presented again next call, and `rate` may have changed in between: mapping it on sight would fix its
     * offset, and scale its `sample_rate`, under a regime that no longer governs its sample. Taking only the tags of
     * `[first, last)` is also what maps each one exactly once, `consumeTags` having retired everything below `first`,
     * and an index carrying more than one tag still maps all of them.
     */
    void mapTags(InputSpanLike auto& inSpan, std::size_t nIn) {
        if (nIn == 0UZ || !inSpan.isConnected) {
            return;
        }
        const std::uint64_t first = static_cast<std::uint64_t>(inSpan.streamIndex);
        const std::uint64_t last  = first + static_cast<std::uint64_t>(nIn);
        for (const gr::Tag& tag : inSpan.rawTags) {
            const std::uint64_t at = static_cast<std::uint64_t>(tag.index);
            if (at < first || at >= last) {
                continue;
            }
            property_map forwarded(tag.map);
            scaleSampleRate(forwarded); // the rate that consumes the sample, which is the rate the tag describes
            _pendingTags.emplace_back(_outOrigin + gr::filter::mapArbitraryOffset(at - _inOrigin, _bankSize, _stepOrigin, _phaseOrigin), std::move(forwarded));
        }
    }

    /// @brief Publish the tags whose output this call produced, and hold the rest. An unconnected port drops them at
    /// `publishTag` rather than here, so the held list cannot grow without bound behind an unconnected port.
    void releaseTags(OutputSpanLike auto& outSpan, std::size_t made) {
        if (_pendingTags.empty()) {
            return;
        }
        const std::uint64_t base = static_cast<std::uint64_t>(outSpan.streamIndex);
        const std::uint64_t end  = base + made;

        std::vector<std::pair<std::uint64_t, property_map>> deferred;
        for (auto& tag : _pendingTags) {
            if (tag.first >= end) { // its output is not in this call: hold it rather than move it
                deferred.push_back(std::move(tag));
                continue;
            }
            outSpan.publishTag(tag.second, static_cast<std::size_t>(tag.first > base ? tag.first - base : 0ULL));
        }
        _pendingTags = std::move(deferred);
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_BLOCKS_ARBITRARY_RESAMPLER_HPP
