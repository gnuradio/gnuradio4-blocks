#ifndef GNURADIO_SYNC_SYMBOL_SYNC_HPP
#define GNURADIO_SYNC_SYMBOL_SYNC_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <cstdio>
#include <format>
#include <numbers>
#include <numeric>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/algorithm/sync/ControlLoop.hpp>
#include <gnuradio-4.0/algorithm/sync/MmseInterpolator.hpp>
#include <gnuradio-4.0/algorithm/sync/TimingErrorDetector.hpp>

#include <gnuradio-4.0/sync/LoopCommon.hpp>

namespace gr::blocks::sync {

/// @brief Stream tag: the optimum sampling instant of the next symbol, in input samples from the tagged sample, in `[-1, 1]`.
inline constexpr std::string_view kTimeEstKey = "time_est";
/// @brief Stream tag: a two-element real tensor `{offset, period}` — `time_est`, plus the average period in input samples per symbol.
inline constexpr std::string_view kClockEstKey = "clock_est";

/// @brief The polyphase partition this bank runs, under the name the timing machinery knows it by.
[[nodiscard]] inline std::vector<float> partitionBranchMajor(std::span<const float> prototype, std::size_t arms) { return gr::filter::fir::design::polyphasePartition(prototype, arms); }

namespace detail {

/**
 * @brief The timing loop: the shared kernel in `double`, its phase left to run.
 *
 * `double` and not `float`, which is the one place this block departs from its neighbors: at a period of 4 samples
 * and a `Bn*T` of 0.01 the integrator's dead band in `float` is 100 ppm, twenty times the steady-state period error
 * this block has to reach. The phase is not reduced, because the interpolator's position is not derived from it — it
 * is a diagnostic no port carries, and reducing it modulo a varying period would cost a `std::remainder` per symbol.
 */
using TimingLoop = gr::sync::ControlLoop<double, gr::sync::PhaseWrap::None>;

/// @brief The interpolator's tap count, or 0 for the polyphase matched-filter path.
[[nodiscard]] inline int interpolatorTapsFromName(std::string_view name) {
    if (name == "mmse8") {
        return 8;
    }
    if (name == "mmse6") {
        return 6;
    }
    if (name == "mmse12") {
        return 12;
    }
    if (name == "polyphase") {
        return 0;
    }
    throw gr::exception(std::format("interpolator must be one of mmse8, mmse6, mmse12, polyphase, got '{}'", name));
}

/// @brief A `time_est` or `clock_est` payload that passed validation, waiting for the position to reach it.
struct TimingTag {
    std::uint64_t offset  = 0ULL; /// absolute input offset of the tagged sample
    double        instant = 0.0;  /// input samples from `offset` to the requested symbol instant
    double        period  = 0.0;  /// input samples per symbol; 0 leaves the tracked period alone
};

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::sync::SymbolSync, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct SymbolSync : Block<SymbolSync<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Symbol timing recovery: estimates the transmitter's symbol clock from the signal and resamples onto it.

The receiver knows roughly how many samples there are per symbol, but not the transmitter's exact clock nor where the
symbol boundaries fall, and both drift. This block estimates the symbol instants from the signal itself and emits
`outputs_per_symbol` samples per symbol taken at those instants. The interpolator's position is advanced by the loop's
period estimate and not derived from its phase.

An AGC is a precondition, as for CostasLoop and FllBandEdge. `detector_gain` is per symbol period and the block
divides by `samples_per_symbol` itself; 0 takes the detector's own gain on a raised-cosine channel of the stated
`rolloff`, which is the right value whenever the transmitter shaped with a matching root-raised cosine. Gardner,
early-late and signal times slope have no gain at all at zero rolloff, and a graph that asks for one is refused
rather than run with a loop that cannot converge. `noise_bandwidth` is normalized to the symbol rate here, the loop
stepping once per symbol, where the carrier loops normalize theirs to the sample rate.

Gardner and early-late are blind and work before the carrier loop has locked; Mueller & Muller, the modified M&M and
zero crossing need decisions, hence a locked carrier and the `constellation` setting. `mmse8` is a clean interpolator
for `sps >= 2.5`; below that use `mmse12` or `polyphase`, which also matched-filters and so must not follow another
matched filter. Output sample `k` estimates the input at a continuous position that is not a fixed multiple of the
output index; `instant_period` and `average_period` carry the clock for a consumer that needs it.

The input port's minimum is held at the count that guarantees one more output, so at end of stream the block ends
as soon as fewer samples than that remain: the trailing samples are dropped, an interpolation without its future
samples being fabrication, and a finite graph terminates instead of waiting on input that cannot arrive.

A `time_est` or `clock_est` tag presets the position, and `clock_est` the period with it, which on a burst link is what
removes the acquisition the loop has too few symbols to do. A preset re-times a sample rather than removing one, so the
stream leaving the block carries one sample per symbol across a tag as it does anywhere else. `trigger` names which
producer's tags to honor and empty honors any; `trigger_presets` turns presetting off altogether. A payload out of
range, one carrying another producer's name, and one whose instant the position has already gone past are each ignored
and counted, and leave the block in the state it would have been in with no tag at all.
)"">;

    PortIn<T>                in;
    PortOut<T>               out;
    PortOut<float, Optional> error;
    PortOut<float, Optional> instant_period;
    PortOut<float, Optional> average_period;

    Annotated<double, "samples_per_symbol", Doc<"nominal input samples per symbol; must exceed 1 and need not be an integer">>                                                        samples_per_symbol = 4.0;
    Annotated<double, "max_deviation", Unit<"samples">, Doc<"period clamp about samples_per_symbol; 0 selects half of it">>                                                           max_deviation      = 0.0;
    Annotated<gr::Size_t, "outputs_per_symbol", Doc<"output samples per symbol; design-time, the internal clock's modulus">>                                                          outputs_per_symbol = 1U;
    Annotated<gr::sync::TimingDetector, "detector", Doc<"the timing error detector the loop runs on">>                                                                                detector           = gr::sync::TimingDetector::MuellerMuller;
    Annotated<std::string, "constellation", Doc<"bpsk, qpsk or pam4 (real streams); also the shared spelling psk with arity 2 or 4. The slicer the decision-directed detectors use">> constellation      = std::string(std::same_as<T, float> ? "bpsk" : "qpsk");
    Annotated<gr::Size_t, "arity", Unit<"symbols">, Doc<"M for the shared 'psk' spelling; the block's own names carry their size">>                                                   arity              = 4U;
    Annotated<double, "detector_gain", Doc<"Kted in error units per symbol period; 0 selects the raised-cosine gain of the detector in use">>                                         detector_gain      = 0.0;
    Annotated<double, "noise_bandwidth", Doc<"closed-loop noise bandwidth normalized to the symbol rate: Bn*Tsym">>                                                                   noise_bandwidth    = 0.01;
    Annotated<double, "damping", Doc<"loop damping; critically damped by default, under-damped timing loops are not useful">>                                                         damping            = 1.0;
    Annotated<std::string, "interpolator", Doc<"mmse8, mmse6, mmse12 or polyphase; polyphase also matched-filters">>                                                                  interpolator       = std::string("mmse8");
    Annotated<gr::Size_t, "polyphase_arms", Doc<"branches in the polyphase bank; rounded up to a power of two">>                                                                      polyphase_arms     = 128U;
    Annotated<std::vector<float>, "matched_filter_taps", Doc<"polyphase prototype at arms*sps; empty designs a root-raised cosine">>                                                  matched_filter_taps{};
    Annotated<double, "rolloff", Doc<"excess bandwidth of the designed prototype, and of the M&M closed-form gain">>                                                                  rolloff = 0.35;
    Annotated<std::string, "trigger", Doc<"the trigger_name a timing tag must carry for its payload to be honored; empty honors any timing tag">>                                     trigger{};
    Annotated<bool, "trigger_presets", Doc<"whether a matching timing tag presets at all; false ignores every time_est and clock_est and counts it">>                                 trigger_presets = true;

    GR_MAKE_REFLECTABLE(SymbolSync, in, out, error, instant_period, average_period, samples_per_symbol, max_deviation, outputs_per_symbol, detector, constellation, arity, detector_gain, noise_bandwidth, damping, interpolator, polyphase_arms, matched_filter_taps, rolloff, trigger, trigger_presets);

    detail::TimingLoop _loop{0.01, 1.0, 1.0, 1.0, 8.0};
    // Tap count follows `interpolator` (mmse6/8/12); the 128 phase steps and the 0.25 design band are
    // fixed here and are not settings. MmseInterpolator documents what each costs.
    gr::sync::MmseInterpolatorBank _bank{8, 128, 0.25, false};
    gr::sync::TimingDetector       _kind = gr::sync::TimingDetector::MuellerMuller;
    gr::sync::TimingDetectorTraits _traits{};

    std::vector<float> _branches{}; /// the polyphase bank, branch-major and reversed
    std::size_t        _arms         = 0UZ;
    std::size_t        _windowLength = 8UZ;  /// `L`: the input samples one interpolation reads
    std::uint64_t      _anchorDelay  = 3ULL; /// `L/2 - 1`: the integer part of the interpolated instant's lead over `base`
    bool               _polyphase    = false;
    bool               _qpsk         = false;
    bool               _pam4         = false;
    bool               _configured   = false;

    static constexpr std::uint8_t kDetectorStep = 1U;
    static constexpr std::uint8_t kOutputStep   = 2U;

    /// @brief Which of the interpolator's instants a timing tag names, and so which side of a step its snap sits on.
    enum class Preset : std::uint8_t { ThisInstant, TheNextInstant };

    int                       _interpolationsPerSymbol = 1;
    double                    _cycleLength             = 1.0; /// the same count as a double, for the per-symbol divide
    int                       _counter                 = 0;
    std::vector<std::uint8_t> _schedule{kDetectorStep | kOutputStep}; /// which of the three clocks each phase of the cycle carries

    std::uint64_t _base       = 0ULL; /// absolute input offset of the interpolation window
    std::uint64_t _baseLead   = 0ULL; /// the base's lead over the point the last call consumed to
    double        _mu         = 0.0;  /// fraction of an input sample, in `[0, 1)`
    bool          _positioned = false;

    double _instantPeriod       = 4.0;
    double _interpolationPeriod = 4.0;
    float  _error               = 0.f;

    std::array<T, 3UZ> _y{};        /// interpolated samples at the last three symbol instants
    std::array<T, 3UZ> _a{};        /// their decisions, when the detector needs them
    T                  _midpoint{}; /// the half-symbol sample the two-input detectors read
    T                  _slope{};    /// the derivative per symbol at the current instant

    std::vector<std::pair<std::uint64_t, property_map>> _pendingTags{};
    std::vector<detail::TimingTag>                      _timingTags{};
    std::uint64_t                                       _tagsSeenThrough = 0ULL;
    std::uint64_t                                       _ignoredTags     = 0ULL;

    /// The loop, the interpolator and the schedule are derived from the members, and a batch that moves no value
    /// never calls back, so a block constructed at its declared defaults is born configured. `newSettings` is a
    /// cost hint below; `_configured` is what guarantees the first build.
    explicit SymbolSync(property_map init = {}) : Block<SymbolSync<T>, NoTagPropagation>(std::move(init)) { configure(property_map{}); }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) { configure(newSettings); }

    void configure(const property_map& newSettings) {
        if (!(samples_per_symbol > 1.0) || !std::isfinite(samples_per_symbol)) {
            throw gr::exception(std::format("samples_per_symbol must exceed 1 — below two samples per symbol no detector has a usable S-curve — got {}", samples_per_symbol.value));
        }
        if (outputs_per_symbol < 1U) {
            throw gr::exception(std::format("outputs_per_symbol must be at least one, got {}", outputs_per_symbol.value));
        }
        if (!(max_deviation >= 0.0) || !std::isfinite(max_deviation)) {
            throw gr::exception(std::format("max_deviation must be finite and not negative; 0 selects half of samples_per_symbol, got {}", max_deviation.value));
        }
        const double deviation = max_deviation > 0.0 ? max_deviation.value : 0.5 * samples_per_symbol.value;
        if (deviation >= samples_per_symbol) {
            throw gr::exception(std::format("max_deviation ({}) must stay below samples_per_symbol ({}) — the clamp has to keep the period positive", deviation, samples_per_symbol.value));
        }
        if (!(rolloff >= 0.0) || !(rolloff <= 1.0)) {
            throw gr::exception(std::format("rolloff must lie in [0, 1], got {}", rolloff.value));
        }
        if (!(noise_bandwidth > 0.0) || !std::isfinite(noise_bandwidth)) {
            throw gr::exception(std::format("noise_bandwidth is a normalized noise bandwidth and must be positive and finite, got {}", noise_bandwidth.value));
        }
        if (!(damping > 0.0) || !std::isfinite(damping)) {
            throw gr::exception(std::format("damping must be positive and finite, got {}", damping.value));
        }
        if (!(detector_gain >= 0.0) || !std::isfinite(detector_gain)) {
            throw gr::exception(std::format("detector_gain is a measured S-curve slope and must be positive and finite; 0 selects the raised-cosine gain of the detector in use, got {}", detector_gain.value));
        }

        const gr::sync::TimingDetector       kind   = detector;
        const int                            nTaps  = detail::interpolatorTapsFromName(interpolator);
        const double                         gain   = effectiveDetectorGain(kind);
        const gr::sync::TimingDetectorTraits traits = gr::sync::traitsOf(kind);

        if (!(gain > 0.0)) {
            throw gr::exception(std::format("detector '{}' has no gain at rolloff {}: a pulse with no excess bandwidth carries no timing line for a detector that reads no decisions", detector.value, rolloff.value));
        }

        const std::string_view shape = constellationShape();
        if (shape.empty()) {
            throw gr::exception(std::format("constellation must be bpsk, qpsk or pam4, or the shared spelling psk with arity 2 or 4, got '{}' with arity {}", constellation.value, arity.value));
        }
        if constexpr (std::same_as<T, float>) {
            if (shape == "qpsk") {
                throw gr::exception("constellation qpsk needs a complex stream; a real one carries bpsk or pam4");
            }
        } else {
            if (shape == "pam4") {
                throw gr::exception("constellation pam4 needs a real stream; a complex one carries bpsk or qpsk");
            }
        }

        _kind   = kind;
        _traits = traits;
        _qpsk   = shape == "qpsk";
        _pam4   = shape == "pam4";

        _loop.setFrequencyLimits(samples_per_symbol - deviation, samples_per_symbol + deviation);
        _loop.setDamping(damping);
        _loop.setNoiseBandwidth(noise_bandwidth);
        _loop.setDetectorGain(gr::sync::detectorGainPerSample(gain, samples_per_symbol));

        static constexpr std::array kRestartKeys{"samples_per_symbol", "outputs_per_symbol", "detector", "constellation", "arity", "interpolator", "polyphase_arms", "matched_filter_taps", "rolloff"};
        if (!_configured || std::ranges::any_of(kRestartKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            buildInterpolator(nTaps);
            restart();
        }
        _configured = true;

        // The input port's minimum is the loop's own production condition at the position the block currently
        // holds: the base's lead over the point the last call consumed to, plus the furthest the detector reads
        // past the base, plus the window. A port minimum is also a refusal -- the scheduler withholds a shorter
        // span -- so any slack in it is input the loop could have used, and at the end of a stream refused input
        // is gone. How many samples are left over there would then be set by where the call boundaries fell,
        // making the symbol count a property of the scheduler's chunking rather than of the recording. Every call
        // rewrites the value from the position it leaves; this one carries a settings change into the same form.
        in.min_samples = static_cast<gr::Size_t>(_baseLead + readAhead(_traits.needsLookahead, _polyphase) + _windowLength);
    }

    void reset() {
        restart();
        _pendingTags.clear();
        _timingTags.clear();
        _tagsSeenThrough = 0ULL;
        _ignoredTags     = 0ULL;
        _base            = 0ULL;
        _baseLead        = 0ULL;
        _positioned      = false;
    }

    /// @brief The decision shape a constellation name selects, empty for a name this slicer cannot carry.
    ///
    /// Both vocabularies are accepted: this block's own `bpsk`, `qpsk` and `pam4`, and the shared spelling `psk`
    /// with `arity`, which the constellation-carrying blocks use. The detectors slice a binary or quadrature
    /// decision, so `psk` is carried at arity 2 and 4 and refused above that rather than sliced wrongly.
    [[nodiscard]] std::string_view constellationShape() const noexcept {
        if (constellation == "psk") {
            if (arity == 2U) {
                return "bpsk";
            }
            return arity == 4U ? "qpsk" : "";
        }
        if (constellation == "bpsk" || constellation == "qpsk" || constellation == "pam4") {
            return constellation.value;
        }
        return "";
    }

    /// @brief The average of `E[|Re{a}|]` and `E[|Im{a}|]` for the sliced constellation at unit power.
    [[nodiscard]] double meanAxisMagnitude() const noexcept {
        const std::string_view shape = constellationShape();
        if (shape == "qpsk") {
            return std::numbers::sqrt2 / 2.0;
        }
        if (shape == "pam4") {
            return 0.4472135954999579; // 1/sqrt(5): the mean of {1, 3}/sqrt(5)
        }
        return 0.5; // one axis carries the whole symbol and the kernel still halves
    }

    /// @brief `Kted` per symbol: what `detector_gain` says, or the raised-cosine value for the detector in use.
    [[nodiscard]] double effectiveDetectorGain(gr::sync::TimingDetector kind) const {
        if (detector_gain > 0.0) {
            return detector_gain.value;
        }
        switch (kind) {
        case gr::sync::TimingDetector::MuellerMuller: return gr::sync::muellerMullerGain(rolloff);
        case gr::sync::TimingDetector::ModifiedMuellerMuller: return gr::sync::modifiedMuellerMullerGain(rolloff);
        case gr::sync::TimingDetector::ZeroCrossing: return gr::sync::zeroCrossingGain(rolloff);
        case gr::sync::TimingDetector::Gardner: return gr::sync::gardnerGain(rolloff);
        case gr::sync::TimingDetector::EarlyLate: return gr::sync::earlyLateGain(rolloff);
        case gr::sync::TimingDetector::SignalTimesSlopeMl: return gr::sync::signalTimesSlopeGain(rolloff);
        case gr::sync::TimingDetector::SignumTimesSlopeMl: return gr::sync::signumTimesSlopeGain(rolloff, meanAxisMagnitude());
        }
        return 0.0;
    }

    [[nodiscard]] std::uint64_t             ignoredTagPayloads() const noexcept { return _ignoredTags; }
    [[nodiscard]] const detail::TimingLoop& loop() const noexcept { return _loop; }
    [[nodiscard]] int                       interpolationsPerSymbol() const noexcept { return _interpolationsPerSymbol; }
    [[nodiscard]] std::size_t               windowLength() const noexcept { return _windowLength; }
    [[nodiscard]] std::span<const float>    branches() const noexcept { return _branches; }

    /// @brief The detector and the interpolator are chosen once per call, so the loop below has neither in it.
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& errorSpan, OutputSpanLike auto& instantSpan, OutputSpanLike auto& averageSpan) {
        using gr::sync::TimingDetector;
        switch (_kind) {
        case TimingDetector::MuellerMuller: return dispatch<TimingDetector::MuellerMuller>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
        case TimingDetector::ModifiedMuellerMuller: return dispatch<TimingDetector::ModifiedMuellerMuller>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
        case TimingDetector::ZeroCrossing: return dispatch<TimingDetector::ZeroCrossing>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
        case TimingDetector::Gardner: return dispatch<TimingDetector::Gardner>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
        case TimingDetector::EarlyLate: return dispatch<TimingDetector::EarlyLate>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
        case TimingDetector::SignalTimesSlopeMl: return dispatch<TimingDetector::SignalTimesSlopeMl>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
        default: return dispatch<TimingDetector::SignumTimesSlopeMl>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
        }
    }

private:
    template<gr::sync::TimingDetector D>
    [[nodiscard]] work::Status dispatch(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& errorSpan, OutputSpanLike auto& instantSpan, OutputSpanLike auto& averageSpan) {
        if constexpr (!gr::sync::traitsOf(D).needsDerivative) {
            if (_polyphase) {
                return sweep<D, true>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
            }
        }
        return sweep<D, false>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
    }

    /// @brief The furthest the detector reads past the base for the next interpolation, in input samples.
    ///
    /// It is `_mu` plus, where the detector reads a late sample, ONE INTERPOLATION period -- early-late
    /// interpolates twice a symbol, so its late sample stands half a symbol on and not a whole one -- rounded up,
    /// with one more branch step on the polyphase path. The loop's production condition and the input port
    /// minimum are the same expression, so both are read from here.
    [[nodiscard]] std::uint64_t readAhead(bool needsLookahead, bool polyphase) const noexcept {
        const double reach = needsLookahead ? _mu + _interpolationPeriod : _mu;
        return static_cast<std::uint64_t>(reach) + (reach > 0.0 ? 1ULL : 0ULL) + (polyphase ? 1ULL : 0ULL);
    }

    /// @brief A 64-bit sample offset narrowed to an index; a template, so the cast stands where `std::size_t` is 32
    /// bits and is not a useless cast where the two are the same 64-bit type.
    template<std::integral To, std::integral From>
    [[nodiscard]] static constexpr To narrowIndex(From value) noexcept {
        return static_cast<To>(value);
    }

    template<gr::sync::TimingDetector D, bool Polyphase>
    [[nodiscard]] work::Status sweep(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& errorSpan, OutputSpanLike auto& instantSpan, OutputSpanLike auto& averageSpan) {
        constexpr gr::sync::TimingDetectorTraits kTraits = gr::sync::traitsOf(D);

        const std::uint64_t windowStart = static_cast<std::uint64_t>(inSpan.streamIndex);
        if (!_positioned) {
            _base            = windowStart;
            _tagsSeenThrough = windowStart;
            _positioned      = true;
        }

        std::size_t room = outSpan.size();
        room             = detail::syncCount(room, errorSpan);
        room             = detail::syncCount(room, instantSpan);
        room             = detail::syncCount(room, averageSpan);

        const bool          wantError   = errorSpan.isConnected;
        const bool          wantInstant = instantSpan.isConnected;
        const bool          wantAverage = averageSpan.isConnected;
        const std::uint64_t inputEnd    = windowStart + inSpan.size();

        collectTags(inSpan, inputEnd);

        const T* const x    = inSpan.data();
        std::size_t    made = 0UZ;
        while (made < room) {
            std::ignore = applyTimingTags(Preset::ThisInstant);
            if (_base + readAhead(kTraits.needsLookahead, Polyphase) + _windowLength - 1UZ >= inputEnd) {
                break;
            }

            _counter                  = _counter + 1 == _interpolationsPerSymbol ? 0 : _counter + 1;
            const T* const     window = x + (_base - windowStart);
            const T            y      = interpolate<Polyphase>(window, _mu);
            const std::uint8_t due    = _schedule[static_cast<std::size_t>(_counter)];

            if (_counter == 0) {
                _y[2] = _y[1];
                _y[1] = _y[0];
                _y[0] = y;
                if constexpr (kTraits.needsDecisions) {
                    _a[2] = _a[1];
                    _a[1] = _a[0];
                    _a[0] = decide(y);
                }
                if constexpr (kTraits.needsDerivative) {
                    // The taps differentiate per input sample; a detector's Kted is per symbol, so the period scales it.
                    _slope = scale(_bank.differentiate(window, quantize(_mu, _bank.steps())), static_cast<float>(_instantPeriod));
                }
                const T late = kTraits.needsLookahead ? lookahead<Polyphase>(x, windowStart) : T{};

                _error               = errorOf<D>(late);
                const double instant = _loop.step(static_cast<double>(_error));
                _instantPeriod       = instant > 0.0 ? instant : _loop.frequency();
                _interpolationPeriod = _instantPeriod / _cycleLength;
            } else if ((due & kDetectorStep) != 0U) {
                _midpoint = y;
            }

            if ((due & kOutputStep) != 0U) {
                outSpan[made] = y;
                if (wantError) {
                    errorSpan[made] = _error;
                }
                if (wantInstant) {
                    instantSpan[made] = static_cast<float>(_instantPeriod);
                }
                if (wantAverage) {
                    averageSpan[made] = static_cast<float>(_loop.frequency());
                }
                releaseTags(_base + _anchorDelay, made, outSpan, errorSpan, instantSpan, averageSpan);
                ++made;
            }

            if (!applyTimingTags(Preset::TheNextInstant)) {
                // `advanced` is positive, so truncation is `floor` and is one instruction; `std::floor` is a dozen.
                const double        advanced = _mu + _interpolationPeriod;
                const std::uint64_t whole    = static_cast<std::uint64_t>(advanced);
                _mu                          = advanced - static_cast<double>(whole);
                _base += whole;
            }
        }

        // Half a window of input is held back: a timing tag first seen at the very end of this window asks the
        // interpolator to sit `L/2 - 1` samples before the sample it names, and that history has to still be there.
        const std::uint64_t reach     = _windowLength / 2UZ;
        const std::uint64_t keepFrom  = inputEnd > reach ? inputEnd - reach : 0ULL;
        const std::uint64_t consumeTo = std::max(windowStart, std::min(_base, keepFrom));

        std::ignore = inSpan.consume(narrowIndex<std::size_t>(consumeTo - windowStart));

        // What the next call needs, measured from where this consume leaves the reader: the loop's own production
        // condition and nothing more, so the block is offered a span exactly when one more output is computable.
        _baseLead      = _base - consumeTo;
        in.min_samples = static_cast<gr::Size_t>(_baseLead + readAhead(kTraits.needsLookahead, Polyphase) + _windowLength);

        outSpan.publish(made);
        errorSpan.publish(wantError ? made : 0UZ);
        instantSpan.publish(wantInstant ? made : 0UZ);
        averageSpan.publish(wantAverage ? made : 0UZ);

        if (made == 0UZ && consumeTo == windowStart) {
            return room == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

    void restart() {
        _interpolationsPerSymbol         = std::lcm(_traits.inputsPerSymbol, static_cast<int>(outputs_per_symbol.value));
        const std::size_t detectorStride = static_cast<std::size_t>(_interpolationsPerSymbol / _traits.inputsPerSymbol);
        const std::size_t outputStride   = static_cast<std::size_t>(_interpolationsPerSymbol / static_cast<int>(outputs_per_symbol.value));

        // The three clocks are a modulo of an integer counter, but a modulo by
        // a value the compiler cannot see is a hardware divide, three of them per interpolation, so the whole cycle is
        // tabulated once per settings change instead.
        _schedule.assign(static_cast<std::size_t>(_interpolationsPerSymbol), 0U);
        for (std::size_t phase = 0UZ; phase < _schedule.size(); ++phase) {
            _schedule[phase] = static_cast<std::uint8_t>((phase % detectorStride == 0UZ ? kDetectorStep : 0U) | (phase % outputStride == 0UZ ? kOutputStep : 0U));
        }
        _counter = _interpolationsPerSymbol - 1;

        _cycleLength = static_cast<double>(_interpolationsPerSymbol);
        _loop.reset(0.0, samples_per_symbol);
        _instantPeriod       = samples_per_symbol;
        _interpolationPeriod = _instantPeriod / _cycleLength;
        _mu                  = 0.0;
        _error               = 0.f;
        _y.fill(T{});
        _a.fill(T{});
        _midpoint = T{};
        _slope    = T{};
    }

    void buildInterpolator(int nTaps) {
        _polyphase = nTaps == 0;
        if (!_polyphase) {
            _bank = gr::sync::MmseInterpolatorBank(nTaps, 128, 0.25, _traits.needsDerivative);
            _branches.clear();
            _arms         = 0UZ;
            _windowLength = _bank.size();
            _anchorDelay  = _bank.delay();
            return;
        }
        if (_traits.needsDerivative) {
            throw gr::exception("the maximum-likelihood detectors need the interpolating differentiator, which the polyphase path does not carry — select an mmse interpolator");
        }

        std::size_t arms = 1UZ;
        while (arms < static_cast<std::size_t>(polyphase_arms.value)) {
            arms <<= 1U;
        }

        std::vector<float> prototype = matched_filter_taps;
        if (prototype.empty()) {
            // The design returns an odd tap count, so ask for one short of the bank and let the partition pad the
            // last branch: the prototype stays centered to within one interpolated sample, 1/arms of an input sample.
            // Eleven symbols of span is a conventional root-raised-cosine truncation: long enough that the
            // matched-filter residue is well inside the timing error the loop already carries at any usable rolloff.
            std::size_t branch = static_cast<std::size_t>(std::ceil(11.0 * samples_per_symbol));
            branch += branch % 2UZ;
            prototype = gr::filter::fir::design::rootRaisedCosine(branch * arms - 1UZ, static_cast<double>(arms) * samples_per_symbol, rolloff, static_cast<double>(arms));
        }

        _arms         = arms;
        _branches     = partitionBranchMajor(prototype, arms);
        _windowLength = _branches.size() / arms;
        _anchorDelay  = _windowLength / 2UZ - 1UZ;
    }

    /**
     * @brief The row or branch a fraction in `[0, 1)` selects, without a library rounding call.
     *
     * `MmseInterpolatorBank::row` states the rule and reaches `std::nearbyint` for it, which the compiler will not
     * inline at any `-march`, since it has to respect the dynamic rounding mode, so it is a libm call on every
     * interpolation. Truncating `x + 0.5` is the same selection for every input except an exact half, where this
     * rounds up and `nearbyint` rounds to even: one row of a 129-row bank, 1/256 of a sample.
     */
    [[nodiscard]] static std::size_t quantize(double fraction, std::size_t steps) noexcept {
        const std::size_t index = static_cast<std::size_t>(fraction * static_cast<double>(steps) + 0.5);
        return index < steps ? index : steps;
    }

    template<bool Polyphase>
    [[nodiscard]] T interpolate(const T* window, double fraction) const noexcept {
        if constexpr (!Polyphase) {
            return _bank.interpolate(window, quantize(fraction, _bank.steps()));
        } else {
            std::size_t branch = quantize(fraction, _arms);
            if (branch >= _arms) {
                branch = 0UZ;
                ++window;
            }
            const float* const taps = _branches.data() + branch * _windowLength;
            T                  sum{};
            for (std::size_t k = 0UZ; k < _windowLength; ++k) {
                sum += taps[k] * window[k];
            }
            return sum;
        }
    }

    /// @brief Early-late's sample after the symbol instant, one interpolation ahead at the period this symbol still holds.
    template<bool Polyphase>
    [[nodiscard]] T lookahead(const T* x, std::uint64_t windowStart) const noexcept {
        const double        advanced = _mu + _interpolationPeriod;
        const std::uint64_t whole    = static_cast<std::uint64_t>(advanced);
        return interpolate<Polyphase>(x + (_base + whole - windowStart), advanced - static_cast<double>(whole));
    }

    [[nodiscard]] static T scale(T value, float factor) noexcept { return value * factor; }

    /// @brief The unit-power hard decision: `+/-1` on a real stream (`pam4`: the nearest of `{+/-1, +/-3}/sqrt(5)`),
    /// the nearest of four at `(+/-1 +/- j)/sqrt(2)` on a complex one.
    [[nodiscard]] T decide(T y) const noexcept {
        if constexpr (std::same_as<T, float>) {
            if (_pam4) {
                constexpr float kInner = 0.4472136f; // 1/sqrt(5): unit average power over the four levels
                const float     level  = (y > 2.f * kInner || y < -2.f * kInner) ? 3.f * kInner : kInner;
                return y > 0.f ? level : -level;
            }
            return y > 0.f ? 1.f : -1.f;
        } else {
            const float re = y.real() > 0.f ? 1.f : -1.f;
            if (!_qpsk) {
                return T{re, 0.f};
            }
            const float half = std::numbers::sqrt2_v<float> / 2.f;
            return T{half * re, half * (y.imag() > 0.f ? 1.f : -1.f)};
        }
    }

    /// @brief The detector's formula, resolved at compile time — no switch and no function pointer inside the loop.
    template<gr::sync::TimingDetector D>
    [[nodiscard]] float errorOf(T late) const noexcept {
        using gr::sync::TimingDetector;
        if constexpr (D == TimingDetector::MuellerMuller) {
            return gr::sync::muellerMullerError(_y[0], _y[1], _a[0], _a[1]);
        } else if constexpr (D == TimingDetector::ModifiedMuellerMuller) {
            return gr::sync::modifiedMuellerMullerError(_y[0], _y[1], _y[2], _a[0], _a[1], _a[2]);
        } else if constexpr (D == TimingDetector::ZeroCrossing) {
            return gr::sync::zeroCrossingError(_midpoint, _a[0], _a[1]);
        } else if constexpr (D == TimingDetector::Gardner) {
            return gr::sync::gardnerError(_y[0], _y[1], _midpoint);
        } else if constexpr (D == TimingDetector::EarlyLate) {
            return gr::sync::earlyLateError(_y[0], _midpoint, late);
        } else if constexpr (D == TimingDetector::SignalTimesSlopeMl) {
            return gr::sync::signalTimesSlopeError(_y[0], _slope);
        } else {
            return gr::sync::signumTimesSlopeError(_y[0], _slope);
        }
    }

    [[nodiscard]] static std::optional<double> finiteNumber(const pmt::Value& value) noexcept {
        if (const std::optional<float> real = detail::finiteReal(value); real.has_value()) {
            return static_cast<double>(*real);
        }
        return std::nullopt;
    }

    /**
     * @brief Whether a timing tag's `trigger_name` is the one this block was told to act on.
     *
     * An empty `trigger` honors a timing tag whatever name it carries or does not, which is the behavior a stream with
     * one timing producer on it wants. A non-empty one is a filter, so a graph carrying two producers — a preamble
     * estimator and a correlator, say — can send each block's presets to the block that asked for them.
     */
    [[nodiscard]] bool namedForThisBlock(const property_map& map) const noexcept {
        if (trigger.value.empty()) {
            return true;
        }
        const auto named = map.find(property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()));
        if (named == map.end()) {
            return false;
        }
        const auto* label = named->second.get_if<std::pmr::string>();
        return label != nullptr && std::string_view(label->data(), label->size()) == std::string_view(trigger.value);
    }

    /// @brief Read `clock_est`'s `{offset, period}` out of a two-element real tensor.
    [[nodiscard]] static std::optional<std::pair<double, double>> clockPayload(const pmt::Value& value) noexcept {
        if (const auto* wide = value.get_if<gr::Tensor<double>>(); wide != nullptr && wide->size() == 2UZ) {
            return std::pair{(*wide)[0], (*wide)[1]};
        }
        if (const auto* narrow = value.get_if<gr::Tensor<float>>(); narrow != nullptr && narrow->size() == 2UZ) {
            return std::pair{static_cast<double>((*narrow)[0]), static_cast<double>((*narrow)[1])};
        }
        return std::nullopt;
    }

    /**
     * @brief Take every tag in the window exactly once, whether or not this call reaches it.
     *
     * A tag is collected as soon as it is visible and held until an output anchor passes it, so placement depends on
     * the anchor sequence alone and not on where the scheduler put its call boundaries. `_tagsSeenThrough` is what
     * keeps a re-presented tag — anything past the point this call consumes to — from being collected twice.
     */
    void collectTags(const auto& inSpan, std::uint64_t inputEnd) {
        const property_map::key_type timeKey{kTimeEstKey};
        const property_map::key_type clockKey{kClockEstKey};

        for (const gr::Tag& tag : inSpan.rawTags) {
            const std::uint64_t at = static_cast<std::uint64_t>(tag.index);
            if (at < _tagsSeenThrough || at >= inputEnd) {
                continue;
            }

            const auto time  = tag.map.find(timeKey);
            const auto clock = tag.map.find(clockKey);
            if (time == tag.map.end() && clock == tag.map.end()) {
                _pendingTags.emplace_back(at, tag.map);
                continue;
            }

            // clock_est wins where both appear, and a payload that fails any part of its validation takes the whole
            // tag with it: half of a repositioning leaves a state the caller cannot reproduce.
            std::optional<double> instant;
            double                period = 0.0;
            if (clock != tag.map.end()) {
                if (const auto payload = clockPayload(clock->second); payload.has_value()) {
                    instant = payload->first;
                    period  = payload->second;
                }
            } else {
                instant = finiteNumber(time->second);
            }

            const bool usable = trigger_presets && namedForThisBlock(tag.map) && instant.has_value() && std::abs(*instant) <= 1.0 && (period == 0.0 || (period >= _loop.minFrequency() && period <= _loop.maxFrequency()));
            if (usable) {
                _timingTags.push_back({at, *instant, period});
            } else {
                ++_ignoredTags;
            }

            property_map rest = tag.map;
            rest.erase(timeKey);
            rest.erase(clockKey);
            if (!rest.empty()) {
                _pendingTags.emplace_back(at, std::move(rest));
            }
        }
        _tagsSeenThrough = std::max(_tagsSeenThrough, inputEnd);
    }

    /**
     * @brief Snap the position onto a measured instant, at the step whose own instant that measurement names.
     *
     * The position only ever moves forward, so a target the anchor has already gone past cannot be reached at all and
     * is ignored and counted rather than applied with the advance clamped at zero — which would force `mu` to zero and
     * clear the history at a phase nobody asked for, destroying a position the loop already holds to honor a request
     * it cannot serve. Which step carries the snap is what keeps the count of samples intact. A target within half an
     * interpolation period of the position names the instant this step is about to take, and the snap moves that
     * instant onto it before the step runs. A target further ahead names the instant *after* this one, and the snap
     * replaces the step's own advance instead, so the sample this step carries is still taken at the position the
     * loop holds. A snap taken at a step whose instant it does not name swallows that step's sample, which on a burst
     * link deletes a symbol from the stream at the very boundary the tag marks.
     *
     * @return whether the position now sits on a target, so that the caller advances it no further.
     */
    [[nodiscard]] bool applyTimingTags(Preset names) {
        while (!_timingTags.empty()) {
            const detail::TimingTag& wanted   = _timingTags.front();
            const double             offset   = wanted.offset >= _base ? static_cast<double>(wanted.offset - _base) : -static_cast<double>(_base - wanted.offset);
            const double             target   = offset + wanted.instant;
            const double             position = static_cast<double>(_anchorDelay) + _mu;
            const double             horizon  = names == Preset::ThisInstant ? 0.5 * _interpolationPeriod : _interpolationPeriod;
            if (position + horizon <= target) {
                return false;
            }

            const double advance = target - static_cast<double>(_anchorDelay);
            if (advance < 0.0) {
                ++_ignoredTags;
                _timingTags.erase(_timingTags.begin());
                continue;
            }

            const std::uint64_t whole = static_cast<std::uint64_t>(advance);
            _base += whole;
            _mu = advance - static_cast<double>(whole);

            // The loop's integrator is its period estimate, so presetting the period is resetting the filter; the
            // phase goes with it, a diagnostic the position never reads, so that it restarts with the burst.
            if (wanted.period > 0.0) {
                _loop.reset(0.0, wanted.period);
            } else {
                _loop.setPhase(0.0);
            }
            _counter             = _interpolationsPerSymbol - 1;
            _instantPeriod       = _loop.frequency();
            _interpolationPeriod = _instantPeriod / _cycleLength;
            _error               = 0.f;
            _y.fill(T{});
            _a.fill(T{});
            _midpoint = T{};
            _slope    = T{};

            _timingTags.erase(_timingTags.begin());
            return true;
        }
        return false;
    }

    /// @brief Publish every held tag whose input offset the anchor has now reached, in input order, on every wired port.
    void releaseTags(std::uint64_t anchor, std::size_t at, auto& outSpan, auto& errorSpan, auto& instantSpan, auto& averageSpan) {
        std::size_t due = 0UZ;
        while (due < _pendingTags.size() && _pendingTags[due].first <= anchor) {
            ++due;
        }
        if (due == 0UZ) {
            return;
        }
        for (std::size_t i = 0UZ; i < due; ++i) {
            const property_map& forwarded = _pendingTags[i].second;
            outSpan.publishTag(forwarded, at);
            errorSpan.publishTag(forwarded, at);
            instantSpan.publishTag(forwarded, at);
            averageSpan.publishTag(forwarded, at);
        }
        _pendingTags.erase(_pendingTags.begin(), _pendingTags.begin() + static_cast<std::ptrdiff_t>(due));
    }
};

} // namespace gr::blocks::sync

#endif // GNURADIO_SYNC_SYMBOL_SYNC_HPP
