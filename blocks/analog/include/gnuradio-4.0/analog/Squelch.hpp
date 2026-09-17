#ifndef GNURADIO_SQUELCH_HPP
#define GNURADIO_SQUELCH_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/analog/detail/Averager.hpp>

namespace gr::blocks::analog {

namespace detail {

inline constexpr const char* kSquelchStartOfBurst = "squelch_sob";
inline constexpr const char* kSquelchEndOfBurst   = "squelch_eob";

enum class GateState : std::uint8_t { Muted, Attack, Unmuted, Decay };

/// @brief A burst marker: the key carries the whole meaning, so the value is none.
[[nodiscard]] inline const property_map& burstTag(bool startOfBurst) {
    static const property_map start{{kSquelchStartOfBurst, gr::pmt::Value(std::monostate{})}};
    static const property_map end{{kSquelchEndOfBurst, gr::pmt::Value(std::monostate{})}};
    return startOfBurst ? start : end;
}

/**
 * @brief The four-state ramp machine shared by the ramping squelches.
 *
 * Turns a per-sample "signal present" boolean into a raised-cosine envelope and a burst whose first and last produced
 * samples are the two tag positions. The attack runs env(1)..env(ramp) and the decay env(ramp)..env(1), so a burst is
 * symmetric and never truncated; neither ramp re-consults the detector, which costs a minimum burst of 2*ramp+1
 * samples and buys immunity to chatter at the threshold. A ramp length arriving mid-ramp rescales the counter.
 */
struct GateMachine {
    std::vector<float> _envelope{1.f};
    std::uint32_t      _ramp     = 0U;
    std::uint32_t      _counter  = 0U;
    GateState          _state    = GateState::Muted;
    bool               _starting = false;
    bool               _closing  = false;

    struct Step {
        float envelope;
        bool  produce;
        bool  endOfBurst;
    };

    void reset() noexcept {
        _state    = GateState::Muted;
        _counter  = 0U;
        _starting = false;
        _closing  = false;
    }

    [[nodiscard]] bool isOpen() const noexcept { return _state == GateState::Attack || _state == GateState::Unmuted; }

    [[nodiscard]] bool takeStart() noexcept { return std::exchange(_starting, false); }

    /// @brief Install a new ramp length, keeping a ramp in progress continuous and, at length zero, terminable.
    void setRamp(std::uint32_t length) {
        const std::uint32_t previous = _ramp;
        _ramp                        = length;
        _envelope.resize(static_cast<std::size_t>(length) + 1UZ);
        for (std::uint32_t r = 0U; r <= length; ++r) {
            _envelope[r] = length == 0U ? 1.f : static_cast<float>(0.5 * (1.0 - std::cos(std::numbers::pi * static_cast<double>(r) / static_cast<double>(length))));
        }
        if (_state != GateState::Attack && _state != GateState::Decay) {
            return;
        }
        if (length == 0U) {
            _closing = _state == GateState::Decay; // a canceled decay still owes its end-of-burst tag a sample
            _state   = _state == GateState::Attack ? GateState::Unmuted : GateState::Muted;
            _counter = 0U;
            return;
        }
        const double rescaled = std::round(static_cast<double>(_counter) * static_cast<double>(length) / static_cast<double>(previous));
        _counter              = static_cast<std::uint32_t>(std::clamp(rescaled, 1.0, static_cast<double>(length)));
    }

    [[nodiscard]] Step step(bool unmute) noexcept {
        if (_closing) {
            _closing = false;
            return {0.f, true, true};
        }
        if (_state == GateState::Muted) {
            if (!unmute) {
                return {0.f, false, false};
            }
            _state    = _ramp > 0U ? GateState::Attack : GateState::Unmuted;
            _counter  = 0U;
            _starting = _ramp == 0U;
        }
        return _state == GateState::Attack ? attack() : (_state == GateState::Unmuted ? open(unmute) : decay());
    }

private:
    [[nodiscard]] Step attack() noexcept {
        ++_counter;
        _starting            = _starting || _counter == 1U;
        const float envelope = _envelope[_counter];
        if (_counter == _ramp) {
            _state = GateState::Unmuted;
        }
        return {envelope, true, false};
    }

    [[nodiscard]] Step open(bool unmute) noexcept {
        if (unmute) {
            return {1.f, true, false};
        }
        if (_ramp == 0U) {
            _state = GateState::Muted;
            return {1.f, true, true};
        }
        _state   = GateState::Decay; // the sample the detector closed on still goes out at full amplitude
        _counter = _ramp;
        return {1.f, true, false};
    }

    [[nodiscard]] Step decay() noexcept {
        const float envelope = _envelope[_counter];
        --_counter;
        if (_counter == 0U) {
            _state = GateState::Muted;
            return {envelope, true, true};
        }
        return {envelope, true, false};
    }
};

/// @brief Single-pole power average against a linear threshold; the dB conversion happens on set, not per sample.
struct PowerDetector {
    double _power     = 0.0;
    double _alpha     = 1e-4;
    double _threshold = 1e-15;

    void reset() noexcept { _power = 0.0; }

    [[nodiscard]] bool unmuted(double magnitudeSquared) noexcept {
        _power = (1.0 - _alpha) * _power + _alpha * magnitudeSquared;
        if (_power < std::numeric_limits<double>::min()) {
            _power = 0.0; // a pole below one decays a silent input into subnormals and stays there; each one costs a microcode assist
        }
        return _power >= _threshold;
    }
};

/// @brief One single-bin DFT of length N by the three-term recursion, normalized so |X| does not depend on N.
struct Goertzel {
    double _cosine = 0.0;
    double _sine   = 0.0;
    double _s1     = 0.0;
    double _s2     = 0.0;

    void configure(double frequency, double sampleRate) {
        const double omega = 2.0 * std::numbers::pi * frequency / sampleRate;
        _cosine            = 2.0 * std::cos(omega);
        _sine              = std::sin(omega);
        reset();
    }

    void reset() noexcept { _s1 = _s2 = 0.0; }

    void push(double sample) noexcept {
        const double next = sample + _cosine * _s1 - _s2;
        _s2               = _s1;
        _s1               = next;
    }

    [[nodiscard]] double magnitude(std::uint32_t length) const noexcept {
        const double real = 0.5 * _cosine * _s1 - _s2;
        const double imag = _sine * _s1;
        return std::sqrt(real * real + imag * imag) / static_cast<double>(length);
    }
};

template<typename T>
[[nodiscard]] constexpr double magnitudeSquared(T sample) noexcept {
    if constexpr (std::same_as<T, float>) {
        return static_cast<double>(sample) * static_cast<double>(sample);
    } else {
        return static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
    }
}

/**
 * @brief Ports, tags and gating for a squelch whose detector the owning block supplies.
 *
 * The owner provides `detectorUnmuted(sample)`; everything here is the part the ramping squelches would otherwise each
 * get subtly differently. Tag forwarding is manual because the produced-sample map is data-dependent while gating: an
 * input tag on a suppressed sample moves forward to the next produced one, and one with no produced sample left in the
 * call is held rather than dropped. The key filter is the framework's, so an owned key such as `sample_rate` leaves
 * carrying the owner's value.
 */
template<typename TOwner>
struct SquelchGate {
    GateMachine               _gate{};
    std::vector<property_map> _carry{};

    void reset() {
        _gate.reset();
        _carry.clear();
    }

    template<typename TInSpan, typename TOutSpan>
    [[nodiscard]] std::size_t run(TOwner& owner, TInSpan& inSpan, TOutSpan& outSpan, std::size_t nSamples, bool gating) {
        const std::size_t           nTags  = inSpan.rawTags.size();
        std::size_t                 cursor = 0UZ;
        std::size_t                 made   = 0UZ;
        std::optional<property_map> cachedSettings;

        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            if (cursor < nTags) {
                collect(owner, inSpan, i, cursor, cachedSettings);
            }
            const GateMachine::Step step = _gate.step(owner.detectorUnmuted(inSpan[i]));
            if (!step.produce && gating) {
                continue;
            }
            outSpan[made] = inSpan[i] * step.envelope;
            emit(outSpan, made, step.endOfBurst);
            ++made;
        }
        return made;
    }

private:
    template<typename TInSpan>
    void collect(TOwner& owner, TInSpan& inSpan, std::size_t index, std::size_t& cursor, std::optional<property_map>& cachedSettings) {
        const auto&       raw   = inSpan.rawTags;
        const std::size_t until = inSpan.streamIndex + index;
        while (cursor < raw.size() && raw[cursor].index <= until) {
            if (raw[cursor].index >= inSpan.streamIndex) {
                // Every key survives, which the framework's auto-forward filter would not allow, and every reserved
                // key the owner declares as a setting leaves carrying the owner's value: the filtered map holds
                // exactly the substituted reserved keys, so overlaying it on the whole map is both rules at once.
                property_map forwarded(raw[cursor].map);
                for (auto& [key, value] : owner.filterAndSubstituteTag(raw[cursor].map, cachedSettings)) {
                    forwarded.insert_or_assign(key, std::move(value));
                }
                _carry.push_back(std::move(forwarded));
            }
            ++cursor;
        }
    }

    template<typename TOutSpan>
    void emit(TOutSpan& outSpan, std::size_t at, bool endOfBurst) {
        for (const property_map& forwarded : _carry) {
            outSpan.publishTag(forwarded, at);
        }
        _carry.clear();
        if (_gate.takeStart()) {
            outSpan.publishTag(burstTag(true), at);
        }
        if (endOfBurst) {
            outSpan.publishTag(burstTag(false), at);
        }
    }
};

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::analog::PowerSquelch, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct PowerSquelch : Block<PowerSquelch<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Silences a stream while its average power sits below a threshold, with click-free ramps and burst tags.

The detector is a single-pole average of `|x[n]|^2` compared against `threshold_db`, which is a power level: `-6.02 dB`
sits at magnitude `0.5`, not `0.25`. The average starts at zero, so the block starts muted and needs roughly `1/alpha`
samples to settle - about 0.2 s at 48 kHz at the default - which is why a squelch appears to take a moment to open.

`alpha` is the averager coefficient; `averaging_time_s`, in seconds and when positive, overrides it with
`alpha = 1 - exp(-1/(sample_rate * averaging_time_s))` and writes the result back. `threshold_db` reads back exactly as
set, never round-tripped through the linear form, so a slider does not drift. `threshold_min_db`, `threshold_max_db`
and `threshold_step_db` are advisory metadata for a user interface and take no part in processing.

`ramp` is the length in samples of a raised-cosine attack and decay; `0` makes transitions instantaneous. A burst is
the closed interval between the `squelch_sob` and `squelch_eob` tags and every sample in it carries signal, both ramps
included, so a consumer slicing on the tag pair gets exactly the audio the squelch passed. With `gate = false` the
block is exactly 1:1 and emits zeros while muted; with `gate = true` it emits nothing while muted and the output rate
becomes data-dependent. Input tags are re-published by hand: while gating, a tag on a suppressed sample moves forward
to the next produced sample rather than to an offset the rate change has invalidated. Every key survives that
re-publication, and a passing `sample_rate` tag leaves carrying this block's own rate rather than the upstream one it
arrived with.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<double, "threshold_db", Unit<"dB">, Doc<"average power below this mutes the output">>       threshold_db      = -150.0;
    Annotated<double, "alpha", Doc<"single-pole power averager coefficient, in (0, 1]">>                  alpha             = 1e-4;
    Annotated<double, "averaging_time_s", Unit<"s">, Doc<"when > 0, sets alpha from this time constant">> averaging_time_s  = 0.0;
    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"only used to derive alpha; also updated by a tag">>  sample_rate       = 48000.f;
    Annotated<gr::Size_t, "ramp", Doc<"attack and decay length in samples; 0 is instantaneous">>          ramp              = 0U;
    Annotated<bool, "gate", Doc<"true drops muted samples instead of zeroing them">>                      gate              = false;
    Annotated<double, "threshold_min_db", Unit<"dB">, Doc<"advisory range for a user interface">>         threshold_min_db  = -150.0;
    Annotated<double, "threshold_max_db", Unit<"dB">, Doc<"advisory range for a user interface">>         threshold_max_db  = 10.0;
    Annotated<double, "threshold_step_db", Unit<"dB">, Doc<"advisory range for a user interface">>        threshold_step_db = 1.0;
    Annotated<bool, "unmuted", Doc<"observable: true while the gate is open or attacking">>               unmuted           = false;

    GR_MAKE_REFLECTABLE(PowerSquelch, in, out, threshold_db, alpha, averaging_time_s, sample_rate, ramp, gate, threshold_min_db, threshold_max_db, threshold_step_db, unmuted);

    detail::PowerDetector                _detector{};
    detail::SquelchGate<PowerSquelch<T>> _machine{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        _detector._alpha     = detail::averagerCoefficient(alpha, averaging_time_s, static_cast<double>(sample_rate));
        _detector._threshold = std::pow(10.0, threshold_db / 10.0);
        alpha                = _detector._alpha;
        _machine._gate.setRamp(static_cast<std::uint32_t>(ramp.value));
    }

    void reset() {
        _detector.reset();
        _machine.reset();
    }

    [[nodiscard]] bool detectorUnmuted(T sample) noexcept { return _detector.unmuted(detail::magnitudeSquared(sample)); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t nSamples = std::min(inSpan.size(), outSpan.size());
        const std::size_t made     = _machine.run(*this, inSpan, outSpan, nSamples, gate);

        unmuted     = _machine._gate.isOpen();
        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(made);
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::analog::CtcssSquelch)

struct CtcssSquelch : Block<CtcssSquelch, NoTagPropagation> {
    using Description = Doc<R""(
@brief Opens only while a specific sub-audible continuous tone is present, using three single-bin DFTs.

Three Goertzel evaluators of length `length` run at `tone_freq` and at `tone_freq * (1 -/+ epsilon)`. Every `length`
samples the block unmutes if the center magnitude reaches `level` and exceeds both neighbors; between decisions the
previous one holds, and the decision before the first block completes is mute. The `1/N` normalization makes the
magnitude independent of the integration length: a unit-amplitude sinusoid exactly on the analysis frequency gives
`0.5`.

What the neighbors can do is bounded by the integration time, a Goertzel of length `N` having a bin width of
`sample_rate/N = 1/T`. The rule reliably rejects broadband noise and tones displaced by more than about `1/T` Hz, but
it does not separate adjacent standard CTCSS tones, which are 3 to 8 Hz apart, because no 100 ms measurement can. A
caller who needs that lengthens `length`, widens `epsilon` to match, and accepts the slower open.

`sample_rate` is a live setting, so a rate change retunes the evaluators instead of needing a new graph; that,
`tone_freq`, `length` or `epsilon` discards the partial integration and restarts, and the standing decision holds until
the first new block completes. Ramping, gating, the burst tags and the tag re-publication that puts this block's own
`sample_rate` into a passing tag of that key are the shared squelch machine documented on PowerSquelch.
)"">;

    PortIn<float>  in;
    PortOut<float> out;

    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"audio rate; also updated by a sample_rate tag">> sample_rate = 8000.f;
    Annotated<double, "tone_freq", Unit<"Hz">, Doc<"center (tone) frequency">>                        tone_freq   = 100.0;
    Annotated<double, "level", Doc<"minimum normalized center-bin magnitude required to unmute">>     level       = 0.01;
    Annotated<gr::Size_t, "length", Doc<"integration length N in samples; 0 selects sample_rate/10">> length      = 0U;
    Annotated<double, "epsilon", Doc<"neighbor bins sit at tone_freq * (1 -/+ epsilon)">>             epsilon     = 0.02;
    Annotated<gr::Size_t, "ramp", Doc<"attack and decay length in samples; 0 is instantaneous">>      ramp        = 0U;
    Annotated<bool, "gate", Doc<"true drops muted samples instead of zeroing them">>                  gate        = false;
    Annotated<double, "level_min", Doc<"advisory range for a user interface">>                        level_min   = 0.0;
    Annotated<double, "level_max", Doc<"advisory range for a user interface">>                        level_max   = 1.0;
    Annotated<double, "level_step", Doc<"advisory range for a user interface">>                       level_step  = 0.01;
    Annotated<bool, "unmuted", Doc<"observable: true while the gate is open or attacking">>           unmuted     = false;

    GR_MAKE_REFLECTABLE(CtcssSquelch, in, out, sample_rate, tone_freq, level, length, epsilon, ramp, gate, level_min, level_max, level_step, unmuted);

    std::array<detail::Goertzel, 3UZ> _bins{};
    std::array<double, 3UZ>           _magnitude{};
    std::uint32_t                     _length = 800U;
    std::uint32_t                     _filled = 0U;
    bool                              _open   = false;
    detail::SquelchGate<CtcssSquelch> _machine{};

    /// The three bins are tuned from the members, and a batch that moves no value never calls back, so a block
    /// constructed at its declared defaults is born on the tone its settings name.
    explicit CtcssSquelch(property_map init = {}) : Block<CtcssSquelch, NoTagPropagation>(std::move(init)) { configure(); }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { configure(); }

    void configure() {
        validate();
        _length = length == 0U ? static_cast<std::uint32_t>(sample_rate / 10.f) : static_cast<std::uint32_t>(length.value);
        _bins[0].configure(tone_freq * (1.0 - epsilon), static_cast<double>(sample_rate));
        _bins[1].configure(tone_freq, static_cast<double>(sample_rate));
        _bins[2].configure(tone_freq * (1.0 + epsilon), static_cast<double>(sample_rate));
        _filled = 0U;
        _machine._gate.setRamp(static_cast<std::uint32_t>(ramp.value));
    }

    void reset() {
        for (detail::Goertzel& bin : _bins) {
            bin.reset();
        }
        _filled = 0U;
        _open   = false;
        _machine.reset();
    }

    [[nodiscard]] bool detectorUnmuted(float sample) noexcept {
        for (detail::Goertzel& bin : _bins) {
            bin.push(static_cast<double>(sample));
        }
        ++_filled;
        return _filled < _length ? _open : decide();
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t nSamples = std::min(inSpan.size(), outSpan.size());
        const std::size_t made     = _machine.run(*this, inSpan, outSpan, nSamples, gate);

        unmuted     = _machine._gate.isOpen();
        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(made);
        return work::Status::OK;
    }

    /// @brief The magnitudes the most recent decision was taken on, in the order lower, center, upper.
    [[nodiscard]] double lastMagnitude(std::size_t which) const noexcept { return _magnitude[which]; }

    [[nodiscard]] std::uint32_t integrationLength() const noexcept { return _length; }

private:
    void validate() const {
        if (!(sample_rate > 0.f) || !std::isfinite(sample_rate)) {
            throw gr::exception(std::format("sample_rate must be positive and finite, got {}", sample_rate.value));
        }
        if (!(epsilon > 0.0) || !(tone_freq > 0.0)) {
            throw gr::exception(std::format("tone_freq ({}) and epsilon ({}) must both be positive", tone_freq.value, epsilon.value));
        }
        if (tone_freq * (1.0 + epsilon) >= 0.5 * static_cast<double>(sample_rate)) {
            throw gr::exception(std::format("the upper neighbor at {} Hz must stay below Nyquist", tone_freq * (1.0 + epsilon)));
        }
        if (length == 0U && sample_rate < 10.f) {
            throw gr::exception(std::format("the default integration length needs a sample_rate of at least 10 Hz, got {}", sample_rate.value));
        }
    }

    [[nodiscard]] bool decide() noexcept {
        for (std::size_t which = 0UZ; which < _bins.size(); ++which) {
            _magnitude[which] = _bins[which].magnitude(_length);
            _bins[which].reset();
        }
        _filled = 0U;
        _open   = _magnitude[1] >= level && _magnitude[1] > _magnitude[0] && _magnitude[1] > _magnitude[2];
        return _open;
    }
};

GR_REGISTER_BLOCK(gr::blocks::analog::SimpleSquelch, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct SimpleSquelch : Block<SimpleSquelch<T>, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief A hard power gate: no ramp, no burst tags, no rate change.

The same single-pole power detector PowerSquelch uses, wired straight to a multiply by one or zero, for a chain that
wants exactly that and no ramp state to reason about. For the same `alpha` and `threshold_db` it agrees with
PowerSquelch at `ramp = 0`, `gate = false` on every sample; the two detectors are one piece of code.

The average starts at zero, so the block starts muted, and `threshold_db` is a power level: the conversion uses `/10`
and happens once per settings change. Always exactly 1:1 and synchronous, so every input tag key passes through at its
own offset, `sample_rate` carrying this block's value.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<double, "threshold_db", Unit<"dB">, Doc<"average power below this mutes the output">>       threshold_db     = -150.0;
    Annotated<double, "alpha", Doc<"single-pole power averager coefficient, in (0, 1]">>                  alpha            = 1e-4;
    Annotated<double, "averaging_time_s", Unit<"s">, Doc<"when > 0, sets alpha from this time constant">> averaging_time_s = 0.0;
    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"only used to derive alpha; also updated by a tag">>  sample_rate      = 48000.f;
    Annotated<bool, "unmuted", Doc<"observable: the most recent sample's decision">>                      unmuted          = false;

    GR_MAKE_REFLECTABLE(SimpleSquelch, in, out, threshold_db, alpha, averaging_time_s, sample_rate, unmuted);

    detail::PowerDetector _detector{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        _detector._alpha     = detail::averagerCoefficient(alpha, averaging_time_s, static_cast<double>(sample_rate));
        _detector._threshold = std::pow(10.0, threshold_db / 10.0);
        alpha                = _detector._alpha;
    }

    void reset() { _detector.reset(); }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        const std::size_t nSamples = std::min(input.size(), output.size());
        bool              open     = unmuted;
        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            open      = _detector.unmuted(detail::magnitudeSquared(input[i]));
            output[i] = open ? input[i] : T{};
        }
        unmuted = open;
        return work::Status::OK;
    }
};

} // namespace gr::blocks::analog

#endif // GNURADIO_SQUELCH_HPP
