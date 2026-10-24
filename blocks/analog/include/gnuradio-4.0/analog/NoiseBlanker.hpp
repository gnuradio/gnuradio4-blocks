#ifndef GNURADIO_NOISE_BLANKER_HPP
#define GNURADIO_NOISE_BLANKER_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <expected>
#include <format>
#include <limits>
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
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/analog/detail/Averager.hpp>

namespace gr::blocks::analog {

namespace detail {

inline constexpr const char* kNoiseBlanked = "noise_blanked";

/// @brief The validator of `sample_rate`: true above 0.
inline constexpr auto kSampleRatePositive = [](float rate) { return rate > 0.f; };

/// @brief The validator of `alpha`: true above 0.
inline constexpr auto kAlphaPositive = [](double pole) { return pole > 0.0; };

enum class Replacement : std::uint8_t { Interpolate, Hold, Zero };

[[nodiscard]] inline std::expected<Replacement, gr::Error> parseReplacement(std::string_view name) {
    if (name == "interpolate") {
        return Replacement::Interpolate;
    }
    if (name == "hold") {
        return Replacement::Hold;
    }
    if (name == "zero") {
        return Replacement::Zero;
    }
    return std::unexpected(gr::Error(std::format("replacement must be 'interpolate', 'hold' or 'zero', got '{}'", name)));
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::analog::NoiseBlanker, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct NoiseBlanker : Block<NoiseBlanker<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Removes impulse noise from ignition, arcing, radar or switching supplies by replacing the offending samples.

The block works on a float or complex stream ahead of the filters that would spread an impulse. It gives one sample out
for each one in, the input delayed by `delay_samples`, the sum of `lookback_samples` and `blank_samples`. The delay
holds while the block is disabled. Every tag moves by `delay_samples` with its sample. With `emit_tags` set, the block
publishes a `noise_blanked` tag on the last replaced sample of each run, stating the run's length. At the end of the
stream the tags still held leave at the end-of-stream index.

A sample whose power `|x|^2` exceeds the tracked mean power by `threshold_db` opens a window. The window replaces the
`lookback_samples` before that sample and `blank_samples` from it on. A sample whose power is not finite counts as a
detection. The tracker leaves out each detected sample and the rest of its window. A detected sample never raises the
threshold it is compared against. At the default settings the block blanks about 0.18 % of a clean
circular-complex-Gaussian stream. With `retrigger` set, a detection inside a window restarts it, up to
`max_window_samples`.

`replacement` sets what replaces a run. `interpolate`, the default, draws a line from the last sample before the run to
the first after it. `hold` repeats the sample before the run, and `zero` writes zeros. Under `interpolate`, a run longer
than the delay line holds the sample before it until the run's end enters the line, then interpolates over the samples
left.

`averaging_time` sets the tracker's time constant in seconds at `sample_rate`. At 0 the block takes `alpha` as the pole,
and otherwise it writes the pole it derives to `alpha`. The block refuses a pole outside (0, 1], an `averaging_time`
long enough that the pole rounds to zero, and a `replacement` other than the three names. The tracker learns the level
during a warm-up of one averaging time, in which only a sample whose power is not finite counts as a detection. Enabling
the block and a change of the pole restart the warm-up. A count rises with each input from a detection to the end of its
window and falls with each passed one. When it reaches one averaging time of samples, the warm-up restarts too. A rise
of the level above the threshold is therefore replaced for about one averaging time and then passes.

A change of `lookback_samples` or `blank_samples` resizes the line at once and keeps the newest inputs. A shorter line
drops its oldest inputs. Their tags, and the marker of a run that ends among them, move to the next sample out. A longer
line puts out as many zeros as it grew by ahead of the kept inputs, or extends the run that is leaving. `reset()` drops
every input in the line as a shorter line does, fills the line with zeros and restarts the tracker.

`tracked_power` reports the mean power the threshold is taken against. `blanked_fraction` reports a slow average, over
ten tracker time constants, of the fraction of inputs from a detection to the end of its window. It leaves out the
`lookback_samples` replaced ahead of each detection. At the default settings it reads about 0.137 % on a clean
circular-complex-Gaussian stream, against about 0.18 % replaced. The block writes both at the end of each call.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<bool, "enabled", Doc<"blanking switch">, Visible>                                                                                                                                             enabled            = false;
    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"stream rate">, Limits<0.f, std::numeric_limits<float>::max(), detail::kSampleRatePositive>>                                                            sample_rate        = 96000.f;
    Annotated<double, "threshold_db", Unit<"dB">, Doc<"how far above the tracked mean power a sample must be">, Limits<std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max()>, Visible> threshold_db       = 9.32;
    Annotated<double, "averaging_time", Unit<"s">, Doc<"tracker time constant, 0 to use alpha directly">, Limits<0.0, std::numeric_limits<double>::max()>>                                                  averaging_time     = 0.01;
    Annotated<double, "alpha", Doc<"the pole when averaging_time is 0, read back as the resolved value">, Limits<0.0, 1.0, detail::kAlphaPositive>>                                                         alpha              = 1e-3;
    Annotated<gr::Size_t, "blank_samples", Doc<"length of the replaced run from a detected sample">, Limits<1U, std::numeric_limits<gr::Size_t>::max()>>                                                    blank_samples      = 7U;
    Annotated<gr::Size_t, "lookback_samples", Doc<"samples before the detected one also replaced">>                                                                                                         lookback_samples   = 2U;
    Annotated<std::string, "replacement", Doc<"'interpolate' (default), 'hold' or 'zero'">, Visible>                                                                                                        replacement        = std::string("interpolate");
    Annotated<bool, "retrigger", Doc<"restart the window on a detection inside one, up to max_window_samples">>                                                                                             retrigger          = false;
    Annotated<gr::Size_t, "max_window_samples", Doc<"cap on a retriggering window, 0 meaning 4 * blank_samples">>                                                                                           max_window_samples = 0U;
    Annotated<bool, "emit_tags", Doc<"publish noise_blanked on each run's last replaced sample, with its length">>                                                                                          emit_tags          = false;
    Annotated<gr::Size_t, "delay_samples", Doc<"observable lookback_samples + blank_samples, present when disabled">>                                                                                       delay_samples      = 9U;
    Annotated<double, "tracked_power", Doc<"observable censored mean power the threshold is taken against">>                                                                                                tracked_power      = 0.0;
    Annotated<double, "blanked_fraction", Doc<"observable slow average of the window duty">>                                                                                                                blanked_fraction   = 0.0;

    GR_MAKE_REFLECTABLE(NoiseBlanker, in, out, enabled, sample_rate, threshold_db, averaging_time, alpha, blank_samples, lookback_samples, replacement, retrigger, max_window_samples, emit_tags, delay_samples, tracked_power, blanked_fraction);

    std::vector<T>                                      _line{};
    std::vector<std::uint8_t>                           _marked{};      // a byte per slot and not a bitset, read and written once per sample
    std::vector<std::pair<std::uint64_t, property_map>> _pendingTags{}; ///< forwarded tags, at the output index their sample leaves at
    std::size_t                                         _cursor       = 0UZ;
    std::size_t                                         _oldest       = 1UZ;
    std::size_t                                         _ring         = 10UZ;
    std::size_t                                         _delay        = 9UZ;
    std::size_t                                         _lookback     = 2UZ;
    std::size_t                                         _blank        = 7UZ;
    std::size_t                                         _maxWindow    = 28UZ;
    std::size_t                                         _maxSetting   = 0UZ; ///< the accepted `max_window_samples`
    detail::Replacement                                 _rule         = detail::Replacement::Interpolate;
    bool                                                _enabled      = false;   ///< the accepted `enabled`
    bool                                                _retrigger    = false;   ///< the accepted `retrigger`
    bool                                                _emitTags     = false;   ///< the accepted `emit_tags`
    float                                               _rate         = 96000.f; ///< the accepted `sample_rate`
    double                                              _averaging    = 0.01;    ///< the accepted `averaging_time`
    double                                              _alpha        = 1e-3;
    double                                              _threshold    = 8.5507;
    double                                              _power        = 0.0;
    double                                              _duty         = 0.0;
    std::size_t                                         _warmup       = 960UZ;
    std::size_t                                         _warmupLength = 960UZ;
    std::uint32_t                                       _remaining    = 0U;
    std::uint32_t                                       _windowLength = 0U;
    std::uint32_t                                       _excess       = 0U;   ///< replaced inputs less passed ones, never below zero
    std::uint32_t                                       _runPosition  = 0U;   ///< replaced outputs of the current run so far
    std::uint32_t                                       _held         = 0U;   ///< of those, the outputs that held the sample before the run
    std::uint64_t                                       _emitted      = 0ULL; ///< outputs since start(), the index of the next one
    T                                                   _anchor{};

    /// Derives the coefficients and the delay line from the default settings.
    explicit NoiseBlanker(property_map init = {}) : Block<NoiseBlanker<T>, NoTagPropagation>(std::move(init)) { configure({}, true); }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) { configure(newSettings, false); }

    /// @brief Check the settings @p newSettings names, every one when @p everything is set, then take them together.
    void configure(const property_map& newSettings, bool everything) {
        const auto          moves       = [&newSettings, everything](std::string_view key) { return everything || newSettings.contains(key); };
        bool                nextEnabled = _enabled;
        float               rate        = _rate;
        double              averaging   = _averaging;
        double              pole        = _alpha;
        double              threshold   = _threshold;
        std::size_t         lookback    = _lookback;
        std::size_t         blank       = _blank;
        std::size_t         maxSetting  = _maxSetting;
        detail::Replacement rule        = _rule;
        bool                nextRetrig  = _retrigger;
        bool                nextTags    = _emitTags;

        if (moves("enabled")) {
            nextEnabled = enabled;
        }
        if (moves("sample_rate")) {
            rate = sample_rate;
        }
        if (moves("averaging_time")) {
            averaging = averaging_time;
        }
        if (moves("alpha")) {
            pole = alpha;
        }
        if (moves("threshold_db")) {
            threshold = std::pow(10.0, threshold_db / 10.0);
        }
        if (moves("blank_samples")) {
            blank = static_cast<std::size_t>(blank_samples.value);
        }
        if (moves("lookback_samples")) {
            lookback = static_cast<std::size_t>(lookback_samples.value);
        }
        if (moves("max_window_samples")) {
            maxSetting = static_cast<std::size_t>(max_window_samples.value);
        }
        if (moves("replacement")) {
            const auto parsed = detail::parseReplacement(replacement.value);
            if (!parsed) {
                throw gr::exception(parsed.error().message);
            }
            rule = *parsed;
        }
        if (moves("retrigger")) {
            nextRetrig = retrigger;
        }
        if (moves("emit_tags")) {
            nextTags = emit_tags;
        }
        const auto coefficient = detail::averagerCoefficient(pole, averaging, static_cast<double>(rate));
        if (!coefficient) {
            throw gr::exception(coefficient.error().message);
        }

        // the tracked power is stale after a pole move or after the tracker was disabled
        const bool restart = everything || *coefficient != _alpha || (nextEnabled && !_enabled);
        _enabled           = nextEnabled;
        _rate              = rate;
        _averaging         = averaging;
        _alpha             = *coefficient;
        alpha              = _alpha;
        _threshold         = threshold;
        _maxSetting        = maxSetting;
        _maxWindow         = maxSetting == 0UZ ? 4UZ * blank : maxSetting;
        _rule              = rule;
        _retrigger         = nextRetrig;
        _emitTags          = nextTags;
        _warmupLength      = averaging > 0.0 ? static_cast<std::size_t>(std::ceil(static_cast<double>(rate) * averaging)) : static_cast<std::size_t>(std::ceil(1.0 / _alpha));
        if (restart) {
            _warmup = _warmupLength;
        }
        if (everything || lookback != _lookback || blank != _blank) {
            relay(lookback, blank);
        }
    }

    void start() {
        _pendingTags.clear();
        _emitted = 0ULL;
        restart();
    }

    /// @brief Restart the line and the tracker. The line's tags move to the next sample out. `start()` discards them.
    void reset() {
        drop(_delay);
        restart();
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t           nSamples = std::min(inSpan.size(), outSpan.size());
        const std::size_t           nTags    = inSpan.rawTags.size();
        std::size_t                 tag      = 0UZ;
        std::size_t                 leaving  = 0UZ;
        std::optional<property_map> cachedSettings;

        // The block forwards the tags itself. Each tag waits `delay_samples` for its sample and leaves in index order
        // with the block's own markers. The filter is the framework's.
        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            const std::uint64_t at = _emitted + i;
            while (tag < nTags && inSpan.rawTags[tag].index <= inSpan.streamIndex + i) {
                if (inSpan.rawTags[tag].index >= inSpan.streamIndex) {
                    property_map forwarded = this->filterAndSubstituteTag(inSpan.rawTags[tag].map, cachedSettings);
                    if (!forwarded.empty()) {
                        _pendingTags.emplace_back(at + _delay, std::move(forwarded));
                    }
                }
                ++tag;
            }
            accept(inSpan[i]);
            while (leaving < _pendingTags.size() && _pendingTags[leaving].first <= at) {
                outSpan.publishTag(_pendingTags[leaving].second, i);
                ++leaving;
            }
            outSpan[i] = emit(outSpan, i);
        }
        _pendingTags.erase(_pendingTags.begin(), _pendingTags.begin() + static_cast<std::ptrdiff_t>(leaving));
        _emitted += nSamples;

        tracked_power    = _power;
        blanked_fraction = _duty;
        std::ignore      = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        return work::Status::OK;
    }

    /**
     * @brief At the end of the stream, publish every tag the block holds at the end-of-stream index.
     *
     * `processBulk` places every tag of an ordinary call. The framework's end-of-stream call passes `processedIn` 0 and
     * output spans of no samples, where offset 0 is the end-of-stream index. The samples in the line never leave. Their
     * forwarded tags, the markers of their runs and the tags left on the input go out there, in index order. The
     * `end_of_stream` key stays behind, since the framework publishes its own.
     */
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        if (processedIn > 0UZ) {
            return;
        }
        std::vector<std::pair<std::uint64_t, property_map>> leaving = std::move(_pendingTags);
        _pendingTags.clear();
        if (_emitTags) {
            std::uint32_t length = _runPosition;
            for (std::size_t ahead = 1UZ; ahead <= _delay; ++ahead) {
                if (_marked[slot(ahead)] == 0U) {
                    continue;
                }
                ++length;
                if (ahead == _delay || _marked[slot(ahead + 1UZ)] == 0U) {
                    leaving.emplace_back(_emitted + ahead - 1UZ, marker(length));
                    length = 0U;
                }
            }
        }
        std::ranges::stable_sort(leaving, std::ranges::less{}, &std::pair<std::uint64_t, property_map>::first);

        const std::pmr::string      endOfStream = static_cast<std::pmr::string>(gr::tag::END_OF_STREAM);
        std::optional<property_map> cachedSettings;
        gr::for_each_reader_span(
            [this, &leaving, &endOfStream, &cachedSettings](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const gr::Tag& tag : span.rawTags) {
                    if (tag.index < span.streamIndex) { // placed by an earlier call
                        continue;
                    }
                    property_map forwarded = this->filterAndSubstituteTag(tag.map, cachedSettings);
                    forwarded.erase(endOfStream);
                    if (!forwarded.empty()) {
                        leaving.emplace_back(std::numeric_limits<std::uint64_t>::max(), std::move(forwarded));
                    }
                }
            },
            inputSpans);

        for (const auto& held : leaving) {
            gr::for_each_writer_span(
                [&held](auto& span) {
                    if (span.isSync && span.isConnected) {
                        span.publishTag(held.second, 0UZ);
                    }
                },
                outputSpans);
        }
    }

private:
    [[nodiscard]] static constexpr double squaredMagnitude(T sample) noexcept {
        if constexpr (std::same_as<T, float>) {
            return static_cast<double>(sample) * static_cast<double>(sample);
        } else {
            return static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
        }
    }

    [[nodiscard]] static property_map marker(std::uint32_t length) { return property_map{{detail::kNoiseBlanked, static_cast<gr::Size_t>(length)}}; }

    /// The oldest entry plus @p ahead, wrapped by a compare. A compare costs less than a runtime integer modulo.
    [[nodiscard]] std::size_t slot(std::size_t ahead) const noexcept {
        const std::size_t at = _oldest + ahead;
        return at >= _ring ? at - _ring : at;
    }

    /**
     * @brief Size the line for @p lookback and @p blank, keeping the newest inputs and their marks.
     *
     * Slot 0 holds the sample emitted last, and slots 1 to the delay the inputs still to leave. A longer line takes
     * zeros ahead of the kept inputs. While a run is leaving, the zeros count as part of it. The run stays whole.
     */
    void relay(std::size_t lookback, std::size_t blank) {
        const std::size_t         delay = lookback + blank;
        std::vector<T>            line(delay + 1UZ, T{});
        std::vector<std::uint8_t> marked(delay + 1UZ, std::uint8_t{0});
        if (!_line.empty()) {
            const std::size_t kept = std::min(delay, _delay);
            line[0UZ]              = _line[slot(0UZ)];
            marked[0UZ]            = _marked[slot(0UZ)];
            for (std::size_t k = 1UZ; k <= kept; ++k) {
                line[delay - kept + k]   = _line[slot(_delay - kept + k)];
                marked[delay - kept + k] = _marked[slot(_delay - kept + k)];
            }
            if (_runPosition > 0U) {
                std::fill(marked.begin() + 1, marked.begin() + static_cast<std::ptrdiff_t>(delay - kept + 1UZ), std::uint8_t{1});
            }
            if (delay >= _delay) {
                for (auto& held : _pendingTags) {
                    held.first += delay - _delay;
                }
            } else {
                drop(_delay - delay);
            }
        }
        if (marked[1UZ] == 0U) {
            _runPosition = 0U;
            _held        = 0U;
        }
        _line         = std::move(line);
        _marked       = std::move(marked);
        _ring         = delay + 1UZ;
        _cursor       = delay;
        _oldest       = 0UZ;
        _delay        = delay;
        _lookback     = lookback;
        _blank        = blank;
        _remaining    = std::min(_remaining, static_cast<std::uint32_t>(blank));
        delay_samples = static_cast<gr::Size_t>(delay);
    }

    /// @brief Fill the line with zeros, clear its marks and restart the tracker.
    void restart() {
        std::ranges::fill(_line, T{});
        _marked.assign(_ring, std::uint8_t{0});
        _cursor       = 0UZ;
        _oldest       = 1UZ;
        _power        = 0.0;
        _duty         = 0.0;
        _warmup       = _warmupLength;
        _remaining    = 0U;
        _windowLength = 0U;
        _excess       = 0U;
        _runPosition  = 0U;
        _held         = 0U;
        _anchor       = T{};
    }

    /**
     * @brief Drop the @p dropped oldest inputs still to leave, as a shorter line or a reset does.
     *
     * Their tags, and the marker of each run that ends among them, move to the next sample out, in index order. A run
     * on the newest input of the line ends there. The last dropped input that was not replaced becomes the sample
     * before a later run. A run that goes on past the dropped inputs counts them in its length and in its
     * interpolation.
     */
    void drop(std::size_t dropped) {
        const bool                                          holding = _runPosition > 0U && _held == _runPosition;
        std::uint32_t                                       length  = _runPosition;
        std::size_t                                         tag     = 0UZ;
        std::vector<std::pair<std::uint64_t, property_map>> pending;
        pending.reserve(_pendingTags.size() + dropped);
        for (std::size_t ahead = 1UZ; ahead <= dropped; ++ahead) {
            for (; tag < _pendingTags.size() && _pendingTags[tag].first < _emitted + ahead; ++tag) {
                pending.emplace_back(_emitted, std::move(_pendingTags[tag].second));
            }
            if (_marked[slot(ahead)] == 0U) {
                _anchor = _line[slot(ahead)];
                continue;
            }
            ++length;
            if (ahead == _delay || _marked[slot(ahead + 1UZ)] == 0U) {
                if (_emitTags) {
                    pending.emplace_back(_emitted, marker(length));
                }
                length = 0U;
            }
        }
        for (; tag < _pendingTags.size(); ++tag) {
            pending.emplace_back(_pendingTags[tag].first - dropped, std::move(_pendingTags[tag].second));
        }
        const bool goesOn = _runPosition > 0U && length == _runPosition + dropped;
        _held             = goesOn ? (holding ? length : _held) : 0U;
        _runPosition      = length;
        _pendingTags      = std::move(pending);
    }

    /// @brief Take one input. Decide first, then track the power from the samples the decision left in place.
    void accept(T sample) {
        const double power      = squaredMagnitude(sample);
        const bool   windowOpen = _remaining > 0U;

        // Compare first and update after. A blanker that updates from every sample, impulse included, and
        // then tests against the value the impulse just raised desensitizes itself. A power that is not finite fails
        // every comparison and counts as a detection.
        bool detected = false;
        if (_warmup > 0UZ) {
            --_warmup;
            detected = _enabled && !(power < std::numeric_limits<double>::infinity());
        } else if (_enabled) {
            detected = !(power <= _threshold * _power);
        }
        if (!windowOpen && !detected && _enabled) {
            _power += _alpha * (power - _power);
            if (_power < std::numeric_limits<double>::min()) {
                _power = 0.0; // a pole below one decays a silent input into subnormals, where it stays, and each one costs a microcode assist
            }
        }

        _cursor        = _cursor + 1UZ == _ring ? 0UZ : _cursor + 1UZ;
        _oldest        = _cursor + 1UZ == _ring ? 0UZ : _cursor + 1UZ;
        _line[_cursor] = sample;
        bool marked    = windowOpen;

        if (detected && !windowOpen) {
            for (std::size_t back = 1UZ; back <= _lookback; ++back) { // the samples before the detected one, still unemitted
                _marked[_cursor >= back ? _cursor - back : _cursor + _ring - back] = 1U;
            }
            _windowLength = static_cast<std::uint32_t>(_lookback);
            _remaining    = static_cast<std::uint32_t>(_blank);
            marked        = true;
        }

        _marked[_cursor] = marked ? std::uint8_t{1} : std::uint8_t{0};
        _windowLength    = marked ? _windowLength + 1U : 0U;
        _excess          = marked ? _excess + 1U : (_excess > 0U ? _excess - 1U : 0U);
        if (_excess >= _warmupLength) { // a level rise keeps most samples above the threshold: learn the new level
            _warmup = _warmupLength;
            _excess = 0U;
        }
        _duty += 0.1 * _alpha * ((marked ? 1.0 : 0.0) - _duty); // ten tracker time constants, since a 0.14 % duty needs a longer average than the power
        if (_duty < std::numeric_limits<double>::min()) {
            _duty = 0.0; // the same decay into the subnormals as the power, ten times slower
        }

        if (detected && windowOpen && _retrigger) {
            const std::uint32_t room = _windowLength > static_cast<std::uint32_t>(_maxWindow) ? 0U : static_cast<std::uint32_t>(_maxWindow) + 1U - _windowLength;
            _remaining               = std::min(static_cast<std::uint32_t>(_blank), room);
        }
        if (_remaining > 0U) {
            --_remaining;
        }
    }

    /**
     * @brief Produce the output for the sample that entered `delay_samples` ago, replaced if it was marked.
     *
     * A run whose end is in the line is interpolated from the sample before it to the sample after it. A run that
     * reaches past the line holds the sample before it until its end enters the line. The interpolation then spans
     * the samples left. The marker goes on the run's last replaced sample.
     */
    template<typename TOutSpan>
    [[nodiscard]] T emit(TOutSpan& outSpan, std::size_t at) {
        const std::size_t oldest = slot(0UZ);
        if (_marked[oldest] == 0U) {
            _anchor      = _line[oldest];
            _runPosition = 0U;
            _held        = 0U;
            return _anchor;
        }

        std::size_t ahead = 1UZ;
        while (ahead <= _delay && _marked[slot(ahead)] != 0U) {
            ++ahead;
        }
        ++_runPosition;
        if (ahead > _delay) {
            _held = _runPosition;
            return _rule == detail::Replacement::Zero ? T{} : _anchor;
        }

        const std::uint32_t position = _runPosition - _held;
        const std::uint32_t span     = position + static_cast<std::uint32_t>(ahead) - 1U;
        if (ahead == 1UZ) {
            if (_emitTags) {
                outSpan.publishTag(marker(_runPosition), at);
            }
            _runPosition = 0U;
            _held        = 0U;
        }

        switch (_rule) {
        case detail::Replacement::Zero: return T{};
        case detail::Replacement::Hold: return _anchor;
        default: break;
        }
        const T      after  = _line[slot(ahead)];
        const double weight = static_cast<double>(position) / static_cast<double>(span + 1U);
        if constexpr (std::same_as<T, float>) {
            return static_cast<float>(static_cast<double>(_anchor) + (static_cast<double>(after) - static_cast<double>(_anchor)) * weight);
        } else {
            return T(static_cast<float>(static_cast<double>(_anchor.real()) + (static_cast<double>(after.real()) - static_cast<double>(_anchor.real())) * weight), //
                static_cast<float>(static_cast<double>(_anchor.imag()) + (static_cast<double>(after.imag()) - static_cast<double>(_anchor.imag())) * weight));
        }
    }
};

} // namespace gr::blocks::analog

#endif // GNURADIO_NOISE_BLANKER_HPP
