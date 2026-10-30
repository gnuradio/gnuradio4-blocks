#ifndef GNURADIO_CHANNEL_RANGE_DELAY_HPP
#define GNURADIO_CHANNEL_RANGE_DELAY_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
#include <expected>
#include <format>
#include <iterator>
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

#include <gnuradio-4.0/algorithm/MeasurementSlot.hpp>
#include <gnuradio-4.0/algorithm/filter/FractionalDelay.hpp>
#include <gnuradio-4.0/algorithm/timing/FrequencySchedule.hpp>
#include <gnuradio-4.0/algorithm/timing/SampleClock.hpp>
#include <gnuradio-4.0/algorithm/timing/ScheduleAnchor.hpp>
#include <gnuradio-4.0/algorithm/timing/TrajectoryFile.hpp>

#include <gnuradio-4.0/channel/DopplerShift.hpp>

namespace gr::blocks::channel {

namespace detail {

/// @brief The validator of `order`: true for every order but 2, inside the limits 0 to 3.
inline constexpr auto kDelayLineOrder = [](int order) { return order != 2; };

/// @brief The validator of `rolloff`: true strictly between 0 and 1.
inline constexpr auto kOpenUnitInterval = [](double value) { return value > 0.0 && value < 1.0; };

/// @brief The validator of `bank_size`: true for 0 or a power of two.
inline constexpr auto kZeroOrPowerOfTwo = [](gr::Size_t size) { return (size & (size - 1U)) == 0U; };

/// What to change when a bank cannot be designed. A derived bank is not set, so the advice names what it follows.
[[nodiscard]] inline std::string designAdvice(bool derived) { return derived ? "'bank_size' 0 derives the bank, so set 'bank_size' to a smaller power of two, raise 'order' to 3, lower 'attenuation_db' or raise 'rolloff'" : "lower 'bank_size' or 'attenuation_db', or raise 'rolloff'"; }

/// The refusal of a bank whose prototype would need more than the design allows, given before any design runs.
[[nodiscard]] inline std::string tooManyTaps(std::size_t bank, bool derived, double rolloff, double attenuationDb) {
    return std::format("RangeDelay: 'bank_size' {}{} with 'rolloff' {} and 'attenuation_db' {} needs about {} taps, past the {} the design allows; {}", //
        bank, derived ? " (derived)" : "", rolloff, attenuationDb, gr::filter::fractionalDelayTapEstimate(bank, rolloff, attenuationDb), gr::filter::kFractionalDelayMaxTaps, designAdvice(derived));
}

/// The refusal of a design that missed its stopband or its ripple limit, or none where it met them.
[[nodiscard]] inline std::optional<std::string> designShortfall(std::size_t bank, bool derived, double rolloff, double attenuationDb, const gr::filter::ResamplerDesign& design) {
    if (design.ok) {
        return std::nullopt;
    }
    return std::format("RangeDelay: 'bank_size' {}{} with 'rolloff' {} and 'attenuation_db' {} designs {} taps that deliver a stopband of {:.2f} dB and a ripple of {:.3f} dB, short of 'attenuation_db' or the ripple limit; {}", //
        bank, derived ? " (derived)" : "", rolloff, attenuationDb, design.taps.size(), design.stopbandDb, design.rippleDb, designAdvice(derived));
}

/// The settings a RangeDelay computes on, other than its table and anchor, as last accepted.
struct RangeDelaySettings {
    float            rate{1.f};
    DopplerDirection direction{DopplerDirection::Apply};
    double           bias{std::numeric_limits<double>::quiet_NaN()}; ///< `bias_s`, NaN to derive it
    gr::Size_t       bankSize{0U};
    int              order{1};
    double           attenuationDb{60.0};
    double           rolloff{0.2};
};

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::channel::RangeDelay, [T], [ std::complex<float>, float ])

template<typename T>
requires std::is_same_v<T, std::complex<float>> || std::is_same_v<T, float>
struct RangeDelay : gr::Block<RangeDelay<T>, gr::NoTagPropagation> {
    using Description = Doc<R""(
@brief Applies a trajectory's envelope delay to a stream, or removes it.

A source at slant range `R(t)` arrives `tau(t) = R(t)/c` seconds late. Its baseband equivalent is
`x(t - tau(t)) * exp(-j 2 pi f_c tau(t))`. This block carries the delay `x(t - tau(t))`, and `DopplerShift` carries
the phase factor. To apply one trajectory, run RangeDelay and then DopplerShift. To remove it, run DopplerShift's
correction and then RangeDelay's. The block outputs one sample per input sample through a
`gr::filter::FractionalDelayLine`.

The delays come from `schedule_times_ns` and `schedule_delays_s`, or from the `range_m` column of a
`#!gr4-trajectory 1` file named in `schedule_file`, read as `range/c`. Setting both is refused. The schedule is linear
between knots and holds its end values outside them. With neither, the commanded delay is the bias alone.

Under `direction` 'apply' the commanded delay is the schedule plus `bias_s`. Under 'correct' it is `bias_s` less the
schedule. A NaN `bias_s`, the default, derives the bias: zero under 'apply' and the schedule's maximum under 'correct'.
The commanded delay is then non-negative, and the corrected stream carries a constant delay of that maximum. A knot
whose commanded delay is negative or past the line's reach is refused, and so is a segment whose slope reaches one
sample per sample. The refusal names the knot or the segment.

`bank_size`, `order`, `attenuation_db` and `rolloff` design the line. `bank_size` 0 derives the bank from the other
three. A bank whose estimated prototype needs more than 65536 taps is refused before it is designed. The estimate
follows `bank_size * (attenuation_db - 8) / rolloff`, and below 45 dB the ripple limit makes it longer, many times so
for a rolloff up to 0.85. From 45 dB up no rolloff below one holds more than 8192 arms. Below 45 dB a rolloff above 0.7
is estimated short, so the check can admit a bank the design cannot hold, and the block refuses it after the design. At
`rolloff` 0.2 with `bank_size` 0, `order` 1 designs up to 110 dB and `order` 3 up to 140 dB. A design near the cap takes
tens of seconds to a few minutes inside the settings change, and longer in a Debug build. A change to any of the four
builds a new bank, drops the line's history and counts in `nBankRebuilds()`. A design that misses `attenuation_db` or
its ripple limit is refused too, and the refusal gives the stopband and ripple it delivers. A change of table,
direction, bias or rate takes effect at the next sample and keeps the history. A new rate re-times the whole stream from
the anchor.

The settings `anchor_source`, `anchor_index`, `anchor_ns` and `honor_trigger_offset` and the counters
`nIgnoredAnchors()`, `nReanchors()` and `nRefusedAnchors()` work as in `DopplerShift`. Until a trigger anchor is armed,
the stream passes through and the line takes nothing. An arm, or a move of an armed anchor to another index or time,
by a tag or by a settings change, clears the line's history. Until it fills, up to `historySamples()` outputs are the filter's response to silence.

A tag with a key of `gr::tag::kDefaultTags` leaves on the sample it arrived on. Every other tag leaves on the first
output sample whose whole read position has reached the tag's input index. A tag still waiting when the stream ends
leaves at the end-of-stream index. `nTagsCoalesced()` counts the output samples that carry two or more such tags.
`nDiscontinuities()` counts `n_dropped_samples` tags, a gap the schedule does not bridge. `nRateDisagreements()`
counts `sample_rate` tags that differ from the accepted `sample_rate`. The schedule stays at that rate.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<std::vector<std::int64_t>, "schedule_times_ns", Visible, Unit<"ns">, Doc<detail::kScheduleTimesDoc>>                           schedule_times_ns{};
    Annotated<std::vector<double>, "schedule_delays_s", Visible, Unit<"s">, Doc<"knot delays in seconds, one per time">>                     schedule_delays_s{};
    Annotated<std::string, "schedule_file", Doc<"#!gr4-trajectory 1 file with a range column">>                                              schedule_file{};
    Annotated<std::string, "direction", Visible, Doc<detail::kDirectionDoc>>                                                                 direction            = std::string("apply");
    Annotated<double, "bias_s", Unit<"s">, Doc<"constant delay added to the schedule, NaN to derive it">>                                    bias_s               = std::numeric_limits<double>::quiet_NaN();
    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<detail::kSampleRateDoc>, detail::ClockRateLimits>                               sample_rate          = 1.f;
    Annotated<std::uint64_t, "anchor_index", Doc<detail::kAnchorIndexDoc>>                                                                   anchor_index         = 0ULL;
    Annotated<std::int64_t, "anchor_ns", Unit<"ns">, Doc<detail::kAnchorNsDoc>, detail::AnchorNsLimits>                                      anchor_ns            = 0LL;
    Annotated<std::string, "anchor_source", Doc<detail::kAnchorSourceDoc>>                                                                   anchor_source        = std::string("setting");
    Annotated<bool, "honor_trigger_offset", Doc<detail::kHonorTriggerOffsetDoc>>                                                             honor_trigger_offset = true;
    Annotated<gr::Size_t, "bank_size", Doc<"polyphase arms, 0 or a power of two, up to 8192">, Limits<0U, 8192U, detail::kZeroOrPowerOfTwo>> bank_size            = 0U;
    Annotated<int, "order", Doc<"interpolation order between arms, 0, 1 or 3">, Limits<0, 3, detail::kDelayLineOrder>>                       order                = 1;
    Annotated<double, "attenuation_db", Unit<"dB">, Doc<"prototype stopband attenuation, in [20, 140]">, Limits<20.0, 140.0>>                attenuation_db       = 60.0;
    Annotated<double, "rolloff", Doc<"the design's excess bandwidth, in (0, 1)">, Limits<0.0, 1.0, detail::kOpenUnitInterval>>               rolloff              = 0.2;

    GR_MAKE_REFLECTABLE(RangeDelay, in, out, schedule_times_ns, schedule_delays_s, schedule_file, direction, bias_s, sample_rate, anchor_index, anchor_ns, anchor_source, honor_trigger_offset, bank_size, order, attenuation_db, rolloff);

    // The accepted settings. A refused change leaves all of them as they were.
    std::optional<gr::timing::DelaySchedule> _schedule{}; ///< the table as configured, before the direction's sign
    detail::RangeDelaySettings               _accepted{};
    detail::TableSettings                    _tableSettings{}; ///< the table settings as last accepted
    detail::AnchorSettings                   _anchorSettings{};
    bool                                     _configured{false}; ///< a settings change has been accepted

    std::optional<gr::filter::FractionalDelayLine<T>>   _line{};
    gr::timing::SampleClock                             _clock{};
    gr::timing::ScheduleAnchor                          _anchor{};
    std::uint64_t                                       _position{0ULL};
    double                                              _bias{0.};        ///< the bias in force, derived when `bias_s` is NaN
    std::int64_t                                        _latencyQ32{0LL}; ///< the line's own lag in the fixed point the tag map reads
    std::vector<double>                                 _delays{};        ///< one call's worth of commanded delays, grown not rebuilt
    std::vector<std::uint64_t>                          _delaysQ32{};
    std::vector<std::pair<std::uint64_t, property_map>> _tagsMoving{}; ///< tags waiting for their output sample, by ascending absolute input index
    std::vector<std::pair<std::uint64_t, property_map>> _tagsHeld{};   ///< the same, for the tags that leave on the sample they arrived on

    /// The design the line's bank was built from.
    std::size_t _builtBank{0UZ};
    int         _builtOrder{0};
    double      _builtRolloff{0.};
    double      _builtAttenuationDb{0.};

    gr::measurement::MeasurementSlot<7UZ> _slot{}; ///< delay s, delay samples, bias s, latency, group delay, schedule position, history, with the stream position as fill count
    std::atomic<std::uint64_t>            _nTagsCoalesced{0ULL};
    std::atomic<std::uint64_t>            _nBankRebuilds{0ULL};
    std::atomic<std::uint64_t>            _nDiscontinuities{0ULL};
    std::atomic<std::uint64_t>            _nRateDisagreements{0ULL};
    // The anchor state, copied for a reader on another thread. An anchor time is a nanosecond count beyond the
    // exact range of a double. It is stored as an integer beside the slot.
    std::atomic<bool>          _armed{false};
    std::atomic<std::uint64_t> _anchorIndex{0ULL};
    std::atomic<std::int64_t>  _anchorNs{0LL};
    std::atomic<std::uint64_t> _nIgnoredAnchors{0ULL};
    std::atomic<std::uint64_t> _nReanchors{0ULL};
    std::atomic<std::uint64_t> _nRefusedAnchors{0ULL};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const auto moved = [&](std::string_view key, bool differs) { return detail::moves(newSettings, _configured, key, differs); };

        detail::RangeDelaySettings next = _accepted;
        if (moved("sample_rate", sample_rate.value != _accepted.rate)) {
            next.rate = sample_rate.value;
        }
        if (moved("direction", direction.value != detail::dopplerDirectionName(_accepted.direction))) {
            next.direction = detail::parseDopplerDirection(direction.value, "RangeDelay");
        }
        if (moved("bias_s", !sameDouble(bias_s.value, _accepted.bias))) {
            next.bias = bias_s.value;
        }
        if (moved("bank_size", bank_size.value != _accepted.bankSize)) {
            next.bankSize = bank_size.value;
        }
        if (moved("order", order.value != _accepted.order)) {
            next.order = order.value;
        }
        if (moved("attenuation_db", attenuation_db.value != _accepted.attenuationDb)) {
            next.attenuationDb = attenuation_db.value;
        }
        if (moved("rolloff", rolloff.value != _accepted.rolloff)) {
            next.rolloff = rolloff.value;
        }
        const detail::TableSettings              tableNamed{schedule_times_ns.value, schedule_delays_s.value, schedule_file.value};
        const bool                               tableDiffers = tableNamed != _tableSettings;
        const bool                               tableMoved   = moved("schedule_times_ns", tableDiffers) || moved("schedule_delays_s", tableDiffers) || moved("schedule_file", tableDiffers);
        std::optional<gr::timing::DelaySchedule> table        = tableMoved ? loadTable() : _schedule;

        const bool             anchorMoved  = moved("anchor_source", anchor_source.value != gr::timing::anchorSourceName(_anchorSettings.source)) || moved("anchor_index", anchor_index.value != _anchorSettings.index) || moved("anchor_ns", anchor_ns.value != _anchorSettings.ns) || moved("honor_trigger_offset", honor_trigger_offset.value != _anchorSettings.honorOffset);
        detail::AnchorSettings anchorConfig = _anchorSettings;
        if (anchorMoved) {
            anchorConfig = {detail::parseAnchorSource(anchor_source.value, "RangeDelay"), anchor_index.value, anchor_ns.value, honor_trigger_offset.value};
        }
        const gr::timing::ScheduleAnchor anchor = anchorMoved ? gr::timing::ScheduleAnchor(anchorConfig.source, anchorConfig.index, anchorConfig.ns, anchorConfig.honorOffset) : _anchor;

        configure(next, std::move(table), anchor);
        _tableSettings  = tableNamed;
        _anchorSettings = anchorConfig;
    }

    void start() {
        if (!_line) { // no settings change has been accepted, and the defaults build the line
            try {
                configure(_accepted, _schedule, gr::timing::ScheduleAnchor(_anchorSettings.source, _anchorSettings.index, _anchorSettings.ns, _anchorSettings.honorOffset));
            } catch (const gr::exception& refusal) {
                this->emitErrorMessage("start", gr::Error(refusal.message, refusal.sourceLocation));
            }
        }
        _position = 0ULL;
        _anchor.reset();
        if (_line) {
            _line->reset();
        }
        _tagsMoving.clear();
        _tagsHeld.clear();
        _nTagsCoalesced.store(0ULL, std::memory_order_relaxed);
        _nDiscontinuities.store(0ULL, std::memory_order_relaxed);
        _nRateDisagreements.store(0ULL, std::memory_order_relaxed);
        publishMeasurements();
    }

    /// @brief The delay commanded now, in seconds. It is the schedule plus the bias, negated in `correct`. Any thread.
    [[nodiscard]] double currentDelaySeconds() const noexcept { return _slot.read().first[0]; }
    /// @brief The same delay in samples at the block's own rate. Any thread.
    [[nodiscard]] double currentDelaySamples() const noexcept { return _slot.read().first[1]; }
    /// @brief The constant the schedule is shifted by, derived under `correct` unless `bias_s` states it. Any thread.
    [[nodiscard]] double biasSeconds() const noexcept { return _slot.read().first[2]; }
    /// @brief The output's lag behind the input at a commanded delay of zero. It is the prototype's lag plus one sample.
    [[nodiscard]] double latencySamples() const noexcept { return _slot.read().first[3]; }
    /// @brief The prototype's own delay of `(N-1)/(2L)` input samples. `latencySamples()` is one sample more.
    [[nodiscard]] double groupDelaySamples() const noexcept { return _slot.read().first[4]; }
    /// @brief The next sample's place relative to the schedule's span. `Before` and `After` hold an end value.
    [[nodiscard]] SchedulePosition schedulePosition() const noexcept { return static_cast<SchedulePosition>(static_cast<std::uint8_t>(_slot.read().first[5])); }
    /// @brief The samples of history the line holds. Any thread.
    [[nodiscard]] std::size_t historySamples() const noexcept { return static_cast<std::size_t>(_slot.read().first[6]); }

    /// @brief The index of the next sample from the stream's start. The anchor places it in time. Any thread.
    [[nodiscard]] std::uint64_t position() const noexcept { return _slot.read().second; }
    [[nodiscard]] bool          anchorArmed() const noexcept { return _armed.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t anchorIndex() const noexcept { return _anchorIndex.load(std::memory_order_relaxed); }
    [[nodiscard]] std::int64_t  anchorNs() const noexcept { return _anchorNs.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nIgnoredAnchors() const noexcept { return _nIgnoredAnchors.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nReanchors() const noexcept { return _nReanchors.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nRefusedAnchors() const noexcept { return _nRefusedAnchors.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nTagsCoalesced() const noexcept { return _nTagsCoalesced.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nBankRebuilds() const noexcept { return _nBankRebuilds.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nDiscontinuities() const noexcept { return _nDiscontinuities.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nRateDisagreements() const noexcept { return _nRateDisagreements.load(std::memory_order_relaxed); }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) {
        const std::size_t nSamples = std::min(input.size(), output.size());
        reserve(nSamples);
        const auto filtered = runSegment(input.first(nSamples), output.first(nSamples), 0UZ);
        publishMeasurements();
        if (!filtered) {
            this->emitErrorMessage("processBulk", filtered.error());
            return work::Status::ERROR;
        }
        return work::Status::OK;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t   nSamples  = std::min(inSpan.size(), outSpan.size());
        const std::uint64_t callStart = _position;
        reserve(nSamples);

        // A tag with a reserved key waits in `_tagsHeld`, and every other tag in `_tagsMoving`. One walk over the
        // output places both kinds in order.
        _tagsHeld.clear();
        for (const auto& [relIndex, mapRef] : inSpan.tags(nSamples)) {
            if (relIndex < 0 || static_cast<std::size_t>(relIndex) >= nSamples) {
                continue;
            }
            const property_map& map = mapRef.get();
            const std::size_t   at  = static_cast<std::size_t>(relIndex);
            countStreamKeys(map);
            if (namesReservedKey(map)) {
                _tagsHeld.emplace_back(callStart + at, map);
            } else {
                _tagsMoving.emplace_back(callStart + at, map);
            }
        }

        std::size_t done = 0UZ;
        for (const auto& [relIndex, mapRef] : inSpan.tags(nSamples)) {
            if (relIndex < 0 || static_cast<std::size_t>(relIndex) >= nSamples) {
                continue;
            }
            const detail::TriggerRead read = detail::readTrigger(mapRef.get(), _anchor.honorTriggerOffset());
            if (!read.present) {
                continue;
            }
            const std::size_t at = static_cast<std::size_t>(relIndex);
            // The call splits at the trigger, and the samples before it keep their anchor.
            if (const auto advanced = advance(inSpan, outSpan, done, at); !advanced) {
                return refuse(inSpan, outSpan, done, advanced.error());
            }
            done = at;

            const gr::timing::ScheduleAnchor::Response response = read.readable ? _anchor.onTrigger(_position, read.timeNs, read.offsetSeconds) : _anchor.onUnreadableTrigger();
            if (response == gr::timing::ScheduleAnchor::Response::armed || response == gr::timing::ScheduleAnchor::Response::reanchored) {
                const auto clock = clockAt(_accepted.rate, _anchor.anchorIndex(), _anchor.anchorNs());
                if (!clock) {
                    return refuse(inSpan, outSpan, done, clock.error());
                }
                _clock = *clock;
                // The line starts on silence at each anchor. Its history holds samples from before a jump in schedule
                // time, or from before a stretch the block passed through unarmed.
                if (_line) {
                    _line->reset();
                }
            }
        }
        if (const auto advanced = advance(inSpan, outSpan, done, nSamples); !advanced) {
            return refuse(inSpan, outSpan, done, advanced.error());
        }

        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        publishMeasurements();
        return work::Status::OK;
    }

    /// @brief At the end of the stream, publishes every tag still waiting, and every tag past the last input, at the
    /// end-of-stream index. `processBulk` places the tags of each chunk.
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        if (processedIn != 0UZ) {
            return;
        }
        std::vector<std::pair<std::uint64_t, property_map>> waiting = std::move(_tagsHeld);
        waiting.insert(waiting.end(), std::make_move_iterator(_tagsMoving.begin()), std::make_move_iterator(_tagsMoving.end()));
        _tagsHeld.clear();
        _tagsMoving.clear();
        gr::for_each_reader_span(
            [&waiting, this](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                const std::pmr::string endOfStream = static_cast<std::pmr::string>(gr::tag::END_OF_STREAM);
                for (const gr::Tag& tag : span.rawTags) {
                    if (tag.index < span.streamIndex) { // placed by an earlier call
                        continue;
                    }
                    property_map map = tag.map;
                    map.erase(endOfStream);
                    if (!map.empty()) {
                        countStreamKeys(map);
                        waiting.emplace_back(static_cast<std::uint64_t>(tag.index), std::move(map));
                    }
                }
            },
            inputSpans);
        if (waiting.empty()) {
            return;
        }
        std::ranges::stable_sort(waiting, [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
        gr::for_each_writer_span(
            [&waiting](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const auto& [index, map] : waiting) {
                    span.publishTag(map, 0UZ);
                }
            },
            outputSpans);
    }

private:
    /// True when @p a and @p b are the same setting. Two NaN values are the same.
    [[nodiscard]] static bool sameDouble(double a, double b) noexcept { return a == b || (std::isnan(a) && std::isnan(b)); }

    /// The bias and the history a table needs, kept apart from the block until every check has passed.
    struct Staged {
        double        bias{0.};
        std::uint64_t wholeDelaySamples{0ULL};
    };

    [[nodiscard]] static std::expected<gr::timing::SampleClock, gr::Error> clockAt(float rate, std::uint64_t index, std::int64_t timeNs) {
        return gr::timing::clockForRateHz(static_cast<double>(rate), index, timeNs).transform_error([](const gr::Error& refusal) { return gr::Error(std::format("RangeDelay: 'sample_rate': {}", refusal.message), refusal.sourceLocation); });
    }

    /**
     * @brief Checks @p next with @p table and @p anchor and puts all three in force.
     *
     * Every check runs before anything changes. A new bank is built only when a design value differs from the
     * line's. Otherwise the line keeps its history and grows to the reach the table needs.
     */
    void configure(const detail::RangeDelaySettings& next, std::optional<gr::timing::DelaySchedule> table, const gr::timing::ScheduleAnchor& anchor) {
        // The clock of an unarmed anchor checks the rate and is not installed.
        const auto clock = clockAt(next.rate, anchor.armed() ? anchor.anchorIndex() : 0ULL, anchor.armed() ? anchor.anchorNs() : 0LL);
        if (!clock) {
            throw gr::exception(clock.error().message, clock.error().sourceLocation);
        }
        const std::size_t bank   = next.bankSize == 0U ? gr::filter::arbitraryBankSize(next.attenuationDb, next.rolloff, next.order) : static_cast<std::size_t>(next.bankSize);
        const Staged      staged = stage(table, next);

        const bool held    = _line.has_value();
        const bool cutBank = !held || bank != _builtBank || next.order != _builtOrder || next.rolloff != _builtRolloff || next.attenuationDb != _builtAttenuationDb;
        if (cutBank) {
            // The held samples do not line up with a new prototype's taps, and a new bank starts on silence.
            if (!gr::filter::fractionalDelayReachable(bank, next.rolloff, next.attenuationDb)) {
                throw gr::exception(detail::tooManyTaps(bank, next.bankSize == 0U, next.rolloff, next.attenuationDb));
            }
            const auto design = gr::filter::designFractionalDelay(bank, next.rolloff, next.attenuationDb);
            if (!design) {
                throw gr::exception(std::format("RangeDelay: {}", design.error().message), design.error().sourceLocation);
            }
            if (const std::optional<std::string> shortfall = detail::designShortfall(bank, next.bankSize == 0U, next.rolloff, next.attenuationDb, *design)) {
                throw gr::exception(*shortfall);
            }
            gr::filter::FractionalDelayLine<T> built(bank, next.order, std::span<const float>(design->taps), staged.wholeDelaySamples);
            _line               = std::move(built);
            _builtBank          = bank;
            _builtOrder         = next.order;
            _builtRolloff       = next.rolloff;
            _builtAttenuationDb = next.attenuationDb;
            if (held) { // the first bank drops no history and is not counted
                _nBankRebuilds.fetch_add(1ULL, std::memory_order_relaxed);
            }
        } else {
            if (const std::expected<void, gr::Error> grown = _line->growMaxDelay(staged.wholeDelaySamples); !grown) {
                throw gr::exception(std::format("RangeDelay: {}", grown.error().message), grown.error().sourceLocation);
            }
        }

        // An arm, or an armed anchor moved to another index or time, starts the line on silence.
        if (anchor.armed() && (!_anchor.armed() || anchor.anchorIndex() != _anchor.anchorIndex() || anchor.anchorNs() != _anchor.anchorNs())) {
            _line->reset();
        }
        _schedule = std::move(table);
        _accepted = next;
        _bias     = staged.bias;
        _anchor   = anchor;
        if (anchor.armed()) {
            _clock = *clock;
        }
        _latencyQ32 = static_cast<std::int64_t>(std::llround(_line->latencySamples() * static_cast<double>(gr::filter::kArbitraryOne)));
        _configured = true;
        publishMeasurements();
    }

    /// Publishes the samples before @p done and reports @p error as the call's result.
    [[nodiscard]] work::Status refuse(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, std::size_t done, const gr::Error& error) {
        std::ignore = inSpan.consume(done);
        outSpan.publish(done);
        publishMeasurements();
        this->emitErrorMessage("processBulk", error);
        return work::Status::ERROR;
    }

    /// The largest delay the fixed point carries at @p rate, in seconds, less one sample. The line's history is the
    /// whole part of the delay plus one. That history stays inside the same bound.
    [[nodiscard]] static double reachSeconds(float rate) noexcept { return static_cast<double>(gr::filter::kMaxFractionalDelaySamples - 1ULL) / static_cast<double>(rate); }

    /// The whole samples of history a commanded delay of @p seconds needs at @p rate. That is its whole part plus the
    /// sample the fraction reaches into. `reachSeconds()` bounds it, and every path checks that bound before this call.
    [[nodiscard]] static std::uint64_t wholeDelayFor(double seconds, float rate) noexcept { return static_cast<std::uint64_t>(std::max(0., std::floor(seconds * static_cast<double>(rate)) + 1.)); }

    /// The table the settings name, before the direction's sign, or none for a delay of the bias alone.
    [[nodiscard]] std::optional<gr::timing::DelaySchedule> loadTable() const {
        const std::vector<std::int64_t>& times  = schedule_times_ns.value;
        const std::vector<double>&       delays = schedule_delays_s.value;
        if (!schedule_file.value.empty() && !(times.empty() && delays.empty())) {
            throw gr::exception("RangeDelay: 'schedule_file' and 'schedule_times_ns'/'schedule_delays_s' are both set; set one");
        }
        if (!schedule_file.value.empty()) {
            auto loaded = gr::timing::loadTrajectoryFile(schedule_file.value);
            if (!loaded) {
                throw gr::exception(std::format("RangeDelay: 'schedule_file': {}", loaded.error().message));
            }
            if (!loaded->delay.has_value()) {
                throw gr::exception(std::format("RangeDelay: '{}' has no delay column; it names no range_m", schedule_file.value));
            }
            return std::move(loaded->delay);
        }
        if (times.empty() && delays.empty()) {
            return std::nullopt;
        }
        try {
            return gr::timing::DelaySchedule(std::span<const std::int64_t>(times), std::span<const double>(delays));
        } catch (const gr::exception& refusal) {
            throw gr::exception(std::format("RangeDelay: 'schedule_times_ns'/'schedule_delays_s': {}", refusal.message));
        }
    }

    /// The bias and the history @p table needs under @p next, or the refusal of a knot, a segment or the bias.
    [[nodiscard]] static Staged stage(const std::optional<gr::timing::DelaySchedule>& table, const detail::RangeDelaySettings& next) {
        const double reach = reachSeconds(next.rate);
        if (!table) {
            // Without a table the delay is the bias alone. The same reach and causality limits apply.
            Staged staged;
            staged.bias = std::isnan(next.bias) ? 0. : next.bias;
            if (!std::isfinite(staged.bias) || staged.bias < 0.) {
                throw gr::exception(std::format("RangeDelay: 'bias_s' is {} s with no table; set it finite and non-negative", staged.bias));
            }
            if (!(staged.bias < reach)) {
                throw gr::exception(std::format("RangeDelay: 'bias_s' is {} s, at or past the {} s the fixed-point delay carries at {} Hz", staged.bias, reach, static_cast<double>(next.rate)));
            }
            staged.wholeDelaySamples = wholeDelayFor(staged.bias, next.rate);
            return staged;
        }

        const double sign = next.direction == detail::DopplerDirection::Correct ? -1. : 1.;
        const double bias = resolveBias(*table, next);

        // The commanded delay is `sign * delay + bias`. It must stay in [0, the line's reach) at every knot. Its
        // slope must stay under one sample per sample, or the read position would run backwards.
        const auto knotTimes = table->times();
        const auto knotVals  = table->values();
        for (std::size_t i = 0UZ; i < knotVals.size(); ++i) {
            const double commanded = sign * knotVals[i] + bias;
            if (!(commanded >= 0.)) {
                throw gr::exception(std::format("RangeDelay: knot {} commands {} s; raise bias_s to make it non-negative", i, commanded));
            }
            if (!(commanded < reach)) {
                throw gr::exception(std::format("RangeDelay: knot {} commands {} s, at or past the {} s the fixed-point delay carries at {} Hz", i, commanded, reach, static_cast<double>(next.rate)));
            }
        }
        for (std::size_t i = 1UZ; i < knotVals.size(); ++i) {
            const double dtSeconds = static_cast<double>(knotTimes[i] - knotTimes[i - 1UZ]) * 1e-9;
            const double slope     = std::abs((knotVals[i] - knotVals[i - 1UZ]) / dtSeconds); // seconds of delay per second, dimensionless
            if (!(slope < 1.0)) {
                throw gr::exception(std::format("RangeDelay: segment {} changes the delay by {} s per second, at or past one sample per sample", i, slope));
            }
        }

        Staged staged;
        staged.bias = bias;
        // The line holds history for the largest delay this table commands. A schedule holds its end values. The
        // maximum therefore depends on the table alone and not on the stream.
        double worst = 0.;
        for (const double value : knotVals) {
            worst = std::max(worst, sign * value + bias);
        }
        staged.wholeDelaySamples = wholeDelayFor(worst, next.rate);
        return staged;
    }

    [[nodiscard]] static double resolveBias(const gr::timing::DelaySchedule& schedule, const detail::RangeDelaySettings& next) {
        if (!std::isnan(next.bias)) {
            if (!std::isfinite(next.bias)) {
                throw gr::exception(std::format("RangeDelay: 'bias_s' is {} s; set it finite, or NaN to derive it", next.bias));
            }
            if (next.direction == detail::DopplerDirection::Apply && next.bias < 0.) {
                throw gr::exception(std::format("RangeDelay: 'bias_s' is {} s under 'apply'; set it non-negative", next.bias));
            }
            return next.bias;
        }
        // The default keeps the commanded delay causal. `apply` is already non-negative and takes zero. `correct`
        // commands `-delay + bias`, least where the delay is greatest. The schedule's maximum lifts that least value
        // to zero.
        return next.direction == detail::DopplerDirection::Correct ? schedule.maxValue() : 0.;
    }

    void reserve(std::size_t nSamples) {
        // The buffers grow to the largest chunk and keep that size. A call allocates only for a chunk larger than
        // every earlier one.
        if (_delays.size() < nSamples) {
            _delays.resize(nSamples);
            _delaysQ32.resize(nSamples);
        }
    }

    /// @brief Fills `[at, at + input.size())` of the call's delays and runs the line over @p input.
    /// True where it filtered, or the line's error for a delay or span it refused.
    [[nodiscard]] std::expected<bool, gr::Error> runSegment(std::span<const T> input, std::span<T> output, std::size_t at) {
        const std::size_t nSamples = input.size();
        if (nSamples == 0UZ) {
            return false;
        }
        if (!_line || !_anchor.armed()) {
            // An unarmed anchor places no sample in time. The input passes through, and the line takes nothing.
            std::ranges::copy(input, output.begin());
            _position += nSamples;
            return false;
        }

        const std::span<double>        delays(_delays.data() + at, nSamples);
        const std::span<std::uint64_t> delaysQ32(_delaysQ32.data() + at, nSamples);
        if (_schedule) {
            _schedule->valuesFor(_clock, _position, delays);
            for (double& value : delays) {
                value = (_accepted.direction == detail::DopplerDirection::Correct ? -value : value) + _bias;
            }
        } else {
            std::ranges::fill(delays, _bias); // no table is a constant bias, which may be zero
        }
        if (const auto converted = gr::filter::fractionalDelayQ32(std::span<const double>(delays), static_cast<double>(_accepted.rate), delaysQ32); !converted) {
            return std::unexpected(converted.error());
        }
        if (const auto ran = _line->process(input, std::span<const std::uint64_t>(delaysQ32), output); !ran) {
            return std::unexpected(ran.error());
        }

        _position += nSamples;
        return true;
    }

    /// @brief Runs `[from, to)` of the call and attaches its tags to the output samples that carry their samples.
    [[nodiscard]] std::expected<void, gr::Error> advance(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, std::size_t from, std::size_t to) {
        if (to <= from) {
            return {};
        }
        const std::uint64_t firstIndex = _position;
        const auto          filtered   = runSegment(std::span<const T>(inSpan.data() + from, to - from), std::span<T>(outSpan.data() + from, to - from), from);
        if (!filtered) {
            return std::unexpected(filtered.error());
        }
        placeTags(outSpan, firstIndex, from, to - from, *filtered);
        return {};
    }

    /**
     * @brief Emits each waiting tag on the first output sample whose whole read position has reached its input index.
     *
     * The read position of output `k` is `k - lag`. The lag is the commanded delay plus the line's own, both in the
     * `Q32` format of the cursor. `floor((k << 32) - lag) >> 32 >= i` equals `(k - i) << 32 >= lag`. The comparison
     * below uses that integer form. The lag is non-negative, and the map is monotone. Tags keep their order. Two
     * tags that reach one output sample are both attached there, in input order.
     */
    void placeTags(OutputSpanLike auto& outSpan, std::uint64_t firstIndex, std::size_t outOffset, std::size_t nSamples, bool filtered) {
        if (_tagsMoving.empty() && _tagsHeld.empty()) {
            return;
        }
        std::size_t moved = 0UZ;
        std::size_t held  = 0UZ;
        for (std::size_t j = 0UZ; j < nSamples && (moved < _tagsMoving.size() || held < _tagsHeld.size()); ++j) {
            const std::uint64_t index = firstIndex + j;
            const std::int64_t  lag   = filtered ? (static_cast<std::int64_t>(_delaysQ32[outOffset + j]) + _latencyQ32) : 0LL;

            while (held < _tagsHeld.size() && _tagsHeld[held].first <= index) {
                outSpan.publishTag(_tagsHeld[held].second, outOffset + j);
                ++held;
            }

            std::size_t here = 0UZ;
            while (moved < _tagsMoving.size()) {
                const std::uint64_t marked = _tagsMoving[moved].first;
                if (index < marked) {
                    break;
                }
                const std::uint64_t ahead = index - marked;
                if (ahead < gr::filter::kMaxFractionalDelaySamples && (static_cast<std::int64_t>(ahead) << gr::filter::kArbitraryFractionBits) < lag) {
                    break;
                }
                outSpan.publishTag(_tagsMoving[moved].second, outOffset + j);
                ++moved;
                ++here;
            }
            if (here > 1UZ) {
                _nTagsCoalesced.fetch_add(1ULL, std::memory_order_relaxed);
            }
        }
        _tagsMoving.erase(_tagsMoving.begin(), _tagsMoving.begin() + static_cast<std::ptrdiff_t>(moved));
        _tagsHeld.erase(_tagsHeld.begin(), _tagsHeld.begin() + static_cast<std::ptrdiff_t>(held));
    }

    /// @brief True for a tag with any key of `gr::tag::kDefaultTags`.
    [[nodiscard]] static bool namesReservedKey(const property_map& map) noexcept {
        for (const auto& entry : map) {
            if (std::ranges::find(gr::tag::kDefaultTags, std::string_view(entry.first)) != gr::tag::kDefaultTags.end()) {
                return true;
            }
        }
        return false;
    }

    /// @brief Counts the gaps and the rates other than the accepted one that the reserved keys report.
    void countStreamKeys(const property_map& map) noexcept {
        if (map.find(property_map::key_type(gr::tag::N_DROPPED_SAMPLES.shortKey())) != map.end()) {
            _nDiscontinuities.fetch_add(1ULL, std::memory_order_relaxed);
        }
        const auto rate = map.find(property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()));
        if (rate == map.end()) {
            return;
        }
        const float* tagged   = rate->second.get_if<float>();
        const double accepted = static_cast<double>(_accepted.rate);
        if (tagged == nullptr || !(std::abs(static_cast<double>(*tagged) - accepted) <= 1e-9 * accepted)) {
            _nRateDisagreements.fetch_add(1ULL, std::memory_order_relaxed);
        }
    }

    [[nodiscard]] double delaySecondsAt(std::uint64_t index) const noexcept {
        if (!_schedule || !_anchor.armed()) {
            return _anchor.armed() ? _bias : 0.;
        }
        const double raw = _schedule->delayAt(_clock.timeOf(index));
        return (_accepted.direction == detail::DopplerDirection::Correct ? -raw : raw) + _bias;
    }

    [[nodiscard]] SchedulePosition positionAt(std::uint64_t index) const noexcept {
        if (!_anchor.armed()) {
            return SchedulePosition::Unarmed;
        }
        if (!_schedule) {
            return SchedulePosition::After;
        }
        const std::int64_t now = _clock.timeOf(index);
        if (now < _schedule->firstTime()) {
            return SchedulePosition::Before;
        }
        if (now > _schedule->lastTime()) {
            return SchedulePosition::After;
        }
        return SchedulePosition::Inside;
    }

    /// Publishes the current state for readers on other threads. Runs once per call.
    void publishMeasurements() noexcept {
        _armed.store(_anchor.armed(), std::memory_order_relaxed);
        _anchorIndex.store(_anchor.anchorIndex(), std::memory_order_relaxed);
        _anchorNs.store(_anchor.anchorNs(), std::memory_order_relaxed);
        _nIgnoredAnchors.store(_anchor.nIgnoredAnchors(), std::memory_order_relaxed);
        _nReanchors.store(_anchor.nReanchors(), std::memory_order_relaxed);
        _nRefusedAnchors.store(_anchor.nRefusedAnchors(), std::memory_order_relaxed);
        const double seconds = delaySecondsAt(_position);
        _slot.publish({seconds, seconds * static_cast<double>(_accepted.rate), _bias, _line ? _line->latencySamples() : 0., _line ? _line->groupDelaySamples() : 0., static_cast<double>(static_cast<std::uint8_t>(positionAt(_position))), static_cast<double>(_line ? _line->historySamples() : 0UZ)}, _position);
    }
};

} // namespace gr::blocks::channel

#endif // GNURADIO_CHANNEL_RANGE_DELAY_HPP
