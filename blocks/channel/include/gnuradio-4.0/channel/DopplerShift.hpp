#ifndef GNURADIO_CHANNEL_DOPPLER_SHIFT_HPP
#define GNURADIO_CHANNEL_DOPPLER_SHIFT_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
#include <expected>
#include <format>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/MeasurementSlot.hpp>
#include <gnuradio-4.0/algorithm/signal/Phasor.hpp>
#include <gnuradio-4.0/algorithm/timing/FrequencySchedule.hpp>
#include <gnuradio-4.0/algorithm/timing/SampleClock.hpp>
#include <gnuradio-4.0/algorithm/timing/ScheduleAnchor.hpp>
#include <gnuradio-4.0/algorithm/timing/TrajectoryFile.hpp>

namespace gr::blocks::channel {

/// Where the next sample stands against the schedule's time span. `Unarmed` means a trigger anchor has not seen
/// its trigger.
enum class SchedulePosition : std::uint8_t { Before = 0, Inside, After, Unarmed };

namespace detail {

/// The direction of the schedule. `apply` puts the trajectory's shift on a clean signal. `correct` removes it.
enum class DopplerDirection : std::uint8_t { Apply = 0, Correct };

/// The direction @p name selects. A refusal message starts with @p block.
[[nodiscard]] inline DopplerDirection parseDopplerDirection(std::string_view name, std::string_view block) {
    if (name == "apply") {
        return DopplerDirection::Apply;
    }
    if (name == "correct") {
        return DopplerDirection::Correct;
    }
    throw gr::exception(std::format("{}: 'direction' must be 'apply' or 'correct', got '{}'", block, name));
}

/// The name of @p seat, as the `direction` setting spells it.
[[nodiscard]] inline constexpr std::string_view dopplerDirectionName(DopplerDirection seat) noexcept { return seat == DopplerDirection::Apply ? "apply" : "correct"; }

/// The anchor mode @p name selects. A refusal names the three modes.
[[nodiscard]] inline gr::timing::AnchorSource parseAnchorSource(std::string_view name, std::string_view block) {
    const std::optional<gr::timing::AnchorSource> source = gr::timing::anchorSourceFrom(name);
    if (!source.has_value()) {
        throw gr::exception(std::format("{}: 'anchor_source' must be 'setting', 'first_trigger' or 'every_trigger', got '{}'", block, name));
    }
    return *source;
}

/// @brief The validator of `sample_rate`: true for a rate inside the range `gr::timing::clockForRateHz` takes.
inline constexpr auto kClockRateHz = [](float rate) {
    const double hz = static_cast<double>(rate);
    return hz >= gr::timing::SampleClock::kMinRateHz && hz <= gr::timing::SampleClock::kMaxRateHz;
};

/// The anchor settings of a block, as last accepted.
struct AnchorSettings {
    gr::timing::AnchorSource source{gr::timing::AnchorSource::setting};
    std::uint64_t            index{0ULL};
    std::int64_t             ns{0LL};
    bool                     honorOffset{true};
};

/// The limits of `sample_rate`.
using ClockRateLimits = Limits<0.f, std::numeric_limits<float>::max(), kClockRateHz>;

/// The limits of `anchor_ns`, the bound `gr::timing::ScheduleAnchor` holds.
using AnchorNsLimits = Limits<-gr::timing::ScheduleAnchor::kMaxAnchorNs, gr::timing::ScheduleAnchor::kMaxAnchorNs>;

/// The `Doc` texts of the settings that blocks reading a trajectory table share.
inline constexpr gr::meta::fixed_string kScheduleTimesDoc      = "strictly increasing knot times on the SampleClock axis, two or more";
inline constexpr gr::meta::fixed_string kDirectionDoc          = "schedule direction, 'apply' or 'correct'";
inline constexpr gr::meta::fixed_string kSampleRateDoc         = "stream sample rate";
inline constexpr gr::meta::fixed_string kAnchorIndexDoc        = "the stream sample that anchor_ns belongs to, under anchor_source 'setting'";
inline constexpr gr::meta::fixed_string kAnchorNsDoc           = "schedule time of anchor_index under anchor_source 'setting'";
inline constexpr gr::meta::fixed_string kAnchorSourceDoc       = "'setting', 'first_trigger' or 'every_trigger'";
inline constexpr gr::meta::fixed_string kHonorTriggerOffsetDoc = "move a trigger anchor by the tag's own trigger_offset seconds";

/// The table settings a block last accepted. A change loads the table again when it names one of them.
struct TableSettings {
    std::vector<std::int64_t> times{};
    std::vector<double>       values{};
    std::string               file{};

    [[nodiscard]] bool operator==(const TableSettings&) const = default;
};

/// @brief True when a change moves @p key. Before the first accepted change every key moves. After it, a key moves
/// when @p newSettings holds it or when @p differs says its member is not the value last accepted.
[[nodiscard]] inline bool moves(const property_map& newSettings, bool configured, std::string_view key, bool differs) { return !configured || differs || newSettings.contains(std::pmr::string(key)); }

/// A trigger tag's time and offset, read with the reserved keys' types. A tag without a time is no anchor event.
struct TriggerRead {
    std::uint64_t timeNs{0ULL};
    float         offsetSeconds{0.f};
    bool          present{false};  ///< the tag carries a `trigger_time` key
    bool          readable{false}; ///< the key, and `trigger_offset` when honored, carry the reserved types
};

/// @brief Reads a tag's `trigger_time` and, when @p honorOffset is set, its `trigger_offset`. A key of a type other
/// than the reserved one leaves `readable` false.
[[nodiscard]] inline TriggerRead readTrigger(const property_map& map, bool honorOffset) {
    TriggerRead read;
    const auto  time = map.find(property_map::key_type(gr::tag::TRIGGER_TIME.shortKey()));
    if (time == map.end()) {
        return read;
    }
    read.present                = true;
    const std::uint64_t* timeNs = time->second.get_if<std::uint64_t>();
    if (timeNs == nullptr) {
        return read;
    }
    read.timeNs   = *timeNs;
    read.readable = true;
    if (honorOffset) {
        if (const auto offset = map.find(property_map::key_type(gr::tag::TRIGGER_OFFSET.shortKey())); offset != map.end()) {
            const float* seconds = offset->second.get_if<float>();
            if (seconds == nullptr) {
                read.readable = false;
                return read;
            }
            read.offsetSeconds = *seconds;
        }
    }
    return read;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::channel::DopplerShift, [T], [std::complex<float>])

template<typename T>
requires std::is_same_v<T, std::complex<float>>
struct DopplerShift : gr::Block<DopplerShift<T>> {
    using Description = Doc<R""(
@brief Applies a trajectory's Doppler frequency shift to a complex stream, or removes it.

In a channel model the block puts a moving source's carrier shift on a clean signal. Ahead of a demodulator it takes
the shift off. Each sample is mixed with a phasor whose phase increment is the integral of a frequency schedule over
that sample's interval. The schedule is linear between its `(time, offset)` knots and holds its end values outside
them. `gr::timing::offsetFor(v_radial, f_carrier)` converts a radial velocity to an offset, and a closing source
reads high. One sample comes out for each sample in.

The knots come from `schedule_times_ns` and `schedule_offsets_hz`, or from the `offset_hz` or `range_rate_m_s` column of
a `#!gr4-trajectory 1` file named in `schedule_file`. Setting both is refused. With neither, the stream passes through.
Every offset must stay under half of `sample_rate` in magnitude, and the refusal names the knot. `direction` 'apply'
adds the offsets, and 'correct' subtracts them. A new table, direction or rate takes effect at the next sample and keeps
the phase. A new rate re-times the whole stream from the anchor.

`anchor_source` selects what ties the schedule's time axis to the stream. Under 'setting', sample `anchor_index` is at
time `anchor_ns`. Under 'first_trigger', the first `trigger_time` tag sets the anchor at its own sample, and later tags
are ignored. Under 'every_trigger', each tag moves the anchor. Every tag that arms or moves the anchor restarts the
phase at zero. `honor_trigger_offset` adds the tag's `trigger_offset` seconds to its time. Until a trigger anchor is
armed, the stream passes through unshifted. A change to one of these four settings rebuilds the anchor and clears its
counters, and a trigger anchor then waits for its next tag. Every other change keeps the anchor.
`gr::timing::ScheduleAnchor` holds these rules.

A `frequency` tag passes unchanged. `currentOffsetHz()` reads the offset at the next sample, `schedulePosition()`
where that sample stands against the knots, and `position()` its index from the stream's start. `nIgnoredAnchors()`
counts the trigger tags the mode ignores, `nReanchors()` the moves under 'every_trigger', and `nRefusedAnchors()` the
tags whose time is unreadable or past the anchor's bound. Each reads from any thread.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<std::vector<std::int64_t>, "schedule_times_ns", Visible, Unit<"ns">, Doc<detail::kScheduleTimesDoc>>                                schedule_times_ns{};
    Annotated<std::vector<double>, "schedule_offsets_hz", Visible, Unit<"Hz">, Doc<"knot offsets, one per time, held at the end values outside">> schedule_offsets_hz{};
    Annotated<std::string, "schedule_file", Doc<"#!gr4-trajectory 1 file with a frequency column">>                                               schedule_file{};
    Annotated<std::string, "direction", Visible, Doc<detail::kDirectionDoc>>                                                                      direction            = std::string("apply");
    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<detail::kSampleRateDoc>, detail::ClockRateLimits>                                    sample_rate          = 1.f;
    Annotated<std::uint64_t, "anchor_index", Doc<detail::kAnchorIndexDoc>>                                                                        anchor_index         = 0ULL;
    Annotated<std::int64_t, "anchor_ns", Unit<"ns">, Doc<detail::kAnchorNsDoc>, detail::AnchorNsLimits>                                           anchor_ns            = 0LL;
    Annotated<std::string, "anchor_source", Doc<detail::kAnchorSourceDoc>>                                                                        anchor_source        = std::string("setting");
    Annotated<bool, "honor_trigger_offset", Doc<detail::kHonorTriggerOffsetDoc>>                                                                  honor_trigger_offset = true;

    GR_MAKE_REFLECTABLE(DopplerShift, in, out, schedule_times_ns, schedule_offsets_hz, schedule_file, direction, sample_rate, anchor_index, anchor_ns, anchor_source, honor_trigger_offset);

    // The accepted settings. A refused change leaves all of them as they were.
    std::shared_ptr<const gr::timing::FrequencySchedule> _table{};         ///< the knots as configured, before the direction's sign
    std::shared_ptr<const gr::timing::FrequencySchedule> _schedule{};      ///< the knots in the direction's sign
    detail::TableSettings                                _tableSettings{}; ///< the table settings as last accepted
    detail::DopplerDirection                             _direction{detail::DopplerDirection::Apply};
    float                                                _rate{1.f};
    detail::AnchorSettings                               _anchorSettings{};
    bool                                                 _configured{false}; ///< a settings change has been accepted

    gr::timing::SampleClock    _clock{};
    gr::timing::ScheduleAnchor _anchor{};
    gr::signal::Phasor<float>  _phasor{};
    std::vector<double>        _increments{};   ///< increments for one call, resized only upward
    std::uint64_t              _position{0ULL}; ///< index of the next sample, counted from the stream's own start

    // The observables, copied for a reader on another thread. The doubles go through the slot. A nanosecond count
    // since the epoch is past the exact integer range of a double, and the anchor time has its own atomic.
    gr::measurement::MeasurementSlot<2UZ> _slot{}; ///< offset in Hz and schedule position, with the stream position as fill count
    std::atomic<bool>                     _armed{true};
    std::atomic<std::uint64_t>            _anchorIndex{0ULL};
    std::atomic<std::int64_t>             _anchorNs{0LL};
    std::atomic<std::uint64_t>            _nIgnoredAnchors{0ULL};
    std::atomic<std::uint64_t>            _nReanchors{0ULL};
    std::atomic<std::uint64_t>            _nRefusedAnchors{0ULL};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const auto moved = [&](std::string_view key, bool differs) { return detail::moves(newSettings, _configured, key, differs); };

        const float                    rate = moved("sample_rate", sample_rate.value != _rate) ? sample_rate.value : _rate;
        const detail::DopplerDirection seat = moved("direction", direction.value != detail::dopplerDirectionName(_direction)) ? detail::parseDopplerDirection(direction.value, "DopplerShift") : _direction;
        const detail::TableSettings    tableNamed{schedule_times_ns.value, schedule_offsets_hz.value, schedule_file.value};
        const bool                     tableDiffers = tableNamed != _tableSettings;
        const bool                     tableMoved   = moved("schedule_times_ns", tableDiffers) || moved("schedule_offsets_hz", tableDiffers) || moved("schedule_file", tableDiffers);
        const auto                     table        = tableMoved ? loadTable() : _table;
        const bool                     anchorMoved  = moved("anchor_source", anchor_source.value != gr::timing::anchorSourceName(_anchorSettings.source)) || moved("anchor_index", anchor_index.value != _anchorSettings.index) || moved("anchor_ns", anchor_ns.value != _anchorSettings.ns) || moved("honor_trigger_offset", honor_trigger_offset.value != _anchorSettings.honorOffset);
        detail::AnchorSettings         anchorConfig = _anchorSettings;
        if (anchorMoved) {
            anchorConfig = {detail::parseAnchorSource(anchor_source.value, "DopplerShift"), anchor_index.value, anchor_ns.value, honor_trigger_offset.value};
        }
        if (table) {
            requireBelowNyquist(table->offsets(), rate);
        }

        const gr::timing::ScheduleAnchor anchor = anchorMoved ? gr::timing::ScheduleAnchor(anchorConfig.source, anchorConfig.index, anchorConfig.ns, anchorConfig.honorOffset) : _anchor;
        // An unarmed anchor keeps the old clock. The rate is still checked against the time axis.
        const auto clock = clockAt(rate, anchor.armed() ? anchor.anchorIndex() : 0ULL, anchor.armed() ? anchor.anchorNs() : 0LL);
        if (!clock) {
            throw gr::exception(clock.error().message, clock.error().sourceLocation);
        }

        _schedule       = (tableMoved || seat != _direction || !_configured) ? signedSchedule(table, seat) : _schedule;
        _table          = table;
        _tableSettings  = tableNamed;
        _direction      = seat;
        _rate           = rate;
        _anchorSettings = anchorConfig;
        _anchor         = anchor;
        if (anchor.armed()) {
            _clock = *clock;
        }
        _configured = true;
        publishMeasurements();
    }

    void start() {
        _position = 0ULL;
        _phasor.setPhase(0.);
        _anchor.reset(); // a trigger anchor waits for the new stream's trigger
        publishMeasurements();
    }

    /// @brief The offset in Hz the block applies to the stream now, negated in `correct`. Any thread.
    [[nodiscard]] double currentOffsetHz() const noexcept { return _slot.read().first[0]; }

    /// @brief The position of the next sample against the schedule's span. `Before` and `After` hold an end value.
    [[nodiscard]] SchedulePosition schedulePosition() const noexcept { return static_cast<SchedulePosition>(static_cast<std::uint8_t>(_slot.read().first[1])); }

    /// @brief The next sample's index counted from the stream's start, which the anchor places in time.
    [[nodiscard]] std::uint64_t position() const noexcept { return _slot.read().second; }

    [[nodiscard]] bool          anchorArmed() const noexcept { return _armed.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t anchorIndex() const noexcept { return _anchorIndex.load(std::memory_order_relaxed); }
    [[nodiscard]] std::int64_t  anchorNs() const noexcept { return _anchorNs.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nIgnoredAnchors() const noexcept { return _nIgnoredAnchors.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nReanchors() const noexcept { return _nReanchors.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nRefusedAnchors() const noexcept { return _nRefusedAnchors.load(std::memory_order_relaxed); }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        const std::size_t  nSamples = std::min(input.size(), output.size());
        const std::span<T> written  = output.first(nSamples);

        if (!_schedule || !_anchor.armed()) {
            std::ranges::copy(input.first(nSamples), written.begin());
            _position += nSamples;
            publishMeasurements();
            return work::Status::OK;
        }

        // The buffer grows to the largest chunk and keeps that size. A call allocates only when its chunk is
        // larger than every earlier one.
        if (_increments.size() < nSamples) {
            _increments.resize(nSamples);
        }
        const std::span<double> increments(_increments.data(), nSamples);
        _schedule->phaseIncrementsFor(_clock, _position, increments);
        _phasor.mixModulated(std::span<const double>(increments), input.first(nSamples), written);

        _position += nSamples;
        publishMeasurements();
        return work::Status::OK;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t nSamples = std::min(inSpan.size(), outSpan.size());

        // Every mode reads the trigger tags, and the anchor counts the ones it ignores.
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
            if (at > done) { // the call splits at the trigger, and the samples before it keep their anchor
                std::ignore = processBulk(std::span<const T>(inSpan.data() + done, at - done), std::span<T>(outSpan.data() + done, at - done));
                done        = at;
            }

            const gr::timing::ScheduleAnchor::Response response = read.readable ? _anchor.onTrigger(_position, read.timeNs, read.offsetSeconds) : _anchor.onUnreadableTrigger();
            if (response == gr::timing::ScheduleAnchor::Response::armed || response == gr::timing::ScheduleAnchor::Response::reanchored) {
                const auto clock = clockAt(_rate, _anchor.anchorIndex(), _anchor.anchorNs());
                if (!clock) {
                    std::ignore = inSpan.consume(done);
                    outSpan.publish(done);
                    this->emitErrorMessage("processBulk", clock.error());
                    return work::Status::ERROR;
                }
                _clock = *clock;
                // Each anchor restarts the phase at zero. The output after a trigger then does not depend on the
                // stream before it.
                _phasor.setPhase(0.);
            }
        }
        if (done < nSamples) {
            std::ignore = processBulk(std::span<const T>(inSpan.data() + done, nSamples - done), std::span<T>(outSpan.data() + done, nSamples - done));
        }

        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        publishMeasurements(); // a call whose last segment ended on a tag has published nothing since that tag
        return work::Status::OK;
    }

private:
    /// The schedule's time axis at @p rate, with sample @p index at @p timeNs.
    [[nodiscard]] static std::expected<gr::timing::SampleClock, gr::Error> clockAt(float rate, std::uint64_t index, std::int64_t timeNs) {
        return gr::timing::clockForRateHz(static_cast<double>(rate), index, timeNs).transform_error([](const gr::Error& refusal) { return gr::Error(std::format("DopplerShift: 'sample_rate': {}", refusal.message), refusal.sourceLocation); });
    }

    /// The table the settings name, before the direction's sign, or none for a stream that passes through.
    [[nodiscard]] std::shared_ptr<const gr::timing::FrequencySchedule> loadTable() const {
        const std::vector<std::int64_t>& times   = schedule_times_ns.value;
        const std::vector<double>&       offsets = schedule_offsets_hz.value;
        if (!schedule_file.value.empty() && !(times.empty() && offsets.empty())) {
            throw gr::exception("DopplerShift: 'schedule_file' and 'schedule_times_ns'/'schedule_offsets_hz' are both set; set one");
        }
        if (!schedule_file.value.empty()) {
            auto loaded = gr::timing::loadTrajectoryFile(schedule_file.value);
            if (!loaded) {
                throw gr::exception(std::format("DopplerShift: 'schedule_file': {}", loaded.error().message));
            }
            if (!loaded->frequency.has_value()) {
                throw gr::exception(std::format("DopplerShift: '{}' has no frequency column; it names neither offset_hz nor range_rate_m_s", schedule_file.value));
            }
            return std::make_shared<const gr::timing::FrequencySchedule>(std::move(*loaded->frequency));
        }
        if (times.empty() && offsets.empty()) {
            return nullptr;
        }
        return std::make_shared<const gr::timing::FrequencySchedule>(std::span<const std::int64_t>(times), std::span<const double>(offsets));
    }

    /// Refuses an offset at or past half of @p rate, naming its knot.
    static void requireBelowNyquist(std::span<const double> offsets, float rate) {
        const double limit = 0.5 * static_cast<double>(rate);
        for (std::size_t i = 0UZ; i < offsets.size(); ++i) {
            if (std::abs(offsets[i]) >= limit) {
                throw gr::exception(std::format("DopplerShift: knot {} offset {} Hz is at or past the {} Hz limit of a {} Hz stream", i, offsets[i], limit, static_cast<double>(rate)));
            }
        }
    }

    /// @p table in the sign of @p seat. 'correct' negates it once here, and the sample path is the same in both.
    [[nodiscard]] static std::shared_ptr<const gr::timing::FrequencySchedule> signedSchedule(const std::shared_ptr<const gr::timing::FrequencySchedule>& table, detail::DopplerDirection seat) {
        if (!table || seat == detail::DopplerDirection::Apply) {
            return table;
        }
        std::vector<double> negated(table->offsets().begin(), table->offsets().end());
        for (double& value : negated) {
            value = -value;
        }
        return std::make_shared<const gr::timing::FrequencySchedule>(table->times(), std::span<const double>(negated));
    }

    /// The schedule's offset at sample @p index, in the direction's sign. An unarmed block or one without a table
    /// shifts nothing.
    [[nodiscard]] double offsetAt(std::uint64_t index) const noexcept { return _schedule && _anchor.armed() ? _schedule->offsetAt(_clock.timeOf(index)) : 0.; }

    /// Where sample @p index stands against the schedule's span.
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

    /// Publishes the block's current state for a reader on another thread. Runs once per call.
    void publishMeasurements() noexcept {
        _armed.store(_anchor.armed(), std::memory_order_relaxed);
        _anchorIndex.store(_anchor.anchorIndex(), std::memory_order_relaxed);
        _anchorNs.store(_anchor.anchorNs(), std::memory_order_relaxed);
        _nIgnoredAnchors.store(_anchor.nIgnoredAnchors(), std::memory_order_relaxed);
        _nReanchors.store(_anchor.nReanchors(), std::memory_order_relaxed);
        _nRefusedAnchors.store(_anchor.nRefusedAnchors(), std::memory_order_relaxed);
        _slot.publish({offsetAt(_position), static_cast<double>(static_cast<std::uint8_t>(positionAt(_position)))}, _position);
    }
};

} // namespace gr::blocks::channel

#endif // GNURADIO_CHANNEL_DOPPLER_SHIFT_HPP
