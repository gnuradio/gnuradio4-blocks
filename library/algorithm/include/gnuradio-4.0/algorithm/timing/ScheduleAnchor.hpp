#ifndef GNURADIO_ALGORITHM_SCHEDULE_ANCHOR_HPP
#define GNURADIO_ALGORITHM_SCHEDULE_ANCHOR_HPP

#include <cmath>
#include <cstdint>
#include <optional>
#include <string_view>

#include <gnuradio-4.0/algorithm/timing/SampleClock.hpp>

namespace gr::timing {

/// Where a schedule's time origin comes from.
enum class AnchorSource : std::uint8_t {
    setting,       ///< the configured index and time, with every trigger tag ignored and counted
    first_trigger, ///< set by the first trigger tag, with every later one ignored and counted
    every_trigger  ///< every trigger tag re-establishes it, with the discontinuity declared
};

[[nodiscard]] inline constexpr std::string_view anchorSourceName(AnchorSource source) noexcept {
    switch (source) {
    case AnchorSource::setting: return "setting";
    case AnchorSource::first_trigger: return "first_trigger";
    case AnchorSource::every_trigger: return "every_trigger";
    }
    return "";
}

/// @brief The source a setting's text names, or `nullopt` for any other text.
[[nodiscard]] inline constexpr std::optional<AnchorSource> anchorSourceFrom(std::string_view name) noexcept {
    if (name == "setting") {
        return AnchorSource::setting;
    }
    if (name == "first_trigger") {
        return AnchorSource::first_trigger;
    }
    if (name == "every_trigger") {
        return AnchorSource::every_trigger;
    }
    return std::nullopt;
}

/**
 * @brief Places a stream's sample indices on a schedule's time axis, from settings or from `trigger_time` tags.
 *
 * A block that reads a time schedule holds one anchor and feeds it each `trigger_time` tag with the tag's absolute
 * stream index. The anchor is a stream index and the schedule time of that index in nanoseconds. `clock()` joins it
 * with a sample rate into a `SampleClock`. Until the anchor is armed, `armed()` is false and the stream has no time on
 * the schedule.
 *
 * The source sets what a tag does. Under `setting`, the configured index and time are the anchor from the start, and
 * every tag is ignored and counted in `nIgnoredAnchors()`. Under `first_trigger`, the first tag arms the anchor at the
 * tag's index and time. Every later tag is ignored and counted in `nIgnoredAnchors()`. Under `every_trigger`, the first
 * tag arms the anchor and each later tag moves it. `onTrigger()` returns `reanchored` for a move, and `nReanchors()`
 * counts it. A move is a jump in schedule time, and the caller restarts any phase or history it holds.
 *
 * Two rules hold in all three modes. `trigger_time` is a `std::uint64_t` count of nanoseconds since the Unix epoch, and
 * the anchor time is a `std::int64_t` within `kMaxAnchorNs` of the epoch, 2^62 ns or about 146 years. A sample within
 * that span of the anchor then has a time on the signed axis. The constructor takes the configured time as given, and
 * the caller holds it within the bound. A tag's time past the bound is refused and counted in `nRefusedAnchors()`, and
 * `onUnreadableTrigger()` counts a `trigger_time` of another type there too. With
 * `honorTriggerOffset` set, the anchor time is `trigger_time + llround(trigger_offset * 1e9)`, where `trigger_offset` is
 * a `float` in seconds. A non-finite offset, or one that moves the time past the bound, is refused and counted in
 * `nRefusedAnchors()`. A `float` offset resolves to under a nanosecond below 15.6 ms, and to 119 ns at 1 s.
 */
class ScheduleAnchor {
public:
    /// The result of a fed tag. `reanchored` means the anchor moved.
    enum class Response : std::uint8_t { ignored, armed, reanchored, refused };

    /// The largest anchor time in either direction, the bound `SampleClock::kMaxAnchorNs` names.
    static constexpr std::int64_t kMaxAnchorNs = SampleClock::kMaxAnchorNs;

    /// The largest `trigger_time` the anchor takes.
    static constexpr std::uint64_t kMaxTriggerTimeNs = static_cast<std::uint64_t>(kMaxAnchorNs);

    constexpr ScheduleAnchor() noexcept = default;

    constexpr ScheduleAnchor(AnchorSource source, std::uint64_t anchorIndex, std::int64_t anchorNs, bool honorTriggerOffset = true) noexcept //
        : _source(source), _index(anchorIndex), _ns(anchorNs), _honorOffset(honorTriggerOffset), _armed(source == AnchorSource::setting), _settingIndex(anchorIndex), _settingNs(anchorNs) {}

    /// @brief Restores the configured anchor and clears the three counters.
    constexpr void reset() noexcept {
        _index    = _settingIndex;
        _ns       = _settingNs;
        _armed    = _source == AnchorSource::setting;
        _ignored  = 0ULL;
        _reanchor = 0ULL;
        _refused  = 0ULL;
    }

    /**
     * @brief Feed one `trigger_time` tag at its absolute stream index and return the rule's result.
     *
     * @param streamIndex the tag's absolute index in the stream
     * @param triggerTimeNs the reserved `trigger_time` value, nanoseconds since the Unix epoch
     * @param triggerOffsetSeconds the reserved `trigger_offset` value, ignored unless `honorTriggerOffset()`
     */
    Response onTrigger(std::uint64_t streamIndex, std::uint64_t triggerTimeNs, float triggerOffsetSeconds = 0.f) noexcept {
        const std::optional<std::int64_t> when = anchorNsFor(triggerTimeNs, _honorOffset ? triggerOffsetSeconds : 0.f);
        if (!when.has_value()) {
            ++_refused;
            return Response::refused;
        }
        if (_source == AnchorSource::setting || (_source == AnchorSource::first_trigger && _armed)) {
            ++_ignored;
            return Response::ignored;
        }

        const bool wasArmed = _armed;
        _index              = streamIndex;
        _ns                 = *when;
        _armed              = true;
        if (wasArmed) {
            ++_reanchor;
            return Response::reanchored;
        }
        return Response::armed;
    }

    /// @brief Counts a `trigger_time` that is not a `std::uint64_t` in `nRefusedAnchors()` and leaves the anchor as it is.
    Response onUnreadableTrigger() noexcept {
        ++_refused;
        return Response::refused;
    }

    /// @brief The anchor time a tag gives, `trigger_time` plus the offset, or `nullopt` for a tag `onTrigger()` refuses.
    [[nodiscard]] static std::optional<std::int64_t> anchorNsFor(std::uint64_t triggerTimeNs, float triggerOffsetSeconds) noexcept {
        if (triggerTimeNs > kMaxTriggerTimeNs) {
            return std::nullopt;
        }
        if (!std::isfinite(triggerOffsetSeconds)) {
            return std::nullopt;
        }

        // `llround` is undefined past the integer range. The seconds are therefore bounded before the call.
        // 2^63 ns is 9.223e18, and a `double` holds that bound exactly.
        constexpr double kMaxNs  = 9.2233720368547758e18;
        const double     shiftNs = static_cast<double>(triggerOffsetSeconds) * 1e9;
        if (!(std::abs(shiftNs) < kMaxNs)) {
            return std::nullopt;
        }

        // The base is at most 2^62, and neither bound below leaves the axis.
        const std::int64_t base  = static_cast<std::int64_t>(triggerTimeNs);
        const std::int64_t shift = std::llround(shiftNs);
        if (shift > kMaxAnchorNs - base || shift < -kMaxAnchorNs - base) {
            return std::nullopt;
        }
        return base + shift;
    }

    [[nodiscard]] constexpr AnchorSource  source() const noexcept { return _source; }
    [[nodiscard]] constexpr bool          honorTriggerOffset() const noexcept { return _honorOffset; }
    [[nodiscard]] constexpr bool          armed() const noexcept { return _armed; }
    [[nodiscard]] constexpr std::uint64_t anchorIndex() const noexcept { return _index; }
    [[nodiscard]] constexpr std::int64_t  anchorNs() const noexcept { return _ns; }
    [[nodiscard]] constexpr std::uint64_t nIgnoredAnchors() const noexcept { return _ignored; }
    [[nodiscard]] constexpr std::uint64_t nReanchors() const noexcept { return _reanchor; }
    [[nodiscard]] constexpr std::uint64_t nRefusedAnchors() const noexcept { return _refused; }

    /// @brief The `SampleClock` at `rateNum/rateDen` samples per second that starts at this anchor.
    [[nodiscard]] SampleClock clock(std::uint64_t rateNum, std::uint64_t rateDen) const { return SampleClock(_index, _ns, rateNum, rateDen); }

private:
    AnchorSource  _source{AnchorSource::setting};
    std::uint64_t _index{0ULL};
    std::int64_t  _ns{0LL};
    bool          _honorOffset{true};
    bool          _armed{true};
    std::uint64_t _ignored{0ULL};
    std::uint64_t _reanchor{0ULL};
    std::uint64_t _refused{0ULL};
    std::uint64_t _settingIndex{0ULL};
    std::int64_t  _settingNs{0LL};
};

} // namespace gr::timing

#endif // GNURADIO_ALGORITHM_SCHEDULE_ANCHOR_HPP
