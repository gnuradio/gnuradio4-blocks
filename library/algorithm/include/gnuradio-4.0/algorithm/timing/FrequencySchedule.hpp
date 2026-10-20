#ifndef GNURADIO_ALGORITHM_FREQUENCY_SCHEDULE_HPP
#define GNURADIO_ALGORITHM_FREQUENCY_SCHEDULE_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <numbers>
#include <span>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/algorithm/timing/SampleClock.hpp>

namespace gr::timing {

/// The speed of light in vacuum, exact, because the meter is defined from it.
inline constexpr double kSpeedOfLight = 299'792'458.;

/**
 * @brief The Doppler offset a radial velocity puts on a carrier, `-f_carrier * v_radial / c`.
 *
 * `v_radial` is the rate of change of range. A closing pass has a negative `v_radial` and a positive offset. An
 * approaching transmitter reads high.
 *
 * This function converts a trajectory's velocity into a schedule's frequency. Orbit propagation, TLEs and station
 * geometry belong to the code that produced the velocity. The schedule takes frequencies.
 */
[[nodiscard]] constexpr double offsetFor(double vRadialMetersPerSecond, double fCarrierHz) noexcept { return -fCarrierHz * vRadialMetersPerSecond / kSpeedOfLight; }

/**
 * @brief The one-way propagation delay of a slant range, `range / c`, in seconds.
 *
 * This is the companion of `offsetFor`. A trajectory shifts the carrier by the offset and moves the envelope by
 * the delay. Both functions are in this file and share one definition of `c`.
 */
[[nodiscard]] constexpr double delayFor(double rangeMeters) noexcept { return rangeMeters / kSpeedOfLight; }

/// @brief How a schedule names itself and its values when it refuses a table.
struct ScheduleNaming {
    std::string_view type;     ///< the fully qualified class, as it opens every message
    std::string_view plural;   ///< the values, plural, for the pairing refusal
    std::string_view singular; ///< the value, singular, for the per-knot refusal
    std::string_view finite;   ///< what a finite value of this kind is called
};

/**
 * @brief An immutable table of knots on a `SampleClock`'s time axis, read piecewise linearly.
 *
 * A curve arrives as a few points and must become one number per sample. The table holds `(t_ns, value)`
 * knots, strictly increasing in time. The value between two knots lies on the straight line between them.
 * Outside the table the end values **hold**. The schedule does not extrapolate past its last knot. It keeps
 * the last value it was given.
 *
 * The segment walk uses a cursor and not a search. Sample times only increase. The cursor therefore advances at
 * most once per knot over a whole span. The per-sample cost does not depend on the number of knots. A benchmark
 * measures this at 100, 1000 and 10000 knots. A per-sample binary search would make a dense table cost more per
 * sample than a sparse one for the same curve.
 *
 * The table is a value. Nothing here is mutable, and nothing is carried between calls. `valuesFor` and
 * `phaseIncrementsFor` give each sample a pure function of its absolute index. One call over a span and a hundred
 * calls give bit-identical results. The caller decides what to do with a schedule, such as its sign, the refusal of
 * an out-of-range value or the swap of one table for another. The two derived schedules differ only in the meaning
 * of their values and in the one output form each adds. They share the knot table, the validation and the cursor.
 * The two cannot diverge in those parts. The constructor throws `gr::exception` for a table it refuses.
 */
class PiecewiseLinearSchedule {
public:
    /// Two knots make a line. With fewer there is nothing to interpolate. The ceiling on the knot count leaves
    /// room for a caller that wants a smoother curve than piecewise-linear to supply denser knots.
    static constexpr std::size_t kMinKnots = 2UZ;
    static constexpr std::size_t kMaxKnots = std::size_t{1} << 20;

    [[nodiscard]] std::size_t                   size() const noexcept { return _times.size(); }
    [[nodiscard]] std::span<const std::int64_t> times() const noexcept { return _times; }
    [[nodiscard]] std::span<const double>       values() const noexcept { return _values; }
    [[nodiscard]] std::int64_t                  firstTime() const noexcept { return _times.front(); }
    [[nodiscard]] std::int64_t                  lastTime() const noexcept { return _times.back(); }

    /// The value at a time, linear between knots and held at the end values outside the table.
    [[nodiscard]] double valueAt(std::int64_t t_ns) const noexcept { return evaluate(locate(t_ns), t_ns); }

    /**
     * @brief The slope, in value units per second, of the segment @p t_ns falls in.
     *
     * The function returns the slope of the segment that holds the time. At an inner knot that is the segment that
     * starts there. Outside the table it returns the slope of the end segment and not the zero slope of the held
     * value.
     */
    [[nodiscard]] double segmentSlopePerSecond(std::int64_t t_ns) const noexcept { return _slopes[locate(t_ns)] * 1e9; }

    /**
     * @brief The value at each of `out.size()` samples starting at `firstIndex` on `clock`.
     *
     * This is the point-evaluation counterpart of `FrequencySchedule::phaseIncrementsFor`. It walks the same
     * `NsWalk` cursor and evaluates the line at each sample's `timeOf`. Each sample costs a few compares and one
     * multiply-add. Values between knots can differ in their last bits between a build that fuses the multiply and
     * the add and one that does not. Within one build they are bit-identical.
     */
    void valuesFor(const SampleClock& clock, std::uint64_t firstIndex, std::span<double> out) const noexcept {
        if (out.empty()) {
            return;
        }

        SampleClock::NsWalk walk    = clock.walkFrom(firstIndex);
        std::size_t         segment = locate(walk.t_ns);
        for (double& value : out) {
            while (segment + 2UZ < _times.size() && _times[segment + 1UZ] <= walk.t_ns) {
                ++segment;
            }
            value = evaluate(segment, walk.t_ns);
            walk.advance();
        }
    }

protected:
    PiecewiseLinearSchedule(std::span<const std::int64_t> timesNs, std::span<const double> values, const ScheduleNaming& naming) : _times(timesNs.begin(), timesNs.end()), _values(values.begin(), values.end()) {
        if (timesNs.size() != values.size()) {
            throw gr::exception(std::format("{}: {} times against {} {} — the two must be paired", naming.type, timesNs.size(), values.size(), naming.plural));
        }
        if (_times.size() < kMinKnots || _times.size() > kMaxKnots) {
            throw gr::exception(std::format("{}: {} knots — the table holds between {} and {}", naming.type, _times.size(), kMinKnots, kMaxKnots));
        }
        for (std::size_t i = 0UZ; i < _values.size(); ++i) {
            if (!std::isfinite(_values[i])) {
                throw gr::exception(std::format("{}: knot {} has {} {}, which is not {}", naming.type, i, naming.singular, _values[i], naming.finite));
            }
        }
        for (std::size_t i = 1UZ; i < _times.size(); ++i) {
            if (_times[i] <= _times[i - 1UZ]) {
                throw gr::exception(std::format("{}: knot {} is at {} ns, at or before knot {} at {} ns — times must strictly increase", naming.type, i, _times[i], i - 1UZ, _times[i - 1UZ]));
            }
        }

        // One slope per segment, computed once. A point then costs a multiply and not a divide.
        _slopes.resize(_times.size() - 1UZ);
        for (std::size_t i = 0UZ; i + 1UZ < _times.size(); ++i) {
            _slopes[i] = (_values[i + 1UZ] - _values[i]) / static_cast<double>(_times[i + 1UZ] - _times[i]);
        }
    }

    /// The segment that holds a time, the last knot at or before it, clamped to a real segment at both ends.
    [[nodiscard]] std::size_t locate(std::int64_t t_ns) const noexcept {
        const std::size_t upper = static_cast<std::size_t>(std::upper_bound(_times.begin(), _times.end(), t_ns) - _times.begin());
        if (upper == 0UZ) {
            return 0UZ;
        }
        return std::min(upper - 1UZ, _times.size() - 2UZ);
    }

    /// The value at a time already known to lie in `segment`. At the two knot times it returns the knot values
    /// themselves and not a point on the line. A point evaluated as the end of one segment and as the start of
    /// the next then gives the same bits. The cursor walk can therefore replace a fresh search.
    [[nodiscard]] double evaluate(std::size_t segment, std::int64_t t_ns) const noexcept {
        if (t_ns <= _times.front()) {
            return _values.front();
        }
        if (t_ns >= _times.back()) {
            return _values.back();
        }
        if (t_ns == _times[segment]) {
            return _values[segment];
        }
        if (t_ns == _times[segment + 1UZ]) {
            return _values[segment + 1UZ];
        }
        return _values[segment] + _slopes[segment] * static_cast<double>(t_ns - _times[segment]);
    }

    std::vector<std::int64_t> _times;
    std::vector<double>       _values;
    std::vector<double>       _slopes; ///< value units per nanosecond across each segment
};

/**
 * @brief A table of frequency-offset knots, and the radian increments a phasor consumes.
 *
 * A pass's Doppler curve arrives as a few points and must become one radian increment per sample. `phaseIncrementsFor`
 * produces that form, one `double` radian increment per sample. Each increment is `2*pi` times the **integral** of the
 * offset across that sample's own interval. It is not `2*pi*offset(t_k)/fs` sampled at the interval's start. The offset
 * is piecewise linear. Its integral is a trapezoid per segment in closed form and costs no more than the sampled form.
 * The accumulated phase is then the schedule's true integral. A coherent demodulator tracks the phase and not the
 * frequency. A sampled increment would give it an error that grows over the run.
 *
 * The sample interval boundaries come from `SampleClock::NsWalk`: the whole nanosecond `timeOf` gives and the
 * sample's position within it, `rem / rate_num` as a `double`. No 128-bit division runs per sample. Each increment
 * is exact to double rounding at any rate. The end of one interval is the start of the next. The sum over a span
 * is therefore the integral between the exact times of its two ends, with no error that grows.
 */
class FrequencySchedule : public PiecewiseLinearSchedule {
public:
    static constexpr double kTwoPi = 2. * std::numbers::pi_v<double>;

    FrequencySchedule(std::span<const std::int64_t> timesNs, std::span<const double> offsetsHz) : PiecewiseLinearSchedule(timesNs, offsetsHz, ScheduleNaming{"gr::timing::FrequencySchedule", "offsets", "offset", "a finite frequency"}) {}

    /// The knot values, in hertz. `values()` returns the same values under a name without the unit.
    [[nodiscard]] std::span<const double> offsets() const noexcept { return _values; }

    /// The offset at a time, in hertz. This is `valueAt` under a name that carries the unit.
    [[nodiscard]] double offsetAt(std::int64_t t_ns) const noexcept { return valueAt(t_ns); }

    /**
     * @brief The radian increments for `out.size()` samples starting at `firstIndex` on `clock`.
     *
     * `out[k]` is `2*pi` times the integral of the offset, in cycles, across the interval from the exact time of
     * sample `firstIndex + k` to the exact time of the next one. The increments are `double`, one per sample.
     */
    void phaseIncrementsFor(const SampleClock& clock, std::uint64_t firstIndex, std::span<double> out) const noexcept {
        if (out.empty()) {
            return;
        }

        SampleClock::NsWalk walk    = clock.walkFrom(firstIndex);
        Instant             start   = instantOf(walk);
        std::size_t         segment = locate(start.ns);
        for (double& increment : out) {
            walk.advance();
            const Instant end = instantOf(walk);
            increment         = kTwoPi * integrateCycles(start, end, segment);
            start             = end;
        }
    }

private:
    /// A sample's time: the whole nanosecond `timeOf` gives and the position within it, in [0, 1). A knot sits on
    /// a whole nanosecond. A knot is therefore at or before an instant exactly when it is at or before
    /// `ns`.
    struct Instant {
        std::int64_t ns{0LL};
        double       fraction{0.};
    };

    [[nodiscard]] static Instant instantOf(const SampleClock::NsWalk& walk) noexcept { return Instant{walk.t_ns, static_cast<double>(walk.rem) / static_cast<double>(walk.modulus)}; }

    [[nodiscard]] static bool before(Instant a, Instant b) noexcept { return a.ns < b.ns || (a.ns == b.ns && a.fraction < b.fraction); }

    /// The offset at an instant already known to lie in `segment`, held at the end values outside the table.
    [[nodiscard]] double evaluateAt(std::size_t segment, Instant x) const noexcept {
        if (x.fraction == 0.) {
            return evaluate(segment, x.ns);
        }
        if (x.ns < _times.front()) {
            return _values.front();
        }
        if (x.ns >= _times.back()) {
            return _values.back();
        }
        return _values[segment] + _slopes[segment] * (static_cast<double>(x.ns - _times[segment]) + x.fraction);
    }

    /**
     * @brief The integral of the offset from `a` to `b`, in cycles, advancing the cursor as it goes.
     *
     * The offset is linear on each segment and constant beyond the ends. The trapezoid over a stretch that
     * crosses no knot is therefore the exact integral. A sample interval that crosses knots is split at each of
     * them. The loop exists for that case. At any usable sample rate it runs once. The cursor enters at or
     * before the segment that holds `a` and leaves at the segment that holds `b`. A whole span moves it at most
     * once per knot.
     */
    [[nodiscard]] double integrateCycles(Instant a, Instant b, std::size_t& segment) const noexcept {
        constexpr double kSecondsPerNs = 1e-9;

        double  cycles = 0.;
        Instant x      = a;
        while (before(x, b)) {
            while (segment + 2UZ < _times.size() && _times[segment + 1UZ] <= x.ns) {
                ++segment;
            }
            // The next knot strictly after `x`. Before the table that is the first knot. There the held value
            // changes to the first line. That point is a breakpoint like any other. A walk that looked only at
            // the far end of the segment would miss it.
            const std::int64_t nextKnot = (x.ns < _times.front()) ? _times.front() : _times[segment + 1UZ];
            const Instant      knot{nextKnot, 0.};
            const Instant      end  = (nextKnot > x.ns && before(knot, b)) ? knot : b;
            const double       span = static_cast<double>(end.ns - x.ns) + (end.fraction - x.fraction);
            cycles += 0.5 * (evaluateAt(segment, x) + evaluateAt(segment, end)) * span * kSecondsPerNs;
            x = end;
        }
        return cycles;
    }
};

/**
 * @brief A table of propagation-delay knots, in seconds, on the same axis and with the same reading rule.
 *
 * `FrequencySchedule` holds the effect of a pass on the carrier. This class holds its effect on the envelope.
 * `valuesFor` gives one delay per sample. The two extremes are exposed. A delay line sizes its history from the
 * largest delay in the table. The table clamps at its ends. The maximum over the table is therefore the maximum
 * over all time.
 */
class DelaySchedule : public PiecewiseLinearSchedule {
public:
    DelaySchedule(std::span<const std::int64_t> timesNs, std::span<const double> delaysSeconds) : PiecewiseLinearSchedule(timesNs, delaysSeconds, ScheduleNaming{"gr::timing::DelaySchedule", "delays", "delay", "a finite delay"}) {}

    /// The knot values, in seconds.
    [[nodiscard]] std::span<const double> delays() const noexcept { return _values; }

    /// The delay at a time, in seconds.
    [[nodiscard]] double delayAt(std::int64_t t_ns) const noexcept { return valueAt(t_ns); }

    /// The largest delay in the table, which is also the largest delay at any time.
    [[nodiscard]] double maxValue() const noexcept { return *std::ranges::max_element(_values); }
    [[nodiscard]] double minValue() const noexcept { return *std::ranges::min_element(_values); }
};

} // namespace gr::timing

#endif // GNURADIO_ALGORITHM_FREQUENCY_SCHEDULE_HPP
