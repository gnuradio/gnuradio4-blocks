#include <boost/ut.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <utility>
#include <vector>

#include <gnuradio-4.0/algorithm/timing/SampleClock.hpp>

using namespace boost::ut;
using gr::timing::IndexAt;
using gr::timing::SampleClock;

namespace {

/// The two indexes a `double` cannot carry. One is the first odd integer it rounds away. The other is just below the
/// top of the signed range, where its spacing is 1024.
constexpr std::uint64_t kBeyondDouble = (1ULL << 53) + 1ULL;
constexpr std::uint64_t kNearTop      = (1ULL << 63) - 2ULL;

/// A plausible wall-clock anchor instead of zero. A sign error in the offset then shows as a wrong timestamp, not as a
/// small number that looks right.
constexpr std::int64_t kEpochNs = 1'700'000'000'000'000'000LL;

/// 44100 * (1 + 1e-6) as an exact rational, the audio rate a trimmed oscillator actually runs at. No `double` sample
/// rate expresses it, because 44100.0441 is not a binary fraction. The kernel does not reduce the fraction and keeps
/// the caller's own terms. The map depends only on the ratio, and one of the tests below pins that.
constexpr std::uint64_t kTrimNum = 44'100'044'100ULL;
constexpr std::uint64_t kTrimDen = 1'000'000ULL;

/// The same rate in lowest terms, with both sides divided by 100.
constexpr std::uint64_t kTrimReducedNum = 441'000'441ULL;
constexpr std::uint64_t kTrimReducedDen = 10'000ULL;

/// The defining property of `indexOf`, restated independently of how it is computed. The sample it names starts at or
/// before the time. The next sample does not start before the time.
///
/// The upper bound is not strict. That comes from the floor and is not a weakness. The next sample may start part-way
/// through the nanosecond the time names. Its floored time is then that same nanosecond. `remainder_num` distinguishes
/// the two cases exactly. The tests below pin it against hand-worked values and do not rely on this predicate for it.
[[nodiscard]] bool bracketsTime(const SampleClock& clock, std::int64_t t_ns) {
    const IndexAt at = clock.indexOf(t_ns).value();
    return clock.timeOf(at.index) <= t_ns && clock.timeOf(at.index + 1ULL) >= t_ns && at.remainder_num < at.remainder_den;
}

} // namespace

const boost::ut::suite<"SampleClock"> sampleClockTests = [] {
    // An integral period. At 250 kS/s a sample is exactly 4000 ns. Index and time are then in
    // bijection. The round trip is an identity, even at indexes a double index would already have lost.
    "an integral period round-trips exactly past 2^53"_test = [] {
        for (const std::uint64_t index : {kBeyondDouble, kNearTop}) {
            const std::uint64_t anchorIndex = index - 1'000'000ULL;
            const SampleClock   clock(anchorIndex, kEpochNs, 250'000ULL, 1ULL);

            expect(eq(clock.timeOf(anchorIndex), kEpochNs));
            expect(eq(clock.timeOf(index), kEpochNs + 4'000'000'000LL)) << "1e6 samples at 4000 ns each";

            for (const std::uint64_t probe : std::initializer_list<std::uint64_t>{index - 3ULL, index - 1ULL, index, index + 1ULL, index + 3ULL}) {
                const IndexAt back = clock.indexOf(clock.timeOf(probe)).value();
                expect(eq(back.index, probe));
                expect(eq(back.remainder_num, 0ULL)) << "a whole number of nanoseconds leaves no sub-sample part";
                expect(eq(back.remainder_den, 1'000'000'000ULL));
            }
        }
    };

    // A rational period. 10^9 * rate_den = 10^15 and 10^15 = 22675 * 44100044100 + 31500032500. A sample
    // of this clock is therefore 22675 ns and a bit, and almost no sample lands on a whole nanosecond. The map is still
    // exact. A sample used as the anchor reads back with no remainder. Every time in the span falls between the two
    // samples it belongs between.
    "a rational period is exact where no sample lands on a nanosecond"_test = [] {
        constexpr std::uint64_t kPeriodNum = 1'000'000'000ULL * kTrimDen;
        expect(eq(kPeriodNum / kTrimNum, 22'675ULL));
        expect(eq(kPeriodNum % kTrimNum, 31'500'032'500ULL));

        for (const std::uint64_t index : {kBeyondDouble, kNearTop}) {
            const SampleClock clock(index, kEpochNs, kTrimNum, kTrimDen);

            expect(eq(clock.timeOf(index), kEpochNs));
            const IndexAt at = clock.indexOf(kEpochNs).value();
            expect(eq(at.index, index));
            expect(eq(at.remainder_num, 0ULL));
            expect(eq(at.remainder_den, kPeriodNum));

            // Steps of 22675 and 22676 ns mix about 2 to 5. 1000 samples cover 22675714 ns. A floor that silently
            // truncated toward the anchor would show up within a few samples of it.
            expect(eq(clock.timeOf(index + 1ULL), kEpochNs + 22'675LL));
            expect(eq(clock.timeOf(index - 1ULL), kEpochNs - 22'676LL)) << "the sample before the anchor floors down, not toward it";

            for (std::int64_t k = -4LL; k <= 4LL; ++k) {
                expect(that % bracketsTime(clock, kEpochNs + k * 22'675LL));
                expect(that % bracketsTime(clock, kEpochNs + k * 22'675LL + 7LL));
            }

            // The sub-sample remainder, worked by hand. 22675 ns past the anchor is
            // 22675 * 44100044100 = 10^15 - 31500032500 sample-periods in. That is the division above read as a
            // fraction. A caller that took only the whole sample would be 0.99997 of a sample short, and nothing would
            // show the error.
            const IndexAt justShort = clock.indexOf(kEpochNs + 22'675LL).value();
            expect(eq(justShort.index, index));
            expect(eq(justShort.remainder_num, 999'968'499'967'500ULL));
            expect(eq(justShort.remainder_den, kPeriodNum));
            expect(eq(justShort.remainder_num + 31'500'032'500ULL, kPeriodNum));
        }
    };

    // The floor is the whole contract on the negative side. There truncation toward zero is the easy mistake. -1e9/3
    // truncates to -333333333 and floors to -333333334. The two differ by a nanosecond on every sample below the
    // anchor.
    "the map floors below the anchor rather than truncating"_test = [] {
        const SampleClock clock(1'000'000ULL, 0LL, 3ULL, 1ULL);

        expect(eq(clock.timeOf(1'000'001ULL), 333'333'333LL));
        expect(eq(clock.timeOf(1'000'002ULL), 666'666'666LL));
        expect(eq(clock.timeOf(1'000'003ULL), 1'000'000'000LL));
        expect(eq(clock.timeOf(999'999ULL), -333'333'334LL));
        expect(eq(clock.timeOf(999'998ULL), -666'666'667LL));
        expect(eq(clock.timeOf(999'997ULL), -1'000'000'000LL));

        // The same floor read the other way. 1 ns before the anchor is still inside the sample that started
        // 333333333.33 ns earlier, 999999997/1e9 of the way through it.
        const IndexAt at = clock.indexOf(-1LL).value();
        expect(eq(at.index, 999'999ULL));
        expect(eq(at.remainder_num, 999'999'997ULL));
        expect(eq(at.remainder_den, 1'000'000'000ULL));

        for (std::int64_t t = -1'000'000'003LL; t <= 1'000'000'003LL; t += 7'919LL) {
            expect(that % bracketsTime(clock, t));
        }
    };

    // An integral period at 250 kS/s makes the sample exactly the last one below zero. That pins the sign of the whole
    // map, not only its floor.
    "times below the epoch anchor are exact"_test = [] {
        const SampleClock clock(1'000'000ULL, 0LL, 250'000ULL, 1ULL);

        expect(eq(clock.timeOf(999'999ULL), -4'000LL));
        expect(eq(clock.timeOf(750'000ULL), -1'000'000'000LL));
        expect(eq(clock.indexOf(-1'000'000'000LL).value().index, 750'000ULL));
        expect(eq(clock.indexOf(-1'000'000'000LL).value().remainder_num, 0ULL));
        expect(eq(clock.indexOf(-999'999'999LL).value().index, 750'000ULL)) << "one nanosecond later is still that sample";
    };

    // The anchor is bookkeeping and not part of the map. A rebase must leave every conversion
    // bit-identical wherever it lands. That covers a rebase inside the span, above it, below it and on a sample whose
    // time is not a whole nanosecond. `anchor_rem` makes that true. A rebase that dropped it would shift the whole span
    // by a nanosecond.
    "rebase leaves the map bit-identical"_test = [] {
        constexpr std::uint64_t kBase = kBeyondDouble;
        constexpr std::size_t   kSpan = 4'096UZ;
        const SampleClock       clock(kBase, kEpochNs, kTrimNum, kTrimDen);

        std::vector<std::int64_t> reference(kSpan);
        for (std::size_t k = 0UZ; k < kSpan; ++k) {
            reference[k] = clock.timeOf(kBase + k);
        }

        for (const std::uint64_t at : std::initializer_list<std::uint64_t>{kBase, kBase + 1ULL, kBase + 1'234ULL, kBase + kSpan - 1ULL, kBase + 10ULL * kSpan, kBase - 7ULL}) {
            const SampleClock moved = clock.rebase(at);

            expect(eq(moved.anchor_index, at));
            expect(eq(moved.timeOf(at), clock.timeOf(at)));
            for (std::size_t k = 0UZ; k < kSpan; ++k) {
                expect(eq(moved.timeOf(kBase + k), reference[k])) << "rebase changed the map";
            }
            for (std::size_t k = 0UZ; k < kSpan; k += 97UZ) {
                expect(eq(moved.indexOf(reference[k]).value().index, clock.indexOf(reference[k]).value().index));
                expect(eq(moved.indexOf(reference[k]).value().remainder_num, clock.indexOf(reference[k]).value().remainder_num));
            }
        }

        // Rebasing twice is rebasing once, which makes it safe to do on every buffer.
        const SampleClock twice = clock.rebase(kBase + 3ULL).rebase(kBase + 11ULL);
        const SampleClock once  = clock.rebase(kBase + 11ULL);
        expect(eq(twice.anchor_ns, once.anchor_ns));
        expect(eq(twice.anchor_rem, once.anchor_rem));
    };

    // The reason for all the integer arithmetic. At the anchor plus one sample, a double index cannot tell the two
    // indexes apart. Near 2^63 its spacing is 1024, and a delta of one sample reads as zero.
    "a double index loses what the integer map keeps"_test = [] {
        {
            const SampleClock clock(1ULL << 53, kEpochNs, 250'000ULL, 1ULL);

            const double doubleDelta = static_cast<double>(kBeyondDouble) - static_cast<double>(1ULL << 53);
            expect(eq(doubleDelta, 0.)) << "2^53 + 1 is not a double";
            expect(eq(kEpochNs + static_cast<std::int64_t>(doubleDelta * 4'000.), kEpochNs)) << "the double path returns the anchor's own time";
            expect(eq(clock.timeOf(kBeyondDouble), kEpochNs + 4'000LL)) << "the integer path is one sample period on, as it should be";
        }
        {
            constexpr std::uint64_t kAnchor = kNearTop - 1ULL;
            const SampleClock       clock(kAnchor, kEpochNs, 250'000ULL, 1ULL);

            // Just under 2^63 a double's neighbors are 1024 apart. A double cannot tell two adjacent samples apart
            // there. Both round to 2^63, and the delta a double path would use is zero.
            expect(eq(static_cast<double>(kNearTop), static_cast<double>(kAnchor)));
            expect(eq(static_cast<double>(kNearTop) - 1., static_cast<double>(kNearTop))) << "a double up there cannot even subtract one";

            const double doubleDelta = static_cast<double>(kNearTop) - static_cast<double>(kAnchor);
            expect(eq(doubleDelta, 0.));
            expect(eq(kEpochNs + static_cast<std::int64_t>(doubleDelta * 4'000.), kEpochNs)) << "the double path returns the anchor's own time";
            expect(eq(clock.timeOf(kNearTop), kEpochNs + 4'000LL)) << "the integer path is one sample period on";
        }
    };

    // The kernel keeps the caller's own terms, so the same rate written two ways is two different structs. It is
    // one map, and it has to read as one.
    "the map depends on the ratio, not on the terms"_test = [] {
        const SampleClock stated(kBeyondDouble, kEpochNs, kTrimNum, kTrimDen);
        const SampleClock reduced(kBeyondDouble, kEpochNs, kTrimReducedNum, kTrimReducedDen);

        expect(neq(stated.rate_num, reduced.rate_num)) << "the kernel does not reduce the fraction";
        for (std::uint64_t k = 0ULL; k < 10'000ULL; k += 13ULL) {
            expect(eq(stated.timeOf(kBeyondDouble + k), reduced.timeOf(kBeyondDouble + k)));
            expect(eq(stated.indexOf(kEpochNs + static_cast<std::int64_t>(k) * 22'675LL).value().index, reduced.indexOf(kEpochNs + static_cast<std::int64_t>(k) * 22'675LL).value().index));
        }
    };

    // The per-sample path. It must be the same function of the absolute index as `timeOf`. Otherwise a caller that
    // chunks its work reads different times from one that does not.
    "the division-free walk reproduces timeOf"_test = [] {
        for (const auto& [num, den] : std::vector<std::pair<std::uint64_t, std::uint64_t>>{{250'000ULL, 1ULL}, {kTrimNum, kTrimDen}, {3ULL, 1ULL}}) {
            const SampleClock clock(kBeyondDouble, kEpochNs, num, den);

            SampleClock::NsWalk walk = clock.walkFrom(kBeyondDouble + 5ULL);
            for (std::uint64_t k = 0ULL; k < 20'000ULL; ++k) {
                expect(eq(walk.t_ns, clock.timeOf(kBeyondDouble + 5ULL + k)));
                walk.advance();
            }
        }
    };

    // A rate change is a new anchor at the sample where it happens. The time of that sample is unchanged, and
    // everything after it runs at the new rate.
    "withRate anchors the new rate at the change index"_test = [] {
        const SampleClock before(1'000ULL, kEpochNs, 250'000ULL, 1ULL);
        const SampleClock after = before.withRate(500'000ULL, 1ULL, 2'000ULL).value();

        expect(eq(after.anchor_index, 2'000ULL));
        expect(eq(after.timeOf(2'000ULL), before.timeOf(2'000ULL)));
        expect(eq(before.timeOf(2'000ULL), kEpochNs + 4'000'000LL)) << "1000 samples at 4000 ns";
        expect(eq(after.timeOf(3'000ULL), kEpochNs + 4'000'000LL + 2'000'000LL)) << "1000 samples at 2000 ns";
        expect(eq(after.rate_num, 500'000ULL));
        expect(eq(after.rate_den, 1ULL));
        expect(eq(before.rate_num, 250'000ULL)) << "the original clock is untouched";
    };

    // Sample 1 of a 3 S/s clock lies a third of a nanosecond past 333333333 ns. Over the new numerator 7 that position
    // is 7/3 sevenths, which truncates to 2. The anchor moves earlier by 1/21 ns, under 1/7 ns.
    "withRate carries a sub-nanosecond anchor over to the new numerator"_test = [] {
        const SampleClock          before(0ULL, 0LL, 3ULL, 1ULL);
        const SampleClock          after = before.withRate(7ULL, 1ULL, 1ULL).value();
        const SampleClock::NsSplit at    = before.offsetFrom(1ULL);

        expect(eq(at.rem, 1ULL)) << "sample 1 is not on a whole nanosecond";
        expect(eq(after.anchor_ns, before.timeOf(1ULL)));
        expect(eq(after.timeOf(1ULL), before.timeOf(1ULL)));
        expect(le(after.anchor_rem * before.rate_num, at.rem * after.rate_num)) << "the anchor does not move later";
        expect(lt(at.rem * after.rate_num, (after.anchor_rem + 1ULL) * before.rate_num)) << "the anchor moves earlier by under 1/7 ns";
        expect(eq(after.timeOf(2ULL), before.timeOf(1ULL) + static_cast<std::int64_t>((after.anchor_rem + 1'000'000'000ULL) / after.rate_num))) << "the next sample counts from the carried position";
    };

    // The index is unsigned. A time before sample 0 has no index. A time past the last index a `std::uint64_t` holds
    // has none either.
    "indexOf refuses a time outside the index range"_test = [] {
        const SampleClock clock(0ULL, 1'000'000LL, 250'000ULL, 1ULL);
        expect(eq(clock.indexOf(clock.timeOf(0ULL)).value().index, 0ULL));
        const auto early = clock.indexOf(clock.timeOf(0ULL) - 1LL);
        expect(fatal(!early.has_value())) << "a time before sample 0";
        expect(early.error().message.contains("before sample 0")) << early.error().message;

        constexpr std::uint64_t kLast = std::numeric_limits<std::uint64_t>::max();
        const SampleClock       top(kLast, 0LL, 250'000ULL, 1ULL);
        expect(eq(top.indexOf(3'999LL).value().index, kLast));
        const auto late = top.indexOf(4'000LL);
        expect(fatal(!late.has_value())) << "a time one period past the last index";
        expect(late.error().message.contains("past the last sample index")) << late.error().message;
    };

    // A time and an anchor can lie more than 2^63 ns apart, past the range of their signed difference.
    "indexOf reads a time more than 2^63 ns from its anchor"_test = [] {
        constexpr std::int64_t kFirst = std::numeric_limits<std::int64_t>::min();
        constexpr std::int64_t kLast  = std::numeric_limits<std::int64_t>::max();

        const SampleClock after(0ULL, kEpochNs, 1ULL, 1ULL);
        const auto        early = after.indexOf(kFirst);
        expect(fatal(!early.has_value())) << "the earliest time lies before sample 0";
        expect(early.error().message.contains("before sample 0")) << early.error().message;

        // At 1 Hz the index counts whole seconds from the anchor and the remainder counts the nanoseconds left.
        const SampleClock   before(0ULL, -kEpochNs, 1ULL, 1ULL);
        const std::uint64_t distance = static_cast<std::uint64_t>(kLast) + static_cast<std::uint64_t>(kEpochNs);
        const auto          late     = before.indexOf(kLast);
        expect(fatal(late.has_value())) << "the latest time lies after sample 0";
        expect(eq(late->index, distance / SampleClock::kNsPerSecond));
        expect(eq(late->remainder_num, distance % SampleClock::kNsPerSecond));
    };

    "a clock refuses a rate it cannot hold exactly"_test = [] {
        expect(throws<gr::exception>([] { (void)SampleClock(0ULL, 0LL, 0ULL, 1ULL); })) << "a zero numerator";
        expect(throws<gr::exception>([] { (void)SampleClock(0ULL, 0LL, 48'000ULL, 0ULL); })) << "a zero denominator";
        expect(throws<gr::exception>([] { (void)SampleClock(0ULL, 0LL, 48'000ULL, SampleClock::kMaxRateDen + 1ULL); })) << "10^9 * rate_den would pass 2^63";
        expect(throws<gr::exception>([] { (void)SampleClock(0ULL, 0LL, SampleClock::kMaxRateNum + 1ULL, 1ULL); })) << "the numerator would pass 2^63";
        expect(throws<gr::exception>([] { (void)SampleClock(0ULL, 0LL, 48'000ULL, 1ULL, 48'000ULL); })) << "the anchor remainder must be a proper fraction";

        expect(nothrow([] { (void)SampleClock(0ULL, 0LL, 48'000ULL, 1ULL, 47'999ULL); }));
        expect(nothrow([] { (void)SampleClock(0ULL, 0LL, SampleClock::kMaxRateNum, SampleClock::kMaxRateDen); }));
    };

    "a rate change and a floating rate return a refused rate as the error"_test = [] {
        const SampleClock clock(0ULL, kEpochNs, 48'000ULL, 1ULL);
        const auto        zero = clock.withRate(0ULL, 1ULL, 100ULL);
        expect(fatal(!zero.has_value()));
        expect(zero.error().message.contains("neither term may be zero")) << zero.error().message;
        expect(!clock.withRate(48'000ULL, SampleClock::kMaxRateDen + 1ULL, 100ULL).has_value()) << "a denominator the constructor would refuse";

        const auto tooSlow = gr::timing::clockForRateHz(1e-7, 0ULL, kEpochNs);
        expect(fatal(!tooSlow.has_value()));
        expect(tooSlow.error().message.contains("outside the")) << tooSlow.error().message;
        expect(!gr::timing::clockForRateHz(std::numeric_limits<double>::quiet_NaN(), 0ULL, kEpochNs).has_value());
        expect(gr::timing::clockForRateHz(SampleClock::kMinRateHz, 0ULL, kEpochNs).has_value()) << "the slowest rate";
        expect(gr::timing::clockForRateHz(SampleClock::kMaxRateHz, 0ULL, kEpochNs).has_value()) << "the fastest rate";
        expect(!gr::timing::clockForRateHz(std::nextafter(SampleClock::kMinRateHz, 0.), 0ULL, kEpochNs).has_value()) << "just below the slowest rate";
        expect(!gr::timing::clockForRateHz(std::nextafter(SampleClock::kMaxRateHz, 2. * SampleClock::kMaxRateHz), 0ULL, kEpochNs).has_value()) << "just above the fastest rate";

        const auto rounded = gr::timing::clockForRateHz(48'000.0000004, 7ULL, kEpochNs);
        expect(fatal(rounded.has_value()));
        expect(eq(rounded->rate_num, 48'000'000'000ULL) && eq(rounded->rate_den, 1'000'000ULL)) << "the microhertz grid";
        expect(eq(rounded->anchor_index, 7ULL));
    };

    "a default clock is one sample per second at the epoch"_test = [] {
        constexpr SampleClock clock{};

        expect(eq(clock.timeOf(0ULL), 0LL));
        expect(eq(clock.timeOf(5ULL), 5'000'000'000LL));
        expect(eq(clock.rateHz(), 1.));
        expect(eq(clock.periodNsNum(), 1'000'000'000ULL));
    };
};

int main() { /* not needed for UT */ }
