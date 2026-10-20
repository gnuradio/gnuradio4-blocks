#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/algorithm/signal/Phasor.hpp>
#include <gnuradio-4.0/algorithm/timing/FrequencySchedule.hpp>
#include <gnuradio-4.0/algorithm/timing/SampleClock.hpp>

using namespace boost::ut;
using gr::signal::Phasor;
using gr::timing::DelaySchedule;
using gr::timing::FrequencySchedule;
using gr::timing::offsetFor;
using gr::timing::SampleClock;

namespace {

constexpr double kTwoPi = 2. * std::numbers::pi_v<double>;

/// A time as a whole nanosecond and the position within it, in [0, 1).
struct Instant {
    std::int64_t ns{0LL};
    double       fraction{0.};
};

[[nodiscard]] bool before(Instant a, Instant b) { return a.ns < b.ns || (a.ns == b.ns && a.fraction < b.fraction); }

/// The exact time of sample `index` on a clock with sample 0 at 0 ns. It is computed from the rate directly and not
/// through `SampleClock`. The product must fit 64 bits, which holds for the indexes these cases use.
[[nodiscard]] Instant exactTime(std::uint64_t rateNum, std::uint64_t rateDen, std::uint64_t index) {
    const std::uint64_t numerator = index * 1'000'000'000ULL * rateDen;
    return Instant{static_cast<std::int64_t>(numerator / rateNum), static_cast<double>(numerator % rateNum) / static_cast<double>(rateNum)};
}

/// The schedule's contract, restated independently. The value is clamped outside the table. Between two knots it is the
/// straight line written the other way round. A shared slope error therefore cannot make both this and the kernel
/// wrong.
[[nodiscard]] double oracleOffset(std::span<const std::int64_t> times, std::span<const double> offsets, std::int64_t t_ns, double fraction = 0.) {
    if (t_ns < times.front() || (t_ns == times.front() && fraction == 0.)) {
        return offsets.front();
    }
    if (t_ns >= times.back()) {
        return offsets.back();
    }
    const std::size_t index = static_cast<std::size_t>(std::upper_bound(times.begin(), times.end(), t_ns) - times.begin()) - 1UZ;
    const double      w     = (static_cast<double>(t_ns - times[index]) + fraction) / static_cast<double>(times[index + 1UZ] - times[index]);
    return offsets[index] * (1. - w) + offsets[index + 1UZ] * w;
}

/// The closed-form integral of that offset from `a` to `b`, in cycles. It sums a trapezoid between consecutive
/// breakpoints. A search per breakpoint locates each one, without a cursor.
[[nodiscard]] double oracleCycles(std::span<const std::int64_t> times, std::span<const double> offsets, Instant a, Instant b) {
    double  cycles = 0.;
    Instant x      = a;
    while (before(x, b)) {
        const auto    next = std::upper_bound(times.begin(), times.end(), x.ns);
        const Instant end  = (next != times.end() && before(Instant{*next, 0.}, b)) ? Instant{*next, 0.} : b;
        const double  span = static_cast<double>(end.ns - x.ns) + (end.fraction - x.fraction);
        cycles += 0.5 * (oracleOffset(times, offsets, x.ns, x.fraction) + oracleOffset(times, offsets, end.ns, end.fraction)) * span * 1e-9;
        x = end;
    }
    return cycles;
}

/// Kahan summation. The test measures the increments, not how well a naive loop adds 48000 of them. A plain sum of
/// 48000 increments can lose more than the 1e-9 rad they are held to.
[[nodiscard]] double compensatedSum(std::span<const double> values) {
    double sum          = 0.;
    double compensation = 0.;
    for (const double value : values) {
        const double adjusted = value - compensation;
        const double next     = sum + adjusted;
        compensation          = (next - sum) - adjusted;
        sum                   = next;
    }
    return sum;
}

/// A pass-shaped table of `nKnots` knots spread evenly over `spanNs`. It follows the S-curve of a LEO Doppler profile.
/// The offset is high on approach, passes through zero at closest approach and is low going away.
struct Profile {
    std::vector<std::int64_t> times;
    std::vector<double>       offsets;
};

[[nodiscard]] Profile leoProfile(std::size_t nKnots, std::int64_t startNs, std::int64_t spanNs) {
    Profile profile;
    profile.times.resize(nKnots);
    profile.offsets.resize(nKnots);
    for (std::size_t i = 0UZ; i < nKnots; ++i) {
        const double u     = static_cast<double>(i) / static_cast<double>(nKnots - 1UZ);
        profile.times[i]   = startNs + static_cast<std::int64_t>(static_cast<double>(spanNs) * u);
        profile.offsets[i] = -10'000. * std::tanh(6. * (u - 0.5));
    }
    return profile;
}

} // namespace

const boost::ut::suite<"FrequencySchedule"> frequencyScheduleTests = [] {
    // A ramp from -5 kHz to +5 kHz over ten seconds. The knots read their own values. The midpoint and the
    // quarter points read the interpolant to within one rounding of the knot values. The slope 1e-6 per nanosecond
    // is not exact in binary. A build that rounds the product reads these points exactly. A build that fuses the
    // multiply and the add keeps the slope's rounding, at most 5000 * epsilon here. Outside the table the end values
    // hold and the line does not continue.
    "a ramp reads its knots, its interpolant and its held ends"_test = [] {
        const std::vector<std::int64_t> times{0LL, 10'000'000'000LL};
        const std::vector<double>       offsets{-5'000., 5'000.};
        const FrequencySchedule         schedule(times, offsets);
        const double                    rounding = 5'000. * std::numeric_limits<double>::epsilon();

        expect(eq(schedule.size(), 2UZ));
        expect(eq(schedule.offsetAt(0LL), -5'000.));
        expect(eq(schedule.offsetAt(10'000'000'000LL), 5'000.));
        expect(approx(schedule.offsetAt(5'000'000'000LL), 0., rounding)) << "the midpoint of a symmetric ramp";
        expect(approx(schedule.offsetAt(2'500'000'000LL), -2'500., rounding));
        expect(approx(schedule.offsetAt(7'500'000'000LL), 2'500., rounding));

        expect(eq(schedule.offsetAt(-1LL), -5'000.)) << "one nanosecond before the table";
        expect(eq(schedule.offsetAt(-1'000'000'000'000LL), -5'000.));
        expect(eq(schedule.offsetAt(std::numeric_limits<std::int64_t>::min()), -5'000.));
        expect(eq(schedule.offsetAt(10'000'000'001LL), 5'000.)) << "one nanosecond after it";
        expect(eq(schedule.offsetAt(std::numeric_limits<std::int64_t>::max()), 5'000.)) << "held, never extrapolated";

        for (std::int64_t t = -2'000'000'000LL; t <= 12'000'000'000LL; t += 37'000'011LL) {
            expect(approx(schedule.offsetAt(t), oracleOffset(times, offsets, t), 1e-9));
        }
    };

    // The span starts before the table and ends after it. It covers the held value, both segments and the
    // held value again. Every knot falls inside a sample interval, not on its boundary. The trapezoid split exists for
    // that case.
    "the accumulated phase is the schedule's integral, however it is chunked"_test = [] {
        constexpr std::size_t kSamples = 48'000UZ;

        const std::vector<std::int64_t> times{200'000'003LL, 500'000'007LL, 800'000'011LL};
        const std::vector<double>       offsets{-5'000., 2'000., 4'000.};
        const FrequencySchedule         schedule(times, offsets);
        const SampleClock               clock(0ULL, 0LL, 48'000ULL, 1ULL);

        std::vector<double> whole(kSamples);
        schedule.phaseIncrementsFor(clock, 0ULL, whole);

        const double reference = kTwoPi * oracleCycles(times, offsets, exactTime(48'000ULL, 1ULL, 0ULL), exactTime(48'000ULL, 1ULL, kSamples));
        const double achieved  = std::abs(compensatedSum(whole) - reference);
        std::println("qa_FrequencySchedule: accumulated phase {:.6f} rad over {} samples, {:.3e} rad from the closed-form integral", compensatedSum(whole), kSamples, achieved);
        expect(lt(achieved, 1e-9)) << "the accumulated phase is not the schedule's integral";

        // The same schedule and clock with different call boundaries. The increments must be the same numbers, not
        // merely close ones. A change of buffer size must not change the output.
        for (const std::size_t chunk : {1UZ, 7UZ, 1'000UZ, 12'345UZ}) {
            std::vector<double> pieced(kSamples);
            for (std::size_t produced = 0UZ; produced < kSamples; produced += chunk) {
                const std::size_t n = std::min(chunk, kSamples - produced);
                schedule.phaseIncrementsFor(clock, produced, std::span<double>(pieced).subspan(produced, n));
            }
            expect(that % std::ranges::equal(pieced, whole)) << std::format("chunk size {} changed the increments", chunk);
            expect(eq(compensatedSum(pieced), compensatedSum(whole)));
        }
    };

    // The cursor is an optimization. It must give the same answer as the computation it replaces. A thousand knots
    // 20820 ns apart against a sample period of 20833 ns put a knot inside very nearly every sample interval. A cursor
    // that moved only once per call would get that case wrong. The span also runs past the last knot, and the walk
    // covers the held tail too.
    "the cursor walk agrees with a search per sample"_test = [] {
        constexpr std::size_t   kSamples = 1'200UZ;
        constexpr std::uint64_t kFirst   = 0ULL;

        const Profile           profile = leoProfile(1'000UZ, 0LL, 20'800'000LL);
        const FrequencySchedule schedule(profile.times, profile.offsets);
        const SampleClock       clock(0ULL, 0LL, 48'000ULL, 1ULL);

        std::vector<double> increments(kSamples);
        schedule.phaseIncrementsFor(clock, kFirst, increments);

        double worst = 0.;
        for (std::size_t k = 0UZ; k < kSamples; ++k) {
            const double want = kTwoPi * oracleCycles(profile.times, profile.offsets, exactTime(48'000ULL, 1ULL, kFirst + k), exactTime(48'000ULL, 1ULL, kFirst + k + 1UZ));
            worst             = std::max(worst, std::abs(increments[k] - want));
        }
        std::println("qa_FrequencySchedule: worst per-sample increment difference against a per-sample search {:.3e} rad", worst);
        expect(lt(worst, 1e-12));

        // A chunk boundary resets the cursor, so a dense table is where chunk independence is worth
        // asking about a second time.
        std::vector<double> oneAtATime(kSamples);
        for (std::size_t k = 0UZ; k < kSamples; ++k) {
            schedule.phaseIncrementsFor(clock, kFirst + k, std::span<double>(oneAtATime).subspan(k, 1UZ));
        }
        expect(that % std::ranges::equal(oneAtATime, increments)) << "a dense table read one sample at a time";
    };

    // A closing pass at 437 MHz and 7 km/s reads +10203.7257 Hz, which is 437e6 * 7000 / 299792458. The sign
    // matters beyond the digits. A closing range reads high.
    "a closing pass reads high, to the hertz"_test = [] {
        const double closing = offsetFor(-7'000., 437e6);

        std::println("qa_FrequencySchedule: offsetFor(-7000 m/s, 437 MHz) = {:.4f} Hz", closing);
        expect(gt(closing, 0.)) << "a closing pass must read high";
        expect(eq(std::llround(closing), 10'204LL)) << "437e6 * 7000 / 299792458 = 10203.7257 Hz";
        expect(approx(closing, 10'203.7257, 1e-3));
        expect(eq(closing, 437e6 * 7'000. / 299'792'458.));

        expect(eq(offsetFor(7'000., 437e6), -closing)) << "a receding pass reads low by the same amount";
        expect(eq(offsetFor(0., 437e6), 0.));
        expect(approx(offsetFor(-1., 299'792'458.), 1., 1e-12)) << "one meter per second on a carrier of c hertz is one hertz";
    };

    "a table that cannot be interpolated is refused"_test = [] {
        const std::vector<std::int64_t> two{0LL, 1'000LL};
        const std::vector<double>       twoOffsets{0., 1.};

        expect(nothrow([&] { (void)FrequencySchedule(two, twoOffsets); })) << "two knots make a line";

        expect(throws<gr::exception>([] {
            const std::vector<std::int64_t> times{0LL};
            const std::vector<double>       offsets{0.};
            (void)FrequencySchedule(times, offsets);
        })) << "one knot is not a schedule";
        expect(throws<gr::exception>([] { (void)FrequencySchedule(std::span<const std::int64_t>{}, std::span<const double>{}); })) << "no knots at all";
        expect(throws<gr::exception>([&] { (void)FrequencySchedule(two, std::span<const double>(twoOffsets).first(1UZ)); })) << "the two vectors must be paired";

        expect(throws<gr::exception>([] {
            const std::vector<std::int64_t> times{0LL, 1'000LL, 1'000LL};
            const std::vector<double>       offsets{0., 1., 2.};
            (void)FrequencySchedule(times, offsets);
        })) << "two knots at the same time";
        expect(throws<gr::exception>([] {
            const std::vector<std::int64_t> times{0LL, 2'000LL, 1'000LL};
            const std::vector<double>       offsets{0., 1., 2.};
            (void)FrequencySchedule(times, offsets);
        })) << "time running backwards";

        for (const double bad : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
            expect(throws<gr::exception>([bad] {
                const std::vector<std::int64_t> times{0LL, 1'000LL};
                const std::vector<double>       offsets{0., bad};
                (void)FrequencySchedule(times, offsets);
            })) << "an offset that is not a frequency";
        }
    };

    "the table holds up to kMaxKnots knots"_test = [] {
        const Profile full = leoProfile(FrequencySchedule::kMaxKnots, 0LL, 600'000'000'000LL);
        expect(nothrow([&] { (void)FrequencySchedule(full.times, full.offsets); }));

        std::vector<std::int64_t> tooMany(FrequencySchedule::kMaxKnots + 1UZ);
        std::vector<double>       tooManyOffsets(FrequencySchedule::kMaxKnots + 1UZ, 0.);
        for (std::size_t i = 0UZ; i < tooMany.size(); ++i) {
            tooMany[i] = static_cast<std::int64_t>(i);
        }
        expect(throws<gr::exception>([&] { (void)FrequencySchedule(tooMany, tooManyOffsets); }));
    };

    // A constant schedule is the degenerate case, and the arithmetic must still get it right. Every increment is the
    // plain 2*pi*f/fs. The trapezoid must not introduce a difference where the integral has none.
    "a flat schedule gives the plain increment"_test = [] {
        const std::vector<std::int64_t> times{0LL, 10'000'000'000LL};
        const std::vector<double>       offsets{1'000., 1'000.};
        const FrequencySchedule         schedule(times, offsets);
        const SampleClock               clock(0ULL, 0LL, 1'000'000ULL, 1ULL); // 1 MS/s, a sample of exactly 1000 ns

        std::vector<double> increments(64UZ);
        schedule.phaseIncrementsFor(clock, 5'000ULL, increments);

        for (const double increment : increments) {
            expect(approx(increment, kTwoPi * 1'000. / 1'000'000., 1e-15));
        }
    };

    // At 2.4 MS/s a sample is 416 2/3 ns, and only every third sample lands on a whole nanosecond. Every sample
    // interval still has the same exact length. Every increment is then the plain 2*pi*f/fs.
    "a flat schedule gives the plain increment where the period is not whole nanoseconds"_test = [] {
        constexpr double kRate   = 2'400'000.;
        constexpr double kOffset = 10'000.;

        const std::vector<std::int64_t> times{0LL, 10'000'000'000LL};
        const std::vector<double>       offsets{kOffset, kOffset};
        const FrequencySchedule         schedule(times, offsets);
        const SampleClock               clock(0ULL, 0LL, 2'400'000ULL, 1ULL);

        std::vector<double> increments(3'000UZ);
        schedule.phaseIncrementsFor(clock, 7ULL, increments);

        const double plain = kTwoPi * kOffset / kRate;
        double       worst = 0.;
        for (const double increment : increments) {
            worst = std::max(worst, std::abs(increment - plain));
        }
        std::println("qa_FrequencySchedule: flat {} Hz at {} S/s, worst increment {:.3e} rad from 2*pi*f/fs = {:.9e} rad", kOffset, kRate, worst, plain);
        expect(lt(worst, plain * 1e-12)) << "the increments follow the floored nanoseconds and not the sample times";
    };

    // The schedule's increments drive the phasor directly.
    //
    // A two-knot ramp from -5 kHz to +15 kHz across two seconds at 48 kS/s. The mixed tone's angle is unwrapped against
    // the schedule's own closed-form integral. The residual is asserted at every sample, not only at the end. An error
    // that grew with the run would show as a growing residual. A coherent demodulator needs an error that does not
    // grow.
    //
    // The tolerance is float32's. It belongs to the output, not to the phasor. The phase is accumulated in double and
    // reduced every step. Its own error over 96000 samples is a few times 1e-11 rad. The test reads `atan2` of a
    // `std::complex<float>` from a complex multiply. The multiply has four products and two sums, each rounded at
    // 2^-24 = 5.96e-8 relative. That bounds the angle of the result at roughly 8 * 2^-24 = 4.8e-7 rad. The test bound
    // is 1e-6 rad, twice that bound. The test prints the achieved figure.
    "a schedule's increments drive the phasor to the schedule's own integral"_test = [] {
        constexpr std::size_t kSamples   = 96'000UZ;
        constexpr double      kTolerance = 1e-6;

        const std::vector<std::int64_t> times{0LL, 2'000'000'000LL};
        const std::vector<double>       offsets{-5'000., 15'000.};
        const FrequencySchedule         schedule(times, offsets);
        const SampleClock               clock(0ULL, 0LL, 48'000ULL, 1ULL);

        std::vector<double> increments(kSamples);
        schedule.phaseIncrementsFor(clock, 0ULL, increments);

        // A constant unit-magnitude input. The output is the input's own angle plus the phasor's, and the test
        // exercises the multiply. 0.6 and 0.8 are a unit vector to within their own float rounding. The reference takes
        // the angle from the stored floats, not from the decimals that produced them.
        const std::complex<float>              carrier{0.6f, 0.8f};
        const std::vector<std::complex<float>> in(kSamples, carrier);
        std::vector<std::complex<float>>       out(kSamples);

        Phasor<float> phasor;
        phasor.configure(0., 0.);
        phasor.mixModulated(std::span<const double>(increments), in, out);

        const double carrierAngle = std::atan2(static_cast<double>(carrier.imag()), static_cast<double>(carrier.real()));
        expect(eq(clock.timeOf(0ULL), 0LL)) << "the reference times below count from sample 0 at 0 ns";

        double worst = 0.;
        double last  = 0.;
        for (std::size_t k = 0UZ; k < kSamples; ++k) {
            const double reference = carrierAngle + kTwoPi * oracleCycles(times, offsets, exactTime(48'000ULL, 1ULL, 0ULL), exactTime(48'000ULL, 1ULL, k));
            const double measured  = std::atan2(static_cast<double>(out[k].imag()), static_cast<double>(out[k].real()));
            const double residual  = std::remainder(measured - reference, kTwoPi);
            worst                  = std::max(worst, std::abs(residual));
            last                   = reference - carrierAngle;
        }

        std::println("qa_FrequencySchedule: the mixed tone follows the schedule's integral to {:.3e} rad over {} samples of {:.1f} rad total", worst, kSamples, last);
        expect(lt(worst, kTolerance)) << "the composition's phase left the schedule's integral";
        expect(gt(std::abs(last), 60'000.)) << "a run short enough that drift could hide is not a test of drift";

        // The increments carry the schedule alone. The same phasor fed a flat increment must not reproduce them.
        // Otherwise the assertion above would pass on a kernel that ignored its input.
        std::vector<double>              flat(kSamples, kTwoPi * 5'000. / 48'000.);
        std::vector<std::complex<float>> flatOut(kSamples);
        Phasor<float>                    other;
        other.configure(0., 0.);
        other.mixModulated(std::span<const double>(flat), in, flatOut);
        expect(that % !std::ranges::equal(flatOut, out)) << "a constant offset is not this schedule";
    };

    "an empty output span is a no-op"_test = [] {
        const std::vector<std::int64_t> times{0LL, 1'000LL};
        const std::vector<double>       offsets{0., 1.};
        const FrequencySchedule         schedule(times, offsets);
        const SampleClock               clock(0ULL, 0LL, 48'000ULL, 1ULL);

        expect(nothrow([&] { schedule.phaseIncrementsFor(clock, 0ULL, std::span<double>{}); }));
        expect(nothrow([&] { schedule.valuesFor(clock, 0ULL, std::span<double>{}); }));
    };

    "valuesFor walks the same cursor a point evaluation would search"_test = [] {
        const std::vector<std::int64_t> times{0LL, 3'000'000LL, 7'000'000LL, 25'000'000LL};
        const std::vector<double>       offsets{-1'000., 500., 500., -250.};
        const FrequencySchedule         schedule(times, offsets);
        const SampleClock               clock(0ULL, -2'000'000LL, 48'000ULL, 1ULL);

        constexpr std::size_t kSamples = 4'096UZ;
        std::vector<double>   walked(kSamples);
        schedule.valuesFor(clock, 0ULL, walked);

        // Against the independently written oracle the agreement is numerical. The oracle weights the two knots, and
        // the kernel adds a slope. Both reach the same line and differ in the last bits.
        double worst  = 0.;
        bool   sameAs = true;
        for (std::size_t k = 0UZ; k < kSamples; ++k) {
            worst  = std::max(worst, std::abs(walked[k] - oracleOffset(times, offsets, clock.timeOf(k))));
            sameAs = sameAs && walked[k] == schedule.offsetAt(clock.timeOf(k));
        }
        expect(lt(worst, 1e-9)) << "the cursor walk is the search; worst difference" << worst;
        expect(sameAs) << "and it is the bit-identical answer a per-sample point evaluation gives";

        // The span is a pure function of the absolute index, so a chunked caller reads the same values.
        std::vector<double> chunked(kSamples);
        for (std::size_t at = 0UZ; at < kSamples; at += 7UZ) {
            const std::size_t n = std::min(7UZ, kSamples - at);
            schedule.valuesFor(clock, at, std::span<double>(chunked).subspan(at, n));
        }
        expect(that % std::ranges::equal(walked, chunked)) << "seven at a time reads what one call reads";
    };

    "a delay schedule is the same table under a different unit"_test = [] {
        const std::vector<std::int64_t> times{0LL, 1'000'000'000LL, 3'000'000'000LL};
        const std::vector<double>       delays{8.339e-3, 6.0e-3, 2.5e-3};
        const DelaySchedule             schedule(times, delays);

        expect(schedule.size() == 3UZ);
        expect(schedule.delayAt(-1LL) == delays.front()) << "the first value holds before the table";
        expect(schedule.delayAt(4'000'000'000LL) == delays.back()) << "the last value holds after it";
        expect(schedule.delayAt(500'000'000LL) == 0.5 * (delays[0] + delays[1])) << "the midpoint of a segment is the mean of its ends";
        expect(schedule.maxValue() == delays.front());
        expect(schedule.minValue() == delays.back());

        // The slope is the segment's, clamped to a real segment at both ends. Past the table it is the end segment's
        // slope and not the zero slope of the held value.
        expect(std::abs(schedule.segmentSlopePerSecond(500'000'000LL) - (delays[1] - delays[0])) < 1e-18) << "the first segment falls 2.339 ms in one second";
        expect(schedule.segmentSlopePerSecond(9'000'000'000LL) == schedule.segmentSlopePerSecond(2'000'000'000LL)) << "past the table the last segment's slope stands";

        std::string refusal;
        try {
            const std::vector<double> broken{1., std::numeric_limits<double>::infinity(), 2.};
            (void)DelaySchedule(times, broken);
        } catch (const gr::exception& refused) {
            refusal = refused.message;
        }
        expect(that % refusal.contains("DelaySchedule") && refusal.contains("delay inf")) << "a non-finite delay is refused, in the delay schedule's own words:" << refusal;
    };
};

int main() { /* not needed for UT */ }
