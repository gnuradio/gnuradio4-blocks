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

#include <gnuradio-4.0/algorithm/filter/FractionalDelay.hpp>
#include <gnuradio-4.0/algorithm/timing/FrequencySchedule.hpp>

using namespace boost::ut;
using gr::filter::arbitraryInterpolationError;
using gr::filter::designFractionalDelay;
using gr::filter::FractionalDelayLine;
using gr::filter::fractionalDelayQ32;
using gr::filter::kArbitraryOne;
using gr::filter::kMaxFractionalDelaySamples;
using gr::timing::DelaySchedule;
using gr::timing::kSpeedOfLight;
using gr::timing::offsetFor;
using gr::timing::SampleClock;

namespace {

constexpr double kTwoPi   = 2. * std::numbers::pi_v<double>;
constexpr double kRolloff = 0.2;

using Complex = std::complex<float>;

/// A unit tone at @p f0 cycles per sample. A fractional delay's contract is written against this signal. A delay of `d`
/// samples is a phase rotation of `-2*pi*f0*d` alone.
[[nodiscard]] std::vector<Complex> tone(double f0, std::size_t n) {
    std::vector<Complex> out(n);
    for (std::size_t k = 0UZ; k < n; ++k) {
        const double phase = kTwoPi * f0 * static_cast<double>(k);
        out[k]             = Complex{static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))};
    }
    return out;
}

/// The prototype's delivered passband ripple as a linear amplitude.
[[nodiscard]] double rippleFloor(double rippleDb) { return std::pow(10., rippleDb / 20.) - 1.; }

/// The prototype's delivered stopband as a linear amplitude. The tone's own images land there. It is the third term of
/// the envelope, beside the interpolation bound and the ripple, and not a fourth term.
[[nodiscard]] double stopbandFloor(double stopbandDb) { return std::pow(10., stopbandDb / 20.); }

/// The slope of a linear fit through @p y against its own index, least squares, closed form.
[[nodiscard]] double slopeOf(std::span<const double> y) {
    const double n     = static_cast<double>(y.size());
    const double meanX = 0.5 * (n - 1.);
    double       meanY = 0.;
    for (const double value : y) {
        meanY += value;
    }
    meanY /= n;

    double covariance = 0.;
    double variance   = 0.;
    for (std::size_t k = 0UZ; k < y.size(); ++k) {
        const double dx = static_cast<double>(k) - meanX;
        covariance += dx * (y[k] - meanY);
        variance += dx * dx;
    }
    return covariance / variance;
}

} // namespace

const boost::ut::suite<"FractionalDelay"> _fractional_delay = [] {
    "a bank of no arms is an error"_test = [] {
        const auto design = designFractionalDelay(0UZ);
        expect(fatal(!design.has_value()));
        expect(design.error().message.contains("at least one arm")) << design.error().message;
    };

    "the length estimate stays under the designed length and within a factor of it, from 20 to 140 dB"_test = [] {
        // Below 45 dB the ripple limit sets the length, and a rolloff above 0.7 has its own factors. The rows cover each
        // range with banks of 16 and 32 arms, which keep the designs short.
        struct Row {
            std::size_t bank;
            double      rolloff;
            double      mostDb;
        };
        bool allOk = true;
        bool under = true;
        bool close = true;
        for (const Row& row : {Row{16UZ, 0.7, 140.}, Row{16UZ, 0.8, 45.}, Row{16UZ, 0.85, 45.}, Row{16UZ, 0.9, 45.}, Row{16UZ, 0.99, 140.}, Row{32UZ, 0.95, 45.}, Row{32UZ, 0.99, 45.}}) {
            for (const double db : {20., 25., 30., 35., 40., 45., 140.}) {
                if (db > row.mostDb) {
                    continue;
                }
                const gr::filter::ResamplerDesign design   = designFractionalDelay(row.bank, row.rolloff, db).value();
                const double                      estimate = static_cast<double>(gr::filter::fractionalDelayTapEstimate(row.bank, row.rolloff, db));
                const double                      length   = static_cast<double>(design.taps.size());
                allOk                                      = allOk && design.ok;
                under                                      = under && estimate <= length * 1.005 + 1.;
                close                                      = close && length <= 8. * estimate;
            }
        }
        expect(that % allOk) << "every design delivers its target";
        expect(that % under) << "the estimate does not pass the designed length, to half a percent and a tap";
        expect(that % close) << "and the design is within 8 times the estimate";
    };

    "the reach check refuses what the design cannot hold and nothing it can"_test = [] {
        using gr::filter::fractionalDelayReachable;
        using gr::filter::fractionalDelayTapEstimate;

        // Designs seen to succeed or to fail at the cap: the estimate and the check agree with them at 20, 60 and 140 dB.
        expect(that % (fractionalDelayReachable(1024UZ, kRolloff, 60.0) && !fractionalDelayReachable(2048UZ, kRolloff, 60.0))) << "60 dB: 1024 arms fit, 2048 do not";
        expect(that % (fractionalDelayReachable(512UZ, kRolloff, 140.0) && !fractionalDelayReachable(1024UZ, kRolloff, 140.0))) << "140 dB: 512 arms fit, 1024 do not";
        expect(that % (fractionalDelayReachable(32UZ, 0.009011, 140.0) && !fractionalDelayReachable(32UZ, 0.008984, 140.0))) << "140 dB near the cap: the first design is found, the second falls short of its stopband";
        expect(that % (fractionalDelayReachable(256UZ, kRolloff, 20.0) && !fractionalDelayReachable(1024UZ, kRolloff, 20.0))) << "20 dB: the ripple limit makes 1024 arms too long where its estimate alone is 8559 taps";

        // Designs the read found at a wide rolloff and 20 dB, which an estimate scaled for a narrow rolloff refused: 2048
        // arms design 5.8k taps and 8192 arms 19.3k. At a rolloff of 0.7 and below, 2048 arms at 20 dB design 61k to 65k
        // taps.
        expect(that % (fractionalDelayReachable(2048UZ, 0.99, 20.0) && fractionalDelayReachable(8192UZ, 0.99, 20.0))) << "20 dB, rolloff 0.99: 2048 and 8192 arms design";
        expect(that % (fractionalDelayReachable(2048UZ, 0.6, 20.0) && fractionalDelayReachable(2048UZ, 0.7, 20.0))) << "20 dB, rolloff 0.6 and 0.7: 2048 arms design";

        // From 45 dB up no rolloff below one holds 16384 arms, and 8192 fit at the attenuation that needs least.
        bool noneHold = true;
        for (const double db : {45., 50., 60., 80., 100., 120., 140.}) {
            noneHold = noneHold && !fractionalDelayReachable(16384UZ, 0.99, db);
        }
        expect(that % noneHold) << "16384 arms";
        expect(that % fractionalDelayReachable(8192UZ, 0.99, 45.0)) << "8192 arms at 45 dB and rolloff 0.99";
        expect(that % fractionalDelayReachable(16384UZ, 0.99, 20.0)) << "16384 arms at 20 dB and rolloff 0.99 are about twice the 8192 that design";

        // Inputs the estimate cannot form give zero, and an extreme one gives a length past the cap without a conversion error.
        const double nan = std::numeric_limits<double>::quiet_NaN();
        expect(that % (fractionalDelayTapEstimate(0UZ, kRolloff, 60.0) == 0UZ && fractionalDelayTapEstimate(32UZ, nan, 60.0) == 0UZ && fractionalDelayTapEstimate(32UZ, kRolloff, nan) == 0UZ && fractionalDelayTapEstimate(32UZ, 0.0, 60.0) == 0UZ)) << "no arms, a rolloff or attenuation that is not a number, or no rolloff";
        expect(that % (!fractionalDelayReachable(0UZ, kRolloff, 60.0) && !fractionalDelayReachable(32UZ, nan, 60.0) && !fractionalDelayReachable(32UZ, kRolloff, nan))) << "and none of them is reachable";
        expect(that % (fractionalDelayTapEstimate(32768UZ, 1e-300, 1e300) > gr::filter::kFractionalDelayMaxTaps && !fractionalDelayReachable(32768UZ, 1e-300, 1e300))) << "extreme values are past the cap";
    };

    "the error bound is the resampler's, at the sizes the table states"_test = [] {
        // The five rows of the closed form. A change to it fails here, before it silently widens the envelope of a
        // delay measurement.
        struct Row {
            std::size_t bank;
            double      order0;
            double      order1;
            double      order3;
        };
        constexpr Row rows[] = {{8UZ, 1.571e-1, 1.234e-2, 2.283e-4}, {16UZ, 7.854e-2, 3.084e-3, 1.427e-5}, {32UZ, 3.927e-2, 7.711e-4, 8.918e-7}, {64UZ, 1.964e-2, 1.928e-4, 5.574e-8}, {128UZ, 9.818e-3, 4.819e-5, 3.484e-9}};

        for (const Row& row : rows) {
            expect(approx(arbitraryInterpolationError(row.bank, kRolloff, 0), row.order0, 1e-3 * row.order0)) << "L =" << row.bank << "at order 0";
            expect(approx(arbitraryInterpolationError(row.bank, kRolloff, 1), row.order1, 1e-3 * row.order1)) << "L =" << row.bank << "at order 1";
            expect(approx(arbitraryInterpolationError(row.bank, kRolloff, 3), row.order3, 1e-3 * row.order3)) << "L =" << row.bank << "at order 3";
        }
    };

    "a constant delay is a phase rotation, inside the bound the bank was cut for"_test = [] {
        constexpr std::size_t kSamples = 4'096UZ;

        struct Arm {
            std::size_t bank;
            int         order;
        };
        constexpr Arm arms[] = {{8UZ, 1}, {32UZ, 1}, {32UZ, 3}, {128UZ, 1}};

        for (const Arm& arm : arms) {
            const gr::filter::ResamplerDesign design = designFractionalDelay(arm.bank, kRolloff).value();
            const double                      interp = arbitraryInterpolationError(arm.bank, kRolloff, arm.order);
            const double                      bound  = interp + rippleFloor(design.rippleDb) + stopbandFloor(design.stopbandDb);

            FractionalDelayLine<Complex> line(arm.bank, arm.order, design.taps, 16ULL);
            const double                 latency = line.latencySamples();

            double worst = 0.;
            for (const double f0 : {0.05, 0.15, 0.25, 0.35}) {
                for (const double delay : {0., 0.25, 0.5, 0.75, 1.5, 7.125}) {
                    const std::vector<Complex> in = tone(f0, kSamples);
                    std::vector<std::uint64_t> delays(kSamples, fractionalDelayQ32(delay).value());
                    std::vector<Complex>       out(kSamples);

                    line.reset();
                    expect(line.process(in, delays, out).has_value());

                    // The first `history + tapsPerArm` outputs are the filter's transient over the zeroed history.
                    const std::size_t settled = line.historySamples() + line.tapsPerArm();
                    for (std::size_t k = settled; k < kSamples; ++k) {
                        const double  phase = kTwoPi * f0 * (static_cast<double>(k) - latency - delay);
                        const Complex ideal{static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))};
                        worst = std::max(worst, static_cast<double>(std::abs(out[k] - ideal)));
                    }
                }
            }
            std::println("L = {:3}, q = {}: worst deviation {:.6e} against the envelope {:.6e} = interpolation {:.6e} + ripple {:.6e} + stopband {:.6e}, over {} taps", arm.bank, arm.order, worst, bound, interp, rippleFloor(design.rippleDb), stopbandFloor(design.stopbandDb), design.taps.size());
            expect(lt(worst, bound)) << std::format("L = {}, q = {} misses its own bound", arm.bank, arm.order);
        }
    };

    "the consistency identity: a range rate is a frequency shift, at the ratio of the two carriers"_test = [] {
        // At the kernel, a range rate `v` delays the envelope by `tau(t) = tau0 + v*t/c`. A baseband tone
        // at `f_b` then emerges at `f_b*(1 - v/c)`, and the derivation is exact, not first order. The same `v` moves a
        // carrier at `f_c` by `offsetFor(v, f_c)`. The two shifts are therefore in the ratio `f_b/f_c`. The ratio
        // assertion fails if either sign is wrong and shows which.
        constexpr double      kSampleRate = 48'000.;
        constexpr double      kToneHz     = 12'000.;
        constexpr double      kCarrierHz  = 437e6;
        constexpr double      kRangeRate  = -7'000.;
        constexpr std::size_t kSamples    = 480'000UZ; // 10 s
        constexpr double      kDelay0     = 20.;

        const double f0     = kToneHz / kSampleRate;
        const double perTap = kRangeRate / kSpeedOfLight; // dtau/dt, and therefore delay samples per sample

        const gr::filter::ResamplerDesign design = designFractionalDelay(32UZ, kRolloff).value();
        FractionalDelayLine<Complex>      line(32UZ, 1, design.taps, 21ULL);

        const std::vector<Complex> in = tone(f0, kSamples);
        std::vector<std::uint64_t> delays(kSamples);
        for (std::size_t k = 0UZ; k < kSamples; ++k) {
            delays[k] = fractionalDelayQ32(kDelay0 + perTap * static_cast<double>(k)).value();
        }
        std::vector<Complex> out(kSamples);
        expect(line.process(in, delays, out).has_value());

        const std::size_t   settled = line.historySamples() + line.tapsPerArm();
        std::vector<double> residual(kSamples - settled);
        double              turns = 0.;
        double              last  = 0.;
        for (std::size_t k = settled; k < kSamples; ++k) {
            const double  phase = kTwoPi * f0 * static_cast<double>(k);
            const Complex reference{static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))};
            const double  wrapped = static_cast<double>(std::arg(out[k] * std::conj(reference)));
            if (k > settled) {
                const double step = wrapped - last;
                turns += (step > std::numbers::pi) ? -kTwoPi : ((step < -std::numbers::pi) ? kTwoPi : 0.);
            }
            last                  = wrapped;
            residual[k - settled] = wrapped + turns;
        }

        // The residual phase is `-2*pi*f0*(latency + D0 + (v/c)*k)`, so its slope names the shift directly.
        const double measuredHz = slopeOf(residual) / kTwoPi * kSampleRate;
        const double expectedHz = -kToneHz * kRangeRate / kSpeedOfLight;
        const double carrierHz  = offsetFor(kRangeRate, kCarrierHz);

        std::println("tone shift {:.7f} Hz against the closed form {:.7f}; carrier shift {:.4f} Hz; ratio {:.6e} against f_b/f_c = {:.6e}", measuredHz, expectedHz, carrierHz, measuredHz / carrierHz, kToneHz / kCarrierHz);
        expect(approx(measuredHz, expectedHz, 0.01 * expectedHz)) << "the envelope's own shift is f_b*(1 - v/c)";
        expect(approx(measuredHz / carrierHz, kToneHz / kCarrierHz, 0.01 * kToneHz / kCarrierHz)) << "and it stands to the carrier's in the ratio of the two frequencies";
        expect(gt(carrierHz, 0.)) << "a closing pass reads high";
    };

    "a delay schedule drives the line through one span call"_test = [] {
        // The path a block takes. The schedule's own cursor walk writes seconds. One conversion makes them Q32 samples.
        // The line sees no `double` delay.
        constexpr double      kSampleRate = 48'000.;
        constexpr std::size_t kSamples    = 8'192UZ;

        const std::vector<std::int64_t> times{0LL, 200'000'000LL};
        const std::vector<double>       seconds{8.339e-3, 6.0e-3};
        const DelaySchedule             schedule(times, seconds);
        const SampleClock               clock(0ULL, 0LL, 48'000ULL, 1ULL);

        std::vector<double> walked(kSamples);
        schedule.valuesFor(clock, 0ULL, walked);

        std::vector<std::uint64_t> delays(kSamples);
        expect(fatal(fractionalDelayQ32(walked, kSampleRate, delays).has_value()));
        expect(delays.front() == fractionalDelayQ32(8.339e-3 * kSampleRate).value()) << "the first sample carries the first knot";
        expect(delays.front() > delays.back()) << "and the table's fall shows up as a fall in the delay";

        const std::uint64_t maxWhole = delays.front() >> 32;
        expect(maxWhole == 400ULL) << "8.339 ms at 48 kS/s is 400 whole samples, got" << maxWhole;

        const gr::filter::ResamplerDesign design = designFractionalDelay(32UZ, kRolloff).value();
        FractionalDelayLine<Complex>      line(32UZ, 1, design.taps, maxWhole);
        expect(line.historySamples() == maxWhole + line.tapsPerArm() + 1UZ);

        const std::vector<Complex> in = tone(0.1, kSamples);
        std::vector<Complex>       out(kSamples);
        expect(line.process(in, delays, out).has_value());
    };

    "the stream does not depend on how it is chunked"_test = [] {
        constexpr std::size_t kSamples = 20'000UZ;

        const gr::filter::ResamplerDesign design = designFractionalDelay(32UZ, kRolloff).value();

        const std::vector<Complex> in = tone(0.17, kSamples);
        std::vector<std::uint64_t> delays(kSamples);
        for (std::size_t k = 0UZ; k < kSamples; ++k) {
            // A delay that moves within a chunk and across chunk boundaries. A per-call reset of the read position
            // would show up and would not cancel.
            delays[k] = fractionalDelayQ32(30. + 12. * std::sin(kTwoPi * static_cast<double>(k) / 7'000.)).value();
        }

        for (const int order : {0, 1, 3}) {
            FractionalDelayLine<Complex> whole(32UZ, order, design.taps, 64ULL);
            std::vector<Complex>         reference(kSamples);
            expect(whole.process(in, delays, reference).has_value());

            for (const std::size_t chunk : {1UZ, 7UZ, 1'000UZ, 12'345UZ}) {
                FractionalDelayLine<Complex> pieced(32UZ, order, design.taps, 64ULL);
                std::vector<Complex>         got(kSamples);
                for (std::size_t at = 0UZ; at < kSamples; at += chunk) {
                    const std::size_t n = std::min(chunk, kSamples - at);
                    expect(pieced.process(std::span<const Complex>(in).subspan(at, n), std::span<const std::uint64_t>(delays).subspan(at, n), std::span<Complex>(got).subspan(at, n)).has_value());
                }
                expect(that % std::ranges::equal(reference, got)) << std::format("order {}, chunk {} changed the stream", order, chunk);
            }
        }
    };

    "a reach grown mid-stream keeps the history, and new taps clear it"_test = [] {
        constexpr std::size_t   kHalf     = 4'000UZ;
        constexpr std::uint64_t kShort    = 16ULL;
        constexpr std::uint64_t kLong     = 400ULL;
        constexpr double        kNear     = 7.25;
        constexpr double        kFar      = 300.5;
        constexpr std::size_t   kSwitchAt = 2UZ * static_cast<std::size_t>(kLong);

        const gr::filter::ResamplerDesign design = designFractionalDelay(32UZ, kRolloff).value();
        const std::vector<Complex>        in     = tone(0.13, 2UZ * kHalf);

        // Delays inside the short reach while it is the line's reach. After the grow, the near delay holds until the
        // longer history has filled with samples the grown line has seen, and then the far delay reaches back into it.
        std::vector<std::uint64_t> delays(2UZ * kHalf, fractionalDelayQ32(kNear).value());
        std::fill(delays.begin() + static_cast<std::ptrdiff_t>(kHalf + kSwitchAt), delays.end(), fractionalDelayQ32(kFar).value());

        for (const int order : {0, 1, 3}) {
            FractionalDelayLine<Complex> wide(32UZ, order, design.taps, kLong);
            std::vector<Complex>         reference(in.size());
            expect(fatal(wide.process(in, delays, reference).has_value()));

            FractionalDelayLine<Complex> grown(32UZ, order, design.taps, kShort);
            std::vector<Complex>         got(in.size());
            expect(fatal(grown.process(std::span<const Complex>(in).first(kHalf), std::span<const std::uint64_t>(delays).first(kHalf), std::span<Complex>(got).first(kHalf)).has_value()));
            expect(fatal(grown.growMaxDelay(kLong).has_value()));
            expect(eq(grown.maxDelaySamples(), kLong));
            expect(eq(grown.historySamples(), wide.historySamples())) << "the grown line holds the history a line of that reach holds";
            expect(fatal(grown.process(std::span<const Complex>(in).subspan(kHalf), std::span<const std::uint64_t>(delays).subspan(kHalf), std::span<Complex>(got).subspan(kHalf)).has_value()));

            expect(that % std::ranges::equal(reference, got)) << std::format("order {}: the grown line matches a line built with the long reach", order);

            // New taps clear the history. The next call then matches a fresh line fed the same samples.
            FractionalDelayLine<Complex> fresh(32UZ, order, design.taps, kLong);
            std::vector<Complex>         afterFresh(kHalf);
            std::vector<Complex>         afterTaps(kHalf);
            const auto                   tail      = std::span<const Complex>(in).subspan(kHalf);
            const auto                   tailDelay = std::span<const std::uint64_t>(delays).first(kHalf);
            expect(fatal(fresh.process(tail, tailDelay, afterFresh).has_value()));
            expect(fatal(grown.setTaps(design.taps).has_value()));
            expect(fatal(grown.process(tail, tailDelay, afterTaps).has_value()));
            expect(that % std::ranges::equal(afterFresh, afterTaps)) << std::format("order {}: a line given new taps starts on silence", order);
        }
    };

    "the accessors state the lag, and the history follows the delay"_test = [] {
        const gr::filter::ResamplerDesign design = designFractionalDelay(32UZ, kRolloff).value();
        FractionalDelayLine<Complex>      line(32UZ, 1, design.taps, 400ULL);

        expect(line.bankSize() == 32UZ);
        expect(line.order() == 1);
        expect(line.prototypeLength() == design.taps.size());
        expect(line.tapsPerArm() == (design.taps.size() + 31UZ) / 32UZ);
        expect(approx(line.groupDelaySamples(), 0.5 * static_cast<double>(design.taps.size() - 1UZ) / 32., 1e-12));
        expect(approx(line.latencySamples(), 1. + line.groupDelaySamples(), 1e-12)) << "the extra sample is the wrap arm's reach, and it is stated";
        expect(line.historySamples() == 400UZ + line.tapsPerArm() + 1UZ);
        expect(line.maxDelaySamples() == 400ULL);

        std::println("L = 32, q = 1 at a 60 dB design: {} taps, {} per arm, group delay {:.4f} samples, latency {:.4f}, history {} samples", design.taps.size(), line.tapsPerArm(), line.groupDelaySamples(), line.latencySamples(), line.historySamples());
    };

    "what the line refuses"_test = [] {
        const gr::filter::ResamplerDesign design = designFractionalDelay(8UZ, kRolloff).value();

        expect(throws<gr::exception>([&] { (void)FractionalDelayLine<Complex>(0UZ, 1, design.taps); })) << "a bank with no arms";
        expect(throws<gr::exception>([&] { (void)FractionalDelayLine<Complex>(8UZ, 2, design.taps); })) << "an order that is not 0, 1 or 3";
        expect(throws<gr::exception>([&] { (void)FractionalDelayLine<Complex>(8UZ, 1, std::span<const float>{}); })) << "no taps";

        FractionalDelayLine<Complex> line(8UZ, 1, design.taps, 4ULL);
        const std::vector<Complex>   in(64UZ);
        std::vector<Complex>         out(64UZ);

        const auto noTaps = line.setTaps(std::span<const float>{});
        expect(fatal(!noTaps.has_value())) << "a setting with no taps";
        expect(noTaps.error().message.contains("no taps")) << noTaps.error().message;
        const auto tooLong = line.setMaxDelay(kMaxFractionalDelaySamples);
        expect(fatal(!tooLong.has_value())) << "a history the fixed point cannot carry";
        expect(tooLong.error().message.contains(std::to_string(kMaxFractionalDelaySamples))) << tooLong.error().message;
        const auto tooFar = line.growMaxDelay(kMaxFractionalDelaySamples);
        expect(fatal(!tooFar.has_value())) << "a reach the fixed point cannot carry";
        expect(tooFar.error().message.contains(std::to_string(kMaxFractionalDelaySamples))) << tooFar.error().message;
        expect(eq(line.maxDelaySamples(), 4ULL)) << "a refused setting leaves the line as it was";
        expect(eq(line.prototypeLength(), design.taps.size())) << "and refused taps leave the prototype";
        expect(line.growMaxDelay(2ULL).has_value() && eq(line.maxDelaySamples(), 4ULL)) << "a reach the line already covers is taken and changes nothing";

        const auto unpaired = line.process(in, std::vector<std::uint64_t>(63UZ), out);
        expect(fatal(!unpaired.has_value())) << "one delay short of the samples";
        expect(unpaired.error().message.contains("each sample needs one delay")) << unpaired.error().message;
        expect(!line.process(in, std::vector<std::uint64_t>(64UZ), std::span<Complex>(out).first(63UZ)).has_value()) << "one output short";
        expect(!line.process(in, std::vector<std::uint64_t>(64UZ, fractionalDelayQ32(5.).value()), out).has_value()) << "a delay past the history the line was sized for";
        expect(line.process(in, std::vector<std::uint64_t>(64UZ, fractionalDelayQ32(4.).value()), out).has_value()) << "and the bound itself is admitted";

        const auto negative = fractionalDelayQ32(-1.);
        expect(fatal(!negative.has_value())) << "a negative delay is a read from the future";
        expect(negative.error().message.contains("not a finite non-negative number")) << negative.error().message;
        expect(!fractionalDelayQ32(std::numeric_limits<double>::quiet_NaN()).has_value());
        expect(!fractionalDelayQ32(static_cast<double>(kMaxFractionalDelaySamples)).has_value()) << "2^31 samples is where the fixed point runs out";
        expect(fractionalDelayQ32(static_cast<double>(kMaxFractionalDelaySamples) - 1.).has_value());

        std::vector<std::uint64_t> two(2UZ);
        const auto                 unpairedSpan = fractionalDelayQ32(std::vector<double>{1e-3, 2e-3, 3e-3}, 48'000., two);
        expect(fatal(!unpairedSpan.has_value())) << "the span conversion is paired too";
        expect(unpairedSpan.error().message.contains("3 delays against 2 outputs")) << unpairedSpan.error().message;
        expect(!fractionalDelayQ32(std::vector<double>{1e-3, 2e-3}, -1., two).has_value()) << "a rate that is not positive";
        expect(!fractionalDelayQ32(std::vector<double>{1e-3, -2e-3}, 48'000., two).has_value()) << "a refused delay inside the span";
    };

    "the fixed point is exact where it says it is"_test = [] {
        expect(fractionalDelayQ32(0.).value() == 0ULL);
        expect(fractionalDelayQ32(1.).value() == kArbitraryOne);
        expect(fractionalDelayQ32(0.5).value() == kArbitraryOne / 2ULL);
        expect(fractionalDelayQ32(7.125).value() == 7ULL * kArbitraryOne + kArbitraryOne / 8ULL) << "an eighth is a whole number of Q32 steps";

        // The claim in the header. A `double`'s spacing is seven orders below what the representation keeps. Two delays
        // a `2^-32` sample apart land on adjacent fixed-point values, not on the same one.
        constexpr double kStep = 1. / 4'294'967'296.;
        expect(fractionalDelayQ32(3. + kStep).value() - fractionalDelayQ32(3.).value() == 1ULL);
    };
};

int main() { /* not needed for UT */ }
