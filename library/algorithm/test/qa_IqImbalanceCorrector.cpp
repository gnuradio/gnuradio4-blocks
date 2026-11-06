#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <random>
#include <span>
#include <tuple>
#include <utility>
#include <vector>

#include <gnuradio-4.0/algorithm/channel/IqImbalanceCorrector.hpp>

namespace {

using gr::channel::IqImbalanceCorrector;

constexpr double kDegree = std::numbers::pi / 180.0;

/// A real 2x2 map of `(I, Q)`: `I' = ii * I + iq * Q`, `Q' = qi * I + qq * Q`.
struct Map {
    double ii;
    double iq;
    double qi;
    double qq;

    [[nodiscard]] Map after(const Map& first) const { return {ii * first.ii + iq * first.qi, ii * first.iq + iq * first.qq, qi * first.ii + qq * first.qi, qi * first.iq + qq * first.qq}; }

    /// The image of the map, `20 * log10(|B| / |A|)` for `y = A * x + B * conj(x)`.
    [[nodiscard]] double imageDb() const {
        const std::complex<double> a(0.5 * (ii + qq), 0.5 * (qi - iq));
        const std::complex<double> b(0.5 * (ii - qq), 0.5 * (qi + iq));
        return 20.0 * std::log10(std::abs(b) / std::abs(a));
    }
};

/// The receiver of the model: `I = I0` and `Q = g * (Q0 * cos(phi) - I0 * sin(phi))`.
[[nodiscard]] Map receiver(double gainDb, double phaseDeg) {
    const double g = std::pow(10.0, gainDb / 20.0);
    return {1.0, 0.0, -g * std::sin(phaseDeg * kDegree), g * std::cos(phaseDeg * kDegree)};
}

template<typename R>
[[nodiscard]] Map correction(const IqImbalanceCorrector<R>& corrector) {
    return {1.0, 0.0, static_cast<double>(corrector.shear()), static_cast<double>(corrector.scale())};
}

template<typename R>
void passThrough(const Map& map, std::vector<std::complex<R>>& x) {
    for (std::complex<R>& v : x) {
        const double i = static_cast<double>(v.real());
        const double q = static_cast<double>(v.imag());
        v              = std::complex<R>(static_cast<R>(map.ii * i + map.iq * q), static_cast<R>(map.qi * i + map.qq * q));
    }
}

/// The image of a receiver whose imbalance would give @p y its powers and correlation. A corrected stream of circular
/// content has none.
template<typename R>
[[nodiscard]] double imageOfStream(std::span<const std::complex<R>> y) {
    double powerI  = 0.0;
    double powerQ  = 0.0;
    double product = 0.0;
    for (const std::complex<R>& v : y) {
        powerI += static_cast<double>(v.real()) * static_cast<double>(v.real());
        powerQ += static_cast<double>(v.imag()) * static_cast<double>(v.imag());
        product += static_cast<double>(v.real()) * static_cast<double>(v.imag());
    }
    return receiver(10.0 * std::log10(powerQ / powerI), std::asin(-product / std::sqrt(powerI * powerQ)) / kDegree).imageDb();
}

/// Circular complex Gaussian noise of @p sigma per component.
template<typename R>
[[nodiscard]] std::vector<std::complex<R>> noise(std::size_t n, std::uint32_t seed, double sigma) {
    std::mt19937                     rng(seed);
    std::normal_distribution<double> gauss(0.0, sigma);
    std::vector<std::complex<R>>     x(n);
    for (std::complex<R>& v : x) {
        v = std::complex<R>(static_cast<R>(gauss(rng)), static_cast<R>(gauss(rng)));
    }
    return x;
}

/// Runs @p corrector over @p x in calls of 1, 7, 1000, 4096, 333 and 1024 samples in turn. Runs then end inside calls,
/// at their ends and in their tails. Returns the corrected stream.
template<typename R>
[[nodiscard]] std::vector<std::complex<R>> drive(IqImbalanceCorrector<R>& corrector, std::span<const std::complex<R>> x) {
    constexpr std::array<std::size_t, 6UZ> kCalls{1UZ, 7UZ, 1000UZ, 4096UZ, 333UZ, 1024UZ};
    std::vector<std::complex<R>>           y(x.size());
    std::size_t                            call = 0UZ;
    for (std::size_t at = 0UZ; at < x.size(); call = (call + 1UZ) % kCalls.size()) {
        const std::size_t count = std::min(kCalls[call], x.size() - at);
        std::ignore             = corrector.process(x.subspan(at, count), std::span<std::complex<R>>(y).subspan(at, count));
        at += count;
    }
    return y;
}

// The estimate's own scatter sets the bounds below. With `tau` samples of averaging the residual image on noise alone
// is `|b|^2 = V / (4 * tau)`, where `V` is 1.5 for the differenced noise: -52.4 dBc rms at `tau = 2^16`. Its power
// follows an exponential distribution. A bound 12 dB above the rms fails once in about 8 million runs.
constexpr double kTau        = 65536.0;
constexpr double kImageBound = -40.0;

} // namespace

const boost::ut::suite<"IqImbalanceCorrector"> iqImbalanceCorrectorTests = [] {
    using namespace boost::ut;

    "the correction removes a frequency-independent imbalance it estimates from circular noise"_test = []<typename R>() {
        using C = std::complex<R>;
        for (const auto& [gainDb, phaseDeg] : {std::pair{1.0, 3.0}, std::pair{-2.0, -5.0}, std::pair{0.0, 0.0}}) {
            std::vector<C> x = noise<R>(1UZ << 20, 0x1d5eedU, 1.0);
            passThrough(receiver(gainDb, phaseDeg), x);
            IqImbalanceCorrector<R> corrector;
            expect(fatal(corrector.setTimeConstant(kTau)));

            const std::span<const C> firstRun = std::span<const C>(x).first(IqImbalanceCorrector<R>::kRun - 1UZ);
            std::vector<C>           early(firstRun.size());
            expect(eq(corrector.process(firstRun, std::span<C>(early)), 0UZ));
            expect(std::ranges::equal(early, firstRun)) << "until the first run ends the output is the input";

            const std::vector<C> y      = drive(corrector, std::span<const C>(x).subspan(firstRun.size()));
            const double         before = receiver(gainDb, phaseDeg).imageDb();
            const double         after  = correction(corrector).after(receiver(gainDb, phaseDeg)).imageDb();
            expect(lt(after, kImageBound)) << std::format("{} dB, {} deg: the image goes from {:.1f} to {:.1f} dBc", gainDb, phaseDeg, before, after);
            // The estimate's standard deviation is about 0.02 dB and 0.2 degrees here. The bounds are five or more of
            // them.
            expect(approx(corrector.gainDb(), gainDb, 0.1)) << std::format("gainDb() reads {}", corrector.gainDb());
            expect(approx(corrector.phaseDeg(), phaseDeg, 1.0)) << std::format("phaseDeg() reads {}", corrector.phaseDeg());

            // The output's own powers and correlation over its last quarter, 2^18 samples, scatter by about -60 dBc.
            const double output = imageOfStream(std::span<const C>(y).last(y.size() / 4UZ));
            expect(lt(output, kImageBound)) << std::format("the output's I and Q have equal power and no correlation: {:.1f} dBc", output);
            expect(std::ranges::equal(std::span<const C>(y), std::span<const C>(x).subspan(firstRun.size()), [](C out, C in) { return out.real() == in.real(); })) << "I passes unchanged";
        }
    } | std::tuple<float, double>{};

    "the correction stops at 3 dB and 10 degrees, and the estimate does not"_test = [] {
        using C = std::complex<float>;
        for (const double sign : {1.0, -1.0}) {
            const double   gainDb   = 4.0 * sign;
            const double   phaseDeg = 12.0 * sign;
            std::vector<C> x        = noise<float>(1UZ << 19, 0xc1a3U, 1.0);
            passThrough(receiver(gainDb, phaseDeg), x);
            IqImbalanceCorrector<float> corrector;
            expect(fatal(corrector.setTimeConstant(kTau)));
            std::ignore = drive(corrector, std::span<const C>(x));

            // The limits: a power ratio of Q against I of 10^(3/10) and a correlation coefficient of sin(10 degrees).
            const double ratio = std::pow(10.0, 0.3 * sign);
            const double sine  = -std::sin(10.0 * kDegree) * sign;
            const double scale = 1.0 / std::sqrt(ratio * (1.0 - sine * sine));
            const double shear = -scale * sine * std::sqrt(ratio);
            expect(approx(static_cast<double>(corrector.scale()), scale, 1e-6 * scale) && approx(static_cast<double>(corrector.shear()), shear, 1e-6 * std::abs(shear))) << "the coefficients are the limits'";
            const double clamped = Map{1.0, 0.0, shear, scale}.after(receiver(gainDb, phaseDeg)).imageDb();
            expect(lt(clamped, receiver(gainDb, phaseDeg).imageDb() - 6.0)) << std::format("and still cut the image, from {:.1f} to {:.1f} dBc", receiver(gainDb, phaseDeg).imageDb(), clamped);
            expect(approx(corrector.gainDb(), gainDb, 0.1) && approx(corrector.phaseDeg(), phaseDeg, 1.0)) << std::format("the estimate reads {} dB and {} deg", corrector.gainDb(), corrector.phaseDeg());
        }
    };

    "a strong real-valued signal near zero frequency barely enters the estimate"_test = [] {
        // A real-valued signal of amplitude 1 along 30 degrees at 1e-4 of the rate puts `4 * sin^2(pi * 1e-4) / 2`,
        // 2e-7, into the difference, against 4e-4 from the noise: a bias near -72 dBc, under the scatter's bound.
        using C            = std::complex<float>;
        std::vector<C> x   = noise<float>(1UZ << 19, 0x5ca1eU, 0.01);
        const double   dir = 30.0 * kDegree;
        for (std::size_t n = 0UZ; n < x.size(); ++n) {
            const double s = std::cos(2.0 * std::numbers::pi * 1e-4 * static_cast<double>(n));
            x[n] += C(static_cast<float>(s * std::cos(dir)), static_cast<float>(s * std::sin(dir)));
        }
        IqImbalanceCorrector<float> corrector;
        expect(fatal(corrector.setTimeConstant(kTau)));
        std::ignore        = drive(corrector, std::span<const C>(x));
        const double image = correction(corrector).imageDb();
        expect(lt(image, kImageBound)) << std::format("the correction adds an image of {:.1f} dBc", image);
    };

    "a run with a NaN or infinite sample leaves the estimate and the coefficients as they were"_test = [] {
        using C             = std::complex<float>;
        constexpr auto kRun = IqImbalanceCorrector<float>::kRun;
        for (const float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
            std::vector<C> x = noise<float>(40UZ * kRun, 0x4a11U, 1.0);
            passThrough(receiver(1.0, 3.0), x);
            IqImbalanceCorrector<float> corrector;
            expect(fatal(corrector.setTimeConstant(4.0 * kRun)));
            std::vector<C> y(x.size());
            expect(eq(corrector.process(std::span<const C>(x).first(20UZ * kRun), std::span<C>(y).first(20UZ * kRun)), 20UZ));
            const double gain  = corrector.gainDb();
            const double phase = corrector.phaseDeg();
            const float  scale = corrector.scale();
            const float  shear = corrector.shear();

            x[20UZ * kRun + 500UZ] = C(0.1f, bad);
            expect(eq(corrector.process(std::span<const C>(x).subspan(20UZ * kRun, kRun), std::span<C>(y).subspan(20UZ * kRun, kRun)), 1UZ));
            expect(corrector.gainDb() == gain && corrector.phaseDeg() == phase && corrector.scale() == scale && corrector.shear() == shear) << "the run holding it changes nothing";

            std::ignore = corrector.process(std::span<const C>(x).subspan(21UZ * kRun), std::span<C>(y).subspan(21UZ * kRun));
            expect(std::ranges::all_of(std::span<const C>(y).subspan(21UZ * kRun), [](C v) { return std::isfinite(v.real()) && std::isfinite(v.imag()); })) << "every later output is finite";
            expect(neq(corrector.gainDb(), gain)) << "and the estimate moves again";
        }
    };

    "silence sets no coefficient"_test = [] {
        using C = std::complex<float>;
        IqImbalanceCorrector<float> corrector;
        const std::vector<C>        zeros(4UZ * IqImbalanceCorrector<float>::kRun, C{});
        std::vector<C>              y(zeros.size(), C(1.f, 1.f));
        expect(eq(corrector.process(std::span<const C>(zeros), std::span<C>(y)), 4UZ));
        expect(corrector.scale() == 1.f && corrector.shear() == 0.f) << "silence from the start leaves the correction at none";
        expect(eq(corrector.gainDb(), 0.0) && eq(corrector.phaseDeg(), 0.0)) << "and the estimate unread";
        expect(std::ranges::all_of(y, [](C v) { return v == C{}; })) << "and the output is the silence";
    };

    "a long silence after an estimate leaves the estimate and the coefficients where they were"_test = [] {
        // With a time constant of one run, a run of zeros that moved the averages would scale them by 1/e. Their
        // product P_I * P_Q would reach zero after about 370 runs, and the averages themselves after about 710. The
        // silence below lasts 800 runs.
        using C                           = std::complex<float>;
        constexpr auto        kRun        = IqImbalanceCorrector<float>::kRun;
        constexpr std::size_t kSilentRuns = 800UZ;
        std::vector<C>        x           = noise<float>(512UZ * kRun, 0x51e7U, 1.0);
        passThrough(receiver(1.0, 3.0), x);
        IqImbalanceCorrector<float> corrector;
        expect(fatal(corrector.setTimeConstant(kTau)));
        std::ignore = drive(corrector, std::span<const C>(x));

        // The first run of zeros holds the step from the last sample down to zero.
        const std::vector<C> zeros(kRun, C{});
        std::vector<C>       y(kRun);
        std::ignore        = corrector.process(std::span<const C>(zeros), std::span<C>(y));
        const double gain  = corrector.gainDb();
        const double phase = corrector.phaseDeg();
        const float  scale = corrector.scale();
        const float  shear = corrector.shear();
        expect(fatal(corrector.setTimeConstant(static_cast<double>(kRun))));
        std::size_t nonZero = 0UZ;
        for (std::size_t run = 1UZ; run < kSilentRuns; ++run) {
            std::ignore = corrector.process(std::span<const C>(zeros), std::span<C>(y));
            nonZero += static_cast<std::size_t>(std::ranges::count_if(y, [](C v) { return v != C{}; }));
        }
        expect(eq(nonZero, 0UZ)) << "every output of the silence is zero";
        expect(corrector.gainDb() == gain && corrector.phaseDeg() == phase) << std::format("the estimate stays at {} dB and {} deg: {} and {}", gain, phase, corrector.gainDb(), corrector.phaseDeg());
        expect(corrector.scale() == scale && corrector.shear() == shear) << std::format("the coefficients stay at {} and {}: {} and {}", scale, shear, corrector.scale(), corrector.shear());

        expect(fatal(corrector.setTimeConstant(kTau)));
        std::vector<C> back = noise<float>(16UZ * kRun, 0x51e8U, 1.0);
        passThrough(receiver(1.0, 3.0), back);
        std::vector<C> out(back.size());
        std::ignore = corrector.process(std::span<const C>(back), std::span<C>(out));
        expect(std::ranges::all_of(out, [](C v) { return std::isfinite(v.real()) && std::isfinite(v.imag()); })) << "every output after the signal returns is finite";
        const double after = correction(corrector).after(receiver(1.0, 3.0)).imageDb();
        expect(lt(after, kImageBound)) << std::format("and the image after the return is {:.1f} dBc", after);
    };

    "a marked gap drops the difference across it"_test = [] {
        // Samples on a grid of 2^-10 within +-8, and an offset of (64, -48), keep every sum and difference exact in
        // float. A stream and the same stream offset then give the same differences.
        using C               = std::complex<float>;
        constexpr auto kRun   = IqImbalanceCorrector<float>::kRun;
        std::vector<C> a      = noise<float>(8UZ * kRun, 0x9a9U, 1.0);
        std::vector<C> b      = noise<float>(8UZ * kRun, 0x9b9U, 1.0);
        const auto     onGrid = [](float v) { return std::clamp(std::round(v * 1024.f) / 1024.f, -8.f, 8.f); };
        for (std::vector<C>* stream : {&a, &b}) {
            for (C& v : *stream) {
                v = C(onGrid(v.real()), onGrid(v.imag()));
            }
        }
        std::vector<C> offset = b;
        for (C& v : offset) {
            v += C(64.f, -48.f);
        }
        const auto estimate = [&a](std::span<const C> second, bool marked) {
            IqImbalanceCorrector<float> corrector;
            expect(fatal(corrector.setTimeConstant(4.0 * kRun)));
            std::vector<C> y(a.size());
            std::ignore = corrector.process(std::span<const C>(a), std::span<C>(y));
            if (marked) {
                corrector.markDiscontinuity();
            }
            std::ignore = corrector.process(second, std::span<C>(y));
            return std::pair{corrector.gainDb(), corrector.phaseDeg()};
        };
        expect(estimate(std::span<const C>(b), true) == estimate(std::span<const C>(offset), true)) << "with the gap marked, an offset across it changes nothing";
        expect(estimate(std::span<const C>(b), false) != estimate(std::span<const C>(offset), false)) << "without the mark, the step across it enters the estimate";
    };

    "a time constant that gives a run no weight is refused"_test = [] {
        IqImbalanceCorrector<float> corrector;
        expect(fatal(corrector.setTimeConstant(5000.0)));
        for (const double refused : {0.0, -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
            expect(!corrector.setTimeConstant(refused)) << std::format("{} samples is refused", refused);
        }
        expect(eq(corrector.timeConstant(), 5000.0)) << "and the time constant keeps the value it had";
    };

    "the exchanged stream is corrected and estimated as the stream with I and Q swapped"_test = [] {
        using C                      = std::complex<float>;
        constexpr std::size_t kCount = 12UZ * IqImbalanceCorrector<float>::kRun + 77UZ;
        std::vector<C>        x      = noise<float>(kCount, 0x77aU, 0.1);
        passThrough(receiver(1.0, 3.0), x);
        std::vector<C> exchanged(kCount);
        std::ranges::transform(x, exchanged.begin(), [](C v) { return C(v.imag(), v.real()); });

        IqImbalanceCorrector<float> folded;
        IqImbalanceCorrector<float> apart;
        expect(fatal(folded.setTimeConstant(4096.0) && apart.setTimeConstant(4096.0)));
        std::vector<C> got(kCount);
        std::vector<C> want(kCount);
        std::vector<C> prepared(kCount);
        std::vector<C> viaPrepare(kCount);
        for (std::size_t at = 0UZ; at < kCount; at += 700UZ) { // calls that end inside runs
            const std::size_t count = std::min(700UZ, kCount - at);
            std::ignore             = folded.processExchanged(std::span<const C>(x).subspan(at, count), std::span<C>(got).subspan(at, count));
            std::ignore             = apart.process(std::span<const C>(exchanged).subspan(at, count), std::span<C>(want).subspan(at, count));
        }
        expect(got == want) << "the output equals process() on the exchanged samples, bit for bit";
        expect(folded.gainDb() == apart.gainDb() && folded.phaseDeg() == apart.phaseDeg()) << "and so does the estimate";

        IqImbalanceCorrector<float> withPrepare;
        expect(fatal(withPrepare.setTimeConstant(4096.0)));
        std::ignore = withPrepare.processExchanged(std::span<const C>(x), std::span<C>(viaPrepare), [](std::span<const C> in, std::span<C> out) { std::ranges::transform(in, out.begin(), [](C v) { return C(v.imag(), v.real()); }); });
        IqImbalanceCorrector<float> reference;
        expect(fatal(reference.setTimeConstant(4096.0)));
        std::ignore = reference.process(std::span<const C>(exchanged), std::span<C>(prepared));
        expect(viaPrepare == prepared) << "a prepare step receives the samples as they lie";
    };

    "exchange() inverts the gain, keeps the quadrature error and continues as the exchanged stream would"_test = [] {
        using C                    = std::complex<float>;
        constexpr std::size_t kRun = IqImbalanceCorrector<float>::kRun;
        std::vector<C>        x    = noise<float>(10UZ * kRun, 0x1badU, 0.1);
        passThrough(receiver(1.0, 3.0), x);
        std::vector<C> exchanged(x.size());
        std::ranges::transform(x, exchanged.begin(), [](C v) { return C(v.imag(), v.real()); });

        IqImbalanceCorrector<float> switched;
        IqImbalanceCorrector<float> steady;
        expect(fatal(switched.setTimeConstant(4096.0) && steady.setTimeConstant(4096.0)));
        std::vector<C>    y(x.size());
        std::vector<C>    z(x.size());
        const std::size_t toggle = 5UZ * kRun + 300UZ; // inside a run
        std::ignore              = switched.process(std::span<const C>(x).first(toggle), std::span<C>(y).first(toggle));
        const double gain        = switched.gainDb();
        const double phase       = switched.phaseDeg();
        switched.exchange();
        expect(approx(switched.gainDb(), -gain, 1e-12) && approx(switched.phaseDeg(), phase, 1e-12)) << "the gain inverts and the quadrature error stays";
        expect(neq(gain, 0.0));
        std::ignore = switched.processExchanged(std::span<const C>(x).subspan(toggle), std::span<C>(y).subspan(toggle));
        std::ignore = steady.processExchanged(std::span<const C>(x), std::span<C>(z));
        expect(std::equal(y.begin() + static_cast<std::ptrdiff_t>(toggle), y.end(), z.begin() + static_cast<std::ptrdiff_t>(toggle))) << "every output after it equals that of a corrector that ran exchanged from the start";
        // The call split differs at the toggle, so the float lanes add in another order: the estimates agree to a
        // rounding of a run's sums, not bit for bit.
        expect(approx(switched.gainDb(), steady.gainDb(), 1e-3) && approx(switched.phaseDeg(), steady.phaseDeg(), 1e-2)) << "and the estimates agree";
    };
};

int main() { /* not needed for UT */ }
