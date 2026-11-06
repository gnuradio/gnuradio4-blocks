#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <random>
#include <span>
#include <tuple>
#include <utility>
#include <vector>

#include <gnuradio-4.0/algorithm/filter/SinglePoleAverage.hpp>

namespace {

using gr::filter::SinglePoleAverage;

template<typename T>
using RealOf = decltype(std::real(std::declval<T>()));

/// Gaussian noise of unit rms per component around the offset `0.3 + 0.2j`, or `0.3` on a real stream.
template<typename T>
[[nodiscard]] std::vector<T> noise(std::size_t n, std::uint32_t seed) {
    std::mt19937                        rng(seed);
    std::normal_distribution<RealOf<T>> gauss;
    std::vector<T>                      x(n);
    for (T& v : x) {
        if constexpr (std::floating_point<T>) {
            v = RealOf<T>(0.3) + gauss(rng);
        } else {
            v = T(RealOf<T>(0.3) + gauss(rng), RealOf<T>(0.2) + gauss(rng));
        }
    }
    return x;
}

/// True for a @p got within one rounding of @p want. A `float` output must be @p want or the `float` next to it. A
/// `double` output must lie within the estimate's drift bound, 1e-12 of the signal @p scale.
template<std::floating_point R>
[[nodiscard]] bool withinOneRounding(R got, R want, double scale) {
    if constexpr (std::same_as<R, float>) {
        return got == want || got == std::nextafter(want, std::numeric_limits<R>::infinity()) || got == std::nextafter(want, -std::numeric_limits<R>::infinity());
    } else {
        return std::abs(got - want) <= 1e-12 * scale;
    }
}

template<typename T>
[[nodiscard]] bool withinOneRounding(T got, T want, double scale) {
    if constexpr (std::floating_point<T>) {
        return withinOneRounding<T>(got, want, scale);
    } else {
        return withinOneRounding(got.real(), want.real(), scale) && withinOneRounding(got.imag(), want.imag(), scale);
    }
}

/// Runs @p average over @p x in calls whose lengths cycle through @p calls, into `subtract()` or `average()`.
template<typename T>
[[nodiscard]] std::vector<T> drive(SinglePoleAverage<T>& average, std::span<const T> x, std::span<const std::size_t> calls, bool residual) {
    std::vector<T> y(x.size());
    std::size_t    call = 0UZ;
    for (std::size_t at = 0UZ; at < x.size(); call = (call + 1UZ) % calls.size()) {
        const std::size_t count = std::min(calls[call], x.size() - at);
        if (residual) {
            average.subtract(x.subspan(at, count), std::span<T>(y).subspan(at, count));
        } else {
            average.average(x.subspan(at, count), std::span<T>(y).subspan(at, count));
        }
        at += count;
    }
    return y;
}

} // namespace

const boost::ut::suite<"SinglePoleAverage"> singlePoleAverageTests = [] {
    using namespace boost::ut;

    "the solve eight samples at a time agrees with the recursion one sample at a time"_test = []<typename T>() {
        using R                                 = RealOf<T>;
        constexpr std::size_t          kSamples = 20000UZ;
        const std::vector<T>           x        = noise<T>(kSamples, 0x5eedU);
        const std::vector<std::size_t> calls    = {1UZ, 7UZ, 8UZ, 13UZ, 1000UZ, 64UZ, 3UZ}; // whole groups, tails and both
        for (const double alpha : {0.5, 1e-3, 1e-6}) {
            for (const bool residual : {true, false}) {
                SinglePoleAverage<T> average;
                expect(fatal(average.setAlpha(alpha)));
                const std::vector<T> y = drive(average, std::span<const T>(x), std::span<const std::size_t>(calls), residual);

                // The recursion in double, one sample at a time, rounded to the output type once.
                double      real    = 0.0;
                double      imag    = 0.0;
                std::size_t outside = 0UZ;
                for (std::size_t i = 0UZ; i < kSamples; ++i) {
                    const double xr = static_cast<double>(std::real(x[i]));
                    const double xi = static_cast<double>(std::imag(x[i]));
                    real            = alpha * xr + (1.0 - alpha) * real;
                    imag            = alpha * xi + (1.0 - alpha) * imag;
                    T want;
                    if constexpr (std::floating_point<T>) {
                        want = static_cast<T>(residual ? xr - real : real);
                    } else {
                        want = residual ? T(static_cast<R>(xr - real), static_cast<R>(xi - imag)) : T(static_cast<R>(real), static_cast<R>(imag));
                    }
                    outside += withinOneRounding(y[i], want, std::abs(xr) + std::abs(xi) + std::abs(real) + std::abs(imag)) ? 0UZ : 1UZ;
                }
                expect(eq(outside, 0UZ)) << std::format("alpha {} {}: every output lies within one rounding of the recursion's", alpha, residual ? "subtract" : "average");
                // Each group's rounding is a few double steps of the signal, and the pole contracts it from one group
                // to the next. The two estimates agree far below a float step.
                const double drift = std::abs(std::complex<double>(average.estimate()) - std::complex<double>(real, imag));
                expect(lt(drift, 1e-12)) << std::format("alpha {}: the estimate stays within {} of the recursion's", alpha, drift);
            }
        }
    } | std::tuple<float, std::complex<float>, double, std::complex<double>>{};

    "a NaN or infinite sample holds the estimate, inside a group of eight and on its own"_test = []<typename T>() {
        using R                      = RealOf<T>;
        constexpr std::size_t kAt    = 1000UZ; // a multiple of eight: both averages solve the same groups
        const std::vector<T>  x      = noise<T>(kAt + 8UZ, 0xbadU);
        const auto            sameAs = [](R got, R want) { return (std::isnan(got) && std::isnan(want)) || got == want; };
        for (const R bad : {std::numeric_limits<R>::quiet_NaN(), std::numeric_limits<R>::infinity(), -std::numeric_limits<R>::infinity()}) {
            T badSample;
            if constexpr (std::floating_point<T>) {
                badSample = bad;
            } else {
                badSample = T(R(0.1), bad);
            }
            // One call holds the bad sample at kAt + 3, inside the last group of eight. The reference skips that sample
            // and takes the group one sample at a time, as the kernel does once the group's sum is not finite.
            SinglePoleAverage<T> inGroup;
            SinglePoleAverage<T> skipped;
            expect(fatal(inGroup.setAlpha(0.01) && skipped.setAlpha(0.01)));
            std::vector<T> withBad = x;
            withBad[kAt + 3UZ]     = badSample;
            std::vector<T> got(withBad.size());
            inGroup.subtract(std::span<const T>(withBad), std::span<T>(got));

            std::vector<T> want(x.size());
            skipped.subtract(std::span<const T>(x).first(kAt), std::span<T>(want).first(kAt));
            auto held = skipped.estimate();
            bool same = true;
            for (std::size_t i = kAt; i < kAt + 8UZ; ++i) {
                if (i == kAt + 3UZ) {
                    held = skipped.estimate(); // the estimate the bad sample meets
                } else {
                    skipped.subtract(std::span<const T>(x).subspan(i, 1UZ), std::span<T>(want).subspan(i, 1UZ));
                    same = same && got[i] == want[i];
                }
            }
            expect(same && inGroup.estimate() == skipped.estimate()) << "every other sample of the group and the estimate after it are the stream's without the bad sample";
            const T                    badOut = got[kAt + 3UZ];
            const std::complex<double> h(held);
            const std::complex<double> b(static_cast<double>(std::real(badSample)), static_cast<double>(std::imag(badSample)));
            expect(sameAs(std::real(badOut), static_cast<R>(b.real() - h.real())) && sameAs(std::imag(badOut), static_cast<R>(b.imag() - h.imag()))) << "the bad sample itself leaves less the held estimate";

            // The same sample in a call of its own goes through the recursion one sample at a time.
            const auto before = inGroup.estimate();
            T          alone{};
            inGroup.subtract(std::span<const T>(&badSample, 1UZ), std::span<T>(&alone, 1UZ));
            expect(inGroup.estimate() == before) << "and on its own it leaves the estimate where it was";
        }
    } | std::tuple<float, std::complex<float>>{};

    "silence takes the estimate to an exact zero and never through the subnormals"_test = []<typename T>() {
        // An alpha of 1e-300 puts an estimate of 1e-300 behind a unit sample. At alpha 0.5 each zero then halves it,
        // and it passes the smallest normal double after some 26 zeros.
        for (const std::size_t call : {1UZ, 8UZ}) { // one sample at a time, and whole groups
            SinglePoleAverage<T> average;
            expect(fatal(average.setAlpha(1e-300)));
            const T one(1);
            T       out{};
            average.average(std::span<const T>(&one, 1UZ), std::span<T>(&out, 1UZ));
            expect(gt(std::abs(std::complex<double>(average.estimate())), 0.0)) << "the estimate starts above zero";
            expect(fatal(average.setAlpha(0.5)));

            const std::vector<T> zeros(call, T{});
            std::vector<T>       y(call);
            std::size_t          subnormal = 0UZ;
            for (std::size_t n = 0UZ; n < 200UZ; n += call) {
                average.subtract(std::span<const T>(zeros), std::span<T>(y));
                const std::complex<double> e(average.estimate());
                subnormal += std::fpclassify(e.real()) == FP_SUBNORMAL || std::fpclassify(e.imag()) == FP_SUBNORMAL ? 1UZ : 0UZ;
            }
            expect(eq(subnormal, 0UZ)) << std::format("calls of {}: no estimate between calls is subnormal", call);
            expect(std::complex<double>(average.estimate()) == std::complex<double>{}) << "and the estimate ends at exactly zero";
        }
    } | std::tuple<float, std::complex<float>>{};

    "alpha outside (0, 1] is refused and alpha 1 makes the average the input"_test = [] {
        using CF                = std::complex<float>;
        const std::vector<CF> x = noise<CF>(100UZ, 0xa1fU);
        SinglePoleAverage<CF> average;
        SinglePoleAverage<CF> twin;
        expect(fatal(average.setAlpha(0.25) && twin.setAlpha(0.25)));
        std::vector<CF> y(x.size());
        std::vector<CF> yTwin(x.size());
        average.subtract(std::span<const CF>(x), std::span<CF>(y));
        twin.subtract(std::span<const CF>(x), std::span<CF>(yTwin));
        for (const double refused : {0.0, -0.25, 1.5, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
            expect(!average.setAlpha(refused)) << std::format("alpha {} is refused", refused);
        }
        expect(eq(average.alpha(), 0.25)) << "and alpha keeps the value it had";
        average.subtract(std::span<const CF>(x), std::span<CF>(y));
        twin.subtract(std::span<const CF>(x), std::span<CF>(yTwin));
        expect(y == yTwin && average.estimate() == twin.estimate()) << "the average computes as one never asked";

        expect(fatal(average.setAlpha(1.0)));
        average.average(std::span<const CF>(x), std::span<CF>(y));
        expect(y == x) << "at alpha 1 the average is each sample";
        average.subtract(std::span<const CF>(x), std::span<CF>(y));
        expect(std::ranges::all_of(y, [](CF v) { return v == CF{}; })) << "and the residual is zero";

        average.reset();
        expect(average.estimate() == std::complex<double>{}) << "reset() sets the estimate to zero";
    };
};

int main() { /* not needed for UT */ }
