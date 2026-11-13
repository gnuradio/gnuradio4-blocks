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

    "an estimate near the top of the double range stays finite where the estimate plus the weighted sum passes it"_test = []<typename T>() {
        using R               = RealOf<T>;
        constexpr double kTop = 0.999 * std::numeric_limits<double>::max();
        const auto       make = [](double re, double im) {
            if constexpr (std::floating_point<T>) {
                return static_cast<T>(re);
            } else {
                return T(static_cast<R>(re), static_cast<R>(im));
            }
        };
        const auto sample  = [&](double v) { return make(v, -0.5 * v); };
        const auto closeTo = [](double got, double want, double scale) { return std::isfinite(want) ? std::abs(got - want) <= 1e-12 * scale : got == want; };
        for (const bool residual : {true, false}) {
            for (const int shape : {0, 1}) { // 0: the same sample again and a group of zeros, 1: a group alternating in sign
                SinglePoleAverage<T> average;
                expect(fatal(average.setAlpha(1.0)));
                double         real = 0.0;
                double         imag = 0.0;
                std::vector<T> got;
                std::vector<T> want;
                // Runs @p x through the kernel and through the recursion one sample at a time.
                const auto run = [&](const std::vector<T>& x, double alpha) {
                    const std::size_t    calls[] = {8UZ};
                    const std::vector<T> y       = drive(average, std::span<const T>(x), std::span<const std::size_t>(calls), residual);
                    got.insert(got.end(), y.begin(), y.end());
                    for (const T& v : x) {
                        const double xr = static_cast<double>(std::real(v));
                        const double xi = static_cast<double>(std::imag(v));
                        real            = alpha * xr + (1.0 - alpha) * real;
                        imag            = alpha * xi + (1.0 - alpha) * imag;
                        want.push_back(residual ? make(xr - real, xi - imag) : make(real, imag));
                    }
                };
                const std::size_t outputsBefore = got.size();
                run(std::vector<T>(1UZ, sample(kTop)), 1.0);
                expect(fatal(average.setAlpha(0.001)));
                if (shape == 0) {
                    run(std::vector<T>(8UZ, sample(kTop)), 0.001);
                } else {
                    std::vector<T> alternating;
                    for (std::size_t i = 0UZ; i < 8UZ; ++i) {
                        alternating.push_back(sample(i % 2UZ == 0UZ ? kTop : -kTop));
                    }
                    run(alternating, 0.001);
                }
                bool allClose  = true;
                bool allFinite = true;
                for (std::size_t i = outputsBefore; i < got.size(); ++i) {
                    const double scale = kTop;
                    allClose           = allClose && closeTo(static_cast<double>(std::real(got[i])), static_cast<double>(std::real(want[i])), scale) && closeTo(static_cast<double>(std::imag(got[i])), static_cast<double>(std::imag(want[i])), scale);
                    allFinite          = allFinite && std::isfinite(std::real(got[i])) && std::isfinite(std::imag(got[i]));
                }
                const auto estimate = std::complex<double>(average.estimate());
                expect(allClose) << std::format("{} shape {}: every output is the recursion's", residual ? "subtract" : "average", shape);
                expect(allFinite || (residual && shape == 1)) << "and finite, a residual of opposite signs apart";
                expect(std::isfinite(estimate.real()) && std::isfinite(estimate.imag())) << "the estimate is finite";
                expect(closeTo(estimate.real(), real, kTop) && closeTo(estimate.imag(), imag, kTop)) << "and the recursion's";

                if (shape == 0) { // the estimate decays as the recursion's does
                    run(std::vector<T>(8UZ, T{}), 0.001);
                    const auto after = std::complex<double>(average.estimate());
                    expect(closeTo(after.real(), real, std::abs(real)) && closeTo(after.imag(), imag, std::abs(imag)) && std::abs(after.real()) < std::abs(estimate.real())) << "a group of zeros takes the estimate down as the recursion does";
                }
            }
        }
    } | std::tuple<double, std::complex<double>>{};

    "an aliased output, either component near the top and the exchanged stream give the recursion's results"_test = []<typename T>() {
        using R               = RealOf<T>;
        constexpr double kTop = 0.999 * std::numeric_limits<double>::max();
        enum class Call { subtract, average, exchanged };
        const auto make = [](double re, double im) {
            if constexpr (std::floating_point<T>) {
                return static_cast<T>(re);
            } else {
                return T(static_cast<R>(re), static_cast<R>(im));
            }
        };
        const auto closeTo = [](double got, double want) { return std::isfinite(want) ? std::abs(got - want) <= 1e-12 * kTop : got == want; };
        for (const Call call : {Call::subtract, Call::average, Call::exchanged}) {
            if (call == Call::exchanged && std::floating_point<T>) {
                continue; // exchanging components needs a complex stream
            }
            for (const bool aliased : {false, true}) {
                for (const bool imagTop : {false, true}) { // the component that overflows
                    if (imagTop && std::floating_point<T>) {
                        continue;
                    }
                    const auto           sample = [&](double v) { return imagTop ? make(-0.5 * v, v) : make(v, -0.5 * v); };
                    SinglePoleAverage<T> average;
                    double               real = 0.0;
                    double               imag = 0.0;
                    bool                 same = true;
                    // Runs one group of samples through the kernel, in place when @p aliased, and through the recursion.
                    const auto run = [&](const std::vector<T>& x, double alpha) {
                        expect(fatal(average.setAlpha(alpha)));
                        std::vector<T> buffer = x;
                        std::vector<T> out(aliased ? 0UZ : x.size());
                        std::span<T>   to   = aliased ? std::span<T>(buffer) : std::span<T>(out);
                        const auto     from = std::span<const T>(buffer);
                        if (call == Call::exchanged) {
                            if constexpr (!std::floating_point<T>) {
                                average.subtractExchanged(from, to);
                            }
                        } else if (call == Call::subtract) {
                            average.subtract(from, to);
                        } else {
                            average.average(from, to);
                        }
                        for (std::size_t i = 0UZ; i < x.size(); ++i) {
                            double xr = static_cast<double>(std::real(x[i]));
                            double xi = static_cast<double>(std::imag(x[i]));
                            if (call == Call::exchanged) {
                                std::swap(xr, xi);
                            }
                            real         = alpha * xr + (1.0 - alpha) * real;
                            imag         = alpha * xi + (1.0 - alpha) * imag;
                            const T want = call == Call::average ? make(real, imag) : make(xr - real, xi - imag);
                            same         = same && closeTo(static_cast<double>(std::real(to[i])), static_cast<double>(std::real(want))) && closeTo(static_cast<double>(std::imag(to[i])), static_cast<double>(std::imag(want)));
                        }
                    };
                    run(std::vector<T>(1UZ, sample(kTop)), 1.0);
                    run(std::vector<T>(8UZ, sample(kTop)), 0.001);
                    std::vector<T> alternating;
                    for (std::size_t i = 0UZ; i < 8UZ; ++i) {
                        alternating.push_back(sample(i % 2UZ == 0UZ ? kTop : -kTop));
                    }
                    run(alternating, 0.001);
                    const auto estimate = std::complex<double>(average.estimate());
                    expect(same) << std::format("call {} aliased {} imaginary {}: every output is the recursion's", static_cast<int>(call), aliased, imagTop);
                    expect(closeTo(estimate.real(), real) && closeTo(estimate.imag(), imag)) << "and the estimate";
                }
            }
        }
    } | std::tuple<double, std::complex<double>>{};

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

    "subtracting the exchanged stream reads the samples where they lie and exchanges the estimate with them"_test = [] {
        using C                      = std::complex<float>;
        constexpr std::size_t kCount = 20003UZ; // whole groups and a tail
        const std::vector<C>  x      = noise<C>(kCount, 0xe4c4U);
        std::vector<C>        exchanged(kCount);
        std::ranges::transform(x, exchanged.begin(), [](C v) { return C(v.imag(), v.real()); });
        for (const double alpha : {0.5, 1e-3, 1e-6}) {
            SinglePoleAverage<C> folded;
            SinglePoleAverage<C> apart;
            expect(fatal(folded.setAlpha(alpha) && apart.setAlpha(alpha)));
            std::vector<C> got(kCount);
            std::vector<C> want(kCount);
            folded.subtractExchanged(std::span<const C>(x), std::span<C>(got));
            apart.subtract(std::span<const C>(exchanged), std::span<C>(want));
            expect(got == want) << std::format("alpha {}: the output equals subtract() on the exchanged samples, bit for bit", alpha);
            expect(folded.estimate() == apart.estimate()) << "and so does the estimate";

            const auto before = folded.estimate();
            folded.exchange();
            expect(folded.estimate() == std::complex<double>(before.imag(), before.real())) << "exchange() swaps the two components of the estimate";
        }
    };
};

int main() { /* not needed for UT */ }
