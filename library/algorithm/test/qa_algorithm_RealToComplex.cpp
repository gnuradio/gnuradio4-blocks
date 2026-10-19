#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <limits>
#include <numbers>
#include <random>
#include <span>
#include <utility>
#include <vector>

#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/algorithm/filter/RealToComplex.hpp>

namespace {

using CF = std::complex<float>;
using CD = std::complex<double>;
using gr::filter::RealToComplex;

[[nodiscard]] RealToComplex make(double sampleRate, double transitionHz) {
    std::expected<RealToComplex, gr::Error> conversion = RealToComplex::make(sampleRate, transitionHz);
    boost::ut::expect(boost::ut::fatal(conversion.has_value())) << std::format("a conversion for {} Hz and {} Hz", sampleRate, transitionHz);
    return std::move(*conversion);
}

[[nodiscard]] std::vector<float> tone(double frequency, double sampleRate, std::size_t n, double amplitude = 1.0) {
    std::vector<float> x(n);
    for (std::size_t i = 0UZ; i < n; ++i) {
        x[i] = static_cast<float>(amplitude * std::sin(2.0 * std::numbers::pi * frequency * static_cast<double>(i) / sampleRate));
    }
    return x;
}

[[nodiscard]] std::vector<float> noise(std::size_t n, std::uint32_t seed) {
    std::mt19937                          rng(seed);
    std::uniform_real_distribution<float> uniform(-1.f, 1.f);
    std::vector<float>                    x(n);
    for (float& v : x) {
        v = uniform(rng);
    }
    return x;
}

[[nodiscard]] std::vector<CF> convert(RealToComplex& conversion, std::span<const float> x) {
    std::vector<CF> y;
    std::ignore = conversion.process(x, y);
    return y;
}

/// The magnitude of the DFT of @p y at @p frequency, a fraction of the output rate, over the samples from @p first on.
[[nodiscard]] double levelAt(const std::vector<CF>& y, double frequency, std::size_t first) {
    CD acc{};
    for (std::size_t n = first; n < y.size(); ++n) {
        acc += CD(y[n]) * std::polar(1.0, -2.0 * std::numbers::pi * frequency * static_cast<double>(n));
    }
    return std::abs(acc) / static_cast<double>(y.size() - first);
}

/**
 * The conversion by its definition, in double: each sample times `exp(-j*pi*n/2)`, the full convolution with every
 * designed tap from a zeroed history, then the samples at even indices.
 */
[[nodiscard]] std::vector<CD> byDefinition(std::span<const float> taps, std::span<const float> x) {
    constexpr CD    kMix[4] = {{1.0, 0.0}, {0.0, -1.0}, {-1.0, 0.0}, {0.0, 1.0}};
    std::vector<CD> y;
    for (std::size_t n = 0UZ; n < x.size(); n += 2UZ) {
        CD acc{};
        for (std::size_t k = 0UZ; k < taps.size() && k <= n; ++k) {
            acc += static_cast<double>(taps[k]) * kMix[(n - k) % 4UZ] * static_cast<double>(x[n - k]);
        }
        y.push_back(acc);
    }
    return y;
}

[[nodiscard]] bool allFinite(std::span<const float> v) {
    return std::ranges::all_of(v, [](float t) { return std::isfinite(t); });
}

[[nodiscard]] std::size_t oddAtLeast(double v) {
    const auto n = static_cast<std::size_t>(std::ceil(v));
    return n % 2UZ == 0UZ ? n + 1UZ : n;
}

} // namespace

const boost::ut::suite<"RealToComplex"> realToComplexTests = [] {
    using namespace boost::ut;

    "a call boundary anywhere changes no output, at an even and an odd center tap"_test = [] {
        std::vector<std::size_t> centerParities;
        for (const auto& [rate, transition] : {std::pair{48'000.0, 200.0}, std::pair{500'000.0, 20'000.0}}) {
            const auto x = tone(0.1875 * rate, rate, 8192UZ);

            RealToComplex     whole = make(rate, transition);
            const auto        once  = convert(whole, x);
            const std::size_t taps  = whole.taps().size();
            centerParities.push_back(whole.groupDelay() % 2UZ);
            RealToComplex   split = make(rate, transition);
            std::vector<CF> pieces;
            std::ignore = split.process(std::span(x).first(2047UZ), pieces);
            std::ignore = split.process(std::span(x).subspan(2047UZ), pieces);
            expect(eq(once.size(), (x.size() + 1UZ) / 2UZ)) << "input 2n makes output n";
            expect(pieces == once) << std::format("{} taps: a split at an odd length gives the same outputs, bit for bit", taps);

            RealToComplex   ragged = make(rate, transition);
            std::vector<CF> parts;
            std::mt19937    rng(7U);
            for (std::size_t at = 0UZ; at < x.size();) {
                const std::size_t length = std::min<std::size_t>(std::uniform_int_distribution<std::size_t>(0UZ, 37UZ)(rng), x.size() - at);
                const std::size_t made   = ragged.outputsFor(length);
                expect(eq(ragged.process(std::span(x).subspan(at, length), parts), made)) << "outputsFor() counts what a call makes";
                at += length;
            }
            expect(parts == once) << std::format("{} taps: calls of 0 to 37 samples give the same outputs", taps);
        }
        expect(centerParities == std::vector<std::size_t>{0UZ, 1UZ}) << "797 taps put the center on an even index and 83 taps on an odd one";
    };

    "a tone at f leaves at f - fs/4 in a stream at fs/2"_test = [] {
        constexpr double kRate = 48'000.0;
        expect(eq(RealToComplex::outputRate(kRate), kRate / 2.0));
        expect(eq(RealToComplex::centerFrequency(kRate), kRate / 4.0));
        const double stopbandDb = gr::filter::fir::design::windowFigures(gr::algorithm::window::Type::Hamming).attenuationDb;
        for (const double f : {3'000.0, 9'000.0, 20'000.0}) {
            RealToComplex     conversion = make(kRate, 200.0);
            const auto        y          = convert(conversion, tone(f, kRate, 16'384UZ));
            const double      outRate    = RealToComplex::outputRate(kRate);
            const double      at         = (f - RealToComplex::centerFrequency(kRate)) / outRate;
            const double      image      = (-f - RealToComplex::centerFrequency(kRate)) / outRate;
            const std::size_t first      = conversion.taps().size();
            const double      mirrorDb   = 20.0 * std::log10(levelAt(y, image, first) / levelAt(y, at, first));
            expect(lt(mirrorDb, -(stopbandDb - 3.0))) << std::format("{} Hz leaves at {} Hz, and its mirror {:.1f} dB below it", f, at * outRate, mirrorDb);
        }
    };

    "a real tone of amplitude A leaves at magnitude A/2"_test = [] {
        constexpr double kRate = 48'000.0;
        for (const double amplitude : {1.0, 0.25}) {
            for (const double f : {3'000.0, 9'000.0, 20'000.0}) {
                RealToComplex     conversion = make(kRate, 200.0);
                const auto        y          = convert(conversion, tone(f, kRate, 8192UZ, amplitude));
                const std::size_t first      = conversion.taps().size();
                double            sum        = 0.0;
                for (std::size_t n = first; n < y.size(); ++n) {
                    sum += std::abs(CD(y[n]));
                }
                const double mean = sum / static_cast<double>(y.size() - first);
                expect(lt(std::abs(mean / amplitude - 0.5), 0.005)) << std::format("amplitude {} at {} Hz converts to magnitude {:.5f}", amplitude, f, mean);
            }
        }
    };

    "the band covered starts at half the transition"_test = [] {
        constexpr double kRate       = 48'000.0;
        constexpr double kTransition = 1'000.0;
        expect(eq(RealToComplex::lowEdge(kTransition), kTransition / 2.0));

        // At the low edge the tone passes whole. Its mirror lands in the stopband at the far side of the transition.
        const double      f          = RealToComplex::lowEdge(kTransition);
        RealToComplex     conversion = make(kRate, kTransition);
        const auto        y          = convert(conversion, tone(f, kRate, 32'768UZ));
        const double      outRate    = RealToComplex::outputRate(kRate);
        const std::size_t first      = conversion.taps().size();
        const double      wanted     = levelAt(y, (f - RealToComplex::centerFrequency(kRate)) / outRate, first);
        const double      mirror     = levelAt(y, (-f - RealToComplex::centerFrequency(kRate)) / outRate, first);
        const double      stopbandDb = gr::filter::fir::design::windowFigures(gr::algorithm::window::Type::Hamming).attenuationDb;
        expect(lt(std::abs(wanted - 0.5), 0.005)) << std::format("the edge tone leaves at {:.5f}", wanted);
        expect(lt(20.0 * std::log10(mirror / wanted), -(stopbandDb - 3.0))) << std::format("its mirror is {:.1f} dB down", 20.0 * std::log10(mirror / wanted));

        // A tone at half the low edge puts its mirror inside the transition, where far more of it leaks through.
        const double  inside = 0.5 * f;
        RealToComplex below  = make(kRate, kTransition);
        const auto    z      = convert(below, tone(inside, kRate, 32'768UZ));
        const double  kept   = levelAt(z, (inside - RealToComplex::centerFrequency(kRate)) / outRate, first);
        const double  leaked = levelAt(z, (-inside - RealToComplex::centerFrequency(kRate)) / outRate, first);
        expect(gt(20.0 * std::log10(leaked / kept), -(stopbandDb - 3.0) + 20.0)) << std::format("a tone at a quarter of the transition leaks its mirror at {:.1f} dB", 20.0 * std::log10(leaked / kept));
    };

    "an impulse leaves after the group delay"_test = [] {
        for (const double transition : {200.0, 1'000.0, 20'000.0}) {
            RealToComplex     conversion = make(48'000.0, transition);
            const std::size_t delay      = conversion.groupDelay();
            expect(eq(delay, (conversion.taps().size() - 1UZ) / 2UZ));
            const std::size_t  at = 100UZ + delay % 2UZ; // the impulse and the delay sum to an even input
            std::vector<float> x(at + 2UZ * conversion.taps().size(), 0.f);
            x[at]            = 1.f;
            const auto  y    = convert(conversion, x);
            std::size_t peak = 0UZ;
            for (std::size_t n = 0UZ; n < y.size(); ++n) {
                if (std::abs(y[n]) > std::abs(y[peak])) {
                    peak = n;
                }
            }
            expect(eq(peak, (at + delay) / 2UZ)) << std::format("{} Hz: an impulse at input {} peaks at output {}", transition, at, peak);
        }
    };

    "the conversion equals its definition within float rounding"_test = [] {
        for (const auto& [rate, transition] : {std::pair{500'000.0, 20'000.0}, std::pair{48'000.0, 200.0}, std::pair{48'000.0, 7'000.0}}) {
            RealToComplex conversion = make(rate, transition);
            const auto    x          = noise(20'001UZ, 11U);
            const auto    y          = convert(conversion, x);
            const auto    reference  = byDefinition(conversion.taps(), x);
            expect(fatal(eq(y.size(), reference.size())));

            // The kernel sums about N/2 products in float. Each sum is within N/2 rounding steps of the magnitudes it
            // adds, and every input is at most 1. The skipped taps add their own size.
            const auto taps    = conversion.taps();
            const auto center  = (taps.size() - 1UZ) / 2UZ;
            double     sumAbs  = 0.0;
            double     skipped = 0.0;
            for (std::size_t k = 0UZ; k < taps.size(); ++k) {
                sumAbs += std::abs(static_cast<double>(taps[k]));
                if (k != center && (k + center) % 2UZ == 0UZ) {
                    skipped += std::abs(static_cast<double>(taps[k]));
                }
            }
            const double bound = static_cast<double>(taps.size() / 2UZ + 2UZ) * static_cast<double>(std::numeric_limits<float>::epsilon()) * sumAbs + skipped;
            double       worst = 0.0;
            for (std::size_t n = 0UZ; n < y.size(); ++n) {
                worst = std::max(worst, std::abs(CD(y[n]) - reference[n]));
            }
            expect(le(worst, bound)) << std::format("{} taps: the largest difference {:.3g} against a bound of {:.3g}", taps.size(), worst, bound);
            expect(gt(worst, 0.0)) << "the two sums differ in rounding, so the comparison reads two computations";
        }
    };

    "the design follows the Hamming transition constant up to the cap"_test = [] {
        const double constant = gr::filter::fir::design::windowFigures(gr::algorithm::window::Type::Hamming).transitionConstant;
        for (const auto& [rate, transition] : {std::pair{48'000.0, 200.0}, std::pair{500'000.0, 20'000.0}, std::pair{2.0e6, 3'000.0}}) {
            const std::size_t n = RealToComplex::tapCount(rate, transition).value();
            expect(eq(n, oddAtLeast(constant * rate / transition))) << std::format("{} Hz at {} Hz", transition, rate);
            expect(eq(RealToComplex::design(rate, transition).value().size(), n));
        }

        const auto narrow = RealToComplex::design(2.0e6, 1.0).value();
        expect(eq(narrow.size(), RealToComplex::kMaxTaps)) << "1 Hz at 2 MS/s is held at the cap";
        expect(eq(narrow.size() % 2UZ, 1UZ)) << "and the cap is odd";
        const auto absurd = RealToComplex::design(1.0e6, 1e-3).value();
        expect(eq(absurd.size(), RealToComplex::kMaxTaps)) << "a length past every integer is held at the same cap";
        expect(allFinite(absurd));
        expect(eq(RealToComplex::tapCount(1e300, 1e-300).value(), RealToComplex::kMaxTaps)) << "a width that rounds to zero is held at the cap";

        const auto wide = RealToComplex::design(48'000.0, 1.0e6).value();
        expect(eq(wide.size(), 3UZ)) << "a transition wider than the band designs the shortest odd filter";
        expect(allFinite(wide));
        RealToComplex shortest = make(48'000.0, 1.0e6);
        expect(eq(convert(shortest, std::vector<float>(64UZ, 1.f)).size(), 32UZ)) << "and that filter converts";
    };

    "a rate or a transition that is not positive and finite is an error"_test = [] {
        constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
        constexpr double kInf = std::numeric_limits<double>::infinity();
        for (const double bad : {0.0, -1.0, kNaN, kInf, -kInf}) {
            expect(!RealToComplex::make(bad, 200.0).has_value()) << std::format("a rate of {}", bad);
            expect(!RealToComplex::make(48'000.0, bad).has_value()) << std::format("a transition of {}", bad);
            expect(!RealToComplex::tapCount(bad, 200.0).has_value());
        }
        const auto refused = RealToComplex::make(0.0, 200.0);
        expect(!refused.has_value() && refused.error().message.contains("positive and finite")) << "the error says why";
    };

    "reset() returns the conversion to the start of a stream"_test = [] {
        const auto    x     = noise(4001UZ, 3U);
        RealToComplex fresh = make(48'000.0, 2'000.0);
        const auto    once  = convert(fresh, x);

        RealToComplex used = make(48'000.0, 2'000.0);
        std::ignore        = convert(used, noise(1235UZ, 5U)); // an odd count leaves the next input odd
        used.reset();
        expect(eq(used.outputsFor(1UZ), 1UZ)) << "the next input is even again";
        expect(convert(used, x) == once) << "the same outputs as a fresh conversion, bit for bit";
    };

    "a NaN or an infinite sample spoils only the outputs that read it"_test = [] {
        RealToComplex     clean  = make(48'000.0, 2'000.0);
        RealToComplex     broken = make(48'000.0, 2'000.0);
        const std::size_t n      = clean.taps().size();
        auto              x      = noise(10UZ * n, 9U);
        const std::size_t nanAt  = 2UZ * n + 1UZ;
        const std::size_t infAt  = 6UZ * n;
        x[nanAt]                 = 0.f;
        x[infAt]                 = 0.f;
        const auto good          = convert(clean, x);
        x[nanAt]                 = std::numeric_limits<float>::quiet_NaN();
        x[infAt]                 = std::numeric_limits<float>::infinity();
        const auto bad           = convert(broken, x);
        expect(fatal(eq(bad.size(), good.size()))) << "the stream keeps its length";

        // Output m reads inputs 2m - N + 1 to 2m.
        const auto reads         = [n](std::size_t m, std::size_t at) { return 2UZ * m >= at && 2UZ * m < at + n; };
        bool       outsideEqual  = true;
        bool       insideSpoiled = false;
        for (std::size_t m = 0UZ; m < bad.size(); ++m) {
            const bool finite = std::isfinite(bad[m].real()) && std::isfinite(bad[m].imag());
            if (reads(m, nanAt) || reads(m, infAt)) {
                insideSpoiled = insideSpoiled || !finite;
            } else {
                outsideEqual = outsideEqual && bad[m] == good[m];
            }
        }
        expect(insideSpoiled) << "the outputs that read the samples are not finite";
        expect(outsideEqual) << "every other output equals the one with zeros in their place, bit for bit";
        expect(bad.back() == good.back()) << "and the stream is clean again at its end";
    };
};

int main() { /* not needed for UT */ }
