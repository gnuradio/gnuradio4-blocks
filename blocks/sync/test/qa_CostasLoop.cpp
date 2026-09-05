#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <limits>
#include <numbers>
#include <random>
#include <span>
#include <vector>

#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/sync/CostasLoop.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

// The S-curves, their slopes and the clipping onsets are evaluated in double against the same error law the block
// runs, which is a template over the value type for exactly this reason: the closed forms are properties of the law
// and not of the width it is carried in.

namespace {

using gr::blocks::sync::costasError;
using gr::blocks::sync::CostasLoop;
using gr::blocks::sync::kOrder8CrossWeight;
using gr::blocks::sync::softSlice;
namespace test = gr::blocks::testing::span;

using CF = std::complex<float>;
using CD = std::complex<double>;

constexpr double kPi        = std::numbers::pi;
constexpr double kBandwidth = 0.01;

template<typename TBlock>
void configure(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
}

CostasLoop makeCostas(gr::property_map settings = {}) {
    CostasLoop block(std::move(settings));
    configure(block);
    return block;
}

auto driveCostas(CostasLoop& block, std::span<const CF> input, std::size_t chunk = 0UZ, std::array<bool, 3UZ> wired = {true, true, true}, std::span<const gr::Tag> tags = {}) { return test::runPorts<3UZ, CF>(block, input, chunk, wired, tags); }

/// @brief The `k`-th symbol phase of an `order`-PSK constellation, rotated so no 8PSK symbol lands on an axis.
[[nodiscard]] double symbolPhase(std::size_t order, std::size_t k) {
    const double spacing = 2.0 * kPi / static_cast<double>(order);
    return order == 2UZ ? static_cast<double>(k) * kPi : 0.5 * spacing + static_cast<double>(k) * spacing;
}

/// @brief The error law dispatched on a run-time order, in double, for the closed-form and slope checks.
[[nodiscard]] double lawAt(std::size_t order, CD y) {
    switch (order) {
    case 2UZ: return costasError<2UZ>(y);
    case 4UZ: return costasError<4UZ>(y);
    default: return costasError<8UZ>(y);
    }
}

/// @brief `S(phi)`: the error law averaged over every symbol of the constellation.
[[nodiscard]] double sCurve(std::size_t order, double amplitude, double phi) {
    double sum = 0.0;
    for (std::size_t k = 0UZ; k < order; ++k) {
        sum += lawAt(order, std::polar(amplitude, symbolPhase(order, k) + phi));
    }
    return sum / static_cast<double>(order);
}

/// @brief `Kdet`: the S-curve's slope through zero, by a central difference fine enough to be exact in double.
[[nodiscard]] double detectorGain(std::size_t order, double amplitude) {
    constexpr double kStep = 1e-6;
    return (sCurve(order, amplitude, kStep) - sCurve(order, amplitude, -kStep)) / (2.0 * kStep);
}

[[nodiscard]] double sCurvePeak(std::size_t order, double amplitude) {
    double peak = 0.0;
    for (int i = -20000; i <= 20000; ++i) {
        peak = std::max(peak, std::abs(sCurve(order, amplitude, kPi * static_cast<double>(i) / 20000.0)));
    }
    return peak;
}

/// @brief The residual phase after the modulation is stripped by raising the sample to the constellation's order.
[[nodiscard]] double residualPhase(CF sample, std::size_t order) {
    CD stripped{1.0, 0.0};
    for (std::size_t k = 0UZ; k < order; ++k) {
        stripped *= CD(sample);
    }
    const double reference = order == 2UZ ? 0.0 : kPi;
    return std::remainder(std::arg(stripped) - reference, 2.0 * kPi) / static_cast<double>(order);
}

[[nodiscard]] std::vector<CF> pskStream(std::size_t nSamples, std::size_t order, double amplitude, double cyclesPerSample, double initialPhase, std::uint32_t seed) {
    std::mt19937                       rng(seed);
    std::uniform_int_distribution<int> pick(0, static_cast<int>(order) - 1);
    std::vector<CF>                    signal(nSamples);
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        const double symbol  = symbolPhase(order, static_cast<std::size_t>(pick(rng)));
        const double carrier = initialPhase + 2.0 * kPi * cyclesPerSample * static_cast<double>(i);
        signal[i]            = static_cast<CF>(std::polar(amplitude, symbol + carrier));
    }
    return signal;
}

} // namespace

const boost::ut::suite<"CostasLoop"> costasTests = [] {
    using namespace boost::ut;

    "the order-2 S-curve is exactly half a squared amplitude times sin(2 phi)"_test = [] {
        for (const double amplitude : {0.5, 1.0, 2.0}) {
            for (int i = -400; i <= 400; ++i) {
                const double phi = kPi * static_cast<double>(i) / 400.0;
                expect(lt(std::abs(sCurve(2UZ, amplitude, phi) - 0.5 * amplitude * amplitude * std::sin(2.0 * phi)), 1e-6)) << "order 2 at A=" << amplitude << " phi=" << phi;
            }
            expect(lt(std::abs(detectorGain(2UZ, amplitude) - amplitude * amplitude), 1e-6)) << "Kdet = A^2 at A=" << amplitude;
        }
    };

    "the order-4 S-curve is exactly A sqrt(2) sin(phi) inside its own quarter"_test = [] {
        for (const double amplitude : {0.5, 1.0, 2.0}) {
            for (int i = -399; i <= 399; ++i) {
                const double phi = 0.25 * kPi * static_cast<double>(i) / 400.0;
                expect(lt(std::abs(sCurve(4UZ, amplitude, phi) - amplitude * std::numbers::sqrt2 * std::sin(phi)), 1e-6)) << "order 4 at A=" << amplitude << " phi=" << phi;
            }
            expect(lt(std::abs(detectorGain(4UZ, amplitude) - amplitude * std::numbers::sqrt2), 1e-6)) << "Kdet = A sqrt(2) at A=" << amplitude;
        }
    };

    "the order-8 law has a zero on every symbol and a gain of 1.08239220 per unit amplitude"_test = [] {
        expect(lt(std::abs(sCurve(8UZ, 1.0, 0.0)), 1e-9)) << "S(0) must be zero";
        for (std::size_t k = 0UZ; k < 8UZ; ++k) {
            expect(lt(std::abs(lawAt(8UZ, std::polar(1.0, symbolPhase(8UZ, k)))), 1e-9)) << "every symbol must be a zero, k=" << k;
        }
        for (const auto& [amplitude, gain] : std::array<std::pair<double, double>, 3UZ>{{{0.5, 0.541196}, {1.0, 1.082392}, {2.0, 2.164784}}}) {
            expect(lt(std::abs(detectorGain(8UZ, amplitude) - gain), 1e-6)) << "Kdet at A=" << amplitude;
        }
    };

    "the order-8 cross weight is tan(pi/8), which is why the zeros fall where they do"_test = [] { expect(lt(std::abs(kOrder8CrossWeight - std::tan(kPi / 8.0)), 1e-15)) << "sqrt(2) - 1 == tan(pi/8)"; };

    "an 8PSK constellation on the axes breaks the detector, which is why the rotation is a requirement"_test = [] {
        constexpr double kEdge = 1e-9;

        double above = 0.0;
        double below = 0.0;
        for (std::size_t k = 0UZ; k < 8UZ; ++k) {
            const double onAxis = static_cast<double>(k) * kPi / 4.0; // the natural rotation, symbols on the axes
            above += costasError<8UZ>(std::polar(1.0, onAxis + kEdge));
            below += costasError<8UZ>(std::polar(1.0, onAxis - kEdge));
        }
        above /= 8.0;
        below /= 8.0;

        expect(lt(above * below, 0.0)) << "the two sides of zero must have opposite signs";
        expect(lt(std::abs(std::abs(above) - 0.414214), 1e-4)) << "S(0+) magnitude, got " << above;
        expect(lt(std::abs(std::abs(below) - 0.414214), 1e-4)) << "S(0-) magnitude, got " << below;
    };

    "the gain goes as the squared amplitude at order 2 and as the amplitude at orders 4 and 8"_test = [] {
        constexpr std::array<double, 4UZ> kAmplitudes{0.5, 1.0, 2.0, 4.0};
        for (const std::size_t order : {2UZ, 4UZ, 8UZ}) {
            const double expected = order == 2UZ ? 4.0 : 2.0;
            for (std::size_t i = 1UZ; i < kAmplitudes.size(); ++i) {
                const double ratio = detectorGain(order, kAmplitudes[i]) / detectorGain(order, kAmplitudes[i - 1UZ]);
                expect(lt(std::abs(ratio - expected), 1e-4)) << "order " << order << " gain ratio " << ratio;
            }
        }
    };

    "additive noise moves the order-2 slope by less than six percent from zero to twenty decibels"_test = [] {
        // The noise-noise cross term is zero mean and the detector is bilinear, so the slope survives and only the
        // variance about it grows. Two hundred thousand paired symbols per point keeps the run short.
        constexpr std::size_t kSymbols = 200000UZ;
        constexpr double      kStep    = 0.05;

        for (const double esn0db : {0.0, 3.0, 6.0, 10.0, 20.0}) {
            const double                     sigma = std::sqrt(std::pow(10.0, -esn0db / 10.0) / 2.0);
            std::mt19937                     rng(1234U);
            std::normal_distribution<double> noise(0.0, sigma);
            std::bernoulli_distribution      bit(0.5);

            double above = 0.0;
            double below = 0.0;
            for (std::size_t i = 0UZ; i < kSymbols; ++i) {
                const CD sample = CD(bit(rng) ? 1.0 : -1.0, 0.0) + CD(noise(rng), noise(rng));
                above += costasError<2UZ>(sample * std::polar(1.0, kStep));
                below += costasError<2UZ>(sample * std::polar(1.0, -kStep));
            }
            const double slope = (above - below) / (2.0 * kStep * static_cast<double>(kSymbols)) / (std::sin(2.0 * kStep) / (2.0 * kStep));
            expect(lt(std::abs(slope - 1.0), 0.06)) << "measured slope " << slope << " at Es/N0 " << esn0db << " dB";
        }
    };

    "the default error limit engages at 1.41421, 1.00000 and 2.41421"_test = [] {
        // Orders 2 and 4 peak at A^2/2 and at A, so the amplitude at which the peak reaches the default limit of 1 is
        // sqrt(2) and 1. Order 8 peaks at the sector edge, where all eight symbols sit on the branch boundary and the
        // law gives A*sin(pi/4)*(1 - K) = A*tan(pi/8) exactly, so the onset is 1/tan(pi/8) = 1 + sqrt(2) = 2.414214.
        // A peak search on a grid that misses pi/8 returns 0.41107 instead, which inverts to 2.4327; the exact peak,
        // 0.414214, inverts to the 2.414214 asserted here.
        expect(lt(std::abs(std::sqrt(1.0 / sCurvePeak(2UZ, 1.0)) - std::numbers::sqrt2), 1e-5)) << "order 2";
        expect(lt(std::abs(1.0 / sCurvePeak(4UZ, 1.0) - 1.0), 1e-5)) << "order 4";
        expect(lt(std::abs(1.0 / sCurvePeak(8UZ, 1.0) - (1.0 + std::numbers::sqrt2)), 1e-5)) << "order 8";
        expect(lt(std::abs(sCurvePeak(8UZ, 1.0) - std::tan(kPi / 8.0)), 1e-9)) << "and the order-8 peak is tan(pi/8) exactly";
    };

    "the soft slicer is within 2.352e-2 of tanh and is defined everywhere"_test = [] {
        double worst = 0.0;
        for (int i = -40000; i <= 40000; ++i) {
            const double x = static_cast<double>(i) / 10000.0;
            worst          = std::max(worst, std::abs(softSlice(x) - std::tanh(x)));
        }
        expect(lt(std::abs(worst - 2.352e-2), 1e-4)) << "maximum absolute error over [-4, 4], got " << worst;
        expect(eq(softSlice(1e30f), softSlice(4.f))) << "and it saturates rather than running away";
        expect(eq(softSlice(2.0f), softSlice(2.0f))) << "including at exactly 2, where a 256-entry table reads past its end";
    };

    "each order acquires a carrier offset and holds it inside a quarter of its own lock spacing"_test = [] {
        constexpr std::size_t kSamples = 8000UZ;
        for (const auto& [order, gain] : std::array<std::pair<std::size_t, double>, 3UZ>{{{2UZ, 1.0}, {4UZ, std::numbers::sqrt2}, {8UZ, 1.08239220}}}) {
            const std::vector<CF> input = pskStream(kSamples, order, 1.0, 1e-3, 0.9, 5U);

            CostasLoop block   = makeCostas({{"noise_bandwidth", kBandwidth}, {"order", static_cast<gr::Size_t>(order)}, {"detector_gain", gain}, {"error_limit", 2.0}});
            const auto tracked = driveCostas(block, std::span<const CF>(input));

            const double bound = 0.25 * (2.0 * kPi / static_cast<double>(order));
            double       worst = 0.0;
            for (std::size_t i = 2000UZ; i < kSamples; ++i) {
                worst = std::max(worst, std::abs(residualPhase(tracked.samples[i], order)));
            }
            expect(lt(worst, bound)) << "order " << order << " residual " << worst << " against " << bound;
            expect(approx(tracked.aux[0][kSamples - 1UZ], static_cast<float>(2.0 * kPi * 1e-3), 5e-4f)) << "order " << order << " settled frequency";
        }
    };

    "degenerate settings are refused rather than silently reinterpreted"_test = [] {
        expect(throws([] { std::ignore = makeCostas({{"order", gr::Size_t(3)}}); })) << "an order with no law";
        expect(throws([] { std::ignore = makeCostas({{"detector_gain", 0.0}}); })) << "a zero detector gain";
        expect(throws([] { std::ignore = makeCostas({{"error_limit", -1.0}}); })) << "a negative error limit";
        expect(throws([] { std::ignore = makeCostas({{"noise_power", 0.0}}); })) << "a zero noise power";
        expect(throws([] { std::ignore = makeCostas({{"min_frequency", 1.f}, {"max_frequency", -1.f}}); })) << "crossed bounds";
    };

    "tags ride through, the two loop tags do not, and a wild payload cannot hang the block"_test = [] {
        constexpr std::size_t      kSamples = 200UZ;
        const std::vector<CF>      input    = pskStream(kSamples, 4UZ, 1.0, 1e-3, 0.0, 9U);
        const std::vector<gr::Tag> passenger{gr::Tag{40UZ, gr::property_map{{"passenger", 1.0}}}};

        CostasLoop block   = makeCostas();
        const auto carried = driveCostas(block, std::span<const CF>(input), 0UZ, {true, true, true}, std::span<const gr::Tag>(passenger));
        expect(eq(carried.offsetsOf("passenger").size(), 1UZ));
        expect(eq(carried.offsetsOf("passenger")[0], 40UZ));

        const std::vector<gr::Tag> steering{gr::Tag{10UZ, gr::property_map{{"phase_est", 0.25}, {"freq_est", 0.01}}}};
        CostasLoop                 steered = makeCostas();
        const auto                 driven  = driveCostas(steered, std::span<const CF>(input), 0UZ, {true, true, true}, std::span<const gr::Tag>(steering));
        expect(eq(driven.tags.size(), 0UZ)) << "the loop tags are consumed";
        expect(approx(driven.aux[1][10UZ], 0.25f, 1e-6f)) << "and are in force on the tagged sample";

        const std::vector<gr::Tag> wild{gr::Tag{5UZ, gr::property_map{{"phase_est", 1e9}}}};
        CostasLoop                 wilded = makeCostas();
        const auto                 tamed  = driveCostas(wilded, std::span<const CF>(input), 0UZ, {true, true, true}, std::span<const gr::Tag>(wild));
        expect(le(std::abs(tamed.aux[1][5UZ]), static_cast<float>(kPi))) << "a payload of 1e9 is wrapped, not walked to, and leaves a canonical angle";

        const std::vector<gr::Tag> bad{gr::Tag{5UZ, gr::property_map{{"phase_est", std::numeric_limits<double>::quiet_NaN()}, {"freq_est", std::numeric_limits<double>::infinity()}}}};
        CostasLoop                 rejected = makeCostas();
        const auto                 guarded  = driveCostas(rejected, std::span<const CF>(input), 0UZ, {true, true, true}, std::span<const gr::Tag>(bad));
        CostasLoop                 plain    = makeCostas();
        const auto                 untagged = driveCostas(plain, std::span<const CF>(input), 0UZ, {true, true, true});
        expect(eq(rejected.ignoredTagPayloads(), 2ULL));
        expect(std::ranges::equal(guarded.samples, untagged.samples));
    };

    "the output does not depend on chunking, the order, or which side ports are wired"_test = [] {
        for (const std::size_t order : {2UZ, 4UZ, 8UZ}) {
            for (const bool soft : {false, true}) {
                const std::vector<CF>  input = pskStream(4000UZ, order, 1.0, 1.3e-3, 0.4, 17U);
                const gr::property_map settings{{"noise_bandwidth", kBandwidth}, {"order", static_cast<gr::Size_t>(order)}, {"soft_decisions", soft}};

                CostasLoop block     = makeCostas(settings);
                const auto reference = driveCostas(block, std::span<const CF>(input));

                for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 4096UZ}) {
                    for (const bool wired : {true, false}) {
                        CostasLoop other = makeCostas(settings);
                        expect(std::ranges::equal(driveCostas(other, std::span<const CF>(input), chunk, {wired, wired, wired}).samples, reference.samples)) << "order " << order << " soft " << soft << " chunk " << chunk;
                    }
                }
            }
        }
    };
};

int main() { /* not needed for ut */ }
