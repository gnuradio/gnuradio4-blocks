#include <benchmark.hpp>

#include <gnuradio-4.0/algorithm/filter/FractionalDelay.hpp>
#include <gnuradio-4.0/algorithm/timing/FrequencySchedule.hpp>
#include <gnuradio-4.0/algorithm/timing/SampleClock.hpp>

#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <numbers>
#include <string_view>
#include <tuple>
#include <vector>

namespace {

using Complex = std::complex<float>;

constexpr std::size_t kSamples = 65'536UZ;
constexpr std::size_t kRepeats = 50UZ;
constexpr double      kRolloff = 0.2;

[[nodiscard]] std::vector<Complex> tone(double f0, std::size_t n) {
    constexpr double     kTwoPi = 2. * std::numbers::pi_v<double>;
    std::vector<Complex> out(n);
    for (std::size_t k = 0UZ; k < n; ++k) {
        const double phase = kTwoPi * f0 * static_cast<double>(k);
        out[k]             = Complex{static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))};
    }
    return out;
}

/// A pass-shaped delay table. The delay falls from 8.339 ms, the delay of a 2500 km slant range, to 2.5 ms over ten
/// minutes.
[[nodiscard]] gr::timing::DelaySchedule passDelays(std::size_t nKnots) {
    constexpr std::int64_t kSpanNs = 600'000'000'000LL;

    std::vector<std::int64_t> times(nKnots);
    std::vector<double>       delays(nKnots);
    for (std::size_t i = 0UZ; i < nKnots; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(nKnots - 1UZ);
        times[i]       = static_cast<std::int64_t>(static_cast<double>(kSpanNs) * u);
        delays[i]      = 8.339e-3 - 5.839e-3 * u;
    }
    return gr::timing::DelaySchedule(times, delays);
}

} // namespace

/// The per-sample cost of the delay line at the three orders and two bank sizes, and separately the cost of
/// feeding it a delay. The second arm tests that the schedule walk and the fixed-point conversion are a small
/// share of the total. Both are O(1) amortized per sample, against `(q+1)*(B+1)` taps.
void benchFractionalDelay() {
    using namespace benchmark;
    using boost::ut::operator""_test;

    "FractionalDelayLine: ns/sample"_test = [] {
        const std::vector<Complex> in = tone(0.17, kSamples);
        std::vector<Complex>       out(kSamples);

        struct Arm {
            std::size_t bank;
            int         order;
        };
        for (const Arm arm : {Arm{32UZ, 0}, Arm{32UZ, 1}, Arm{32UZ, 3}, Arm{128UZ, 1}}) {
            const gr::filter::ResamplerDesign        design = gr::filter::designFractionalDelay(arm.bank, kRolloff).value();
            gr::filter::FractionalDelayLine<Complex> line(arm.bank, arm.order, design.taps, 512ULL);

            std::vector<std::uint64_t> delays(kSamples);
            for (std::size_t k = 0UZ; k < kSamples; ++k) {
                delays[k] = gr::filter::fractionalDelayQ32(400. + 100. * std::sin(static_cast<double>(k) * 1e-4)).value();
            }

            const auto name                                                    = std::format("delay line, L={:3} q={} B={:3}, N={}", arm.bank, arm.order, line.tapsPerArm(), kSamples);
            ::benchmark::benchmark<kRepeats>(std::string_view(name), kSamples) = [&] { std::ignore = line.process(in, delays, out); };
        }
        ::benchmark::results::add_separator();
    };

    "the schedule walk's own share"_test = [] {
        const gr::timing::DelaySchedule schedule = passDelays(1'000UZ);
        const gr::timing::SampleClock   clock(0ULL, 0LL, 1'000'000ULL, 1ULL);

        std::vector<double>        seconds(kSamples);
        std::vector<std::uint64_t> delays(kSamples);

        const auto walk                                                 = std::format("DelaySchedule::valuesFor, 1000 knots, N={}", kSamples);
        ::benchmark::benchmark<200UZ>(std::string_view(walk), kSamples) = [&] { schedule.valuesFor(clock, 1'000'000ULL, seconds); };

        schedule.valuesFor(clock, 1'000'000ULL, seconds);
        const auto convert                                                 = std::format("fractionalDelayQ32 over a span,   N={}", kSamples);
        ::benchmark::benchmark<200UZ>(std::string_view(convert), kSamples) = [&] { std::ignore = gr::filter::fractionalDelayQ32(seconds, 1e6, delays); };
        ::benchmark::results::add_separator();
    };
}

inline const boost::ut::suite<"fractional delay benchmarks"> _fractional_delay_bm = [] { benchFractionalDelay(); };

int main() { /* not needed by the UT framework */ }
