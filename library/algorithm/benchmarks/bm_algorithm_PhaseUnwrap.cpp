#include <benchmark.hpp>

#include <gnuradio-4.0/algorithm/measurement/PhaseUnwrap.hpp>

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
constexpr std::size_t kRepeats = 200UZ;

[[nodiscard]] std::vector<Complex> tone(double f0, std::size_t n) {
    constexpr double     kTwoPi = 2. * std::numbers::pi_v<double>;
    std::vector<Complex> out(n);
    for (std::size_t k = 0UZ; k < n; ++k) {
        const double phase = kTwoPi * f0 * static_cast<double>(k);
        out[k]             = Complex{static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))};
    }
    return out;
}

} // namespace

/// Times `CycleUnwrapper::process` and a loop of `atan2` alone over the same input. Per sample, the unwrapper calls
/// `atan2` once and adds a few compares and adds.
void benchPhaseUnwrap() {
    using namespace benchmark;
    using boost::ut::operator""_test;

    "CycleUnwrapper: ns/sample, and the atan2 share"_test = [] {
        const std::vector<Complex> in = tone(0.37, kSamples);

        gr::measurement::CycleUnwrapper unwrapper;
        std::vector<std::int64_t>       cycles(kSamples);
        std::vector<float>              phase(kSamples);

        const auto name                                                    = std::format("CycleUnwrapper::process, N={}", kSamples);
        ::benchmark::benchmark<kRepeats>(std::string_view(name), kSamples) = [&] { std::ignore = unwrapper.process(in, cycles, phase); };

        const auto bare                                                    = std::format("atan2 alone,             N={}", kSamples);
        ::benchmark::benchmark<kRepeats>(std::string_view(bare), kSamples) = [&] {
            float sum = 0.f;
            for (const Complex sample : in) {
                sum += std::atan2(sample.imag(), sample.real());
            }
            ::benchmark::force_to_memory(sum);
        };
        ::benchmark::results::add_separator();
    };
}

inline const boost::ut::suite<"phase unwrap benchmarks"> _phase_unwrap_bm = [] { benchPhaseUnwrap(); };

int main() { /* not needed by the UT framework */ }
