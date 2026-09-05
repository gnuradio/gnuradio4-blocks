#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <random>
#include <span>
#include <vector>

#include <gnuradio-4.0/sync/PreambleTiming.hpp>

// Every figure is ns per input sample, which is the rate the block sits at. The disabled arm is what a continuous
// link pays for carrying a stage it does not use: the recipe dialect has no conditional, so the stage is in every
// chain and `preamble_symbols = 0` is how a link turns it off.

namespace {

using gr::blocks::sync::PreambleTiming;
namespace test = gr::blocks::testing::span;

constexpr std::size_t kSamplesPerCall = 16384UZ;
constexpr std::size_t kRepeats        = 7UZ;
constexpr float       kSampleRate     = 48000.f;
constexpr float       kSymbolRate     = 9600.f;
constexpr std::size_t kSps            = 5UZ;

/// @brief An alternating training tone under noise, which is the stream the detector is armed on.
std::vector<float> makeTone() {
    std::mt19937                     rng(23U);
    std::normal_distribution<double> noise(0.0, 0.2);

    std::vector<float> stream(kSamplesPerCall);
    for (std::size_t k = 0UZ; k < kSamplesPerCall; ++k) {
        const double phase = std::numbers::pi * static_cast<double>(k) / static_cast<double>(kSps);
        stream[k]          = static_cast<float>(0.35 * std::cos(phase) + noise(rng));
    }
    return stream;
}

PreambleTiming<float> make(std::size_t preambleSymbols) {
    PreambleTiming<float> block({{"sample_rate", kSampleRate}, {"symbol_rate", kSymbolRate}, {"preamble_symbols", static_cast<gr::Size_t>(preambleSymbols)}});
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

double drive(PreambleTiming<float>& block, std::span<const float> input, std::span<float> output, bool wantStatistic) {
    block.reset();
    test::InputSpan<float>  inSpan(input, 0UZ);
    test::OutputSpan<float> outSpan(output, 0UZ, nullptr);
    test::OutputSpan<float> side(std::span<float>{}, 0UZ, nullptr, wantStatistic);
    std::ignore = block.processBulk(inSpan, outSpan, side);
    return static_cast<double>(std::abs(output[outSpan.count / 2UZ]));
}

} // namespace

int main() {
    const std::vector<float> input = makeTone();
    std::vector<float>       output(kSamplesPerCall);

    PreambleTiming<float> disabled = make(0UZ);
    PreambleTiming<float> ais      = make(24UZ);
    PreambleTiming<float> longer   = make(64UZ);

    std::array<gr::blocks::testing::bench::Arm, 3UZ> arms{{
        {"disabled (preamble_symbols 0), the wire", [&] { return drive(disabled, std::span<const float>(input), std::span<float>(output), false); }},
        {"enabled, 24 symbols (AIS), sps 5", [&] { return drive(ais, std::span<const float>(input), std::span<float>(output), false); }},
        {"enabled, 64 symbols, sps 5", [&] { return drive(longer, std::span<const float>(input), std::span<float>(output), false); }},
    }};

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kSamplesPerCall, kRepeats);
}
