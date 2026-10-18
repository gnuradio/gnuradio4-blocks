#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/measurement/PhaseUnwrap.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::measurement::PhaseUnwrap;
using gr::blocks::testing::bench::Arm;
using CF = std::complex<float>;

constexpr std::size_t kSamplesPerCall = 1UZ << 16;
constexpr std::size_t kRepeats        = 9UZ;

/// Applies a block's settings and starts it.
template<typename TBlock>
void init(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

/// A unit tone at 0.37 cycles per sample with a small seeded phase offset on each sample.
[[nodiscard]] std::vector<CF> tone() {
    std::vector<CF>  data(kSamplesPerCall);
    constexpr double twoPi = 2. * std::numbers::pi;
    std::uint64_t    state = 0x243f6a8885a308d3ULL;
    for (std::size_t k = 0UZ; k < data.size(); ++k) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        const double phase = twoPi * 0.37 * static_cast<double>(k) + 0.01 * (static_cast<double>(state % 1024ULL) / 512. - 1.);
        data[k]            = CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
    }
    return data;
}

[[nodiscard]] double sweep(PhaseUnwrap& block, std::span<const CF> input, std::span<std::int64_t> cycles, std::span<float> phase) {
    namespace shim = gr::blocks::testing::span;
    shim::InputSpan<CF>            inSpan(input);
    shim::OutputSpan<std::int64_t> cyclesSpan(cycles);
    shim::OutputSpan<float>        phaseSpan(phase);
    std::ignore = block.processBulk(inSpan, cyclesSpan, phaseSpan);
    return static_cast<double>(cycles[kSamplesPerCall / 2UZ]) + static_cast<double>(phase[kSamplesPerCall / 2UZ]);
}

/// @brief The phase of each sample by `std::arg` alone.
[[nodiscard]] double argOnly(std::span<const CF> input, std::span<float> phase) {
    for (std::size_t k = 0UZ; k < input.size(); ++k) {
        phase[k] = std::arg(input[k]);
    }
    return static_cast<double>(phase[kSamplesPerCall / 2UZ]);
}

} // namespace

int main() {
    const std::vector<CF>     input = tone();
    std::vector<std::int64_t> cycles(kSamplesPerCall);
    std::vector<float>        phase(kSamplesPerCall);

    PhaseUnwrap unwrapper({{"origin", std::string("first_sample")}});
    PhaseUnwrap unwrapperZero({{"origin", std::string("zero")}});
    init(unwrapper);
    init(unwrapperZero);

    std::vector<Arm> arms{
        {"std::arg alone", [&] { return argOnly(std::span<const CF>(input), std::span<float>(phase)); }},
        {"PhaseUnwrap, origin first_sample", [&] { return sweep(unwrapper, std::span<const CF>(input), std::span<std::int64_t>(cycles), std::span<float>(phase)); }},
        {"PhaseUnwrap, origin zero", [&] { return sweep(unwrapperZero, std::span<const CF>(input), std::span<std::int64_t>(cycles), std::span<float>(phase)); }},
    };

    std::println("ns per sample. The first arm computes the phase alone.");
    gr::blocks::testing::bench::report(std::span<Arm>(arms), kSamplesPerCall, kRepeats);
}
