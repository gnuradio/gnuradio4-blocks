#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

#include <array>
#include <cmath>
#include <complex>
#include <numbers>
#include <random>
#include <span>
#include <vector>

#include <gnuradio-4.0/sync/CostasLoop.hpp>

namespace {

using gr::blocks::sync::CostasLoop;
using CF       = std::complex<float>;
namespace test = gr::blocks::testing::span;

constexpr std::size_t kSamplesPerCall = 65536UZ;
constexpr std::size_t kRepeats        = 7UZ;

std::vector<CF> makeQpsk() {
    std::mt19937                       rng(11U);
    std::uniform_int_distribution<int> pick(0, 3);
    std::vector<CF>                    signal(kSamplesPerCall);
    for (std::size_t i = 0UZ; i < signal.size(); ++i) {
        const double symbol  = 0.25 * std::numbers::pi + 0.5 * std::numbers::pi * static_cast<double>(pick(rng));
        const double carrier = 2.0 * std::numbers::pi * 0.001 * static_cast<double>(i);
        signal[i]            = static_cast<CF>(std::polar(1.0, symbol + carrier));
    }
    return signal;
}

CostasLoop make(gr::Size_t order, bool soft) {
    CostasLoop block({{"order", order}, {"soft_decisions", soft}, {"detector_gain", 1.41421356}});
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

double drive(CostasLoop& block, std::span<const CF> input, std::span<CF> output) {
    test::InputSpan<CF>     inSpan(input, 0UZ);
    test::OutputSpan<CF>    outSpan(output, 0UZ, nullptr);
    test::OutputSpan<float> a(std::span<float>{}, 0UZ, nullptr, false);
    test::OutputSpan<float> b(std::span<float>{}, 0UZ, nullptr, false);
    test::OutputSpan<float> c(std::span<float>{}, 0UZ, nullptr, false);
    std::ignore = block.processBulk(inSpan, outSpan, a, b, c);
    return static_cast<double>(std::abs(output[kSamplesPerCall / 2UZ]));
}

} // namespace

int main() {
    const std::vector<CF> input = makeQpsk();
    std::vector<CF>       output(kSamplesPerCall);

    CostasLoop bpsk     = make(2U, false);
    CostasLoop qpsk     = make(4U, false);
    CostasLoop psk8     = make(8U, false);
    CostasLoop bpskSoft = make(2U, true);
    CostasLoop qpskSoft = make(4U, true);
    CostasLoop psk8Soft = make(8U, true);

    std::array<gr::blocks::testing::bench::Arm, 6UZ> arms{{
        {"CostasLoop order 2", [&] { return drive(bpsk, input, std::span<CF>(output)); }},
        {"CostasLoop order 4", [&] { return drive(qpsk, input, std::span<CF>(output)); }},
        {"CostasLoop order 8", [&] { return drive(psk8, input, std::span<CF>(output)); }},
        {"CostasLoop order 2, soft decisions", [&] { return drive(bpskSoft, input, std::span<CF>(output)); }},
        {"CostasLoop order 4, soft decisions", [&] { return drive(qpskSoft, input, std::span<CF>(output)); }},
        {"CostasLoop order 8, soft decisions", [&] { return drive(psk8Soft, input, std::span<CF>(output)); }},
    }};

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kSamplesPerCall, kRepeats);
}
