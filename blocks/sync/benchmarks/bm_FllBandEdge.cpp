#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

#include <array>
#include <cmath>
#include <complex>
#include <numbers>
#include <random>
#include <span>
#include <vector>

#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/sync/FllBandEdge.hpp>

namespace {

using gr::blocks::sync::FllBandEdge;
using CF       = std::complex<float>;
using CD       = std::complex<double>;
namespace test = gr::blocks::testing::span;

constexpr std::size_t kSamplesPerCall = 16384UZ;
constexpr std::size_t kRepeats        = 7UZ;

std::vector<CF> makeShapedQpsk() {
    constexpr int            kSps = 4;
    const std::vector<float> taps = gr::filter::fir::design::rootRaisedCosine(static_cast<std::size_t>(8 * kSps + 1), static_cast<double>(kSps), 0.35);

    std::mt19937                       rng(13U);
    std::uniform_int_distribution<int> pick(0, 3);
    std::vector<CD>                    shaped(kSamplesPerCall, CD{});
    for (std::size_t k = 0UZ; k * static_cast<std::size_t>(kSps) < kSamplesPerCall; ++k) {
        const CD symbol = std::polar(1.0, 0.25 * std::numbers::pi + 0.5 * std::numbers::pi * static_cast<double>(pick(rng)));
        for (std::size_t j = 0UZ; j < taps.size(); ++j) {
            const std::size_t at = k * static_cast<std::size_t>(kSps) + j;
            if (at < kSamplesPerCall) {
                shaped[at] += symbol * static_cast<double>(taps[j]);
            }
        }
    }

    std::vector<CF> signal(kSamplesPerCall);
    for (std::size_t i = 0UZ; i < kSamplesPerCall; ++i) {
        signal[i] = static_cast<CF>(shaped[i] * std::polar(1.0, 2.0 * std::numbers::pi * 0.01 * static_cast<double>(i)));
    }
    return signal;
}

FllBandEdge make(gr::Size_t nTaps, bool normalized) {
    FllBandEdge block({{"samples_per_symbol", 4.0}, {"rolloff", 0.35}, {"filter_length", nTaps}, {"normalized_discriminant", normalized}});
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

double drive(FllBandEdge& block, std::span<const CF> input, std::span<CF> output) {
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
    const std::vector<CF> input = makeShapedQpsk();
    std::vector<CF>       output(kSamplesPerCall);

    FllBandEdge short33   = make(33U, false);
    FllBandEdge mid45     = make(45U, false);
    FllBandEdge long89    = make(89U, false);
    FllBandEdge mid45norm = make(45U, true);

    std::array<gr::blocks::testing::bench::Arm, 4UZ> arms{{
        {"FllBandEdge N=33", [&] { return drive(short33, input, std::span<CF>(output)); }},
        {"FllBandEdge N=45", [&] { return drive(mid45, input, std::span<CF>(output)); }},
        {"FllBandEdge N=89", [&] { return drive(long89, input, std::span<CF>(output)); }},
        {"FllBandEdge N=45, normalized", [&] { return drive(mid45norm, input, std::span<CF>(output)); }},
    }};

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kSamplesPerCall, kRepeats);
}
