#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

#include <array>
#include <cmath>
#include <complex>
#include <numbers>
#include <random>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/sync/SymbolSync.hpp>

// Every figure is ns per input sample, which is the rate the block sits at; the outputs per input differ
// between arms by construction and a per-output figure would hide that a symbol at sps 8 costs the same as one at 2.

namespace {

using gr::blocks::sync::SymbolSync;
using CF       = std::complex<float>;
using CD       = std::complex<double>;
namespace test = gr::blocks::testing::span;

constexpr std::size_t kSamplesPerCall = 16384UZ;
constexpr std::size_t kRepeats        = 7UZ;

std::vector<CF> makeShapedQpsk() {
    constexpr int            kSps = 4;
    const std::vector<float> taps = gr::filter::fir::design::rootRaisedCosine(static_cast<std::size_t>(8 * kSps + 1), static_cast<double>(kSps), 0.35);

    std::mt19937                       rng(17U);
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
        signal[i] = static_cast<CF>(shaped[i]);
    }
    return signal;
}

SymbolSync<CF> make(double sps, std::string detector, double gain, std::string interpolator) {
    SymbolSync<CF> block({{"samples_per_symbol", sps}, {"detector", std::move(detector)}, {"detector_gain", gain}, {"interpolator", std::move(interpolator)}, {"rolloff", 0.35}, {"noise_bandwidth", 0.01}});
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

double drive(SymbolSync<CF>& block, std::span<const CF> input, std::span<CF> output) {
    block.reset();
    test::InputSpan<CF>     inSpan(input, 0UZ);
    test::OutputSpan<CF>    outSpan(output, 0UZ, nullptr);
    test::OutputSpan<float> a(std::span<float>{}, 0UZ, nullptr, false);
    test::OutputSpan<float> b(std::span<float>{}, 0UZ, nullptr, false);
    test::OutputSpan<float> c(std::span<float>{}, 0UZ, nullptr, false);
    std::ignore = block.processBulk(inSpan, outSpan, a, b, c);
    return static_cast<double>(std::abs(output[outSpan.count / 2UZ]));
}

} // namespace

int main() {
    const std::vector<CF> input = makeShapedQpsk();
    std::vector<CF>       output(kSamplesPerCall);

    SymbolSync<CF> mm2          = make(2.0, "MuellerMuller", 0.0, "mmse8");
    SymbolSync<CF> mm4          = make(4.0, "MuellerMuller", 0.0, "mmse8");
    SymbolSync<CF> mm8          = make(8.0, "MuellerMuller", 0.0, "mmse8");
    SymbolSync<CF> gardner2     = make(2.0, "Gardner", 1.06739, "mmse8");
    SymbolSync<CF> gardner4     = make(4.0, "Gardner", 1.06739, "mmse8");
    SymbolSync<CF> gardner8     = make(8.0, "Gardner", 1.06739, "mmse8");
    SymbolSync<CF> slope4       = make(4.0, "SignalTimesSlopeMl", 1.0, "mmse8");
    SymbolSync<CF> mmPoly4      = make(4.0, "MuellerMuller", 0.0, "polyphase");
    SymbolSync<CF> gardnerPoly4 = make(4.0, "Gardner", 1.06739, "polyphase");

    std::array<gr::blocks::testing::bench::Arm, 9UZ> arms{{
        {"M&M, mmse8, sps 2", [&] { return drive(mm2, input, std::span<CF>(output)); }},
        {"M&M, mmse8, sps 4", [&] { return drive(mm4, input, std::span<CF>(output)); }},
        {"M&M, mmse8, sps 8", [&] { return drive(mm8, input, std::span<CF>(output)); }},
        {"Gardner, mmse8, sps 2", [&] { return drive(gardner2, input, std::span<CF>(output)); }},
        {"Gardner, mmse8, sps 4", [&] { return drive(gardner4, input, std::span<CF>(output)); }},
        {"Gardner, mmse8, sps 8", [&] { return drive(gardner8, input, std::span<CF>(output)); }},
        {"signal x slope ML, mmse8, sps 4", [&] { return drive(slope4, input, std::span<CF>(output)); }},
        {"M&M, polyphase 128, sps 4", [&] { return drive(mmPoly4, input, std::span<CF>(output)); }},
        {"Gardner, polyphase 128, sps 4", [&] { return drive(gardnerPoly4, input, std::span<CF>(output)); }},
    }};

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kSamplesPerCall, kRepeats);
}
