#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

#include <array>
#include <cmath>
#include <complex>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/sync/Pll.hpp>

namespace {

using gr::blocks::sync::PllCarrierTracking;
using gr::blocks::sync::PllFreqDet;
using gr::blocks::sync::PllRefOut;
using CF       = std::complex<float>;
namespace test = gr::blocks::testing::span;

constexpr std::size_t kSamplesPerCall = 65536UZ;
constexpr std::size_t kRepeats        = 7UZ;

std::vector<CF> makeTone() {
    std::vector<CF> signal(kSamplesPerCall);
    for (std::size_t i = 0UZ; i < signal.size(); ++i) {
        const double phase = 2.0 * std::numbers::pi * 0.002 * static_cast<double>(i);
        signal[i]          = CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
    }
    return signal;
}

template<typename TBlock>
void configure(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
}

/// @brief One `processBulk` over the whole buffer, with the side ports wired or not. No allocation inside the timer.
template<typename TBlock, typename TOut>
double drive(TBlock& block, std::span<const CF> input, std::span<TOut> output, std::span<float> side, bool wired) {
    test::InputSpan<CF>     inSpan(input, 0UZ);
    test::OutputSpan<TOut>  outSpan(output, 0UZ, nullptr);
    test::OutputSpan<float> a(wired ? side : std::span<float>{}, 0UZ, nullptr, wired);
    test::OutputSpan<float> b(wired ? side : std::span<float>{}, 0UZ, nullptr, wired);
    test::OutputSpan<float> c(wired ? side : std::span<float>{}, 0UZ, nullptr, wired);
    std::ignore = block.processBulk(inSpan, outSpan, a, b, c);
    return static_cast<double>(std::abs(output[kSamplesPerCall / 2UZ]));
}

} // namespace

int main() {
    const std::vector<CF> input = makeTone();
    std::vector<CF>       complexOut(kSamplesPerCall);
    std::vector<float>    realOut(kSamplesPerCall);
    std::vector<float>    sideOut(kSamplesPerCall);

    PllCarrierTracking bare;
    PllCarrierTracking wired;
    PllCarrierTracking squelched({{"squelch_when_unlocked", true}});
    PllFreqDet         detector;
    PllFreqDet         detectorWired;
    PllRefOut          reference;
    PllRefOut          referenceWired;
    configure(bare);
    configure(wired);
    configure(squelched);
    configure(detector);
    configure(detectorWired);
    configure(reference);
    configure(referenceWired);

    /// @brief PllFreqDet carries two optional ports rather than three, so it needs its own call rather than `drive`.
    const auto driveDetector = [&](PllFreqDet& block, bool sideWired) {
        test::InputSpan<CF>     inSpan(input, 0UZ);
        test::OutputSpan<float> outSpan(std::span<float>(realOut), 0UZ, nullptr);
        test::OutputSpan<float> a(sideWired ? std::span<float>(sideOut) : std::span<float>{}, 0UZ, nullptr, sideWired);
        test::OutputSpan<float> b(sideWired ? std::span<float>(sideOut) : std::span<float>{}, 0UZ, nullptr, sideWired);
        std::ignore = block.processBulk(inSpan, outSpan, a, b);
        return static_cast<double>(realOut[kSamplesPerCall / 2UZ]);
    };

    // The wired and unwired shapes of every block are here together because the lock detector's cost is exactly the
    // difference between them: it is the only per-sample work the connection state switches on.
    std::array<gr::blocks::testing::bench::Arm, 7UZ> arms{{
        {"PllCarrierTracking, no side ports", [&] { return drive(bare, input, std::span<CF>(complexOut), sideOut, false); }},
        {"PllCarrierTracking, freq+phase+lock", [&] { return drive(wired, input, std::span<CF>(complexOut), sideOut, true); }},
        {"PllCarrierTracking, squelched", [&] { return drive(squelched, input, std::span<CF>(complexOut), sideOut, false); }},
        {"PllFreqDet, no side ports", [&] { return driveDetector(detector, false); }},
        {"PllFreqDet, phase+lock", [&] { return driveDetector(detectorWired, true); }},
        {"PllRefOut, no side ports", [&] { return drive(reference, input, std::span<CF>(complexOut), sideOut, false); }},
        {"PllRefOut, freq+phase+lock", [&] { return drive(referenceWired, input, std::span<CF>(complexOut), sideOut, true); }},
    }};

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kSamplesPerCall, kRepeats);
}
