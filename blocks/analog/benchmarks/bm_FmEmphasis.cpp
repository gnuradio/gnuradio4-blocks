#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <cmath>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/analog/FmEmphasis.hpp>

namespace {

constexpr std::size_t kSamplesPerCall = 65536UZ;
constexpr std::size_t kIterations     = 512UZ;

std::vector<float> makeAudioBurst() {
    std::vector<float> signal(kSamplesPerCall);
    for (std::size_t i = 0UZ; i < signal.size(); ++i) {
        const double phase = 2.0 * std::numbers::pi * 997.0 * static_cast<double>(i) / 48000.0;
        signal[i]          = static_cast<float>(0.5 * std::sin(phase));
    }
    return signal;
}

template<typename TBlock>
TBlock makeBlock() {
    TBlock block;
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

} // namespace

int main() {
    const std::vector<float> input = makeAudioBurst();
    std::vector<float>       output(kSamplesPerCall);

    auto deemphasis = makeBlock<gr::blocks::analog::FmDeemphasis<float>>();
    gr::blocks::testing::bench::report("FmDeemphasis<float>", kSamplesPerCall, kIterations, [&deemphasis, &input, &output] {
        std::ignore = deemphasis.processBulk(std::span<const float>(input), std::span<float>(output));
        return static_cast<double>(output[kSamplesPerCall / 2UZ]);
    });

    auto preemphasis = makeBlock<gr::blocks::analog::FmPreemphasis<float>>();
    gr::blocks::testing::bench::report("FmPreemphasis<float>", kSamplesPerCall, kIterations, [&preemphasis, &input, &output] {
        std::ignore = preemphasis.processBulk(std::span<const float>(input), std::span<float>(output));
        return static_cast<double>(output[kSamplesPerCall / 2UZ]);
    });
}
