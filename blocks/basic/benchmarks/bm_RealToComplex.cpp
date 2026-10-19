#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <complex>
#include <cstddef>
#include <format>
#include <random>
#include <span>
#include <utility>
#include <vector>

#include <gnuradio-4.0/algorithm/filter/RealToComplex.hpp>
#include <gnuradio-4.0/basic/RealToComplex.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::testing::bench::Arm;

using CF = std::complex<float>;

constexpr std::size_t kSamples = 1UZ << 20;
constexpr std::size_t kChunk   = 8192UZ;
constexpr std::size_t kRepeats = 7UZ;

[[nodiscard]] gr::blocks::basic::RealToComplex<float> makeBlock(float sampleRate, float transitionHz) {
    gr::blocks::basic::RealToComplex<float> block({{"sample_rate", sampleRate}, {"transition_hz", transitionHz}});
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

} // namespace

/// Each pair of arms converts the same input in calls of `kChunk` samples: the kernel alone, then the block through
/// test spans with the input's stream index. The figure is per input sample.
int main() {
    std::mt19937                          rng(1U);
    std::uniform_real_distribution<float> uniform(-1.f, 1.f);
    std::vector<float>                    x(kSamples);
    for (float& v : x) {
        v = uniform(rng);
    }
    std::vector<CF> y(kChunk / 2UZ + 1UZ);

    constexpr std::pair<float, float>                    kSettings[]{{500'000.f, 20'000.f}, {48'000.f, 200.f}};
    std::vector<gr::filter::RealToComplex>               kernels;
    std::vector<gr::blocks::basic::RealToComplex<float>> blocks;
    for (const auto& [rate, transition] : kSettings) {
        kernels.push_back(gr::filter::RealToComplex::make(static_cast<double>(rate), static_cast<double>(transition)).value());
        blocks.push_back(makeBlock(rate, transition));
    }

    std::vector<Arm> arms;
    for (std::size_t a = 0UZ; a < std::size(kSettings); ++a) {
        const std::size_t nTaps = kernels[a].taps().size();
        arms.emplace_back(std::format("RealToComplex kernel, {} taps", nTaps), kSamples, [&kernels, &x, &y, a] {
            for (std::size_t base = 0UZ; base < kSamples; base += kChunk) {
                std::ignore = kernels[a].process(std::span<const float>(x).subspan(base, kChunk), std::span<CF>(y));
            }
            return static_cast<double>(y[kChunk / 4UZ].real());
        });
        arms.emplace_back(std::format("RealToComplex<float> block, {} taps", nTaps), kSamples, [&blocks, &x, &y, a] {
            namespace test      = gr::blocks::testing::span;
            std::size_t written = 0UZ;
            for (std::size_t base = 0UZ; base < kSamples;) {
                test::InputSpan<float> inSpan(std::span<const float>(x).subspan(base, kChunk), base);
                test::OutputSpan<CF>   outSpan(std::span<CF>(y), written);
                std::ignore = blocks[a].processBulk(inSpan, outSpan);
                base += inSpan.consumed;
                written += outSpan.count;
            }
            return static_cast<double>(y[kChunk / 4UZ].real());
        });
    }

    gr::blocks::testing::bench::report(std::span<Arm>(arms), kRepeats);
}
