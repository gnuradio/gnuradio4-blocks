#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <algorithm>
#include <cstddef>
#include <format>
#include <span>
#include <vector>

#include <gnuradio-4.0/basic/SampleDelay.hpp>

namespace {

using gr::blocks::basic::SampleDelay;
using gr::blocks::testing::bench::Arm;

constexpr std::size_t kSamples = 1UZ << 22;
constexpr std::size_t kChunk   = 4096UZ;
constexpr std::size_t kRepeats = 7UZ;
constexpr gr::Size_t  kDelays[]{1U, 8U, 80U, 4095U, 65536U};

[[nodiscard]] SampleDelay<float> makeDelay(gr::Size_t delay) {
    SampleDelay<float> block({{"delay", delay}});
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

} // namespace

int main() {
    std::vector<float> x(kSamples);
    for (std::size_t i = 0UZ; i < x.size(); ++i) {
        x[i] = static_cast<float>(i % 1024UZ);
    }
    std::vector<float> y(kSamples);

    std::vector<SampleDelay<float>> blocks;
    for (const gr::Size_t delay : kDelays) {
        blocks.push_back(makeDelay(delay));
    }

    std::vector<Arm> arms;
    arms.emplace_back("plain span copy, the 1:1 floor", kSamples, [&x, &y] {
        for (std::size_t base = 0UZ; base < kSamples; base += kChunk) {
            std::copy_n(x.begin() + static_cast<std::ptrdiff_t>(base), kChunk, y.begin() + static_cast<std::ptrdiff_t>(base));
        }
        return static_cast<double>(y[kSamples / 2UZ]);
    });
    for (std::size_t a = 0UZ; a < std::size(kDelays); ++a) {
        arms.emplace_back(std::format("SampleDelay<float>, delay {}", kDelays[a]), kSamples, [&blocks, &x, &y, a] {
            for (std::size_t base = 0UZ; base < kSamples; base += kChunk) {
                std::ignore = blocks[a].processBulk(std::span<const float>(x).subspan(base, kChunk), std::span<float>(y).subspan(base, kChunk));
            }
            return static_cast<double>(y[kSamples / 2UZ]);
        });
    }

    gr::blocks::testing::bench::report(std::span<Arm>(arms), kRepeats);
}
