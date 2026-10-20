#include <complex>
#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/filter/Channelizer.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::filter::PolyphaseChannelizer;
using gr::blocks::filter::PolyphaseSynthesizer;
namespace test = gr::blocks::testing::span;
using CF       = std::complex<float>;

/// The denominator is the wideband stream in both directions. That is the analysis bank's input and the synthesizer's
/// output. A consumer plans against that rate, and it makes the two banks directly comparable.
constexpr std::size_t kWidebandSamples = 65536UZ;
constexpr std::size_t kRepeats         = 9UZ;

constexpr std::array<gr::Size_t, 3UZ> kChannelCounts{4U, 16U, 64U};

template<typename TBlock>
[[nodiscard]] std::unique_ptr<TBlock> make(gr::property_map settings) {
    auto block = std::make_unique<TBlock>(std::move(settings));
    block->settings().init();
    std::ignore = block->settings().applyStagedParameters();
    block->start();
    return block;
}

[[nodiscard]] std::vector<CF> noise(std::size_t count) {
    std::vector<CF> data(count);
    std::uint64_t   state = 0x243f6a8885a308d3ULL;
    for (CF& sample : data) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        sample = CF(2.f * static_cast<float>(state % 1024ULL) / 1024.f - 1.f, 2.f * static_cast<float>((state >> 20U) % 1024ULL) / 1024.f - 1.f);
    }
    return data;
}

/// One analysis bank with the buffers its call needs, so the measured body allocates nothing.
struct Analysis {
    std::unique_ptr<PolyphaseChannelizer<float>> bank;
    std::vector<std::vector<CF>>                 channelOut;
    std::vector<test::OutputSpan<CF>>            outSpans;

    explicit Analysis(gr::Size_t channels) : bank(make<PolyphaseChannelizer<float>>({{"n_channels", channels}})) {
        channelOut.resize(bank->_channels);
        for (std::vector<CF>& one : channelOut) {
            one.assign(kWidebandSamples / bank->_stride, CF{});
        }
    }

    [[nodiscard]] double run(std::span<const CF> input) {
        outSpans.clear();
        for (std::vector<CF>& one : channelOut) {
            outSpans.emplace_back(std::span<CF>(one), 0UZ);
        }
        test::InputSpan<CF>             inSpan(input, 0UZ);
        std::span<test::OutputSpan<CF>> outputs(outSpans);
        std::ignore = bank->processBulk(inSpan, outputs);
        return static_cast<double>(channelOut[0][channelOut[0].size() / 2UZ].real());
    }
};

/// One synthesis bank, fed a per-channel stream that adds back up to the wideband rate.
struct Synthesis {
    std::unique_ptr<PolyphaseSynthesizer<float>> bank;
    std::vector<std::vector<CF>>                 channelIn;
    std::vector<test::InputSpan<CF>>             inSpans;
    std::vector<CF>                              output;

    explicit Synthesis(gr::Size_t channels, std::span<const CF> source) : bank(make<PolyphaseSynthesizer<float>>({{"n_channels", channels}})), output(kWidebandSamples, CF{}) {
        const std::size_t perChannel = kWidebandSamples / bank->_stride;
        channelIn.resize(bank->_channels);
        for (std::size_t channel = 0UZ; channel < channelIn.size(); ++channel) {
            channelIn[channel].assign(source.begin(), source.begin() + static_cast<std::ptrdiff_t>(perChannel));
        }
    }

    [[nodiscard]] double run() {
        inSpans.clear();
        for (const std::vector<CF>& one : channelIn) {
            inSpans.emplace_back(std::span<const CF>(one), 0UZ);
        }
        std::span<test::InputSpan<CF>> inputs(inSpans);
        std::ignore = bank->processBulk(inputs, std::span<CF>(output));
        return static_cast<double>(output[output.size() / 2UZ].real());
    }
};

} // namespace

int main() {
    const std::vector<CF> input = noise(kWidebandSamples);

    std::vector<std::unique_ptr<Analysis>>  analysis;
    std::vector<std::unique_ptr<Synthesis>> synthesis;
    for (const gr::Size_t channels : kChannelCounts) {
        analysis.push_back(std::make_unique<Analysis>(channels));
        synthesis.push_back(std::make_unique<Synthesis>(channels, std::span<const CF>(input)));
    }

    std::vector<gr::blocks::testing::bench::Arm> arms;
    for (std::size_t which = 0UZ; which < kChannelCounts.size(); ++which) {
        arms.push_back({std::format("PolyphaseChannelizer, {} channels", kChannelCounts[which]), [&, which] { return analysis[which]->run(std::span<const CF>(input)); }});
    }
    for (std::size_t which = 0UZ; which < kChannelCounts.size(); ++which) {
        arms.push_back({std::format("PolyphaseSynthesizer, {} channels", kChannelCounts[which]), [&, which] { return synthesis[which]->run(); }});
    }

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kWidebandSamples, kRepeats);
}
