#include <array>
#include <complex>
#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/fourier/FrequencyCompressor.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::fourier::FrequencyCompressor;
namespace test = gr::blocks::testing::span;
using CF       = std::complex<float>;

/// The denominator is the input stream, the rate a consumer plans against.
constexpr std::size_t kInputSamples = 65536UZ;
constexpr std::size_t kRepeats      = 9UZ;
constexpr gr::Size_t  kFrame        = 4096U;

constexpr std::array<gr::Size_t, 4UZ> kDivisors{1U, 4U, 8U, 32U};

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

/// One compressor with the output buffer its call needs. The measured body allocates nothing after warm-up.
struct Compression {
    std::unique_ptr<FrequencyCompressor> block;
    std::vector<CF>                      output;

    explicit Compression(gr::Size_t divisor) : block(std::make_unique<FrequencyCompressor>(gr::property_map{{"divisor", divisor}, {"frame", kFrame}})), output(kInputSamples, CF{}) {
        block->settings().init();
        std::ignore = block->settings().applyStagedParameters();
        block->start();
    }

    [[nodiscard]] double run(std::span<const CF> input) {
        test::InputSpan<CF>  inSpan(input, 0UZ);
        test::OutputSpan<CF> outSpan(std::span<CF>(output), 0UZ);
        std::ignore = block->processBulk(inSpan, outSpan);
        return static_cast<double>(output[0].real());
    }
};

} // namespace

int main() {
    const std::vector<CF> input = noise(kInputSamples);

    std::vector<std::unique_ptr<Compression>> compressions;
    for (const gr::Size_t divisor : kDivisors) {
        compressions.push_back(std::make_unique<Compression>(divisor));
    }

    std::vector<gr::blocks::testing::bench::Arm> arms;
    for (std::size_t which = 0UZ; which < kDivisors.size(); ++which) {
        arms.push_back({std::format("FrequencyCompressor, frame {}, divisor {}", kFrame, kDivisors[which]), [&, which] { return compressions[which]->run(std::span<const CF>(input)); }});
    }

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kInputSamples, kRepeats);
}
