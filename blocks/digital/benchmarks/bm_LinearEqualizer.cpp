#include <array>
#include <complex>
#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/digital/LinearEqualizer.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::digital::LinearEqualizer;
namespace test = gr::blocks::testing::span;
using CF       = std::complex<float>;

constexpr std::size_t kSamplesPerCall = 32768UZ;
constexpr std::size_t kRepeats        = 9UZ;

/// The rows the cost varies over: the tap count is the inner loop, the update rule is what runs over it, and a
/// fractionally spaced filter runs the same loop at twice the input rate for the same symbol output.
struct Row {
    const char* name;
    gr::Size_t  taps;
    const char* algorithm;
    gr::Size_t  samplesPerSymbol;
};

constexpr std::array<Row, 5UZ> kRows{
    Row{"11 taps, lms", 11U, "lms", 1U},
    Row{"31 taps, lms", 31U, "lms", 1U},
    Row{"127 taps, lms", 127U, "lms", 1U},
    Row{"31 taps, nlms", 31U, "nlms", 1U},
    Row{"31 taps, cma", 31U, "cma", 1U},
};

/// A block carrying a seqlock is neither copyable nor movable, so each one is built where it is used and held by
/// pointer for the arm that drives it.
[[nodiscard]] std::unique_ptr<LinearEqualizer<float>> make(const Row& row) {
    auto block = std::make_unique<LinearEqualizer<float>>(gr::property_map{{"num_taps", row.taps}, {"algorithm", std::string(row.algorithm)}, {"samples_per_symbol", row.samplesPerSymbol}, {"step_size", 0.005}});
    block->settings().init();
    std::ignore = block->settings().applyStagedParameters();
    block->start();
    return block;
}

/// A noisy QPSK stream: the decision-directed reference has real decisions to make, which is the working condition.
[[nodiscard]] std::vector<CF> qpskWithNoise() {
    std::vector<CF> data(kSamplesPerCall);
    std::uint64_t   state = 0x243f6a8885a308d3ULL;
    const auto      next  = [&state] {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    };
    for (CF& sample : data) {
        const std::uint64_t bits   = next();
        const float         re     = (bits & 1ULL) ? 0.7071f : -0.7071f;
        const float         im     = (bits & 2ULL) ? 0.7071f : -0.7071f;
        const std::uint64_t jitter = next();
        sample                     = CF(re + 0.05f * (static_cast<float>(jitter % 1024ULL) / 512.f - 1.f), im + 0.05f * (static_cast<float>((jitter >> 20U) % 1024ULL) / 512.f - 1.f));
    }
    return data;
}

} // namespace

int main() {
    const std::vector<CF> input = qpskWithNoise();
    std::vector<CF>       output(kSamplesPerCall);

    std::vector<std::unique_ptr<LinearEqualizer<float>>> equalizers;
    for (const Row& row : kRows) {
        equalizers.push_back(make(row));
    }

    std::vector<gr::blocks::testing::bench::Arm> arms;
    for (std::size_t which = 0UZ; which < kRows.size(); ++which) {
        arms.push_back({std::format("LinearEqualizer, {}", kRows[which].name), [&, which] {
                            std::vector<gr::Tag> published;
                            test::InputSpan<CF>  inSpan(std::span<const CF>(input), 0UZ);
                            test::OutputSpan<CF> outSpan(std::span<CF>(output), 0UZ, &published);
                            std::ignore = equalizers[which]->processBulk(inSpan, outSpan);
                            return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
                        }});
    }

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kSamplesPerCall, kRepeats);
}
