#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/measurement/PolarizationCombiner.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::measurement::PolarizationCombiner;
using gr::blocks::testing::bench::Arm;
using CF = std::complex<float>;

constexpr std::size_t kSamplesPerCall = 1UZ << 16;
constexpr std::size_t kMaxRecords     = 64UZ;
constexpr std::size_t kRepeats        = 9UZ;
constexpr float       kSampleRate     = 1.0e6f;

/// Applies a block's settings and starts it.
template<typename TBlock>
void init(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

struct Branches {
    std::vector<CF> first;
    std::vector<CF> second;
};

/// One tone seen on two branches with a relative phase and unequal gains, each under its own noise.
[[nodiscard]] Branches scene() {
    Branches         b{std::vector<CF>(kSamplesPerCall), std::vector<CF>(kSamplesPerCall)};
    constexpr double twoPi = 2. * std::numbers::pi;
    std::uint64_t    state = 0x243f6a8885a308d3ULL;
    const auto       draw  = [&state] {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return static_cast<float>(state % 1024ULL) / 512.f - 1.f;
    };
    for (std::size_t k = 0UZ; k < kSamplesPerCall; ++k) {
        const double phase = twoPi * 0.1231 * static_cast<double>(k);
        const CF     s(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
        b.first[k]  = s + CF(0.3f * draw(), 0.3f * draw());
        b.second[k] = 0.5f * s * CF(std::cos(0.7f), std::sin(0.7f)) + CF(0.3f * draw(), 0.3f * draw());
    }
    return b;
}

[[nodiscard]] double sweep(PolarizationCombiner& block, const Branches& in, std::span<CF> out, std::span<CF> ortho, bool orthoConnected, std::span<gr::DataSet<float>> records, bool recordsConnected) {
    namespace shim = gr::blocks::testing::span;
    shim::InputSpan<CF>                  in0(std::span<const CF>(in.first));
    shim::InputSpan<CF>                  in1(std::span<const CF>(in.second));
    shim::OutputSpan<CF>                 outSpan(out);
    shim::OutputSpan<CF>                 orthoSpan(ortho, 0UZ, nullptr, orthoConnected);
    shim::OutputSpan<gr::DataSet<float>> recordSpan(records, 0UZ, nullptr, recordsConnected);
    std::ignore = block.processBulk(in0, in1, outSpan, orthoSpan, recordSpan);
    return static_cast<double>(out[kSamplesPerCall / 2UZ].real());
}

} // namespace

int main() {
    const Branches                  input = scene();
    std::vector<CF>                 out(kSamplesPerCall);
    std::vector<CF>                 ortho(kSamplesPerCall);
    std::vector<gr::DataSet<float>> records(kMaxRecords);

    PolarizationCombiner withOrtho({{"window", gr::Size_t{4096U}}, {"sample_rate", kSampleRate}});
    PolarizationCombiner withoutOrtho({{"window", gr::Size_t{4096U}}, {"sample_rate", kSampleRate}, {"emit_orthogonal", false}});
    PolarizationCombiner noRecords({{"window", gr::Size_t{4096U}}, {"sample_rate", kSampleRate}, {"emit_orthogonal", false}, {"emit_records", false}});
    PolarizationCombiner selection({{"window", gr::Size_t{4096U}}, {"sample_rate", kSampleRate}, {"mode", std::string("selection")}, {"emit_orthogonal", false}});
    init(withOrtho);
    init(withoutOrtho);
    init(noRecords);
    init(selection);

    std::vector<Arm> arms{
        {"PolarizationCombiner mrc, ortho and records", [&] { return sweep(withOrtho, input, std::span<CF>(out), std::span<CF>(ortho), true, std::span<gr::DataSet<float>>(records), true); }},
        {"PolarizationCombiner mrc, records, no ortho", [&] { return sweep(withoutOrtho, input, std::span<CF>(out), std::span<CF>(ortho), false, std::span<gr::DataSet<float>>(records), true); }},
        {"PolarizationCombiner mrc, no ortho, no records", [&] { return sweep(noRecords, input, std::span<CF>(out), std::span<CF>(ortho), false, std::span<gr::DataSet<float>>(records), false); }},
        {"PolarizationCombiner selection, no ortho", [&] { return sweep(selection, input, std::span<CF>(out), std::span<CF>(ortho), false, std::span<gr::DataSet<float>>(records), true); }},
    };

    std::println("ns per input sample, one sample on each branch. The first two arms differ by the orthogonal output.");
    gr::blocks::testing::bench::report(std::span<Arm>(arms), kSamplesPerCall, kRepeats);
}
