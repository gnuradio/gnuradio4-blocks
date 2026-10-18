#include <complex>
#include <cstddef>
#include <cstdint>
#include <print>
#include <span>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/measurement/Kurtosis.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::measurement::Kurtosis;
using gr::blocks::testing::bench::Arm;
using CF = std::complex<float>;

constexpr std::size_t kSamplesPerCall = 1UZ << 16;
constexpr std::size_t kMaxRecords     = 64UZ; // more than the shortest window completes in one call
constexpr std::size_t kRepeats        = 9UZ;
constexpr float       kSampleRate     = 1.0e6f;

/// Applies a block's settings and starts it.
template<typename TBlock>
void init(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

[[nodiscard]] std::vector<CF> noise() {
    std::vector<CF> data(kSamplesPerCall);
    std::uint64_t   state = 0x243f6a8885a308d3ULL;
    for (CF& sample : data) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        sample = CF(2.f * static_cast<float>(state % 1024ULL) / 1024.f - 1.f, 2.f * static_cast<float>((state >> 20U) % 1024ULL) / 1024.f - 1.f);
    }
    return data;
}

/// @brief Runs one `processBulk` call over the input. Returns the first value of the first record, or the
/// slot's reading when no record went out.
template<typename T>
[[nodiscard]] double sweep(Kurtosis<T>& block, std::span<const T> input, std::span<gr::DataSet<float>> records, bool recordsConnected) {
    namespace shim = gr::blocks::testing::span;
    shim::InputSpan<T>                   inSpan(input);
    shim::OutputSpan<gr::DataSet<float>> outSpan(records, 0UZ, nullptr, recordsConnected);
    std::ignore = block.processBulk(inSpan, outSpan);
    return outSpan.count == 0UZ ? block.excessKurtosis() : static_cast<double>(records[0UZ].signal_values[0UZ]);
}

} // namespace

int main() {
    const std::vector<CF> complexInput = noise();
    std::vector<float>    realInput(kSamplesPerCall);
    for (std::size_t k = 0UZ; k < kSamplesPerCall; ++k) {
        realInput[k] = complexInput[k].real();
    }
    std::vector<gr::DataSet<float>> records(kMaxRecords);

    Kurtosis<CF>    complex4096({{"window", gr::Size_t{4096U}}, {"sample_rate", kSampleRate}});
    Kurtosis<CF>    complexNoRecords({{"window", gr::Size_t{4096U}}, {"sample_rate", kSampleRate}, {"emit_records", false}});
    Kurtosis<CF>    complex65536({{"window", gr::Size_t{65536U}}, {"sample_rate", kSampleRate}});
    Kurtosis<float> real4096({{"window", gr::Size_t{4096U}}, {"sample_rate", kSampleRate}});
    init(complex4096);
    init(complexNoRecords);
    init(complex65536);
    init(real4096);

    std::vector<Arm> arms{
        {"Kurtosis<complex> window 4096, records", [&] { return sweep(complex4096, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records), true); }},
        {"Kurtosis<complex> window 4096, no records", [&] { return sweep(complexNoRecords, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records), false); }},
        {"Kurtosis<complex> window 65536, records", [&] { return sweep(complex65536, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records), true); }},
        {"Kurtosis<float> window 4096, records", [&] { return sweep(real4096, std::span<const float>(realInput), std::span<gr::DataSet<float>>(records), true); }},
    };

    std::println("ns per input sample. A 65536-sample call over a 4096-sample window makes sixteen records.");
    std::println("The no-records arm builds no record.");
    gr::blocks::testing::bench::report(std::span<Arm>(arms), kSamplesPerCall, kRepeats);
}
