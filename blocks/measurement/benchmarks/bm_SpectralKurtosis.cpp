#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <print>
#include <span>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/measurement/SpectralEstimate.hpp>
#include <gnuradio-4.0/measurement/SpectralKurtosis.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::measurement::SpectralKurtosis;
using gr::blocks::measurement::WelchPsd;
using gr::blocks::testing::bench::Arm;
using CF = std::complex<float>;

constexpr std::size_t kFftSize        = 1024UZ;
constexpr std::size_t kAverages       = 16UZ;
constexpr std::size_t kRecordsPerCall = 64UZ; // two readings at n_spectra 32
constexpr std::size_t kRepeats        = 9UZ;
constexpr float       kSampleRate     = 1.0e6f;

/// Applies a block's settings and starts it.
template<typename TBlock>
void init(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

[[nodiscard]] std::vector<CF> noise(std::size_t n) {
    std::vector<CF> data(n);
    std::uint64_t   state = 0x243f6a8885a308d3ULL;
    for (CF& sample : data) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        sample = CF(2.f * static_cast<float>(state % 1024ULL) / 1024.f - 1.f, 2.f * static_cast<float>((state >> 20U) % 1024ULL) / 1024.f - 1.f);
    }
    return data;
}

/// @brief Welch estimates of noise at zero overlap, of `kFftSize` bins each. The input makes `kRecordsPerCall` records.
[[nodiscard]] std::vector<gr::DataSet<float>> spectra() {
    namespace shim = gr::blocks::testing::span;
    WelchPsd<CF> welch({{"fft_size", gr::Size_t{kFftSize}}, {"n_averages", gr::Size_t{kAverages}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}, {"one_record_per_call", false}});
    init(welch);
    const std::vector<CF>                input = noise(kRecordsPerCall * kAverages * kFftSize + kFftSize);
    std::vector<gr::DataSet<float>>      records(kRecordsPerCall + 2UZ);
    shim::InputSpan<CF>                  inSpan{std::span<const CF>(input)};
    shim::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(records)};
    std::ignore = welch.processBulk(inSpan, outSpan);
    records.resize(outSpan.count);
    return records;
}

/// @brief Runs one `processBulk` call over the input records. Returns the first value of the first reading, or zero.
[[nodiscard]] double sweep(SpectralKurtosis& block, std::span<const gr::DataSet<float>> input, std::span<gr::DataSet<float>> readings) {
    namespace shim = gr::blocks::testing::span;
    shim::InputSpan<gr::DataSet<float>>  inSpan(input);
    shim::OutputSpan<gr::DataSet<float>> outSpan(readings);
    std::ignore = block.processBulk(inSpan, outSpan);
    return outSpan.count == 0UZ ? 0.0 : static_cast<double>(readings[0UZ].signal_values[0UZ]);
}

} // namespace

int main() {
    const std::vector<gr::DataSet<float>> input = spectra();
    if (input.size() != kRecordsPerCall) {
        std::println(stderr, "WelchPsd emitted {} records, and the arms need {}", input.size(), kRecordsPerCall);
        return 1;
    }
    std::vector<gr::DataSet<float>> readings(8UZ);

    SpectralKurtosis sk32({{"n_spectra", gr::Size_t{32U}}});
    SpectralKurtosis sk64({{"n_spectra", gr::Size_t{64U}}});
    SpectralKurtosis skNoRecords({{"n_spectra", gr::Size_t{32U}}, {"emit_records", false}});
    init(sk32);
    init(sk64);
    init(skNoRecords);

    std::vector<Arm> arms{
        {"SpectralKurtosis n_spectra 32, records", [&] { return sweep(sk32, std::span<const gr::DataSet<float>>(input), std::span<gr::DataSet<float>>(readings)); }},
        {"SpectralKurtosis n_spectra 64, records", [&] { return sweep(sk64, std::span<const gr::DataSet<float>>(input), std::span<gr::DataSet<float>>(readings)); }},
        {"SpectralKurtosis n_spectra 32, no records", [&] { return sweep(skNoRecords, std::span<const gr::DataSet<float>>(input), std::span<gr::DataSet<float>>(readings)); }},
    };

    std::println("ns per input bin: {} records of {} bins a call. A reading at n_spectra 32 covers {} bins.", kRecordsPerCall, kFftSize, 32UZ * kFftSize);
    gr::blocks::testing::bench::report(std::span<Arm>(arms), kRecordsPerCall * kFftSize, kRepeats);
}
