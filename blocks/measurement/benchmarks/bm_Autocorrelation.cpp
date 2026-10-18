#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <print>
#include <span>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

#include <gnuradio-4.0/measurement/Autocorrelation.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::measurement::Autocorrelation;
using gr::blocks::testing::bench::Arm;
using CF = std::complex<float>;

constexpr std::size_t kSamplesPerCall = 1UZ << 17;
constexpr std::size_t kMaxRecords     = 64UZ;
constexpr std::size_t kRepeats        = 5UZ;
constexpr float       kSampleRate     = 2.0e6f;

/// Applies a block's settings and starts it.
template<typename TBlock>
void init(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

template<typename T>
[[nodiscard]] std::vector<T> scene() {
    std::vector<T> data(kSamplesPerCall);
    std::uint64_t  state = 0x243f6a8885a308d3ULL;
    const auto     next  = [&state] {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return static_cast<double>(state >> 11U) / static_cast<double>(1ULL << 53U) - 0.5;
    };
    for (T& sample : data) {
        if constexpr (std::is_same_v<T, CF>) {
            sample = CF(static_cast<float>(next()), static_cast<float>(next()));
        } else {
            sample = static_cast<float>(next());
        }
    }
    return data;
}

/// @brief Runs one `processBulk` call over the input. Returns lag 1 of the first record, or 0 when no record
/// went out.
template<typename TBlock, typename T>
[[nodiscard]] double sweep(TBlock& block, std::span<const T> input, std::span<gr::DataSet<float>> records, std::span<gr::DataSet<float>> phase) {
    namespace shim = gr::blocks::testing::span;
    shim::InputSpan<T>                   inSpan(input);
    shim::OutputSpan<gr::DataSet<float>> outSpan(records);
    shim::OutputSpan<gr::DataSet<float>> phaseSpan(phase);
    std::ignore = block.processBulk(inSpan, outSpan, phaseSpan);
    return outSpan.count == 0UZ ? 0.0 : static_cast<double>(records[0UZ].signal_values[1UZ]);
}

/// @brief Runs the kernel alone over the same input, without a record. Returns R(0) of the last result.
template<typename T>
[[nodiscard]] double kernelOnly(gr::analysis::Autocorrelation<T>& kernel, std::span<const T> input) {
    double     last = 0.;
    const auto sink = [&last](const gr::analysis::AcfResult& result) { last = result.zeroLag; };
    kernel.reset();
    std::size_t at = 0UZ;
    while (at < input.size()) {
        const std::size_t used = kernel.process(input.subspan(at), sink);
        if (used == 0UZ) {
            break;
        }
        at += used;
    }
    return last;
}

} // namespace

int main() {
    const std::vector<CF>    complexInput = scene<CF>();
    const std::vector<float> realInput    = scene<float>();

    std::vector<gr::DataSet<float>> records(kMaxRecords);
    std::vector<gr::DataSet<float>> phase(kMaxRecords);

    gr::analysis::AcfConfig floorConfig{};
    floorConfig.windowLength = 8192UZ;
    floorConfig.maxLag       = 2048UZ;
    floorConfig.kind         = gr::analysis::AcfKind::Envelope;
    floorConfig.overlap      = 0.5;
    floorConfig.nAverages    = 16UZ;
    gr::analysis::Autocorrelation<CF> floorKernel;
    floorKernel.prepare(floorConfig);

    Autocorrelation<CF>    envelope8192({{"window_length", gr::Size_t{8192U}}, {"max_lag", gr::Size_t{2048U}}, {"kind", std::string("envelope")}, {"n_averages", gr::Size_t{16U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}});
    Autocorrelation<CF>    complex8192({{"window_length", gr::Size_t{8192U}}, {"max_lag", gr::Size_t{2048U}}, {"kind", std::string("complex")}, {"n_averages", gr::Size_t{16U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}});
    Autocorrelation<CF>    disjoint8192({{"window_length", gr::Size_t{8192U}}, {"max_lag", gr::Size_t{2048U}}, {"kind", std::string("envelope")}, {"n_averages", gr::Size_t{16U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}});
    Autocorrelation<CF>    envelope4096({{"window_length", gr::Size_t{4096U}}, {"max_lag", gr::Size_t{1024U}}, {"kind", std::string("envelope")}, {"n_averages", gr::Size_t{16U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}});
    Autocorrelation<CF>    everyWindow({{"window_length", gr::Size_t{8192U}}, {"max_lag", gr::Size_t{2048U}}, {"kind", std::string("envelope")}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}});
    Autocorrelation<float> real8192({{"window_length", gr::Size_t{8192U}}, {"max_lag", gr::Size_t{2048U}}, {"kind", std::string("complex")}, {"n_averages", gr::Size_t{16U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}});
    init(envelope8192);
    init(complex8192);
    init(disjoint8192);
    init(envelope4096);
    init(everyWindow);
    init(real8192);

    std::vector<Arm> arms{
        {"kernel only, no record, 8192/2048 envelope", [&] { return kernelOnly(floorKernel, std::span<const CF>(complexInput)); }},
        {"Autocorrelation<complex> 8192/2048 envelope, 16 averages", [&] { return sweep(envelope8192, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records), std::span<gr::DataSet<float>>(phase)); }},
        {"Autocorrelation<complex> 8192/2048 complex, 16 averages", [&] { return sweep(complex8192, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records), std::span<gr::DataSet<float>>(phase)); }},
        {"Autocorrelation<complex> 8192/2048 envelope, no overlap", [&] { return sweep(disjoint8192, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records), std::span<gr::DataSet<float>>(phase)); }},
        {"Autocorrelation<complex> 4096/1024 envelope, 16 averages", [&] { return sweep(envelope4096, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records), std::span<gr::DataSet<float>>(phase)); }},
        {"Autocorrelation<complex> 8192/2048 envelope, 1 average", [&] { return sweep(everyWindow, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records), std::span<gr::DataSet<float>>(phase)); }},
        {"Autocorrelation<float>   8192/2048 complex, 16 averages", [&] { return sweep(real8192, std::span<const float>(realInput), std::span<gr::DataSet<float>>(records), std::span<gr::DataSet<float>>(phase)); }},
    };

    std::println("ns per input sample. One window costs one transform pair of length M, the next power of two at or");
    std::println("above N + L. At N=8192 and L=2048 that is one 16384-point pair every hop, every 4096 input samples at");
    std::println("half overlap and every 8192 with none. n_averages changes the record rate and not the transform rate.");
    gr::blocks::testing::bench::report(std::span<Arm>(arms), kSamplesPerCall, kRepeats);
}
