#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <print>
#include <span>
#include <tuple>
#include <type_traits>
#include <vector>

#include <gnuradio-4.0/measurement/SpectralEstimate.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::measurement::Spectrogram;
using gr::blocks::measurement::WelchPsd;
using gr::blocks::testing::bench::Arm;
using CF = std::complex<float>;

constexpr std::size_t kSamplesPerCall = 1UZ << 16;
constexpr std::size_t kMaxRecords     = 256UZ; // more than the shortest transform and hop produce in one call
constexpr std::size_t kRepeats        = 9UZ;
constexpr float       kSampleRate     = 1.0e6f;

/// Applies the settings of a block built in place and starts it.
template<typename TBlock>
void init(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

template<typename T>
[[nodiscard]] std::vector<T> tone() {
    std::vector<T>   data(kSamplesPerCall);
    constexpr double twoPi = 2. * std::numbers::pi;
    for (std::size_t k = 0UZ; k < data.size(); ++k) {
        const double phase = twoPi * 0.1231 * static_cast<double>(k);
        if constexpr (std::is_same_v<T, CF>) {
            data[k] = CF(static_cast<float>(0.7 * std::cos(phase)), static_cast<float>(0.7 * std::sin(phase)));
        } else {
            data[k] = static_cast<float>(0.7 * std::cos(phase));
        }
    }
    return data;
}

/// @brief Runs one `processBulk` call over the whole input. Returns the first value of the first record, or zero.
template<typename TBlock, typename T>
[[nodiscard]] double sweep(TBlock& block, std::span<const T> input, std::span<gr::DataSet<float>> records) {
    namespace shim = gr::blocks::testing::span;
    shim::InputSpan<T>                   inSpan(input);
    shim::OutputSpan<gr::DataSet<float>> outSpan(records);
    std::ignore = block.processBulk(inSpan, outSpan);
    return outSpan.count == 0UZ ? 0.0 : static_cast<double>(records[0UZ].signal_values[0UZ]);
}

/// @brief Runs the input through `block` in calls of at most one hop, as a scheduler does for a block with
/// `one_record_per_call` on, whose input span is capped to the hop. Returns the first value of the last record, or zero.
template<typename TBlock, typename T>
[[nodiscard]] double sweepCapped(TBlock& block, std::span<const T> input, std::span<gr::DataSet<float>> records) {
    namespace shim   = gr::blocks::testing::span;
    double      last = 0.0;
    std::size_t at   = 0UZ;
    while (at < input.size()) {
        shim::InputSpan<T>                   inSpan(input.subspan(at, std::min(block._core.hop, input.size() - at)));
        shim::OutputSpan<gr::DataSet<float>> outSpan(records);
        std::ignore = block.processBulk(inSpan, outSpan);
        if (outSpan.count > 0UZ) {
            last = static_cast<double>(records[outSpan.count - 1UZ].signal_values[0UZ]);
        }
        if (inSpan.consumed == 0UZ) {
            break;
        }
        at += inSpan.consumed;
    }
    return last;
}

/// @brief Windows, transforms and accumulates the input in power without building a record, through the calls the
/// block makes: one segment from the span after another. This arm is the baseline for the arms that emit records.
[[nodiscard]] double segmentsOnly(gr::blocks::measurement::detail::SegmentAccumulator<CF>& core, std::span<const CF> input) {
    std::size_t from = 0UZ;
    while (input.size() - from >= core.fftSize) {
        core.accumulateSegment({}, input.subspan(from));
        from += core.advanceGrid(input.size() - from);
        if (core.segments >= core.nAverages) {
            core.segments = 0UZ; // starts a new average in place and emits nothing
        }
    }
    return static_cast<double>(core.accumulator[0UZ]);
}

} // namespace

int main() {
    const std::vector<CF>    complexInput = tone<CF>();
    const std::vector<float> realInput    = tone<float>();

    std::vector<gr::DataSet<float>> records(kMaxRecords);

    gr::blocks::measurement::detail::SegmentAccumulator<CF> floorCore{gr::blocks::measurement::detail::SpectralSettings{.fftSize = 1024UZ, .hop = 512UZ, .nAverages = 16UZ, .windowShape = std::numeric_limits<float>::quiet_NaN(), .sampleRate = kSampleRate}};

    // Each arm times one work call over the whole input, and every record the call completes is emitted.
    WelchPsd<CF> welch1024({{"fft_size", gr::Size_t{1024U}}, {"n_averages", gr::Size_t{16U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}, {"one_record_per_call", false}});
    WelchPsd<CF> welchEvery({{"fft_size", gr::Size_t{1024U}}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}, {"one_record_per_call", false}});
    WelchPsd<CF> welch4096({{"fft_size", gr::Size_t{4096U}}, {"n_averages", gr::Size_t{16U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}, {"one_record_per_call", false}});
    WelchPsd<CF> welchGap1024({{"fft_size", gr::Size_t{1024U}}, {"n_averages", gr::Size_t{1U}}, {"hop", gr::Size_t{4096U}}, {"sample_rate", kSampleRate}, {"one_record_per_call", false}});
    WelchPsd<CF> welchGap4096({{"fft_size", gr::Size_t{4096U}}, {"n_averages", gr::Size_t{1U}}, {"hop", gr::Size_t{16384U}}, {"sample_rate", kSampleRate}, {"one_record_per_call", false}});
    WelchPsd<CF> welch8192({{"fft_size", gr::Size_t{8192U}}, {"n_averages", gr::Size_t{16U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}, {"one_record_per_call", false}});
    // The default setting: the span of a call is capped to a hop.
    WelchPsd<CF>    welchDefault1024({{"fft_size", gr::Size_t{1024U}}, {"n_averages", gr::Size_t{16U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}});
    WelchPsd<CF>    welchDefaultGap1024({{"fft_size", gr::Size_t{1024U}}, {"n_averages", gr::Size_t{1U}}, {"hop", gr::Size_t{4096U}}, {"sample_rate", kSampleRate}});
    WelchPsd<float> welchReal({{"fft_size", gr::Size_t{1024U}}, {"n_averages", gr::Size_t{16U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}, {"one_record_per_call", false}});
    Spectrogram<CF> spectrogram({{"fft_size", gr::Size_t{1024U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}, {"one_record_per_call", false}});
    init(welch1024);
    init(welchEvery);
    init(welch4096);
    init(welchGap1024);
    init(welchGap4096);
    init(welch8192);
    init(welchDefault1024);
    init(welchDefaultGap1024);
    init(welchReal);
    init(spectrogram);

    std::vector<Arm> arms{
        {"window + transform, no record, 1024", [&] { return segmentsOnly(floorCore, std::span<const CF>(complexInput)); }},
        {"WelchPsd<complex> 1024, 16 averages", [&] { return sweep(welch1024, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records)); }},
        {"WelchPsd<complex> 1024, 1 average", [&] { return sweep(welchEvery, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records)); }},
        {"WelchPsd<complex> 4096, 16 averages", [&] { return sweep(welch4096, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records)); }},
        {"WelchPsd<complex> 1024, hop 4096, 1 average", [&] { return sweep(welchGap1024, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records)); }},
        {"WelchPsd<complex> 4096, hop 16384, 1 average", [&] { return sweep(welchGap4096, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records)); }},
        {"WelchPsd<complex> 8192, 16 averages", [&] { return sweep(welch8192, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records)); }},
        {"WelchPsd<complex> 1024, 16 averages, default setting", [&] { return sweepCapped(welchDefault1024, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records)); }},
        {"WelchPsd<complex> 1024, hop 4096, 1 average, default setting", [&] { return sweepCapped(welchDefaultGap1024, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records)); }},
        {"WelchPsd<float> 1024, 16 averages", [&] { return sweep(welchReal, std::span<const float>(realInput), std::span<gr::DataSet<float>>(records)); }},
        {"Spectrogram<complex> 1024", [&] { return sweep(spectrogram, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records)); }},
    };

    std::println("ns per input sample. A 1024-point transform at half overlap starts a segment every 512 samples.");
    std::println("One average per record makes one record per 512 samples, and 16 averages make one per 8192.");
    gr::blocks::testing::bench::report(std::span<Arm>(arms), kSamplesPerCall, kRepeats);
}
