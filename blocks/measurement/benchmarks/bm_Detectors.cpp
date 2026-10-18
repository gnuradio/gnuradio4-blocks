#include <cmath>
#include <cstddef>
#include <cstdint>
#include <print>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/measurement/Detectors.hpp>
#include <gnuradio-4.0/measurement/OccupiedBandwidth.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::measurement::CfarDetect;
using gr::blocks::measurement::OccupiedBandwidth;
using gr::blocks::measurement::PeakDetect;
using gr::blocks::testing::bench::Arm;

constexpr std::size_t kBins           = 1024UZ;
constexpr std::size_t kRecordsPerCall = 64UZ;
constexpr std::size_t kRepeats        = 9UZ;
constexpr float       kSampleRate     = 1.0e6f;

/// Applies a block's settings and starts it. The detectors hold atomic counters and cannot move.
template<typename TBlock>
void init(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

/// @brief A density record of exponential noise, with a 400-bin band 20 dB over the floor and three narrow peaks in
/// the band.
[[nodiscard]] gr::DataSet<float> densityRecord(std::uint64_t& state) {
    const auto next = [&state] {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        const double uniform = (static_cast<double>(state >> 11U) + 0.5) / static_cast<double>(1ULL << 53U);
        return static_cast<float>(-std::log(uniform));
    };

    gr::DataSet<float> ds;
    ds.extents = {static_cast<std::int32_t>(kBins)};
    ds.layout  = gr::LayoutRight{};

    ds.axis_names = {"Frequency"};
    ds.axis_units = {"Hz"};
    ds.axis_values.resize(1UZ);
    ds.axis_values[0UZ].resize(kBins);
    const float binWidth = kSampleRate / static_cast<float>(kBins);
    for (std::size_t k = 0UZ; k < kBins; ++k) {
        ds.axis_values[0UZ][k] = static_cast<float>(k) * binWidth - 0.5f * kSampleRate;
    }

    ds.signal_names      = {"psd"};
    ds.signal_quantities = {"PowerSpectralDensity"};
    ds.signal_units      = {"1/Hz"};
    ds.signal_values.resize(kBins);
    for (std::size_t k = 0UZ; k < kBins; ++k) {
        const bool inBand   = k > 312UZ && k < 712UZ;
        ds.signal_values[k] = (inBand ? 1.f : 0.01f) * next();
    }
    for (const std::size_t at : {380UZ, 512UZ, 640UZ}) {
        for (std::size_t k = at - 2UZ; k <= at + 2UZ; ++k) {
            ds.signal_values[k] += static_cast<float>(30. * std::exp(-0.5 * static_cast<double>((k - at) * (k - at))));
        }
    }
    ds.signal_ranges.resize(1UZ);

    ds.meta_information.resize(1UZ);
    ds.meta_information[0UZ] = gr::property_map{{std::pmr::string("sample_rate"), gr::pmt::Value(kSampleRate)}, {std::pmr::string("sample_start"), gr::pmt::Value(std::uint64_t{0ULL})}};
    ds.timing_events.resize(1UZ);
    return ds;
}

/// @brief Runs one `processBulk` call of a records-in, records-out block over the whole record list.
template<typename TBlock>
[[nodiscard]] double sweep(TBlock& block, std::span<const gr::DataSet<float>> input, std::span<gr::DataSet<float>> output) {
    namespace shim = gr::blocks::testing::span;
    shim::InputSpan<gr::DataSet<float>>  inSpan(input);
    shim::OutputSpan<gr::DataSet<float>> outSpan(output);
    std::ignore = block.processBulk(inSpan, outSpan);
    return outSpan.count == 0UZ ? 0.0 : static_cast<double>(output[0UZ].signal_values[0UZ]);
}

} // namespace

int main() {
    std::uint64_t                   state = 0x243f6a8885a308d3ULL;
    std::vector<gr::DataSet<float>> input;
    input.reserve(kRecordsPerCall);
    for (std::size_t r = 0UZ; r < kRecordsPerCall; ++r) {
        input.push_back(densityRecord(state));
    }
    std::vector<gr::DataSet<float>> output(kRecordsPerCall);

    PeakDetect        peakMedian({{"threshold_db", 20.0}, {"reference", std::string("above_median")}});
    PeakDetect        peakAbsolute({{"threshold_db", 10.0}, {"reference", std::string("absolute")}});
    CfarDetect        cfarNarrow({{"n_train", gr::Size_t{16U}}, {"n_guard", gr::Size_t{2U}}, {"pfa", 1e-3}});
    CfarDetect        cfarWide({{"n_train", gr::Size_t{64U}}, {"n_guard", gr::Size_t{4U}}, {"pfa", 1e-3}});
    OccupiedBandwidth occupied({{"fraction", 0.99}});
    init(peakMedian);
    init(peakAbsolute);
    init(cfarNarrow);
    init(cfarWide);
    init(occupied);

    std::vector<Arm> arms{
        {"PeakDetect, above_median", [&] { return sweep(peakMedian, std::span<const gr::DataSet<float>>(input), std::span<gr::DataSet<float>>(output)); }},
        {"PeakDetect, absolute", [&] { return sweep(peakAbsolute, std::span<const gr::DataSet<float>>(input), std::span<gr::DataSet<float>>(output)); }},
        {"CfarDetect, 16 training cells", [&] { return sweep(cfarNarrow, std::span<const gr::DataSet<float>>(input), std::span<gr::DataSet<float>>(output)); }},
        {"CfarDetect, 64 training cells", [&] { return sweep(cfarWide, std::span<const gr::DataSet<float>>(input), std::span<gr::DataSet<float>>(output)); }},
        {"OccupiedBandwidth", [&] { return sweep(occupied, std::span<const gr::DataSet<float>>(input), std::span<gr::DataSet<float>>(output)); }},
    };

    std::println("ns per bin over {} records of {} bins each; a record costs {} times the figure.", kRecordsPerCall, kBins, kBins);
    gr::blocks::testing::bench::report(std::span<Arm>(arms), kRecordsPerCall * kBins, kRepeats);
}
