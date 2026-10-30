#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/measurement/Detectors.hpp>
#include <gnuradio-4.0/measurement/OccupiedBandwidth.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_detect {

using gr::blocks::measurement::CfarDetect;
using gr::blocks::measurement::OccupiedBandwidth;
using gr::blocks::measurement::PeakDetect;

constexpr float       kSampleRate = 48000.f;
constexpr std::size_t kBins       = 1024UZ;

/// Emits fixed records, then ends the stream.
struct RecordSource : gr::Block<RecordSource> {
    gr::PortOut<gr::DataSet<float>, gr::Async> out;

    std::vector<gr::DataSet<float>> records{};
    std::size_t                     at = 0UZ;

    GR_MAKE_REFLECTABLE(RecordSource, out);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (at >= records.size()) {
            outSpan.publish(0UZ);
            this->requestStop();
            return gr::work::Status::DONE;
        }
        const std::size_t take = std::min(records.size() - at, outSpan.size());
        for (std::size_t k = 0UZ; k < take; ++k) {
            outSpan[k] = records[at + k];
        }
        at += take;
        outSpan.publish(take);
        return gr::work::Status::OK;
    }
};

struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<gr::DataSet<float>, gr::Async> in;

    std::vector<gr::DataSet<float>> records{};

    GR_MAKE_REFLECTABLE(RecordSink, in);

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        records.insert(records.end(), inSpan.begin(), inSpan.end());
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/// @brief A density record on a two-sided axis, with the values the caller supplies.
[[nodiscard]] gr::DataSet<float> densityRecord(std::vector<float> values, float sampleRate = kSampleRate) {
    gr::DataSet<float> ds;
    const std::size_t  n = values.size();
    ds.extents           = {static_cast<std::int32_t>(n)};
    ds.layout            = gr::LayoutRight{};
    ds.axis_names        = {"Frequency"};
    ds.axis_units        = {"Hz"};
    ds.axis_values.resize(1UZ);
    ds.axis_values[0UZ].resize(n);
    const float binWidth = sampleRate / static_cast<float>(n);
    const float offset   = static_cast<float>(n / 2UZ) * binWidth;
    for (std::size_t k = 0UZ; k < n; ++k) {
        ds.axis_values[0UZ][k] = static_cast<float>(k) * binWidth - offset;
    }
    ds.signal_names      = {"psd"};
    ds.signal_quantities = {"PowerSpectralDensity"};
    ds.signal_units      = {"1/Hz"};
    ds.signal_values     = std::move(values);
    ds.signal_ranges.resize(1UZ);
    ds.meta_information.resize(1UZ);
    ds.meta_information[0UZ] = gr::property_map{{std::pmr::string("sample_rate"), gr::pmt::Value(sampleRate)}, {std::pmr::string("sample_start"), gr::pmt::Value(std::uint64_t{7ULL})}};
    ds.timing_events.resize(1UZ);
    return ds;
}

/// @brief Runs a record source through `TBlock` into a sink. Returns the records the sink took and, for a detector,
/// its counts of emitted records and empty results after the run.
template<typename TBlock>
[[nodiscard]] std::pair<std::vector<gr::DataSet<float>>, std::pair<std::uint64_t, std::uint64_t>> collectCounted(gr::property_map settings, std::vector<gr::DataSet<float>> records) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<RecordSource>();
    auto&     block  = graph.emplaceBlock<TBlock>(std::move(settings));
    auto&     sink   = graph.emplaceBlock<RecordSink>();
    source.records   = std::move(records);

    if (!graph.connect<"out", "in">(source, block).has_value() || !graph.connect<"out", "in">(block, sink).has_value()) {
        return {};
    }
    gr::scheduler::Simple scheduler;
    if (!scheduler.exchange(std::move(graph)).has_value()) {
        return {};
    }
    std::ignore = scheduler.runAndWait(); // the source ends the stream

    std::pair<std::uint64_t, std::uint64_t> counts{};
    if constexpr (requires { block.nEmptyResults(); }) {
        counts = {block.nRecords(), block.nEmptyResults()};
    }
    return {std::move(sink.records), counts};
}

/// @brief Runs a record source through `TBlock` into a sink and returns the records the sink took.
template<typename TBlock>
[[nodiscard]] std::vector<gr::DataSet<float>> collect(gr::property_map settings, std::vector<gr::DataSet<float>> records) {
    return collectCounted<TBlock>(std::move(settings), std::move(records)).first;
}

/// @brief Column `c` of a detection record. The three columns lie end to end.
[[nodiscard]] std::span<const float> column(const gr::DataSet<float>& record, std::size_t c) {
    const std::size_t n = record.signal_values.size() / 3UZ;
    return std::span<const float>(record.signal_values).subspan(c * n, n);
}

/// @brief Repeats the five admission checks of `DataSetToStream`, in its order, and returns the reason for a rejection
/// or nullptr.
[[nodiscard]] const char* admissionFailure(const gr::DataSet<float>& ds, std::size_t signalIndex = 0UZ) {
    if (ds.signal_names.empty()) {
        return "no_signals";
    }
    if (signalIndex >= ds.signal_names.size()) {
        return "signal_index_out_of_range";
    }
    if (ds.extents.size() != 1UZ) {
        return "not_one_dimensional";
    }
    if (ds.extents[0UZ] <= 0) {
        return "empty_or_negative_extent";
    }
    if (ds.signal_values.size() != ds.signal_names.size() * static_cast<std::size_t>(ds.extents[0UZ])) {
        return "inconsistent_extent";
    }
    return nullptr;
}

/// @brief Initializes a block's settings and starts it.
template<typename TBlock>
void startBlock(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

/// @brief Applies `changes` in one batch through `settings().set()` and a context activation.
template<typename TBlock>
void applyLive(TBlock& block, gr::property_map changes) {
    std::ignore = block.settings().set(std::move(changes));
    std::ignore = block.settings().activateContext();
    std::ignore = block.settings().applyStagedParameters();
}

/// @brief Stages `changes` and applies them. Returns the settings the limits refused.
template<typename TBlock>
[[nodiscard]] gr::property_map refusedByLimits(TBlock& block, gr::property_map changes) {
    std::ignore = block.settings().setStaged(std::move(changes));
    return block.settings().applyStagedParameters().failedParameters;
}

/// @brief One work call on `records` with room for `room` records. Returns the status, the records consumed and the
/// records published.
template<typename TBlock>
[[nodiscard]] std::tuple<gr::work::Status, std::size_t, std::vector<gr::DataSet<float>>> workCall(TBlock& block, const std::vector<gr::DataSet<float>>& records, std::size_t room) {
    std::vector<gr::DataSet<float>>                           out(room);
    gr::blocks::testing::span::InputSpan<gr::DataSet<float>>  in{std::span<const gr::DataSet<float>>(records)};
    gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> os{std::span<gr::DataSet<float>>(out)};
    const gr::work::Status                                    status = block.processBulk(in, os);
    out.resize(os.count);
    return {status, in.consumed, std::move(out)};
}

/// @brief A density record with three Gaussian lobes, at bins 200, 512 and 800, over a floor of 1e-6.
[[nodiscard]] std::vector<float> threeLobes() {
    std::vector<float> values(kBins, 1e-6f);
    for (const std::size_t at : {200UZ, 512UZ, 800UZ}) {
        for (std::size_t k = at - 3UZ; k <= at + 3UZ; ++k) {
            const double d = static_cast<double>(k) - static_cast<double>(at);
            values[k] += static_cast<float>(std::exp(-0.5 * d * d)) * (at == 512UZ ? 0.5f : 1.f);
        }
    }
    return values;
}

/// @brief Seeded, exponentially distributed power of unit mean: the distribution of a noise bin's squared magnitude.
struct ExponentialNoise {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] float operator()() {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        const double uniform = (static_cast<double>(state >> 11U) + 0.5) / static_cast<double>(1ULL << 53U);
        return static_cast<float>(-std::log(uniform));
    }
};

} // namespace qa_detect

const boost::ut::suite<"Detectors"> detectorTests = [] {
    using namespace boost::ut;
    using namespace qa_detect;

    "the parabolic refinement is exact on a lobe that is a parabola in decibels"_test = [] {
        // A Gaussian lobe is an exact parabola in decibels. The case checks the arithmetic of the fit. The bias of a
        // windowed tone is measured in qa_SpectralEstimate.
        const auto lobeAt = [](double centerBin) {
            std::vector<float> values(kBins, 1e-9f);
            for (std::size_t k = 0UZ; k < kBins; ++k) {
                const double d = (static_cast<double>(k) - centerBin) / 1.5; // a main lobe about three bins wide
                values[k] += static_cast<float>(std::exp(-d * d));
            }
            return values;
        };
        const float binWidth = kSampleRate / static_cast<float>(kBins);

        for (const double offset : {0.0, 0.3, -0.3, 0.5}) {
            const double center  = 300. + offset;
            const auto   records = collect<PeakDetect>({{"threshold_db", 20.0}, {"reference", std::string("above_median")}}, {densityRecord(lobeAt(center))});
            expect(eq(records.size(), 1UZ));
            if (records.empty()) {
                continue;
            }
            const auto frequencies = column(records.front(), 0UZ);
            expect(eq(frequencies.size(), 1UZ)) << "one lobe is one detection, at offset " << offset;
            if (frequencies.size() != 1UZ) {
                continue;
            }
            const float expected  = (static_cast<float>(center) - static_cast<float>(kBins / 2UZ)) * binWidth;
            const float errorBins = std::abs(frequencies[0UZ] - expected) / binWidth;
            if (offset == 0.0) {
                expect(errorBins < 1e-4f) << "at a bin center the refinement adds nothing, measured " << errorBins << " bins";
            } else {
                expect(errorBins < 1e-3f) << "off-center error at offset " << offset << " measured " << errorBins << " bins";
            }
        }
    };

    "a peak within min_distance_hz of a stronger one is dropped"_test = [] {
        std::vector<float> values(kBins, 1e-9f);
        values[400UZ]        = 1.0f;
        values[401UZ]        = 0.25f;
        values[402UZ]        = 0.25f;
        values[403UZ]        = 0.5f; // a weaker maximum three bins from the one at bin 400
        values[600UZ]        = 0.8f; // a third maximum 200 bins away
        const float binWidth = kSampleRate / static_cast<float>(kBins);

        const auto loose = collect<PeakDetect>({{"threshold_db", 20.0}, {"min_distance_hz", 0.0}}, {densityRecord(values)});
        const auto tight = collect<PeakDetect>({{"threshold_db", 20.0}, {"min_distance_hz", static_cast<double>(10.f * binWidth)}}, {densityRecord(values)});
        expect(!loose.empty() && !tight.empty()) << fatal;
        expect(eq(column(loose.front(), 0UZ).size(), 3UZ)) << "without a distance the three maxima are reported";
        expect(eq(column(tight.front(), 0UZ).size(), 2UZ)) << "the maximum three bins from a stronger one is dropped and the far one is kept";
    };

    "max_peaks keeps the strongest, listed in order of frequency"_test = [] {
        const auto records = collect<PeakDetect>({{"threshold_db", 20.0}, {"max_peaks", gr::Size_t{2U}}}, {densityRecord(threeLobes())});
        expect(eq(records.size(), 1UZ)) << fatal;
        const auto  frequencies = column(records.front(), 0UZ);
        const float binWidth    = kSampleRate / static_cast<float>(kBins);
        expect(eq(frequencies.size(), 2UZ)) << fatal;
        expect(std::abs(frequencies[0UZ] - (200.f - 512.f) * binWidth) < 0.01f * binWidth) << "the lobe at bin 200 is kept";
        expect(std::abs(frequencies[1UZ] - (800.f - 512.f) * binWidth) < 0.01f * binWidth) << "and the lobe at bin 800, and the weaker one at bin 512 is dropped";
    };

    "with the absolute reference the threshold is a level in dBFS per hertz"_test = [] {
        // The lobes peak at 0 dB and at -3.01 dB over a floor of -60 dB.
        const auto countAt = [](double thresholdDb) {
            const auto records = collect<PeakDetect>({{"threshold_db", thresholdDb}, {"reference", std::string("absolute")}}, {densityRecord(threeLobes())});
            return records.empty() ? 0UZ : column(records.front(), 0UZ).size();
        };
        expect(eq(countAt(-10.0), 3UZ)) << "a level of -10 dB passes all three lobes";
        expect(eq(countAt(-2.0), 2UZ)) << "a level of -2 dB passes the two lobes that peak at 0 dB";
        expect(eq(countAt(1.0), 0UZ)) << "and a level of +1 dB passes none";
    };

    "the width column is the half-power width of the peak"_test = [] {
        // Around bin 500: 0.25, 0.75, 1, 0.75, 0.25. Half power is crossed half way between 0.75 and 0.25 on each side,
        // 1.5 bins from the center.
        std::vector<float> values(kBins, 1e-9f);
        values[498UZ]        = 0.25f;
        values[499UZ]        = 0.75f;
        values[500UZ]        = 1.f;
        values[501UZ]        = 0.75f;
        values[502UZ]        = 0.25f;
        const float binWidth = kSampleRate / static_cast<float>(kBins);

        const auto peaks = collect<PeakDetect>({{"threshold_db", 20.0}}, {densityRecord(values)});
        expect(eq(peaks.size(), 1UZ)) << fatal;
        expect(eq(column(peaks.front(), 2UZ).size(), 1UZ)) << fatal;
        expect(std::abs(column(peaks.front(), 2UZ)[0UZ] - 3.f * binWidth) < 1e-3f * binWidth) << "the width is three bins, measured " << column(peaks.front(), 2UZ)[0UZ] / binWidth;
        expect(std::abs(column(peaks.front(), 1UZ)[0UZ]) < 1e-4f) << "and the level column holds the peak's 0 dB";
    };

    "a record of several signals is read as its first signal, and a malformed record is dropped and counted"_test = [] {
        // Four signals on one axis of 256 bins. The first signal holds a peak at bin 100, and the third a stronger one at
        // bin 200.
        constexpr std::size_t kExtent = 256UZ;
        gr::DataSet<float>    four    = densityRecord(std::vector<float>(kExtent, 1e-6f));
        four.signal_names             = {"Magnitude", "Phase", "Re", "Im"};
        four.signal_values.assign(4UZ * kExtent, 1e-6f);
        four.signal_values[100UZ]                 = 1.f;
        four.signal_values[2UZ * kExtent + 200UZ] = 10.f;
        const float expected                      = (100.f - static_cast<float>(kExtent / 2UZ)) * kSampleRate / static_cast<float>(kExtent);

        const auto [peaks, peakCounts] = collectCounted<PeakDetect>({{"threshold_db", 20.0}}, {four});
        expect(eq(peaks.size(), 1UZ)) << fatal;
        expect(eq(column(peaks.front(), 0UZ).size(), 1UZ)) << "only the first signal is searched";
        expect(std::abs(column(peaks.front(), 0UZ)[0UZ] - expected) < 1.f) << "and its peak is read on the record's axis";

        const auto [cells, cellCounts] = collectCounted<CfarDetect>({{"n_train", gr::Size_t{16U}}, {"n_guard", gr::Size_t{2U}}, {"pfa", 1e-3}}, {four});
        expect(eq(cells.size(), 1UZ)) << fatal;
        expect(eq(column(cells.front(), 0UZ).size(), 1UZ)) << "the CFAR detector tests the first signal only";
        expect(std::abs(column(cells.front(), 0UZ)[0UZ] - expected) < 1.f);

        // The first signal holds a band of 56 bins and the third a band of 200. OccupiedBandwidth reads the first.
        gr::DataSet<float> bands = densityRecord(std::vector<float>(kExtent, 0.f));
        bands.signal_names       = {"Magnitude", "Phase", "Re", "Im"};
        bands.signal_values.assign(4UZ * kExtent, 0.f);
        std::fill_n(bands.signal_values.begin() + 100, 56, 1.f);
        std::fill_n(bands.signal_values.begin() + static_cast<std::ptrdiff_t>(2UZ * kExtent) + 28, 200, 1.f);
        const auto alone  = collect<OccupiedBandwidth>({{"fraction", 1.0}}, {densityRecord(std::vector<float>(bands.signal_values.begin(), bands.signal_values.begin() + static_cast<std::ptrdiff_t>(kExtent)))});
        const auto ofFour = collect<OccupiedBandwidth>({{"fraction", 1.0}}, {bands});
        expect(eq(alone.size(), 1UZ) && eq(ofFour.size(), 1UZ)) << fatal;
        expect(alone.front().signal_values[0UZ] > 0.f) << "the first signal alone reads its band";
        expect(ofFour.front().signal_values == alone.front().signal_values) << "and the record of four signals reads the same band and edges";

        std::vector<gr::DataSet<float>> malformed(4UZ, densityRecord(std::vector<float>(kExtent, 1e-6f)));
        malformed[0UZ].axis_values[0UZ].pop_back(); // an axis shorter than the extent
        malformed[1UZ].signal_values.pop_back();    // values that do not fill the signal
        malformed[2UZ].extents = {0};
        malformed[3UZ].extents = {16, 16};
        PeakDetect peak({{"threshold_db", 20.0}});
        CfarDetect cfar({{"n_train", gr::Size_t{16U}}});
        startBlock(peak);
        startBlock(cfar);
        const auto [peakStatus, peakTaken, peakMade] = workCall(peak, malformed, 4UZ);
        const auto [cfarStatus, cfarTaken, cfarMade] = workCall(cfar, malformed, 4UZ);
        expect(eq(peakTaken, 4UZ) && eq(cfarTaken, 4UZ)) << "the malformed records are consumed";
        expect(peakMade.empty() && cfarMade.empty()) << "and give no record";
        expect(eq(peak.nMalformed(), std::uint64_t{4ULL}) && eq(cfar.nMalformed(), std::uint64_t{4ULL})) << "each is counted as malformed";
        expect(eq(peak.nEmptyResults(), std::uint64_t{0ULL}) && eq(cfar.nEmptyResults(), std::uint64_t{0ULL})) << "and not as an empty result";

        OccupiedBandwidth bandwidth({{"fraction", 0.99}});
        startBlock(bandwidth);
        const auto [bandStatus, bandTaken, bandMade] = workCall(bandwidth, malformed, 4UZ);
        expect(eq(bandTaken, 4UZ) && bandMade.empty()) << "OccupiedBandwidth consumes the malformed records and gives no record";
        expect(eq(bandwidth.nMalformed(), std::uint64_t{4ULL})) << "it counts each as malformed";
        expect(eq(bandwidth.nRecords(), std::uint64_t{0ULL}) && eq(bandwidth.bandwidth(), 0.)) << "and publishes no reading";
    };

    // The signal count times the extent is 2^32 values, a product that is zero in a 32-bit size.
    "a record whose value count is not its signal count times its extent is dropped, also when that product wraps"_test = [] {
        constexpr std::size_t           kExtent = 4096UZ;
        constexpr std::uint64_t         kNames  = (std::uint64_t{1} << 32U) / std::uint64_t{kExtent};
        std::vector<gr::DataSet<float>> records(2UZ, densityRecord(std::vector<float>(kExtent, 1e-6f)));
        records[0UZ].signal_names.assign(static_cast<std::size_t>(kNames), std::string{});
        records[0UZ].signal_values.clear();
        records[1UZ].signal_values.push_back(1e-6f); // one value more than the signal holds

        const auto check = [&records](auto& block, std::string_view name) {
            startBlock(block);
            const auto [status, taken, made] = workCall(block, records, 4UZ);
            expect(eq(taken, records.size()) && made.empty()) << name << ": both records are consumed and give no record";
            expect(eq(block.nMalformed(), std::uint64_t{records.size()})) << name << ": each is counted as malformed";
        };
        PeakDetect        peak({{"threshold_db", 20.0}});
        CfarDetect        cfar({{"n_train", gr::Size_t{16U}}});
        OccupiedBandwidth bandwidth({{"fraction", 0.99}});
        check(peak, "PeakDetect");
        check(cfar, "CfarDetect");
        check(bandwidth, "OccupiedBandwidth");
    };

    "a call that moves nothing says what it lacks"_test = [] {
        std::vector<float> values(kBins, 1e-6f);
        values[300UZ]    = 1.f;
        const auto check = [&values](auto& block, std::string_view name) {
            startBlock(block);
            const auto [idle, idleTaken, idleMade] = workCall(block, {}, 4UZ);
            expect(idle == gr::work::Status::INSUFFICIENT_INPUT_ITEMS) << name << ": an empty input is a lack of input";
            const auto [full, fullTaken, fullMade] = workCall(block, {densityRecord(values)}, 0UZ);
            expect(full == gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS) << name << ": a full output is a lack of room";
            expect(eq(fullTaken, 0UZ)) << name << ": and the record stays in the input";
            const auto [fed, fedTaken, fedMade] = workCall(block, {densityRecord(values)}, 4UZ);
            expect(fed == gr::work::Status::OK && fedTaken == 1UZ) << name << ": a record with room is progress";
        };
        PeakDetect        peak({{"threshold_db", 20.0}});
        CfarDetect        cfar({{"n_train", gr::Size_t{16U}}});
        OccupiedBandwidth bandwidth({{"fraction", 0.99}});
        check(peak, "PeakDetect");
        check(cfar, "CfarDetect");
        check(bandwidth, "OccupiedBandwidth");
    };

    "a refused setting is written back and changes nothing the detectors compute, and the limits refuse a value outside them alone"_test = [] {
        const std::vector<gr::DataSet<float>> input{densityRecord(threeLobes())};

        // A block that computed on the refused values would report no detection, or one. The later change drops the lobe at
        // bin 512, which lies within 20 kHz of the lobe at bin 200, and keeps the lobes at bins 200 and 800.
        PeakDetect peakReference({{"threshold_db", 20.0}});
        PeakDetect peakRefused({{"threshold_db", 20.0}});
        startBlock(peakReference);
        startBlock(peakRefused);
        const std::string acceptedReference = peakRefused.reference.value;
        const gr::Size_t  acceptedMaxPeaks  = peakRefused.max_peaks.value;
        expect(throws([&] { applyLive(peakRefused, {{"reference", std::string("bogus")}, {"threshold_db", 70.0}, {"max_peaks", gr::Size_t{1U}}}); })) << "an unknown reference is refused";
        expect(eq(peakRefused.reference.value, acceptedReference) && eq(peakRefused.threshold_db.value, 20.0) && eq(peakRefused.max_peaks.value, acceptedMaxPeaks)) << "and the accepted values are written back to the members";
        expect(nothrow([&] { applyLive(peakRefused, {{"min_distance_hz", 20000.0}}); })) << "a later change of another setting is applied";
        expect(throws([&] { applyLive(peakRefused, {{"reference", std::string("bogus")}}); })) << "the refused value sent again alone is refused again";
        applyLive(peakReference, {{"min_distance_hz", 20000.0}});
        const auto [peakStatus, peakTaken, peaks]      = workCall(peakRefused, input, 4UZ);
        const auto [refStatus, refTaken, peakExpected] = workCall(peakReference, input, 4UZ);
        expect(eq(peaks.size(), 1UZ) && eq(peakExpected.size(), 1UZ)) << fatal;
        expect(eq(column(peaks.front(), 0UZ).size(), 2UZ)) << "the later change is in force and the refused values are not";
        expect(peaks.front().signal_values == peakExpected.front().signal_values) << "PeakDetect reports what a block given the later change alone reports";

        // Exponential noise with a tone 30 dB over its mean at bin 300. At 20 dB over the median only the tone is a
        // detection, and every local maximum of the noise passes a NaN threshold.
        ExponentialNoise   noise;
        std::vector<float> noisy(kBins);
        for (float& bin : noisy) {
            bin = noise();
        }
        noisy[300UZ] = 1000.f;
        const std::vector<gr::DataSet<float>> noisyInput{densityRecord(std::move(noisy))};
        for (const double threshold : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
            PeakDetect peak({{"threshold_db", 20.0}});
            startBlock(peak);
            expect(throws([&] { std::ignore = peak.settings().set({{"threshold_db", threshold}}); })) << "settings().set() refuses a threshold of " << threshold << " dB at the call";
            expect(refusedByLimits(peak, {{"threshold_db", threshold}}).contains("threshold_db")) << "the limits refuse a staged threshold of " << threshold << " dB";
            expect(eq(peak.threshold_db.value, 20.0)) << "and the member keeps the accepted threshold";
            const auto [status, taken, found] = workCall(peak, noisyInput, 4UZ);
            expect(eq(found.size(), 1UZ) && (found.empty() || eq(column(found.front(), 0UZ).size(), 1UZ))) << "the tone alone stays detected at the accepted 20 dB, threshold " << threshold;
        }

        CfarDetect cfarReference({{"n_train", gr::Size_t{16U}}, {"pfa", 1e-3}});
        CfarDetect cfarRefused({{"n_train", gr::Size_t{16U}}, {"pfa", 1e-3}});
        startBlock(cfarReference);
        startBlock(cfarRefused);
        const gr::property_map cfarLimits = refusedByLimits(cfarRefused, {{"pfa", 1.5}, {"n_guard", gr::Size_t{6U}}});
        expect(eq(cfarLimits.size(), 1UZ) && cfarLimits.contains("pfa")) << "the limits refuse a probability above one alone";
        expect(eq(cfarRefused.pfa.value, 1e-3)) << "and pfa keeps the accepted value";
        for (const double bound : {0.0, 1.0}) {
            expect(refusedByLimits(cfarRefused, {{"pfa", bound}}).contains("pfa")) << "the limits refuse a probability of " << bound;
        }
        expect(eq(cfarRefused.pfa.value, 1e-3)) << "and pfa keeps the accepted value at either bound";
        expect(refusedByLimits(cfarRefused, {{"n_train", gr::Size_t{0U}}}).contains("n_train")) << "the limits refuse an n_train of zero";
        expect(nothrow([&] { applyLive(cfarRefused, {{"n_train", gr::Size_t{8U}}}); })) << "a later change of another setting is applied";
        applyLive(cfarReference, {{"n_guard", gr::Size_t{6U}}, {"n_train", gr::Size_t{8U}}});
        expect(eq(cfarRefused.alpha(), cfarReference.alpha()) && eq(cfarRefused.alpha(), CfarDetect::alphaFor(8UZ, 1e-3))) << "the multiplier follows the new n_train and the accepted pfa";
        expect(eq(cfarRefused.testableCells(kBins), kBins - 28UZ)) << "and the margin takes the n_guard of the refused application";
        const auto [cfarStatus, cfarTaken, cells]               = workCall(cfarRefused, input, 4UZ);
        const auto [cfarRefStatus, cfarRefTaken, cellsExpected] = workCall(cfarReference, input, 4UZ);
        expect(eq(cells.size(), 1UZ) && eq(cellsExpected.size(), 1UZ)) << fatal;
        expect(cells.front().signal_values == cellsExpected.front().signal_values) << "CfarDetect reports what a block given the later change alone reports";

        OccupiedBandwidth bandReference({{"fraction", 0.99}});
        OccupiedBandwidth bandRefused({{"fraction", 0.99}});
        startBlock(bandReference);
        startBlock(bandRefused);
        for (const double outside : {0.0, 1.5, std::numeric_limits<double>::quiet_NaN()}) {
            expect(refusedByLimits(bandRefused, {{"fraction", outside}}).contains("fraction")) << "the limits refuse a fraction of " << outside;
        }
        expect(eq(bandRefused.fraction.value, 0.99)) << "and the member keeps the accepted fraction";
        std::ignore = workCall(bandRefused, input, 4UZ);
        std::ignore = workCall(bandReference, input, 4UZ);
        expect(eq(bandRefused.bandwidth(), bandReference.bandwidth())) << "OccupiedBandwidth measures with the accepted fraction";
        expect(eq(bandRefused.lowerEdge(), bandReference.lowerEdge()) && eq(bandRefused.upperEdge(), bandReference.upperEdge()));
    };

    "a non-finite bin makes the readings of its record NaN, and the next record reads clean"_test = [] {
        std::vector<float> clean(kBins, 0.f);
        std::fill(clean.begin() + 412, clean.begin() + 612, 1.f);
        for (const float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
            std::vector<float> values = clean;
            values[10UZ]              = bad; // outside the band
            OccupiedBandwidth block({{"fraction", 0.99}});
            startBlock(block);

            const auto [status, taken, out] = workCall(block, {densityRecord(values)}, 1UZ);
            expect(eq(out.size(), 1UZ)) << fatal;
            expect(std::isnan(block.bandwidth()) && std::isnan(block.lowerEdge()) && std::isnan(block.upperEdge())) << "a bin of " << bad << " gives NaN readings, distinct from a band of zero";
            expect(std::ranges::all_of(out.front().signal_values, [](float v) { return std::isnan(v); })) << "and so does the record";
            expect(eq(block.nRecords(), std::uint64_t{1ULL}));

            std::ignore = workCall(block, {densityRecord(clean)}, 1UZ);
            expect(block.bandwidth() > 0. && std::isfinite(block.bandwidth())) << "the next record reads its band";
            expect(eq(block.nRecords(), std::uint64_t{2ULL}));
        }
    };

    "on records that average 16 segments the CFAR false-alarm rate falls below pfa"_test = [] {
        // Each bin is the mean of 16 exponential draws, as in a 16-segment average of white noise. The threshold is
        // designed for single exponential draws.
        constexpr double      kPfa     = 1e-3;
        constexpr std::size_t kRecords = 256UZ;
        ExponentialNoise      noise;

        std::vector<gr::DataSet<float>> records;
        records.reserve(kRecords);
        for (std::size_t r = 0UZ; r < kRecords; ++r) {
            std::vector<float> values(kBins);
            for (float& value : values) {
                float sum = 0.f;
                for (std::size_t m = 0UZ; m < 16UZ; ++m) {
                    sum += noise();
                }
                value = sum / 16.f;
            }
            records.push_back(densityRecord(std::move(values)));
        }
        const auto [out, counts] = collectCounted<CfarDetect>({{"n_train", gr::Size_t{16U}}, {"n_guard", gr::Size_t{2U}}, {"pfa", kPfa}}, std::move(records));
        std::size_t detections   = 0UZ;
        for (const auto& record : out) {
            detections += record.signal_values.size() / 3UZ;
        }
        const double measured = static_cast<double>(detections) / static_cast<double>(kRecords * (kBins - 36UZ));
        std::println("CFAR on 16-segment averages: {} detections in {} cells, rate {:.3e} against the design {:.1e}", detections, kRecords * (kBins - 36UZ), measured, kPfa);
        expect(measured < 0.1 * kPfa) << "the averaged bins cross the single-periodogram threshold far less often than pfa";
    };

    "an input record with no detection emits no record and is counted"_test = [] {
        // An empty result is counted instead of emitted.
        std::vector<gr::DataSet<float>> quiet;
        for (std::size_t r = 0UZ; r < 3UZ; ++r) {
            quiet.push_back(densityRecord(std::vector<float>(kBins, 1.f)));
        }
        const auto [records, counts] = collectCounted<PeakDetect>({{"threshold_db", 60.0}}, std::move(quiet));
        expect(records.empty()) << "no maximum passed the threshold, and no record was emitted";
        expect(eq(counts.first, std::uint64_t{0ULL})) << "no record is counted as emitted";
        expect(eq(counts.second, std::uint64_t{3ULL})) << "the three inputs are counted as empty results";

        std::vector<float> withPeak(kBins, 1e-9f);
        withPeak[500UZ]                = 1.f;
        const auto [found, foundCount] = collectCounted<PeakDetect>({{"threshold_db", 20.0}}, {densityRecord(std::move(withPeak))});
        expect(eq(found.size(), 1UZ)) << "an input with a detection emits one record";
        expect(eq(foundCount.first, std::uint64_t{1ULL}));
        expect(eq(foundCount.second, std::uint64_t{0ULL}));
    };

    "the CFAR detector counts its empty results"_test = [] {
        // one record of seeded noise at a design probability of 1e-9
        ExponentialNoise   noise;
        std::vector<float> values(kBins);
        std::ranges::generate(values, [&noise] { return noise(); });

        const auto [records, counts] = collectCounted<CfarDetect>({{"n_train", gr::Size_t{16U}}, {"n_guard", gr::Size_t{2U}}, {"pfa", 1e-9}}, {densityRecord(std::move(values))});
        expect(records.empty()) << "no cell crossed the threshold";
        expect(eq(counts.first, std::uint64_t{0ULL}));
        expect(eq(counts.second, std::uint64_t{1ULL}));
    };

    "the measured false-alarm rate matches the design, on noise alone"_test = [] {
        constexpr double      kPfa     = 1e-3;
        constexpr std::size_t kRecords = 1024UZ; // about 10^6 tested cells at the margin below
        ExponentialNoise      noise;

        std::vector<gr::DataSet<float>> records;
        records.reserve(kRecords);
        for (std::size_t r = 0UZ; r < kRecords; ++r) {
            std::vector<float> values(kBins);
            std::ranges::generate(values, [&noise] { return noise(); });
            records.push_back(densityRecord(std::move(values)));
        }

        const gr::property_map settings{{"n_train", gr::Size_t{16U}}, {"n_guard", gr::Size_t{2U}}, {"pfa", kPfa}};
        const auto [out, counts] = collectCounted<CfarDetect>(settings, std::move(records));
        expect(eq(counts.first + counts.second, static_cast<std::uint64_t>(kRecords))) << "every input record is counted, as a record out or as an empty result";
        expect(eq(out.size(), static_cast<std::size_t>(counts.first))) << "the sink took every record counted as emitted";

        std::size_t detections = 0UZ;
        for (const auto& record : out) {
            detections += record.signal_values.size() / 3UZ;
        }
        const std::size_t margin   = 16UZ + 2UZ;
        const std::size_t tested   = kRecords * (kBins - 2UZ * margin);
        const double      measured = static_cast<double>(detections) / static_cast<double>(tested);
        expect(tested > 900000UZ) << "the rate is measured over about a million cells, tested " << tested;
        expect(measured < 2. * kPfa && measured > 0.5 * kPfa) << "measured false-alarm rate " << measured << " against a design " << kPfa;
    };

    "a tone 10 dB over the noise is detected, with few false alarms beside it"_test = [] {
        ExponentialNoise   noise;
        std::vector<float> values(kBins);
        std::ranges::generate(values, [&noise] { return noise(); });
        values[512UZ] += 10.f; // ten times the mean noise power

        const gr::property_map settings{{"n_train", gr::Size_t{16U}}, {"n_guard", gr::Size_t{2U}}, {"pfa", 1e-3}};
        const auto             out = collect<CfarDetect>(settings, {densityRecord(std::move(values))});
        expect(eq(out.size(), 1UZ));
        if (out.empty()) {
            return;
        }
        const auto  frequencies = column(out.front(), 0UZ);
        const float binWidth    = kSampleRate / static_cast<float>(kBins);
        const float expected    = (512.f - static_cast<float>(kBins / 2UZ)) * binWidth;
        expect(std::ranges::any_of(frequencies, [&](float f) { return std::abs(f - expected) < 0.5f * binWidth; })) << "the tone is found";
        expect(frequencies.size() <= 3UZ) << "the record holds at most three detections";
    };

    "alpha() and testableCells() follow the closed form and the margin"_test = [] {
        CfarDetect block({{"n_train", gr::Size_t{16U}}, {"n_guard", gr::Size_t{2U}}, {"pfa", 1e-3}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        // alpha = N * (pfa^(-1/N) - 1) at N = 32
        const double expected = 32. * (std::pow(1e-3, -1. / 32.) - 1.);
        expect(std::abs(block.alpha() - expected) < 1e-9) << "alpha() is the closed cell-averaging form";
        expect(eq(block.testableCells(kBins), kBins - 36UZ)) << "the training and guard cells at both ends are untested";
        expect(eq(block.testableCells(10UZ), 0UZ)) << "a record shorter than both margins has no testable cell";
    };

    // A record holds at most 2^31 - 1 cells, and a cell's test window spans 2 (n_train + n_guard) + 1 of them. Two
    // halves of 2^32 sum to zero in a 32-bit size.
    "a train and guard pair wider than any record is refused and written back, and the margin does not wrap"_test = [] {
        using gr::blocks::measurement::detail::kMaxCfarMargin;
        constexpr gr::Size_t  kHalf   = 1U << 31U;
        constexpr gr::Size_t  kWidest = kMaxCfarMargin;
        constexpr gr::Size_t  kOver   = (kWidest + 1U) / 2U; // two of them sum to one above the widest pair
        constexpr std::size_t kTrain  = 16UZ;
        constexpr std::size_t kGuard  = 2UZ;
        CfarDetect            block({{"n_train", gr::Size_t{kTrain}}, {"n_guard", gr::Size_t{kGuard}}, {"pfa", 1e-3}});
        startBlock(block);
        const std::size_t accepted = kBins - 2UZ * (kTrain + kGuard);

        const gr::property_map halves = refusedByLimits(block, {{"n_train", kHalf}, {"n_guard", kHalf}});
        expect(halves.contains("n_train") && halves.contains("n_guard")) << "the limits refuse each half of 2^32";
        expect(eq(block.testableCells(kBins), accepted)) << "and the accepted pair stays in force";

        const gr::property_map caps = refusedByLimits(block, {{"n_train", gr::Size_t{kWidest + 1U}}, {"n_guard", kWidest}});
        expect(caps.contains("n_train") && caps.contains("n_guard")) << "the limits refuse each value one above its own cap";
        expect(eq(block.testableCells(kBins), accepted)) << "and the accepted pair stays in force";

        expect(throws([&] { applyLive(block, {{"n_train", kOver}, {"n_guard", kOver}}); })) << "the block refuses a pair whose sum is one above the widest";
        expect(eq(block.testableCells(kBins), accepted)) << "and the accepted pair stays in force";

        const std::optional<gr::pmt::Value> reported = block.settings().get("n_train");
        expect(eq(block.n_train.value, gr::Size_t{kTrain}) && reported.has_value() && reported->get_if<gr::Size_t>() != nullptr && *reported->get_if<gr::Size_t>() == kTrain) << "the accepted n_train is written back to the member and settings().get()";
        constexpr std::size_t kNewGuard = 3UZ;
        expect(nothrow([&] { applyLive(block, {{"n_guard", gr::Size_t{kNewGuard}}}); })) << "a later change of n_guard alone is applied";
        expect(eq(block.testableCells(kBins), kBins - 2UZ * (kTrain + kNewGuard))) << "the block applies the new n_guard and keeps the accepted n_train";

        expect(nothrow([&] { applyLive(block, {{"n_train", gr::Size_t{kWidest - 1U}}, {"n_guard", gr::Size_t{1U}}}); })) << "a pair whose sum is the widest is accepted";
        expect(eq(block.testableCells(2UZ * std::size_t{kWidest} + 1UZ), 1UZ)) << "a record of 2^31 - 1 cells has one testable cell";
        expect(eq(block.testableCells(2UZ * std::size_t{kWidest}), 0UZ)) << "one cell fewer has none";
        const auto [status, taken, found] = workCall(block, {densityRecord(threeLobes())}, 4UZ);
        expect(eq(taken, 1UZ) && found.empty() && eq(block.nEmptyResults(), std::uint64_t{1ULL})) << "a record shorter than the test window reads no cell and detects nothing";
    };

    "a flat band with no transition reads its own edges"_test = [] {
        // 200 bins of unit density, zero outside them
        constexpr std::size_t kLow  = 412UZ;
        constexpr std::size_t kHigh = 612UZ;
        std::vector<float>    values(kBins, 0.f);
        std::fill(values.begin() + kLow, values.begin() + kHigh, 1.f);
        const float binWidth = kSampleRate / static_cast<float>(kBins);

        const auto full = collect<OccupiedBandwidth>({{"fraction", 1.0}}, {densityRecord(values)});
        expect(eq(full.size(), 1UZ));
        if (full.empty()) {
            return;
        }
        const float measured = full.front().signal_values[0UZ];
        const float designed = static_cast<float>(kHigh - kLow - 1UZ) * binWidth;
        expect(std::abs(measured - designed) < 2.f * binWidth) << "at fraction 1 the band spans every occupied bin, measured " << measured << " against " << designed;

        const auto ninetyNine = collect<OccupiedBandwidth>({{"fraction", 0.99}}, {densityRecord(values)});
        expect(!ninetyNine.empty());
        if (!ninetyNine.empty()) {
            const float narrowed = ninetyNine.front().signal_values[0UZ];
            expect(narrowed < measured) << "leaving out one percent of the power narrows the band";
            expect(narrowed > 0.97f * measured) << "by about one percent";
        }
    };

    "a noise band with raised-cosine edges reads its design bandwidth"_test = [] {
        // Seeded noise shaped by a flat band with raised-cosine edges of a stated width. The 99 % bandwidth must land
        // within one transition width of the design bandwidth.
        constexpr double      kBandBins       = 400.;
        constexpr double      kTransitionBins = 40.;
        constexpr std::size_t kTrials         = 32UZ;

        const auto shape = [](std::size_t k) {
            const double d     = std::abs(static_cast<double>(k) - static_cast<double>(kBins / 2UZ));
            const double inner = 0.5 * (kBandBins - kTransitionBins);
            const double outer = 0.5 * (kBandBins + kTransitionBins);
            if (d <= inner) {
                return 1.;
            }
            if (d >= outer) {
                return 1e-6; // a floor 60 dB below the band
            }
            return 0.5 * (1. + std::cos(std::numbers::pi * (d - inner) / kTransitionBins));
        };

        ExponentialNoise                noise;
        std::vector<gr::DataSet<float>> band;
        band.reserve(kTrials);
        for (std::size_t r = 0UZ; r < kTrials; ++r) {
            std::vector<float> values(kBins);
            for (std::size_t k = 0UZ; k < kBins; ++k) {
                values[k] = static_cast<float>(shape(k)) * noise();
            }
            band.push_back(densityRecord(std::move(values)));
        }

        const auto out = collect<OccupiedBandwidth>({{"fraction", 0.99}}, std::move(band));
        expect(eq(out.size(), kTrials));
        if (out.empty()) {
            return;
        }
        const float binWidth = kSampleRate / static_cast<float>(kBins);
        double      mean     = 0.;
        double      lowest   = 1e300;
        double      highest  = 0.;
        for (const auto& record : out) {
            const double bins = static_cast<double>(record.signal_values[0UZ]) / static_cast<double>(binWidth);
            mean += bins;
            lowest  = std::min(lowest, bins);
            highest = std::max(highest, bins);
        }
        mean /= static_cast<double>(out.size());
        std::println("99 % bandwidth of a {:.0f}-bin band with a {:.0f}-bin transition: mean {:.2f} bins over {} trials, range {:.2f} to {:.2f}", kBandBins, kTransitionBins, mean, out.size(), lowest, highest);

        expect(std::abs(mean - kBandBins) < kTransitionBins) << "the mean 99 % bandwidth lies within one transition width of the design, measured " << mean;
        expect(highest - lowest < 2. * kTransitionBins) << "the spread over the trials is under two transition widths, spread " << (highest - lowest);
    };

    "every detection and bandwidth record is well formed and carries its metadata"_test = [] {
        std::vector<float> withPeaks(kBins, 1e-6f);
        for (const std::size_t at : {200UZ, 512UZ, 800UZ}) {
            for (std::size_t k = at - 3UZ; k <= at + 3UZ; ++k) {
                withPeaks[k] += static_cast<float>(std::exp(-0.5 * static_cast<double>((k - at) * (k - at))));
            }
        }

        std::vector<gr::DataSet<float>> emitted;
        for (const auto& record : collect<PeakDetect>({{"threshold_db", 20.0}}, {densityRecord(withPeaks)})) {
            emitted.push_back(record);
        }
        for (const auto& record : collect<CfarDetect>({{"n_train", gr::Size_t{16U}}, {"n_guard", gr::Size_t{2U}}, {"pfa", 1e-3}}, {densityRecord(withPeaks)})) {
            emitted.push_back(record);
        }
        for (const auto& record : collect<OccupiedBandwidth>({{"fraction", 0.99}}, {densityRecord(withPeaks)})) {
            emitted.push_back(record);
        }
        expect(eq(emitted.size(), 3UZ)) << "each of the three blocks emits one record";

        for (std::size_t r = 0UZ; r < emitted.size(); ++r) {
            const auto& ds     = emitted[r];
            const char* reason = admissionFailure(ds);
            expect(reason == nullptr) << "record " << r << " would be rejected as " << (reason == nullptr ? "" : reason);
            expect(eq(ds.meta_information.size(), ds.signal_names.size())) << "record " << r << " lacks metadata on a signal";

            const auto& meta = ds.meta_information[0UZ];
            expect(meta.contains(std::pmr::string("sample_rate"))) << "record " << r << " lacks sample_rate";
            expect(meta.contains(std::pmr::string("sample_start"))) << "record " << r << " lacks sample_start";

            const auto& axis = ds.axis_values[0UZ];
            expect(eq(axis.size(), static_cast<std::size_t>(ds.extents[0UZ]))) << "record " << r << " has an axis of a different length from its extent";
            if (axis.size() >= 2UZ) {
                const float step = axis[1UZ] - axis[0UZ];
                for (std::size_t k = 1UZ; k < axis.size(); ++k) {
                    expect(std::abs((axis[k] - axis[k - 1UZ]) - step) <= 1e-6f * std::abs(step)) << "record " << r << " axis is not uniform at " << k;
                }
            }
        }
    };

    "the polled readings are zero before the first record and after a restart"_test = [] {
        OccupiedBandwidth block({{"fraction", 0.99}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
        expect(eq(block.nRecords(), std::uint64_t{0ULL})) << "no record is measured before the first input";
        expect(eq(block.bandwidth(), 0.));

        block._slot.publish({1234.0, -600.0, 634.0}, 5ULL); // readings a first run leaves
        expect(eq(block.nRecords(), std::uint64_t{5ULL}));
        block.start();
        expect(eq(block.nRecords(), std::uint64_t{0ULL})) << "a restart clears the first run's record count";
        expect(eq(block.bandwidth(), 0.)) << "and its bandwidth";
        expect(eq(block.lowerEdge(), 0.) && eq(block.upperEdge(), 0.));
    };

    "the scalar record carries the source record's sample_start"_test = [] {
        const auto records = collect<OccupiedBandwidth>({{"fraction", 0.99}}, {densityRecord(std::vector<float>(kBins, 1.f))});
        expect(eq(records.size(), 1UZ));
        if (records.empty()) {
            return;
        }
        const auto& ds = records.front();
        expect(eq(ds.extents.size(), 1UZ) && eq(ds.extents[0UZ], 1));
        expect(eq(ds.signal_names.size(), 3UZ) && eq(ds.signal_values.size(), 3UZ));
        expect(eq(ds.axis_names[0UZ], std::string("Measurement")));
        expect(eq(ds.axis_units[0UZ], std::string("index")));
        expect(eq(ds.meta_information.size(), 3UZ)) << "every signal carries the metadata";
        const auto& meta = ds.meta_information[0UZ];
        expect(meta.contains(std::pmr::string("fraction")));
        const auto start = meta.find(std::pmr::string("sample_start"));
        expect(start != meta.end()) << "the record carries sample_start";
        if (start != meta.end()) {
            const auto* index = start->second.get_if<std::uint64_t>();
            expect(index != nullptr && *index == 7ULL) << "equal to the source record's";
        }
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
