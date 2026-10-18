#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <limits>
#include <memory>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/algorithm/fourier/window.hpp>
#include <gnuradio-4.0/measurement/SpectralEstimate.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_spectral {

using gr::blocks::measurement::Spectrogram;
using gr::blocks::measurement::WelchPsd;
using CF = std::complex<float>;

constexpr std::size_t kFft        = 256UZ;
constexpr float       kSampleRate = 48000.f;

/// Emits a fixed sequence in bursts of a stated size, then ends the stream. Two burst sizes over the same samples
/// test chunk independence. Each tag in `tags` is published at its stream index, and a tag can change a setting of
/// the block downstream.
template<typename T>
struct BurstSource : gr::Block<BurstSource<T>> {
    gr::PortOut<T> out;

    std::vector<T>       samples{};
    std::vector<gr::Tag> tags{}; ///< index is a stream index
    std::size_t          burst = 4096UZ;
    std::size_t          at    = 0UZ;

    GR_MAKE_REFLECTABLE(BurstSource, out);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (at >= samples.size()) {
            outSpan.publish(0UZ);
            this->requestStop();
            return gr::work::Status::DONE;
        }
        const std::size_t take = std::min({burst, samples.size() - at, outSpan.size()});
        for (const gr::Tag& tag : tags) {
            if (tag.index >= at && tag.index < at + take) {
                outSpan.publishTag(tag.map, tag.index - at);
            }
        }
        std::copy_n(samples.begin() + static_cast<std::ptrdiff_t>(at), take, outSpan.begin());
        at += take;
        outSpan.publish(take);
        return gr::work::Status::OK;
    }
};

/// Takes at most `stride` records a call. A small stride holds back the block under test.
struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<gr::DataSet<float>, gr::Async> in;

    std::vector<gr::DataSet<float>> records{};
    std::size_t                     stride = std::numeric_limits<std::size_t>::max();
    std::size_t                     seen   = 0UZ;  ///< records taken, with `keep` on or off
    bool                            keep   = true; ///< off discards the records taken

    GR_MAKE_REFLECTABLE(RecordSink, in);

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        const std::size_t take = std::min(inSpan.size(), stride);
        if (keep) {
            records.insert(records.end(), inSpan.begin(), inSpan.begin() + static_cast<std::ptrdiff_t>(take));
        }
        seen += take;
        std::ignore = inSpan.consume(take);
        return gr::work::Status::OK;
    }
};

/// Takes one record on every `period`-th call. A period above one is a consumer slower than the block under test.
struct SlowSink : gr::Block<SlowSink> {
    gr::PortIn<gr::DataSet<float>, gr::Async> in;

    std::size_t                     period = 1UZ;
    std::size_t                     calls  = 0UZ;
    std::vector<gr::DataSet<float>> records{};

    GR_MAKE_REFLECTABLE(SlowSink, in);

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        ++calls;
        if (inSpan.size() == 0UZ || calls % period != 0UZ) {
            std::ignore = inSpan.consume(0UZ);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        records.push_back(inSpan[0UZ]);
        std::ignore = inSpan.consume(1UZ);
        return gr::work::Status::OK;
    }
};

[[nodiscard]] std::vector<CF> tone(std::size_t count, double binsPerFft, std::size_t fftSize, float amplitude = 1.f) {
    std::vector<CF>  data(count);
    constexpr double twoPi = 2. * std::numbers::pi;
    for (std::size_t k = 0UZ; k < count; ++k) {
        const double phase = twoPi * binsPerFft * static_cast<double>(k) / static_cast<double>(fftSize);
        data[k]            = CF(amplitude * static_cast<float>(std::cos(phase)), amplitude * static_cast<float>(std::sin(phase)));
    }
    return data;
}

/// @brief Seeded standard normal pairs, the parts of a complex white-noise sample.
struct GaussianNoise {
    std::uint64_t state = 0x243f6a8885a308d3ULL;

    [[nodiscard]] double uniform() {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return (static_cast<double>(state >> 11U) + 0.5) / static_cast<double>(1ULL << 53U);
    }

    [[nodiscard]] CF operator()() {
        const double radius = std::sqrt(-2. * std::log(uniform()));
        const double angle  = 2. * std::numbers::pi * uniform();
        return CF(static_cast<float>(radius * std::cos(angle)), static_cast<float>(radius * std::sin(angle)));
    }
};

/// @brief The five conditions `DataSetToStream` checks before it reads a record, in its order. Returns the reason
/// for a rejection, or nullptr. The block's own check is a private member, and this function repeats it.
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

[[nodiscard]] double metaNumber(const gr::DataSet<float>& record, std::string_view key, double fallback = -1.) {
    if (record.meta_information.empty()) {
        return fallback;
    }
    const auto& map = record.meta_information[0UZ];
    const auto  it  = map.find(std::pmr::string(key));
    if (it == map.end()) {
        return fallback;
    }
    if (const auto* asU64 = it->second.template get_if<std::uint64_t>()) {
        return static_cast<double>(*asU64);
    }
    if (const auto* asFloat = it->second.template get_if<float>()) {
        return static_cast<double>(*asFloat);
    }
    if (const auto* asDouble = it->second.template get_if<double>()) {
        return *asDouble;
    }
    return fallback;
}

[[nodiscard]] std::string metaString(const gr::DataSet<float>& record, std::string_view key) {
    if (record.meta_information.empty()) {
        return {};
    }
    const auto it = record.meta_information[0UZ].find(std::pmr::string(key));
    if (it == record.meta_information[0UZ].end() || !it->second.is_string()) {
        return {};
    }
    return it->second.value_or(std::string());
}

/// @brief Applies `changes` to a block through `settings().set()` and a context activation, in one batch.
template<typename TBlock>
void applyLive(TBlock& block, gr::property_map changes) {
    std::ignore = block.settings().set(std::move(changes));
    std::ignore = block.settings().activateContext();
    std::ignore = block.settings().applyStagedParameters();
}

/// @brief Stages `changes` on a started block and applies them. Returns the settings the limits refused.
template<typename TBlock>
[[nodiscard]] gr::property_map refusedByLimits(TBlock& block, gr::property_map changes) {
    std::ignore = block.settings().setStaged(std::move(changes));
    return block.settings().applyStagedParameters().failedParameters;
}

/// @brief Initializes a block's settings and starts it, as a scheduler does before the first work call.
template<typename TBlock>
void startBlock(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

/// @brief One work call on `samples` with room for `room` records. Returns the records and the samples consumed.
template<typename TBlock, typename T>
[[nodiscard]] std::pair<std::vector<gr::DataSet<float>>, std::size_t> workCall(TBlock& block, std::span<const T> samples, std::size_t room) {
    std::vector<gr::DataSet<float>>                           out(room);
    gr::blocks::testing::span::InputSpan<T>                   in{samples};
    gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> os{std::span<gr::DataSet<float>>(out)};
    std::ignore = block.processBulk(in, os);
    out.resize(os.count);
    return {std::move(out), in.consumed};
}

/**
 * @brief Feeds a whole stream through a started block by the framework's end-of-stream protocol, into an edge of
 * `capacity` records that a consumer empties by one record on every `period`-th call.
 *
 * The block's `processBulk` is called while the input holds at least `in.min_samples` samples, with a span of at most
 * `in.max_samples` and an output span of the edge's free room. `processEpilogue` is then called once with the rest of
 * the input and the free room at that moment. Returns the records in the order the block published them.
 */
template<typename TBlock, typename T>
[[nodiscard]] std::vector<gr::DataSet<float>> driveToEnd(TBlock& block, const std::vector<T>& samples, std::size_t capacity, std::size_t period) {
    std::vector<gr::DataSet<float>> published;
    std::size_t                     inEdge = 0UZ;
    std::size_t                     at     = 0UZ;
    for (std::size_t call = 1UZ; call <= 100UZ * samples.size(); ++call) {
        if (call % period == 0UZ && inEdge > 0UZ) {
            --inEdge;
        }
        std::vector<gr::DataSet<float>>                           room(capacity - inEdge);
        gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> out{std::span<gr::DataSet<float>>(room)};
        const std::size_t                                         left     = samples.size() - at;
        const bool                                                epilogue = left < std::max(block.in.min_samples, 1UZ);
        if (epilogue) {
            gr::blocks::testing::span::InputSpan<T> in{std::span<const T>(samples).subspan(at)};
            std::ignore = block.processEpilogue(in, out);
        } else {
            gr::blocks::testing::span::InputSpan<T> in{std::span<const T>(samples).subspan(at, std::min(left, block.in.max_samples))};
            std::ignore = block.processBulk(in, out);
            at += in.consumed;
        }
        published.insert(published.end(), room.begin(), room.begin() + static_cast<std::ptrdiff_t>(out.count));
        inEdge += out.count;
        if (epilogue) {
            break;
        }
    }
    return published;
}

/// @brief Runs one source-through-block-into-sink graph and returns the records the sink took.
template<typename TBlock, typename T>
[[nodiscard]] std::vector<gr::DataSet<float>> collect(gr::property_map settings, std::vector<T> samples, std::size_t burst, std::size_t sinkStride = std::numeric_limits<std::size_t>::max()) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<BurstSource<T>>();
    auto&     block  = graph.emplaceBlock<TBlock>(std::move(settings));
    auto&     sink   = graph.emplaceBlock<RecordSink>();
    source.samples   = std::move(samples);
    source.burst     = burst;
    sink.stride      = sinkStride;

    if (!graph.connect<"out", "in">(source, block).has_value() || !graph.connect<"out", "in">(block, sink).has_value()) {
        return {};
    }
    gr::scheduler::Simple scheduler;
    if (!scheduler.exchange(std::move(graph)).has_value()) {
        return {};
    }
    std::ignore = scheduler.runAndWait(); // the source ends the stream
    return sink.records;
}

/// @brief Runs a stream through `TBlock` into a sink that takes one record on every `period`-th call, and returns the
/// number of records the sink took.
template<typename TBlock>
[[nodiscard]] std::size_t recordsBehindSlowSink(gr::property_map settings, std::vector<CF> samples, std::size_t period) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<BurstSource<CF>>();
    auto&     block  = graph.emplaceBlock<TBlock>(std::move(settings));
    auto&     sink   = graph.emplaceBlock<SlowSink>();
    source.samples   = std::move(samples);
    sink.period      = period;

    if (!graph.connect<"out", "in">(source, block).has_value() || !graph.connect<"out", "in">(block, sink).has_value()) {
        return 0UZ;
    }
    gr::scheduler::Simple scheduler;
    if (!scheduler.exchange(std::move(graph)).has_value()) {
        return 0UZ;
    }
    std::ignore = scheduler.runAndWait();
    return sink.records.size();
}

/// @brief The variance of a record's bins over the square of their mean. For white noise the bins are independent
/// draws of one distribution, and the figure is the estimator's relative variance.
[[nodiscard]] double relativeVariance(const gr::DataSet<float>& record) {
    const auto& values = record.signal_values;
    double      mean   = 0.;
    for (const float value : values) {
        mean += static_cast<double>(value);
    }
    mean /= static_cast<double>(values.size());

    double variance = 0.;
    for (const float value : values) {
        const double deviation = static_cast<double>(value) - mean;
        variance += deviation * deviation;
    }
    variance /= static_cast<double>(values.size() - 1UZ);
    return variance / (mean * mean);
}

/// @brief The `compute_seconds` a record states, and the wall clock of the `processBulk` call that made it.
///
/// The call covers everything the figure covers, plus the copy into the accumulator's buffer and the span
/// bookkeeping. `settings` must make one whole segment complete one record. The reading is taken after `warmup`
/// records, since the first record includes the transform's plan and the first use of every buffer.
template<typename TBlock>
[[nodiscard]] std::pair<double, double> oneRecordCost(gr::property_map settings, std::size_t fftSize, std::size_t warmup = 2UZ) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();

    const std::vector<CF>           samples = tone(fftSize + 1UZ, 32., fftSize);
    std::vector<gr::DataSet<float>> room(2UZ);

    std::pair<double, double> reading{-1., 0.};
    for (std::size_t pass = 0UZ; pass <= warmup; ++pass) {
        gr::blocks::testing::span::InputSpan<CF>                  in{std::span<const CF>(samples)};
        gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> out{std::span<gr::DataSet<float>>(room)};

        const auto started   = std::chrono::steady_clock::now();
        std::ignore          = block.processBulk(in, out);
        const double outside = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

        reading = {out.count == 0UZ ? -1. : metaNumber(room[0UZ], "compute_seconds", -1.), outside};
    }
    return reading;
}

/// @brief The `compute_seconds` of the one record a graph makes, and the wall clock of the whole run. The run also
/// covers the scheduler's start-up, the source's copies and the copy of the record to the sink.
template<typename TBlock>
[[nodiscard]] std::pair<double, double> oneRecordGraphCost(gr::property_map settings, std::size_t fftSize) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<BurstSource<CF>>();
    auto&     block  = graph.emplaceBlock<TBlock>(std::move(settings));
    auto&     sink   = graph.emplaceBlock<RecordSink>();
    source.samples   = tone(fftSize + 1UZ, 32., fftSize);
    source.burst     = 65536UZ;

    if (!graph.connect<"out", "in">(source, block).has_value() || !graph.connect<"out", "in">(block, sink).has_value()) {
        return {-1., 0.};
    }
    const auto            started = std::chrono::steady_clock::now();
    gr::scheduler::Simple scheduler;
    if (!scheduler.exchange(std::move(graph)).has_value()) {
        return {-1., 0.};
    }
    std::ignore          = scheduler.runAndWait();
    const double outside = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

    return {sink.records.empty() ? -1. : metaNumber(sink.records.front(), "compute_seconds", -1.), outside};
}

/// @brief Records per second and the record count of one graph run. The graph feeds a fixed stream through one block
/// into a sink that discards the records.
template<typename TBlock>
[[nodiscard]] std::pair<double, std::size_t> recordRate(gr::property_map settings, std::size_t samples) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<BurstSource<CF>>();
    auto&     block  = graph.emplaceBlock<TBlock>(std::move(settings));
    auto&     sink   = graph.emplaceBlock<RecordSink>();
    source.samples.assign(samples, CF{0.7f, 0.2f});
    source.burst = 65536UZ;
    sink.keep    = false;

    if (!graph.connect<"out", "in">(source, block).has_value() || !graph.connect<"out", "in">(block, sink).has_value()) {
        return {0., 0UZ};
    }
    const auto            started = std::chrono::steady_clock::now();
    gr::scheduler::Simple scheduler;
    if (!scheduler.exchange(std::move(graph)).has_value()) {
        return {0., 0UZ};
    }
    std::ignore          = scheduler.runAndWait();
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

    return {elapsed > 0. ? static_cast<double>(sink.seen) / elapsed : 0., sink.seen};
}

} // namespace qa_spectral

const boost::ut::suite<"SpectralEstimate"> spectralTests = [] {
    using namespace boost::ut;
    using namespace qa_spectral;

    "a record states the segment count that made it, its overlap, rate, noise bandwidth and units"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{4U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kFft * 8UZ, 32., kFft), 4096UZ);

        expect(!records.empty()) << "an eight-segment input at four averages must produce records";
        if (records.empty()) {
            return;
        }
        const auto& first = records.front();
        expect(eq(first.signal_values.size(), kFft)) << "a complex input's record is two-sided";
        expect(eq(metaNumber(first, "n_averaged"), 4.)) << "the record states the segments that went into it";
        expect(approx(metaNumber(first, "overlap"), 0.5, 1e-9)) << "and the fraction they share";
        expect(eq(metaNumber(first, "fft_size"), static_cast<double>(kFft)));
        expect(approx(metaNumber(first, "sample_rate"), static_cast<double>(kSampleRate), 1e-3));
        expect(metaNumber(first, "enbw_bins") > 1.) << "a Hann window's noise bandwidth exceeds a bin";
        expect(eq(metaNumber(first, "sample_start"), 0.)) << "the first record starts at the stream's first sample";
        expect(eq(first.axis_names.size(), 1UZ) && eq(first.axis_units.size(), 1UZ));
        expect(eq(first.axis_units[0UZ], std::string("Hz")));
        expect(eq(first.signal_units[0UZ], std::string("1/Hz"))) << "the unit is that of a linear density";
    };

    "the axis is uniform and runs negative to positive for a complex input"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"sample_rate", kSampleRate}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kFft * 4UZ, 32., kFft), 4096UZ);
        expect(!records.empty());
        if (records.empty()) {
            return;
        }
        const auto& axis = records.front().axis_values[0UZ];
        expect(eq(axis.size(), kFft));
        const float step = axis[1UZ] - axis[0UZ];
        expect(approx(step, kSampleRate / static_cast<float>(kFft), 1e-3f));
        for (std::size_t k = 1UZ; k < axis.size(); ++k) {
            expect(std::abs((axis[k] - axis[k - 1UZ]) - step) < 1e-6f * std::abs(step) + 1e-6f) << "the axis steps are uniform";
        }
        expect(axis.front() < 0.f && axis.back() > 0.f) << "a two-sided axis straddles DC";
    };

    "a tone reads its own power through the stated noise-bandwidth correction"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{4U}}, {"sample_rate", kSampleRate}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kFft * 16UZ, 32., kFft), 4096UZ);
        expect(!records.empty());
        if (records.empty()) {
            return;
        }
        const auto& values   = records.front().signal_values;
        const float peak     = *std::ranges::max_element(values);
        const float binWidth = kSampleRate / static_cast<float>(kFft);
        const float power    = peak * static_cast<float>(metaNumber(records.front(), "enbw_bins")) * binWidth;
        expect(std::abs(10.f * std::log10(power)) < 0.01f) << "a full-scale complex tone on a bin center reads 0 dBFS, measured " << 10.f * std::log10(power);
    };

    "a full-scale real sine reads half the power of a full-scale complex tone, and the record names the reference"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{4U}}, {"sample_rate", kSampleRate}};
        std::vector<float>     real(kFft * 16UZ);
        for (std::size_t k = 0UZ; k < real.size(); ++k) {
            real[k] = static_cast<float>(std::cos(2. * std::numbers::pi * 32. * static_cast<double>(k) / static_cast<double>(kFft)));
        }
        const auto realRecords    = collect<WelchPsd<float>, float>(settings, real, 4096UZ);
        const auto complexRecords = collect<WelchPsd<CF>, CF>(settings, tone(kFft * 16UZ, 32., kFft), 4096UZ);
        expect(!realRecords.empty() && !complexRecords.empty()) << fatal;

        const float binWidth = kSampleRate / static_cast<float>(kFft);
        const auto  integral = [binWidth](const gr::DataSet<float>& record) {
            double total = 0.;
            for (const float value : record.signal_values) {
                total += static_cast<double>(value);
            }
            return total * static_cast<double>(binWidth);
        };
        expect(std::abs(integral(complexRecords.front()) - 1.) < 1e-4) << "a full-scale complex tone integrates to unit power";
        expect(std::abs(integral(realRecords.front()) - 0.5) < 1e-4) << "a full-scale real sine integrates to half of it, -3.01 dB";
        expect(eq(metaString(realRecords.front(), "level_reference"), std::string("unit power"))) << "the real record names the reference its level is read on";
        expect(eq(metaString(complexRecords.front(), "level_reference"), std::string("unit power"))) << "and so does the complex record";
    };

    "a record states the mode it was accumulated in"_test = [] {
        const gr::property_map mean{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{4U}}, {"sample_rate", kSampleRate}};
        const gr::property_map hold{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{4U}}, {"sample_rate", kSampleRate}, {"mode", std::string("max_hold")}};
        const gr::property_map rows{{"fft_size", gr::Size_t{kFft}}, {"sample_rate", kSampleRate}};
        const auto             samples = tone(kFft * 8UZ, 32., kFft);
        std::vector<float>     real(samples.size());
        std::ranges::transform(samples, real.begin(), [](CF sample) { return sample.real(); });
        const auto statesMode = [](const std::vector<gr::DataSet<float>>& records, const std::string& mode) { return !records.empty() && std::ranges::all_of(records, [&mode](const gr::DataSet<float>& record) { return metaString(record, "mode") == mode; }); };

        expect(statesMode(collect<WelchPsd<CF>, CF>(mean, samples, 4096UZ), "mean")) << "a WelchPsd record in mean mode states 'mean'";
        expect(statesMode(collect<WelchPsd<CF>, CF>(hold, samples, 4096UZ), "max_hold")) << "a WelchPsd record in max_hold mode states 'max_hold'";
        expect(statesMode(collect<WelchPsd<float>, float>(hold, real, 4096UZ), "max_hold")) << "and so does a record of a real input";
        expect(statesMode(collect<Spectrogram<CF>, CF>(rows, samples, 4096UZ), "mean")) << "a Spectrogram row, one segment, states 'mean'";

        // At one segment a record the hold made has the values of a mean record, and only `mode` tells them apart.
        const gr::property_map single{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}, {"one_record_per_call", false}};
        WelchPsd<CF>           reference(single);
        startBlock(reference);
        const auto [plain, all] = workCall(reference, std::span<const CF>(samples), 8UZ);
        WelchPsd<CF> block(single);
        startBlock(block);
        const auto [before, taken] = workCall(block, std::span<const CF>(samples).first(kFft * 2UZ), 4UZ);
        applyLive(block, {{"mode", std::string("max_hold")}});
        const auto [after, more] = workCall(block, std::span<const CF>(samples).subspan(kFft * 2UZ), 8UZ);
        expect(eq(plain.size(), 8UZ) && eq(before.size(), 2UZ) && eq(after.size(), 6UZ)) << fatal;
        expect(statesMode(before, "mean")) << "the records before a live change state the mode they were made in";
        expect(statesMode(after, "max_hold")) << "and the records after it state the new mode";
        expect(after.front().signal_values == plain[2UZ].signal_values) << "with the values a mean record of the same segment holds";
    };

    "chunk independence: the same samples in different bursts give the same records"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"sample_rate", kSampleRate}};
        const auto             samples = tone(kFft * 8UZ, 32., kFft);
        const auto             wide    = collect<WelchPsd<CF>, CF>(settings, samples, 4096UZ);
        const auto             narrow  = collect<WelchPsd<CF>, CF>(settings, samples, 37UZ);

        expect(eq(wide.size(), narrow.size())) << "the record count does not depend on the chunking";
        for (std::size_t r = 0UZ; r < std::min(wide.size(), narrow.size()); ++r) {
            expect(eq(metaNumber(wide[r], "sample_start"), metaNumber(narrow[r], "sample_start"))) << "the window grid counts from the stream's first sample";
            expect(eq(wide[r].signal_values.size(), narrow[r].signal_values.size()));
            for (std::size_t k = 0UZ; k < wide[r].signal_values.size(); ++k) {
                expect(wide[r].signal_values[k] == narrow[r].signal_values[k]) << "record " << r << " bin " << k << " differs between chunkings";
            }
        }
    };

    "a real input emits the one-sided record, DC through Nyquist"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"sample_rate", kSampleRate}};
        std::vector<float>     samples(kFft * 4UZ);
        for (std::size_t k = 0UZ; k < samples.size(); ++k) {
            samples[k] = std::cos(2.f * std::numbers::pi_v<float> * 32.f * static_cast<float>(k) / static_cast<float>(kFft));
        }
        const auto records = collect<WelchPsd<float>, float>(settings, samples, 4096UZ);
        expect(!records.empty());
        if (records.empty()) {
            return;
        }
        expect(eq(records.front().signal_values.size(), kFft / 2UZ + 1UZ));
        expect(approx(records.front().axis_values[0UZ].front(), 0.f, 1e-3f)) << "a one-sided axis starts at DC";
        expect(approx(records.front().axis_values[0UZ].back(), kSampleRate / 2.f, 1.f)) << "and ends at Nyquist";
    };

    "the spectrogram emits one record per hop, timestamped by the hop"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}};
        const auto             records = collect<Spectrogram<CF>, CF>(settings, tone(kFft * 4UZ, 32., kFft), 4096UZ);
        expect(records.size() >= 3UZ) << "four segments at half overlap yield several rows";
        if (records.size() < 2UZ) {
            return;
        }
        const double hop = static_cast<double>(kFft) / 2.;
        for (std::size_t r = 1UZ; r < records.size(); ++r) {
            expect(eq(metaNumber(records[r], "sample_start") - metaNumber(records[r - 1UZ], "sample_start"), hop)) << "rows advance by exactly one hop";
            expect(eq(metaNumber(records[r], "n_averaged"), 1.)) << "a spectrogram row is one transform";
            expect(approx(metaNumber(records[r], "overlap"), 0.5, 1e-9)) << "and it states its overlap, as the averaged records do";
        }
    };

    "a hop above fft_size transforms one segment in every hop and consumes the rest"_test = [] {
        // At hop 1024 the block transforms 256 of every 1024 samples. It skips the other 768 and counts them into
        // the stream position.
        constexpr std::size_t  kHop     = 1024UZ;
        constexpr std::size_t  kSamples = kHop * 10UZ;
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"hop", gr::Size_t{kHop}}, {"sample_rate", kSampleRate}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kSamples, 32., kFft), 4096UZ);

        expect(eq(records.size(), 10UZ)) << "one segment starts at every hop the stream has room for, made " << records.size();
        for (std::size_t r = 0UZ; r < records.size(); ++r) {
            expect(eq(metaNumber(records[r], "sample_start"), static_cast<double>(r * kHop))) << "record " << r << " starts off the hop grid";
            expect(eq(metaNumber(records[r], "hop"), static_cast<double>(kHop))) << "the record states its hop";
            expect(eq(metaNumber(records[r], "overlap"), 0.)) << "segments a gap apart share no samples";
            expect(eq(metaNumber(records[r], "fft_size"), static_cast<double>(kFft))) << "the transform has the stated length, not the hop";
            expect(eq(records[r].signal_values.size(), kFft));
        }
    };

    "a skipped gap changes nothing about the segments that are transformed"_test = [] {
        // The same stream at hop 1024 and at hop 256. Each hop-1024 record is the same 256 samples through the same
        // window as every fourth hop-256 record, and the two must be bit-identical.
        const auto             samples = tone(kFft * 16UZ, 32., kFft);
        const gr::property_map gapped{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"hop", gr::Size_t{kFft * 4UZ}}, {"sample_rate", kSampleRate}};
        const gr::property_map dense{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};

        const auto sparse = collect<WelchPsd<CF>, CF>(gapped, samples, 4096UZ);
        const auto every  = collect<WelchPsd<CF>, CF>(dense, samples, 4096UZ);
        expect(eq(sparse.size(), 4UZ)) << "sixteen segments of stream hold four segments a gap apart";
        expect(eq(every.size(), 16UZ));
        for (std::size_t r = 0UZ; r < std::min(sparse.size(), every.size() / 4UZ); ++r) {
            const auto& skipped = sparse[r];
            const auto& whole   = every[r * 4UZ];
            expect(eq(metaNumber(skipped, "sample_start"), metaNumber(whole, "sample_start")));
            for (std::size_t k = 0UZ; k < skipped.signal_values.size(); ++k) {
                expect(skipped.signal_values[k] == whole.signal_values[k]) << "record " << r << " bin " << k << " differs from the record without a gap";
            }
        }
    };

    "chunk independence holds across a skipped gap"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"hop", gr::Size_t{1000U}}, {"sample_rate", kSampleRate}};
        const auto             samples = tone(12000UZ, 32., kFft);
        const auto             wide    = collect<WelchPsd<CF>, CF>(settings, samples, 4096UZ);
        const auto             narrow  = collect<WelchPsd<CF>, CF>(settings, samples, 37UZ);

        expect(!wide.empty()) << "a hop of 1000 samples produces records";
        expect(eq(wide.size(), narrow.size())) << "the record count does not depend on the chunking";
        for (std::size_t r = 0UZ; r < std::min(wide.size(), narrow.size()); ++r) {
            expect(eq(metaNumber(wide[r], "sample_start"), metaNumber(narrow[r], "sample_start"))) << "record " << r << " moved in the stream";
            for (std::size_t k = 0UZ; k < wide[r].signal_values.size(); ++k) {
                expect(wide[r].signal_values[k] == narrow[r].signal_values[k]) << "record " << r << " bin " << k << " differs between chunkings";
            }
        }
    };

    "the spectrogram's rows may be a gap apart, and stay on the stream's own time axis"_test = [] {
        constexpr std::size_t  kHop = 1024UZ;
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"hop", gr::Size_t{kHop}}, {"sample_rate", kSampleRate}};
        const auto             records = collect<Spectrogram<CF>, CF>(settings, tone(kHop * 6UZ, 32., kFft), 4096UZ);

        expect(eq(records.size(), 6UZ)) << "six hops fit in the stream, made " << records.size();
        for (std::size_t r = 1UZ; r < records.size(); ++r) {
            expect(eq(metaNumber(records[r], "sample_start") - metaNumber(records[r - 1UZ], "sample_start"), static_cast<double>(kHop))) << "rows advance by exactly one hop";
            expect(eq(metaNumber(records[r], "hop"), static_cast<double>(kHop)));
        }
    };

    "a hop below fft_size is an overlap named in samples"_test = [] {
        // A hop of 100 out of 256 samples is a fraction that `overlap` cannot state exactly.
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"hop", gr::Size_t{100U}}, {"sample_rate", kSampleRate}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kFft * 8UZ, 32., kFft), 4096UZ);

        expect(records.size() >= 3UZ);
        if (records.size() < 2UZ) {
            return;
        }
        expect(eq(metaNumber(records[1UZ], "sample_start") - metaNumber(records[0UZ], "sample_start"), 100.)) << "the stated hop takes precedence over the overlap";
        expect(approx(metaNumber(records[0UZ], "hop"), 100., 1e-9));
        expect(approx(metaNumber(records[0UZ], "overlap"), 1. - 100. / static_cast<double>(kFft), 1e-9)) << "and the record states the fraction that hop shares";
    };

    "the hop is live, and moving it restarts the accumulation without moving the stream"_test = [] {
        WelchPsd<CF> block({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{8U}}, {"sample_rate", kSampleRate}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
        expect(eq(block._core.hop, kFft / 2UZ)) << "with no hop stated, the overlap sets it";

        block._core.streamAt = 4096ULL;
        block._core.segments = 3UZ;
        block._core.skipping = 700UZ; // a gap left by the hop about to be replaced

        std::ignore = block.settings().set({{"hop", gr::Size_t{5000U}}});
        std::ignore = block.settings().activateContext();
        std::ignore = block.settings().applyStagedParameters();

        expect(eq(block._core.hop, 5000UZ)) << "the new hop is in force";
        expect(eq(block._core.segments, 0UZ)) << "and the accumulation in progress is dropped";
        expect(eq(block._core.skipping, 0UZ)) << "the old hop's gap is dropped";
        expect(eq(block._core.gridStart, std::uint64_t{4096ULL})) << "a new hop starts a new grid at the stream position";
        expect(eq(block._core.streamAt, std::uint64_t{4096ULL})) << "the stream position is kept";

        // A change that keeps the hop keeps the grid and the gap.
        block._core.skipping = 700UZ;
        std::ignore          = block.settings().set({{"window", std::string("Hamming")}});
        std::ignore          = block.settings().activateContext();
        std::ignore          = block.settings().applyStagedParameters();
        expect(eq(block._core.skipping, 700UZ)) << "a window change keeps the gap";
        expect(eq(block._core.gridStart, std::uint64_t{4096ULL}));
    };

    "window_param builds the window the library builds, and the record says which one it is"_test = [] {
        // Kaiser at beta 6.76 and Kaiser at the default beta 1.6 are different windows.
        constexpr float kBeta = 6.76f;

        const auto build = [](gr::property_map settings) {
            auto block = std::make_unique<WelchPsd<CF>>(std::move(settings));
            block->settings().init();
            std::ignore = block->settings().applyStagedParameters();
            return block;
        };
        const auto stated  = build({{"fft_size", gr::Size_t{kFft}}, {"sample_rate", kSampleRate}, {"window", std::string("Kaiser")}, {"window_param", kBeta}});
        const auto omitted = build({{"fft_size", gr::Size_t{kFft}}, {"sample_rate", kSampleRate}, {"window", std::string("Kaiser")}});

        const auto direct  = gr::algorithm::window::create<float>(gr::algorithm::window::Type::Kaiser, kFft, kBeta);
        const auto library = gr::algorithm::window::create<float>(gr::algorithm::window::Type::Kaiser, kFft);

        expect(eq(stated->_core.window.size(), direct.size()));
        for (std::size_t k = 0UZ; k < std::min(stated->_core.window.size(), direct.size()); ++k) {
            expect(stated->_core.window[k] == direct[k]) << "tap " << k << " is not what window::create builds at beta " << kBeta;
        }
        std::size_t differing = 0UZ;
        for (std::size_t k = 0UZ; k < stated->_core.window.size(); ++k) {
            differing += stated->_core.window[k] != library[k] ? 1UZ : 0UZ;
        }
        expect(differing > kFft / 2UZ) << "a stated beta gives a different window from the default, " << differing << " taps differ";

        for (std::size_t k = 0UZ; k < omitted->_core.window.size(); ++k) {
            expect(omitted->_core.window[k] == library[k]) << "tap " << k << " does not match the library's default";
        }

        // The parameter changes the noise bandwidth, and the record states both the window and the parameter.
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"sample_rate", kSampleRate}, {"window", std::string("Kaiser")}, {"window_param", kBeta}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kFft * 8UZ, 32., kFft), 4096UZ);
        expect(!records.empty());
        if (records.empty()) {
            return;
        }
        expect(approx(metaNumber(records.front(), "window_param"), static_cast<double>(kBeta), 1e-5)) << "the record states its beta";

        const gr::property_map defaulted{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"sample_rate", kSampleRate}, {"window", std::string("Kaiser")}};
        const auto             plain = collect<WelchPsd<CF>, CF>(defaulted, tone(kFft * 8UZ, 32., kFft), 4096UZ);
        expect(!plain.empty());
        if (plain.empty()) {
            return;
        }
        expect(approx(metaNumber(plain.front(), "window_param"), 1.6, 1e-5)) << "with no parameter set, the record states the window's default";

        // A larger beta lowers the sidelobes and widens the main lobe, and the noise bandwidth grows. The test asserts
        // the order and prints the two figures.
        const double statedEnbw  = metaNumber(records.front(), "enbw_bins");
        const double defaultEnbw = metaNumber(plain.front(), "enbw_bins");
        std::println("window_param: Kaiser ENBW {:.4f} bins at beta {:.2f}, {:.4f} at the default 1.6", statedEnbw, static_cast<double>(kBeta), defaultEnbw);
        expect(statedEnbw > defaultEnbw) << "beta 6.76 spreads a tone over more bins than beta 1.6, measured " << statedEnbw << " against " << defaultEnbw;
    };

    "a window parameter outside the window's range is refused"_test = [] {
        const auto refused = [](gr::property_map settings) {
            settings.insert({std::pmr::string("sample_rate"), gr::pmt::Value(kSampleRate)});
            WelchPsd<CF> block(std::move(settings));
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        };
        expect(throws([&] { refused({{"window", std::string("Gaussian")}, {"window_param", 0.9f}}); })) << "a Gaussian sigma is a fraction of the half-length";
        expect(throws([&] { refused({{"window", std::string("Tukey")}, {"window_param", 2.0f}}); })) << "a Tukey alpha is a fraction";
        expect(throws([&] { refused({{"window", std::string("Kaiser")}, {"window_param", -1.0f}}); })) << "a Kaiser beta is not negative";
        expect(nothrow([&] { refused({{"window", std::string("Hann")}, {"window_param", 3.0f}}); })) << "a window without a parameter ignores one, as the library does";
    };

    "the largest transform length is accepted, and one above it is refused"_test = [] {
        // At 2^22 the test checks the settings and the window. A run at that length belongs in a benchmark. A length of
        // 65536 runs through a graph below.
        constexpr gr::Size_t kCeiling = 4194304U;

        WelchPsd<CF> atCeiling({{"fft_size", kCeiling}, {"n_averages", gr::Size_t{1U}}, {"sample_rate", kSampleRate}});
        expect(nothrow([&] {
            atCeiling.settings().init();
            std::ignore = atCeiling.settings().applyStagedParameters();
        })) << "the largest length is accepted";
        expect(eq(atCeiling._core.fftSize, static_cast<std::size_t>(kCeiling)));
        expect(eq(atCeiling._core.window.size(), static_cast<std::size_t>(kCeiling))) << "and the window is built for it";

        WelchPsd<CF> above({{"sample_rate", kSampleRate}});
        startBlock(above);
        expect(refusedByLimits(above, {{"fft_size", kCeiling * 2U}}).contains("fft_size")) << "and the limits refuse the next power of two";
        expect(eq(above.fft_size.value, gr::Size_t{1024U}));

        constexpr std::size_t  kMid = 65536UZ;
        const gr::property_map settings{{"fft_size", gr::Size_t{static_cast<unsigned>(kMid)}}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kMid * 3UZ, 1024., kMid), 65536UZ);
        expect(eq(records.size(), 3UZ)) << "three whole transforms of 65536 samples, made " << records.size();
        if (!records.empty()) {
            expect(eq(records.front().signal_values.size(), kMid));
            expect(eq(records.front().axis_values[0UZ].size(), kMid)) << "the axis has one point per bin";
        }
    };

    "settings that cannot describe a measurement are refused"_test = [] {
        const auto refused = [](gr::property_map settings) {
            settings.insert({std::pmr::string("sample_rate"), gr::pmt::Value(kSampleRate)});
            WelchPsd<CF> block(std::move(settings));
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        };
        expect(throws([&] { refused({{"mode", std::string("median")}}); })) << "a mode the accumulation has no rule for";
        expect(throws([&] { refused({{"window", std::string("Gaussian2")}}); })) << "a window the vocabulary has no name for";

        // values outside the declared limits
        const std::vector<std::pair<gr::property_map, std::string_view>> outside{
            {{{"fft_size", gr::Size_t{100U}}}, "a transform length that is not a power of two"},
            {{{"fft_size", gr::Size_t{32U}}}, "below the stated floor"},
            {{{"overlap", 1.0}}, "a segment cannot wholly overlap its successor"},
            {{{"overlap", -0.1}}, "a negative overlap"},
            {{{"n_averages", gr::Size_t{0U}}}, "no segment to average"},
            {{{"sample_rate", 0.f}}, "a rate of zero"},
            {{{"sample_rate", -48000.f}}, "a negative rate"},
            {{{"sample_rate", std::numeric_limits<float>::infinity()}}, "an infinite rate"},
            {{{"sample_rate", std::numeric_limits<float>::quiet_NaN()}}, "a rate that is not a number"},
            {{{"center_frequency", std::numeric_limits<double>::infinity()}}, "an infinite tuning"},
        };
        for (const auto& [setting, what] : outside) {
            WelchPsd<CF> block(gr::property_map{});
            startBlock(block);
            expect(throws([&] { std::ignore = block.settings().set(setting); })) << what << ": settings().set() refuses it at the call";
            expect(eq(refusedByLimits(block, setting).size(), 1UZ)) << what << ": the limits refuse a staged one";
            expect(eq(block.fft_size.value, gr::Size_t{1024U}) && eq(block.overlap.value, 0.5) && eq(block.n_averages.value, gr::Size_t{16U}) && eq(block.sample_rate.value, 1.f)) << what << ": the members keep the accepted values";
        }
        Spectrogram<CF> rows(gr::property_map{});
        startBlock(rows);
        expect(refusedByLimits(rows, {{"sample_rate", std::numeric_limits<float>::infinity()}}).contains("sample_rate")) << "the spectrogram's limits refuse an infinite rate too";
    };

    "a value outside its limits on a tag is refused, and the run goes on at the accepted value"_test = [] {
        // fft_size keeps its default of 1024, so a tag reaches it. The live fft_size case above shows that a valid tag
        // reaches the block.
        constexpr std::size_t kAt = 4096UZ;
        struct Outcome {
            bool                            completed{};
            std::vector<gr::DataSet<float>> records;
            gr::Size_t                      member{};
        };
        const auto run = [&]<typename TBlock>(gr::property_map settings, const gr::property_map& tagged) {
            gr::Graph graph;
            auto&     source = graph.emplaceBlock<BurstSource<CF>>();
            auto&     block  = graph.emplaceBlock<TBlock>(std::move(settings));
            auto&     sink   = graph.emplaceBlock<RecordSink>();
            source.samples   = tone(8192UZ, 32., 1024UZ);
            source.burst     = 512UZ;
            source.tags.push_back(gr::Tag{kAt, tagged});
            Outcome outcome;
            if (!graph.connect<"out", "in">(source, block).has_value() || !graph.connect<"out", "in">(block, sink).has_value()) {
                return outcome;
            }
            gr::scheduler::Simple scheduler;
            if (!scheduler.exchange(std::move(graph)).has_value()) {
                return outcome;
            }
            outcome.completed = scheduler.runAndWait().has_value();
            outcome.records   = sink.records;
            outcome.member    = block.fft_size.value;
            return outcome;
        };
        const gr::property_map settings{{"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};
        const gr::property_map rows{{"overlap", 0.0}, {"sample_rate", kSampleRate}};
        for (const gr::Size_t size : {gr::Size_t{100U}, gr::Size_t{32U}}) {
            const gr::property_map tagged{{"fft_size", size}};
            for (const auto& [name, outcome] : {std::pair{"WelchPsd", run.template operator()<WelchPsd<CF>>(settings, tagged)}, std::pair{"Spectrogram", run.template operator()<Spectrogram<CF>>(rows, tagged)}}) {
                expect(outcome.completed) << name << ", fft_size " << size << ": the run completes";
                expect(eq(outcome.records.size(), 8UZ)) << name << ", fft_size " << size << ": eight transforms of 1024 samples";
                for (const auto& record : outcome.records) {
                    expect(eq(metaNumber(record, "fft_size"), 1024.)) << name << ", fft_size " << size << ": every record uses the accepted length";
                }
                expect(eq(outcome.member, gr::Size_t{1024U})) << name << ", fft_size " << size << ": the member keeps the accepted value";
            }
        }
    };

    "a stream ending mid-average flushes what it has, marked with its count"_test = [] {
        // Five whole segments at eight averages. The one record is the end-of-stream flush.
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{8U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kFft * 5UZ, 32., kFft), 4096UZ);

        expect(eq(records.size(), 1UZ)) << "the partial accumulation is emitted";
        if (records.empty()) {
            return;
        }
        expect(eq(metaNumber(records.front(), "n_averaged"), 5.)) << "and it counts its segments";
        expect(eq(metaNumber(records.front(), "sample_start"), 0.));
    };

    "the spectrogram's last hop is emitted, not left in the buffer"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};
        const auto             records = collect<Spectrogram<CF>, CF>(settings, tone(kFft * 5UZ, 32., kFft), 4096UZ);
        expect(eq(records.size(), 5UZ)) << "five whole transforms fit in the stream and five rows come out";
    };

    "no record is lost behind a sink that takes one at a time"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};
        const auto             samples = tone(kFft * 400UZ, 32., kFft);
        const auto             quick   = collect<WelchPsd<CF>, CF>(settings, samples, 4096UZ);
        const auto             slow    = collect<WelchPsd<CF>, CF>(settings, samples, 4096UZ, 1UZ);

        expect(quick.size() >= 190UZ) << "the run is long enough to fill the record buffer, made " << quick.size();
        expect(eq(slow.size(), quick.size())) << "back-pressure delays records; it does not drop them";
        for (std::size_t r = 0UZ; r < std::min(slow.size(), quick.size()); ++r) {
            expect(eq(metaNumber(slow[r], "sample_start"), metaNumber(quick[r], "sample_start"))) << "record " << r << " lost its place in the stream";
        }
    };

    "a starved call says it lacks input rather than answering OK"_test = [] {
        // The framework calls the block with an empty input span, since the output port is asynchronous. A call that
        // takes and makes nothing must not answer OK. OK reports progress, and the scheduler would call the block
        // again at once.
        const auto fresh = [] {
            auto block = std::make_unique<WelchPsd<CF>>(gr::property_map{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}});
            block->settings().init();
            std::ignore = block->settings().applyStagedParameters();
            block->start();
            return block;
        };

        std::vector<gr::DataSet<float>> room(4UZ);

        auto                                                      starved = fresh();
        gr::blocks::testing::span::InputSpan<CF>                  noInput{std::span<const CF>{}};
        gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> outEmpty{std::span<gr::DataSet<float>>(room)};
        expect(starved->processBulk(noInput, outEmpty) == gr::work::Status::INSUFFICIENT_INPUT_ITEMS) << "an empty span is a lack of input, not progress";
        expect(eq(outEmpty.count, 0UZ)) << "and nothing is published on that path";

        // A span of one sample with room in the output is consumed.
        const std::vector<CF>                                     single{CF{1.f, 0.f}};
        gr::blocks::testing::span::InputSpan<CF>                  oneSample{std::span<const CF>(single)};
        gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> outOne{std::span<gr::DataSet<float>>(room)};
        expect(starved->processBulk(oneSample, outOne) == gr::work::Status::OK) << "one sample is input the block takes";
        expect(eq(oneSample.consumed, 1UZ)) << "and it is consumed";
        expect(eq(starved->in.min_samples, 1UZ)) << "the block asks for no more than one sample a call";

        // With no output room, a span whose first sample completes a record moves nothing and says it waits on the
        // output.
        auto                                                      blocked = fresh();
        const std::vector<CF>                                     segment = tone(kFft, 32., kFft);
        std::vector<gr::DataSet<float>>                           none;
        gr::blocks::testing::span::InputSpan<CF>                  almost{std::span<const CF>(segment).first(kFft - 1UZ)};
        gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> outNone{std::span<gr::DataSet<float>>(none)};
        expect(blocked->processBulk(almost, outNone) == gr::work::Status::OK) << "samples short of a record are taken without room";
        gr::blocks::testing::span::InputSpan<CF>                  last{std::span<const CF>(segment).last(1UZ)};
        gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> outStill{std::span<gr::DataSet<float>>(none)};
        expect(blocked->processBulk(last, outStill) == gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS) << "the sample that completes a record waits for room";
        expect(eq(last.consumed, 0UZ)) << "and stays in the input";

        // A span the block can fold answers OK, and the status tells the two cases apart.
        auto                                                      fed     = fresh();
        const std::vector<CF>                                     samples = tone(kFft + 1UZ, 32., kFft);
        gr::blocks::testing::span::InputSpan<CF>                  hasInput{std::span<const CF>(samples)};
        gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> outFed{std::span<gr::DataSet<float>>(room)};
        expect(fed->processBulk(hasInput, outFed) == gr::work::Status::OK) << "a span it can take from is progress";
        expect(gt(hasInput.consumed, 0UZ)) << "and it took from it";
    };

    "a live setting restarts the accumulation and keeps the stream-absolute grid"_test = [] {
        WelchPsd<CF> block({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{8U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();

        block._core.pending.assign(100UZ, CF{1.f, 0.f});
        block._core.streamAt      = 4096ULL;
        block._core.recordStartAt = 4096ULL;
        block._core.segments      = 3UZ;

        std::ignore = block.settings().set({{"n_averages", gr::Size_t{4U}}, {"window", std::string("Hamming")}});
        std::ignore = block.settings().activateContext();
        std::ignore = block.settings().applyStagedParameters();

        expect(eq(block._core.streamAt, std::uint64_t{4096ULL})) << "the stream position is kept";
        expect(eq(block._core.pending.size(), 100UZ)) << "and no buffered sample is dropped";
        expect(eq(block._core.segments, 0UZ)) << "the partial average is dropped, since a record never mixes two windows";
        expect(eq(block._core.recordStartAt, std::uint64_t{4096ULL})) << "the next record starts at the stream position";
        expect(eq(block._core.nAverages, 4UZ)) << "and the new setting is in force";

        block._core.segments = 2UZ;
        applyLive(block, {{"signal_name", std::string("psd_b")}});
        expect(eq(block._core.segments, 2UZ)) << "a change to signal_name alone keeps the accumulation";
    };

    "fft_size moves under a running block, keeping the stream and losing no buffered sample"_test = [] {
        const auto running = [](gr::property_map settings) {
            auto block = std::make_unique<WelchPsd<CF>>(std::move(settings));
            block->settings().init();
            std::ignore = block->settings().applyStagedParameters();
            block->start();
            return block;
        };
        const auto live = [](auto& block, gr::property_map changes) {
            std::ignore = block.settings().set(std::move(changes));
            std::ignore = block.settings().activateContext();
            std::ignore = block.settings().applyStagedParameters();
        };

        {
            auto block = running({{"fft_size", gr::Size_t{kFft}}, {"sample_rate", kSampleRate}});
            expect(nothrow([&] { live(*block, {{"overlap", 0.75}}); })) << "the overlap is live";
            expect(eq(block->_core.hop, kFft / 4UZ));
        }
        {
            auto block = running({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{8U}}, {"sample_rate", kSampleRate}});
            block->_core.pending.assign(60UZ, CF{1.f, 0.f});
            block->_core.streamAt      = 9000ULL;
            block->_core.recordStartAt = 9000ULL;
            block->_core.segments      = 5UZ;

            expect(nothrow([&] { live(*block, {{"fft_size", gr::Size_t{512U}}}); })) << "a running block takes a new transform length";
            expect(eq(block->_core.fftSize, 512UZ)) << "and is built for it";
            expect(eq(block->_core.window.size(), 512UZ));
            expect(eq(block->_core.windowed.size(), 512UZ)) << "the transform's buffers follow the length";
            expect(eq(block->_core.accumulator.size(), 512UZ));
            expect(eq(block->_core.segments, 0UZ)) << "the five segments at the old length are dropped";
            expect(eq(block->_core.pending.size(), 60UZ)) << "no buffered sample is dropped";
            expect(eq(block->_core.streamAt, std::uint64_t{9000ULL})) << "and the stream position is kept";
            expect(eq(block->_core.gridStart, std::uint64_t{9000ULL})) << "the new grid starts at the stream position";
        }
    };

    "a live fft_size change reaches a block inside a running graph, through a tag"_test = [] {
        // A tag whose key names a setting changes that setting in a running graph. The framework applies such a tag
        // only to a setting the constructor left at its default, and fft_size stays at its default of 1024 here. The
        // source places the tag at a known sample.
        constexpr std::size_t kFirst  = 1024UZ; // the block's default
        constexpr std::size_t kSecond = 256UZ;
        constexpr std::size_t kAt     = 4096UZ; // a segment boundary of the first grid

        gr::property_map settings{{"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};
        gr::property_map moveTo{{"fft_size", gr::Size_t{static_cast<unsigned>(kSecond)}}};

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<BurstSource<CF>>();
        auto&     block  = graph.emplaceBlock<WelchPsd<CF>>(std::move(settings));
        auto&     sink   = graph.emplaceBlock<RecordSink>();
        source.samples   = tone(8192UZ, 32., kFirst);
        source.burst     = 512UZ;
        source.tags.push_back(gr::Tag{kAt, std::move(moveTo)});

        expect(graph.connect<"out", "in">(source, block).has_value() && graph.connect<"out", "in">(block, sink).has_value()) << fatal;
        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value()) << fatal;
        std::ignore = scheduler.runAndWait();

        const auto& records = sink.records;
        expect(records.size() > 4UZ) << "the run produces records at both lengths, made " << records.size();

        std::size_t atFirst  = 0UZ;
        std::size_t atSecond = 0UZ;
        for (const auto& record : records) {
            const auto size = static_cast<std::size_t>(metaNumber(record, "fft_size"));
            if (size == kFirst) {
                ++atFirst;
                expect(eq(metaNumber(record, "grid_start"), 0.)) << "records before the change are on the first grid";
            } else if (size == kSecond) {
                ++atSecond;
                expect(eq(record.signal_values.size(), kSecond)) << "a record's length is that of its transform";
                expect(metaNumber(record, "grid_start") > 0.) << "and records after it state the new grid";
                expect(metaNumber(record, "sample_start") >= metaNumber(record, "grid_start")) << "no segment starts before its grid";
            }
            expect(eq(metaNumber(record, "n_averaged"), 1.)) << "no record is a partial average across the change";
        }
        expect(atFirst > 0UZ) << "records at the first length";
        expect(atSecond > 0UZ) << "and records at the second";

        // The stream position counts the same stream across the change, and sample_start only increases.
        for (std::size_t r = 1UZ; r < records.size(); ++r) {
            expect(metaNumber(records[r], "sample_start") > metaNumber(records[r - 1UZ], "sample_start")) << "record " << r << " went backwards in the stream";
        }
    };

    "a restart clears the flush latch, so the second run reports its own partial record"_test = [] {
        WelchPsd<CF> block({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{8U}}, {"sample_rate", kSampleRate}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();

        block.start();
        block._flushed       = true; // the state at the end of a first run
        block._core.streamAt = 999ULL;
        block._core.pending.assign(7UZ, CF{});

        block.start(); // the start of a second run
        expect(!block._flushed) << "a second run's flush is not suppressed by the first run's";
        expect(eq(block._core.streamAt, std::uint64_t{0ULL})) << "and the grid starts at the new stream's first sample";
        expect(block._core.pending.empty());
    };

    "halving n_averages doubles the estimator variance, within an envelope"_test = [] {
        // For white noise, the average of n independent periodograms has a per-bin variance of mean^2 / n. The ratio
        // between 32 and 64 averages is 2. The segments do not overlap and are independent. The test prints the
        // measured ratio and bounds it.
        GaussianNoise   noise;
        std::vector<CF> samples(kFft * 64UZ * 4UZ);
        std::ranges::generate(samples, [&noise] { return noise(); });

        const auto measure = [&samples](gr::Size_t averages) {
            const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", averages}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};
            const auto             records = collect<WelchPsd<CF>, CF>(settings, samples, 4096UZ);
            double                 total   = 0.;
            std::size_t            counted = 0UZ;
            for (const auto& record : records) {
                if (metaNumber(record, "n_averaged") == static_cast<double>(averages)) { // whole records only, not the flush
                    total += relativeVariance(record);
                    ++counted;
                }
            }
            return counted == 0UZ ? 0. : total / static_cast<double>(counted);
        };

        const double at64  = measure(64U);
        const double at32  = measure(32U);
        const double ratio = at32 / at64;
        std::println("relative estimator variance {:.5f} at 64 averages, {:.5f} at 32, ratio {:.3f} (nominal 1/64, 1/32, 2)", at64, at32, ratio);

        expect(at64 > 0. && at32 > 0.) << "both settings produce whole records";
        expect(ratio > 1.5 && ratio < 2.7) << "the variance envelope is a factor of two, measured " << ratio;
    };

    "every emitted record is well formed and states its rate, start, segment count, overlap and noise bandwidth"_test = [] {
        const gr::property_map psdSettings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{4U}}, {"sample_rate", kSampleRate}};
        const gr::property_map rowSettings{{"fft_size", gr::Size_t{kFft}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}};

        std::vector<gr::DataSet<float>> everything;
        for (const auto& record : collect<WelchPsd<CF>, CF>(psdSettings, tone(kFft * 12UZ, 32., kFft), 4096UZ)) {
            everything.push_back(record);
        }
        std::vector<float> real(kFft * 12UZ);
        for (std::size_t k = 0UZ; k < real.size(); ++k) {
            real[k] = std::cos(2.f * std::numbers::pi_v<float> * 32.f * static_cast<float>(k) / static_cast<float>(kFft));
        }
        for (const auto& record : collect<WelchPsd<float>, float>(psdSettings, real, 4096UZ)) {
            everything.push_back(record);
        }
        for (const auto& record : collect<Spectrogram<CF>, CF>(rowSettings, tone(kFft * 8UZ, 32., kFft), 4096UZ)) {
            everything.push_back(record);
        }
        expect(everything.size() >= 8UZ) << "all three configurations produce records";

        for (std::size_t r = 0UZ; r < everything.size(); ++r) {
            const auto& ds     = everything[r];
            const char* reason = admissionFailure(ds);
            expect(reason == nullptr) << "record " << r << " would be rejected as " << (reason == nullptr ? "" : reason);

            expect(metaNumber(ds, "sample_rate") > 0.) << "record " << r << " states no rate";
            expect(metaNumber(ds, "sample_start") >= 0.) << "record " << r << " states no place in the stream";
            expect(metaNumber(ds, "n_averaged") >= 1.) << "record " << r << " states no segment count";
            expect(metaNumber(ds, "overlap") >= 0. && metaNumber(ds, "overlap") < 1.) << "record " << r << " states no segment overlap";
            expect(metaNumber(ds, "enbw_bins") > 0.) << "record " << r << " states no noise bandwidth";

            const auto& axis = ds.axis_values[0UZ];
            expect(eq(axis.size(), ds.signal_values.size())) << "record " << r << " has an axis of a different length from its values";
            const float step = axis[1UZ] - axis[0UZ];
            for (std::size_t k = 1UZ; k < axis.size(); ++k) {
                expect(std::abs((axis[k] - axis[k - 1UZ]) - step) <= 1e-6f * std::abs(step)) << "record " << r << " axis is not uniform at bin " << k;
            }
        }
    };

    "chunk independence holds for the spectrogram and for a real input"_test = [] {
        const gr::property_map rowSettings{{"fft_size", gr::Size_t{kFft}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}};
        const auto             samples = tone(kFft * 8UZ, 32., kFft);
        const auto             wideRow = collect<Spectrogram<CF>, CF>(rowSettings, samples, 4096UZ);
        const auto             thinRow = collect<Spectrogram<CF>, CF>(rowSettings, samples, 37UZ);

        expect(!wideRow.empty());
        expect(eq(wideRow.size(), thinRow.size())) << "the row count does not depend on the chunking";
        for (std::size_t r = 0UZ; r < std::min(wideRow.size(), thinRow.size()); ++r) {
            expect(eq(metaNumber(wideRow[r], "sample_start"), metaNumber(thinRow[r], "sample_start"))) << "row " << r << " moved on the time axis";
            for (std::size_t k = 0UZ; k < wideRow[r].signal_values.size(); ++k) {
                expect(wideRow[r].signal_values[k] == thinRow[r].signal_values[k]) << "row " << r << " bin " << k << " differs between chunkings";
            }
        }

        const gr::property_map psdSettings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"sample_rate", kSampleRate}};
        std::vector<float>     real(kFft * 8UZ);
        for (std::size_t k = 0UZ; k < real.size(); ++k) {
            real[k] = std::cos(2.f * std::numbers::pi_v<float> * 32.f * static_cast<float>(k) / static_cast<float>(kFft));
        }
        const auto wideReal = collect<WelchPsd<float>, float>(psdSettings, real, 4096UZ);
        const auto thinReal = collect<WelchPsd<float>, float>(psdSettings, real, 37UZ);

        expect(!wideReal.empty());
        expect(eq(wideReal.size(), thinReal.size())) << "a real stream's record count does not depend on its chunking either";
        for (std::size_t r = 0UZ; r < std::min(wideReal.size(), thinReal.size()); ++r) {
            expect(eq(metaNumber(wideReal[r], "sample_start"), metaNumber(thinReal[r], "sample_start")));
            for (std::size_t k = 0UZ; k < wideReal[r].signal_values.size(); ++k) {
                expect(wideReal[r].signal_values[k] == thinReal[r].signal_values[k]) << "real record " << r << " bin " << k << " differs between chunkings";
            }
        }
    };

    "max_hold keeps the largest density each bin reached, never the average"_test = [] {
        // Four tone segments and four silent ones, at overlap 0. Every tone segment holds the same samples. The hold
        // keeps one tone segment's density, and the mean halves it.
        std::vector<CF> samples = tone(kFft * 8UZ, 32., kFft);
        std::fill(samples.begin() + static_cast<std::ptrdiff_t>(kFft * 4UZ), samples.end(), CF{});

        const gr::property_map meanSettings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{8U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}, {"mode", std::string("mean")}};
        const gr::property_map holdSettings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{8U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}, {"mode", std::string("max_hold")}};
        const gr::property_map oneSettings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};

        const auto meanRecords = collect<WelchPsd<CF>, CF>(meanSettings, samples, 4096UZ);
        const auto holdRecords = collect<WelchPsd<CF>, CF>(holdSettings, samples, 4096UZ);
        const auto oneRecords  = collect<WelchPsd<CF>, CF>(oneSettings, samples, 4096UZ);
        expect(meanRecords.size() == 1UZ && holdRecords.size() == 1UZ && !oneRecords.empty()) << fatal;

        const float meanPeak = *std::ranges::max_element(meanRecords.front().signal_values);
        const float holdPeak = *std::ranges::max_element(holdRecords.front().signal_values);
        const float onePeak  = *std::ranges::max_element(oneRecords.front().signal_values);
        expect(eq(holdPeak, onePeak)) << "the hold's peak is one tone segment's peak";
        expect(std::abs(meanPeak / holdPeak - 0.5f) < 1e-5f) << "and the mean over four tone and four silent segments is half of it, measured " << meanPeak / holdPeak;
    };

    "center_frequency reaches the record and restarts nothing"_test = [] {
        constexpr double       kCenter = 435.5e6;
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{4U}}, {"sample_rate", kSampleRate}, {"center_frequency", kCenter}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kFft * 8UZ, 32., kFft), 4096UZ);
        expect(!records.empty());
        if (records.empty()) {
            return;
        }
        expect(eq(metaNumber(records.front(), "center_frequency"), kCenter)) << "the record carries the tuning beside sample_rate";
        expect(approx(records.front().axis_values[0UZ].front(), -kSampleRate / 2.f, 1.f)) << "and the axis itself stays at baseband";

        const auto rows = collect<Spectrogram<CF>, CF>({{"fft_size", gr::Size_t{kFft}}, {"sample_rate", kSampleRate}, {"center_frequency", kCenter}}, tone(kFft * 4UZ, 32., kFft), 4096UZ);
        expect(!rows.empty());
        if (!rows.empty()) {
            expect(eq(metaNumber(rows.front(), "center_frequency"), kCenter)) << "and so does a spectrogram row";
        }

        WelchPsd<CF> block({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{8U}}, {"sample_rate", kSampleRate}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
        block._core.pending.assign(60UZ, CF{1.f, 0.f});
        block._core.streamAt = 9000ULL;
        block._core.segments = 5UZ;

        const auto live = [](auto& target, gr::property_map changes) {
            std::ignore = target.settings().set(std::move(changes));
            std::ignore = target.settings().activateContext();
            std::ignore = target.settings().applyStagedParameters();
        };
        expect(nothrow([&] { live(block, {{"center_frequency", 88.5e6}}); })) << "a running block retunes";
        expect(eq(block._accepted.centerFrequency, 88.5e6)) << "and the new tuning is in force";
        expect(eq(block._core.pending.size(), 60UZ)) << "with every buffered sample kept";
        expect(eq(block._core.gridStart, std::uint64_t{0ULL})) << "and the grid where it was";
        expect(eq(block._core.segments, 5UZ)) << "a retune alone keeps the average in progress";

        expect(throws([&] { live(block, {{"center_frequency", std::numeric_limits<double>::quiet_NaN()}}); })) << "a tuning that is not a number is refused";
    };

    "every record states the wall clock computing it cost"_test = [] {
        const auto samples = tone(kFft * 16UZ, 32., kFft);

        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{4U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, samples, 4096UZ);
        expect(records.size() >= 2UZ) << "the run makes more than one record";
        for (std::size_t r = 0UZ; r < records.size(); ++r) {
            expect(metaNumber(records[r], "compute_seconds", -1.) > 0.) << std::format("record {} states no compute cost", r);
        }

        const auto rows = collect<Spectrogram<CF>, CF>({{"fft_size", gr::Size_t{kFft}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}}, samples, 4096UZ);
        expect(rows.size() >= 2UZ) << "and so does the spectrogram's";
        for (std::size_t r = 0UZ; r < rows.size(); ++r) {
            expect(metaNumber(rows[r], "compute_seconds", -1.) > 0.) << std::format("row {} states no compute cost", r);
        }
    };

    "the stated cost is the block's own compute, and grows with the transform"_test = [] {
        // The share of the call the figure covers depends on the machine's load, and the test prints it.
        const auto check = [](std::string_view name, auto costAt) {
            constexpr std::size_t kShort = 1UZ << 10UZ;
            constexpr std::size_t kLong  = 1UZ << 20UZ;

            const auto [shortCost, shortCall] = costAt(kShort);
            const auto [longCost, longCall]   = costAt(kLong);
            std::println("{}: compute_seconds {:.6e} s at {} points ({:.2f} of the call), {:.6e} s at {} points ({:.2f} of the call)", name, shortCost, kShort, shortCost / shortCall, longCost, kLong, longCost / longCall);

            expect(shortCost > 0. && longCost > 0.) << name << ": a record that cost nothing was not computed";
            expect(shortCost <= shortCall && longCost <= longCall) << name << ": the figure is measured inside the call and cannot exceed it";
            expect(longCost > 10. * shortCost) << name << ": a transform a thousand times longer costs more";
        };

        check("WelchPsd", [](std::size_t size) { return oneRecordCost<WelchPsd<CF>>({{"fft_size", static_cast<gr::Size_t>(size)}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}}, size); });
        check("Spectrogram", [](std::size_t size) { return oneRecordCost<Spectrogram<CF>>({{"fft_size", static_cast<gr::Size_t>(size)}, {"overlap", 0.0}, {"sample_rate", kSampleRate}}, size); });
    };

    "the stated cost is consistent with an outside reading of a single-record graph"_test = [] {
        // A record's figure is a part of the wall clock of the graph run that made it.
        constexpr std::size_t kSize = 1UZ << 18UZ;

        const auto check = [](std::string_view name, std::pair<double, double> reading) {
            const auto [cost, run] = reading;
            std::println("{}: a single-record graph ran in {:.6e} s and its record states {:.6e} s, {:.3f} of the run", name, run, cost, cost / run);
            expect(cost > 0.) << name << ": the graph made a record and it states a cost";
            expect(cost <= run) << name << ": a record cannot cost more than the run that made it";
        };

        check("WelchPsd", oneRecordGraphCost<WelchPsd<CF>>({{"fft_size", static_cast<gr::Size_t>(kSize)}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}}, kSize));
        check("Spectrogram", oneRecordGraphCost<Spectrogram<CF>>({{"fft_size", static_cast<gr::Size_t>(kSize)}, {"overlap", 0.0}, {"sample_rate", kSampleRate}}, kSize));
    };

    "the input span is held to one hop, so a work call makes at most one record"_test = [] {
        // A record needs at least one hop of input, and a span of one hop completes at most one record. The cap is
        // the hop in force, from `hop` or from `overlap`.
        const auto live = [](auto& target, gr::property_map changes) {
            std::ignore = target.settings().set(std::move(changes));
            std::ignore = target.settings().activateContext();
            std::ignore = target.settings().applyStagedParameters();
        };
        const auto started = [](auto& target) {
            target.settings().init();
            std::ignore = target.settings().applyStagedParameters();
            target.start();
        };
        const auto recordsInOneCall = [](auto& target, std::size_t span) {
            const std::vector<CF>                                     samples = tone(span, 32., kFft);
            std::vector<gr::DataSet<float>>                           room(64UZ);
            gr::blocks::testing::span::InputSpan<CF>                  in{std::span<const CF>(samples)};
            gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> out{std::span<gr::DataSet<float>>(room)};
            std::ignore = target.processBulk(in, out);
            return out.count;
        };
        // Four calls at the cap. The test reads the most records one call made and the total, and a cap that made no
        // records fails on the total.
        const auto overFourCalls = [&recordsInOneCall](auto& target) {
            std::size_t most  = 0UZ;
            std::size_t total = 0UZ;
            for (std::size_t call = 0UZ; call < 4UZ; ++call) {
                const std::size_t made = recordsInOneCall(target, target.in.max_samples);
                most                   = std::max(most, made);
                total += made;
            }
            return std::pair{most, total};
        };

        WelchPsd<CF> block({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}});
        started(block);
        expect(eq(block.in.max_samples, kFft)) << "at zero overlap the hop is the whole transform";
        const auto [mostAtHop, totalAtHop] = overFourCalls(block);
        expect(eq(mostAtHop, 1UZ)) << "a span of one hop completes one record and cannot complete two";
        expect(totalAtHop >= 3UZ) << "and the calls made records";

        live(block, {{"overlap", 0.5}});
        expect(eq(block.in.max_samples, kFft / 2UZ)) << "a live overlap change moves the hop, and the cap follows it";

        live(block, {{"hop", gr::Size_t{4U * kFft}}});
        expect(eq(block.in.max_samples, 4UZ * kFft)) << "a stated hop above the transform is the cap";
        const auto [mostAtGap, totalAtGap] = overFourCalls(block);
        expect(eq(mostAtGap, 1UZ)) << "a hop that leaves a gap still gives one record a call";
        expect(totalAtGap >= 3UZ) << "and those calls made records too";

        live(block, {{"one_record_per_call", false}});
        expect(eq(block.in.max_samples, std::numeric_limits<std::size_t>::max())) << "turning the cap off releases the span";

        WelchPsd<CF> uncapped({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}, {"one_record_per_call", false}});
        started(uncapped);
        expect(recordsInOneCall(uncapped, 8UZ * kFft) > 1UZ) << "an uncapped call makes as many records as its span and the output allow";

        Spectrogram<CF> rows({{"fft_size", gr::Size_t{kFft}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}});
        started(rows);
        expect(eq(rows.in.max_samples, kFft)) << "the spectrogram takes the same cap";
        const auto [mostRows, totalRows] = overFourCalls(rows);
        expect(eq(mostRows, 1UZ)) << "and a call makes at most one row";
        expect(totalRows >= 3UZ) << "and the rows are made";
    };

    "what the input cap costs at a small hop and at a large one"_test = [] {
        constexpr std::size_t kSmallStream = 1UZ << 19UZ;
        constexpr std::size_t kLargeStream = 8UZ << 20UZ;

        const auto smallHop = [](bool capped) { return recordRate<WelchPsd<CF>>({{"fft_size", gr::Size_t{512U}}, {"n_averages", gr::Size_t{1U}}, {"hop", gr::Size_t{256U}}, {"sample_rate", kSampleRate}, {"one_record_per_call", capped}}, kSmallStream); };
        const auto largeHop = [](bool capped) { return recordRate<WelchPsd<CF>>({{"fft_size", gr::Size_t{8192U}}, {"n_averages", gr::Size_t{1U}}, {"hop", gr::Size_t{1U << 20U}}, {"sample_rate", kSampleRate}, {"one_record_per_call", capped}}, kLargeStream); };

        // The two arms alternate, and the best rate of each is kept.
        const auto compare = [](std::string_view name, auto shape) {
            double      on       = 0.;
            double      off      = 0.;
            std::size_t onCount  = 0UZ;
            std::size_t offCount = 0UZ;
            for (std::size_t rep = 0UZ; rep < 3UZ; ++rep) {
                const auto [onRate, onMade]   = shape(true);
                const auto [offRate, offMade] = shape(false);
                on                            = std::max(on, onRate);
                off                           = std::max(off, offRate);
                onCount                       = onMade;
                offCount                      = offMade;
            }
            std::println("input cap, {}: {:.1f} records/s capped, {:.1f} uncapped, best of three alternating runs, {} records a run", name, on, off, onCount);
            expect(eq(onCount, offCount)) << name << ": the cap changes the pacing of the records and not which records are made";
        };

        compare("hop 256 at 512 points", smallHop);
        compare("hop 1048576 at 8192 points", largeHop);
    };

    "the record the last sample completes leaves behind a consumer slower than the block"_test = [] {
        // The sink takes one record on every second or third call, and the record edge fills before the stream ends.
        // Odd segment counts end with 100 samples that complete no segment.
        const gr::property_map rows{{"fft_size", gr::Size_t{kFft}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};
        const gr::property_map single{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};
        const gr::property_map pairs{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};
        for (const std::size_t period : {2UZ, 3UZ}) {
            for (std::size_t segments = 300UZ; segments < 310UZ; ++segments) {
                const std::size_t length  = segments * kFft + (segments % 2UZ) * 100UZ;
                const auto        samples = tone(length, 32., kFft);
                expect(eq(recordsBehindSlowSink<Spectrogram<CF>>(rows, samples, period), segments)) << "Spectrogram rows for " << length << " samples, sink period " << period;
                expect(eq(recordsBehindSlowSink<WelchPsd<CF>>(single, samples, period), segments)) << "WelchPsd records of one segment for " << length << " samples, sink period " << period;
                expect(eq(recordsBehindSlowSink<WelchPsd<CF>>(pairs, samples, period), (segments + 1UZ) / 2UZ)) << "WelchPsd records of two segments for " << length << " samples, sink period " << period;
            }
        }
    };

    "the partial average leaves at the end of the stream while the record edge is full"_test = [] {
        // The edge holds one record and the consumer empties it on every fifth call. The epilogue runs once, with the
        // edge's free room as its output.
        for (const std::size_t segments : {5UZ, 6UZ, 7UZ, 8UZ, 9UZ, 13UZ}) {
            WelchPsd<CF> block({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{4U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}});
            startBlock(block);
            const auto records = driveToEnd(block, tone(segments * kFft, 32., kFft), 1UZ, 5UZ);
            expect(eq(records.size(), (segments + 3UZ) / 4UZ)) << segments << " segments make every whole record and the partial one";
            if (!records.empty()) {
                const double last = segments % 4UZ == 0UZ ? 4. : static_cast<double>(segments % 4UZ);
                expect(eq(metaNumber(records.back(), "n_averaged"), last)) << "the last record counts its own segments, " << segments << " segments";
            }
        }
    };

    "a NaN sample makes its record NaN in both modes and at any segment, and the next record is clean"_test = [] {
        for (const std::string& mode : {std::string("mean"), std::string("max_hold")}) {
            for (const std::size_t segment : {0UZ, 2UZ}) {
                WelchPsd<CF> block({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{4U}}, {"overlap", 0.0}, {"mode", mode}, {"sample_rate", kSampleRate}, {"one_record_per_call", false}});
                startBlock(block);
                auto samples                   = tone(kFft * 8UZ, 32., kFft);
                samples[segment * kFft + 10UZ] = CF(std::numeric_limits<float>::quiet_NaN(), 0.f);
                const auto [records, consumed] = workCall(block, std::span<const CF>(samples), 4UZ);
                expect(eq(records.size(), 2UZ)) << fatal;
                expect(std::ranges::all_of(records[0UZ].signal_values, [](float v) { return std::isnan(v); })) << mode << ": a NaN in segment " << segment << " makes every bin of its record NaN";
                expect(std::ranges::none_of(records[1UZ].signal_values, [](float v) { return std::isnan(v); })) << mode << ": and the next record holds no NaN";
            }
        }
    };

    "an fft_size change inside a gap starts the new grid at once"_test = [] {
        // At hop 1024 and 256 points the stream is 44 samples into a gap of 768 when the length changes.
        WelchPsd<CF> block({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"hop", gr::Size_t{1024U}}, {"sample_rate", kSampleRate}, {"one_record_per_call", false}});
        startBlock(block);
        const auto first           = tone(300UZ, 32., kFft);
        const auto [before, taken] = workCall(block, std::span<const CF>(first), 4UZ);
        expect(eq(before.size(), 1UZ));
        expect(block._core.streamAt == 300ULL && block._core.skipping > 0UZ) << "the stream position lies inside the gap";

        applyLive(block, {{"fft_size", gr::Size_t{512U}}});
        expect(eq(block._core.gridStart, std::uint64_t{300ULL})) << "the new grid starts at the stream position";
        expect(eq(block._core.skipping, 0UZ)) << "and the old gap is dropped";

        const auto second         = tone(4000UZ, 32., kFft);
        const auto [after, again] = workCall(block, std::span<const CF>(second), 8UZ);
        expect(eq(after.size(), 4UZ)) << "four segments of the new grid fit in the stream";
        for (std::size_t r = 0UZ; r < after.size(); ++r) {
            expect(eq(metaNumber(after[r], "grid_start"), 300.));
            expect(eq(metaNumber(after[r], "sample_start"), 300. + 1024. * static_cast<double>(r))) << "segment " << r << " lies on the stated grid";
        }
    };

    "at a hop of one the cap is one sample, and a capped call makes one record"_test = [] {
        WelchPsd<CF> block({{"fft_size", gr::Size_t{64U}}, {"n_averages", gr::Size_t{1U}}, {"hop", gr::Size_t{1U}}, {"sample_rate", kSampleRate}});
        startBlock(block);
        expect(eq(block.in.max_samples, 1UZ)) << "the cap is the hop";

        const auto samples          = tone(65UZ, 3., 64UZ);
        const auto [blocked, taken] = workCall(block, std::span<const CF>(samples), 0UZ);
        expect(eq(taken, 63UZ)) << "with no output room, the sample that completes the first record stays in the input";
        const auto [one, takenOne] = workCall(block, std::span<const CF>(samples).subspan(63UZ, 2UZ), 4UZ);
        expect(eq(one.size(), 1UZ)) << "a call with room makes one record";

        // A shorter transform leaves many whole segments in the buffer, and a capped call still makes one record.
        WelchPsd<CF> shrunk({{"fft_size", gr::Size_t{128U}}, {"n_averages", gr::Size_t{1U}}, {"hop", gr::Size_t{1U}}, {"sample_rate", kSampleRate}});
        startBlock(shrunk);
        const auto buffered = tone(127UZ, 3., 64UZ);
        std::ignore         = workCall(shrunk, std::span<const CF>(buffered), 4UZ);
        applyLive(shrunk, {{"fft_size", gr::Size_t{64U}}});
        expect(shrunk._core.pending.size() > 64UZ) << "the buffer holds more than one whole segment of the new length";
        const auto [records, consumed] = workCall(shrunk, std::span<const CF>(buffered).first(1UZ), 8UZ);
        expect(eq(records.size(), 1UZ)) << "a capped call makes one record from the buffered segments";
    };

    "a refused setting changes nothing the block computes, and a later application reads only the settings it names"_test = [] {
        // Each arm refuses a batch in the middle of a record, then applies a later change. The records match a run that
        // applies the later change alone. A later change that restores the accepted value keeps the record in progress.
        const auto samples = tone(kFft * 8UZ, 32., kFft);
        const auto run     = [&samples]<typename TBlock>(gr::property_map settings, const gr::property_map& refusal, const gr::property_map& later) {
            TBlock block(std::move(settings));
            startBlock(block);
            auto [records, taken] = workCall(block, std::span<const CF>(samples).first(kFft * 3UZ), 8UZ);
            if (!refusal.empty()) {
                expect(throws([&] { applyLive(block, refusal); })) << "the batch is refused";
            }
            expect(nothrow([&] { applyLive(block, later); })) << "the later change is applied";
            auto [rest, more] = workCall(block, std::span<const CF>(samples).subspan(kFft * 3UZ), 8UZ);
            records.insert(records.end(), rest.begin(), rest.end());
            return records;
        };
        const auto same = [](const std::vector<gr::DataSet<float>>& got, const std::vector<gr::DataSet<float>>& want, std::string_view arm) {
            expect(eq(got.size(), want.size())) << arm << ": the record count";
            for (std::size_t r = 0UZ; r < std::min(got.size(), want.size()); ++r) {
                expect(got[r].signal_values == want[r].signal_values) << arm << ": record " << r << " values";
                expect(got[r].axis_values == want[r].axis_values) << arm << ": record " << r << " axis";
                expect(got[r].signal_names == want[r].signal_names) << arm << ": record " << r << " signal name";
                for (const std::string_view key : {"sample_rate", "center_frequency", "fft_size", "hop", "n_averaged", "sample_start", "window_param", "grid_start"}) {
                    expect(eq(metaNumber(got[r], key), metaNumber(want[r], key))) << arm << ": record " << r << " " << key;
                }
                expect(eq(metaString(got[r], "window"), metaString(want[r], "window"))) << arm << ": record " << r << " window";
            }
        };
        struct Arm {
            std::string_view name;
            gr::property_map refusal;
            gr::property_map later;
        };
        const gr::property_map retune{{"center_frequency", 1e6}};

        const gr::property_map welch{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}, {"one_record_per_call", false}};
        expect(eq(run.template operator()<WelchPsd<CF>>(welch, {}, retune).size(), 4UZ)) << "eight segments make four records of two";
        const std::vector<Arm> welchArms{
            {"sample_rate 0", {{"sample_rate", 0.f}}, retune},
            {"sample_rate inf", {{"sample_rate", std::numeric_limits<float>::infinity()}}, retune},
            {"fft_size 100", {{"fft_size", gr::Size_t{100U}}}, retune},
            {"window Gaussian2", {{"window", std::string("Gaussian2")}}, retune},
            {"center_frequency NaN", {{"center_frequency", std::numeric_limits<double>::quiet_NaN()}}, retune},
            {"signal_name with overlap 1", {{"signal_name", std::string("other")}, {"overlap", 1.0}}, retune},
            {"mode median", {{"mode", std::string("median")}}, retune},
            {"n_averages 0", {{"n_averages", gr::Size_t{0U}}}, retune},
            {"sample_rate 0, then restored", {{"sample_rate", 0.f}}, {{"sample_rate", kSampleRate}}},
            {"fft_size 100, then restored", {{"fft_size", gr::Size_t{100U}}}, {{"fft_size", gr::Size_t{kFft}}}},
            {"window Gaussian2, then restored", {{"window", std::string("Gaussian2")}}, {{"window", std::string("Hann")}}},
            {"n_averages 0, then restored", {{"n_averages", gr::Size_t{0U}}}, {{"n_averages", gr::Size_t{2U}}}},
        };
        for (const auto& [arm, refusal, later] : welchArms) {
            same(run.template operator()<WelchPsd<CF>>(welch, refusal, later), run.template operator()<WelchPsd<CF>>(welch, {}, later), arm);
        }

        const gr::property_map rows{{"fft_size", gr::Size_t{kFft}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}, {"one_record_per_call", false}};
        expect(eq(run.template operator()<Spectrogram<CF>>(rows, {}, retune).size(), 8UZ));
        const std::vector<Arm> rowArms{
            {"Spectrogram sample_rate 0", {{"sample_rate", 0.f}}, retune},
            {"Spectrogram window Gaussian2", {{"window", std::string("Gaussian2")}}, retune},
            {"Spectrogram window Gaussian2, then restored", {{"window", std::string("Gaussian2")}}, {{"window", std::string("Hann")}}},
        };
        for (const auto& [arm, refusal, later] : rowArms) {
            same(run.template operator()<Spectrogram<CF>>(rows, refusal, later), run.template operator()<Spectrogram<CF>>(rows, {}, later), arm);
        }
    };

    "after a refused application, the density stays on the rate and window its record states"_test = [] {
        // A refused batch, then sample_rate and window in one application, then a valid center_frequency, each applied
        // through `settings().set()`. The density integrates to the tone's unit power at the rate the record states,
        // and the records equal those of a block built with the settings they state, whether the second application
        // is accepted or refused.
        const auto samples = tone(kFft * 4UZ, 32., kFft);
        const auto check   = [&samples]<typename TBlock>(gr::property_map settings, const gr::property_map& refusal, std::string_view name) {
            TBlock block(settings);
            startBlock(block);
            expect(throws([&] { applyLive(block, refusal); })) << name << ": the first batch is refused";
            bool moved = true;
            try {
                applyLive(block, {{"sample_rate", kSampleRate}, {"window", std::string("Hamming")}});
            } catch (const gr::exception&) {
                moved = false;
            }
            expect(nothrow([&] { applyLive(block, {{"center_frequency", 1e6}}); })) << name << ": a valid tuning is applied";
            const auto [records, taken] = workCall(block, std::span<const CF>(samples), 8UZ);
            expect(!records.empty()) << fatal;

            const double      rate   = metaNumber(records.front(), "sample_rate");
            const std::string window = metaString(records.front(), "window");
            expect(eq(rate, static_cast<double>(moved ? kSampleRate : 1000.f)) && eq(window, std::string(moved ? "Hamming" : "Hann"))) << name << ": the record states the rate and window the block accepted";
            settings.insert_or_assign(std::pmr::string("sample_rate"), gr::pmt::Value(static_cast<float>(rate)));
            settings.insert_or_assign(std::pmr::string("window"), gr::pmt::Value(window));
            settings.insert_or_assign(std::pmr::string("center_frequency"), gr::pmt::Value(1e6));
            TBlock clean(std::move(settings));
            startBlock(clean);
            const auto [expected, cleanTaken] = workCall(clean, std::span<const CF>(samples), 8UZ);

            expect(eq(records.size(), expected.size())) << fatal;
            for (std::size_t r = 0UZ; r < records.size(); ++r) {
                expect(eq(metaNumber(records[r], "center_frequency"), 1e6)) << name << ": record " << r << " states the tuning";
                expect(eq(metaNumber(records[r], "enbw_bins"), metaNumber(expected[r], "enbw_bins"))) << name << ": and the noise bandwidth of the window it states";
                double integral = 0.;
                for (const float value : records[r].signal_values) {
                    integral += static_cast<double>(value);
                }
                integral *= metaNumber(records[r], "sample_rate") / metaNumber(records[r], "fft_size");
                expect(std::abs(10. * std::log10(integral)) < 0.01) << name << ": record " << r << " integrates to " << 10. * std::log10(integral) << " dB at its stated rate";
                expect(records[r].signal_values == expected[r].signal_values) << name << ": record " << r << " equals the record of a block built with the settings it states";
            }
        };
        const gr::property_map tuning{{"center_frequency", std::numeric_limits<double>::quiet_NaN()}};
        const gr::property_map rate{{"sample_rate", 0.f}};
        const gr::property_map welch{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"overlap", 0.0}, {"sample_rate", 1000.f}, {"one_record_per_call", false}};
        const gr::property_map rows{{"fft_size", gr::Size_t{kFft}}, {"overlap", 0.0}, {"sample_rate", 1000.f}, {"one_record_per_call", false}};
        check.template         operator()<WelchPsd<CF>>(welch, tuning, "WelchPsd, center_frequency NaN");
        check.template         operator()<WelchPsd<CF>>(welch, rate, "WelchPsd, sample_rate 0");
        check.template         operator()<Spectrogram<CF>>(rows, tuning, "Spectrogram, center_frequency NaN");
        check.template         operator()<Spectrogram<CF>>(rows, rate, "Spectrogram, sample_rate 0");
    };

    "after a smaller fft_size, each segment buffered at the end of the stream leaves in a record or is counted"_test = [] {
        // 1000 samples buffered at 1024 points make 30 whole segments at 64 points and a hop of 32. The epilogue runs
        // once, with the free room of the record edge as its output.
        constexpr std::size_t kWhole  = 30UZ;
        const auto            samples = tone(1000UZ, 3., 64UZ);
        const auto            drive   = [&samples]<typename TBlock>(gr::property_map settings, std::size_t room) {
            TBlock block(std::move(settings));
            startBlock(block);
            const auto [none, taken] = workCall(block, std::span<const CF>(samples), 4UZ);
            expect(none.empty() && eq(taken, samples.size())) << "the whole input is buffered";
            applyLive(block, {{"fft_size", gr::Size_t{64U}}});

            std::vector<gr::DataSet<float>>                           out(room);
            gr::blocks::testing::span::InputSpan<CF>                  in{std::span<const CF>{}};
            gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> os{std::span<gr::DataSet<float>>(out)};
            std::ignore = block.processEpilogue(in, os);
            out.resize(os.count);
            return std::pair{std::move(out), block.nDroppedSegments()};
        };
        const auto carried = [](const std::vector<gr::DataSet<float>>& records) {
            double segments = 0.;
            for (const auto& record : records) {
                segments += metaNumber(record, "n_averaged");
            }
            return static_cast<std::uint64_t>(segments);
        };

        for (const std::size_t room : {0UZ, 1UZ, 4UZ, 64UZ}) {
            const auto [rows, rowsDropped] = drive.template operator()<Spectrogram<CF>>({{"fft_size", gr::Size_t{1024U}}, {"sample_rate", kSampleRate}}, room);
            expect(eq(rows.size(), std::min(room, kWhole))) << "Spectrogram emits a row for each free slot, room " << room;
            expect(eq(carried(rows) + rowsDropped, std::uint64_t{kWhole})) << "and counts every row it cannot emit, room " << room;

            const auto [single, singleDropped] = drive.template operator()<WelchPsd<CF>>({{"fft_size", gr::Size_t{1024U}}, {"n_averages", gr::Size_t{1U}}, {"sample_rate", kSampleRate}}, room);
            expect(eq(single.size(), std::min(room, kWhole))) << "WelchPsd at one average, room " << room;
            expect(eq(carried(single) + singleDropped, std::uint64_t{kWhole})) << "WelchPsd at one average counts the rest, room " << room;

            const auto [fours, foursDropped] = drive.template operator()<WelchPsd<CF>>({{"fft_size", gr::Size_t{1024U}}, {"n_averages", gr::Size_t{4U}}, {"sample_rate", kSampleRate}}, room);
            expect(eq(fours.size(), std::min(room, 8UZ))) << "WelchPsd at four averages makes seven records and the partial one, room " << room;
            expect(eq(carried(fours) + foursDropped, std::uint64_t{kWhole})) << "and counts the segments of the records it cannot emit, room " << room;
        }

        // A stop request hands the epilogue the trailing input, here three whole segments with room for one row.
        Spectrogram<CF> stopped({{"fft_size", gr::Size_t{kFft}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}});
        startBlock(stopped);
        const auto                                                trailing = tone(kFft * 3UZ, 32., kFft);
        std::vector<gr::DataSet<float>>                           one(1UZ);
        gr::blocks::testing::span::InputSpan<CF>                  in{std::span<const CF>(trailing)};
        gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> os{std::span<gr::DataSet<float>>(one)};
        std::ignore = stopped.processEpilogue(in, os);
        expect(eq(os.count, 1UZ) && eq(stopped.nDroppedSegments(), std::uint64_t{2ULL})) << "the two rows of trailing input without a slot are counted";
        stopped.start();
        expect(eq(stopped.nDroppedSegments(), std::uint64_t{0ULL})) << "and a restart clears the count";
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
