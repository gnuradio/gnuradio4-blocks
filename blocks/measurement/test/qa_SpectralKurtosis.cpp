#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
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
#include <gnuradio-4.0/algorithm/measurement/Kurtosis.hpp>
#include <gnuradio-4.0/measurement/SpectralEstimate.hpp>
#include <gnuradio-4.0/measurement/SpectralKurtosis.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_sk {

using gr::blocks::measurement::SpectralKurtosis;
using gr::blocks::measurement::WelchPsd;
namespace shim = gr::blocks::testing::span;

/// @brief Seeded uniform, exponential, Gamma and Gaussian draws from xorshift64.
struct Rng {
    std::uint64_t state;

    explicit Rng(std::uint64_t seed = 0xd1b54a32d192ed03ULL) : state(seed) {}

    [[nodiscard]] double uniform() {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return (static_cast<double>(state >> 11U) + 0.5) / static_cast<double>(1ULL << 53U);
    }

    /// @brief One Exponential(1) draw.
    [[nodiscard]] double exponential() { return -std::log(uniform()); }

    /// @brief One Gamma(d, 1) draw for integer d, as the sum of d independent Exponential(1) draws.
    [[nodiscard]] double gammaInt(std::size_t d) {
        double total = 0.;
        for (std::size_t k = 0UZ; k < d; ++k) {
            total += exponential();
        }
        return total;
    }

    /// @brief One standard-normal draw, Box-Muller.
    [[nodiscard]] double normal() {
        const double radius = std::sqrt(-2. * std::log(uniform()));
        const double angle  = 2. * std::numbers::pi * uniform();
        return radius * std::cos(angle);
    }

    /// @brief `|A + n|^2` with `n` circular complex Gaussian of total power `noisePower`: one periodogram bin of a
    /// carrier of amplitude `A` in noise.
    [[nodiscard]] double cwBin(double amplitude, double noisePower) {
        const double sigma = std::sqrt(noisePower / 2.);
        const double re    = amplitude + sigma * normal();
        const double im    = sigma * normal();
        return re * re + im * im;
    }
};

/// @brief Emits a fixed list of records, `perCall` at a time, then ends the stream.
struct RecordSource : gr::Block<RecordSource> {
    gr::PortOut<gr::DataSet<float>, gr::Async> out;

    std::vector<gr::DataSet<float>> records{};
    std::size_t                     perCall = 4096UZ;
    std::size_t                     at      = 0UZ;

    GR_MAKE_REFLECTABLE(RecordSource, out);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (at >= records.size()) {
            outSpan.publish(0UZ);
            this->requestStop();
            return gr::work::Status::DONE;
        }
        const std::size_t take = std::min({perCall, records.size() - at, outSpan.size()});
        for (std::size_t k = 0UZ; k < take; ++k) {
            outSpan[k] = records[at + k];
        }
        at += take;
        outSpan.publish(take);
        return gr::work::Status::OK;
    }
};

/// @brief Emits a fixed list of samples, at most 1000 a call, then ends the stream.
struct SampleSource : gr::Block<SampleSource> {
    gr::PortOut<float> out;

    std::vector<float> samples{};
    std::size_t        at = 0UZ;

    GR_MAKE_REFLECTABLE(SampleSource, out);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (at >= samples.size()) {
            outSpan.publish(0UZ);
            this->requestStop();
            return gr::work::Status::DONE;
        }
        const std::size_t take = std::min({1000UZ, samples.size() - at, outSpan.size()});
        std::copy_n(samples.begin() + static_cast<std::ptrdiff_t>(at), take, outSpan.begin());
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

/// @brief A density record of `values.size()` bins with five of the metadata keys `WelchPsd` writes, in its types.
[[nodiscard]] gr::DataSet<float> spectrum(std::vector<float> values, std::uint64_t nAveraged, double overlap, std::uint64_t sampleStart = 0ULL) {
    gr::DataSet<float> ds;
    const std::size_t  n = values.size();
    ds.extents           = {static_cast<std::int32_t>(n)};
    ds.layout            = gr::LayoutRight{};
    ds.axis_names        = {"Frequency"};
    ds.axis_units        = {"Hz"};
    ds.axis_values.resize(1UZ);
    ds.axis_values[0UZ].resize(n);
    for (std::size_t k = 0UZ; k < n; ++k) {
        ds.axis_values[0UZ][k] = static_cast<float>(k) - static_cast<float>(n / 2UZ);
    }
    ds.signal_names      = {"psd"};
    ds.signal_quantities = {"PowerSpectralDensity"};
    ds.signal_units      = {"1/Hz"};
    ds.signal_values     = std::move(values);
    ds.signal_ranges.resize(1UZ);
    ds.meta_information.resize(1UZ);
    ds.meta_information[0UZ] = gr::property_map{
        {std::pmr::string("sample_rate"), gr::pmt::Value(48000.f)},
        {std::pmr::string("sample_start"), gr::pmt::Value(sampleStart)},
        {std::pmr::string("n_averaged"), gr::pmt::Value(nAveraged)},
        {std::pmr::string("overlap"), gr::pmt::Value(overlap)},
        {std::pmr::string("window"), gr::pmt::Value(std::string("Hann"))},
    };
    ds.timing_events.resize(1UZ);
    return ds;
}

/// @brief The record with the metadata key @p key removed.
[[nodiscard]] gr::DataSet<float> withoutMeta(gr::DataSet<float> record, std::string_view key) {
    if (!record.meta_information.empty()) {
        record.meta_information[0UZ].erase(std::pmr::string(key));
    }
    return record;
}

/// @brief @p count copies of one record.
[[nodiscard]] std::vector<gr::DataSet<float>> repeated(const gr::DataSet<float>& record, std::size_t count) { return std::vector<gr::DataSet<float>>(count, record); }

/// @brief One run's records and the block's counters.
struct Run {
    std::vector<gr::DataSet<float>> records;

    std::uint64_t nRecords{};
    std::uint64_t nRefusedRecords{};
    std::uint64_t nDegenerateBins{};
    std::uint64_t worstBin{};
    double        worstBinSk{};
    double        coverage{};
};

/// @brief Runs `records` through a source, `SpectralKurtosis` and a record sink. Returns the records and the
/// counters.
[[nodiscard]] Run runWith(gr::property_map settings, const std::vector<gr::DataSet<float>>& records, std::size_t perCall = 4096UZ) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<RecordSource>();
    auto&     block  = graph.emplaceBlock<SpectralKurtosis>(std::move(settings));
    auto&     sink   = graph.emplaceBlock<RecordSink>();
    source.records   = records;
    source.perCall   = perCall;

    if (!graph.connect<"out", "spectra">(source, block).has_value() || !graph.connect<"measurements", "in">(block, sink).has_value()) {
        return {};
    }
    gr::scheduler::Simple scheduler;
    if (!scheduler.exchange(std::move(graph)).has_value()) {
        return {};
    }
    std::ignore = scheduler.runAndWait(); // the source ends the stream

    Run result;
    result.records         = std::move(sink.records);
    result.nRecords        = block.nRecords();
    result.nRefusedRecords = block.nRefusedRecords();
    result.nDegenerateBins = block.nDegenerateBins();
    result.worstBin        = block.worstBin();
    result.worstBinSk      = block.worstBinSk();
    result.coverage        = block.coverage();
    return result;
}

[[nodiscard]] std::vector<gr::DataSet<float>> collect(gr::property_map settings, const std::vector<gr::DataSet<float>>& records, std::size_t perCall = 4096UZ) { return runWith(std::move(settings), records, perCall).records; }

/// @brief Applies staged settings and starts the block.
template<typename TBlock>
void startBlock(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

/// @brief Stages @p settings and applies them. Returns the settings the limits refused.
[[nodiscard]] gr::property_map refusedByLimits(SpectralKurtosis& block, gr::property_map settings) {
    std::ignore = block.settings().setStaged(std::move(settings));
    return block.settings().applyStagedParameters().failedParameters;
}

/// @brief The value `settings().get()` returns for `n_spectra`, or 0 when it holds none.
[[nodiscard]] gr::Size_t reportedSpectra(SpectralKurtosis& block) {
    const std::optional<gr::pmt::Value> reported = block.settings().get("n_spectra");
    return reported.has_value() && reported->get_if<gr::Size_t>() != nullptr ? *reported->get_if<gr::Size_t>() : gr::Size_t{0U};
}

/// @brief One work call over @p rows. Returns the records it emits.
[[nodiscard]] std::vector<gr::DataSet<float>> feed(SpectralKurtosis& block, std::span<const gr::DataSet<float>> rows) {
    std::vector<gr::DataSet<float>>      out(rows.size() + 1UZ);
    shim::InputSpan<gr::DataSet<float>>  inSpan{rows};
    shim::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(out)};
    std::ignore = block.processBulk(inSpan, outSpan);
    out.resize(outSpan.count);
    return out;
}

/// @brief The record with the metadata key @p key set to @p value.
template<typename V>
[[nodiscard]] gr::DataSet<float> withMeta(gr::DataSet<float> record, std::string_view key, V value) {
    record.meta_information[0UZ].insert_or_assign(std::pmr::string(key), gr::pmt::Value(value));
    return record;
}

/// @brief Eight two-bin records at `center_frequency` 1.42e9, one full reading at `n_spectra` 8. Bin 0 is constant,
/// and its SK is 0. Bin 1 holds one pulse: S1 = 1 and S2 = 1, M*S2/S1^2 = 8, and SK = (9/7)*(8-1) = 9. Bin 1 departs
/// further from 1.
[[nodiscard]] std::vector<gr::DataSet<float>> onePulseRows() {
    std::vector<gr::DataSet<float>> rows;
    for (std::size_t k = 0UZ; k < 8UZ; ++k) {
        rows.push_back(withMeta(spectrum({1.f, k == 0UZ ? 1.f : 0.f}, 1ULL, 0.0), "center_frequency", 1.42e9));
    }
    return rows;
}

/// @brief The large-`M` limit `L` of `SK` and its first-order bias coefficient `b1`, `E[SK] = L + b1/M`, at `d = 1`
/// for a carrier of bin power `S` present a fraction `p` of the time over noise of unit power.
/// The mixture's raw moments are `m1`, `m2` and `m3`, with `v = m2 - m1^2`. The delta method on `M*S2/S1^2` gives
/// `b1 = 2L + 3 m2 v/m1^4 - 2 (m3 - m1 m2)/m1^3`. On noise alone `b1` is zero.
struct PulsedLimit {
    double limit{0.};
    double bias{0.};
};

[[nodiscard]] constexpr PulsedLimit pulsedLimit(double p, double s) noexcept {
    const double m1 = p * (s + 1.) + (1. - p);
    const double m2 = p * (s * s + 4. * s + 2.) + (1. - p) * 2.;
    const double m3 = p * (s * s * s + 9. * s * s + 18. * s + 6.) + (1. - p) * 6.;
    const double v  = m2 - m1 * m1;
    const double l  = m2 / (m1 * m1) - 1.;
    return {.limit = l, .bias = 2. * l + 3. * m2 * v / (m1 * m1 * m1 * m1) - 2. * (m3 - m1 * m2) / (m1 * m1 * m1)};
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
    if (const auto* asBool = it->second.template get_if<bool>()) {
        return *asBool ? 1. : 0.;
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
    const auto& map = record.meta_information[0UZ];
    const auto  it  = map.find(std::pmr::string(key));
    if (it == map.end()) {
        return {};
    }
    if (const auto* asString = it->second.template get_if<std::pmr::string>()) {
        return std::string(std::string_view(*asString));
    }
    return {};
}

/// @brief `m` records of `nBins` independent Gamma(d) draws, the distribution of a noise bin summed over `d`
/// periodograms of circular complex Gaussian noise.
[[nodiscard]] std::vector<gr::DataSet<float>> noiseRows(Rng& rng, std::size_t m, std::size_t nBins, std::size_t d) {
    std::vector<gr::DataSet<float>> rows;
    rows.reserve(m);
    for (std::size_t row = 0UZ; row < m; ++row) {
        std::vector<float> values(nBins);
        for (float& v : values) {
            v = static_cast<float>(rng.gammaInt(d));
        }
        rows.push_back(spectrum(std::move(values), static_cast<std::uint64_t>(d), 0.0));
    }
    return rows;
}

[[nodiscard]] double mean(const std::vector<float>& values) {
    double total = 0.;
    for (const float v : values) {
        total += static_cast<double>(v);
    }
    return values.empty() ? 0. : total / static_cast<double>(values.size());
}

[[nodiscard]] double sampleStdDev(const std::vector<float>& values, double meanValue) {
    if (values.size() < 2UZ) {
        return 0.;
    }
    double sumSq = 0.;
    for (const float v : values) {
        const double d = static_cast<double>(v) - meanValue;
        sumSq += d * d;
    }
    return std::sqrt(sumSq / static_cast<double>(values.size() - 1UZ));
}

} // namespace qa_sk

const boost::ut::suite<"SpectralKurtosis"> spectralKurtosisTests = [] {
    using namespace boost::ut;
    using namespace qa_sk;

    // SK reads 1 on noise at d = 1, 4 and 16. Each bin of one accumulation is an independent reading. The mean over
    // the bins lies within four standard errors of 1, and their spread lies within 15% of the closed form.
    "spectral kurtosis reads 1 on noise, with the closed-form spread"_test = [] {
        constexpr std::size_t kM    = 64UZ;
        constexpr std::size_t kBins = 2048UZ;
        Rng                   rng(0x2545f4914f6cdd1dULL);

        // The closed-form spreads at M = 64 are literals: 0.24251, 0.19728 and 0.18321 at d = 1, 4 and 16.
        struct Shape {
            std::size_t d;
            double      spread;
        };
        const std::array<Shape, 3UZ> shapes{{{1UZ, 0.24251}, {4UZ, 0.19728}, {16UZ, 0.18321}}};

        for (const Shape& shape : shapes) {
            const auto rows    = noiseRows(rng, kM, kBins, shape.d);
            const auto records = collect({{"n_spectra", gr::Size_t{kM}}, {"shape", gr::Size_t{static_cast<gr::Size_t>(shape.d)}}}, rows, 4096UZ);
            expect(eq(records.size(), 1UZ)) << std::format("d={}", shape.d);
            if (records.empty()) {
                continue;
            }
            const auto&  values     = records.front().signal_values;
            const double measured   = mean(values);
            const double measuredSd = sampleStdDev(values, measured);
            const double sem        = 4. * shape.spread / std::sqrt(static_cast<double>(kBins));

            std::println("noise: d={} mean {:.5f} (band 1 +- {:.5f}), spread {:.5f} (closed form {:.5f})", shape.d, measured, sem, measuredSd, shape.spread);
            expect(std::abs(measured - 1.) < sem) << std::format("d={}: mean SK too far from 1", shape.d);
            expect(std::abs(measuredSd - shape.spread) < 0.15 * shape.spread) << std::format("d={}: spread too far from the closed-form value", shape.d);

            // The record states the closed-form spread and the expectation.
            std::println("noise: d={} record sk_std {:.5f}, sk_expectation {:.5f}", shape.d, metaNumber(records.front(), "sk_std"), metaNumber(records.front(), "sk_expectation"));
            expect(std::abs(metaNumber(records.front(), "sk_std") - shape.spread) < 5e-5) << std::format("d={}: the record's sk_std must be the closed-form spread", shape.d);
            expect(eq(metaNumber(records.front(), "sk_expectation"), 1.)) << "sk_expectation is exactly 1, at every M and every d";
        }
    };

    // A noiseless tone gives identical bin powers and SK = 0 at every M. The comparison is exact.
    "a noiseless tone reads SK = 0 exactly, at every M"_test = [] {
        for (const std::size_t m : {8UZ, 64UZ, 1024UZ}) {
            std::vector<gr::DataSet<float>> rows;
            for (std::size_t row = 0UZ; row < m; ++row) {
                rows.push_back(spectrum(std::vector<float>{3.5f, 3.5f, 3.5f, 3.5f}, 1ULL, 0.0));
            }
            const auto records = collect({{"n_spectra", gr::Size_t{static_cast<gr::Size_t>(m)}}, {"shape", gr::Size_t{1U}}}, rows, 4096UZ);
            expect(eq(records.size(), 1UZ)) << std::format("M={}", m);
            if (!records.empty()) {
                for (const float v : records.front().signal_values) {
                    expect(eq(v, 0.f)) << std::format("M={}: identical bin powers must give SK exactly 0", m);
                }
            }
        }
    };

    // A carrier in noise at bin ratios rho = 1, 2, 10 and 100 reads its closed form within 0.01 at M = 1024. A pulsed
    // signal reads its large-M limit with the first-order bias, within four standard errors of the bin mean.
    "a carrier in noise reads its closed form, and a pulsed signal reads its limit with the first-order bias"_test = [] {
        constexpr std::size_t kM    = 1024UZ;
        constexpr std::size_t kBins = 400UZ;
        Rng                   rng(0x9e3779b97f4a7c15ULL);

        const auto cwSk = [](double rho) { return (2. * rho + 1.) / ((rho + 1.) * (rho + 1.)); };
        for (const double rho : {1., 2., 10., 100.}) {
            std::vector<gr::DataSet<float>> rows;
            const double                    amplitude = std::sqrt(rho); // N0 = 1
            for (std::size_t row = 0UZ; row < kM; ++row) {
                std::vector<float> values(kBins);
                for (float& v : values) {
                    v = static_cast<float>(rng.cwBin(amplitude, 1.));
                }
                rows.push_back(spectrum(std::move(values), 1ULL, 0.0));
            }
            const auto records = collect({{"n_spectra", gr::Size_t{kM}}, {"shape", gr::Size_t{1U}}}, rows, 4096UZ);
            expect(eq(records.size(), 1UZ)) << std::format("rho={}", rho);
            if (records.empty()) {
                continue;
            }
            const double measured = mean(records.front().signal_values);
            const double expected = cwSk(rho);
            std::println("CW rho={:g} measured {:.5f}, closed form {:.5f}", rho, measured, expected);
            expect(std::abs(measured - expected) < 0.01) << std::format("rho={}: CW arm outside 0.01 absolute", rho);
        }

        // A pulsed signal at S/N0 = 100 and p = 0.1, at M = 1024 and M = 8192, and at p = 0.25 and 0.5 at M = 8192.
        const double kSN0       = 100.;
        const double kAmp       = std::sqrt(kSN0);
        const auto   pulsedMean = [&](double p, std::size_t m) -> std::pair<double, double> {
            std::vector<gr::DataSet<float>> rows;
            for (std::size_t row = 0UZ; row < m; ++row) {
                std::vector<float> values(kBins);
                for (float& v : values) {
                    v = rng.uniform() < p ? static_cast<float>(rng.cwBin(kAmp, 1.)) : static_cast<float>(rng.gammaInt(1UZ));
                }
                rows.push_back(spectrum(std::move(values), 1ULL, 0.0));
            }
            const auto records = collect({{"n_spectra", gr::Size_t{static_cast<gr::Size_t>(m)}}, {"shape", gr::Size_t{1U}}}, rows, 4096UZ);
            if (records.empty()) {
                return {0., 0.};
            }
            const double measured = mean(records.front().signal_values);
            return {measured, sampleStdDev(records.front().signal_values, measured) / std::sqrt(static_cast<double>(kBins))};
        };

        struct Arm {
            double      p;
            std::size_t m;
        };
        for (const Arm arm : {Arm{0.1, 1024UZ}, Arm{0.1, 8192UZ}, Arm{0.25, 8192UZ}, Arm{0.5, 8192UZ}}) {
            const PulsedLimit closed             = pulsedLimit(arm.p, kSN0);
            const double      expected           = closed.limit + closed.bias / static_cast<double>(arm.m);
            const auto [measured, standardError] = pulsedMean(arm.p, arm.m);
            std::println("pulsed p={:g} M={}: measured {:.4f}, L + b1/M = {:.4f} (L {:.4f}), band +- {:.4f}", arm.p, arm.m, measured, expected, closed.limit, 4. * standardError);
            expect(std::abs(measured - expected) < 4. * standardError) << std::format("p={} M={}: outside four standard errors of L + b1/M", arm.p, arm.m);
        }
    };

    // The block folds a record only when it carries `overlap` and that value is exactly zero.
    "a record is folded only when its overlap metadata is present and zero"_test = [] {
        const gr::DataSet<float> clean = spectrum({1.f, 2.f, 3.f}, 1ULL, 0.0);

        {
            std::vector<gr::DataSet<float>> rows{spectrum({1.f, 2.f, 3.f}, 1ULL, 0.5)};
            for (std::size_t k = 0UZ; k < 8UZ; ++k) {
                rows.push_back(clean);
            }
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}}, rows);
            expect(eq(r.nRefusedRecords, std::uint64_t{1ULL})) << "the overlapping record must be refused and counted";
            expect(eq(r.records.size(), 1UZ)) << "the eight remaining records still complete the n_spectra=8 accumulation";
        }
        {
            std::vector<gr::DataSet<float>> rows{withoutMeta(clean, "overlap")};
            for (std::size_t k = 0UZ; k < 8UZ; ++k) {
                rows.push_back(clean);
            }
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}}, rows);
            expect(eq(r.nRefusedRecords, std::uint64_t{1ULL})) << "a record without overlap metadata is refused";
            expect(eq(r.records.size(), 1UZ));
        }
    };

    // The other refusals. Each is counted, and the block measures again after each.
    "the further refusals: shape, absent n_averaged, a negative bin, a changed bin count"_test = [] {
        const gr::DataSet<float> shape1 = spectrum({1.f, 2.f, 3.f, 4.f}, 1ULL, 0.0);
        const gr::DataSet<float> shape4 = spectrum({1.f, 2.f, 3.f, 4.f}, 4ULL, 0.0);

        { // a `shape` setting that disagrees with the record it receives
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{4U}}}, repeated(shape1, 8UZ));
            expect(eq(r.nRefusedRecords, std::uint64_t{8ULL})) << "every record disagreeing with the shape setting is refused";
            expect(eq(r.records.size(), 0UZ));
        }
        { // shape = 0 reads the shape from the record, and a record without `n_averaged` is refused
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}}, repeated(withoutMeta(shape1, "n_averaged"), 8UZ));
            expect(eq(r.nRefusedRecords, std::uint64_t{8ULL}));
            expect(eq(r.records.size(), 0UZ));
        }
        { // a shape change inside an accumulation ends it with a partial reading and refuses only that record
            std::vector<gr::DataSet<float>> rows = repeated(shape1, 2UZ);
            rows.push_back(shape4);
            for (const auto& row : repeated(shape4, 8UZ)) {
                rows.push_back(row);
            }
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}}, rows);
            expect(eq(r.nRefusedRecords, std::uint64_t{1ULL})) << "only the record that disagreed is refused";
            expect(eq(r.nRecords, std::uint64_t{1ULL})) << "nRecords counts the full accumulation alone";
            expect(fatal(eq(r.records.size(), 2UZ))) << "the partial reading, then the eight records at the new shape";
            expect(eq(metaNumber(r.records[0UZ], "n_spectra"), 2.)) << "the partial reading states its two spectra";
            expect(eq(metaNumber(r.records[0UZ], "shape"), 1.)) << "under the old shape";
            expect(eq(metaNumber(r.records[0UZ], "valid"), 0.)) << "and is not valid below eight spectra";
            expect(eq(metaNumber(r.records[1UZ], "shape"), 4.)) << "the full reading is measured under the new shape";
        }
        { // a negative bin
            std::vector<gr::DataSet<float>> rows{spectrum({1.f, -1.f, 3.f, 4.f}, 1ULL, 0.0)};
            for (const auto& row : repeated(shape1, 8UZ)) {
                rows.push_back(row);
            }
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}}, rows);
            expect(eq(r.nRefusedRecords, std::uint64_t{1ULL})) << "a negative bin refuses the record and folds nothing";
            expect(eq(r.records.size(), 1UZ));
        }
        { // a bin count that does not match the accumulation in progress
            std::vector<gr::DataSet<float>> rows = repeated(shape1, 2UZ);
            rows.push_back(spectrum({1.f, 2.f, 3.f}, 1ULL, 0.0));
            for (const auto& row : repeated(spectrum({1.f, 2.f, 3.f}, 1ULL, 0.0), 8UZ)) {
                rows.push_back(row);
            }
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}}, rows);
            expect(eq(r.nRefusedRecords, std::uint64_t{1ULL}));
            expect(fatal(eq(r.records.size(), 2UZ)));
            expect(eq(r.records[0UZ].signal_values.size(), 4UZ)) << "the partial reading at the old bin count";
            expect(eq(metaNumber(r.records[0UZ], "n_spectra"), 2.));
            expect(eq(r.records[1UZ].signal_values.size(), 3UZ)) << "the accumulation restarts at the new bin count";
        }
        { // a window change ends the accumulation the same way
            std::vector<gr::DataSet<float>> rows     = repeated(shape1, 3UZ);
            const gr::DataSet<float>        harris   = withMeta(shape1, "window", std::string("BlackmanHarris"));
            const std::vector               rowsTail = repeated(harris, 9UZ);
            rows.insert(rows.end(), rowsTail.begin(), rowsTail.end());
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}}, rows);
            expect(eq(r.nRefusedRecords, std::uint64_t{1ULL}));
            expect(fatal(eq(r.records.size(), 2UZ)));
            expect(eq(metaNumber(r.records[0UZ], "n_spectra"), 3.)) << "the partial reading of the three records under Hann";
            expect(eq(metaString(r.records[0UZ], "window"), std::string("Hann")));
            expect(eq(metaString(r.records[1UZ], "window"), std::string("BlackmanHarris"))) << "a full reading under the new window";
        }
        { // a rate change or a retune ends the accumulation the same way
            const gr::DataSet<float> tuned = withMeta(shape1, "center_frequency", 1.42e9);
            for (const gr::DataSet<float>& next : {withMeta(tuned, "sample_rate", 96000.f), withMeta(tuned, "center_frequency", 1.43e9)}) {
                std::vector<gr::DataSet<float>> rows     = repeated(tuned, 3UZ);
                const std::vector               rowsTail = repeated(next, 9UZ);
                rows.insert(rows.end(), rowsTail.begin(), rowsTail.end());
                const Run r = runWith({{"n_spectra", gr::Size_t{8U}}}, rows);
                expect(eq(r.nRefusedRecords, std::uint64_t{1ULL})) << "the first record after the change is refused";
                expect(fatal(eq(r.records.size(), 2UZ)));
                expect(eq(metaNumber(r.records[0UZ], "n_spectra"), 3.)) << "the partial reading of the three records before the change";
                expect(eq(metaNumber(r.records[0UZ], "sample_rate"), 48000.) && eq(metaNumber(r.records[0UZ], "center_frequency"), 1.42e9)) << "states their rate and tuning";
                expect(eq(metaNumber(r.records[1UZ], "n_spectra"), 8.)) << "a full reading follows";
                expect(eq(metaNumber(r.records[1UZ], "sample_rate"), metaNumber(next, "sample_rate")) && eq(metaNumber(r.records[1UZ], "center_frequency"), metaNumber(next, "center_frequency"))) << "at the new rate and tuning";
            }
        }
    };

    "n_spectra below 8 or above 2^20 is refused by its limits"_test = [] {
        for (const gr::Size_t value : {gr::Size_t{7U}, gr::Size_t{(1U << 20U) + 1U}}) {
            SpectralKurtosis block(gr::property_map{});
            startBlock(block);
            expect(throws([&] { std::ignore = block.settings().set({{"n_spectra", value}}); })) << "settings().set() refuses " << value << " at the call";
            expect(refusedByLimits(block, {{"n_spectra", value}}).contains("n_spectra")) << "a staged " << value << " is refused at the apply";
            expect(eq(block.n_spectra.value, gr::Size_t{32U})) << "and the member keeps the accepted value";
        }
        for (const gr::Size_t value : {gr::Size_t{8U}, gr::Size_t{1U << 20U}}) {
            SpectralKurtosis block(gr::property_map{});
            startBlock(block);
            expect(refusedByLimits(block, {{"n_spectra", value}}).empty()) << "the limit " << value << " itself is admitted";
            expect(eq(block.n_spectra.value, value));
        }
    };

    // The record copies the input axis and states its conditions. `valid` is false only when every bin is degenerate.
    "the record states its conditions, and a dead bin leaves it valid"_test = [] {
        constexpr std::size_t kM = 8UZ;

        { // one dead bin among three is counted and written 0, and the record stays valid
            const Run r = runWith({{"n_spectra", gr::Size_t{kM}}}, repeated(spectrum({1.f, 2.f, 0.f}, 1ULL, 0.0, 4242ULL), kM));
            expect(eq(r.records.size(), 1UZ));
            expect(eq(r.nDegenerateBins, std::uint64_t{1ULL})) << "the all-zero bin is counted degenerate";
            if (!r.records.empty()) {
                const auto& record = r.records.front();
                expect(eq(record.signal_values.at(2UZ), 0.f)) << "a bin with no defined ratio is written 0";
                expect(eq(metaNumber(record, "valid"), 1.)) << "a single dead bin leaves the record valid";
                expect(eq(metaNumber(record, "n_spectra"), 8.));
                expect(eq(metaNumber(record, "shape"), 1.));
                expect(eq(metaNumber(record, "sample_start"), 4242.)) << "the first input record's sample_start";
                expect(std::abs(metaNumber(record, "sample_rate") - 48000.) < 1e-3) << "the rate is copied from the input record";
                expect(eq(metaString(record, "window"), std::string("Hann"))) << "the window name is copied from the input record";
                expect(std::abs(metaNumber(record, "sk_std") - 0.5766) < 1e-4) << "the closed-form spread at M=8, d=1 is 0.5766";
                expect(eq(record.signal_names.at(0UZ), std::string("spectral_kurtosis")));
                expect(eq(record.axis_values.at(0UZ).size(), 3UZ)) << "the input's frequency axis";
            }
        }
        { // with every bin dead the record is marked invalid
            const Run r = runWith({{"n_spectra", gr::Size_t{kM}}}, repeated(spectrum({0.f, 0.f, 0.f}, 1ULL, 0.0), kM));
            expect(eq(r.records.size(), 1UZ));
            expect(eq(r.nDegenerateBins, std::uint64_t{3ULL}));
            if (!r.records.empty()) {
                expect(eq(metaNumber(r.records.front(), "valid"), 0.)) << "a record whose every bin is degenerate is invalid";
            }
        }
        { // the worst bin lies furthest from 1, on either side
            // Bin 0 alternates 0 and 1 over M = 8. S1 = 4 and S2 = 4, M*S2/S1^2 = 2, and SK = (9/7)*(2-1) = 1.2857.
            // Bin 1 is constant, and its SK is 0. Bin 1 departs further from 1.
            std::vector<gr::DataSet<float>> rows;
            for (std::size_t k = 0UZ; k < kM; ++k) {
                rows.push_back(spectrum({k % 2UZ == 0UZ ? 0.f : 1.f, 2.5f}, 1ULL, 0.0));
            }
            const Run r = runWith({{"n_spectra", gr::Size_t{kM}}}, rows);
            expect(eq(r.records.size(), 1UZ));
            std::println("record: worstBin {} worstBinSk {:.5f}, bin 0 SK {:.5f} (closed form 9/7 = {:.5f})", r.worstBin, r.worstBinSk, r.records.empty() ? 0.f : r.records.front().signal_values.at(0UZ), 9. / 7.);
            expect(eq(r.worstBin, std::uint64_t{1ULL})) << "the constant bin departs from 1 by the most";
            expect(std::abs(r.worstBinSk) < 1e-6) << "and its SK is exactly 0";
            if (!r.records.empty()) {
                expect(std::abs(static_cast<double>(r.records.front().signal_values.at(0UZ)) - 9. / 7.) < 1e-5) << "the alternating bin reads the closed 9/7";
            }
        }
    };

    "emit_records = false measures without publishing"_test = [] {
        Rng       rng(0x3c6ef372fe94f82bULL);
        const Run r = runWith({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}, {"emit_records", false}}, noiseRows(rng, 16UZ, 8UZ, 1UZ));
        expect(eq(r.records.size(), 0UZ)) << "no record is published";
        expect(eq(r.nRecords, std::uint64_t{2ULL})) << "and both accumulations are measured and counted";
        expect(eq(r.nRefusedRecords, std::uint64_t{0ULL}));
    };

    // The same input records, delivered a different number per call, give bit-identical records and counters.
    "the records and counters do not depend on how many input records arrive per call"_test = [] {
        Rng                             rng(0x1122334455667788ULL);
        std::vector<gr::DataSet<float>> rows = noiseRows(rng, 96UZ, 64UZ, 1UZ);
        // One record of another bin count, for the refusal counter. It ends the second accumulation with a partial
        // reading.
        rows.insert(rows.begin() + 40, spectrum({1.f, 2.f, 3.f}, 1ULL, 0.0));
        const auto settings  = [] { return gr::property_map{{"n_spectra", gr::Size_t{32U}}, {"shape", gr::Size_t{1U}}}; };
        const Run  reference = runWith(settings(), rows, 4096UZ);
        expect(!reference.records.empty());
        expect(eq(reference.nRefusedRecords, std::uint64_t{1ULL}));

        for (const std::size_t perCall : {1UZ, 7UZ, 1000UZ, 12345UZ}) {
            const Run candidate = runWith(settings(), rows, perCall);
            expect(eq(candidate.records.size(), reference.records.size())) << std::format("perCall {}", perCall);
            expect(eq(candidate.nRecords, reference.nRecords)) << std::format("perCall {}: nRecords", perCall);
            expect(eq(candidate.nRefusedRecords, reference.nRefusedRecords)) << std::format("perCall {}: nRefusedRecords", perCall);
            expect(eq(candidate.nDegenerateBins, reference.nDegenerateBins)) << std::format("perCall {}: nDegenerateBins", perCall);
            expect(eq(candidate.worstBin, reference.worstBin)) << std::format("perCall {}: worstBin", perCall);
            expect(eq(candidate.worstBinSk, reference.worstBinSk)) << std::format("perCall {}: worstBinSk", perCall);
            for (std::size_t r = 0UZ; r < std::min(candidate.records.size(), reference.records.size()); ++r) {
                expect(std::ranges::equal(candidate.records[r].signal_values, reference.records[r].signal_values)) << std::format("perCall {}: record {}", perCall, r);
            }
        }
    };

    // A started block takes a refused application and then more records. Its readings match a block that saw no
    // refusal, and a valid n_spectra after the refusal applies.
    "an n_spectra outside its limits on a running block changes nothing it computes, and the rest of its application applies"_test = [] {
        Rng                                       rng(0x243f6a8885a308d3ULL);
        const std::vector<gr::DataSet<float>>     rows = noiseRows(rng, 40UZ, 16UZ, 1UZ);
        const std::span<const gr::DataSet<float>> all(rows);
        const auto                                settings = [] { return gr::property_map{{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}}; };

        SpectralKurtosis reference(settings());
        startBlock(reference);
        expect(eq(feed(reference, all.first(8UZ)).size(), 1UZ));
        const auto referenceTail = feed(reference, all.subspan(8UZ));
        expect(eq(referenceTail.size(), 4UZ));

        for (const gr::Size_t value : {gr::Size_t{0U}, gr::Size_t{(1U << 20U) + 1U}}) {
            SpectralKurtosis block(settings());
            startBlock(block);
            expect(eq(feed(block, all.first(8UZ)).size(), 1UZ));
            expect(refusedByLimits(block, {{"n_spectra", value}}).contains("n_spectra")) << std::format("n_spectra {} is refused", value);
            expect(eq(block.n_spectra.value, gr::Size_t{8U})) << std::format("n_spectra {}: the member keeps the accepted value", value);
            expect(eq(reportedSpectra(block), gr::Size_t{8U})) << std::format("n_spectra {}: and settings().get() returns it", value);
            const auto tail = feed(block, all.subspan(8UZ));
            expect(eq(tail.size(), referenceTail.size())) << std::format("n_spectra {}: the same readings", value);
            for (std::size_t k = 0UZ; k < std::min(tail.size(), referenceTail.size()); ++k) {
                expect(std::ranges::equal(tail[k].signal_values, referenceTail[k].signal_values)) << std::format("n_spectra {}: reading {}", value, k);
                expect(eq(metaNumber(tail[k], "n_spectra"), 8.)) << std::format("n_spectra {}: reading {} n_spectra", value, k);
                expect(eq(metaNumber(tail[k], "sk_std"), metaNumber(referenceTail[k], "sk_std"))) << std::format("n_spectra {}: reading {} sk_std", value, k);
            }
            expect(eq(block.nRecords(), reference.nRecords())) << std::format("n_spectra {}: nRecords", value);
            expect(eq(block.nRefusedRecords(), std::uint64_t{0ULL})) << std::format("n_spectra {}: no record refused", value);
        }

        // n_spectra lies outside its limits, and emit_records applies.
        SpectralKurtosis block(settings());
        startBlock(block);
        std::ignore                    = feed(block, all.first(8UZ));
        const gr::property_map refused = refusedByLimits(block, {{"n_spectra", gr::Size_t{0U}}, {"emit_records", false}});
        expect(eq(refused.size(), 1UZ) && refused.contains("n_spectra")) << "only n_spectra is refused";
        expect(!block.emit_records.value) << "emit_records applies";
        expect(feed(block, all.subspan(8UZ, 16UZ)).empty()) << "and the block publishes no record";
        expect(eq(block.nRecords(), std::uint64_t{3ULL})) << "while it reads every 8 spectra, the accepted n_spectra";
        expect(refusedByLimits(block, {{"n_spectra", gr::Size_t{16U}}, {"emit_records", true}}).empty()) << "a valid n_spectra is accepted";
        const auto afterChange = feed(block, all.subspan(24UZ));
        expect(eq(afterChange.size(), 1UZ)) << "16 records make one reading of 16";
        for (const auto& record : afterChange) {
            expect(eq(metaNumber(record, "n_spectra"), 16.));
            expect(eq(metaNumber(record, "shape"), 1.));
        }
    };

    // Each real-input bin's shape under Hann matches the transform of the squared window, summed directly. Bins 0 and 32
    // have shape d/2, and bins 1 and 31 have d/(1 + rho^2). Every bin the list leaves out has d to float precision.
    "under Hann bins 1 and N/2 - 1 of a real-input spectrum have about 0.97 d, and the DC and Nyquist bins d/2"_test = [] {
        constexpr std::size_t    kFft   = 64UZ;
        const std::vector<float> window = gr::algorithm::window::create<float>(gr::algorithm::window::Type::Hann, kFft);
        const auto               direct = [&window](std::size_t k) {
            double re    = 0.;
            double im    = 0.;
            double power = 0.;
            for (std::size_t n = 0UZ; n < kFft; ++n) {
                const double squared = static_cast<double>(window[n]) * static_cast<double>(window[n]);
                const double phase   = 4. * std::numbers::pi * static_cast<double>(k * n) / static_cast<double>(kFft);
                re += squared * std::cos(phase);
                im -= squared * std::sin(phase);
                power += squared;
            }
            const double rho = std::hypot(re, im) / power;
            return 1. / (1. + rho * rho);
        };
        const auto          shaped = gr::blocks::measurement::detail::realInputShapedBins(std::span<const float>(window));
        std::vector<double> fractions(kFft / 2UZ + 1UZ, 1.);
        for (const auto& entry : shaped) {
            expect(fatal(entry.bin < fractions.size()));
            fractions[entry.bin] = entry.fraction;
            expect(std::abs(entry.fraction - direct(entry.bin)) < 1e-12) << std::format("bin {}: the direct sum", entry.bin);
        }
        for (std::size_t k = 0UZ; k < fractions.size(); ++k) {
            if (fractions[k] == 1.) {
                expect(1. - direct(k) <= static_cast<double>(std::numeric_limits<float>::epsilon())) << std::format("bin {} has d to float precision", k);
            }
        }
        std::println("Hann at fft_size 64: {} bins below d; bins 1 and 31 have {:.5f} d and {:.5f} d", shaped.size(), fractions[1UZ], fractions[31UZ]);
        expect(eq(fractions[0UZ], 0.5)) << "the DC bin has d/2";
        expect(eq(fractions[32UZ], 0.5)) << "the Nyquist bin has d/2";
        expect(std::abs(fractions[1UZ] - 0.97) < 0.005) << "bin 1 has about 0.97 d";
        expect(std::abs(fractions[31UZ] - 0.97) < 0.005) << "bin 31 has about 0.97 d";
    };

    // Welch estimates of real Gaussian noise at fft_size 64: 33 one-sided bins, bin 0 at DC and bin 32 at Nyquist. Under
    // a window the bins at and next to DC and Nyquist have a Gamma shape below d, by 3% at bin 1 under Hann and by 17%
    // under Blackman-Harris. Each of bins 0, 1, 2, 30, 31 and 32 reads 1 within four standard errors of the mean over
    // 200 readings, taken at the spread of shape d/2, the widest of them. The worst bin lies among those six no more
    // often than chance allows. Under chance the count is binomial with p = 6/33, and the bound is four standard
    // deviations above its mean.
    "on a one-sided spectrum of windowed real noise the bins at and next to DC and Nyquist read 1"_test = [] {
        constexpr std::size_t                kFft      = 64UZ;
        constexpr std::size_t                kBins     = kFft / 2UZ + 1UZ;
        constexpr std::size_t                kM        = 64UZ;
        constexpr std::size_t                kReadings = 200UZ;
        constexpr std::array<std::size_t, 6> kEdge{0UZ, 1UZ, 2UZ, kBins - 3UZ, kBins - 2UZ, kBins - 1UZ};

        for (const std::string_view windowName : {"Hann", "BlackmanHarris"}) {
            Rng                rng(0x6c62272e07bb0142ULL);
            std::vector<float> samples(kReadings * kM * kFft + 1UZ);
            for (float& sample : samples) {
                sample = static_cast<float>(rng.normal());
            }

            WelchPsd<float> welch({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", 1.f}, {"window", std::string(windowName)}, {"one_record_per_call", false}});
            startBlock(welch);
            std::vector<gr::DataSet<float>> spectra(kReadings * kM + 4UZ);
            {
                shim::InputSpan<float>               inSpan{std::span<const float>(samples)};
                shim::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(spectra)};
                std::ignore = welch.processBulk(inSpan, outSpan);
                spectra.resize(outSpan.count);
            }
            expect(fatal(eq(spectra.size(), kReadings * kM)));
            expect(fatal(eq(spectra.front().signal_values.size(), kBins)));

            SpectralKurtosis sk({{"n_spectra", gr::Size_t{kM}}});
            startBlock(sk);
            std::array<double, kEdge.size()> means{};
            std::size_t                      edgeHits = 0UZ;
            for (std::size_t r = 0UZ; r < kReadings; ++r) {
                const auto readings = feed(sk, std::span<const gr::DataSet<float>>(spectra).subspan(r * kM, kM));
                expect(fatal(eq(readings.size(), 1UZ)));
                for (std::size_t i = 0UZ; i < kEdge.size(); ++i) {
                    means[i] += static_cast<double>(readings.front().signal_values[kEdge[i]]) / static_cast<double>(kReadings);
                }
                edgeHits += std::ranges::contains(kEdge, sk.worstBin()) ? 1UZ : 0UZ;
                if (r == 0UZ) {
                    const auto& record = readings.front();
                    expect(eq(metaNumber(record, "one_sided"), 1.)) << "the record keeps the input's sidedness";
                    expect(std::abs(metaNumber(record, "sk_std_edge") - gr::measurement::spectralKurtosisSpread(kM, 0.5)) < 1e-12) << "the record states the DC bin's spread";
                }
            }
            const double band     = 4. * gr::measurement::spectralKurtosisSpread(kM, 0.5) / std::sqrt(static_cast<double>(kReadings));
            const double chance   = static_cast<double>(kEdge.size()) / static_cast<double>(kBins);
            const double hitBound = static_cast<double>(kReadings) * chance + 4. * std::sqrt(static_cast<double>(kReadings) * chance * (1. - chance));
            std::println("{} on real noise: bins 0, 1, 2 read {:.4f} {:.4f} {:.4f}, bins 30, 31, 32 read {:.4f} {:.4f} {:.4f} (band 1 +- {:.4f}); worst bin among them in {} of {} readings (bound {:.1f})", windowName, means[0UZ], means[1UZ], means[2UZ], means[3UZ], means[4UZ], means[5UZ], band, edgeHits, kReadings, hitBound);
            for (std::size_t i = 0UZ; i < kEdge.size(); ++i) {
                expect(std::abs(means[i] - 1.) < band) << std::format("{}: bin {} reads 1 on noise", windowName, kEdge[i]);
            }
            expect(static_cast<double>(edgeHits) < hitBound) << std::format("{}: the worst bin lies there no more often than chance", windowName);
        }
    };

    // WelchPsd at four segments a record turns 6784 samples into 26 records and a last record of two segments. That
    // record ends the accumulation of the ten records before it. With `shape` at zero the block emits the partial
    // reading as the record arrives, and with `shape` at 4 it refuses the record and emits the reading at the end of
    // the stream.
    "behind WelchPsd the last record of a stream ends the accumulation, and its partial reading goes out"_test = [] {
        Rng                rng(0x9e3779b97f4a7c15ULL);
        std::vector<float> samples(26UZ * 256UZ + 128UZ);
        for (float& sample : samples) {
            sample = static_cast<float>(rng.normal());
        }
        for (const gr::Size_t shape : {gr::Size_t{0U}, gr::Size_t{4U}}) {
            gr::Graph graph;
            auto&     source = graph.emplaceBlock<SampleSource>();
            auto&     welch  = graph.emplaceBlock<WelchPsd<float>>({{"fft_size", gr::Size_t{64U}}, {"n_averages", gr::Size_t{4U}}, {"overlap", 0.0}, {"sample_rate", 1.f}, {"one_record_per_call", false}});
            auto&     block  = graph.emplaceBlock<SpectralKurtosis>({{"n_spectra", gr::Size_t{16U}}, {"shape", shape}});
            auto&     sink   = graph.emplaceBlock<RecordSink>();
            source.samples   = samples;
            expect(fatal(graph.connect<"out", "in">(source, welch).has_value()));
            expect(fatal(graph.connect<"out", "spectra">(welch, block).has_value()));
            expect(fatal(graph.connect<"measurements", "in">(block, sink).has_value()));
            gr::scheduler::Simple scheduler;
            expect(fatal(scheduler.exchange(std::move(graph)).has_value()));
            expect(scheduler.runAndWait().has_value());

            std::println("shape {}: {} records, nRecords {}, nRefusedRecords {}", shape, sink.records.size(), block.nRecords(), block.nRefusedRecords());
            expect(fatal(eq(sink.records.size(), 2UZ))) << std::format("shape {}: one full reading and one partial", shape);
            expect(eq(metaNumber(sink.records[0UZ], "n_spectra"), 16.));
            expect(eq(metaNumber(sink.records[1UZ], "n_spectra"), 10.)) << std::format("shape {}: the partial reading states its ten spectra", shape);
            expect(eq(metaNumber(sink.records[1UZ], "shape"), 4.));
            expect(eq(metaNumber(sink.records[1UZ], "valid"), 1.)) << "and is valid from eight spectra";
            expect(eq(block.nRecords(), std::uint64_t{1ULL}));
            expect(eq(block.nRefusedRecords(), std::uint64_t{1ULL})) << "the record of two segments is refused";
        }
    };

    // WelchPsd at one segment a record feeds the block at n_spectra 32. Each round of 32 records switches the rate
    // between 1000 and 2000 Hz after its 16th record. A density at twice the rate reads half as high, and one
    // accumulation of both rates would read about 1.22 on noise. Each change ends the accumulation in progress and
    // refuses the first record at the new rate. Every reading then covers one rate, states it and reads 1 on noise.
    "behind WelchPsd a sample_rate change inside an accumulation ends it, and every reading covers one rate"_test = [] {
        constexpr std::size_t kFft    = 64UZ;
        constexpr std::size_t kBins   = kFft / 2UZ + 1UZ;
        constexpr std::size_t kHalf   = 16UZ;
        constexpr std::size_t kRounds = 200UZ;
        Rng                   rng(0x51ed2701a3b4c5d6ULL);
        WelchPsd<float>       welch({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", 1000.f}, {"one_record_per_call", false}});
        startBlock(welch);
        SpectralKurtosis sk({{"n_spectra", gr::Size_t{2U * kHalf}}});
        startBlock(sk);

        std::vector<float> samples(kHalf * kFft);
        double             sum      = 0.;
        double             variance = 0.; // of the sum, from each reading's closed-form spread
        std::size_t        count    = 0UZ;
        std::size_t        oneRate  = 0UZ; // readings that cover the records before their change and state their rate
        for (std::size_t round = 0UZ; round < kRounds; ++round) {
            std::vector<gr::DataSet<float>> spectra;
            for (std::size_t half = 0UZ; half < 2UZ; ++half) {
                if (half == 1UZ) {
                    std::ignore = welch.settings().setStaged({{"sample_rate", round % 2UZ == 0UZ ? 2000.f : 1000.f}});
                    std::ignore = welch.settings().applyStagedParameters();
                }
                for (float& sample : samples) {
                    sample = static_cast<float>(rng.normal());
                }
                std::vector<gr::DataSet<float>>      made(kHalf + 1UZ);
                shim::InputSpan<float>               inSpan{std::span<const float>(samples)};
                shim::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(made)};
                std::ignore = welch.processBulk(inSpan, outSpan);
                made.resize(outSpan.count);
                expect(fatal(eq(made.size(), kHalf)));
                spectra.insert(spectra.end(), made.begin(), made.end());
            }
            const auto readings = feed(sk, std::span<const gr::DataSet<float>>(spectra));
            expect(fatal(eq(readings.size(), 1UZ))) << "each rate change ends one accumulation";
            const auto&       reading   = readings.front();
            const std::size_t spectraIn = round == 0UZ ? kHalf : 2UZ * kHalf - 1UZ;
            if (metaNumber(reading, "n_spectra") == static_cast<double>(spectraIn) && metaNumber(reading, "sample_rate") == (round % 2UZ == 0UZ ? 1000. : 2000.)) {
                ++oneRate;
            }
            const double spread = gr::measurement::spectralKurtosisSpread(static_cast<std::size_t>(metaNumber(reading, "n_spectra")), 1.);
            for (std::size_t k = 2UZ; k + 2UZ < kBins; ++k) {
                sum += static_cast<double>(reading.signal_values[k]);
                variance += spread * spread;
                ++count;
            }
        }
        const double meanSk = sum / static_cast<double>(count);
        const double band   = 5. * std::sqrt(variance) / static_cast<double>(count);
        std::println("rate changes inside accumulations: mean SK over bins 2 to {} is {:.4f} (band 1 +- {:.4f}), {} of {} readings cover one rate, nRecords {}, nRefusedRecords {}", kBins - 3UZ, meanSk, band, oneRate, kRounds, sk.nRecords(), sk.nRefusedRecords());
        expect(eq(oneRate, kRounds)) << "every reading covers the records before its change and states their rate";
        expect(std::abs(meanSk - 1.) < band) << "the readings read 1 on noise";
        expect(eq(sk.nRefusedRecords(), std::uint64_t{kRounds})) << "one refused record per change";
        expect(eq(sk.nRecords(), std::uint64_t{0ULL})) << "no accumulation reaches n_spectra";
    };

    // A record of two signals carries the density in its first. The block reads that signal against the axis, as the
    // detectors do, and its reading equals the reading of the same records with one signal. A record whose values do
    // not match its extent is refused, and the accumulation in progress goes on.
    "a record of two signals is read by its first, and a record that does not match its extent is refused"_test = [] {
        constexpr std::size_t           kBins = 33UZ;
        Rng                             rng(0x0badc0ffee123457ULL);
        std::vector<gr::DataSet<float>> single;
        std::vector<gr::DataSet<float>> pairs;
        for (std::size_t r = 0UZ; r < 8UZ; ++r) {
            std::vector<float> values(2UZ * kBins);
            for (float& value : values) {
                value = static_cast<float>(rng.exponential());
            }
            single.push_back(spectrum(std::vector<float>(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(kBins)), 1ULL, 0.0));
            gr::DataSet<float> two = single.back();
            two.signal_names.emplace_back("other");
            two.signal_quantities.emplace_back("PowerSpectralDensity");
            two.signal_units.emplace_back("1/Hz");
            two.signal_values = std::move(values);
            two.signal_ranges.resize(2UZ);
            two.meta_information.push_back(two.meta_information.front());
            two.timing_events.resize(2UZ);
            pairs.push_back(std::move(two));
        }
        const Run alone = runWith({{"n_spectra", gr::Size_t{8U}}}, single);
        const Run both  = runWith({{"n_spectra", gr::Size_t{8U}}}, pairs);
        expect(fatal(eq(alone.records.size(), 1UZ)));
        expect(fatal(eq(both.records.size(), 1UZ)));
        const auto& reading = both.records.front();
        expect(eq(both.nRefusedRecords, std::uint64_t{0ULL}));
        expect(eq(reading.extents.at(0UZ), static_cast<std::int32_t>(kBins))) << "the reading has the input's extent";
        expect(eq(reading.signal_values.size(), kBins) && eq(reading.axis_values.at(0UZ).size(), kBins)) << "and matches its axis";
        expect(reading.signal_values == alone.records.front().signal_values) << "the reading equals that of the first signals alone";

        std::vector<gr::DataSet<float>> rows(single.begin(), single.begin() + 4);
        gr::DataSet<float>              malformed = single[4UZ];
        malformed.signal_values.push_back(1.f);
        rows.push_back(std::move(malformed));
        rows.insert(rows.end(), single.begin() + 4, single.end());
        const Run withMalformed = runWith({{"n_spectra", gr::Size_t{8U}}}, rows);
        expect(eq(withMalformed.nRefusedRecords, std::uint64_t{1ULL})) << "a record of one value more than its extent is refused";
        expect(fatal(eq(withMalformed.records.size(), 1UZ))) << "and the accumulation goes on";
        expect(withMalformed.records.front().signal_values == alone.records.front().signal_values);
    };

    "a record that states a mode other than the mean is refused"_test = [] {
        const gr::DataSet<float>        clean = spectrum({1.f, 2.f, 3.f}, 1ULL, 0.0);
        std::vector<gr::DataSet<float>> rows  = repeated(withMeta(clean, "mode", std::string("max_hold")), 8UZ);
        for (const auto& row : repeated(withMeta(clean, "mode", std::string("mean")), 8UZ)) {
            rows.push_back(row);
        }
        const Run r = runWith({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}}, rows);
        expect(eq(r.nRefusedRecords, std::uint64_t{8ULL})) << "every max_hold record is refused";
        expect(eq(r.records.size(), 1UZ)) << "the eight mean records make one reading";
    };

    "a record with an infinite bin is refused"_test = [] {
        Rng  rng(0xa4093822299f31d0ULL);
        auto rows                    = noiseRows(rng, 9UZ, 16UZ, 1UZ);
        rows[3UZ].signal_values[5UZ] = std::numeric_limits<float>::infinity();
        const Run r                  = runWith({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}}, rows);
        expect(eq(r.nRefusedRecords, std::uint64_t{1ULL})) << "the record with the infinite bin is refused";
        expect(eq(r.records.size(), 1UZ));
        if (!r.records.empty()) {
            expect(std::ranges::all_of(r.records.front().signal_values, [](float v) { return std::isfinite(v); })) << "every bin of the reading is finite";
        }
    };

    "a record whose n_averaged is not a positive finite number is refused, and no reading carries a NaN"_test = [] {
        // At shape 0 the block reads the shape from the record's n_averaged. It refuses a record whose n_averaged is
        // NaN, negative or 0, and the reading's shape and sk_std stay positive and finite.
        const gr::DataSet<float> clean     = spectrum({1.f, 2.f, 3.f, 4.f}, 1ULL, 0.0);
        const auto               allFinite = [](const gr::DataSet<float>& record) { return std::isfinite(metaNumber(record, "shape")) && std::isfinite(metaNumber(record, "sk_std")) && metaNumber(record, "sk_std") > 0. && std::ranges::all_of(record.signal_values, [](float v) { return std::isfinite(v); }); };

        for (const double averaged : {std::numeric_limits<double>::quiet_NaN(), -0.5, 0.}) {
            const std::size_t               nBad = std::isnan(averaged) ? 1UZ : 8UZ;
            std::vector<gr::DataSet<float>> rows = repeated(withMeta(clean, "n_averaged", averaged), nBad);
            for (const auto& row : repeated(clean, 8UZ)) {
                rows.push_back(row);
            }
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}}, rows);
            expect(eq(r.nRefusedRecords, static_cast<std::uint64_t>(nBad))) << std::format("n_averaged {}: every such record is refused and counted", averaged);
            expect(fatal(eq(r.records.size(), 1UZ))) << std::format("n_averaged {}: one reading of the eight clean records", averaged);
            expect(eq(metaNumber(r.records[0UZ], "shape"), 1.)) << std::format("n_averaged {}: the reading's shape", averaged);
            expect(allFinite(r.records[0UZ])) << std::format("n_averaged {}: the reading is finite", averaged);
        }
    };

    "at the end of the stream the partial accumulation goes out, valid from eight spectra"_test = [] {
        Rng rng(0x082efa98ec4e6c89ULL);
        for (const std::size_t tail : {3UZ, 10UZ}) {
            const Run r = runWith({{"n_spectra", gr::Size_t{16U}}, {"shape", gr::Size_t{1U}}}, noiseRows(rng, 16UZ + tail, 8UZ, 1UZ));
            expect(eq(r.records.size(), 2UZ)) << std::format("tail {}: one reading and one flush", tail);
            expect(eq(r.nRecords, std::uint64_t{1ULL})) << "nRecords counts the full accumulation alone";
            if (r.records.size() == 2UZ) {
                expect(eq(metaNumber(r.records[1UZ], "n_spectra"), static_cast<double>(tail))) << "the flush states its spectra";
                expect(eq(metaNumber(r.records[1UZ], "valid"), tail >= 8UZ ? 1. : 0.)) << std::format("tail {}: valid only from eight spectra", tail);
            }
        }
    };

    // A full reading of eight spectra, then one spectrum ended by a retune, then two spectra ended by the stream.
    "a partial reading below eight spectra leaves worstBin() and worstBinSk() at the last full reading"_test = [] {
        // A reading of one spectrum is 0 at every bin.
        std::vector<gr::DataSet<float>> rows   = onePulseRows();
        const gr::DataSet<float>        steady = spectrum({1.f, 1.f}, 1ULL, 0.0);
        rows.push_back(withMeta(steady, "center_frequency", 1.42e9));
        for (const auto& row : repeated(withMeta(steady, "center_frequency", 1.43e9), 3UZ)) {
            rows.push_back(row);
        }
        const Run r = runWith({{"n_spectra", gr::Size_t{8U}}}, rows);
        expect(eq(r.nRefusedRecords, std::uint64_t{1ULL})) << "the first retuned record is refused";
        expect(fatal(eq(r.records.size(), 3UZ))) << "the full reading, the reading the retune ends and the one the stream ends";
        expect(std::abs(static_cast<double>(r.records[0UZ].signal_values.at(1UZ)) - 9.) < 1e-5) << "the full reading's bin 1 reads 9";
        expect(eq(metaNumber(r.records[1UZ], "n_spectra"), 1.) && eq(metaNumber(r.records[1UZ], "valid"), 0.)) << "one spectrum before the retune, not valid";
        expect(eq(metaNumber(r.records[2UZ], "n_spectra"), 2.) && eq(metaNumber(r.records[2UZ], "valid"), 0.)) << "two spectra at the end of the stream, not valid";
        expect(eq(r.worstBin, std::uint64_t{1ULL})) << "worstBin() holds the full reading's bin";
        expect(std::abs(r.worstBinSk - 9.) < 1e-9) << "worstBinSk() holds that bin's SK of 9";
    };

    "a dead bin takes no part in worstBin(), and a reading of dead bins leaves the polled values at the last valid reading"_test = [] {
        { // Bin 0 is dead and reads 0, further from 1 than bin 1. Bin 1 alternates 0 and 1, and its SK is 9/7.
            std::vector<gr::DataSet<float>> rows;
            for (std::size_t k = 0UZ; k < 8UZ; ++k) {
                rows.push_back(spectrum({0.f, k % 2UZ == 0UZ ? 0.f : 1.f}, 1ULL, 0.0));
            }
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}}, rows);
            expect(fatal(eq(r.records.size(), 1UZ)));
            expect(eq(metaNumber(r.records[0UZ], "valid"), 1.)) << "one live bin makes the reading valid";
            expect(eq(r.nDegenerateBins, std::uint64_t{1ULL}));
            expect(eq(r.worstBin, std::uint64_t{1ULL})) << "worstBin() is the live bin";
            expect(std::abs(r.worstBinSk - 9. / 7.) < 1e-9) << "worstBinSk() reads its 9/7";
        }
        { // A full reading of eight all-zero records follows a valid one, and every bin of it is dead.
            std::vector<gr::DataSet<float>> rows = onePulseRows();
            for (const auto& row : repeated(withMeta(spectrum({0.f, 0.f}, 1ULL, 0.0), "center_frequency", 1.42e9), 8UZ)) {
                rows.push_back(row);
            }
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}}, rows);
            expect(fatal(eq(r.records.size(), 2UZ))) << "two full readings";
            expect(eq(r.nRecords, std::uint64_t{2ULL}));
            expect(eq(metaNumber(r.records[1UZ], "valid"), 0.)) << "the all-zero reading is not valid";
            expect(eq(r.nDegenerateBins, std::uint64_t{2ULL})) << "both of its bins are dead";
            expect(eq(r.worstBin, std::uint64_t{1ULL})) << "worstBin() holds the valid reading's bin";
            expect(std::abs(r.worstBinSk - 9.) < 1e-9) << "worstBinSk() holds that bin's SK of 9";
        }
    };

    // The block runs through the test spans, and a second start() begins a new run.
    "worstBin(), worstBinSk() and nValidReadings() read 0 before the first valid reading of a run"_test = [] {
        const std::vector<gr::DataSet<float>> dead  = repeated(spectrum({0.f, 0.f}, 1ULL, 0.0), 8UZ);
        const std::vector<gr::DataSet<float>> pulse = onePulseRows();

        SpectralKurtosis block({{"n_spectra", gr::Size_t{8U}}});
        expect(eq(block.worstBin(), std::uint64_t{0ULL})) << "worstBin() at construction";
        expect(eq(block.worstBinSk(), 0.)) << "worstBinSk() at construction";
        expect(eq(block.nValidReadings(), std::uint64_t{0ULL})) << "nValidReadings() at construction";

        startBlock(block);
        expect(eq(feed(block, dead).size(), 1UZ));
        expect(eq(block.worstBin(), std::uint64_t{0ULL})) << "worstBin() after a reading that is not valid";
        expect(eq(block.worstBinSk(), 0.)) << "worstBinSk() after a reading that is not valid";
        expect(eq(block.nValidReadings(), std::uint64_t{0ULL})) << "nValidReadings() after a reading that is not valid";

        expect(eq(feed(block, pulse).size(), 1UZ));
        expect(eq(block.worstBin(), std::uint64_t{1ULL})) << "worstBin() at the first valid reading";
        expect(std::abs(block.worstBinSk() - 9.) < 1e-9) << "worstBinSk() at the first valid reading";
        expect(eq(block.nValidReadings(), std::uint64_t{1ULL})) << "nValidReadings() at the first valid reading";

        block.start();
        expect(eq(block.worstBin(), std::uint64_t{0ULL})) << "worstBin() after start()";
        expect(eq(block.worstBinSk(), 0.)) << "worstBinSk() after start()";
        expect(eq(block.nValidReadings(), std::uint64_t{0ULL})) << "nValidReadings() after start()";

        expect(eq(feed(block, pulse).size(), 1UZ));
        expect(eq(block.nValidReadings(), std::uint64_t{1ULL})) << "nValidReadings() at the first valid reading of the new run";
    };

    "worstBinReading() returns the bin, its SK and the count of one valid reading"_test = [] {
        // Bin 0 holds a pulse in two of eight spectra: M*S2/S1^2 = 4 and SK = (9/7)*(4-1) = 27/7. Bin 1 is constant
        // and reads 0, nearer to 1.
        std::vector<gr::DataSet<float>> twoPulses;
        for (std::size_t k = 0UZ; k < 8UZ; ++k) {
            twoPulses.push_back(spectrum({k < 2UZ ? 1.f : 0.f, 1.f}, 1ULL, 0.0));
        }
        SpectralKurtosis block({{"n_spectra", gr::Size_t{8U}}});
        startBlock(block);
        const SpectralKurtosis::WorstBinReading none = block.worstBinReading();
        expect(eq(none.bin, std::uint64_t{0ULL}) && eq(none.sk, 0.) && eq(none.validReadings, std::uint64_t{0ULL})) << "all three read 0 before the first valid reading";

        expect(eq(feed(block, onePulseRows()).size(), 1UZ));
        const SpectralKurtosis::WorstBinReading first = block.worstBinReading();
        expect(eq(first.bin, std::uint64_t{1ULL})) << "the first reading's bin";
        expect(std::abs(first.sk - 9.) < 1e-9) << "its SK of 9";
        expect(eq(first.validReadings, std::uint64_t{1ULL})) << "one valid reading";

        expect(eq(feed(block, twoPulses).size(), 1UZ));
        const SpectralKurtosis::WorstBinReading second = block.worstBinReading();
        expect(eq(second.bin, std::uint64_t{0ULL})) << "the second reading's bin";
        expect(std::abs(second.sk - 27. / 7.) < 1e-9) << "its SK of 27/7";
        expect(eq(second.validReadings, std::uint64_t{2ULL})) << "two valid readings";
        expect(eq(second.bin, block.worstBin()) && eq(second.sk, block.worstBinSk()) && eq(second.validReadings, block.nValidReadings())) << "the single readers read the same reading";
    };

    "a center_frequency that is not finite stays off the record, and a sample_start outside [0, 2^64) reads 0"_test = [] {
        const gr::DataSet<float> clean = spectrum({1.f, 2.f, 3.f, 4.f}, 1ULL, 0.0);
        for (const auto& [center, start] : {std::pair{std::numeric_limits<double>::quiet_NaN(), -1.0}, std::pair{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}, std::pair{1.42e9, 1e20}}) {
            const gr::DataSet<float> odd = withMeta(withMeta(clean, "center_frequency", center), "sample_start", start);
            const Run                r   = runWith({{"n_spectra", gr::Size_t{8U}}}, repeated(odd, 8UZ));
            expect(fatal(eq(r.records.size(), 1UZ))) << std::format("center_frequency {}: one reading", center);
            if (std::isfinite(center)) {
                expect(eq(metaNumber(r.records[0UZ], "center_frequency"), center)) << std::format("center_frequency {}: the record states it", center);
            } else {
                expect(!r.records[0UZ].meta_information[0UZ].contains(std::pmr::string("center_frequency"))) << std::format("center_frequency {}: the record states none", center);
            }
            expect(eq(metaNumber(r.records[0UZ], "sample_start"), 0.)) << std::format("sample_start {}: the record's sample_start reads 0", start);
        }
    };

    "the record keeps the input's center frequency and sidedness, and the record port may stay unconnected"_test = [] {
        const gr::DataSet<float> twoSided = withMeta(spectrum({1.f, 2.f, 3.f, 4.f}, 1ULL, 0.0), "center_frequency", 1.42e9);
        const Run                plain    = runWith({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}}, repeated(twoSided, 8UZ));
        expect(fatal(eq(plain.records.size(), 1UZ)));
        expect(eq(metaNumber(plain.records.front(), "center_frequency"), 1.42e9)) << "the input's center frequency";
        expect(eq(metaNumber(plain.records.front(), "one_sided"), 0.)) << "a record without one_sided is read as two-sided";
        expect(eq(metaNumber(plain.records.front(), "sk_std_edge"), -1.)) << "and states no edge spread";

        // Five bins, one-sided, from a transform of eight samples: bins 0 and 4 are real.
        const gr::DataSet<float> oneSided = withMeta(withMeta(spectrum({1.f, 2.f, 3.f, 4.f, 5.f}, 1ULL, 0.0), "one_sided", true), "fft_size", std::uint64_t{8ULL});
        const Run                real     = runWith({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}}, repeated(oneSided, 8UZ));
        expect(fatal(eq(real.records.size(), 1UZ)));
        expect(eq(metaNumber(real.records.front(), "one_sided"), 1.));
        expect(std::abs(metaNumber(real.records.front(), "sk_std_edge") - gr::measurement::spectralKurtosisSpread(8UZ, 0.5)) < 1e-12);

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<RecordSource>();
        auto&     block  = graph.emplaceBlock<SpectralKurtosis>({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}});
        Rng       rng(0x3f84d5b5b5470917ULL);
        source.records = noiseRows(rng, 16UZ, 8UZ, 1UZ);
        expect(fatal(graph.connect<"out", "spectra">(source, block).has_value()));
        gr::scheduler::Simple scheduler;
        expect(fatal(scheduler.exchange(std::move(graph)).has_value()));
        expect(scheduler.runAndWait().has_value()) << "the graph runs with the record port unconnected";
        expect(eq(block.nRecords(), std::uint64_t{2ULL})) << "and the block measures every accumulation";
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
