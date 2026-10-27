#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
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
#include <type_traits>
#include <utility>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/measurement/Kurtosis.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_kurtosis {

using gr::blocks::measurement::Kurtosis;
using CF       = std::complex<float>;
namespace shim = gr::blocks::testing::span;

/// @brief Emits a fixed sample sequence in bursts of a stated size, then ends the stream. It can publish one
/// `gr:sample_rate` tag and one `gr:n_dropped_samples` tag at stated stream indices.
template<typename T>
struct SampleSource : gr::Block<SampleSource<T>> {
    gr::PortOut<T> out;

    std::vector<T> samples{};
    std::size_t    burst   = 4096UZ;
    std::size_t    at      = 0UZ;
    float          rateTag = 0.f; ///< nonzero publishes one gr:sample_rate tag at rateAt
    std::size_t    rateAt  = 0UZ;
    gr::Size_t     dropTag = 0U; ///< positive publishes one gr:n_dropped_samples tag at dropAt
    std::size_t    dropAt  = 0UZ;

    GR_MAKE_REFLECTABLE(SampleSource, out);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (at >= samples.size()) {
            outSpan.publish(0UZ);
            this->requestStop();
            return gr::work::Status::DONE;
        }
        const std::size_t take = std::min({burst, samples.size() - at, outSpan.size()});
        std::copy_n(samples.begin() + static_cast<std::ptrdiff_t>(at), take, outSpan.begin());
        if (rateTag != 0.f && rateAt >= at && rateAt < at + take) {
            outSpan.publishTag(gr::property_map{{std::pmr::string(gr::tag::SAMPLE_RATE.key()), gr::pmt::Value(rateTag)}}, rateAt - at);
        }
        if (dropTag > 0U && dropAt >= at && dropAt < at + take) {
            outSpan.publishTag(gr::property_map{{std::pmr::string(gr::tag::N_DROPPED_SAMPLES.key()), gr::pmt::Value(dropTag)}}, dropAt - at);
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

/// @brief Seeded Gaussian samples from xorshift and Box-Muller, real or circular complex.
struct GaussianNoise {
    std::uint64_t state;

    explicit GaussianNoise(std::uint64_t seed = 0x243f6a8885a308d3ULL) : state(seed) {}

    [[nodiscard]] double uniform() {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return (static_cast<double>(state >> 11U) + 0.5) / static_cast<double>(1ULL << 53U);
    }

    /// @brief One circular complex Gaussian sample, `E|z|^2 = 1`, `u,v ~ N(0, 1/2)` independent.
    [[nodiscard]] CF complexSample() {
        const double radius = std::sqrt(-std::log(uniform()));
        const double angle  = 2. * std::numbers::pi * uniform();
        return CF(static_cast<float>(radius * std::cos(angle)), static_cast<float>(radius * std::sin(angle)));
    }

    /// @brief One real standard normal sample, Box-Muller.
    [[nodiscard]] float realSample() {
        const double radius = std::sqrt(-2. * std::log(uniform()));
        const double angle  = 2. * std::numbers::pi * uniform();
        return static_cast<float>(radius * std::cos(angle));
    }
};

/// @brief The records and the block's counters after one run, read while the scheduler holds the graph.
struct Run {
    std::vector<gr::DataSet<float>> records;

    double        coverage{};
    std::uint64_t nWindows{};
    std::uint64_t nDegenerateWindows{};
    std::uint64_t nWindowResets{};
    std::uint64_t nDroppedSampleTags{};
    bool          completed{};  ///< `runAndWait()` returned no error
    float         rateMember{}; ///< the `sample_rate` member after the run
};

struct Tagging {
    float       rate   = 0.f;
    std::size_t rateAt = 0UZ;
    gr::Size_t  drop   = 0U;
    std::size_t dropAt = 0UZ;
};

/// @brief Runs `samples` through a source, `Kurtosis<T>` and a record sink. Returns the records and the counters.
/// The block's `sample_rate` follows `gr:sample_rate`.
template<typename T>
[[nodiscard]] Run runWith(gr::property_map settings, const std::vector<T>& samples, std::size_t burst, Tagging tagging = {}) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<SampleSource<T>>();
    auto&     block  = graph.emplaceBlock<Kurtosis<T>>(std::move(settings));
    auto&     sink   = graph.emplaceBlock<RecordSink>();
    block.settings().setTagCouplings({{gr::tag::SAMPLE_RATE.name(), std::string(gr::tag::SAMPLE_RATE.key())}});
    source.samples = samples;
    source.burst   = burst;
    source.rateTag = tagging.rate;
    source.rateAt  = tagging.rateAt;
    source.dropTag = tagging.drop;
    source.dropAt  = tagging.dropAt;

    if (!graph.connect<"out", "in">(source, block).has_value() || !graph.connect<"measurements", "in">(block, sink).has_value()) {
        return {};
    }
    gr::scheduler::Simple scheduler;
    if (!scheduler.exchange(std::move(graph)).has_value()) {
        return {};
    }
    const bool completed = scheduler.runAndWait().has_value(); // the source ends the stream

    Run result;
    result.completed          = completed;
    result.rateMember         = block.sample_rate.value;
    result.records            = std::move(sink.records);
    result.coverage           = block.coverage();
    result.nWindows           = block.nWindows();
    result.nDegenerateWindows = block.nDegenerateWindows();
    result.nWindowResets      = block.nWindowResets();
    result.nDroppedSampleTags = block.nDroppedSampleTags();
    return result;
}

template<typename T>
[[nodiscard]] std::vector<gr::DataSet<float>> collect(gr::property_map settings, const std::vector<T>& samples, std::size_t burst) {
    return runWith<T>(std::move(settings), samples, burst).records;
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

/// @brief Applies the staged settings and starts the block, as the scheduler does before the first work call.
template<typename TBlock>
void startBlock(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

/// @brief Stages @p settings on a started block and applies them. Returns the settings the limits refused.
template<typename TBlock>
[[nodiscard]] gr::property_map refusedByLimits(TBlock& block, gr::property_map settings) {
    std::ignore = block.settings().setStaged(std::move(settings));
    return block.settings().applyStagedParameters().failedParameters;
}

/// @brief The records of one work call and the samples it consumed.
struct Call {
    std::vector<gr::DataSet<float>> records;
    std::size_t                     consumed{};
};

/// @brief One work call over @p samples, the first of them at stream index @p at, with room for 64 records.
template<typename T>
[[nodiscard]] Call call(Kurtosis<T>& block, std::span<const T> samples, std::size_t at = 0UZ, std::span<const gr::Tag> tags = {}) {
    std::vector<gr::DataSet<float>>      out(64UZ);
    shim::InputSpan<T>                   inSpan{samples, at, tags};
    shim::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(out)};
    std::ignore = block.processBulk(inSpan, outSpan);
    out.resize(outSpan.count);
    return {std::move(out), inSpan.consumed};
}

[[nodiscard]] gr::Tag droppedTag(std::size_t index, gr::Size_t missing) { return gr::Tag{index, gr::property_map{{std::pmr::string(gr::tag::N_DROPPED_SAMPLES.key()), gr::pmt::Value(missing)}}}; }

[[nodiscard]] double mean(const std::vector<double>& values) {
    double total = 0.;
    for (const double v : values) {
        total += v;
    }
    return values.empty() ? 0. : total / static_cast<double>(values.size());
}

[[nodiscard]] double sampleStdDev(const std::vector<double>& values, double meanValue) {
    if (values.size() < 2UZ) {
        return 0.;
    }
    double sumSq = 0.;
    for (const double v : values) {
        const double d = v - meanValue;
        sumSq += d * d;
    }
    return std::sqrt(sumSq / static_cast<double>(values.size() - 1UZ));
}

} // namespace qa_kurtosis

const boost::ut::suite<"Kurtosis"> kurtosisTests = [] {
    using namespace boost::ut;
    using namespace qa_kurtosis;

    // Seeded circular complex and real Gaussian input. The mean excess over the windows lies within four standard
    // errors of the closed-form bias, and the spread of the readings matches the closed form.
    "Gaussian excess kurtosis matches the closed-form bias and spread"_test = [] {
        constexpr std::size_t kWindow = 4096UZ;
        constexpr std::size_t kTrials = 400UZ;

        GaussianNoise   noise(0x9e3779b97f4a7c15ULL);
        std::vector<CF> complexSamples(kWindow * kTrials);
        std::ranges::generate(complexSamples, [&] { return noise.complexSample(); });
        const auto complexRecords = collect<CF>({{"window", gr::Size_t{kWindow}}}, complexSamples, 4096UZ);

        std::vector<float> realSamples(kWindow * kTrials);
        std::ranges::generate(realSamples, [&] { return noise.realSample(); });
        const auto realRecords = collect<float>({{"window", gr::Size_t{kWindow}}}, realSamples, 4096UZ);

        expect(eq(complexRecords.size(), kTrials));
        expect(eq(realRecords.size(), kTrials));

        std::vector<double> complexExcess;
        std::vector<double> realExcess;
        for (const auto& r : complexRecords) {
            complexExcess.push_back(static_cast<double>(r.signal_values.at(0UZ)));
        }
        for (const auto& r : realRecords) {
            realExcess.push_back(static_cast<double>(r.signal_values.at(0UZ)));
        }

        const double biasComplex   = -2. / static_cast<double>(kWindow);
        const double spreadComplex = 2. / std::sqrt(static_cast<double>(kWindow));
        const double semComplex    = 4. * spreadComplex / std::sqrt(static_cast<double>(kTrials));
        const double biasReal      = -6. / static_cast<double>(kWindow);
        const double spreadReal    = std::sqrt(24. / static_cast<double>(kWindow));
        const double semReal       = 4. * spreadReal / std::sqrt(static_cast<double>(kTrials));

        const double meanComplex = mean(complexExcess);
        const double meanReal    = mean(realExcess);
        const double sdComplex   = sampleStdDev(complexExcess, meanComplex);
        const double sdReal      = sampleStdDev(realExcess, meanReal);

        std::println("Gaussian: complex mean excess {:.6f} (band {:.6f} +- {:.6f}), spread {:.5f} (nominal {:.5f})", meanComplex, biasComplex, semComplex, sdComplex, spreadComplex);
        std::println("Gaussian: real    mean excess {:.6f} (band {:.6f} +- {:.6f}), spread {:.5f} (nominal {:.5f})", meanReal, biasReal, semReal, sdReal, spreadReal);

        expect(std::abs(meanComplex - biasComplex) < semComplex) << "complex mean excess kurtosis outside its 4*sigma_mean band";
        expect(std::abs(meanReal - biasReal) < semReal) << "real mean excess kurtosis outside its 4*sigma_mean band";
        expect(std::abs(sdComplex - spreadComplex) < 0.10 * spreadComplex) << "complex per-window spread outside 10% of the closed form";
        expect(std::abs(sdReal - spreadReal) < 0.10 * spreadReal) << "real per-window spread outside 10% of the closed form";

        // The unshifted channel reads K = m4/m2^2 = 2 within the same band.
        std::vector<double> complexNormalized;
        for (const auto& r : complexRecords) {
            complexNormalized.push_back(static_cast<double>(r.signal_values.at(1UZ)));
        }
        const double meanK = mean(complexNormalized);
        std::println("complex Gaussian normalized fourth moment mean {:.6f} (nominal 2)", meanK);
        expect(std::abs(meanK - (2. + biasComplex)) < semComplex) << "circular complex Gaussian must read K=2 within the Gaussian excess-kurtosis band";
    };

    // The three other closed values, each over one long window.
    "constant modulus, two equal tones and a real sinusoid read their closed excess values"_test = [] {
        constexpr std::size_t kWindow = 1UZ << 18U; // 262144

        // Constant modulus, |z| = A with a random phase. K = 1 and the excess is -1 for any window and any phase.
        {
            GaussianNoise    noise(0x1234567890abcdefULL);
            std::vector<CF>  samples(kWindow);
            constexpr double kAmplitude = 1.7;
            for (CF& s : samples) {
                const double theta = 2. * std::numbers::pi * noise.uniform();
                s                  = CF(static_cast<float>(kAmplitude * std::cos(theta)), static_cast<float>(kAmplitude * std::sin(theta)));
            }
            const auto records = collect<CF>({{"window", gr::Size_t{kWindow}}}, samples, 8192UZ);
            expect(eq(records.size(), 1UZ));
            if (!records.empty()) {
                const double excess = static_cast<double>(records.front().signal_values.at(0UZ));
                std::println("constant modulus excess {:.6f} (exact -1)", excess);
                expect(std::abs(excess - (-1.)) < 1e-4) << "constant modulus must read excess -1 to float rounding";
            }
        }

        // Two equal complex tones, |z|^2 = 2 + 2*cos(D) with D the difference phase. D sweeps a non-integer number of
        // cycles over the window and is close to uniform.
        {
            std::vector<CF>  samples(kWindow);
            constexpr double kBin1 = 41.0;
            constexpr double kBin2 = 41.0 + 3000.37; // a large, non-integer cycle difference over the window
            for (std::size_t k = 0UZ; k < kWindow; ++k) {
                const double n  = static_cast<double>(k);
                const double p1 = 2. * std::numbers::pi * kBin1 * n / static_cast<double>(kWindow);
                const double p2 = 2. * std::numbers::pi * kBin2 * n / static_cast<double>(kWindow);
                samples[k]      = CF(static_cast<float>(std::cos(p1) + std::cos(p2)), static_cast<float>(std::sin(p1) + std::sin(p2)));
            }
            const auto records = collect<CF>({{"window", gr::Size_t{kWindow}}}, samples, 8192UZ);
            expect(eq(records.size(), 1UZ));
            if (!records.empty()) {
                const double excess = static_cast<double>(records.front().signal_values.at(0UZ));
                std::println("two equal complex tones excess {:.6f} (nominal -0.5)", excess);
                expect(std::abs(excess - (-0.5)) < 0.01) << "two equal-amplitude complex tones must read excess close to -0.5";
            }
        }

        // A real sinusoid, m4 = 3A^4/8 and m2 = A^2/2. K = 3/2 and the excess is -1.5 over a whole number of cycles.
        {
            std::vector<float> samples(kWindow);
            constexpr double   kBin = 977.0;
            for (std::size_t k = 0UZ; k < kWindow; ++k) {
                samples[k] = static_cast<float>(std::cos(2. * std::numbers::pi * kBin * static_cast<double>(k) / static_cast<double>(kWindow)));
            }
            const auto records = collect<float>({{"window", gr::Size_t{kWindow}}}, samples, 8192UZ);
            expect(eq(records.size(), 1UZ));
            if (!records.empty()) {
                const double excess = static_cast<double>(records.front().signal_values.at(0UZ));
                std::println("real sinusoid excess {:.6f} (nominal -1.5)", excess);
                expect(std::abs(excess - (-1.5)) < 1e-3) << "a real sinusoid over a whole number of cycles must read excess -1.5";
            }
        }
    };

    // The closed-form impulsive excess of `x = g + b*A`, with g standard Gaussian and b Bernoulli(p), over 60 windows
    // of 65536 samples.
    "an impulsive stream reads the closed-form positive excess"_test = [] {
        constexpr std::size_t kWindow = 1UZ << 16U; // 65536
        constexpr std::size_t kTrials = 60UZ;

        const auto impulsiveExcess = [](double p, double a) { return (3. + 6. * p * a * a + p * a * a * a * a) / ((1. + p * a * a) * (1. + p * a * a)) - 3.; };

        struct Case {
            double p;
            double a;
        };
        const std::array<Case, 3UZ> cases{{{0.001, 10.}, {0.01, 5.}, {0.001, 20.}}};

        GaussianNoise noise(0xabcdef0123456789ULL);
        for (const Case& c : cases) {
            std::vector<float> samples(kWindow * kTrials);
            for (float& s : samples) {
                const bool impulse = noise.uniform() < c.p;
                s                  = noise.realSample() + (impulse ? static_cast<float>(c.a) : 0.f);
            }
            const auto records = collect<float>({{"window", gr::Size_t{kWindow}}}, samples, 8192UZ);
            expect(eq(records.size(), kTrials));

            std::vector<double> readings;
            for (const auto& r : records) {
                readings.push_back(static_cast<double>(r.signal_values.at(0UZ)));
            }
            const double measured = mean(readings);
            const double expected = impulsiveExcess(c.p, c.a);
            std::println("impulsive: p={:.3f} A={:.1f} measured excess {:.4f}, closed form {:.4f}", c.p, c.a, measured, expected);

            expect(measured > 0.) << "the impulsive case must read a positive excess";
            expect(std::abs(measured - expected) < 0.10 * std::abs(expected) + 0.3) << std::format("p={} A={}: measured {} too far from closed form {}", c.p, c.a, measured, expected);
        }
    };

    // An all-zero window is degenerate. A stream that ends inside a window flushes the samples it covered.
    "a degenerate window reads zero with valid false, and a partial window states its coverage"_test = [] {
        const auto everyChannelFinite = [](const gr::DataSet<float>& record, std::string_view what) {
            for (std::size_t k = 0UZ; k < 3UZ; ++k) {
                expect(std::isfinite(record.signal_values.at(k))) << std::format("{}: channel {} must be finite, read {}", what, k, record.signal_values.at(k));
            }
        };

        std::vector<float> zeros(4096UZ, 0.f);
        const Run          zeroRun = runWith<float>({{"window", gr::Size_t{4096U}}}, zeros, 4096UZ);
        expect(eq(zeroRun.records.size(), 1UZ));
        expect(eq(zeroRun.nDegenerateWindows, std::uint64_t{1ULL})) << "the all-zero window must be counted degenerate";
        if (!zeroRun.records.empty()) {
            const auto& r = zeroRun.records.front();
            expect(eq(r.signal_values.at(0UZ), 0.f));
            expect(eq(r.signal_values.at(1UZ), 0.f));
            expect(eq(r.signal_values.at(2UZ), 0.f));
            expect(eq(metaNumber(r, "valid"), 0.)) << "an all-zero window must be marked invalid";
            everyChannelFinite(r, "degenerate window");
        }

        {
            GaussianNoise      noise(0x55aa55aa55aa55aaULL);
            std::vector<float> samples(4096UZ + 900UZ);
            std::ranges::generate(samples, [&] { return noise.realSample(); });
            const Run partial = runWith<float>({{"window", gr::Size_t{4096U}}}, samples, 4096UZ);

            expect(eq(partial.records.size(), 2UZ)) << "one full window and one partial flush";
            expect(eq(partial.nWindows, std::uint64_t{1ULL})) << "only the full window counts as a window";
            std::println("coverage after the partial flush {:.6f} (900/4096 = {:.6f})", partial.coverage, 900. / 4096.);
            expect(std::abs(partial.coverage - 900. / 4096.) < 1e-9) << "coverage() reads the fraction the flush covered";
            expect(partial.coverage < 1.) << "a partial window must report a coverage below one";
            if (partial.records.size() == 2UZ) {
                expect(eq(metaNumber(partial.records[1UZ], "n_samples"), 900.)) << "the flush states its sample count";
                expect(eq(metaNumber(partial.records[0UZ], "n_samples"), 4096.));
                everyChannelFinite(partial.records[0UZ], "full window");
                everyChannelFinite(partial.records[1UZ], "partial flush");
            }
        }
    };

    // The window bounds, checked when the settings are applied.
    "a window outside [8, 2^24] and a sample_rate that is not positive and finite are refused by their limits"_test = [] {
        const std::array<gr::property_map, 6UZ> outside{{
            {{"window", gr::Size_t{7U}}},
            {{"window", gr::Size_t{(1U << 24U) + 1U}}},
            {{"sample_rate", 0.f}},
            {{"sample_rate", -48000.f}},
            {{"sample_rate", std::numeric_limits<float>::infinity()}},
            {{"sample_rate", std::numeric_limits<float>::quiet_NaN()}},
        }};
        for (std::size_t r = 0UZ; r < outside.size(); ++r) {
            Kurtosis<float> block(gr::property_map{});
            startBlock(block);
            expect(throws([&] { std::ignore = block.settings().set(outside[r]); })) << std::format("value {}: settings().set() refuses it at the call", r);
            expect(eq(refusedByLimits(block, outside[r]).size(), 1UZ)) << std::format("value {}: a staged one is refused at the apply", r);
            expect(eq(block.window.value, gr::Size_t{4096U}) && eq(block.sample_rate.value, 96000.f)) << std::format("value {}: the members keep the accepted values", r);
        }
        for (const gr::Size_t value : {gr::Size_t{8U}, gr::Size_t{1U << 24U}}) {
            Kurtosis<float> block(gr::property_map{});
            startBlock(block);
            expect(refusedByLimits(block, {{"window", value}}).empty()) << "the limit " << value << " itself is admitted";
            expect(eq(block.window.value, value));
        }
    };

    // Before the first window, coverage() reads 0.
    "coverage() reads 0 before the first window closes"_test = [] {
        Kurtosis<CF> block;
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
        expect(eq(block.coverage(), 0.));
        expect(eq(block.nWindows(), std::uint64_t{0ULL}));
    };

    // Chunk independence in both domains. The burst sizes put call boundaries on and off the window boundaries and the
    // tagged sample. The test compares the counters and the records.
    "chunk independence, bit-identical records and counters at every chunk size"_test = [] {
        const auto sweep = [](const auto& samples, std::string_view domain) {
            using Sample                   = std::remove_cvref_t<decltype(samples.front())>;
            constexpr std::size_t kDropAt  = 1200UZ; // inside window 1, off a window boundary
            const auto            settings = [] { return gr::property_map{{"window", gr::Size_t{1024U}}}; };
            const Tagging         tagging{.rate = 0.f, .rateAt = 0UZ, .drop = gr::Size_t{64U}, .dropAt = kDropAt};

            const Run reference = runWith<Sample>(settings(), samples, 1024UZ, tagging);
            expect(!reference.records.empty()) << domain;
            expect(eq(reference.nDroppedSampleTags, std::uint64_t{1ULL})) << std::format("{}: the one dropped-sample tag must be counted once", domain);

            for (const std::size_t burst : {1UZ, 7UZ, 1000UZ, 12345UZ}) {
                const Run candidate = runWith<Sample>(settings(), samples, burst, tagging);
                expect(eq(candidate.records.size(), reference.records.size())) << std::format("{} burst {}: record count", domain, burst);
                expect(eq(candidate.nWindows, reference.nWindows)) << std::format("{} burst {}: nWindows", domain, burst);
                expect(eq(candidate.nDegenerateWindows, reference.nDegenerateWindows)) << std::format("{} burst {}: nDegenerateWindows", domain, burst);
                expect(eq(candidate.nWindowResets, reference.nWindowResets)) << std::format("{} burst {}: nWindowResets", domain, burst);
                expect(eq(candidate.nDroppedSampleTags, reference.nDroppedSampleTags)) << std::format("{} burst {}: nDroppedSampleTags", domain, burst);
                for (std::size_t r = 0UZ; r < std::min(candidate.records.size(), reference.records.size()); ++r) {
                    for (std::size_t k = 0UZ; k < 3UZ; ++k) {
                        expect(eq(candidate.records[r].signal_values.at(k), reference.records[r].signal_values.at(k))) << std::format("{} burst {}: record {} channel {}", domain, burst, r, k);
                    }
                    expect(eq(metaNumber(candidate.records[r], "n_samples"), metaNumber(reference.records[r], "n_samples"))) << std::format("{} burst {}: record {} n_samples", domain, burst, r);
                    expect(eq(metaNumber(candidate.records[r], "n_dropped_samples"), metaNumber(reference.records[r], "n_dropped_samples"))) << std::format("{} burst {}: record {} n_dropped_samples", domain, burst, r);
                }
            }
        };

        GaussianNoise   noise(0x0f0f0f0f0f0f0f0fULL);
        std::vector<CF> complexSamples(3UZ * 1024UZ + 777UZ);
        std::ranges::generate(complexSamples, [&] { return noise.complexSample(); });
        sweep(complexSamples, "complex");

        std::vector<float> realSamples(3UZ * 1024UZ + 777UZ);
        std::ranges::generate(realSamples, [&] { return noise.realSample(); });
        sweep(realSamples, "real");
    };

    // The settings and the record keys: emit_records off, the label, and the rate the record states, set by the
    // setting and by the reserved gr:sample_rate tag.
    "settings and record metadata: emit_records, signal_name, sample_rate by setting and by tag"_test = [] {
        GaussianNoise   noise(0x6a09e667f3bcc908ULL);
        std::vector<CF> samples(2UZ * 4096UZ);
        std::ranges::generate(samples, [&] { return noise.complexSample(); });

        {
            const Run silent = runWith<CF>({{"window", gr::Size_t{4096U}}, {"emit_records", false}}, samples, 4096UZ);
            expect(eq(silent.records.size(), 0UZ)) << "emit_records = false publishes no record";
            expect(eq(silent.nWindows, std::uint64_t{2ULL})) << "the windows are still measured and counted";
            expect(std::abs(silent.coverage - 1.) < 1e-12) << "and coverage() reads one full window";
        }
        {
            const Run named = runWith<CF>({{"window", gr::Size_t{4096U}}, {"signal_name", std::string("rfi_probe")}}, samples, 4096UZ);
            expect(eq(named.records.size(), 2UZ));
            if (!named.records.empty()) {
                expect(eq(metaString(named.records.front(), "signal_name"), std::string("rfi_probe")));
                expect(eq(metaString(named.records.front(), "input_domain"), std::string("complex")));
                std::println("settings: default record sample_rate {:.1f} Hz", metaNumber(named.records.front(), "sample_rate"));
                expect(std::abs(metaNumber(named.records.front(), "sample_rate") - 96000.) < 1e-3) << "the record states the default rate";
            }
        }
        {
            const Run set = runWith<CF>({{"window", gr::Size_t{4096U}}, {"sample_rate", 250000.f}}, samples, 4096UZ);
            expect(eq(set.records.size(), 2UZ));
            for (const auto& record : set.records) {
                expect(std::abs(metaNumber(record, "sample_rate") - 250000.) < 1e-3) << "a sample_rate setting reaches every record";
            }
        }
        {
            // The coupled sample_rate takes the gr:sample_rate tag at the first sample.
            const Run tagged = runWith<CF>({{"window", gr::Size_t{4096U}}}, samples, 1024UZ, Tagging{.rate = 32000.f, .rateAt = 0UZ, .drop = 0U, .dropAt = 0UZ});
            expect(eq(tagged.records.size(), 2UZ));
            for (const auto& record : tagged.records) {
                std::println("settings: record sample_rate after the tag {:.1f} Hz", metaNumber(record, "sample_rate"));
                expect(std::abs(metaNumber(record, "sample_rate") - 32000.) < 1e-3) << "a sample_rate tag sets the rate the record states";
            }
        }
        {
            const Run dropped = runWith<CF>({{"window", gr::Size_t{4096U}}}, samples, 4096UZ, Tagging{.rate = 0.f, .rateAt = 0UZ, .drop = gr::Size_t{7U}, .dropAt = 100UZ});
            expect(eq(dropped.nDroppedSampleTags, std::uint64_t{1ULL})) << "the dropped-sample tag is counted";
            expect(eq(dropped.records.size(), 2UZ));
            if (!dropped.records.empty()) {
                expect(eq(metaNumber(dropped.records[0UZ], "n_dropped_samples"), 7.)) << "the window holding the tag states the missing count";
                expect(eq(metaNumber(dropped.records[1UZ], "n_dropped_samples"), -1.)) << "a window with no missing samples has no such key";
            }
        }
    };

    // A new `window` discards the partial accumulation, and nWindowResets() counts the discard. The test drives the
    // block's settings directly.
    "a window change discards the partial accumulation and counts the reset"_test = [] {
        Kurtosis<float> block({{"window", gr::Size_t{1024U}}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
        expect(eq(block.nWindowResets(), std::uint64_t{0ULL}));

        const std::vector<float> partial(100UZ, 0.5f);
        block._acc.add(std::span<const float>(partial)); // a part-filled window
        expect(eq(block._acc.count(), 100UZ));

        expect(block.settings().setStaged(gr::property_map{{"window", gr::Size_t{2048U}}}).empty()) << "the new window is accepted";
        std::ignore = block.settings().applyStagedParameters();
        std::println("window change: nWindowResets() = {}, accumulated {} samples after it", block.nWindowResets(), block._acc.count());
        expect(eq(block.nWindowResets(), std::uint64_t{1ULL})) << "the discarded partial accumulation is counted";
        expect(eq(block._acc.count(), 0UZ)) << "and the accumulation is empty";
        expect(eq(block.window.value, gr::Size_t{2048U}));

        expect(block.settings().setStaged(gr::property_map{{"window", gr::Size_t{4096U}}}).empty());
        std::ignore = block.settings().applyStagedParameters();
        expect(eq(block.nWindowResets(), std::uint64_t{1ULL})) << "a change with an empty accumulation counts no reset";
    };

    "a sample_rate outside its limits on a tag is refused, and the run goes on at the accepted rate"_test = [] {
        // The valid arm shows that a gr:sample_rate tag reaches the block.
        GaussianNoise   noise(0xa4093822299f31d0ULL);
        std::vector<CF> samples(4UZ * 4096UZ);
        std::ranges::generate(samples, [&] { return noise.complexSample(); });

        const Run valid = runWith<CF>({}, samples, 1024UZ, Tagging{.rate = 32000.f, .rateAt = 8192UZ, .drop = 0U, .dropAt = 0UZ});
        expect(valid.completed);
        expect(fatal(eq(valid.records.size(), 4UZ)));
        expect(eq(metaNumber(valid.records[1UZ], "sample_rate"), 96000.) && eq(metaNumber(valid.records[2UZ], "sample_rate"), 32000.)) << "a valid tag moves the rate from its sample on";

        for (const float rate : {-1.f, std::numeric_limits<float>::quiet_NaN()}) {
            const Run refused = runWith<CF>({}, samples, 1024UZ, Tagging{.rate = rate, .rateAt = 8192UZ, .drop = 0U, .dropAt = 0UZ});
            expect(refused.completed) << std::format("rate {}: the run completes", rate);
            expect(eq(refused.records.size(), 4UZ)) << std::format("rate {}: every window is read", rate);
            for (const auto& record : refused.records) {
                expect(eq(metaNumber(record, "sample_rate"), 96000.)) << std::format("rate {}: every record states the accepted rate", rate);
            }
            expect(eq(refused.rateMember, 96000.f)) << std::format("rate {}: the member keeps the accepted rate", rate);
        }
    };

    // A started block takes an application its limits refuse and then the rest of the stream. Its records match a block
    // that saw no refusal, and a valid window after the refusal applies.
    "a value outside its limits on a running block changes nothing it computes, and the rest of its application applies"_test = [] {
        GaussianNoise      noise(0x13198a2e03707344ULL);
        std::vector<float> samples(12000UZ);
        std::ranges::generate(samples, [&] { return noise.realSample(); });
        const std::span<const float> all(samples);
        const auto                   settings = [] { return gr::property_map{{"window", gr::Size_t{1024U}}, {"sample_rate", 48000.f}}; };

        Kurtosis<float> reference(settings());
        startBlock(reference);
        const Call        referenceHead = call(reference, all.first(1500UZ));
        const std::size_t split         = referenceHead.consumed;
        const Call        referenceTail = call(reference, all.subspan(split), split);
        expect(eq(referenceTail.records.size(), 10UZ));

        const std::array<gr::property_map, 4UZ> refusals{{
            {{"window", gr::Size_t{4U}}},
            {{"window", gr::Size_t{(1U << 24U) + 1U}}},
            {{"sample_rate", 0.f}},
            {{"sample_rate", std::numeric_limits<float>::quiet_NaN()}},
        }};
        for (std::size_t r = 0UZ; r < refusals.size(); ++r) {
            Kurtosis<float> block(settings());
            startBlock(block);
            expect(eq(call(block, all.first(1500UZ)).consumed, split));
            expect(eq(refusedByLimits(block, refusals[r]).size(), 1UZ)) << std::format("refusal {} is refused", r);
            expect(eq(block.window.value, gr::Size_t{1024U}) && eq(block.sample_rate.value, 48000.f)) << std::format("refusal {}: the members keep the accepted values", r);
            const Call tail = call(block, all.subspan(split), split);
            expect(eq(tail.records.size(), referenceTail.records.size())) << std::format("refusal {}: the same records", r);
            for (std::size_t k = 0UZ; k < std::min(tail.records.size(), referenceTail.records.size()); ++k) {
                const auto& got  = tail.records[k];
                const auto& want = referenceTail.records[k];
                expect(std::ranges::equal(got.signal_values, want.signal_values)) << std::format("refusal {}: record {} figures", r, k);
                for (const std::string_view key : {"sample_start", "n_samples", "window", "sample_rate"}) {
                    expect(eq(metaNumber(got, key), metaNumber(want, key))) << std::format("refusal {}: record {} {}", r, k, key);
                }
            }
            expect(eq(block.nWindows(), reference.nWindows())) << std::format("refusal {}: nWindows", r);
            expect(eq(block.nWindowResets(), std::uint64_t{0ULL})) << std::format("refusal {}: no reset", r);
            expect(eq(block.coverage(), reference.coverage())) << std::format("refusal {}: coverage", r);
        }

        // The limits refuse window, and signal_name applies.
        Kurtosis<float> block(settings());
        startBlock(block);
        std::ignore                    = call(block, all.first(1500UZ));
        const gr::property_map refused = refusedByLimits(block, {{"window", gr::Size_t{4U}}, {"signal_name", std::string("renamed")}});
        expect(eq(refused.size(), 1UZ) && refused.contains("window")) << "only window is refused";
        expect(eq(block.nWindowResets(), std::uint64_t{0ULL})) << "and the block keeps its window";
        expect(refusedByLimits(block, {{"window", gr::Size_t{2048U}}}).empty()) << "a valid window is accepted";
        expect(eq(block.nWindowResets(), std::uint64_t{1ULL})) << "the partial window is discarded";
        const Call afterChange = call(block, all.subspan(split), split);
        expect(eq(afterChange.records.size(), (samples.size() - split) / 2048UZ));
        for (const auto& record : afterChange.records) {
            expect(eq(metaNumber(record, "window"), 2048.)) << "the records use the new window";
            expect(eq(metaString(record, "signal_name"), std::string("renamed"))) << "and the label the refused application carried";
        }
        if (!afterChange.records.empty()) {
            expect(eq(metaNumber(afterChange.records.front(), "sample_start"), static_cast<double>(split))) << "the new window starts where the change took effect";
        }
    };

    "a call that ends on a window boundary closes that window in the same call"_test = [] {
        GaussianNoise      noise(0xa4093822299f31d0ULL);
        std::vector<float> samples(4096UZ);
        std::ranges::generate(samples, [&] { return noise.realSample(); });
        Kurtosis<float> block({{"window", gr::Size_t{4096U}}});
        startBlock(block);
        const Call result = call(block, std::span<const float>(samples));
        expect(eq(result.consumed, 4096UZ)) << "the block takes the whole call";
        expect(eq(result.records.size(), 1UZ)) << "and emits the window its last sample completes";
        expect(eq(block.nWindows(), std::uint64_t{1ULL}));
    };

    "a flushed reading below eight samples is not valid"_test = [] {
        GaussianNoise noise(0x082efa98ec4e6c89ULL);
        for (const std::size_t tail : {5UZ, 8UZ}) {
            std::vector<float> samples(4096UZ + tail);
            std::ranges::generate(samples, [&] { return noise.realSample(); });
            const Run run = runWith<float>({{"window", gr::Size_t{4096U}}}, samples, 4096UZ);
            expect(eq(run.records.size(), 2UZ)) << std::format("tail {}: one window and one flush", tail);
            if (run.records.size() == 2UZ) {
                expect(eq(metaNumber(run.records[1UZ], "n_samples"), static_cast<double>(tail)));
                expect(eq(metaNumber(run.records[1UZ], "valid"), tail >= 8UZ ? 1. : 0.)) << std::format("tail {}: valid only from the floor of eight", tail);
                expect(eq(metaNumber(run.records[0UZ], "valid"), 1.));
            }
        }
    };

    // One window of three holds a NaN or an infinite sample. That window reads zero, is marked and counted, and the
    // windows around it are unchanged.
    "a NaN or infinite sample makes its window invalid, counted and marked non_finite"_test = [] {
        const auto check = [](const Run& run, std::string_view what) {
            expect(eq(run.records.size(), 3UZ)) << what;
            expect(eq(run.nDegenerateWindows, std::uint64_t{1ULL})) << std::format("{}: the window is counted", what);
            if (run.records.size() != 3UZ) {
                return;
            }
            const auto& bad = run.records[1UZ];
            expect(eq(metaNumber(bad, "valid"), 0.)) << std::format("{}: the window is not valid", what);
            expect(eq(metaNumber(bad, "non_finite"), 1.)) << std::format("{}: the record says why", what);
            for (std::size_t k = 0UZ; k < 3UZ; ++k) {
                expect(eq(bad.signal_values.at(k), 0.f)) << std::format("{}: channel {} reads zero", what, k);
            }
            for (const std::size_t k : {0UZ, 2UZ}) {
                expect(eq(metaNumber(run.records[k], "valid"), 1.)) << std::format("{}: record {} stays valid", what, k);
                expect(eq(metaNumber(run.records[k], "non_finite"), -1.)) << std::format("{}: record {} has no non_finite key", what, k);
            }
        };
        GaussianNoise noise(0x452821e638d01377ULL);
        for (const float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
            std::vector<float> samples(3UZ * 64UZ);
            std::ranges::generate(samples, [&] { return noise.realSample(); });
            samples[64UZ + 10UZ] = bad;
            check(runWith<float>({{"window", gr::Size_t{64U}}}, samples, 4096UZ), std::isnan(bad) ? "real NaN" : "real infinity");
        }
        std::vector<CF> complexSamples(3UZ * 64UZ);
        std::ranges::generate(complexSamples, [&] { return noise.complexSample(); });
        complexSamples[64UZ + 10UZ] = CF(0.f, std::numeric_limits<float>::quiet_NaN());
        check(runWith<CF>({{"window", gr::Size_t{64U}}}, complexSamples, 4096UZ), "complex NaN");
    };

    // The input's stream index at the first call is not zero, as after a restart. The block counts positions from
    // `start()`.
    "the dropped-sample count holds when the input does not start at stream index zero"_test = [] {
        GaussianNoise      noise(0xbe5466cf34e90c6cULL);
        std::vector<float> samples(2049UZ);
        std::ranges::generate(samples, [&] { return noise.realSample(); });
        for (const std::size_t at : {0UZ, 100000UZ}) {
            Kurtosis<float> block({{"window", gr::Size_t{1024U}}});
            startBlock(block);
            const std::array<gr::Tag, 1UZ> tags{droppedTag(at + 10UZ, gr::Size_t{5U})};
            const Call                     result = call(block, std::span<const float>(samples), at, std::span<const gr::Tag>(tags));
            expect(eq(block.nDroppedSampleTags(), std::uint64_t{1ULL})) << std::format("start index {}: the tag is counted", at);
            expect(fatal(eq(result.records.size(), 2UZ)));
            expect(eq(metaNumber(result.records[0UZ], "n_dropped_samples"), 5.)) << std::format("start index {}: the first window states the missing samples", at);
            expect(eq(metaNumber(result.records[0UZ], "sample_start"), 0.)) << "the record counts from the first sample after start()";
            expect(eq(metaNumber(result.records[1UZ], "n_dropped_samples"), -1.));
        }
    };

    // A tag in a partial window that a window change discards does not reach the next record.
    "after a window change the dropped count covers the new window alone"_test = [] {
        GaussianNoise      noise(0xc0ac29b7c97c50ddULL);
        std::vector<float> samples(6000UZ);
        std::ranges::generate(samples, [&] { return noise.realSample(); });
        const std::span<const float>   all(samples);
        const std::array<gr::Tag, 1UZ> tags{droppedTag(1200UZ, gr::Size_t{64U})};

        Kurtosis<float> block({{"window", gr::Size_t{1024U}}});
        startBlock(block);
        const Call head = call(block, all.first(1500UZ), 0UZ, std::span<const gr::Tag>(tags));
        expect(eq(head.records.size(), 1UZ));
        expect(eq(block.nDroppedSampleTags(), std::uint64_t{1ULL})) << "the tag in the partial window is counted";
        expect(refusedByLimits(block, {{"window", gr::Size_t{2048U}}}).empty());
        expect(eq(block.nWindowResets(), std::uint64_t{1ULL}));

        const Call tail = call(block, all.subspan(head.consumed), head.consumed, std::span<const gr::Tag>(tags));
        expect(fatal(!tail.records.empty()));
        expect(eq(metaNumber(tail.records.front(), "n_dropped_samples"), -1.)) << "the discarded window's missing samples are not stated";
    };

    "after a window change coverage() reads the last reading's own fraction"_test = [] {
        GaussianNoise      noise(0x9216d5d98979fb1bULL);
        std::vector<float> samples(1500UZ);
        std::ranges::generate(samples, [&] { return noise.realSample(); });
        Kurtosis<float> block({{"window", gr::Size_t{1024U}}});
        startBlock(block);
        expect(eq(call(block, std::span<const float>(samples)).records.size(), 1UZ));
        expect(eq(block.coverage(), 1.)) << "the reading covers its window";
        expect(refusedByLimits(block, {{"window", gr::Size_t{2048U}}}).empty());
        expect(eq(block.coverage(), 1.)) << "the same reading still covers the window it was taken over";
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
