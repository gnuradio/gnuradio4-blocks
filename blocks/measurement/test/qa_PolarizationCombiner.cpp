#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <future>
#include <limits>
#include <numbers>
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
#include <gnuradio-4.0/measurement/PolarizationCombiner.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_polcomb {

using gr::blocks::measurement::PolarizationCombiner;
using CF = std::complex<float>;

/// @brief Emits a fixed sample sequence in bursts, then ends the stream. It can publish one tag of settings at a
/// stated stream index.
template<typename T>
struct SampleSource : gr::Block<SampleSource<T>> {
    gr::PortOut<T> out;

    std::vector<T>   samples{};
    std::size_t      burst = 4096UZ;
    gr::property_map settingsTag{}; ///< published at settingsAt when not empty
    std::size_t      settingsAt = 0UZ;
    std::size_t      at         = 0UZ;

    GR_MAKE_REFLECTABLE(SampleSource, out);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (at >= samples.size()) {
            outSpan.publish(0UZ);
            this->requestStop();
            return gr::work::Status::DONE;
        }
        const std::size_t take = std::min({burst, samples.size() - at, outSpan.size()});
        std::copy_n(samples.begin() + static_cast<std::ptrdiff_t>(at), take, outSpan.begin());
        if (!settingsTag.empty() && settingsAt >= at && settingsAt < at + take) {
            outSpan.publishTag(settingsTag, settingsAt - at);
        }
        at += take;
        outSpan.publish(take);
        return gr::work::Status::OK;
    }
};

template<typename T>
struct Collector : gr::Block<Collector<T>> {
    gr::PortIn<T> in;

    std::vector<T> items{};

    GR_MAKE_REFLECTABLE(Collector, in);

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        items.insert(items.end(), inSpan.begin(), inSpan.end());
        std::ignore = inSpan.consume(inSpan.size());
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

/// @brief Seeded circular complex Gaussian samples from xorshift and Box-Muller, `E|z|^2 = 1`.
struct Rng {
    std::uint64_t state;
    explicit Rng(std::uint64_t seed = 0xbf58476d1ce4e5b9ULL) : state(seed) {}
    [[nodiscard]] double uniform() {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return (static_cast<double>(state >> 11U) + 0.5) / static_cast<double>(1ULL << 53U);
    }
    [[nodiscard]] CF complexSample() {
        const double radius = std::sqrt(-std::log(uniform()));
        const double angle  = 2. * std::numbers::pi * uniform();
        return CF(static_cast<float>(radius * std::cos(angle)), static_cast<float>(radius * std::sin(angle)));
    }
};

struct Branches {
    std::vector<CF> r0;
    std::vector<CF> r1;
    std::vector<CF> s; ///< the transmitted signal
};

/// @brief `r_i = h_i * s + n_i`, with `s` and both noises independent circular complex Gaussian. `noisePower` is the
/// equal noise power per branch. `signalPower` scales `s` before the channel.
[[nodiscard]] Branches twoBranch(Rng& rng, std::size_t n, CF h0, CF h1, double signalPower, double noisePower) {
    Branches b;
    b.r0.resize(n);
    b.r1.resize(n);
    b.s.resize(n);
    const float sigS = static_cast<float>(std::sqrt(signalPower));
    const float sigN = static_cast<float>(std::sqrt(noisePower));
    for (std::size_t k = 0UZ; k < n; ++k) {
        const CF s  = sigS * rng.complexSample();
        const CF n0 = sigN * rng.complexSample();
        const CF n1 = sigN * rng.complexSample();
        b.s[k]      = s;
        b.r0[k]     = h0 * s + n0;
        b.r1[k]     = h1 * s + n1;
    }
    return b;
}

/// @brief The outputs, the records, the readers and the counters after one run, read while the scheduler holds the
/// graph.
struct Run {
    std::vector<CF>                 out;
    std::vector<CF>                 ortho;
    std::vector<gr::DataSet<float>> records;

    double relativePhase{};
    double amplitudeRatio{};
    double branch0Db{};
    double branch1Db{};
    double combinedDb{};
    double combiningGainDb{};
    int    selectedBranch{};
    double coverage{};

    std::uint64_t nWindows{};
    std::uint64_t nWindowResets{};
    std::uint64_t nBranchSwitches{};
    std::uint64_t nSaturatedFigures{};
    std::uint64_t nDroppedSampleTags{};

    bool endedOnItsOwn{true};

    std::complex<double> weight0{}, weight1{}, ortho0{}, ortho1{}; ///< the weights in use when the run ended
};

/// @brief Runs `scheduler` on its own thread and requests a stop after `bound`. Returns whether the run ended first.
template<typename TScheduler>
[[nodiscard]] bool runWithin(TScheduler& scheduler, std::chrono::milliseconds bound) {
    auto       running = std::async(std::launch::async, [&scheduler] { std::ignore = scheduler.runAndWait(); });
    const bool inTime  = running.wait_for(bound) == std::future_status::ready;
    if (!inTime) {
        scheduler.requestStop();
    }
    running.wait();
    return inTime;
}

/// @brief Runs two branches through `PolarizationCombiner` into collectors and a record sink. Without
/// @p connectOrtho, the graph holds no `ortho` collector.
[[nodiscard]] Run run(gr::property_map settings, const std::vector<CF>& r0, const std::vector<CF>& r1, std::size_t burst, bool connectOrtho = true) {
    gr::Graph      graph;
    auto&          src0   = graph.emplaceBlock<SampleSource<CF>>();
    auto&          src1   = graph.emplaceBlock<SampleSource<CF>>();
    auto&          block  = graph.emplaceBlock<PolarizationCombiner>(std::move(settings));
    auto&          outS   = graph.emplaceBlock<Collector<CF>>();
    auto&          recS   = graph.emplaceBlock<RecordSink>();
    Collector<CF>* orthoS = connectOrtho ? &graph.emplaceBlock<Collector<CF>>() : nullptr;

    src0.samples = r0;
    src0.burst   = burst;
    src1.samples = r1;
    src1.burst   = burst;

    bool connected = graph.connect<"out", "in0">(src0, block).has_value() && graph.connect<"out", "in1">(src1, block).has_value() && graph.connect<"out", "in">(block, outS).has_value() && graph.connect<"measurements", "in">(block, recS).has_value();
    if (orthoS != nullptr) {
        connected = connected && graph.connect<"ortho", "in">(block, *orthoS).has_value();
    }
    Run notRun;
    notRun.endedOnItsOwn = false;
    if (!connected) {
        return notRun;
    }
    gr::scheduler::Simple scheduler;
    if (!scheduler.exchange(std::move(graph)).has_value()) {
        return notRun;
    }
    // Every scene is at most a few hundred thousand samples and ends well within a second. A graph that does not end
    // within the bound fails an assertion.
    const bool inTime = runWithin(scheduler, std::chrono::seconds(3));

    Run result;
    result.endedOnItsOwn      = inTime;
    result.out                = std::move(outS.items);
    result.ortho              = orthoS != nullptr ? std::move(orthoS->items) : std::vector<CF>{};
    result.records            = std::move(recS.records);
    result.relativePhase      = block.relativePhase();
    result.amplitudeRatio     = block.amplitudeRatio();
    result.branch0Db          = block.branchSnrDb(0UZ);
    result.branch1Db          = block.branchSnrDb(1UZ);
    result.combinedDb         = block.combinedSnrDb();
    result.combiningGainDb    = block.combiningGainDb();
    result.selectedBranch     = block.selectedBranch();
    result.coverage           = block.coverage();
    result.nWindows           = block.nWindows();
    result.nWindowResets      = block.nWindowResets();
    result.nBranchSwitches    = block.nBranchSwitches();
    result.nSaturatedFigures  = block.nSaturatedFigures();
    result.nDroppedSampleTags = block.nDroppedSampleTags();
    result.weight0            = static_cast<std::complex<double>>(block._estimate.weight0);
    result.weight1            = static_cast<std::complex<double>>(block._estimate.weight1);
    result.ortho0             = static_cast<std::complex<double>>(block._estimate.ortho0);
    result.ortho1             = static_cast<std::complex<double>>(block._estimate.ortho1);
    return result;
}

[[nodiscard]] double metaNumber(const gr::DataSet<float>& record, std::string_view key, double fallback = -1.) {
    if (record.meta_information.empty()) {
        return fallback;
    }
    const auto it = record.meta_information[0UZ].find(std::pmr::string(key));
    if (it == record.meta_information[0UZ].end()) {
        return fallback;
    }
    if (const auto* asI32 = it->second.template get_if<std::int32_t>()) {
        return static_cast<double>(*asI32);
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
    return fallback;
}

[[nodiscard]] std::string metaString(const gr::DataSet<float>& record, std::string_view key) {
    if (record.meta_information.empty()) {
        return {};
    }
    const auto it = record.meta_information[0UZ].find(std::pmr::string(key));
    if (it == record.meta_information[0UZ].end()) {
        return {};
    }
    if (const auto* asString = it->second.template get_if<std::pmr::string>()) {
        return std::string(std::string_view(*asString));
    }
    return {};
}

/// @brief Starts @p block on the settings it was built with.
void startBlock(PolarizationCombiner& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

/// @brief Returns whether the block or its limits refuse @p settings, staged on a running block.
[[nodiscard]] bool refuses(PolarizationCombiner& block, gr::property_map settings) {
    std::ignore = block.settings().setStaged(std::move(settings));
    try {
        return !block.settings().applyStagedParameters().failedParameters.empty();
    } catch (const gr::exception&) {
        return true;
    }
}

/// @brief Stages @p settings on the running block and returns the settings the limits refused.
[[nodiscard]] gr::property_map refusedByLimits(PolarizationCombiner& block, gr::property_map settings) {
    std::ignore = block.settings().setStaged(std::move(settings));
    return block.settings().applyStagedParameters().failedParameters;
}

/// @brief The outputs and records of a block driven on test spans, the next stream index to offer and the calls made.
struct Driven {
    std::vector<CF>                 out;
    std::vector<CF>                 ortho;
    std::vector<gr::DataSet<float>> records;
    std::size_t                     at{0UZ};
    std::size_t                     calls{0UZ};
};

/// @brief Calls processBulk on test spans over `[driven.at, to)`, @p chunk samples per call, with @p recordSlots free
/// record slots. Stops at a call that takes nothing.
void drive(PolarizationCombiner& block, const std::vector<CF>& r0, const std::vector<CF>& r1, std::size_t to, std::size_t chunk, Driven& driven, bool recordsConnected = true, std::size_t recordSlots = 64UZ) {
    namespace shim = gr::blocks::testing::span;
    driven.out.resize(r0.size());
    driven.ortho.resize(r0.size());
    std::vector<gr::DataSet<float>> slots(recordSlots);
    while (driven.at < to) {
        const std::size_t                    n = std::min(chunk, to - driven.at);
        shim::InputSpan<CF>                  in0(std::span<const CF>(r0).subspan(driven.at, n), driven.at);
        shim::InputSpan<CF>                  in1(std::span<const CF>(r1).subspan(driven.at, n), driven.at);
        shim::OutputSpan<CF>                 outSpan(std::span<CF>(driven.out).subspan(driven.at, n), driven.at);
        shim::OutputSpan<CF>                 orthoSpan(std::span<CF>(driven.ortho).subspan(driven.at, n), driven.at);
        shim::OutputSpan<gr::DataSet<float>> recordSpan(std::span<gr::DataSet<float>>(slots), 0UZ, nullptr, recordsConnected);
        std::ignore = block.processBulk(in0, in1, outSpan, orthoSpan, recordSpan);
        ++driven.calls;
        driven.records.insert(driven.records.end(), slots.begin(), slots.begin() + static_cast<std::ptrdiff_t>(recordSpan.count));
        if (in0.consumed == 0UZ) {
            return;
        }
        driven.at += in0.consumed;
    }
}

/// @brief Calls processEpilogue with no trailing sample, the end of the stream.
void finish(PolarizationCombiner& block, Driven& driven) {
    namespace shim = gr::blocks::testing::span;
    std::vector<gr::DataSet<float>>      slots(64UZ);
    shim::InputSpan<CF>                  in0(std::span<const CF>{}, driven.at);
    shim::InputSpan<CF>                  in1(std::span<const CF>{}, driven.at);
    shim::OutputSpan<CF>                 outSpan(std::span<CF>{}, driven.at);
    shim::OutputSpan<CF>                 orthoSpan(std::span<CF>{}, driven.at);
    shim::OutputSpan<gr::DataSet<float>> recordSpan{std::span<gr::DataSet<float>>(slots)};
    std::ignore = block.processEpilogue(in0, in1, outSpan, orthoSpan, recordSpan);
    driven.records.insert(driven.records.end(), slots.begin(), slots.begin() + static_cast<std::ptrdiff_t>(recordSpan.count));
}

/// @brief The least-squares gain of a known reference in a measured stream and the residual power, over one range.
struct Projection {
    std::complex<double> gain{};          ///< the least-squares complex gain of @p reference in @p measured
    double               signalPower{};   ///< the power of the reference times the gain
    double               residualPower{}; ///< the power of the remainder
};

[[nodiscard]] Projection project(std::span<const CF> measured, std::span<const CF> reference, std::size_t from, std::size_t to) {
    std::complex<double> cross{};
    double               referencePower = 0.;
    for (std::size_t k = from; k < to; ++k) {
        const std::complex<double> r = static_cast<std::complex<double>>(reference[k]);
        cross += std::conj(r) * static_cast<std::complex<double>>(measured[k]);
        referencePower += std::norm(r);
    }
    Projection out;
    out.gain = referencePower > 0. ? cross / referencePower : std::complex<double>{};

    double residual = 0.;
    for (std::size_t k = from; k < to; ++k) {
        residual += std::norm(static_cast<std::complex<double>>(measured[k]) - out.gain * static_cast<std::complex<double>>(reference[k]));
    }
    const double n    = static_cast<double>(to - from);
    out.signalPower   = n > 0. ? std::norm(out.gain) * referencePower / n : 0.;
    out.residualPower = n > 0. ? residual / n : 0.;
    return out;
}

/// @brief The signal-to-noise ratio of @p measured against the known transmitted @p reference, in dB.
[[nodiscard]] double measuredSnrDb(std::span<const CF> measured, std::span<const CF> reference, std::size_t from, std::size_t to) {
    const Projection p = project(measured, reference, from, to);
    return (p.residualPower > 0.) ? 10. * std::log10(p.signalPower / p.residualPower) : std::numeric_limits<double>::infinity();
}

} // namespace qa_polcomb

const boost::ut::suite<"PolarizationCombiner"> polarizationCombinerTests = [] {
    using namespace boost::ut;
    using namespace qa_polcomb;

    // The gain over the better branch is 3.0103 dB at equal branches and 0.9691 dB with branch 1 at -6 dB. The
    // envelope is 0.05 dB. It covers the loss from estimating the weights, under 0.011 dB at M >= 512, and the spread
    // of the ratio measurement, 4.343/sqrt(N) dB, 0.018 dB over the 57 344 samples after the first window.
    "maximal-ratio gain over the better branch within 0.05 dB"_test = [] {
        constexpr std::size_t kN      = 65536UZ;
        constexpr gr::Size_t  kWindow = 8192U; // shorter than the stream, and the windows after the first are combined
        constexpr std::size_t kFrom   = static_cast<std::size_t>(kWindow);
        Rng                   rng(0x243f6a8885a308d3ULL);

        // The ratios are measured on the streams against the transmitted signal, over the windows after the first.
        // Window 0 passes branch 0 through.
        const auto arm = [kWindow](const Branches& b, double nominalGainDb, std::string_view what) {
            const Run r = run({{"window", kWindow}}, b.r0, b.r1, 8192UZ);
            expect(r.endedOnItsOwn) << what;
            expect(eq(r.out.size(), b.s.size())) << what;
            if (r.out.size() != b.s.size()) {
                return;
            }
            const double snrOut     = measuredSnrDb(r.out, b.s, kFrom, b.s.size());
            const double snrBranch0 = measuredSnrDb(b.r0, b.s, kFrom, b.s.size());
            const double snrBranch1 = measuredSnrDb(b.r1, b.s, kFrom, b.s.size());
            const double best       = std::max(snrBranch0, snrBranch1);
            const double gain       = snrOut - best;

            std::println("MRC gain ({}): measured out {:.4f} dB, branches {:.4f} / {:.4f} dB, gain {:.4f} dB (nominal {:.4f}); block reports {:.4f} dB", what, snrOut, snrBranch0, snrBranch1, gain, nominalGainDb, r.combiningGainDb);
            expect(std::isfinite(snrOut) && std::isfinite(best)) << std::format("{}: the measured ratios are finite", what);
            expect(std::abs(gain - nominalGainDb) < 0.05) << std::format("{}: the measured gain is outside its 0.05 dB envelope", what);
            expect(std::abs(r.combiningGainDb - nominalGainDb) < 0.05) << std::format("{}: the reported gain is outside its 0.05 dB envelope", what);
        };

        arm(twoBranch(rng, kN, CF{1.f, 0.f}, CF(static_cast<float>(std::cos(0.7)), static_cast<float>(std::sin(0.7))), 1., 0.1), 3.0103, "equal branches");
        arm(twoBranch(rng, kN, CF{1.f, 0.f}, 0.5f * CF(static_cast<float>(std::cos(2.)), static_cast<float>(std::sin(2.))), 1., 0.1), 0.9691, "-6 dB relative branch");
    };

    // Seeded data with no channel model. The two branch ratios sum in linear terms to the combined ratio within 1e-9
    // relative.
    "branchSnrDb(0) and branchSnrDb(1) sum in linear terms to combinedSnrDb()"_test = [] {
        Rng             rng(0x9e3779b97f4a7c15ULL);
        std::vector<CF> r0(8192UZ);
        std::vector<CF> r1(8192UZ);
        std::ranges::generate(r0, [&] { return rng.complexSample(); });
        std::ranges::generate(r1, [&] { return rng.complexSample(); });

        const Run r = run({{"window", gr::Size_t{8192U}}}, r0, r1, 4096UZ);

        // A ratio of zero reads 0 dB, the placeholder of an unmeasured figure. Each figure is checked finite and
        // positive before the sum.
        std::println("branch0 {:.6f} dB, branch1 {:.6f} dB, combined {:.6f} dB", r.branch0Db, r.branch1Db, r.combinedDb);
        expect(std::isfinite(r.branch0Db) && std::isfinite(r.branch1Db) && std::isfinite(r.combinedDb)) << "every figure is finite";
        expect(r.combinedDb > r.branch0Db && r.combinedDb > r.branch1Db) << "the combined figure exceeds either branch's";
        if (!r.records.empty()) {
            expect(eq(metaNumber(r.records.back(), "valid"), 1.)) << "the record of this window is valid";
        }

        const double linear0  = std::pow(10., r.branch0Db / 10.);
        const double linear1  = std::pow(10., r.branch1Db / 10.);
        const double sum      = linear0 + linear1;
        const double combined = std::pow(10., r.combinedDb / 10.);
        std::println("branch0 {:.6f} + branch1 {:.6f} = {:.6f}, combined {:.6f}", linear0, linear1, sum, combined);
        expect(linear0 > 0. && linear1 > 0. && combined > 0.) << "each ratio is positive";
        expect(std::abs(sum - combined) < 1e-9 * std::max(1., combined)) << "the branch ratios sum to the combined ratio within 1e-9 relative";
    };

    "relativePhase and amplitudeRatio recover the channel within 4/sqrt(M)"_test = [] {
        constexpr std::size_t kM = 262144UZ;
        Rng                   rng(0x1234567890abcdefULL);
        const Branches        b = twoBranch(rng, kM, CF{1.f, 0.f}, 0.5f * CF(static_cast<float>(std::cos(2.)), static_cast<float>(std::sin(2.))), 1., 0.1);

        const Run    r         = run({{"window", gr::Size_t{static_cast<gr::Size_t>(kM)}}}, b.r0, b.r1, 8192UZ);
        const double tolerance = 4. / std::sqrt(static_cast<double>(kM));
        std::println("relativePhase {:.5f} (nominal -2, tol {:.4f}), amplitudeRatio {:.5f} (nominal 2)", r.relativePhase, tolerance, r.amplitudeRatio);
        expect(std::abs(r.relativePhase - (-2.)) < tolerance) << "relativePhase reads arg(h0) - arg(h1) = -2 within 4/sqrt(M)";
        expect(std::abs(r.amplitudeRatio - 2.) < tolerance) << "amplitudeRatio reads |h0|/|h1| = 2 within the same tolerance";
    };

    // The orthogonal output carries the transmitted signal at least 40 dB below `out`. The orthogonal weights null
    // the channel the scene applied. `w^H u` is zero for any `w` and does not test that.
    "the orthogonal output nulls the signal and the applied channel"_test = [] {
        constexpr std::size_t kWindow  = 8192UZ;
        constexpr std::size_t kWindows = 3UZ;
        Rng                   rng(0xd1b54a32d192ed03ULL);
        const CF              h0 = CF{1.f, 0.f};
        const CF              h1 = 0.5f * CF(static_cast<float>(std::cos(2.)), static_cast<float>(std::sin(2.)));
        const Branches        b  = twoBranch(rng, kWindow * kWindows, h0, h1, 1., 0.02);

        const Run r = run({{"window", gr::Size_t{kWindow}}}, b.r0, b.r1, 4096UZ);
        expect(eq(r.out.size(), b.s.size()));
        expect(eq(r.ortho.size(), b.s.size()));
        if (r.out.size() != b.s.size() || r.ortho.size() != b.s.size()) {
            return;
        }

        // Window 0 passes the branches through. Windows 1 and 2 use weights solved from the stationary channel.
        std::complex<double> corrOut{};
        std::complex<double> corrOrtho{};
        for (std::size_t k = kWindow; k < kWindow * kWindows; ++k) {
            corrOut += std::conj(static_cast<std::complex<double>>(b.s[k])) * static_cast<std::complex<double>>(r.out[k]);
            corrOrtho += std::conj(static_cast<std::complex<double>>(b.s[k])) * static_cast<std::complex<double>>(r.ortho[k]);
        }
        const double ratioDb = 20. * std::log10(std::abs(corrOrtho) / std::abs(corrOut));
        std::println("|corr(s,ortho)| / |corr(s,out)| = {:.2f} dB", ratioDb);
        expect(ratioDb < -40.) << "the orthogonal output carries the transmitted signal at least 40 dB below out";

        // The orthogonal weights null the channel the scene applied, and the combining weights pass it.
        const std::complex<double> trueH0       = static_cast<std::complex<double>>(h0);
        const std::complex<double> trueH1       = static_cast<std::complex<double>>(h1);
        const double               throughOut   = std::abs(std::conj(r.weight0) * trueH0 + std::conj(r.weight1) * trueH1);
        const double               throughOrtho = std::abs(std::conj(r.ortho0) * trueH0 + std::conj(r.ortho1) * trueH1);
        const double               nullDb       = 20. * std::log10(throughOrtho / throughOut);
        std::println("|u^H h| / |w^H h| = {:.2f} dB (out gain {:.5f}, ortho gain {:.3e})", nullDb, throughOut, throughOrtho);
        expect(throughOut > 0.5) << "the combining weights pass the applied channel";
        expect(nullDb < -40.) << "the orthogonal weights null the applied channel by at least 40 dB";
    };

    // The branch powers cross once. selection switches once, and mrc reads a positive gain in every window.
    "selection switches once at a branch crossover, and mrc reads a positive gain throughout"_test = [] {
        constexpr std::size_t kWindow  = 2048UZ;
        constexpr std::size_t kWindows = 4UZ;
        Rng                   rng(0x2545f4914f6cdd1dULL);

        std::vector<CF> r0(kWindow * kWindows);
        std::vector<CF> r1(kWindow * kWindows);
        for (std::size_t w = 0UZ; w < kWindows; ++w) {
            const bool branch0Louder = w < 2UZ;
            const CF   h0            = branch0Louder ? CF{2.f, 0.f} : CF{0.4f, 0.f};
            const CF   h1            = branch0Louder ? CF{0.4f, 0.f} : CF{2.f, 0.f};
            for (std::size_t k = 0UZ; k < kWindow; ++k) {
                const CF s          = rng.complexSample();
                const CF n0         = 0.2f * rng.complexSample();
                const CF n1         = 0.2f * rng.complexSample();
                r0[w * kWindow + k] = h0 * s + n0;
                r1[w * kWindow + k] = h1 * s + n1;
            }
        }

        const Run selRun = run({{"window", gr::Size_t{kWindow}}, {"mode", std::string("selection")}}, r0, r1, 4096UZ);
        std::println("selection nBranchSwitches() = {}", selRun.nBranchSwitches);
        expect(eq(selRun.nBranchSwitches, std::uint64_t{1ULL})) << "the branch powers cross once";

        const Run mrcRun = run({{"window", gr::Size_t{kWindow}}, {"mode", std::string("mrc")}, {"emit_records", true}}, r0, r1, 4096UZ);
        expect(ge(mrcRun.records.size(), 3UZ)) << "the three windows after the first publish records";
        for (std::size_t i = 1UZ; i < mrcRun.records.size(); ++i) { // record 0 is the solve over window 0
            const float gainDb = mrcRun.records[i].signal_values.at(5UZ);
            std::println("mrc record {} combining_gain_db = {:.3f}", i, gainDb);
            expect(gainDb > 0.f) << std::format("record {}: mrc reads a positive gain over the stronger branch", i);
        }
    };

    "saturation is counted at both bounds, and an unbounded amplitude ratio passes branch 0 unchanged"_test = [] {
        // (a) Two noiseless branches. The covariance entries are small integers, and lambda_- is exactly zero. The
        // ratios read the cap, 60 dB or 1e6 linear.
        {
            constexpr std::size_t kWindow = 1024UZ;
            std::vector<CF>       r0(kWindow, CF{1.f, 0.f});
            std::vector<CF>       r1(kWindow, CF{2.f, 0.f});
            const Run             r = run({{"window", gr::Size_t{kWindow}}}, r0, r1, 4096UZ);
            std::println("noiseless branchSnrDb(0)={:.3f} combinedSnrDb={:.3f} nSaturatedFigures={}", r.branch0Db, r.combinedDb, r.nSaturatedFigures);
            expect(r.nSaturatedFigures > 0ULL) << "a noiseless input saturates at least one figure";
            expect(std::abs(r.combinedDb - 60.) < 1e-6) << "the saturated figure reads 60 dB, 1e6 linear";
        }

        // (b) Branch 1 carries a deterministic sequence orthogonal to branch 0. The cross-covariance is exactly zero.
        // The amplitude ratio saturates, and the weights select branch 0.
        {
            constexpr std::size_t kWindow  = 1024UZ;
            constexpr std::size_t kWindows = 3UZ;
            std::vector<CF>       r0(kWindow * kWindows);
            std::vector<CF>       r1(kWindow * kWindows);
            for (std::size_t k = 0UZ; k < r0.size(); ++k) {
                r0[k] = (k % 2UZ == 0UZ) ? CF{1.f, 0.f} : CF{-1.f, 0.f};         // period 2
                r1[k] = ((k / 2UZ) % 2UZ == 0UZ) ? CF{1.f, 0.f} : CF{-1.f, 0.f}; // period 4, orthogonal to r0 over any multiple of 4
            }
            const Run r = run({{"window", gr::Size_t{kWindow}}}, r0, r1, 4096UZ);
            expect(eq(r.out.size(), r0.size()));
            if (r.out.size() != r0.size()) {
                return;
            }
            std::println("orthogonal-branch-1 amplitudeRatio={:.3f} nSaturatedFigures={} branch0Db={:.3f} combiningGainDb={:.3f}", r.amplitudeRatio, r.nSaturatedFigures, r.branch0Db, r.combiningGainDb);
            expect(r.nSaturatedFigures > 0ULL) << "an uncorrelated branch 1 saturates the amplitude ratio";
            expect(std::abs(r.amplitudeRatio - 1e6) < 1.) << "the saturated ratio reads 1e6";
            for (std::size_t k = kWindow; k < r0.size(); ++k) { // the first window passes branch 0 through
                expect(eq(r.out[k], r0[k])) << std::format("sample {}: with branch 1 uncorrelated, out equals branch 0 bit for bit", k);
            }

            // Every estimated signal power in this scene is zero. Each decibel figure is at the floor, and each reader
            // and record channel reads a finite placeholder.
            expect(std::isfinite(r.branch0Db) && std::isfinite(r.branch1Db) && std::isfinite(r.combinedDb)) << "the readers carry finite placeholders";
            expect(std::isfinite(r.combiningGainDb)) << "combiningGainDb() is finite";
            expect(!r.records.empty()) << "the windows publish records";
            for (std::size_t i = 0UZ; i < r.records.size(); ++i) {
                for (std::size_t c = 0UZ; c < 6UZ; ++c) {
                    expect(std::isfinite(r.records[i].signal_values.at(c))) << std::format("record {} channel {} is finite, read {}", i, c, r.records[i].signal_values.at(c));
                }
                expect(eq(metaNumber(r.records[i], "valid"), 0.)) << std::format("record {}: a window with a replaced figure is not valid", i);
            }
        }
    };

    "every record states the sample_rate setting, the mode, the normalization and the window"_test = [] {
        Rng             rng(0x9e3779b97f4a7c15ULL);
        std::vector<CF> r0(4096UZ);
        std::vector<CF> r1(4096UZ);
        std::ranges::generate(r0, [&] { return rng.complexSample(); });
        std::ranges::generate(r1, [&] { return rng.complexSample(); });

        const Run r = run({{"window", gr::Size_t{2048U}}, {"sample_rate", 192000.f}}, r0, r1, 1024UZ);
        expect(!r.records.empty());
        for (const auto& record : r.records) {
            expect(std::abs(metaNumber(record, "sample_rate") - 192000.) < 1e-3) << "every record carries the rate";
            expect(eq(metaString(record, "mode"), std::string("mrc")));
            expect(eq(metaString(record, "normalize"), std::string("unit_noise")));
            expect(eq(metaNumber(record, "window"), 2048.));
        }
    };

    "values outside the limits and unknown settings are refused, and the window floor is accepted"_test = [] {
        const std::array<gr::property_map, 6UZ> outside{{
            {{"window", gr::Size_t{63U}}},
            {{"window", gr::Size_t{(1U << 24U) + 1U}}},
            {{"weight_smoothing", 1.0}},
            {{"weight_smoothing", -0.1}},
            {{"sample_rate", 0.f}},
            {{"sample_rate", std::numeric_limits<float>::quiet_NaN()}},
        }};
        for (std::size_t r = 0UZ; r < outside.size(); ++r) {
            PolarizationCombiner block;
            startBlock(block);
            expect(throws([&] { std::ignore = block.settings().set(outside[r]); })) << std::format("value {}: settings().set() refuses it at the call", r);
            expect(eq(refusedByLimits(block, outside[r]).size(), 1UZ)) << std::format("value {}: the limits refuse a staged one", r);
            expect(eq(block.window.value, gr::Size_t{4096U}) && eq(block.weight_smoothing.value, 0.0) && eq(block.sample_rate.value, 96000.f)) << std::format("value {}: the members keep the accepted values", r);
        }
        PolarizationCombiner floor;
        startBlock(floor);
        expect(refusedByLimits(floor, {{"window", gr::Size_t{64U}}}).empty()) << "the floor of 64 is accepted";

        const auto staged = [](gr::property_map settings) {
            PolarizationCombiner block(std::move(settings));
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        };
        expect(throws([&] { staged({{"mode", std::string("diversity")}}); })) << "mode takes 'mrc' or 'selection'";
        expect(throws([&] { staged({{"normalize", std::string("unit_power")}}); })) << "normalize takes 'unit_noise' or 'unit_signal'";
        expect(throws([&] { staged({{"noise_powers", std::vector<double>{1.}}}); })) << "two branches take two noise powers";
        expect(throws([&] { staged({{"noise_powers", std::vector<double>{1., 0.}}}); })) << "a zero noise power is refused";
        expect(nothrow([&] { staged({{"noise_powers", std::vector<double>{0.1, 0.5}}}); }));
    };

    "a sample_rate outside its limits on a tag is refused, and the run goes on at the accepted rate"_test = [] {
        // The tag sits on branch 0 at a window boundary. The valid arm shows that a sample_rate tag takes effect.
        constexpr std::size_t kWindow = 1024UZ;
        constexpr std::size_t kAt     = 2UZ * kWindow;
        Rng                   rng(0xcafeULL);
        const Branches        b      = twoBranch(rng, 4UZ * kWindow, CF{1.f, 0.f}, CF{0.5f, 0.f}, 1., 0.1);
        const auto            tagged = [&b](float rate) {
            gr::Graph graph;
            auto&     src0   = graph.emplaceBlock<SampleSource<CF>>();
            auto&     src1   = graph.emplaceBlock<SampleSource<CF>>();
            auto&     block  = graph.emplaceBlock<PolarizationCombiner>({{"window", gr::Size_t{kWindow}}});
            auto&     outS   = graph.emplaceBlock<Collector<CF>>();
            auto&     recS   = graph.emplaceBlock<RecordSink>();
            src0.samples     = b.r0;
            src0.burst       = 512UZ;
            src0.settingsTag = {{"sample_rate", rate}};
            src0.settingsAt  = kAt;
            src1.samples     = b.r1;
            src1.burst       = 512UZ;

            struct Outcome {
                bool                            completed{};
                std::size_t                     out{};
                std::vector<gr::DataSet<float>> records;
                float                           member{};
            } outcome;
            if (!graph.connect<"out", "in0">(src0, block).has_value() || !graph.connect<"out", "in1">(src1, block).has_value() || !graph.connect<"out", "in">(block, outS).has_value() || !graph.connect<"measurements", "in">(block, recS).has_value()) {
                return outcome;
            }
            gr::scheduler::Simple scheduler;
            if (!scheduler.exchange(std::move(graph)).has_value()) {
                return outcome;
            }
            auto running = std::async(std::launch::async, [&scheduler] { return scheduler.runAndWait().has_value(); });
            if (running.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
                scheduler.requestStop();
            }
            outcome.completed = running.get();
            outcome.out       = outS.items.size();
            outcome.records   = std::move(recS.records);
            outcome.member    = block.sample_rate.value;
            return outcome;
        };

        const auto valid = tagged(48000.f);
        expect(valid.completed);
        expect(fatal(eq(valid.records.size(), 4UZ)));
        expect(eq(metaNumber(valid.records[1UZ], "sample_rate"), 96000.) && eq(metaNumber(valid.records[2UZ], "sample_rate"), 48000.)) << "a valid tag moves the rate from its sample on";

        for (const float rate : {-1.f, std::numeric_limits<float>::quiet_NaN()}) {
            const auto refused = tagged(rate);
            expect(refused.completed) << std::format("rate {}: the run completes", rate);
            expect(eq(refused.out, b.r0.size())) << std::format("rate {}: every sample is combined", rate);
            expect(eq(refused.records.size(), 4UZ)) << std::format("rate {}: every window is read", rate);
            for (const auto& record : refused.records) {
                expect(eq(metaNumber(record, "sample_rate"), 96000.)) << std::format("rate {}: every record states the accepted rate", rate);
            }
            expect(eq(refused.member, 96000.f)) << std::format("rate {}: the member keeps the accepted rate", rate);
        }
    };

    // unit_noise keeps ||v|| = 1, and the output signal power is P|h|^2. unit_signal scales it to 1.
    "unit_noise and unit_signal scale the output signal power to P|h|^2 and to 1"_test = [] {
        constexpr std::size_t kWindow  = 4096UZ;
        constexpr std::size_t kWindows = 4UZ;
        Rng                   rng(0x1234567890abcdefULL);
        const CF              h1 = 0.5f * CF(static_cast<float>(std::cos(2.)), static_cast<float>(std::sin(2.)));
        const Branches        b  = twoBranch(rng, kWindow * kWindows, CF{1.f, 0.f}, h1, 1., 0.05);

        const auto outputSignalPower = [&b](std::string_view normalize) {
            const Run r = run({{"window", gr::Size_t{kWindow}}, {"normalize", std::string(normalize)}}, b.r0, b.r1, 4096UZ);
            expect(eq(r.out.size(), b.s.size())) << normalize;
            return r.out.size() == b.s.size() ? project(r.out, b.s, kWindow, b.s.size()).signalPower : 0.;
        };

        const double noisePower  = outputSignalPower("unit_noise");
        const double signalPower = outputSignalPower("unit_signal");
        std::println("normalize: unit_noise output signal power {:.5f} (nominal |h|^2 = 1.25), unit_signal {:.5f} (nominal 1)", noisePower, signalPower);
        // The mean |s|^2 of the projection has a relative spread of about 1/sqrt(12288), 0.9 percent, over the
        // measured range. Each window's weights come from the window before it.
        expect(std::abs(noisePower - 1.25) < 0.05) << "under unit_noise the output signal power is P|h|^2 = 1.25";
        expect(std::abs(signalPower - 1.) < 0.05) << "unit_signal scales the output signal power to 1";
        expect(std::abs(noisePower - signalPower) > 0.1) << "the two normalizations differ on this stream";
    };

    // The blend runs on the whitened weights, and the orthogonal weights rebuilt from it null the signal under unequal
    // branch noise.
    "weight_smoothing under unequal branch noise keeps the orthogonal null"_test = [] {
        constexpr std::size_t kWindow  = 4096UZ;
        constexpr std::size_t kWindows = 5UZ;
        constexpr double      kNoise0  = 0.02;
        constexpr double      kNoise1  = 0.20; // a ten-to-one noise imbalance

        Rng             rng(0x2545f4914f6cdd1dULL);
        const CF        h0 = CF{1.f, 0.f};
        const CF        h1 = 0.5f * CF(static_cast<float>(std::cos(2.)), static_cast<float>(std::sin(2.)));
        std::vector<CF> r0(kWindow * kWindows);
        std::vector<CF> r1(kWindow * kWindows);
        std::vector<CF> s(kWindow * kWindows);
        for (std::size_t k = 0UZ; k < s.size(); ++k) {
            const CF sample = rng.complexSample();
            s[k]            = sample;
            r0[k]           = h0 * sample + static_cast<float>(std::sqrt(kNoise0)) * rng.complexSample();
            r1[k]           = h1 * sample + static_cast<float>(std::sqrt(kNoise1)) * rng.complexSample();
        }

        const Run r = run({{"window", gr::Size_t{kWindow}}, {"noise_powers", std::vector<double>{kNoise0, kNoise1}}, {"weight_smoothing", 0.5}}, r0, r1, 4096UZ);
        expect(r.endedOnItsOwn);
        expect(eq(r.out.size(), s.size()));
        if (r.out.size() != s.size()) {
            return;
        }

        // Measured over the windows whose weights come from the blend, window 2 onward.
        const double throughOut   = std::abs(project(r.out, s, 2UZ * kWindow, s.size()).gain);
        const double throughOrtho = std::abs(project(r.ortho, s, 2UZ * kWindow, s.size()).gain);
        const double nullDb       = 20. * std::log10(throughOrtho / throughOut);
        std::println("weight_smoothing: out gain {:.5f}, ortho gain {:.3e}, null {:.2f} dB", throughOut, throughOrtho, nullDb);
        expect(throughOut > 0.5) << "the combined output carries the signal";
        expect(nullDb < -30.) << "the smoothed orthogonal output nulls the signal by at least 30 dB";

        // The whitened combination reaches |h_0|^2/N_0 + |h_1|^2/N_1 = 1/0.02 + 0.25/0.2 = 51.25, 17.10 dB.
        const double snrOut = measuredSnrDb(r.out, s, 2UZ * kWindow, s.size());
        std::println("weight_smoothing: measured output SNR {:.3f} dB (optimum 10*log10(1/0.02 + 0.25/0.2) = {:.3f} dB)", snrOut, 10. * std::log10(1. / kNoise0 + 0.25 / kNoise1));
        expect(std::abs(snrOut - 10. * std::log10(1. / kNoise0 + 0.25 / kNoise1)) < 0.3) << "the whitened combination reaches the unequal-noise optimum within 0.3 dB";
    };

    "emit_records false measures and publishes no record"_test = [] {
        Rng             rng(0x5deece66dULL);
        std::vector<CF> r0(8192UZ);
        std::vector<CF> r1(8192UZ);
        std::ranges::generate(r0, [&] { return rng.complexSample(); });
        std::ranges::generate(r1, [&] { return rng.complexSample(); });

        const Run r = run({{"window", gr::Size_t{2048U}}, {"emit_records", false}}, r0, r1, 4096UZ);
        expect(eq(r.records.size(), 0UZ)) << "no record is published";
        expect(eq(r.nWindows, std::uint64_t{4ULL})) << "the four windows are measured and counted";
        expect(std::abs(r.coverage - 1.) < 1e-12) << "the readers carry the last complete window";
        expect(eq(r.out.size(), r0.size())) << "out carries every sample";
    };

    // With records on, a record output with no free slot holds back the sample that closes a window. Records turned
    // off at that point leave nothing for the record output to hold back.
    "records turned off on a full record output leave every sample combined"_test = [] {
        constexpr std::size_t kWindow = 64UZ;
        Rng                   rng(0x2545f4914f6cdd1dULL);
        const Branches        b = twoBranch(rng, 16UZ * kWindow, CF{1.f, 0.f}, CF{0.5f, 0.f}, 1., 0.1);

        PolarizationCombiner block({{"window", gr::Size_t{kWindow}}, {"emit_records", true}});
        startBlock(block);
        Driven driven;
        drive(block, b.r0, b.r1, b.r0.size(), b.r0.size(), driven, true, 0UZ);
        expect(eq(driven.at, kWindow - 1UZ)) << "with records on, the sample that closes the first window waits for a record slot";

        expect(!refuses(block, {{"emit_records", false}})) << "records are turned off";
        drive(block, b.r0, b.r1, b.r0.size(), b.r0.size(), driven, true, 0UZ);
        expect(eq(driven.at, b.r0.size())) << "with records off, the full record output holds back no sample";
        expect(eq(block.nWindows(), std::uint64_t{16ULL})) << "every window is measured";
        expect(driven.records.empty()) << "and no record is published";
    };

    // At the largest window, 256 free record slots give room for 2^32 samples, beyond a 32-bit size.
    "at the largest window, 256 free record slots hold back no sample"_test = [] {
        constexpr std::size_t kOffer = 4096UZ;
        Rng                   rng(0x9e3779b97f4a7c15ULL);
        const Branches        b = twoBranch(rng, kOffer, CF{1.f, 0.f}, CF{0.5f, 0.f}, 1., 0.1);

        PolarizationCombiner block({{"window", gr::Size_t{1U << 24U}}});
        startBlock(block);
        Driven driven;
        drive(block, b.r0, b.r1, kOffer, kOffer, driven, true, 256UZ);
        expect(eq(driven.calls, 1UZ) && eq(driven.at, kOffer)) << "one call takes every sample offered";
    };

    // Two cases: nothing connected to `ortho`, and `ortho` connected with emit_orthogonal false. The combined output
    // completes in both.
    "an unconnected ortho or a cleared emit_orthogonal leaves the combined output complete"_test = [] {
        Rng             rng(0xbf58476d1ce4e5b9ULL);
        std::vector<CF> r0(8192UZ);
        std::vector<CF> r1(8192UZ);
        std::ranges::generate(r0, [&] { return rng.complexSample(); });
        std::ranges::generate(r1, [&] { return rng.complexSample(); });

        {
            const Run r = run({{"window", gr::Size_t{2048U}}}, r0, r1, 4096UZ, false);
            expect(r.endedOnItsOwn) << "the graph ends with ortho unconnected";
            expect(eq(r.out.size(), r0.size())) << "the combined output is complete with ortho unconnected";
            expect(eq(r.nWindows, std::uint64_t{4ULL}));
        }
        {
            const Run r = run({{"window", gr::Size_t{2048U}}, {"emit_orthogonal", false}}, r0, r1, 4096UZ, true);
            expect(r.endedOnItsOwn) << "the graph ends with emit_orthogonal false and ortho connected";
            expect(eq(r.out.size(), r0.size())) << "the combined output is complete";
            expect(eq(r.ortho.size(), 0UZ)) << "the orthogonal port publishes nothing";
            expect(eq(r.nWindows, std::uint64_t{4ULL}));
        }
    };

    "before the first window, out and ortho carry the two branches unchanged, and coverage() reads 0"_test = [] {
        // A graph run ends with the end-of-stream flush, which sets the readers. The readers before any sample are
        // read on a started block alone.
        PolarizationCombiner block;
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
        expect(eq(block.coverage(), 0.));
        expect(eq(block.relativePhase(), 0.));
        expect(eq(block.amplitudeRatio(), 0.));
        expect(eq(block.nWindows(), std::uint64_t{0ULL}));

        // A stream shorter than one window. The window does not close, and every sample passes through.
        Rng             rng(0x0f0f0f0f0f0f0f0fULL);
        std::vector<CF> r0(2000UZ);
        std::vector<CF> r1(2000UZ);
        std::ranges::generate(r0, [&] { return rng.complexSample(); });
        std::ranges::generate(r1, [&] { return rng.complexSample(); });

        const Run r = run({{"window", gr::Size_t{4096U}}}, r0, r1, 777UZ);
        expect(eq(r.out.size(), r0.size()));
        expect(eq(r.ortho.size(), r1.size()));
        for (std::size_t k = 0UZ; k < r0.size(); ++k) {
            expect(eq(r.out[k], r0[k])) << std::format("sample {}: out equals branch 0 bit for bit before the first window closes", k);
            expect(eq(r.ortho[k], r1[k])) << std::format("sample {}: ortho equals branch 1 bit for bit before the first window closes", k);
        }
    };

    "any burst size gives bit-identical outputs, records and counters"_test = [] {
        Rng            rng(0x5deece66dULL);
        const Branches b         = twoBranch(rng, 3UZ * 1024UZ + 777UZ, CF{1.f, 0.f}, 0.6f * CF(static_cast<float>(std::cos(1.1)), static_cast<float>(std::sin(1.1))), 1., 0.15);
        const auto     settings  = gr::property_map{{"window", gr::Size_t{1024U}}, {"emit_records", true}};
        const Run      reference = run(settings, b.r0, b.r1, 1024UZ);

        for (const std::size_t burst : {1UZ, 7UZ, 1000UZ, 12345UZ}) {
            const Run candidate = run(settings, b.r0, b.r1, burst);
            expect(eq(candidate.out.size(), reference.out.size())) << std::format("burst {}: out size", burst);
            expect(std::ranges::equal(candidate.out, reference.out)) << std::format("burst {}: out stream differs", burst);
            expect(std::ranges::equal(candidate.ortho, reference.ortho)) << std::format("burst {}: ortho stream differs", burst);
            expect(eq(candidate.nWindows, reference.nWindows)) << std::format("burst {}: nWindows", burst);
            expect(eq(candidate.nWindowResets, reference.nWindowResets)) << std::format("burst {}: nWindowResets", burst);
            expect(eq(candidate.nBranchSwitches, reference.nBranchSwitches)) << std::format("burst {}: nBranchSwitches", burst);
            expect(eq(candidate.nSaturatedFigures, reference.nSaturatedFigures)) << std::format("burst {}: nSaturatedFigures", burst);
            expect(eq(candidate.nDroppedSampleTags, reference.nDroppedSampleTags)) << std::format("burst {}: nDroppedSampleTags", burst);
            expect(eq(candidate.combiningGainDb, reference.combiningGainDb)) << std::format("burst {}: combiningGainDb", burst);
            expect(eq(candidate.records.size(), reference.records.size())) << std::format("burst {}: record count", burst);
            for (std::size_t r = 0UZ; r < std::min(candidate.records.size(), reference.records.size()); ++r) {
                expect(std::ranges::equal(candidate.records[r].signal_values, reference.records[r].signal_values)) << std::format("burst {}: record {}", burst, r);
                expect(eq(metaNumber(candidate.records[r], "selected_branch"), metaNumber(reference.records[r], "selected_branch"))) << std::format("burst {}: record {}", burst, r);
            }
        }
    };

    "a silent branch gives records that say they are saturated and not valid"_test = [] {
        constexpr std::size_t kWindow = 4096UZ;
        Rng                   rng(0x1111ULL);
        const Branches        b = twoBranch(rng, 3UZ * kWindow, CF{1.f, 0.f}, CF{0.f, 0.f}, 1., 0.1);
        const std::vector<CF> zeros(b.r0.size());

        const Run r = run({{"window", gr::Size_t{kWindow}}}, b.r0, zeros, 4096UZ);
        expect(eq(r.records.size(), 3UZ));
        for (std::size_t i = 0UZ; i < r.records.size(); ++i) {
            const auto& values = r.records[i].signal_values;
            std::println("silent branch 1, record {}: branch0 {} dB, branch1 {} dB, combined {} dB, saturated_snr {}, valid {}", i, values.at(2UZ), values.at(3UZ), values.at(4UZ), metaNumber(r.records[i], "saturated_snr"), metaNumber(r.records[i], "valid"));
            expect(eq(metaNumber(r.records[i], "saturated_snr"), 1.)) << std::format("record {} says its ratios are unbounded", i);
            expect(eq(metaNumber(r.records[i], "valid"), 0.)) << std::format("record {} is not valid", i);
            expect(eq(values.at(3UZ), 0.f)) << std::format("record {}: the silent branch reads the placeholder of a zero ratio", i);
            expect(std::abs(values.at(4UZ) - 60.f) < 1e-4f) << std::format("record {}: the combined ratio reads the cap", i);
        }
        expect(r.nSaturatedFigures > 0ULL);
    };

    "a noiseless input reads the cap in records that say they are saturated and not valid"_test = [] {
        constexpr std::size_t kWindow = 1024UZ;
        std::vector<CF>       r0(3UZ * kWindow);
        std::vector<CF>       r1(r0.size());
        const CF              g = std::polar(0.3f, -1.1f); // rounding leaves lambda_- positive for this gain
        for (std::size_t k = 0UZ; k < r0.size(); ++k) {
            const double phase = 2. * std::numbers::pi * 0.031 * static_cast<double>(k);
            r0[k]              = CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
            r1[k]              = g * r0[k];
        }

        const Run r = run({{"window", gr::Size_t{kWindow}}}, r0, r1, 4096UZ);
        expect(eq(r.records.size(), 3UZ));
        for (std::size_t i = 0UZ; i < r.records.size(); ++i) {
            std::println("noiseless, record {}: combined {} dB, saturated_snr {}, valid {}", i, r.records[i].signal_values.at(4UZ), metaNumber(r.records[i], "saturated_snr"), metaNumber(r.records[i], "valid"));
            expect(std::abs(r.records[i].signal_values.at(4UZ) - 60.f) < 1e-4f) << std::format("record {} reads the cap", i);
            expect(eq(metaNumber(r.records[i], "saturated_snr"), 1.)) << std::format("record {} says so", i);
            expect(eq(metaNumber(r.records[i], "valid"), 0.)) << std::format("record {} is not valid", i);
        }
    };

    // Branch 1 at -6 dB. The selected branch passes alone, scaled to unit signal power.
    "under selection the records give the selected branch's ratio and no gain, and unit_signal gives unit power"_test = [] {
        constexpr std::size_t kWindow = 4096UZ;
        Rng                   rng(0x3333ULL);
        const Branches        b = twoBranch(rng, 4UZ * kWindow, CF{1.f, 0.f}, 0.5f * CF(static_cast<float>(std::cos(0.7)), static_cast<float>(std::sin(0.7))), 1., 0.1);

        const Run r = run({{"window", gr::Size_t{kWindow}}, {"mode", std::string("selection")}, {"normalize", std::string("unit_signal")}}, b.r0, b.r1, 4096UZ);
        expect(eq(r.out.size(), b.s.size()));
        expect(eq(r.records.size(), 4UZ));
        if (r.out.size() != b.s.size() || r.records.empty()) {
            return;
        }
        for (std::size_t i = 0UZ; i < r.records.size(); ++i) {
            const auto& values = r.records[i].signal_values;
            expect(eq(metaNumber(r.records[i], "selected_branch"), 0.)) << std::format("record {} selects branch 0", i);
            expect(eq(values.at(4UZ), values.at(2UZ))) << std::format("record {}: the combined ratio is the selected branch's", i);
            expect(eq(values.at(5UZ), 0.f)) << std::format("record {}: selection gains nothing over the better branch", i);
        }
        const double signalPower = project(r.out, b.s, kWindow, b.s.size()).signalPower;
        const double measuredDb  = measuredSnrDb(r.out, b.s, kWindow, b.s.size());
        std::println("selection, unit_signal: output signal power {:.4f}, measured {:.3f} dB, reported {:.3f} dB", signalPower, measuredDb, r.combinedDb);
        expect(std::abs(signalPower - 1.) < 0.05) << "unit_signal sets the selected branch's signal power to 1";
        expect(std::abs(measuredDb - r.combinedDb) < 0.3) << "the reported combined ratio is the output's";
    };

    // Branch 1 carries the signal at the opposite phase, and the branches cross after window 4.
    "under selection the weights switch at once, and the signal keeps its power across a switch"_test = [] {
        constexpr std::size_t kWindow  = 2048UZ;
        constexpr std::size_t kWindows = 12UZ;
        Rng                   rng(0x5555ULL);
        std::vector<CF>       r0(kWindow * kWindows);
        std::vector<CF>       r1(r0.size());
        std::vector<CF>       s(r0.size());
        for (std::size_t w = 0UZ; w < kWindows; ++w) {
            const CF h0 = w < 4UZ ? CF{1.f, 0.f} : CF{0.9f, 0.f};
            const CF h1 = w < 4UZ ? CF{-0.9f, 0.f} : CF{-1.f, 0.f};
            for (std::size_t k = 0UZ; k < kWindow; ++k) {
                const CF x          = rng.complexSample();
                s[w * kWindow + k]  = x;
                r0[w * kWindow + k] = h0 * x + 0.1f * rng.complexSample();
                r1[w * kWindow + k] = h1 * x + 0.1f * rng.complexSample();
            }
        }

        const Run r = run({{"window", gr::Size_t{kWindow}}, {"mode", std::string("selection")}, {"weight_smoothing", 0.5}}, r0, r1, 4096UZ);
        expect(eq(r.nBranchSwitches, std::uint64_t{1ULL})) << "the branches cross once";
        expect(eq(r.out.size(), s.size()));
        if (r.out.size() != s.size()) {
            return;
        }
        double weakestDb = 0.;
        for (std::size_t w = 1UZ; w < kWindows; ++w) {
            weakestDb = std::min(weakestDb, 10. * std::log10(project(r.out, s, w * kWindow, (w + 1UZ) * kWindow).signalPower));
        }
        std::println("selection with weight_smoothing 0.5: weakest window signal power {:.2f} dB", weakestDb);
        expect(weakestDb > -3.) << "no window loses half the signal power across the switch";
    };

    // Branch 0 at -30 dB and branch 1 at 20 dB. The weight on branch 0 carries mostly estimation noise.
    "the output phase holds still between windows while branch 0 fades"_test = [] {
        constexpr std::size_t kWindow  = 4096UZ;
        constexpr std::size_t kWindows = 16UZ;
        constexpr double      kNoise   = 0.01;
        Rng                   rng(0x9999ULL);
        const float           a0 = static_cast<float>(std::sqrt(kNoise * 1e-3));
        const Branches        b  = twoBranch(rng, kWindow * kWindows, CF{a0, 0.f}, CF(static_cast<float>(std::cos(0.6)), static_cast<float>(std::sin(0.6))), 1., kNoise);

        const Run r = run({{"window", gr::Size_t{kWindow}}}, b.r0, b.r1, 4096UZ);
        expect(eq(r.out.size(), b.s.size()));
        if (r.out.size() != b.s.size()) {
            return;
        }
        double worst    = 0.;
        double previous = std::arg(project(r.out, b.s, kWindow, 2UZ * kWindow).gain);
        for (std::size_t w = 2UZ; w < kWindows; ++w) {
            const double phase = std::arg(project(r.out, b.s, w * kWindow, (w + 1UZ) * kWindow).gain);
            worst              = std::max(worst, std::abs(std::remainder(phase - previous, 2. * std::numbers::pi)));
            previous           = phase;
        }
        std::println("branch 0 at -30 dB: worst output phase step between windows {:.4f} rad", worst);
        expect(worst < 0.05) << "the output phase steps by less than 0.05 rad between windows";
    };

    // Branch 1 is the stronger branch over windows 0 to 5 and branch 0 after them, and the channels differ in phase by
    // 2.9 rad. selection moves from the pass-through to branch 1 after window 0 and to branch 0 after window 6.
    "under selection the output phase holds across a switch"_test = [] {
        constexpr std::size_t kWindow  = 2048UZ;
        constexpr std::size_t kWindows = 12UZ;
        Rng                   rng(0x6666ULL);
        const CF              phase0(static_cast<float>(std::cos(0.4)), static_cast<float>(std::sin(0.4)));
        const CF              phase1(static_cast<float>(std::cos(-2.5)), static_cast<float>(std::sin(-2.5)));
        std::vector<CF>       r0(kWindow * kWindows);
        std::vector<CF>       r1(r0.size());
        std::vector<CF>       s(r0.size());
        for (std::size_t w = 0UZ; w < kWindows; ++w) {
            const bool branch1Louder = w < 6UZ;
            const CF   h0            = (branch1Louder ? 0.9f : 1.f) * phase0;
            const CF   h1            = (branch1Louder ? 1.f : 0.9f) * phase1;
            for (std::size_t k = 0UZ; k < kWindow; ++k) {
                const CF x          = rng.complexSample();
                s[w * kWindow + k]  = x;
                r0[w * kWindow + k] = h0 * x + 0.1f * rng.complexSample();
                r1[w * kWindow + k] = h1 * x + 0.1f * rng.complexSample();
            }
        }

        const Run r = run({{"window", gr::Size_t{kWindow}}, {"mode", std::string("selection")}}, r0, r1, 4096UZ);
        expect(eq(r.out.size(), s.size()));
        expect(eq(r.records.size(), kWindows));
        if (r.out.size() != s.size() || r.records.size() != kWindows) {
            return;
        }
        expect(eq(metaNumber(r.records.front(), "selected_branch"), 1.)) << "window 0 selects branch 1";
        expect(eq(metaNumber(r.records.back(), "selected_branch"), 0.)) << "the last window selects branch 0";
        expect(eq(r.nBranchSwitches, std::uint64_t{1ULL})) << "the branches cross once";

        double worst    = 0.;
        double previous = std::arg(project(r.out, s, 0UZ, kWindow).gain);
        for (std::size_t w = 1UZ; w < kWindows; ++w) {
            const double phase = std::arg(project(r.out, s, w * kWindow, (w + 1UZ) * kWindow).gain);
            worst              = std::max(worst, std::abs(std::remainder(phase - previous, 2. * std::numbers::pi)));
            previous           = phase;
        }
        std::println("selection across two switches: worst output phase step between windows {:.4f} rad", worst);
        expect(worst < 0.05) << "the output phase steps by less than 0.05 rad between windows, at each switch as well";
    };

    "one non-finite sample spoils its own window alone, and that window's record says so"_test = [] {
        constexpr std::size_t kWindow  = 1024UZ;
        constexpr std::size_t kWindows = 8UZ;
        constexpr std::size_t kAt      = 1500UZ; // in window 1
        Rng                   rng(0x7777ULL);
        const Branches        b = twoBranch(rng, kWindow * kWindows, CF{1.f, 0.f}, CF{0.3f, 0.f}, 1., 0.01);

        for (const auto& [bad, smoothing, what] : {std::tuple{CF{std::numeric_limits<float>::quiet_NaN(), 0.f}, 0.0, "a NaN"}, std::tuple{CF{std::numeric_limits<float>::infinity(), 0.f}, 0.5, "an infinity with weight_smoothing 0.5"}}) {
            std::vector<CF> r0 = b.r0;
            r0[kAt]            = bad;
            const Run r        = run({{"window", gr::Size_t{kWindow}}, {"weight_smoothing", smoothing}}, r0, b.r1, 4096UZ);
            expect(eq(r.out.size(), b.s.size())) << what;
            expect(eq(r.records.size(), kWindows)) << what;
            if (r.out.size() != b.s.size() || r.records.size() != kWindows) {
                continue;
            }

            const auto nonFinite = std::ranges::count_if(r.out, [](CF value) { return !std::isfinite(value.real()) || !std::isfinite(value.imag()); });
            expect(eq(nonFinite, std::ptrdiff_t{1})) << what << "leaves one output sample non-finite";
            expect(!std::isfinite(r.out[kAt].real()) || !std::isfinite(r.out[kAt].imag())) << what << "spoils its own output sample";
            for (std::size_t i = 0UZ; i < kWindows; ++i) {
                const bool spoiled = i == kAt / kWindow;
                expect(eq(metaNumber(r.records[i], "non_finite"), spoiled ? 1. : 0.)) << what << std::format("record {} states its non-finite count", i);
                expect(eq(metaNumber(r.records[i], "valid"), spoiled ? 0. : 1.)) << what << std::format("record {}", i);
            }
            const double next  = measuredSnrDb(r.out, b.s, 2UZ * kWindow, 3UZ * kWindow);
            const double later = measuredSnrDb(r.out, b.s, 3UZ * kWindow, 4UZ * kWindow);
            std::println("{}: output ratio {:.2f} dB in the next window, {:.2f} dB in the one after", what, next, later);
            expect(std::abs(next - later) < 1.) << what << "leaves the next window's weights intact";
            expect(std::isfinite(std::abs(r.weight0)) && std::isfinite(std::abs(r.weight1))) << what << "leaves the weights finite";
        }
    };

    "the end-of-stream record states its sample count, and a tail under 64 samples is not valid"_test = [] {
        constexpr std::size_t kWindow = 1024UZ;
        for (const std::size_t tail : {1UZ, 40UZ, 500UZ}) {
            Rng            rng(0x8888ULL);
            const Branches b = twoBranch(rng, 2UZ * kWindow + tail, CF{1.f, 0.f}, CF{0.5f, 0.f}, 1., 0.1);
            const Run      r = run({{"window", gr::Size_t{kWindow}}}, b.r0, b.r1, 4096UZ);
            expect(eq(r.out.size(), b.s.size())) << std::format("tail {}: every sample is combined", tail);
            expect(eq(r.records.size(), 3UZ)) << std::format("tail {}: two full windows and the flush", tail);
            if (r.records.size() != 3UZ) {
                continue;
            }
            std::println("tail {}: last record n_samples {}, valid {}", tail, metaNumber(r.records.back(), "n_samples"), metaNumber(r.records.back(), "valid"));
            expect(eq(metaNumber(r.records[0UZ], "n_samples"), static_cast<double>(kWindow)));
            expect(eq(metaNumber(r.records[1UZ], "n_samples"), static_cast<double>(kWindow)));
            expect(eq(metaNumber(r.records.back(), "n_samples"), static_cast<double>(tail))) << "the flushed record states its sample count";
            expect(eq(metaNumber(r.records.back(), "sample_start"), static_cast<double>(2UZ * kWindow)));
            expect(eq(metaNumber(r.records.back(), "valid"), tail >= 64UZ ? 1. : 0.)) << std::format("tail {}", tail);
        }
    };

    "a call combines every sample it is offered"_test = [] {
        Rng            rng(0x0f0f0f0f0f0f0f0fULL);
        const Branches b = twoBranch(rng, 777UZ, CF{1.f, 0.f}, CF{0.5f, 0.f}, 1., 0.1);

        PolarizationCombiner block({{"window", gr::Size_t{1024U}}});
        startBlock(block);
        Driven driven;
        drive(block, b.r0, b.r1, b.r0.size(), b.r0.size(), driven);
        expect(eq(driven.calls, 1UZ) && eq(driven.at, b.r0.size())) << "one call takes all 777 samples";
        expect(std::ranges::equal(driven.out, b.r0)) << "and passes branch 0 before the first window closes";

        finish(block, driven);
        expect(eq(driven.records.size(), 1UZ)) << "the end of the stream flushes the partial window";
        if (!driven.records.empty()) {
            expect(eq(metaNumber(driven.records[0UZ], "n_samples"), 777.));
        }
    };

    // A reference block and a candidate take the same stream. The candidate refuses an application after 1500 samples,
    // in the middle of window 1, and every output, record and counter of the two stays equal.
    "a refused run-time setting changes nothing the block computes"_test = [] {
        constexpr std::size_t kWindow = 1024UZ;
        constexpr std::size_t kAt     = 1500UZ;
        Rng                   rng(0xbbbbULL);
        const Branches        b        = twoBranch(rng, 4UZ * kWindow + 300UZ, CF{1.f, 0.f}, 0.5f * CF(static_cast<float>(std::cos(1.1)), static_cast<float>(std::sin(1.1))), 1., 0.1);
        const auto            settings = gr::property_map{{"window", gr::Size_t{kWindow}}, {"weight_smoothing", 0.5}, {"sample_rate", 48000.f}};

        struct Arm {
            gr::property_map refused;
            std::string_view what;
            bool             recordsConnected;
        };
        const std::vector<Arm> arms{
            {{{"window", gr::Size_t{0U}}}, "window 0", true},
            {{{"window", gr::Size_t{0U}}}, "window 0, measurements unconnected", false},
            {{{"noise_powers", std::vector<double>{1.}}}, "one noise power", true},
            {{{"weight_smoothing", 1.0}}, "weight_smoothing 1", true},
            {{{"sample_rate", 0.f}}, "sample_rate 0", true},
            {{{"mode", std::string("selection")}, {"emit_orthogonal", false}, {"emit_records", false}, {"noise_powers", std::vector<double>{1.}}}, "a batch with one noise power", true},
        };
        for (const Arm& arm : arms) {
            PolarizationCombiner reference(settings);
            PolarizationCombiner candidate(settings);
            startBlock(reference);
            startBlock(candidate);
            Driven expected;
            Driven driven;
            drive(reference, b.r0, b.r1, kAt, 700UZ, expected, arm.recordsConnected);
            drive(candidate, b.r0, b.r1, kAt, 700UZ, driven, arm.recordsConnected);

            expect(refuses(candidate, arm.refused)) << arm.what << "is refused";
            drive(reference, b.r0, b.r1, b.r0.size(), 700UZ, expected, arm.recordsConnected);
            drive(candidate, b.r0, b.r1, b.r0.size(), 700UZ, driven, arm.recordsConnected);
            finish(reference, expected);
            finish(candidate, driven);

            std::println("{}: {} of {} samples, {} records against {}", arm.what, driven.at, expected.at, driven.records.size(), expected.records.size());
            expect(eq(expected.at, b.r0.size())) << arm.what << "the reference takes every sample";
            expect(eq(driven.at, expected.at)) << arm.what << "the candidate takes every sample";
            expect(std::ranges::equal(driven.out, expected.out) && std::ranges::equal(driven.ortho, expected.ortho)) << arm.what << "leaves the outputs";
            expect(eq(driven.records.size(), expected.records.size())) << arm.what << "leaves the record count";
            for (std::size_t i = 0UZ; i < std::min(driven.records.size(), expected.records.size()); ++i) {
                expect(std::ranges::equal(driven.records[i].signal_values, expected.records[i].signal_values)) << arm.what << std::format("record {}", i);
                expect(eq(metaNumber(driven.records[i], "sample_rate"), 48000.) && eq(metaNumber(driven.records[i], "window"), static_cast<double>(kWindow)) && eq(metaString(driven.records[i], "mode"), std::string("mrc"))) << arm.what << std::format("record {} states the accepted settings", i);
            }
            expect(eq(candidate.nWindows(), reference.nWindows()) && eq(candidate.nWindowResets(), reference.nWindowResets())) << arm.what;
        }
    };

    // A candidate refuses a batch after 1500 samples, in the middle of window 1, and then takes a correction. A
    // reference takes only the values the correction changes. Both blocks change the window to 512 at sample 3000, and
    // every output, record and counter of the two stays equal.
    "a refused batch is written back and leaves its values out of the next accepted application"_test = [] {
        constexpr std::size_t kWindow = 1024UZ;
        constexpr std::size_t kAt     = 1500UZ;
        constexpr std::size_t kLater  = 3000UZ;
        Rng                   rng(0xddddULL);
        const Branches        b        = twoBranch(rng, 8UZ * kWindow, CF{1.f, 0.f}, 0.5f * CF(static_cast<float>(std::cos(1.1)), static_cast<float>(std::sin(1.1))), 1., 0.1);
        const auto            settings = gr::property_map{{"window", gr::Size_t{kWindow}}};
        const auto            smaller  = gr::property_map{{"window", gr::Size_t{512U}}};

        struct Arm {
            gr::property_map refused;
            gr::property_map correction;
            gr::property_map inForce; ///< what the correction changes in force
            std::string_view what;
            std::uint64_t    resets;      ///< the reference's discards
            double           windowAfter; ///< the window the reference closes after the correction
        };
        const std::vector<Arm> arms{
            {{{"window", gr::Size_t{256U}}, {"mode", std::string("diversity")}}, {{"mode", std::string("selection")}}, {{"mode", std::string("selection")}}, "the corrected setting alone", 2ULL, static_cast<double>(kWindow)},
            {{{"window", gr::Size_t{256U}}, {"mode", std::string("diversity")}}, {{"window", gr::Size_t{256U}}, {"mode", std::string("selection")}}, {{"window", gr::Size_t{256U}}, {"mode", std::string("selection")}}, "the whole batch again", 2ULL, 256.},
            {{{"weight_smoothing", 1.0}}, {{"mode", std::string("selection")}}, {{"mode", std::string("selection")}}, "another setting", 2ULL, static_cast<double>(kWindow)},
        };
        for (const Arm& arm : arms) {
            PolarizationCombiner reference(settings);
            PolarizationCombiner candidate(settings);
            startBlock(reference);
            startBlock(candidate);
            Driven expected;
            Driven driven;
            drive(reference, b.r0, b.r1, kAt, 700UZ, expected);
            drive(candidate, b.r0, b.r1, kAt, 700UZ, driven);

            expect(refuses(candidate, arm.refused)) << arm.what << "the batch is refused";
            expect(!refuses(candidate, arm.correction)) << arm.what << "the correction is accepted";
            expect(!refuses(reference, arm.inForce)) << arm.what;
            drive(reference, b.r0, b.r1, kLater, 700UZ, expected);
            drive(candidate, b.r0, b.r1, kLater, 700UZ, driven);
            expect(!refuses(reference, smaller)) << arm.what << "the window change is accepted";
            expect(!refuses(candidate, smaller)) << arm.what << "the window change is accepted";
            drive(reference, b.r0, b.r1, b.r0.size(), 700UZ, expected);
            drive(candidate, b.r0, b.r1, b.r0.size(), 700UZ, driven);
            finish(reference, expected);
            finish(candidate, driven);

            std::println("{}: {} of {} samples, {} records against {}, nWindowResets {} against {}", arm.what, driven.at, expected.at, driven.records.size(), expected.records.size(), candidate.nWindowResets(), reference.nWindowResets());
            expect(eq(expected.at, b.r0.size()) && eq(driven.at, expected.at)) << arm.what << "both blocks take every sample";
            expect(eq(reference.nWindowResets(), arm.resets)) << arm.what << "the reference discards where its accepted values change";
            expect(ge(expected.records.size(), 3UZ)) << arm.what;
            if (expected.records.size() >= 3UZ) {
                expect(eq(metaNumber(expected.records[1UZ], "window"), arm.windowAfter) && eq(metaNumber(expected.records.back(), "window"), 512.)) << arm.what << "the reference closes windows of the size in force after the correction, then of 512";
            }
            expect(std::ranges::equal(driven.out, expected.out) && std::ranges::equal(driven.ortho, expected.ortho)) << arm.what << "leaves the outputs";
            expect(eq(driven.records.size(), expected.records.size())) << arm.what << "leaves the record count";
            for (std::size_t i = 0UZ; i < std::min(driven.records.size(), expected.records.size()); ++i) {
                expect(std::ranges::equal(driven.records[i].signal_values, expected.records[i].signal_values)) << arm.what << std::format("record {}", i);
                expect(eq(metaNumber(driven.records[i], "sample_start"), metaNumber(expected.records[i], "sample_start")) && eq(metaNumber(driven.records[i], "n_samples"), metaNumber(expected.records[i], "n_samples")) && eq(metaNumber(driven.records[i], "window"), metaNumber(expected.records[i], "window")) && eq(metaString(driven.records[i], "mode"), metaString(expected.records[i], "mode"))) << arm.what << std::format("record {} states the same window and settings", i);
            }
            expect(eq(candidate.nWindows(), reference.nWindows()) && eq(candidate.nWindowResets(), reference.nWindowResets())) << arm.what;
        }
    };

    // Each reset key staged after 1500 samples, in the middle of window 1. The window starts again at sample 1500.
    "a change of window, noise_powers, mode or normalize discards the partial window and counts the discard"_test = [] {
        constexpr std::size_t kWindow = 1024UZ;
        Rng                   rng(0xccccULL);
        const Branches        b = twoBranch(rng, 6UZ * kWindow, CF{1.f, 0.f}, CF{0.5f, 0.f}, 1., 0.1);

        struct Arm {
            gr::property_map change;
            std::string_view what;
            std::size_t      at;
            std::uint64_t    resets;
            double           nextStart;
            double           nextSamples;
        };
        const std::vector<Arm> arms{
            {{{"window", gr::Size_t{2048U}}}, "window", 1500UZ, 1ULL, 1500., 2048.},
            {{{"noise_powers", std::vector<double>{0.1, 0.1}}}, "noise_powers", 1500UZ, 1ULL, 1500., 1024.},
            {{{"mode", std::string("selection")}}, "mode", 1500UZ, 1ULL, 1500., 1024.},
            {{{"normalize", std::string("unit_signal")}}, "normalize", 1500UZ, 1ULL, 1500., 1024.},
            {{{"weight_smoothing", 0.3}}, "weight_smoothing, not a reset key", 1500UZ, 0ULL, 1024., 1024.},
            {{{"window", gr::Size_t{1024U}}}, "the window in force", 1500UZ, 0ULL, 1024., 1024.},
            {{{"mode", std::string("selection")}}, "mode at a window boundary", 1024UZ, 0ULL, 1024., 1024.},
        };
        for (const Arm& arm : arms) {
            PolarizationCombiner block({{"window", gr::Size_t{kWindow}}});
            startBlock(block);
            Driven driven;
            drive(block, b.r0, b.r1, arm.at, 500UZ, driven);
            expect(!refuses(block, arm.change)) << arm.what << "is accepted";
            expect(eq(block.nWindowResets(), arm.resets)) << arm.what;
            drive(block, b.r0, b.r1, b.r0.size(), 500UZ, driven);
            finish(block, driven);

            expect(ge(driven.records.size(), 2UZ)) << arm.what;
            if (driven.records.size() < 2UZ) {
                continue;
            }
            std::println("{}: nWindowResets {}, record 1 starts at {} with {} samples", arm.what, block.nWindowResets(), metaNumber(driven.records[1UZ], "sample_start"), metaNumber(driven.records[1UZ], "n_samples"));
            expect(eq(metaNumber(driven.records[0UZ], "sample_start"), 0.) && eq(metaNumber(driven.records[0UZ], "n_samples"), 1024.)) << arm.what << "window 0 closes before the change";
            expect(eq(metaNumber(driven.records[1UZ], "sample_start"), arm.nextStart)) << arm.what << "sets where the next window starts";
            expect(eq(metaNumber(driven.records[1UZ], "n_samples"), arm.nextSamples)) << arm.what;
        }
    };

    // One full window of 128 closes, and a window change is accepted before the next sample. coverage() reads the
    // closed window against its own length. A partial window of 64 then closes at the end of the stream, against the
    // window in force.
    "coverage() reads each window against the length it closed with"_test = [] {
        constexpr gr::Size_t  kWindow  = 128U;
        constexpr std::size_t kPartial = 64UZ;
        Rng                   rng(0xeeeeULL);
        const Branches        b = twoBranch(rng, kWindow + kPartial, CF{1.f, 0.f}, CF{0.5f, 0.f}, 1., 0.1);

        struct Arm {
            gr::Size_t       next;
            std::string_view what;
        };
        const std::array<Arm, 2UZ> arms{{{256U, "a larger window"}, {kWindow, "the window in force"}}};
        for (const Arm& arm : arms) {
            PolarizationCombiner block({{"window", kWindow}});
            startBlock(block);
            Driven driven;
            drive(block, b.r0, b.r1, kWindow, kWindow, driven);
            expect(eq(driven.at, std::size_t{kWindow}) && eq(driven.records.size(), 1UZ)) << arm.what << "one full window closes";
            if (driven.records.size() != 1UZ) {
                continue;
            }
            const double closedShare = metaNumber(driven.records[0UZ], "n_samples") / metaNumber(driven.records[0UZ], "window");
            const double phase       = block.relativePhase();
            expect(eq(block.coverage(), closedShare)) << arm.what << "coverage() is the closed window's share of its window";

            expect(!refuses(block, {{"window", arm.next}})) << arm.what << "is accepted";
            expect(eq(block.coverage(), closedShare)) << arm.what << "leaves the coverage of the closed window";
            expect(eq(block.relativePhase(), phase)) << arm.what << "leaves the closed window's figures";

            drive(block, b.r0, b.r1, b.r0.size(), kPartial, driven);
            finish(block, driven);
            expect(eq(driven.records.size(), 2UZ)) << arm.what << "the partial window closes at the end of the stream";
            if (driven.records.size() != 2UZ) {
                continue;
            }
            const double partialShare = metaNumber(driven.records[1UZ], "n_samples") / metaNumber(driven.records[1UZ], "window");
            expect(eq(metaNumber(driven.records[1UZ], "window"), static_cast<double>(arm.next))) << arm.what << "the partial window closes against the window in force";
            expect(eq(block.coverage(), partialShare) && lt(block.coverage(), 1.)) << arm.what << "coverage() is the partial window's share of its window";
        }
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
