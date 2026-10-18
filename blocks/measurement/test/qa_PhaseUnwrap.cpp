#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <limits>
#include <map>
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
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/measurement/PhaseUnwrap.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_unwrap {

using gr::blocks::measurement::PhaseUnwrap;
using CF = std::complex<float>;

/// @brief A unit tone at a stated rate in cycles per sample and a stated starting phase, in bursts of a stated size.
/// It can publish one `n_dropped_samples` tag and one tag of settings at stated stream indices.
struct ToneSource : gr::Block<ToneSource> {
    gr::PortOut<CF> out;

    double           cyclesPerSample = 0.1;
    double           phaseOffset     = 0.;
    std::size_t      total           = 10000UZ;
    std::size_t      burst           = 4096UZ;
    bool             tagDrop         = false;
    std::size_t      dropAt          = 0UZ;
    gr::property_map settingsTag{}; ///< published at settingsAt when not empty
    std::size_t      settingsAt = 0UZ;
    std::size_t      at         = 0UZ;

    GR_MAKE_REFLECTABLE(ToneSource, out);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (at >= total) {
            outSpan.publish(0UZ);
            this->requestStop();
            return gr::work::Status::DONE;
        }
        const std::size_t take = std::min({burst, total - at, outSpan.size()});
        for (std::size_t k = 0UZ; k < take; ++k) {
            const double phase = phaseOffset + 2. * std::numbers::pi * cyclesPerSample * static_cast<double>(at + k);
            outSpan[k]         = CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
        }
        if (tagDrop && dropAt >= at && dropAt < at + take) {
            outSpan.publishTag(gr::property_map{{std::pmr::string(gr::tag::N_DROPPED_SAMPLES.shortKey()), gr::pmt::Value(gr::Size_t{100U})}}, dropAt - at);
        }
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

    std::vector<T>                            items{};
    std::map<std::uint64_t, gr::property_map> tags{};

    GR_MAKE_REFLECTABLE(Collector, in);

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const gr::Tag& tag : inSpan.rawTags) {
            tags.insert({static_cast<std::uint64_t>(tag.index), tag.map});
        }
        items.insert(items.end(), inSpan.begin(), inSpan.end());
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/// @brief The two output streams, their tags and the block's counters after one run.
struct Run {
    std::vector<std::int64_t> cycles;
    std::vector<float>        phase;

    std::map<std::uint64_t, gr::property_map> cycleTags;
    std::map<std::uint64_t, gr::property_map> phaseTags;

    std::uint64_t nSuspectSteps{};
    std::uint64_t nResets{};
    std::uint64_t nSaturations{};
    std::int64_t  lastSuspectIndex{-1};
    bool          completed{};       ///< `runAndWait()` returned no error
    double        maxStepFraction{}; ///< the `max_step_fraction` member after the run
};

struct Scene {
    double           cyclesPerSample = 0.1;
    double           phaseOffset     = 0.;
    std::size_t      total           = 10000UZ;
    std::size_t      burst           = 4096UZ;
    bool             tagDrop         = false;
    std::size_t      dropAt          = 0UZ;
    gr::property_map settingsTag{};
    std::size_t      settingsAt = 0UZ;
};

/// @brief Runs a tone through `PhaseUnwrap` into two collectors. The counters are read while the scheduler holds the
/// graph.
[[nodiscard]] Run runScene(gr::property_map settings, const Scene& scene) {
    gr::Graph graph;
    auto&     source     = graph.emplaceBlock<ToneSource>();
    auto&     block      = graph.emplaceBlock<PhaseUnwrap>(std::move(settings));
    auto&     cyclesSink = graph.emplaceBlock<Collector<std::int64_t>>();
    auto&     phaseSink  = graph.emplaceBlock<Collector<float>>();

    source.cyclesPerSample = scene.cyclesPerSample;
    source.phaseOffset     = scene.phaseOffset;
    source.total           = scene.total;
    source.burst           = scene.burst;
    source.tagDrop         = scene.tagDrop;
    source.dropAt          = scene.dropAt;
    source.settingsTag     = scene.settingsTag;
    source.settingsAt      = scene.settingsAt;

    if (!graph.connect<"out", "in">(source, block).has_value() || !graph.connect<"cycles", "in">(block, cyclesSink).has_value() || !graph.connect<"phase", "in">(block, phaseSink).has_value()) {
        return {};
    }
    gr::scheduler::Simple scheduler;
    if (!scheduler.exchange(std::move(graph)).has_value()) {
        return {};
    }
    const bool completed = scheduler.runAndWait().has_value(); // the source ends the stream

    Run result;
    result.completed        = completed;
    result.maxStepFraction  = block.max_step_fraction.value;
    result.cycles           = std::move(cyclesSink.items);
    result.phase            = std::move(phaseSink.items);
    result.cycleTags        = std::move(cyclesSink.tags);
    result.phaseTags        = std::move(phaseSink.tags);
    result.nSuspectSteps    = block.nSuspectSteps();
    result.nResets          = block.nResets();
    result.nSaturations     = block.nSaturations();
    result.lastSuspectIndex = block.lastSuspectIndex().has_value() ? static_cast<std::int64_t>(*block.lastSuspectIndex()) : -1LL;
    return result;
}

[[nodiscard]] Run run(gr::property_map settings, double cyclesPerSample, std::size_t total, std::size_t burst, bool tagDrop = false, std::size_t dropAt = 0UZ) { return runScene(std::move(settings), Scene{.cyclesPerSample = cyclesPerSample, .phaseOffset = 0., .total = total, .burst = burst, .tagDrop = tagDrop, .dropAt = dropAt}); }

[[nodiscard]] std::string tagString(const std::map<std::uint64_t, gr::property_map>& tags, std::uint64_t index, std::string_view key) {
    const auto at = tags.find(index);
    if (at == tags.end()) {
        return "<no tag>";
    }
    const auto it = at->second.find(std::pmr::string(key));
    if (it == at->second.end()) {
        return "<no key>";
    }
    if (const auto* asString = it->second.get_if<std::pmr::string>()) {
        return std::string(std::string_view(*asString));
    }
    return "<not a string>";
}

/// @brief A unit tone at @p cyclesPerSample over @p n samples, from stream index @p first.
[[nodiscard]] std::vector<CF> toneSamples(double cyclesPerSample, std::size_t n, std::size_t first = 0UZ) {
    std::vector<CF> out(n);
    for (std::size_t k = 0UZ; k < n; ++k) {
        const double phase = 2. * std::numbers::pi * cyclesPerSample * static_cast<double>(first + k);
        out[k]             = CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
    }
    return out;
}

/// @brief Applies the settings @p block was built with and starts it.
void startBlock(PhaseUnwrap& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

/// @brief One processBulk call on test spans over @p in from stream index @p at, with @p tags on the input. Returns
/// the samples the call consumed.
std::size_t drive(PhaseUnwrap& block, std::span<const CF> in, std::size_t at, std::span<std::int64_t> cycles, std::span<float> phase, std::span<const gr::Tag> tags = {}) {
    namespace shim = gr::blocks::testing::span;
    shim::InputSpan<CF>            inSpan(in, at, tags);
    shim::OutputSpan<std::int64_t> cyclesSpan(cycles, at);
    shim::OutputSpan<float>        phaseSpan(phase, at);
    std::ignore = block.processBulk(inSpan, cyclesSpan, phaseSpan);
    return inSpan.consumed;
}

/// @brief Returns whether the block or the limits refuse @p settings, staged on a running block and applied.
[[nodiscard]] bool refuses(PhaseUnwrap& block, gr::property_map settings) {
    std::ignore = block.settings().setStaged(std::move(settings));
    try {
        return !block.settings().applyStagedParameters().failedParameters.empty();
    } catch (const gr::exception&) {
        return true;
    }
}

/// @brief Stages @p settings on a running block and applies them. Returns the settings the limits refused.
[[nodiscard]] gr::property_map refusedByLimits(PhaseUnwrap& block, gr::property_map settings) {
    std::ignore = block.settings().setStaged(std::move(settings));
    return block.settings().applyStagedParameters().failedParameters;
}

/// @brief Wraps to `[-pi, pi)`.
[[nodiscard]] double wrap(double radians) {
    double out = std::fmod(radians + std::numbers::pi, 2. * std::numbers::pi);
    if (out < 0.) {
        out += 2. * std::numbers::pi;
    }
    return out - std::numbers::pi;
}

} // namespace qa_unwrap

const boost::ut::suite<"PhaseUnwrap"> phaseUnwrapTests = [] {
    using namespace boost::ut;
    using namespace qa_unwrap;

    // 5 242 881 samples at 0.4 cycles per sample end exactly on turn 2 097 152, the scene of the kernel's test.
    "cycles and phase stay exact where a float sum is off by more than a cycle"_test = [] {
        constexpr double      kCyclesPerSample = 0.4;
        constexpr std::size_t kTotal           = 5'242'881UZ;

        const Run run_ = run(gr::property_map{}, kCyclesPerSample, kTotal, 4096UZ);
        expect(eq(run_.cycles.size(), kTotal));
        if (run_.cycles.empty()) {
            return;
        }

        const std::int64_t measuredCycles = run_.cycles.back();
        const float        measuredPhase  = run_.phase.back();
        std::println("after {} samples: cycles {} phase {:.8f}", kTotal, measuredCycles, measuredPhase);
        expect(eq(measuredCycles, std::int64_t{2'097'152LL})) << "the count is exact at 2^21 turns";

        const double trueRadians  = 2. * std::numbers::pi * kCyclesPerSample * static_cast<double>(kTotal - 1UZ);
        const double trueResidual = wrap(trueRadians);
        std::println("analytic residual {:.8f}, block residual {:.8f}", trueResidual, static_cast<double>(measuredPhase));
        expect(std::abs(static_cast<double>(measuredPhase) - trueResidual) < 4. * 2.38e-7) << "phase lies within four times a float's spacing at pi of the analytic residual";

        const double unwrappedRadians = 2. * std::numbers::pi * static_cast<double>(measuredCycles) + static_cast<double>(measuredPhase);
        expect(std::abs(unwrappedRadians - trueRadians) < 1e-3) << "cycles and phase together give the true unwrapped phase";

        // A float sum of the same steps. Its error is asserted; the double sum's is printed.
        float       floatAccumulator  = 0.f;
        double      doubleAccumulator = 0.;
        const float step              = static_cast<float>(2. * std::numbers::pi * kCyclesPerSample);
        for (std::size_t k = 1UZ; k < kTotal; ++k) {
            floatAccumulator += step;
            doubleAccumulator += 2. * std::numbers::pi * kCyclesPerSample;
        }
        const double floatError  = std::abs(static_cast<double>(floatAccumulator) - trueRadians);
        const double doubleError = std::abs(doubleAccumulator - trueRadians);
        const float  spacing     = std::nextafter(floatAccumulator, floatAccumulator * 2.f) - floatAccumulator;
        std::println("float sum off by {:.3f} rad ({:.1f} cycles), final spacing {:.3f} rad; double sum off by {:.3e} rad", floatError, floatError / (2. * std::numbers::pi), spacing, doubleError);
        expect(floatError > 2. * std::numbers::pi) << "a float sum of the same steps is off by more than one cycle";
    };

    // Below half the sample rate the unwrap is exact. Above it the count follows the alias. The suspect count starts at
    // 0.47 cycles per sample and stays zero at 0.6.
    "unwraps exactly below half the sample rate, follows the alias above it, and flags steps near pi"_test = [] {
        constexpr std::size_t kTotal = 1'000'000UZ;

        {
            const Run    run_        = run(gr::property_map{}, 0.4, kTotal, 8192UZ);
            const double trueRadians = 2. * std::numbers::pi * 0.4 * static_cast<double>(kTotal - 1UZ);
            const double unwrapped   = 2. * std::numbers::pi * static_cast<double>(run_.cycles.back()) + static_cast<double>(run_.phase.back());
            std::println("0.4 fs unwrapped {:.3f}, analytic {:.3f}", unwrapped, trueRadians);
            expect(std::abs(unwrapped - trueRadians) < 1e-2) << "0.4 cycles per sample unwraps exactly over 10^6 samples";
        }

        {
            // 0.6 cycles per sample steps by 1.2*pi, which wraps to -0.8*pi, the step of -0.4 cycles per sample.
            const Run    run_         = run(gr::property_map{}, 0.6, kTotal, 8192UZ);
            const double aliasRadians = 2. * std::numbers::pi * (-0.4) * static_cast<double>(kTotal - 1UZ);
            const double unwrapped    = 2. * std::numbers::pi * static_cast<double>(run_.cycles.back()) + static_cast<double>(run_.phase.back());
            std::println("0.6 fs unwrapped {:.3f}, alias at -0.4 fs {:.3f}", unwrapped, aliasRadians);
            expect(std::abs(unwrapped - aliasRadians) < 1e-2) << "0.6 cycles per sample reads the alias at -0.4 cycles per sample";
        }

        {
            // The default max_step_fraction 0.9 puts the threshold at 0.9*pi. The wrapped step at 0.6 fs is 0.8*pi.
            const Run run_ = run(gr::property_map{}, 0.6, 2000UZ, 4096UZ);
            std::println("nSuspectSteps() at 0.6 fs = {}", run_.nSuspectSteps);
            expect(eq(run_.nSuspectSteps, std::uint64_t{0ULL})) << "no step is flagged at 0.6 fs, where the wrapped step is 0.8*pi";
        }
        {
            // At 0.47 cycles per sample the step of 0.94*pi exceeds the default 0.9*pi.
            const Run run_ = run(gr::property_map{}, 0.47, 2000UZ, 4096UZ);
            std::println("nSuspectSteps() at 0.47 fs = {}", run_.nSuspectSteps);
            expect(run_.nSuspectSteps > 0ULL) << "0.47 cycles per sample crosses the default threshold";
        }
    };

    // The same stream, with only reset_on_discontinuity differing.
    "an n_dropped_samples tag resets the count when reset_on_discontinuity is set"_test = [] {
        constexpr std::size_t kTotal  = 10000UZ;
        constexpr std::size_t kDropAt = 4000UZ;

        const Run withReset    = run(gr::property_map{{"reset_on_discontinuity", true}}, 0.1, kTotal, 777UZ, true, kDropAt);
        const Run withoutReset = run(gr::property_map{{"reset_on_discontinuity", false}}, 0.1, kTotal, 777UZ, true, kDropAt);

        expect(eq(withReset.cycles.size(), kTotal));
        expect(eq(withoutReset.cycles.size(), kTotal));
        if (withReset.cycles.size() != kTotal || withoutReset.cycles.size() != kTotal) {
            return;
        }

        for (std::size_t k = 0UZ; k < kDropAt; ++k) {
            expect(eq(withReset.cycles[k], withoutReset.cycles[k])) << std::format("sample {}: the two runs agree before the tag", k);
        }
        std::println("discontinuity: cycles before the tag {}, at the tag with reset {}, without reset {}", withReset.cycles[kDropAt - 1UZ], withReset.cycles[kDropAt], withoutReset.cycles[kDropAt]);
        expect(eq(withReset.cycles[kDropAt], std::int64_t{0LL})) << "with reset_on_discontinuity, the count is zero at the tagged sample";
        expect(withoutReset.cycles[kDropAt] > std::int64_t{100LL}) << "without reset_on_discontinuity, the count at the tagged sample is about 400";
        expect(std::abs(withoutReset.cycles[kDropAt] - withReset.cycles[kDropAt - 1UZ]) <= std::int64_t{1LL}) << "without the flag, the count continues across the tag";

        expect(eq(withReset.nResets, std::uint64_t{1ULL})) << "reset_on_discontinuity: one tag adds one to nResets()";
        expect(eq(withoutReset.nResets, std::uint64_t{0ULL})) << "with the flag false, nResets() stays zero";
    };

    "the count reads 0 before any sample"_test = [] {
        PhaseUnwrap block;
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
        expect(eq(block._unwrapper.cycles(), std::int64_t{0LL}));
    };

    // `origin = zero` subtracts the first sample's phase. Neither origin changes the differences between samples.
    "origin 'zero' starts at exactly zero, and both origins give the same differences"_test = [] {
        constexpr std::size_t kTotal  = 4000UZ;
        constexpr double      kRate   = 0.137;
        constexpr double      kOffset = 0.9; // the tone's first phase is not zero

        const Scene scene{.cyclesPerSample = kRate, .phaseOffset = kOffset, .total = kTotal, .burst = 777UZ, .tagDrop = false, .dropAt = 0UZ};

        const Run first = runScene({{"origin", std::string("first_sample")}}, scene);
        const Run zero  = runScene({{"origin", std::string("zero")}}, scene);
        expect(eq(first.cycles.size(), kTotal));
        expect(eq(zero.cycles.size(), kTotal));
        if (first.cycles.size() != kTotal || zero.cycles.size() != kTotal) {
            return;
        }

        std::println("origin: first_sample phase[0] = {:.6f}, zero phase[0] = {:.6f}", first.phase.front(), zero.phase.front());
        expect(std::abs(first.phase.front()) > 1e-9f) << "first_sample reports the tone's nonzero first phase";
        expect(eq(zero.phase.front(), 0.f)) << "origin = zero starts the output at exactly zero";
        expect(eq(first.cycles.front(), std::int64_t{0LL}));
        expect(eq(zero.cycles.front(), std::int64_t{0LL}));

        double worst = 0.;
        for (std::size_t k = 1UZ; k < kTotal; ++k) {
            const double stepFirst = 2. * std::numbers::pi * static_cast<double>(first.cycles[k] - first.cycles[k - 1UZ]) + static_cast<double>(first.phase[k] - first.phase[k - 1UZ]);
            const double stepZero  = 2. * std::numbers::pi * static_cast<double>(zero.cycles[k] - zero.cycles[k - 1UZ]) + static_cast<double>(zero.phase[k] - zero.phase[k - 1UZ]);
            worst                  = std::max(worst, std::abs(stepFirst - stepZero));
        }
        std::println("origin: worst difference between the per-sample steps {:.3e} rad", worst);
        expect(worst < 1e-5) << "the two origins give the same differences";
    };

    // The refusals, and max_step_fraction on the same stream at two values: the suspect count differs, and the
    // outputs are the same.
    "max_step_fraction is refused outside (0, 1] by its limits and moves the suspect count and no output"_test = [] {
        for (const double outside : {0.0, 1.5, std::numeric_limits<double>::quiet_NaN()}) {
            PhaseUnwrap block;
            startBlock(block);
            expect(throws([&] { std::ignore = block.settings().set({{"max_step_fraction", outside}}); })) << "settings().set() refuses " << outside << " at the call";
            expect(refusedByLimits(block, {{"max_step_fraction", outside}}).contains("max_step_fraction")) << "the limits refuse a staged " << outside;
            expect(eq(block.max_step_fraction.value, 0.9)) << "and the member keeps the accepted value";
        }
        PhaseUnwrap block;
        startBlock(block);
        expect(refusedByLimits(block, {{"max_step_fraction", 1.0}}).empty()) << "one is accepted";
        std::ignore = block.settings().setStaged({{"origin", std::string("elsewhere")}});
        expect(throws([&] { std::ignore = block.settings().applyStagedParameters(); })) << "the block throws for an origin outside 'first_sample' and 'zero'";

        // 0.42 cycles per sample steps by 0.84*pi, below the default 0.9 and above 0.8.
        constexpr std::size_t kTotal = 2000UZ;
        const Run             loose  = run({{"max_step_fraction", 0.9}}, 0.42, kTotal, 4096UZ);
        const Run             tight  = run({{"max_step_fraction", 0.8}}, 0.42, kTotal, 4096UZ);
        std::println("max_step_fraction: nSuspectSteps() at 0.9 = {}, at 0.8 = {} (the step is 0.84*pi)", loose.nSuspectSteps, tight.nSuspectSteps);
        expect(eq(loose.nSuspectSteps, std::uint64_t{0ULL})) << "0.84*pi stays below a 0.9*pi threshold";
        expect(eq(tight.nSuspectSteps, std::uint64_t{kTotal - 1UZ})) << "0.84*pi crosses a 0.8*pi threshold at every step after the first";
        expect(eq(tight.lastSuspectIndex, static_cast<std::int64_t>(kTotal - 1UZ))) << "the last flagged step is the last sample";
        expect(eq(loose.lastSuspectIndex, std::int64_t{-1LL})) << "with no flagged step there is no index";
        expect(std::ranges::equal(loose.cycles, tight.cycles)) << "the threshold leaves the count alone";
        expect(std::ranges::equal(loose.phase, tight.phase));
        expect(eq(loose.nSaturations, std::uint64_t{0ULL})) << "no run here reaches 2^63 turns";
    };

    // A change of origin sets the count to zero and sets the origin again. The test drives the block directly.
    "an origin change resets the count and adds one to nResets()"_test = [] {
        PhaseUnwrap block;
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();

        constexpr std::size_t     kSamples = 5000UZ; // about 500 turns at 0.1 cycles per sample
        std::vector<CF>           in(kSamples);
        std::vector<std::int64_t> cycles(kSamples);
        std::vector<float>        phase(kSamples);
        for (std::size_t k = 0UZ; k < kSamples; ++k) {
            const double p = 0.9 + 2. * std::numbers::pi * 0.1 * static_cast<double>(k);
            in[k]          = CF(static_cast<float>(std::cos(p)), static_cast<float>(std::sin(p)));
        }
        expect(block._unwrapper.process(in, cycles, phase).has_value());
        std::println("origin change: cycles before the change {}, nResets() {}", block._unwrapper.cycles(), block.nResets());
        expect(block._unwrapper.cycles() > std::int64_t{400LL}) << "about 500 turns have run up";
        expect(eq(block.nResets(), std::uint64_t{0ULL}));

        expect(block.settings().setStaged(gr::property_map{{"origin", std::string("zero")}}).empty()) << "the new origin is accepted";
        std::ignore = block.settings().applyStagedParameters();
        expect(eq(block.nResets(), std::uint64_t{1ULL})) << "an origin change is one reset";
        expect(eq(block._unwrapper.cycles(), std::int64_t{0LL})) << "the count returns to zero";
        expect(!block._unwrapper.started()) << "the next sample sets the origin";

        expect(block.settings().setStaged(gr::property_map{{"origin", std::string("zero")}}).empty());
        std::ignore = block.settings().applyStagedParameters();
        expect(eq(block.nResets(), std::uint64_t{1ULL})) << "staging the same origin again is not a change";
    };

    // Two blocks take the same stream. One refuses an application halfway, and every output and counter of the two
    // stays equal. 0.42 cycles per sample steps by 0.84*pi, below the default 0.9*pi.
    "a refused max_step_fraction or batch changes nothing the block computes"_test = [] {
        constexpr std::size_t kHalf  = 1000UZ;
        const auto            stream = toneSamples(0.42, 2UZ * kHalf);
        // An n_dropped_samples tag in the second half pins reset_on_discontinuity, which the refused batch carries.
        const std::vector<gr::Tag> drop{gr::Tag{1500UZ, gr::property_map{{std::pmr::string(gr::tag::N_DROPPED_SAMPLES.shortKey()), gr::pmt::Value(gr::Size_t{10U})}}}};

        for (const auto& [refused, what] : {std::pair{gr::property_map{{"max_step_fraction", 0.0}}, "max_step_fraction 0"}, std::pair{gr::property_map{{"max_step_fraction", 0.5}, {"reset_on_discontinuity", false}, {"origin", std::string("elsewhere")}}, "a batch with a refused origin"}}) {
            PhaseUnwrap reference;
            PhaseUnwrap candidate;
            startBlock(reference);
            startBlock(candidate);

            std::vector<std::int64_t> cyclesRef(stream.size());
            std::vector<float>        phaseRef(stream.size());
            std::vector<std::int64_t> cycles(stream.size());
            std::vector<float>        phase(stream.size());
            const std::span<const CF> first  = std::span<const CF>(stream).first(kHalf);
            const std::span<const CF> second = std::span<const CF>(stream).last(kHalf);
            expect(eq(drive(reference, first, 0UZ, std::span(cyclesRef).first(kHalf), std::span(phaseRef).first(kHalf)), kHalf));
            expect(eq(drive(candidate, first, 0UZ, std::span(cycles).first(kHalf), std::span(phase).first(kHalf)), kHalf));

            expect(refuses(candidate, refused)) << what << "is refused";
            expect(eq(drive(reference, second, kHalf, std::span(cyclesRef).last(kHalf), std::span(phaseRef).last(kHalf), drop), kHalf));
            expect(eq(drive(candidate, second, kHalf, std::span(cycles).last(kHalf), std::span(phase).last(kHalf), drop), kHalf));

            std::println("{}: nSuspectSteps {} against {}, nResets {} against {}", what, candidate.nSuspectSteps(), reference.nSuspectSteps(), candidate.nResets(), reference.nResets());
            expect(eq(reference.nSuspectSteps(), std::uint64_t{0ULL})) << "the reference flags no step";
            expect(eq(reference.nResets(), std::uint64_t{1ULL})) << "the reference resets at the tag";
            expect(eq(candidate.nSuspectSteps(), reference.nSuspectSteps())) << what << "leaves the threshold";
            expect(eq(candidate.nResets(), reference.nResets())) << what << "leaves reset_on_discontinuity";
            expect(candidate.lastSuspectIndex() == reference.lastSuspectIndex()) << what;
            expect(std::ranges::equal(cycles, cyclesRef) && std::ranges::equal(phase, phaseRef)) << what << "leaves the outputs";
        }

        // An accepted value applies at once.
        PhaseUnwrap block;
        startBlock(block);
        std::vector<std::int64_t> cycles(stream.size());
        std::vector<float>        phase(stream.size());
        expect(refusedByLimits(block, {{"max_step_fraction", 0.0}}).contains("max_step_fraction"));
        expect(!refuses(block, {{"max_step_fraction", 0.8}})) << "a value in (0, 1] is accepted";
        expect(eq(drive(block, stream, 0UZ, cycles, phase), stream.size()));
        expect(eq(block.nSuspectSteps(), std::uint64_t{stream.size() - 1UZ})) << "0.84*pi crosses the accepted 0.8*pi at every step";
    };

    "a max_step_fraction outside its limits on a tag is refused, and the run goes on at the accepted value"_test = [] {
        // 0.42 cycles per sample steps by 0.84*pi, below the default 0.9*pi. The valid arm shows that a
        // max_step_fraction tag reaches the block.
        constexpr std::size_t kTotal = 4000UZ;
        constexpr std::size_t kAt    = 1000UZ;
        const auto            tagged = [](double fraction) { return runScene({}, Scene{.cyclesPerSample = 0.42, .total = kTotal, .burst = 500UZ, .settingsTag = {{"max_step_fraction", fraction}}, .settingsAt = kAt}); };

        const Run valid = tagged(0.8);
        expect(valid.completed);
        expect(eq(valid.nSuspectSteps, std::uint64_t{kTotal - kAt})) << "a valid tag flags every step from its sample on";

        for (const double outside : {1.5, std::numeric_limits<double>::quiet_NaN()}) {
            const Run refused = tagged(outside);
            expect(refused.completed) << "max_step_fraction " << outside << ": the run completes";
            expect(eq(refused.cycles.size(), kTotal) && eq(refused.phase.size(), kTotal)) << "max_step_fraction " << outside << ": every sample is unwrapped";
            expect(eq(refused.nSuspectSteps, std::uint64_t{0ULL})) << "max_step_fraction " << outside << ": the accepted 0.9*pi flags no step";
            expect(eq(refused.maxStepFraction, 0.9)) << "max_step_fraction " << outside << ": the member keeps the accepted value";
        }
    };

    // A candidate refuses a batch after 1000 samples and then takes a correction. A reference block takes the corrected
    // values alone. Both blocks then take origin 'first_sample' at 2000 and origin 'zero' at 2500, and every output and
    // counter of the two stays equal.
    "a refused batch leaves its values out of the next accepted application"_test = [] {
        constexpr std::size_t kRefusedAt     = 1000UZ;
        constexpr std::size_t kFirstSampleAt = 2000UZ;
        constexpr std::size_t kZeroAt        = 2500UZ;
        const auto            stream         = toneSamples(0.1, 3000UZ, 3UZ); // the first phase is 0.6*pi

        struct Arm {
            gr::property_map refused;
            gr::property_map correction;
            gr::property_map inForce; ///< what the correction changes in force
            std::string_view what;
            std::uint64_t    resets; ///< the reference's resets
        };
        const std::vector<Arm> arms{
            {{{"origin", std::string("elsewhere")}, {"max_step_fraction", 0.5}}, {{"origin", std::string("zero")}, {"max_step_fraction", 0.5}}, {{"origin", std::string("zero")}}, "the whole batch again", 3ULL},
            {{{"max_step_fraction", 0.0}}, {{"origin", std::string("zero")}}, {{"origin", std::string("zero")}}, "another setting", 3ULL},
        };
        for (const Arm& arm : arms) {
            PhaseUnwrap reference;
            PhaseUnwrap candidate;
            startBlock(reference);
            startBlock(candidate);
            std::vector<std::int64_t> cyclesRef(stream.size());
            std::vector<float>        phaseRef(stream.size());
            std::vector<std::int64_t> cycles(stream.size());
            std::vector<float>        phase(stream.size());
            const auto                driveBoth = [&](std::size_t from, std::size_t to) {
                const std::span<const CF> part = std::span<const CF>(stream).subspan(from, to - from);
                expect(eq(drive(reference, part, from, std::span(cyclesRef).subspan(from, to - from), std::span(phaseRef).subspan(from, to - from)), to - from));
                expect(eq(drive(candidate, part, from, std::span(cycles).subspan(from, to - from), std::span(phase).subspan(from, to - from)), to - from));
            };

            driveBoth(0UZ, kRefusedAt);
            expect(refuses(candidate, arm.refused)) << arm.what << "the batch is refused";
            expect(!refuses(candidate, arm.correction)) << arm.what << "the correction is accepted";
            expect(!refuses(reference, arm.inForce)) << arm.what;
            driveBoth(kRefusedAt, kFirstSampleAt);
            expect(!refuses(reference, {{"origin", std::string("first_sample")}})) << arm.what;
            expect(!refuses(candidate, {{"origin", std::string("first_sample")}})) << arm.what;
            driveBoth(kFirstSampleAt, kZeroAt);
            expect(!refuses(reference, {{"origin", std::string("zero")}})) << arm.what;
            expect(!refuses(candidate, {{"origin", std::string("zero")}})) << arm.what;
            driveBoth(kZeroAt, stream.size());

            std::println("{}: nResets {} against {}, cycles at 1999/2000/2999: {} {} {} against {} {} {}", arm.what, candidate.nResets(), reference.nResets(), cycles[1999UZ], cycles[2000UZ], cycles[2999UZ], cyclesRef[1999UZ], cyclesRef[2000UZ], cyclesRef[2999UZ]);
            expect(eq(reference.nResets(), arm.resets)) << arm.what << "the reference resets where its accepted origin changes";
            expect(eq(cyclesRef[kZeroAt], std::int64_t{0LL}) && eq(phaseRef[kZeroAt], 0.f)) << arm.what << "origin 'zero' starts the count again at exactly zero";
            expect(eq(candidate.nResets(), reference.nResets())) << arm.what;
            expect(eq(candidate.nSuspectSteps(), reference.nSuspectSteps())) << arm.what;
            expect(std::ranges::equal(cycles, cyclesRef) && std::ranges::equal(phase, phaseRef)) << arm.what << "leaves the outputs";
        }
    };

    // At 0.2 cycles per sample the phase wraps between samples 5m + 2 and 5m + 3.
    "a non-finite sample repeats the current outputs, is counted, and keeps the turn across it"_test = [] {
        constexpr std::size_t kSamples = 2001UZ;
        const auto            clean    = toneSamples(0.2, kSamples);
        auto                  spoiled  = clean;
        spoiled[1002UZ]                = CF{std::numeric_limits<float>::quiet_NaN(), 0.f};

        PhaseUnwrap reference;
        PhaseUnwrap block;
        startBlock(reference);
        startBlock(block);
        std::vector<std::int64_t> cyclesRef(kSamples);
        std::vector<float>        phaseRef(kSamples);
        std::vector<std::int64_t> cycles(kSamples);
        std::vector<float>        phase(kSamples);
        expect(eq(drive(reference, clean, 0UZ, cyclesRef, phaseRef), kSamples));
        expect(eq(drive(block, spoiled, 0UZ, cycles, phase), kSamples));

        std::println("a NaN at 1002: cycles {} against {}, nNonFiniteSamples {}", cycles.back(), cyclesRef.back(), block.nNonFiniteSamples());
        expect(eq(cycles.back(), cyclesRef.back())) << "the turn across the NaN sample is counted";
        expect(eq(block.nNonFiniteSamples(), std::uint64_t{1ULL}));
        expect(eq(reference.nNonFiniteSamples(), std::uint64_t{0ULL}));
        expect(eq(cycles[1002UZ], cycles[1001UZ]) && eq(phase[1002UZ], phase[1001UZ])) << "the NaN sample repeats the outputs before it";
        expect(eq(reference.nSuspectSteps(), std::uint64_t{0ULL})) << "no step of the clean stream crosses the threshold";
        expect(eq(block.nSuspectSteps(), std::uint64_t{1ULL})) << "the step across the gap is counted at 0.8*pi";
        expect(block.lastSuspectIndex() == std::optional<std::uint64_t>{1003ULL}) << "the counted step ends at the finite sample after the gap";
    };

    // At 0.04 cycles per sample a three-sample gap moves the phase by 0.32*pi, below the default 0.9*pi. The stream
    // starts at 0.96*pi. The gap at 599 and 600 crosses a call boundary. An n_dropped_samples tag at 800 follows the
    // gap at 798 and 799, and the gap at 0 has no finite sample before it.
    "each gap of non-finite samples counts once among the suspect steps"_test = [] {
        constexpr std::size_t kSamples = 1000UZ;
        constexpr std::size_t kSplit   = 600UZ;
        const auto            clean    = toneSamples(0.04, kSamples, 12UZ);
        auto                  spoiled  = clean;
        for (const std::size_t at : {0UZ, 100UZ, 400UZ, 401UZ, 402UZ, 599UZ, 600UZ, 798UZ, 799UZ}) {
            spoiled[at] = CF{std::numeric_limits<float>::quiet_NaN(), 0.f};
        }
        const std::vector<gr::Tag> drop{gr::Tag{800UZ, gr::property_map{{std::pmr::string(gr::tag::N_DROPPED_SAMPLES.shortKey()), gr::pmt::Value(gr::Size_t{10U})}}}};

        std::vector<std::int64_t> cycles(kSamples);
        std::vector<float>        phase(kSamples);
        const auto                twoCalls = [&cycles, &phase, &drop](PhaseUnwrap& block, const std::vector<CF>& stream) {
            startBlock(block);
            const std::size_t first = drive(block, std::span<const CF>(stream).first(kSplit), 0UZ, std::span(cycles).first(kSplit), std::span(phase).first(kSplit));
            return first + drive(block, std::span<const CF>(stream).subspan(kSplit), kSplit, std::span(cycles).subspan(kSplit), std::span(phase).subspan(kSplit), drop);
        };

        PhaseUnwrap reference;
        PhaseUnwrap block;
        expect(eq(twoCalls(reference, clean), kSamples));
        expect(eq(twoCalls(block, spoiled), kSamples));
        std::println("gaps at 0, 100, 400-402, 599-600 and 798-799: nSuspectSteps {}, last at {}, nNonFiniteSamples {}", block.nSuspectSteps(), block.lastSuspectIndex().value_or(0ULL), block.nNonFiniteSamples());
        expect(eq(reference.nSuspectSteps(), std::uint64_t{0ULL})) << "no step of the clean stream crosses the threshold";
        expect(eq(reference.nResets(), std::uint64_t{1ULL}) && eq(block.nResets(), std::uint64_t{1ULL})) << "both blocks reset at the tag";
        expect(eq(block.nNonFiniteSamples(), std::uint64_t{9ULL}));
        expect(eq(block.nSuspectSteps(), std::uint64_t{3ULL})) << "the gaps at 100, 400 to 402 and 599 to 600 count once each";
        expect(block.lastSuspectIndex() == std::optional<std::uint64_t>{601ULL}) << "the last counted step ends at sample 601";
    };

    "the two outputs carry their signal_unit, rad on phase and an empty unit on cycles"_test = [] {
        const Run r = run(gr::property_map{}, 0.1, 2000UZ, 4096UZ);
        std::println("signal_unit: cycles '{}', phase '{}'", tagString(r.cycleTags, 0ULL, "signal_unit"), tagString(r.phaseTags, 0ULL, "signal_unit"));
        expect(eq(tagString(r.cycleTags, 0ULL, "signal_unit"), std::string(""))) << "a whole-turn count has no unit";
        expect(eq(tagString(r.phaseTags, 0ULL, "signal_unit"), std::string("rad"))) << "the residual is in radians";
    };

    "any burst size gives bit-identical outputs and counters"_test = [] {
        constexpr std::size_t kTotal  = 3UZ * 4096UZ + 777UZ;
        constexpr std::size_t kDropAt = 4096UZ; // on a burst boundary at burst 4096

        const Run reference = run(gr::property_map{}, 0.37, kTotal, 4096UZ, true, kDropAt);
        expect(eq(reference.cycles.size(), kTotal));
        expect(eq(reference.nResets, std::uint64_t{1ULL}));
        expect(eq(reference.nSuspectSteps, std::uint64_t{0ULL})) << "0.37 cycles per sample steps by 0.74*pi, and the step across the reset is not tested";

        for (const std::size_t burst : {1UZ, 7UZ, 1000UZ, 12345UZ}) {
            const Run candidate = run(gr::property_map{}, 0.37, kTotal, burst, true, kDropAt);
            expect(eq(candidate.cycles.size(), reference.cycles.size())) << std::format("burst {}", burst);
            expect(std::ranges::equal(candidate.cycles, reference.cycles)) << std::format("burst {}: cycles stream differs", burst);
            expect(std::ranges::equal(candidate.phase, reference.phase)) << std::format("burst {}: phase stream differs", burst);
            expect(eq(candidate.nResets, reference.nResets)) << std::format("burst {}: nResets", burst);
            expect(eq(candidate.nSuspectSteps, reference.nSuspectSteps)) << std::format("burst {}: nSuspectSteps", burst);
            expect(eq(candidate.nSaturations, reference.nSaturations)) << std::format("burst {}: nSaturations", burst);
            expect(eq(candidate.lastSuspectIndex, reference.lastSuspectIndex)) << std::format("burst {}: lastSuspectIndex", burst);
        }
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
