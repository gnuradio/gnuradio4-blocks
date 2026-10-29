#include <boost/ut.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/YamlPmt.hpp>
#include <gnuradio-4.0/filter/DcBlocker.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include "StreamEndSink.hpp"

namespace {

using gr::blocks::filter::DcBlocker;

using CF = std::complex<float>;

constexpr gr::Size_t kLength = 32U;
constexpr gr::Size_t kReseed = 4096U;

template<typename T>
[[nodiscard]] DcBlocker<T> makeBlock(gr::property_map settings) {
    DcBlocker<T> block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

template<typename T>
[[nodiscard]] std::vector<T> run(DcBlocker<T>& block, std::span<const T> input, std::size_t chunkSize = 0UZ) {
    std::vector<T>    output(input.size());
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    for (std::size_t base = 0UZ; base < input.size(); base += stride) {
        const std::size_t count = std::min(stride, input.size() - base);
        std::ignore             = block.processBulk(input.subspan(base, count), std::span<T>(output).subspan(base, count));
    }
    return output;
}

/// @brief |H(f)| measured, by running a complex exponential through and reading the settled magnitude.
[[nodiscard]] double response(bool longForm, double normalizedFrequency) {
    DcBlocker<CF>   block = makeBlock<CF>({{"length", kLength}, {"long_form", longForm}});
    std::vector<CF> input(8192UZ);
    for (std::size_t i = 0UZ; i < input.size(); ++i) {
        const double phase = 2.0 * std::numbers::pi * normalizedFrequency * static_cast<double>(i);
        input[i]           = CF{static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))};
    }
    const std::vector<CF> output = run<CF>(block, std::span<const CF>(input));
    return static_cast<double>(std::abs(output.back()));
}

/// @brief The frequency at which the measured response crosses -3 dB, by bisection.
[[nodiscard]] double corner(bool longForm) {
    double low = 1e-4, high = 0.04;
    for (int step = 0; step < 24; ++step) {
        const double middle = 0.5 * (low + high);
        if (response(longForm, middle) < std::numbers::sqrt2 / 2.0) {
            low = middle;
        } else {
            high = middle;
        }
    }
    return 0.5 * (low + high);
}

/// The writable settings, round-tripped through YAML.
template<typename TBlock>
[[nodiscard]] gr::property_map savedSettings(TBlock& block) {
    gr::property_map stored;
    const auto&      writable = block.settings().writableMembers();
    for (const auto& [key, value] : block.settings().get()) {
        if (writable.contains(std::string(key))) {
            stored.insert_or_assign(key, value);
        }
    }
    return gr::pmt::yaml::deserialize(gr::pmt::yaml::serialize(stored)).value_or(gr::property_map{});
}

/// A block built from `settings`, after `init()`.
template<typename TBlock>
[[nodiscard]] std::unique_ptr<TBlock> loaded(const gr::property_map& settings) {
    auto block = std::make_unique<TBlock>(settings);
    block->init(std::make_shared<gr::Sequence>());
    return block;
}

[[nodiscard]] std::vector<float> pseudoRandom(std::size_t nSamples) {
    std::vector<float> input(nSamples);
    std::uint64_t      state = 0xB7E151628AED2A6BULL;
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        state    = state * 6364136223846793005ULL + 1442695040888963407ULL;
        input[i] = static_cast<float>(static_cast<double>(state >> 11U) / static_cast<double>(1ULL << 53U) - 0.5);
    }
    return input;
}

/// @brief The same pipeline in double, as the drift reference.
[[nodiscard]] std::vector<double> referenceBlocker(std::span<const float> input, std::size_t length, std::size_t stages) {
    const std::size_t   delay = stages / 2UZ * (length - 1UZ);
    std::vector<double> chain(input.begin(), input.end());
    for (std::size_t s = 0UZ; s < stages; ++s) {
        std::vector<double> next(chain.size());
        for (std::size_t n = 0UZ; n < chain.size(); ++n) {
            const std::size_t first = n + 1UZ >= length ? n + 1UZ - length : 0UZ;
            double            sum   = 0.0;
            for (std::size_t at = first; at <= n; ++at) {
                sum += chain[at];
            }
            next[n] = sum / static_cast<double>(length);
        }
        chain = std::move(next);
    }
    std::vector<double> output(input.size());
    for (std::size_t n = 0UZ; n < input.size(); ++n) {
        output[n] = (n >= delay ? static_cast<double>(input[n - delay]) : 0.0) - chain[n];
    }
    return output;
}

/// Runs `nSamples` through a `DcBlocker<float>` made with `settings`, with a tag carrying `trigger_name` and a key
/// outside `gr::tag::kDefaultTags` on each input of `at`.
[[nodiscard]] gr::blocks::filter::testing::EndRun runTagged(gr::property_map settings, gr::Size_t nSamples, std::initializer_list<std::size_t> at) {
    using gr::blocks::testing::ProcessFunction;
    using gr::blocks::testing::TagSource;

    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", nSamples}, {"mark_tag", false}});
    for (const std::size_t offset : at) {
        source._tags.emplace_back(offset, gr::property_map{{gr::property_map::key_type{"trigger_name"}, std::string("edge")}, {gr::property_map::key_type{"private_key"}, std::string("carried")}});
    }
    auto& block = graph.emplaceBlock<DcBlocker<float>>(std::move(settings));
    auto  sinks = gr::blocks::filter::testing::connectEndSinks<float>(graph, block);

    gr::blocks::filter::testing::EndRun run;
    if (!graph.connect<"out", "in">(source, block).has_value() || !sinks.has_value()) {
        return run;
    }
    gr::scheduler::Simple scheduler;
    run.ran        = scheduler.exchange(std::move(graph)).has_value() && scheduler.runAndWait().has_value();
    run.samples    = sinks->end._samples.size();
    run.endIndex   = sinks->end.endIndex();
    run.tags       = sinks->end._tags;
    run.sampleTags = sinks->samples._tags;
    run.ran        = run.ran && sinks->samples._samples.size() == run.samples;
    return run;
}

/// The value `settings().get()` returns for `key` as a `gr::Size_t`, or the largest value when it holds none.
[[nodiscard]] gr::Size_t reported(DcBlocker<float>& block, const char* key) {
    const gr::property_map active = block.settings().get();
    const auto             found  = active.find(gr::property_map::key_type{key});
    if (found == active.end() || found->second.get_if<gr::Size_t>() == nullptr) {
        return std::numeric_limits<gr::Size_t>::max();
    }
    return *found->second.get_if<gr::Size_t>();
}

/// Stages and applies `settings`. True when every key is refused.
[[nodiscard]] bool refused(DcBlocker<float>& block, gr::property_map settings) {
    const std::size_t keys = settings.size();
    std::ignore            = block.settings().setStaged(std::move(settings));
    return block.settings().applyStagedParameters().failedParameters.size() == keys;
}

/// What a scheduled run of a `DcBlocker<float>` delivered, and the `reseed_interval` it held at the end.
struct RetunedRun {
    bool               ran = false;
    std::vector<float> samples;
    gr::Size_t         interval         = 0U;
    gr::Size_t         reportedInterval = 0U;
};

/// Runs 8192 counted samples carrying `tags` through a long-form `DcBlocker<float>` of length `kLength`.
[[nodiscard]] RetunedRun runRetuned(const std::vector<gr::Tag>& tags) {
    using gr::blocks::testing::ProcessFunction;
    using gr::blocks::testing::TagSink;
    using gr::blocks::testing::TagSource;

    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 8192U}, {"mark_tag", false}});
    source._tags     = tags;
    auto& block      = graph.emplaceBlock<DcBlocker<float>>({{"length", kLength}, {"long_form", true}});
    auto& sink       = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

    RetunedRun run;
    if (!graph.connect<"out", "in">(source, block).has_value() || !graph.connect<"out", "in">(block, sink).has_value()) {
        return run;
    }
    gr::scheduler::Simple scheduler;
    run.ran              = scheduler.exchange(std::move(graph)).has_value() && scheduler.runAndWait().has_value();
    run.samples          = std::vector<float>(sink._samples.begin(), sink._samples.end());
    run.interval         = block.reseed_interval.value;
    run.reportedInterval = reported(block, "reseed_interval");
    return run;
}

} // namespace

const boost::ut::suite<"DcBlocker"> dcBlockerTests = [] {
    using namespace boost::ut;

    "a constant is removed exactly"_test = [] {
        for (const bool longForm : {true, false}) {
            // The pipeline fills in stages*(D-1) samples, twice the group delay. The cascade's impulse response is
            // stages*(D-1)+1 long, and the group delay is its center. The step transient is exactly half way at the
            // group delay and ends at the fill.
            const std::size_t        transient = (longForm ? 4UZ : 2UZ) * (kLength - 1UZ);
            DcBlocker<float>         block     = makeBlock<float>({{"length", kLength}, {"long_form", longForm}});
            const std::vector<float> input(2000UZ, 1.f);
            const std::vector<float> output = run<float>(block, std::span<const float>(input));
            for (std::size_t n = transient; n < output.size(); ++n) {
                expect(lt(std::abs(static_cast<double>(output[n])), 1e-6)) << (longForm ? "long" : "short") << " form at " << n;
            }

            DcBlocker<CF>         complexBlock = makeBlock<CF>({{"length", kLength}, {"long_form", longForm}});
            const std::vector<CF> complexInput(2000UZ, CF{1.f, 1.f});
            const std::vector<CF> complexOutput = run<CF>(complexBlock, std::span<const CF>(complexInput));
            for (std::size_t n = transient; n < complexOutput.size(); ++n) {
                expect(lt(static_cast<double>(std::abs(complexOutput[n])), 1e-6)) << "complex " << (longForm ? "long" : "short") << " at " << n;
            }
        }
    };

    "group_delay is stated and is where the step is half way"_test = [] {
        for (const bool longForm : {true, false}) {
            const std::size_t stages = longForm ? 4UZ : 2UZ;
            const std::size_t delay  = stages / 2UZ * (kLength - 1UZ);

            DcBlocker<float> block = makeBlock<float>({{"length", kLength}, {"long_form", longForm}});
            expect(eq(block.group_delay.value, static_cast<gr::Size_t>(delay)));

            const std::vector<float> input(2000UZ, 1.f);
            const std::vector<float> output = run<float>(block, std::span<const float>(input));
            expect(lt(std::abs(std::abs(static_cast<double>(output[delay])) - 0.5), 0.05)) << (longForm ? "long" : "short") << " form is half way through its transient at the group delay";
            expect(gt(std::abs(static_cast<double>(output[delay - 1UZ])), 0.1)) << "which is well inside it, not at its end";
        }
    };

    "the passband follows |1 - A(f)^stages|"_test = [] {
        const auto formula = [](bool longForm, double f) {
            const double d = static_cast<double>(kLength);
            const double a = std::sin(std::numbers::pi * d * f) / (d * std::sin(std::numbers::pi * f));
            return std::abs(1.0 - std::pow(a, longForm ? 4.0 : 2.0));
        };
        for (const double f : {0.005, 0.010, 0.015625, 0.200}) {
            for (const bool longForm : {true, false}) {
                const double want = formula(longForm, f);
                expect(lt(std::abs(response(longForm, f) - want) / want, 1e-3)) << (longForm ? "long" : "short") << " form at f/fs = " << f << ", formula " << want;
            }
        }
        expect(gt(response(true, 0.2), response(false, 0.2))) << "the long form is the flatter of the two";
    };

    "the long form has the narrower notch as well"_test = [] {
        const double longCorner  = corner(true);
        const double shortCorner = corner(false);
        expect(lt(std::abs(longCorner - 0.013088) / 0.013088, 0.05)) << "long form corner measured at " << longCorner;
        expect(lt(std::abs(shortCorner - 0.017908) / 0.017908, 0.05)) << "short form corner measured at " << shortCorner;
        expect(lt(longCorner, shortCorner)) << "the long form's corner is the lower of the two";

        expect(lt(std::abs(response(true, 1.0 / static_cast<double>(kLength)) - 1.0), 1e-3)) << "and the response is back at unity on the boxcar's own null";
    };

    "the output is bit-identical whatever the chunking"_test = [] {
        const std::vector<float> input = pseudoRandom(4UZ * kReseed);
        const gr::property_map   settings{{"length", kLength}, {"long_form", true}, {"reseed_interval", kReseed}};

        DcBlocker<float>         reference = makeBlock<float>(settings);
        const std::vector<float> want      = run<float>(reference, std::span<const float>(input));

        for (const std::size_t chunkSize : {1UZ, 7UZ, 32UZ, 4095UZ, 4096UZ, 4097UZ}) {
            DcBlocker<float>         block = makeBlock<float>(settings);
            const std::vector<float> got   = run<float>(block, std::span<const float>(input), chunkSize);
            expect(std::ranges::equal(got, want)) << "chunk size " << chunkSize;
        }
    };

    "the reseed keeps the drift bounded"_test = [] {
        const std::vector<float>  input = pseudoRandom(1000000UZ);
        const std::vector<double> want  = referenceBlocker(std::span<const float>(input), kLength, 2UZ);

        const auto drift = [&input, &want](gr::Size_t interval) {
            DcBlocker<float>         block = makeBlock<float>({{"length", kLength}, {"long_form", false}, {"reseed_interval", interval}});
            const std::vector<float> got   = run<float>(block, std::span<const float>(input), 997UZ);
            double                   worst = 0.0;
            for (std::size_t n = 0UZ; n < got.size(); ++n) {
                worst = std::max(worst, std::abs(static_cast<double>(got[n]) - want[n]));
            }
            return worst;
        };

        const double reseeded  = drift(kReseed);
        const double unbounded = drift(1000000000U); // the same run with no reseed inside a million samples
        std::println("DcBlocker short-form drift over 1e6 samples: {:.3e} at R = {}, {:.3e} with no reseed", reseeded, kReseed, unbounded);
        expect(lt(reseeded, 7e-7)) << "the bound is a factor of two above the drift this pipeline shows";
        expect(gt(unbounded, 2.0 * reseeded)) << "and the reseed is what holds it there";
    };

    "a length below two is rejected at construction"_test = [] {
        expect(throws([] { std::ignore = makeBlock<float>({{"length", 1U}}); }));
        expect(throws([] { std::ignore = makeBlock<float>({{"length", 0U}}); }));
    };

    "nanoseconds per sample"_test = [] {
        if (std::getenv("ENABLE_BENCHMARK_TESTS") == nullptr) {
            return; // opt-in, since a throughput figure belongs to a controlled run and not to every ctest invocation
        }
        using Clock = std::chrono::steady_clock;

        const std::vector<float> x = pseudoRandom(1UZ << 16);
        std::vector<float>       y(x.size());

        struct Arm {
            const char*      label;
            gr::property_map settings;
        };
        const Arm     kArms[]  = {{"DcBlocker<float> long D=32", {{"length", kLength}, {"long_form", true}}}, {"DcBlocker<float> short D=32", {{"length", kLength}, {"long_form", false}}}};
        constexpr int kRepeats = 7;

        std::vector<DcBlocker<float>> blocks;
        for (const Arm& arm : kArms) {
            blocks.push_back(makeBlock<float>(arm.settings));
        }

        std::vector<double> best(std::size(kArms), 1e30);
        std::vector<double> worst(std::size(kArms), 0.0);
        for (int repeat = 0; repeat < kRepeats; ++repeat) { // arms interleaved, so a thermal drift moves all of them
            for (std::size_t a = 0UZ; a < std::size(kArms); ++a) {
                const auto start = Clock::now();
                std::ignore      = blocks[a].processBulk(std::span<const float>(x), std::span<float>(y));
                const double ns  = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count()) / static_cast<double>(x.size());
                best[a]          = std::min(best[a], ns);
                worst[a]         = std::max(worst[a], ns);
            }
        }
        for (std::size_t a = 0UZ; a < std::size(kArms); ++a) {
            std::println("{}: best {:.3f} ns/sample, spread {:.3f} ns", kArms[a].label, best[a], worst[a] - best[a]);
        }
    };

    "every tag moves by the group delay, and a tag past the end leaves at the end-of-stream index"_test = [] {
        for (const bool longForm : {true, false}) {
            const std::size_t delay = (longForm ? 2UZ : 1UZ) * (kLength - 1UZ);
            const auto        got   = runTagged({{"length", kLength}, {"long_form", longForm}}, 2048U, {7UZ, 300UZ, 2047UZ - delay, 2048UZ - delay, 2047UZ});
            expect(got.ran);
            expect(eq(got.samples, 2048UZ)) << "every output";
            expect(that % (got.endIndex == std::optional<std::size_t>{2048UZ})) << "the stream ends one past the last output";
            for (const std::string_view key : {"trigger_name", "private_key"}) {
                expect(that % (got.offsetsOf(key) == std::vector<std::size_t>{7UZ + delay, 300UZ + delay, 2047UZ, 2048UZ, 2048UZ})) << std::format("{} form: {} on input i leaves on output i + {}, and past the last output at the end-of-stream index", longForm ? "long" : "short", key, delay);
                expect(that % (got.sampleOffsetsOf(key) == std::vector<std::size_t>{7UZ + delay, 300UZ + delay, 2047UZ})) << std::format("{}: a sample-by-sample consumer sees the tags on samples", key);
            }
        }
    };

    "tags that an upstream blocker leaves at its end-of-stream index leave this one at its own"_test = [] {
        constexpr gr::Size_t kInputs = 512U;
        for (const bool longForm : {true, false}) {
            const std::size_t delay = (longForm ? 2UZ : 1UZ) * (kLength - 1UZ);
            const std::size_t both  = 2UZ * delay;
            // 100 lies inside the stream, the next two on the last output and one past it, the last two past both delays
            const std::vector<std::size_t> at{100UZ, kInputs - 1UZ - both, kInputs - both, kInputs - 8UZ, kInputs - 1UZ};
            std::vector<gr::Tag>           tags;
            for (const std::size_t offset : at) {
                tags.emplace_back(offset, gr::property_map{{gr::property_map::key_type{"trigger_name"}, std::string("edge")}, {gr::property_map::key_type{"private_key"}, std::string("carried")}});
            }
            const gr::property_map settings{{"length", kLength}, {"long_form", longForm}};
            const auto             got = gr::blocks::filter::testing::runChained<DcBlocker<float>, DcBlocker<float>>(settings, settings, kInputs, tags);
            expect(got.ran);
            expect(eq(got.samples, std::size_t{kInputs})) << "every output";
            expect(that % (got.endIndex == std::optional<std::size_t>{std::size_t{kInputs}})) << "the stream ends one past the last output";
            for (const std::string_view key : {"trigger_name", "private_key"}) {
                expect(that % (got.offsetsOf(key) == std::vector<std::size_t>{100UZ + both, kInputs - 1UZ, kInputs, kInputs, kInputs})) << std::format("{} form: {} moves by both delays, and a tag the first blocker leaves at its end index leaves the second at the downstream end index", longForm ? "long" : "short", key);
                expect(that % (got.sampleOffsetsOf(key) == std::vector<std::size_t>{100UZ + both, kInputs - 1UZ})) << std::format("{}: a sample-by-sample consumer sees only the tags that lie on samples", key);
            }
        }
    };

    "length and long_form are refused while the block runs and accepted while it is stopped"_test = [] {
        const auto running = [](gr::property_map settings) {
            auto block = std::make_unique<DcBlocker<float>>(std::move(settings));
            block->settings().init();
            std::ignore = block->settings().applyStagedParameters();
            block->start();
            return block;
        };
        const auto live = [](DcBlocker<float>& block, gr::property_map changes) {
            std::ignore = block.settings().set(std::move(changes));
            std::ignore = block.settings().activateContext();
            std::ignore = block.settings().applyStagedParameters();
        };

        {
            auto block = running({{"length", gr::Size_t(32)}, {"long_form", true}});
            expect(throws([&] { live(*block, {{"length", gr::Size_t(64)}}); })) << "a running block refuses a new length";
        }
        {
            auto block = running({{"length", gr::Size_t(32)}, {"long_form", true}});
            expect(throws([&] { live(*block, {{"long_form", false}}); })) << "and a new form";
        }
        {
            // reseed_interval touches no pipeline state and stays live
            auto             block  = running({{"length", gr::Size_t(32)}, {"long_form", true}});
            const gr::Size_t before = block->group_delay;
            expect(nothrow([&] { live(*block, {{"reseed_interval", gr::Size_t(2048)}}); }));
            expect(eq(block->reseed_interval.value, gr::Size_t(2048)));
            expect(eq(block->group_delay.value, before)) << "and moves nothing the refusals protect";
        }
        {
            // a stopped block takes both
            auto             block  = running({{"length", gr::Size_t(32)}, {"long_form", true}});
            const gr::Size_t before = block->group_delay;
            block->stop();
            expect(nothrow([&] { live(*block, {{"length", gr::Size_t(64)}}); }));
            expect(eq(block->length.value, gr::Size_t(64)));
            expect(block->group_delay.value != before) << "and the rebuilt pipeline states its new delay";
        }
    };

    "a NaN or infinite input leaves the output non-finite for fewer than stages * (reseed_interval + length) samples"_test = [] {
        constexpr std::size_t kAt = 100UZ;
        for (const bool longForm : {true, false}) {
            const std::size_t stages = longForm ? 4UZ : 2UZ;
            const std::size_t delay  = stages / 2UZ * (kLength - 1UZ);
            for (const gr::Size_t interval : {kReseed, gr::Size_t(16)}) {
                const std::size_t bound = stages * (static_cast<std::size_t>(interval) + kLength);
                for (const float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
                    const gr::property_map   settings{{"length", kLength}, {"long_form", longForm}, {"reseed_interval", interval}};
                    std::vector<float>       input = pseudoRandom(kAt + bound + 4096UZ);
                    DcBlocker<float>         clean = makeBlock<float>(settings);
                    const std::vector<float> want  = run<float>(clean, std::span<const float>(input));
                    input[kAt]                     = bad;
                    DcBlocker<float>         block = makeBlock<float>(settings);
                    const std::vector<float> got   = run<float>(block, std::span<const float>(input));

                    std::size_t last = 0UZ;
                    for (std::size_t n = 0UZ; n < got.size(); ++n) {
                        last = std::isfinite(got[n]) ? last : n;
                    }
                    const std::string label = std::format("{} form, reseed_interval {}, input {}", longForm ? "long" : "short", interval, bad);
                    expect(ge(last, kAt + delay)) << label << ": the delayed input itself is non-finite";
                    expect(lt(last, kAt + bound)) << label << ": the last non-finite output lies inside the bound";
                    expect(std::ranges::equal(std::span<const float>(got).subspan(kAt + bound), std::span<const float>(want).subspan(kAt + bound))) << label << ": and from there the output is the clean stream's, bit for bit";
                }
            }
        }
    };

    "a refused change leaves the block computing on the pipeline it accepted"_test = [] {
        // The reference sees only the accepted changes. After every refusal both blocks filter the same samples, and the
        // outputs must agree bit for bit.
        const std::vector<float> x       = pseudoRandom(6UZ * 8192UZ);
        const auto               started = [] {
            auto block = std::make_unique<DcBlocker<float>>(gr::property_map{{"length", kLength}, {"long_form", true}});
            block->settings().init();
            std::ignore = block->settings().applyStagedParameters();
            block->start();
            return block;
        };
        const auto apply = [](DcBlocker<float>& block, gr::property_map changes) {
            std::ignore = block.settings().setStaged(std::move(changes));
            std::ignore = block.settings().applyStagedParameters();
        };
        auto        block     = started();
        auto        reference = started();
        std::size_t at        = 0UZ;
        const auto  same      = [&](std::size_t count) {
            const std::span<const float> part = std::span<const float>(x).subspan(at, count);
            at += count;
            return std::ranges::equal(run<float>(*block, part), run<float>(*reference, part));
        };
        expect(same(5000UZ));

        expect(refused(*block, {{"reseed_interval", gr::Size_t(0)}})) << "the limits refuse reseed_interval = 0";
        expect(eq(block->reseed_interval.value, kReseed) && eq(reported(*block, "reseed_interval"), kReseed)) << "the member and settings().get() keep the accepted interval";
        expect(same(8192UZ)) << "the block reseeds at the accepted interval";

        expect(throws([&] { apply(*block, {{"length", gr::Size_t(64)}}); })) << "a running block refuses a new length";
        expect(eq(block->group_delay.value, gr::Size_t(62))) << "and keeps the pipeline it runs";
        expect(nothrow([&] { apply(*block, {{"reseed_interval", gr::Size_t(2048)}}); })) << "a later change of another setting is checked on its own";
        apply(*reference, {{"reseed_interval", gr::Size_t(2048)}});
        expect(same(8192UZ)) << "after a refused length";

        expect(throws([&] { apply(*block, {{"group_delay", gr::Size_t(5)}}); })) << "group_delay follows from length and long_form and takes no write";
        expect(eq(reported(*block, "group_delay"), gr::Size_t(5))) << "settings().get() returns the written value until the next accepted application";
        expect(same(4096UZ)) << "after a refused write to group_delay";
        apply(*block, {{"reseed_interval", gr::Size_t(1024)}});
        apply(*reference, {{"reseed_interval", gr::Size_t(1024)}});
        expect(eq(block->group_delay.value, gr::Size_t(62))) << "an accepted application writes group_delay again";

        for (auto* restarted : {block.get(), reference.get()}) {
            restarted->stop();
            restarted->reset();
            restarted->start();
        }
        expect(eq(block->group_delay.value, gr::Size_t(62))) << "a restart rebuilds the accepted pipeline, not the refused length";
        expect(same(8192UZ)) << "after a restart";
    };

    "a block made from the settings saved from a configured block filters as that block"_test = [] {
        DcBlocker<float>       configured = makeBlock<float>({{"length", gr::Size_t(64)}, {"long_form", false}, {"reseed_interval", gr::Size_t(512)}});
        const gr::property_map stored     = savedSettings(configured);
        expect(stored.contains(gr::property_map::key_type{"group_delay"})) << "the saved settings carry the derived delay";

        const auto reloaded = loaded<DcBlocker<float>>(stored);
        expect(reloaded->state() == gr::lifecycle::State::INITIALISED) << "the block accepts the delay its pipeline has";
        expect(eq(reloaded->group_delay.value, gr::Size_t(63)) && eq(reloaded->_interval, gr::Size_t(512)));
        const std::vector<float> x = pseudoRandom(4096UZ);
        expect(std::ranges::equal(run<float>(*reloaded, std::span<const float>(x)), run<float>(configured, std::span<const float>(x)))) << "and filters on the saved pipeline";

        // The framework leaves a key equal to its member out of the application. A map saved at the default length and
        // form therefore applies its group_delay without them.
        DcBlocker<float> atDefaults = makeBlock<float>({{"reseed_interval", gr::Size_t(512)}});
        gr::property_map edited     = savedSettings(atDefaults);
        edited.insert_or_assign(gr::property_map::key_type{"group_delay"}, gr::Size_t(5));
        expect(loaded<DcBlocker<float>>(edited)->state() == gr::lifecycle::State::ERROR) << "a saved group_delay other than the pipeline's is refused";
    };

    "a block saved after it refused a change of length loads again at the saved length"_test = [] {
        auto block = std::make_unique<DcBlocker<float>>(gr::property_map{{"length", gr::Size_t(64)}, {"long_form", false}});
        block->settings().init();
        std::ignore = block->settings().applyStagedParameters();
        block->start();
        expect(throws([&] {
            std::ignore = block->settings().setStaged({{"length", gr::Size_t(128)}});
            std::ignore = block->settings().applyStagedParameters();
        })) << "a running block refuses a new length";
        expect(eq(reported(*block, "length"), gr::Size_t(128)) && eq(reported(*block, "group_delay"), gr::Size_t(63))) << "the saved settings pair the refused length with the running pipeline's delay";

        const gr::property_map stored   = savedSettings(*block);
        const auto             reloaded = loaded<DcBlocker<float>>(stored);
        expect(reloaded->state() == gr::lifecycle::State::INITIALISED) << "the saved settings load again";
        expect(eq(reloaded->group_delay.value, gr::Size_t(127))) << "group_delay takes the delay D-1 of the pipeline the saved length builds";
        DcBlocker<float>         want = makeBlock<float>({{"length", gr::Size_t(128)}, {"long_form", false}});
        const std::vector<float> x    = pseudoRandom(4096UZ);
        expect(std::ranges::equal(run<float>(*reloaded, std::span<const float>(x)), run<float>(want, std::span<const float>(x)))) << "and filters at the saved length";

        gr::property_map edited = stored;
        edited.insert_or_assign(gr::property_map::key_type{"group_delay"}, gr::Size_t(5));
        const auto withLength = loaded<DcBlocker<float>>(edited);
        expect(withLength->state() == gr::lifecycle::State::INITIALISED && eq(withLength->group_delay.value, gr::Size_t(127))) << "a group_delay written with length takes the delay of the pipeline that application builds";
    };

    "a reset to defaults that moves length back is refused while the block runs, and the block keeps its history"_test = [] {
        // init() stores the defaults with length 32. Each block then takes length 64 through settings().set() while it is
        // stopped. The reference is made the same way and sees no reset.
        const std::vector<float> x       = pseudoRandom(2UZ * 8192UZ);
        const auto               retuned = [] {
            auto block = std::make_unique<DcBlocker<float>>(gr::property_map{{"length", kLength}, {"long_form", true}});
            block->settings().init();
            std::ignore = block->settings().applyStagedParameters();
            std::ignore = block->settings().set({{"length", gr::Size_t(64)}});
            std::ignore = block->settings().activateContext();
            std::ignore = block->settings().applyStagedParameters();
            return block;
        };
        {
            auto stopped = retuned();
            expect(eq(stopped->group_delay.value, gr::Size_t(126)));
            expect(nothrow([&] { stopped->settings().resetDefaults(); })) << "a stopped block takes the reset";
            expect(eq(stopped->length.value, kLength) && eq(stopped->group_delay.value, gr::Size_t(62))) << "and rebuilds at the default length";
        }

        auto block     = retuned();
        auto reference = retuned();
        block->start();
        reference->start();
        std::size_t at   = 0UZ;
        const auto  same = [&](std::size_t count) {
            const std::span<const float> part = std::span<const float>(x).subspan(at, count);
            at += count;
            return std::ranges::equal(run<float>(*block, part), run<float>(*reference, part));
        };
        expect(same(5000UZ));

        expect(throws([&] { block->settings().resetDefaults(); })) << "a running block refuses the reset that moves length back";
        expect(eq(block->length.value, kLength)) << "the refused default stays in the member";
        expect(eq(block->group_delay.value, gr::Size_t(126))) << "and the block keeps the pipeline it runs";
        expect(same(8192UZ)) << "with its history, since reset() did not run";
    };

    "a length or reseed_interval outside its limits on a tag mid-run leaves the run going on the accepted pipeline"_test = [] {
        const gr::property_map accepted{{"reseed_interval", gr::Size_t(16)}};
        const RetunedRun       clean = runRetuned({gr::Tag{0UZ, accepted}});
        const RetunedRun       got   = runRetuned({gr::Tag{0UZ, accepted}, gr::Tag{3000UZ, gr::property_map{{"reseed_interval", gr::Size_t(0)}}}, gr::Tag{5000UZ, gr::property_map{{"length", gr::Size_t(1)}}}});
        expect(clean.ran && eq(clean.samples.size(), 8192UZ)) << "the reference run";
        expect(got.ran) << "the refusals go out as plain messages, and the run completes";
        expect(eq(got.samples.size(), 8192UZ));
        expect(std::ranges::equal(got.samples, clean.samples)) << "the block reseeds at the accepted interval from the refused tags on";
        expect(eq(got.interval, gr::Size_t(16)) && eq(got.reportedInterval, gr::Size_t(16))) << "the member and settings().get() keep the accepted interval";
    };
};

int main() { /* tests are automatically registered and run */ }
