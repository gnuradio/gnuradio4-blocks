#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <numeric>
#include <random>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/algorithm/filter/RealToComplex.hpp>
#include <gnuradio-4.0/basic/RealToComplex.hpp>

#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_real_to_complex {

using gr::blocks::basic::RealToComplex;
using CF        = std::complex<float>;
using Block     = RealToComplex<float>;
using Kernel    = gr::filter::RealToComplex;
namespace spans = gr::blocks::testing::span;

[[nodiscard]] Block makeBlock(gr::property_map settings = {}) {
    Block block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

[[nodiscard]] Kernel makeKernel(double sampleRate, double transitionHz) { return Kernel::make(sampleRate, transitionHz).value(); }

[[nodiscard]] std::vector<float> counted(std::size_t n) {
    std::vector<float> x(n);
    std::iota(x.begin(), x.end(), 0.f);
    return x;
}

[[nodiscard]] std::vector<float> noise(std::size_t n, std::uint32_t seed) {
    std::mt19937                          rng(seed);
    std::uniform_real_distribution<float> uniform(-1.f, 1.f);
    std::vector<float>                    x(n);
    for (float& v : x) {
        v = uniform(rng);
    }
    return x;
}

[[nodiscard]] std::vector<CF> byKernel(double sampleRate, double transitionHz, std::span<const float> x) {
    Kernel          kernel = makeKernel(sampleRate, transitionHz);
    std::vector<CF> y;
    std::ignore = kernel.process(x, y);
    return y;
}

[[nodiscard]] gr::property_map probe(std::size_t which) { return {{"probe", static_cast<gr::Size_t>(which)}}; }

[[nodiscard]] float rateOf(const gr::Tag& tag) {
    const auto found = tag.map.find(gr::property_map::key_type{"sample_rate"});
    if (found == tag.map.end() || found->second.get_if<float>() == nullptr) {
        return 0.f;
    }
    return *found->second.get_if<float>();
}

/// What a run through test spans published.
struct Run {
    std::vector<CF>      samples;
    std::vector<gr::Tag> tags;
    std::size_t          consumed = 0UZ;

    [[nodiscard]] std::vector<std::size_t> offsetsOf(std::string_view key) const {
        std::vector<std::size_t> offsets;
        for (const gr::Tag& tag : tags) {
            if (tag.map.contains(gr::property_map::key_type{key})) {
                offsets.push_back(tag.index);
            }
        }
        return offsets;
    }
};

/// Feeds @p input to @p block in calls of @p chunks samples in turn, each with room for @p room outputs, continuing
/// @p run. A chunk of 0 samples is a call with no input. Tags reach the block at their absolute input indices.
void feed(Block& block, std::span<const float> input, std::span<const std::size_t> chunks, std::size_t room, Run& run, std::span<const gr::Tag> tags = {}) {
    std::vector<CF> scratch(room);
    std::size_t     base = run.consumed;
    for (std::size_t call = 0UZ; base < input.size() && call < 100'000UZ; ++call) {
        const std::size_t       count = std::min(chunks[call % chunks.size()], input.size() - base);
        spans::InputSpan<float> inSpan(input.subspan(base, count), base, tags);
        spans::OutputSpan<CF>   outSpan(std::span<CF>(scratch), run.samples.size(), &run.tags);
        std::ignore = block.processBulk(inSpan, outSpan);
        run.samples.insert(run.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        base += inSpan.consumed;
    }
    run.consumed = base;
}

/// Records its input, the end-of-stream tags included.
template<typename T>
struct EndSink : gr::Block<EndSink<T>> {
    gr::PortIn<T> in;

    GR_MAKE_REFLECTABLE(EndSink, in);

    std::vector<T>       _samples;
    std::vector<gr::Tag> _tags;

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& input) {
        record(input);
        return gr::work::Status::OK;
    }

    [[nodiscard]] gr::work::Status processEpilogue(gr::InputSpanLike auto& input) {
        record(input);
        const std::size_t end = input.streamIndex + input.size();
        for (const gr::Tag& tag : in.tagReader().get()) {
            if (tag.index >= end) {
                _tags.push_back(tag);
            }
        }
        return gr::work::Status::OK;
    }

    void record(auto& input) {
        const std::size_t end = input.streamIndex + input.size();
        for (const gr::Tag& tag : input.rawTags) {
            if (tag.index >= input.streamIndex && tag.index < end) {
                _tags.push_back(tag);
            }
        }
        _samples.insert(_samples.end(), input.begin(), input.end());
    }
};

/// What a graph run delivered.
struct GraphRun {
    bool                 ran = false;
    std::vector<CF>      samples;
    std::vector<gr::Tag> tags;
    gr::property_map     active;

    [[nodiscard]] std::vector<std::size_t> offsetsOf(std::string_view key) const {
        std::vector<std::size_t> offsets;
        for (const gr::Tag& tag : tags) {
            if (tag.map.contains(gr::property_map::key_type{key})) {
                offsets.push_back(tag.index);
            }
        }
        return offsets;
    }
};

/// Runs @p nSamples counted samples carrying @p tags through a block made with @p settings into an `EndSink`.
[[nodiscard]] GraphRun runGraph(gr::property_map settings, gr::Size_t nSamples, const std::vector<gr::Tag>& tags) {
    using gr::blocks::testing::ProcessFunction;
    using gr::blocks::testing::TagSource;

    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", nSamples}, {"mark_tag", false}, {"sample_rate", 48'000.f}});
    source._tags     = tags;
    auto& block      = graph.emplaceBlock<Block>(std::move(settings));
    auto& sink       = graph.emplaceBlock<EndSink<CF>>();

    GraphRun run;
    if (!graph.connect<"out", "in">(source, block).has_value() || !graph.connect<"out", "in">(block, sink).has_value()) {
        return run;
    }
    gr::scheduler::Simple scheduler;
    run.ran     = scheduler.exchange(std::move(graph)).has_value() && scheduler.runAndWait().has_value();
    run.samples = sink._samples;
    run.tags    = sink._tags;
    run.active  = block.settings().get();
    return run;
}

} // namespace qa_real_to_complex

const boost::ut::suite<"RealToComplex"> realToComplexTests = [] {
    using namespace boost::ut;
    using namespace qa_real_to_complex;

    "the output is the kernel's, in calls of any length and any output room"_test = [] {
        const auto        x         = noise(5001UZ, 1U);
        const auto        reference = byKernel(48'000.0, 200.0, x);
        const std::size_t chunks[]  = {1UZ, 7UZ, 0UZ, 64UZ, 333UZ, 2UZ};
        for (const std::size_t room : {1UZ, 3UZ, 4096UZ}) {
            Block block = makeBlock();
            Run   run;
            feed(block, x, chunks, room, run);
            expect(eq(run.consumed, x.size()));
            expect(eq(run.samples.size(), (x.size() + 1UZ) / 2UZ)) << "input 2n makes output n";
            expect(run.samples == reference) << std::format("room for {} outputs: the kernel's outputs, bit for bit", room);
        }
    };

    "a tag moves by the conversion's delay, and the first output states the output rate"_test = [] {
        Block                block = makeBlock({{"sample_rate", 500'000.f}, {"transition_hz", 20'000.f}});
        const std::size_t    delay = makeKernel(500'000.0, 20'000.0).groupDelay();
        const auto           x     = counted(400UZ);
        std::vector<gr::Tag> tags{{0UZ, probe(0UZ)}, {5UZ, probe(1UZ)}, {6UZ, probe(2UZ)}, {201UZ, {{"probe", gr::Size_t(3)}, {"sample_rate", 1.f}}}, {202UZ, {{"probe", gr::Size_t(4)}, {"sample_rate", 1.0}}}};
        const std::size_t    chunks[] = {37UZ};
        Run                  run;
        feed(block, x, chunks, 4096UZ, run, tags);

        std::vector<std::size_t> wanted;
        for (const gr::Tag& incoming : tags) {
            wanted.push_back((incoming.index + delay + 1UZ) / 2UZ);
        }
        expect(that % (run.offsetsOf("probe") == wanted)) << "input i leaves on output floor((i + D + 1)/2)";
        expect(fatal(!run.tags.empty()));
        expect(eq(run.tags.front().index, 0UZ) && eq(rateOf(run.tags.front()), 250'000.f)) << "the first tag states fs/2 at output 0";
        std::size_t restated = 0UZ;
        for (const gr::Tag& published : run.tags) {
            if (!published.map.contains(gr::property_map::key_type{"probe"}) || !published.map.contains(gr::property_map::key_type{"sample_rate"})) {
                continue;
            }
            ++restated;
            const auto& rate = published.map.at(gr::property_map::key_type{"sample_rate"});
            if (*published.map.at(gr::property_map::key_type{"probe"}).get_if<gr::Size_t>() == 4U) {
                expect(rate.get_if<double>() != nullptr && *rate.get_if<double>() == 250'000.0) << "a double sample_rate key leaves as a double stating the output rate";
            } else {
                expect(eq(rateOf(published), 250'000.f)) << "a float sample_rate key leaves as a float stating the output rate";
            }
        }
        expect(eq(restated, 2UZ)) << "both moved sample_rate keys are read";
    };

    "under the scheduler the tags move by the delay and the late ones leave at the end"_test = [] {
        constexpr gr::Size_t kSamples = 4000U;
        const std::size_t    delay    = makeKernel(48'000.0, 2'000.0).groupDelay();
        const std::size_t    last     = kSamples - 3U;
        const GraphRun       run      = runGraph({{"sample_rate", 48'000.f}, {"transition_hz", 2'000.f}}, kSamples, {{10UZ, probe(0UZ)}, {1001UZ, probe(1UZ)}, {last, probe(2UZ)}});
        expect(fatal(run.ran));
        expect(eq(run.samples.size(), std::size_t{kSamples} / 2UZ));
        expect(run.samples == byKernel(48'000.0, 2'000.0, counted(kSamples))) << "the kernel's outputs";
        const std::vector<std::size_t> wanted{(10UZ + delay + 1UZ) / 2UZ, (1001UZ + delay + 1UZ) / 2UZ, run.samples.size()};
        expect(that % (run.offsetsOf("probe") == wanted)) << "the last tag lies past the last output and leaves at the end-of-stream index";
        const auto rates = run.offsetsOf("sample_rate");
        expect(fatal(!rates.empty()));
        expect(eq(rates.front(), 0UZ) && eq(rateOf(run.tags.front()), 24'000.f)) << "the output rate leaves on output 0";
    };

    "a rate change on a tag restarts the conversion at the tagged sample"_test = [] {
        constexpr gr::Size_t  kSamples = 6000U;
        constexpr std::size_t kAt      = 2000UZ;
        const GraphRun        run      = runGraph({}, kSamples, {{1990UZ, probe(0UZ)}, {kAt, {{"sample_rate", 96'000.f}}}});
        expect(fatal(run.ran));
        expect(eq(*run.active.at("sample_rate").get_if<float>(), 96'000.f));

        const auto x      = counted(kSamples);
        auto       before = byKernel(48'000.0, 200.0, std::span(x).first(kAt));
        const auto after  = byKernel(96'000.0, 200.0, std::span(x).subspan(kAt));
        expect(fatal(eq(run.samples.size(), before.size() + after.size())));
        expect(std::equal(before.begin(), before.end(), run.samples.begin())) << "the first conversion up to the tag";
        expect(std::equal(after.begin(), after.end(), run.samples.begin() + static_cast<std::ptrdiff_t>(before.size()))) << "a fresh conversion at the new rate from the tag on";

        std::vector<std::pair<std::size_t, float>> rates;
        for (const gr::Tag& published : run.tags) {
            if (published.map.contains(gr::property_map::key_type{"sample_rate"})) {
                rates.emplace_back(published.index, rateOf(published));
            }
        }
        // The source states its own rate on input 0, and that tag moves by the delay as well.
        const std::size_t                                delayBefore = makeKernel(48'000.0, 200.0).groupDelay();
        const std::size_t                                delayAfter  = makeKernel(96'000.0, 200.0).groupDelay();
        const std::vector<std::pair<std::size_t, float>> wanted{{0UZ, 24'000.f}, {(delayBefore + 1UZ) / 2UZ, 24'000.f}, {before.size(), 48'000.f}, {before.size() + (delayAfter + 1UZ) / 2UZ, 48'000.f}};
        expect(rates == wanted) << std::format("the new rate leaves on the first output of the restart, and the moved tag states it too: {}", rates);
        expect(fatal(gt((1990UZ + delayBefore + 1UZ) / 2UZ, before.size()))) << "the tag on input 1990 is held across the restart";
        expect(that % (run.offsetsOf("probe") == std::vector<std::size_t>{before.size()})) << "a tag held across the restart leaves on the restart's first output";
    };

    "a value outside the limits on a tag is refused and the run goes on"_test = [] {
        constexpr gr::Size_t kSamples = 3000U;
        const GraphRun       run      = runGraph({}, kSamples, {{1000UZ, {{"sample_rate", -1.f}}}, {2000UZ, {{"transition_hz", std::numeric_limits<float>::quiet_NaN()}}}});
        expect(fatal(run.ran)) << "the run completes";
        expect(eq(*run.active.at("sample_rate").get_if<float>(), 48'000.f));
        expect(eq(*run.active.at("transition_hz").get_if<float>(), 200.f));
        expect(run.samples == byKernel(48'000.0, 200.0, counted(kSamples))) << "the block computes on the accepted values throughout";
        for (const gr::Tag& published : run.tags) {
            if (published.map.contains(gr::property_map::key_type{"sample_rate"})) {
                expect(eq(rateOf(published), 24'000.f)) << "every sample_rate leaving states the output rate";
            }
        }
    };

    "a transition of half the rate or more is refused across the two settings"_test = [] {
        Block      block = makeBlock();
        const auto apply = [&block](gr::property_map changes) {
            std::ignore = block.settings().setStaged(std::move(changes));
            std::ignore = block.settings().applyStagedParameters();
        };
        expect(throws([&apply] { apply({{"transition_hz", 24'000.f}}); })) << "the application is refused";
        expect(eq(block.transition_hz.value, 24'000.f)) << "the refused value stays in its member";

        const auto        x        = noise(1001UZ, 4U);
        const std::size_t chunks[] = {100UZ};
        Run               run;
        feed(block, x, chunks, 4096UZ, run);
        expect(run.samples == byKernel(48'000.0, 200.0, x)) << "and the block computes on the accepted transition";

        expect(nothrow([&apply] { apply({{"sample_rate", 40'000.f}}); })) << "a later rate is checked against the accepted transition: 40 kHz passes 200 Hz and fails the refused 24 kHz";
    };

    "a staged change restarts the conversion at the next call, and a new rate leaves first"_test = [] {
        Block                      block    = makeBlock();
        const auto                 x        = noise(3000UZ, 5U);
        const std::size_t          chunks[] = {101UZ};
        Run                        run;
        const std::vector<gr::Tag> waiting{{1100UZ, {{"probe", gr::Size_t(8)}, {"sample_rate", 1.f}}}};
        feed(block, std::span(x).first(1111UZ), chunks, 4096UZ, run, waiting);
        const std::size_t before = run.samples.size();
        expect(fatal(lt(before, (1100UZ + makeKernel(48'000.0, 200.0).groupDelay() + 1UZ) / 2UZ))) << "the tag on input 1100 still waits for its output";
        std::ignore = block.settings().setStaged({{"sample_rate", 8'000.f}, {"transition_hz", 400.f}});
        expect(fatal(block.settings().applyStagedParameters().failedParameters.empty()));
        const std::vector<gr::Tag> held{{1511UZ, probe(9UZ)}};
        feed(block, x, chunks, 4096UZ, run, held);

        const auto after = byKernel(8'000.0, 400.0, std::span(x).subspan(1111UZ));
        expect(fatal(eq(run.samples.size(), before + after.size())));
        expect(std::equal(after.begin(), after.end(), run.samples.begin() + static_cast<std::ptrdiff_t>(before))) << "the rebuilt conversion starts at the first sample of the next call";
        expect(fatal(ge(run.tags.size(), 3UZ)));
        expect(eq(run.tags[1].index, before) && eq(rateOf(run.tags[1]), 4'000.f)) << "the new output rate leaves on the first output of the rebuilt conversion";

        for (const gr::Tag& published : run.tags) {
            if (published.map.contains(gr::property_map::key_type{"probe"}) && published.map.contains(gr::property_map::key_type{"sample_rate"})) {
                expect(eq(rateOf(published), 4'000.f)) << "a tag waiting across the change states the new output rate";
            }
        }
        // The restart lies on input 1111, an odd one. The tag on input 1511 sits 400 inputs after it. With an odd delay a
        // count from the stream's start puts that tag one output early.
        const std::size_t delayAfter = makeKernel(8'000.0, 400.0).groupDelay();
        expect(fatal(eq(delayAfter % 2UZ, 1UZ)));
        const std::vector<std::size_t> wanted{before, before + (400UZ + delayAfter + 1UZ) / 2UZ};
        expect(that % (run.offsetsOf("probe") == wanted)) << "the waiting tag leaves on the restart's first output, and a tag after the restart moves by the new delay, counted from the restart";
    };

    "a tag held across a restart leaves on the restart's first output, and a later tag is not held behind it"_test = [] {
        Block                      block = makeBlock();
        const auto                 x     = noise(4000UZ, 9U);
        const std::vector<gr::Tag> tags{{1000UZ, probe(0UZ)}, {1012UZ, probe(1UZ)}};
        const std::size_t          upTo[] = {1000UZ, 10UZ};
        Run                        run;
        feed(block, std::span(x).first(1010UZ), upTo, 4096UZ, run, tags);
        const std::size_t before = run.samples.size();
        expect(fatal(eq(before, 505UZ)));
        const std::size_t delayBefore = makeKernel(48'000.0, 200.0).groupDelay();
        expect(fatal(gt((1000UZ + delayBefore + 1UZ) / 2UZ, before))) << "the tag on input 1000 waits across the restart";
        std::ignore = block.settings().setStaged({{"transition_hz", 2'000.f}});
        expect(fatal(block.settings().applyStagedParameters().failedParameters.empty()));
        const std::size_t rest[] = {2990UZ};
        feed(block, x, rest, 4096UZ, run, tags);

        const auto after = byKernel(48'000.0, 2'000.0, std::span(x).subspan(1010UZ));
        expect(fatal(eq(run.samples.size(), before + after.size())));
        expect(std::equal(after.begin(), after.end(), run.samples.begin() + static_cast<std::ptrdiff_t>(before))) << "the rebuilt conversion starts at input 1010";
        const std::size_t              delayAfter = makeKernel(48'000.0, 2'000.0).groupDelay();
        const std::vector<std::size_t> wanted{before, before + (2UZ + delayAfter + 1UZ) / 2UZ};
        expect(that % (run.offsetsOf("probe") == wanted)) << "the held tag on the restart's first output, the tag on input 1012 by the new delay";
    };

    "a refused rate on a tag ends a run whose messages nothing reads"_test = [] {
        const GraphRun accepted = runGraph({}, 3000U, {{1000UZ, {{"sample_rate", 500.f}}}});
        expect(accepted.ran) << "500 Hz is above twice the accepted 200 Hz transition, and the same graph completes";
        const GraphRun refused = runGraph({}, 3000U, {{1000UZ, {{"sample_rate", 300.f}}}});
        expect(!refused.ran) << "300 Hz is below twice the accepted 200 Hz transition";
    };

    "reset() returns the conversion to the start of a stream"_test = [] {
        Block             block    = makeBlock({{"transition_hz", 2'000.f}});
        const auto        x        = noise(2001UZ, 6U);
        const std::size_t chunks[] = {333UZ};
        Run               warm;
        feed(block, noise(777UZ, 7U), chunks, 4096UZ, warm);
        block.reset();
        Run run;
        feed(block, x, chunks, 4096UZ, run);
        expect(run.samples == byKernel(48'000.0, 2'000.0, x)) << "the outputs of a fresh conversion, bit for bit";
    };

    "a reset to defaults keeps the held tags and releases them on the first output after it"_test = [] {
        Block                      block = makeBlock();
        const auto                 x     = noise(4000UZ, 10U);
        const std::vector<gr::Tag> tags{{1000UZ, probe(0UZ)}};
        const std::size_t          upTo[] = {1010UZ};
        Run                        run;
        feed(block, std::span(x).first(1010UZ), upTo, 4096UZ, run, tags);
        const std::size_t before = run.samples.size();
        expect(fatal(eq(before, 505UZ)));
        expect(fatal(run.offsetsOf("probe").empty())) << "the tag on input 1000 is held";
        block.settings().resetDefaults();
        const std::size_t rest[] = {2990UZ};
        feed(block, x, rest, 4096UZ, run, tags);

        const auto after = byKernel(48'000.0, 200.0, std::span(x).subspan(1010UZ));
        expect(fatal(eq(run.samples.size(), before + after.size())));
        expect(std::equal(after.begin(), after.end(), run.samples.begin() + static_cast<std::ptrdiff_t>(before))) << "the reset restarts the conversion at input 1010";
        expect(that % (run.offsetsOf("probe") == std::vector<std::size_t>{before})) << "the held tag leaves on the first output after the reset";
    };

    "a NaN or an infinite sample does not stop the stream"_test = [] {
        using gr::blocks::testing::ProcessFunction;
        auto x  = noise(4000UZ, 8U);
        x[1000] = std::numeric_limits<float>::quiet_NaN();
        x[2001] = std::numeric_limits<float>::infinity();

        gr::Graph  graph;
        const auto values = gr::Tensor<float>(x.begin(), x.end());
        auto&      source = graph.emplaceBlock<gr::blocks::testing::TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 4000U}, {"values", values}, {"sample_rate", 48'000.f}});
        auto&      block  = graph.emplaceBlock<Block>({{"transition_hz", 2'000.f}});
        auto&      sink   = graph.emplaceBlock<EndSink<CF>>();
        expect(fatal(graph.connect<"out", "in">(source, block).has_value() && graph.connect<"out", "in">(block, sink).has_value()));
        gr::scheduler::Simple scheduler;
        expect(fatal(scheduler.exchange(std::move(graph)).has_value() && scheduler.runAndWait().has_value())) << "the run completes";

        const auto reference = byKernel(48'000.0, 2'000.0, x);
        expect(eq(sink._samples.size(), reference.size())) << "every output is made";
        const std::size_t n      = makeKernel(48'000.0, 2'000.0).taps().size();
        const auto        finite = [](CF v) { return std::isfinite(v.real()) && std::isfinite(v.imag()); };
        expect(fatal(eq(sink._samples.size(), reference.size())));
        // output m reads inputs 2m - N + 1 to 2m
        const auto readers = [&](std::size_t at) { return std::span(sink._samples).subspan(at / 2UZ, (at + n + 1UZ) / 2UZ - at / 2UZ); };
        expect(!std::ranges::all_of(readers(1000UZ), finite)) << "the outputs that read the NaN are not finite";
        expect(!std::ranges::all_of(readers(2001UZ), finite)) << "nor are those that read the infinity";
        const std::size_t clean = (2001UZ + n + 1UZ) / 2UZ;
        expect(std::ranges::all_of(std::span(sink._samples).subspan(clean), finite)) << "the outputs after them are finite";
        expect(std::ranges::equal(std::span(sink._samples).subspan(clean), std::span(reference).subspan(clean))) << "and they are the kernel's";
    };
};

int main() { /* not needed for UT */ }
