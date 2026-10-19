#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <numeric>
#include <random>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/algorithm/filter/RealToComplex.hpp>
#include <gnuradio-4.0/basic/RealToComplex.hpp>
#include <gnuradio-4.0/basic/VectorSource.hpp>

#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_real_to_complex {

using gr::blocks::basic::RealToComplex;
using CF        = std::complex<float>;
using Block     = RealToComplex<float>;
using Kernel    = gr::filter::RealToComplex;
namespace spans = gr::blocks::testing::span;

constexpr float kDefaultTransition = 0.02f;    ///< the block's default `transition`
constexpr float kSourceRate        = 48'000.f; ///< the rate the graph runs' source states

[[nodiscard]] Block makeBlock(gr::property_map settings = {}) {
    Block block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

[[nodiscard]] Kernel makeKernel(float transition) { return Kernel::make(1.0, static_cast<double>(transition)).value(); }

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

[[nodiscard]] std::vector<CF> byKernel(float transition, std::span<const float> x) {
    Kernel          kernel = makeKernel(transition);
    std::vector<CF> y;
    std::ignore = kernel.process(x, y);
    return y;
}

[[nodiscard]] gr::property_map probe(std::size_t which) { return {{"probe", static_cast<gr::Size_t>(which)}}; }

[[nodiscard]] float rateOf(const gr::Tag& tag) {
    const auto found = tag.map.find(gr::property_map::key_type{gr::tag::SAMPLE_RATE.key()});
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

/// Runs @p nSamples counted samples carrying @p tags from a source at `kSourceRate` through a block made with
/// @p settings into an `EndSink`. With @p coupled the block's `transition` follows a tag key of that name.
[[nodiscard]] GraphRun runGraph(gr::property_map settings, gr::Size_t nSamples, const std::vector<gr::Tag>& tags, bool coupled = true) {
    using gr::blocks::testing::ProcessFunction;
    using gr::blocks::testing::TagSource;

    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", nSamples}, {"mark_tag", false}, {"sample_rate", kSourceRate}});
    source._tags     = tags;
    auto& block      = graph.emplaceBlock<Block>(std::move(settings));
    if (coupled) {
        block.settings().setTagCouplings({{"transition", std::string("transition")}});
    }
    auto& sink = graph.emplaceBlock<EndSink<CF>>();

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
        const auto        reference = byKernel(kDefaultTransition, x);
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

    "a transition too narrow for 4095 taps gets the width 4095 taps reach"_test = [] {
        constexpr float   kRequested = 2e-5f;
        const double      requested  = static_cast<double>(kRequested);
        const auto        figures    = gr::filter::fir::design::windowFigures(gr::algorithm::window::Type::Hamming);
        const double      achieved   = figures.transitionConstant / static_cast<double>(Kernel::kMaxTaps);
        const std::size_t taps       = Kernel::tapCount(1.0, requested).value();
        expect(eq(taps, Kernel::kMaxTaps)) << std::format("a transition of {} is held at the cap, a transition of {:.3g}", kRequested, achieved);
        expect(lt(requested, achieved));

        // A tone at f leaves at f - fs/4, and its mirror at fs/4 - f. The filter weighs them at fs/4 - f and fs/4 + f.
        const std::vector<float> h        = Kernel::design(1.0, requested).value();
        const auto               designDb = [&h](double normalized) {
            std::complex<double> wanted{};
            std::complex<double> mirror{};
            for (std::size_t k = 0UZ; k < h.size(); ++k) {
                const double phase = 2.0 * std::numbers::pi * static_cast<double>(k);
                wanted += static_cast<double>(h[k]) * std::polar(1.0, -phase * (0.25 - normalized));
                mirror += static_cast<double>(h[k]) * std::polar(1.0, -phase * (0.25 + normalized));
            }
            return 20.0 * std::log10(std::abs(mirror) / std::abs(wanted));
        };
        // The two leave 4 f / fs apart in cycles per output sample, 1.3 DFT bins of this record at a tone of 2e-5 fs. A
        // least-squares fit of the two exponentials separates them at any spacing.
        const auto measuredDb = [](const std::vector<CF>& y, double normalized, std::size_t first) {
            const double         wantedAt = 2.0 * normalized - 0.5;
            const double         mirrorAt = 0.5 - 2.0 * normalized;
            std::complex<double> cross{};
            std::complex<double> onWanted{};
            std::complex<double> onMirror{};
            for (std::size_t n = first; n < y.size(); ++n) {
                const double               index  = static_cast<double>(n);
                const std::complex<double> wanted = std::polar(1.0, 2.0 * std::numbers::pi * wantedAt * index);
                const std::complex<double> mirror = std::polar(1.0, 2.0 * std::numbers::pi * mirrorAt * index);
                cross += std::conj(wanted) * mirror;
                onWanted += std::conj(wanted) * std::complex<double>(y[n]);
                onMirror += std::conj(mirror) * std::complex<double>(y[n]);
            }
            const double               count       = static_cast<double>(y.size() - first);
            const double               determinant = count * count - std::norm(cross);
            const std::complex<double> a           = (count * onWanted - cross * onMirror) / determinant;
            const std::complex<double> b           = (count * onMirror - std::conj(cross) * onWanted) / determinant;
            return 20.0 * std::log10(std::abs(b) / std::abs(a));
        };

        // the mirror's level against the tone's at the block's output, and the design's figure for it
        const auto mirrorOf = [&](double f) {
            std::vector<float> x(40'960UZ);
            for (std::size_t i = 0UZ; i < x.size(); ++i) {
                x[i] = static_cast<float>(std::sin(2.0 * std::numbers::pi * f * static_cast<double>(i)));
            }
            Block                              block = makeBlock({{"transition", kRequested}});
            Run                                run;
            const std::array<std::size_t, 1UZ> chunks{4096UZ};
            feed(block, x, chunks, 4096UZ, run);
            return std::pair{measuredDb(run.samples, f, taps), designDb(f)};
        };

        const double edge                     = 0.5 * achieved;
        const auto [edgeMeasured, edgeDesign] = mirrorOf(edge);
        expect(lt(std::abs(edgeMeasured - edgeDesign), 0.1)) << std::format("{:.3g} fs: the block's mirror at {:.2f} dB against the design's {:.2f} dB", edge, edgeMeasured, edgeDesign);
        expect(lt(edgeMeasured, -(figures.attenuationDb - 3.0))) << std::format("a tone at half the achieved width, {:.3g} fs, has its mirror {:.1f} dB down", edge, edgeMeasured);

        const auto [measured, designed] = mirrorOf(requested);
        expect(lt(std::abs(measured - designed), 0.1)) << std::format("{} fs: the block's mirror at {:.2f} dB against the design's {:.2f} dB", kRequested, measured, designed);
        expect(gt(measured, -6.0)) << std::format("a tone at {} fs lies inside the achieved transition, and its mirror is only {:.2f} dB down", kRequested, measured);
    };

    "a tag moves by the conversion's delay, and a sample_rate key leaves at half its value"_test = [] {
        Block                block = makeBlock({{"transition", 0.04f}});
        const std::size_t    delay = makeKernel(0.04f).groupDelay();
        const auto           x     = counted(400UZ);
        std::vector<gr::Tag> tags{{0UZ, probe(0UZ)}, {5UZ, probe(1UZ)}, {6UZ, probe(2UZ)}, {201UZ, {{"probe", gr::Size_t(3)}, {gr::tag::SAMPLE_RATE.key(), 1.f}}}, {202UZ, {{"probe", gr::Size_t(4)}, {gr::tag::SAMPLE_RATE.key(), 1.0}}}, {203UZ, {{"probe", gr::Size_t(5)}, {gr::tag::SAMPLE_RATE.key(), std::int64_t{96'000}}}}};
        const std::size_t    chunks[] = {37UZ};
        Run                  run;
        feed(block, x, chunks, 4096UZ, run, tags);

        std::vector<std::size_t> wanted;
        for (const gr::Tag& incoming : tags) {
            wanted.push_back((incoming.index + delay + 1UZ) / 2UZ);
        }
        expect(that % (run.offsetsOf("probe") == wanted)) << "input i leaves on output floor((i + D + 1)/2)";
        const std::vector<std::size_t> rated{wanted[3], wanted[4], wanted[5]};
        expect(that % (run.offsetsOf(gr::tag::SAMPLE_RATE.key()) == rated)) << "the block publishes no rate tag of its own";
        std::size_t restated = 0UZ;
        for (const gr::Tag& published : run.tags) {
            if (!published.map.contains(gr::property_map::key_type{"probe"}) || !published.map.contains(gr::property_map::key_type{gr::tag::SAMPLE_RATE.key()})) {
                continue;
            }
            ++restated;
            const auto&      rate  = published.map.at(gr::property_map::key_type{gr::tag::SAMPLE_RATE.key()});
            const gr::Size_t which = *published.map.at(gr::property_map::key_type{"probe"}).get_if<gr::Size_t>();
            if (which == 3U) {
                expect(eq(rateOf(published), 0.5f * 1.f)) << "a float sample_rate key leaves as a float at half its value";
            } else if (which == 4U) {
                expect(rate.get_if<double>() != nullptr && *rate.get_if<double>() == 0.5 * 1.0) << "a double sample_rate key leaves as a double at half its value";
            } else {
                expect(rate.get_if<std::int64_t>() != nullptr && *rate.get_if<std::int64_t>() == std::int64_t{96'000}) << "a sample_rate key of another type leaves as it arrived";
            }
        }
        expect(eq(restated, 3UZ)) << "the three moved sample_rate keys are read";
    };

    "under the scheduler the tags move by the delay and the late ones leave at the end"_test = [] {
        constexpr gr::Size_t kSamples = 4000U;
        const std::size_t    delay    = makeKernel(0.04f).groupDelay();
        const std::size_t    last     = kSamples - 3U;
        const GraphRun       run      = runGraph({{"transition", 0.04f}}, kSamples, {{10UZ, probe(0UZ)}, {1001UZ, probe(1UZ)}, {last, probe(2UZ)}});
        expect(fatal(run.ran));
        expect(eq(run.samples.size(), std::size_t{kSamples} / 2UZ));
        expect(run.samples == byKernel(0.04f, counted(kSamples))) << "the kernel's outputs";
        const std::vector<std::size_t> wanted{(10UZ + delay + 1UZ) / 2UZ, (1001UZ + delay + 1UZ) / 2UZ, run.samples.size()};
        expect(that % (run.offsetsOf("probe") == wanted)) << "the last tag lies past the last output and leaves at the end-of-stream index";
        // The source states its rate on input 0. That tag is the only rate tag leaving, moved by the delay.
        const auto rates = run.offsetsOf(gr::tag::SAMPLE_RATE.key());
        expect(fatal(that % (rates == std::vector<std::size_t>{(delay + 1UZ) / 2UZ}))) << "the block publishes no rate tag of its own";
        const auto first = std::ranges::find_if(run.tags, [](const gr::Tag& published) { return published.map.contains(gr::property_map::key_type{gr::tag::SAMPLE_RATE.key()}); });
        expect(eq(rateOf(*first), 0.5f * kSourceRate)) << "the source's rate leaves halved";
    };

    "a transition change on a tag restarts the conversion at the tagged sample"_test = [] {
        constexpr gr::Size_t  kSamples = 6000U;
        constexpr std::size_t kAt      = 2000UZ;
        constexpr float       kNext    = 0.008f;
        const GraphRun        run      = runGraph({}, kSamples, {{1990UZ, probe(0UZ)}, {kAt, {{"transition", kNext}}}});
        expect(fatal(run.ran));
        expect(eq(*run.active.at("transition").get_if<float>(), kNext));

        const auto x      = counted(kSamples);
        auto       before = byKernel(kDefaultTransition, std::span(x).first(kAt));
        const auto after  = byKernel(kNext, std::span(x).subspan(kAt));
        expect(fatal(eq(run.samples.size(), before.size() + after.size())));
        expect(std::equal(before.begin(), before.end(), run.samples.begin())) << "the first conversion up to the tag";
        expect(std::equal(after.begin(), after.end(), run.samples.begin() + static_cast<std::ptrdiff_t>(before.size()))) << "a fresh conversion at the new transition from the tag on";

        const std::size_t delayBefore = makeKernel(kDefaultTransition).groupDelay();
        expect(that % (run.offsetsOf(gr::tag::SAMPLE_RATE.key()) == std::vector<std::size_t>{(delayBefore + 1UZ) / 2UZ})) << "the source's rate tag alone carries a rate, and the restart publishes none";
        expect(fatal(gt((1990UZ + delayBefore + 1UZ) / 2UZ, before.size()))) << "the tag on input 1990 is held across the restart";
        expect(that % (run.offsetsOf("probe") == std::vector<std::size_t>{before.size()})) << "a tag held across the restart leaves on the restart's first output";
    };

    "a value outside the limits on a tag is refused and the run goes on"_test = [] {
        constexpr gr::Size_t kSamples = 3000U;
        const GraphRun       run      = runGraph({}, kSamples, {{1000UZ, {{"transition", 0.5f}}}, {2000UZ, {{"transition", std::numeric_limits<float>::quiet_NaN()}}}});
        expect(fatal(run.ran)) << "the run completes";
        expect(eq(*run.active.at("transition").get_if<float>(), kDefaultTransition));
        expect(run.samples == byKernel(kDefaultTransition, counted(kSamples))) << "the block computes on the accepted transition throughout";
    };

    "a transition at or below 0, or at or above 0.5, is refused"_test = [] {
        Block      block   = makeBlock();
        const auto refused = [&block](float value) {
            std::ignore = block.settings().setStaged({{"transition", value}});
            return !block.settings().applyStagedParameters().failedParameters.empty();
        };
        for (const float value : {0.f, -0.1f, 0.5f, 0.6f, std::numeric_limits<float>::quiet_NaN()}) {
            expect(refused(value)) << std::format("a transition of {} is refused", value);
            expect(eq(block.transition.value, kDefaultTransition)) << "the block keeps the accepted transition";
        }

        const auto        x        = noise(1001UZ, 4U);
        const std::size_t chunks[] = {100UZ};
        Run               run;
        feed(block, x, chunks, 4096UZ, run);
        expect(run.samples == byKernel(kDefaultTransition, x)) << "and the block computes on the accepted transition";

        expect(!refused(0.49f)) << "a transition just below 0.5 is accepted";
        expect(eq(block.transition.value, 0.49f));
    };

    "a staged change restarts the conversion at the next call, and a waiting rate keeps the value it arrived with"_test = [] {
        Block                      block    = makeBlock();
        const auto                 x        = noise(3000UZ, 5U);
        const std::size_t          chunks[] = {101UZ};
        Run                        run;
        const std::vector<gr::Tag> waiting{{1100UZ, {{"probe", gr::Size_t(8)}, {gr::tag::SAMPLE_RATE.key(), 1.f}}}};
        feed(block, std::span(x).first(1111UZ), chunks, 4096UZ, run, waiting);
        const std::size_t before = run.samples.size();
        expect(fatal(lt(before, (1100UZ + makeKernel(kDefaultTransition).groupDelay() + 1UZ) / 2UZ))) << "the tag on input 1100 still waits for its output";
        std::ignore = block.settings().setStaged({{"transition", 0.05f}});
        expect(fatal(block.settings().applyStagedParameters().failedParameters.empty()));
        const std::vector<gr::Tag> held{{1511UZ, probe(9UZ)}};
        feed(block, x, chunks, 4096UZ, run, held);

        const auto after = byKernel(0.05f, std::span(x).subspan(1111UZ));
        expect(fatal(eq(run.samples.size(), before + after.size())));
        expect(std::equal(after.begin(), after.end(), run.samples.begin() + static_cast<std::ptrdiff_t>(before))) << "the rebuilt conversion starts at the first sample of the next call";
        expect(that % (run.offsetsOf(gr::tag::SAMPLE_RATE.key()) == std::vector<std::size_t>{before})) << "the waiting tag alone carries a rate, and the rebuild publishes none";
        for (const gr::Tag& published : run.tags) {
            if (published.map.contains(gr::property_map::key_type{gr::tag::SAMPLE_RATE.key()})) {
                expect(eq(rateOf(published), 0.5f * 1.f)) << "a tag waiting across the change leaves at half the rate it arrived with";
            }
        }
        // The restart lies on input 1111, an odd one. The tag on input 1511 sits 400 inputs after it. With an odd delay a
        // count from the stream's start puts that tag one output early.
        const std::size_t delayAfter = makeKernel(0.05f).groupDelay();
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
        const std::size_t delayBefore = makeKernel(kDefaultTransition).groupDelay();
        expect(fatal(gt((1000UZ + delayBefore + 1UZ) / 2UZ, before))) << "the tag on input 1000 waits across the restart";
        std::ignore = block.settings().setStaged({{"transition", 0.04f}});
        expect(fatal(block.settings().applyStagedParameters().failedParameters.empty()));
        const std::size_t rest[] = {2990UZ};
        feed(block, x, rest, 4096UZ, run, tags);

        const auto after = byKernel(0.04f, std::span(x).subspan(1010UZ));
        expect(fatal(eq(run.samples.size(), before + after.size())));
        expect(std::equal(after.begin(), after.end(), run.samples.begin() + static_cast<std::ptrdiff_t>(before))) << "the rebuilt conversion starts at input 1010";
        const std::size_t              delayAfter = makeKernel(0.04f).groupDelay();
        const std::vector<std::size_t> wanted{before, before + (2UZ + delayAfter + 1UZ) / 2UZ};
        expect(that % (run.offsetsOf("probe") == wanted)) << "the held tag on the restart's first output, the tag on input 1012 by the new delay";
    };

    "a forwarded gr:sample_rate leaves at half the value it arrived with, a float as a float"_test = [] {
        constexpr float kArrived = 96'000.f;
        const GraphRun  run      = runGraph({}, 4000U, {{1000UZ, {{"probe", gr::Size_t(0)}, {gr::tag::SAMPLE_RATE.key(), kArrived}}}}, false);
        expect(fatal(run.ran));
        expect(fatal(eq(run.offsetsOf("probe").size(), 1UZ)));
        for (const gr::Tag& published : run.tags) {
            if (published.map.contains(gr::property_map::key_type{"probe"})) {
                const auto& rate = published.map.at(gr::property_map::key_type{gr::tag::SAMPLE_RATE.key()});
                expect(rate.get_if<float>() != nullptr && *rate.get_if<float>() == 0.5f * kArrived) << std::format("the tag arrived at {} Hz and leaves at {}", kArrived, rateOf(published));
            }
        }
    };

    "a forwarded gr:sample_rate leaves at half the value it arrived with, a double as a double"_test = [] {
        constexpr double kArrived = 96'000.0;
        const GraphRun   run      = runGraph({}, 4000U, {{1000UZ, {{"probe", gr::Size_t(0)}, {gr::tag::SAMPLE_RATE.key(), kArrived}}}}, false);
        expect(fatal(run.ran));
        expect(fatal(eq(run.offsetsOf("probe").size(), 1UZ)));
        for (const gr::Tag& published : run.tags) {
            if (published.map.contains(gr::property_map::key_type{"probe"})) {
                const auto&   rate = published.map.at(gr::property_map::key_type{gr::tag::SAMPLE_RATE.key()});
                const double* wide = rate.get_if<double>();
                expect(wide != nullptr && *wide == 0.5 * kArrived) << std::format("the tag arrived at {} Hz and leaves at {}", kArrived, wide != nullptr ? *wide : 0.0);
            }
        }
    };

    "a rate tag on the input's end-of-stream index leaves halved on the output's end-of-stream index"_test = [] {
        constexpr std::size_t  kSamples = 4000UZ;
        constexpr float        kArrived = 96'000.f;
        const gr::property_map late{{std::pmr::string(std::to_string(kSamples)), gr::property_map{{gr::tag::SAMPLE_RATE.key(), kArrived}}}};

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<gr::blocks::basic::VectorSource<float>>({{"values", std::vector<float>(kSamples, 0.f)}, {"tags", late}});
        auto&     block  = graph.emplaceBlock<Block>();
        auto&     sink   = graph.emplaceBlock<EndSink<CF>>();
        expect(fatal(graph.connect<"out", "in">(source, block).has_value() && graph.connect<"out", "in">(block, sink).has_value()));
        gr::scheduler::Simple scheduler;
        expect(fatal(scheduler.exchange(std::move(graph)).has_value() && scheduler.runAndWait().has_value())) << "the run completes";
        expect(fatal(eq(sink._samples.size(), kSamples / 2UZ)));

        std::vector<std::pair<std::size_t, float>> rates;
        for (const gr::Tag& published : sink._tags) {
            if (published.map.contains(gr::property_map::key_type{gr::tag::SAMPLE_RATE.key()})) {
                rates.emplace_back(published.index, rateOf(published));
            }
        }
        const std::vector<std::pair<std::size_t, float>> wanted{{sink._samples.size(), 0.5f * kArrived}};
        expect(rates == wanted) << std::format("the tag past the last input leaves at the end-of-stream index at half its value: {}", rates);
    };

    "reset() returns the conversion to the start of a stream"_test = [] {
        Block             block    = makeBlock({{"transition", 0.04f}});
        const auto        x        = noise(2001UZ, 6U);
        const std::size_t chunks[] = {333UZ};
        Run               warm;
        feed(block, noise(777UZ, 7U), chunks, 4096UZ, warm);
        block.reset();
        Run run;
        feed(block, x, chunks, 4096UZ, run);
        expect(run.samples == byKernel(0.04f, x)) << "the outputs of a fresh conversion, bit for bit";
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

        const auto after = byKernel(kDefaultTransition, std::span(x).subspan(1010UZ));
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
        auto&      source = graph.emplaceBlock<gr::blocks::testing::TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 4000U}, {"values", values}, {"sample_rate", kSourceRate}});
        auto&      block  = graph.emplaceBlock<Block>({{"transition", 0.04f}});
        auto&      sink   = graph.emplaceBlock<EndSink<CF>>();
        expect(fatal(graph.connect<"out", "in">(source, block).has_value() && graph.connect<"out", "in">(block, sink).has_value()));
        gr::scheduler::Simple scheduler;
        expect(fatal(scheduler.exchange(std::move(graph)).has_value() && scheduler.runAndWait().has_value())) << "the run completes";

        const auto reference = byKernel(0.04f, x);
        expect(eq(sink._samples.size(), reference.size())) << "every output is made";
        const std::size_t n      = makeKernel(0.04f).taps().size();
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
