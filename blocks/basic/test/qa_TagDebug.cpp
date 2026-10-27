#include <boost/ut.hpp>

#include <cstddef>
#include <format>
#include <iostream>
#include <sstream>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>

#include <gnuradio-4.0/basic/TagDebug.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace {

using gr::blocks::basic::TagDebug;
using gr::blocks::testing::ProcessFunction;
using gr::blocks::testing::TagSink;
using gr::blocks::testing::TagSource;

constexpr gr::Size_t  kSamples   = 1000U;
constexpr float       kRate      = 1000.f;
constexpr std::size_t kMaxChunk  = 64UZ;
constexpr std::size_t kMidStream = 600UZ;
constexpr std::size_t kMinChunk  = 40UZ; // above kMidStream % kMaxChunk and at most kSamples % kMaxChunk

/// Sends everything written to std::cout into a string until the object is destroyed.
struct CoutCapture {
    std::ostringstream text;
    std::streambuf*    previous;

    CoutCapture() : previous(std::cout.rdbuf(text.rdbuf())) {}
    ~CoutCapture() { std::cout.rdbuf(previous); }
    CoutCapture(const CoutCapture&)            = delete;
    CoutCapture& operator=(const CoutCapture&) = delete;
};

struct Run {
    std::vector<float>   samples{};
    std::vector<gr::Tag> tags{};
    gr::Size_t           nTags             = 0U;
    gr::Size_t           nTagsFromSettings = 0U;
    std::string          printed{};
};

[[nodiscard]] std::vector<gr::Tag> twoTags() {
    return {gr::Tag{0UZ, gr::property_map{{"first", gr::pmt::Value(std::string("start"))}}}, //
        gr::Tag{kMidStream, gr::property_map{{"second", gr::pmt::Value(std::int32_t{42})}}}};
}

/// The tags TagDebug sees: the source's `gr:sample_rate` tag on its first sample, then @p tags.
[[nodiscard]] std::vector<gr::Tag> seenTags(const std::vector<gr::Tag>& tags) {
    std::vector<gr::Tag> seen{gr::Tag{0UZ, gr::property_map{{gr::tag::SAMPLE_RATE.key(), gr::pmt::Value(kRate)}}}};
    seen.insert(seen.end(), tags.begin(), tags.end());
    return seen;
}

/// Runs a TagSource of the ramp 0, 1, ... with @p tags through a TagDebug into a TagSink. The input port of the
/// TagDebug takes at most kMaxChunk samples per call. A tag past kMaxChunk reaches a later work call. A @p minChunk
/// above 0 sets the least number of samples the input port takes per call.
[[nodiscard]] Run runThrough(const std::vector<gr::Tag>& tags, gr::property_map debugSettings, std::size_t minChunk = 0UZ) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", kSamples}, {"sample_rate", kRate}, {"mark_tag", false}});
    source._tags     = tags;
    debugSettings.insert_or_assign("name", gr::pmt::Value(std::string("debug")));
    auto& debug          = graph.emplaceBlock<TagDebug<float>>(std::move(debugSettings));
    debug.in.max_samples = kMaxChunk;
    if (minChunk > 0UZ) {
        debug.in.min_samples = minChunk;
    }
    auto& sink = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "sink"}});

    boost::ut::expect(graph.connect<"out", "in">(source, debug).has_value());
    boost::ut::expect(graph.connect<"out", "in">(debug, sink).has_value());

    Run                   run;
    gr::scheduler::Simple scheduler; // owns the blocks once it holds the graph
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    {
        CoutCapture capture;
        boost::ut::expect(scheduler.runAndWait().has_value());
        run.printed = capture.text.str();
    }
    run.samples.assign(sink._samples.begin(), sink._samples.end());
    run.tags  = sink._tags;
    run.nTags = debug.n_tags.value;
    if (const auto value = debug.settings().get("n_tags"); value.has_value()) {
        run.nTagsFromSettings = value->value_or<gr::Size_t>(0U);
    }
    return run;
}

[[nodiscard]] std::string lineFor(const gr::Tag& tag) { return std::format("debug tag at {}: {}\n", tag.index, tag.map); }

[[nodiscard]] std::string linesFor(const std::vector<gr::Tag>& tags) {
    std::string lines;
    for (const gr::Tag& tag : tags) {
        lines += lineFor(tag);
    }
    return lines;
}

[[nodiscard]] bool holds(const std::vector<gr::Tag>& received, const gr::Tag& wanted) {
    for (const gr::Tag& tag : received) {
        if (tag.index != wanted.index) {
            continue;
        }
        bool all = true;
        for (const auto& [key, value] : wanted.map) {
            const auto found = tag.map.find(key);
            all              = all && found != tag.map.end() && found->second == value;
        }
        if (all) {
            return true;
        }
    }
    return false;
}

} // namespace

const boost::ut::suite<"TagDebug"> tagDebugTests = [] {
    using namespace boost::ut;

    "samples and tags pass unchanged and each tag prints once with its stream index"_test = [] {
        const std::vector<gr::Tag> tags = twoTags();
        const Run                  run  = runThrough(tags, {});

        std::vector<float> ramp(kSamples);
        for (std::size_t i = 0UZ; i < ramp.size(); ++i) {
            ramp[i] = static_cast<float>(i);
        }
        expect(run.samples == ramp) << "the samples arrive unchanged";
        expect(holds(run.tags, tags[0])) << "the tag at 0 is forwarded at 0";
        expect(holds(run.tags, tags[1])) << std::format("the tag at {} is forwarded at {}", kMidStream, kMidStream);
        const std::vector<gr::Tag> seen = seenTags(tags);
        expect(eq(run.nTags, static_cast<gr::Size_t>(seen.size())));
        expect(eq(run.nTagsFromSettings, static_cast<gr::Size_t>(seen.size()))) << "a settings get reads the counter";
        expect(eq(run.printed, linesFor(seen))) << run.printed;
    };

    "a tag inside a work call prints with its stream index"_test = [] {
        // The call that reaches kMidStream starts kMidStream % kMaxChunk samples before it. That count is below
        // kMinChunk. The call cannot end at the tag, and the tag sits inside it at an offset above 0.
        const std::vector<gr::Tag> tags = twoTags();
        const Run                  run  = runThrough(tags, {}, kMinChunk);

        expect(eq(run.samples.size(), static_cast<std::size_t>(kSamples))) << "every sample arrives";
        expect(holds(run.tags, tags[1])) << std::format("the tag at {} is forwarded at {}", kMidStream, kMidStream);
        const std::vector<gr::Tag> seen = seenTags(tags);
        expect(eq(run.nTags, static_cast<gr::Size_t>(seen.size())));
        expect(eq(run.printed, linesFor(seen))) << run.printed;
    };

    "key_filter counts and prints only the tags holding the key"_test = [] {
        const std::vector<gr::Tag> tags = twoTags();
        const Run                  run  = runThrough(tags, {{"key_filter", gr::pmt::Value(std::string("second"))}});

        expect(eq(run.nTags, gr::Size_t{1U}));
        expect(eq(run.nTagsFromSettings, gr::Size_t{1U}));
        expect(eq(run.printed, lineFor(tags[1]))) << run.printed;
        expect(holds(run.tags, tags[0])) << "a filtered tag is still forwarded";
    };

    "display false counts the tags and prints nothing"_test = [] {
        const std::vector<gr::Tag> tags = twoTags();
        const Run                  run  = runThrough(tags, {{"display", false}});

        const std::vector<gr::Tag> seen = seenTags(tags);
        expect(eq(run.nTags, static_cast<gr::Size_t>(seen.size())));
        expect(eq(run.nTagsFromSettings, static_cast<gr::Size_t>(seen.size())));
        expect(run.printed.empty()) << run.printed;
        expect(holds(run.tags, tags[0]) && holds(run.tags, tags[1])) << "the tags are still forwarded";
    };
};

int main() { /* not needed for UT */ }
