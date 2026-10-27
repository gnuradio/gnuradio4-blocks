#include <boost/ut.hpp>

#include <algorithm>
#include <cstddef>
#include <format>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>

#include <gnuradio-4.0/basic/DropTagKeys.hpp>
#include <gnuradio-4.0/testing/GraphTestBlocks.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace {

using gr::blocks::basic::DropTagKeys;
using gr::blocks::testing::ProcessFunction;
using gr::blocks::testing::TagSink;
using gr::blocks::testing::TagSource;

constexpr gr::Size_t  kSamples = 64U;
constexpr const char* kDropped = "test:dropped";
constexpr const char* kKept    = "test:kept";

struct Run {
    std::vector<float>   samples{};
    std::vector<gr::Tag> tags{};
};

/// @brief Run a ramp of kSamples with @p tags through a DropTagKeys built with @p settings into a TagSink.
///
/// @p prepare receives the block before the run. A sink with `USE_PROCESS_BULK` records every map it receives, an empty
/// one included.
template<ProcessFunction kSinkFunction = ProcessFunction::USE_PROCESS_ONE, typename FPrepare>
[[nodiscard]] Run throughBlock(std::vector<gr::Tag> tags, const gr::property_map& settings, FPrepare&& prepare) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", kSamples}, {"mark_tag", false}});
    source._tags     = std::move(tags);
    auto& block      = graph.emplaceBlock<DropTagKeys<float>>(settings);
    prepare(block);
    auto& sink = graph.emplaceBlock<TagSink<float, kSinkFunction>>();

    boost::ut::expect(graph.connect<"out", "in">(source, block).has_value()) << boost::ut::fatal;
    boost::ut::expect(graph.connect<"out", "in">(block, sink).has_value()) << boost::ut::fatal;

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value()) << boost::ut::fatal;
    boost::ut::expect(scheduler.runAndWait().has_value()) << "the run ends without an error";
    return {std::vector<float>(sink._samples.begin(), sink._samples.end()), sink._tags};
}

template<ProcessFunction kSinkFunction = ProcessFunction::USE_PROCESS_ONE>
[[nodiscard]] Run throughBlock(std::vector<gr::Tag> tags, const gr::property_map& settings) {
    return throughBlock<kSinkFunction>(std::move(tags), settings, [](DropTagKeys<float>&) {});
}

[[nodiscard]] gr::property_map dropping(std::vector<std::string> keys) { return {{"keys", std::move(keys)}}; }

/// @brief A preparation that holds the block's input to chunks of exactly @p chunk samples.
[[nodiscard]] auto inChunksOf(std::size_t chunk) {
    return [chunk](DropTagKeys<float>& block) {
        block.in.min_samples = chunk;
        block.in.max_samples = chunk;
    };
}

/// @brief The value of @p key on the tag at @p index, if a tag there carries it.
[[nodiscard]] std::optional<int> valueAt(const std::vector<gr::Tag>& tags, std::size_t index, const std::string& key) {
    for (const gr::Tag& tag : tags) {
        if (tag.index != index) {
            continue;
        }
        if (const auto it = tag.map.find(gr::property_map::key_type(key)); it != tag.map.end()) {
            return it->second.value_or<int>(-1);
        }
    }
    return std::nullopt;
}

/// @brief The number of tags that carry @p key.
[[nodiscard]] std::size_t countOf(const std::vector<gr::Tag>& tags, const std::string& key) {
    return static_cast<std::size_t>(std::ranges::count_if(tags, [&key](const gr::Tag& tag) { return tag.map.contains(gr::property_map::key_type(key)); }));
}

void expectRamp(const std::vector<float>& samples) {
    using namespace boost::ut;
    expect(eq(samples.size(), static_cast<std::size_t>(kSamples))) << "every sample leaves";
    for (std::size_t i = 0UZ; i < samples.size(); ++i) {
        expect(eq(samples[i], static_cast<float>(i))) << std::format("sample {} leaves as it arrived", i);
    }
}

} // namespace

const boost::ut::suite<"DropTagKeys"> dropTagKeysTests = [] {
    using namespace boost::ut;

    "a listed key does not cross, and the tag it leaves empty is not published"_test = [] {
        constexpr std::size_t kTagged = 10UZ;
        const Run             run     = throughBlock<ProcessFunction::USE_PROCESS_BULK>({{kTagged, {{kDropped, 1}}}}, dropping({kDropped}));
        expectRamp(run.samples);
        expect(eq(countOf(run.tags, kDropped), 0UZ)) << "the listed key is removed";
        expect(std::ranges::none_of(run.tags, [](const gr::Tag& t) { return t.index == kTagged; })) << "no tag leaves on the sample whose only key was removed";
    };

    "a key the list does not name crosses at its sample with its value"_test = [] {
        const Run run = throughBlock({{10UZ, {{kKept, 2}}}}, dropping({kDropped}));
        expectRamp(run.samples);
        expect(eq(valueAt(run.tags, 10UZ, kKept).value_or(0), 2)) << "the unlisted key leaves on sample 10";
        expect(eq(countOf(run.tags, kKept), 1UZ)) << "the unlisted key leaves once";
    };

    "a tag carrying both keys leaves with the unlisted one alone"_test = [] {
        const Run run = throughBlock({{10UZ, {{kDropped, 1}, {kKept, 2}}}}, dropping({kDropped}));
        expectRamp(run.samples);
        expect(eq(valueAt(run.tags, 10UZ, kKept).value_or(0), 2)) << "the unlisted key stays on the tag";
        expect(eq(countOf(run.tags, kDropped), 0UZ)) << "the listed key leaves the tag";
    };

    "a gr: key matches only when the list names it with its domain"_test = [] {
        const std::string rate{gr::tag::SAMPLE_RATE.key()};
        const Run         bare   = throughBlock({}, dropping({std::string(gr::tag::SAMPLE_RATE.name())}));
        const Run         listed = throughBlock({}, dropping({rate}));
        expect(eq(countOf(bare.tags, rate), 1UZ)) << "the bare name leaves the source's rate tag alone";
        expect(eq(countOf(listed.tags, rate), 0UZ)) << "the full key removes the source's rate tag";
    };

    // An input minimum keeps the scheduler from ending a chunk at a tag. The tags of these cases arrive inside a chunk.
    "a tag inside a whole chunk leaves at its own sample, once"_test = [] {
        constexpr std::size_t kChunk  = 16UZ;
        constexpr std::size_t kTagged = 10UZ;
        static_assert(kTagged > 0UZ && kTagged < kChunk);
        const Run run = throughBlock({{kTagged, {{kKept, 2}}}}, dropping({}), inChunksOf(kChunk));
        expectRamp(run.samples);
        expect(eq(valueAt(run.tags, kTagged, kKept).value_or(0), 2)) << "the tag leaves on the sample it arrived on";
        expect(eq(countOf(run.tags, kKept), 1UZ)) << "the tag leaves once";
    };

    "tags inside a later whole chunk leave at their own samples without the listed key"_test = [] {
        constexpr std::size_t kChunk     = 16UZ;
        constexpr std::size_t kChunkHead = 2UZ * kChunk;
        constexpr std::size_t kKeptAt    = kChunkHead + 5UZ;
        constexpr std::size_t kEmptiedAt = kChunkHead + 9UZ;
        constexpr std::size_t kBothAt    = kChunkHead + 12UZ;
        static_assert(kBothAt < kChunkHead + kChunk && kChunkHead + kChunk < kSamples);
        const Run run = throughBlock({{kKeptAt, {{kKept, 3}}}, {kEmptiedAt, {{kDropped, 1}}}, {kBothAt, {{kDropped, 5}, {kKept, 4}}}}, dropping({kDropped}), inChunksOf(kChunk));
        expectRamp(run.samples);
        expect(eq(valueAt(run.tags, kKeptAt, kKept).value_or(0), 3)) << "the first tag leaves on the sample it arrived on";
        expect(eq(valueAt(run.tags, kBothAt, kKept).value_or(0), 4)) << "the tag carrying both keys leaves on the sample it arrived on";
        expect(eq(countOf(run.tags, kKept), 2UZ)) << "each kept tag leaves once";
        expect(eq(countOf(run.tags, kDropped), 0UZ)) << "the listed key is removed inside the chunk";
        expect(std::ranges::none_of(run.tags, [](const gr::Tag& t) { return t.index == kEmptiedAt; })) << "no tag leaves on the sample whose only key was removed";
    };

    "a list a tag switches to inside a whole chunk applies from the next chunk on"_test = [] {
        constexpr std::size_t kChunk     = 16UZ;
        constexpr std::size_t kSwitchAt  = 10UZ;
        constexpr std::size_t kNextAt    = kChunk + 4UZ;
        constexpr std::size_t kEmptiedAt = 2UZ * kChunk + 8UZ;
        static_assert(kSwitchAt < kChunk && kEmptiedAt + kChunk < kSamples);
        const std::string context{"drop"};
        const std::string contextKey{gr::tag::CONTEXT.key()};
        const auto        prepare = [&context](DropTagKeys<float>& block) {
            inChunksOf(kChunk)(block);
            expect(block.settings().set(dropping({kDropped}), gr::SettingsCtx{.context = context}).empty()) << "the block stores the list for the context";
        };
        const Run run = throughBlock({{kSwitchAt, {{gr::tag::CONTEXT.key(), context}}}, {kNextAt, {{kDropped, 1}, {kKept, 2}}}, {kEmptiedAt, {{kDropped, 3}}}}, dropping({}), prepare);
        expectRamp(run.samples);
        expect(eq(countOf(run.tags, contextKey), 1UZ)) << "the tag that switches the list leaves once";
        expect(std::ranges::any_of(run.tags, [&contextKey](const gr::Tag& t) { return t.index == kSwitchAt && t.map.contains(gr::property_map::key_type(contextKey)); })) << "the tag that switches the list leaves on its own sample";
        expect(eq(countOf(run.tags, kDropped), 0UZ)) << "the tags of the later chunks lose the listed key";
        expect(eq(valueAt(run.tags, kNextAt, kKept).value_or(0), 2)) << "the next chunk's tag keeps its unlisted key";
        expect(eq(countOf(run.tags, kKept), 1UZ)) << "the unlisted key leaves once";
        expect(std::ranges::none_of(run.tags, [](const gr::Tag& t) { return t.index == kEmptiedAt; })) << "no tag leaves on the sample whose only key the list removed";
    };

    // A sink records no tag past its last sample, so ExpectSink, which reads the tags at the end-of-stream index, checks
    // this case.
    "a tag past the last whole chunk leaves at the end of the stream without the listed key"_test = [] {
        constexpr std::size_t kChunk  = 40UZ;
        constexpr std::size_t kTagged = kChunk + 10UZ;
        constexpr float       kRate   = 1000.0f;

        gr::Graph graph;
        auto&     source     = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", kSamples}, {"mark_tag", false}, {"sample_rate", kRate}});
        source._tags         = {{kTagged, {{kDropped, 1}, {kKept, 2}}}};
        auto& block          = graph.emplaceBlock<DropTagKeys<float>>(dropping({kDropped}));
        block.in.min_samples = kChunk;
        block.in.max_samples = kChunk;
        std::vector<float> firstChunk(kChunk);
        std::ranges::generate(firstChunk, [i = 0.0f]() mutable { return i++; });
        const std::string      endIndex = std::to_string(kChunk);
        const gr::property_map expectedTags{{"0", gr::property_map{{gr::tag::SAMPLE_RATE.key(), kRate}}}, {gr::property_map::key_type(std::string_view(endIndex)), gr::property_map{{kKept, 2}}}};
        auto&                  sink = graph.emplaceBlock<gr::blocks::testing::ExpectSink<float>>({{"expected", firstChunk}, {"expected_tags", expectedTags}, {"tags_exact", true}});
        expect(graph.connect<"out", "in">(source, block).has_value()) << fatal;
        expect(graph.connect<"out", "in">(block, sink).has_value()) << fatal;

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value()) << fatal;
        const auto result = scheduler.runAndWait();
        expect(result.has_value()) << (result.has_value() ? std::string() : std::string(result.error().message));
    };

    "a list changed while the stream runs applies to the tags after the change"_test = [] {
        constexpr std::size_t kSwitch = 30UZ;
        const std::string     context{"drop"};
        const Run             run = throughBlock({{10UZ, {{kDropped, 1}}}, {kSwitch, {{gr::tag::CONTEXT.key(), context}}}, {50UZ, {{kDropped, 3}}}}, dropping({}), [&context](DropTagKeys<float>& block) { //
            expect(block.settings().set(dropping({kDropped}), gr::SettingsCtx{.context = context}).empty()) << "the block stores the list for the context";
        });
        expectRamp(run.samples);
        expect(eq(valueAt(run.tags, 10UZ, kDropped).value_or(0), 1)) << "before the change, the key crosses";
        expect(!valueAt(run.tags, 50UZ, kDropped).has_value()) << "after the change, the key is removed";
        expect(eq(countOf(run.tags, kDropped), 1UZ)) << "only the tag before the change carries the key";
    };
};

int main() { /* not needed for UT */ }
