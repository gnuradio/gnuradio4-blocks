#include <boost/ut.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <future>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <memory_resource>
#include <span>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/LifeCycle.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/basic/NullSources.hpp>
#include <gnuradio-4.0/basic/VectorSource.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace gr::blocks::basic::vector_source_test {

/// The number of samples ChunkSink takes per call.
constexpr std::size_t kChunk = 3072UZ;

/// Takes exactly `kChunk` samples per call and keeps each tag that carries the key "mark" once. `Policy` sets which
/// tags of a chunk the input retires with its samples.
template<typename... Policy>
struct ChunkSink : gr::Block<ChunkSink<Policy...>, Policy...> {
    gr::PortIn<float, gr::RequiredSamples<kChunk, kChunk>> in;

    GR_MAKE_REFLECTABLE(ChunkSink, in);

    std::vector<gr::Tag> _tags;
    std::uint64_t        _nSamples = 0U;

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& input) {
        // A tag the input kept from an earlier chunk sits before the first sample of this one.
        for (const gr::Tag& tag : input.rawTags) {
            if (tag.index >= input.streamIndex && tag.map.contains("mark")) {
                _tags.push_back(tag);
            }
        }
        _nSamples += input.size();
        std::ignore = input.consume(input.size());
        return gr::work::Status::OK;
    }
};

} // namespace gr::blocks::basic::vector_source_test

namespace {

using gr::property_map;
using gr::blocks::basic::VectorSource;
using gr::blocks::basic::vector_source_test::ChunkSink;
using gr::blocks::basic::vector_source_test::kChunk;

/// Applies the settings `source` was built with and moves it to RUNNING, as a scheduler does before the first call.
void start(VectorSource<float>& source) {
    source.init(source.progress);
    boost::ut::expect(boost::ut::fatal(source.changeStateTo(gr::lifecycle::State::RUNNING).has_value()));
}

/// Runs one call of `chunk` samples from the position of `source` and returns the tags it published.
std::vector<gr::Tag> publishedInCall(VectorSource<float>& source, std::size_t chunk) {
    std::vector<gr::Tag>                         published;
    std::vector<float>                           buffer(chunk, 0.f);
    gr::blocks::testing::span::OutputSpan<float> outSpan{std::span<float>(buffer), gr::blocks::basic::detail::boundedSize(source._emitted), &published};
    std::ignore = source.processBulk(outSpan);
    return published;
}

/// Runs `source` from its position through calls of `chunk` samples each until it ends the stream and returns the tags
/// it published.
std::vector<gr::Tag> publishedTags(VectorSource<float>& source, std::size_t chunk) {
    std::vector<gr::Tag> published;
    std::vector<float>   buffer(chunk, 0.f);
    std::size_t          at = gr::blocks::basic::detail::boundedSize(source._emitted);
    for (gr::work::Status status = gr::work::Status::OK; status == gr::work::Status::OK;) {
        gr::blocks::testing::span::OutputSpan<float> outSpan{std::span<float>(buffer), at, &published};
        status = source.processBulk(outSpan);
        at += outSpan.count;
    }
    return published;
}

/// Returns the sample indices of `published`, in order.
std::vector<std::size_t> indicesOf(const std::vector<gr::Tag>& published) {
    std::vector<std::size_t> indices;
    for (const gr::Tag& tag : published) {
        indices.push_back(tag.index);
    }
    return indices;
}

/// Returns the `tags` key of sample `index`.
std::pmr::string indexKey(std::uint64_t index) { return std::pmr::string(std::to_string(index)); }

gr::pmt::Value listOf(std::initializer_list<property_map> maps) {
    std::pmr::vector<gr::pmt::Value> values;
    for (const property_map& map : maps) {
        values.emplace_back(map);
    }
    return gr::pmt::Value(gr::Tensor<gr::pmt::Value>(std::move(values)));
}

/// Runs a source built with `settings` into a NullSink over an edge built with `edge` and returns the error message,
/// empty when the run succeeds.
std::string runError(const property_map& settings, gr::EdgeParameters edge = {}) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<VectorSource<float>>(settings);
    auto&     sink   = graph.emplaceBlock<gr::blocks::basic::NullSink<float>>();
    if (!graph.connect<"out", "in">(source, sink, std::move(edge)).has_value()) {
        return "connect failed";
    }
    gr::scheduler::Simple<> scheduler;
    if (auto exchanged = scheduler.exchange(std::move(graph)); !exchanged) {
        return exchanged.error().message;
    }
    if (auto result = scheduler.runAndWait(); !result) {
        return result.error().message;
    }
    return {};
}

/// The time a run of a few thousand samples is given to end by itself. It ends in milliseconds.
constexpr std::chrono::seconds kRunBound{5};

/// Runs `scheduler` and requests a stop once `kRunBound` has passed. Returns whether the run ended by itself, and sets
/// `error` to the error of the run.
bool runWithin(gr::scheduler::Simple<>& scheduler, std::string& error) {
    std::promise<std::string> ended;
    std::future<std::string>  message = ended.get_future();
    std::thread               runner([&scheduler, &ended] {
        const auto result = scheduler.runAndWait();
        ended.set_value(result.has_value() ? std::string{} : result.error().message);
    });
    const bool                byItself = message.wait_for(kRunBound) == std::future_status::ready;
    if (!byItself) {
        scheduler.requestStop();
    }
    runner.join();
    error = message.get();
    return byItself;
}

/// What a run of a source delivers to a ChunkSink: the tags that carry the key "mark", the sample count and the error.
/// It also holds whether the run ended within `kRunBound` and the counts of the source's output: the tags it dropped
/// and the end-of-stream tags it could not publish.
struct Delivery {
    std::vector<gr::Tag> tags;
    std::uint64_t        samples = 0U;
    std::string          error;
    bool                 ended           = false;
    std::size_t          dropped         = 0UZ;
    std::size_t          endOfStreamLost = 0UZ;
};

/// Runs a source built with `settings` into a `Sink` over an edge of `bufferSize` samples, for at most `kRunBound`.
/// The default sink passes Block's default resampling explicitly: gr::meta::type_name() reads past the end of an empty
/// template argument list such as `ChunkSink<>`.
template<typename Sink = ChunkSink<gr::Resampling<1U, 1U, true>>>
Delivery deliver(const property_map& settings, std::size_t bufferSize) {
    Delivery  delivery;
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<VectorSource<float>>(settings);
    auto&     sink   = graph.emplaceBlock<Sink>();
    if (!graph.connect<"out", "in">(source, sink, {.minBufferSize = bufferSize}).has_value()) {
        delivery.error = "connect failed";
        return delivery;
    }
    gr::scheduler::Simple<> scheduler;
    if (auto exchanged = scheduler.exchange(std::move(graph)); !exchanged) {
        delivery.error = exchanged.error().message;
        return delivery;
    }
    delivery.ended           = runWithin(scheduler, delivery.error);
    delivery.tags            = sink._tags;
    delivery.samples         = sink._nSamples;
    delivery.dropped         = source.out.nTagsDropped;
    delivery.endOfStreamLost = source.out.nEoSTagsLost;
    return delivery;
}

/// Reads `n` samples at `in` with their tags and appends the tags that carry the key "mark" to `marks`.
void take(gr::PortIn<float>& in, std::size_t n, std::vector<gr::Tag>& marks) {
    auto input = in.get<gr::SpanReleasePolicy::ProcessNone>(n);
    for (const gr::Tag& tag : input.rawTags) {
        if (tag.map.contains("mark")) {
            marks.push_back(tag);
        }
    }
    std::ignore = input.consume(n);
}

/// Reads every sample that waits at `in` and appends the tags that carry the key "mark" to `marks`.
void drain(gr::PortIn<float>& in, std::vector<gr::Tag>& marks) { take(in, in.streamReader().available(), marks); }

/// Returns whether a tag that waits at `in` after its samples carries the end-of-stream key at sample `index`.
bool endOfStreamAt(gr::PortIn<float>& in, std::size_t index) {
    const auto rest = in.getRemainingTags<gr::SpanReleasePolicy::ProcessNone>();
    return std::ranges::any_of(rest.rawTags, [index](const gr::Tag& tag) { return tag.index == index && tag.map.contains(static_cast<std::pmr::string>(gr::tag::END_OF_STREAM)); });
}

/// Returns the "tags" setting with `perIndex` maps at each index from `first` up to `last`, the maps marked with
/// `firstMark` and on in index order.
property_map denseTags(std::size_t first, std::size_t last, std::size_t perIndex, std::uint64_t firstMark) {
    property_map  tags;
    std::uint64_t mark = firstMark;
    for (std::size_t index = first; index < last; ++index) {
        std::pmr::vector<gr::pmt::Value> maps;
        for (std::size_t i = 0UZ; i < perIndex; ++i) {
            maps.emplace_back(property_map{{"mark", mark++}});
        }
        tags.insert_or_assign(indexKey(index), gr::pmt::Value(gr::Tensor<gr::pmt::Value>(std::move(maps))));
    }
    return tags;
}

/// Returns the sample index and mark of each map of `denseTags(first, last, perIndex, firstMark)`, in order.
std::vector<std::pair<std::size_t, std::uint64_t>> denseMarks(std::size_t first, std::size_t last, std::size_t perIndex, std::uint64_t firstMark) {
    std::vector<std::pair<std::size_t, std::uint64_t>> marks;
    std::uint64_t                                      mark = firstMark;
    for (std::size_t index = first; index < last; ++index) {
        for (std::size_t i = 0UZ; i < perIndex; ++i) {
            marks.emplace_back(index, mark++);
        }
    }
    return marks;
}

/// Returns the sample index and the "mark" value of each tag, in order.
std::vector<std::pair<std::size_t, std::uint64_t>> marksOf(const std::vector<gr::Tag>& tags) {
    std::vector<std::pair<std::size_t, std::uint64_t>> marks;
    for (const gr::Tag& tag : tags) {
        const auto* mark = tag.map.at("mark").get_if<std::uint64_t>();
        marks.emplace_back(tag.index, mark != nullptr ? *mark : std::uint64_t{0});
    }
    return marks;
}

} // namespace

const boost::ut::suite<"VectorSource"> vectorSourceTests = [] {
    using namespace boost::ut;

    "values shorter than the emitted count end the stream with nothing emitted"_test = [] {
        VectorSource<float> source;
        source.values   = std::vector<float>{1.f, 2.f, 3.f};
        source._emitted = 5U;
        std::vector<float>                           buffer(4UZ, 0.f);
        gr::blocks::testing::span::OutputSpan<float> outSpan{std::span<float>(buffer)};
        const gr::work::Status                       status = source.processBulk(outSpan);
        expect(status == gr::work::Status::DONE);
        expect(eq(outSpan.count, 0UZ));
        expect(eq(source._emitted, std::uint64_t{5}));
    };

    "each tags entry is published at the sample index its key names"_test = [] {
        const std::vector<float> values{1.f, 2.f, 3.f, 4.f, 5.f, 6.f};
        const property_map       first{{"a", 1}};
        const property_map       second{{"b", "x"}};
        VectorSource<float>      source({{"values", values}, {"tags", property_map{{"1", first}, {"4", second}}}});
        start(source);
        const auto published = publishedTags(source, 4UZ);
        expect(fatal(eq(published.size(), 2UZ)));
        expect(eq(published[0].index, 1UZ));
        expect(published[0].map == first);
        expect(eq(published[1].index, 4UZ)) << "the second tag falls in the second call";
        expect(published[1].map == second);
    };

    "a list at one index publishes one tag per map in list order"_test = [] {
        const std::vector<float> values{1.f, 2.f, 3.f, 4.f};
        const property_map       level{{"level", 3}};
        const property_map       gate{{"gate", "open"}};
        const property_map       last{{"last", true}};
        VectorSource<float>      source({{"values", values}, {"tags", property_map{{"1", listOf({level, gate})}, {"2", listOf({last})}}}});
        start(source);
        const auto published = publishedTags(source, 8UZ);
        expect(fatal(eq(published.size(), 3UZ)));
        expect(indicesOf(published) == std::vector<std::size_t>{1UZ, 1UZ, 2UZ});
        expect(published[0].map == level);
        expect(published[1].map == gate) << "the maps of one index stay separate tags";
        expect(published[2].map == last) << "a list of one map publishes that map";
    };

    "an entry at the sample count is published at the end-of-stream index"_test = [] {
        const std::vector<float> values{1.f, 2.f, 3.f};
        const std::uint64_t      passes = 2U;
        const property_map       last{{"end", true}};
        VectorSource<float>      source({{"values", values}, {"repeat", passes}, {"tags", property_map{{indexKey(values.size() * passes), last}}}});
        start(source);
        const auto published = publishedTags(source, 4UZ);
        expect(fatal(eq(published.size(), 1UZ)));
        expect(eq(published[0].index, values.size() * passes));
        expect(published[0].map == last);
    };

    "an entry past the sample count fails the run at the start with an error that names the index"_test = [] {
        const std::vector<float> values{1.f, 2.f, 3.f};
        const std::string        error = runError({{"values", values}, {"tags", property_map{{"0", property_map{{"a", 1}}}, {indexKey(values.size() + 1UZ), property_map{{"late", true}}}}}});
        expect(error.contains("tags entry at sample 4 is past the 3 samples")) << error;
    };

    "a tags key that is not a decimal index fails the run with an error that names the key"_test = [] {
        const std::string error = runError({{"values", std::vector<float>{1.f, 2.f}}, {"tags", property_map{{"1x", property_map{{"a", 1}}}}}});
        expect(error.contains("tags key '1x' is not a sample index")) << error;
    };

    "tags count across passes and are published once unless repeat_tags is set"_test = [] {
        const std::vector<float> values{1.f, 2.f, 3.f};
        const property_map       mark{{"mark", 1}};
        VectorSource<float>      source({{"values", values}, {"repeat", std::uint64_t{3}}, {"tags", property_map{{"0", mark}, {"4", mark}}}});
        start(source);
        expect(!source.repeat_tags.value) << "the default places each tag once";
        expect(indicesOf(publishedTags(source, 4UZ)) == std::vector<std::size_t>{0UZ, 4UZ});
    };

    "with repeat_tags the tags of one pass are published again on every pass"_test = [] {
        const std::vector<float> values{1.f, 2.f, 3.f};
        const std::size_t        passes = 3UZ;
        const property_map       first{{"first", true}};
        const property_map       third{{"third", true}};
        VectorSource<float>      source({{"values", values}, {"repeat", std::uint64_t{passes}}, {"repeat_tags", true}, {"tags", property_map{{"0", first}, {"2", third}}}});
        start(source);
        const auto               published = publishedTags(source, 4UZ);
        std::vector<std::size_t> wanted;
        for (std::size_t pass = 0UZ; pass < passes; ++pass) {
            wanted.push_back(pass * values.size());
            wanted.push_back(pass * values.size() + 2UZ);
        }
        expect(indicesOf(published) == wanted) << "the calls of four samples cut the passes at different points";
        expect(fatal(eq(published.size(), wanted.size())));
        expect(published[4].map == first);
        expect(published[5].map == third);
    };

    "with repeat_tags an index at the size of values fails the run at the start"_test = [] {
        const std::vector<float> values{1.f, 2.f, 3.f};
        const std::string        error = runError({{"values", values}, {"repeat", std::uint64_t{2}}, {"repeat_tags", true}, {"tags", property_map{{indexKey(values.size()), property_map{{"a", 1}}}}}});
        expect(error.contains("tags entry at sample 3 is past the 3 values of one pass")) << error;
    };

    "a repeat past 2^32 samples counts the stream in 64 bits"_test = [] {
        const std::vector<float> values{1.f, 2.f, 3.f};
        const std::uint64_t      passes = std::uint64_t{1} << 32U;
        const std::uint64_t      total  = values.size() * passes;
        const property_map       mark{{"mark", 1}};
        VectorSource<float>      source({{"values", values}, {"repeat", passes}, {"tags", property_map{{indexKey(total - 1U), mark}, {indexKey(total), mark}}}});
        start(source);
        expect(fatal(eq(source.repeat.value, passes)));
        const std::uint64_t remaining = 4U;
        source._emitted               = total - remaining;
        std::vector<gr::Tag>                         published;
        std::vector<float>                           buffer(8UZ, 0.f);
        gr::blocks::testing::span::OutputSpan<float> outSpan{std::span<float>(buffer), 0UZ, &published};
        expect(source.processBulk(outSpan) == gr::work::Status::DONE);
        expect(fatal(eq(outSpan.count, 4UZ)));
        std::vector<float> wanted;
        for (std::uint64_t index = total - remaining; index < total; ++index) {
            wanted.push_back(values[gr::blocks::basic::detail::boundedSize(index % values.size())]);
        }
        expect(std::vector<float>(buffer.begin(), buffer.begin() + 4) == wanted) << "each sample is the value its 64-bit index selects";
        expect(eq(source._emitted, total));
        expect(indicesOf(published) == std::vector<std::size_t>{3UZ, 4UZ}) << "the last sample and the end-of-stream index";
    };

    "a refused tags value on a running source leaves the tags and the schedule as they were"_test = [] {
        const std::vector<float> values{1.f, 2.f, 3.f, 4.f};
        const property_map       kept{{"a", 1}};
        const property_map       tagsBefore{{"1", kept}};
        VectorSource<float>      source({{"values", values}, {"tags", tagsBefore}});
        start(source);
        const property_map notStaged = source.settings().setStaged(property_map{{"tags", property_map{{"2x", property_map{{"b", 2}}}}}});
        expect(notStaged.empty()) << "the value reaches the settings, and the block refuses it when it applies";
        source.applyChangedSettings();
        expect(source.tags.value == tagsBefore) << "the member keeps the old value";
        const property_map settings = source.settings().get();
        const auto*        read     = settings.at("tags").get_if<property_map>();
        expect(fatal(read != nullptr));
        expect(*read == tagsBefore) << "a settings read agrees with the member";
        const auto published = publishedTags(source, 8UZ);
        expect(fatal(eq(published.size(), 1UZ)));
        expect(eq(published[0].index, 1UZ));
        expect(published[0].map == kept);
    };

    "a batch with a refused tags value leaves the other settings of the batch as they were"_test = [] {
        const std::vector<float> values{1.f, 2.f, 3.f, 4.f};
        const property_map       kept{{"a", 1}};
        VectorSource<float>      source({{"values", values}, {"tags", property_map{{"1", kept}}}});
        start(source);
        const property_map notStaged = source.settings().setStaged(property_map{{"values", std::vector<float>{9.f}}, {"tags", property_map{{"2x", property_map{{"b", 2}}}}}});
        expect(notStaged.empty());
        source.applyChangedSettings();
        expect(source.values.value == values);
        const auto published = publishedTags(source, 8UZ);
        expect(fatal(eq(published.size(), 1UZ)));
        expect(eq(published[0].index, 1UZ));
        expect(published[0].map == kept);
    };

    "an accepted tags change on a running source places its tags at their sample indices from the next call"_test = [] {
        const std::vector<float> values(8UZ, 1.f);
        const property_map       first{{"first", true}};
        const property_map       behind{{"behind", true}};
        const property_map       ahead{{"ahead", true}};
        VectorSource<float>      source({{"values", values}, {"tags", property_map{{"0", first}}}});
        start(source);
        const std::size_t emitted = values.size() / 2UZ;
        expect(indicesOf(publishedInCall(source, emitted)) == std::vector<std::size_t>{0UZ});
        expect(fatal(eq(source._emitted, std::uint64_t{emitted})));
        const property_map changed{{indexKey(emitted - 2UZ), behind}, {indexKey(emitted + 2UZ), ahead}};
        expect(source.settings().setStaged(property_map{{"tags", changed}}).empty());
        source.applyChangedSettings();
        expect(source.tags.value == changed);
        const auto published = publishedTags(source, values.size());
        expect(fatal(eq(published.size(), 1UZ))) << "an index below the samples already emitted is not published";
        expect(eq(published[0].index, emitted + 2UZ)) << "the index counts from the first sample";
        expect(published[0].map == ahead);
    };

    "a change on a running source that leaves an index past the sample count is refused and the run goes on"_test = [] {
        const std::vector<float> values{1.f, 2.f, 3.f, 4.f};
        const property_map       kept{{"kept", true}};
        const property_map       tagsBefore{{indexKey(values.size() - 1UZ), kept}};
        VectorSource<float>      source({{"values", values}, {"tags", tagsBefore}});
        start(source);
        expect(publishedInCall(source, 2UZ).empty());
        expect(source.settings().setStaged(property_map{{"tags", property_map{{indexKey(values.size() + 1UZ), property_map{{"late", true}}}}}}).empty());
        source.applyChangedSettings();
        expect(source.tags.value == tagsBefore) << "a tags index past the count";
        expect(source.settings().setStaged(property_map{{"values", std::vector<float>(values.size() - 2UZ, 9.f)}}).empty());
        source.applyChangedSettings();
        expect(source.values.value == values) << "a shorter values that leaves the tags index past the count";
        const auto published = publishedTags(source, values.size());
        expect(!source._failed) << "the run ends without an error";
        expect(eq(source._emitted, std::uint64_t{values.size()}));
        expect(fatal(eq(published.size(), 1UZ)));
        expect(eq(published[0].index, values.size() - 1UZ));
        expect(published[0].map == kept);
    };

    "a tags setting with more maps in a run of the output buffer than its tag buffer holds fails the run at the start"_test = [] {
        // The edge holds kBuffer samples and kBuffer + 1 tags. The output keeps one tag slot for the end-of-stream tag.
        constexpr std::size_t kBuffer  = 4096UZ;
        const std::size_t     nSamples = 4UZ * kChunk;
        const std::size_t     dense    = kChunk + kChunk / 2UZ;
        expect(fatal(gt(2UZ * std::min(kBuffer, nSamples - dense), kBuffer))) << "two maps per sample from dense outnumber the tag slots";
        const Delivery delivery = deliver({{"values", std::vector<float>(nSamples, 1.f)}, {"tags", denseTags(dense, nSamples, 2UZ, 0U)}}, kBuffer);
        expect(delivery.error.contains(std::format("tags entry at sample {} starts a run of {} samples", dense, kBuffer))) << delivery.error;
        expect(delivery.tags.empty());
    };

    "the maps at the end-of-stream index count with the last sample of a run"_test = [] {
        constexpr std::size_t kBuffer  = 4096UZ;
        const std::size_t     nSamples = 2UZ * kBuffer;
        const std::size_t     first    = nSamples - kBuffer;
        property_map          tags     = denseTags(first, first + 1UZ, kBuffer / 2UZ, 0U);
        for (const auto& [key, entry] : denseTags(nSamples, nSamples + 1UZ, kBuffer / 2UZ + 1UZ, kBuffer / 2UZ)) {
            tags.insert_or_assign(key, entry);
        }
        const std::string error = runError({{"values", std::vector<float>(nSamples, 1.f)}, {"tags", tags}}, {.minBufferSize = kBuffer});
        expect(error.contains(std::format("tags entry at sample {} starts a run", first))) << "the run from the first index holds kBuffer + 1 maps: " << error;
    };

    "a tags setting with as many maps in a run as the tag slots of the output runs to the end with every map at its index"_test = [] {
        constexpr std::size_t kBuffer  = 4096UZ;
        const std::size_t     nSamples = 4UZ * kChunk;
        const std::size_t     dense    = kChunk + kChunk / 2UZ;
        const std::size_t     last     = dense + kBuffer / 2UZ;
        const Delivery        delivery = deliver({{"values", std::vector<float>(nSamples, 1.f)}, {"tags", denseTags(dense, last, 2UZ, 0U)}}, kBuffer);
        expect(delivery.error.empty()) << delivery.error;
        expect(delivery.ended);
        expect(eq(delivery.samples, std::uint64_t{nSamples})) << "a sink that takes kChunk samples per call receives every sample";
        const std::vector<std::pair<std::size_t, std::uint64_t>> wanted = denseMarks(dense, last, 2UZ, 0U);
        expect(marksOf(delivery.tags) == wanted) << "kBuffer maps in kBuffer / 2 samples";
        expect(eq(delivery.dropped, 0UZ)) << "the sink takes the tags of each chunk with its samples";
    };

    "a reader that holds tags past the samples it consumed loses the tags that do not fit and the run ends"_test = [] {
        // One map per sample passes the start check at the limit. The sink retires only the tags at the first sample of
        // each chunk and holds the others until its next call.
        constexpr std::size_t kBuffer  = 4096UZ;
        const std::size_t     nSamples = 4UZ * kChunk;
        const Delivery        delivery = deliver<ChunkSink<gr::FilteredTagPropagation>>({{"values", std::vector<float>(nSamples, 1.f)}, {"tags", denseTags(0UZ, nSamples, 1UZ, 0U)}}, kBuffer);
        expect(delivery.error.empty()) << delivery.error;
        expect(delivery.ended) << "the run ends by itself, which needs the end-of-stream tag at the sink";
        expect(eq(delivery.endOfStreamLost, 0UZ));
        expect(eq(delivery.samples, std::uint64_t{nSamples}));
        expect(gt(delivery.dropped, 0UZ)) << "the held tags leave too few free slots";
        expect(eq(delivery.tags.size() + delivery.dropped, nSamples)) << "each map arrives or is counted as dropped";
        const std::vector<std::pair<std::size_t, std::uint64_t>> marks = marksOf(delivery.tags);
        expect(std::ranges::all_of(marks, [](const auto& mark) { return mark.first == mark.second; })) << "each map that arrives sits at its index";
        expect(std::ranges::is_sorted(marks)) << "in index order";
    };

    "a running change to a tags setting with more maps in a run than the tag buffer holds is refused and the run goes on"_test = [] {
        const std::size_t   nSamples = 2UZ * gr::PortOut<float>::kDefaultBufferSize;
        const property_map  tagsBefore{{"1", property_map{{"mark", std::uint64_t{1}}}}, {indexKey(nSamples - 1UZ), property_map{{"mark", std::uint64_t{2}}}}};
        VectorSource<float> source({{"values", std::vector<float>(nSamples, 1.f)}, {"tags", tagsBefore}});
        gr::PortIn<float>   downstream;
        expect(fatal(source.out.connect(downstream).has_value()));
        start(source);
        const std::size_t slots = source.out.tagWriter().buffer().size() - 1UZ;
        expect(fatal(gt(2UZ * std::min(source.out.bufferSize(), nSamples), slots))) << "two maps per sample outnumber the tag slots of a run";
        std::vector<gr::Tag> marks;
        std::ignore = source.work(std::numeric_limits<std::size_t>::max());
        drain(downstream, marks);
        expect(fatal(lt(source._emitted, std::uint64_t{nSamples})));
        expect(source.settings().setStaged(property_map{{"tags", denseTags(0UZ, nSamples, 2UZ, 0U)}}).empty());
        source.applyChangedSettings();
        expect(source.tags.value == tagsBefore) << "the member keeps the old value";
        for (std::size_t call = 0UZ; call < nSamples && source._emitted < nSamples; ++call) {
            std::ignore = source.work(std::numeric_limits<std::size_t>::max());
            drain(downstream, marks);
        }
        expect(eq(source._emitted, std::uint64_t{nSamples}));
        const std::vector<std::pair<std::size_t, std::uint64_t>> wanted{{1UZ, 1U}, {nSamples - 1UZ, 2U}};
        expect(marksOf(marks) == wanted) << "the tags of the old setting at their indices";
    };

    "a running change that crowds the tags of an earlier setting in the output loses the tags that do not fit and the run ends"_test = [] {
        // The reader takes kChunk samples at a time with their tags. Two maps at each index from kBuffer / 2 fill the tag
        // slots in the first call. The reader leaves the maps past kChunk unread, and the change crowds the next call.
        constexpr std::size_t kBuffer  = gr::PortOut<float>::kDefaultBufferSize;
        const std::size_t     nSamples = 3UZ * kBuffer;
        const std::size_t     half     = kBuffer / 2UZ;
        VectorSource<float>   source({{"values", std::vector<float>(nSamples, 1.f)}, {"tags", denseTags(half, kBuffer, 2UZ, 0U)}});
        gr::PortIn<float>     downstream;
        expect(fatal(source.out.connect(downstream).has_value()));
        start(source);
        const std::size_t slots = source.out.tagWriter().buffer().size() - 1UZ;
        expect(fatal(eq(source.out.bufferSize(), kBuffer)));
        expect(fatal(eq(2UZ * half, slots))) << "the first setting fills the tag slots";
        expect(fatal(half < kChunk && kChunk < kBuffer)) << "the reader leaves maps of the first setting unread";
        std::vector<gr::Tag> marks;
        std::ignore = source.work(std::numeric_limits<std::size_t>::max());
        expect(fatal(eq(source._emitted, std::uint64_t{kBuffer})));
        take(downstream, kChunk, marks);
        const std::size_t  held    = 2UZ * (kBuffer - kChunk);
        const property_map changed = denseTags(kBuffer, kBuffer + half, 2UZ, 2UZ * half);
        expect(source.settings().setStaged(property_map{{"tags", changed}}).empty());
        source.applyChangedSettings();
        expect(fatal(source.tags.value == changed)) << "the change alone fits the slots";
        std::size_t taken = kChunk;
        for (std::size_t call = 0UZ; call < nSamples && taken < nSamples; ++call) {
            std::ignore = source.work(std::numeric_limits<std::size_t>::max());
            for (; downstream.streamReader().available() >= kChunk; taken += kChunk) {
                take(downstream, kChunk, marks);
            }
        }
        expect(eq(source._emitted, std::uint64_t{nSamples}));
        expect(eq(taken, nSamples)) << "the reader takes every sample in chunks of kChunk";
        expect(endOfStreamAt(downstream, nSamples));
        expect(eq(source.out.nEoSTagsLost, 0UZ));
        // The next call reaches every map of the change. The held maps leave room for slots - held of them.
        const std::size_t excess = 2UZ * half - (slots - held);
        expect(eq(source.out.nTagsDropped, excess));
        std::vector<std::pair<std::size_t, std::uint64_t>> wanted = denseMarks(half, kBuffer, 2UZ, 0U);
        std::ranges::copy(denseMarks(kBuffer, kBuffer + half, 2UZ, 2UZ * half), std::back_inserter(wanted));
        wanted.resize(wanted.size() - excess);
        expect(marksOf(marks) == wanted) << "the maps past the free slots are the ones dropped";
    };

    "a call that ends the stream while tags of an earlier tags setting wait in the output emits every sample and the end-of-stream tag"_test = [] {
        // After the first sample, two maps at each later index fill the tag slots less the end-of-stream slot.
        const std::size_t   nSamples = gr::PortOut<float>::kDefaultBufferSize / 2UZ + 1UZ;
        VectorSource<float> source({{"values", std::vector<float>(nSamples, 1.f)}, {"tags", property_map{{"0", property_map{{"mark", std::uint64_t{0}}}}}}});
        gr::PortIn<float>   downstream;
        expect(fatal(source.out.connect(downstream).has_value()));
        start(source);
        const std::size_t slots = source.out.tagWriter().buffer().size() - 1UZ;
        expect(fatal(eq(slots, 2UZ * (nSamples - 1UZ))));
        {
            auto outSpan = source.out.tryReserve<gr::SpanReleasePolicy::ProcessNone>(1UZ);
            std::ignore  = source.processBulk(outSpan);
        }
        expect(fatal(eq(source._emitted, std::uint64_t{1})));
        const property_map changed = denseTags(1UZ, nSamples, 2UZ, 1U);
        expect(source.settings().setStaged(property_map{{"tags", changed}}).empty());
        source.applyChangedSettings();
        expect(fatal(source.tags.value == changed));
        const gr::work::Result result = source.work(std::numeric_limits<std::size_t>::max());
        expect(result.status == gr::work::Status::DONE) << "the call emits every sample left";
        expect(eq(source._emitted, std::uint64_t{nSamples}));
        std::vector<gr::Tag> marks;
        drain(downstream, marks);
        expect(endOfStreamAt(downstream, nSamples)) << "the end-of-stream tag takes the slot kept for it";
        expect(eq(source.out.nEoSTagsLost, 0UZ));
        // The unread map of the first sample takes one of the slots the change fills.
        const std::size_t excess = 1UZ + 2UZ * (nSamples - 1UZ) - slots;
        expect(eq(source.out.nTagsDropped, excess));
        std::vector<std::pair<std::size_t, std::uint64_t>> wanted{{0UZ, 0U}};
        std::ranges::copy(denseMarks(1UZ, nSamples, 2UZ, 1U), std::back_inserter(wanted));
        wanted.resize(wanted.size() - excess);
        expect(marksOf(marks) == wanted) << "every map at its index but the ones past the free slots";
    };
};

int main() { /* not needed for UT */ }
