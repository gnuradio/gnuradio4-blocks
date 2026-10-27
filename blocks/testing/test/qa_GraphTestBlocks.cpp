#include <boost/ut.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/basic/VectorSource.hpp>
#include <gnuradio-4.0/testing/GraphTestBlocks.hpp>

namespace {

/// @brief Emits 1, 2, ..., `n` in one call and publishes one tag carrying `key` at sample index `at`.
/// An `at` equal to `n` places the tag at the end-of-stream index. An empty `key` publishes no tag.
template<typename T>
struct TagAtSource : gr::Block<TagAtSource<T>> {
    gr::PortOut<T> out;
    gr::Size_t     n  = 4U;
    gr::Size_t     at = 4U;
    std::string    key{"mark"};

    GR_MAKE_REFLECTABLE(TagAtSource, out, n, at, key);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t count = std::min(outSpan.size(), static_cast<std::size_t>(n));
        for (std::size_t i = 0UZ; i < count; ++i) {
            outSpan[i] = static_cast<T>(i + 1UZ);
        }
        if (!key.empty()) {
            gr::property_map map;
            map.insert_or_assign(std::pmr::string(key.begin(), key.end()), gr::pmt::Value(std::string("x")));
            outSpan.publishTag(map, static_cast<std::size_t>(at));
        }
        outSpan.publish(count);
        return gr::work::Status::DONE;
    }
};

/// @brief Runs a source into an ExpectSink and returns the error message, empty when the run succeeds.
template<typename T, template<typename> typename TSource = TagAtSource>
std::string runIntoSink(gr::property_map sourceSettings, gr::property_map sinkSettings) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TSource<T>>(std::move(sourceSettings));
    auto&     sink   = graph.emplaceBlock<gr::blocks::testing::ExpectSink<T>>(std::move(sinkSettings));
    if (!graph.connect<"out", "in">(source, sink).has_value()) {
        return "connect failed";
    }
    gr::scheduler::Simple scheduler;
    if (auto exchanged = scheduler.exchange(std::move(graph)); !exchanged) {
        return "exchange failed";
    }
    const auto result = scheduler.runAndWait();
    return result.has_value() ? std::string{} : std::string(result.error().message);
}

} // namespace

const boost::ut::suite<"ExpectSink"> expectSinkTests = [] {
    using namespace boost::ut;
    using gr::property_map;
    const std::vector<float> oneToFour{1.f, 2.f, 3.f, 4.f};

    "a tag at the end-of-stream index counts at the sample count"_test = [&] {
        const std::string error = runIntoSink<float>(property_map{{"n", gr::Size_t(4)}, {"at", gr::Size_t(4)}}, //
            property_map{{"expected", oneToFour}, {"tag_key", std::string("mark")}, {"tag_indices", std::vector<gr::Size_t>{4U}}});
        expect(error.empty()) << error;
    };

    "an end-of-stream tag expected elsewhere fails with both index lists"_test = [&] {
        const std::string error = runIntoSink<float>(property_map{{"n", gr::Size_t(4)}, {"at", gr::Size_t(4)}}, //
            property_map{{"expected", oneToFour}, {"tag_key", std::string("mark")}, {"tag_indices", std::vector<gr::Size_t>{3U}}});
        expect(error.contains("tags with key 'mark' at [4], expected at [3]")) << error;
    };

    "tag_key matches a tag key only as spelled"_test = [&] {
        const property_map prefixed{{"n", gr::Size_t(4)}, {"at", gr::Size_t(4)}, {"key", std::string("gr:mark")}};
        const std::string  spelled = runIntoSink<float>(prefixed, property_map{{"expected", oneToFour}, {"tag_key", std::string("gr:mark")}, {"tag_indices", std::vector<gr::Size_t>{4U}}});
        expect(spelled.empty()) << spelled;
        const std::string unprefixed = runIntoSink<float>(prefixed, property_map{{"expected", oneToFour}, {"tag_key", std::string("mark")}, {"tag_indices", std::vector<gr::Size_t>{4U}}});
        expect(unprefixed.contains("tags with key 'mark' at [], expected at [4]")) << unprefixed;
    };

    "a repeated index in tag_indices counts once"_test = [&] {
        const std::string error = runIntoSink<float>(property_map{{"n", gr::Size_t(4)}, {"at", gr::Size_t(1)}}, //
            property_map{{"expected", oneToFour}, {"tag_key", std::string("mark")}, {"tag_indices", std::vector<gr::Size_t>{1U, 1U}}});
        expect(error.empty()) << error;
    };

    "uint32 samples are compared"_test = [] {
        const std::string match = runIntoSink<std::uint32_t>(property_map{{"n", gr::Size_t(4)}, {"key", std::string()}}, //
            property_map{{"expected", std::vector<std::uint32_t>{1U, 2U, 3U, 4U}}});
        expect(match.empty()) << match;
        const std::string mismatch = runIntoSink<std::uint32_t>(property_map{{"n", gr::Size_t(4)}, {"key", std::string()}}, //
            property_map{{"expected", std::vector<std::uint32_t>{1U, 2U, 9U, 4U}}});
        expect(mismatch.contains("sample 2 is 3, expected 9")) << mismatch;
    };
};

const boost::ut::suite<"ExpectSink expected_tags"> expectedTagsTests = [] {
    using namespace boost::ut;
    using gr::property_map;
    using gr::blocks::basic::VectorSource;
    const std::vector<float> oneToFour{1.f, 2.f, 3.f, 4.f};
    const property_map       frameTag{{"mark", std::string("frame")}, {"level", std::int64_t(3)}};
    const property_map       source{{"values", oneToFour}, {"tags", property_map{{"1", frameTag}, {"3", frameTag}}}};

    "two tags match exactly and the end-of-stream tag passes tags_exact"_test = [&] {
        const std::string error = runIntoSink<float, VectorSource>(source, //
            property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"1", frameTag}, {"3", frameTag}}}, {"tags_exact", true}});
        expect(error.empty()) << error;
    };

    "a listed end-of-stream tag matches at the sample count"_test = [&] {
        const std::string error = runIntoSink<float>(property_map{{"n", gr::Size_t(4)}, {"key", std::string()}}, //
            property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"4", property_map{{gr::tag::END_OF_STREAM.key(), true}}}}}, {"tags_exact", true}});
        expect(error.empty()) << error;
    };

    "a listed key matches a tag key only as spelled"_test = [&] {
        const property_map prefixed{{"n", gr::Size_t(4)}, {"at", gr::Size_t(2)}, {"key", std::string("gr:mark")}};
        const std::string  spelled = runIntoSink<float>(prefixed, property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"2", property_map{{"gr:mark", std::string("x")}}}}}, {"tags_exact", true}});
        expect(spelled.empty()) << spelled;
        const std::string unprefixed = runIntoSink<float>(prefixed, property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"2", property_map{{"mark", std::string("x")}}}}}, {"tags_exact", true}});
        expect(unprefixed.contains("tag 'gr:mark' at sample 2 is ") && unprefixed.contains("expected no such tag")) << unprefixed;
    };

    "a wrong value fails with the index, the key and both values"_test = [&] {
        const property_map wrongLevel{{"mark", std::string("frame")}, {"level", std::int64_t(4)}};
        const std::string  error = runIntoSink<float, VectorSource>(source, //
             property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"1", wrongLevel}, {"3", frameTag}}}});
        expect(error.contains("tag 'level' at sample 1 is 3, expected 4")) << error;
    };

    "a numeric value is compared in the type of the expected value"_test = [&] {
        const auto runWithLevel = [&](gr::pmt::Value level, gr::pmt::Value expectedLevel) {
            return runIntoSink<float, VectorSource>(property_map{{"values", oneToFour}, {"tags", property_map{{"1", property_map{{"level", level}}}}}}, //
                property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"1", property_map{{"level", expectedLevel}}}}}});
        };
        const std::string widened = runWithLevel(gr::pmt::Value(gr::Size_t(3)), gr::pmt::Value(std::int64_t(3)));
        expect(widened.empty()) << widened;
        const std::string inexact = runWithLevel(gr::pmt::Value(0.1f), gr::pmt::Value(0.1));
        expect(inexact.contains("tag 'level' at sample 1 is 0.1 (float32), expected 0.1 (float64)")) << inexact;
        const std::string text = runWithLevel(gr::pmt::Value(std::string("3")), gr::pmt::Value(std::int64_t(3)));
        expect(text.contains("(string), expected 3 (int64)")) << text;
        const std::string integerAsDouble = runWithLevel(gr::pmt::Value(std::int64_t(3)), gr::pmt::Value(3.0));
        expect(integerAsDouble.empty()) << integerAsDouble;
        const std::string doubleAsInteger = runWithLevel(gr::pmt::Value(3.0), gr::pmt::Value(std::int64_t(3)));
        expect(doubleAsInteger.empty()) << doubleAsInteger;
        const std::string singleAsInteger = runWithLevel(gr::pmt::Value(3.0f), gr::pmt::Value(std::int64_t(3)));
        expect(singleAsInteger.empty()) << singleAsInteger;
        const std::string singleAsDouble = runWithLevel(gr::pmt::Value(3.0f), gr::pmt::Value(3.0));
        expect(singleAsDouble.empty()) << singleAsDouble;
        const std::string lossy = runWithLevel(gr::pmt::Value(0.1000000001), gr::pmt::Value(0.1f));
        expect(lossy.contains("tag 'level' at sample 1 is 0.1000000001 (float64), expected 0.1 (float32)")) << lossy;
    };

    "a missing tag fails when the stream ends"_test = [&] {
        const std::string error = runIntoSink<float, VectorSource>(source, //
            property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"1", frameTag}, {"2", property_map{{"mark", std::string("frame")}}}}}});
        expect(error.contains("tag 'mark' at sample 2 is missing, expected")) << error;
    };

    "tags_exact refuses a key not listed at its index"_test = [&] {
        const std::string error = runIntoSink<float, VectorSource>(source, //
            property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"1", frameTag}, {"3", property_map{{"mark", std::string("frame")}}}}}, {"tags_exact", true}});
        expect(error.contains("tag 'level' at sample 3 is 3, expected no such tag")) << error;
    };

    "tags_exact refuses a tag at an index the map does not list"_test = [&] {
        const std::string error = runIntoSink<float, VectorSource>(source, //
            property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"1", frameTag}}}, {"tags_exact", true}});
        expect(error.contains(" at sample 3 is ") && error.contains("expected no such tag")) << error;
    };

    "two spellings of one index listing the same key fail"_test = [&] {
        const std::string error = runIntoSink<float, VectorSource>(source, //
            property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"1", frameTag}, {"01", property_map{{"mark", std::string("frame")}}}}}});
        expect(error.contains("expected_tags keys '") && error.contains("both list tag 'mark' at sample 1") && error.contains("'01'")) << error;
    };

    "unlisted tags pass without tags_exact"_test = [&] {
        const std::string error = runIntoSink<float, VectorSource>(source, //
            property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"1", property_map{{"mark", std::string("frame")}}}}}});
        expect(error.empty()) << error;
    };

    "an expected_tags key that is not a sample index fails"_test = [&] {
        const std::string error = runIntoSink<float, VectorSource>(source, //
            property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"one", frameTag}}}});
        expect(error.contains("expected_tags key 'one' is not a sample index")) << error;
    };
};

int main() { /* not needed for UT */ }
