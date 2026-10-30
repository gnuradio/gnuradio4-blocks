#include <boost/ut.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <memory_resource>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/basic/DataSetSource.hpp>
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

/// @brief Runs a source into a sink and returns the error message, empty when the run succeeds.
template<typename TSource, typename TSink>
std::string runChain(gr::property_map sourceSettings, gr::property_map sinkSettings) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TSource>(std::move(sourceSettings));
    auto&     sink   = graph.emplaceBlock<TSink>(std::move(sinkSettings));
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

/// @brief Runs a source into an ExpectSink and returns the error message, empty when the run succeeds.
template<typename T, template<typename> typename TSource = TagAtSource>
std::string runIntoSink(gr::property_map sourceSettings, gr::property_map sinkSettings) {
    return runChain<TSource<T>, gr::blocks::testing::ExpectSink<T>>(std::move(sourceSettings), std::move(sinkSettings));
}

/// @brief Returns a list of maps, the form of several tags at one index or of several meta maps.
gr::pmt::Value listOf(std::initializer_list<gr::property_map> maps) {
    std::pmr::vector<gr::pmt::Value> values;
    for (const gr::property_map& map : maps) {
        values.emplace_back(map);
    }
    return gr::pmt::Value(gr::Tensor<gr::pmt::Value>(std::move(values)));
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

    "two spellings of one index fail whatever their entries hold"_test = [&] {
        const std::string sameKey = runIntoSink<float, VectorSource>(source, //
            property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"1", frameTag}, {"01", property_map{{"mark", std::string("frame")}}}}}});
        expect(sameKey.contains("expected_tags keys '01' and '1' both name sample 1")) << sameKey;
        const std::string otherKeys = runIntoSink<float, VectorSource>(source, //
            property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"1", property_map{{"mark", std::string("frame")}}}, {"01", property_map{{"level", std::int64_t(3)}}}}}});
        expect(otherKeys.contains("expected_tags keys '01' and '1' both name sample 1")) << otherKeys;
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

    "a list tells two tags at one index from one tag with both keys"_test = [&] {
        const property_map level{{"level", std::int64_t(3)}};
        const property_map gate{{"gate", std::string("open")}};
        const property_map joined{{"level", std::int64_t(3)}, {"gate", std::string("open")}};
        const auto         run = [&](gr::pmt::Value published, gr::pmt::Value wanted, bool exact = true) {
            return runIntoSink<float, VectorSource>(property_map{{"values", oneToFour}, {"tags", property_map{{"1", published}}}}, //
                        property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"1", wanted}}}, {"tags_exact", exact}});
        };
        const std::string split = run(listOf({level, gate}), listOf({level, gate}));
        expect(split.empty()) << split;
        const std::string together = run(gr::pmt::Value(joined), listOf({joined}));
        expect(together.empty()) << together;
        const std::string keysOnly = run(listOf({level, gate}), gr::pmt::Value(joined));
        expect(keysOnly.empty()) << keysOnly;
        const std::string tooFew = run(gr::pmt::Value(joined), listOf({level, gate}), false);
        expect(tooFew.contains("sample 1 carries 1 tags, expected 2")) << tooFew;
        const std::string spread = run(listOf({level, gate}), listOf({joined}));
        expect(spread.contains("tag 0 at sample 1 lacks key 'gate', expected \"open\"")) << spread;
        const std::string swapped = run(listOf({gate, level}), listOf({level, gate}));
        expect(swapped.contains("tag 0 at sample 1 lacks key 'level', expected 3")) << swapped;
        const std::string unlisted = run(listOf({joined, gate}), listOf({level, gate}));
        expect(unlisted.contains("tag 0 at sample 1 key 'gate' is \"open\", expected no such key")) << unlisted;
    };

    "a list counts the tags at the end-of-stream index"_test = [&] {
        const property_map mark{{"mark", std::string("end")}};
        const std::string  error = runIntoSink<float, VectorSource>(property_map{{"values", oneToFour}, {"tags", property_map{{"4", listOf({mark, mark})}}}}, //
             property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"4", listOf({mark, mark})}}}, {"tags_exact", true}});
        expect(error.empty()) << error;
        const std::string beyond = runIntoSink<float, VectorSource>(property_map{{"values", oneToFour}, {"tags", property_map{{"4", listOf({mark, mark})}}}}, //
            property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"4", listOf({mark})}}}});
        expect(beyond.contains("tag 1 at sample 4 is ") && beyond.contains("beyond the 1 tags expected there")) << beyond;
    };

    "a list matches a tag key only as spelled"_test = [&] {
        const property_map prefixed{{"gr:mark", std::string("x")}};
        const property_map bare{{"mark", std::string("x")}};
        const property_map both{{"gr:mark", std::string("x")}, {"mark", std::string("x")}};
        const auto         run = [&](const property_map& published, const property_map& wanted) {
            return runIntoSink<float, VectorSource>(property_map{{"values", oneToFour}, {"tags", property_map{{"1", listOf({published})}}}}, //
                        property_map{{"expected", oneToFour}, {"expected_tags", property_map{{"1", listOf({wanted})}}}, {"tags_exact", true}});
        };
        const std::string spelled = run(prefixed, prefixed);
        expect(spelled.empty()) << spelled;
        const std::string unprefixed = run(prefixed, bare);
        expect(unprefixed.contains("tag 0 at sample 1 lacks key 'mark', expected \"x\"")) << unprefixed;
        const std::string unlisted = run(both, bare);
        expect(unlisted.contains("tag 0 at sample 1 key 'gr:mark' is \"x\", expected no such key")) << unlisted;
    };
};

const boost::ut::suite<"DataSet blocks"> dataSetTests = [] {
    using namespace boost::ut;
    using gr::property_map;
    using Source = gr::blocks::basic::DataSetSource<std::uint8_t>;
    using Sink   = gr::blocks::testing::ExpectDataSetSink<std::uint8_t>;
    const property_map records{{"0", property_map{{"signal_values", std::vector<std::uint8_t>{1U, 2U, 3U}}, {"signal_names", std::vector<std::string>{"payload"}}, //
                                         {"axis_names", std::vector<std::string>{"time"}}, {"meta_information", property_map{{"seq", std::uint32_t(0)}, {"ok", true}}}}},
        {"1", property_map{}}, {"2", property_map{{"signal_values", std::vector<std::uint8_t>{4U}}, {"meta_information", property_map{{"seq", std::uint64_t(1ULL << 40U)}}}}}};

    "DataSetSource records reach ExpectDataSetSink in order"_test = [&] {
        const property_map wanted{{"0", property_map{{"signal_values", std::vector<std::uint8_t>{1U, 2U, 3U}}, {"extents", std::vector<std::int32_t>{3}}, {"signal_names", std::vector<std::string>{"payload"}}, //
                                            {"axis_names", std::vector<std::string>{"time"}}, {"meta_information", property_map{{"seq", std::int64_t(0)}, {"ok", true}}}}},
            {"1", property_map{{"signal_values", std::vector<std::uint8_t>{}}, {"extents", std::vector<std::int32_t>{0}}, {"meta_information", listOf({})}}}, {"2", property_map{{"meta_information", property_map{{"seq", std::uint64_t(1ULL << 40U)}}}, {"meta_absent", std::vector<std::string>{"ok"}}, {"meta_present", std::vector<std::string>{"seq"}}}}};
        const std::string  error = runChain<Source, Sink>(property_map{{"records", records}}, property_map{{"records", wanted}, {"meta_exact", true}});
        expect(error.empty()) << error;
    };

    "ExpectDataSetSink names the record, the field and both values"_test = [&] {
        const std::string wrongMeta = runChain<Source, Sink>(property_map{{"records", records}}, //
            property_map{{"records", property_map{{"0", property_map{{"meta_information", property_map{{"seq", std::int64_t(1)}}}}}, {"1", property_map{}}, {"2", property_map{}}}}});
        expect(wrongMeta.contains("record 0 field 'meta_information' key 'seq' is 0 (uint32), expected 1 (int64)")) << wrongMeta;
        const std::string wrongAxis = runChain<Source, Sink>(property_map{{"records", records}}, //
            property_map{{"records", property_map{{"0", property_map{{"axis_names", std::vector<std::string>{"other"}}}}, {"1", property_map{}}, {"2", property_map{}}}}});
        expect(wrongAxis.contains("record 0 field 'axis_names' is [time], expected [other]")) << wrongAxis;
        const std::string missingKey = runChain<Source, Sink>(property_map{{"records", records}}, //
            property_map{{"records", property_map{{"0", property_map{{"meta_present", std::vector<std::string>{"gone"}}}}, {"1", property_map{}}, {"2", property_map{}}}}});
        expect(missingKey.contains("record 0 field 'meta_present' key 'gone' is missing")) << missingKey;
        const std::string noMap = runChain<Source, Sink>(property_map{{"records", records}}, //
            property_map{{"records", property_map{{"0", property_map{}}, {"1", property_map{{"meta_present", std::vector<std::string>{"seq"}}}}, {"2", property_map{}}}}});
        expect(noMap.contains("record 1 field 'meta_present' key 'seq' is missing, the DataSet has no meta map")) << noMap;
        const std::string tooFew = runChain<Source, Sink>(property_map{{"records", records}}, //
            property_map{{"records", property_map{{"0", property_map{}}, {"1", property_map{}}, {"2", property_map{}}, {"3", property_map{}}}}});
        expect(tooFew.contains("the stream ended after 3 DataSets, expected 4")) << tooFew;
    };

    "DataSetSource names the record and the field of a malformed entry"_test = [] {
        const std::string error = runChain<Source, Sink>(property_map{{"records", property_map{{"0", property_map{{"signal_value", std::vector<std::uint8_t>{1U}}}}}}}, property_map{});
        expect(error.contains("record 0 field 'signal_value' is not a record field")) << error;
    };
};

int main() { /* not needed for UT */ }
