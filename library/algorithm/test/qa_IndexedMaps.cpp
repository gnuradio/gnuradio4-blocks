#include <boost/ut.hpp>

#include <cstdint>
#include <expected>
#include <limits>
#include <memory_resource>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/Tensor.hpp>

#include <gnuradio-4.0/algorithm/dataset/DataSetRecords.hpp>
#include <gnuradio-4.0/algorithm/dataset/IndexedMaps.hpp>

namespace {

using gr::property_map;

gr::pmt::Value listOf(std::initializer_list<property_map> maps) {
    std::pmr::vector<gr::pmt::Value> values;
    for (const property_map& map : maps) {
        values.emplace_back(map);
    }
    return gr::pmt::Value(gr::Tensor<gr::pmt::Value>(std::move(values)));
}

std::string readError(const property_map& entries) {
    const auto read = gr::dataset::readIndexedMaps<std::uint64_t>(entries, "tags", "a sample index");
    return read.has_value() ? std::string{} : read.error();
}

std::string recordError(const property_map& fields) {
    const auto read = gr::dataset::readDataSetRecord<std::uint8_t>(3UZ, fields);
    return read.has_value() ? std::string{} : read.error();
}

} // namespace

const boost::ut::suite<"index-keyed maps"> indexedMapsTests = [] {
    using namespace boost::ut;
    using namespace gr::dataset;

    "an index reads as a decimal integer in the range of its type"_test = [] {
        expect(parseIndex<std::uint64_t>("0") == std::optional<std::uint64_t>(0U));
        expect(parseIndex<std::uint64_t>("18446744073709551615") == std::optional<std::uint64_t>(std::numeric_limits<std::uint64_t>::max()));
        expect(!parseIndex<std::uint64_t>("18446744073709551616").has_value()) << "one past the largest value";
        expect(!parseIndex<std::uint64_t>("-1").has_value());
        expect(parseIndex<std::int64_t>("-1") == std::optional<std::int64_t>(-1));
        for (const std::string_view text : {"", "1x", "+1", " 1", "1 ", "0x1"}) {
            expect(!parseIndex<std::uint64_t>(text).has_value()) << text;
        }
    };

    "the maps come back in index order and the maps of a list in list order"_test = [] {
        const property_map first{{"a", 1}};
        const property_map second{{"b", 2}};
        const property_map third{{"c", 3}};
        const property_map entries{{"12", third}, {"3", listOf({second, first})}, {"0", first}};
        const auto         read = readIndexedMaps<std::uint64_t>(entries, "tags", "a sample index");
        expect(fatal(read.has_value())) << readError(entries);
        expect(fatal(eq(read->size(), 4UZ)));
        const std::vector<std::pair<std::uint64_t, property_map>> wanted{{0U, first}, {3U, second}, {3U, first}, {12U, third}};
        for (std::size_t i = 0UZ; i < wanted.size(); ++i) {
            expect(eq((*read)[i].index, wanted[i].first)) << i;
            expect((*read)[i].map == wanted[i].second) << i;
        }
    };

    "a malformed entry is an error that names its key"_test = [] {
        expect(eq(readError(property_map{{"1x", property_map{}}}), std::string("tags key '1x' is not a sample index")));
        expect(eq(readError(property_map{{"-1", property_map{}}}), std::string("tags key '-1' is not a sample index")));
        expect(eq(readError(property_map{{"1", 5}}), std::string("tags entry at 1 is 5, expected a map or a list of maps")));
        expect(eq(readError(property_map{{"2", listOf({property_map{{"a", 1}}})}, {"1", listOf({})}, {"4", gr::pmt::Value(gr::Tensor<gr::pmt::Value>(std::pmr::vector<gr::pmt::Value>{gr::pmt::Value(5)}))}}), std::string("tags entry at 4 holds 5, expected a map")));
        expect(eq(readError(property_map{{"1", property_map{}}, {"01", property_map{}}}), std::string("tags keys '01' and '1' both name 1")));
    };

    "maps added at one index come back as a list in the order they were added"_test = [] {
        const property_map first{{"a", 1}};
        const property_map second{{"a", 2}};
        const property_map third{{"b", true}};
        property_map       entries;
        appendIndexedMap(entries, std::uint64_t{7}, first);
        expect(entries == property_map{{"7", first}}) << "one map at an index is stored as the map";
        appendIndexedMap(entries, std::uint64_t{7}, second);
        appendIndexedMap(entries, std::uint64_t{2}, third);
        appendIndexedMap(entries, std::uint64_t{7}, third);
        expect(entries == property_map{{"7", listOf({first, second, third})}, {"2", third}});
        const auto read = readIndexedMaps<std::uint64_t>(entries, "tags", "a sample index");
        expect(fatal(read.has_value())) << readError(entries);
        expect(fatal(eq(read->size(), 4UZ)));
        expect((*read)[0].map == third);
        expect((*read)[1].map == first);
        expect((*read)[2].map == second);
        expect((*read)[3].map == third);
    };
};

const boost::ut::suite<"DataSet records"> dataSetRecordsTests = [] {
    using namespace boost::ut;
    using namespace gr::dataset;
    // The error of a read, or an empty string for a read that succeeded.
    const auto errorOf = [](const auto& read) { return read.has_value() ? std::string{} : std::string(read.error()); };

    "a null entry is an empty record and the records come back in index order"_test = [&] {
        const property_map first{{"timestamp", 7}};
        const auto         read = readRecordMaps(property_map{{"1", gr::pmt::Value()}, {"0", first}});
        expect(fatal(read.has_value())) << errorOf(read);
        expect(fatal(eq(read->size(), 2UZ)));
        expect((*read)[0] == first);
        expect((*read)[1].empty());
        const auto none = readRecordMaps(property_map{});
        expect(none.has_value() && none->empty());
    };

    "a records entry that is a list is refused"_test = [&] {
        const auto read = readRecordMaps(property_map{{"0", listOf({property_map{}})}});
        expect(fatal(!read.has_value()));
        const std::string error = errorOf(read);
        expect(error.starts_with("records entry at 0 is ") && error.ends_with(", expected a map of fields")) << error;
    };

    const std::vector<std::string> names{"a", "b"};

    "absent fields take their defaults"_test = [&] {
        const auto one = readDataSetRecord<std::uint8_t>(0UZ, property_map{{"signal_values", std::vector<std::uint8_t>{1U, 2U, 3U, 4U}}});
        expect(fatal(one.has_value())) << errorOf(one);
        expect(one->dataSet.signal_names == std::vector<std::string>{""});
        expect(one->dataSet.extents == std::vector<std::int32_t>{4});
        expect(eq(one->dataSet.timing_events.size(), 1UZ));
        expect(one->dataSet.meta_information.empty());
        expect(eq(one->dataSet.timestamp, gr::DataSet<std::uint8_t>{}.timestamp));
        expect(one->lists("signal_values") && !one->lists("extents"));
        const auto two = readDataSetRecord<std::uint8_t>(0UZ, property_map{{"signal_values", std::vector<std::uint8_t>{1U, 2U, 3U, 4U}}, {"signal_names", names}});
        expect(fatal(two.has_value())) << errorOf(two);
        expect(two->dataSet.signal_names == names);
        expect(two->dataSet.extents == std::vector<std::int32_t>{static_cast<std::int32_t>(4UZ / names.size())});
        expect(eq(two->dataSet.timing_events.size(), names.size()));
    };

    "values read from a typed list or from an untagged list of numbers"_test = [&] {
        const std::pmr::vector<gr::pmt::Value> untagged{gr::pmt::Value(std::int64_t(0)), gr::pmt::Value(std::int64_t(255))};
        const auto                             read = readDataSetRecord<std::uint8_t>(0UZ, property_map{{"signal_values", gr::pmt::Value(gr::Tensor<gr::pmt::Value>(untagged))}, {"extents", std::vector<std::int32_t>{2}}});
        expect(fatal(read.has_value())) << errorOf(read);
        expect(read->dataSet.signal_values == std::vector<std::uint8_t>{0U, 255U});
        expect(read->dataSet.extents == std::vector<std::int32_t>{2});
        const std::string wrongType = recordError(property_map{{"signal_values", std::vector<float>{1.f}}});
        expect(wrongType.starts_with("record 3 field 'signal_values' is ") && wrongType.ends_with(", expected a list of uint8")) << wrongType;
        const std::string tooLarge = recordError(property_map{{"signal_values", gr::pmt::Value(gr::Tensor<gr::pmt::Value>(std::pmr::vector<gr::pmt::Value>{gr::pmt::Value(std::int64_t(256))}))}});
        expect(tooLarge.starts_with("record 3 field 'signal_values' element 256 does not convert to uint8: ")) << tooLarge;
    };

    "timing events keep their negative indices in index order"_test = [&] {
        const property_map before{{"mark", std::string("before")}};
        const property_map after{{"mark", std::string("after")}};
        const auto         events = readTimingEvents(gr::pmt::Value(gr::Tensor<gr::pmt::Value>(std::pmr::vector<gr::pmt::Value>{gr::pmt::Value(property_map{{"2", after}, {"-1", before}}), gr::pmt::Value(property_map{})})));
        expect(fatal(events.has_value())) << errorOf(events);
        const TimingEvents wanted{{{-1, before}, {2, after}}, {}};
        expect(*events == wanted);
        const auto refused = readTimingEvents(gr::pmt::Value(5));
        expect(fatal(!refused.has_value()));
        expect(eq(errorOf(refused), std::string("is 5, expected a list with one map of events per signal")));
    };

    "a field error names the record index and the field"_test = [&] {
        expect(eq(recordError(property_map{{"signal_value", 1}}), std::string("record 3 field 'signal_value' is not a record field")));
        expect(eq(recordError(property_map{{"signal_units", 1}}), std::string("record 3 field 'signal_units' is 1, expected a list of strings")));
        const gr::pmt::Value notMaps(gr::Tensor<gr::pmt::Value>(std::pmr::vector<gr::pmt::Value>{gr::pmt::Value(5)}));
        expect(eq(recordError(property_map{{"meta_information", notMaps}}), std::string("record 3 field 'meta_information' holds 5, expected a map")));
        expect(eq(recordError(property_map{{"meta_information", listOf({})}, {"signal_values", std::vector<std::uint8_t>{1U, 2U, 3U}}, {"signal_names", names}}), std::string("record 3 field 'extents' has no default for 3 values in 2 signals")));
    };
};

int main() { /* not needed for UT */ }
