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

int main() { /* not needed for UT */ }
