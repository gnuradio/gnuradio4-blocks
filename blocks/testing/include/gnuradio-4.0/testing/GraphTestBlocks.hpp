#ifndef GNURADIO_TESTING_GRAPH_TEST_BLOCKS_HPP
#define GNURADIO_TESTING_GRAPH_TEST_BLOCKS_HPP

#include <algorithm>
#include <charconv>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/PmtTypeHelpers.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/algorithm/dataset/DataSetRecords.hpp>
#include <gnuradio-4.0/formatter/ValueFormatter.hpp>
#include <gnuradio-4.0/meta/formatter.hpp>

namespace gr::blocks::testing {

namespace detail {

/// Returns whether the distance between the samples is at most `tolerance + relativeTolerance * |reference|`.
/// With finite tolerances, the comparison runs at a quarter of the scale when the distance or the bound exceeds
/// the largest double. At that scale the distance between two finite samples, real or complex, stays finite.
/// A bound that still exceeds the largest double then exceeds every such distance.
/// With a non-finite tolerance, the comparison runs at full scale. A quarter of a subnormal expected sample can
/// round to zero, and an infinite relative tolerance times zero is NaN.
template<typename Sample>
[[nodiscard]] bool withinDistance(const Sample& value, const Sample& reference, double tolerance, double relativeTolerance) {
    const double distance = std::abs(value - reference);
    const double bound    = tolerance + relativeTolerance * std::abs(reference);
    if (!std::isfinite(tolerance) || !std::isfinite(relativeTolerance)) {
        return distance <= bound;
    }
    if (std::isfinite(distance) && std::isfinite(bound)) {
        return distance <= bound;
    }
    constexpr double scale = 0.25;
    return std::abs(value * scale - reference * scale) <= tolerance * scale + relativeTolerance * std::abs(reference * scale);
}

/// A NaN matches a NaN. An infinity, or a complex value with an infinite part, matches only an equal value. Any other
/// value matches within the tolerances.
template<typename T>
[[nodiscard]] bool withinTolerance(const T& value, const T& reference, double tolerance, double relativeTolerance) {
    if constexpr (meta::complex_like<T>) {
        const bool valueNan     = std::isnan(value.real()) || std::isnan(value.imag());
        const bool referenceNan = std::isnan(reference.real()) || std::isnan(reference.imag());
        if (valueNan || referenceNan) {
            return valueNan && referenceNan;
        }
        const bool valueInf     = std::isinf(value.real()) || std::isinf(value.imag());
        const bool referenceInf = std::isinf(reference.real()) || std::isinf(reference.imag());
        if (valueInf || referenceInf) {
            return value == reference;
        }
        return withinDistance(std::complex<double>(value), std::complex<double>(reference), tolerance, relativeTolerance);
    } else {
        if constexpr (std::floating_point<T>) {
            if (std::isnan(value) || std::isnan(reference)) {
                return std::isnan(value) && std::isnan(reference);
            }
            if (std::isinf(value) || std::isinf(reference)) {
                return value == reference;
            }
        }
        if constexpr (std::same_as<T, double>) {
            return withinDistance(value, reference, tolerance, relativeTolerance);
        } else {
            return withinDistance(static_cast<double>(value), static_cast<double>(reference), tolerance, relativeTolerance);
        }
    }
}

/// Calls `fn` with the identity of the numeric type `type` names. Returns false for a type that is not numeric.
template<typename Fn>
[[nodiscard]] bool withNumericType(pmt::Value::ValueType type, Fn&& fn) {
    using enum pmt::Value::ValueType;
    switch (type) {
    case Int8: return fn(std::type_identity<std::int8_t>{});
    case Int16: return fn(std::type_identity<std::int16_t>{});
    case Int32: return fn(std::type_identity<std::int32_t>{});
    case Int64: return fn(std::type_identity<std::int64_t>{});
    case UInt8: return fn(std::type_identity<std::uint8_t>{});
    case UInt16: return fn(std::type_identity<std::uint16_t>{});
    case UInt32: return fn(std::type_identity<std::uint32_t>{});
    case UInt64: return fn(std::type_identity<std::uint64_t>{});
    case Float32: return fn(std::type_identity<float>{});
    case Float64: return fn(std::type_identity<double>{});
    default: return false;
    }
}

/// Converts `value` to `U` and compares it with `reference`. The converted value must convert back to `value`.
template<typename U>
[[nodiscard]] bool equalAs(const pmt::Value& value, const pmt::Value& reference) {
    const std::expected<U, std::string> converted = pmt::convert_numerically<U>(value);
    const U*                            wanted    = reference.get_if<U>();
    if (!converted.has_value() || wanted == nullptr || *converted != *wanted) {
        return false;
    }
    return withNumericType(value.value_type(), [&]<typename V>(std::type_identity<V>) {
        const std::expected<V, std::string> back     = pmt::convert_numerically<V>(pmt::Value(*converted));
        const V*                            received = value.get_if<V>();
        return back.has_value() && received != nullptr && *back == *received;
    });
}

/// Compares a numeric expected value in its own type and any other value by type and content.
[[nodiscard]] inline bool valueMatches(const pmt::Value& value, const pmt::Value& reference) {
    if (!reference.is_integral() && !reference.is_floating_point()) {
        return value == reference;
    }
    return withNumericType(reference.value_type(), [&]<typename U>(std::type_identity<U>) { return equalAs<U>(value, reference); });
}

/// Formats a value. A floating-point value takes the shortest text that reads back to the same value.
[[nodiscard]] inline std::string describe(const pmt::Value& value) {
    if (const float* single = value.get_if<float>(); single != nullptr) {
        return std::format("{}", *single);
    }
    if (const double* wide = value.get_if<double>(); wide != nullptr) {
        return std::format("{}", *wide);
    }
    return std::format("{}", value);
}

/// Formats a received and an expected value as "<received>, expected <expected>". Each value carries its type
/// when the types differ.
[[nodiscard]] inline std::string describeMismatch(const pmt::Value& value, const pmt::Value& reference) {
    if (value.value_type() == reference.value_type() && value.container_type() == reference.container_type()) {
        return std::format("{}, expected {}", describe(value), describe(reference));
    }
    return std::format("{} ({}), expected {} ({})", describe(value), pmt::detail::type_name(value), describe(reference), pmt::detail::type_name(reference));
}

/// Returns the first difference between a received map and an expected one. Each listed key must arrive with a
/// matching value, and a key listed with a null value must be missing. With `exact`, an unlisted key is a difference.
[[nodiscard]] inline std::optional<std::string> mapMismatch(const property_map& value, const property_map& reference, bool exact) {
    for (const auto& [key, wanted] : reference) {
        const auto found = value.find(key);
        if (wanted.is_monostate()) {
            if (found != value.end()) {
                return std::format("key '{}' is {}, expected no such key", std::string_view(key), describe(found->second));
            }
            continue;
        }
        if (found == value.end()) {
            return std::format("key '{}' is missing, expected {}", std::string_view(key), describe(wanted));
        }
        if (!valueMatches(found->second, wanted)) {
            return std::format("key '{}' is {}", std::string_view(key), describeMismatch(found->second, wanted));
        }
    }
    if (exact) {
        for (const auto& [key, received] : value) {
            if (!reference.contains(key)) {
                return std::format("key '{}' is {}, expected no such key", std::string_view(key), describe(received));
            }
        }
    }
    return std::nullopt;
}

/// Returns whether two sets of timing events hold equal indices in the same order, their maps compared by
/// `mapMismatch`.
[[nodiscard]] inline bool eventsMatch(const gr::dataset::TimingEvents& value, const gr::dataset::TimingEvents& reference, bool exact) {
    if (value.size() != reference.size()) {
        return false;
    }
    for (std::size_t list = 0UZ; list < value.size(); ++list) {
        if (value[list].size() != reference[list].size()) {
            return false;
        }
        for (std::size_t event = 0UZ; event < value[list].size(); ++event) {
            if (value[list][event].first != reference[list][event].first || mapMismatch(value[list][event].second, reference[list][event].second, exact).has_value()) {
                return false;
            }
        }
    }
    return true;
}

/// One record of ExpectDataSetSink's `records`: the DataSet fields it lists, and the keys of `meta_absent` and
/// `meta_present`.
template<typename T>
struct ExpectedRecord {
    gr::dataset::DataSetRecord<T> fields{};
    std::vector<std::string>      metaAbsent{};
    std::vector<std::string>      metaPresent{};
};

/// Reads record `index`: `meta_absent` and `meta_present` here, the other fields as `gr::dataset::readDataSetRecord`.
template<typename T>
[[nodiscard]] std::expected<ExpectedRecord<T>, std::string> readExpectedRecord(std::size_t index, const property_map& fields) {
    ExpectedRecord<T> result;
    property_map      dataSetFields;
    for (const auto& [key, value] : fields) {
        const std::string_view field(key);
        if (field != "meta_absent" && field != "meta_present") {
            dataSetFields.emplace(key, value);
            continue;
        }
        std::expected<std::vector<std::string>, std::string> keys = gr::dataset::readStrings(value);
        if (!keys.has_value()) {
            return std::unexpected(std::format("record {} field '{}' {}", index, field, keys.error()));
        }
        (field == "meta_absent" ? result.metaAbsent : result.metaPresent) = std::move(*keys);
    }
    std::expected<gr::dataset::DataSetRecord<T>, std::string> record = gr::dataset::readDataSetRecord<T>(index, dataSetFields);
    if (!record.has_value()) {
        return std::unexpected(record.error());
    }
    result.fields = std::move(*record);
    return result;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::testing::ExpectSink, [T], [ uint8_t, int16_t, uint16_t, int32_t, uint32_t, float, double, std::complex<float>, std::complex<double> ])

template<typename T>
struct ExpectSink : Block<ExpectSink<T>> {
    using Description = Doc<R""(@brief Compares the input stream with `expected` and reports the first difference as an error.

A sample matches when its distance from the expected sample is at most
`tolerance + relative_tolerance * |expected|`. A NaN matches a NaN, and an infinity matches the same infinity.
A complex sample with an infinite part matches only an equal sample.
The first sample that does not match ends the run with an error that names its index, its value and the
expected value. A stream shorter than `expected` ends with an error that names both counts. A sample past the
end of `expected` ends the run with an error that names its index.
A non-empty `tag_key` also checks the tags. The sample indices of the tags carrying that key must equal
`tag_indices` when the stream ends, each index counted once. An empty `tag_indices` requires that no tag
carries the key. A tag at the end-of-stream index, one past the last sample, has the sample count as its index.
`expected_tags` maps sample indices to the tags expected there. Each key of the map is a sample index written as
a string, and each value is a map of tag keys and values. The block reads `expected_tags` when it starts.
Every listed key must arrive on a tag at its index.
A tag key with the `gr:` prefix matches the listed key without it. A numeric expected value is compared in its
own type: the received value must convert to that type and back without loss and be equal. Any other
value must have the same type and be equal. A wrong value ends the run with an error that names the index, the key,
the received value and the expected value. A listed key still missing at the end of the stream ends the run
with an error that names the index, the key and the expected value.
With `tags_exact`, a tag key not listed at its index ends the run with an error that names the index, the key
and the value. This check skips the end-of-stream key. A listed end-of-stream key is matched like any other.
`tags_exact` also refuses a setting an upstream block forwards as a tag, such as `sample_rate`, unless `expected_tags`
lists it.
Two spellings of one index that list the same tag key end the run with an error that names both spellings.
)"">;

    PortIn<T> in;

    Annotated<std::vector<T>, "expected", Doc<"samples the input must carry, in order">>                  expected{};
    Annotated<double, "tolerance", Doc<"absolute deviation accepted per sample">>                         tolerance          = 0.0;
    Annotated<double, "relative_tolerance", Doc<"deviation accepted per sample, relative to |expected|">> relative_tolerance = 0.0;
    Annotated<gr::Size_t, "received", Doc<"samples received and matched">>                                received           = 0U;
    Annotated<std::string, "tag_key", Doc<"tag key whose indices are checked">>                           tag_key{};
    Annotated<std::vector<gr::Size_t>, "tag_indices", Doc<"sample indices of the tags carrying tag_key">> tag_indices{};
    Annotated<property_map, "expected_tags", Doc<"tag keys and values expected at each sample index">>    expected_tags{};
    Annotated<bool, "tags_exact", Doc<"refuse tag keys unlisted at their sample index">>                  tags_exact = false;

    GR_MAKE_REFLECTABLE(ExpectSink, in, expected, tolerance, relative_tolerance, received, tag_key, tag_indices, expected_tags, tags_exact);

    struct ExpectedTag {
        std::size_t index = 0UZ;
        std::string spelling{};
        std::string key{};
        pmt::Value  value{};
        bool        seen = false;
    };

    bool                     _failed = false;
    std::vector<std::size_t> _tagged{};
    std::vector<ExpectedTag> _expectedTags{};

    void start() {
        received = 0U;
        _failed  = false;
        _tagged.clear();
        _expectedTags.clear();
        for (const auto& [indexKey, entry] : expected_tags.value) {
            std::size_t       index = 0UZ;
            const char* const last  = indexKey.data() + indexKey.size();
            const auto [parsed, ec] = std::from_chars(indexKey.data(), last, index);
            if (indexKey.empty() || ec != std::errc{} || parsed != last) {
                failAtStart(std::format("expected_tags key '{}' is not a sample index", indexKey));
                return;
            }
            const property_map* tagMap = entry.get_if<property_map>();
            if (tagMap == nullptr) {
                failAtStart(std::format("expected_tags entry at sample {} is {}, expected a map of tag keys and values", index, entry));
                return;
            }
            for (const auto& [key, value] : *tagMap) {
                _expectedTags.push_back(ExpectedTag{.index = index, .spelling = std::string(indexKey), .key = std::string(key), .value = value});
            }
        }
        std::ranges::sort(_expectedTags, {}, [](const ExpectedTag& tag) { return std::tie(tag.index, tag.key); });
        const auto duplicate = std::ranges::adjacent_find(_expectedTags, [](const ExpectedTag& a, const ExpectedTag& b) { return a.index == b.index && a.key == b.key; });
        if (duplicate != _expectedTags.end()) {
            failAtStart(std::format("expected_tags keys '{}' and '{}' both list tag '{}' at sample {}", duplicate->spelling, std::next(duplicate)->spelling, duplicate->key, duplicate->index));
        }
    }

    void stop() {
        if (!_failed && static_cast<std::size_t>(received.value) != expected->size()) {
            _failed = true;
            this->emitErrorMessage("stop", std::format("the stream ended after {} samples, expected {}", received.value, expected->size()));
        }
        if (!_failed && !tag_key->empty()) {
            std::vector<std::size_t> wanted(tag_indices->begin(), tag_indices->end());
            std::ranges::sort(wanted);
            wanted.erase(std::ranges::unique(wanted).begin(), wanted.end());
            std::ranges::sort(_tagged);
            if (wanted != _tagged) {
                _failed = true;
                this->emitErrorMessage("stop", std::format("tags with key '{}' at {}, expected at {}", tag_key.value, _tagged, wanted));
            }
        }
        if (!_failed && static_cast<std::size_t>(received.value) == expected->size()) {
            if (std::optional<std::string> error = missingTag()) {
                _failed = true;
                this->emitErrorMessage("stop", *error);
            }
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& input) {
        if (_failed) {
            return work::Status::ERROR;
        }
        if (!tag_key->empty() || checksTags()) {
            if (std::optional<std::string> error = readTags(input)) {
                return fail("processBulk", *error);
            }
        }
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            const std::size_t index = static_cast<std::size_t>(received.value) + i;
            if (index >= expected->size()) {
                received = static_cast<gr::Size_t>(index);
                return fail("processBulk", std::format("sample {} is {}, beyond the {} expected samples", index, input[i], expected->size()));
            }
            if (!detail::withinTolerance(input[i], expected.value[index], tolerance.value, relative_tolerance.value)) {
                received = static_cast<gr::Size_t>(index);
                return fail("processBulk", std::format("sample {} is {}, expected {}", index, input[i], expected.value[index]));
            }
        }
        received = static_cast<gr::Size_t>(received.value + input.size());
        return work::Status::OK;
    }

    /// The stream's last samples, then the tags at or past the end of the stream, which no sample carries.
    /// A listed tag still missing then fails the run. A wrong sample count fails the run in `stop()` instead.
    /// `stop()` repeats the missing-tag check for a stream that ends without the epilogue.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& input) {
        const work::Status status = processBulk(input);
        if (status != work::Status::OK || (tag_key->empty() && !checksTags())) {
            return status;
        }
        const std::size_t end = input.streamIndex + input.size();
        for (const gr::Tag& tag : in.tagReader().get()) {
            if (tag.index < end) {
                continue;
            }
            if (std::optional<std::string> error = readTag(static_cast<std::size_t>(received.value) + (tag.index - end), tag.map)) {
                return fail("processEpilogue", *error);
            }
        }
        if (static_cast<std::size_t>(received.value) != expected->size()) {
            return status;
        }
        if (std::optional<std::string> error = missingTag()) {
            return fail("processEpilogue", *error);
        }
        return status;
    }

    [[nodiscard]] std::optional<std::string> missingTag() const {
        const auto missing = std::ranges::find_if(_expectedTags, [](const ExpectedTag& tag) { return !tag.seen; });
        if (missing == _expectedTags.end()) {
            return std::nullopt;
        }
        return std::format("tag '{}' at sample {} is missing, expected {}", missing->key, missing->index, detail::describe(missing->value));
    }

    [[nodiscard]] bool checksTags() const { return !_expectedTags.empty() || tags_exact.value; }

    /// A tag at a negative relative index belongs to an earlier call and is skipped.
    [[nodiscard]] std::optional<std::string> readTags(InputSpanLike auto& input) {
        for (const auto& [relIndex, tagMap] : input.tags()) {
            if (relIndex < 0 || static_cast<std::size_t>(relIndex) >= input.size()) {
                continue;
            }
            if (std::optional<std::string> error = readTag(static_cast<std::size_t>(received.value) + static_cast<std::size_t>(relIndex), tagMap.get())) {
                return error;
            }
        }
        return std::nullopt;
    }

    /// Records the index of a tag carrying `tag_key` and checks each key of the tag against `expected_tags`.
    [[nodiscard]] std::optional<std::string> readTag(std::size_t index, const property_map& map) {
        if (!tag_key->empty() && (map.contains(tag_key.value) || map.contains("gr:" + tag_key.value)) && !std::ranges::contains(_tagged, index)) {
            _tagged.push_back(index);
        }
        if (!checksTags()) {
            return std::nullopt;
        }
        for (const auto& [mapKey, value] : map) {
            const std::string_view key(mapKey);
            const auto             listed = std::ranges::find_if(_expectedTags, [index, key](const ExpectedTag& tag) { return tag.index == index && (key == tag.key || (key.starts_with("gr:") && key.substr(3) == tag.key)); });
            if (listed == _expectedTags.end()) {
                if (tags_exact.value && key != tag::END_OF_STREAM.shortKey() && key != tag::END_OF_STREAM.key()) {
                    return std::format("tag '{}' at sample {} is {}, expected no such tag", key, index, detail::describe(value));
                }
                continue;
            }
            if (!detail::valueMatches(value, listed->value)) {
                return std::format("tag '{}' at sample {} is {}", listed->key, index, detail::describeMismatch(value, listed->value));
            }
            listed->seen = true;
        }
        return std::nullopt;
    }

    void failAtStart(const std::string& message) {
        _failed = true;
        this->emitErrorMessage("start", message);
    }

    [[nodiscard]] work::Status fail(std::string_view endpoint, const std::string& message) {
        _failed = true;
        this->emitErrorMessage(endpoint, message);
        return work::Status::ERROR;
    }
};

GR_REGISTER_BLOCK(gr::blocks::testing::ExpectDataSetSink, [T], [ uint8_t, float ])

template<typename T>
struct ExpectDataSetSink : Block<ExpectDataSetSink<T>> {
    using Description = Doc<R""(@brief Compares each input DataSet with the record of `records` at its index and reports the first difference as an error.

`records` takes the form of the `records` setting of `basic::DataSetSource`, and a record checks only the fields it
lists. An empty record accepts any DataSet.
`signal_values` must hold as many values as the record lists. A value matches when its distance from the expected
value is at most `tolerance + relative_tolerance * |expected|`. A NaN matches a NaN, and an infinity matches the
same infinity. `signal_names`, `signal_quantities`, `signal_units`, `axis_names`, `extents` and `timestamp` must be
equal.
`meta_information` must hold as many maps as the record lists. Each key of a listed map must arrive in the map at
the same position. A numeric expected value is compared in its own type: the received value must convert to that
type and back without loss and be equal. Any other value must have the same type and be equal. A key listed with a
null value must be missing. `meta_absent` lists keys that every map of the DataSet must lack. `meta_present` lists
keys that every map of the DataSet must carry, with any value, and a DataSet without a meta map lacks them.
With `meta_exact`, a key that a listed meta map or event map does not list ends the run with an error. A record
without `meta_information` accepts any meta map.
`timing_events` must hold as many lists as the record lists, each with the listed events in order. Two events match
when their indices are equal and their maps match as the meta maps do.
The block reads `records` when it starts and refuses a malformed record as `basic::DataSetSource` does.
A DataSet that does not match ends the run with an error that names the record index, the field, the received value
and the expected value. A DataSet past the last record ends the run with an error that names its index. A stream
that ends with fewer DataSets than `records` lists ends with an error that names both counts.
)"">;

    PortIn<DataSet<T>, Async> in;

    Annotated<property_map, "records", Doc<"expected DataSets, keyed by record index">>                  records{};
    Annotated<double, "tolerance", Doc<"absolute deviation accepted per value">>                         tolerance          = 0.0;
    Annotated<double, "relative_tolerance", Doc<"deviation accepted per value, relative to |expected|">> relative_tolerance = 0.0;
    Annotated<bool, "meta_exact", Doc<"refuse meta and event keys unlisted in the record">>              meta_exact         = false;
    Annotated<gr::Size_t, "received", Doc<"DataSets received and matched">>                              received           = 0U;

    GR_MAKE_REFLECTABLE(ExpectDataSetSink, in, records, tolerance, relative_tolerance, meta_exact, received);

    std::vector<detail::ExpectedRecord<T>> _expected{};
    bool                                   _failed = false;

    void start() {
        received = 0U;
        _failed  = false;
        _expected.clear();
        std::expected<std::vector<property_map>, std::string> maps = gr::dataset::readRecordMaps(records.value);
        if (!maps.has_value()) {
            failAtStart(maps.error());
            return;
        }
        _expected.reserve(maps->size());
        for (std::size_t i = 0UZ; i < maps->size(); ++i) {
            std::expected<detail::ExpectedRecord<T>, std::string> record = detail::readExpectedRecord<T>(i, (*maps)[i]);
            if (!record.has_value()) {
                failAtStart(record.error());
                return;
            }
            _expected.push_back(std::move(*record));
        }
    }

    void stop() {
        if (!_failed && static_cast<std::size_t>(received.value) != _expected.size()) {
            _failed = true;
            this->emitErrorMessage("stop", std::format("the stream ended after {} DataSets, expected {}", received.value, _expected.size()));
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& input) {
        if (_failed) {
            return work::Status::ERROR;
        }
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            const std::size_t index = static_cast<std::size_t>(received.value);
            if (index >= _expected.size()) {
                return fail(std::format("record {} is beyond the {} expected records", index, _expected.size()));
            }
            if (std::optional<std::string> error = mismatch(index, input[i], _expected[index])) {
                return fail(*error);
            }
            received = static_cast<gr::Size_t>(index + 1UZ);
        }
        std::ignore = input.consume(input.size());
        return work::Status::OK;
    }

    /// Returns the first difference between DataSet `index` and its record, in the order of the Description.
    [[nodiscard]] std::optional<std::string> mismatch(std::size_t index, const DataSet<T>& value, const detail::ExpectedRecord<T>& expectedRecord) const {
        const gr::dataset::DataSetRecord<T>& record    = expectedRecord.fields;
        const DataSet<T>&                    reference = record.dataSet;
        const auto                           differs   = [index, &record]<typename Field>(std::string_view field, const Field& actual, const Field& wanted) -> std::optional<std::string> {
            if (!record.lists(field) || actual == wanted) {
                return std::nullopt;
            }
            return std::format("record {} field '{}' is {}, expected {}", index, field, actual, wanted);
        };
        if (record.lists("signal_values")) {
            if (value.signal_values.size() != reference.signal_values.size()) {
                return std::format("record {} field 'signal_values' holds {} values, expected {}", index, value.signal_values.size(), reference.signal_values.size());
            }
            for (std::size_t v = 0UZ; v < value.signal_values.size(); ++v) {
                if (!detail::withinTolerance(value.signal_values[v], reference.signal_values[v], tolerance.value, relative_tolerance.value)) {
                    return std::format("record {} field 'signal_values' value {} is {}, expected {}", index, v, value.signal_values[v], reference.signal_values[v]);
                }
            }
        }
        if (auto error = differs("signal_names", value.signal_names, reference.signal_names)) {
            return error;
        }
        if (auto error = differs("signal_quantities", value.signal_quantities, reference.signal_quantities)) {
            return error;
        }
        if (auto error = differs("signal_units", value.signal_units, reference.signal_units)) {
            return error;
        }
        if (auto error = differs("axis_names", value.axis_names, reference.axis_names)) {
            return error;
        }
        if (auto error = differs("extents", value.extents, reference.extents)) {
            return error;
        }
        if (auto error = differs("timestamp", value.timestamp, reference.timestamp)) {
            return error;
        }
        if (record.lists("meta_information")) {
            if (value.meta_information.size() != reference.meta_information.size()) {
                return std::format("record {} field 'meta_information' holds {} maps, expected {}", index, value.meta_information.size(), reference.meta_information.size());
            }
            for (std::size_t m = 0UZ; m < value.meta_information.size(); ++m) {
                if (std::optional<std::string> error = detail::mapMismatch(value.meta_information[m], reference.meta_information[m], meta_exact.value)) {
                    const std::string position = value.meta_information.size() > 1UZ ? std::format("map {} ", m) : std::string{};
                    return std::format("record {} field 'meta_information' {}{}", index, position, *error);
                }
            }
        }
        for (const property_map& map : value.meta_information) {
            for (const std::string& key : expectedRecord.metaAbsent) {
                if (const auto found = map.find(key); found != map.end()) {
                    return std::format("record {} field 'meta_absent' key '{}' is {}, expected no such key", index, key, detail::describe(found->second));
                }
            }
        }
        for (const std::string& key : expectedRecord.metaPresent) {
            if (value.meta_information.empty()) {
                return std::format("record {} field 'meta_present' key '{}' is missing, the DataSet has no meta map", index, key);
            }
            for (const property_map& map : value.meta_information) {
                if (!map.contains(key)) {
                    return std::format("record {} field 'meta_present' key '{}' is missing", index, key);
                }
            }
        }
        if (record.lists("timing_events") && !detail::eventsMatch(value.timing_events, reference.timing_events, meta_exact.value)) {
            return std::format("record {} field 'timing_events' is {}, expected {}", index, value.timing_events, reference.timing_events);
        }
        return std::nullopt;
    }

    void failAtStart(const std::string& message) {
        _failed = true;
        this->emitErrorMessage("start", message);
    }

    [[nodiscard]] work::Status fail(const std::string& message) {
        _failed = true;
        this->emitErrorMessage("processBulk", message);
        return work::Status::ERROR;
    }
};

} // namespace gr::blocks::testing

#endif // GNURADIO_TESTING_GRAPH_TEST_BLOCKS_HPP
