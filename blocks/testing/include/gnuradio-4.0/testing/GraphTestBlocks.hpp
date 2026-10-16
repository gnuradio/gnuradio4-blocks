#ifndef GNURADIO_TESTING_GRAPH_TEST_BLOCKS_HPP
#define GNURADIO_TESTING_GRAPH_TEST_BLOCKS_HPP

#include <algorithm>
#include <charconv>
#include <cmath>
#include <complex>
#include <cstdint>
#include <expected>
#include <format>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/PmtTypeHelpers.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/formatter/ValueFormatter.hpp>
#include <gnuradio-4.0/meta/formatter.hpp>

namespace gr::blocks::testing {

GR_REGISTER_BLOCK(gr::blocks::testing::VectorSource, [T], [ uint8_t, int16_t, int32_t, float, double, std::complex<float> ])

template<typename T>
struct VectorSource : Block<VectorSource<T>> {
    using Description = Doc<R""(@brief Emits the samples of `values` in order and then ends the stream.

The whole of `values` is emitted `repeat` times. An empty `values` ends the stream at once.
The block publishes `tag_map` as a tag at every sample index in `tag_indices` below the sample count.
An empty `tag_map` publishes no tag.
)"">;

    PortOut<T> out;

    Annotated<std::vector<T>, "values", Doc<"samples to emit, in order">>                       values{};
    Annotated<gr::Size_t, "repeat", Doc<"number of passes over the values">>                    repeat = 1U;
    Annotated<std::vector<gr::Size_t>, "tag_indices", Doc<"sample indices that carry tag_map">> tag_indices{};
    Annotated<property_map, "tag_map", Doc<"keys and values of the published tag">>             tag_map{};

    GR_MAKE_REFLECTABLE(VectorSource, out, values, repeat, tag_indices, tag_map);

    std::size_t _emitted = 0UZ;

    void start() { _emitted = 0UZ; }

    [[nodiscard]] work::Status processBulk(OutputSpanLike auto& outSpan) {
        const std::size_t nValues = values->size();
        const std::size_t total   = nValues * static_cast<std::size_t>(repeat.value);
        const std::size_t n       = std::min(outSpan.size(), total > _emitted ? total - _emitted : 0UZ);
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = values.value[(_emitted + i) % nValues];
        }
        if (!tag_map->empty()) {
            std::vector<gr::Size_t> indices = tag_indices.value;
            std::ranges::sort(indices);
            indices.erase(std::ranges::unique(indices).begin(), indices.end());
            for (const gr::Size_t tagIndex : indices) {
                const std::size_t index = static_cast<std::size_t>(tagIndex);
                if (index >= _emitted && index < _emitted + n) {
                    outSpan.publishTag(tag_map.value, index - _emitted);
                }
            }
        }
        outSpan.publish(n);
        _emitted += n;
        return _emitted >= total ? work::Status::DONE : work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::testing::ExpectSink, [T], [ uint8_t, int16_t, int32_t, uint32_t, float, double, std::complex<float> ])

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
            if (!matches(input[i], expected.value[index])) {
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
        return std::format("tag '{}' at sample {} is missing, expected {}", missing->key, missing->index, describe(missing->value));
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
                    return std::format("tag '{}' at sample {} is {}, expected no such tag", key, index, describe(value));
                }
                continue;
            }
            if (!tagValueMatches(value, listed->value)) {
                if (value.value_type() == listed->value.value_type() && value.container_type() == listed->value.container_type()) {
                    return std::format("tag '{}' at sample {} is {}, expected {}", listed->key, index, describe(value), describe(listed->value));
                }
                return std::format("tag '{}' at sample {} is {} ({}), expected {} ({})", listed->key, index, describe(value), pmt::detail::type_name(value), describe(listed->value), pmt::detail::type_name(listed->value));
            }
            listed->seen = true;
        }
        return std::nullopt;
    }

    /// Compares a numeric expected value in its own type and any other value by type and content.
    [[nodiscard]] static bool tagValueMatches(const pmt::Value& value, const pmt::Value& reference) {
        if (!reference.is_integral() && !reference.is_floating_point()) {
            return value == reference;
        }
        return withNumericType(reference.value_type(), [&]<typename U>(std::type_identity<U>) { return equalAs<U>(value, reference); });
    }

    /// Calls `fn` with the identity of the numeric type `type` names. Returns false for a type that is not numeric.
    template<typename Fn>
    [[nodiscard]] static bool withNumericType(pmt::Value::ValueType type, Fn&& fn) {
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

    /// Formats a value. A floating-point value takes the shortest text that reads back to the same value.
    [[nodiscard]] static std::string describe(const pmt::Value& value) {
        if (const float* single = value.get_if<float>(); single != nullptr) {
            return std::format("{}", *single);
        }
        if (const double* wide = value.get_if<double>(); wide != nullptr) {
            return std::format("{}", *wide);
        }
        return std::format("{}", value);
    }

    /// Converts `value` to `U` and compares it with `reference`. The converted value must convert back to `value`.
    template<typename U>
    [[nodiscard]] static bool equalAs(const pmt::Value& value, const pmt::Value& reference) {
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

    void failAtStart(const std::string& message) {
        _failed = true;
        this->emitErrorMessage("start", message);
    }

    [[nodiscard]] work::Status fail(std::string_view endpoint, const std::string& message) {
        _failed = true;
        this->emitErrorMessage(endpoint, message);
        return work::Status::ERROR;
    }

    [[nodiscard]] bool matches(const T& value, const T& reference) const {
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
            return within(std::complex<double>(value), std::complex<double>(reference));
        } else {
            if constexpr (std::floating_point<T>) {
                if (std::isnan(value) || std::isnan(reference)) {
                    return std::isnan(value) && std::isnan(reference);
                }
                if (std::isinf(value) || std::isinf(reference)) {
                    return value == reference;
                }
            }
            return within(asDouble(value), asDouble(reference));
        }
    }

    [[nodiscard]] static double asDouble(const T& sample) {
        if constexpr (std::same_as<T, double>) {
            return sample;
        } else {
            return static_cast<double>(sample);
        }
    }

    /// Returns whether the distance between the samples is at most `tolerance + relative_tolerance * |reference|`.
    /// With finite tolerances, the sink compares at a quarter of the scale when the distance or the bound exceeds
    /// the largest double. At that scale the distance between two finite samples, real or complex, stays finite.
    /// A bound that still exceeds the largest double then exceeds every such distance.
    /// With a non-finite tolerance, the sink compares at full scale. A quarter of a subnormal expected sample can
    /// round to zero, and an infinite relative tolerance times zero is NaN.
    template<typename Sample>
    [[nodiscard]] bool within(const Sample& value, const Sample& reference) const {
        const double distance = std::abs(value - reference);
        const double bound    = tolerance.value + relative_tolerance.value * std::abs(reference);
        if (!std::isfinite(tolerance.value) || !std::isfinite(relative_tolerance.value)) {
            return distance <= bound;
        }
        if (std::isfinite(distance) && std::isfinite(bound)) {
            return distance <= bound;
        }
        constexpr double scale = 0.25;
        return std::abs(value * scale - reference * scale) <= tolerance.value * scale + relative_tolerance.value * std::abs(reference * scale);
    }
};

} // namespace gr::blocks::testing

#endif // GNURADIO_TESTING_GRAPH_TEST_BLOCKS_HPP
