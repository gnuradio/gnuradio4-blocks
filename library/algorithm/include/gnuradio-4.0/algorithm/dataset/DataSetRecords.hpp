#ifndef GNURADIO_ALGORITHM_DATASET_DATASETRECORDS_HPP
#define GNURADIO_ALGORITHM_DATASET_DATASETRECORDS_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/PmtTypeHelpers.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/algorithm/dataset/IndexedMaps.hpp>
#include <gnuradio-4.0/formatter/ValueFormatter.hpp>
#include <gnuradio-4.0/meta/utils.hpp>

namespace gr::dataset {

// A records map maps each record index, written as a decimal string, to a map of DataSet fields.

/// Reads a records map: record indices from 0 without a gap, each mapped to a map of fields. A null entry is a record
/// without fields. Returns the field maps in record order.
[[nodiscard]] inline std::expected<std::vector<property_map>, std::string> readRecordMaps(const property_map& records) {
    property_map keys;
    for (const auto& [key, value] : records) {
        if (!value.is_monostate() && value.get_if<property_map>() == nullptr) {
            return std::unexpected(std::format("records entry at {} is {}, expected a map of fields", std::string_view(key), value));
        }
        keys.emplace(key, property_map{});
    }
    std::expected<std::vector<IndexedMap<std::size_t>>, std::string> read = readIndexedMaps<std::size_t>(keys, "records", "a record index");
    if (!read.has_value()) {
        return std::unexpected(read.error());
    }
    for (std::size_t i = 0UZ; i < read->size(); ++i) {
        if ((*read)[i].index != i) {
            return std::unexpected(std::format("records lists no record {}", i));
        }
    }
    std::vector<property_map> result(read->size());
    for (const auto& [key, value] : records) {
        const std::optional<std::size_t> index  = parseIndex<std::size_t>(std::string_view(key));
        const property_map*              fields = value.get_if<property_map>();
        if (index.has_value() && *index < result.size() && fields != nullptr) {
            result[*index] = *fields;
        }
    }
    return result;
}

// The error of `readStrings`, `readTimingEvents`, `detail::readNumbers` and `detail::readMetaMaps` is the end of a
// sentence. `readDataSetRecord` starts it with the record index and the field.

/// Reads a list whose every element is a string.
[[nodiscard]] inline std::expected<std::vector<std::string>, std::string> readStrings(const pmt::Value& value) {
    const Tensor<pmt::Value>* list = value.get_if<Tensor<pmt::Value>>();
    if (list == nullptr) {
        return std::unexpected(std::format("is {}, expected a list of strings", value));
    }
    std::vector<std::string> result;
    result.reserve(list->size());
    for (const pmt::Value& element : *list) {
        if (!element.is_string()) {
            return std::unexpected(std::format("holds {}, expected a string", element));
        }
        result.emplace_back(element.value_or(std::string_view{}));
    }
    return result;
}

namespace detail {

/// Reads a list of numbers of type `N`: a list tagged with type `N`, or an untagged list whose elements convert to `N`
/// without loss.
template<typename N>
[[nodiscard]] std::expected<std::vector<N>, std::string> readNumbers(const pmt::Value& value) {
    if (const Tensor<N>* typed = value.get_if<Tensor<N>>(); typed != nullptr) {
        return std::vector<N>(typed->begin(), typed->end());
    }
    const Tensor<pmt::Value>* list = value.get_if<Tensor<pmt::Value>>();
    if (list == nullptr) {
        return std::unexpected(std::format("is {}, expected a list of {}", value, meta::type_name<N>()));
    }
    std::vector<N> result;
    result.reserve(list->size());
    for (const pmt::Value& element : *list) {
        const std::expected<N, std::string> number = pmt::convert_safely<N>(element);
        if (!number.has_value()) {
            return std::unexpected(std::format("element {} does not convert to {}: {}", element, meta::type_name<N>(), number.error()));
        }
        result.push_back(*number);
    }
    return result;
}

/// Reads a map, or a list of maps, of meta information.
[[nodiscard]] inline std::expected<std::vector<property_map>, std::string> readMetaMaps(const pmt::Value& value) {
    if (const property_map* single = value.get_if<property_map>(); single != nullptr) {
        return std::vector<property_map>{*single};
    }
    const Tensor<pmt::Value>* list = value.get_if<Tensor<pmt::Value>>();
    if (list == nullptr) {
        return std::unexpected(std::format("is {}, expected a map or a list of maps", value));
    }
    std::vector<property_map> result;
    result.reserve(list->size());
    for (const pmt::Value& element : *list) {
        const property_map* map = element.get_if<property_map>();
        if (map == nullptr) {
            return std::unexpected(std::format("holds {}, expected a map", element));
        }
        result.push_back(*map);
    }
    return result;
}

/// The fields a record may list.
inline constexpr std::array<std::string_view, 9UZ> kDataSetRecordFields{"signal_values", "signal_names", "signal_quantities", "signal_units", "axis_names", "extents", "timestamp", "meta_information", "timing_events"};

} // namespace detail

/// The `timing_events` of a DataSet: one list per signal of event indices and maps.
using TimingEvents = std::vector<std::vector<std::pair<std::ptrdiff_t, property_map>>>;

/// Reads a list with one entry per signal. Each entry is an index-keyed map of event indices, which may be negative.
[[nodiscard]] inline std::expected<TimingEvents, std::string> readTimingEvents(const pmt::Value& value) {
    const Tensor<pmt::Value>* list = value.get_if<Tensor<pmt::Value>>();
    if (list == nullptr) {
        return std::unexpected(std::format("is {}, expected a list with one map of events per signal", value));
    }
    TimingEvents result;
    result.reserve(list->size());
    for (const pmt::Value& element : *list) {
        const property_map* events = element.get_if<property_map>();
        if (events == nullptr) {
            return std::unexpected(std::format("holds {}, expected a map of events", element));
        }
        std::expected<std::vector<IndexedMap<std::ptrdiff_t>>, std::string> read = readIndexedMaps<std::ptrdiff_t>(*events, std::format("list {}", result.size()), "an event index");
        if (!read.has_value()) {
            return std::unexpected(read.error());
        }
        auto& signalEvents = result.emplace_back();
        signalEvents.reserve(read->size());
        for (IndexedMap<std::ptrdiff_t>& event : *read) {
            signalEvents.emplace_back(event.index, std::move(event.map));
        }
    }
    return result;
}

/// One record of a records map: the DataSet with the defaults of absent fields, and the fields the record lists.
template<typename T>
struct DataSetRecord {
    DataSet<T>               dataSet{};
    std::vector<std::string> listed{};

    [[nodiscard]] bool lists(std::string_view field) const { return std::ranges::contains(listed, field); }
};

/// Reads the fields of record `index`. A record without `signal_names` has one signal with an empty name. A record
/// without `extents` has one dimension, the value count divided by the signal count. A record without
/// `timing_events` has one empty list per signal. Other absent fields keep the defaults of `DataSet<T>`. An unknown
/// field, a value that does not convert and a value count the signal count does not divide are errors that name the
/// record index and the field.
template<typename T>
[[nodiscard]] std::expected<DataSetRecord<T>, std::string> readDataSetRecord(std::size_t index, const property_map& fields) {
    DataSetRecord<T> record;
    DataSet<T>&      dataSet = record.dataSet;
    for (const auto& [key, value] : fields) {
        const std::string_view field(key);
        if (!std::ranges::contains(detail::kDataSetRecordFields, field)) {
            return std::unexpected(std::format("record {} field '{}' is not a record field", index, field));
        }
        record.listed.emplace_back(field);
        std::optional<std::string> error;
        if (field == "signal_values") {
            std::expected<std::vector<T>, std::string> values = detail::readNumbers<T>(value);
            if (values.has_value()) {
                dataSet.signal_values = std::move(*values);
            } else {
                error = values.error();
            }
        } else if (field == "extents") {
            std::expected<std::vector<std::int32_t>, std::string> extents = detail::readNumbers<std::int32_t>(value);
            if (extents.has_value()) {
                dataSet.extents = std::move(*extents);
            } else {
                error = extents.error();
            }
        } else if (field == "timestamp") {
            const std::expected<std::int64_t, std::string> timestamp = pmt::convert_safely<std::int64_t>(value);
            if (timestamp.has_value()) {
                dataSet.timestamp = *timestamp;
            } else {
                error = timestamp.error();
            }
        } else if (field == "meta_information") {
            std::expected<std::vector<property_map>, std::string> maps = detail::readMetaMaps(value);
            if (maps.has_value()) {
                dataSet.meta_information = std::move(*maps);
            } else {
                error = maps.error();
            }
        } else if (field == "timing_events") {
            std::expected<TimingEvents, std::string> events = readTimingEvents(value);
            if (events.has_value()) {
                dataSet.timing_events = std::move(*events);
            } else {
                error = events.error();
            }
        } else {
            std::expected<std::vector<std::string>, std::string> strings = readStrings(value);
            if (!strings.has_value()) {
                error = strings.error();
            } else if (field == "signal_names") {
                dataSet.signal_names = std::move(*strings);
            } else if (field == "signal_quantities") {
                dataSet.signal_quantities = std::move(*strings);
            } else if (field == "signal_units") {
                dataSet.signal_units = std::move(*strings);
            } else {
                dataSet.axis_names = std::move(*strings);
            }
        }
        if (error.has_value()) {
            return std::unexpected(std::format("record {} field '{}' {}", index, field, *error));
        }
    }
    if (!record.lists("signal_names")) {
        dataSet.signal_names.assign(1UZ, std::string{});
    }
    if (!record.lists("extents")) {
        const std::size_t signals = std::max(dataSet.signal_names.size(), 1UZ);
        const std::size_t count   = dataSet.signal_values.size();
        if (count % signals != 0UZ || !std::in_range<std::int32_t>(count / signals)) {
            return std::unexpected(std::format("record {} field 'extents' has no default for {} values in {} signals", index, count, signals));
        }
        dataSet.extents.assign(1UZ, static_cast<std::int32_t>(count / signals));
    }
    if (!record.lists("timing_events")) {
        dataSet.timing_events.resize(dataSet.signal_names.size());
    }
    return record;
}

} // namespace gr::dataset

#endif // GNURADIO_ALGORITHM_DATASET_DATASETRECORDS_HPP
