#ifndef GNURADIO_ALGORITHM_DATASET_INDEXEDMAPS_HPP
#define GNURADIO_ALGORITHM_DATASET_INDEXEDMAPS_HPP

#include <algorithm>
#include <charconv>
#include <concepts>
#include <expected>
#include <format>
#include <iterator>
#include <memory_resource>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/formatter/ValueFormatter.hpp>

namespace gr::dataset {

// An index-keyed map maps each index, written as a decimal string, to one map or to a list of maps.

/// Reads `text` as a decimal integer of type `Index`. Returns nothing for any other text, and for a value outside the
/// range of `Index`.
template<std::integral Index>
[[nodiscard]] std::optional<Index> parseIndex(std::string_view text) noexcept {
    if (text.empty()) {
        return std::nullopt;
    }
    Index             index = 0;
    const char* const last  = text.data() + text.size();
    const auto [parsed, ec] = std::from_chars(text.data(), last, index);
    if (ec != std::errc{} || parsed != last) {
        return std::nullopt;
    }
    return index;
}

/// One map of an index-keyed map, with its index.
template<std::integral Index>
struct IndexedMap {
    Index        index{};
    property_map map{};
};

/// Reads an index-keyed map. Returns its maps sorted by index, the maps of one list in list order. A key that is not
/// an `Index`, two keys naming one index, an entry that is neither a map nor a list, and a list element that is not a
/// map are errors. `name` and `indexName` word the error: "<name> key '<key>' is not <indexName>".
template<std::integral Index>
[[nodiscard]] std::expected<std::vector<IndexedMap<Index>>, std::string> readIndexedMaps(const property_map& entries, std::string_view name, std::string_view indexName) {
    struct Entry {
        Index             index{};
        std::string_view  spelling{};
        const pmt::Value* value = nullptr;
    };
    std::vector<Entry> keyed;
    keyed.reserve(entries.size());
    for (const auto& [key, entry] : entries) {
        const std::optional<Index> index = parseIndex<Index>(std::string_view(key));
        if (!index.has_value()) {
            return std::unexpected(std::format("{} key '{}' is not {}", name, std::string_view(key), indexName));
        }
        keyed.push_back(Entry{.index = *index, .spelling = std::string_view(key), .value = &entry});
    }
    std::ranges::sort(keyed, {}, [](const Entry& entry) { return std::pair(entry.index, entry.spelling); });
    if (const auto twice = std::ranges::adjacent_find(keyed, {}, &Entry::index); twice != keyed.end()) {
        return std::unexpected(std::format("{} keys '{}' and '{}' both name {}", name, twice->spelling, std::next(twice)->spelling, twice->index));
    }

    std::vector<IndexedMap<Index>> result;
    result.reserve(keyed.size());
    for (const Entry& entry : keyed) {
        const pmt::Value& value = *entry.value;
        if (const property_map* single = value.get_if<property_map>(); single != nullptr) {
            result.push_back(IndexedMap<Index>{.index = entry.index, .map = *single});
            continue;
        }
        const Tensor<pmt::Value>* list = value.get_if<Tensor<pmt::Value>>();
        if (list == nullptr) {
            return std::unexpected(std::format("{} entry at {} is {}, expected a map or a list of maps", name, entry.spelling, value));
        }
        for (const pmt::Value& element : *list) {
            const property_map* map = element.get_if<property_map>();
            if (map == nullptr) {
                return std::unexpected(std::format("{} entry at {} holds {}, expected a map", name, entry.spelling, element));
            }
            result.push_back(IndexedMap<Index>{.index = entry.index, .map = *map});
        }
    }
    return result;
}

/// Adds `map` at `index` to an index-keyed map. The first map at an index is stored as a map. A further map at that
/// index turns the entry into a list of maps in the order they were added.
template<std::integral Index>
void appendIndexedMap(property_map& entries, Index index, property_map map) {
    std::pmr::string key(std::to_string(index));
    const auto       found = entries.find(key);
    if (found == entries.end()) {
        entries.emplace(std::move(key), std::move(map));
        return;
    }
    std::pmr::vector<pmt::Value> list;
    if (const property_map* single = found->second.get_if<property_map>(); single != nullptr) {
        list.emplace_back(*single);
    } else if (const Tensor<pmt::Value>* previous = found->second.get_if<Tensor<pmt::Value>>(); previous != nullptr) {
        list.reserve(previous->size() + 1UZ);
        list.insert(list.end(), previous->begin(), previous->end());
    }
    list.emplace_back(std::move(map));
    found->second = pmt::Value(Tensor<pmt::Value>(std::move(list)));
}

} // namespace gr::dataset

#endif // GNURADIO_ALGORITHM_DATASET_INDEXEDMAPS_HPP
