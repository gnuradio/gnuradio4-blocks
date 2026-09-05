#ifndef GNURADIO_FEC_RECORD_SHAPE_HPP
#define GNURADIO_FEC_RECORD_SHAPE_HPP

#include <cstdint>
#include <format>
#include <iterator>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Tag.hpp>

namespace gr::blocks::fec::detail {

// What every adapter in this module does with a record apart from its own arithmetic: shape the output record,
// carry the input's facts onto it, and state the counters once the graph has stopped. One copy, because eleven
// blocks answering the same question differently is eleven answers to keep true.

//! A record's value under @p key, with the key's absence answered by @p fallback.
template<typename V>
[[nodiscard]] inline V metaOr(const property_map& map, const char* key, V fallback) {
    if (const auto entry = map.find(property_map::key_type(key)); entry != map.end()) {
        return entry->second.value_or(V(fallback));
    }
    return fallback;
}

//! The record's signal name, with the module's label standing in where it names none.
template<typename T>
[[nodiscard]] inline std::string originName(const DataSet<T>& record) {
    return record.signal_names.empty() ? std::string("fec") : record.signal_names[0UZ];
}

//! The record's metadata map, with an empty one standing in where it carries none.
template<typename T>
[[nodiscard]] inline property_map originMeta(const DataSet<T>& record) {
    return record.meta_information.empty() ? property_map{} : record.meta_information[0UZ];
}

//! Shape @p out as this module shapes an output record, stamping it with the name and facts of its origin.
template<typename T>
inline void carryOrigin(DataSet<T>& out, const std::string& name, const property_map& meta) {
    out.extents.push_back(static_cast<std::int32_t>(out.signal_values.size()));
    out.signal_names.emplace_back(name);
    out.timing_events.resize(1UZ);
    out.meta_information.resize(1UZ);
    out.meta_information[0UZ] = meta; // the record's facts carry through; an adapter has no status of its own
}

//! Shape @p out as this module shapes an output record, carrying @p record's facts onto it.
template<typename TOut, typename TIn>
inline void carry(DataSet<TOut>& out, const DataSet<TIn>& record) {
    carryOrigin(out, originName(record), originMeta(record));
}

//! One line of counters once the graph has stopped, in the shape every block of the module uses.
inline void report(std::string_view block, std::string_view name, std::span<const std::pair<std::string_view, std::uint64_t>> counters) {
    std::string line;
    for (const auto& [label, count] : counters) {
        if (count > 0ULL) {
            std::format_to(std::back_inserter(line), "{}{}: {}", line.empty() ? "" : ", ", label, count);
        }
    }
    if (!line.empty()) {
        std::println(stderr, "{} '{}': {}", block, name, line);
    }
}

} // namespace gr::blocks::fec::detail

#endif // GNURADIO_FEC_RECORD_SHAPE_HPP
