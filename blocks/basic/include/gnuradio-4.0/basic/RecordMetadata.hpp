#ifndef GNURADIO_RECORDMETADATA_HPP
#define GNURADIO_RECORDMETADATA_HPP

#include <algorithm>
#include <cctype>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Tag.hpp>

namespace gr::blocks::basic {

// The record-metadata vocabulary: which keys a record's metadata map may carry and what type each of them holds. It
// has a header of its own because every boundary that a record's metadata crosses has to agree on it — the record to
// packet conversion, the packet to record conversion and the transport between them — and one table spelled three
// times is three tables that drift. The readers the three boundary blocks share sit here for the same reason.

namespace detail {

/// @brief The key without the `gr:` prefix the framework carries internally, which the short form compares equal to.
[[nodiscard]] inline constexpr std::string_view shortKey(std::string_view key) noexcept {
    constexpr std::string_view prefix = "gr:";
    return key.starts_with(prefix) ? key.substr(prefix.size()) : key;
}

/// @brief ASCII case-insensitive equality, which is what an axis name comparison needs and no more.
[[nodiscard]] inline bool equalIgnoringCase(std::string_view left, std::string_view right) noexcept {
    return std::ranges::equal(left, right, [](char a, char b) { return std::tolower(static_cast<unsigned char>(a)) == std::tolower(static_cast<unsigned char>(b)); });
}

/// @brief Look a short reserved key up in `map`, accepting the `gr:`-prefixed spelling of the same key.
[[nodiscard]] inline const pmt::Value* find(const property_map& map, std::string_view key) noexcept {
    if (const auto it = map.find(key); it != map.end()) {
        return &it->second;
    }
    for (const auto& [candidate, value] : map) {
        if (shortKey(std::string_view(candidate)) == key) {
            return &value;
        }
    }
    return nullptr;
}

/// @brief Insert a key the block derived, counting in @p nOverridden the copied value it displaces when the two differ.
inline void putDerived(property_map& map, std::string_view key, pmt::Value value, std::uint64_t& nOverridden) {
    const auto it = map.find(key);
    if (it == map.end()) {
        map.emplace(property_map::key_type(key), std::move(value));
        return;
    }
    if (!(it->second == value)) {
        ++nOverridden;
    }
    it->second = std::move(value);
}

/// @brief Read the record's rate off its time axis, or nothing when any precondition of that route fails.
template<typename T>
[[nodiscard]] inline std::optional<float> axisRate(const DataSet<T>& ds, std::size_t nSamples, double tolerance) {
    if constexpr (!std::floating_point<T>) { // an integer axis collapses to a constant and a complex one has no defined reading
        return std::nullopt;
    } else {
        if (nSamples < 2UZ || ds.axis_values.empty() || ds.axis_values[0UZ].size() != nSamples) {
            return std::nullopt;
        }
        if (ds.axis_names.empty() || !equalIgnoringCase(ds.axis_names[0UZ], "time")) {
            return std::nullopt;
        }
        const std::vector<T>& axis  = ds.axis_values[0UZ];
        const double          first = static_cast<double>(axis[0UZ]);
        // the endpoint form, whose comparison against the first step is the uniformity test, so one of the two is free
        const double step = (static_cast<double>(axis[nSamples - 1UZ]) - first) / static_cast<double>(nSamples - 1UZ);
        if (!std::isfinite(step) || step <= 0.0) {
            return std::nullopt;
        }
        const double firstStep = static_cast<double>(axis[1UZ]) - first;
        if (std::abs(firstStep - step) > tolerance * step) {
            return std::nullopt; // not a constant rate, which is a legal record and not an error
        }
        return static_cast<float>(1.0 / step);
    }
}

} // namespace detail

namespace detail::packet {

/// @brief The declared value type of a record-metadata vocabulary key, as the group of keys sharing it.
enum class VocabularyType : std::uint8_t { NotVocabulary, Float, Double, String, Size, UInt64, Bool };

/// @brief The type `key` is declared to hold in the record-metadata vocabulary, or `NotVocabulary` when no vocabulary
/// key is spelled that way. Nine keys take a reserved stream key's spelling and its declared type; twelve are the
/// vocabulary's own. Every type is fixed-width or a string, because a metadata map that crosses a process is a wire
/// format whether or not anybody calls it one.
[[nodiscard]] inline VocabularyType vocabularyType(std::string_view key) noexcept {
    using enum VocabularyType;
    if (key == tag::SAMPLE_RATE.shortKey() || key == tag::SIGNAL_MIN.shortKey() || key == tag::SIGNAL_MAX.shortKey()) {
        return Float;
    }
    if (key == tag::FREQUENCY.shortKey()) {
        return Double;
    }
    if (key == tag::SIGNAL_NAME.shortKey() || key == tag::SIGNAL_QUANTITY.shortKey() || key == tag::SIGNAL_UNIT.shortKey() || key == tag::TRIGGER_NAME.shortKey()) {
        return String;
    }
    if (key == tag::N_DROPPED_SAMPLES.shortKey()) {
        return Size;
    }
    if (key == "protocol" || key == "discontinuity" || key == "discard_reason" || key == "source_id") {
        return String;
    }
    if (key == "schema_version" || key == "corrected_errors" || key == "uncorrectable_errors" || key == "dropped_events") {
        return Size;
    }
    // `header_corrected_errors` is deliberately not folded into `corrected_errors`, which accumulates across a chain:
    // a header correction and a payload correction are corrections in different fields, and summing them would make a
    // record whose header took three errors look like one whose payload did. It is declared UInt64 rather than Size
    // because that is the width the header layout that writes it uses for a per-packet count.
    if (key == "sample_start" || key == "sequence" || key == "header_corrected_errors") {
        return UInt64;
    }
    if (key == "crc_ok") {
        return Bool;
    }
    return NotVocabulary;
}

/// @brief Whether `value` holds the type `type` names. A key outside the vocabulary imposes nothing and always passes.
[[nodiscard]] inline bool holdsVocabularyType(VocabularyType type, const pmt::Value& value) noexcept {
    using enum VocabularyType;
    switch (type) {
    case Float: return value.get_if<float>() != nullptr;
    case Double: return value.get_if<double>() != nullptr;
    case String: return value.get_if<std::pmr::string>() != nullptr;
    case Size: return value.get_if<gr::Size_t>() != nullptr;
    case UInt64: return value.get_if<std::uint64_t>() != nullptr;
    case Bool: return value.get_if<bool>() != nullptr;
    case NotVocabulary: break;
    }
    return true;
}

} // namespace detail::packet

} // namespace gr::blocks::basic

#endif // GNURADIO_RECORDMETADATA_HPP
