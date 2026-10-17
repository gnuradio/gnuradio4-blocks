#ifndef GNURADIO_ALGORITHM_DATASET_RECORDVOCABULARY_HPP
#define GNURADIO_ALGORITHM_DATASET_RECORDVOCABULARY_HPP

#include <cstdint>
#include <string>
#include <string_view>

#include <gnuradio-4.0/Tag.hpp>

namespace gr::dataset {

// The record-metadata vocabulary: the keys a record's metadata map may carry and the type each key holds.

/// @brief The key without the `gr:` prefix the framework carries internally, which the short form compares equal to.
[[nodiscard]] inline constexpr std::string_view shortKey(std::string_view key) noexcept {
    constexpr std::string_view prefix = "gr:";
    return key.starts_with(prefix) ? key.substr(prefix.size()) : key;
}

/// @brief The declared value type of a record-metadata vocabulary key, as the group of keys sharing it.
enum class VocabularyType : std::uint8_t { NotVocabulary, Float, Double, String, Size, UInt64, Bool };

/// @brief The type `key` is declared to hold in the record-metadata vocabulary, or `NotVocabulary` when no vocabulary
/// key is spelled that way. Nine keys take a reserved stream key's spelling and its declared type; twelve are the
/// vocabulary's own. Every type is fixed-width or a string, because a metadata map that crosses a process boundary is
/// a wire format.
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

} // namespace gr::dataset

#endif // GNURADIO_ALGORITHM_DATASET_RECORDVOCABULARY_HPP
