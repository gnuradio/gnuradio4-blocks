#ifndef GNURADIO_NETWORK_RECORDVOCABULARY_HPP
#define GNURADIO_NETWORK_RECORDVOCABULARY_HPP

#include <cstdint>
#include <string_view>

#include <gnuradio-4.0/basic/RecordMetadata.hpp>

namespace gr::blocks::network::detail {

// The record-metadata vocabulary and its declared types are the basic module's, reused rather than restated: two
// tables for one vocabulary is the drift these blocks exist to avoid, and both ends of a transport have to agree on
// the same table as the blocks that convert records into the packets it carries. Every transport in this module reads
// the vocabulary through this header, so the module names it once for the same reason.

using gr::blocks::basic::detail::shortKey;
using gr::blocks::basic::detail::packet::holdsVocabularyType;
using gr::blocks::basic::detail::packet::vocabularyType;

/// @brief Vocabulary keys of @p map whose value type disagrees with the declaration.
///
/// Counted and never dropped. At a record boundary a wrongly typed key is dropped because an absent key at least
/// reads as absent, but across a transport the value's author is in another process and cannot be told: dropping it
/// would erase the only evidence that a peer is misconfigured.
[[nodiscard]] inline std::uint64_t countMistypedKeys(const property_map& map) noexcept {
    std::uint64_t mistyped = 0ULL;
    for (const auto& [key, value] : map) {
        if (!holdsVocabularyType(vocabularyType(shortKey(std::string_view(key))), value)) {
            ++mistyped;
        }
    }
    return mistyped;
}

} // namespace gr::blocks::network::detail

#endif // GNURADIO_NETWORK_RECORDVOCABULARY_HPP
