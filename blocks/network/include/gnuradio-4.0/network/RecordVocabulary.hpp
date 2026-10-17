#ifndef GNURADIO_NETWORK_RECORDVOCABULARY_HPP
#define GNURADIO_NETWORK_RECORDVOCABULARY_HPP

#include <cstdint>
#include <string_view>

#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/algorithm/dataset/RecordVocabulary.hpp>

namespace gr::blocks::network::detail {

// The transports of this module check a metadata map against the record-metadata vocabulary in `gr::dataset`.

/// @brief Vocabulary keys of @p map whose value type disagrees with the declaration.
///
/// Counted and never dropped. At a record boundary a wrongly typed key is dropped because an absent key at least
/// reads as absent, but across a transport the value's author is in another process and cannot be told: dropping it
/// would erase the only evidence that a peer is misconfigured.
[[nodiscard]] inline std::uint64_t countMistypedKeys(const property_map& map) noexcept {
    std::uint64_t mistyped = 0ULL;
    for (const auto& [key, value] : map) {
        if (!gr::dataset::holdsVocabularyType(gr::dataset::vocabularyType(gr::dataset::shortKey(std::string_view(key))), value)) {
            ++mistyped;
        }
    }
    return mistyped;
}

} // namespace gr::blocks::network::detail

#endif // GNURADIO_NETWORK_RECORDVOCABULARY_HPP
