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
#include <gnuradio-4.0/algorithm/dataset/RecordVocabulary.hpp>

namespace gr::blocks::basic {

// Helpers for record metadata: a reserved-key lookup that accepts the `gr:` prefix, a derived-key insert that counts
// what it overrides, the rate read off a record's time axis, and the case-insensitive compare that read uses.

namespace detail {

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
        if (gr::dataset::shortKey(std::string_view(candidate)) == key) {
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

} // namespace gr::blocks::basic

#endif // GNURADIO_RECORDMETADATA_HPP
