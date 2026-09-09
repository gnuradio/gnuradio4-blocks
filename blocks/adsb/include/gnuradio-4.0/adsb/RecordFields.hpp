#ifndef GNURADIO_ADSB_RECORD_FIELDS_HPP
#define GNURADIO_ADSB_RECORD_FIELDS_HPP

#include <cstdint>
#include <memory_resource>
#include <optional>
#include <string>
#include <string_view>

#include <gnuradio-4.0/Tag.hpp>

namespace gr::blocks::adsb::detail {

// How a record's metadata is read in this module: at the exact declared type, so that a value of the wrong type
// reads as absent rather than as a number nobody wrote. The decoder and the printer share it because a reader
// that disagreed with the writer about a type is the failure this module is most exposed to.

/// @brief Reads a key of the exact type @p T, which is how a metadata value is read where a wrong type is absent.
template<typename T>
[[nodiscard]] inline std::optional<T> read(const property_map& meta, std::string_view key) {
    const auto entry = meta.find(property_map::key_type(key));
    if (entry == meta.end()) {
        return std::nullopt;
    }
    const T* value = entry->second.template get_if<T>();
    return value == nullptr ? std::nullopt : std::optional<T>{*value};
}

/// @brief The same for a string, which a metadata map holds in its own allocator's string type.
[[nodiscard]] inline std::optional<std::string> readText(const property_map& meta, std::string_view key) {
    const std::optional<std::pmr::string> value = read<std::pmr::string>(meta, key);
    return value.has_value() ? std::optional<std::string>{std::string(*value)} : std::nullopt;
}

/// @brief Seconds into the stream the frame started, from the two keys the framer writes, or nothing without them.
[[nodiscard]] inline std::optional<double> timeOf(const property_map& meta) {
    const std::optional<std::uint64_t> start = read<std::uint64_t>(meta, "sample_start");
    const std::optional<float>         rate  = read<float>(meta, "sample_rate");
    if (!start.has_value() || !rate.has_value() || !(*rate > 0.f)) {
        return std::nullopt;
    }
    return static_cast<double>(*start) / static_cast<double>(*rate);
}

} // namespace gr::blocks::adsb::detail

#endif // GNURADIO_ADSB_RECORD_FIELDS_HPP
