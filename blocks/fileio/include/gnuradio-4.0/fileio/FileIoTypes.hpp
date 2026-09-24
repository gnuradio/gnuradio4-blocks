#ifndef GNURADIO_FILEIO_TYPES_HPP
#define GNURADIO_FILEIO_TYPES_HPP

#include <filesystem>
#include <format>
#include <fstream>
#include <optional>
#include <string>
#include <system_error>

#include <gnuradio-4.0/fileio/NamespaceCompatibility.hpp>

namespace gr::blocks::fileio {

enum class Mode { overwrite, append, multi };

namespace detail {

// the directory of a file name: its parent path, or "." for a bare name such as "tone.f32" whose parent path is empty
[[nodiscard]] inline std::filesystem::path parentDirectory(const std::filesystem::path& filePath) { return filePath.has_parent_path() ? filePath.parent_path() : std::filesystem::path("."); }

inline void ensureDirectoryExists(const std::filesystem::path& filePath) {
    if (filePath.has_parent_path()) {
        std::filesystem::create_directories(filePath.parent_path());
    }
}

// the reason a local file cannot be read, or nothing when it opens for reading
[[nodiscard]] inline std::optional<std::string> unreadableFileReason(const std::filesystem::path& filePath) {
    std::error_code ec;
    const auto      status = std::filesystem::status(filePath, ec);
    if (status.type() == std::filesystem::file_type::not_found) {
        return std::format("file '{}' does not exist", filePath.string());
    }
    if (ec) {
        return std::format("cannot read '{}': {}", filePath.string(), ec.message());
    }
    if (!std::filesystem::is_regular_file(status)) {
        return std::format("'{}' is not a regular file", filePath.string());
    }
    if (!std::ifstream(filePath, std::ios::binary).is_open()) {
        return std::format("cannot open '{}' for reading", filePath.string());
    }
    return std::nullopt;
}

} // namespace detail

} // namespace gr::blocks::fileio

#endif // GNURADIO_FILEIO_TYPES_HPP
