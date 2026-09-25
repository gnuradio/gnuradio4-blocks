#ifndef GNURADIO_FILEIO_TYPES_HPP
#define GNURADIO_FILEIO_TYPES_HPP

#include <cerrno>
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

// the directory of a file name: its parent path, or the working directory for a bare name such as "tone.f32"
[[nodiscard]] inline std::filesystem::path parentDirectory(const std::filesystem::path& filePath) { return filePath.has_parent_path() ? filePath.parent_path() : std::filesystem::current_path(); }

inline void ensureDirectoryExists(const std::filesystem::path& filePath) {
    if (filePath.has_parent_path()) {
        std::filesystem::create_directories(filePath.parent_path());
    }
}

// the reason a local file cannot be read, taken from an open and a first read of it, or nothing when both succeed
[[nodiscard]] inline std::optional<std::string> unreadableFileReason(const std::filesystem::path& filePath) {
    errno = 0;
    std::ifstream in(filePath, std::ios::binary);
    if (!in.is_open()) {
        const std::error_code reason(errno, std::generic_category());
        return reason ? std::format("cannot open '{}' for reading: {}", filePath.string(), reason.message()) : std::format("cannot open '{}' for reading", filePath.string());
    }
    errno = 0;
    if (in.peek() == std::char_traits<char>::eof() && errno != 0) { // a directory opens for reading on most systems and fails at the first read
        return std::format("cannot read '{}': {}", filePath.string(), std::error_code(errno, std::generic_category()).message());
    }
    return std::nullopt;
}

} // namespace detail

} // namespace gr::blocks::fileio

#endif // GNURADIO_FILEIO_TYPES_HPP
