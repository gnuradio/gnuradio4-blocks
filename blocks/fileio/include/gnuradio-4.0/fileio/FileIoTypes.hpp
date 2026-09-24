#ifndef GNURADIO_FILEIO_TYPES_HPP
#define GNURADIO_FILEIO_TYPES_HPP

#include <filesystem>

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

} // namespace detail

} // namespace gr::blocks::fileio

#endif // GNURADIO_FILEIO_TYPES_HPP
