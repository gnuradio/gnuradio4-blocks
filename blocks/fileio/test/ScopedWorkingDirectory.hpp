#ifndef GNURADIO_FILEIO_TEST_SCOPED_WORKING_DIRECTORY_HPP
#define GNURADIO_FILEIO_TEST_SCOPED_WORKING_DIRECTORY_HPP

#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <format>
#include <print>
#include <random>
#include <stdexcept>
#include <string_view>
#include <system_error>

namespace gr::blocks::fileio::test {

// a new directory under the temporary directory, unique because it is created rather than named: a name already
// taken is retried with new randomness
[[nodiscard]] inline std::filesystem::path createTemporaryDirectory(std::string_view prefix, std::size_t maxAttempts = 128UZ) {
    std::random_device random;
    for (std::size_t attempt = 0UZ; attempt < maxAttempts; ++attempt) {
        const std::filesystem::path candidate = std::filesystem::temp_directory_path() / std::format("{}-{:08x}{:08x}", prefix, random(), random());
        std::error_code             ec;
        if (std::filesystem::create_directory(candidate, ec)) {
            return candidate;
        }
        if (ec && ec != std::errc::file_exists) {
            throw std::runtime_error(std::format("cannot create '{}': {}", candidate.string(), ec.message()));
        }
    }
    throw std::runtime_error(std::format("no free name for a '{}' directory in {} attempts", prefix, maxAttempts));
}

// sets the working directory to a new temporary directory, and restores it and removes the directory on exit; a
// failure of either is printed on standard error
struct ScopedWorkingDirectory {
    std::filesystem::path previous  = std::filesystem::current_path();
    std::filesystem::path directory = createTemporaryDirectory("gr4-fileio-cwd");

    ScopedWorkingDirectory() { std::filesystem::current_path(directory); }
    ScopedWorkingDirectory(const ScopedWorkingDirectory&)            = delete;
    ScopedWorkingDirectory& operator=(const ScopedWorkingDirectory&) = delete;
    ~ScopedWorkingDirectory() {
        std::error_code ec;
        std::filesystem::current_path(previous, ec);
        if (ec) {
            std::println(stderr, "ScopedWorkingDirectory: cannot return to '{}': {}", previous.string(), ec.message());
        }
        std::filesystem::remove_all(directory, ec);
        if (ec) {
            std::println(stderr, "ScopedWorkingDirectory: cannot remove '{}': {}", directory.string(), ec.message());
        }
    }
};

} // namespace gr::blocks::fileio::test

#endif // GNURADIO_FILEIO_TEST_SCOPED_WORKING_DIRECTORY_HPP
