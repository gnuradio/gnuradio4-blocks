#ifndef GNURADIO_FILEIO_TEST_FILE_OPEN_COUNTER_HPP
#define GNURADIO_FILEIO_TEST_FILE_OPEN_COUNTER_HPP

#if defined(__linux__) && !defined(__EMSCRIPTEN__)

#include <array>
#include <cerrno>
#include <cstddef>
#include <filesystem>
#include <format>
#include <stdexcept>
#include <system_error>

#include <sys/inotify.h>
#include <unistd.h>

namespace gr::blocks::fileio::test {

// counts the opens of one file by any thread of any process, from construction on, through the kernel's inotify events;
// the watch takes the closes too, because the kernel merges an event into an identical one queued just before it, and
// two opens with no close between them count as one
struct FileOpenCounter {
    int _fd = ::inotify_init1(IN_NONBLOCK | IN_CLOEXEC);

    explicit FileOpenCounter(const std::filesystem::path& filePath) {
        if (_fd < 0 || ::inotify_add_watch(_fd, filePath.c_str(), IN_OPEN | IN_CLOSE) < 0) {
            const std::error_code reason(errno, std::generic_category());
            if (_fd >= 0) {
                ::close(_fd);
            }
            throw std::runtime_error(std::format("cannot watch '{}' for opens: {}", filePath.string(), reason.message()));
        }
    }
    FileOpenCounter(const FileOpenCounter&)            = delete;
    FileOpenCounter& operator=(const FileOpenCounter&) = delete;
    ~FileOpenCounter() { ::close(_fd); }

    // the opens since construction or since the previous call
    [[nodiscard]] std::size_t count() const {
        std::size_t                                   nOpens = 0UZ;
        alignas(inotify_event) std::array<char, 4096> events{};
        for (ssize_t nBytes = ::read(_fd, events.data(), events.size()); nBytes > 0; nBytes = ::read(_fd, events.data(), events.size())) {
            for (std::size_t offset = 0UZ; offset < static_cast<std::size_t>(nBytes);) {
                const auto* event = reinterpret_cast<const inotify_event*>(events.data() + offset);
                if ((event->mask & IN_OPEN) != 0U) {
                    ++nOpens;
                }
                offset += sizeof(inotify_event) + event->len;
            }
        }
        return nOpens;
    }
};

} // namespace gr::blocks::fileio::test

#endif // __linux__ && !__EMSCRIPTEN__

#endif // GNURADIO_FILEIO_TEST_FILE_OPEN_COUNTER_HPP
