#ifndef GR_SDR_RECEIVE_LEASE_HPP
#define GR_SDR_RECEIVE_LEASE_HPP

#include <cerrno>
#include <cstdint>
#include <expected>
#include <format>
#include <string>
#include <tuple>
#include <utility>

#if (defined(__unix__) || defined(__APPLE__)) && !defined(__EMSCRIPTEN__) && !defined(GR_SDR_DISABLE_RECEIVE_LEASE) && __has_include(<sys/file.h>)
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#if defined(O_DIRECTORY) && defined(O_CLOEXEC) && defined(O_NOFOLLOW) && defined(LOCK_EX) && defined(LOCK_NB)
#define GR_SDR_POSIX_RECEIVE_LEASE 1
#endif
#endif

namespace gr::blocks::sdr {

// Cooperative RX exclusion for one opened hardware identity, across channels.
// Scope: same host, uid and /tmp namespace. Does not arbitrate TX or device open.
class ReceiveLease {
    int _fd = -1;
    bool _backendUnavailable = false;

public:
#ifdef GR_SDR_POSIX_RECEIVE_LEASE
    static constexpr bool supported = true;
#else
    static constexpr bool supported = false;
#endif
    ReceiveLease() = default;
    ReceiveLease(const ReceiveLease&) = delete;
    ReceiveLease& operator=(const ReceiveLease&) = delete;
    ~ReceiveLease() { std::ignore = reset(); }

    // Intentional retained descriptor: no in-process recovery after uncertain
    // driver cleanup. Exclusion lasts until exit or exec (CLOEXEC), without a stop claim.
    void quarantine() noexcept { _fd = -1; }
    bool held() const noexcept { return _fd >= 0; }
    bool backendUnavailable() const noexcept { return _backendUnavailable; }

    bool reset() noexcept {
#ifdef GR_SDR_POSIX_RECEIVE_LEASE
        if (_fd >= 0) { return ::close(std::exchange(_fd, -1)) == 0; }
#endif
        return true;
    }

    std::expected<void, std::string> acquire(const std::string& identity) {
        _backendUnavailable = !supported;
        if (_fd >= 0) { return std::unexpected("receive lease already held"); }
        if (identity.empty()) { return std::unexpected("canonical hardware identity unavailable"); }
#ifdef GR_SDR_POSIX_RECEIVE_LEASE
        std::uint64_t hash = 14695981039346656037ULL;
        for (char c : identity) { hash = (hash ^ static_cast<unsigned char>(c)) * 1099511628211ULL; }
        const auto directory = std::format("/tmp/gr4-soapy-rx-{}", ::getuid());
        std::ignore = ::mkdir(directory.c_str(), 0700);
        const int dir = ::open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
        struct stat info{};
        if (dir < 0) { return std::unexpected("cannot open receive lock directory"); }
        if (::fstat(dir, &info) != 0 || info.st_uid != ::getuid() || (info.st_mode & 0077) != 0) {
            ::close(dir);
            return std::unexpected("unsafe receive lock directory");
        }
        _fd = ::openat(dir, std::format("{:016x}.lock", hash).c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
        ::close(dir);
        if (_fd < 0) { return std::unexpected("cannot open receive lock"); }
        if (::fstat(_fd, &info) != 0 || !S_ISREG(info.st_mode) || info.st_uid != ::getuid() || info.st_nlink != 1) {
            std::ignore = reset();
            return std::unexpected("unsafe receive lock file");
        }
        if (::flock(_fd, LOCK_EX | LOCK_NB) != 0) {
            const int failure = errno;
            _backendUnavailable = failure == ENOSYS || failure == ENOTSUP || failure == EOPNOTSUPP;
            std::ignore = reset();
            return std::unexpected(_backendUnavailable ? "receive ownership backend unavailable" : "receive ownership unavailable: " + identity);
        }
        // Never unlink: all cooperating clients must lock the same inode.
        // Stable hash collisions cause false contention, never simultaneous RX.
        return {};
#else
        return std::unexpected("receive ownership backend unavailable");
#endif
    }
};

} // namespace gr::blocks::sdr
#undef GR_SDR_POSIX_RECEIVE_LEASE
#endif
