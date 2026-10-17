#ifndef GNURADIO_FILEIO_FD_IO_HPP
#define GNURADIO_FILEIO_FD_IO_HPP

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <climits>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <tuple>
#include <utility>

#include <fcntl.h>
#include <poll.h>
#include <sys/stat.h>
#include <unistd.h>

#if defined(__linux__)
#include <csignal>
#include <ctime>
#include <pthread.h>
#endif

#include <gnuradio-4.0/fileio/NamespaceCompatibility.hpp>

namespace gr::blocks::fileio {

namespace detail {

[[nodiscard]] inline std::string errnoText(int code) { return std::error_code(code, std::system_category()).message(); }

// the access mode of an open descriptor, or the errno of the failed query
struct DescriptorMode {
    int accessMode = 0;
    int error      = 0;
};

[[nodiscard]] inline DescriptorMode descriptorMode(int fd) noexcept {
    const int flags = ::fcntl(fd, F_GETFL);
    if (flags < 0) {
        return {.accessMode = 0, .error = errno};
    }
    return {.accessMode = flags & O_ACCMODE, .error = 0};
}

// Returns the events that are ready on the descriptor at once, without waiting.
[[nodiscard]] inline short readyEvents(int fd, short events) {
    ::pollfd item{.fd = fd, .events = events, .revents = 0};
    int      nReady = 0;
    do {
        nReady = ::poll(&item, 1, 0);
    } while (nReady < 0 && errno == EINTR);
    if (nReady < 0) {
        throw gr::exception(std::format("polling descriptor {} failed: {}", fd, errnoText(errno)));
    }
    return nReady == 0 ? static_cast<short>(0) : item.revents;
}

// whether the errno of a read or write on a non-blocking descriptor means that no data or room is ready
[[nodiscard]] constexpr bool wouldBlock(int code) noexcept {
#if EAGAIN == EWOULDBLOCK
    return code == EAGAIN;
#else
    return code == EAGAIN || code == EWOULDBLOCK;
#endif
}

// the bytes one read or write moved, or the errno of its failure
struct IoResult {
    std::size_t nBytes = 0UZ;
    int         error  = 0;
};

[[nodiscard]] inline IoResult readOnce(int fd, std::byte* data, std::size_t nBytes) noexcept {
    ::ssize_t nRead = 0;
    do {
        nRead = ::read(fd, data, nBytes);
    } while (nRead < 0 && errno == EINTR);
    if (nRead < 0) {
        return {.nBytes = 0UZ, .error = errno};
    }
    return {.nBytes = static_cast<std::size_t>(nRead), .error = 0};
}

#if defined(__linux__)
// Blocks SIGPIPE in the calling thread while it lives and restores the earlier mask at the end. A write to a pipe or
// socket without a reader raises SIGPIPE in the writing thread. discardRaised() takes that signal off the pending set
// before the mask is restored, unless SIGPIPE was pending already when the guard began.
class SigpipeGuard {
    sigset_t _sigpipe{};
    sigset_t _previousMask{};
    bool     _wasPending = false;

public:
    SigpipeGuard() noexcept {
        sigemptyset(&_sigpipe);
        sigaddset(&_sigpipe, SIGPIPE);
        sigset_t pending{};
        sigemptyset(&pending);
        _wasPending = sigpending(&pending) == 0 && sigismember(&pending, SIGPIPE) == 1;
        pthread_sigmask(SIG_BLOCK, &_sigpipe, &_previousMask);
    }
    SigpipeGuard(const SigpipeGuard&)            = delete;
    SigpipeGuard& operator=(const SigpipeGuard&) = delete;
    ~SigpipeGuard() { pthread_sigmask(SIG_SETMASK, &_previousMask, nullptr); }

    void discardRaised() noexcept {
        if (_wasPending) {
            return;
        }
        const ::timespec noWait{.tv_sec = 0, .tv_nsec = 0};
        while (sigtimedwait(&_sigpipe, nullptr, &noWait) < 0 && errno == EINTR) {
        }
    }
};
#else
class SigpipeGuard {
public:
    void discardRaised() noexcept {}
};
#endif

// Writes once and retries an interrupted write. A failure with EPIPE discards the SIGPIPE that the write raised.
[[nodiscard]] inline IoResult writeOnce(int fd, const std::byte* data, std::size_t nBytes, SigpipeGuard& guard) noexcept {
    ::ssize_t nWritten = 0;
    do {
        nWritten = ::write(fd, data, nBytes);
    } while (nWritten < 0 && errno == EINTR);
    if (nWritten < 0) {
        const int error = errno;
        if (error == EPIPE) {
            guard.discardRaised();
        }
        return {.nBytes = 0UZ, .error = error};
    }
    return {.nBytes = static_cast<std::size_t>(nWritten), .error = 0};
}

inline void closeDescriptor(int fd) {
    if (::close(fd) != 0 && errno != EINTR) {
        throw gr::exception(std::format("closing descriptor {} failed: {}", fd, errnoText(errno)));
    }
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::fileio::FdSource, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
struct FdSource : Block<FdSource<T>> {
    using Description = Doc<R""(@brief Reads raw samples from an open file descriptor and ends the stream at end of file.

The descriptor can be a pipe, a socket, a terminal, standard input or a regular file. The block takes `fd` at
start and checks that it is open for reading. The bytes are samples of `T` in host byte order, as `FdSink` and
`BasicFileSink` write them. A complex sample is its real part followed by its imaginary part.
The block reads only when the descriptor reports data or end of file. Otherwise it returns to the scheduler
without samples. It reports that it waits on input. The scheduler calls it again. A writer that keeps the
descriptor open and writes nothing keeps the stream open. A stop request ends it.
End of file ends the stream. The block drops the bytes of an incomplete last sample and counts them in
`n_dropped_bytes`. A failed read ends the stream with an error that names the descriptor.
The block closes the descriptor at stop when `close_on_stop` is true.
The block needs a `poll` that returns when a worker thread calls it. Under node, Emscripten's `poll` from a
worker thread waits until the main thread returns to its event loop.
)"">;

    PortOut<T> out;

    Annotated<int, "fd", Doc<"open file descriptor to read">, Visible>                                 fd              = 0;
    Annotated<bool, "close_on_stop", Doc<"close the descriptor at stop">, Visible>                     close_on_stop   = false;
    Annotated<gr::Size_t, "n_samples", Doc<"samples read, modulo 2^32">>                               n_samples       = 0U;
    Annotated<gr::Size_t, "n_dropped_bytes", Doc<"bytes of an incomplete last sample at end of file">> n_dropped_bytes = 0U;

    GR_MAKE_REFLECTABLE(FdSource, out, fd, close_on_stop, n_samples, n_dropped_bytes);

    int                              _fd = -1;
    std::array<std::byte, sizeof(T)> _partial{};
    std::size_t                      _nPartial = 0UZ;

    void start() {
        _fd             = -1;
        _nPartial       = 0UZ;
        n_samples       = 0U;
        n_dropped_bytes = 0U;
        std::ignore     = this->updateActiveParameters({"n_samples", "n_dropped_bytes"});

        const detail::DescriptorMode mode = detail::descriptorMode(fd.value);
        if (mode.error != 0) {
            throw gr::exception(std::format("descriptor {} is not open: {}", fd.value, detail::errnoText(mode.error)));
        }
        if (mode.accessMode == O_WRONLY) {
            throw gr::exception(std::format("descriptor {} is open for writing only", fd.value));
        }
        _fd = fd.value;
    }

    void stop() {
        const int descriptor = std::exchange(_fd, -1);
        if (close_on_stop.value && descriptor >= 0) {
            detail::closeDescriptor(descriptor);
        }
    }

    [[nodiscard]] work::Status processBulk(OutputSpanLike auto& dataOut) {
        if (dataOut.empty()) {
            return work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        const short revents = detail::readyEvents(_fd, POLLIN);
        if (revents == 0) {
            dataOut.publish(0UZ);
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        if ((revents & POLLNVAL) != 0) {
            throw gr::exception(std::format("descriptor {} is not open", _fd));
        }

        // the bytes of an incomplete sample from the previous read go first
        std::byte* bytes = reinterpret_cast<std::byte*>(dataOut.data());
        std::memcpy(bytes, _partial.data(), _nPartial);
        const detail::IoResult result = detail::readOnce(_fd, bytes + _nPartial, dataOut.size() * sizeof(T) - _nPartial);
        if (detail::wouldBlock(result.error)) {
            dataOut.publish(0UZ);
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        if (result.error != 0) {
            throw gr::exception(std::format("reading descriptor {} failed: {}", _fd, detail::errnoText(result.error)));
        }
        if (result.nBytes == 0UZ) {
            if (_nPartial > 0UZ) {
                n_dropped_bytes = static_cast<gr::Size_t>(_nPartial);
                _nPartial       = 0UZ;
                std::ignore     = this->updateActiveParameters({"n_dropped_bytes"});
            }
            dataOut.publish(0UZ);
            return work::Status::DONE;
        }

        const std::size_t nBytes   = _nPartial + result.nBytes;
        const std::size_t nSamples = nBytes / sizeof(T);
        _nPartial                  = nBytes % sizeof(T);
        std::memcpy(_partial.data(), bytes + nSamples * sizeof(T), _nPartial);
        dataOut.publish(nSamples);
        if (nSamples > 0UZ) {
            n_samples   = n_samples.value + static_cast<gr::Size_t>(nSamples);
            std::ignore = this->updateActiveParameters({"n_samples"});
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::fileio::FdSink, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
struct FdSink : Block<FdSink<T>> {
    using Description = Doc<R""(@brief Writes the input samples as raw bytes to an open file descriptor.

The descriptor can be a pipe, a socket, a terminal, standard output or a regular file. The block takes `fd` at
start and checks that it is open for writing. It writes each sample of `T` in host byte order. A complex sample
is its real part followed by its imaginary part.
The block writes every sample. It writes only when the descriptor reports room. Otherwise it returns to the
scheduler. It reports that it waits on output. The scheduler calls it again. A single write to a descriptor
other than a regular file takes at most `PIPE_BUF` bytes. A pipe that reports room accepts that many bytes at
once. A terminal or a socket that reports room can still hold the write in the kernel until its reader takes
the data. The block retries a short write and keeps the part of a sample it wrote for the next call.
A failed write ends the stream with an error that names the descriptor. A closed reader (`EPIPE`) is such a
failure. The samples that earlier writes of the failing call delivered are counted in `n_samples`.
On Linux the block blocks `SIGPIPE` in the calling thread while it writes and discards the signal that
a write raises. The process keeps running. On other systems the process's own `SIGPIPE` handling applies.
The block closes the descriptor at stop when `close_on_stop` is true.
The block needs a `poll` that returns when a worker thread calls it. Under node, Emscripten's `poll` from a
worker thread waits until the main thread returns to its event loop.
)"">;

    PortIn<T> in;

    Annotated<int, "fd", Doc<"open file descriptor to write">, Visible>            fd            = 1;
    Annotated<bool, "close_on_stop", Doc<"close the descriptor at stop">, Visible> close_on_stop = false;
    Annotated<gr::Size_t, "n_samples", Doc<"samples written, modulo 2^32">>        n_samples     = 0U;

    GR_MAKE_REFLECTABLE(FdSink, in, fd, close_on_stop, n_samples);

    int         _fd          = -1;
    std::size_t _writeLimit  = std::numeric_limits<std::size_t>::max();
    std::size_t _nSampleHead = 0UZ; // bytes of the first input sample that an earlier call wrote

    void start() {
        _fd          = -1;
        _nSampleHead = 0UZ;
        n_samples    = 0U;
        std::ignore  = this->updateActiveParameters({"n_samples"});

        const detail::DescriptorMode mode = detail::descriptorMode(fd.value);
        if (mode.error != 0) {
            throw gr::exception(std::format("descriptor {} is not open: {}", fd.value, detail::errnoText(mode.error)));
        }
        if (mode.accessMode == O_RDONLY) {
            throw gr::exception(std::format("descriptor {} is open for reading only", fd.value));
        }
        struct ::stat status {};
        const bool    regularFile = ::fstat(fd.value, &status) == 0 && S_ISREG(status.st_mode);
        _writeLimit               = regularFile ? std::numeric_limits<std::size_t>::max() : static_cast<std::size_t>(PIPE_BUF);
        _fd                       = fd.value;
    }

    void stop() {
        const int descriptor = std::exchange(_fd, -1);
        if (close_on_stop.value && descriptor >= 0) {
            detail::closeDescriptor(descriptor);
        }
    }

    // Consumes the whole samples among the first `nDone` bytes of the input, counts them and stores the bytes of a split
    // sample in `_nSampleHead`.
    void finishCall(InputSpanLike auto& dataIn, std::size_t nDone) {
        const std::size_t nSamples = nDone / sizeof(T);
        _nSampleHead               = nDone % sizeof(T);
        if (!dataIn.consume(nSamples)) {
            throw gr::exception("could not consume input samples");
        }
        if (nSamples > 0UZ) {
            n_samples   = n_samples.value + static_cast<gr::Size_t>(nSamples);
            std::ignore = this->updateActiveParameters({"n_samples"});
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& dataIn) {
        if (dataIn.empty()) {
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        const auto                          bytes  = std::as_bytes(std::span(dataIn.data(), dataIn.size()));
        std::size_t                         nDone  = _nSampleHead;
        bool                                noRoom = false;
        std::optional<detail::SigpipeGuard> guard;
        while (nDone < bytes.size()) {
            const short revents = detail::readyEvents(_fd, POLLOUT);
            if (revents == 0) {
                noRoom = true;
                break;
            }
            // a failed call still counts and consumes the samples its earlier writes delivered
            const auto fail = [&](std::string message) {
                finishCall(dataIn, nDone);
                throw gr::exception(std::move(message));
            };
            if ((revents & POLLNVAL) != 0) {
                fail(std::format("descriptor {} is not open", _fd));
            }
            if (!guard.has_value()) {
                guard.emplace();
            }
            const detail::IoResult result = detail::writeOnce(_fd, bytes.data() + nDone, std::min(bytes.size() - nDone, _writeLimit), *guard);
            if (detail::wouldBlock(result.error)) {
                noRoom = true;
                break;
            }
            if (result.error == EPIPE) {
                fail(std::format("the reader of descriptor {} is closed: {}", _fd, detail::errnoText(result.error)));
            }
            if (result.error != 0) {
                fail(std::format("writing descriptor {} failed: {}", _fd, detail::errnoText(result.error)));
            }
            nDone += result.nBytes;
        }

        if (nDone / sizeof(T) == 0UZ && noRoom) {
            // the descriptor is the block's output, and the scheduler counts this call as a wait on it
            _nSampleHead = nDone % sizeof(T);
            return work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        finishCall(dataIn, nDone);
        return work::Status::OK;
    }
};

} // namespace gr::blocks::fileio

#endif // GNURADIO_FILEIO_FD_IO_HPP
