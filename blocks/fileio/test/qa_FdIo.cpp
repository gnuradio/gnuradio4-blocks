#include <boost/ut.hpp>

#include <gnuradio-4.0/fileio/FdIo.hpp>

#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/GraphTestBlocks.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <format>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <thread>
#include <tuple>
#include <type_traits>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#if defined(__linux__)
#include <csignal>
#include <sys/ioctl.h>
#include <sys/syscall.h>

namespace {
// Plays the reader of a pipe at a fixed point of the sink's run: when the writes to the watched descriptor have
// delivered `nBytes`, it reads exactly those bytes from the read end and closes the read end. The sink's next write
// then fails with EPIPE, as the write of a reader that took `nBytes` and left would.
struct ClosingReader {
    std::atomic<int>       writeEnd{-1}; // the descriptor to watch
    int*                   readEnd  = nullptr;
    std::size_t            nBytes   = 0UZ;
    std::size_t            nWritten = 0UZ;
    std::vector<std::byte> taken;
    bool                   closed = false;

    void arm(int* readEndToClose, int writeEndToWatch, std::size_t nBytesToTake) {
        readEnd  = readEndToClose;
        nBytes   = nBytesToTake;
        nWritten = 0UZ;
        taken.clear();
        closed = false;
        writeEnd.store(writeEndToWatch);
    }

    void afterWrite(std::size_t nDelivered) {
        nWritten += nDelivered;
        if (closed || nWritten < nBytes) {
            return;
        }
        taken.resize(nBytes);
        std::size_t nRead = 0UZ;
        while (nRead < nBytes) {
            const long n = ::syscall(SYS_read, *readEnd, taken.data() + nRead, nBytes - nRead);
            if (n <= 0) {
                break;
            }
            nRead += static_cast<std::size_t>(n);
        }
        taken.resize(nRead);
        ::close(*readEnd);
        *readEnd = -1;
        closed   = true;
    }
};

ClosingReader closingReader;
} // namespace

// The test replaces `write` for the whole process. It passes every call on and tells the closing reader about the
// bytes a write delivered to the watched descriptor.
extern "C" ::ssize_t write(int fd, const void* data, std::size_t nBytes) {
    const long nWritten = ::syscall(SYS_write, fd, data, nBytes);
    if (nWritten > 0 && fd == closingReader.writeEnd.load()) {
        const int savedErrno = errno;
        closingReader.afterWrite(static_cast<std::size_t>(nWritten));
        errno = savedErrno;
    }
    return static_cast<::ssize_t>(nWritten);
}
#endif

namespace {
using namespace std::chrono_literals;

// both ends of a pipe, closed at the end of the scope unless a test hands an end over
struct Pipe {
    int readEnd  = -1;
    int writeEnd = -1;

    Pipe() {
        int ends[2] = {-1, -1};
        if (::pipe(ends) != 0) {
            throw std::system_error(errno, std::system_category(), "pipe");
        }
        readEnd  = ends[0];
        writeEnd = ends[1];
    }
    Pipe(const Pipe&)            = delete;
    Pipe& operator=(const Pipe&) = delete;
    ~Pipe() {
        closeEnd(readEnd);
        closeEnd(writeEnd);
    }

    static void closeEnd(int& end) {
        if (end >= 0) {
            ::close(end);
            end = -1;
        }
    }
};

[[nodiscard]] bool isOpen(int fd) { return ::fcntl(fd, F_GETFD) != -1; }

void writeAll(int fd, std::span<const std::byte> bytes) {
    while (!bytes.empty()) {
        const ::ssize_t nWritten = ::write(fd, bytes.data(), bytes.size());
        if (nWritten < 0) {
            throw std::system_error(errno, std::system_category(), "write");
        }
        bytes = bytes.subspan(static_cast<std::size_t>(nWritten));
    }
}

// reads the descriptor until end of file
std::vector<std::byte> readAll(int fd) {
    std::vector<std::byte>        bytes;
    std::array<std::byte, 4096UZ> chunk{};
    while (true) {
        const ::ssize_t nRead = ::read(fd, chunk.data(), chunk.size());
        if (nRead < 0 && errno == EINTR) {
            continue;
        }
        if (nRead < 0) {
            throw std::system_error(errno, std::system_category(), "read");
        }
        if (nRead == 0) {
            return bytes;
        }
        bytes.insert(bytes.end(), chunk.begin(), chunk.begin() + nRead);
    }
}

template<typename T>
std::vector<T> knownSamples(std::size_t nSamples) {
    std::vector<T> samples(nSamples);
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        if constexpr (std::is_same_v<T, std::complex<float>>) {
            samples[i] = T{static_cast<float>(i), -0.5f * static_cast<float>(i)};
        } else {
            samples[i] = static_cast<T>((i * 37UZ + 11UZ) % 251UZ);
        }
    }
    return samples;
}

// Runs the scheduler and requests a stop when the run has not ended within the time-out.
std::expected<void, gr::Error> runBounded(gr::scheduler::Simple<>& sched, std::atomic_bool& timedOut, std::chrono::milliseconds timeOut = 30000ms) {
    std::atomic_bool finished{false};
    std::thread      watchdog([&sched, &finished, &timedOut, timeOut] {
        const auto deadline = std::chrono::steady_clock::now() + timeOut;
        while (!finished.load()) {
            if (std::chrono::steady_clock::now() > deadline) {
                timedOut.store(true);
                sched.requestStop();
                return;
            }
            std::this_thread::sleep_for(5ms);
        }
    });
    auto             result = sched.runAndWait();
    finished.store(true);
    watchdog.join();
    return result;
}

void exchangeOrThrow(gr::scheduler::Simple<>& sched, gr::Graph&& graph) {
    if (auto exchanged = sched.exchange(std::move(graph)); !exchanged) {
        throw std::runtime_error(std::format("failed to initialize scheduler: {}", exchanged.error()));
    }
}

[[nodiscard]] std::string errorMessage(const std::expected<void, gr::Error>& result) { return result.has_value() ? std::string{} : std::string(result.error().message); }

// Runs FdSource on the descriptor into an ExpectSink of the expected samples.
struct SourceRun {
    std::expected<void, gr::Error> result;
    bool                           timedOut;
    gr::Size_t                     nSamples;
    gr::Size_t                     nDroppedBytes;
    gr::Size_t                     nReceived;
};

template<typename T>
SourceRun runSource(int fd, const std::vector<T>& expected, bool closeOnStop = false) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<gr::blocks::fileio::FdSource<T>>({{"fd", fd}, {"close_on_stop", closeOnStop}});
    auto&     sink   = graph.emplaceBlock<gr::blocks::testing::ExpectSink<T>>({{"expected", expected}});
    boost::ut::expect(graph.connect<"out", "in">(source, sink).has_value());

    gr::scheduler::Simple<> sched;
    exchangeOrThrow(sched, std::move(graph));
    std::atomic_bool timedOut{false};
    auto             result = runBounded(sched, timedOut);
    return {std::move(result), timedOut.load(), source.n_samples.value, source.n_dropped_bytes.value, sink.received.value};
}

// Runs a VectorSource of the samples into FdSink on the descriptor.
struct SinkRun {
    std::expected<void, gr::Error> result;
    bool                           timedOut;
    gr::Size_t                     nSamples;
};

template<typename T>
SinkRun runSink(int fd, const std::vector<T>& samples, bool closeOnStop = false) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<gr::blocks::testing::VectorSource<T>>({{"values", samples}});
    auto&     sink   = graph.emplaceBlock<gr::blocks::fileio::FdSink<T>>({{"fd", fd}, {"close_on_stop", closeOnStop}});
    boost::ut::expect(graph.connect<"out", "in">(source, sink).has_value());

    gr::scheduler::Simple<> sched;
    exchangeOrThrow(sched, std::move(graph));
    std::atomic_bool timedOut{false};
    auto             result = runBounded(sched, timedOut);
    return {std::move(result), timedOut.load(), sink.n_samples.value};
}

} // namespace

const boost::ut::suite<"FdSource and FdSink"> fdIoTests = [] {
    using namespace boost::ut;
    using gr::blocks::fileio::FdSink;
    using gr::blocks::fileio::FdSource;

    "the defaults read standard input, write standard output and close nothing"_test = [] {
        const FdSource<float> source;
        const FdSink<float>   sink;
        expect(eq(source.fd.value, 0));
        expect(eq(sink.fd.value, 1));
        expect(!source.close_on_stop.value);
        expect(!sink.close_on_stop.value);
    };

    "a pipe carries a known vector from FdSink to FdSource"_test = []<typename T>() {
        // The vector holds more bytes than a 64 KiB pipe. The sink waits for room while the source drains the pipe.
        const std::vector<T> samples = knownSamples<T>(70'000UZ);
        Pipe                 pipe;

        std::optional<SourceRun> sourceRun;
        std::jthread             reader([&] { sourceRun = runSource<T>(pipe.readEnd, samples, true); });
        const SinkRun            sinkRun = runSink<T>(pipe.writeEnd, samples, true);
        reader.join();

        const std::string typeName = gr::meta::type_name<T>();
        expect(sinkRun.result.has_value()) << typeName << ": " << errorMessage(sinkRun.result);
        expect(!sinkRun.timedOut) << typeName;
        expect(eq(sinkRun.nSamples, samples.size())) << typeName;
        expect(fatal(sourceRun.has_value()));
        expect(sourceRun->result.has_value()) << typeName << ": " << errorMessage(sourceRun->result);
        expect(!sourceRun->timedOut) << typeName << ": the source ends at end of file";
        expect(eq(sourceRun->nSamples, samples.size())) << typeName;
        expect(eq(sourceRun->nReceived, samples.size())) << typeName;
        expect(eq(sourceRun->nDroppedBytes, 0U)) << typeName;

        // close_on_stop closed both ends
        expect(!isOpen(pipe.readEnd)) << typeName;
        expect(!isOpen(pipe.writeEnd)) << typeName;
        pipe.readEnd  = -1;
        pipe.writeEnd = -1;
    } | std::tuple<std::uint8_t, std::int16_t, std::int32_t, float, std::complex<float>>{};

    "FdSource joins a sample split across reads and drops an incomplete last sample"_test = [] {
        const std::vector<float>         samples = knownSamples<float>(4UZ);
        const auto                       bytes   = std::as_bytes(std::span(samples));
        constexpr std::size_t            kSplit  = 6UZ; // one sample and half of the next
        const std::array<std::byte, 3UZ> tail{std::byte{1}, std::byte{2}, std::byte{3}};
        Pipe                             pipe;

        std::optional<SourceRun> run;
        std::jthread             reader([&] { run = runSource<float>(pipe.readEnd, samples); });
        writeAll(pipe.writeEnd, bytes.first(kSplit));
#if defined(__linux__)
        // the source reads the first bytes before the rest arrive
        int nQueued = 1;
        for (auto deadline = std::chrono::steady_clock::now() + 10s; nQueued > 0 && std::chrono::steady_clock::now() < deadline;) {
            if (::ioctl(pipe.readEnd, FIONREAD, &nQueued) != 0) {
                break;
            }
            std::this_thread::sleep_for(1ms);
        }
        expect(eq(nQueued, 0)) << "the source drains the first write";
#endif
        writeAll(pipe.writeEnd, bytes.subspan(kSplit));
        writeAll(pipe.writeEnd, tail);
        Pipe::closeEnd(pipe.writeEnd);
        reader.join();

        expect(fatal(run.has_value()));
        expect(run->result.has_value()) << errorMessage(run->result);
        expect(!run->timedOut);
        expect(eq(run->nSamples, samples.size()));
        expect(eq(run->nReceived, samples.size()));
        expect(eq(run->nDroppedBytes, tail.size()));
        expect(isOpen(pipe.readEnd)) << "the source leaves the descriptor open by default";
    };

    "FdSink writes every sample while the reader pauses"_test = [] {
        // The reader starts after the sink has filled the pipe. 40000 floats are more than the 64 KiB the pipe holds.
        const std::vector<float> samples = knownSamples<float>(40'000UZ);
        Pipe                     pipe;

        std::vector<std::byte> received;
        std::jthread           reader([&] {
            std::this_thread::sleep_for(250ms);
            received = readAll(pipe.readEnd);
        });
        const SinkRun          run = runSink<float>(pipe.writeEnd, samples, true);
        reader.join();

        expect(run.result.has_value()) << errorMessage(run.result);
        expect(!run.timedOut);
        expect(eq(run.nSamples, samples.size()));
        const auto expected = std::as_bytes(std::span(samples));
        expect(eq(received.size(), expected.size()));
        expect(std::ranges::equal(received, expected)) << "the reader receives every byte in order";
        pipe.writeEnd = -1;
    };

#if defined(__linux__)
    // The SIGPIPE guard of FdSink is built on Linux alone. The closed-reader cases need it, and every case of this block
    // runs on Linux.
    "FdSink ends with an error when the reader of the pipe is closed"_test = [] {
        Pipe pipe;
        Pipe::closeEnd(pipe.readEnd);
        // The default action of SIGPIPE ends the process. The run passes only when the sink keeps the signal away.
        struct ::sigaction defaultAction {};
        struct ::sigaction previousAction {};
        defaultAction.sa_handler = SIG_DFL;
        expect(fatal(::sigaction(SIGPIPE, &defaultAction, &previousAction) == 0));

        const SinkRun run = runSink<float>(pipe.writeEnd, knownSamples<float>(1000UZ));
        ::sigaction(SIGPIPE, &previousAction, nullptr);
        const std::string message = errorMessage(run.result);
        expect(!run.result.has_value()) << "the run must fail";
        expect(!run.timedOut);
        expect(eq(run.nSamples, 0U));
        expect(message.contains(std::format("the reader of descriptor {} is closed", pipe.writeEnd))) << message;
        expect(isOpen(pipe.writeEnd)) << "the sink leaves the descriptor open by default";
    };

    "FdSink counts the samples its writes delivered before the reader closed"_test = [] {
        // The reader takes 3 pipe writes, 3072 floats, and closes. The sink's next write fails with EPIPE. Without the
        // count of earlier writes, n_samples reads 0 when the failing call holds more input than the reader took.
        constexpr std::size_t nBytesTaken   = 3UZ * PIPE_BUF;
        constexpr std::size_t nSamplesTaken = nBytesTaken / sizeof(float);
        const auto            samples       = knownSamples<float>(20'000UZ);
        Pipe                  pipe;
        closingReader.arm(&pipe.readEnd, pipe.writeEnd, nBytesTaken);

        struct ::sigaction defaultAction {};
        struct ::sigaction previousAction {};
        defaultAction.sa_handler = SIG_DFL;
        expect(fatal(::sigaction(SIGPIPE, &defaultAction, &previousAction) == 0));
        const SinkRun run = runSink<float>(pipe.writeEnd, samples);
        ::sigaction(SIGPIPE, &previousAction, nullptr);
        closingReader.writeEnd.store(-1);

        const std::string message = errorMessage(run.result);
        expect(!run.result.has_value()) << "the run must fail";
        expect(!run.timedOut);
        expect(message.contains(std::format("the reader of descriptor {} is closed", pipe.writeEnd))) << message;
        expect(closingReader.closed);
        const auto expectedBytes = std::as_bytes(std::span(samples)).first(nBytesTaken);
        expect(std::ranges::equal(closingReader.taken, expectedBytes)) << "the reader took the first samples in order";
        expect(eq(run.nSamples, static_cast<gr::Size_t>(nSamplesTaken))) << "n_samples counts the samples the reader took";
    };

    "FdSource ends with an error when a read fails"_test = [] {
        // a directory opens for reading, and a read from it fails
        const int directory = ::open("/", O_RDONLY | O_DIRECTORY);
        expect(fatal(directory >= 0));

        const SourceRun   run     = runSource<float>(directory, {});
        const std::string message = errorMessage(run.result);
        expect(!run.result.has_value()) << "the run must fail";
        expect(!run.timedOut);
        expect(message.contains(std::format("reading descriptor {} failed", directory))) << message;
        ::close(directory);
    };
#endif

    "a descriptor that is closed or open the other way refuses the start"_test = [] {
        Pipe              pipe;
        const std::string wrongSource = errorMessage(runSource<float>(pipe.writeEnd, {}).result);
        expect(wrongSource.contains(std::format("descriptor {} is open for writing only", pipe.writeEnd))) << wrongSource;
        const std::string wrongSink = errorMessage(runSink<float>(pipe.readEnd, {1.f}).result);
        expect(wrongSink.contains(std::format("descriptor {} is open for reading only", pipe.readEnd))) << wrongSink;
        const std::string closed = errorMessage(runSource<float>(-1, {}).result);
        expect(closed.contains("descriptor -1 is not open")) << closed;
    };

    "a stop request ends FdSource while the writer stays open and silent"_test = [] {
        Pipe      pipe;
        gr::Graph graph;
        auto&     source = graph.emplaceBlock<FdSource<float>>({{"fd", pipe.readEnd}});
        auto&     sink   = graph.emplaceBlock<gr::blocks::testing::ExpectSink<float>>();
        expect(graph.connect<"out", "in">(source, sink).has_value());

        gr::scheduler::Simple<> sched;
        exchangeOrThrow(sched, std::move(graph));
        std::atomic_bool               finished{false};
        std::expected<void, gr::Error> result;
        std::thread                    runner([&] {
            result = sched.runAndWait();
            finished.store(true);
        });
        while (sched.state() != gr::lifecycle::State::RUNNING && !finished.load()) {
            std::this_thread::sleep_for(1ms);
        }
        sched.requestStop();
        const auto deadline = std::chrono::steady_clock::now() + 10s;
        while (!finished.load() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(1ms);
        }
        const bool stoppedInTime = finished.load();
        if (!stoppedInTime) {
            Pipe::closeEnd(pipe.writeEnd); // end of file releases a source that waits in a read
        }
        runner.join();
        expect(stoppedInTime) << "the stop request ends the run";
        expect(result.has_value()) << errorMessage(result);
        expect(eq(source.n_samples.value, 0U));
    };
};

int main() { /* not needed for UT */ }
