#include <boost/ut.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <cerrno>
#include <dirent.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/network/TcpByteIo.hpp>

// A transport is only proved by a peer it did not build, so the second-peer refusal below meets the sink or source
// at a raw socket rather than at another instance of the block under test. The byte-identity criterion, in contrast,
// needs no raw peer: the sink and the source are both this tree's own blocks, in one graph, and what crosses is
// asserted against the vector the source side was given.
namespace qa_tcp_byte_io {

using namespace std::chrono_literals;
using gr::blocks::network::TcpByteSink;
using gr::blocks::network::TcpByteSource;

// ─── endpoints ────────────────────────────────────────────────────────────────────────────────────────────────────

/// @brief An ephemeral loopback port: bind a throwaway socket to port 0, read back what the kernel chose, close it,
/// which leaves the block under test free to bind that same port a moment later.
[[nodiscard]] std::uint16_t reservePort() {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    boost::ut::expect(fd >= 0) << "the test could not open a socket to reserve a port";
    ::sockaddr_in address{};
    address.sin_family      = AF_INET;
    address.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
    address.sin_port        = 0;
    boost::ut::expect(::bind(fd, reinterpret_cast<const ::sockaddr*>(&address), static_cast<::socklen_t>(sizeof(address))) == 0);
    ::socklen_t length = static_cast<::socklen_t>(sizeof(address));
    boost::ut::expect(::getsockname(fd, reinterpret_cast<::sockaddr*>(&address), &length) == 0);
    const std::uint16_t port = ::ntohs(address.sin_port);
    std::ignore              = ::close(fd);
    return port;
}

[[nodiscard]] std::string endpointFor(std::uint16_t port) { return std::format("127.0.0.1:{}", port); }

// ─── faulting a descriptor the block owns ─────────────────────────────────────────────────────────────────────────
// A block owns its sockets end to end and lends out no handle to them, which is the property the fatal-exit tests
// have to work around: to break the listening socket a test must first name it, and the only description of it that
// leaves the block is the kernel's own. Every open descriptor of this process appears under /proc/self/fd, and on a
// port this test reserved for one block exactly one of them is a listening socket.

/// @brief The listening TCP socket of this process bound to @p port, or -1 while there is none.
[[nodiscard]] int findListeningSocket(std::uint16_t port) {
    DIR* directory = ::opendir("/proc/self/fd");
    if (directory == nullptr) {
        return -1;
    }
    int found = -1;
    for (const ::dirent* entry = ::readdir(directory); entry != nullptr; entry = ::readdir(directory)) {
        const int fd = std::atoi(entry->d_name);
        if (fd <= 2 || fd == ::dirfd(directory)) {
            continue; // the standard streams and the handle this scan is holding are nobody's socket
        }
        int         listening = 0;
        ::socklen_t flagSize  = static_cast<::socklen_t>(sizeof(listening));
        if (::getsockopt(fd, SOL_SOCKET, SO_ACCEPTCONN, &listening, &flagSize) != 0 || listening == 0) {
            continue; // an accepted connection carries the same local port, and it is not what is wanted here
        }
        ::sockaddr_in address{};
        ::socklen_t   size = static_cast<::socklen_t>(sizeof(address));
        if (::getsockname(fd, reinterpret_cast<::sockaddr*>(&address), &size) == 0 && address.sin_family == AF_INET && ::ntohs(address.sin_port) == port) {
            found = fd;
            break;
        }
    }
    std::ignore = ::closedir(directory);
    return found;
}

/// @brief Put something that is not a socket at @p fd, so a poll on it reports ready and the next accept refuses it.
///
/// The descriptor number stays taken throughout, so the block's own close is still correct and no other thread can
/// claim the number in between, which closing it outright would allow.
void replaceWithNonSocket(int fd) {
    const int placeholder = ::open("/dev/null", O_RDWR);
    boost::ut::expect(placeholder >= 0) << "the test could not open /dev/null";
    boost::ut::expect(::dup2(placeholder, fd) == fd) << "the test could not put a non-socket at the block's descriptor";
    std::ignore = ::close(placeholder);
}

/// @brief The local port of every socket of this process whose connection attempt is still unanswered, leaving out the
/// descriptors @p mine names.
///
/// A half-open attempt appears in none of a block's own state, so the same scan the fatal-exit tests use serves here:
/// `TCP_INFO` names the state of each descriptor, and an attempt still waiting is one in `TCP_SYN_SENT` that the test
/// does not own itself. The local port identifies the attempt, since a second one is a second socket and the kernel
/// gives that a port of its own.
[[nodiscard]] std::vector<std::uint16_t> pendingConnectPorts(std::span<const int> mine) {
    std::vector<std::uint16_t> ports;
    DIR*                       directory = ::opendir("/proc/self/fd");
    if (directory == nullptr) {
        return ports;
    }
    for (const ::dirent* entry = ::readdir(directory); entry != nullptr; entry = ::readdir(directory)) {
        const int fd = std::atoi(entry->d_name);
        if (fd <= 2 || fd == ::dirfd(directory) || std::ranges::find(mine, fd) != mine.end()) {
            continue;
        }
        ::tcp_info  info{};
        ::socklen_t infoSize = static_cast<::socklen_t>(sizeof(info));
        if (::getsockopt(fd, IPPROTO_TCP, TCP_INFO, &info, &infoSize) != 0 || info.tcpi_state != TCP_SYN_SENT) {
            continue;
        }
        ::sockaddr_in address{};
        ::socklen_t   size = static_cast<::socklen_t>(sizeof(address));
        if (::getsockname(fd, reinterpret_cast<::sockaddr*>(&address), &size) == 0 && address.sin_family == AF_INET) {
            ports.push_back(::ntohs(address.sin_port));
        }
    }
    std::ignore = ::closedir(directory);
    return ports;
}

/// @brief A raw TCP connection the test drives itself, so a refusal, a disconnect and a reconnect are all proved
/// against a peer neither block built.
struct RawStream {
    int fd = -1;

    RawStream()                            = default;
    RawStream(const RawStream&)            = delete;
    RawStream& operator=(const RawStream&) = delete;
    ~RawStream() { close(); }

    void connectTo(std::uint16_t port, std::chrono::milliseconds deadline = 5000ms) {
        const auto until = std::chrono::steady_clock::now() + deadline;
        while (std::chrono::steady_clock::now() < until) {
            const int candidate = ::socket(AF_INET, SOCK_STREAM, 0);
            if (candidate < 0) {
                continue;
            }
            ::sockaddr_in address{};
            address.sin_family      = AF_INET;
            address.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
            address.sin_port        = ::htons(port);
            if (::connect(candidate, reinterpret_cast<const ::sockaddr*>(&address), static_cast<::socklen_t>(sizeof(address))) == 0) {
                fd = candidate;
                return;
            }
            std::ignore = ::close(candidate);
            std::this_thread::yield();
        }
        boost::ut::expect(fd >= 0) << "the test could not connect to port " << port;
    }

    /// @brief Write @p bytes whole to the block under test, on a blocking socket the block itself keeps draining.
    void send(std::span<const std::uint8_t> bytes) const {
        std::size_t sent = 0UZ;
        while (sent < bytes.size()) {
            const ::ssize_t written = ::send(fd, bytes.data() + sent, bytes.size() - sent, MSG_NOSIGNAL);
            if (written > 0) {
                sent += static_cast<std::size_t>(written);
                continue;
            }
            if (written < 0 && (errno == EINTR || errno == EAGAIN)) {
                std::this_thread::yield();
                continue;
            }
            break;
        }
        boost::ut::expect(sent == bytes.size()) << "the test could not write " << bytes.size() << " bytes to its raw peer";
    }

    /// @brief True once the peer has closed: a refused connection is accepted and immediately closed, so a read
    /// returning zero (or an error other than would-block) is the refusal's own evidence.
    [[nodiscard]] bool closedByPeer(std::chrono::milliseconds deadline = 5000ms) const {
        const auto until = std::chrono::steady_clock::now() + deadline;
        while (std::chrono::steady_clock::now() < until) {
            ::pollfd  item{.fd = fd, .events = POLLIN, .revents = 0};
            const int ready = ::poll(&item, 1UL, 50);
            if (ready > 0) {
                std::uint8_t    scratch  = 0U;
                const ::ssize_t received = ::recv(fd, &scratch, 1UZ, MSG_DONTWAIT);
                if (received <= 0 && !(received < 0 && (errno == EAGAIN || errno == EINTR))) {
                    return true;
                }
            }
        }
        return false;
    }

    void close() {
        if (fd >= 0) {
            std::ignore = ::close(fd);
            fd          = -1;
        }
    }
};

/// @brief A loopback endpoint that begins a connection and never finishes one.
///
/// The kernel drops a SYN that arrives at a listening socket whose accept queue is full, and a client that sent one
/// stays in `TCP_SYN_SENT` retransmitting for minutes. Taking the single queue slot of a socket nobody accepts from
/// therefore gives a connect that reports `EINPROGRESS` and is answered by nobody, without assuming anything about
/// what this machine's routes do with an address off the loopback.
struct UnansweredEndpoint {
    int           listener = -1;
    std::uint16_t port     = 0U;
    RawStream     occupant{};

    UnansweredEndpoint() {
        listener = ::socket(AF_INET, SOCK_STREAM, 0);
        boost::ut::expect(listener >= 0) << "the test could not open a listening socket";
        ::sockaddr_in address{};
        address.sin_family      = AF_INET;
        address.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
        address.sin_port        = 0;
        boost::ut::expect(::bind(listener, reinterpret_cast<const ::sockaddr*>(&address), static_cast<::socklen_t>(sizeof(address))) == 0);
        boost::ut::expect(::listen(listener, 0) == 0) << "the test could not listen with the shortest accept queue there is";
        ::socklen_t length = static_cast<::socklen_t>(sizeof(address));
        boost::ut::expect(::getsockname(listener, reinterpret_cast<::sockaddr*>(&address), &length) == 0);
        port = ::ntohs(address.sin_port);
        occupant.connectTo(port); // the one slot, taken and never accepted from
    }

    UnansweredEndpoint(const UnansweredEndpoint&)            = delete;
    UnansweredEndpoint& operator=(const UnansweredEndpoint&) = delete;

    ~UnansweredEndpoint() {
        if (listener >= 0) {
            std::ignore = ::close(listener);
        }
    }

    /// @brief The descriptors this endpoint owns, which a scan of the process's sockets must leave to it.
    [[nodiscard]] std::vector<int> descriptors() const { return {listener, occupant.fd}; }
};

// ─── a byte source that never ends, and a sink whose collection the test thread may read ─────────────────────────

struct ByteVectorSource : gr::Block<ByteVectorSource> {
    gr::PortOut<std::uint8_t> out;
    GR_MAKE_REFLECTABLE(ByteVectorSource, out);

    std::vector<std::uint8_t> _bytes{};
    std::size_t               _next = 0UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (_next >= _bytes.size()) {
            outSpan.publish(0UZ);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS; // idle rather than done: the test owns the teardown
        }
        const std::size_t room = std::min(outSpan.size(), _bytes.size() - _next);
        for (std::size_t i = 0UZ; i < room; ++i) {
            outSpan[i] = _bytes[_next + i];
        }
        _next += room;
        outSpan.publish(room);
        return room == 0UZ ? gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS : gr::work::Status::OK;
    }
};

struct ByteVectorSink : gr::Block<ByteVectorSink> {
    gr::PortIn<std::uint8_t> in;
    GR_MAKE_REFLECTABLE(ByteVectorSink, in);

    mutable std::mutex        _mutex;
    std::vector<std::uint8_t> _bytes{};

    [[nodiscard]] std::size_t count() const {
        std::lock_guard lock(_mutex);
        return _bytes.size();
    }

    [[nodiscard]] std::vector<std::uint8_t> take() const {
        std::lock_guard lock(_mutex);
        return _bytes;
    }

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        {
            std::lock_guard lock(_mutex);
            for (std::size_t i = 0UZ; i < inSpan.size(); ++i) {
                _bytes.push_back(inSpan[i]);
            }
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

// ─── running a graph beside the test's own socket work ─────────────────────────────────────────────────────────────

struct GraphRunner {
    gr::scheduler::Simple<> scheduler;
    std::thread             worker;
    std::atomic<bool>       finished{false};
    std::atomic<bool>       failed{false}; ///< the run ended in the scheduler's ERROR state rather than by request

    explicit GraphRunner(gr::Graph&& graph) {
        boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
        worker = std::thread([this] {
            const auto outcome = scheduler.runAndWait();
            failed.store(!outcome.has_value());
            finished.store(true);
        });
    }

    GraphRunner(const GraphRunner&)            = delete;
    GraphRunner& operator=(const GraphRunner&) = delete;

    void stop() {
        scheduler.requestStop();
        if (worker.joinable()) {
            worker.join();
        }
    }

    ~GraphRunner() { stop(); }
};

template<typename F>
[[nodiscard]] bool waitFor(F&& ready, std::chrono::milliseconds deadline = 8000ms) {
    const auto until = std::chrono::steady_clock::now() + deadline;
    while (std::chrono::steady_clock::now() < until) {
        if (ready()) {
            return true;
        }
        std::this_thread::yield();
    }
    return ready();
}

// TcpByteSink/TcpByteSource hold a mutex, a condition_variable and the IoThreadGuard directly, so they are neither
// copyable nor movable: a settings check is done in place rather than through a return-by-value helper.
template<typename TBlock>
[[nodiscard]] bool stagingThrows(gr::property_map settings) {
    try {
        TBlock block(std::move(settings));
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        return false;
    } catch (const gr::exception&) {
        return true;
    }
}

/// @brief A seeded byte stream: every value 0x00-0xFF first, then runs of the two SLIP-significant values this layer
/// must not touch, then a reproducible pseudo-random fill to reach @p total.
[[nodiscard]] std::vector<std::uint8_t> seededStream(std::size_t total) {
    std::vector<std::uint8_t> bytes;
    bytes.reserve(total);
    for (unsigned v = 0U; v <= 0xFFU; ++v) {
        bytes.push_back(static_cast<std::uint8_t>(v));
    }
    bytes.insert(bytes.end(), 24UZ, 0xC0U);
    bytes.insert(bytes.end(), 24UZ, 0xDBU);

    std::uint64_t rng = 0x9E3779B97F4A7C15ULL;
    while (bytes.size() < total) {
        rng = rng * 6364136223846793005ULL + 1442695040888963407ULL;
        bytes.push_back(static_cast<std::uint8_t>(rng >> 56U));
    }
    bytes.resize(total);
    return bytes;
}

} // namespace qa_tcp_byte_io

using namespace qa_tcp_byte_io;

const boost::ut::suite<"TcpByteIo"> tcpByteIoTests = [] {
    using namespace boost::ut;

    "the settings that gate start() are checked before any socket opens"_test = [] {
        expect(stagingThrows<TcpByteSink>({{"queue_bytes", gr::Size_t{4096U}}})) << "endpoint has no default";
        expect(stagingThrows<TcpByteSink>({{"endpoint", std::string("127.0.0.1:5555")}})) << "queue_bytes has no default";
        expect(stagingThrows<TcpByteSink>({{"endpoint", std::string("127.0.0.1:5555")}, {"queue_bytes", gr::Size_t{4096U}}, {"overflow", std::string("sideways")}})) << "overflow names one of two values";
        expect(stagingThrows<TcpByteSink>({{"endpoint", std::string("127.0.0.1:5555")}, {"queue_bytes", gr::Size_t{4096U}}, {"reconnect_ms", gr::Size_t{0U}}})) << "a zero reconnect interval spends the thread on nothing else";
        expect(stagingThrows<TcpByteSink>({{"endpoint", std::string("127.0.0.1:5555")}, {"queue_bytes", gr::Size_t{4096U}}, {"connect_timeout_ms", gr::Size_t{0U}}})) << "a zero connect timeout abandons every attempt before it can be answered";
        expect(!stagingThrows<TcpByteSink>({{"endpoint", std::string("127.0.0.1:5555")}, {"queue_bytes", gr::Size_t{4096U}}}));

        expect(stagingThrows<TcpByteSource>({{"queue_bytes", gr::Size_t{4096U}}})) << "endpoint has no default";
        expect(stagingThrows<TcpByteSource>({{"endpoint", std::string("127.0.0.1:5555")}})) << "queue_bytes has no default";
        expect(stagingThrows<TcpByteSource>({{"endpoint", std::string("127.0.0.1:5555")}, {"queue_bytes", gr::Size_t{4096U}}, {"overflow", std::string("sideways")}})) << "overflow names one of two values";
        expect(stagingThrows<TcpByteSource>({{"endpoint", std::string("127.0.0.1:5555")}, {"queue_bytes", gr::Size_t{4096U}}, {"reconnect_ms", gr::Size_t{0U}}})) << "a zero reconnect interval spends the thread on nothing else";
        expect(stagingThrows<TcpByteSource>({{"endpoint", std::string("127.0.0.1:5555")}, {"queue_bytes", gr::Size_t{4096U}}, {"connect_timeout_ms", gr::Size_t{0U}}})) << "a zero connect timeout abandons every attempt before it can be answered";
        expect(!stagingThrows<TcpByteSource>({{"endpoint", std::string("127.0.0.1:5555")}, {"queue_bytes", gr::Size_t{4096U}}}));
    };

    // Criterion 8: the byte transport is transparent, in either role assignment, including a second peer refused.
    "a seeded 1 MiB stream crosses byte-identical, and a second peer connecting is refused"_test = [] {
        const auto crossing = [](bool sinkBinds) {
            const std::uint16_t             port = reservePort();
            const std::vector<std::uint8_t> sent = seededStream(1UZ << 20U);

            gr::Graph graph;
            auto&     producer = graph.emplaceBlock<ByteVectorSource>();
            producer._bytes    = sent;
            auto& sink         = graph.emplaceBlock<TcpByteSink>({{"endpoint", endpointFor(port)}, {"bind", sinkBinds}, {"overflow", std::string("backpressure")}, {"queue_bytes", gr::Size_t{1U << 21U}}});
            auto& source       = graph.emplaceBlock<TcpByteSource>({{"endpoint", endpointFor(port)}, {"bind", !sinkBinds}, {"overflow", std::string("backpressure")}, {"queue_bytes", gr::Size_t{1U << 21U}}});
            auto& collector    = graph.emplaceBlock<ByteVectorSink>();
            expect(graph.connect<"out", "in">(producer, sink).has_value());
            expect(graph.connect<"out", "in">(source, collector).has_value());

            GraphRunner runner(std::move(graph));
            expect(waitFor([&collector, &sent] { return collector.count() >= sent.size(); }, 15000ms)) << std::format("sink binds {}: only {} of {} bytes crossed", sinkBinds, collector.count(), sent.size());

            const std::vector<std::uint8_t> received = collector.take();
            expect(eq(received.size(), sent.size())) << std::format("sink binds {}", sinkBinds);
            expect(that % (received == sent)) << std::format("sink binds {}: byte-identical, 0x00-0xFF and the 0xC0/0xDB runs included", sinkBinds);
            expect(eq(sink.counters().socketErrors, std::uint64_t{0ULL}));
            expect(eq(source.counters().socketErrors, std::uint64_t{0ULL}));
            expect(eq(sink.counters().bytesDropped, std::uint64_t{0ULL})) << "backpressure, not a drop, was the queue's answer to being full";
            expect(eq(source.counters().bytesDropped, std::uint64_t{0ULL}));

            // the peer bound to `port` is whichever block has `bind = true`; the other end is already the one peer
            // served, so a raw extra connection to the listening port is accepted and immediately closed
            RawStream extra;
            extra.connectTo(port);
            expect(extra.closedByPeer()) << std::format("sink binds {}: a second peer was not refused", sinkBinds);
            if (sinkBinds) {
                expect(eq(sink.counters().peersRefused, std::uint64_t{1ULL})) << "the sink is the listening end here: one extra connection, one refusal";
            } else {
                expect(eq(source.counters().peersRefused, std::uint64_t{1ULL})) << "the source is the listening end here: one extra connection, one refusal";
            }

            runner.stop();
        };

        crossing(true);
        crossing(false);
    };

    // Criterion 8's disconnect half, at the sink: what the sink counts when its one peer goes away and another takes
    // its place.
    "a sink counts a disconnect mid-stream and the reconnection that follows"_test = [] {
        const std::uint16_t port = reservePort();

        gr::Graph graph;
        auto&     sink = graph.emplaceBlock<TcpByteSink>({{"endpoint", endpointFor(port)}, {"bind", true}, {"overflow", std::string("backpressure")}, {"queue_bytes", gr::Size_t{1U << 16U}}});

        // the source side is a raw peer the test connects, disconnects and reconnects itself
        std::vector<std::uint8_t> producedBytes{0x11U, 0x22U, 0x33U};
        auto&                     producer = graph.emplaceBlock<ByteVectorSource>();
        producer._bytes                    = producedBytes;
        expect(graph.connect<"out", "in">(producer, sink).has_value());

        GraphRunner runner(std::move(graph));

        {
            RawStream first;
            first.connectTo(port);
            expect(waitFor([&sink, &producedBytes] { return sink.counters().bytes >= producedBytes.size(); })) << "the first peer never received anything";
        } // closing the RawStream disconnects the sink's one peer

        expect(waitFor([&sink] { return sink.counters().disconnects >= 1ULL; })) << "the disconnection was not counted";

        RawStream second;
        second.connectTo(port);
        expect(waitFor([&sink] { return sink.counters().reconnects >= 1ULL; })) << "the reconnection was not counted";

        runner.stop();
    };

    // Criterion 8's disconnect half, at the source: the block under test is a real TcpByteSource, and what the second
    // peer sends must reach the graph — a reconnect that counts but delivers nothing is not a resumption.
    "a source counts a disconnect and delivers what the peer that replaces it sends"_test = [] {
        const std::uint16_t port = reservePort();

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<TcpByteSource>({{"endpoint", endpointFor(port)}, {"bind", true}, {"overflow", std::string("backpressure")}, {"queue_bytes", gr::Size_t{1U << 12U}}});
        auto&     collector = graph.emplaceBlock<ByteVectorSink>();
        expect(graph.connect<"out", "in">(source, collector).has_value());

        GraphRunner runner(std::move(graph));

        const std::vector<std::uint8_t> before{0x11U, 0x22U, 0x33U};
        const std::vector<std::uint8_t> after{0x44U, 0x55U};
        {
            RawStream first;
            first.connectTo(port);
            first.send(before);
            expect(waitFor([&collector, &before] { return collector.count() >= before.size(); })) << "the first peer's bytes never reached the graph";
        } // closing the RawStream disconnects the source's one peer

        expect(waitFor([&source] { return source.counters().disconnects >= 1ULL; })) << "the disconnection was not counted";

        RawStream second;
        second.connectTo(port);
        expect(waitFor([&source] { return source.counters().reconnects >= 1ULL; })) << "the reconnection was not counted";
        second.send(after);
        expect(waitFor([&collector, &before, &after] { return collector.count() >= before.size() + after.size(); })) << "the source published nothing from the peer that replaced the first";

        runner.stop();

        std::vector<std::uint8_t> expected = before;
        expected.insert(expected.end(), after.begin(), after.end());
        expect(that % (collector.take() == expected)) << "across the reconnect the stream is the two peers' bytes in order, with nothing invented for the gap";
    };

    // The default overflow policy, which the backpressure arms above never reach.
    "the sink's drop_oldest sheds every byte it cannot deliver, and the teardown residue with them"_test = [] {
        constexpr std::size_t kQueueBytes = 64UZ;
        constexpr std::size_t kOffered    = 4096UZ;
        const std::uint16_t   port        = reservePort();

        gr::Graph graph;
        auto&     producer = graph.emplaceBlock<ByteVectorSource>();
        producer._bytes    = seededStream(kOffered);
        // a listening sink nobody connects to: the queue is the only place a byte can go, and `overflow` is left at
        // its default rather than stated, because the default is what a recipe that says nothing gets
        auto& sink = graph.emplaceBlock<TcpByteSink>({{"endpoint", endpointFor(port)}, {"bind", true}, {"queue_bytes", gr::Size_t{static_cast<std::uint32_t>(kQueueBytes)}}});
        expect(graph.connect<"out", "in">(producer, sink).has_value());

        GraphRunner runner(std::move(graph));
        expect(waitFor([&sink] { return sink.counters().bytesDropped >= kOffered - kQueueBytes; })) << "the sink never took all the offered bytes, so drop_oldest was not exercised to the end";
        runner.stop();

        const auto counted = sink.counters();
        expect(eq(counted.bytes, std::uint64_t{0ULL})) << "no peer ever connected, so nothing was written";
        expect(eq(counted.bytesDropped, std::uint64_t{kOffered})) << "every offered byte was shed: all but the last queue_bytes on overflow, those last ones as the teardown residue";
        expect(eq(counted.socketErrors, std::uint64_t{0ULL})) << "a queue that overflows is not a socket that failed";
    };

    // The source's own drop_oldest, including the read that arrives larger than the room left: the byte-stream
    // contract has no frame to hold a read to, so the excess is shed exactly as an overflow is.
    "the source's drop_oldest sheds the stalest bytes, however a single read is sized"_test = [] {
        constexpr std::size_t kQueueBytes = 64UZ;
        constexpr std::size_t kSent       = 4096UZ;
        const std::uint16_t   port        = reservePort();

        // driven without a graph, so nothing drains the receive queue and `overflow` is the only thing acting on it
        TcpByteSource source({{"endpoint", endpointFor(port)}, {"bind", true}, {"queue_bytes", gr::Size_t{static_cast<std::uint32_t>(kQueueBytes)}}});
        source.settings().init();
        std::ignore = source.settings().applyStagedParameters();
        source.start();

        RawStream peer;
        peer.connectTo(port);
        peer.send(seededStream(kSent));
        expect(waitFor([&source] { return source.counters().bytes >= kSent; })) << "the source did not read everything the peer sent";

        const auto counted = source.counters();
        source.stop();

        expect(eq(counted.bytes, std::uint64_t{kSent}));
        expect(eq(counted.bytesDropped, std::uint64_t{kSent - kQueueBytes})) << "each byte past the bound displaced one, whatever sizes the kernel gave the individual reads";
        expect(eq(counted.socketErrors, std::uint64_t{0ULL}));
    };

    // A dead reader is not a quiet wire, and the two must not look alike to the graph.
    "a source whose reader has ended reports ERROR once its queue has drained"_test = [] {
        const std::uint16_t port = reservePort();

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<TcpByteSource>({{"endpoint", endpointFor(port)}, {"bind", true}, {"overflow", std::string("backpressure")}, {"queue_bytes", gr::Size_t{1U << 12U}}});
        auto&     collector = graph.emplaceBlock<ByteVectorSink>();
        expect(graph.connect<"out", "in">(source, collector).has_value());

        GraphRunner runner(std::move(graph));

        RawStream peer;
        peer.connectTo(port);
        const std::vector<std::uint8_t> bytes{0x11U, 0x22U, 0x33U};
        peer.send(bytes);
        expect(waitFor([&collector, &bytes] { return collector.count() >= bytes.size(); })) << "nothing crossed, so the queue was never non-empty and the drain condition means nothing";
        expect(!runner.finished.load()) << "a live, merely quiet connection is not an error";

        // A TCP reader reconnects for as long as it is allowed to, so no socket fault of its own ends it; the state
        // its exit does leave is written here, under the very lock the reader would take to write it. What is under
        // test is the status the source owes the graph once the reader is gone and the queue behind it is empty.
        {
            std::lock_guard lock(source._mutex);
            source._readerFailed = true;
        }
        expect(waitFor([&runner] { return runner.finished.load(); })) << "a dead reader over a drained queue did not end the graph";
        expect(runner.failed.load()) << "the run ended, but not in the error a dead reader owes a graph that would otherwise wait forever";

        runner.stop();
    };

    // The fault that reaches the status above in service: the listening socket the reader owns stops being one it can
    // accept on. Both shapes are provoked here — a socket that has been shut down, which polls as POLLHUP, and a
    // descriptor that is no longer a socket at all, which polls ready and refuses the accept.
    "a listening socket that cannot serve again ends the reader, which names the fault"_test = [] {
        const auto endsOnListenerFault = [](bool asNonSocket) {
            const std::uint16_t port = reservePort();

            gr::Graph graph;
            auto&     source    = graph.emplaceBlock<TcpByteSource>({{"endpoint", endpointFor(port)}, {"bind", true}, {"overflow", std::string("backpressure")}, {"queue_bytes", gr::Size_t{1U << 12U}}});
            auto&     collector = graph.emplaceBlock<ByteVectorSink>();
            expect(graph.connect<"out", "in">(source, collector).has_value());

            GraphRunner runner(std::move(graph));

            int listener = -1;
            expect(waitFor([&listener, port] {
                listener = findListeningSocket(port);
                return listener >= 0;
            })) << "the source never opened a listening socket on its own port";

            if (asNonSocket) {
                replaceWithNonSocket(listener);
            } else {
                std::ignore = ::shutdown(listener, SHUT_RDWR);
            }

            expect(waitFor([&runner] { return runner.finished.load(); })) << std::format("non-socket {}: a listening socket that cannot serve again left the reader running", asNonSocket);
            expect(runner.failed.load()) << "the reader ended on a fault and the graph did not end in error";
            runner.stop();

            const std::string reason = source.lastReaderError();
            if (asNonSocket) {
                expect(reason.starts_with("accept failed")) << "the reader did not name the accept that refused it: " << reason;
                expect(reason.contains("socket")) << "the reason drops the system's own wording: " << reason;
            } else {
                expect(reason.contains("POLLHUP") || reason.contains("POLLERR") || reason.contains("POLLNVAL")) << "the reader did not name the flag its listening socket reported: " << reason;
            }
            expect(eq(source.nReaderFailures, std::uint64_t{1ULL})) << "one reader ended on a fault, so one failure is counted";
            expect(eq(collector.count(), 0UZ)) << "no peer ever connected, so nothing should have been published";
        };

        endsOnListenerFault(false);
        endsOnListenerFault(true);
    };

    // The other side of that ruling: what a peer does is never a fault of the reader's own socket, however abruptly
    // it does it. A reset is the sharpest form — no orderly close, and the reader learns of it as a failed read.
    "a peer that resets its connection costs the connection and not the reader"_test = [] {
        const std::uint16_t port = reservePort();

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<TcpByteSource>({{"endpoint", endpointFor(port)}, {"bind", true}, {"overflow", std::string("backpressure")}, {"queue_bytes", gr::Size_t{1U << 12U}}});
        auto&     collector = graph.emplaceBlock<ByteVectorSink>();
        expect(graph.connect<"out", "in">(source, collector).has_value());

        GraphRunner runner(std::move(graph));

        const std::vector<std::uint8_t> before{0x11U, 0x22U, 0x33U};
        const std::vector<std::uint8_t> after{0x44U, 0x55U};
        {
            RawStream first;
            first.connectTo(port);
            first.send(before);
            expect(waitFor([&collector, &before] { return collector.count() >= before.size(); })) << "the first peer's bytes never reached the graph";
            // zero linger turns the close below into a reset rather than a FIN, which is what the reader meets as a
            // failed read instead of an orderly end of stream
            const ::linger immediate{.l_onoff = 1, .l_linger = 0};
            expect(::setsockopt(first.fd, SOL_SOCKET, SO_LINGER, &immediate, static_cast<::socklen_t>(sizeof(immediate))) == 0) << "the test could not arm a reset";
        }

        expect(waitFor([&source] { return source.counters().socketErrors >= 1ULL; })) << "the reset was not seen as a failed read";
        expect(waitFor([&source] { return source.counters().disconnects >= 1ULL; })) << "the reset connection was not given up";
        expect(!runner.finished.load()) << "a peer's reset ended the graph";

        RawStream second;
        second.connectTo(port);
        second.send(after);
        expect(waitFor([&collector, &before, &after] { return collector.count() >= before.size() + after.size(); })) << "the source did not serve the peer that replaced the reset one";
        runner.stop();

        expect(eq(source.nReaderFailures, std::uint64_t{0ULL})) << "a peer's reset is not a fault of the socket the reader owns";
        expect(source.lastReaderError().empty()) << "the reader named a fault it did not have: " << source.lastReaderError();
    };

    // What a connecting block owes a peer a long round trip away: the attempt is one socket for as long as
    // `connect_timeout_ms` allows, and the 100 ms wait that keeps `stop()` prompt is not what ends it.
    "an unanswered connect keeps its one socket until connect_timeout_ms"_test = [] {
        const UnansweredEndpoint blocked;

        {
            TcpByteSource source({{"endpoint", endpointFor(blocked.port)}, {"bind", false}, {"queue_bytes", gr::Size_t{4096U}}, {"reconnect_ms", gr::Size_t{10U}}, {"connect_timeout_ms", gr::Size_t{60000U}}});
            source.settings().init();
            std::ignore = source.settings().applyStagedParameters();
            source.start();

            std::vector<std::uint16_t> first;
            expect(waitFor([&first, &blocked] {
                first = pendingConnectPorts(blocked.descriptors());
                return first.size() == 1UZ;
            })) << "the source began no connection that stayed unanswered; a kernel that answers a SYN at a full accept queue needs another seam for this";

            std::this_thread::sleep_for(350ms); // more than three of the I/O thread's 100 ms waits
            const std::vector<std::uint16_t> later = pendingConnectPorts(blocked.descriptors());
            expect(eq(later.size(), 1UZ)) << "after 350 ms the attempt was not one socket";
            expect(later.size() == 1UZ && later.front() == first.front()) << "the attempt was abandoned at a poll interval and made again on a new socket";

            const auto asked = std::chrono::steady_clock::now();
            source.stop();
            const auto took = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - asked);
            expect(took < 1000ms) << "stop() waited on the half-open attempt rather than the poll interval: " << took.count() << " ms";
        }

        // and the deadline does end it: a short one closes the socket and the next attempt is a different socket
        TcpByteSource impatient({{"endpoint", endpointFor(blocked.port)}, {"bind", false}, {"queue_bytes", gr::Size_t{4096U}}, {"reconnect_ms", gr::Size_t{10U}}, {"connect_timeout_ms", gr::Size_t{150U}}});
        impatient.settings().init();
        std::ignore = impatient.settings().applyStagedParameters();
        impatient.start();

        std::vector<std::uint16_t> initial;
        expect(waitFor([&initial, &blocked] {
            initial = pendingConnectPorts(blocked.descriptors());
            return initial.size() == 1UZ;
        })) << "the source began no connection that stayed unanswered";
        expect(waitFor([&initial, &blocked] {
            const std::vector<std::uint16_t> now = pendingConnectPorts(blocked.descriptors());
            return now.size() == 1UZ && now.front() != initial.front();
        })) << "connect_timeout_ms never ended the attempt, so a peer that has gone would never be tried again";
        impatient.stop();
    };

    // The other half of the same path: an answered connect is taken at once, with no interval spent waiting on it.
    "a connecting source takes a connection the peer answers"_test = [] {
        const int listener = ::socket(AF_INET, SOCK_STREAM, 0);
        expect(listener >= 0) << "the test could not open a listening socket";
        ::sockaddr_in address{};
        address.sin_family      = AF_INET;
        address.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
        address.sin_port        = 0;
        expect(::bind(listener, reinterpret_cast<const ::sockaddr*>(&address), static_cast<::socklen_t>(sizeof(address))) == 0);
        expect(::listen(listener, 4) == 0);
        ::socklen_t length = static_cast<::socklen_t>(sizeof(address));
        expect(::getsockname(listener, reinterpret_cast<::sockaddr*>(&address), &length) == 0);
        const std::uint16_t port = ::ntohs(address.sin_port);

        TcpByteSource source({{"endpoint", endpointFor(port)}, {"bind", false}, {"queue_bytes", gr::Size_t{4096U}}, {"reconnect_ms", gr::Size_t{10U}}});
        source.settings().init();
        std::ignore = source.settings().applyStagedParameters();
        source.start();

        expect(waitFor([listener] {
            ::pollfd item{.fd = listener, .events = POLLIN, .revents = 0};
            return ::poll(&item, 1UL, 50) > 0;
        })) << "the source never connected to a listener that answers";
        const int accepted = ::accept(listener, nullptr, nullptr);
        expect(accepted >= 0) << "the test could not accept the source's connection";

        const std::vector<std::uint8_t> bytes{0x11U, 0x22U, 0x33U};
        expect(::send(accepted, bytes.data(), bytes.size(), MSG_NOSIGNAL) == static_cast<::ssize_t>(bytes.size()));
        expect(waitFor([&source, &bytes] { return source.counters().bytes >= bytes.size(); })) << "the connection was taken but nothing was read from it";

        source.stop();
        std::ignore = ::close(accepted);
        std::ignore = ::close(listener);
    };

    // Backpressure at the source is a read cut to the room the queue has, so a byte off the wire is a byte delivered.
    "the source's backpressure stops reading rather than dropping what it read"_test = [] {
        constexpr std::size_t kQueueBytes = 64UZ;
        constexpr std::size_t kSent       = 4096UZ;
        const std::uint16_t   port        = reservePort();

        // driven without a graph, so nothing drains the receive queue and the read loop meets a full one
        TcpByteSource source({{"endpoint", endpointFor(port)}, {"bind", true}, {"overflow", std::string("backpressure")}, {"queue_bytes", gr::Size_t{static_cast<std::uint32_t>(kQueueBytes)}}});
        source.settings().init();
        std::ignore = source.settings().applyStagedParameters();
        source.start();

        RawStream peer;
        peer.connectTo(port);
        peer.send(seededStream(kSent));
        expect(waitFor([&source] { return source.counters().bytes >= kQueueBytes; })) << "the source never filled its queue";

        std::this_thread::sleep_for(300ms); // three more poll intervals, in which a loop that read on would have read
        const auto counted = source.counters();
        source.stop();

        expect(eq(counted.bytes, std::uint64_t{kQueueBytes})) << "the source read more than its queue had room for";
        expect(eq(counted.bytesDropped, std::uint64_t{0ULL})) << "backpressure shed bytes it had already taken off the wire";
        expect(eq(counted.socketErrors, std::uint64_t{0ULL}));
    };
};

int main() { /* not needed for UT */ }
