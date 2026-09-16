#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <format>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
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
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>

#include <gnuradio-4.0/network/SocketPacketIO.hpp>

// A transport is only proved by a peer it did not build, so the assertions below meet the blocks at a socket rather
// than at a span: a raw TCP connection and a raw UDP socket in the test stand in for the far process, and the framing
// properties are asserted through bytes that were actually written to and read from a descriptor. Nothing here sleeps
// to wait for an arrival; every wait ends on the arrival or on a deadline that fails the test.

namespace {

using namespace std::chrono_literals;
using gr::blocks::network::TcpPacketSink;
using gr::blocks::network::TcpPacketSource;
using gr::blocks::network::UdpPacketSink;
using gr::blocks::network::UdpPacketSource;

// ─── endpoints ────────────────────────────────────────────────────────────────────────────────────────────────────
// A fixed port is a parallel-ctest hazard, so every endpoint below is an ephemeral one the kernel chose. Where the
// test owns the listening end it reads the port back from the socket it bound; where a block owns it, the port is
// reserved by binding one to zero, reading it, and closing it again, which is what leaves the block free to bind it.

[[nodiscard]] std::uint16_t reservePort(int socketType) {
    const int fd = ::socket(AF_INET, socketType, 0);
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

// ─── faulting a descriptor a block owns ───────────────────────────────────────────────────────────────────────────
// A block owns its sockets end to end and lends out no handle to them, which is the property the fatal-exit tests
// have to work around: to break the socket a reader waits on, a test must first name it, and the only description of
// it that leaves the block is the kernel's own. Every open descriptor of this process appears under /proc/self/fd, and
// on a port reserved for one block exactly one of them is bound to it.

/// @brief The socket of this process bound to @p port, of @p socketType, or -1 while there is none.
///
/// A listening TCP socket is asked for by `SO_ACCEPTCONN`, because a connection accepted on it carries the same local
/// port and is not the descriptor these tests mean to break.
[[nodiscard]] int findSocketBoundTo(std::uint16_t port, int socketType) {
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
        int         type     = 0;
        ::socklen_t typeSize = static_cast<::socklen_t>(sizeof(type));
        if (::getsockopt(fd, SOL_SOCKET, SO_TYPE, &type, &typeSize) != 0 || type != socketType) {
            continue;
        }
        if (socketType == SOCK_STREAM) {
            int         listening = 0;
            ::socklen_t flagSize  = static_cast<::socklen_t>(sizeof(listening));
            if (::getsockopt(fd, SOL_SOCKET, SO_ACCEPTCONN, &listening, &flagSize) != 0 || listening == 0) {
                continue;
            }
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

/// @brief Put something that is not a socket at @p fd, so a poll on it reports ready and the next socket call refuses.
///
/// The descriptor number stays taken throughout, so the block's own close is still correct and no other thread can
/// claim the number in between, which closing it outright would allow.
void replaceWithNonSocket(int fd) {
    const int placeholder = ::open("/dev/null", O_RDWR);
    boost::ut::expect(placeholder >= 0) << "the test could not open /dev/null";
    boost::ut::expect(::dup2(placeholder, fd) == fd) << "the test could not put a non-socket at the block's descriptor";
    std::ignore = ::close(placeholder);
}

/// @brief Put a pipe whose write end is already closed at @p fd, so every poll on it reports POLLHUP.
void replaceWithHungUpPipe(int fd) {
    std::array<int, 2UZ> ends{-1, -1};
    boost::ut::expect(::pipe(ends.data()) == 0) << "the test could not make a pipe";
    std::ignore = ::close(ends[1]);
    boost::ut::expect(::dup2(ends[0], fd) == fd) << "the test could not put a hung-up pipe at the block's descriptor";
    std::ignore = ::close(ends[0]);
}

// ─── raw peers, which are the whole point ─────────────────────────────────────────────────────────────────────────

/// @brief A TCP listener the test owns, so a connecting block has something to connect to and the test knows the port.
struct RawListener {
    int           fd   = -1;
    std::uint16_t port = 0;

    RawListener() {
        fd = ::socket(AF_INET, SOCK_STREAM, 0);
        boost::ut::expect(fd >= 0);
        const int on = 1;
        std::ignore  = ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, static_cast<::socklen_t>(sizeof(on)));
        ::sockaddr_in address{};
        address.sin_family      = AF_INET;
        address.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
        address.sin_port        = 0;
        boost::ut::expect(::bind(fd, reinterpret_cast<const ::sockaddr*>(&address), static_cast<::socklen_t>(sizeof(address))) == 0);
        boost::ut::expect(::listen(fd, 8) == 0);
        ::socklen_t length = static_cast<::socklen_t>(sizeof(address));
        boost::ut::expect(::getsockname(fd, reinterpret_cast<::sockaddr*>(&address), &length) == 0);
        port = ::ntohs(address.sin_port);
    }

    RawListener(const RawListener&)            = delete;
    RawListener& operator=(const RawListener&) = delete;
    ~RawListener() {
        if (fd >= 0) {
            std::ignore = ::close(fd);
        }
    }

    [[nodiscard]] std::string endpoint() const { return endpointFor(port); }

    /// @brief Accept one connection, or return -1 when the deadline wins.
    [[nodiscard]] int acceptOne(std::chrono::milliseconds deadline = 5000ms) const {
        ::pollfd  item{.fd = fd, .events = POLLIN, .revents = 0};
        const int ready = ::poll(&item, 1UL, static_cast<int>(deadline.count()));
        if (ready <= 0) {
            return -1;
        }
        return ::accept(fd, nullptr, nullptr);
    }
};

/// @brief One end of a TCP connection the test drives byte by byte.
struct RawStream {
    int fd = -1;

    RawStream() = default;
    explicit RawStream(int descriptor) noexcept : fd(descriptor) {}
    RawStream(const RawStream&)            = delete;
    RawStream& operator=(const RawStream&) = delete;
    ~RawStream() { close(); }

    /// @brief Connect to a listening port, retrying until the deadline so a block's listener may still be opening.
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
                const int on = 1;
                std::ignore  = ::setsockopt(candidate, IPPROTO_TCP, TCP_NODELAY, &on, static_cast<::socklen_t>(sizeof(on)));
                fd           = candidate;
                return;
            }
            std::ignore = ::close(candidate);
            std::this_thread::yield();
        }
        boost::ut::expect(fd >= 0) << "the test could not connect to port" << port;
    }

    void write(std::span<const std::uint8_t> bytes) const {
        std::size_t sent = 0UZ;
        while (sent < bytes.size()) {
            const ::ssize_t written = ::send(fd, bytes.data() + sent, bytes.size() - sent, MSG_NOSIGNAL);
            if (written > 0) {
                sent += static_cast<std::size_t>(written);
                continue;
            }
            if (written < 0 && (errno == EAGAIN || errno == EINTR)) {
                continue;
            }
            boost::ut::expect(false) << "a raw stream could not place its bytes";
            return;
        }
    }

    /// @brief Read until @p count bytes have arrived or the deadline wins; the shortfall is the caller's to judge.
    [[nodiscard]] std::vector<std::uint8_t> read(std::size_t count, std::chrono::milliseconds deadline = 5000ms) const {
        std::vector<std::uint8_t> bytes;
        const auto                until = std::chrono::steady_clock::now() + deadline;
        while (bytes.size() < count && std::chrono::steady_clock::now() < until) {
            ::pollfd  item{.fd = fd, .events = POLLIN, .revents = 0};
            const int ready = ::poll(&item, 1UL, 100);
            if (ready <= 0) {
                continue;
            }
            std::array<std::uint8_t, 4096UZ> chunk{};
            const ::ssize_t                  received = ::recv(fd, chunk.data(), std::min(chunk.size(), count - bytes.size()), 0);
            if (received <= 0) {
                break;
            }
            bytes.insert(bytes.end(), chunk.begin(), chunk.begin() + received);
        }
        return bytes;
    }

    void close() {
        if (fd >= 0) {
            std::ignore = ::close(fd);
            fd          = -1;
        }
    }
};

/// @brief A UDP socket the test owns, which both sends datagrams to a block and receives them from one.
struct RawDatagram {
    int           fd   = -1;
    std::uint16_t port = 0;

    RawDatagram() {
        fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        boost::ut::expect(fd >= 0);
        ::sockaddr_in address{};
        address.sin_family      = AF_INET;
        address.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
        address.sin_port        = 0;
        boost::ut::expect(::bind(fd, reinterpret_cast<const ::sockaddr*>(&address), static_cast<::socklen_t>(sizeof(address))) == 0);
        ::socklen_t length = static_cast<::socklen_t>(sizeof(address));
        boost::ut::expect(::getsockname(fd, reinterpret_cast<::sockaddr*>(&address), &length) == 0);
        port = ::ntohs(address.sin_port);
    }

    RawDatagram(const RawDatagram&)            = delete;
    RawDatagram& operator=(const RawDatagram&) = delete;
    ~RawDatagram() {
        if (fd >= 0) {
            std::ignore = ::close(fd);
        }
    }

    [[nodiscard]] std::string endpoint() const { return endpointFor(port); }

    void sendTo(std::uint16_t target, std::span<const std::uint8_t> bytes) const {
        ::sockaddr_in address{};
        address.sin_family      = AF_INET;
        address.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
        address.sin_port        = ::htons(target);
        const ::ssize_t written = ::sendto(fd, bytes.data(), bytes.size(), MSG_NOSIGNAL, reinterpret_cast<const ::sockaddr*>(&address), static_cast<::socklen_t>(sizeof(address)));
        boost::ut::expect(written >= 0) << "the test could not place a datagram";
    }
};

// ─── a source of prepared packets, which never ends so the test decides when the graph stops ──────────────────────

template<typename T>
struct PacketVectorSource : gr::Block<PacketVectorSource<T>> {
    gr::PortOut<gr::Packet<T>> out;

    GR_MAKE_REFLECTABLE(PacketVectorSource, out);

    std::vector<gr::Packet<T>> _packets{};
    bool                       _repeat = false;
    std::size_t                _next   = 0UZ;
    std::atomic<std::uint64_t> _emitted{0ULL};

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (_packets.empty() || (!_repeat && _next >= _packets.size())) {
            outSpan.publish(0UZ);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS; // idle rather than done: the test owns the teardown
        }
        const std::size_t room = _repeat ? outSpan.size() : std::min(outSpan.size(), _packets.size() - _next);
        for (std::size_t i = 0UZ; i < room; ++i) {
            outSpan[i] = _packets[(_next + i) % _packets.size()];
        }
        _next += room;
        _emitted.fetch_add(static_cast<std::uint64_t>(room));
        outSpan.publish(room);
        return room == 0UZ ? gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS : gr::work::Status::OK;
    }
};

template<typename T>
struct PacketVectorSink : gr::Block<PacketVectorSink<T>> {
    gr::PortIn<gr::Packet<T>> in;

    GR_MAKE_REFLECTABLE(PacketVectorSink, in);

    mutable std::mutex         _mutex;
    std::vector<gr::Packet<T>> _packets{};

    [[nodiscard]] std::size_t count() const {
        std::lock_guard lock(_mutex);
        return _packets.size();
    }

    [[nodiscard]] std::vector<gr::Packet<T>> take() const {
        std::lock_guard lock(_mutex);
        return _packets;
    }

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        {
            std::lock_guard lock(_mutex);
            for (std::size_t i = 0UZ; i < inSpan.size(); ++i) {
                _packets.push_back(inSpan[i]);
            }
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

// ─── running a graph beside the test's own socket ─────────────────────────────────────────────────────────────────

struct GraphRunner {
    gr::scheduler::Simple<> scheduler;
    std::thread             worker;
    std::atomic<bool>       finished{false};
    std::string             failure{};

    explicit GraphRunner(gr::Graph&& graph) {
        boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
        worker = std::thread([this] {
            const auto result = scheduler.runAndWait();
            if (!result.has_value()) {
                failure = std::format("{}", result.error());
            }
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

/// @brief Wait until @p ready holds, ending on the condition rather than on a duration. False means the deadline won.
template<typename F>
[[nodiscard]] bool waitFor(F&& ready, std::chrono::milliseconds deadline = 5000ms) {
    const auto until = std::chrono::steady_clock::now() + deadline;
    while (std::chrono::steady_clock::now() < until) {
        if (ready()) {
            return true;
        }
        std::this_thread::yield();
    }
    return ready();
}

// ─── envelopes built by hand, so a stream can be malformed in ways no sink would produce ──────────────────────────

/// @brief The message bound every source below runs under, well above anything these tests send.
constexpr std::uint64_t kBound = 1ULL << 20U;

[[nodiscard]] std::vector<std::uint8_t> bytesOf(std::string_view text) { //
    return std::vector<std::uint8_t>(reinterpret_cast<const std::uint8_t*>(text.data()), reinterpret_cast<const std::uint8_t*>(text.data()) + text.size());
}

/// @brief Recompute the header CRC after a test has edited a covered byte, so a field refusal is reached rather than
/// the CRC catching the edit first.
void reseal(std::vector<std::uint8_t>& header) {
    const std::uint32_t crc = static_cast<std::uint32_t>(gr::network::kHeaderCrc.compute(std::span<const std::uint8_t>(header.data(), 28UZ)));
    header[28UZ]            = static_cast<std::uint8_t>(crc & 0xFFU);
    header[29UZ]            = static_cast<std::uint8_t>((crc >> 8U) & 0xFFU);
    header[30UZ]            = static_cast<std::uint8_t>((crc >> 16U) & 0xFFU);
    header[31UZ]            = static_cast<std::uint8_t>((crc >> 24U) & 0xFFU);
}

[[nodiscard]] std::vector<std::uint8_t> headerBytes(std::uint16_t itemType, std::uint8_t itemSize, std::uint32_t itemCount, std::uint32_t payloadBytes, std::uint32_t metaBytes) {
    gr::network::EnvelopeHeader header;
    header.item_type     = itemType;
    header.item_size     = itemSize;
    header.item_count    = itemCount;
    header.payload_bytes = payloadBytes;
    header.meta_bytes    = metaBytes;
    const auto encoded   = gr::network::encodeHeader(header);
    return std::vector<std::uint8_t>(encoded.begin(), encoded.end());
}

[[nodiscard]] std::string metadataFor(std::uint64_t sequence, std::string_view sourceId) {
    gr::property_map map;
    map.insert_or_assign(gr::property_map::key_type("sequence"), gr::pmt::Value(sequence));
    map.insert_or_assign(gr::property_map::key_type("source_id"), gr::pmt::Value(std::string(sourceId)));
    return gr::pmt::yaml::serialize(map);
}

/// @brief A well-formed uint8 envelope of @p payload, laid out as the three parts go on a wire.
[[nodiscard]] std::vector<std::uint8_t> uint8Envelope(std::span<const std::uint8_t> payload, std::uint64_t sequence, std::string_view sourceId = "peer") {
    const std::string               metadata      = metadataFor(sequence, sourceId);
    std::vector<std::uint8_t>       envelope      = headerBytes(6U, 1U, static_cast<std::uint32_t>(payload.size()), static_cast<std::uint32_t>(payload.size()), static_cast<std::uint32_t>(metadata.size()));
    const std::vector<std::uint8_t> metadataBytes = bytesOf(metadata);
    envelope.insert(envelope.end(), metadataBytes.begin(), metadataBytes.end());
    envelope.insert(envelope.end(), payload.begin(), payload.end());
    return envelope;
}

// ─── prepared packets and the small readers the assertions use ────────────────────────────────────────────────────

/// @brief A packet of @p nItems bytes counting up from @p first, carrying @p sequence when it is stated.
[[nodiscard]] gr::Packet<std::uint8_t> countingPacket(std::size_t nItems, std::uint8_t first, std::optional<std::uint64_t> sequence = std::nullopt) {
    gr::Packet<std::uint8_t> packet;
    packet.signal_values.resize(nItems);
    for (std::size_t i = 0UZ; i < nItems; ++i) {
        packet.signal_values[i] = static_cast<std::uint8_t>(first + i);
    }
    packet.meta_information.resize(1UZ);
    packet.meta_information[0UZ].insert_or_assign(gr::property_map::key_type("source_id"), gr::pmt::Value(std::string("qa")));
    if (sequence.has_value()) {
        packet.meta_information[0UZ].insert_or_assign(gr::property_map::key_type("sequence"), gr::pmt::Value(*sequence));
    }
    return packet;
}

[[nodiscard]] std::string stringValue(const gr::property_map& map, std::string_view key) {
    const auto entry = map.find(key);
    if (entry == map.end()) {
        return {};
    }
    const std::pmr::string* value = entry->second.get_if<std::pmr::string>();
    return value == nullptr ? std::string{} : std::string(value->begin(), value->end());
}

[[nodiscard]] std::string reasonOf(const gr::Packet<std::uint8_t>& packet) { //
    return packet.meta_information.empty() ? std::string{} : stringValue(packet.meta_information[0UZ], "discard_reason");
}

[[nodiscard]] std::uint64_t sequenceOf(const gr::Packet<std::uint8_t>& packet) { //
    return packet.meta_information.empty() ? ~0ULL : packet.meta_information[0UZ].at("sequence").value_or<std::uint64_t>(~0ULL);
}

} // namespace

const boost::ut::suite<"TcpPacketIO"> tcpPacketIoTests = [] {
    using namespace boost::ut;

    // The loopback both ways round: which end listens is a deployment choice, so the crossing must not depend on it.
    "a packet vector crosses a TCP pair in either role assignment"_test = [] {
        const auto crossing = [](bool sinkBinds) {
            const std::uint16_t                   port = reservePort(SOCK_STREAM);
            std::vector<gr::Packet<std::uint8_t>> sent;
            for (std::uint32_t i = 0U; i < 64U; ++i) {
                const std::size_t nItems = (i == 17U) ? 1UZ : (1UZ + (i % 23UZ)); // one packet of a single item, deliberately
                sent.push_back(countingPacket(nItems, static_cast<std::uint8_t>(i)));
            }
            sent[3UZ].timestamp = 1'700'000'000'000'000'000;

            gr::Graph graph;
            auto&     producer = graph.emplaceBlock<PacketVectorSource<std::uint8_t>>();
            producer._packets  = sent;
            auto& sink         = graph.emplaceBlock<TcpPacketSink<std::uint8_t>>({{"endpoint", endpointFor(port)}, {"bind", sinkBinds}, {"overflow", std::string("backpressure")}});
            auto& source       = graph.emplaceBlock<TcpPacketSource<std::uint8_t>>({{"endpoint", endpointFor(port)}, {"bind", !sinkBinds}, {"max_message_bytes", kBound}});
            auto& collector    = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
            expect(graph.connect<"out", "in">(producer, sink).has_value());
            expect(graph.connect<"out", "in">(source, collector).has_value());

            GraphRunner runner(std::move(graph));
            expect(waitFor([&collector] { return collector.count() >= 64UZ; }, 15000ms)) << std::format("sink binds {}: only {} of 64 packets crossed", sinkBinds, collector.count());
            runner.stop();

            const std::vector<gr::Packet<std::uint8_t>> received = collector.take();
            expect(eq(received.size(), 64UZ)) << std::format("sink binds {}", sinkBinds);
            for (std::size_t i = 0UZ; i < std::min(received.size(), sent.size()); ++i) {
                expect(std::ranges::equal(received[i].signal_values, sent[i].signal_values)) << std::format("sink binds {}: packet {} payload", sinkBinds, i);
                expect(eq(received[i].meta_information.size(), 1UZ));
                if (received[i].meta_information.empty()) {
                    continue;
                }
                expect(eq(stringValue(received[i].meta_information[0UZ], "source_id"), std::string("qa"))) << std::format("sink binds {}: packet {} source_id", sinkBinds, i);
                expect(eq(sequenceOf(received[i]), static_cast<std::uint64_t>(i))) << std::format("sink binds {}: packet {} sequence", sinkBinds, i);
                expect(!received[i].meta_information[0UZ].contains("packet_timestamp")) << "the crossing key was left in the map";
            }
            if (received.size() > 3UZ) {
                expect(eq(received[3UZ].timestamp, std::int64_t{1'700'000'000'000'000'000})) << "the carrier's timestamp did not cross";
                expect(eq(received[2UZ].timestamp, std::int64_t{0})) << "a zero timestamp gained a value it never had";
            }
            expect(eq(source.nSequenceGaps, std::uint64_t{0ULL})) << "a lossless link reported loss";
            expect(eq(source.nTruncatedEnvelopes, std::uint64_t{0ULL}));
            expect(eq(source.nResyncs, std::uint64_t{0ULL})) << "an unbroken stream needed a resynchronization";
            expect(eq(source.nBytesSkipped, std::uint64_t{0ULL}));
            expect(eq(sink.nSendErrors, std::uint64_t{0ULL}));
            expect(eq(sink.nTimestampsCarried, std::uint64_t{1ULL}));
            expect(eq(source.nTimestampsCarried, std::uint64_t{1ULL}));
        };

        crossing(true);
        crossing(false);
    };

    // What ZeroMQ's atomicity paid for, paid here instead by the magic scan: the reader finds the envelopes in a
    // stream that also carries bytes it was never meant to read, and a payload that spells the magic is stepped over
    // by length rather than mistaken for a header.
    "garbage between envelopes is skipped and counted, and a magic inside a payload does not desynchronize"_test = [] {
        const std::uint16_t port = reservePort(SOCK_STREAM);

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<TcpPacketSource<std::uint8_t>>({{"endpoint", endpointFor(port)}, {"bind", true}, {"max_message_bytes", kBound}});
        auto&     collector = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        auto&     refusals  = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(source, collector).has_value());
        expect(graph.connect<"reject", "in">(source, refusals).has_value());

        GraphRunner runner(std::move(graph));
        RawStream   peer;
        peer.connectTo(port);

        const std::vector<std::uint8_t>   leading(11UZ, 0x5AU); // bytes that cannot begin the magic
        const std::vector<std::uint8_t>   between(7UZ, 0x21U);
        const std::array<std::uint8_t, 4> payloadOne{1U, 2U, 3U, 4U};
        const std::array<std::uint8_t, 8> payloadTwo{0x47U, 0x52U, 0x34U, 0x50U, 0xFFU, 0x00U, 0x11U, 0x22U};

        std::vector<std::uint8_t> stream = leading;
        const auto                first  = uint8Envelope(payloadOne, 1ULL);
        stream.insert(stream.end(), first.begin(), first.end());
        stream.insert(stream.end(), between.begin(), between.end());
        const auto second = uint8Envelope(payloadTwo, 2ULL);
        stream.insert(stream.end(), second.begin(), second.end());
        peer.write(stream);

        expect(waitFor([&collector] { return collector.count() >= 2UZ; })) << "only" << collector.count() << "of 2 envelopes were found in the stream";
        runner.stop();

        const std::vector<gr::Packet<std::uint8_t>> received = collector.take();
        expect(eq(received.size(), 2UZ));
        if (received.size() == 2UZ) {
            expect(std::ranges::equal(received[0UZ].signal_values, payloadOne));
            expect(std::ranges::equal(received[1UZ].signal_values, payloadTwo)) << "a payload spelling the magic did not survive";
            expect(eq(sequenceOf(received[0UZ]), std::uint64_t{1ULL}));
            expect(eq(sequenceOf(received[1UZ]), std::uint64_t{2ULL}));
        }
        expect(eq(source.nBytesSkipped, std::uint64_t{18ULL})) << "the scanned-past bytes were not counted exactly";
        expect(eq(source.nResyncs, std::uint64_t{1ULL})) << "garbage at a connection's start is not a resynchronization, garbage between envelopes is";
        expect(eq(source.nEnvelopesReceived, std::uint64_t{2ULL}));
        expect(eq(refusals.count(), 0UZ)) << "a scanned-past byte is not a refused envelope";
    };

    // The resynchronization the redundant length field exists for: a CRC-valid header is trusted to size the skip, so
    // a refusal costs one envelope and not the stream.
    "an envelope over the bound is skipped by length and the next one arrives intact"_test = [] {
        const std::uint16_t port = reservePort(SOCK_STREAM);

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<TcpPacketSource<std::uint8_t>>({{"endpoint", endpointFor(port)}, {"bind", true}, {"max_message_bytes", static_cast<std::uint64_t>(512)}});
        auto&     collector = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        auto&     refusals  = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(source, collector).has_value());
        expect(graph.connect<"reject", "in">(source, refusals).has_value());

        GraphRunner runner(std::move(graph));
        RawStream   peer;
        peer.connectTo(port);

        const std::vector<std::uint8_t>   oversize(4096UZ, 0xA5U);
        const std::array<std::uint8_t, 4> small{7U, 8U, 9U, 10U};
        std::vector<std::uint8_t>         stream = uint8Envelope(oversize, 1ULL);
        const auto                        next   = uint8Envelope(small, 2ULL);
        stream.insert(stream.end(), next.begin(), next.end());
        peer.write(stream);

        expect(waitFor([&collector] { return collector.count() >= 1UZ; })) << "the envelope after the refused one never arrived";
        runner.stop();

        const std::vector<gr::Packet<std::uint8_t>> received = collector.take();
        expect(eq(received.size(), 1UZ));
        if (received.size() == 1UZ) {
            expect(std::ranges::equal(received[0UZ].signal_values, small)) << "the envelope after the refused one was not intact";
            expect(eq(sequenceOf(received[0UZ]), std::uint64_t{2ULL}));
        }
        expect(eq(source.nOverMax, std::uint64_t{1ULL}));
        expect(eq(source.nEnvelopesReceived, std::uint64_t{2ULL}));
        expect(eq(source.nBytesSkipped, std::uint64_t{0ULL})) << "a body skipped by length is not scanned past";
        expect(eq(source.nResyncs, std::uint64_t{0ULL})) << "a length-trusted skip must leave the stream synchronized";
        expect(eq(refusals.count(), 0UZ)) << "a body the reader never buffered must not be published as a reject";
    };

    // A connection that ends mid-envelope loses that envelope and says so, and what was lost is also readable at the
    // far end as a gap in `sequence` — which is the whole of the loss model over a connection-oriented transport.
    "a disconnect mid-envelope is counted and the stream recovers on the next connection"_test = [] {
        const std::uint16_t port = reservePort(SOCK_STREAM);

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<TcpPacketSource<std::uint8_t>>({{"endpoint", endpointFor(port)}, {"bind", true}, {"max_message_bytes", kBound}});
        auto&     collector = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(source, collector).has_value());

        GraphRunner runner(std::move(graph));
        {
            RawStream peer;
            peer.connectTo(port);
            const std::array<std::uint8_t, 2> payload{1U, 2U};
            peer.write(uint8Envelope(payload, 1ULL));
            expect(waitFor([&collector] { return collector.count() >= 1UZ; })) << "the first envelope did not arrive";

            const std::vector<std::uint8_t> wide(16UZ, 0x3CU);
            const std::vector<std::uint8_t> partial = uint8Envelope(wide, 2ULL);
            expect(gt(partial.size(), 40UZ));
            peer.write(std::span<const std::uint8_t>(partial).first(40UZ)); // a header and part of a body, and no more
            expect(waitFor([&source] { return source.nEnvelopesReceived >= 2ULL; })) << "the truncated envelope's header was never read";
        }
        expect(waitFor([&source] { return source.nTruncatedEnvelopes >= 1ULL; })) << "a connection that ended mid-envelope was not counted";

        RawStream rejoined;
        rejoined.connectTo(port);
        const std::array<std::uint8_t, 3> resumed{9U, 9U, 9U};
        rejoined.write(uint8Envelope(resumed, 5ULL));
        expect(waitFor([&collector] { return collector.count() >= 2UZ; })) << "the source did not serve a peer that connected after the disconnection";
        runner.stop();

        expect(eq(source.nTruncatedEnvelopes, std::uint64_t{1ULL}));
        expect(ge(source.nDisconnects, std::uint64_t{1ULL}));
        expect(ge(source.nReconnects, std::uint64_t{1ULL}));
        expect(eq(source.nSequenceGaps, std::uint64_t{1ULL})) << "the loss was not visible in sequence";
        expect(eq(source.nPacketsLost, std::uint64_t{3ULL})) << "the gap between sequence 1 and sequence 5 is three packets";
    };
};

const boost::ut::suite<"UdpPacketIO"> udpPacketIoTests = [] {
    using namespace boost::ut;

    // The datagram loopback, in the arrangement that has no third option: the source binds and the sink sends. The
    // source's graph is started first and waited for, so no datagram is aimed at a port nothing is listening on.
    "a packet vector crosses a UDP pair"_test = [] {
        // the block binds the port and the sink must name it, so it is reserved here rather than left to the kernel
        const std::uint16_t port = reservePort(SOCK_DGRAM);

        gr::Graph receiving;
        auto&     source    = receiving.emplaceBlock<UdpPacketSource<std::uint8_t>>({{"endpoint", endpointFor(port)}, {"max_message_bytes", kBound}});
        auto&     collector = receiving.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(receiving.connect<"out", "in">(source, collector).has_value());

        GraphRunner receiver(std::move(receiving));
        expect(waitFor([&source] { return source._socketOpen; })) << "the source never bound its endpoint";

        std::vector<gr::Packet<std::uint8_t>> sent;
        for (std::uint32_t i = 0U; i < 64U; ++i) {
            sent.push_back(countingPacket(1UZ + (i % 19UZ), static_cast<std::uint8_t>(i)));
        }
        sent[5UZ].timestamp = -1;

        gr::Graph sending;
        auto&     producer = sending.emplaceBlock<PacketVectorSource<std::uint8_t>>();
        producer._packets  = sent;
        auto& sink         = sending.emplaceBlock<UdpPacketSink<std::uint8_t>>({{"endpoint", endpointFor(port)}, {"overflow", std::string("backpressure")}});
        expect(sending.connect<"out", "in">(producer, sink).has_value());

        GraphRunner sender(std::move(sending));
        expect(waitFor([&collector] { return collector.count() >= 64UZ; })) << "only" << collector.count() << "of 64 datagrams crossed";
        sender.stop();
        receiver.stop();

        const std::vector<gr::Packet<std::uint8_t>> received = collector.take();
        expect(eq(received.size(), 64UZ));
        for (std::size_t i = 0UZ; i < std::min(received.size(), sent.size()); ++i) {
            expect(std::ranges::equal(received[i].signal_values, sent[i].signal_values)) << "datagram" << i << "payload";
            expect(eq(sequenceOf(received[i]), static_cast<std::uint64_t>(i))) << "datagram" << i << "sequence";
            expect(eq(stringValue(received[i].meta_information[0UZ], "source_id"), std::string("qa")));
        }
        if (received.size() > 5UZ) {
            expect(eq(received[5UZ].timestamp, std::int64_t{-1})) << "the carrier's timestamp did not cross";
        }
        expect(eq(source.nSequenceGaps, std::uint64_t{0ULL})) << "a loopback datagram link lost something";
        expect(eq(sink.nSendErrors, std::uint64_t{0ULL}));
    };

    // The sink's own refusal name, which is the datagram spelling of the stream transports' size bound.
    "an envelope over max_datagram_bytes is refused at the sink"_test = [] {
        const std::uint16_t port = reservePort(SOCK_DGRAM);

        gr::Graph graph;
        auto&     producer = graph.emplaceBlock<PacketVectorSource<std::uint8_t>>();
        producer._packets  = {countingPacket(4096UZ, 0U), countingPacket(4UZ, 1U)};
        auto& sink         = graph.emplaceBlock<UdpPacketSink<std::uint8_t>>({{"endpoint", endpointFor(port)}, {"max_datagram_bytes", static_cast<std::uint64_t>(512)}});
        auto& refusals     = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(producer, sink).has_value());
        expect(graph.connect<"reject", "in">(sink, refusals).has_value());

        GraphRunner runner(std::move(graph));
        expect(waitFor([&refusals] { return refusals.count() >= 1UZ; })) << "the oversize packet never reached the failure port";
        runner.stop();

        expect(eq(sink.nRejectedPackets, std::uint64_t{1ULL}));
        const std::vector<gr::Packet<std::uint8_t>> refused = refusals.take();
        expect(eq(refused.size(), 1UZ));
        if (refused.size() == 1UZ) {
            expect(eq(reasonOf(refused[0UZ]), std::string("over_max_datagram")));
            expect(eq(refused[0UZ].signal_values.size(), 4096UZ)) << "a refused packet is republished whole";
        }
    };

    // The check a byte stream cannot make and a datagram can: the header's lengths and the datagram's own are two
    // statements about one message, and they must agree.
    "a datagram whose length disagrees with its header is refused"_test = [] {
        const std::uint16_t port = reservePort(SOCK_DGRAM);

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<UdpPacketSource<std::uint8_t>>({{"endpoint", endpointFor(port)}, {"max_message_bytes", kBound}});
        auto&     collector = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        auto&     refusals  = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(source, collector).has_value());
        expect(graph.connect<"reject", "in">(source, refusals).has_value());

        GraphRunner runner(std::move(graph));
        expect(waitFor([&source] { return source._socketOpen; }));

        const RawDatagram peer;
        // a header that decodes cleanly and states four payload bytes, in a datagram that carries eight
        const std::string               metadata  = metadataFor(1ULL, "peer");
        std::vector<std::uint8_t>       wrong     = headerBytes(6U, 1U, 4U, 4U, static_cast<std::uint32_t>(metadata.size()));
        const std::vector<std::uint8_t> metaBytes = bytesOf(metadata);
        wrong.insert(wrong.end(), metaBytes.begin(), metaBytes.end());
        wrong.insert(wrong.end(), 8UZ, 0x5EU);
        peer.sendTo(port, wrong);

        expect(waitFor([&refusals] { return refusals.count() >= 1UZ; })) << "the mismatched datagram was not refused";

        const std::array<std::uint8_t, 3> good{4U, 5U, 6U};
        peer.sendTo(port, uint8Envelope(good, 2ULL));
        expect(waitFor([&collector] { return collector.count() >= 1UZ; })) << "the reader did not recover";
        runner.stop();

        expect(eq(source.nLengthMismatch, std::uint64_t{1ULL}));
        const std::vector<gr::Packet<std::uint8_t>> refused = refusals.take();
        expect(eq(refused.size(), 1UZ));
        if (refused.size() == 1UZ) {
            const std::uint64_t total = wrong.size();
            expect(eq(reasonOf(refused[0UZ]), std::string("length_mismatch")));
            expect(eq(refused[0UZ].meta_information[0UZ].at("envelope_bytes_total").value_or<std::uint64_t>(0ULL), total));
            expect(eq(refused[0UZ].meta_information[0UZ].at("envelope_bytes_kept").value_or<std::uint64_t>(0ULL), std::min(total, std::uint64_t{256ULL})));
            expect(!refused[0UZ].meta_information[0UZ].contains("envelope_frames")) << "a frame count is a ZeroMQ fact and does not belong here";
        }
    };

    // A datagram larger than the bound is named from a bounded copy: the true length comes back from the kernel, so
    // nothing the peer claimed ever sizes an allocation in this process.
    "an oversize datagram is refused from a bounded copy"_test = [] {
        const std::uint16_t port = reservePort(SOCK_DGRAM);

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<UdpPacketSource<std::uint8_t>>({{"endpoint", endpointFor(port)}, {"max_message_bytes", static_cast<std::uint64_t>(512)}});
        auto&     collector = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        auto&     refusals  = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(source, collector).has_value());
        expect(graph.connect<"reject", "in">(source, refusals).has_value());

        GraphRunner runner(std::move(graph));
        expect(waitFor([&source] { return source._socketOpen; }));

        const RawDatagram               peer;
        const std::vector<std::uint8_t> oversize(4096UZ, 0xA5U);
        const std::vector<std::uint8_t> datagram = uint8Envelope(oversize, 1ULL);
        peer.sendTo(port, datagram);
        expect(waitFor([&refusals] { return refusals.count() >= 1UZ; })) << "the oversize datagram was not refused";
        runner.stop();

        expect(eq(source.nOverMax, std::uint64_t{1ULL}));
        expect(eq(collector.count(), 0UZ)) << "a refused datagram appeared on out";
        const std::vector<gr::Packet<std::uint8_t>> refused = refusals.take();
        expect(eq(refused.size(), 1UZ));
        if (refused.size() == 1UZ) {
            expect(eq(reasonOf(refused[0UZ]), std::string("over_max")));
            expect(le(refused[0UZ].signal_values.size(), 256UZ)) << "the refusal kept more than max_reject_bytes";
            const std::uint64_t total = datagram.size();
            expect(eq(refused[0UZ].meta_information[0UZ].at("envelope_bytes_total").value_or<std::uint64_t>(0ULL), total)) << "the datagram's true length was not reported from the truncated read";
        }
    };
};

// ─── the golden refusal set, replayed over both transports ────────────────────────────────────────────────────────
// The eleven names a header can earn on its own, each produced by one edit to an otherwise valid header. Over UDP
// every one reaches `reject` with its name, because the datagram is in hand. Over TCP the same corruptions reach the
// same counters, but no reject packet: `bad_magic` is subsumed by the magic scan and the rest are refused before a
// body has been read, so what a reject would carry is bytes the reader deliberately did not buffer.

namespace {

struct Corruption {
    std::string_view          name;
    std::vector<std::uint8_t> header;
};

[[nodiscard]] std::vector<Corruption> goldenRefusals() {
    const auto edited = [](std::size_t at, std::uint8_t value, bool sealAgain) {
        std::vector<std::uint8_t> header = headerBytes(6U, 1U, 0U, 0U, 0U);
        header[at]                       = value;
        if (sealAgain) {
            reseal(header);
        }
        return header;
    };

    std::vector<Corruption> set;
    set.push_back({"bad_magic", edited(0UZ, 0x58U, false)});     // checked before the CRC, so the edit stands unsealed
    set.push_back({"bad_version", edited(4UZ, 0x00U, true)});    // wire_version 0, which is not a version
    set.push_back({"future_version", edited(4UZ, 0x02U, true)}); // a layout this reader cannot guess at
    set.push_back({"byte_order", edited(6UZ, 0x01U, true)});
    set.push_back({"bad_header_bytes", edited(7UZ, 31U, true)});
    set.push_back({"bad_header_crc", edited(12UZ, 0x01U, false)}); // one covered byte changed and the CRC left alone
    set.push_back({"unsupported_item_type", edited(8UZ, 0x00U, true)});
    set.push_back({"bad_item_size", edited(10UZ, 0x02U, true)});
    set.push_back({"payload_length_field", headerBytes(6U, 1U, 4U, 5U, 0U)});
    set.push_back({"unknown_meta_encoding", edited(11UZ, 0x02U, true)});
    set.push_back({"unknown_flags", edited(24UZ, 0x01U, true)});
    return set;
}

} // namespace

const boost::ut::suite<"SocketPacketIOGoldenRefusals"> goldenRefusalTests = [] {
    using namespace boost::ut;

    "every header refusal keeps its name over a raw UDP socket"_test = [] {
        const std::uint16_t port = reservePort(SOCK_DGRAM);

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<UdpPacketSource<std::uint8_t>>({{"endpoint", endpointFor(port)}, {"max_message_bytes", kBound}});
        auto&     collector = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        auto&     refusals  = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(source, collector).has_value());
        expect(graph.connect<"reject", "in">(source, refusals).has_value());

        GraphRunner runner(std::move(graph));
        expect(waitFor([&source] { return source._socketOpen; }));

        const RawDatagram             peer;
        const std::vector<Corruption> golden = goldenRefusals();
        for (std::size_t i = 0UZ; i < golden.size(); ++i) {
            peer.sendTo(port, golden[i].header);
            // one at a time, so the order the names are read back in is the order they were sent in
            expect(waitFor([&refusals, i] { return refusals.count() >= i + 1UZ; })) << "refusal" << golden[i].name << "never arrived";
        }
        runner.stop();

        const std::vector<gr::Packet<std::uint8_t>> refused = refusals.take();
        expect(eq(refused.size(), golden.size()));
        for (std::size_t i = 0UZ; i < std::min(refused.size(), golden.size()); ++i) {
            expect(eq(reasonOf(refused[i]), std::string(golden[i].name))) << "refusal" << i;
        }
        expect(eq(collector.count(), 0UZ)) << "a refused datagram appeared on out";
        expect(eq(source.nBadMagic, std::uint64_t{1ULL}));
        expect(eq(source.nRefusedVersion, std::uint64_t{2ULL})) << "one counter covers both spellings of a version fault";
        expect(eq(source.nBadByteOrder, std::uint64_t{1ULL}));
        expect(eq(source.nBadHeaderBytes, std::uint64_t{1ULL}));
        expect(eq(source.nBadHeaderCrc, std::uint64_t{1ULL}));
        expect(eq(source.nUnsupportedItemType, std::uint64_t{1ULL}));
        expect(eq(source.nBadItemSize, std::uint64_t{1ULL}));
        expect(eq(source.nBadPayloadLength, std::uint64_t{1ULL}));
        expect(eq(source.nUnknownMetaEncoding, std::uint64_t{1ULL}));
        expect(eq(source.nUnknownFlags, std::uint64_t{1ULL}));
    };

    "every header refusal reaches the same counter over a raw TCP connection"_test = [] {
        const std::uint16_t port = reservePort(SOCK_STREAM);

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<TcpPacketSource<std::uint8_t>>({{"endpoint", endpointFor(port)}, {"bind", true}, {"max_message_bytes", kBound}});
        auto&     collector = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        auto&     refusals  = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(source, collector).has_value());
        expect(graph.connect<"reject", "in">(source, refusals).has_value());

        GraphRunner runner(std::move(graph));
        RawStream   peer;
        peer.connectTo(port);

        const std::vector<Corruption> golden = goldenRefusals();
        std::vector<std::uint8_t>     stream;
        for (const Corruption& corruption : golden) {
            stream.insert(stream.end(), corruption.header.begin(), corruption.header.end());
        }
        peer.write(stream);

        // ten of the eleven reach a counter; the bad magic is scanned past instead, and the whole of it is skipped
        expect(waitFor([&source] { return source.nResyncs >= 10ULL; })) << "only" << source.nResyncs << "of 10 header refusals were seen";

        const std::array<std::uint8_t, 2> payload{3U, 4U};
        peer.write(uint8Envelope(payload, 1ULL)); // and the reader is still able to take a legal envelope afterwards
        expect(waitFor([&collector] { return collector.count() >= 1UZ; })) << "the reader did not recover";
        runner.stop();

        expect(eq(refusals.count(), 0UZ)) << "a header refused before its body was read must publish no reject";
        expect(eq(source.nRefusedVersion, std::uint64_t{2ULL}));
        expect(eq(source.nBadByteOrder, std::uint64_t{1ULL}));
        expect(eq(source.nBadHeaderBytes, std::uint64_t{1ULL}));
        expect(eq(source.nBadHeaderCrc, std::uint64_t{1ULL}));
        expect(eq(source.nUnsupportedItemType, std::uint64_t{1ULL}));
        expect(eq(source.nBadItemSize, std::uint64_t{1ULL}));
        expect(eq(source.nBadPayloadLength, std::uint64_t{1ULL}));
        expect(eq(source.nUnknownMetaEncoding, std::uint64_t{1ULL}));
        expect(eq(source.nUnknownFlags, std::uint64_t{1ULL}));
        expect(eq(source.nResyncs, std::uint64_t{10ULL})) << "each refused candidate returns the reader to seeking exactly once";
        expect(eq(source.nPacketsPublished, std::uint64_t{1ULL}));
    };
};

const boost::ut::suite<"SocketPacketIOLifecycle"> socketLifecycleTests = [] {
    using namespace boost::ut;

    "endpoints are parsed and resolved before anything binds"_test = [] {
        const auto refusesTcpSource = [](gr::property_map settings, std::string_view what) {
            gr::Graph graph;
            auto&     block = graph.emplaceBlock<TcpPacketSource<std::uint8_t>>(std::move(settings));
            expect(throws<gr::exception>([&block] { block.start(); })) << what;
        };
        refusesTcpSource({{"endpoint", std::string("")}, {"max_message_bytes", kBound}}, "an empty endpoint");
        refusesTcpSource({{"endpoint", std::string("127.0.0.1:5555")}}, "an unset max_message_bytes");
        refusesTcpSource({{"endpoint", std::string("127.0.0.1")}, {"max_message_bytes", kBound}}, "an endpoint with no port");
        refusesTcpSource({{"endpoint", std::string("no.such.host.invalid:5555")}, {"max_message_bytes", kBound}}, "an endpoint that does not resolve");
        refusesTcpSource({{"endpoint", std::string("127.0.0.1:5555")}, {"max_message_bytes", static_cast<std::uint64_t>(64)}, {"max_reject_bytes", static_cast<gr::Size_t>(256)}}, "a refusal copy larger than a message");

        const auto refusesUdpSink = [](gr::property_map settings, std::string_view what) {
            gr::Graph graph;
            auto&     block = graph.emplaceBlock<UdpPacketSink<std::uint8_t>>(std::move(settings));
            expect(throws<gr::exception>([&block] { block.start(); })) << what;
        };
        refusesUdpSink({{"endpoint", std::string("")}}, "an empty endpoint");
        refusesUdpSink({{"endpoint", std::string("127.0.0.1:5555")}, {"max_datagram_bytes", static_cast<std::uint64_t>(0)}}, "a zero datagram bound");
        refusesUdpSink({{"endpoint", std::string("127.0.0.1:5555")}, {"max_datagram_bytes", static_cast<std::uint64_t>(70000)}}, "a datagram bound above what IPv4 can carry");

        // a bracketed IPv6 literal parses, which is what the last-colon split exists for
        gr::Graph graph;
        auto&     source = graph.emplaceBlock<TcpPacketSource<std::uint8_t>>({{"endpoint", std::format("[::1]:{}", reservePort(SOCK_STREAM))}, {"bind", true}, {"max_message_bytes", kBound}});
        expect(nothrow([&source] { source.start(); })) << "a bracketed IPv6 endpoint did not parse";
        expect(nothrow([&source] { source.stop(); }));
    };

    "a socket setting cannot change under a running block"_test = [] {
        gr::Graph   graph;
        auto&       source = graph.emplaceBlock<TcpPacketSource<std::uint8_t>>({{"endpoint", endpointFor(reservePort(SOCK_STREAM))}, {"bind", true}, {"max_message_bytes", kBound}});
        std::string refusal;
        source.start();
        try {
            source.max_message_bytes.value = 4096ULL;
            source.rebuild();
        } catch (const gr::exception& error) {
            refusal = error.message;
        }
        source.stop();
        expect(refusal.contains("max_message_bytes")) << "a live change to a socket setting was not refused by name: " << refusal;
    };

    "start, stop and start again leave nothing behind"_test = [] {
        const std::uint16_t tcpPort = reservePort(SOCK_STREAM);
        const std::uint16_t udpPort = reservePort(SOCK_DGRAM);

        gr::Graph graph;
        auto&     tcpSource = graph.emplaceBlock<TcpPacketSource<std::uint8_t>>({{"endpoint", endpointFor(tcpPort)}, {"bind", true}, {"max_message_bytes", kBound}});
        auto&     tcpSink   = graph.emplaceBlock<TcpPacketSink<std::uint8_t>>({{"endpoint", endpointFor(tcpPort)}, {"bind", false}});
        auto&     udpSource = graph.emplaceBlock<UdpPacketSource<std::uint8_t>>({{"endpoint", endpointFor(udpPort)}, {"max_message_bytes", kBound}});
        auto&     udpSink   = graph.emplaceBlock<UdpPacketSink<std::uint8_t>>({{"endpoint", endpointFor(udpPort)}});
        for (std::size_t i = 0UZ; i < 10UZ; ++i) {
            expect(nothrow([&tcpSource] { tcpSource.start(); })) << "tcp source start" << i;
            expect(nothrow([&tcpSource] { tcpSource.stop(); })) << "tcp source stop" << i;
            expect(nothrow([&tcpSink] { tcpSink.start(); })) << "tcp sink start" << i;
            expect(nothrow([&tcpSink] { tcpSink.stop(); })) << "tcp sink stop" << i;
            expect(nothrow([&udpSource] { udpSource.start(); })) << "udp source start" << i;
            expect(nothrow([&udpSource] { udpSource.stop(); })) << "udp source stop" << i;
            expect(nothrow([&udpSink] { udpSink.start(); })) << "udp sink start" << i;
            expect(nothrow([&udpSink] { udpSink.stop(); })) << "udp sink stop" << i;
        }
    };

    // A listening end serves one peer at a time, and the extra client is told so at accept rather than left waiting.
    "a second connection to a listening end is refused at accept"_test = [] {
        const std::uint16_t port = reservePort(SOCK_STREAM);

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<TcpPacketSource<std::uint8_t>>({{"endpoint", endpointFor(port)}, {"bind", true}, {"max_message_bytes", kBound}});
        auto&     collector = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(source, collector).has_value());

        GraphRunner runner(std::move(graph));
        RawStream   served;
        served.connectTo(port);
        const std::array<std::uint8_t, 2> payload{1U, 2U};
        served.write(uint8Envelope(payload, 1ULL));
        expect(waitFor([&collector] { return collector.count() >= 1UZ; })) << "the served peer's envelope did not arrive";

        RawStream extra;
        extra.connectTo(port);
        expect(waitFor([&source] { return source.nPeersRefused >= 1ULL; })) << "a second connection was neither served nor refused";

        // and the peer that was already being served is still served
        served.write(uint8Envelope(payload, 2ULL));
        expect(waitFor([&collector] { return collector.count() >= 2UZ; })) << "refusing an extra peer cost the served one its connection";
        runner.stop();

        expect(eq(source.nPeersRefused, std::uint64_t{1ULL}));
        expect(eq(source.nPacketsPublished, std::uint64_t{2ULL}));
    };

    // The fault behind the ERROR status: the listening socket the reader owns stops being one it can accept on. It has
    // two shapes — a socket that has been shut down, which polls as POLLHUP, and a descriptor that is no longer a
    // socket at all, which polls ready and refuses the accept. The first is taken through a whole graph, because the
    // status is what a graph is owed; the second is taken at the block, where the reader's own record is the subject.
    "a TCP source whose listening socket cannot serve again ends its reader and names the fault"_test = [] {
        const std::uint16_t shutPort = reservePort(SOCK_STREAM);

        gr::Graph graph;
        auto&     shutSource = graph.emplaceBlock<TcpPacketSource<std::uint8_t>>({{"endpoint", endpointFor(shutPort)}, {"bind", true}, {"max_message_bytes", kBound}});
        auto&     collector  = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(shutSource, collector).has_value());

        GraphRunner runner(std::move(graph));

        int listener = -1;
        expect(waitFor([&listener, shutPort] {
            listener = findSocketBoundTo(shutPort, SOCK_STREAM);
            return listener >= 0;
        })) << "the source never opened a listening socket on its own port";
        std::ignore = ::shutdown(listener, SHUT_RDWR);

        expect(waitFor([&runner] { return runner.finished.load(); })) << "a listening socket that cannot serve again left the reader running";
        runner.stop();
        expect(!runner.failure.empty()) << "the reader ended on a fault and the graph did not end in error";

        const std::string shutReason = shutSource.lastReaderError();
        expect(shutReason.contains("POLLHUP") || shutReason.contains("POLLERR") || shutReason.contains("POLLNVAL")) << "the reader did not name the flag its listening socket reported: " << shutReason;
        expect(eq(shutSource.nReaderFailures, std::uint64_t{1ULL})) << "one reader ended on a fault, so one failure is counted";
        expect(eq(collector.count(), 0UZ)) << "no peer ever connected, so nothing should have been published";

        // the second shape, driven without a scheduler: what the reader records is the whole of what is under test
        const std::uint16_t           blindPort = reservePort(SOCK_STREAM);
        TcpPacketSource<std::uint8_t> blindSource({{"endpoint", endpointFor(blindPort)}, {"bind", true}, {"max_message_bytes", kBound}});
        blindSource.settings().init();
        std::ignore = blindSource.settings().applyStagedParameters();
        blindSource.start();

        int replaced = -1;
        expect(waitFor([&replaced, blindPort] {
            replaced = findSocketBoundTo(blindPort, SOCK_STREAM);
            return replaced >= 0;
        })) << "the second source never opened a listening socket on its own port";
        replaceWithNonSocket(replaced);

        expect(waitFor([&blindSource] { return !blindSource.lastReaderError().empty(); })) << "an accept that can never succeed again left the reader running";
        const std::string blindReason = blindSource.lastReaderError();
        {
            std::lock_guard lock(blindSource._mutex);
            expect(blindSource._readerFailed) << "the reader ended on a fault without raising the flag processBulk answers on";
        }
        blindSource.stop();

        expect(blindReason.starts_with("accept failed")) << "the reader did not name the accept that refused it: " << blindReason;
        expect(blindReason.contains("socket")) << "the reason drops the system's own wording: " << blindReason;
        expect(eq(blindSource.nReaderFailures, std::uint64_t{1ULL})) << "one reader ended on a fault, so one failure is counted";
    };

    // The other side of that ruling: what a peer does is never a fault of the reader's own socket, however abruptly it
    // does it. A reset is the sharpest form — no orderly close, and the reader learns of it as a failed read.
    "a peer that resets its connection costs the connection and not the reader"_test = [] {
        const std::uint16_t port = reservePort(SOCK_STREAM);

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<TcpPacketSource<std::uint8_t>>({{"endpoint", endpointFor(port)}, {"bind", true}, {"max_message_bytes", kBound}});
        auto&     collector = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(source, collector).has_value());

        GraphRunner                       runner(std::move(graph));
        const std::array<std::uint8_t, 2> payload{1U, 2U};
        {
            RawStream first;
            first.connectTo(port);
            first.write(uint8Envelope(payload, 1ULL));
            expect(waitFor([&collector] { return collector.count() >= 1UZ; })) << "the first peer's envelope never arrived";
            // zero linger turns the close below into a reset rather than a FIN, which is what the reader meets as a
            // failed read instead of an orderly end of stream
            const ::linger immediate{.l_onoff = 1, .l_linger = 0};
            expect(::setsockopt(first.fd, SOL_SOCKET, SO_LINGER, &immediate, static_cast<::socklen_t>(sizeof(immediate))) == 0) << "the test could not arm a reset";
        }

        expect(waitFor([&source] { return source.nRecvErrors >= 1ULL; })) << "the reset was not seen as a failed read";
        expect(waitFor([&source] { return source.nDisconnects >= 1ULL; })) << "the reset connection was not given up";
        expect(!runner.finished.load()) << "a peer's reset ended the graph";

        RawStream second;
        second.connectTo(port);
        second.write(uint8Envelope(payload, 2ULL));
        expect(waitFor([&collector] { return collector.count() >= 2UZ; })) << "the source did not serve the peer that replaced the reset one";
        runner.stop();

        expect(eq(source.nReaderFailures, std::uint64_t{0ULL})) << "a peer's reset is not a fault of the socket the reader owns";
        expect(source.lastReaderError().empty()) << "the reader named a fault it did not have: " << source.lastReaderError();
    };

    // The datagram source has one socket and no peer to lose, so the same ruling reads: a malformed datagram is the
    // wire's, and only the bound socket itself can end the reader. The hang-up is taken through a whole graph, because
    // the status is what a graph is owed; the read that can never succeed is taken at the block.
    "a UDP source outlives a bad datagram and ends on a bound socket that cannot read again"_test = [] {
        const std::uint16_t hungPort = reservePort(SOCK_DGRAM);

        gr::Graph graph;
        auto&     hungSource = graph.emplaceBlock<UdpPacketSource<std::uint8_t>>({{"endpoint", endpointFor(hungPort)}, {"max_message_bytes", kBound}});
        auto&     collector  = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(hungSource, collector).has_value());

        GraphRunner runner(std::move(graph));
        expect(waitFor([&hungSource] { return hungSource._socketOpen; })) << "the source never bound its endpoint";

        // a datagram too short to hold a header is refused and read past; the reader is expected to still be there
        const RawDatagram                   peer;
        const std::array<std::uint8_t, 4UZ> stub{0x01U, 0x02U, 0x03U, 0x04U};
        peer.sendTo(hungPort, stub);
        expect(waitFor([&hungSource] { return hungSource.nShortHeader >= 1ULL; })) << "the short datagram was never refused";
        expect(!runner.finished.load()) << "a datagram the reader could not use ended the reader";
        expect(eq(hungSource.nReaderFailures, std::uint64_t{0ULL})) << "a malformed datagram is not a fault of the socket the reader owns";

        int bound = -1;
        expect(waitFor([&bound, hungPort] {
            bound = findSocketBoundTo(hungPort, SOCK_DGRAM);
            return bound >= 0;
        })) << "the source's bound socket was not found on its own port";
        replaceWithHungUpPipe(bound);

        expect(waitFor([&runner] { return runner.finished.load(); })) << "a bound socket that reports a hang-up left the reader running";
        runner.stop();
        expect(!runner.failure.empty()) << "the reader ended on a fault and the graph did not end in error";

        const std::string hungReason = hungSource.lastReaderError();
        expect(hungReason.contains("POLLHUP") || hungReason.contains("POLLERR") || hungReason.contains("POLLNVAL")) << "the reader did not name the flag its bound socket reported: " << hungReason;
        expect(eq(hungSource.nReaderFailures, std::uint64_t{1ULL})) << "one reader ended on a fault, so one failure is counted";

        // the second shape, driven without a scheduler: a read that can never succeed again, met at the block
        const std::uint16_t           blindPort = reservePort(SOCK_DGRAM);
        UdpPacketSource<std::uint8_t> blindSource({{"endpoint", endpointFor(blindPort)}, {"max_message_bytes", kBound}});
        blindSource.settings().init();
        std::ignore = blindSource.settings().applyStagedParameters();
        blindSource.start();

        int replaced = -1;
        expect(waitFor([&replaced, blindPort] {
            replaced = findSocketBoundTo(blindPort, SOCK_DGRAM);
            return replaced >= 0;
        })) << "the second source's bound socket was not found on its own port";
        replaceWithNonSocket(replaced);

        expect(waitFor([&blindSource] { return !blindSource.lastReaderError().empty(); })) << "a read that can never succeed again left the reader running";
        const std::string blindReason = blindSource.lastReaderError();
        {
            std::lock_guard lock(blindSource._mutex);
            expect(blindSource._readerFailed) << "the reader ended on a fault without raising the flag processBulk answers on";
        }
        blindSource.stop();

        expect(blindReason.starts_with("receiving a datagram failed")) << "the reader did not name the read that refused it: " << blindReason;
        expect(blindReason.contains("socket")) << "the reason drops the system's own wording: " << blindReason;
        expect(eq(blindSource.nReaderFailures, std::uint64_t{1ULL})) << "one reader ended on a fault, so one failure is counted";
    };
};

int main() { /* not needed for UT */ }
