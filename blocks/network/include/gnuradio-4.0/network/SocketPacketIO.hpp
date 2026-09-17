#ifndef GNURADIO_SOCKETPACKETIO_HPP
#define GNURADIO_SOCKETPACKETIO_HPP

#include <algorithm>
#include <array>
#include <chrono>
#include <concepts>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <expected>
#include <format>
#include <mutex>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <cerrno>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <gnuradio-4.0/AtomicRef.hpp>
#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/YamlPmt.hpp>
#include <gnuradio-4.0/annotated.hpp>
#include <gnuradio-4.0/meta/utils.hpp>
#include <gnuradio-4.0/thread/thread_pool.hpp>

#include <gnuradio-4.0/network/PacketEnvelope.hpp>
#include <gnuradio-4.0/network/RecordVocabulary.hpp>

/**
 * @brief The packet envelope over plain TCP and UDP sockets, with no message library between the block and the wire.
 *
 * The envelope these blocks carry is the one `gr::network::encodeHeader` writes, unchanged: the same 32 bytes in a
 * ZeroMQ frame, in a byte stream and in a datagram. What changes is the framing the transport supplies underneath.
 * ZeroMQ hands a reader whole messages; TCP hands it a byte stream with no boundaries at all, and UDP hands it
 * datagrams that may be lost, reordered or duplicated but never torn. Those two facts are the whole of the delta,
 * and they appear here as one state machine and one datagram policy.
 *
 * The topic frame does not exist. It was a subscription artifact, and a TCP connection or a UDP port is one channel,
 * so there is no `topic` and no `pattern` setting anywhere in this file.
 */
namespace gr::blocks::network {

namespace detail::sockio {

/// @brief The key the carrier's `timestamp` field crosses under, removed again by the receiving block.
///
/// Producer-private rather than vocabulary: it exists to serve one carrier field across one boundary and is consumed
/// on arrival, so the vocabulary's one-spelling rule survives the crossing. Restated here rather than shared with the
/// ZeroMQ header because the spelling is the envelope's, not any one transport's.
inline constexpr std::string_view kTimestampKey = "packet_timestamp";

/// @brief The bound every socket wait in this file carries, so a stop request is observed promptly.
inline constexpr int kPollMs = 100;

/// @brief The poll flags that say the descriptor itself is finished rather than merely idle.
inline constexpr short kPollFault = static_cast<short>(POLLERR | POLLHUP | POLLNVAL);

/// @brief How many consecutive transient faults an I/O thread absorbs before it treats the shortage as permanent.
inline constexpr std::size_t kTransientFaultLimit = 8UZ;

/// @brief The shortest interval between two resolutions of one endpoint, so a retry loop is not a resolver flood.
inline constexpr std::uint32_t kResolveIntervalMs = 1000U;

/// @brief A 64-bit length narrowed to an index; a template, so the cast stands where `std::size_t` is 32
/// bits and is not a useless cast where the two are the same 64-bit type.
template<std::integral To, std::integral From>
[[nodiscard]] constexpr To narrowIndex(From value) noexcept {
    return static_cast<To>(value);
}

/// @brief The system's own wording for an `errno` value, taken through the thread-safe accessor.
[[nodiscard]] inline std::string errorText(int code) { return std::system_category().message(code); }

/// @brief A file descriptor that closes itself, so no error path can leak one.
class Descriptor {
public:
    Descriptor() noexcept = default;
    explicit Descriptor(int fd) noexcept : _fd(fd) {}
    Descriptor(const Descriptor&)            = delete;
    Descriptor& operator=(const Descriptor&) = delete;
    Descriptor(Descriptor&& other) noexcept : _fd(std::exchange(other._fd, -1)) {}
    Descriptor& operator=(Descriptor&& other) noexcept {
        if (this != &other) {
            reset(std::exchange(other._fd, -1));
        }
        return *this;
    }
    ~Descriptor() { reset(); }

    [[nodiscard]] int  get() const noexcept { return _fd; }
    [[nodiscard]] bool valid() const noexcept { return _fd >= 0; }

    /// @brief Give up ownership of the descriptor without closing it, for a hand-off to another holder.
    [[nodiscard]] int release() noexcept { return std::exchange(_fd, -1); }

    void reset(int fd = -1) noexcept {
        if (_fd >= 0) {
            std::ignore = ::close(_fd);
        }
        _fd = fd;
    }

private:
    int _fd = -1;
};

/// @brief One resolved endpoint, kept in the storage form the socket calls take.
struct Address {
    ::sockaddr_storage storage{};
    ::socklen_t        length = 0;
    int                family = 0;
};

/// @brief Resolve `"host:port"` into the addresses a socket may be opened on, or say why it cannot be resolved.
///
/// The split is at the last colon so that an IPv6 literal, whose own colons would otherwise decide the split, parses
/// once its brackets are stripped. `AI_NUMERICSERV` keeps the port a decimal number rather than a service name, which
/// is what the endpoint grammar states; `AI_PASSIVE` gives a wildcard address when the caller intends to listen.
///
/// @p gaiStatus, where a caller passes one, receives the `getaddrinfo` code behind a failure and zero otherwise, which
/// is what tells a temporary resolver outage from a name that is gone.
[[nodiscard]] inline std::expected<std::vector<Address>, std::string> resolveEndpoint(std::string_view endpoint, int socketType, bool passive, int* gaiStatus = nullptr) {
    if (gaiStatus != nullptr) {
        *gaiStatus = 0;
    }
    const std::size_t colon = endpoint.rfind(':');
    if (colon == std::string_view::npos || colon == 0UZ || colon + 1UZ == endpoint.size()) {
        return std::unexpected(std::format("endpoint '{}' is not 'host:port'; a socket endpoint states both, for example 127.0.0.1:5555 or [::1]:5555", endpoint));
    }
    std::string_view host = endpoint.substr(0UZ, colon);
    if (host.size() >= 2UZ && host.front() == '[' && host.back() == ']') {
        host = host.substr(1UZ, host.size() - 2UZ);
    }
    if (host.empty()) {
        return std::unexpected(std::format("endpoint '{}' names no host", endpoint));
    }

    const std::string hostText(host);
    const std::string portText(endpoint.substr(colon + 1UZ));

    ::addrinfo hints{};
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = socketType;
    hints.ai_flags    = passive ? (AI_NUMERICSERV | AI_PASSIVE) : AI_NUMERICSERV;

    ::addrinfo* results = nullptr;
    const int   status  = ::getaddrinfo(hostText.c_str(), portText.c_str(), &hints, &results);
    if (status != 0) {
        if (gaiStatus != nullptr) {
            *gaiStatus = status;
        }
        return std::unexpected(std::format("cannot resolve endpoint '{}': {}", endpoint, ::gai_strerror(status)));
    }

    std::vector<Address> addresses;
    for (const ::addrinfo* entry = results; entry != nullptr; entry = entry->ai_next) {
        Address address;
        std::memcpy(&address.storage, entry->ai_addr, entry->ai_addrlen);
        address.length = entry->ai_addrlen;
        address.family = entry->ai_family;
        addresses.push_back(address);
    }
    ::freeaddrinfo(results);

    if (addresses.empty()) {
        return std::unexpected(std::format("endpoint '{}' resolved to no address", endpoint));
    }
    return addresses;
}

[[nodiscard]] inline const ::sockaddr* addressOf(const Address& address) noexcept { return reinterpret_cast<const ::sockaddr*>(&address.storage); }

[[nodiscard]] inline bool setNonBlocking(int fd) noexcept {
    const int flags = ::fcntl(fd, F_GETFL, 0);
    return flags >= 0 && ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

/// @brief Wait at most @p timeoutMs for @p events on @p fd; an expired wait reports no events at all.
[[nodiscard]] inline short pollFor(int fd, short events, int timeoutMs) noexcept {
    ::pollfd  item{.fd = fd, .events = events, .revents = 0};
    const int ready = ::poll(&item, 1UL, timeoutMs);
    return ready > 0 ? item.revents : static_cast<short>(0);
}

/// @brief Whether an `errno` value means "nothing to do just now" rather than a fault worth counting.
///
/// `EWOULDBLOCK` is not tested separately: on this platform it is the same value as `EAGAIN`.
[[nodiscard]] inline bool wouldBlock(int code) noexcept { return code == EAGAIN || code == EINTR; }

/// @brief What an I/O thread does about a fault its own socket reported.
enum class Fault : std::uint8_t {
    Retry,     ///< nothing to do just now, or something the far end did; the loop carries on unchanged
    Transient, ///< a shortage that may pass; the loop waits and tries again, a bounded number of times
    Fatal      ///< the descriptor cannot serve again, and there is nothing left for the thread to do
};

/// @brief How a fault at a socket the I/O thread owns is judged.
///
/// A would-block and an interrupt are not faults at all. Neither is anything the far end or the route between can
/// cause: `accept` hands back the pending errors of the connection it would have returned, and a datagram socket is
/// told of an unreachable peer through its own next read, so a transport that died of either would die of a peer
/// misbehaving, which is the opposite of what a reconnecting transport is for. A shortage of descriptors or of kernel
/// memory is real but may lift, so it is retried; the caller bounds how often. Everything else is the descriptor
/// itself, and a listening or bound socket that has broken does not mend.
[[nodiscard]] inline Fault classifyFault(int code) noexcept {
    switch (code) {
    case EAGAIN:
    case EINTR:
    case ECONNABORTED:
    case ECONNREFUSED:
    case ECONNRESET:
    case EPROTO:
    case ENOPROTOOPT:
    case EOPNOTSUPP:
    case ENETDOWN:
    case ENETUNREACH:
    case ENONET:
    case EHOSTDOWN:
    case EHOSTUNREACH:
    case ETIMEDOUT: return Fault::Retry;
    case EMFILE:
    case ENFILE:
    case ENOBUFS:
    case ENOMEM: return Fault::Transient;
    default: return Fault::Fatal;
    }
}

/// @brief The fault flags of @p revents, spelled as poll(2) spells them, for the message a thread ends on.
[[nodiscard]] inline std::string pollFaultText(short revents) {
    std::string text;
    const auto  add = [&text](std::string_view flag) {
        if (!text.empty()) {
            text += '|';
        }
        text += flag;
    };
    if ((revents & POLLERR) != 0) {
        add("POLLERR");
    }
    if ((revents & POLLHUP) != 0) {
        add("POLLHUP");
    }
    if ((revents & POLLNVAL) != 0) {
        add("POLLNVAL");
    }
    return text;
}

} // namespace detail::sockio

GR_REGISTER_BLOCK(gr::blocks::network::TcpPacketSink, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
requires gr::network::EnvelopeItem<T>
struct TcpPacketSink : Block<TcpPacketSink<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Writes each incoming gr::Packet<T> to a TCP peer as one versioned envelope in the byte stream.

One packet becomes `header || metadata || payload` written consecutively, with no delimiter of any kind: the 32-byte
header states both lengths, so a reader finds the next envelope from the header it has just read. The header also
states the item type, item size, item count, byte order and wire version, so a receiver this process did not build
reads the payload without being told anything out of band. The metadata is the packet's own `meta_information[0]`
serialized by `gr::pmt::yaml`, copied key for key with nothing filtered.

A listening sink serves one peer at a time. A second connection is accepted and immediately closed, counted in
`nPeersRefused`, because refusing at accept is the only way the extra client learns anything at all. A connecting
sink retries forever at `reconnect_ms`; a transport does not decide for the graph that an endpoint is gone for good,
so `start()` succeeds whether or not the peer is up yet and only a socket that cannot be created or an endpoint that
cannot be resolved is fatal.

While no peer is connected nothing leaves the queue, so the queue fills and `overflow` decides what happens next. The
default `drop_oldest` discards the stalest envelope and counts it, and the receiver sees the loss as a `sequence` gap;
`backpressure` instead consumes fewer input items, so the stall propagates upstream by the framework's own path.

The block fills `sequence` only when the packet does not already carry one. An upstream `sequence` counts what the
original producer emitted, so a gap in it at the receiver means "lost somewhere between that producer and here",
while this block's own counter can only count what reached the sink.

The socket is owned end to end by a dedicated I/O thread. `processBulk` serializes and enqueues and never touches a
descriptor. Teardown is bounded and discards what is still queued, counted in `nDroppedAtStop`, because an unbounded
drain against a stalled peer is a graph that will not tear down; an envelope lost part-written when a peer disappears
is counted in `nSendErrors` and dropped rather than retried, since the receiver would have to resynchronize past its
truncated prefix either way.
)"">;

    PortIn<Packet<T>>                   in;
    PortOut<Packet<T>, Async, Optional> reject;

    Annotated<std::string, "endpoint", Visible, Doc<"'host:port', e.g. 127.0.0.1:5555 or [::1]:5555; required, there is no default">> endpoint{};
    Annotated<bool, "bind", Doc<"listen on the endpoint rather than connect to it; the producer is normally the stable end">>         bind              = true;
    Annotated<std::string, "overflow", Doc<"'drop_oldest' or 'backpressure', applied when the in-process send queue is full">>        overflow          = std::string("drop_oldest");
    Annotated<gr::Size_t, "queue_messages", Doc<"in-process send queue depth">>                                                       queue_messages    = 1024U;
    Annotated<std::uint64_t, "queue_bytes", Unit<"byte">, Doc<"in-process send queue size">>                                          queue_bytes       = 16777216ULL;
    Annotated<std::uint64_t, "max_message_bytes", Unit<"byte">, Doc<"a packet whose envelope would exceed this is rejected">>         max_message_bytes = 16777216ULL;
    Annotated<gr::Size_t, "reconnect_ms", Unit<"ms">, Doc<"interval between connection attempts while connecting">>                   reconnect_ms      = 100U;

    GR_MAKE_REFLECTABLE(TcpPacketSink, in, reject, endpoint, bind, overflow, queue_messages, queue_bytes, max_message_bytes, reconnect_ms);

    // Counted, stated drops and refusals. Plain members, printed once by stop(); nothing here is on the sample path.
    std::uint64_t nPacketsSent          = 0ULL; ///< envelopes written whole to a peer
    std::uint64_t nBytesSent            = 0ULL; ///< envelope bytes written whole to a peer
    std::uint64_t nRejectedPackets      = 0ULL; ///< packets refused for exceeding max_message_bytes
    std::uint64_t nOverQueueBytes       = 0ULL; ///< packets refused because one envelope exceeds queue_bytes
    std::uint64_t nDroppedOnOverflow    = 0ULL; ///< queued envelopes discarded under overflow = drop_oldest
    std::uint64_t nBackpressureStalls   = 0ULL; ///< processBulk calls that consumed fewer items than they read
    std::uint64_t nSequenceDeclined     = 0ULL; ///< packets that already stated sequence
    std::uint64_t nMetaKeysMistyped     = 0ULL; ///< vocabulary keys whose type disagrees with the declaration
    std::uint64_t nTimestampsCarried    = 0ULL; ///< packets with a non-zero Packet::timestamp
    std::uint64_t nDefaultValuesDropped = 0ULL; ///< packets whose default_value differs from T(), which has no wire field
    std::uint64_t nDroppedAtStop        = 0ULL; ///< envelopes still queued when the I/O thread stopped
    std::uint64_t nSendErrors           = 0ULL; ///< envelopes lost to a write that could not complete
    std::uint64_t nPeersRefused         = 0ULL; ///< connections accepted and closed because one peer was already served
    std::uint64_t nDisconnects          = 0ULL; ///< connections lost
    std::uint64_t nReconnects           = 0ULL; ///< connections established after the first

    std::mutex                            _mutex;
    std::condition_variable               _cv;
    std::deque<std::vector<std::uint8_t>> _queue;
    std::uint64_t                         _queuedBytes   = 0ULL;
    bool                                  _stopRequested = false;
    bool                                  _opened        = false;
    std::string                           _openFailure{};
    bool                                  _ioThreadDone = true; ///< true until start() launches the I/O thread

    std::uint64_t _sequence = 0ULL; ///< the value the sink writes when a packet states none
    std::string   _endpoint{};      ///< the socket settings, frozen for the duration of one run
    bool          _bind            = true;
    std::size_t   _queueMessages   = 1024UZ;
    std::uint64_t _queueBytes      = 16777216ULL;
    std::uint32_t _reconnectMs     = 100U;
    bool          _socketOpen      = false;
    bool          _backpressure    = false;
    std::uint64_t _maxMessageBytes = 16777216ULL;
    bool          _everConnected   = false;
    std::size_t   _addressCursor   = 0UZ; ///< which resolved address the next connection attempt takes; the I/O thread's alone

    /// @brief Joins the I/O thread however the block dies.
    ///
    /// Must be the last declared member, so it is destroyed first and the thread is gone before the queue, mutex and
    /// condition variable it uses. `stop()` cannot be relied on: the scheduler does not call it when a graph ends in
    /// ERROR, and `~Block()` cannot stand in because derived members are destroyed before it runs.
    struct IoThreadGuard {
        TcpPacketSink* self;
        explicit IoThreadGuard(TcpPacketSink* owner) noexcept : self(owner) {}
        IoThreadGuard(const IoThreadGuard&)            = delete;
        IoThreadGuard(IoThreadGuard&&)                 = delete;
        IoThreadGuard& operator=(const IoThreadGuard&) = delete;
        IoThreadGuard& operator=(IoThreadGuard&&)      = delete;
        ~IoThreadGuard() { self->requestStopAndJoin(); }
    };
    IoThreadGuard _ioGuard{this};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() {
        validate();
        freezeSocketSettings();
        {
            std::lock_guard lock(_mutex);
            _stopRequested = false;
            _opened        = false;
            _openFailure.clear();
            _queue.clear();
            _queuedBytes = 0ULL;
        }
        _everConnected = false;
        gr::atomic_ref(_ioThreadDone).store_release(false);
        thread_pool::Manager::defaultIoPool()->execute([this] { ioSendLoop(); });

        std::unique_lock lock(_mutex);
        _cv.wait(lock, [this] { return _opened; });
        if (!_openFailure.empty()) {
            const std::string failure = _openFailure;
            lock.unlock();
            gr::atomic_ref(_ioThreadDone).wait(false); // a failed start leaves no thread behind
            throw gr::exception(failure);
        }
        _socketOpen = true;
    }

    void stop() {
        requestStopAndJoin();
        _socketOpen = false;
        report();
    }

    void rebuild() {
        validate();
        if (_socketOpen) {
            refuseFrozenChange();
        }
        _backpressure    = overflow.value == "backpressure";
        _maxMessageBytes = max_message_bytes.value;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& rejectSpan) {
        const bool  rejectConnected = rejectSpan.isConnected; // read once, so the room test and the store cannot disagree
        std::size_t consumed        = 0UZ;
        std::size_t onReject        = 0UZ;

        for (std::size_t i = 0UZ; i < inSpan.size(); ++i) {
            const Packet<T>& packet = inSpan[i];

            std::uint64_t payloadBytes = packet.signal_values.size();
            payloadBytes *= sizeof(T);
            property_map      map      = buildMetadata(packet);
            const std::string metadata = pmt::yaml::serialize(map);

            const std::uint64_t headerBytes = gr::network::kHeaderBytesV1;
            const std::uint64_t total       = headerBytes + metadata.size() + payloadBytes;
            // the two size bounds, in the order they bind: what the wire will not carry, then what the queue cannot
            // hold. A queue cannot shed its way to room for an envelope larger than the whole of it, so such an
            // envelope is refused here rather than admitted over the bound the graph set.
            const bool overMax   = total > _maxMessageBytes;
            const bool overQueue = !overMax && total > _queueBytes;
            if (overMax || overQueue) {
                if (rejectConnected && onReject >= rejectSpan.size()) {
                    break; // no room on the port this packet belongs on; it stays in the buffer
                }
                if (rejectConnected) {
                    Packet<T> refused = packet; // republished whole: what is wrong with it is its size, not its content
                    refused.meta_information.resize(1UZ);
                    refused.meta_information[0UZ].insert_or_assign(property_map::key_type("discard_reason"), pmt::Value(std::string(overMax ? "over_max_message_bytes" : "over_queue_bytes")));
                    rejectSpan[onReject] = std::move(refused);
                }
                ++onReject;
                if (overMax) {
                    ++nRejectedPackets;
                } else {
                    ++nOverQueueBytes;
                }
                ++consumed;
                continue; // the sequence counter never advanced, so a refused packet does not renumber the stream
            }

            if (!enqueue(assemble(packet, metadata, payloadBytes, total))) {
                ++nBackpressureStalls; // the item is not consumed, so the input buffer fills and the stall propagates
                break;
            }
            ++_sequence;
            ++consumed;
        }

        std::ignore = inSpan.consume(consumed);
        rejectSpan.publish(rejectConnected ? onReject : 0UZ);
        if (consumed == 0UZ) {
            return inSpan.size() == 0UZ ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    void validate() {
        if (endpoint.value.empty()) {
            throw gr::exception("endpoint is empty; a transport sink has no default endpoint and will not bind to nothing");
        }
        if (overflow.value != "drop_oldest" && overflow.value != "backpressure") {
            throw gr::exception(std::format("overflow is '{}'; it must be 'drop_oldest' (shed the stalest envelope, count it) or 'backpressure' (consume fewer input items)", overflow.value));
        }
        if (queue_messages.value == 0U) {
            throw gr::exception("queue_messages is 0; the in-process send queue must hold at least one envelope");
        }
        if (queue_bytes.value == 0ULL) {
            throw gr::exception("queue_bytes is 0; the in-process send queue must hold at least one envelope");
        }
        if (max_message_bytes.value == 0ULL) {
            throw gr::exception("max_message_bytes is 0; every packet would be rejected");
        }
        if (max_message_bytes.value > gr::network::kMaxEnvelopeBytes) {
            throw gr::exception(std::format("max_message_bytes is {}; {} is the largest envelope the header's 32-bit length fields can state", max_message_bytes.value, gr::network::kMaxEnvelopeBytes));
        }
        if (reconnect_ms.value == 0U) {
            throw gr::exception("reconnect_ms is 0; a connecting sink would retry without pause and spend the thread on nothing else");
        }
    }

    /// @brief Refuse a change to a setting the running socket was built from, naming it.
    ///
    /// Re-creating a socket under a running graph means tearing down and restarting the I/O thread mid-flight, which
    /// is a teardown race; the graph rebuild the framework already supports is the supported way to move an endpoint.
    void refuseFrozenChange() const {
        const auto refuse = [](std::string_view name) { throw gr::exception(std::format("setting '{}' is read once when the socket opens and cannot change while the block is running; rebuild the graph instead", name)); };
        if (endpoint.value != _endpoint) {
            refuse("endpoint");
        }
        if (bind.value != _bind) {
            refuse("bind");
        }
        if (static_cast<std::size_t>(queue_messages.value) != _queueMessages) {
            refuse("queue_messages");
        }
        if (queue_bytes.value != _queueBytes) {
            refuse("queue_bytes");
        }
        if (reconnect_ms.value != _reconnectMs) {
            refuse("reconnect_ms");
        }
    }

    void freezeSocketSettings() {
        _endpoint      = endpoint.value;
        _bind          = bind.value;
        _queueMessages = static_cast<std::size_t>(queue_messages.value);
        _queueBytes    = queue_bytes.value;
        _reconnectMs   = reconnect_ms.value;
        _addressCursor = 0UZ;
    }

    /// @brief The metadata map the envelope carries: the packet's own, plus `sequence` where it states none.
    [[nodiscard]] property_map buildMetadata(const Packet<T>& packet) {
        property_map map;
        if (!packet.meta_information.empty()) {
            map = packet.meta_information[0UZ]; // copied key for key, nothing filtered and nothing consumed
        }
        nMetaKeysMistyped += detail::countMistypedKeys(map);

        if (map.find("sequence") == map.end()) {
            map.insert_or_assign(property_map::key_type("sequence"), pmt::Value(_sequence));
        } else {
            ++nSequenceDeclined; // the producer's own count is the less local one, so it stands
        }
        if (packet.timestamp != 0) {
            map.insert_or_assign(property_map::key_type(detail::sockio::kTimestampKey), pmt::Value(packet.timestamp));
            ++nTimestampsCarried;
        }
        if (packet.default_value != T()) {
            ++nDefaultValuesDropped; // the field is not reflected and the envelope has no place for it; counted, not silent
        }
        return map;
    }

    /// @brief The three envelope parts laid end to end, which is the form both socket families write.
    [[nodiscard]] std::vector<std::uint8_t> assemble(const Packet<T>& packet, const std::string& metadata, std::uint64_t payloadBytes, std::uint64_t total) const {
        gr::network::EnvelopeHeader header;
        header.item_type     = gr::network::kItemTypeCode<T>;
        header.item_size     = static_cast<std::uint8_t>(sizeof(T));
        header.item_count    = static_cast<std::uint32_t>(packet.signal_values.size());
        header.payload_bytes = static_cast<std::uint32_t>(payloadBytes);
        header.meta_bytes    = static_cast<std::uint32_t>(metadata.size());

        std::vector<std::uint8_t> bytes(detail::sockio::narrowIndex<std::size_t>(total));
        const auto                encoded = gr::network::encodeHeader(header);
        std::ranges::copy(encoded, bytes.begin());
        if (!metadata.empty()) {
            std::memcpy(bytes.data() + gr::network::kHeaderBytesV1, metadata.data(), metadata.size());
        }
        // copied, never moved out of the input span: a Packet output port may fan out and the buffer slot is shared,
        // so moving would empty a packet another reader is about to see
        const std::span<const std::byte> raw = std::as_bytes(std::span<const T>(packet.signal_values));
        if (!raw.empty()) {
            std::memcpy(bytes.data() + gr::network::kHeaderBytesV1 + metadata.size(), raw.data(), raw.size());
        }
        return bytes;
    }

    /// @brief Put an envelope on the send queue, applying `overflow` when it is full. False means "not consumed".
    ///
    /// `processBulk` refuses an envelope larger than `queue_bytes` before offering it, so what arrives here always
    /// fits and shedding always makes room for it: an empty queue holds nothing and the test below is then false.
    [[nodiscard]] bool enqueue(std::vector<std::uint8_t>&& envelope) {
        const std::uint64_t bytes = envelope.size();
        std::unique_lock    lock(_mutex);
        while (_queue.size() >= _queueMessages || _queuedBytes + bytes > _queueBytes) {
            if (_backpressure) {
                return false;
            }
            _queuedBytes -= _queue.front().size(); // the newest packets are what a live consumer wants
            _queue.pop_front();
            ++nDroppedOnOverflow;
        }
        _queuedBytes += bytes;
        _queue.push_back(std::move(envelope));
        lock.unlock();
        _cv.notify_one();
        return true;
    }

    void requestStopAndJoin() {
        {
            std::lock_guard lock(_mutex);
            _stopRequested = true;
        }
        _cv.notify_all();
        gr::atomic_ref(_ioThreadDone).wait(false);
    }

    void report() {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("packets sent", nPacketsSent);
        append("bytes sent", nBytesSent);
        append("rejected packets", nRejectedPackets);
        append("over queue bytes", nOverQueueBytes);
        append("dropped on overflow", nDroppedOnOverflow);
        append("backpressure stalls", nBackpressureStalls);
        append("sequence declined", nSequenceDeclined);
        append("metadata keys mistyped", nMetaKeysMistyped);
        append("timestamps carried", nTimestampsCarried);
        append("default values dropped", nDefaultValuesDropped);
        append("dropped at stop", nDroppedAtStop);
        append("send errors", nSendErrors);
        append("peers refused", nPeersRefused);
        append("disconnects", nDisconnects);
        append("reconnects", nReconnects);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::network::TcpPacketSink '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] bool stopRequested() {
        std::lock_guard lock(_mutex);
        return _stopRequested;
    }

    /// @brief Wait at most @p milliseconds, ending early when a stop is requested.
    void waitBounded(std::uint32_t milliseconds) {
        std::unique_lock lock(_mutex);
        std::ignore = _cv.wait_for(lock, std::chrono::milliseconds(milliseconds), [this] { return _stopRequested; });
    }

    /// @brief The whole of this block's contact with the socket: one thread opens it, writes, and closes it.
    void ioSendLoop() {
        thread_pool::thread::setThreadName(std::format("tcppktsink:{}", this->name.value));
        detail::sockio::Descriptor listener;
        std::string                failure;

        const auto addresses = detail::sockio::resolveEndpoint(_endpoint, SOCK_STREAM, _bind);
        if (!addresses.has_value()) {
            failure = addresses.error();
        } else if (_bind) {
            failure = openListener(listener, *addresses); // a port that cannot be bound is a fault of this process
        } else {
            // a peer that is not up yet is not a fault, so only the socket the connection would use is proved here
            const detail::sockio::Descriptor probe(::socket(addresses->front().family, SOCK_STREAM, 0));
            if (!probe.valid()) {
                failure = std::format("cannot create a TCP socket for '{}': {}", _endpoint, detail::sockio::errorText(errno));
            }
        }

        {
            std::lock_guard lock(_mutex);
            _openFailure = failure;
            _opened      = true;
        }
        _cv.notify_all();

        if (failure.empty()) {
            sendUntilStopped(listener, *addresses);
        }
        gr::atomic_ref(_ioThreadDone).store_release(true);
        gr::atomic_ref(_ioThreadDone).notify_all();
    }

    /// @brief Bind and listen on the first address that accepts both, or say why none did.
    [[nodiscard]] std::string openListener(detail::sockio::Descriptor& listener, std::span<const detail::sockio::Address> addresses) {
        std::string lastError = "no address was tried";
        for (const detail::sockio::Address& address : addresses) {
            detail::sockio::Descriptor candidate(::socket(address.family, SOCK_STREAM, 0));
            if (!candidate.valid()) {
                lastError = detail::sockio::errorText(errno);
                continue;
            }
            const int on = 1; // so a restarted graph can rebind its port inside TIME_WAIT
            std::ignore  = ::setsockopt(candidate.get(), SOL_SOCKET, SO_REUSEADDR, &on, static_cast<::socklen_t>(sizeof(on)));
            if (::bind(candidate.get(), detail::sockio::addressOf(address), address.length) != 0) {
                lastError = detail::sockio::errorText(errno);
                continue;
            }
            if (::listen(candidate.get(), 8) != 0) {
                lastError = detail::sockio::errorText(errno);
                continue;
            }
            std::ignore = detail::sockio::setNonBlocking(candidate.get());
            listener    = std::move(candidate);
            return {};
        }
        return std::format("cannot listen on '{}': {}", _endpoint, lastError);
    }

    void sendUntilStopped(detail::sockio::Descriptor& listener, std::span<const detail::sockio::Address> addresses) {
        using namespace std::chrono_literals;
        detail::sockio::Descriptor peer;

        while (!stopRequested()) {
            if (!peer.valid()) {
                acquirePeer(listener, peer, addresses);
                continue;
            }
            refuseExtraPeers(listener);
            if (!peerAlive(peer)) {
                losePeer(peer);
                continue;
            }

            std::vector<std::uint8_t> envelope;
            {
                std::unique_lock lock(_mutex);
                _cv.wait_for(lock, 100ms, [this] { return _stopRequested || !_queue.empty(); });
                if (_stopRequested) {
                    break;
                }
                if (_queue.empty()) {
                    continue;
                }
                envelope = std::move(_queue.front());
                _queue.pop_front();
                _queuedBytes -= envelope.size();
            }

            if (writeAll(peer, envelope)) {
                std::lock_guard lock(_mutex);
                ++nPacketsSent;
                nBytesSent += envelope.size();
                continue;
            }
            if (stopRequested()) {
                ++nDroppedAtStop;
                break;
            }
            // the peer went away with part of an envelope written; a retry would append to a prefix the reader must
            // already resynchronize past, so the envelope is counted and dropped instead
            ++nSendErrors;
            losePeer(peer);
        }

        // teardown is bounded and loses what is in flight; a drain against a stalled peer is unbounded, and an
        // unbounded stop() is a graph that will not tear down. One exit, so the loss is counted however the loop ends.
        std::lock_guard lock(_mutex);
        nDroppedAtStop += _queue.size();
        _queue.clear();
        _queuedBytes = 0ULL;
    }

    /// @brief Obtain the one peer this sink serves, waiting no longer than one poll interval for it.
    void acquirePeer(detail::sockio::Descriptor& listener, detail::sockio::Descriptor& peer, std::span<const detail::sockio::Address> addresses) {
        if (_bind) {
            const short revents = detail::sockio::pollFor(listener.get(), POLLIN, detail::sockio::kPollMs);
            if ((revents & POLLIN) == 0) {
                return;
            }
            const int accepted = ::accept(listener.get(), nullptr, nullptr);
            if (accepted < 0) {
                return;
            }
            adoptPeer(peer, accepted);
            return;
        }

        // one endpoint may resolve to several addresses, so each attempt takes the next of them in turn: pinning every
        // retry to the first would leave a host reachable only on its second address permanently unreachable
        const detail::sockio::Address& address = addresses[_addressCursor % addresses.size()];
        ++_addressCursor;

        detail::sockio::Descriptor candidate(::socket(address.family, SOCK_STREAM, 0));
        if (!candidate.valid()) {
            waitBounded(_reconnectMs);
            return;
        }
        std::ignore = detail::sockio::setNonBlocking(candidate.get());
        if (::connect(candidate.get(), detail::sockio::addressOf(address), address.length) != 0) {
            if (errno != EINPROGRESS) {
                waitBounded(_reconnectMs);
                return;
            }
            const short revents = detail::sockio::pollFor(candidate.get(), POLLOUT, detail::sockio::kPollMs);
            int         pending = 0;
            ::socklen_t length  = static_cast<::socklen_t>(sizeof(pending));
            if ((revents & POLLOUT) == 0 || ::getsockopt(candidate.get(), SOL_SOCKET, SO_ERROR, &pending, &length) != 0 || pending != 0) {
                waitBounded(_reconnectMs);
                return;
            }
        }
        adoptPeer(peer, candidate.release());
    }

    void adoptPeer(detail::sockio::Descriptor& peer, int fd) {
        const int on = 1; // an envelope is a message, and delaying a small one to coalesce it defeats the transport
        std::ignore  = ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, static_cast<::socklen_t>(sizeof(on)));
        std::ignore  = detail::sockio::setNonBlocking(fd);
        peer.reset(fd);
        std::lock_guard lock(_mutex);
        if (_everConnected) {
            ++nReconnects;
        }
        _everConnected = true;
    }

    void losePeer(detail::sockio::Descriptor& peer) {
        peer.reset();
        std::lock_guard lock(_mutex);
        ++nDisconnects;
    }

    /// @brief Accept and close every extra connection, so the client that made one is told rather than left waiting.
    void refuseExtraPeers(detail::sockio::Descriptor& listener) {
        if (!listener.valid()) {
            return;
        }
        while ((detail::sockio::pollFor(listener.get(), POLLIN, 0) & POLLIN) != 0) {
            const int extra = ::accept(listener.get(), nullptr, nullptr);
            if (extra < 0) {
                return;
            }
            std::ignore = ::close(extra);
            std::lock_guard lock(_mutex);
            ++nPeersRefused;
        }
    }

    /// @brief Whether the peer is still there, asked without writing anything to it.
    [[nodiscard]] bool peerAlive(detail::sockio::Descriptor& peer) {
        const short revents = detail::sockio::pollFor(peer.get(), POLLIN, 0);
        if ((revents & (POLLHUP | POLLERR)) != 0) {
            return false;
        }
        if ((revents & POLLIN) != 0) {
            std::array<std::uint8_t, 256UZ> scratch{};
            const ::ssize_t                 received = ::recv(peer.get(), scratch.data(), scratch.size(), MSG_DONTWAIT);
            if (received == 0) {
                return false; // an orderly close from the far end
            }
            if (received < 0 && !detail::sockio::wouldBlock(errno)) {
                return false;
            }
            // a sink's peer is not expected to speak, and whatever it said is not this block's to interpret
        }
        return true;
    }

    /// @brief Write one envelope whole, waiting on the socket rather than on a timer when it is full.
    [[nodiscard]] bool writeAll(detail::sockio::Descriptor& peer, std::span<const std::uint8_t> bytes) {
        std::size_t sent = 0UZ;
        while (sent < bytes.size()) {
            const ::ssize_t written = ::send(peer.get(), bytes.data() + sent, bytes.size() - sent, MSG_NOSIGNAL);
            if (written > 0) {
                sent += static_cast<std::size_t>(written);
                continue;
            }
            if (written < 0 && errno == EINTR) {
                continue;
            }
            if (written < 0 && errno == EAGAIN) {
                const short revents = detail::sockio::pollFor(peer.get(), POLLOUT, detail::sockio::kPollMs);
                if ((revents & (POLLHUP | POLLERR)) != 0 || stopRequested()) {
                    return false;
                }
                continue;
            }
            return false;
        }
        return true;
    }
};

GR_REGISTER_BLOCK(gr::blocks::network::TcpPacketSource, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
requires gr::network::EnvelopeItem<T>
struct TcpPacketSource : Block<TcpPacketSource<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Reads versioned envelopes out of a TCP byte stream and publishes each as one gr::Packet<T>.

A byte stream supplies no message boundaries, so the reader supplies them itself and owes what a message library's
atomicity would otherwise pay for. It holds one buffer and three states.

**Seeking.** Scan for the four magic bytes. Everything the scan passes over is counted in `nBytesSkipped`. Entering
this state counts one `nResyncs` except at a connection's start, where the reader has no alignment to lose: a scan
that has to hunt from a position the reader believed was an envelope boundary counts one, and so does every return
from a header that did not decode.

**Header.** With a magic candidate at the head, accumulate 32 bytes and decode them. A failure means the candidate was
four coincidental bytes or a damaged header: the refusal is counted, no reject packet is published because no body has
been read, the buffer advances one byte past the match and the reader returns to seeking. The header CRC is what makes
that loop terminate at a real header and nowhere else.

**Body.** An envelope whose stated size exceeds `max_message_bytes`, or whose `item_type` is not this block's, is
refused and its body is stepped over **by length rather than buffered** — the CRC-valid header is trusted to size the
skip, which is what the redundant `payload_bytes` field is for, and it is what keeps the stream synchronized through a
refusal. A refusal at this stage therefore publishes no reject packet either, because a reject built from bytes the
reader deliberately did not buffer would be an empty claim. Otherwise the body is accumulated and the metadata parsed,
and a metadata refusal does publish a reject, because those bytes are in hand.

There is no `bad_magic` refusal here and no `length_mismatch`: a stream has no frame sizes to disagree with the header,
and foreign bytes are scanned past rather than named, which is the price of framing that the transport does not supply.

A connection that closes mid-envelope discards the partial envelope and counts it in `nTruncatedEnvelopes`; the loss is
also visible as a `sequence` gap at the next whole envelope. The next connection begins by seeking, because a new
stream owes no alignment to the old one.

The reader dies of its own socket and of nothing else, which is what makes `work::Status::ERROR` mean something. Four
faults end it: the listening socket reporting `POLLERR`, `POLLHUP` or `POLLNVAL`; an `accept` that fails for anything
but a would-block, an interrupt, an aborted connection or a passing shortage of descriptors or memory; a socket the
connecting role cannot create for such a reason; and an endpoint that has stopped resolving, which the connecting role
asks again once a second while it has no peer, so a peer that moves is followed and a name that is gone is fatal. A
shortage retries a bounded number of times before it counts as permanent. What a peer does — closing, resetting,
refusing a connection — costs the connection and nothing more, and the reconnect loop carries on. `nReaderFailures`
counts the reader threads that ended on a fault and `lastReaderError()` names the last one.

Validation happens on the block's own reader thread, so a flood of malformed bytes costs that thread and never the
scheduler, and the queue the graph drains holds only objects whose shape is established. `max_message_bytes` is
required and has no default: it is the bound that decides what a peer's claimed length is allowed to size.
)"">;

    PortOut<Packet<T>, Async>                      out;
    PortOut<Packet<std::uint8_t>, Async, Optional> reject;

    Annotated<std::string, "endpoint", Visible, Doc<"'host:port', e.g. 127.0.0.1:5555 or [::1]:5555; required, there is no default">>       endpoint{};
    Annotated<bool, "bind", Doc<"listen on the endpoint rather than connect to it; a consumer normally connects">>                          bind                = false;
    Annotated<std::uint64_t, "max_message_bytes", Unit<"byte">, Doc<"the largest envelope this reader will buffer; required, must be > 0">> max_message_bytes   = 0ULL;
    Annotated<gr::Size_t, "queue_messages", Doc<"in-process receive queue depth">>                                                          queue_messages      = 1024U;
    Annotated<std::uint64_t, "queue_bytes", Unit<"byte">, Doc<"in-process receive queue size">>                                             queue_bytes         = 16777216ULL;
    Annotated<gr::Size_t, "max_reject_bytes", Unit<"byte">, Doc<"raw bytes of a refused envelope kept for inspection">>                     max_reject_bytes    = 256U;
    Annotated<gr::Size_t, "max_tracked_sources", Doc<"distinct source_id values whose sequence is tracked">>                                max_tracked_sources = 8U;
    Annotated<gr::Size_t, "reconnect_ms", Unit<"ms">, Doc<"interval between connection attempts while connecting">>                         reconnect_ms        = 100U;

    GR_MAKE_REFLECTABLE(TcpPacketSource, out, reject, endpoint, bind, max_message_bytes, queue_messages, queue_bytes, max_reject_bytes, max_tracked_sources, reconnect_ms);

    std::uint64_t nEnvelopesReceived = 0ULL; ///< headers that decoded, each of which identifies one envelope
    std::uint64_t nPacketsPublished  = 0ULL; ///< packets published on out
    std::uint64_t nBytesReceived     = 0ULL; ///< bytes taken off the socket, envelope and skipped alike

    std::uint64_t nBytesSkipped       = 0ULL; ///< bytes the magic scan passed over
    std::uint64_t nResyncs            = 0ULL; ///< entries into the seeking state other than at a connection's start
    std::uint64_t nTruncatedEnvelopes = 0ULL; ///< envelopes a closing connection left incomplete

    std::uint64_t nRefusedVersion      = 0ULL; ///< wire_version 0 or above this reader's; one fault however it is spelled
    std::uint64_t nBadByteOrder        = 0ULL;
    std::uint64_t nBadHeaderBytes      = 0ULL;
    std::uint64_t nBadHeaderCrc        = 0ULL;
    std::uint64_t nUnsupportedItemType = 0ULL;
    std::uint64_t nBadItemSize         = 0ULL;
    std::uint64_t nBadPayloadLength    = 0ULL;
    std::uint64_t nUnknownMetaEncoding = 0ULL;
    std::uint64_t nUnknownFlags        = 0ULL;
    std::uint64_t nItemTypeMismatch    = 0ULL; ///< a well-formed envelope carrying a different item type
    std::uint64_t nOverMax             = 0ULL; ///< an envelope whose stated size exceeds max_message_bytes
    std::uint64_t nBadMetadata         = 0ULL; ///< the metadata bytes did not parse

    std::uint64_t nSequenceGaps          = 0ULL; ///< sequence discontinuities at a tracked source
    std::uint64_t nPacketsLost           = 0ULL; ///< the total size of those gaps
    std::uint64_t nSequenceResets        = 0ULL; ///< a sequence at or below the last seen from that source
    std::uint64_t nSourcesUntracked      = 0ULL; ///< a distinct source_id beyond max_tracked_sources
    std::uint64_t nDroppedByBackpressure = 0ULL; ///< envelopes discarded because the in-process queue was full
    std::uint64_t nOverQueueBytes        = 0ULL; ///< arrivals discarded because one alone exceeds queue_bytes
    std::uint64_t nMetaKeysMistyped      = 0ULL; ///< vocabulary keys whose type disagrees with the declaration
    std::uint64_t nTimestampsCarried     = 0ULL; ///< packet_timestamp values consumed into the carrier field

    std::uint64_t nPeersRefused   = 0ULL; ///< connections accepted and closed because one peer was already served
    std::uint64_t nDisconnects    = 0ULL; ///< connections lost
    std::uint64_t nReconnects     = 0ULL; ///< connections established after the first
    std::uint64_t nRecvErrors     = 0ULL; ///< read failures other than would-block and interrupt
    std::uint64_t nReaderFailures = 0ULL; ///< reader threads that ended on a fault of the socket they own

    /// @brief One decoded arrival, on whichever port it belongs.
    struct Incoming {
        Packet<T>            accepted{};
        Packet<std::uint8_t> refused{};
        bool                 isRefusal = false;
        std::uint64_t        bytes     = 0ULL;
    };

    std::mutex              _mutex;
    std::condition_variable _cv;
    std::deque<Incoming>    _queue;
    std::uint64_t           _queuedBytes   = 0ULL;
    bool                    _stopRequested = false;
    bool                    _opened        = false;
    bool                    _readerFailed  = false;
    std::string             _lastReaderError{}; ///< what ended the reader, empty while it runs and after a requested stop
    std::string             _openFailure{};
    bool                    _ioThreadDone = true;

    /// @brief The last `sequence` seen from a source, with the arrival ordinal that bounds the tracker by eviction.
    struct SourceState {
        std::uint64_t lastSequence = 0ULL;
        std::uint64_t lastSeen     = 0ULL;
    };
    std::vector<std::pair<std::string, SourceState>> _sources{}; ///< at most max_tracked_sources, least recently seen evicted
    std::uint64_t                                    _arrivals = 0ULL;
    std::vector<std::uint16_t>                       _loggedVersions{}; ///< one log line per distinct unsupported version

    /// @brief Where the framing state machine stands in the stream.
    enum class ReadState : std::uint8_t { Seek, Header, Body, Skip };

    std::vector<std::uint8_t>   _buffer{};
    std::size_t                 _cursor        = 0UZ;
    ReadState                   _state         = ReadState::Seek;
    bool                        _aligned       = false; ///< whether the head of the buffer should be an envelope boundary
    std::uint64_t               _skipRemaining = 0ULL;
    gr::network::EnvelopeHeader _header{};

    std::string   _endpoint{};
    bool          _bind              = false;
    std::size_t   _queueMessages     = 1024UZ;
    std::uint64_t _queueBytes        = 16777216ULL;
    std::uint64_t _maxMessageBytes   = 0ULL;
    std::size_t   _maxTrackedSources = 8UZ;
    std::size_t   _maxRejectBytes    = 256UZ;
    std::uint32_t _reconnectMs       = 100U;
    bool          _socketOpen        = false;
    bool          _everConnected     = false;
    std::size_t   _addressCursor     = 0UZ; ///< which resolved address the next connection attempt takes; the I/O thread's alone
    std::size_t   _transientFaults   = 0UZ; ///< consecutive faults of a kind that may lift; the I/O thread's alone

    std::chrono::steady_clock::time_point _resolvedAt{}; ///< when the endpoint was last resolved; the I/O thread's alone

    struct IoThreadGuard { // must be last member — destroyed first, so the reader is gone before the queue it fills
        TcpPacketSource* self;
        explicit IoThreadGuard(TcpPacketSource* owner) noexcept : self(owner) {}
        IoThreadGuard(const IoThreadGuard&)            = delete;
        IoThreadGuard(IoThreadGuard&&)                 = delete;
        IoThreadGuard& operator=(const IoThreadGuard&) = delete;
        IoThreadGuard& operator=(IoThreadGuard&&)      = delete;
        ~IoThreadGuard() { self->requestStopAndJoin(); }
    };
    IoThreadGuard _ioGuard{this};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() {
        validate();
        freezeSocketSettings();
        {
            std::lock_guard lock(_mutex);
            _stopRequested = false;
            _opened        = false;
            _readerFailed  = false;
            _lastReaderError.clear();
            _openFailure.clear();
            _queue.clear();
            _queuedBytes = 0ULL;
        }
        _sources.clear();
        _loggedVersions.clear();
        _arrivals        = 0ULL;
        _everConnected   = false;
        _transientFaults = 0UZ;
        resetStream();
        gr::atomic_ref(_ioThreadDone).store_release(false);
        thread_pool::Manager::defaultIoPool()->execute([this] { ioReadLoop(); });

        std::unique_lock lock(_mutex);
        _cv.wait(lock, [this] { return _opened; });
        if (!_openFailure.empty()) {
            const std::string failure = _openFailure;
            lock.unlock();
            gr::atomic_ref(_ioThreadDone).wait(false);
            throw gr::exception(failure);
        }
        _socketOpen = true;
    }

    void stop() {
        requestStopAndJoin();
        _socketOpen = false;
        report();
    }

    void rebuild() {
        validate();
        if (_socketOpen) {
            refuseFrozenChange();
        }
        _maxRejectBytes = static_cast<std::size_t>(max_reject_bytes.value);
    }

    /// @brief What ended the reader thread, or empty while it runs and after a stop it was asked for.
    ///
    /// Taken under the mutex the reader writes it behind, because the writer is another thread and a string read
    /// while it is being assigned is not a string.
    [[nodiscard]] std::string lastReaderError() {
        std::lock_guard lock(_mutex);
        return _lastReaderError;
    }

    [[nodiscard]] work::Status processBulk(OutputSpanLike auto& outSpan, OutputSpanLike auto& rejectSpan) {
        const bool  outConnected    = outSpan.isConnected;
        const bool  rejectConnected = rejectSpan.isConnected;
        std::size_t onOut           = 0UZ;
        std::size_t onReject        = 0UZ;
        bool        readerFailed    = false;
        bool        drained         = false;

        {
            std::lock_guard lock(_mutex);
            while (!_queue.empty()) {
                Incoming& front = _queue.front();
                if (front.isRefusal) {
                    if (rejectConnected) {
                        if (onReject >= rejectSpan.size()) {
                            break;
                        }
                        rejectSpan[onReject] = std::move(front.refused);
                        ++onReject;
                    }
                    // an unconnected reject port still drains: a refusal must not wedge the queue behind it
                } else {
                    if (outConnected) {
                        if (onOut >= outSpan.size()) {
                            break;
                        }
                        outSpan[onOut] = std::move(front.accepted);
                        ++onOut;
                    }
                    ++nPacketsPublished;
                }
                _queuedBytes -= front.bytes;
                _queue.pop_front();
            }
            readerFailed = _readerFailed;
            drained      = _queue.empty();
        }

        outSpan.publish(outConnected ? onOut : 0UZ);
        rejectSpan.publish(rejectConnected ? onReject : 0UZ);
        if (onOut == 0UZ && onReject == 0UZ) {
            // a reader that died other than by request must be distinguishable from a quiet wire: OK would leave the
            // graph running behind a permanently silent input with nothing to tell a dead endpoint from an idle one
            return readerFailed && drained ? work::Status::ERROR : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    void validate() {
        if (endpoint.value.empty()) {
            throw gr::exception("endpoint is empty; a transport source has no default endpoint and will not connect to nothing");
        }
        if (max_message_bytes.value == 0ULL) {
            throw gr::exception("max_message_bytes is 0 or unset; a byte stream states its own lengths, so a source must state a bound before a peer's claimed length sizes anything");
        }
        if (queue_messages.value == 0U) {
            throw gr::exception("queue_messages is 0; the in-process receive queue must hold at least one envelope");
        }
        if (queue_bytes.value == 0ULL) {
            throw gr::exception("queue_bytes is 0; the in-process receive queue must hold at least one envelope");
        }
        if (max_tracked_sources.value == 0U) {
            throw gr::exception("max_tracked_sources is 0; no source_id would be tracked and no loss would be detected");
        }
        if (static_cast<std::uint64_t>(max_reject_bytes.value) > max_message_bytes.value) {
            throw gr::exception(std::format("max_reject_bytes {} exceeds max_message_bytes {}; a refusal cannot keep more than a message may hold", max_reject_bytes.value, max_message_bytes.value));
        }
        if (reconnect_ms.value == 0U) {
            throw gr::exception("reconnect_ms is 0; a connecting source would retry without pause and spend the thread on nothing else");
        }
    }

    void refuseFrozenChange() const {
        const auto refuse = [](std::string_view name) { throw gr::exception(std::format("setting '{}' is read once when the socket opens and cannot change while the block is running; rebuild the graph instead", name)); };
        if (endpoint.value != _endpoint) {
            refuse("endpoint");
        }
        if (bind.value != _bind) {
            refuse("bind");
        }
        if (max_message_bytes.value != _maxMessageBytes) {
            refuse("max_message_bytes");
        }
        if (static_cast<std::size_t>(queue_messages.value) != _queueMessages) {
            refuse("queue_messages");
        }
        if (queue_bytes.value != _queueBytes) {
            refuse("queue_bytes");
        }
        if (static_cast<std::size_t>(max_tracked_sources.value) != _maxTrackedSources) {
            refuse("max_tracked_sources");
        }
        if (reconnect_ms.value != _reconnectMs) {
            refuse("reconnect_ms");
        }
    }

    void freezeSocketSettings() {
        _endpoint          = endpoint.value;
        _bind              = bind.value;
        _maxMessageBytes   = max_message_bytes.value;
        _queueMessages     = static_cast<std::size_t>(queue_messages.value);
        _queueBytes        = queue_bytes.value;
        _maxTrackedSources = static_cast<std::size_t>(max_tracked_sources.value);
        _maxRejectBytes    = static_cast<std::size_t>(max_reject_bytes.value);
        _reconnectMs       = reconnect_ms.value;
        _addressCursor     = 0UZ;
    }

    void requestStopAndJoin() {
        {
            std::lock_guard lock(_mutex);
            _stopRequested = true;
        }
        _cv.notify_all();
        gr::atomic_ref(_ioThreadDone).wait(false);
    }

    void report() {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("envelopes received", nEnvelopesReceived);
        append("packets published", nPacketsPublished);
        append("bytes received", nBytesReceived);
        append("bytes skipped", nBytesSkipped);
        append("resyncs", nResyncs);
        append("truncated envelopes", nTruncatedEnvelopes);
        append("refused version", nRefusedVersion);
        append("bad byte order", nBadByteOrder);
        append("bad header bytes", nBadHeaderBytes);
        append("bad header crc", nBadHeaderCrc);
        append("unsupported item type", nUnsupportedItemType);
        append("bad item size", nBadItemSize);
        append("bad payload length", nBadPayloadLength);
        append("unknown meta encoding", nUnknownMetaEncoding);
        append("unknown flags", nUnknownFlags);
        append("item type mismatch", nItemTypeMismatch);
        append("over max", nOverMax);
        append("bad metadata", nBadMetadata);
        append("sequence gaps", nSequenceGaps);
        append("packets lost", nPacketsLost);
        append("sequence resets", nSequenceResets);
        append("sources untracked", nSourcesUntracked);
        append("dropped by backpressure", nDroppedByBackpressure);
        append("over queue bytes", nOverQueueBytes);
        append("metadata keys mistyped", nMetaKeysMistyped);
        append("timestamps carried", nTimestampsCarried);
        append("peers refused", nPeersRefused);
        append("disconnects", nDisconnects);
        append("reconnects", nReconnects);
        append("receive errors", nRecvErrors);
        append("reader failures", nReaderFailures);
        if (!_lastReaderError.empty()) {
            std::format_to(std::back_inserter(report), "{}reader ended: {}", report.empty() ? "" : ", ", _lastReaderError);
        }
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::network::TcpPacketSource '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] bool stopRequested() {
        std::lock_guard lock(_mutex);
        return _stopRequested;
    }

    void waitBounded(std::uint32_t milliseconds) {
        std::unique_lock lock(_mutex);
        std::ignore = _cv.wait_for(lock, std::chrono::milliseconds(milliseconds), [this] { return _stopRequested; });
    }

    void ioReadLoop() {
        thread_pool::thread::setThreadName(std::format("tcppktsrc:{}", this->name.value));
        detail::sockio::Descriptor           listener;
        std::string                          failure;
        std::vector<detail::sockio::Address> addresses;

        auto resolved = detail::sockio::resolveEndpoint(_endpoint, SOCK_STREAM, _bind);
        if (!resolved.has_value()) {
            failure = resolved.error();
        } else {
            addresses   = std::move(*resolved);
            _resolvedAt = std::chrono::steady_clock::now();
            if (_bind) {
                failure = openListener(listener, addresses);
            } else {
                const detail::sockio::Descriptor probe(::socket(addresses.front().family, SOCK_STREAM, 0));
                if (!probe.valid()) {
                    failure = std::format("cannot create a TCP socket for '{}': {}", _endpoint, detail::sockio::errorText(errno));
                }
            }
        }

        {
            std::lock_guard lock(_mutex);
            _openFailure = failure;
            _opened      = true;
        }
        _cv.notify_all();

        std::string fatal;
        if (failure.empty()) {
            fatal = receiveUntilStopped(listener, addresses);
        }
        {
            // the reader is gone; if nobody asked it to go, the wire is not quiet but dead, and processBulk owes the
            // graph that distinction once the queue behind it has drained
            std::lock_guard lock(_mutex);
            if (!fatal.empty()) {
                _lastReaderError = fatal;
                ++nReaderFailures;
            }
            _readerFailed = !_stopRequested;
        }
        _cv.notify_all();
        gr::atomic_ref(_ioThreadDone).store_release(true);
        gr::atomic_ref(_ioThreadDone).notify_all();
    }

    /// @brief What a fault at a socket this thread owns costs: the reason the reader ends, or empty to carry on.
    ///
    /// A shortage of descriptors or memory is given @ref detail::sockio::kTransientFaultLimit consecutive attempts,
    /// spaced by one poll interval, before it is called permanent: a shortage that never lifts is as final as a broken
    /// descriptor, and a reader that hid it would spin against the same failure for the life of the graph.
    [[nodiscard]] std::string noteFault(int code, std::string_view what) {
        switch (detail::sockio::classifyFault(code)) {
        case detail::sockio::Fault::Retry: return {};
        case detail::sockio::Fault::Transient:
            ++_transientFaults;
            if (_transientFaults < detail::sockio::kTransientFaultLimit) {
                waitBounded(static_cast<std::uint32_t>(detail::sockio::kPollMs));
                return {};
            }
            return std::format("{} failed {} times running: {}", what, _transientFaults, detail::sockio::errorText(code));
        case detail::sockio::Fault::Fatal: break;
        }
        return std::format("{} failed: {}", what, detail::sockio::errorText(code));
    }

    /// @brief Resolve the endpoint again for the next pass over its addresses, or say why it no longer resolves.
    ///
    /// A name is not fixed for the life of a graph: a peer that moved is reachable again on the next pass, and one
    /// whose name is gone is unreachable for good, which is the one resolution failure that ends the reader. A
    /// resolver that cannot answer just now keeps the addresses it last gave and the retries carry on with them. The
    /// question is asked at most once every @ref detail::sockio::kResolveIntervalMs, whatever `reconnect_ms` is, so a
    /// fast retry loop does not become a flood of queries.
    [[nodiscard]] std::string resolveAgain(std::vector<detail::sockio::Address>& addresses) {
        const auto now = std::chrono::steady_clock::now();
        if (now - _resolvedAt < std::chrono::milliseconds(detail::sockio::kResolveIntervalMs)) {
            return {};
        }
        _resolvedAt   = now;
        int  status   = 0;
        auto resolved = detail::sockio::resolveEndpoint(_endpoint, SOCK_STREAM, false, &status);
        if (resolved.has_value()) {
            addresses = std::move(*resolved);
            return {};
        }
        return status == EAI_AGAIN ? std::string{} : resolved.error();
    }

    [[nodiscard]] std::string openListener(detail::sockio::Descriptor& listener, std::span<const detail::sockio::Address> addresses) {
        std::string lastError = "no address was tried";
        for (const detail::sockio::Address& address : addresses) {
            detail::sockio::Descriptor candidate(::socket(address.family, SOCK_STREAM, 0));
            if (!candidate.valid()) {
                lastError = detail::sockio::errorText(errno);
                continue;
            }
            const int on = 1; // so a restarted graph can rebind its port inside TIME_WAIT
            std::ignore  = ::setsockopt(candidate.get(), SOL_SOCKET, SO_REUSEADDR, &on, static_cast<::socklen_t>(sizeof(on)));
            if (::bind(candidate.get(), detail::sockio::addressOf(address), address.length) != 0) {
                lastError = detail::sockio::errorText(errno);
                continue;
            }
            if (::listen(candidate.get(), 8) != 0) {
                lastError = detail::sockio::errorText(errno);
                continue;
            }
            std::ignore = detail::sockio::setNonBlocking(candidate.get());
            listener    = std::move(candidate);
            return {};
        }
        return std::format("cannot listen on '{}': {}", _endpoint, lastError);
    }

    /// @brief Serve peers until a stop is asked for, or name the fault of this thread's own socket that ends it.
    [[nodiscard]] std::string receiveUntilStopped(detail::sockio::Descriptor& listener, std::vector<detail::sockio::Address>& addresses) {
        detail::sockio::Descriptor peer;
        std::vector<std::uint8_t>  chunk(65536UZ);

        while (!stopRequested()) {
            if (!peer.valid()) {
                if (std::string fatal = acquirePeer(listener, peer, addresses); !fatal.empty()) {
                    return fatal;
                }
                continue;
            }
            if (std::string fatal = refuseExtraPeers(listener); !fatal.empty()) {
                return fatal;
            }

            const short revents = detail::sockio::pollFor(peer.get(), POLLIN, detail::sockio::kPollMs);
            if (revents == 0) {
                continue; // the bounded wait expired, which is how the loop returns to observe a stop request
            }
            const ::ssize_t received = ::recv(peer.get(), chunk.data(), chunk.size(), 0);
            if (received > 0) {
                const std::size_t count = static_cast<std::size_t>(received);
                nBytesReceived += count;
                feed(std::span<const std::uint8_t>(chunk).first(count));
                continue;
            }
            if (received < 0 && detail::sockio::wouldBlock(errno)) {
                continue;
            }
            if (received < 0) {
                ++nRecvErrors;
            }
            losePeer(peer); // a fault on the connection costs the connection; the listening socket is still good
        }
        return {};
    }

    /// @brief Take the next peer, or name the fault of this thread's own socket that ends the reader.
    [[nodiscard]] std::string acquirePeer(detail::sockio::Descriptor& listener, detail::sockio::Descriptor& peer, std::vector<detail::sockio::Address>& addresses) {
        if (_bind) {
            const short revents = detail::sockio::pollFor(listener.get(), POLLIN, detail::sockio::kPollMs);
            if ((revents & detail::sockio::kPollFault) != 0) {
                return std::format("the listening socket reported {}", detail::sockio::pollFaultText(revents));
            }
            if ((revents & POLLIN) == 0) {
                return {};
            }
            const int accepted = ::accept(listener.get(), nullptr, nullptr);
            if (accepted < 0) {
                return noteFault(errno, "accept");
            }
            _transientFaults = 0UZ;
            adoptPeer(peer, accepted);
            return {};
        }

        // one endpoint may resolve to several addresses, so each attempt takes the next of them in turn: pinning every
        // retry to the first would leave a host reachable only on its second address permanently unreachable. A pass
        // that has been through them all asks the name again before the next one begins.
        if (_addressCursor >= addresses.size()) {
            if (std::string fatal = resolveAgain(addresses); !fatal.empty()) {
                return fatal;
            }
            _addressCursor = 0UZ;
        }
        const detail::sockio::Address& address = addresses[_addressCursor];
        ++_addressCursor;

        detail::sockio::Descriptor candidate(::socket(address.family, SOCK_STREAM, 0));
        if (!candidate.valid()) {
            if (std::string fatal = noteFault(errno, "creating a TCP socket"); !fatal.empty()) {
                return fatal;
            }
            waitBounded(_reconnectMs);
            return {};
        }
        _transientFaults = 0UZ;
        std::ignore      = detail::sockio::setNonBlocking(candidate.get());
        // whatever the far end answers with — a refusal, a silence, a route that is gone — is the far end's, not this
        // socket's, so every failure from here on waits out the reconnect interval and tries again
        if (::connect(candidate.get(), detail::sockio::addressOf(address), address.length) != 0) {
            if (errno != EINPROGRESS) {
                waitBounded(_reconnectMs);
                return {};
            }
            const short revents = detail::sockio::pollFor(candidate.get(), POLLOUT, detail::sockio::kPollMs);
            int         pending = 0;
            ::socklen_t length  = static_cast<::socklen_t>(sizeof(pending));
            if ((revents & POLLOUT) == 0 || ::getsockopt(candidate.get(), SOL_SOCKET, SO_ERROR, &pending, &length) != 0 || pending != 0) {
                waitBounded(_reconnectMs);
                return {};
            }
        }
        adoptPeer(peer, candidate.release());
        return {};
    }

    void adoptPeer(detail::sockio::Descriptor& peer, int fd) {
        const int on = 1;
        std::ignore  = ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, static_cast<::socklen_t>(sizeof(on)));
        std::ignore  = detail::sockio::setNonBlocking(fd);
        peer.reset(fd);
        resetStream(); // a new stream owes no alignment to the old one, so the reader begins by seeking
        if (_everConnected) {
            ++nReconnects;
        }
        _everConnected = true;
    }

    void losePeer(detail::sockio::Descriptor& peer) {
        peer.reset();
        if (_state != ReadState::Seek) {
            ++nTruncatedEnvelopes; // the partial envelope is discarded; the loss also shows as a sequence gap
        }
        resetStream();
        ++nDisconnects;
    }

    [[nodiscard]] std::string refuseExtraPeers(detail::sockio::Descriptor& listener) {
        if (!listener.valid()) {
            return {};
        }
        for (;;) {
            const short revents = detail::sockio::pollFor(listener.get(), POLLIN, 0);
            if ((revents & detail::sockio::kPollFault) != 0) {
                return std::format("the listening socket reported {}", detail::sockio::pollFaultText(revents));
            }
            if ((revents & POLLIN) == 0) {
                return {};
            }
            const int extra = ::accept(listener.get(), nullptr, nullptr);
            if (extra < 0) {
                return noteFault(errno, "accept");
            }
            std::ignore      = ::close(extra); // refusing at accept is the only way the extra client learns anything at all
            _transientFaults = 0UZ;
            ++nPeersRefused;
        }
    }

    void resetStream() {
        _buffer.clear();
        _cursor        = 0UZ;
        _state         = ReadState::Seek;
        _aligned       = false;
        _skipRemaining = 0ULL;
    }

    [[nodiscard]] std::span<const std::uint8_t> pending() const noexcept { return std::span<const std::uint8_t>(_buffer).subspan(_cursor); }

    void discard(std::size_t count) {
        _cursor += count;
        if (_cursor >= _buffer.size()) {
            _buffer.clear();
            _cursor = 0UZ;
        } else if (_cursor > 4096UZ && _cursor > _buffer.size() / 2UZ) {
            _buffer.erase(_buffer.begin(), _buffer.begin() + static_cast<std::ptrdiff_t>(_cursor));
            _cursor = 0UZ;
        }
    }

    /// @brief Note that the reader has lost envelope alignment, once per episode rather than once per byte.
    void noteResync() {
        if (_aligned) {
            ++nResyncs;
            _aligned = false;
        }
    }

    void feed(std::span<const std::uint8_t> bytes) {
        _buffer.insert(_buffer.end(), bytes.begin(), bytes.end());
        while (advance()) {
        }
    }

    /// @brief One step of the framing state machine. False means it needs more bytes than the buffer holds.
    [[nodiscard]] bool advance() {
        switch (_state) {
        case ReadState::Seek: return seek();
        case ReadState::Header: return readHeader();
        case ReadState::Body: return readBody();
        case ReadState::Skip: return skipBody();
        }
        return false;
    }

    [[nodiscard]] bool seek() {
        const std::span<const std::uint8_t> bytes = pending();
        const auto                          found = std::ranges::search(bytes, gr::network::kMagic);
        if (!found.empty()) {
            const std::size_t skipped = static_cast<std::size_t>(std::ranges::distance(bytes.begin(), found.begin()));
            if (skipped > 0UZ) {
                noteResync();
                nBytesSkipped += skipped;
                discard(skipped);
            }
            _state = ReadState::Header;
            return true;
        }
        // no match, so everything but a tail that could still be the start of one is scanned past
        const std::size_t keep    = std::min(bytes.size(), gr::network::kMagic.size() - 1UZ);
        const std::size_t skipped = bytes.size() - keep;
        if (skipped > 0UZ) {
            noteResync();
            nBytesSkipped += skipped;
            discard(skipped);
        }
        return false;
    }

    [[nodiscard]] bool readHeader() {
        const std::span<const std::uint8_t> bytes = pending();
        if (bytes.size() < gr::network::kHeaderBytesV1) {
            return false;
        }
        const std::span<const std::uint8_t> headerBytes = bytes.first(gr::network::kHeaderBytesV1);
        const auto                          header      = gr::network::decodeHeader(headerBytes);
        if (!header.has_value()) {
            countKernelRefusal(header.error(), headerBytes);
            // no reject packet: the body was never read, so a refusal built here would be an empty claim
            ++nResyncs; // a candidate that did not decode always returns the reader to seeking
            _aligned = false;
            ++nBytesSkipped;
            discard(1UZ);
            _state = ReadState::Seek;
            return true;
        }

        ++nEnvelopesReceived;
        _header                  = *header;
        const std::uint64_t body = static_cast<std::uint64_t>(_header.meta_bytes) + static_cast<std::uint64_t>(_header.payload_bytes);
        if (static_cast<std::uint64_t>(_header.header_bytes) + body > _maxMessageBytes) {
            ++nOverMax;
            return beginSkip(body);
        }
        if (_header.item_type != gr::network::kItemTypeCode<T>) {
            ++nItemTypeMismatch;
            return beginSkip(body);
        }
        _state = ReadState::Body;
        return true;
    }

    /// @brief Step over a refused body by its stated length, which is what keeps the stream synchronized.
    ///
    /// The header's CRC is what makes its lengths trustworthy enough to be used this way, and using them is the
    /// resynchronization path the redundant `payload_bytes` field exists for.
    [[nodiscard]] bool beginSkip(std::uint64_t body) {
        discard(gr::network::kHeaderBytesV1);
        _skipRemaining = body;
        if (_skipRemaining == 0ULL) {
            finishEnvelope();
            return true;
        }
        _state = ReadState::Skip;
        return true;
    }

    [[nodiscard]] bool skipBody() {
        const std::span<const std::uint8_t> bytes = pending();
        if (bytes.empty()) {
            return false;
        }
        const std::size_t step = detail::sockio::narrowIndex<std::size_t>(std::min<std::uint64_t>(bytes.size(), _skipRemaining));
        discard(step);
        _skipRemaining -= step;
        if (_skipRemaining == 0ULL) {
            finishEnvelope();
        }
        return true;
    }

    [[nodiscard]] bool readBody() {
        const std::size_t                   need  = gr::network::kHeaderBytesV1 + _header.meta_bytes + std::size_t{_header.payload_bytes};
        const std::span<const std::uint8_t> bytes = pending();
        if (bytes.size() < need) {
            return false;
        }
        handleEnvelope(bytes.first(need));
        discard(need);
        finishEnvelope();
        return true;
    }

    /// @brief Leave the machine at an envelope boundary, where a later hunt for the magic is a resynchronization.
    void finishEnvelope() {
        _state   = ReadState::Seek;
        _aligned = true;
    }

    /// @brief Validate one whole envelope and enqueue either the packet it carries or the refusal it earned.
    void handleEnvelope(std::span<const std::uint8_t> envelope) {
        const std::uint64_t                 total    = envelope.size();
        const std::span<const std::uint8_t> metadata = envelope.subspan(gr::network::kHeaderBytesV1, _header.meta_bytes);
        const std::span<const std::uint8_t> payload  = envelope.subspan(std::size_t{gr::network::kHeaderBytesV1} + _header.meta_bytes, _header.payload_bytes);

        property_map map;
        if (_header.meta_bytes != 0U) { // the empty map short-circuits rather than asking what an empty document means
            const std::string_view text(reinterpret_cast<const char*>(metadata.data()), metadata.size());
            const auto             parsed = pmt::yaml::deserialize(text);
            if (!parsed.has_value()) {
                ++nBadMetadata;
                std::println(stderr, "gr::blocks::network::TcpPacketSource '{}': metadata line {} column {}: {}", this->name, parsed.error().line, parsed.error().column, parsed.error().message);
                enqueueRefusal(envelope, total, "bad_metadata");
                return;
            }
            map = *parsed;
        }
        nMetaKeysMistyped += detail::countMistypedKeys(map);

        Incoming arrival;
        arrival.accepted.signal_values.resize(_header.item_count);
        if (_header.payload_bytes != 0U) {
            std::memcpy(arrival.accepted.signal_values.data(), payload.data(), _header.payload_bytes);
        }
        if (const auto stamp = map.find(detail::sockio::kTimestampKey); stamp != map.end()) {
            if (const std::int64_t* value = stamp->second.get_if<std::int64_t>(); value != nullptr) {
                arrival.accepted.timestamp = *value;
                ++nTimestampsCarried;
            }
            map.erase(stamp->first); // consumed, so the carrier field is the value's only spelling after the crossing
        }
        trackSequence(map);
        arrival.accepted.meta_information.resize(1UZ);
        arrival.accepted.meta_information[0UZ] = std::move(map);
        arrival.bytes                          = total;
        enqueue(std::move(arrival));
    }

    void countKernelRefusal(gr::network::EnvelopeError error, std::span<const std::uint8_t> headerBytes) {
        using gr::network::EnvelopeError;
        switch (error) {
        case EnvelopeError::BadMagic: return; // unreachable: the seek state only leaves a matched magic at the head
        case EnvelopeError::BadVersion:
        case EnvelopeError::FutureVersion: {
            ++nRefusedVersion; // a peer at the wrong version is one fault however it is spelled
            const std::uint16_t version = static_cast<std::uint16_t>(static_cast<std::uint16_t>(headerBytes[4UZ]) | static_cast<std::uint16_t>(static_cast<std::uint16_t>(headerBytes[5UZ]) << 8U));
            if (std::ranges::find(_loggedVersions, version) == _loggedVersions.end()) {
                _loggedVersions.push_back(version); // once per distinct value: a mismatched peer at rate would flood
                std::println(stderr, "gr::blocks::network::TcpPacketSource '{}': refusing wire version {} on '{}'; this reader implements version {}", this->name, version, _endpoint, gr::network::kWireVersion);
            }
            return;
        }
        case EnvelopeError::ByteOrder: ++nBadByteOrder; return;
        case EnvelopeError::BadHeaderBytes: ++nBadHeaderBytes; return;
        case EnvelopeError::BadHeaderCrc: ++nBadHeaderCrc; return;
        case EnvelopeError::UnsupportedItemType: ++nUnsupportedItemType; return;
        case EnvelopeError::BadItemSize: ++nBadItemSize; return;
        case EnvelopeError::PayloadLengthField: ++nBadPayloadLength; return;
        case EnvelopeError::UnknownMetaEncoding: ++nUnknownMetaEncoding; return;
        case EnvelopeError::UnknownFlags: ++nUnknownFlags; return;
        }
    }

    /// @brief Publish the raw bytes of a refused envelope so the failure can be read without a packet capture.
    ///
    /// `max_reject_bytes` bounds the copy, so a hostile envelope cannot be pulled whole into the graph by the very
    /// path that refused it. There is no frame count to state, because a stream has no frames.
    void enqueueRefusal(std::span<const std::uint8_t> envelope, std::uint64_t total, std::string_view reason) {
        Incoming arrival;
        arrival.isRefusal = true;
        arrival.bytes     = total;

        std::vector<std::uint8_t>& bytes = arrival.refused.signal_values;
        const std::size_t          kept  = std::min(envelope.size(), _maxRejectBytes);
        bytes.assign(envelope.begin(), envelope.begin() + static_cast<std::ptrdiff_t>(kept));

        arrival.refused.meta_information.resize(1UZ);
        property_map& map = arrival.refused.meta_information[0UZ];
        map.insert_or_assign(property_map::key_type("discard_reason"), pmt::Value(std::string(reason)));
        map.insert_or_assign(property_map::key_type("envelope_bytes_total"), pmt::Value(total));
        const std::uint64_t keptBytes = bytes.size();
        map.insert_or_assign(property_map::key_type("envelope_bytes_kept"), pmt::Value(keptBytes));
        enqueue(std::move(arrival));
    }

    /// @brief Put one decoded arrival on the queue `processBulk` drains, shedding the oldest where it does not fit.
    ///
    /// An arrival larger than `queue_bytes` is discarded and counted rather than queued: shedding what is queued
    /// cannot make room for one, and queueing it anyway would put the queue above the bound the graph set. A source
    /// has no upstream to refuse, so the count is where the loss is stated, as it is for every other loss here.
    void enqueue(Incoming&& arrival) {
        {
            std::lock_guard lock(_mutex);
            if (arrival.bytes > _queueBytes) {
                ++nOverQueueBytes;
                return;
            }
            while (_queue.size() >= _queueMessages || _queuedBytes + arrival.bytes > _queueBytes) {
                _queuedBytes -= _queue.front().bytes;
                _queue.pop_front();
                ++nDroppedByBackpressure; // the one loss class this end counts exactly, which is what makes the
                                          // subtraction from nPacketsLost separate a slow graph from a lossy wire
            }
            _queuedBytes += arrival.bytes;
            _queue.push_back(std::move(arrival));
        }
        _cv.notify_one();
    }

    /// @brief Update the per-source sequence tracker and count what the gaps imply.
    void trackSequence(const property_map& map) {
        const auto sequenceEntry = map.find("sequence");
        if (sequenceEntry == map.end()) {
            return; // a hand-built peer may omit it; a peer built from these blocks cannot
        }
        const std::uint64_t* sequence = sequenceEntry->second.get_if<std::uint64_t>();
        if (sequence == nullptr) {
            return; // already counted as a mistyped vocabulary key
        }
        std::string sourceId;
        if (const auto entry = map.find("source_id"); entry != map.end()) {
            if (const std::pmr::string* value = entry->second.get_if<std::pmr::string>(); value != nullptr) {
                sourceId.assign(value->begin(), value->end());
            }
        }

        ++_arrivals;
        const auto found = std::ranges::find(_sources, sourceId, [](const auto& entry) { return entry.first; });
        if (found != _sources.end()) {
            SourceState& state = found->second;
            if (*sequence > state.lastSequence + 1ULL) {
                ++nSequenceGaps;
                nPacketsLost += *sequence - state.lastSequence - 1ULL;
            } else if (*sequence <= state.lastSequence) {
                ++nSequenceResets; // a producer restarted or a second producer shares the id; not loss, because it is not
            }
            state.lastSequence = *sequence;
            state.lastSeen     = _arrivals;
            return;
        }

        // the key comes off the wire, so the number of them is bounded before it sizes anything
        if (_sources.size() >= _maxTrackedSources) {
            const auto stalest = std::ranges::min_element(_sources, {}, [](const auto& entry) { return entry.second.lastSeen; });
            _sources.erase(stalest);
            ++nSourcesUntracked;
        }
        _sources.emplace_back(std::move(sourceId), SourceState{.lastSequence = *sequence, .lastSeen = _arrivals});
        // the first envelope from a source establishes the baseline and is never a gap: a reader has no history
    }
};

GR_REGISTER_BLOCK(gr::blocks::network::UdpPacketSink, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
requires gr::network::EnvelopeItem<T>
struct UdpPacketSink : Block<UdpPacketSink<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Sends each incoming gr::Packet<T> to a UDP endpoint as exactly one datagram.

An envelope never spans datagrams and a datagram never carries more than one envelope. A datagram is the transport's
own indivisible unit, so aligning the envelope to it means every loss is a whole-envelope loss, visible at the
receiver as a `sequence` gap, and the receiver needs no state across datagrams — reordering and duplication then
degrade into the sequence tracker's existing gap and reset accounting instead of corrupting a reassembly buffer.

There is no `bind` setting, because there is no third arrangement: a datagram sink sends to its endpoint. The socket
is connected to the resolved target once at `start()`, which fixes the destination for every later send and lets the
kernel report an ICMP port-unreachable back as a send error rather than dropping it unattributed.

An envelope larger than `max_datagram_bytes` is refused to `reject` with `discard_reason = "over_max_datagram"` — the
datagram spelling of the stream transports' size refusal. The default `65507` is the IPv4 UDP maximum and admits
envelopes that IP will fragment; a deployment that must avoid fragmentation lowers the setting toward its path MTU,
which this block states rather than probes, because path MTU is a property of a route and not of a block.

A send that fails costs one envelope and never the thread: the error is counted in `nSendErrors`, logged once per
distinct cause, and the next packet is attempted normally.
)"">;

    PortIn<Packet<T>>                   in;
    PortOut<Packet<T>, Async, Optional> reject;

    Annotated<std::string, "endpoint", Visible, Doc<"'host:port', e.g. 127.0.0.1:5555 or [::1]:5555; required, there is no default">>               endpoint{};
    Annotated<std::string, "overflow", Doc<"'drop_oldest' or 'backpressure', applied when the in-process send queue is full">>                      overflow           = std::string("drop_oldest");
    Annotated<gr::Size_t, "queue_messages", Doc<"in-process send queue depth">>                                                                     queue_messages     = 1024U;
    Annotated<std::uint64_t, "queue_bytes", Unit<"byte">, Doc<"in-process send queue size">>                                                        queue_bytes        = 16777216ULL;
    Annotated<std::uint64_t, "max_datagram_bytes", Unit<"byte">, Doc<"a packet whose envelope would exceed one datagram of this size is rejected">> max_datagram_bytes = 65507ULL;

    GR_MAKE_REFLECTABLE(UdpPacketSink, in, reject, endpoint, overflow, queue_messages, queue_bytes, max_datagram_bytes);

    std::uint64_t nPacketsSent          = 0ULL; ///< datagrams handed to the kernel
    std::uint64_t nBytesSent            = 0ULL; ///< envelope bytes handed to the kernel
    std::uint64_t nRejectedPackets      = 0ULL; ///< packets refused for exceeding max_datagram_bytes
    std::uint64_t nOverQueueBytes       = 0ULL; ///< packets refused because one envelope exceeds queue_bytes
    std::uint64_t nDroppedOnOverflow    = 0ULL; ///< queued envelopes discarded under overflow = drop_oldest
    std::uint64_t nBackpressureStalls   = 0ULL; ///< processBulk calls that consumed fewer items than they read
    std::uint64_t nSequenceDeclined     = 0ULL; ///< packets that already stated sequence
    std::uint64_t nMetaKeysMistyped     = 0ULL; ///< vocabulary keys whose type disagrees with the declaration
    std::uint64_t nTimestampsCarried    = 0ULL; ///< packets with a non-zero Packet::timestamp
    std::uint64_t nDefaultValuesDropped = 0ULL; ///< packets whose default_value differs from T(), which has no wire field
    std::uint64_t nDroppedAtStop        = 0ULL; ///< envelopes still queued when the I/O thread stopped
    std::uint64_t nSendErrors           = 0ULL; ///< datagrams lost to a send that failed

    std::mutex                            _mutex;
    std::condition_variable               _cv;
    std::deque<std::vector<std::uint8_t>> _queue;
    std::uint64_t                         _queuedBytes   = 0ULL;
    bool                                  _stopRequested = false;
    bool                                  _opened        = false;
    std::string                           _openFailure{};
    bool                                  _ioThreadDone = true;

    std::uint64_t    _sequence = 0ULL;
    std::string      _endpoint{};
    std::size_t      _queueMessages    = 1024UZ;
    std::uint64_t    _queueBytes       = 16777216ULL;
    bool             _socketOpen       = false;
    bool             _backpressure     = false;
    std::uint64_t    _maxDatagramBytes = 65507ULL;
    std::vector<int> _loggedSendErrors{}; ///< one log line per distinct cause, so a dead peer cannot flood

    struct IoThreadGuard { // must be last member — destroyed first, so the sender is gone before the queue it drains
        UdpPacketSink* self;
        explicit IoThreadGuard(UdpPacketSink* owner) noexcept : self(owner) {}
        IoThreadGuard(const IoThreadGuard&)            = delete;
        IoThreadGuard(IoThreadGuard&&)                 = delete;
        IoThreadGuard& operator=(const IoThreadGuard&) = delete;
        IoThreadGuard& operator=(IoThreadGuard&&)      = delete;
        ~IoThreadGuard() { self->requestStopAndJoin(); }
    };
    IoThreadGuard _ioGuard{this};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() {
        validate();
        freezeSocketSettings();
        {
            std::lock_guard lock(_mutex);
            _stopRequested = false;
            _opened        = false;
            _openFailure.clear();
            _queue.clear();
            _queuedBytes = 0ULL;
        }
        _loggedSendErrors.clear();
        gr::atomic_ref(_ioThreadDone).store_release(false);
        thread_pool::Manager::defaultIoPool()->execute([this] { ioSendLoop(); });

        std::unique_lock lock(_mutex);
        _cv.wait(lock, [this] { return _opened; });
        if (!_openFailure.empty()) {
            const std::string failure = _openFailure;
            lock.unlock();
            gr::atomic_ref(_ioThreadDone).wait(false);
            throw gr::exception(failure);
        }
        _socketOpen = true;
    }

    void stop() {
        requestStopAndJoin();
        _socketOpen = false;
        report();
    }

    void rebuild() {
        validate();
        if (_socketOpen) {
            refuseFrozenChange();
        }
        _backpressure     = overflow.value == "backpressure";
        _maxDatagramBytes = max_datagram_bytes.value;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& rejectSpan) {
        const bool  rejectConnected = rejectSpan.isConnected;
        std::size_t consumed        = 0UZ;
        std::size_t onReject        = 0UZ;

        for (std::size_t i = 0UZ; i < inSpan.size(); ++i) {
            const Packet<T>& packet = inSpan[i];

            std::uint64_t payloadBytes = packet.signal_values.size();
            payloadBytes *= sizeof(T);
            property_map      map      = buildMetadata(packet);
            const std::string metadata = pmt::yaml::serialize(map);

            const std::uint64_t headerBytes = gr::network::kHeaderBytesV1;
            const std::uint64_t total       = headerBytes + metadata.size() + payloadBytes;
            // the two size bounds, in the order they bind: what one datagram will carry, then what the queue cannot
            // hold. A queue cannot shed its way to room for an envelope larger than the whole of it, so such an
            // envelope is refused here rather than admitted over the bound the graph set.
            const bool overDatagram = total > _maxDatagramBytes;
            const bool overQueue    = !overDatagram && total > _queueBytes;
            if (overDatagram || overQueue) {
                if (rejectConnected && onReject >= rejectSpan.size()) {
                    break; // no room on the port this packet belongs on; it stays in the buffer
                }
                if (rejectConnected) {
                    Packet<T> refused = packet; // republished whole: what is wrong with it is its size, not its content
                    refused.meta_information.resize(1UZ);
                    refused.meta_information[0UZ].insert_or_assign(property_map::key_type("discard_reason"), pmt::Value(std::string(overDatagram ? "over_max_datagram" : "over_queue_bytes")));
                    rejectSpan[onReject] = std::move(refused);
                }
                ++onReject;
                if (overDatagram) {
                    ++nRejectedPackets;
                } else {
                    ++nOverQueueBytes;
                }
                ++consumed;
                continue; // the sequence counter never advanced, so a refused packet does not renumber the stream
            }

            if (!enqueue(assemble(packet, metadata, payloadBytes, total))) {
                ++nBackpressureStalls;
                break;
            }
            ++_sequence;
            ++consumed;
        }

        std::ignore = inSpan.consume(consumed);
        rejectSpan.publish(rejectConnected ? onReject : 0UZ);
        if (consumed == 0UZ) {
            return inSpan.size() == 0UZ ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    void validate() {
        if (endpoint.value.empty()) {
            throw gr::exception("endpoint is empty; a transport sink has no default endpoint and will not send to nothing");
        }
        if (overflow.value != "drop_oldest" && overflow.value != "backpressure") {
            throw gr::exception(std::format("overflow is '{}'; it must be 'drop_oldest' (shed the stalest envelope, count it) or 'backpressure' (consume fewer input items)", overflow.value));
        }
        if (queue_messages.value == 0U) {
            throw gr::exception("queue_messages is 0; the in-process send queue must hold at least one envelope");
        }
        if (queue_bytes.value == 0ULL) {
            throw gr::exception("queue_bytes is 0; the in-process send queue must hold at least one envelope");
        }
        if (max_datagram_bytes.value == 0ULL) {
            throw gr::exception("max_datagram_bytes is 0; every packet would be rejected");
        }
        if (max_datagram_bytes.value > 65507ULL) {
            throw gr::exception(std::format("max_datagram_bytes is {}; 65507 is the largest payload an IPv4 UDP datagram can carry", max_datagram_bytes.value));
        }
    }

    void refuseFrozenChange() const {
        const auto refuse = [](std::string_view name) { throw gr::exception(std::format("setting '{}' is read once when the socket opens and cannot change while the block is running; rebuild the graph instead", name)); };
        if (endpoint.value != _endpoint) {
            refuse("endpoint");
        }
        if (static_cast<std::size_t>(queue_messages.value) != _queueMessages) {
            refuse("queue_messages");
        }
        if (queue_bytes.value != _queueBytes) {
            refuse("queue_bytes");
        }
    }

    void freezeSocketSettings() {
        _endpoint      = endpoint.value;
        _queueMessages = static_cast<std::size_t>(queue_messages.value);
        _queueBytes    = queue_bytes.value;
    }

    [[nodiscard]] property_map buildMetadata(const Packet<T>& packet) {
        property_map map;
        if (!packet.meta_information.empty()) {
            map = packet.meta_information[0UZ]; // copied key for key, nothing filtered and nothing consumed
        }
        nMetaKeysMistyped += detail::countMistypedKeys(map);

        if (map.find("sequence") == map.end()) {
            map.insert_or_assign(property_map::key_type("sequence"), pmt::Value(_sequence));
        } else {
            ++nSequenceDeclined; // the producer's own count is the less local one, so it stands
        }
        if (packet.timestamp != 0) {
            map.insert_or_assign(property_map::key_type(detail::sockio::kTimestampKey), pmt::Value(packet.timestamp));
            ++nTimestampsCarried;
        }
        if (packet.default_value != T()) {
            ++nDefaultValuesDropped; // the field is not reflected and the envelope has no place for it; counted, not silent
        }
        return map;
    }

    /// @brief The three envelope parts laid end to end, which is the one datagram this sink sends.
    [[nodiscard]] std::vector<std::uint8_t> assemble(const Packet<T>& packet, const std::string& metadata, std::uint64_t payloadBytes, std::uint64_t total) const {
        gr::network::EnvelopeHeader header;
        header.item_type     = gr::network::kItemTypeCode<T>;
        header.item_size     = static_cast<std::uint8_t>(sizeof(T));
        header.item_count    = static_cast<std::uint32_t>(packet.signal_values.size());
        header.payload_bytes = static_cast<std::uint32_t>(payloadBytes);
        header.meta_bytes    = static_cast<std::uint32_t>(metadata.size());

        std::vector<std::uint8_t> bytes(detail::sockio::narrowIndex<std::size_t>(total));
        const auto                encoded = gr::network::encodeHeader(header);
        std::ranges::copy(encoded, bytes.begin());
        if (!metadata.empty()) {
            std::memcpy(bytes.data() + gr::network::kHeaderBytesV1, metadata.data(), metadata.size());
        }
        const std::span<const std::byte> raw = std::as_bytes(std::span<const T>(packet.signal_values));
        if (!raw.empty()) {
            std::memcpy(bytes.data() + gr::network::kHeaderBytesV1 + metadata.size(), raw.data(), raw.size());
        }
        return bytes;
    }

    /// @brief Put a datagram on the send queue, applying `overflow` when it is full. False means "not consumed".
    ///
    /// `processBulk` refuses an envelope larger than `queue_bytes` before offering it, so what arrives here always
    /// fits and shedding always makes room for it: an empty queue holds nothing and the test below is then false.
    [[nodiscard]] bool enqueue(std::vector<std::uint8_t>&& envelope) {
        const std::uint64_t bytes = envelope.size();
        std::unique_lock    lock(_mutex);
        while (_queue.size() >= _queueMessages || _queuedBytes + bytes > _queueBytes) {
            if (_backpressure) {
                return false;
            }
            _queuedBytes -= _queue.front().size(); // the newest packets are what a live consumer wants
            _queue.pop_front();
            ++nDroppedOnOverflow;
        }
        _queuedBytes += bytes;
        _queue.push_back(std::move(envelope));
        lock.unlock();
        _cv.notify_one();
        return true;
    }

    void requestStopAndJoin() {
        {
            std::lock_guard lock(_mutex);
            _stopRequested = true;
        }
        _cv.notify_all();
        gr::atomic_ref(_ioThreadDone).wait(false);
    }

    void report() {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("packets sent", nPacketsSent);
        append("bytes sent", nBytesSent);
        append("rejected packets", nRejectedPackets);
        append("over queue bytes", nOverQueueBytes);
        append("dropped on overflow", nDroppedOnOverflow);
        append("backpressure stalls", nBackpressureStalls);
        append("sequence declined", nSequenceDeclined);
        append("metadata keys mistyped", nMetaKeysMistyped);
        append("timestamps carried", nTimestampsCarried);
        append("default values dropped", nDefaultValuesDropped);
        append("dropped at stop", nDroppedAtStop);
        append("send errors", nSendErrors);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::network::UdpPacketSink '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] bool stopRequested() {
        std::lock_guard lock(_mutex);
        return _stopRequested;
    }

    void ioSendLoop() {
        thread_pool::thread::setThreadName(std::format("udppktsink:{}", this->name.value));
        detail::sockio::Descriptor socket;
        std::string                failure;

        const auto addresses = detail::sockio::resolveEndpoint(_endpoint, SOCK_DGRAM, false);
        if (!addresses.has_value()) {
            failure = addresses.error();
        } else {
            failure = openSocket(socket, *addresses);
        }

        {
            std::lock_guard lock(_mutex);
            _openFailure = failure;
            _opened      = true;
        }
        _cv.notify_all();

        if (failure.empty()) {
            sendUntilStopped(socket);
        }
        gr::atomic_ref(_ioThreadDone).store_release(true);
        gr::atomic_ref(_ioThreadDone).notify_all();
    }

    /// @brief Open and connect the datagram socket, which fixes the destination for every later send.
    [[nodiscard]] std::string openSocket(detail::sockio::Descriptor& socket, std::span<const detail::sockio::Address> addresses) {
        std::string lastError = "no address was tried";
        for (const detail::sockio::Address& address : addresses) {
            detail::sockio::Descriptor candidate(::socket(address.family, SOCK_DGRAM, 0));
            if (!candidate.valid()) {
                lastError = detail::sockio::errorText(errno);
                continue;
            }
            if (::connect(candidate.get(), detail::sockio::addressOf(address), address.length) != 0) {
                lastError = detail::sockio::errorText(errno);
                continue;
            }
            std::ignore = detail::sockio::setNonBlocking(candidate.get());
            socket      = std::move(candidate);
            return {};
        }
        return std::format("cannot open a UDP socket for '{}': {}", _endpoint, lastError);
    }

    void sendUntilStopped(detail::sockio::Descriptor& socket) {
        using namespace std::chrono_literals;
        while (true) {
            std::vector<std::uint8_t> envelope;
            {
                std::unique_lock lock(_mutex);
                _cv.wait_for(lock, 100ms, [this] { return _stopRequested || !_queue.empty(); });
                if (_stopRequested) {
                    break;
                }
                if (_queue.empty()) {
                    continue;
                }
                envelope = std::move(_queue.front());
                _queue.pop_front();
                _queuedBytes -= envelope.size();
            }
            sendDatagram(socket, envelope);
        }
        // teardown is bounded and loses what is in flight, for the reason the stream sink's is
        std::lock_guard lock(_mutex);
        nDroppedAtStop += _queue.size();
        _queue.clear();
        _queuedBytes = 0ULL;
    }

    /// @brief Send one envelope as one datagram, waiting on the socket while the kernel's buffer is full.
    void sendDatagram(detail::sockio::Descriptor& socket, std::span<const std::uint8_t> envelope) {
        while (true) {
            const ::ssize_t written = ::send(socket.get(), envelope.data(), envelope.size(), MSG_NOSIGNAL);
            if (written >= 0) {
                std::lock_guard lock(_mutex);
                ++nPacketsSent;
                nBytesSent += envelope.size();
                return;
            }
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN) {
                const short revents = detail::sockio::pollFor(socket.get(), POLLOUT, detail::sockio::kPollMs);
                if ((revents & POLLOUT) != 0) {
                    continue;
                }
                if (stopRequested()) {
                    std::lock_guard lock(_mutex);
                    ++nDroppedAtStop;
                    return;
                }
                continue;
            }
            // an ICMP port-unreachable arrives here as ECONNREFUSED on the next send; it costs this datagram and
            // nothing else, because a datagram peer that is absent now may be present for the next one
            const int         code = errno;
            const std::string text = detail::sockio::errorText(code);
            std::lock_guard   lock(_mutex);
            ++nSendErrors;
            if (std::ranges::find(_loggedSendErrors, code) == _loggedSendErrors.end()) {
                _loggedSendErrors.push_back(code);
                std::println(stderr, "gr::blocks::network::UdpPacketSink '{}': send to '{}' failed: {}", this->name, _endpoint, text);
            }
            return;
        }
    }
};

GR_REGISTER_BLOCK(gr::blocks::network::UdpPacketSource, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
requires gr::network::EnvelopeItem<T>
struct UdpPacketSource : Block<UdpPacketSource<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Receives one envelope per UDP datagram and publishes each as one gr::Packet<T>.

The datagram boundary is the framing, so there is no seeking state and no buffer that outlives one read: a datagram
carries exactly one envelope or it carries a refusal. That is what makes a lost datagram a whole-envelope loss,
visible as a `sequence` gap, and it is why reordering and duplication reach the sequence tracker rather than a
reassembly buffer.

There is no `bind` setting, because there is no third arrangement: a datagram source binds its endpoint. Each
datagram is read with the flag that reports its true length even when the buffer could not hold it, so a datagram
larger than `max_message_bytes` is refused as `"over_max"` from a bounded copy rather than sizing an allocation from a
peer's claim. A datagram shorter than a header is `"short_header"`; one whose length disagrees with
`32 + meta_bytes + payload_bytes` is `"length_mismatch"`, which is the check a byte stream cannot make and a datagram
can. Every refusal here publishes a reject packet, because the datagram is fully in hand.

Validation happens on the block's own reader thread, so a flood of malformed datagrams costs that thread and never the
scheduler. `max_message_bytes` is required and has no default: it is the bound that decides how much of a datagram
this block is willing to hold.

The reader dies of its own socket and of nothing else, which is what makes `work::Status::ERROR` mean something: the
bound socket reporting `POLLERR`, `POLLHUP` or `POLLNVAL`, or a read failing for a reason that is the descriptor's
rather than one datagram's. A peer that cannot be reached, a datagram that arrives malformed and a port that answers
with an ICMP refusal are all counted and read past, because a datagram socket outlives any one datagram; a shortage of
descriptors or memory is retried a bounded number of times before it counts as permanent. `nReaderFailures` counts the
reader threads that ended on a fault and `lastReaderError()` names the last one.
)"">;

    PortOut<Packet<T>, Async>                      out;
    PortOut<Packet<std::uint8_t>, Async, Optional> reject;

    Annotated<std::string, "endpoint", Visible, Doc<"'host:port' to bind, e.g. 0.0.0.0:5555 or [::]:5555; required, there is no default">>  endpoint{};
    Annotated<std::uint64_t, "max_message_bytes", Unit<"byte">, Doc<"the largest datagram this reader will accept; required, must be > 0">> max_message_bytes   = 0ULL;
    Annotated<gr::Size_t, "queue_messages", Doc<"in-process receive queue depth">>                                                          queue_messages      = 1024U;
    Annotated<std::uint64_t, "queue_bytes", Unit<"byte">, Doc<"in-process receive queue size">>                                             queue_bytes         = 16777216ULL;
    Annotated<gr::Size_t, "max_reject_bytes", Unit<"byte">, Doc<"raw bytes of a refused datagram kept for inspection">>                     max_reject_bytes    = 256U;
    Annotated<gr::Size_t, "max_tracked_sources", Doc<"distinct source_id values whose sequence is tracked">>                                max_tracked_sources = 8U;

    GR_MAKE_REFLECTABLE(UdpPacketSource, out, reject, endpoint, max_message_bytes, queue_messages, queue_bytes, max_reject_bytes, max_tracked_sources);

    std::uint64_t nEnvelopesReceived = 0ULL; ///< datagrams taken off the socket
    std::uint64_t nPacketsPublished  = 0ULL; ///< packets published on out
    std::uint64_t nBytesReceived     = 0ULL; ///< datagram bytes taken off the socket, as the datagrams state them

    std::uint64_t nShortHeader         = 0ULL; ///< a datagram shorter than a header
    std::uint64_t nBadMagic            = 0ULL; ///< foreign traffic on the bound port
    std::uint64_t nRefusedVersion      = 0ULL; ///< wire_version 0 or above this reader's; one fault however it is spelled
    std::uint64_t nBadByteOrder        = 0ULL;
    std::uint64_t nBadHeaderBytes      = 0ULL;
    std::uint64_t nBadHeaderCrc        = 0ULL;
    std::uint64_t nUnsupportedItemType = 0ULL;
    std::uint64_t nBadItemSize         = 0ULL;
    std::uint64_t nBadPayloadLength    = 0ULL;
    std::uint64_t nUnknownMetaEncoding = 0ULL;
    std::uint64_t nUnknownFlags        = 0ULL;
    std::uint64_t nItemTypeMismatch    = 0ULL; ///< a well-formed envelope carrying a different item type
    std::uint64_t nLengthMismatch      = 0ULL; ///< the header's lengths disagree with the datagram's own
    std::uint64_t nOverMax             = 0ULL; ///< a datagram longer than max_message_bytes
    std::uint64_t nBadMetadata         = 0ULL; ///< the metadata bytes did not parse

    std::uint64_t nSequenceGaps          = 0ULL; ///< sequence discontinuities at a tracked source
    std::uint64_t nPacketsLost           = 0ULL; ///< the total size of those gaps
    std::uint64_t nSequenceResets        = 0ULL; ///< a sequence at or below the last seen from that source
    std::uint64_t nSourcesUntracked      = 0ULL; ///< a distinct source_id beyond max_tracked_sources
    std::uint64_t nDroppedByBackpressure = 0ULL; ///< datagrams discarded because the in-process queue was full
    std::uint64_t nOverQueueBytes        = 0ULL; ///< arrivals discarded because one alone exceeds queue_bytes
    std::uint64_t nMetaKeysMistyped      = 0ULL; ///< vocabulary keys whose type disagrees with the declaration
    std::uint64_t nTimestampsCarried     = 0ULL; ///< packet_timestamp values consumed into the carrier field
    std::uint64_t nRecvErrors            = 0ULL; ///< read failures other than would-block and interrupt
    std::uint64_t nReaderFailures        = 0ULL; ///< reader threads that ended on a fault of the socket they own

    /// @brief One decoded arrival, on whichever port it belongs.
    struct Incoming {
        Packet<T>            accepted{};
        Packet<std::uint8_t> refused{};
        bool                 isRefusal = false;
        std::uint64_t        bytes     = 0ULL;
    };

    std::mutex              _mutex;
    std::condition_variable _cv;
    std::deque<Incoming>    _queue;
    std::uint64_t           _queuedBytes   = 0ULL;
    bool                    _stopRequested = false;
    bool                    _opened        = false;
    bool                    _readerFailed  = false;
    std::string             _lastReaderError{}; ///< what ended the reader, empty while it runs and after a requested stop
    std::string             _openFailure{};
    bool                    _ioThreadDone = true;

    /// @brief The last `sequence` seen from a source, with the arrival ordinal that bounds the tracker by eviction.
    struct SourceState {
        std::uint64_t lastSequence = 0ULL;
        std::uint64_t lastSeen     = 0ULL;
    };
    std::vector<std::pair<std::string, SourceState>> _sources{};
    std::uint64_t                                    _arrivals = 0ULL;
    std::vector<std::uint16_t>                       _loggedVersions{};

    std::string   _endpoint{};
    std::size_t   _queueMessages     = 1024UZ;
    std::uint64_t _queueBytes        = 16777216ULL;
    std::uint64_t _maxMessageBytes   = 0ULL;
    std::size_t   _maxTrackedSources = 8UZ;
    std::size_t   _maxRejectBytes    = 256UZ;
    bool          _socketOpen        = false;
    std::size_t   _transientFaults   = 0UZ; ///< consecutive faults of a kind that may lift; the I/O thread's alone

    struct IoThreadGuard { // must be last member — destroyed first, so the reader is gone before the queue it fills
        UdpPacketSource* self;
        explicit IoThreadGuard(UdpPacketSource* owner) noexcept : self(owner) {}
        IoThreadGuard(const IoThreadGuard&)            = delete;
        IoThreadGuard(IoThreadGuard&&)                 = delete;
        IoThreadGuard& operator=(const IoThreadGuard&) = delete;
        IoThreadGuard& operator=(IoThreadGuard&&)      = delete;
        ~IoThreadGuard() { self->requestStopAndJoin(); }
    };
    IoThreadGuard _ioGuard{this};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() {
        validate();
        freezeSocketSettings();
        {
            std::lock_guard lock(_mutex);
            _stopRequested = false;
            _opened        = false;
            _readerFailed  = false;
            _lastReaderError.clear();
            _openFailure.clear();
            _queue.clear();
            _queuedBytes = 0ULL;
        }
        _sources.clear();
        _loggedVersions.clear();
        _arrivals        = 0ULL;
        _transientFaults = 0UZ;
        gr::atomic_ref(_ioThreadDone).store_release(false);
        thread_pool::Manager::defaultIoPool()->execute([this] { ioReadLoop(); });

        std::unique_lock lock(_mutex);
        _cv.wait(lock, [this] { return _opened; });
        if (!_openFailure.empty()) {
            const std::string failure = _openFailure;
            lock.unlock();
            gr::atomic_ref(_ioThreadDone).wait(false);
            throw gr::exception(failure);
        }
        _socketOpen = true;
    }

    void stop() {
        requestStopAndJoin();
        _socketOpen = false;
        report();
    }

    void rebuild() {
        validate();
        if (_socketOpen) {
            refuseFrozenChange();
        }
        _maxRejectBytes = static_cast<std::size_t>(max_reject_bytes.value);
    }

    /// @brief What ended the reader thread, or empty while it runs and after a stop it was asked for.
    ///
    /// Taken under the mutex the reader writes it behind, because the writer is another thread and a string read
    /// while it is being assigned is not a string.
    [[nodiscard]] std::string lastReaderError() {
        std::lock_guard lock(_mutex);
        return _lastReaderError;
    }

    [[nodiscard]] work::Status processBulk(OutputSpanLike auto& outSpan, OutputSpanLike auto& rejectSpan) {
        const bool  outConnected    = outSpan.isConnected;
        const bool  rejectConnected = rejectSpan.isConnected;
        std::size_t onOut           = 0UZ;
        std::size_t onReject        = 0UZ;
        bool        readerFailed    = false;
        bool        drained         = false;

        {
            std::lock_guard lock(_mutex);
            while (!_queue.empty()) {
                Incoming& front = _queue.front();
                if (front.isRefusal) {
                    if (rejectConnected) {
                        if (onReject >= rejectSpan.size()) {
                            break;
                        }
                        rejectSpan[onReject] = std::move(front.refused);
                        ++onReject;
                    }
                    // an unconnected reject port still drains: a refusal must not wedge the queue behind it
                } else {
                    if (outConnected) {
                        if (onOut >= outSpan.size()) {
                            break;
                        }
                        outSpan[onOut] = std::move(front.accepted);
                        ++onOut;
                    }
                    ++nPacketsPublished;
                }
                _queuedBytes -= front.bytes;
                _queue.pop_front();
            }
            readerFailed = _readerFailed;
            drained      = _queue.empty();
        }

        outSpan.publish(outConnected ? onOut : 0UZ);
        rejectSpan.publish(rejectConnected ? onReject : 0UZ);
        if (onOut == 0UZ && onReject == 0UZ) {
            return readerFailed && drained ? work::Status::ERROR : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    void validate() {
        if (endpoint.value.empty()) {
            throw gr::exception("endpoint is empty; a transport source has no default endpoint and will not bind to nothing");
        }
        if (max_message_bytes.value == 0ULL) {
            throw gr::exception("max_message_bytes is 0 or unset; a datagram states its own lengths, so a source must state a bound before a peer's claimed length sizes anything");
        }
        if (queue_messages.value == 0U) {
            throw gr::exception("queue_messages is 0; the in-process receive queue must hold at least one envelope");
        }
        if (queue_bytes.value == 0ULL) {
            throw gr::exception("queue_bytes is 0; the in-process receive queue must hold at least one envelope");
        }
        if (max_tracked_sources.value == 0U) {
            throw gr::exception("max_tracked_sources is 0; no source_id would be tracked and no loss would be detected");
        }
        if (static_cast<std::uint64_t>(max_reject_bytes.value) > max_message_bytes.value) {
            throw gr::exception(std::format("max_reject_bytes {} exceeds max_message_bytes {}; a refusal cannot keep more than a message may hold", max_reject_bytes.value, max_message_bytes.value));
        }
    }

    void refuseFrozenChange() const {
        const auto refuse = [](std::string_view name) { throw gr::exception(std::format("setting '{}' is read once when the socket opens and cannot change while the block is running; rebuild the graph instead", name)); };
        if (endpoint.value != _endpoint) {
            refuse("endpoint");
        }
        if (max_message_bytes.value != _maxMessageBytes) {
            refuse("max_message_bytes");
        }
        if (static_cast<std::size_t>(queue_messages.value) != _queueMessages) {
            refuse("queue_messages");
        }
        if (queue_bytes.value != _queueBytes) {
            refuse("queue_bytes");
        }
        if (static_cast<std::size_t>(max_tracked_sources.value) != _maxTrackedSources) {
            refuse("max_tracked_sources");
        }
    }

    void freezeSocketSettings() {
        _endpoint          = endpoint.value;
        _maxMessageBytes   = max_message_bytes.value;
        _queueMessages     = static_cast<std::size_t>(queue_messages.value);
        _queueBytes        = queue_bytes.value;
        _maxTrackedSources = static_cast<std::size_t>(max_tracked_sources.value);
        _maxRejectBytes    = static_cast<std::size_t>(max_reject_bytes.value);
    }

    void requestStopAndJoin() {
        {
            std::lock_guard lock(_mutex);
            _stopRequested = true;
        }
        _cv.notify_all();
        gr::atomic_ref(_ioThreadDone).wait(false);
    }

    void report() {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("envelopes received", nEnvelopesReceived);
        append("packets published", nPacketsPublished);
        append("bytes received", nBytesReceived);
        append("short header", nShortHeader);
        append("bad magic", nBadMagic);
        append("refused version", nRefusedVersion);
        append("bad byte order", nBadByteOrder);
        append("bad header bytes", nBadHeaderBytes);
        append("bad header crc", nBadHeaderCrc);
        append("unsupported item type", nUnsupportedItemType);
        append("bad item size", nBadItemSize);
        append("bad payload length", nBadPayloadLength);
        append("unknown meta encoding", nUnknownMetaEncoding);
        append("unknown flags", nUnknownFlags);
        append("item type mismatch", nItemTypeMismatch);
        append("length mismatch", nLengthMismatch);
        append("over max", nOverMax);
        append("bad metadata", nBadMetadata);
        append("sequence gaps", nSequenceGaps);
        append("packets lost", nPacketsLost);
        append("sequence resets", nSequenceResets);
        append("sources untracked", nSourcesUntracked);
        append("dropped by backpressure", nDroppedByBackpressure);
        append("over queue bytes", nOverQueueBytes);
        append("metadata keys mistyped", nMetaKeysMistyped);
        append("timestamps carried", nTimestampsCarried);
        append("receive errors", nRecvErrors);
        append("reader failures", nReaderFailures);
        if (!_lastReaderError.empty()) {
            std::format_to(std::back_inserter(report), "{}reader ended: {}", report.empty() ? "" : ", ", _lastReaderError);
        }
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::network::UdpPacketSource '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] bool stopRequested() {
        std::lock_guard lock(_mutex);
        return _stopRequested;
    }

    void waitBounded(std::uint32_t milliseconds) {
        std::unique_lock lock(_mutex);
        std::ignore = _cv.wait_for(lock, std::chrono::milliseconds(milliseconds), [this] { return _stopRequested; });
    }

    void ioReadLoop() {
        thread_pool::thread::setThreadName(std::format("udppktsrc:{}", this->name.value));
        detail::sockio::Descriptor socket;
        std::string                failure;

        const auto addresses = detail::sockio::resolveEndpoint(_endpoint, SOCK_DGRAM, true);
        if (!addresses.has_value()) {
            failure = addresses.error();
        } else {
            failure = openSocket(socket, *addresses);
        }

        {
            std::lock_guard lock(_mutex);
            _openFailure = failure;
            _opened      = true;
        }
        _cv.notify_all();

        std::string fatal;
        if (failure.empty()) {
            fatal = receiveUntilStopped(socket);
        }
        {
            // the reader is gone; if nobody asked it to go, the wire is not quiet but dead, and processBulk owes the
            // graph that distinction once the queue behind it has drained
            std::lock_guard lock(_mutex);
            if (!fatal.empty()) {
                _lastReaderError = fatal;
                ++nReaderFailures;
            }
            _readerFailed = !_stopRequested;
        }
        _cv.notify_all();
        gr::atomic_ref(_ioThreadDone).store_release(true);
        gr::atomic_ref(_ioThreadDone).notify_all();
    }

    /// @brief What a fault at the bound socket costs: the reason the reader ends, or empty to read on.
    ///
    /// A shortage of descriptors or memory is given @ref detail::sockio::kTransientFaultLimit consecutive attempts,
    /// spaced by one poll interval, before it is called permanent: a shortage that never lifts is as final as a broken
    /// descriptor, and a reader that hid it would spin against the same failure for the life of the graph.
    [[nodiscard]] std::string noteFault(int code, std::string_view what) {
        switch (detail::sockio::classifyFault(code)) {
        case detail::sockio::Fault::Retry: return {};
        case detail::sockio::Fault::Transient:
            ++_transientFaults;
            if (_transientFaults < detail::sockio::kTransientFaultLimit) {
                waitBounded(static_cast<std::uint32_t>(detail::sockio::kPollMs));
                return {};
            }
            return std::format("{} failed {} times running: {}", what, _transientFaults, detail::sockio::errorText(code));
        case detail::sockio::Fault::Fatal: break;
        }
        return std::format("{} failed: {}", what, detail::sockio::errorText(code));
    }

    [[nodiscard]] std::string openSocket(detail::sockio::Descriptor& socket, std::span<const detail::sockio::Address> addresses) {
        std::string lastError = "no address was tried";
        for (const detail::sockio::Address& address : addresses) {
            detail::sockio::Descriptor candidate(::socket(address.family, SOCK_DGRAM, 0));
            if (!candidate.valid()) {
                lastError = detail::sockio::errorText(errno);
                continue;
            }
            const int on = 1; // so a restarted graph can rebind its port immediately
            std::ignore  = ::setsockopt(candidate.get(), SOL_SOCKET, SO_REUSEADDR, &on, static_cast<::socklen_t>(sizeof(on)));
            if (::bind(candidate.get(), detail::sockio::addressOf(address), address.length) != 0) {
                lastError = detail::sockio::errorText(errno);
                continue;
            }
            std::ignore = detail::sockio::setNonBlocking(candidate.get());
            socket      = std::move(candidate);
            return {};
        }
        return std::format("cannot bind '{}': {}", _endpoint, lastError);
    }

    /// @brief Read datagrams until a stop is asked for, or name the fault of the bound socket that ends the reader.
    [[nodiscard]] std::string receiveUntilStopped(detail::sockio::Descriptor& socket) {
        // one read buffer for the run, sized by the bound the block states and capped by what a datagram can be; the
        // true length comes back from the kernel either way, so an oversize datagram is named rather than reassembled
        const std::size_t         capacity = detail::sockio::narrowIndex<std::size_t>(std::min<std::uint64_t>(65536ULL, _maxMessageBytes + 1ULL));
        std::vector<std::uint8_t> buffer(capacity);

        while (!stopRequested()) {
            const short revents = detail::sockio::pollFor(socket.get(), POLLIN, detail::sockio::kPollMs);
            if ((revents & detail::sockio::kPollFault) != 0) {
                return std::format("the bound socket reported {}", detail::sockio::pollFaultText(revents));
            }
            if ((revents & POLLIN) == 0) {
                continue; // the bounded wait expired, which is how the loop returns to observe a stop request
            }
            const ::ssize_t received = ::recv(socket.get(), buffer.data(), buffer.size(), MSG_TRUNC);
            if (received < 0) {
                if (detail::sockio::wouldBlock(errno)) {
                    continue;
                }
                ++nRecvErrors; // a datagram socket survives a fault on one datagram, so the reader stays alive
                if (std::string fatal = noteFault(errno, "receiving a datagram"); !fatal.empty()) {
                    return fatal;
                }
                continue;
            }
            _transientFaults           = 0UZ;
            const std::uint64_t length = static_cast<std::uint64_t>(received);
            ++nEnvelopesReceived;
            nBytesReceived += length;
            const std::size_t inHand = detail::sockio::narrowIndex<std::size_t>(std::min<std::uint64_t>(buffer.size(), length));
            handleDatagram(std::span<const std::uint8_t>(buffer).first(inHand), length);
        }
        return {};
    }

    /// @brief Validate one datagram and enqueue either the packet it carries or the refusal it earned.
    void handleDatagram(std::span<const std::uint8_t> bytes, std::uint64_t length) {
        if (length > _maxMessageBytes) {
            ++nOverMax; // the kept copy is bounded, so a peer's claim never sizes an allocation here
            enqueueRefusal(bytes, length, "over_max");
            return;
        }
        if (length < gr::network::kHeaderBytesV1) {
            ++nShortHeader;
            enqueueRefusal(bytes, length, "short_header");
            return;
        }

        const auto header = gr::network::decodeHeader(bytes.first(gr::network::kHeaderBytesV1));
        if (!header.has_value()) {
            countKernelRefusal(header.error(), bytes.first(gr::network::kHeaderBytesV1));
            enqueueRefusal(bytes, length, gr::network::discardReason(header.error()));
            return;
        }
        if (header->item_type != gr::network::kItemTypeCode<T>) {
            ++nItemTypeMismatch;
            enqueueRefusal(bytes, length, "item_type_mismatch");
            return;
        }
        const std::uint64_t stated = static_cast<std::uint64_t>(header->header_bytes) + static_cast<std::uint64_t>(header->meta_bytes) + static_cast<std::uint64_t>(header->payload_bytes);
        if (stated != length) {
            ++nLengthMismatch; // the header's lengths and the datagram's own are two statements about one message
            enqueueRefusal(bytes, length, "length_mismatch");
            return;
        }

        const std::span<const std::uint8_t> metadata = bytes.subspan(gr::network::kHeaderBytesV1, header->meta_bytes);
        const std::span<const std::uint8_t> payload  = bytes.subspan(std::size_t{gr::network::kHeaderBytesV1} + header->meta_bytes, header->payload_bytes);

        property_map map;
        if (header->meta_bytes != 0U) { // the empty map short-circuits rather than asking what an empty document means
            const std::string_view text(reinterpret_cast<const char*>(metadata.data()), metadata.size());
            const auto             parsed = pmt::yaml::deserialize(text);
            if (!parsed.has_value()) {
                ++nBadMetadata;
                std::println(stderr, "gr::blocks::network::UdpPacketSource '{}': metadata line {} column {}: {}", this->name, parsed.error().line, parsed.error().column, parsed.error().message);
                enqueueRefusal(bytes, length, "bad_metadata");
                return;
            }
            map = *parsed;
        }
        nMetaKeysMistyped += detail::countMistypedKeys(map);

        Incoming arrival;
        arrival.accepted.signal_values.resize(header->item_count);
        if (header->payload_bytes != 0U) {
            std::memcpy(arrival.accepted.signal_values.data(), payload.data(), header->payload_bytes);
        }
        if (const auto stamp = map.find(detail::sockio::kTimestampKey); stamp != map.end()) {
            if (const std::int64_t* value = stamp->second.get_if<std::int64_t>(); value != nullptr) {
                arrival.accepted.timestamp = *value;
                ++nTimestampsCarried;
            }
            map.erase(stamp->first); // consumed, so the carrier field is the value's only spelling after the crossing
        }
        trackSequence(map);
        arrival.accepted.meta_information.resize(1UZ);
        arrival.accepted.meta_information[0UZ] = std::move(map);
        arrival.bytes                          = length;
        enqueue(std::move(arrival));
    }

    void countKernelRefusal(gr::network::EnvelopeError error, std::span<const std::uint8_t> headerBytes) {
        using gr::network::EnvelopeError;
        switch (error) {
        case EnvelopeError::BadMagic: ++nBadMagic; return;
        case EnvelopeError::BadVersion:
        case EnvelopeError::FutureVersion: {
            ++nRefusedVersion; // a peer at the wrong version is one fault however it is spelled
            const std::uint16_t version = static_cast<std::uint16_t>(static_cast<std::uint16_t>(headerBytes[4UZ]) | static_cast<std::uint16_t>(static_cast<std::uint16_t>(headerBytes[5UZ]) << 8U));
            if (std::ranges::find(_loggedVersions, version) == _loggedVersions.end()) {
                _loggedVersions.push_back(version); // once per distinct value: a mismatched peer at rate would flood
                std::println(stderr, "gr::blocks::network::UdpPacketSource '{}': refusing wire version {} on '{}'; this reader implements version {}", this->name, version, _endpoint, gr::network::kWireVersion);
            }
            return;
        }
        case EnvelopeError::ByteOrder: ++nBadByteOrder; return;
        case EnvelopeError::BadHeaderBytes: ++nBadHeaderBytes; return;
        case EnvelopeError::BadHeaderCrc: ++nBadHeaderCrc; return;
        case EnvelopeError::UnsupportedItemType: ++nUnsupportedItemType; return;
        case EnvelopeError::BadItemSize: ++nBadItemSize; return;
        case EnvelopeError::PayloadLengthField: ++nBadPayloadLength; return;
        case EnvelopeError::UnknownMetaEncoding: ++nUnknownMetaEncoding; return;
        case EnvelopeError::UnknownFlags: ++nUnknownFlags; return;
        }
    }

    /// @brief Publish the raw bytes of a refused datagram so the failure can be read without a packet capture.
    ///
    /// `max_reject_bytes` bounds the copy, so a hostile datagram cannot be pulled whole into the graph by the very
    /// path that refused it. There is no frame count to state, because a datagram has no frames.
    void enqueueRefusal(std::span<const std::uint8_t> bytes, std::uint64_t total, std::string_view reason) {
        Incoming arrival;
        arrival.isRefusal = true;
        arrival.bytes     = total;

        std::vector<std::uint8_t>& kept     = arrival.refused.signal_values;
        const std::size_t          keptSize = std::min(bytes.size(), _maxRejectBytes);
        kept.assign(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(keptSize));

        arrival.refused.meta_information.resize(1UZ);
        property_map& map = arrival.refused.meta_information[0UZ];
        map.insert_or_assign(property_map::key_type("discard_reason"), pmt::Value(std::string(reason)));
        map.insert_or_assign(property_map::key_type("envelope_bytes_total"), pmt::Value(total));
        const std::uint64_t keptBytes = kept.size();
        map.insert_or_assign(property_map::key_type("envelope_bytes_kept"), pmt::Value(keptBytes));
        enqueue(std::move(arrival));
    }

    /// @brief Put one decoded arrival on the queue `processBulk` drains, shedding the oldest where it does not fit.
    ///
    /// An arrival larger than `queue_bytes` is discarded and counted rather than queued: shedding what is queued
    /// cannot make room for one, and queueing it anyway would put the queue above the bound the graph set. A source
    /// has no upstream to refuse, so the count is where the loss is stated, as it is for every other loss here.
    void enqueue(Incoming&& arrival) {
        {
            std::lock_guard lock(_mutex);
            if (arrival.bytes > _queueBytes) {
                ++nOverQueueBytes;
                return;
            }
            while (_queue.size() >= _queueMessages || _queuedBytes + arrival.bytes > _queueBytes) {
                _queuedBytes -= _queue.front().bytes;
                _queue.pop_front();
                ++nDroppedByBackpressure; // the one loss class this end counts exactly, which is what makes the
                                          // subtraction from nPacketsLost separate a slow graph from a lossy wire
            }
            _queuedBytes += arrival.bytes;
            _queue.push_back(std::move(arrival));
        }
        _cv.notify_one();
    }

    /// @brief Update the per-source sequence tracker and count what the gaps imply.
    void trackSequence(const property_map& map) {
        const auto sequenceEntry = map.find("sequence");
        if (sequenceEntry == map.end()) {
            return; // a hand-built peer may omit it; a peer built from these blocks cannot
        }
        const std::uint64_t* sequence = sequenceEntry->second.get_if<std::uint64_t>();
        if (sequence == nullptr) {
            return; // already counted as a mistyped vocabulary key
        }
        std::string sourceId;
        if (const auto entry = map.find("source_id"); entry != map.end()) {
            if (const std::pmr::string* value = entry->second.get_if<std::pmr::string>(); value != nullptr) {
                sourceId.assign(value->begin(), value->end());
            }
        }

        ++_arrivals;
        const auto found = std::ranges::find(_sources, sourceId, [](const auto& entry) { return entry.first; });
        if (found != _sources.end()) {
            SourceState& state = found->second;
            if (*sequence > state.lastSequence + 1ULL) {
                ++nSequenceGaps;
                nPacketsLost += *sequence - state.lastSequence - 1ULL;
            } else if (*sequence <= state.lastSequence) {
                ++nSequenceResets; // a producer restarted, a datagram was duplicated, or two producers share the id
            }
            state.lastSequence = *sequence;
            state.lastSeen     = _arrivals;
            return;
        }

        // the key comes off the wire, so the number of them is bounded before it sizes anything
        if (_sources.size() >= _maxTrackedSources) {
            const auto stalest = std::ranges::min_element(_sources, {}, [](const auto& entry) { return entry.second.lastSeen; });
            _sources.erase(stalest);
            ++nSourcesUntracked;
        }
        _sources.emplace_back(std::move(sourceId), SourceState{.lastSequence = *sequence, .lastSeen = _arrivals});
        // the first envelope from a source establishes the baseline and is never a gap: a reader has no history
    }
};

} // namespace gr::blocks::network

#endif // GNURADIO_SOCKETPACKETIO_HPP
