#ifndef GNURADIO_NETWORK_TCPBYTEIO_HPP
#define GNURADIO_NETWORK_TCPBYTEIO_HPP

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <expected>
#include <format>
#include <mutex>
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
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>
#include <gnuradio-4.0/thread/thread_pool.hpp>

/**
 * @brief A raw byte stream over TCP, no envelope, no framing and no interpretation of any kind.
 *
 * `TcpByteSink` and `TcpByteSource` are the transport half of a legacy interchange format such as KISS: every byte
 * that arrives on `in` leaves the socket unchanged, and every byte the socket delivers leaves `out` unchanged. The
 * connection contract matches the tree's other TCP/UDP transports: `endpoint` as `"host:port"` resolved through
 * `getaddrinfo` at `start()`; `bind` with a sink listening and a source connecting by default, either reversible;
 * one peer at a time with a second accepted and immediately closed; `reconnect_ms` default `100`; `TCP_NODELAY` and
 * `SO_REUSEADDR`; one dedicated I/O thread owning the socket end to end with the last-declared teardown guard;
 * `processBulk` touching only a bounded in-process queue; every socket wait bounded at 100 ms so `stop()` is
 * honored promptly; `ERROR` from `processBulk` on a dead reader with a drained queue.
 *
 * What this pair deletes from that contract is the whole of the envelope: there is no header, no metadata frame, no
 * `sequence`, no `discard_reason`, no reject port, no `max_message_bytes` and no `item_type` check, because there is
 * nothing to check. `queue_bytes` is the one bound this pair keeps, on the same rule the envelope pair's queue
 * settings follow — the peer's send rate is not the graph's to choose, so the buffer's size is a required setting
 * with no default.
 *
 * The low-level socket code below sits in a namespace of its own so this header carries no dependency on the
 * envelope kernel: a byte transport has no business including a header format it does not speak.
 */
namespace gr::blocks::network {

namespace detail::bytesockio {

/// @brief The bound every socket wait in this file carries, so a stop request is observed promptly.
inline constexpr int kPollMs = 100;

/// @brief The poll flags that say the descriptor itself is finished rather than merely idle.
inline constexpr short kPollFault = static_cast<short>(POLLERR | POLLHUP | POLLNVAL);

/// @brief How many consecutive transient faults an I/O thread absorbs before it treats the shortage as permanent.
inline constexpr std::size_t kTransientFaultLimit = 8UZ;

/// @brief The shortest interval between two resolutions of one endpoint, so a retry loop is not a resolver flood.
inline constexpr std::uint32_t kResolveIntervalMs = 1000U;

/// @brief The system's own wording for an `errno` value, taken through the thread-safe accessor.
[[nodiscard]] inline std::string errorText(int code) { return std::system_category().message(code); }

/// @brief What an I/O thread does about a fault its own socket reported.
enum class Fault : std::uint8_t {
    Retry,     ///< nothing to do just now, or something the far end did; the loop carries on unchanged
    Transient, ///< a shortage that may pass; the loop waits and tries again, a bounded number of times
    Fatal      ///< the descriptor cannot serve again, and there is nothing left for the thread to do
};

/// @brief How a fault at a socket the I/O thread owns is judged.
///
/// A would-block and an interrupt are not faults at all. Neither is anything the far end or the route between can
/// cause: `accept` hands back the pending errors of the connection it would have returned, and a transport that died
/// of one would die of a peer misbehaving, which is the opposite of what a reconnecting transport is for. A shortage
/// of descriptors or of kernel memory is real but may lift, so it is retried; the caller bounds how often. Everything
/// else is the descriptor itself, and a listening socket that has broken does not mend.
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
[[nodiscard]] inline bool wouldBlock(int code) noexcept { return code == EAGAIN || code == EINTR; }

/// @brief One block's counters read as a set, so an observer outside the I/O thread sees them as of one instant.
///
/// The counters themselves are plain members the I/O thread increments behind the block's mutex; reading them one
/// by one from another thread would be a race, so the block hands out a copy taken under that same mutex.
struct Counters {
    std::uint64_t bytes        = 0ULL; ///< bytes that crossed the socket whole
    std::uint64_t bytesDropped = 0ULL; ///< queued bytes discarded rather than crossing
    std::uint64_t socketErrors = 0ULL; ///< send or receive attempts that could not complete
    std::uint64_t peersRefused = 0ULL; ///< connections accepted and closed because one peer was already served
    std::uint64_t disconnects  = 0ULL; ///< connections lost
    std::uint64_t reconnects   = 0ULL; ///< connections established after the first
};

} // namespace detail::bytesockio

GR_REGISTER_BLOCK(gr::blocks::network::TcpByteSink)

struct TcpByteSink : Block<TcpByteSink, NoTagPropagation> {
    using Description = Doc<R""(
@brief Writes every incoming byte to a TCP peer, untouched: no header, no framing, no interpretation.

A listening sink serves one peer at a time. A second connection is accepted and immediately closed, counted in
`nPeersRefused`. A connecting sink retries forever at `reconnect_ms`; `start()` succeeds whether or not the peer is
up yet and only a socket that cannot be created or an endpoint that cannot be resolved is fatal.

While no peer is connected nothing leaves the queue, so it fills and `overflow` decides what happens next:
`drop_oldest` discards the stalest bytes and counts them in `nBytesDropped`, `backpressure` instead consumes fewer
input items, so the stall propagates upstream by the framework's own path. A byte lost to a peer that disappeared
mid-write is counted in `nBytesDropped` too, and the write attempt itself in `nSendErrors` — the two answer different
questions, how much was lost and how many faults caused it.

A sink's peer is not expected to speak. Whatever it sends is read off the socket and discarded, up to 256 bytes each
time the connection is checked, because a byte a raw transport cannot interpret is a byte it must not keep: leaving it
unread would eventually stall the peer behind a full receive buffer.

At teardown the queue is offered to the peer for one bounded write; whatever the bound leaves behind is counted in
`nBytesDropped`, since a drain against a stalled peer is unbounded and an unbounded teardown is a graph that will not
end.

The socket is owned end to end by a dedicated I/O thread; `processBulk` only enqueues.
)"">;

    PortIn<std::uint8_t> in;

    Annotated<std::string, "endpoint", Visible, Doc<"'host:port', e.g. 127.0.0.1:5555 or [::1]:5555; required, there is no default">> endpoint{};
    Annotated<bool, "bind", Doc<"listen on the endpoint rather than connect to it; a sink normally listens">>                         bind         = true;
    Annotated<std::string, "overflow", Doc<"'drop_oldest' or 'backpressure', applied when the in-process byte queue is full">>        overflow     = std::string("drop_oldest");
    Annotated<gr::Size_t, "queue_bytes", Unit<"byte">, Doc<"the in-process byte queue's bound; required, there is no default">>       queue_bytes  = 0U;
    Annotated<gr::Size_t, "reconnect_ms", Unit<"ms">, Doc<"interval between connection attempts while connecting">>                   reconnect_ms = 100U;

    GR_MAKE_REFLECTABLE(TcpByteSink, in, endpoint, bind, overflow, queue_bytes, reconnect_ms);

    // Counted, stated drops and refusals. Plain members, printed once by stop(); nothing here is on the sample path.
    std::uint64_t nBytes        = 0ULL; ///< bytes written whole to a peer
    std::uint64_t nBytesDropped = 0ULL; ///< queued bytes discarded: on overflow, lost to a peer that disappeared, or left over after the bounded teardown flush
    std::uint64_t nSendErrors   = 0ULL; ///< write attempts that could not complete
    std::uint64_t nPeersRefused = 0ULL; ///< connections accepted and closed because one peer was already served
    std::uint64_t nDisconnects  = 0ULL; ///< connections lost
    std::uint64_t nReconnects   = 0ULL; ///< connections established after the first

    std::mutex               _mutex;
    std::condition_variable  _cv;
    std::deque<std::uint8_t> _queue;
    bool                     _stopRequested = false;
    bool                     _opened        = false;
    std::string              _openFailure{};
    bool                     _ioThreadDone = true; ///< true until start() launches the I/O thread

    std::string   _endpoint{}; ///< the socket settings, frozen for the duration of one run
    bool          _bind          = true;
    std::size_t   _queueBytes    = 0UZ;
    std::uint32_t _reconnectMs   = 100U;
    bool          _backpressure  = false;
    bool          _socketOpen    = false;
    bool          _everConnected = false;
    std::size_t   _addressCursor = 0UZ; ///< which resolved address the next connection attempt takes; the I/O thread's alone

    /// @brief Joins the I/O thread however the block dies.
    ///
    /// Must be the last declared member, so it is destroyed first and the thread is gone before the queue, mutex and
    /// condition variable it uses. `stop()` cannot be relied on: the scheduler does not call it when a graph ends in
    /// ERROR, and `~Block()` cannot stand in because derived members are destroyed before it runs.
    struct IoThreadGuard {
        TcpByteSink* self;
        explicit IoThreadGuard(TcpByteSink* owner) noexcept : self(owner) {}
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
            refuseFrozenChange(); // every setting is read once when the socket opens, so nothing is recomputed here
            return;
        }
        _backpressure = overflow.value == "backpressure";
    }

    /// @brief The counters as one set, taken under the lock the I/O thread writes them behind.
    [[nodiscard]] detail::bytesockio::Counters counters() {
        std::lock_guard lock(_mutex);
        return {.bytes = nBytes, .bytesDropped = nBytesDropped, .socketErrors = nSendErrors, .peersRefused = nPeersRefused, .disconnects = nDisconnects, .reconnects = nReconnects};
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan) {
        std::size_t consumed = 0UZ;
        if (inSpan.size() > 0UZ) {
            std::unique_lock lock(_mutex);
            for (; consumed < inSpan.size(); ++consumed) {
                if (_queue.size() >= _queueBytes) {
                    if (_backpressure) {
                        break; // consume fewer items; the stall propagates upstream by the framework's own path
                    }
                    _queue.pop_front(); // the newest bytes are what a live consumer wants
                    ++nBytesDropped;
                }
                _queue.push_back(inSpan[consumed]);
            }
            lock.unlock();
            if (consumed > 0UZ) {
                _cv.notify_one();
            }
        }
        std::ignore = inSpan.consume(consumed);
        if (consumed == 0UZ) {
            // nothing offered is an idle input; something offered and nothing taken is the full queue refusing it,
            // which is where `backpressure` propagates the stall
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
            throw gr::exception(std::format("overflow is '{}'; it must be 'drop_oldest' (shed the stalest bytes, count them) or 'backpressure' (consume fewer input items)", overflow.value));
        }
        if (queue_bytes.value == 0U) {
            throw gr::exception("queue_bytes is 0 or unset; the in-process byte queue must hold at least one byte, and the peer's send rate is not the graph's to choose without a stated bound");
        }
        if (reconnect_ms.value == 0U) {
            throw gr::exception("reconnect_ms is 0; a connecting sink would retry without pause and spend the thread on nothing else");
        }
    }

    /// @brief Refuse a change to a setting the running socket was built from, naming it.
    void refuseFrozenChange() const {
        const auto refuse = [](std::string_view setting) { throw gr::exception(std::format("setting '{}' is read once when the socket opens and cannot change while the block is running; rebuild the graph instead", setting)); };
        if (endpoint.value != _endpoint) {
            refuse("endpoint");
        }
        if (bind.value != _bind) {
            refuse("bind");
        }
        if ((overflow.value == "backpressure") != _backpressure) {
            refuse("overflow");
        }
        if (static_cast<std::size_t>(queue_bytes.value) != _queueBytes) {
            refuse("queue_bytes");
        }
        if (reconnect_ms.value != _reconnectMs) {
            refuse("reconnect_ms");
        }
    }

    void freezeSocketSettings() {
        _endpoint      = endpoint.value;
        _bind          = bind.value;
        _backpressure  = overflow.value == "backpressure";
        _queueBytes    = static_cast<std::size_t>(queue_bytes.value);
        _reconnectMs   = reconnect_ms.value;
        _addressCursor = 0UZ;
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
        append("bytes sent", nBytes);
        append("bytes dropped", nBytesDropped);
        append("send errors", nSendErrors);
        append("peers refused", nPeersRefused);
        append("disconnects", nDisconnects);
        append("reconnects", nReconnects);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::network::TcpByteSink '{}': {}", this->name, report);
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

    void ioSendLoop() {
        thread_pool::thread::setThreadName(std::format("tcpbytesink:{}", this->name.value));
        detail::bytesockio::Descriptor listener;
        std::string                    failure;

        const auto addresses = detail::bytesockio::resolveEndpoint(_endpoint, SOCK_STREAM, _bind);
        if (!addresses.has_value()) {
            failure = addresses.error();
        } else if (_bind) {
            failure = openListener(listener, *addresses);
        } else {
            const detail::bytesockio::Descriptor probe(::socket(addresses->front().family, SOCK_STREAM, 0));
            if (!probe.valid()) {
                failure = std::format("cannot create a TCP socket for '{}': {}", _endpoint, detail::bytesockio::errorText(errno));
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

    [[nodiscard]] std::string openListener(detail::bytesockio::Descriptor& listener, std::span<const detail::bytesockio::Address> addresses) {
        std::string lastError = "no address was tried";
        for (const detail::bytesockio::Address& address : addresses) {
            detail::bytesockio::Descriptor candidate(::socket(address.family, SOCK_STREAM, 0));
            if (!candidate.valid()) {
                lastError = detail::bytesockio::errorText(errno);
                continue;
            }
            const int on = 1; // so a restarted graph can rebind its port inside TIME_WAIT
            std::ignore  = ::setsockopt(candidate.get(), SOL_SOCKET, SO_REUSEADDR, &on, static_cast<::socklen_t>(sizeof(on)));
            if (::bind(candidate.get(), detail::bytesockio::addressOf(address), address.length) != 0) {
                lastError = detail::bytesockio::errorText(errno);
                continue;
            }
            if (::listen(candidate.get(), 8) != 0) {
                lastError = detail::bytesockio::errorText(errno);
                continue;
            }
            std::ignore = detail::bytesockio::setNonBlocking(candidate.get());
            listener    = std::move(candidate);
            return {};
        }
        return std::format("cannot listen on '{}': {}", _endpoint, lastError);
    }

    void sendUntilStopped(detail::bytesockio::Descriptor& listener, std::span<const detail::bytesockio::Address> addresses) {
        detail::bytesockio::Descriptor peer;
        std::vector<std::uint8_t>      chunk;

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

            chunk.clear();
            {
                std::unique_lock lock(_mutex);
                _cv.wait_for(lock, std::chrono::milliseconds(detail::bytesockio::kPollMs), [this] { return _stopRequested || !_queue.empty(); });
                if (_stopRequested) {
                    break;
                }
                if (_queue.empty()) {
                    continue;
                }
                chunk.assign(_queue.begin(), _queue.end());
                _queue.clear();
            }

            const std::size_t sent = writeSome(peer, chunk, [this] { return stopRequested(); });
            {
                std::lock_guard lock(_mutex);
                nBytes += sent;
            }
            if (sent == chunk.size()) {
                continue;
            }
            {
                std::lock_guard lock(_mutex);
                nBytesDropped += chunk.size() - sent;
            }
            if (stopRequested()) {
                break;
            }
            // the peer went away mid-write; the unsent remainder is counted above and not retried, since a retry
            // would resend into a connection that no longer exists
            {
                std::lock_guard lock(_mutex);
                ++nSendErrors;
            }
            losePeer(peer);
        }

        flushAtTeardown(peer); // one exit, so the queue meets the same bounded offer however the loop ends
    }

    /// @brief Offer what is still queued to the peer for one poll interval, and count whatever that bound leaves.
    ///
    /// A drain against a stalled peer is unbounded, and an unbounded teardown is a graph that will not end, so the
    /// residue is a drop like any other rather than a wait nobody can predict the length of.
    void flushAtTeardown(detail::bytesockio::Descriptor& peer) {
        std::vector<std::uint8_t> residue;
        {
            std::lock_guard lock(_mutex);
            residue.assign(_queue.begin(), _queue.end());
            _queue.clear();
        }
        if (residue.empty()) {
            return;
        }
        const auto        deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(detail::bytesockio::kPollMs);
        const std::size_t sent     = peer.valid() ? writeSome(peer, residue, [deadline] { return std::chrono::steady_clock::now() >= deadline; }) : 0UZ;
        std::lock_guard   lock(_mutex);
        nBytes += sent;
        nBytesDropped += residue.size() - sent;
    }

    void acquirePeer(detail::bytesockio::Descriptor& listener, detail::bytesockio::Descriptor& peer, std::span<const detail::bytesockio::Address> addresses) {
        if (_bind) {
            const short revents = detail::bytesockio::pollFor(listener.get(), POLLIN, detail::bytesockio::kPollMs);
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
        const detail::bytesockio::Address& address = addresses[_addressCursor % addresses.size()];
        ++_addressCursor;

        detail::bytesockio::Descriptor candidate(::socket(address.family, SOCK_STREAM, 0));
        if (!candidate.valid()) {
            waitBounded(_reconnectMs);
            return;
        }
        std::ignore = detail::bytesockio::setNonBlocking(candidate.get());
        if (::connect(candidate.get(), detail::bytesockio::addressOf(address), address.length) != 0) {
            if (errno != EINPROGRESS) {
                waitBounded(_reconnectMs);
                return;
            }
            const short revents = detail::bytesockio::pollFor(candidate.get(), POLLOUT, detail::bytesockio::kPollMs);
            int         pending = 0;
            ::socklen_t length  = static_cast<::socklen_t>(sizeof(pending));
            if ((revents & POLLOUT) == 0 || ::getsockopt(candidate.get(), SOL_SOCKET, SO_ERROR, &pending, &length) != 0 || pending != 0) {
                waitBounded(_reconnectMs);
                return;
            }
        }
        adoptPeer(peer, candidate.release());
    }

    void adoptPeer(detail::bytesockio::Descriptor& peer, int fd) {
        const int on = 1; // a byte is written as soon as it is queued; delaying it to coalesce defeats the transport
        std::ignore  = ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, static_cast<::socklen_t>(sizeof(on)));
        std::ignore  = detail::bytesockio::setNonBlocking(fd);
        peer.reset(fd);
        std::lock_guard lock(_mutex);
        if (_everConnected) {
            ++nReconnects;
        }
        _everConnected = true;
    }

    void losePeer(detail::bytesockio::Descriptor& peer) {
        peer.reset();
        std::lock_guard lock(_mutex);
        ++nDisconnects;
    }

    /// @brief Accept and close every extra connection, so the client that made one is told rather than left waiting.
    void refuseExtraPeers(detail::bytesockio::Descriptor& listener) {
        if (!listener.valid()) {
            return;
        }
        while ((detail::bytesockio::pollFor(listener.get(), POLLIN, 0) & POLLIN) != 0) {
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
    [[nodiscard]] bool peerAlive(detail::bytesockio::Descriptor& peer) {
        const short revents = detail::bytesockio::pollFor(peer.get(), POLLIN, 0);
        if ((revents & (POLLHUP | POLLERR)) != 0) {
            return false;
        }
        if ((revents & POLLIN) != 0) {
            std::array<std::uint8_t, 256UZ> scratch{};
            const ::ssize_t                 received = ::recv(peer.get(), scratch.data(), scratch.size(), MSG_DONTWAIT);
            if (received == 0) {
                return false; // an orderly close from the far end
            }
            if (received < 0 && !detail::bytesockio::wouldBlock(errno)) {
                return false;
            }
            // a sink's peer is not expected to speak, and whatever it said is not this block's to interpret
        }
        return true;
    }

    /// @brief Write as many of @p bytes as the peer accepts, until it faults or @p giveUp says to stop waiting.
    ///
    /// The predicate is consulted only where the socket has made the caller wait, so a peer that keeps taking bytes
    /// is never interrupted mid-chunk: the send loop asks a stop request on the way out, and teardown asks a deadline.
    template<typename TGiveUp>
    [[nodiscard]] std::size_t writeSome(detail::bytesockio::Descriptor& peer, std::span<const std::uint8_t> bytes, TGiveUp giveUp) {
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
                const short revents = detail::bytesockio::pollFor(peer.get(), POLLOUT, detail::bytesockio::kPollMs);
                if ((revents & (POLLHUP | POLLERR)) != 0 || giveUp()) {
                    return sent;
                }
                continue;
            }
            return sent;
        }
        return sent;
    }
};

GR_REGISTER_BLOCK(gr::blocks::network::TcpByteSource)

struct TcpByteSource : Block<TcpByteSource, NoTagPropagation> {
    using Description = Doc<R""(
@brief Reads a raw TCP byte stream and publishes it on `out`, untouched: no header, no framing, no interpretation.

A listening source serves one peer at a time, exactly as the sink does; a connecting source retries forever at
`reconnect_ms`. `overflow` governs the in-process receive queue the same way it does at the sink: `drop_oldest`
discards the stalest bytes already queued, `backpressure` instead stops reading the socket once the queue is full,
which is the one place a byte transport can propagate a stall onto the wire itself — TCP's own flow control then
slows the peer. Either way a single read that arrives larger than the room left can still push the queue over
`queue_bytes` by the size of that one read; the byte-stream contract has no frame to hold a read to, so `overflow`
is a bound the queue is kept near, not one no single read may ever cross.

A connection that closes is counted in `nDisconnects`; the source resumes on the next connection or reconnect
without inventing anything for what was lost, because a byte stream carries no boundary that would let it say what
that was. `processBulk` only drains the queue, and reports `work::Status::ERROR` only once the reader thread has
died and the queue is empty — a live, merely quiet connection is `INSUFFICIENT_INPUT_ITEMS` instead, since silence is
not death.

The reader dies of its own socket and of nothing else. Four faults end it: the listening socket reporting `POLLERR`,
`POLLHUP` or `POLLNVAL`; an `accept` that fails for anything but a would-block, an interrupt, an aborted connection or
a passing shortage of descriptors or memory; a socket the connecting role cannot create for such a reason; and an
endpoint that has stopped resolving, which the connecting role asks again once a second while it has no peer, so a
peer that moves is followed and a name that is gone is fatal. A shortage retries a bounded number of times before it
counts as permanent. What a peer does — closing, resetting, refusing a connection — costs the connection and
nothing more, and the reconnect loop carries on. `nReaderFailures` counts the reader threads that ended this way and
`lastReaderError()` names the last fault.
)"">;

    PortOut<std::uint8_t> out;

    Annotated<std::string, "endpoint", Visible, Doc<"'host:port', e.g. 127.0.0.1:5555 or [::1]:5555; required, there is no default">> endpoint{};
    Annotated<bool, "bind", Doc<"listen on the endpoint rather than connect to it; a source normally connects">>                      bind         = false;
    Annotated<std::string, "overflow", Doc<"'drop_oldest' or 'backpressure', applied when the in-process byte queue is full">>        overflow     = std::string("drop_oldest");
    Annotated<gr::Size_t, "queue_bytes", Unit<"byte">, Doc<"the in-process byte queue's bound; required, there is no default">>       queue_bytes  = 0U;
    Annotated<gr::Size_t, "reconnect_ms", Unit<"ms">, Doc<"interval between connection attempts while connecting">>                   reconnect_ms = 100U;

    GR_MAKE_REFLECTABLE(TcpByteSource, out, endpoint, bind, overflow, queue_bytes, reconnect_ms);

    std::uint64_t nBytes          = 0ULL; ///< bytes taken off the socket
    std::uint64_t nBytesDropped   = 0ULL; ///< queued bytes discarded on overflow
    std::uint64_t nRecvErrors     = 0ULL; ///< read failures other than would-block and interrupt
    std::uint64_t nPeersRefused   = 0ULL; ///< connections accepted and closed because one peer was already served
    std::uint64_t nDisconnects    = 0ULL; ///< connections lost
    std::uint64_t nReconnects     = 0ULL; ///< connections established after the first
    std::uint64_t nReaderFailures = 0ULL; ///< reader threads that ended on a fault of the socket they own

    std::mutex               _mutex;
    std::condition_variable  _cv;
    std::deque<std::uint8_t> _queue;
    bool                     _stopRequested = false;
    bool                     _opened        = false;
    bool                     _readerFailed  = false; ///< set if the reader thread ever exits other than by request
    std::string              _lastReaderError{};     ///< what ended the reader, empty while it runs and after a requested stop
    std::string              _openFailure{};
    bool                     _ioThreadDone = true;

    std::string   _endpoint{};
    bool          _bind            = false;
    std::size_t   _queueBytes      = 0UZ;
    std::uint32_t _reconnectMs     = 100U;
    bool          _backpressure    = false;
    bool          _socketOpen      = false;
    bool          _everConnected   = false;
    std::size_t   _addressCursor   = 0UZ; ///< which resolved address the next connection attempt takes; the I/O thread's alone
    std::size_t   _transientFaults = 0UZ; ///< consecutive faults of a kind that may lift; the I/O thread's alone

    std::chrono::steady_clock::time_point _resolvedAt{}; ///< when the endpoint was last resolved; the I/O thread's alone

    struct IoThreadGuard { // must be last member — destroyed first, so the reader is gone before the queue it fills
        TcpByteSource* self;
        explicit IoThreadGuard(TcpByteSource* owner) noexcept : self(owner) {}
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
        }
        _everConnected   = false;
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
            refuseFrozenChange(); // every setting is read once when the socket opens, so nothing is recomputed here
            return;
        }
        _backpressure = overflow.value == "backpressure";
    }

    /// @brief The counters as one set, taken under the lock the I/O thread writes them behind.
    [[nodiscard]] detail::bytesockio::Counters counters() {
        std::lock_guard lock(_mutex);
        return {.bytes = nBytes, .bytesDropped = nBytesDropped, .socketErrors = nRecvErrors, .peersRefused = nPeersRefused, .disconnects = nDisconnects, .reconnects = nReconnects};
    }

    /// @brief What ended the reader thread, or empty while it runs and after a stop it was asked for.
    ///
    /// Taken under the mutex the reader writes it behind, for the reason the counters are: the writer is another
    /// thread, and a string read while it is being assigned is not a string.
    [[nodiscard]] std::string lastReaderError() {
        std::lock_guard lock(_mutex);
        return _lastReaderError;
    }

    [[nodiscard]] work::Status processBulk(OutputSpanLike auto& outSpan) {
        std::size_t made         = 0UZ;
        bool        readerFailed = false;
        bool        drained      = false;
        {
            std::lock_guard   lock(_mutex);
            const std::size_t n = std::min(outSpan.size(), _queue.size());
            for (std::size_t i = 0UZ; i < n; ++i) {
                outSpan[i] = _queue.front();
                _queue.pop_front();
            }
            made         = n;
            readerFailed = _readerFailed;
            drained      = _queue.empty();
        }
        outSpan.publish(made);
        if (made == 0UZ) {
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
        if (overflow.value != "drop_oldest" && overflow.value != "backpressure") {
            throw gr::exception(std::format("overflow is '{}'; it must be 'drop_oldest' (shed the stalest bytes, count them) or 'backpressure' (stop reading the socket until there is room)", overflow.value));
        }
        if (queue_bytes.value == 0U) {
            throw gr::exception("queue_bytes is 0 or unset; a byte stream states no lengths of its own, so a source must state a queue bound before a peer's send rate sizes anything");
        }
        if (reconnect_ms.value == 0U) {
            throw gr::exception("reconnect_ms is 0; a connecting source would retry without pause and spend the thread on nothing else");
        }
    }

    void refuseFrozenChange() const {
        const auto refuse = [](std::string_view setting) { throw gr::exception(std::format("setting '{}' is read once when the socket opens and cannot change while the block is running; rebuild the graph instead", setting)); };
        if (endpoint.value != _endpoint) {
            refuse("endpoint");
        }
        if (bind.value != _bind) {
            refuse("bind");
        }
        if ((overflow.value == "backpressure") != _backpressure) {
            refuse("overflow");
        }
        if (static_cast<std::size_t>(queue_bytes.value) != _queueBytes) {
            refuse("queue_bytes");
        }
        if (reconnect_ms.value != _reconnectMs) {
            refuse("reconnect_ms");
        }
    }

    void freezeSocketSettings() {
        _endpoint      = endpoint.value;
        _bind          = bind.value;
        _backpressure  = overflow.value == "backpressure";
        _queueBytes    = static_cast<std::size_t>(queue_bytes.value);
        _reconnectMs   = reconnect_ms.value;
        _addressCursor = 0UZ;
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
        append("bytes received", nBytes);
        append("bytes dropped", nBytesDropped);
        append("receive errors", nRecvErrors);
        append("peers refused", nPeersRefused);
        append("disconnects", nDisconnects);
        append("reconnects", nReconnects);
        append("reader failures", nReaderFailures);
        if (!_lastReaderError.empty()) {
            std::format_to(std::back_inserter(report), "{}reader ended: {}", report.empty() ? "" : ", ", _lastReaderError);
        }
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::network::TcpByteSource '{}': {}", this->name, report);
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
        thread_pool::thread::setThreadName(std::format("tcpbytesrc:{}", this->name.value));
        detail::bytesockio::Descriptor           listener;
        std::string                              failure;
        std::vector<detail::bytesockio::Address> addresses;

        auto resolved = detail::bytesockio::resolveEndpoint(_endpoint, SOCK_STREAM, _bind);
        if (!resolved.has_value()) {
            failure = resolved.error();
        } else {
            addresses   = std::move(*resolved);
            _resolvedAt = std::chrono::steady_clock::now();
            if (_bind) {
                failure = openListener(listener, addresses);
            } else {
                const detail::bytesockio::Descriptor probe(::socket(addresses.front().family, SOCK_STREAM, 0));
                if (!probe.valid()) {
                    failure = std::format("cannot create a TCP socket for '{}': {}", _endpoint, detail::bytesockio::errorText(errno));
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
    /// A shortage of descriptors or memory is given @ref detail::bytesockio::kTransientFaultLimit consecutive attempts,
    /// spaced by one poll interval, before it is called permanent: a shortage that never lifts is as final as a broken
    /// descriptor, and a reader that hid it would spin against the same failure for the life of the graph.
    [[nodiscard]] std::string noteFault(int code, std::string_view what) {
        switch (detail::bytesockio::classifyFault(code)) {
        case detail::bytesockio::Fault::Retry: return {};
        case detail::bytesockio::Fault::Transient:
            ++_transientFaults;
            if (_transientFaults < detail::bytesockio::kTransientFaultLimit) {
                waitBounded(static_cast<std::uint32_t>(detail::bytesockio::kPollMs));
                return {};
            }
            return std::format("{} failed {} times running: {}", what, _transientFaults, detail::bytesockio::errorText(code));
        case detail::bytesockio::Fault::Fatal: break;
        }
        return std::format("{} failed: {}", what, detail::bytesockio::errorText(code));
    }

    /// @brief Resolve the endpoint again for the next pass over its addresses, or say why it no longer resolves.
    ///
    /// A name is not fixed for the life of a graph: a peer that moved is reachable again on the next pass, and one
    /// whose name is gone is unreachable for good, which is the one resolution failure that ends the reader. A
    /// resolver that cannot answer just now keeps the addresses it last gave and the retries carry on with them. The
    /// question is asked at most once every @ref detail::bytesockio::kResolveIntervalMs, whatever `reconnect_ms` is,
    /// so a fast retry loop does not become a flood of queries.
    [[nodiscard]] std::string resolveAgain(std::vector<detail::bytesockio::Address>& addresses) {
        const auto now = std::chrono::steady_clock::now();
        if (now - _resolvedAt < std::chrono::milliseconds(detail::bytesockio::kResolveIntervalMs)) {
            return {};
        }
        _resolvedAt   = now;
        int  status   = 0;
        auto resolved = detail::bytesockio::resolveEndpoint(_endpoint, SOCK_STREAM, false, &status);
        if (resolved.has_value()) {
            addresses = std::move(*resolved);
            return {};
        }
        return status == EAI_AGAIN ? std::string{} : resolved.error();
    }

    [[nodiscard]] std::string openListener(detail::bytesockio::Descriptor& listener, std::span<const detail::bytesockio::Address> addresses) {
        std::string lastError = "no address was tried";
        for (const detail::bytesockio::Address& address : addresses) {
            detail::bytesockio::Descriptor candidate(::socket(address.family, SOCK_STREAM, 0));
            if (!candidate.valid()) {
                lastError = detail::bytesockio::errorText(errno);
                continue;
            }
            const int on = 1;
            std::ignore  = ::setsockopt(candidate.get(), SOL_SOCKET, SO_REUSEADDR, &on, static_cast<::socklen_t>(sizeof(on)));
            if (::bind(candidate.get(), detail::bytesockio::addressOf(address), address.length) != 0) {
                lastError = detail::bytesockio::errorText(errno);
                continue;
            }
            if (::listen(candidate.get(), 8) != 0) {
                lastError = detail::bytesockio::errorText(errno);
                continue;
            }
            std::ignore = detail::bytesockio::setNonBlocking(candidate.get());
            listener    = std::move(candidate);
            return {};
        }
        return std::format("cannot listen on '{}': {}", _endpoint, lastError);
    }

    [[nodiscard]] bool queueFull() {
        std::lock_guard lock(_mutex);
        return _queue.size() >= _queueBytes;
    }

    /// @brief Serve peers until a stop is asked for, or name the fault of this thread's own socket that ends it.
    [[nodiscard]] std::string receiveUntilStopped(detail::bytesockio::Descriptor& listener, std::vector<detail::bytesockio::Address>& addresses) {
        detail::bytesockio::Descriptor peer;
        std::vector<std::uint8_t>      chunk(65536UZ);

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

            if (_backpressure && queueFull()) {
                waitBounded(detail::bytesockio::kPollMs); // the socket is not read again until the graph makes room
                continue;
            }

            const short revents = detail::bytesockio::pollFor(peer.get(), POLLIN, detail::bytesockio::kPollMs);
            if (revents == 0) {
                continue; // the bounded wait expired, which is how the loop returns to observe a stop request
            }
            const ::ssize_t received = ::recv(peer.get(), chunk.data(), chunk.size(), 0);
            if (received > 0) {
                enqueue(std::span<const std::uint8_t>(chunk).first(static_cast<std::size_t>(received)));
                continue;
            }
            if (received == 0) {
                losePeer(peer); // an orderly close from the far end
                continue;
            }
            if (detail::bytesockio::wouldBlock(errno)) {
                continue;
            }
            {
                std::lock_guard lock(_mutex);
                ++nRecvErrors;
            }
            losePeer(peer); // a fault on the connection costs the connection; the listening socket is still good
        }
        return {};
    }

    /// @brief Take the next peer, or name the fault of this thread's own socket that ends the reader.
    [[nodiscard]] std::string acquirePeer(detail::bytesockio::Descriptor& listener, detail::bytesockio::Descriptor& peer, std::vector<detail::bytesockio::Address>& addresses) {
        if (_bind) {
            const short revents = detail::bytesockio::pollFor(listener.get(), POLLIN, detail::bytesockio::kPollMs);
            if ((revents & detail::bytesockio::kPollFault) != 0) {
                return std::format("the listening socket reported {}", detail::bytesockio::pollFaultText(revents));
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
        const detail::bytesockio::Address& address = addresses[_addressCursor];
        ++_addressCursor;

        detail::bytesockio::Descriptor candidate(::socket(address.family, SOCK_STREAM, 0));
        if (!candidate.valid()) {
            if (std::string fatal = noteFault(errno, "creating a TCP socket"); !fatal.empty()) {
                return fatal;
            }
            waitBounded(_reconnectMs);
            return {};
        }
        _transientFaults = 0UZ;
        std::ignore      = detail::bytesockio::setNonBlocking(candidate.get());
        // whatever the far end answers with — a refusal, a silence, a route that is gone — is the far end's, not this
        // socket's, so every failure from here on waits out the reconnect interval and tries again
        if (::connect(candidate.get(), detail::bytesockio::addressOf(address), address.length) != 0) {
            if (errno != EINPROGRESS) {
                waitBounded(_reconnectMs);
                return {};
            }
            const short revents = detail::bytesockio::pollFor(candidate.get(), POLLOUT, detail::bytesockio::kPollMs);
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

    void adoptPeer(detail::bytesockio::Descriptor& peer, int fd) {
        const int on = 1;
        std::ignore  = ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, static_cast<::socklen_t>(sizeof(on)));
        std::ignore  = detail::bytesockio::setNonBlocking(fd);
        peer.reset(fd);
        std::lock_guard lock(_mutex);
        if (_everConnected) {
            ++nReconnects;
        }
        _everConnected = true;
    }

    void losePeer(detail::bytesockio::Descriptor& peer) {
        peer.reset();
        std::lock_guard lock(_mutex);
        ++nDisconnects;
    }

    [[nodiscard]] std::string refuseExtraPeers(detail::bytesockio::Descriptor& listener) {
        if (!listener.valid()) {
            return {};
        }
        for (;;) {
            const short revents = detail::bytesockio::pollFor(listener.get(), POLLIN, 0);
            if ((revents & detail::bytesockio::kPollFault) != 0) {
                return std::format("the listening socket reported {}", detail::bytesockio::pollFaultText(revents));
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
            std::lock_guard lock(_mutex);
            ++nPeersRefused;
        }
    }

    /// @brief Queue @p bytes whole, applying `overflow` for whatever does not fit.
    ///
    /// `_backpressure` already keeps the read loop from calling in here once the queue is full, but one `recv()` may
    /// still return more than the room left: TCP hands over whatever arrived in one read and there is no frame to
    /// hold that read to, so the excess is counted exactly as a `drop_oldest` overflow would count it.
    void enqueue(std::span<const std::uint8_t> bytes) {
        {
            std::lock_guard lock(_mutex);
            for (const std::uint8_t byte : bytes) {
                if (_queue.size() >= _queueBytes) {
                    _queue.pop_front();
                    ++nBytesDropped;
                }
                _queue.push_back(byte);
            }
            nBytes += bytes.size();
        }
        _cv.notify_one();
    }
};

} // namespace gr::blocks::network

#endif // GNURADIO_NETWORK_TCPBYTEIO_HPP
