#ifndef GNURADIO_NETWORK_ZMQENVELOPEIO_HPP
#define GNURADIO_NETWORK_ZMQENVELOPEIO_HPP

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <format>
#include <functional>
#include <mutex>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <zmq.hpp>

#include <gnuradio-4.0/AtomicRef.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/thread/thread_pool.hpp>

#include <gnuradio-4.0/network/PacketEnvelope.hpp>
#include <gnuradio-4.0/network/ZmqTransport.hpp>

/**
 * @brief The ZeroMQ carriage of `gr::network::PacketEnvelope`, owned once rather than per block.
 *
 * Every block in this module that puts the envelope on a ZeroMQ socket needs the same two things and needs them
 * identically: a queue whose I/O thread creates a socket, sends four-frame messages and closes it, and a reader
 * thread that takes whole multipart messages off a socket and hands them to a decoder. Neither depends on what the
 * payload means, and both are the parts where a mistake is a race rather than a wrong number — which is exactly the
 * code a module must not hold three copies of.
 *
 * Two rules the callers inherit by using these types. **A libzmq socket is a single-thread object**, so each of these
 * owns its socket end to end and no scheduler thread ever touches one; a socket used from a pool worker trips
 * libzmq's own signaler assertion. **The thread must be gone before what it writes into**, so each type joins its
 * thread in its own destructor and everything the thread touches is a member of the same object — which makes the
 * member declaration order inside a block irrelevant, where a bare thread would have made it load-bearing.
 */
namespace gr::blocks::network::detail::zmqenvelope {

using detail::zmqio::endpointHint;
using detail::zmqio::Pattern;

/// @brief One finished envelope waiting for the I/O thread, frames 1 to 3. Frame 0 is the block's constant topic.
struct Outgoing {
    std::array<std::uint8_t, gr::network::kHeaderBytesV1> header{};
    std::string                                           metadata{};
    std::vector<std::uint8_t>                             payload{};

    [[nodiscard]] std::uint64_t bytes() const noexcept { return header.size() + metadata.size() + payload.size(); }
};

/// @brief The socket settings a block freezes when it starts, in one object because they are read as one.
struct SocketConfig {
    std::string   endpoint{};
    std::string   topic{};
    Pattern       pattern         = Pattern::Pub;
    bool          bind            = true;
    std::int32_t  hwm             = 16;   ///< ZMQ_SNDHWM or ZMQ_RCVHWM, whichever this socket has
    std::int32_t  lingerMs        = 0;    ///< ZMQ_LINGER
    std::uint64_t maxMessageBytes = 0ULL; ///< ZMQ_MAXMSGSIZE; receivers only, and required there
};

/// @brief The in-process send queue and the one thread that owns the socket it drains into.
///
/// `enqueue` returns false only under backpressure, which the caller reads as "these items are not consumed"; under
/// `drop_oldest` it sheds the stalest envelope, counts it, and always succeeds. Teardown is bounded and discards what
/// is still queued, counted in `droppedAtStop`, because an unbounded drain against a stalled peer is a graph that
/// will not tear down.
class SendQueue {
public:
    struct Counters {
        std::uint64_t packetsSent       = 0ULL; ///< envelopes handed to libzmq
        std::uint64_t bytesSent         = 0ULL; ///< envelope bytes handed to libzmq
        std::uint64_t droppedOnOverflow = 0ULL; ///< queued envelopes discarded under drop_oldest
        std::uint64_t droppedAtStop     = 0ULL; ///< envelopes still queued when the I/O thread stopped
        std::uint64_t sendErrors        = 0ULL; ///< zmq_send failures other than EAGAIN
    };

    SendQueue()                            = default;
    SendQueue(const SendQueue&)            = delete;
    SendQueue(SendQueue&&)                 = delete;
    SendQueue& operator=(const SendQueue&) = delete;
    SendQueue& operator=(SendQueue&&)      = delete;
    ~SendQueue() { stop(); }

    void configure(std::size_t queueMessages, std::uint64_t queueBytes, bool backpressure) noexcept {
        _queueMessages = queueMessages;
        _queueBytes    = queueBytes;
        _backpressure  = backpressure;
    }

    /// @brief Open the socket on a thread of the framework's default I/O pool. Throws what libzmq refused, with the endpoint named.
    void start(SocketConfig config, std::string_view blockName) {
        _config = std::move(config);
        _name.assign(blockName);
        {
            std::lock_guard lock(_mutex);
            _stopRequested = false;
            _opened        = false;
            _openFailure.clear();
            _queue.clear();
            _queuedBytes = 0ULL;
        }
        gr::atomic_ref(_threadDone).store_release(false);
        thread_pool::Manager::defaultIoPool()->execute([this] { sendLoop(); });

        std::unique_lock lock(_mutex);
        _cv.wait(lock, [this] { return _opened; });
        if (!_openFailure.empty()) {
            const std::string failure = _openFailure;
            lock.unlock();
            gr::atomic_ref(_threadDone).wait(false); // a failed start leaves no thread behind
            throw gr::exception(failure);
        }
    }

    void stop() {
        {
            std::lock_guard lock(_mutex);
            _stopRequested = true;
        }
        _cv.notify_all();
        gr::atomic_ref(_threadDone).wait(false);
    }

    /// @brief Put an envelope on the queue, applying the overflow rule when it is full. False means "not consumed".
    [[nodiscard]] bool enqueue(Outgoing&& envelope) {
        const std::uint64_t bytes = envelope.bytes();
        std::unique_lock    lock(_mutex);
        while (_queue.size() >= _queueMessages || (!_queue.empty() && _queuedBytes + bytes > _queueBytes)) {
            if (_backpressure) {
                return false;
            }
            _queuedBytes -= _queue.front().bytes(); // the newest envelopes are what a live consumer wants
            _queue.pop_front();
            ++_counters.droppedOnOverflow;
        }
        _queuedBytes += bytes;
        _queue.push_back(std::move(envelope));
        lock.unlock();
        _cv.notify_one();
        return true;
    }

    [[nodiscard]] Counters counters() const {
        std::lock_guard lock(_mutex);
        return _counters;
    }

private:
    mutable std::mutex      _mutex;
    std::condition_variable _cv;
    std::deque<Outgoing>    _queue;
    Counters                _counters{};
    std::uint64_t           _queuedBytes   = 0ULL;
    bool                    _stopRequested = false;
    bool                    _opened        = false;
    std::string             _openFailure{};
    bool                    _threadDone = true; ///< true until start() launches the thread

    SocketConfig  _config{};
    std::string   _name{};
    std::size_t   _queueMessages = 1024UZ;
    std::uint64_t _queueBytes    = 16777216ULL;
    bool          _backpressure  = false;

    /// @brief The whole of this object's contact with libzmq: one thread creates, uses and closes the socket.
    void sendLoop() {
        thread_pool::thread::setThreadName(std::format("zmqenvtx:{}", _name));
        std::optional<zmq::context_t> context;
        std::optional<zmq::socket_t>  socket;
        std::string                   failure;
        try {
            context.emplace(1);
            socket.emplace(*context, _config.pattern == Pattern::Pub ? zmq::socket_type::pub : zmq::socket_type::push);
            socket->set(zmq::sockopt::sndhwm, _config.hwm);
            socket->set(zmq::sockopt::linger, _config.lingerMs);
            if (_config.bind) {
                socket->bind(_config.endpoint);
            } else {
                socket->connect(_config.endpoint);
            }
        } catch (const zmq::error_t& error) {
            failure = std::format("cannot {} '{}': {}{}", _config.bind ? "bind" : "connect to", _config.endpoint, error.what(), endpointHint(_config.endpoint));
        }

        {
            std::lock_guard lock(_mutex);
            _openFailure = failure;
            _opened      = true;
        }
        _cv.notify_all();

        if (failure.empty()) {
            sendUntilStopped(*socket);
        }
        socket.reset();
        context.reset();
        gr::atomic_ref(_threadDone).store_release(true);
        gr::atomic_ref(_threadDone).notify_all();
    }

    void sendUntilStopped(zmq::socket_t& socket) {
        using namespace std::chrono_literals;
        while (true) {
            Outgoing envelope;
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
                _queuedBytes -= envelope.bytes();
            }
            if (!sendEnvelope(socket, envelope)) {
                break;
            }
        }
        // one exit, so what teardown loses is counted however the loop ended
        std::lock_guard lock(_mutex);
        _counters.droppedAtStop += _queue.size();
        _queue.clear();
        _queuedBytes = 0ULL;
    }

    /// @brief Send one four-frame message, retrying while a PUSH peer is absent. False means the thread should exit.
    ///
    /// Every part goes with ZMQ_DONTWAIT, so the thread never blocks indefinitely inside libzmq. A PUB socket never
    /// reports EAGAIN; a PUSH socket does when no peer is ready, and libzmq only tests that condition on the first
    /// part of a message, so an EAGAIN there leaves nothing half-written. The retry waits on the socket rather than
    /// on a timer, so a peer that connects is served at once.
    [[nodiscard]] bool sendEnvelope(zmq::socket_t& socket, const Outgoing& envelope) {
        using namespace std::chrono_literals;
        constexpr zmq::send_flags more = zmq::send_flags::sndmore | zmq::send_flags::dontwait;
        while (true) {
            try {
                const zmq::send_result_t first = socket.send(zmq::buffer(_config.topic), more);
                if (!first.has_value()) { // the socket is in its mute state and nothing was written
                    zmq_pollitem_t item{socket.handle(), 0, ZMQ_POLLOUT, 0};
                    std::ignore = zmq::poll(&item, 1UZ, 100ms);
                    std::lock_guard lock(_mutex);
                    if (_stopRequested) {
                        ++_counters.droppedAtStop;
                        return false;
                    }
                    continue;
                }
                std::ignore = socket.send(zmq::buffer(envelope.header), more);
                std::ignore = socket.send(zmq::buffer(envelope.metadata), more);
                std::ignore = socket.send(zmq::buffer(envelope.payload), zmq::send_flags::dontwait);
            } catch (const zmq::error_t& error) {
                std::lock_guard lock(_mutex);
                ++_counters.sendErrors;
                std::println(stderr, "gr::blocks::network '{}': send failed on '{}': {}", _name, _config.endpoint, error.what());
                return !_stopRequested;
            }
            std::lock_guard lock(_mutex);
            ++_counters.packetsSent;
            _counters.bytesSent += envelope.bytes();
            return true;
        }
    }
};

/// @brief The one thread that owns a receiving socket, handing whole multipart messages to a decoder.
///
/// The decoder runs on this thread, which is the point: a flood of malformed messages costs the reader and never the
/// scheduler, and the queue a graph drains then holds only objects whose shape is already established. What it is
/// handed is at most five frames — one more than a legal message, which is all it takes to refuse a longer one — the
/// total part count, and the envelope's byte count with frame 0 excluded, because the topic is the transport's.
class Receiver {
public:
    using Decoder = std::function<void(std::span<zmq::message_t> parts, std::size_t partCount, std::uint64_t bytes)>;

    Receiver()                           = default;
    Receiver(const Receiver&)            = delete;
    Receiver(Receiver&&)                 = delete;
    Receiver& operator=(const Receiver&) = delete;
    Receiver& operator=(Receiver&&)      = delete;
    ~Receiver() { stop(); }

    /// @brief Open the socket on a thread of the framework's default I/O pool. Throws what libzmq refused, with the endpoint named.
    void start(SocketConfig config, std::string_view blockName, Decoder decoder) {
        _config = std::move(config);
        _name.assign(blockName);
        _decoder = std::move(decoder);
        {
            std::lock_guard lock(_mutex);
            _stopRequested = false;
            _opened        = false;
            _failed        = false;
            _openFailure.clear();
        }
        gr::atomic_ref(_threadDone).store_release(false);
        thread_pool::Manager::defaultIoPool()->execute([this] { readLoop(); });

        std::unique_lock lock(_mutex);
        _cv.wait(lock, [this] { return _opened; });
        if (!_openFailure.empty()) {
            const std::string failure = _openFailure;
            lock.unlock();
            gr::atomic_ref(_threadDone).wait(false);
            throw gr::exception(failure);
        }
    }

    void stop() {
        {
            std::lock_guard lock(_mutex);
            _stopRequested = true;
        }
        _cv.notify_all();
        gr::atomic_ref(_threadDone).wait(false);
    }

    /// @brief True once the reader has died other than by request, which a quiet wire never does.
    [[nodiscard]] bool failed() const {
        std::lock_guard lock(_mutex);
        return _failed;
    }

    [[nodiscard]] std::uint64_t messagesReceived() const {
        std::lock_guard lock(_mutex);
        return _messages;
    }

    [[nodiscard]] std::uint64_t bytesReceived() const {
        std::lock_guard lock(_mutex);
        return _bytes;
    }

private:
    mutable std::mutex      _mutex;
    std::condition_variable _cv;
    bool                    _stopRequested = false;
    bool                    _opened        = false;
    bool                    _failed        = false;
    std::string             _openFailure{};
    bool                    _threadDone = true;
    std::uint64_t           _messages   = 0ULL;
    std::uint64_t           _bytes      = 0ULL;

    SocketConfig _config{};
    std::string  _name{};
    Decoder      _decoder{};

    void readLoop() {
        thread_pool::thread::setThreadName(std::format("zmqenvrx:{}", _name));
        std::optional<zmq::context_t> context;
        std::optional<zmq::socket_t>  socket;
        std::string                   failure;
        try {
            context.emplace(1);
            socket.emplace(*context, _config.pattern == Pattern::Sub ? zmq::socket_type::sub : zmq::socket_type::pull);
            // the bound goes into libzmq as well as into the block, so an oversize message is refused before the
            // library allocates for it; the peer that sent one is disconnected, and the loss reads as a gap
            socket->set(zmq::sockopt::maxmsgsize, static_cast<std::int64_t>(_config.maxMessageBytes));
            socket->set(zmq::sockopt::rcvhwm, _config.hwm);
            socket->set(zmq::sockopt::rcvtimeo, 100); // a bounded wait keeps stop() responsive without busy-spinning
            socket->set(zmq::sockopt::linger, _config.lingerMs);
            if (_config.pattern == Pattern::Sub) {
                socket->set(zmq::sockopt::subscribe, _config.topic);
            }
            if (_config.bind) {
                socket->bind(_config.endpoint);
            } else {
                socket->connect(_config.endpoint);
            }
        } catch (const zmq::error_t& error) {
            failure = std::format("cannot {} '{}': {}{}", _config.bind ? "bind" : "connect to", _config.endpoint, error.what(), endpointHint(_config.endpoint));
        }

        {
            std::lock_guard lock(_mutex);
            _openFailure = failure;
            _opened      = true;
        }
        _cv.notify_all();

        if (failure.empty()) {
            receiveUntilStopped(*socket);
        }
        socket.reset();
        context.reset();
        gr::atomic_ref(_threadDone).store_release(true);
        gr::atomic_ref(_threadDone).notify_all();
    }

    void receiveUntilStopped(zmq::socket_t& socket) {
        std::vector<zmq::message_t> parts;
        while (true) {
            {
                std::lock_guard lock(_mutex);
                if (_stopRequested) {
                    return;
                }
            }
            parts.clear();
            std::size_t   partCount = 0UZ;
            std::uint64_t partBytes = 0ULL;
            try {
                zmq::message_t part;
                if (!socket.recv(part, zmq::recv_flags::none).has_value()) {
                    continue; // ZMQ_RCVTIMEO expired, which is how the loop returns to observe a stop request
                }
                while (true) {
                    ++partCount;
                    if (partCount > 1UZ) { // frame 0 is the topic and is no part of the envelope
                        partBytes += part.size();
                    }
                    if (parts.size() < 5UZ) { // five is one more than a legal message, which is all it takes to refuse
                        parts.push_back(std::move(part));
                    }
                    if (socket.get(zmq::sockopt::rcvmore) == 0) {
                        break;
                    }
                    if (!socket.recv(part, zmq::recv_flags::none).has_value()) {
                        break; // atomic delivery makes this unreachable; treated as a frame-count refusal if it happens
                    }
                }
            } catch (const zmq::error_t& error) {
                std::lock_guard lock(_mutex);
                if (!_stopRequested) {
                    _failed = true;
                    std::println(stderr, "gr::blocks::network '{}': receive failed on '{}': {}", _name, _config.endpoint, error.what());
                }
                return;
            }
            {
                std::lock_guard lock(_mutex);
                ++_messages;
                _bytes += partBytes;
            }
            _decoder(parts, partCount, partBytes);
        }
    }
};

} // namespace gr::blocks::network::detail::zmqenvelope

#endif // GNURADIO_NETWORK_ZMQENVELOPEIO_HPP
