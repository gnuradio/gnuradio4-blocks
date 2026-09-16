#ifndef GNURADIO_ZMQPACKETIO_HPP
#define GNURADIO_ZMQPACKETIO_HPP

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <format>
#include <mutex>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <zmq.hpp>

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

#include <gnuradio-4.0/basic/RecordMetadata.hpp>
#include <gnuradio-4.0/network/PacketEnvelope.hpp>

namespace gr::blocks::network {

namespace detail::zmqio {

// The record-metadata vocabulary and its declared types are the basic module's, reused rather than restated: two
// tables for one vocabulary is the drift these blocks exist to avoid, and both ends of this transport have to agree
// on the same table as the blocks that convert records into the packets it carries.
using gr::blocks::basic::detail::packet::holdsVocabularyType;
using gr::blocks::basic::detail::packet::shortKey;
using gr::blocks::basic::detail::packet::vocabularyType;

/// @brief The socket pattern a block runs, resolved from its `pattern` setting once at configure time.
enum class Pattern : std::uint8_t { Pub, Push, Sub, Pull };

/// @brief The key the carrier's `timestamp` field crosses under, removed again by the receiving block.
///
/// Producer-private rather than vocabulary: it exists to serve one carrier field across one boundary and is consumed
/// on arrival, so the vocabulary's one-spelling rule survives the crossing. The alternative to carrying it is losing
/// it silently, which is what the envelope exists to prevent.
inline constexpr std::string_view kTimestampKey = "packet_timestamp";

/// @brief Vocabulary keys of @p map whose value type disagrees with the declaration.
///
/// Counted and never dropped. At a record boundary a wrongly typed key is dropped because an absent key at least
/// reads as absent, but here the value's author is in another process and cannot be told: dropping it would erase the
/// only evidence that a peer is misconfigured. The value still reads as absent through the declared accessor, so
/// nothing downstream is misled; the counter is what makes the peer's bug visible.
[[nodiscard]] inline std::uint64_t countMistypedKeys(const property_map& map) noexcept {
    std::uint64_t mistyped = 0ULL;
    for (const auto& [key, value] : map) {
        if (!holdsVocabularyType(vocabularyType(shortKey(std::string_view(key))), value)) {
            ++mistyped;
        }
    }
    return mistyped;
}

/// @brief The one endpoint diagnostic worth stating rather than leaving to be rediscovered.
///
/// libzmq answers an endpoint with no transport prefix with a bare `EINVAL`, and that is the common typo.
[[nodiscard]] inline std::string endpointHint(std::string_view endpoint) { return endpoint.find("://") == std::string_view::npos ? std::format(" — '{}' names no transport; a libzmq endpoint begins with a prefix such as tcp://, ipc:// or inproc://", endpoint) : std::string{}; }

[[nodiscard]] inline Pattern sendPatternFromName(std::string_view name) {
    if (name == "pub") {
        return Pattern::Pub;
    }
    if (name == "push") {
        return Pattern::Push;
    }
    throw gr::exception(std::format("pattern is '{}'; a sink takes 'pub' (fan out, never blocks, drops in mute state) or 'push' (round-robin, blocks in mute state, discards nothing)", name));
}

[[nodiscard]] inline Pattern receivePatternFromName(std::string_view name) {
    if (name == "sub") {
        return Pattern::Sub;
    }
    if (name == "pull") {
        return Pattern::Pull;
    }
    throw gr::exception(std::format("pattern is '{}'; a source takes 'sub' (subscribes to a prefix) or 'pull' (fair-queued)", name));
}

/// @brief One finished envelope waiting for the I/O thread, frames 1 to 3. Frame 0 is the block's constant topic.
struct Outgoing {
    std::array<std::uint8_t, gr::network::kHeaderBytesV1> header{};
    std::string                                           metadata{};
    std::vector<std::uint8_t>                             payload{};

    [[nodiscard]] std::uint64_t bytes() const noexcept { return header.size() + metadata.size() + payload.size(); }
};

} // namespace detail::zmqio

GR_REGISTER_BLOCK(gr::blocks::network::ZmqPacketSink, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
requires gr::network::EnvelopeItem<T>
struct ZmqPacketSink : Block<ZmqPacketSink<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Publishes each incoming gr::Packet<T> as one versioned four-frame ZeroMQ message.

One packet becomes one message of four parts — topic, 32-byte header, metadata, payload — and libzmq guarantees that
a peer receives all four or none, so an envelope is never torn. The header states the item type, item size, item
count, payload length, metadata length, byte order and wire version, so a receiver this process did not build reads
the payload without being told anything out of band. The metadata frame is the packet's own `meta_information[0]`
serialized by `gr::pmt::yaml`, copied key for key with nothing filtered: a producer-private key crosses the process
boundary intact.

`pub` is the default pattern because the failure mode of the alternative is invisible. Two consumers on a `push` sink
each receive half the packets, with no error and no counter anywhere; two on a `pub` sink each receive all of them.
`push` is offered because a lossless hand-off to a decoder that must see every packet is a real requirement, and it
is the pattern to pair with `overflow = "backpressure"`.

A slow subscriber must not be able to stall the flowgraph, so the default `overflow` is `drop_oldest`: the oldest
queued envelope is discarded and counted, the newest is kept, and the receiver sees the loss as a `sequence` gap.
`backpressure` instead consumes fewer input items, so the stall propagates upstream by the framework's own path
rather than by blocking a thread — the honest setting for an offline chain and the wrong one for a radio, where it
trades a counted packet drop for an uncounted sample drop further upstream.

The block fills `sequence` only when the packet does not already carry one. Both values are derived, and the less
local one is the more useful: an upstream `sequence` counts what the original producer emitted, so a gap in it at the
receiver means "lost somewhere between that producer and here", while the sink's own counter can only count what
reached the sink and writing it would renumber the stream and hide upstream loss. Every envelope this sink emits
carries `sequence`, which is what makes loss reconstructible at the far end.

A libzmq socket is a single-thread object and a scheduler pool worker is not a stable home for one, so the block owns
a dedicated I/O thread that creates, uses and closes the socket. `processBulk` serializes and enqueues and never
touches the socket; the I/O thread does nothing but send. Teardown is bounded and discards what is still queued,
counted in `nDroppedAtStop`, because an unbounded drain against a stalled peer is a graph that will not tear down.
)"">;

    PortIn<Packet<T>>                   in;
    PortOut<Packet<T>, Async, Optional> reject;

    Annotated<std::string, "endpoint", Visible, Doc<"libzmq endpoint, e.g. tcp://127.0.0.1:5555; required, there is no default">> endpoint{};
    Annotated<bool, "bind", Doc<"bind the endpoint rather than connect to it; the producer is normally the stable end">>          bind              = true;
    Annotated<std::string, "pattern", Doc<"'pub' (fan out) or 'push' (round-robin, lossless, one consumer)">>                     pattern           = std::string("pub");
    Annotated<std::string, "topic", Doc<"frame 0, the subscription prefix; empty publishes to every subscriber">>                 topic             = std::string("");
    Annotated<std::string, "overflow", Doc<"'drop_oldest' or 'backpressure', applied when the in-process send queue is full">>    overflow          = std::string("drop_oldest");
    Annotated<gr::Size_t, "queue_messages", Doc<"in-process send queue depth">>                                                   queue_messages    = 1024U;
    Annotated<std::uint64_t, "queue_bytes", Unit<"byte">, Doc<"in-process send queue size">>                                      queue_bytes       = 16777216ULL;
    Annotated<gr::Size_t, "send_hwm", Doc<"ZMQ_SNDHWM; libzmq's own 1000 is up to 64 MiB of buffer this graph cannot see">>       send_hwm          = 16U;
    Annotated<std::uint64_t, "max_message_bytes", Unit<"byte">, Doc<"a packet whose envelope would exceed this is rejected">>     max_message_bytes = 16777216ULL;
    Annotated<std::int32_t, "linger_ms", Unit<"ms">, Doc<"ZMQ_LINGER; 0 discards pending messages immediately on close">>         linger_ms         = 0;

    GR_MAKE_REFLECTABLE(ZmqPacketSink, in, reject, endpoint, bind, pattern, topic, overflow, queue_messages, queue_bytes, send_hwm, max_message_bytes, linger_ms);

    // Counted, stated drops and refusals. Plain members, printed once by stop(); nothing here is on the sample path.
    // Everything the block sheds is counted; what libzmq sheds inside a PUB socket is not countable at the sender and
    // is reported instead at the receiver, from the `sequence` this sink guarantees.
    std::uint64_t nPacketsSent          = 0ULL; ///< envelopes handed to libzmq
    std::uint64_t nBytesSent            = 0ULL; ///< envelope bytes handed to libzmq
    std::uint64_t nRejectedPackets      = 0ULL; ///< packets refused for exceeding max_message_bytes
    std::uint64_t nDroppedOnOverflow    = 0ULL; ///< queued envelopes discarded under overflow = drop_oldest
    std::uint64_t nBackpressureStalls   = 0ULL; ///< processBulk calls that consumed fewer items than they read
    std::uint64_t nSequenceDeclined     = 0ULL; ///< packets that already stated sequence
    std::uint64_t nMetaKeysMistyped     = 0ULL; ///< vocabulary keys whose type disagrees with the declaration
    std::uint64_t nTimestampsCarried    = 0ULL; ///< packets with a non-zero Packet::timestamp
    std::uint64_t nDefaultValuesDropped = 0ULL; ///< packets whose default_value differs from T(), which has no wire field
    std::uint64_t nDroppedAtStop        = 0ULL; ///< envelopes still queued when the I/O thread stopped
    std::uint64_t nSendErrors           = 0ULL; ///< zmq_send failures other than EAGAIN

    std::mutex                          _mutex;
    std::condition_variable             _cv;
    std::deque<detail::zmqio::Outgoing> _queue;
    std::uint64_t                       _queuedBytes   = 0ULL;
    bool                                _stopRequested = false;
    bool                                _opened        = false;
    std::string                         _openFailure{};
    bool                                _ioThreadDone = true; ///< true until start() launches the I/O thread

    std::uint64_t          _sequence = 0ULL; ///< the value the sink writes when a packet states none
    std::string            _endpoint{};      ///< the socket settings, frozen for the duration of one run
    std::string            _topic{};
    detail::zmqio::Pattern _pattern         = detail::zmqio::Pattern::Pub;
    bool                   _bind            = true;
    std::size_t            _queueMessages   = 1024UZ;
    std::uint64_t          _queueBytes      = 16777216ULL;
    std::int32_t           _sendHwm         = 16;
    std::int32_t           _lingerMs        = 0;
    bool                   _socketOpen      = false;
    bool                   _backpressure    = false;
    std::uint64_t          _maxMessageBytes = 16777216ULL;

    /// @brief Joins the I/O thread however the block dies.
    ///
    /// Must be the last declared member, so it is destroyed first and the thread is gone before the queue, mutex and
    /// condition variable it uses. `stop()` cannot be relied on: the scheduler does not call it when a graph ends in
    /// ERROR, and `~Block()` cannot stand in because derived members are destroyed before it runs.
    struct IoThreadGuard {
        ZmqPacketSink* self;
        explicit IoThreadGuard(ZmqPacketSink* owner) noexcept : self(owner) {}
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

            detail::zmqio::Outgoing envelope;
            std::uint64_t           payloadBytes = packet.signal_values.size();
            payloadBytes *= sizeof(T);
            property_map map  = buildMetadata(packet);
            envelope.metadata = pmt::yaml::serialize(map);

            const std::uint64_t headerBytes = gr::network::kHeaderBytesV1;
            const std::uint64_t total       = headerBytes + envelope.metadata.size() + payloadBytes;
            if (total > _maxMessageBytes) {
                if (rejectConnected && onReject >= rejectSpan.size()) {
                    break; // no room on the port this packet belongs on; it stays in the buffer
                }
                if (rejectConnected) {
                    Packet<T> refused = packet; // republished whole: what is wrong with it is its size, not its content
                    refused.meta_information.resize(1UZ);
                    refused.meta_information[0UZ].insert_or_assign(property_map::key_type("discard_reason"), pmt::Value(std::string("over_max_message_bytes")));
                    rejectSpan[onReject] = std::move(refused);
                }
                ++onReject;
                ++nRejectedPackets;
                ++consumed;
                continue; // the sequence counter never advanced, so a refused packet does not renumber the stream
            }

            gr::network::EnvelopeHeader header;
            header.item_type     = gr::network::kItemTypeCode<T>;
            header.item_size     = static_cast<std::uint8_t>(sizeof(T));
            header.item_count    = static_cast<std::uint32_t>(packet.signal_values.size());
            header.payload_bytes = static_cast<std::uint32_t>(payloadBytes);
            header.meta_bytes    = static_cast<std::uint32_t>(envelope.metadata.size());
            envelope.header      = gr::network::encodeHeader(header);

            // copied, never moved out of the input span: a Packet output port may fan out and the buffer slot is
            // shared, so moving would empty a packet another reader is about to see
            const std::span<const std::byte> raw = std::as_bytes(std::span<const T>(packet.signal_values));
            envelope.payload.resize(raw.size());
            if (!raw.empty()) {
                std::memcpy(envelope.payload.data(), raw.data(), raw.size());
            }

            if (!enqueue(std::move(envelope))) {
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
        std::ignore = detail::zmqio::sendPatternFromName(pattern.value);
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
        if (topic.value != _topic) {
            refuse("topic");
        }
        if (bind.value != _bind) {
            refuse("bind");
        }
        if (detail::zmqio::sendPatternFromName(pattern.value) != _pattern) {
            refuse("pattern");
        }
        if (static_cast<std::size_t>(queue_messages.value) != _queueMessages) {
            refuse("queue_messages");
        }
        if (queue_bytes.value != _queueBytes) {
            refuse("queue_bytes");
        }
        if (static_cast<std::int32_t>(send_hwm.value) != _sendHwm) {
            refuse("send_hwm");
        }
        if (linger_ms.value != _lingerMs) {
            refuse("linger_ms");
        }
    }

    void freezeSocketSettings() {
        _endpoint      = endpoint.value;
        _topic         = topic.value;
        _bind          = bind.value;
        _pattern       = detail::zmqio::sendPatternFromName(pattern.value);
        _queueMessages = static_cast<std::size_t>(queue_messages.value);
        _queueBytes    = queue_bytes.value;
        _sendHwm       = static_cast<std::int32_t>(send_hwm.value);
        _lingerMs      = linger_ms.value;
    }

    /// @brief The metadata map the envelope carries: the packet's own, plus `sequence` where it states none.
    [[nodiscard]] property_map buildMetadata(const Packet<T>& packet) {
        property_map map;
        if (!packet.meta_information.empty()) {
            map = packet.meta_information[0UZ]; // copied key for key, nothing filtered and nothing consumed
        }
        nMetaKeysMistyped += detail::zmqio::countMistypedKeys(map);

        if (map.find("sequence") == map.end()) {
            map.insert_or_assign(property_map::key_type("sequence"), pmt::Value(_sequence));
        } else {
            ++nSequenceDeclined; // the producer's own count is the less local one, so it stands
        }
        if (packet.timestamp != 0) {
            map.insert_or_assign(property_map::key_type(detail::zmqio::kTimestampKey), pmt::Value(packet.timestamp));
            ++nTimestampsCarried;
        }
        if (packet.default_value != T()) {
            ++nDefaultValuesDropped; // the field is not reflected and the envelope has no place for it; counted, not silent
        }
        return map;
    }

    /// @brief Put an envelope on the send queue, applying `overflow` when it is full. False means "not consumed".
    [[nodiscard]] bool enqueue(detail::zmqio::Outgoing&& envelope) {
        const std::uint64_t bytes = envelope.bytes();
        std::unique_lock    lock(_mutex);
        while (_queue.size() >= _queueMessages || (!_queue.empty() && _queuedBytes + bytes > _queueBytes)) {
            if (_backpressure) {
                return false;
            }
            _queuedBytes -= _queue.front().bytes(); // the newest packets are what a live consumer wants
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
        append("dropped on overflow", nDroppedOnOverflow);
        append("backpressure stalls", nBackpressureStalls);
        append("sequence declined", nSequenceDeclined);
        append("metadata keys mistyped", nMetaKeysMistyped);
        append("timestamps carried", nTimestampsCarried);
        append("default values dropped", nDefaultValuesDropped);
        append("dropped at stop", nDroppedAtStop);
        append("send errors", nSendErrors);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::network::ZmqPacketSink '{}': {}", this->name, report);
        }
    }

    /// @brief The whole of this block's contact with libzmq: one thread creates the context and socket, sends, and
    /// closes them. A socket used from a scheduler pool worker trips libzmq's own signaler assertion.
    void ioSendLoop() {
        thread_pool::thread::setThreadName(std::format("zmqpktsink:{}", this->name.value));
        std::optional<zmq::context_t> context;
        std::optional<zmq::socket_t>  socket;
        std::string                   failure;
        try {
            context.emplace(1);
            socket.emplace(*context, _pattern == detail::zmqio::Pattern::Pub ? zmq::socket_type::pub : zmq::socket_type::push);
            socket->set(zmq::sockopt::sndhwm, _sendHwm);
            socket->set(zmq::sockopt::linger, _lingerMs);
            if (_bind) {
                socket->bind(_endpoint);
            } else {
                socket->connect(_endpoint);
            }
        } catch (const zmq::error_t& error) {
            failure = std::format("cannot {} '{}': {}{}", _bind ? "bind" : "connect to", _endpoint, error.what(), detail::zmqio::endpointHint(_endpoint));
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
        gr::atomic_ref(_ioThreadDone).store_release(true);
        gr::atomic_ref(_ioThreadDone).notify_all();
    }

    void sendUntilStopped(zmq::socket_t& socket) {
        using namespace std::chrono_literals;
        while (true) {
            detail::zmqio::Outgoing envelope;
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
        // teardown is bounded and loses what is in flight; a drain against a stalled peer is unbounded, and an
        // unbounded stop() is a graph that will not tear down. One exit, so the loss is counted however the loop ends.
        std::lock_guard lock(_mutex);
        nDroppedAtStop += _queue.size();
        _queue.clear();
        _queuedBytes = 0ULL;
    }

    /// @brief Send one four-frame message, retrying while a PUSH peer is absent. False means the thread should exit.
    ///
    /// Every part goes with ZMQ_DONTWAIT, so the I/O thread never blocks indefinitely inside libzmq. A PUB socket
    /// never reports EAGAIN; a PUSH socket does when no peer is ready, and libzmq only tests that condition on the
    /// first part of a message, so an EAGAIN there leaves nothing half-written. The retry waits on the socket itself
    /// rather than on a timer, so a peer that connects is served at once.
    [[nodiscard]] bool sendEnvelope(zmq::socket_t& socket, const detail::zmqio::Outgoing& envelope) {
        using namespace std::chrono_literals;
        constexpr zmq::send_flags more = zmq::send_flags::sndmore | zmq::send_flags::dontwait;
        while (true) {
            try {
                const zmq::send_result_t first = socket.send(zmq::buffer(_topic), more);
                if (!first.has_value()) { // the socket is in its mute state and nothing was written
                    zmq_pollitem_t item{socket.handle(), 0, ZMQ_POLLOUT, 0};
                    std::ignore = zmq::poll(&item, 1UZ, 100ms);
                    std::lock_guard lock(_mutex);
                    if (_stopRequested) {
                        ++nDroppedAtStop;
                        return false;
                    }
                    continue;
                }
                std::ignore = socket.send(zmq::buffer(envelope.header), more);
                std::ignore = socket.send(zmq::buffer(envelope.metadata), more);
                std::ignore = socket.send(zmq::buffer(envelope.payload), zmq::send_flags::dontwait);
            } catch (const zmq::error_t& error) {
                std::lock_guard lock(_mutex);
                ++nSendErrors;
                std::println(stderr, "gr::blocks::network::ZmqPacketSink '{}': send failed on '{}': {}", this->name, _endpoint, error.what());
                return !_stopRequested;
            }
            std::lock_guard lock(_mutex);
            ++nPacketsSent;
            nBytesSent += envelope.bytes();
            return true;
        }
    }
};

GR_REGISTER_BLOCK(gr::blocks::network::ZmqPacketSource, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
requires gr::network::EnvelopeItem<T>
struct ZmqPacketSource : Block<ZmqPacketSource<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Receives versioned four-frame ZeroMQ messages and publishes each as one gr::Packet<T>.

Validation happens on the block's own reader thread, so a flood of malformed messages costs that thread and never the
scheduler, and the queue the graph drains holds only objects whose shape is established. A message is refused for one
named reason — the first that applies — and a refusal is observable three ways at once: the raw bytes go to `reject`
with `discard_reason` in their metadata, a counter increments, and the first envelope of each distinct unsupported
wire version is logged once. Once per distinct value, because a mismatched peer publishing at rate would otherwise
turn a version mismatch into a log flood.

`T` is fixed at compile time, so an envelope whose `item_type` names a different item is refused rather than
converted. Nothing is filtered on the way in: every key of the decoded map reaches `meta_information[0]` at whatever
type it carries, and a vocabulary key at the wrong type is counted rather than dropped — the value's author is in
another process and cannot be told, so dropping the key would erase the only evidence that the peer is misconfigured.

`max_message_bytes` is required and has no default. libzmq's own `ZMQ_MAXMSGSIZE` defaults to no limit, so a source
that did not set it would have the library allocate whatever a peer claims before the block sees a byte. The bound is
pushed into libzmq as well as checked here; the cost of an oversize message is therefore a disconnection, and over a
connection-oriented transport an automatic reconnection, whose loss appears at this end as a `sequence` gap.

Loss is not countable by either end of a ZeroMQ link — a publisher's drops happen inside the library after the send
has already succeeded — so it is reconstructed instead from what arrives. The source tracks the last `sequence` seen
per `source_id`, counts gaps and the packets they imply, and treats a lower value as a producer restart rather than
as loss. Subtracting `nDroppedByBackpressure` from `nPacketsLost` separates what the wire lost from what this graph
was too slow to take. The first envelope from a source is never a gap, because a subscriber has no history and may
legitimately join mid-stream.
)"">;

    PortOut<Packet<T>, Async>                      out;
    PortOut<Packet<std::uint8_t>, Async, Optional> reject;

    Annotated<std::string, "endpoint", Visible, Doc<"libzmq endpoint; required, there is no default">>                                   endpoint{};
    Annotated<bool, "bind", Doc<"bind the endpoint rather than connect to it; a consumer normally connects">>                            bind                = false;
    Annotated<std::string, "pattern", Doc<"'sub' (prefix subscription) or 'pull' (fair-queued)">>                                        pattern             = std::string("sub");
    Annotated<std::string, "topic", Doc<"ZMQ_SUBSCRIBE prefix, 'sub' only; empty accepts everything and refuses it observably">>         topic               = std::string("");
    Annotated<std::uint64_t, "max_message_bytes", Unit<"byte">, Doc<"ZMQ_MAXMSGSIZE and this block's own bound; required, must be > 0">> max_message_bytes   = 0ULL;
    Annotated<gr::Size_t, "queue_messages", Doc<"in-process receive queue depth">>                                                       queue_messages      = 1024U;
    Annotated<std::uint64_t, "queue_bytes", Unit<"byte">, Doc<"in-process receive queue size">>                                          queue_bytes         = 16777216ULL;
    Annotated<gr::Size_t, "recv_hwm", Doc<"ZMQ_RCVHWM, libzmq's own inbound queue bound">>                                               recv_hwm            = 16U;
    Annotated<gr::Size_t, "max_reject_bytes", Unit<"byte">, Doc<"raw bytes of a refused envelope kept for inspection">>                  max_reject_bytes    = 256U;
    Annotated<gr::Size_t, "max_tracked_sources", Doc<"distinct source_id values whose sequence is tracked">>                             max_tracked_sources = 8U;
    Annotated<std::int32_t, "linger_ms", Unit<"ms">, Doc<"ZMQ_LINGER; 0 discards pending messages immediately on close">>                linger_ms           = 0;

    GR_MAKE_REFLECTABLE(ZmqPacketSource, out, reject, endpoint, bind, pattern, topic, max_message_bytes, queue_messages, queue_bytes, recv_hwm, max_reject_bytes, max_tracked_sources, linger_ms);

    std::uint64_t nEnvelopesReceived = 0ULL; ///< messages taken off the socket
    std::uint64_t nPacketsPublished  = 0ULL; ///< packets published on out
    std::uint64_t nBytesReceived     = 0ULL; ///< bytes taken off the socket, frames 1 to 3

    std::uint64_t nBadFrameCount       = 0ULL; ///< a message that was not exactly four parts
    std::uint64_t nShortHeader         = 0ULL; ///< frame 1 shorter than a header
    std::uint64_t nBadMagic            = 0ULL; ///< foreign traffic, which the permissive default topic lets through so it can be counted
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
    std::uint64_t nLengthMismatch      = 0ULL; ///< the header's lengths disagree with the frame sizes libzmq reported
    std::uint64_t nOverMax             = 0ULL; ///< a message that got through libzmq's bound but claims more
    std::uint64_t nBadMetadata         = 0ULL; ///< frame 2 did not parse

    std::uint64_t nSequenceGaps          = 0ULL; ///< sequence discontinuities at a tracked source
    std::uint64_t nPacketsLost           = 0ULL; ///< the total size of those gaps
    std::uint64_t nSequenceResets        = 0ULL; ///< a sequence at or below the last seen from that source
    std::uint64_t nSourcesUntracked      = 0ULL; ///< a distinct source_id beyond max_tracked_sources
    std::uint64_t nDroppedByBackpressure = 0ULL; ///< envelopes discarded because the in-process queue was full
    std::uint64_t nMetaKeysMistyped      = 0ULL; ///< vocabulary keys whose type disagrees with the declaration
    std::uint64_t nTimestampsCarried     = 0ULL; ///< packet_timestamp values consumed into the carrier field

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

    std::string            _endpoint{};
    std::string            _topic{};
    detail::zmqio::Pattern _pattern           = detail::zmqio::Pattern::Sub;
    bool                   _bind              = false;
    std::size_t            _queueMessages     = 1024UZ;
    std::uint64_t          _queueBytes        = 16777216ULL;
    std::int32_t           _recvHwm           = 16;
    std::int32_t           _lingerMs          = 0;
    std::uint64_t          _maxMessageBytes   = 0ULL;
    std::size_t            _maxTrackedSources = 8UZ;
    bool                   _socketOpen        = false;
    std::size_t            _maxRejectBytes    = 256UZ;

    struct IoThreadGuard { // must be last member — destroyed first, so the reader is gone before the queue it fills
        ZmqPacketSource* self;
        explicit IoThreadGuard(ZmqPacketSource* owner) noexcept : self(owner) {}
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
            _openFailure.clear();
            _queue.clear();
            _queuedBytes = 0ULL;
        }
        _sources.clear();
        _loggedVersions.clear();
        _arrivals = 0ULL;
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
        std::ignore = detail::zmqio::receivePatternFromName(pattern.value);
        if (max_message_bytes.value == 0ULL) {
            throw gr::exception("max_message_bytes is 0 or unset; libzmq's own default is no limit, so a source must state a bound before a peer's claimed length sizes anything");
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
        if (topic.value != _topic) {
            refuse("topic");
        }
        if (bind.value != _bind) {
            refuse("bind");
        }
        if (detail::zmqio::receivePatternFromName(pattern.value) != _pattern) {
            refuse("pattern");
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
        if (static_cast<std::int32_t>(recv_hwm.value) != _recvHwm) {
            refuse("recv_hwm");
        }
        if (static_cast<std::size_t>(max_tracked_sources.value) != _maxTrackedSources) {
            refuse("max_tracked_sources");
        }
        if (linger_ms.value != _lingerMs) {
            refuse("linger_ms");
        }
    }

    void freezeSocketSettings() {
        _endpoint          = endpoint.value;
        _topic             = topic.value;
        _bind              = bind.value;
        _pattern           = detail::zmqio::receivePatternFromName(pattern.value);
        _maxMessageBytes   = max_message_bytes.value;
        _queueMessages     = static_cast<std::size_t>(queue_messages.value);
        _queueBytes        = queue_bytes.value;
        _recvHwm           = static_cast<std::int32_t>(recv_hwm.value);
        _lingerMs          = linger_ms.value;
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
        append("bad frame count", nBadFrameCount);
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
        append("metadata keys mistyped", nMetaKeysMistyped);
        append("timestamps carried", nTimestampsCarried);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::network::ZmqPacketSource '{}': {}", this->name, report);
        }
    }

    void ioReadLoop() {
        thread_pool::thread::setThreadName(std::format("zmqpktsrc:{}", this->name.value));
        std::optional<zmq::context_t> context;
        std::optional<zmq::socket_t>  socket;
        std::string                   failure;
        try {
            context.emplace(1);
            socket.emplace(*context, _pattern == detail::zmqio::Pattern::Sub ? zmq::socket_type::sub : zmq::socket_type::pull);
            // the bound goes into libzmq as well as into this block, so an oversize message is refused before the
            // library allocates for it; the peer that sent one is disconnected, and the loss reads as a sequence gap
            socket->set(zmq::sockopt::maxmsgsize, static_cast<std::int64_t>(_maxMessageBytes));
            socket->set(zmq::sockopt::rcvhwm, _recvHwm);
            socket->set(zmq::sockopt::rcvtimeo, 100); // a bounded wait keeps stop() responsive without busy-spinning
            socket->set(zmq::sockopt::linger, _lingerMs);
            if (_pattern == detail::zmqio::Pattern::Sub) {
                socket->set(zmq::sockopt::subscribe, _topic);
            }
            if (_bind) {
                socket->bind(_endpoint);
            } else {
                socket->connect(_endpoint);
            }
        } catch (const zmq::error_t& error) {
            failure = std::format("cannot {} '{}': {}{}", _bind ? "bind" : "connect to", _endpoint, error.what(), detail::zmqio::endpointHint(_endpoint));
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
        gr::atomic_ref(_ioThreadDone).store_release(true);
        gr::atomic_ref(_ioThreadDone).notify_all();
    }

    void receiveUntilStopped(zmq::socket_t& socket) {
        while (true) {
            {
                std::lock_guard lock(_mutex);
                if (_stopRequested) {
                    return;
                }
            }
            std::vector<zmq::message_t> parts;
            std::size_t                 partCount = 0UZ;
            std::uint64_t               partBytes = 0ULL;
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
                    _readerFailed = true;
                    std::println(stderr, "gr::blocks::network::ZmqPacketSource '{}': receive failed on '{}': {}", this->name, _endpoint, error.what());
                }
                return;
            }
            nEnvelopesReceived += 1ULL;
            nBytesReceived += partBytes;
            handleMessage(parts, partCount, partBytes);
        }
    }

    /// @brief Validate one message and enqueue either the packet it carries or the refusal it earned.
    void handleMessage(std::span<zmq::message_t> parts, std::size_t partCount, std::uint64_t partBytes) {
        if (partCount != 4UZ) {
            ++nBadFrameCount;
            enqueueRefusal(parts, partCount, partBytes, "frame_count");
            return;
        }
        const std::span<const std::uint8_t> headerFrame(static_cast<const std::uint8_t*>(parts[1UZ].data()), parts[1UZ].size());
        if (headerFrame.size() < gr::network::kHeaderBytesV1) {
            ++nShortHeader;
            enqueueRefusal(parts, partCount, partBytes, "short_header");
            return;
        }

        const auto header = gr::network::decodeHeader(headerFrame);
        if (!header.has_value()) {
            countKernelRefusal(header.error(), headerFrame);
            enqueueRefusal(parts, partCount, partBytes, gr::network::discardReason(header.error()));
            return;
        }
        if (header->item_type != gr::network::kItemTypeCode<T>) {
            ++nItemTypeMismatch;
            enqueueRefusal(parts, partCount, partBytes, "item_type_mismatch");
            return;
        }
        if (header->meta_bytes != parts[2UZ].size() || header->payload_bytes != parts[3UZ].size() || headerFrame.size() != header->header_bytes) {
            ++nLengthMismatch; // the header's lengths and libzmq's are two statements about one message
            enqueueRefusal(parts, partCount, partBytes, "length_mismatch");
            return;
        }
        if (static_cast<std::uint64_t>(header->header_bytes) + static_cast<std::uint64_t>(header->meta_bytes) + static_cast<std::uint64_t>(header->payload_bytes) > _maxMessageBytes) {
            ++nOverMax;
            enqueueRefusal(parts, partCount, partBytes, "over_max");
            return;
        }

        property_map map;
        if (header->meta_bytes != 0U) { // the empty map short-circuits rather than asking what an empty document means
            const std::string_view text(static_cast<const char*>(parts[2UZ].data()), parts[2UZ].size());
            const auto             parsed = pmt::yaml::deserialize(text);
            if (!parsed.has_value()) {
                ++nBadMetadata;
                std::println(stderr, "gr::blocks::network::ZmqPacketSource '{}': metadata frame line {} column {}: {}", this->name, parsed.error().line, parsed.error().column, parsed.error().message);
                enqueueRefusal(parts, partCount, partBytes, "bad_metadata");
                return;
            }
            map = *parsed;
        }
        nMetaKeysMistyped += detail::zmqio::countMistypedKeys(map);

        Incoming arrival;
        arrival.accepted.signal_values.resize(header->item_count);
        if (header->payload_bytes != 0U) {
            std::memcpy(arrival.accepted.signal_values.data(), parts[3UZ].data(), header->payload_bytes);
        }
        if (const auto stamp = map.find(detail::zmqio::kTimestampKey); stamp != map.end()) {
            if (const std::int64_t* value = stamp->second.get_if<std::int64_t>(); value != nullptr) {
                arrival.accepted.timestamp = *value;
                ++nTimestampsCarried;
            }
            map.erase(stamp->first); // consumed, so the carrier field is the value's only spelling after the crossing
        }
        trackSequence(map);
        arrival.accepted.meta_information.resize(1UZ);
        arrival.accepted.meta_information[0UZ] = std::move(map);
        arrival.bytes                          = partBytes;
        enqueue(std::move(arrival));
    }

    void countKernelRefusal(gr::network::EnvelopeError error, std::span<const std::uint8_t> headerFrame) {
        using gr::network::EnvelopeError;
        switch (error) {
        case EnvelopeError::BadMagic: ++nBadMagic; return;
        case EnvelopeError::BadVersion:
        case EnvelopeError::FutureVersion: {
            ++nRefusedVersion; // a peer at the wrong version is one fault however it is spelled
            const std::uint16_t version = static_cast<std::uint16_t>(static_cast<std::uint16_t>(headerFrame[4UZ]) | static_cast<std::uint16_t>(static_cast<std::uint16_t>(headerFrame[5UZ]) << 8U));
            if (std::ranges::find(_loggedVersions, version) == _loggedVersions.end()) {
                _loggedVersions.push_back(version); // once per distinct value: a mismatched peer at rate would flood
                std::println(stderr, "gr::blocks::network::ZmqPacketSource '{}': refusing wire version {} on '{}'; this reader implements version {}", this->name, version, _endpoint, gr::network::kWireVersion);
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

    /// @brief Publish the raw bytes of a refused message so the failure can be read without a packet capture.
    ///
    /// `max_reject_bytes` bounds the copy, so a hostile message cannot be pulled whole into the graph by the very
    /// path that refused it. The topic frame is excluded because it is the transport's and no part of the envelope.
    void enqueueRefusal(std::span<zmq::message_t> parts, std::size_t partCount, std::uint64_t partBytes, std::string_view reason) {
        Incoming arrival;
        arrival.isRefusal = true;
        arrival.bytes     = partBytes;

        std::vector<std::uint8_t>& bytes = arrival.refused.signal_values;
        bytes.reserve(std::min(static_cast<std::size_t>(partBytes), _maxRejectBytes));
        for (std::size_t i = 1UZ; i < parts.size() && bytes.size() < _maxRejectBytes; ++i) {
            const std::span<const std::uint8_t> frame(static_cast<const std::uint8_t*>(parts[i].data()), parts[i].size());
            const std::size_t                   room = _maxRejectBytes - bytes.size();
            const auto                          take = frame.first(std::min(room, frame.size()));
            bytes.insert(bytes.end(), take.begin(), take.end());
        }

        arrival.refused.meta_information.resize(1UZ);
        property_map& map = arrival.refused.meta_information[0UZ];
        map.insert_or_assign(property_map::key_type("discard_reason"), pmt::Value(std::string(reason)));
        map.insert_or_assign(property_map::key_type("envelope_bytes_total"), pmt::Value(partBytes));
        const std::uint64_t kept = bytes.size();
        map.insert_or_assign(property_map::key_type("envelope_bytes_kept"), pmt::Value(kept));
        map.insert_or_assign(property_map::key_type("envelope_frames"), pmt::Value(static_cast<gr::Size_t>(partCount)));
        enqueue(std::move(arrival));
    }

    void enqueue(Incoming&& arrival) {
        {
            std::lock_guard lock(_mutex);
            while (_queue.size() >= _queueMessages || (!_queue.empty() && _queuedBytes + arrival.bytes > _queueBytes)) {
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
        // the first envelope from a source establishes the baseline and is never a gap: a subscriber has no history
    }
};

} // namespace gr::blocks::network

#endif // GNURADIO_ZMQPACKETIO_HPP
