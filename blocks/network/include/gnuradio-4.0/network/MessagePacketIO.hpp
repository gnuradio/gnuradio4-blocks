#ifndef GNURADIO_NETWORK_MESSAGEPACKETIO_HPP
#define GNURADIO_NETWORK_MESSAGEPACKETIO_HPP

#include <array>
#include <cstdint>
#include <deque>
#include <format>
#include <limits>
#include <mutex>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <zmq.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/YamlPmt.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/network/PacketEnvelope.hpp>
#include <gnuradio-4.0/network/ZmqEnvelopeIo.hpp>
#include <gnuradio-4.0/network/ZmqTransport.hpp>

/**
 * @brief The message plane carried across a process boundary, in the module's own envelope.
 *
 * The stream pair next door carries samples and the tags standing in them; this one carries the other half of what a
 * flowgraph is made of. A `gr::Message` is what a host application uses to read a block's settings, change them,
 * subscribe to a property, or hear that something failed, and none of it reaches a block in another process. These
 * two blocks put that plane on a wire: the sink publishes what arrives on its `msgIn`, the source emits what arrives
 * on its socket into its own graph's plane, and between them the envelope is the module's, unchanged and at wire
 * version 1.
 *
 * A message has no payload items, so the packet carries none: `item_count` is zero, which the envelope explicitly
 * allows, and the whole message is the metadata frame — one nested map under `message`, with `sequence` beside it as
 * on every other packet this module sends. That is what keeps one reader able to read all three pairs.
 *
 * Loss is announced the way the stream pair announces it, in the only vocabulary this plane has: a `sequence` gap
 * becomes a message of its own, on a named endpoint, carrying how many were lost and where. A message plane that
 * silently drops a `Set` is worse than one that admits it, because the block on the far side then holds a setting
 * nobody asked for and nothing says so.
 */
namespace gr::blocks::network {

namespace detail::messagepacket {

/// @brief The nested map the whole message rides in, so that no field of it can collide with a vocabulary key.
///
/// `protocol` is the case that forces this: the record-metadata vocabulary declares `protocol` as the name of a
/// framing protocol ("ax25"), and `gr::Message::protocol` is the Majordomo protocol version. One key, two meanings,
/// and neither could be read safely. Nesting costs one map and settles it for every field at once.
inline constexpr std::string_view kMessageKey = "message";

// The fields of `gr::Message`, spelled as this tree spells wire keys rather than as C++ spells members.
inline constexpr std::string_view kProtocolKey        = "protocol";
inline constexpr std::string_view kCommandKey         = "command";
inline constexpr std::string_view kServiceNameKey     = "service_name";
inline constexpr std::string_view kClientRequestIdKey = "client_request_id";
inline constexpr std::string_view kEndpointKey        = "endpoint";
inline constexpr std::string_view kDataKey            = "data";
inline constexpr std::string_view kErrorKey           = "error";
inline constexpr std::string_view kRbacKey            = "rbac";

/// @brief The endpoint a receiving block announces lost messages on.
inline constexpr std::string_view kGapEndpoint = "gr::blocks::network::MessagePacketSource::gap";

/// @brief How many messages the `sequence` discontinuity accounts for, in a gap message's payload.
inline constexpr std::string_view kMessagesLostKey = "messages_lost";

/// @brief The highest `gr::message::Command` this file knows, which is what bounds a value that came off a wire.
inline constexpr std::uint8_t kMaxCommand = static_cast<std::uint8_t>(gr::message::Command::Heartbeat);

/// @brief One message as the metadata frame states it, with the sequence the sender numbered it by.
[[nodiscard]] inline property_map encode(const Message& message, std::uint64_t sequence) {
    property_map fields;
    const auto   put = [&fields](std::string_view key, std::string value) {
        if (!value.empty()) { // an absent key reads as the empty string, which is what it would have carried
            fields.insert_or_assign(property_map::key_type(key), pmt::Value(std::move(value)));
        }
    };
    put(kProtocolKey, message.protocol);
    fields.insert_or_assign(property_map::key_type(kCommandKey), pmt::Value(static_cast<std::uint8_t>(message.cmd)));
    put(kServiceNameKey, message.serviceName);
    put(kClientRequestIdKey, message.clientRequestID);
    put(kEndpointKey, message.endpoint);
    put(kRbacKey, message.rbac);
    if (message.data.has_value()) {
        fields.insert_or_assign(property_map::key_type(kDataKey), pmt::Value(message.data.value()));
    } else {
        // the error's whole statement as one string: its time, its file and line inside the peer, its message and the
        // function it came from. A std::source_location cannot be reconstructed here, and one manufactured here would
        // name this process's file as if it were the peer's, so the peer's is kept where it reads as text.
        fields.insert_or_assign(property_map::key_type(kErrorKey), pmt::Value(std::format("{:t}", message.data.error())));
    }

    property_map map;
    map.insert_or_assign(property_map::key_type("sequence"), pmt::Value(sequence));
    map.insert_or_assign(property_map::key_type(kMessageKey), pmt::Value(std::move(fields)));
    return map;
}

/// @brief The message a metadata frame states, or nothing when it does not state one this reader can trust.
///
/// The command is the one field that is validated rather than copied: it is an enumeration whose numbering is the
/// Majordomo protocol's, so a value outside it names no command and a block acting on it would be acting on a guess.
[[nodiscard]] inline std::optional<Message> decode(const property_map& fields) {
    const auto text = [&fields](std::string_view key) -> std::string {
        const auto entry = fields.find(property_map::key_type(key));
        if (entry == fields.end()) {
            return {};
        }
        const auto* value = entry->second.get_if<std::pmr::string>();
        return value == nullptr ? std::string{} : std::string(value->begin(), value->end());
    };

    const auto commandEntry = fields.find(property_map::key_type(kCommandKey));
    if (commandEntry == fields.end()) {
        return std::nullopt;
    }
    const std::uint8_t* command = commandEntry->second.get_if<std::uint8_t>();
    if (command == nullptr || *command > kMaxCommand) {
        return std::nullopt;
    }

    Message message;
    message.protocol        = text(kProtocolKey);
    message.cmd             = static_cast<gr::message::Command>(*command);
    message.serviceName     = text(kServiceNameKey);
    message.clientRequestID = text(kClientRequestIdKey);
    message.endpoint        = text(kEndpointKey);
    message.rbac            = text(kRbacKey);
    if (const auto entry = fields.find(property_map::key_type(kErrorKey)); entry != fields.end()) {
        message.data = std::unexpected(Error(text(kErrorKey)));
        return message;
    }
    property_map data;
    if (const auto entry = fields.find(property_map::key_type(kDataKey)); entry != fields.end()) {
        if (const property_map* value = entry->second.get_if<property_map>(); value != nullptr) {
            data = *value;
        } else {
            return std::nullopt; // `data` at another type is not a payload, and an empty one would be a lie
        }
    }
    message.data = std::move(data);
    return message;
}

/// @brief The envelope a message packet takes: a zero-item `uint8` payload and the message in the metadata frame.
[[nodiscard]] inline zmqenvelope::Outgoing envelopeOf(const property_map& map) {
    zmqenvelope::Outgoing envelope;
    envelope.metadata = pmt::yaml::serialize(map);

    gr::network::EnvelopeHeader header;
    header.item_type     = gr::network::kItemTypeCode<std::uint8_t>;
    header.item_size     = 1U;
    header.item_count    = 0U;
    header.payload_bytes = 0U;
    header.meta_bytes    = static_cast<std::uint32_t>(envelope.metadata.size());
    envelope.header      = gr::network::encodeHeader(header);
    return envelope;
}

} // namespace detail::messagepacket

GR_REGISTER_BLOCK("gr::blocks::network::MessagePacketSink", gr::blocks::network::MessagePacketSink)

struct MessagePacketSink : Block<MessagePacketSink> {
    using Description = Doc<R""(
@brief Publishes the messages reaching this block's message plane as versioned four-frame ZeroMQ messages.

A scheduler hands every message bound for its blocks to all of them and lets each one decide; this block decides to
put them on a wire. What it publishes is therefore the graph's whole inbound message plane — a `Set` on some block's
settings, a `Subscribe`, a `Get` — which is what makes the far half of a split graph reachable at all.

The one exception is a message that names this block, by its `name` or its `unique_name`. Those are how a host
application reads and changes the sink's own settings, so they are answered here rather than forwarded, and the
block stays as controllable as any other. A message with an empty `serviceName` is a broadcast, so it is both
answered locally and forwarded: the far side's blocks are addressed by it too.

The packet carries no payload items. `item_count` is zero, which the envelope allows, and the message is the whole
of the metadata frame: one nested map under `message` holding the seven fields `gr::Message` declares, with
`sequence` beside it exactly as on the module's other packets. Nesting is what keeps `protocol` — a Majordomo
version here and a framing protocol in the record vocabulary — from being one key with two meanings.

An error carried instead of a payload crosses as text, under `error`, formatted with its time, its file and line
inside the peer, its message and its function. A `std::source_location` cannot be reconstructed in another process,
and one manufactured at the far end would name the receiving file as if it were the sender's.

The transport is the module's, with the same settings, patterns and defaults as the other two pairs.
)"">;

    Annotated<std::string, "endpoint", Visible, Doc<"libzmq endpoint, e.g. tcp://127.0.0.1:5555; required, there is no default">> endpoint{};
    Annotated<bool, "bind", Doc<"bind the endpoint rather than connect to it; the producer is normally the stable end">>          bind              = true;
    Annotated<std::string, "pattern", Doc<"'pub' (fan out) or 'push' (round-robin, lossless, one consumer)">>                     pattern           = std::string("pub");
    Annotated<std::string, "topic", Doc<"frame 0, the subscription prefix; empty publishes to every subscriber">>                 topic             = std::string("");
    Annotated<std::string, "overflow", Doc<"'drop_oldest' or 'backpressure', applied when the in-process send queue is full">>    overflow          = std::string("drop_oldest");
    Annotated<gr::Size_t, "queue_messages", Doc<"in-process send queue depth">>                                                   queue_messages    = 1024U;
    Annotated<std::uint64_t, "queue_bytes", Unit<"byte">, Doc<"in-process send queue size">>                                      queue_bytes       = 16777216ULL;
    Annotated<gr::Size_t, "send_hwm", Doc<"ZMQ_SNDHWM; libzmq's own 1000 is up to 64 MiB of buffer this graph cannot see">>       send_hwm          = 16U;
    Annotated<std::uint64_t, "max_message_bytes", Unit<"byte">, Doc<"a message whose envelope would exceed this is refused">>     max_message_bytes = 16777216ULL;
    Annotated<std::int32_t, "linger_ms", Unit<"ms">, Doc<"ZMQ_LINGER; 0 discards pending messages immediately on close">>         linger_ms         = 0;

    GR_MAKE_REFLECTABLE(MessagePacketSink, endpoint, bind, pattern, topic, overflow, queue_messages, queue_bytes, send_hwm, max_message_bytes, linger_ms);

    /// @brief The block's counters and its transport's, as one set.
    struct Counters {
        std::uint64_t packetsSent       = 0ULL;
        std::uint64_t bytesSent         = 0ULL;
        std::uint64_t messagesForwarded = 0ULL;
        std::uint64_t messagesKeptLocal = 0ULL; ///< messages addressed to this block by name, answered rather than sent
        std::uint64_t messagesRejected  = 0ULL; ///< messages whose envelope would exceed max_message_bytes
        std::uint64_t overQueueBytes    = 0ULL; ///< messages refused because one envelope exceeds queue_bytes
        std::uint64_t droppedOnOverflow = 0ULL;
        std::uint64_t droppedAtStop     = 0ULL;
        std::uint64_t sendErrors        = 0ULL;
    };

    std::uint64_t nMessagesForwarded = 0ULL; ///< messages handed to the transport
    std::uint64_t nMessagesKeptLocal = 0ULL; ///< messages this block answered instead of forwarding
    std::uint64_t nMessagesRejected  = 0ULL; ///< messages refused for exceeding max_message_bytes
    std::uint64_t nOverQueueBytes    = 0ULL; ///< messages refused because one envelope exceeds queue_bytes

    std::uint64_t _sequence        = 0ULL; ///< messages this sink has published
    bool          _socketOpen      = false;
    bool          _backpressure    = false;
    std::uint64_t _maxMessageBytes = 16777216ULL;
    std::uint64_t _queueBytes      = 16777216ULL; ///< frozen beside the transport's copy, which start() configures

    detail::zmqenvelope::SocketConfig _frozen{}; ///< the socket settings, read once when the socket opens
    detail::zmqenvelope::SendQueue    _sender{}; ///< joins its own thread however the block dies

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() {
        validate();
        _frozen     = frozenSocketConfig();
        _queueBytes = queue_bytes.value;
        _sender.configure(static_cast<std::size_t>(queue_messages.value), _queueBytes, _backpressure);
        _sequence = 0ULL; // a restarted sink restarts its stream, which the far end reads as a producer reset
        _sender.start(_frozen, this->name.value);
        _socketOpen = true;
    }

    void stop() {
        _sender.stop();
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

    [[nodiscard]] Counters counters() const {
        const auto transport = _sender.counters();
        return {.packetsSent = transport.packetsSent, .bytesSent = transport.bytesSent, .messagesForwarded = nMessagesForwarded, .messagesKeptLocal = nMessagesKeptLocal, .messagesRejected = nMessagesRejected, .overQueueBytes = nOverQueueBytes, .droppedOnOverflow = transport.droppedOnOverflow, .droppedAtStop = transport.droppedAtStop, .sendErrors = transport.sendErrors};
    }

    /// @brief What a block with no stream ports does when the scheduler calls it: nothing, and says so.
    ///
    /// Everything these two blocks do happens on the message plane — in `processMessages` and in
    /// `processScheduledMessages`, which the scheduler calls on every block on every iteration — so there is no
    /// sample work to do. The framework's own `work()` cannot stand in: with no stream ports its dispatch reaches
    /// the branch that requires a `processOne` or a `processBulk`, and neither has anything to process here.
    /// `INSUFFICIENT_INPUT_ITEMS` is the honest status for a block whose input is a socket, and it is what keeps a
    /// scheduler from counting an idle plane as progress; `DONE` on shutdown is what lets the job end.
    [[nodiscard]] work::Result work(std::size_t requestedWork = std::numeric_limits<std::size_t>::max()) noexcept {
        if (lifecycle::isShuttingDown(this->state())) {
            return {requestedWork, 0UZ, work::Status::DONE};
        }
        return {requestedWork, 0UZ, work::Status::INSUFFICIENT_INPUT_ITEMS};
    }

    /// @brief Answer what is addressed to this block, publish everything else.
    ///
    /// Naming the inherited handler is what keeps the sink's own settings readable and writable from its own graph;
    /// it filters by `serviceName` itself, so it is handed the whole span and forwarding is decided separately.
    void processMessages(const MsgPortInBuiltin& port, std::span<const Message> messages) {
        gr::BlockBase::processMessages(port, messages);
        for (const Message& message : messages) {
            if (!message.serviceName.empty() && (message.serviceName == this->unique_name || message.serviceName == this->name.value)) {
                ++nMessagesKeptLocal;
                continue;
            }
            publish(message);
        }
    }

private:
    void validate() {
        if (endpoint.value.empty()) {
            throw gr::exception("endpoint is empty; a transport sink has no default endpoint and will not bind to nothing");
        }
        std::ignore = detail::zmqio::sendPatternFromName(pattern.value);
        if (overflow.value != "drop_oldest" && overflow.value != "backpressure") {
            throw gr::exception(std::format("overflow is '{}'; it must be 'drop_oldest' (shed the stalest envelope, count it) or 'backpressure' (leave the message on the plane)", overflow.value));
        }
        if (queue_messages.value == 0U) {
            throw gr::exception("queue_messages is 0; the in-process send queue must hold at least one envelope");
        }
        if (queue_bytes.value == 0ULL) {
            throw gr::exception("queue_bytes is 0; the in-process send queue must hold at least one envelope");
        }
        if (max_message_bytes.value == 0ULL) {
            throw gr::exception("max_message_bytes is 0; every message would be refused");
        }
    }

    void refuseFrozenChange() const {
        const auto refuse = [](std::string_view setting) { throw gr::exception(std::format("setting '{}' is read once when the socket opens and cannot change while the block is running; rebuild the graph instead", setting)); };
        const auto wanted = frozenSocketConfig();
        if (wanted.endpoint != _frozen.endpoint) {
            refuse("endpoint");
        }
        if (wanted.topic != _frozen.topic) {
            refuse("topic");
        }
        if (wanted.bind != _frozen.bind) {
            refuse("bind");
        }
        if (wanted.pattern != _frozen.pattern) {
            refuse("pattern");
        }
        if (wanted.hwm != _frozen.hwm) {
            refuse("send_hwm");
        }
        if (wanted.lingerMs != _frozen.lingerMs) {
            refuse("linger_ms");
        }
    }

    [[nodiscard]] detail::zmqenvelope::SocketConfig frozenSocketConfig() const { return {.endpoint = endpoint.value, .topic = topic.value, .pattern = detail::zmqio::sendPatternFromName(pattern.value), .bind = bind.value, .hwm = static_cast<std::int32_t>(send_hwm.value), .lingerMs = linger_ms.value, .maxMessageBytes = 0ULL}; }

    void publish(const Message& message) {
        detail::zmqenvelope::Outgoing envelope = detail::messagepacket::envelopeOf(detail::messagepacket::encode(message, _sequence));
        if (envelope.bytes() > _maxMessageBytes) {
            ++nMessagesRejected;
            ++_sequence; // the far end reads the hole as one lost message, which is exactly what happened
            return;
        }
        // a queue cannot shed its way to room for an envelope larger than the whole of it, so such a message is
        // refused here rather than admitted over the bound the graph set
        if (envelope.bytes() > _queueBytes) {
            ++nOverQueueBytes;
            ++_sequence;
            return;
        }
        if (!_sender.enqueue(std::move(envelope))) {
            return; // backpressure: the message stays unsent and is counted by the plane's own dropped-message counter
        }
        ++nMessagesForwarded;
        ++_sequence;
    }

    void report() const {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        const Counters counted = counters();
        append("packets sent", counted.packetsSent);
        append("bytes sent", counted.bytesSent);
        append("messages forwarded", counted.messagesForwarded);
        append("messages kept local", counted.messagesKeptLocal);
        append("messages rejected", counted.messagesRejected);
        append("over queue bytes", counted.overQueueBytes);
        append("dropped on overflow", counted.droppedOnOverflow);
        append("dropped at stop", counted.droppedAtStop);
        append("send errors", counted.sendErrors);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::network::MessagePacketSink '{}': {}", this->name, report);
        }
    }
};

GR_REGISTER_BLOCK("gr::blocks::network::MessagePacketSource", gr::blocks::network::MessagePacketSource)

struct MessagePacketSource : Block<MessagePacketSource> {
    using Description = Doc<R""(
@brief Receives message packets and emits each one into this graph's message plane.

This is the far half of a split graph's control path. Every message the peer's `MessagePacketSink` published is
rebuilt field for field and written to this block's `msgOut`, which the scheduler forwards to its own `msgOut` —
the route a host application reads a graph through.

**A gap is announced, never silent.** A `sequence` discontinuity means the wire or the in-process queue lost
messages, and a lost `Set` leaves a block holding a setting nobody asked for. One message of its own is emitted
before the message that followed the gap: `Notify`, on the endpoint
`gr::blocks::network::MessagePacketSource::gap`, from this block's `unique_name`, carrying `messages_lost` and the
`sequence` of the first message that went missing. A `sequence` at or below the last seen is a producer restart,
counted as such and not announced as loss.

A packet is refused for one named reason and counted: the envelope kernel's own header refusals by its spelling,
and beyond them a message that is not four frames, whose frame sizes disagree with the header, that carries payload
items where this pair carries none, whose metadata does not parse, that states no `sequence`, or whose `message` map
states no command this reader knows. The command is the one field validated rather than copied — its numbering is
the Majordomo protocol's, and a block acting on a value outside it would be acting on a guess.

Decoding runs on the block's own reader thread, so a flood of malformed packets costs that thread and never the
scheduler. The queue the plane drains is bounded and sheds its oldest message when the plane cannot keep up, which
the far end's next packet then reports as a gap like any other loss.
)"">;

    Annotated<std::string, "endpoint", Visible, Doc<"libzmq endpoint; required, there is no default">>                                   endpoint{};
    Annotated<bool, "bind", Doc<"bind the endpoint rather than connect to it; a consumer normally connects">>                            bind              = false;
    Annotated<std::string, "pattern", Doc<"'sub' (prefix subscription) or 'pull' (fair-queued)">>                                        pattern           = std::string("sub");
    Annotated<std::string, "topic", Doc<"ZMQ_SUBSCRIBE prefix, 'sub' only; empty accepts everything and refuses it observably">>         topic             = std::string("");
    Annotated<std::uint64_t, "max_message_bytes", Unit<"byte">, Doc<"ZMQ_MAXMSGSIZE and this block's own bound; required, must be > 0">> max_message_bytes = 0ULL;
    Annotated<gr::Size_t, "queue_messages", Doc<"in-process receive queue depth">>                                                       queue_messages    = 1024U;
    Annotated<std::uint64_t, "queue_bytes", Unit<"byte">, Doc<"in-process receive queue size">>                                          queue_bytes       = 16777216ULL;
    Annotated<gr::Size_t, "recv_hwm", Doc<"ZMQ_RCVHWM, libzmq's own inbound queue bound">>                                               recv_hwm          = 16U;
    Annotated<std::int32_t, "linger_ms", Unit<"ms">, Doc<"ZMQ_LINGER; 0 discards pending messages immediately on close">>                linger_ms         = 0;

    GR_MAKE_REFLECTABLE(MessagePacketSource, endpoint, bind, pattern, topic, max_message_bytes, queue_messages, queue_bytes, recv_hwm, linger_ms);

    /// @brief The counters as one set, taken under the lock the reader thread writes most of them behind.
    struct Counters {
        std::uint64_t envelopesReceived     = 0ULL;
        std::uint64_t bytesReceived         = 0ULL;
        std::uint64_t messagesEmitted       = 0ULL;
        std::uint64_t gapsAnnounced         = 0ULL;
        std::uint64_t messagesLost          = 0ULL;
        std::uint64_t sequenceResets        = 0ULL;
        std::uint64_t messagesRefused       = 0ULL; ///< the sum of every refusal reason
        std::uint64_t droppedByBackpressure = 0ULL;
        std::uint64_t overQueueBytes        = 0ULL; ///< arrivals discarded because one alone exceeds queue_bytes
    };

    /// @brief One counter per envelope-kernel refusal, indexed by the error, reported under its own `discard_reason`.
    std::array<std::uint64_t, 11UZ> nHeaderRefusals{};

    std::uint64_t nMessagesEmitted   = 0ULL;
    std::uint64_t nGapsAnnounced     = 0ULL;
    std::uint64_t nMessagesLost      = 0ULL;
    std::uint64_t nSequenceResets    = 0ULL;
    std::uint64_t nBadFrameCount     = 0ULL; ///< a message that was not exactly four parts
    std::uint64_t nShortHeader       = 0ULL; ///< frame 1 shorter than a header
    std::uint64_t nItemTypeMismatch  = 0ULL; ///< a well-formed envelope whose payload is not this pair's
    std::uint64_t nUnexpectedPayload = 0ULL; ///< payload items where a message packet carries none
    std::uint64_t nLengthMismatch    = 0ULL; ///< the header's lengths disagree with the frame sizes libzmq reported
    std::uint64_t nOverMax           = 0ULL; ///< a message that got through libzmq's bound but claims more
    std::uint64_t nBadMetadata       = 0ULL; ///< frame 2 did not parse
    std::uint64_t nMissingSequence   = 0ULL; ///< no `sequence`; the packet cannot be placed in the stream
    std::uint64_t nMalformedMessage  = 0ULL; ///< no `message` map, or no command this reader knows

    std::uint64_t nDroppedByBackpressure = 0ULL; ///< messages the in-process queue shed
    std::uint64_t nOverQueueBytes        = 0ULL; ///< arrivals discarded because one alone exceeds queue_bytes

    /// @brief One decoded message, everything about it established on the reader thread.
    struct Arrival {
        Message       message{};
        std::uint64_t sequence = 0ULL;
        std::uint64_t bytes    = 0ULL;
    };

    mutable std::mutex         _mutex; ///< guards the queue and the counters the reader thread writes
    std::deque<Arrival>        _queue;
    std::uint64_t              _queuedBytes = 0ULL;
    std::vector<std::uint16_t> _loggedVersions{}; ///< one log line per distinct unsupported version

    // the emitting side's state, touched only by processScheduledMessages
    std::optional<Arrival> _current{};    ///< the message being emitted, which may wait for room on the plane
    std::optional<Message> _pendingGap{}; ///< the announcement that must go out before it
    bool                   _haveHistory  = false;
    std::uint64_t          _lastSequence = 0ULL;

    bool                              _socketOpen    = false;
    std::size_t                       _queueMessages = 1024UZ; ///< frozen with the socket, since the reader reads them
    std::uint64_t                     _queueBytes    = 16777216ULL;
    detail::zmqenvelope::SocketConfig _frozen{};

    /// @brief Must be the last declared member: it is destroyed first, so the reader thread is gone before the queue
    /// and the counters its decoder writes into.
    detail::zmqenvelope::Receiver _receiver{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() {
        validate();
        _frozen        = frozenSocketConfig();
        _queueMessages = static_cast<std::size_t>(queue_messages.value);
        _queueBytes    = queue_bytes.value;
        {
            std::lock_guard lock(_mutex);
            _queue.clear();
            _queuedBytes = 0ULL;
            _loggedVersions.clear();
        }
        _current.reset();
        _pendingGap.reset();
        _haveHistory  = false;
        _lastSequence = 0ULL;
        _receiver.start(_frozen, this->name.value, [this](std::span<zmq::message_t> parts, std::size_t partCount, std::uint64_t bytes) { handleMessage(parts, partCount, bytes); });
        _socketOpen = true;
    }

    void stop() {
        _receiver.stop();
        _socketOpen = false;
        report();
    }

    void rebuild() {
        validate();
        if (_socketOpen) {
            refuseFrozenChange(); // every setting is read once when the socket opens, so nothing is recomputed here
        }
    }

    [[nodiscard]] Counters counters() const {
        std::lock_guard lock(_mutex);
        std::uint64_t   refused = nBadFrameCount + nShortHeader + nItemTypeMismatch + nUnexpectedPayload + nLengthMismatch + nOverMax + nBadMetadata + nMissingSequence + nMalformedMessage;
        for (const std::uint64_t count : nHeaderRefusals) {
            refused += count;
        }
        return {.envelopesReceived = _receiver.messagesReceived(), .bytesReceived = _receiver.bytesReceived(), .messagesEmitted = nMessagesEmitted, .gapsAnnounced = nGapsAnnounced, .messagesLost = nMessagesLost, .sequenceResets = nSequenceResets, .messagesRefused = refused, .droppedByBackpressure = nDroppedByBackpressure, .overQueueBytes = nOverQueueBytes};
    }

    /// @brief What a block with no stream ports does when the scheduler calls it: nothing, and says so.
    ///
    /// Everything these two blocks do happens on the message plane — in `processMessages` and in
    /// `processScheduledMessages`, which the scheduler calls on every block on every iteration — so there is no
    /// sample work to do. The framework's own `work()` cannot stand in: with no stream ports its dispatch reaches
    /// the branch that requires a `processOne` or a `processBulk`, and neither has anything to process here.
    /// `INSUFFICIENT_INPUT_ITEMS` is the honest status for a block whose input is a socket, and it is what keeps a
    /// scheduler from counting an idle plane as progress; `DONE` on shutdown is what lets the job end.
    [[nodiscard]] work::Result work(std::size_t requestedWork = std::numeric_limits<std::size_t>::max()) noexcept {
        if (lifecycle::isShuttingDown(this->state())) {
            return {requestedWork, 0UZ, work::Status::DONE};
        }
        return {requestedWork, 0UZ, work::Status::INSUFFICIENT_INPUT_ITEMS};
    }

    /// @brief The scheduler's per-iteration call on every block, which is this block's only pump.
    ///
    /// It carries no stream ports, so nothing else would ever call it. The inherited implementation is named first
    /// because it is what answers heartbeat and property requests addressed to this block; hiding it without calling
    /// it would make the source the one block in a graph that cannot be asked anything.
    void processScheduledMessages() {
        Block<MessagePacketSource>::processScheduledMessages();
        drainToPlane();
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
            throw gr::exception("queue_messages is 0; the in-process receive queue must hold at least one message");
        }
        if (queue_bytes.value == 0ULL) {
            throw gr::exception("queue_bytes is 0; the in-process receive queue must hold at least one message");
        }
    }

    void refuseFrozenChange() const {
        const auto refuse = [](std::string_view setting) { throw gr::exception(std::format("setting '{}' is read once when the socket opens and cannot change while the block is running; rebuild the graph instead", setting)); };
        const auto wanted = frozenSocketConfig();
        if (wanted.endpoint != _frozen.endpoint) {
            refuse("endpoint");
        }
        if (wanted.topic != _frozen.topic) {
            refuse("topic");
        }
        if (wanted.bind != _frozen.bind) {
            refuse("bind");
        }
        if (wanted.pattern != _frozen.pattern) {
            refuse("pattern");
        }
        if (wanted.maxMessageBytes != _frozen.maxMessageBytes) {
            refuse("max_message_bytes");
        }
        if (wanted.hwm != _frozen.hwm) {
            refuse("recv_hwm");
        }
        if (wanted.lingerMs != _frozen.lingerMs) {
            refuse("linger_ms");
        }
    }

    [[nodiscard]] detail::zmqenvelope::SocketConfig frozenSocketConfig() const { return {.endpoint = endpoint.value, .topic = topic.value, .pattern = detail::zmqio::receivePatternFromName(pattern.value), .bind = bind.value, .hwm = static_cast<std::int32_t>(recv_hwm.value), .lingerMs = linger_ms.value, .maxMessageBytes = max_message_bytes.value}; }

    /// @brief Emit what has arrived, stopping at the first message the plane has no room for.
    ///
    /// A message that does not fit stays queued rather than being dropped: the ring is drained every scheduler
    /// iteration, so the room comes back, and losing a `Set` at the last step would defeat the whole path.
    void drainToPlane() {
        while (true) {
            if (!_current.has_value()) {
                std::lock_guard lock(_mutex);
                if (_queue.empty()) {
                    return;
                }
                _current = std::move(_queue.front());
                _queue.pop_front();
                _queuedBytes -= _current->bytes;
                _pendingGap = gapFor(_current->sequence); // decided exactly once per arrival, so a retry cannot recount
            }
            if (_pendingGap.has_value()) {
                if (!emit(*_pendingGap)) {
                    return; // the gap goes out before the message that followed it, or neither goes yet
                }
                _pendingGap.reset();
                std::lock_guard lock(_mutex);
                ++nGapsAnnounced;
            }
            if (!emit(_current->message)) {
                return;
            }
            _lastSequence = _current->sequence;
            _haveHistory  = true;
            _current.reset();
            std::lock_guard lock(_mutex);
            ++nMessagesEmitted;
        }
    }

    /// @brief The message announcing what the jump to @p sequence lost, or nothing when nothing was lost.
    ///
    /// Called with `_mutex` held, because it is decided in the same step that takes the arrival off the queue.
    [[nodiscard]] std::optional<Message> gapFor(std::uint64_t sequence) {
        if (!_haveHistory) {
            return std::nullopt; // a subscriber has no history and may legitimately join mid-stream
        }
        if (sequence <= _lastSequence) {
            ++nSequenceResets; // a producer restarted; not loss, because it is not
            return std::nullopt;
        }
        const std::uint64_t lost = sequence - _lastSequence - 1ULL;
        if (lost == 0ULL) {
            return std::nullopt;
        }
        nMessagesLost += lost;
        const std::uint64_t firstMissing = _lastSequence + 1ULL;
        property_map        payload;
        payload.insert_or_assign(property_map::key_type(detail::messagepacket::kMessagesLostKey), pmt::Value(lost));
        payload.insert_or_assign(property_map::key_type("sequence"), pmt::Value(firstMissing));

        Message announcement;
        announcement.protocol    = gr::message::defaultBlockProtocol;
        announcement.cmd         = gr::message::Command::Notify;
        announcement.serviceName = this->unique_name;
        announcement.endpoint    = std::string(detail::messagepacket::kGapEndpoint);
        announcement.data        = std::move(payload);
        return announcement;
    }

    /// @brief Put one message on the plane. False means the ring had no room and the caller should try again later.
    [[nodiscard]] bool emit(const Message& message) {
        WriterSpanLike auto span = this->msgOut.streamWriter().tryReserve<SpanReleasePolicy::ProcessAll>(1UZ);
        if (span.empty()) {
            return false;
        }
        span[0] = message;
        span.publish(1UZ);
        return true;
    }

    void report() const {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        std::lock_guard lock(_mutex);
        append("envelopes received", _receiver.messagesReceived());
        append("bytes received", _receiver.bytesReceived());
        append("messages emitted", nMessagesEmitted);
        append("gaps announced", nGapsAnnounced);
        append("messages lost", nMessagesLost);
        append("sequence resets", nSequenceResets);
        append("bad frame count", nBadFrameCount);
        append("short header", nShortHeader);
        for (std::size_t i = 0UZ; i < nHeaderRefusals.size(); ++i) {
            append(gr::network::discardReason(static_cast<gr::network::EnvelopeError>(i)), nHeaderRefusals[i]);
        }
        append("item type mismatch", nItemTypeMismatch);
        append("unexpected payload", nUnexpectedPayload);
        append("length mismatch", nLengthMismatch);
        append("over max", nOverMax);
        append("bad metadata", nBadMetadata);
        append("missing sequence", nMissingSequence);
        append("malformed message", nMalformedMessage);
        append("dropped by backpressure", nDroppedByBackpressure);
        append("over queue bytes", nOverQueueBytes);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::network::MessagePacketSource '{}': {}", this->name, report);
        }
    }

    /// @brief Validate one packet and queue the message it carries, or count the refusal it earned. Reader thread.
    void handleMessage(std::span<zmq::message_t> parts, std::size_t partCount, std::uint64_t partBytes) {
        const auto refuse = [this](std::uint64_t& counter) {
            std::lock_guard lock(_mutex);
            ++counter;
        };
        if (partCount != 4UZ) {
            refuse(nBadFrameCount);
            return;
        }
        const std::span<const std::uint8_t> headerFrame(static_cast<const std::uint8_t*>(parts[1UZ].data()), parts[1UZ].size());
        if (headerFrame.size() < gr::network::kHeaderBytesV1) {
            refuse(nShortHeader);
            return;
        }
        const auto header = gr::network::decodeHeader(headerFrame);
        if (!header.has_value()) {
            countKernelRefusal(header.error(), headerFrame);
            return;
        }
        if (header->item_type != gr::network::kItemTypeCode<std::uint8_t>) {
            refuse(nItemTypeMismatch);
            return;
        }
        if (header->meta_bytes != parts[2UZ].size() || header->payload_bytes != parts[3UZ].size() || headerFrame.size() != header->header_bytes) {
            refuse(nLengthMismatch); // the header's lengths and libzmq's are two statements about one message
            return;
        }
        if (header->item_count != 0U) {
            refuse(nUnexpectedPayload); // a message rides in the metadata frame; items here mean a peer of another kind
            return;
        }
        if (static_cast<std::uint64_t>(header->header_bytes) + static_cast<std::uint64_t>(header->meta_bytes) > _frozen.maxMessageBytes) {
            refuse(nOverMax);
            return;
        }

        const std::string_view text(static_cast<const char*>(parts[2UZ].data()), parts[2UZ].size());
        const auto             parsed = pmt::yaml::deserialize(text);
        if (!parsed.has_value()) {
            std::lock_guard lock(_mutex);
            ++nBadMetadata;
            std::println(stderr, "gr::blocks::network::MessagePacketSource '{}': metadata frame line {} column {}: {}", this->name, parsed.error().line, parsed.error().column, parsed.error().message);
            return;
        }

        const auto sequenceEntry = parsed->find(property_map::key_type("sequence"));
        if (sequenceEntry == parsed->end() || sequenceEntry->second.get_if<std::uint64_t>() == nullptr) {
            refuse(nMissingSequence);
            return;
        }
        const auto messageEntry = parsed->find(property_map::key_type(detail::messagepacket::kMessageKey));
        if (messageEntry == parsed->end() || messageEntry->second.get_if<property_map>() == nullptr) {
            refuse(nMalformedMessage);
            return;
        }
        std::optional<Message> message = detail::messagepacket::decode(*messageEntry->second.get_if<property_map>());
        if (!message.has_value()) {
            refuse(nMalformedMessage);
            return;
        }

        Arrival arrival;
        arrival.message  = std::move(*message);
        arrival.sequence = *sequenceEntry->second.get_if<std::uint64_t>();
        arrival.bytes    = partBytes;
        enqueue(std::move(arrival));
    }

    void countKernelRefusal(gr::network::EnvelopeError error, std::span<const std::uint8_t> headerFrame) {
        std::lock_guard lock(_mutex);
        ++nHeaderRefusals[static_cast<std::size_t>(error)];
        if (error != gr::network::EnvelopeError::BadVersion && error != gr::network::EnvelopeError::FutureVersion) {
            return;
        }
        const std::uint16_t version = static_cast<std::uint16_t>(static_cast<std::uint16_t>(headerFrame[4UZ]) | static_cast<std::uint16_t>(static_cast<std::uint16_t>(headerFrame[5UZ]) << 8U));
        if (std::ranges::find(_loggedVersions, version) == _loggedVersions.end()) {
            _loggedVersions.push_back(version); // once per distinct value: a mismatched peer at rate would flood
            std::println(stderr, "gr::blocks::network::MessagePacketSource '{}': refusing wire version {} on '{}'; this reader implements version {}", this->name, version, _frozen.endpoint, gr::network::kWireVersion);
        }
    }

    /// @brief Put one decoded arrival on the queue the message plane drains, shedding the oldest where it does not fit.
    ///
    /// An arrival larger than `queue_bytes` is discarded and counted rather than queued: shedding what is queued
    /// cannot make room for one, and queueing it anyway would put the queue above the bound the graph set.
    void enqueue(Arrival&& arrival) {
        std::lock_guard lock(_mutex);
        if (arrival.bytes > _queueBytes) {
            ++nOverQueueBytes;
            return;
        }
        while (_queue.size() >= _queueMessages || _queuedBytes + arrival.bytes > _queueBytes) {
            _queuedBytes -= _queue.front().bytes;
            _queue.pop_front();
            ++nDroppedByBackpressure; // reported downstream by the same gap message as a loss on the wire
        }
        _queuedBytes += arrival.bytes;
        _queue.push_back(std::move(arrival));
    }
};

} // namespace gr::blocks::network

#endif // GNURADIO_NETWORK_MESSAGEPACKETIO_HPP
