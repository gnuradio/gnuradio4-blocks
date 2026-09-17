#ifndef GNURADIO_NETWORK_STREAMPACKETIO_HPP
#define GNURADIO_NETWORK_STREAMPACKETIO_HPP

#include <algorithm>
#include <array>
#include <complex>
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
#include <utility>
#include <vector>

#include <zmq.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/YamlPmt.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/network/PacketEnvelope.hpp>
#include <gnuradio-4.0/network/RecordVocabulary.hpp>
#include <gnuradio-4.0/network/ZmqEnvelopeIo.hpp>
#include <gnuradio-4.0/network/ZmqTransport.hpp>

/**
 * @brief A tagged sample stream carried across a process boundary, in the module's own envelope.
 *
 * The record pair next door carries a `gr::Packet<T>`, which is a finished object with its own metadata; this pair
 * carries what a flowgraph edge carries — a continuous stream of `T` and the tags standing in it — so a graph can be
 * cut in two and the halves put in different processes without either half being rewritten. Nothing about the wire
 * changes: the same 32-byte header, the same YAML metadata frame and the same four ZeroMQ frames, at wire version 1.
 * A chunk of samples is simply what the payload holds, and the three facts a stream needs that a packet does not —
 * where the chunk sits in the stream, what tags stand in it and at what offsets, and the rate the stream runs at —
 * ride in the metadata frame under reserved keys.
 *
 * The one property that governs the design is that **a hole in the stream must be visible at the far end**. A
 * subscriber loses packets inside libzmq with no error at either end, so both `sequence` and the absolute
 * `stream_position` cross on every packet, and the receiver announces what is missing as one tag rather than
 * concatenating across it. A stream that silently splices is worse than one that admits a gap: everything
 * downstream of the splice is timed wrongly and nothing says so.
 */
namespace gr::blocks::network {

namespace detail::streampacket {

// The reserved metadata keys this pair adds to the envelope. Each is producer-private: it serves one boundary
// crossing and is consumed by the receiving block, so the record-metadata vocabulary keeps its one-spelling rule.
// `sequence` and `sample_rate` are not listed here because they are the vocabulary's own and are spelled as it
// spells them.

/// @brief The absolute index of a chunk's first sample, counted by the sink from its own start.
///
/// Not `sample_start`: that vocabulary key states where a *record* begins in a producer's stream and crosses as a
/// fact about the record. This one is the sink's own count over the edge it was placed on, and it exists so the
/// receiver can name the exact span a loss swallowed. It is consumed on arrival and reappears only on a gap tag,
/// where it names the first sample the hole swallowed.
inline constexpr std::string_view kStreamPositionKey = "stream_position";

/// @brief The tags standing in a chunk, as a list of `{offset, map}` entries with offsets from its first sample.
inline constexpr std::string_view kTagsKey      = "packet_tags";
inline constexpr std::string_view kTagOffsetKey = "offset"; ///< within one `packet_tags` entry
inline constexpr std::string_view kTagMapKey    = "map";    ///< within one `packet_tags` entry

/// @brief On a gap tag, how many packets the `sequence` discontinuity accounts for.
inline constexpr std::string_view kPacketsLostKey = "packets_lost";

/// @brief One tag as it travels: an offset from the chunk's first sample and the map that stands there.
struct CarriedTag {
    std::size_t  offset = 0UZ;
    property_map map{};
};

/// @brief The value of `sample_rate` in @p map, at the type the vocabulary declares, however the key is spelled.
///
/// A tag map may carry the framework's prefixed `gr:sample_rate` or the short form the vocabulary uses; both name one
/// key, so both are read. A value at another type is not a sample rate and is left alone — it still crosses inside
/// the tag it belongs to, where the far end sees exactly what the near end did.
[[nodiscard]] inline std::optional<float> sampleRateOf(const property_map& map) noexcept {
    using gr::blocks::network::detail::shortKey;
    for (const auto& [key, value] : map) {
        if (shortKey(std::string_view(key)) == gr::tag::SAMPLE_RATE.shortKey()) {
            if (const float* rate = value.get_if<float>(); rate != nullptr) {
                return *rate;
            }
            return std::nullopt;
        }
    }
    return std::nullopt;
}

} // namespace detail::streampacket

GR_REGISTER_BLOCK(gr::blocks::network::StreamPacketSink, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
requires gr::network::EnvelopeItem<T>
struct StreamPacketSink : Block<StreamPacketSink<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Cuts a tagged stream of T into packets and publishes each as one versioned four-frame ZeroMQ message.

This is where a flowgraph edge leaves the process. `max_items` sets the cut: zero means one packet per work call,
which follows the scheduler's own chunking, and any other value bounds the packet so a downstream latency budget is a
number rather than a consequence of buffer sizing. The envelope is the module's, unchanged and at wire version 1, so
a hand-built peer reads the header and the payload of these messages without knowing that a stream produced them.

Three facts ride in the metadata frame beside `sequence`. `stream_position` is the absolute index of the chunk's
first sample, counted by this block from its own start; it is what lets the far end state the exact span a lost
packet swallowed instead of only that something was lost. `packet_tags` carries the tags standing in the chunk as a
list of `{offset, map}` entries, offsets relative to the chunk's first sample, maps copied key for key with nothing
filtered. `sample_rate` is stated on every packet once the stream has stated one, so a subscriber that joined
mid-stream learns the rate at once rather than waiting for the next tag to come round. Where the stream states none —
the tree's own `SignalGenerator` carries its rate in a setting and never tags it — the `sample_rate` setting is
stated instead; a tag always wins over it, because a tag is the more local truth.

The framework delivers those tags in one of two shapes and the block reads both. A block with no forward tag
propagation has its input span cut at the next tag, so the ordinary case is one tag at relative index 0; where tags
stand closer together than the port's minimum request it cannot cut there, and the span carries several at their own
offsets. Both are the same code path — the span's tag view, offsets clamped into the span — because a block that
only read index 0 would silently drop the second tag of a close pair.

A chunk whose envelope would exceed `max_message_bytes` is refused, counted, and skipped over: `sequence` and
`stream_position` both advance past it, so the far end reports exactly one lost packet and the exact sample span it
covered. The record sink withholds the sequence for a refusal because a refused packet leaves by its `reject` port
and would renumber a stream that still carries it; here the samples are simply gone, and saying so is the only
honest reading.

Everything else is the record sink's contract, for the same reasons: `pub` by default because a `push` sink splits a
stream between two consumers with no error anywhere, and `drop_oldest` by default because a slow subscriber must not
stall the graph.
)"">;

    PortIn<T> in;

    Annotated<std::string, "endpoint", Visible, Doc<"libzmq endpoint, e.g. tcp://127.0.0.1:5555; required, there is no default">> endpoint{};
    Annotated<bool, "bind", Doc<"bind the endpoint rather than connect to it; the producer is normally the stable end">>          bind              = true;
    Annotated<std::string, "pattern", Doc<"'pub' (fan out) or 'push' (round-robin, lossless, one consumer)">>                     pattern           = std::string("pub");
    Annotated<std::string, "topic", Doc<"frame 0, the subscription prefix; empty publishes to every subscriber">>                 topic             = std::string("");
    Annotated<gr::Size_t, "max_items", Doc<"samples per packet; 0 emits one packet per work call">>                               max_items         = 0U;
    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"rate to state where the stream states none; 0 states none">>                 sample_rate       = 0.0f;
    Annotated<std::string, "overflow", Doc<"'drop_oldest' or 'backpressure', applied when the in-process send queue is full">>    overflow          = std::string("drop_oldest");
    Annotated<gr::Size_t, "queue_messages", Doc<"in-process send queue depth">>                                                   queue_messages    = 1024U;
    Annotated<std::uint64_t, "queue_bytes", Unit<"byte">, Doc<"in-process send queue size">>                                      queue_bytes       = 16777216ULL;
    Annotated<gr::Size_t, "send_hwm", Doc<"ZMQ_SNDHWM; libzmq's own 1000 is up to 64 MiB of buffer this graph cannot see">>       send_hwm          = 16U;
    Annotated<std::uint64_t, "max_message_bytes", Unit<"byte">, Doc<"a chunk whose envelope would exceed this is refused">>       max_message_bytes = 16777216ULL;
    Annotated<std::int32_t, "linger_ms", Unit<"ms">, Doc<"ZMQ_LINGER; 0 discards pending messages immediately on close">>         linger_ms         = 0;

    GR_MAKE_REFLECTABLE(StreamPacketSink, in, endpoint, bind, pattern, topic, max_items, sample_rate, overflow, queue_messages, queue_bytes, send_hwm, max_message_bytes, linger_ms);

    /// @brief The block's counters and its transport's, as one set.
    ///
    /// The block's own are written by `processBulk` and by nothing else, so they read true once the graph has
    /// stopped; the transport's are taken under the send queue's own lock and may be read at any time.
    struct Counters {
        std::uint64_t packetsSent        = 0ULL;
        std::uint64_t bytesSent          = 0ULL;
        std::uint64_t samplesSent        = 0ULL;
        std::uint64_t tagsCarried        = 0ULL;
        std::uint64_t packetsRejected    = 0ULL;
        std::uint64_t samplesRejected    = 0ULL;
        std::uint64_t tagsRejected       = 0ULL;
        std::uint64_t overQueueBytes     = 0ULL; ///< chunks refused because one envelope exceeds queue_bytes
        std::uint64_t droppedOnOverflow  = 0ULL;
        std::uint64_t backpressureStalls = 0ULL;
        std::uint64_t droppedAtStop      = 0ULL;
        std::uint64_t sendErrors         = 0ULL;
    };

    // Counted, stated drops and refusals. What libzmq sheds inside a PUB socket is not countable at the sender and is
    // reported instead at the receiver, from the `sequence` and `stream_position` this sink puts on every packet.
    std::uint64_t nSamplesSent        = 0ULL; ///< samples enqueued for the wire
    std::uint64_t nTagsCarried        = 0ULL; ///< tags placed in a `packet_tags` list
    std::uint64_t nPacketsRejected    = 0ULL; ///< chunks refused for exceeding max_message_bytes
    std::uint64_t nOverQueueBytes     = 0ULL; ///< chunks refused because one envelope exceeds queue_bytes
    std::uint64_t nSamplesRejected    = 0ULL; ///< samples those chunks held
    std::uint64_t nTagsRejected       = 0ULL; ///< tags those chunks held
    std::uint64_t nBackpressureStalls = 0ULL; ///< processBulk calls that consumed fewer samples than they read

    std::uint64_t                                 _sequence       = 0ULL; ///< packets this sink has emitted
    std::uint64_t                                 _streamPosition = 0ULL; ///< samples this sink has read
    std::optional<float>                          _sampleRate{};          ///< the last rate the stream stated
    std::optional<float>                          _statedRate{};          ///< the `sample_rate` setting, where one was given
    std::vector<detail::streampacket::CarriedTag> _spanTags{};            ///< one work call's tags, kept to spare an allocation

    bool          _socketOpen      = false;
    bool          _backpressure    = false;
    std::uint64_t _maxMessageBytes = 16777216ULL;
    std::uint64_t _queueBytes      = 16777216ULL; ///< frozen beside the transport's copy, which start() configures
    std::size_t   _maxItems        = 0UZ;

    detail::zmqenvelope::SocketConfig _frozen{}; ///< the socket settings, read once when the socket opens
    detail::zmqenvelope::SendQueue    _sender{}; ///< joins its own thread however the block dies

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() {
        validate();
        _frozen     = frozenSocketConfig();
        _queueBytes = queue_bytes.value;
        _sender.configure(static_cast<std::size_t>(queue_messages.value), _queueBytes, _backpressure);
        _sequence       = 0ULL; // a restarted sink restarts its stream, which the far end reads as a producer reset
        _streamPosition = 0ULL;
        _sampleRate.reset();
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
        _maxItems        = static_cast<std::size_t>(max_items.value);
        _statedRate      = sample_rate.value > 0.0f ? std::optional<float>(sample_rate.value) : std::nullopt;
    }

    [[nodiscard]] Counters counters() const {
        const auto transport = _sender.counters();
        return {.packetsSent = transport.packetsSent, .bytesSent = transport.bytesSent, .samplesSent = nSamplesSent, .tagsCarried = nTagsCarried, .packetsRejected = nPacketsRejected, .samplesRejected = nSamplesRejected, .tagsRejected = nTagsRejected, .overQueueBytes = nOverQueueBytes, .droppedOnOverflow = transport.droppedOnOverflow, .backpressureStalls = nBackpressureStalls, .droppedAtStop = transport.droppedAtStop, .sendErrors = transport.sendErrors};
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan) {
        const std::size_t available = inSpan.size();
        if (available == 0UZ) {
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        collectTags(inSpan, available);

        const std::size_t chunkItems = _maxItems == 0UZ ? available : _maxItems;
        std::size_t       consumed   = 0UZ;
        std::size_t       nextTag    = 0UZ; // _spanTags is ordered by offset, so one cursor walks it
        while (consumed < available) {
            const std::size_t items = std::min(chunkItems, available - consumed);
            const std::size_t first = nextTag;
            while (nextTag < _spanTags.size() && _spanTags[nextTag].offset < consumed + items) {
                ++nextTag;
            }

            detail::zmqenvelope::Outgoing envelope;
            envelope.metadata                = pmt::yaml::serialize(buildMetadata(consumed, first, nextTag));
            const std::uint64_t payloadBytes = static_cast<std::uint64_t>(items) * sizeof(T);
            const std::uint64_t total        = gr::network::kHeaderBytesV1 + envelope.metadata.size() + payloadBytes;
            if (total > _maxMessageBytes) {
                rejectChunk(nPacketsRejected, items, nextTag - first);
                consumed += items;
                continue; // both counters advance, so the far end reads the hole as one lost packet of exactly these samples
            }
            // a queue cannot shed its way to room for an envelope larger than the whole of it, so such a chunk is
            // refused here rather than admitted over the bound the graph set
            if (total > _queueBytes) {
                rejectChunk(nOverQueueBytes, items, nextTag - first);
                consumed += items;
                continue;
            }

            gr::network::EnvelopeHeader header;
            header.item_type     = gr::network::kItemTypeCode<T>;
            header.item_size     = static_cast<std::uint8_t>(sizeof(T));
            header.item_count    = static_cast<std::uint32_t>(items);
            header.payload_bytes = static_cast<std::uint32_t>(payloadBytes);
            header.meta_bytes    = static_cast<std::uint32_t>(envelope.metadata.size());
            envelope.header      = gr::network::encodeHeader(header);

            const std::span<const std::byte> raw = std::as_bytes(std::span<const T>(inSpan).subspan(consumed, items));
            envelope.payload.resize(raw.size());
            std::memcpy(envelope.payload.data(), raw.data(), raw.size());

            if (!_sender.enqueue(std::move(envelope))) {
                ++nBackpressureStalls; // the samples are not consumed, so the input buffer fills and the stall propagates
                break;
            }
            nSamplesSent += items;
            nTagsCarried += nextTag - first;
            ++_sequence;
            _streamPosition += items;
            consumed += items;
        }

        std::ignore = inSpan.consume(consumed);
        return consumed == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }

private:
    void validate() {
        if (endpoint.value.empty()) {
            throw gr::exception("endpoint is empty; a transport sink has no default endpoint and will not bind to nothing");
        }
        std::ignore = detail::zmqio::sendPatternFromName(pattern.value);
        if (overflow.value != "drop_oldest" && overflow.value != "backpressure") {
            throw gr::exception(std::format("overflow is '{}'; it must be 'drop_oldest' (shed the stalest envelope, count it) or 'backpressure' (consume fewer input samples)", overflow.value));
        }
        if (queue_messages.value == 0U) {
            throw gr::exception("queue_messages is 0; the in-process send queue must hold at least one envelope");
        }
        if (queue_bytes.value == 0ULL) {
            throw gr::exception("queue_bytes is 0; the in-process send queue must hold at least one envelope");
        }
        if (max_message_bytes.value == 0ULL) {
            throw gr::exception("max_message_bytes is 0; every chunk would be refused");
        }
        if (max_message_bytes.value > gr::network::kMaxEnvelopeBytes) {
            throw gr::exception(std::format("max_message_bytes is {}; {} is the largest envelope the header's 32-bit length fields can state", max_message_bytes.value, gr::network::kMaxEnvelopeBytes));
        }
    }

    /// @brief Refuse a change to a setting the running socket was built from, naming it.
    ///
    /// Re-creating a socket under a running graph means tearing down and restarting the I/O thread mid-flight, which
    /// is a teardown race; the graph rebuild the framework already supports is the supported way to move an endpoint.
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

    /// @brief This work call's tags, in offset order, and the rate the last of them states.
    ///
    /// A negative relative index is a tag the framework has not consumed yet; it belongs to the span's first sample
    /// and is clamped there rather than dropped, which is what the framework's own readers do with it.
    void collectTags(InputSpanLike auto& inSpan, std::size_t available) {
        _spanTags.clear();
        for (const auto& [relIndex, tagMapRef] : inSpan.tags()) {
            const std::size_t offset = relIndex < 0 ? 0UZ : static_cast<std::size_t>(relIndex);
            if (offset >= available) {
                continue; // the span's tag view is bounded by the span, so this is defensive and costs one compare
            }
            _spanTags.emplace_back(offset, tagMapRef.get());
            if (const std::optional<float> rate = detail::streampacket::sampleRateOf(_spanTags.back().map); rate.has_value()) {
                _sampleRate = rate;
            }
        }
    }

    /// @brief The metadata frame for a chunk: where it sits, what rate it runs at, and the tags standing in it.
    [[nodiscard]] property_map buildMetadata(std::size_t chunkStart, std::size_t firstTag, std::size_t lastTag) const {
        property_map map;
        map.insert_or_assign(property_map::key_type("sequence"), pmt::Value(_sequence));
        map.insert_or_assign(property_map::key_type(detail::streampacket::kStreamPositionKey), pmt::Value(_streamPosition));
        // the stream's own statement wins over the setting: a tag is the more local truth, and the setting exists
        // for the producers that carry their rate in their settings alone and never tag it
        if (const std::optional<float> rate = _sampleRate.has_value() ? _sampleRate : _statedRate; rate.has_value()) {
            map.insert_or_assign(property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()), pmt::Value(*rate));
        }
        if (lastTag == firstTag) {
            return map; // an absent key rather than an empty list: the common chunk carries no tags at all
        }
        gr::Tensor<pmt::Value> carried;
        for (std::size_t i = firstTag; i < lastTag; ++i) {
            property_map entry;
            entry.insert_or_assign(property_map::key_type(detail::streampacket::kTagOffsetKey), pmt::Value(std::uint64_t{_spanTags[i].offset - chunkStart}));
            entry.insert_or_assign(property_map::key_type(detail::streampacket::kTagMapKey), pmt::Value(_spanTags[i].map));
            carried.push_back(pmt::Value(std::move(entry)));
        }
        map.insert_or_assign(property_map::key_type(detail::streampacket::kTagsKey), pmt::Value(std::move(carried)));
        return map;
    }

    /// @brief Skip a chunk whose envelope will not fit, advancing both counters so the hole is exactly stated.
    ///
    /// @p reason is the counter for the bound that refused it, which is the only thing that differs between the two.
    void rejectChunk(std::uint64_t& reason, std::size_t items, std::size_t tags) {
        ++reason;
        nSamplesRejected += items;
        nTagsRejected += tags;
        ++_sequence;
        _streamPosition += items;
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
        append("samples sent", counted.samplesSent);
        append("tags carried", counted.tagsCarried);
        append("packets rejected", counted.packetsRejected);
        append("samples rejected", counted.samplesRejected);
        append("tags rejected", counted.tagsRejected);
        append("over queue bytes", counted.overQueueBytes);
        append("dropped on overflow", counted.droppedOnOverflow);
        append("backpressure stalls", counted.backpressureStalls);
        append("dropped at stop", counted.droppedAtStop);
        append("send errors", counted.sendErrors);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::network::StreamPacketSink '{}': {}", this->name, report);
        }
    }
};

GR_REGISTER_BLOCK(gr::blocks::network::StreamPacketSource, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
requires gr::network::EnvelopeItem<T>
struct StreamPacketSource : Block<StreamPacketSource<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Receives the stream pair's packets and republishes the samples and the tags standing in them.

This is where the flowgraph edge `StreamPacketSink` cut resumes. The payload becomes samples on `out` and every
carried tag is republished at the same offset it had, so a tag that stood at sample k of the sender's stream stands
at sample k of this one. `sample_rate` is republished as a tag whenever the value changes, which for a subscriber
that joined mid-stream is at its first sample: the rate is a fact the receiving half needs before it can do anything
with the samples, and a subscriber has no way to ask for it.

**A gap is announced, never spliced.** A packet lost inside libzmq costs nothing at either end and is invisible in
the payloads, so the block reconstructs the loss instead: `sequence` says how many packets went missing and
`stream_position` says exactly which samples they held. One tag is published at the next sample, carrying
`n_dropped_samples` (the framework's own key, so a block downstream that already understands dropped samples needs
no new vocabulary), `packets_lost` and `stream_position` — the absolute index of the first missing sample, so the
missing span is `[stream_position, stream_position + n_dropped_samples)`. The check runs where the packet is
published rather than where it arrives, so a packet the in-process queue shed is announced by the same mechanism as
one the wire lost, and `droppedByBackpressure` is what separates the two causes.

A packet is refused for one named reason and counted. The header refusals are the envelope kernel's own, by its
spelling; beyond them a message that is not four frames, whose frame sizes disagree with the header's lengths, that
carries a different item type, or whose payload is not a whole number of items is refused, and so is one that states
no `sequence` and `stream_position`: a packet this block cannot place in the stream is one it cannot honestly
concatenate, and the gap tag exists precisely so that it never has to.

Validation and decoding happen on the block's own reader thread, so a flood of malformed messages costs that thread
and never the scheduler, and the queue the graph drains holds only packets whose shape is established. The queue is
bounded and drops its oldest packet when the graph falls behind, which for a live stream is the right end to shed.

Unlike the record source this block tracks one stream, not one per `source_id`. Two producers on one endpoint
interleave two sample streams into one edge, which no gap tag can describe; a graph that wants two streams uses two
sources.
)"">;

    PortOut<T> out;

    Annotated<std::string, "endpoint", Visible, Doc<"libzmq endpoint; required, there is no default">>                                   endpoint{};
    Annotated<bool, "bind", Doc<"bind the endpoint rather than connect to it; a consumer normally connects">>                            bind              = false;
    Annotated<std::string, "pattern", Doc<"'sub' (prefix subscription) or 'pull' (fair-queued)">>                                        pattern           = std::string("sub");
    Annotated<std::string, "topic", Doc<"ZMQ_SUBSCRIBE prefix, 'sub' only; empty accepts everything and refuses it observably">>         topic             = std::string("");
    Annotated<std::uint64_t, "max_message_bytes", Unit<"byte">, Doc<"ZMQ_MAXMSGSIZE and this block's own bound; required, must be > 0">> max_message_bytes = 0ULL;
    Annotated<gr::Size_t, "queue_messages", Doc<"in-process receive queue depth">>                                                       queue_messages    = 1024U;
    Annotated<std::uint64_t, "queue_bytes", Unit<"byte">, Doc<"in-process receive queue size">>                                          queue_bytes       = 16777216ULL;
    Annotated<gr::Size_t, "recv_hwm", Doc<"ZMQ_RCVHWM, libzmq's own inbound queue bound">>                                               recv_hwm          = 16U;
    Annotated<std::int32_t, "linger_ms", Unit<"ms">, Doc<"ZMQ_LINGER; 0 discards pending messages immediately on close">>                linger_ms         = 0;

    GR_MAKE_REFLECTABLE(StreamPacketSource, out, endpoint, bind, pattern, topic, max_message_bytes, queue_messages, queue_bytes, recv_hwm, linger_ms);

    /// @brief The counters as one set, taken under the lock the reader thread writes most of them behind.
    struct Counters {
        std::uint64_t envelopesReceived     = 0ULL;
        std::uint64_t bytesReceived         = 0ULL;
        std::uint64_t packetsPublished      = 0ULL;
        std::uint64_t samplesPublished      = 0ULL;
        std::uint64_t tagsPublished         = 0ULL;
        std::uint64_t messagesRefused       = 0ULL; ///< the sum of every refusal reason
        std::uint64_t sequenceGaps          = 0ULL;
        std::uint64_t packetsLost           = 0ULL;
        std::uint64_t samplesLost           = 0ULL;
        std::uint64_t sequenceResets        = 0ULL;
        std::uint64_t streamRewinds         = 0ULL;
        std::uint64_t droppedByBackpressure = 0ULL;
        std::uint64_t overQueueBytes        = 0ULL; ///< arrivals discarded because one alone exceeds queue_bytes
        std::uint64_t tagsMalformed         = 0ULL;
        std::uint64_t emptyPackets          = 0ULL;
        std::uint64_t metaKeysMistyped      = 0ULL;
    };

    /// @brief One counter per envelope-kernel refusal, indexed by the error, reported under its own `discard_reason`.
    ///
    /// An array rather than eleven members because the kernel already names the reasons, and a second table of names
    /// beside its own is the drift this module exists to avoid.
    std::array<std::uint64_t, 11UZ> nHeaderRefusals{};

    std::uint64_t nPacketsPublished  = 0ULL; ///< packets whose last sample reached out
    std::uint64_t nSamplesPublished  = 0ULL;
    std::uint64_t nTagsPublished     = 0ULL; ///< carried tags, gap tags and rate tags together
    std::uint64_t nBadFrameCount     = 0ULL; ///< a message that was not exactly four parts
    std::uint64_t nShortHeader       = 0ULL; ///< frame 1 shorter than a header
    std::uint64_t nItemTypeMismatch  = 0ULL; ///< a well-formed envelope carrying a different item type
    std::uint64_t nLengthMismatch    = 0ULL; ///< the header's lengths disagree with the frame sizes libzmq reported
    std::uint64_t nOverMax           = 0ULL; ///< a message that got through libzmq's bound but claims more
    std::uint64_t nBadMetadata       = 0ULL; ///< frame 2 did not parse
    std::uint64_t nMissingStreamKeys = 0ULL; ///< no `sequence` or no `stream_position`; the packet cannot be placed

    std::uint64_t nSequenceGaps          = 0ULL; ///< sequence discontinuities, counted where the packet is published
    std::uint64_t nPacketsLost           = 0ULL; ///< the total size of those gaps in packets
    std::uint64_t nSamplesLost           = 0ULL; ///< and in samples, which is what stream_position adds
    std::uint64_t nSequenceResets        = 0ULL; ///< a sequence at or below the last seen; a producer restarted
    std::uint64_t nStreamRewinds         = 0ULL; ///< a stream_position behind where the last packet ended
    std::uint64_t nDroppedByBackpressure = 0ULL; ///< packets discarded because the in-process queue was full
    std::uint64_t nOverQueueBytes        = 0ULL; ///< arrivals discarded because one alone exceeds queue_bytes
    std::uint64_t nTagsMalformed         = 0ULL; ///< `packet_tags` entries that were not an offset in range and a map
    std::uint64_t nEmptyPackets          = 0ULL; ///< a zero-item packet; its tags move to the next packet's first sample
    std::uint64_t nMetaKeysMistyped      = 0ULL; ///< vocabulary keys whose type disagrees with the declaration

    /// @brief One decoded packet, everything about it established on the reader thread.
    struct Arrival {
        std::vector<T>                                samples{};
        std::vector<detail::streampacket::CarriedTag> tags{};
        std::uint64_t                                 sequence       = 0ULL;
        std::uint64_t                                 streamPosition = 0ULL;
        std::optional<float>                          sampleRate{};
        std::uint64_t                                 bytes = 0ULL;
    };

    mutable std::mutex         _mutex; ///< guards the queue and the counters the reader thread writes
    std::deque<Arrival>        _queue;
    std::uint64_t              _queuedBytes = 0ULL;
    std::vector<std::uint16_t> _loggedVersions{}; ///< one log line per distinct unsupported version

    // the publishing side's state, touched only by processBulk
    std::optional<Arrival> _current{};            ///< the packet being published, which may span work calls
    std::size_t            _currentOffset = 0UZ;  ///< how many of its samples have gone out
    property_map           _prologue{};           ///< what has to be said at the current packet's first sample
    bool                   _haveHistory  = false; ///< false until the first packet establishes the baseline
    std::uint64_t          _lastSequence = 0ULL;
    std::uint64_t          _streamEnd    = 0ULL; ///< where the last published packet ended, in the sender's count
    std::optional<float>   _publishedRate{};     ///< the last rate republished as a tag

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
        _currentOffset = 0UZ;
        _prologue.clear();
        _haveHistory  = false;
        _lastSequence = 0ULL;
        _streamEnd    = 0ULL;
        _publishedRate.reset();
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
        std::uint64_t   refused = nBadFrameCount + nShortHeader + nItemTypeMismatch + nLengthMismatch + nOverMax + nBadMetadata + nMissingStreamKeys;
        for (const std::uint64_t count : nHeaderRefusals) {
            refused += count;
        }
        return {.envelopesReceived = _receiver.messagesReceived(), .bytesReceived = _receiver.bytesReceived(), .packetsPublished = nPacketsPublished, .samplesPublished = nSamplesPublished, .tagsPublished = nTagsPublished, .messagesRefused = refused, .sequenceGaps = nSequenceGaps, .packetsLost = nPacketsLost, .samplesLost = nSamplesLost, .sequenceResets = nSequenceResets, .streamRewinds = nStreamRewinds, .droppedByBackpressure = nDroppedByBackpressure, .overQueueBytes = nOverQueueBytes, .tagsMalformed = nTagsMalformed, .emptyPackets = nEmptyPackets, .metaKeysMistyped = nMetaKeysMistyped};
    }

    [[nodiscard]] work::Status processBulk(OutputSpanLike auto& outSpan) {
        std::size_t published = 0UZ;
        while (published < outSpan.size()) {
            if (!_current.has_value() && !takeNextPacket()) {
                break;
            }
            published += publishFrom(outSpan, published);
        }

        const bool readerFailed = _receiver.failed();
        bool       drained      = false;
        {
            std::lock_guard lock(_mutex);
            nSamplesPublished += published;
            drained = _queue.empty();
        }

        outSpan.publish(published);
        if (published == 0UZ) {
            // a reader that died other than by request must be distinguishable from a quiet wire: OK would leave the
            // graph running behind a permanently silent input with nothing to tell a dead endpoint from an idle one
            return readerFailed && drained && !_current.has_value() ? work::Status::ERROR : work::Status::INSUFFICIENT_INPUT_ITEMS;
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
            throw gr::exception("queue_messages is 0; the in-process receive queue must hold at least one packet");
        }
        if (queue_bytes.value == 0ULL) {
            throw gr::exception("queue_bytes is 0; the in-process receive queue must hold at least one packet");
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

    /// @brief Take the next packet off the queue and work out what has to be said at its first sample.
    [[nodiscard]] bool takeNextPacket() {
        Arrival arrival;
        {
            std::lock_guard lock(_mutex);
            if (_queue.empty()) {
                return false;
            }
            arrival = std::move(_queue.front());
            _queue.pop_front();
            _queuedBytes -= arrival.bytes;
        }
        notePlacement(arrival);
        if (arrival.sampleRate.has_value() && arrival.sampleRate != _publishedRate) {
            // stated on every packet by the sink, republished only when it changes; for a subscriber that joined
            // mid-stream that is its first sample, which is the case the rate has to cross for
            _publishedRate = arrival.sampleRate;
            _prologue.insert_or_assign(property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()), pmt::Value(*arrival.sampleRate));
        }
        if (arrival.samples.empty()) {
            std::lock_guard lock(_mutex);
            ++nEmptyPackets;
            // no sample to hang a tag on, so what it says moves to the next packet's first sample rather than vanishing
            for (const auto& tag : arrival.tags) {
                for (const auto& [key, value] : tag.map) {
                    _prologue.insert_or_assign(key, value);
                }
            }
            return false;
        }
        _current       = std::move(arrival);
        _currentOffset = 0UZ;
        return true;
    }

    /// @brief Count the discontinuity between the last published packet and @p arrival, and say it in the prologue.
    void notePlacement(const Arrival& arrival) {
        if (!_haveHistory) {
            _haveHistory = true; // a subscriber has no history and may legitimately join mid-stream: never a gap
        } else if (arrival.sequence <= _lastSequence) {
            std::lock_guard lock(_mutex);
            ++nSequenceResets; // a producer restarted; not loss, because it is not
        } else if (arrival.streamPosition < _streamEnd) {
            std::lock_guard lock(_mutex);
            ++nStreamRewinds; // the samples overlap what was already published; a gap tag would misdescribe it
        } else if (arrival.sequence > _lastSequence + 1ULL || arrival.streamPosition > _streamEnd) {
            const std::uint64_t lostPackets = arrival.sequence - _lastSequence - 1ULL;
            const std::uint64_t lostSamples = arrival.streamPosition - _streamEnd;
            {
                std::lock_guard lock(_mutex);
                ++nSequenceGaps;
                nPacketsLost += lostPackets;
                nSamplesLost += lostSamples;
            }
            gr::tag::put(_prologue, gr::tag::N_DROPPED_SAMPLES, static_cast<gr::Size_t>(lostSamples));
            _prologue.insert_or_assign(property_map::key_type(detail::streampacket::kPacketsLostKey), pmt::Value(lostPackets));
            _prologue.insert_or_assign(property_map::key_type(detail::streampacket::kStreamPositionKey), pmt::Value(_streamEnd));
        }
        _lastSequence = arrival.sequence;
        _streamEnd    = arrival.streamPosition + static_cast<std::uint64_t>(arrival.samples.size());
    }

    /// @brief Copy as much of the current packet as fits, with the tags that fall in what was copied.
    ///
    /// A packet larger than one output span is published across several calls, so the block never waits for a span
    /// big enough to hold a whole packet — a bound it cannot see and a peer chooses.
    [[nodiscard]] std::size_t publishFrom(OutputSpanLike auto& outSpan, std::size_t at) {
        const Arrival&    arrival = *_current;
        const std::size_t take    = std::min(outSpan.size() - at, arrival.samples.size() - _currentOffset);
        std::ranges::copy(std::span(arrival.samples).subspan(_currentOffset, take), outSpan.begin() + static_cast<std::ptrdiff_t>(at));

        std::uint64_t tagsOut = 0ULL;
        for (const auto& tag : arrival.tags) {
            if (tag.offset < _currentOffset || tag.offset >= _currentOffset + take) {
                continue;
            }
            const std::size_t offset = at + tag.offset - _currentOffset;
            if (tag.offset == 0UZ && !_prologue.empty()) {
                property_map merged = _prologue; // one tag rather than two at one sample: what is said there is one statement
                for (const auto& [key, value] : tag.map) {
                    merged.insert_or_assign(key, value);
                }
                _prologue.clear();
                outSpan.publishTag(merged, offset);
            } else {
                outSpan.publishTag(tag.map, offset);
            }
            ++tagsOut;
        }
        if (!_prologue.empty() && _currentOffset == 0UZ) {
            outSpan.publishTag(_prologue, at); // no carried tag stood at the first sample, so it stands alone
            _prologue.clear();
            ++tagsOut;
        }

        _currentOffset += take;
        const bool finished = _currentOffset == arrival.samples.size();
        {
            std::lock_guard lock(_mutex);
            nTagsPublished += tagsOut;
            if (finished) {
                ++nPacketsPublished;
            }
        }
        if (finished) {
            _current.reset();
            _currentOffset = 0UZ;
        }
        return take;
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
        append("packets published", nPacketsPublished);
        append("samples published", nSamplesPublished);
        append("tags published", nTagsPublished);
        append("bad frame count", nBadFrameCount);
        append("short header", nShortHeader);
        for (std::size_t i = 0UZ; i < nHeaderRefusals.size(); ++i) {
            append(gr::network::discardReason(static_cast<gr::network::EnvelopeError>(i)), nHeaderRefusals[i]);
        }
        append("item type mismatch", nItemTypeMismatch);
        append("length mismatch", nLengthMismatch);
        append("over max", nOverMax);
        append("bad metadata", nBadMetadata);
        append("missing stream keys", nMissingStreamKeys);
        append("sequence gaps", nSequenceGaps);
        append("packets lost", nPacketsLost);
        append("samples lost", nSamplesLost);
        append("sequence resets", nSequenceResets);
        append("stream rewinds", nStreamRewinds);
        append("dropped by backpressure", nDroppedByBackpressure);
        append("over queue bytes", nOverQueueBytes);
        append("tags malformed", nTagsMalformed);
        append("empty packets", nEmptyPackets);
        append("metadata keys mistyped", nMetaKeysMistyped);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::network::StreamPacketSource '{}': {}", this->name, report);
        }
    }

    /// @brief Validate one message and queue the packet it carries, or count the refusal it earned. Reader thread.
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
        if (header->item_type != gr::network::kItemTypeCode<T>) {
            refuse(nItemTypeMismatch);
            return;
        }
        if (header->meta_bytes != parts[2UZ].size() || header->payload_bytes != parts[3UZ].size() || headerFrame.size() != header->header_bytes) {
            refuse(nLengthMismatch); // the header's lengths and libzmq's are two statements about one message
            return;
        }
        if (static_cast<std::uint64_t>(header->header_bytes) + static_cast<std::uint64_t>(header->meta_bytes) + static_cast<std::uint64_t>(header->payload_bytes) > _frozen.maxMessageBytes) {
            refuse(nOverMax);
            return;
        }

        property_map map;
        if (header->meta_bytes != 0U) { // the empty map short-circuits rather than asking what an empty document means
            const std::string_view text(static_cast<const char*>(parts[2UZ].data()), parts[2UZ].size());
            const auto             parsed = pmt::yaml::deserialize(text);
            if (!parsed.has_value()) {
                std::lock_guard lock(_mutex);
                ++nBadMetadata;
                std::println(stderr, "gr::blocks::network::StreamPacketSource '{}': metadata frame line {} column {}: {}", this->name, parsed.error().line, parsed.error().column, parsed.error().message);
                return;
            }
            map = *parsed;
        }

        const std::uint64_t* sequence = nullptr;
        const std::uint64_t* position = nullptr;
        if (const auto entry = map.find(property_map::key_type("sequence")); entry != map.end()) {
            sequence = entry->second.template get_if<std::uint64_t>();
        }
        if (const auto entry = map.find(property_map::key_type(detail::streampacket::kStreamPositionKey)); entry != map.end()) {
            position = entry->second.template get_if<std::uint64_t>();
        }
        if (sequence == nullptr || position == nullptr) {
            // the pair's contract is that both cross on every packet; a packet this block cannot place is one it
            // cannot honestly concatenate, and concatenating anyway is what the gap tag exists to prevent
            refuse(nMissingStreamKeys);
            return;
        }

        Arrival arrival;
        arrival.sequence       = *sequence;
        arrival.streamPosition = *position;
        arrival.sampleRate     = detail::streampacket::sampleRateOf(map);
        arrival.bytes          = partBytes;
        arrival.samples.resize(header->item_count);
        if (header->payload_bytes != 0U) {
            std::memcpy(arrival.samples.data(), parts[3UZ].data(), header->payload_bytes);
        }

        const std::uint64_t mistyped  = detail::countMistypedKeys(map);
        const std::uint64_t malformed = decodeTags(map, arrival);
        {
            std::lock_guard lock(_mutex);
            nMetaKeysMistyped += mistyped;
            nTagsMalformed += malformed;
        }
        enqueue(std::move(arrival));
    }

    /// @brief Read `packet_tags` into @p arrival, returning how many entries were not a usable `{offset, map}` pair.
    ///
    /// An entry is dropped rather than refusing the packet: the samples are sound and are what a stream is for, and a
    /// dropped annotation that is counted is a smaller loss than a hole in the stream that is not.
    [[nodiscard]] std::uint64_t decodeTags(const property_map& map, Arrival& arrival) const {
        const auto entry = map.find(property_map::key_type(detail::streampacket::kTagsKey));
        if (entry == map.end()) {
            return 0ULL;
        }
        const auto* carried = entry->second.template get_if<gr::Tensor<pmt::Value>>();
        if (carried == nullptr) {
            return 1ULL;
        }
        std::uint64_t malformed = 0ULL;
        for (const auto& element : *carried) {
            const auto* fields = element.template get_if<property_map>();
            if (fields == nullptr) {
                ++malformed;
                continue;
            }
            const auto offsetEntry = fields->find(property_map::key_type(detail::streampacket::kTagOffsetKey));
            const auto mapEntry    = fields->find(property_map::key_type(detail::streampacket::kTagMapKey));
            if (offsetEntry == fields->end() || mapEntry == fields->end()) {
                ++malformed;
                continue;
            }
            const std::uint64_t* offset = offsetEntry->second.template get_if<std::uint64_t>();
            const property_map*  tagMap = mapEntry->second.template get_if<property_map>();
            if (offset == nullptr || tagMap == nullptr || *offset >= arrival.samples.size()) {
                ++malformed; // an offset off the end of the payload names no sample and cannot be placed
                continue;
            }
            arrival.tags.emplace_back(static_cast<std::size_t>(*offset), *tagMap);
        }
        std::ranges::stable_sort(arrival.tags, {}, &detail::streampacket::CarriedTag::offset);
        return malformed;
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
            std::println(stderr, "gr::blocks::network::StreamPacketSource '{}': refusing wire version {} on '{}'; this reader implements version {}", this->name, version, _frozen.endpoint, gr::network::kWireVersion);
        }
    }

    /// @brief Put one decoded packet on the queue `processBulk` drains, shedding the oldest where it does not fit.
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
            ++nDroppedByBackpressure; // announced downstream by the same gap tag as a loss on the wire, and counted
                                      // separately here, which is what tells a slow graph from a lossy wire
        }
        _queuedBytes += arrival.bytes;
        _queue.push_back(std::move(arrival));
    }
};

} // namespace gr::blocks::network

#endif // GNURADIO_NETWORK_STREAMPACKETIO_HPP
