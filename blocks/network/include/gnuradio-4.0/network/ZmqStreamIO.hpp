#ifndef GNURADIO_NETWORK_ZMQSTREAMIO_HPP
#define GNURADIO_NETWORK_ZMQSTREAMIO_HPP

#include <algorithm>
#include <complex>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <format>
#include <mutex>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include <zmq.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/network/ByteRing.hpp>
#include <gnuradio-4.0/network/ZmqEnvelopeIo.hpp>
#include <gnuradio-4.0/network/ZmqTransport.hpp>

/**
 * @brief A raw sample stream over ZeroMQ, in the wire format GNU Radio 3.10's `gr-zeromq` publishes.
 *
 * This exists for compatibility and for nothing else. The module's envelope pair speaks a versioned header this
 * module defines, which no 3.10 flowgraph can produce; `zmq_pub_sink` and `zmq_push_sink` with `pass_tags=false`
 * put nothing on the wire but the items themselves — one message per buffer, `nitems * itemsize` bytes, no header
 * and no framing — and a non-empty `key` prepends the subscription prefix as a frame of its own. A graph built on
 * this block therefore takes a stock 3.10 publisher unchanged, which is what lets a migration move one end at a
 * time. Nothing else about the format is worth adopting: it carries no type, no length and no version, so a peer
 * that changes its item type says so only by producing nonsense.
 *
 * `pass_tags=true` is not read. That variant prefixes a serialized tag header GNU Radio 4 has no reader for, and
 * decoding it would mean carrying a second wire format for the sake of the one it exists to leave behind; a
 * publisher set that way is heard as items and the header lands in the stream as noise.
 */
namespace gr::blocks::network {

GR_REGISTER_BLOCK(gr::blocks::network::ZmqStreamSource, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
requires std::is_trivially_copyable_v<T>
struct ZmqStreamSource : Block<ZmqStreamSource<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Receives raw `T` samples over a ZeroMQ SUB or PULL socket and publishes them on `out`, unchanged.

The wire format is `gr-zeromq`'s with `pass_tags=false`: one message per buffer of items, `nitems * sizeof(T)`
bytes and nothing else, preceded by the publisher's `key` as a frame of its own where one is set. A message of any
other shape — a length that is not a whole number of items — is refused and counted rather than spliced in, because
the alternative is a stream whose item boundary is permanently half a sample out and no way to say so.

A dedicated I/O thread owns the context and the socket end to end, because a libzmq socket is a single-thread
object and a scheduler pool worker is not a stable home for one. It appends whole messages to a bounded byte queue;
`processBulk` only drains that queue, so no scheduler thread ever waits on a socket.

The queue drops its oldest samples when a publisher outruns the graph. That is the right end to shed for a live
stream — the newest samples are the ones still worth having — and the sender is pacing the stream anyway, since
neither end of a ZeroMQ link can apply backpressure to a publisher that has already handed its message to the
library. The drop is a whole number of samples, so what a consumer sees across an overrun is a gap and never a
phase shift. `nSamplesDropped` counts them.

A reader that dies other than by request is not a quiet wire, and `processBulk` reports `work::Status::ERROR` once
its queue has drained rather than an endless stream of empty work: `OK` would leave the graph running behind a
permanently flat input with nothing to tell a dead endpoint from an idle one.
)"">;

    PortOut<T> out;

    Annotated<std::string, "endpoint", Visible, Doc<"libzmq endpoint, e.g. tcp://127.0.0.1:5555; required, there is no default">>        endpoint{};
    Annotated<bool, "bind", Doc<"bind the endpoint rather than connect to it; a consumer normally connects">>                            bind              = false;
    Annotated<std::string, "pattern", Doc<"'sub' (prefix subscription) or 'pull' (fair-queued)">>                                        pattern           = std::string("sub");
    Annotated<std::string, "topic", Doc<"ZMQ_SUBSCRIBE prefix, 'sub' only; the publisher's 'key', empty where it has none">>             topic             = std::string("");
    Annotated<std::uint64_t, "max_message_bytes", Unit<"byte">, Doc<"ZMQ_MAXMSGSIZE and this block's own bound; required, must be > 0">> max_message_bytes = 0ULL;
    Annotated<std::uint64_t, "queue_bytes", Unit<"byte">, Doc<"in-process sample queue size, rounded down to whole samples">>            queue_bytes       = 16777216ULL;
    Annotated<gr::Size_t, "recv_hwm", Doc<"ZMQ_RCVHWM, libzmq's own inbound queue bound">>                                               recv_hwm          = 16U;
    Annotated<std::int32_t, "linger_ms", Unit<"ms">, Doc<"ZMQ_LINGER; 0 discards pending messages immediately on close">>                linger_ms         = 0;

    GR_MAKE_REFLECTABLE(ZmqStreamSource, out, endpoint, bind, pattern, topic, max_message_bytes, queue_bytes, recv_hwm, linger_ms);

    static constexpr std::size_t kSampleBytes = sizeof(T);

    std::uint64_t nSamplesReceived  = 0ULL; ///< samples the accepted messages carried
    std::uint64_t nSamplesPublished = 0ULL; ///< samples published on out
    std::uint64_t nSamplesDropped   = 0ULL; ///< samples the queue shed because the publisher outran the graph
    std::uint64_t nMessagesRefused  = 0ULL; ///< messages this block could not read as a whole number of samples

    /// @brief The counters as one set, taken under the lock the I/O thread writes them behind.
    struct Counters {
        std::uint64_t messagesReceived = 0ULL;
        std::uint64_t samplesReceived  = 0ULL;
        std::uint64_t samplesPublished = 0ULL;
        std::uint64_t samplesDropped   = 0ULL;
        std::uint64_t messagesRefused  = 0ULL;
    };

    std::mutex _mutex; ///< guards the ring and the counters, which the reader thread writes and processBulk reads
    ByteRing   _ring{};
    bool       _refusalLogged = false; ///< a misconfigured peer publishing at rate would otherwise flood the log

    std::size_t _queueBytes = 0UZ; ///< the ring's capacity, a whole number of samples
    bool        _socketOpen = false;

    std::vector<std::byte> _payload{}; ///< one message's frames joined; the reader thread's alone, kept to spare an allocation

    detail::zmqenvelope::SocketConfig _frozen{}; ///< the socket settings, read once when the socket opens

    /// @brief Owns the socket and the thread that reads it; joins that thread however the block dies.
    ///
    /// Last declared member, so it is destroyed first and the reader is gone before the ring, mutex and counters it
    /// writes into. `stop()` cannot be relied on: the scheduler does not call it when a graph ends in ERROR, and
    /// `~Block()` cannot stand in because derived members are destroyed before it runs.
    detail::zmqenvelope::Receiver _receiver{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() {
        validate();
        _frozen     = frozenSocketConfig();
        _queueBytes = queueCapacity();
        {
            std::lock_guard lock(_mutex);
            _refusalLogged = false;
            _ring.reset(_queueBytes);
        }
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

    [[nodiscard]] Counters counters() {
        const std::uint64_t messages = _receiver.messagesReceived(); // the transport's own count, under its own lock
        std::lock_guard     lock(_mutex);
        return {.messagesReceived = messages, .samplesReceived = nSamplesReceived, .samplesPublished = nSamplesPublished, .samplesDropped = nSamplesDropped, .messagesRefused = nMessagesRefused};
    }

    [[nodiscard]] work::Status processBulk(OutputSpanLike auto& outSpan) {
        std::size_t made         = 0UZ;
        const bool  readerFailed = _receiver.failed();
        bool        drained      = false;
        {
            std::lock_guard   lock(_mutex);
            const std::size_t samples = std::min(outSpan.size(), _ring.size() / kSampleBytes);
            if (samples > 0UZ) {
                _ring.pop(reinterpret_cast<std::byte*>(outSpan.data()), samples * kSampleBytes);
            }
            made = samples;
            nSamplesPublished += samples;
            drained = _ring.empty();
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
        std::ignore = detail::zmqio::receivePatternFromName(pattern.value);
        if (max_message_bytes.value == 0ULL) {
            throw gr::exception("max_message_bytes is 0 or unset; libzmq's own default is no limit, so a source must state a bound before a peer's claimed length sizes anything");
        }
        if (queue_bytes.value < kSampleBytes) {
            throw gr::exception(std::format("queue_bytes is {}; the in-process queue must hold at least one sample of {} bytes", queue_bytes.value, kSampleBytes));
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
        if (queueCapacity() != _queueBytes) {
            refuse("queue_bytes");
        }
        if (wanted.hwm != _frozen.hwm) {
            refuse("recv_hwm");
        }
        if (wanted.lingerMs != _frozen.lingerMs) {
            refuse("linger_ms");
        }
    }

    /// @brief A 64-bit byte count narrowed to an index; a template, so the cast stands where `std::size_t` is
    /// 32 bits and is not a useless cast where the two are the same 64-bit type. A private member because the
    /// module's ZMQ detail namespace is shared with the packet transport's header.
    template<std::integral To, std::integral From>
    [[nodiscard]] static constexpr To narrowIndex(From value) noexcept {
        return static_cast<To>(value);
    }

    /// @brief `queue_bytes` as the ring is actually sized: whole samples, so that shedding the oldest of them can
    /// never leave the stream half a sample out of step.
    [[nodiscard]] std::size_t queueCapacity() const noexcept { return narrowIndex<std::size_t>(queue_bytes.value / kSampleBytes * kSampleBytes); }

    /// @brief The socket settings as the transport reads them; `start()` freezes the result for the run.
    [[nodiscard]] detail::zmqenvelope::SocketConfig frozenSocketConfig() const { return {.endpoint = endpoint.value, .topic = topic.value, .pattern = detail::zmqio::receivePatternFromName(pattern.value), .bind = bind.value, .hwm = static_cast<std::int32_t>(recv_hwm.value), .lingerMs = linger_ms.value, .maxMessageBytes = max_message_bytes.value}; }

    void report() {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("messages received", _receiver.messagesReceived());
        append("samples received", nSamplesReceived);
        append("samples published", nSamplesPublished);
        append("samples dropped", nSamplesDropped);
        append("messages refused", nMessagesRefused);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::network::ZmqStreamSource '{}': {}", this->name, report);
        }
    }

    /// @brief Join one message's frames into a payload and hand it to the queue. Runs on the reader's thread.
    ///
    /// A single-frame message is payload throughout, as an unkeyed publisher sends; where more frames follow, frame 0
    /// is the publisher's key, which the subscription has already matched. The reader keeps at most five frames, more
    /// than either shape this format has, so a message with more is refused whole rather than spliced together from
    /// the frames that were kept.
    void handleMessage(std::span<zmq::message_t> parts, std::size_t partCount, std::uint64_t /*bytes*/) {
        if (partCount > parts.size()) {
            std::lock_guard lock(_mutex);
            ++nMessagesRefused;
            if (!_refusalLogged) {
                _refusalLogged = true;
                std::println(stderr, "gr::blocks::network::ZmqStreamSource '{}': '{}' delivered a message of {} frames; this format has one or two, so the peer is not a gr-zeromq publisher and its messages are refused", this->name, _frozen.endpoint, partCount);
            }
            return;
        }
        _payload.clear();
        const auto append = [this](const zmq::message_t& part) {
            const auto* bytes = static_cast<const std::byte*>(part.data());
            _payload.insert(_payload.end(), bytes, bytes + part.size());
        };
        if (parts.size() == 1UZ) {
            append(parts[0UZ]);
        } else {
            for (std::size_t i = 1UZ; i < parts.size(); ++i) {
                append(parts[i]);
            }
        }
        enqueue(_payload);
    }

    /// @brief Append one message's payload, shedding the oldest samples where it does not fit.
    void enqueue(std::span<const std::byte> payload) {
        std::lock_guard lock(_mutex);
        if (payload.size() % kSampleBytes != 0UZ) {
            ++nMessagesRefused;
            if (!_refusalLogged) {
                _refusalLogged = true;
                std::println(stderr, "gr::blocks::network::ZmqStreamSource '{}': '{}' delivered {} bytes, which is not a whole number of {}-byte samples; the peer is not publishing this item type and its messages are refused", this->name, _frozen.endpoint, payload.size(), kSampleBytes);
            }
            return;
        }
        // ByteRing sheds exactly what it must to take the push, and both its capacity and every payload here are
        // whole samples, so the shed is whole samples too
        const std::size_t held = _ring.size();
        if (held + payload.size() > _ring.capacity()) {
            nSamplesDropped += (held + payload.size() - _ring.capacity()) / kSampleBytes;
        }
        _ring.push(payload);
        nSamplesReceived += payload.size() / kSampleBytes;
    }
};

} // namespace gr::blocks::network

#endif // GNURADIO_NETWORK_ZMQSTREAMIO_HPP
