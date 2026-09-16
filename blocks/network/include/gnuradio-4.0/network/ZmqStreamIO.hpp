#ifndef GNURADIO_NETWORK_ZMQSTREAMIO_HPP
#define GNURADIO_NETWORK_ZMQSTREAMIO_HPP

#include <algorithm>
#include <complex>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <format>
#include <mutex>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include <zmq.hpp>

#include <gnuradio-4.0/AtomicRef.hpp>
#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>
#include <gnuradio-4.0/thread/thread_pool.hpp>

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

    std::uint64_t nMessagesReceived = 0ULL; ///< messages taken off the socket, whatever their shape
    std::uint64_t nSamplesReceived  = 0ULL; ///< samples the accepted messages carried
    std::uint64_t nSamplesPublished = 0ULL; ///< samples published on out
    std::uint64_t nSamplesDropped   = 0ULL; ///< samples the queue shed because the publisher outran the graph
    std::uint64_t nMessagesRefused  = 0ULL; ///< messages whose payload was not a whole number of samples

    /// @brief The counters as one set, taken under the lock the I/O thread writes them behind.
    struct Counters {
        std::uint64_t messagesReceived = 0ULL;
        std::uint64_t samplesReceived  = 0ULL;
        std::uint64_t samplesPublished = 0ULL;
        std::uint64_t samplesDropped   = 0ULL;
        std::uint64_t messagesRefused  = 0ULL;
    };

    std::mutex              _mutex;
    std::condition_variable _cv;
    gr::ByteRing            _ring{};
    bool                    _stopRequested = false;
    bool                    _opened        = false;
    bool                    _readerFailed  = false;
    std::string             _openFailure{};
    bool                    _ioThreadDone  = true;
    bool                    _refusalLogged = false; ///< a misconfigured peer publishing at rate would otherwise flood the log

    std::string            _endpoint{}; ///< the socket settings, frozen for the duration of one run
    std::string            _topic{};
    detail::zmqio::Pattern _pattern         = detail::zmqio::Pattern::Sub;
    bool                   _bind            = false;
    std::uint64_t          _maxMessageBytes = 0ULL;
    std::size_t            _queueBytes      = 0UZ; ///< the ring's capacity, a whole number of samples
    std::int32_t           _recvHwm         = 16;
    std::int32_t           _lingerMs        = 0;
    bool                   _socketOpen      = false;

    struct IoThreadGuard { // must be last member — destroyed first, so the reader is gone before the queue it fills
        ZmqStreamSource* self;
        explicit IoThreadGuard(ZmqStreamSource* owner) noexcept : self(owner) {}
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
            _refusalLogged = false;
            _openFailure.clear();
            _ring.reset(_queueBytes);
        }
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
        }
    }

    [[nodiscard]] Counters counters() {
        std::lock_guard lock(_mutex);
        return {.messagesReceived = nMessagesReceived, .samplesReceived = nSamplesReceived, .samplesPublished = nSamplesPublished, .samplesDropped = nSamplesDropped, .messagesRefused = nMessagesRefused};
    }

    [[nodiscard]] work::Status processBulk(OutputSpanLike auto& outSpan) {
        std::size_t made         = 0UZ;
        bool        readerFailed = false;
        bool        drained      = false;
        {
            std::lock_guard   lock(_mutex);
            const std::size_t samples = std::min(outSpan.size(), _ring.size() / kSampleBytes);
            if (samples > 0UZ) {
                _ring.pop(reinterpret_cast<std::byte*>(outSpan.data()), samples * kSampleBytes);
            }
            made = samples;
            nSamplesPublished += samples;
            readerFailed = _readerFailed;
            drained      = _ring.empty();
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
        if (queueCapacity() != _queueBytes) {
            refuse("queue_bytes");
        }
        if (static_cast<std::int32_t>(recv_hwm.value) != _recvHwm) {
            refuse("recv_hwm");
        }
        if (linger_ms.value != _lingerMs) {
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

    void freezeSocketSettings() {
        _endpoint        = endpoint.value;
        _topic           = topic.value;
        _bind            = bind.value;
        _pattern         = detail::zmqio::receivePatternFromName(pattern.value);
        _maxMessageBytes = max_message_bytes.value;
        _queueBytes      = queueCapacity();
        _recvHwm         = static_cast<std::int32_t>(recv_hwm.value);
        _lingerMs        = linger_ms.value;
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
        append("messages received", nMessagesReceived);
        append("samples received", nSamplesReceived);
        append("samples published", nSamplesPublished);
        append("samples dropped", nSamplesDropped);
        append("messages refused", nMessagesRefused);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::network::ZmqStreamSource '{}': {}", this->name, report);
        }
    }

    void ioReadLoop() {
        thread_pool::thread::setThreadName(std::format("zmqstrsrc:{}", this->name.value));
        std::optional<zmq::context_t> context;
        std::optional<zmq::socket_t>  socket;
        std::string                   failure;
        try {
            context.emplace(1);
            socket.emplace(*context, _pattern == detail::zmqio::Pattern::Sub ? zmq::socket_type::sub : zmq::socket_type::pull);
            // the bound goes into libzmq as well as into this block, so an oversize message is refused before the
            // library allocates for it; the peer that sent one is disconnected, and its loss reads as a gap
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
        std::vector<std::byte> payload;
        while (true) {
            {
                std::lock_guard lock(_mutex);
                if (_stopRequested) {
                    return;
                }
            }
            payload.clear();
            try {
                const auto append = [&payload](const zmq::message_t& part) {
                    const auto* bytes = static_cast<const std::byte*>(part.data());
                    payload.insert(payload.end(), bytes, bytes + part.size());
                };
                zmq::message_t part;
                if (!socket.recv(part, zmq::recv_flags::none).has_value()) {
                    continue; // ZMQ_RCVTIMEO expired, which is how the loop returns to observe a stop request
                }
                // a single-frame message is payload throughout, as an unkeyed publisher sends; where more frames
                // follow, frame 0 is the publisher's key, which the subscription has already matched
                bool more = socket.get(zmq::sockopt::rcvmore) != 0;
                if (!more) {
                    append(part);
                }
                while (more) {
                    if (!socket.recv(part, zmq::recv_flags::none).has_value()) {
                        break; // atomic delivery makes this unreachable; what arrived is taken as the message
                    }
                    append(part);
                    more = socket.get(zmq::sockopt::rcvmore) != 0;
                }
            } catch (const zmq::error_t& error) {
                std::lock_guard lock(_mutex);
                if (!_stopRequested) {
                    _readerFailed = true;
                    std::println(stderr, "gr::blocks::network::ZmqStreamSource '{}': receive failed on '{}': {}", this->name, _endpoint, error.what());
                }
                return;
            }
            enqueue(payload);
        }
    }

    /// @brief Append one message's payload, shedding the oldest samples where it does not fit.
    void enqueue(std::span<const std::byte> payload) {
        std::lock_guard lock(_mutex);
        ++nMessagesReceived;
        if (payload.size() % kSampleBytes != 0UZ) {
            ++nMessagesRefused;
            if (!_refusalLogged) {
                _refusalLogged = true;
                std::println(stderr, "gr::blocks::network::ZmqStreamSource '{}': '{}' delivered {} bytes, which is not a whole number of {}-byte samples; the peer is not publishing this item type and its messages are refused", this->name, _endpoint, payload.size(), kSampleBytes);
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
