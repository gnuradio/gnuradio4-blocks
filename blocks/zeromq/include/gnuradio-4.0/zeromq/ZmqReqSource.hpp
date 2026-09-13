#pragma once

#include "detail/ZmqCommon.hpp"
#include "detail/ZmqReceive.hpp"
#include "detail/ZmqTagHeaders.hpp"
#include "trait_helpers.hpp"

#include <algorithm>
#include <cstring>
#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/meta/reflection.hpp>
#include <gnuradio-4.0/zeromq/detail/ZmqPmtCodec.hpp>
#include <optional>
#include <vector>

namespace gr::blocks::zeromq {

template<typename T>
concept ZmqReqSourceAcceptableTypes = std::is_same_v<T, std::vector<typename T::value_type, typename T::allocator_type>> || std::is_same_v<T, std::complex<typename T::value_type>> || std::is_integral<T>::value || std::is_floating_point<T>::value || std::is_same_v<T, gr::pmt::Value>;

template<ZmqReqSourceAcceptableTypes T>
class ZmqReqSource : public gr::Block<ZmqReqSource<T>> {
public:
    using Description = Doc<R""(
@brief ZMQ REQ Source.

This block sends REQ requests and converts the returned ZMQ replies to type T.

)"">;

    gr::PortOut<T> out;
    std::string    endpoint         = "tcp://127.0.0.1:5555";
    int            timeout          = 100;
    bool           bind             = false;
    bool           pass_tags        = false;
    PmtWireFormat  pmt_wire_format  = PmtWireFormat::GR4_YAML_V1;
    int            linger           = 1000;
    int            hwm              = -1;
    std::int64_t   max_message_size = 64 * 1024 * 1024;

    detail::ZmqSocketTransport _transport{zmq::socket_type::req};
    detail::ZmqReceiveCounters _receive_counters;

    [[maybe_unused]] std::vector<T>                          _pending_items;
    [[maybe_unused]] std::vector<detail::ZmqTagHeaderRecord> _pending_tags;
    std::size_t                                              _pending_offset = 0;
    bool                                                     _req_pending    = false;
    std::optional<std::chrono::steady_clock::time_point>     _req_pending_since;

    GR_MAKE_REFLECTABLE(ZmqReqSource, out, endpoint, timeout, bind, pass_tags, pmt_wire_format, linger, hwm, max_message_size);

    void start() {
        detail::ZmqSocketTransport::require_nonnegative_timeout(timeout);
        clear_run_state();
        _receive_counters.reset();
        _transport.open(endpoint, bind, linger, hwm, false, max_message_size);
    }

    void stop() {
        _transport.close();
        clear_run_state();
    }

    void reset() { stop(); }

    [[nodiscard]] std::string                  last_endpoint() const { return _transport.last_endpoint(); }
    [[nodiscard]] detail::ZmqReceiveStatistics receive_statistics() const noexcept { return _receive_counters.snapshot(); }

    static constexpr std::size_t request_count_for(std::size_t room) {
        if constexpr (std::is_same_v<T, gr::pmt::Value>) {
            return 1;
        } else {
            return room;
        }
    }

    [[nodiscard]] work::Status processBulk(OutputSpanLike auto& outputSpan) {
        auto                             operation = _transport.acquire_operation_guard();
        const detail::InvocationDeadline deadline{timeout};
        auto                             cancelled    = detail::cancellation_check(*this);
        const std::size_t                nProcessOut  = outputSpan.size();
        std::size_t                      npublished   = 0;
        std::size_t                      transactions = 0;

        auto publish_pending_tags = [&outputSpan](std::size_t base_offset, std::size_t consumed, std::vector<detail::ZmqTagHeaderRecord>& tags) {
            std::vector<detail::ZmqTagHeaderRecord> remaining;
            remaining.reserve(tags.size());
            for (auto& tag : tags) {
                if (tag.offset < consumed) {
                    outputSpan.publishTag(detail::tag_map_from_record(tag), base_offset + static_cast<std::size_t>(tag.offset));
                } else {
                    tag.offset -= consumed;
                    remaining.push_back(std::move(tag));
                }
            }
            tags = std::move(remaining);
        };

        if constexpr (is_arithmetic_or_complex_v<T>) {
            if (!_pending_items.empty()) {
                const auto remaining = _pending_items.size() - _pending_offset;
                const auto n         = std::min(nProcessOut, remaining);
                std::copy_n(_pending_items.begin() + static_cast<std::ptrdiff_t>(_pending_offset), n, outputSpan.begin());
                publish_pending_tags(0, n, _pending_tags);
                npublished = n;
                _pending_offset += n;
                if (_pending_offset == _pending_items.size()) {
                    _pending_items.clear();
                    _pending_offset = 0;
                }
            }
        }

        // Publish already-decoded data promptly; do not start another network transaction in this call.
        if (npublished > 0 || nProcessOut == 0) {
            outputSpan.publish(npublished);
            return gr::work::Status::OK;
        }

        while (transactions < detail::kMaxMessagesPerWork && npublished < nProcessOut) {
            if (!_req_pending) {
                if (!_transport.wait_writable(deadline, transactions == 0, cancelled)) {
                    break;
                }
                const uint32_t request_size = static_cast<uint32_t>(request_count_for(nProcessOut - npublished));
                zmq::message_t request(sizeof(request_size));
                std::memcpy(request.data(), &request_size, sizeof(request_size));
                if (!_transport.socket().send(request, zmq::send_flags::dontwait)) {
                    break;
                }
                _req_pending       = true;
                _req_pending_since = std::chrono::steady_clock::now();
            }

            if (!_transport.wait_readable(deadline, transactions == 0, cancelled)) {
                // Relaxed/correlated REQ mode can retry after a lost reply without recreating the
                // socket. This is especially important for bound sockets because libzmq may not
                // release their listening endpoint quickly enough for an immediate rebind.
                // `timeout` is also the scheduler polling budget and is commonly very short.
                // Give an already-connected but not-yet-running peer a small startup grace.
                const auto recovery_timeout = std::max(std::chrono::milliseconds{timeout}, std::chrono::milliseconds{500});
                const bool recovery_expired = _req_pending_since.has_value() && std::chrono::steady_clock::now() - *_req_pending_since >= recovery_timeout;
                if (_req_pending && recovery_expired && !cancelled()) {
                    _req_pending = false;
                    _req_pending_since.reset();
                }
                break;
            }

            auto parts = detail::receive_all_parts(_transport.socket(), 1);
            if (!parts) {
                break;
            }
            _req_pending = false;
            _req_pending_since.reset();
            ++transactions;
            if (parts->has_extra_parts || parts->parts.size() != 1) {
                _receive_counters.record_refused();
                continue;
            }

            const auto& msg = parts->parts.front();
            {
                auto decoded = detail::decode_frame<T>(msg, pass_tags, pmt_wire_format, _receive_counters);
                if (!decoded) {
                    continue;
                }
                auto&                       tags           = decoded->tags;
                [[maybe_unused]] const auto consumed_bytes = decoded->consumed_bytes;
                const auto*                 payload        = static_cast<const std::uint8_t*>(msg.data()) + consumed_bytes;
                const auto                  payload_size   = msg.size() - consumed_bytes;

                if constexpr (is_arithmetic_or_complex_v<T>) {
                    const std::size_t nels = payload_size / sizeof(T);
                    const std::size_t n    = std::min(nels, nProcessOut);
                    detail::copy_items_from_bytes(payload, n, outputSpan.data());
                    publish_pending_tags(0, n, tags);
                    npublished = n;
                    if (nels > n) {
                        _pending_items.resize(nels - n);
                        _pending_offset = 0;
                        detail::copy_items_from_bytes(payload + n * sizeof(T), nels - n, _pending_items.data());
                        _pending_tags = std::move(tags);
                    }
                    if (nels == 0) {
                        _receive_counters.record_accepted(0);
                        break;
                    }
                    _receive_counters.record_accepted(nels);
                } else if constexpr (is_vector_of_arithmetic_or_complex_v<T>) {
                    using Element          = typename T::value_type;
                    auto&             vec  = outputSpan[0];
                    const std::size_t nels = payload_size / sizeof(Element);
                    vec.resize(nels);
                    detail::copy_items_from_bytes(payload, nels, vec.data());
                    for (const auto& tag : tags) {
                        outputSpan.publishTag(detail::tag_map_from_record(tag), 0);
                    }
                    npublished = 1;
                    _receive_counters.record_accepted(1);
                } else {
                    outputSpan[0] = std::move(decoded->pmt);
                    for (const auto& tag : tags) {
                        outputSpan.publishTag(detail::tag_map_from_record(tag), 0);
                    }
                    npublished = 1;
                    _receive_counters.record_accepted(1);
                }
            }

            // Any useful reply is returned immediately. Empty scalar replies end this call too.
            break;
        }

        outputSpan.publish(npublished);
        return gr::work::Status::OK;
    }

private:
    void clear_run_state() {
        _pending_items.clear();
        _pending_tags.clear();
        _pending_offset = 0;
        _req_pending    = false;
        _req_pending_since.reset();
    }
};

} // namespace gr::blocks::zeromq

GR_REGISTER_BLOCK("gr::blocks::zeromq::ZmqReqSource", gr::blocks::zeromq::ZmqReqSource, ([T]), [ uint8_t, int16_t, int32_t, float, std::complex<float>, std::vector<float>, std::vector<std::complex<float>>, gr::pmt::Value ])
