#pragma once

#include "detail/ZmqCommon.hpp"
#include "detail/ZmqTagHeaders.hpp"
#include "trait_helpers.hpp"

#include <cstring>
#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/meta/reflection.hpp>
#include <gnuradio-4.0/zeromq/detail/ZmqPmtCodec.hpp>
#include <tuple>
#include <vector>

namespace gr::blocks::zeromq {

template<typename T>
concept ZmqPubSinkAcceptableTypes = std::is_same_v<T, std::vector<typename T::value_type, typename T::allocator_type>> || std::is_same_v<T, std::complex<typename T::value_type>> || std::is_integral<T>::value || std::is_floating_point<T>::value || std::is_same_v<T, gr::pmt::Value>;

template<ZmqPubSinkAcceptableTypes T>
class ZmqPubSink : public gr::Block<ZmqPubSink<T>> {
public:
    using Description = Doc<R""(
@brief ZMQ PUB Sink.

This block sends items of type T as ZMQ messages using a PUB socket.

)"">;

public:
    gr::PortIn<T> in;
    std::string   endpoint        = "tcp://*:5555";
    int           timeout         = 100;
    bool          bind            = true;
    bool          pass_tags       = false;
    PmtWireFormat pmt_wire_format = PmtWireFormat::GR4_YAML_V1;
    int           linger          = 1000;
    int           hwm             = -1;
    std::string   key;
    bool          drop_on_hwm = true;

    detail::ZmqSocketTransport _transport{zmq::socket_type::pub};
    detail::ZmqSendCounters    _send_counters;

    GR_MAKE_REFLECTABLE(ZmqPubSink, in, endpoint, timeout, bind, pass_tags, pmt_wire_format, linger, hwm, key, drop_on_hwm);

    void start() {
        detail::ZmqSocketTransport::require_nonnegative_timeout(timeout);
        _send_counters.reset();
        _transport.open(endpoint, bind, linger, hwm, true, std::nullopt, drop_on_hwm);
    }

    void stop() { _transport.close(); }

    void reset() { stop(); }

    [[nodiscard]] std::string               last_endpoint() const { return _transport.last_endpoint(); }
    [[nodiscard]] detail::ZmqSendStatistics send_statistics() const noexcept { return _send_counters.snapshot(); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inData) {
        auto                             operation = _transport.acquire_operation_guard();
        const detail::InvocationDeadline deadline{timeout};
        auto                             cancelled = detail::cancellation_check(*this);
        if (inData.size() == 0) {
            std::ignore = inData.consume(0);
            return gr::work::Status::OK;
        }

        if (!_transport.wait_writable(deadline, true, cancelled)) {
            if (cancelled() || !drop_on_hwm) {
                if (!cancelled()) {
                    _send_counters.record_refused();
                }
                std::ignore = inData.consume(0);
            } else {
                const std::size_t message_count = is_arithmetic_or_complex_v<T> ? 1 : inData.size();
                _send_counters.record_dropped(message_count, inData.size());
                std::ignore = inData.consume(inData.size());
            }
            return gr::work::Status::OK;
        }

        std::size_t consumed = 0;
        auto&       socket   = _transport.socket();
        const auto  tags     = pass_tags ? detail::collect_tag_records(inData) : std::vector<detail::ZmqTagHeaderRecord>{};

        auto send_payload = [&](auto&& payload) {
            if (!key.empty()) {
                zmq::message_t key_message(key.size());
                std::memcpy(key_message.data(), key.data(), key.size());
                if (!socket.send(key_message, zmq::send_flags::sndmore | zmq::send_flags::dontwait)) {
                    return false;
                }
            }
            return bool(socket.send(payload, zmq::send_flags::dontwait));
        };

        if constexpr (is_vector_of_arithmetic_or_complex_v<T>) {
            for (auto& a : inData) {
                if (consumed == detail::kMaxMessagesPerWork) {
                    break;
                }
                const auto   header        = pass_tags ? detail::message_tag_header(tags, consumed, pmt_wire_format) : std::vector<std::uint8_t>{};
                const size_t size_in_bytes = a.size() * sizeof(typename T::value_type);

                zmq::message_t zmsg(header.size() + size_in_bytes);
                if (!header.empty()) {
                    std::memcpy(zmsg.data(), header.data(), header.size());
                }
                std::memcpy(static_cast<std::uint8_t*>(zmsg.data()) + header.size(), a.data(), size_in_bytes);
                if (!send_payload(zmsg)) {
                    if (!drop_on_hwm) {
                        _send_counters.record_refused();
                        break;
                    }
                    _send_counters.record_dropped(1, 1);
                } else {
                    _send_counters.record_accepted(1);
                }
                ++consumed;
            }
        } else if constexpr (is_arithmetic_or_complex_v<T>) {
            const auto     header        = pass_tags ? detail::serialize_tag_header(0, tags, pmt_wire_format) : std::vector<std::uint8_t>{};
            const size_t   size_in_bytes = inData.size() * sizeof(T);
            zmq::message_t zmsg(header.size() + size_in_bytes);
            if (!header.empty()) {
                std::memcpy(zmsg.data(), header.data(), header.size());
            }
            std::memcpy(static_cast<std::uint8_t*>(zmsg.data()) + header.size(), inData.data(), size_in_bytes);
            if (send_payload(zmsg)) {
                consumed = inData.size();
                _send_counters.record_accepted(consumed);
            } else if (drop_on_hwm) {
                consumed = inData.size();
                _send_counters.record_dropped(1, consumed);
            } else {
                _send_counters.record_refused();
            }
        } else if constexpr (std::is_same_v<T, gr::pmt::Value>) {
            for (auto& pmtObj : inData) {
                if (consumed == detail::kMaxMessagesPerWork) {
                    break;
                }
                const auto           header     = pass_tags ? detail::message_tag_header(tags, consumed, pmt_wire_format) : std::vector<std::uint8_t>{};
                std::vector<uint8_t> serialized = detail::serialize_pmt(pmtObj, pmt_wire_format);
                zmq::message_t       zmsg(header.size() + serialized.size());
                if (!header.empty()) {
                    std::memcpy(zmsg.data(), header.data(), header.size());
                }
                std::memcpy(static_cast<std::uint8_t*>(zmsg.data()) + header.size(), serialized.data(), serialized.size());
                if (!send_payload(zmsg)) {
                    if (!drop_on_hwm) {
                        _send_counters.record_refused();
                        break;
                    }
                    _send_counters.record_dropped(1, 1);
                } else {
                    _send_counters.record_accepted(1);
                }
                ++consumed;
            }
        }

        std::ignore = inData.consume(consumed);
        return gr::work::Status::OK;
    }
};

} // namespace gr::blocks::zeromq

GR_REGISTER_BLOCK("gr::blocks::zeromq::ZmqPubSink", gr::blocks::zeromq::ZmqPubSink, ([T]), [ uint8_t, int16_t, int32_t, float, std::complex<float>, std::vector<float>, std::vector<std::complex<float>>, gr::pmt::Value ])
