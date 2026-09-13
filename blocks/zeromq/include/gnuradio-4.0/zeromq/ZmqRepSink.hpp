#pragma once

#include "detail/ZmqCommon.hpp"
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
concept ZmqRepSinkAcceptableTypes = std::is_same_v<T, std::vector<typename T::value_type, typename T::allocator_type>> || std::is_same_v<T, std::complex<typename T::value_type>> || std::is_integral<T>::value || std::is_floating_point<T>::value || std::is_same_v<T, gr::pmt::Value>;

namespace detail {

inline std::vector<ZmqTagHeaderRecord> select_pending_tags(const std::vector<ZmqTagHeaderRecord>& pending, std::size_t count) {
    std::vector<ZmqTagHeaderRecord> selected;
    for (const auto& tag : pending) {
        if (tag.offset < count) {
            selected.push_back(tag);
        }
    }
    return selected;
}

inline void commit_pending_tags(std::vector<ZmqTagHeaderRecord>& pending, std::size_t consumed) {
    std::vector<ZmqTagHeaderRecord> remaining;
    remaining.reserve(pending.size());
    for (auto& tag : pending) {
        if (tag.offset >= consumed) {
            tag.offset -= consumed;
            remaining.push_back(std::move(tag));
        }
    }
    pending = std::move(remaining);
}

} // namespace detail

template<ZmqRepSinkAcceptableTypes T>
class ZmqRepSink : public gr::Block<ZmqRepSink<T>> {
public:
    using Description = Doc<R""(
@brief ZMQ REP Sink.

This block receives stream items and replies to REQ requests using a REP socket.

)"">;

    gr::PortIn<T> in;
    std::string   endpoint        = "tcp://*:5555";
    int           timeout         = 100;
    bool          bind            = true;
    bool          pass_tags       = false;
    PmtWireFormat pmt_wire_format = PmtWireFormat::GR4_YAML_V1;
    int           linger          = 1000;
    int           hwm             = -1;
    // Covers ZMTP handshake metadata, identities and correlated REQ envelopes,
    // while bounding allocation before the four-byte body validation below.
    std::int64_t max_message_size = 4096;

    detail::ZmqSocketTransport _transport{zmq::socket_type::rep};
    detail::ZmqSendCounters    _send_counters;

    // A REP socket must retain its reply between work calls after accepting a request.
    // This is deliberately the only cross-call stream storage in this block.
    std::optional<zmq::message_t> _pending_reply;
    std::size_t                   _pending_reply_items = 0;

    GR_MAKE_REFLECTABLE(ZmqRepSink, in, endpoint, timeout, bind, pass_tags, pmt_wire_format, linger, hwm, max_message_size);

    void start() {
        detail::ZmqSocketTransport::require_nonnegative_timeout(timeout);
        clear_run_state();
        _send_counters.reset();
        _transport.open(endpoint, bind, linger, hwm, true, max_message_size);
    }

    void stop() {
        _transport.close();
        clear_run_state();
    }

    void reset() { stop(); }

    [[nodiscard]] std::string               last_endpoint() const { return _transport.last_endpoint(); }
    [[nodiscard]] detail::ZmqSendStatistics send_statistics() const noexcept { return _send_counters.snapshot(); }

    static std::size_t parse_request_count(const zmq::message_t& request) {
        uint32_t n = 0;
        std::memcpy(&n, request.data(), sizeof(n));
        return static_cast<std::size_t>(n);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inData) {
        auto                             operation = _transport.acquire_operation_guard();
        const detail::InvocationDeadline deadline{timeout};
        auto                             cancelled = detail::cancellation_check(*this);
        std::size_t                      messages  = 0;

        auto make_empty_reply = [&]() {
            const auto header = pass_tags ? detail::serialize_tag_header(0, {}, pmt_wire_format) : std::vector<std::uint8_t>{};
            if constexpr (std::is_same_v<T, gr::pmt::Value>) {
                const auto     serialized = detail::serialize_pmt(gr::pmt::Value{}, pmt_wire_format);
                zmq::message_t reply(header.size() + serialized.size());
                if (!header.empty()) {
                    std::memcpy(reply.data(), header.data(), header.size());
                }
                if (!serialized.empty()) {
                    std::memcpy(static_cast<std::uint8_t*>(reply.data()) + header.size(), serialized.data(), serialized.size());
                }
                return reply;
            } else {
                zmq::message_t reply(header.size());
                if (!header.empty()) {
                    std::memcpy(reply.data(), header.data(), header.size());
                }
                return reply;
            }
        };

        auto stage_reply = [&](std::size_t requested, bool malformed) {
            if (malformed || requested == 0 || inData.empty()) {
                _pending_reply       = make_empty_reply();
                _pending_reply_items = 0;
                return;
            }

            if constexpr (is_arithmetic_or_complex_v<T>) {
                const std::size_t nsend    = std::min(requested, inData.size());
                const auto        all_tags = pass_tags ? detail::collect_tag_records(inData) : std::vector<detail::ZmqTagHeaderRecord>{};
                const auto        tags     = pass_tags ? detail::select_pending_tags(all_tags, nsend) : std::vector<detail::ZmqTagHeaderRecord>{};
                const auto        header   = pass_tags ? detail::serialize_tag_header(0, tags, pmt_wire_format) : std::vector<std::uint8_t>{};
                zmq::message_t    reply(header.size() + nsend * sizeof(T));
                if (!header.empty()) {
                    std::memcpy(reply.data(), header.data(), header.size());
                }
                std::memcpy(static_cast<std::uint8_t*>(reply.data()) + header.size(), inData.data(), nsend * sizeof(T));
                _pending_reply       = std::move(reply);
                _pending_reply_items = nsend;
            } else if constexpr (is_vector_of_arithmetic_or_complex_v<T>) {
                const auto&    vec      = inData.front();
                const auto     all_tags = pass_tags ? detail::collect_tag_records(inData) : std::vector<detail::ZmqTagHeaderRecord>{};
                const auto     tags     = pass_tags ? detail::select_pending_tags(all_tags, 1) : std::vector<detail::ZmqTagHeaderRecord>{};
                const auto     header   = pass_tags ? detail::serialize_tag_header(0, tags, pmt_wire_format) : std::vector<std::uint8_t>{};
                zmq::message_t reply(header.size() + vec.size() * sizeof(typename T::value_type));
                if (!header.empty()) {
                    std::memcpy(reply.data(), header.data(), header.size());
                }
                if (!vec.empty()) {
                    std::memcpy(static_cast<std::uint8_t*>(reply.data()) + header.size(), vec.data(), vec.size() * sizeof(typename T::value_type));
                }
                _pending_reply       = std::move(reply);
                _pending_reply_items = 1;
            } else {
                const auto     all_tags   = pass_tags ? detail::collect_tag_records(inData) : std::vector<detail::ZmqTagHeaderRecord>{};
                const auto     tags       = pass_tags ? detail::select_pending_tags(all_tags, 1) : std::vector<detail::ZmqTagHeaderRecord>{};
                const auto     header     = pass_tags ? detail::serialize_tag_header(0, tags, pmt_wire_format) : std::vector<std::uint8_t>{};
                const auto     serialized = detail::serialize_pmt(inData.front(), pmt_wire_format);
                zmq::message_t reply(header.size() + serialized.size());
                if (!header.empty()) {
                    std::memcpy(reply.data(), header.data(), header.size());
                }
                if (!serialized.empty()) {
                    std::memcpy(static_cast<std::uint8_t*>(reply.data()) + header.size(), serialized.data(), serialized.size());
                }
                _pending_reply       = std::move(reply);
                _pending_reply_items = 1;
            }
        };

        while (messages < detail::kMaxMessagesPerWork) {
            if (_pending_reply.has_value()) {
                if (!_transport.wait_writable(deadline, true, cancelled)) {
                    if (!cancelled()) {
                        _send_counters.record_refused();
                    }
                    break;
                }
                if (!_transport.socket().send(*_pending_reply, zmq::send_flags::dontwait)) {
                    _send_counters.record_refused();
                    break;
                }

                const std::size_t committed = _pending_reply_items;
                _send_counters.record_accepted(committed);
                _pending_reply.reset();
                _pending_reply_items = 0;
                ++messages;
                std::ignore = inData.consume(committed);
                // Return after every completed REP transaction. In particular, an empty reply
                // cannot drive an unlimited request/reply loop in one scheduler invocation.
                return gr::work::Status::OK;
            }

            if (!_transport.wait_readable(deadline, messages == 0, cancelled)) {
                break;
            }
            auto request = detail::receive_all_parts(_transport.socket(), 1);
            if (!request) {
                break;
            }

            const bool        malformed = request->has_extra_parts || request->parts.size() != 1 || request->parts.front().size() != sizeof(uint32_t);
            const std::size_t requested = malformed ? 0 : parse_request_count(request->parts.front());
            stage_reply(requested, malformed);
        }

        std::ignore = inData.consume(0);
        return gr::work::Status::OK;
    }

private:
    void clear_run_state() {
        _pending_reply.reset();
        _pending_reply_items = 0;
    }
};

} // namespace gr::blocks::zeromq

GR_REGISTER_BLOCK("gr::blocks::zeromq::ZmqRepSink", gr::blocks::zeromq::ZmqRepSink, ([T]), [ uint8_t, int16_t, int32_t, float, std::complex<float>, std::vector<float>, std::vector<std::complex<float>>, gr::pmt::Value ])
