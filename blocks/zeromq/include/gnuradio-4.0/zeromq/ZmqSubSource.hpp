#pragma once

#include "detail/ZmqCommon.hpp"
#include "detail/ZmqTagHeaders.hpp"
#include "trait_helpers.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/meta/reflection.hpp>
#include <gnuradio-4.0/zeromq/detail/ZmqPmtCodec.hpp>
#include <vector>

namespace gr::blocks::zeromq {

template<typename T>
concept ZmqSubSourceAcceptableTypes = std::is_same_v<T, std::vector<typename T::value_type, typename T::allocator_type>> || std::is_same_v<T, std::complex<typename T::value_type>> || std::is_integral<T>::value || std::is_floating_point<T>::value || std::is_same_v<T, gr::pmt::Value>;

template<ZmqSubSourceAcceptableTypes T>
class ZmqSubSource : public gr::Block<ZmqSubSource<T>> {
public:
    using Description = Doc<R""(
@brief ZMQ SUB Source.

This block receives messages on a ZMQ SUB socket and converts them to type T.

)"">;

public:
    gr::PortOut<T> out;
    std::string    endpoint         = "tcp://127.0.0.1:5555";
    int            timeout          = 100;
    bool           bind             = false;
    bool           pass_tags        = false;
    PmtWireFormat  pmt_wire_format  = PmtWireFormat::GR4_YAML_V1;
    int            linger           = 1000;
    int            hwm              = -1;
    std::int64_t   max_message_size = 64 * 1024 * 1024;
    std::string    key;

    detail::ZmqSocketTransport _transport{zmq::socket_type::sub};
    detail::ZmqReceiveCounters _receive_counters;

    [[maybe_unused]] std::vector<T>                          _pending_items;
    [[maybe_unused]] std::vector<detail::ZmqTagHeaderRecord> _pending_tags;
    std::size_t                                              _pending_offset = 0;

    GR_MAKE_REFLECTABLE(ZmqSubSource, out, endpoint, timeout, bind, pass_tags, pmt_wire_format, linger, hwm, max_message_size, key);

    void start() {
        detail::ZmqSocketTransport::require_nonnegative_timeout(timeout);
        _pending_items.clear();
        _pending_tags.clear();
        _pending_offset = 0;
        _receive_counters.reset();
        _transport.open(endpoint, bind, linger, hwm, false, max_message_size);
        _transport.socket().set(zmq::sockopt::subscribe, key);
    }

    void stop() {
        _transport.close();
        _pending_items.clear();
        _pending_tags.clear();
        _pending_offset = 0;
    }

    void reset() { stop(); }

    [[nodiscard]] std::string                  last_endpoint() const { return _transport.last_endpoint(); }
    [[nodiscard]] detail::ZmqReceiveStatistics receive_statistics() const noexcept { return _receive_counters.snapshot(); }

    [[nodiscard]] work::Status processBulk(OutputSpanLike auto& outputSpan) {
        auto                             operation = _transport.acquire_operation_guard();
        const detail::InvocationDeadline deadline{timeout};
        auto                             cancelled   = detail::cancellation_check(*this);
        const std::size_t                nProcessOut = outputSpan.size();
        size_t                           npublished  = 0;
        std::size_t                      messages    = 0;

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

        if constexpr (is_vector_of_arithmetic_or_complex_v<T>) {
            for (std::size_t i = 0; i < nProcessOut && messages < detail::kMaxMessagesPerWork; ++i) {
                if (!_transport.wait_readable(deadline, messages == 0 && npublished == 0, cancelled)) {
                    break;
                }

                auto parts = detail::receive_all_parts(_transport.socket(), 2);
                if (!parts) {
                    break;
                }
                ++messages;
                if (parts->has_extra_parts || parts->parts.empty()) {
                    _receive_counters.record_refused();
                    continue;
                }
                const auto& payload = parts->parts.back();
                try {
                    std::uint64_t                           header_offset = 0;
                    std::vector<detail::ZmqTagHeaderRecord> tags;
                    std::size_t                             consumed_bytes = 0;
                    if (pass_tags) {
                        consumed_bytes = detail::parse_tag_header(static_cast<const std::uint8_t*>(payload.data()), payload.size(), header_offset, tags, pmt_wire_format);
                        for (auto& tag : tags) {
                            if (tag.offset >= header_offset) {
                                tag.offset -= header_offset;
                            }
                        }
                    }
                    detail::ZmqSocketTransport::require_multiple_of(payload.size() - consumed_bytes, sizeof(typename T::value_type), "ZmqSubSource vector payload");
                    auto&  vec  = outputSpan[i];
                    size_t nels = (payload.size() - consumed_bytes) / sizeof(typename T::value_type);
                    vec.resize(nels);
                    detail::copy_items_from_bytes(static_cast<const std::uint8_t*>(payload.data()) + consumed_bytes, nels, vec.data());
                    if (pass_tags) {
                        for (const auto& tag : tags) {
                            outputSpan.publishTag(detail::tag_map_from_record(tag), i);
                        }
                    }
                    ++npublished;
                    _receive_counters.record_accepted(1);
                    if (payload.size() == 0) {
                        break;
                    }
                } catch (...) {
                    _receive_counters.record_refused();
                    continue;
                }
            }
        } else if constexpr (is_arithmetic_or_complex_v<T>) {
            while (messages < detail::kMaxMessagesPerWork) {
                size_t room_in_span = nProcessOut - npublished;
                if (_pending_items.size()) {
                    const auto remaining = _pending_items.size() - _pending_offset;
                    const auto n         = std::min(room_in_span, remaining);
                    std::copy_n(_pending_items.data() + _pending_offset, n, outputSpan.begin() + static_cast<std::ptrdiff_t>(npublished));
                    publish_pending_tags(npublished, n, _pending_tags);
                    npublished += n;
                    _pending_offset += n;
                    if (_pending_offset == _pending_items.size()) {
                        _pending_items.clear();
                        _pending_offset = 0;
                    }
                }

                room_in_span = nProcessOut - npublished;
                if (!room_in_span) {
                    break;
                }

                if (!_transport.wait_readable(deadline, messages == 0 && npublished == 0, cancelled)) {
                    break;
                }

                auto parts = detail::receive_all_parts(_transport.socket(), 2);
                if (!parts) {
                    break;
                }
                ++messages;
                if (parts->has_extra_parts || parts->parts.empty()) {
                    _receive_counters.record_refused();
                    continue;
                }
                const auto& payload = parts->parts.back();
                try {
                    std::uint64_t                           header_offset = 0;
                    std::vector<detail::ZmqTagHeaderRecord> tags;
                    std::size_t                             consumed_bytes = 0;
                    if (pass_tags) {
                        consumed_bytes = detail::parse_tag_header(static_cast<const std::uint8_t*>(payload.data()), payload.size(), header_offset, tags, pmt_wire_format);
                        for (auto& tag : tags) {
                            if (tag.offset >= header_offset) {
                                tag.offset -= header_offset;
                            }
                        }
                    }
                    detail::ZmqSocketTransport::require_multiple_of(payload.size() - consumed_bytes, sizeof(T), "ZmqSubSource scalar payload");
                    size_t nels = (payload.size() - consumed_bytes) / sizeof(T);
                    auto   n    = std::min(nels, room_in_span);
                    auto   rem  = nels - n;
                    detail::copy_items_from_bytes(static_cast<const std::uint8_t*>(payload.data()) + consumed_bytes, n, outputSpan.data() + npublished);
                    publish_pending_tags(npublished, n, _pending_tags);
                    publish_pending_tags(npublished, n, tags);
                    npublished += n;
                    _receive_counters.record_accepted(nels);
                    if (nels == 0) {
                        break;
                    }
                    if (rem) {
                        _pending_items.resize(rem);
                        _pending_offset = 0;
                        detail::copy_items_from_bytes(static_cast<const std::uint8_t*>(payload.data()) + consumed_bytes + n * sizeof(T), rem, _pending_items.data());
                        _pending_tags = std::move(tags);
                        break;
                    }
                } catch (...) {
                    _receive_counters.record_refused();
                    continue;
                }
            }
        } else if constexpr (std::is_same_v<T, gr::pmt::Value>) {
            for (std::size_t i = 0; i < nProcessOut && messages < detail::kMaxMessagesPerWork; ++i) {
                if (!_transport.wait_readable(deadline, messages == 0 && npublished == 0, cancelled)) {
                    break;
                }

                try {
                    auto parts = detail::receive_all_parts(_transport.socket(), 2);
                    if (!parts) {
                        break;
                    }
                    ++messages;
                    if (parts->has_extra_parts || parts->parts.empty()) {
                        _receive_counters.record_refused();
                        continue;
                    }
                    const auto& payload = parts->parts.back();
                    {
                        std::uint64_t                           header_offset = 0;
                        std::vector<detail::ZmqTagHeaderRecord> tags;
                        std::size_t                             consumed_bytes = 0;
                        if (pass_tags) {
                            consumed_bytes = detail::parse_tag_header(static_cast<const std::uint8_t*>(payload.data()), payload.size(), header_offset, tags, pmt_wire_format);
                            for (auto& tag : tags) {
                                if (tag.offset >= header_offset) {
                                    tag.offset -= header_offset;
                                }
                            }
                        }
                        outputSpan[i] = detail::deserialize_pmt(static_cast<const uint8_t*>(payload.data()) + consumed_bytes, payload.size() - consumed_bytes, pmt_wire_format);
                        if (pass_tags) {
                            for (const auto& tag : tags) {
                                outputSpan.publishTag(detail::tag_map_from_record(tag), i);
                            }
                        }
                    }
                    ++npublished;
                    _receive_counters.record_accepted(1);
                } catch (...) {
                    _receive_counters.record_refused();
                    continue;
                }
            }
        }

        outputSpan.publish(npublished);
        return gr::work::Status::OK;
    }
};

} // namespace gr::blocks::zeromq

GR_REGISTER_BLOCK("gr::blocks::zeromq::ZmqSubSource", gr::blocks::zeromq::ZmqSubSource, ([T]), [ uint8_t, int16_t, int32_t, float, std::complex<float>, std::vector<float>, std::vector<std::complex<float>>, gr::pmt::Value ])
