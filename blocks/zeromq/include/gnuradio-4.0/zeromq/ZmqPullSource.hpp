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
concept ZmqPullSourceAcceptableTypes = std::is_same_v<T, std::vector<typename T::value_type, typename T::allocator_type>> || std::is_same_v<T, std::complex<typename T::value_type>> || std::is_integral<T>::value || std::is_floating_point<T>::value || std::is_same_v<T, gr::pmt::Value>;

template<ZmqPullSourceAcceptableTypes T>

class ZmqPullSource : public gr::Block<ZmqPullSource<T>> {
public:
    using Description = Doc<R""(
@brief ZMQ PULL Source.

This block receives ZMQ messages using a PULL socket and converts to type T.

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

    detail::ZmqSocketTransport _transport{zmq::socket_type::pull};
    detail::ZmqReceiveCounters _receive_counters;

    [[maybe_unused]] std::vector<T>                          _pending_items;
    [[maybe_unused]] std::vector<detail::ZmqTagHeaderRecord> _pending_tags;
    std::size_t                                              _pending_offset = 0;

    GR_MAKE_REFLECTABLE(ZmqPullSource, out, endpoint, timeout, bind, pass_tags, pmt_wire_format, linger, hwm, max_message_size);

    void start() {
        detail::ZmqSocketTransport::require_nonnegative_timeout(timeout);
        _pending_items.clear();
        _pending_tags.clear();
        _pending_offset = 0;
        _receive_counters.reset();
        _transport.open(endpoint, bind, linger, hwm, false, max_message_size);
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

        size_t      npublished = 0;
        std::size_t messages   = 0;

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
                if (!_pending_items.empty()) {
                    auto& vec = outputSpan[npublished];
                    vec       = _pending_items.front();
                    publish_pending_tags(npublished, 1, _pending_tags);
                    ++npublished;
                    _pending_items.erase(_pending_items.begin());
                    if (npublished >= nProcessOut) {
                        break;
                    }
                }
                if (_transport.wait_readable(deadline, messages == 0 && npublished == 0, cancelled)) {
                    auto parts = detail::receive_all_parts(_transport.socket(), 1);
                    if (!parts) {
                        break;
                    }
                    ++messages;
                    if (parts->has_extra_parts || parts->parts.size() != 1) {
                        _receive_counters.record_refused();
                        continue;
                    }
                    try {
                        const auto&                             msg           = parts->parts.front();
                        std::uint64_t                           header_offset = 0;
                        std::vector<detail::ZmqTagHeaderRecord> tags;
                        std::size_t                             consumed_bytes = 0;
                        if (pass_tags) {
                            consumed_bytes = detail::parse_tag_header(static_cast<const std::uint8_t*>(msg.data()), msg.size(), header_offset, tags, pmt_wire_format);
                            for (auto& tag : tags) {
                                if (tag.offset >= header_offset) {
                                    tag.offset -= header_offset;
                                }
                            }
                        }
                        const auto* payload      = static_cast<const std::uint8_t*>(msg.data()) + consumed_bytes;
                        const auto  payload_size = msg.size() - consumed_bytes;
                        detail::ZmqSocketTransport::require_multiple_of(payload_size, sizeof(typename T::value_type), "ZmqPullSource vector payload");

                        auto&  vec  = outputSpan[npublished];
                        size_t nels = payload_size / sizeof(typename T::value_type);
                        vec.resize(nels);
                        detail::copy_items_from_bytes(payload, nels, vec.data());
                        if (pass_tags) {
                            for (const auto& tag : tags) {
                                outputSpan.publishTag(detail::tag_map_from_record(tag), npublished);
                            }
                        }
                        ++npublished;
                        _receive_counters.record_accepted(1);
                        if (nels == 0) {
                            break;
                        }
                    } catch (...) {
                        _receive_counters.record_refused();
                        continue;
                    }

                } else {
                    break;
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
                if (_transport.wait_readable(deadline, messages == 0 && npublished == 0, cancelled)) {
                    auto parts = detail::receive_all_parts(_transport.socket(), 1);
                    if (!parts) {
                        break;
                    }
                    ++messages;
                    if (parts->has_extra_parts || parts->parts.size() != 1) {
                        _receive_counters.record_refused();
                        continue;
                    }
                    try {
                        const auto&                             msg           = parts->parts.front();
                        std::uint64_t                           header_offset = 0;
                        std::vector<detail::ZmqTagHeaderRecord> tags;
                        std::size_t                             consumed_bytes = 0;
                        if (pass_tags) {
                            consumed_bytes = detail::parse_tag_header(static_cast<const std::uint8_t*>(msg.data()), msg.size(), header_offset, tags, pmt_wire_format);
                            for (auto& tag : tags) {
                                if (tag.offset >= header_offset) {
                                    tag.offset -= header_offset;
                                }
                            }
                        }
                        const auto* payload      = static_cast<const std::uint8_t*>(msg.data()) + consumed_bytes;
                        const auto  payload_size = msg.size() - consumed_bytes;
                        detail::ZmqSocketTransport::require_multiple_of(payload_size, sizeof(T), "ZmqPullSource scalar payload");

                        auto&  vec  = outputSpan;
                        size_t nels = payload_size / sizeof(T);
                        auto   n    = std::min(nels, room_in_span);
                        auto   rem  = nels - n;

                        detail::copy_items_from_bytes(payload, n, vec.data() + npublished);
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
                            detail::copy_items_from_bytes(payload + n * sizeof(T), rem, _pending_items.data());
                            _pending_tags = std::move(tags);
                            break;
                        }
                    } catch (...) {
                        _receive_counters.record_refused();
                        continue;
                    }

                } else {
                    break;
                }
            }
        } else if constexpr (std::is_same_v<T, gr::pmt::Value>) {
            for (std::size_t i = 0; i < nProcessOut && messages < detail::kMaxMessagesPerWork; ++i) {
                if (_transport.wait_readable(deadline, messages == 0 && npublished == 0, cancelled)) {
                    try {
                        auto parts = detail::receive_all_parts(_transport.socket(), 1);
                        if (!parts) {
                            break;
                        }
                        ++messages;
                        if (parts->has_extra_parts || parts->parts.size() != 1) {
                            _receive_counters.record_refused();
                            continue;
                        }
                        const auto&                             msg           = parts->parts.front();
                        std::uint64_t                           header_offset = 0;
                        std::vector<detail::ZmqTagHeaderRecord> tags;
                        std::size_t                             consumed_bytes = 0;
                        if (pass_tags) {
                            consumed_bytes = detail::parse_tag_header(static_cast<const std::uint8_t*>(msg.data()), msg.size(), header_offset, tags, pmt_wire_format);
                            for (auto& tag : tags) {
                                if (tag.offset >= header_offset) {
                                    tag.offset -= header_offset;
                                }
                            }
                        }
                        outputSpan[i] = detail::deserialize_pmt(static_cast<const uint8_t*>(msg.data()) + consumed_bytes, msg.size() - consumed_bytes, pmt_wire_format);
                        if (pass_tags) {
                            for (const auto& tag : tags) {
                                outputSpan.publishTag(detail::tag_map_from_record(tag), npublished);
                            }
                        }
                        npublished++;
                        _receive_counters.record_accepted(1);
                    } catch (...) {
                        _receive_counters.record_refused();
                        continue;
                    }
                } else {
                    break;
                }
            }
        }

        outputSpan.publish(npublished);

        return gr::work::Status::OK;
    }
};

} // namespace gr::blocks::zeromq

GR_REGISTER_BLOCK("gr::blocks::zeromq::ZmqPullSource", gr::blocks::zeromq::ZmqPullSource, ([T]), [ uint8_t, int16_t, int32_t, float, std::complex<float>, std::vector<float>, std::vector<std::complex<float>>, gr::pmt::Value ])
