#pragma once
#include "../trait_helpers.hpp"
#include "ZmqCommon.hpp"
#include "ZmqTagHeaders.hpp"

namespace gr::blocks::zeromq::detail {
struct DecodedFrame {
    std::size_t                     consumed_bytes = 0;
    std::vector<ZmqTagHeaderRecord> tags;
    gr::pmt::Value                  pmt;
};

// This boundary covers only header/payload validation and decoding. In
// particular, neither socket I/O nor output/tag publication belongs here.
template<typename T>
std::optional<DecodedFrame> decode_frame(const zmq::message_t& msg, bool pass_tags, PmtWireFormat format, ZmqReceiveCounters& counters) {
    try {
        DecodedFrame  decoded;
        std::uint64_t header_offset = 0;
        if (pass_tags) {
            decoded.consumed_bytes = parse_tag_header(static_cast<const std::uint8_t*>(msg.data()), msg.size(), header_offset, decoded.tags, format);
            for (auto& tag : decoded.tags) {
                if (tag.offset >= header_offset) {
                    tag.offset -= header_offset;
                }
            }
        }
        const auto size = msg.size() - decoded.consumed_bytes;
        if constexpr (std::is_same_v<T, gr::pmt::Value>) {
            decoded.pmt = deserialize_pmt(static_cast<const std::uint8_t*>(msg.data()) + decoded.consumed_bytes, size, format);
        } else if constexpr (is_vector_of_arithmetic_or_complex_v<T>) {
            ZmqSocketTransport::require_multiple_of(size, sizeof(typename T::value_type), "ZMQ vector payload");
        } else {
            ZmqSocketTransport::require_multiple_of(size, sizeof(T), "ZMQ scalar payload");
        }
        return decoded;
    } catch (const DecodeError&) {
        counters.record_refused();
        return std::nullopt;
    }
}
} // namespace gr::blocks::zeromq::detail
