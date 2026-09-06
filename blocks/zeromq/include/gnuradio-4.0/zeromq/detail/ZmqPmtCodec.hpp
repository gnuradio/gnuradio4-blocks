#pragma once

#include "pmt_legacy_codec.hpp"

#include <gnuradio-4.0/YamlPmt.hpp>

#include <cstdint>
#include <limits>
#include <memory_resource>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace gr::blocks::zeromq {

enum class PmtWireFormat : std::uint8_t { GR3, GR4_YAML_V1 };

namespace detail {

inline constexpr std::size_t kGr4YamlV1PmtLengthSize = sizeof(std::uint32_t);

inline std::vector<std::uint8_t> serialize_gr4_yaml_v1_pmt(const gr::pmt::Value& value) {
    gr::pmt::Value::Map envelope{std::pmr::get_default_resource()};
    envelope.emplace("value", value);
    const auto yaml = gr::pmt::yaml::serialize(envelope);
    if (yaml.size() > std::numeric_limits<std::uint32_t>::max()) {
        throw std::length_error("GR4 YAML v1 PMT payload exceeds the 32-bit frame length");
    }

    const auto                length = static_cast<std::uint32_t>(yaml.size());
    std::vector<std::uint8_t> result;
    result.reserve(kGr4YamlV1PmtLengthSize + yaml.size());
    result.push_back(static_cast<std::uint8_t>(length >> 24U));
    result.push_back(static_cast<std::uint8_t>(length >> 16U));
    result.push_back(static_cast<std::uint8_t>(length >> 8U));
    result.push_back(static_cast<std::uint8_t>(length));
    result.insert(result.end(), yaml.begin(), yaml.end());
    return result;
}

inline gr::pmt::Value deserialize_gr4_yaml_v1_pmt(const std::uint8_t*& data, const std::uint8_t* end) {
    if (data == nullptr || end == nullptr || end < data || static_cast<std::size_t>(end - data) < kGr4YamlV1PmtLengthSize) {
        throw std::runtime_error("truncated GR4 YAML v1 PMT length");
    }

    const auto length = (static_cast<std::uint32_t>(data[0]) << 24U) | (static_cast<std::uint32_t>(data[1]) << 16U) | (static_cast<std::uint32_t>(data[2]) << 8U) | static_cast<std::uint32_t>(data[3]);
    data += kGr4YamlV1PmtLengthSize;
    if (static_cast<std::size_t>(end - data) < length) {
        throw std::runtime_error("truncated GR4 YAML v1 PMT payload");
    }

    const std::string_view yaml(reinterpret_cast<const char*>(data), length);
    data += length;
    auto decoded = gr::pmt::yaml::deserialize(yaml);
    if (!decoded.has_value()) {
        throw std::runtime_error("invalid GR4 YAML v1 PMT payload");
    }
    auto value = decoded->find("value");
    if (value == decoded->end() || decoded->size() != 1UZ) {
        throw std::runtime_error("invalid GR4 YAML v1 PMT envelope");
    }
    return std::move(value->second);
}

inline std::vector<std::uint8_t> serialize_pmt(const gr::pmt::Value& value, PmtWireFormat format) {
    switch (format) {
    case PmtWireFormat::GR4_YAML_V1: return serialize_gr4_yaml_v1_pmt(value);
    case PmtWireFormat::GR3: return legacy_pmt::serialize_to_legacy(value);
    }
    throw std::invalid_argument("Unknown PMT wire format");
}

inline gr::pmt::Value deserialize_pmt(const std::uint8_t* data, std::size_t size, PmtWireFormat format) {
    switch (format) {
    case PmtWireFormat::GR4_YAML_V1: {
        if (data == nullptr || size < kGr4YamlV1PmtLengthSize) {
            throw std::runtime_error("truncated GR4 YAML v1 PMT payload");
        }
        const auto* cursor = data;
        const auto* end    = data + size;
        auto        value  = deserialize_gr4_yaml_v1_pmt(cursor, end);
        if (cursor != end) {
            throw std::runtime_error("trailing bytes after GR4 YAML v1 PMT payload");
        }
        return value;
    }
    case PmtWireFormat::GR3: return legacy_pmt::deserialize_from_legacy(data, size);
    }
    throw std::invalid_argument("Unknown PMT wire format");
}

inline gr::pmt::Value deserialize_pmt(const std::uint8_t*& data, const std::uint8_t* end, PmtWireFormat format) {
    switch (format) {
    case PmtWireFormat::GR4_YAML_V1: return deserialize_gr4_yaml_v1_pmt(data, end);
    case PmtWireFormat::GR3: return legacy_pmt::deserialize_from_legacy(data, end);
    }
    throw std::invalid_argument("Unknown PMT wire format");
}

} // namespace detail
} // namespace gr::blocks::zeromq
