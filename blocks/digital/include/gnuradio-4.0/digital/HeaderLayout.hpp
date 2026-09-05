#ifndef GNURADIO_DIGITAL_HEADER_LAYOUT_HPP
#define GNURADIO_DIGITAL_HEADER_LAYOUT_HPP

#include <cstddef>
#include <format>
#include <stdexcept>
#include <string>
#include <string_view>

#include <gnuradio-4.0/Block.hpp>

#include <gnuradio-4.0/algorithm/digital/HeaderFormat.hpp>

namespace gr::blocks::digital::detail {

// The settings a header layout is built from, and the one place that turns them into a `gr::digital::HeaderFormat`.
// A receiving `PacketFramer` and a transmitting `LengthHeaderFramer` must agree on every one of these, down to the
// spelling, or the two ends of a chain frame different numbers of items; one builder means they cannot drift.

/// @brief The layout settings by their setting names, with the defaults the two blocks declare.
struct HeaderLayoutSettings {
    std::string format{};
    gr::Size_t  fixedPayloadItems = 0U;
    gr::Size_t  maxPayloadItems   = 0U;
    gr::Size_t  lengthBits        = 0U;
    std::string byteOrder{};
    std::string flagsPosition{};
    std::string countCovers{};
    gr::Size_t  countUnitItems   = 8U;
    gr::Size_t  checkItems       = 0U;
    gr::Size_t  framePrefixItems = 0U;
};

/// @brief What the length field counts, by the `count_covers` spelling. There is no default: the three give different
/// answers for the same wire bits, so an unset value is a refusal and not a choice.
[[nodiscard]] inline gr::digital::CountCovers countCoversFromName(std::string_view name) {
    if (name == "payload") {
        return gr::digital::CountCovers::Payload;
    }
    if (name == "payload_and_check") {
        return gr::digital::CountCovers::PayloadAndCheck;
    }
    if (name == "whole_frame") {
        return gr::digital::CountCovers::WholeFrame;
    }
    if (name.empty()) {
        throw gr::exception("count_covers is required by the length formats and has no default: 'payload', 'payload_and_check' and 'whole_frame' give different answers for the same wire bits, and a silent choice is a silent interoperability assumption");
    }
    throw gr::exception(std::format("count_covers must be 'payload', 'payload_and_check' or 'whole_frame', got '{}'", name));
}

/// @brief The length field's byte order, by the `byte_order` spelling.
[[nodiscard]] inline gr::digital::HeaderByteOrder headerByteOrderFromName(std::string_view name) {
    if (name == "big") {
        return gr::digital::HeaderByteOrder::Big;
    }
    if (name == "little") {
        return gr::digital::HeaderByteOrder::Little;
    }
    throw gr::exception(std::format("byte_order must be 'big' or 'little', got '{}'", name));
}

/// @brief Which end of the Golay header's information bits the flags occupy, by the `flags_position` spelling.
[[nodiscard]] inline gr::digital::FlagsPosition flagsPositionFromName(std::string_view name) {
    if (name == "high") {
        return gr::digital::FlagsPosition::High;
    }
    if (name == "low") {
        return gr::digital::FlagsPosition::Low;
    }
    throw gr::exception(std::format("flags_position must be 'high' or 'low', got '{}'", name));
}

/**
 * @brief The layout @p wanted names, or `gr::exception` naming the setting that could not be read.
 *
 * Every refusal names a setting and quotes the value that caused it. A setting belonging to one realization and staged
 * away from its default while another is selected is one of them: that is `DelimiterFramer`'s rule for
 * `stuff_after_ones`, and it is here for the same reason, so a chain configured for one layout and switched to another
 * says so rather than quietly ignoring half of what it was given.
 *
 * The kernel's own constructors do the range checking -- the field widths, the little-endian whole-byte rule and the
 * count parameters -- and their `std::invalid_argument` is rewrapped rather than a second set of bounds being written
 * beside them.
 */
[[nodiscard]] inline gr::digital::HeaderFormat headerLayoutFrom(const HeaderLayoutSettings& wanted) {
    const std::string_view format(wanted.format);
    const bool             lengthField = format == "length_plain" || format == "length_golay24";

    const auto refuseStray = [format](std::string_view named, std::string_view belongsTo, const auto& value) { throw gr::exception(std::format("{} belongs to {} and header_format is '{}', so a value away from the default is a settings error rather than a harmless extra, got {}", named, belongsTo, format, value)); };
    if (!lengthField) {
        constexpr std::string_view kBoth = "the length formats";
        if (wanted.lengthBits != 0U) {
            refuseStray("length_bits", kBoth, wanted.lengthBits);
        }
        if (!wanted.countCovers.empty()) {
            refuseStray("count_covers", kBoth, wanted.countCovers);
        }
        if (wanted.countUnitItems != 8U) {
            refuseStray("count_unit_items", kBoth, wanted.countUnitItems);
        }
        if (wanted.checkItems != 0U) {
            refuseStray("check_items", kBoth, wanted.checkItems);
        }
        if (wanted.framePrefixItems != 0U) {
            refuseStray("frame_prefix_items", kBoth, wanted.framePrefixItems);
        }
    }
    if (format != "length_plain" && wanted.byteOrder != "big") {
        refuseStray("byte_order", "'length_plain'", wanted.byteOrder);
    }
    if (format != "length_golay24" && wanted.flagsPosition != "high") {
        refuseStray("flags_position", "'length_golay24'", wanted.flagsPosition);
    }
    if (format != "fixed_length" && wanted.fixedPayloadItems != 0U) {
        refuseStray("fixed_payload_items", "'fixed_length'", wanted.fixedPayloadItems);
    }

    if (format == "length_crc") {
        return gr::digital::LengthCrcHeader{};
    }
    if (format == "length_repeated") {
        return gr::digital::LengthRepeatedHeader{};
    }
    if (format == "fixed_length") {
        if (wanted.fixedPayloadItems == 0U || wanted.fixedPayloadItems > wanted.maxPayloadItems) {
            throw gr::exception(std::format("fixed_payload_items {} must be between 1 and max_payload_items {}", wanted.fixedPayloadItems, wanted.maxPayloadItems));
        }
        return gr::digital::FixedLengthHeader{static_cast<std::size_t>(wanted.fixedPayloadItems)};
    }
    if (!lengthField) {
        throw gr::exception(std::format("header_format must be 'length_crc', 'length_repeated', 'fixed_length', 'length_plain' or 'length_golay24', got '{}'", format));
    }

    if (wanted.lengthBits == 0U) {
        throw gr::exception(std::format("length_bits is required by '{}' and has no default: the field's width is a wire-format fact and there is no width that is right by chance", format));
    }
    gr::digital::LengthCount count;
    count.covers           = countCoversFromName(wanted.countCovers);
    count.unitItems        = static_cast<std::size_t>(wanted.countUnitItems);
    count.checkItems       = static_cast<std::size_t>(wanted.checkItems);
    count.framePrefixItems = static_cast<std::size_t>(wanted.framePrefixItems);

    try {
        if (format == "length_plain") {
            return gr::digital::LengthPlainHeader{static_cast<std::size_t>(wanted.lengthBits), headerByteOrderFromName(wanted.byteOrder), count};
        }
        return gr::digital::LengthGolay24Header{static_cast<std::size_t>(wanted.lengthBits), flagsPositionFromName(wanted.flagsPosition), count};
    } catch (const std::invalid_argument& reason) {
        throw gr::exception(std::string(reason.what()));
    }
}

} // namespace gr::blocks::digital::detail

#endif // GNURADIO_DIGITAL_HEADER_LAYOUT_HPP
