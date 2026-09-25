#ifndef GNURADIO_DIGITAL_CRC_BLOCKS_HPP
#define GNURADIO_DIGITAL_CRC_BLOCKS_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <print>
#include <span>
#include <string>
#include <string_view>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/digital/Crc.hpp>

namespace gr::blocks::digital {

namespace detail {

/// @brief Which end of the appended field the most significant CRC byte sits at.
enum class ByteOrder : std::uint8_t { Big, Little };

[[nodiscard]] inline ByteOrder byteOrderFromName(std::string_view name) {
    if (name == "big") {
        return ByteOrder::Big;
    }
    if (name == "little") {
        return ByteOrder::Little;
    }
    throw gr::exception(std::format("crc_byte_order must be 'big' or 'little', got '{}'", name));
}

/**
 * @brief Build the kernel, validating `width` before anything is derived from it.
 *
 * No mask, table or byte count is computed until the value has been accepted, so an out-of-range width cannot reach a
 * shift whose count equals its operand's width.
 */
[[nodiscard]] inline gr::digital::Crc buildByteWideCrc(gr::Size_t width, std::uint64_t polynomial, std::uint64_t initialValue, std::uint64_t finalXor, bool inputReflected, bool resultReflected) {
    if (width < 8U || width > 64U || width % 8U != 0U) {
        throw gr::exception(std::format("width must be a multiple of 8 in [8, 64] because the block appends whole bytes, got {}", width));
    }
    return gr::digital::Crc(static_cast<std::uint8_t>(width), polynomial, initialValue, finalXor, inputReflected, resultReflected);
}

[[nodiscard]] inline std::uint64_t readTrailingCrc(std::span<const std::uint8_t> bytes, std::size_t crcBytes, ByteOrder order) noexcept {
    const std::span<const std::uint8_t> field = bytes.last(crcBytes);
    std::uint64_t                       value = 0ULL;
    for (std::size_t i = 0UZ; i < crcBytes; ++i) {
        const std::size_t which = order == ByteOrder::Big ? i : crcBytes - 1UZ - i;
        value                   = (value << 8U) | static_cast<std::uint64_t>(field[which]);
    }
    return value;
}

inline void appendCrc(std::vector<std::uint8_t>& bytes, std::uint64_t value, std::size_t crcBytes, ByteOrder order) {
    for (std::size_t i = 0UZ; i < crcBytes; ++i) {
        const std::size_t shift = order == ByteOrder::Big ? 8UZ * (crcBytes - 1UZ - i) : 8UZ * i;
        bytes.push_back(static_cast<std::uint8_t>((value >> shift) & 0xFFULL));
    }
}

/// @brief One record's byte array, with the single-signal restriction enforced at the port rather than assumed.
[[nodiscard]] inline std::span<const std::uint8_t> payloadOf(const DataSet<std::uint8_t>& record) {
    if (record.signal_names.size() > 1UZ) {
        throw gr::exception(std::format("a CRC covers one flat byte array; this DataSet carries {} signals", record.signal_names.size()));
    }
    return std::span<const std::uint8_t>(record.signal_values);
}

inline void putRecordMeta(DataSet<std::uint8_t>& record, std::string_view key, pmt::Value value) {
    if (record.meta_information.empty()) {
        record.meta_information.emplace_back();
    }
    record.meta_information[0UZ].insert_or_assign(property_map::key_type(key), std::move(value));
}

inline void resizeRecord(DataSet<std::uint8_t>& record, std::size_t items) {
    record.signal_values.resize(items);
    if (record.extents.empty()) {
        record.extents.push_back(static_cast<std::int32_t>(items));
    } else {
        record.extents[0UZ] = static_cast<std::int32_t>(items);
    }
}

/// @brief The six-tuple's documentation, quoted by both blocks so the `initial_value` domain is stated at each of them.
using CrcSettingsDoc = Doc<R""(
The Rocksoft model's six-tuple. `poly` is MSB-first with the implicit `x^w` term omitted; `initial_value` is the seed
in the unreflected domain, which is the domain every published catalog's INIT column is in, so a catalog value is
used as printed. A seed taken from an implementation that loads it into a mirrored register is the bit-reversal of
the same number and has to be reversed first, unless it is a palindrome.
)"">;

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::digital::CrcAppend)

struct CrcAppend : Block<CrcAppend> {
    using Description = Doc<R""(
@brief Appends the CRC of each packet to it, as whole bytes, and records what it computed.

A packet is one `DataSet<std::uint8_t>` whose `signal_values` are its bytes, carried one per item on an `Async` port;
GNU Radio 4 has no PDU type and no length-tag convention, so the native variable-length record is the container.
Per-record scalars live in `meta_information[0]`, and the block writes `crc_value` and `crc_width` there, which is what
lets a receiver's `CrcCheck` be told what to expect by the stream rather than by a duplicated setting.

`width` must be a multiple of 8 and at least 8, because the block appends whole bytes; the kernel itself goes down to 3
and is reached that way by the header formats. `skip_header_bytes` excludes leading bytes from the computation and
keeps them in the output, and a record no longer than `skip_header_bytes` has nothing to protect and is dropped with a
warning naming both numbers rather than being published unprotected. `crc_byte_order` says which end of the appended
field the most significant byte sits at and defaults to `big`.
)"">;

    PortIn<DataSet<std::uint8_t>>         in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "width", Unit<"bit">, detail::CrcSettingsDoc, Visible>                                                        width             = 32U;
    Annotated<std::uint64_t, "poly", Doc<"generator polynomial, MSB-first, x^w omitted">>                                               poly              = 0x04C11DB7ULL;
    Annotated<std::uint64_t, "initial_value", Doc<"seed, in the unreflected domain">>                                                   initial_value     = 0xFFFFFFFFULL;
    Annotated<std::uint64_t, "final_xor", Doc<"XORed into the result last">>                                                            final_xor         = 0xFFFFFFFFULL;
    Annotated<bool, "input_reflected", Doc<"each message byte enters LSB first">>                                                       input_reflected   = true;
    Annotated<bool, "result_reflected", Doc<"the register is bit-reversed before final_xor">>                                           result_reflected  = true;
    Annotated<std::string, "crc_byte_order", Doc<"'big' (most significant CRC byte first) or 'little'">>                                crc_byte_order    = std::string("big");
    Annotated<gr::Size_t, "skip_header_bytes", Unit<"byte">, Doc<"leading bytes excluded from the computation and kept in the output">> skip_header_bytes = 0U;

    GR_MAKE_REFLECTABLE(CrcAppend, in, out, width, poly, initial_value, final_xor, input_reflected, result_reflected, crc_byte_order, skip_header_bytes);

    gr::digital::Crc  _crc{32U, 0x04C11DB7ULL, 0xFFFFFFFFULL, 0xFFFFFFFFULL, true, true};
    detail::ByteOrder _order    = detail::ByteOrder::Big;
    std::size_t       _crcBytes = 4UZ;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        _order    = detail::byteOrderFromName(crc_byte_order);
        _crc      = detail::buildByteWideCrc(width, poly, initial_value, final_xor, input_reflected, result_reflected);
        _crcBytes = static_cast<std::size_t>(width) / 8UZ;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const bool        connected = outSpan.isConnected; // read once: the count below is derived from it
        const std::size_t available = inSpan.size();
        const std::size_t nRecords  = connected ? std::min(available, outSpan.size()) : available;
        std::size_t       published = 0UZ;

        for (std::size_t i = 0UZ; i < nRecords; ++i) {
            const std::span<const std::uint8_t> bytes = detail::payloadOf(inSpan[i]);
            if (bytes.size() <= static_cast<std::size_t>(skip_header_bytes)) {
                std::println(stderr, "gr::blocks::digital::CrcAppend: dropping a record of {} bytes, which is no longer than skip_header_bytes = {} and has nothing to protect", bytes.size(), skip_header_bytes.value);
                continue;
            }
            const std::uint64_t value = _crc.compute(bytes.subspan(static_cast<std::size_t>(skip_header_bytes)));

            DataSet<std::uint8_t> record = inSpan[i];
            record.signal_values.reserve(bytes.size() + _crcBytes);
            detail::appendCrc(record.signal_values, value, _crcBytes, _order);
            detail::resizeRecord(record, bytes.size() + _crcBytes);
            detail::putRecordMeta(record, "crc_value", pmt::Value(value));
            detail::putRecordMeta(record, "crc_width", pmt::Value(static_cast<std::uint64_t>(width.value)));

            if (connected && published < outSpan.size()) {
                outSpan[published] = std::move(record);
            }
            ++published;
        }

        std::ignore = inSpan.consume(nRecords);
        outSpan.publish(connected ? published : 0UZ);
        // No room on the output is backpressure and an empty input is waiting, not a stall: the framework counts a bare
        // OK with nothing consumed and nothing published as no progress and reports the block. The async output counts
        // as ready whenever it has room, so the framework also calls on an empty input.
        if (available == 0UZ) {
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return nRecords == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::digital::CrcCheck)

struct CrcCheck : Block<CrcCheck> {
    using Description = Doc<R""(
@brief Recomputes each packet's CRC, compares it against the trailing bytes, and routes the record by the answer.

A failing packet leaves by the `fail` port rather than being dropped, because a receive chain that cannot see its
failures cannot be debugged. A passing record is never dropped: with no room on `ok` it waits in the input buffer
and the block asks for output room. A failing record is never held: with `fail` unconnected, or connected and
full, it is consumed and dropped, and the block reports the first such drop. A block that returns without
consuming or publishing is the scheduler's definition of no progress, and a chain whose records are nearly all
damaged would stop at this one.

The length guard is mandatory: a record must hold more than `skip_header_bytes + width/8` bytes or it is dropped with
a warning, a packet length being attacker-controlled in any receive chain. The check compares values rather than
testing the residue, so it can report what it received: `meta_information[0]` carries `crc_ok` and the received
`crc_value`.
)"">;

    PortIn<DataSet<std::uint8_t>>         in;
    PortOut<DataSet<std::uint8_t>, Async> ok;
    PortOut<DataSet<std::uint8_t>, Async> fail;

    Annotated<gr::Size_t, "width", Unit<"bit">, detail::CrcSettingsDoc, Visible>                                                    width             = 32U;
    Annotated<std::uint64_t, "poly", Doc<"generator polynomial, MSB-first, x^w omitted">>                                           poly              = 0x04C11DB7ULL;
    Annotated<std::uint64_t, "initial_value", Doc<"seed, in the unreflected domain">>                                               initial_value     = 0xFFFFFFFFULL;
    Annotated<std::uint64_t, "final_xor", Doc<"XORed into the result last">>                                                        final_xor         = 0xFFFFFFFFULL;
    Annotated<bool, "input_reflected", Doc<"each message byte enters LSB first">>                                                   input_reflected   = true;
    Annotated<bool, "result_reflected", Doc<"the register is bit-reversed before final_xor">>                                       result_reflected  = true;
    Annotated<std::string, "crc_byte_order", Doc<"'big' (most significant CRC byte first) or 'little'">>                            crc_byte_order    = std::string("big");
    Annotated<gr::Size_t, "skip_header_bytes", Unit<"byte">, Doc<"leading bytes excluded from the computation and never stripped">> skip_header_bytes = 0U;
    Annotated<bool, "discard_crc", Doc<"strip the CRC bytes, on both ports alike">>                                                 discard_crc       = false;

    GR_MAKE_REFLECTABLE(CrcCheck, in, ok, fail, width, poly, initial_value, final_xor, input_reflected, result_reflected, crc_byte_order, skip_header_bytes, discard_crc);

    gr::digital::Crc  _crc{32U, 0x04C11DB7ULL, 0xFFFFFFFFULL, 0xFFFFFFFFULL, true, true};
    detail::ByteOrder _order           = detail::ByteOrder::Big;
    std::size_t       _crcBytes        = 4UZ;
    bool              _reportedRefused = false;

    /// @brief Records that failed and were dropped because the `fail` port had no room for them.
    std::uint64_t nRefusedDropped = 0ULL;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() {
        rebuild();
        nRefusedDropped  = 0ULL;
        _reportedRefused = false;
    }

    void rebuild() {
        _order    = detail::byteOrderFromName(crc_byte_order);
        _crc      = detail::buildByteWideCrc(width, poly, initial_value, final_xor, input_reflected, result_reflected);
        _crcBytes = static_cast<std::size_t>(width) / 8UZ;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& okSpan, OutputSpanLike auto& failSpan) {
        // the guard subtracts rather than adds: skip_header_bytes is 32 bits wide, so on a target whose size_t is also
        // 32 bits a skip within width/8 bytes of the maximum would carry the sum past the length it is compared with,
        // and every record would pass a guard it cannot satisfy. shortest is formed in 64 bits, for the message alone.
        const std::size_t   skip          = static_cast<std::size_t>(skip_header_bytes);
        const std::uint64_t shortest      = static_cast<std::uint64_t>(skip_header_bytes) + static_cast<std::uint64_t>(_crcBytes);
        const std::size_t   available     = inSpan.size();
        const bool          okConnected   = okSpan.isConnected; // read once, so the room test and the store cannot disagree
        const bool          failConnected = failSpan.isConnected;
        std::size_t         consumed      = 0UZ;
        std::size_t         onOk          = 0UZ;
        std::size_t         onFail        = 0UZ;

        for (std::size_t i = 0UZ; i < available; ++i) {
            const std::span<const std::uint8_t> bytes = detail::payloadOf(inSpan[i]);
            if (bytes.size() <= skip || bytes.size() - skip <= _crcBytes) {
                std::println(stderr, "gr::blocks::digital::CrcCheck: dropping a record of {} bytes, which is no longer than skip_header_bytes + width/8 = {}", bytes.size(), shortest);
                ++consumed;
                continue;
            }

            const std::uint64_t received   = detail::readTrailingCrc(bytes, _crcBytes, _order);
            const std::uint64_t recomputed = _crc.compute(bytes.subspan(skip, bytes.size() - skip - _crcBytes));
            const bool          passed     = received == recomputed;

            if (passed) {
                if (okConnected && onOk >= okSpan.size()) {
                    break; // a record that passed is not dropped; it waits in the buffer until there is room
                }
            } else if (failConnected && onFail >= failSpan.size()) {
                // A refused record is dropped rather than held: holding it consumes nothing, and a chain whose
                // records are nearly all damaged would stop there for as long as the fail port stays full.
                if (!_reportedRefused) {
                    _reportedRefused = true;
                    std::println(stderr, "gr::blocks::digital::CrcCheck: the fail port has no room; refused records are dropped from here on rather than held, which would stop the graph");
                }
                ++nRefusedDropped;
                ++consumed;
                continue;
            }

            DataSet<std::uint8_t> record = inSpan[i];
            if (discard_crc) {
                detail::resizeRecord(record, bytes.size() - _crcBytes);
            }
            detail::putRecordMeta(record, "crc_ok", pmt::Value(passed));
            detail::putRecordMeta(record, "crc_value", pmt::Value(received));

            if (passed) {
                if (okConnected) {
                    okSpan[onOk] = std::move(record);
                }
                ++onOk;
            } else {
                if (failConnected) {
                    failSpan[onFail] = std::move(record);
                }
                ++onFail;
            }
            ++consumed;
        }

        std::ignore = inSpan.consume(consumed);
        okSpan.publish(okConnected ? onOk : 0UZ);
        failSpan.publish(failConnected ? onFail : 0UZ);
        // No room on `ok` is backpressure and an empty input is waiting, not a stall: the framework counts a bare OK
        // with nothing consumed and nothing published as no progress and reports the block. The async outputs count as
        // ready whenever they have room, so the framework also calls on an empty input.
        if (available == 0UZ) {
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return consumed == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_CRC_BLOCKS_HPP
