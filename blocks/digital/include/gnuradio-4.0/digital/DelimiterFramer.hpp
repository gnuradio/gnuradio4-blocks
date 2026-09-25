#ifndef GNURADIO_DIGITAL_DELIMITER_FRAMER_HPP
#define GNURADIO_DIGITAL_DELIMITER_FRAMER_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/digital/BitPacking.hpp>
#include <gnuradio-4.0/algorithm/digital/Delimiter.hpp>

namespace gr::blocks::digital {

GR_REGISTER_BLOCK(gr::blocks::digital::DelimiterFramer)

/**
 * @brief `DelimiterExtractor`'s inverse: one record becomes one delimited, transparency-coded frame.
 *
 * The assembly, the transparency coding and the transmit-side validation are `gr::digital::DelimiterEncoder`'s over
 * the same `DelimiterConfig` the extractor scans with; what is added here is the ports, the settings, the record and
 * the counters.
 */
struct DelimiterFramer : Block<DelimiterFramer> {
    using Description = Doc<R""(
@brief Wraps each record in a delimiter a receiver can find, coding the payload so the delimiter cannot occur inside it.

The transmit side of `DelimiterExtractor`, over the same framing: HDLC, AX.25 and synchronous PPP with a bit-stuffed
`01111110` flag, SLIP and KISS with a byte-escaped `0xC0`, NMEA 0183 with `$` and `CR LF` over a printable-ASCII
payload. `gr::digital::hdlc()`, `slip()`, `pppAsync()` and `nmea0183()` are those four framings as library constants,
and none of them is a default here either, because a silent delimiter is a silent interoperability assumption.

One record in, one framed record out. The output record's items are wire items at the configured `bits_per_item` —
bit items for HDLC, byte items for SLIP, KISS and PPP — carrying the opening delimiter, the coded payload and the
closing delimiter in wire order. A framed record is a burst a transmitter can key on; turning a run of them into a
continuous stream is `DataSetToStream`'s job, and an idle link's inter-frame fill is the transmitter's policy.

`end_delimiter` and `max_payload_items` are both required and neither has a default; without either the block refuses
to `start()` and is inert if driven. The bound is the number the receiving end refuses above, which makes it the number
this end must not send: a record whose payload exceeds it, and an empty record, are counted drops that publish nothing,
because framing either would put something on the air the peer is specified to reject.

Under byte escaping the settings are held to more than a receiver can hold them to. Every item value of a delimiter,
and the escape introducer itself, must appear as an original in the escape map, because here the encoder is the peer
whose conformance a receiver could only warn about. Under `none` nothing is checkable at all, so the block counts
instead: every payload item at which the receiver's match register would hold the delimiter increments
`nForgedDelimiters`, each one a frame the receiver will cut in half.

Metadata crosses verbatim and the record gains `stuffing_inserted`, the mirror of the extractor's `stuffing_removed`,
so a loopback chain can be read frame by frame.
)"">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<std::string, "end_delimiter", Doc<"the closing delimiter as '0'/'1' characters, MSB first in wire order; required, empty leaves the block inert">, Visible>                                             end_delimiter{};
    Annotated<std::string, "frame_open", Doc<"what opens a frame: 'delimiter' (the closing delimiter again), 'start' (a separate one) or 'immediate' (nothing at all)">, Visible>                                     frame_open = std::string("delimiter");
    Annotated<std::string, "start_delimiter", Doc<"the opening delimiter; required and non-empty for frame_open 'start', and must be empty otherwise">>                                                               start_delimiter{};
    Annotated<gr::Size_t, "bits_per_item", Doc<"bits each emitted item carries, 1 to 8">>                                                                                                                             bits_per_item    = 1U;
    Annotated<gr::digital::BitOrder, "bit_order", Doc<"order of an item's bits on the wire, MsbFirst or LsbFirst; meaningful only above one bit per item">>                                                           bit_order        = gr::digital::BitOrder::MsbFirst;
    Annotated<std::string, "transparency", Doc<"how the payload is kept clear of the delimiter: 'none', 'bit_stuffing' or 'byte_escape'">, Visible>                                                                   transparency     = std::string("none");
    Annotated<gr::Size_t, "stuff_after_ones", Doc<"bit stuffing only: a 0 is inserted after the payload one that brings the consecutive-ones count to exactly this, 2 to 62">>                                        stuff_after_ones = 5U;
    Annotated<gr::Size_t, "abort_ones", Doc<"bit stuffing only: the receiver's abort run length; validated so one configuration describes both ends, and nothing here sends an abort">>                               abort_ones       = 7U;
    Annotated<gr::Size_t, "escape_item", Doc<"byte escaping only: the introducer's item value, 0 to 255">>                                                                                                            escape_item      = 0U;
    Annotated<std::vector<gr::Size_t>, "escape_map", Doc<"byte escaping only: flat (escaped, original) pairs; a repeated original encodes by the last pair naming it">>                                               escape_map{};
    Annotated<gr::Size_t, "max_payload_items", Unit<"items">, Doc<"the largest payload a frame may carry, in wire items; required, 0 leaves the block inert, and a longer record is dropped">, Visible>               max_payload_items = 0U;
    Annotated<gr::Size_t, "payload_pack_bits", Doc<"0 takes the record's items as wire items; 1 to 8 unpacks each of them into that many bit items first, and needs bits_per_item 1">>                                payload_pack_bits = 0U;
    Annotated<gr::digital::BitOrder, "payload_bit_order", Doc<"the order the unpack stage takes a record item's bits in, separate from bit_order because the payload's byte order and the wire's item order differ">> payload_bit_order = gr::digital::BitOrder::MsbFirst;

    GR_MAKE_REFLECTABLE(DelimiterFramer, in, out, end_delimiter, frame_open, start_delimiter, bits_per_item, bit_order, transparency, stuff_after_ones, abort_ones, escape_item, escape_map, max_payload_items, payload_pack_bits, payload_bit_order);

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords          = 0ULL; ///< records framed and published; the denominator the others are read against
    std::uint64_t nItemsEmitted     = 0ULL; ///< wire items published, both delimiters included
    std::uint64_t nStuffedBits      = 0ULL; ///< zeros inserted under bit stuffing
    std::uint64_t nEscapedItems     = 0ULL; ///< introducers inserted under byte escaping
    std::uint64_t nForgedDelimiters = 0ULL; ///< payload items carrying the delimiter under 'none', each a frame the receiver cuts in half
    std::uint64_t nRecordsRefused   = 0ULL; ///< records dropped as empty or longer than the bound

    gr::digital::DelimiterEncoder _encoder{};
    std::vector<std::uint8_t>     _framed{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() {
        rebuild();
        if (end_delimiter.value.empty()) {
            throw gr::exception("end_delimiter is required and has no default: there is no universal delimiter and a silent one would be a silent interoperability assumption");
        }
        if (max_payload_items == 0U) {
            throw gr::exception("max_payload_items is required and has no default: the bound the receiving end refuses above is the bound this end must not send");
        }
    }

    /// @brief Rebuilds the encoder from the settings, refusing everything the framing cannot be.
    void rebuild() {
        gr::digital::DelimiterConfig cfg;
        try {
            cfg.frameOpen    = gr::digital::frameOpenFromName(frame_open.value);
            cfg.transparency = gr::digital::transparencyFromName(transparency.value);
            cfg.bitOrder     = bit_order;
            cfg.packOrder    = payload_bit_order;
        } catch (const std::invalid_argument& error) {
            throw gr::exception(std::string(error.what()));
        }

        if (cfg.transparency != gr::digital::Transparency::BitStuffing && (stuff_after_ones.value != 5U || abort_ones.value != 7U)) {
            throw gr::exception(std::format("stuff_after_ones and abort_ones belong to 'bit_stuffing' and transparency is '{}', so a value away from the default is a settings error rather than a harmless extra", transparency.value));
        }
        if (cfg.transparency != gr::digital::Transparency::ByteEscape && (escape_item.value != 0U || !escape_map.value.empty())) {
            throw gr::exception(std::format("escape_item and escape_map belong to 'byte_escape' and transparency is '{}', so a value away from the default is a settings error rather than a harmless extra", transparency.value));
        }
        if (escape_item.value > 255U) {
            throw gr::exception(std::format("escape_item is an item value and must be in [0, 255], got {}", escape_item.value));
        }
        if (escape_map.value.size() % 2UZ != 0UZ) {
            throw gr::exception(std::format("escape_map is a flat list of (escaped, original) pairs and must have an even length, got {}", escape_map.value.size()));
        }
        if (escape_map.value.size() > 512UZ) {
            throw gr::exception(std::format("escape_map holds {} values, which is more than the 256 pairs a byte alphabet can distinguish", escape_map.value.size()));
        }
        for (const gr::Size_t value : escape_map.value) {
            if (value > 255U) {
                throw gr::exception(std::format("escape_map carries {}, which is not an item value in [0, 255]", value));
            }
        }

        cfg.endDelimiter    = end_delimiter.value;
        cfg.startDelimiter  = start_delimiter.value;
        cfg.bitsPerItem     = static_cast<unsigned>(bits_per_item.value);
        cfg.stuffAfterOnes  = static_cast<unsigned>(stuff_after_ones.value);
        cfg.abortOnes       = static_cast<unsigned>(abort_ones.value);
        cfg.escapeItem      = static_cast<std::uint8_t>(escape_item.value);
        cfg.maxPayloadItems = static_cast<std::size_t>(max_payload_items.value);
        cfg.packBits        = static_cast<unsigned>(payload_pack_bits.value);
        cfg.escapeMap.reserve(escape_map.value.size() / 2UZ);
        for (std::size_t i = 0UZ; i + 1UZ < escape_map.value.size(); i += 2UZ) {
            cfg.escapeMap.emplace_back(static_cast<std::uint8_t>(escape_map.value[i]), static_cast<std::uint8_t>(escape_map.value[i + 1UZ]));
        }

        try {
            gr::digital::framerConfigure(cfg);
        } catch (const std::invalid_argument& error) {
            throw gr::exception(std::string(error.what()));
        }

        _encoder.prepare(std::move(cfg));
        _framed.clear();
        _framed.reserve(gr::digital::framedItemsBound(_encoder.config, static_cast<std::size_t>(max_payload_items.value)));
        nRecords          = 0ULL;
        nItemsEmitted     = 0ULL;
        nStuffedBits      = 0ULL;
        nEscapedItems     = 0ULL;
        nForgedDelimiters = 0ULL;
        nRecordsRefused   = 0ULL;
    }

    void stop() {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("records framed", nRecords);
        append("items emitted", nItemsEmitted);
        append("stuffed bits", nStuffedBits);
        append("escaped items", nEscapedItems);
        append("forged delimiters", nForgedDelimiters);
        append("records refused", nRecordsRefused);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::digital::DelimiterFramer '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_encoder.configured()) { // unconfigured: inert rather than framing something arbitrary
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        const bool        connected = outSpan.isConnected; // read once: the record count below is derived from it
        const std::size_t available = inSpan.size();
        const std::size_t nInput    = connected ? std::min(available, outSpan.size()) : available;
        std::size_t       published = 0UZ;

        for (std::size_t i = 0UZ; i < nInput; ++i) {
            const std::span<const std::uint8_t> payload = payloadOf(inSpan[i]);
            const std::size_t                   items   = _encoder.unpackedItems(payload.size());
            if (items == 0UZ || items > static_cast<std::size_t>(max_payload_items.value)) {
                // a frame with no items is the receiver's idle rather than a frame, and one above the bound is a frame
                // the peer is specified to reject; both are counted here and stated at stop()
                ++nRecordsRefused;
                continue;
            }

            const gr::digital::EncodeResult framed = _encoder.encode(payload, _framed);
            nItemsEmitted += static_cast<std::uint64_t>(framed.emitted);
            nStuffedBits += _encoder.config.transparency == gr::digital::Transparency::BitStuffing ? static_cast<std::uint64_t>(framed.inserted) : 0ULL;
            nEscapedItems += _encoder.config.transparency == gr::digital::Transparency::ByteEscape ? static_cast<std::uint64_t>(framed.inserted) : 0ULL;
            nForgedDelimiters += static_cast<std::uint64_t>(framed.forged);
            ++nRecords;

            DataSet<std::uint8_t> record = inSpan[i];
            record.signal_values.assign(_framed.begin(), _framed.end());
            if (record.extents.empty()) {
                record.extents.push_back(static_cast<std::int32_t>(_framed.size()));
            } else {
                record.extents[0UZ] = static_cast<std::int32_t>(_framed.size());
            }
            if (record.signal_names.empty()) {
                record.signal_names.emplace_back("frame");
                record.signal_quantities.emplace_back("");
                record.signal_units.emplace_back("");
            }
            // the record's items are wire items now, and an index measured in payload items names none of them
            record.timing_events.clear();
            record.timing_events.emplace_back();
            if (record.meta_information.empty()) {
                record.meta_information.emplace_back();
            }
            record.meta_information[0UZ].insert_or_assign(property_map::key_type("stuffing_inserted"), pmt::Value(static_cast<gr::Size_t>(framed.inserted)));

            if (connected && published < outSpan.size()) {
                outSpan[published] = std::move(record);
            }
            ++published;
        }

        std::ignore = inSpan.consume(nInput);
        outSpan.publish(connected ? published : 0UZ);
        // No room on the output is backpressure and an empty input is waiting, not a stall: the framework counts a bare
        // OK with nothing consumed and nothing published as no progress and reports the block. The async output counts
        // as ready whenever it has room, so the framework also calls on an empty input.
        if (available == 0UZ) {
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return nInput == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }

    /// @brief One record's item array, with the single-signal restriction stated at the port rather than assumed.
    [[nodiscard]] static std::span<const std::uint8_t> payloadOf(const DataSet<std::uint8_t>& record) {
        if (record.signal_names.size() > 1UZ) {
            throw gr::exception(std::format("a frame carries one flat item array; this DataSet carries {} signals", record.signal_names.size()));
        }
        return std::span<const std::uint8_t>(record.signal_values);
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_DELIMITER_FRAMER_HPP
