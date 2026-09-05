#ifndef GNURADIO_DIGITAL_LENGTH_HEADER_FRAMER_HPP
#define GNURADIO_DIGITAL_LENGTH_HEADER_FRAMER_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <print>
#include <span>
#include <stdexcept>
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
#include <gnuradio-4.0/algorithm/digital/HeaderFormat.hpp>

#include <gnuradio-4.0/digital/HeaderLayout.hpp>

namespace gr::blocks::digital {

GR_REGISTER_BLOCK(gr::blocks::digital::LengthHeaderFramer)

struct LengthHeaderFramer : Block<LengthHeaderFramer> {
    using Description = Doc<R""(
@brief Prefixes each record with a sync word and a length header a `PacketFramer` can frame from.

The transmit side of `PacketFramer`, over the same header layouts and the same settings in the same spellings, so the
two ends of a chain are one configuration written twice rather than two that have to be kept in step. One record in,
one framed record out:

    | sync_word (optional) | header items | payload items |

The emitted items are bit items, because a length header is a bit field: `payload_pack_bits` of 8 unpacks a
byte-carrying record into them first, and 0 takes the record's items as wire items already. The length the header
carries is the wire payload count, after any unpacking, inverted through the same `count_covers` arithmetic the
receiving end reads it with -- a 34-byte record at `payload_pack_bits` 8, `count_covers` 'payload', `count_unit_items`
8 and `check_items` 16 writes the field value 32 and puts 272 items on the wire.

`sync_word` has no default and empty means none is emitted, which is the right shape for a chain whose preamble is
generated upstream; there is no universal sync word and a silent one would be a silent interoperability assumption.
`max_payload_items` is required too, and it is the number the receiving `PacketFramer` refuses above, which is what
makes it the number this end must not send.

Three counted drops, each publishing nothing and each named at `stop()`: a record that is empty or above the bound, a
payload whose item count does not invert to a whole field value the layout can carry, and a metadata value the layout
read and rejected -- a `packet_number` or `header_flags` too wide for its field. Metadata crosses verbatim and the
record gains `header_items`, the count the header occupied, so a loopback chain can be read frame by frame.
)"">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<std::string, "sync_word", Doc<"the sync word as '0'/'1' characters, up to 64 of them, emitted ahead of the header; empty emits none and there is no default">, Visible>              sync_word{};
    Annotated<std::string, "header_format", Doc<"'length_crc' (default), 'length_repeated', 'fixed_length', 'length_plain' or 'length_golay24'">, Visible>                                         header_format       = std::string("length_crc");
    Annotated<gr::Size_t, "fixed_payload_items", Unit<"items">, Doc<"the length, for 'fixed_length' only">>                                                                                        fixed_payload_items = 0U;
    Annotated<gr::Size_t, "length_bits", Unit<"bit">, Doc<"the length field's width, [1,32] for 'length_plain' and [1,12] for 'length_golay24'; required by both, 0 is unset">>                    length_bits         = 0U;
    Annotated<std::string, "byte_order", Doc<"'big' or 'little': the length field's byte order, for 'length_plain' only; 'little' needs a width that is a whole number of bytes">>                 byte_order          = std::string("big");
    Annotated<std::string, "flags_position", Doc<"'high' or 'low': which end of the Golay header's twelve information bits the flags occupy, for 'length_golay24' only">>                          flags_position      = std::string("high");
    Annotated<std::string, "count_covers", Doc<"what the length field counts: 'payload', 'payload_and_check' or 'whole_frame'; required by both length formats and there is no default">, Visible> count_covers{};
    Annotated<gr::Size_t, "count_unit_items", Unit<"items">, Doc<"items per counted unit, [1,64]; 8 because a wire item is one bit and a length field counts bytes">>                              count_unit_items   = 8U;
    Annotated<gr::Size_t, "check_items", Unit<"items">, Doc<"trailing check items the field does not count; only meaningful under 'payload'">>                                                     check_items        = 0U;
    Annotated<gr::Size_t, "frame_prefix_items", Unit<"items">, Doc<"frame items ahead of the payload the field does count; only under 'whole_frame', and 0 means the header's own item count">>    frame_prefix_items = 0U;
    Annotated<gr::Size_t, "max_payload_items", Unit<"items">, Doc<"the largest wire payload a frame may carry; required, 0 leaves the block inert, and a longer record is dropped">, Visible>      max_payload_items  = 0U;
    Annotated<gr::Size_t, "payload_pack_bits", Doc<"0 takes the record's items as wire bit items; 1 to 8 unpacks each record item into that many bit items first">>                                payload_pack_bits  = 0U;
    Annotated<gr::digital::BitOrder, "payload_bit_order", Doc<"the order the unpack stage takes a record item's bits in, MsbFirst or LsbFirst">>                                                   payload_bit_order  = gr::digital::BitOrder::MsbFirst;

    GR_MAKE_REFLECTABLE(LengthHeaderFramer, in, out, sync_word, header_format, fixed_payload_items, length_bits, byte_order, flags_position, count_covers, count_unit_items, check_items, frame_prefix_items, max_payload_items, payload_pack_bits, payload_bit_order);

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords               = 0ULL; ///< records framed and published; the denominator the others are read against
    std::uint64_t nItemsEmitted          = 0ULL; ///< wire items published, sync word and header included
    std::uint64_t nRecordsRefused        = 0ULL; ///< records dropped as empty or longer than the bound
    std::uint64_t nLengthUnrepresentable = 0ULL; ///< payloads whose item count does not invert to a whole field value in range
    std::uint64_t nMetaRefused           = 0ULL; ///< records carrying a metadata value the layout read and rejected

    gr::digital::HeaderFormat _format{gr::digital::LengthCrcHeader{}};
    std::size_t               _headerItems = 0UZ;
    std::vector<std::uint8_t> _sync{};
    gr::digital::BitRepack    _unpack{};
    std::vector<std::uint8_t> _frame{};
    bool                      _configured = false;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() {
        rebuild();
        if (max_payload_items == 0U) {
            throw gr::exception("max_payload_items is required and has no default: the bound the receiving end refuses above is the bound this end must not send");
        }
    }

    void rebuild() {
        const std::string_view word(sync_word.value);
        if (word.size() > 64UZ) {
            throw gr::exception(std::format("sync_word is {} bits; a detector's register is 64 wide, and there is no default sync word", word.size()));
        }
        std::vector<std::uint8_t> sync;
        sync.reserve(word.size());
        for (const char character : word) {
            if (character != '0' && character != '1') {
                throw gr::exception(std::format("sync_word must be '0' and '1' characters only, got '{}' in \"{}\"", character, word));
            }
            sync.push_back(static_cast<std::uint8_t>(character == '1' ? 1U : 0U));
        }

        const auto packBits = static_cast<unsigned>(payload_pack_bits.value);
        if (packBits > 8U) {
            throw gr::exception(std::format("payload_pack_bits is 0, for a record already carrying bit items, or in [1, 8], got {}", packBits));
        }
        gr::digital::BitRepack unpack;
        if (packBits != 0U) {
            try {
                gr::digital::configureRepack(unpack, packBits, 1U, payload_bit_order, gr::digital::BitOrder::MsbFirst);
            } catch (const std::invalid_argument& reason) {
                throw gr::exception(std::string(reason.what()));
            }
        }

        // built last, because it is the setting group most likely to be rejected and nothing above it has been written
        _format      = detail::headerLayoutFrom(detail::HeaderLayoutSettings{header_format.value, fixed_payload_items.value, max_payload_items.value, length_bits.value, byte_order.value, flags_position.value, count_covers.value, count_unit_items.value, check_items.value, frame_prefix_items.value});
        _headerItems = gr::digital::headerItemsOf(_format);
        _sync        = std::move(sync);
        _unpack      = unpack;
        _configured  = max_payload_items != 0U;

        _frame.clear();
        _frame.reserve(_sync.size() + _headerItems + static_cast<std::size_t>(max_payload_items.value));
        nRecords               = 0ULL;
        nItemsEmitted          = 0ULL;
        nRecordsRefused        = 0ULL;
        nLengthUnrepresentable = 0ULL;
        nMetaRefused           = 0ULL;
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
        append("records refused", nRecordsRefused);
        append("lengths unrepresentable", nLengthUnrepresentable);
        append("metadata refused", nMetaRefused);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::digital::LengthHeaderFramer '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) { // unconfigured: inert rather than framing against an unbounded length
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        const bool        connected = outSpan.isConnected; // read once: the record count below is derived from it
        const std::size_t available = inSpan.size();
        const std::size_t nInput    = connected ? std::min(available, outSpan.size()) : available;
        const auto        packBits  = static_cast<std::size_t>(payload_pack_bits.value);
        std::size_t       published = 0UZ;

        for (std::size_t i = 0UZ; i < nInput; ++i) {
            const DataSet<std::uint8_t>&        record  = inSpan[i];
            const std::span<const std::uint8_t> payload = payloadOf(record);

            // the wire count is computed before anything is resized, so nothing an upstream record supplied reaches a
            // resize unchecked
            const std::size_t wireItems = packBits == 0UZ ? payload.size() : payload.size() * packBits;
            if (wireItems == 0UZ || wireItems > static_cast<std::size_t>(max_payload_items.value)) {
                ++nRecordsRefused;
                continue;
            }

            _frame.assign(_sync.begin(), _sync.end());
            _frame.resize(_sync.size() + _headerItems + wireItems, std::uint8_t{0});
            const std::span<std::uint8_t> header = std::span<std::uint8_t>(_frame).subspan(_sync.size(), _headerItems);

            const property_map extra = headerExtraOf(record);
            if (!gr::digital::formatHeader<std::uint8_t>(_format, wireItems, extra, header)) {
                // `formatHeader` reports one failure for two causes, so the cause is attributed by asking it again
                // with nothing staged: what fails without the metadata is the length, and what only fails with it is
                // the metadata. This runs on the refusal path alone.
                ++(gr::digital::formatHeader<std::uint8_t>(_format, wireItems, property_map{}, header) ? nMetaRefused : nLengthUnrepresentable);
                continue;
            }

            const std::span<std::uint8_t> wire = std::span<std::uint8_t>(_frame).subspan(_sync.size() + _headerItems, wireItems);
            if (packBits == 0UZ) {
                std::ranges::copy(payload, wire.begin());
            } else {
                gr::digital::repack(_unpack, payload, wire);
            }

            nItemsEmitted += _frame.size();
            ++nRecords;

            DataSet<std::uint8_t> framed = record;
            framed.signal_values.assign(_frame.begin(), _frame.end());
            if (framed.extents.empty()) {
                framed.extents.push_back(static_cast<std::int32_t>(_frame.size()));
            } else {
                framed.extents[0UZ] = static_cast<std::int32_t>(_frame.size());
            }
            if (framed.signal_names.empty()) {
                framed.signal_names.emplace_back("frame");
                framed.signal_quantities.emplace_back("");
                framed.signal_units.emplace_back("");
            }
            // the record's items are wire items now, and an index measured in payload items names none of them
            framed.timing_events.clear();
            framed.timing_events.emplace_back();
            if (framed.meta_information.empty()) {
                framed.meta_information.emplace_back();
            }
            framed.meta_information[0UZ].insert_or_assign(property_map::key_type("header_items"), pmt::Value(static_cast<gr::Size_t>(_headerItems)));

            if (connected && published < outSpan.size()) {
                outSpan[published] = std::move(framed);
            }
            ++published;
        }

        std::ignore = inSpan.consume(nInput);
        outSpan.publish(connected ? published : 0UZ);
        // No room on the output is backpressure, not a stall: the framework counts a bare OK with nothing consumed and
        // nothing published as no progress and reports the block.
        return nInput == 0UZ && available != 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }

    /// @brief The keys a header layout reads out of a record's own metadata. A key the selected layout does not read
    /// is ignored by it; one it reads and rejects is the counted refusal above.
    [[nodiscard]] static property_map headerExtraOf(const DataSet<std::uint8_t>& record) {
        property_map extra;
        if (record.meta_information.empty()) {
            return extra;
        }
        const property_map& meta = record.meta_information[0UZ];
        for (const std::string_view name : {"packet_number", "header_flags"}) {
            if (const auto entry = meta.find(property_map::key_type(name)); entry != meta.end()) {
                extra.insert_or_assign(property_map::key_type(name), entry->second);
            }
        }
        return extra;
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

#endif // GNURADIO_DIGITAL_LENGTH_HEADER_FRAMER_HPP
