#ifndef GNURADIO_DIGITAL_DELIMITER_EXTRACTOR_HPP
#define GNURADIO_DIGITAL_DELIMITER_EXTRACTOR_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <limits>
#include <optional>
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
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/digital/BitPacking.hpp>
#include <gnuradio-4.0/algorithm/digital/Delimiter.hpp>

#include <gnuradio-4.0/digital/AccessCodeCorrelator.hpp>

namespace gr::blocks::digital {

GR_REGISTER_BLOCK(gr::blocks::digital::DelimiterExtractor, [T], [ std::uint8_t, float ])

/**
 * @brief The third extractor: frames delimited by a reserved item pattern the block matches itself.
 *
 * `StreamToDataSet` cuts a fixed window from a trigger and `PacketFramer` reads a length out of a header. A
 * delimiter-framed protocol has neither, because the length is not known until the frame ends, so this block
 * matches the delimiter and cuts on it. The scanning, the transparency decoding and the length classification are
 * `gr::digital::DelimiterScanner`'s; what is added here is the ports, the settings, the records and the tags.
 */
template<BitLike T>
struct DelimiterExtractor : Block<DelimiterExtractor<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Cuts frames out of a stream on a reserved delimiter that may not occur in the payload, one `DataSet<T>` per frame.

The extractor for protocols that mark the end of a frame rather than declaring its length: HDLC, AX.25 and
synchronous PPP with a bit-stuffed `01111110` flag, SLIP and KISS with a byte-escaped `0xC0`, NMEA 0183 with `$` and
`CR LF` over a printable-ASCII payload. `gr::digital::hdlc()`, `slip()`, `pppAsync()` and `nmea0183()` are those four
framings as library constants; none of them is a default here, because there is no universal delimiter and a silent
one would be a silent interoperability assumption.

The delimiter is spelled as a bit string, most significant bit first in wire order, and `bits_per_item` says how many
bits an input item carries, so a bit-level and a byte-level delimiter are the same mechanism at 1 and at 8.
`transparency` names how the protocol keeps the delimiter out of the payload, and the settings are checked against
that claim when they are staged: under bit stuffing a delimiter a conforming encoder could produce is refused
outright, and under byte escaping, where the guarantee depends on a peer this block cannot inspect, it is warned
about.

`end_delimiter` and `max_payload_items` are both required and neither has a default; without either the block refuses
to `start()` and is inert if driven. The bound is what keeps a frame that never ends — a corrupted closing delimiter,
or a stream that is not the protocol configured — from accumulating with the stream.

Every record on `out` was closed by a delimiter the block matched. A frame that reached the bound, one shorter than
`min_payload_items`, and one whose decoded length is not a multiple of `payload_pack_bits` leave by `reject` instead,
carrying `discard_reason`. A region between two delimiters carrying no items is idle rather than a short frame and is
counted without being published, because an idle link is back-to-back delimiters. `sequence` counts records across
both ports, so a gap in `out`'s own sequence is exactly a record that went to `reject`.
)"">;

    PortIn<T>                            in;
    PortOut<DataSet<T>, Async>           out;
    PortOut<DataSet<T>, Async, Optional> reject;

    Annotated<std::string, "end_delimiter", Doc<"the closing delimiter as '0'/'1' characters, MSB first in wire order; required, empty leaves the block inert">, Visible>                                                                                                               end_delimiter{};
    Annotated<std::string, "frame_open", Doc<"'delimiter' (the closing delimiter opens the next frame), 'start' or 'immediate'">, Visible>                                                                                                                                              frame_open = std::string("delimiter");
    Annotated<std::string, "start_delimiter", Doc<"the opening delimiter; required and non-empty for frame_open 'start', and must be empty otherwise">>                                                                                                                                 start_delimiter{};
    Annotated<gr::Size_t, "bits_per_item", Doc<"bits each input item carries, 1 to 8; a soft stream carries one bit per item and any other value is refused">>                                                                                                                          bits_per_item    = 1U;
    Annotated<gr::digital::BitOrder, "bit_order", Doc<"order of an item's bits on the wire, MsbFirst or LsbFirst; meaningful only above one bit per item">>                                                                                                                             bit_order        = gr::digital::BitOrder::MsbFirst;
    Annotated<std::string, "transparency", Doc<"how the payload avoids the delimiter: 'none', 'bit_stuffing' or 'byte_escape'">, Visible>                                                                                                                                               transparency     = std::string("none");
    Annotated<gr::Size_t, "stuff_after_ones", Doc<"bit stuffing only: a 0 arriving at exactly this consecutive-ones count is removed, 2 to 62">>                                                                                                                                        stuff_after_ones = 5U;
    Annotated<gr::Size_t, "abort_ones", Doc<"bit stuffing only: this many consecutive 1s abandons the frame in progress; 0 disables, and it must exceed stuff_after_ones">>                                                                                                             abort_ones       = 7U;
    Annotated<gr::Size_t, "escape_item", Doc<"byte escaping only: the introducer's item value, 0 to 255">>                                                                                                                                                                              escape_item      = 0U;
    Annotated<std::vector<gr::Size_t>, "escape_map", Doc<"byte escaping only: flat (escaped, original) pairs, at most 256 of them, every value 0 to 255 and no escaped value repeated">>                                                                                                escape_map{};
    Annotated<gr::Size_t, "max_payload_items", Unit<"items">, Doc<"the largest payload a frame may carry, in decoded items; required, 0 leaves the block inert">, Visible>                                                                                                              max_payload_items = 0U;
    Annotated<gr::Size_t, "min_payload_items", Unit<"items">, Doc<"a closed region shorter than this is refused as undersize, in the decoded items max_payload_items counts; 1 accepts every non-empty region, and an HDLC link at bits_per_item 1 sets 24 for a three-octet minimum">> min_payload_items = 1U;
    Annotated<gr::Size_t, "payload_pack_bits", Doc<"0 emits decoded items unchanged; 1 to 8 repacks the decoded bits into items of that width, and needs bits_per_item 1 and byte items">>                                                                                              payload_pack_bits = 0U;
    Annotated<gr::digital::BitOrder, "payload_bit_order", Doc<"the order the pack stage assembles bits in, separate from bit_order because the wire's item order and the payload's byte order differ">>                                                                                 payload_bit_order = gr::digital::BitOrder::MsbFirst;
    Annotated<std::string, "trigger", Doc<"the trigger_name that resynchronizes the machine; empty accepts any trigger tag">>                                                                                                                                                           trigger{};
    Annotated<bool, "trigger_resets", Doc<"whether a matching trigger tag resets the machine at all">>                                                                                                                                                                                  trigger_resets = true;
    Annotated<std::string, "frame_label", Doc<"written under the record's trigger_name key when no detector supplied one; deliberately not named for that reserved key">>                                                                                                               frame_label    = std::string("delimiter");

    GR_MAKE_REFLECTABLE(DelimiterExtractor, in, out, reject, end_delimiter, frame_open, start_delimiter, bits_per_item, bit_order, transparency, stuff_after_ones, abort_ones, escape_item, escape_map, max_payload_items, min_payload_items, payload_pack_bits, payload_bit_order, trigger, trigger_resets, frame_label);

    // Counted, stated drops. Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nFramesEmitted       = 0ULL; ///< records published on `out`; the denominator the others are read against
    std::uint64_t nOverrunFrames       = 0ULL; ///< frames that reached `max_payload_items` with no closing delimiter
    std::uint64_t nUndersizeFrames     = 0ULL; ///< closed frames shorter than `min_payload_items`
    std::uint64_t nUnalignedFrames     = 0ULL; ///< closed frames whose decoded count is not a multiple of `payload_pack_bits`
    std::uint64_t nIdleDelimiters      = 0ULL; ///< delimiters closing a zero-item region
    std::uint64_t nAborts              = 0ULL; ///< abort runs under bit stuffing
    std::uint64_t nRestarts            = 0ULL; ///< start delimiters arriving while a frame was open
    std::uint64_t nTriggerResets       = 0ULL; ///< trigger tags that resynchronized the machine
    std::uint64_t nEscapeViolations    = 0ULL; ///< escaped items with no entry in the escape map
    std::uint64_t nItemsMasked         = 0ULL; ///< items whose bits above `bits_per_item` were not zero
    std::uint64_t nItemsDiscarded      = 0ULL; ///< items consumed outside a frame, or belonging to a frame that was abandoned
    std::uint64_t nTagsOutsideFrames   = 0ULL; ///< input tags outside any emitted frame's payload region
    std::uint64_t nTagsOnRemovedItems  = 0ULL; ///< input tags on a removed item, carried to the next surviving item
    std::uint64_t nDroppedTimingEvents = 0ULL; ///< events whose computed index fell outside the record

    /// @brief One input tag waiting for its frame to close, at the record index the machine computed when it arrived.
    struct Pending {
        std::size_t  index   = 0UZ;
        bool         removed = false; ///< whether the tag's own item was removed, which displaces it by one item
        property_map map{};
    };

    /// @brief One arriving tag, as a position in the current span and a borrowed map.
    struct Arriving {
        std::size_t         at  = 0UZ;
        const property_map* map = nullptr;
    };

    /// @brief The event list is bounded by `max_payload_items`; this is how much of that bound is reserved up front.
    static constexpr std::size_t kPendingReserve = 4096UZ;

    gr::digital::DelimiterScanner<T> _scanner{};
    std::vector<Pending>             _pending{};
    std::vector<Arriving>            _tags{};
    property_map                     _triggerMeta{};
    std::optional<float>             _rate{};
    std::uint64_t                    _sequence = 0ULL;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() {
        rebuild();
        if (end_delimiter.value.empty()) {
            throw gr::exception("end_delimiter is required and has no default: there is no universal delimiter and a silent one would be a silent interoperability assumption");
        }
        if (max_payload_items == 0U) {
            throw gr::exception("max_payload_items is required and has no default: without it a frame whose closing delimiter was corrupted accumulates with the stream");
        }
    }

    void reset() {
        _scanner.reset();
        _pending.clear();
        _triggerMeta.clear();
        _rate.reset();
        _sequence = 0ULL;
    }

    /// @brief Rebuilds the scanner from the settings, refusing everything the framing cannot be.
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

        if constexpr (std::floating_point<T>) {
            if (bits_per_item.value != 1U) {
                throw gr::exception(std::format("a soft item carries one sliced bit and nothing else, so bits_per_item must be 1 on a floating-point stream, got {}", bits_per_item.value));
            }
            if (cfg.transparency == gr::digital::Transparency::ByteEscape) {
                throw gr::exception("byte escaping needs byte items and this stream is floating point");
            }
            if (payload_pack_bits.value != 0U) {
                throw gr::exception("the pack stage assembles bits into bytes and needs byte items; this stream is floating point");
            }
        }

        cfg.endDelimiter    = end_delimiter.value;
        cfg.startDelimiter  = start_delimiter.value;
        cfg.bitsPerItem     = static_cast<unsigned>(bits_per_item.value);
        cfg.stuffAfterOnes  = static_cast<unsigned>(stuff_after_ones.value);
        cfg.abortOnes       = static_cast<unsigned>(abort_ones.value);
        cfg.escapeItem      = static_cast<std::uint8_t>(escape_item.value);
        cfg.maxPayloadItems = static_cast<std::size_t>(max_payload_items.value);
        cfg.minPayloadItems = static_cast<std::size_t>(min_payload_items.value);
        cfg.packBits        = static_cast<unsigned>(payload_pack_bits.value);
        cfg.escapeMap.reserve(escape_map.value.size() / 2UZ);
        for (std::size_t i = 0UZ; i + 1UZ < escape_map.value.size(); i += 2UZ) {
            cfg.escapeMap.emplace_back(static_cast<std::uint8_t>(escape_map.value[i]), static_cast<std::uint8_t>(escape_map.value[i + 1UZ]));
        }

        try {
            gr::digital::configureDelimiter(cfg);
        } catch (const std::invalid_argument& error) {
            throw gr::exception(std::string(error.what()));
        }

        if (const std::string warning = gr::digital::unforgeabilityWarning(cfg); !warning.empty()) {
            std::println(stderr, "gr::blocks::digital::DelimiterExtractor '{}': {}", this->name, warning);
        }

        _scanner.prepare(std::move(cfg));
        _pending.clear();
        _pending.reserve(std::min(static_cast<std::size_t>(max_payload_items.value), kPendingReserve));
        _triggerMeta.clear();
        _rate.reset();
        _sequence            = 0ULL;
        nFramesEmitted       = 0ULL;
        nTriggerResets       = 0ULL;
        nTagsOutsideFrames   = 0ULL;
        nTagsOnRemovedItems  = 0ULL;
        nDroppedTimingEvents = 0ULL;
        refreshCounters();
    }

    void stop() {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("frames emitted", nFramesEmitted);
        append("overrun frames", nOverrunFrames);
        append("undersize frames", nUndersizeFrames);
        append("unaligned frames", nUnalignedFrames);
        append("idle delimiters", nIdleDelimiters);
        append("aborts", nAborts);
        append("restarts", nRestarts);
        append("trigger resets", nTriggerResets);
        append("escape violations", nEscapeViolations);
        append("items masked", nItemsMasked);
        append("items discarded", nItemsDiscarded);
        append("tags outside frames", nTagsOutsideFrames);
        append("tags on removed items", nTagsOnRemovedItems);
        append("dropped timing events", nDroppedTimingEvents);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::digital::DelimiterExtractor '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& rejectSpan) {
        if (!_scanner.configured()) { // unconfigured: inert rather than matching or accumulating something arbitrary
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            rejectSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        _tags.clear();
        for (const auto& [relIndex, tagMap] : inSpan.tags()) {
            // a block holding a partial frame across calls is shown an earlier call's tag again at a negative index
            if (relIndex >= 0 && static_cast<std::size_t>(relIndex) < inSpan.size()) {
                _tags.push_back({static_cast<std::size_t>(relIndex), &tagMap.get()});
            }
        }
        std::ranges::stable_sort(_tags, std::ranges::less{}, &Arriving::at);

        const bool        outConnected = outSpan.isConnected;
        const bool        rejConnected = rejectSpan.isConnected;
        const std::size_t outRoom      = outConnected ? outSpan.size() : std::numeric_limits<std::size_t>::max();
        const std::size_t rejRoom      = rejConnected ? rejectSpan.size() : std::numeric_limits<std::size_t>::max();

        std::size_t onOut     = 0UZ;
        std::size_t onReject  = 0UZ;
        std::size_t consumed  = 0UZ;
        std::size_t tagCursor = 0UZ;
        bool        starved   = false;

        _scanner.seek(inSpan.streamIndex);
        for (std::size_t i = 0UZ; i < inSpan.size(); ++i) {
            if (onOut >= outRoom || onReject >= rejRoom) { // any item may close a frame, so both ports need room for one
                starved = true;
                break;
            }

            const std::size_t tagBegin = tagCursor;
            while (tagCursor < _tags.size() && _tags[tagCursor].at == i) {
                ++tagCursor;
            }
            const std::size_t tagEnd = tagCursor;

            bool resetHere = false;
            for (std::size_t k = tagBegin; k < tagEnd; ++k) {
                readRate(*_tags[k].map);
                if (!resetHere && trigger_resets && matches(*_tags[k].map)) {
                    _triggerMeta = *_tags[k].map;
                    _scanner.resynchronize();
                    dropPending();
                    ++nTriggerResets;
                    resetHere = true;
                }
            }

            const gr::digital::ScanStep step = _scanner.push(inSpan[i]);

            for (std::size_t k = tagBegin; k < tagEnd; ++k) {
                if (_tags[k].map->empty() || (resetHere && matches(*_tags[k].map))) {
                    continue; // the trigger tag is consumed into the record's metadata rather than becoming an event
                }
                if (!step.inFrame) {
                    ++nTagsOutsideFrames;
                    continue;
                }
                if (_pending.size() >= static_cast<std::size_t>(max_payload_items.value)) {
                    ++nDroppedTimingEvents; // the framework permits several tags at one index, so the list is bounded by fiat
                    continue;
                }
                _pending.push_back({step.index, step.removed, *_tags[k].map});
            }

            switch (step.event) {
            case gr::digital::ScanEvent::Record: {
                DataSet<T> record = buildRecord();
                if (_scanner.frameOutcome() == gr::digital::FrameOutcome::Emitted) {
                    if (outConnected && onOut < outSpan.size()) {
                        outSpan[onOut] = std::move(record);
                    }
                    ++onOut;
                    ++nFramesEmitted;
                } else {
                    if (rejConnected && onReject < rejectSpan.size()) {
                        rejectSpan[onReject] = std::move(record);
                    }
                    ++onReject;
                }
                break;
            }
            case gr::digital::ScanEvent::Idle:
            case gr::digital::ScanEvent::Abort:
            case gr::digital::ScanEvent::Restart: dropPending(); break;
            default: break;
            }

            consumed = i + 1UZ;
        }

        refreshCounters();
        std::ignore = inSpan.consume(consumed);
        outSpan.publish(outConnected ? onOut : 0UZ);
        rejectSpan.publish(rejConnected ? onReject : 0UZ);
        if (consumed == 0UZ && onOut == 0UZ && onReject == 0UZ) {
            return starved ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

    /// @brief The record the closed frame becomes, with its metadata, its timing events and its sequence number.
    [[nodiscard]] DataSet<T> buildRecord() {
        const std::span<const T> payload = _scanner.frame();
        const std::size_t        decoded = _scanner.frameDecodedItems();
        // a packed item holds `payload_pack_bits` decoded items, so a tag on any of them lands on that one item
        const std::size_t width = _scanner.framePacked() ? static_cast<std::size_t>(payload_pack_bits.value) : 1UZ;

        DataSet<T> record;
        record.signal_values.assign(payload.begin(), payload.end());
        record.extents.push_back(static_cast<std::int32_t>(payload.size()));
        record.signal_names.emplace_back("payload");
        record.signal_quantities.emplace_back("");
        record.signal_units.emplace_back("");
        record.meta_information.emplace_back();
        record.timing_events.emplace_back();
        record.timestamp = triggerTime();

        const std::uint64_t start    = _scanner.frameStart();
        const std::uint64_t sequence = _sequence;
        property_map&       meta     = record.meta_information[0UZ];
        meta.insert_or_assign(property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), triggerName());
        meta.insert_or_assign(property_map::key_type("sample_start"), pmt::Value(start));
        meta.insert_or_assign(property_map::key_type("sequence"), pmt::Value(sequence));
        meta.insert_or_assign(property_map::key_type("stuffing_removed"), pmt::Value(static_cast<gr::Size_t>(_scanner.frameRemoved())));
        if (_rate.has_value()) {
            meta.insert_or_assign(property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()), pmt::Value(*_rate));
        }
        if (const std::optional<pmt::Value> errors = syncErrors(); errors.has_value()) {
            meta.insert_or_assign(property_map::key_type("sync_errors"), *errors);
        }
        if (const std::string_view reason = gr::digital::discardReasonName(_scanner.frameOutcome()); !reason.empty()) {
            meta.insert_or_assign(property_map::key_type("discard_reason"), pmt::Value(reason));
        }

        auto& events = record.timing_events[0UZ];
        for (Pending& event : _pending) {
            if (event.index >= decoded) {
                // a tag on the frame's last raw item when that item was removed lands one past the end, and a tag
                // inside the closing delimiter's own items lands further out still; neither is clamped
                ++(event.removed ? nDroppedTimingEvents : nTagsOutsideFrames);
                continue;
            }
            if (event.removed) {
                ++nTagsOnRemovedItems;
            }
            events.emplace_back(static_cast<std::ptrdiff_t>(event.index / width), std::move(event.map));
        }
        _pending.clear();

        ++_sequence; // one counter across both ports, so a gap in `out`'s own sequence is exactly a rejected record
        _triggerMeta.clear();
        return record;
    }

    /// @brief Discards the events of a frame that never became a record, counting them as tags outside a frame.
    void dropPending() {
        nTagsOutsideFrames += _pending.size();
        _pending.clear();
    }

    /// @brief Whether @p tagMap is a trigger tag this block accepts; an empty `trigger` setting accepts any.
    [[nodiscard]] bool matches(const property_map& tagMap) const {
        const auto entry = tagMap.find(property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()));
        if (entry == tagMap.end()) {
            return false;
        }
        if (trigger.value.empty()) {
            return true;
        }
        return entry->second.value_or(std::string_view{}) == std::string_view(trigger.value);
    }

    /// @brief Holds the most recent input `sample_rate` as provenance about the stream the frames were taken from.
    void readRate(const property_map& tagMap) {
        const auto entry = tagMap.find(property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()));
        if (entry == tagMap.end()) {
            return;
        }
        if (const float* value = entry->second.template get_if<float>(); value != nullptr) {
            _rate = *value;
        }
    }

    /// @brief The trigger's own label, or `frame_label` when no detector supplied one.
    [[nodiscard]] pmt::Value triggerName() const {
        const auto entry = _triggerMeta.find(property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()));
        return entry == _triggerMeta.end() ? pmt::Value(std::string_view(frame_label.value)) : entry->second;
    }

    /// @brief The detector's Hamming distance, which travels inside `trigger_meta_info` rather than as a tag key.
    [[nodiscard]] std::optional<pmt::Value> syncErrors() const {
        const auto outer = _triggerMeta.find(property_map::key_type(gr::tag::TRIGGER_META_INFO.shortKey()));
        if (outer == _triggerMeta.end()) {
            return std::nullopt;
        }
        const auto* nested = outer->second.template get_if<property_map>();
        if (nested == nullptr) {
            return std::nullopt;
        }
        const auto inner = nested->find(property_map::key_type("sync_errors"));
        return inner == nested->end() ? std::nullopt : std::optional<pmt::Value>(inner->second);
    }

    /// @brief The trigger's `trigger_time`, or zero when none was supplied; an absent time is a stated absence.
    [[nodiscard]] std::int64_t triggerTime() const {
        const auto entry = _triggerMeta.find(property_map::key_type(gr::tag::TRIGGER_TIME.shortKey()));
        if (entry != _triggerMeta.end()) {
            if (const auto* stamp = entry->second.template get_if<std::uint64_t>(); stamp != nullptr) {
                return static_cast<std::int64_t>(*stamp);
            }
        }
        return 0;
    }

    /// @brief Mirrors the scanner's counters onto this block's own, once per call rather than once per item.
    void refreshCounters() noexcept {
        const gr::digital::DelimiterCounters& counted = _scanner.counters;
        nOverrunFrames                                = counted.nOverrunFrames;
        nUndersizeFrames                              = counted.nUndersizeFrames;
        nUnalignedFrames                              = counted.nUnalignedFrames;
        nIdleDelimiters                               = counted.nIdleDelimiters;
        nAborts                                       = counted.nAborts;
        nRestarts                                     = counted.nRestarts;
        nEscapeViolations                             = counted.nEscapeViolations;
        nItemsMasked                                  = counted.nItemsMasked;
        nItemsDiscarded                               = counted.nItemsDiscarded;
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_DELIMITER_EXTRACTOR_HPP
