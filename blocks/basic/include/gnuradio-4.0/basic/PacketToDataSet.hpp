#ifndef GNURADIO_PACKETTODATASET_HPP
#define GNURADIO_PACKETTODATASET_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <optional>
#include <print>
#include <ranges>
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
#include <gnuradio-4.0/meta/utils.hpp>

// The record-metadata vocabulary table is included rather than copied because the table is the contract: a second
// copy here would be a second place to edit when a key is added, and every boundary a record's metadata crosses has
// to agree on it.
#include <gnuradio-4.0/basic/RecordMetadata.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::PacketToDataSet, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
requires(std::is_arithmetic_v<T> || gr::meta::complex_like<T>)
struct PacketToDataSet : Block<PacketToDataSet<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Converts each incoming gr::Packet<T> into a single-signal, one-dimensional DataSet<T>.

The inverse of `DataSetToPacket`, and the block that lets a packet off a transport re-enter a chain of blocks that
speak records — a CRC check, a decoder, a record sink. The forward conversion discards: fourteen record fields reach
four packet fields. This one fills, and the discipline is which of the ten empty fields it may write.

**A record field is written only where the packet states the fact, or where the field restates the payload's own
length.** `default_value` and `timestamp` cross into the fields of the same name, the payload becomes
`signal_values`, and `extents` is its length. `meta_information[0]` crosses key for key. Four vocabulary keys are
additionally lifted into the typed fields that hold the same facts — `signal_name`, `signal_quantity`,
`signal_unit`, and the `signal_min`/`signal_max` pair — and each stays in the map as well, because the vocabulary's
default handling is copy. **No metadata key is written, transformed or consumed.**

**No axis is fabricated and no timing event is synthesized.** A time axis would need an origin no packet states, it
would be a second spelling for `sample_rate` that could disagree with the first, and it would cost one value per
sample to say what one scalar says. The annotations were dropped at the forward boundary and stay dropped; the
vocabulary key `dropped_events` crosses like any other, so a record with an empty event list and `dropped_events`
set says exactly what happened to it. `sample_rate` and `sample_start` ride as metadata and are not turned into
structure.

A copied key that names a record-metadata vocabulary key but holds the wrong type is dropped and counted: an exactly
typed read returns nothing for it, so it would reach the record as neither a value nor an error. `signal_min` and
`signal_max` are `float` and a `Range<T>` holds `T`, so a value the record's own type cannot represent is dropped
with them rather than cast.

Three admission clauses, each a fact the block indexes on: exactly one metadata map, because `Packet<T>` has no field
that sizes the vector and `[0]` would otherwise be a guess; a non-empty payload unless `allow_empty_payload` says
otherwise; and a payload short enough to state in `extents`, which is a `std::int32_t`. A packet failing one leaves
by the `reject` port, unchanged, with `discard_reason` on a tag beside it.
)"">;

    PortIn<Packet<T>>                   in;
    PortOut<DataSet<T>, Async>          out;
    PortOut<Packet<T>, Async, Optional> reject;

    Annotated<std::string, "signal_label", Visible, Doc<"the record's signal name when the packet states no signal_name; a record with no signal name is unusable, so one is always written">> signal_label{"packet"};
    Annotated<bool, "allow_empty_payload", Doc<"admit a packet with no items, producing a record whose extent is zero">>                                                                       allow_empty_payload{false};

    GR_MAKE_REFLECTABLE(PacketToDataSet, in, out, reject, signal_label, allow_empty_payload);

    // Counted, stated drops. Plain members, read by the owning thread and reported once at stop(). There is no
    // counter for a key a derived value displaced, because this block derives none.
    std::uint64_t nRejectedPackets        = 0ULL; ///< packets the admission predicate turned away
    std::uint64_t nMetaKeysDropped        = 0ULL; ///< vocabulary keys whose type disagreed, and range pairs the record's T cannot hold
    std::uint64_t nSignalNamesSynthesized = 0ULL; ///< records whose signal name came from signal_label

    void stop() {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("rejected packets", nRejectedPackets);
        append("metadata keys dropped", nMetaKeysDropped);
        append("signal names synthesized", nSignalNamesSynthesized);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::basic::PacketToDataSet '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& rejectSpan) {
        const bool outConnected    = outSpan.isConnected; // read once, so the room test and the store cannot disagree
        const bool rejectConnected = rejectSpan.isConnected;

        std::size_t consumed = 0UZ;
        std::size_t onOut    = 0UZ;
        std::size_t onReject = 0UZ;

        auto       tagView = inSpan.tags(); // ascending by index: a tag at relative index r belongs to packet r
        auto       tagIt   = std::ranges::begin(tagView);
        const auto tagEnd  = std::ranges::end(tagView);

        for (std::size_t i = 0UZ; i < inSpan.size(); ++i) {
            // an unconsumed tag is presented again at a negative relative index; no packet is held across calls, so
            // this skips only what a packet already converted in this call took
            while (tagIt != tagEnd && (*tagIt).first < static_cast<std::ptrdiff_t>(i)) {
                ++tagIt;
            }
            property_map arrivingTag;
            while (tagIt != tagEnd && (*tagIt).first == static_cast<std::ptrdiff_t>(i)) {
                for (const auto& [key, value] : (*tagIt).second.get()) {
                    arrivingTag.insert_or_assign(key, value);
                }
                ++tagIt;
            }

            const Packet<T>& packet = inSpan[i];
            if (const char* reason = admit(packet); reason != nullptr) {
                if (rejectConnected && onReject >= rejectSpan.size()) {
                    break; // no room on the port this packet belongs on; it stays in the buffer
                }
                if (rejectConnected) {
                    rejectSpan[onReject] = packet; // republished untouched: what is wrong with it may be the field one would edit
                    arrivingTag.insert_or_assign(property_map::key_type("discard_reason"), pmt::Value(std::string(reason)));
                    rejectSpan.publishTag(arrivingTag, onReject);
                }
                ++onReject;
                ++nRejectedPackets;
                ++consumed;
                continue;
            }

            if (outConnected && onOut >= outSpan.size()) {
                break;
            }
            DataSet<T> record = convert(packet, arrivingTag);
            if (outConnected && onOut < outSpan.size()) {
                outSpan[onOut] = std::move(record);
            }
            ++onOut;
            ++consumed;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(outConnected ? onOut : 0UZ);
        rejectSpan.publish(rejectConnected ? onReject : 0UZ);
        if (consumed == 0UZ) {
            return inSpan.size() == 0UZ ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    /// @brief The three facts the block indexes on. Returns the `discard_reason` of the first that fails, else nullptr.
    [[nodiscard]] const char* admit(const Packet<T>& packet) const noexcept {
        if (packet.meta_information.size() != 1UZ) { // the declaration sizes this vector nowhere, so [0] would be a guess
            return "not_one_metadata_map";
        }
        if (packet.signal_values.empty() && !allow_empty_payload.value) {
            return "empty_payload";
        }
        if (packet.signal_values.size() > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())) {
            return "payload_too_long_for_extent"; // extents is a std::int32_t and the wire's item count is wider
        }
        return nullptr;
    }

    /// @brief The value of `key` as a string, or nothing when it is absent or holds something else.
    [[nodiscard]] static std::optional<std::string> readString(const property_map& map, std::string_view key) {
        const pmt::Value* value = detail::find(map, key);
        if (value == nullptr) {
            return std::nullopt;
        }
        const std::pmr::string* text = value->get_if<std::pmr::string>();
        if (text == nullptr) {
            return std::nullopt;
        }
        return std::string(text->begin(), text->end());
    }

    /// @brief `value` as a `T`, or nothing when the record's own type cannot hold it.
    ///
    /// The vocabulary types `signal_min` and `signal_max` `float` while `Range<T>` holds `T`, so the lift narrows for
    /// every integral `T`. Casting a floating-point value outside the target's range is undefined, and the value came
    /// off a wire, so the comparison is made in `double` — which represents every bound of every registered integral
    /// `T` exactly — against the value the cast will actually produce.
    [[nodiscard]] static std::optional<T> narrowed(float value) noexcept {
        if constexpr (std::floating_point<T>) {
            return value;
        } else {
            if (!std::isfinite(value)) {
                return std::nullopt;
            }
            const double truncated = std::trunc(static_cast<double>(value));
            if (truncated < static_cast<double>(std::numeric_limits<T>::lowest()) || truncated > static_cast<double>(std::numeric_limits<T>::max())) {
                return std::nullopt;
            }
            return static_cast<T>(value);
        }
    }

    /// @brief Copy `source` into `map`, dropping and counting a vocabulary key whose type disagrees with the table.
    void copyInto(property_map& map, const property_map& source) {
        for (const auto& [key, value] : source) {
            const std::string_view name = detail::shortKey(std::string_view(key));
            if (!detail::packet::holdsVocabularyType(detail::packet::vocabularyType(name), value)) {
                ++nMetaKeysDropped; // a typed read on this value would return nothing and report nothing
                continue;
            }
            map.insert_or_assign(key, value);
        }
    }

    /// @brief Lift the two range ends into `signal_ranges`, or leave it empty and count the pair that did not fit.
    void liftRange(DataSet<T>& record, const property_map& map) {
        if constexpr (std::is_arithmetic_v<T>) { // a complex signal_range holds two samples ordered by magnitude, not two limits
            const pmt::Value* low  = detail::find(map, tag::SIGNAL_MIN.shortKey());
            const pmt::Value* high = detail::find(map, tag::SIGNAL_MAX.shortKey());
            if (low == nullptr || high == nullptr) {
                return; // a range with one end is not a range; the key that is present still crossed in the map
            }
            const float* lowValue  = low->get_if<float>();
            const float* highValue = high->get_if<float>();
            if (lowValue == nullptr || highValue == nullptr) {
                return; // the type rule already dropped and counted whichever end was mistyped
            }
            const std::optional<T> lowLimit  = narrowed(*lowValue);
            const std::optional<T> highLimit = narrowed(*highValue);
            if (!lowLimit.has_value() || !highLimit.has_value()) {
                ++nMetaKeysDropped; // the fact did not reach the field it names, though both keys still crossed
                return;
            }
            record.signal_ranges.push_back(Range<T>{*lowLimit, *highLimit});
        } else {
            std::ignore = record;
            std::ignore = map;
        }
    }

    /// @brief Build the record: the payload, its extent, the two carried fields, one metadata map and the lifted keys.
    [[nodiscard]] DataSet<T> convert(const Packet<T>& packet, const property_map& arrivingTag) {
        DataSet<T> record;
        record.default_value = packet.default_value; // declared by both carriers and reflected by neither
        record.timestamp     = packet.timestamp;     // both are std::int64_t, so the crossing is lossless and needs no guard
        record.signal_values.assign(packet.signal_values.begin(), packet.signal_values.end());
        // The admission check bounds the cast. `layout` is left at the default-constructed LayoutRight: with one extent there is one
        // ordering, and neither StreamToDataSet nor PacketFramer writes the field either.
        record.extents.push_back(static_cast<std::int32_t>(packet.signal_values.size()));

        record.meta_information.resize(1UZ);
        property_map& map = record.meta_information[0UZ];
        copyInto(map, packet.meta_information[0UZ]); // the admission check makes the index safe
        copyInto(map, arrivingTag);                  // the tag at this packet's index annotates this packet

        record.timing_events.resize(1UZ); // one list per signal, and nothing ever puts an entry in it

        if (const std::optional<std::string> name = readString(map, tag::SIGNAL_NAME.shortKey()); name.has_value()) {
            record.signal_names.push_back(*name);
        } else {
            record.signal_names.push_back(signal_label.value); // a record with no signal name is unusable downstream
            ++nSignalNamesSynthesized;
        }
        if (const std::optional<std::string> quantity = readString(map, tag::SIGNAL_QUANTITY.shortKey()); quantity.has_value()) {
            record.signal_quantities.push_back(*quantity);
        }
        if (const std::optional<std::string> unit = readString(map, tag::SIGNAL_UNIT.shortKey()); unit.has_value()) {
            record.signal_units.push_back(*unit);
        }
        liftRange(record, map);
        return record;
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_PACKETTODATASET_HPP
