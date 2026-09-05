#ifndef GNURADIO_DATASETTOPACKET_HPP
#define GNURADIO_DATASETTOPACKET_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <optional>
#include <print>
#include <ranges>
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
#include <gnuradio-4.0/meta/utils.hpp>

#include <gnuradio-4.0/basic/RecordMetadata.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::DataSetToPacket, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
requires(std::is_arithmetic_v<T> || gr::meta::complex_like<T>)
struct DataSetToPacket : Block<DataSetToPacket<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Converts one signal of each incoming DataSet<T> into a gr::Packet<T>, the terminal form for a transport sink.

This is the boundary at which a payload's interior positions stop mattering. `DataSet<T>` declares fourteen fields and
`Packet<T>` declares four, so the conversion is lossy by construction and the line it draws is exact: **positional
facts stop, provenance facts cross.** `timing_events` and the axis are indexed by a sample position and do not cross;
`sample_rate`, `sample_start`, `frequency` and the UTC `timestamp` describe the sampling without indexing anything and
do. A graph that still needs the interior annotated should not be converting.

The packet carries exactly one metadata map. It holds the selected signal's `meta_information` copied key for key, the
keys derived from the record's own signal annotations — `signal_name`, `signal_quantity`, `signal_unit`, `signal_min`,
`signal_max` — a `sample_rate` when the record did not state one, and `dropped_events` when annotations were dropped.
A copied key that names a record-metadata vocabulary key but holds the wrong type is dropped and counted: an exactly
typed read on the far side of a transport returns nothing for it, so it would arrive as neither a value nor an error.

`protocol_label`, `source_label` and `frequency_hz` stamp the three vocabulary keys no producer inside a graph can
know — the payload's format, the producing identity and the receiver's tuning — and only where the record left them
unstated. Nothing else here states a fact the record did not.

A record the block cannot index leaves by the `reject` port, unchanged, with `discard_reason` on a tag beside it. The
six admission clauses are the facts the block indexes on: at least one signal, `signal_index` in range, at most one
dimension, `signal_values` divisible by the signal count, an `extents` that agrees when present, and a payload of at
least one item. `extents` is optional because `Packet<T>` has no field to carry it into, and a zero-length payload is
refused because a packet's length is `signal_values.size()`, so an empty packet is indistinguishable on a wire from
one that was lost.

One instance emits one signal. A four-signal record is read by four instances at `signal_index` 0 to 3 on the same
record port; they see the same records and admit them on the same predicate, so packet k of every stream comes from
record k.
)"">;

    PortIn<DataSet<T>>                   in;
    PortOut<Packet<T>, Async>            out;
    PortOut<DataSet<T>, Async, Optional> reject;

    Annotated<gr::Size_t, "signal_index", Visible, Doc<"which signal of a multi-signal record becomes the payload; out of range rejects the record">>     signal_index{0U};
    Annotated<std::string, "protocol_label", Doc<"written under the vocabulary key protocol when non-empty and the record does not already state one">>   protocol_label{""};
    Annotated<std::string, "source_label", Doc<"written under the vocabulary key source_id when non-empty and the record does not already state one">>    source_label{""};
    Annotated<double, "frequency_hz", Doc<"written in hertz under the vocabulary key frequency when non-zero and the record does not already state one">> frequency_hz{0.0};
    Annotated<float, "axis_rate_tolerance", Doc<"relative tolerance of the time axis uniformity test; must be finite and greater than zero">>             axis_rate_tolerance{1e-6f};

    GR_MAKE_REFLECTABLE(DataSetToPacket, in, out, reject, signal_index, protocol_label, source_label, frequency_hz, axis_rate_tolerance);

    // Counted, stated drops. Plain members, read by the owning thread and reported once at stop(). The far side of
    // this boundary is another process, so the one drop a peer cannot infer travels in band as `dropped_events` too.
    std::uint64_t nRejectedRecords     = 0ULL; ///< records the admission predicate turned away
    std::uint64_t nSignalsNotEmitted   = 0ULL; ///< signals of admitted records that no packet carries
    std::uint64_t nDroppedTimingEvents = 0ULL; ///< record annotations the target carrier has no field for
    std::uint64_t nMetaKeysDropped     = 0ULL; ///< copied vocabulary keys whose type disagreed with the declaration
    std::uint64_t nMetaKeysOverridden  = 0ULL; ///< copied keys a derived key displaced with a different value
    std::uint64_t nStampsDeclined      = 0ULL; ///< labels suppressed because the record already stated that key

    float                _tolerance = 1e-6f; ///< the validated axis_rate_tolerance in force
    std::optional<float> _tagRate{};         ///< the most recent sample_rate seen on the input port, which is rate route 2

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void reset() { _tagRate.reset(); }

    void rebuild() {
        if (!std::isfinite(axis_rate_tolerance.value) || axis_rate_tolerance.value <= 0.f) {
            const float rejected      = axis_rate_tolerance.value;
            axis_rate_tolerance.value = _tolerance; // the block keeps working at the tolerance it already had
            throw gr::exception(std::format("axis_rate_tolerance is {}; it must be finite and greater than zero", rejected));
        }
        _tolerance = axis_rate_tolerance.value;
    }

    void stop() {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("rejected records", nRejectedRecords);
        append("signals not emitted", nSignalsNotEmitted);
        append("dropped timing events", nDroppedTimingEvents);
        append("metadata keys dropped", nMetaKeysDropped);
        append("metadata keys overridden", nMetaKeysOverridden);
        append("stamps declined", nStampsDeclined);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::basic::DataSetToPacket '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& rejectSpan) {
        const bool outConnected    = outSpan.isConnected; // read once, so the room test and the store cannot disagree
        const bool rejectConnected = rejectSpan.isConnected;
        const auto signalIndex     = static_cast<std::size_t>(signal_index.value);

        std::size_t consumed = 0UZ;
        std::size_t onOut    = 0UZ;
        std::size_t onReject = 0UZ;

        auto       tagView = inSpan.tags(); // ascending by index: a tag at relative index r belongs to record r
        auto       tagIt   = std::ranges::begin(tagView);
        const auto tagEnd  = std::ranges::end(tagView);

        for (std::size_t i = 0UZ; i < inSpan.size(); ++i) {
            // an unconsumed tag is presented again at a negative relative index; no record is held across calls, so
            // this skips only what a record already converted in this call took
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
            if (const pmt::Value* rate = detail::find(arrivingTag, tag::SAMPLE_RATE.shortKey()); rate != nullptr) {
                if (const float* value = rate->get_if<float>(); value != nullptr) {
                    _tagRate = *value; // rate route 2, from the port rather than from any one record
                }
            }

            const DataSet<T>& ds = inSpan[i];
            if (const char* reason = admit(ds, signalIndex); reason != nullptr) {
                if (rejectConnected && onReject >= rejectSpan.size()) {
                    break; // no room on the port this record belongs on; it stays in the buffer
                }
                if (rejectConnected) {
                    rejectSpan[onReject] = ds; // republished untouched: what is wrong with it may be the field one would edit
                    arrivingTag.insert_or_assign(property_map::key_type("discard_reason"), pmt::Value(std::string(reason)));
                    rejectSpan.publishTag(arrivingTag, onReject);
                }
                ++onReject;
                ++nRejectedRecords;
                ++consumed;
                continue;
            }

            if (outConnected && onOut >= outSpan.size()) {
                break;
            }
            Packet<T> packet = convert(ds, signalIndex, arrivingTag);
            if (outConnected && onOut < outSpan.size()) {
                outSpan[onOut] = std::move(packet);
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
    /// @brief The six facts the block indexes on. Returns the `discard_reason` of the first that fails, else nullptr.
    [[nodiscard]] static const char* admit(const DataSet<T>& ds, std::size_t signalIndex) noexcept {
        if (ds.signal_names.empty()) {
            return "no_signals";
        }
        if (signalIndex >= ds.signal_names.size()) {
            return "signal_index_out_of_range";
        }
        if (ds.extents.size() > 1UZ) { // flattening would destroy a shape the target carrier has no field for
            return "not_one_dimensional";
        }
        if (ds.signal_values.size() % ds.signal_names.size() != 0UZ) {
            return "ragged_signals";
        }
        const std::size_t valuesPerSignal = ds.signal_values.size() / ds.signal_names.size();
        if (!ds.extents.empty() && (ds.extents[0UZ] <= 0 || static_cast<std::size_t>(ds.extents[0UZ]) != valuesPerSignal)) {
            return "inconsistent_extent"; // absent is admissible; present and disagreeing is not
        }
        if (valuesPerSignal < 1UZ) { // a packet's length is signal_values.size(), so an empty one is a lost one
            return "empty_payload";
        }
        return nullptr;
    }

    /// @brief Write `value` under `key` unless the record already stated that key, which is counted.
    void stamp(property_map& map, const DataSet<T>& ds, std::size_t signalIndex, std::string_view key, pmt::Value value) {
        if (signalIndex < ds.meta_information.size() && detail::find(ds.meta_information[signalIndex], key) != nullptr) {
            ++nStampsDeclined; // a constant a human typed is the less informed source, so the record's word stands
            return;
        }
        map.insert_or_assign(property_map::key_type(key), std::move(value));
    }

    /// @brief Build the packet: the payload, the two carried fields, and the one metadata map the packet holds.
    [[nodiscard]] Packet<T> convert(const DataSet<T>& ds, std::size_t signalIndex, const property_map& arrivingTag) {
        // The extent check makes the division exact and the signal-count check bounds the offset, so the payload is reached without a checked accessor
        const std::size_t        valuesPerSignal = ds.signal_values.size() / ds.signal_names.size();
        const std::span<const T> payload         = std::span<const T>(ds.signal_values).subspan(signalIndex * valuesPerSignal, valuesPerSignal);

        Packet<T> packet;
        packet.default_value = ds.default_value; // declared by both carriers and reflected by neither
        packet.timestamp     = ds.timestamp;     // both are std::int64_t, so the crossing is lossless and needs no guard
        packet.signal_values.assign(payload.begin(), payload.end());
        packet.meta_information.resize(1UZ); // a packet this block produces has exactly one map, and it is [0]
        property_map& map = packet.meta_information[0UZ];

        nSignalsNotEmitted += static_cast<std::uint64_t>(ds.signal_names.size() - 1UZ);

        bool statedRate = false;
        if (signalIndex < ds.meta_information.size()) { // the record's own statement about itself, copied key for key
            for (const auto& [key, value] : ds.meta_information[signalIndex]) {
                const std::string_view name = detail::shortKey(std::string_view(key));
                if (!detail::packet::holdsVocabularyType(detail::packet::vocabularyType(name), value)) {
                    ++nMetaKeysDropped; // a typed read on the far side would return nothing and report nothing
                    continue;
                }
                statedRate = statedRate || name == tag::SAMPLE_RATE.shortKey(); // rate route 1, which is the absence of a derivation
                map.insert_or_assign(key, value);
            }
        }
        for (const auto& [key, value] : arrivingTag) { // the tag at this record's index annotates this record
            const std::string_view name = detail::shortKey(std::string_view(key));
            if (!detail::packet::holdsVocabularyType(detail::packet::vocabularyType(name), value)) {
                ++nMetaKeysDropped;
                continue;
            }
            map.insert_or_assign(key, value);
        }

        detail::putDerived(map, tag::SIGNAL_NAME.shortKey(), pmt::Value(ds.signal_names[signalIndex]), nMetaKeysOverridden);
        if (signalIndex < ds.signal_quantities.size()) {
            detail::putDerived(map, tag::SIGNAL_QUANTITY.shortKey(), pmt::Value(ds.signal_quantities[signalIndex]), nMetaKeysOverridden);
        }
        if (signalIndex < ds.signal_units.size()) {
            detail::putDerived(map, tag::SIGNAL_UNIT.shortKey(), pmt::Value(ds.signal_units[signalIndex]), nMetaKeysOverridden);
        }
        if constexpr (std::is_arithmetic_v<T>) { // a complex signal_range holds two samples ordered by magnitude, not two limits
            if (signalIndex < ds.signal_ranges.size()) {
                detail::putDerived(map, tag::SIGNAL_MIN.shortKey(), pmt::Value(static_cast<float>(ds.signal_ranges[signalIndex].min)), nMetaKeysOverridden);
                detail::putDerived(map, tag::SIGNAL_MAX.shortKey(), pmt::Value(static_cast<float>(ds.signal_ranges[signalIndex].max)), nMetaKeysOverridden);
            }
        }
        if (!statedRate) { // routes 2 and 3, in that order; an absent rate is a stated absence and is not counted
            const std::optional<float> derived = _tagRate.has_value() ? _tagRate : detail::axisRate(ds, valuesPerSignal, static_cast<double>(_tolerance));
            if (derived.has_value()) {
                detail::putDerived(map, tag::SAMPLE_RATE.shortKey(), pmt::Value(*derived), nMetaKeysOverridden);
            }
        }

        std::uint64_t events = 0ULL;
        for (const auto& list : ds.timing_events) { // one size read per signal list, never a walk of the entries
            events += static_cast<std::uint64_t>(list.size());
        }
        nDroppedTimingEvents += events;
        if (events > 0ULL) { // a counter in this process tells nobody on the far side of a transport
            detail::putDerived(map, "dropped_events", pmt::Value(static_cast<gr::Size_t>(std::min(events, static_cast<std::uint64_t>(std::numeric_limits<gr::Size_t>::max())))), nMetaKeysOverridden);
        }

        // An unset setting states nothing at all, so it neither writes a key nor declines one. Zero is the unset
        // frequency: a receiver at 0 Hz is not a tuning any transport carries, and the alternative — an optional —
        // is not a reflectable setting type.
        if (!protocol_label.value.empty()) {
            stamp(map, ds, signalIndex, "protocol", pmt::Value(protocol_label.value));
        }
        if (!source_label.value.empty()) {
            stamp(map, ds, signalIndex, "source_id", pmt::Value(source_label.value));
        }
        if (frequency_hz.value != 0.0) {
            stamp(map, ds, signalIndex, tag::FREQUENCY.shortKey(), pmt::Value(frequency_hz.value));
        }
        return packet;
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_DATASETTOPACKET_HPP
