#ifndef GNURADIO_DATASETTOSTREAM_HPP
#define GNURADIO_DATASETTOSTREAM_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <numeric>
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

namespace detail {

/// @brief The declared value type of a reserved tag key, as the group of keys sharing it.
enum class ReservedTagType : std::uint8_t { NotReserved, Float, Double, String, Size, UInt64, Bool, Map };

/// @brief The type `key` is declared to hold in `gr::tag`, or `NotReserved` when the framework reserves no such key.
[[nodiscard]] inline ReservedTagType reservedTagType(std::string_view key) noexcept {
    const std::string_view name = shortKey(key);
    using enum ReservedTagType;
    if (name == tag::SAMPLE_RATE.shortKey() || name == tag::SIGNAL_MIN.shortKey() || name == tag::SIGNAL_MAX.shortKey() || name == tag::TRIGGER_OFFSET.shortKey()) {
        return Float;
    }
    if (name == tag::FREQUENCY.shortKey()) {
        return Double;
    }
    if (name == tag::SIGNAL_NAME.shortKey() || name == tag::SIGNAL_QUANTITY.shortKey() || name == tag::SIGNAL_UNIT.shortKey() || name == tag::TRIGGER_NAME.shortKey() || name == tag::CONTEXT.shortKey()) {
        return String;
    }
    if (name == tag::NUM_CHANNELS.shortKey() || name == tag::N_DROPPED_SAMPLES.shortKey()) {
        return Size;
    }
    if (name == tag::TRIGGER_TIME.shortKey() || name == tag::CONTEXT_TIME.shortKey()) {
        return UInt64;
    }
    if (name == tag::RX_OVERFLOW.shortKey() || name == tag::RESET_DEFAULTS.shortKey() || name == tag::STORE_DEFAULTS.shortKey() || name == tag::END_OF_STREAM.shortKey()) {
        return Bool;
    }
    if (name == tag::TRIGGER_META_INFO.shortKey()) {
        return Map;
    }
    return NotReserved;
}

/// @brief Whether `value` holds the type `type` names. A non-reserved key imposes nothing and always passes.
[[nodiscard]] inline bool holdsReservedType(ReservedTagType type, const pmt::Value& value) noexcept {
    using enum ReservedTagType;
    switch (type) {
    case Float: return value.get_if<float>() != nullptr;
    case Double: return value.get_if<double>() != nullptr;
    case String: return value.get_if<std::pmr::string>() != nullptr;
    case Size: return value.get_if<gr::Size_t>() != nullptr;
    case UInt64: return value.get_if<std::uint64_t>() != nullptr;
    case Bool: return value.get_if<bool>() != nullptr;
    case Map: return value.get_if<property_map>() != nullptr;
    case NotReserved: break;
    }
    return true;
}

/// @brief An unsigned integer of any of the four widths, widened to 64 bits, or nothing when the value is not one.
[[nodiscard]] inline std::optional<std::uint64_t> readUnsigned(const pmt::Value& value) noexcept {
    if (const std::uint64_t* wide = value.get_if<std::uint64_t>(); wide != nullptr) {
        return *wide;
    }
    if (const std::uint32_t* word = value.get_if<std::uint32_t>(); word != nullptr) {
        return static_cast<std::uint64_t>(*word);
    }
    if (const std::uint16_t* half = value.get_if<std::uint16_t>(); half != nullptr) {
        return static_cast<std::uint64_t>(*half);
    }
    if (const std::uint8_t* byte = value.get_if<std::uint8_t>(); byte != nullptr) {
        return static_cast<std::uint64_t>(*byte);
    }
    return std::nullopt;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::basic::DataSetToStream, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
requires(std::is_arithmetic_v<T> || gr::meta::complex_like<T>)
struct DataSetToStream : Block<DataSetToStream<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Emits one signal of each incoming DataSet<T> as a plain stream, with the record's metadata as stream tags.

The inverse of `StreamToDataSet`: a bounded record carrying its own rate, signal metadata, UTC time and interior
annotations becomes an unbounded stream carrying tags. Every admitted record marks its first output sample with one
tag naming the boundary, and the record's facts ride that tag under the framework's reserved keys wherever one
exists — `sample_rate`, `signal_name`, `signal_quantity`, `signal_unit`, `signal_min`, `signal_max`, `trigger_name`,
`trigger_time` — announced at the first record and thereafter only when they change. `meta_information` for the
selected signal is copied key for key, and `timing_events` become one tag each at the translated offset.

A record the block cannot index leaves by the `reject` port, unchanged, with `discard_reason` on a tag beside it. The
five admission clauses are exactly the facts the block indexes on: at least one signal, `signal_index` in range, one
dimension, a positive extent, and `signal_values` sized to `signals * extent`. Everything else a record may carry is
optional and its absence is reported rather than refused, so a packet with no axis and no units is admitted whole.

One instance emits one signal. A four-signal record is read by four instances at `signal_index` 0 to 3 on the same
output port; they see the same records and emit the same counts, so their streams are sample-aligned by construction.

Records concatenate. Two records cut from different parts of a stream produce one contiguous output with nothing
inserted between them: the gap is reported through `discontinuity` and `n_dropped_samples`, never synthesized.
)"">;

    PortIn<DataSet<T>, Async>            in;
    PortOut<T, Async>                    out;
    PortOut<DataSet<T>, Async, Optional> reject;

    Annotated<gr::Size_t, "signal_index", Visible, Doc<"which signal of a multi-signal record becomes the output stream; out of range rejects the record">>                      signal_index{0U};
    Annotated<std::string, "boundary_label", Doc<"written under trigger_name at each record's first sample, unless the record names its own trigger; empty suppresses the key">> boundary_label{"dataset"};
    Annotated<float, "axis_rate_tolerance", Doc<"relative tolerance of the time axis uniformity test; must be finite and greater than zero">>                                    axis_rate_tolerance{1e-6f};

    GR_MAKE_REFLECTABLE(DataSetToStream, in, out, reject, signal_index, boundary_label, axis_rate_tolerance);

    // Counted, stated drops. Plain members, read by the owning thread and reported once at stop().
    std::uint64_t nRejectedRecords     = 0ULL; ///< records the admission predicate turned away
    std::uint64_t nSignalsNotEmitted   = 0ULL; ///< signals of admitted records that no output carries
    std::uint64_t nDroppedTimingEvents = 0ULL; ///< events whose index lies outside the record
    std::uint64_t nRateLessRecords     = 0ULL; ///< admitted records for which no rate route yielded a rate
    std::uint64_t nMetaKeysDropped     = 0ULL; ///< copied reserved keys whose type disagreed with the declaration
    std::uint64_t nMetaKeysOverridden  = 0ULL; ///< copied keys a derived key displaced with a different value

    /// @brief What the last admitted record stated, which the next one is compared against for `discontinuity`.
    struct RecordFacts {
        bool                                   present = false;
        std::optional<float>                   rate{};
        std::string                            name{};
        std::optional<std::string>             quantity{};
        std::optional<std::string>             unit{};
        std::optional<std::pair<float, float>> range{};
        std::optional<std::uint64_t>           sampleStart{};
        std::size_t                            length = 0UZ;
    };

    /// @brief The last value the block put on the stream for each announced-on-change key.
    struct AnnouncedKeys {
        std::optional<float>       rate{};
        std::optional<std::string> name{};
        std::optional<std::string> quantity{};
        std::optional<std::string> unit{};
        std::optional<float>       min{};
        std::optional<float>       max{};
    };

    float                      _tolerance    = 1e-6f; ///< the validated axis_rate_tolerance in force
    std::size_t                _sampleCursor = 0UZ;   ///< samples of the record in progress already emitted
    std::size_t                _eventCursor  = 0UZ;   ///< timing events of the record in progress already visited
    bool                       _eventsSorted = true;  ///< whether the record's event indices arrived in ascending order
    std::vector<std::uint32_t> _eventOrder{};         ///< ascending permutation of the events, only when they are not sorted
    std::optional<float>       _tagRate{};            ///< the most recent sample_rate seen on the input port
    RecordFacts                _previous{};
    AnnouncedKeys              _announced{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void reset() {
        _sampleCursor = 0UZ;
        _eventCursor  = 0UZ;
        _eventsSorted = true;
        _eventOrder.clear();
        _tagRate.reset();
        _previous  = RecordFacts{};
        _announced = AnnouncedKeys{};
    }

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
        append("records without a rate", nRateLessRecords);
        append("metadata keys dropped", nMetaKeysDropped);
        append("metadata keys overridden", nMetaKeysOverridden);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::basic::DataSetToStream '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& rejectSpan) {
        if (inSpan.size() == 0UZ) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            rejectSpan.publish(0UZ);
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        const DataSet<T>& ds = inSpan[0UZ];
        // A record held across work() calls sees its own tag presented again at relative index 0; its keys were
        // merged when the record started, so acting on it again would duplicate them once per call.
        const bool resuming = _sampleCursor > 0UZ;

        property_map arrivingTag;
        if (!resuming) {
            for (const auto& [relIndex, tagMap] : inSpan.tags()) {
                if (relIndex != 0) { // negative: already merged; positive: a later record's
                    continue;
                }
                for (const auto& [key, value] : tagMap.get()) {
                    arrivingTag.insert_or_assign(key, value);
                }
            }
            if (const pmt::Value* rate = detail::find(arrivingTag, tag::SAMPLE_RATE.shortKey()); rate != nullptr) {
                if (const float* value = rate->get_if<float>(); value != nullptr) {
                    _tagRate = *value; // route 2, from the port rather than from this record
                }
            }
        }

        const auto signalIndex = static_cast<std::size_t>(signal_index.value);
        if (!resuming) {
            if (const char* reason = admit(ds, signalIndex); reason != nullptr) {
                const bool wanted = rejectSpan.isConnected;
                if (wanted && rejectSpan.size() == 0UZ) { // the record stays in the buffer until the port has room
                    std::ignore = inSpan.consume(0UZ);
                    outSpan.publish(0UZ);
                    rejectSpan.publish(0UZ);
                    return work::Status::INSUFFICIENT_OUTPUT_ITEMS;
                }
                if (wanted) {
                    rejectSpan[0UZ] = ds; // republished untouched: what is wrong with it may be the field one would edit
                    arrivingTag.insert_or_assign(property_map::key_type("discard_reason"), pmt::Value(std::string(reason)));
                    rejectSpan.publishTag(arrivingTag, 0UZ);
                }
                ++nRejectedRecords;
                std::ignore = inSpan.consume(1UZ);
                outSpan.publish(0UZ);
                rejectSpan.publish(wanted ? 1UZ : 0UZ);
                return work::Status::OK;
            }
        }

        const auto nSamples = static_cast<std::size_t>(ds.extents[0UZ]);
        if (outSpan.size() == 0UZ) { // nothing above this point has changed any state, so the call simply repeats
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            rejectSpan.publish(0UZ);
            return work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }

        if (!resuming) {
            outSpan.publishTag(beginRecord(ds, signalIndex, nSamples, arrivingTag), 0UZ);
        }

        // The extent check makes the division exact and the signal-count check bounds the offset, so the samples are reached without a checked accessor.
        const std::size_t        valuesPerSignal = ds.signal_values.size() / ds.signal_names.size();
        const std::span<const T> samples         = std::span<const T>(ds.signal_values).subspan(signalIndex * valuesPerSignal, valuesPerSignal);

        const std::size_t nToEmit = std::min(nSamples - _sampleCursor, outSpan.size());
        std::ranges::copy_n(std::next(samples.begin(), static_cast<std::ptrdiff_t>(_sampleCursor)), static_cast<std::ptrdiff_t>(nToEmit), outSpan.begin());
        publishTimingEvents(outSpan, ds, signalIndex, nSamples, nToEmit);

        _sampleCursor += nToEmit;
        outSpan.publish(nToEmit);
        rejectSpan.publish(0UZ);

        if (_sampleCursor == nSamples) { // the input item is consumed on the call that finishes it, and nowhere else
            std::ignore   = inSpan.consume(1UZ);
            _sampleCursor = 0UZ;
            _eventCursor  = 0UZ;
        } else {
            std::ignore = inSpan.consume(0UZ);
        }
        return work::Status::OK;
    }

private:
    /// @brief The five facts the block indexes on. Returns the `discard_reason` of the first that fails, else nullptr.
    [[nodiscard]] static const char* admit(const DataSet<T>& ds, std::size_t signalIndex) noexcept {
        if (ds.signal_names.empty()) {
            return "no_signals";
        }
        if (signalIndex >= ds.signal_names.size()) {
            return "signal_index_out_of_range";
        }
        if (ds.extents.size() != 1UZ) {
            return "not_one_dimensional";
        }
        if (ds.extents[0UZ] <= 0) {
            return "empty_or_negative_extent";
        }
        if (ds.signal_values.size() != ds.signal_names.size() * static_cast<std::size_t>(ds.extents[0UZ])) {
            return "inconsistent_extent";
        }
        return nullptr;
    }

    /// @brief Evaluate the record once: plan its events, resolve its rate, and build the tag its first sample carries.
    [[nodiscard]] property_map beginRecord(const DataSet<T>& ds, std::size_t signalIndex, std::size_t nSamples, const property_map& arrivingTag) {
        nSignalsNotEmitted += static_cast<std::uint64_t>(ds.signal_names.size() - 1UZ);
        planTimingEvents(ds, signalIndex, nSamples);

        property_map                 map;
        std::optional<float>         metaRate{};
        std::optional<std::uint64_t> sampleStart{};
        std::optional<std::string>   recordTrigger{};

        if (signalIndex < ds.meta_information.size()) { // (i) the record's own statement about itself, copied verbatim
            for (const auto& [key, value] : ds.meta_information[signalIndex]) {
                const std::string_view        name = detail::shortKey(std::string_view(key));
                const detail::ReservedTagType type = detail::reservedTagType(name);
                if (!detail::holdsReservedType(type, value)) { // (iii) a typed read would return nothing and report nothing
                    ++nMetaKeysDropped;
                    continue;
                }
                if (name == tag::SAMPLE_RATE.shortKey()) {
                    metaRate = *value.template get_if<float>(); // route 1: the producer's own words, exact and typed
                } else if (name == tag::TRIGGER_NAME.shortKey()) {
                    recordTrigger = std::string(std::string_view(*value.template get_if<std::pmr::string>()));
                } else if (name == "sample_start") {
                    sampleStart = detail::readUnsigned(value);
                }
                map.insert_or_assign(key, value);
            }
        }
        for (const auto& [key, value] : arrivingTag) { // the input tag belongs at the record's first sample
            map.insert_or_assign(key, value);
        }

        RecordFacts facts;
        facts.present     = true;
        facts.length      = nSamples;
        facts.sampleStart = sampleStart;
        facts.rate        = metaRate.has_value() ? metaRate : (_tagRate.has_value() ? _tagRate : detail::axisRate(ds, nSamples, static_cast<double>(_tolerance)));
        facts.name        = ds.signal_names[signalIndex];
        if (signalIndex < ds.signal_quantities.size()) {
            facts.quantity = ds.signal_quantities[signalIndex];
        }
        if (signalIndex < ds.signal_units.size()) {
            facts.unit = ds.signal_units[signalIndex];
        }
        if constexpr (std::is_arithmetic_v<T>) { // a complex signal_range holds two samples ordered by magnitude, not two limits
            if (signalIndex < ds.signal_ranges.size()) {
                facts.range = std::pair<float, float>{static_cast<float>(ds.signal_ranges[signalIndex].min), static_cast<float>(ds.signal_ranges[signalIndex].max)};
            }
        }
        if (!facts.rate.has_value()) {
            ++nRateLessRecords;
        }

        detail::putDerived(map, "dataset_length", pmt::Value(static_cast<gr::Size_t>(nSamples)), nMetaKeysOverridden);
        const std::string triggerName = recordTrigger.has_value() && !recordTrigger->empty() ? *recordTrigger : boundary_label.value;
        if (!triggerName.empty()) {
            detail::putDerived(map, tag::TRIGGER_NAME.shortKey(), pmt::Value(triggerName), nMetaKeysOverridden);
        }
        if (ds.timestamp > 0) { // a negative timestamp reinterprets as a date in the year 2262
            detail::putDerived(map, tag::TRIGGER_TIME.shortKey(), pmt::Value(static_cast<std::uint64_t>(ds.timestamp)), nMetaKeysOverridden);
        }
        announceChanges(map, facts);
        markDiscontinuity(map, facts);

        _previous = std::move(facts);
        return map;
    }

    /// @brief Put each announced-on-change key on the tag when this record's value differs from the last one emitted.
    void announceChanges(property_map& map, const RecordFacts& facts) {
        if (facts.rate.has_value() && _announced.rate != facts.rate) {
            detail::putDerived(map, tag::SAMPLE_RATE.shortKey(), pmt::Value(*facts.rate), nMetaKeysOverridden);
            _announced.rate = facts.rate;
        }
        if (_announced.name != facts.name) {
            detail::putDerived(map, tag::SIGNAL_NAME.shortKey(), pmt::Value(facts.name), nMetaKeysOverridden);
            _announced.name = facts.name;
        }
        if (facts.quantity.has_value() && _announced.quantity != facts.quantity) {
            detail::putDerived(map, tag::SIGNAL_QUANTITY.shortKey(), pmt::Value(*facts.quantity), nMetaKeysOverridden);
            _announced.quantity = facts.quantity;
        }
        if (facts.unit.has_value() && _announced.unit != facts.unit) {
            detail::putDerived(map, tag::SIGNAL_UNIT.shortKey(), pmt::Value(*facts.unit), nMetaKeysOverridden);
            _announced.unit = facts.unit;
        }
        if (facts.range.has_value() && (_announced.min != facts.range->first || _announced.max != facts.range->second)) {
            detail::putDerived(map, tag::SIGNAL_MIN.shortKey(), pmt::Value(facts.range->first), nMetaKeysOverridden);
            detail::putDerived(map, tag::SIGNAL_MAX.shortKey(), pmt::Value(facts.range->second), nMetaKeysOverridden);
            _announced.min = facts.range->first;
            _announced.max = facts.range->second;
        }
    }

    /// @brief Name the causes by which this record does not continue the last one, and the gap where one is countable.
    void markDiscontinuity(property_map& map, const RecordFacts& facts) {
        if (!_previous.present) { // a stream's own beginning is not discontinuous with anything
            return;
        }
        std::string causes;
        const auto  fire = [&causes](std::string_view cause) {
            if (!causes.empty()) {
                causes.push_back(',');
            }
            causes.append(cause);
        };
        if (facts.rate != _previous.rate) {
            fire(tag::SAMPLE_RATE.shortKey());
        }
        if (facts.name != _previous.name) {
            fire(tag::SIGNAL_NAME.shortKey());
        }
        if (facts.quantity != _previous.quantity) {
            fire(tag::SIGNAL_QUANTITY.shortKey());
        }
        if (facts.unit != _previous.unit) {
            fire(tag::SIGNAL_UNIT.shortKey());
        }
        if (facts.range != _previous.range) {
            fire("signal_range");
        }

        std::optional<std::uint64_t> dropped{};
        if (facts.sampleStart.has_value() && _previous.sampleStart.has_value()) {
            const std::uint64_t expected = *_previous.sampleStart + static_cast<std::uint64_t>(_previous.length);
            if (*facts.sampleStart != expected) {
                fire("gap");
                // a record starting before the last one ended repeats samples rather than dropping any, and there is
                // no reserved key for that; the same samples twice are also not countable in a gr::Size_t
                if (*facts.sampleStart > expected && *facts.sampleStart - expected <= std::numeric_limits<gr::Size_t>::max()) {
                    dropped = *facts.sampleStart - expected;
                }
            }
        }
        if (causes.empty()) {
            return;
        }
        detail::putDerived(map, "discontinuity", pmt::Value(causes), nMetaKeysOverridden);
        if (dropped.has_value()) {
            detail::putDerived(map, tag::N_DROPPED_SAMPLES.shortKey(), pmt::Value(static_cast<gr::Size_t>(*dropped)), nMetaKeysOverridden);
        }
    }

    /// @brief Count the record's unattachable events and, only when its indices are unsorted, order them once.
    void planTimingEvents(const DataSet<T>& ds, std::size_t signalIndex, std::size_t nSamples) {
        _eventsSorted = true;
        _eventOrder.clear();
        if (signalIndex >= ds.timing_events.size()) {
            return;
        }
        const auto& events = ds.timing_events[signalIndex];
        for (const auto& event : events) {
            if (event.first < 0 || static_cast<std::size_t>(event.first) >= nSamples) {
                ++nDroppedTimingEvents; // no sample of this record carries it, and a clamped position would be false
            }
        }
        _eventsSorted = std::ranges::is_sorted(events, {}, &DataSet<T>::idx_pmt_map::first);
        if (!_eventsSorted) { // publishTag requires ascending offsets; no producer of a record reaches here
            _eventOrder.resize(events.size());
            std::iota(_eventOrder.begin(), _eventOrder.end(), std::uint32_t{0U});
            std::ranges::stable_sort(_eventOrder, {}, [&events](std::uint32_t index) { return events[static_cast<std::size_t>(index)].first; });
        }
    }

    /// @brief Publish every event of the record falling in the samples this call emits, keys verbatim.
    void publishTimingEvents(OutputSpanLike auto& outSpan, const DataSet<T>& ds, std::size_t signalIndex, std::size_t nSamples, std::size_t nToEmit) {
        if (signalIndex >= ds.timing_events.size()) {
            return;
        }
        const auto&       events = ds.timing_events[signalIndex];
        const std::size_t upper  = _sampleCursor + nToEmit;
        while (_eventCursor < events.size()) {
            const auto& event = _eventsSorted ? events[_eventCursor] : events[static_cast<std::size_t>(_eventOrder[_eventCursor])];
            if (event.first < 0 || static_cast<std::size_t>(event.first) >= nSamples) {
                ++_eventCursor; // already counted when the record started
                continue;
            }
            const auto index = static_cast<std::size_t>(event.first);
            if (index >= upper) {
                break;
            }
            outSpan.publishTag(event.second, index - _sampleCursor);
            ++_eventCursor;
        }
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_DATASETTOSTREAM_HPP
