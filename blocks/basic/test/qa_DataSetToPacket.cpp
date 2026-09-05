#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/YamlPmt.hpp>

#include <gnuradio-4.0/algorithm/dataset/DataSetHelper.hpp>
#include <gnuradio-4.0/basic/DataSetToPacket.hpp>
#include <gnuradio-4.0/digital/AccessCodeCorrelator.hpp>
#include <gnuradio-4.0/digital/CrcBlocks.hpp>
#include <gnuradio-4.0/digital/PacketFramer.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

// The block is a carrier boundary, and a lossy one: fourteen record fields become four packet fields. So the
// properties asserted below are boundary properties, and half of them are absences — what does *not* reach the packet
// is as much the contract as what does, because a packet that quietly carried an interior position would make the
// carrier's whole meaning negotiable. The other load-bearing half is typing: a vocabulary key at the wrong width is
// invisible rather than wrong on the far side of a transport, so every declared type is asserted through the exact
// accessor a peer would use.

namespace {

using gr::blocks::basic::DataSetToPacket;

template<typename T>
using Record = gr::DataSet<T>;

template<typename T>
using Pkt = gr::Packet<T>;

// ─── a minimal three-port span harness ────────────────────────────────────────────────────────────────────────────
// `tags()` yields the framework's own (relative index, map) pairs, including the negative index an unconsumed tag is
// presented at, which is what the block's skip guard is for.

using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;
using gr::blocks::testing::span::TagView;

/// @brief Everything one run of the block published, with tag offsets absolute in the port they were published on.
template<typename T>
struct Capture {
    std::vector<Pkt<T>>    packets{};
    std::vector<gr::Tag>   packetTags{};
    std::vector<Record<T>> rejected{};
    std::vector<gr::Tag>   rejectTags{};
    std::size_t            consumed = 0UZ;
    std::size_t            calls    = 0UZ;
    bool                   stalled  = false;
};

/**
 * @brief Drive @p block over @p records with an output span of @p outRoom packets.
 *
 * @p outRoom of 0 offers room for every record in one span. @p inTags carry an absolute *record* index, which is what
 * a `DataSet` port's tag index means. Consumed tags stay in view so that the negative relative index the framework
 * presents an already-visited tag at is exercised rather than assumed away.
 */
template<typename T>
[[nodiscard]] Capture<T> run(DataSetToPacket<T>& block, std::span<const Record<T>> records, std::size_t outRoom = 0UZ, std::span<const gr::Tag> inTags = {}, bool rejectConnected = true, bool outConnected = true) {
    Capture<T>        capture;
    const std::size_t room = outRoom == 0UZ ? std::max(records.size(), 1UZ) : outRoom;

    std::vector<Pkt<T>>    outScratch(room);
    std::vector<Record<T>> rejectScratch(records.size() + 1UZ);

    std::size_t consumed = 0UZ;
    while (consumed < records.size()) {
        InputSpan<Record<T>>  inSpan(records.subspan(consumed), consumed, inTags);
        OutputSpan<Pkt<T>>    outSpan(outConnected ? std::span<Pkt<T>>(outScratch) : std::span<Pkt<T>>{}, capture.packets.size(), &capture.packetTags, outConnected);
        OutputSpan<Record<T>> rejectSpan(rejectConnected ? std::span<Record<T>>(rejectScratch) : std::span<Record<T>>{}, capture.rejected.size(), &capture.rejectTags, rejectConnected);

        std::ignore = block.processBulk(inSpan, outSpan, rejectSpan);
        ++capture.calls;

        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            capture.packets.push_back(std::move(outScratch[k]));
        }
        for (std::size_t k = 0UZ; k < rejectSpan.count; ++k) {
            capture.rejected.push_back(std::move(rejectScratch[k]));
        }
        consumed += inSpan.consumed;
        capture.consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && rejectSpan.count == 0UZ) {
            capture.stalled = true; // no progress against a span it will be offered again: the graph would wedge here
            break;
        }
    }
    return capture;
}

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

// ─── record builders ──────────────────────────────────────────────────────────────────────────────────────────────

/// @brief `value` as a sample of type @p T, the complex case taking it as the real part; explicit at every step so
/// that -Wconversion has nothing to say about a test's own scaffolding.
template<typename T>
[[nodiscard]] constexpr T sampleValue(std::size_t value) {
    if constexpr (gr::meta::complex_like<T>) {
        return T{static_cast<typename T::value_type>(value), typename T::value_type{}};
    } else {
        return static_cast<T>(value);
    }
}

/// @brief A record of @p nSignals signals of @p nSamples samples each, signal `i` holding `1000*i + j`.
template<typename T>
[[nodiscard]] Record<T> makeRecord(std::size_t nSamples, std::size_t nSignals = 1UZ) {
    Record<T> record;
    record.extents.push_back(static_cast<std::int32_t>(nSamples));
    record.signal_values.resize(nSignals * nSamples);
    for (std::size_t signal = 0UZ; signal < nSignals; ++signal) {
        record.signal_names.push_back(std::format("signal{}", signal));
        record.signal_quantities.push_back(std::format("quantity{}", signal));
        record.signal_units.push_back(std::format("unit{}", signal));
        record.signal_ranges.push_back(gr::Range<T>{sampleValue<T>(0UZ), sampleValue<T>(signal + 1UZ)});
        record.meta_information.emplace_back();
        record.timing_events.emplace_back();
        for (std::size_t j = 0UZ; j < nSamples; ++j) {
            record.signal_values[signal * nSamples + j] = sampleValue<T>(1000UZ * signal + j);
        }
    }
    return record;
}

/// @brief The shape `StreamToDataSet` builds: one signal, a `"time"` axis at @p sampleRate, every vector sized to one.
template<typename T>
[[nodiscard]] Record<T> makeExtractedRecord(std::size_t nSamples, float sampleRate) {
    Record<T> record = makeRecord<T>(nSamples);
    record.axis_names.emplace_back("time");
    record.axis_units.emplace_back("s");
    record.axis_values.resize(1UZ);
    record.axis_values[0UZ].reserve(nSamples);
    for (std::size_t j = 0UZ; j < nSamples; ++j) { // StreamToDataSet::fillAxisValues, verbatim
        record.axis_values[0UZ].emplace_back(static_cast<T>(static_cast<float>(static_cast<int>(j)) / sampleRate));
    }
    return record;
}

/// @brief The wave-4 packet convention: samples, one extent, one signal name, one metadata map, and no axis at all.
template<typename T>
[[nodiscard]] Record<T> makePacketRecord(std::size_t nSamples) {
    Record<T> record;
    record.signal_values.resize(nSamples);
    for (std::size_t j = 0UZ; j < nSamples; ++j) {
        record.signal_values[j] = sampleValue<T>(j);
    }
    record.extents.push_back(static_cast<std::int32_t>(nSamples));
    record.signal_names.emplace_back("payload");
    record.signal_quantities.emplace_back("");
    record.signal_units.emplace_back("");
    record.meta_information.resize(1UZ);
    record.timing_events.resize(1UZ);
    return record;
}

void putMeta(gr::property_map& map, std::string_view key, gr::pmt::Value value) { map.insert_or_assign(gr::property_map::key_type(key), std::move(value)); }

// ─── readers ──────────────────────────────────────────────────────────────────────────────────────────────────────

template<typename TValue>
[[nodiscard]] std::optional<TValue> read(const gr::property_map& map, std::string_view key) {
    const auto it = map.find(key);
    if (it == map.end()) {
        return std::nullopt;
    }
    if (const TValue* value = it->second.template get_if<TValue>(); value != nullptr) {
        return *value;
    }
    return std::nullopt;
}

[[nodiscard]] std::optional<std::string> readString(const gr::property_map& map, std::string_view key) {
    const auto it = map.find(key);
    if (it == map.end()) {
        return std::nullopt;
    }
    if (const std::pmr::string* value = it->second.get_if<std::pmr::string>(); value != nullptr) {
        return std::string(std::string_view(*value));
    }
    return std::nullopt;
}

[[nodiscard]] bool hasKey(const gr::property_map& map, std::string_view key) { return map.find(key) != map.end(); }

/// @brief The one metadata map a packet this block produced carries, or an empty one when the shape is wrong.
template<typename T>
[[nodiscard]] gr::property_map metaOf(const Pkt<T>& packet) {
    return packet.meta_information.size() == 1UZ ? packet.meta_information.front() : gr::property_map{};
}

/// @brief The offsets at which a tag carrying @p key was published, in publication order.
[[nodiscard]] std::vector<std::size_t> offsetsOf(std::span<const gr::Tag> tags, std::string_view key) {
    std::vector<std::size_t> offsets;
    for (const gr::Tag& tag : tags) {
        if (hasKey(tag.map, key)) {
            offsets.push_back(tag.index);
        }
    }
    return offsets;
}

// ─── graph-side blocks, so that Packet<T> is exercised as a real port item and not only through a mock span ───────

template<typename T>
struct RecordSource : gr::Block<RecordSource<T>> {
    gr::PortOut<gr::DataSet<T>> out;
    GR_MAKE_REFLECTABLE(RecordSource, out);

    std::vector<gr::DataSet<T>> _records{};
    std::vector<gr::Tag>        _tags{}; ///< index is an absolute record index, ascending
    std::size_t                 _emitted = 0UZ;

    gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t nRecords = std::min(outSpan.size(), _records.size() - _emitted);
        for (std::size_t k = 0UZ; k < nRecords; ++k) {
            for (const gr::Tag& tag : _tags) {
                if (tag.index == _emitted + k) {
                    outSpan.publishTag(tag.map, k);
                }
            }
            outSpan[k] = _records[_emitted + k];
        }
        _emitted += nRecords;
        outSpan.publish(nRecords);
        return _emitted >= _records.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

template<typename TItem>
struct Collector : gr::Block<Collector<TItem>> {
    gr::PortIn<TItem> in;
    GR_MAKE_REFLECTABLE(Collector, in);

    std::vector<TItem>   _items{};
    std::vector<gr::Tag> _tags{};

    gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const auto& [relIndex, tagMap] : inSpan.tags()) {
            if (relIndex >= 0 && relIndex < static_cast<std::ptrdiff_t>(inSpan.size())) {
                _tags.push_back(gr::Tag{_items.size() + static_cast<std::size_t>(relIndex), tagMap.get()});
            }
        }
        for (std::size_t k = 0UZ; k < inSpan.size(); ++k) {
            _items.push_back(inSpan[k]);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

} // namespace

// ─── the vocabulary itself, asserted against the framework's own declarations ──────────────────────────────────────

const boost::ut::suite<"DataSetToPacket vocabulary"> vocabularyTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::basic::detail;
    using namespace gr::blocks::basic::detail::packet;

    // The vocabulary is what the record-metadata declaration says, and a framework retyping breaks the
    // build here rather than a peer at run time
    "the nine borrowed keys keep the framework's declared types"_test = [] {
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::SAMPLE_RATE)>::value_type, float>);
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::FREQUENCY)>::value_type, double>);
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::SIGNAL_NAME)>::value_type, std::string>);
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::SIGNAL_QUANTITY)>::value_type, std::string>);
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::SIGNAL_UNIT)>::value_type, std::string>);
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::SIGNAL_MIN)>::value_type, float>);
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::SIGNAL_MAX)>::value_type, float>);
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::N_DROPPED_SAMPLES)>::value_type, gr::Size_t>);
        static_assert(std::same_as<std::remove_cvref_t<decltype(gr::tag::TRIGGER_NAME)>::value_type, std::string>);
        expect(true) << "the nine borrowed declarations are asserted at compile time";
    };

    "twenty-one keys, each with exactly one declared type"_test = [] {
        using enum VocabularyType;
        const std::pair<std::string_view, VocabularyType> declared[]{                                                                     // borrowed from the framework's reserved stream vocabulary
            {"sample_rate", Float}, {"frequency", Double}, {"signal_name", String}, {"signal_quantity", String}, {"signal_unit", String}, //
            {"signal_min", Float}, {"signal_max", Float}, {"n_dropped_samples", Size}, {"trigger_name", String},
            // defined by the record-metadata vocabulary
            {"protocol", String}, {"schema_version", Size}, {"sample_start", UInt64}, {"sequence", UInt64}, {"discontinuity", String},                                 //
            {"crc_ok", Bool}, {"corrected_errors", Size}, {"uncorrectable_errors", Size}, {"discard_reason", String}, {"source_id", String}, {"dropped_events", Size}, //
            {"header_corrected_errors", UInt64}};

        expect(eq(std::size(declared), 21UZ)) << "twelve defined here and nine borrowed";
        for (const auto& [key, type] : declared) {
            expect(vocabularyType(key) == type) << std::format("{} is declared once", key);
            const std::string prefixed = std::format("gr:{}", key);
            expect(vocabularyType(shortKey(prefixed)) == type) << std::format("gr:{} is the same key once the framework's prefix is stripped", key);
        }

        // a producer's own private keys are unbounded and impose nothing
        for (const std::string_view key : {"ctx", "n_pre", "n_post", "n_max", "sync_errors", "header_ok", "crc_value", "crc_width", "packet_number", "header_flags", "header_items", "trigger_time", "trigger_offset", "trigger_meta_info", "num_channels"}) {
            expect(vocabularyType(key) == NotVocabulary) << std::format("{} is producer-private or a stream-only reserved key", key);
        }
    };

    "a value of the wrong type fails its key's declaration and a private key passes anything"_test = [] {
        expect(holdsVocabularyType(VocabularyType::Float, gr::pmt::Value(48000.f)));
        expect(!holdsVocabularyType(VocabularyType::Float, gr::pmt::Value(48000.0))) << "no promotion, no narrowing, no conversion";
        expect(holdsVocabularyType(VocabularyType::Size, gr::pmt::Value(gr::Size_t{1U})));
        expect(!holdsVocabularyType(VocabularyType::Size, gr::pmt::Value(std::uint64_t{1ULL})));
        expect(holdsVocabularyType(VocabularyType::UInt64, gr::pmt::Value(std::uint64_t{1ULL})));
        expect(!holdsVocabularyType(VocabularyType::UInt64, gr::pmt::Value(gr::Size_t{1U})));
        expect(holdsVocabularyType(VocabularyType::Bool, gr::pmt::Value(true)));
        expect(holdsVocabularyType(VocabularyType::String, gr::pmt::Value(std::string("ax25"))));
        expect(holdsVocabularyType(VocabularyType::NotVocabulary, gr::pmt::Value(std::uint64_t{1ULL})));
        expect(holdsVocabularyType(VocabularyType::NotVocabulary, gr::pmt::Value(gr::property_map{})));
    };
};

// ─── the conversion, driven through spans ─────────────────────────────────────────────────────────────────────────

const boost::ut::suite<"DataSetToPacket"> dataSetToPacketTests = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::basic;

    // Every field of the record has one fate, and the absences are half the contract
    "every field of the record has one fate"_test = [] {
        Record<float> record = makeExtractedRecord<float>(8UZ, 48000.f);
        record.default_value = 42.f;
        record.timestamp     = 1'700'000'000'000'000'000LL;
        record.layout        = LayoutRight{};
        putMeta(record.meta_information[0UZ], "ctx", pmt::Value(std::string("FAIR.SELECTOR")));
        putMeta(record.meta_information[0UZ], "n_pre", pmt::Value(gr::Size_t{3U}));
        putMeta(record.meta_information[0UZ], "sample_start", pmt::Value(std::uint64_t{4096ULL}));
        record.timing_events[0UZ].emplace_back(2, property_map{{"marker", std::string("edge")}});
        record.timing_events[0UZ].emplace_back(7, property_map{{"marker", std::string("last")}});

        auto                block = make<DataSetToPacket<float>>();
        const Record<float> input[]{record};
        const auto          capture = run<float>(block, input);

        expect(eq(capture.consumed, 1UZ));
        expect(eq(capture.packets.size(), 1UZ));
        const Pkt<float>& packet = capture.packets.front();

        expect(eq(packet.meta_information.size(), 1UZ)) << "a packet this block produces carries exactly one map";
        expect(std::ranges::equal(packet.signal_values, std::vector<float>{0.f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f})) << "the selected signal's items, and nothing else";
        expect(eq(packet.timestamp, 1'700'000'000'000'000'000LL)) << "carried into the same field, both being int64";
        expect(eq(packet.default_value, 42.f)) << "carried into the same field";

        const property_map meta = metaOf(packet);
        expect(eq(readString(meta, "signal_name").value_or(""), std::string("signal0"))) << "declared string";
        expect(eq(readString(meta, "signal_quantity").value_or(""), std::string("quantity0")));
        expect(eq(readString(meta, "signal_unit").value_or(""), std::string("unit0")));
        expect(eq(read<float>(meta, "signal_min").value_or(-1.f), 0.f)) << "declared float";
        expect(eq(read<float>(meta, "signal_max").value_or(-1.f), 1.f));
        expect(read<float>(meta, "sample_rate").has_value()) << "declared float, salvaged from the axis";
        expect(eq(read<gr::Size_t>(meta, "dropped_events").value_or(0U), gr::Size_t{2U})) << "declared gr::Size_t";
        expect(eq(readString(meta, "ctx").value_or(""), std::string("FAIR.SELECTOR"))) << "meta_information copied verbatim";
        expect(eq(read<gr::Size_t>(meta, "n_pre").value_or(99U), gr::Size_t{3U}));
        expect(eq(read<std::uint64_t>(meta, "sample_start").value_or(0ULL), 4096ULL)) << "provenance crosses, as declared uint64";

        // the absences, which is the half a coverage test usually omits
        expect(!hasKey(meta, "extents"));
        expect(!hasKey(meta, "layout"));
        expect(!hasKey(meta, "axis_values"));
        expect(!hasKey(meta, "axis_names"));
        expect(!hasKey(meta, "timing_events"));
        expect(!hasKey(meta, "marker")) << "no interior annotation reaches the packet under any spelling";
        expect(!hasKey(meta, "schema_version")) << "version 1 writes no version";
        expect(!hasKey(meta, "protocol")) << "an unset label stamps nothing";
        expect(!hasKey(meta, "source_id"));
        expect(eq(capture.packetTags.size(), 0UZ)) << "no tag is ever published on out";
    };

    // A wrongly typed vocabulary key reads as nothing, so it is dropped and counted
    "a mistyped vocabulary key is dropped and a private key of any type is not"_test = [] {
        { // (a) sample_rate as a double is not a slightly wrong rate
            Record<float> record = makeRecord<float>(4UZ);
            putMeta(record.meta_information[0UZ], "sample_rate", pmt::Value(48000.0));
            auto                block = make<DataSetToPacket<float>>();
            const Record<float> input[]{record};
            const auto          capture = run<float>(block, input);
            const property_map  meta    = metaOf(capture.packets.at(0UZ));
            expect(eq(read<float>(meta, "sample_rate").value_or(-1.f), -1.f)) << "a typed read returns the fallback";
            expect(!hasKey(meta, "sample_rate")) << "and the key is not there at all";
            expect(eq(block.nMetaKeysDropped, 1ULL));
        }
        { // (b) the same key at its declared width crosses and equals
            Record<float> record = makeRecord<float>(4UZ);
            putMeta(record.meta_information[0UZ], "sample_rate", pmt::Value(48000.f));
            auto                block = make<DataSetToPacket<float>>();
            const Record<float> input[]{record};
            const auto          capture = run<float>(block, input);
            expect(eq(read<float>(metaOf(capture.packets.at(0UZ)), "sample_rate").value_or(0.f), 48000.f));
            expect(eq(block.nMetaKeysDropped, 0ULL));
        }
        { // (c) sample_start as a gr::Size_t is the same trap one width down
            Record<float> record = makeRecord<float>(4UZ);
            putMeta(record.meta_information[0UZ], "sample_start", pmt::Value(gr::Size_t{7U}));
            auto                block = make<DataSetToPacket<float>>();
            const Record<float> input[]{record};
            const auto          capture = run<float>(block, input);
            expect(!hasKey(metaOf(capture.packets.at(0UZ)), "sample_start"));
            expect(eq(block.nMetaKeysDropped, 1ULL));
        }
        { // (d) a producer-private key is copied unexamined, whatever its type
            Record<float> record = makeRecord<float>(4UZ);
            putMeta(record.meta_information[0UZ], "crc_width", pmt::Value(std::uint64_t{32ULL}));
            putMeta(record.meta_information[0UZ], "sync_errors", pmt::Value(gr::Size_t{2U}));
            putMeta(record.meta_information[0UZ], "trigger_meta_info", pmt::Value(property_map{}));
            auto                block = make<DataSetToPacket<float>>();
            const Record<float> input[]{record};
            const auto          capture = run<float>(block, input);
            const property_map  meta    = metaOf(capture.packets.at(0UZ));
            expect(eq(read<std::uint64_t>(meta, "crc_width").value_or(0ULL), 32ULL));
            expect(eq(read<gr::Size_t>(meta, "sync_errors").value_or(0U), gr::Size_t{2U}));
            expect(hasKey(meta, "trigger_meta_info"));
            expect(eq(block.nMetaKeysDropped, 0ULL));
        }
    };

    // The conversion's limits, asserted as differences rather than as a round-trip claim
    "what a packet cannot reproduce is gone and counted"_test = [] {
        Record<float> record = makeExtractedRecord<float>(6UZ, 48000.f);
        record.signal_names.push_back("signal1");
        record.signal_quantities.push_back("quantity1");
        record.signal_units.push_back("unit1");
        record.signal_ranges.push_back(gr::Range<float>{0.f, 2.f});
        record.meta_information.emplace_back();
        record.timing_events.emplace_back();
        record.signal_values.resize(12UZ);
        for (std::size_t j = 0UZ; j < 6UZ; ++j) {
            record.signal_values[6UZ + j] = static_cast<float>(1000UZ + j);
        }
        record.timing_events[0UZ].emplace_back(0, property_map{{"event", std::string("first")}});
        record.timing_events[0UZ].emplace_back(3, property_map{{"event", std::string("middle")}});
        record.timing_events[1UZ].emplace_back(5, property_map{{"event", std::string("other signal")}});

        auto                block = make<DataSetToPacket<float>>();
        const Record<float> input[]{record};
        const auto          capture = run<float>(block, input);

        expect(eq(capture.packets.size(), 1UZ));
        const property_map meta = metaOf(capture.packets.front());
        expect(eq(capture.packets.front().signal_values.size(), 6UZ)) << "one signal's items, not both";
        expect(eq(read<gr::Size_t>(meta, "dropped_events").value_or(0U), gr::Size_t{3U})) << "every entry of every signal's list";
        expect(eq(block.nDroppedTimingEvents, 3ULL));
        expect(eq(block.nSignalsNotEmitted, 1ULL));
        expect(!hasKey(meta, "event")) << "no event's own keys survive under their own names";
        for (const auto& [key, value] : meta) { // and none under any other name either
            expect(std::string_view(key) != "timing_events");
            expect(value.get_if<property_map>() == nullptr) << std::format("{} is not a folded container", std::string_view(key));
        }
        expect(!hasKey(meta, "extents"));
        expect(!hasKey(meta, "layout"));
    };

    // A record cannot stop the graph, and a missing annotation is not a malformed record
    "twelve malformed records are handled without throwing"_test = [] {
        // every record carries two signals and the block reads the second, so the one-signal record is the one whose signal is absent
        std::vector<Record<float>> records;
        Record<float>              noSignals = makeRecord<float>(4UZ, 2UZ);
        noSignals.signal_names.clear();
        records.push_back(std::move(noSignals));
        records.push_back(makeRecord<float>(4UZ, 1UZ));
        Record<float> twoExtents = makeRecord<float>(4UZ, 2UZ);
        twoExtents.extents.push_back(2);
        records.push_back(std::move(twoExtents));
        Record<float> zeroExtent = makeRecord<float>(4UZ, 2UZ);
        zeroExtent.extents[0UZ]  = 0;
        records.push_back(std::move(zeroExtent));
        Record<float> negativeExtent = makeRecord<float>(4UZ, 2UZ);
        negativeExtent.extents[0UZ]  = -4;
        records.push_back(std::move(negativeExtent));
        Record<float> wrongExtent = makeRecord<float>(4UZ, 2UZ);
        wrongExtent.extents[0UZ]  = 3;
        records.push_back(std::move(wrongExtent));
        Record<float> ragged = makeRecord<float>(4UZ, 2UZ);
        ragged.signal_values.pop_back();
        records.push_back(std::move(ragged));
        Record<float> empty = makeRecord<float>(4UZ, 2UZ);
        empty.signal_values.clear();
        empty.extents.clear();
        records.push_back(std::move(empty));
        Record<float> shortUnits = makeRecord<float>(4UZ, 2UZ);
        shortUnits.signal_units.clear();
        records.push_back(std::move(shortUnits)); // admitted: an absent annotation is not a malformed record
        Record<float> noMeta = makeRecord<float>(4UZ, 2UZ);
        noMeta.meta_information.clear();
        records.push_back(std::move(noMeta)); // admitted
        Record<float> noEvents = makeRecord<float>(4UZ, 2UZ);
        noEvents.timing_events.clear();
        records.push_back(std::move(noEvents)); // admitted
        Record<float> noExtents = makeRecord<float>(4UZ, 2UZ);
        noExtents.extents.clear();
        records.push_back(std::move(noExtents)); // admitted: the target carrier has no field for the shape

        auto           block = make<DataSetToPacket<float>>({{"signal_index", gr::Size_t{1U}}});
        Capture<float> capture;
        expect(nothrow([&] { capture = run<float>(block, std::span<const Record<float>>(records)); })) << "no accessor on the sample path throws";
        expect(!capture.stalled);
        expect(eq(capture.consumed, records.size()));
        expect(eq(block.nRejectedRecords, 8ULL));
        expect(eq(capture.rejected.size(), 8UZ));
        expect(eq(capture.packets.size(), 4UZ)) << "the last four are admitted";

        const std::vector<std::string> expected{"no_signals", "signal_index_out_of_range", "not_one_dimensional", "inconsistent_extent", "inconsistent_extent", "inconsistent_extent", "ragged_signals", "empty_payload"};
        std::vector<std::string>       reasons;
        for (const Tag& tag : capture.rejectTags) {
            reasons.push_back(readString(tag.map, "discard_reason").value_or("<missing>"));
        }
        expect(std::ranges::equal(reasons, expected)) << "each rejection names the first clause that failed";
        expect(eq(capture.rejected.at(1UZ).signal_names.size(), 1UZ)) << "the record travels unchanged";
        for (const Pkt<float>& packet : capture.packets) {
            expect(eq(packet.signal_values.size(), 4UZ));
            expect(eq(packet.signal_values.front(), 1000.f)) << "signal 1, in every admitted case";
        }
    };

    // The boundary predicate and the framework's validator agree on the packet
    "a packet built here is admitted by the block and by the framework's validator"_test = [] {
        const Record<std::uint8_t> packet = makePacketRecord<std::uint8_t>(17UZ);
        expect(gr::dataset::checkConsistency(packet).has_value()) << "an axis-free record validates: a payload index is not a physical axis";

        auto                       block = make<DataSetToPacket<std::uint8_t>>();
        const Record<std::uint8_t> input[]{packet};
        const auto                 capture = run<std::uint8_t>(block, input);
        expect(eq(capture.packets.size(), 1UZ)) << "and yet every one of its items reaches the packet";
        expect(eq(capture.packets.front().signal_values.size(), 17UZ));
        expect(eq(block.nRejectedRecords, 0ULL));

        // a record with no extents at all: admitted here, and refused by the record-to-stream predicate, which needs one
        Record<std::uint8_t> noExtents = makePacketRecord<std::uint8_t>(9UZ);
        noExtents.extents.clear();
        expect(noExtents.extents.empty()) << "which is exactly the extent list the record-to-stream predicate requires to be of size one";
        auto                       relaxed = make<DataSetToPacket<std::uint8_t>>();
        const Record<std::uint8_t> relaxedInput[]{noExtents};
        const auto                 relaxedCapture = run<std::uint8_t>(relaxed, relaxedInput);
        expect(eq(relaxedCapture.packets.size(), 1UZ));
        expect(eq(relaxedCapture.packets.front().signal_values.size(), 9UZ));
        expect(eq(relaxed.nRejectedRecords, 0ULL));
    };

    // Schema_version in version 1: read, never written, never a reason to refuse
    "a stated version crosses and an unstated one stays unstated"_test = [] {
        { // (a) absent stays absent
            auto                block = make<DataSetToPacket<float>>();
            const Record<float> input[]{makeRecord<float>(4UZ)};
            const auto          capture = run<float>(block, input);
            expect(!hasKey(metaOf(capture.packets.at(0UZ)), "schema_version")) << "no key of this version has a since above 1";
        }
        for (const gr::Size_t version : {gr::Size_t{1U}, gr::Size_t{7U}}) { // (b) and (c)
            Record<float> record = makeRecord<float>(4UZ);
            putMeta(record.meta_information[0UZ], "schema_version", pmt::Value(version));
            auto                block = make<DataSetToPacket<float>>();
            const Record<float> input[]{record};
            const auto          capture = run<float>(block, input);
            expect(eq(capture.packets.size(), 1UZ)) << std::format("version {}: a higher version is never a rejection", version);
            expect(eq(read<gr::Size_t>(metaOf(capture.packets.front()), "schema_version").value_or(0U), version));
            expect(eq(capture.packets.front().signal_values.size(), 4UZ)) << "and the record is otherwise converted normally";
            expect(eq(block.nRejectedRecords, 0ULL));
        }
    };

    // The stamps fill a gap and never overwrite the record's own word
    "a label stamps only what the record left unstated"_test = [] {
        const auto labeled = [](std::string_view setting, std::string_view label) {
            property_map settings;
            putMeta(settings, setting, pmt::Value(std::string(label)));
            return settings;
        };
        const auto check = [&labeled](std::string_view setting, std::string_view key, std::string_view stated) {
            { // (a) the record says nothing, so the label is written
                auto                block = make<DataSetToPacket<float>>(labeled(setting, "ax25"));
                const Record<float> input[]{makeRecord<float>(4UZ)};
                const auto          capture = run<float>(block, input);
                expect(eq(readString(metaOf(capture.packets.at(0UZ)), key).value_or(""), std::string("ax25"))) << key;
                expect(eq(block.nStampsDeclined, 0ULL)) << key;
            }
            { // (b) the record already said so, and the record is the better informed source
                Record<float> record = makeRecord<float>(4UZ);
                putMeta(record.meta_information[0UZ], key, pmt::Value(std::string(stated)));
                auto                block = make<DataSetToPacket<float>>(labeled(setting, "ax25"));
                const Record<float> input[]{record};
                const auto          capture = run<float>(block, input);
                expect(eq(readString(metaOf(capture.packets.at(0UZ)), key).value_or(""), std::string(stated))) << key;
                expect(eq(block.nStampsDeclined, 1ULL)) << key;
            }
            { // (c) an empty label writes nothing and declines nothing
                auto                block = make<DataSetToPacket<float>>(labeled(setting, ""));
                const Record<float> input[]{makeRecord<float>(4UZ)};
                const auto          capture = run<float>(block, input);
                expect(!hasKey(metaOf(capture.packets.at(0UZ)), key)) << key;
                expect(eq(block.nStampsDeclined, 0ULL)) << key;
            }
        };
        check("protocol_label", "protocol", "ccsds/tm");
        check("source_label", "source_id", "dev0");
    };

    // The receiver's own fact: a chain is handed rates and gains and never a tuning, so `frequency` is
    // the third key the record cannot state and the block can, on the same terms as the two labels
    "the tuning stamps only what the record left unstated"_test = [] {
        constexpr double kTuned  = 433'921'337.0; // deliberately off a channel grid: a rounded stamp is not this
        constexpr double kStated = 145'825'000.0;
        { // (a) the record says nothing, so the tuning is written, and at the width a peer reads
            auto                       block = make<DataSetToPacket<std::uint8_t>>({{"frequency_hz", kTuned}});
            const Record<std::uint8_t> input[]{makePacketRecord<std::uint8_t>(8UZ)};
            const auto                 capture = run<std::uint8_t>(block, input);
            expect(eq(read<double>(metaOf(capture.packets.at(0UZ)), "frequency").value_or(0.), kTuned));
            expect(!read<float>(metaOf(capture.packets.at(0UZ)), "frequency").has_value()) << "declared double, so a float read has to find nothing";
            expect(eq(block.nStampsDeclined, 0ULL));
        }
        { // (b) the record already said so, and a record that knows its own tuning is the better informed source
            Record<std::uint8_t> record = makePacketRecord<std::uint8_t>(8UZ);
            putMeta(record.meta_information[0UZ], "frequency", pmt::Value(kStated));
            auto                       block = make<DataSetToPacket<std::uint8_t>>({{"frequency_hz", kTuned}});
            const Record<std::uint8_t> input[]{record};
            const auto                 capture = run<std::uint8_t>(block, input);
            expect(eq(read<double>(metaOf(capture.packets.at(0UZ)), "frequency").value_or(0.), kStated));
            expect(eq(block.nStampsDeclined, 1ULL));
        }
        { // (c) zero states no tuning, so nothing is written and nothing is declined
            auto                       block = make<DataSetToPacket<std::uint8_t>>();
            const Record<std::uint8_t> input[]{makePacketRecord<std::uint8_t>(8UZ)};
            const auto                 capture = run<std::uint8_t>(block, input);
            expect(!hasKey(metaOf(capture.packets.at(0UZ)), "frequency"));
            expect(eq(block.nStampsDeclined, 0ULL));
        }
        { // (d) one counter for all three stamps, so a graph author who set them and sees none has a number
            Record<std::uint8_t> record = makePacketRecord<std::uint8_t>(8UZ);
            putMeta(record.meta_information[0UZ], "protocol", pmt::Value(std::string("ax25")));
            putMeta(record.meta_information[0UZ], "source_id", pmt::Value(std::string("framer")));
            putMeta(record.meta_information[0UZ], "frequency", pmt::Value(kStated));
            auto                       block = make<DataSetToPacket<std::uint8_t>>({{"protocol_label", std::string("ccsds/tm")}, {"source_label", std::string("dev0")}, {"frequency_hz", kTuned}});
            const Record<std::uint8_t> input[]{record};
            const auto                 capture = run<std::uint8_t>(block, input);
            expect(eq(block.nStampsDeclined, 3ULL));
            expect(eq(readString(metaOf(capture.packets.at(0UZ)), "protocol").value_or(""), std::string("ax25")));
            expect(eq(readString(metaOf(capture.packets.at(0UZ)), "source_id").value_or(""), std::string("framer")));
            expect(eq(read<double>(metaOf(capture.packets.at(0UZ)), "frequency").value_or(0.), kStated));
        }
    };

    // The stamp is worth its precision only if the encoding a transport uses keeps it. `gr::pmt::yaml` is that
    // encoding, and an emitter that writes `std::ostream`'s default six significant digits rounds a tuning near
    // 1 GHz to 10 kHz, where a receiver on a channel center sees no error at all -- which is what makes that loss
    // hard to notice. Round numbers are therefore not enough to assert on, and none of these is one.
    "a stamped tuning survives the metadata encoding to the hertz"_test = [] {
        for (const double hz : {145'825'001.0, 433'921'337.0, 1'090'000'007.0, 2'400'000'001.0}) {
            auto                       block = make<DataSetToPacket<std::uint8_t>>({{"frequency_hz", hz}});
            const Record<std::uint8_t> input[]{makePacketRecord<std::uint8_t>(8UZ)};
            const auto                 capture = run<std::uint8_t>(block, input);
            const auto                 parsed  = gr::pmt::yaml::deserialize(gr::pmt::yaml::serialize(metaOf(capture.packets.at(0UZ))));
            expect(parsed.has_value()) << std::format("{} Hz: the stamped map has to be a document a peer can read", hz);
            if (!parsed.has_value()) {
                continue;
            }
            const auto back = read<double>(*parsed, "frequency");
            expect(back.has_value()) << std::format("{} Hz: and it has to come back as a double", hz);
            expect(eq(back.value_or(0.), hz)) << std::format("{} Hz: a tuning that reaches a peer rounded names a different channel", hz);
        }
    };

    // Multi-signal fan-out is a composition, and it is packet-aligned
    "four instances on one record port produce four aligned packet streams"_test = [] {
        const std::vector<Record<float>> records{makeRecord<float>(6UZ, 4UZ), makeRecord<float>(11UZ, 4UZ)};
        std::vector<Capture<float>>      captures;
        for (gr::Size_t index = 0U; index < 4U; ++index) {
            auto block = make<DataSetToPacket<float>>({{"signal_index", index}});
            captures.push_back(run<float>(block, std::span<const Record<float>>(records)));
            expect(eq(block.nSignalsNotEmitted, 6ULL)) << std::format("signal_index {}: three per record, two records", index);
        }
        for (gr::Size_t index = 0U; index < 4U; ++index) {
            expect(eq(captures[index].packets.size(), 2UZ));
            expect(eq(captures[index].packets.at(0UZ).signal_values.size(), 6UZ));
            expect(eq(captures[index].packets.at(1UZ).signal_values.size(), 11UZ));
            expect(eq(captures[index].packets.at(0UZ).signal_values.front(), static_cast<float>(1000U * index)));
            expect(eq(readString(metaOf(captures[index].packets.at(0UZ)), "signal_name").value_or(""), std::format("signal{}", index)));
        }
        // the one that matters for the no-move rule: every instance saw an intact record
        expect(eq(records.at(0UZ).signal_values.size(), 24UZ)) << "the input records are never moved from";
        expect(eq(records.at(1UZ).signal_names.size(), 4UZ));

        auto                narrow = make<DataSetToPacket<float>>({{"signal_index", gr::Size_t{4U}}});
        const Record<float> input[]{makeRecord<float>(6UZ, 4UZ)};
        const auto          rejected = run<float>(narrow, input);
        expect(eq(rejected.rejected.size(), 1UZ));
        expect(eq(readString(rejected.rejectTags.at(0UZ).map, "discard_reason").value_or(""), std::string("signal_index_out_of_range")));
    };

    // The rate's three routes, in order, and the fourth outcome that is not a failure
    "the rate routes are tried in order"_test = [] {
        { // (a) the record's own statement beats an axis saying something else
            Record<float> record = makeExtractedRecord<float>(1024UZ, 1e6f);
            putMeta(record.meta_information[0UZ], "sample_rate", pmt::Value(48000.f));
            auto                block = make<DataSetToPacket<float>>();
            const Record<float> input[]{record};
            const auto          capture = run<float>(block, input);
            expect(eq(read<float>(metaOf(capture.packets.at(0UZ)), "sample_rate").value_or(0.f), 48000.f));
            expect(eq(block.nMetaKeysOverridden, 0ULL)) << "route 1 is the absence of a derivation, so it displaces nothing";
        }
        { // (b) the port's most recent tag beats the axis
            const Record<float>    input[]{makeExtractedRecord<float>(1024UZ, 1e6f)};
            const std::vector<Tag> inTags{Tag{0UZ, property_map{{"sample_rate", 96000.f}}}};
            auto                   block   = make<DataSetToPacket<float>>();
            const auto             capture = run<float>(block, input, 0UZ, std::span<const Tag>(inTags));
            expect(eq(read<float>(metaOf(capture.packets.at(0UZ)), "sample_rate").value_or(0.f), 96000.f));
        }
        { // (c) the axis, to within the endpoint form's own error
            for (const std::size_t length : {1024UZ, 65536UZ}) {
                const Record<float> input[]{makeExtractedRecord<float>(length, 48000.f)};
                auto                block   = make<DataSetToPacket<float>>();
                const auto          capture = run<float>(block, input);
                const auto          rate    = read<float>(metaOf(capture.packets.at(0UZ)), "sample_rate");
                expect(rate.has_value()) << std::format("length {}", length);
                expect(lt(std::abs(static_cast<double>(rate.value_or(0.f)) - 48000.0) / 48000.0, 3.2e-08)) << std::format("length {}: a bound, not an equality", length);
            }
        }
        { // (d) a byte record has no readable axis, and an absent rate is information rather than a drop
            const Record<std::uint8_t> input[]{makePacketRecord<std::uint8_t>(64UZ)};
            auto                       block   = make<DataSetToPacket<std::uint8_t>>();
            const auto                 capture = run<std::uint8_t>(block, input);
            expect(!hasKey(metaOf(capture.packets.at(0UZ)), "sample_rate"));
            expect(eq(block.nRejectedRecords, 0ULL));
            expect(eq(block.nMetaKeysDropped, 0ULL));
            expect(eq(block.nMetaKeysOverridden, 0ULL));
            expect(eq(block.nDroppedTimingEvents, 0ULL));
            expect(eq(block.nStampsDeclined, 0ULL));
        }
    };

    // The compile-time half
    "the block is not admissible for UnfilteredTagPropagation"_test = [] {
        static_assert(!gr::block::kUnfilteredTagPropagationAdmissible<DataSetToPacket<std::uint8_t>>, "asynchronous outputs and NoTagPropagation each refuse the flag on their own");
        static_assert(!gr::block::kUnfilteredTagPropagationAdmissible<DataSetToPacket<float>>);
        expect(!gr::block::kUnfilteredTagPropagationAdmissible<DataSetToPacket<std::uint8_t>>);
    };

    // A tag belongs to its record and to no other, and nothing is published on out
    "an input tag lands in its record's packet and nowhere else"_test = [] {
        std::vector<Record<float>> records;
        for (std::size_t k = 0UZ; k < 9UZ; ++k) {
            records.push_back(makeRecord<float>(5UZ));
        }
        for (const std::size_t at : {0UZ, 1UZ, 7UZ}) {
            const std::vector<Tag> inTags{Tag{at, property_map{{"sample_rate", 96000.f}, {"probe", std::string("here")}}}};
            for (const std::size_t room : {0UZ, 1UZ, 4UZ}) {
                auto       block   = make<DataSetToPacket<float>>();
                const auto capture = run<float>(block, std::span<const Record<float>>(records), room, std::span<const Tag>(inTags));
                expect(eq(capture.packets.size(), 9UZ)) << std::format("record {} room {}", at, room);
                std::size_t carriers = 0UZ;
                for (std::size_t k = 0UZ; k < capture.packets.size(); ++k) {
                    if (hasKey(metaOf(capture.packets[k]), "probe")) {
                        ++carriers;
                        expect(eq(k, at)) << std::format("record {} room {}: the tag belongs to its own record", at, room);
                    }
                }
                expect(eq(carriers, 1UZ)) << std::format("record {} room {}: multiplicity 1", at, room);
                expect(eq(capture.packetTags.size(), 0UZ)) << "and no tag is published on out, for any input";
            }
        }
        { // a tag key the record also states: the record's copy is displaced by the tag, and the derived key by neither
            Record<float> record = makeRecord<float>(5UZ);
            putMeta(record.meta_information[0UZ], "probe", pmt::Value(std::string("record")));
            const Record<float>    input[]{record};
            const std::vector<Tag> inTags{Tag{0UZ, property_map{{"probe", std::string("tag")}}}};
            auto                   block   = make<DataSetToPacket<float>>();
            const auto             capture = run<float>(block, input, 0UZ, std::span<const Tag>(inTags));
            expect(eq(readString(metaOf(capture.packets.at(0UZ)), "probe").value_or(""), std::string("tag")));
        }
    };

    // Chunk independence, which is what the statelessness claim buys
    "the packets do not depend on the output chunk size"_test = [] {
        std::vector<Record<float>> records;
        const std::size_t          lengths[]{1UZ, 5UZ, 64UZ, 3UZ, 1UZ, 4097UZ, 17UZ, 2UZ, 100UZ, 7UZ, 33UZ, 4096UZ, 9UZ, 11UZ, 1UZ, 256UZ, 8UZ, 13UZ, 21UZ, 34UZ};
        for (std::size_t k = 0UZ; k < std::size(lengths); ++k) {
            Record<float> record = makeRecord<float>(lengths[k]);
            putMeta(record.meta_information[0UZ], "sequence", pmt::Value(static_cast<std::uint64_t>(k)));
            record.timing_events[0UZ].emplace_back(0, property_map{{"event", std::string("first")}});
            records.push_back(std::move(record));
        }

        auto       reference = make<DataSetToPacket<float>>();
        const auto single    = run<float>(reference, std::span<const Record<float>>(records));
        expect(eq(single.packets.size(), std::size(lengths)));
        for (const std::size_t room : {1UZ, 3UZ, 17UZ, 4096UZ}) {
            auto       block   = make<DataSetToPacket<float>>();
            const auto capture = run<float>(block, std::span<const Record<float>>(records), room);
            expect(eq(capture.packets.size(), single.packets.size())) << std::format("room {}: multiplicity", room);
            for (std::size_t k = 0UZ; k < std::min(capture.packets.size(), single.packets.size()); ++k) {
                expect(std::ranges::equal(capture.packets[k].signal_values, single.packets[k].signal_values)) << std::format("room {}: packet {} items", room, k);
                expect(eq(read<std::uint64_t>(metaOf(capture.packets[k]), "sequence").value_or(~0ULL), static_cast<std::uint64_t>(k))) << std::format("room {}: packet {} sequence", room, k);
                expect(eq(read<gr::Size_t>(metaOf(capture.packets[k]), "dropped_events").value_or(0U), gr::Size_t{1U})) << std::format("room {}: packet {} events", room, k);
                expect(eq(metaOf(capture.packets[k]).size(), metaOf(single.packets[k]).size())) << std::format("room {}: packet {} map size", room, k);
            }
            expect(eq(block.nDroppedTimingEvents, reference.nDroppedTimingEvents)) << std::format("room {}: counters", room);
            expect(eq(block.nSignalsNotEmitted, reference.nSignalsNotEmitted)) << std::format("room {}", room);
        }
    };

    // The reject port unconnected, which is the framework question this block leans on
    "an unconnected Optional Async output port takes a span, a publish and a tag"_test = [] {
        DataSetToPacket<float> block; // never connected to anything
        expect(!block.reject.isConnected()) << "an output port with no reader";
        auto span = block.reject.template tryReserve<gr::SpanReleasePolicy::ProcessNone>(block.reject.streamWriter().available());
        expect(gt(span.size(), 0UZ)) << "the framework presents a writable span all the same: an unconnected writer is ungated";
        expect(!span.isConnected);
        expect(nothrow([&] {
            span[0UZ] = makeRecord<float>(2UZ);
            span.publishTag(property_map{{"discard_reason", std::string("no_signals")}}, 0UZ);
            span.publish(1UZ);
        })) << "publish and publishTag are both safe on it";
    };

    "everything is refused the same way with reject unconnected"_test = [] {
        std::vector<Record<float>> records;
        Record<float>              noSignals = makeRecord<float>(4UZ);
        noSignals.signal_names.clear();
        records.push_back(noSignals);
        Record<float> empty = makeRecord<float>(4UZ);
        empty.signal_values.clear();
        empty.extents.clear();
        records.push_back(empty);
        records.push_back(makeRecord<float>(4UZ));

        auto           connected = make<DataSetToPacket<float>>();
        const auto     withPort  = run<float>(connected, std::span<const Record<float>>(records));
        auto           orphaned  = make<DataSetToPacket<float>>();
        Capture<float> withoutPort;
        expect(nothrow([&] { withoutPort = run<float>(orphaned, std::span<const Record<float>>(records), 0UZ, {}, false); }));
        expect(eq(withoutPort.packets.size(), withPort.packets.size()));
        expect(eq(orphaned.nRejectedRecords, connected.nRejectedRecords));
        expect(eq(orphaned.nRejectedRecords, 2ULL));
        expect(eq(withoutPort.rejected.size(), 0UZ)) << "nothing is written to a port nobody reads";
        expect(eq(withoutPort.consumed, records.size())) << "and the records are still consumed";

        auto           noOutput = make<DataSetToPacket<float>>();
        Capture<float> withoutOut;
        expect(nothrow([&] { withoutOut = run<float>(noOutput, std::span<const Record<float>>(records), 0UZ, {}, true, false); }));
        expect(eq(withoutOut.packets.size(), 0UZ));
        expect(eq(withoutOut.consumed, records.size()));
        expect(eq(noOutput.nRejectedRecords, 2ULL)) << "the counters are the same with no packet reader either";
    };

    // Validation rejects at configuration time, and signal_index deliberately does not
    "an unusable tolerance is refused and the working one survives"_test = [] {
        for (const float bad : {0.f, -1.f, std::numeric_limits<float>::quiet_NaN()}) {
            auto block = make<DataSetToPacket<float>>({{"axis_rate_tolerance", 1e-4f}});
            expect(eq(block.axis_rate_tolerance.value, 1e-4f));
            bool threw = false;
            try {
                std::ignore = block.settings().set({{"axis_rate_tolerance", bad}});
                std::ignore = block.settings().activateContext();
                std::ignore = block.settings().applyStagedParameters();
            } catch (const gr::exception& error) {
                threw = true;
                expect(that % (std::string(error.message).find(std::format("{}", bad)) != std::string::npos)) << std::format("the message names {}", bad);
            }
            expect(threw) << std::format("axis_rate_tolerance = {}", bad);
            expect(eq(block.axis_rate_tolerance.value, 1e-4f)) << "the previous configuration is intact";
        }
        expect(nothrow([] { std::ignore = make<DataSetToPacket<float>>({{"signal_index", std::numeric_limits<gr::Size_t>::max()}}); })) << "signal_index is a property of each record, not of the setting";
        auto                block = make<DataSetToPacket<float>>({{"signal_index", std::numeric_limits<gr::Size_t>::max()}});
        const Record<float> input[]{makeRecord<float>(4UZ)};
        const auto          capture = run<float>(block, input);
        expect(eq(capture.rejected.size(), 1UZ));
        expect(eq(readString(capture.rejectTags.at(0UZ).map, "discard_reason").value_or(""), std::string("signal_index_out_of_range")));
        expect(nothrow([] { std::ignore = make<DataSetToPacket<float>>({{"protocol_label", std::string("")}, {"source_label", std::string("")}}); })) << "any string is a legal label, the empty one included";
    };

    "every registered type converts"_test = [] {
        const auto check = []<typename T>() {
            auto            block = make<DataSetToPacket<T>>();
            const Record<T> input[]{makeRecord<T>(5UZ)};
            const auto      capture = run<T>(block, input);
            expect(eq(capture.packets.size(), 1UZ)) << gr::meta::type_name<T>();
            expect(eq(capture.packets.front().signal_values.size(), 5UZ)) << gr::meta::type_name<T>();
            expect(capture.packets.front().signal_values.back() == sampleValue<T>(4UZ)) << gr::meta::type_name<T>();
            expect(eq(readString(metaOf(capture.packets.front()), "signal_name").value_or(""), std::string("signal0"))) << gr::meta::type_name<T>();
        };
        check.template operator()<std::uint8_t>();
        check.template operator()<std::int16_t>();
        check.template operator()<std::int32_t>();
        check.template operator()<float>();
        check.template operator()<std::complex<float>>();

        { // a complex signal_range holds two samples ordered by magnitude, so it is not a pair of float limits
            auto                              block = make<DataSetToPacket<std::complex<float>>>();
            const Record<std::complex<float>> input[]{makeRecord<std::complex<float>>(5UZ)};
            const auto                        capture = run<std::complex<float>>(block, input);
            expect(!hasKey(metaOf(capture.packets.front()), "signal_min"));
            expect(!hasKey(metaOf(capture.packets.front()), "signal_max"));
        }
    };
};

// ─── under the scheduler: Packet<T> as a real port item, and the whole chain end to end ───────────────────────────

namespace {

constexpr std::uint64_t kSyncWord = 0xACDDA4E2F28C20FCULL;
constexpr std::size_t   kSyncBits = 64UZ;

[[nodiscard]] std::string codeString(std::uint64_t word, std::size_t bits) {
    std::string text(bits, '0');
    for (std::size_t i = 0UZ; i < bits; ++i) {
        text[i] = ((word >> (bits - 1UZ - i)) & 1ULL) != 0ULL ? '1' : '0';
    }
    return text;
}

/// @brief One framed packet on the wire: sync word, header, payload. Returns the payload it wrote.
[[nodiscard]] std::vector<std::uint8_t> appendPacket(std::vector<std::uint8_t>& stream, const gr::digital::HeaderFormat& format, std::size_t payloadItems, std::uint64_t packetNumber) {
    for (std::size_t i = 0UZ; i < kSyncBits; ++i) {
        stream.push_back(static_cast<std::uint8_t>((kSyncWord >> (kSyncBits - 1UZ - i)) & 1ULL));
    }
    const std::size_t                                      headerItems = gr::digital::headerItemsOf(format);
    std::array<std::uint8_t, gr::digital::kMaxHeaderItems> header{};
    if (!gr::digital::formatHeader<std::uint8_t>(format, payloadItems, gr::property_map{{"packet_number", packetNumber}}, std::span<std::uint8_t>(header.data(), headerItems))) {
        throw gr::exception(std::format("the format refused a payload of {} items", payloadItems));
    }
    stream.insert(stream.end(), header.begin(), std::next(header.begin(), static_cast<std::ptrdiff_t>(headerItems)));

    std::vector<std::uint8_t> payload(payloadItems);
    for (std::size_t j = 0UZ; j < payloadItems; ++j) {
        payload[j] = static_cast<std::uint8_t>((j * 7UZ + payloadItems) & 1UZ);
    }
    stream.insert(stream.end(), payload.begin(), payload.end());
    return payload;
}

} // namespace

const boost::ut::suite<"DataSetToPacket under the scheduler"> schedulerTests = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::basic;

    // The whole chain end to end, which is where the declared widths have to agree in practice
    "the framing chain's metadata arrives with the widths a peer would read"_test = [] {
        constexpr std::size_t           kPackets = 6UZ;
        const gr::digital::HeaderFormat format{gr::digital::LengthCrcHeader{}};
        const std::size_t               headerItems = gr::digital::headerItemsOf(format);

        std::vector<std::uint8_t>              stream;
        std::vector<std::size_t>               starts;
        std::vector<std::vector<std::uint8_t>> payloads;
        for (std::size_t which = 0UZ; which < kPackets; ++which) {
            starts.push_back(stream.size() + kSyncBits + headerItems);
            payloads.push_back(appendPacket(stream, format, 8UZ + 16UZ * which, which));
        }

        Graph      graph;
        const auto values   = gr::Tensor<std::uint8_t>(stream.begin(), stream.end());
        auto&      source   = graph.emplaceBlock<gr::blocks::testing::TagSource<std::uint8_t, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(stream.size())}, {"values", values}, {"mark_tag", false}, {"verbose_console", false}});
        auto&      detector = graph.emplaceBlock<gr::blocks::digital::AccessCodeCorrelator<std::uint8_t>>({{"access_code", codeString(kSyncWord, kSyncBits)}});
        auto&      framer   = graph.emplaceBlock<gr::blocks::digital::PacketFramer<std::uint8_t>>({{"max_payload_items", gr::Size_t{4095U}}});
        auto&      appender = graph.emplaceBlock<gr::blocks::digital::CrcAppend>({{"width", gr::Size_t{32U}}});
        auto&      checker  = graph.emplaceBlock<gr::blocks::digital::CrcCheck>({{"width", gr::Size_t{32U}}, {"discard_crc", true}});
        auto&      convert  = graph.emplaceBlock<DataSetToPacket<std::uint8_t>>({{"protocol_label", std::string("test/frames")}, {"source_label", std::string("qa")}});
        auto&      sink     = graph.emplaceBlock<Collector<gr::Packet<std::uint8_t>>>();

        expect(graph.connect<"out", "in">(source, detector).has_value());
        expect(graph.connect<"out", "in">(detector, framer).has_value());
        expect(graph.connect<"out", "in">(framer, appender).has_value());
        expect(graph.connect<"out", "in">(appender, checker).has_value());
        expect(graph.connect<"ok", "in">(checker, convert).has_value());
        expect(graph.connect<"out", "in">(convert, sink).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        expect(eq(sink._items.size(), kPackets)) << "every framed packet reaches the terminal carrier";
        for (std::size_t which = 0UZ; which < std::min(sink._items.size(), kPackets); ++which) {
            const gr::Packet<std::uint8_t>& packet = sink._items[which];
            const std::string               label  = std::format("packet {}", which);
            expect(eq(packet.meta_information.size(), 1UZ)) << label;
            expect(std::ranges::equal(packet.signal_values, payloads[which])) << label << ": the payload bytes, exactly";

            const property_map meta = metaOf(packet);
            expect(read<bool>(meta, "crc_ok").value_or(false)) << label << ": crc_ok reads through bool";
            expect(eq(read<std::uint64_t>(meta, "sample_start").value_or(0ULL), static_cast<std::uint64_t>(starts[which]))) << label << ": sample_start needs uint64";
            expect(eq(read<gr::Size_t>(meta, "sync_errors").value_or(~gr::Size_t{0U}), gr::Size_t{0U})) << label << ": sync_errors needs gr::Size_t";
            expect(read<bool>(meta, "header_ok").value_or(false)) << label;
            expect(eq(readString(meta, "trigger_name").value_or("<missing>"), std::string("access_code"))) << label << ": the detector's own label, copied not invented";
            expect(eq(read<std::uint64_t>(meta, "packet_number").value_or(~0ULL), static_cast<std::uint64_t>(which))) << label << ": a value off the air keeps its protocol's name";
            expect(eq(readString(meta, "protocol").value_or(""), std::string("test/frames"))) << label;
            expect(eq(readString(meta, "source_id").value_or(""), std::string("qa"))) << label;
            expect(eq(readString(meta, "signal_name").value_or(""), std::string("payload"))) << label;
            expect(!hasKey(meta, "schema_version")) << label;
        }
        expect(eq(convert.nRejectedRecords, 0ULL));
        expect(eq(convert.nSignalsNotEmitted, 0ULL));
        expect(eq(convert.nMetaKeysDropped, 0ULL)) << "nothing the chain writes is a mistyped vocabulary key";
        expect(eq(sink._tags.size(), 0UZ)) << "and no tag arrives on the packet port";
    };

    // Runtime half: discard_reason rides a tag, so it survives a direct connection and one hop kills it
    "a rejection reason reaches a sink on reject and does not survive one ordinary block"_test = [] {
        const auto rejections = [](bool intervening) {
            std::vector<Record<std::uint8_t>> records;
            Record<std::uint8_t>              noSignals = makePacketRecord<std::uint8_t>(4UZ);
            noSignals.signal_names.clear();
            records.push_back(std::move(noSignals));
            records.push_back(makePacketRecord<std::uint8_t>(4UZ));

            Graph graph;
            auto& source    = graph.emplaceBlock<RecordSource<std::uint8_t>>();
            source._records = records;
            auto& convert   = graph.emplaceBlock<DataSetToPacket<std::uint8_t>>();
            auto& packets   = graph.emplaceBlock<Collector<gr::Packet<std::uint8_t>>>();
            auto& refused   = graph.emplaceBlock<Collector<gr::DataSet<std::uint8_t>>>();

            expect(graph.connect<"out", "in">(source, convert).has_value());
            expect(graph.connect<"out", "in">(convert, packets).has_value());
            if (intervening) {
                auto& hop = graph.emplaceBlock<gr::blocks::testing::Copy<gr::DataSet<std::uint8_t>>>();
                expect(graph.connect<"reject", "in">(convert, hop).has_value());
                expect(graph.connect<"out", "in">(hop, refused).has_value());
            } else {
                expect(graph.connect<"reject", "in">(convert, refused).has_value());
            }

            gr::scheduler::Simple scheduler;
            expect(scheduler.exchange(std::move(graph)).has_value());
            expect(scheduler.runAndWait().has_value());
            return std::pair<std::size_t, std::size_t>{refused._items.size(), offsetsOf(std::span<const Tag>(refused._tags), "discard_reason").size()};
        };

        const auto [directRecords, directReasons] = rejections(false);
        expect(eq(directRecords, 1UZ)) << "the refused record leaves by the reject port";
        expect(eq(directReasons, 1UZ)) << "with its reason on a tag beside it";

        const auto [hoppedRecords, hoppedReasons] = rejections(true);
        expect(eq(hoppedRecords, 1UZ)) << "the record itself survives the hop";
        expect(eq(hoppedReasons, 0UZ)) << "discard_reason is not a reserved key, so the default forwarder drops it";
    };
};

int main() { /* not needed for UT */ }
