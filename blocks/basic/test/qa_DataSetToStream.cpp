#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <numeric>
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

#include <gnuradio-4.0/algorithm/dataset/DataSetHelper.hpp>
#include <gnuradio-4.0/basic/DataSetToStream.hpp>
#include <gnuradio-4.0/basic/StreamToDataSet.hpp>
#include <gnuradio-4.0/testing/BuiltinTestBlocks.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

// The block is a carrier boundary: one bounded record in, a stream of one signal plus tags out. So the properties
// asserted below are boundary properties — what crosses, what is refused, what is counted, and what the output does
// not depend on. Chunk independence is the load-bearing one: every offset the block computes is a difference against
// a cursor it carries between calls, and a cursor that is not written back is right for a large output span and
// wrong for a small one.

namespace {

using gr::blocks::basic::DataSetToStream;

template<typename T>
using Record = gr::DataSet<T>;

// ─── a minimal three-port span harness ────────────────────────────────────────────────────────────────────────────
// The block publishes its own tags at offsets it computes from a cursor, so a test has to be able to fix the output
// chunk size and the absolute output position exactly. `tags()` yields the framework's own (relative index, map)
// pairs, including the negative index an unconsumed tag is presented at, which is what the guard below is for.

using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;
using gr::blocks::testing::span::TagView;

/// @brief Everything one run of the block published, with output tag offsets absolute in the output stream.
template<typename T>
struct Capture {
    std::vector<T>         samples{};
    std::vector<gr::Tag>   tags{};
    std::vector<Record<T>> rejected{};
    std::vector<gr::Tag>   rejectTags{};
    std::size_t            consumed = 0UZ;
    std::size_t            calls    = 0UZ;
    bool                   stalled  = false;
};

/**
 * @brief Drive @p block over @p records with an output span of @p outRoom samples.
 *
 * @p outRoom of 0 offers the whole output in one span. @p inTags carry an absolute *record* index, which is what a
 * `DataSet` port's tag index means. Consumed tags are deliberately left in view, so a record spanning several calls
 * sees its own tag again — at relative index 0 while it is the first unconsumed item, and at a negative one after.
 */
template<typename T>
[[nodiscard]] Capture<T> run(DataSetToStream<T>& block, std::span<const Record<T>> records, std::size_t outRoom = 0UZ, std::span<const gr::Tag> inTags = {}, bool rejectConnected = true) {
    Capture<T>  capture;
    std::size_t total = 0UZ;
    for (const Record<T>& record : records) {
        if (record.extents.size() == 1UZ && record.extents[0UZ] > 0) {
            total += static_cast<std::size_t>(record.extents[0UZ]);
        }
    }
    const std::size_t      room = outRoom == 0UZ ? std::max(total, 1UZ) : outRoom;
    std::vector<T>         scratch(room);
    std::vector<Record<T>> rejectScratch(records.size() + 1UZ);

    std::size_t consumed = 0UZ;
    while (consumed < records.size()) {
        InputSpan<Record<T>>  inSpan(records.subspan(consumed), consumed, inTags);
        OutputSpan<T>         outSpan(std::span<T>(scratch.data(), room), capture.samples.size(), &capture.tags);
        OutputSpan<Record<T>> rejectSpan(rejectConnected ? std::span<Record<T>>(rejectScratch) : std::span<Record<T>>{}, capture.rejected.size(), &capture.rejectTags, rejectConnected);

        const gr::work::Status status = block.processBulk(inSpan, outSpan, rejectSpan);
        ++capture.calls;

        capture.samples.insert(capture.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        for (std::size_t k = 0UZ; k < rejectSpan.count; ++k) {
            capture.rejected.push_back(std::move(rejectScratch[k]));
        }
        consumed += inSpan.consumed;
        capture.consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && rejectSpan.count == 0UZ) {
            capture.stalled = true; // no progress against a span it will be offered again: the graph would wedge here
            // a call that consumed nothing and published nothing has to say why, so that the scheduler can tell a
            // block waiting for room from one that has stopped
            boost::ut::expect(status != gr::work::Status::OK) << "no progress was made and the status says OK";
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

/// @brief `value` as a sample of type @p T, the complex case taking it as the real part; explicit at every step
/// so that -Wconversion has nothing to say about a test's own scaffolding.
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
[[nodiscard]] Record<T> makePacket(std::size_t nSamples) {
    Record<T> record;
    record.timestamp = 0;
    record.signal_values.resize(nSamples);
    for (std::size_t j = 0UZ; j < nSamples; ++j) {
        record.signal_values[j] = static_cast<T>(j);
    }
    record.extents.push_back(static_cast<std::int32_t>(nSamples));
    record.signal_names.emplace_back("payload");
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

/// @brief The tags published at exactly @p offset, in publication order.
[[nodiscard]] std::vector<gr::Tag> tagsAt(std::span<const gr::Tag> tags, std::size_t offset) {
    std::vector<gr::Tag> found;
    for (const gr::Tag& tag : tags) {
        if (tag.index == offset) {
            found.push_back(tag);
        }
    }
    return found;
}

/// @brief The first tag published at @p offset, or an empty one.
[[nodiscard]] gr::property_map mapAt(std::span<const gr::Tag> tags, std::size_t offset) {
    for (const gr::Tag& tag : tags) {
        if (tag.index == offset) {
            return tag.map;
        }
    }
    return {};
}

} // namespace

const boost::ut::suite<"DataSetToStream"> dataSetToStreamTests = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::basic;
    using namespace gr::blocks::testing;

    // The map covers the type, both what appears and what does not
    "every field of the record has one fate"_test = [] {
        Record<float> record = makeExtractedRecord<float>(8UZ, 48000.f);
        record.default_value = 42.f;
        record.timestamp     = 1'700'000'000'000'000'000LL;
        putMeta(record.meta_information[0UZ], "ctx", pmt::Value(std::string("FAIR.SELECTOR")));
        putMeta(record.meta_information[0UZ], "n_pre", pmt::Value(gr::Size_t{3U}));
        putMeta(record.meta_information[0UZ], "source_id", pmt::Value(std::string("dev0")));
        record.timing_events[0UZ].emplace_back(2, property_map{{"trigger_name", std::string("edge")}});
        record.timing_events[0UZ].emplace_back(7, property_map{{"marker", std::string("last")}});

        auto                block = make<DataSetToStream<float>>();
        const Record<float> input[]{record};
        const auto          capture = run<float>(block, input);

        expect(eq(capture.consumed, 1UZ));
        expect(eq(capture.samples.size(), 8UZ)) << "the selected signal's samples, and nothing else";
        expect(std::ranges::equal(capture.samples, std::vector<float>{0.f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f}));

        const property_map start = mapAt(capture.tags, 0UZ);
        expect(eq(read<gr::Size_t>(start, "dataset_length").value_or(0U), gr::Size_t{8U}));
        expect(eq(read<float>(start, "sample_rate").has_value(), true)) << "declared float";
        expect(eq(readString(start, "signal_name").value_or(""), std::string("signal0"))) << "declared string";
        expect(eq(readString(start, "signal_quantity").value_or(""), std::string("quantity0")));
        expect(eq(readString(start, "signal_unit").value_or(""), std::string("unit0")));
        expect(eq(read<float>(start, "signal_min").value_or(-1.f), 0.f)) << "declared float";
        expect(eq(read<float>(start, "signal_max").value_or(-1.f), 1.f));
        expect(eq(read<std::uint64_t>(start, "trigger_time").value_or(0ULL), 1'700'000'000'000'000'000ULL)) << "declared uint64";
        expect(eq(readString(start, "trigger_name").value_or(""), std::string("dataset")));
        expect(eq(readString(start, "ctx").value_or(""), std::string("FAIR.SELECTOR"))) << "meta_information copied verbatim";
        expect(eq(read<gr::Size_t>(start, "n_pre").value_or(99U), gr::Size_t{3U}));
        expect(eq(readString(start, "source_id").value_or(""), std::string("dev0")));

        expect(that % (offsetsOf(capture.tags, "marker") == std::vector<std::size_t>{7UZ})) << "every in-range timing event";
        expect(that % (offsetsOf(capture.tags, "trigger_name") == std::vector<std::size_t>{0UZ, 2UZ}));

        for (const Tag& tag : capture.tags) { // the absences, which a coverage test usually omits
            expect(!hasKey(tag.map, "default_value"));
            expect(!hasKey(tag.map, "layout"));
            expect(!hasKey(tag.map, "axis_values"));
        }
        expect(std::ranges::none_of(capture.samples, [](float value) { return value == 42.f; })) << "the padding value never reaches the stream";
    };

    "a multi-signal record emits only the selected signal"_test = [] {
        const Record<float> input[]{makeRecord<float>(4UZ, 4UZ)};
        for (gr::Size_t index = 0U; index < 4U; ++index) {
            auto       block   = make<DataSetToStream<float>>({{"signal_index", index}});
            const auto capture = run<float>(block, input);
            expect(eq(capture.samples.size(), 4UZ));
            expect(eq(capture.samples.front(), static_cast<float>(1000U * index)));
            expect(eq(block.nSignalsNotEmitted, 3ULL));
            expect(eq(readString(mapAt(capture.tags, 0UZ), "signal_name").value_or(""), std::format("signal{}", index)));
        }
    };

    // The block's predicate and the framework's validator agree on the packet it builds
    "a packet is admitted and passes checkConsistency"_test = [] {
        const Record<std::uint8_t> packet = makePacket<std::uint8_t>(17UZ);
        expect(gr::dataset::checkConsistency(packet).has_value()) << "an axis-free record validates: a payload index is not a physical axis";

        auto                       block = make<DataSetToStream<std::uint8_t>>();
        const Record<std::uint8_t> input[]{packet};
        const auto                 capture = run<std::uint8_t>(block, input);
        expect(eq(capture.samples.size(), 17UZ)) << "and yet every one of its samples reaches the stream";
        expect(eq(capture.rejected.size(), 0UZ));
        expect(eq(block.nRejectedRecords, 0ULL));

        const Record<float> extracted = makeExtractedRecord<float>(32UZ, 8000.f);
        expect(gr::dataset::checkConsistency(extracted).has_value()) << "a StreamToDataSet record satisfies both predicates";
        auto                floatBlock = make<DataSetToStream<float>>();
        const Record<float> extractedInput[]{extracted};
        const auto          floatCapture = run<float>(floatBlock, extractedInput);
        expect(eq(floatCapture.samples.size(), 32UZ));
    };

    // A record cannot stop the graph, and a missing annotation is not a malformed record
    "eleven malformed records are refused without throwing"_test = [] {
        // every record carries two signals and the block reads the second, so a record with one signal is the one
        // whose second signal is missing while the others are malformed in another way
        std::vector<Record<float>> records;
        Record<float>              noSignals = makeRecord<float>(4UZ, 2UZ);
        noSignals.signal_names.clear();
        records.push_back(std::move(noSignals));
        records.push_back(makeRecord<float>(4UZ, 1UZ));
        Record<float> noExtent = makeRecord<float>(4UZ, 2UZ);
        noExtent.extents.clear();
        records.push_back(std::move(noExtent));
        Record<float> twoExtents = makeRecord<float>(4UZ, 2UZ);
        twoExtents.extents.push_back(2);
        records.push_back(std::move(twoExtents));
        Record<float> zeroExtent = makeRecord<float>(4UZ, 2UZ);
        zeroExtent.extents[0UZ]  = 0;
        records.push_back(std::move(zeroExtent));
        Record<float> negativeExtent = makeRecord<float>(4UZ, 2UZ);
        negativeExtent.extents[0UZ]  = -4;
        records.push_back(std::move(negativeExtent));
        Record<float> shortValues = makeRecord<float>(4UZ, 2UZ);
        shortValues.signal_values.pop_back();
        records.push_back(std::move(shortValues));
        Record<float> longValues = makeRecord<float>(4UZ, 2UZ);
        longValues.signal_values.push_back(0.f);
        records.push_back(std::move(longValues));
        Record<float> shortUnits = makeRecord<float>(4UZ, 2UZ);
        shortUnits.signal_units.clear();
        records.push_back(std::move(shortUnits)); // admitted: an absent annotation is not a malformed record
        Record<float> noMeta = makeRecord<float>(4UZ, 2UZ);
        noMeta.meta_information.clear();
        records.push_back(std::move(noMeta)); // admitted
        Record<float> noEvents = makeRecord<float>(4UZ, 2UZ);
        noEvents.timing_events.clear();
        records.push_back(std::move(noEvents)); // admitted

        auto           block = make<DataSetToStream<float>>({{"signal_index", gr::Size_t{1U}}});
        Capture<float> capture;
        expect(nothrow([&] { capture = run<float>(block, std::span<const Record<float>>(records)); })) << "no accessor on the sample path throws";
        expect(!capture.stalled) << "a malformed record does not stop the block making progress";
        expect(eq(capture.consumed, records.size()));
        expect(eq(block.nRejectedRecords, 8ULL));
        expect(eq(capture.rejected.size(), 8UZ));
        expect(eq(capture.samples.size(), 12UZ)) << "the last three records are admitted, at four samples each";
        expect(eq(block.nSignalsNotEmitted, 3ULL)) << "one unemitted signal per admitted record";

        const std::vector<std::string> expected{"no_signals", "signal_index_out_of_range", "not_one_dimensional", "not_one_dimensional", "empty_or_negative_extent", "empty_or_negative_extent", "inconsistent_extent", "inconsistent_extent"};
        std::vector<std::string>       reasons;
        for (const Tag& tag : capture.rejectTags) {
            reasons.push_back(readString(tag.map, "discard_reason").value_or("<missing>"));
        }
        expect(std::ranges::equal(reasons, expected)) << "each rejection names the first clause that failed";
        expect(eq(capture.rejected.at(1UZ).signal_names.size(), 1UZ)) << "the record travels unchanged";
    };

    // A record larger than the output buffer, tested by not wedging
    "a million-sample record survives an output span of one"_test = [] {
        constexpr std::size_t      kLength = 1'000'000UZ;
        std::vector<Record<float>> records{makeRecord<float>(kLength)};
        for (const std::size_t room : {1024UZ, 1UZ}) {
            auto       block   = make<DataSetToStream<float>>();
            const auto capture = run<float>(block, std::span<const Record<float>>(records), room);
            expect(!capture.stalled) << std::format("room {}: the block always makes progress", room);
            expect(eq(capture.samples.size(), kLength)) << std::format("room {}", room);
            expect(eq(capture.consumed, 1UZ)) << std::format("room {}: the input item is consumed exactly once", room);
            expect(that % (offsetsOf(capture.tags, "dataset_length") == std::vector<std::size_t>{0UZ})) << std::format("room {}: one record-start tag, at offset 0", room);
            bool ordered = true;
            for (std::size_t j = 0UZ; j < kLength; ++j) {
                ordered = ordered && capture.samples[j] == static_cast<float>(j);
            }
            expect(ordered) << std::format("room {}: the samples arrive in order", room);
        }
    };

    // Chunk independence, bit-identical
    "the output does not depend on the output chunk size"_test = [] {
        std::vector<Record<float>> records;
        const std::size_t          lengths[]{1UZ, 5UZ, 64UZ, 3UZ, 1UZ, 1'048'577UZ, 17UZ, 2UZ, 100UZ, 7UZ, 33UZ, 4096UZ, 9UZ, 11UZ, 1UZ, 256UZ, 8UZ, 13UZ, 21UZ, 34UZ};
        for (std::size_t k = 0UZ; k < std::size(lengths); ++k) {
            Record<float> record = makeRecord<float>(lengths[k]);
            putMeta(record.meta_information[0UZ], "sequence", pmt::Value(static_cast<gr::Size_t>(k)));
            record.timing_events[0UZ].emplace_back(0, property_map{{"event", std::string("first")}});
            record.timing_events[0UZ].emplace_back(static_cast<std::ptrdiff_t>(lengths[k] - 1UZ), property_map{{"event", std::string("last")}});
            records.push_back(std::move(record));
        }

        auto       reference        = make<DataSetToStream<float>>();
        const auto single           = run<float>(reference, std::span<const Record<float>>(records));
        const auto referenceOffsets = offsetsOf(single.tags, "event");
        for (const std::size_t room : {1UZ, 3UZ, 17UZ, 4096UZ, 1'048'576UZ}) {
            auto       block   = make<DataSetToStream<float>>();
            const auto capture = run<float>(block, std::span<const Record<float>>(records), room);
            expect(eq(capture.samples.size(), single.samples.size())) << std::format("room {}", room);
            expect(std::ranges::equal(capture.samples, single.samples)) << std::format("room {}: samples", room);
            expect(eq(capture.tags.size(), single.tags.size())) << std::format("room {}: tag multiplicity", room);
            expect(that % (offsetsOf(capture.tags, "event") == referenceOffsets)) << std::format("room {}: event offsets", room);
            expect(that % (offsetsOf(capture.tags, "dataset_length") == offsetsOf(single.tags, "dataset_length"))) << std::format("room {}: record starts", room);
        }
    };

    // Tags in, tags out, exactly once
    "an input tag lands on its record's start tag and nowhere else"_test = [] {
        std::vector<Record<float>> records;
        for (std::size_t k = 0UZ; k < 9UZ; ++k) {
            records.push_back(makeRecord<float>(5UZ));
        }
        for (const std::size_t at : {0UZ, 1UZ, 7UZ}) {
            const std::vector<Tag> inTags{Tag{at, property_map{{"sample_rate", 96000.f}, {"probe", std::string("here")}}}};
            auto                   block   = make<DataSetToStream<float>>();
            const auto             capture = run<float>(block, std::span<const Record<float>>(records), 1UZ, std::span<const Tag>(inTags));
            expect(that % (offsetsOf(capture.tags, "probe") == std::vector<std::size_t>{at * 5UZ})) << std::format("record {}: multiplicity 1, at that record's first sample", at);
            expect(eq(capture.samples.size(), 45UZ)) << std::format("record {}", at);
        }
    };

    // The rate's four routes, in order
    "the rate routes are tried in order"_test = [] {
        { // (a) route 1 beats route 3
            Record<float> record = makeExtractedRecord<float>(1024UZ, 1e6f);
            putMeta(record.meta_information[0UZ], "sample_rate", pmt::Value(48000.f));
            auto                block = make<DataSetToStream<float>>();
            const Record<float> input[]{record};
            const auto          capture = run<float>(block, input);
            expect(eq(read<float>(mapAt(capture.tags, 0UZ), "sample_rate").value_or(0.f), 48000.f));
        }
        { // (b) route 2 beats route 3
            const Record<float>    input[]{makeExtractedRecord<float>(1024UZ, 1e6f)};
            const std::vector<Tag> inTags{Tag{0UZ, property_map{{"sample_rate", 96000.f}}}};
            auto                   block   = make<DataSetToStream<float>>();
            const auto             capture = run<float>(block, input, 0UZ, std::span<const Tag>(inTags));
            expect(eq(read<float>(mapAt(capture.tags, 0UZ), "sample_rate").value_or(0.f), 96000.f));
        }
        { // (c) route 3, to within the endpoint form's measured error
            for (const std::size_t length : {1024UZ, 65536UZ}) {
                const Record<float> input[]{makeExtractedRecord<float>(length, 48000.f)};
                auto                block   = make<DataSetToStream<float>>();
                const auto          capture = run<float>(block, input);
                const auto          rate    = read<float>(mapAt(capture.tags, 0UZ), "sample_rate");
                expect(rate.has_value()) << std::format("length {}", length);
                expect(lt(std::abs(static_cast<double>(*rate) - 48000.0) / 48000.0, 3.2e-08)) << std::format("length {}: the endpoint form is within a float ulp", length);
                expect(eq(block.nRateLessRecords, 0ULL));
            }
        }
        { // (d) a byte record has no readable axis, so there is no rate to announce
            const Record<std::uint8_t> input[]{makePacket<std::uint8_t>(64UZ)};
            auto                       block   = make<DataSetToStream<std::uint8_t>>();
            const auto                 capture = run<std::uint8_t>(block, input);
            expect(!hasKey(mapAt(capture.tags, 0UZ), "sample_rate"));
            expect(eq(block.nRateLessRecords, 1ULL));
        }
    };

    // The axis is read O(1) and refuses what it should
    "the axis route refuses a non-uniform axis and reads three elements"_test = [] {
        { // a log-spaced axis, and a jittered one
            Record<float> logAxis = makeExtractedRecord<float>(1024UZ, 48000.f);
            for (std::size_t j = 0UZ; j < 1024UZ; ++j) {
                logAxis.axis_values[0UZ][j] = static_cast<float>(std::pow(10.0, -6.0 + 6.0 * static_cast<double>(j) / 1023.0));
            }
            Record<float> jittered = makeExtractedRecord<float>(1024UZ, 48000.f);
            jittered.axis_values[0UZ][1UZ] *= 1.01f;

            for (const Record<float>& record : {logAxis, jittered}) {
                auto                block = make<DataSetToStream<float>>();
                const Record<float> input[]{record};
                const auto          capture = run<float>(block, input);
                expect(!hasKey(mapAt(capture.tags, 0UZ), "sample_rate")) << "a non-uniform axis is not a constant rate and is not announced as one";
                expect(eq(block.nRateLessRecords, 1ULL));
            }
        }
        { // a constant axis has no positive step
            Record<float> flat = makeExtractedRecord<float>(64UZ, 48000.f);
            std::ranges::fill(flat.axis_values[0UZ], 0.f);
            auto                block = make<DataSetToStream<float>>();
            const Record<float> input[]{flat};
            const auto          capture = run<float>(block, input);
            expect(!hasKey(mapAt(capture.tags, 0UZ), "sample_rate"));
        }
        { // a one-sample record's axis is never read
            Record<float>       single = makeExtractedRecord<float>(1UZ, 48000.f);
            auto                block  = make<DataSetToStream<float>>();
            const Record<float> input[]{single};
            const auto          capture = run<float>(block, input);
            expect(!hasKey(mapAt(capture.tags, 0UZ), "sample_rate"));
            expect(eq(capture.samples.size(), 1UZ));
        }
        { // every axis element but the first, the second and the last is a NaN: a rate comes back only if none was read
            constexpr std::size_t kLength  = 1'000'000UZ;
            Record<float>         poisoned = makeExtractedRecord<float>(kLength, 48000.f);
            const float           first    = poisoned.axis_values[0UZ][0UZ];
            const float           second   = poisoned.axis_values[0UZ][1UZ];
            const float           last     = poisoned.axis_values[0UZ][kLength - 1UZ];
            std::ranges::fill(poisoned.axis_values[0UZ], std::numeric_limits<float>::quiet_NaN());
            poisoned.axis_values[0UZ][0UZ]           = first;
            poisoned.axis_values[0UZ][1UZ]           = second;
            poisoned.axis_values[0UZ][kLength - 1UZ] = last;

            auto                block = make<DataSetToStream<float>>();
            const Record<float> input[]{std::move(poisoned)};
            const auto          capture = run<float>(block, input, 1UZ << 20U);
            const auto          rate    = read<float>(mapAt(capture.tags, 0UZ), "sample_rate");
            expect(rate.has_value()) << "the axis is read at three positions, not scanned";
            expect(lt(std::abs(static_cast<double>(rate.value_or(0.f)) - 48000.0) / 48000.0, 1e-06));
        }
    };

    // Multi-signal fan-out is a composition, and it is sample-aligned
    "four instances on one record port produce four aligned streams"_test = [] {
        std::vector<Record<float>>  records{makeRecord<float>(6UZ, 4UZ), makeRecord<float>(11UZ, 4UZ)};
        std::vector<Capture<float>> captures;
        for (gr::Size_t index = 0U; index < 4U; ++index) {
            auto block = make<DataSetToStream<float>>({{"signal_index", index}});
            captures.push_back(run<float>(block, std::span<const Record<float>>(records)));
            expect(eq(block.nSignalsNotEmitted, 6ULL)) << std::format("signal_index {}: three per record, two records", index);
        }
        for (gr::Size_t index = 0U; index < 4U; ++index) {
            expect(eq(captures[index].samples.size(), 17UZ));
            expect(eq(captures[index].samples.front(), static_cast<float>(1000U * index)));
            expect(that % (offsetsOf(captures[index].tags, "dataset_length") == std::vector<std::size_t>{0UZ, 6UZ})) << "the record starts land at the same offset on every stream";
        }

        auto                narrow = make<DataSetToStream<float>>({{"signal_index", gr::Size_t{4U}}});
        const Record<float> input[]{makeRecord<float>(6UZ, 4UZ)};
        const auto          rejected = run<float>(narrow, input);
        expect(eq(rejected.rejected.size(), 1UZ));
        expect(eq(readString(rejected.rejectTags.at(0UZ).map, "discard_reason").value_or(""), std::string("signal_index_out_of_range")));
    };

    // Discontinuity, all four causes and the half that matters
    "a discontinuity names its causes and an unchanged record names none"_test = [] {
        const auto withStart = [](Record<float> record, std::uint64_t start) {
            putMeta(record.meta_information[0UZ], "sample_start", pmt::Value(start));
            return record;
        };

        { // rate
            Record<float> first  = makeRecord<float>(4UZ);
            Record<float> second = makeRecord<float>(4UZ);
            putMeta(first.meta_information[0UZ], "sample_rate", pmt::Value(48000.f));
            putMeta(second.meta_information[0UZ], "sample_rate", pmt::Value(96000.f));
            const Record<float> input[]{first, second};
            auto                block   = make<DataSetToStream<float>>();
            const auto          capture = run<float>(block, input);
            expect(eq(readString(mapAt(capture.tags, 4UZ), "discontinuity").value_or(""), std::string("sample_rate")));
            expect(!hasKey(mapAt(capture.tags, 4UZ), "n_dropped_samples"));
        }
        { // unit
            Record<float> first      = makeRecord<float>(4UZ);
            Record<float> second     = makeRecord<float>(4UZ);
            second.signal_units[0UZ] = "V";
            const Record<float> input[]{first, second};
            auto                block   = make<DataSetToStream<float>>();
            const auto          capture = run<float>(block, input);
            expect(eq(readString(mapAt(capture.tags, 4UZ), "discontinuity").value_or(""), std::string("signal_unit")));
            expect(eq(readString(mapAt(capture.tags, 4UZ), "signal_unit").value_or(""), std::string("V")));
        }
        { // range
            Record<float> first           = makeRecord<float>(4UZ);
            Record<float> second          = makeRecord<float>(4UZ);
            second.signal_ranges[0UZ].max = 7.f;
            const Record<float> input[]{first, second};
            auto                block   = make<DataSetToStream<float>>();
            const auto          capture = run<float>(block, input);
            expect(eq(readString(mapAt(capture.tags, 4UZ), "discontinuity").value_or(""), std::string("signal_range")));
            expect(eq(read<float>(mapAt(capture.tags, 4UZ), "signal_max").value_or(0.f), 7.f));
        }
        { // a positional gap of 1000 samples
            const Record<float> input[]{withStart(makeRecord<float>(4UZ), 0ULL), withStart(makeRecord<float>(4UZ), 1004ULL)};
            auto                block   = make<DataSetToStream<float>>();
            const auto          capture = run<float>(block, input);
            expect(eq(readString(mapAt(capture.tags, 4UZ), "discontinuity").value_or(""), std::string("gap")));
            expect(eq(read<gr::Size_t>(mapAt(capture.tags, 4UZ), "n_dropped_samples").value_or(0U), gr::Size_t{1000U}));
        }
        { // overlapping windows: the same samples twice is not a dropped-sample count
            const Record<float> input[]{withStart(makeRecord<float>(4UZ), 100ULL), withStart(makeRecord<float>(4UZ), 102ULL)};
            auto                block   = make<DataSetToStream<float>>();
            const auto          capture = run<float>(block, input);
            expect(eq(readString(mapAt(capture.tags, 4UZ), "discontinuity").value_or(""), std::string("gap")));
            expect(!hasKey(mapAt(capture.tags, 4UZ), "n_dropped_samples"));
        }
        { // every cause at once, in the table's order
            Record<float> first  = makeRecord<float>(4UZ);
            Record<float> second = makeRecord<float>(4UZ);
            putMeta(first.meta_information[0UZ], "sample_rate", pmt::Value(48000.f));
            putMeta(second.meta_information[0UZ], "sample_rate", pmt::Value(96000.f));
            second.signal_names[0UZ]      = "other";
            second.signal_quantities[0UZ] = "current";
            second.signal_units[0UZ]      = "A";
            second.signal_ranges[0UZ].min = -5.f;
            const Record<float> input[]{withStart(first, 0ULL), withStart(second, 1004ULL)};
            auto                block   = make<DataSetToStream<float>>();
            const auto          capture = run<float>(block, input);
            expect(eq(readString(mapAt(capture.tags, 4UZ), "discontinuity").value_or(""), std::string("sample_rate,signal_name,signal_quantity,signal_unit,signal_range,gap")));
        }
        { // two identical records: the boundary is still marked, and nothing else is repeated
            Record<float> record = makeRecord<float>(4UZ);
            putMeta(record.meta_information[0UZ], "sample_rate", pmt::Value(48000.f));
            const Record<float> input[]{record, record};
            auto                block   = make<DataSetToStream<float>>();
            const auto          capture = run<float>(block, input);
            const property_map  second  = mapAt(capture.tags, 4UZ);
            expect(eq(read<gr::Size_t>(second, "dataset_length").value_or(0U), gr::Size_t{4U})) << "the record-start tag is authored for every record";
            expect(!hasKey(second, "discontinuity"));
            expect(!hasKey(second, "signal_name"));
            expect(!hasKey(second, "signal_unit"));
            expect(hasKey(second, "sample_rate")) << "the record's own metadata is copied verbatim at every record";
            expect(eq(block.nMetaKeysOverridden, 0ULL)) << "an unannounced rate does not displace the copied one";
            expect(!hasKey(mapAt(capture.tags, 0UZ), "discontinuity")) << "a stream's own beginning is not a discontinuity";
        }
    };

    // Nothing is synthesized
    "a gap is reported and never filled"_test = [] {
        Record<float> first  = makeRecord<float>(37UZ);
        Record<float> second = makeRecord<float>(53UZ);
        putMeta(first.meta_information[0UZ], "sample_start", pmt::Value(std::uint64_t{0ULL}));
        putMeta(second.meta_information[0UZ], "sample_start", pmt::Value(std::uint64_t{1037ULL}));
        const Record<float> input[]{first, second};
        auto                block   = make<DataSetToStream<float>>();
        const auto          capture = run<float>(block, input);
        expect(eq(capture.samples.size(), 90UZ)) << "exactly the two records, and not one sample more";
        expect(eq(capture.samples[36UZ], 36.f));
        expect(eq(capture.samples[37UZ], 0.f)) << "record two begins immediately after record one";
    };

    // Timing events
    "timing events become tags at translated offsets"_test = [] {
        { // (a) in-range events land exactly, and (b) out-of-range ones are dropped rather than clamped
            Record<float> record = makeRecord<float>(16UZ);
            record.timing_events[0UZ].emplace_back(-1, property_map{{"before", std::string("x")}});
            record.timing_events[0UZ].emplace_back(0, property_map{{"at", std::string("first")}});
            record.timing_events[0UZ].emplace_back(1, property_map{{"at", std::string("second")}});
            record.timing_events[0UZ].emplace_back(15, property_map{{"at", std::string("last")}});
            record.timing_events[0UZ].emplace_back(16, property_map{{"after", std::string("y")}});
            const Record<float> input[]{record};
            for (const std::size_t room : {0UZ, 1UZ, 5UZ}) {
                auto       block   = make<DataSetToStream<float>>();
                const auto capture = run<float>(block, input, room);
                expect(that % (offsetsOf(capture.tags, "at") == std::vector<std::size_t>{0UZ, 1UZ, 15UZ})) << std::format("room {}", room);
                expect(that % (offsetsOf(capture.tags, "before") == std::vector<std::size_t>{})) << std::format("room {}: never clamped to 0", room);
                expect(that % (offsetsOf(capture.tags, "after") == std::vector<std::size_t>{})) << std::format("room {}: never clamped to the last sample", room);
                expect(eq(block.nDroppedTimingEvents, 2ULL)) << std::format("room {}", room);
            }
        }
        { // (c) three events at one index are three tags, and (d) an event at 0 does not merge with the record start
            Record<float> record = makeRecord<float>(4UZ);
            for (const char* label : {"a", "b", "c"}) {
                record.timing_events[0UZ].emplace_back(0, property_map{{"event", std::string(label)}});
            }
            const Record<float> input[]{record};
            auto                block   = make<DataSetToStream<float>>();
            const auto          capture = run<float>(block, input);
            expect(that % (offsetsOf(capture.tags, "event") == std::vector<std::size_t>{0UZ, 0UZ, 0UZ})) << "three annotations stay three";
            const std::vector<Tag> atZero = tagsAt(capture.tags, 0UZ);
            expect(eq(atZero.size(), 4UZ)) << "the record-start tag plus three events";
            expect(hasKey(atZero.front().map, "dataset_length")) << "and the record start is published first";
            expect(eq(readString(atZero.at(1UZ).map, "event").value_or(""), std::string("a")));
        }
        { // (e) an unsorted event list produces the sorted permutation's output
            Record<float>        sorted   = makeRecord<float>(8UZ);
            Record<float>        unsorted = makeRecord<float>(8UZ);
            const std::ptrdiff_t indices[]{5, 1, 7, 0, 3};
            for (const std::ptrdiff_t index : indices) {
                unsorted.timing_events[0UZ].emplace_back(index, property_map{{"event", static_cast<gr::Size_t>(index)}});
            }
            std::vector<std::ptrdiff_t> ascending(std::begin(indices), std::end(indices));
            std::ranges::sort(ascending);
            for (const std::ptrdiff_t index : ascending) {
                sorted.timing_events[0UZ].emplace_back(index, property_map{{"event", static_cast<gr::Size_t>(index)}});
            }
            auto                sortedBlock   = make<DataSetToStream<float>>();
            auto                unsortedBlock = make<DataSetToStream<float>>();
            const Record<float> sortedInput[]{sorted};
            const Record<float> unsortedInput[]{unsorted};
            const auto          sortedCapture   = run<float>(sortedBlock, sortedInput);
            const auto          unsortedCapture = run<float>(unsortedBlock, unsortedInput);
            expect(that % (offsetsOf(unsortedCapture.tags, "event") == offsetsOf(sortedCapture.tags, "event")));
            expect(that % (offsetsOf(unsortedCapture.tags, "event") == std::vector<std::size_t>{0UZ, 1UZ, 3UZ, 5UZ, 7UZ}));
        }
    };

    // Metadata collision rules
    "a derived key wins, and a mistyped reserved key is dropped"_test = [] {
        { // (a) a copied signal_unit differing from the record's own
            Record<float> record = makeRecord<float>(4UZ);
            putMeta(record.meta_information[0UZ], "signal_unit", pmt::Value(std::string("wrong")));
            const Record<float> input[]{record};
            auto                block   = make<DataSetToStream<float>>();
            const auto          capture = run<float>(block, input);
            expect(eq(readString(mapAt(capture.tags, 0UZ), "signal_unit").value_or(""), std::string("unit0")));
            expect(eq(block.nMetaKeysOverridden, 1ULL));
        }
        { // (b) a sample_rate carried as a double is not a slightly wrong rate
            Record<float> record = makeRecord<float>(4UZ);
            putMeta(record.meta_information[0UZ], "sample_rate", pmt::Value(48000.0));
            const Record<float> input[]{record};
            auto                block   = make<DataSetToStream<float>>();
            const auto          capture = run<float>(block, input);
            expect(!hasKey(mapAt(capture.tags, 0UZ), "sample_rate"));
            expect(eq(block.nMetaKeysDropped, 1ULL));
            expect(eq(block.nRateLessRecords, 1ULL));
        }
        { // (c) the same key as the declared float is route 1 and is emitted
            Record<float> record = makeRecord<float>(4UZ);
            putMeta(record.meta_information[0UZ], "sample_rate", pmt::Value(48000.f));
            const Record<float> input[]{record};
            auto                block   = make<DataSetToStream<float>>();
            const auto          capture = run<float>(block, input);
            expect(eq(read<float>(mapAt(capture.tags, 0UZ), "sample_rate").value_or(0.f), 48000.f));
            expect(eq(block.nMetaKeysDropped, 0ULL));
        }
        { // (d) a non-reserved key of any type travels unexamined
            Record<float> record = makeRecord<float>(4UZ);
            putMeta(record.meta_information[0UZ], "schema_version", pmt::Value(std::uint64_t{7ULL}));
            putMeta(record.meta_information[0UZ], "crc_ok", pmt::Value(true));
            const Record<float> input[]{record};
            auto                block   = make<DataSetToStream<float>>();
            const auto          capture = run<float>(block, input);
            expect(eq(read<std::uint64_t>(mapAt(capture.tags, 0UZ), "schema_version").value_or(0ULL), 7ULL));
            expect(eq(read<bool>(mapAt(capture.tags, 0UZ), "crc_ok").value_or(false), true));
            expect(eq(block.nMetaKeysDropped, 0ULL));
        }
        { // the record's own trigger_name is preferred, so a framing round trip gets its detector's label back
            Record<float> record = makeRecord<float>(4UZ);
            putMeta(record.meta_information[0UZ], "trigger_name", pmt::Value(std::string("access_code")));
            const Record<float> input[]{record};
            auto                block   = make<DataSetToStream<float>>({{"boundary_label", std::string("dataset")}});
            const auto          capture = run<float>(block, input);
            expect(eq(readString(mapAt(capture.tags, 0UZ), "trigger_name").value_or(""), std::string("access_code")));
            expect(eq(block.nMetaKeysOverridden, 0ULL)) << "echoing the record's own value displaces nothing";
        }
        { // an empty boundary_label with no record trigger suppresses the key, never the tag
            auto                block = make<DataSetToStream<float>>({{"boundary_label", std::string("")}});
            const Record<float> input[]{makeRecord<float>(4UZ)};
            const auto          capture = run<float>(block, input);
            expect(!hasKey(mapAt(capture.tags, 0UZ), "trigger_name"));
            expect(hasKey(mapAt(capture.tags, 0UZ), "dataset_length")) << "the boundary is still marked";
        }
    };

    // The reject port unconnected, which is the framework question this block leans on
    "an unconnected Optional Async output port takes a span, a publish and a tag"_test = [] {
        DataSetToStream<float> block; // never connected to anything
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
        records.push_back(makeRecord<float>(4UZ));
        Record<float> zeroExtent = makeRecord<float>(4UZ);
        zeroExtent.extents[0UZ]  = 0;
        records.push_back(zeroExtent);

        auto           connected = make<DataSetToStream<float>>();
        const auto     withPort  = run<float>(connected, std::span<const Record<float>>(records));
        auto           orphaned  = make<DataSetToStream<float>>();
        Capture<float> withoutPort;
        expect(nothrow([&] { withoutPort = run<float>(orphaned, std::span<const Record<float>>(records), 0UZ, {}, false); }));
        expect(eq(withoutPort.samples.size(), withPort.samples.size()));
        expect(std::ranges::equal(withoutPort.samples, withPort.samples));
        expect(eq(orphaned.nRejectedRecords, connected.nRejectedRecords));
        expect(eq(orphaned.nRejectedRecords, 2ULL));
        expect(eq(withoutPort.rejected.size(), 0UZ)) << "nothing is written to a port nobody reads";
        expect(eq(withoutPort.consumed, records.size())) << "and the records are still consumed";
    };

    // Validation rejects, and signal_index does not
    "an unusable tolerance is refused and the working one survives"_test = [] {
        for (const float bad : {0.f, -1.f, std::numeric_limits<float>::quiet_NaN()}) {
            auto block = make<DataSetToStream<float>>({{"axis_rate_tolerance", 1e-4f}});
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
        expect(nothrow([] { std::ignore = make<DataSetToStream<float>>({{"signal_index", std::numeric_limits<gr::Size_t>::max()}}); })) << "signal_index is a property of each record, not of the setting";
        auto                block = make<DataSetToStream<float>>({{"signal_index", std::numeric_limits<gr::Size_t>::max()}});
        const Record<float> input[]{makeRecord<float>(4UZ)};
        const auto          capture = run<float>(block, input);
        expect(eq(capture.rejected.size(), 1UZ));
        expect(eq(readString(capture.rejectTags.at(0UZ).map, "discard_reason").value_or(""), std::string("signal_index_out_of_range")));
    };

    // The compile-time half
    "the block is not admissible for UnfilteredTagPropagation"_test = [] {
        static_assert(!gr::block::kUnfilteredTagPropagationAdmissible<DataSetToStream<float>>, "asynchronous ports and NoTagPropagation each refuse the flag on their own");
        static_assert(!gr::block::kUnfilteredTagPropagationAdmissible<DataSetToStream<std::complex<float>>>);
        expect(!gr::block::kUnfilteredTagPropagationAdmissible<DataSetToStream<float>>);
    };

    "every registered type converts"_test = [] {
        const auto check = []<typename T>() {
            auto            block = make<DataSetToStream<T>>();
            const Record<T> input[]{makeRecord<T>(5UZ)};
            const auto      capture = run<T>(block, input);
            expect(eq(capture.samples.size(), 5UZ)) << gr::meta::type_name<T>();
            expect(eq(capture.samples.back(), static_cast<T>(4))) << gr::meta::type_name<T>();
            expect(eq(read<gr::Size_t>(mapAt(capture.tags, 0UZ), "dataset_length").value_or(0U), gr::Size_t{5U})) << gr::meta::type_name<T>();
        };
        check.template operator()<std::uint8_t>();
        check.template operator()<std::int16_t>();
        check.template operator()<std::int32_t>();
        check.template operator()<float>();
        check.template operator()<std::complex<float>>();

        { // a complex signal_range holds two samples ordered by magnitude, so it is not a pair of float limits
            auto                              block = make<DataSetToStream<std::complex<float>>>();
            const Record<std::complex<float>> input[]{makeRecord<std::complex<float>>(5UZ)};
            const auto                        capture = run<std::complex<float>>(block, input);
            expect(!hasKey(mapAt(capture.tags, 0UZ), "signal_min"));
            expect(!hasKey(mapAt(capture.tags, 0UZ), "signal_max"));
        }
    };
};

// ─── under the scheduler: the boundary in a real graph, with a buffer narrower than a record ──────────────────────

namespace {

/// @brief Emits its items one output span at a time, publishing each staged tag with the item it belongs to.
template<typename TItem>
struct StagedSource : gr::Block<StagedSource<TItem>> {
    gr::PortOut<TItem> out;
    GR_MAKE_REFLECTABLE(StagedSource, out);

    std::vector<TItem>   _items{};
    std::vector<gr::Tag> _tags{}; ///< index is an absolute item index, ascending
    std::size_t          _emitted = 0UZ;

    gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t nItems = std::min(outSpan.size(), _items.size() - _emitted);
        for (std::size_t k = 0UZ; k < nItems; ++k) {
            for (const gr::Tag& tag : _tags) {
                if (tag.index == _emitted + k) {
                    outSpan.publishTag(tag.map, k);
                }
            }
            outSpan[k] = _items[_emitted + k];
        }
        _emitted += nItems;
        outSpan.publish(nItems);
        return _emitted >= _items.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

/// @brief Keeps every item and every tag that reached it, each tag index absolute in the stream it arrived on.
template<typename TItem>
struct ItemCollector : gr::Block<ItemCollector<TItem>> {
    gr::PortIn<TItem> in;
    GR_MAKE_REFLECTABLE(ItemCollector, in);

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

const boost::ut::suite<"DataSetToStream boundary under the scheduler"> boundarySchedulerTests = [] {
    using namespace boost::ut;
    using namespace gr;

    "records cross the boundary under a scheduler whose buffer is narrower than a record"_test = [] {
        constexpr std::size_t kSamples = 8192UZ;
        constexpr std::size_t kGood    = 5UZ;

        Graph graph;
        auto& source = graph.emplaceBlock<StagedSource<Record<float>>>();
        for (std::size_t k = 0UZ; k < kGood; ++k) {
            source._items.push_back(makeExtractedRecord<float>(kSamples, 48000.f));
        }
        Record<float> twoExtents = makeExtractedRecord<float>(kSamples, 48000.f);
        twoExtents.extents.push_back(static_cast<std::int32_t>(kSamples)); // a shape no stream can carry
        source._items.push_back(std::move(twoExtents));
        source._tags.emplace_back(1UZ, property_map{{"probe", std::string("here")}});

        std::vector<float> expected;
        for (std::size_t k = 0UZ; k < kGood; ++k) {
            expected.insert(expected.end(), source._items[k].signal_values.begin(), source._items[k].signal_values.end());
        }

        auto& convert = graph.emplaceBlock<DataSetToStream<float>>();
        auto& samples = graph.emplaceBlock<ItemCollector<float>>();
        auto& refused = graph.emplaceBlock<ItemCollector<Record<float>>>();
        expect(graph.connect<"out", "in">(source, convert).has_value());
        expect(graph.connect<"out", "in">(convert, samples, EdgeParameters{.minBufferSize = 1024UZ}).has_value());
        expect(graph.connect<"reject", "in">(convert, refused).has_value());

        scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        expect(eq(samples._items.size(), expected.size())) << "every sample of every admitted record reaches the stream";
        expect(std::ranges::equal(samples._items, expected)) << "in order and exactly once, however the buffer cut them";
        expect(eq(refused._items.size(), 1UZ)) << "the record no stream can carry leaves by the reject port rather than stopping the graph";
        expect(eq(convert.nRejectedRecords, 1ULL));

        std::size_t probes  = 0UZ;
        std::size_t probeAt = 0UZ;
        for (const Tag& tag : samples._tags) {
            if (tag.map.contains(property_map::key_type("probe"))) {
                ++probes;
                probeAt = tag.index;
            }
        }
        expect(eq(probes, 1UZ)) << "the tag that arrived with the second record is published once";
        expect(eq(probeAt, kSamples)) << "at the first sample of that record";
    };
};

int main() { /* not needed for UT */ }
