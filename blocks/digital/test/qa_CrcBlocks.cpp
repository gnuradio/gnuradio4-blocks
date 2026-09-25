#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <format>
#include <limits>
#include <numeric>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/digital/Crc.hpp>
#include <gnuradio-4.0/digital/CrcBlocks.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

// Both blocks are record-oriented — one `DataSet` in, at most one `DataSet` out — and the kernel is stateless
// and immutable after construction, so no window size can change an answer. The property asserted below is
// the one that matters at that shape: a record's answer does not depend on how many records traveled with it.

namespace qa_crc_blocks {

using gr::blocks::digital::CrcAppend;
using gr::blocks::digital::CrcCheck;

using Record = gr::DataSet<std::uint8_t>;

struct ParameterSet {
    const char*   name;
    gr::Size_t    width;
    std::uint64_t poly;
    std::uint64_t init;
    std::uint64_t xorOut;
    bool          refIn;
    bool          refOut;
};

/// Three catalog parameter sets, plus a 64-bit row so the byte loop is exercised at every width.
constexpr ParameterSet kSets[] = {
    {"CRC-32/ISO-HDLC", 32U, 0x04C11DB7ULL, 0xFFFFFFFFULL, 0xFFFFFFFFULL, true, true},
    {"CRC-16/IBM-3740", 16U, 0x1021ULL, 0xFFFFULL, 0x0000ULL, false, false},
    {"CRC-8 poly 0x07 init 0xFF", 8U, 0x07ULL, 0xFFULL, 0x00ULL, false, false},
    {"CRC-64/XZ", 64U, 0x42F0E1EBA9EA3693ULL, ~0ULL, ~0ULL, true, true},
};

constexpr const char* kOrders[] = {"big", "little"};

[[nodiscard]] gr::property_map settingsOf(const ParameterSet& set, const char* order, gr::Size_t skip = 0U) { return {{"width", set.width}, {"poly", set.poly}, {"initial_value", set.init}, {"final_xor", set.xorOut}, {"input_reflected", set.refIn}, {"result_reflected", set.refOut}, {"crc_byte_order", std::string(order)}, {"skip_header_bytes", skip}}; }

[[nodiscard]] gr::digital::Crc kernelOf(const ParameterSet& set) { return gr::digital::Crc(static_cast<std::uint8_t>(set.width), set.poly, set.init, set.xorOut, set.refIn, set.refOut); }

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

[[nodiscard]] Record makeRecord(std::span<const std::uint8_t> bytes, std::int64_t timestamp = 12345LL) {
    Record record;
    record.timestamp = timestamp;
    record.signal_values.assign(bytes.begin(), bytes.end());
    record.extents.push_back(static_cast<std::int32_t>(bytes.size()));
    record.signal_names.emplace_back("payload");
    record.signal_quantities.emplace_back("bytes");
    record.signal_units.emplace_back("");
    record.meta_information.emplace_back();
    record.meta_information[0UZ].insert_or_assign(gr::property_map::key_type("trigger_name"), gr::pmt::Value(std::string("access_code")));
    record.timing_events.emplace_back();
    record.timing_events[0UZ].emplace_back(3, gr::property_map{});
    return record;
}

[[nodiscard]] std::vector<Record> append(CrcAppend& block, std::span<const Record> input) {
    std::vector<Record>                           output(input.size());
    gr::blocks::testing::span::InputSpan<Record>  inSpan{input};
    gr::blocks::testing::span::OutputSpan<Record> outSpan{std::span<Record>(output)};
    std::ignore = block.processBulk(inSpan, outSpan);
    output.resize(outSpan.count);
    return output;
}

struct Sorted {
    std::vector<Record> ok{};
    std::vector<Record> fail{};
    std::size_t         consumed = 0UZ;
};

[[nodiscard]] Sorted check(CrcCheck& block, std::span<const Record> input, bool failConnected = true) {
    Sorted                                        result;
    std::vector<Record>                           okBuffer(input.size());
    std::vector<Record>                           failBuffer(input.size());
    gr::blocks::testing::span::InputSpan<Record>  inSpan{input};
    gr::blocks::testing::span::OutputSpan<Record> okSpan{std::span<Record>(okBuffer)};
    gr::blocks::testing::span::OutputSpan<Record> failSpan{failConnected ? std::span<Record>(failBuffer) : std::span<Record>{}, 0UZ, nullptr, failConnected};
    std::ignore = block.processBulk(inSpan, okSpan, failSpan);

    okBuffer.resize(okSpan.count);
    failBuffer.resize(failSpan.count);
    result.ok       = std::move(okBuffer);
    result.fail     = std::move(failBuffer);
    result.consumed = inSpan.consumed;
    return result;
}

/// The record's own map, or an empty one: a `DataSet` is not required to carry metadata and the tests read it anyway.
[[nodiscard]] const gr::property_map& metaOf(const Record& record) {
    static const gr::property_map kEmpty{};
    return record.meta_information.empty() ? kEmpty : record.meta_information.front();
}

/// @brief Index a result, throwing rather than reading past the end when it is shorter than @p index.
[[nodiscard]] const Record& at(const std::vector<Record>& records, std::size_t index) {
    if (index >= records.size()) {
        throw gr::exception(std::format("expected more than {} records, got {}", index, records.size()));
    }
    return records[index];
}

/// @brief The record a one-record run produced.
[[nodiscard]] Record only(const std::vector<Record>& records) { return at(records, 0UZ); }

/// @brief Flip one bit of a record, throwing when the byte is not there.
[[nodiscard]] Record corrupt(const Record& record, std::size_t bit) {
    Record            copy  = record;
    const std::size_t index = bit / 8UZ;
    if (index >= copy.signal_values.size()) {
        throw gr::exception(std::format("bit {} is past the {} bytes of this record", bit, copy.signal_values.size()));
    }
    copy.signal_values[index] = static_cast<std::uint8_t>(copy.signal_values[index] ^ (1U << (bit % 8UZ)));
    return copy;
}

/// @brief The axis layout as a flat shape, which compares without a nested empty-vector comparison.
[[nodiscard]] std::vector<std::size_t> axisShape(const Record& record) {
    std::vector<std::size_t> shape;
    for (const std::vector<std::uint8_t>& axis : record.axis_values) {
        shape.push_back(axis.size());
    }
    return shape;
}

/// @brief The first per-sample annotation of a record, as (index, key count), or (-1, 0) where there is none.
[[nodiscard]] std::pair<std::ptrdiff_t, std::size_t> firstEvent(const Record& record) {
    if (record.timing_events.empty() || record.timing_events.front().empty()) {
        return {-1, 0UZ};
    }
    const auto& event = record.timing_events.front().front();
    return {event.first, event.second.size()};
}

[[nodiscard]] std::uint64_t metaU64(const Record& record, std::string_view key) {
    const gr::property_map& map   = metaOf(record);
    const auto              entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? ~0ULL : entry->second.value_or(std::uint64_t{~0ULL});
}

[[nodiscard]] bool metaBool(const Record& record, std::string_view key) {
    const gr::property_map& map   = metaOf(record);
    const auto              entry = map.find(gr::property_map::key_type(key));
    return entry != map.end() && entry->second.value_or(false);
}

[[nodiscard]] bool metaHas(const Record& record, std::string_view key) { return metaOf(record).contains(gr::property_map::key_type(key)); }

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
};

[[nodiscard]] std::vector<std::uint8_t> randomBytes(Rng& rng, std::size_t count) {
    std::vector<std::uint8_t> bytes(count);
    for (std::uint8_t& byte : bytes) {
        byte = static_cast<std::uint8_t>(rng.next() & 0xFFU);
    }
    return bytes;
}

/// @brief Emits a fixed list of records and then ends, so the run is bounded by the data rather than by a timer.
struct RecordSource : gr::Block<RecordSource> {
    gr::PortOut<Record> out;
    GR_MAKE_REFLECTABLE(RecordSource, out);

    std::vector<Record> _records{};
    std::size_t         _position = 0UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t count = std::min(outSpan.size(), _records.size() - _position);
        for (std::size_t i = 0UZ; i < count; ++i) {
            outSpan[i] = _records[_position + i];
        }
        outSpan.publish(count);
        _position += count;
        if (_position == _records.size()) {
            return gr::work::Status::DONE;
        }
        return count == 0UZ ? gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS : gr::work::Status::OK;
    }
};

/// @brief Takes every record offered and publishes the running count, which is what gates the sink below.
struct PassingSink : gr::Block<PassingSink> {
    gr::PortIn<Record, gr::Async> in;
    GR_MAKE_REFLECTABLE(PassingSink, in);

    std::atomic<std::size_t>* _delivered = nullptr;
    std::vector<Record>       _records{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const Record& record : inSpan) {
            _records.push_back(record);
        }
        std::ignore = inSpan.consume(inSpan.size());
        _delivered->store(_records.size(), std::memory_order_release);
        return gr::work::Status::OK;
    }
};

/**
 * @brief Drains nothing until the passing path has delivered every record the run expects there.
 *
 * This is the shape that stalls a receive chain: the failure path only moves once the passing path has, so a check
 * that holds a refused record for want of room on `fail` holds the whole graph and never gets the room back.
 */
struct GatedFailingSink : gr::Block<GatedFailingSink> {
    gr::PortIn<Record, gr::Async> in;
    GR_MAKE_REFLECTABLE(GatedFailingSink, in);

    const std::atomic<std::size_t>* _delivered  = nullptr;
    std::size_t                     _releaseAt  = 0UZ;
    std::size_t                     _taken      = 0UZ;
    bool                            _allRefused = true;

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        if (_delivered->load(std::memory_order_acquire) < _releaseAt) {
            std::ignore = inSpan.consume(0UZ);
            return gr::work::Status::OK;
        }
        for (const Record& record : inSpan) {
            _allRefused = _allRefused && !metaBool(record, "crc_ok");
        }
        _taken += inSpan.size();
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/**
 * @brief Runs a graph under the simple scheduler, failing rather than hanging when it does not finish.
 *
 * The bound is a hang guard and nothing else: it is what turns a graph that never completes into a failed test on
 * any machine, and no assertion is drawn from how far below it the run lands.
 *
 * @p collect runs while the scheduler still owns the graph, because the references `emplaceBlock` returned point
 * into blocks the scheduler destroys with itself.
 */
template<typename TCollect>
void runGraph(gr::Graph flow, std::chrono::milliseconds bound, TCollect&& collect) {
    gr::scheduler::Simple<> scheduler;
    boost::ut::expect(scheduler.exchange(std::move(flow)).has_value());
    std::atomic<bool> done{false};
    std::thread       runner([&scheduler, &done] {
        std::ignore = scheduler.runAndWait();
        done        = true;
    });

    const auto start = std::chrono::steady_clock::now();
    while (!done.load() && std::chrono::steady_clock::now() - start < bound) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!done.load()) {
        scheduler.requestStop();
        boost::ut::expect(false) << std::format("the graph did not finish within {} ms", bound.count());
    }
    runner.join();
    collect();
}

} // namespace qa_crc_blocks

using namespace qa_crc_blocks;

const boost::ut::suite<"crc blocks"> crcBlockTests = [] {
    using namespace boost::ut;

    "append then check puts every record on ok, for every parameter set and both byte orders"_test = [] {
        Rng rng{};
        for (const ParameterSet& set : kSets) {
            for (const char* order : kOrders) {
                CrcAppend appender = make<CrcAppend>(settingsOf(set, order));
                CrcCheck  checker  = make<CrcCheck>(settingsOf(set, order));

                std::vector<Record> records;
                for (std::size_t i = 0UZ; i < 1000UZ; ++i) {
                    records.push_back(makeRecord(randomBytes(rng, 1UZ + (i % 97UZ))));
                }

                const std::vector<Record> protectedRecords = append(appender, std::span<const Record>(records));
                expect(eq(protectedRecords.size(), records.size())) << set.name;

                const Sorted sorted = check(checker, std::span<const Record>(protectedRecords));
                expect(eq(sorted.ok.size(), records.size())) << std::format("{} {}: every record passes", set.name, order);
                expect(eq(sorted.fail.size(), 0UZ)) << std::format("{} {}", set.name, order);
                expect(eq(sorted.consumed, records.size()));
            }
        }
    };

    "the appended value is the kernel's, and the block says what it computed"_test = [] {
        Rng rng{};
        for (const ParameterSet& set : kSets) {
            const gr::digital::Crc          kernel   = kernelOf(set);
            const std::size_t               crcBytes = static_cast<std::size_t>(set.width) / 8UZ;
            const std::vector<std::uint8_t> payload  = randomBytes(rng, 61UZ);

            for (const char* order : kOrders) {
                CrcAppend                 appender = make<CrcAppend>(settingsOf(set, order));
                const std::vector<Record> input{makeRecord(payload)};
                const std::vector<Record> out = append(appender, std::span<const Record>(input));

                expect(eq(out.size(), 1UZ));
                const std::uint64_t want = kernel.compute(payload);
                expect(eq(metaU64(at(out, 0UZ), "crc_value"), want)) << std::format("{} {}", set.name, order);
                expect(eq(metaU64(at(out, 0UZ), "crc_width"), static_cast<std::uint64_t>(set.width))) << set.name;
                expect(eq(at(out, 0UZ).signal_values.size(), payload.size() + crcBytes));
                expect(eq(static_cast<std::size_t>(at(out, 0UZ).extents[0]), payload.size() + crcBytes)) << "extents follows the new size";

                for (std::size_t i = 0UZ; i < crcBytes; ++i) {
                    const std::size_t  shift = std::string_view(order) == "big" ? 8UZ * (crcBytes - 1UZ - i) : 8UZ * i;
                    const std::uint8_t got   = at(out, 0UZ).signal_values[payload.size() + i];
                    expect(eq(static_cast<std::uint64_t>(got), (want >> shift) & 0xFFULL)) << std::format("{} {}: appended byte {}", set.name, order, i);
                }
            }
        }
    };

    "every single-bit corruption of a 32-byte record lands on fail"_test = [] {
        Rng rng{};
        for (const ParameterSet& set : kSets) {
            CrcAppend                       appender = make<CrcAppend>(settingsOf(set, "big"));
            CrcCheck                        checker  = make<CrcCheck>(settingsOf(set, "big"));
            const std::vector<std::uint8_t> payload  = randomBytes(rng, 32UZ);
            const std::vector<Record>       input{makeRecord(payload)};
            const Record                    good = only(append(appender, std::span<const Record>(input)));

            std::size_t detected = 0UZ;
            for (std::size_t bit = 0UZ; bit < 8UZ * payload.size(); ++bit) {
                const std::vector<Record> one{corrupt(good, bit)};
                const Sorted              sorted = check(checker, std::span<const Record>(one));
                detected += sorted.fail.size();
                if (!sorted.fail.empty()) {
                    expect(!metaBool(at(sorted.fail, 0UZ), "crc_ok")) << set.name;
                }
            }
            expect(eq(detected, 8UZ * payload.size())) << std::format("{}: all 256 single-bit flips detected, asserted exhaustively", set.name);
        }
    };

    "a record no longer than the guard is dropped rather than read past"_test = [] {
        constexpr gr::Size_t kSkip = 3U;
        for (const ParameterSet& set : kSets) {
            const std::size_t crcBytes = static_cast<std::size_t>(set.width) / 8UZ;
            const std::size_t shortest = static_cast<std::size_t>(kSkip) + crcBytes;

            CrcCheck            checker = make<CrcCheck>(settingsOf(set, "big", kSkip));
            std::vector<Record> tooShort;
            for (std::size_t n = 0UZ; n <= shortest; ++n) {
                tooShort.push_back(makeRecord(std::vector<std::uint8_t>(n, 0x5AU)));
            }
            const Sorted sorted = check(checker, std::span<const Record>(tooShort));
            expect(eq(sorted.ok.size(), 0UZ)) << set.name;
            expect(eq(sorted.fail.size(), 0UZ)) << std::format("{}: a record too short to hold a CRC is not a failed CRC", set.name);
            expect(eq(sorted.consumed, tooShort.size())) << "dropped, and consumed, so nothing wedges";

            CrcAppend           appender = make<CrcAppend>(settingsOf(set, "big", kSkip));
            std::vector<Record> nothingToProtect;
            for (std::size_t n = 0UZ; n <= static_cast<std::size_t>(kSkip); ++n) {
                nothingToProtect.push_back(makeRecord(std::vector<std::uint8_t>(n, 0xA5U)));
            }
            expect(eq(append(appender, std::span<const Record>(nothingToProtect)).size(), 0UZ)) << std::format("{}: nothing to protect, nothing published", set.name);

            // one byte past the guard is the first record that survives, on both blocks
            const std::vector<Record> justLongEnough{makeRecord(std::vector<std::uint8_t>(shortest + 1UZ, 0x11U))};
            expect(eq(check(checker, std::span<const Record>(justLongEnough)).fail.size(), 1UZ)) << set.name;
        }

        // skip_header_bytes at the largest value its own type holds. The guard reaches it by subtracting from the
        // record's length, so the record is dropped; a guard that added the skip to the CRC width would wrap on a
        // target whose size_t is 32 bits, and every record would pass a test it cannot satisfy.
        CrcCheck                  atMaximumSkip = make<CrcCheck>(settingsOf(kSets[0], "big", std::numeric_limits<gr::Size_t>::max()));
        const std::vector<Record> oneRecord{makeRecord(std::vector<std::uint8_t>(64UZ, 0x11U))};
        const Sorted              sorted = check(atMaximumSkip, std::span<const Record>(oneRecord));
        expect(eq(sorted.ok.size(), 0UZ));
        expect(eq(sorted.fail.size(), 0UZ)) << "a record shorter than skip_header_bytes is not a failed CRC";
        expect(eq(sorted.consumed, 1UZ)) << "dropped, and consumed, so nothing wedges";
    };

    "skip_header_bytes is excluded from the computation and never stripped"_test = [] {
        constexpr gr::Size_t kSkip = 5U;
        Rng                  rng{};
        for (const ParameterSet& set : kSets) {
            const gr::digital::Crc          kernel = kernelOf(set);
            const std::vector<std::uint8_t> whole  = randomBytes(rng, 40UZ);

            CrcAppend                 appender = make<CrcAppend>(settingsOf(set, "big", kSkip));
            const std::vector<Record> input{makeRecord(whole)};
            const Record              out = only(append(appender, std::span<const Record>(input)));
            expect(eq(metaU64(out, "crc_value"), kernel.compute(std::span<const std::uint8_t>(whole).subspan(kSkip)))) << set.name;

            CrcCheck     stripping = make<CrcCheck>([&] {
                gr::property_map settings = settingsOf(set, "big", kSkip);
                settings["discard_crc"]   = true;
                return settings;
            }());
            const Sorted sorted    = check(stripping, std::span<const Record>(std::vector<Record>{out}));
            expect(eq(sorted.ok.size(), 1UZ)) << set.name;
            expect(that % (at(sorted.ok, 0UZ).signal_values == whole)) << std::format("{}: discard_crc leaves the header and the payload exactly as they arrived", set.name);
            expect(eq(static_cast<std::size_t>(at(sorted.ok, 0UZ).extents[0]), whole.size()));
        }
    };

    "discard_crc is symmetric across ok and fail"_test = [] {
        Rng                             rng{};
        const ParameterSet&             set     = kSets[0];
        const std::vector<std::uint8_t> payload = randomBytes(rng, 24UZ);

        CrcAppend                 appender = make<CrcAppend>(settingsOf(set, "big"));
        const std::vector<Record> input{makeRecord(payload)};
        const Record              good = only(append(appender, std::span<const Record>(input)));
        const Record              bad  = corrupt(good, 0UZ);

        gr::property_map settings = settingsOf(set, "big");
        settings["discard_crc"]   = true;
        CrcCheck     checker      = make<CrcCheck>(settings);
        const Sorted sorted       = check(checker, std::span<const Record>(std::vector<Record>{good, bad}));

        expect(eq(sorted.ok.size(), 1UZ));
        expect(eq(sorted.fail.size(), 1UZ)) << "a failing record is forwarded, never dropped";
        expect(eq(at(sorted.ok, 0UZ).signal_values.size(), payload.size()));
        expect(eq(at(sorted.fail, 0UZ).signal_values.size(), payload.size())) << "stripped on the failing port exactly as on the passing one";
        expect(that % (at(sorted.ok, 0UZ).signal_values == payload));
        expect(eq(static_cast<std::size_t>(at(sorted.fail, 0UZ).extents[0]), payload.size()));
    };

    "the metadata says what happened and everything else is carried through"_test = [] {
        Rng                             rng{};
        const ParameterSet&             set      = kSets[0];
        const std::vector<std::uint8_t> payload  = randomBytes(rng, 50UZ);
        const Record                    original = makeRecord(payload, 987654321LL);

        CrcAppend    appender = make<CrcAppend>(settingsOf(set, "big"));
        const Record appended = only(append(appender, std::span<const Record>(std::vector<Record>{original})));

        expect(eq(appended.timestamp, original.timestamp));
        expect(that % (appended.signal_names == original.signal_names));
        expect(that % (appended.signal_quantities == original.signal_quantities));
        expect(that % (appended.axis_names == original.axis_names));
        expect(that % (axisShape(appended) == axisShape(original)));
        expect(eq(appended.timing_events.size(), original.timing_events.size()));
        expect(that % (firstEvent(appended) == firstEvent(original))) << "timing_events ride through untouched";
        expect(metaHas(appended, "trigger_name")) << "an upstream key survives beside the two this block writes";
        expect(metaHas(appended, "crc_value"));
        expect(metaHas(appended, "crc_width"));
        expect(!metaHas(appended, "crc_ok")) << "crc_append does not claim a verdict";

        CrcCheck     checker = make<CrcCheck>(settingsOf(set, "big"));
        const Record bad     = corrupt(appended, 63UZ);
        const Sorted sorted  = check(checker, std::span<const Record>(std::vector<Record>{appended, bad}));

        expect(eq(sorted.ok.size(), 1UZ));
        expect(eq(sorted.fail.size(), 1UZ));
        expect(metaBool(at(sorted.ok, 0UZ), "crc_ok"));
        expect(!metaBool(at(sorted.fail, 0UZ), "crc_ok"));
        expect(eq(metaU64(at(sorted.ok, 0UZ), "crc_value"), metaU64(appended, "crc_value"))) << "the received value, which on a passing record is the computed one";
        expect(metaHas(at(sorted.fail, 0UZ), "crc_value")) << "a merged port can still tell the two apart";
        expect(eq(at(sorted.ok, 0UZ).timestamp, original.timestamp));
        expect(metaHas(at(sorted.fail, 0UZ), "trigger_name"));
    };

    "an unconnected fail port is free"_test = [] {
        Rng                             rng{};
        const ParameterSet&             set      = kSets[0];
        const std::vector<std::uint8_t> payload  = randomBytes(rng, 16UZ);
        CrcAppend                       appender = make<CrcAppend>(settingsOf(set, "big"));
        const Record                    good     = only(append(appender, std::span<const Record>(std::vector<Record>{makeRecord(payload)})));
        const Record                    bad      = corrupt(good, 22UZ);

        CrcCheck     checker = make<CrcCheck>(settingsOf(set, "big"));
        const Sorted sorted  = check(checker, std::span<const Record>(std::vector<Record>{bad, good, bad}), false);
        expect(eq(sorted.ok.size(), 1UZ));
        expect(eq(sorted.consumed, 3UZ)) << "the failures are consumed rather than stalling the graph";
    };

    "a graph fed mostly damaged records finishes, and a full fail port does not hold it up"_test = [] {
        constexpr std::size_t kRecords    = 512UZ;
        constexpr std::size_t kEveryGood  = 128UZ; // one good record ends each group of this many
        constexpr std::size_t kGood       = kRecords / kEveryGood;
        constexpr std::size_t kPayload    = 24UZ;
        constexpr std::size_t kFailBuffer = 32UZ; // smaller than the run, so the port fills and stays full

        Rng                 rng{};
        const ParameterSet& set      = kSets[0];
        CrcAppend           appender = make<CrcAppend>(settingsOf(set, "big"));

        std::vector<Record> payloads;
        for (std::size_t i = 0UZ; i < kRecords; ++i) {
            payloads.push_back(makeRecord(randomBytes(rng, kPayload)));
        }
        const std::vector<Record> protectedRecords = append(appender, std::span<const Record>(payloads));
        expect(eq(protectedRecords.size(), kRecords));

        std::vector<Record> onTheWire;
        for (std::size_t i = 0UZ; i < kRecords; ++i) {
            const bool good = (i % kEveryGood) == kEveryGood - 1UZ;
            onTheWire.push_back(good ? at(protectedRecords, i) : corrupt(at(protectedRecords, i), 8UZ * (i % kPayload)));
        }

        std::atomic<std::size_t> delivered{0UZ};

        gr::Graph flow;
        auto&     source   = flow.emplaceBlock<RecordSource>();
        source._records    = onTheWire;
        auto& checker      = flow.emplaceBlock<CrcCheck>(settingsOf(set, "big"));
        auto& passing      = flow.emplaceBlock<PassingSink>();
        passing._delivered = &delivered;
        auto& failing      = flow.emplaceBlock<GatedFailingSink>();
        failing._delivered = &delivered;
        failing._releaseAt = kGood;

        expect(flow.connect<"out", "in">(source, checker).has_value());
        expect(flow.connect<"ok", "in">(checker, passing).has_value());
        expect(flow.connect<"fail", "in">(checker, failing, gr::EdgeParameters{.minBufferSize = kFailBuffer}).has_value());

        std::size_t   passedCount = 0UZ;
        bool          allPassed   = true;
        std::size_t   takenOnFail = 0UZ;
        bool          allRefused  = true;
        std::uint64_t dropped     = 0ULL;

        runGraph(std::move(flow), std::chrono::seconds(30), [&passing, &failing, &checker, &passedCount, &allPassed, &takenOnFail, &allRefused, &dropped] {
            passedCount = passing._records.size();
            for (const Record& record : passing._records) {
                allPassed = allPassed && metaBool(record, "crc_ok");
            }
            takenOnFail = failing._taken;
            allRefused  = failing._allRefused;
            dropped     = checker.nRefusedDropped;
        });

        expect(eq(passedCount, kGood)) << "every good record reaches the passing port";
        expect(allPassed) << "and nothing damaged arrives there";
        expect(allRefused) << "every record that did leave by the fail port is marked as failed";
        expect(lt(takenOnFail, kRecords - kGood)) << "the fail port ran out of room, which is the condition under test";
        expect(gt(dropped, 0ULL)) << "and the check counts the refused records it dropped for want of room, which is the state this run puts it in";
    };

    "a record's verdict does not depend on what traveled with it"_test = [] {
        Rng                 rng{};
        const ParameterSet& set      = kSets[1];
        CrcAppend           appender = make<CrcAppend>(settingsOf(set, "little"));
        CrcCheck            checker  = make<CrcCheck>(settingsOf(set, "little"));

        std::vector<Record> records;
        for (std::size_t i = 0UZ; i < 64UZ; ++i) {
            records.push_back(makeRecord(randomBytes(rng, 1UZ + i)));
        }
        const std::vector<Record> batch = append(appender, std::span<const Record>(records));

        std::vector<Record> oneAtATime;
        for (const Record& record : records) {
            const std::vector<Record> single{record};
            oneAtATime.push_back(only(append(appender, std::span<const Record>(single))));
        }
        for (std::size_t i = 0UZ; i < records.size(); ++i) {
            expect(that % (at(batch, i).signal_values == at(oneAtATime, i).signal_values)) << std::format("record {} is the same alone as in a batch", i);
        }
        expect(eq(check(checker, std::span<const Record>(batch)).ok.size(), records.size()));
    };

    "degenerate parameters throw at the block before anything is derived from them"_test = [] {
        for (const gr::Size_t bad : {0U, 1U, 2U, 3U, 5U, 6U, 12U, 65U, 100U, 1000U}) {
            expect(throws([bad] { std::ignore = make<CrcAppend>({{"width", bad}}); })) << std::format("width {}", bad);
            expect(throws([bad] { std::ignore = make<CrcCheck>({{"width", bad}}); })) << std::format("width {}", bad);
        }
        for (const gr::Size_t good : {8U, 16U, 24U, 32U, 64U}) {
            expect(nothrow([good] { std::ignore = make<CrcAppend>({{"width", good}, {"poly", std::uint64_t{0x07}}}); })) << std::format("width {}", good);
        }
        expect(throws([] { std::ignore = make<CrcAppend>({{"crc_byte_order", std::string("network")}}); }));
        expect(throws([] { std::ignore = make<CrcCheck>({{"crc_byte_order", std::string("")}}); }));
    };

    "a multi-signal DataSet is refused rather than read as one flat array"_test = [] {
        CrcAppend appender   = make<CrcAppend>(settingsOf(kSets[0], "big"));
        Record    twoSignals = makeRecord(std::vector<std::uint8_t>(16UZ, 0x33U));
        twoSignals.signal_names.emplace_back("second");
        const std::vector<Record> input{twoSignals};
        expect(throws([&appender, &input] { std::ignore = append(appender, std::span<const Record>(input)); }));
    };

    "an idle CRC block answers that it waits for input, not progress"_test = [] {
        CrcAppend appender = make<CrcAppend>(settingsOf(kSets[0], "big"));
        CrcCheck  checker  = make<CrcCheck>(settingsOf(kSets[0], "big"));
        appender.start();
        checker.start();
        std::vector<Record> okBuffer(2UZ);
        std::vector<Record> failBuffer(2UZ);

        gr::blocks::testing::span::InputSpan<Record>  appendIn{std::span<const Record>{}};
        gr::blocks::testing::span::OutputSpan<Record> appendOut{std::span<Record>(okBuffer)};
        expect(appender.processBulk(appendIn, appendOut) == gr::work::Status::INSUFFICIENT_INPUT_ITEMS) << "CrcAppend";

        gr::blocks::testing::span::InputSpan<Record>  checkIn{std::span<const Record>{}};
        gr::blocks::testing::span::OutputSpan<Record> okSpan{std::span<Record>(okBuffer)};
        gr::blocks::testing::span::OutputSpan<Record> failSpan{std::span<Record>(failBuffer)};
        expect(checker.processBulk(checkIn, okSpan, failSpan) == gr::work::Status::INSUFFICIENT_INPUT_ITEMS) << "CrcCheck";
    };
};

int main() { /* not needed for UT */ }
