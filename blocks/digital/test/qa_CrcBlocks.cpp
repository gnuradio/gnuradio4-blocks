#include <boost/ut.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

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

/// The two catalog parameter sets the cases below run under.
constexpr ParameterSet kSets[] = {
    {"CRC-32/ISO-HDLC", 32U, 0x04C11DB7ULL, 0xFFFFFFFFULL, 0xFFFFFFFFULL, true, true},
    {"CRC-16/IBM-3740", 16U, 0x1021ULL, 0xFFFFULL, 0x0000ULL, false, false},
};

[[nodiscard]] gr::property_map settingsOf(const ParameterSet& set, const char* order, gr::Size_t skip = 0U) { return {{"width", set.width}, {"poly", set.poly}, {"initial_value", set.init}, {"final_xor", set.xorOut}, {"input_reflected", set.refIn}, {"result_reflected", set.refOut}, {"crc_byte_order", std::string(order)}, {"skip_header_bytes", skip}}; }

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

[[nodiscard]] bool metaBool(const Record& record, std::string_view key) {
    const gr::property_map& map   = metaOf(record);
    const auto              entry = map.find(gr::property_map::key_type(key));
    return entry != map.end() && entry->second.value_or(false);
}

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
