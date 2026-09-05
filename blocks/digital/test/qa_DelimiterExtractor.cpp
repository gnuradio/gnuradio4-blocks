#include <boost/ut.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/digital/Crc.hpp>
#include <gnuradio-4.0/digital/AccessCodeCorrelator.hpp>
#include <gnuradio-4.0/digital/CrcBlocks.hpp>
#include <gnuradio-4.0/digital/DelimiterExtractor.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::digital::AccessCodeCorrelator;
using gr::blocks::digital::CrcCheck;
using gr::blocks::digital::DelimiterExtractor;
using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;

template<typename T>
using Record = gr::DataSet<T>;

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
    [[nodiscard]] std::size_t below(std::size_t bound) noexcept { return next() % bound; }
};

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

/// One item per '0'/'1' character.
[[nodiscard]] std::vector<std::uint8_t> itemsOf(std::string_view bits) {
    std::vector<std::uint8_t> items;
    items.reserve(bits.size());
    for (const char character : bits) {
        items.push_back(static_cast<std::uint8_t>(character == '1' ? 1 : 0));
    }
    return items;
}

/// The bit-stuffing encoder, written here because a round trip needs the direction the block does not specify.
[[nodiscard]] std::vector<std::uint8_t> stuff(std::span<const std::uint8_t> bits, unsigned k) {
    std::vector<std::uint8_t> coded;
    coded.reserve(bits.size() + bits.size() / k + 1UZ);
    unsigned ones = 0U;
    for (const std::uint8_t bit : bits) {
        coded.push_back(bit);
        if (bit != 0U) {
            ++ones;
            if (ones == k) {
                coded.push_back(0U);
                ones = 0U;
            }
        } else {
            ones = 0U;
        }
    }
    return coded;
}

/// Each byte as eight items, least significant bit first, which is what HDLC and AX.25 transmit.
[[nodiscard]] std::vector<std::uint8_t> bitsOfBytesLsbFirst(std::span<const std::uint8_t> bytes) {
    std::vector<std::uint8_t> bits;
    bits.reserve(bytes.size() * 8UZ);
    for (const std::uint8_t byte : bytes) {
        for (unsigned bit = 0U; bit < 8U; ++bit) {
            bits.push_back(static_cast<std::uint8_t>((byte >> bit) & 1U));
        }
    }
    return bits;
}

/// What one drive of the block produced on each of its two ports.
template<typename T>
struct Extracted {
    std::vector<Record<T>> out{};
    std::vector<Record<T>> reject{};
    std::size_t            consumed = 0UZ;
};

/**
 * @brief Drives the block over @p input in windows of @p chunk items, with @p tags planted at absolute offsets.
 *
 * Tags from up to 32 items before the window are presented again at a negative relative index, which is what the
 * framework does to a block holding a partial record across calls, so the filter that skips them is exercised.
 */
template<typename T>
[[nodiscard]] Extracted<T> feed(DelimiterExtractor<T>& block, std::span<const T> input, std::size_t chunk = 0UZ, std::span<const gr::Tag> tags = {}, bool rejectConnected = true, std::size_t room = 64UZ) {
    Extracted<T>           result;
    const std::size_t      stride = chunk == 0UZ ? std::max(input.size(), 1UZ) : chunk;
    std::vector<Record<T>> outScratch(room);
    std::vector<Record<T>> rejectScratch(room);

    for (std::size_t base = 0UZ; base < input.size();) {
        const std::size_t count    = std::min(stride, input.size() - base);
        const std::size_t lookback = base > 32UZ ? base - 32UZ : 0UZ;
        const auto        first    = std::ranges::lower_bound(tags, lookback, std::ranges::less{}, &gr::Tag::index);
        const auto        last     = std::ranges::lower_bound(tags, base + count, std::ranges::less{}, &gr::Tag::index);

        InputSpan<T>          inSpan(input.subspan(base, count), base, std::span<const gr::Tag>(first, last));
        OutputSpan<Record<T>> outSpan(std::span<Record<T>>(outScratch), 0UZ, nullptr, true);
        OutputSpan<Record<T>> rejectSpan(rejectConnected ? std::span<Record<T>>(rejectScratch) : std::span<Record<T>>{}, 0UZ, nullptr, rejectConnected);

        std::ignore = block.processBulk(inSpan, outSpan, rejectSpan);

        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            result.out.push_back(std::move(outScratch[k]));
        }
        for (std::size_t k = 0UZ; k < rejectSpan.count; ++k) {
            result.reject.push_back(std::move(rejectScratch[k]));
        }
        result.consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && rejectSpan.count == 0UZ) {
            break;
        }
        base += inSpan.consumed;
    }
    return result;
}

template<typename T>
[[nodiscard]] const gr::property_map& metaOf(const Record<T>& record) {
    static const gr::property_map kEmpty{};
    return record.meta_information.empty() ? kEmpty : record.meta_information.front();
}

template<typename T>
[[nodiscard]] std::uint64_t metaU64(const Record<T>& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? ~0ULL : entry->second.value_or(std::uint64_t{~0ULL});
}

template<typename T>
[[nodiscard]] gr::Size_t metaSize(const Record<T>& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::numeric_limits<gr::Size_t>::max() : entry->second.value_or(std::numeric_limits<gr::Size_t>::max());
}

template<typename T>
[[nodiscard]] std::string metaString(const Record<T>& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::string{} : std::string(entry->second.value_or(std::string_view{}));
}

template<typename T>
[[nodiscard]] bool metaHas(const Record<T>& record, std::string_view key) {
    return metaOf(record).contains(gr::property_map::key_type(key));
}

/// `0xC0` between frames over byte items with no transparency, which is the smallest framing the length tests need.
[[nodiscard]] gr::property_map byteFraming(gr::Size_t bound, gr::Size_t floorItems = 1U) { return {{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}, {"max_payload_items", bound}, {"min_payload_items", floorItems}}; }

/// The HDLC framing of anchor A over bit items.
[[nodiscard]] gr::property_map hdlcFraming(gr::Size_t bound) { return {{"end_delimiter", std::string("01111110")}, {"transparency", std::string("bit_stuffing")}, {"max_payload_items", bound}}; }

/// Flag, stuffed payload bits, flag — with the frames sharing their flags, as an HDLC link does.
[[nodiscard]] std::vector<std::uint8_t> hdlcWire(std::span<const std::vector<std::uint8_t>> payloads) {
    const std::string         flag = "01111110";
    std::vector<std::uint8_t> wire = itemsOf(flag);
    for (const std::vector<std::uint8_t>& payload : payloads) {
        const std::vector<std::uint8_t> coded = stuff(std::span<const std::uint8_t>(payload), 5U);
        wire.insert(wire.end(), coded.begin(), coded.end());
        const std::vector<std::uint8_t> closing = itemsOf(flag);
        wire.insert(wire.end(), closing.begin(), closing.end());
    }
    return wire;
}

/// `feed` for the boundary sweep, which produces tens of thousands of records and needs none of them kept.
template<typename T>
[[nodiscard]] std::size_t feedCounting(DelimiterExtractor<T>& block, std::span<const T> input, std::size_t chunk, std::size_t room) {
    const std::size_t      stride = chunk == 0UZ ? std::max(input.size(), 1UZ) : chunk;
    std::vector<Record<T>> outScratch(room);
    std::vector<Record<T>> rejectScratch(room);
    std::size_t            consumed = 0UZ;

    for (std::size_t base = 0UZ; base < input.size();) {
        const std::size_t     count = std::min(stride, input.size() - base);
        InputSpan<T>          inSpan(input.subspan(base, count), base);
        OutputSpan<Record<T>> outSpan(std::span<Record<T>>(outScratch), 0UZ, nullptr, true);
        OutputSpan<Record<T>> rejectSpan(std::span<Record<T>>(rejectScratch), 0UZ, nullptr, true);

        std::ignore = block.processBulk(inSpan, outSpan, rejectSpan);
        consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && rejectSpan.count == 0UZ) {
            break;
        }
        base += inSpan.consumed;
    }
    return consumed;
}

template<typename T>
struct FiniteSource : gr::Block<FiniteSource<T>> {
    gr::PortOut<T> out;
    GR_MAKE_REFLECTABLE(FiniteSource, out);
    std::vector<T> _data{};
    std::size_t    _pos = 0UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) noexcept {
        const std::size_t n = std::min(outSpan.size(), _data.size() - _pos);
        std::copy_n(_data.begin() + static_cast<std::ptrdiff_t>(_pos), n, outSpan.begin());
        outSpan.publish(n);
        _pos += n;
        return _pos == _data.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

template<typename T>
struct RecordSink : gr::Block<RecordSink<T>> {
    gr::PortIn<gr::DataSet<T>, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);
    std::vector<gr::DataSet<T>> _records{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const auto& record : inSpan) {
            _records.push_back(record);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/**
 * @brief Runs a graph to completion under the simple scheduler, stopping it rather than hanging if it wedges.
 *
 * @p collect runs while the scheduler still owns the graph, because the references `emplaceBlock` returned point
 * into blocks the scheduler destroys with itself. Reading a block after this function returns reads freed memory.
 */
template<typename TCollect>
void runGraph(gr::Graph flow, TCollect&& collect) {
    gr::scheduler::Simple<> scheduler;
    boost::ut::expect(scheduler.exchange(std::move(flow)).has_value());
    std::atomic<bool> done{false};
    std::thread       runner([&scheduler, &done] {
        std::ignore = scheduler.runAndWait();
        done        = true;
    });
    const auto        start = std::chrono::steady_clock::now();
    while (!done.load() && std::chrono::steady_clock::now() - start < std::chrono::seconds(30)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!done.load()) {
        scheduler.requestStop();
        boost::ut::expect(false) << "the graph did not finish within thirty seconds";
    }
    runner.join();
    collect();
}

} // namespace

const boost::ut::suite<"delimiter extractor"> delimiterExtractorTests = [] {
    using namespace boost::ut;

    "the records do not change with the chunk size"_test = [] {
        // anchor A extended to two frames: 0x3F with one stuffed bit, then 0x00 with none
        const std::vector<std::uint8_t> stream = itemsOf(std::string("01111110") + "111110100" + "01111110" + "00000000" + "01111110");

        for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 4096UZ}) {
            DelimiterExtractor<std::uint8_t> block     = make<DelimiterExtractor<std::uint8_t>>(hdlcFraming(1024U));
            const Extracted<std::uint8_t>    extracted = feed<std::uint8_t>(block, std::span<const std::uint8_t>(stream), chunk);
            expect(eq(extracted.out.size(), 2UZ)) << std::format("chunk {}", chunk);
            expect(eq(extracted.reject.size(), 0UZ)) << std::format("chunk {}", chunk);
            if (extracted.out.size() != 2UZ) {
                continue;
            }
            expect(that % (extracted.out[0UZ].signal_values == std::vector<std::uint8_t>{1U, 1U, 1U, 1U, 1U, 1U, 0U, 0U})) << std::format("chunk {}", chunk);
            expect(that % (extracted.out[1UZ].signal_values == std::vector<std::uint8_t>(8UZ, 0U))) << std::format("chunk {}", chunk);
            expect(eq(metaU64(extracted.out[0UZ], "sample_start"), std::uint64_t{8})) << std::format("chunk {}", chunk);
            expect(eq(metaU64(extracted.out[1UZ], "sample_start"), std::uint64_t{25})) << std::format("chunk {}", chunk);
            expect(eq(metaSize(extracted.out[0UZ], "stuffing_removed"), gr::Size_t{1})) << std::format("chunk {}", chunk);
            expect(eq(metaSize(extracted.out[1UZ], "stuffing_removed"), gr::Size_t{0})) << std::format("chunk {}", chunk);
            expect(eq(metaU64(extracted.out[0UZ], "sequence"), std::uint64_t{0})) << std::format("chunk {}", chunk);
            expect(eq(metaU64(extracted.out[1UZ], "sequence"), std::uint64_t{1})) << std::format("chunk {}", chunk);
            expect(eq(metaString(extracted.out[0UZ], "trigger_name"), std::string("delimiter"))) << std::format("chunk {}", chunk);
            expect(that % (extracted.out[0UZ].signal_names == std::vector<std::string>{"payload"})) << std::format("chunk {}", chunk);
            expect(eq(static_cast<std::size_t>(extracted.out[0UZ].extents.at(0)), 8UZ)) << std::format("chunk {}", chunk);
        }

        // and the same records come out under the scheduler, where the window is the framework's to choose
        gr::Graph flow;
        auto&     source = flow.emplaceBlock<FiniteSource<std::uint8_t>>();
        source._data     = stream;
        auto& block      = flow.emplaceBlock<DelimiterExtractor<std::uint8_t>>(hdlcFraming(1024U));
        auto& sink       = flow.emplaceBlock<RecordSink<std::uint8_t>>();
        expect(flow.connect<"out", "in">(source, block).has_value());
        expect(flow.connect<"out", "in">(block, sink).has_value());
        std::vector<Record<std::uint8_t>> scheduled;
        runGraph(std::move(flow), [&scheduled, &sink] { scheduled = sink._records; });
        expect(eq(scheduled.size(), 2UZ)) << "under the scheduler";
        if (scheduled.size() == 2UZ) {
            expect(that % (scheduled[0UZ].signal_values == std::vector<std::uint8_t>{1U, 1U, 1U, 1U, 1U, 1U, 0U, 0U}));
            expect(that % (scheduled[1UZ].signal_values == std::vector<std::uint8_t>(8UZ, 0U)));
        }
    };

    "an overrun is refused whole and the link recovers"_test = [] {
        // 0xC0 delimiter, a bound of four, a five-item region, then a genuine three-item frame
        const std::vector<std::uint8_t>  stream{0xC0U, 0x11U, 0x22U, 0x33U, 0x44U, 0x55U, 0xC0U, 0x66U, 0x77U, 0x88U, 0xC0U};
        DelimiterExtractor<std::uint8_t> block     = make<DelimiterExtractor<std::uint8_t>>(byteFraming(4U));
        const Extracted<std::uint8_t>    extracted = feed<std::uint8_t>(block, std::span<const std::uint8_t>(stream));

        expect(eq(extracted.reject.size(), 1UZ));
        expect(eq(extracted.out.size(), 1UZ));
        expect(eq(block.nOverrunFrames, 1ULL));
        if (extracted.reject.size() == 1UZ) {
            expect(that % (extracted.reject[0UZ].signal_values == std::vector<std::uint8_t>{0x11U, 0x22U, 0x33U, 0x44U})) << "closed at exactly max_payload_items";
            expect(eq(metaString(extracted.reject[0UZ], "discard_reason"), std::string("overrun")));
        }
        if (extracted.out.size() == 1UZ) {
            expect(that % (extracted.out[0UZ].signal_values == std::vector<std::uint8_t>{0x66U, 0x77U, 0x88U})) << "the next genuine frame is recovered intact";
        }

        // ten thousand consecutive overruns, and the block is still producing records at the end
        std::vector<std::uint8_t> hostile;
        hostile.reserve(60'010UZ);
        for (std::size_t which = 0UZ; which < 10'000UZ; ++which) {
            hostile.push_back(0xC0U);
            for (std::size_t k = 0UZ; k < 5UZ; ++k) {
                hostile.push_back(static_cast<std::uint8_t>(1U + k));
            }
        }
        hostile.push_back(0xC0U);
        hostile.push_back(0x99U);
        hostile.push_back(0xAAU);
        hostile.push_back(0xC0U);

        DelimiterExtractor<std::uint8_t> wedged  = make<DelimiterExtractor<std::uint8_t>>(byteFraming(4U));
        const Extracted<std::uint8_t>    survive = feed<std::uint8_t>(wedged, std::span<const std::uint8_t>(hostile));
        expect(eq(wedged.nOverrunFrames, 10'000ULL));
        expect(eq(survive.out.size(), 1UZ)) << "still producing after ten thousand overruns";
        if (survive.out.size() == 1UZ) {
            expect(that % (survive.out[0UZ].signal_values == std::vector<std::uint8_t>{0x99U, 0xAAU}));
        }
    };

    "undersize is a refusal and an empty region is not a frame"_test = [] {
        const std::vector<std::uint8_t>  shortFrame{0xC0U, 0x11U, 0x22U, 0xC0U};
        DelimiterExtractor<std::uint8_t> block    = make<DelimiterExtractor<std::uint8_t>>(byteFraming(16U, 3U));
        const Extracted<std::uint8_t>    tooShort = feed<std::uint8_t>(block, std::span<const std::uint8_t>(shortFrame));
        expect(eq(tooShort.reject.size(), 1UZ));
        expect(eq(tooShort.out.size(), 0UZ));
        expect(eq(block.nUndersizeFrames, 1ULL));
        expect(eq(block.nIdleDelimiters, 0ULL));
        if (tooShort.reject.size() == 1UZ) {
            expect(eq(metaString(tooShort.reject[0UZ], "discard_reason"), std::string("undersize")));
            expect(that % (tooShort.reject[0UZ].signal_values == std::vector<std::uint8_t>{0x11U, 0x22U}));
        }

        // an empty region publishes nothing on either port and consumes no sequence number
        const std::vector<std::uint8_t>  empty{0xC0U, 0xC0U, 0x11U, 0x22U, 0x33U, 0xC0U};
        DelimiterExtractor<std::uint8_t> idle      = make<DelimiterExtractor<std::uint8_t>>(byteFraming(16U, 3U));
        const Extracted<std::uint8_t>    extracted = feed<std::uint8_t>(idle, std::span<const std::uint8_t>(empty));
        expect(eq(extracted.reject.size(), 0UZ));
        expect(eq(extracted.out.size(), 1UZ));
        expect(eq(idle.nIdleDelimiters, 1ULL));
        expect(eq(idle.nUndersizeFrames, 0ULL));
        if (extracted.out.size() == 1UZ) {
            expect(eq(metaU64(extracted.out[0UZ], "sequence"), std::uint64_t{0})) << "the idle region consumed no sequence number";
        }
    };

    "an unaligned frame is refused carrying its unpacked bits"_test = [] {
        const std::string               payload = "10101010101010101010"; // twenty bits, not a whole byte
        const std::vector<std::uint8_t> stream  = itemsOf(std::string("01111110") + payload + "01111110");
        gr::property_map                settings{{"end_delimiter", std::string("01111110")}, {"max_payload_items", gr::Size_t{1024}}, {"payload_pack_bits", gr::Size_t{8}}};

        DelimiterExtractor<std::uint8_t> block     = make<DelimiterExtractor<std::uint8_t>>(settings);
        const Extracted<std::uint8_t>    extracted = feed<std::uint8_t>(block, std::span<const std::uint8_t>(stream));
        expect(eq(extracted.out.size(), 0UZ));
        expect(eq(extracted.reject.size(), 1UZ));
        expect(eq(block.nUnalignedFrames, 1ULL));
        if (extracted.reject.size() == 1UZ) {
            expect(eq(metaString(extracted.reject[0UZ], "discard_reason"), std::string("unaligned")));
            expect(eq(extracted.reject[0UZ].signal_values.size(), 20UZ)) << "the twenty unpacked bits, so nothing is lost to a partial item";
            expect(that % (extracted.reject[0UZ].signal_values == itemsOf(payload)));
        }
    };

    "an abort abandons the frame and the next flag recovers"_test = [] {
        const std::vector<std::uint8_t> stream = itemsOf(std::string("01111110") + "10100" + "1111111" + "01111110" + "01100" + "01111110");
        expect(eq(stream.size(), 41UZ));

        gr::property_map                 aborting = hdlcFraming(1024U);
        DelimiterExtractor<std::uint8_t> block    = make<DelimiterExtractor<std::uint8_t>>(aborting);
        const Extracted<std::uint8_t>    aborted  = feed<std::uint8_t>(block, std::span<const std::uint8_t>(stream));
        expect(eq(block.nAborts, 1ULL));
        expect(eq(aborted.reject.size(), 0UZ)) << "an abandoned frame has no stated extent, so it is not a record";
        expect(eq(aborted.out.size(), 1UZ));
        if (aborted.out.size() == 1UZ) {
            expect(that % (aborted.out[0UZ].signal_values == std::vector<std::uint8_t>{0U, 1U, 1U, 0U, 0U})) << "the frame after the abort is recovered intact";
        }

        gr::property_map disabled                = hdlcFraming(1024U);
        disabled["abort_ones"]                   = gr::Size_t{0};
        DelimiterExtractor<std::uint8_t> patient = make<DelimiterExtractor<std::uint8_t>>(disabled);
        const Extracted<std::uint8_t>    kept    = feed<std::uint8_t>(patient, std::span<const std::uint8_t>(stream));
        expect(eq(patient.nAborts, 0ULL));
        expect(eq(kept.out.size(), 2UZ)) << "with the abort disabled the seven ones are payload and both frames close";
    };

    "a start delimiter arriving mid-frame abandons and re-opens"_test = [] {
        // '$' opens and a line feed closes, which is NMEA 0183's shape with a one-item terminator
        const gr::property_map          settings{{"end_delimiter", std::string("00001010")}, {"start_delimiter", std::string("00100100")}, //
                     {"frame_open", std::string("start")}, {"bits_per_item", gr::Size_t{8}}, {"max_payload_items", gr::Size_t{64}}};
        const std::vector<std::uint8_t> stream{0x24U, 0x41U, 0x42U, 0x24U, 0x43U, 0x44U, 0x0AU};

        DelimiterExtractor<std::uint8_t> block     = make<DelimiterExtractor<std::uint8_t>>(settings);
        const Extracted<std::uint8_t>    extracted = feed<std::uint8_t>(block, std::span<const std::uint8_t>(stream));
        expect(eq(block.nRestarts, 1ULL));
        expect(eq(extracted.reject.size(), 0UZ)) << "an abandoned frame publishes nothing";
        expect(eq(extracted.out.size(), 1UZ));
        if (extracted.out.size() == 1UZ) {
            expect(that % (extracted.out[0UZ].signal_values == std::vector<std::uint8_t>{0x43U, 0x44U})) << "the frame opened at the restart is recovered";
        }
    };

    "a detector's trigger resynchronizes the machine and its metadata crosses onto one record"_test = [] {
        // a sync word with no run of six ones, so it cannot forge the flag it precedes
        constexpr std::uint64_t kSync    = 0xACDDA4E2F28C20ACULL;
        const auto              syncBits = [] {
            std::string text;
            for (unsigned bit = 64U; bit-- > 0U;) {
                text.push_back(((kSync >> bit) & 1ULL) != 0ULL ? '1' : '0');
            }
            return text;
        }();

        const std::vector<std::vector<std::uint8_t>> payloads{itemsOf("10110010"), itemsOf("01001101")};
        std::vector<std::uint8_t>                    stream = itemsOf(syncBits);
        const std::vector<std::uint8_t>              framed = hdlcWire(std::span<const std::vector<std::uint8_t>>(payloads));
        stream.insert(stream.end(), framed.begin(), framed.end());

        gr::Graph flow;
        auto&     source = flow.emplaceBlock<FiniteSource<std::uint8_t>>();
        source._data     = stream;
        auto& detector   = flow.emplaceBlock<AccessCodeCorrelator<std::uint8_t>>({{"access_code", syncBits}});
        auto& block      = flow.emplaceBlock<DelimiterExtractor<std::uint8_t>>(hdlcFraming(1024U));
        auto& sink       = flow.emplaceBlock<RecordSink<std::uint8_t>>();
        expect(flow.connect<"out", "in">(source, detector).has_value());
        expect(flow.connect<"out", "in">(detector, block).has_value());
        expect(flow.connect<"out", "in">(block, sink).has_value());
        std::vector<Record<std::uint8_t>> records;
        std::uint64_t                     resets = 0ULL;
        runGraph(std::move(flow), [&records, &resets, &sink, &block] {
            records = sink._records;
            resets  = block.nTriggerResets;
        });

        expect(eq(records.size(), 2UZ));
        expect(eq(resets, 1ULL));
        if (records.size() == 2UZ) {
            expect(eq(metaString(records[0UZ], "trigger_name"), std::string("access_code"))) << "the detector's label reaches the first record after the trigger";
            expect(eq(metaSize(records[0UZ], "sync_errors"), gr::Size_t{0}));
            expect(that % (records[0UZ].signal_values == payloads[0UZ]));
            expect(eq(metaString(records[1UZ], "trigger_name"), std::string("delimiter"))) << "and not the second, the held map being cleared with the first record";
            expect(!metaHas(records[1UZ], "sync_errors"));
            expect(that % (records[1UZ].signal_values == payloads[1UZ]));
        }
    };

    "a trigger arriving mid-frame abandons it unless the mechanism is off"_test = [] {
        const std::vector<std::vector<std::uint8_t>> payloads{itemsOf("10110010"), itemsOf("01001101")};
        const std::vector<std::uint8_t>              stream = hdlcWire(std::span<const std::vector<std::uint8_t>>(payloads));
        // the first payload starts at item 8, so a tag four items into it lands mid-frame
        const std::vector<gr::Tag> tags{gr::Tag{12UZ, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("access_code")}}}};

        DelimiterExtractor<std::uint8_t> resetting = make<DelimiterExtractor<std::uint8_t>>(hdlcFraming(1024U));
        const Extracted<std::uint8_t>    reset     = feed<std::uint8_t>(resetting, std::span<const std::uint8_t>(stream), 0UZ, std::span<const gr::Tag>(tags));
        expect(eq(resetting.nTriggerResets, 1ULL));
        expect(eq(reset.out.size(), 1UZ)) << "the frame the trigger fell inside is abandoned";
        if (reset.out.size() == 1UZ) {
            expect(that % (reset.out[0UZ].signal_values == payloads[1UZ]));
        }

        gr::property_map ignoring                = hdlcFraming(1024U);
        ignoring["trigger_resets"]               = false;
        DelimiterExtractor<std::uint8_t> passive = make<DelimiterExtractor<std::uint8_t>>(ignoring);
        const Extracted<std::uint8_t>    kept    = feed<std::uint8_t>(passive, std::span<const std::uint8_t>(stream), 0UZ, std::span<const gr::Tag>(tags));
        expect(eq(passive.nTriggerResets, 0ULL));
        expect(eq(kept.out.size(), 2UZ)) << "with the mechanism off the frame the trigger would have destroyed is produced";
    };

    "sequence is contiguous across both ports"_test = [] {
        constexpr std::size_t kFrames = 100UZ;
        Rng                   rng{};

        const std::vector<std::size_t> kUndersize{7UZ, 40UZ, 73UZ};
        const std::vector<std::size_t> kOverrun{3UZ, 16UZ, 29UZ, 42UZ, 55UZ, 68UZ, 81UZ};

        std::vector<std::size_t>  rejectPositions;
        std::vector<std::uint8_t> stream{0xC0U};
        for (std::size_t which = 0UZ; which < kFrames; ++which) {
            const bool        undersize = std::ranges::find(kUndersize, which) != kUndersize.end();
            const bool        overrun   = std::ranges::find(kOverrun, which) != kOverrun.end();
            const std::size_t length    = undersize ? 1UZ : (overrun ? 12UZ : 4UZ);
            if (undersize || overrun) {
                rejectPositions.push_back(which);
            }
            for (std::size_t k = 0UZ; k < length; ++k) {
                stream.push_back(static_cast<std::uint8_t>(1U + (rng.next() % 0x7EULL)));
            }
            stream.push_back(0xC0U);
        }
        expect(eq(rejectPositions.size(), 10UZ)) << "seven overruns and three undersize frames";

        DelimiterExtractor<std::uint8_t> block     = make<DelimiterExtractor<std::uint8_t>>(byteFraming(8U, 2U));
        const Extracted<std::uint8_t>    extracted = feed<std::uint8_t>(block, std::span<const std::uint8_t>(stream), 0UZ, {}, true, 256UZ);
        expect(eq(extracted.out.size(), 90UZ));
        expect(eq(extracted.reject.size(), 10UZ));

        std::vector<std::uint64_t> merged;
        std::vector<std::uint64_t> onOut;
        for (const Record<std::uint8_t>& record : extracted.out) {
            onOut.push_back(metaU64(record, "sequence"));
            merged.push_back(onOut.back());
        }
        for (const Record<std::uint8_t>& record : extracted.reject) {
            merged.push_back(metaU64(record, "sequence"));
        }
        std::ranges::sort(merged);
        std::vector<std::uint64_t> wanted(kFrames);
        for (std::size_t i = 0UZ; i < kFrames; ++i) {
            wanted[i] = static_cast<std::uint64_t>(i);
        }
        expect(that % (merged == wanted)) << "the union of the two ports is contiguous from zero with no gap and no repeat";

        std::vector<std::uint64_t> gaps;
        for (std::size_t i = 1UZ; i < onOut.size(); ++i) {
            for (std::uint64_t missing = onOut[i - 1UZ] + 1ULL; missing < onOut[i]; ++missing) {
                gaps.push_back(missing);
            }
        }
        if (!onOut.empty()) {
            for (std::uint64_t missing = 0ULL; missing < onOut.front(); ++missing) {
                gaps.push_back(missing);
            }
        }
        std::ranges::sort(gaps);
        std::vector<std::uint64_t> rejected;
        for (const Record<std::uint8_t>& record : extracted.reject) {
            rejected.push_back(metaU64(record, "sequence"));
        }
        std::ranges::sort(rejected);
        expect(that % (gaps == rejected)) << "a gap in the main port's own sequence is exactly a record that went to reject";
    };

    "interior tags become relative timing events"_test = [] {
        // anchor A: raw payload items 8 to 16, of which item 13 is the stuffed bit
        const std::vector<std::uint8_t> stream = itemsOf(std::string("01111110") + "111110100" + "01111110");
        std::vector<gr::Tag>            tags;
        tags.emplace_back(0UZ, gr::property_map{{"marker", gr::Size_t{99}}}); // outside any frame
        for (std::size_t at = 8UZ; at <= 16UZ; ++at) {
            tags.emplace_back(at, gr::property_map{{"marker", static_cast<gr::Size_t>(at)}});
        }

        for (const std::size_t chunk : {1UZ, 5UZ, 4096UZ}) {
            DelimiterExtractor<std::uint8_t> block     = make<DelimiterExtractor<std::uint8_t>>(hdlcFraming(1024U));
            const Extracted<std::uint8_t>    extracted = feed<std::uint8_t>(block, std::span<const std::uint8_t>(stream), chunk, std::span<const gr::Tag>(tags));
            expect(eq(extracted.out.size(), 1UZ)) << std::format("chunk {}", chunk);
            if (extracted.out.size() != 1UZ) {
                continue;
            }
            const auto& events = extracted.out[0UZ].timing_events.at(0);
            expect(eq(events.size(), 9UZ)) << std::format("chunk {}: one event per payload item, and no tag twice", chunk);
            const std::vector<std::ptrdiff_t> wanted{0, 1, 2, 3, 4, 5, 5, 6, 7};
            for (std::size_t i = 0UZ; i < std::min(events.size(), wanted.size()); ++i) {
                expect(eq(events[i].first, wanted[i])) << std::format("chunk {}: event {}", chunk, i);
            }
            expect(eq(block.nTagsOnRemovedItems, 1ULL)) << std::format("chunk {}: the tag on the stuffed bit lands on the next surviving item", chunk);
            expect(eq(block.nTagsOutsideFrames, 1ULL)) << std::format("chunk {}: the tag before the first flag is outside every frame", chunk);
        }

        // a tag on a stuffed bit that is the frame's last raw item computes an index one past the end
        const std::vector<std::uint8_t>  trailing = itemsOf(std::string("01111110") + "111110" + "01111110");
        const std::vector<gr::Tag>       last{gr::Tag{13UZ, gr::property_map{{"marker", gr::Size_t{1}}}}};
        DelimiterExtractor<std::uint8_t> edge    = make<DelimiterExtractor<std::uint8_t>>(hdlcFraming(1024U));
        const Extracted<std::uint8_t>    dropped = feed<std::uint8_t>(edge, std::span<const std::uint8_t>(trailing), 1UZ, std::span<const gr::Tag>(last));
        expect(eq(dropped.out.size(), 1UZ));
        if (dropped.out.size() == 1UZ) {
            expect(eq(dropped.out[0UZ].signal_values.size(), 5UZ));
            expect(eq(dropped.out[0UZ].timing_events.at(0).size(), 0UZ)) << "an out-of-range event is dropped rather than clamped to the last item";
        }
        expect(eq(edge.nDroppedTimingEvents, 1ULL));
    };

    "a soft record carries the original values rather than the slices"_test = [] {
        const std::vector<std::uint8_t> bits = itemsOf(std::string("01111110") + "111110100" + "01111110");
        std::vector<float>              soft(bits.size());
        for (std::size_t i = 0UZ; i < bits.size(); ++i) {
            soft[i] = bits[i] != 0U ? 0.75f : -0.25f; // the slices and the values differ at every item
        }

        DelimiterExtractor<float> block     = make<DelimiterExtractor<float>>(hdlcFraming(1024U));
        const Extracted<float>    extracted = feed<float>(block, std::span<const float>(soft));
        expect(eq(extracted.out.size(), 1UZ));
        if (extracted.out.size() == 1UZ) {
            const std::vector<float> wanted{0.75f, 0.75f, 0.75f, 0.75f, 0.75f, 0.75f, -0.25f, -0.25f};
            expect(that % (extracted.out[0UZ].signal_values == wanted)) << "the machine reads slices and the record carries items";
        }
    };

    "the pack stage assembles the decoded bits into bytes"_test = [] {
        const std::vector<std::uint8_t> stream   = itemsOf(std::string("01111110") + "111110100" + "01111110");
        gr::property_map                settings = hdlcFraming(1024U);
        settings["payload_pack_bits"]            = gr::Size_t{8};
        settings["payload_bit_order"]            = std::string("LsbFirst");

        DelimiterExtractor<std::uint8_t> block     = make<DelimiterExtractor<std::uint8_t>>(settings);
        const Extracted<std::uint8_t>    extracted = feed<std::uint8_t>(block, std::span<const std::uint8_t>(stream));
        expect(eq(extracted.out.size(), 1UZ));
        if (extracted.out.size() == 1UZ) {
            expect(that % (extracted.out[0UZ].signal_values == std::vector<std::uint8_t>{0x3FU})) << "0x3F, written out by hand rather than repacked from the input";
            expect(eq(static_cast<std::size_t>(extracted.out[0UZ].extents.at(0)), 1UZ));
            expect(eq(metaSize(extracted.out[0UZ], "stuffing_removed"), gr::Size_t{1})) << "the removed count stays in raw items throughout";
        }
    };

    "the stuffing_removed identity holds on every record"_test = [] {
        constexpr std::size_t     kFrames = 1000UZ;
        Rng                       rng{};
        std::vector<std::size_t>  rawRegions;
        std::vector<std::uint8_t> wire = itemsOf("01111110");
        for (std::size_t which = 0UZ; which < kFrames; ++which) {
            const std::size_t         length = 1UZ + rng.below(200UZ);
            std::vector<std::uint8_t> payload(length);
            for (std::uint8_t& bit : payload) {
                bit = static_cast<std::uint8_t>(rng.next() & 1ULL);
            }
            const std::vector<std::uint8_t> coded = stuff(std::span<const std::uint8_t>(payload), 5U);
            rawRegions.push_back(coded.size());
            wire.insert(wire.end(), coded.begin(), coded.end());
            const std::vector<std::uint8_t> flag = itemsOf("01111110");
            wire.insert(wire.end(), flag.begin(), flag.end());
        }

        gr::property_map settings                  = hdlcFraming(4096U);
        settings["min_payload_items"]              = gr::Size_t{5}; // so that some frames leave by reject and the identity is tested there too
        DelimiterExtractor<std::uint8_t> block     = make<DelimiterExtractor<std::uint8_t>>(settings);
        const Extracted<std::uint8_t>    extracted = feed<std::uint8_t>(block, std::span<const std::uint8_t>(wire), 0UZ, {}, true, 2048UZ);
        expect(eq(extracted.out.size() + extracted.reject.size(), kFrames));
        expect(gt(extracted.reject.size(), 0UZ)) << "the floor sends the short frames to the failure port";

        std::vector<std::pair<std::uint64_t, std::size_t>> byOrder;
        for (const Record<std::uint8_t>& record : extracted.out) {
            byOrder.emplace_back(metaU64(record, "sequence"), static_cast<std::size_t>(record.extents.at(0)) + static_cast<std::size_t>(metaSize(record, "stuffing_removed")));
        }
        for (const Record<std::uint8_t>& record : extracted.reject) {
            byOrder.emplace_back(metaU64(record, "sequence"), static_cast<std::size_t>(record.extents.at(0)) + static_cast<std::size_t>(metaSize(record, "stuffing_removed")));
        }
        std::ranges::sort(byOrder);
        expect(eq(byOrder.size(), rawRegions.size()));
        for (std::size_t i = 0UZ; i < std::min(byOrder.size(), rawRegions.size()); ++i) {
            expect(eq(byOrder[i].second, rawRegions[i])) << std::format("frame {}: extents[0] + stuffing_removed is the raw region between the delimiters", i);
        }
    };

    "the settings table is refused row by row"_test = [] {
        using Block       = DelimiterExtractor<std::uint8_t>;
        const auto staged = [](gr::property_map settings) { std::ignore = make<Block>(std::move(settings)); };

        expect(nothrow([&staged] { staged(byteFraming(16U)); })) << "the framing the other rows are varied from";

        expect(throws([&staged] { staged({{"end_delimiter", std::string("110000O0")}, {"bits_per_item", gr::Size_t{8}}, {"max_payload_items", gr::Size_t{16}}}); })) << "a character that is not '0' or '1'";
        expect(throws([&staged] { staged({{"end_delimiter", std::string(65UZ, '0')}, {"max_payload_items", gr::Size_t{16}}}); })) << "longer than the register";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("1100000")}, {"bits_per_item", gr::Size_t{8}}, {"max_payload_items", gr::Size_t{16}}}); })) << "not a whole number of items";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{9}}, {"max_payload_items", gr::Size_t{16}}}); })) << "bits_per_item outside [1, 8]";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("11000000")}, {"frame_open", std::string("whenever")}, {"max_payload_items", gr::Size_t{16}}}); })) << "an unknown frame_open";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("11000000")}, {"frame_open", std::string("start")}, {"max_payload_items", gr::Size_t{16}}}); })) << "frame_open 'start' with no start delimiter";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("11000000")}, {"start_delimiter", std::string("00100100")}, {"max_payload_items", gr::Size_t{16}}}); })) << "a start delimiter nothing consults";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("11000000")}, {"transparency", std::string("magic")}, {"max_payload_items", gr::Size_t{16}}}); })) << "an unknown transparency";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("01111110")}, {"stuff_after_ones", gr::Size_t{4}}, {"max_payload_items", gr::Size_t{16}}}); })) << "stuff_after_ones away from its default outside bit stuffing";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("01111110")}, {"escape_item", gr::Size_t{0xDB}}, {"max_payload_items", gr::Size_t{16}}}); })) << "escape_item away from its default outside byte escaping";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("01111110")}, {"transparency", std::string("bit_stuffing")}, {"bits_per_item", gr::Size_t{8}}, {"max_payload_items", gr::Size_t{16}}}); })) << "bit stuffing is a bit-level code";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("11000000")}, {"transparency", std::string("byte_escape")}, {"max_payload_items", gr::Size_t{16}}}); })) << "byte escaping needs a byte";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("01111110")}, {"transparency", std::string("bit_stuffing")}, {"stuff_after_ones", gr::Size_t{1}}, {"max_payload_items", gr::Size_t{16}}}); })) << "stuff_after_ones below two";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("01111110")}, {"transparency", std::string("bit_stuffing")}, {"abort_ones", gr::Size_t{5}}, {"max_payload_items", gr::Size_t{16}}}); })) << "an abort that fires before a stuff";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}, {"transparency", std::string("byte_escape")}, {"escape_item", gr::Size_t{0xDB}}, {"escape_map", std::vector<gr::Size_t>{0xDCU}}, {"max_payload_items", gr::Size_t{16}}}); })) << "an odd-length escape map";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}, {"transparency", std::string("byte_escape")}, {"escape_item", gr::Size_t{0xDB}}, {"escape_map", std::vector<gr::Size_t>{0xDCU, 0xC0U, 0xDCU, 0xDBU}}, {"max_payload_items", gr::Size_t{16}}}); })) << "a repeated escaped value";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}, {"transparency", std::string("byte_escape")}, {"escape_item", gr::Size_t{300}}, {"max_payload_items", gr::Size_t{16}}}); })) << "an escape item outside a byte";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}, {"max_payload_items", gr::Size_t{16}}, {"min_payload_items", gr::Size_t{0}}}); })) << "a zero floor";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}, {"max_payload_items", gr::Size_t{4}}, {"min_payload_items", gr::Size_t{5}}}); })) << "a floor above the bound";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}, {"max_payload_items", gr::Size_t{2147483648U}}}); })) << "a bound a record's extent could not describe";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("01111110")}, {"max_payload_items", gr::Size_t{16}}, {"payload_pack_bits", gr::Size_t{9}}}); })) << "a pack width outside [1, 8]";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}, {"max_payload_items", gr::Size_t{16}}, {"payload_pack_bits", gr::Size_t{8}}}); })) << "the pack stage needs one bit per item";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("0111110")}, {"transparency", std::string("bit_stuffing")}, {"max_payload_items", gr::Size_t{16}}}); })) << "a delimiter the destuffing rule would eat into";

        expect(throws([] {
            DelimiterExtractor<float> block(gr::property_map{{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}, {"max_payload_items", gr::Size_t{16}}});
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        })) << "a soft item carries one bit";
        expect(throws([] {
            DelimiterExtractor<float> block(gr::property_map{{"end_delimiter", std::string("01111110")}, {"max_payload_items", gr::Size_t{16}}, {"payload_pack_bits", gr::Size_t{8}}});
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        })) << "the pack stage needs byte items";
    };

    "the two required settings leave the block refusing to start and inert if driven"_test = [] {
        const auto inert = [](gr::property_map settings) {
            DelimiterExtractor<std::uint8_t>  block = make<DelimiterExtractor<std::uint8_t>>(std::move(settings));
            const std::vector<std::uint8_t>   stream{0xC0U, 0x11U, 0x22U, 0xC0U};
            std::vector<Record<std::uint8_t>> outScratch(4UZ);
            std::vector<Record<std::uint8_t>> rejectScratch(4UZ);

            InputSpan<std::uint8_t>          inSpan(std::span<const std::uint8_t>(stream), 0UZ);
            OutputSpan<Record<std::uint8_t>> outSpan(std::span<Record<std::uint8_t>>(outScratch), 0UZ, nullptr, true);
            OutputSpan<Record<std::uint8_t>> rejectSpan(std::span<Record<std::uint8_t>>(rejectScratch), 0UZ, nullptr, true);
            const gr::work::Status           status = block.processBulk(inSpan, outSpan, rejectSpan);
            boost::ut::expect(status == gr::work::Status::ERROR);
            boost::ut::expect(boost::ut::eq(inSpan.consumed, 0UZ));
            boost::ut::expect(boost::ut::eq(outSpan.count, 0UZ));
        };

        expect(throws([] {
            DelimiterExtractor<std::uint8_t> block{}; // nothing staged, so nothing is validated until start()
            block.start();
        })) << "both settings are required and neither has a default";
        expect(throws([] {
            DelimiterExtractor<std::uint8_t> block = make<DelimiterExtractor<std::uint8_t>>({{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}});
            block.start();
        })) << "a delimiter with no bound";
        expect(throws([] {
            DelimiterExtractor<std::uint8_t> block = make<DelimiterExtractor<std::uint8_t>>({{"max_payload_items", gr::Size_t{16}}});
            block.start();
        })) << "a bound with no delimiter";

        inert({{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}});
        inert({{"max_payload_items", gr::Size_t{16}}});
    };

    "the chain to CrcCheck carries a frame and refuses a corrupted one"_test = [] {
        const gr::digital::Crc                       fcs(std::uint8_t{16}, 0x1021ULL, 0xFFFFULL, 0xFFFFULL, true, true);
        const std::vector<std::vector<std::uint8_t>> messages{{0x41U, 0x42U, 0x43U}, {0x44U, 0x45U, 0x46U}, {0x47U, 0x48U, 0x49U}};

        std::vector<std::vector<std::uint8_t>> frames;
        for (std::size_t which = 0UZ; which < messages.size(); ++which) {
            std::vector<std::uint8_t> bytes = messages[which];
            const std::uint64_t       value = fcs.compute(std::span<const std::uint8_t>(bytes));
            bytes.push_back(static_cast<std::uint8_t>(value & 0xFFULL)); // HDLC sends the frame check sequence low byte first
            bytes.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFULL));
            std::vector<std::uint8_t> bits = bitsOfBytesLsbFirst(std::span<const std::uint8_t>(bytes));
            if (which == 1UZ) {
                bits[3UZ] = static_cast<std::uint8_t>(bits[3UZ] != 0U ? 0U : 1U); // one flipped payload bit
            }
            frames.push_back(std::move(bits));
        }
        const std::vector<std::uint8_t> wire = hdlcWire(std::span<const std::vector<std::uint8_t>>(frames));

        gr::property_map settings     = hdlcFraming(1024U);
        settings["payload_pack_bits"] = gr::Size_t{8};
        settings["payload_bit_order"] = std::string("LsbFirst");
        settings["min_payload_items"] = gr::Size_t{24};

        gr::Graph flow;
        auto&     source = flow.emplaceBlock<FiniteSource<std::uint8_t>>();
        source._data     = wire;
        auto& extractor  = flow.emplaceBlock<DelimiterExtractor<std::uint8_t>>(settings);
        auto& check      = flow.emplaceBlock<CrcCheck>({{"width", gr::Size_t{16}}, {"poly", std::uint64_t{0x1021}}, {"initial_value", std::uint64_t{0xFFFF}}, //
                 {"final_xor", std::uint64_t{0xFFFF}}, {"input_reflected", true}, {"result_reflected", true}, {"crc_byte_order", std::string("little")}});
        auto& passed     = flow.emplaceBlock<RecordSink<std::uint8_t>>();
        auto& failed     = flow.emplaceBlock<RecordSink<std::uint8_t>>();
        expect(flow.connect<"out", "in">(source, extractor).has_value());
        expect(flow.connect<"out", "in">(extractor, check).has_value());
        expect(flow.connect<"ok", "in">(check, passed).has_value());
        expect(flow.connect<"fail", "in">(check, failed).has_value());
        std::vector<Record<std::uint8_t>> clean;
        std::vector<Record<std::uint8_t>> corrupt;
        runGraph(std::move(flow), [&clean, &corrupt, &passed, &failed] {
            clean   = passed._records;
            corrupt = failed._records;
        });

        expect(eq(clean.size(), 2UZ)) << "the clean frames leave by ok, the link carrying frames after the failure";
        expect(eq(corrupt.size(), 1UZ)) << "the corrupted frame leaves by fail";
        if (clean.size() == 2UZ) {
            expect(that % (std::vector<std::uint8_t>(clean[0UZ].signal_values.begin(), std::next(clean[0UZ].signal_values.begin(), 3)) == messages[0UZ]));
            expect(that % (std::vector<std::uint8_t>(clean[1UZ].signal_values.begin(), std::next(clean[1UZ].signal_values.begin(), 3)) == messages[2UZ]));
        }
    };

    "delimiters at every boundary, in all three transparency modes"_test = [] {
        struct Mode {
            const char* transparency;
            gr::Size_t  bitsPerItem;
            const char* shortest;
            const char* longest;
        };
        // the 64-bit bit-stuffing delimiter carries one run of six ones and no zero arriving at exactly five
        static const std::string kLongStuffed = std::string("01111110") + [] {
            std::string tail;
            for (std::size_t i = 0UZ; i < 28UZ; ++i) {
                tail += "01";
            }
            return tail;
        }();
        const Mode kModes[]{{"none", 1U, "1", "01111110"}, {"bit_stuffing", 1U, "01111110", kLongStuffed.c_str()}, {"none", 8U, "11000000", "1100000011000000"}};

        Rng rng{};
        for (const Mode& mode : kModes) {
            for (const char* delimiter : {mode.shortest, mode.longest}) {
                const std::string         text(delimiter);
                const std::size_t         items = text.size() / static_cast<std::size_t>(mode.bitsPerItem);
                std::vector<std::uint8_t> pattern;
                if (mode.bitsPerItem == 1U) {
                    pattern = itemsOf(text);
                } else {
                    for (std::size_t at = 0UZ; at < text.size(); at += 8UZ) {
                        unsigned value = 0U;
                        for (unsigned bit = 0U; bit < 8U; ++bit) {
                            value = (value << 1U) | (text[at + bit] == '1' ? 1U : 0U);
                        }
                        pattern.push_back(static_cast<std::uint8_t>(value));
                    }
                }

                // a delimiter at the very first item, at the very last, and at random positions in between
                std::vector<std::uint8_t> stream;
                stream.reserve(100'000UZ + items);
                stream.insert(stream.end(), pattern.begin(), pattern.end());
                while (stream.size() < 100'000UZ - items) {
                    if (rng.below(64UZ) == 0UZ) {
                        stream.insert(stream.end(), pattern.begin(), pattern.end());
                    } else {
                        stream.push_back(static_cast<std::uint8_t>(rng.next() & (mode.bitsPerItem == 1U ? 1ULL : 0x7FULL)));
                    }
                }
                stream.insert(stream.end(), pattern.begin(), pattern.end());

                for (const std::size_t chunk : {1UZ, 2UZ, 3UZ, 17UZ, 4096UZ}) {
                    const gr::property_map           settings{{"end_delimiter", text}, {"bits_per_item", mode.bitsPerItem}, {"transparency", std::string(mode.transparency)}, {"max_payload_items", gr::Size_t{512}}};
                    DelimiterExtractor<std::uint8_t> block    = make<DelimiterExtractor<std::uint8_t>>(settings);
                    const std::size_t                consumed = feedCounting<std::uint8_t>(block, std::span<const std::uint8_t>(stream), chunk, 512UZ);
                    expect(eq(consumed, stream.size())) << std::format("{} at {} items, chunk {}", mode.transparency, items, chunk);
                }
            }
        }
    };
};

int main() { /* not needed for UT */ }
