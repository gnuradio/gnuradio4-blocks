#include <boost/ut.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/digital/Delimiter.hpp>
#include <gnuradio-4.0/digital/CrcBlocks.hpp>
#include <gnuradio-4.0/digital/DelimiterExtractor.hpp>
#include <gnuradio-4.0/digital/DelimiterFramer.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_delimiter_framer {

using gr::blocks::digital::CrcAppend;
using gr::blocks::digital::CrcCheck;
using gr::blocks::digital::DelimiterExtractor;
using gr::blocks::digital::DelimiterFramer;
using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;

using Record = gr::DataSet<std::uint8_t>;

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

/// One item per '0'/'1' character, which is what a bit-level stream carries.
[[nodiscard]] std::vector<std::uint8_t> itemsOf(std::string_view bits) {
    std::vector<std::uint8_t> items;
    items.reserve(bits.size());
    for (const char character : bits) {
        items.push_back(static_cast<std::uint8_t>(character == '1' ? 1 : 0));
    }
    return items;
}

/// One packet as the chain carries it: a flat byte array with its extent, its name and its own metadata.
[[nodiscard]] Record recordOf(std::vector<std::uint8_t> bytes, gr::property_map meta = {}) {
    Record record;
    record.signal_values = std::move(bytes);
    record.extents.push_back(static_cast<std::int32_t>(record.signal_values.size()));
    record.signal_names.emplace_back("payload");
    record.signal_quantities.emplace_back("");
    record.signal_units.emplace_back("");
    record.meta_information.push_back(std::move(meta));
    record.timing_events.emplace_back();
    return record;
}

[[nodiscard]] const gr::property_map& metaOf(const Record& record) {
    static const gr::property_map kEmpty{};
    return record.meta_information.empty() ? kEmpty : record.meta_information.front();
}

[[nodiscard]] gr::Size_t metaSize(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::numeric_limits<gr::Size_t>::max() : entry->second.value_or(std::numeric_limits<gr::Size_t>::max());
}

[[nodiscard]] std::string metaString(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::string{} : std::string(entry->second.value_or(std::string_view{}));
}

[[nodiscard]] bool metaOk(const Record& record) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type("crc_ok"));
    return entry != map.end() && entry->second.value_or(false);
}

/// The HDLC profile: the flag, bit stuffing, and each payload byte unpacked least significant bit first.
[[nodiscard]] gr::property_map hdlcFraming(gr::Size_t bound) {
    return {{"end_delimiter", std::string("01111110")}, {"transparency", std::string("bit_stuffing")}, {"max_payload_items", bound}, //
        {"payload_pack_bits", gr::Size_t{8}}, {"payload_bit_order", std::string("LsbFirst")}};
}

[[nodiscard]] gr::property_map slipFraming(gr::Size_t bound) {
    return {{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}, {"transparency", std::string("byte_escape")}, //
        {"escape_item", gr::Size_t{0xDB}}, {"escape_map", std::vector<gr::Size_t>{0xDCU, 0xC0U, 0xDDU, 0xDBU}}, {"max_payload_items", bound}};
}

/// RFC 1662's 34-entry control character map, taken from the library constant rather than transcribed again.
[[nodiscard]] std::vector<gr::Size_t> pppEscapeMap() {
    std::vector<gr::Size_t> flat;
    for (const auto& [escaped, original] : gr::digital::pppAsync().escapeMap) {
        flat.push_back(static_cast<gr::Size_t>(escaped));
        flat.push_back(static_cast<gr::Size_t>(original));
    }
    return flat;
}

[[nodiscard]] gr::property_map pppFraming(gr::Size_t bound) {
    return {{"end_delimiter", std::string("01111110")}, {"bits_per_item", gr::Size_t{8}}, {"transparency", std::string("byte_escape")}, //
        {"escape_item", gr::Size_t{0x7D}}, {"escape_map", pppEscapeMap()}, {"max_payload_items", bound}};
}

[[nodiscard]] gr::property_map nmeaFraming(gr::Size_t bound) {
    return {{"end_delimiter", std::string("0000110100001010")}, {"start_delimiter", std::string("00100100")}, {"frame_open", std::string("start")}, //
        {"bits_per_item", gr::Size_t{8}}, {"max_payload_items", bound}};
}

/// What one drive of the framer produced.
struct Framed {
    std::vector<Record> out{};
    std::size_t         consumed = 0UZ;
};

/// Drives the framer over @p records in windows of @p chunk records, with @p room records of output space.
[[nodiscard]] Framed feed(DelimiterFramer& block, std::span<const Record> records, std::size_t chunk = 0UZ, bool connected = true, std::size_t room = 64UZ) {
    Framed              result;
    const std::size_t   stride = chunk == 0UZ ? std::max(records.size(), 1UZ) : chunk;
    std::vector<Record> scratch(room);

    for (std::size_t base = 0UZ; base < records.size();) {
        const std::size_t  count = std::min(stride, records.size() - base);
        InputSpan<Record>  inSpan(records.subspan(base, count), base);
        OutputSpan<Record> outSpan(connected ? std::span<Record>(scratch) : std::span<Record>{}, 0UZ, nullptr, connected);

        std::ignore = block.processBulk(inSpan, outSpan);
        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            result.out.push_back(std::move(scratch[k]));
        }
        result.consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ) {
            break;
        }
        base += inSpan.consumed;
    }
    return result;
}

/// Drives the extractor over a whole wire and keeps what left by `out`.
[[nodiscard]] std::vector<Record> extract(DelimiterExtractor<std::uint8_t>& block, std::span<const std::uint8_t> wire, std::size_t room = 256UZ) {
    std::vector<Record> records;
    std::vector<Record> outScratch(room);
    std::vector<Record> rejectScratch(room);

    for (std::size_t base = 0UZ; base < wire.size();) {
        InputSpan<std::uint8_t> inSpan(wire.subspan(base), base);
        OutputSpan<Record>      outSpan(std::span<Record>(outScratch), 0UZ, nullptr, true);
        OutputSpan<Record>      rejectSpan(std::span<Record>(rejectScratch), 0UZ, nullptr, true);

        std::ignore = block.processBulk(inSpan, outSpan, rejectSpan);
        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            records.push_back(std::move(outScratch[k]));
        }
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && rejectSpan.count == 0UZ) {
            break;
        }
        base += inSpan.consumed;
    }
    return records;
}

struct RecordSource : gr::Block<RecordSource> {
    gr::PortOut<Record, gr::Async> out;
    GR_MAKE_REFLECTABLE(RecordSource, out);
    std::vector<Record> _records{};
    std::size_t         _pos = 0UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(outSpan.size(), _records.size() - _pos);
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = _records[_pos + i];
        }
        outSpan.publish(n);
        _pos += n;
        return _pos == _records.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

/**
 * @brief Flattens each framed record onto a stream, keeping a copy of what it flattened.
 *
 * The copies are what lets one graph run be read from both ends: what the framer emitted and what the extractor made
 * of it. `_flipAt` inverts one wire item on the way past, which is the corrupted frame of the profile test.
 */
struct RecordToStream : gr::Block<RecordToStream> {
    gr::PortIn<Record, gr::Async>        in;
    gr::PortOut<std::uint8_t, gr::Async> out;
    GR_MAKE_REFLECTABLE(RecordToStream, in, out);

    std::vector<Record>       _seen{};
    std::vector<std::uint8_t> _pending{};
    std::size_t               _sent   = 0UZ;
    std::size_t               _index  = 0UZ;
    std::ptrdiff_t            _flipAt = -1;

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) {
        std::size_t consumed = 0UZ;
        if (_sent == _pending.size() && inSpan.size() > 0UZ) {
            _seen.push_back(inSpan[0UZ]);
            _pending = inSpan[0UZ].signal_values;
            _sent    = 0UZ;
            consumed = 1UZ;
        }

        const std::size_t n = std::min(_pending.size() - _sent, outSpan.size());
        for (std::size_t i = 0UZ; i < n; ++i) {
            const std::uint8_t item = _pending[_sent + i];
            outSpan[i]              = (_flipAt >= 0 && static_cast<std::size_t>(_flipAt) == _index + i) ? static_cast<std::uint8_t>(item != 0U ? 0U : 1U) : item;
        }
        _sent += n;
        _index += n;

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(n);
        return consumed == 0UZ && n == 0UZ ? gr::work::Status::INSUFFICIENT_INPUT_ITEMS : gr::work::Status::OK;
    }
};

struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<Record, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);
    std::vector<Record> _records{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const Record& record : inSpan) {
            _records.push_back(record);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/**
 * @brief Runs a graph to completion under the simple scheduler, stopping it rather than hanging if it wedges.
 *
 * @p collect runs while the scheduler still owns the graph, because the references `emplaceBlock` returned point into
 * blocks the scheduler destroys with itself.
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

/// What one loopback run produced: the frames the framer wrote, and the payloads the extractor read back.
struct Loop {
    std::vector<Record> framed{};
    std::vector<Record> received{};
};

/// `records` through `DelimiterFramer` and `DelimiterExtractor` in one graph, under @p framing at both ends.
[[nodiscard]] Loop loopback(const gr::property_map& framing, std::vector<Record> records, std::ptrdiff_t flipAt = -1) {
    gr::Graph flow;
    auto&     source = flow.emplaceBlock<RecordSource>();
    source._records  = std::move(records);
    auto& framer     = flow.emplaceBlock<DelimiterFramer>(framing);
    auto& wire       = flow.emplaceBlock<RecordToStream>();
    wire._flipAt     = flipAt;
    auto& extractor  = flow.emplaceBlock<DelimiterExtractor<std::uint8_t>>(framing);
    auto& sink       = flow.emplaceBlock<RecordSink>();

    boost::ut::expect(flow.connect<"out", "in">(source, framer).has_value());
    boost::ut::expect(flow.connect<"out", "in">(framer, wire).has_value());
    boost::ut::expect(flow.connect<"out", "in">(wire, extractor).has_value());
    boost::ut::expect(flow.connect<"out", "in">(extractor, sink).has_value());

    Loop result;
    runGraph(std::move(flow), [&result, &wire, &sink] {
        result.framed   = wire._seen;
        result.received = sink._records;
    });
    return result;
}

/// The opening and closing delimiter's item counts under @p framing, which the emitted-item identity is measured with.
[[nodiscard]] std::pair<std::size_t, std::size_t> delimiterItems(const gr::property_map& framing) {
    DelimiterFramer   block  = make<DelimiterFramer>(framing);
    const std::size_t ending = block._encoder.config.endItems;
    const std::size_t open   = block._encoder.config.frameOpen == gr::digital::FrameOpen::Start ? block._encoder.config.startItems : (block._encoder.config.frameOpen == gr::digital::FrameOpen::Delimiter ? ending : 0UZ);
    return {open, ending};
}

} // namespace qa_delimiter_framer

using namespace qa_delimiter_framer;

const boost::ut::suite<"delimiter framer"> delimiterFramerTests = [] {
    using namespace boost::ut;

    "the framed HDLC frame reaches the wire item by item"_test = [] {
        // anchor G: the flag, 0x7E least significant bit first with one stuffed zero after its fifth one, 0xFF with
        // one more, then the flag -- 34 bit items, hand-walked and compared one at a time
        const std::vector<Record> records{recordOf({0x7EU, 0xFFU})};
        DelimiterFramer           block  = make<DelimiterFramer>(hdlcFraming(1024U));
        const Framed              framed = feed(block, std::span<const Record>(records));

        expect(eq(framed.out.size(), 1UZ));
        if (framed.out.size() != 1UZ) {
            return;
        }
        const std::vector<std::uint8_t> wanted = itemsOf(std::string("01111110") + "011111010" + "111110111" + "01111110");
        expect(eq(wanted.size(), 34UZ));
        expect(that % (framed.out[0UZ].signal_values == wanted)) << "the wire items, against the hand-walked string";
        expect(eq(static_cast<std::size_t>(framed.out[0UZ].extents.at(0)), 34UZ));
        expect(eq(metaSize(framed.out[0UZ], "stuffing_inserted"), gr::Size_t{2}));
        expect(eq(block.nItemsEmitted, 34ULL));
        expect(eq(block.nStuffedBits, 2ULL));
        expect(eq(block.nRecords, 1ULL));
        expect(eq(block.nForgedDelimiters, 0ULL));

        DelimiterExtractor<std::uint8_t> receiver = make<DelimiterExtractor<std::uint8_t>>(hdlcFraming(1024U));
        const std::vector<Record>        back     = extract(receiver, std::span<const std::uint8_t>(framed.out[0UZ].signal_values));
        expect(eq(back.size(), 1UZ));
        if (back.size() == 1UZ) {
            expect(that % (back[0UZ].signal_values == std::vector<std::uint8_t>{0x7EU, 0xFFU})) << "and the receiver reads the two bytes back out of it";
            expect(eq(metaSize(back[0UZ], "stuffing_removed"), gr::Size_t{2}));
        }
    };

    "the round trip reproduces every payload, and the mirrored identity holds frame by frame"_test = [] {
        struct Framing {
            const char*      name;
            gr::property_map settings;
            bool             restricted; ///< 'none' needs an alphabet that cannot forge the delimiter, which is the protocol's own rule
        };
        const std::vector<Framing> framings{{"hdlc", hdlcFraming(8192U), false}, {"slip", slipFraming(1024U), false}, //
            {"ppp", pppFraming(1024U), false}, {"nmea", nmeaFraming(1024U), true}};

        for (const Framing& framing : framings) {
            Rng                 rng{};
            std::vector<Record> records;
            for (const std::size_t length : {1UZ, 2UZ, 7UZ, 33UZ, 64UZ}) {
                std::vector<std::uint8_t> bytes(length);
                for (std::uint8_t& byte : bytes) {
                    // the restricted alphabet is upper-case letters, which carry neither '$' nor CR nor LF
                    byte = static_cast<std::uint8_t>(framing.restricted ? 0x41U + (rng.next() % 26ULL) : (rng.next() & 0xFFULL));
                }
                records.push_back(recordOf(std::move(bytes), {{"packet", static_cast<gr::Size_t>(records.size())}}));
            }

            const std::vector<Record> sent = records;
            const Loop                run  = loopback(framing.settings, std::move(records));
            expect(eq(run.framed.size(), sent.size())) << framing.name;
            expect(eq(run.received.size(), sent.size())) << framing.name;
            if (run.received.size() != sent.size() || run.framed.size() != sent.size()) {
                continue;
            }

            const auto [opening, closing] = delimiterItems(framing.settings);
            const std::size_t packBits    = framing.settings.contains(gr::property_map::key_type("payload_pack_bits")) ? 8UZ : 1UZ;

            for (std::size_t which = 0UZ; which < sent.size(); ++which) {
                expect(that % (run.received[which].signal_values == sent[which].signal_values)) << std::format("{}: payload {}", framing.name, which);
                expect(eq(metaSize(run.framed[which], "packet"), static_cast<gr::Size_t>(which))) << std::format("{}: the record's own metadata crosses the framer verbatim", framing.name);

                const std::size_t inserted = static_cast<std::size_t>(metaSize(run.framed[which], "stuffing_inserted"));
                const std::size_t emitted  = static_cast<std::size_t>(run.framed[which].extents.at(0));
                const std::size_t payload  = sent[which].signal_values.size() * packBits;
                expect(eq(emitted, opening + closing + payload + inserted)) << std::format("{}: frame {} emits its delimiters, its payload and its insertions", framing.name, which);
                expect(eq(static_cast<std::size_t>(metaSize(run.received[which], "stuffing_removed")), inserted)) << std::format("{}: frame {} is destuffed by exactly what stuffed it", framing.name, which);
            }
        }
    };

    "the HDLC profile carries a frame check sequence both ways"_test = [] {
        const gr::property_map crc{{"width", gr::Size_t{16}}, {"poly", std::uint64_t{0x1021}}, {"initial_value", std::uint64_t{0xFFFF}}, //
            {"final_xor", std::uint64_t{0xFFFF}}, {"input_reflected", true}, {"result_reflected", true}, {"crc_byte_order", std::string("little")}};

        Rng                                    rng{};
        std::vector<std::vector<std::uint8_t>> messages{{0x41U, 0x42U, 0x43U}, std::vector<std::uint8_t>(24UZ, 0xFFU), {}};
        messages[2UZ].resize(40UZ);
        for (std::uint8_t& byte : messages[2UZ]) {
            byte = static_cast<std::uint8_t>(rng.next() & 0xFFULL);
        }

        const auto build = [&messages] {
            std::vector<Record> records;
            for (const std::vector<std::uint8_t>& message : messages) {
                records.push_back(recordOf(message));
            }
            return records;
        };

        const auto profile = [&crc](std::vector<Record> records, std::ptrdiff_t flipAt, std::vector<Record>& clean, std::vector<Record>& corrupt, std::vector<Record>& framed) {
            gr::Graph flow;
            auto&     source           = flow.emplaceBlock<RecordSource>();
            source._records            = std::move(records);
            auto& append               = flow.emplaceBlock<CrcAppend>(crc);
            auto& framer               = flow.emplaceBlock<DelimiterFramer>(hdlcFraming(8192U));
            auto& wire                 = flow.emplaceBlock<RecordToStream>();
            wire._flipAt               = flipAt;
            auto&            extractor = flow.emplaceBlock<DelimiterExtractor<std::uint8_t>>(hdlcFraming(8192U));
            gr::property_map checking  = crc;
            checking["discard_crc"]    = true;
            auto& check                = flow.emplaceBlock<CrcCheck>(checking);
            auto& passed               = flow.emplaceBlock<RecordSink>();
            auto& failed               = flow.emplaceBlock<RecordSink>();

            boost::ut::expect(flow.connect<"out", "in">(source, append).has_value());
            boost::ut::expect(flow.connect<"out", "in">(append, framer).has_value());
            boost::ut::expect(flow.connect<"out", "in">(framer, wire).has_value());
            boost::ut::expect(flow.connect<"out", "in">(wire, extractor).has_value());
            boost::ut::expect(flow.connect<"out", "in">(extractor, check).has_value());
            boost::ut::expect(flow.connect<"ok", "in">(check, passed).has_value());
            boost::ut::expect(flow.connect<"fail", "in">(check, failed).has_value());
            runGraph(std::move(flow), [&clean, &corrupt, &framed, &passed, &failed, &wire] {
                clean   = passed._records;
                corrupt = failed._records;
                framed  = wire._seen;
            });
        };

        std::vector<Record> clean;
        std::vector<Record> corrupt;
        std::vector<Record> framed;
        profile(build(), -1, clean, corrupt, framed);
        expect(eq(clean.size(), messages.size())) << "every frame survives the chain with its check sequence";
        expect(eq(corrupt.size(), 0UZ));
        for (std::size_t which = 0UZ; which < std::min(clean.size(), messages.size()); ++which) {
            expect(metaOk(clean[which])) << std::format("frame {}", which);
            expect(that % (clean[which].signal_values == messages[which])) << std::format("frame {}: byte identical after the check sequence is stripped", which);
        }
        expect(gt(metaSize(framed.at(1UZ), "stuffing_inserted"), gr::Size_t{0})) << "the all-ones payload works the stuffing at its worst-case density";

        // one flipped wire bit, chosen inside the third frame and inside a run of ones short enough that no stuffing
        // decision either side of it changes: the frame keeps its length and fails only its check sequence
        const std::size_t                earlier = framed.at(0UZ).signal_values.size() + framed.at(1UZ).signal_values.size();
        const std::vector<std::uint8_t>& target  = framed.at(2UZ).signal_values;
        std::ptrdiff_t                   flipAt  = -1;
        for (std::size_t i = 9UZ; i + 9UZ < target.size(); ++i) {
            if (target[i] == 0U) {
                continue;
            }
            std::size_t run = 1UZ;
            for (std::size_t back = i; back > 0UZ && target[back - 1UZ] != 0U; --back) {
                ++run;
            }
            for (std::size_t ahead = i + 1UZ; ahead < target.size() && target[ahead] != 0U; ++ahead) {
                ++run;
            }
            if (run <= 4UZ) {
                flipAt = static_cast<std::ptrdiff_t>(earlier + i);
                break;
            }
        }
        expect(gt(flipAt, std::ptrdiff_t{0})) << "a wire item to corrupt was found in the third frame";

        std::vector<Record> stillClean;
        std::vector<Record> failedFrames;
        std::vector<Record> reframed;
        profile(build(), flipAt, stillClean, failedFrames, reframed);
        expect(eq(failedFrames.size(), 1UZ)) << "the corrupted frame leaves by the fail port";
        expect(eq(stillClean.size(), messages.size() - 1UZ)) << "and the link carries the frames either side of it";
        if (failedFrames.size() == 1UZ) {
            expect(!metaOk(failedFrames[0UZ]));
        }
    };

    "a delimiter ending in ones round-trips exactly"_test = [] {
        // the counter the encoder runs is the machine's own, and the machine clears it as the opening delimiter opens
        // the frame; a delimiter whose last item is a one is where the two readings of that rule part company
        const gr::property_map framing{{"end_delimiter", std::string("0111111")}, {"transparency", std::string("bit_stuffing")}, //
            {"abort_ones", gr::Size_t{0}}, {"max_payload_items", gr::Size_t{4096}}};

        Rng                 rng{};
        std::vector<Record> records;
        for (std::size_t which = 0UZ; which < 40UZ; ++which) {
            const std::size_t         length = 1UZ + rng.below(200UZ);
            std::vector<std::uint8_t> bits(length);
            for (std::uint8_t& bit : bits) {
                bit = static_cast<std::uint8_t>(rng.next() & 1ULL);
            }
            records.push_back(recordOf(std::move(bits)));
        }
        // a payload ending in exactly five ones, which the encoder must follow with a stuffed zero or the delimiter's
        // own leading zero is removed and the frame loses its last item
        records.push_back(recordOf({0U, 1U, 1U, 1U, 1U, 1U}));

        const std::vector<Record> sent = records;
        const Loop                run  = loopback(framing, std::move(records));
        expect(eq(run.received.size(), sent.size()));
        if (run.received.size() != sent.size()) {
            return;
        }
        for (std::size_t which = 0UZ; which < sent.size(); ++which) {
            expect(that % (run.received[which].signal_values == sent[which].signal_values)) << std::format("payload {}", which);
        }
        expect(eq(metaSize(run.framed.back(), "stuffing_inserted"), gr::Size_t{1})) << "the insertion after the payload's last item is the one the closing delimiter depends on";
    };

    "the framer refuses what only an encoder can prove, and counts what it drops"_test = [] {
        const auto staged = [](gr::property_map settings) { std::ignore = make<DelimiterFramer>(std::move(settings)); };

        expect(nothrow([&staged] { staged(slipFraming(256U)); })) << "SLIP's map covers the delimiter and the introducer alike";
        expect(nothrow([&staged] { staged(pppFraming(256U)); })) << "and so does PPP's";
        expect(throws([&staged] {
            gr::property_map missing = slipFraming(256U);
            missing["escape_map"]    = std::vector<gr::Size_t>{0xDCU, 0xC0U};
            staged(missing);
        })) << "the introducer has no pair, so a payload carrying it would reach the receiver unescaped";
        expect(throws([&staged] {
            gr::property_map missing = slipFraming(256U);
            missing["escape_map"]    = std::vector<gr::Size_t>{0xDDU, 0xDBU};
            staged(missing);
        })) << "the delimiter has no pair, which a receiver could only warn about";
        expect(throws([&staged] {
            gr::property_map packed     = slipFraming(256U);
            packed["payload_pack_bits"] = gr::Size_t{8};
            staged(packed);
        })) << "the unpack stage produces bit items, so it needs one bit per wire item";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("01111110")}, {"transparency", std::string("bit_stuffing")}, {"stuff_after_ones", gr::Size_t{1}}, {"max_payload_items", gr::Size_t{16}}}); })) << "stuff_after_ones below two";
        expect(throws([&staged] { staged({{"end_delimiter", std::string("0111110")}, {"transparency", std::string("bit_stuffing")}, {"max_payload_items", gr::Size_t{16}}}); })) << "a delimiter the destuffing rule would eat into";

        // an empty record and one above the bound are counted drops, and the record after each of them is framed
        const std::vector<Record> records{recordOf({}), recordOf({0x11U, 0x22U, 0x33U}), recordOf(std::vector<std::uint8_t>(9UZ, 0x44U)), recordOf({0x55U})};
        DelimiterFramer           block  = make<DelimiterFramer>(slipFraming(8U));
        const Framed              framed = feed(block, std::span<const Record>(records));

        expect(eq(framed.out.size(), 2UZ));
        expect(eq(block.nRecordsRefused, 2ULL));
        expect(eq(block.nRecords, 2ULL));
        expect(eq(framed.consumed, records.size())) << "a dropped record is consumed rather than left in the buffer";
        if (framed.out.size() == 2UZ) {
            expect(that % (framed.out[0UZ].signal_values == std::vector<std::uint8_t>{0xC0U, 0x11U, 0x22U, 0x33U, 0xC0U}));
            expect(that % (framed.out[1UZ].signal_values == std::vector<std::uint8_t>{0xC0U, 0x55U, 0xC0U})) << "the record after the oversize one is framed";
        }
    };

    "forgery under 'none' is counted, and the receiver cuts where the count said"_test = [] {
        const gr::property_map    framing{{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}, {"max_payload_items", gr::Size_t{64}}};
        const std::vector<Record> records{recordOf({0x11U, 0xC0U, 0x22U, 0xC0U, 0x33U})};

        DelimiterFramer block  = make<DelimiterFramer>(framing);
        const Framed    framed = feed(block, std::span<const Record>(records));
        expect(eq(framed.out.size(), 1UZ));
        expect(eq(block.nForgedDelimiters, 2ULL)) << "each payload item at which the receiver's register holds the delimiter";
        expect(eq(metaSize(framed.out.at(0UZ), "stuffing_inserted"), gr::Size_t{0})) << "'none' codes nothing, so it inserts nothing";

        DelimiterExtractor<std::uint8_t> receiver = make<DelimiterExtractor<std::uint8_t>>(framing);
        const std::vector<Record>        back     = extract(receiver, std::span<const std::uint8_t>(framed.out.at(0UZ).signal_values));
        expect(eq(back.size(), 3UZ)) << "one frame in, three out: the two forged delimiters cut it where the counter said they would";
        if (back.size() == 3UZ) {
            expect(that % (back[0UZ].signal_values == std::vector<std::uint8_t>{0x11U}));
            expect(that % (back[1UZ].signal_values == std::vector<std::uint8_t>{0x22U}));
            expect(that % (back[2UZ].signal_values == std::vector<std::uint8_t>{0x33U}));
        }
    };

    "the adapter conforms: metadata, publish discipline and chunk independence"_test = [] {
        const std::vector<Record> records{recordOf({0x11U, 0x22U}, {{"packet", gr::Size_t{7}}, {"origin", std::string("qa")}}), //
            recordOf({0xC0U, 0xDBU}, {{"packet", gr::Size_t{8}}}), recordOf({0x33U})};

        // the same records come out whatever the window is, because a record is framed whole or not at all
        std::vector<std::vector<std::uint8_t>> reference;
        for (const std::size_t chunk : {1UZ, 2UZ, 0UZ}) {
            DelimiterFramer block  = make<DelimiterFramer>(slipFraming(64U));
            const Framed    framed = feed(block, std::span<const Record>(records), chunk);
            expect(eq(framed.out.size(), records.size())) << std::format("chunk {}", chunk);
            std::vector<std::vector<std::uint8_t>> produced;
            for (const Record& record : framed.out) {
                produced.push_back(record.signal_values);
            }
            if (reference.empty()) {
                reference = produced;
            } else {
                expect(that % (produced == reference)) << std::format("chunk {}", chunk);
            }
        }

        DelimiterFramer block  = make<DelimiterFramer>(slipFraming(64U));
        const Framed    framed = feed(block, std::span<const Record>(records));
        expect(eq(framed.out.size(), 3UZ));
        if (framed.out.size() == 3UZ) {
            expect(eq(metaSize(framed.out[0UZ], "packet"), gr::Size_t{7})) << "the record's own keys cross verbatim";
            expect(eq(metaString(framed.out[0UZ], "origin"), std::string("qa")));
            expect(eq(metaSize(framed.out[0UZ], "stuffing_inserted"), gr::Size_t{0}));
            expect(eq(metaSize(framed.out[1UZ], "stuffing_inserted"), gr::Size_t{2})) << "both reserved values are escaped";
            expect(that % (framed.out[1UZ].signal_values == std::vector<std::uint8_t>{0xC0U, 0xDBU, 0xDCU, 0xDBU, 0xDDU, 0xC0U}));
            expect(eq(framed.out[0UZ].timing_events.size(), 1UZ));
            expect(eq(framed.out[0UZ].timing_events.at(0).size(), 0UZ)) << "the items are wire items, so an index measured in payload items names none of them";
        }
        expect(eq(block.nRecords, 3ULL));
        expect(eq(block.nEscapedItems, 2ULL));
        expect(eq(block.nItemsEmitted, 4ULL + 6ULL + 3ULL));
        expect(nothrow([&block] { block.stop(); })) << "the report is written once, at stop()";

        // an unconnected output publishes nothing and still consumes, which is the Async contract stated explicitly
        DelimiterFramer silent = make<DelimiterFramer>(slipFraming(64U));
        const Framed    none   = feed(silent, std::span<const Record>(records), 0UZ, false);
        expect(eq(none.out.size(), 0UZ));
        expect(eq(none.consumed, records.size()));
        expect(eq(silent.nRecords, 3ULL)) << "the frames were built and counted; only the publish was skipped";

        // room for fewer records than arrived: the rest stay in the buffer for the next call
        DelimiterFramer     narrow = make<DelimiterFramer>(slipFraming(64U));
        std::vector<Record> scratch(2UZ);
        InputSpan<Record>   inSpan(std::span<const Record>(records), 0UZ);
        OutputSpan<Record>  outSpan(std::span<Record>(scratch), 0UZ, nullptr, true);
        std::ignore = narrow.processBulk(inSpan, outSpan);
        expect(eq(outSpan.count, 2UZ));
        expect(eq(inSpan.consumed, 2UZ)) << "only the records the port had room for were consumed";
    };

    "the two required settings leave the block refusing to start and inert if driven"_test = [] {
        expect(throws([] {
            DelimiterFramer block{}; // nothing staged, so nothing is validated until start()
            block.start();
        })) << "both settings are required and neither has a default";
        expect(throws([] {
            DelimiterFramer block = make<DelimiterFramer>({{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}});
            block.start();
        })) << "a delimiter with no bound";
        expect(throws([] {
            DelimiterFramer block = make<DelimiterFramer>({{"max_payload_items", gr::Size_t{16}}});
            block.start();
        })) << "a bound with no delimiter";

        const std::vector<Record> records{recordOf({0x11U, 0x22U})};
        DelimiterFramer           block = make<DelimiterFramer>({{"max_payload_items", gr::Size_t{16}}});
        std::vector<Record>       scratch(4UZ);
        InputSpan<Record>         inSpan(std::span<const Record>(records), 0UZ);
        OutputSpan<Record>        outSpan(std::span<Record>(scratch), 0UZ, nullptr, true);
        const gr::work::Status    status = block.processBulk(inSpan, outSpan);
        expect(status == gr::work::Status::ERROR);
        expect(eq(inSpan.consumed, 0UZ));
        expect(eq(outSpan.count, 0UZ));
    };

    "an idle framer answers that it waits for input, not progress"_test = [] {
        DelimiterFramer block = make<DelimiterFramer>(hdlcFraming(1024U));
        block.start();
        std::vector<Record> room(2UZ);
        InputSpan<Record>   inSpan(std::span<const Record>{}, 0UZ);
        OutputSpan<Record>  outSpan(std::span<Record>(room), 0UZ, nullptr, true);
        expect(block.processBulk(inSpan, outSpan) == gr::work::Status::INSUFFICIENT_INPUT_ITEMS);
    };
};

int main() { /* not needed for UT */ }
