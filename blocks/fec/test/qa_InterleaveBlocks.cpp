#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <print>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/fec/ConvBlocks.hpp>
#include <gnuradio-4.0/fec/InterleaveBlocks.hpp>

/*
 * The interleaver pair is exercised through the scheduler rather than by calling it directly,
 * because what it promises is a record contract: the framed kinds turn one record into one record,
 * the convolutional kind carries its delay lines across the record boundary, and a record that is
 * not a whole number of frames is counted rather than absorbed.
 *
 * The kernel's own index arithmetic is established beside it and is not restated here. What these
 * tests pin is the layer this module owns: the settings that select a family and its refusals, the
 * record arithmetic, the rule that decides which record's facts a convolutional output inherits,
 * the counters, and — the reason the family exists — that an interleaver placed around a channel
 * burst lets a convolutional decode recover where the same burst without it does not.
 */
namespace qa_interleave_blocks {

using gr::blocks::fec::ConvEncode;
using gr::blocks::fec::ViterbiDecode;

using Deinterleave = gr::blocks::fec::Deinterleave<std::uint8_t>;
using Interleave   = gr::blocks::fec::Interleave<std::uint8_t>;

using Record = gr::DataSet<std::uint8_t>;

//! The classic constraint length 7 rate 1/2 code, over which the interleaver's pairing is measured.
[[nodiscard]] gr::property_map classic() { return {{"constraint_length", gr::Size_t{7U}}, {"polynomials", std::vector<gr::Size_t>{0171U, 0133U}}}; }

std::uint64_t rng = 0x9E3779B97F4A7C15ULL;

[[nodiscard]] std::uint64_t next() {
    rng = rng * 6364136223846793005ULL + 1442695040888963407ULL;
    return rng >> 17U;
}

//! @p count seeded random bits.
[[nodiscard]] std::vector<std::uint8_t> randomBits(std::size_t count) {
    std::vector<std::uint8_t> bits(count);
    for (std::size_t i = 0UZ; i < count; ++i) {
        bits[i] = static_cast<std::uint8_t>(next() & 1ULL);
    }
    return bits;
}

//! @p count seeded distinct item values, so a permutation that moves an item is visible.
[[nodiscard]] std::vector<std::uint8_t> rampItems(std::size_t count) {
    std::vector<std::uint8_t> items(count);
    for (std::size_t i = 0UZ; i < count; ++i) {
        items[i] = static_cast<std::uint8_t>((i * 37UZ + 11UZ) & 0xFFUZ);
    }
    return items;
}

//! A record of @p values, shaped as a record producer shapes one.
[[nodiscard]] Record record(std::vector<std::uint8_t> values, gr::property_map meta = {}) {
    Record r;
    r.signal_values = std::move(values);
    r.extents.push_back(static_cast<std::int32_t>(r.signal_values.size()));
    r.signal_names.emplace_back("fec");
    r.timing_events.resize(1UZ);
    if (!meta.empty()) {
        r.meta_information.push_back(std::move(meta));
    }
    return r;
}

//! @p values cut into records of @p chunk items each, the last one short where it does not divide.
[[nodiscard]] std::vector<Record> chunked(const std::vector<std::uint8_t>& values, std::size_t chunk) {
    std::vector<Record> records;
    for (std::size_t at = 0UZ; at < values.size(); at += chunk) {
        const std::size_t take = std::min(chunk, values.size() - at);
        records.push_back(record(std::vector<std::uint8_t>(values.begin() + static_cast<std::ptrdiff_t>(at), values.begin() + static_cast<std::ptrdiff_t>(at + take))));
    }
    return records;
}

//! The items of @p records, end to end, which is what a stream-shaped family is judged on.
[[nodiscard]] std::vector<std::uint8_t> flattened(const std::vector<Record>& records) {
    std::vector<std::uint8_t> values;
    for (const Record& r : records) {
        values.insert(values.end(), r.signal_values.begin(), r.signal_values.end());
    }
    return values;
}

//! A record's count under @p key, with the key's absence reported rather than defaulted away.
[[nodiscard]] gr::Size_t metaCount(const Record& r, const char* key) {
    boost::ut::expect(!r.meta_information.empty()) << "the record carries a metadata map";
    if (r.meta_information.empty()) {
        return 0U;
    }
    const auto& map   = r.meta_information[0UZ];
    const auto  entry = map.find(gr::property_map::key_type(key));
    boost::ut::expect(entry != map.end()) << key;
    return entry == map.end() ? 0U : entry->second.value_or(gr::Size_t{0U});
}

template<typename T>
struct RecordSourceOf : gr::Block<RecordSourceOf<T>> {
    gr::PortOut<gr::DataSet<T>, gr::Async> out;
    GR_MAKE_REFLECTABLE(RecordSourceOf, out);
    std::vector<gr::DataSet<T>>    _records;
    std::size_t                    _pos{0UZ};
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
using RecordSource = RecordSourceOf<std::uint8_t>;

template<typename T>
struct RecordSinkOf : gr::Block<RecordSinkOf<T>> {
    gr::PortIn<gr::DataSet<T>, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSinkOf, in);
    std::vector<gr::DataSet<T>>    _records;
    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const auto& r : inSpan) {
            _records.push_back(r);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};
using RecordSink = RecordSinkOf<std::uint8_t>;

//! Flips the bits at fixed item positions of every record passing through, so the damage a chain
//! sees is the same on every run and on every schedule.
struct BitFlipper : gr::Block<BitFlipper> {
    gr::PortIn<Record, gr::Async>  in;
    gr::PortOut<Record, gr::Async> out;
    GR_MAKE_REFLECTABLE(BitFlipper, in, out);
    std::vector<std::size_t>       _positions;
    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(inSpan.size(), outSpan.size());
        for (std::size_t i = 0UZ; i < n; ++i) {
            Record damaged = inSpan[i];
            for (const std::size_t p : _positions) {
                if (p < damaged.signal_values.size()) {
                    damaged.signal_values[p] = static_cast<std::uint8_t>(damaged.signal_values[p] ^ 1U);
                }
            }
            outSpan[i] = std::move(damaged);
        }
        std::ignore = inSpan.consume(n);
        outSpan.publish(n);
        return n == 0UZ ? gr::work::Status::INSUFFICIENT_INPUT_ITEMS : gr::work::Status::OK;
    }
};

//! Run @p flow to completion under the simple scheduler, then call @p after while its blocks are
//! still alive. A run that has not finished inside the timeout is stopped and reported.
template<typename After>
void runFlow(gr::Graph&& flow, After&& after) {
    gr::scheduler::Simple<> scheduler;
    boost::ut::expect(scheduler.exchange(std::move(flow)).has_value());
    std::atomic<bool> done{false};
    std::thread       runner([&scheduler, &done] {
        std::ignore = scheduler.runAndWait();
        done        = true;
    });
    const auto        start = std::chrono::steady_clock::now();
    while (!done.load() && std::chrono::steady_clock::now() - start < std::chrono::seconds(30)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!done.load()) {
        scheduler.requestStop();
    }
    runner.join();
    boost::ut::expect(done.load());
    after();
}

//! Run @p records through one block configured by @p settings, handing the block to @p inspect
//! before the graph is torn down.
template<typename TBlock, typename Inspect>
[[nodiscard]] std::vector<Record> runBlock(gr::property_map settings, std::vector<Record> records, Inspect&& inspect) {
    gr::Graph flow;
    auto&     src = flow.emplaceBlock<RecordSource>();
    src._records  = std::move(records);
    auto& mover   = flow.emplaceBlock<TBlock>(std::move(settings));
    auto& sink    = flow.emplaceBlock<RecordSink>();
    boost::ut::expect(flow.connect<"out", "in">(src, mover).has_value());
    boost::ut::expect(flow.connect<"out", "in">(mover, sink).has_value());

    std::vector<Record> published;
    runFlow(std::move(flow), [&] {
        inspect(mover);
        published = std::move(sink._records);
    });
    return published;
}

template<typename TBlock>
[[nodiscard]] std::vector<Record> runBlock(gr::property_map settings, std::vector<Record> records) {
    return runBlock<TBlock>(std::move(settings), std::move(records), [](const TBlock&) {});
}

//! Run @p records through an Interleave and a Deinterleave under the same @p settings.
[[nodiscard]] std::vector<Record> roundTrip(gr::property_map settings, std::vector<Record> records) {
    gr::Graph flow;
    auto&     src = flow.emplaceBlock<RecordSource>();
    src._records  = std::move(records);
    auto& forward = flow.emplaceBlock<Interleave>(settings);
    auto& back    = flow.emplaceBlock<Deinterleave>(std::move(settings));
    auto& sink    = flow.emplaceBlock<RecordSink>();
    boost::ut::expect(flow.connect<"out", "in">(src, forward).has_value());
    boost::ut::expect(flow.connect<"out", "in">(forward, back).has_value());
    boost::ut::expect(flow.connect<"out", "in">(back, sink).has_value());

    std::vector<Record> published;
    runFlow(std::move(flow), [&] { published = std::move(sink._records); });
    return published;
}

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

[[nodiscard]] gr::property_map blockKind(gr::Size_t rows, gr::Size_t cols) { return {{"kind", std::string("block")}, {"rows", rows}, {"cols", cols}}; }
[[nodiscard]] gr::property_map convolutionalKind(gr::Size_t branches, gr::Size_t unitDelay) { return {{"kind", std::string("convolutional")}, {"branches", branches}, {"unit_delay", unitDelay}}; }

/*!
 * @brief The rectangular interleaver's table built by writing a `rows` by `cols` array row-major
 * and reading it column-major, which is the discipline the standards state rather than the closed
 * index form the kernel evaluates.
 *
 * The cross-check rests on the two routes being independent: this one moves items through
 * an array exactly as a published figure describes, and the kernel's evaluates `(j mod rows) * cols
 * + j / rows`. A table that reproduces the kernel's output item for item says the closed form is
 * the standard's rule and not merely a self-consistent one.
 */
[[nodiscard]] std::vector<gr::Size_t> rectangleTable(std::size_t rows, std::size_t cols) {
    std::vector<std::vector<std::size_t>> grid(rows, std::vector<std::size_t>(cols, 0UZ));
    std::size_t                           at = 0UZ;
    for (std::size_t r = 0UZ; r < rows; ++r) {
        for (std::size_t c = 0UZ; c < cols; ++c) {
            grid[r][c] = at++;
        }
    }
    std::vector<gr::Size_t> table;
    table.reserve(rows * cols);
    for (std::size_t c = 0UZ; c < cols; ++c) {
        for (std::size_t r = 0UZ; r < rows; ++r) {
            table.push_back(static_cast<gr::Size_t>(grid[r][c]));
        }
    }
    return table;
}

//! Items of @p a and @p b that differ, which is the only measure the burst-pairing case makes.
[[nodiscard]] std::size_t differences(const std::vector<std::uint8_t>& a, const std::vector<std::uint8_t>& b) {
    boost::ut::expect(boost::ut::eq(a.size(), b.size()));
    std::size_t count = 0UZ;
    for (std::size_t i = 0UZ; i < std::min(a.size(), b.size()); ++i) {
        count += (a[i] != b[i]) ? 1UZ : 0UZ;
    }
    return count;
}

} // namespace qa_interleave_blocks

using namespace qa_interleave_blocks;

int main() {
    using namespace boost::ut;

    "the family settings are required and are checked before anything runs"_test = [] {
        expect(throws([] {
            Interleave block(gr::property_map{{"kind", std::string("")}}); // the empty spelling is the default, so the batch moves nothing and the refusal comes at start
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
            block.start();
        })) << "no family is the default";
        expect(throws([] { std::ignore = make<Interleave>({{"kind", std::string("rectangular")}}); })) << "a family this module does not carry";
        expect(throws([] { std::ignore = make<Interleave>(blockKind(0U, 8U)); })) << "a rectangle needs a row";
        expect(throws([] { std::ignore = make<Deinterleave>(blockKind(8U, 0U)); })) << "a rectangle needs a column";
        expect(throws([] { std::ignore = make<Interleave>(convolutionalKind(1U, 1U)); })) << "below two branches there is nothing to interleave";
        expect(throws([] { std::ignore = make<Interleave>(convolutionalKind(4U, 0U)); })) << "a branch step of no cells is no delay";
        expect(throws([] { std::ignore = make<Interleave>({{"kind", std::string("permutation")}}); })) << "the permutation kind has no default table";
        expect(throws([] { std::ignore = make<Deinterleave>({{"kind", std::string("permutation")}, {"table", std::vector<gr::Size_t>{0U, 2U, 2U}}}); })) << "a repeated index is not a permutation";
        expect(throws([] { std::ignore = make<Interleave>({{"kind", std::string("permutation")}, {"table", std::vector<gr::Size_t>{0U, 3U, 1U}}}); })) << "an index outside the frame";
        expect(throws([] { std::ignore = make<Interleave>({{"kind", std::string("block")}, {"rows", gr::Size_t{4U}}, {"cols", gr::Size_t{5U}}, {"fill_value", gr::Size_t{256U}}}); })) << "a fill value wider than an item";
        expect(nothrow([] { std::ignore = make<Interleave>(blockKind(4U, 5U)); })) << "a rectangle is a family";
        expect(nothrow([] { std::ignore = make<Deinterleave>(convolutionalKind(4U, 3U)); })) << "branches and a step are a family";
    };

    // Framed kinds: an interleave and its deinterleave reproduce the frame exactly,
    // over a grid of shapes including the degenerate single row and the non-square rectangles.
    "the framed kinds invert exactly, over a grid of shapes"_test = [] {
        struct Shape {
            gr::Size_t rows;
            gr::Size_t cols;
        };
        constexpr std::array<Shape, 6UZ> shapes{{{1U, 16U}, {16U, 1U}, {4U, 4U}, {3U, 17U}, {17U, 3U}, {8U, 20U}}};
        for (const Shape& shape : shapes) {
            const std::size_t               frame = static_cast<std::size_t>(shape.rows) * static_cast<std::size_t>(shape.cols);
            const std::vector<std::uint8_t> items = rampItems(frame * 3UZ);

            const std::vector<Record> out = roundTrip(blockKind(shape.rows, shape.cols), {record(items)});
            expect(eq(out.size(), 1UZ)) << "rows" << shape.rows << "cols" << shape.cols;
            if (!out.empty()) {
                expect(std::ranges::equal(out[0UZ].signal_values, items)) << "rows" << shape.rows << "cols" << shape.cols;
            }

            // The same frame through the permutation kind under the rectangle's own table.
            const gr::property_map    table   = {{"kind", std::string("permutation")}, {"table", rectangleTable(shape.rows, shape.cols)}};
            const std::vector<Record> byTable = roundTrip(table, {record(items)});
            expect(eq(byTable.size(), 1UZ)) << "table rows" << shape.rows;
            if (!byTable.empty()) {
                expect(std::ranges::equal(byTable[0UZ].signal_values, items)) << "table rows" << shape.rows;
            }
        }
    };

    // The convolutional kind: the round trip reproduces the stream after exactly
    // B * (B - 1) * M items, and those first items are the stated fill. The delay identity and the
    // inverse identity are one assertion, because the second is only true at the first.
    "the convolutional kind inverts after exactly B * (B - 1) * M items"_test = [] {
        struct Shape {
            gr::Size_t branches;
            gr::Size_t unitDelay;
        };
        constexpr std::array<Shape, 4UZ> shapes{{{2U, 1U}, {3U, 1U}, {4U, 3U}, {8U, 2U}}};
        for (const Shape& shape : shapes) {
            const std::size_t               latency = static_cast<std::size_t>(shape.branches) * static_cast<std::size_t>(shape.branches - 1U) * static_cast<std::size_t>(shape.unitDelay);
            const std::size_t               items   = latency * 4UZ + 137UZ;
            const std::vector<std::uint8_t> stream  = rampItems(items);

            gr::property_map settings = convolutionalKind(shape.branches, shape.unitDelay);
            settings["fill_value"]    = gr::Size_t{0xA5U};

            const std::vector<std::uint8_t> out = flattened(roundTrip(settings, chunked(stream, 29UZ)));
            expect(eq(out.size(), items)) << "B" << shape.branches << "M" << shape.unitDelay;
            if (out.size() != items) {
                continue;
            }
            for (std::size_t i = 0UZ; i < latency; ++i) {
                expect(eq(out[i], std::uint8_t{0xA5U})) << "B" << shape.branches << "M" << shape.unitDelay << "fill at" << i;
            }
            for (std::size_t i = latency; i < items; ++i) {
                expect(eq(out[i], stream[i - latency])) << "B" << shape.branches << "M" << shape.unitDelay << "item" << i;
            }
        }
    };

    /*
     * The pairing the family exists for. One pinned scene: the classic (171, 133)
     * K = 7 rate 1/2 code over 246 information bits, so a terminated frame is 504 coded bits and
     * a 24 x 21 rectangle is exactly one frame. A burst of 24 adjacent flips is placed at the
     * same transmitted positions on both legs.
     *
     * The rectangle's arithmetic decides the scene. Two adjacent transmitted positions come from
     * coded positions `cols` apart, so a burst no longer than `rows` arrives at the decoder as
     * that many single errors 21 coded bits — ten and a half trellis steps — apart, which the
     * code absorbs. Without the interleaver the same 24 flips arrive whole: twelve consecutive
     * steps with both coded bits wrong, past the code's free distance and past the register's
     * memory, and the decoder cannot bridge them.
     */
    "an interleaver around a burst lets the decode recover where the bare code does not"_test = [] {
        constexpr std::size_t           infoBits  = 246UZ;
        constexpr std::size_t           codedBits = (infoBits + 6UZ) * 2UZ;
        constexpr std::size_t           burst     = 24UZ;
        constexpr std::size_t           burstAt   = 200UZ;
        const std::vector<std::uint8_t> info      = randomBits(infoBits);
        static_assert(codedBits == 504UZ);

        std::vector<std::size_t> positions(burst);
        for (std::size_t i = 0UZ; i < burst; ++i) {
            positions[i] = burstAt + i;
        }

        const auto spread = [&info, &positions] {
            gr::Graph flow;
            auto&     src    = flow.emplaceBlock<RecordSource>();
            src._records     = {record(info)};
            auto& coder      = flow.emplaceBlock<ConvEncode>(classic());
            auto& forward    = flow.emplaceBlock<Interleave>(blockKind(24U, 21U));
            auto& noise      = flow.emplaceBlock<BitFlipper>();
            noise._positions = positions;
            auto& back       = flow.emplaceBlock<Deinterleave>(blockKind(24U, 21U));
            auto& decoder    = flow.emplaceBlock<ViterbiDecode>(classic());
            auto& sink       = flow.emplaceBlock<RecordSink>();
            expect(flow.connect<"out", "in">(src, coder).has_value());
            expect(flow.connect<"out", "in">(coder, forward).has_value());
            expect(flow.connect<"out", "in">(forward, noise).has_value());
            expect(flow.connect<"out", "in">(noise, back).has_value());
            expect(flow.connect<"out", "in">(back, decoder).has_value());
            expect(flow.connect<"out", "in">(decoder, sink).has_value());
            std::vector<Record> out;
            runFlow(std::move(flow), [&] { out = std::move(sink._records); });
            return out;
        }();

        const auto bare = [&info, &positions] {
            gr::Graph flow;
            auto&     src    = flow.emplaceBlock<RecordSource>();
            src._records     = {record(info)};
            auto& coder      = flow.emplaceBlock<ConvEncode>(classic());
            auto& noise      = flow.emplaceBlock<BitFlipper>();
            noise._positions = positions;
            auto& decoder    = flow.emplaceBlock<ViterbiDecode>(classic());
            auto& sink       = flow.emplaceBlock<RecordSink>();
            expect(flow.connect<"out", "in">(src, coder).has_value());
            expect(flow.connect<"out", "in">(coder, noise).has_value());
            expect(flow.connect<"out", "in">(noise, decoder).has_value());
            expect(flow.connect<"out", "in">(decoder, sink).has_value());
            std::vector<Record> out;
            runFlow(std::move(flow), [&] { out = std::move(sink._records); });
            return out;
        }();

        expect(eq(spread.size(), 1UZ));
        expect(eq(bare.size(), 1UZ));
        if (spread.size() != 1UZ || bare.size() != 1UZ) {
            return;
        }
        const std::size_t spreadErrors = differences(spread[0UZ].signal_values, info);
        const std::size_t bareErrors   = differences(bare[0UZ].signal_values, info);
        std::println("the pinned scene: {} information bit errors with the interleaver, {} without it", spreadErrors, bareErrors);
        expect(eq(spreadErrors, 0UZ)) << "24 single errors 21 bits apart are inside the code's budget";
        expect(gt(bareErrors, 0UZ)) << "24 adjacent errors are past the code's free distance";
        expect(lt(spreadErrors, bareErrors)) << "the interleaver is the difference between the two legs";
    };

    /*
     * The cross-check oracle. No DVB-T inner interleaver rule is stated anywhere here, so the
     * oracle is the AO-40 telemetry interleaver's rectangle, whose published
     * geometry is arithmetic: with `rows = stride` and `cols = sync items`, an item written at
     * transmitted position `i * stride` deinterleaves to index `i`. Both AO-40 forms are checked,
     * and the rectangle's map is reproduced through the permutation kind from a table built by
     * moving items through an array rather than by evaluating the kernel's index form.
     */
    "the AO-40 rectangle reproduces through the permutation route and places the sync word"_test = [] {
        struct Form {
            const char* name;
            gr::Size_t  syncItems;
            gr::Size_t  stride;
            std::size_t frameItems; //!< the survey's own figure for the form, checked against the product
        };
        constexpr std::array<Form, 2UZ> forms{{{"long", 65U, 80U, 5200UZ}, {"short", 52U, 51U, 2652UZ}}};
        for (const Form& form : forms) {
            const std::size_t frame = static_cast<std::size_t>(form.syncItems) * static_cast<std::size_t>(form.stride);
            expect(eq(frame, form.frameItems)) << form.name;

            // The sync word is the first `syncItems` items of the deinterleaved frame.
            std::vector<std::uint8_t> frameItems(frame, 0U);
            for (std::size_t i = 0UZ; i < form.syncItems; ++i) {
                frameItems[i] = 1U;
            }

            const gr::property_map    rectangle = blockKind(form.stride, form.syncItems);
            const std::vector<Record> sent      = runBlock<Interleave>(rectangle, {record(frameItems)});
            expect(eq(sent.size(), 1UZ)) << form.name;
            if (sent.empty()) {
                continue;
            }
            for (std::size_t p = 0UZ; p < frame; ++p) {
                const bool onTheStride = (p % form.stride) == 0UZ && (p / form.stride) < form.syncItems;
                expect(eq(sent[0UZ].signal_values[p], static_cast<std::uint8_t>(onTheStride ? 1U : 0U))) << form.name << "transmitted position" << p;
            }

            // The same frame through the permutation kind under the array-built table.
            const gr::property_map    byTable  = {{"kind", std::string("permutation")}, {"table", rectangleTable(form.stride, form.syncItems)}};
            const std::vector<Record> alsoSent = runBlock<Interleave>(byTable, {record(frameItems)});
            expect(eq(alsoSent.size(), 1UZ)) << form.name;
            if (!alsoSent.empty()) {
                expect(std::ranges::equal(alsoSent[0UZ].signal_values, sent[0UZ].signal_values)) << form.name << "the two routes state one map";
            }
        }
    };

    // First part: the framed kinds carry a record's facts verbatim, since an
    // interleaver has no status of its own to write over them.
    "a framed record's metadata and signal name cross verbatim"_test = [] {
        const std::vector<std::uint8_t> items = rampItems(20UZ);
        gr::property_map                meta{{"corrected_errors", gr::Size_t{3U}}, {"who", std::string("earlier stage")}};
        const std::vector<Record>       out = runBlock<Interleave>(blockKind(4U, 5U), {record(items, std::move(meta))});
        expect(eq(out.size(), 1UZ));
        if (out.empty()) {
            return;
        }
        expect(eq(metaCount(out[0UZ], "corrected_errors"), gr::Size_t{3U})) << "an interleaver corrects nothing and reports nothing";
        const auto& map = out[0UZ].meta_information[0UZ];
        const auto  who = map.find(gr::property_map::key_type("who"));
        expect(who != map.end());
        if (who != map.end()) {
            expect(eq(who->second.value_or(std::string{}), std::string("earlier stage")));
        }
        expect(map.find(gr::property_map::key_type("uncorrectable_errors")) == map.end()) << "there is no refusal to report";
        expect(eq(out[0UZ].signal_names.size(), 1UZ));
        expect(eq(out[0UZ].extents.size(), 1UZ));
        expect(eq(out[0UZ].extents[0UZ], static_cast<std::int32_t>(items.size())));
    };

    /*
     * Second part: which record a convolutional output inherits its facts from. The
     * span opening at absolute position `p` opens on branch `p mod B`, whose items entered the
     * lines `(p mod B) * M * B` items earlier, and the record holding that item is the one named.
     * Three-item records over B = 4, M = 1 walk the commutator through every branch, so the
     * inherited sequence is a fingerprint of the rule rather than of one arithmetic accident.
     */
    "a convolutional output carries the facts of the record its span begins in"_test = [] {
        constexpr std::size_t                     records = 10UZ;
        constexpr std::array<gr::Size_t, records> expected{{0U, 1U, 2U, 1U, 4U, 1U, 3U, 5U, 8U, 5U}};
        std::vector<Record>                       sources;
        for (std::size_t r = 0UZ; r < records; ++r) {
            sources.push_back(record(rampItems(3UZ), {{"seq", static_cast<gr::Size_t>(r)}}));
        }

        const std::vector<Record> out = runBlock<Interleave>(convolutionalKind(4U, 1U), std::move(sources));
        expect(eq(out.size(), records));
        if (out.size() != records) {
            return;
        }
        for (std::size_t r = 0UZ; r < records; ++r) {
            expect(eq(metaCount(out[r], "seq"), expected[r])) << "output record" << r;
        }
    };

    // Third part: a record that is not a whole number of frames is dropped, counted
    // and stated, and the record that follows is interleaved normally.
    "a misaligned record is a counted, stated drop"_test = [] {
        const std::vector<std::uint8_t> whole   = rampItems(40UZ);
        const std::vector<std::uint8_t> partial = rampItems(23UZ);

        const std::vector<Record> out = runBlock<Interleave>(blockKind(4U, 5U), {record(partial), record(whole), record({})}, [](const Interleave& block) {
            expect(eq(block.nRecordsRefused, std::uint64_t{2ULL})) << "the short record and the empty one";
            expect(eq(block.nRecords, std::uint64_t{1ULL}));
            expect(eq(block.nItems, std::uint64_t{40ULL}));
        });
        expect(eq(out.size(), 1UZ));
        if (!out.empty()) {
            expect(eq(out[0UZ].signal_values.size(), 40UZ));
        }

        // The output window changes what a frame emits, not what a record must hold: a record that
        // is not whole frames is refused as before, and each accepted frame emits the window alone.
        gr::property_map windowed = blockKind(4U, 5U);
        windowed["output_offset"] = gr::Size_t{2U};
        windowed["output_length"] = gr::Size_t{12U};

        const std::vector<Record> back = runBlock<Deinterleave>(windowed, {record(rampItems(23UZ)), record(rampItems(40UZ))}, [](const Deinterleave& block) {
            expect(eq(block.nRecordsRefused, std::uint64_t{1ULL})) << "23 items is not a whole number of 20-item frames";
            expect(eq(block.nRecords, std::uint64_t{1ULL}));
            expect(eq(block.nItems, std::uint64_t{24ULL})) << "two frames keep 12 items each";
        });
        expect(eq(back.size(), 1UZ));
        if (!back.empty()) {
            expect(eq(back[0UZ].signal_values.size(), 24UZ));
        }
    };

    // The window addresses the DEINTERLEAVED order: interleave a frame of distinct items whole,
    // deinterleave with a window, and the emitted items are the original frame's [offset, offset+length).
    "the output window emits the stated slice of each deinterleaved frame"_test = [] {
        const std::vector<std::uint8_t> items = rampItems(20UZ);
        const std::vector<Record>       sent  = runBlock<Interleave>(blockKind(4U, 5U), {record(items)});
        expect(eq(sent.size(), 1UZ));

        gr::property_map windowed        = blockKind(4U, 5U);
        windowed["output_offset"]        = gr::Size_t{7U};
        windowed["output_length"]        = gr::Size_t{9U};
        const std::vector<Record> sliced = runBlock<Deinterleave>(windowed, sent);
        expect(eq(sliced.size(), 1UZ));
        if (!sliced.empty()) {
            const std::vector<std::uint8_t> want(items.begin() + 7, items.begin() + 16);
            expect(std::ranges::equal(sliced[0UZ].signal_values, want)) << "items [7, 16) of the deinterleaved frame";
            expect(eq(sliced[0UZ].extents.size(), 1UZ));
            if (!sliced[0UZ].extents.empty()) {
                expect(eq(sliced[0UZ].extents[0UZ], std::int32_t{9})) << "the extent names the emitted length";
            }
        }

        // a length of zero reads to the end of the frame, which is how a sync-word prefix is dropped
        gr::property_map tail            = blockKind(4U, 5U);
        tail["output_offset"]            = gr::Size_t{15U};
        const std::vector<Record> tailed = runBlock<Deinterleave>(tail, sent);
        expect(eq(tailed.size(), 1UZ));
        if (!tailed.empty()) {
            const std::vector<std::uint8_t> want(items.begin() + 15, items.end());
            expect(std::ranges::equal(tailed[0UZ].signal_values, want)) << "items [15, 20) of the deinterleaved frame";
        }

        // and the window's refusals fire where it has no meaning
        expect(throws([] {
            std::ignore = make<Deinterleave>([] {
                auto s             = blockKind(4U, 5U);
                s["output_offset"] = gr::Size_t{20U};
                return s;
            }());
        })) << "an offset past the frame";
        expect(throws([] {
            std::ignore = make<Deinterleave>([] {
                auto s             = blockKind(4U, 5U);
                s["output_offset"] = gr::Size_t{5U};
                s["output_length"] = gr::Size_t{16U};
                return s;
            }());
        })) << "a window past the frame end";
        expect(throws([] {
            std::ignore = make<Deinterleave>([] {
                auto s             = convolutionalKind(4U, 3U);
                s["output_offset"] = gr::Size_t{1U};
                return s;
            }());
        })) << "the stream-shaped kind has no frame to window";
    };

    // The framed kinds are addressing arithmetic, so a float record crosses them value-for-value;
    // the convolutional kind carries bytes only and refuses a float block at configure.
    "float records cross the framed kinds and the convolutional kind refuses them"_test = [] {
        using FloatInterleave   = gr::blocks::fec::Interleave<float>;
        using FloatDeinterleave = gr::blocks::fec::Deinterleave<float>;

        std::vector<float> values(20UZ);
        for (std::size_t i = 0UZ; i < values.size(); ++i) {
            values[i] = 0.25f * static_cast<float>(i) - 2.0f;
        }
        gr::DataSet<float> soft;
        soft.signal_values = values;
        soft.extents.push_back(20);
        soft.signal_names.emplace_back("fec");
        soft.timing_events.resize(1UZ);

        gr::Graph flow;
        auto&     src = flow.emplaceBlock<RecordSourceOf<float>>();
        src._records  = {soft};
        auto& forward = flow.emplaceBlock<FloatInterleave>(blockKind(4U, 5U));
        auto& back    = flow.emplaceBlock<FloatDeinterleave>(blockKind(4U, 5U));
        auto& sink    = flow.emplaceBlock<RecordSinkOf<float>>();
        expect(flow.connect<"out", "in">(src, forward).has_value());
        expect(flow.connect<"out", "in">(forward, back).has_value());
        expect(flow.connect<"out", "in">(back, sink).has_value());
        std::vector<gr::DataSet<float>> published;
        runFlow(std::move(flow), [&] { published = std::move(sink._records); });

        expect(eq(published.size(), 1UZ));
        if (!published.empty()) {
            expect(std::ranges::equal(published[0UZ].signal_values, values)) << "the float frame inverts exactly";
        }

        expect(nothrow([] { std::ignore = make<FloatInterleave>({{"kind", std::string("permutation")}, {"table", std::vector<gr::Size_t>{2U, 0U, 1U}}}); })) << "a float permutation is a family";
        expect(throws([] { std::ignore = make<FloatInterleave>(convolutionalKind(4U, 3U)); })) << "a float convolutional interleaver is refused";
        expect(throws([] { std::ignore = make<FloatDeinterleave>(convolutionalKind(4U, 3U)); })) << "and so is its inverse";
    };

    /*
     * The convolutional family's state crosses the record boundary, so the same stream
     * cut three different ways interleaves to the same items. That is the property that makes the
     * record boundary transparent, and it is asserted on the stream rather than on the records.
     */
    "the convolutional kind gives the same stream whatever the record split"_test = [] {
        const std::vector<std::uint8_t> stream = rampItems(3000UZ);
        const gr::property_map          family = convolutionalKind(5U, 4U);

        const std::vector<std::uint8_t> one  = flattened(runBlock<Interleave>(family, chunked(stream, 1UZ)));
        const std::vector<std::uint8_t> few  = flattened(runBlock<Interleave>(family, chunked(stream, 37UZ)));
        const std::vector<std::uint8_t> many = flattened(runBlock<Interleave>(family, chunked(stream, 1000UZ)));
        expect(eq(one.size(), stream.size()));
        expect(std::ranges::equal(one, few)) << "one item a record and 37 are the same stream";
        expect(std::ranges::equal(one, many)) << "one item a record and 1000 are the same stream";

        // And the same on the way back, which is the half a receiver runs.
        const std::vector<std::uint8_t> backOne  = flattened(runBlock<Deinterleave>(family, chunked(stream, 1UZ)));
        const std::vector<std::uint8_t> backMany = flattened(runBlock<Deinterleave>(family, chunked(stream, 1000UZ)));
        expect(std::ranges::equal(backOne, backMany)) << "the reverse lines are as record-blind as the forward ones";
    };

    return 0;
}
