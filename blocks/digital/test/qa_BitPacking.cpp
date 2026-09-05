#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <format>
#include <numeric>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/digital/BitPacking.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_bit_packing {

using gr::blocks::digital::PackBits;
using gr::blocks::digital::RepackBits;
using gr::blocks::digital::UnpackBits;
using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;
using gr::testing::ProcessFunction;
using gr::testing::TagMonitor;
using gr::testing::TagSink;
using gr::testing::TagSource;

/**
 * @brief A declared rate changer that records what the framework's end-of-stream hook does.
 *
 * It reports when `processEpilogue` runs, how much trailing input it carries, how much output the framework reserved
 * for it, and how many of those items a publish smaller than the reservation actually delivers. Nothing else exercises the
 * hook, and the tail contract of the bit-packing blocks rests on it.
 */
struct EpilogueProbe : gr::Block<EpilogueProbe, gr::Resampling<1UZ, 1UZ, false>> {
    gr::PortIn<std::uint8_t>  in;
    gr::PortOut<std::uint8_t> out;

    gr::Annotated<gr::Size_t, "chunk_in">     chunk_in     = 8U;
    gr::Annotated<gr::Size_t, "chunk_out">    chunk_out    = 3U;
    gr::Annotated<gr::Size_t, "tail_publish"> tail_publish = 0U;

    GR_MAKE_REFLECTABLE(EpilogueProbe, in, out, chunk_in, chunk_out, tail_publish);

    std::size_t _bulkCalls               = 0UZ;
    std::size_t _bulkIn                  = 0UZ;
    std::size_t _bulkOut                 = 0UZ;
    std::size_t _epilogueCalls           = 0UZ;
    std::size_t _epilogueIn              = 0UZ;
    std::size_t _epilogueReserved        = 0UZ;
    std::size_t _epiloguePublished       = 0UZ;
    std::size_t _bulkCallsBeforeEpilogue = 0UZ;

    void settingsChanged(const gr::property_map& /*oldSettings*/, const gr::property_map& /*newSettings*/) { declareRate(); }

    void start() { declareRate(); } // a probe built entirely at its defaults moves no value and calls back nowhere

    void declareRate() {
        this->input_chunk_size  = chunk_in;
        this->output_chunk_size = chunk_out;
    }

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) {
        ++_bulkCalls;
        _bulkIn += inSpan.size();
        _bulkOut += outSpan.size();
        std::ranges::fill(outSpan, std::uint8_t{1});
        return gr::work::Status::OK;
    }

    [[nodiscard]] gr::work::Status processEpilogue(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) {
        ++_epilogueCalls;
        _epilogueIn              = inSpan.size();
        _epilogueReserved        = outSpan.size();
        _bulkCallsBeforeEpilogue = _bulkCalls;

        const std::size_t emitted = std::min(static_cast<std::size_t>(tail_publish), outSpan.size());
        std::ranges::fill(std::span<std::uint8_t>(outSpan.data(), emitted), std::uint8_t{2});
        outSpan.publish(emitted);
        _epiloguePublished = emitted;
        return gr::work::Status::OK;
    }
};

struct ProbeRun {
    std::size_t               bulkIn       = 0UZ;
    std::size_t               bulkOut      = 0UZ;
    std::size_t               calls        = 0UZ;
    std::size_t               trailingIn   = 0UZ;
    std::size_t               reservedOut  = 0UZ;
    std::size_t               published    = 0UZ;
    std::size_t               bulkCallsRun = 0UZ;
    std::size_t               bulkCallsAt  = 0UZ;
    std::vector<std::uint8_t> received{};
};

[[nodiscard]] ProbeRun runProbe(gr::Size_t samples, gr::Size_t chunkIn, gr::Size_t chunkOut, gr::Size_t tailPublish) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<std::uint8_t, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", samples}, {"mark_tag", false}});
    auto&     probe  = graph.emplaceBlock<EpilogueProbe>({{"chunk_in", chunkIn}, {"chunk_out", chunkOut}, {"tail_publish", tailPublish}});
    auto&     sink   = graph.emplaceBlock<TagSink<std::uint8_t, ProcessFunction::USE_PROCESS_BULK>>({{"name", "TagSink"}});

    boost::ut::expect(graph.connect<"out", "in">(source, probe).has_value());
    boost::ut::expect(graph.connect<"out", "in">(probe, sink).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value());

    ProbeRun result;
    result.bulkIn       = probe._bulkIn;
    result.bulkOut      = probe._bulkOut;
    result.calls        = probe._epilogueCalls;
    result.trailingIn   = probe._epilogueIn;
    result.reservedOut  = probe._epilogueReserved;
    result.published    = probe._epiloguePublished;
    result.bulkCallsRun = probe._bulkCalls;
    result.bulkCallsAt  = probe._bulkCallsBeforeEpilogue;
    result.received.assign(sink._samples.begin(), sink._samples.end());
    return result;
}

/// @brief A configured block, taken through the same steps the framework takes before the first call.
template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
    return block;
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

[[nodiscard]] std::vector<std::uint8_t> randomFields(std::size_t count, unsigned bits, std::uint64_t seed) {
    Rng                       rng{seed};
    const std::uint8_t        mask = static_cast<std::uint8_t>((1U << bits) - 1U);
    std::vector<std::uint8_t> items(count);
    for (std::uint8_t& item : items) {
        item = static_cast<std::uint8_t>(rng.next() & mask);
    }
    return items;
}

struct Run {
    std::vector<std::uint8_t> samples{};
    std::vector<gr::Tag>      tags{};
};

/**
 * @brief Drive a bit-packing block over @p input in calls of @p periodsPerCall periods, as the framework would.
 *
 * The framework hands whole chunks and calls `forwardTags` before `processBulk`, and both are reproduced here so a
 * test can pick the call size and the absolute stream offset, neither of which a scheduler-driven test can reach.
 * @p flush runs the end-of-stream hook over whatever is left short of a period.
 */
template<typename TBlock>
[[nodiscard]] Run drive(TBlock& block, std::span<const std::uint8_t> input, std::size_t periodsPerCall = 0UZ, std::span<const gr::Tag> tags = {}, bool flush = false, std::size_t startOffset = 0UZ) {
    const std::size_t inChunk  = block.input_chunk_size;
    const std::size_t outChunk = block.output_chunk_size;
    const std::size_t stride   = (periodsPerCall == 0UZ ? std::max(input.size() / inChunk, 1UZ) : periodsPerCall) * inChunk;
    const std::size_t outStart = startOffset / inChunk * outChunk;

    Run                       result;
    std::vector<std::uint8_t> scratch(stride / inChunk * outChunk + outChunk);

    std::size_t inBase  = 0UZ;
    std::size_t outBase = 0UZ;
    while (inBase + inChunk <= input.size()) {
        const std::size_t count = std::min(stride, (input.size() - inBase) / inChunk * inChunk);
        const std::size_t made  = count / inChunk * outChunk;
        const auto        first = std::ranges::lower_bound(tags, startOffset + inBase, std::ranges::less{}, &gr::Tag::index);
        const auto        last  = std::ranges::lower_bound(tags, startOffset + inBase + count, std::ranges::less{}, &gr::Tag::index);

        InputSpan<std::uint8_t>  inSpan(input.subspan(inBase, count), startOffset + inBase, std::span<const gr::Tag>(first, last));
        OutputSpan<std::uint8_t> outSpan(std::span<std::uint8_t>(scratch.data(), made), outStart + outBase, &result.tags);

        auto inputs  = std::tie(inSpan);
        auto outputs = std::tie(outSpan);
        block.forwardTags(inputs, outputs, count);
        std::ignore = block.processBulk(inSpan, outSpan);

        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(made));
        inBase += count;
        outBase += made;
    }

    if (flush && inBase < input.size()) {
        const std::size_t trailing = input.size() - inBase;
        const std::size_t reserved = std::max((trailing * outChunk + inChunk - 1UZ) / inChunk, outChunk);
        scratch.assign(reserved, std::uint8_t{0});

        InputSpan<std::uint8_t>  inSpan(input.subspan(inBase, trailing), startOffset + inBase);
        OutputSpan<std::uint8_t> outSpan(std::span<std::uint8_t>(scratch.data(), reserved), outStart + outBase, &result.tags);
        std::ignore = block.processEpilogue(inSpan, outSpan);
        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
    }
    return result;
}

/**
 * @brief What the end-of-stream hook of @p block publishes when the stream ended on a chunk boundary.
 *
 * The framework runs the hook once per stream whether or not anything trails the last whole chunk, and reserves a
 * whole output chunk for it either way, so a block that emits on the strength of the reservation rather than of its
 * trailing input would add items to the end of every stream. `drive` cannot reach this: its flush branch needs a
 * trailing input to have something to flush.
 */
template<typename TBlock>
[[nodiscard]] std::size_t emptyTailPublish(TBlock& block) {
    std::vector<std::uint8_t> scratch(static_cast<std::size_t>(block.output_chunk_size), std::uint8_t{0});

    InputSpan<std::uint8_t>  inSpan(std::span<const std::uint8_t>{});
    OutputSpan<std::uint8_t> outSpan(std::span<std::uint8_t>(scratch), 0UZ, nullptr);
    std::ignore = block.processEpilogue(inSpan, outSpan);
    return outSpan.count;
}

constexpr std::array<const char*, 2> kOrders{"MsbFirst", "LsbFirst"};

struct Marker {
    const char*    key;
    std::size_t    at;
    gr::pmt::Value value;
};

/// Six reserved keys and one the default forwarder filters, two of them sharing an input offset. The array is held
/// inside the function so that it outlives no test: a namespace-scope value owning pmr storage is destroyed in an
/// order the test runner does not fix, and the runner walks the suites from its own destructor.
[[nodiscard]] const std::array<Marker, 7>& markers() {
    static const std::array<Marker, 7> kMarkers{{
        {"sample_rate", 0UZ, gr::pmt::Value(48000.0f)},
        {"signal_name", 8UZ, gr::pmt::Value(std::string("bits"))},
        {"signal_unit", 8UZ, gr::pmt::Value(std::string("none"))},
        {"signal_min", 24UZ, gr::pmt::Value(0.0f)},
        {"signal_max", 40UZ, gr::pmt::Value(1.0f)},
        {"trigger_name", 56UZ, gr::pmt::Value(std::string("burst"))},
        {"private_key", 72UZ, gr::pmt::Value(std::string("carried"))},
    }};
    return kMarkers;
}

/// @brief Run @p TBlock between a tagging source and a tag sink, and report where every marker came out.
template<typename TBlock>
[[nodiscard]] std::vector<std::size_t> markerOffsets(gr::property_map settings, gr::Size_t samples) {
    constexpr std::size_t kAbsent = static_cast<std::size_t>(-1);

    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<std::uint8_t, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", samples}, {"mark_tag", false}});
    for (const Marker& marker : markers()) {
        source._tags.emplace_back(marker.at, gr::property_map{{gr::property_map::key_type{marker.key}, marker.value}});
    }
    std::ranges::sort(source._tags, std::ranges::less{}, &gr::Tag::index);

    auto& block = graph.emplaceBlock<TBlock>(std::move(settings));
    auto& sink  = graph.emplaceBlock<TagSink<std::uint8_t, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

    boost::ut::expect(graph.connect<"out", "in">(source, block).has_value());
    boost::ut::expect(graph.connect<"out", "in">(block, sink).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value());

    std::vector<std::size_t> offsets(markers().size(), kAbsent);
    for (const gr::Tag& tag : sink._tags) {
        for (std::size_t which = 0UZ; which < markers().size(); ++which) {
            const auto found = tag.map.find(gr::property_map::key_type{markers()[which].key});
            if (found != tag.map.end() && found->second == markers()[which].value) {
                offsets[which] = tag.index;
            }
        }
    }
    return offsets;
}

[[nodiscard]] std::size_t graphSampleCount(gr::property_map settings, gr::Size_t samples) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<std::uint8_t, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", samples}, {"mark_tag", false}});
    auto&     block  = graph.emplaceBlock<RepackBits>(std::move(settings));
    auto&     sink   = graph.emplaceBlock<TagSink<std::uint8_t, ProcessFunction::USE_PROCESS_BULK>>({{"name", "TagSink"}});

    boost::ut::expect(graph.connect<"out", "in">(source, block).has_value());
    boost::ut::expect(graph.connect<"out", "in">(block, sink).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value());
    return sink._samples.size();
}

static_assert(!gr::block::kUnfilteredTagPropagationAdmissible<RepackBits>, "a declared rate changer that writes its own tag map must not claim offset identity");
static_assert(!gr::block::kUnfilteredTagPropagationAdmissible<PackBits>);
static_assert(!gr::block::kUnfilteredTagPropagationAdmissible<UnpackBits>);

} // namespace qa_bit_packing

using namespace qa_bit_packing;

const boost::ut::suite<"BitPackingEpilogue"> epilogueProbeTests = [] {
    using namespace boost::ut;

    "the end-of-stream hook carries the trailing input a whole chunk cannot cover"_test = [] {
        const ProbeRun run = runProbe(39U, 8U, 3U, 3U);

        expect(eq(run.calls, 1UZ)) << "the hook fires exactly once, at end of stream";
        expect(eq(run.bulkIn, 32UZ)) << "ordinary calls take whole chunks only";
        expect(eq(run.bulkOut, 12UZ));
        expect(eq(run.trailingIn, 7UZ)) << "39 items leave 7 short of the 8-item chunk";
        expect(eq(run.reservedOut, 3UZ)) << "the reservation is ceil(trailing * chunkOut / chunkIn)";
        expect(eq(run.bulkCallsAt, run.bulkCallsRun)) << "the hook runs after the last ordinary call";
        expect(eq(run.received.size(), 15UZ)) << "12 from the chunks, 3 from the tail";
    };

    "the hook publishes what it asks for and nothing more"_test = [] {
        for (const gr::Size_t asked : {0U, 1U, 2U, 3U}) {
            const ProbeRun run = runProbe(39U, 8U, 3U, asked);

            expect(eq(run.published, static_cast<std::size_t>(asked)));
            expect(eq(run.received.size(), 12UZ + static_cast<std::size_t>(asked))) << std::format("a publish of {} reaches the sink as {} items", asked, asked);
            expect(eq(run.calls, 1UZ)) << "the trailing input is consumed either way: a hook that left it would be called again";
            for (std::size_t i = 12UZ; i < run.received.size(); ++i) {
                expect(eq(run.received[i], std::uint8_t{2})) << "the tail items are the ones the hook wrote";
            }
        }
    };

    "the reservation is never smaller than one output chunk"_test = [] {
        const ProbeRun run = runProbe(25U, 3U, 8U, 8U);

        expect(eq(run.calls, 1UZ));
        expect(eq(run.trailingIn, 1UZ)) << "25 items leave 1 short of the 3-item chunk";
        expect(eq(run.reservedOut, 8UZ)) << "ceil(1 * 8 / 3) is 3, and the reservation is raised to the 8-item output chunk";
        expect(eq(run.published, 8UZ));
    };

    "a stream ending on a chunk boundary reaches the hook with an empty tail"_test = [] {
        const ProbeRun quiet = runProbe(40U, 8U, 3U, 0U);

        expect(eq(quiet.bulkIn, 40UZ)) << "40 items are five whole 8-item chunks";
        expect(eq(quiet.bulkOut, 15UZ));
        expect(eq(quiet.calls, 1UZ)) << "the hook fires once per stream, whatever the last chunk left";
        expect(eq(quiet.trailingIn, 0UZ)) << "nothing trails a whole period";
        expect(eq(quiet.bulkCallsAt, quiet.bulkCallsRun)) << "the hook still runs after the last ordinary call";
        expect(eq(quiet.received.size(), 15UZ)) << "a hook that publishes nothing adds nothing to the stream";

        const ProbeRun asking = runProbe(40U, 8U, 3U, 3U);

        expect(eq(asking.trailingIn, 0UZ));
        expect(eq(asking.reservedOut, 3UZ)) << "the reservation is the whole output chunk even with no trailing input";
        expect(eq(asking.published, 3UZ));
        expect(eq(asking.received.size(), 18UZ)) << "the framework publishes what the hook asked for, so deciding that an empty tail yields nothing is the block's own job";
    };

    "the hook of a bit-packing block reached with an empty tail publishes nothing"_test = [] {
        for (const bool flush : {true, false}) {
            RepackBits repack   = make<RepackBits>({{"bits_in", 8U}, {"bits_out", 3U}, {"flush_partial", flush}});
            RepackBits widening = make<RepackBits>({{"bits_in", 3U}, {"bits_out", 8U}, {"flush_partial", flush}});
            PackBits   packer   = make<PackBits>({{"k", 5U}, {"flush_partial", flush}});
            UnpackBits unpacker = make<UnpackBits>({{"k", 5U}, {"flush_partial", flush}});

            expect(eq(emptyTailPublish(repack), 0UZ)) << "no trailing item strands no bit, so there is nothing to pad and nothing to drop";
            expect(eq(emptyTailPublish(widening), 0UZ));
            expect(eq(emptyTailPublish(packer), 0UZ));
            expect(eq(emptyTailPublish(unpacker), 0UZ));
        }
    };
};

const boost::ut::suite<"RepackBits"> repackBitsTests = [] {
    using namespace boost::ut;

    "every width pair and every order pair round trips exactly"_test = [] {
        for (gr::Size_t bitsIn = 1U; bitsIn <= 8U; ++bitsIn) {
            for (gr::Size_t bitsOut = 1U; bitsOut <= 8U; ++bitsOut) {
                for (const char* orderIn : kOrders) {
                    for (const char* orderOut : kOrders) {
                        RepackBits forward = make<RepackBits>({{"bits_in", bitsIn}, {"bits_out", bitsOut}, {"input_bit_order", std::string(orderIn)}, {"output_bit_order", std::string(orderOut)}});
                        RepackBits back    = make<RepackBits>({{"bits_in", bitsOut}, {"bits_out", bitsIn}, {"input_bit_order", std::string(orderOut)}, {"output_bit_order", std::string(orderIn)}});

                        const std::vector<std::uint8_t> data      = randomFields(11UZ * forward.input_chunk_size, bitsIn, 0x243f6a88ULL + 64ULL * bitsIn + bitsOut);
                        const Run                       converted = drive(forward, std::span<const std::uint8_t>(data), 3UZ);
                        const Run                       recovered = drive(back, std::span<const std::uint8_t>(converted.samples), 2UZ);

                        expect(eq(converted.samples.size(), data.size() / forward.input_chunk_size * forward.output_chunk_size));
                        expect(std::ranges::equal(recovered.samples, data)) << std::format("{}->{}, {}/{}: the bit stream is not preserved", bitsIn, bitsOut, orderIn, orderOut);
                    }
                }
            }
        }
    };

    "equal widths are the identity, and unequal orders are a per-byte bit reversal"_test = [] {
        std::vector<std::uint8_t> everyByte(256UZ);
        std::iota(everyByte.begin(), everyByte.end(), std::uint8_t{0});

        for (const char* order : kOrders) {
            RepackBits same = make<RepackBits>({{"bits_in", 8U}, {"bits_out", 8U}, {"input_bit_order", std::string(order)}, {"output_bit_order", std::string(order)}});
            expect(std::ranges::equal(drive(same, std::span<const std::uint8_t>(everyByte)).samples, everyByte)) << std::format("{} on both sides is a copy", order);
        }

        for (const char* orderIn : kOrders) {
            const char* orderOut = orderIn == kOrders[0] ? kOrders[1] : kOrders[0];
            RepackBits  crossed  = make<RepackBits>({{"bits_in", 8U}, {"bits_out", 8U}, {"input_bit_order", std::string(orderIn)}, {"output_bit_order", std::string(orderOut)}});
            const Run   reversed = drive(crossed, std::span<const std::uint8_t>(everyByte));

            for (std::size_t value = 0UZ; value < everyByte.size(); ++value) {
                std::uint8_t expected = 0U;
                for (unsigned bit = 0U; bit < 8U; ++bit) {
                    expected = static_cast<std::uint8_t>(expected | (((value >> bit) & 1UZ) << (7U - bit)));
                }
                expect(eq(reversed.samples[value], expected)) << std::format("{} in, {} out at 8/8 reverses the byte", orderIn, orderOut);
            }
        }
    };

    "the worked conversions reproduce"_test = [] {
        const std::array<std::uint8_t, 3> threeBytes{0xC5U, 0x3AU, 0x9FU};

        RepackBits msb = make<RepackBits>({{"bits_in", 8U}, {"bits_out", 3U}});
        RepackBits lsb = make<RepackBits>({{"bits_in", 8U}, {"bits_out", 3U}, {"input_bit_order", std::string("LsbFirst")}, {"output_bit_order", std::string("LsbFirst")}});

        const std::array<std::uint8_t, 8> msbSymbols{6U, 1U, 2U, 3U, 5U, 2U, 3U, 7U};
        const std::array<std::uint8_t, 8> lsbSymbols{5U, 0U, 3U, 5U, 3U, 6U, 7U, 4U};
        expect(std::ranges::equal(drive(msb, std::span<const std::uint8_t>(threeBytes)).samples, msbSymbols));
        expect(std::ranges::equal(drive(lsb, std::span<const std::uint8_t>(threeBytes)).samples, lsbSymbols));

        RepackBits msbBack = make<RepackBits>({{"bits_in", 3U}, {"bits_out", 8U}});
        RepackBits lsbBack = make<RepackBits>({{"bits_in", 3U}, {"bits_out", 8U}, {"input_bit_order", std::string("LsbFirst")}, {"output_bit_order", std::string("LsbFirst")}});
        expect(std::ranges::equal(drive(msbBack, std::span<const std::uint8_t>(msbSymbols)).samples, threeBytes)) << "three bytes are one period and nothing is left over";
        expect(std::ranges::equal(drive(lsbBack, std::span<const std::uint8_t>(lsbSymbols)).samples, threeBytes));

        const std::array<std::uint8_t, 2> twoBytes{0xF5U, 0x08U};
        RepackBits                        nibblesMsb = make<RepackBits>({{"bits_in", 8U}, {"bits_out", 4U}});
        RepackBits                        nibblesLsb = make<RepackBits>({{"bits_in", 8U}, {"bits_out", 4U}, {"input_bit_order", std::string("LsbFirst")}, {"output_bit_order", std::string("LsbFirst")}});
        expect(std::ranges::equal(drive(nibblesMsb, std::span<const std::uint8_t>(twoBytes)).samples, std::array<std::uint8_t, 4>{0xFU, 0x5U, 0x0U, 0x8U}));
        expect(std::ranges::equal(drive(nibblesLsb, std::span<const std::uint8_t>(twoBytes)).samples, std::array<std::uint8_t, 4>{0x5U, 0xFU, 0x8U, 0x0U}));
    };

    "the output does not depend on how the stream is divided into calls"_test = [] {
        for (const auto [bitsIn, bitsOut] : std::array<std::pair<gr::Size_t, gr::Size_t>, 2>{{{8U, 3U}, {3U, 8U}}}) {
            RepackBits                      reference = make<RepackBits>({{"bits_in", bitsIn}, {"bits_out", bitsOut}});
            const std::vector<std::uint8_t> data      = randomFields(512UZ * reference.input_chunk_size, bitsIn, 0xb7e15162ULL + bitsIn);
            const Run                       whole     = drive(reference, std::span<const std::uint8_t>(data));

            for (const std::size_t periods : {1UZ, 3UZ, 17UZ, 4096UZ}) {
                RepackBits chunked = make<RepackBits>({{"bits_in", bitsIn}, {"bits_out", bitsOut}});
                expect(std::ranges::equal(drive(chunked, std::span<const std::uint8_t>(data), periods).samples, whole.samples)) << std::format("{}->{} at {} periods per call", bitsIn, bitsOut, periods);
            }
        }
    };

    "a tag lands on the output item holding the first bit of the item it marked"_test = [] {
        const auto mapped = [](RepackBits& block, std::span<const std::size_t> at) {
            std::vector<gr::Tag> tags;
            for (const std::size_t index : at) {
                tags.emplace_back(index, gr::property_map{{gr::property_map::key_type{"signal_name"}, gr::pmt::Value(std::to_string(index))}});
            }
            const std::size_t          items = (at.back() + 1UZ + block.input_chunk_size) / block.input_chunk_size * block.input_chunk_size;
            const std::vector<uint8_t> data(items, std::uint8_t{0});
            const Run                  run = drive(block, std::span<const std::uint8_t>(data), 2UZ, std::span<const gr::Tag>(tags));

            std::vector<std::size_t> offsets;
            for (const gr::Tag& seenTag : run.tags) {
                offsets.push_back(seenTag.index);
            }
            return offsets;
        };

        const std::array<std::size_t, 9> nine{0UZ, 1UZ, 2UZ, 3UZ, 4UZ, 5UZ, 6UZ, 7UZ, 8UZ};
        const std::array<std::size_t, 9> eightToThree{0UZ, 2UZ, 5UZ, 8UZ, 10UZ, 13UZ, 16UZ, 18UZ, 21UZ};
        RepackBits                       widening = make<RepackBits>({{"bits_in", 8U}, {"bits_out", 3U}});
        expect(std::ranges::equal(mapped(widening, std::span<const std::size_t>(nine)), eightToThree));

        const std::array<std::size_t, 10> ten{0UZ, 1UZ, 2UZ, 3UZ, 4UZ, 5UZ, 6UZ, 7UZ, 8UZ, 9UZ};
        const std::array<std::size_t, 10> threeToEight{0UZ, 0UZ, 0UZ, 1UZ, 1UZ, 1UZ, 2UZ, 2UZ, 3UZ, 3UZ};
        RepackBits                        narrowing = make<RepackBits>({{"bits_in", 3U}, {"bits_out", 8U}});
        expect(std::ranges::equal(mapped(narrowing, std::span<const std::size_t>(ten)), threeToEight)) << "several input tags share one output offset, in input order";
    };

    "two tags at one input offset both survive, in order"_test = [] {
        RepackBits           block = make<RepackBits>({{"bits_in", 3U}, {"bits_out", 8U}});
        std::vector<gr::Tag> tags{
            gr::Tag{4UZ, gr::property_map{{gr::property_map::key_type{"signal_name"}, gr::pmt::Value(std::string("first"))}}},
            gr::Tag{4UZ, gr::property_map{{gr::property_map::key_type{"signal_unit"}, gr::pmt::Value(std::string("second"))}}},
        };
        const std::vector<std::uint8_t> data(16UZ, std::uint8_t{0});
        const Run                       run = drive(block, std::span<const std::uint8_t>(data), 1UZ, std::span<const gr::Tag>(tags));

        expect(eq(run.tags.size(), 2UZ));
        expect(eq(run.tags[0].index, 1UZ));
        expect(eq(run.tags[1].index, 1UZ));
        expect(run.tags[0].map.contains(gr::property_map::key_type{"signal_name"}));
        expect(run.tags[1].map.contains(gr::property_map::key_type{"signal_unit"}));
    };

    "a tag offset past the exact range of a double maps to the exact integer"_test = [] {
        if constexpr (sizeof(std::size_t) != 8UZ) {
            return; // the offset below is past 2^53, which a 32-bit std::size_t does not hold at all
        }
        constexpr std::size_t kBeyondDouble = static_cast<std::size_t>(9007199254740993ULL); // 2^53 + 1, three times 3002399751580331

        RepackBits                      packing = make<RepackBits>({{"bits_in", 1U}, {"bits_out", 3U}});
        const std::vector<gr::Tag>      packTag{gr::Tag{kBeyondDouble, gr::property_map{{gr::property_map::key_type{"signal_name"}, gr::pmt::Value(std::string("far"))}}}};
        const std::vector<std::uint8_t> data(12UZ, std::uint8_t{0});
        const Run                       packed = drive(packing, std::span<const std::uint8_t>(data), 2UZ, std::span<const gr::Tag>(packTag), false, kBeyondDouble);

        expect(eq(packed.tags.size(), 1UZ));
        expect(eq(packed.tags[0].index, static_cast<std::size_t>(3002399751580331ULL))) << "a double path would answer one less";

        RepackBits                 unpacking = make<RepackBits>({{"bits_in", 3U}, {"bits_out", 1U}});
        const std::vector<gr::Tag> unpackTag{gr::Tag{kBeyondDouble, gr::property_map{{gr::property_map::key_type{"signal_name"}, gr::pmt::Value(std::string("far"))}}}};
        const Run                  unpacked = drive(unpacking, std::span<const std::uint8_t>(data), 2UZ, std::span<const gr::Tag>(unpackTag), false, kBeyondDouble);

        expect(eq(unpacked.tags.size(), 1UZ));
        expect(eq(unpacked.tags[0].index, 3UZ * kBeyondDouble));
    };

    "a tag whose output item is past the call is held until the output reaches it"_test = [] {
        RepackBits block = make<RepackBits>({{"bits_in", 3U}, {"bits_out", 8U}});

        std::vector<gr::Tag> tags;
        for (const std::size_t at : {1UZ, 5UZ, 9UZ, 13UZ, 17UZ}) {
            tags.emplace_back(at, gr::property_map{{gr::property_map::key_type{"signal_name"}, gr::pmt::Value(std::to_string(at))}});
        }
        const std::vector<std::uint8_t> data(24UZ, std::uint8_t{0});
        const Run                       held = drive(block, std::span<const std::uint8_t>(data), 1UZ, std::span<const gr::Tag>(tags));
        const Run                       once = drive(block, std::span<const std::uint8_t>(data), 0UZ, std::span<const gr::Tag>(tags));

        expect(eq(held.tags.size(), tags.size())) << "no tag is dropped when its output item arrives in a later call";
        expect(eq(held.tags.size(), once.tags.size()));
        for (std::size_t which = 0UZ; which < held.tags.size(); ++which) {
            expect(eq(held.tags[which].index, once.tags[which].index)) << "one call at a time and one call for the whole stream place the tags alike";
        }
    };

    "a block left at its defaults declares its own rate"_test = [] {
        RepackBits general = make<RepackBits>();
        expect(eq(static_cast<std::size_t>(general.input_chunk_size), 8UZ)) << "the default widths are 1 bit in and 8 bits out";
        expect(eq(static_cast<std::size_t>(general.output_chunk_size), 1UZ));
        expect(eq(graphSampleCount({}, 32U), 4UZ)) << "a block that staged no setting runs in a graph at the rate it declares";
    };

    "a width or an order outside the set is refused and the conversion survives"_test = [] {
        expect(throws([] { std::ignore = make<RepackBits>({{"bits_in", 0U}}); })) << "zero is not a field width";
        expect(throws([] { std::ignore = make<RepackBits>({{"bits_out", 0U}}); }));
        expect(throws([] { std::ignore = make<RepackBits>({{"bits_in", 9U}}); })) << "nine bits do not fit the item";
        expect(throws([] { std::ignore = make<RepackBits>({{"bits_out", 9U}}); }));

        RepackBits block = make<RepackBits>({{"bits_in", 8U}, {"bits_out", 3U}});
        expect(throws([&block] {
            std::ignore = block.settings().set({{"bits_out", 9U}});
            std::ignore = block.settings().activateContext();
            std::ignore = block.settings().applyStagedParameters();
        })) << "a live width change is validated on the same route as a constructed one";
        expect(eq(static_cast<unsigned>(block._repack.bitsIn), 8U));
        expect(eq(static_cast<unsigned>(block._repack.bitsOut), 3U)) << "a rejected width leaves the conversion as it was";
        expect(std::ranges::equal(drive(block, std::span<const std::uint8_t>(std::array<std::uint8_t, 3>{0xC5U, 0x3AU, 0x9FU})).samples, std::array<std::uint8_t, 8>{6U, 1U, 2U, 3U, 5U, 2U, 3U, 7U}));
    };

    "a stream value cannot escape its field or stop the block"_test = [] {
        for (gr::Size_t bitsIn = 1U; bitsIn <= 8U; ++bitsIn) {
            for (gr::Size_t bitsOut = 1U; bitsOut <= 8U; ++bitsOut) {
                RepackBits wide   = make<RepackBits>({{"bits_in", bitsIn}, {"bits_out", bitsOut}});
                RepackBits narrow = make<RepackBits>({{"bits_in", bitsIn}, {"bits_out", bitsOut}});

                const std::vector<std::uint8_t> loud   = randomFields(64UZ * wide.input_chunk_size, 8U, 0x9e3779b9ULL + 64ULL * bitsIn + bitsOut);
                std::vector<std::uint8_t>       masked = loud;
                for (std::uint8_t& item : masked) {
                    item = static_cast<std::uint8_t>(item & ((1U << bitsIn) - 1U));
                }

                const Run fromLoud   = drive(wide, std::span<const std::uint8_t>(loud));
                const Run fromMasked = drive(narrow, std::span<const std::uint8_t>(masked));
                expect(std::ranges::equal(fromLoud.samples, fromMasked.samples)) << std::format("{}->{}: the unused high bits of an input item are ignored", bitsIn, bitsOut);
                expect(std::ranges::all_of(fromLoud.samples, [bitsOut](std::uint8_t item) { return item < (1U << bitsOut); })) << "no output item leaves its field";
            }
        }
    };

    "the tail is flushed zero-padded, or dropped and reported"_test = [] {
        const std::array<std::uint8_t, 5> data{0xC5U, 0x3AU, 0x9FU, 0xC5U, 0x3BU};

        for (const char* orderOut : kOrders) {
            RepackBits padding  = make<RepackBits>({{"bits_in", 8U}, {"bits_out", 3U}, {"output_bit_order", std::string(orderOut)}});
            RepackBits dropping = make<RepackBits>({{"bits_in", 8U}, {"bits_out", 3U}, {"output_bit_order", std::string(orderOut)}, {"flush_partial", false}});

            const Run flushed = drive(padding, std::span<const std::uint8_t>(data), 0UZ, {}, true);
            const Run dropped = drive(dropping, std::span<const std::uint8_t>(data), 0UZ, {}, true);

            expect(eq(flushed.samples.size(), 14UZ)) << "8 from the whole period, 5 whole symbols from the 16 stranded bits, and one padded";
            expect(eq(dropped.samples.size(), 13UZ)) << "the single remaining bit is dropped";
            expect(std::ranges::equal(std::span<const std::uint8_t>(flushed.samples).first(13UZ), dropped.samples)) << "the two answers differ only in the padded item";

            const unsigned last = flushed.samples.back();
            if (std::string(orderOut) == "MsbFirst") {
                expect(eq(last & 0x3U, 0U)) << "the pad takes the positions the later bits would have gone to";
                expect(eq(last >> 2U, 1U)) << "the one real bit is the top of the field";
            } else {
                expect(eq(last >> 1U, 0U));
                expect(eq(last & 1U, 1U)) << "the one real bit is the bottom of the field";
            }
        }
    };

    "the flush setting is inert where a single input item is a whole period"_test = [] {
        const std::vector<std::uint8_t> data = randomFields(37UZ, 8U, 0x452821e6ULL);

        for (const auto [bitsIn, bitsOut] : std::array<std::pair<gr::Size_t, gr::Size_t>, 3>{{{8U, 1U}, {8U, 4U}, {8U, 8U}}}) {
            RepackBits padding  = make<RepackBits>({{"bits_in", bitsIn}, {"bits_out", bitsOut}});
            RepackBits dropping = make<RepackBits>({{"bits_in", bitsIn}, {"bits_out", bitsOut}, {"flush_partial", false}});

            expect(eq(static_cast<std::size_t>(padding.input_chunk_size), 1UZ));
            expect(std::ranges::equal(drive(padding, std::span<const std::uint8_t>(data), 0UZ, {}, true).samples, drive(dropping, std::span<const std::uint8_t>(data), 0UZ, {}, true).samples)) << std::format("{}->{} has no trailing input to decide about", bitsIn, bitsOut);
        }
    };

    "the tail reaches the sink in a running graph"_test = [] {
        expect(eq(graphSampleCount({{"bits_in", 8U}, {"bits_out", 3U}}, 5U), 14UZ));
        expect(eq(graphSampleCount({{"bits_in", 8U}, {"bits_out", 3U}, {"flush_partial", false}}, 5U), 13UZ));
        expect(eq(graphSampleCount({{"bits_in", 8U}, {"bits_out", 3U}}, 6U), 16UZ)) << "six bytes are 48 bits, exactly 16 symbols and no remainder";
    };

    "every key is forwarded, and a default-forwarding neighbor does not forward them all"_test = [] {
        const std::vector<std::size_t> throughRepack = markerOffsets<RepackBits>({{"bits_in", 8U}, {"bits_out", 8U}}, 128U);
        for (std::size_t which = 0UZ; which < markers().size(); ++which) {
            expect(eq(throughRepack[which], markers()[which].at)) << std::format("key '{}' survives at its own offset, the map being the identity at 8/8", markers()[which].key);
        }

        const std::vector<std::size_t> throughNeighbor = markerOffsets<TagMonitor<std::uint8_t, ProcessFunction::USE_PROCESS_BULK>>({{"name", "TagMonitor"}}, 128U);
        expect(neq(throughNeighbor.back(), markers().back().at)) << "the default forwarder keeps only the reserved keys, so 'private_key' does not survive it";
    };

    "a tag map is placed at the offsets a rate change puts it at"_test = [] {
        const std::vector<std::size_t> packed = markerOffsets<RepackBits>({{"bits_in", 1U}, {"bits_out", 8U}}, 256U);
        for (std::size_t which = 0UZ; which < markers().size(); ++which) {
            expect(eq(packed[which], markers()[which].at / 8UZ)) << std::format("key '{}' at input {} maps to output {}", markers()[which].key, markers()[which].at, markers()[which].at / 8UZ);
        }
    };
};

const boost::ut::suite<"PackBits"> packBitsTests = [] {
    using namespace boost::ut;

    "packing is the general regrouper wearing one width and one order"_test = [] {
        for (gr::Size_t k = 1U; k <= 8U; ++k) {
            for (const char* order : kOrders) {
                PackBits   packer  = make<PackBits>({{"k", k}, {"bit_order", std::string(order)}});
                RepackBits general = make<RepackBits>({{"bits_in", 1U}, {"bits_out", k}, {"output_bit_order", std::string(order)}});

                const std::vector<std::uint8_t> bits = randomFields(97UZ * k, 1U, 0xc0ac29b7ULL + k);
                expect(std::ranges::equal(drive(packer, std::span<const std::uint8_t>(bits), 5UZ).samples, drive(general, std::span<const std::uint8_t>(bits), 5UZ).samples)) << std::format("k={} {}: the two faces are one kernel", k, order);
            }
        }
    };

    "eight bits make a byte, from whichever end"_test = [] {
        const std::array<std::uint8_t, 8> bits{1U, 0U, 1U, 1U, 0U, 0U, 1U, 0U};

        PackBits msb = make<PackBits>();
        PackBits lsb = make<PackBits>({{"bit_order", std::string("LsbFirst")}});
        expect(std::ranges::equal(drive(msb, std::span<const std::uint8_t>(bits)).samples, std::array<std::uint8_t, 1>{0xB2U}));
        expect(std::ranges::equal(drive(lsb, std::span<const std::uint8_t>(bits)).samples, std::array<std::uint8_t, 1>{0x4DU})) << "the two orders are bit reversals of each other whenever the input is one bit wide";
    };

    "a one-bit field has no order, and packing it is a copy of the low bit"_test = [] {
        const std::vector<std::uint8_t> loud = randomFields(256UZ, 8U, 0x21b1a9c6ULL);

        PackBits  msb = make<PackBits>({{"k", 1U}});
        PackBits  lsb = make<PackBits>({{"k", 1U}, {"bit_order", std::string("LsbFirst")}});
        const Run one = drive(msb, std::span<const std::uint8_t>(loud));
        const Run two = drive(lsb, std::span<const std::uint8_t>(loud));

        expect(std::ranges::equal(one.samples, two.samples)) << "the two orders agree at k = 1";
        for (std::size_t i = 0UZ; i < loud.size(); ++i) {
            expect(eq(one.samples[i], static_cast<std::uint8_t>(loud[i] & 1U))) << "only the low bit of an input item is read";
        }
    };

    "several input tags share the output item holding their bits"_test = [] {
        const std::array<std::size_t, 7> at{0UZ, 3UZ, 7UZ, 8UZ, 15UZ, 16UZ, 23UZ};
        const std::array<std::size_t, 7> expected{0UZ, 0UZ, 0UZ, 1UZ, 1UZ, 2UZ, 2UZ};

        PackBits             block = make<PackBits>();
        std::vector<gr::Tag> tags;
        for (const std::size_t index : at) {
            tags.emplace_back(index, gr::property_map{{gr::property_map::key_type{"signal_name"}, gr::pmt::Value(std::to_string(index))}});
        }
        const std::vector<std::uint8_t> bits(32UZ, std::uint8_t{1});
        const Run                       run = drive(block, std::span<const std::uint8_t>(bits), 1UZ, std::span<const gr::Tag>(tags));

        expect(eq(run.tags.size(), at.size()));
        for (std::size_t which = 0UZ; which < run.tags.size(); ++which) {
            expect(eq(run.tags[which].index, expected[which])) << std::format("a tag on bit {} belongs on byte {}", at[which], expected[which]);
        }
    };

    "the last incomplete byte is emitted padded, or dropped and reported"_test = [] {
        const std::array<std::uint8_t, 13> bits{1U, 1U, 0U, 1U, 0U, 0U, 0U, 1U, 1U, 0U, 1U, 1U, 1U};

        PackBits  padding  = make<PackBits>();
        PackBits  dropping = make<PackBits>({{"flush_partial", false}});
        const Run flushed  = drive(padding, std::span<const std::uint8_t>(bits), 0UZ, {}, true);
        const Run dropped  = drive(dropping, std::span<const std::uint8_t>(bits), 0UZ, {}, true);

        expect(eq(flushed.samples.size(), 2UZ)) << "eight bits make the first byte and the remaining five make a padded second";
        expect(eq(dropped.samples.size(), 1UZ));
        expect(eq(flushed.samples[0], std::uint8_t{0xD1U}));
        expect(eq(flushed.samples[1], std::uint8_t{0b10111000U})) << "the five real bits are the top of the field and the pad is the bottom";
        expect(eq(dropped.samples[0], flushed.samples[0]));
    };

    "a block left at its defaults declares its own rate"_test = [] {
        PackBits packer = make<PackBits>();
        expect(eq(static_cast<std::size_t>(packer.input_chunk_size), 8UZ)) << "eight one-bit items make one byte";
        expect(eq(static_cast<std::size_t>(packer.output_chunk_size), 1UZ));
    };

    "a width or an order outside the set is refused"_test = [] {
        expect(throws([] { std::ignore = make<PackBits>({{"k", 0U}}); }));
        expect(throws([] { std::ignore = make<PackBits>({{"k", 9U}}); })) << "nine bits do not fit the byte a packed item is";

        PackBits block = make<PackBits>({{"k", 5U}});
        expect(throws([&block] {
            std::ignore = block.settings().set({{"k", 9U}});
            std::ignore = block.settings().activateContext();
            std::ignore = block.settings().applyStagedParameters();
        }));
        expect(eq(static_cast<unsigned>(block._repack.bitsOut), 5U)) << "a rejected width leaves the conversion as it was";
    };
};

const boost::ut::suite<"UnpackBits"> unpackBitsTests = [] {
    using namespace boost::ut;

    "unpacking is the general regrouper wearing one width and one order"_test = [] {
        for (gr::Size_t k = 1U; k <= 8U; ++k) {
            for (const char* order : kOrders) {
                UnpackBits unpacker = make<UnpackBits>({{"k", k}, {"bit_order", std::string(order)}});
                RepackBits general  = make<RepackBits>({{"bits_in", k}, {"bits_out", 1U}, {"input_bit_order", std::string(order)}});

                const std::vector<std::uint8_t> items = randomFields(83UZ, k, 0x38d01377ULL + k);
                expect(std::ranges::equal(drive(unpacker, std::span<const std::uint8_t>(items), 7UZ).samples, drive(general, std::span<const std::uint8_t>(items), 7UZ).samples)) << std::format("k={} {}: the two faces are one kernel", k, order);
            }
        }
    };

    "a byte leaves as eight bits, from whichever end"_test = [] {
        const std::array<std::uint8_t, 2> bytes{0xF5U, 0x08U};

        UnpackBits msb = make<UnpackBits>();
        UnpackBits lsb = make<UnpackBits>({{"bit_order", std::string("LsbFirst")}});
        expect(std::ranges::equal(drive(msb, std::span<const std::uint8_t>(bytes)).samples, std::array<std::uint8_t, 16>{1U, 1U, 1U, 1U, 0U, 1U, 0U, 1U, 0U, 0U, 0U, 0U, 1U, 0U, 0U, 0U}));
        expect(std::ranges::equal(drive(lsb, std::span<const std::uint8_t>(bytes)).samples, std::array<std::uint8_t, 16>{1U, 0U, 1U, 0U, 1U, 1U, 1U, 1U, 0U, 0U, 0U, 1U, 0U, 0U, 0U, 0U}));
    };

    "packing and unpacking are inverses, and unpacking one bit is a copy"_test = [] {
        for (gr::Size_t k = 1U; k <= 8U; ++k) {
            for (const char* order : kOrders) {
                UnpackBits unpacker = make<UnpackBits>({{"k", k}, {"bit_order", std::string(order)}});
                PackBits   packer   = make<PackBits>({{"k", k}, {"bit_order", std::string(order)}});

                const std::vector<std::uint8_t> items     = randomFields(64UZ, k, 0xbe5466cfULL + k);
                const Run                       bits      = drive(unpacker, std::span<const std::uint8_t>(items), 9UZ);
                const Run                       recovered = drive(packer, std::span<const std::uint8_t>(bits.samples), 9UZ);

                expect(eq(bits.samples.size(), items.size() * k)) << "one item in, k out";
                expect(std::ranges::all_of(bits.samples, [](std::uint8_t bit) { return bit < 2U; })) << "every output item is one bit";
                expect(std::ranges::equal(recovered.samples, items)) << std::format("k={} {}: the pair round trips", k, order);
            }
        }
    };

    "only the low k bits of an input item are read"_test = [] {
        const std::vector<std::uint8_t> loud = randomFields(128UZ, 8U, 0x2ff1c34cULL);

        for (gr::Size_t k = 1U; k <= 8U; ++k) {
            std::vector<std::uint8_t> masked = loud;
            for (std::uint8_t& item : masked) {
                item = static_cast<std::uint8_t>(item & ((1U << k) - 1U));
            }

            UnpackBits wide   = make<UnpackBits>({{"k", k}});
            UnpackBits narrow = make<UnpackBits>({{"k", k}});
            expect(std::ranges::equal(drive(wide, std::span<const std::uint8_t>(loud)).samples, drive(narrow, std::span<const std::uint8_t>(masked)).samples)) << std::format("k={}: the unused high bits are ignored", k);
        }

        UnpackBits msb = make<UnpackBits>({{"k", 1U}});
        UnpackBits lsb = make<UnpackBits>({{"k", 1U}, {"bit_order", std::string("LsbFirst")}});
        const Run  one = drive(msb, std::span<const std::uint8_t>(loud));
        expect(std::ranges::equal(one.samples, drive(lsb, std::span<const std::uint8_t>(loud)).samples)) << "the two orders agree at k = 1";
        for (std::size_t i = 0UZ; i < loud.size(); ++i) {
            expect(eq(one.samples[i], static_cast<std::uint8_t>(loud[i] & 1U)));
        }
    };

    "a tag reaches the first of the k items its own item became, held across calls if it must be"_test = [] {
        const std::array<std::size_t, 6> at{0UZ, 1UZ, 2UZ, 3UZ, 4UZ, 5UZ};

        UnpackBits           block = make<UnpackBits>();
        std::vector<gr::Tag> tags;
        for (const std::size_t index : at) {
            tags.emplace_back(index, gr::property_map{{gr::property_map::key_type{"signal_name"}, gr::pmt::Value(std::to_string(index))}});
        }
        const std::vector<std::uint8_t> bytes(8UZ, std::uint8_t{0xA5U});
        const Run                       oneAtATime = drive(block, std::span<const std::uint8_t>(bytes), 1UZ, std::span<const gr::Tag>(tags));
        const Run                       allAtOnce  = drive(block, std::span<const std::uint8_t>(bytes), 0UZ, std::span<const gr::Tag>(tags));

        expect(eq(oneAtATime.tags.size(), at.size()));
        expect(eq(allAtOnce.tags.size(), at.size()));
        for (std::size_t which = 0UZ; which < at.size(); ++which) {
            expect(eq(oneAtATime.tags[which].index, 8UZ * at[which])) << std::format("a tag on item {} belongs on bit {}", at[which], 8UZ * at[which]);
            expect(eq(allAtOnce.tags[which].index, oneAtATime.tags[which].index)) << "a tag whose bits fall past the call is held, not moved";
        }
    };

    "the flush setting is inert, because one input item is a whole period"_test = [] {
        const std::vector<std::uint8_t> items = randomFields(29UZ, 8U, 0x8e79dcb0ULL);

        for (gr::Size_t k = 1U; k <= 8U; ++k) {
            UnpackBits padding  = make<UnpackBits>({{"k", k}});
            UnpackBits dropping = make<UnpackBits>({{"k", k}, {"flush_partial", false}});

            expect(eq(static_cast<std::size_t>(padding.input_chunk_size), 1UZ)) << "there is no trailing input to decide about";
            expect(std::ranges::equal(drive(padding, std::span<const std::uint8_t>(items), 0UZ, {}, true).samples, drive(dropping, std::span<const std::uint8_t>(items), 0UZ, {}, true).samples)) << std::format("k={}", k);
        }
    };

    "a block left at its defaults declares its own rate, and a bad width is refused"_test = [] {
        UnpackBits unpacker = make<UnpackBits>();
        expect(eq(static_cast<std::size_t>(unpacker.input_chunk_size), 1UZ));
        expect(eq(static_cast<std::size_t>(unpacker.output_chunk_size), 8UZ)) << "one byte becomes eight bits";

        expect(throws([] { std::ignore = make<UnpackBits>({{"k", 0U}}); }));
        expect(throws([] { std::ignore = make<UnpackBits>({{"k", 9U}}); })) << "nine bits do not fit the byte an unpacked item comes from";
    };
};

const boost::ut::suite<"record repack"> recordRepackTests = [] {
    using namespace boost::ut;
    using gr::blocks::digital::RecordRepackBits;
    using Record = gr::DataSet<std::uint8_t>;

    const auto recordOf = [](std::vector<std::uint8_t> values) {
        Record r;
        r.signal_values = std::move(values);
        r.extents.push_back(static_cast<std::int32_t>(r.signal_values.size()));
        r.signal_names.emplace_back("digital");
        r.timing_events.resize(1UZ);
        r.meta_information.resize(1UZ);
        r.meta_information[0UZ]["origin"] = std::string("qa");
        return r;
    };

    const auto through = [](gr::property_map settings, const std::vector<Record>& records) {
        RecordRepackBits block(std::move(settings));
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start(); // a batch that moves no value calls back nowhere, so this is what configures a block left at its defaults
        return gr::blocks::testing::span::run<Record>(block, std::span<const Record>(records)).samples;
    };

    "a record's bits regroup and the boundary survives"_test = [&] {
        // 24 one-bit items pack to three bytes, msb_first: the stream's first bit is each byte's top bit
        std::vector<std::uint8_t> bits(24UZ, 0U);
        for (const std::size_t at : {0UZ, 5UZ, 8UZ, 15UZ, 23UZ}) {
            bits[at] = 1U;
        }
        const auto packed = through({{"bits_in", gr::Size_t{1U}}, {"bits_out", gr::Size_t{8U}}}, {recordOf(bits)});
        expect(eq(packed.size(), 1UZ));
        if (!packed.empty()) {
            const std::vector<std::uint8_t> want{0x84U, 0x81U, 0x01U};
            expect(std::ranges::equal(packed[0UZ].signal_values, want)) << "the first bit lands at the top of the first byte";
            expect(eq(packed[0UZ].extents[0UZ], std::int32_t{3})) << "the extent names the regrouped length";
            const auto entry = packed[0UZ].meta_information[0UZ].find(gr::property_map::key_type("origin"));
            expect(that % (entry != packed[0UZ].meta_information[0UZ].end())) << "the record's facts cross verbatim";

            // and back: unpacking the packed record reproduces the bits, so the pair is an inverse
            const auto unpacked = through({{"bits_in", gr::Size_t{8U}}, {"bits_out", gr::Size_t{1U}}}, {packed[0UZ]});
            expect(eq(unpacked.size(), 1UZ));
            if (!unpacked.empty()) {
                expect(std::ranges::equal(unpacked[0UZ].signal_values, bits));
            }
        }
    };

    "a remainder is a counted, stated drop and the next record regroups"_test = [&] {
        RecordRepackBits block({{"bits_in", gr::Size_t{1U}}, {"bits_out", gr::Size_t{8U}}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start(); // both values are the defaults, so the batch moves nothing and start is what configures the block

        const std::vector<Record>                     records{recordOf(std::vector<std::uint8_t>(13UZ, 1U)), recordOf(std::vector<std::uint8_t>(16UZ, 1U))};
        std::vector<Record>                           outBuf(4UZ);
        gr::blocks::testing::span::InputSpan<Record>  inSpan{std::span<const Record>(records)};
        gr::blocks::testing::span::OutputSpan<Record> outSpan{std::span<Record>(outBuf)};
        expect(that % (block.processBulk(inSpan, outSpan) == gr::work::Status::OK));
        expect(eq(outSpan.count, 1UZ)) << "13 bits are not a whole number of bytes";
        expect(eq(block.nRecordsRefused, std::uint64_t{1ULL}));
        expect(eq(block.nRecords, std::uint64_t{1ULL}));
        expect(eq(outBuf[0UZ].signal_values.size(), 2UZ)) << "the 16-bit record after it regroups";
    };

    "the record form groups exactly as the kernel does"_test = [&] {
        std::vector<std::uint8_t> bits(64UZ);
        std::uint64_t             lcg = 0x9E3779B97F4A7C15ULL;
        for (std::uint8_t& bit : bits) {
            lcg = lcg * 6364136223846793005ULL + 1442695040888963407ULL;
            bit = static_cast<std::uint8_t>((lcg >> 33U) & 1ULL);
        }
        for (const gr::digital::BitOrder order : {gr::digital::BitOrder::MsbFirst, gr::digital::BitOrder::LsbFirst}) {
            const auto packed = through({{"bits_in", gr::Size_t{1U}}, {"bits_out", gr::Size_t{8U}}, {"output_bit_order", std::string(gr::meta::enumName(order).value_or(""))}}, {recordOf(bits)});

            std::vector<std::uint8_t> want(8UZ);
            gr::digital::detail::packEightBits(std::span<const std::uint8_t>(bits), std::span<std::uint8_t>(want), order);

            expect(eq(packed.size(), 1UZ)) << gr::meta::enumName(order).value_or("");
            if (!packed.empty()) {
                expect(std::ranges::equal(packed[0UZ].signal_values, want)) << gr::meta::enumName(order).value_or("") << ": one grouping rule, two carriers";
            }
        }
    };
};

int main() { /* tests are statically registered */ }
