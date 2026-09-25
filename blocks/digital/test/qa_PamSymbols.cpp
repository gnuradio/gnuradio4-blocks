#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <numbers>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/digital/PamSymbols.hpp>

#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_pam_symbols {

using gr::blocks::digital::LevelTracker;
using gr::blocks::digital::PamSlicer;
using gr::testing::ProcessFunction;
using gr::testing::TagSink;
using gr::testing::TagSource;

/// A block carrying a seqlock is neither copyable nor movable, so every one below is built where it is used.
template<typename TBlock>
void init(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    if constexpr (requires { block.start(); }) {
        block.start();
    }
}

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    if constexpr (requires { block.start(); }) {
        block.start();
    }
    return block;
}

/// A deterministic four-level symbol stream on the ±1/±3 grid, scaled and shifted.
[[nodiscard]] std::vector<float> symbols(std::size_t count, double scale, double shift, std::uint64_t seed) {
    std::uint64_t      state = seed;
    std::vector<float> out(count);
    for (float& v : out) {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        const int level = 2 * static_cast<int>(state & 3U) - 3; // -3 -1 +1 +3
        v               = static_cast<float>(static_cast<double>(level) * scale + shift);
    }
    return out;
}

/// @brief Drive the tracker at an exact chunk size with its record port unwired, which is what these cases measure.
[[nodiscard]] std::vector<float> run(LevelTracker<float>& block, std::span<const float> input, std::size_t chunk = 0UZ) {
    namespace test = gr::blocks::testing::span;
    std::vector<float> out(input.size());
    const std::size_t  stride = chunk == 0UZ ? input.size() : chunk;
    for (std::size_t base = 0UZ; base < input.size(); base += stride) {
        const std::size_t                    until = std::min(base + stride, input.size());
        test::InputSpan<float>               inSpan(input.subspan(base, until - base), base);
        test::OutputSpan<float>              outSpan(std::span<float>(out).subspan(base, until - base), base);
        test::OutputSpan<gr::DataSet<float>> recordSpan(std::span<gr::DataSet<float>>{}, 0UZ, nullptr, false);

        std::ignore = block.processBulk(inSpan, outSpan, recordSpan);
    }
    return out;
}

/// @brief Keeps every record that reached it, which is all a graph reading the port wants from it.
struct RecordCollector : gr::Block<RecordCollector> {
    gr::PortIn<gr::DataSet<float>> in;
    GR_MAKE_REFLECTABLE(RecordCollector, in);

    std::vector<gr::DataSet<float>> _records{};

    gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (std::size_t k = 0UZ; k < inSpan.size(); ++k) {
            _records.push_back(inSpan[k]);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/// One scheduler-driven run: the normalized stream, what the record port published, and the readings after it.
struct Run {
    std::vector<float>              samples{};
    std::vector<gr::DataSet<float>> records{};
    double                          spread = 0.0;
    double                          offset = 0.0;
};

/// @brief Run @p input through a tracker under the scheduler, the record port wired to a collector or left unwired.
[[nodiscard]] Run throughGraph(std::span<const float> input, bool wireRecords) {
    gr::Graph  graph;
    const auto values  = gr::Tensor<float>(input.begin(), input.end());
    auto&      source  = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(input.size())}, {"values", values}, {"mark_tag", false}});
    auto&      tracker = graph.emplaceBlock<LevelTracker<float>>();
    auto&      sink    = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_BULK>>({{"name", "TagSink"}});

    boost::ut::expect(graph.connect<"out", "in">(source, tracker).has_value());
    boost::ut::expect(graph.connect<"out", "in">(tracker, sink).has_value());

    RecordCollector* collector = nullptr;
    if (wireRecords) {
        collector = &graph.emplaceBlock<RecordCollector>();
        boost::ut::expect(graph.connect<"records", "in">(tracker, *collector).has_value());
    }

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    const auto finished = scheduler.runAndWait();
    boost::ut::expect(finished.has_value()) << (finished.has_value() ? std::string{} : finished.error().message);

    Run result{std::vector<float>(sink._samples.begin(), sink._samples.end()), {}, tracker.spread(), tracker.offset()};
    if (collector != nullptr) {
        result.records = std::move(collector->_records);
    }
    return result;
}

} // namespace qa_pam_symbols

using namespace qa_pam_symbols;

const boost::ut::suite<"PamSymbols"> pamSymbolsTests = [] {
    using namespace boost::ut;

    static_assert(!gr::HasConstProcessOneFunction<LevelTracker<float>>, "the tracker carries its loops");
    static_assert(gr::HasConstProcessOneFunction<PamSlicer<float>>, "the slicer is a pure table");

    "a clean stream settles spread at nominal and offset at zero"_test = [] {
        LevelTracker<float> block;
        init(block);
        const auto in  = symbols(6000UZ, 1.0, 0.0, 0x9e3779b97f4a7c15ULL);
        const auto out = run(block, in);
        expect(lt(std::abs(block.spread() - 2.0), 0.02)) << std::format("spread {}", block.spread());
        expect(lt(std::abs(block.offset()), 0.02)) << std::format("offset {}", block.offset());
        // Settled, the output IS the input: the grid was already nominal.
        double worst = 0.0;
        for (std::size_t i = 4000UZ; i < out.size(); ++i) {
            worst = std::max(worst, static_cast<double>(std::abs(out[i] - in[i])));
        }
        expect(lt(worst, 0.05)) << "the normalization is transparent on a nominal stream";
    };

    "a deviation 15 % high or low is measured, not just tolerated"_test = [] {
        for (const double scale : {1.15, 0.85}) {
            LevelTracker<float> block;
            init(block);
            const auto in = symbols(8000UZ, scale, 0.0, 0x2545F4914F6CDD1DULL);
            std::ignore   = run(block, in);
            expect(lt(std::abs(block.spread() - 2.0 * scale), 0.05)) << std::format("scale {}: spread {}", scale, block.spread());
            expect(lt(std::abs(block.offset()), 0.05));
        }
    };

    "a static shift up to four units is measured and removed, spacing undisturbed"_test = [] {
        for (const double shift : {-4.0, -1.25, 0.75, 4.0}) {
            LevelTracker<float> block;
            init(block);
            const auto in  = symbols(8000UZ, 1.0, shift, 0x123456789abcdefULL);
            const auto out = run(block, in);
            expect(lt(std::abs(block.offset() - shift), 0.05)) << std::format("shift {}: offset {}", shift, block.offset());
            expect(lt(std::abs(block.spread() - 2.0), 0.05)) << "a common shift does not disturb the spacing";
            // Settled, the output sits back on the nominal grid.
            double worst = 0.0;
            for (std::size_t i = 6000UZ; i < out.size(); ++i) {
                const float nearest = 2.0f * std::round(out[i] / 2.0f + 0.5f) - 1.0f; // nearest odd level
                worst               = std::max(worst, static_cast<double>(std::abs(out[i] - std::clamp(nearest, -3.0f, 3.0f))));
            }
            expect(lt(worst, 0.1)) << "settled symbols land on the grid";
        }
    };

    "the clamp bounds the spacing and lets it return"_test = [] {
        LevelTracker<float> block;
        init(block);
        std::ignore = run(block, symbols(8000UZ, 2.0, 0.0, 0xdeadbeefULL)); // deviation doubled
        expect(lt(std::abs(block.spread() - 2.4), 1e-6)) << "clamped at nominal * 1.2";
        std::ignore = run(block, symbols(8000UZ, 1.0, 0.0, 0xdeadbeefULL));
        expect(lt(std::abs(block.spread() - 2.0), 0.05)) << "and returns once the stream does";
    };

    "the tracker does not depend on the chunking"_test = [] {
        const auto          in = symbols(5000UZ, 1.1, -0.6, 0xfeedfaceULL);
        LevelTracker<float> reference;
        init(reference);
        const auto want = run(reference, in);
        for (const std::size_t chunk : {1UZ, 7UZ, 997UZ}) {
            LevelTracker<float> block;
            init(block);
            expect(that % (run(block, in, chunk) == want)) << std::format("chunk {}", chunk);
        }
    };

    "the slicer's decision table, thresholds and label maps"_test = [] {
        auto identity = make<PamSlicer<float>>();
        auto c4fm     = make<PamSlicer<float>>({{"labels", std::vector<gr::Size_t>{3U, 2U, 0U, 1U}}});

        struct Case {
            float        x;
            std::uint8_t rank;
        };
        // Ranks by region: the thresholds sit at -2, 0, +2 with ties toward the upper region,
        // and the outer regions are unbounded.
        for (const Case c : {Case{-1e30f, 0}, Case{-3.f, 0}, Case{-2.01f, 0}, Case{-2.f, 1}, Case{-1.f, 1}, Case{-0.01f, 1}, Case{0.f, 2}, Case{1.f, 2}, Case{1.99f, 2}, Case{2.f, 3}, Case{3.f, 3}, Case{1e30f, 3}}) {
            expect(eq(identity.processOne(c.x), c.rank)) << std::format("identity at {}", c.x);
            constexpr std::uint8_t kC4fm[4]{3U, 2U, 0U, 1U};
            expect(eq(c4fm.processOne(c.x), kC4fm[c.rank])) << std::format("c4fm at {}", c.x);
        }
    };

    "refusals fire by name"_test = [] {
        expect(throws([] { std::ignore = make<PamSlicer<float>>({{"labels", std::vector<gr::Size_t>{0U, 1U}}}); })) << "a label per level";
        expect(throws([] { std::ignore = make<PamSlicer<float>>({{"n_levels", gr::Size_t(3)}, {"labels", std::vector<gr::Size_t>{0U, 1U, 2U}}}); })) << "even M only";
        expect(throws([] {
            LevelTracker<float> tracker({{"n_levels", gr::Size_t(3)}});
            init(tracker);
        })) << "even M only";
    };

    "the record port carries what the readers carry, and an unwired one changes nothing"_test = [] {
        constexpr double kScale = 1.1;
        constexpr double kShift = -0.6;
        const auto       in     = symbols(8000UZ, kScale, kShift, 0x51ed270bULL);

        const Run wired = throughGraph(in, true);
        expect(!wired.records.empty()) << "a graph that wires the port receives records";
        if (wired.records.empty()) {
            return;
        }

        const auto& last = wired.records.back();
        expect(eq(last.signal_names.size(), 2UZ));
        expect(eq(last.signal_names[0UZ], std::string("spread")));
        expect(eq(last.signal_names[1UZ], std::string("offset")));
        expect(eq(last.signal_values[0UZ], static_cast<float>(wired.spread))) << "the record carries the reading a poller sees";
        expect(eq(last.signal_values[1UZ], static_cast<float>(wired.offset)));
        expect(lt(std::abs(static_cast<double>(last.signal_values[0UZ]) - 2.0 * kScale), 0.05)) << "which is the spacing the stream really has";
        expect(lt(std::abs(static_cast<double>(last.signal_values[1UZ]) - kShift), 0.05)) << "and the shift it really carries";

        const auto& meta = last.meta_information[0UZ];
        expect(meta.find(std::pmr::string("sample_rate")) == meta.end()) << "a symbol-domain record states no sample rate rather than a zero one";
        expect(meta.find(std::pmr::string("sample_start")) != meta.end());
        expect(meta.find(std::pmr::string("index_unit")) != meta.end()) << "and says what its index counts";

        const Run bare = throughGraph(in, false);
        expect(bare.records.empty()) << "an unwired port has nothing published to it";
        expect(eq(bare.spread, wired.spread)) << "the tracking does not depend on whether anyone is listening";
        expect(eq(bare.offset, wired.offset));
        expect(that % (bare.samples == wired.samples)) << "nor does the stream the block normalizes";
    };

    "an idle tracker answers what it waits for, not progress"_test = [] {
        namespace test = gr::blocks::testing::span;
        LevelTracker<float> block;
        block.start();
        std::vector<float>                   room(4UZ);
        const std::vector<float>             symbols{1.f, -1.f, 3.f, -3.f};
        test::OutputSpan<gr::DataSet<float>> recordSpan(std::span<gr::DataSet<float>>{}, 0UZ, nullptr, false);

        test::InputSpan<float>  empty{std::span<const float>{}};
        test::OutputSpan<float> outSpan{std::span<float>(room)};
        expect(block.processBulk(empty, outSpan, recordSpan) == gr::work::Status::INSUFFICIENT_INPUT_ITEMS) << "an empty input";

        test::InputSpan<float>  inSpan{std::span<const float>(symbols)};
        test::OutputSpan<float> full{std::span<float>{}};
        expect(block.processBulk(inSpan, full, recordSpan) == gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS) << "a full output";
        expect(eq(inSpan.consumed, 0UZ));
    };
};

int main() { /* not needed for UT */ }
