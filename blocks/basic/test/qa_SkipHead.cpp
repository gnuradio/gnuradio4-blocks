#include <boost/ut.hpp>

#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/basic/SkipHead.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::basic::SkipHead;
using gr::testing::ProcessFunction;
using gr::testing::TagSink;
using gr::testing::TagSource;
namespace span = gr::blocks::testing::span;

template<typename T>
[[nodiscard]] SkipHead<T> make(gr::Size_t nSkip) {
    SkipHead<T> block({{"n_skip", nSkip}});
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

template<typename T>
void setSkip(SkipHead<T>& block, gr::Size_t nSkip) {
    std::ignore = block.settings().setStaged({{"n_skip", nSkip}});
    std::ignore = block.settings().applyStagedParameters();
}

/// @brief Drive @p block over @p input in calls of @p chunkSize samples, calling @p before with the input offset of each.
///
/// Each call runs forwardTags() and then processBulk(), in the framework's order, and the tag cursor hands a tag that
/// a call did not retire to the next call at a negative index.
template<typename T, typename FBefore>
[[nodiscard]] span::Capture<T> drive(SkipHead<T>& block, std::span<const T> input, std::size_t chunkSize, std::span<const gr::Tag> tags, FBefore&& before) {
    span::Capture<T> result;
    std::vector<T>   scratch(chunkSize);
    span::TagCursor  cursor(tags);

    for (std::size_t base = 0UZ; base < input.size();) {
        before(base);
        const std::size_t count = std::min(chunkSize, input.size() - base);

        span::InputSpan<T>  inSpan(input.subspan(base, count), base, cursor.window(base, count));
        span::OutputSpan<T> outSpan(std::span<T>(scratch.data(), count), result.samples.size(), &result.tags);
        auto                inSpans  = std::tie(inSpan);
        auto                outSpans = std::tie(outSpan);

        block.forwardTags(inSpans, outSpans, count);
        std::ignore = block.processBulk(inSpan, outSpan);
        cursor.retire(inSpan);

        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        result.consumed += inSpan.consumed;
        base += inSpan.consumed;
    }
    return result;
}

template<typename T>
[[nodiscard]] span::Capture<T> drive(SkipHead<T>& block, std::span<const T> input, std::size_t chunkSize, std::span<const gr::Tag> tags = {}) {
    return drive<T>(block, input, chunkSize, tags, [](std::size_t) {});
}

template<typename T>
[[nodiscard]] std::vector<T> ramp(std::size_t count, std::size_t from = 0UZ) {
    std::vector<T> values(count);
    for (std::size_t i = 0UZ; i < count; ++i) {
        if constexpr (gr::meta::complex_like<T>) {
            values[i] = T(static_cast<typename T::value_type>(from + i), static_cast<typename T::value_type>(i));
        } else {
            values[i] = static_cast<T>(from + i);
        }
    }
    return values;
}

[[nodiscard]] std::vector<gr::Tag> marks(std::initializer_list<std::size_t> offsets) {
    std::vector<gr::Tag> tags;
    for (const std::size_t offset : offsets) {
        tags.emplace_back(offset, gr::property_map{{"mark", gr::pmt::Value(static_cast<gr::Size_t>(offset))}});
    }
    return tags;
}

/// @brief The input offset each "mark" tag names, listed at its output offset.
[[nodiscard]] std::vector<std::pair<std::size_t, gr::Size_t>> markOffsets(const std::vector<gr::Tag>& tags) {
    std::vector<std::pair<std::size_t, gr::Size_t>> found;
    for (const gr::Tag& tag : tags) {
        if (const auto it = tag.map.find(gr::property_map::key_type{"mark"}); it != tag.map.end()) {
            found.emplace_back(tag.index, it->second.value_or<gr::Size_t>(0U));
        }
    }
    return found;
}

struct GraphRun {
    std::vector<float>   samples{};
    std::vector<gr::Tag> tags{};
};

/// @brief Run a ramp of @p nSamples with "mark" tags at @p tagged through a SkipHead into a TagSink.
[[nodiscard]] GraphRun throughGraph(gr::Size_t nSamples, gr::Size_t nSkip, std::initializer_list<std::size_t> tagged) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", nSamples}, {"mark_tag", false}});
    source._tags     = marks(tagged);
    auto& block      = graph.emplaceBlock<SkipHead<float>>({{"n_skip", nSkip}});
    auto& sink       = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

    boost::ut::expect(graph.connect<"out", "in">(source, block).has_value());
    boost::ut::expect(graph.connect<"out", "in">(block, sink).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value()) << "the run ends without an error";
    return {std::vector<float>(sink._samples.begin(), sink._samples.end()), sink._tags};
}

} // namespace

const boost::ut::suite<"SkipHead"> skipHeadTests = [] {
    using namespace boost::ut;

    "the skip spans work calls and the tail passes unchanged"_test = [] {
        const std::vector<float> x = ramp<float>(40UZ);
        for (const std::size_t chunk : {1UZ, 3UZ, 7UZ, 11UZ, 12UZ, 13UZ, 40UZ}) {
            SkipHead<float>            block = make<float>(12U);
            const span::Capture<float> y     = drive<float>(block, x, chunk);
            expect(eq(y.consumed, x.size())) << std::format("chunk {}: every input is consumed", chunk);
            expect(that % (y.samples == ramp<float>(28UZ, 12UZ))) << std::format("chunk {}: the output is the input from sample 12", chunk);
        }
    };

    "n_skip 0 passes every sample unchanged for every registered type"_test = []<typename T>() {
        SkipHead<T>          block = make<T>(0U);
        const std::vector<T> x     = ramp<T>(100UZ, 1UZ);
        expect(that % (drive<T>(block, x, 32UZ).samples == x));
    } | std::tuple<std::uint8_t, std::int16_t, std::int32_t, float, double, std::complex<float>, std::complex<double>>{};

    "a skip longer than the stream consumes it all and produces nothing"_test = [] {
        SkipHead<float>            block = make<float>(50U);
        const std::vector<float>   x     = ramp<float>(40UZ);
        const span::Capture<float> y     = drive<float>(block, x, 16UZ, marks({0UZ, 39UZ}));
        expect(eq(y.consumed, x.size()));
        expect(y.samples.empty());
        expect(y.tags.empty()) << "the tags of dropped samples are dropped";

        const GraphRun run = throughGraph(40U, 50U, {});
        expect(run.samples.empty()) << "the stream ends during the skip: the block ends with it";
    };

    "a tag on a dropped sample is dropped and a tag on a kept sample stays on it"_test = [] {
        const std::vector<std::pair<std::size_t, gr::Size_t>> want{{0UZ, 5U}, {4UZ, 9U}, {15UZ, 20U}};
        const std::vector<float>                              x = ramp<float>(30UZ);
        for (const std::size_t chunk : {1UZ, 3UZ, 5UZ, 30UZ}) {
            SkipHead<float>            block = make<float>(5U);
            const span::Capture<float> y     = drive<float>(block, x, chunk, marks({0UZ, 4UZ, 5UZ, 9UZ, 20UZ}));
            expect(that % (markOffsets(y.tags) == want)) << std::format("chunk {}: a tag at input t leaves at output t - 5", chunk);
        }

        const GraphRun run = throughGraph(30U, 5U, {0UZ, 4UZ, 5UZ, 9UZ, 20UZ});
        expect(that % (run.samples == ramp<float>(25UZ, 5UZ))) << "through a graph";
        expect(that % (markOffsets(run.tags) == want)) << "through a graph, on the same samples";
    };

    "a change of n_skip after the skip has no effect"_test = [] {
        SkipHead<float>            block = make<float>(3U);
        const std::vector<float>   x     = ramp<float>(20UZ);
        const span::Capture<float> y     = drive<float>(block, x, 1UZ, {}, [&block](std::size_t base) {
            if (base == 5UZ) {
                setSkip(block, 10U);
            }
        });
        expect(that % (y.samples == ramp<float>(17UZ, 3UZ)));
    };

    "a change of n_skip during the skip sets a new count from the start of the stream"_test = [] {
        for (const gr::Size_t changed : {2U, 6U, 15U}) {
            SkipHead<float>            block = make<float>(10U);
            const std::vector<float>   x     = ramp<float>(20UZ);
            const span::Capture<float> y     = drive<float>(block, x, 1UZ, {}, [&block, changed](std::size_t base) {
                if (base == 4UZ) {
                    setSkip(block, changed);
                }
            });

            const std::size_t from = std::max(4UZ, static_cast<std::size_t>(changed));
            expect(that % (y.samples == ramp<float>(20UZ - from, from))) << std::format("changed to {} after 4 dropped", changed);
        }
    };

    "at the end of the stream the tags past the remaining skip leave at the end index"_test = [] {
        SkipHead<float>          block = make<float>(10U);
        const std::vector<float> x     = ramp<float>(6UZ);
        std::ignore                    = drive<float>(block, x, 6UZ);
        expect(eq(block.remainingSkip(), 4UZ));

        const gr::property_map::key_type endOfStream{static_cast<std::pmr::string>(gr::tag::END_OF_STREAM)};
        const std::vector<gr::Tag>       tags{
                  {7UZ, gr::property_map{{"mark", gr::pmt::Value(gr::Size_t{7})}}},
                  {10UZ, gr::property_map{{"mark", gr::pmt::Value(gr::Size_t{10})}, {endOfStream, gr::pmt::Value(true)}}},
        };
        std::vector<gr::Tag>    published;
        span::InputSpan<float>  inSpan(std::span<const float>{}, 6UZ, tags);
        span::OutputSpan<float> outSpan(std::span<float>{}, 0UZ, &published);
        auto                    inSpans  = std::tie(inSpan);
        auto                    outSpans = std::tie(outSpan);
        block.forwardTags(inSpans, outSpans, 0UZ);

        const std::vector<std::pair<std::size_t, gr::Size_t>> want{{0UZ, 10U}};
        expect(that % (markOffsets(published) == want)) << "the tag inside the skip is dropped, the one past it leaves at offset 0";
        expect(published.size() == 1UZ && !published.front().map.contains(endOfStream)) << "without the end-of-stream key";

        const std::vector<gr::Tag> endOnly{{6UZ, gr::property_map{{endOfStream, gr::pmt::Value(true)}}}};
        std::vector<gr::Tag>       none;
        span::InputSpan<float>     endSpan(std::span<const float>{}, 6UZ, endOnly);
        span::OutputSpan<float>    endOut(std::span<float>{}, 0UZ, &none);
        auto                       endSpans    = std::tie(endSpan);
        auto                       endOutSpans = std::tie(endOut);
        block.forwardTags(endSpans, endOutSpans, 0UZ);
        expect(none.empty()) << "a tag that carries only the end-of-stream key publishes nothing";
    };

    "a reset starts the count again"_test = [] {
        SkipHead<float>          block = make<float>(4U);
        const std::vector<float> x     = ramp<float>(10UZ);
        std::ignore                    = drive<float>(block, x, 10UZ);
        block.reset();
        expect(that % (drive<float>(block, x, 10UZ).samples == ramp<float>(6UZ, 4UZ)));
    };
};

int main() { /* not needed for UT */ }
