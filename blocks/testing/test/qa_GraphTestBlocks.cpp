#include <boost/ut.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/GraphTestBlocks.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

/// @brief Emits 1, 2, ..., `n` in one call and publishes one tag carrying `key` at sample index `at`.
/// An `at` equal to `n` places the tag at the end-of-stream index. An empty `key` publishes no tag.
template<typename T>
struct TagAtSource : gr::Block<TagAtSource<T>> {
    gr::PortOut<T> out;
    gr::Size_t     n  = 4U;
    gr::Size_t     at = 4U;
    std::string    key{"mark"};

    GR_MAKE_REFLECTABLE(TagAtSource, out, n, at, key);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t count = std::min(outSpan.size(), static_cast<std::size_t>(n));
        for (std::size_t i = 0UZ; i < count; ++i) {
            outSpan[i] = static_cast<T>(i + 1UZ);
        }
        if (!key.empty()) {
            gr::property_map map;
            map.insert_or_assign(std::pmr::string(key.begin(), key.end()), gr::pmt::Value(std::string("x")));
            outSpan.publishTag(map, static_cast<std::size_t>(at));
        }
        outSpan.publish(count);
        return gr::work::Status::DONE;
    }
};

/// @brief Runs a TagAtSource into an ExpectSink and returns the error message, empty when the run succeeds.
template<typename T>
std::string runIntoSink(gr::property_map sourceSettings, gr::property_map sinkSettings) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagAtSource<T>>(std::move(sourceSettings));
    auto&     sink   = graph.emplaceBlock<gr::blocks::testing::ExpectSink<T>>(std::move(sinkSettings));
    if (!graph.connect<"out", "in">(source, sink).has_value()) {
        return "connect failed";
    }
    gr::scheduler::Simple scheduler;
    if (auto exchanged = scheduler.exchange(std::move(graph)); !exchanged) {
        return "exchange failed";
    }
    const auto result = scheduler.runAndWait();
    return result.has_value() ? std::string{} : std::string(result.error().message);
}

} // namespace

const boost::ut::suite<"ExpectSink"> expectSinkTests = [] {
    using namespace boost::ut;
    using gr::property_map;
    const std::vector<float> oneToFour{1.f, 2.f, 3.f, 4.f};

    "a tag at the end-of-stream index counts at the sample count"_test = [&] {
        const std::string error = runIntoSink<float>(property_map{{"n", gr::Size_t(4)}, {"at", gr::Size_t(4)}}, //
            property_map{{"expected", oneToFour}, {"tag_key", std::string("mark")}, {"tag_indices", std::vector<gr::Size_t>{4U}}});
        expect(error.empty()) << error;
    };

    "an end-of-stream tag expected elsewhere fails with both index lists"_test = [&] {
        const std::string error = runIntoSink<float>(property_map{{"n", gr::Size_t(4)}, {"at", gr::Size_t(4)}}, //
            property_map{{"expected", oneToFour}, {"tag_key", std::string("mark")}, {"tag_indices", std::vector<gr::Size_t>{3U}}});
        expect(error.contains("tags with key 'mark' at [4], expected at [3]")) << error;
    };

    "a key with the gr: prefix matches tag_key"_test = [&] {
        const std::string error = runIntoSink<float>(property_map{{"n", gr::Size_t(4)}, {"at", gr::Size_t(4)}, {"key", std::string("gr:mark")}}, //
            property_map{{"expected", oneToFour}, {"tag_key", std::string("mark")}, {"tag_indices", std::vector<gr::Size_t>{4U}}});
        expect(error.empty()) << error;
    };

    "a repeated index in tag_indices counts once"_test = [&] {
        const std::string error = runIntoSink<float>(property_map{{"n", gr::Size_t(4)}, {"at", gr::Size_t(1)}}, //
            property_map{{"expected", oneToFour}, {"tag_key", std::string("mark")}, {"tag_indices", std::vector<gr::Size_t>{1U, 1U}}});
        expect(error.empty()) << error;
    };

    "uint32 samples are compared"_test = [] {
        const std::string match = runIntoSink<std::uint32_t>(property_map{{"n", gr::Size_t(4)}, {"key", std::string()}}, //
            property_map{{"expected", std::vector<std::uint32_t>{1U, 2U, 3U, 4U}}});
        expect(match.empty()) << match;
        const std::string mismatch = runIntoSink<std::uint32_t>(property_map{{"n", gr::Size_t(4)}, {"key", std::string()}}, //
            property_map{{"expected", std::vector<std::uint32_t>{1U, 2U, 9U, 4U}}});
        expect(mismatch.contains("sample 2 is 3, expected 9")) << mismatch;
    };
};

const boost::ut::suite<"VectorSource"> vectorSourceTests = [] {
    using namespace boost::ut;

    "values shorter than the emitted count end the stream with nothing emitted"_test = [] {
        gr::blocks::testing::VectorSource<float> source;
        source.values   = std::vector<float>{1.f, 2.f, 3.f};
        source._emitted = 5UZ;
        std::vector<float>                           buffer(4UZ, 0.f);
        gr::blocks::testing::span::OutputSpan<float> outSpan{std::span<float>(buffer)};
        const gr::work::Status                       status = source.processBulk(outSpan);
        expect(status == gr::work::Status::DONE);
        expect(eq(outSpan.count, 0UZ));
        expect(eq(source._emitted, 5UZ));
    };
};

int main() { /* not needed for UT */ }
