#ifndef GR4_BLOCKS_FILTER_TEST_STREAM_END_SINK_HPP
#define GR4_BLOCKS_FILTER_TEST_STREAM_END_SINK_HPP

#include <boost/ut.hpp>

#include <cstddef>
#include <format>
#include <initializer_list>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace gr::blocks::filter::testing {

/// @brief The absolute offsets of the tags of @p tags that carry @p key.
[[nodiscard]] inline std::vector<std::size_t> offsetsOf(const std::vector<gr::Tag>& tags, std::string_view key) {
    std::vector<std::size_t> offsets;
    for (const gr::Tag& tag : tags) {
        if (tag.map.contains(gr::property_map::key_type{key})) {
            offsets.push_back(tag.index);
        }
    }
    return offsets;
}

/// @brief Each tag of @p tags as its index and keys, for a failure message.
[[nodiscard]] inline std::string describe(const std::vector<gr::Tag>& tags) {
    std::string out;
    for (const gr::Tag& tag : tags) {
        out += std::format("{}{}:", out.empty() ? "" : " ", tag.index);
        for (const auto& [key, value] : tag.map) {
            out += std::format(" {}", std::string_view(key));
        }
    }
    return out;
}

/**
 * @brief Records every sample and every tag of its input, the tags at the stream's end-of-stream index included.
 *
 * A tag at the index one past the last sample rides no sample, and a sample-by-sample consumer never sees it. This sink
 * reads its input's tag ring when the stream ends and records every tag there, the `end_of_stream` tag among them.
 */
template<typename T>
struct StreamEndSink : gr::Block<StreamEndSink<T>> {
    gr::PortIn<T> in;

    GR_MAKE_REFLECTABLE(StreamEndSink, in);

    std::vector<T>       _samples;
    std::vector<gr::Tag> _tags;

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& input) {
        record(input);
        return gr::work::Status::OK;
    }

    [[nodiscard]] gr::work::Status processEpilogue(gr::InputSpanLike auto& input) {
        record(input);
        const std::size_t end = input.streamIndex + input.size();
        for (const gr::Tag& tag : in.tagReader().get()) {
            if (tag.index >= end) {
                _tags.push_back(tag);
            }
        }
        return gr::work::Status::OK;
    }

    [[nodiscard]] std::vector<std::size_t> offsetsOf(std::string_view key) const { return testing::offsetsOf(_tags, key); }

    /// @brief The index of the `end_of_stream` tag, where the stream's end reached this sink.
    [[nodiscard]] std::optional<std::size_t> endIndex() const {
        const std::vector<std::size_t> ends = offsetsOf(static_cast<std::pmr::string>(gr::tag::END_OF_STREAM));
        return ends.empty() ? std::nullopt : std::optional<std::size_t>(ends.front());
    }

private:
    void record(auto& input) {
        const std::size_t end = input.streamIndex + input.size();
        for (const gr::Tag& tag : input.rawTags) {
            if (tag.index >= input.streamIndex && tag.index < end) {
                _tags.push_back(tag);
            }
        }
        _samples.insert(_samples.end(), input.begin(), input.end());
    }
};

/// @brief A sink that reads the tag ring at the stream's end, beside a consumer that sees a tag only on a sample.
template<typename T>
struct EndSinks {
    StreamEndSink<T>&                                                                       end;
    gr::blocks::testing::TagSink<T, gr::blocks::testing::ProcessFunction::USE_PROCESS_ONE>& samples;
};

/// @brief Emplace both sinks of `EndSinks` in @p graph and connect the output `out` of @p block to each.
template<typename T, typename TBlock>
[[nodiscard]] std::optional<EndSinks<T>> connectEndSinks(gr::Graph& graph, TBlock& block) {
    using namespace gr::blocks::testing;
    auto& end     = graph.emplaceBlock<StreamEndSink<T>>();
    auto& samples = graph.emplaceBlock<TagSink<T, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});
    if (!graph.connect<"out", "in">(block, end).has_value() || !graph.connect<"out", "in">(block, samples).has_value()) {
        return std::nullopt;
    }
    return EndSinks<T>{end, samples};
}

/**
 * @brief Expect a stream of @p outputs samples at both sinks of @p sinks, ending at index @p outputs, with each tag
 * carrying one of @p keys at that end-of-stream index and on no sample.
 */
template<typename T>
void expectAtStreamEnd(const EndSinks<T>& sinks, std::size_t outputs, std::initializer_list<std::string_view> keys, std::string_view label = {}) {
    using namespace boost::ut;
    expect(eq(sinks.end._samples.size(), outputs)) << std::format("{}: every output, and none past the last input", label);
    expect(eq(sinks.samples._samples.size(), outputs)) << std::format("{}: the same outputs at the sample-by-sample consumer", label);
    expect(eq(sinks.end.endIndex().value_or(std::numeric_limits<std::size_t>::max()), outputs)) << std::format("{}: the stream ends one past the last output, tags seen: {}", label, describe(sinks.end._tags));
    for (const std::string_view key : keys) {
        expect(that % (sinks.end.offsetsOf(key) == std::vector<std::size_t>{outputs})) << std::format("{}: {} at the end-of-stream index", label, key);
        expect(that % offsetsOf(sinks.samples._tags, key).empty()) << std::format("{}: {} on no sample of a sample-by-sample consumer", label, key);
    }
}

} // namespace gr::blocks::filter::testing

#endif // GR4_BLOCKS_FILTER_TEST_STREAM_END_SINK_HPP
