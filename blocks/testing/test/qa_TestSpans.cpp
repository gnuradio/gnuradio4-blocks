#include <boost/ut.hpp>

#include <algorithm>
#include <cstddef>
#include <numeric>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using namespace gr::blocks::testing::span;

/// @brief What one call was handed: where it starts, how many samples it covers, and the tags it can see.
struct CallRecord {
    std::size_t                                              streamIndex = 0UZ;
    std::size_t                                              nSamples    = 0UZ;
    std::vector<std::pair<std::ptrdiff_t, gr::property_map>> tags{};
};

/// @brief One call's record, taken the same way from a driven mock span and from a port's own span.
CallRecord recordOf(auto& span) {
    CallRecord record{span.streamIndex, span.size(), {}};
    for (const auto& [relativeIndex, map] : span.tags()) {
        record.tags.emplace_back(relativeIndex, map.get());
    }
    return record;
}

gr::Tag tagAt(std::size_t index, int id) {
    gr::property_map map;
    map["id"] = id;
    return gr::Tag{index, std::move(map)};
}

/// @brief Records the tag view of every call and consumes a fixed number of samples from each.
struct TagRecorder {
    std::size_t                consumePerCall = 1UZ;
    std::optional<std::size_t> firstCallConsume{};     ///< what the first call consumes, where it differs
    std::optional<std::size_t> firstCallConsumeTags{}; ///< the first call's own tag-retirement request

    std::vector<CallRecord> calls{};

    gr::work::Status processBulk(auto& in, auto& out) {
        calls.push_back(recordOf(in));
        const bool isFirst = calls.size() == 1UZ;
        if (isFirst && firstCallConsumeTags.has_value()) {
            in.consumeTags(firstCallConsumeTags.value());
        }
        const std::size_t wanted = isFirst && firstCallConsume.has_value() ? firstCallConsume.value() : consumePerCall;
        const std::size_t taken  = std::min(wanted, in.size());
        const std::size_t made   = std::min(taken, out.size());
        for (std::size_t k = 0UZ; k < made; ++k) {
            out[k] = in[k];
        }
        out.publish(made);
        std::ignore = in.consume(taken);
        return gr::work::Status::OK;
    }
};

} // namespace

const boost::ut::suite TestSpanTagCursorTests = [] {
    using namespace boost::ut;

    "a tag inside an unconsumed chunk comes round at a negative index"_test = [] {
        const std::vector<int>     input(6UZ, 1);
        const std::vector<gr::Tag> tags{tagAt(1UZ, 1)};

        TagRecorder block{.consumePerCall = 2UZ};
        std::ignore = runAsync<int>(block, std::span<const int>(input), 2UZ, 4UZ, std::span<const gr::Tag>(tags));

        expect(eq(block.calls.size(), 3UZ)) << "three calls of two samples" << fatal;
        expect(eq(block.calls[0UZ].tags.size(), 1UZ)) << "the tag lies inside the first chunk" << fatal;
        expect(eq(block.calls[0UZ].tags[0UZ].first, std::ptrdiff_t{1}));
        expect(eq(block.calls[1UZ].tags.size(), 1UZ)) << "the first call retires only its first tag" << fatal;
        expect(eq(block.calls[1UZ].tags[0UZ].first, std::ptrdiff_t{-1}));
        expect(block.calls[1UZ].tags[0UZ].second == tags[0UZ].map) << "the same tag";
        expect(eq(block.calls[2UZ].tags.size(), 0UZ)) << "the second call retires it";
    };

    "a tag the block retired is not handed over again"_test = [] {
        const std::vector<int>     input(6UZ, 1);
        const std::vector<gr::Tag> tags{tagAt(1UZ, 1)};

        TagRecorder block{.consumePerCall = 2UZ, .firstCallConsume = 0UZ, .firstCallConsumeTags = 2UZ};
        std::ignore = runAsync<int>(block, std::span<const int>(input), 2UZ, 4UZ, std::span<const gr::Tag>(tags));

        expect(ge(block.calls.size(), 2UZ)) << "the block consumes nothing on its first call" << fatal;
        expect(eq(block.calls[0UZ].tags.size(), 1UZ));
        for (std::size_t call = 1UZ; call < block.calls.size(); ++call) {
            expect(eq(block.calls[call].tags.size(), 0UZ)) << "call" << call;
        }
    };

    "the first-tag rule retains what the whole-window rule retires"_test = [] {
        const std::vector<int>     input(4UZ, 1);
        const std::vector<gr::Tag> tags{tagAt(1UZ, 1), tagAt(3UZ, 3)};

        TagCursor byFirstTag(std::span<const gr::Tag>(tags), TagRetirement::FirstTagOnly);
        TagCursor byWholeWindow(std::span<const gr::Tag>(tags), TagRetirement::WholeWindow);

        InputSpan<int> firstTagSpan(std::span<const int>(input), 0UZ, byFirstTag.window(0UZ, 4UZ));
        std::ignore = firstTagSpan.consume(4UZ);
        byFirstTag.retire(firstTagSpan);

        InputSpan<int> wholeWindowSpan(std::span<const int>(input), 0UZ, byWholeWindow.window(0UZ, 4UZ));
        std::ignore = wholeWindowSpan.consume(4UZ);
        byWholeWindow.retire(wholeWindowSpan);

        expect(eq(firstTagSpan.rawTags.size(), 2UZ)) << "both tags lie inside the window";
        expect(eq(byFirstTag.retired(), 0UZ)) << "neither tag is at relative index 0";
        expect(eq(byWholeWindow.retired(), 2UZ)) << "both lie below the consumed prefix";

        const InputSpan<int> nextSpan(std::span<const int>(input), 4UZ, byFirstTag.window(4UZ, 4UZ));
        const auto           view = nextSpan.tags();
        expect(eq(view.size(), 2UZ)) << "the next call is handed both again" << fatal;
        expect(eq(view[0UZ].first, std::ptrdiff_t{-3}));
        expect(eq(view[1UZ].first, std::ptrdiff_t{-1}));
        expect(eq(byWholeWindow.window(4UZ, 4UZ).size(), 0UZ));
    };

    "the driver and a port agree span for span"_test = [] {
        constexpr std::size_t feed           = 2UZ;
        constexpr std::size_t consumePerCall = 2UZ;

        std::vector<int> input(8UZ);
        std::iota(input.begin(), input.end(), 0);
        const std::vector<gr::Tag> tags{tagAt(1UZ, 1), tagAt(2UZ, 2), tagAt(5UZ, 5)};

        TagRecorder block{.consumePerCall = consumePerCall};
        std::ignore = runAsync<int>(block, std::span<const int>(input), feed, 8UZ, std::span<const gr::Tag>(tags));

        gr::PortIn<int> port;
        auto            streamWriter = port.buffer().streamBuffer.new_writer();
        auto            tagWriter    = port.buffer().tagBuffer.new_writer();
        {
            auto published = tagWriter.tryReserve(tags.size());
            expect(eq(published.size(), tags.size())) << fatal;
            std::ranges::copy(tags, published.begin());
            published.publish(tags.size());
        }

        std::vector<CallRecord> oracle;
        std::size_t             consumed = 0UZ;
        std::size_t             fed      = 0UZ;
        while (consumed < input.size()) {
            const std::size_t arriving = std::min(feed, input.size() - fed);
            if (arriving > 0UZ) {
                auto written = streamWriter.tryReserve<gr::SpanReleasePolicy::ProcessAll>(arriving);
                expect(eq(written.size(), arriving)) << fatal;
                std::ranges::copy(std::span<const int>(input).subspan(fed, arriving), written.begin());
                written.publish(arriving);
                fed += arriving;
            }
            std::size_t taken = 0UZ;
            { // the port retires the call's tags where the span goes out of scope
                auto span = port.get<gr::SpanReleasePolicy::ProcessNone, true>(port.streamReader().available());
                oracle.push_back(recordOf(span));
                taken       = std::min(consumePerCall, span.size());
                std::ignore = span.consume(taken);
            }
            consumed += taken;
        }

        expect(eq(block.calls.size(), oracle.size())) << "the same number of calls" << fatal;
        for (std::size_t call = 0UZ; call < oracle.size(); ++call) {
            expect(eq(block.calls[call].streamIndex, oracle[call].streamIndex)) << "call" << call << "position";
            expect(eq(block.calls[call].nSamples, oracle[call].nSamples)) << "call" << call << "sample count";
            expect(eq(block.calls[call].tags.size(), oracle[call].tags.size())) << "call" << call << "tag count";
            for (std::size_t k = 0UZ; k < std::min(block.calls[call].tags.size(), oracle[call].tags.size()); ++k) {
                expect(eq(block.calls[call].tags[k].first, oracle[call].tags[k].first)) << "call" << call << "tag" << k << "index";
                expect(block.calls[call].tags[k].second == oracle[call].tags[k].second) << "call" << call << "tag" << k << "content";
            }
        }
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
