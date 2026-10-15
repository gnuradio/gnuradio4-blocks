#include <boost/ut.hpp>

#include <array>
#include <complex>
#include <cstdint>
#include <format>
#include <span>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/testing/GraphTestBlocks.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>
#include <gnuradio-4.0/testing/VectorSink.hpp>

namespace {

template<typename T>
[[nodiscard]] std::vector<T> makeRamp(std::size_t n) {
    std::vector<T> values(n);
    for (std::size_t i = 0UZ; i < n; ++i) {
        if constexpr (requires { typename T::value_type; }) {
            using V   = typename T::value_type;
            values[i] = T{static_cast<V>(i + 1UZ), -static_cast<V>(i)};
        } else {
            values[i] = static_cast<T>(i + 1UZ);
        }
    }
    return values;
}

template<typename TScheduler>
void run(gr::Graph&& graph, TScheduler& sched) {
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
    }
    boost::ut::expect(sched.runAndWait().has_value());
}

} // namespace

const boost::ut::suite<"VectorSink"> vectorSinkTests = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::testing;
    namespace test = gr::blocks::testing::span;

    "VectorSource to VectorSink round trip"_test = []<typename T>(const T&) {
        const std::vector<T> input = makeRamp<T>(37UZ);

        Graph g;
        auto& src  = g.emplaceBlock<VectorSource<T>>(property_map{{"values", input}});
        auto& sink = g.emplaceBlock<VectorSink<T>>();
        expect(fatal(g.connect<"out", "in">(src, sink).has_value()));
        scheduler::Simple sched;
        run(std::move(g), sched);

        const std::vector<T> received = sink.data();
        expect(fatal(eq(received.size(), input.size())));
        expect(received == input);
    } | std::tuple<uint8_t, int16_t, int32_t, float, double, std::complex<float>>();

    "VectorSink with complex<double>"_test = [] {
        using CD                    = std::complex<double>;
        const std::vector<CD> input = {CD{1.5, -2.5}, CD{3.0, 4.0}, CD{-0.5, 0.5}, CD{10.0, -10.0}};

        VectorSink<CD>      sink;
        test::InputSpan<CD> span{std::span<const CD>(input)};
        expect(sink.processBulk(span) == work::Status::OK);

        const std::vector<CD> received = sink.data();
        expect(fatal(eq(received.size(), input.size())));
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            expect(eq(received[i].real(), input[i].real()));
            expect(eq(received[i].imag(), input[i].imag()));
        }
    };

    "VectorSink with uint32_t"_test = [] {
        const std::vector<uint32_t> input = {1000U, 2000U, 3000U, 4000U};
        VectorSink<uint32_t>        sink;
        test::InputSpan<uint32_t>   span{std::span<const uint32_t>(input)};
        expect(sink.processBulk(span) == work::Status::OK);
        const std::vector<uint32_t> received = sink.data();
        expect(fatal(eq(received.size(), input.size())));
        expect(received == input);
    };

    "VectorSink skips negative relative tags from history window"_test = [] {
        VectorSink<float>        sink;
        const std::vector<float> chunk    = {1.f, 2.f, 3.f};
        const std::vector<Tag>   incoming = {{0UZ, {{"k0", "hist"}}}, {6UZ, {{"k1", "curr"}}}};
        test::InputSpan<float>   input{std::span<const float>(chunk), 5UZ, std::span<const Tag>(incoming)};
        expect(sink.processBulk(input) == work::Status::OK);

        const std::vector<Tag> tags = sink.tags();
        expect(fatal(eq(tags.size(), 1UZ)));
        expect(eq(tags[0].index, 1UZ));
        expect(tags[0].map.contains("k1"));
    };

    "VectorSink processEpilogue records trailing samples and end-of-stream user tags"_test = [] {
        VectorSink<float>        sink;
        const std::vector<float> chunk    = {10.f, 20.f};
        const std::vector<Tag>   incoming = {{2UZ, {{"user_eof", "payload"}, {"end_of_stream", true}}}};
        test::InputSpan<float>   input{std::span<const float>(chunk), 0UZ, std::span<const Tag>(incoming)};
        expect(sink.processEpilogue(input) == work::Status::OK);
        expect(fatal(eq(sink.size(), 2UZ)));
        expect(sink.data() == chunk);

        const std::vector<Tag> tags = sink.tags();
        expect(fatal(eq(tags.size(), 1UZ)));
        expect(eq(tags[0].index, 2UZ));
        expect(tags[0].map.contains("user_eof"));
        expect(!tags[0].map.contains("end_of_stream"));
    };

    "VectorSink processEpilogue does not record tags after truncated samples"_test = [] {
        VectorSink<float> sink;
        sink.n_samples_max = 1UZ;

        const std::vector<float> chunk    = {10.f, 20.f};
        const std::vector<Tag>   incoming = {{2UZ, {{"user_eof", "payload"}}}};
        test::InputSpan<float>   input{std::span<const float>(chunk), 0UZ, std::span<const Tag>(incoming)};
        expect(sink.processEpilogue(input) == work::Status::DONE);
        expect(fatal(eq(sink.size(), 1UZ)));
        expect(sink.tags().empty());
    };

    "VectorSink records tags at sample offsets from VectorSource"_test = [] {
        const std::vector<float> input = {10.f, 20.f, 30.f, 40.f, 50.f};
        property_map             tagMap;
        tagMap.insert_or_assign("marker", pmt::Value(std::string("sync")));

        Graph g;
        auto& src  = g.emplaceBlock<VectorSource<float>>(property_map{{"values", input}, {"tag_indices", std::vector<gr::Size_t>{1U, 3U}}, {"tag_map", tagMap}});
        auto& sink = g.emplaceBlock<VectorSink<float>>();
        expect(fatal(g.connect<"out", "in">(src, sink).has_value()));
        scheduler::Simple sched;
        run(std::move(g), sched);

        expect(fatal(eq(sink.size(), input.size())));
        const std::vector<Tag> tags = sink.tags();
        expect(fatal(eq(tags.size(), 2UZ)));
        expect(eq(tags[0].index, 1UZ));
        expect(eq(tags[1].index, 3UZ));
        expect(tags[0].map.contains("marker"));
        expect(tags[1].map.contains("marker"));
    };

    "VectorSink n_samples_max caps collection from repeating source"_test = [] {
        constexpr std::uint64_t  kMax    = 11UZ;
        const std::vector<float> pattern = {10.f, 20.f, 30.f};

        Graph g;
        auto& src          = g.emplaceBlock<VectorSource<float>>(property_map{{"values", pattern}, {"repeat", 100U}});
        auto& sink         = g.emplaceBlock<VectorSink<float>>();
        sink.n_samples_max = kMax;
        expect(fatal(g.connect<"out", "in">(src, sink).has_value()));
        scheduler::Simple sched;
        run(std::move(g), sched);

        const std::vector<float> received = sink.data();
        expect(fatal(eq(received.size(), static_cast<std::size_t>(kMax))));
        for (std::size_t i = 0UZ; i < received.size(); ++i) {
            expect(eq(received[i], pattern[i % pattern.size()])) << std::format("at index {}", i);
        }
    };

    "VectorSink n_samples_max consumes exactly the samples it records"_test = [] {
        VectorSink<float> sink;
        sink.n_samples_max = 11UZ;

        const std::vector<float> chunk = makeRamp<float>(6UZ);
        test::InputSpan<float>   first{std::span<const float>(chunk)};
        expect(sink.processBulk(first) == work::Status::OK);
        expect(!first.consumeRequested) << "below the limit the whole window is consumed by default";

        test::InputSpan<float> second{std::span<const float>(chunk), 6UZ};
        expect(sink.processBulk(second) == work::Status::DONE);
        expect(second.consumeRequested);
        expect(eq(second.consumed, 5UZ));

        const std::vector<float> received = sink.data();
        expect(fatal(eq(received.size(), 11UZ)));
        expect(eq(received[10], chunk[4]));
    };

    "VectorSink n_samples_max preserves sample accounting across take"_test = [] {
        VectorSink<float> sink;
        sink.n_samples_max = 6UZ;

        const std::vector<float> chunk1 = makeRamp<float>(3UZ);
        const std::vector<Tag>   tag1   = {{1UZ, {{"batch", "first"}}}};
        test::InputSpan<float>   first{std::span<const float>(chunk1), 0UZ, std::span<const Tag>(tag1)};
        expect(sink.processBulk(first) == work::Status::OK);
        expect(eq(sink.size(), 3UZ));
        const std::vector<Tag> tags1 = sink.tags();
        expect(fatal(eq(tags1.size(), 1UZ)));
        expect(eq(tags1[0].index, 1UZ));

        const std::vector<float> taken = sink.take();
        expect(taken == chunk1);
        expect(eq(sink.size(), 0UZ));
        expect(sink.tags().empty());

        const std::vector<float> chunk2 = {10.f, 20.f, 30.f, 40.f, 50.f};
        const std::vector<Tag>   tag2   = {{4UZ, {{"batch", "second"}}}};
        test::InputSpan<float>   second{std::span<const float>(chunk2), 3UZ, std::span<const Tag>(tag2)};
        expect(sink.processBulk(second) == work::Status::DONE);
        expect(second.consumeRequested);
        expect(eq(second.consumed, 3UZ));
        expect(eq(sink.size(), 3UZ));
        const std::vector<Tag> tags2 = sink.tags();
        expect(fatal(eq(tags2.size(), 1UZ)));
        expect(eq(tags2[0].index, 1UZ));
    };

    "VectorSink keeps mid-window tags and drops tags past n_samples_max"_test = [] {
        VectorSink<float> sink;
        sink.n_samples_max = 4UZ;

        const std::vector<float> chunk    = makeRamp<float>(6UZ);
        const std::vector<Tag>   incoming = {{1UZ, {{"key", "kept"}}}, {5UZ, {{"key", "dropped"}}}};
        test::InputSpan<float>   input{std::span<const float>(chunk), 0UZ, std::span<const Tag>(incoming)};
        expect(sink.processBulk(input) == work::Status::DONE);
        expect(eq(input.consumed, 4UZ));

        const std::vector<Tag> tags = sink.tags();
        expect(fatal(eq(tags.size(), 1UZ)));
        expect(eq(tags[0].index, 1UZ));
    };

    "VectorSink start clears the previous record"_test = [] {
        VectorSink<int16_t>        sink;
        const std::vector<int16_t> chunk = {1, 2, 3};
        test::InputSpan<int16_t>   input{std::span<const int16_t>(chunk)};
        expect(sink.processBulk(input) == work::Status::OK);
        expect(eq(sink.size(), 3UZ));

        sink.start();
        expect(eq(sink.size(), 0UZ));
        expect(sink.tags().empty());
    };

    "VectorSink take leaves the record empty"_test = [] {
        VectorSink<int32_t>        sink;
        const std::vector<int32_t> chunk    = {100, 200, 300};
        const std::vector<Tag>     incoming = {{1UZ, {{"k", "v"}}}};
        test::InputSpan<int32_t>   input{std::span<const int32_t>(chunk), 0UZ, std::span<const Tag>(incoming)};
        expect(sink.processBulk(input) == work::Status::OK);
        expect(eq(sink.size(), 3UZ));
        expect(eq(sink.tags().size(), 1UZ));

        const std::vector<int32_t> taken = sink.take();
        expect(taken == chunk);
        expect(eq(sink.size(), 0UZ));
        expect(sink.tags().empty());
    };

    "VectorSink clear empties samples and tags"_test = [] {
        VectorSink<float>        sink;
        const std::vector<float> chunk    = {1.f, 2.f};
        const std::vector<Tag>   incoming = {{0UZ, {{"k", "v"}}}};
        test::InputSpan<float>   input{std::span<const float>(chunk), 0UZ, std::span<const Tag>(incoming)};
        expect(sink.processBulk(input) == work::Status::OK);
        expect(eq(sink.size(), 2UZ));
        expect(eq(sink.tags().size(), 1UZ));

        sink.clear();
        expect(eq(sink.size(), 0UZ));
        expect(sink.tags().empty());

        const std::vector<float> postChunk = {10.f, 20.f};
        const std::vector<Tag>   postTag   = {{3UZ, {{"post", "tag"}}}};
        test::InputSpan<float>   postInput{std::span<const float>(postChunk), 2UZ, std::span<const Tag>(postTag)};
        expect(sink.processBulk(postInput) == work::Status::OK);
        expect(eq(sink.size(), 2UZ));
        const std::vector<Tag> postTags = sink.tags();
        expect(fatal(eq(postTags.size(), 1UZ)));
        expect(eq(postTags[0].index, 1UZ));
    };
};

int main() { /* not needed for UT */ }
