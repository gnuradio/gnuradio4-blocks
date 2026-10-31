#include <boost/ut.hpp>

#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <initializer_list>
#include <memory_resource>
#include <span>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/basic/VectorSink.hpp>
#include <gnuradio-4.0/basic/VectorSource.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::property_map;
using gr::blocks::basic::VectorSink;
using gr::blocks::basic::VectorSource;

/// Returns 1, 2, ..., `n`, with a negative imaginary part for complex types.
template<typename T>
std::vector<T> makeRamp(std::size_t n) {
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

gr::pmt::Value listOf(std::initializer_list<property_map> maps) {
    std::pmr::vector<gr::pmt::Value> values;
    for (const property_map& map : maps) {
        values.emplace_back(map);
    }
    return gr::pmt::Value(gr::Tensor<gr::pmt::Value>(std::move(values)));
}

/// A VectorSource connected to a VectorSink, in a graph that a scheduler holds.
template<typename T>
struct Chain {
    gr::Graph               graph;
    VectorSource<T>&        source;
    VectorSink<T>&          sink;
    gr::scheduler::Simple<> scheduler;

    Chain(const property_map& sourceSettings, const property_map& sinkSettings) : source(graph.emplaceBlock<VectorSource<T>>(sourceSettings)), sink(graph.emplaceBlock<VectorSink<T>>(sinkSettings)) {
        boost::ut::expect(boost::ut::fatal(graph.connect<"out", "in">(source, sink).has_value()));
        boost::ut::expect(boost::ut::fatal(scheduler.exchange(std::move(graph)).has_value()));
    }

    void run() { boost::ut::expect(scheduler.runAndWait().has_value()); }

    /// Reads the sink's settings through the block model the scheduler holds, as a host without the C++ type does.
    property_map sinkSettings() {
        for (const auto& block : scheduler.graph().blocks()) {
            if (block->uniqueName() == sink.unique_name) {
                return block->settings().get();
            }
        }
        return {};
    }
};

} // namespace

const boost::ut::suite<"VectorSink"> vectorSinkTests = [] {
    using namespace boost::ut;

    "samples and tags round trip from VectorSource into VectorSink"_test = []<typename T>(const T&) {
        const std::vector<T> values = makeRamp<T>(37UZ);
        const property_map   tags{{"0", property_map{{"first", true}}}, {"17", listOf({property_map{{"mark", "middle"}}, property_map{{"n", 17}}})}, {std::pmr::string(std::to_string(values.size())), property_map{{"last", true}}}};

        Chain<T> first({{"values", values}, {"tags", tags}}, {});
        first.run();
        expect(first.sink.data.value == values);
        expect(first.sink.tags.value == tags) << "the record holds each tag at its index, a list and the end-of-stream tag included";
        expect(eq(first.sink.received.value, std::uint64_t{values.size()}));

        Chain<T> second({{"values", first.sink.data.value}, {"tags", first.sink.tags.value}}, {});
        second.run();
        expect(second.sink.data.value == values) << "a recorded run fed to a source comes out unchanged";
        expect(second.sink.tags.value == tags);
    } | std::tuple<std::uint8_t, std::int16_t, std::uint16_t, std::int32_t, std::uint32_t, float, double, std::complex<float>, std::complex<double>>();

    "the sample limit ends the run with exactly the limit recorded"_test = [] {
        const std::vector<float> pattern{10.f, 20.f, 30.f};
        constexpr std::uint64_t  kRepeat = 100000U;
        constexpr std::size_t    kLimit  = 11UZ;
        const property_map       kept{{"k", "kept"}};
        const property_map       sourceTags{{"4", kept}, {"20", property_map{{"k", "dropped"}}}};

        Chain<float> chain({{"values", pattern}, {"repeat", kRepeat}, {"tags", sourceTags}}, {{"n_samples_max", static_cast<gr::Size_t>(kLimit)}});
        chain.run();
        std::vector<float> expected(kLimit);
        for (std::size_t i = 0UZ; i < kLimit; ++i) {
            expected[i] = pattern[i % pattern.size()];
        }
        expect(chain.sink.data.value == expected);
        expect(chain.sink.tags.value == property_map{{"4", kept}}) << "a tag past the limit is not recorded";
        expect(eq(chain.sink.received.value, std::uint64_t{kLimit}));
        expect(lt(chain.source._emitted, pattern.size() * kRepeat)) << "the run ends before the source runs out";
    };

    "a call that reaches the limit consumes exactly the samples it records"_test = [] {
        VectorSink<float> sink;
        sink.n_samples_max = 11U;
        sink.start();

        const std::vector<float>                    chunk = makeRamp<float>(6UZ);
        gr::blocks::testing::span::InputSpan<float> first{std::span<const float>(chunk)};
        expect(sink.processBulk(first) == gr::work::Status::OK);
        expect(!first.consumeRequested) << "below the limit the whole span is consumed";

        const std::vector<gr::Tag>                  incoming{{7UZ, property_map{{"k", "kept"}}}, {11UZ, property_map{{"k", "dropped"}}}};
        gr::blocks::testing::span::InputSpan<float> second{std::span<const float>(chunk), 6UZ, std::span<const gr::Tag>(incoming)};
        expect(sink.processBulk(second) == gr::work::Status::DONE);
        expect(second.consumeRequested);
        expect(eq(second.consumed, 11UZ - chunk.size()));
        sink.stop();
        expect(fatal(eq(sink.data->size(), 11UZ)));
        expect(eq(sink.data->back(), chunk[4]));
        expect(sink.tags.value == property_map{{"7", property_map{{"k", "kept"}}}});
    };

    "several tags at one index record as a list in arrival order"_test = [] {
        VectorSink<float> sink;
        sink.start();
        const property_map                          firstTag{{"a", 1}, {"b", 1}};
        const property_map                          secondTag{{"b", 2}};
        const std::vector<float>                    chunk{1.f, 2.f, 3.f};
        const std::vector<gr::Tag>                  incoming{{4UZ, property_map{{"old", true}}}, {6UZ, firstTag}, {6UZ, secondTag}, {7UZ, secondTag}};
        gr::blocks::testing::span::InputSpan<float> input{std::span<const float>(chunk), 5UZ, std::span<const gr::Tag>(incoming)};
        expect(sink.processBulk(input) == gr::work::Status::OK);
        sink.stop();
        expect(sink.tags.value == property_map{{"1", listOf({firstTag, secondTag})}, {"2", secondTag}}) << "a tag behind the span is not recorded";
    };

    "the record reaches the settings when the block stops"_test = [] {
        VectorSink<float> sink;
        sink.init(sink.progress);
        sink.start();
        const std::vector<float>                    chunk{1.f, 2.f, 3.f};
        const std::vector<gr::Tag>                  incoming{{0UZ, property_map{{"mark", true}}}};
        gr::blocks::testing::span::InputSpan<float> input{std::span<const float>(chunk), 0UZ, std::span<const gr::Tag>(incoming)};
        expect(sink.processBulk(input) == gr::work::Status::OK);
        const property_map during = sink.settings().get();
        expect(sink.data->empty() && sink.tags->empty()) << "the members hold the record of the last stop";
        expect(during.at("received") == gr::pmt::Value(std::uint64_t{0})) << "a settings read during the run returns the cleared record";
        sink.stop();
        const property_map after = sink.settings().get();
        expect(after.at("received") == gr::pmt::Value(std::uint64_t{chunk.size()}));
        expect(sink.data.value == chunk);
        expect(sink.tags.value == property_map{{"0", property_map{{"mark", true}}}});
    };

    "a restart clears the record"_test = [] {
        const std::vector<float> values{1.f, 2.f, 3.f, 4.f, 5.f};
        const property_map       tags{{"2", property_map{{"mark", true}}}};
        Chain<float>             chain({{"values", values}, {"tags", tags}}, {});
        chain.run();
        chain.run();
        expect(chain.sink.data.value == values) << "the second run's samples alone";
        expect(chain.sink.tags.value == tags);
        expect(eq(chain.sink.received.value, std::uint64_t{values.size()}));
    };

    "a settings read through the block model returns the record"_test = []<typename T>(const T&) {
        const std::vector<T> values = makeRamp<T>(23UZ);
        const property_map   tags{{"3", property_map{{"mark", "a"}}}, {"9", property_map{{"mark", "b"}}}};
        for (const gr::Size_t limit : {gr::Size_t{0U}, gr::Size_t{7U}}) {
            Chain<T> chain({{"values", values}, {"tags", tags}}, {{"n_samples_max", limit}});
            chain.run();
            const property_map settings = chain.sinkSettings();
            const auto*        data     = settings.at("data").template get_if<gr::Tensor<T>>();
            expect(fatal(data != nullptr)) << "data reads as a tensor of samples";
            expect(std::ranges::equal(std::span(data->data(), data->size()), chain.sink.data.value)) << std::format("limit {}", limit);
            expect(eq(chain.sink.data->size(), limit == 0U ? values.size() : static_cast<std::size_t>(limit)));
            const auto* recorded = settings.at("tags").template get_if<property_map>();
            expect(fatal(recorded != nullptr));
            expect(*recorded == chain.sink.tags.value) << std::format("limit {}", limit);
            expect(settings.at("received") == gr::pmt::Value(chain.sink.received.value));
        }
    } | std::tuple<float, std::complex<double>>();
};

int main() { /* not needed for UT */ }
