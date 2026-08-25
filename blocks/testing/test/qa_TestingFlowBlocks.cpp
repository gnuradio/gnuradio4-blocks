#include <boost/ut.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

const boost::ut::suite<"testing flow blocks"> testingFlowBlocks = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::testing;
    constexpr auto kTestTypes = std::tuple<uint8_t, int16_t, int32_t, float>();

    "ConstantSource->Copy->CountingSink"_test = []<typename T>(const T&) {
        constexpr gr::Size_t                          N     = 10;
        constexpr typename ConstantSource<T>::value_t value = typename ConstantSource<T>::value_t(7);

        Graph g;
        auto& src  = g.emplaceBlock<ConstantSource<T>>(property_map{{"default_value", value}, {"n_samples_max", N}});
        auto& copy = g.emplaceBlock<Copy<T>>();
        auto& sink = g.emplaceBlock<CountingSink<T>>(property_map{{"n_samples_max", N}});

        expect(g.connect<"out", "in">(src, copy).has_value());
        expect(g.connect<"out", "in">(copy, sink).has_value());

        gr::scheduler::Simple sch;
        if (auto ret = sch.exchange(std::move(g)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sch.runAndWait().has_value());
        expect(eq(sink.count, N));
    } | kTestTypes;

    "CountingSource->HeadBlock->CountingSink"_test = []<typename T>(const T&) {
        constexpr gr::Size_t N_total     = 10;
        constexpr gr::Size_t N_head      = 4;
        constexpr T          start_value = T(5);

        Graph g;
        auto& src  = g.emplaceBlock<CountingSource<T>>(property_map{{"default_value", start_value}, {"n_samples_max", N_total}});
        auto& head = g.emplaceBlock<HeadBlock<T>>(property_map{{"n_samples_max", N_head}});
        auto& sink = g.emplaceBlock<CountingSink<T>>(property_map{{"n_samples_max", N_head}});

        expect(g.connect<"out", "in">(src, head).has_value());
        expect(g.connect<"out", "in">(head, sink).has_value());

        gr::scheduler::Simple sch;
        if (auto ret = sch.exchange(std::move(g)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sch.runAndWait().has_value());
        expect(eq(sink.count, N_head));
    } | kTestTypes;

    "TagSource mark_tag publishes only 0 and 1"_test = []<typename T>(const T&) {
        constexpr gr::Size_t N = 16;

        Graph g;
        auto& src  = g.emplaceBlock<TagSource<T, ProcessFunction::USE_PROCESS_BULK>>(property_map{{"n_samples_max", N}, {"mark_tag", true}});
        auto& sink = g.emplaceBlock<TagSink<T, ProcessFunction::USE_PROCESS_ONE>>(property_map{{"n_samples_expected", N}});
        src._tags  = {{3UZ, {{"key", "a"}}}, {11UZ, {{"key", "b"}}}};

        expect(g.connect<"out", "in">(src, sink).has_value());

        gr::scheduler::Simple sch;
        if (auto ret = sch.exchange(std::move(g)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sch.runAndWait().has_value());

        expect(eq(sink._samples.size(), static_cast<std::size_t>(N)));
        std::vector<T> expected(N, T(0));
        expected[3]  = T(1);
        expected[11] = T(1);
        expect(std::ranges::equal(sink._samples, expected)) << std::format("mark_tag must publish 1 at tagged offsets and 0 everywhere else, got {}", sink._samples);
    } | kTestTypes;

    "TagMonitor reports the stream without rewriting it"_test = [] {
        using Monitor                   = TagMonitor<float, ProcessFunction::USE_PROCESS_ONE>;
        constexpr gr::Size_t N          = 16;
        constexpr float      streamRate = 4242.f;

        // the monitor owns none of the reserved keys, so it neither substitutes its own value into a forwarded tag
        // nor tracks a passing one
        const auto& writable = gr::CtxSettings<Monitor>::allWritableMembers();
        expect(!writable.contains(std::string(gr::tag::SAMPLE_RATE.shortKey()))) << "a monitor must not own the stream keys it observes";
        expect(!writable.contains(std::string(gr::tag::SIGNAL_NAME.shortKey()))) << "a monitor must not own the stream keys it observes";

        Graph g;
        auto& src     = g.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>(property_map{{"n_samples_max", N}, {gr::tag::SAMPLE_RATE.shortKey(), 1000.f}});
        auto& monitor = g.emplaceBlock<Monitor>(property_map{{"n_samples_expected", N}});
        auto& sink    = g.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>(property_map{{"n_samples_expected", N}});
        src._tags     = {{6UZ, {{gr::tag::SAMPLE_RATE.shortKey(), streamRate}}}};

        expect(g.connect<"out", "in">(src, monitor).has_value());
        expect(g.connect<"out", "in">(monitor, sink).has_value());

        gr::scheduler::Simple sch;
        if (auto ret = sch.exchange(std::move(g)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sch.runAndWait().has_value());

        std::size_t nSeen = 0UZ;
        for (const auto& tag : sink._tags) {
            const auto it = tag.map.find(gr::tag::SAMPLE_RATE.shortKey());
            if (it == tag.map.end() || it->second.value_or(0.f) == 1000.f) {
                continue; // the settings-forward tag carries the source's own rate
            }
            ++nSeen;
            expect(eq(it->second.value_or(0.f), streamRate)) << "the monitor rewrote the sample rate it forwarded";
        }
        expect(eq(nSeen, 1UZ)) << std::format("the sample_rate tag did not cross the monitor, sink saw {} tags", sink._tags.size());
    };
};

int main() { /* not needed for UT */ }
