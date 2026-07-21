#include <boost/ut.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>

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
};

int main() { /* not needed for UT */ }
