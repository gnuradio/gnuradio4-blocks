#include <boost/ut.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>

const boost::ut::suite<"SimCompute"> simComputeTests = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::testing;
    constexpr auto kTestTypes = std::tuple<uint8_t, int16_t, int32_t, float>();

    "zero complexity"_test = []<typename T>(const T&) {
        constexpr gr::Size_t N = 8;
        Graph                g;
        auto&                src  = g.emplaceBlock<ConstantSource<T>>(property_map{{"default_value", typename ConstantSource<T>::value_t(5)}, {"n_samples_max", N}});
        auto&                sim  = g.emplaceBlock<SimCompute<T>>(property_map{{"complexity_order", 0.0f}, {"busy_wait", true}});
        auto&                sink = g.emplaceBlock<CountingSink<T>>(property_map{{"n_samples_max", N}});
        expect(g.connect<"out", "in">(src, sim).has_value());
        expect(g.connect<"out", "in">(sim, sink).has_value());
        gr::scheduler::Simple sch;
        if (auto ret = sch.exchange(std::move(g)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sch.runAndWait().has_value());
        expect(eq(sink.count, N));
    } | kTestTypes;

    "linear complexity"_test = []<typename T>(const T&) {
        constexpr gr::Size_t N = 8;
        Graph                g;
        auto&                src  = g.emplaceBlock<ConstantSource<T>>(property_map{{"default_value", typename ConstantSource<T>::value_t(5)}, {"n_samples_max", N}});
        auto&                sim  = g.emplaceBlock<SimCompute<T>>(property_map{{"complexity_order", 1.0f}, {"busy_wait", true}});
        auto&                sink = g.emplaceBlock<CountingSink<T>>(property_map{{"n_samples_max", N}});
        expect(g.connect<"out", "in">(src, sim).has_value());
        expect(g.connect<"out", "in">(sim, sink).has_value());
        gr::scheduler::Simple sch;
        if (auto ret = sch.exchange(std::move(g)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sch.runAndWait().has_value());
        expect(eq(sink.count, N));
    } | kTestTypes;

    "quadratic complexity"_test = []<typename T>(const T&) {
        constexpr gr::Size_t N = 6;
        Graph                g;
        auto&                src  = g.emplaceBlock<CountingSource<T>>(property_map{{"default_value", T(0)}, {"n_samples_max", N}});
        auto&                sim  = g.emplaceBlock<SimCompute<T>>(property_map{{"complexity_order", 2.0f}, {"busy_wait", false}});
        auto&                sink = g.emplaceBlock<CountingSink<T>>(property_map{{"n_samples_max", N}});
        expect(g.connect<"out", "in">(src, sim).has_value());
        expect(g.connect<"out", "in">(sim, sink).has_value());
        gr::scheduler::Simple sch;
        if (auto ret = sch.exchange(std::move(g)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sch.runAndWait().has_value());
        expect(eq(sink.count, N));
    } | kTestTypes;

    "compute_delay_seconds"_test = [] {
        SimCompute<float> sim;
        sim.target_throughput   = 1e6f;
        sim.reference_work_size = 1000U;
        sim.complexity_order    = 2.0f;

        constexpr std::size_t N            = 2000;
        const auto            delay        = sim.compute_delay_seconds(N);
        const double          expected_sec = std::pow(double(N) / 1000.0, 2.0) * (1000.0 / 1e6);
        expect(approx(delay.count(), expected_sec, 1e-3));
    };
};

int main() { /* not needed for UT */ }
