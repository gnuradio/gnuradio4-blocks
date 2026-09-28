#include <boost/ut.hpp>

#include <cstdint>
#include <limits>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/testing/NullSources.hpp>

const boost::ut::suite<"Null[..] and Testing Blocks"> nullSourcesTests = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::testing;

    constexpr auto kTestTypes = std::tuple<uint8_t, int16_t, int32_t, float>();

    "NullSource->CountingSink"_test = []<typename T>(const T&) {
        constexpr std::uint64_t N = 12;

        Graph g;
        auto& src  = g.emplaceBlock<NullSource<T>>();
        auto& sink = g.emplaceBlock<CountingSink<T>>(property_map{{"n_samples_max", N}});

        expect(g.connect<"out", "in">(src, sink).has_value());

        gr::scheduler::Simple sch;
        if (auto ret = sch.exchange(std::move(g)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sch.runAndWait().has_value());
        expect(eq(sink.count, N));
    } | kTestTypes;

    "CountingSource->NullSink"_test = []<typename T>(const T&) {
        constexpr std::uint64_t N_total     = 7;
        constexpr T             start_value = T(3);

        Graph g;
        auto& src  = g.emplaceBlock<CountingSource<T>>(property_map{{"default_value", start_value}, {"n_samples_max", N_total}});
        auto& sink = g.emplaceBlock<NullSink<T>>();

        expect(g.connect<"out", "in">(src, sink).has_value());

        gr::scheduler::Simple sch;
        if (auto ret = sch.exchange(std::move(g)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sch.runAndWait().has_value());
    } | kTestTypes;

    "ConstantSource->NullSink"_test = []<typename T>(const T&) {
        constexpr std::uint64_t N = 5;

        Graph g;
        auto& src  = g.emplaceBlock<ConstantSource<T>>(property_map{{"default_value", typename ConstantSource<T>::value_t(99)}, {"n_samples_max", N}});
        auto& sink = g.emplaceBlock<NullSink<T>>();

        expect(g.connect<"out", "in">(src, sink).has_value());

        gr::scheduler::Simple sch;
        if (auto ret = sch.exchange(std::move(g)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sch.runAndWait().has_value());
    } | kTestTypes;

    "SlowSource->CountingSink"_test = []<typename T>(const T&) {
        constexpr std::uint64_t N = 3;

        Graph g;
        auto& src  = g.emplaceBlock<SlowSource<T>>(property_map{{"default_value", typename SlowSource<T>::value_t(77)}, {"delay", 10U}});
        auto& sink = g.emplaceBlock<CountingSink<T>>(property_map{{"n_samples_max", N}});

        expect(g.connect<"out", "in">(src, sink).has_value());

        gr::scheduler::Simple sch;
        if (auto ret = sch.exchange(std::move(g)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sch.runAndWait().has_value());
        expect(eq(sink.count, N));
    } | kTestTypes;

    // the count starts six below 2^32 and ten samples arrive; a 32-bit count reads 4
    static constexpr std::uint64_t kStart = 4'294'967'290U;
    static constexpr std::uint64_t kFed   = 10U;
    static_assert(kStart + kFed > std::numeric_limits<std::uint32_t>::max());

    "CountingSink count continues past 2^32"_test = [] {
        Graph g;
        auto& src  = g.emplaceBlock<ConstantSource<float>>(property_map{{"n_samples_max", kFed}});
        auto& sink = g.emplaceBlock<CountingSink<float>>(property_map{{"count", kStart}});

        expect(g.connect<"out", "in">(src, sink).has_value());

        gr::scheduler::Simple sch;
        if (auto ret = sch.exchange(std::move(g)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sch.runAndWait().has_value());
        expect(eq(sink.count.value, kStart + kFed));
    };

    "HeadBlock count continues past 2^32"_test = [] {
        Graph g;
        auto& src  = g.emplaceBlock<ConstantSource<float>>(property_map{{"n_samples_max", kFed}});
        auto& head = g.emplaceBlock<HeadBlock<float>>(property_map{{"count", kStart}});
        auto& sink = g.emplaceBlock<CountingSink<float>>();

        expect(g.connect<"out", "in">(src, head).has_value());
        expect(g.connect<"out", "in">(head, sink).has_value());

        gr::scheduler::Simple sch;
        if (auto ret = sch.exchange(std::move(g)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sch.runAndWait().has_value());
        expect(eq(head.count.value, kStart + kFed));
        expect(eq(sink.count.value, kFed));
    };

    "HeadBlock honors an n_samples_max above 2^32"_test = [] {
        // a 32-bit limit truncates 2^32 + 4 to 4, below the preset count
        constexpr std::uint64_t kLimit = kStart + kFed;
        constexpr std::uint64_t kOffer = 2U * kFed;

        Graph g;
        auto& src  = g.emplaceBlock<ConstantSource<float>>(property_map{{"n_samples_max", kOffer}});
        auto& head = g.emplaceBlock<HeadBlock<float>>(property_map{{"n_samples_max", kLimit}, {"count", kStart}});
        auto& sink = g.emplaceBlock<CountingSink<float>>();

        expect(eq(head.n_samples_max.value, kLimit));

        expect(g.connect<"out", "in">(src, head).has_value());
        expect(g.connect<"out", "in">(head, sink).has_value());

        gr::scheduler::Simple sch;
        if (auto ret = sch.exchange(std::move(g)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sch.runAndWait().has_value());
        expect(eq(head.count.value, kLimit));
        expect(eq(sink.count.value, kLimit - kStart)) << "the head must pass exactly the samples left below its limit";
    };

    "HeadBlock takes n_samples_max from a plain integer"_test = []<typename TInt>(const TInt&) {
        constexpr TInt          kHead  = 4;
        constexpr std::uint64_t kOffer = 10U;

        Graph g;
        auto& src  = g.emplaceBlock<ConstantSource<float>>(property_map{{"n_samples_max", kOffer}});
        auto& head = g.emplaceBlock<HeadBlock<float>>(property_map{{"n_samples_max", kHead}});
        auto& sink = g.emplaceBlock<CountingSink<float>>();

        expect(eq(head.n_samples_max.value, static_cast<std::uint64_t>(kHead)));

        expect(g.connect<"out", "in">(src, head).has_value());
        expect(g.connect<"out", "in">(head, sink).has_value());

        gr::scheduler::Simple sch;
        if (auto ret = sch.exchange(std::move(g)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sch.runAndWait().has_value());
        expect(eq(sink.count.value, static_cast<std::uint64_t>(kHead)));
    } | std::tuple<int, std::int64_t, gr::Size_t>();

    "double header type smoke test"_test = [] {
        static_assert(BlockLike<ConstantSource<double>>);
        static_assert(BlockLike<NullSink<double>>);

        ConstantSource<double> source;
        NullSink<double>       sink;
        source.default_value = 1.5;
        expect(eq(source.processOne(), 1.5));
        sink.processOne(1.5);
    };

    "complex canonical type smoke test"_test = [] {
        using T = std::complex<float>;

        static_assert(BlockLike<ConstantSource<T>>);
        static_assert(BlockLike<NullSink<T>>);

        ConstantSource<T> source(property_map{{"default_value", 1.5F}});
        NullSink<T>       sink;
        source.init(source.progress);
        expect(eq(source.processOne(), T{1.5F, 0.F}));
        sink.processOne(T{1.5F, -2.F});
    };
};

int main() { /* not needed for UT */ }
