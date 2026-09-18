#include <boost/ut.hpp>

#include <atomic>
#include <chrono>
#include <print>
#include <string_view>
#include <thread>

#include <gnuradio-4.0/Graph.hpp>

#include <gnuradio-4.0/basic/ClockSource.hpp>
#include <gnuradio-4.0/basic/FunctionGenerator.hpp>
#include <gnuradio-4.0/basic/SignalGenerator.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>

namespace {

// A teardown parked on a flag that nothing will set cannot be released from outside, by the graph's destructor
// or by the harness, so the case runs on a thread of its own: one that outlives the bound is already wedged, and
// ending the process with the reason makes that a failure in seconds rather than the harness's timeout.
void withinBound(std::chrono::milliseconds bound, std::string_view what, auto&& body) {
    std::atomic<bool> finished{false};

    auto runner = std::thread([&finished, &body] {
        body();
        finished.store(true, std::memory_order_release);
    });

    const auto deadline = std::chrono::steady_clock::now() + bound;
    while (!finished.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!finished.load(std::memory_order_acquire)) {
        std::println(stderr, "[qa_ClockSourceStop] {}: still running {} ms after it should have finished", what, bound.count());
        std::_Exit(1);
    }

    runner.join();
}

template<typename TSource>
void destroyGraphWhileSourceRuns() {
    using namespace boost::ut;

    for (std::size_t cycle = 0UZ; cycle < 10UZ; ++cycle) {
        gr::Graph flow;
        auto&     source   = flow.emplaceBlock<TSource>();
        source.sample_rate = 100000.f;
        source.chunk_size  = gr::Size_t{16};
        auto& sink         = flow.emplaceBlock<gr::blocks::testing::CountingSink<float>>({{"n_samples_max", gr::Size_t{500}}});
        expect(flow.connect<"out", "in">(source, sink).has_value());
        expect(source.changeStateTo(gr::lifecycle::State::RUNNING).has_value());

        std::this_thread::sleep_for(std::chrono::microseconds(200));
        // the graph, and with it the source, goes out of scope in RUNNING
    }
}

} // namespace

const boost::ut::suite<"wall-clock source teardown"> teardownTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::basic;

    "a running ClockSource is destructible"_test = [] { withinBound(std::chrono::seconds{5}, "a running ClockSource", [] { destroyGraphWhileSourceRuns<ClockSource<float>>(); }); };

    "a running SignalGenerator is destructible"_test = [] { withinBound(std::chrono::seconds{5}, "a running SignalGenerator", [] { destroyGraphWhileSourceRuns<SignalGenerator<float>>(); }); };

    "a running FunctionGenerator is destructible"_test = [] { withinBound(std::chrono::seconds{5}, "a running FunctionGenerator", [] { destroyGraphWhileSourceRuns<FunctionGenerator<float>>(); }); };
};

int main() { /* not needed for UT */ }
