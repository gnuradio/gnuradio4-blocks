#include <boost/ut.hpp>

#include <gnuradio-4.0/math/ExpressionBlocks.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/meta/formatter.hpp>

namespace gr::blocks::math {
static_assert(std::is_constructible_v<ExpressionBulk<float>, property_map>, "Block type ExpressionBulk must be constructible from property_map");
} // namespace gr::blocks::math

const boost::ut::suite<"bulk expression block tests"> bulkExpression = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::math;
    using testing::ProcessFunction::USE_PROCESS_ONE;

    "ExpressionBulk"_test = []<typename T>(const T&) {
        Graph graph;

        auto& source    = graph.emplaceBlock<testing::CountingSource<T>>({{"n_samples_max", 10U}});
        auto& exprBlock = graph.emplaceBlock<ExpressionBulk<T>>({{"expr_string", "vecOut := a * vecIn"}, {"param_a", T(2)}});
        auto& tagSink   = graph.emplaceBlock<testing::TagSink<T, USE_PROCESS_ONE>>({{"log_samples", true}});

        expect(graph.connect<"out", "in">(source, exprBlock).has_value());
        expect(graph.connect<"out", "in">(exprBlock, tagSink).has_value());

        gr::scheduler::Simple<> sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sched.runAndWait().has_value());

        expect(eq(tagSink._samples.size(), source.n_samples_max));
        expect(approx(exprBlock.param_a.value, T(2), T(1e-3f)));
        for (std::size_t i = 0; i < tagSink._samples.size(); ++i) {
            T expected = T(1 + i) * T(2);
            expect(approx(tagSink._samples[i], expected, T(1e-6))) << std::format("should be: output[{}] ({}) == {} * input[{}] ({}) ", i, tagSink._samples[i], exprBlock.param_a, i, expected);
        }
    } | std::tuple<float, double>{};

    "ExpressionBulk - exceptions"_test = [](const bool enableERuntimeChecks) {
        Graph graph;

        auto&             source    = graph.emplaceBlock<testing::CountingSource<float>>({{"n_samples_max", 10U}});
        const std::string exprStr   = R""(for (var i := 0; i < 100; i += 1) {
   vecOut[i] := vecIn[i+1];
}
)"";
        auto&             exprBlock = graph.emplaceBlock<ExpressionBulk<float>>({{"expr_string", exprStr}, {"runtime_checks", enableERuntimeChecks}});
        auto&             tagSink   = graph.emplaceBlock<testing::TagSink<float, USE_PROCESS_ONE>>({{"log_samples", true}});

        expect(graph.connect<"out", "in">(source, exprBlock).has_value());
        expect(graph.connect<"out", "in">(exprBlock, tagSink).has_value());

        gr::scheduler::Simple<> sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }

        try {
            expect(sched.runAndWait().has_value());
            expect(false) << std::format("should have failed");
        } catch (const gr::exception& ex) {
            expect(true);
            std::println("failed correctly with:\n{}\n", ex);
        } catch (...) {
            expect(false) << std::format("caught unknown/unexpected exception");
        }
    } | std::vector{true /*, false -- disabled on purpose as this would correctly trigger the ASAN checks*/};
};

int main() { /* not needed for UT */ }
