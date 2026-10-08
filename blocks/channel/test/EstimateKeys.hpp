#ifndef GNURADIO_CHANNEL_TEST_ESTIMATE_KEYS_HPP
#define GNURADIO_CHANNEL_TEST_ESTIMATE_KEYS_HPP

#include <algorithm>
#include <array>
#include <complex>
#include <functional>
#include <initializer_list>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace gr::blocks::channel::test {

/// the estimate keys a carrier or timing loop reads, and one key no block in the module names
inline constexpr std::array<std::string_view, 5UZ> kTagKeys{"freq_est", "phase_est", "time_est", "clock_est", "burst_id"};

/// @brief The test keys that reach a sink behind @p TBlock, from two tags that carry all of them.
template<typename TBlock>
[[nodiscard]] std::set<std::string, std::less<>> keysCrossing(gr::property_map settings = {}) {
    using C = std::complex<float>;
    using gr::testing::ProcessFunction;

    gr::Graph              graph;
    auto&                  source = graph.emplaceBlock<gr::testing::TagSource<C, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", gr::Size_t{128U}}, {"sample_rate", 1.f}, {"mark_tag", false}});
    const gr::property_map keys{{"freq_est", 0.01f}, {"phase_est", 0.5f}, {"time_est", 0.25f}, {"clock_est", std::vector<float>{0.25f, 4.f}}, {"burst_id", std::string("b7")}};
    source._tags = {gr::Tag{0UZ, keys}, gr::Tag{64UZ, keys}};

    auto& block = graph.emplaceBlock<TBlock>(std::move(settings));
    auto& sink  = graph.emplaceBlock<gr::testing::TagSink<C, ProcessFunction::USE_PROCESS_BULK>>({});
    boost::ut::expect(graph.connect<"out", "in">(source, block).has_value());
    boost::ut::expect(graph.connect<"out", "in">(block, sink).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value());

    std::set<std::string, std::less<>> crossed;
    for (const gr::Tag& tag : sink._tags) {
        for (std::string_view key : kTagKeys) {
            if (tag.map.contains(key)) {
                crossed.emplace(key);
            }
        }
    }
    return crossed;
}

/// @brief The test keys a block that drops @p dropped lets through.
[[nodiscard]] inline std::set<std::string, std::less<>> keysOtherThan(std::initializer_list<std::string_view> dropped) {
    std::set<std::string, std::less<>> kept;
    for (std::string_view key : kTagKeys) {
        if (std::ranges::find(dropped, key) == dropped.end()) {
            kept.emplace(key);
        }
    }
    return kept;
}

} // namespace gr::blocks::channel::test

#endif // GNURADIO_CHANNEL_TEST_ESTIMATE_KEYS_HPP
