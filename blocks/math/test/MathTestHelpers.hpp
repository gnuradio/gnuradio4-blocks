#ifndef GR4_BLOCKS_MATH_TEST_MATH_TEST_HELPERS_HPP
#define GR4_BLOCKS_MATH_TEST_MATH_TEST_HELPERS_HPP

#include <boost/ut.hpp>

#include <array>
#include <bit>
#include <cmath>
#include <complex>
#include <cstdint>
#include <functional>
#include <limits>
#include <random>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/math/Math.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

/// @brief Finite complex values: every pair of parts from the signed zeros, one and the smallest and largest normals
/// of either sign, then random finite bit patterns and Gaussian noise.
[[nodiscard]] inline std::vector<std::complex<float>> finiteComplexInput() {
    using L                                = std::numeric_limits<float>;
    const std::array<float, 8UZ>     parts = {0.f, -0.f, 1.f, -1.f, L::min(), -L::min(), L::max(), -L::max()};
    std::vector<std::complex<float>> input;
    for (const float re : parts) {
        for (const float im : parts) {
            input.emplace_back(re, im);
        }
    }
    std::mt19937                                 rng(20261103U);
    std::uniform_int_distribution<std::uint32_t> bits;
    std::normal_distribution<float>              noise(0.f, 0.3f);
    while (input.size() < 4096UZ) {
        const std::complex<float> pattern(std::bit_cast<float>(bits(rng)), std::bit_cast<float>(bits(rng)));
        if (std::isfinite(pattern.real()) && std::isfinite(pattern.imag())) {
            input.push_back(pattern);
        }
        input.emplace_back(noise(rng), noise(rng));
    }
    return input;
}

/// @brief True for two complex values whose parts are the same values, a NaN part matching a NaN part.
[[nodiscard]] inline bool sameParts(std::complex<float> a, std::complex<float> b) {
    const auto same = [](float x, float y) { return (std::isnan(x) && std::isnan(y)) || (x == y && std::signbit(x) == std::signbit(y)); };
    return same(a.real(), b.real()) && same(a.imag(), b.imag());
}

template<typename T>
struct TestParameters {
    std::vector<gr::Tensor<T>> inputs;
    gr::Tensor<T>              output;
};

template<typename T, typename BlockUnderTest>
void test_block(const TestParameters<T> p) {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::testing;
    const Size_t n_inputs = static_cast<Size_t>(p.inputs.size());

    Graph graph;
    auto& block = graph.emplaceBlock<BlockUnderTest>({{"n_inputs", n_inputs}});
    for (Size_t i = 0; i < n_inputs; ++i) {
        auto& src = graph.emplaceBlock<TagSource<T>>({{"values", p.inputs[i]}, {"n_samples_max", static_cast<Size_t>(p.inputs[i].size())}});
        expect(graph.connect(src, "out"s, block, "in#"s + std::to_string(i)).has_value()) << std::format("Failed to connect output port of src {} to input port 'in#{}' of block", i, i);
    }
    auto& sink = graph.emplaceBlock<TagSink<T, ProcessFunction::USE_PROCESS_ONE>>();
    expect(graph.connect(block, "out"s, sink, "in"s).has_value()) << "Failed to connect output port 'out' of block to input port of sink";

    gr::scheduler::Simple sched;
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
    }
    expect(sched.runAndWait().has_value()) << "Failed to run graph: No value";
    expect(std::ranges::equal(sink._samples, p.output)) << std::format("Failed to validate block output: Expected {} but got {} for input {}", p.output, sink._samples, p.inputs);
}

/// @brief The keys among @p keys that reach a sink behind @p BlockUnderTest, its input port named @p inputPort.
template<typename T, typename BlockUnderTest>
[[nodiscard]] std::set<std::string, std::less<>> keysCrossing(const std::vector<std::string_view>& keys, gr::property_map settings, const std::string& inputPort) {
    using namespace gr::blocks::testing;

    gr::property_map tagMap;
    for (std::string_view key : keys) {
        tagMap.insert_or_assign(gr::property_map::key_type{key}, 0.5f);
    }

    gr::Graph graph;
    auto&     src = graph.emplaceBlock<TagSource<T>>({{"n_samples_max", gr::Size_t{64U}}, {"mark_tag", false}});
    src._tags     = {gr::Tag{0UZ, tagMap}, gr::Tag{32UZ, tagMap}};
    auto& block   = graph.emplaceBlock<BlockUnderTest>(std::move(settings));
    auto& sink    = graph.emplaceBlock<TagSink<T, ProcessFunction::USE_PROCESS_BULK>>();
    boost::ut::expect(graph.connect(src, std::string("out"), block, inputPort).has_value());
    boost::ut::expect(graph.connect(block, std::string("out"), sink, std::string("in")).has_value());

    gr::scheduler::Simple sched;
    boost::ut::expect(sched.exchange(std::move(graph)).has_value());
    boost::ut::expect(sched.runAndWait().has_value());

    std::set<std::string, std::less<>> crossed;
    for (const gr::Tag& tag : sink._tags) {
        for (std::string_view key : keys) {
            if (tag.map.contains(key)) {
                crossed.emplace(key);
            }
        }
    }
    return crossed;
}

template<typename T>
constexpr T val(double x) {
    if constexpr (gr::meta::complex_like<T>) {
        using V = typename T::value_type;
        return T{static_cast<V>(x), static_cast<V>(0)};
    } else {
        return static_cast<T>(x);
    }
}

#endif
