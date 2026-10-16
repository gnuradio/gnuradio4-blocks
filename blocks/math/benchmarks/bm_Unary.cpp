#include <benchmark.hpp>

#include <complex>
#include <format>
#include <span>
#include <vector>

#include <gnuradio-4.0/math/Unary.hpp>

namespace {

constexpr std::size_t kSamples     = 65536UZ;
constexpr std::size_t kRepetitions = 200UZ;
constexpr std::size_t kWarmUp      = 50UZ;

template<typename T>
std::vector<T> generateInput() {
    std::vector<T> input(kSamples);
    for (std::size_t i = 0UZ; i < kSamples; ++i) {
        // positive values between 0.1 and 10; the logarithm of each is finite
        const float value = 0.1f + 9.9f * static_cast<float>((i * 7919UZ) % 1000UZ) / 1000.f;
        if constexpr (gr::meta::complex_like<T>) {
            input[i] = T(value, 1.f - value);
        } else {
            input[i] = static_cast<T>(value);
        }
    }
    return input;
}

// calls processOne over simd vectors where the block takes them, and the scalar overload for the tail
template<typename TBlock, typename T, typename R>
void processSpan(const TBlock& block, std::span<const T> input, std::span<R> output) {
    std::size_t i = 0UZ;
    if constexpr (TBlock::kSimdPath) {
        using V = vir::stdx::native_simd<T>;
        for (; i + V::size() <= input.size(); i += V::size()) {
            const V x(input.data() + i, vir::stdx::element_aligned);
            block.processOne(x).copy_to(output.data() + i, vir::stdx::element_aligned);
        }
    }
    for (; i < input.size(); ++i) {
        output[i] = block.processOne(input[i]);
    }
}

template<template<typename> typename TAlias, typename T>
void benchmarkBlock(std::string_view name, gr::property_map settings = {}) {
    using TBlock = TAlias<T>;
    using R      = typename TBlock::R;

    const std::vector<T> input = generateInput<T>();
    std::vector<R>       output(kSamples);

    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();

    auto pass = [&block, &input, &output] {
        processSpan(block, std::span<const T>(input), std::span<R>(output));
        ::benchmark::force_to_memory(output);
    };
    for (std::size_t i = 0UZ; i < kWarmUp; ++i) {
        pass();
    }
    ::benchmark::benchmark<kRepetitions>(std::format("{}<{}>", name, gr::meta::type_name<T>()), kSamples) = pass;
}

} // namespace

inline const boost::ut::suite _unary_bm_tests = [] {
    using namespace gr::blocks::math;
    benchmarkBlock<Negate, float>("Negate");
    benchmarkBlock<Negate, std::complex<float>>("Negate");
    benchmarkBlock<Log10, float>("Log10");
    benchmarkBlock<Log10, float>("Log10, scale 10", {{"scale", 10.f}});
    benchmarkBlock<MagnitudeSquared, std::complex<float>>("MagnitudeSquared");
    benchmarkBlock<Conjugate, std::complex<float>>("Conjugate");
    ::benchmark::results::add_separator();
};

int main() { /* not needed by the UT framework */ }
