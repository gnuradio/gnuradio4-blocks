#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <numbers>
#include <random>
#include <span>
#include <tuple>
#include <utility>
#include <vector>

#include <gnuradio-4.0/math/Unary.hpp>

namespace {

template<typename TBlock>
void applySettings(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
}

/// @brief `value` modulo 2^N in the integer type `T`.
template<typename T>
[[nodiscard]] T wrapped(std::int64_t value) {
    return static_cast<T>(value);
}

/// @brief The output of one scalar call per sample.
template<typename TBlock, typename T>
[[nodiscard]] std::vector<T> scalarPath(const TBlock& block, std::span<const T> input) {
    std::vector<T> output(input.size());
    for (std::size_t i = 0UZ; i < input.size(); ++i) {
        output[i] = block.processOne(input[i]);
    }
    return output;
}

/// @brief The output of one call per vector of type `V`, over the input padded with zeros to whole vectors.
template<typename V, typename TBlock, typename T>
[[nodiscard]] std::vector<T> simdPathOf(const TBlock& block, std::span<const T> input) {
    const std::size_t padded = (input.size() + V::size() - 1UZ) / V::size() * V::size();
    std::vector<T>    in(padded, T{0});
    std::ranges::copy(input, in.begin());
    std::vector<T> out(padded);
    for (std::size_t i = 0UZ; i < padded; i += V::size()) {
        block.processOne(V(in.data() + i, vir::stdx::element_aligned)).copy_to(out.data() + i, vir::stdx::element_aligned);
    }
    out.resize(input.size());
    return out;
}

/// @brief The output of one simd call per native vector, over the input padded with zeros to whole vectors.
template<typename TBlock, typename T>
[[nodiscard]] std::vector<T> simdPath(const TBlock& block, std::span<const T> input) {
    return simdPathOf<vir::stdx::native_simd<T>>(block, input);
}

/// @brief Two vectors and three samples of the values at the ends of `T`, the minimum first and the maximum last.
template<typename T>
[[nodiscard]] std::vector<T> boundaryInput() {
    using L                        = std::numeric_limits<T>;
    const std::array<T, 7UZ> cycle = {L::min(), static_cast<T>(L::min() + 1), T{-1}, T{0}, T{1}, static_cast<T>(L::max() - 1), L::max()};
    std::vector<T>           input(2UZ * vir::stdx::native_simd<T>::size() + 3UZ);
    for (std::size_t i = 0UZ; i < input.size(); ++i) {
        input[i] = cycle[i % cycle.size()];
    }
    input.front() = L::min();
    input.back()  = L::max();
    return input;
}

/// @brief The scale and offset pairs of the agreement case: the identity, an offset alone, and two that overflow.
template<typename T>
[[nodiscard]] std::array<std::pair<T, T>, 4UZ> affineSettings() {
    using L = std::numeric_limits<T>;
    return {{{T{1}, T{0}}, {T{1}, T{1}}, {T{-1}, L::min()}, {static_cast<T>(L::max() / 3), L::max()}}};
}

/// @brief Signed zeros, halves either side of an integer, the smallest magnitude from which every value is an integer,
/// the normal and subnormal minimums, the largest finite values, the infinities and a NaN: the inputs where a vector
/// rounding function or square root most often departs from the C library.
template<std::floating_point T>
[[nodiscard]] std::vector<T> exactInput() {
    using L                         = std::numeric_limits<T>;
    const T           integralLimit = T(1) / L::epsilon(); // 2^23 for float32, 2^52 for float64: every larger value is an integer
    std::vector<T>    input{T(0), T(0.25), T(0.5), std::nextafter(T(0.5), T(0)), T(1), T(1.5), T(2), T(2.5), T(3.5), T(4), T(1e6) + T(0.5), integralLimit - T(0.5), integralLimit, integralLimit + T(1), L::min(), L::denorm_min(), L::max(), L::infinity(), L::quiet_NaN()};
    const std::size_t positive = input.size();
    for (std::size_t i = 0UZ; i < positive; ++i) {
        input.push_back(-input[i]);
    }
    return input;
}

/// @brief True for two equal values of the same sign and for two NaNs.
template<std::floating_point T>
[[nodiscard]] bool sameValue(T a, T b) {
    return (std::isnan(a) && std::isnan(b)) || (a == b && std::signbit(a) == std::signbit(b));
}

/// @brief The number of samples where `output` is not the same value as `reference` applied to `input`.
template<typename T, typename F>
[[nodiscard]] std::size_t mismatchCount(std::span<const T> input, const std::vector<T>& output, F reference) {
    std::size_t mismatches = 0UZ;
    for (std::size_t i = 0UZ; i < input.size(); ++i) {
        mismatches += static_cast<std::size_t>(!sameValue(output[i], reference(input[i])));
    }
    return mismatches;
}

/// @brief The mismatches of the scalar call, the native simd call and the 32-wide simd call (the widest the scheduler
/// runs) of a block made from `TBlock` against `reference`, over `exactInput`.
template<typename TBlock, typename T, typename F>
[[nodiscard]] std::size_t exactMismatches(F reference) {
    static_assert(TBlock::kSimdPath);
    TBlock block;
    applySettings(block);
    const std::vector<T>     input = exactInput<T>();
    const std::span<const T> in(input);
    return mismatchCount(in, scalarPath(block, in), reference) + mismatchCount(in, simdPathOf<vir::stdx::native_simd<T>>(block, in), reference) + mismatchCount(in, simdPathOf<vir::stdx::fixed_size_simd<T, 32>>(block, in), reference);
}

} // namespace

const boost::ut::suite<"Unary exact simd"> unaryExactSimd = [] {
    using namespace boost::ut;
    using namespace gr::blocks::math;

    // IEEE 754 defines the square root as correctly rounded and the rounding functions as exact, so the error bound of
    // every call is zero: each output is the C library's value, with the sign of a zero and a NaN for a NaN.
    "Sqrt, Floor, Ceil, Round and Sign give the C library's value in every call"_test = []<typename T>(const T&) {
        expect(eq(exactMismatches<Sqrt<T>, T>([](T x) { return std::sqrt(x); }), 0UZ)) << "Sqrt";
        expect(eq(exactMismatches<Floor<T>, T>([](T x) { return std::floor(x); }), 0UZ)) << "Floor";
        expect(eq(exactMismatches<Ceil<T>, T>([](T x) { return std::ceil(x); }), 0UZ)) << "Ceil";
        expect(eq(exactMismatches<Round<T>, T>([](T x) { return std::round(x); }), 0UZ)) << "Round";
        expect(eq(exactMismatches<Sign<T>, T>([](T x) { return x > T(0) ? T(1) : (x < T(0) ? T(-1) : x); }), 0UZ)) << "Sign";
    } | std::tuple<float, double>{};
};

const boost::ut::suite<"Unary integer wrap"> unaryWrap = [] {
    using namespace boost::ut;
    using namespace gr::blocks::math;
    using L32 = std::numeric_limits<std::int32_t>;

    "Negate of the int32 minimum is the minimum"_test = [] {
        Negate<std::int32_t> block;
        applySettings(block);
        expect(eq(block.processOne(L32::min()), wrapped<std::int32_t>(-std::int64_t{L32::min()})));
        expect(eq(block.processOne(L32::min()), L32::min()));
    };

    "Abs of the int32 minimum is the minimum"_test = [] {
        Abs<std::int32_t> block;
        applySettings(block);
        expect(eq(block.processOne(L32::min()), L32::min()));
    };

    "an int32 scale wraps the product"_test = [] {
        Negate<std::int32_t> block({{"scale", std::int32_t{50000}}});
        applySettings(block);
        // -50000 * 50000 lies below the int32 minimum
        expect(eq(block.processOne(std::int32_t{50000}), wrapped<std::int32_t>(std::int64_t{-50000} * 50000)));
    };

    "an int32 offset wraps the sum"_test = [] {
        Negate<std::int32_t> block({{"offset", std::int32_t{1}}});
        applySettings(block);
        // -(-max) + 1 is max + 1
        expect(eq(block.processOne(-L32::max()), wrapped<std::int32_t>(std::int64_t{L32::max()} + 1)));
        expect(eq(block.processOne(-L32::max()), L32::min()));
    };

    "the scalar and the simd calls agree with the wrapped result at the ends of the type"_test = []<typename T>(const T&) {
        static_assert(Negate<T>::kSimdPath && Abs<T>::kSimdPath);
        const std::vector<T> input = boundaryInput<T>();

        std::size_t mismatches = 0UZ;
        for (const auto& [scale, offset] : affineSettings<T>()) {
            Negate<T> negateBlock({{"scale", scale}, {"offset", offset}});
            Abs<T>    absBlock({{"scale", scale}, {"offset", offset}});
            applySettings(negateBlock);
            applySettings(absBlock);

            std::vector<T> negated(input.size());
            std::vector<T> absolute(input.size());
            for (std::size_t i = 0UZ; i < input.size(); ++i) {
                const std::int64_t x = input[i];
                negated[i]           = wrapped<T>(-x * scale + offset);
                absolute[i]          = wrapped<T>((x < 0 ? -x : x) * scale + offset);
            }
            mismatches += static_cast<std::size_t>(scalarPath(negateBlock, std::span<const T>(input)) != negated);
            mismatches += static_cast<std::size_t>(simdPath(negateBlock, std::span<const T>(input)) != negated);
            mismatches += static_cast<std::size_t>(scalarPath(absBlock, std::span<const T>(input)) != absolute);
            mismatches += static_cast<std::size_t>(simdPath(absBlock, std::span<const T>(input)) != absolute);
        }
        expect(eq(mismatches, 0UZ)) << "calls whose output differs from the wrapped result";
    } | std::tuple<std::int16_t, std::int32_t>{};
};

namespace {

using CF = std::complex<float>;

/// @brief Every pair of parts from the signed zeros, the smallest subnormal, the smallest and largest normals, one,
/// the infinities and a NaN, then random bit patterns and Gaussian noise.
[[nodiscard]] std::vector<CF> complexInput() {
    using L                             = std::numeric_limits<float>;
    const std::array<float, 13UZ> parts = {0.f, -0.f, L::denorm_min(), -L::denorm_min(), L::min(), -L::min(), 1.f, -1.f, L::max(), -L::max(), L::infinity(), -L::infinity(), L::quiet_NaN()};
    std::vector<CF>               input;
    for (const float re : parts) {
        for (const float im : parts) {
            input.emplace_back(re, im);
        }
    }
    std::mt19937                                 rng(20261103U);
    std::uniform_int_distribution<std::uint32_t> bits;
    std::normal_distribution<float>              noise(0.f, 0.3f);
    for (std::size_t i = 0UZ; i < 4096UZ; ++i) {
        input.emplace_back(std::bit_cast<float>(bits(rng)), std::bit_cast<float>(bits(rng)));
        input.emplace_back(noise(rng), noise(rng));
    }
    return input;
}

/// @brief The output of `processBulk` over `input` in calls of `chunk` samples.
template<typename TBlock>
[[nodiscard]] std::vector<float> spanPath(const TBlock& block, std::span<const CF> input, std::size_t chunk) {
    std::vector<float> output(input.size());
    for (std::size_t base = 0UZ; base < input.size(); base += chunk) {
        const std::size_t count = std::min(chunk, input.size() - base);
        std::ignore             = block.processBulk(input.subspan(base, count), std::span<float>(output).subspan(base, count));
    }
    return output;
}

/// @brief The number of calls of `chunk` samples, for each chunk size below, whose output is not `whole` bit for bit.
template<typename TBlock>
[[nodiscard]] std::size_t chunkMismatches(const TBlock& block, std::span<const CF> input, const std::vector<float>& whole) {
    std::size_t mismatches = 0UZ;
    for (const std::size_t chunk : {1UZ, 3UZ, 255UZ, 257UZ}) {
        const std::vector<float> chunked = spanPath(block, input, chunk);
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            mismatches += static_cast<std::size_t>(!sameValue(chunked[i], whole[i]));
        }
    }
    return mismatches;
}

/// @brief The spacing of float values at the magnitude of `value`.
[[nodiscard]] double floatUlp(double value) {
    using L       = std::numeric_limits<float>;
    const int exp = std::max(std::ilogb(value), L::min_exponent - 1);
    return std::ldexp(1.0, exp - (L::digits - 1));
}

} // namespace

const boost::ut::suite<"Unary complex span loops"> unaryComplexSpans = [] {
    using namespace boost::ut;
    using namespace gr::blocks::math;

    static_assert(Magnitude<CF>::kSpanPath && gr::HasProcessBulkFunction<Magnitude<CF>> && !gr::HasProcessOneFunction<Magnitude<CF>>);
    static_assert(Phase<CF>::kSpanPath && gr::HasProcessBulkFunction<Phase<CF>> && !gr::HasProcessOneFunction<Phase<CF>>);

    // The double hypot of two float parts rounded to float is the magnitude computed in double and rounded once,
    // with C's special values: +inf for an infinite part beside a NaN, NaN for a NaN beside a finite part.
    "Magnitude gives the double hypot of the parts rounded to float, at every chunk size"_test = [] {
        Magnitude<CF> block;
        applySettings(block);
        const std::vector<CF>     input = complexInput();
        const std::span<const CF> in(input);
        const std::vector<float>  whole = spanPath(block, in, in.size());

        std::size_t mismatches = 0UZ;
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            mismatches += static_cast<std::size_t>(!sameValue(whole[i], static_cast<float>(std::hypot(static_cast<double>(input[i].real()), static_cast<double>(input[i].imag())))));
        }
        expect(eq(mismatches, 0UZ)) << "samples whose magnitude differs from the double hypot";
        expect(eq(chunkMismatches(block, in, whole), 0UZ)) << "chunked outputs that differ from one call";
#ifdef __GLIBC__
        // glibc's hypotf squares and sums the parts in double and rounds the square root once
        std::size_t libraryMismatches = 0UZ;
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            libraryMismatches += static_cast<std::size_t>(!sameValue(whole[i], std::abs(input[i])));
        }
        expect(eq(libraryMismatches, 0UZ)) << "samples whose magnitude differs from std::abs";
#endif
    };

    "a Magnitude scale and offset apply to the magnitude"_test = [] {
        Magnitude<CF> block({{"scale", 2.f}, {"offset", 0.5f}});
        applySettings(block);
        const std::vector<CF>    input{{3.f, 4.f}, {0.f, -1.f}, {0.f, 0.f}};
        const std::vector<float> output = spanPath(block, std::span<const CF>(input), input.size());
        expect(that % (output == std::vector<float>{10.5f, 2.5f, 0.5f}));
    };

    // gr::math::scaledAtan2 is within 2 ulp of the exact arctangent with g++ and 4 ulp with clang; the double
    // std::atan2 of the parts stands for the exact value, its own error far below a float ulp. This test checks the
    // wiring of the call; qa_Atan2 pins the accuracy of scaledAtan2 itself.
#if defined(__clang__)
    constexpr double kPhaseUlps = 4.0;
#else
    constexpr double kPhaseUlps = 2.0;
#endif
    "Phase is within the bound of std::atan2 of the parts, with C's zeros and NaN, at every chunk size"_test = [] {
        Phase<CF> block;
        applySettings(block);
        const std::vector<CF>     input = complexInput();
        const std::span<const CF> in(input);
        const std::vector<float>  whole = spanPath(block, in, in.size());

        std::size_t outside = 0UZ;
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            const double reference = std::atan2(static_cast<double>(input[i].imag()), static_cast<double>(input[i].real()));
            const double phase     = static_cast<double>(whole[i]);
            if (std::isnan(reference) || reference == 0.0) {
                outside += static_cast<std::size_t>(!sameValue(phase, reference));
            } else {
                outside += static_cast<std::size_t>(!(std::abs(phase - reference) <= kPhaseUlps * floatUlp(reference)));
            }
        }
        expect(eq(outside, 0UZ)) << "samples whose phase lies outside the bound";
        expect(eq(chunkMismatches(block, in, whole), 0UZ)) << "chunked outputs that differ from one call";
    };

    "a zero sample's Phase is std::arg's: 0 with the imaginary part's sign, or +-pi for a real part of -0"_test = [] {
        Phase<CF> block;
        applySettings(block);
        const std::vector<CF>    input{{0.f, 0.f}, {0.f, -0.f}, {-0.f, 0.f}, {-0.f, -0.f}};
        const std::vector<float> output     = spanPath(block, std::span<const CF>(input), input.size());
        std::size_t              mismatches = 0UZ;
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            mismatches += static_cast<std::size_t>(!sameValue(output[i], std::arg(input[i])));
        }
        expect(eq(mismatches, 0UZ));
    };
};

namespace {

/// @brief The signed zeros, the smallest subnormal, the smallest and largest normals, one, the infinities, a NaN and
/// each of `edges` with its two float neighbors, then random bit patterns, then values uniform in [`low`, `high`]. The
/// uniform values come last and apart, so that whole vectors of them take the simd form's path for ordinary arguments.
[[nodiscard]] std::vector<float> elementaryInput(std::initializer_list<float> edges, float low, float high) {
    using L = std::numeric_limits<float>;
    std::vector<float> input{0.f, L::denorm_min(), L::min(), 1.f, L::max(), L::infinity(), L::quiet_NaN()};
    for (const float edge : edges) {
        input.insert(input.end(), {std::nextafter(edge, 0.f), edge, std::nextafter(edge, L::infinity())});
    }
    const std::size_t positive = input.size();
    for (std::size_t i = 0UZ; i < positive; ++i) {
        input.push_back(-input[i]);
    }
    std::mt19937                                 rng(20261103U);
    std::uniform_int_distribution<std::uint32_t> bits;
    std::uniform_real_distribution<float>        uniform(low, high);
    for (std::size_t i = 0UZ; i < 4096UZ; ++i) {
        input.push_back(std::bit_cast<float>(bits(rng)));
    }
    for (std::size_t i = 0UZ; i < 4096UZ; ++i) {
        input.push_back(uniform(rng));
    }
    return input;
}

/// @brief True when `output` lies within `ulps` float steps of the exact value `reference`, the step taken at the
/// reference. A zero, infinite or NaN reference is matched exactly, sign included. A finite reference beyond the float
/// range stands at 2^128, one step past the largest float, where an infinite output also stands.
[[nodiscard]] bool withinUlps(float output, double reference, double ulps) {
    using L = std::numeric_limits<float>;
    if (std::isnan(reference) || std::isinf(reference) || reference == 0.0) {
        return sameValue(static_cast<double>(output), reference);
    }
    constexpr double kBeyond  = 0x1p128;
    const double     exact    = std::clamp(reference, -kBeyond, kBeyond);
    const double     value    = std::isinf(output) ? std::copysign(kBeyond, static_cast<double>(output)) : static_cast<double>(output);
    const int        exponent = std::clamp(std::ilogb(exact), L::min_exponent - 1, L::max_exponent - 1);
    return std::abs(value - exact) <= ulps * std::ldexp(1.0, exponent - (L::digits - 1));
}

/// @brief The outputs of the scalar call, the native simd call and the 32-wide simd call (the widest the scheduler
/// runs) of a float block made from `TBlock` that lie outside `ulps` steps of the double `reference`.
template<typename TBlock, typename F>
[[nodiscard]] std::size_t outsideBound(std::span<const float> in, F reference, double ulps) {
    static_assert(TBlock::kSimdPath);
    TBlock block;
    applySettings(block);
    std::size_t outside = 0UZ;
    for (const std::vector<float>& output : {scalarPath(block, in), simdPathOf<vir::stdx::native_simd<float>>(block, in), simdPathOf<vir::stdx::fixed_size_simd<float, 32>>(block, in)}) {
        for (std::size_t i = 0UZ; i < in.size(); ++i) {
            outside += static_cast<std::size_t>(!withinUlps(output[i], reference(static_cast<double>(in[i])), ulps));
        }
    }
    return outside;
}

/// @brief The outputs of a double block made from `TBlock` over `in`, called as the scheduler calls it, that are not
/// `reference`'s value: simd vectors where the block takes them, one sample at a time otherwise.
template<typename TBlock, typename F>
[[nodiscard]] std::size_t doubleMismatches(std::span<const float> in, F reference) {
    TBlock block;
    applySettings(block);
    std::vector<double> input(in.size());
    std::ranges::transform(in, input.begin(), [](float x) { return static_cast<double>(x); });
    const std::span<const double> values(input);
    std::vector<double>           output;
    if constexpr (TBlock::kSimdPath) {
        output = simdPathOf<vir::stdx::fixed_size_simd<double, 32>>(block, values);
    } else {
        output = scalarPath(block, values);
    }
    return mismatchCount(values, output, reference);
}

// core's gr::meta functions on float lanes are within 2 ulp of the exact result, with C's special values
constexpr double kElementaryUlps = 2.0;

/// @brief Expects the float block of `TBlock` within 2 ulp of `reference` over `input`, the double function of the
/// float argument standing for the exact result (its own error is far below a float step), and the double block to
/// give `reference`'s value: the C library's, one sample at a time.
template<template<typename> typename TBlock, typename F>
void expectElementary(const std::vector<float>& input, F reference) {
    using namespace boost::ut;
    expect(eq(outsideBound<TBlock<float>>(input, reference, kElementaryUlps), 0UZ)) << "float outputs outside the bound";
    expect(!TBlock<double>::kSimdPath) << "a double block takes simd vectors";
    expect(eq(doubleMismatches<TBlock<double>>(input, reference), 0UZ)) << "double outputs that differ from the C library's";
}

} // namespace

const boost::ut::suite<"Unary float elementary functions"> unaryElementary = [] {
    using namespace boost::ut;
    using namespace gr::blocks::math;
    using L = std::numeric_limits<float>;

    // The edges are where a result overflows, underflows, reaches 0 or 1 or changes form, and the bounds of the
    // arguments for which a vector of float lanes takes the shorter path.
    "Exp of float is within 2 ulp of double std::exp, and Exp of double is std::exp per sample"_test = [] {
        const std::vector<float> input = elementaryInput({0x1p-60f, 1.f, 87.f, 87.33654f, 88.72284f, 103.27893f, 103.97208f}, -87.f, 87.f);
        expectElementary<Exp>(input, [](double x) { return std::exp(x); });
    };
    "Log of float is within 2 ulp of double std::log, and Log of double is std::log per sample"_test = [] {
        const std::vector<float> input = elementaryInput({L::min(), 1.f, 2.f, 10.f}, 1e-3f, 1e4f);
        expectElementary<Log>(input, [](double x) { return std::log(x); });
    };
    "Log10 of float is within 2 ulp of double std::log10, and Log10 of double is std::log10 per sample"_test = [] {
        const std::vector<float> input = elementaryInput({L::min(), 1.f, 2.f, 10.f}, 1e-3f, 1e4f);
        expectElementary<Log10>(input, [](double x) { return std::log10(x); });
    };
    "Log2 of float is within 2 ulp of double std::log2, and Log2 of double is std::log2 per sample"_test = [] {
        const std::vector<float> input = elementaryInput({L::min(), 1.f, 2.f, 10.f}, 1e-3f, 1e4f);
        expectElementary<Log2>(input, [](double x) { return std::log2(x); });
    };
    "Sin of float is within 2 ulp of double std::sin, and Sin of double is std::sin per sample"_test = [] {
        const std::vector<float> input = elementaryInput({0x1p-27f, std::numbers::pi_v<float> / 2.f, std::numbers::pi_v<float>, 8192.f, 1e6f, 1e30f}, -8192.f, 8192.f);
        expectElementary<Sin>(input, [](double x) { return std::sin(x); });
    };
    "Cos of float is within 2 ulp of double std::cos, and Cos of double is std::cos per sample"_test = [] {
        const std::vector<float> input = elementaryInput({0x1p-27f, std::numbers::pi_v<float> / 2.f, std::numbers::pi_v<float>, 8192.f, 1e6f, 1e30f}, -8192.f, 8192.f);
        expectElementary<Cos>(input, [](double x) { return std::cos(x); });
    };
    "Tan of float is within 2 ulp of double std::tan, and Tan of double is std::tan per sample"_test = [] {
        const std::vector<float> input = elementaryInput({0x1p-27f, std::numbers::pi_v<float> / 2.f, std::numbers::pi_v<float>, 8192.f, 1e6f, 1e30f}, -8192.f, 8192.f);
        expectElementary<Tan>(input, [](double x) { return std::tan(x); });
    };
    "Atan of float is within 2 ulp of double std::atan, and Atan of double is std::atan per sample"_test = [] {
        const std::vector<float> input = elementaryInput({0x1p-28f, 1.f, 0x1p26f}, -100.f, 100.f);
        expectElementary<Atan>(input, [](double x) { return std::atan(x); });
    };
    "Sinh of float is within 2 ulp of double std::sinh, and Sinh of double is std::sinh per sample"_test = [] {
        const std::vector<float> input = elementaryInput({0x1p-28f, 1.f, 20.f, 86.f, 89.41599f}, -86.f, 86.f);
        expectElementary<Sinh>(input, [](double x) { return std::sinh(x); });
    };
    "Cosh of float is within 2 ulp of double std::cosh, and Cosh of double is std::cosh per sample"_test = [] {
        const std::vector<float> input = elementaryInput({0x1p-60f, 1.f, 20.f, 86.f, 89.41599f}, -86.f, 86.f);
        expectElementary<Cosh>(input, [](double x) { return std::cosh(x); });
    };
    "Tanh of float is within 2 ulp of double std::tanh, and Tanh of double is std::tanh per sample"_test = [] {
        const std::vector<float> input = elementaryInput({0x1p-28f, 0.75f, 9.01f, 20.f, 43.f}, -43.f, 43.f);
        expectElementary<Tanh>(input, [](double x) { return std::tanh(x); });
    };
};

int main() { /* not needed for UT */ }
