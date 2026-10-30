#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
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

int main() { /* not needed for UT */ }
