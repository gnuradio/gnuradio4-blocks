#include <boost/ut.hpp>

#include <algorithm>
#include <array>
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

/// @brief The output of one simd call per vector, over the input padded with zeros to whole vectors.
template<typename TBlock, typename T>
[[nodiscard]] std::vector<T> simdPath(const TBlock& block, std::span<const T> input) {
    using V                  = vir::stdx::native_simd<T>;
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

} // namespace

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
