#include <boost/ut.hpp>

#include <array>
#include <cmath>
#include <complex>
#include <span>

#include <gnuradio-4.0/testing/TestSpans.hpp>

#include "MathTestHelpers.hpp"

const boost::ut::suite<"multiply and divide tests"> multiplyDivide = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::math;
    constexpr auto kArithmeticTypes = std::tuple<uint8_t, int16_t, int32_t, float, std::complex<float>>();

    "Multiply"_test = []<typename T>(const T&) {
        test_block<T, Multiply<T>>({.inputs = {gr::Tensor<T>(gr::data_from, {1, 2, 8, 17})}, .output = gr::Tensor<T>(gr::data_from, {1, 2, 8, 17})});
        test_block<T, Multiply<T>>({.inputs = {gr::Tensor<T>(gr::data_from, {T(1), T(2), T(3), val<T>(4.0)}), gr::Tensor<T>(gr::data_from, {T(4), T(5), T(6), val<T>(7.1)})}, .output = gr::Tensor<T>(gr::data_from, {T(4), T(10), T(18), val<T>(28.4)})});
        test_block<T, Multiply<T>>({.inputs = {gr::Tensor<T>(gr::data_from, {0, 1, 2, 3}), gr::Tensor<T>(gr::data_from, {4, 5, 6, 2}), gr::Tensor<T>(gr::data_from, {8, 9, 10, 11})}, .output = gr::Tensor<T>(gr::data_from, {0, 45, 120, 66})});
    } | kArithmeticTypes;

    "Divide"_test = []<typename T>(const T&) {
        test_block<T, Divide<T>>({.inputs = {gr::Tensor<T>(gr::data_from, {1, 2, 8, 17})}, .output = gr::Tensor<T>(gr::data_from, {1, 2, 8, 17})});
        test_block<T, Divide<T>>({.inputs = {gr::Tensor<T>(gr::data_from, {T(9), T(4), T(5), val<T>(7.0)}), gr::Tensor<T>(gr::data_from, {T(3), T(4), T(1), val<T>(2.0)})}, .output = gr::Tensor<T>(gr::data_from, {T(3), T(1), T(5), val<T>(3.5)})});
        test_block<T, Divide<T>>({.inputs = {gr::Tensor<T>(gr::data_from, {0, 10, 40, 80}), gr::Tensor<T>(gr::data_from, {1, 2, 4, 20}), gr::Tensor<T>(gr::data_from, {1, 5, 5, 2})}, .output = gr::Tensor<T>(gr::data_from, {0, 1, 2, 2})});
    } | kArithmeticTypes;

    "Divide by a zero divisor yields zero for integral types"_test = []<typename T>(const T&) { test_block<T, Divide<T>>({.inputs = {gr::Tensor<T>(gr::data_from, {6, 8, 10, 12}), gr::Tensor<T>(gr::data_from, {2, 0, 5, 0})}, .output = gr::Tensor<T>(gr::data_from, {3, 0, 2, 0})}); } | std::tuple<uint8_t, int16_t, int32_t>();

    "a complex Multiply of two and three finite streams gives std::multiplies' products"_test = [] {
        using C                    = std::complex<float>;
        namespace spans            = gr::blocks::testing::span;
        const std::vector<C> first = finiteComplexInput();
        std::vector<C>       second(first.rbegin(), first.rend());
        std::vector<C>       third(first.size());
        for (std::size_t i = 0UZ; i < first.size(); ++i) {
            third[i] = first[(i * 61UZ) % first.size()];
        }

        auto twoInputs = Multiply<C>(property_map{{"n_inputs", Size_t{2U}}});
        twoInputs.init(twoInputs.progress);
        auto threeInputs = Multiply<C>(property_map{{"n_inputs", Size_t{3U}}});
        threeInputs.init(threeInputs.progress);

        std::vector<C>                       twoOut(first.size());
        std::vector<C>                       threeOut(first.size());
        std::array<spans::InputSpan<C>, 2UZ> two{spans::InputSpan<C>{std::span<const C>(first)}, spans::InputSpan<C>{std::span<const C>(second)}};
        std::array<spans::InputSpan<C>, 3UZ> three{spans::InputSpan<C>{std::span<const C>(first)}, spans::InputSpan<C>{std::span<const C>(second)}, spans::InputSpan<C>{std::span<const C>(third)}};
        spans::OutputSpan<C>                 twoSpan{std::span<C>(twoOut)};
        spans::OutputSpan<C>                 threeSpan{std::span<C>(threeOut)};
        std::ignore = twoInputs.processBulk(std::span<spans::InputSpan<C>>(two), twoSpan);
        std::ignore = threeInputs.processBulk(std::span<spans::InputSpan<C>>(three), threeSpan);

        std::size_t mismatches = 0UZ;
        for (std::size_t i = 0UZ; i < first.size(); ++i) {
            const C product = std::multiplies<C>{}(first[i], second[i]);
            mismatches += static_cast<std::size_t>(!sameParts(twoOut[i], product));
            if (std::isfinite(product.real()) && std::isfinite(product.imag())) { // C recovers an infinity from an overflowed product
                mismatches += static_cast<std::size_t>(!sameParts(threeOut[i], std::multiplies<C>{}(product, third[i])));
            }
        }
        expect(eq(mismatches, 0UZ)) << "products that differ from std::multiplies";
    };

    "a product or a quotient of streams drops the carrier estimates, a sum forwards them"_test = [] {
        using C = std::complex<float>;
        const std::vector<std::string_view> keys{"freq_est", "phase_est", "burst_id"};
        const gr::property_map              oneInput{{"n_inputs", gr::Size_t{1U}}};
        expect(keysCrossing<C, Multiply<C>>(keys, oneInput, "in#0") == std::set<std::string, std::less<>>{"burst_id"}) << "Multiply";
        expect(keysCrossing<C, Divide<C>>(keys, oneInput, "in#0") == std::set<std::string, std::less<>>{"burst_id"}) << "Divide";
        expect(keysCrossing<C, Add<C>>(keys, oneInput, "in#0") == std::set<std::string, std::less<>>{"burst_id", "freq_est", "phase_est"}) << "Add";
    };
};

int main() { /* not needed for UT */ }
