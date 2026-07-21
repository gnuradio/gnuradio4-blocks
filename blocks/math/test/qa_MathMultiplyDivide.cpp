#include <boost/ut.hpp>

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
};

int main() { /* not needed for UT */ }
