#include <boost/ut.hpp>

#include "MathTestHelpers.hpp"

#include <gnuradio-4.0/testing/SettingsLimits.hpp>

const boost::ut::suite<"basic math tests"> basicMath = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::math;
    constexpr auto kArithmeticTypes = std::tuple<uint8_t, int16_t, int32_t, float, std::complex<float>>();

    "defaults lie inside their limits"_test = [] {
        const Add<float> block;
        const auto       outside = gr::blocks::testing::settingsOutsideLimits(block);
        expect(outside.empty()) << std::format("settings outside their limits: {}", outside);
        expect(eq(block.in.size(), static_cast<std::size_t>(block.n_inputs)));
    };

    "given its default input count keeps that many inputs"_test = [] {
        const Size_t nDefault = Add<float>{}.n_inputs;
        Add<float>   block({{"n_inputs", nDefault}});
        block.init(block.progress);
        expect(eq(block.in.size(), static_cast<std::size_t>(nDefault)));
    };

    "Add"_test = []<typename T>(const T&) { //
        test_block<T, Add<T>>({
            .inputs = {gr::Tensor<T>(gr::data_from, {1, 2, 8, 17})}, //
            .output = gr::Tensor<T>(gr::data_from, {1, 2, 8, 17})    //
        });
        test_block<T, Add<T>>({
            .inputs = {gr::Tensor<T>(gr::data_from, {T(1), T(2), T(3), val<T>(4.2)}), //
                gr::Tensor<T>(gr::data_from, {T(5), T(6), T(7), val<T>(8.3)})},       //
            .output = gr::Tensor<T>(gr::data_from, {T(6), T(8), T(10), val<T>(12.5)}) //
        });
        test_block<T, Add<T>>({
            .inputs = {gr::Tensor<T>(gr::data_from, {12, 35, 18, 17}), //
                gr::Tensor<T>(gr::data_from, {31, 15, 27, 36}),        //
                gr::Tensor<T>(gr::data_from, {83, 46, 37, 41})},       //
            .output = gr::Tensor<T>(gr::data_from, {126, 96, 82, 94})  //
        });
    } | kArithmeticTypes;

    "Subtract"_test = []<typename T>(const T&) {
        test_block<T, Subtract<T>>({
            .inputs = {gr::Tensor<T>(gr::data_from, {1, 2, 8, 17})}, //
            .output = gr::Tensor<T>(gr::data_from, {1, 2, 8, 17})    //
        });
        test_block<T, Subtract<T>>({                                                   //
            .inputs = {gr::Tensor<T>(gr::data_from, {T(9), T(7), T(5), val<T>(3.5)}),  //
                gr::Tensor<T>(gr::data_from, {T(3), T(2), T(0), val<T>(1.2)})},        //
            .output = gr::Tensor<T>(gr::data_from, {T(6), T(5), T(5), val<T>(2.3)})}); //
        test_block<T, Subtract<T>>({                                                   //
            .inputs = {gr::Tensor<T>(gr::data_from, {15, 38, 88, 29}),                 //
                gr::Tensor<T>(gr::data_from, {3, 12, 26, 18}),                         //
                gr::Tensor<T>(gr::data_from, {0, 10, 50, 7})},                         //
            .output = gr::Tensor<T>(gr::data_from, {12, 16, 12, 4})});                 //
    } | kArithmeticTypes;
};

int main() { /* not needed for UT */ }
