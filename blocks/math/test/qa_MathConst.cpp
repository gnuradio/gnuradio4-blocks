#include <boost/ut.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/math/Math.hpp>

const boost::ut::suite<"constant math tests"> constantMath = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::math;
    constexpr auto kArithmeticTypes = std::tuple<uint8_t, int16_t, int32_t, float, std::complex<float>>();

    "AddConst"_test = []<typename T>(const T&) {
        expect(eq(AddConst<T>().processOne(T(4)), T(4) + T(1))) << std::format("AddConst test for type {}\n", meta::type_name<T>());
        auto block = AddConst<T>(property_map{{"value", T(2)}});
        block.init(block.progress);
        expect(eq(block.processOne(T(4)), T(4) + T(2))) << std::format("AddConst(2) test for type {}\n", meta::type_name<T>());
    } | kArithmeticTypes;

    "SubtractConst"_test = []<typename T>(const T&) {
        expect(eq(SubtractConst<T>().processOne(T(4)), T(4) - T(1))) << std::format("SubtractConst test for type {}\n", meta::type_name<T>());
        auto block = SubtractConst<T>(property_map{{"value", T(2)}});
        block.init(block.progress);
        expect(eq(block.processOne(T(4)), T(4) - T(2))) << std::format("SubtractConst(2) test for type {}\n", meta::type_name<T>());
    } | kArithmeticTypes;

    "MultiplyConst"_test = []<typename T>(const T&) {
        expect(eq(MultiplyConst<T>().processOne(T(4)), T(4) * T(1))) << std::format("MultiplyConst test for type {}\n", meta::type_name<T>());
        auto block = MultiplyConst<T>(property_map{{"value", T(2)}});
        block.init(block.progress);
        expect(eq(block.processOne(T(4)), T(4) * T(2))) << std::format("MultiplyConst(2) test for type {}\n", meta::type_name<T>());
    } | kArithmeticTypes;

    "DivideConst"_test = []<typename T>(const T&) {
        expect(eq(DivideConst<T>().processOne(T(4)), T(4) / T(1))) << std::format("DivideConst test for type {}\n", meta::type_name<T>());
        auto block = DivideConst<T>(property_map{{"value", T(2)}});
        block.init(block.progress);
        expect(eq(block.processOne(T(4)), T(4) / T(2))) << std::format("DivideConst(2) test for type {}\n", meta::type_name<T>());
    } | kArithmeticTypes;

    "DivideConst by a zero value yields zero for integral types"_test = []<typename T>(const T&) {
        auto block = DivideConst<T>(property_map{{"value", T(0)}});
        block.init(block.progress);
        expect(eq(block.processOne(T(4)), T(0))) << std::format("DivideConst(0) test for type {}\n", meta::type_name<T>());
    } | std::tuple<uint8_t, int16_t, int32_t>();

    "noncanonical header types smoke test"_test = [] {
        static_assert(BlockLike<Add<double>>);
        static_assert(BlockLike<Multiply<std::complex<double>>>);

        AddConst<double> add(property_map{{"value", 2.5}});
        add.init(add.progress);
        expect(eq(add.processOne(1.5), 4.0));

        MultiplyConst<std::complex<double>> multiply(property_map{{"value", std::complex<double>{2.0, 0.0}}});
        multiply.init(multiply.progress);
        expect(eq(multiply.processOne(std::complex<double>{1.0, 1.0}), std::complex<double>{2.0, 2.0}));
    };
};

int main() { /* not needed for UT */ }
