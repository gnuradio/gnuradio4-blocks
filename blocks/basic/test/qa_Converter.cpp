#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/basic/ConverterBlocks.hpp>
#include <gnuradio-4.0/basic/NullSources.hpp>
#include <gnuradio-4.0/basic/VectorSink.hpp>
#include <gnuradio-4.0/basic/VectorSource.hpp>

#include <array>
#include <bit>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <format>
#include <iostream>
#include <limits>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

namespace unittest {
template<class T>
[[nodiscard]] constexpr auto abs_diff(const std::complex<T>& t, const std::complex<T>& u) {
    return std::abs(t - u);
}
} // namespace unittest

template<typename... Ts>
inline constexpr std::ostream& operator<<(std::ostream& os, const std::tuple<Ts...>& tuple) noexcept {
    std::apply(
        [&os](const Ts&... args) {
            os << '(';
            std::size_t n = 0;
            ((os << args << (++n != sizeof...(Ts) ? ", " : "")), ...);
            os << ')';
        },
        tuple);
    return os;
}

#include <boost/ut.hpp>

namespace unittest {
/// Returns an integer `value` as `long long` or `unsigned long long`, which `std::format` prints as a number, and any
/// other value unchanged.
template<typename T>
[[nodiscard]] constexpr auto printable(T value) {
    if constexpr (std::is_integral_v<T>) {
        return static_cast<std::conditional_t<std::is_signed_v<T>, long long, unsigned long long>>(value);
    } else {
        return value;
    }
}

/// Returns the integer `value` limited to the range of `R`.
template<typename R>
[[nodiscard]] constexpr R clampToRange(long long value) {
    using Limits = std::numeric_limits<R>;
    if (value < static_cast<long long>(Limits::min())) {
        return Limits::min();
    }
    if (value > static_cast<long long>(Limits::max())) {
        return Limits::max();
    }
    return static_cast<R>(value);
}

/// Returns `value` rounded to the nearest integer, ties to even, and limited to the range of `R`.
template<typename R>
[[nodiscard]] R nearestInRange(double value) {
    return clampToRange<R>(static_cast<long long>(std::nearbyint(value)));
}

/// Runs `block.processOne` on each input alone and on all inputs as one SIMD vector, and expects `expected` from both.
template<typename TBlock, typename T, typename R, std::size_t N>
void expectBothPaths(const TBlock& block, const std::array<T, N>& inputs, const std::array<R, N>& expected) {
    using V = vir::stdx::fixed_size_simd<T, N>;
    const V     batch([&inputs](auto i) { return inputs[i]; });
    const auto  simdResult = block.processOne(batch);
    std::string mismatches;
    for (std::size_t i = 0UZ; i < N; ++i) {
        const R scalar = block.processOne(inputs[i]);
        const R lane   = simdResult[i];
        if (scalar != expected[i] || lane != expected[i]) {
            mismatches += std::format("{} {}: scalar {}, SIMD {}, expected {}\n", gr::meta::type_name<TBlock>(), printable(inputs[i]), printable(scalar), printable(lane), printable(expected[i]));
        }
    }
    boost::ut::expect(mismatches.empty()) << mismatches;
}

/// Expects `Convert<T, R>` to give the lowest and the highest value of `R` for themselves, and the nearest integer
/// within the range for the values 0.5 inside and 0.5 outside each of them.
template<typename T, typename R>
void expectAtLimits() {
    using Limits                  = std::numeric_limits<R>;
    const double             low  = static_cast<double>(Limits::min());
    const double             high = static_cast<double>(Limits::max());
    const std::array<T, 6UZ> inputs{static_cast<T>(low), static_cast<T>(high), static_cast<T>(low + 0.5), static_cast<T>(high - 0.5), static_cast<T>(low - 0.5), static_cast<T>(high + 0.5)};
    std::array<R, 6UZ>       expected{};
    for (std::size_t i = 0UZ; i < inputs.size(); ++i) {
        expected[i] = nearestInRange<R>(static_cast<double>(inputs[i]));
    }
    boost::ut::expect(expected[0] == Limits::min() && expected[1] == Limits::max() && expected[4] == Limits::min() && expected[5] == Limits::max());
    expectBothPaths(gr::blocks::basic::Convert<T, R>{}, inputs, expected);
}

/// Expects `Convert<T, R>` to give 0 for NaN and the lowest and highest value of `R` for the two infinities. The NaNs
/// are the quiet one, the one with the lowest mantissa and the negative one with the lowest mantissa; the bit patterns
/// come from the infinity's.
template<std::floating_point T, typename R>
void expectNaNAndInfinity() {
    using Limits             = std::numeric_limits<R>;
    using Bits               = std::conditional_t<sizeof(T) == sizeof(std::uint32_t), std::uint32_t, std::uint64_t>;
    constexpr T    kInfinity = std::numeric_limits<T>::infinity();
    constexpr Bits kLowest   = std::bit_cast<Bits>(kInfinity) + 1U;
    constexpr Bits kSign     = Bits{1} << (std::numeric_limits<Bits>::digits - 1);
    expectBothPaths(gr::blocks::basic::Convert<T, R>{}, std::array<T, 5UZ>{std::numeric_limits<T>::quiet_NaN(), std::bit_cast<T>(kLowest), std::bit_cast<T>(kLowest | kSign), kInfinity, -kInfinity}, std::array<R, 5UZ>{R{0}, R{0}, R{0}, Limits::max(), Limits::min()});
}

/// Runs `inputs` through a graph of a `VectorSource`, `TBlock` with `settings` and a `VectorSink`, and returns what the
/// sink received.
template<typename TBlock, typename T, typename R>
std::vector<R> runGraph(const std::vector<T>& inputs, const gr::property_map& settings) {
    gr::Graph flow;
    auto&     source    = flow.emplaceBlock<gr::blocks::basic::VectorSource<T>>({{"values", inputs}});
    auto&     converter = flow.emplaceBlock<TBlock>(settings);
    auto&     sink      = flow.emplaceBlock<gr::blocks::basic::VectorSink<R>>();
    boost::ut::expect(boost::ut::fatal(flow.connect<"out", "in">(source, converter).has_value()));
    boost::ut::expect(boost::ut::fatal(flow.connect<"out", "in">(converter, sink).has_value()));
    gr::scheduler::Simple<> scheduler;
    boost::ut::expect(boost::ut::fatal(scheduler.exchange(std::move(flow)).has_value()));
    boost::ut::expect(scheduler.runAndWait().has_value());
    return sink.data.value;
}
} // namespace unittest

const boost::ut::suite<"basic Conversion tests"> basicConversion = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::basic;
    namespace stdx = vir::stdx;

    using TArithmeticTypes = std::tuple<uint8_t, int16_t, int32_t, float>;
    struct ConvertBlock {};
    struct ScalingConvertBlock {};

    "ConversionTest"_test = []<typename TSwitch>(TSwitch&& /*noop*/) {
        constexpr static bool kIsScalingBlock = std::is_same_v<TSwitch, ScalingConvertBlock>;

        "static block properties"_test = [] {
            using TBlock = std::conditional_t<kIsScalingBlock, ScalingConvert<int32_t, float>, Convert<int32_t, float>>;
            static_assert(gr::traits::block::can_processOne_scalar<TBlock>);
            static_assert(gr::traits::block::can_processOne_simd<TBlock>);
            static_assert(HasProcessOneFunction<TBlock>);
            static_assert(HasConstProcessOneFunction<TBlock>);
            static_assert(HasRequiredProcessFunction<TBlock>);

            // To force the runtime code-coverage
            expect(HasProcessOneFunction<TBlock>);
            expect(HasConstProcessOneFunction<TBlock>);
        };

        "up-convert std::uint8_t to ..."_test = []<typename R>(R /*noop*/) {
            using T        = uint8_t;
            using TConvert = std::conditional_t<kIsScalingBlock, ScalingConvert<T, R>, Convert<T, R>>;
            TConvert converter;
            R        result = converter.processOne(/*input*/ static_cast<T>(42));

            expect(eq(result, static_cast<R>(42))) << std::format("convert from {} to {} failed\n", gr::meta::type_name<T>(), gr::meta::type_name<R>());
        } | TArithmeticTypes();

        "up-convert std::simd<std::uint8_t> to ..."_test = []<typename R>(R /*noop*/) {
            using T = uint8_t;
            // careful here: max_fixed_size is 32 *except with AVX-512 and sizeof(T) == 1* where it is 64.
            using V       = std::conditional_t<stdx::native_simd<T>::size() <= stdx::simd_abi::max_fixed_size<R>, stdx::native_simd<T>, stdx::simd<T, stdx::simd_abi::deduce_t<T, stdx::simd_abi::max_fixed_size<R>>>>;
            using RetType = stdx::rebind_simd_t<R, V>;

            using TConvert = std::conditional_t<kIsScalingBlock, ScalingConvert<T, R>, Convert<T, R>>;
            TConvert converter;
            RetType  result = converter.processOne(/*input*/ static_cast<V>(42));

            for (size_t i = 0; i < RetType::size(); ++i) {
                expect(eq(static_cast<R>(result[i]), static_cast<R>(42))) << std::format("up-convert from {} to {} failed at index {}\n", gr::meta::type_name<V>(), gr::meta::type_name<RetType>(), i);
            }
        } | TArithmeticTypes();

        "down-convert std::simd<float> to ..."_test = []<typename R>(R /*noop*/) {
            using T       = float;
            using V       = stdx::native_simd<T>;
            using RetType = stdx::rebind_simd_t<R, V>;

            using TConvert = std::conditional_t<kIsScalingBlock, ScalingConvert<T, R>, Convert<T, R>>;
            TConvert converter;
            RetType  result = converter.processOne(/*input*/ static_cast<V>(42));
            if constexpr (kIsScalingBlock) {
                converter.scale = 0.5;
                result          = converter.processOne(/*input*/ static_cast<V>(42));
            } else {
                result = converter.processOne(/*input*/ static_cast<V>(21));
            }

            for (size_t i = 0; i < RetType::size(); ++i) {
                expect(eq(static_cast<R>(result[i]), static_cast<R>(21))) << std::format("down-convert from {} to {} failed at index {}", gr::meta::type_name<V>(), gr::meta::type_name<RetType>(), i);
            }
        } | TArithmeticTypes();

        "down-convert std::simd<int64_t> to ..."_test = []<typename R>(R /*noop*/) {
            using T       = int64_t;
            using V       = stdx::native_simd<T>;
            using RetType = stdx::rebind_simd_t<R, V>;

            using TConvert = std::conditional_t<kIsScalingBlock, ScalingConvert<T, R>, Convert<T, R>>;
            TConvert converter;
            if constexpr (kIsScalingBlock) {
                converter.scale = 1.0;
            }
            RetType result = converter.processOne(/*input*/ static_cast<V>(42));

            for (size_t i = 0; i < RetType::size(); ++i) {
                expect(eq(static_cast<R>(result[i]), static_cast<R>(42))) << std::format("down-convert from {} to {} failed at index {}", gr::meta::type_name<V>(), gr::meta::type_name<RetType>(), i);
            }
        } | TArithmeticTypes();
    } | std::tuple<ConvertBlock, ScalingConvertBlock>();
};

const boost::ut::suite<"complex To/From conversion tests"> complexConversion = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::basic;
    namespace stdx = vir::stdx;

    constexpr auto kArithmeticTypes = std::tuple<float, double>();

    "std::complex<T> to ..."_test = []<typename T>(T /*noop*/) {
        "std::abs([complex<T>, T])"_test = [] {
            static_assert(HasConstProcessOneFunction<Abs<std::complex<T>>>);
            static_assert(HasConstProcessOneFunction<Abs<T>>);
            expect(HasConstProcessOneFunction<Abs<std::complex<T>>>);
            expect(HasConstProcessOneFunction<Abs<T>>);

            Abs<std::complex<T>> complexConverter;
            expect(eq(complexConverter.processOne(std::complex<T>{1, 0}), T(1)));
            expect(eq(complexConverter.processOne(std::complex<T>{0, 1}), T(1)));
            expect(eq(complexConverter.processOne(std::complex<T>{1, 1}), T(std::sqrt(2))));

            Abs<T> converter;
            expect(eq(converter.processOne(T(1)), T(1)));
        };

        "std::Real([complex<T>, T])"_test = [] {
            static_assert(HasConstProcessOneFunction<Real<std::complex<T>>>);
            static_assert(HasConstProcessOneFunction<Real<T>>);
            expect(HasConstProcessOneFunction<Real<std::complex<T>>>);
            expect(HasConstProcessOneFunction<Real<T>>);

            Real<std::complex<T>> complexConverter;
            expect(eq(complexConverter.processOne(std::complex<T>{1, 0}), T(1)));
            expect(eq(complexConverter.processOne(std::complex<T>{0, 1}), T(0)));
            expect(eq(complexConverter.processOne(std::complex<T>{1, 1}), T(1)));

            Real<T> converter;
            expect(eq(converter.processOne(T(1)), T(1)));
        };

        "std::Imag([complex<T>, T])"_test = [] {
            static_assert(HasConstProcessOneFunction<Imag<std::complex<T>>>);
            static_assert(HasConstProcessOneFunction<Imag<T>>);
            expect(HasConstProcessOneFunction<Imag<std::complex<T>>>);
            expect(HasConstProcessOneFunction<Imag<T>>);

            Imag<std::complex<T>> complexConverter;
            expect(eq(complexConverter.processOne(std::complex<T>{1, 0}), T(0)));
            expect(eq(complexConverter.processOne(std::complex<T>{0, 1}), T(1)));
            expect(eq(complexConverter.processOne(std::complex<T>{1, 1}), T(1)));

            Imag<T> converter;
            expect(eq(converter.processOne(T(1)), T(0)));
        };

        "std::arg([complex<T>, T])"_test = [] {
            static_assert(HasConstProcessOneFunction<Arg<std::complex<T>>>);
            static_assert(HasConstProcessOneFunction<Arg<T>>);
            expect(HasConstProcessOneFunction<Arg<std::complex<T>>>);
            expect(HasConstProcessOneFunction<Arg<T>>);

            Arg<std::complex<T>> complexConverter;
            expect(eq(complexConverter.processOne(std::complex<T>{1, 0}), T(0.0) * std::numbers::pi_v<T>));
            expect(eq(complexConverter.processOne(std::complex<T>{0, 1}), T(0.5) * std::numbers::pi_v<T>));
            expect(eq(complexConverter.processOne(std::complex<T>{1, 1}), T(0.25) * std::numbers::pi_v<T>));

            Arg<T> converter;
            expect(eq(converter.processOne(T(1)), T(0)));
        };

        "rad <-> deg"_test = [] {
            static_assert(HasConstProcessOneFunction<RadiansToDegree<T>>);
            static_assert(HasConstProcessOneFunction<DegreeToRadians<T>>);
            expect(HasConstProcessOneFunction<RadiansToDegree<T>>);
            expect(HasConstProcessOneFunction<DegreeToRadians<T>>);

            RadiansToDegree<T> rad2deg;
            expect(eq(rad2deg.processOne(T(0)), T(0))) << "0 [rad] = 0°";
            expect(eq(rad2deg.processOne(T(0.5) * std::numbers::pi_v<T>), T(90))) << "pi/2 [rad] = 90°";
            expect(eq(rad2deg.processOne(std::numbers::pi_v<T>), T(180))) << "pi [rad] = 180°";

            DegreeToRadians<T> deg2rad;
            expect(eq(deg2rad.processOne(T(0)), T(0))) << "0° = 0 [rad]";
            expect(eq(deg2rad.processOne(T(90)), T(0.5) * std::numbers::pi_v<T>)) << "90° = pi/2 [rad]";
            expect(eq(deg2rad.processOne(T(180)), T(std::numbers::pi_v<T>))) << "180° = pi [rad]";

            for (T angle = T(0); angle < T(360); angle += T(10)) {
                T val = rad2deg.processOne(deg2rad.processOne(angle));
                expect(approx(val, angle, T(1e-3))) << std::format("identity {}° - diff {}°", angle, (val - angle));
            }
        };

        "complex <-> {real, imag}"_test = [] {
            static_assert(HasConstProcessOneFunction<ToRealImag<std::complex<T>>>);
            static_assert(HasConstProcessOneFunction<RealImagToComplex<T>>);
            expect(HasConstProcessOneFunction<ToRealImag<std::complex<T>>>);
            expect(HasConstProcessOneFunction<RealImagToComplex<T>>);

            ToRealImag<std::complex<T>> complexConverter;
            expect(eq(complexConverter.processOne(std::complex<T>{1, 0}), std::tuple<T, T>{1, 0}));
            expect(eq(complexConverter.processOne(std::complex<T>{0, 1}), std::tuple<T, T>{0, 1}));
            expect(eq(complexConverter.processOne(std::complex<T>{1, 1}), std::tuple<T, T>{1, 1}));

            RealImagToComplex<T> realImagConverter;
            expect(eq(realImagConverter.processOne(T(1), T(0)), std::complex<T>{1, 0}));
            expect(eq(realImagConverter.processOne(T(0), T(1)), std::complex<T>{0, 1}));
            expect(eq(realImagConverter.processOne(T(1), T(1)), std::complex<T>{1, 1}));
        };

        "complex <-> {magnitude, phase}"_test = [] {
            static_assert(HasConstProcessOneFunction<ToMagPhase<std::complex<T>>>);
            static_assert(HasConstProcessOneFunction<MagPhaseToComplex<T>>);
            expect(HasConstProcessOneFunction<ToMagPhase<std::complex<T>>>);
            expect(HasConstProcessOneFunction<MagPhaseToComplex<T>>);

            ToMagPhase<std::complex<T>> magPhaseConverter;
            expect(eq(magPhaseConverter.processOne(std::complex<T>{1, 0}), std::tuple<T, T>{1, 0}));
            expect(eq(magPhaseConverter.processOne(std::complex<T>{0, 1}), std::tuple<T, T>{1, std::numbers::pi_v<T> / 2}));
            expect(eq(magPhaseConverter.processOne(std::complex<T>{1, 1}), std::tuple<T, T>{std::sqrt(2), std::numbers::pi_v<T> / 4}));

            MagPhaseToComplex<T> magPhaseToComplexConverter;
            expect(unittest::abs_diff(magPhaseToComplexConverter.processOne(T(1), T(0)), std::complex<T>{1, 0}) < T(1e-3));
            expect(unittest::abs_diff(magPhaseToComplexConverter.processOne(T(1), T(std::numbers::pi_v<T> / 2)), std::complex<T>{0, 1}) < T(1e-3));
            expect(unittest::abs_diff(magPhaseToComplexConverter.processOne(T(std::sqrt(2)), T(std::numbers::pi_v<T> / 4)), std::complex<T>{1, 1}) < T(1e-3));
        };

        "complex <-> interleaved"_test = []<typename R> {
            static_assert(HasNoexceptProcessBulkFunction<ComplexToInterleaved<std::complex<T>, R>>);
            static_assert(HasNoexceptProcessBulkFunction<InterleavedToComplex<R, std::complex<T>>>);
            expect(HasNoexceptProcessBulkFunction<ComplexToInterleaved<std::complex<T>, R>>);
            expect(HasNoexceptProcessBulkFunction<InterleavedToComplex<R, std::complex<T>>>);

            std::vector<std::complex<T>> complexData = {{1, 2}, {3, 4}, {5, 6}};
            std::vector<T>               expected{1, 2, 3, 4, 5, 6};
            std::vector<R>               interleavedData(complexData.size() * 2);
            std::vector<std::complex<T>> outputComplexData(complexData.size());

            ComplexToInterleaved<std::complex<T>, R> complexToInterleaved;
            InterleavedToComplex<R, std::complex<T>> interleavedToComplex;

            expect(complexToInterleaved.processBulk(complexData, interleavedData) == gr::work::Status::OK);
            expect(eq(interleavedData[0], R(1.0f)));
            expect(eq(interleavedData[1], R(2.0f)));
            expect(eq(interleavedData[2], R(3.0f)));
            expect(eq(interleavedData[3], R(4.0f)));
            expect(eq(interleavedData[4], R(5.0f)));
            expect(eq(interleavedData[5], R(6.0f)));

            expect(interleavedToComplex.processBulk(interleavedData, outputComplexData) == gr::work::Status::OK);
            expect(eq(outputComplexData[0], std::complex<T>{T(1.0f), T(2.0f)}));
            expect(eq(outputComplexData[1], std::complex<T>{T(3.0f), T(4.0f)}));
            expect(eq(outputComplexData[2], std::complex<T>{T(5.0f), T(6.0f)}));
        } | std::tuple<float, double, std::int8_t, std::int16_t>();
    } | kArithmeticTypes;
};

const boost::ut::suite<"converter port names"> converterPortNames = [] {
    using namespace boost::ut;
    using namespace gr::blocks::basic;
    using gr::blocks::basic::NullSink;
    using gr::blocks::basic::NullSource;

    // graph files connect these blocks by port name
    "the one-input converters keep their port names"_test = [] {
        gr::Graph flow;
        auto&     complexSource = flow.emplaceBlock<NullSource<std::complex<float>>>();
        auto&     realSource    = flow.emplaceBlock<NullSource<float>>();
        auto&     abs           = flow.emplaceBlock<Abs<std::complex<float>>>();
        auto&     real          = flow.emplaceBlock<Real<std::complex<float>>>();
        auto&     imag          = flow.emplaceBlock<Imag<std::complex<float>>>();
        auto&     arg           = flow.emplaceBlock<Arg<std::complex<float>>>();
        auto&     toDegree      = flow.emplaceBlock<RadiansToDegree<float>>();
        auto&     toRadians     = flow.emplaceBlock<DegreeToRadians<float>>();

        expect(flow.connect<"out", "in">(complexSource, abs).has_value());
        expect(flow.connect<"out", "in">(complexSource, real).has_value());
        expect(flow.connect<"out", "in">(complexSource, imag).has_value());
        expect(flow.connect<"out", "in">(complexSource, arg).has_value());
        expect(flow.connect<"out", "rad">(realSource, toDegree).has_value());
        expect(flow.connect<"out", "deg">(realSource, toRadians).has_value());
        expect(flow.connect<"abs", "in">(abs, flow.emplaceBlock<NullSink<float>>()).has_value());
        expect(flow.connect<"real", "in">(real, flow.emplaceBlock<NullSink<float>>()).has_value());
        expect(flow.connect<"imag", "in">(imag, flow.emplaceBlock<NullSink<float>>()).has_value());
        expect(flow.connect<"arg", "in">(arg, flow.emplaceBlock<NullSink<float>>()).has_value());
        expect(flow.connect<"deg", "in">(toDegree, flow.emplaceBlock<NullSink<float>>()).has_value());
        expect(flow.connect<"rad", "in">(toRadians, flow.emplaceBlock<NullSink<float>>()).has_value());
    };
};

const boost::ut::suite<"conversion range and rounding"> conversionRange = [] {
    using namespace boost::ut;
    using namespace gr::blocks::basic;
    using unittest::clampToRange;
    using unittest::expectBothPaths;

    constexpr static float kNaN = std::numeric_limits<float>::quiet_NaN();
    constexpr static float kInf = std::numeric_limits<float>::infinity();

    "float to integer rounds to nearest, ties to even"_test = [] {
        expectBothPaths(Convert<float, std::int16_t>{}, std::array{2.5f, 3.5f, -2.5f, -3.5f, 0.5f, 1.5f, 2.4f, -0.7f}, std::array<std::int16_t, 8UZ>{2, 4, -2, -4, 0, 2, 2, -1});
        expectBothPaths(Convert<double, std::int64_t>{}, std::array{2.5, 3.5, -2.5, -3.5, 0.7, -0.7}, std::array<std::int64_t, 6UZ>{2, 4, -2, -4, 1, -1});
        // a floating-point input other than float and double rounds in double
        static_assert(std::is_same_v<decltype(Convert<long double, std::int16_t>{}.processOne(2.5L)), std::int16_t>);
        expectBothPaths(Convert<long double, std::int16_t>{}, std::array{2.5L, 3.5L, -0.7L, 40000.L}, std::array<std::int16_t, 4UZ>{2, 4, -1, std::numeric_limits<std::int16_t>::max()});
    };

    "float to integer saturates at the range and maps NaN to 0"_test = []<typename R>(R /*noop*/) {
        using Limits = std::numeric_limits<R>;
        expectBothPaths(Convert<float, R>{}, std::array{40000.f, -40000.f, 1e10f, -1e10f, kNaN, kInf, -kInf}, std::array{clampToRange<R>(40000), clampToRange<R>(-40000), Limits::max(), Limits::min(), R{0}, Limits::max(), Limits::min()});
    } | std::tuple<std::uint8_t, std::int16_t, std::int32_t>();

    "float to integer at and just past each limit"_test = [] {
        using Int16  = std::numeric_limits<std::int16_t>;
        using Uint8  = std::numeric_limits<std::uint8_t>;
        using Int32  = std::numeric_limits<std::int32_t>;
        using Int64  = std::numeric_limits<std::int64_t>;
        using Uint64 = std::numeric_limits<std::uint64_t>;
        // a tie next to a limit rounds to its even neighbor, and a value past a limit saturates
        expectBothPaths(Convert<float, std::int16_t>{}, std::array{32766.5f, 32767.f, 32767.5f, 32768.f, -32767.5f, -32768.f, -32768.5f, -32768.6f}, std::array{std::int16_t{32766}, Int16::max(), Int16::max(), Int16::max(), Int16::min(), Int16::min(), Int16::min(), Int16::min()});
        expectBothPaths(Convert<float, std::uint8_t>{}, std::array{254.5f, 255.f, 255.5f, 256.f, 0.5f, -0.4f, -0.5f, -0.6f}, std::array{std::uint8_t{254}, Uint8::max(), Uint8::max(), Uint8::max(), std::uint8_t{0}, std::uint8_t{0}, std::uint8_t{0}, std::uint8_t{0}});
        // the largest float below 2^31 and the next one below -2^31
        expectBothPaths(Convert<float, std::int32_t>{}, std::array{2147483520.f, 2147483648.f, -2147483648.f, -2147483904.f}, std::array{std::int32_t{2147483520}, Int32::max(), Int32::min(), Int32::min()});
        // the largest double below 2^63 and the next one below -2^63
        expectBothPaths(Convert<double, std::int64_t>{}, std::array{9223372036854774784.0, 9223372036854775808.0, -9223372036854775808.0, -9223372036854777856.0}, std::array{std::int64_t{9223372036854774784}, Int64::max(), Int64::min(), Int64::min()});
        // the largest float below 2^64
        expectBothPaths(Convert<float, std::uint64_t>{}, std::array{18446742974197923840.f, 18446744073709551616.f, -0.5f, -1.f}, std::array{std::uint64_t{18446742974197923840ULL}, Uint64::max(), std::uint64_t{0}, std::uint64_t{0}});
    };

    "NaN and infinity through a double input and a 64-bit output"_test = [] {
        using Bits = gr::blocks::basic::detail::RoundingBits<float>;
        using Wide = gr::blocks::basic::detail::RoundingBits<double>;
        // the first NaN is the bit pattern after the infinity's
        expect(Bits::kFirstNaN == std::bit_cast<std::int32_t>(std::numeric_limits<float>::infinity()) + 1);
        expect(Wide::kFirstNaN == std::bit_cast<std::int64_t>(std::numeric_limits<double>::infinity()) + 1);
        unittest::expectNaNAndInfinity<double, std::int16_t>();
        unittest::expectNaNAndInfinity<double, std::int32_t>();
        unittest::expectNaNAndInfinity<double, std::int64_t>();
        unittest::expectNaNAndInfinity<double, std::uint64_t>();
        unittest::expectNaNAndInfinity<float, std::int64_t>();
        unittest::expectNaNAndInfinity<float, std::uint64_t>();
    };

    "float and double to integer at each limit and 0.5 inside and outside it"_test = [] {
        unittest::expectAtLimits<float, std::int8_t>();
        unittest::expectAtLimits<float, std::uint8_t>();
        unittest::expectAtLimits<float, std::int16_t>();
        unittest::expectAtLimits<float, std::uint16_t>();
        unittest::expectAtLimits<double, std::int8_t>();
        unittest::expectAtLimits<double, std::uint8_t>();
        unittest::expectAtLimits<double, std::int16_t>();
        unittest::expectAtLimits<double, std::int32_t>();
        unittest::expectAtLimits<double, std::uint32_t>();
    };

    "float to int32_t on both sides of 2^22 and 2^23"_test = [] {
        // a float steps by 0.5 from 2^22 on and by 1 from 2^23 on
        constexpr std::array<float, 12UZ> kNear{4194302.5f, 4194303.5f, 4194303.75f, 4194304.f, 4194304.5f, 4194305.5f, 8388606.5f, 8388607.5f, 8388608.f, 8388609.f, -4194303.5f, -8388607.5f};
        std::array<std::int32_t, 12UZ>    expected{};
        for (std::size_t i = 0UZ; i < kNear.size(); ++i) {
            expected[i] = unittest::nearestInRange<std::int32_t>(static_cast<double>(kNear[i]));
        }
        expectBothPaths(Convert<float, std::int32_t>{}, kNear, expected);
    };

    "integer narrowing saturates at the range"_test = [] {
        constexpr std::array<std::int32_t, 6UZ> kWide{70000, -70000, 40000, -1, 255, 256};
        std::array<std::int16_t, 6UZ>           toInt16{};
        std::array<std::uint8_t, 6UZ>           toUint8{};
        for (std::size_t i = 0UZ; i < kWide.size(); ++i) {
            toInt16[i] = clampToRange<std::int16_t>(kWide[i]);
            toUint8[i] = clampToRange<std::uint8_t>(kWide[i]);
        }
        expect(eq(toInt16[0], std::numeric_limits<std::int16_t>::max()));
        expect(eq(toUint8[3], std::uint8_t{0}));
        expectBothPaths(Convert<std::int32_t, std::int16_t>{}, kWide, toInt16);
        expectBothPaths(Convert<std::int32_t, std::uint8_t>{}, kWide, toUint8);
        expectBothPaths(Convert<std::int16_t, std::uint8_t>{}, std::array<std::int16_t, 4UZ>{-1, 300, -32768, 255}, std::array<std::uint8_t, 4UZ>{0, 255, 0, 255});
    };

    "ScalingConvert<float, int16_t> rounds and saturates the product"_test = [] {
        using Limits = std::numeric_limits<std::int16_t>;
        ScalingConvert<float, std::int16_t> converter;
        converter.scale = 32767;
        // 0.5 * 32767 is 16383.5, a tie that rounds to the even 16384
        expectBothPaths(converter, std::array{0.5f, 1.f, -1.f, 1.5f, -1.5f, kNaN, kInf}, std::array<std::int16_t, 7UZ>{16384, Limits::max(), -Limits::max(), Limits::max(), Limits::min(), 0, Limits::max()});
    };

    "ScalingConvert<float, int32_t> forms the product in float"_test = [] {
        ScalingConvert<float, std::int32_t> converter;
        converter.scale = 3;
        // 16777215 x 3 is not a float; the product rounds to the nearest float before it converts
        constexpr std::array<float, 2UZ> kInputs{16777215.f, -16777215.f};
        std::array<std::int32_t, 2UZ>    expected{};
        for (std::size_t i = 0UZ; i < kInputs.size(); ++i) {
            expected[i] = unittest::nearestInRange<std::int32_t>(static_cast<double>(kInputs[i] * 3.f));
        }
        expect(neq(expected[0], 3 * 16777215));
        expectBothPaths(converter, kInputs, expected);
    };

    "ScalingConvert with an integer input forms the product without overflow"_test = [] {
        ScalingConvert<std::int16_t, std::int32_t> toInt32;
        toInt32.scale = 4;
        constexpr std::array<std::int16_t, 4UZ> kShort{16384, 32767, -32768, -1};
        std::array<std::int32_t, 4UZ>           scaled{};
        for (std::size_t i = 0UZ; i < kShort.size(); ++i) {
            scaled[i] = 4 * std::int32_t{kShort[i]};
        }
        expectBothPaths(toInt32, kShort, scaled);

        ScalingConvert<std::int32_t, float> toFloat;
        toFloat.scale = 65536;
        expectBothPaths(toFloat, std::array<std::int32_t, 3UZ>{1, 32768, -65536}, std::array{65536.f, 2147483648.f, -4294967296.f});
    };

    "ScalingConvert<int16_t, int32_t> gives the same output in a graph as in processOne"_test = [] {
        using TBlock = ScalingConvert<std::int16_t, std::int32_t>;
        constexpr std::array<std::int16_t, 4UZ> kShort{16384, 32767, -32768, -1};
        std::vector<std::int16_t>               inputs;
        // 71 samples run the full SIMD width and the tail widths 4, 2 and 1
        for (std::size_t i = 0UZ; i < 71UZ; ++i) {
            inputs.push_back(kShort[i % kShort.size()]);
        }
        const std::vector<std::int32_t> received = unittest::runGraph<TBlock, std::int16_t, std::int32_t>(inputs, {{"scale", 4.0}});
        TBlock                          reference({{"scale", 4.0}});
        reference.settings().init();
        std::ignore = reference.settings().applyStagedParameters();
        std::vector<std::int32_t> expected;
        for (const std::int16_t value : inputs) {
            expected.push_back(reference.processOne(value));
        }
        expect(eq(expected[0], 65536)) << "16384 * 4 in processOne";
        expect(received == expected) << "the graph's output equals processOne's";
    };

    "ScalingConvert takes a fractional double scale for an integer input"_test = [] {
        using TBlock = ScalingConvert<std::int16_t, float>;
        expect(std::is_same_v<decltype(TBlock::scale), double>);
        TBlock converter({{"scale", 1.0 / 32768.0}});
        converter.settings().init();
        std::ignore = converter.settings().applyStagedParameters();
        expectBothPaths(converter, std::array<std::int16_t, 3UZ>{-32768, 16384, 1}, std::array{-1.f, 0.5f, 1.f / 32768.f});
    };

    "ComplexToInterleaved to an integer type rounds and saturates each component"_test = [] {
        ComplexToInterleaved<std::complex<float>, std::int16_t> converter;
        const std::vector<std::complex<float>>                  input{{40000.f, kNaN}, {2.5f, -40000.f}};
        std::vector<std::int16_t>                               output(2UZ * input.size());
        expect(converter.processBulk(input, output) == gr::work::Status::OK);
        expect(output == std::vector<std::int16_t>{std::numeric_limits<std::int16_t>::max(), 0, 2, std::numeric_limits<std::int16_t>::min()});
    };
};

int main() { /* not needed for UT */ }
