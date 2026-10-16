#ifndef CONVERTERBLOCKS_HPP
#define CONVERTERBLOCKS_HPP

#include <cstring>
#include <tuple>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/math/Unary.hpp>
#include <gnuradio-4.0/meta/UncertainValue.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::Convert, ([T], [U]), [ uint8_t, int16_t, int32_t, float ], [ uint8_t, int16_t, int32_t, float ])

template<typename T, typename R>
requires std::is_arithmetic_v<T> && std::is_arithmetic_v<R>
struct Convert : Block<Convert<T, R>> {
    using Description = Doc<"(@brief basic block to perform a input to output data type conversion (N.B. w/o scaling))">;
    PortIn<T>  in;
    PortOut<R> out;

    GR_MAKE_REFLECTABLE(Convert, in, out);

    template<gr::meta::t_or_simd<T> V>
    [[nodiscard]] constexpr auto processOne(const V& input) const noexcept {
        if constexpr (gr::meta::any_simd<V>) { // simd case
            using RetType = vir::stdx::rebind_simd_t<R, V>;
            return vir::stdx::static_simd_cast<RetType>(input);
        } else { // non-simd case
            return static_cast<R>(input);
        }
    }
};

GR_REGISTER_BLOCK(gr::blocks::basic::ScalingConvert, ([T], [U]), [ uint8_t, int16_t, int32_t, float ], [ uint8_t, int16_t, int32_t, float ])

template<typename T, typename R>
requires std::is_arithmetic_v<T> && std::is_arithmetic_v<R>
struct ScalingConvert : Block<ScalingConvert<T, R>> {
    using Description = Doc<R""(@brief basic block to perform a input to output data type conversion

Performs scaling, i.e. 'R output = R(input * scale)'
)"">;
    PortIn<T>  in;
    PortOut<R> out;
    T          scale = static_cast<T>(1);

    GR_MAKE_REFLECTABLE(ScalingConvert, in, out, scale);

    template<gr::meta::t_or_simd<T> V>
    [[nodiscard]] constexpr auto processOne(const V& input) const noexcept {
        if constexpr (gr::meta::any_simd<V>) { // simd case
            using RetType = vir::stdx::rebind_simd_t<R, V>;
            return vir::stdx::static_simd_cast<RetType>(input * scale);
        } else { // non-simd case
            return static_cast<R>(input * scale);
        }
    }
};

namespace detail {

/*
 * The converters below are `math::Unary` blocks whose operations name the converters' ports:
 * `in` and `abs`, `in` and `imag`, `in` and `real`, `in` and `arg`, `rad` and `deg`, `deg` and `rad`.
 */

template<typename Derived, typename T, typename R>
struct AbsPorts : Block<Derived> {
    using Block<Derived>::Block;

    PortIn<T>  in;
    PortOut<R> abs;

    GR_MAKE_REFLECTABLE(AbsPorts, in, abs);
};

template<typename Derived, typename T, typename R>
struct ImagPorts : Block<Derived> {
    using Block<Derived>::Block;

    PortIn<T>  in;
    PortOut<R> imag;

    GR_MAKE_REFLECTABLE(ImagPorts, in, imag);
};

template<typename Derived, typename T, typename R>
struct RealPorts : Block<Derived> {
    using Block<Derived>::Block;

    PortIn<T>  in;
    PortOut<R> real;

    GR_MAKE_REFLECTABLE(RealPorts, in, real);
};

template<typename Derived, typename T, typename R>
struct ArgPorts : Block<Derived> {
    using Block<Derived>::Block;

    PortIn<T>  in;
    PortOut<R> arg;

    GR_MAKE_REFLECTABLE(ArgPorts, in, arg);
};

template<typename Derived, typename T, typename R>
struct RadiansToDegreePorts : Block<Derived> {
    using Block<Derived>::Block;

    PortIn<T>  rad;
    PortOut<R> deg;

    GR_MAKE_REFLECTABLE(RadiansToDegreePorts, rad, deg);
};

template<typename Derived, typename T, typename R>
struct DegreeToRadiansPorts : Block<Derived> {
    using Block<Derived>::Block;

    PortIn<T>  deg;
    PortOut<R> rad;

    GR_MAKE_REFLECTABLE(DegreeToRadiansPorts, deg, rad);
};

struct AbsOp : math::unary::Abs {
    using Description = Doc<"@brief basic block to calculate the absolute (magnitude) value of a complex or arithmetic input stream">;
    template<typename Derived, typename T, typename R>
    using ports = AbsPorts<Derived, T, R>;
};

struct ImagOp : math::unary::Imag {
    using Description = Doc<"@brief basic block to the imaginary component of a complex input stream">;
    template<typename Derived, typename T, typename R>
    using ports = ImagPorts<Derived, T, R>;
};

struct RealOp : math::unary::Real {
    using Description = Doc<"@brief basic block to the real component of a complex input stream">;
    template<typename Derived, typename T, typename R>
    using ports = RealPorts<Derived, T, R>;
};

struct ArgOp : math::unary::Phase {
    using Description = Doc<"@brief basic block to calculate the argument (phase angle, [radians]) of a complex input stream">;
    template<typename Derived, typename T, typename R>
    using ports = ArgPorts<Derived, T, R>;
};

struct RadiansToDegreeOp : math::unary::RadToDeg {
    using Description = Doc<"@brief convert radians to degree">;
    template<typename Derived, typename T, typename R>
    using ports = RadiansToDegreePorts<Derived, T, R>;
};

struct DegreeToRadiansOp : math::unary::DegToRad {
    using Description = Doc<"@brief convert degree to radians">;
    template<typename Derived, typename T, typename R>
    using ports = DegreeToRadiansPorts<Derived, T, R>;
};

} // namespace detail

GR_REGISTER_BLOCK("gr::blocks::basic::Abs", gr::blocks::basic::Abs, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])
template<typename T>
requires std::is_arithmetic_v<T> || meta::complex_like<T>
using Abs = math::Unary<detail::AbsOp, T>;

GR_REGISTER_BLOCK("gr::blocks::basic::Imag", gr::blocks::basic::Imag, [T], [std::complex<float>])
template<typename T>
requires std::is_arithmetic_v<T> || meta::complex_like<T>
using Imag = math::Unary<detail::ImagOp, T>;

GR_REGISTER_BLOCK("gr::blocks::basic::Real", gr::blocks::basic::Real, [T], [std::complex<float>])
template<typename T>
requires std::is_arithmetic_v<T> || meta::complex_like<T>
using Real = math::Unary<detail::RealOp, T>;

GR_REGISTER_BLOCK("gr::blocks::basic::Arg", gr::blocks::basic::Arg, [T], [std::complex<float>])
template<typename T>
requires std::is_arithmetic_v<T> || meta::complex_like<T>
using Arg = math::Unary<detail::ArgOp, T>;

GR_REGISTER_BLOCK("gr::blocks::basic::RadiansToDegree", gr::blocks::basic::RadiansToDegree, [T], [float])
template<std::floating_point T>
using RadiansToDegree = math::Unary<detail::RadiansToDegreeOp, T>;

GR_REGISTER_BLOCK("gr::blocks::basic::DegreeToRadians", gr::blocks::basic::DegreeToRadians, [T], [float])
template<std::floating_point T>
using DegreeToRadians = math::Unary<detail::DegreeToRadiansOp, T>;

GR_REGISTER_BLOCK(gr::blocks::basic::ToRealImag, [T], [std::complex<float>])

template<typename T>
requires std::is_arithmetic_v<T> || meta::complex_like<T>
struct ToRealImag : Block<ToRealImag<T>> {
    using R           = meta::fundamental_base_value_type_t<T>;
    using Description = Doc<"(@brief decompose complex (or arithmetic) numbers their real and imaginary component">;
    PortIn<T>  in;
    PortOut<R> real;
    PortOut<R> imag;

    GR_MAKE_REFLECTABLE(ToRealImag, in, real, imag);

    [[nodiscard]] constexpr std::tuple<R, R> processOne(T complexIn) const noexcept { // some SIMD-fication potential here
        return {static_cast<R>(std::real(complexIn)), static_cast<R>(std::imag(complexIn))};
    }
};

GR_REGISTER_BLOCK(gr::blocks::basic::RealImagToComplex, [T], [float])

template<std::floating_point T>
struct RealImagToComplex : Block<RealImagToComplex<T>> {
    using R           = std::complex<T>;
    using Description = Doc<"(@brief compose complex (or arithmetic) numbers from their real and imaginary component">;
    PortIn<T>  real;
    PortIn<T>  imag;
    PortOut<R> out;

    GR_MAKE_REFLECTABLE(RealImagToComplex, real, imag, out);

    [[nodiscard]] constexpr R processOne(T re, T im) const noexcept { // some SIMD-fication potential here
        return {re, im};
    }
};

GR_REGISTER_BLOCK(gr::blocks::basic::ToMagPhase, [T], [std::complex<float>])

template<typename T>
requires std::is_arithmetic_v<T> || meta::complex_like<T>
struct ToMagPhase : Block<ToMagPhase<T>> {
    using R           = meta::fundamental_base_value_type_t<T>;
    using Description = Doc<"(@brief decompose complex (or arithmetic) numbers their magnitude (abs) and phase (arg, [rad]) component">;
    PortIn<T>  in;
    PortOut<R> mag;
    PortOut<R> phase;

    GR_MAKE_REFLECTABLE(ToMagPhase, in, mag, phase);

    [[nodiscard]] constexpr std::tuple<R, R> processOne(T complexIn) const noexcept { // some SIMD-fication potential here
        return {static_cast<R>(std::abs(complexIn)), static_cast<R>(std::arg(complexIn))};
    }
};

GR_REGISTER_BLOCK(gr::blocks::basic::MagPhaseToComplex, [T], [float])

template<typename T>
requires std::is_arithmetic_v<T>
struct MagPhaseToComplex : public gr::Block<MagPhaseToComplex<T>> {
    using R           = std::complex<T>;
    using Description = Doc<"(@brief compose complex (or arithmetic) numbers from their abs and phase ([rad]) components">;
    PortIn<T>  mag;
    PortIn<T>  phase;
    PortOut<R> out;

    GR_MAKE_REFLECTABLE(MagPhaseToComplex, mag, phase, out);

    [[nodiscard]] constexpr std::complex<T> processOne(T r, T theta) const noexcept { // some SIMD-fication potential here
        return std::polar(r, theta);
    }
};

GR_REGISTER_BLOCK(gr::blocks::basic::ComplexToInterleaved, ([T], [U]), [std::complex<float>], [float])

template<typename T, typename R>
requires meta::complex_like<T> && (std::floating_point<R> || std::is_same_v<R, std::int8_t> || std::is_same_v<R, std::int16_t>)
struct ComplexToInterleaved : Block<ComplexToInterleaved<T, R>, BackwardTagPropagation, Resampling<1UZ, 2UZ, true>> {
    using Description = Doc<R""(@brief convert stream of complex to a stream of interleaved specified type.

The output stream contains twice as many output items as input items.
For every complex input item, we produce two output items that alternate between the real and imaginary component of the complex value.)"">;
    PortIn<T>  in;
    PortOut<R> interleaved;

    GR_MAKE_REFLECTABLE(ComplexToInterleaved, in, interleaved);

    [[nodiscard]] constexpr work::Status processBulk(std::span<const T> complexInput, std::span<R> interleavedOut) const noexcept { // some SIMD-fication potential here (needs permute)
        if constexpr (std::is_same_v<R, typename T::value_type>) {
            static_assert(sizeof(T) == 2UZ * sizeof(R) && alignof(T) >= alignof(R), "std::complex<R> is not layout-compatible with R[2]");
            if (!complexInput.empty()) {
                std::memcpy(static_cast<void*>(interleavedOut.data()), static_cast<const void*>(complexInput.data()), complexInput.size() * sizeof(T));
            }
        } else {
            for (std::size_t i = 0; i < complexInput.size(); ++i) {
                interleavedOut[2 * i]     = static_cast<R>(complexInput[i].real());
                interleavedOut[2 * i + 1] = static_cast<R>(complexInput[i].imag());
            }
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::basic::InterleavedToComplex, ([T], [U]), [float], [std::complex<float>])

template<typename T, typename R>
requires(std::floating_point<T> || std::is_same_v<T, std::int8_t> || std::is_same_v<T, std::int16_t>) && meta::complex_like<R>
struct InterleavedToComplex : public gr::Block<InterleavedToComplex<T, R>, gr::BackwardTagPropagation, gr::Resampling<2UZ, 1UZ, true>> {
    using Description = Doc<R""(@brief convert stream of interleaved values to a stream of complex numbers.

The input stream contains twice as many input items as output items.
For every pair of interleaved input items (real, imag), we produce one complex output item.
)"">;
    gr::PortIn<T>  interleaved;
    gr::PortOut<R> out;

    GR_MAKE_REFLECTABLE(InterleavedToComplex, interleaved, out);

    [[nodiscard]] constexpr work::Status processBulk(std::span<const T> interleavedInput, std::span<R> complexOut) const noexcept { // some SIMD-fication potential here (needs permute)
        if constexpr (std::is_same_v<T, typename R::value_type>) {
            static_assert(sizeof(R) == 2UZ * sizeof(T) && alignof(R) >= alignof(T), "std::complex<T> is not layout-compatible with T[2]");
            if (!complexOut.empty()) {
                std::memcpy(static_cast<void*>(complexOut.data()), static_cast<const void*>(interleavedInput.data()), complexOut.size() * sizeof(R));
            }
        } else {
            for (std::size_t i = 0; i < complexOut.size(); ++i) {
                complexOut[i] = R{static_cast<typename R::value_type>(interleavedInput[2 * i]), static_cast<typename R::value_type>(interleavedInput[2 * i + 1])};
            }
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::basic

#endif // CONVERTERBLOCKS_HPP
