#ifndef CONVERTERBLOCKS_HPP
#define CONVERTERBLOCKS_HPP

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <tuple>
#include <type_traits>
#include <utility>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/math/Unary.hpp>
#include <gnuradio-4.0/meta/UncertainValue.hpp>

#include <vir/simd_bit.h>

namespace gr::blocks::basic {

namespace detail {

/// The floating-point type `saturatingCast` rounds a value of `T` in toward the integer type `R`: `T` when `T` is
/// `float` or `double` and every value of `R` is at most 2^(digits - 2) in magnitude, otherwise `double` when that
/// holds for `double`, and `void` when it holds for neither. With `void` each lane takes the comparing scalar code.
/// A value of another floating-point type is converted to `double` first, so a 64-bit output is the only one that
/// rounds it exactly.
/// `float` to `std::int32_t` takes its own steps in `float` and does not use this type.
template<typename T, typename R>
using RoundingType = std::conditional_t<((std::same_as<T, float> || std::same_as<T, double>) && std::numeric_limits<R>::digits + 2 <= std::numeric_limits<T>::digits), T, std::conditional_t<(std::numeric_limits<R>::digits + 2 <= std::numeric_limits<double>::digits), double, void>>;

/// Selects the steps for `float` to `std::int32_t`. A `float` sum with 1.5 x 2^23 is exact only up to 2^22, and the
/// range of `std::int32_t` reaches 2^31.
template<typename T, typename R>
inline constexpr bool kRoundsFloatToInt32 = std::same_as<T, float> && std::same_as<R, std::int32_t>;

/// The constants of the rounding in the floating-point type `W`, with bit patterns in `Int`, the signed integer type of
/// its size.
///
/// The bits of a NaN without its sign are `kFirstNaN` or more: an exponent of all ones and a mantissa other than 0.
/// `kMagic` is 1.5 x 2^(digits - 1).
/// A value at most 2^(digits - 2) in magnitude plus `kMagic` lies where the spacing of `W` is 1. The sum is the value
/// rounded to an integer, and its bits less `kMagicBits` are that integer.
/// `kExact` is 2^(digits - 1), and `kExactBits` are its bits. From there on every value of `W` is an integer.
template<std::floating_point W>
struct RoundingBits {
    using Int                       = std::conditional_t<sizeof(W) == sizeof(std::int32_t), std::int32_t, std::int64_t>;
    static constexpr Int kSign      = std::numeric_limits<Int>::min();
    static constexpr Int kMagnitude = std::numeric_limits<Int>::max();
    static constexpr Int kFirstNaN  = (((Int{1} << (std::numeric_limits<Int>::digits + 1 - std::numeric_limits<W>::digits)) - 1) << (std::numeric_limits<W>::digits - 1)) + 1;
    static constexpr W   kMagic     = static_cast<W>(std::uint64_t{3} << (std::numeric_limits<W>::digits - 2));
    static constexpr Int kMagicBits = std::bit_cast<Int>(kMagic);
    static constexpr W   kExact     = static_cast<W>(std::uint64_t{1} << (std::numeric_limits<W>::digits - 1));
    static constexpr Int kExactBits = std::bit_cast<Int>(kExact);
};

/// True when `value` is a NaN. A `float` or `double` is read from its bits, since `std::isnan` is false under
/// `-ffast-math`.
template<std::floating_point T>
[[nodiscard]] constexpr bool isNaN(T value) noexcept {
    if constexpr (std::same_as<T, float> || std::same_as<T, double>) {
        using Bits = RoundingBits<T>;
        return (std::bit_cast<typename Bits::Int>(value) & Bits::kMagnitude) >= Bits::kFirstNaN;
    } else {
        return std::isnan(value);
    }
}

/// The limits of the `float` to `std::int32_t` conversion. A value clamps to `kLowest`, -2^31, and to `kBelowMax`, the
/// largest `float` below 2^31. `kHalfMaxBits` are the bits of 2^31 shifted right by one. The comparison of the shifted
/// bits cannot overflow for a negative value. A value of 2^31 or more takes the highest value of `std::int32_t`,
/// `kBelowMax` with the bits of `kPastMaxBits` set.
struct Int32FromFloat {
    static constexpr float        kLowest      = -2147483648.f;
    static constexpr float        kBelowMax    = 2147483520.f;
    static constexpr std::int32_t kHalfMaxBits = std::bit_cast<std::int32_t>(2147483648.f) >> 1;
    static constexpr std::int32_t kPastMaxBits = std::numeric_limits<std::int32_t>::max() - 2147483520;
};

/// Converts `value` to `R`, saturating when `R` is an integer type.
///
/// A floating-point value rounds to the nearest integer, ties to even, in the default rounding mode, and NaN gives 0.
/// A value below or above the range of `R` gives the lowest or the highest value of `R`. The scalar and the SIMD
/// overloads give the same output for every input. A conversion to a floating-point type is a plain cast.
/// Under `-ffast-math` the shifts of `float` to `std::int32_t` fold away and the result truncates, in `Convert` and
/// `ScalingConvert`. Every other conversion from `float` or `double` keeps its result.
template<typename R, typename T>
requires std::is_arithmetic_v<T> && std::is_arithmetic_v<R>
[[gnu::always_inline, nodiscard]] constexpr R saturatingCast(T value) noexcept {
    using Limits = std::numeric_limits<R>;
    if constexpr (kRoundsFloatToInt32<T, R>) {
        // a NaN clears its bits; below 2^23 in magnitude, adding and subtracting 2^23 with the sign of the value
        // rounds it
        using Bits                 = RoundingBits<float>;
        using L                    = Int32FromFloat;
        const std::int32_t raw     = std::bit_cast<std::int32_t>(value);
        const std::int32_t bits    = raw & (((raw & Bits::kMagnitude) - Bits::kFirstNaN) >> 31);
        const float        c       = std::min(std::max(std::bit_cast<float>(bits), L::kLowest), L::kBelowMax);
        const std::int32_t clamped = std::bit_cast<std::int32_t>(c);
        const std::int32_t below   = ((clamped & Bits::kMagnitude) - Bits::kExactBits) >> 31;
        const float        shift   = std::bit_cast<float>(((clamped & Bits::kSign) | Bits::kExactBits) & below);
        const std::int32_t inside  = ((bits >> 1) - L::kHalfMaxBits) >> 31;
        return static_cast<std::int32_t>((c + shift) - shift) | (~inside & L::kPastMaxBits);
    } else if constexpr (std::integral<R> && std::floating_point<T>) {
        using W = RoundingType<T, R>;
        if constexpr (std::is_void_v<W>) {
            // the lowest value of R and 2^digits are zero or powers of two, exact in T
            constexpr T kLowest  = static_cast<T>(Limits::min());
            constexpr T kPastMax = static_cast<T>(Limits::max() / 2 + 1) * T{2};
            if (isNaN(value)) {
                return R{0};
            }
            const T rounded = std::nearbyint(value);
            if (rounded < kLowest) {
                return Limits::min();
            }
            if (rounded >= kPastMax) {
                return Limits::max();
            }
            return static_cast<R>(rounded);
        } else {
            // a NaN clears its bits, the value clamps to the range of R, and the sum with kMagic rounds it
            using Bits     = RoundingBits<W>;
            using Int      = typename Bits::Int;
            const Int raw  = std::bit_cast<Int>(static_cast<W>(value));
            const Int bits = raw & (((raw & Bits::kMagnitude) - Bits::kFirstNaN) >> std::numeric_limits<Int>::digits);
            const W   c    = std::min(std::max(std::bit_cast<W>(bits), static_cast<W>(Limits::min())), static_cast<W>(Limits::max()));
            return static_cast<R>(std::bit_cast<Int>(c + Bits::kMagic) - Bits::kMagicBits);
        }
    } else if constexpr (std::integral<R> && std::integral<T>) {
        if constexpr (std::cmp_less(std::numeric_limits<T>::min(), Limits::min())) {
            if (std::cmp_less(value, Limits::min())) {
                return Limits::min();
            }
        }
        if constexpr (std::cmp_greater(std::numeric_limits<T>::max(), Limits::max())) {
            if (std::cmp_greater(value, Limits::max())) {
                return Limits::max();
            }
        }
        return static_cast<R>(value);
    } else {
        return static_cast<R>(value);
    }
}

template<typename R, gr::meta::any_simd V>
requires std::is_arithmetic_v<R>
[[gnu::always_inline, nodiscard]] constexpr auto saturatingCast(const V& value) noexcept {
    using T       = typename V::value_type;
    using RetType = vir::stdx::rebind_simd_t<R, V>;
    using Limits  = std::numeric_limits<R>;
    if constexpr (kRoundsFloatToInt32<T, R>) {
        using Bits            = RoundingBits<float>;
        using L               = Int32FromFloat;
        const RetType raw     = vir::simd_bit_cast<RetType>(value);
        const RetType bits    = raw & (((raw & Bits::kMagnitude) - Bits::kFirstNaN) >> 31);
        const V       c       = vir::stdx::min(vir::stdx::max(vir::simd_bit_cast<V>(bits), V(L::kLowest)), V(L::kBelowMax));
        const RetType clamped = vir::simd_bit_cast<RetType>(c);
        const RetType below   = ((clamped & Bits::kMagnitude) - Bits::kExactBits) >> 31;
        const V       shift   = vir::simd_bit_cast<V>(((clamped & Bits::kSign) | Bits::kExactBits) & below);
        const RetType inside  = ((bits >> 1) - L::kHalfMaxBits) >> 31;
        return vir::stdx::static_simd_cast<RetType>((c + shift) - shift) | (~inside & L::kPastMaxBits);
    } else if constexpr (std::integral<R> && std::floating_point<T>) {
        using W = RoundingType<T, R>;
        if constexpr (std::is_void_v<W>) {
            return RetType([&value](auto i) { return saturatingCast<R, T>(value[i]); });
        } else {
            using Bits      = RoundingBits<W>;
            using Int       = typename Bits::Int;
            using WV        = vir::stdx::rebind_simd_t<W, V>;
            using IntV      = vir::stdx::rebind_simd_t<Int, V>;
            const IntV raw  = vir::simd_bit_cast<IntV>(vir::stdx::static_simd_cast<WV>(value));
            const IntV bits = raw & (((raw & Bits::kMagnitude) - Bits::kFirstNaN) >> std::numeric_limits<Int>::digits);
            const WV   c    = vir::stdx::min(vir::stdx::max(vir::simd_bit_cast<WV>(bits), WV(static_cast<W>(Limits::min()))), WV(static_cast<W>(Limits::max())));
            return vir::stdx::static_simd_cast<RetType>(vir::simd_bit_cast<IntV>(c + Bits::kMagic) - Bits::kMagicBits);
        }
    } else if constexpr (std::integral<R> && std::integral<T>) {
        V x = value;
        if constexpr (std::cmp_less(std::numeric_limits<T>::min(), Limits::min())) {
            x = vir::stdx::max(x, V(static_cast<T>(Limits::min())));
        }
        if constexpr (std::cmp_greater(std::numeric_limits<T>::max(), Limits::max())) {
            x = vir::stdx::min(x, V(static_cast<T>(Limits::max())));
        }
        return vir::stdx::static_simd_cast<RetType>(x);
    } else {
        return vir::stdx::static_simd_cast<RetType>(value);
    }
}

/// The type `ScalingConvert` forms its product in: `float` when `T` is `float` or when `float` holds every value of
/// `T` and of `R` exactly, otherwise `double`.
template<typename T, typename R>
using ScalingProductType = std::conditional_t<std::same_as<T, float> || (std::numeric_limits<T>::digits <= std::numeric_limits<float>::digits && std::numeric_limits<R>::digits <= std::numeric_limits<float>::digits), float, double>;

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::basic::Convert, ([T], [U]), [ uint8_t, int16_t, int32_t, float ], [ uint8_t, int16_t, int32_t, float ])

template<typename T, typename R>
requires std::is_arithmetic_v<T> && std::is_arithmetic_v<R>
struct Convert : Block<Convert<T, R>> {
    using Description = Doc<R""(@brief Converts each input sample to the output type, without scaling.

An integer output saturates: an input below or above the output type's range gives its lowest or highest value.
A floating-point input to an integer output rounds to the nearest integer, ties to even, in the default rounding
mode, and NaN gives 0.
`float` to `int32_t` needs the default floating-point semantics: under `-ffast-math` the result truncates.
A floating-point output is the input rounded to the nearest value of the output type.
)"">;
    PortIn<T>  in;
    PortOut<R> out;

    GR_MAKE_REFLECTABLE(Convert, in, out);

    template<gr::meta::t_or_simd<T> V>
    [[nodiscard]] constexpr auto processOne(const V& input) const noexcept {
        return detail::saturatingCast<R>(input);
    }
};

GR_REGISTER_BLOCK(gr::blocks::basic::ScalingConvert, ([T], [U]), [ uint8_t, int16_t, int32_t, float ], [ uint8_t, int16_t, int32_t, float ])

template<typename T, typename R>
requires std::is_arithmetic_v<T> && std::is_arithmetic_v<R>
struct ScalingConvert : Block<ScalingConvert<T, R>> {
    using Description = Doc<R""(@brief Multiplies each input sample by `scale` and converts the product to the output type.

`scale` is a `double` for every pair of types. The product is formed in `float` when the input is `float` or when
`float` holds every input and output value exactly, and in `double` otherwise. In `float` the scale is rounded to
`float` first, and the product is rounded to `float` before it converts.
An integer input cannot overflow the product. An `int64_t` or `uint64_t` input past 2^53 loses its low bits in the
`double` product.
The product converts to the output type as in `Convert`: an integer output rounds to nearest, ties to even (in the
default rounding mode), takes 0 for NaN and saturates at its range.
`float` to `int32_t` needs the default floating-point semantics: under `-ffast-math` the result truncates.
)"">;
    PortIn<T>  in;
    PortOut<R> out;
    double     scale = 1.0;

    GR_MAKE_REFLECTABLE(ScalingConvert, in, out, scale);

    template<gr::meta::t_or_simd<T> V>
    [[nodiscard]] constexpr auto processOne(const V& input) const noexcept {
        using P = detail::ScalingProductType<T, R>;
        if constexpr (gr::meta::any_simd<V>) {
            using PV = vir::stdx::rebind_simd_t<P, V>;
            return detail::saturatingCast<R>(vir::stdx::static_simd_cast<PV>(input) * static_cast<P>(scale));
        } else {
            return detail::saturatingCast<R>(static_cast<P>(input) * static_cast<P>(scale));
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

/// the real types take `Abs`'s steps; `std::complex<float>` takes `Magnitude`'s span loop
struct AbsOp : math::unary::Magnitude {
    using Description = Doc<R""(@brief basic block to calculate the absolute (magnitude) value of a complex or arithmetic input stream

For `std::complex<float>` the block squares and sums the parts in double and rounds the square root to float once. An infinite part gives +inf, also beside a NaN.
)"">;
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
    using Description = Doc<R""(@brief basic block to calculate the argument (phase angle, [radians]) of a complex input stream

For `std::complex<float>` the argument is `math::Phase`'s, `gr::math::scaledAtan2` of the parts: within 2 ulp of the
exact value with g++ and within 4 ulp with clang. Zeros, infinities and NaN give the C library's results: a zero sample
gives 0 with the sign of its imaginary part, or +-pi when its real part is -0.
)"">;
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
} // namespace gr::blocks::basic

namespace gr::blocks::math::unary {
template<>
inline constexpr bool kSpanLoop<basic::detail::AbsOp> = true;
template<>
inline constexpr bool kSpanLoop<basic::detail::ArgOp> = true;
} // namespace gr::blocks::math::unary

namespace gr::blocks::basic {

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
For every complex input item, we produce two output items that alternate between the real and imaginary component of the complex value.
An integer output converts each component as `Convert` does: rounded to nearest, ties to even, in the default rounding mode, NaN as 0, saturated at the type's range.)"">;
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
                interleavedOut[2 * i]     = detail::saturatingCast<R>(complexInput[i].real());
                interleavedOut[2 * i + 1] = detail::saturatingCast<R>(complexInput[i].imag());
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
