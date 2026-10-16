#ifndef GNURADIO_MATH_UNARY_HPP
#define GNURADIO_MATH_UNARY_HPP

#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <type_traits>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/meta/utils.hpp>

namespace gr::blocks::math {

namespace unary {

/// converts `value` to `R`
template<typename R, typename U>
[[nodiscard]] constexpr R narrow(const U& value) noexcept {
    if constexpr (std::same_as<R, U>) {
        return value;
    } else {
        return static_cast<R>(value);
    }
}

/*
 * The tag keys a `Unary` block drops. A block with a real output forwards every key. A block with a complex output
 * drops `phase_est`, as `Negate` and a complex `scale` rotate the carrier. An operation that changes the carrier
 * frequency f sets `kChangesFrequency`, and its complex blocks also drop `freq_est`: `Conjugate` and `Reciprocal`
 * give -f, `Square` gives 2f and `Power` gives nf.
 */
template<typename op>
inline constexpr bool kChangesFrequency = requires { requires op::kChangesFrequency; };

template<typename op, typename R>
using DroppedKeys = std::conditional_t<!meta::complex_like<R>, DroppedTagKeys<>, std::conditional_t<kChangesFrequency<op>, DroppedTagKeys<"freq_est", "phase_est">, DroppedTagKeys<"phase_est">>>;

/// the ports of a `Unary` block: `in` of the input type and `out` of the result type
template<typename Derived, typename T, typename R, typename TDropped = DroppedTagKeys<>>
struct Ports : Block<Derived, TDropped> {
    using Block<Derived, TDropped>::Block;

    PortIn<T>  in;
    PortOut<R> out;

    GR_MAKE_REFLECTABLE(Ports, in, out);
};

/// the ports of `Power` and its integer exponent `n`
template<typename Derived, typename T, typename R, typename TDropped = DroppedTagKeys<>>
struct PowerPorts : Block<Derived, TDropped> {
    using Block<Derived, TDropped>::Block;

    PortIn<T>                                             in;
    PortOut<R>                                            out;
    Annotated<std::int32_t, "n", Doc<"integer exponent">> n = 2;

    GR_MAKE_REFLECTABLE(PowerPorts, in, out, n);
};

template<typename op, typename Derived, typename T, typename R>
struct ports_of {
    using type = Ports<Derived, T, R, DroppedKeys<op, R>>;
};

template<typename op, typename Derived, typename T, typename R>
requires requires { typename op::template ports<Derived, T, R>; }
struct ports_of<op, Derived, T, R> {
    using type = typename op::template ports<Derived, T, R>;
};

/*
 * An operation is a type with:
 * - `template<typename T> using result`, the output type for the input type `T`;
 * - `kSimd`, true when `apply` takes a `vir::stdx::simd` of an arithmetic input type;
 * - `static apply(x, block)`, the function of one sample or of one simd vector;
 * - `Description`, the documentation of the blocks it makes;
 * - optionally `kChangesFrequency`, true when the operation changes the frequency of a complex carrier;
 * - optionally `template<typename Derived, typename T, typename R> using ports`, the ports and the
 *   settings of its blocks, `Ports` when absent. A `ports` type passes `DroppedKeys<op, R>` to `Block`.
 */

struct Negate {
    using Description = Doc<"@brief Negates each sample.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = true;
    template<typename V, typename B>
    [[nodiscard]] static constexpr V apply(const V& x, const B& /*block*/) noexcept {
        return narrow<V>(-x);
    }
};

struct Abs {
    using Description = Doc<"@brief Outputs the absolute value of each sample.">;
    template<typename T>
    using result                = meta::fundamental_base_value_type_t<T>;
    static constexpr bool kSimd = true;
    template<typename V, typename B>
    [[nodiscard]] static auto apply(const V& x, const B& /*block*/) noexcept {
        if constexpr (meta::any_simd<V>) {
            if constexpr (std::is_unsigned_v<typename V::value_type>) {
                return x;
            } else {
                return vir::stdx::abs(x);
            }
        } else if constexpr (std::is_unsigned_v<V>) {
            return x;
        } else {
            return narrow<meta::fundamental_base_value_type_t<V>>(std::abs(x));
        }
    }
};

struct Conjugate {
    using Description = Doc<"@brief Outputs the complex conjugate of each sample.">;
    template<typename T>
    using result                            = T;
    static constexpr bool kSimd             = false;
    static constexpr bool kChangesFrequency = true;
    template<typename V, typename B>
    [[nodiscard]] static constexpr V apply(const V& x, const B& /*block*/) noexcept {
        return std::conj(x);
    }
};

struct Square {
    using Description = Doc<"@brief Outputs the square of each sample.">;
    template<typename T>
    using result                            = T;
    static constexpr bool kSimd             = true;
    static constexpr bool kChangesFrequency = true;
    template<typename V, typename B>
    [[nodiscard]] static constexpr V apply(const V& x, const B& /*block*/) noexcept {
        return x * x;
    }
};

struct Reciprocal {
    using Description = Doc<"@brief Outputs the reciprocal of each sample, and 0 for a zero sample.">;
    template<typename T>
    using result                            = T;
    static constexpr bool kSimd             = true;
    static constexpr bool kChangesFrequency = true;
    template<typename V, typename B>
    [[nodiscard]] static constexpr V apply(const V& x, const B& /*block*/) noexcept {
        if constexpr (meta::any_simd<V>) {
            V reciprocal                 = V(1) / x;
            where(x == V(0), reciprocal) = V(0);
            return reciprocal;
        } else {
            return x == V{} ? V{} : V(1) / x;
        }
    }
};

struct Power {
    using Description = Doc<"@brief Raises each sample to the integer power n. A negative n gives infinity for a real zero sample and (inf, NaN) for a complex one.">;
    template<typename T>
    using result                            = T;
    static constexpr bool kSimd             = true;
    static constexpr bool kChangesFrequency = true;
    template<typename Derived, typename T, typename R>
    using ports = PowerPorts<Derived, T, R, DroppedKeys<Power, R>>;
    template<typename V, typename B>
    [[nodiscard]] static constexpr V apply(const V& x, const B& block) noexcept {
        const std::int32_t n        = block.n.value;
        std::uint32_t      exponent = n < 0 ? 0U - static_cast<std::uint32_t>(n) : static_cast<std::uint32_t>(n);
        V                  base     = x;
        V                  power    = V(1);
        while (exponent != 0U) {
            if ((exponent & 1U) != 0U) {
                power = power * base;
            }
            base = base * base;
            exponent >>= 1U;
        }
        return n < 0 ? V(1) / power : power;
    }
};

struct DegToRad {
    using Description = Doc<"@brief Converts each sample from degrees to radians.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = true;
    template<typename V, typename B>
    [[nodiscard]] static constexpr V apply(const V& x, const B& /*block*/) noexcept {
        using F = meta::fundamental_base_value_type_t<V>;
        return (x / F(180)) * std::numbers::pi_v<F>;
    }
};

struct RadToDeg {
    using Description = Doc<"@brief Converts each sample from radians to degrees.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = true;
    template<typename V, typename B>
    [[nodiscard]] static constexpr V apply(const V& x, const B& /*block*/) noexcept {
        using F = meta::fundamental_base_value_type_t<V>;
        return (x / std::numbers::pi_v<F>)*F(180);
    }
};

struct Sign {
    using Description = Doc<"@brief Outputs -1, 0 or +1 by the sign of each sample. A NaN sample stays NaN.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static constexpr V apply(const V& x, const B& /*block*/) noexcept {
        return x > V(0) ? V(1) : (x < V(0) ? V(-1) : x);
    }
};

struct Sqrt {
    using Description = Doc<"@brief Outputs the square root of each sample, and NaN for a negative sample.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& x, const B& /*block*/) noexcept {
        return std::sqrt(x);
    }
};

struct Exp {
    using Description = Doc<"@brief Outputs e raised to each sample.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& x, const B& /*block*/) noexcept {
        return std::exp(x);
    }
};

struct Log {
    using Description = Doc<"@brief Outputs the natural logarithm of each sample.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& x, const B& /*block*/) noexcept {
        return std::log(x);
    }
};

struct Log10 {
    using Description = Doc<"@brief Outputs the base-10 logarithm of each sample. With scale 10 the output is the power in dB.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& x, const B& /*block*/) noexcept {
        return std::log10(x);
    }
};

struct Log2 {
    using Description = Doc<"@brief Outputs the base-2 logarithm of each sample.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& x, const B& /*block*/) noexcept {
        return std::log2(x);
    }
};

struct Sin {
    using Description = Doc<"@brief Outputs the sine of each sample, in radians.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& x, const B& /*block*/) noexcept {
        return std::sin(x);
    }
};

struct Cos {
    using Description = Doc<"@brief Outputs the cosine of each sample, in radians.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& x, const B& /*block*/) noexcept {
        return std::cos(x);
    }
};

struct Tan {
    using Description = Doc<"@brief Outputs the tangent of each sample, in radians.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& x, const B& /*block*/) noexcept {
        return std::tan(x);
    }
};

struct Atan {
    using Description = Doc<"@brief Outputs the arc tangent of each sample, in radians.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& x, const B& /*block*/) noexcept {
        return std::atan(x);
    }
};

struct Sinh {
    using Description = Doc<"@brief Outputs the hyperbolic sine of each sample.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& x, const B& /*block*/) noexcept {
        return std::sinh(x);
    }
};

struct Cosh {
    using Description = Doc<"@brief Outputs the hyperbolic cosine of each sample.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& x, const B& /*block*/) noexcept {
        return std::cosh(x);
    }
};

struct Tanh {
    using Description = Doc<"@brief Outputs the hyperbolic tangent of each sample.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& x, const B& /*block*/) noexcept {
        return std::tanh(x);
    }
};

struct Floor {
    using Description = Doc<"@brief Rounds each sample down to an integer value.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& x, const B& /*block*/) noexcept {
        return std::floor(x);
    }
};

struct Ceil {
    using Description = Doc<"@brief Rounds each sample up to an integer value.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& x, const B& /*block*/) noexcept {
        return std::ceil(x);
    }
};

struct Round {
    using Description = Doc<"@brief Rounds each sample to the nearest integer value, halves away from zero.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& x, const B& /*block*/) noexcept {
        return std::round(x);
    }
};

/// the magnitude of a complex sample; the absolute value of a real one
struct Magnitude : Abs {
    using Description = Doc<"@brief Outputs the magnitude of each complex sample.">;
};

/// re*re + im*im of a complex sample; x*x of a real one
struct MagnitudeSquared {
    using Description = Doc<"@brief Outputs the squared magnitude of each complex sample.">;
    template<typename T>
    using result                = meta::fundamental_base_value_type_t<T>;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static constexpr meta::fundamental_base_value_type_t<V> apply(const V& x, const B& /*block*/) noexcept {
        if constexpr (meta::complex_like<V>) {
            return x.real() * x.real() + x.imag() * x.imag();
        } else {
            return x * x;
        }
    }
};

/// the argument of a complex sample in radians, in [-pi, pi]; 0 or pi for a real one
struct Phase {
    using Description = Doc<"@brief Outputs the phase of each complex sample, in radians.">;
    template<typename T>
    using result                = meta::fundamental_base_value_type_t<T>;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static meta::fundamental_base_value_type_t<V> apply(const V& x, const B& /*block*/) noexcept {
        return narrow<meta::fundamental_base_value_type_t<V>>(std::arg(x));
    }
};

struct Real {
    using Description = Doc<"@brief Outputs the real part of each complex sample.">;
    template<typename T>
    using result                = meta::fundamental_base_value_type_t<T>;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static constexpr meta::fundamental_base_value_type_t<V> apply(const V& x, const B& /*block*/) noexcept {
        return narrow<meta::fundamental_base_value_type_t<V>>(std::real(x));
    }
};

struct Imag {
    using Description = Doc<"@brief Outputs the imaginary part of each complex sample.">;
    template<typename T>
    using result                = meta::fundamental_base_value_type_t<T>;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static constexpr meta::fundamental_base_value_type_t<V> apply(const V& x, const B& /*block*/) noexcept {
        return narrow<meta::fundamental_base_value_type_t<V>>(std::imag(x));
    }
};

} // namespace unary

/**
 * @brief One input, one output: `out = op(in) * scale + offset`, sample by sample.
 *
 * `in` has the type `T`. `out` has the result type of `op`, which is `T` or the real type of a complex `T`. The
 * block forwards every tag key, less the keys of `unary::DroppedKeys<op, R>`.
 *
 * `scale` and `offset` have the output type. At 1 and 0 the output is `op(in)` exactly. Integer results wrap in the
 * type: the lowest value of a signed type negates to itself.
 */
template<typename op, typename T>
struct Unary : unary::ports_of<op, Unary<op, T>, T, typename op::template result<T>>::type {
    using R           = typename op::template result<T>;
    using Description = typename op::Description;

    Annotated<R, "scale", Doc<"factor applied to the result">>      scale  = R(1);
    Annotated<R, "offset", Doc<"value added to the scaled result">> offset = R{};

    GR_MAKE_REFLECTABLE(Unary, scale, offset);

    static_assert(std::same_as<typename Unary::DroppedTagKeysControl, unary::DroppedKeys<op, R>>, "the ports of an operation pass unary::DroppedKeys<op, R> to Block");

    static constexpr bool kSimdPath = op::kSimd && std::is_arithmetic_v<T> && std::same_as<T, R>;

    bool _affine = false; // scale or offset differs from the identity

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { _affine = scale.value != R(1) || offset.value != R{}; }

    template<gr::meta::t_or_simd<T> V>
    requires(kSimdPath)
    [[nodiscard]] constexpr V processOne(const V& x) const noexcept {
        const V y = op::apply(x, *this);
        return _affine ? unary::narrow<V>(y * scale.value + offset.value) : y;
    }

    [[nodiscard]] constexpr R processOne(const T& x) const noexcept
    requires(!kSimdPath)
    {
        const R y = op::apply(x, *this);
        if (!_affine) {
            return y;
        }
        if constexpr (meta::complex_like<R>) {
            // the product by components, without the library's NaN and infinity recovery
            const R& s = scale.value;
            return R(y.real() * s.real() - y.imag() * s.imag(), y.real() * s.imag() + y.imag() * s.real()) + offset.value;
        } else {
            return unary::narrow<R>(y * scale.value + offset.value);
        }
    }
};

GR_REGISTER_BLOCK("gr::blocks::math::Negate", gr::blocks::math::Negate, [T], [ int16_t, int32_t, float, double, std::complex<float> ])
template<typename T>
using Negate = Unary<unary::Negate, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Abs", gr::blocks::math::Abs, [T], [ uint8_t, int16_t, int32_t, float, double ])
template<typename T>
using Abs = Unary<unary::Abs, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Conjugate", gr::blocks::math::Conjugate, [T], [std::complex<float>])
template<typename T>
using Conjugate = Unary<unary::Conjugate, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Square", gr::blocks::math::Square, [T], [ float, double, std::complex<float> ])
template<typename T>
using Square = Unary<unary::Square, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Sqrt", gr::blocks::math::Sqrt, [T], [ float, double ])
template<typename T>
using Sqrt = Unary<unary::Sqrt, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Reciprocal", gr::blocks::math::Reciprocal, [T], [ float, double, std::complex<float> ])
template<typename T>
using Reciprocal = Unary<unary::Reciprocal, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Exp", gr::blocks::math::Exp, [T], [ float, double ])
template<typename T>
using Exp = Unary<unary::Exp, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Log", gr::blocks::math::Log, [T], [ float, double ])
template<typename T>
using Log = Unary<unary::Log, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Log10", gr::blocks::math::Log10, [T], [ float, double ])
template<typename T>
using Log10 = Unary<unary::Log10, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Log2", gr::blocks::math::Log2, [T], [ float, double ])
template<typename T>
using Log2 = Unary<unary::Log2, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Sin", gr::blocks::math::Sin, [T], [ float, double ])
template<typename T>
using Sin = Unary<unary::Sin, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Cos", gr::blocks::math::Cos, [T], [ float, double ])
template<typename T>
using Cos = Unary<unary::Cos, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Tan", gr::blocks::math::Tan, [T], [ float, double ])
template<typename T>
using Tan = Unary<unary::Tan, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Atan", gr::blocks::math::Atan, [T], [ float, double ])
template<typename T>
using Atan = Unary<unary::Atan, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Sinh", gr::blocks::math::Sinh, [T], [ float, double ])
template<typename T>
using Sinh = Unary<unary::Sinh, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Cosh", gr::blocks::math::Cosh, [T], [ float, double ])
template<typename T>
using Cosh = Unary<unary::Cosh, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Tanh", gr::blocks::math::Tanh, [T], [ float, double ])
template<typename T>
using Tanh = Unary<unary::Tanh, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Power", gr::blocks::math::Power, [T], [ float, double, std::complex<float> ])
template<typename T>
using Power = Unary<unary::Power, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Floor", gr::blocks::math::Floor, [T], [ float, double ])
template<typename T>
using Floor = Unary<unary::Floor, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Ceil", gr::blocks::math::Ceil, [T], [ float, double ])
template<typename T>
using Ceil = Unary<unary::Ceil, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Round", gr::blocks::math::Round, [T], [ float, double ])
template<typename T>
using Round = Unary<unary::Round, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Sign", gr::blocks::math::Sign, [T], [ float, double ])
template<typename T>
using Sign = Unary<unary::Sign, T>;

GR_REGISTER_BLOCK("gr::blocks::math::DegToRad", gr::blocks::math::DegToRad, [T], [ float, double ])
template<typename T>
using DegToRad = Unary<unary::DegToRad, T>;

GR_REGISTER_BLOCK("gr::blocks::math::RadToDeg", gr::blocks::math::RadToDeg, [T], [ float, double ])
template<typename T>
using RadToDeg = Unary<unary::RadToDeg, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Magnitude", gr::blocks::math::Magnitude, [T], [std::complex<float>])
template<typename T>
using Magnitude = Unary<unary::Magnitude, T>;

GR_REGISTER_BLOCK("gr::blocks::math::MagnitudeSquared", gr::blocks::math::MagnitudeSquared, [T], [std::complex<float>])
template<typename T>
using MagnitudeSquared = Unary<unary::MagnitudeSquared, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Phase", gr::blocks::math::Phase, [T], [std::complex<float>])
template<typename T>
using Phase = Unary<unary::Phase, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Real", gr::blocks::math::Real, [T], [std::complex<float>])
template<typename T>
using Real = Unary<unary::Real, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Imag", gr::blocks::math::Imag, [T], [std::complex<float>])
template<typename T>
using Imag = Unary<unary::Imag, T>;

} // namespace gr::blocks::math

#endif // GNURADIO_MATH_UNARY_HPP
