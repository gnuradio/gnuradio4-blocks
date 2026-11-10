#ifndef GNURADIO_ALGORITHM_MATH_ATAN2_HPP
#define GNURADIO_ALGORITHM_MATH_ATAN2_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <numbers>
#include <span>

#include <gnuradio-4.0/meta/simd_math.hpp>

namespace gr::math {

/// @brief The implementations of `scaledAtan2`; `atan2Form<T>()` names the one a build takes.
enum class Atan2Form {
    Lanes,      ///< `gr::meta::atan2` on `native_simd<T>` lanes, within 2 ulp
    Branchless, ///< one loop of branch-free arithmetic that the compiler vectorizes, within 4 ulp
    Library     ///< `std::atan2`, the C library's accuracy
};

namespace detail {

/// @brief `out[i] = scale * gr::meta::atan2(y[i], x[i])` on `native_simd<T>` lanes. A lane's value does not depend on its
/// position or on the span's length; the tail runs in one zero-padded vector.
template<std::floating_point T>
void scaledAtan2Lanes(std::span<const T> y, std::span<const T> x, std::span<T> out, T scale) noexcept {
    using V                 = vir::stdx::native_simd<T>;
    constexpr std::size_t W = V::size();
    const std::size_t     n = std::min({y.size(), x.size(), out.size()});
    const V               factor(scale);
    std::size_t           i = 0UZ;
    for (; i + W <= n; i += W) {
        V result = factor * gr::meta::atan2(V(&y[i], vir::stdx::element_aligned), V(&x[i], vir::stdx::element_aligned));
        result.copy_to(&out[i], vir::stdx::element_aligned);
    }
    if (i < n) {
        std::array<T, W> tailY{};
        std::array<T, W> tailX{};
        std::array<T, W> tailOut;
        std::copy_n(&y[i], n - i, tailY.begin());
        std::copy_n(&x[i], n - i, tailX.begin());
        V result = factor * gr::meta::atan2(V(tailY.data(), vir::stdx::element_aligned), V(tailX.data(), vir::stdx::element_aligned));
        result.copy_to(tailOut.data(), vir::stdx::element_aligned);
        std::copy_n(tailOut.begin(), n - i, &out[i]);
    }
}

/// @brief The arctangent of `t` for `0 <= |t| <= tan(pi/8)`: an odd polynomial in float, a rational function in double.
template<std::floating_point T>
[[gnu::always_inline]] inline T atanSmall(T t) noexcept {
    const T z = t * t;
    if constexpr (std::is_same_v<T, float>) {
        return (((8.05374449538e-2f * z - 1.38776856032e-1f) * z + 1.99777106478e-1f) * z - 3.33329491539e-1f) * z * t + t;
    } else {
        const T p = (((-8.750608600031904122785e-1 * z - 1.615753718733365076637e1) * z - 7.500855792314704667340e1) * z - 1.228866684490136173410e2) * z - 6.485021904942025371773e1;
        const T q = ((((z + 2.485846490142306297962e1) * z + 1.650270098316988542046e2) * z + 4.328810604912902668951e2) * z + 4.853903996359136964868e2) * z + 1.945506571482613964425e2;
        return t + t * z * (p / q);
    }
}

/// @brief `atan2(y, x)` from selects of constants and a division only, so that a loop over it vectorizes. With
/// `t = min(|x|, |y|) / max(|x|, |y|)`, the result is `k pi/4 + s atan(u)` for a whole `k` between 0 and 4, a sign `s`
/// and `u = t` or, above `tan(pi/8)`, `u = (mn - mx) / (mx + mn)`. Both `k pi/4` and the correction are exact in
/// two parts, so the result takes one rounding of the sum after the arctangent's own.
/// Infinities enter as a ratio of 1 (both infinite) or 0, values above a quarter of the largest finite are quartered so
/// that `mx + mn` stays finite, and a NaN in either argument returns a NaN.
template<std::floating_point T>
[[gnu::always_inline]] inline T atan2Branchless(T y, T x) noexcept {
    constexpr T kInf  = std::numeric_limits<T>::infinity();
    constexpr T kHuge = std::numeric_limits<T>::max() / T(4);
    // pi/4 = kQuarterHigh + kQuarterLow, the high part with few enough bits that its multiples up to 4 are exact
    constexpr T kQuarterHigh = std::is_same_v<T, float> ? T(0.78515625) : static_cast<T>(static_cast<float>(std::numbers::pi / 4.0));
    constexpr T kQuarterLow  = static_cast<T>(std::numbers::pi_v<long double> / 4.0L - static_cast<long double>(kQuarterHigh));
    const T     ax           = std::fabs(x);
    const T     ay           = std::fabs(y);
    T           mx           = ax < ay ? ay : ax;
    T           mn           = ax < ay ? ax : ay;
    const bool  infinite     = mx == kInf;
    mn                       = infinite ? (mn == kInf ? T(1) : T(0)) : mn;
    mx                       = infinite ? T(1) : mx;
    const T shrink           = mx > kHuge ? T(0.25) : T(1);
    mx *= shrink;
    mn *= shrink;
    const bool upper = mn > T(0.41421356237309503) * mx;
    const T    num   = upper ? mn - mx : mn;
    const T    den0  = upper ? mx + mn : mx;
    const T    den   = den0 == T(0) ? T(1) : den0;
    const T    atanU = atanSmall<T>(num / den);
    const T    k0    = upper ? T(1) : T(0);
    const T    k1    = ay > ax ? T(2) - k0 : k0;
    const T    s1    = ay > ax ? T(-1) : T(1);
    const bool neg   = std::signbit(x);
    const T    k     = neg ? T(4) - k1 : k1;
    const T    s     = neg ? -s1 : s1;
    const T    r     = std::copysign(k * kQuarterHigh + (s * atanU + k * kQuarterLow), y);
    return (x != x || y != y) ? x + y : r;
}

/// @brief `out[i] = scale * atan2Branchless(y[i], x[i])`; one loop, no call, the same arithmetic for every element.
template<std::floating_point T>
void scaledAtan2Branchless(std::span<const T> y, std::span<const T> x, std::span<T> out, T scale) noexcept {
    const std::size_t n  = std::min({y.size(), x.size(), out.size()});
    const T* const    py = y.data();
    const T* const    px = x.data();
    T* const          po = out.data();
#if defined(__clang__)
// the hint is a request: a target without a vector unit for T (WebAssembly without simd128 for double) leaves the loop scalar, which is not an error
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpass-failed"
#pragma clang loop vectorize(enable)
#endif
    for (std::size_t i = 0UZ; i < n; ++i) {
        po[i] = scale * atan2Branchless<T>(py[i], px[i]);
    }
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
}

template<std::floating_point T>
void scaledAtan2Library(std::span<const T> y, std::span<const T> x, std::span<T> out, T scale) noexcept {
    const std::size_t n = std::min({y.size(), x.size(), out.size()});
    for (std::size_t i = 0UZ; i < n; ++i) {
        out[i] = scale * std::atan2(y[i], x[i]);
    }
}

} // namespace detail

/// @brief The implementation that a build of this header takes for `T`. A compile-time choice by compiler, type and the
/// native vector width; no setting or run-time switch changes it.
template<std::floating_point T>
[[nodiscard]] constexpr Atan2Form atan2Form() noexcept {
#if defined(__clang__)
    return Atan2Form::Branchless;
#else
    if constexpr (std::is_same_v<T, float> || vir::stdx::native_simd<T>::size() >= 4UZ) {
        return Atan2Form::Lanes;
    } else {
        return Atan2Form::Library;
    }
#endif
}

/// @brief `out[i] = scale * atan2(y[i], x[i])` for the first `min(y.size(), x.size(), out.size())` elements.
///
/// Every element of one build goes through the same implementation with the same arithmetic, so the values of a stream
/// cut into calls anywhere equal the values of one call. The implementation is `atan2Form<T>()`:
/// - `Lanes`, taken with g++ for `float`, and for `double` when the native vector holds four lanes or more: within 2 ulp
///   of the exact result (`qa_Atan2` measures at most 0.66 ulp for `float` and 1.33 for `double`);
/// - `Branchless`, taken with clang (including the WebAssembly target): within 4 ulp of the exact result (`qa_Atan2`
///   measures at most 2.34 ulp for `float` and 2.23 for `double` under clang, 2.62 and 2.41 under g++). The compiler
///   vectorizes one loop where `gr::meta::atan2` on lanes would run as a loop over lanes;
/// - `Library`, taken with g++ for `double` on a vector narrower than four lanes: `std::atan2`.
///
/// Results follow C's `atan2` at zeros of either sign, on the axes, at infinities and for a NaN, which returns a NaN. A
/// result can differ from `std::atan2`'s by up to the form's bound. `scale` multiplies the result of the arctangent.
/// `out` may be `y` or `x` exactly, and must not partly overlap either.
template<std::floating_point T>
void scaledAtan2(std::span<const T> y, std::span<const T> x, std::span<T> out, T scale) noexcept {
    constexpr Atan2Form form = atan2Form<T>();
    if constexpr (form == Atan2Form::Lanes) {
        detail::scaledAtan2Lanes<T>(y, x, out, scale);
    } else if constexpr (form == Atan2Form::Branchless) {
        detail::scaledAtan2Branchless<T>(y, x, out, scale);
    } else {
        detail::scaledAtan2Library<T>(y, x, out, scale);
    }
}

} // namespace gr::math

#endif // GNURADIO_ALGORITHM_MATH_ATAN2_HPP
