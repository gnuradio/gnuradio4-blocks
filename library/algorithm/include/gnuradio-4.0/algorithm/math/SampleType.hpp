#ifndef GNURADIO_ALGORITHM_MATH_SAMPLE_TYPE_HPP
#define GNURADIO_ALGORITHM_MATH_SAMPLE_TYPE_HPP

#include <complex>
#include <concepts>

namespace gr::math {

/// @brief A real or complex floating-point sample: `float`, `double`, `std::complex<float>` or `std::complex<double>`.
template<typename T>
concept RealOrComplexSample = std::same_as<T, float> || std::same_as<T, double> || std::same_as<T, std::complex<float>> || std::same_as<T, std::complex<double>>;

/// @brief The real scalar of a sample type and whether the type is complex.
///
/// `type` is `T` for a real type and `F` for `std::complex<F>`. `isComplex` is true for `std::complex<F>` alone.
template<typename T>
struct RealScalarOf {
    using type                      = T;
    static constexpr bool isComplex = false;
};

template<typename F>
struct RealScalarOf<std::complex<F>> {
    using type                      = F;
    static constexpr bool isComplex = true;
};

template<typename T>
using RealScalarOfT = typename RealScalarOf<T>::type;

} // namespace gr::math

#endif // GNURADIO_ALGORITHM_MATH_SAMPLE_TYPE_HPP
