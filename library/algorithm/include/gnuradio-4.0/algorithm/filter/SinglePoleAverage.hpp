#ifndef GNURADIO_ALGORITHM_FILTER_SINGLE_POLE_AVERAGE_HPP
#define GNURADIO_ALGORITHM_FILTER_SINGLE_POLE_AVERAGE_HPP

#include <array>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <limits>
#include <span>
#include <type_traits>
#include <utility>

#include <gnuradio-4.0/meta/utils.hpp>

namespace gr::filter {

/**
 * @brief The single-pole average `e = alpha * x + (1 - alpha) * e` of a real or complex stream, kept in `double`.
 *
 * `subtract()` writes each sample less the average updated with it: a high-pass with its zero at DC, which removes an
 * offset. `average()` writes the average itself. Both update the same estimate, which starts at zero. `alpha` lies in
 * (0, 1], and the time constant is `1/alpha - 1` samples. The estimate runs in `double`, since in `float` the factor
 * `1 - alpha` rounds to 1 once `1/alpha` reaches `2^25`, and the recursion then integrates its input.
 *
 * The kernel solves eight samples at a time. With `p = 1 - alpha`, the estimate after sample `k` of a group is
 * `p^(k+1) * (e + sum over j <= k of alpha * p^-(j+1) * x[j])`, an inclusive prefix sum. Only the estimate carried from
 * one group to the next is serial. The estimate carried between groups differs from the recursion's by a few `double`
 * roundings of the signal. A `float` output differs from the recursion's by less than one float rounding of the signal,
 * and where the stream is split into calls can change which outputs differ. Where `(1 - alpha)^8` lies below `2^-256`
 * the kernel runs the recursion one sample at a time.
 *
 * A sample with a NaN or infinite component leaves the estimate unchanged, and its output uses the held estimate. A
 * component of the estimate below the smallest normal `double` is set to zero. A silent input then takes the estimate
 * to an exact zero and keeps it out of the subnormal range, where every operation is slow.
 */
template<typename T>
requires(std::floating_point<T> || gr::meta::complex_like<T>)
class SinglePoleAverage {
    using Real = decltype(std::real(std::declval<T>()));

    static constexpr std::size_t kWidth = sizeof(T) / sizeof(Real); // components per sample
    static constexpr std::size_t kBlock = 8UZ;                      // samples solved together
    static constexpr std::size_t kCount = kBlock * kWidth;          // components solved together

public:
    using value_type    = T;
    using estimate_type = std::conditional_t<kWidth == 2UZ, std::complex<double>, double>;

    SinglePoleAverage() noexcept { configure(1.0); }

    /// Sets the weight of each new sample and keeps the estimate. Returns false and changes nothing for an `alpha`
    /// outside (0, 1].
    [[nodiscard]] bool setAlpha(double alpha) noexcept {
        if (!(alpha > 0.0 && alpha <= 1.0)) {
            return false;
        }
        configure(alpha);
        return true;
    }

    [[nodiscard]] double alpha() const noexcept { return _alpha; }

    [[nodiscard]] estimate_type estimate() const noexcept {
        if constexpr (kWidth == 2UZ) {
            return {_real, _imag};
        } else {
            return _real;
        }
    }

    /// Sets the estimate to zero.
    void reset() noexcept {
        _real = 0.0;
        _imag = 0.0;
    }

    /// Writes each sample of @p in less the estimate updated with it to @p out, which holds at least as many samples.
    /// @p out may be @p in itself.
    void subtract(std::span<const T> in, std::span<T> out) noexcept { run<true>(in, out); }

    /// Writes the estimate updated with each sample of @p in to @p out, which holds at least as many samples. @p out
    /// may be @p in itself.
    void average(std::span<const T> in, std::span<T> out) noexcept { run<false>(in, out); }

private:
    double                     _alpha         = 1.0;
    double                     _oneMinusAlpha = 0.0;
    double                     _real          = 0.0;
    double                     _imag          = 0.0;
    bool                       _blocked       = false; ///< whole groups are solved together, off for alpha near 1
    std::array<double, kBlock> _weight{};              ///< alpha * (1 - alpha)^-(k+1)
    std::array<double, kBlock> _decay{};               ///< (1 - alpha)^(k+1)

    void configure(double alpha) noexcept {
        _alpha         = alpha;
        _oneMinusAlpha = 1.0 - alpha;
        double power   = 1.0;
        for (std::size_t k = 0UZ; k < kBlock; ++k) {
            power *= _oneMinusAlpha;
            _decay[k]  = power;
            _weight[k] = alpha / power;
        }
        _blocked = power >= 0x1p-256; // weights below 2^256 keep the sums of finite floats finite
    }

    template<bool kResidual>
    void run(std::span<const T> in, std::span<T> out) noexcept {
        const std::size_t blocked = _blocked ? in.size() / kBlock * kBlock : 0UZ;
        for (std::size_t start = 0UZ; start < blocked; start += kBlock) {
            solveBlock<kResidual>(in.data() + start, out.data() + start);
        }
        for (std::size_t i = blocked; i < in.size(); ++i) {
            out[i] = step<kResidual>(in[i]);
        }
    }

    template<bool kResidual>
    [[nodiscard]] T step(T sample) noexcept {
        if constexpr (kWidth == 2UZ) {
            const double real     = static_cast<double>(sample.real());
            const double imag     = static_cast<double>(sample.imag());
            const double nextReal = _alpha * real + _oneMinusAlpha * _real;
            const double nextImag = _alpha * imag + _oneMinusAlpha * _imag;
            if (std::isfinite(nextReal) && std::isfinite(nextImag)) {
                _real = flushed(nextReal);
                _imag = flushed(nextImag);
            }
            return kResidual ? T(static_cast<Real>(real - _real), static_cast<Real>(imag - _imag)) : T(static_cast<Real>(_real), static_cast<Real>(_imag));
        } else {
            const double real     = static_cast<double>(sample);
            const double nextReal = _alpha * real + _oneMinusAlpha * _real;
            if (std::isfinite(nextReal)) {
                _real = flushed(nextReal);
            }
            return static_cast<T>(kResidual ? real - _real : _real);
        }
    }

    /// Solves the recursion for `kBlock` samples from the estimate before them, by the prefix sum the class describes.
    template<bool kResidual>
    void solveBlock(const T* input, T* output) noexcept {
        const auto*                x = static_cast<const Real*>(static_cast<const void*>(input));
        auto*                      y = static_cast<Real*>(static_cast<void*>(output));
        std::array<double, kCount> value; // the input in double
        std::array<double, kCount> sum;
        for (std::size_t j = 0UZ; j < kCount; ++j) {
            value[j] = static_cast<double>(x[j]);
            sum[j]   = _weight[j / kWidth] * value[j];
        }
        for (std::size_t shift = kWidth; shift < kCount; shift *= 2UZ) {
            const std::array<double, kCount> before = sum;
            for (std::size_t j = shift; j < kCount; ++j) {
                sum[j] += before[j - shift];
            }
        }
        if (!std::isfinite(sum[kCount - kWidth]) || !std::isfinite(sum[kCount - 1UZ])) { // a NaN or infinite sample holds the estimate
            for (std::size_t k = 0UZ; k < kBlock; ++k) {
                output[k] = step<kResidual>(input[k]);
            }
            return;
        }
        const std::array<double, 2UZ> start{_real, _imag};
        for (std::size_t j = 0UZ; j < kCount; ++j) {
            sum[j] = _decay[j / kWidth] * (start[j % kWidth] + sum[j]);
            y[j]   = static_cast<Real>(kResidual ? value[j] - sum[j] : sum[j]);
        }
        _real = flushed(sum[kCount - kWidth]);
        if constexpr (kWidth == 2UZ) {
            _imag = flushed(sum[kCount - 1UZ]);
        }
    }

    [[nodiscard]] static constexpr double flushed(double value) noexcept { return std::abs(value) < std::numeric_limits<double>::min() ? 0.0 : value; }
};

} // namespace gr::filter

#endif // GNURADIO_ALGORITHM_FILTER_SINGLE_POLE_AVERAGE_HPP
