#ifndef GNURADIO_HARD_LIMITER_HPP
#define GNURADIO_HARD_LIMITER_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <span>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/meta/simd.hpp>
#include <gnuradio-4.0/meta/utils.hpp>

namespace gr::blocks::analog {

GR_REGISTER_BLOCK(gr::blocks::analog::HardLimiter, [T], [ std::complex<float>, float ])

template<typename T>
requires(gr::meta::complex_like<T> || std::floating_point<T>)
struct HardLimiter : Block<HardLimiter<T>> {
    using Description = Doc<R""(
@brief Scales every complex sample to unit magnitude, leaving only the phase; the signum on a real stream.

A constant-envelope demodulator wants only the phase, and removing the amplitude with no time constant is the point:
an automatic gain control in the same position moves its gain on a schedule of its own, and while a discriminator's
angle is immune to gain, the noise floor it produces between transmissions is not - a rising gain in dead air lifts
noise into a weak signal's range, and a symbol tracker downstream cannot tell the two apart. A fixed limiter has
nothing to drift with.

A sample of exactly zero has no direction and passes through as zero - never a non-finite value - which is what its
phase difference with anything already is. Stateless and 1:1. The real and `std::complex<double>` forms may sit anywhere in a fused run; the
`std::complex<float>` form processes each span whole and does not fuse.

Each part of a `std::complex<float>` output is within four float steps of the part divided by the exact magnitude.
A sample whose squared magnitude lies outside 1e-30 to 1e30 (a tiny, huge, infinite or NaN one) is computed in double.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    GR_MAKE_REFLECTABLE(HardLimiter, in, out);

    /// @brief The unit sample of one `std::complex<float>`, its parts and their squares held in double: no square of a float part overflows or underflows.
    [[nodiscard]] static constexpr T exactUnit(T sample) noexcept
    requires std::same_as<T, std::complex<float>>
    {
        const double re  = static_cast<double>(sample.real());
        const double im  = static_cast<double>(sample.imag());
        const double sum = re * re + im * im;
        if (!(sum > 0.0)) {
            return T{};
        }
        const double inverse = 1.0 / std::sqrt(sum);
        return T(static_cast<float>(re * inverse), static_cast<float>(im * inverse));
    }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept
    requires std::same_as<T, std::complex<float>>
    {
        // Within [kLow, kHigh] the float square of the magnitude neither overflows nor loses its digits to underflow, and a
        // float square root and divide give the unit sample; a zero, a tiny, a huge, an infinite or a NaN sample takes
        // exactUnit(). The parts are taken apart a tile at a time so that the square root and the divide run on whole
        // vectors: the per-sample form branches on the zero case and runs a double square root and a double divide,
        // and the compiler does not vectorize it.
        using V                      = vir::stdx::native_simd<float>;
        constexpr std::size_t kWidth = V::size();
        constexpr std::size_t kTile  = 256UZ;
        static_assert(kTile % kWidth == 0UZ);
        constexpr float kLow  = 1e-30f;
        constexpr float kHigh = 1e30f;

        std::array<float, kTile> re;
        std::array<float, kTile> im;
        std::array<float, kTile> unitRe;
        std::array<float, kTile> unitIm;
        const std::size_t        nSamples = std::min(input.size(), output.size());
        for (std::size_t base = 0UZ; base < nSamples; base += kTile) {
            const std::size_t count  = std::min(kTile, nSamples - base);
            const std::size_t padded = (count + kWidth - 1UZ) / kWidth * kWidth;
            for (std::size_t i = 0UZ; i < count; ++i) {
                re[i] = input[base + i].real();
                im[i] = input[base + i].imag();
            }
            std::fill(re.begin() + static_cast<std::ptrdiff_t>(count), re.begin() + static_cast<std::ptrdiff_t>(padded), 1.f); // an in-range filler
            std::fill(im.begin() + static_cast<std::ptrdiff_t>(count), im.begin() + static_cast<std::ptrdiff_t>(padded), 0.f);
            for (std::size_t i = 0UZ; i < padded; i += kWidth) {
                const V    r(re.data() + i, vir::stdx::element_aligned);
                const V    m(im.data() + i, vir::stdx::element_aligned);
                const V    sum     = r * r + m * m;
                const V    inverse = V(1.f) / vir::stdx::sqrt(sum);
                V          a       = r * inverse;
                V          b       = m * inverse;
                const auto outside = !(sum >= kLow && sum <= kHigh); // a zero, tiny, huge, infinite or NaN sum
                if (vir::stdx::any_of(outside)) {
                    for (std::size_t k = 0UZ; k < kWidth; ++k) {
                        if (outside[k]) {
                            const T unit = exactUnit(T(re[i + k], im[i + k]));
                            a[k]         = unit.real();
                            b[k]         = unit.imag();
                        }
                    }
                }
                a.copy_to(unitRe.data() + i, vir::stdx::element_aligned);
                b.copy_to(unitIm.data() + i, vir::stdx::element_aligned);
            }
            for (std::size_t i = 0UZ; i < count; ++i) {
                output[base + i] = T(unitRe[i], unitIm[i]);
            }
        }
        return work::Status::OK;
    }

    [[nodiscard]] constexpr T processOne(T sample) const noexcept
    requires(!std::same_as<T, std::complex<float>>)
    {
        if constexpr (gr::meta::complex_like<T>) {
            const auto magnitude = std::abs(sample);
            return magnitude > typename T::value_type{0} ? sample / magnitude : T{};
        } else {
            return sample > T{0} ? T{1} : (sample < T{0} ? T{-1} : T{0});
        }
    }
};

} // namespace gr::blocks::analog

#endif // GNURADIO_HARD_LIMITER_HPP
