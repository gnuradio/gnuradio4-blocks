#ifndef GNURADIO_QUADRATURE_DEMOD_HPP
#define GNURADIO_QUADRATURE_DEMOD_HPP

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
#include <gnuradio-4.0/annotated.hpp>
#include <gnuradio-4.0/meta/simd_math.hpp>

namespace gr::blocks::analog {

GR_REGISTER_BLOCK(gr::blocks::analog::QuadratureDemod, [T], [float])

template<std::floating_point T>
struct QuadratureDemod : Block<QuadratureDemod<T>> {
    using Description = Doc<R""(
@brief Recovers the instantaneous frequency of a complex baseband signal.

Computes `y[n] = gain * arg(x[n] * conj(x[n-1]))`, the per-sample phase advance in radians scaled by `gain`. This is the
discriminator for FM, FSK, GMSK and any other constant-envelope angle modulation.

`gain` is a raw scale factor with no sample-rate dependence. An FM demodulator whose output should reach +-1.0 at full
deviation uses `gain = sample_rate / (2 * pi * deviation)`; a caller that changes the sample rate must recompute and
re-apply `gain` itself, because the mapping from radians to physical output units is an application decision.

The block holds one sample of history, zero at construction and after `reset()`, so the first output sample of a run is
exactly `0.0`. The argument is `gr::meta::atan2` of the product on simd vectors, within 2 ulp of the exact value. A
product of zero gives exactly `0.0`, whatever the signs of its zero parts.

The block is 1:1, so every input tag key passes through at its own offset.
)"">;

    PortIn<std::complex<T>> in;
    PortOut<T>              out;

    Annotated<T, "gain", Unit<"1/rad">, Doc<"output units per radian of per-sample phase advance">> gain = T(1);

    GR_MAKE_REFLECTABLE(QuadratureDemod, in, out, gain);

    std::complex<T> _previous{};

    void reset() { _previous = std::complex<T>{}; }

    [[nodiscard]] work::Status processBulk(std::span<const std::complex<T>> input, std::span<T> output) noexcept {
        using V                      = vir::stdx::native_simd<T>;
        constexpr std::size_t kWidth = V::size();
        constexpr std::size_t kTile  = 256UZ;
        static_assert(kTile % kWidth == 0UZ);
        const std::size_t nSamples = std::min(input.size(), output.size());
        if (nSamples == 0UZ) {
            return work::Status::OK;
        }

        std::array<T, kTile> productReal;
        std::array<T, kTile> productImag;
        for (std::size_t base = 0UZ; base < nSamples; base += kTile) {
            const std::size_t count  = std::min(kTile, nSamples - base);
            const std::size_t padded = (count + kWidth - 1UZ) / kWidth * kWidth;
            std::size_t       first  = 0UZ;
            if (base == 0UZ) {
                const std::complex<T> sample = input[0UZ];
                productReal[0UZ]             = sample.real() * _previous.real() + sample.imag() * _previous.imag();
                productImag[0UZ]             = sample.imag() * _previous.real() - sample.real() * _previous.imag();
                first                        = 1UZ;
            }
            for (std::size_t i = first; i < count; ++i) {
                const std::complex<T> sample = input[base + i];
                const std::complex<T> prior  = input[base + i - 1UZ];
                productReal[i]               = sample.real() * prior.real() + sample.imag() * prior.imag();
                productImag[i]               = sample.imag() * prior.real() - sample.real() * prior.imag();
            }
            std::fill(productReal.begin() + static_cast<std::ptrdiff_t>(count), productReal.begin() + static_cast<std::ptrdiff_t>(padded), T(1));
            std::fill(productImag.begin() + static_cast<std::ptrdiff_t>(count), productImag.begin() + static_cast<std::ptrdiff_t>(padded), T(0));
            for (std::size_t i = 0UZ; i < padded; i += kWidth) {
                const V real(productReal.data() + i, vir::stdx::element_aligned);
                const V imag(productImag.data() + i, vir::stdx::element_aligned);
                V       phase                              = gr::meta::atan2(imag, real);
                where(real == V(0) && imag == V(0), phase) = V(0);
                const V scaled                             = V(gain.value) * phase;
                if (i + kWidth <= count) {
                    scaled.copy_to(output.data() + base + i, vir::stdx::element_aligned);
                } else {
                    std::array<T, kWidth> last;
                    scaled.copy_to(last.data(), vir::stdx::element_aligned);
                    std::copy_n(last.begin(), count - i, output.begin() + static_cast<std::ptrdiff_t>(base + i));
                }
            }
        }

        _previous = input[nSamples - 1UZ];
        return work::Status::OK;
    }
};

} // namespace gr::blocks::analog

#endif // GNURADIO_QUADRATURE_DEMOD_HPP
