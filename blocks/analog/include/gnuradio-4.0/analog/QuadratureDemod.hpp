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
#include <gnuradio-4.0/algorithm/math/Atan2.hpp>
#include <gnuradio-4.0/annotated.hpp>

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
exactly `0.0`. The argument is evaluated by `gr::math::scaledAtan2` on each tile of products, within 4 ulp of the exact
result (2 ulp with g++ on simd lanes, `std::atan2` with g++ for double on a vector narrower than four lanes, 4 ulp
with clang++). A product of zero gives exactly `0.0`, whatever the signs of
its zero parts. The output can differ from `std::atan2`'s by up to the bound, and a stream cut at any call boundary gives
the values of one call. The float rounding of the complex product mostly sets the error floor: a frequency-modulated
tone's residual against a double reference is -147.21 dB (5 kHz deviation at 48 kS/s) and -152.19 dB (75 kHz at 240 kS/s)
below the tone with g++, the same as `std::atan2`, and -145.31 dB and -151.72 dB with clang.

The block is 1:1, so every input tag key passes through at its own offset.
)"">;

    PortIn<std::complex<T>> in;
    PortOut<T>              out;

    Annotated<T, "gain", Unit<"1/rad">, Doc<"output units per radian of per-sample phase advance">> gain = T(1);

    GR_MAKE_REFLECTABLE(QuadratureDemod, in, out, gain);

    std::complex<T> _previous{};

    void reset() { _previous = std::complex<T>{}; }

    [[nodiscard]] work::Status processBulk(std::span<const std::complex<T>> input, std::span<T> output) noexcept {
        const std::size_t nSamples = std::min(input.size(), output.size());
        if (nSamples == 0UZ) {
            return work::Status::OK;
        }

        constexpr std::size_t kTile = 256UZ;
        std::array<T, kTile>  productReal;
        std::array<T, kTile>  productImag;
        // a product of zero is written as +0 in both parts, so its argument is 0 whatever the signs of its zero parts
        const auto store = [&](std::size_t i, T real, T imag) {
            const bool zero = real == T(0) && imag == T(0);
            productReal[i]  = zero ? T(0) : real;
            productImag[i]  = zero ? T(0) : imag;
        };
        for (std::size_t base = 0UZ; base < nSamples; base += kTile) {
            const std::size_t count = std::min(kTile, nSamples - base);
            std::size_t       first = 0UZ;
            if (base == 0UZ) { // the first sample pairs with the history
                const std::complex<T> sample = input[0UZ];
                store(0UZ, sample.real() * _previous.real() + sample.imag() * _previous.imag(), sample.imag() * _previous.real() - sample.real() * _previous.imag());
                first = 1UZ;
            }
            for (std::size_t i = first; i < count; ++i) {
                const std::complex<T> sample = input[base + i];
                const std::complex<T> prior  = input[base + i - 1UZ];
                store(i, sample.real() * prior.real() + sample.imag() * prior.imag(), sample.imag() * prior.real() - sample.real() * prior.imag());
            }
            gr::math::scaledAtan2<T>(std::span<const T>(productImag).first(count), std::span<const T>(productReal).first(count), output.subspan(base, count), gain);
        }

        _previous = input[nSamples - 1UZ];
        return work::Status::OK;
    }
};

} // namespace gr::blocks::analog

#endif // GNURADIO_QUADRATURE_DEMOD_HPP
