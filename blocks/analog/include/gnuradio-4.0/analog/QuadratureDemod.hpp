#ifndef GNURADIO_QUADRATURE_DEMOD_HPP
#define GNURADIO_QUADRATURE_DEMOD_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <concepts>
#include <span>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::analog {

GR_REGISTER_BLOCK(gr::blocks::analog::QuadratureDemod, [T], [float])

template<std::floating_point T>
struct QuadratureDemod : Block<QuadratureDemod<T>, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief Recovers the instantaneous frequency of a complex baseband signal.

Computes `y[n] = gain * arg(x[n] * conj(x[n-1]))`, the per-sample phase advance in radians scaled by `gain`. This is the
discriminator for FM, FSK, GMSK and any other constant-envelope angle modulation.

`gain` is a raw scale factor with no sample-rate dependence. An FM demodulator whose output should reach +-1.0 at full
deviation uses `gain = sample_rate / (2 * pi * deviation)`; a caller that changes the sample rate must recompute and
re-apply `gain` itself, because the mapping from radians to physical output units is an application decision.

The block holds one sample of history, zero at construction and after `reset()`, so the first output sample of a run is
exactly `0.0`. The argument is evaluated with `std::atan2`, with no lookup table or polynomial approximation, so the
output carries no signal-correlated error floor.

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

        {
            const std::complex<T> sample = input[0UZ];
            const T               real   = sample.real() * _previous.real() + sample.imag() * _previous.imag();
            const T               imag   = sample.imag() * _previous.real() - sample.real() * _previous.imag();
            output[0UZ]                  = gain * std::atan2(imag, real);
        }

        constexpr std::size_t kTile = 256UZ;
        std::array<T, kTile>  productReal;
        std::array<T, kTile>  productImag;
        for (std::size_t base = 1UZ; base < nSamples; base += kTile) {
            const std::size_t count = std::min(kTile, nSamples - base);
            for (std::size_t i = 0UZ; i < count; ++i) {
                const std::complex<T> sample = input[base + i];
                const std::complex<T> prior  = input[base + i - 1UZ];
                productReal[i]               = sample.real() * prior.real() + sample.imag() * prior.imag();
                productImag[i]               = sample.imag() * prior.real() - sample.real() * prior.imag();
            }
            for (std::size_t i = 0UZ; i < count; ++i) {
                output[base + i] = gain * std::atan2(productImag[i], productReal[i]);
            }
        }

        _previous = input[nSamples - 1UZ];
        return work::Status::OK;
    }
};

} // namespace gr::blocks::analog

#endif // GNURADIO_QUADRATURE_DEMOD_HPP
