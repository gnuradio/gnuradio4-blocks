#ifndef GNURADIO_SDR_SATURATE_HPP
#define GNURADIO_SDR_SATURATE_HPP

#include <algorithm>
#include <complex>
#include <span>

namespace gr::blocks::sdr {

// Saturates each part of every sample to [-1, 1]. A CF32 SoapySDR stream is full scale at +/-1.0, and a driver
// converts the real and the imaginary part separately, so what has to stay within full scale is each part and not
// the magnitude. The first form writes the saturated samples to `to`, which holds as many samples as `from` and may
// be `from` itself; the second saturates in place.
//
// A part already within the range comes back bit-identical, the sign of a zero included: the bounds are applied
// with std::min and std::max, which compare with <, and not with std::fmin and std::fmax, which return the bound
// for a NaN operand. A NaN part therefore passes through unchanged; it is a fault upstream of the transmitter, and
// this function neither hides it nor invents a sample value for it. An infinity is ordered and becomes the bound.
inline void saturateToFullScale(std::span<const std::complex<float>> from, std::span<std::complex<float>> to) noexcept {
    for (std::size_t i = 0UZ; i < from.size(); ++i) {
        to[i] = std::complex<float>(std::min(std::max(from[i].real(), -1.0f), 1.0f), std::min(std::max(from[i].imag(), -1.0f), 1.0f));
    }
}

inline void saturateToFullScale(std::span<std::complex<float>> samples) noexcept { saturateToFullScale(samples, samples); }

} // namespace gr::blocks::sdr

#endif // GNURADIO_SDR_SATURATE_HPP
