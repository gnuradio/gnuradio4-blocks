#ifndef GNURADIO_SDR_SATURATE_HPP
#define GNURADIO_SDR_SATURATE_HPP

#include <algorithm>
#include <complex>
#include <span>

namespace gr::blocks::sdr {

// Saturates each part of every sample to [-1, 1], in place. A CF32 SoapySDR stream is full scale at +/-1.0, and a
// driver converts the real and the imaginary part separately, so what has to stay within full scale is each part
// and not the magnitude.
//
// A part already within the range comes back bit-identical, the sign of a zero included: the bounds are applied
// with std::min and std::max, which compare with <, and not with std::fmin and std::fmax, which return the bound
// for a NaN operand. A NaN part therefore passes through unchanged; it is a fault upstream of the transmitter, and
// this function neither hides it nor invents a sample value for it. An infinity is ordered and becomes the bound.
inline void saturateToFullScale(std::span<std::complex<float>> samples) noexcept {
    for (std::complex<float>& sample : samples) {
        sample = std::complex<float>(std::min(std::max(sample.real(), -1.0f), 1.0f), std::min(std::max(sample.imag(), -1.0f), 1.0f));
    }
}

} // namespace gr::blocks::sdr

#endif // GNURADIO_SDR_SATURATE_HPP
