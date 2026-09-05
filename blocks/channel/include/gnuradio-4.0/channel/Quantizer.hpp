#ifndef GNURADIO_CHANNEL_QUANTIZER_HPP
#define GNURADIO_CHANNEL_QUANTIZER_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <type_traits>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::channel {

GR_REGISTER_BLOCK(gr::blocks::channel::Quantizer, [T], [ float, std::complex<float> ])

/**
 * @brief Mid-tread converter quantization with saturation.
 *
 *     y = clamp(round(x/delta), -2^(B-1), 2^(B-1) - 1) * delta,   delta = full_scale * 2^(1-B)
 *
 * Complex input quantizes each axis independently, which is what a pair of real converters does. Mid-tread
 * means zero is a code, so a zero input survives exactly. Idempotent by construction: quantizing an already
 * quantized value lands on the same code.
 */
template<typename T>
requires(std::is_same_v<T, float> || std::is_same_v<T, std::complex<float>>)
struct Quantizer : gr::Block<Quantizer<T>> {
    using Description = Doc<R""(
@brief Mid-tread quantization with saturation, one converter per axis.

`bits` counts the whole word including its sign, so 8 bits gives codes -128..127, and values beyond
`full_scale` land exactly on the rails.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "bits", Visible, Doc<"converter word length including the sign bit, 2-24">> bits       = 12U;
    Annotated<double, "full_scale", Visible, Doc<"amplitude mapping to the positive rail">>           full_scale = 1.0;

    GR_MAKE_REFLECTABLE(Quantizer, in, out, bits, full_scale);

    /// Half the code range, 2^(bits-1): the grid is stated here once, and read from the members wherever it is needed.
    [[nodiscard]] static double halfRange(gr::Size_t wordLength) noexcept { return std::exp2(static_cast<double>(wordLength) - 1.); }

    // The grid follows the members, never the keys a batch named: a converter left at its declared defaults is
    // never called back and must still quantize on its own grid.
    double _delta       = full_scale.value / halfRange(bits.value);
    double _lowestCode  = -halfRange(bits.value);
    double _highestCode = halfRange(bits.value) - 1.;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (bits < 2U || bits > 24U) {
            throw gr::exception(std::format("Quantizer: 'bits' must be in [2, 24], got {}", bits.value));
        }
        if (full_scale <= 0.0) {
            throw gr::exception(std::format("Quantizer: 'full_scale' must be positive, got {}", full_scale.value));
        }
        _delta       = full_scale.value / halfRange(bits.value);
        _lowestCode  = -halfRange(bits.value);
        _highestCode = halfRange(bits.value) - 1.;
    }

    [[nodiscard]] float quantize(float value) const noexcept {
        const double code = std::clamp(std::round(static_cast<double>(value) / _delta), _lowestCode, _highestCode);
        return static_cast<float>(code * _delta);
    }

    [[nodiscard]] T processOne(T sample) const noexcept {
        if constexpr (std::is_same_v<T, std::complex<float>>) {
            return T(quantize(sample.real()), quantize(sample.imag()));
        } else {
            return quantize(sample);
        }
    }
};

} // namespace gr::blocks::channel

#endif // GNURADIO_CHANNEL_QUANTIZER_HPP
