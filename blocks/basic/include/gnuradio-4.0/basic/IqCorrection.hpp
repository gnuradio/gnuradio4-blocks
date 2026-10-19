#ifndef GNURADIO_IQ_CORRECTION_HPP
#define GNURADIO_IQ_CORRECTION_HPP

#include <array>
#include <cmath>
#include <complex>
#include <format>
#include <limits>
#include <numbers>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::basic {

namespace detail {

/// @brief The validator of `sample_rate`: true above 0.
inline constexpr auto kSampleRatePositive = [](float rate) { return rate > 0.f; };

/// @brief The validator of `tau`: true above 0.
inline constexpr auto kTauPositive = [](double seconds) { return seconds > 0.0; };

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::basic::IqSwap, [T], [std::complex<float>])

template<typename T>
requires(gr::meta::complex_like<T>)
struct IqSwap : Block<IqSwap<T>> {
    using Description = Doc<R""(
@brief Exchanges the real and imaginary components of every sample, mirroring the spectrum about DC.

The block follows a receiver whose hardware or driver exchanges I and Q. Such a receiver shows a signal from `+f` at
`-f`, with its sidebands inverted. The block puts the signal back. It takes complex samples and gives one out for each
one in. Every tag passes through on its own sample.

`enabled` turns the exchange on, from the next sample the block processes. Disabled, the default, the block passes every
sample through unchanged. The exchange equals conjugation followed by a quarter turn, `swap(x) = j * conj(x)`.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<bool, "enabled", Doc<"exchange I and Q">, Visible> enabled = false;

    GR_MAKE_REFLECTABLE(IqSwap, in, out, enabled);

    [[nodiscard]] constexpr T processOne(T sample) const noexcept { return enabled ? T(sample.imag(), sample.real()) : sample; }
};

GR_REGISTER_BLOCK(gr::blocks::basic::DcOffsetCorrect, [T], [ std::complex<float>, float ])

template<typename T>
requires(gr::meta::complex_like<T> || std::floating_point<T>)
struct DcOffsetCorrect : Block<DcOffsetCorrect<T>> {
    using Description = Doc<R""(
@brief Subtracts a running estimate of the mean, removing DC from a complex or a real stream.

On a complex stream the block removes a converter's bias and an LO leaking into its own mixer. On a real stream it
removes the carrier level from an AM detector's envelope. It gives one sample out for each one in, of the input's type.
The output has no delay. Every tag passes through on its own sample, and a
`sample_rate` key in a tag leaves with this block's value.

The estimate is a single-pole average, `e = alpha * x + (1 - alpha) * e`, with `alpha = 1/(1 + tau*sample_rate)`. It
runs in `double`, since in `float` the factor `1 - alpha` rounds to 1 once `tau*sample_rate` reaches `2^25`. A sample
with a NaN or infinite component leaves the estimate unchanged, and the block outputs that sample less the held
estimate. `dcEstimate()` returns the estimate, with the imaginary part zero on a real stream.

`enabled` turns the subtraction on. Disabled, the default, the block passes every sample through unchanged and holds the
estimate. `tau` sets the time constant in seconds, and `sample_rate` the stream rate it counts against. A change of
either moves the pole and keeps the estimate. The block writes the derived `alpha` and `corner_hz`, the high-pass -3 dB
point. It refuses a `tau` and `sample_rate` whose product overflows a `double`, and a written `alpha` or `corner_hz`
that differs from the value it derives. `reset()` sets the estimate to zero.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<bool, "enabled", Doc<"subtract the tracked offset">, Visible>                                                                                                  enabled     = false;
    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"stream rate, written into a passing tag">, Limits<0.f, std::numeric_limits<float>::max(), detail::kSampleRatePositive>> sample_rate = 96000.f;
    Annotated<double, "tau", Unit<"s">, Doc<"time constant of the estimate, corner 1/(2*pi*tau) Hz">, Limits<0.0, std::numeric_limits<double>::max(), detail::kTauPositive>> tau         = 1.0;
    Annotated<double, "alpha", Doc<"derived smoothing coefficient 1/(1 + tau*sample_rate)">>                                                                                 alpha       = 0.0;
    Annotated<double, "corner_hz", Unit<"Hz">, Doc<"derived highpass -3 dB point, alpha*sample_rate/(2*pi)">>                                                                corner_hz   = 0.0;

    GR_MAKE_REFLECTABLE(DcOffsetCorrect, in, out, enabled, sample_rate, tau, alpha, corner_hz);

    bool   _enabled       = enabled.value;     ///< the accepted `enabled`
    float  _sampleRate    = sample_rate.value; ///< the accepted `sample_rate`
    double _tau           = tau.value;         ///< the accepted `tau`
    double _alpha         = 0.0;
    double _oneMinusAlpha = 1.0;
    double _real          = 0.0;
    double _imag          = 0.0;

    /// Derives `alpha`, the pole `1 - alpha` and `corner_hz` from the default settings.
    explicit DcOffsetCorrect(property_map init = {}) : Block<DcOffsetCorrect<T>>(std::move(init)) { accept(_enabled, _sampleRate, _tau, 1.0 / (1.0 + _tau * static_cast<double>(_sampleRate))); }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const float  rate      = newSettings.contains("sample_rate") ? sample_rate.value : _sampleRate;
        const double seconds   = newSettings.contains("tau") ? tau.value : _tau;
        const double nextAlpha = 1.0 / (1.0 + seconds * static_cast<double>(rate));
        if (!(nextAlpha > 0.0) || nextAlpha > 1.0) {
            throw gr::exception(std::format("alpha must lie in (0, 1], got {} from tau {} and sample_rate {}", nextAlpha, seconds, rate));
        }
        const double nextCorner = cornerOf(nextAlpha, rate);
        if ((newSettings.contains("alpha") && alpha.value != nextAlpha) || (newSettings.contains("corner_hz") && corner_hz.value != nextCorner)) {
            throw gr::exception(std::format("alpha and corner_hz follow from tau and sample_rate, which give {} and {}", nextAlpha, nextCorner));
        }
        accept(newSettings.contains("enabled") ? enabled.value : _enabled, rate, seconds, nextAlpha);
    }

    void reset() {
        _real = 0.0;
        _imag = 0.0;
    }

    /// The tracked DC offset. Call it on the thread that runs the block, or while the block is stopped.
    [[nodiscard]] std::complex<double> dcEstimate() const noexcept { return {_real, _imag}; }

    /// Updates the estimate with @p sample and returns @p sample less the estimate, or @p sample unchanged while
    /// disabled. A component of the estimate below the smallest normal `double` is set to zero, and a silent input
    /// leaves the estimate at zero.
    [[nodiscard]] constexpr T processOne(T sample) noexcept {
        if (!_enabled) {
            return sample;
        }
        if constexpr (meta::complex_like<T>) {
            using Real            = typename T::value_type;
            const double real     = static_cast<double>(sample.real());
            const double imag     = static_cast<double>(sample.imag());
            const double nextReal = _alpha * real + _oneMinusAlpha * _real;
            const double nextImag = _alpha * imag + _oneMinusAlpha * _imag;
            if (std::isfinite(nextReal) && std::isfinite(nextImag)) {
                _real = flushed(nextReal);
                _imag = flushed(nextImag);
            }
            return T(static_cast<Real>(real - _real), static_cast<Real>(imag - _imag));
        } else {
            const double real     = static_cast<double>(sample);
            const double nextReal = _alpha * real + _oneMinusAlpha * _real;
            if (std::isfinite(nextReal)) {
                _real = flushed(nextReal);
            }
            return static_cast<T>(real - _real);
        }
    }

private:
    [[nodiscard]] static constexpr double flushed(double value) noexcept { return std::abs(value) < std::numeric_limits<double>::min() ? 0.0 : value; }

    [[nodiscard]] static constexpr double cornerOf(double nextAlpha, float rate) noexcept { return nextAlpha * static_cast<double>(rate) / (2.0 * std::numbers::pi); }

    void accept(bool on, float rate, double seconds, double nextAlpha) {
        _enabled       = on;
        _sampleRate    = rate;
        _tau           = seconds;
        _alpha         = nextAlpha;
        _oneMinusAlpha = 1.0 - nextAlpha;
        alpha          = nextAlpha;
        corner_hz      = cornerOf(nextAlpha, rate);
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_IQ_CORRECTION_HPP
