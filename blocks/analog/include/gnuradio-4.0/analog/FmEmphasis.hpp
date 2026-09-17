#ifndef GNURADIO_FM_EMPHASIS_HPP
#define GNURADIO_FM_EMPHASIS_HPP

#include <algorithm>
#include <cmath>
#include <concepts>
#include <limits>
#include <numbers>
#include <span>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::analog {

namespace detail {

[[nodiscard]] inline double prewarpedCorner(double frequency, double sampleRate) { return std::tan(std::numbers::pi * frequency / sampleRate); }

inline void checkEmphasisRate(double sampleRate) {
    if (!(sampleRate > 0.0) || !std::isfinite(sampleRate)) {
        throw gr::exception(std::format("sample_rate must be positive and finite, got {}", sampleRate));
    }
}

inline void checkEmphasisCorner(double frequency, double sampleRate, std::string_view name) {
    if (!(frequency > 0.0) || !(frequency < 0.5 * sampleRate)) {
        throw gr::exception(std::format("{} ({} Hz) must lie strictly inside (0, {} Hz)", name, frequency, 0.5 * sampleRate));
    }
}

struct EmphasisSection {
    double b0{};
    double b1{};
    double p{};
};

inline void checkEmphasisTau(double tau) {
    if (!(tau >= 0.0) || !std::isfinite(tau)) {
        throw gr::exception(std::format("tau must be zero or positive and finite, got {}", tau));
    }
}

/// @brief The section that passes its input through unchanged: `y[n] = x[n]`, at the same index and with no delay.
[[nodiscard]] inline constexpr EmphasisSection bypassSection() noexcept { return {1.0, 0.0, 0.0}; }

[[nodiscard]] inline EmphasisSection designDeemphasis(double sampleRate, double tau) {
    if (tau == 0.0) {
        checkEmphasisRate(sampleRate);
        return bypassSection(); // the limit of w_c = 1/tau going to infinity, which is the passthrough it looks like
    }
    checkEmphasisRate(sampleRate);
    checkEmphasisTau(tau);

    const double corner = 1.0 / (2.0 * std::numbers::pi * tau);
    checkEmphasisCorner(corner, sampleRate, "the corner derived from tau");

    const double k = prewarpedCorner(corner, sampleRate);
    return {k / (1.0 + k), k / (1.0 + k), (1.0 - k) / (1.0 + k)};
}

[[nodiscard]] inline EmphasisSection designPreemphasis(double sampleRate, double tau, double highCorner) {
    if (tau == 0.0) {
        // Bypass by definition, not by limit. Taking tau to zero here sends the low corner to infinity, which drives
        // b0 and b1 to zero and the filter to silence rather than to passthrough — the opposite of what it does on
        // the de-emphasis side. The pair is meant to be cascaded at one tau, so the spelling is made to mean the same
        // thing on both halves deliberately. Do not "correct" this toward the analytic limit.
        checkEmphasisRate(sampleRate);
        return bypassSection();
    }
    checkEmphasisRate(sampleRate);
    checkEmphasisTau(tau);

    const double lowCorner = 1.0 / (2.0 * std::numbers::pi * tau);
    checkEmphasisCorner(lowCorner, sampleRate, "the corner derived from tau");

    const double effectiveTop = (highCorner > 0.0 && highCorner < 0.5 * sampleRate) ? highCorner : 0.925 * 0.5 * sampleRate;
    checkEmphasisCorner(effectiveTop, sampleRate, "high_corner");

    const double kl = prewarpedCorner(lowCorner, sampleRate);
    const double kh = prewarpedCorner(effectiveTop, sampleRate);
    return {(kh / kl) * (1.0 + kl) / (1.0 + kh), -(kh / kl) * (1.0 - kl) / (1.0 + kh), (1.0 - kh) / (1.0 + kh)};
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::analog::FmDeemphasis, [T], [float])

template<std::floating_point T>
struct FmDeemphasis : Block<FmDeemphasis<T>, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief FM de-emphasis: the receive half of the emphasis pair, a single-pole RC lowpass.

Bilinear transform of `H(s) = w_c/(s + w_c)` with the corner `f_c = 1/(2*pi*tau)` prewarped, so the digital corner lands
exactly at `f_c`: with `k = tan(pi*f_c/sample_rate)` the recursion is `y[n] = b0*x[n] + b1*x[n-1] + p*y[n-1]` with
`b0 = b1 = k/(1+k)` and `p = (1-k)/(1+k)`, exactly unity at DC and `1/sqrt(2)` at `f_c`.

`tau` is in seconds: `75 us` in the Americas and South Korea, `50 us` elsewhere. **`tau = 0` is a bypass**, passing
the stream through bit for bit at the same sample index — the block adds no delay either way, so switching in and out
of it moves nothing in time. On this half that is also the analytic limit, the corner `1/(2*pi*tau)` running off to
infinity; `FmPreemphasis` spells it the same way by definition rather than by limit, so a chain can carry one `tau`
and turn the pair off together. A `sample_rate` tag retunes the filter,
and a settings change recomputes the coefficients while keeping the delay elements, so retuning a live audio stream
produces a short transient rather than a click. A non-positive rate or `tau`, or a corner outside `(0, sample_rate/2)`,
throws at settings time.

Prewarping places the corner exactly but leaves the response between DC and Nyquist warped relative to the analog
prototype the transmitter used: at `tau = 75 us` the attenuation at 15 kHz is -20.63 dB at 48 kHz against -17.07 dB for
the analog curve.

The block is 1:1, so every input tag key passes through at its own offset, `sample_rate` carrying this block's value.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"rate of the audio stream">> sample_rate = 48000.f;
    Annotated<double, "tau", Unit<"s">, Doc<"emphasis time constant">>           tau         = 75e-6;

    detail::EmphasisSection _section = detail::designDeemphasis(static_cast<double>(sample_rate), tau);
    T                       _lastInput{};
    T                       _lastOutput{};

    GR_MAKE_REFLECTABLE(FmDeemphasis, in, out, sample_rate, tau);

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { _section = detail::designDeemphasis(static_cast<double>(sample_rate), tau); }

    void reset() {
        _lastInput  = T{};
        _lastOutput = T{};
    }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        const T b0 = static_cast<T>(_section.b0);
        const T b1 = static_cast<T>(_section.b1);
        const T p  = static_cast<T>(_section.p);

        T lastInput  = _lastInput;
        T lastOutput = _lastOutput;
        for (std::size_t i = 0UZ; i < std::min(input.size(), output.size()); ++i) {
            const T sample = input[i];
            lastOutput     = b0 * sample + b1 * lastInput + p * lastOutput;
            if (std::abs(lastOutput) < std::numeric_limits<T>::min()) {
                lastOutput = T{}; // a pole below one decays a silent input into subnormals and stays there; each one costs a microcode assist
            }
            lastInput = sample;
            output[i] = lastOutput;
        }
        _lastInput  = lastInput;
        _lastOutput = lastOutput;

        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::analog::FmPreemphasis, [T], [float])

template<std::floating_point T>
struct FmPreemphasis : Block<FmPreemphasis<T>, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief FM pre-emphasis: the transmit half of the emphasis pair, a 6 dB/octave rise that flattens at a high corner.

Bilinear transform of `H(s) = (s + w_l)/(s + w_h)` with both corners prewarped, normalized to unity at DC. With
`kl = tan(pi*f_cl/sample_rate)`, `kh = tan(pi*high_corner/sample_rate)` and `f_cl = 1/(2*pi*tau)` the recursion is
`y[n] = b0*x[n] + b1*x[n-1] + p*y[n-1]` with `b0 = (kh/kl)*(1+kl)/(1+kh)`, `b1 = -(kh/kl)*(1-kl)/(1+kh)` and
`p = (1-kh)/(1+kh)`.

**`tau = 0` is a bypass**, matching `FmDeemphasis` so one parameter turns the pair off together; the block adds no
delay either way, so switching in and out of it moves nothing in time. It is a convention here rather than a limit:
taking `tau` to zero would send the low corner to infinity and the filter to silence, not to passthrough.

`tau` is in seconds and `high_corner`, in hertz, is where the rise flattens; `0.0`, or any value outside
`(0, sample_rate/2)`, selects `0.925*sample_rate/2`, because a corner at `0` or Nyquist would place the pole on the
unit circle. Cascading this block with `FmDeemphasis` at the same `sample_rate` and `tau` is exactly a single-pole lowpass
with prewarped corner `high_corner`, because the pre-emphasis zero cancels the de-emphasis pole.

The block is 1:1, so every input tag key passes through at its own offset, `sample_rate` carrying this block's value.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"rate of the audio stream">>                                sample_rate = 48000.f;
    Annotated<double, "tau", Unit<"s">, Doc<"emphasis time constant">>                                          tau         = 75e-6;
    Annotated<double, "high_corner", Unit<"Hz">, Doc<"where the rise flattens; 0 selects 0.925*sample_rate/2">> high_corner = 0.0;

    detail::EmphasisSection _section = detail::designPreemphasis(static_cast<double>(sample_rate), tau, high_corner);
    T                       _lastInput{};
    T                       _lastOutput{};

    GR_MAKE_REFLECTABLE(FmPreemphasis, in, out, sample_rate, tau, high_corner);

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { _section = detail::designPreemphasis(static_cast<double>(sample_rate), tau, high_corner); }

    void reset() {
        _lastInput  = T{};
        _lastOutput = T{};
    }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        const T b0 = static_cast<T>(_section.b0);
        const T b1 = static_cast<T>(_section.b1);
        const T p  = static_cast<T>(_section.p);

        T lastInput  = _lastInput;
        T lastOutput = _lastOutput;
        for (std::size_t i = 0UZ; i < std::min(input.size(), output.size()); ++i) {
            const T sample = input[i];
            lastOutput     = b0 * sample + b1 * lastInput + p * lastOutput;
            if (std::abs(lastOutput) < std::numeric_limits<T>::min()) {
                lastOutput = T{}; // a pole below one decays a silent input into subnormals and stays there; each one costs a microcode assist
            }
            lastInput = sample;
            output[i] = lastOutput;
        }
        _lastInput  = lastInput;
        _lastOutput = lastOutput;

        return work::Status::OK;
    }
};

} // namespace gr::blocks::analog

#endif // GNURADIO_FM_EMPHASIS_HPP
