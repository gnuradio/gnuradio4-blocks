#ifndef GNURADIO_CHANNEL_CARRIER_IMPAIRMENTS_HPP
#define GNURADIO_CHANNEL_CARRIER_IMPAIRMENTS_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/algorithm/signal/NoiseGenerator.hpp>
#include <gnuradio-4.0/algorithm/signal/Phasor.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::channel {

GR_REGISTER_BLOCK(gr::blocks::channel::FrequencyOffset, [T], [std::complex<float>])

/**
 * @brief Carrier frequency offset, a fixed phase offset, and an optional linear frequency drift.
 *
 * `out[k] = in[k] * exp(j*(2*pi*(f0*t_k + rate*t_k^2/2) + phi0))` with `t_k = k/fs` counted from the start
 * of the stream, not of the call.
 *
 * This block models an impairment, so the stream's nominal center frequency is unchanged and a passing
 * `gr::tag::FREQUENCY` is forwarded untouched. `Rotator` retunes that tag instead, because rotating on purpose
 * moves what the stream is centered on; the two blocks differ in exactly that contract.
 */
template<typename T>
requires std::is_same_v<T, std::complex<float>>
struct FrequencyOffset : gr::Block<FrequencyOffset<T>> {
    using Description = Doc<R""(
@brief Carrier frequency offset with optional linear drift.

Multiplies by `exp(j*(2*pi*(f0*t + rate*t^2/2) + phi0))`. A passing `frequency` tag is forwarded unchanged:
the impairment does not move what the stream is centered on, it moves the signal within it.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"stream sample rate">>                           sample_rate      = 1.f;
    Annotated<double, "frequency_offset", Visible, Unit<"Hz">, Doc<"constant carrier offset">>                frequency_offset = 0.0;
    Annotated<double, "drift", Visible, Unit<"Hz/s">, Doc<"linear frequency drift, 0 for none">>              drift            = 0.0;
    Annotated<double, "phase_offset", Visible, Unit<"rad">, Doc<"constant phase offset at the stream start">> phase_offset     = 0.0;

    GR_MAKE_REFLECTABLE(FrequencyOffset, in, out, sample_rate, frequency_offset, drift, phase_offset);

    gr::signal::Phasor<float> _phasor{};
    std::uint64_t             _position{0ULL};  ///< stream-absolute sample index, the drift's time base
    bool                      _chirping{false}; ///< the drift schedule has been handed to the phasor

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (sample_rate <= 0.f) {
            throw gr::exception(std::format("FrequencyOffset: 'sample_rate' must be positive, got {}", sample_rate.value));
        }
        _phasor.setIncrement(2. * std::numbers::pi_v<double> * frequency_offset / static_cast<double>(sample_rate));
        _chirping = false;
        if (newSettings.contains("phase_offset")) {
            _phasor.setPhase(phase_offset);
            _position = 0ULL;
        }
    }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        const std::size_t  nSamples = std::min(input.size(), output.size());
        const std::span<T> written  = output.first(nSamples);

        if (drift == 0.0) {
            _phasor.mix(input.first(nSamples), written);
        } else {
            // Exact per-sample increment for a linearly swept frequency: integrating 2*pi*(f0 + rate*t) over one
            // sample is the midpoint rule, which is exact for a linear integrand. That schedule is arithmetic -
            // inc(k) = inc(0) + k*step - so it is a chirp, and the phasor advances it by multiplication rather
            // than evaluating a sine and a cosine per sample.
            const double fs    = static_cast<double>(sample_rate);
            const double twoPi = 2. * std::numbers::pi_v<double>;
            const double step  = twoPi * drift / (fs * fs);
            if (!_chirping) {
                _phasor.setIncrement(twoPi * frequency_offset / fs + 0.5 * step + static_cast<double>(_position) * step);
                _chirping = true;
            }
            _phasor.mixChirp(step, input.first(nSamples), written);
        }

        _position += nSamples;
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::channel::PhaseNoise, [T], [std::complex<float>])

/**
 * @brief Oscillator phase noise as a Wiener process: `out[k] = in[k] * exp(j*theta_k)`.
 *
 * `theta_{k+1} = theta_k + w_k` with `w_k ~ N(0, sigma^2)` and `sigma^2 = 2*pi*linewidth/fs` — the random
 * walk whose power spectrum is a Lorentzian of full width at half maximum `linewidth`, with a skirt falling
 * at 20 dB/decade. Shaped spectra — flicker, noise floors, multi-pole skirts — wait for a consumer.
 *
 * A rotation cannot change magnitude, so the block preserves the signal's power exactly.
 */
template<typename T>
requires std::is_same_v<T, std::complex<float>>
struct PhaseNoise : gr::Block<PhaseNoise<T>> {
    using Description = Doc<R""(
@brief Wiener (random-walk) oscillator phase noise.

Multiplies by `exp(j*theta)` where theta random-walks with per-sample variance `2*pi*linewidth/fs`, giving a
Lorentzian line of the stated width. Deterministic from `seed`; `seed` is a staged-restart setting.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"stream sample rate">>                                          sample_rate = 1.f;
    Annotated<double, "linewidth", Visible, Unit<"Hz">, Doc<"Lorentzian full width at half maximum; 0 disables">>            linewidth   = 0.0;
    Annotated<std::uint64_t, "seed", Visible, Doc<"PRNG seed; 0 is a fixed default. Staged restart: refused while running">> seed        = 0ULL;

    GR_MAKE_REFLECTABLE(PhaseNoise, in, out, sample_rate, linewidth, seed);

    gr::signal::NoiseGenerator<float> _generator{};
    double                            _phase{0.};
    double                            _sigma{0.};
    bool                              _running{false};
    std::vector<float>                _steps{};

    void start() {
        _generator.configure(gr::signal::NoiseType::Gaussian, 1.f, 0.f, seed.value);
        _running = true;
    }

    void stop() { _running = false; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (sample_rate <= 0.f) {
            throw gr::exception(std::format("PhaseNoise: 'sample_rate' must be positive, got {}", sample_rate.value));
        }
        if (linewidth < 0.0) {
            throw gr::exception(std::format("PhaseNoise: 'linewidth' must be non-negative, got {}", linewidth.value));
        }
        if (_running && newSettings.contains("seed")) {
            throw gr::exception("PhaseNoise: 'seed' is a staged-restart setting — a mid-stream reseed is a new realization, not a parameter change. Stop the graph to restart it.");
        }
        _sigma = std::sqrt(2. * std::numbers::pi_v<double> * linewidth / static_cast<double>(sample_rate));
        if (!_running) {
            _generator.configure(gr::signal::NoiseType::Gaussian, 1.f, 0.f, seed.value);
        }
    }

    // not noexcept: the step buffer is sized to the call's span, and an allocation that throws is not a terminate
    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) {
        const std::size_t  nSamples = std::min(input.size(), output.size());
        const std::span<T> written  = output.first(nSamples);

        if (linewidth == 0.0) {
            // with no walk there is nothing to draw, and a copy keeps the samples bit-exact
            std::ranges::copy(input.first(nSamples), written.begin());
            return work::Status::OK;
        }

        _steps.resize(nSamples);
        _generator.fill(std::span<float>(_steps));

        constexpr double twoPi = 2. * std::numbers::pi_v<double>;
        for (std::size_t k = 0UZ; k < nSamples; ++k) {
            const float phRe = static_cast<float>(std::cos(_phase));
            const float phIm = static_cast<float>(std::sin(_phase));
            const float re   = input[k].real();
            const float im   = input[k].imag();
            written[k]       = T(re * phRe - im * phIm, re * phIm + im * phRe);

            _phase += _sigma * static_cast<double>(_steps[k]);
            // reduce every step so the walk's state is a pure function of the previous phase and one draw,
            // which is what keeps a split stream bit-identical
            if (_phase > std::numbers::pi_v<double> || _phase < -std::numbers::pi_v<double>) {
                _phase = std::remainder(_phase, twoPi);
            }
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::channel::IqImbalance, [T], [std::complex<float>])

/**
 * @brief Quadrature amplitude and phase imbalance: `out[k] = alpha*in[k] + beta*conj(in[k])`.
 *
 * With `g = 10^(amplitude_imbalance_db/20)` and `phi = phase_imbalance`:
 *
 *     alpha = (1 + g*exp(j*phi)) / 2      beta = (1 - g*exp(j*phi)) / 2
 *
 * `g = 1, phi = 0` gives `alpha = 1, beta = 0` — exact passthrough.
 *
 * One block serves both the transmitter and the receiver seat. Both are the same linear map of `(x, conj x)`;
 * only the interpretation of the parameters differs, and the block's position in the graph states which is
 * meant. The image-rejection ratio `|alpha|^2 / |beta|^2` is closed form, which makes the model its own
 * check: `imageRejectionDb()` reports what the settings imply and QA measures the spectrum against it.
 */
template<typename T>
requires std::is_same_v<T, std::complex<float>>
struct IqImbalance : gr::Block<IqImbalance<T>> {
    using Description = Doc<R""(
@brief Quadrature amplitude/phase imbalance, one block for the transmitter and receiver seats alike.

`out = alpha*in + beta*conj(in)`, the standard one-image model. Stateless, so it is trivially chunk safe, and
its image-rejection ratio is closed form.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<double, "amplitude_imbalance_db", Visible, Unit<"dB">, Doc<"gain difference between the I and Q arms">> amplitude_imbalance_db = 0.0;
    Annotated<double, "phase_imbalance", Visible, Unit<"rad">, Doc<"quadrature phase error">>                         phase_imbalance        = 0.0;

    GR_MAKE_REFLECTABLE(IqImbalance, in, out, amplitude_imbalance_db, phase_imbalance);

    std::complex<float> _alpha{1.f, 0.f};
    std::complex<float> _beta{0.f, 0.f};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        const double               gain    = std::pow(10., amplitude_imbalance_db / 20.);
        const std::complex<double> rotated = std::polar(gain, phase_imbalance.value);
        _alpha                             = static_cast<std::complex<float>>((1. + rotated) * 0.5);
        _beta                              = static_cast<std::complex<float>>((1. - rotated) * 0.5);
    }

    /// The image-rejection ratio the current settings imply, in dB; infinite at the identity.
    [[nodiscard]] double imageRejectionDb() const noexcept {
        const double leak = static_cast<double>(std::norm(_beta));
        return leak == 0. ? std::numeric_limits<double>::infinity() : 10. * std::log10(static_cast<double>(std::norm(_alpha)) / leak);
    }

    [[nodiscard]] constexpr T processOne(T sample) const noexcept {
        const float re = sample.real();
        const float im = sample.imag();
        // alpha*x + beta*conj(x), written out so the conjugate costs a sign rather than a temporary
        return T(_alpha.real() * re - _alpha.imag() * im + _beta.real() * re + _beta.imag() * im, //
            _alpha.real() * im + _alpha.imag() * re + _beta.imag() * re - _beta.real() * im);
    }
};

} // namespace gr::blocks::channel

#endif // GNURADIO_CHANNEL_CARRIER_IMPAIRMENTS_HPP
