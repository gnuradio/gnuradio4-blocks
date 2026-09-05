#ifndef GNURADIO_CHANNEL_NONLINEARITY_HPP
#define GNURADIO_CHANNEL_NONLINEARITY_HPP

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

enum class NonlinearityModel : int {
    Rapp,  ///< solid-state amplifier: soft envelope limiting, no AM/PM
    Saleh, ///< traveling-wave tube: envelope compression with an amplitude-dependent phase shift
};

GR_REGISTER_BLOCK(gr::blocks::channel::Nonlinearity, [T], [std::complex<float>])

/**
 * @brief Memoryless power-amplifier nonlinearity, Rapp or Saleh.
 *
 * Rapp (SSPA), phase untouched:
 *
 *     A_out = A / (1 + (A/A_sat)^(2p))^(1/(2p))
 *
 * Saleh (TWTA), which compresses the envelope and rotates by an amount that depends on it:
 *
 *     AM/AM = alpha_a*A / (1 + beta_a*A^2)      AM/PM = alpha_p*A^2 / (1 + beta_p*A^2)   [rad]
 *
 * Both are closed forms, so the equation is the oracle: QA checks the block against it at a table of
 * amplitudes. `input_backoff_db` pre-scales the input so an operating point states itself.
 */
template<typename T>
requires std::is_same_v<T, std::complex<float>>
struct Nonlinearity : gr::Block<Nonlinearity<T>> {
    using Description = Doc<R""(
@brief Memoryless amplifier nonlinearity (Rapp SSPA or Saleh TWTA).

Acts on the envelope only, so it is stateless and chunk safe by construction. `saturation` is in the same
units as the input amplitude; `input_backoff_db` pre-scales so an operating point can be stated directly.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<NonlinearityModel, "model", Visible, Doc<"rapp (SSPA, no AM/PM) or saleh (TWTA, with AM/PM)">>         model            = NonlinearityModel::Rapp;
    Annotated<double, "input_backoff_db", Visible, Unit<"dB">, Doc<"input pre-scale; positive backs the drive off">> input_backoff_db = 0.0;
    Annotated<double, "saturation", Visible, Doc<"Rapp saturation amplitude A_sat">>                                 saturation       = 1.0;
    Annotated<double, "smoothness", Visible, Doc<"Rapp knee sharpness p; larger is a harder limit">>                 smoothness       = 2.0;
    Annotated<double, "saleh_alpha_a", Visible, Doc<"Saleh AM/AM numerator">>                                        saleh_alpha_a    = 2.1587;
    Annotated<double, "saleh_beta_a", Visible, Doc<"Saleh AM/AM denominator">>                                       saleh_beta_a     = 1.1517;
    Annotated<double, "saleh_alpha_p", Visible, Unit<"rad">, Doc<"Saleh AM/PM numerator">>                           saleh_alpha_p    = 4.0033;
    Annotated<double, "saleh_beta_p", Visible, Doc<"Saleh AM/PM denominator">>                                       saleh_beta_p     = 9.1040;

    GR_MAKE_REFLECTABLE(Nonlinearity, in, out, model, input_backoff_db, saturation, smoothness, saleh_alpha_a, saleh_beta_a, saleh_alpha_p, saleh_beta_p);

    double _drive = std::pow(10., -input_backoff_db.value / 20.); // derived from the member, so a block never called back still drives at its declared backoff

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (saturation <= 0.0) {
            throw gr::exception(std::format("Nonlinearity: 'saturation' must be positive, got {}", saturation.value));
        }
        if (smoothness <= 0.0) {
            throw gr::exception(std::format("Nonlinearity: 'smoothness' must be positive, got {}", smoothness.value));
        }
        _drive = std::pow(10., -input_backoff_db / 20.);
    }

    /// The model's own equation, exposed so QA and consumers read the same curve the block runs.
    [[nodiscard]] std::complex<double> transferAt(double amplitude) const noexcept {
        const double driven = amplitude * _drive;
        if (driven == 0.) {
            return {0., 0.};
        }
        if (model == NonlinearityModel::Rapp) {
            const double ratio = std::pow(driven / saturation, 2. * smoothness);
            return {driven / std::pow(1. + ratio, 1. / (2. * smoothness)), 0.};
        }
        const double squared = driven * driven;
        return {saleh_alpha_a * driven / (1. + saleh_beta_a * squared), saleh_alpha_p * squared / (1. + saleh_beta_p * squared)};
    }

    [[nodiscard]] T processOne(T sample) const noexcept {
        const double amplitude = std::hypot(static_cast<double>(sample.real()), static_cast<double>(sample.imag()));
        if (amplitude == 0.) {
            return T(0.f, 0.f); // a zero envelope has no phase to preserve, and passes through exactly
        }
        const std::complex<double> response = transferAt(amplitude);
        const double               gain     = response.real() / amplitude;
        if (response.imag() == 0.) {
            return T(static_cast<float>(static_cast<double>(sample.real()) * gain), static_cast<float>(static_cast<double>(sample.imag()) * gain));
        }
        const std::complex<double> rotated = std::polar(gain, response.imag()) * std::complex<double>(sample);
        return T(static_cast<float>(rotated.real()), static_cast<float>(rotated.imag()));
    }
};

} // namespace gr::blocks::channel

#endif // GNURADIO_CHANNEL_NONLINEARITY_HPP
