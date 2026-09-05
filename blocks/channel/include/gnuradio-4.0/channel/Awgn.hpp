#ifndef GNURADIO_CHANNEL_AWGN_HPP
#define GNURADIO_CHANNEL_AWGN_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <span>
#include <type_traits>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/algorithm/channel/NoisePower.hpp>
#include <gnuradio-4.0/algorithm/signal/NoiseGenerator.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::channel {

GR_REGISTER_BLOCK(gr::blocks::channel::AwgnChannel, [T], [ float, std::complex<float> ])

/**
 * @brief Adds white Gaussian noise of a stated power: `out[k] = in[k] + sqrt(noise_power) * n[k]`.
 *
 * Real `T` draws `n ~ N(0,1)`; complex `T` draws `nI` and `nQ ~ N(0, 1/2)`, so `E[|n|^2] = 1` before
 * scaling. Either way `noise_power` is the mean power of what is added, and means the same thing for
 * both sample types.
 *
 * Parameterizing by power is what keeps the model deterministic from its seed alone, the property every
 * model in this family rests on. A graph wanting an Es/N0 computes the power from what it transmitted, with
 * `gr::channel::noisePowerFor()`.
 *
 * Deterministic from `seed` and independent of how the scheduler chunks the stream: the generator's state
 * carries across calls and nothing is derived from a span's length. `seed` is a staged-restart parameter —
 * changing it on a running block is refused, because a reseed is a new realization rather than a tweak.
 */
template<typename T>
requires(std::is_same_v<T, float> || std::is_same_v<T, std::complex<float>>)
struct AwgnChannel : gr::Block<AwgnChannel<T>> {
    using Description = Doc<R""(
@brief Additive white Gaussian noise of a stated power.

`out[k] = in[k] + sqrt(noise_power) * n[k]`, with `noise_power` the mean power of the added noise. Complex
noise uses the unit-mean-square convention, so the setting means the same for real and complex streams.
Deterministic from `seed`, and bit-identical however the stream is split into calls.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<double, "noise_power", Visible, Doc<"mean power of the added noise, linear per sample">>                       noise_power = 0.0;
    Annotated<std::uint64_t, "seed", Visible, Doc<"PRNG seed; 0 is a fixed default. Staged restart: refused while running">> seed        = 0ULL;

    GR_MAKE_REFLECTABLE(AwgnChannel, in, out, noise_power, seed);

    gr::signal::NoiseGenerator<float> _generator{};
    bool                              _running{false};

    [[nodiscard]] float amplitude() const noexcept { return static_cast<float>(std::sqrt(noise_power.value)); }

    void start() {
        _generator.configure(gr::signal::NoiseType::Gaussian, 1.f, 0.f, seed.value);
        _running = true;
    }

    void stop() { _running = false; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (noise_power < 0.0) {
            throw gr::exception(std::format("AwgnChannel: 'noise_power' must be non-negative, got {}", noise_power.value));
        }
        if (_running && newSettings.contains("seed")) {
            throw gr::exception("AwgnChannel: 'seed' is a staged-restart setting — a mid-stream reseed is a new realization, not a parameter change. Stop the graph to restart it.");
        }
        // The generator draws at unit amplitude and processBulk scales what it drew, so noise_power stays
        // live-settable without the reconfigure that would reseed the generator and start a new realization.
        if (!_running) {
            _generator.configure(gr::signal::NoiseType::Gaussian, 1.f, 0.f, seed.value);
        }
    }

    [[nodiscard]] constexpr work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        const std::size_t  nSamples = std::min(input.size(), output.size());
        const std::span<T> written  = output.first(nSamples);

        if (noise_power == 0.0) {
            // a copy preserves negative zero exactly, and with no noise there is no draw to make, so the
            // generator's state stands
            std::ranges::copy(input.first(nSamples), written.begin());
            return work::Status::OK;
        }

        const float scale = amplitude();
        if constexpr (std::is_same_v<T, std::complex<float>>) {
            _generator.fillComplex(written);
        } else {
            _generator.fill(written);
        }
        for (std::size_t k = 0UZ; k < nSamples; ++k) {
            written[k] = input[k] + scale * written[k];
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::channel

#endif // GNURADIO_CHANNEL_AWGN_HPP
