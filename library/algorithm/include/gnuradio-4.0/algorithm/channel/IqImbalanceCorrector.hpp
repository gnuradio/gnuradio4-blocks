#ifndef GNURADIO_ALGORITHM_CHANNEL_IQ_IMBALANCE_CORRECTOR_HPP
#define GNURADIO_ALGORITHM_CHANNEL_IQ_IMBALANCE_CORRECTOR_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <limits>
#include <numbers>
#include <span>
#include <tuple>
#include <utility>

namespace gr::channel {

/**
 * @brief Estimates the gain and phase imbalance between the I and Q arms of a complex stream from the stream itself,
 * and corrects it.
 *
 * The model is a receiver whose Q arm gives `Q = g * (Q0 * cos(phi) - I0 * sin(phi))` for an I arm `I = I0`, where `I0`
 * and `Q0` have equal power and no correlation. With `P_I = E[I^2]`, `P_Q = E[Q^2]` and `C = E[I*Q]`, the correction
 * writes `I' = I` and `Q' = k * (Q - c * I)`, with `c = C/P_I` and `k = sqrt(P_I/(P_Q - C^2/P_I))`: two components of
 * equal power and no correlation. Only an imbalance that does not depend on frequency is corrected.
 *
 * The averages come from the first difference of the stream, `x[n] - x[n-1]`. The difference carries the same
 * imbalance and no DC. It weighs content at frequency `f` by `4 * sin^2(pi * f / fs)`: a signal close to zero
 * frequency, such as a carrier or an AM station tuned to the center, barely enters the estimate. Over each run of
 * `kRun` samples the kernel sums the squares and the product of the difference. At the run's end the averages move
 * toward the run's means by `1 - exp(-kRun / tau)`, with `tau` the time constant in samples, and the coefficients
 * follow. Each run is corrected with the coefficients of the runs before it. Until the first run ends, the output
 * equals the input. The averages start at zero. The coefficients are ratios, and the first run already sets them.
 * Each `process()` call returns the number of runs it completed.
 *
 * The correction reaches at most 3 dB of gain and 10 degrees of phase, and a larger estimate is corrected to that
 * limit. `gainDb()` and `phaseDeg()` return the estimate itself. A run with a NaN or infinite sample leaves the
 * averages and the coefficients unchanged. A run of silence, or one with an arm silent, leaves the averages and the
 * coefficients unchanged.
 *
 * The estimate assumes circular content, as noise and most band content are. A strong real-valued signal at the
 * center whose difference spreads across the band, such as a fast BPSK or one with unfiltered symbol steps, biases
 * it. The correction then adds an image of its own. The limit bounds that image: on a receiver with no imbalance of
 * its own it stays at or below -14.3 dBc.
 */
template<std::floating_point R = float>
class IqImbalanceCorrector {
public:
    using Sample = std::complex<R>;

    static constexpr std::size_t kRun = 1024UZ; ///< samples per update of the estimate

    IqImbalanceCorrector() noexcept { std::ignore = setTimeConstant(1.0e5); }

    /// Sets the time constant of the averages in samples and keeps the estimate. Returns false and changes nothing for
    /// a value that gives a run no weight: zero, negative, infinite or NaN.
    [[nodiscard]] bool setTimeConstant(double samples) noexcept {
        const double step = -std::expm1(-static_cast<double>(kRun) / samples);
        if (!(samples > 0.0) || !(step > 0.0)) {
            return false;
        }
        _timeConstant = samples;
        _step         = step;
        return true;
    }

    [[nodiscard]] double timeConstant() const noexcept { return _timeConstant; }

    /// Writes @p in, corrected, to @p out, which holds at least as many samples and does not overlap @p in, and adds
    /// the stream to the estimate.
    std::size_t process(std::span<const Sample> in, std::span<Sample> out) noexcept {
        return run<false>(in, out, [](std::span<const Sample>, std::span<Sample>) {});
    }

    /// Corrects the samples @p prepare writes and adds @p in to the estimate. For each segment of the call,
    /// `prepare(inSegment, outSegment)` writes the samples to correct into `outSegment`, and the correction then
    /// rewrites them in place while they are in cache: a step such as DC removal runs between the estimate's input and
    /// the correction. @p out holds at least as many samples as @p in and does not overlap it.
    template<typename Prepare>
    std::size_t process(std::span<const Sample> in, std::span<Sample> out, Prepare&& prepare) {
        return run<true>(in, out, std::forward<Prepare>(prepare));
    }

    /// Marks a gap in the stream: the next sample is not differenced against the last one seen.
    void markDiscontinuity() noexcept { _primed = false; }

    /// Sets the averages to zero, the correction to none and the run to its start.
    void reset() noexcept {
        _at     = 0UZ;
        _sum    = {};
        _moment = {};
        _scale  = R(1);
        _shear  = R(0);
        _primed = false;
    }

    /// The gain of Q against I, `10 * log10(P_Q / P_I)`, zero before the first estimate.
    [[nodiscard]] double gainDb() const noexcept {
        const auto [powerI, powerQ, product] = _moment;
        return powerI > 0.0 && powerQ > 0.0 ? 10.0 * std::log10(powerQ / powerI) : 0.0;
    }

    /// The quadrature error `phi` in degrees, `asin(-C / (sqrt(P_I) * sqrt(P_Q)))`, zero before the first estimate.
    [[nodiscard]] double phaseDeg() const noexcept {
        const auto [powerI, powerQ, product] = _moment;
        if (!(powerI > 0.0 && powerQ > 0.0)) {
            return 0.0;
        }
        return std::asin(std::clamp(-product / (std::sqrt(powerI) * std::sqrt(powerQ)), -1.0, 1.0)) * 180.0 / std::numbers::pi;
    }

    /// The coefficients the next sample is corrected with: `Q' = scale * Q + shear * I`, where `scale` is `k` and
    /// `shear` is `-k * c`.
    [[nodiscard]] R scale() const noexcept { return _scale; }
    [[nodiscard]] R shear() const noexcept { return _shear; }

private:
    static constexpr std::size_t kLanes = 8UZ;

    static constexpr double kGainLimit = 1.9952623149688795;  // 10^(3/10): the power ratio of 3 dB
    static constexpr double kSineLimit = 0.17364817766693033; // sin(10 degrees)

    double                  _timeConstant = 0.0;
    double                  _step         = 0.0; ///< 1 - exp(-kRun / tau), the weight of a run
    std::size_t             _at           = 0UZ; ///< samples into the current run
    std::array<double, 3UZ> _sum{};              ///< the current run's sums of dI^2, dQ^2 and dI*dQ
    std::array<double, 3UZ> _moment{};           ///< the averages P_I, P_Q and C
    R                       _scale = R(1);
    R                       _shear = R(0);
    std::array<R, 2UZ>      _last{};         ///< the last sample seen
    bool                    _primed = false; ///< `_last` holds a sample to difference against

    template<bool kPrepared, typename Prepare>
    std::size_t run(std::span<const Sample> in, std::span<Sample> out, [[maybe_unused]] Prepare&& prepare) {
        std::size_t updates = 0UZ;
        for (std::size_t start = 0UZ; start < in.size();) {
            const std::size_t count = std::min(kRun - _at, in.size() - start);
            if constexpr (kPrepared) {
                prepare(in.subspan(start, count), out.subspan(start, count));
            }
            correct<kPrepared>(static_cast<const R*>(static_cast<const void*>(in.data() + start)), static_cast<R*>(static_cast<void*>(out.data() + start)), count);
            start += count;
            _at += count;
            if (_at == kRun) {
                update();
                ++updates;
            }
        }
        return updates;
    }

    /// Writes `count` interleaved samples corrected with the held coefficients to `y`: the samples `x` with `kInPlace`
    /// false, and the samples `y` holds on entry with it true. Adds the squares and the product of the first
    /// differences of `x` to the run's sums, in eight lanes each, added in double.
    template<bool kInPlace>
    void correct(const R* x, R* y, std::size_t count) noexcept {
        const std::size_t size = count * 2UZ;
        if (size == 0UZ) {
            return;
        }
        const R               scale = _scale;
        const R               shear = _shear;
        std::array<R, kLanes> squareI{};
        std::array<R, kLanes> squareQ{};
        std::array<R, kLanes> cross{};
        const auto            one = [&](std::size_t at, R lastI, R lastQ) {
            const R diffI = x[at] - lastI;
            const R diffQ = x[at + 1UZ] - lastQ;
            squareI[0] += diffI * diffI;
            squareQ[0] += diffQ * diffQ;
            cross[0] += diffI * diffQ;
            const R baseI = kInPlace ? y[at] : x[at];
            const R baseQ = kInPlace ? y[at + 1UZ] : x[at + 1UZ];
            y[at]         = baseI;
            y[at + 1UZ]   = scale * baseQ + shear * baseI;
        };
        one(0UZ, _primed ? _last[0] : x[0], _primed ? _last[1] : x[1]); // the first sample after a gap adds nothing
        std::size_t at = 2UZ;
        for (; at + 2UZ * kLanes <= size; at += 2UZ * kLanes) {
            std::array<R, kLanes> valueI;
            std::array<R, kLanes> valueQ;
            std::array<R, kLanes> lastI;
            std::array<R, kLanes> lastQ;
            std::array<R, kLanes> baseI;
            std::array<R, kLanes> baseQ;
            for (std::size_t k = 0UZ; k < kLanes; ++k) {
                valueI[k] = x[at + 2UZ * k];
                valueQ[k] = x[at + 2UZ * k + 1UZ];
                lastI[k]  = x[at + 2UZ * k - 2UZ];
                lastQ[k]  = x[at + 2UZ * k - 1UZ];
                baseI[k]  = kInPlace ? y[at + 2UZ * k] : valueI[k];
                baseQ[k]  = kInPlace ? y[at + 2UZ * k + 1UZ] : valueQ[k];
            }
            for (std::size_t k = 0UZ; k < kLanes; ++k) {
                const R diffI = valueI[k] - lastI[k];
                const R diffQ = valueQ[k] - lastQ[k];
                squareI[k] += diffI * diffI;
                squareQ[k] += diffQ * diffQ;
                cross[k] += diffI * diffQ;
                y[at + 2UZ * k]       = baseI[k];
                y[at + 2UZ * k + 1UZ] = scale * baseQ[k] + shear * baseI[k];
            }
        }
        for (; at < size; at += 2UZ) {
            one(at, x[at - 2UZ], x[at - 1UZ]);
        }
        for (std::size_t k = 0UZ; k < kLanes; ++k) {
            _sum[0] += static_cast<double>(squareI[k]);
            _sum[1] += static_cast<double>(squareQ[k]);
            _sum[2] += static_cast<double>(cross[k]);
        }
        _last   = {x[size - 2UZ], x[size - 1UZ]};
        _primed = true;
    }

    /// Moves the averages one run toward the run's means and derives the coefficients, the gain ratio held within
    /// `kGainLimit` and the phase's sine within `kSineLimit`.
    void update() noexcept {
        const std::array<double, 3UZ> sum = _sum;
        _sum                              = {};
        _at                               = 0UZ;
        if (!std::isfinite(sum[0] + sum[1] + sum[2])) {
            return;
        }
        // a silent arm carries no imbalance, and decaying the averages through a long silence would take them into
        // the subnormal range, where their ratios lose all precision
        if (!(sum[0] > 0.0) || !(sum[1] > 0.0)) {
            return;
        }
        for (std::size_t m = 0UZ; m < sum.size(); ++m) {
            _moment[m] = flushed(_moment[m] + _step * (sum[m] / static_cast<double>(kRun) - _moment[m]));
        }
        const auto [powerI, powerQ, product] = _moment;
        if (!(powerI > 0.0) || !(powerQ > 0.0)) {
            return;
        }
        const double ratio = std::clamp(powerQ / powerI, 1.0 / kGainLimit, kGainLimit);
        const double sine  = std::clamp(product / (std::sqrt(powerI) * std::sqrt(powerQ)), -kSineLimit, kSineLimit);
        const double scale = 1.0 / std::sqrt(ratio * (1.0 - sine * sine));
        _scale             = static_cast<R>(scale);
        _shear             = static_cast<R>(-scale * sine * std::sqrt(ratio));
    }

    [[nodiscard]] static constexpr double flushed(double value) noexcept { return std::abs(value) < std::numeric_limits<double>::min() ? 0.0 : value; }
};

} // namespace gr::channel

#endif // GNURADIO_ALGORITHM_CHANNEL_IQ_IMBALANCE_CORRECTOR_HPP
