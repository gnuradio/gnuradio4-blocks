#ifndef GNURADIO_ALGORITHM_PHASOR_HPP
#define GNURADIO_ALGORITHM_PHASOR_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <numbers>
#include <span>
#include <stdexcept>

#include <gnuradio-4.0/algorithm/signal/LanePhasor.hpp>

namespace gr::signal {

/**
 * @brief Stateful unit phasor: generates, or mixes into a stream, exp(j*phase) at a fixed or a per-sample
 * phase increment.
 *
 * Fixed-increment generation (fill/mix) is a LanePhasor: kLanes interleaved phasors, each advanced by
 * (e^{j*increment})^kLanes, so the sample loop carries no dependency between neighbors and vectorizes. Every
 * kReseedInterval samples the lane vector is seeded again from an exact phase, which bounds the recurrence's
 * error by the seed rounding instead of letting it walk: the phase error at any output sample does not grow with
 * stream length, which is the kernel's accuracy contract.
 *
 * The output depends on the absolute sample index since the last configure(), setPhase(), setIncrement() or
 * advance(), so a stream split into calls anywhere is as exact as one call. Where the compiler contracts to fused
 * multiply-add the last bits can differ by where calls cut. Those four re-anchor the grid at the current phase,
 * starting a new realization there.
 *
 * Modulated generation (fillModulated/mixModulated) takes one increment per sample and cannot use the lane
 * recurrence, so it accumulates phase scalar-wise in double. Its phase sequence is the prefix sum of the
 * increments reduced to (-pi, pi] after each step, which likewise makes each phase a pure function of the
 * previous one and so of the same accuracy however the stream is split. A modulated call re-anchors the grid at its
 * end phase.
 *
 * All phase state is double regardless of F: a value_type accumulator rounds the re-seed phase once per call,
 * which for F = float is ~2.4e-7 rad and dominates every other error over a stream.
 *
 * Units are radians and radians/sample throughout. Hz belongs to blocks, which know a sample rate.
 */
template<std::floating_point F>
struct Phasor {
    using value_type = F;
    using Complex    = std::complex<F>;
    using Lane       = LanePhasor<F>;

    /// interleaved generation lanes: the sample loop advances kLanes phasors that depend on none of their
    /// neighbors, which is what lets it vectorize
    static constexpr std::size_t kLanes = Lane::kLanes;

    /// Re-seed cadence, a whole number of lane groups. It sets the longest chain of rounded multiplications
    /// between two exact seeds, which is the dominant error term in float, so a short cadence is what holds
    /// the phase error near its floor; the seed costs one sin/cos per cadence, which at 256 stays below a
    /// hundredth of the per-sample work.
    static constexpr std::size_t kReseedInterval = Lane::kReseedInterval;

    /// The fixed-increment stream: it owns the phase, the increment and the lanes whenever no chirp is running.
    Lane _lane{};

    /// The chirp's own phase frame, live while `_chirpActive`: the exact phase and increment at the start of the
    /// current interval and the samples since then, in [0, kReseedInterval). A chirp's increment is not
    /// constant, so the lane machinery does not serve it.
    double      _seedPhase{0.};
    double      _increment{0.};
    std::size_t _position{0UZ};
    bool        _chirpActive{false};

    /// The chirp recurrence: the phasor, the rotator that advances it, and whether the pair is live. They are
    /// carried across calls for the same reason the lanes are, and are seeded only at offset 0 of a re-seed
    /// interval, so the sequence of operations depends on the stream position alone.
    std::complex<double> _chirpPhasor{1., 0.};
    std::complex<double> _chirpRotator{1., 0.};
    bool                 _chirpValid{false};

    [[nodiscard]] static double wrapPhase(double phase) noexcept { return std::remainder(phase, 2. * std::numbers::pi_v<double>); }

    void configure(double increment, double initialPhase) {
        if (!std::isfinite(increment) || !std::isfinite(initialPhase)) {
            throw std::invalid_argument("gr::signal::Phasor: increment and initial phase must both be finite");
        }
        _chirpActive = false;
        _lane.configure(increment, initialPhase);
    }

    /// The increment changes, the phase runs on: a frequency change advances the phase from where it stands
    /// rather than restarting it. The grid re-anchors on the current phase, so the new increment takes effect
    /// from the next sample.
    void setIncrement(double increment) {
        if (!std::isfinite(increment)) {
            throw std::invalid_argument("gr::signal::Phasor: increment must be finite");
        }
        leaveChirp();
        _lane.setIncrement(increment);
    }

    void setPhase(double newPhase) {
        if (!std::isfinite(newPhase)) {
            throw std::invalid_argument("gr::signal::Phasor: phase must be finite");
        }
        leaveChirp();
        _lane.setPhase(newPhase);
    }

    [[nodiscard]] double phase() const noexcept { return _chirpActive ? wrapPhase(_seedPhase + static_cast<double>(_position) * _increment) : _lane.phase(); }
    [[nodiscard]] double increment() const noexcept { return _chirpActive ? _increment : _lane.increment(); }

    /// Moves the phase by nSamples increments and re-anchors the generation grid there.
    void advance(std::ptrdiff_t nSamples) noexcept {
        const double target = phase() + static_cast<double>(nSamples) * increment();
        leaveChirp();
        _lane.setPhase(target);
    }

    /// out[k] = exp(j*(phase + k*increment)); leaves the phase advanced by out.size() increments
    void fill(std::span<Complex> out) noexcept {
        leaveChirp();
        _lane.fill(out);
    }

    /// out[k] = in[k] * exp(j*(phase + k*increment)); leaves the phase advanced by the sample count
    void mix(std::span<const Complex> in, std::span<Complex> out) noexcept {
        leaveChirp();
        _lane.mix(in.first(std::min(in.size(), out.size())), out);
    }

    /// out[k] = exp(j*phase_k) with phase_{k+1} = phase_k + increments[k] — the CPM/FM entry point
    template<std::floating_point FIncrement>
    void fillModulated(std::span<const FIncrement> increments, std::span<Complex> out) noexcept {
        modulate<false>(increments, {}, out);
    }

    /// the same, multiplied into a stream — the time-varying-CFO entry point
    template<std::floating_point FIncrement>
    void mixModulated(std::span<const FIncrement> increments, std::span<const Complex> in, std::span<Complex> out) noexcept {
        modulate<true>(increments, in, out);
    }

    /// @brief Fills `out` with a linear chirp: the increment starts at the current one and grows by
    /// `incrementStep` radians per sample, per sample.
    ///
    /// The schedule a linear frequency sweep produces is arithmetic, so the phasor needs no transcendental per
    /// sample: it is multiplied by a rotator which is itself multiplied by a constant. Set the starting increment
    /// with `setIncrement` before the first call; a chirp and a plain fill are two state machines over the same
    /// phase, so a given Phasor runs one or the other, not both at once.
    void fillChirp(double incrementStep, std::span<Complex> out) noexcept { chirp<false>(incrementStep, {}, out); }

    /// @brief Multiplies `in` by that chirp into `out`, over `min(in.size(), out.size())` samples.
    void mixChirp(double incrementStep, std::span<const Complex> in, std::span<Complex> out) noexcept { chirp<true>(incrementStep, in, out.first(std::min(in.size(), out.size()))); }

private:
    /// Hands the stream back to the lanes at the phase and increment a chirp has reached, so the fixed-increment
    /// calls continue from where it stopped.
    void leaveChirp() noexcept {
        if (_chirpActive) {
            _lane.configure(_increment, phase());
            _chirpActive = false;
        }
    }

    /// Starts the chirp frame at the lane stream's phase and increment.
    void enterChirp() noexcept {
        if (!_chirpActive) {
            _seedPhase   = _lane.phase();
            _increment   = _lane.increment();
            _position    = 0UZ;
            _chirpActive = true;
            _chirpValid  = false;
        }
    }

    /// Seeds the chirp pair at offset 0 of the current interval, where the exact phase is `_seedPhase` and the
    /// exact increment is `_increment`.
    void seedChirp() noexcept {
        _chirpPhasor  = std::polar(1., _seedPhase);
        _chirpRotator = std::polar(1., _increment);
        _chirpValid   = true;
    }

    /// Rolls the interval forward by one whole cadence, exactly: the phase advances by the sum of the schedule
    /// over the interval, and the increment by the interval's worth of steps.
    void rollChirpInterval(double incrementStep) noexcept {
        constexpr double m = static_cast<double>(kReseedInterval);
        _seedPhase         = wrapPhase(_seedPhase + m * _increment + incrementStep * m * (m - 1.) * 0.5);
        _increment += m * incrementStep;
        _position = 0UZ;
        seedChirp();
    }

    template<bool kMix>
    void chirp(double incrementStep, std::span<const Complex> in, std::span<Complex> out) noexcept {
        const std::size_t nSamples = out.size();
        if (nSamples == 0UZ) {
            return;
        }
        enterChirp();
        if (!_chirpValid) {
            seedChirp();
        }
        const std::complex<double> stepRotator = std::polar(1., incrementStep);

        for (std::size_t k = 0UZ; k < nSamples; ++k) {
            if (_position == kReseedInterval) {
                rollChirpInterval(incrementStep);
            }
            const Complex phasor(static_cast<F>(_chirpPhasor.real()), static_cast<F>(_chirpPhasor.imag()));
            if constexpr (kMix) {
                out[k] = in[k] * phasor;
            } else {
                out[k] = phasor;
            }
            _chirpPhasor *= _chirpRotator;
            _chirpRotator *= stepRotator;
            ++_position;
        }
    }

    template<bool kMix, std::floating_point FIncrement>
    void modulate(std::span<const FIncrement> increments, std::span<const Complex> in, std::span<Complex> out) noexcept {
        std::size_t nSamples = std::min(increments.size(), out.size());
        if constexpr (kMix) {
            nSamples = std::min(nSamples, in.size());
        }

        constexpr double twoPi = 2. * std::numbers::pi_v<double>;
        leaveChirp();
        double value = _lane.phase();
        for (std::size_t k = 0UZ; k < nSamples; ++k) {
            const F phRe = static_cast<F>(std::cos(value));
            const F phIm = static_cast<F>(std::sin(value));
            if constexpr (kMix) {
                const F re = in[k].real();
                const F im = in[k].imag();
                out[k]     = Complex(re * phRe - im * phIm, re * phIm + im * phRe);
            } else {
                out[k] = Complex(phRe, phIm);
            }
            value += static_cast<double>(increments[k]);
            // reduce every step rather than once per call, so the phase sequence depends only on the previous
            // phase and the increment and stays equally exact however the stream is split
            if (value > std::numbers::pi_v<double> || value < -std::numbers::pi_v<double>) {
                value = std::remainder(value, twoPi);
            }
        }
        _lane.setPhase(value);
    }
};

} // namespace gr::signal

#endif // GNURADIO_ALGORITHM_PHASOR_HPP
