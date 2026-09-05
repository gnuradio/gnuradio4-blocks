#ifndef GNURADIO_SYNC_PLL_HPP
#define GNURADIO_SYNC_PLL_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <numbers>
#include <tuple>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/sync/ControlLoop.hpp>

#include <gnuradio-4.0/sync/LoopCommon.hpp>

namespace gr::blocks::sync {

namespace detail {

/// @brief The phase detector's S-curve slope. `arg` of the derotated sample has slope one at any input amplitude, which
/// is why this family needs no AGC and why the constant is exact rather than measured.
inline constexpr double kArgumentDetectorGain = 1.0;

/**
 * @brief The lock metric: a coherent average divided by an amplitude reference, with a time constant of its own.
 *
 * One accumulator holds `Re{y}`, the carrier's amplitude when the loop is locked and zero mean when it is not; the
 * other holds `|y|`, which is that amplitude either way. Their ratio is therefore 1 at lock and near 0 away from it,
 * at every input level. `tau` is a parameter rather than the loop's proportional gain, so the metric's response time
 * is not welded to the loop bandwidth. The ratio is signed, so a loop locked half a turn out reports unlocked.
 */
struct LockDetector {
    float _coherent{0.f};
    float _amplitude{0.f};
    float _gain{1e-3f};

    void setTimeConstant(double samples) noexcept { _gain = samples > 0.0 ? static_cast<float>(-std::expm1(-1.0 / samples)) : 1.f; }

    void reset() noexcept {
        _coherent  = 0.f;
        _amplitude = 0.f;
    }

    void accumulate(float coherent, float magnitude) noexcept {
        _coherent += _gain * (coherent - _coherent);
        _amplitude += _gain * (magnitude - _amplitude);
    }

    [[nodiscard]] float metric() const noexcept { return std::clamp(_coherent / std::max(_amplitude, 1e-20f), 0.f, 1.f); }
};

/// @brief What one turn of the shared sequence produces; the three blocks differ only in which of these they emit.
struct CarrierStep {
    std::complex<float> rotor;     /// `exp(-j*p_n)`, the phasor the loop actually used
    std::complex<float> derotated; /// `y = x[n] * rotor`
    float               phase;     /// `p_n`, before this sample's update
    float               frequency; /// the integrator, after this sample's update
};

/**
 * @brief Derotate, take the argument, advance the loop.
 *
 * The error is `arg(y)` and not `arg(x) - p`: the two are the same number, but the first is already in `[-pi, pi]` and
 * needs no reduction, and `y` has to be formed anyway. `atan2` is used rather than a polynomial approximation, a polynomial's systematic angle error
 * inside a feedback loop being a frequency bias rather than noise. A zero sample carries no phase and is fed to the
 * loop as zero error, because derotating an exact zero leaves signed zeros and `atan2(+0, -0)` is `pi`, which would
 * drive a silent input to the frequency clamp instead of letting it coast.
 */
[[nodiscard]] inline CarrierStep advanceCarrier(CarrierLoop& loop, std::complex<float> sample) noexcept {
    const float               phase  = loop.phase();
    const std::complex<float> rotor  = derotator(phase);
    const std::complex<float> y      = sample * rotor;
    const bool                silent = sample.real() == 0.f && sample.imag() == 0.f;
    std::ignore                      = loop.step(silent ? 0.f : std::atan2(y.imag(), y.real()));
    return {rotor, y, phase, loop.frequency()};
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::sync::PllCarrierTracking)

struct PllCarrierTracking : Block<PllCarrierTracking, NoTagPropagation> {
    using Description = Doc<R""(
@brief Tracks the dominant carrier in a complex input and emits the input with that carrier removed.

The loop estimates the phase and frequency of one dominant sinusoid and outputs `y[n] = x[n] * exp(-j*p_n)`, with
`p_n` estimated from every sample before this one; amplitude passes through untouched. For synchronous AM the real
part of the output is the recovered audio plus the carrier's DC; for a suppressed carrier it is the demodulated
baseband.

Frequencies are radians per sample on every port and parameter, `pi` being Nyquist; the block takes no sample rate and
reads no rate tag. `noise_bandwidth` is `Bn*T`, the closed loop's one-sided noise bandwidth normalized to the sample
rate. The lock metric is maintained only while the `lock` port is connected or `squelch_when_unlocked` is set, so
`locked` reads false and `lockMetric()` stays frozen otherwise. `phase_est` and `freq_est` tags set the loop state
before the tagged sample and are consumed; everything else passes through at its own offset.
)"">;

    PortIn<std::complex<float>>  in;
    PortOut<std::complex<float>> out;
    PortOut<float, Optional>     freq;
    PortOut<float, Optional>     phase;
    PortOut<float, Optional>     lock;

    Annotated<double, "noise_bandwidth", Doc<"closed-loop one-sided noise bandwidth, normalized: Bn*T in cycles/sample">> noise_bandwidth       = 0.01;
    Annotated<double, "damping", Doc<"loop damping factor; sqrt(2)/2 is the classical maximally flat choice">>            damping               = std::numbers::sqrt2 / 2.0;
    Annotated<float, "max_frequency", Unit<"rad/sample">, Doc<"upper clamp on the loop frequency; pi is Nyquist">>        max_frequency         = std::numbers::pi_v<float>;
    Annotated<float, "min_frequency", Unit<"rad/sample">, Doc<"lower clamp; must not exceed max_frequency">>              min_frequency         = -std::numbers::pi_v<float>;
    Annotated<double, "lock_time_constant", Doc<"averaging length of the lock metric, in samples">>                       lock_time_constant    = 1000.0;
    Annotated<double, "lock_threshold", Doc<"lock metric above which locked is true; dimensionless, in [0, 1]">>          lock_threshold        = 0.7;
    Annotated<bool, "squelch_when_unlocked", Doc<"zero the output while unlocked; a hard gate, clicking at each edge">>   squelch_when_unlocked = false;
    Annotated<bool, "locked", Doc<"observable: metric above threshold and the loop not parked on a bound">>               locked                = false;

    GR_MAKE_REFLECTABLE(PllCarrierTracking, in, out, freq, phase, lock, noise_bandwidth, damping, max_frequency, min_frequency, lock_time_constant, lock_threshold, squelch_when_unlocked, locked);

    detail::CarrierLoop  _loop{0.01, std::numbers::sqrt2 / 2.0, detail::kArgumentDetectorGain};
    detail::LockDetector _lock{};
    std::uint64_t        _ignoredTags = 0ULL;

    /// The loop gains, the frequency clamp and the lock metric are derived from the members, and a batch that moves
    /// no value never calls back, so a block constructed at its declared defaults is born with the clamp it names.
    explicit PllCarrierTracking(property_map init = {}) : Block<PllCarrierTracking, NoTagPropagation>(std::move(init)) { configure(); }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { configure(); }

    void configure() {
        detail::applyCarrierSettings(_loop, noise_bandwidth, damping, min_frequency, max_frequency);
        _lock.setTimeConstant(lock_time_constant);
    }

    void reset() {
        _loop.reset();
        _lock.reset();
        _ignoredTags = 0ULL;
    }

    /// @brief Tag payloads rejected as not finite, which a caller checking its own upstream wants to see.
    [[nodiscard]] std::uint64_t ignoredTagPayloads() const noexcept { return _ignoredTags; }

    [[nodiscard]] float lockMetric() const noexcept { return _lock.metric(); }

    [[nodiscard]] const detail::CarrierLoop& loop() const noexcept { return _loop; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& freqSpan, OutputSpanLike auto& phaseSpan, OutputSpanLike auto& lockSpan) {
        std::size_t nSamples = std::min(inSpan.size(), outSpan.size());
        nSamples             = detail::syncCount(nSamples, freqSpan);
        nSamples             = detail::syncCount(nSamples, phaseSpan);
        nSamples             = detail::syncCount(nSamples, lockSpan);

        const bool  wantFreq   = freqSpan.isConnected;
        const bool  wantPhase  = phaseSpan.isConnected;
        const bool  wantLock   = lockSpan.isConnected;
        const bool  gate       = squelch_when_unlocked;
        const bool  wantMetric = wantLock || gate;
        const float threshold  = static_cast<float>(lock_threshold);

        const auto&       rawTags = inSpan.rawTags;
        const std::size_t nTags   = rawTags.size();
        std::size_t       cursor  = 0UZ;

        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            while (cursor < nTags && rawTags[cursor].index <= inSpan.streamIndex + i) {
                if (rawTags[cursor].index >= inSpan.streamIndex) {
                    detail::routeTag(rawTags[cursor].map, _loop, _ignoredTags, [&](const property_map& forwarded) {
                        outSpan.publishTag(forwarded, i);
                        freqSpan.publishTag(forwarded, i);
                        phaseSpan.publishTag(forwarded, i);
                        lockSpan.publishTag(forwarded, i);
                    });
                }
                ++cursor;
            }

            const detail::CarrierStep step = detail::advanceCarrier(_loop, inSpan[i]);

            if (wantMetric) {
                _lock.accumulate(step.derotated.real(), std::sqrt(std::norm(step.derotated)));
                const float metric       = _lock.metric();
                const bool  sampleLocked = metric > threshold && !_loop.saturated();
                outSpan[i]               = gate && !sampleLocked ? std::complex<float>{} : step.derotated;
                if (wantLock) {
                    lockSpan[i] = metric;
                }
            } else {
                outSpan[i] = step.derotated;
            }
            if (wantFreq) {
                freqSpan[i] = step.frequency;
            }
            if (wantPhase) {
                phaseSpan[i] = step.phase;
            }
        }

        locked      = wantMetric && _lock.metric() > threshold && !_loop.saturated();
        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        freqSpan.publish(wantFreq ? nSamples : 0UZ);
        phaseSpan.publish(wantPhase ? nSamples : 0UZ);
        lockSpan.publish(wantLock ? nSamples : 0UZ);
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::sync::PllFreqDet)

struct PllFreqDet : Block<PllFreqDet, NoTagPropagation> {
    using Description = Doc<R""(
@brief An FM discriminator with a memory: the loop's frequency estimate, in radians per sample, as a real stream.

The same loop PllCarrierTracking runs, emitting the integrator arm instead of the derotated signal. A pure tone at `dnu`
cycles per sample settles at exactly `2*pi*dnu`, and the loop is second order, so a frequency step leaves no residual
phase error. The output at index `n` has already seen sample `n` — the post-update convention, which is what "the
instantaneous frequency at sample n" means and which puts the frequency and phase outputs on the same sample as each
other.

Against QuadratureDemod, which measures the same quantity, the estimate is filtered by the closed loop, so its output
noise bandwidth is `noise_bandwidth` rather than the full band; the price is that the tracking bandwidth must exceed the
modulation bandwidth, or the modulation is filtered away with the noise. Parameters, units, the lock metric and the tag
contract are PllCarrierTracking's, and so is the freedom from an AGC.
)"">;

    PortIn<std::complex<float>> in;
    PortOut<float>              out;
    PortOut<float, Optional>    phase;
    PortOut<float, Optional>    lock;

    Annotated<double, "noise_bandwidth", Doc<"closed-loop one-sided noise bandwidth, normalized: Bn*T in cycles/sample">> noise_bandwidth    = 0.01;
    Annotated<double, "damping", Doc<"loop damping factor; sqrt(2)/2 is the classical maximally flat choice">>            damping            = std::numbers::sqrt2 / 2.0;
    Annotated<float, "max_frequency", Unit<"rad/sample">, Doc<"upper clamp on the loop frequency; pi is Nyquist">>        max_frequency      = std::numbers::pi_v<float>;
    Annotated<float, "min_frequency", Unit<"rad/sample">, Doc<"lower clamp; must not exceed max_frequency">>              min_frequency      = -std::numbers::pi_v<float>;
    Annotated<double, "lock_time_constant", Doc<"averaging length of the lock metric, in samples">>                       lock_time_constant = 1000.0;
    Annotated<double, "lock_threshold", Doc<"lock metric above which locked is true; dimensionless, in [0, 1]">>          lock_threshold     = 0.7;
    Annotated<bool, "locked", Doc<"observable: metric above threshold and the loop not parked on a bound">>               locked             = false;

    GR_MAKE_REFLECTABLE(PllFreqDet, in, out, phase, lock, noise_bandwidth, damping, max_frequency, min_frequency, lock_time_constant, lock_threshold, locked);

    detail::CarrierLoop  _loop{0.01, std::numbers::sqrt2 / 2.0, detail::kArgumentDetectorGain};
    detail::LockDetector _lock{};
    std::uint64_t        _ignoredTags = 0ULL;

    /// The loop gains, the frequency clamp and the lock metric are derived from the members, and a batch that moves
    /// no value never calls back, so a block constructed at its declared defaults is born with the clamp it names.
    explicit PllFreqDet(property_map init = {}) : Block<PllFreqDet, NoTagPropagation>(std::move(init)) { configure(); }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { configure(); }

    void configure() {
        detail::applyCarrierSettings(_loop, noise_bandwidth, damping, min_frequency, max_frequency);
        _lock.setTimeConstant(lock_time_constant);
    }

    void reset() {
        _loop.reset();
        _lock.reset();
        _ignoredTags = 0ULL;
    }

    [[nodiscard]] std::uint64_t ignoredTagPayloads() const noexcept { return _ignoredTags; }

    [[nodiscard]] float lockMetric() const noexcept { return _lock.metric(); }

    [[nodiscard]] const detail::CarrierLoop& loop() const noexcept { return _loop; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& phaseSpan, OutputSpanLike auto& lockSpan) {
        std::size_t nSamples = std::min(inSpan.size(), outSpan.size());
        nSamples             = detail::syncCount(nSamples, phaseSpan);
        nSamples             = detail::syncCount(nSamples, lockSpan);

        const bool  wantPhase = phaseSpan.isConnected;
        const bool  wantLock  = lockSpan.isConnected;
        const float threshold = static_cast<float>(lock_threshold);

        const auto&       rawTags = inSpan.rawTags;
        const std::size_t nTags   = rawTags.size();
        std::size_t       cursor  = 0UZ;

        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            while (cursor < nTags && rawTags[cursor].index <= inSpan.streamIndex + i) {
                if (rawTags[cursor].index >= inSpan.streamIndex) {
                    detail::routeTag(rawTags[cursor].map, _loop, _ignoredTags, [&](const property_map& forwarded) {
                        outSpan.publishTag(forwarded, i);
                        phaseSpan.publishTag(forwarded, i);
                        lockSpan.publishTag(forwarded, i);
                    });
                }
                ++cursor;
            }

            const detail::CarrierStep step = detail::advanceCarrier(_loop, inSpan[i]);

            outSpan[i] = step.frequency;
            if (wantPhase) {
                phaseSpan[i] = step.phase;
            }
            if (wantLock) {
                _lock.accumulate(step.derotated.real(), std::sqrt(std::norm(step.derotated)));
                lockSpan[i] = _lock.metric();
            }
        }

        locked      = wantLock && _lock.metric() > threshold && !_loop.saturated();
        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        phaseSpan.publish(wantPhase ? nSamples : 0UZ);
        lockSpan.publish(wantLock ? nSamples : 0UZ);
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::sync::PllRefOut)

struct PllRefOut : Block<PllRefOut, NoTagPropagation> {
    using Description = Doc<R""(
@brief A clean unit-amplitude replica of the tracked carrier: `exp(+j*p_n)`, for regenerating a pilot.

The same loop the rest of the family runs, emitting the phasor it used rather than the derotated sample. The output is
exactly unit amplitude on every sample whatever the input amplitude and whether or not the loop is locked; a caller
who wants the pilot at its measured amplitude multiplies by the `lock` port or by their own measurement.

`p_n` is the phase before this sample's update, so `PllCarrierTracking.out[n] * PllRefOut.out[n] == x[n]` sample for
sample on the same input with the same parameters. Parameters, units, the lock metric and the tag contract are
PllCarrierTracking's, and so is the freedom from an AGC.
)"">;

    PortIn<std::complex<float>>  in;
    PortOut<std::complex<float>> out;
    PortOut<float, Optional>     freq;
    PortOut<float, Optional>     phase;
    PortOut<float, Optional>     lock;

    Annotated<double, "noise_bandwidth", Doc<"closed-loop one-sided noise bandwidth, normalized: Bn*T in cycles/sample">> noise_bandwidth    = 0.01;
    Annotated<double, "damping", Doc<"loop damping factor; sqrt(2)/2 is the classical maximally flat choice">>            damping            = std::numbers::sqrt2 / 2.0;
    Annotated<float, "max_frequency", Unit<"rad/sample">, Doc<"upper clamp on the loop frequency; pi is Nyquist">>        max_frequency      = std::numbers::pi_v<float>;
    Annotated<float, "min_frequency", Unit<"rad/sample">, Doc<"lower clamp; must not exceed max_frequency">>              min_frequency      = -std::numbers::pi_v<float>;
    Annotated<double, "lock_time_constant", Doc<"averaging length of the lock metric, in samples">>                       lock_time_constant = 1000.0;
    Annotated<double, "lock_threshold", Doc<"lock metric above which locked is true; dimensionless, in [0, 1]">>          lock_threshold     = 0.7;
    Annotated<bool, "locked", Doc<"observable: metric above threshold and the loop not parked on a bound">>               locked             = false;

    GR_MAKE_REFLECTABLE(PllRefOut, in, out, freq, phase, lock, noise_bandwidth, damping, max_frequency, min_frequency, lock_time_constant, lock_threshold, locked);

    detail::CarrierLoop  _loop{0.01, std::numbers::sqrt2 / 2.0, detail::kArgumentDetectorGain};
    detail::LockDetector _lock{};
    std::uint64_t        _ignoredTags = 0ULL;

    /// The loop gains, the frequency clamp and the lock metric are derived from the members, and a batch that moves
    /// no value never calls back, so a block constructed at its declared defaults is born with the clamp it names.
    explicit PllRefOut(property_map init = {}) : Block<PllRefOut, NoTagPropagation>(std::move(init)) { configure(); }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { configure(); }

    void configure() {
        detail::applyCarrierSettings(_loop, noise_bandwidth, damping, min_frequency, max_frequency);
        _lock.setTimeConstant(lock_time_constant);
    }

    void reset() {
        _loop.reset();
        _lock.reset();
        _ignoredTags = 0ULL;
    }

    [[nodiscard]] std::uint64_t ignoredTagPayloads() const noexcept { return _ignoredTags; }

    [[nodiscard]] float lockMetric() const noexcept { return _lock.metric(); }

    [[nodiscard]] const detail::CarrierLoop& loop() const noexcept { return _loop; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& freqSpan, OutputSpanLike auto& phaseSpan, OutputSpanLike auto& lockSpan) {
        std::size_t nSamples = std::min(inSpan.size(), outSpan.size());
        nSamples             = detail::syncCount(nSamples, freqSpan);
        nSamples             = detail::syncCount(nSamples, phaseSpan);
        nSamples             = detail::syncCount(nSamples, lockSpan);

        const bool  wantFreq  = freqSpan.isConnected;
        const bool  wantPhase = phaseSpan.isConnected;
        const bool  wantLock  = lockSpan.isConnected;
        const float threshold = static_cast<float>(lock_threshold);

        const auto&       rawTags = inSpan.rawTags;
        const std::size_t nTags   = rawTags.size();
        std::size_t       cursor  = 0UZ;

        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            while (cursor < nTags && rawTags[cursor].index <= inSpan.streamIndex + i) {
                if (rawTags[cursor].index >= inSpan.streamIndex) {
                    detail::routeTag(rawTags[cursor].map, _loop, _ignoredTags, [&](const property_map& forwarded) {
                        outSpan.publishTag(forwarded, i);
                        freqSpan.publishTag(forwarded, i);
                        phaseSpan.publishTag(forwarded, i);
                        lockSpan.publishTag(forwarded, i);
                    });
                }
                ++cursor;
            }

            const detail::CarrierStep step = detail::advanceCarrier(_loop, inSpan[i]);

            outSpan[i] = std::conj(step.rotor);
            if (wantFreq) {
                freqSpan[i] = step.frequency;
            }
            if (wantPhase) {
                phaseSpan[i] = step.phase;
            }
            if (wantLock) {
                _lock.accumulate(step.derotated.real(), std::sqrt(std::norm(step.derotated)));
                lockSpan[i] = _lock.metric();
            }
        }

        locked      = wantLock && _lock.metric() > threshold && !_loop.saturated();
        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        freqSpan.publish(wantFreq ? nSamples : 0UZ);
        phaseSpan.publish(wantPhase ? nSamples : 0UZ);
        lockSpan.publish(wantLock ? nSamples : 0UZ);
        return work::Status::OK;
    }
};

} // namespace gr::blocks::sync

#endif // GNURADIO_SYNC_PLL_HPP
