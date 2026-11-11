#ifndef GNURADIO_SAMPLE_RATE_ESTIMATOR_HPP
#define GNURADIO_SAMPLE_RATE_ESTIMATOR_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>
#include <type_traits>
#include <vector>

#include <gnuradio-4.0/algorithm/filter/FilterTool.hpp>

namespace gr::algorithm {

/**
 * @brief Estimates the true sample rate from timestamped chunk arrivals.
 *
 * Each update() call provides a wall-clock timestamp and the number of samples
 * delivered. The instantaneous period per sample (dt / nSamples) is fed through
 * a configurable IIR low-pass filter (Butterworth, default 2nd order) to smooth
 * out transport jitter. The estimated rate is 1 / filteredPeriod.
 *
 * Usage: call update(tObsSeconds, nSamples) once per chunk delivery.
 * Query estimatedRate() or estimatedPpm() at any time.
 * resetPhase() after a retune/reconfiguration preserves the filter state.
 *
 * Inspired by:
 *   Adriaensen, "Using a DLL to filter time" (2005)
 *   https://kokkinizita.linuxaudio.org/papers/usingdll.pdf
 *   Adriaensen, "Adaptive resampling"
 *   https://kokkinizita.linuxaudio.org/papers/adapt-resamp.pdf
 */
struct SampleRateEstimator {
    float       filter_cutoff_hz = 0.1f; // LP cutoff for period smoothing
    std::size_t filter_order     = 2UZ;  // Butterworth order
    float       ppm_initial      = 0.0f;

    double                 _periodEst   = 0.0; // filtered period per sample (s/sample)
    double                 _nominalRate = 0.0;
    double                 _tPrev       = 0.0;
    bool                   _initialised = false;
    bool                   _hasPrev     = false;
    filter::Filter<double> _lpFilter;
    double                 _updateRate = 0.0; // estimated chunk delivery rate for filter design

    void reset(double nominalRate, double expectedUpdateRateHz = 250.0) {
        _nominalRate = nominalRate;
        _periodEst   = (nominalRate > 0.0) ? (1.0 / nominalRate) : 0.0;
        if (ppm_initial != 0.0f) {
            _periodEst *= (1.0 + static_cast<double>(ppm_initial) * 1e-6);
        }
        _tPrev       = 0.0;
        _initialised = false;
        _hasPrev     = false;
        _updateRate  = expectedUpdateRateHz;
        rebuildFilter();
    }

    void resetPhase() {
        _tPrev   = 0.0;
        _hasPrev = false;
    }

    void update(double tObs, std::size_t nSamples) {
        if (_nominalRate <= 0.0 || nSamples == 0) {
            return;
        }

        if (!_hasPrev) {
            _tPrev   = tObs;
            _hasPrev = true;
            return;
        }

        double dt = tObs - _tPrev;
        _tPrev    = tObs;

        if (dt <= 0.0) {
            return;
        }

        double measuredPeriod = dt / static_cast<double>(nSamples);
        _periodEst            = _lpFilter.processOne(measuredPeriod);
        _initialised          = true;
    }

    [[nodiscard]] double estimatedRate() const noexcept {
        if (!_initialised || _periodEst <= 0.0 || _nominalRate <= 0.0) {
            return _nominalRate;
        }
        return 1.0 / _periodEst;
    }

    [[nodiscard]] float estimatedPpm() const noexcept {
        if (!_initialised || _periodEst <= 0.0 || _nominalRate <= 0.0) {
            return 0.0f;
        }
        double estRate = 1.0 / _periodEst;
        return static_cast<float>((estRate - _nominalRate) / _nominalRate * 1e6);
    }

    void rebuildFilter() {
        if (_updateRate > 0.0 && filter_cutoff_hz > 0.f) {
            double cutoff = std::min(static_cast<double>(filter_cutoff_hz), _updateRate * 0.4);
            auto   coeffs = filter::iir::designFilter<double>(filter::Type::LOWPASS, filter::FilterParameters{.order = filter_order, .fLow = cutoff, .fs = _updateRate}, filter::iir::Design::BUTTERWORTH);
            _lpFilter     = filter::Filter<double>(coeffs);
            _lpFilter.reset(_periodEst);
        }
    }
};

/**
 * @brief Drift correction mode for DriftCompensator.
 *
 * Controls how clock drift between two domains is corrected:
 * - None: no correction, ring buffer absorbs drift (may underrun/overrun)
 * - Linear: insert/drop one frame when accumulator crosses ±1, linear interpolation at splice
 * - Cubic: same as Linear but uses cubic Hermite interpolation at splice for smoother result
 * - AdaptiveResampling: every output sample is resampled at a continuously varying ratio
 *   using cubic Hermite interpolation. No discrete correction events. Based on the adaptive
 *   resampling approach described in:
 *     Fons Adriaensen, "Using a DLL to filter time", Linux Audio Conference 2005
 *     Fons Adriaensen, "Controlling adaptive resampling", Linux Audio Conference 2012
 *   As implemented in zita-ajbridge and adopted by PipeWire and JACK.
 */
enum class DriftCorrection {
    None,               // no correction
    Linear,             // insert/drop with linear interpolation at splice boundary
    Cubic,              // insert/drop with cubic Hermite interpolation at splice boundary
    AdaptiveResampling, // continuous variable-rate cubic Hermite resampling (Adriaensen)
};

/**
 * @brief Compensates small clock drift between two sample-rate domains.
 *
 * Tracks a fractional sample accumulator driven by the ratio between estimated and
 * nominal rates. The correction method depends on the DriftCorrection mode.
 * The accumulator is clamped to ±kMaxAccumulator to prevent burst corrections after gaps.
 */
template<typename T>
requires std::is_arithmetic_v<T>
struct DriftCompensator {
    static constexpr double kMaxAccumulator = 2.0;
    static constexpr double kStartPhase     = -2.0; // AdaptiveResampling: the first output frame, in frames from the first input frame

    DriftCorrection mode{DriftCorrection::Linear};
    double          fractionalAccumulator{0.0};
    double          resamplerPhase{0.0}; // AdaptiveResampling: position of the next output frame, in input frames from the next call's first

    // AdaptiveResampling: where the next call's first output frame lies, in input frames from that call's first
    // (the stream starts two frames early, see resampleStream)
    [[nodiscard]] double nextOutputPhase() const { return _primed ? resamplerPhase : kStartPhase; }

    void reset() {
        fractionalAccumulator = 0.0;
        resamplerPhase        = 0.0;
        _primed               = false;
        _history.clear();
    }

    std::size_t compensateSource(std::span<T> output, std::size_t nProduced, double estimatedRate, double nominalRate, std::size_t channelCount) {
        if (mode == DriftCorrection::None || estimatedRate <= 0.0 || nominalRate <= 0.0 || nProduced == 0U || channelCount == 0U) {
            return nProduced;
        }

        if (mode == DriftCorrection::AdaptiveResampling) {
            return resampleInPlace(output, nProduced, estimatedRate / nominalRate, channelCount);
        }

        const double ratio = estimatedRate / nominalRate;
        fractionalAccumulator += static_cast<double>(nProduced / channelCount) * (ratio - 1.0);
        fractionalAccumulator = std::clamp(fractionalAccumulator, -kMaxAccumulator, kMaxAccumulator);

        if (fractionalAccumulator >= 1.0 && nProduced + channelCount <= output.size()) {
            const std::size_t insertAt = nProduced;
            for (std::size_t ch = 0U; ch < channelCount; ++ch) {
                output[insertAt + ch] = interpolateAtSplice(output, nProduced, ch, channelCount, 0.5);
            }
            fractionalAccumulator -= 1.0;
            return nProduced + channelCount;
        }

        if (fractionalAccumulator <= -1.0 && nProduced >= 2U * channelCount) {
            for (std::size_t ch = 0U; ch < channelCount; ++ch) {
                const T kept                               = output[nProduced - 2U * channelCount + ch];
                const T dropped                            = output[nProduced - channelCount + ch];
                output[nProduced - 2U * channelCount + ch] = interpolatePair(kept, dropped, 0.5);
            }
            fractionalAccumulator += 1.0;
            return nProduced - channelCount;
        }

        return nProduced;
    }

    std::size_t compensateSink(std::span<const T> input, std::span<T> adjusted, std::size_t nAvailable, double estimatedRate, double nominalRate, std::size_t channelCount) {
        if (mode == DriftCorrection::None || estimatedRate <= 0.0 || nominalRate <= 0.0 || nAvailable == 0U || channelCount == 0U) {
            std::copy_n(input.begin(), static_cast<std::ptrdiff_t>(nAvailable), adjusted.begin());
            return nAvailable;
        }

        if (mode == DriftCorrection::AdaptiveResampling) {
            return resampleSink(input, adjusted, nAvailable, nominalRate / estimatedRate, channelCount);
        }

        const double ratio = nominalRate / estimatedRate;
        fractionalAccumulator += static_cast<double>(nAvailable / channelCount) * (ratio - 1.0);
        fractionalAccumulator = std::clamp(fractionalAccumulator, -kMaxAccumulator, kMaxAccumulator);

        std::copy_n(input.begin(), static_cast<std::ptrdiff_t>(nAvailable), adjusted.begin());

        if (fractionalAccumulator >= 1.0 && nAvailable + channelCount <= adjusted.size()) {
            for (std::size_t ch = 0U; ch < channelCount; ++ch) {
                adjusted[nAvailable + ch] = interpolateAtSplice(std::span<const T>(adjusted), nAvailable, ch, channelCount, 0.5);
            }
            fractionalAccumulator -= 1.0;
            return nAvailable + channelCount;
        }

        if (fractionalAccumulator <= -1.0 && nAvailable >= 2U * channelCount) {
            for (std::size_t ch = 0U; ch < channelCount; ++ch) {
                const T kept                                  = adjusted[nAvailable - 2U * channelCount + ch];
                const T dropped                               = adjusted[nAvailable - channelCount + ch];
                adjusted[nAvailable - 2U * channelCount + ch] = interpolatePair(kept, dropped, 0.5);
            }
            fractionalAccumulator += 1.0;
            return nAvailable - channelCount;
        }

        return nAvailable;
    }

private:
    [[nodiscard]] T interpolatePair(T a, T b, double t) const {
        if constexpr (std::is_floating_point_v<T>) {
            return static_cast<T>(static_cast<double>(a) + (static_cast<double>(b) - static_cast<double>(a)) * t);
        } else {
            return static_cast<T>(static_cast<double>(a) + (static_cast<double>(b) - static_cast<double>(a)) * t);
        }
    }

    // get sample at frame index for a given channel from interleaved data
    [[nodiscard]] static double getSample(std::span<const T> data, std::size_t frameIdx, std::size_t ch, std::size_t channelCount) {
        const std::size_t idx = frameIdx * channelCount + ch;
        return idx < data.size() ? static_cast<double>(data[idx]) : 0.0;
    }

    [[nodiscard]] static double getSample(std::span<T> data, std::size_t frameIdx, std::size_t ch, std::size_t channelCount) {
        const std::size_t idx = frameIdx * channelCount + ch;
        return idx < data.size() ? static_cast<double>(data[idx]) : 0.0;
    }

    [[nodiscard]] static T toSample(double value) {
        if constexpr (std::is_floating_point_v<T>) {
            return static_cast<T>(value);
        } else {
            return static_cast<T>(std::clamp(std::lround(value), static_cast<long>(std::numeric_limits<T>::min()), static_cast<long>(std::numeric_limits<T>::max())));
        }
    }

    // cubic Hermite interpolation between 4 points at fractional position t in [0,1]
    [[nodiscard]] static double cubicHermite(double y0, double y1, double y2, double y3, double t) {
        const double a = -0.5 * y0 + 1.5 * y1 - 1.5 * y2 + 0.5 * y3;
        const double b = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
        const double c = -0.5 * y0 + 0.5 * y2;
        const double d = y1;
        return ((a * t + b) * t + c) * t + d;
    }

    // interpolate at the splice point (insert/drop boundary)
    template<typename Span>
    [[nodiscard]] T interpolateAtSplice(Span data, std::size_t nProduced, std::size_t ch, std::size_t channelCount, double t) const {
        const std::size_t nFrames = nProduced / channelCount;
        if (mode == DriftCorrection::Cubic && nFrames >= 2U) {
            const double y0 = nFrames >= 3U ? getSample(data, nFrames - 3U, ch, channelCount) : getSample(data, nFrames - 2U, ch, channelCount);
            const double y1 = getSample(data, nFrames - 2U, ch, channelCount);
            const double y2 = getSample(data, nFrames - 1U, ch, channelCount);
            const double y3 = y2; // extrapolate: repeat last
            return toSample(cubicHermite(y0, y1, y2, y3, t));
        }
        // linear fallback
        const double prev = nFrames >= 2U ? getSample(data, nFrames - 2U, ch, channelCount) : 0.0;
        const double curr = nFrames >= 1U ? getSample(data, nFrames - 1U, ch, channelCount) : 0.0;
        return toSample(prev + (curr - prev) * t);
    }

    // continuous variable-rate resampling in-place (source): the call's frames are read at `ratio` input frames per
    // output frame; the grid carries on from the previous call (see resampleStream)
    std::size_t resampleInPlace(std::span<T> output, std::size_t nProduced, double ratio, std::size_t channelCount) { return resampleStream(std::span<const T>(output.data(), nProduced), output, nProduced / channelCount, ratio, channelCount); }

    // continuous variable-rate resampling for sink: input -> adjusted at given ratio (output frames per input frame)
    std::size_t resampleSink(std::span<const T> input, std::span<T> adjusted, std::size_t nAvailable, double ratio, std::size_t channelCount) { return resampleStream(input, adjusted, nAvailable / channelCount, 1.0 / ratio, channelCount); }

    // One resampling grid across calls. Output frame k reads the input stream at `resamplerPhase + k * step`, in
    // frames from the call's first one; what is left over (position and the last kHistory input frames) is carried to
    // the next call, so the output count follows the exact ratio and the cubic sees the true neighbors at a call
    // boundary. A cubic at position p needs frames floor(p)-1 .. floor(p)+2, so output stops at p < nFrames - 2 and
    // the stream lags its input by two frames. `input` may alias `output`: it is copied before the first write.
    std::size_t resampleStream(std::span<const T> input, std::span<T> output, std::size_t nFrames, double step, std::size_t channelCount) {
        constexpr std::size_t kHistory = 3U;
        const std::size_t     histLen  = kHistory * channelCount;
        if (nFrames == 0U) {
            return 0U;
        }

        _work.resize(histLen + nFrames * channelCount);
        if (!_primed) { // the first call: the stream's start is the first frame, repeated
            for (std::size_t f = 0U; f < kHistory; ++f) {
                std::copy_n(input.begin(), static_cast<std::ptrdiff_t>(channelCount), _work.begin() + static_cast<std::ptrdiff_t>(f * channelCount));
            }
            resamplerPhase = kStartPhase;
            _primed        = true;
        } else {
            std::copy_n(_history.begin(), static_cast<std::ptrdiff_t>(histLen), _work.begin());
        }
        std::copy_n(input.begin(), static_cast<std::ptrdiff_t>(nFrames * channelCount), _work.begin() + static_cast<std::ptrdiff_t>(histLen));

        const double limit      = static_cast<double>(nFrames) - 2.0;
        double       pos        = resamplerPhase;
        std::size_t  nOutFrames = 0U;
        while (pos < limit) {
            if ((nOutFrames + 1U) * channelCount > output.size()) {
                pos = limit; // no room: the frames still due are dropped, not carried as a debt
                break;
            }
            const double      floorPos = std::floor(pos);
            const double      fracPos  = pos - floorPos;
            const std::size_t base     = static_cast<std::size_t>(static_cast<std::ptrdiff_t>(floorPos) + static_cast<std::ptrdiff_t>(kHistory)); // frame of y1 in _work
            for (std::size_t ch = 0U; ch < channelCount; ++ch) {
                const double y0                        = static_cast<double>(_work[(base - 1U) * channelCount + ch]);
                const double y1                        = static_cast<double>(_work[base * channelCount + ch]);
                const double y2                        = static_cast<double>(_work[(base + 1U) * channelCount + ch]);
                const double y3                        = static_cast<double>(_work[(base + 2U) * channelCount + ch]);
                output[nOutFrames * channelCount + ch] = toSample(cubicHermite(y0, y1, y2, y3, fracPos));
            }
            ++nOutFrames;
            pos = resamplerPhase + static_cast<double>(nOutFrames) * step;
        }

        resamplerPhase = pos - static_cast<double>(nFrames);
        _history.assign(_work.end() - static_cast<std::ptrdiff_t>(histLen), _work.end());
        return nOutFrames * channelCount;
    }

    bool           _primed{false};
    std::vector<T> _history; // the last input frames of the previous call
    std::vector<T> _work;    // history and the current call's input
};

} // namespace gr::algorithm

#endif // GNURADIO_SAMPLE_RATE_ESTIMATOR_HPP
