#ifndef GNURADIO_MEASUREMENT_POLARIZATION_COMBINER_HPP
#define GNURADIO_MEASUREMENT_POLARIZATION_COMBINER_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/MeasurementRecord.hpp>
#include <gnuradio-4.0/algorithm/MeasurementSlot.hpp>
#include <gnuradio-4.0/algorithm/math/Decibel.hpp>
#include <gnuradio-4.0/algorithm/measurement/PolarizationCombiner.hpp>

namespace gr::blocks::measurement {

namespace detail {
inline constexpr gr::Size_t kMinPolarizationWindow = 64U;
inline constexpr gr::Size_t kMaxPolarizationWindow = 1U << 24U;

/// @brief The validator of `sample_rate`: true above 0.
inline constexpr auto kCombinerSampleRatePositive = [](float value) { return value > 0.f; };

/// @brief The validator of `weight_smoothing`: true below 1.
inline constexpr auto kWeightSmoothingBelowOne = [](double value) { return value < 1.0; };
} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::measurement::PolarizationCombiner)

/**
 * @brief Combines two antenna branches at the maximal-ratio optimum, from the 2x2 branch covariance in closed form.
 *
 * The block takes two sample-aligned complex streams that carry one signal through two antennas or polarizations, and
 * puts out their weighted sum. Over each window of `window` samples it accumulates the branch covariance, and at the
 * window's close it solves for new weights. It combines each window with the weights the previous window gave, and
 * the weights lag the input by one window. Before the first window closes, `out` carries branch 0 and `ortho` carries
 * branch 1 unchanged.
 *
 * `out` carries the combined signal. `ortho` carries the orthogonal combination, which nulls the signal, while
 * `emit_orthogonal` is set and the port is connected. Each new weight vector is turned to give the signal in `out` the
 * phase the weights in use give it. The output phase then changes only with the channel, across a change of the
 * selected branch as well. A pair with a non-finite sample gives a non-finite output sample and stays out of the
 * covariance. The weights in use stay when a window holds no finite pair or its solve gives a weight that is not
 * finite.
 *
 * At each window's close the block publishes the relative phase `arg(h0) - arg(h1)`, the amplitude ratio `|h0|/|h1|`,
 * the signal-to-noise ratio of each branch and of the output, and the combining gain over the better branch, in dB.
 * `relativePhase()`, `amplitudeRatio()`, `branchSnrDb()`, `combinedSnrDb()`, `combiningGainDb()`, `selectedBranch()`
 * and `coverage()` read the latest window from any thread. `coverage()` reads the fraction of its own window the
 * latest estimate covers, and `selectedBranch()` reads -1 under 'mrc'. With `emit_records` set and `measurements`
 * connected, each window also goes out as a scalar record. Its metadata states `window`, `mode`, `normalize`,
 * `selected_branch`, `n_samples`, the pairs with a non-finite sample as `non_finite`, and the flags `saturated_snr` and
 * `saturated_ratio`. While records go out, a call stops before a sample that closes a window with no free record slot.
 *
 * A window with no measured noise has unbounded signal-to-noise ratios. A window with no correlation between the
 * branches and the larger power on branch 0 has an unbounded amplitude ratio. Both read `1e6` (60 dB) and set their
 * flag. A branch with no power reads a ratio of zero and a decibel figure of 0 dB. A record is `valid` when no figure
 * is saturated or at that floor, every pair was finite, and the window held at least 64 samples.
 *
 * `mode` 'mrc' maximizes the output signal-to-noise ratio. `mode` 'selection' passes the branch with the larger
 * whitened power, and its combining gain is 0 dB. `noise_powers`, empty or the two branch noise powers, whitens the
 * covariance for branches of unequal noise. `normalize` 'unit_noise' gives the output the noise power of one whitened
 * branch, and 'unit_signal' an estimated signal power of 1. Under 'mrc', `weight_smoothing` blends each new weight
 * vector with the one in use, and under 'selection' the weights switch at once. `sample_rate` is stated in the
 * records. A change of `window`, `noise_powers`, `mode` or `normalize` discards the partial window. At the end of the
 * stream the block publishes the partial window.
 *
 * `nWindows()` counts the full windows, and `nWindowResets()` the partial windows a setting change discarded.
 * `nBranchSwitches()` counts the changes of the selected branch under 'selection'. `nSaturatedFigures()` counts the
 * saturated figures, two at most per window. `nDroppedSampleTags()` counts the `n_dropped_samples` tags on either
 * branch.
 */
struct PolarizationCombiner : Block<PolarizationCombiner> {
    using Description = Doc<R""(
@brief Combines two antenna branches at the maximal-ratio optimum and publishes the estimator's figures.

The block accumulates the branch covariance over windows of `window` samples. It combines each window with the
weights the previous window's covariance gives. `mode = mrc` maximizes the combined signal-to-noise ratio. `selection`
takes the stronger branch from the same covariance. Before the first window closes, `out` carries branch 0 and `ortho`
carries branch 1 unchanged. With `emit_records` set, each window publishes one record of the relative phase, the
amplitude ratio, the two branch ratios, the combined ratio and the combining gain.
)"">;

    PortIn<std::complex<float>>              in0;
    PortIn<std::complex<float>>              in1;
    PortOut<std::complex<float>>             out;
    PortOut<std::complex<float>, Optional>   ortho;
    PortOut<DataSet<float>, Async, Optional> measurements;

    Annotated<std::string, "mode", Visible, Doc<"combining rule, 'mrc' or 'selection'">>                                                                                             mode             = std::string("mrc");
    Annotated<gr::Size_t, "window", Visible, Doc<"samples per estimate, from 64 to 2^24">, Limits<detail::kMinPolarizationWindow, detail::kMaxPolarizationWindow>>                   window           = 4096U;
    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"stream sample rate, positive and finite">, Limits<0.f, std::numeric_limits<float>::max(), detail::kCombinerSampleRatePositive>> sample_rate      = 96000.f;
    Annotated<std::vector<double>, "noise_powers", Visible, Doc<"branch noise powers, empty or two positive entries">>                                                               noise_powers     = std::vector<double>{};
    Annotated<std::string, "normalize", Visible, Doc<"weight scaling, 'unit_noise' or 'unit_signal'">>                                                                               normalize        = std::string("unit_noise");
    Annotated<double, "weight_smoothing", Visible, Doc<"share of the previous weights under 'mrc', in [0, 1)">, Limits<0.0, 1.0, detail::kWeightSmoothingBelowOne>>                  weight_smoothing = 0.0;
    Annotated<bool, "emit_orthogonal", Visible, Doc<"compute and publish the orthogonal output">>                                                                                    emit_orthogonal  = true;
    Annotated<bool, "emit_records", Visible, Doc<"publish one DataSet<float> record per window">>                                                                                    emit_records     = true;

    GR_MAKE_REFLECTABLE(PolarizationCombiner, in0, in1, out, ortho, measurements, mode, window, sample_rate, noise_powers, normalize, weight_smoothing, emit_orthogonal, emit_records);

    gr::measurement::PolarizationMode          _mode{gr::measurement::PolarizationMode::mrc};
    gr::measurement::PolarizationNormalization _normalize{gr::measurement::PolarizationNormalization::unit_noise};
    gr::Size_t                                 _window{4096U};
    float                                      _sampleRate{96000.f};
    std::vector<double>                        _noisePowers{};
    double                                     _smoothing{0.};
    bool                                       _emitOrthogonal{true};
    bool                                       _emitRecords{true};
    gr::measurement::BranchCovariance          _covariance{};
    gr::measurement::PolarizationEstimate      _estimate{gr::measurement::polarizationPassthrough(0UZ)}; ///< the weights in use
    bool                                       _haveEstimate{false};

    std::size_t   _covCount{0UZ};
    std::uint64_t _streamAt{0ULL};
    std::uint64_t _windowStartAt{0ULL};
    bool          _flushed{false};

    int  _lastSelected{-1};
    bool _haveSelected{false};

    gr::measurement::MeasurementSlot<8UZ> _slot{}; ///< relativePhase(rad), amplitudeRatio, branch0Db, branch1Db, combinedDb, selectedBranch, combiningGainDb, coverage

    std::atomic<std::uint64_t> _nWindows{0ULL};
    std::atomic<std::uint64_t> _nWindowResets{0ULL};
    std::atomic<std::uint64_t> _nBranchSwitches{0ULL};
    std::atomic<std::uint64_t> _nSaturatedFigures{0ULL};
    std::atomic<std::uint64_t> _nDroppedSampleTags{0ULL};

    std::vector<DataSet<float>> _pending{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const auto changes = [&newSettings](std::string_view key) { return newSettings.contains(key); };

        gr::measurement::PolarizationMode nextMode = _mode;
        if (changes("mode")) {
            const auto parsed = gr::measurement::polarizationModeFrom(mode.value);
            if (!parsed.has_value()) {
                throw gr::exception(std::format("mode must be 'mrc' or 'selection', got '{}'", mode.value));
            }
            nextMode = *parsed;
        }
        gr::measurement::PolarizationNormalization nextNorm = _normalize;
        if (changes("normalize")) {
            const auto parsed = gr::measurement::polarizationNormalizationFrom(normalize.value);
            if (!parsed.has_value()) {
                throw gr::exception(std::format("normalize must be 'unit_noise' or 'unit_signal', got '{}'", normalize.value));
            }
            nextNorm = *parsed;
        }
        if (changes("noise_powers") && !noise_powers.value.empty()) {
            if (noise_powers.value.size() != 2UZ) {
                throw gr::exception(std::format("noise_powers takes exactly two branches, got {}", noise_powers.value.size()));
            }
            for (const double power : noise_powers.value) {
                if (!std::isfinite(power) || !(power > 0.)) {
                    throw gr::exception(std::format("noise_powers entries must be finite and positive, got {}", power));
                }
            }
        }

        const gr::Size_t           nextWindow = changes("window") ? window.value : _window;
        const std::vector<double>& nextNoise  = changes("noise_powers") ? noise_powers.value : _noisePowers;
        if (nextWindow != _window || nextNoise != _noisePowers || nextMode != _mode || nextNorm != _normalize) {
            if (_covCount > 0UZ) {
                _nWindowResets.fetch_add(1ULL, std::memory_order_relaxed);
            }
            _covariance.reset();
            _covCount      = 0UZ;
            _windowStartAt = _streamAt;
        }

        _mode      = nextMode;
        _normalize = nextNorm;
        _window    = nextWindow;
        if (changes("noise_powers")) {
            _noisePowers = noise_powers.value;
        }
        if (changes("sample_rate")) {
            _sampleRate = sample_rate.value;
        }
        if (changes("weight_smoothing")) {
            _smoothing = weight_smoothing.value;
        }
        if (changes("emit_orthogonal")) {
            _emitOrthogonal = emit_orthogonal.value;
        }
        if (changes("emit_records")) {
            _emitRecords = emit_records.value;
        }
    }

    void start() {
        _covariance.reset();
        _covCount      = 0UZ;
        _streamAt      = 0ULL;
        _windowStartAt = 0ULL;
        _flushed       = false;
        _estimate      = gr::measurement::polarizationPassthrough(0UZ);
        _haveEstimate  = false;
        _lastSelected  = -1;
        _haveSelected  = false;
        _nWindows.store(0ULL, std::memory_order_relaxed);
        _nWindowResets.store(0ULL, std::memory_order_relaxed);
        _nBranchSwitches.store(0ULL, std::memory_order_relaxed);
        _nSaturatedFigures.store(0ULL, std::memory_order_relaxed);
        _nDroppedSampleTags.store(0ULL, std::memory_order_relaxed);
        _pending.clear();
        _slot.publish({0., 0., 0., 0., 0., 0., 0., 0.}, 0ULL);
    }

    [[nodiscard]] double relativePhase() const noexcept { return _slot.read().first[0]; }
    [[nodiscard]] double amplitudeRatio() const noexcept { return _slot.read().first[1]; }
    [[nodiscard]] double branchSnrDb(std::size_t branch) const noexcept { return _slot.read().first[branch == 0UZ ? 2UZ : 3UZ]; }
    [[nodiscard]] double combinedSnrDb() const noexcept { return _slot.read().first[4]; }
    /// @brief The combining gain over the better branch, in dB, as the record carries it.
    [[nodiscard]] double combiningGainDb() const noexcept { return _slot.read().first[6]; }
    [[nodiscard]] int    selectedBranch() const noexcept { return static_cast<int>(_slot.read().first[5]); }
    [[nodiscard]] double coverage() const noexcept { return _slot.read().first[7]; }

    [[nodiscard]] std::uint64_t nWindows() const noexcept { return _nWindows.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nWindowResets() const noexcept { return _nWindowResets.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nBranchSwitches() const noexcept { return _nBranchSwitches.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nSaturatedFigures() const noexcept { return _nSaturatedFigures.load(std::memory_order_relaxed); }
    /// @brief `n_dropped_samples` tags seen on either branch. Callable from any thread.
    [[nodiscard]] std::uint64_t nDroppedSampleTags() const noexcept { return _nDroppedSampleTags.load(std::memory_order_relaxed); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& in0Span, InputSpanLike auto& in1Span, OutputSpanLike auto& outSpan, OutputSpanLike auto& orthoSpan, OutputSpanLike auto& measurementsSpan) {
        std::size_t made = drain(measurementsSpan, 0UZ);

        const bool        wantOrtho   = _emitOrthogonal && orthoSpan.isConnected;
        const bool        wantRecords = _emitRecords && measurementsSpan.isConnected;
        const std::size_t offer       = std::min(in0Span.size(), in1Span.size());
        const std::size_t orthoRoom   = wantOrtho ? orthoSpan.size() : std::numeric_limits<std::size_t>::max();
        const std::size_t take        = std::min({offer, outSpan.size(), orthoRoom, wantRecords ? roomFor(measurementsSpan, made, offer) : offer});

        countDroppedTags(in0Span, in1Span, take);
        if (const auto combined = combine(std::span<const std::complex<float>>(in0Span).first(take), std::span<const std::complex<float>>(in1Span).first(take), std::span<std::complex<float>>(outSpan).first(take), std::span<std::complex<float>>(orthoSpan.data(), wantOrtho ? take : 0UZ), measurementsSpan.isConnected); !combined) {
            std::ignore = in0Span.consume(0UZ);
            std::ignore = in1Span.consume(0UZ);
            return refuse(outSpan, orthoSpan, measurementsSpan, made, combined.error());
        }
        made += drain(measurementsSpan, made);

        outSpan.publish(take);
        orthoSpan.publish(wantOrtho ? take : 0UZ);
        measurementsSpan.publish(made);
        std::ignore = in0Span.consume(take);
        std::ignore = in1Span.consume(take);
        return take == 0UZ && made == 0UZ && offer > 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }

    /// @brief At the end of the stream, combines the trailing samples and flushes the partial window.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& in0Span, InputSpanLike auto& in1Span, OutputSpanLike auto& outSpan, OutputSpanLike auto& orthoSpan, OutputSpanLike auto& measurementsSpan) {
        const bool        wantOrtho = _emitOrthogonal && orthoSpan.isConnected;
        const std::size_t take      = std::min({in0Span.size(), in1Span.size(), outSpan.size(), wantOrtho ? orthoSpan.size() : std::numeric_limits<std::size_t>::max()});
        countDroppedTags(in0Span, in1Span, take);
        if (const auto combined = combine(std::span<const std::complex<float>>(in0Span).first(take), std::span<const std::complex<float>>(in1Span).first(take), std::span<std::complex<float>>(outSpan).first(take), std::span<std::complex<float>>(orthoSpan.data(), wantOrtho ? take : 0UZ), measurementsSpan.isConnected); !combined) {
            return refuse(outSpan, orthoSpan, measurementsSpan, 0UZ, combined.error());
        }

        std::size_t made = drain(measurementsSpan, 0UZ);
        if (!_flushed && _covCount > 0UZ) {
            if (const auto closed = closeWindow(measurementsSpan.isConnected, false); !closed) {
                return refuse(outSpan, orthoSpan, measurementsSpan, made, closed.error());
            }
            made += drain(measurementsSpan, made);
            _flushed = true;
        }
        outSpan.publish(take);
        orthoSpan.publish(wantOrtho ? take : 0UZ);
        measurementsSpan.publish(made);
        return work::Status::OK;
    }

private:
    /// @brief Publishes the @p made records already drained and no samples, sends @p error to `emitErrorMessage` and
    /// returns `ERROR`.
    [[nodiscard]] work::Status refuse(OutputSpanLike auto& outSpan, OutputSpanLike auto& orthoSpan, OutputSpanLike auto& measurementsSpan, std::size_t made, const gr::Error& error) {
        outSpan.publish(0UZ);
        orthoSpan.publish(0UZ);
        measurementsSpan.publish(made);
        this->emitErrorMessage("processBulk", error);
        return work::Status::ERROR;
    }

    /// @brief Counts the `n_dropped_samples` tags on the @p count samples this call takes, on either branch. A tag on a
    /// later sample is counted by the call that takes that sample.
    void countDroppedTags(InputSpanLike auto& in0Span, InputSpanLike auto& in1Span, std::size_t count) {
        const std::uint64_t base   = _streamAt;
        const auto          onSpan = [this, base, count](const auto& span) {
            for (const gr::Tag& tag : span.rawTags) {
                const std::uint64_t at = static_cast<std::uint64_t>(tag.index);
                if (at < base || at >= base + count) {
                    continue;
                }
                if (tag.map.find(gr::tag::N_DROPPED_SAMPLES.shortKey()) == tag.map.end()) {
                    continue;
                }
                _nDroppedSampleTags.fetch_add(1ULL, std::memory_order_relaxed);
            }
        };
        onSpan(in0Span);
        onSpan(in1Span);
    }

    [[nodiscard]] std::expected<void, gr::Error> combine(std::span<const std::complex<float>> branch0, std::span<const std::complex<float>> branch1, std::span<std::complex<float>> combined, std::span<std::complex<float>> orthogonal, bool wantRecords) {
        std::size_t at = 0UZ;
        while (at < branch0.size()) {
            const std::size_t remaining = static_cast<std::size_t>(_window) - _covCount;
            const std::size_t chunk     = std::min(remaining, branch0.size() - at);

            if (auto mixed = gr::measurement::polarizationCombine(branch0.subspan(at, chunk), branch1.subspan(at, chunk), _estimate, combined.subspan(at, chunk), orthogonal.empty() ? std::span<std::complex<float>>{} : orthogonal.subspan(at, chunk)); !mixed) {
                return mixed;
            }
            if (auto added = _covariance.add(branch0.subspan(at, chunk), branch1.subspan(at, chunk)); !added) {
                return added;
            }

            _covCount += chunk;
            _streamAt += chunk;
            at += chunk;
            if (_covCount == static_cast<std::size_t>(_window)) {
                if (auto closed = closeWindow(wantRecords, true); !closed) {
                    return closed;
                }
            }
        }
        return {};
    }

    [[nodiscard]] std::expected<void, gr::Error> closeWindow(bool wantRecords, bool full) {
        const std::span<const double> noisePowers(_noisePowers);
        const auto                    solution = _covariance.solve(_mode, _normalize, noisePowers);
        if (!solution) {
            return std::unexpected(solution.error());
        }
        const gr::measurement::PolarizationEstimate& solved = *solution;

        if (solved.saturatedSnr) {
            _nSaturatedFigures.fetch_add(1ULL, std::memory_order_relaxed);
        }
        if (solved.saturatedRatio) {
            _nSaturatedFigures.fetch_add(1ULL, std::memory_order_relaxed);
        }

        if (_mode == gr::measurement::PolarizationMode::selection) {
            if (_haveSelected && solved.selectedBranch != _lastSelected) {
                _nBranchSwitches.fetch_add(1ULL, std::memory_order_relaxed);
            }
            _lastSelected = solved.selectedBranch;
            _haveSelected = true;
        }

        const auto finite = [](std::complex<double> value) noexcept { return std::isfinite(value.real()) && std::isfinite(value.imag()); };
        if (_covariance.count() > 0UZ && finite(solved.weight0) && finite(solved.weight1) && finite(solved.ortho0) && finite(solved.ortho1)) {
            const auto channel = _mode == gr::measurement::PolarizationMode::selection ? _covariance.solve(gr::measurement::PolarizationMode::mrc, gr::measurement::PolarizationNormalization::unit_noise, noisePowers) : solution;
            if (!channel) {
                return std::unexpected(channel.error());
            }
            gr::measurement::PolarizationEstimate applied = aligned(solved, _estimate, *channel, noisePowers);
            if (_mode == gr::measurement::PolarizationMode::mrc && _smoothing > 0. && _haveEstimate) {
                applied = smoothed(applied, _estimate, _smoothing, noisePowers);
            }
            _estimate     = applied;
            _haveEstimate = true;
        }

        const std::uint64_t filled  = full ? static_cast<std::uint64_t>(_window) : static_cast<std::uint64_t>(_covCount);
        const DbFigures     figures = dbFigures(solved);
        publishSlot(solved, figures, filled);

        if (wantRecords && _emitRecords) {
            _pending.push_back(makeRecord(solved, figures, _windowStartAt, filled, _covariance.nonFinite()));
        }
        if (full) {
            _nWindows.fetch_add(1ULL, std::memory_order_relaxed);
        }

        _covariance.reset();
        _covCount      = 0UZ;
        _windowStartAt = _streamAt;
        return {};
    }

    /// @brief The whitened weight vector of @p estimate, the weights times `sqrt(N_i)` normalized. The estimate's scale
    /// divides out.
    [[nodiscard]] static std::pair<std::complex<double>, std::complex<double>> whitenedUnit(const gr::measurement::PolarizationEstimate& estimate, double sqrtN0, double sqrtN1) noexcept {
        std::complex<double> v0    = estimate.weight0 * sqrtN0;
        std::complex<double> v1    = estimate.weight1 * sqrtN1;
        const double         norm2 = std::sqrt(std::norm(v0) + std::norm(v1));
        if (norm2 > 0.) {
            v0 /= norm2;
            v1 /= norm2;
        }
        return {v0, v1};
    }

    /**
     * @brief Turns the weights of @p fresh by the unit-modulus factor that gives them the phase @p prior has on the
     * estimated channel.
     *
     * The estimated channel is the whitened weight vector of @p channel. The phase of a weight vector `v` on it is
     * `arg(v^H h)`, the phase of the signal in the output. Under `mrc` @p channel is @p fresh, and the turn makes the
     * whitened inner product of @p fresh with @p prior real and positive. The orthogonal weights turn by the conjugate
     * factor and stay orthogonal. @p fresh stays as it is when either weight vector passes no signal on the channel.
     */
    [[nodiscard]] static gr::measurement::PolarizationEstimate aligned(const gr::measurement::PolarizationEstimate& fresh, const gr::measurement::PolarizationEstimate& prior, const gr::measurement::PolarizationEstimate& channel, std::span<const double> noisePowers) {
        const double sqrtN0               = noisePowers.empty() ? 1. : std::sqrt(noisePowers[0]);
        const double sqrtN1               = noisePowers.empty() ? 1. : std::sqrt(noisePowers[1]);
        const auto [freshV0, freshV1]     = whitenedUnit(fresh, sqrtN0, sqrtN1);
        const auto [priorV0, priorV1]     = whitenedUnit(prior, sqrtN0, sqrtN1);
        const auto [channelV0, channelV1] = whitenedUnit(channel, sqrtN0, sqrtN1);

        const std::complex<double> freshGain = std::conj(freshV0) * channelV0 + std::conj(freshV1) * channelV1;
        const std::complex<double> priorGain = std::conj(priorV0) * channelV0 + std::conj(priorV1) * channelV1;
        const std::complex<double> inner     = freshGain * std::conj(priorGain);
        const double               magnitude = std::abs(inner);
        if (!(magnitude > 0.)) {
            return fresh;
        }
        const std::complex<double>            rotation = inner / magnitude;
        gr::measurement::PolarizationEstimate out      = fresh;
        out.weight0                                    = fresh.weight0 * rotation;
        out.weight1                                    = fresh.weight1 * rotation;
        out.ortho0                                     = fresh.ortho0 * std::conj(rotation);
        out.ortho1                                     = fresh.ortho1 * std::conj(rotation);
        return out;
    }

    /**
     * @brief Blends `v = (1-g) v_new + g v_prev` on the whitened weight vectors.
     *
     * The caller aligns @p fresh to @p prior, and the two add in phase. The blend is normalized, unwhitened with the new
     * estimate's scale, and the orthogonal weights are rebuilt from it. In the whitened basis the orthogonal weights
     * null the signal under unequal branch noise as well.
     */
    [[nodiscard]] static gr::measurement::PolarizationEstimate smoothed(const gr::measurement::PolarizationEstimate& fresh, const gr::measurement::PolarizationEstimate& prior, double g, std::span<const double> noisePowers) {
        const double sqrtN0           = noisePowers.empty() ? 1. : std::sqrt(noisePowers[0]);
        const double sqrtN1           = noisePowers.empty() ? 1. : std::sqrt(noisePowers[1]);
        const auto [freshV0, freshV1] = whitenedUnit(fresh, sqrtN0, sqrtN1);
        const auto [priorV0, priorV1] = whitenedUnit(prior, sqrtN0, sqrtN1);

        std::complex<double> v0    = (1. - g) * freshV0 + g * priorV0;
        std::complex<double> v1    = (1. - g) * freshV1 + g * priorV1;
        const double         norm2 = std::sqrt(std::norm(v0) + std::norm(v1));
        if (norm2 > 0.) {
            v0 /= norm2;
            v1 /= norm2;
        }

        gr::measurement::PolarizationEstimate out = fresh;
        out.weight0                               = fresh.scale * v0 / sqrtN0;
        out.weight1                               = fresh.scale * v1 / sqrtN1;
        out.ortho0                                = -std::conj(v1) / sqrtN0;
        out.ortho1                                = std::conj(v0) / sqrtN1;
        return out;
    }

    /**
     * @brief The decibel figures of one window, with an unmeasured figure replaced by zero.
     *
     * A figure is unmeasured when it is not finite or lies at or below `gr::math::kDbFloor`, the level of a zero
     * ratio. The gain is unmeasured when the combined figure is unmeasured or both branch figures are. The gain is
     * taken against the larger measured branch figure. `finite` is false when any figure is unmeasured, and the
     * record's `valid` is then false.
     */
    struct DbFigures {
        double branch0{0.};
        double branch1{0.};
        double combined{0.};
        double gain{0.};
        bool   finite{true};
    };

    [[nodiscard]] static DbFigures dbFigures(const gr::measurement::PolarizationEstimate& e) noexcept {
        const double branch0  = e.branchSnrDb(0UZ);
        const double branch1  = e.branchSnrDb(1UZ);
        const double combined = e.combinedSnrDb();
        const auto   measured = [](double db) noexcept { return std::isfinite(db) && db > gr::math::kDbFloor; };
        const bool   has0     = measured(branch0);
        const bool   has1     = measured(branch1);
        const bool   hasC     = measured(combined);
        const double gain     = (has0 && has1) ? combined - std::max(branch0, branch1) : combined - (has0 ? branch0 : branch1);
        const bool   hasGain  = hasC && (has0 || has1) && std::isfinite(gain);

        DbFigures figures;
        figures.finite   = has0 && has1 && hasC && hasGain;
        figures.branch0  = has0 ? branch0 : 0.;
        figures.branch1  = has1 ? branch1 : 0.;
        figures.combined = hasC ? combined : 0.;
        figures.gain     = hasGain ? gain : 0.;
        return figures;
    }

    /// @brief Publishes the figures of one window and its coverage, @p filled over the window in force.
    void publishSlot(const gr::measurement::PolarizationEstimate& e, const DbFigures& figures, std::uint64_t filled) {
        const double coverage = std::min(1., static_cast<double>(filled) / static_cast<double>(_window));
        _slot.publish({e.relativePhase, e.amplitudeRatio, figures.branch0, figures.branch1, figures.combined, static_cast<double>(e.selectedBranch), figures.gain, coverage}, filled);
    }

    [[nodiscard]] DataSet<float> makeRecord(const gr::measurement::PolarizationEstimate& e, const DbFigures& figures, std::uint64_t sampleStart, std::uint64_t filled, std::uint64_t nonFinite) const {
        const std::array<gr::measurement::ScalarChannel, 6UZ> channels{{
            {"relative_phase", "Phase", "rad", static_cast<float>(e.relativePhase)},
            {"amplitude_ratio", "Ratio", "", static_cast<float>(e.amplitudeRatio)},
            {"branch0_snr_db", "SNR", "dB", static_cast<float>(figures.branch0)},
            {"branch1_snr_db", "SNR", "dB", static_cast<float>(figures.branch1)},
            {"combined_snr_db", "SNR", "dB", static_cast<float>(figures.combined)},
            {"combining_gain_db", "CombiningGain", "dB", static_cast<float>(figures.gain)},
        }};
        property_map                                          extra{
                                                     {std::pmr::string("window"), pmt::Value(static_cast<std::uint64_t>(_window))},
                                                     {std::pmr::string("mode"), pmt::Value(std::string(gr::measurement::polarizationModeName(_mode)))},
                                                     {std::pmr::string("normalize"), pmt::Value(std::string(gr::measurement::polarizationNormalizationName(_normalize)))},
                                                     {std::pmr::string("selected_branch"), pmt::Value(static_cast<std::int32_t>(e.selectedBranch))},
                                                     {std::pmr::string("n_samples"), pmt::Value(filled)},
                                                     {std::pmr::string("non_finite"), pmt::Value(nonFinite)},
                                                     {std::pmr::string("saturated_snr"), pmt::Value(e.saturatedSnr)},
                                                     {std::pmr::string("saturated_ratio"), pmt::Value(e.saturatedRatio)},
                                                     {std::pmr::string("valid"), pmt::Value(filled >= detail::kMinPolarizationWindow && figures.finite && !e.saturatedSnr && !e.saturatedRatio && nonFinite == 0ULL)},
        };
        return gr::measurement::makeScalarRecord(std::span<const gr::measurement::ScalarChannel>(channels), _sampleRate, sampleStart, std::move(extra));
    }

    /// @brief The input samples, at most @p offer, this call can take with a free record slot for every window they
    /// close. The sum is formed only when the further windows fit in the offer, which keeps it inside 64 bits.
    [[nodiscard]] std::size_t roomFor(OutputSpanLike auto& measurementsSpan, std::size_t made, std::size_t offer) const {
        const std::size_t   free        = measurementsSpan.size() > made + _pending.size() ? measurementsSpan.size() - made - _pending.size() : 0UZ;
        const std::uint64_t windowSize  = _window;
        const std::uint64_t accumulated = _covCount;
        const std::uint64_t untilClose  = windowSize > accumulated ? windowSize - accumulated : std::uint64_t{0};
        const std::uint64_t limit       = offer;
        if (free == 0UZ) {
            return static_cast<std::size_t>(std::min<std::uint64_t>(untilClose > 0U ? untilClose - 1U : 0U, limit));
        }
        const std::uint64_t further = free - 1UZ;
        if (further > limit / windowSize) {
            return offer;
        }
        return static_cast<std::size_t>(std::min<std::uint64_t>(untilClose + further * windowSize, limit));
    }

    [[nodiscard]] std::size_t drain(OutputSpanLike auto& measurementsSpan, std::size_t at) {
        if (_pending.empty() || !measurementsSpan.isConnected || at >= measurementsSpan.size()) {
            return 0UZ;
        }
        const std::size_t made = std::min(_pending.size(), measurementsSpan.size() - at);
        for (std::size_t k = 0UZ; k < made; ++k) {
            measurementsSpan[at + k] = std::move(_pending[k]);
        }
        _pending.erase(_pending.begin(), _pending.begin() + static_cast<std::ptrdiff_t>(made));
        return made;
    }
};

} // namespace gr::blocks::measurement

#endif // GNURADIO_MEASUREMENT_POLARIZATION_COMBINER_HPP
