#ifndef GNURADIO_MEASUREMENT_AUTOCORRELATION_HPP
#define GNURADIO_MEASUREMENT_AUTOCORRELATION_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/algorithm/analysis/Autocorrelation.hpp>
#include <gnuradio-4.0/annotated.hpp>
#include <gnuradio-4.0/measurement/SpectralEstimate.hpp>

namespace gr::blocks::measurement {

namespace detail {

/// @brief Progress of one `processBulk` or `processEpilogue` call, with a separate record count for the phase port.
struct AcfProgress {
    std::size_t taken      = 0UZ;   ///< input samples moved into the pending window
    std::size_t made       = 0UZ;   ///< records written to the correlation output span
    std::size_t madePhase  = 0UZ;   ///< records written to the phase output span
    bool        outputFull = false; ///< a ready window could complete a record, and the output had no room for it
};

/// @brief Builds one record on the module's conventions, with a lag axis in seconds.
[[nodiscard]] inline DataSet<float> makeAcfRecord(std::span<const float> values, float sampleRate, std::string_view signalName, std::string_view quantity, std::string_view unit, const property_map& meta) {
    DataSet<float>    ds;
    const std::size_t lags = values.size();

    ds.extents = {static_cast<std::int32_t>(lags)};
    ds.layout  = gr::LayoutRight{};

    ds.axis_names = {"Lag"};
    ds.axis_units = {"s"};
    ds.axis_values.resize(1UZ);
    ds.axis_values[0UZ].resize(lags);
    for (std::size_t k = 0UZ; k < lags; ++k) {
        ds.axis_values[0UZ][k] = static_cast<float>(k) / sampleRate;
    }

    ds.signal_names      = {std::string(signalName)};
    ds.signal_quantities = {std::string(quantity)};
    ds.signal_units      = {std::string(unit)};
    ds.signal_values.assign(values.begin(), values.end());
    ds.signal_ranges.resize(1UZ);

    ds.meta_information.resize(1UZ);
    ds.meta_information[0UZ] = meta;
    ds.timing_events.resize(1UZ);
    ds.timestamp = 0;
    return ds;
}

/// @brief Rejects a `max_lag` of at least `window_length`, naming both values.
inline void requireAcfLag(gr::Size_t maxLag, gr::Size_t windowLength) {
    if (maxLag >= windowLength) {
        throw gr::exception(std::format("max_lag ({}) must be below window_length ({})", maxLag, windowLength));
    }
}

/// @brief Rejects a sample rate that is not positive and finite.
inline void requireAcfSampleRate(float sampleRate) {
    if (!(sampleRate > 0.f) || !std::isfinite(sampleRate)) {
        throw gr::exception(std::format("sample_rate must be positive and finite, got {}", sampleRate));
    }
}

/// @brief The setting values the block computes on, as the last accepted application left them.
struct AcfAccepted {
    gr::analysis::AcfConfig config{};
    std::string             window{};
    bool                    emitPhase{};
    float                   sampleRate{};
    std::string             signalName{};
};

/// @brief The refusal of a staged-restart key that changes while the block runs.
[[nodiscard]] inline gr::exception runningAcfKeyRefusal(std::string_view key) { return gr::exception(std::format("{} is a staged-restart setting; stop the block to change it", key)); }

/// @brief The validator of `false_alarm_rate`: true strictly between 0 and 1.
inline constexpr auto kFalseAlarmRateInRange = [](double value) { return value > 0.0 && value < 1.0; };

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::measurement::Autocorrelation, [T], [ float, std::complex<float> ])

/**
 * @brief Publishes one autocorrelation record per averaged group of windows, with the scatter and detection threshold.
 *
 * The block shows the lags at which a signal repeats, such as an OFDM cyclic prefix or a symbol period. It takes a
 * `float` or `std::complex<float>` stream, cuts it into windows of `window_length` samples, and averages the
 * correlation of `n_averages` windows into one record. A record covers `window_length + (n_averages - 1) hop` distinct
 * samples and goes out every `n_averages` hops.
 *
 * `out` carries `|R(tau)|/R(0)` over lags 0 to `max_lag` on a lag axis in seconds. Under the complex kind with
 * `emit_phase` set, `phase` carries `arg R(tau)` in radians, named `signal_name` with `_phase` added. Each record's
 * metadata states the settings it was computed with, `sample_rate`, `sample_start`, a `sequence` number, `hop`,
 * `transform_length`, the averaged windows as `n_averaged` and the distinct samples as `n_samples`. `zero_lag` is
 * `R(0)` of the correlated sequence, in the input's units squared under the complex kind and to the fourth power under
 * the envelope kind. `scatter` is the standard deviation of a noise record at its noisiest lag, `scatter_lag`, and
 * `n_pairs` is the effective count of independent products behind that lag. `real_valued` marks a real estimate. The
 * outputs carry no input tag.
 *
 * Noise alone clears `detection_threshold` at some lag in a fraction `false_alarm_rate` of records.
 * `peak_threshold_db` is a margin in decibels over the record's own median, and noise alone clears it at some lag in
 * at most that fraction of records. Both figures assume a Gaussian tail. At lags with few effective products, as at
 * the longest lags under the unbiased normalization, noise clears them more often. Under the envelope kind with
 * `remove_mean` false, every noise lag carries the squared mean of the envelope, and both figures are infinite.
 *
 * `kind` 'complex' estimates `R(tau) = E[x(t) x*(t - tau)]` on the samples. It is coherent: its phase at a known lag is
 * a frequency estimate, and a carrier offset leaves its magnitude unchanged. `kind` 'envelope' correlates the
 * mean-removed `|x|^2`, which oscillator phase noise leaves unchanged. An OFDM cyclic prefix shows under both kinds,
 * and a shaped linear modulation's symbol period only under the envelope. With `remove_mean` set, a constant-modulus
 * window has no envelope fluctuation and adds to no record under the envelope kind, and a stream of such windows
 * publishes no record.
 *
 * `normalization` 'unbiased' divides lag `tau` by the taper's own autocorrelation `r_w(tau)` and estimates `R(tau)`.
 * 'biased' divides every lag by `r_w(0)`. Its variance falls with the lag, and the estimate is positive-semidefinite.
 * `window` names the taper, and `overlap` sets the share of a window the next one repeats. `remove_mean` removes each
 * window's mean and adds back the share of the zero lag that the removal takes from a noise lag. A noise lag then has
 * zero mean, and a signal's lag moves by at most `1/(window_length - 1)`.
 *
 * `window_length`, `max_lag`, `kind`, `normalization`, `overlap`, `window` and `remove_mean` take effect at the next
 * start, and a change to one of them while the block runs is refused. `max_lag` stays below `window_length`.
 * `n_averages`, `false_alarm_rate`, `emit_phase`, `signal_name` and `sample_rate` apply while the block runs. When
 * `n_averages` falls to or below the windows already pending, the next record holds them all. A new `sample_rate` takes
 * effect at the sample its tag sits on. It discards the window in progress and flushes the accumulation as a record on
 * the old axis. While the block runs, it ignores a rate that is not positive and finite, and a stopped block refuses
 * one. At the end of the stream the block publishes the accumulation in progress.
 *
 * `nSamples()` counts the input samples taken, and `nWindows()` every window the grid produced. `nRecords()` counts the
 * records of exactly `n_averages` windows, and `nLongRecords()` those of more. `nPartialFlushes()` counts the records
 * of fewer, and `nPartialWindows()` the windows behind them. `nDegenerate()` counts the windows with nothing to
 * correlate, and `nNonFinite()` those holding a NaN or an infinity. Neither kind adds to a record. `nWindowResets()`
 * counts the windows a rate change discarded, and `nRateRefused()` the rates the block ignored. `nTailDropped()` counts
 * the samples no window covered at the end of the stream, and `nRecordsDropped()` the records it found no room for.
 */
template<typename T>
requires(std::is_same_v<T, float> || std::is_same_v<T, std::complex<float>>)
struct Autocorrelation : Block<Autocorrelation<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Computes a windowed, averaged autocorrelation with its scatter and detection threshold.

Each DataSet record holds |R(tau)|/R(0) over lags 0..max_lag from n_averages windows. kind selects the complex
estimator or the envelope estimator. The record states the threshold that noise alone clears in a fraction
false_alarm_rate of records. It passes no input tag on.
)"">;
    using Real        = float;

    PortIn<T>                               in;
    PortOut<DataSet<Real>, Async>           out;   ///< `|R(tau)|/R(0)` over lags 0..max_lag
    PortOut<DataSet<Real>, Async, Optional> phase; ///< `arg R(tau)`, fed only when `emit_phase` is set and `kind` is complex

    Annotated<gr::Size_t, "window_length", Visible, Doc<"samples per window, in [16, 1048576]">, Limits<16U, 1048576U>>                                                   window_length    = 4096U;
    Annotated<gr::Size_t, "max_lag", Visible, Doc<"longest published lag in samples, below window_length">, Limits<1U, std::numeric_limits<gr::Size_t>::max()>>           max_lag          = 1024U;
    Annotated<std::string, "kind", Visible, Doc<"estimator kind, 'complex' or 'envelope'">>                                                                               kind             = std::string("envelope");
    Annotated<std::string, "normalization", Visible, Doc<"lag divisor, 'unbiased' or 'biased'">>                                                                          normalization    = std::string("unbiased");
    Annotated<gr::Size_t, "n_averages", Visible, Doc<"windows per emitted record">, Limits<1U, std::numeric_limits<gr::Size_t>::max()>>                                   n_averages       = 16U;
    Annotated<double, "overlap", Visible, Doc<"fraction of a window shared with the next, in [0, 1)">, Limits<0.0, 1.0, [](double v) { return v < 1.0; }>>                overlap          = 0.5;
    Annotated<bool, "remove_mean", Visible, Doc<"remove each window's mean before correlating">>                                                                          remove_mean      = true;
    Annotated<std::string, "window", Visible, Doc<gr::algorithm::window::TypeNames>>                                                                                      window           = std::string("Rectangular");
    Annotated<double, "false_alarm_rate", Visible, Doc<"probability per record that noise alone clears the threshold">, Limits<0.0, 1.0, detail::kFalseAlarmRateInRange>> false_alarm_rate = 1e-3;
    Annotated<bool, "emit_phase", Visible, Doc<"publish arg R(tau) on the phase port, complex kind only">>                                                                emit_phase       = false;
    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"input sample rate">>                                                                                        sample_rate      = 1.f;
    Annotated<std::string, "signal_name", Doc<"signal name of the emitted record, with _phase on the phase record">>                                                      signal_name      = std::string("acf");

    GR_MAKE_REFLECTABLE(Autocorrelation, in, out, phase, window_length, max_lag, kind, normalization, n_averages, overlap, remove_mean, window, false_alarm_rate, emit_phase, sample_rate, signal_name);

    gr::analysis::Autocorrelation<T> _kernel{};
    std::vector<T>                   _pending{};                      ///< samples taken and not yet covered by a whole window
    detail::AcfAccepted              _accepted  = acceptedDefaults(); ///< the values the block computes on
    bool                             _running   = false;
    bool                             _flushed   = false;
    float                            _rate      = 1.f; ///< the rate in force for the windows in the accumulation
    bool                             _rateMoved = false;
    std::uint64_t                    _sequence  = 0ULL;

    std::atomic<std::uint64_t> _nSamples{0ULL};
    std::atomic<std::uint64_t> _nWindowResets{0ULL};
    std::atomic<std::uint64_t> _nRateRefused{0ULL};
    std::atomic<std::uint64_t> _nTailDropped{0ULL};
    std::atomic<std::uint64_t> _nRecordsDropped{0ULL};

    /// @brief Checks a moved `kind`, `normalization` or `window`, `max_lag` against `window_length` when either moves,
    /// and a moved `sample_rate`. While the block runs, it then refuses a change to a staged-restart value.
    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const auto          moved = [&newSettings](std::string_view key) { return newSettings.contains(std::pmr::string(key)); };
        detail::AcfAccepted next  = _accepted;
        if (moved("window_length")) {
            next.config.windowLength = static_cast<std::size_t>(window_length.value);
        }
        if (moved("max_lag")) {
            next.config.maxLag = static_cast<std::size_t>(max_lag.value);
        }
        if (moved("window_length") || moved("max_lag")) {
            detail::requireAcfLag(static_cast<gr::Size_t>(next.config.maxLag), static_cast<gr::Size_t>(next.config.windowLength));
        }
        if (moved("kind")) {
            next.config.kind = parsedKind();
        }
        if (moved("normalization")) {
            next.config.normalization = parsedNormalization();
        }
        if (moved("overlap")) {
            next.config.overlap = overlap;
        }
        if (moved("remove_mean")) {
            next.config.removeMean = remove_mean;
        }
        if (moved("window")) {
            next.config.window = detail::requireWindow(window.value);
            next.window        = window.value;
        }
        if (moved("n_averages")) {
            next.config.nAverages = static_cast<std::size_t>(n_averages.value);
        }
        if (moved("false_alarm_rate")) {
            next.config.falseAlarmRate = false_alarm_rate;
        }
        if (moved("emit_phase")) {
            next.emitPhase = emit_phase;
        }
        if (moved("signal_name")) {
            next.signalName = signal_name.value;
        }
        bool rateRefused = false;
        if (moved("sample_rate")) {
            if (_running && (!(sample_rate.value > 0.f) || !std::isfinite(sample_rate.value))) {
                rateRefused = true;
            } else {
                detail::requireAcfSampleRate(sample_rate);
                next.sampleRate = sample_rate;
            }
        }
        if (_running) {
            const auto refuseChange = [](std::string_view key, bool changes) {
                if (changes) {
                    throw detail::runningAcfKeyRefusal(key);
                }
            };
            refuseChange("window_length", next.config.windowLength != _accepted.config.windowLength);
            refuseChange("max_lag", next.config.maxLag != _accepted.config.maxLag);
            refuseChange("kind", next.config.kind != _accepted.config.kind);
            refuseChange("normalization", next.config.normalization != _accepted.config.normalization);
            refuseChange("overlap", next.config.overlap != _accepted.config.overlap);
            refuseChange("window", next.config.window != _accepted.config.window);
            refuseChange("remove_mean", next.config.removeMean != _accepted.config.removeMean);
        }

        if (rateRefused) {
            _nRateRefused.fetch_add(1ULL, std::memory_order_relaxed);
            sample_rate = _accepted.sampleRate;
        }
        if (_running) {
            if (next.config.nAverages != _accepted.config.nAverages) {
                _kernel.setAveraging(next.config.nAverages);
            }
            if (next.config.falseAlarmRate != _accepted.config.falseAlarmRate) {
                _kernel.setFalseAlarmRate(next.config.falseAlarmRate);
            }
            if (next.sampleRate != _accepted.sampleRate) {
                _rateMoved = true;
            }
        }
        _accepted = std::move(next);
    }

    /// @brief Builds the kernel from the accepted values.
    void start() {
        _kernel.prepare(_accepted.config);
        _pending.clear();
        _rate      = _accepted.sampleRate;
        _sequence  = 0ULL;
        _flushed   = false;
        _rateMoved = false;
        _running   = true;
        _nSamples.store(0ULL, std::memory_order_relaxed);
        _nWindowResets.store(0ULL, std::memory_order_relaxed);
        _nRateRefused.store(0ULL, std::memory_order_relaxed);
        _nTailDropped.store(0ULL, std::memory_order_relaxed);
        _nRecordsDropped.store(0ULL, std::memory_order_relaxed);
    }

    void stop() {
        // the pending samples past the end of the last window
        const std::uint64_t pendingEnd = _kernel.streamAt() + static_cast<std::uint64_t>(_pending.size());
        const std::uint64_t covered    = std::max(_kernel.coveredUntil(), _kernel.streamAt());
        _nTailDropped.fetch_add(pendingEnd > covered ? pendingEnd - covered : 0ULL, std::memory_order_relaxed);
        _running = false;
    }

    /// @brief Input samples this block has taken. Callable from any thread.
    [[nodiscard]] std::uint64_t nSamples() const noexcept { return _nSamples.load(std::memory_order_relaxed); }

    /// @brief Windows the grid produced, degenerate ones included.
    [[nodiscard]] std::uint64_t nWindows() const noexcept { return _kernel.nWindows; }

    /// @brief Records of exactly `n_averages` windows.
    [[nodiscard]] std::uint64_t nRecords() const noexcept { return _kernel.nRecords; }

    /// @brief Records of more than `n_averages` windows, published after a live `n_averages` fell below the windows
    /// pending.
    [[nodiscard]] std::uint64_t nLongRecords() const noexcept { return _kernel.nLongRecords; }

    /// @brief Windows in progress discarded because the rate under them moved. Callable from any thread.
    [[nodiscard]] std::uint64_t nWindowResets() const noexcept { return _nWindowResets.load(std::memory_order_relaxed); }

    /// @brief Rates applied while the block runs that are not positive and finite, and that the block ignored. Callable
    /// from any thread.
    [[nodiscard]] std::uint64_t nRateRefused() const noexcept { return _nRateRefused.load(std::memory_order_relaxed); }

    /// @brief Samples no window covered at end of stream. Callable from any thread.
    [[nodiscard]] std::uint64_t nTailDropped() const noexcept { return _nTailDropped.load(std::memory_order_relaxed); }

    /// @brief Records published with fewer than `n_averages` windows behind them.
    [[nodiscard]] std::uint64_t nPartialFlushes() const noexcept { return _kernel.nPartialFlushes; }

    /// @brief Windows behind those partial records.
    [[nodiscard]] std::uint64_t nPartialWindows() const noexcept { return _kernel.nPartialWindows; }

    /// @brief Windows with no fluctuation to correlate. They publish nothing.
    [[nodiscard]] std::uint64_t nDegenerate() const noexcept { return _kernel.nDegenerate; }

    /// @brief Windows holding a NaN or an infinity. They publish nothing.
    [[nodiscard]] std::uint64_t nNonFinite() const noexcept { return _kernel.nNonFinite; }

    /// @brief Records the end of stream found no output room for, on either port. Callable from any thread.
    [[nodiscard]] std::uint64_t nRecordsDropped() const noexcept { return _nRecordsDropped.load(std::memory_order_relaxed); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& phaseSpan) {
        const detail::AcfProgress progress = fold(inSpan, outSpan, phaseSpan, false);
        if (progress.outputFull && progress.made == 0UZ && progress.taken == 0UZ) {
            outSpan.publish(0UZ);
            phaseSpan.publish(0UZ);
            std::ignore = inSpan.consume(0UZ);
            return work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        outSpan.publish(progress.made);
        phaseSpan.publish(progress.madePhase);
        std::ignore = inSpan.consume(progress.taken);
        return work::Status::OK;
    }

    /// @brief At end of stream, folds in the trailing samples and emits the accumulation in progress. The record
    /// states how many windows it holds. A record the outputs have no room for is counted in `nRecordsDropped()`.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& phaseSpan) {
        detail::AcfProgress progress = fold(inSpan, outSpan, phaseSpan, true);
        if (!_flushed) {
            std::ignore = _kernel.flush([&](const gr::analysis::AcfResult& result) { emit(result, outSpan, phaseSpan, progress); });
            _flushed    = true;
        }
        outSpan.publish(progress.made);
        phaseSpan.publish(progress.madePhase);
        return work::Status::OK;
    }

private:
    /// @brief The defaults of the settings, read from the members at construction.
    [[nodiscard]] detail::AcfAccepted acceptedDefaults() const {
        return detail::AcfAccepted{
            .config =
                gr::analysis::AcfConfig{
                    .windowLength   = static_cast<std::size_t>(window_length.value),
                    .maxLag         = static_cast<std::size_t>(max_lag.value),
                    .kind           = parsedKind(),
                    .normalization  = parsedNormalization(),
                    .overlap        = overlap,
                    .nAverages      = static_cast<std::size_t>(n_averages.value),
                    .removeMean     = remove_mean,
                    .window         = detail::requireWindow(window.value),
                    .falseAlarmRate = false_alarm_rate,
                },
            .window     = window.value,
            .emitPhase  = emit_phase,
            .sampleRate = sample_rate,
            .signalName = signal_name.value,
        };
    }

    [[nodiscard]] gr::analysis::AcfKind parsedKind() const {
        const auto parsed = gr::analysis::acfKindFrom(kind.value);
        if (!parsed.has_value()) {
            throw gr::exception(std::format("kind must be 'complex' or 'envelope', got '{}'", kind.value));
        }
        return *parsed;
    }

    [[nodiscard]] gr::analysis::AcfNormalization parsedNormalization() const {
        const auto parsed = gr::analysis::acfNormalizationFrom(normalization.value);
        if (!parsed.has_value()) {
            throw gr::exception(std::format("normalization must be 'biased' or 'unbiased', got '{}'", normalization.value));
        }
        return *parsed;
    }

    [[nodiscard]] bool wantsPhase(const auto& phaseSpan) const noexcept { return _accepted.emitPhase && _kernel.config().kind == gr::analysis::AcfKind::Complex && phaseSpan.isConnected; }

    [[nodiscard]] bool hasRoom(const auto& outSpan, const auto& phaseSpan, const detail::AcfProgress& progress) const noexcept { return progress.made < outSpan.size() && (!wantsPhase(phaseSpan) || progress.madePhase < phaseSpan.size()); }

    /// @brief Takes input into whole windows and runs them. Before a window that may complete a record it waits for
    /// room on both outputs, except at end of stream, where @p atEnd is set and nothing follows.
    [[nodiscard]] detail::AcfProgress fold(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& phaseSpan, bool atEnd) {
        detail::AcfProgress progress{};
        if (!_kernel.configured()) {
            return progress;
        }
        const std::size_t length = _kernel.config().windowLength;

        // A rate tag takes effect before the span that starts on its sample. The grid restarts on that sample.
        if (_rateMoved) {
            if (!atEnd && _kernel.windowsPending() > 0UZ && !hasRoom(outSpan, phaseSpan, progress)) {
                progress.outputFull = true; // the change waits until both outputs have room for the flushed record
                return progress;
            }
            _rateMoved                    = false;
            const std::uint64_t restartAt = _kernel.streamAt() + static_cast<std::uint64_t>(_pending.size());
            if (!_pending.empty()) {
                // Only a window holding samples is discarded and counted. Without overlap a tag on a window boundary,
                // the stream's first sample included, leaves none.
                _nWindowResets.fetch_add(1ULL, std::memory_order_relaxed);
            }
            _pending.clear();
            std::ignore = _kernel.flush([&](const gr::analysis::AcfResult& result) { emit(result, outSpan, phaseSpan, progress); });
            _kernel.seek(restartAt);
            _rate = _accepted.sampleRate;
        }

        for (;;) {
            if (_pending.size() >= length) {
                const bool mayComplete = _kernel.windowsPending() + 1UZ >= _kernel.config().nAverages;
                if (mayComplete && !atEnd && !hasRoom(outSpan, phaseSpan, progress)) {
                    progress.outputFull = true;
                    return progress;
                }
                const std::size_t used = _kernel.process(std::span<const T>(_pending).first(length), [&](const gr::analysis::AcfResult& result) { emit(result, outSpan, phaseSpan, progress); });
                if (used == 0UZ) {
                    return progress;
                }
                _pending.erase(_pending.begin(), _pending.begin() + static_cast<std::ptrdiff_t>(used));
                continue;
            }
            if (progress.taken == inSpan.size()) {
                return progress;
            }
            const std::size_t take  = std::min(length - _pending.size(), inSpan.size() - progress.taken);
            const auto        first = inSpan.begin() + static_cast<std::ptrdiff_t>(progress.taken);
            _pending.insert(_pending.end(), first, first + static_cast<std::ptrdiff_t>(take));
            progress.taken += take;
            _nSamples.fetch_add(static_cast<std::uint64_t>(take), std::memory_order_relaxed);
        }
    }

    void emit(const gr::analysis::AcfResult& result, OutputSpanLike auto& outSpan, OutputSpanLike auto& phaseSpan, detail::AcfProgress& progress) {
        const property_map meta = metaOf(result);
        if (progress.made < outSpan.size()) {
            outSpan[progress.made] = detail::makeAcfRecord(result.magnitude, _rate, _accepted.signalName, "Autocorrelation", "", meta);
            ++progress.made;
        } else {
            _nRecordsDropped.fetch_add(1ULL, std::memory_order_relaxed);
        }
        if (wantsPhase(phaseSpan) && !result.phase.empty()) {
            if (progress.madePhase < phaseSpan.size()) {
                phaseSpan[progress.madePhase] = detail::makeAcfRecord(result.phase, _rate, _accepted.signalName + "_phase", "Phase", "rad", meta);
                ++progress.madePhase;
            } else {
                _nRecordsDropped.fetch_add(1ULL, std::memory_order_relaxed);
            }
        }
        ++_sequence;
    }

    [[nodiscard]] property_map metaOf(const gr::analysis::AcfResult& result) const {
        return property_map{
            {std::pmr::string("sample_rate"), pmt::Value(_rate)},
            {std::pmr::string("sample_start"), pmt::Value(result.sampleStart)},
            {std::pmr::string("sequence"), pmt::Value(_sequence)},
            {std::pmr::string("kind"), pmt::Value(std::string(gr::analysis::acfKindName(_kernel.config().kind)))},
            {std::pmr::string("normalization"), pmt::Value(std::string(gr::analysis::acfNormalizationName(_kernel.config().normalization)))},
            {std::pmr::string("window_length"), pmt::Value(static_cast<std::uint64_t>(_kernel.config().windowLength))},
            {std::pmr::string("max_lag"), pmt::Value(static_cast<std::uint64_t>(_kernel.config().maxLag))},
            {std::pmr::string("hop"), pmt::Value(static_cast<std::uint64_t>(_kernel.hop()))},
            {std::pmr::string("overlap"), pmt::Value(_kernel.config().overlap)},
            {std::pmr::string("window"), pmt::Value(_accepted.window)},
            {std::pmr::string("n_averaged"), pmt::Value(static_cast<std::uint64_t>(result.nAveraged))},
            {std::pmr::string("n_samples"), pmt::Value(result.nDistinctSamples)},
            // the effective independent sample products behind lag scatter_lag
            {std::pmr::string("n_pairs"), pmt::Value(result.nPairs)},
            {std::pmr::string("scatter_lag"), pmt::Value(static_cast<std::uint64_t>(result.scatterLag))},
            {std::pmr::string("mean_removed"), pmt::Value(_kernel.config().removeMean)},
            // true for a real estimate, whose thresholds use the folded normal tail
            {std::pmr::string("real_valued"), pmt::Value(result.realValued)},
            {std::pmr::string("zero_lag"), pmt::Value(result.zeroLag)},
            {std::pmr::string("scatter"), pmt::Value(result.scatter)},
            {std::pmr::string("detection_threshold"), pmt::Value(result.threshold)},
            {std::pmr::string("false_alarm_rate"), pmt::Value(_kernel.config().falseAlarmRate)},
            {std::pmr::string("peak_threshold_db"), pmt::Value(result.peakThresholdDb)},
            {std::pmr::string("transform_length"), pmt::Value(static_cast<std::uint64_t>(_kernel.transformLength()))},
        };
    }
};

} // namespace gr::blocks::measurement

#endif // GNURADIO_MEASUREMENT_AUTOCORRELATION_HPP
