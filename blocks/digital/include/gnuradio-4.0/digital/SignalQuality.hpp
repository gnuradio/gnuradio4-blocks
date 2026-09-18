#ifndef GNURADIO_SIGNAL_QUALITY_HPP
#define GNURADIO_SIGNAL_QUALITY_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/MeasurementRecord.hpp>
#include <gnuradio-4.0/algorithm/MeasurementSlot.hpp>
#include <gnuradio-4.0/algorithm/digital/Constellation.hpp>

#include <gnuradio-4.0/digital/ConstellationSettings.hpp>

namespace gr::blocks::digital {

namespace detail {

template<std::size_t kValues>
using MeasurementSlot = gr::measurement::MeasurementSlot<kValues>;

/// The mean power of a constellation's points after its declared normalization: one under `power`, the default.
template<std::floating_point F>
[[nodiscard]] double referencePower(const gr::digital::Constellation<F>& constellation) noexcept {
    double total = 0.;
    for (const std::complex<F>& point : constellation.points()) {
        const double re = static_cast<double>(point.real());
        const double im = static_cast<double>(point.imag());
        total += re * re + im * im;
    }
    return total / static_cast<double>(constellation.points().size());
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::digital::EvmMeter, [T], [float])

template<std::floating_point F>
struct EvmMeter : Block<EvmMeter<F>> {
    using Description = Doc<R""(
@brief Error vector magnitude and modulation error ratio over a window of decided symbols, readable from another thread.

A sink. Each symbol is compared against the constellation point nearest it, and the window's mean squared distance,
divided by the constellation's own mean symbol power, is the error vector magnitude:

    evm_rms = sqrt(mean |z - d(z)|^2 / P_ref)    evm_db = 20*log10(evm_rms)    mer_db = -evm_db

The denominator is the reference average power, which is the convention conformance figures are quoted against; peak
power would read 0.471 dB better on 16QAM and identically on any constant-modulus constellation. `evmPeak()` is the
worst single symbol of the window, the figure a conformance mask quotes beside the mean.

Windows are stream absolute: a boundary falls at a fixed symbol index whatever the call sizes are, so a reading does
not depend on how the stream was chunked. Until the first window closes the partial accumulation is published and
`coverage()` reports the fraction of a window it covers.

The optional `records` port carries the same reading as one `DataSet<float>` per closed window, stamped with the
window's first input symbol; a stream that ends mid-window emits a final record marked with the symbols it actually
covers.

The input is one complex sample per decided symbol, carrier locked and scaled to the constellation's own geometry -
the stream `SymbolSync` and the carrier loops deliver. This block does not scale it.
)"">;

    PortIn<std::complex<F>> in;
    /// One record per closed window, for a consumer outside C++. The port is optional: leaving it unconnected costs
    /// nothing and the readers below remain the whole interface for a graph that polls.
    PortOut<DataSet<float>, Async, Optional> records;

    Annotated<std::string, "constellation", detail::ConstellationSettingsDoc, Visible> constellation = std::string("qpsk");
    Annotated<gr::Size_t, "arity", Unit<"symbols">, Doc<"M, for 'psk' and 'qam'">>     arity         = 4U;
    Annotated<F, "phase_offset", Unit<"rad">, Doc<"rotation of the 'psk' point set">>  phase_offset  = F{0};
    Annotated<gr::Size_t, "label_xor", Doc<"label offset of the 'psk' point set">>     label_xor     = 0U;
    Annotated<std::vector<F>, "points", Doc<"interleaved re,im for 'custom'">>         points{};
    Annotated<std::string, "normalization", Doc<"'power', 'amplitude' or 'none'">>     normalization = std::string("power");

    Annotated<gr::Size_t, "window", Unit<"symbols">, Visible, Doc<"symbols per reading; a change restarts the window">> window = 1000U;
    Annotated<std::string, "mode", Doc<"'decision_directed'; a known-reference mode would arrive under this name">>     mode   = std::string("decision_directed");

    GR_MAKE_REFLECTABLE(EvmMeter, in, records, constellation, arity, phase_offset, label_xor, points, normalization, window, mode);

    gr::digital::Constellation<F> _constellation  = gr::digital::Constellation<F>::qpsk();
    double                        _referencePower = 1.;
    double                        _sumSquared     = 0.;
    double                        _peakSquared    = 0.;
    std::size_t                   _filled         = 0UZ;
    std::size_t                   _window         = 1000UZ;
    bool                          _closedOnce     = false;

    /// values[0] is the window's mean squared error, values[1] the worst single symbol's, both already divided by the
    /// reference power
    detail::MeasurementSlot<2UZ> _slot{};
    /// A reader touches these two as well, so they carry the same guarantee as the published values.
    std::atomic<std::uint64_t> _windowSize{1000ULL};
    std::atomic<std::uint64_t> _windows{0ULL};
    std::uint64_t              _symbolsSeen{0ULL}; ///< symbols consumed, which is where the next window starts

    std::vector<DataSet<float>> _pending{};           ///< records built at a window's close and not yet published
    std::uint64_t               _windowStartAt{0ULL}; ///< first symbol of the window now filling, a record's abscissa
    bool                        _flushed{false};      ///< the end-of-stream record has gone out

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() {
        rebuild();
        // The framework runs `processEpilogue` only over a non-empty trailing span, and that epilogue is what reports
        // the window in progress when the stream ends. `processBulk` therefore leaves the last symbol of a call
        // unconsumed, and asking for two keeps that from stalling the steady state.
        in.min_samples = 2UZ;
    }

    void rebuild() {
        if (mode != "decision_directed") {
            throw gr::exception(std::format("mode must be 'decision_directed', got '{}'", mode.value));
        }
        if (window < 1U) {
            throw gr::exception("window must hold at least one symbol");
        }
        _constellation  = detail::build<F>(constellation, arity, phase_offset, label_xor, std::span<const F>(points.value), normalization);
        _referencePower = detail::referencePower(_constellation);
        if (!(_referencePower > 0.)) {
            throw gr::exception("the constellation's points carry no power to measure against");
        }
        _window = static_cast<std::size_t>(window.value);
        _windowSize.store(static_cast<std::uint64_t>(_window), std::memory_order_relaxed);
        restart();
    }

    void restart() noexcept {
        _sumSquared  = 0.;
        _peakSquared = 0.;
        _filled      = 0UZ;
        _closedOnce  = false;
        _windows.store(0ULL, std::memory_order_relaxed);
        _symbolsSeen   = 0ULL;
        _windowStartAt = 0ULL;
        _flushed       = false;
        _pending.clear();
        _slot.publish({0., 0.}, 0ULL);
    }

    /// @brief Root mean square error vector magnitude of the last window, as a ratio. Callable from any thread.
    [[nodiscard]] double evmRms() const noexcept { return std::sqrt(_slot.read().first[0]); }

    /// @brief The same reading as a percentage, which is how a mask is usually written.
    [[nodiscard]] double evmPercent() const noexcept { return 100. * evmRms(); }

    /// @brief The same reading in decibels; -inf is impossible because a zero reading returns the floor below.
    [[nodiscard]] double evmDb() const noexcept { return 20. * std::log10(std::max(evmRms(), kFloor)); }

    /// @brief Modulation error ratio, the same quantity with the opposite sign.
    [[nodiscard]] double merDb() const noexcept { return -evmDb(); }

    /// @brief The worst single symbol of the window, as a ratio.
    [[nodiscard]] double evmPeak() const noexcept { return std::sqrt(_slot.read().first[1]); }

    /// @brief Fraction of a window the reading covers, 0 to 1. Callable from any thread.
    [[nodiscard]] double coverage() const noexcept { return std::min(1., static_cast<double>(_slot.read().second) / static_cast<double>(_windowSize.load(std::memory_order_relaxed))); }

    /// @brief Windows completed since the last settings change, which is how a reader tells a fresh reading from a repeat.
    [[nodiscard]] std::uint64_t nWindows() const noexcept { return _windows.load(std::memory_order_relaxed); }

    /// @brief The window's reading as a record, on the module's measurement conventions. Symbols, so no sample rate.
    [[nodiscard]] DataSet<float> makeRecord() const {
        const std::array<gr::measurement::ScalarChannel, 6UZ> channels{{
            {"evm_rms", "ErrorVectorMagnitude", "1", static_cast<float>(evmRms())},
            {"evm_percent", "ErrorVectorMagnitude", "%", static_cast<float>(evmPercent())},
            {"evm_db", "ErrorVectorMagnitude", "dB", static_cast<float>(evmDb())},
            {"mer_db", "ModulationErrorRatio", "dB", static_cast<float>(merDb())},
            {"evm_peak", "ErrorVectorMagnitude", "1", static_cast<float>(evmPeak())},
            {"coverage", "Coverage", "1", static_cast<float>(coverage())},
        }};
        return gr::measurement::makeScalarRecord(std::span<const gr::measurement::ScalarChannel>(channels), std::numeric_limits<float>::quiet_NaN(), _windowStartAt, property_map{{std::pmr::string("n_windows"), pmt::Value(nWindows())}, {std::pmr::string("window"), pmt::Value(static_cast<std::uint64_t>(_window))}, {std::pmr::string("index_unit"), pmt::Value(std::string("symbol"))}});
    }

    /**
     * @brief One record per window that closes, never one per call: several windows can close inside one chunk and
     * each is its own reading.
     *
     * A symbol is consumed only once it has been folded in, and no more are taken than the output span has room for
     * the records they would close, so a slow consumer stalls the input rather than growing a backlog here.
     */
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::span<const std::complex<F>> input(inSpan);
        std::size_t                            made = drain(outSpan, 0UZ);

        // The last symbol of a call is held back so the end-of-stream epilogue always has a span to run on. A call
        // carrying a single symbol is one a caller drove by hand rather than one the framework composed, and takes it.
        const std::size_t offer = input.size() >= 2UZ ? input.size() - 1UZ : input.size();
        const std::size_t take  = std::min(offer, roomFor(outSpan, made));

        accumulate(input.first(take), outSpan.isConnected);
        made += drain(outSpan, made);

        outSpan.publish(made);
        std::ignore = inSpan.consume(take);
        return take == 0UZ && made == 0UZ && !input.empty() ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }

    /// @brief End of stream: fold the trailing symbols, then report the window in progress with the symbol count that
    /// actually reached it, which is what that record's `coverage` states.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        accumulate(std::span<const std::complex<F>>(inSpan), outSpan.isConnected);
        std::size_t made = drain(outSpan, 0UZ);

        if (!_flushed && _filled > 0UZ) {
            publishWindow(); // a partial window is reported, never suppressed
            if (outSpan.isConnected && made < outSpan.size()) {
                outSpan[made] = makeRecord();
                ++made;
            }
            _flushed = true;
        }
        outSpan.publish(made);
        return work::Status::OK;
    }

private:
    /// The smallest ratio evmDb() will take a logarithm of, which is -400 dB.
    static constexpr double kFloor = 1e-20;

    void accumulate(std::span<const std::complex<F>> input, bool wantRecords) {
        for (const std::complex<F> sample : input) {
            const std::complex<F> decided = _constellation.point(_constellation.hardDecision(sample));
            const double          re      = static_cast<double>(sample.real()) - static_cast<double>(decided.real());
            const double          im      = static_cast<double>(sample.imag()) - static_cast<double>(decided.imag());
            const double          squared = re * re + im * im;

            _sumSquared += squared;
            _peakSquared = std::max(_peakSquared, squared);
            ++_filled;
            ++_symbolsSeen;

            if (_filled == _window) {
                publishWindow();
                _windows.fetch_add(1ULL, std::memory_order_relaxed);
                if (wantRecords) { // built here, so the record states the window that closed and where it began
                    _pending.push_back(makeRecord());
                }
                _sumSquared    = 0.;
                _peakSquared   = 0.;
                _filled        = 0UZ;
                _closedOnce    = true;
                _windowStartAt = _symbolsSeen;
            }
        }
        if (!_closedOnce && _filled > 0UZ) { // a partial first window is reported rather than suppressed
            publishWindow();
        }
    }

    /// @brief How many symbols may be taken before a record would have nowhere to go.
    [[nodiscard]] std::size_t roomFor(OutputSpanLike auto& outSpan, std::size_t made) const {
        if (!outSpan.isConnected) {
            return std::numeric_limits<std::size_t>::max();
        }
        const std::size_t free       = outSpan.size() > made + _pending.size() ? outSpan.size() - made - _pending.size() : 0UZ;
        const std::size_t untilClose = _window > _filled ? _window - _filled : 0UZ;
        if (free == 0UZ) {
            return untilClose > 0UZ ? untilClose - 1UZ : 0UZ;
        }
        return untilClose + (free - 1UZ) * _window;
    }

    /// @brief Move what has been built into the output span from @p at, and report how many went.
    [[nodiscard]] std::size_t drain(OutputSpanLike auto& outSpan, std::size_t at) {
        if (_pending.empty() || !outSpan.isConnected || at >= outSpan.size()) {
            return 0UZ;
        }
        const std::size_t made = std::min(_pending.size(), outSpan.size() - at);
        for (std::size_t k = 0UZ; k < made; ++k) {
            outSpan[at + k] = std::move(_pending[k]);
        }
        _pending.erase(_pending.begin(), _pending.begin() + static_cast<std::ptrdiff_t>(made));
        return made;
    }

    void publishWindow() noexcept {
        const double count = static_cast<double>(_filled);
        _slot.publish({_sumSquared / (count * _referencePower), _peakSquared / _referencePower}, static_cast<std::uint64_t>(_filled));
    }
};

GR_REGISTER_BLOCK(gr::blocks::digital::SnrEstimator, [T], [float])

template<std::floating_point F>
struct SnrEstimator : Block<SnrEstimator<F>> {
    using Description = Doc<R""(
@brief Signal-to-noise ratio over a window of symbols, by moments or by decision, readable from another thread.

A sink, with two methods.

`m2m4` is blind: from the second and fourth moments `M2 = mean|z|^2` and `M4 = mean|z|^4` of a constant-modulus
signal in circular complex Gaussian noise,

    S = sqrt(2*M2^2 - M4)    N = M2 - S    snr = S/N

The derivation assumes a signal kurtosis of one, which is what constant modulus means, so on a constellation that is
not constant modulus the estimate is biased and the bias is a property of the constellation rather than of the noise.
A finite window at low signal-to-noise ratio can drive `2*M2^2 - M4` below zero; that window reports zero and counts
itself in `nDegenerate()`.

`decision_directed` is the linear modulation error ratio, `P_ref / mean|z - d(z)|^2`, and reads high once the
decisions themselves start to err.

Windows are stream absolute, `coverage()` reports a partial first window, and `snrDb()` is clamped to `ceiling_db`
either side, so no infinity or NaN reaches a reader.

The optional `records` port carries the same reading as one `DataSet<float>` per closed window, stamped with the
window's first input symbol; a stream that ends mid-window emits a final record marked with the symbols it actually
covers.

The input is one complex sample per decided symbol, carrier locked and scaled to the constellation's own geometry.
This block does not scale it.
)"">;

    PortIn<std::complex<F>>                  in;
    PortOut<DataSet<float>, Async, Optional> records;

    Annotated<std::string, "method", Visible, Doc<"'m2m4', blind and constant-modulus, or 'decision_directed'">>        method     = std::string("m2m4");
    Annotated<gr::Size_t, "window", Unit<"symbols">, Visible, Doc<"symbols per reading; a change restarts the window">> window     = 4096U;
    Annotated<double, "ceiling_db", Unit<"dB">, Doc<"the magnitude snrDb() is clamped to, either side of zero">>        ceiling_db = 100.0;

    Annotated<std::string, "constellation", detail::ConstellationSettingsDoc, Visible> constellation = std::string("qpsk");
    Annotated<gr::Size_t, "arity", Unit<"symbols">, Doc<"M, for 'psk' and 'qam'">>     arity         = 4U;
    Annotated<F, "phase_offset", Unit<"rad">, Doc<"rotation of the 'psk' point set">>  phase_offset  = F{0};
    Annotated<gr::Size_t, "label_xor", Doc<"label offset of the 'psk' point set">>     label_xor     = 0U;
    Annotated<std::vector<F>, "points", Doc<"interleaved re,im for 'custom'">>         points{};
    Annotated<std::string, "normalization", Doc<"'power', 'amplitude' or 'none'">>     normalization = std::string("power");

    GR_MAKE_REFLECTABLE(SnrEstimator, in, records, method, window, ceiling_db, constellation, arity, phase_offset, label_xor, points, normalization);

    gr::digital::Constellation<F> _constellation    = gr::digital::Constellation<F>::qpsk();
    double                        _referencePower   = 1.;
    bool                          _decisionDirected = false;
    double                        _sumSquared       = 0.; ///< sum |z|^2, or sum |z - d(z)|^2 when decision directed
    double                        _sumFourth        = 0.; ///< sum |z|^4, m2m4 only
    std::size_t                   _filled           = 0UZ;
    std::size_t                   _window           = 4096UZ;
    bool                          _closedOnce       = false;

    /// values[0] is the linear signal-to-noise ratio of the last window
    detail::MeasurementSlot<1UZ> _slot{};
    /// A reader touches these as well, so they carry the same guarantee as the published values.
    std::atomic<std::uint64_t> _windowSize{4096ULL};
    std::atomic<double>        _ceiling{100.0};
    std::atomic<std::uint64_t> _windows{0ULL};
    std::uint64_t              _symbolsSeen{0ULL}; ///< symbols consumed, which is where the next window starts
    std::atomic<std::uint64_t> _degenerate{0ULL};

    std::vector<DataSet<float>> _pending{};           ///< records built at a window's close and not yet published
    std::uint64_t               _windowStartAt{0ULL}; ///< first symbol of the window now filling, a record's abscissa
    bool                        _flushed{false};      ///< the end-of-stream record has gone out

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() {
        rebuild();
        in.min_samples = 2UZ;
    }

    void rebuild() {
        if (method != "m2m4" && method != "decision_directed") {
            throw gr::exception(std::format("method must be 'm2m4' or 'decision_directed', got '{}'", method.value));
        }
        if (window < 1U) {
            throw gr::exception("window must hold at least one symbol");
        }
        if (!(ceiling_db > 0.)) {
            throw gr::exception(std::format("ceiling_db is a magnitude and must be positive, got {}", ceiling_db.value));
        }
        // The vocabulary is validated whichever method runs, so a name that is wrong is heard about rather than
        // ignored; only the decision-directed method reads the result.
        _constellation    = detail::build<F>(constellation, arity, phase_offset, label_xor, std::span<const F>(points.value), normalization);
        _referencePower   = detail::referencePower(_constellation);
        _decisionDirected = method == "decision_directed";
        if (_decisionDirected && !(_referencePower > 0.)) {
            throw gr::exception("the constellation's points carry no power to measure against");
        }
        _window = static_cast<std::size_t>(window.value);
        _windowSize.store(static_cast<std::uint64_t>(_window), std::memory_order_relaxed);
        _ceiling.store(ceiling_db, std::memory_order_relaxed);
        restart();
    }

    void restart() noexcept {
        _sumSquared = 0.;
        _sumFourth  = 0.;
        _filled     = 0UZ;
        _closedOnce = false;
        _windows.store(0ULL, std::memory_order_relaxed);
        _symbolsSeen = 0ULL;
        _degenerate.store(0ULL, std::memory_order_relaxed);
        _windowStartAt = 0ULL;
        _flushed       = false;
        _pending.clear();
        _slot.publish({0.}, 0ULL);
    }

    /// @brief The last window's signal-to-noise ratio as a power ratio. Callable from any thread.
    [[nodiscard]] double snrLinear() const noexcept { return _slot.read().first[0]; }

    /// @brief The same reading in decibels, clamped to +-ceiling_db so no infinity escapes.
    [[nodiscard]] double snrDb() const noexcept {
        const double ceiling = _ceiling.load(std::memory_order_relaxed); // read once, so a settings change cannot invert the clamp
        const double ratio   = snrLinear();
        if (!(ratio > 0.)) {
            return -ceiling;
        }
        return std::clamp(10. * std::log10(ratio), -ceiling, ceiling);
    }

    /// @brief Fraction of a window the reading covers, 0 to 1. Callable from any thread.
    [[nodiscard]] double coverage() const noexcept { return std::min(1., static_cast<double>(_slot.read().second) / static_cast<double>(_windowSize.load(std::memory_order_relaxed))); }

    /// @brief Windows completed since the last settings change.
    [[nodiscard]] std::uint64_t nWindows() const noexcept { return _windows.load(std::memory_order_relaxed); }

    /// @brief Windows whose moments admitted no positive signal power, reported as zero rather than as a NaN.
    [[nodiscard]] std::uint64_t nDegenerate() const noexcept { return _degenerate.load(std::memory_order_relaxed); }

    /// @brief The window's reading as a record, on the module's measurement conventions. Symbols, so no sample rate.
    [[nodiscard]] DataSet<float> makeRecord() const {
        const std::array<gr::measurement::ScalarChannel, 3UZ> channels{{
            {"snr_linear", "SignalToNoiseRatio", "1", static_cast<float>(snrLinear())},
            {"snr_db", "SignalToNoiseRatio", "dB", static_cast<float>(snrDb())},
            {"coverage", "Coverage", "1", static_cast<float>(coverage())},
        }};
        return gr::measurement::makeScalarRecord(std::span<const gr::measurement::ScalarChannel>(channels), std::numeric_limits<float>::quiet_NaN(), _windowStartAt, property_map{{std::pmr::string("n_windows"), pmt::Value(nWindows())}, {std::pmr::string("n_degenerate"), pmt::Value(nDegenerate())}, {std::pmr::string("window"), pmt::Value(static_cast<std::uint64_t>(_window))}, {std::pmr::string("index_unit"), pmt::Value(std::string("symbol"))}});
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::span<const std::complex<F>> input(inSpan);
        std::size_t                            made = drain(outSpan, 0UZ);

        const std::size_t offer = input.size() >= 2UZ ? input.size() - 1UZ : input.size();
        const std::size_t take  = std::min(offer, roomFor(outSpan, made));

        accumulate(input.first(take), outSpan.isConnected);
        made += drain(outSpan, made);

        outSpan.publish(made);
        std::ignore = inSpan.consume(take);
        return take == 0UZ && made == 0UZ && !input.empty() ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }

    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        accumulate(std::span<const std::complex<F>>(inSpan), outSpan.isConnected);
        std::size_t made = drain(outSpan, 0UZ);

        if (!_flushed && _filled > 0UZ) {
            publishWindow();
            if (outSpan.isConnected && made < outSpan.size()) {
                outSpan[made] = makeRecord();
                ++made;
            }
            _flushed = true;
        }
        outSpan.publish(made);
        return work::Status::OK;
    }

private:
    void accumulate(std::span<const std::complex<F>> input, bool wantRecords) {
        for (const std::complex<F> sample : input) {
            if (_decisionDirected) {
                const std::complex<F> decided = _constellation.point(_constellation.hardDecision(sample));
                const double          re      = static_cast<double>(sample.real()) - static_cast<double>(decided.real());
                const double          im      = static_cast<double>(sample.imag()) - static_cast<double>(decided.imag());
                _sumSquared += re * re + im * im;
            } else {
                const double re      = static_cast<double>(sample.real());
                const double im      = static_cast<double>(sample.imag());
                const double squared = re * re + im * im;
                _sumSquared += squared;
                _sumFourth += squared * squared;
            }
            ++_filled;
            ++_symbolsSeen;

            if (_filled == _window) {
                publishWindow();
                _windows.fetch_add(1ULL, std::memory_order_relaxed);
                if (wantRecords) {
                    _pending.push_back(makeRecord());
                }
                _sumSquared    = 0.;
                _sumFourth     = 0.;
                _filled        = 0UZ;
                _closedOnce    = true;
                _windowStartAt = _symbolsSeen;
            }
        }
        if (!_closedOnce && _filled > 0UZ) {
            publishWindow();
        }
    }

    [[nodiscard]] std::size_t roomFor(OutputSpanLike auto& outSpan, std::size_t made) const {
        if (!outSpan.isConnected) {
            return std::numeric_limits<std::size_t>::max();
        }
        const std::size_t free       = outSpan.size() > made + _pending.size() ? outSpan.size() - made - _pending.size() : 0UZ;
        const std::size_t untilClose = _window > _filled ? _window - _filled : 0UZ;
        if (free == 0UZ) {
            return untilClose > 0UZ ? untilClose - 1UZ : 0UZ;
        }
        return untilClose + (free - 1UZ) * _window;
    }

    [[nodiscard]] std::size_t drain(OutputSpanLike auto& outSpan, std::size_t at) {
        if (_pending.empty() || !outSpan.isConnected || at >= outSpan.size()) {
            return 0UZ;
        }
        const std::size_t made = std::min(_pending.size(), outSpan.size() - at);
        for (std::size_t k = 0UZ; k < made; ++k) {
            outSpan[at + k] = std::move(_pending[k]);
        }
        _pending.erase(_pending.begin(), _pending.begin() + static_cast<std::ptrdiff_t>(made));
        return made;
    }

    void publishWindow() noexcept {
        const double count = static_cast<double>(_filled);
        double       ratio = 0.;

        if (_decisionDirected) {
            const double meanSquaredError = _sumSquared / count;
            ratio                         = meanSquaredError > 0. ? _referencePower / meanSquaredError : std::pow(10., 0.1 * ceiling_db);
        } else {
            const double m2   = _sumSquared / count;
            const double m4   = _sumFourth / count;
            const double disc = 2. * m2 * m2 - m4;
            if (disc <= 0.) {
                _degenerate.fetch_add(1ULL, std::memory_order_relaxed);
            } else {
                const double signal = std::sqrt(disc);
                const double noise  = m2 - signal;
                ratio               = noise > 0. ? signal / noise : std::pow(10., 0.1 * ceiling_db);
            }
        }
        _slot.publish({ratio}, static_cast<std::uint64_t>(_filled));
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_SIGNAL_QUALITY_HPP
