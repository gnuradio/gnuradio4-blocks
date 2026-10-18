#ifndef GNURADIO_MEASUREMENT_SPECTRAL_KURTOSIS_HPP
#define GNURADIO_MEASUREMENT_SPECTRAL_KURTOSIS_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
#include <expected>
#include <format>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/MeasurementSlot.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/algorithm/fourier/window.hpp>
#include <gnuradio-4.0/algorithm/measurement/Kurtosis.hpp>
#include <gnuradio-4.0/measurement/Detectors.hpp>

#include <magic_enum.hpp>

namespace gr::blocks::measurement {

namespace detail {

inline constexpr gr::Size_t kMinSpectralKurtosisSpectra = 8U;
inline constexpr gr::Size_t kMaxSpectralKurtosisSpectra = 1U << 20U;

/// @brief The value under @p key as a `double`, read from `std::uint64_t`, `double` or `float`. `std::nullopt` when
/// the key is absent or holds another type.
[[nodiscard]] inline std::optional<double> metaNumber(const property_map& map, std::string_view key) {
    const auto it = map.find(std::pmr::string(key));
    if (it == map.end()) {
        return std::nullopt;
    }
    if (const auto* asU64 = it->second.get_if<std::uint64_t>()) {
        return static_cast<double>(*asU64);
    }
    if (const auto* asDouble = it->second.get_if<double>()) {
        return *asDouble;
    }
    if (const auto* asFloat = it->second.get_if<float>()) {
        return static_cast<double>(*asFloat);
    }
    return std::nullopt;
}

/// @brief The flag under @p key, or `std::nullopt` when the key is absent or holds another type.
[[nodiscard]] inline std::optional<bool> metaFlag(const property_map& map, std::string_view key) {
    const auto it = map.find(std::pmr::string(key));
    if (it == map.end()) {
        return std::nullopt;
    }
    if (const auto* asBool = it->second.get_if<bool>()) {
        return *asBool;
    }
    return std::nullopt;
}

/// @brief The string under @p key, or `std::nullopt` when the key is absent or holds another type.
[[nodiscard]] inline std::optional<std::string> metaString(const property_map& map, std::string_view key) {
    const auto it = map.find(std::pmr::string(key));
    if (it == map.end()) {
        return std::nullopt;
    }
    if (const auto* asString = it->second.get_if<std::pmr::string>()) {
        return std::string(std::string_view(*asString));
    }
    return std::nullopt;
}

/// @brief True when @p a and @p b are equal or both NaN.
[[nodiscard]] inline bool sameNumber(double a, double b) noexcept { return a == b || (std::isnan(a) && std::isnan(b)); }

/// @brief One bin whose Gamma shape is a fraction of `d`, and that fraction.
struct ShapedBin {
    std::size_t bin;
    double      fraction;
};

/**
 * @brief The bins of a one-sided spectrum of real noise, transformed through @p window, whose power has a Gamma shape
 * below `d`, each with the fraction of `d` it has.
 *
 * Bin `k` of real white noise has the pseudo-variance `E[X^2] = sigma^2 sum w^2 exp(-4 pi i k n/N)`. Its magnitude is
 * `rho E[|X|^2]`, with `rho = |sum w^2 exp(-4 pi i k n/N)| / sum w^2`. The power of one periodogram bin then has
 * `Var/E^2 = 1 + rho^2`, and the sum of `d` periodograms has the variance of the Gamma shape `d/(1 + rho^2)`. `rho` is
 * 1 at the DC and Nyquist bins and 0 at every other bin of a rectangular window. The sum over `n` is the transform of
 * `w^2` at index `2k`, which is the transform over `N/2` points of `w^2` folded at `N/2`. A fraction within float
 * precision of 1 moves a reading by less than float precision, and that bin is left out. Returns no bin for a window of
 * fewer than four samples or with no power.
 */
[[nodiscard]] inline std::vector<ShapedBin> realInputShapedBins(std::span<const float> window) {
    const std::size_t half = window.size() / 2UZ;
    if (half < 2UZ) {
        return {};
    }
    std::vector<std::complex<double>> folded(half);
    for (std::size_t n = 0UZ; n < half; ++n) {
        const double early = static_cast<double>(window[n]);
        const double late  = static_cast<double>(window[n + half]);
        folded[n]          = early * early + late * late;
    }
    gr::algorithm::FFT<std::complex<double>> transform;
    const auto                               squared = transform.compute(folded);
    const double                             power   = squared[0UZ].real();
    if (!(power > 0.)) {
        return {};
    }
    std::vector<ShapedBin> shaped;
    for (std::size_t k = 0UZ; k <= half; ++k) {
        const double rho      = std::abs(squared[k % half]) / power;
        const double fraction = 1. / (1. + rho * rho);
        if (1. - fraction > static_cast<double>(std::numeric_limits<float>::epsilon())) {
            shaped.push_back({k, fraction});
        }
    }
    return shaped;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::measurement::SpectralKurtosis)

/**
 * @brief Measures spectral kurtosis per bin over `n_spectra` independent density records.
 *
 * The block reads the records of a spectral estimate, such as `WelchPsd` at an `overlap` of zero. For every
 * `n_spectra` records it accepts, it emits one record with the spectral kurtosis `SK` of each bin. `SK` reads 1 on
 * stationary noise and does not depend on the scale of the input. A noiseless tone reads 0, and a signal present in
 * fewer than half the spectra reads above 1.
 *
 * The block reads the first signal of each record, as `PeakDetect` does. It accepts a record when its `overlap`
 * metadata is present and zero, since the statistic needs independent spectra, and when its `mode` is 'mean' or absent.
 * A record that states `n_averaged` must state a positive finite number. At a `shape` of zero the record must state it,
 * and it is the Gamma shape `d` of the record's bins. At a nonzero `shape` a record may omit it. The block refuses a
 * record whose values or axis do not match its extent, and a record with a negative, NaN or infinite bin.
 *
 * One accumulation holds records of one shape, bin count, `one_sided` flag, window, `sample_rate` and
 * `center_frequency`. A record that differs in any of them ends the accumulation in progress. The block emits the
 * partial reading, refuses the record and starts a new accumulation at the next one. The last record of a spectral
 * estimate often holds fewer segments. At a `shape` of zero it ends the last accumulation this way. At a nonzero
 * `shape` the block refuses it, and the partial reading leaves at the end of the stream.
 *
 * The output record holds one `spectral_kurtosis` value per bin, on the input's axis. Its metadata states `n_spectra`,
 * `shape`, `window`, `one_sided`, `valid`, the expectation 1 as `sk_expectation`, and the spread of a bin of noise as
 * `sk_std`. A one-sided record also states the spread at the DC bin as `sk_std_edge`. The record carries the first
 * input record's `sample_start`, exact up to 2^53, and the input's `sample_rate` and `center_frequency` when they are
 * finite. A bin with no power reads 0. `valid` is false below 8 spectra and when no bin has power. The output carries
 * no input tag.
 *
 * A one-sided record is the spectrum of real input, and its bins near DC and Nyquist have a Gamma shape below `d`. The
 * shape is `d/2` at the DC and Nyquist bins. Next to them it depends on the window, and under Hann bins 1 and
 * `N/2 - 1` have about `0.97 d`. The block rebuilds the window from the record's `window`, `window_param` and
 * `fft_size` and evaluates each bin at its own shape. When it cannot build the window, it evaluates the DC and Nyquist
 * bins at `d/2` and every other bin at `d`. When `fft_size` differs from `2 * (bins - 1)`, the last bin is no Nyquist
 * bin, and only the DC bin takes `d/2`.
 *
 * `n_spectra` sets the spectra per reading, and a change applies to the accumulation in progress. `shape` fixes `d`,
 * and a nonzero `shape` refuses a record whose `n_averaged` differs. `emit_records` turns the output records on. At the
 * end of the stream the block emits the partial accumulation, and its `n_spectra` states the spectra it covers.
 *
 * At each valid reading the block publishes its worst bin: among the bins with power, the one whose `SK` lies furthest
 * from 1 in units of its own spread. `worstBinReading()` reads the bin, its `SK` and `nValidReadings()` together from
 * any thread. `worstBin()`, `worstBinSk()` and `nValidReadings()` read one value each, and all read 0 before the first
 * valid reading. `coverage()` reads the fraction of `n_spectra` the accumulation in progress holds.
 *
 * `nRecords()` counts the full readings, and `nValidReadings()` the valid readings, full or partial.
 * `nRefusedRecords()` counts the input records refused, and `nDegenerateBins()` the bins with no power, summed over all
 * readings.
 */
struct SpectralKurtosis : Block<SpectralKurtosis, NoTagPropagation> {
    using Description = Doc<R""(
@brief Measures spectral kurtosis per frequency bin over a run of independent density records.

The block takes `DataSet<float>` density records and emits one `spectral_kurtosis` record per `n_spectra` accepted
inputs, with one value per bin. `SK` reads 1 on noise, 0 on a noiseless tone, above 1 on a signal present in fewer
than half the spectra and below 1 on a steady one. The statistic assumes independent spectra. The block refuses a
record unless its `overlap` metadata is present and exactly zero. It passes no input tag on.
)"">;

    PortIn<DataSet<float>, Async>            spectra;
    PortOut<DataSet<float>, Async, Optional> measurements;

    Annotated<gr::Size_t, "n_spectra", Visible, Doc<"spectra per reading, 8 to 2^20">, Limits<detail::kMinSpectralKurtosisSpectra, detail::kMaxSpectralKurtosisSpectra>> n_spectra    = 32U;
    Annotated<gr::Size_t, "shape", Visible, Doc<"Gamma shape d; 0 reads the record's n_averaged">>                                                                       shape        = 0U;
    Annotated<bool, "emit_records", Visible, Doc<"publish one DataSet<float> record per accumulation">>                                                                  emit_records = true;

    GR_MAKE_REFLECTABLE(SpectralKurtosis, spectra, measurements, n_spectra, shape, emit_records);

    gr::measurement::SpectralKurtosisAccumulator _acc{};
    std::vector<double>                          _skValues{};

    std::atomic<std::uint64_t> _nRecords{0ULL};
    std::atomic<std::uint64_t> _nRefusedRecords{0ULL};
    std::atomic<std::uint64_t> _nDegenerateBins{0ULL};
    std::atomic<std::uint64_t> _filled{0ULL}; ///< spectra folded into the accumulation in progress

    gr::measurement::MeasurementSlot<2UZ> _slot{};              ///< the worst bin and its spectral kurtosis
    std::uint64_t                         _validReadings{0ULL}; ///< valid readings in this run

    // The settings the block last accepted. The work thread reads these copies.
    std::atomic<gr::Size_t> _nSpectra{n_spectra.value};
    gr::Size_t              _shape{shape.value};
    bool                    _emitRecords{emit_records.value};

    bool                  _haveCycle{false};        ///< the accumulation in progress holds at least one accepted spectrum
    double                _cycleShape{0.};          ///< the shape of the accumulation in progress
    bool                  _cycleOneSided{false};    ///< the accumulation holds spectra of real input
    bool                  _cycleRealLastBin{false}; ///< the last bin of those spectra is the Nyquist bin
    std::uint64_t         _cycleSampleStart{0ULL};
    double                _cycleSampleRate{std::numeric_limits<double>::quiet_NaN()};
    std::optional<double> _cycleCenterFrequency{};
    std::string           _cycleWindow{};
    double                _cycleWindowParam{std::numeric_limits<double>::quiet_NaN()};
    std::size_t           _cycleFftSize{0UZ}; ///< the transform length when it is `2 * (bins - 1)`, else 0
    std::vector<float>    _cycleAxis{};
    std::string           _cycleAxisName{"Frequency"};
    std::string           _cycleAxisUnit{"Hz"};

    // The bins below shape `d` of the last one-sided accumulation, and the window, parameter and transform length they
    // were built for. A transform length of zero marks a list built without a window.
    std::vector<detail::ShapedBin> _shapedBins{};
    std::string                    _shapedWindow{};
    double                         _shapedWindowParam{std::numeric_limits<double>::quiet_NaN()};
    std::size_t                    _shapedFftSize{0UZ};

    /// @brief Accepts the keys @p newSettings names.
    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const auto moved = [&newSettings](std::string_view key) { return newSettings.contains(key); };
        if (moved("n_spectra")) {
            _nSpectra.store(n_spectra.value, std::memory_order_relaxed);
        }
        if (moved("shape")) {
            _shape = shape.value;
        }
        if (moved("emit_records")) {
            _emitRecords = emit_records.value;
        }
    }

    /// @brief Clears the accumulation and the counters.
    void start() {
        _acc.resize(0UZ);
        _skValues.clear();
        _nRecords.store(0ULL, std::memory_order_relaxed);
        _nRefusedRecords.store(0ULL, std::memory_order_relaxed);
        _nDegenerateBins.store(0ULL, std::memory_order_relaxed);
        _validReadings = 0ULL;
        _slot.publish({0., 0.}, 0ULL);
        _filled.store(0ULL, std::memory_order_relaxed);
        _haveCycle  = false;
        _cycleShape = 0.;
    }

    /// @brief Accumulations of `n_spectra` spectra evaluated. Callable from any thread.
    [[nodiscard]] std::uint64_t nRecords() const noexcept { return _nRecords.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nRefusedRecords() const noexcept { return _nRefusedRecords.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nDegenerateBins() const noexcept { return _nDegenerateBins.load(std::memory_order_relaxed); }
    /// @brief The worst bin of one valid reading, its spectral kurtosis and the valid readings of the run up to it.
    struct WorstBinReading {
        std::uint64_t bin{0ULL};
        double        sk{0.};
        std::uint64_t validReadings{0ULL};
    };

    /// @brief The bin, its spectral kurtosis and `nValidReadings()`, all from the last valid reading. All three read 0
    /// before the first valid reading of a run. Callable from any thread.
    [[nodiscard]] WorstBinReading worstBinReading() const noexcept {
        const auto [values, count] = _slot.read();
        return {static_cast<std::uint64_t>(values[0UZ]), values[1UZ], count};
    }
    /// @brief The bin that lies furthest from 1 in units of its own spread, in the last valid reading. 0 while
    /// `nValidReadings()` reads 0. Two calls can read two readings, and `worstBinReading()` reads the bin and its
    /// spectral kurtosis together. Callable from any thread.
    [[nodiscard]] std::uint64_t worstBin() const noexcept { return static_cast<std::uint64_t>(_slot.read().first[0UZ]); }
    /// @brief The spectral kurtosis of the bin `worstBin()` reads, in the last valid reading. 0 while
    /// `nValidReadings()` reads 0. Callable from any thread.
    [[nodiscard]] double worstBinSk() const noexcept { return _slot.read().first[1UZ]; }
    /// @brief Valid readings in this run, full or partial. 0 means `worstBin()` and `worstBinSk()` hold no reading
    /// yet. Callable from any thread.
    [[nodiscard]] std::uint64_t nValidReadings() const noexcept { return _slot.read().second; }
    /// @brief Spectra folded into the accumulation in progress, as a fraction of the accepted `n_spectra`. Callable
    /// from any thread.
    [[nodiscard]] double coverage() const noexcept { return std::min(1., static_cast<double>(_filled.load(std::memory_order_relaxed)) / static_cast<double>(std::max(_nSpectra.load(std::memory_order_relaxed), gr::Size_t{1U}))); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const bool   wantRecords = _emitRecords && outSpan.isConnected;
        const Folded folded      = foldAll(inSpan, outSpan, wantRecords, "processBulk");
        outSpan.publish(folded.made);
        std::ignore = inSpan.consume(folded.consumed);
        if (folded.failed) {
            return work::Status::ERROR;
        }
        return folded.consumed == 0UZ && inSpan.size() > 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }

    /// @brief Folds in the trailing records at the end of the stream and evaluates the partial accumulation.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const bool wantRecords = _emitRecords && outSpan.isConnected;
        Folded     folded      = foldAll(inSpan, outSpan, wantRecords, "processEpilogue");
        if (!folded.failed && _haveCycle) {
            auto record = evaluate(wantRecords);
            if (!record) {
                this->emitErrorMessage("processEpilogue", record.error());
                folded.failed = true;
            } else if (record->has_value() && folded.made < outSpan.size()) {
                outSpan[folded.made] = std::move(**record);
                ++folded.made;
            }
        }
        outSpan.publish(folded.made);
        std::ignore = inSpan.consume(folded.consumed);
        return folded.failed ? work::Status::ERROR : work::Status::OK;
    }

private:
    struct Folded {
        std::size_t made{0UZ};
        std::size_t consumed{0UZ};
        bool        failed{false};
    };

    /// @brief Folds the input records in while the output has room for a record. Stops at an evaluation error, which
    /// goes out through `emitErrorMessage`.
    [[nodiscard]] Folded foldAll(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, bool wantRecords, std::string_view site) {
        Folded folded;
        for (; folded.consumed < inSpan.size() && (!wantRecords || folded.made < outSpan.size()); ++folded.consumed) {
            auto record = fold(inSpan[folded.consumed], wantRecords);
            if (!record) {
                this->emitErrorMessage(site, record.error());
                folded.failed = true;
                break;
            }
            if (record->has_value()) {
                outSpan[folded.made] = std::move(**record);
                ++folded.made;
            }
        }
        return folded;
    }

    /// @brief Folds one input record in or refuses it. Returns the output record when this input completes the
    /// accumulation and @p wantRecords is set, or the evaluation's error.
    [[nodiscard]] std::expected<std::optional<DataSet<float>>, gr::Error> fold(const DataSet<float>& record, bool wantRecords) {
        const std::span<const float> values = detail::firstSignal(record);
        if (values.empty()) {
            refuse();
            return std::nullopt;
        }
        static const property_map kNoMetadata{};
        const property_map&       meta = record.meta_information.empty() ? kNoMetadata : record.meta_information.front();

        const std::optional<double> overlap = detail::metaNumber(meta, "overlap");
        if (!overlap.has_value() || *overlap != 0.) {
            refuse();
            return std::nullopt;
        }
        if (const std::optional<std::string> mode = detail::metaString(meta, "mode"); mode.has_value() && *mode != "mean") {
            refuse();
            return std::nullopt;
        }

        const std::optional<double> averaged = detail::metaNumber(meta, "n_averaged");
        if (averaged.has_value() && !(std::isfinite(*averaged) && *averaged > 0.)) {
            refuse();
            return std::nullopt;
        }
        double effectiveShape;
        if (_shape != 0U) {
            effectiveShape = static_cast<double>(_shape);
            if (averaged.has_value() && *averaged != effectiveShape) {
                refuse();
                return std::nullopt;
            }
        } else {
            if (!averaged.has_value()) {
                refuse();
                return std::nullopt;
            }
            effectiveShape = *averaged;
        }

        constexpr double            kAbsent         = std::numeric_limits<double>::quiet_NaN();
        const std::size_t           bins            = values.size();
        const bool                  oneSided        = detail::metaFlag(meta, "one_sided").value_or(false);
        const std::optional<double> fftSize         = detail::metaNumber(meta, "fft_size");
        const bool                  realLast        = oneSided && bins >= 2UZ && fftSize.has_value() && *fftSize == 2. * static_cast<double>(bins - 1UZ);
        const std::string           window          = detail::metaString(meta, "window").value_or(std::string());
        const double                windowParam     = detail::metaNumber(meta, "window_param").value_or(kAbsent);
        const double                sampleRate      = detail::metaNumber(meta, "sample_rate").value_or(kAbsent);
        const std::optional<double> centerFrequency = detail::metaNumber(meta, "center_frequency");

        const bool sameKind = bins == _acc.bins() && effectiveShape == _cycleShape && oneSided == _cycleOneSided && realLast == _cycleRealLastBin && window == _cycleWindow && detail::sameNumber(windowParam, _cycleWindowParam) && detail::sameNumber(sampleRate, _cycleSampleRate) && detail::sameNumber(centerFrequency.value_or(kAbsent), _cycleCenterFrequency.value_or(kAbsent));
        if (_haveCycle && !sameKind) {
            // One `S1`/`S2` pair holds spectra of one kind. A record of another kind ends the accumulation in
            // progress, which goes out as a partial reading, and the record is refused.
            refuse();
            return evaluate(wantRecords);
        }
        if (_acc.bins() != bins) {
            _acc.resize(bins); // no accumulation is in progress
        }

        const gr::measurement::SpectrumResponse response = _acc.accumulate(values);
        if (response != gr::measurement::SpectrumResponse::accepted) {
            refuse();
            return std::nullopt;
        }

        if (!_haveCycle) {
            const auto sampleStart = detail::metaNumber(meta, "sample_start");
            _cycleShape            = effectiveShape;
            _cycleOneSided         = oneSided;
            _cycleRealLastBin      = realLast;
            _cycleSampleStart      = sampleStart.has_value() && *sampleStart >= 0. && *sampleStart < 0x1p64 ? static_cast<std::uint64_t>(*sampleStart) : 0ULL;
            _cycleSampleRate       = sampleRate;
            _cycleCenterFrequency  = centerFrequency;
            _cycleWindow           = window;
            _cycleWindowParam      = windowParam;
            _cycleFftSize          = realLast ? bins * 2UZ - 2UZ : 0UZ;
            _cycleAxis             = record.axis_values.front();
            _cycleAxisName         = record.axis_names.empty() ? std::string("Frequency") : record.axis_names.front();
            _cycleAxisUnit         = record.axis_units.empty() ? std::string("Hz") : record.axis_units.front();
            _haveCycle             = true;
        }
        const std::uint64_t filled = _filled.fetch_add(1ULL, std::memory_order_relaxed) + 1ULL;

        if (filled < static_cast<std::uint64_t>(_nSpectra.load(std::memory_order_relaxed))) {
            return std::nullopt;
        }
        auto evaluated = evaluate(wantRecords);
        if (evaluated) {
            _nRecords.fetch_add(1ULL, std::memory_order_relaxed);
        }
        return evaluated;
    }

    void refuse() { _nRefusedRecords.fetch_add(1ULL, std::memory_order_relaxed); }

    /// @brief The accumulation's bins below shape `d`, in bin order. A two-sided accumulation has none. A one-sided one
    /// whose last bin is the Nyquist bin has those of the window it names when the window accepts the record's
    /// `window_param`. Otherwise, and with no known window, the list holds the DC bin at `d/2`, and the Nyquist bin at
    /// `d/2` when the record has one. A list built from a window is kept until the window, its parameter or the
    /// transform length changes.
    [[nodiscard]] const std::vector<detail::ShapedBin>& shapedBins() {
        if (!_cycleOneSided) {
            _shapedBins.clear();
            _shapedFftSize = 0UZ;
            return _shapedBins;
        }
        if (_cycleFftSize != 0UZ && _cycleFftSize == _shapedFftSize && _cycleWindow == _shapedWindow && detail::sameNumber(_cycleWindowParam, _shapedWindowParam)) {
            return _shapedBins;
        }
        _shapedBins.clear();
        _shapedFftSize = 0UZ;
        if (const auto type = magic_enum::enum_cast<gr::algorithm::window::Type>(_cycleWindow); type.has_value() && _cycleFftSize != 0UZ) {
            try {
                const std::vector<float> taps = gr::algorithm::window::create<float>(*type, _cycleFftSize, static_cast<float>(_cycleWindowParam));
                _shapedBins                   = detail::realInputShapedBins(std::span<const float>(taps));
                _shapedWindow                 = _cycleWindow;
                _shapedWindowParam            = _cycleWindowParam;
                _shapedFftSize                = _cycleFftSize;
            } catch (const std::invalid_argument&) {
                _shapedBins.clear(); // a parameter the window refuses
            }
        }
        if (_shapedFftSize == 0UZ) { // the real bins alone, as under a rectangular window
            _shapedBins.push_back({0UZ, 0.5});
            if (_cycleRealLastBin) {
                _shapedBins.push_back({_acc.bins() - 1UZ, 0.5});
            }
        }
        return _shapedBins;
    }

    /// @brief Evaluates the accumulation in progress and clears it. Returns the record when @p wantRecords is set.
    [[nodiscard]] std::expected<std::optional<DataSet<float>>, gr::Error> evaluate(bool wantRecords) {
        const std::size_t bins = _acc.bins();
        const std::size_t m    = _acc.count();
        _skValues.assign(bins, 0.);
        const auto degenerate = _acc.evaluate(_cycleShape, std::span<double>(_skValues));
        if (!degenerate) {
            return std::unexpected(degenerate.error());
        }
        _nDegenerateBins.fetch_add(static_cast<std::uint64_t>(*degenerate), std::memory_order_relaxed);

        const std::vector<detail::ShapedBin>& shaped = shapedBins();
        for (const detail::ShapedBin& entry : shaped) {
            if (entry.bin < bins) {
                _skValues[entry.bin] = gr::measurement::spectralKurtosis(_acc.sums()[entry.bin], _acc.sumSquares()[entry.bin], m, _cycleShape * entry.fraction);
            }
        }

        const double spread     = gr::measurement::spectralKurtosisSpread(m, _cycleShape);
        const double edgeSpread = gr::measurement::spectralKurtosisSpread(m, _cycleShape / 2.);
        const bool   valid      = m >= static_cast<std::size_t>(detail::kMinSpectralKurtosisSpectra) && *degenerate < bins;
        if (valid) {
            const std::span<const double> sums       = _acc.sums();
            std::size_t                   worst      = 0UZ;
            double                        worstScore = -1.;
            std::size_t                   next       = 0UZ; // the next entry of `shaped`
            for (std::size_t k = 0UZ; k < bins; ++k) {
                double binSpread = spread;
                if (next < shaped.size() && shaped[next].bin == k) {
                    binSpread = gr::measurement::spectralKurtosisSpread(m, _cycleShape * shaped[next].fraction);
                    ++next;
                }
                if (!(sums[k] > 0.)) {
                    continue;
                }
                const double score = std::abs(_skValues[k] - gr::measurement::spectralKurtosisExpectation()) / (binSpread > 0. ? binSpread : 1.);
                if (score > worstScore) {
                    worstScore = score;
                    worst      = k;
                }
            }
            ++_validReadings;
            _slot.publish({static_cast<double>(worst), _skValues[worst]}, _validReadings);
        }

        std::optional<DataSet<float>> record;
        if (wantRecords) {
            record = makeRecord(m, spread, edgeSpread, valid);
        }
        _acc.resize(bins); // clears the sums for the next accumulation
        _haveCycle = false;
        _filled.store(0ULL, std::memory_order_relaxed);
        return record;
    }

    [[nodiscard]] DataSet<float> makeRecord(std::size_t m, double spread, double edgeSpread, bool valid) const {
        const std::size_t bins = _skValues.size();
        DataSet<float>    ds;
        ds.extents = {static_cast<std::int32_t>(bins)};
        ds.layout  = gr::LayoutRight{};

        ds.axis_names = {_cycleAxisName};
        ds.axis_units = {_cycleAxisUnit};
        ds.axis_values.resize(1UZ);
        ds.axis_values[0UZ] = _cycleAxis;

        ds.signal_names      = {"spectral_kurtosis"};
        ds.signal_quantities = {"SpectralKurtosis"};
        ds.signal_units      = {""};
        ds.signal_values.resize(bins);
        for (std::size_t k = 0UZ; k < bins; ++k) {
            ds.signal_values[k] = static_cast<float>(_skValues[k]);
        }
        ds.signal_ranges.resize(1UZ);

        property_map extra{
            {std::pmr::string("sample_start"), pmt::Value(_cycleSampleStart)},
            {std::pmr::string("window"), pmt::Value(_cycleWindow)},
            {std::pmr::string("n_spectra"), pmt::Value(static_cast<std::uint64_t>(m))},
            {std::pmr::string("shape"), pmt::Value(_cycleShape)},
            {std::pmr::string("one_sided"), pmt::Value(_cycleOneSided)},
            {std::pmr::string("sk_expectation"), pmt::Value(gr::measurement::spectralKurtosisExpectation())},
            {std::pmr::string("sk_std"), pmt::Value(spread)},
            {std::pmr::string("valid"), pmt::Value(valid)},
        };
        if (_cycleOneSided) {
            extra.insert_or_assign(std::pmr::string("sk_std_edge"), pmt::Value(edgeSpread));
        }
        if (std::isfinite(_cycleSampleRate)) {
            extra.insert_or_assign(std::pmr::string("sample_rate"), pmt::Value(static_cast<float>(_cycleSampleRate)));
        }
        if (_cycleCenterFrequency.has_value() && std::isfinite(*_cycleCenterFrequency)) {
            extra.insert_or_assign(std::pmr::string("center_frequency"), pmt::Value(*_cycleCenterFrequency));
        }
        ds.meta_information.resize(1UZ);
        ds.meta_information[0UZ] = std::move(extra);
        ds.timing_events.resize(1UZ);
        ds.timestamp = 0;
        return ds;
    }
};

} // namespace gr::blocks::measurement

#endif // GNURADIO_MEASUREMENT_SPECTRAL_KURTOSIS_HPP
