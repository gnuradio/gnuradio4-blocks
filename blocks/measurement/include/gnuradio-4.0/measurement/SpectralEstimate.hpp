#ifndef GNURADIO_MEASUREMENT_SPECTRAL_ESTIMATE_HPP
#define GNURADIO_MEASUREMENT_SPECTRAL_ESTIMATE_HPP

#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/algorithm/fourier/SpectralCalibration.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/algorithm/fourier/window.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <magic_enum.hpp>

namespace gr::blocks::measurement {

namespace detail {

/// @brief The values a spectral block computes on, and the setting values the hop and the window shape follow from.
/// The block changes them only when an application passes every check.
struct SpectralSettings {
    std::size_t                 fftSize          = 1024UZ;
    std::size_t                 hop              = 512UZ;
    std::size_t                 nAverages        = 1UZ;
    bool                        maxHold          = false;
    gr::algorithm::window::Type windowType       = gr::algorithm::window::Type::Hann;
    float                       windowShape      = 0.f;
    float                       sampleRate       = 1.f;
    double                      centerFrequency  = 0.;
    bool                        oneRecordPerCall = true;
    std::string                 windowName{};
    std::string                 signalName{};
    double                      overlap     = 0.5; ///< the `overlap` setting, which sets the hop at a `hop` setting of 0
    gr::Size_t                  hopSetting  = 0U;  ///< the `hop` setting
    float                       windowParam = 0.f; ///< the `window_param` setting, before the window's default applies
};

/// @brief The segmenting, windowing and power accumulation of both spectral blocks.
///
/// The window grid starts at sample zero of the stream, and each segment starts one hop after the previous one. A
/// record's `sample_start` is therefore an index into the whole stream. Any chunking of the same input gives the same
/// records. Segments accumulate in power, and a record holds the density `SpectralCalibration.hpp` defines.
///
/// A hop larger than the transform leaves a gap between segments. The accumulator skips the samples in the gap and
/// counts them into the stream position.
template<typename T>
struct SegmentAccumulator {
    using Real     = float;
    using Spectrum = std::complex<Real>;

    static constexpr bool kRealInput = std::is_same_v<T, Real>;

    /// A real input uses the real-to-complex transform. The windowed segment has the input's element type.
    using Windowed = std::conditional_t<kRealInput, Real, Spectrum>;

    std::size_t fftSize   = 1024UZ;
    std::size_t hop       = 512UZ;
    std::size_t nAverages = 16UZ;
    bool        maxHold   = false;
    Real        shape     = std::numeric_limits<Real>::quiet_NaN(); ///< the window's shape parameter, stated in the record

    std::vector<Real>                       window{};
    gr::algorithm::fft::SpectralScale<Real> scale{};
    gr::algorithm::FFT<Windowed, Spectrum>  transform{};

    std::vector<T>        pending{};  ///< input samples held until they fill a segment
    std::vector<Windowed> windowed{}; ///< one windowed segment, the transform's input
    std::vector<Spectrum> spectrum{}; ///< the transform's output
    std::vector<Real>     accumulator{};
    std::vector<Real>     hold{}; ///< one segment's density, compared bin by bin in max_hold mode

    std::size_t   segments      = 0UZ;  ///< segments in the accumulator, reported as `n_averaged`
    std::size_t   skipping      = 0UZ;  ///< samples to skip before the next segment starts, when hop > fftSize
    std::uint64_t streamAt      = 0ULL; ///< stream index of the first sample in `pending`
    std::uint64_t recordStartAt = 0ULL; ///< stream index of the first segment in the accumulator
    std::uint64_t gridStart     = 0ULL; ///< stream index where the current window grid starts, stated in the record

    /// Wall-clock seconds the record in progress has cost so far, on `steady_clock`. The sum covers the windowing,
    /// the transform and the accumulation of each segment.
    double costSeconds = 0.;

    SegmentAccumulator() = default;

    /// @brief An accumulator built for `settings` at the start of a stream.
    explicit SegmentAccumulator(const SpectralSettings& settings) { configure(settings); }

    [[nodiscard]] std::size_t bins() const noexcept { return kRealInput ? fftSize / 2UZ + 1UZ : fftSize; }

    /// @brief True when the next segment completes a record.
    [[nodiscard]] bool completesRecord() const noexcept { return segments + 1UZ >= nAverages; }

    /// @brief Builds the transform, the window and the buffers for a transform length. The stream position and the
    /// grid start are left to the caller.
    void resize(const SpectralSettings& settings) {
        fftSize   = settings.fftSize;
        transform = gr::algorithm::FFT<Windowed, Spectrum>{};
        windowed.assign(fftSize, Windowed{});
        spectrum.assign(fftSize, Spectrum{});
        reconfigure(settings);
    }

    /// @brief Builds for a transform length at the start of a stream. The stream position and the grid start are zero.
    void configure(const SpectralSettings& settings) {
        pending.clear();
        streamAt  = 0ULL;
        gridStart = 0ULL;
        resize(settings);
    }

    /// @brief Changes the transform length while a stream runs.
    ///
    /// `pending` and the stream position are kept, and `sample_start` continues on the same stream. The accumulation
    /// in progress is dropped, and a record never mixes two transform lengths. The new grid starts at the current
    /// stream position, and `gridStart` records it. The gap left by the old grid is dropped.
    void rebuild(const SpectralSettings& settings) {
        skipping  = 0UZ;
        gridStart = streamAt;
        resize(settings);
    }

    /// @brief Sets the parameters that may change while the block runs, and drops the accumulation in progress.
    ///
    /// A record never mixes two windows. `pending` and `streamAt` are kept. The grid stays where it is unless the hop
    /// changes.
    void reconfigure(const SpectralSettings& settings) {
        if (settings.hop != hop) {
            // A new hop starts a new grid at the current stream position. The gap left by the old hop is dropped. An
            // unchanged hop keeps the gap and the grid start.
            skipping  = 0UZ;
            gridStart = streamAt;
        }
        hop       = settings.hop;
        nAverages = settings.nAverages;
        maxHold   = settings.maxHold;
        shape     = settings.windowShape;
        window    = gr::algorithm::window::create<Real>(settings.windowType, fftSize, settings.windowShape);
        scale     = gr::algorithm::fft::spectralScale(std::span<const Real>(window), settings.sampleRate);
        hold.assign(bins(), Real{0});
        restart();
    }

    /// @brief Drops the accumulation in progress. The stream position, the grid and any gap still to skip are kept.
    void restart() {
        accumulator.assign(bins(), Real{0});
        segments      = 0UZ;
        recordStartAt = streamAt;
    }

    /// @brief Clears the stream position, the grid start and the accumulation for a new run.
    void reset() {
        pending.clear();
        streamAt  = 0ULL;
        gridStart = 0ULL;
        skipping  = 0UZ;
        restart();
    }

    /// @brief Folds the first whole segment of `pending` into the accumulator and adds its cost to `costSeconds`.
    void accumulateFront() {
        const auto started = std::chrono::steady_clock::now();
        if (segments == 0UZ) {
            recordStartAt = streamAt;
            costSeconds   = 0.;
        }
        for (std::size_t k = 0UZ; k < fftSize; ++k) {
            if constexpr (kRealInput) {
                windowed[k] = pending[k] * window[k];
            } else {
                windowed[k] = Spectrum(pending[k].real() * window[k], pending[k].imag() * window[k]);
            }
        }
        transform.compute(windowed, std::span<Spectrum>(spectrum));

        if (maxHold) {
            // A NaN bin stays NaN until the record is taken, as it does in the mean.
            std::ranges::fill(hold, Real{0});
            gr::algorithm::fft::accumulatePowerSpectrum(std::span<const Spectrum>(spectrum), scale.density, kRealInput, std::span<Real>(hold));
            for (std::size_t k = 0UZ; k < accumulator.size(); ++k) {
                accumulator[k] = (segments == 0UZ || std::isnan(hold[k]) || hold[k] > accumulator[k]) ? hold[k] : accumulator[k];
            }
        } else {
            gr::algorithm::fft::accumulatePowerSpectrum(std::span<const Spectrum>(spectrum), scale.density, kRealInput, std::span<Real>(accumulator));
        }
        ++segments;

        // The grid advances by one hop. The part of the hop beyond the buffered samples becomes `skipping`, and the
        // caller consumes it without copying.
        const std::size_t advance = std::min(hop, pending.size());
        pending.erase(pending.begin(), pending.begin() + static_cast<std::ptrdiff_t>(advance));
        streamAt += advance;
        skipping = hop - advance;

        costSeconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    }

    /// @brief The segments no record carries yet: the segments in the accumulator, and the whole segments that
    /// `pending` and `unread` further input samples hold on the grid.
    [[nodiscard]] std::size_t segmentsLeft(std::size_t unread) const noexcept {
        // A gap still to skip leaves `pending` empty and comes first in the unread samples.
        const std::size_t held = pending.size() + (unread > skipping ? unread - skipping : 0UZ);
        return segments + (held >= fftSize ? (held - fftSize) / hop + 1UZ : 0UZ);
    }

    /// @brief The record the accumulator holds, averaged over its segments, and an empty accumulator.
    [[nodiscard]] std::vector<Real> take() {
        std::vector<Real> values = accumulator;
        if (!maxHold && segments > 1UZ) {
            const Real divisor = static_cast<Real>(segments);
            for (Real& value : values) {
                value /= divisor;
            }
        }
        if constexpr (!kRealInput) { // a two-sided axis runs from negative to positive frequency, and the halves swap
            std::rotate(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2UZ), values.end());
        }
        accumulator.assign(bins(), Real{0});
        segments = 0UZ;
        return values;
    }
};

/// @brief The work done by one `processBulk` or `processEpilogue` call.
struct Progress {
    std::size_t taken      = 0UZ;   ///< input samples consumed, buffered or skipped
    std::size_t made       = 0UZ;   ///< records written to the output span
    bool        outputFull = false; ///< the call stopped for lack of a free output slot
};

/**
 * @brief Folds the input span into the accumulator and writes each record it completes to the output span.
 *
 * A sample is consumed once it is copied into `pending` or skipped as part of a gap. While the output span has no free
 * slot, the sample that completes a record stays in the input span. The last sample of the span stays as well when the
 * accumulator holds a segment or that sample completes one. The end of the stream then finds a free slot for the
 * partial record. With `oneRecordPerCall` set, the call returns after its first record.
 */
template<typename T, typename Emit>
[[nodiscard]] Progress foldSegments(SegmentAccumulator<T>& core, InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, bool oneRecordPerCall, Emit&& emit) {
    Progress          progress{};
    const std::size_t offer = inSpan.size();
    for (;;) {
        const bool full = progress.made == outSpan.size();
        if (core.pending.size() >= core.fftSize) {
            if (full && core.completesRecord()) {
                progress.outputFull = true;
                return progress;
            }
            core.accumulateFront();
            if (core.segments >= core.nAverages) {
                outSpan[progress.made] = emit();
                ++progress.made;
                if (oneRecordPerCall) {
                    return progress;
                }
            }
            continue;
        }
        const std::size_t limit = (full && core.segments > 0UZ && offer > 0UZ) ? offer - 1UZ : offer;
        if (progress.taken >= limit) {
            progress.outputFull = progress.taken < offer;
            return progress;
        }
        if (core.skipping > 0UZ) {
            // The gap of a hop above `fftSize`. It is consumed through the port and counted into the stream position
            // without a copy.
            const std::size_t drop = std::min(core.skipping, limit - progress.taken);
            core.skipping -= drop;
            core.streamAt += drop;
            progress.taken += drop;
            continue;
        }
        std::size_t take = std::min(core.fftSize - core.pending.size(), limit - progress.taken);
        if (full && core.pending.size() + take == core.fftSize && (core.completesRecord() || progress.taken + take == offer)) {
            --take;
            progress.outputFull = true;
        }
        const auto first = inSpan.begin() + static_cast<std::ptrdiff_t>(progress.taken);
        core.pending.insert(core.pending.end(), first, first + static_cast<std::ptrdiff_t>(take));
        progress.taken += take;
        if (progress.outputFull) {
            return progress;
        }
    }
}

/// @brief The status of a `processBulk` call: `OK` when it moved anything, and otherwise what it lacked.
[[nodiscard]] inline work::Status statusOf(const Progress& progress) noexcept {
    if (progress.made > 0UZ || progress.taken > 0UZ) {
        return work::Status::OK;
    }
    return progress.outputFull ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
}

/// @brief The fraction of a segment the next segment shares. Zero when the hop is at least the transform length.
[[nodiscard]] inline double overlapAt(std::size_t fftSize, std::size_t hop) noexcept { return hop >= fftSize ? 0.0 : 1.0 - static_cast<double>(hop) / static_cast<double>(fftSize); }

/// @brief Seconds of wall clock since a steady-clock reading.
[[nodiscard]] inline double secondsSince(std::chrono::steady_clock::time_point since) noexcept { return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count(); }

/// @brief Builds the density record of both spectral blocks.
template<typename Real>
[[nodiscard]] DataSet<Real> makeSpectralRecord(std::vector<Real> values, Real sampleRate, double centerFrequency, std::size_t fftSize, bool oneSided, std::uint64_t sampleStart, std::size_t nAveraged, bool maxHold, std::size_t hop, Real enbwBins, std::string_view windowName, Real windowParam, std::uint64_t gridStart, std::string_view signalName) {
    DataSet<Real> ds;
    const auto    bins = values.size();

    ds.extents = {static_cast<std::int32_t>(bins)};
    ds.layout  = gr::LayoutRight{};

    ds.axis_names = {"Frequency"};
    ds.axis_units = {"Hz"};
    ds.axis_values.resize(1UZ);
    ds.axis_values[0UZ].resize(bins);

    const Real binWidth = sampleRate / static_cast<Real>(fftSize);
    const Real offset   = oneSided ? Real{0} : static_cast<Real>(bins / 2UZ) * binWidth;
    for (std::size_t k = 0UZ; k < bins; ++k) {
        ds.axis_values[0UZ][k] = static_cast<Real>(k) * binWidth - offset;
    }

    ds.signal_names      = {std::string(signalName)};
    ds.signal_quantities = {"PowerSpectralDensity"};
    // Linear power per hertz, relative to unit power. `10*log10(value)` is dBFS/Hz. Band integrals and further averages
    // use the linear form.
    ds.signal_units  = {"1/Hz"};
    ds.signal_values = std::move(values);
    ds.signal_ranges.resize(1UZ);

    ds.meta_information.resize(1UZ);
    ds.meta_information[0UZ] = property_map{
        {std::pmr::string("sample_rate"), pmt::Value(sampleRate)},
        // The frequency in hertz at the axis's zero. The axis itself stays at baseband.
        {std::pmr::string("center_frequency"), pmt::Value(centerFrequency)},
        {std::pmr::string("sample_start"), pmt::Value(sampleStart)},
        {std::pmr::string("n_averaged"), pmt::Value(static_cast<std::uint64_t>(nAveraged))},
        // The accumulation mode, 'mean' or 'max_hold'. A 'max_hold' record holds each bin's largest density over its
        // segments.
        {std::pmr::string("mode"), pmt::Value(std::string(maxHold ? "max_hold" : "mean"))},
        // The fraction each segment shares with the next. Overlapping segments are correlated, and the number of
        // independent segments is then smaller than `n_averaged`.
        {std::pmr::string("overlap"), pmt::Value(overlapAt(fftSize, hop))},
        // The samples the grid advances between segments. Above `fft_size`, the record covers `fft_size / hop` of
        // the stream.
        {std::pmr::string("hop"), pmt::Value(static_cast<std::uint64_t>(hop))},
        {std::pmr::string("enbw_bins"), pmt::Value(enbwBins)},
        {std::pmr::string("window"), pmt::Value(std::string(windowName))},
        // The shape parameter the window was built with, after the default is applied. Kaiser at beta 1.6 and Kaiser
        // at beta 6.76 differ in sidelobe level and in noise bandwidth.
        {std::pmr::string("window_param"), pmt::Value(windowParam)},
        {std::pmr::string("fft_size"), pmt::Value(static_cast<std::uint64_t>(fftSize))},
        // The stream index where the current window grid starts. It is zero until `fft_size` or the hop in force
        // changes during a run. Such a change starts a new grid at the current stream position.
        {std::pmr::string("grid_start"), pmt::Value(gridStart)},
        {std::pmr::string("one_sided"), pmt::Value(oneSided)},
        // A full-scale complex tone has unit power. A full-scale real sine has half of it.
        {std::pmr::string("level_reference"), pmt::Value(std::string("unit power"))},
    };
    ds.timing_events.resize(1UZ);
    ds.timestamp = 0;
    return ds;
}

/// @brief Writes a record's computing cost under the key `compute_seconds`, in seconds of wall clock.
///
/// The figure covers the windowing, the transform and the power accumulation of every segment in the record, and the
/// assembly of the record. Time between work calls is excluded. The value is a `double`.
template<typename Real>
inline void stateComputeSeconds(DataSet<Real>& ds, double seconds) {
    ds.meta_information[0UZ].insert_or_assign(std::pmr::string("compute_seconds"), pmt::Value(seconds));
}

/// The transform lengths both blocks accept. A record holds `bins` values and `bins` axis points, `8 * bins` bytes,
/// and 32 MiB at the largest length. The accumulator holds about `36 * fft_size` bytes for a complex stream.
inline constexpr gr::Size_t kMinFftSize = 64U;
inline constexpr gr::Size_t kMaxFftSize = 4194304U;

/// @brief The hop `overlap` implies for a transform of `size` samples, at least one sample.
[[nodiscard]] inline std::size_t hopFor(std::size_t size, double overlap) noexcept { return std::max(1UZ, static_cast<std::size_t>(std::llround(static_cast<double>(size) * (1.0 - overlap)))); }

/// @brief The hop in force. A non-zero `hop` setting is the hop. A zero setting takes the hop `overlap` implies. A
/// hop above `size` leaves a gap between segments.
[[nodiscard]] inline std::size_t hopFrom(std::size_t size, double overlap, gr::Size_t hopSetting) noexcept { return hopSetting == 0U ? hopFor(size, overlap) : static_cast<std::size_t>(hopSetting); }

/// @brief The window type a name selects. Throws for a name outside the window vocabulary.
[[nodiscard]] inline gr::algorithm::window::Type requireWindow(const std::string& name) {
    const auto windowType = magic_enum::enum_cast<gr::algorithm::window::Type>(name);
    if (!windowType.has_value()) {
        throw gr::exception(std::format("window must be one of {}, got '{}'", std::string_view(gr::algorithm::window::TypeNames), name));
    }
    return *windowType;
}

/// @brief The shape parameter the window is built with: the setting, or the window's default when the setting is 0
/// or NaN. Throws for a value the window type does not accept, naming the setting.
///
/// A window whose parameter is legitimately zero has a type of its own. Kaiser at beta 0 is the rectangular window.
[[nodiscard]] inline float windowParamFor(gr::algorithm::window::Type windowType, float param) {
    const float shape = (param == 0.f || std::isnan(param)) ? gr::algorithm::window::defaultParameter<float>(windowType) : param;
    try {
        gr::algorithm::window::validateParameter<float>(windowType, shape);
    } catch (const std::invalid_argument& refusal) {
        throw gr::exception(std::format("window_param: {}", refusal.what()));
    }
    return shape;
}

/**
 * @brief The settings a spectral block computes on after an application.
 *
 * Each setting `moved` names comes from its member. Every other setting keeps its value in `next`, and the block
 * ignores its member. The hop in force and the window shape follow from the result. Throws for a moved window or mode
 * the block does not know, and for a window shape the window in force refuses, naming the setting.
 */
template<typename TBlock, typename Moved>
[[nodiscard]] SpectralSettings settingsAfter(const TBlock& block, SpectralSettings next, Moved moved) {
    if (moved("fft_size")) {
        next.fftSize = static_cast<std::size_t>(block.fft_size.value);
    }
    if (moved("overlap")) {
        next.overlap = block.overlap.value;
    }
    if (moved("hop")) {
        next.hopSetting = block.hop.value;
    }
    if (moved("sample_rate")) {
        next.sampleRate = block.sample_rate.value;
    }
    if (moved("center_frequency")) {
        next.centerFrequency = block.center_frequency.value;
    }
    if (moved("window")) {
        next.windowType = requireWindow(block.window.value);
        next.windowName = block.window.value;
    }
    if (moved("window_param")) {
        next.windowParam = block.window_param.value;
    }
    if (moved("signal_name")) {
        next.signalName = block.signal_name.value;
    }
    if (moved("one_record_per_call")) {
        next.oneRecordPerCall = block.one_record_per_call.value;
    }
    if constexpr (requires { block.n_averages; }) {
        if (moved("n_averages")) {
            next.nAverages = static_cast<std::size_t>(block.n_averages.value);
        }
        if (moved("mode")) {
            if (block.mode.value != "mean" && block.mode.value != "max_hold") {
                throw gr::exception(std::format("mode must be 'mean' or 'max_hold', got '{}'", block.mode.value));
            }
            next.maxHold = block.mode.value == "max_hold";
        }
    }
    next.hop         = hopFrom(next.fftSize, next.overlap, next.hopSetting);
    next.windowShape = windowParamFor(next.windowType, next.windowParam);
    return next;
}

/// @brief Names every setting, for the settings a block starts from.
inline constexpr auto everySetting = [](std::string_view) { return true; };

/// @brief True when `next` changes what the segment accumulator computes: the transform length, the hop, the
/// averaging, the window or its shape, or the sample rate. Two NaN shapes are the same shape.
[[nodiscard]] inline bool changesEstimate(const SpectralSettings& next, const SpectralSettings& accepted) noexcept {
    const bool sameShape = next.windowShape == accepted.windowShape || (std::isnan(next.windowShape) && std::isnan(accepted.windowShape));
    return next.fftSize != accepted.fftSize || next.hop != accepted.hop || next.nAverages != accepted.nAverages || next.maxHold != accepted.maxHold || next.windowType != accepted.windowType || !sameShape || next.sampleRate != accepted.sampleRate;
}

/// @brief The validator of `sample_rate`: true above 0.
inline constexpr auto kSampleRatePositive = [](float value) { return value > 0.f; };

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::measurement::WelchPsd, [T], [ float, std::complex<float> ])

/**
 * @brief Welch's averaged periodogram: one calibrated power spectral density record per `n_averages` segments.
 *
 * The block measures the spectrum of a sample stream. It cuts the stream into windowed segments of `fft_size` samples,
 * transforms each segment and averages the segment powers. Each record it emits is one spectrum, for display or for a
 * block that reads density records.
 *
 * The input is a stream of `float` or `std::complex<float>` samples. The output is one `DataSet` per `n_averages`
 * segments. Its one signal is linear power per hertz relative to unit power, and `10*log10(value)` is dBFS/Hz. A
 * full-scale complex tone integrates to 0 dBFS, and a full-scale real sine to -3.01 dBFS. A tone's power is its peak
 * density times `enbw_bins` times the bin width. The frequency axis is in hertz at baseband. It runs from 0 to half the
 * sample rate for a real input and is centered on 0 for a complex input. A record that holds a NaN sample reads NaN in
 * every bin. The output carries no input tag.
 *
 * The record's metadata states `sample_rate`, `center_frequency`, `fft_size`, `window`, the shape in force as
 * `window_param`, `enbw_bins`, `hop`, `overlap`, `one_sided` and `level_reference`. `sample_start` is the stream index
 * of the record's first segment, and `n_averaged` counts its segments. `mode` is 'mean' or 'max_hold'. `grid_start` is
 * the stream index where the current segment grid starts. `compute_seconds` is the wall-clock time the record cost.
 *
 * `fft_size` sets the transform length, and the bin width is `sample_rate / fft_size`. `window` names the window.
 * `window_param` sets the shape of the Kaiser (beta), Tukey (alpha), Gaussian (sigma) and Exponential (decay in dB)
 * windows, and zero or NaN selects the window's default. The block refuses a shape the window in force does not accept:
 * a negative Kaiser beta or Exponential decay, a Tukey alpha outside [0, 1] or a Gaussian sigma outside (0, 0.5]. A
 * change of `window` alone is refused when the shape in force does not suit the new window. `hop` sets the samples
 * between segment starts, and at zero the hop follows `overlap`. A hop above `fft_size` skips the samples between
 * segments, and `sample_start` still counts them. In `mean` mode a record holds the average of its segments. In
 * `max_hold` mode each bin holds its largest density. `center_frequency` is the frequency at the axis's zero.
 * `one_record_per_call` limits a work call to one hop of input and one record.
 *
 * Every setting can change while the block runs. A change to `fft_size`, the window, its shape, the hop, `n_averages`,
 * `mode` or `sample_rate` drops the segments of the record in progress and keeps the buffered samples. A change to
 * `fft_size` or to the hop also starts a new segment grid at the current stream position. A change to
 * `center_frequency` or `signal_name` applies to the next record. At the end of the stream the block emits the partial
 * record.
 *
 * `nDroppedSegments()` counts the segments left at the end of the stream, or at a stop request, without a free output
 * slot for their record.
 */
template<typename T>
requires(std::is_same_v<T, float> || std::is_same_v<T, std::complex<float>>)
struct WelchPsd : Block<WelchPsd<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Computes Welch's averaged power spectral density of a real or complex stream.

Each DataSet record averages the power of n_averages windowed segments, or holds each bin's largest value in max_hold
mode. Values are linear power per hertz, where a full-scale complex tone has unit power. The record states the
settings it was computed with, the stream index of its first segment and its computing cost. It passes no input tag on.
)"">;
    using Real        = float;

    PortIn<T>                     in;
    PortOut<DataSet<Real>, Async> out;

    Annotated<gr::Size_t, "fft_size", Visible, Doc<"power-of-two transform length, 64 to 4194304">, Limits<detail::kMinFftSize, detail::kMaxFftSize, [](gr::Size_t v) { return std::has_single_bit(v); }>> fft_size            = 1024U;
    Annotated<std::string, "window", Visible, Doc<gr::algorithm::window::TypeNames>>                                                                                                                       window              = std::string("Hann");
    Annotated<float, "window_param", Visible, Doc<"shape parameter, default at 0 or NaN">>                                                                                                                 window_param        = 0.f;
    Annotated<double, "overlap", Visible, Doc<"segment overlap fraction in [0, 1)">, Limits<0.0, 1.0, [](double v) { return v < 1.0; }>>                                                                   overlap             = 0.5;
    Annotated<gr::Size_t, "hop", Visible, Unit<"samples">, Doc<"samples between segment starts; 0 follows overlap">>                                                                                       hop                 = 0U;
    Annotated<gr::Size_t, "n_averages", Visible, Doc<"segments per emitted record, at least 1">, Limits<1U, std::numeric_limits<gr::Size_t>::max()>>                                                       n_averages          = 16U;
    Annotated<std::string, "mode", Visible, Doc<"accumulation mode, 'mean' or 'max_hold'">>                                                                                                                mode                = std::string("mean");
    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"input sample rate">, Limits<0.f, std::numeric_limits<float>::max(), detail::kSampleRatePositive>>                                            sample_rate         = 1.f;
    Annotated<std::string, "signal_name", Doc<"signal name of the emitted record">>                                                                                                                        signal_name         = std::string("psd");
    Annotated<double, "center_frequency", Visible, Unit<"Hz">, Doc<"frequency at the axis zero">, Limits<std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max()>>                       center_frequency    = 0.;
    Annotated<bool, "one_record_per_call", Visible, Doc<"emit at most one record per work call">>                                                                                                          one_record_per_call = true;

    GR_MAKE_REFLECTABLE(WelchPsd, in, out, fft_size, window, window_param, overlap, hop, n_averages, mode, sample_rate, signal_name, center_frequency, one_record_per_call);

    detail::SpectralSettings      _accepted = detail::settingsAfter(*this, {}, detail::everySetting); ///< the settings the block computes on
    detail::SegmentAccumulator<T> _core{_accepted};
    bool                          _flushed = false;
    std::atomic<std::uint64_t>    _droppedSegments{0ULL};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        detail::SpectralSettings next = detail::settingsAfter(*this, _accepted, [&newSettings](std::string_view key) { return newSettings.contains(key); });
        if (next.fftSize != _accepted.fftSize) {
            // A new transform length drops the accumulation in progress and keeps `pending` and the stream position.
            _core.rebuild(next);
        } else if (detail::changesEstimate(next, _accepted)) {
            _core.reconfigure(next);
        }
        _accepted = std::move(next);
        capInput();
    }

    /// @brief Limits the input span to the hop in force when `one_record_per_call` is on. A hop is the least input one
    /// record needs.
    void capInput() { in.max_samples = _accepted.oneRecordPerCall ? _core.hop : std::numeric_limits<std::size_t>::max(); }

    void start() {
        _core.reset();
        _flushed = false;
        _droppedSegments.store(0ULL, std::memory_order_relaxed);
        capInput();
    }

    /// @brief Whole segments the end of the stream held, buffered or in the trailing input, that no record carries for
    /// lack of a free output slot. Callable from any thread.
    [[nodiscard]] std::uint64_t nDroppedSegments() const noexcept { return _droppedSegments.load(std::memory_order_relaxed); }

    /// @brief Folds input into the accumulator and emits every record it completes. A call that moves nothing returns
    /// `INSUFFICIENT_OUTPUT_ITEMS` when it waits on the output and `INSUFFICIENT_INPUT_ITEMS` otherwise.
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const detail::Progress progress = detail::foldSegments(_core, inSpan, outSpan, _accepted.oneRecordPerCall, [this] { return emit(); });
        outSpan.publish(progress.made);
        std::ignore = inSpan.consume(progress.taken);
        return detail::statusOf(progress);
    }

    /// @brief At the end of the stream, folds in any trailing samples and emits the accumulation in progress. The
    /// segments left without a free output slot are counted in `nDroppedSegments()`.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        detail::Progress progress = detail::foldSegments(_core, inSpan, outSpan, false, [this] { return emit(); });
        if (!_flushed && _core.segments > 0UZ && progress.made < outSpan.size()) {
            outSpan[progress.made] = emit();
            ++progress.made;
            _flushed = true;
        }
        outSpan.publish(progress.made);
        _droppedSegments.fetch_add(_core.segmentsLeft(inSpan.size() - progress.taken), std::memory_order_relaxed);
        return work::Status::OK;
    }

private:
    [[nodiscard]] DataSet<Real> emit() {
        const auto          assembly = std::chrono::steady_clock::now();
        const double        folded   = _core.costSeconds;
        const std::size_t   averaged = _core.segments;
        const std::uint64_t startAt  = _core.recordStartAt;

        DataSet<Real> record = detail::makeSpectralRecord<Real>(_core.take(), _accepted.sampleRate, _accepted.centerFrequency, _core.fftSize, detail::SegmentAccumulator<T>::kRealInput, startAt, averaged, _core.maxHold, _core.hop, _core.scale.enbwBins, _accepted.windowName, _core.shape, _core.gridStart, _accepted.signalName);
        detail::stateComputeSeconds(record, folded + detail::secondsSince(assembly));
        return record;
    }
};

GR_REGISTER_BLOCK(gr::blocks::measurement::Spectrogram, [T], [ float, std::complex<float> ])

/**
 * @brief A spectrogram: one calibrated power spectral density record per windowed transform.
 *
 * The block works as `WelchPsd` does with one segment per record. Each record is one row in time, and successive rows
 * start one hop apart. It takes the same input, emits the same record fields and has the same settings except
 * `n_averages` and `mode`. Each row states `n_averaged` 1 and `mode` 'mean', and its `sample_start` is the stream index
 * of its segment.
 *
 * A setting change keeps every buffered sample, and the next row is computed on the new settings. A change to
 * `fft_size` or to the hop starts a new grid at the current stream position, as in `WelchPsd`. The end of the stream
 * emits no partial row. The output carries no input tag.
 *
 * `nDroppedSegments()` counts the rows left at the end of the stream, or at a stop request, without a free output slot.
 */
template<typename T>
requires(std::is_same_v<T, float> || std::is_same_v<T, std::complex<float>>)
struct Spectrogram : Block<Spectrogram<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Computes a spectrogram: one power spectral density record per windowed transform.

Rows start one hop apart, and each row carries the stream index of its segment. Values are calibrated as in WelchPsd.
It passes no input tag on.
)"">;
    using Real        = float;

    PortIn<T>                     in;
    PortOut<DataSet<Real>, Async> out;

    Annotated<gr::Size_t, "fft_size", Visible, Doc<"power-of-two transform length, 64 to 4194304">, Limits<detail::kMinFftSize, detail::kMaxFftSize, [](gr::Size_t v) { return std::has_single_bit(v); }>> fft_size            = 1024U;
    Annotated<std::string, "window", Visible, Doc<gr::algorithm::window::TypeNames>>                                                                                                                       window              = std::string("Hann");
    Annotated<float, "window_param", Visible, Doc<"shape parameter, default at 0 or NaN">>                                                                                                                 window_param        = 0.f;
    Annotated<double, "overlap", Visible, Doc<"segment overlap fraction in [0, 1)">, Limits<0.0, 1.0, [](double v) { return v < 1.0; }>>                                                                   overlap             = 0.5;
    Annotated<gr::Size_t, "hop", Visible, Unit<"samples">, Doc<"samples between row starts; 0 follows overlap">>                                                                                           hop                 = 0U;
    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"input sample rate">, Limits<0.f, std::numeric_limits<float>::max(), detail::kSampleRatePositive>>                                            sample_rate         = 1.f;
    Annotated<std::string, "signal_name", Doc<"signal name of the emitted record">>                                                                                                                        signal_name         = std::string("spectrogram");
    Annotated<double, "center_frequency", Visible, Unit<"Hz">, Doc<"frequency at the axis zero">, Limits<std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max()>>                       center_frequency    = 0.;
    Annotated<bool, "one_record_per_call", Visible, Doc<"emit at most one row per work call">>                                                                                                             one_record_per_call = true;

    GR_MAKE_REFLECTABLE(Spectrogram, in, out, fft_size, window, window_param, overlap, hop, sample_rate, signal_name, center_frequency, one_record_per_call);

    detail::SpectralSettings      _accepted = detail::settingsAfter(*this, {}, detail::everySetting); ///< the settings the block computes on
    detail::SegmentAccumulator<T> _core{_accepted};
    std::atomic<std::uint64_t>    _droppedSegments{0ULL};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        detail::SpectralSettings next = detail::settingsAfter(*this, _accepted, [&newSettings](std::string_view key) { return newSettings.contains(key); });
        if (next.fftSize != _accepted.fftSize) {
            _core.rebuild(next);
        } else if (detail::changesEstimate(next, _accepted)) {
            _core.reconfigure(next);
        }
        _accepted = std::move(next);
        capInput();
    }

    /// @brief Limits the input span to the hop in force, as `WelchPsd` does.
    void capInput() { in.max_samples = _accepted.oneRecordPerCall ? _core.hop : std::numeric_limits<std::size_t>::max(); }

    void start() {
        _core.reset();
        _droppedSegments.store(0ULL, std::memory_order_relaxed);
        capInput();
    }

    /// @brief Whole segments the end of the stream held that no row carries for lack of a free output slot, counted as
    /// `WelchPsd::nDroppedSegments()` counts them. Callable from any thread.
    [[nodiscard]] std::uint64_t nDroppedSegments() const noexcept { return _droppedSegments.load(std::memory_order_relaxed); }

    /// @brief Folds input and emits a row per segment, as `WelchPsd::processBulk` does. A call that moves nothing
    /// reports why.
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const detail::Progress progress = detail::foldSegments(_core, inSpan, outSpan, _accepted.oneRecordPerCall, [this] { return emit(); });
        outSpan.publish(progress.made);
        std::ignore = inSpan.consume(progress.taken);
        return detail::statusOf(progress);
    }

    /// @brief At the end of the stream, emits a row for each whole segment the block holds or any trailing samples
    /// complete, and counts the rows left without a free output slot.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const detail::Progress progress = detail::foldSegments(_core, inSpan, outSpan, false, [this] { return emit(); });
        outSpan.publish(progress.made);
        _droppedSegments.fetch_add(_core.segmentsLeft(inSpan.size() - progress.taken), std::memory_order_relaxed);
        return work::Status::OK;
    }

private:
    [[nodiscard]] DataSet<Real> emit() {
        const auto          assembly = std::chrono::steady_clock::now();
        const double        folded   = _core.costSeconds;
        const std::uint64_t startAt  = _core.recordStartAt;

        DataSet<Real> record = detail::makeSpectralRecord<Real>(_core.take(), _accepted.sampleRate, _accepted.centerFrequency, _core.fftSize, detail::SegmentAccumulator<T>::kRealInput, startAt, 1UZ, false, _core.hop, _core.scale.enbwBins, _accepted.windowName, _core.shape, _core.gridStart, _accepted.signalName);
        detail::stateComputeSeconds(record, folded + detail::secondsSince(assembly));
        return record;
    }
};

} // namespace gr::blocks::measurement

#endif // GNURADIO_MEASUREMENT_SPECTRAL_ESTIMATE_HPP
