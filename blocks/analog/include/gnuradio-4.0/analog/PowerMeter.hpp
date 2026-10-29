#ifndef GNURADIO_POWER_METER_HPP
#define GNURADIO_POWER_METER_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <span>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/MeasurementRecord.hpp>
#include <gnuradio-4.0/algorithm/MeasurementSlot.hpp>

namespace gr::blocks::analog {

namespace detail {

/// @brief The validator of `sample_rate`: true above 0.
inline constexpr auto kPowerMeterSampleRatePositive = [](float rate) { return rate > 0.f; };

/// @brief The validator of `window_time`: true above 0.
inline constexpr auto kWindowTimePositive = [](double time) { return time > 0.0; };

/// @brief The longest window in samples, 2^53. Every whole number of samples up to it is exact in a double.
inline constexpr double kMaxWindowSamples = 9007199254740992.0;

/// @brief @p count narrowed to `std::size_t`, saturated at its maximum.
[[nodiscard]] constexpr std::size_t saturatedSize(std::uint64_t count) noexcept { return count < std::numeric_limits<std::size_t>::max() ? static_cast<std::size_t>(count) : std::numeric_limits<std::size_t>::max(); }

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::analog::PowerMeter, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct PowerMeter : Block<PowerMeter<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief An S-meter giving the mean power of the last `window_time` seconds in dBFS, readable from another thread.

The meter is a sink on a float or complex stream. It reads the mean power over a rolling window. 0 dBFS is a mean power
of exactly 1.0, and a complex tone of magnitude one reads 0 dB. Input tags stop at the meter.

`level()` returns the reading in dBFS, clamped at `floor_db`, and `linear_power()` returns the mean power as a ratio,
without the floor. `coverage()` returns the fraction of the window the reading covers, from 0 to 1. A reading over a
partial window is reported with its coverage. The three may be called from any thread, and the reading refreshes once
per segment.

The optional `records` port carries one `DataSet<float>` per completed window. A record holds the signals `power` in
dBFS, `power_linear` and `coverage`, and states `n_segments` and `window_samples`. Its `sample_start` is the window's
first input sample, counted from the start of the run. While the port has no room, the meter holds its records and takes
only the input whose records fit. A stream that ends mid-window emits a final record of the segments closed since the
last record, and none when no segment closed.

`window_time` sets the window in seconds, `segments` the number of parts it is read in, and `sample_rate` the rate it
counts against. A change of any of the three rebuilds the window and restarts the reading. The window spans at most
2^53 samples, `sample_rate` times `window_time`. The meter refuses a longer one and keeps the window it has.
`segment_samples` and `window_samples` report the window as built. `floor_db` sets the lowest level reported, from the
next read on, and keeps the window. A reset restarts the window, and the record stamps keep counting.
)"">;

    PortIn<T> in;
    /// One measurement record per completed window.
    PortOut<DataSet<float>, Async, Optional> records;

    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"stream rate">, Limits<0.f, std::numeric_limits<float>::max(), detail::kPowerMeterSampleRatePositive>>                              sample_rate     = 96000.f;
    Annotated<double, "window_time", Unit<"s">, Doc<"length of the rolling average, up to 2^53 samples">, Limits<0.0, std::numeric_limits<double>::max(), detail::kWindowTimePositive>> window_time     = 0.100;
    Annotated<gr::Size_t, "segments", Doc<"segments per window">, Limits<1U, std::numeric_limits<gr::Size_t>::max()>>                                                                   segments        = 16U;
    Annotated<double, "floor_db", Unit<"dBFS">, Doc<"lowest level reported">, Limits<std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max()>>                        floor_db        = -200.0;
    Annotated<std::uint64_t, "segment_samples", Doc<"observable realized segment length, at least one sample">>                                                                         segment_samples = 600U;
    Annotated<std::uint64_t, "window_samples", Doc<"observable realized window, segments * segment_samples">>                                                                           window_samples  = 9600U;

    GR_MAKE_REFLECTABLE(PowerMeter, in, records, sample_rate, window_time, segments, floor_db, segment_samples, window_samples);

    std::array<double, 8UZ> _lanes{};
    std::vector<double>     _segments{};
    std::uint64_t           _segmentSamples = 600ULL;
    std::uint64_t           _index          = 0ULL; ///< position within the current segment
    std::size_t             _completed      = 0UZ;
    float                   _rate           = 96000.f; ///< the `sample_rate` the window is built for
    double                  _windowTime     = 0.100;   ///< the `window_time` the window is built for

    // A reader touches only atomics. A settings change beside a poll is not a race.
    gr::measurement::MeasurementSlot<1UZ> _slot{};

    std::vector<DataSet<float>> _pending{};             ///< records built at a window's close and not yet published
    std::size_t                 _sinceRecord   = 0UZ;   ///< segments closed since the last record was built
    std::uint64_t               _streamAt      = 0ULL;  ///< stream index of the next input sample, counted from start()
    std::uint64_t               _windowStartAt = 0ULL;  ///< stream index of the first sample of the window now filling
    bool                        _flushed       = false; ///< the end-of-stream record has gone out
    std::atomic<std::uint32_t>  _windowSegments{16U};
    std::atomic<double>         _floorLinear{1e-20};
    std::atomic<double>         _floorDb{-200.0};

    /// Sizes the window for the default settings.
    explicit PowerMeter(property_map init = {}) : Block<PowerMeter<T>, NoTagPropagation>(std::move(init)) { configure({}, true); }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) { configure(newSettings, false); }

    /// @brief Take the settings @p newSettings names, every one when @p everything is set.
    ///
    /// A change of the rate, the window time or the segment count rebuilds the window. The stream index stays. A
    /// window longer than 2^53 samples throws before anything changes.
    void configure(const property_map& newSettings, bool everything) {
        const auto        moves   = [&newSettings, everything](std::string_view key) { return everything || newSettings.contains(key); };
        const float       rate    = moves("sample_rate") ? sample_rate.value : _rate;
        const double      time    = moves("window_time") ? window_time.value : _windowTime;
        const std::size_t count   = moves("segments") ? static_cast<std::size_t>(segments.value) : _segments.size();
        const bool        rebuild = everything || rate != _rate || time != _windowTime || count != _segments.size();
        const double      span    = static_cast<double>(rate) * time;
        if (rebuild && !(span <= detail::kMaxWindowSamples)) {
            throw gr::exception(std::format("the window of {} s at {} Hz is {} samples, more than 2^53", time, rate, span));
        }

        _floorDb.store(floor_db, std::memory_order_relaxed);
        _floorLinear.store(std::pow(10.0, floor_db / 10.0), std::memory_order_relaxed);
        if (!rebuild) {
            return;
        }
        // a span of at most 2^53 is inside the range of llround, and the window fits 64 bits
        _rate           = rate;
        _windowTime     = time;
        _segmentSamples = std::max<std::uint64_t>(1ULL, static_cast<std::uint64_t>(std::llround(span / static_cast<double>(count))));
        segment_samples = _segmentSamples;
        window_samples  = _segmentSamples * count;
        _segments.assign(count, 0.0);
        _windowSegments.store(static_cast<std::uint32_t>(count), std::memory_order_relaxed);
        reset();
    }

    void start() {
        _streamAt = 0ULL;
        _flushed  = false;
        _pending.clear();
        reset();
    }

    /// @brief Zero the window. Call it from the block's own thread.
    void reset() {
        _lanes.fill(0.0);
        std::ranges::fill(_segments, 0.0);
        _index         = 0ULL;
        _completed     = 0UZ;
        _sinceRecord   = 0UZ;
        _windowStartAt = _streamAt;
        publish(0.0, 0U);
    }

    /// @brief Mean power over the accumulated window, in dBFS, clamped at floor_db. Callable from any thread.
    [[nodiscard]] float level() const noexcept {
        const auto [power, filled] = read();
        return clampedLevel(power, filled);
    }

    /// @brief Fraction of the nominal window the reading covers, 0 to 1. Callable from any thread.
    [[nodiscard]] float coverage() const noexcept { return coverageOf(read().second); }

    /// @brief The same reading without the logarithm, as a ratio and not in decibels.
    [[nodiscard]] double linear_power() const noexcept { return read().first; }

    /// @brief Fold the input into the window and publish the records it closes.
    ///
    /// A call that moves nothing answers `INSUFFICIENT_INPUT_ITEMS` on an empty input and `INSUFFICIENT_OUTPUT_ITEMS`
    /// when a record has no room.
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::span<const T> input(inSpan);
        std::size_t              made = drain(outSpan, 0UZ);
        const std::size_t        take = std::min(input.size(), roomFor(outSpan, made));

        accumulate(input.first(take), outSpan.isConnected);
        made += drain(outSpan, made);

        outSpan.publish(made);
        std::ignore = inSpan.consume(take);
        if (take > 0UZ || made > 0UZ) {
            return work::Status::OK;
        }
        return input.empty() ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::INSUFFICIENT_OUTPUT_ITEMS;
    }

    /// @brief At end of stream, fold the trailing samples, then emit a record of the segments closed since the last one.
    ///
    /// The record starts at `sample_start` and covers `n_segments` segments. No record goes out when none closed.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        accumulate(std::span<const T>(inSpan), outSpan.isConnected);
        std::size_t made = drain(outSpan, 0UZ);

        if (!_flushed && outSpan.isConnected && _sinceRecord > 0UZ && made < outSpan.size()) {
            double total = 0.0;
            for (std::size_t back = _sinceRecord; back > 0UZ; --back) { // oldest first, a fixed order like the window's
                total += _segments[(_completed - back) % _segments.size()];
            }
            outSpan[made] = makeRecord(total / static_cast<double>(_sinceRecord), static_cast<std::uint32_t>(_sinceRecord));
            ++made;
            _flushed = true;
        }
        outSpan.publish(made);
        return work::Status::OK;
    }

    /// @brief The reading as a measurement record.
    ///
    /// It holds the power the window closed at, in the clamped form `level()` returns, beside the raw ratio and the
    /// coverage.
    [[nodiscard]] DataSet<float> makeRecord(double power, std::uint32_t filled) const {
        const std::array<gr::measurement::ScalarChannel, 3UZ> channels{{
            {"power", "Power", "dBFS", clampedLevel(power, filled)},
            {"power_linear", "Power", "1", static_cast<float>(power)},
            {"coverage", "Coverage", "1", coverageOf(filled)},
        }};
        return gr::measurement::makeScalarRecord(std::span<const gr::measurement::ScalarChannel>(channels), _rate, _windowStartAt, property_map{{std::pmr::string("n_segments"), pmt::Value(static_cast<std::uint64_t>(filled))}, {std::pmr::string("window_samples"), pmt::Value(window_samples.value)}});
    }

private:
    [[nodiscard]] static constexpr double squaredMagnitude(T sample) noexcept {
        if constexpr (std::same_as<T, float>) {
            return static_cast<double>(sample) * static_cast<double>(sample);
        } else {
            return static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
        }
    }

    /// @brief The reading in dBFS with the floor applied, as `level()` and a record state it.
    [[nodiscard]] float clampedLevel(double power, std::uint32_t filled) const noexcept {
        if (filled == 0U || power <= _floorLinear.load(std::memory_order_relaxed)) {
            return static_cast<float>(_floorDb.load(std::memory_order_relaxed));
        }
        return static_cast<float>(10.0 * std::log10(power));
    }

    [[nodiscard]] float coverageOf(std::uint32_t filled) const noexcept { return std::min(1.f, static_cast<float>(filled) / static_cast<float>(_windowSegments.load(std::memory_order_relaxed))); }

    /// @brief How many input samples may be taken before a record would have nowhere to go.
    ///
    /// A record closes every `window_samples` samples. The room is that many samples per free output slot, less what
    /// the current window already holds.
    [[nodiscard]] std::size_t roomFor(OutputSpanLike auto& outSpan, std::size_t made) const {
        if (!outSpan.isConnected) {
            return std::numeric_limits<std::size_t>::max();
        }
        const std::size_t   free          = outSpan.size() > made + _pending.size() ? outSpan.size() - made - _pending.size() : 0UZ;
        const std::uint64_t windowSamples = _segments.size() * _segmentSamples;
        const std::uint64_t into          = _sinceRecord * _segmentSamples + _index;
        const std::uint64_t untilClose    = windowSamples > into ? windowSamples - into : 0ULL;
        if (free == 0UZ) {
            return detail::saturatedSize(untilClose > 0ULL ? untilClose - 1ULL : 0ULL);
        }
        const std::uint64_t windows = free - 1UZ;
        if (windows > (std::numeric_limits<std::uint64_t>::max() - untilClose) / windowSamples) {
            return std::numeric_limits<std::size_t>::max();
        }
        return detail::saturatedSize(untilClose + windows * windowSamples);
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

    void accumulate(std::span<const T> input, bool wantRecords) {
        std::array<double, 8UZ> lanes = _lanes;

        for (std::size_t at = 0UZ; at < input.size();) {
            const std::size_t run   = static_cast<std::size_t>(std::min<std::uint64_t>(input.size() - at, _segmentSamples - _index)); // at most input.size()
            const std::size_t phase = static_cast<std::size_t>(_index & 7ULL);
            std::size_t       k     = 0UZ;

            for (; k < run && ((phase + k) & 7UZ) != 0UZ; ++k) { // up to a lane boundary; the loop below takes eight at a time
                lanes[(phase + k) & 7UZ] += squaredMagnitude(input[at + k]);
            }
            for (; k + 8UZ <= run; k += 8UZ) {
                for (std::size_t lane = 0UZ; lane < 8UZ; ++lane) {
                    lanes[lane] += squaredMagnitude(input[at + k + lane]);
                }
            }
            for (; k < run; ++k) {
                lanes[(phase + k) & 7UZ] += squaredMagnitude(input[at + k]);
            }

            _index += run;
            at += run;
            _streamAt += run;
            if (_index == _segmentSamples) {
                closeSegment(lanes, wantRecords);
            }
        }

        _lanes = lanes;
    }

    void closeSegment(std::array<double, 8UZ>& lanes, bool wantRecords) {
        double total = 0.0;
        for (const double lane : lanes) { // a fixed index order: the sum depends on the input alone
            total += lane;
        }
        lanes.fill(0.0);
        _index = 0ULL;

        _segments[_completed % _segments.size()] = total / static_cast<double>(_segmentSamples);
        ++_completed;

        const std::size_t filled = std::min(_completed, _segments.size());
        double            mean   = 0.0;
        for (std::size_t slot = 0UZ; slot < filled; ++slot) { // slot order, fixed for the same reason
            mean += _segments[slot];
        }
        mean /= static_cast<double>(filled);
        publish(mean, static_cast<std::uint32_t>(filled));

        // A record covers one completed window. It holds the reading at that window's close.
        if (++_sinceRecord >= _segments.size()) {
            _sinceRecord = 0UZ;
            if (wantRecords) {
                _pending.push_back(makeRecord(mean, static_cast<std::uint32_t>(filled)));
            }
            _windowStartAt = _streamAt;
        }
    }

    /// The one reading and the segments behind it, carried across the thread boundary by the seqlock.
    void publish(double power, std::uint32_t filled) noexcept { _slot.publish({power}, static_cast<std::uint64_t>(filled)); }

    [[nodiscard]] std::pair<double, std::uint32_t> read() const noexcept {
        const auto [values, filled] = _slot.read();
        return {values[0], static_cast<std::uint32_t>(filled)};
    }
};

} // namespace gr::blocks::analog

#endif // GNURADIO_POWER_METER_HPP
