#ifndef GNURADIO_MEASUREMENT_KURTOSIS_HPP
#define GNURADIO_MEASUREMENT_KURTOSIS_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
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
#include <gnuradio-4.0/algorithm/measurement/Kurtosis.hpp>

namespace gr::blocks::measurement {

namespace detail {
inline constexpr gr::Size_t kMinKurtosisWindow = 8U;
inline constexpr gr::Size_t kMaxKurtosisWindow = 1U << 24U;

/// @brief The validator of `sample_rate`: true above 0.
inline constexpr auto kKurtosisSampleRatePositive = [](float value) { return value > 0.f; };

/// @brief The settings `Kurtosis` computes on, as the applications it accepted left them.
struct KurtosisSettings {
    gr::Size_t  window{};
    float       sampleRate{};
    bool        emitRecords{};
    std::string signalName{};
};
} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::measurement::Kurtosis, [T], [ float, std::complex<float> ])

/**
 * @brief Measures the normalized fourth moment of a stream over consecutive windows, as excess kurtosis and as
 * `m4/m2^2`.
 *
 * The block is a sink at the end of a sample stream. Kurtosis tells a constant-envelope or Gaussian signal from an
 * impulsive one. Each window of `window` samples gives one reading. The first window starts at the first sample of the
 * run, and each window starts where the previous one closed.
 *
 * The block takes `float` or `std::complex<float>` samples. A reading holds `excess_kurtosis`,
 * `normalized_fourth_moment` and `mean_power`, the mean of `|x|^2`. The Gaussian value of `m4/m2^2` is 2 for complex
 * input and 3 for real input, and the excess is the reading less that value. `excessKurtosis()`,
 * `normalizedFourthMoment()`, `meanPower()` and `coverage()` read the latest reading from any thread, each on its own.
 * `coverage()` reads the fraction of `window` the latest reading covers, 1 for a full window.
 *
 * With `emit_records` set and the `measurements` port connected, the block also emits one scalar `DataSet<float>`
 * record per reading. Its metadata states `input_domain` ('complex' or 'real'), `window`, `n_samples`, `valid`,
 * `signal_name` and `sample_rate`, and its `sample_start` is the stream index of the window's first sample. When
 * `gr:n_dropped_samples` tags report missing samples within the window, the record states their sum in
 * `n_dropped_samples`. A window that held a NaN or infinite sample states `non_finite` true. A window of zero power or
 * with a non-finite sample reads zero in all three values and states `valid` false. The block passes no input tag on.
 *
 * A change of `window` discards the partial window and starts a new one at the current stream position. `sample_rate`
 * and `signal_name` are stated in the records that follow a change. At the end of the stream the block emits the
 * partial window. Its `n_samples` states the samples it covers, and it is valid from 8 samples.
 *
 * `nWindows()` counts the full windows, and `nDegenerateWindows()` the readings of zero power or with a non-finite
 * sample. `nWindowResets()` counts the partial windows a change of `window` discarded, and `nDroppedSampleTags()` the
 * `gr:n_dropped_samples` tags on the input.
 */
template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct Kurtosis : Block<Kurtosis<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Measures excess kurtosis and the normalized fourth moment `m4/m2^2` over consecutive windows of a stream.

The block is a sink. Every `window` samples it reports `excess_kurtosis`, `normalized_fourth_moment` and `mean_power`.
`excess_kurtosis` is 0 for a zero-mean Gaussian of the input's domain. The block keeps the latest figures for polling
and emits one `DataSet<float>` record per window when `emit_records` is set. An all-zero window reads zero, and its
record has `valid` false. It passes no input tag on.
)"">;

    static constexpr bool kComplexDomain = std::same_as<T, std::complex<float>>;

    PortIn<T>                                in;
    PortOut<DataSet<float>, Async, Optional> measurements;

    Annotated<gr::Size_t, "window", Visible, Doc<"samples per reading, 8 to 2^24">, Limits<detail::kMinKurtosisWindow, detail::kMaxKurtosisWindow>>                                 window       = 4096U;
    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"input sample rate, positive and finite">, Limits<0.f, std::numeric_limits<float>::max(), detail::kKurtosisSampleRatePositive>> sample_rate  = 96000.f;
    Annotated<bool, "emit_records", Visible, Doc<"publish one DataSet<float> record per window">>                                                                                   emit_records = true;
    Annotated<std::string, "signal_name", Doc<"label written to each record's metadata">>                                                                                           signal_name  = std::string("kurtosis");

    GR_MAKE_REFLECTABLE(Kurtosis, in, measurements, window, sample_rate, emit_records, signal_name);

    gr::measurement::KurtosisAccumulator<T> _acc{};
    gr::measurement::MeasurementSlot<4UZ>   _slot{}; ///< excess_kurtosis, normalized_fourth_moment, mean_power, coverage

    /// @brief One `gr:n_dropped_samples` tag, with its stream position and the number of samples it reports missing.
    struct DroppedTag {
        std::uint64_t at;
        std::uint64_t count;
    };

    std::atomic<std::uint64_t> _nWindows{0ULL};
    std::atomic<std::uint64_t> _nDegenerateWindows{0ULL};
    std::atomic<std::uint64_t> _nWindowResets{0ULL};
    std::atomic<std::uint64_t> _nDroppedSampleTags{0ULL};

    detail::KurtosisSettings _accepted{.window = window.value, .sampleRate = sample_rate.value, .emitRecords = emit_records.value, .signalName = signal_name.value}; ///< the settings the work thread reads

    std::uint64_t               _streamAt{0ULL};        ///< samples taken since `start()`
    std::uint64_t               _windowStartAt{0ULL};   ///< the position of the first sample of the window now filling
    bool                        _flushed{false};        ///< the end-of-stream flush has run
    std::vector<DroppedTag>     _droppedTags{};         ///< the dropped-sample tags on the samples this call takes
    std::uint64_t               _droppedInWindow{0ULL}; ///< samples reported missing in the window now filling
    std::vector<DataSet<float>> _pending{};             ///< records built at a window's close and not yet published

    /// @brief Accepts the keys @p newSettings names. A new window discards the partial window.
    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const auto               moved = [&newSettings](std::string_view key) { return newSettings.contains(key); };
        detail::KurtosisSettings next  = _accepted;
        if (moved("window")) {
            next.window = window.value;
        }
        if (moved("sample_rate")) {
            next.sampleRate = sample_rate.value;
        }
        if (moved("emit_records")) {
            next.emitRecords = emit_records.value;
        }
        if (moved("signal_name")) {
            next.signalName = signal_name.value;
        }
        if (next.window != _accepted.window) {
            if (_acc.count() > 0UZ) {
                _nWindowResets.fetch_add(1ULL, std::memory_order_relaxed);
            }
            _acc.reset();
            _windowStartAt   = _streamAt;
            _droppedInWindow = 0ULL;
        }
        _accepted = std::move(next);
    }

    /// @brief Clears the window and the counters.
    void start() {
        _acc.reset();
        _nWindows.store(0ULL, std::memory_order_relaxed);
        _nDegenerateWindows.store(0ULL, std::memory_order_relaxed);
        _nWindowResets.store(0ULL, std::memory_order_relaxed);
        _nDroppedSampleTags.store(0ULL, std::memory_order_relaxed);
        _streamAt        = 0ULL;
        _windowStartAt   = 0ULL;
        _flushed         = false;
        _droppedInWindow = 0ULL;
        _droppedTags.clear();
        _pending.clear();
        _slot.publish({0., 0., 0., 0.}, 0ULL);
    }

    /// @brief Excess kurtosis of the last completed or flushed window. Callable from any thread.
    [[nodiscard]] double excessKurtosis() const noexcept { return _slot.read().first[0]; }
    /// @brief `m4/m2^2` of the last completed or flushed window. Callable from any thread.
    [[nodiscard]] double normalizedFourthMoment() const noexcept { return _slot.read().first[1]; }
    /// @brief Mean of `|x|^2` over the last completed or flushed window. Callable from any thread.
    [[nodiscard]] double meanPower() const noexcept { return _slot.read().first[2]; }
    /// @brief Fraction of its window the last reading covers, 0 before the first window closes. Callable from any
    /// thread.
    [[nodiscard]] double coverage() const noexcept { return _slot.read().first[3]; }

    [[nodiscard]] std::uint64_t nWindows() const noexcept { return _nWindows.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nDegenerateWindows() const noexcept { return _nDegenerateWindows.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nWindowResets() const noexcept { return _nWindowResets.load(std::memory_order_relaxed); }
    /// @brief `gr:n_dropped_samples` tags seen on the input stream. Callable from any thread.
    [[nodiscard]] std::uint64_t nDroppedSampleTags() const noexcept { return _nDroppedSampleTags.load(std::memory_order_relaxed); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::span<const T> input(inSpan);
        const bool               wantRecords = _accepted.emitRecords && outSpan.isConnected;
        std::size_t              made        = drain(outSpan, 0UZ);
        const std::size_t        take        = wantRecords ? roomFor(outSpan, made, input.size()) : input.size();

        scanDroppedTags(inSpan, take);
        accumulate(input.first(take), wantRecords);
        made += drain(outSpan, made);

        outSpan.publish(made);
        std::ignore = inSpan.consume(take);
        return take == 0UZ && made == 0UZ && !input.empty() ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }

    /// @brief Folds in the trailing samples at the end of the stream and flushes the partial window. The reading
    /// states the sample count it covers.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const bool wantRecords = _accepted.emitRecords && outSpan.isConnected;
        scanDroppedTags(inSpan, inSpan.size());
        accumulate(std::span<const T>(inSpan), wantRecords);
        if (!_flushed && _acc.count() > 0UZ) {
            publishReading(_acc.count(), wantRecords);
            _flushed = true;
        }
        outSpan.publish(drain(outSpan, 0UZ));
        return work::Status::OK;
    }

private:
    /// @brief Collects the `gr:n_dropped_samples` tags on the first @p count samples of @p inSpan. A tag on a later
    /// sample waits for the call that takes that sample.
    void scanDroppedTags(InputSpanLike auto& inSpan, std::size_t count) {
        _droppedTags.clear();
        const std::size_t first = inSpan.streamIndex;
        for (const gr::Tag& tag : inSpan.rawTags) {
            if (tag.index < first || tag.index - first >= count) {
                continue;
            }
            const auto it = tag.map.find(gr::tag::N_DROPPED_SAMPLES.key());
            if (it == tag.map.end()) {
                continue;
            }
            _droppedTags.push_back({_streamAt + (tag.index - first), droppedCount(it->second)});
            _nDroppedSampleTags.fetch_add(1ULL, std::memory_order_relaxed);
        }
    }

    /// @brief The tag's count as `gr::Size_t`, the key's own type, or as `std::uint64_t`. Any other type counts zero.
    [[nodiscard]] static std::uint64_t droppedCount(const pmt::Value& value) {
        if (const auto* asSize = value.get_if<gr::Size_t>(); asSize != nullptr) {
            return static_cast<std::uint64_t>(*asSize);
        }
        if (const auto* asWide = value.get_if<std::uint64_t>(); asWide != nullptr) {
            return *asWide;
        }
        return 0ULL;
    }

    /// @brief The missing samples reported by this call's tags in `[from, from + count)`.
    [[nodiscard]] std::uint64_t droppedWithin(std::uint64_t from, std::size_t count) const {
        std::uint64_t total = 0ULL;
        for (const DroppedTag& tag : _droppedTags) {
            if (tag.at >= from && tag.at < from + count) {
                total += tag.count;
            }
        }
        return total;
    }

    void accumulate(std::span<const T> input, bool wantRecords) {
        const std::size_t windowSize = static_cast<std::size_t>(_accepted.window);
        std::size_t       at         = 0UZ;
        while (at < input.size()) {
            const std::size_t take = std::min(windowSize - _acc.count(), input.size() - at);
            _acc.add(input.subspan(at, take));
            _droppedInWindow += droppedWithin(_streamAt, take);
            at += take;
            _streamAt += take;
            if (_acc.count() == windowSize) {
                publishReading(windowSize, wantRecords);
                _nWindows.fetch_add(1ULL, std::memory_order_relaxed);
                _acc.reset();
                _windowStartAt   = _streamAt;
                _droppedInWindow = 0ULL;
            }
        }
    }

    /// @brief Publishes the reading of the @p nSamples now accumulated to the slot, and queues its record when
    /// @p wantRecords is set.
    void publishReading(std::size_t nSamples, bool wantRecords) {
        const bool degenerate = _acc.degenerate();
        if (degenerate) {
            _nDegenerateWindows.fetch_add(1ULL, std::memory_order_relaxed);
        }
        const double covered = std::min(1., static_cast<double>(nSamples) / static_cast<double>(_accepted.window));
        _slot.publish({_acc.excess(), _acc.normalized(), _acc.meanPower(), covered}, static_cast<std::uint64_t>(nSamples));
        if (wantRecords) {
            _pending.push_back(makeRecord(nSamples, !degenerate && nSamples >= static_cast<std::size_t>(detail::kMinKurtosisWindow)));
        }
    }

    [[nodiscard]] DataSet<float> makeRecord(std::size_t nSamples, bool valid) const {
        const std::array<gr::measurement::ScalarChannel, 3UZ> channels{{
            {"excess_kurtosis", "kurtosis", "", static_cast<float>(_acc.excess())},
            {"normalized_fourth_moment", "kurtosis", "", static_cast<float>(_acc.normalized())},
            {"mean_power", "power", "", static_cast<float>(_acc.meanPower())},
        }};
        property_map                                          extra{
                                                     {std::pmr::string("n_samples"), pmt::Value(static_cast<std::uint64_t>(nSamples))},
                                                     {std::pmr::string("window"), pmt::Value(static_cast<std::uint64_t>(_accepted.window))},
                                                     {std::pmr::string("input_domain"), pmt::Value(std::string(kComplexDomain ? "complex" : "real"))},
                                                     {std::pmr::string("valid"), pmt::Value(valid)},
                                                     {std::pmr::string("signal_name"), pmt::Value(_accepted.signalName)},
        };
        if (_droppedInWindow > 0ULL) {
            extra.insert_or_assign(std::pmr::string("n_dropped_samples"), pmt::Value(_droppedInWindow));
        }
        if (!_acc.finite()) {
            extra.insert_or_assign(std::pmr::string("non_finite"), pmt::Value(true));
        }
        return gr::measurement::makeScalarRecord(std::span<const gr::measurement::ScalarChannel>(channels), _accepted.sampleRate, _windowStartAt, std::move(extra));
    }

    /// @brief The input samples, at most @p offer, this call can take with room for every record they complete. The
    /// product of the further windows is formed only when it fits in the offer.
    [[nodiscard]] std::size_t roomFor(OutputSpanLike auto& outSpan, std::size_t made, std::size_t offer) const {
        if (!outSpan.isConnected) {
            return offer;
        }
        const std::size_t   free        = outSpan.size() > made + _pending.size() ? outSpan.size() - made - _pending.size() : 0UZ;
        const std::uint64_t windowSize  = _accepted.window;
        const std::uint64_t accumulated = _acc.count();
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

    /// @brief Moves pending records into the output span from @p at. Returns the number moved.
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
};

} // namespace gr::blocks::measurement

#endif // GNURADIO_MEASUREMENT_KURTOSIS_HPP
