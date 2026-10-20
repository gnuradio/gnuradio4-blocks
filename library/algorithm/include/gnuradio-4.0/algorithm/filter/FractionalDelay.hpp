#ifndef GNURADIO_FRACTIONAL_DELAY_HPP
#define GNURADIO_FRACTIONAL_DELAY_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <source_location>
#include <span>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/algorithm/filter/ArbitraryResampler.hpp>

/**
 * @brief A delay line whose fractional delay may change on every sample, over the arbitrary resampler's bank.
 *
 * `FractionalDelayLine` delays a stream by a time that may change from one sample to the next, such as the envelope
 * delay of a moving source. It cuts its prototype with `detail::polyphaseArmBank` and reads it with
 * `detail::polyphaseArmBlend`, as `ArbitraryResampler` does. Its read position advances by `1 - dtau/dt` per output
 * where the resampler's advances by a fixed `step`.
 *
 * A delay is `Q32` fixed point in samples, with the `kArbitraryFractionBits` of the resampler. The whole part selects
 * how far back in the history to read. The fraction selects the arm and `mu`. `fractionalDelayQ32` converts a `double`
 * delay and refuses one at or past `kMaxFractionalDelaySamples`.
 *
 * For a signal band-limited to `f_max = (1 - rolloff)/2` cycles per sample and `L` arms, the error against an ideal
 * fractional delay is at most `h/2`, `h^2/8` and `h^4 * 9/384` at orders 0, 1 and 3, with `h = 2*pi*f_max/L`.
 * `arbitraryInterpolationError` computes that bound. The prototype's passband ripple is the floor below it. A larger
 * bank helps only while the bound is above that floor. Under a moving delay the bound holds with
 * `f_max / (1 - dtau/dt)` in place of `f_max`.
 *
 * `process` allocates nothing. Each output costs `q+1` dot products of `B+1` taps, or one blended product for complex
 * `T`.
 */
namespace gr::filter {

/// @brief The bound on a delay in samples, `2^63 / 2^32`. A `Q32` delay below it fits a `std::int64_t`.
inline constexpr std::uint64_t kMaxFractionalDelaySamples = 1ULL << 31;

/// @brief The `maxTaps` of `designFractionalDelay`. A design that needs more returns `kFractionalDelayMaxTaps | 1`
/// taps and is not `ok`.
inline constexpr std::size_t kFractionalDelayMaxTaps = 1UZ << 16;

namespace detail {

/// The least ratio of designed to estimated length seen at each attenuation from 20 to 45 dB, where the 0.1 dB ripple
/// limit and not the attenuation sets the length. One table per rolloff range: up to 0.7, above 0.7 up to 0.85, and
/// above 0.85, where the passband is narrow and the limit binds least.
inline constexpr std::array<std::pair<double, double>, 6> kRippleLimitedFactor{{{20., 10.}, {25., 10.}, {30., 4.5}, {35., 2.}, {40., 1.}, {45., 0.99}}};
inline constexpr std::array<std::pair<double, double>, 6> kMiddleRolloffFactor{{{20., 12.}, {25., 8.}, {30., 1.5}, {35., 1.1}, {40., 1.}, {45., 0.99}}};
inline constexpr std::array<std::pair<double, double>, 4> kWideRolloffFactor{{{20., 1.5}, {25., 1.1}, {30., 0.95}, {45., 0.99}}};

/// The widest rolloffs the first two tables cover.
inline constexpr double kNarrowRolloffMax = 0.7;
inline constexpr double kMiddleRolloffMax = 0.85;

/// The factor @p table gives at @p attenuationDb: the end values outside its range and a geometric join inside.
template<std::size_t N>
[[nodiscard]] inline double fractionalDelayTableFactor(const std::array<std::pair<double, double>, N>& table, double attenuationDb) {
    if (attenuationDb <= table.front().first) {
        return table.front().second;
    }
    for (std::size_t i = 1UZ; i < table.size(); ++i) {
        if (attenuationDb < table[i].first) {
            const double t = (attenuationDb - table[i - 1UZ].first) / (table[i].first - table[i - 1UZ].first);
            return std::exp((1. - t) * std::log(table[i - 1UZ].second) + t * std::log(table[i].second));
        }
    }
    return table.back().second;
}

/// The factor between Kaiser's estimate and the designed length at @p attenuationDb and @p rolloff. Below 45 dB it
/// comes from the table for the rolloff. From 45 to 100 dB it is 0.99. Above 100 dB it is `1 + 5e-5 * (attenuationDb - 100)`.
[[nodiscard]] inline double fractionalDelayLengthFactor(double attenuationDb, double rolloff) {
    constexpr double kFloor = 0.99;
    if (attenuationDb < kRippleLimitedFactor.back().first) {
        if (rolloff <= kNarrowRolloffMax) {
            return fractionalDelayTableFactor(kRippleLimitedFactor, attenuationDb);
        }
        if (rolloff <= kMiddleRolloffMax) {
            return fractionalDelayTableFactor(kMiddleRolloffFactor, attenuationDb);
        }
        return fractionalDelayTableFactor(kWideRolloffFactor, attenuationDb);
    }
    return attenuationDb <= 100. ? kFloor : 1. + 5e-5 * (attenuationDb - 100.);
}

} // namespace detail

/**
 * @brief The prototype length `designFractionalDelay` needs for @p bankSize arms, a @p rolloff and an @p attenuationDb.
 *
 * It is `fir::design::kaiserLength` over the transition the design cuts, `rolloff` times the stopband edge `0.5/L`,
 * times a factor for the cases where the design runs longer: below 45 dB the ripple limit sets the length, and above
 * 100 dB the stopband needs slightly more than the estimate. The factors are the least ratios of designed to estimated
 * length seen on probe grids of 1 to 2048 arms, rolloffs of 0.02 to 0.99 and 20 to 140 dB. On those grids the result
 * does not exceed the length the design finds, to about one percent at the attenuations of the tables and up to about
 * 20 percent over between them. Beyond them it is not proved. Below 45 dB a rolloff above 0.7 has smaller factors,
 * which read the length short where the design runs far longer, so the check admits some combinations the design
 * cannot hold. A transition below 1e-6 cycles per sample counts as 1e-6, and an attenuation above 1000 dB as 1000 dB.
 * A bank of no arms, a rolloff that is not positive and finite and an attenuation that is not finite return zero.
 */
[[nodiscard]] inline std::size_t fractionalDelayTapEstimate(std::size_t bankSize, double rolloff, double attenuationDb) {
    constexpr double kNarrowest = 1e-6;
    constexpr double kMostDb    = 1000.;
    if (bankSize == 0UZ || !std::isfinite(rolloff) || !(rolloff > 0.) || !std::isfinite(attenuationDb)) {
        return 0UZ;
    }
    const double db     = std::min(attenuationDb, kMostDb);
    const double width  = std::max(0.5 * rolloff / static_cast<double>(bankSize), kNarrowest);
    const double length = static_cast<double>(fir::design::kaiserLength(db, width)) * detail::fractionalDelayLengthFactor(db, rolloff);
    return static_cast<std::size_t>(std::ceil(length));
}

/**
 * @brief True when `fractionalDelayTapEstimate` is a nonzero length under `kFractionalDelayMaxTaps`.
 *
 * It reads three numbers and designs nothing. A combination it refuses is past the cap on the grids the estimate was
 * measured on. One it accepts can still miss its target: the design then returns a length of
 * `kFractionalDelayMaxTaps | 1` that is not `ok`. A design near the cap takes tens of seconds to a few minutes, and
 * longer in a Debug build.
 */
[[nodiscard]] inline bool fractionalDelayReachable(std::size_t bankSize, double rolloff, double attenuationDb) {
    const std::size_t estimate = fractionalDelayTapEstimate(bankSize, rolloff, attenuationDb);
    return estimate != 0UZ && estimate < kFractionalDelayMaxTaps;
}

/**
 * @brief The prototype for a pure delay over a bank of @p bankSize arms.
 *
 * It is `designArbitraryResampler` at a rate of one, with the stopband edge at `0.5/L`. A bank of no arms is an
 * error. A combination `fractionalDelayReachable` refuses takes seconds to come back with a design that is not `ok`.
 */
[[nodiscard]] inline std::expected<ResamplerDesign, gr::Error> designFractionalDelay(std::size_t bankSize, double rolloff = 0.2, double attenuationDb = 60.0, double maxRippleDb = 0.1, std::source_location location = std::source_location::current()) {
    if (bankSize == 0UZ) {
        return std::unexpected(gr::Error("gr::filter::designFractionalDelay: the bank must have at least one arm", location));
    }
    return designArbitraryResampler(bankSize, 1.0, rolloff, attenuationDb, maxRippleDb, kFractionalDelayMaxTaps);
}

/// @brief A delay in whole and fractional samples as the `Q32` the line consumes. A delay that is negative, not
/// finite or past `kMaxFractionalDelaySamples` is an error.
[[nodiscard]] inline std::expected<std::uint64_t, gr::Error> fractionalDelayQ32(double delaySamples, std::source_location loc = std::source_location::current()) {
    if (!std::isfinite(delaySamples) || delaySamples < 0.) {
        return std::unexpected(gr::Error(std::format("gr::filter::fractionalDelayQ32: a delay of {} samples is not a finite non-negative number", delaySamples), loc));
    }
    if (delaySamples >= static_cast<double>(kMaxFractionalDelaySamples)) {
        return std::unexpected(gr::Error(std::format("gr::filter::fractionalDelayQ32: {} samples is at or past the {} the fixed point carries", delaySamples, kMaxFractionalDelaySamples), loc));
    }
    return static_cast<std::uint64_t>(std::llround(delaySamples * static_cast<double>(kArbitraryOne)));
}

/**
 * @brief Converts delays in seconds, one per sample, to `Q32` samples at @p sampleRateHz.
 *
 * A length mismatch, a sample rate that is not positive and finite, or a delay the scalar form refuses is an error.
 * The outputs before a refused delay are written.
 */
[[nodiscard]] inline std::expected<void, gr::Error> fractionalDelayQ32(std::span<const double> delaySeconds, double sampleRateHz, std::span<std::uint64_t> out, std::source_location loc = std::source_location::current()) {
    if (out.size() != delaySeconds.size()) {
        return std::unexpected(gr::Error(std::format("gr::filter::fractionalDelayQ32: {} delays against {} outputs", delaySeconds.size(), out.size()), loc));
    }
    if (!std::isfinite(sampleRateHz) || !(sampleRateHz > 0.)) {
        return std::unexpected(gr::Error(std::format("gr::filter::fractionalDelayQ32: a sample rate of {} Hz is not positive and finite", sampleRateHz), loc));
    }
    for (std::size_t i = 0UZ; i < delaySeconds.size(); ++i) {
        const std::expected<std::uint64_t, gr::Error> q32 = fractionalDelayQ32(delaySeconds[i] * sampleRateHz, loc);
        if (!q32) {
            return std::unexpected(q32.error());
        }
        out[i] = *q32;
    }
    return {};
}

/**
 * @brief A variable fractional-delay line with one output per (input sample, delay) pair.
 *
 * The line holds the history its largest delay needs. Output `k` depends on the input up to sample `k` and the
 * delay commanded at `k`. The output is therefore the same however the input is split into calls.
 *
 * At a commanded delay of zero the output lags the input by `latencySamples()`, which is `1 + groupDelaySamples()`.
 * The group delay is the prototype's own, `(N-1)/(2L)` input samples, and stays in the output. The extra whole
 * sample is the reach of the wrap arm's first tap, one input sample past the window's anchor.
 *
 * The constructor throws `gr::exception` for a value outside its domain. `setTaps`, `setMaxDelay`, `growMaxDelay`
 * and `process` return their error, and a refused call changes nothing.
 */
template<typename T>
requires PolyphaseSample<T>
class FractionalDelayLine {
    using Traits      = detail::PolyphaseTraits<T, float>;
    using SampleValue = typename Traits::SampleValue;

    static constexpr std::size_t kSampleLanes = Traits::kSampleLanes;

public:
    using sample_type = T;

    /**
     * @brief A bank of @p bankSize arms at interpolation @p order over @p taps, holding history for @p maxWholeDelay samples.
     *
     * @p order is 0 (nearest arm), 1 (linear) or 3 (cubic Lagrange).
     */
    FractionalDelayLine(std::size_t bankSize, int order, std::span<const float> taps, std::uint64_t maxWholeDelay = 0ULL) : _bankSize(bankSize), _order(order) {
        if (bankSize == 0UZ) {
            throw gr::exception("FractionalDelayLine: the bank must have at least one arm");
        }
        if (order != 0 && order != 1 && order != 3) {
            throw gr::exception("FractionalDelayLine: the interpolation order is 0, 1 or 3");
        }
        if (taps.empty()) {
            throw gr::exception("FractionalDelayLine: no taps");
        }
        build(taps);
        if (const std::expected<void, gr::Error> sized = setMaxDelay(maxWholeDelay); !sized) {
            throw gr::exception(sized.error().message, sized.error().sourceLocation);
        }
    }

    /// @brief Clear the history. The next `process` starts a new stream on silence.
    void reset() { std::ranges::fill(_window, T{}); }

    /// @brief Install a new prototype and clear the history.
    [[nodiscard]] std::expected<void, gr::Error> setTaps(std::span<const float> taps, std::source_location loc = std::source_location::current()) {
        if (taps.empty()) {
            return std::unexpected(gr::Error("FractionalDelayLine: no taps", loc));
        }
        build(taps);
        return setMaxDelay(_maxWholeDelay, loc);
    }

    /// @brief Size the history for delays up to @p maxWholeDelay whole samples, and clear it.
    [[nodiscard]] std::expected<void, gr::Error> setMaxDelay(std::uint64_t maxWholeDelay, std::source_location loc = std::source_location::current()) {
        if (maxWholeDelay >= kMaxFractionalDelaySamples) {
            return std::unexpected(gr::Error(std::format("FractionalDelayLine: a history for {} samples is at or past the {} the fixed point carries", maxWholeDelay, kMaxFractionalDelaySamples), loc));
        }
        _maxWholeDelay = maxWholeDelay;
        _history       = historyNeeded(maxWholeDelay);
        _window.assign(_history + _windowLength, T{});
        return {};
    }

    /**
     * @brief Reach back for delays up to @p maxWholeDelay whole samples, keeping the history already held.
     *
     * The held samples move to the end of the longer window, and the part before them reads as silence. A request
     * the line already covers changes nothing.
     */
    [[nodiscard]] std::expected<void, gr::Error> growMaxDelay(std::uint64_t maxWholeDelay, std::source_location loc = std::source_location::current()) {
        if (maxWholeDelay <= _maxWholeDelay) {
            return {};
        }
        if (maxWholeDelay >= kMaxFractionalDelaySamples) {
            return std::unexpected(gr::Error(std::format("FractionalDelayLine: a history for {} samples is at or past the {} the fixed point carries", maxWholeDelay, kMaxFractionalDelaySamples), loc));
        }
        const std::size_t grown = historyNeeded(maxWholeDelay);
        std::vector<T>    next(grown + _windowLength, T{});
        const std::size_t kept = std::min(_history, grown);
        std::copy_n(_window.begin() + static_cast<std::ptrdiff_t>(_history - kept), kept, next.begin() + static_cast<std::ptrdiff_t>(grown - kept));
        _window        = std::move(next);
        _maxWholeDelay = maxWholeDelay;
        _history       = grown;
        return {};
    }

    /// @brief The history a delay of @p maxDelayWhole whole samples needs, `maxDelayWhole + tapsPerArm + 1` samples.
    [[nodiscard]] std::size_t historyNeeded(std::uint64_t maxDelayWhole) const noexcept { return static_cast<std::size_t>(maxDelayWhole) + _windowLength + 1UZ; }

    [[nodiscard]] std::size_t bankSize() const noexcept { return _bankSize; }
    [[nodiscard]] int         order() const noexcept { return _order; }
    /// @brief `B = ceil(N/L)`. An arm stores `B+1` taps. The extra tap is the wrap term, zero below arm `L`.
    [[nodiscard]] std::size_t tapsPerArm() const noexcept { return _windowLength; }
    /// @brief `N`, the prototype as supplied, before padding to a multiple of `L`.
    [[nodiscard]] std::size_t prototypeLength() const noexcept { return _prototypeLength; }
    /// @brief The prototype's group delay, `(N-1)/(2L)` input samples.
    [[nodiscard]] double groupDelaySamples() const noexcept { return 0.5 * static_cast<double>(_prototypeLength - 1UZ) / static_cast<double>(_bankSize); }
    /// @brief The output's lag behind the input at a commanded delay of zero, `1 + groupDelaySamples()`.
    [[nodiscard]] double        latencySamples() const noexcept { return 1. + groupDelaySamples(); }
    [[nodiscard]] std::uint64_t maxDelaySamples() const noexcept { return _maxWholeDelay; }
    [[nodiscard]] std::size_t   historySamples() const noexcept { return _history; }

    /**
     * @brief One output per (input sample, delay) pair, `out[k]` the input at `k - latency - delayQ32[k]/2^32`.
     *
     * @param in        the input samples
     * @param delayQ32  one delay per input sample, in whole samples in the high 32 bits
     * @param out       at least `in.size()` outputs
     *
     * Returns an error for unpaired spans, too few outputs, or a delay whose whole part exceeds the history the
     * line was sized for, and then writes nothing.
     */
    [[nodiscard]] std::expected<void, gr::Error> process(std::span<const T> in, std::span<const std::uint64_t> delayQ32, std::span<T> out, std::source_location loc = std::source_location::current()) {
        if (in.size() != delayQ32.size()) {
            return std::unexpected(gr::Error(std::format("FractionalDelayLine::process: {} samples against {} delays; each sample needs one delay", in.size(), delayQ32.size()), loc));
        }
        if (out.size() < in.size()) {
            return std::unexpected(gr::Error(std::format("FractionalDelayLine::process: {} outputs for {} samples", out.size(), in.size()), loc));
        }
        if (in.empty()) {
            return {};
        }

        std::uint64_t worst = 0ULL;
        for (const std::uint64_t delay : delayQ32) {
            worst = std::max(worst, delay);
        }
        if ((worst >> kArbitraryFractionBits) > _maxWholeDelay) {
            return std::unexpected(gr::Error(std::format("FractionalDelayLine::process: a delay of {} whole samples past the {} this line holds history for", worst >> kArbitraryFractionBits, _maxWholeDelay), loc));
        }

        const std::size_t n       = in.size();
        const std::size_t armStep = _windowLength + 1UZ;

        // The head of `in` is copied into the tail of the window. A read across the boundary between the history
        // and the caller's span is then still one contiguous run of B+1 samples.
        const std::size_t head = std::min(n, _windowLength);
        std::copy_n(in.begin(), head, _window.begin() + static_cast<std::ptrdiff_t>(_history));

        const float* const       bank    = _bank.data();
        const SampleValue* const windowF = reinterpret_cast<const SampleValue*>(_window.data());
        const SampleValue* const inF     = reinterpret_cast<const SampleValue*>(in.data());
        const std::int64_t       history = static_cast<std::int64_t>(_history);
        const std::int64_t       reach   = static_cast<std::int64_t>(_windowLength) - 1;

        for (std::size_t k = 0UZ; k < n; ++k) {
            // The commanded delay, plus the one whole sample for the reach of the wrap arm.
            const std::int64_t  position = (static_cast<std::int64_t>(k) << kArbitraryFractionBits) - static_cast<std::int64_t>(delayQ32[k]) - static_cast<std::int64_t>(kArbitraryOne);
            const std::int64_t  anchor   = position >> kArbitraryFractionBits; // an arithmetic shift, which floors on both sides of zero
            const std::uint64_t frac     = static_cast<std::uint64_t>(position) & kArbitraryMask;

            const std::uint64_t armPos = frac * _bankSize;
            const std::uint64_t arm    = armPos >> kArbitraryFractionBits;
            const double        mu     = static_cast<double>(armPos & kArbitraryMask) / static_cast<double>(kArbitraryOne);

            const std::int64_t       base = anchor - reach + history;
            const SampleValue* const x    = (base < history) ? windowF + kSampleLanes * static_cast<std::size_t>(base) : inF + kSampleLanes * static_cast<std::size_t>(base - history);

            out[k] = detail::polyphaseArmBlend<T>(bank + (arm + 1UZ) * armStep, x, armStep, mu, _order);
        }

        // The history is the last `_history` input samples. They come from this call alone, or from this call
        // and the samples already held.
        if (n >= _history) {
            std::copy_n(in.end() - static_cast<std::ptrdiff_t>(_history), _history, _window.begin());
        } else {
            std::copy_n(_window.begin() + static_cast<std::ptrdiff_t>(n), _history - n, _window.begin());
            std::copy_n(in.begin(), n, _window.begin() + static_cast<std::ptrdiff_t>(_history - n));
        }
        return {};
    }

private:
    void build(std::span<const float> taps) {
        _prototypeLength      = taps.size();
        detail::ArmBank built = detail::polyphaseArmBank(taps, _bankSize, _order);
        _windowLength         = built.windowLength;
        _bank                 = std::move(built.taps);
    }

    std::size_t        _bankSize;
    int                _order;
    std::size_t        _prototypeLength = 0UZ;
    std::size_t        _windowLength    = 0UZ; ///< `B`
    std::vector<float> _bank;                  ///< arm-major, each arm reversed, adjacent arms adjacent
    std::uint64_t      _maxWholeDelay = 0ULL;
    std::size_t        _history       = 0UZ;
    std::vector<T>     _window; ///< `_history` past samples, then room for the head of the call
};

} // namespace gr::filter

#endif // GNURADIO_FRACTIONAL_DELAY_HPP
