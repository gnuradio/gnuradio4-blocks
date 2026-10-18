#ifndef GNURADIO_ALGORITHM_ANALYSIS_AUTOCORRELATION_HPP
#define GNURADIO_ALGORITHM_ANALYSIS_AUTOCORRELATION_HPP

#include <algorithm>
#include <bit>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <vector>

#include <gnuradio-4.0/MemoryAllocators.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/algorithm/fourier/window.hpp>

/**
 * @brief The windowed, averaged autocorrelation of a stream, with its scatter and its detection threshold.
 *
 * Noise alone exceeds the threshold at some lag in a stated fraction of records. The estimator has two kinds. They
 * share the code and differ only in the sequence they correlate.
 *
 * - The **complex** kind estimates `R(tau) = E[x(t) x*(t - tau)]` on the input samples. The estimate is coherent.
 *   A frequency offset `df` multiplies `R(tau)` by `exp(j 2 pi df tau)` and leaves `|R(tau)|` unchanged. The phase
 *   at a known lag is therefore a frequency estimate, `df = arg R(tau) / (2 pi tau)`. The estimate is unambiguous
 *   while `|df| < 1/(2 tau)`.
 * - The **envelope** kind estimates the autocorrelation of `|x|^2`. Oscillator phase noise does not change it:
 *   `|s(t) exp(j phi(t))| = |s(t)|` for any phase `phi(t)`.
 *
 * Both kinds remove each window's mean by default. Write `y = |x|^2` and `P = E[y]`. Without the removal the
 * published ratio is `[P^2 + C_y(tau)] / [P^2 + Var(y)]`, with `C_y` the autocovariance of `y`. For circular complex
 * Gaussian `x`, `Var(y) = P^2`. Every lag then reads `0.5` plus half the true value. A feature that is a fraction `f`
 * of the fluctuation reads `f/2`. For a constant-modulus signal `Var(y) = 0`, and the ratio is exactly `1` at every
 * lag. With the mean removed, `R(0) = Var(y)`, and every lag correlates the fluctuation alone. The same holds for the
 * complex kind when the input carries a DC offset.
 *
 * The correlation over lags `0..L` is the inverse transform of the magnitude-squared forward transform of one window.
 * The window is zero-padded. The padding keeps circular wraparound out of every published lag.
 *
 * A record averages `K` windows that start `H` samples apart. It covers `N + (K-1)H` distinct samples, and the
 * estimator publishes one every `K*H` samples. With overlap, a feature of at most `N - H` samples lies whole inside at
 * least one window. Overlapping windows share sample products, and a taper weights the products of a window
 * unequally. The scatter and the threshold count both effects.
 */
namespace gr::analysis {

/// @brief The sequence the estimator correlates, the samples themselves or their squared magnitude.
enum class AcfKind : std::uint8_t { Complex, Envelope };

/// @brief Which divisor turns the accumulated products into an estimate of `R(tau)`.
enum class AcfNormalization : std::uint8_t { Biased, Unbiased };

[[nodiscard]] inline constexpr std::string_view acfKindName(AcfKind kind) noexcept { return kind == AcfKind::Complex ? "complex" : "envelope"; }

/// @brief The kind named by a setting's text, `complex` or `envelope`, or no value for any other text.
[[nodiscard]] inline constexpr std::optional<AcfKind> acfKindFrom(std::string_view name) noexcept {
    if (name == "complex") {
        return AcfKind::Complex;
    }
    if (name == "envelope") {
        return AcfKind::Envelope;
    }
    return std::nullopt;
}

[[nodiscard]] inline constexpr std::string_view acfNormalizationName(AcfNormalization normalization) noexcept { return normalization == AcfNormalization::Biased ? "biased" : "unbiased"; }

/// @brief The normalization named by a setting's text, `biased` or `unbiased`, or no value for any other text.
[[nodiscard]] inline constexpr std::optional<AcfNormalization> acfNormalizationFrom(std::string_view name) noexcept {
    if (name == "biased") {
        return AcfNormalization::Biased;
    }
    if (name == "unbiased") {
        return AcfNormalization::Unbiased;
    }
    return std::nullopt;
}

/// @brief The geometry and the estimator choices of one accumulation.
struct AcfConfig {
    std::size_t                 windowLength{}; ///< `N`, samples per window
    std::size_t                 maxLag{};       ///< `L`, the longest published lag, in samples
    AcfKind                     kind           = AcfKind::Envelope;
    AcfNormalization            normalization  = AcfNormalization::Unbiased;
    double                      overlap        = 0.5;  ///< fraction a window shares with the next, in [0, 1)
    std::size_t                 nAverages      = 16UZ; ///< windows per published record
    bool                        removeMean     = true;
    gr::algorithm::window::Type window         = gr::algorithm::window::Type::Rectangular;
    double                      falseAlarmRate = 1e-3; ///< probability per published record that noise alone clears the threshold
};

/**
 * @brief The transform length one window of `n` samples needs to publish lags `0..maxLag` without wraparound.
 *
 * The circular correlation of a length-`M` zero-padded sequence at lag `tau` is the linear correlation at `tau` plus
 * the linear correlation at `tau - M`. The linear correlation of an `n`-sample sequence vanishes outside
 * `|lag| <= n - 1`. No published lag is contaminated exactly when `maxLag - M < -(n - 1)`, that is when
 * `M >= n + maxLag`. The function returns the next power of two at or above that length.
 */
[[nodiscard]] inline std::size_t acfTransformLength(std::size_t n, std::size_t maxLag) noexcept { return std::bit_ceil(n + maxLag); }

/**
 * @brief The standard deviation of a normalized autocorrelation at a lag standing on `nPairs` independent products.
 *
 * For `N` samples of circular complex white noise, the biased estimator at `tau != 0` is a sum of `N - tau`
 * uncorrelated terms. Each term has variance `sigma^4`. This gives `E|R^(tau)|^2 = (N - tau) sigma^4 / N^2` against
 * `R^(0) = sigma^2`. The unbiased estimator divides by the pair count instead and gives `1/sqrt(N - tau)`. Averaging
 * replaces `N - tau` by the effective product count of the record. `|R^|/R^(0)` is Rayleigh with
 * `E|.|^2 = 1/nPairs`. Therefore `P(|R^|/R^(0) > u) = exp(-u^2 nPairs)`.
 */
[[nodiscard]] inline double acfScatter(std::size_t nPairs) noexcept { return nPairs == 0UZ ? std::numeric_limits<double>::infinity() : 1. / std::sqrt(static_cast<double>(nPairs)); }

/**
 * @brief The normalized height a peak must reach for a record of `nLags` searched lags to false-alarm at rate `pFa`.
 *
 * Per lag, `P(|R^|/R^(0) > gamma) = exp(-gamma^2 nPairs)`. Over `nLags` searched lags the union bound gives a
 * per-record rate of `nLags exp(-gamma^2 nPairs)`. Solving for the threshold gives
 * `gamma = sqrt(ln(nLags/pFa) / nPairs)`. That is the scatter times `sqrt(ln(nLags/pFa))`. The factor depends only on
 * the searched-lag count and the stated rate.
 */
[[nodiscard]] inline double acfThreshold(std::size_t nPairs, std::size_t nLags, double pFa) noexcept {
    if (nPairs == 0UZ || nLags == 0UZ || !(pFa > 0.) || !(pFa < 1.)) {
        return std::numeric_limits<double>::infinity();
    }
    return std::sqrt(std::log(static_cast<double>(nLags) / pFa) / static_cast<double>(nPairs));
}

/// @brief The standard normal deviate `t` with `P(|z| > t) = tail`, by bisection on the complementary error function.
/// One hundred halvings of `[0, 40]` reach the resolution of a double.
[[nodiscard]] inline double acfNormalDeviate(double tail) noexcept {
    if (!(tail > 0.) || !(tail < 1.)) {
        return std::numeric_limits<double>::infinity();
    }
    double low  = 0.;
    double high = 40.; // erfc(40/sqrt2) underflows to zero, below every positive tail
    for (std::size_t step = 0UZ; step < 100UZ; ++step) {
        const double middle = 0.5 * (low + high);
        if (std::erfc(middle / std::numbers::sqrt2) > tail) {
            low = middle;
        } else {
            high = middle;
        }
    }
    return 0.5 * (low + high);
}

/**
 * @brief The same two thresholds for an estimate that is real instead of circular complex.
 *
 * The envelope kind correlates `|x|^2`. The complex kind of a real input correlates real samples. In both cases the
 * transform's magnitude squared is real and even. `R(tau)` is then real, and `|R(tau)|/R(0)` is a folded normal and
 * not a Rayleigh. Its tail is `erfc(u sqrt(nPairs/2))` and not `exp(-u^2 nPairs)`. That tail is far heavier at the
 * same height. At `nLags = 1024` and `pFa = 1e-3` the Rayleigh threshold gives a per-lag rate of `2e-4` on a real
 * estimate, against a design rate of `9.8e-7`. The per-record rate is then about 0.2. A real estimate therefore
 * takes `u = acfNormalDeviate(pFa/nLags) / sqrt(nPairs)`.
 */
[[nodiscard]] inline double acfRealThreshold(std::size_t nPairs, std::size_t nLags, double pFa) noexcept {
    if (nPairs == 0UZ || nLags == 0UZ || !(pFa > 0.) || !(pFa < 1.)) {
        return std::numeric_limits<double>::infinity();
    }
    return acfNormalDeviate(pFa / static_cast<double>(nLags)) / std::sqrt(static_cast<double>(nPairs));
}

/// @brief The probability that noise alone lifts a lag of scatter @p scatter above @p height. A lag of zero scatter
/// publishes zero and never clears a positive height.
[[nodiscard]] inline double acfLagTail(double height, double scatter, bool real) noexcept {
    if (!(scatter > 0.)) {
        return 0.;
    }
    const double ratio = height / scatter;
    return real ? std::erfc(ratio / std::numbers::sqrt2) : std::exp(-ratio * ratio);
}

/**
 * @brief The height that noise alone exceeds at any of the given lags in a fraction @p pFa of records.
 *
 * `lagScatter` holds the scatter of each searched lag. Over the lags the union bound gives a per-record rate of
 * `sum acfLagTail(gamma, scatter, real)`. The function finds the `gamma` that makes the sum `pFa`, by bisection
 * between the threshold of the noisiest lag alone and that lag's threshold over every lag. For lags of equal scatter
 * the result is `acfThreshold` or `acfRealThreshold`.
 */
[[nodiscard]] inline double acfRecordThreshold(std::span<const double> lagScatter, double pFa, bool real) noexcept {
    double largest = 0.;
    for (const double scatter : lagScatter) {
        largest = std::max(largest, scatter);
    }
    if (!(largest > 0.) || !std::isfinite(largest) || !(pFa > 0.) || !(pFa < 1.)) {
        return std::numeric_limits<double>::infinity();
    }
    const auto lags = static_cast<double>(lagScatter.size());
    double     low  = largest * (real ? acfNormalDeviate(pFa) : std::sqrt(-std::log(pFa)));
    double     high = largest * (real ? acfNormalDeviate(pFa / lags) : std::sqrt(std::log(lags / pFa)));
    const auto rate = [&](double height) {
        double sum = 0.;
        for (const double scatter : lagScatter) {
            sum += acfLagTail(height, scatter, real);
        }
        return sum;
    };
    for (std::size_t step = 0UZ; step < 60UZ; ++step) {
        const double middle = 0.5 * (low + high);
        if (rate(middle) > pFa) {
            low = middle;
        } else {
            high = middle;
        }
    }
    return high;
}

/**
 * @brief A height the median of a noise record falls below in at most a fraction @p tail of records.
 *
 * The record holds lag 0 at one and the `L` lags of `lagScatter`. Its median is the value at index `floor((L+1)/2)`
 * of the `L + 1` sorted values. The median lies below `x` when at least `a = floor((L+1)/2) + 1` lags lie below `x`.
 * The noise model takes the lags as independent. Their count below `x` is then a sum of independent trials of mean
 * probability `p(x)`, and the Chernoff bound gives `P(count >= a) <= exp(-L D(a/L, p(x)))`. `D` is the divergence of
 * two Bernoulli trials, `D(q, p) = q ln(q/p) + (1-q) ln((1-q)/(1-p))`. The function returns the height at which that
 * bound is @p tail, by bisection. A lag of zero scatter lies below every positive height. The function returns zero
 * when the bound exceeds @p tail at every positive height.
 */
[[nodiscard]] inline double acfMedianLowerBound(std::span<const double> lagScatter, double tail, bool real) noexcept {
    double largest = 0.;
    for (const double scatter : lagScatter) {
        largest = std::max(largest, scatter);
    }
    if (!(largest > 0.) || !std::isfinite(largest) || !(tail > 0.) || !(tail < 1.)) {
        return 0.;
    }
    const auto        lags   = static_cast<double>(lagScatter.size());
    const std::size_t needed = (lagScatter.size() + 1UZ) / 2UZ + 1UZ;
    if (needed > lagScatter.size()) {
        return 1.; // a single lag: the median is lag 0 or a lag above it
    }
    const double share     = static_cast<double>(needed) / lags;
    const auto   term      = [](double q, double p) { return q > 0. ? q * std::log(q / p) : 0.; };
    const auto   exceedsAt = [&](double height) {
        double below = 0.;
        for (const double scatter : lagScatter) {
            below += 1. - acfLagTail(height, scatter, real);
        }
        const double p = below / lags;
        if (p >= share) {
            return true;
        }
        return p > 0. && std::exp(-lags * (term(share, p) + term(1. - share, 1. - p))) > tail;
    };
    double low  = 0.;
    double high = 10. * largest; // a lag lies below ten scatters with a probability within 1e-22 of one
    if (exceedsAt(std::numeric_limits<double>::min())) {
        return 0.;
    }
    for (std::size_t step = 0UZ; step < 60UZ; ++step) {
        const double middle = 0.5 * (low + high);
        if (exceedsAt(middle)) {
            high = middle;
        } else {
            low = middle;
        }
    }
    return low;
}

/**
 * @brief The margin in decibels over a noise record's own median that noise alone exceeds at some lag in at most a
 * fraction @p pFa of records.
 *
 * A detector that refers a margin to the record's median reads a median that scatters. The margin therefore splits
 * the rate in two halves. `high` is `acfRecordThreshold` at `pFa/2`, and `low` is `acfMedianLowerBound` at `pFa/2`.
 * The margin is `10 log10(high/low)`. In a record whose median lies at or above `low`, a lag clears the margin only
 * above `high`. The two halves add to at most `pFa`. The zero lag before normalization scales every lag and the median
 * alike, and it cancels in the comparison.
 */
[[nodiscard]] inline double acfPeakThresholdDb(std::span<const double> lagScatter, double pFa, bool real) noexcept {
    if (!(pFa > 0.) || !(pFa < 1.)) {
        return std::numeric_limits<double>::infinity();
    }
    const double high = acfRecordThreshold(lagScatter, 0.5 * pFa, real);
    const double low  = acfMedianLowerBound(lagScatter, 0.5 * pFa, real);
    return low > 0. && std::isfinite(high) ? 10. * std::log10(high / low) : std::numeric_limits<double>::infinity();
}

/**
 * @brief The window grid an analysis estimator segments a stream on.
 *
 * The grid holds the geometry, which is `windowLength`, `overlap` and the hop they imply. It holds the taper and the
 * taper's own autocorrelation `r_w`. It counts the windows behind the accumulation in progress. The grid holds no
 * sample buffer. The caller presents a span and gets back how many leading samples the grid has finished with. The
 * caller presents the remainder again, with more samples after it. The grid is anchored at the stream start. The same
 * input split at different points gives the same windows.
 */
template<typename T>
struct WindowedSegmenter {
    std::size_t                 windowLength = 0UZ;
    std::size_t                 maxLag       = 0UZ;
    double                      overlap      = 0.;
    std::size_t                 hop          = 1UZ;
    gr::algorithm::window::Type windowType   = gr::algorithm::window::Type::Rectangular;

    std::vector<float>  taper{};    ///< `w[0..windowLength-1]`
    std::vector<double> taperAcf{}; ///< `r_w(tau) = sum_n w[n] w[n+tau]` for `tau = 0..maxLag`

    std::uint64_t streamAt     = 0ULL; ///< absolute index of the next presented span's first sample
    std::size_t   windows      = 0UZ;  ///< windows in the accumulation in progress
    std::uint64_t groupStartAt = 0ULL; ///< absolute index of that accumulation's first window's first sample
    std::uint64_t coveredUntil = 0ULL; ///< absolute index one past the last sample a window covered

    /// @brief The hop an overlap implies, at least one sample.
    [[nodiscard]] static std::size_t hopFor(std::size_t length, double fraction) noexcept { return std::max(1UZ, static_cast<std::size_t>(std::llround(static_cast<double>(length) * (1. - fraction)))); }

    /**
     * @brief Adopt a geometry and a taper. `taperAcf` holds `lag + 1` zeros for the caller to fill.
     * @throws gr::exception with the window's message when the taper cannot be built at @p length.
     */
    void prepare(std::size_t length, std::size_t lag, double fraction, gr::algorithm::window::Type type) {
        windowLength = length;
        maxLag       = lag;
        overlap      = fraction;
        windowType   = type;
        hop          = hopFor(length, fraction);
        try {
            taper = gr::algorithm::window::create<float>(type, length);
        } catch (const std::invalid_argument& refusal) {
            throw gr::exception(refusal.what());
        }
        taperAcf.assign(lag + 1UZ, 0.);
        reset();
    }

    /// @brief Drop the accumulation in progress, keeping the stream position.
    void restart() noexcept { windows = 0UZ; }

    /// @brief Drop everything, the stream position included.
    void reset() noexcept {
        streamAt     = 0ULL;
        groupStartAt = 0ULL;
        coveredUntil = 0ULL;
        restart();
    }

    /// @brief State where the next presented span's first sample sits in the stream. No window covers a sample from
    /// there on.
    void seek(std::uint64_t absolute) noexcept {
        streamAt     = absolute;
        groupStartAt = absolute;
        coveredUntil = absolute;
    }

    /// @brief Take one window into the accumulation, remembering where the group started.
    void noteWindow(std::uint64_t absoluteStart) noexcept {
        if (windows == 0UZ) {
            groupStartAt = absoluteStart;
        }
        ++windows;
    }

    /// @brief Distinct input samples behind the accumulation in progress, `N + (K-1)H`.
    [[nodiscard]] std::uint64_t distinctSamples() const noexcept { return windows == 0UZ ? 0ULL : std::uint64_t{windowLength} + std::uint64_t{windows - 1UZ} * std::uint64_t{hop}; }

    /**
     * @brief Present @p in to the grid, calling @p onWindow with each whole window that lies inside it.
     * @return how many leading samples the grid has finished with. The caller presents the remainder again.
     */
    template<typename Fn>
    std::size_t forEachWindow(std::span<const T> in, Fn&& onWindow) {
        std::size_t consumed = 0UZ;
        if (windowLength == 0UZ) {
            return 0UZ;
        }
        while (consumed + windowLength <= in.size()) {
            const std::uint64_t start = streamAt + static_cast<std::uint64_t>(consumed);
            coveredUntil              = start + static_cast<std::uint64_t>(windowLength);
            onWindow(in.subspan(consumed, windowLength), start);
            consumed += hop;
        }
        streamAt += static_cast<std::uint64_t>(consumed);
        return consumed;
    }
};

/// @brief One published record, the normalized correlation over lags `0..L` with the statistics that describe it.
struct AcfResult {
    std::span<const float> magnitude;          ///< `|R(tau)|/R(0)` for lags `0..L`, with `magnitude[0]` exactly 1 and zero at a lag of zero scatter
    std::span<const float> phase;              ///< `arg R(tau)` in radians, zero at a lag of zero scatter, empty for the envelope kind
    double                 zeroLag{};          ///< `R(0)` of the correlated sequence before normalization
    std::uint64_t          sampleStart{};      ///< absolute index of the first averaged window's first sample
    std::size_t            nAveraged{};        ///< windows behind this record
    std::uint64_t          nDistinctSamples{}; ///< `N + (nAveraged-1) H`
    double                 nPairs{};           ///< effective independent products behind lag `scatterLag`
    std::size_t            scatterLag{};       ///< the lag with the largest scatter
    double                 scatter{};          ///< the scatter of lag `scatterLag`
    double                 threshold{};        ///< the height noise alone exceeds at any lag `1..L` in a fraction `falseAlarmRate` of records, infinite under the envelope kind without mean removal
    double                 peakThresholdDb{};  ///< the margin in decibels over a record's own median that noise alone clears at some lag in at most a fraction `falseAlarmRate` of records, infinite under the envelope kind without mean removal
    bool                   realValued{};       ///< the estimate is real, with a folded normal tail in place of a Rayleigh tail
};

/**
 * @brief The windowed, averaged autocorrelation of a stream, without a scheduler.
 *
 * A caller builds the estimator with `prepare()` and presents samples to `process()`, which passes each completed
 * `AcfResult` to a sink. `process()` holds no sample buffer. It consumes each whole hop whose window lies inside the
 * given span and returns how many leading samples it has finished with. The caller keeps the unused tail and presents
 * it again with more samples. `flush()` publishes the accumulation in progress, at the end of a stream or before a
 * `seek()`. `setAveraging()` and `setFalseAlarmRate()` change those two values without a new `prepare()`.
 *
 * Averaging is coherent over the complex estimate, and the magnitude is taken once at the end. The transform runs in
 * double precision. A window whose energy after mean removal is at most `kDegenerateFluctuation` times its power is
 * degenerate and adds to no record. Under the envelope kind a constant-modulus window is degenerate. A window holding
 * a NaN or an infinity adds to no record either. `nDegenerate` and `nNonFinite` count the two.
 *
 * The noise model of a record covers white noise. Mean removal gives lag `tau` of a noise window the mean
 * `-r_w(tau) / (r_w(0) (N - 1))` of its zero lag. That is `-1/(N-1)` of the zero lag under the unbiased normalization.
 * The estimator adds that share of the zero lag back to each lag of each window. The lags of a noise record then have
 * zero mean. On a signal the added share moves a normalized lag by at most `1/(N-1)`. Under the envelope kind
 * without mean removal every noise lag carries the squared mean of the envelope, and the threshold and the decibel
 * margin are infinite.
 *
 * The model takes a product of two samples at lag `tau` as uncorrelated with every other product. Window `k` gives
 * the product at stream index `t` the weight `w[t - kH] w[t - kH + tau]`. Let `c(t)` be the sum of those weights over
 * the record's windows. The scatter of lag `tau` is then `sqrt(sum c^2) / (K r_w(tau))` under the unbiased
 * normalization and `sqrt(sum c^2) / (K r_w(0))` under the biased one. The effective product count is
 * `(sum c)^2 / sum c^2`. The sum of `c^2` is formed from the taper's products `w[n] w[n + mH]` for each window shift
 * `mH`, through the transform pair. A lag whose `sum c^2` lies at or below the transform's resolution,
 * `kModelResolution log2(M)` times the zero lag's, takes zero scatter and publishes zero. So does an unbiased lag with
 * no taper overlap. A taper with zero ends leaves such lags near `N`. The threshold holds the stated rate over every
 * lag `1..L` together. It takes the tail of a Gaussian estimate. A lag on few effective products has a heavier tail: an
 * envelope estimate on a few thousand products or fewer, and a complex estimate on about a hundred or fewer, as at the
 * longest lags under the unbiased normalization. Noise then clears the threshold more often than stated, and on an
 * envelope estimate the decibel margin too.
 */
template<typename T>
requires(std::is_same_v<T, float> || std::is_same_v<T, std::complex<float>>)
struct Autocorrelation {
    using Scalar    = double;
    using Transform = std::complex<Scalar>;
    template<typename U>
    using AlignedVector = std::vector<U, gr::allocator::Aligned<U>>;

    /// The fluctuation power, relative to the sequence's own power, at or below which a window is degenerate. A
    /// constant-modulus waveform stored in single precision fluctuates by a few units in the last place. That is a
    /// relative power of about `1e-14`.
    static constexpr double kDegenerateFluctuation = 1e-12;

    /// A thousand times the rounding of a double. The transform pair rounds a correlation by about `eps log2(M)` of
    /// its zero lag. A modeled `sum c^2` above `kModelResolution log2(M)` times its zero lag is therefore accurate to
    /// about one part in a thousand.
    static constexpr double kModelResolution = 1e3 * std::numeric_limits<double>::epsilon();

    std::uint64_t nWindows{};        ///< windows the grid produced, degenerate and non-finite ones included
    std::uint64_t nRecords{};        ///< records of exactly `nAverages` windows
    std::uint64_t nSamples{};        ///< input samples the estimator is done with
    std::uint64_t nDegenerate{};     ///< windows with no fluctuation to correlate
    std::uint64_t nNonFinite{};      ///< windows holding a NaN or an infinity
    std::uint64_t nPartialFlushes{}; ///< records published with fewer than `nAverages` windows
    std::uint64_t nPartialWindows{}; ///< windows those partial records were made of
    std::uint64_t nLongRecords{};    ///< records of more than `nAverages` windows, published after `nAverages` fell below the windows pending

    AcfConfig            _config{};
    WindowedSegmenter<T> _segmenter{};
    std::size_t          _transformLength = 0UZ;

    gr::algorithm::FFT<Transform, Transform, gr::algorithm::Direction::Forward>  _forward{};
    gr::algorithm::FFT<Transform, Transform, gr::algorithm::Direction::Backward> _backward{};

    AlignedVector<Transform> _work{};     ///< one window, tapered and zero-padded, then the transform's output
    AlignedVector<Transform> _spectrum{}; ///< the forward transform's output, then its magnitude squared
    std::vector<Transform>   _accumulator{};
    std::vector<float>       _magnitude{};
    std::vector<float>       _phase{};
    std::vector<double>      _lagScatter{}; ///< the modeled scatter of each lag `0..L` for `_modelWindows` windows

    std::size_t _modelWindows     = 0UZ; ///< windows the scatter curve holds for, zero for none
    double      _modelRate        = 0.;  ///< false-alarm rate the threshold holds for, zero for none
    double      _modelThreshold   = 0.;
    double      _modelThresholdDb = 0.;
    double      _modelScatter     = 0.;
    double      _modelPairs       = 0.;
    std::size_t _modelLag         = 0UZ;

    /**
     * @brief Adopt a configuration, sizing every buffer and building the taper and its own autocorrelation.
     * @throws gr::exception naming the setting and the bound it broke.
     */
    void prepare(const AcfConfig& config) {
        if (config.windowLength < 16UZ || config.windowLength > 1048576UZ) {
            throw gr::exception(std::format("gr::analysis::Autocorrelation: windowLength must lie in [16, 1048576], got {}", config.windowLength));
        }
        if (config.maxLag == 0UZ) {
            throw gr::exception("gr::analysis::Autocorrelation: maxLag is the longest published lag and must be at least 1");
        }
        if (config.maxLag >= config.windowLength) {
            throw gr::exception(std::format("gr::analysis::Autocorrelation: maxLag ({}) must be below windowLength ({})", config.maxLag, config.windowLength));
        }
        if (config.nAverages == 0UZ) {
            throw gr::exception("gr::analysis::Autocorrelation: nAverages counts the windows a record is made of and must be at least 1");
        }
        if (!(config.overlap >= 0.) || !(config.overlap < 1.)) {
            throw gr::exception(std::format("gr::analysis::Autocorrelation: overlap is the fraction a window shares with the next and must lie in [0, 1), got {}", config.overlap));
        }
        if (!(config.falseAlarmRate > 0.) || !(config.falseAlarmRate < 1.)) {
            throw gr::exception(std::format("gr::analysis::Autocorrelation: falseAlarmRate is a probability per record and must lie in (0, 1), got {}", config.falseAlarmRate));
        }

        _config          = config;
        _transformLength = acfTransformLength(config.windowLength, config.maxLag);
        _segmenter.prepare(config.windowLength, config.maxLag, config.overlap, config.window);

        _forward  = gr::algorithm::FFT<Transform, Transform, gr::algorithm::Direction::Forward>{};
        _backward = gr::algorithm::FFT<Transform, Transform, gr::algorithm::Direction::Backward>{};
        _work.assign(_transformLength, Transform{});
        _spectrum.assign(_transformLength, Transform{});
        _accumulator.assign(config.maxLag + 1UZ, Transform{});
        _magnitude.assign(config.maxLag + 1UZ, 0.f);
        _phase.assign(config.maxLag + 1UZ, 0.f);
        _lagScatter.assign(config.maxLag + 1UZ, 0.);
        _modelWindows = 0UZ;
        _modelRate    = 0.;

        buildTaperAutocorrelation();
        reset();
    }

    /// @brief Return to the state a fresh stream starts from, counters included.
    void reset() {
        _segmenter.reset();
        std::ranges::fill(_accumulator, Transform{});
        nWindows = nRecords = nSamples = nDegenerate = nNonFinite = nPartialFlushes = nPartialWindows = nLongRecords = 0ULL;
    }

    /// @brief Drop the accumulation in progress, keeping the stream position and the counters.
    void restart() {
        _segmenter.restart();
        std::ranges::fill(_accumulator, Transform{});
    }

    /// @brief Forward a stream position to the grid. A record's start index is then absolute.
    void seek(std::uint64_t absolute) noexcept { _segmenter.seek(absolute); }

    /// @brief Set the windows per record. The accumulation in progress and the window grid are kept. When the
    /// accumulation already holds @p nAverages windows or more, `process()` publishes it with every window it holds at
    /// the next window it folds in, and `nLongRecords` counts that record. `flush()` publishes it too when called
    /// before that window.
    void setAveraging(std::size_t nAverages) {
        if (nAverages == 0UZ) {
            throw gr::exception("gr::analysis::Autocorrelation: nAverages counts the windows a record is made of and must be at least 1");
        }
        _config.nAverages = nAverages;
    }

    /// @brief Set the per-record false-alarm rate the published threshold is computed for.
    void setFalseAlarmRate(double pFa) {
        if (!(pFa > 0.) || !(pFa < 1.)) {
            throw gr::exception(std::format("gr::analysis::Autocorrelation: falseAlarmRate is a probability per record and must lie in (0, 1), got {}", pFa));
        }
        _config.falseAlarmRate = pFa;
    }

    /// @brief True when the correlated sequence is real, under the envelope kind or for a real input. The thresholds
    /// then use the folded normal tail.
    [[nodiscard]] bool                    realValued() const noexcept { return _config.kind == AcfKind::Envelope || std::is_same_v<T, float>; }
    [[nodiscard]] std::uint64_t           streamAt() const noexcept { return _segmenter.streamAt; }
    [[nodiscard]] std::uint64_t           coveredUntil() const noexcept { return _segmenter.coveredUntil; }
    [[nodiscard]] std::span<const double> lagScatter() const noexcept { return std::span<const double>(_lagScatter); }
    [[nodiscard]] bool                    configured() const noexcept { return _segmenter.windowLength != 0UZ; }
    [[nodiscard]] std::size_t             hop() const noexcept { return _segmenter.hop; }
    [[nodiscard]] std::size_t             transformLength() const noexcept { return _transformLength; }
    [[nodiscard]] const AcfConfig&        config() const noexcept { return _config; }
    [[nodiscard]] std::size_t             windowsPending() const noexcept { return _segmenter.windows; }
    [[nodiscard]] std::span<const double> taperAutocorrelation() const noexcept { return std::span<const double>(_segmenter.taperAcf); }

    /**
     * @brief Estimate over every whole window inside @p in, reporting each completed record to @p onResult.
     * @return how many leading samples the estimator has finished with. The caller presents the remainder again.
     */
    template<typename Sink>
    std::size_t process(std::span<const T> in, Sink&& onResult) {
        if (!configured()) {
            return 0UZ;
        }
        const std::size_t consumed = _segmenter.forEachWindow(in, [&](std::span<const T> window, std::uint64_t at) {
            ++nWindows;
            if (const WindowFold fold = accumulate(window); fold != WindowFold::Accumulated) {
                ++(fold == WindowFold::NonFinite ? nNonFinite : nDegenerate);
                return;
            }
            _segmenter.noteWindow(at);
            if (_segmenter.windows >= _config.nAverages) {
                ++(_segmenter.windows == _config.nAverages ? nRecords : nLongRecords);
                publish(onResult);
                restart();
            }
        });
        nSamples += consumed;
        return consumed;
    }

    /**
     * @brief Publish the accumulation in progress as a record marked with the windows it holds. A record of fewer than
     * `nAverages` windows counts in `nPartialFlushes`, one of exactly `nAverages` in `nRecords`, and a longer one in
     * `nLongRecords`.
     * @return whether a record was published. An accumulation with no complete window publishes nothing.
     */
    template<typename Sink>
    bool flush(Sink&& onResult) {
        if (_segmenter.windows == 0UZ) {
            return false;
        }
        if (_segmenter.windows < _config.nAverages) {
            nPartialWindows += _segmenter.windows;
            ++nPartialFlushes;
        } else {
            ++(_segmenter.windows == _config.nAverages ? nRecords : nLongRecords);
        }
        publish(onResult);
        restart();
        return true;
    }

private:
    enum class WindowFold : std::uint8_t { Accumulated, Degenerate, NonFinite };

    /// @brief The taper's own autocorrelation, through the same transform pair the estimate runs on.
    void buildTaperAutocorrelation() {
        std::ranges::fill(_work, Transform{});
        for (std::size_t n = 0UZ; n < _segmenter.windowLength; ++n) {
            _work[n] = Transform(static_cast<Scalar>(_segmenter.taper[n]), Scalar{0});
        }
        correlate();
        const Scalar inverse = Scalar{1} / static_cast<Scalar>(_transformLength);
        for (std::size_t lag = 0UZ; lag <= _config.maxLag; ++lag) {
            _segmenter.taperAcf[lag] = _work[lag].real() * inverse;
        }
    }

    /// @brief `_work` holds a zero-padded sequence on entry and `transformLength` times its circular autocorrelation
    /// on exit. The transform pair is unnormalized.
    void correlate() {
        _forward.compute(_work, _spectrum);
        for (Transform& bin : _spectrum) {
            bin = Transform(bin.real() * bin.real() + bin.imag() * bin.imag(), Scalar{0});
        }
        _backward.compute(_spectrum, _work);
    }

    /// @brief Fold one window into the accumulator, unless it holds a non-finite sample or no fluctuation.
    [[nodiscard]] WindowFold accumulate(std::span<const T> window) {
        const std::size_t n = _segmenter.windowLength;
        std::ranges::fill(_work, Transform{});

        Transform mean{};
        Scalar    scale = Scalar{0};
        for (std::size_t k = 0UZ; k < n; ++k) {
            const Transform value = sequence(window[k]);
            _work[k]              = value;
            mean += value;
            scale += std::norm(value);
        }
        if (!std::isfinite(scale)) {
            return WindowFold::NonFinite;
        }
        mean /= static_cast<Scalar>(n);
        if (!_config.removeMean) {
            mean = Transform{};
        }

        Scalar energy = Scalar{0};
        for (std::size_t k = 0UZ; k < n; ++k) {
            const Transform centered = _work[k] - mean;
            energy += std::norm(centered);
            _work[k] = centered * static_cast<Scalar>(_segmenter.taper[k]);
        }
        if (!(scale > Scalar{0}) || !(energy > kDegenerateFluctuation * scale)) {
            return WindowFold::Degenerate;
        }

        correlate();
        const Scalar inverse = Scalar{1} / static_cast<Scalar>(_transformLength);
        // Mean removal gives lag `tau` of a white-noise window the mean `-r_w(tau) / (r_w(0) (N - 1))` of its zero
        // lag. `restore` is that share per unit of `r_w(tau)`.
        Scalar       restore  = Scalar{0};
        const Scalar taperAt0 = _segmenter.taperAcf[0UZ];
        if (_config.removeMean && taperAt0 > Scalar{0}) {
            restore = _work[0UZ].real() * inverse / (taperAt0 * static_cast<Scalar>(n - 1UZ));
        }
        _accumulator[0UZ] += _work[0UZ] * inverse;
        for (std::size_t lag = 1UZ; lag <= _config.maxLag; ++lag) {
            _accumulator[lag] += _work[lag] * inverse + restore * _segmenter.taperAcf[lag];
        }
        return WindowFold::Accumulated;
    }

    /// @brief The sequence element one input sample contributes, promoted to the transform's own type.
    [[nodiscard]] Transform sequence(const T& sample) const noexcept {
        if constexpr (std::is_same_v<T, float>) {
            const Scalar value = static_cast<Scalar>(sample);
            return _config.kind == AcfKind::Envelope ? Transform(value * value, Scalar{0}) : Transform(value, Scalar{0});
        } else {
            const Scalar re = static_cast<Scalar>(sample.real());
            const Scalar im = static_cast<Scalar>(sample.imag());
            return _config.kind == AcfKind::Envelope ? Transform(re * re + im * im, Scalar{0}) : Transform(re, im);
        }
    }

    /// @brief Fill `_lagScatter` with the scatter of each lag of a record of @p windows windows.
    void modelScatter(std::size_t windows) {
        const std::size_t n     = _segmenter.windowLength;
        const std::size_t hop   = _segmenter.hop;
        const std::size_t lags  = _config.maxLag;
        const Scalar      scale = Scalar{1} / static_cast<Scalar>(_transformLength);
        std::ranges::fill(_lagScatter, 0.);
        // `sum c^2` at lag `tau` is `K G(0) + 2 sum_m (K - m) G(mH)`, with `G(d)` the correlation at lag `tau` of
        // the taper products `w[n] w[n + d]`. `G(d)` vanishes at every lag from 1 once `d + 1 >= N`.
        for (std::size_t shift = 0UZ, m = 0UZ; m < windows && shift + 1UZ < n; ++m, shift += hop) {
            std::ranges::fill(_work, Transform{});
            for (std::size_t k = 0UZ; k + shift < n; ++k) {
                _work[k] = Transform(static_cast<Scalar>(_segmenter.taper[k]) * static_cast<Scalar>(_segmenter.taper[k + shift]), Scalar{0});
            }
            correlate();
            const Scalar weight = m == 0UZ ? static_cast<Scalar>(windows) : Scalar{2} * static_cast<Scalar>(windows - m);
            for (std::size_t lag = 0UZ; lag <= lags; ++lag) {
                _lagScatter[lag] += weight * _work[lag].real() * scale;
            }
        }
        const Scalar resolution = kModelResolution * std::log2(static_cast<Scalar>(_transformLength)) * _lagScatter[0UZ];
        const auto   count      = static_cast<Scalar>(windows);
        for (std::size_t lag = 0UZ; lag <= lags; ++lag) {
            const Scalar divisor = count * (_config.normalization == AcfNormalization::Biased ? _segmenter.taperAcf[0UZ] : _segmenter.taperAcf[lag]);
            _lagScatter[lag]     = lag > 0UZ && divisor > Scalar{0} && _lagScatter[lag] > resolution ? std::sqrt(_lagScatter[lag]) / divisor : Scalar{0};
        }
    }

    /// @brief Bring the scatter curve and the threshold to a record of @p windows windows at the configured rate.
    void model(std::size_t windows) {
        if (windows != _modelWindows) {
            modelScatter(windows);
            _modelWindows       = windows;
            _modelRate          = 0.;
            const auto noisiest = std::ranges::max_element(_lagScatter);
            _modelLag           = static_cast<std::size_t>(std::distance(_lagScatter.begin(), noisiest));
            _modelScatter       = *noisiest;
            const Scalar norm   = _config.normalization == AcfNormalization::Biased ? _segmenter.taperAcf[0UZ] : _segmenter.taperAcf[_modelLag];
            const Scalar ratio  = norm > Scalar{0} ? _segmenter.taperAcf[_modelLag] / norm : Scalar{0};
            _modelPairs         = _modelScatter > Scalar{0} ? ratio * ratio / (_modelScatter * _modelScatter) : Scalar{0};
        }
        if (_config.falseAlarmRate != _modelRate) {
            const bool                    real   = realValued();
            const std::span<const double> search = std::span<const double>(_lagScatter).subspan(1UZ);
            if (_config.kind == AcfKind::Envelope && !_config.removeMean) {
                // a noise envelope with its mean kept carries the squared mean at every lag
                _modelThreshold   = std::numeric_limits<double>::infinity();
                _modelThresholdDb = std::numeric_limits<double>::infinity();
            } else {
                _modelThreshold   = acfRecordThreshold(search, _config.falseAlarmRate, real);
                _modelThresholdDb = acfPeakThresholdDb(search, _config.falseAlarmRate, real);
            }
            _modelRate = _config.falseAlarmRate;
        }
    }

    /// @brief Normalize the accumulation and hand it to the sink.
    template<typename Sink>
    void publish(Sink&& onResult) {
        const std::size_t averaged = _segmenter.windows;
        const std::size_t lags     = _config.maxLag;
        const Scalar      zeroLag  = std::abs(_accumulator[0UZ]);
        const Scalar      taperAt0 = _segmenter.taperAcf[0UZ];

        model(averaged);
        for (std::size_t lag = 0UZ; lag <= lags; ++lag) {
            Scalar ratio = Scalar{0};
            Scalar phase = Scalar{0};
            if (zeroLag > Scalar{0} && (lag == 0UZ || _lagScatter[lag] > Scalar{0})) {
                ratio = std::abs(_accumulator[lag]) / zeroLag;
                if (_config.normalization == AcfNormalization::Unbiased) {
                    ratio *= taperAt0 / _segmenter.taperAcf[lag];
                }
                phase = std::arg(_accumulator[lag]);
            }
            _magnitude[lag] = static_cast<float>(ratio);
            _phase[lag]     = static_cast<float>(phase);
        }

        const AcfResult result{
            .magnitude        = std::span<const float>(_magnitude),
            .phase            = _config.kind == AcfKind::Complex ? std::span<const float>(_phase) : std::span<const float>{},
            .zeroLag          = zeroLag / (static_cast<Scalar>(averaged) * taperAt0),
            .sampleStart      = _segmenter.groupStartAt,
            .nAveraged        = averaged,
            .nDistinctSamples = _segmenter.distinctSamples(),
            .nPairs           = _modelPairs,
            .scatterLag       = _modelLag,
            .scatter          = _modelScatter,
            .threshold        = _modelThreshold,
            .peakThresholdDb  = _modelThresholdDb,
            .realValued       = realValued(),
        };
        onResult(result);
    }
};

} // namespace gr::analysis

#endif // GNURADIO_ALGORITHM_ANALYSIS_AUTOCORRELATION_HPP
