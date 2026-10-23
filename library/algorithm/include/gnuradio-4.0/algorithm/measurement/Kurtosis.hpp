#ifndef GNURADIO_ALGORITHM_KURTOSIS_HPP
#define GNURADIO_ALGORITHM_KURTOSIS_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <limits>
#include <source_location>
#include <span>
#include <vector>

#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/algorithm/math/SampleType.hpp>

/**
 * @brief The normalized fourth moment of a stream and of a run of spectra, with its bias and spread in closed form.
 *
 * A caller adds samples to `KurtosisAccumulator<T>` and reads `normalized()`, `excess()` and `meanPower()` at a
 * window's close. `SpectralKurtosisAccumulator` takes a run of spectra through `accumulate()`, and `evaluate()` writes
 * the per-bin `SK`. `kurtosisBias()` and `kurtosisSpread()` give the plug-in estimate's bias and spread over `n`
 * samples.
 *
 * Kurtosis is the normalized fourth moment of a stream. Spectral kurtosis is the same statistic per frequency bin
 * across a run of spectra. Both use the normalization below.
 *
 * **The normalization.** For a stream `x` with `m2 = mean|x|^2` and `m4 = mean|x|^4`,
 *
 * ```
 * K = m4 / m2^2
 * ```
 *
 * `K` is dimensionless and does not depend on scale. Replacing `x` by `a*x` multiplies both terms by `|a|^4`. The
 * **excess** is `K` less the value for a zero-mean Gaussian of the same domain. A zero-mean Gaussian reads an excess
 * of zero. The Gaussian value is 3 for real input and 2 for complex input. The moments are taken about zero. A
 * Gaussian with a nonzero mean reads a negative excess.
 *
 * - Real. For `x ~ N(0, s^2)`, `E[x^2] = s^2` and `E[x^4] = 3 s^4`. Then `K = 3`.
 * - Complex, circular. Take `z = u + jv` with `u, v ~ N(0, s^2/2)` independent, so that `E|z|^2 = s^2`. Then
 *   `|z|^2` is exponential with mean `s^2`. The second moment of an exponential is twice its mean squared. Then
 *   `E|z|^4 = 2 s^4` and `K = 2`. The value is 2 and not 3 because `|z|^2` is a sum of two squared normals.
 * - Constant modulus. `|z| = A` gives `K = 1` and an excess of `-1`.
 * - Two equal complex tones. `|z|^2 = 2 + 2*cos(D)` with `D` uniform over a long window gives `E|z|^2 = 2` and
 *   `E|z|^4 = 6`. Then `K = 3/2`, and the excess is `-0.5`.
 * - A real sinusoid. `m4 = 3A^4/8` and `m2 = A^2/2`. Then `K = 3/2`, and the excess is `-1.5`.
 *
 * These bounded signals read a negative excess. An impulsive signal reads a positive excess.
 *
 * **The estimator's bias and spread.** The plug-in estimator `m4_hat/m2_hat^2` is biased low. Apply the delta
 * method to `u = |z|^2`, exponential with mean `mu`. Then `E[u^2] = 2mu^2`, `E[u^3] = 6mu^3` and `E[u^4] = 24mu^4`.
 * Write `A = mean(u^2)` and `B = mean(u)`.
 *
 * ```
 * Var(A) = 20 mu^4 / N     Var(B) = mu^2 / N     Cov(A,B) = 4 mu^3 / N
 * E[A/B^2] ~ 2 - 8/N + 6/N = 2 - 2/N
 * Var(A/B^2) ~ 20/N + 16/N - 32/N = 4/N
 * ```
 *
 * The complex excess has a bias of `-2/N` and a standard deviation of `2/sqrt(N)`. The same construction on a real
 * Gaussian gives a bias of `-6/N` and the classical variance `24/N`, a standard deviation of `4.899/sqrt(N)`.
 *
 * The functions return the plug-in estimate. Subtracting `kurtosisBias` from it removes the bias to first order in
 * `1/N`.
 */
namespace gr::measurement {

/// The sample types of the moments: `float`, `double` and their complex forms.
template<typename T>
concept MomentSample = gr::math::RealOrComplexSample<T>;

namespace detail {

template<typename T>
using MomentDomain = gr::math::RealScalarOf<T>;

} // namespace detail

/// @brief `m4/m2^2` of a Gaussian: 3 for real input and 2 for complex input.
[[nodiscard]] constexpr double gaussianFourthMoment(bool complexDomain) noexcept { return complexDomain ? 2. : 3.; }

/// @brief The plug-in estimator's bias over `n` samples: `-2/n` complex, `-6/n` real, and zero at `n = 0`.
[[nodiscard]] constexpr double kurtosisBias(std::size_t n, bool complexDomain) noexcept { return n == 0UZ ? 0. : -(complexDomain ? 2. : 6.) / static_cast<double>(n); }

/// @brief The estimator's variance over `n` samples: `4/n` complex, `24/n` real, and zero at `n = 0`.
[[nodiscard]] constexpr double kurtosisVariance(std::size_t n, bool complexDomain) noexcept { return n == 0UZ ? 0. : (complexDomain ? 4. : 24.) / static_cast<double>(n); }

/// @brief The standard deviation of one window's reading: `2/sqrt(n)` complex and `4.899/sqrt(n)` real.
[[nodiscard]] inline double kurtosisSpread(std::size_t n, bool complexDomain) noexcept { return std::sqrt(kurtosisVariance(n, complexDomain)); }

/**
 * @brief Accumulates `n`, `sum|x|^2` and `sum|x|^4` in `double` and derives `K`, the excess and the mean power.
 *
 * The sums are plain sums. The relative error of a plain sum of `N` non-negative terms is at most `N * eps`, with
 * `eps = 2^-52 = 2.22e-16`. At a window of `2^24` samples that is `3.7e-9`. The estimate's spread at that window is
 * `2/sqrt(N) = 4.9e-4`, five orders of magnitude larger.
 *
 * `degenerate()` is true for an empty window, for a window whose `sum|x|^2` is exactly zero, and for a window whose
 * sums are not finite. A NaN or infinite sample makes the sums NaN or infinite. The ratio is then undefined, and
 * `normalized()`, `excess()` and `meanPower()` read zero. `finite()` is false in the last case alone.
 */
template<MomentSample T>
class KurtosisAccumulator {
public:
    static constexpr bool kComplexDomain = detail::MomentDomain<T>::isComplex;

    constexpr void reset() noexcept {
        _n  = 0UZ;
        _m2 = 0.;
        _m4 = 0.;
    }

    constexpr void add(T x) noexcept {
        const double p = power(x);
        ++_n;
        _m2 += p;
        _m4 += p * p;
    }

    constexpr void add(std::span<const T> in) noexcept {
        double sum2 = _m2;
        double sum4 = _m4;
        for (const T x : in) {
            const double p = power(x);
            sum2 += p;
            sum4 += p * p;
        }
        _n += in.size();
        _m2 = sum2;
        _m4 = sum4;
    }

    [[nodiscard]] constexpr std::size_t count() const noexcept { return _n; }
    [[nodiscard]] constexpr double      sumPower() const noexcept { return _m2; }
    [[nodiscard]] constexpr double      sumPowerSquared() const noexcept { return _m4; }
    [[nodiscard]] constexpr bool        finite() const noexcept { return _m2 <= std::numeric_limits<double>::max() && _m4 <= std::numeric_limits<double>::max(); }
    [[nodiscard]] constexpr bool        degenerate() const noexcept { return _n == 0UZ || _m2 == 0. || !finite(); }
    [[nodiscard]] constexpr double      meanPower() const noexcept { return degenerate() ? 0. : _m2 / static_cast<double>(_n); }

    /// @brief `m4/m2^2`, or zero on a degenerate window.
    [[nodiscard]] constexpr double normalized() const noexcept { return degenerate() ? 0. : static_cast<double>(_n) * _m4 / (_m2 * _m2); }

    /// @brief `normalized()` less the Gaussian value of the domain, or zero on a degenerate window.
    [[nodiscard]] constexpr double excess() const noexcept { return degenerate() ? 0. : normalized() - gaussianFourthMoment(kComplexDomain); }

    /// @brief The closed-form bias and spread at this window's sample count.
    [[nodiscard]] constexpr double bias() const noexcept { return kurtosisBias(_n, kComplexDomain); }
    [[nodiscard]] double           spread() const noexcept { return kurtosisSpread(_n, kComplexDomain); }

private:
    [[nodiscard]] static constexpr double power(T x) noexcept {
        if constexpr (kComplexDomain) {
            const double re = static_cast<double>(x.real());
            const double im = static_cast<double>(x.imag());
            return re * re + im * im;
        } else {
            const double v = static_cast<double>(x);
            return v * v;
        }
    }

    std::size_t _n{0UZ};
    double      _m2{0.};
    double      _m4{0.};
};

/**
 * @brief Spectral kurtosis of one bin over `M` independent estimates, from `S1 = sum u_k` and `S2 = sum u_k^2`.
 *
 * With `u_k` the bin's power in estimate `k`,
 *
 * ```
 * SK = ((M*d + 1) / (M - 1)) * ( M * S2 / S1^2 - 1 )
 * ```
 *
 * where `d` is the Gamma shape of the bin's power. A bin of a transform of circular complex noise has `d = 1` in one
 * periodogram and `d = n` in the mean of `n` periodograms. The DC and Nyquist bins of a transform of real input are
 * real, and their shape is half of that, `n/2`. For `u_k` i.i.d. Gamma of shape `d`, the vector `u_k/S1` is
 * Dirichlet with all parameters `d` and is independent of `S1`. With `Q = S2/S1^2`, the Dirichlet moment
 * `E[w^2] = d(d+1)/(Md(Md+1))` gives
 *
 * ```
 * E[Q] = (d+1)/(Md+1)          E[M*Q - 1] = (M - 1)/(Md + 1)
 * ```
 *
 * The leading factor is the reciprocal of `E[M*Q - 1]`. On noise, evaluated at the bin's own shape, `E[SK] = 1`
 * exactly, at every `M` and every `d`.
 *
 * Returns zero when @p s1 is not positive or @p m is below 2.
 */
[[nodiscard]] inline double spectralKurtosis(double s1, double s2, std::size_t m, double shape) noexcept {
    if (!(s1 > 0.) || m < 2UZ) {
        return 0.;
    }
    const double count = static_cast<double>(m);
    return ((count * shape + 1.) / (count - 1.)) * (count * s2 / (s1 * s1) - 1.);
}

/// @brief The expected spectral kurtosis on noise: 1 at every `M` and `d`.
[[nodiscard]] constexpr double spectralKurtosisExpectation() noexcept { return 1.; }

/**
 * @brief `Var(SK) = 2 M^2 d (d+1) / ((M-1)(Md+2)(Md+3))`, from the same Dirichlet moments.
 *
 * It tends to `2(d+1)/(dM)`. At `d = 1` it is `4M^2/((M-1)(M+2)(M+3))`, which approaches `4/M`. Returns zero when
 * @p m is below 2.
 */
[[nodiscard]] inline double spectralKurtosisVariance(std::size_t m, double shape) noexcept {
    if (m < 2UZ) {
        return 0.;
    }
    const double count = static_cast<double>(m);
    const double md    = count * shape;
    return 2. * count * count * shape * (shape + 1.) / ((count - 1.) * (md + 2.) * (md + 3.));
}

/// @brief The standard deviation of one spectral-kurtosis reading.
[[nodiscard]] inline double spectralKurtosisSpread(std::size_t m, double shape) noexcept { return std::sqrt(spectralKurtosisVariance(m, shape)); }

/**
 * @brief Spectral kurtosis of a carrier in noise at bin signal-to-noise ratio @p rho, `(2*rho + 1)/(rho + 1)^2`.
 *
 * `u = |A + n|^2` has `E[u] = S + N0` and `E[u^2] = S^2 + 4 S N0 + 2 N0^2`. The large-`M` limit of `SK` is
 * `E[u^2]/E[u]^2 - 1`, which reduces to the expression above. It is 1 at `rho = 0`, noise alone. It falls through
 * 0.75, 0.556, 0.174 and 0.0197 at `rho = 1, 2, 10, 100` and reaches 0 for a noiseless carrier.
 *
 * A carrier present in a fraction `p` of the spectra has the large-`M` limit `E[u^2]/E[u]^2 - 1` over the mixture.
 * At `d = 1` it is above 1 for `p` below one half, exactly 1 at one half and below 1 above it, at every signal level.
 */
[[nodiscard]] constexpr double spectralKurtosisCw(double rho) noexcept { return (2. * rho + 1.) / ((rho + 1.) * (rho + 1.)); }

/// The result of folding one spectrum into the accumulation.
enum class SpectrumResponse : std::uint8_t {
    accepted,      ///< folded in
    wrongBinCount, ///< a bin count other than the accumulation's; nothing folded in
    invalidBin     ///< a negative, NaN or infinite bin; nothing folded in
};

/**
 * @brief Accumulates `S1` and `S2` per bin over a run of spectra and evaluates the per-bin spectral kurtosis.
 *
 * Folding a spectrum in scans it for refused bins, then costs one add and one multiply-add per bin. An evaluation
 * costs one division per bin.
 *
 * `SK` is homogeneous of degree zero in `u`. Replacing `u_k` by `a*u_k` leaves `M*S2/S1^2` unchanged. A positive
 * scale that is the same for every spectrum of the run cancels, such as a window's noise bandwidth or a density
 * calibration. The input is linear power. A spectrum in decibels is converted to linear power first.
 *
 * The derivation requires the `M` spectra to be independent. Periodograms of overlapping segments share samples and
 * are correlated, and their `SK` is biased. The class does not check independence.
 *
 * A spectrum with the wrong bin count, or with a negative, NaN or infinite bin, is refused and counted in
 * `nRefused()`. It leaves the accumulation unchanged.
 */
class SpectralKurtosisAccumulator {
public:
    SpectralKurtosisAccumulator() = default;
    explicit SpectralKurtosisAccumulator(std::size_t nBins) { resize(nBins); }

    /// @brief Sizes the accumulation for @p nBins bins and clears it.
    void resize(std::size_t nBins) {
        _s1.assign(nBins, 0.);
        _s2.assign(nBins, 0.);
        _m = 0UZ;
    }

    /// @brief Clears the sums and the spectrum count. The bin count and `nRefused()` stay.
    void reset() noexcept {
        std::ranges::fill(_s1, 0.);
        std::ranges::fill(_s2, 0.);
        _m = 0UZ;
    }

    /// @brief Folds one spectrum in. A refused spectrum leaves the accumulation unchanged.
    SpectrumResponse accumulate(std::span<const float> spectrum) noexcept {
        if (spectrum.size() != _s1.size()) {
            ++_refused;
            return SpectrumResponse::wrongBinCount;
        }
        for (const float value : spectrum) {
            if (!(value >= 0.f && value <= std::numeric_limits<float>::max())) {
                ++_refused;
                return SpectrumResponse::invalidBin;
            }
        }
        for (std::size_t bin = 0UZ; bin < spectrum.size(); ++bin) {
            const double u = static_cast<double>(spectrum[bin]);
            _s1[bin] += u;
            _s2[bin] += u * u;
        }
        ++_m;
        return SpectrumResponse::accepted;
    }

    /**
     * @brief Writes the per-bin statistic at shape @p shape to @p out and returns the number of bins with no power.
     *
     * A bin whose `S1` is exactly zero has no defined `SK`. It is written as zero and counted. Below two spectra every
     * bin is written as zero, as `spectralKurtosis` reads. An @p out without one entry per bin is an error.
     */
    [[nodiscard]] std::expected<std::size_t, gr::Error> evaluate(double shape, std::span<double> out, std::source_location loc = std::source_location::current()) const {
        if (out.size() != _s1.size()) {
            return std::unexpected(gr::Error("gr::measurement::SpectralKurtosisAccumulator::evaluate: the output span must have one entry per bin", loc));
        }
        std::size_t  degenerate = 0UZ;
        const double count      = static_cast<double>(_m);
        const double factor     = _m < 2UZ ? 0. : (count * shape + 1.) / (count - 1.);
        for (std::size_t bin = 0UZ; bin < out.size(); ++bin) {
            if (!(_s1[bin] > 0.)) {
                out[bin] = 0.;
                ++degenerate;
                continue;
            }
            out[bin] = factor * (count * _s2[bin] / (_s1[bin] * _s1[bin]) - 1.);
        }
        return degenerate;
    }

    [[nodiscard]] std::size_t             bins() const noexcept { return _s1.size(); }
    [[nodiscard]] std::size_t             count() const noexcept { return _m; }
    [[nodiscard]] std::uint64_t           nRefused() const noexcept { return _refused; }
    [[nodiscard]] std::span<const double> sums() const noexcept { return _s1; }
    [[nodiscard]] std::span<const double> sumSquares() const noexcept { return _s2; }

private:
    std::vector<double> _s1;
    std::vector<double> _s2;
    std::size_t         _m{0UZ};
    std::uint64_t       _refused{0ULL};
};

} // namespace gr::measurement

#endif // GNURADIO_ALGORITHM_KURTOSIS_HPP
