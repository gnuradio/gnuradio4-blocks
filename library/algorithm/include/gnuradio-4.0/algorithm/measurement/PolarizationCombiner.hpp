#ifndef GNURADIO_ALGORITHM_POLARIZATION_COMBINER_HPP
#define GNURADIO_ALGORITHM_POLARIZATION_COMBINER_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <optional>
#include <source_location>
#include <span>
#include <string_view>

#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/algorithm/math/Decibel.hpp>

/**
 * @brief Combines two antenna branches at the maximal-ratio optimum, from the 2x2 covariance in closed form.
 *
 * A caller adds the paired samples of a window to `BranchCovariance`, calls `solve()` at the window's close, and
 * applies the returned weights with `polarizationCombine`. The estimate also carries the relative phase, the amplitude
 * ratio and the signal-to-noise ratio of each branch and of the output.
 *
 * Two branches carry one signal through two channels, `r_i(k) = h_i s(k) + n_i(k)`. The branch noises are
 * independent. A linear combiner forms `y = conj(w_0) r_0 + conj(w_1) r_1`, and
 *
 * ```
 * SNR_out = P * |sum_i conj(w_i) h_i|^2 / (sum_i |w_i|^2 N_i)
 * ```
 *
 * By Cauchy-Schwarz this is largest at `w_i` proportional to `h_i / N_i`, where
 *
 * ```
 * SNR_out = P * sum_i |h_i|^2 / N_i = SNR_0 + SNR_1
 * ```
 *
 * The output signal-to-noise ratio is the sum of the branch ratios. For two equal branches the gain over either
 * branch is `10*log10(2) = 3.0103 dB`. For unequal branches the gain over the stronger one is
 * `10*log10(1 + SNR_min/SNR_max)`: 3.0103, 1.7609, 0.9691 and 0.4139 dB at ratios 1, 0.5, 0.25 and 0.1.
 *
 * **The estimator.** The combiner measures the branch covariance.
 *
 * ```
 * R = E[ r r^H ] = P h h^H + diag(N_0, N_1) = [[a, c], [conj(c), b]]
 * ```
 *
 * Under equal branch noise `R = P h h^H + N I`. Its principal eigenvector is `h`, the maximal-ratio weight vector.
 * The 2x2 case has a closed form.
 *
 * ```
 * lambda_+ = (a+b)/2 + sqrt( ((a-b)/2)^2 + |c|^2 )
 * lambda_- = (a+b)/2 - sqrt( ((a-b)/2)^2 + |c|^2 )
 * v        = [ c , lambda_+ - a ]
 * ```
 *
 * `v` is an exact eigenvector. The second row of `(R - lambda_+ I) v = 0` reads `(lambda_+ - a)(lambda_+ - b) =
 * |c|^2`, the characteristic equation. Three properties follow.
 *
 * - The phase and the amplitude ratio carry no noise bias. With independent branch noises,
 *   `c = E[r_0 conj(r_1)] = P h_0 conj(h_1)`, and `arg(c) = arg(h_0) - arg(h_1)`. With `lambda_+ - a = P|h_1|^2` and
 *   `|c| = P|h_0||h_1|`, the ratio `|c|/(lambda_+ - a)` is `|h_0|/|h_1|`. The ratio
 *   `sqrt(a/b) = sqrt((P|h_0|^2+N)/(P|h_1|^2+N))` is biased toward 1 by the noise.
 * - The ratios add on the estimates. `a - lambda_- = P|h_0|^2` and `b - lambda_- = P|h_1|^2`. The two branch ratios
 *   sum to `(a + b - 2 lambda_-)/lambda_- = (lambda_+ - lambda_-)/lambda_-`, the combined ratio under `mrc`. The
 *   identity holds for any data while `lambda_-` lies above the noise floor below.
 * - The orthogonal output nulls the signal. `u = [-conj(w_1), conj(w_0)]` satisfies `w^H u = 0` for any `w`. With
 *   `w` along `h`, the output of `u` holds the interference and the noise.
 *
 * **Selection.** `selection` passes the branch with the larger whitened diagonal entry alone, and the orthogonal
 * weights pass the other branch. The combined ratio is then the selected branch's ratio, and the combining gain is 0
 * dB. `unit_signal` scales the selected branch to an estimated signal power of 1. That estimate is the branch's
 * diagonal entry less `lambda_-`.
 *
 * **Saturation.** Noiseless branches leave `lambda_-` at zero or at the rounding level of the float samples, near
 * 150 dB below the total power and of either sign. `solve()` reads a `lambda_-` at or below `kPolarizationNoiseFloor`
 * times the trace `a + b`, 100 dB below the total power, as no measured noise. `saturatedSnr` is then set, and each
 * ratio reads `kPolarizationSaturation`. A branch whose diagonal entry lies at or below the same floor carries no
 * signal, and its ratio reads 0. A `lambda_+ - a` at or below the floor sets `saturatedRatio`, and the amplitude
 * ratio reads the cap.
 *
 * **Unequal branch noise.** Dividing `r_i` by `sqrt(N_i)` gives both noises unit power and turns `h_i` into
 * `h_i/sqrt(N_i)`. The solution above then returns the whitened weights. The weight applied to `r_i` is the whitened
 * weight over `sqrt(N_i)`, proportional to `h_i/N_i`. The whitened amplitude ratio is `|h_0|/|h_1| * sqrt(N_1/N_0)`,
 * and `solve()` multiplies it by `sqrt(N_0/N_1)`.
 *
 * **The gauge.** An eigenvector is defined up to a complex scalar of unit modulus. `solve()` normalizes `v` and
 * rotates it to `arg(v_0) = 0`. The weight on branch 0 is then real and non-negative. Under `mrc` the signal in the
 * combined output has the phase of the signal in branch 0.
 */
namespace gr::measurement {

/// The rule that turns the two branches into one.
enum class PolarizationMode : std::uint8_t {
    mrc,      ///< the maximal-ratio combination
    selection ///< the stronger branch alone, chosen from the same covariance
};

/// The scaling of the reported weight vector.
enum class PolarizationNormalization : std::uint8_t {
    unit_noise, ///< the whitened weight vector has unit norm, and the output noise power equals one whitened branch's
    unit_signal ///< the estimated output signal power is 1
};

[[nodiscard]] inline constexpr std::string_view polarizationModeName(PolarizationMode mode) noexcept { return mode == PolarizationMode::selection ? "selection" : "mrc"; }
[[nodiscard]] inline constexpr std::string_view polarizationNormalizationName(PolarizationNormalization norm) noexcept { return norm == PolarizationNormalization::unit_signal ? "unit_signal" : "unit_noise"; }

[[nodiscard]] inline constexpr std::optional<PolarizationMode> polarizationModeFrom(std::string_view name) noexcept {
    if (name == "mrc") {
        return PolarizationMode::mrc;
    }
    if (name == "selection") {
        return PolarizationMode::selection;
    }
    return std::nullopt;
}

[[nodiscard]] inline constexpr std::optional<PolarizationNormalization> polarizationNormalizationFrom(std::string_view name) noexcept {
    if (name == "unit_noise") {
        return PolarizationNormalization::unit_noise;
    }
    if (name == "unit_signal") {
        return PolarizationNormalization::unit_signal;
    }
    return std::nullopt;
}

/// The value reported in place of an unbounded ratio, `1e6` or 60 dB. `saturatedSnr` or `saturatedRatio` marks
/// each use.
inline constexpr double kPolarizationSaturation = 1e6;

/// The share of the covariance trace at or below which `lambda_-`, `lambda_+ - a` or a diagonal entry reads as zero,
/// 100 dB.
inline constexpr double kPolarizationNoiseFloor = 1e-10;

/// @brief The result of the closed-form eigensolution, with the weights to apply.
struct PolarizationEstimate {
    double lambdaPlus{0.};  ///< the principal eigenvalue of the whitened covariance
    double lambdaMinus{0.}; ///< the other eigenvalue, the noise power in whitened units

    std::complex<double> weight0{1., 0.}; ///< applied as `conj(weight0) * r_0 + conj(weight1) * r_1`
    std::complex<double> weight1{0., 0.};
    std::complex<double> ortho0{0., 0.}; ///< the orthogonal weights, which null the signal under `mrc`
    std::complex<double> ortho1{1., 0.};

    double relativePhase{0.};     ///< `arg(h_0) - arg(h_1)`, in radians
    double amplitudeRatio{1.};    ///< `|h_0| / |h_1|`
    double branchSnr0{0.};        ///< linear power ratio
    double branchSnr1{0.};        ///< linear power ratio
    double combinedSnr{0.};       ///< linear power ratio of the output
    double scale{1.};             ///< the normalization factor in the weights
    int    selectedBranch{-1};    ///< the branch `selection` or a pass-through chose, or -1 under `mrc`
    bool   saturatedSnr{false};   ///< `lambda_-` is at or below the noise floor, and each ratio reads the cap or 0
    bool   saturatedRatio{false}; ///< `lambda_+ - a` is at or below the noise floor, and `amplitudeRatio` reads the cap

    [[nodiscard]] double branchSnr(std::size_t branch) const noexcept { return branch == 0UZ ? branchSnr0 : branchSnr1; }
    [[nodiscard]] double branchSnrDb(std::size_t branch) const noexcept { return toDb(branchSnr(branch)); }
    [[nodiscard]] double combinedSnrDb() const noexcept { return toDb(combinedSnr); }
    /// @brief The gain of the output over the better of the two branches, in dB.
    [[nodiscard]] double combiningGainDb() const noexcept { return combinedSnrDb() - std::max(branchSnrDb(0UZ), branchSnrDb(1UZ)); }

    [[nodiscard]] static double toDb(double linear) noexcept { return gr::math::toDb(linear); }
};

/// @brief An estimate that passes branch @p branch through unchanged and sends the other branch to the orthogonal
/// output. With branch 0 its gauge matches the one `BranchCovariance::solve` fixes.
[[nodiscard]] inline PolarizationEstimate polarizationPassthrough(std::size_t branch = 0UZ) {
    PolarizationEstimate out;
    out.selectedBranch = static_cast<int>(branch);
    if (branch == 0UZ) {
        out.weight0 = {1., 0.};
        out.weight1 = {0., 0.};
        out.ortho0  = {0., 0.};
        out.ortho1  = {1., 0.};
    } else {
        out.weight0 = {0., 0.};
        out.weight1 = {1., 0.};
        out.ortho0  = {1., 0.};
        out.ortho1  = {0., 0.};
    }
    return out;
}

/**
 * @brief Accumulates the three covariance entries over a window and solves for the weights in closed form.
 *
 * `a = mean |r_0|^2`, `b = mean |r_1|^2` and `c = mean r_0 conj(r_1)` are summed in `double` in stream order. Any
 * split of the stream gives the same sums. `count()` is the number of pairs in the sums. A pair with a non-finite
 * component stays out of the sums and adds one to `nonFinite()`.
 */
class BranchCovariance {
public:
    void reset() noexcept {
        _n         = 0UZ;
        _nonFinite = 0UZ;
        _a         = 0.;
        _b         = 0.;
        _cRe       = 0.;
        _cIm       = 0.;
    }

    void add(std::complex<float> r0, std::complex<float> r1) noexcept {
        const double x0 = static_cast<double>(r0.real());
        const double y0 = static_cast<double>(r0.imag());
        const double x1 = static_cast<double>(r1.real());
        const double y1 = static_cast<double>(r1.imag());
        const double p0 = x0 * x0 + y0 * y0;
        const double p1 = x1 * x1 + y1 * y1;
        // The square of a finite float is finite in double, and the sum is finite exactly when the four parts are.
        if (!std::isfinite(p0 + p1)) {
            ++_nonFinite;
            return;
        }
        _a += p0;
        _b += p1;
        _cRe += x0 * x1 + y0 * y1; // r_0 * conj(r_1)
        _cIm += y0 * x1 - x0 * y1;
        ++_n;
    }

    /// @brief Accumulates paired samples. Returns a `gr::Error` and accumulates nothing when the spans differ in
    /// length.
    [[nodiscard]] std::expected<void, gr::Error> add(std::span<const std::complex<float>> in0, std::span<const std::complex<float>> in1, std::source_location loc = std::source_location::current()) {
        if (in0.size() != in1.size()) {
            return std::unexpected(gr::Error(std::format("gr::measurement::BranchCovariance::add: {} samples on branch 0 against {} on branch 1", in0.size(), in1.size()), loc));
        }
        for (std::size_t k = 0UZ; k < in0.size(); ++k) {
            add(in0[k], in1[k]);
        }
        return {};
    }

    [[nodiscard]] std::size_t          count() const noexcept { return _n; }
    [[nodiscard]] std::size_t          nonFinite() const noexcept { return _nonFinite; }
    [[nodiscard]] double               a() const noexcept { return _n == 0UZ ? 0. : _a / static_cast<double>(_n); }
    [[nodiscard]] double               b() const noexcept { return _n == 0UZ ? 0. : _b / static_cast<double>(_n); }
    [[nodiscard]] std::complex<double> c() const noexcept { return _n == 0UZ ? std::complex<double>{} : std::complex<double>{_cRe / static_cast<double>(_n), _cIm / static_cast<double>(_n)}; }

    /**
     * @brief Solves for the weights, the channel estimates and the three signal-to-noise ratios.
     *
     * @param mode        maximal ratio, or the stronger branch alone
     * @param norm        the scaling of the weights
     * @param noisePowers empty for equal branch noise, or the two branch noise powers to whiten by
     *
     * Returns a `gr::Error` for any other count of noise powers, or for a noise power that is not positive and finite.
     */
    [[nodiscard]] std::expected<PolarizationEstimate, gr::Error> solve(PolarizationMode mode = PolarizationMode::mrc, PolarizationNormalization norm = PolarizationNormalization::unit_noise, std::span<const double> noisePowers = {}, std::source_location loc = std::source_location::current()) const {
        double n0 = 1.;
        double n1 = 1.;
        if (!noisePowers.empty()) {
            if (noisePowers.size() != 2UZ) {
                return std::unexpected(gr::Error(std::format("gr::measurement::BranchCovariance::solve: {} noise powers for two branches", noisePowers.size()), loc));
            }
            for (const double power : noisePowers) {
                if (!std::isfinite(power) || !(power > 0.)) {
                    return std::unexpected(gr::Error(std::format("gr::measurement::BranchCovariance::solve: a noise power of {} is not positive and finite", power), loc));
                }
            }
            n0 = noisePowers[0];
            n1 = noisePowers[1];
        }

        // Whitening scales the three covariance entries once per solve.
        const double               whitenedA = a() / n0;
        const double               whitenedB = b() / n1;
        const std::complex<double> whitenedC = c() / std::sqrt(n0 * n1);

        PolarizationEstimate out;
        const double         trace      = whitenedA + whitenedB;
        const double         noiseFloor = kPolarizationNoiseFloor * trace;
        const double         half       = 0.5 * (whitenedA - whitenedB);
        const double         disc       = std::sqrt(half * half + std::norm(whitenedC));
        out.lambdaPlus                  = 0.5 * trace + disc;
        out.lambdaMinus                 = 0.5 * trace - disc;

        const double magnitude = std::abs(whitenedC);
        const double along     = out.lambdaPlus - whitenedA; // `P|h_1|^2` under the model

        std::complex<double> v0{};
        std::complex<double> v1{};
        if (magnitude > 0.) {
            // The rotation by `exp(-j arg(c))` makes `v_0 = |c|`, real and positive. `v_1` takes the same rotation.
            v0 = {magnitude, 0.};
            v1 = along * std::conj(whitenedC) / magnitude;
        } else if (whitenedA >= whitenedB) {
            // With `c = 0` the weights select the branch with the larger whitened power.
            v0 = {1., 0.};
            v1 = {0., 0.};
        } else {
            v0 = {0., 0.};
            v1 = {1., 0.};
        }

        if (mode == PolarizationMode::selection) {
            // `a - lambda_-` and `b - lambda_-` differ by `a - b`. The stronger branch has the larger whitened diagonal
            // entry.
            out.selectedBranch = (whitenedA >= whitenedB) ? 0 : 1;
            v0                 = (out.selectedBranch == 0) ? std::complex<double>{1., 0.} : std::complex<double>{};
            v1                 = (out.selectedBranch == 0) ? std::complex<double>{} : std::complex<double>{1., 0.};
        }

        const double norm2 = std::sqrt(std::norm(v0) + std::norm(v1));
        if (norm2 > 0.) {
            v0 /= norm2;
            v1 /= norm2;
        }

        out.relativePhase  = (magnitude > 0.) ? std::arg(whitenedC) : 0.;
        out.saturatedRatio = !(along > noiseFloor);
        out.amplitudeRatio = out.saturatedRatio ? kPolarizationSaturation : magnitude / along * std::sqrt(n0 / n1);

        out.saturatedSnr = !(out.lambdaMinus > noiseFloor);
        if (out.saturatedSnr) {
            // The combined ratio is then not the sum of the branch ratios.
            const bool silent0 = !(whitenedA > noiseFloor);
            const bool silent1 = !(whitenedB > noiseFloor);
            out.branchSnr0     = silent0 ? 0. : kPolarizationSaturation;
            out.branchSnr1     = silent1 ? 0. : kPolarizationSaturation;
            out.combinedSnr    = (silent0 && silent1) ? 0. : kPolarizationSaturation;
        } else {
            out.branchSnr0  = (whitenedA - out.lambdaMinus) / out.lambdaMinus;
            out.branchSnr1  = (whitenedB - out.lambdaMinus) / out.lambdaMinus;
            out.combinedSnr = (out.lambdaPlus - out.lambdaMinus) / out.lambdaMinus;
        }
        const bool selected = mode == PolarizationMode::selection;
        if (selected) {
            out.combinedSnr = out.branchSnr(static_cast<std::size_t>(out.selectedBranch));
        }

        double scale = 1.;
        if (norm == PolarizationNormalization::unit_signal) {
            // The whitened signal power the weights pass: `lambda_+ - lambda_-` under `mrc`, the selected branch's
            // diagonal entry less `lambda_-` under `selection`.
            const double passed      = selected ? ((out.selectedBranch == 0) ? whitenedA : whitenedB) : out.lambdaPlus;
            const double signalPower = passed - out.lambdaMinus;
            scale                    = (signalPower > 0.) ? 1. / std::sqrt(signalPower) : 1.;
        }
        out.scale = scale;

        // The weight applied to `r_i` is the whitened weight over `sqrt(N_i)`. The output noise power
        // `sum |w_i|^2 N_i` is then `scale^2`.
        out.weight0 = scale * v0 / std::sqrt(n0);
        out.weight1 = scale * v1 / std::sqrt(n1);

        const std::complex<double> u0 = -std::conj(v1);
        const std::complex<double> u1 = std::conj(v0);
        out.ortho0                    = u0 / std::sqrt(n0);
        out.ortho1                    = u1 / std::sqrt(n1);
        return out;
    }

private:
    std::size_t _n{0UZ};
    std::size_t _nonFinite{0UZ};
    double      _a{0.};
    double      _b{0.};
    double      _cRe{0.};
    double      _cIm{0.};
};

/**
 * @brief Applies an estimate's weights, `out = conj(w_0) r_0 + conj(w_1) r_1`, and its orthogonal weights to @p ortho.
 *
 * An empty @p ortho skips the orthogonal output. Returns a `gr::Error` and writes nothing when the inputs differ in
 * length or an output span is shorter than the inputs.
 */
[[nodiscard]] inline std::expected<void, gr::Error> polarizationCombine(std::span<const std::complex<float>> in0, std::span<const std::complex<float>> in1, const PolarizationEstimate& estimate, std::span<std::complex<float>> out, std::span<std::complex<float>> ortho = {}, std::source_location loc = std::source_location::current()) {
    if (in0.size() != in1.size()) {
        return std::unexpected(gr::Error(std::format("gr::measurement::polarizationCombine: {} samples on branch 0 against {} on branch 1", in0.size(), in1.size()), loc));
    }
    if (out.size() < in0.size()) {
        return std::unexpected(gr::Error(std::format("gr::measurement::polarizationCombine: {} outputs for {} samples", out.size(), in0.size()), loc));
    }
    if (!ortho.empty() && ortho.size() < in0.size()) {
        return std::unexpected(gr::Error(std::format("gr::measurement::polarizationCombine: {} orthogonal outputs for {} samples", ortho.size(), in0.size()), loc));
    }

    const std::complex<float> w0 = static_cast<std::complex<float>>(std::conj(estimate.weight0));
    const std::complex<float> w1 = static_cast<std::complex<float>>(std::conj(estimate.weight1));
    const std::complex<float> u0 = static_cast<std::complex<float>>(std::conj(estimate.ortho0));
    const std::complex<float> u1 = static_cast<std::complex<float>>(std::conj(estimate.ortho1));

    for (std::size_t k = 0UZ; k < in0.size(); ++k) {
        out[k] = w0 * in0[k] + w1 * in1[k];
    }
    if (!ortho.empty()) {
        for (std::size_t k = 0UZ; k < in0.size(); ++k) {
            ortho[k] = u0 * in0[k] + u1 * in1[k];
        }
    }
    return {};
}

} // namespace gr::measurement

#endif // GNURADIO_ALGORITHM_POLARIZATION_COMBINER_HPP
