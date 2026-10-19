#ifndef GNURADIO_ALGORITHM_FILTER_REAL_TO_COMPLEX_HPP
#define GNURADIO_ALGORITHM_FILTER_REAL_TO_COMPLEX_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <expected>
#include <format>
#include <source_location>
#include <span>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/algorithm/fourier/window.hpp>

/**
 * @brief Converts a real stream to a complex stream at half the rate.
 *
 * The conversion mixes the input down by a quarter of its rate `fs`, low-pass filters it at `fs/4` and keeps every
 * second sample. The output runs at `fs/2`, and its zero frequency is the input's `fs/4`. A tone at `f` in the input
 * leaves at `f - fs/4`. A real tone of amplitude A leaves as a complex tone of magnitude A/2, 6 dB down. The real tone
 * is the sum of two complex exponentials of half its amplitude, and the conversion keeps one of them.
 *
 * The filter is a Hamming-windowed low-pass with unit gain at zero frequency. Its -6 dB points sit at the input's zero
 * frequency and at its `fs/2`. The band passed without loss runs from half a transition width above zero to half a
 * transition width below `fs/2`. Outside the transition the mirror image lies about 53 dB below the band. A narrower
 * transition costs more taps.
 *
 * `make()` builds a conversion for one rate and one transition width. A change of either takes a new conversion.
 * Each output costs one real dot product over about half the taps and one multiply.
 */
namespace gr::filter {

class RealToComplex {
public:
    /// @brief The longest filter designed. A transition narrower than `3.319 fs / 4095` gets this length.
    static constexpr std::size_t kMaxTaps = 4095UZ;

    /**
     * @brief The filter length for a transition @p transitionHz wide at the rate @p sampleRate.
     *
     * The length follows from the Hamming window's measured transition constant, rounded up to odd. It is at least 3
     * and at most `kMaxTaps`. A rate or a transition width that is not positive and finite is an error.
     */
    [[nodiscard]] static std::expected<std::size_t, gr::Error> tapCount(double sampleRate, double transitionHz, std::source_location location = std::source_location::current()) {
        if (!(std::isfinite(sampleRate) && sampleRate > 0.0) || !(std::isfinite(transitionHz) && transitionHz > 0.0)) {
            return std::unexpected(gr::Error(std::format("RealToComplex: the sample rate ({}) and the transition width ({}) must be positive and finite", sampleRate, transitionHz), location));
        }
        using gr::algorithm::window::Type;
        const double width = transitionHz / sampleRate;
        if (!(fir::design::windowFigures(Type::Hamming).transitionConstant / width < static_cast<double>(kMaxTaps))) { // checked in double, ahead of any conversion to an integer
            return kMaxTaps;
        }
        return fir::design::oddLength(fir::design::windowLength(Type::Hamming, width));
    }

    /// @brief The Hamming-windowed low-pass at a quarter of the rate, with unit gain at zero frequency, of `tapCount` taps.
    [[nodiscard]] static std::expected<std::vector<float>, gr::Error> design(double sampleRate, double transitionHz, std::source_location location = std::source_location::current()) {
        const std::expected<std::size_t, gr::Error> n = tapCount(sampleRate, transitionHz, location);
        if (!n) {
            return std::unexpected(n.error());
        }
        return fir::design::lowpass(*n, 0.25, fir::design::WindowSpec{gr::algorithm::window::Type::Hamming}, 1.0);
    }

    /// @brief A conversion for the rate @p sampleRate and a transition @p transitionHz wide, at its start-of-stream state.
    [[nodiscard]] static std::expected<RealToComplex, gr::Error> make(double sampleRate, double transitionHz, std::source_location location = std::source_location::current()) {
        std::expected<std::vector<float>, gr::Error> taps = design(sampleRate, transitionHz, location);
        if (!taps) {
            return std::unexpected(taps.error());
        }
        return RealToComplex(std::move(*taps));
    }

    /// @brief The output rate for an input at @p sampleRate.
    [[nodiscard]] static constexpr double outputRate(double sampleRate) noexcept { return 0.5 * sampleRate; }

    /// @brief The input frequency that leaves at zero, for an input at @p sampleRate.
    [[nodiscard]] static constexpr double centerFrequency(double sampleRate) noexcept { return 0.25 * sampleRate; }

    /// @brief The lowest input frequency inside the passband, for a transition @p transitionHz wide.
    [[nodiscard]] static constexpr double lowEdge(double transitionHz) noexcept { return 0.5 * transitionHz; }

    /// @brief The designed taps, the zeros included.
    [[nodiscard]] std::span<const float> taps() const noexcept { return _taps; }

    /// @brief The filter's group delay, `(N-1)/2` input samples.
    [[nodiscard]] std::size_t groupDelay() const noexcept { return (_taps.size() - 1UZ) / 2UZ; }

    /**
     * @brief The outputs that @p inputs further input samples make.
     *
     * Counted from the start of the stream, input `2n` makes output `n`. A call that starts on an even input
     * makes `ceil(inputs/2)` outputs, and one that starts on an odd input `floor(inputs/2)`.
     */
    [[nodiscard]] std::size_t outputsFor(std::size_t inputs) const noexcept { return _oddNext ? inputs / 2UZ : (inputs + 1UZ) / 2UZ; }

    /// @brief The most input samples whose outputs fit in @p outputs.
    [[nodiscard]] std::size_t inputsFor(std::size_t outputs) const noexcept { return 2UZ * outputs + (_oddNext ? 1UZ : 0UZ); }

    /// @brief Return to the start-of-stream state: zero history, the mixer at its first phase, the next input even.
    void reset() {
        _dotLine.assign(_dotHistory, 0.f);
        _singleLine.assign(_singleHistory, 0.f);
        _oddNext   = false;
        _negateOut = false;
    }

    /**
     * @brief Convert @p in and write `outputsFor(in.size())` outputs to the front of @p out. Returns that count.
     *
     * @p out must hold that many, or the call converts nothing and returns 0. A call may have any length, an odd one
     * included. The history, the mixer phase and the decimation phase carry from one call to the next. A stream split
     * into calls anywhere gives the same outputs, bit for bit, as the stream in one call.
     */
    std::size_t process(std::span<const float> in, std::span<std::complex<float>> out) {
        const std::size_t made = outputsFor(in.size());
        if (made > out.size()) {
            return 0UZ;
        }
        split(in);

        const float* line   = _dotLine.data();
        const float* single = _singleLine.data();
        const float* dot    = _dot.data();
        const auto   nDot   = _dot.size();

        constexpr std::size_t kLanes = 16UZ;
        std::size_t           j      = 0UZ;
        for (; j + kLanes <= made; j += kLanes) {
            float acc[kLanes] = {};
            for (std::size_t t = 0UZ; t < nDot; ++t) {
                const float  w = dot[t];
                const float* p = line + j + t;
                for (std::size_t u = 0UZ; u < kLanes; ++u) {
                    acc[u] += p[u] * w;
                }
            }
            for (std::size_t u = 0UZ; u < kLanes; ++u) {
                out[j + u] = emit(acc[u], single[j + u]);
            }
        }
        // The tail loop sums over t in the blocks' order. An output then does not depend on where a call boundary fell.
        for (; j < made; ++j) {
            float acc = 0.f;
            for (std::size_t t = 0UZ; t < nDot; ++t) {
                acc += line[j + t] * dot[t];
            }
            out[j] = emit(acc, single[j]);
        }

        _dotLine.erase(_dotLine.begin(), _dotLine.begin() + static_cast<std::ptrdiff_t>(made));
        _singleLine.erase(_singleLine.begin(), _singleLine.begin() + static_cast<std::ptrdiff_t>(made));
        _oddNext = (in.size() % 2UZ == 1UZ) != _oddNext;
        return made;
    }

    /// @brief Convert @p in and append its outputs to @p out. Returns the count appended.
    std::size_t process(std::span<const float> in, std::vector<std::complex<float>>& out) {
        const std::size_t at = out.size();
        out.resize(at + outputsFor(in.size()));
        return process(in, std::span<std::complex<float>>(out).subspan(at));
    }

private:
    /**
     * The taps come from `design()`, a low-pass at a quarter of the rate. It is a halfband filter: every tap at an
     * even, nonzero offset from the center is zero.
     *
     * Output `n` is `(-1)^n * sum_k h[k] * m[k] * x[2n-k]`, where `m[k]` is 1, +j, -1, -j for `k mod 4` = 0, 1, 2, 3.
     * With the center index `c`, the taps that remain are `h[c]` and those at an odd offset from it. An even `c` puts
     * `h[c]` on the real part and the dot product over the odd inputs on the imaginary part. An odd `c` swaps the two.
     * The dot product reads its line forward from output `n`, oldest sample first, with the taps reversed and signed.
     */
    explicit RealToComplex(std::vector<float> taps) : _taps(std::move(taps)) {
        const std::size_t n = _taps.size();
        const std::size_t c = (n - 1UZ) / 2UZ;
        _centerOdd          = c % 2UZ == 1UZ;
        const auto mixSign  = [](std::size_t k) { return (k % 4UZ == 0UZ || k % 4UZ == 1UZ) ? 1.f : -1.f; };
        // the first tap at an odd offset from c. Output n reads that tap's input at index n - first of its parity.
        const std::size_t first = _centerOdd ? 0UZ : 1UZ;
        for (std::size_t k = first; k < n; k += 2UZ) {
            _dot.push_back(mixSign(k) * _taps[k]);
        }
        std::ranges::reverse(_dot);
        _single        = mixSign(c) * _taps[c];
        _dotHistory    = first + _dot.size() - 1UZ;
        _singleHistory = _centerOdd ? (c + 1UZ) / 2UZ : c / 2UZ;
        reset();
    }

    /// Appends each input sample to the line of its parity.
    void split(std::span<const float> in) {
        std::vector<float>& even  = _centerOdd ? _dotLine : _singleLine;
        std::vector<float>& odd   = _centerOdd ? _singleLine : _dotLine;
        bool                isOdd = _oddNext;
        for (const float x : in) {
            (isOdd ? odd : even).push_back(x);
            isOdd = !isOdd;
        }
    }

    [[nodiscard]] std::complex<float> emit(float sum, float sample) noexcept {
        const float centerPart = _single * sample;
        const float sign       = _negateOut ? -1.f : 1.f;
        _negateOut             = !_negateOut;
        return _centerOdd ? std::complex<float>{sign * sum, sign * centerPart} : std::complex<float>{sign * centerPart, sign * sum};
    }

    std::vector<float> _taps;
    std::vector<float> _dot;                 ///< the taps at odd offsets from the center, signed and reversed
    float              _single        = 0.f; ///< the center tap, signed
    bool               _centerOdd     = false;
    std::size_t        _dotHistory    = 0UZ; ///< the zeros ahead of the first input in the dot product's line
    std::size_t        _singleHistory = 0UZ; ///< the zeros ahead of the first input in the center tap's line
    std::vector<float> _dotLine;
    std::vector<float> _singleLine;
    bool               _oddNext   = false; ///< the next input has an odd index
    bool               _negateOut = false; ///< the next output takes the sign -1
};

} // namespace gr::filter

#endif // GNURADIO_ALGORITHM_FILTER_REAL_TO_COMPLEX_HPP
