#ifndef GNURADIO_ALGORITHM_POLYPHASE_CHANNELIZER_HPP
#define GNURADIO_ALGORITHM_POLYPHASE_CHANNELIZER_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <expected>
#include <format>
#include <numbers>
#include <source_location>
#include <span>
#include <vector>

#include <initializer_list>
#include <mutex>
#include <numeric>
#include <ranges>
#include <string>
#include <utility>

#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>

namespace gr::filter {

namespace detail {

/// The prototype zero-padded to a whole number of branches. Both banks index the padded length.
[[nodiscard]] inline std::vector<float> padToBranches(std::span<const float> prototype, std::size_t channels) {
    const std::size_t  length = ((prototype.size() + channels - 1UZ) / channels) * channels;
    std::vector<float> padded(std::max(length, channels), 0.f);
    std::ranges::copy(prototype, padded.begin());
    return padded;
}

} // namespace detail

/// @brief A designed channelizer prototype and what it was measured to deliver, at unit DC gain.
struct ChannelizerDesign {
    std::vector<float> taps;                 ///< as designed or as supplied; the bank pads them to a whole number of branches
    int                designLength = 0;     ///< the length of `taps`
    std::size_t        span         = 0UZ;   ///< channel spacings the prototype covers
    double             stopbandDb   = 0.0;   ///< worst level from the stop edge to Nyquist, relative to unit DC gain
    double             rippleDb     = 0.0;   ///< peak to trough from DC to the pass edge
    bool               ok           = false; ///< false only for a lowpass design that misses its target
};

namespace detail {

struct ChannelizerDesignKey {
    std::size_t channels = 0UZ;
    std::string family;
    double      attenuationDb = 0.;
    double      transition    = 0.;
    double      maxRippleDb   = 0.;
    std::size_t span          = 0UZ;
    std::size_t oversample    = 0UZ;
    std::size_t maxSpan       = 0UZ;

    [[nodiscard]] bool operator==(const ChannelizerDesignKey&) const = default;
};

/// @brief `fir::design::scanEdges` on the design grid, as a pair of stopband peak and passband ripple, in dB.
[[nodiscard]] inline std::pair<double, double> channelizerEdgeScan(const std::vector<float>& taps, double passEdge, double stopEdge) {
    const gr::filter::fir::design::LowpassScan scan = gr::filter::fir::design::scanEdges(taps, passEdge, stopEdge);
    return {scan.stopbandDb, scan.rippleDb};
}

} // namespace detail

/**
 * @brief Designs a channelizer prototype and returns it with its measured stopband and ripple.
 *
 * The lowpass length is the shortest that meets `attenuationDb` and `maxRippleDb`. `root_nyquist` covers `span`
 * channel spacings, and its squared responses sum flat across the bank. `lowpass` is a Kaiser cut off at half a
 * spacing. `boxcar` is M taps of 1/M.
 *
 * @param channels       M
 * @param family         `root_nyquist`, `lowpass` or `boxcar`
 * @param attenuationDb  the stopband target of the lowpass search
 * @param transition     excess bandwidth in channel spacings: the lowpass transition width or the root-Nyquist rolloff
 * @param maxRippleDb    the passband ripple limit of the lowpass search
 * @param span           channel spacings the root-Nyquist prototype covers, 2 to `maxSpan`
 * @param oversample     the commutator stride the bank runs, which sets the alias edge
 * @param maxSpan        the longest root-Nyquist span; `maxSpan * channels` also floors the lowpass search's cap
 *
 * The design is memoized by its parameters; the memo holds the last eight designs.
 *
 * A parameter outside its range or an unknown family is an error.
 */
[[nodiscard]] inline std::expected<ChannelizerDesign, gr::Error> designChannelizerPrototype(std::size_t channels, std::string_view family, double attenuationDb, double transition, std::size_t span = 16UZ, std::size_t oversample = 1UZ, double maxRippleDb = 0.1, std::size_t maxSpan = 64UZ, std::source_location location = std::source_location::current()) {
    if (channels < 2UZ) {
        return std::unexpected(gr::Error(std::format("gr::filter::designChannelizerPrototype: {} channels; at least two", channels), location));
    }
    if (!(transition > 0.) || !(transition < 1.)) {
        return std::unexpected(gr::Error(std::format("gr::filter::designChannelizerPrototype: transition is a fraction of the channel spacing and must lie in (0, 1), got {}", transition), location));
    }
    if (!(attenuationDb > 0.)) {
        return std::unexpected(gr::Error(std::format("gr::filter::designChannelizerPrototype: attenuation_db must be positive, got {}", attenuationDb), location));
    }
    if (family == "root_nyquist" && (span < 2UZ || span > maxSpan)) {
        return std::unexpected(gr::Error(std::format("gr::filter::designChannelizerPrototype: span is {}; the range is 2 to {}", span, maxSpan), location));
    }

    const detail::ChannelizerDesignKey key{channels, std::string(family), attenuationDb, transition, maxRippleDb, span, oversample, maxSpan};

    static std::mutex                                                              mutex;
    static std::vector<std::pair<detail::ChannelizerDesignKey, ChannelizerDesign>> cache;
    {
        const std::lock_guard<std::mutex> lock(mutex);
        for (const auto& [cached, value] : cache) {
            if (cached == key) {
                return value;
            }
        }
    }

    // Decimating by channels/oversample folds every multiple of oversample/channels onto baseband, so that is where
    // energy starts aliasing into a channel's own passband. The stop edge is a property of the stride, not only of
    // the channel count: oversampling by two moves it a whole spacing further out, where a prototype is far lower.
    const double spacing  = 1. / static_cast<double>(channels);
    const double passEdge = 0.5 * (1. - transition) * spacing;
    const double stopEdge = std::min(0.5, static_cast<double>(oversample) * spacing - passEdge);

    ChannelizerDesign out;
    if (family == "boxcar") {
        out.taps = std::vector<float>(channels, 1.f / static_cast<float>(channels));
        out.span = 1UZ;
    } else if (family == "root_nyquist") {
        // The root-Nyquist length is `span` spacings; `stopbandDb` reports what it achieves.
        out.span = span;
        out.taps = gr::filter::fir::design::rootRaisedCosine(span * channels - 1UZ, static_cast<double>(channels), transition, 1.);
    } else if (family == "lowpass") {
        const auto build    = [&](std::size_t n) { return gr::filter::fir::design::kaiserLowpass(n, 0.5 * spacing, attenuationDb); };
        const auto delivers = [&](std::size_t n) {
            const auto [stopbandDb, rippleDb] = detail::channelizerEdgeScan(build(n), passEdge, stopEdge);
            return stopbandDb <= -attenuationDb && rippleDb <= maxRippleDb;
        };
        // The shortest length is found by bisection. The upper bound is at least 1024 taps and four times the estimate.
        std::size_t       use = gr::filter::fir::design::kaiserLength(attenuationDb, stopEdge - passEdge) | 1UZ;
        const std::size_t cap = std::max({maxSpan * channels, 4UZ * use, 1024UZ});
        while (use < cap && !delivers(use)) {
            use = std::min(cap, 2UZ * use) | 1UZ;
        }
        std::size_t low = 5UZ;
        while (low + 2UZ <= use) {
            const std::size_t mid = ((low + use) / 2UZ) | 1UZ;
            if (mid >= use) {
                break;
            }
            if (delivers(mid)) {
                use = mid;
            } else {
                low = mid + 2UZ;
            }
        }
        out.taps = build(use);
        out.span = (out.taps.size() + channels - 1UZ) / channels;
    } else {
        return std::unexpected(gr::Error(std::format("gr::filter::designChannelizerPrototype: prototype must be 'root_nyquist', 'lowpass' or 'boxcar', got '{}'", family), location));
    }

    out.designLength                       = static_cast<int>(out.taps.size());
    std::tie(out.stopbandDb, out.rippleDb) = detail::channelizerEdgeScan(out.taps, passEdge, stopEdge);
    // Only the lowpass family has a target. `ok` is false only for a lowpass that misses it.
    out.ok = family != "lowpass" || (out.stopbandDb <= -attenuationDb && out.rippleDb <= maxRippleDb);

    return fir::design::detail::keepDesign<fir::design::detail::kDesignMemoEntries>(mutex, cache, key, std::move(out));
}

/**
 * @brief Measures a supplied prototype on the terms a designed one is measured.
 *
 * `ok` is true. The figures are relative to the prototype's DC gain. Leading and trailing zero taps are left out of
 * the scan. The scan assumes a symmetric prototype. An asymmetric one is measured as if it were symmetric.
 *
 * Fewer than two channels, an empty prototype or one with no DC gain is an error.
 */
[[nodiscard]] inline std::expected<ChannelizerDesign, gr::Error> measureChannelizerPrototype(std::span<const float> taps, std::size_t channels, double transition, std::size_t oversample = 1UZ, std::source_location location = std::source_location::current()) {
    if (channels < 2UZ) {
        return std::unexpected(gr::Error(std::format("gr::filter::measureChannelizerPrototype: {} channels; at least two", channels), location));
    }
    if (taps.empty()) {
        return std::unexpected(gr::Error("gr::filter::measureChannelizerPrototype: the prototype is empty", location));
    }
    const auto   first  = std::ranges::find_if(taps, [](float tap) { return tap != 0.f; });
    const auto   last   = std::ranges::find_if(taps | std::views::reverse, [](float tap) { return tap != 0.f; }).base();
    const double dcGain = first < last ? std::accumulate(first, last, 0.0, [](double sum, float tap) { return sum + static_cast<double>(tap); }) : 0.0;
    if (!(std::abs(dcGain) > 0.0) || !std::isfinite(dcGain)) {
        return std::unexpected(gr::Error("gr::filter::measureChannelizerPrototype: the taps have no DC gain", location));
    }

    const double spacing  = 1. / static_cast<double>(channels);
    const double passEdge = 0.5 * (1. - transition) * spacing;
    const double stopEdge = std::min(0.5, static_cast<double>(oversample) * spacing - passEdge);

    const std::span<const float> trimmed(first, last);
    std::vector<float>           normalized(trimmed.size());
    std::ranges::transform(trimmed, normalized.begin(), [dcGain](float tap) { return static_cast<float>(static_cast<double>(tap) / dcGain); });

    ChannelizerDesign out;
    out.taps                               = std::vector<float>(taps.begin(), taps.end());
    out.designLength                       = static_cast<int>(out.taps.size());
    out.span                               = (out.taps.size() + channels - 1UZ) / channels;
    std::tie(out.stopbandDb, out.rippleDb) = detail::channelizerEdgeScan(normalized, passEdge, stopEdge);
    out.ok                                 = true;
    return out;
}

/**
 * @brief The analysis half of a polyphase filter bank: one stream in, `M` channels out.
 *
 * Channel `k` is the input mixed down by `exp(-j*2*pi*k*n/M)`, filtered by the prototype and decimated by the
 * commutator's stride `S = M / oversample`:
 *
 *     y_k[n] = exp(-j*2*pi*k*n*S/M) * sum_r exp(+j*2*pi*k*r/M) * v_r[n]
 *     v_r[n] = sum_j h[j*M + r] * x[n*S - j*M - r]
 *
 * The inner sum is a branch filter and the outer one an unnormalized inverse discrete Fourier transform across the
 * branches. The newest input sample belongs to branch zero, and the branch index counts backwards through the step's
 * samples.
 *
 * Critically sampled, the leading phase is one at every step. Oversampled by two, the leading phase is `(-1)^k`.
 *
 * Channel `k` is centered at `k*fs/M`, with indices above `M/2` reading as the negative frequencies in the usual
 * transform order.
 */
template<std::floating_point F>
struct PolyphaseChannelizer {
    using Complex = std::complex<F>;

    /// The widest bank accepted.
    static constexpr std::size_t kMaxChannels = 256UZ;

    std::vector<float>   _prototype{}; ///< zero-padded to a whole number of branches
    std::vector<Complex> _history{};   ///< the last N input samples, newest first, stored twice
    std::vector<Complex> _branch{};    ///< v_r for the current step
    std::size_t          _channels   = 1UZ;
    std::size_t          _oversample = 1UZ;
    std::size_t          _length     = 1UZ; ///< N, the padded prototype length
    std::size_t          _position   = 0UZ; ///< slot holding the newest input; walks backwards
    std::size_t          _phase      = 0UZ; ///< (n*S) mod M

    gr::algorithm::FFT<Complex, Complex, gr::algorithm::Direction::Backward> _inverse{};

    /**
     * @brief Sizes the bank and installs its prototype.
     *
     * @param nChannels  M, 2 to kMaxChannels
     * @param prototype  the lowpass every branch is cut from; zero-padded to a multiple of M
     * @param oversample 1 for critical sampling, 2 for a commutator stride of M/2
     *
     * A parameter outside its range returns an error that names it, and the bank is unchanged.
     */
    [[nodiscard]] std::expected<void, gr::Error> configure(std::size_t nChannels, std::span<const float> prototype, std::size_t oversample = 1UZ, std::source_location location = std::source_location::current()) {
        if (nChannels < 2UZ || nChannels > kMaxChannels) {
            return std::unexpected(gr::Error(std::format("gr::filter::PolyphaseChannelizer: {} channels; the range is 2 to {}", nChannels, kMaxChannels), location));
        }
        if (oversample != 1UZ && oversample != 2UZ) {
            return std::unexpected(gr::Error(std::format("gr::filter::PolyphaseChannelizer: oversample is 1 or 2, got {}", oversample), location));
        }
        if (oversample == 2UZ && (nChannels % 2UZ) != 0UZ) {
            return std::unexpected(gr::Error(std::format("gr::filter::PolyphaseChannelizer: an oversampled bank needs an even channel count, got {}", nChannels), location));
        }
        if (prototype.empty()) {
            return std::unexpected(gr::Error("gr::filter::PolyphaseChannelizer: the prototype is empty", location));
        }

        _channels   = nChannels;
        _oversample = oversample;
        _prototype  = detail::padToBranches(prototype, nChannels);
        _length     = _prototype.size();
        _history.assign(2UZ * _length, Complex{});
        _branch.assign(nChannels, Complex{});
        _inverse = {};
        reset();
        return {};
    }

    void reset() noexcept {
        std::ranges::fill(_history, Complex{});
        _position = 0UZ;
        _phase    = 0UZ;
    }

    [[nodiscard]] std::size_t            channels() const noexcept { return _channels; }
    [[nodiscard]] std::size_t            oversample() const noexcept { return _oversample; }
    [[nodiscard]] std::size_t            stride() const noexcept { return _channels / _oversample; }
    [[nodiscard]] std::size_t            prototypeLength() const noexcept { return _length; }
    [[nodiscard]] std::span<const float> prototype() const noexcept { return _prototype; }

    /// Accepts one input sample.
    void push(Complex sample) noexcept {
        _position                     = _position == 0UZ ? _length - 1UZ : _position - 1UZ;
        _history[_position]           = sample;
        _history[_position + _length] = sample;
    }

    /**
     * @brief Consumes `stride()` input samples and writes one sample per channel.
     *
     * @param input  at least `stride()` samples, oldest first
     * @param out    at least `channels()` slots
     */
    void step(std::span<const Complex> input, std::span<Complex> out) {
        const std::size_t nChannels = _channels;
        const std::size_t advance   = stride();
        for (std::size_t i = 0UZ; i < advance; ++i) {
            push(input[i]);
        }

        std::ranges::fill(_branch, Complex{});
        for (std::size_t j = 0UZ; j * nChannels < _length; ++j) {
            const std::size_t tapBase     = j * nChannels;
            const std::size_t historyBase = _position + tapBase;
            for (std::size_t r = 0UZ; r < nChannels; ++r) {
                _branch[r] += static_cast<F>(_prototype[tapBase + r]) * _history[historyBase + r];
            }
        }

        _inverse.compute(_branch, out.first(nChannels));

        if (_phase != 0UZ) {                 // critical sampling never reaches this: the commutator advances a whole M
            if (2UZ * _phase == nChannels) { // oversampled by two, the only phase besides zero
                for (std::size_t k = 1UZ; k < nChannels; k += 2UZ) {
                    out[k] = -out[k]; // exp(-j*pi*k) is (-1)^k, so the odd channels change sign
                }
            } else {
                constexpr double twoPi = 2. * std::numbers::pi_v<double>;
                for (std::size_t k = 0UZ; k < nChannels; ++k) {
                    const double angle = -twoPi * static_cast<double>(k * _phase % nChannels) / static_cast<double>(nChannels);
                    const auto   turn  = Complex(static_cast<F>(std::cos(angle)), static_cast<F>(std::sin(angle)));
                    out[k] *= turn;
                }
            }
        }
        _phase = (_phase + advance) % nChannels;
    }
};

/**
 * @brief The synthesis half: `M` channel streams in, one stream out, the transpose of the analysis bank.
 *
 * Each step takes one sample per channel, undoes the analysis bank's leading phase, transforms across channels, and
 * adds the result into an overlap-add line the prototype spans; the `stride()` oldest samples of that line are then
 * complete and leave. A synthesizer fed an analyzer's channels reproduces the input delayed by `(N-1)/S` steps and
 * scaled by the cascade gain the prototype gives it.
 */
template<std::floating_point F>
struct PolyphaseSynthesizer {
    using Complex = std::complex<F>;

    static constexpr std::size_t kMaxChannels = PolyphaseChannelizer<F>::kMaxChannels;

    std::vector<float>   _prototype{};
    std::vector<Complex> _channelIn{};   ///< the step's channel samples, phase undone
    std::vector<Complex> _branch{};      ///< u_r for the current step
    std::vector<Complex> _accumulator{}; ///< the overlap-add line, oldest first
    std::size_t          _channels   = 1UZ;
    std::size_t          _oversample = 1UZ;
    std::size_t          _length     = 1UZ;
    std::size_t          _phase      = 0UZ;

    gr::algorithm::FFT<Complex, Complex> _forward{};

    /// @brief Sizes the bank and installs its prototype, on the ranges of `PolyphaseChannelizer::configure`.
    [[nodiscard]] std::expected<void, gr::Error> configure(std::size_t nChannels, std::span<const float> prototype, std::size_t oversample = 1UZ, std::source_location location = std::source_location::current()) {
        if (nChannels < 2UZ || nChannels > kMaxChannels) {
            return std::unexpected(gr::Error(std::format("gr::filter::PolyphaseSynthesizer: {} channels; the range is 2 to {}", nChannels, kMaxChannels), location));
        }
        if (oversample != 1UZ && oversample != 2UZ) {
            return std::unexpected(gr::Error(std::format("gr::filter::PolyphaseSynthesizer: oversample is 1 or 2, got {}", oversample), location));
        }
        if (oversample == 2UZ && (nChannels % 2UZ) != 0UZ) {
            return std::unexpected(gr::Error(std::format("gr::filter::PolyphaseSynthesizer: an oversampled bank needs an even channel count, got {}", nChannels), location));
        }
        if (prototype.empty()) {
            return std::unexpected(gr::Error("gr::filter::PolyphaseSynthesizer: the prototype is empty", location));
        }

        _channels   = nChannels;
        _oversample = oversample;
        _prototype  = detail::padToBranches(prototype, nChannels);
        _length     = _prototype.size();
        _channelIn.assign(nChannels, Complex{});
        _branch.assign(nChannels, Complex{});
        _accumulator.assign(_length, Complex{});
        _forward = {};
        reset();
        return {};
    }

    void reset() noexcept {
        std::ranges::fill(_accumulator, Complex{});
        _phase = 0UZ;
    }

    [[nodiscard]] std::size_t            channels() const noexcept { return _channels; }
    [[nodiscard]] std::size_t            oversample() const noexcept { return _oversample; }
    [[nodiscard]] std::size_t            stride() const noexcept { return _channels / _oversample; }
    [[nodiscard]] std::size_t            prototypeLength() const noexcept { return _length; }
    [[nodiscard]] std::span<const float> prototype() const noexcept { return _prototype; }

    /// The steps of output that are still inside the prototype when the last channel sample arrives.
    [[nodiscard]] std::size_t pipelineDelay() const noexcept { return (_length - 1UZ) / stride(); }

    /**
     * @brief Consumes one sample per channel and writes `stride()` output samples, oldest first.
     */
    void step(std::span<const Complex> channelsIn, std::span<Complex> out) {
        const std::size_t nChannels = _channels;
        const std::size_t advance   = stride();

        if (_phase == 0UZ) {
            std::ranges::copy(channelsIn.first(nChannels), _channelIn.begin());
        } else {
            constexpr double twoPi = 2. * std::numbers::pi_v<double>;
            for (std::size_t k = 0UZ; k < nChannels; ++k) {
                const double angle = twoPi * static_cast<double>(k * _phase % nChannels) / static_cast<double>(nChannels);
                _channelIn[k]      = channelsIn[k] * Complex(static_cast<F>(std::cos(angle)), static_cast<F>(std::sin(angle)));
            }
        }
        _forward.compute(_channelIn, _branch);

        const F scale = static_cast<F>(1. / static_cast<double>(nChannels));
        for (std::size_t j = 0UZ; j * nChannels < _length; ++j) {
            const std::size_t tapBase = j * nChannels;
            for (std::size_t r = 0UZ; r < nChannels; ++r) {
                const std::size_t d = tapBase + r;
                _accumulator[_length - 1UZ - d] += (scale * static_cast<F>(_prototype[d])) * _branch[r];
            }
        }

        std::ranges::copy(_accumulator.begin(), _accumulator.begin() + static_cast<std::ptrdiff_t>(advance), out.begin());
        std::ranges::move(_accumulator.begin() + static_cast<std::ptrdiff_t>(advance), _accumulator.end(), _accumulator.begin());
        std::ranges::fill(_accumulator.end() - static_cast<std::ptrdiff_t>(advance), _accumulator.end(), Complex{});

        _phase = (_phase + advance) % nChannels;
    }
};

} // namespace gr::filter

#endif // GNURADIO_ALGORITHM_POLYPHASE_CHANNELIZER_HPP
