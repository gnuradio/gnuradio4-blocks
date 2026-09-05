#ifndef GNURADIO_DIGITAL_PREAMBLE_CORRELATOR_HPP
#define GNURADIO_DIGITAL_PREAMBLE_CORRELATOR_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <limits>
#include <print>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::digital {

GR_REGISTER_BLOCK(gr::blocks::digital::PreambleCorrelator, [T], [float])

template<std::floating_point F>
struct PreambleCorrelator : Block<PreambleCorrelator<F>> {
    using Description = Doc<R""(
@brief Correlates the stream against a known complex sequence and tags each peak with its phase, amplitude and sub-sample timing.

The symbol-domain detector, for a burst whose sync word is a known complex sequence rather than a known bit pattern.
It is the natural upstream of a Costas loop and a symbol synchronizer: the tag carries `phase_est`, `amp_est` and a
`trigger_offset` holding the sub-sample timing.

`out` lags `in` by `N` samples, `N` being the sequence length, and every tag sits at the sync word's first symbol on
the output; a consumer wanting the payload start adds `N`, which is what `PacketFramer`'s `trigger_to_header` is for.
The debug port `corr` shares that alignment, and both ports carry the same sequence: the detections and the forwarded
input tags, merged and in offset order.

`absolute` tests `|r|^2` against a fraction of the noiseless peak, which is scale-dependent and makes an upstream AGC
a stated precondition; `relative` tests against a multiple of the running mean and is scale-free. A detection is
followed by `round(samples_per_symbol)` samples of dead time.

The label the emitted tags carry is the `trigger_label` setting, deliberately not named for the `trigger_name` key it
is written under: a setting sharing a reserved key's name is driven by every passing tag of that key, so an upstream
detector's tags would silently retune this block's own label.
)"">;

    PortIn<std::complex<F>>  in;
    PortOut<std::complex<F>> out;
    PortOut<float, Optional> corr;

    Annotated<std::vector<F>, "sequence", Doc<"the sync sequence as interleaved re,im pairs; there is no default">, Visible>        sequence{};
    Annotated<F, "threshold", Doc<"'absolute': a fraction of the noiseless peak. 'relative': a multiple of the running mean, > 1">> threshold          = static_cast<F>(0.9);
    Annotated<std::string, "threshold_mode", Doc<"'absolute' (needs an AGC) or 'relative' (scale-free)">>                           threshold_mode     = std::string("absolute");
    Annotated<F, "samples_per_symbol", Unit<"samples">, Doc<"dead time after a detection, in samples">>                             samples_per_symbol = F{1};
    Annotated<std::string, "trigger_label", Doc<"the label written under the trigger_name key of the emitted tags">>                trigger_label      = std::string("preamble");

    GR_MAKE_REFLECTABLE(PreambleCorrelator, in, out, corr, sequence, threshold, threshold_mode, samples_per_symbol, trigger_label);

    std::vector<std::complex<F>>                        _reference;   ///< conj(s), so the correlation is a plain dot product
    std::vector<std::complex<F>>                        _window;      ///< `_carry` followed by this call's input
    std::vector<std::complex<F>>                        _carry;       ///< the last `N` inputs, which are also the delay line
    std::vector<F>                                      _magnitude;   ///< `|r|^2` for each input index of this call
    std::vector<std::pair<std::uint64_t, property_map>> _pendingTags; ///< every tag still to publish, at the input index its output carries plus `N`
    F                                                   _energy   = F{1};
    F                                                   _previous = F{0}; ///< `|r|^2` one input before this call
    F                                                   _twoBack  = F{0};
    bool                                                _relative = false;
    bool                                                _carrying = false; ///< whether the index before this call is still a candidate
    bool                                                _warned   = false;
    std::size_t                                         _deadTime = 1UZ;
    std::uint64_t                                       _resumeAt = 0ULL; ///< absolute input index the dead time ends at

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void reset() {
        std::ranges::fill(_carry, std::complex<F>{});
        _previous = F{0};
        _twoBack  = F{0};
        _carrying = false;
        _resumeAt = 0ULL;
        _pendingTags.clear();
    }

    void rebuild() {
        if (sequence.value.empty() || sequence.value.size() % 2UZ != 0UZ) {
            throw gr::exception(std::format("sequence is the sync word as interleaved re,im pairs and must be a non-empty even-length list, got {} values", sequence.value.size()));
        }
        if (!(threshold > F{0})) {
            throw gr::exception(std::format("threshold must be strictly positive, got {}", static_cast<double>(threshold.value)));
        }
        if (threshold_mode != "absolute" && threshold_mode != "relative") {
            throw gr::exception(std::format("threshold_mode must be 'absolute' or 'relative', got '{}'", threshold_mode.value));
        }
        if (threshold_mode == "relative" && !(threshold > F{1})) {
            throw gr::exception(std::format("a 'relative' threshold is a multiple of the mean and must exceed 1, got {}", static_cast<double>(threshold.value)));
        }
        if (!(samples_per_symbol >= F{1})) {
            throw gr::exception(std::format("samples_per_symbol is the dead time in samples and must be at least 1, got {}", static_cast<double>(samples_per_symbol.value)));
        }

        const std::size_t length = sequence.value.size() / 2UZ;
        _reference.resize(length);
        double energy = 0.0;
        for (std::size_t k = 0UZ; k < length; ++k) {
            const std::complex<F> symbol = std::complex<F>(sequence.value[2UZ * k], sequence.value[2UZ * k + 1UZ]);
            _reference[k]                = std::conj(symbol);
            energy += static_cast<double>(std::norm(symbol));
        }
        if (!(energy > 0.0)) {
            throw gr::exception("sequence has zero energy, so no threshold on the correlation means anything");
        }
        _energy   = static_cast<F>(energy);
        _relative = threshold_mode == "relative";
        _deadTime = static_cast<std::size_t>(std::lround(static_cast<double>(samples_per_symbol.value)));
        _carry.assign(length, std::complex<F>{});
        reset();

        if (length > 64UZ && !_warned) {
            _warned = true;
            std::println(stderr, "gr::blocks::digital::PreambleCorrelator: a {}-symbol sequence is {} complex multiply-accumulates per input sample; an overlap-save FFT filter is the cheaper form above about this length", length, length);
        }
    }

    /// @brief The sequence length, which is exactly the delay from `in` to `out`.
    [[nodiscard]] std::size_t delaySamples() const noexcept { return _reference.size(); }

    /**
     * @brief Take in every arriving input tag at the output index its sample reaches, which is `N` later.
     *
     * Publication waits for `processBulk`, which is where the block's own detections are found: a port takes one
     * ordered sequence and not two interleaved ones, so the two sources are merged there rather than published as
     * each arises.
     */
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& /*outputSpans*/, std::size_t processedIn) {
        const auto delay = static_cast<std::uint64_t>(_reference.size());
        gr::for_each_reader_span(
            [this, processedIn, delay](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const auto& [relIndex, tagMap] : span.tags(processedIn)) {
                    if (relIndex < 0) { // a tag from before this window is one this block has already placed
                        continue;
                    }
                    _pendingTags.emplace_back(static_cast<std::uint64_t>(span.streamIndex) + static_cast<std::uint64_t>(relIndex) + delay, tagMap.get());
                }
            },
            inputSpans);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& corrSpan) {
        if (_reference.empty()) { // the block is inert rather than correlating against nothing
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            corrSpan.publish(0UZ);
            return work::Status::ERROR;
        }
        const bool        corrConnected = corrSpan.isConnected;
        const std::size_t nSamples      = corrConnected ? std::min({inSpan.size(), outSpan.size(), corrSpan.size()}) : std::min(inSpan.size(), outSpan.size());
        if (nSamples == 0UZ) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            corrSpan.publish(0UZ);
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        const std::size_t   length = _reference.size();
        const std::uint64_t base   = static_cast<std::uint64_t>(inSpan.streamIndex); // the port's own numbering, which is also what the arriving tags were placed in

        _window.assign(_carry.begin(), _carry.end());
        _window.insert(_window.end(), inSpan.begin(), std::next(inSpan.begin(), static_cast<std::ptrdiff_t>(nSamples)));

        _magnitude.resize(nSamples);
        double sum = 0.0;
        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            F real = F{0};
            F imag = F{0};
            for (std::size_t k = 0UZ; k < length; ++k) { // the window ending at input base+i, against conj(s)
                const std::complex<F> sample = _window[i + 1UZ + k];
                const std::complex<F> tap    = _reference[k];
                real += sample.real() * tap.real() - sample.imag() * tap.imag();
                imag += sample.real() * tap.imag() + sample.imag() * tap.real();
            }
            _magnitude[i] = real * real + imag * imag;
            sum += static_cast<double>(_magnitude[i]);
            outSpan[i] = _window[i]; // the N-sample lag, straight out of the carry
        }
        if (corrConnected) {
            for (std::size_t i = 0UZ; i < nSamples; ++i) {
                corrSpan[i] = static_cast<float>(i == 0UZ ? _previous : _magnitude[i - 1UZ]);
            }
        }

        const F level = _relative ? static_cast<F>(threshold * static_cast<F>(sum / static_cast<double>(nSamples))) : static_cast<F>(threshold * _energy * _energy);

        if (_carrying) { // the index before this call could only be confirmed once its successor arrived
            examine(base - 1ULL, _twoBack, _previous, _magnitude[0UZ], level, base);
        }
        for (std::size_t i = 0UZ; i + 1UZ < nSamples; ++i) {
            examine(base + i, i == 0UZ ? _previous : _magnitude[i - 1UZ], _magnitude[i], _magnitude[i + 1UZ], level, base);
        }

        _twoBack  = nSamples >= 2UZ ? _magnitude[nSamples - 2UZ] : _previous;
        _previous = _magnitude[nSamples - 1UZ];
        _carrying = true;
        _carry.assign(_window.end() - static_cast<std::ptrdiff_t>(length), _window.end());

        releaseTags(base, nSamples, outSpan, corrSpan);

        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        corrSpan.publish(corrConnected ? nSamples : 0UZ);
        return work::Status::OK;
    }

    /// @brief Test the triple `(a, b, c)` around input index @p at, and tag the sync-word start if it is a peak.
    void examine(std::uint64_t at, F a, F b, F c, F level, std::uint64_t base) {
        if (at < _resumeAt || !(b > level) || !(b >= a) || !(b > c)) {
            return;
        }

        const F denominator = a - F{2} * b + c;
        F       offset      = F{0};
        if (std::abs(denominator) > std::numeric_limits<F>::epsilon() * b) {
            offset = std::clamp(F{0.5} * (a - c) / denominator, F{-0.5}, F{0.5});
        }

        const std::size_t windowStart = static_cast<std::size_t>(at + 1ULL - base); // 0 for the index carried in from the previous call
        const std::size_t length      = _reference.size();
        std::complex<F>   peak{};
        for (std::size_t k = 0UZ; k < length; ++k) { // the phase needs the complex value, not its magnitude
            peak += _window[windowStart + k] * _reference[k];
        }

        // the sync word starts at input `at + 1 - N`, which the `N`-sample lag puts at output `at + 1`
        _pendingTags.emplace_back(at + 1ULL, property_map{{gr::tag::TRIGGER_NAME.shortKey(), trigger_label.value},                                                                              //
                                                 {gr::tag::TRIGGER_OFFSET.shortKey(), static_cast<float>(offset)},                                                                              //
                                                 {gr::tag::TRIGGER_META_INFO.shortKey(), property_map{{"phase_est", static_cast<float>(std::atan2(peak.imag(), peak.real()))},                  //
                                                                                             {"corr_mag", static_cast<float>(b)},                                                               //
                                                                                             {"amp_est", static_cast<float>(std::sqrt(static_cast<double>(b)) / static_cast<double>(_energy))}, //
                                                                                             {"sps", static_cast<float>(samples_per_symbol.value)}}}});
        _resumeAt = at + 1ULL + static_cast<std::uint64_t>(_deadTime);
    }

    /**
     * @brief Publish every tag whose output this call produced, in one ordered sequence, and hold the rest.
     *
     * The forwarded input tags are taken in before the call and the detections are found during it, so the two are
     * sorted together here rather than published as they arise: a port rejects an index below the last one it was
     * given. The deferral is decided once, against the outputs this call produced, and every connected port publishes
     * the same survivors: what is held belongs to the block and not to any one port.
     */
    void releaseTags(std::uint64_t base, std::size_t made, OutputSpanLike auto&... spans) {
        if (_pendingTags.empty()) {
            return;
        }
        std::ranges::stable_sort(_pendingTags, std::ranges::less{}, &std::pair<std::uint64_t, property_map>::first);

        const std::uint64_t end   = base + static_cast<std::uint64_t>(made);
        const std::size_t   ready = static_cast<std::size_t>(std::ranges::distance(_pendingTags.begin(), std::ranges::find_if(_pendingTags, [end](const auto& tag) { return tag.first >= end; })));
        for (std::size_t k = 0UZ; k < ready; ++k) {
            const std::size_t offset = static_cast<std::size_t>(_pendingTags[k].first > base ? _pendingTags[k].first - base : 0ULL);
            (spans.publishTag(_pendingTags[k].second, offset), ...);
        }
        _pendingTags.erase(_pendingTags.begin(), _pendingTags.begin() + static_cast<std::ptrdiff_t>(ready));
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_PREAMBLE_CORRELATOR_HPP
