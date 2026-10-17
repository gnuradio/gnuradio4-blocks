#ifndef GNURADIO_OFDM_SCHMIDL_COX_SYNC_HPP
#define GNURADIO_OFDM_SCHMIDL_COX_SYNC_HPP

#include <algorithm>
#include <bit>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <initializer_list>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/signal/Phasor.hpp>

#include <gnuradio-4.0/ofdm/Numerology.hpp>

namespace gr::blocks::ofdm {

GR_REGISTER_BLOCK(gr::blocks::ofdm::SchmidlCoxSync)

namespace detail {

/// @brief The validator of `threshold`: true strictly between 0 and 1.
inline constexpr auto kThresholdInRange = [](float value) { return value > 0.f && value < 1.f; };

/// @brief The validator of `r_floor`: true above 0.
inline constexpr auto kRFloorPositive = [](float value) { return value > 0.f; };

/// @brief Converts @p value to `std::size_t`. The caller has reduced or checked it below a `std::size_t` bound.
template<std::unsigned_integral T>
[[nodiscard]] constexpr std::size_t sizeFrom(T value) noexcept {
    if constexpr (std::is_same_v<T, std::size_t>) {
        return value;
    } else {
        return static_cast<std::size_t>(value);
    }
}

} // namespace detail

/**
 * @brief Finds OFDM frames by a preamble whose halves repeat, and estimates the fractional carrier frequency offset.
 *
 * In an OFDM receiver the block takes the complex baseband stream and feeds `CpRemove`. Each frame opens with a
 * preamble symbol whose time-domain halves repeat, behind a cyclic prefix of `cp_len` samples.
 * `gr::ofdm::schmidlCoxPreamble` builds one. The block passes the stream through, one sample out per sample in, delayed
 * by `delaySamples()`, which is `3*fft_len/2 + cp_len - 1`. It tags the first sample of each frame it finds, the
 * preamble's first prefix sample, with `trigger_name` set to `trigger_label` and `trigger_offset` 0. The tag's
 * `trigger_meta_info` holds `cfo_fractional`, the carrier offset in subcarrier spacings, and `plateau_samples`, the
 * width of the detection.
 *
 * Input tags leave on their own samples, delayed with them. The block removes the trigger keys from an input tag whose
 * `trigger_name` equals `trigger_label` and forwards the tag's other keys. The last `delaySamples()` inputs of a stream
 * never reach the output, and their tags leave at the end-of-stream index.
 *
 * With `L = fft_len/2`, the block slides two sums over the stream at a constant cost per sample:
 *
 *     P(d) = sum_{m=0..L-1} conj(r[d+m]) * r[d+m+L]      R(d) = sum_{m=0..L-1} |r[d+m+L]|^2
 *
 * A window `d` is above the threshold when `|P(d)|^2 > threshold * R(d)^2` and `R(d) > r_floor`. At a preamble the
 * windows stay above it for about `cp_len + 1` samples. The block places the trigger `cp_len/2` samples before the
 * midpoint of that run, and `cfo_fractional` is `angle(P)/pi` at the midpoint. The estimate covers offsets below one
 * spacing in magnitude, and a larger offset aliases into that range. A NaN or infinite sample adds nothing to either
 * sum.
 *
 * `threshold`, between 0 and 1, sets how closely the halves must agree. `r_floor`, above 0, is the energy a window
 * needs. The ratio alone reads high where both sums are near zero. After a trigger the block ignores `min_gap` symbols
 * of `cp_len + fft_len` samples each. With `min_gap` 1 or more, a frame triggers once. With `correct_cfo` set the
 * block derotates the output by each estimate from its trigger on. The phase stays continuous across a new estimate,
 * and a constant phase remains after the correction. A change of `correct_cfo` applies from the next output sample.
 * Switched on, it derotates by the estimate of the last trigger output before that sample.
 *
 * `fft_len` is a power of two from 8 to 65536, and `cp_len` is at most `fft_len`. The block refuses a value that breaks
 * either rule. A change of both lengths goes in one application when one alone would break the second rule. A change of
 * `fft_len` or `cp_len` changes the delay and restarts both sums. The delay line keeps its newest inputs, with zeros
 * ahead of them for a longer delay. A change of any other setting applies from the next sample. A reset to defaults
 * empties the delay line. In both cases a held input tag whose sample leaves the delay line goes out on the next
 * output, and a held trigger whose sample leaves counts in `nLate()`.
 *
 * `nTriggers()` counts the frames found. `nLate()` counts the detections the block could not place: the frame began
 * before the stream, or its sample left the delay line. `nShortPlateaus()` counts the runs above the threshold at most
 * `cp_len/2` samples wide, and `nWidePlateaus()` the runs wider than `cp_len + fft_len - 1` samples. The block places
 * no trigger for either. `lastCfoFractional()` returns the most recent estimate. `start()` sets the counters to 0.
 */
struct SchmidlCoxSync : Block<SchmidlCoxSync> {
    using Description = Doc<"Detects OFDM frames by their repeated-half preamble and estimates the fractional carrier frequency offset. The block passes the complex stream through, delayed by delaySamples(), which is 3*fft_len/2 + cp_len - 1 samples. It tags the first sample of each frame with trigger_name and with cfo_fractional in subcarrier spacings, and it can derotate the stream by that estimate">;

    PortIn<Complex>  in;
    PortOut<Complex> out;

    Annotated<gr::Size_t, "fft_len", Visible, Doc<"transform length, a power of two">, detail::FftLengthLimits>                                             fft_len       = 64U;
    Annotated<gr::Size_t, "cp_len", Visible, Doc<"prefix length of the preamble symbol">>                                                                   cp_len        = 16U;
    Annotated<float, "threshold", Visible, Doc<"metric level that opens a plateau, in (0, 1)">, Limits<0.f, 1.f, detail::kThresholdInRange>>                threshold     = 0.6f;
    Annotated<gr::Size_t, "min_gap", Visible, Unit<"symbols">, Doc<"dead time after a trigger, in cp_len + fft_len sample symbols">>                        min_gap       = 1U;
    Annotated<bool, "correct_cfo", Visible, Doc<"derotate the output by the estimate from the trigger onward">>                                             correct_cfo   = true;
    Annotated<float, "r_floor", Doc<"least received energy a trigger needs">, Limits<0.f, std::numeric_limits<float>::infinity(), detail::kRFloorPositive>> r_floor       = 1e-9f;
    Annotated<std::string, "trigger_label", Doc<"trigger_name of the emitted tags">>                                                                        trigger_label = std::string("ofdm_frame");

    GR_MAKE_REFLECTABLE(SchmidlCoxSync, in, out, fft_len, cp_len, threshold, min_gap, correct_cfo, r_floor, trigger_label);

    /// A tag held until the output reaches its sample. On the block's own detections, `increment` re-tunes the
    /// derotation at the trigger's sample, with `correct_cfo` set or not.
    struct PendingTag {
        std::uint64_t         at{0ULL}; ///< the output index, counted in 64 bits on every target
        property_map          map{};
        std::optional<double> increment{};
        bool                  ownDetection{false};
    };

    /// The settings the block computes on.
    struct Accepted {
        gr::Size_t  fftLength  = 0U;
        gr::Size_t  cpLength   = 0U;
        float       threshold  = 0.f;
        gr::Size_t  minGap     = 0U;
        bool        correctCfo = false;
        float       rFloor     = 0.f;
        std::string label{};
    };

    Accepted                  _accepted = settingsAfter(Accepted{}, [](std::string_view) { return true; });
    gr::signal::Phasor<float> _phasor{};

    std::vector<Complex>    _history{}; ///< the last `delaySamples()` inputs, which form the delay line
    std::vector<Complex>    _window{};  ///< `_history` followed by this call's input
    std::vector<Complex>    _pRing{};   ///< P(d) for recent d; the falling edge reads the midpoint's value here
    std::vector<PendingTag> _pending{};

    /// The last `L` per-sample terms of each sum. `conj(r[n-L]) * r[n]` is the term P(d) gains at sample `n` and loses
    /// `L` samples later. `|r[n]|^2` has the same relation to R(d). Keeping the terms halves the multiplies of the
    /// sliding update. `L` is a power of two, and the index is a mask.
    std::vector<double> _termRe{};
    std::vector<double> _termIm{};
    std::vector<double> _termNorm{};

    double        _pRe      = 0.; ///< the running correlation, held as two scalars
    double        _pIm      = 0.;
    double        _r        = 0.;  ///< the running energy of the later half
    std::size_t   _half     = 0UZ; ///< L = fft_len/2
    std::size_t   _delay    = 0UZ; ///< 3*fft_len/2 + cp_len - 1
    std::size_t   _ring     = 0UZ; ///< retained P history, a power of two
    std::size_t   _mask     = 0UZ;
    std::uint64_t _deadTime = 0ULL; ///< min_gap symbols, in samples
    std::size_t   _midShift = 0UZ;  ///< cp_len/2, from the plateau midpoint back to the frame's first sample
    bool          _above    = false;
    std::uint64_t _riseAt   = 0ULL;
    std::size_t   _callAt   = 0UZ;  ///< the stream index of the current call's first input
    std::uint64_t _wideAt   = 0ULL; ///< `_callAt` counted in 64 bits on every target
    std::uint64_t _resumeAt = 0ULL; ///< the 64-bit sample index where the dead time ends
    std::uint64_t _triggers = 0ULL;
    std::uint64_t _late     = 0ULL;
    std::uint64_t _short    = 0ULL; ///< crossings too narrow to be a preamble's plateau
    std::uint64_t _wide     = 0ULL; ///< crossings wider than any preamble's plateau
    std::size_t   _minWidth = 1UZ;
    std::size_t   _maxWidth = 1UZ; ///< cp_len + fft_len - 1
    double        _lastCfo  = 0.;
    double        _level    = 0.; ///< the accepted `threshold`
    double        _floor    = 0.; ///< the accepted `r_floor`

    /// @brief The number of samples the output lags the input.
    [[nodiscard]] std::size_t delaySamples() const noexcept { return _delay; }
    /// @brief Frames detected over the run.
    [[nodiscard]] std::uint64_t nTriggers() const noexcept { return _triggers; }
    /// @brief Detections not placed: the frame began before the stream, or its sample left the delay line first.
    [[nodiscard]] std::uint64_t nLate() const noexcept { return _late; }
    /// @brief Threshold crossings too narrow to be a preamble's plateau.
    [[nodiscard]] std::uint64_t nShortPlateaus() const noexcept { return _short; }
    /// @brief Threshold crossings wider than any preamble's plateau.
    [[nodiscard]] std::uint64_t nWidePlateaus() const noexcept { return _wide; }
    /// @brief The fractional offset of the most recent trigger, in subcarrier spacings.
    [[nodiscard]] double lastCfoFractional() const noexcept { return _lastCfo; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        take(settingsAfter(_accepted, [&newSettings](std::string_view key) { return newSettings.contains(key); }));
    }

    /// @brief Resets the estimator, drops the held tags and restarts the counters. `settingsChanged()` builds the
    /// estimator when a setting takes a value other than its default. For a block left at its defaults, `start()` builds
    /// it.
    void start() {
        if (_history.empty()) {
            take(_accepted);
        }
        _pending.clear();
        _triggers = 0ULL;
        _late     = 0ULL;
        _short    = 0ULL;
        _wide     = 0ULL;
        _lastCfo  = 0.;
        _callAt   = 0UZ;
        _wideAt   = 0ULL;
        reset();
    }

    /// @brief Restarts the delay line, the sums, the dead time and the derotation. Each held input tag leaves on the
    /// next output, and each held detection counts in `nLate()`.
    void reset() {
        std::ranges::fill(_history, Complex{});
        std::ranges::fill(_termRe, 0.);
        std::ranges::fill(_termIm, 0.);
        std::ranges::fill(_termNorm, 0.);
        _pRe      = 0.;
        _pIm      = 0.;
        _r        = 0.;
        _above    = false;
        _riseAt   = 0ULL;
        _resumeAt = 0ULL;
        // Every held tag sits on a sample of the delay line. At index 0 an input tag leaves on the next output.
        _late += static_cast<std::uint64_t>(std::ranges::count_if(_pending, &PendingTag::ownDetection));
        std::erase_if(_pending, [](const PendingTag& tag) { return tag.ownDetection; });
        for (PendingTag& tag : _pending) {
            tag.at = 0ULL;
        }
        _phasor.configure(0., 0.);
    }

    /// @brief Holds each arriving input tag for the output index of its sample, `delaySamples()` later.
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        if (processedIn == 0UZ) {
            releaseAtEndOfStream(inputSpans, outputSpans);
            return;
        }
        gr::for_each_reader_span(
            [this, processedIn](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const auto& [relIndex, tagMap] : span.tags(processedIn)) {
                    if (relIndex < 0) { // a tag from before this window is one this block has already placed
                        continue;
                    }
                    property_map map = tagMap.get();
                    dropOwnTrigger(map);
                    if (!map.empty()) {
                        _pending.emplace_back(widen(span.streamIndex) + static_cast<std::uint64_t>(relIndex) + _delay, std::move(map), std::nullopt, false);
                    }
                }
            },
            inputSpans);
    }

    /**
     * @brief Publishes every held input tag, and every tag left on the input, at the end-of-stream index.
     *
     * The tags leave in the order of their input samples, without their `end_of_stream` key. The block drops a held
     * detection, whose frame start is never output.
     */
    template<typename TInputSpans, typename TOutputSpans>
    void releaseAtEndOfStream(TInputSpans& inputSpans, TOutputSpans& outputSpans) {
        const std::pmr::string endOfStream = static_cast<std::pmr::string>(gr::tag::END_OF_STREAM);
        gr::for_each_reader_span(
            [this, &endOfStream](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const gr::Tag& tag : span.rawTags) {
                    const std::uint64_t at = widen(tag.index);
                    if (at < widen(span.streamIndex)) { // placed by an earlier call
                        continue;
                    }
                    property_map map = tag.map;
                    map.erase(endOfStream);
                    dropOwnTrigger(map);
                    if (!map.empty()) {
                        _pending.emplace_back(at + _delay, std::move(map), std::nullopt, false);
                    }
                }
            },
            inputSpans);
        std::ranges::stable_sort(_pending, std::ranges::less{}, &PendingTag::at);
        for (const PendingTag& tag : _pending) {
            if (!tag.ownDetection) {
                gr::for_each_writer_span([&tag](auto& span) { span.publishTag(tag.map, 0UZ); }, outputSpans);
            }
        }
        _pending.clear();
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t nSamples = std::min(inSpan.size(), outSpan.size());
        if (nSamples == 0UZ) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        const std::size_t index = inSpan.streamIndex;
        _wideAt += index - _callAt; // a `std::size_t` difference is exact across a 32-bit wrap
        _callAt = index;

        _window.assign(_history.begin(), _history.end());
        _window.insert(_window.end(), inSpan.begin(), std::next(inSpan.begin(), static_cast<std::ptrdiff_t>(nSamples)));

        detect(_wideAt, nSamples);
        if (const auto emitted = emit(_wideAt, nSamples, outSpan); !emitted) {
            this->emitErrorMessage("processBulk", emitted.error());
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        _history.assign(_window.end() - static_cast<std::ptrdiff_t>(_delay), _window.end());
        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        return work::Status::OK;
    }

private:
    /// @brief Removes the trigger keys of an input tag whose `trigger_name` equals `trigger_label`.
    void dropOwnTrigger(property_map& map) const {
        const auto  entry = map.find(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey()));
        const auto* label = entry == map.end() ? nullptr : entry->second.get_if<std::pmr::string>();
        if (label == nullptr || std::string_view(label->data(), label->size()) != std::string_view(_accepted.label)) {
            return;
        }
        for (const char* key : {gr::tag::TRIGGER_NAME.shortKey(), gr::tag::TRIGGER_TIME.shortKey(), gr::tag::TRIGGER_OFFSET.shortKey(), gr::tag::TRIGGER_META_INFO.shortKey()}) {
            map.erase(std::pmr::string(key));
        }
    }

    /// @brief Copies the named members into @p next. Throws for a `cp_len` longer than `fft_len`.
    template<typename Moved>
    [[nodiscard]] Accepted settingsAfter(Accepted next, Moved moved) const {
        if (moved("fft_len")) {
            next.fftLength = fft_len.value;
        }
        if (moved("cp_len")) {
            next.cpLength = cp_len.value;
        }
        if (moved("threshold")) {
            next.threshold = threshold.value;
        }
        if (moved("min_gap")) {
            next.minGap = min_gap.value;
        }
        if (moved("correct_cfo")) {
            next.correctCfo = correct_cfo.value;
        }
        if (moved("r_floor")) {
            next.rFloor = r_floor.value;
        }
        if (moved("trigger_label")) {
            next.label = trigger_label.value;
        }
        if (next.cpLength > next.fftLength) {
            throw gr::exception(std::format("cp_len is {} and fft_len is {}: a prefix is a copy of the symbol's own tail and cannot be longer than the symbol", next.cpLength, next.fftLength));
        }
        return next;
    }

    /// @brief Takes @p next as the settings the block computes on. Derives the estimator's values from them and resizes
    /// the delay line when the lag changes.
    void take(Accepted next) {
        _accepted                       = std::move(next);
        const std::size_t previousDelay = _delay;
        derive();
        if (_history.size() != _delay || _termRe.size() != _half) {
            resize(previousDelay);
        }
    }

    /// @brief Derives the lag, the dead time, the ring, the width bounds and the levels from the accepted settings.
    void derive() noexcept {
        const auto        fft      = static_cast<std::size_t>(_accepted.fftLength);
        const auto        cp       = static_cast<std::size_t>(_accepted.cpLength);
        const std::size_t maxWidth = cp + fft - 1UZ;
        _half                      = fft / 2UZ;
        _delay                     = 3UZ * _half + cp - 1UZ;
        _deadTime                  = std::max(std::uint64_t{1}, std::uint64_t{_accepted.minGap} * (cp + fft));
        _midShift                  = cp / 2UZ;
        _level                     = static_cast<double>(_accepted.threshold);
        _floor                     = static_cast<double>(_accepted.rFloor);
        // The falling edge reads P at the crossing's midpoint, at most half the widest accepted crossing back. The ring
        // holds that many values. A power-of-two size turns the per-sample index into a mask.
        _ring = std::bit_ceil(maxWidth / 2UZ + 1UZ);
        _mask = _ring - 1UZ;
        // A repeated-half preamble holds M above any threshold below 1 across its whole prefix. A much narrower
        // crossing comes from the ratio at the end of a burst, where one sample divided by itself reads as a perfect
        // correlation. A crossing shorter than half the prefix is refused and counted in nShortPlateaus().
        _minWidth = cp / 2UZ + 1UZ;
        _maxWidth = maxWidth;
    }

    /**
     * @brief Sizes the delay line and the rings for the derived lag and restarts both sums.
     *
     * The delay line keeps its newest inputs, and each held tag moves with its sample. A tag whose sample is dropped
     * keeps an index below the next output, and `emit()` publishes it on that output. A tag that `reset()` moved to
     * index 0 stays there. The counters, the dead time and the derotation continue.
     */
    void resize(std::size_t previousDelay) {
        std::vector<Complex> history(_delay, Complex{});
        const std::size_t    kept = std::min(previousDelay, _delay);
        std::copy(_history.end() - static_cast<std::ptrdiff_t>(kept), _history.end(), history.end() - static_cast<std::ptrdiff_t>(kept));
        _history = std::move(history);
        for (PendingTag& tag : _pending) {
            if (tag.at >= previousDelay) {
                tag.at = tag.at - previousDelay + _delay;
            }
        }
        _pRing.assign(_ring, Complex{});
        _termRe.assign(_half, 0.);
        _termIm.assign(_half, 0.);
        _termNorm.assign(_half, 0.);
        _pRe    = 0.;
        _pIm    = 0.;
        _r      = 0.;
        _above  = false;
        _riseAt = 0ULL;
    }

    /// @brief Slides `P` and `R` over the call's samples and turns each plateau into one trigger.
    void detect(std::uint64_t base, std::size_t nSamples) {
        const std::size_t half    = _half;
        const std::size_t offset  = _delay; // `_window[offset + j]` is input `base + j`
        const double      floorSq = _floor;
        const double      level   = _level;
        const Complex*    window  = _window.data();

        double            pRe      = _pRe;
        double            pIm      = _pIm;
        double            r        = _r;
        const std::size_t termMask = half - 1UZ;
        double*           termRe   = _termRe.data();
        double*           termIm   = _termIm.data();
        double*           termNorm = _termNorm.data();

        for (std::size_t j = 0UZ; j < nSamples; ++j) {
            // The arriving sample completes the window that starts at `d`. The zeroed history and term rings hold
            // silence before the stream. The arithmetic is written out in scalars: a `std::complex` multiply calls a
            // library helper that handles infinities.
            const double newRe = static_cast<double>(window[offset + j].real());
            const double newIm = static_cast<double>(window[offset + j].imag());
            const double midRe = static_cast<double>(window[offset + j - half].real());
            const double midIm = static_cast<double>(window[offset + j - half].imag());

            double gainRe   = midRe * newRe + midIm * newIm;
            double gainIm   = midRe * newIm - midIm * newRe;
            double gainNorm = newRe * newRe + newIm * newIm;
            if (!std::isfinite(gainRe + gainIm + gainNorm)) {
                gainRe   = 0.;
                gainIm   = 0.;
                gainNorm = 0.;
            }

            const std::size_t slot = detail::sizeFrom((base + j) & termMask);
            pRe += gainRe - termRe[slot];
            pIm += gainIm - termIm[slot];
            r += gainNorm - termNorm[slot];
            termRe[slot]   = gainRe;
            termIm[slot]   = gainIm;
            termNorm[slot] = gainNorm;

            if (base + j + 1U < 2UZ * half) {
                continue;
            }
            const std::uint64_t at    = base + j + 1U - 2UZ * half;
            const bool          above = r > floorSq && (pRe * pRe + pIm * pIm) > level * r * r;
            if (above) {
                if (!_above) {
                    _above  = true;
                    _riseAt = at;
                }
                // P is stored only inside a plateau, the one place it is read back. Off a plateau the loop makes no
                // store that could alias its loads.
                _pRing[detail::sizeFrom(at & _mask)] = Complex(static_cast<float>(pRe), static_cast<float>(pIm));
                continue;
            }
            if (!_above) {
                continue;
            }
            _pRe   = pRe;
            _pIm   = pIm;
            _r     = r;
            _above = false;
            close(at, _riseAt);
        }

        _pRe = pRe;
        _pIm = pIm;
        _r   = r;
    }

    /// @brief Places the trigger of the ended plateau `[riseAt, fallAt)` at the frame's first sample.
    void close(std::uint64_t fallAt, std::uint64_t riseAt) {
        const std::uint64_t width = fallAt - riseAt;
        if (width < _minWidth) {
            ++_short;
            return;
        }
        if (width > _maxWidth) {
            ++_wide;
            return;
        }
        const std::uint64_t middle = riseAt + (width - 1U) / 2U;
        const Complex       peak   = _pRing[detail::sizeFrom(middle & _mask)];

        if (middle < _midShift) {
            ++_late; // the frame began before the stream
            return;
        }
        const std::uint64_t frameAt = middle - _midShift;
        if (_triggers > 0ULL && frameAt < _resumeAt) {
            return; // inside the dead time a frame's own structure can reopen the plateau
        }

        const double eps = std::atan2(static_cast<double>(peak.imag()), static_cast<double>(peak.real())) / std::numbers::pi;
        _lastCfo         = eps;
        ++_triggers;
        _resumeAt = frameAt + _deadTime;

        _pending.emplace_back(frameAt + _delay,
            property_map{{gr::tag::TRIGGER_NAME.shortKey(), _accepted.label}, //
                {gr::tag::TRIGGER_OFFSET.shortKey(), 0.f},                    //
                {gr::tag::TRIGGER_META_INFO.shortKey(), property_map{{"cfo_fractional", static_cast<float>(eps)}, {"plateau_samples", static_cast<gr::Size_t>(width)}}}},
            std::optional<double>(-std::numbers::pi * eps / static_cast<double>(_half)), true);
    }

    /// @brief The 64-bit index of stream index @p index, which lies within 2^31 samples of the current call's first
    /// input. Reads `index - _callAt` as signed: the offset is negative for an input before the call and exact across
    /// a 32-bit wrap.
    [[nodiscard]] std::uint64_t widen(std::size_t index) const noexcept {
        const std::int64_t offset = static_cast<std::ptrdiff_t>(index - _callAt);
        return _wideAt + static_cast<std::uint64_t>(offset);
    }

    /**
     * @brief Writes the delayed stream, split at every tag this call reaches.
     *
     * The forwarded input tags and the block's own detections go out as one sequence sorted by sample. The derotation
     * takes its new increment at the sample its trigger marks. A correction that is not finite returns an error.
     */
    [[nodiscard]] std::expected<void, gr::Error> emit(std::uint64_t base, std::size_t nSamples, OutputSpanLike auto& outSpan) {
        std::ranges::stable_sort(_pending, std::ranges::less{}, &PendingTag::at);

        const Complex* source   = _window.data();
        std::size_t    produced = 0UZ;
        std::size_t    k        = 0UZ;

        while (produced < nSamples) {
            while (k < _pending.size() && _pending[k].at < base + produced) {
                if (_pending[k].ownDetection) {
                    ++_late; // its sample has left the delay line, and placing it here would misalign the frame
                } else {
                    outSpan.publishTag(_pending[k].map, produced);
                }
                ++k;
            }
            std::size_t next = nSamples;
            if (k < _pending.size() && _pending[k].at < base + nSamples) {
                next = detail::sizeFrom(_pending[k].at - base);
            }
            if (next > produced) {
                write(source + produced, next - produced, produced, outSpan);
                produced = next;
                continue;
            }
            outSpan.publishTag(_pending[k].map, produced);
            if (_pending[k].increment.has_value()) {
                const double increment = *_pending[k].increment;
                if (!std::isfinite(increment)) {
                    return std::unexpected(gr::Error(std::format("gr::blocks::ofdm::SchmidlCoxSync: the carrier-offset correction {} rad per sample is not finite", increment)));
                }
                _phasor.setIncrement(increment);
            }
            ++k;
        }
        _pending.erase(_pending.begin(), _pending.begin() + static_cast<std::ptrdiff_t>(k));
        return {};
    }

    void write(const Complex* source, std::size_t count, std::size_t at, OutputSpanLike auto& outSpan) {
        const std::span<Complex> destination(outSpan.begin() + static_cast<std::ptrdiff_t>(at), count);
        if (_accepted.correctCfo) {
            _phasor.mix(std::span<const Complex>(source, count), destination);
        } else {
            std::copy_n(source, count, destination.begin());
        }
    }
};

} // namespace gr::blocks::ofdm

#endif // GNURADIO_OFDM_SCHMIDL_COX_SYNC_HPP
