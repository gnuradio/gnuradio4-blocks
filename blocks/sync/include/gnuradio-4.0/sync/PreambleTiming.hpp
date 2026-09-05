#ifndef GNURADIO_SYNC_PREAMBLE_TIMING_HPP
#define GNURADIO_SYNC_PREAMBLE_TIMING_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <format>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/sync/PreambleTone.hpp>

namespace gr::blocks::sync {

GR_REGISTER_BLOCK(gr::blocks::sync::PreambleTiming, [T], [float])

template<typename T>
requires(std::same_as<T, float>)
struct PreambleTiming : Block<PreambleTiming<T>, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief Data-aided burst timing: measures a burst's symbol phase from its alternating training sequence and tags it.

A two-level training sequence of alternating symbols is, through an FM discriminator, a real tone at half the symbol
rate. The frequency is known, so that tone's phase is the symbol phase and one dot product measures it — where a timing
loop has to acquire the phase from an unknown start, which a burst gives it too few symbols to do. The block watches the
fraction of a sliding window's energy lying in that bin, takes the first crossing of `threshold`, follows it for one
further window and fires on the best-filled window of that span. One excursion above the threshold is one tag, so a
burst yields exactly one; `hold_off_symbols` additionally silences the block for a stated span after a tag, for a link
whose payload can hold an alternating run as long as its preamble. It is off by default, and a value at or above the
spacing between two bursts' preambles swallows the second burst. The statistic is a ratio, so it needs no AGC and no
absolute level and one threshold serves every link.

The fractional phase a symbol synchronizer acts on rides on `clock_est` in input samples beside the nominal period, or
on `time_est` alone when `preset_period` is false. Naming a `trigger_label` adds the reserved `trigger_name` beside it,
and `trigger_offset` carrying the same phase in the seconds that key declares, which turns the tag into a burst gate as
well; it is off by default because a framer downstream resynchronizes on any trigger tag, and a phase measurement taken
at the end of a training sequence is not where a frame starts. The period in the tag is `sample_rate / symbol_rate`, a
constant of the configuration and not a measurement: a preamble short enough for a burst measures the rate far too
coarsely to be worth presetting, and what the constant buys is that the loop's integrator restarts with each burst
instead of carrying whatever it drifted to across the preceding gap.

`preamble_symbols = 0` disables the block: the stream passes through unchanged and no tag is emitted, which is what lets
one recipe serve a continuous link and a burst one. Enabled, the block holds back one analysis window plus one symbol of
output, because the tag names a sample that can lie a symbol ahead of the read position; that hold-back is the whole
latency, the analysis window itself being history rather than delay, and a tag arriving from upstream rides through at
the call offset the framework gives it rather than being delayed with the stream. `float` only: the statistic is defined
on the real discriminator stream, and an alternating preamble on a complex stream carries a different timing line.
)"">;

    PortIn<T>                in;
    PortOut<T>               out;
    PortOut<float, Optional> statistic;

    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"the stream's rate; read to write trigger_offset in the seconds its declaration requires">>                                             sample_rate      = 48000.f;
    Annotated<float, "symbol_rate", Unit<"Hz">, Doc<"symbol rate; samples per symbol is sample_rate/symbol_rate and the training tone sits at half of it">>                                 symbol_rate      = 9600.f;
    Annotated<gr::Size_t, "preamble_symbols", Unit<"symbols">, Doc<"alternating symbols the analysis window spans; 0 disables the block, which then passes the stream through">>            preamble_symbols = 0U;
    Annotated<double, "threshold", Doc<"the share of a window's energy in the tone's bin that declares a preamble; in (0, 1)">>                                                             threshold        = 0.35;
    Annotated<gr::Size_t, "hold_off_symbols", Unit<"symbols">, Doc<"symbols after a tag during which no second tag is emitted; must stay under the spacing between two bursts' preambles">> hold_off_symbols = 0U;
    Annotated<std::string, "trigger_label", Doc<"a burst gate under the reserved trigger_name key beside the timing payload; empty writes no trigger keys at all">>                         trigger_label{};
    Annotated<bool, "preset_period", Doc<"whether the tag carries the nominal period beside the phase, that is clock_est rather than time_est">>                                            preset_period = true;

    GR_MAKE_REFLECTABLE(PreambleTiming, in, out, statistic, sample_rate, symbol_rate, preamble_symbols, threshold, hold_off_symbols, trigger_label, preset_period);

    std::uint64_t nDetections = 0ULL; ///< tags emitted, one per burst
    std::uint64_t nSuppressed = 0ULL; ///< up-crossings that fell inside a hold-off and produced no tag

    gr::sync::PreambleToneEstimator _estimator{};

    bool          _configured       = false;
    bool          _enabled          = false;
    double        _samplesPerSymbol = 5.0;
    std::uint64_t _symbolStride     = 5ULL; /// `ceil(samples_per_symbol)`: how far past a window an instant can sit
    std::uint64_t _holdOff          = 0ULL;
    std::uint64_t _holdBack         = 0ULL; /// samples pushed and not yet published, so a tag always names one of them
    std::size_t   _retained         = 1UZ;  /// the ring: the hold-back to publish from, and two windows for the refit

    std::vector<float> _history{};    /// the retained input, indexed by absolute offset modulo the ring
    std::vector<float> _statistics{}; /// `Lambda` of the window ending at each retained sample

    std::vector<std::pair<std::uint64_t, property_map>> _pending{}; /// tags whose sample has not been published yet

    std::uint64_t _pushed        = 0ULL; /// absolute offset one past the last sample fed to the estimator
    std::uint64_t _published     = 0ULL; /// absolute offset one past the last sample published
    bool          _above         = false;
    bool          _arming        = false;
    std::uint64_t _armEnd        = 0ULL;
    std::uint64_t _bestEnd       = 0ULL;
    double        _bestStatistic = 0.0;
    std::uint64_t _suppressUntil = 0ULL;
    bool          _anchored      = false;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (!(sample_rate > 0.f) || !std::isfinite(sample_rate)) {
            throw gr::exception(std::format("sample_rate must be positive and finite, got {}", sample_rate.value));
        }
        if (!(symbol_rate > 0.f) || !std::isfinite(symbol_rate)) {
            throw gr::exception(std::format("symbol_rate must be positive and finite, got {}", symbol_rate.value));
        }
        if (!(threshold > 0.0) || !(threshold < 1.0)) {
            throw gr::exception(std::format("threshold is a share of a window's energy and must lie in (0, 1), got {}", threshold.value));
        }

        const double sps  = static_cast<double>(sample_rate.value) / static_cast<double>(symbol_rate.value);
        _samplesPerSymbol = sps;
        _holdOff          = static_cast<std::uint64_t>(std::llround(sps * static_cast<double>(hold_off_symbols.value)));

        if (preamble_symbols > 0U && !(sps > 1.0)) {
            throw gr::exception(std::format("sample_rate / symbol_rate is {} — the tone at half the symbol rate needs more than two samples a period", sps));
        }

        static constexpr std::array kRestartKeys{"sample_rate", "symbol_rate", "preamble_symbols"};
        if (_configured && !std::ranges::any_of(kRestartKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            return;
        }
        _configured = true;
        _enabled    = preamble_symbols > 0U;

        if (!_enabled) {
            _holdBack = 0ULL;
            _retained = 1UZ;
            reset();
            in.min_samples = 1UZ;
            return;
        }

        _estimator.configure(sps, static_cast<double>(preamble_symbols.value));
        _symbolStride = static_cast<std::uint64_t>(std::ceil(sps));

        // The tag names a sample the block has not published, and that sample can lie anywhere from the chosen
        // window's last sample — at most one window behind the read position — to one symbol past that position.
        // Holding back that span is what makes the tag reachable, and it is the block's whole latency.
        const std::uint64_t window = _estimator.windowLength();
        _holdBack                  = window + _symbolStride + 1ULL;
        // The refit reaches one window behind the chosen window's end, which itself is at most a window behind the
        // read position; the ring also has to hold everything pushed and not yet published.
        _retained = static_cast<std::size_t>(std::max(2ULL * window, _holdBack + 1ULL));
        reset();
        in.min_samples = static_cast<std::size_t>(_holdBack) + 1UZ;
    }

    void reset() {
        _estimator.reset();
        _history.assign(_retained, 0.f);
        _statistics.assign(_retained, 0.f);
        _pending.clear();
        _pushed        = 0ULL;
        _published     = 0ULL;
        _above         = false;
        _arming        = false;
        _armEnd        = 0ULL;
        _bestEnd       = 0ULL;
        _bestStatistic = 0.0;
        _suppressUntil = 0ULL;
        _anchored      = false;
    }

    void start() {
        nDetections = 0ULL;
        nSuppressed = 0ULL;
        reset();
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& statisticSpan) {
        const bool        wantStatistic = statisticSpan.isConnected;
        const std::size_t room          = wantStatistic ? std::min(outSpan.size(), statisticSpan.size()) : outSpan.size();

        if (!_enabled) { // a disabled block is a wire: the stream passes through sample for sample and carries no tag
            const std::size_t copied = std::min(inSpan.size(), room);
            std::copy_n(inSpan.begin(), copied, outSpan.begin());
            if (wantStatistic) {
                std::fill_n(statisticSpan.begin(), copied, 0.f);
            }
            std::ignore = inSpan.consume(copied);
            outSpan.publish(copied);
            statisticSpan.publish(wantStatistic ? copied : 0UZ);
            return copied == 0UZ ? (room == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS) : work::Status::OK;
        }

        if (!_anchored) { // the absolute stream position every tag offset is stated against
            _pushed    = static_cast<std::uint64_t>(inSpan.streamIndex);
            _published = _pushed;
            _anchored  = true;
        }

        std::size_t taken = 0UZ;
        std::size_t made  = 0UZ;
        while (true) {
            // Everything past the hold-back is owed to the output and goes out first, which returns the ring the room
            // the next step needs; the ring carries the rest, so a step never runs past what is still owed.
            const std::uint64_t ready     = _pushed > _holdBack ? _pushed - _holdBack : 0ULL;
            const std::size_t   available = narrowIndex<std::size_t>(ready > _published ? ready - _published : 0ULL);
            const std::size_t   emitted   = std::min(room - made, available);
            for (std::size_t k = 0UZ; k < emitted; ++k) {
                const std::uint64_t at = _published + k;
                outSpan[made + k]      = _history[slotOf(at)];
                if (wantStatistic) {
                    statisticSpan[made + k] = _statistics[slotOf(at)];
                }
                releaseTag(at, made + k, outSpan, statisticSpan, wantStatistic);
            }
            _published += emitted;
            made += emitted;

            const std::size_t held = narrowIndex<std::size_t>(_pushed - _published);
            const std::size_t step = std::min({inSpan.size() - taken, room - made, _retained - held});
            if (step == 0UZ) {
                break;
            }
            for (std::size_t k = 0UZ; k < step; ++k) {
                observe(static_cast<double>(inSpan[taken + k]));
            }
            taken += step;
        }

        std::ignore = inSpan.consume(taken);
        outSpan.publish(made);
        statisticSpan.publish(wantStatistic ? made : 0UZ);

        if (taken == 0UZ && made == 0UZ) {
            return room == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    /// @brief A 64-bit sample offset narrowed to an index; a template, so the cast stands where `std::size_t` is 32
    /// bits and is not a useless cast where the two are the same 64-bit type.
    template<std::integral To, std::integral From>
    [[nodiscard]] static constexpr To narrowIndex(From value) noexcept {
        return static_cast<To>(value);
    }

    [[nodiscard]] std::size_t slotOf(std::uint64_t offset) const noexcept { return offset % _retained; }

    /// @brief One sample into the sliding statistic, and the detector's state machine over what it returns.
    void observe(double sample) {
        const double        value = _estimator.push(sample);
        const std::uint64_t at    = _pushed;

        _history[slotOf(at)]    = static_cast<float>(sample);
        _statistics[slotOf(at)] = static_cast<float>(value);
        ++_pushed;

        if (!_estimator.filled()) {
            return;
        }
        const bool above = value > threshold;
        const bool rose  = above && !_above;
        _above           = above;

        if (_arming) { // one window past the first crossing, then the window of that span best filled by the preamble
            if (value > _bestStatistic) {
                _bestStatistic = value;
                _bestEnd       = at;
            }
            if (at >= _armEnd) {
                fire();
            }
            return;
        }
        if (!rose) {
            return;
        }
        if (at < _suppressUntil) { // an alternating run inside the payload the last burst carried is not a second burst
            ++nSuppressed;
            return;
        }
        _arming        = true;
        _bestStatistic = value;
        _bestEnd       = at;
        _armEnd        = at + _estimator.windowLength();
    }

    /**
     * @brief Refits the chosen window and queues the tag the phase it returns names.
     *
     * The recursion said which window to fit; the fit itself is the `O(N)` dot product, so nothing the recursion
     * drifted by reaches the tag.
     *
     * The instant named is the **last symbol instant inside the window**, which is the last one the fit speaks for.
     * A synchronizer takes the sample before a preset at the phase it was already holding, so the instant the tag
     * names is the first one it takes correctly: naming an instant past the window would leave the last training
     * symbol on the old phase, and a differential decoder downstream reads that symbol together with the first of
     * the frame.
     */
    void fire() {
        _arming = false;

        const std::size_t   length = _estimator.windowLength();
        const std::uint64_t first  = _bestEnd + 1ULL - length;
        std::vector<float>  window(length);
        for (std::size_t k = 0UZ; k < length; ++k) {
            window[k] = _history[slotOf(first + k)];
        }

        const gr::sync::PreambleToneFit fit      = _estimator.fit(std::span<const float>(window));
        const double                    instant  = static_cast<double>(first) + _estimator.instantAfter(fit.phase, static_cast<double>(length - 1ULL) - _samplesPerSymbol);
        const auto                      offset   = static_cast<std::uint64_t>(std::llround(instant));
        const auto                      fraction = static_cast<float>(instant - static_cast<double>(offset));

        // The timing payload alone is inert to everything but a symbol synchronizer. The reserved trigger pair says
        // "a burst starts here", which a framer downstream resynchronizes on, so it is written only where a graph
        // asks for a burst gate by naming one.
        property_map tag{};
        if (!trigger_label.value.empty()) {
            tag[property_map::key_type(gr::tag::TRIGGER_NAME.shortKey())]   = trigger_label.value;
            tag[property_map::key_type(gr::tag::TRIGGER_OFFSET.shortKey())] = fraction / sample_rate.value;
        }
        if (preset_period) {
            tag[property_map::key_type("clock_est")] = std::vector<float>{fraction, static_cast<float>(_samplesPerSymbol)};
        } else {
            tag[property_map::key_type("time_est")] = fraction;
        }
        _pending.emplace_back(offset, std::move(tag));

        ++nDetections;
        _suppressUntil = offset + _holdOff;
    }

    /// @brief Publishes every queued tag the sample now being written has reached, at that sample's own output offset.
    void releaseTag(std::uint64_t at, std::size_t index, auto& outSpan, auto& statisticSpan, bool wantStatistic) {
        while (!_pending.empty() && _pending.front().first <= at) {
            const property_map& tag = _pending.front().second;
            outSpan.publishTag(tag, index);
            if (wantStatistic) {
                statisticSpan.publishTag(tag, index);
            }
            _pending.erase(_pending.begin());
        }
    }
};

} // namespace gr::blocks::sync

#endif // GNURADIO_SYNC_PREAMBLE_TIMING_HPP
