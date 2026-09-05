#ifndef GNURADIO_PAM_SYMBOLS_HPP
#define GNURADIO_PAM_SYMBOLS_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/algorithm/MeasurementRecord.hpp>
#include <gnuradio-4.0/algorithm/MeasurementSlot.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::digital {

namespace detail {

/// The uniform M-PAM grid both blocks share: levels at `(2r - (M-1)) * spread / 2` for rank
/// `r = 0 .. M-1` — `±1, ±3, …` at the nominal spacing 2 — with decision thresholds at the
/// midpoints and the two outer regions unbounded.
[[nodiscard]] inline std::size_t pamRank(double value, double spread, std::size_t levels) noexcept {
    const double scaled = value / spread + 0.5 * static_cast<double>(levels - 1UZ);
    if (!(scaled > 0.0)) {
        return 0UZ;
    }
    const auto rank = static_cast<std::size_t>(std::lround(scaled));
    return rank >= levels ? levels - 1UZ : rank;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::digital::LevelTracker, [T], [float])

template<std::floating_point T>
struct LevelTracker : Block<LevelTracker<T>> {
    using Description = Doc<R""(
@brief Decision-directed gain and DC normalization for real M-PAM symbols.

One sample per symbol in, the same sample normalized out so the M levels land on the `±1, ±3, …` grid whatever
spacing and common shift the stream carries. Two loops ride the decision error - the distance from the nearest level
of the tracker's current belief: the OFFSET integrates it (a common shift of every level is exactly the running mean
of the error; for a discriminator-fed chain it is the carrier frequency offset), and the SPREAD - the spacing between
adjacent levels, nominally 2 - grows or shrinks as symbols fall outside or inside their decided places. Both are
readable while the block runs, through `spread()` and `offset()`. An outer
level's distance from center carries proportionally more leverage on the spacing, so its update weight is halved
against the innermost pair's - the proven compromise carried from the C4FM tracker this block generalizes, whose
deviation-tracking accuracy (±15 % measured to ±0.05 of truth) is the behavior it must reproduce. The spread is
clamped to a band about nominal so a noise burst cannot walk it somewhere it cannot return from; the outer decision
regions are unbounded, an overshoot being still the symbol it overshot.

The optional `records` port carries the two readings as one `DataSet<float>` per call that folds symbols in - the
cadence at which `spread()` and `offset()` themselves refresh - stamped with the call's first input symbol. Both
values are in the input's own units, which the block does not know, so the record states no unit for either. A call
that finds the port's buffer full skips its record rather than stalling the symbol path: the two readings are a
running state, and the next record carries a newer one.

Even M only; a grid with a level at zero changes the decision and both loops, and no consumer exists.
)"">;

    PortIn<T>  in;
    PortOut<T> out;
    /// One record per call, for a consumer outside C++. The port is optional: leaving it unconnected costs nothing
    /// and the two readers below remain the whole interface for a graph that polls.
    PortOut<DataSet<float>, Async, Optional> records;

    Annotated<gr::Size_t, "n_levels", Visible, Doc<"M, even and at least 2">>                              n_levels     = 4U;
    Annotated<double, "spread_rate", Doc<"level-spacing loop gain">>                                       spread_rate  = 0.0100;
    Annotated<double, "offset_rate", Doc<"common-shift loop gain">>                                        offset_rate  = 0.1250;
    Annotated<double, "spread_limit", Doc<"fractional band the spacing may move in, about its nominal 2">> spread_limit = 0.20;

    GR_MAKE_REFLECTABLE(LevelTracker, in, out, records, n_levels, spread_rate, offset_rate, spread_limit);

    /// The tracked quantities, in this order, so that a reader sees both of one instant together.
    static constexpr std::size_t kSpread = 0UZ;
    static constexpr std::size_t kOffset = 1UZ;

    gr::measurement::MeasurementSlot<2UZ> _tracked{};

    double _spread = 2.0;
    double _offset = 0.0;

    std::uint64_t _symbolsSeen = 0ULL; ///< symbols folded in so far, which is where the next record's stretch begins

    /// @brief The tracked spacing between adjacent levels, 2 at nominal. Callable from any thread.
    [[nodiscard]] double spread() const noexcept { return _tracked.read().first[kSpread]; }

    /// @brief The tracked common shift of every level, in input units. Callable from any thread.
    [[nodiscard]] double offset() const noexcept { return _tracked.read().first[kOffset]; }

    /// @brief The two readings as a record, on the module's measurement conventions. Symbols, so no sample rate; and
    /// no unit, the two being in the input's own units, which the block does not know.
    [[nodiscard]] DataSet<float> makeRecord(std::uint64_t startsAt, std::size_t nSymbols) const {
        const std::array<gr::measurement::ScalarChannel, 2UZ> channels{{
            {"spread", "LevelSpacing", "", static_cast<float>(spread())},
            {"offset", "LevelOffset", "", static_cast<float>(offset())},
        }};
        return gr::measurement::makeScalarRecord(std::span<const gr::measurement::ScalarChannel>(channels), std::numeric_limits<float>::quiet_NaN(), startsAt, property_map{{std::pmr::string("n_symbols"), pmt::Value(static_cast<std::uint64_t>(nSymbols))}, {std::pmr::string("index_unit"), pmt::Value(std::string("symbol"))}});
    }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (n_levels < 2U || (n_levels % 2U) != 0U) {
            throw gr::exception(std::format("n_levels must be even and at least 2, got {}", n_levels.value));
        }
        if (newSettings.contains("n_levels")) { // a new grid invalidates every tracked quantity
            _spread = 2.0;
            _offset = 0.0;
            _tracked.publish({_spread, _offset}, 1ULL);
        }
    }

    void start() {
        _spread      = 2.0;
        _offset      = 0.0;
        _symbolsSeen = 0ULL;
        _tracked.publish({_spread, _offset}, 1ULL);
    }

    /// @brief One record per call, built where the two readings are of one instant. Building it allocates, which is
    /// why the method is not `noexcept`; the tracking loops themselves allocate nothing.
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& recordSpan) {
        const std::size_t nSamples = std::min(inSpan.size(), outSpan.size());
        const auto        levels   = static_cast<std::size_t>(n_levels.value);
        const double      spreadLo = 2.0 * (1.0 - spread_limit);
        const double      spreadHi = 2.0 * (1.0 + spread_limit);
        const std::size_t innerLo  = levels / 2UZ - 1UZ;

        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            const double value = static_cast<double>(inSpan[i]) - _offset;
            outSpan[i]         = static_cast<T>(2.0 * value / _spread);

            const std::size_t rank  = detail::pamRank(value, _spread, levels);
            const double      level = (2.0 * static_cast<double>(rank) - static_cast<double>(levels - 1UZ)) * 0.5 * _spread;
            const double      error = value - level;

            const double weight = (rank == innerLo || rank == innerLo + 1UZ) ? 1.0 : 0.5;
            _spread += (level > 0.0 ? error : -error) * weight * spread_rate;
            _spread = std::clamp(_spread, spreadLo, spreadHi);
            _offset += error * offset_rate;
        }

        std::size_t made = 0UZ;
        if (nSamples > 0UZ) {
            _tracked.publish({_spread, _offset}, 1ULL); // the readers refresh once per call, never per sample
            if (recordSpan.isConnected && recordSpan.size() > 0UZ) {
                recordSpan[0UZ] = makeRecord(_symbolsSeen, nSamples);
                made            = 1UZ;
            }
            _symbolsSeen += static_cast<std::uint64_t>(nSamples);
        }

        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        recordSpan.publish(made);
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::digital::PamSlicer, [T], [float])

template<std::floating_point T>
struct PamSlicer : Block<PamSlicer<T>> {
    using Description = Doc<R""(
@brief The M-PAM hard decision: one symbol on the `±1, ±3, …` grid in, one label out.

Thresholds sit at the midpoints of the adjacent-level gaps and the two outer regions are unbounded: a symbol that
overshoots is still the symbol it overshot, and bounding the top region would turn a one-level overshoot into the
most distant wrong answer available. The label table maps level rank, lowest to highest, to the emitted value,
because real protocols' maps are non-monotonic - C4FM's is {3, 2, 0, 1} for {-3, -1, +1, +3}. An empty table emits the
rank itself, the identity map for any M. The input's spacing is the nominal 2; a stream at any other spacing is
normalized upstream.
)"">;

    PortIn<T>             in;
    PortOut<std::uint8_t> out;

    Annotated<gr::Size_t, "n_levels", Visible, Doc<"M, even and at least 2">>                                                                      n_levels = 4U;
    Annotated<std::vector<gr::Size_t>, "labels", Doc<"level rank (lowest to highest) to emitted value; size M, or empty to emit the rank itself">> labels{};

    GR_MAKE_REFLECTABLE(PamSlicer, in, out, n_levels, labels);

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (n_levels < 2U || (n_levels % 2U) != 0U) {
            throw gr::exception(std::format("n_levels must be even and at least 2, got {}", n_levels.value));
        }
        if (!labels.value.empty() && labels.value.size() != static_cast<std::size_t>(n_levels.value)) {
            throw gr::exception(std::format("labels holds {} entries for {} levels; one per level rank", labels.value.size(), n_levels.value));
        }
        for (const gr::Size_t label : labels.value) {
            if (label > 255U) { // the emitted item is one byte, and a wider label would be truncated on the sample path
                throw gr::exception(std::format("labels holds {}, which does not fit the byte the block emits; every label is at most 255", label));
            }
        }
    }

    [[nodiscard]] std::uint8_t processOne(T sample) const noexcept {
        const std::size_t rank = detail::pamRank(static_cast<double>(sample), 2.0, static_cast<std::size_t>(n_levels.value));
        return static_cast<std::uint8_t>(labels.value.empty() ? rank : labels.value[rank]);
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_PAM_SYMBOLS_HPP
