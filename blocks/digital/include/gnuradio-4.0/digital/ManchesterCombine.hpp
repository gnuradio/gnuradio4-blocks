#ifndef GNURADIO_DIGITAL_MANCHESTERCOMBINE_HPP
#define GNURADIO_DIGITAL_MANCHESTERCOMBINE_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <string>
#include <string_view>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::digital {

namespace detail {

/// @brief What a soft chip stream carries, which decides how a bit is read out of a chip pair.
enum class ManchesterInput : std::uint8_t {
    Differential, ///< the output of a differential detector: one parity is a constant, the other is the bit
    Chips         ///< chip decisions: a bit is half the difference of the two chips of a pair
};

[[nodiscard]] inline ManchesterInput manchesterInput(std::string_view name) {
    if (name == "differential") {
        return ManchesterInput::Differential;
    }
    if (name == "chips") {
        return ManchesterInput::Chips;
    }
    throw gr::exception(std::format("input must be 'differential' or 'chips', got '{}'", name));
}

/**
 * @brief Which parity of the chip index a bit's pair starts at, estimated from two leaky means.
 *
 * One statistic per parity of the absolute chip index, and the two parities are told apart because exactly one of
 * them carries a constant. On a differential detector's output the statistic is the item itself: the two chips of
 * one bit are antipodal, so the product the detector forms inside a bit is the same negative constant at every
 * bit, while the product it forms across a bit boundary is the differentially detected bit and averages to
 * nothing. On chip decisions the statistic is the product of adjacent items, which reads that same constant on
 * the parity that closes a pair and nothing on the parity that straddles two bits. Either way the parity whose
 * mean stands further from zero is the one inside a bit, and `phase` is the other one.
 *
 * `phase` is the parity of the absolute chip index at which a pair starts, which is `ManchesterDecoder`'s reading
 * of the same word. It moves only after `hold` symbols in a row have asked for the same move, so no single noisy
 * symbol can move a grid a whole averaging window agreed on.
 */
struct ManchesterPairing {
    std::array<double, 2UZ> stat{0., 0.}; ///< the leaky mean of each parity's statistic
    double                  alpha    = 1. / 256.;
    unsigned                phase    = 0U;
    std::size_t             hold     = 64UZ;
    std::size_t             disagree = 0UZ;
    bool                    pinned   = false; ///< the pairing is a setting rather than an estimate

    void observe(unsigned parity, double value) noexcept { stat[parity] += alpha * (value - stat[parity]); }

    /// @brief The parity a pair starts at under the statistics as they stand.
    [[nodiscard]] unsigned preferred(bool differential) const noexcept {
        if (differential) { // the intra-bit parity is the constant one, so the published parity is the other
            return std::abs(stat[0UZ]) <= std::abs(stat[1UZ]) ? 0U : 1U;
        }
        // the product is negative inside a bit, and the pair starts one chip before the index that closes it
        return (stat[0UZ] <= stat[1UZ] ? 0U : 1U) ^ 1U;
    }

    /// @brief One symbol of evidence.
    void step(bool differential) noexcept {
        if (pinned) {
            return;
        }
        const unsigned want = preferred(differential);
        if (want == phase) {
            disagree = 0UZ;
            return;
        }
        if (++disagree >= hold) {
            phase    = want;
            disagree = 0UZ;
        }
    }

    /// @brief How far apart the two parities stand, on [0, 1]: 1 when the published parity's mean is exactly zero.
    [[nodiscard]] float confidence() const noexcept {
        const double dropped = std::abs(stat[phase ^ 1U]);
        const double kept    = std::abs(stat[phase]);
        const double total   = dropped + kept;
        return total > 0. ? static_cast<float>(std::clamp((dropped - kept) / total, 0., 1.)) : 0.f;
    }

    void reset() noexcept {
        stat     = {0., 0.};
        disagree = 0UZ;
    }
};

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::digital::ManchesterCombine)

struct ManchesterCombine : Block<ManchesterCombine, BackwardTagPropagation, Resampling<2UZ, 1UZ, true>> {
    using Description = Doc<R""(
@brief One soft bit from the two chips of a Manchester symbol, at the pairing the stream itself says is in force.

The soft counterpart of ManchesterDecoder, and the operation is not the same one. Chip decisions arrive as a `{+1, -1}`
stream and a bit is half the difference of its pair, which is the textbook soft combination. A DIFFERENTIAL detector's
output is a different stream and wants a different reading: with chips `c[2k] = +b_k` and `c[2k+1] = -b_k` the detector
forms `Re(s[n] conj(s[n-1]))`, which inside a bit multiplies two antipodal chips and is the constant `-|A|^2` at every
bit, and across a bit boundary is `-b_{k-1} b_k`, the differentially detected bit. Half the difference of two adjacent
outputs is then `0.5 (1 - b_{k-1} b_k)`, a UNIPOLAR `{0, 1}` variable in either pairing, half of whose levels sit on a
slicer's threshold rather than either side of it. `input` says which stream arrived; on `differential` the bit is the
inter-bit parity selected, not a difference taken.

The pairing is estimated from the stream and needs no help from the framing: exactly one parity of the chip index
carries a constant, so the parity whose leaky mean stands further from zero is the one inside a bit and the other is
published. `averaging_symbols` sets that mean's window and `hold_symbols` how many symbols in a row must ask for a
move before the grid takes it. `phase` pins the grid instead, at the parity of the absolute chip index a pair starts
at -- `ManchesterDecoder`'s reading of the same word. `chip_phase` reports the grid in force and `confidence` the
normalized distance between the two parities' statistics, both one item per bit and both optional.

The block is exactly 2:1, so a forwarded `sample_rate` is halved and a tag arriving at input item t leaves at output
item floor(t/2). Being a rate changer it forwards the framework's own tag vocabulary only.
)"">;

    PortIn<float>            in;
    PortOut<float>           out;
    PortOut<float, Optional> chip_phase;
    PortOut<float, Optional> confidence;

    Annotated<std::string, "input", Doc<"'differential' for a differential detector's output, 'chips' for chip decisions">, Visible> input             = std::string("differential");
    Annotated<gr::Size_t, "averaging_symbols", Unit<"symbol">, Doc<"symbols the pairing estimate's leaky mean averages over">>       averaging_symbols = 256U;
    Annotated<gr::Size_t, "hold_symbols", Unit<"symbol">, Doc<"symbols of unbroken disagreement before the pairing moves">>          hold_symbols      = 64U;
    Annotated<std::int32_t, "phase", Doc<"0 or 1 pins the parity of the chip index a pair starts at; -1 estimates it">>              phase             = -1;

    GR_MAKE_REFLECTABLE(ManchesterCombine, in, out, chip_phase, confidence, input, averaging_symbols, hold_symbols, phase);

    detail::ManchesterPairing _pairing{};
    bool                      _differential = true;
    float                     _previous     = 0.f;   ///< the chip before this chunk, which a pair on the other parity reads
    bool                      _havePrevious = false; ///< false only for the first pair of a stream

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void reset() {
        _pairing.reset();
        _havePrevious = false;
        _previous     = 0.f;
    }

    /**
     * @brief Validates every setting, then configures the estimate.
     *
     * Nothing is written until all four are accepted, so a rejected setting leaves the estimate whole and the block
     * keeps running on it.
     */
    void rebuild() {
        const detail::ManchesterInput mode = detail::manchesterInput(input.value);
        if (averaging_symbols.value < 1U) {
            throw gr::exception(std::format("averaging_symbols must be at least 1, got {}", averaging_symbols.value));
        }
        if (phase.value < -1 || phase.value > 1) {
            throw gr::exception(std::format("phase must be -1 to estimate the pairing, or 0 or 1 to pin it, got {}", phase.value));
        }
        _differential   = mode == detail::ManchesterInput::Differential;
        _pairing.alpha  = 1. / static_cast<double>(averaging_symbols.value);
        _pairing.hold   = static_cast<std::size_t>(hold_symbols.value);
        _pairing.pinned = phase.value >= 0;
        if (_pairing.pinned) {
            _pairing.phase = static_cast<unsigned>(phase.value);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& phaseSpan, OutputSpanLike auto& confidenceSpan) {
        const bool  wantPhase      = phaseSpan.isConnected;
        const bool  wantConfidence = confidenceSpan.isConnected;
        std::size_t nItems         = std::min(inSpan.size() / 2UZ, outSpan.size());
        nItems                     = wantPhase ? std::min(nItems, phaseSpan.size()) : nItems;
        nItems                     = wantConfidence ? std::min(nItems, confidenceSpan.size()) : nItems;

        // every item of a pair is two apart from its neighbor in the next pair, so one parity holds for the chunk
        const unsigned firstParity = static_cast<unsigned>(inSpan.streamIndex & 1ULL);

        for (std::size_t m = 0UZ; m < nItems; ++m) {
            const float first  = inSpan[2UZ * m];
            const float second = inSpan[2UZ * m + 1UZ];

            if (_differential) {
                _pairing.observe(firstParity, static_cast<double>(first));
                _pairing.observe(firstParity ^ 1U, static_cast<double>(second));
            } else {
                if (_havePrevious) {
                    _pairing.observe(firstParity, static_cast<double>(first) * static_cast<double>(_previous));
                }
                _pairing.observe(firstParity ^ 1U, static_cast<double>(second) * static_cast<double>(first));
            }
            _pairing.step(_differential);

            const bool  leads = firstParity == _pairing.phase;         // this chunk's first item opens the pair
            const float bit   = _differential                          //
                                    ? (leads ? first : second)         //
                                    : (leads ? 0.5f * (first - second) //
                                             : (_havePrevious ? 0.5f * (_previous - first) : 0.f));

            outSpan[m] = bit;
            if (wantPhase) {
                phaseSpan[m] = static_cast<float>(_pairing.phase);
            }
            if (wantConfidence) {
                confidenceSpan[m] = _pairing.confidence();
            }

            _previous     = second;
            _havePrevious = true;
        }

        outSpan.publish(nItems);
        phaseSpan.publish(wantPhase ? nItems : 0UZ);
        confidenceSpan.publish(wantConfidence ? nItems : 0UZ);
        std::ignore = inSpan.consume(2UZ * nItems);
        return work::Status::OK;
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_MANCHESTERCOMBINE_HPP
