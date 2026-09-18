#ifndef GNURADIO_DIGITAL_MANCHESTER_HPP
#define GNURADIO_DIGITAL_MANCHESTER_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/algorithm/digital/Manchester.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/digital/BitPacking.hpp>

namespace gr::blocks::digital {

namespace detail {

/// @brief The convention @p name selects, refusing the empty default and reporting through the graph's exception type.
[[nodiscard]] inline gr::digital::ManchesterConvention manchesterConvention(std::string_view name) {
    using gr::digital::ManchesterConvention;
    if (name == gr::digital::conventionName(ManchesterConvention::Ieee8023)) {
        return ManchesterConvention::Ieee8023;
    }
    if (name == gr::digital::conventionName(ManchesterConvention::GeThomas)) {
        return ManchesterConvention::GeThomas;
    }
    if (name.empty()) {
        throw gr::exception("convention has no default and must be set to 'ieee802_3' or 'ge_thomas': a stream read under the wrong one decodes to the exact complement of the data and raises no violation anywhere, so a guess is invisible");
    }
    throw gr::exception(std::format("convention must be 'ieee802_3' or 'ge_thomas', got '{}'", name));
}

/**
 * @brief Validates the four settings, then configures @p cfg.
 *
 * Everything is checked before the kernel is touched, so a rejected setting leaves the previous
 * configuration whole and the block keeps running on it. The kernel's own rejections are the same two
 * bounds and would throw `std::invalid_argument`; they are unreachable from here.
 */
inline void configureManchester(gr::digital::ManchesterConfig& cfg, std::string_view convention, gr::Size_t bitsPerItem, gr::digital::BitOrder order, gr::Size_t chipPhase) {
    const gr::digital::ManchesterConvention mode = manchesterConvention(convention);
    if (bitsPerItem < 1U || bitsPerItem > 8U) {
        throw gr::exception(std::format("bits_per_item must be in [1, 8], got {}", bitsPerItem));
    }
    if (chipPhase > 1U) {
        throw gr::exception(std::format("chip_phase must be 0 or 1, got {}", chipPhase));
    }
    gr::digital::configureManchester(cfg, mode, static_cast<unsigned>(bitsPerItem), order, static_cast<unsigned>(chipPhase));
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::digital::ManchesterEncoder)

struct ManchesterEncoder : Block<ManchesterEncoder, BackwardTagPropagation, Resampling<1UZ, 2UZ, true>> {
    using Description = Doc<R""(
@brief Manchester line coding: one data bit becomes two chips of opposite value, at twice the rate.

Every bit carries a transition at its center, the running disparity returns to zero at every bit boundary and no more
than two like chips ever occur in a row -- for any data, with no seed and no state. That is what buys a receiver a
clock to lock to and an AC-coupled line its DC balance, and it costs exactly one thing: twice the signaling rate.

`convention` is required and has no default. The two live readings of the code are exact chipwise complements of one
another, so a receiver on the wrong one is handed the bitwise complement of the data with no error indication of any
kind; nothing short of a downstream CRC can tell. IEEE Std 802.3 is `ieee802_3`, MIL-STD-1553 is `ge_thomas`, and
IEC 62386 lighting is `ieee802_3`.

An item carries `bits_per_item` significant bits in its low positions, traversed in `bit_order` on both ports. Bits
above that are masked away rather than rejected, because a stream value must not be able to stop a graph.

The block is exactly 1:2, so a forwarded `sample_rate` is doubled and a tag arriving at input item t leaves at output
item 2t. Being a rate changer it forwards only the framework's own tag vocabulary: a custom key does not survive it.
)"">;

    PortIn<std::uint8_t>  in;
    PortOut<std::uint8_t> out;

    Annotated<std::string, "convention", Doc<"'ieee802_3' or 'ge_thomas'; there is no default, because the wrong one is silent">, Visible> convention{};
    Annotated<gr::Size_t, "bits_per_item", Unit<"bit">, Doc<"significant bits per item, 1 to 8, on both ports">>                           bits_per_item = 8U;
    Annotated<gr::digital::BitOrder, "bit_order", Doc<"the traversal of each item's field, not the order of bytes">>                       bit_order     = gr::digital::BitOrder::MsbFirst;

    GR_MAKE_REFLECTABLE(ManchesterEncoder, in, out, convention, bits_per_item, bit_order);

    gr::digital::ManchesterConfig _coder{};
    bool                          _configured = false;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        detail::configureManchester(_coder, convention, bits_per_item, bit_order, 0U);
        _configured = true; // only reached when every setting was accepted
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) noexcept {
        if (!_configured) { // the block is inert rather than coding under a convention nobody chose
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        const std::size_t nItems = std::min(inSpan.size(), outSpan.size() / 2UZ);
        gr::digital::encodeManchester(_coder, std::span<const std::uint8_t>(inSpan.data(), nItems), std::span<std::uint8_t>(outSpan.data(), 2UZ * nItems));

        outSpan.publish(2UZ * nItems);
        std::ignore = inSpan.consume(nItems);
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::digital::ManchesterDecoder)

struct ManchesterDecoder : Block<ManchesterDecoder, BackwardTagPropagation, Resampling<2UZ, 1UZ, true>> {
    using Description = Doc<R""(
@brief Manchester line decoding: two chips become one data bit, and a flag saying whether they were a legal pair.

The decoded bit is the first chip of the pair corrected by the convention, and the second chip only sets the
violation flag. On a legal pair that is the same answer as any other reading; on a chip pair of `00` or `11` -- which
an encoder cannot produce -- it is a stated convention that makes `(out, violation)` a lossless recoding of the chip
stream and leaves the decoder one branch-free expression.

Violations are reported and never absorbed, corrected or resynchronized on. MIL-STD-1553's syncs and IEEE 802.5's
J/K delimiters are invalid Manchester waveforms on purpose, so a decoder that realigned itself on a violation would
fail hardest on the protocols that use the feature. What to do about one belongs to whatever knows the protocol; the
`violation` port is how it finds out, and it may be left unconnected by a chain that does not care.

`chip_phase` decides which chip starts a pair, and a tag at `align_tag_key` re-anchors that grid so the tagged item's
first chip begins a pair, effective at the next two-item chunk boundary. The move costs exactly one chip in either
direction -- to phase 1 the chip at the boundary leaves as a flagged orphan, to phase 0 the held chip is dropped --
which is the only assignment that keeps the block exactly one output item per two input items. Both events are
counted, and `stop()` reports a non-zero drop.

Misalignment is loud in proportion to the data's transition density and completely silent on a constant stream,
where it produces the complement of the data: the same symptom as reading the wrong `convention`, which is why that
setting is required and has no default. See ManchesterEncoder for the conventions and the item model.

The block is exactly 2:1, so a forwarded `sample_rate` is halved and a tag arriving at input item t leaves at output
item floor(t/2). Being a rate changer it forwards the framework's own tag vocabulary only.
)"">;

    PortIn<std::uint8_t>                in;
    PortOut<std::uint8_t>               out;
    PortOut<std::uint8_t, gr::Optional> violation;

    Annotated<std::string, "convention", Doc<"'ieee802_3' or 'ge_thomas'; there is no default, because the wrong one is silent">, Visible>              convention{};
    Annotated<gr::Size_t, "bits_per_item", Unit<"bit">, Doc<"significant bits per item, 1 to 8, on every port">>                                        bits_per_item = 8U;
    Annotated<gr::digital::BitOrder, "bit_order", Doc<"the traversal of each item's field, not the order of bytes">>                                    bit_order     = gr::digital::BitOrder::MsbFirst;
    Annotated<gr::Size_t, "chip_phase", Doc<"0 or 1: the parity of the absolute chip index at which a pair starts">>                                    chip_phase    = 0U;
    Annotated<std::string, "align_tag_key", Doc<"tag key whose position re-anchors the pairing grid; empty disables tag-driven alignment">>             align_tag_key = std::string(gr::tag::TRIGGER_NAME.shortKey());
    Annotated<std::string, "align_tag_value", Doc<"if non-empty, the tag's value at that key must be a string equal to this; empty matches any value">> align_tag_value{};
    Annotated<bool, "flush_partial", Doc<"at end of stream emit the trailing partial item, its pad positions flagged as violations">>                   flush_partial = true;

    GR_MAKE_REFLECTABLE(ManchesterDecoder, in, out, violation, convention, bits_per_item, bit_order, chip_phase, align_tag_key, align_tag_value, flush_partial);

    gr::digital::ManchesterConfig _coder{};
    bool                          _configured = false;
    std::vector<std::uint8_t>     _spare{}; ///< flags for a chunk an unconnected violation port could not hold

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        // only the four settings that move the pairing grid reconfigure the kernel, and the kernel's own copy of them
        // is what says whether one did. A carry chip held across a change of align_tag_key, align_tag_value or
        // flush_partial is still the chip it was, because those select a boundary or a tail rule rather than move one.
        if (_configured                                                           //
            && convention.value == gr::digital::conventionName(_coder.convention) //
            && bits_per_item.value == static_cast<gr::Size_t>(_coder.bitsPerItem) //
            && bit_order.value == _coder.order                                    //
            && chip_phase.value == static_cast<gr::Size_t>(_coder.chipPhase)) {
            return;
        }
        rebuild();
    }

    void start() { rebuild(); }

    void reset() { gr::digital::resetManchester(_coder); }

    void rebuild() {
        detail::configureManchester(_coder, convention, bits_per_item, bit_order, chip_phase);
        _configured = true;
    }

    /// @brief Items fabricated by a grid change to phase 1, cumulative across settings changes and resets.
    [[nodiscard]] std::uint64_t nOrphanItems() const noexcept { return _coder.nOrphanItems; }

    /// @brief Chips dropped by a grid change to phase 0 or by an unflushed tail, cumulative.
    [[nodiscard]] std::uint64_t nDroppedChips() const noexcept { return _coder.nDroppedChips; }

    void stop() {
        if (_coder.hasCarry) { // the stream ended on a chunk boundary, so no epilogue ran to pair this chip
            _coder.carry    = 0U;
            _coder.hasCarry = false;
            ++_coder.nDroppedChips;
        }
        if (_coder.nDroppedChips != 0ULL) {
            std::println(stderr, "gr::blocks::digital::ManchesterDecoder: dropped {} chip(s) at grid changes and at end of stream", _coder.nDroppedChips);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& violationSpan) {
        if (!_configured) { // the block is inert rather than decoding under a convention nobody chose
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            violationSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        const bool        wanted = violationSpan.isConnected;
        const std::size_t nItems = wanted ? std::min({inSpan.size() / 2UZ, outSpan.size(), violationSpan.size()}) : std::min(inSpan.size() / 2UZ, outSpan.size());

        const std::span<const std::uint8_t> chips(inSpan.data(), 2UZ * nItems);
        const std::span<std::uint8_t>       bits(outSpan.data(), nItems);
        const std::span<std::uint8_t>       flags = flagRoom(violationSpan, nItems);

        std::size_t done = 0UZ;
        if (!align_tag_key.value.empty()) {
            for (const auto& [relIndex, tagMap] : inSpan.tags(inSpan.size())) {
                // a tag interior to a chunk that could not be broken at it returns in the next chunk at a negative
                // relative index; it was acted on the first time and must not move the grid twice
                if (relIndex < 0 || !isAlignmentTag(tagMap.get())) {
                    continue;
                }
                const std::size_t   item      = static_cast<std::size_t>(relIndex);
                const std::size_t   at        = std::min(nItems, (item + 1UZ) / 2UZ); // the chunk boundary at or after the tagged item
                const unsigned      width     = _coder.bitsPerItem;
                const std::uint64_t chipIndex = inSpan.streamIndex + item; // the tagged item's first chip, whose parity is the request
                decodeRange(chips, bits, flags, done, at);
                done        = at;
                std::ignore = gr::digital::realignManchester(_coder, chipIndex * width);
            }
        }
        decodeRange(chips, bits, flags, done, nItems);

        outSpan.publish(nItems);
        violationSpan.publish(wanted ? nItems : 0UZ);
        std::ignore = inSpan.consume(2UZ * nItems);
        return work::Status::OK;
    }

    /**
     * @brief The trailing input the 2:1 chunking could not pair, plus the held chip.
     *
     * Under `flush_partial` the complete pairs fill the first positions of one more item and the positions no chip
     * reached are zero in `out` and set in `violation`, which is the marking a pad has: a pad position is by
     * definition not a decoded bit and the violation stream is exactly the channel that says so. A final unpaired
     * chip leaves as a flagged orphan. Otherwise the trailing chips are dropped and counted, and `stop()` reports
     * the total. Nothing here is on the sample path.
     */
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& violationSpan) {
        const unsigned    width      = static_cast<unsigned>(_coder.bitsPerItem);
        const std::size_t totalChips = _configured ? inSpan.size() * width + (_coder.hasCarry ? 1UZ : 0UZ) : 0UZ;
        if (totalChips == 0UZ) {
            outSpan.publish(0UZ);
            violationSpan.publish(0UZ);
            return work::Status::OK;
        }

        const bool        owedOrphan  = _coder.gridParity == 1U && !_coder.hasCarry;
        const std::size_t afterOrphan = totalChips - (owedOrphan ? 1UZ : 0UZ);
        const std::size_t nBits       = (owedOrphan ? 1UZ : 0UZ) + afterOrphan / 2UZ + afterOrphan % 2UZ;
        const std::size_t nItems      = (nBits + width - 1UZ) / width;

        const bool        wanted = violationSpan.isConnected;
        const std::size_t room   = wanted ? std::min(outSpan.size(), violationSpan.size()) : outSpan.size();

        if (!flush_partial || nItems > room) {
            _coder.nDroppedChips += totalChips;
            _coder.carry    = 0U;
            _coder.hasCarry = false;
            outSpan.publish(0UZ);
            violationSpan.publish(0UZ);
            return work::Status::OK;
        }

        const std::span<std::uint8_t> flags    = flagRoom(violationSpan, nItems);
        const unsigned                k        = static_cast<unsigned>(_coder.k);
        const bool                    msb      = _coder.order == gr::digital::BitOrder::MsbFirst;
        const auto                    position = [width, msb](unsigned index) noexcept { return msb ? width - 1U - index : index; };

        std::size_t  published = 0UZ;
        unsigned     bits      = 0U;
        unsigned     marks     = 0U;
        unsigned     slot      = 0U;
        std::uint8_t carry     = _coder.carry;
        bool         hasCarry  = _coder.hasCarry;
        bool         orphan    = owedOrphan;

        const auto place = [&](unsigned bit, unsigned flag) noexcept {
            bits |= bit << position(slot);
            marks |= flag << position(slot);
            if (++slot == width) {
                outSpan[published] = static_cast<std::uint8_t>(bits);
                flags[published]   = static_cast<std::uint8_t>(marks);
                ++published;
                bits  = 0U;
                marks = 0U;
                slot  = 0U;
            }
        };

        for (std::size_t n = 0UZ; n < inSpan.size(); ++n) {
            for (unsigned i = 0U; i < width; ++i) {
                const unsigned chip = (static_cast<unsigned>(inSpan[n]) >> position(i)) & 1U;
                if (orphan) {
                    place(chip ^ k, 1U);
                    orphan = false;
                    ++_coder.nOrphanItems;
                } else if (!hasCarry) {
                    carry    = static_cast<std::uint8_t>(chip);
                    hasCarry = true;
                } else {
                    place(static_cast<unsigned>(carry) ^ k, 1U - (static_cast<unsigned>(carry) ^ chip));
                    hasCarry = false;
                }
            }
        }
        if (hasCarry) {
            place(static_cast<unsigned>(carry) ^ k, 1U);
            ++_coder.nOrphanItems;
        }
        while (slot != 0U) {
            place(0U, 1U);
        }

        _coder.carry    = 0U;
        _coder.hasCarry = false;
        outSpan.publish(published);
        violationSpan.publish(wanted ? published : 0UZ);
        return work::Status::OK;
    }

    /// @brief Whether @p tagMap asks for the grid to be re-anchored at its own item.
    [[nodiscard]] bool isAlignmentTag(const property_map& tagMap) const {
        const auto entry = tagMap.find(property_map::key_type(align_tag_key.value));
        if (entry == tagMap.end()) {
            return false;
        }
        if (align_tag_value.value.empty()) {
            return true;
        }
        return entry->second.value_or(std::string_view{}) == std::string_view(align_tag_value.value);
    }

    /**
     * @brief Where the flags of @p nItems go.
     *
     * An unconnected optional output keeps the default buffer the graph never resized, so its span is the reserved
     * size while the request fits and empty once it does not. Both are written the same way, and the spare only
     * grows to the largest chunk that ever missed.
     */
    [[nodiscard]] std::span<std::uint8_t> flagRoom(OutputSpanLike auto& violationSpan, std::size_t nItems) {
        if (violationSpan.size() >= nItems) {
            return {violationSpan.data(), nItems};
        }
        _spare.resize(nItems);
        return {_spare};
    }

    void decodeRange(std::span<const std::uint8_t> chips, std::span<std::uint8_t> bits, std::span<std::uint8_t> flags, std::size_t from, std::size_t to) noexcept {
        if (to <= from) {
            return;
        }
        const std::size_t count = to - from;
        gr::digital::decodeManchester(_coder, chips.subspan(2UZ * from, 2UZ * count), bits.subspan(from, count), flags.subspan(from, count));
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_MANCHESTER_HPP
