#ifndef GNURADIO_DIGITAL_SCRAMBLER_HPP
#define GNURADIO_DIGITAL_SCRAMBLER_HPP

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <limits>
#include <print>
#include <ranges>
#include <span>
#include <stdexcept>
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

#include <gnuradio-4.0/algorithm/digital/BitPacking.hpp>

#include <gnuradio-4.0/algorithm/digital/Scrambler.hpp>

namespace gr::blocks::digital {

namespace detail {

/// @brief The settings the generator is built from, which are exactly the ones a change to reconfigures and resets.
/// `reset_tag_key` and `reset_tag_value` are deliberately absent: they select an epoch boundary rather than move one.
struct ScramblerSettings {
    std::string           profile{};
    std::string           taps{};
    std::string           seed{};
    std::string           sequence{};
    bool                  sequenceRepeat = true;
    gr::Size_t            bitsPerItem    = 8U;
    gr::digital::BitOrder bitOrder       = gr::digital::BitOrder::MsbFirst;
    gr::Size_t            resetPeriod    = 0U;
    bool                  invertOutput   = false;
    gr::Size_t            forceAfter     = 0U;
    std::string           monitorDelays{};

    [[nodiscard]] bool operator==(const ScramblerSettings&) const = default;
};

/// @brief What a scrambler block holds: the configured generator, the settings it was built from, the tag that marks
/// an epoch boundary, and the items since the last reset of any cause.
///
/// `configured` and not `config.degree` says whether a generator is in place, because an explicit sequence has no
/// register and therefore no degree, and a block carrying one is configured all the same.
struct ScramblerState {
    gr::digital::ScramblerConfig config{};
    ScramblerSettings            applied{};
    property_map::key_type       resetKey{};
    std::string                  resetValue{};
    std::uint64_t                sinceReset = 0ULL;
    bool                         configured = false;
};

/**
 * @brief Validates @p wanted and, if it moves the generator, rebuilds @p state from it and resets the register.
 *
 * Every rejection throws `gr::exception` quoting the value that caused it, so an invalid configuration stops the graph
 * before it runs and no invalid configuration reaches the kernel. The kernel's own parsers and resolvers do the
 * reading, their `std::invalid_argument` rewrapped under the setting's name rather than a second parser being written
 * beside them. The generator is built beside the running one and moved into place only once every value has been
 * accepted, so a rejected setting leaves the configuration it was applied to intact rather than half moved.
 *
 * Three mask sources are alternatives and at most one may be staged: `taps` with `seed`, a named `profile`, or an
 * explicit `sequence`. A profile fills `taps`, `seed`, `bits_per_item` and `bit_order` outright, so a value staged
 * beside it would leave two answers to one question; `bits_per_item` and `bit_order` carry defaults and cannot be told
 * from an explicit staging, so the profile's values simply win there and the refusal covers `taps`, `seed` and
 * `sequence`, which are the settings that are empty until somebody names them.
 */
inline void configureScrambler(ScramblerState& state, const ScramblerSettings& wanted, gr::digital::ScramblerMode mode) {
    const std::string_view profileName(wanted.profile);
    const std::string_view tapList(wanted.taps);
    const std::string_view seedBits(wanted.seed);
    const std::string_view sequenceText(wanted.sequence);
    const gr::Size_t       bitsPerItem = wanted.bitsPerItem;

    if (!profileName.empty()) {
        const auto refuseMerge = [profileName](std::string_view named, std::string_view value) { throw gr::exception(std::format("profile \"{}\" names the generator, so {} \"{}\" cannot be staged beside it: there is one source of truth per configured block", profileName, named, value)); };
        if (!tapList.empty()) {
            refuseMerge("taps", tapList);
        }
        if (!seedBits.empty()) {
            refuseMerge("seed", seedBits);
        }
        if (!sequenceText.empty()) {
            refuseMerge("sequence", sequenceText);
        }
    }
    if (!tapList.empty() && !sequenceText.empty()) {
        throw gr::exception(std::format("taps \"{}\" walks a recursion and sequence \"{}\" supplies the mask outright: they are alternatives and only one of them may be staged", tapList, sequenceText));
    }
    if (profileName.empty() && tapList.empty() && sequenceText.empty()) {
        throw gr::exception("taps names the feedback delays as a decimal list such as \"1,3,5,8\" and has no default: a scrambler is an agreement between two ends, and an arbitrary polynomial guarantees only that the other end disagrees; profile names a published agreement instead, and sequence supplies the mask outright");
    }

    if (bitsPerItem < 1U || bitsPerItem > 8U) {
        throw gr::exception(std::format("bits_per_item is the significant bits of an item and must be in [1, 8], got {}", bitsPerItem));
    }

    const gr::digital::BitOrder order = wanted.bitOrder;

    if (state.configured && wanted == state.applied) {
        return; // nothing the generator is built from moved, so the register stays where the stream left it
    }

    gr::digital::ScramblerConfig candidate{};
    if (!sequenceText.empty()) {
        std::vector<std::uint8_t> bytes;
        try {
            bytes = gr::digital::sequenceFromHex(sequenceText);
            gr::digital::configureExplicit(candidate, std::span<const std::uint8_t>(bytes), bytes.size() * 8UZ, bitsPerItem, order, wanted.sequenceRepeat);
        } catch (const std::invalid_argument& reason) {
            throw gr::exception(std::format("setting sequence: {}", reason.what()));
        }
    } else if (!profileName.empty()) {
        try {
            gr::digital::applyProfile(candidate, profileName, mode);
        } catch (const std::invalid_argument& reason) {
            throw gr::exception(std::format("setting profile: {}", reason.what()));
        }
    } else {
        gr::digital::TapMask taps = 0ULL;
        try {
            taps = gr::digital::tapsFromDelayList(tapList);
        } catch (const std::invalid_argument& reason) {
            throw gr::exception(std::format("setting taps: {}", reason.what()));
        }
        const auto degree = gr::digital::degreeOf(taps);

        std::uint64_t seed = 0ULL;
        if (seedBits.empty()) {
            // an all-zero multiplicative register is a legitimate starting history whose influence is gone `degree` bits
            // later, which is why the empty spelling means all zeros there and is refused here
            if (mode == gr::digital::ScramblerMode::Additive) {
                throw gr::exception(std::format("seed is the first {} bits of the sequence and has no default: the two ends of an additive link must start at the same phase", degree));
            }
        } else {
            try {
                seed = gr::digital::seedFromBitString(seedBits, degree);
            } catch (const std::invalid_argument& reason) {
                throw gr::exception(std::format("setting seed: {}", reason.what()));
            }
            if (mode == gr::digital::ScramblerMode::Additive && seed == 0ULL) {
                throw gr::exception(std::format("seed \"{}\" is all zeros, which is a dead generator: an all-zero additive register produces an all-zero sequence for ever and the block would silently pass its input through", seedBits));
            }
        }
        gr::digital::configureScrambler(candidate, taps, seed, mode, bitsPerItem, order);
    }

    // the forced-transition group is the multiplicative pair's alone and is applied last, because every generator call
    // above puts it back to disabled
    if (mode != gr::digital::ScramblerMode::Additive) {
        try {
            gr::digital::configureMonitor(candidate, wanted.invertOutput, static_cast<std::uint32_t>(wanted.forceAfter), wanted.monitorDelays);
        } catch (const std::invalid_argument& reason) {
            throw gr::exception(std::format("setting the forced-transition group: {}", reason.what()));
        }
    }

    // a register carried across a polynomial change would be meaningless, so a change that reconfigures is a
    // discontinuity: the new configuration takes effect at the first item of the call the change lands in
    state.config     = std::move(candidate);
    state.applied    = wanted;
    state.sinceReset = 0ULL;
    state.configured = true;
}

/**
 * @brief Refuses an explicit sequence a periodic epoch is known in advance to outlast.
 *
 * `sequence_repeat` false leaves everything past the sequence's `S` bits unscrambled, and with a non-zero
 * `reset_period` the shortfall is arithmetic rather than a property of the stream: it is known before the graph runs
 * and is a refusal naming `S`, the period and the shortfall. The dynamic case -- a tag-driven epoch that outlasts the
 * sequence -- cannot be known at configure time and is what `nUnscrambledItems` counts.
 */
inline void checkSequenceCoversEpoch(const ScramblerState& state) {
    if (state.config.source != gr::digital::SequenceSource::Explicit || state.config.sequenceRepeat) {
        return;
    }
    const std::uint64_t period = state.applied.resetPeriod;
    if (period == 0ULL) {
        return;
    }

    const std::uint64_t epochBits = period * static_cast<std::uint64_t>(state.config.bitsPerItem);
    const auto          have      = static_cast<std::uint64_t>(state.config.tablePeriod);
    if (epochBits > have) {
        throw gr::exception(std::format("sequence holds {} bits and sequence_repeat is false, so a reset_period of {} items at {} bits each leaves the last {} bits of every epoch unscrambled", have, period, state.config.bitsPerItem, epochBits - have));
    }
}

/// @brief Whether @p map marks an epoch boundary: it carries @p key and, when @p value is non-empty, a string equal to
/// it at that key. A value of any other type never matches a narrowing value, and any value matches an empty one.
[[nodiscard]] inline bool marksEpoch(const property_map& map, const property_map::key_type& key, std::string_view value) {
    const auto entry = map.find(key);
    if (entry == map.end()) {
        return false;
    }
    return value.empty() || entry->second.value_or(std::string_view{}) == value;
}

/**
 * @brief One call of a scrambler block: the items between epoch boundaries scrambled a run at a time.
 *
 * A reset applies before the item it lands on, so that item is the first of the new epoch and off by one here would
 * cost the whole frame. At most one reset happens at any item: a tag arriving exactly on a periodic boundary is one
 * reset, and because it zeroes the counter the next periodic boundary is `reset_period` items after the tag rather
 * than after the boundary it replaced. A tag is read and forwarded, never consumed, and one from before the window
 * carries a negative relative index and is skipped.
 */
template<typename TInput, typename TOutput>
[[nodiscard]] work::Status processScrambler(ScramblerState& state, TInput& inSpan, TOutput& outSpan) {
    constexpr std::size_t kNoBoundary = std::numeric_limits<std::size_t>::max();

    if (!state.configured) { // never configured, so inert rather than a pass-through nothing downstream could detect
        std::ignore = inSpan.consume(0UZ);
        outSpan.publish(0UZ);
        return work::Status::ERROR;
    }

    const std::size_t   nItems = std::min(inSpan.size(), outSpan.size());
    const std::uint64_t period = state.applied.resetPeriod;

    auto       tagView = inSpan.tags();
    auto       tag     = std::ranges::begin(tagView);
    const auto tagEnd  = std::ranges::end(tagView);

    // the relative index of the next tag that marks a boundary, leaving the iterator on it so the caller can step past
    const auto nextBoundary = [&]() -> std::size_t {
        if (state.resetKey.empty()) { // the mechanism is disabled entirely, for a chain that must not follow a detector
            return kNoBoundary;
        }
        for (; tag != tagEnd; ++tag) {
            const auto& [relIndex, map] = *tag;
            if (relIndex >= static_cast<std::ptrdiff_t>(nItems)) {
                break;
            }
            if (relIndex >= 0 && marksEpoch(map.get(), state.resetKey, state.resetValue)) {
                return static_cast<std::size_t>(relIndex);
            }
        }
        return kNoBoundary;
    };

    std::size_t done     = 0UZ;
    std::size_t boundary = nextBoundary();
    while (done < nItems) {
        bool epoch = false;
        while (boundary != kNoBoundary && boundary <= done) {
            epoch = true;
            ++tag;
            boundary = nextBoundary();
        }
        if (period != 0ULL && state.sinceReset >= period) {
            epoch = true;
        }
        if (epoch) {
            gr::digital::resetScrambler(state.config);
            state.sinceReset = 0ULL;
        }

        std::size_t limit = boundary == kNoBoundary ? nItems : std::min(nItems, boundary);
        if (period != 0ULL) {
            limit = std::min(limit, done + static_cast<std::size_t>(period > state.sinceReset ? period - state.sinceReset : 0ULL));
        }

        const std::size_t count = limit - done;
        gr::digital::scramble(state.config, std::span<const std::uint8_t>(inSpan.data() + done, count), std::span<std::uint8_t>(outSpan.data() + done, count));
        state.sinceReset += count;
        done = limit;
    }

    std::ignore = inSpan.consume(nItems);
    outSpan.publish(nItems);
    return work::Status::OK;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::digital::AdditiveScrambler, [T], [ std::uint8_t, gr::DataSet<std::uint8_t> ])

template<typename T>
requires std::same_as<T, std::uint8_t> || std::same_as<T, DataSet<std::uint8_t>>
struct AdditiveScrambler : std::conditional_t<std::same_as<T, std::uint8_t>, Block<AdditiveScrambler<T>, UnfilteredTagPropagation>, Block<AdditiveScrambler<T>>> {
    using Description = Doc<R""(
@brief XORs the stream with a free-running LFSR sequence, so that what goes on the air looks like noise whatever the data is.

A transmitter sending a long run of identical bits gives the clock recovery loop nothing to lock to, drifts the AGC,
feeds the DC blocker real signal and grows discrete lines where the spectrum should be flat. This block fixes all of it
in one place. The sequence is a function of `taps` and `seed` alone, so the whitening does not depend on the data and a
channel bit error costs exactly one output bit error, which is as well behaved as anything in a receive chain gets.

The price is that both ends must agree on the phase: one bit of phase error leaves the descrambled stream agreeing with
the truth at chance, and nothing inside the block can tell, because there is nothing to compare against. The detection
belongs downstream at a CRC. Three mechanisms reach the epoch boundary the two ends agree on — `start()` and any
setting that reconfigures the generator; a tag carrying `reset_tag_key`, which resets before the tagged item so that
item is the first of the new epoch; and `reset_period` items since the last reset of any cause. At most one reset
happens at any item, and a tag re-anchors the period. The receive side finds its boundary in a detector's
`trigger_name` tag, which is the default key; the transmit side counts it, which is what `reset_period` is for.

XOR is an involution, so this block descrambles as well as it scrambles and there is no second name for the same code.

`taps` is a list of feedback delays rather than a polynomial's exponents, because a three-term polynomial is written
both ways and the standards disagree on which their spelling means: CCSDS's `x^8 + x^7 + x^5 + x^3 + 1` is `"1,3,5,8"`
while IEEE 802.11's `x^7 + x^4 + 1` is `"4,7"`, each confirmed by the sequence its own standard publishes. There is no
default polynomial and no default seed, and the block refuses to start without both, because a scrambler is an
agreement between two ends and an arbitrary default guarantees only that the other end disagrees.

Two other ways of naming the same mask are alternatives to `taps` and `seed`, and staging more than one of the three is
a refusal rather than a merge. `profile` names a published agreement -- `ccsds131`, `ccsds131_17` or `pn9` -- and fills
`taps`, `seed`, `bits_per_item` and `bit_order` from it; it is never a default, and the multiplicative `g3ruh` is
refused here naming its family, because a self-synchronizing agreement run additively is a silent wrong answer.
`sequence` supplies the mask outright as hexadecimal bytes, which is how about half the vendor deframers state theirs.
An explicit sequence of `S` bits either tiles the epoch, which is what a period-`S` recursion would do and cannot lose
whitening, or covers its first `S` bits and leaves the rest unscrambled -- every such item counted in
`nUnscrambledItems` and stated at `stop()`, since a 1:1 block cannot drop an item. Where a periodic epoch is known in
advance to outlast the sequence the shortfall is arithmetic and `start()` refuses it.

The block is 1:1, so every input tag key passes through at its own offset; the reset tag is read and forwarded, never
consumed, and `sample_rate` leaves with the value it arrived with.

The carrier is a bit stream or a bit record. On the `DataSet<std::uint8_t>` instantiation one record is one epoch —
the generator resets at the first item of every record, which is 10.4.3's own rule once extraction has made one
record one codeblock — so no tag can reset on the wrong detector and no period needs counting: `reset_tag_key`,
`reset_tag_value` and `reset_period` are inert there. The record's metadata, name and shape cross verbatim, XOR
being an involution with no status to report.
)"">;

    static constexpr bool kRecordCarrier = std::same_as<T, DataSet<std::uint8_t>>;

    std::conditional_t<kRecordCarrier, PortIn<T, Async>, PortIn<T>>   in;
    std::conditional_t<kRecordCarrier, PortOut<T, Async>, PortOut<T>> out;

    Annotated<std::string, "profile", Doc<"a published generator by name: 'ccsds131', 'ccsds131_17', 'pn9' or 'si4463', filling taps, seed, bits_per_item and bit_order; empty selects none">, Visible> profile{};
    Annotated<std::string, "taps", Doc<"feedback delays, decimal and comma-separated as in \"1,3,5,8\"; each in [1,64], distinct, at least one, and there is no default">, Visible>                     taps{};
    Annotated<std::string, "seed", Doc<"the first `degree` bits of the sequence, '0'/'1' leftmost first, exactly `degree` of them; all zeros is a dead generator and is refused">, Visible>             seed{};
    Annotated<std::string, "sequence", Doc<"the mask outright, as hexadecimal byte pairs such as \"FF E1 1D 9A\" read MSB first, 1 to 2^20 bits; an alternative to taps and to profile">, Visible>      sequence{};
    Annotated<bool, "sequence_repeat", Doc<"whether an explicit sequence tiles the epoch; false covers its first S bits and counts every item past them in nUnscrambledItems">>                         sequence_repeat = true;
    Annotated<gr::Size_t, "bits_per_item", Unit<"bit">, Doc<"significant bits per item, [1,8]; an item's remaining bits are ignored on input and zero on output">>                                      bits_per_item   = 8U;
    Annotated<gr::digital::BitOrder, "bit_order", Doc<"how each item's field is traversed, which is not a byte order">>                                                                                 bit_order       = gr::digital::BitOrder::MsbFirst;
    Annotated<std::string, "reset_tag_key", Doc<"tag key marking an epoch boundary, the reset applying before the tagged item; empty disables the tag-driven reset">>                                   reset_tag_key   = std::string(gr::tag::TRIGGER_NAME.shortKey());
    Annotated<std::string, "reset_tag_value", Doc<"if non-empty, the tag's value at that key must be a string equal to this, which narrows the boundary to one detector; empty matches any">>           reset_tag_value{};
    Annotated<gr::Size_t, "reset_period", Unit<"items">, Doc<"items per epoch, counted from the last reset of any cause; 0 disables the periodic reset">>                                               reset_period = 0U;

    GR_MAKE_REFLECTABLE(AdditiveScrambler, in, out, profile, taps, seed, sequence, sequence_repeat, bits_per_item, bit_order, reset_tag_key, reset_tag_value, reset_period);

    detail::ScramblerState _state{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        _state.resetKey   = property_map::key_type(reset_tag_key.value);
        _state.resetValue = reset_tag_value.value;
        rebuild();
    }

    void start() {
        rebuild();
        detail::checkSequenceCoversEpoch(_state);
        reset();
    }

    void reset() {
        gr::digital::resetScrambler(_state.config);
        _state.sinceReset = 0ULL;
    }

    void rebuild() { detail::configureScrambler(_state, detail::ScramblerSettings{profile.value, taps.value, seed.value, sequence.value, sequence_repeat.value, bits_per_item.value, bit_order.value, reset_period.value, false, 0U, {}}, gr::digital::ScramblerMode::Additive); }

    /// @brief Items a non-repeating explicit sequence could not cover, cumulative over the configuration's life.
    /// Structurally zero without one, since a recursion has no end to run past.
    [[nodiscard]] std::uint64_t nUnscrambledItems() const noexcept { return static_cast<std::uint64_t>(_state.config.nUnscrambledItems); }

    /// @brief Zero here: the forced-transition rule belongs to the multiplicative pair, which has feedback for it to
    /// act on. Read alongside its sibling so one reader shape serves all three blocks.
    [[nodiscard]] std::uint64_t nForcedTransitions() const noexcept { return static_cast<std::uint64_t>(_state.config.nForcedTransitions); }

    void stop() {
        if (_state.config.nUnscrambledItems != 0UZ) {
            std::println(stderr, "gr::blocks::digital::AdditiveScrambler '{}': {} item(s) past the end of the explicit sequence left unscrambled", this->name, _state.config.nUnscrambledItems);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if constexpr (!kRecordCarrier) {
            return detail::processScrambler(_state, inSpan, outSpan);
        } else {
            if (!_state.configured) { // never configured, so inert rather than a pass-through nothing downstream could detect
                std::ignore = inSpan.consume(0UZ);
                outSpan.publish(0UZ);
                return work::Status::ERROR;
            }

            std::size_t consumed = 0UZ;
            std::size_t made     = 0UZ;
            for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
                const DataSet<std::uint8_t>& record = inSpan[consumed];
                DataSet<std::uint8_t>        masked = record; // the record's facts cross verbatim; only the values move
                gr::digital::resetScrambler(_state.config);   // one record is one epoch, at the record's first item
                gr::digital::scramble(_state.config, std::span<const std::uint8_t>(record.signal_values), std::span<std::uint8_t>(masked.signal_values));
                outSpan[made] = std::move(masked);
                ++made;
            }

            std::ignore = inSpan.consume(consumed);
            outSpan.publish(made);
            if (made == 0UZ && consumed == 0UZ) {
                return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
            }
            return work::Status::OK;
        }
    }
};

GR_REGISTER_BLOCK(gr::blocks::digital::MultiplicativeScrambler)

struct MultiplicativeScrambler : Block<MultiplicativeScrambler, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief Feeds the transmitted bits back through the shift register: `out[k] = in[k] XOR f(out, k)`.

The transmit half of the self-synchronizing pair. Where an additive scrambler needs the two ends to agree on a phase,
this one needs no boundary agreement at all: MultiplicativeDescrambler's register holds only received bits, so its
output is correct `degree` bits after it starts, from any state and from any offset into the stream. That is what a
continuous link with no framing to lean on uses, and it is why the V-series modems and 802.3's 64B/66B scramble this
way while CCSDS, DVB and 802.11's OFDM PHY, all framed, scramble additively. The polynomial does not decide which
family a link is in: 802.11 uses the same `"4,7"` additively in its OFDM PHY and self-synchronizingly in its DSSS PHY.

It is paid for twice. One channel bit error becomes `1 + tapCount` errors at the descrambler, which is the number that
decides whether the scrambler goes inside or outside a forward error correcting code. And the whitening depends on the
data, unlike the additive family's: an all-zero register carrying an all-zero input is a fixed point that emits zeros
for ever, and for any seed there is an input -- one equal to the feedback at every step -- that holds the output at a
constant for as long as it continues. The V-series answer to that is `force_transition_after` with `monitor_delays`,
which both ends compute from the transmitted stream alone: the counter runs while `y[k-p] XOR y[k-q]` stays zero and a
complement is injected at the `N`-th such item. Because the monitor reads only what is on the wire, the descrambler
computes the same counter from the same bits and stays exactly this block's inverse -- so the rule is a setting after
all, not a protocol. What it cannot do is bound the output's run length: the pair is a bijection on bit streams, so for
every output, the constant one included, there is exactly one input producing it. What it does do is destroy the
lock-up: fed the plain recursion's own lock-up input, the output stops being constant within `N` items.
`nForcedTransitions` counts the firings, and a chain whose count is zero is a chain that could have set `N` to zero.

`seed` is the `degree` bits assumed to precede the stream, oldest first, and an empty seed means all zeros and is
legal, because those bits reach only the first `degree` bits of an epoch. Resetting therefore buys reproducibility
rather than correctness: the same data with the same seed at the same boundaries produces the same transmitted bits.

`profile` names a published agreement -- `g3ruh`, the 9600-baud packet-radio scrambler -- and fills `taps`, `seed`,
`bits_per_item` and `bit_order` from it, refusing to merge with any of them and refusing an additive profile naming its
family. `taps`, the three reset mechanisms and the tag policy are AdditiveScrambler's, unchanged.

The block is 1:1, so every input tag key passes through at its own offset; the reset tag is read and forwarded, never
consumed, and `sample_rate` leaves with the value it arrived with.
)"">;

    PortIn<std::uint8_t>  in;
    PortOut<std::uint8_t> out;

    Annotated<std::string, "profile", Doc<"a published generator by name: 'g3ruh', filling taps, seed, bits_per_item and bit_order; an additive profile is refused naming its family; empty selects none">, Visible>         profile{};
    Annotated<std::string, "taps", Doc<"feedback delays, decimal and comma-separated as in \"18,23\"; each in [1,64], distinct, at least one, and there is no default">, Visible>                                            taps{};
    Annotated<std::string, "seed", Doc<"the `degree` bits assumed to precede the stream, '0'/'1' oldest first; empty means all zeros, which is a legitimate history here">, Visible>                                         seed{};
    Annotated<gr::Size_t, "bits_per_item", Unit<"bit">, Doc<"significant bits per item, [1,8]; an item's remaining bits are ignored on input and zero on output">>                                                           bits_per_item          = 8U;
    Annotated<gr::digital::BitOrder, "bit_order", Doc<"how each item's field is traversed, which is not a byte order">>                                                                                                      bit_order              = gr::digital::BitOrder::MsbFirst;
    Annotated<bool, "invert_output", Doc<"complement every bit of the scrambled stream; the 'v' the V-series family carries, and the descrambler must carry the same value">>                                                invert_output          = false;
    Annotated<gr::Size_t, "force_transition_after", Unit<"items">, Doc<"N: inject a complement at the Nth consecutive item over which the monitored comparison stayed zero, [2,4096]; 0 disables the rule and its cost">>    force_transition_after = 0U;
    Annotated<std::string, "monitor_delays", Doc<"the two transmitted-stream delays 'p,q' the monitor compares, distinct and each in [1,64]; required when force_transition_after is non-zero and refused when it is zero">> monitor_delays{};
    Annotated<std::string, "reset_tag_key", Doc<"tag key marking an epoch boundary, the reset applying before the tagged item; empty disables the tag-driven reset">>                                                        reset_tag_key = std::string(gr::tag::TRIGGER_NAME.shortKey());
    Annotated<std::string, "reset_tag_value", Doc<"if non-empty, the tag's value at that key must be a string equal to this, which narrows the boundary to one detector; empty matches any">>                                reset_tag_value{};
    Annotated<gr::Size_t, "reset_period", Unit<"items">, Doc<"items per epoch, counted from the last reset of any cause; 0 disables the periodic reset">>                                                                    reset_period = 0U;

    GR_MAKE_REFLECTABLE(MultiplicativeScrambler, in, out, profile, taps, seed, bits_per_item, bit_order, invert_output, force_transition_after, monitor_delays, reset_tag_key, reset_tag_value, reset_period);

    detail::ScramblerState _state{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        _state.resetKey   = property_map::key_type(reset_tag_key.value);
        _state.resetValue = reset_tag_value.value;
        rebuild();
    }

    void start() {
        rebuild();
        reset();
    }

    void reset() {
        gr::digital::resetScrambler(_state.config);
        _state.sinceReset = 0ULL;
    }

    void rebuild() { detail::configureScrambler(_state, detail::ScramblerSettings{profile.value, taps.value, seed.value, {}, true, bits_per_item.value, bit_order.value, reset_period.value, invert_output.value, force_transition_after.value, monitor_delays.value}, gr::digital::ScramblerMode::MultiplicativeScramble); }

    /// @brief Zero here: an explicit sequence is the additive family's mask source, and a recursion never runs out.
    /// Read alongside its sibling so one reader shape serves all three blocks.
    [[nodiscard]] std::uint64_t nUnscrambledItems() const noexcept { return static_cast<std::uint64_t>(_state.config.nUnscrambledItems); }

    /// @brief Times the forcing term fired, cumulative over the configuration's life. Not a failure: it is what says
    /// the rule is reachable in a given chain at all.
    [[nodiscard]] std::uint64_t nForcedTransitions() const noexcept { return static_cast<std::uint64_t>(_state.config.nForcedTransitions); }

    void stop() {
        if (_state.config.forceAfter != 0U) {
            std::println(stderr, "gr::blocks::digital::MultiplicativeScrambler '{}': the forcing term fired {} time(s)", this->name, _state.config.nForcedTransitions);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) { return detail::processScrambler(_state, inSpan, outSpan); }
};

GR_REGISTER_BLOCK(gr::blocks::digital::MultiplicativeDescrambler)

struct MultiplicativeDescrambler : Block<MultiplicativeDescrambler, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief Feeds the received bits forward through the shift register: `out[k] = in[k] XOR f(in, k)`.

The inverse of MultiplicativeScrambler, exact from item 0 when both carry the same seed. Its register holds nothing but
received bits, so once `degree` of them have arrived every bit in it is data and the output is correct from there on
whatever the register started as -- from a wrong seed, from a mid-stream join, from any offset at all. That is what
self-synchronizing means, and it is why a reset here is a convenience rather than a requirement: a reset changes at
most the first `degree` output bits of an epoch and nothing else. The contrast with the additive family is the whole
argument for this one, and it is stark: an additive descrambler one bit out of phase agrees with the truth at chance
for ever, with nothing inside the block able to tell.

What it costs is error multiplication. One channel bit error becomes `1 + tapCount` output errors -- three for a
two-tap polynomial -- so on a link with forward error correction this number is what decides whether the scrambler
goes inside the code or outside it.

`seed` is the `degree` bits assumed to precede the stream, oldest first, and an empty seed means all zeros and is
legal. `profile`, `invert_output`, `force_transition_after` and `monitor_delays` are MultiplicativeScrambler's and must
carry the same values it does: the monitor reads only the transmitted stream, so both ends compute the same counter
from the same bits and this block stays the exact inverse -- from item 0 with the same seed, and from item `degree`
with any other. `taps`, the three reset mechanisms and the tag policy are AdditiveScrambler's, unchanged.

The block is 1:1, so every input tag key passes through at its own offset; the reset tag is read and forwarded, never
consumed, and `sample_rate` leaves with the value it arrived with.
)"">;

    PortIn<std::uint8_t>  in;
    PortOut<std::uint8_t> out;

    Annotated<std::string, "profile", Doc<"a published generator by name: 'g3ruh', filling taps, seed, bits_per_item and bit_order; an additive profile is refused naming its family; empty selects none">, Visible> profile{};
    Annotated<std::string, "taps", Doc<"feedback delays, decimal and comma-separated as in \"18,23\"; each in [1,64], distinct, at least one, and there is no default">, Visible>                                    taps{};
    Annotated<std::string, "seed", Doc<"the `degree` bits assumed to precede the stream, '0'/'1' oldest first; empty means all zeros, and a wrong seed costs only the first `degree` bits">, Visible>                seed{};
    Annotated<gr::Size_t, "bits_per_item", Unit<"bit">, Doc<"significant bits per item, [1,8]; an item's remaining bits are ignored on input and zero on output">>                                                   bits_per_item          = 8U;
    Annotated<gr::digital::BitOrder, "bit_order", Doc<"how each item's field is traversed, which is not a byte order">>                                                                                              bit_order              = gr::digital::BitOrder::MsbFirst;
    Annotated<bool, "invert_output", Doc<"the scrambler's 'v', undone here; the two ends must carry the same value">>                                                                                                invert_output          = false;
    Annotated<gr::Size_t, "force_transition_after", Unit<"items">, Doc<"N, the scrambler's; the monitor runs on the received stream, which is the scrambler's transmitted one, so the two counters agree">>          force_transition_after = 0U;
    Annotated<std::string, "monitor_delays", Doc<"the two delays 'p,q' the monitor compares, the scrambler's; required when force_transition_after is non-zero and refused when it is zero">>                        monitor_delays{};
    Annotated<std::string, "reset_tag_key", Doc<"tag key marking an epoch boundary, the reset applying before the tagged item; empty disables the tag-driven reset">>                                                reset_tag_key = std::string(gr::tag::TRIGGER_NAME.shortKey());
    Annotated<std::string, "reset_tag_value", Doc<"if non-empty, the tag's value at that key must be a string equal to this, which narrows the boundary to one detector; empty matches any">>                        reset_tag_value{};
    Annotated<gr::Size_t, "reset_period", Unit<"items">, Doc<"items per epoch, counted from the last reset of any cause; 0 disables the periodic reset">>                                                            reset_period = 0U;

    GR_MAKE_REFLECTABLE(MultiplicativeDescrambler, in, out, profile, taps, seed, bits_per_item, bit_order, invert_output, force_transition_after, monitor_delays, reset_tag_key, reset_tag_value, reset_period);

    detail::ScramblerState _state{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        _state.resetKey   = property_map::key_type(reset_tag_key.value);
        _state.resetValue = reset_tag_value.value;
        rebuild();
    }

    void start() {
        rebuild();
        reset();
    }

    void reset() {
        gr::digital::resetScrambler(_state.config);
        _state.sinceReset = 0ULL;
    }

    void rebuild() { detail::configureScrambler(_state, detail::ScramblerSettings{profile.value, taps.value, seed.value, {}, true, bits_per_item.value, bit_order.value, reset_period.value, invert_output.value, force_transition_after.value, monitor_delays.value}, gr::digital::ScramblerMode::MultiplicativeDescramble); }

    /// @brief Zero here: an explicit sequence is the additive family's mask source, and a recursion never runs out.
    /// Read alongside its sibling so one reader shape serves all three blocks.
    [[nodiscard]] std::uint64_t nUnscrambledItems() const noexcept { return static_cast<std::uint64_t>(_state.config.nUnscrambledItems); }

    /// @brief Times the forcing term fired, cumulative over the configuration's life. It must equal the scrambler's
    /// over the same stream, which is what says the two ends computed the same counter.
    [[nodiscard]] std::uint64_t nForcedTransitions() const noexcept { return static_cast<std::uint64_t>(_state.config.nForcedTransitions); }

    void stop() {
        if (_state.config.forceAfter != 0U) {
            std::println(stderr, "gr::blocks::digital::MultiplicativeDescrambler '{}': the forcing term fired {} time(s)", this->name, _state.config.nForcedTransitions);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) { return detail::processScrambler(_state, inSpan, outSpan); }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_SCRAMBLER_HPP
