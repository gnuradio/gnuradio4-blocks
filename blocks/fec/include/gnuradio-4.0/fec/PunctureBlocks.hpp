#ifndef GNURADIO_FEC_PUNCTURE_BLOCKS_HPP
#define GNURADIO_FEC_PUNCTURE_BLOCKS_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/fec/RecordShape.hpp>

/**
 * Rate matching by puncturing: a stated mask deletes coded bits on the way out, and the receiving
 * end puts erasures back where they stood.
 *
 * The mask is a string of '1' and '0' characters — keep and delete — applied cyclically over the
 * coded stream in the encoder's own output order, one bit per polynomial with the polynomials in
 * the order they were given. The string form is chosen so that a published rate-compatible pattern
 * transcribes without reordering: such patterns are quoted as matrices read column by column, and a
 * column-major read of an n-row matrix in polynomial order is exactly this string. The classic
 * rate-2/3 and rate-3/4 punctures of the (171, 133) code are spelled "1110" and "111001" here.
 *
 * The mask's length is not tied to the number of polynomials. A pattern is a property of the coded
 * stream rather than of one encoder step, and the useful published patterns walk across step
 * boundaries: rate 3/4 from rate 1/2 has a period of six over steps of two.
 *
 * The phase resets at every record. A record is one frame, so a frame's puncturing is reproducible
 * in isolation — what a stored or forwarded frame needs — and the two blocks agree on phase by
 * construction rather than through a counter they would have to keep in step.
 *
 * Puncture carries bit items and Depuncture soft ones, on the convolutional adapters' conventions:
 * one std::uint8_t per bit with the low bit holding the value, and one float per coded bit with the
 * sign carrying the bit and the magnitude the confidence. A deleted position comes back as 0.0F,
 * the convention's pure erasure, which is precisely the value the Viterbi correlation metric weighs
 * at nothing. There is no hard-decision counterpart, because a hard word has nowhere to say that a
 * bit is missing rather than zero.
 *
 * Neither block writes metadata. Puncturing is an encoder-side act with no status, and an inserted
 * erasure is the pattern's own fact rather than a report about the channel; the decoder's
 * `corrected_errors` already accounts for what the channel and the puncturing together cost.
 *
 * The pattern is immutable configuration rather than a live setting, for the reason the code is:
 * both ends of a link agree on it before the first bit.
 */
namespace gr::blocks::fec {

namespace detail {

//! A validated puncturing mask: the characters themselves and the kept positions in one period.
struct PuncturePattern {
    std::string mask{};     //!< the '1' and '0' characters, walked cyclically from each record's start
    std::size_t kept = 0UZ; //!< kept positions in one period of `mask`

    [[nodiscard]] std::size_t period() const noexcept { return mask.size(); }

    //! Whether the cyclic walk keeps the coded bit at @p position.
    [[nodiscard]] bool keeps(std::size_t position) const noexcept { return mask[position % mask.size()] == '1'; }

    //! The kept positions among the first @p length positions of the walk.
    [[nodiscard]] std::size_t keptIn(std::size_t length) const noexcept {
        std::size_t count = (length / mask.size()) * kept;
        for (std::size_t i = 0UZ, tail = length % mask.size(); i < tail; ++i) {
            count += (mask[i] == '1') ? 1UZ : 0UZ;
        }
        return count;
    }
};

//! The pattern @p spelling names, reporting its refusals through the graph's exception type.
[[nodiscard]] inline PuncturePattern puncturePattern(const std::string& spelling) {
    if (spelling.empty()) {
        throw gr::exception("pattern has no default: a puncture is the mask it applies, and neither end of a link can guess the other's");
    }
    std::size_t kept = 0UZ;
    for (const char position : spelling) {
        if (position != '0' && position != '1') {
            throw gr::exception(std::format("pattern spells one keep or delete per coded bit as '1' or '0', got '{}'", spelling));
        }
        kept += (position == '1') ? 1UZ : 0UZ;
    }
    if (kept == 0UZ) {
        throw gr::exception(std::format("pattern deletes every position and would carry nothing, got '{}'", spelling));
    }
    return PuncturePattern{spelling, kept};
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::fec::Puncture)

struct Puncture : Block<Puncture> {
    using Description = Doc<R""(
@brief Puncture: a record of coded bits in, the bits `pattern` keeps out.

The record's bits are walked with the pattern from phase zero, positions marked '1' being kept in order and positions
marked '0' dropped. The pattern needs no alignment with the record's end, so any nonzero length punctures; only an
empty record is refused, counted in `nRecordsRefused` and stated at `stop()`, and the record after it is punctured
normally.

An encoder-side rate match has no status to report, so the record's metadata crosses unchanged; its signal name and
its single-map shape follow it, and the output record's extent names its own length.

A record whose length is not a whole number of pattern periods punctures here and is refused by Depuncture, the
receiving counterpart.
)"">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<std::string, "pattern", Doc<"'1' keeps and '0' deletes, walked cyclically over the coded bits in the encoder's output order; required. \"1110\" is the classic rate 2/3 puncture of a rate 1/2 code">, Visible> pattern{};

    GR_MAKE_REFLECTABLE(Puncture, in, out, pattern);

    detail::PuncturePattern _pattern{};
    bool                    _configured = false;

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords        = 0ULL; ///< records published on `out`
    std::uint64_t nBits           = 0ULL; ///< coded bits those records kept
    std::uint64_t nRecordsRefused = 0ULL; ///< records this pattern could not puncture

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        _pattern    = detail::puncturePattern(pattern.value);
        _configured = true; // only reached when the setting named a pattern the checks above accept
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 3UZ> counters{{{"records", nRecords}, {"bits kept", nBits}, {"records refused", nRecordsRefused}}};
        detail::report("gr::blocks::fec::Puncture", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) { // inert rather than deleting bits under a pattern nobody chose
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record = inSpan[consumed];
            const std::size_t            bits   = record.signal_values.size();
            if (bits == 0UZ) {
                ++nRecordsRefused;
                continue;
            }

            DataSet<std::uint8_t> punctured;
            punctured.signal_values.reserve(_pattern.keptIn(bits));
            for (std::size_t i = 0UZ; i < bits; ++i) {
                if (_pattern.keeps(i)) {
                    punctured.signal_values.push_back(record.signal_values[i]);
                }
            }
            nBits += punctured.signal_values.size();
            detail::carry(punctured, record);

            ++nRecords;
            outSpan[made] = std::move(punctured);
            ++made;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::fec::Depuncture)

struct Depuncture : Block<Depuncture> {
    using Description = Doc<R""(
@brief Depuncture: a record of the soft values that survived the puncture in, the coded frame with erasures at the
deleted positions out.

The pattern is walked from phase zero over the output. Every position marked '1' takes the next input value and every
position marked '0' becomes 0.0F, the soft convention's pure erasure, which the Viterbi correlation metric weighs at
nothing.

The output's length is the input's kept count expanded back over whole periods of the pattern: an input of `m` values
under a pattern keeping `w` of every `p` positions becomes `(m / w) * p` values. An input whose length is not a whole
multiple of `w` has no length to expand to and is a counted, stated drop, as is an empty record; the record after it
is depunctured normally. The pair is an exact inverse on records whose length is a whole number of pattern periods.
Outside that domain a kept count cannot name the length it came from, because the deleted positions at a record's end
leave no trace in what was sent.

Nothing is written to metadata; every key crosses verbatim. An inserted erasure is the pattern's own fact, and a
decoder's `corrected_errors` reports what the channel and the puncturing together cost.
)"">;

    PortIn<DataSet<float>, Async>  in;
    PortOut<DataSet<float>, Async> out;

    Annotated<std::string, "pattern", Doc<"the same mask the sending Puncture was given; required. A position marked '0' returns as 0.0F, the soft convention's pure erasure">, Visible> pattern{};

    GR_MAKE_REFLECTABLE(Depuncture, in, out, pattern);

    detail::PuncturePattern _pattern{};
    bool                    _configured = false;

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords        = 0ULL; ///< records published on `out`
    std::uint64_t nBits           = 0ULL; ///< coded positions those records carry, erasures included
    std::uint64_t nRecordsRefused = 0ULL; ///< records whose length was inconsistent with this pattern

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        _pattern    = detail::puncturePattern(pattern.value);
        _configured = true; // only reached when the setting named a pattern the checks above accept
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 3UZ> counters{{{"records", nRecords}, {"items emitted", nBits}, {"records refused", nRecordsRefused}}};
        detail::report("gr::blocks::fec::Depuncture", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) { // inert rather than inserting erasures under a pattern nobody chose
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<float>& record = inSpan[consumed];
            const std::size_t     items  = record.signal_values.size();
            if (items == 0UZ || items % _pattern.kept != 0UZ) {
                ++nRecordsRefused;
                continue;
            }

            DataSet<float>    soft;
            const std::size_t length = (items / _pattern.kept) * _pattern.period();
            soft.signal_values.resize(length);
            std::size_t taken = 0UZ;
            for (std::size_t i = 0UZ; i < length; ++i) {
                soft.signal_values[i] = _pattern.keeps(i) ? record.signal_values[taken++] : 0.0F;
            }
            detail::carry(soft, record);

            ++nRecords;
            nBits += length;
            outSpan[made] = std::move(soft);
            ++made;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::fec

#endif // GNURADIO_FEC_PUNCTURE_BLOCKS_HPP
