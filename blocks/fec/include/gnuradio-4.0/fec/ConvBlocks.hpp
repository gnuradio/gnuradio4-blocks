#ifndef GNURADIO_FEC_CONV_BLOCKS_HPP
#define GNURADIO_FEC_CONV_BLOCKS_HPP

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

#include <gnuradio-4.0/algorithm/fec/Convolutional.hpp>

#include <gnuradio-4.0/fec/RecordShape.hpp>

/**
 * The record-native adapters over the convolutional code and the Viterbi decoder in
 * `gnuradio-4.0/algorithm/fec`.
 *
 * The hard-decision carrier is the same record of bits the block-code adapters take: a
 * `DataSet<std::uint8_t>` with one bit per item, the low bit holding the value and the bits above
 * it ignored on read and emitted zero. Item `i` of a coded record is coded bit `i`, and the coded
 * bits of one input step follow one another in the order the polynomials are given, so a puncturer
 * or an interleaver composes downstream without either end restating the layout.
 *
 * The soft-decision carrier is a `DataSet<float>` of one value per coded bit, the sign carrying
 * the bit and the magnitude the confidence, with zero a pure erasure. The branch metric is a
 * correlation, so a demodulator hands over whatever amplitude it has: scaling every value scales
 * the metric and moves no decision.
 *
 * One record is one frame. The encoder appends the K-1 zero bits that return the register to the
 * zero state, so `k` information bits become `(k + K - 1) * n` coded bits, and the decoder holds
 * the whole trellis and tracks back from the state termination guarantees. That is what makes the
 * decode exact maximum likelihood rather than the survivor of a truncated window, and it is why
 * the record boundary, not a truncation depth, is the unit here. The decoders also take
 * `termination = "open"` for a record cut out of a continuous convolutional stream — frames that
 * run into each other, as a CCSDS channel's do: every step carries an information bit, neither
 * end state is known, the trellis converges within about 5K steps of the record's start and the
 * last K-1 information bits lack the future that would resolve them, so a caller extracts that
 * margin beyond its payload and discards it. The encoder has no open mode: an encoder that does
 * not terminate is a stream, not a record.
 *
 * A Viterbi decode has no refusal — the trellis has a best path through any word — so the
 * decoders write `corrected_errors` and no `uncorrectable_errors`. A key that could only ever be
 * zero would say something about the decode that is not true.
 *
 * The code is immutable configuration rather than a live setting. Both ends of a link agree on it
 * before the first bit, so changing it makes a different chain, which is what rebuilding a graph
 * is for.
 */
namespace gr::blocks::fec {

namespace detail {

//! The code the settings name — a convention by `code`, or the explicit `constraint_length` and
//! `polynomials` pair — reporting the kernel's own refusals through the graph's exception type.
//! An explicit setting staged beside a named convention must agree with it: two answers to one
//! question is a refusal naming both, never a silent preference.
[[nodiscard]] inline gr::fec::ConvolutionalCode convolutionalCode(gr::Size_t constraintLength, const std::vector<gr::Size_t>& polynomials, const std::string& codeName, gr::Size_t invertOutputs) {
    if (!codeName.empty()) {
        const gr::fec::ConvolutionalConvention* convention = gr::fec::conventionByName(codeName);
        if (convention == nullptr) {
            std::string names;
            for (const gr::fec::ConvolutionalConvention& entry : gr::fec::kConvConventions) {
                std::format_to(std::back_inserter(names), "{}'{}'", names.empty() ? "" : ", ", entry.name);
            }
            throw gr::exception(std::format("code must name one of {}, got '{}'", names, codeName));
        }
        if (constraintLength != 0U && static_cast<std::size_t>(constraintLength) != convention->constraintLength) {
            throw gr::exception(std::format("code '{}' supplies constraint_length {} and the explicit setting says {}: one question, two answers", codeName, convention->constraintLength, constraintLength));
        }
        if (!polynomials.empty()) {
            bool agrees = polynomials.size() == convention->polynomialCount;
            for (std::size_t i = 0UZ; agrees && i < polynomials.size(); ++i) {
                agrees = polynomials[i] == convention->polynomials[i];
            }
            if (!agrees) {
                throw gr::exception(std::format("code '{}' supplies its own polynomials and the explicit setting disagrees: one question, two answers", codeName));
            }
        }
        if (invertOutputs != 0U && invertOutputs != convention->outputInversion) {
            throw gr::exception(std::format("code '{}' supplies invert_outputs {:#b} and the explicit setting says {:#b}: one question, two answers", codeName, convention->outputInversion, invertOutputs));
        }
        gr::fec::ConvolutionalCode named;
        std::ignore = gr::fec::configureConvention(named, codeName); // the name resolved, so this cannot refuse
        return named;
    }

    if (constraintLength == 0U && polynomials.empty()) {
        throw gr::exception("constraint_length and polynomials have no defaults: a convolutional code is the pair, and neither end of a link can guess the other's; code names a published convention instead");
    }
    if (polynomials.size() < gr::fec::kConvMinPolynomials || polynomials.size() > gr::fec::kConvMaxPolynomials) {
        throw gr::exception(std::format("polynomials names one generator per coded bit and must hold {} to {} of them, got {}", gr::fec::kConvMinPolynomials, gr::fec::kConvMaxPolynomials, polynomials.size()));
    }

    std::array<std::uint32_t, gr::fec::kConvMaxPolynomials> generators{};
    for (std::size_t i = 0UZ; i < polynomials.size(); ++i) {
        generators[i] = polynomials[i];
    }

    gr::fec::ConvolutionalCode code;
    if (!code.configure(static_cast<std::size_t>(constraintLength), std::span<const std::uint32_t>(generators).first(polynomials.size()), invertOutputs)) {
        throw gr::exception(std::format("constraint_length must be {} to {}, every polynomial must be nonzero, no wider than constraint_length bits and named once, and invert_outputs may set only bits below the polynomial count; got constraint_length {}, invert_outputs {:#b}", //
            gr::fec::kConvMinConstraintLength, gr::fec::kConvMaxConstraintLength, constraintLength, invertOutputs));
    }
    return code;
}

//! The termination mode the `termination` setting names, or the refusal naming its two values.
[[nodiscard]] inline gr::fec::ConvTermination terminationByName(const std::string& name) {
    if (name == "terminated") {
        return gr::fec::ConvTermination::Terminated;
    }
    if (name == "open") {
        return gr::fec::ConvTermination::Open;
    }
    throw gr::exception(std::format("termination must be 'terminated' or 'open', got '{}'", name));
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::fec::ConvEncode)

struct ConvEncode : Block<ConvEncode> {
    using Description = Doc<R""(
@brief Convolutional encode: one information-bit record in, one terminated coded frame out, under the code
`constraint_length` and `polynomials` name.

A record is a frame. Its `k` information bits become `(k + K - 1) * n` coded bits, the tail being the K-1 zero bits
that leave the register in the zero state, so the decoder's traceback is exact. Any nonzero length encodes; an empty
record is refused, counted in `nRecordsRefused` and stated at `stop()`, and the record after it is encoded normally.
Every frame is terminated, so this block has no `termination` setting.

An encoder has no status to report, so the record's metadata crosses unchanged; its signal name and its single-map
shape follow it, and the output record's extent names its own length. ViterbiDecode and ViterbiDecodeSoft are the
decoding counterparts.
)"">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "constraint_length", Doc<"K, the register's width in input bits: 3 to 9; required, with no default">, Visible>                                                                                                                  constraint_length{};
    Annotated<std::vector<gr::Size_t>, "polynomials", Doc<"one generator per coded bit, 2 to 4 of them, each the octal spelling's value, 0171 and 0133 for the classic K = 7 pair; bit 0 taps the current input bit">, Visible>                           polynomials{};
    Annotated<std::string, "code", Doc<"a published convention supplying constraint_length, polynomials and invert_outputs: 'ccsds', 'ccsds_uninverted', 'nasa_dsn' or 'nasa_dsn_uninverted'; an explicit setting staged beside it must agree">, Visible> code{};
    Annotated<gr::Size_t, "invert_outputs", Doc<"bit j set: coded output j is emitted complemented; bits at or past the polynomial count are refused">>                                                                                                   invert_outputs = 0U;

    GR_MAKE_REFLECTABLE(ConvEncode, in, out, constraint_length, polynomials, code, invert_outputs);

    gr::fec::ConvolutionalCode _code{};
    bool                       _configured = false;

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords        = 0ULL; ///< frames published on `out`
    std::uint64_t nInfoBits       = 0ULL; ///< information bits those frames carry
    std::uint64_t nRecordsRefused = 0ULL; ///< records that were not a frame this code can encode

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        _code       = detail::convolutionalCode(constraint_length.value, polynomials.value, code.value, invert_outputs.value);
        _configured = true; // only reached when the settings named a code the kernel accepts
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 3UZ> counters{{{"frames", nRecords}, {"information bits", nInfoBits}, {"records refused", nRecordsRefused}}};
        detail::report("gr::blocks::fec::ConvEncode", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) { // inert rather than coding under a code nobody chose
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

            DataSet<std::uint8_t> coded;
            coded.signal_values.resize(gr::fec::convolutionalEncodedBits(_code, bits));
            std::ignore = gr::fec::convolutionalEncode(_code, record.signal_values, coded.signal_values);
            detail::carry(coded, record);

            ++nRecords;
            nInfoBits += bits;
            outSpan[made] = std::move(coded);
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

GR_REGISTER_BLOCK(gr::blocks::fec::ViterbiDecode)

struct ViterbiDecode : Block<ViterbiDecode> {
    using Description = Doc<R""(
@brief Viterbi decode of hard decisions: one terminated coded frame in, its information bits out, with the channel's
account in metadata.

A record's length must be a nonzero multiple of `n` and hold at least `K * n` bits, a frame shorter than the tail
carrying no information; the record then yields `len / n - (K - 1)` information bits. A length failing that test is a
counted, stated drop, as an empty record is at ConvEncode.

The account is carried in metadata: `corrected_errors` gains the decode's distance (the bits by which the received frame and
the winning path disagree) added to whatever the key already carried, so a chain of correcting stages reports one
total rather than its last stage's share. Every other key crosses verbatim, and a record arriving without a metadata
map gains one to carry the key.

There is no `uncorrectable_errors`: the trellis has a best path through any received word, so the key would always be
zero. The distance in `corrected_errors` is how far the received frame stood from the path chosen.
)"">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "constraint_length", Doc<"K, the register's width in input bits: 3 to 9; required, with no default">, Visible>                                                                                                                  constraint_length{};
    Annotated<std::vector<gr::Size_t>, "polynomials", Doc<"one generator per coded bit, 2 to 4 of them, each the octal spelling's value, 0171 and 0133 for the classic K = 7 pair; bit 0 taps the current input bit">, Visible>                           polynomials{};
    Annotated<std::string, "code", Doc<"a published convention supplying constraint_length, polynomials and invert_outputs: 'ccsds', 'ccsds_uninverted', 'nasa_dsn' or 'nasa_dsn_uninverted'; an explicit setting staged beside it must agree">, Visible> code{};
    Annotated<gr::Size_t, "invert_outputs", Doc<"bit j set: coded output j arrived complemented and is compared against the complement; bits at or past the polynomial count are refused">>                                                               invert_outputs = 0U;
    Annotated<std::string, "termination", Doc<"'terminated': the frame ends in the zero state and carries K-1 tail bits. 'open': a record cut out of a continuous stream, every step an information bit and both end states free">>                       termination    = std::string("terminated");

    GR_MAKE_REFLECTABLE(ViterbiDecode, in, out, constraint_length, polynomials, code, invert_outputs, termination);

    gr::fec::ConvolutionalCode _code{};
    gr::fec::ViterbiDecoder    _decoder{};
    gr::fec::ConvTermination   _termination = gr::fec::ConvTermination::Terminated;
    bool                       _configured  = false;

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords         = 0ULL; ///< frames published on `out`
    std::uint64_t nInfoBits        = 0ULL; ///< information bits those frames carry
    std::uint64_t nRecordsRefused  = 0ULL; ///< records whose length was not a frame of this code
    std::uint64_t nCorrectedErrors = 0ULL; ///< bits between the received frames and the paths decoded, totaled

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        _code        = detail::convolutionalCode(constraint_length.value, polynomials.value, code.value, invert_outputs.value);
        _termination = detail::terminationByName(termination.value);
        std::ignore  = _decoder.configure(_code, _termination);
        _configured  = true; // only reached when the settings named a code the kernel accepts
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 4UZ> counters{{{"frames", nRecords}, {"information bits", nInfoBits}, {"records refused", nRecordsRefused}, {"corrected errors", nCorrectedErrors}}};
        detail::report("gr::blocks::fec::ViterbiDecode", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) { // inert rather than decoding under a code nobody chose
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record  = inSpan[consumed];
            const std::size_t            carried = gr::fec::convolutionalInfoBits(_code, record.signal_values.size(), _termination);
            if (carried == 0UZ) {
                ++nRecordsRefused;
                continue;
            }

            DataSet<std::uint8_t> info;
            info.signal_values.resize(carried);
            const gr::fec::ViterbiResult result = _decoder.decodeHard(record.signal_values, info.signal_values);
            detail::carry(info, record);

            const gr::Size_t corrected = static_cast<gr::Size_t>(result.distance);
            property_map&    map       = info.meta_information[0UZ];
            map["corrected_errors"]    = gr::Size_t{detail::metaOr<gr::Size_t>(map, "corrected_errors", 0U) + corrected};

            ++nRecords;
            nInfoBits += carried;
            nCorrectedErrors += result.distance;
            outSpan[made] = std::move(info);
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

GR_REGISTER_BLOCK(gr::blocks::fec::ViterbiDecodeSoft)

struct ViterbiDecodeSoft : Block<ViterbiDecodeSoft> {
    using Description = Doc<R""(
@brief Viterbi decode of soft decisions: one record of soft coded values in, its information bits out, with the
channel's account in metadata.

Identical to ViterbiDecode but for the carrier: one `float` per coded bit, the sign carrying the bit (positive is a
one) and the magnitude the confidence, with zero a pure erasure. The branch metric is the correlation between a
branch's output and the received values, so any consistent scaling of the input scales the metric and changes no
decision; a demodulator needs neither a normalization step nor a quantization table before this block.

`corrected_errors` gains the distance between the sign-sliced input and the winning path, the same account of the
channel the hard decoder gives, and there is no `uncorrectable_errors` here either. The length contract, the counted
drops and the metadata carry are ViterbiDecode's.
)"">;

    PortIn<DataSet<float>, Async>         in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "constraint_length", Doc<"K, the register's width in input bits: 3 to 9; required, with no default">, Visible>                                                                                                                  constraint_length{};
    Annotated<std::vector<gr::Size_t>, "polynomials", Doc<"one generator per coded bit, 2 to 4 of them, each the octal spelling's value, 0171 and 0133 for the classic K = 7 pair; bit 0 taps the current input bit">, Visible>                           polynomials{};
    Annotated<std::string, "code", Doc<"a published convention supplying constraint_length, polynomials and invert_outputs: 'ccsds', 'ccsds_uninverted', 'nasa_dsn' or 'nasa_dsn_uninverted'; an explicit setting staged beside it must agree">, Visible> code{};
    Annotated<gr::Size_t, "invert_outputs", Doc<"bit j set: coded output j arrived complemented and enters the correlation sign-flipped; bits at or past the polynomial count are refused">>                                                              invert_outputs = 0U;
    Annotated<std::string, "termination", Doc<"'terminated': the frame ends in the zero state and carries K-1 tail bits. 'open': a record cut out of a continuous stream, every step an information bit and both end states free">>                       termination    = std::string("terminated");

    GR_MAKE_REFLECTABLE(ViterbiDecodeSoft, in, out, constraint_length, polynomials, code, invert_outputs, termination);

    gr::fec::ConvolutionalCode _code{};
    gr::fec::ViterbiDecoder    _decoder{};
    gr::fec::ConvTermination   _termination = gr::fec::ConvTermination::Terminated;
    bool                       _configured  = false;

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords         = 0ULL; ///< frames published on `out`
    std::uint64_t nInfoBits        = 0ULL; ///< information bits those frames carry
    std::uint64_t nRecordsRefused  = 0ULL; ///< records whose length was not a frame of this code
    std::uint64_t nCorrectedErrors = 0ULL; ///< bits between the sign-sliced frames and the paths decoded, totaled

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        _code        = detail::convolutionalCode(constraint_length.value, polynomials.value, code.value, invert_outputs.value);
        _termination = detail::terminationByName(termination.value);
        std::ignore  = _decoder.configure(_code, _termination);
        _configured  = true; // only reached when the settings named a code the kernel accepts
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 4UZ> counters{{{"frames", nRecords}, {"information bits", nInfoBits}, {"records refused", nRecordsRefused}, {"corrected errors", nCorrectedErrors}}};
        detail::report("gr::blocks::fec::ViterbiDecodeSoft", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) { // inert rather than decoding under a code nobody chose
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<float>& record  = inSpan[consumed];
            const std::size_t     carried = gr::fec::convolutionalInfoBits(_code, record.signal_values.size(), _termination);
            if (carried == 0UZ) {
                ++nRecordsRefused;
                continue;
            }

            DataSet<std::uint8_t> info;
            info.signal_values.resize(carried);
            const gr::fec::ViterbiResult result = _decoder.decodeSoft(record.signal_values, info.signal_values);
            detail::carry(info, record);

            const gr::Size_t corrected = static_cast<gr::Size_t>(result.distance);
            property_map&    map       = info.meta_information[0UZ];
            map["corrected_errors"]    = gr::Size_t{detail::metaOr<gr::Size_t>(map, "corrected_errors", 0U) + corrected};

            ++nRecords;
            nInfoBits += carried;
            nCorrectedErrors += result.distance;
            outSpan[made] = std::move(info);
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

#endif // GNURADIO_FEC_CONV_BLOCKS_HPP
