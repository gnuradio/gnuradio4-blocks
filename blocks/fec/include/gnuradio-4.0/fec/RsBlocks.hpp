#ifndef GNURADIO_FEC_RS_BLOCKS_HPP
#define GNURADIO_FEC_RS_BLOCKS_HPP

#include <algorithm>
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

#include <gnuradio-4.0/algorithm/fec/ReedSolomon.hpp>

#include <gnuradio-4.0/fec/RecordShape.hpp>

/**
 * The record-native adapters over the Reed-Solomon kernel in `gnuradio-4.0/algorithm/fec`.
 *
 * The carrier is a record of symbols: a `DataSet<std::uint8_t>` with one symbol per item, each
 * item holding a field value in its low bits — six of them for GF(64), the whole item for the
 * CCSDS profiles' GF(256). A symbol is the code's natural unit, and packing symbols into a bit
 * stream is composition work for the bit-packing blocks rather than something to fold in here.
 *
 * Two families share the pair. The validated GF(64) instantiations are selected by `roots` at
 * 8, 12 or 16 parity symbols over the 63-symbol block. The `code` setting names a CCSDS
 * profile instead — `ccsds_255_223` or `ccsds_255_239`, 131.0-B-5 section 4's codes over
 * GF(256) with their first consecutive root and primitive step — and with it come the two
 * conventions the standard attaches: `basis` says whether the wire's symbols are dual-basis
 * strings (4.3.9's recoding, one table lookup per symbol on the way in and on the information
 * on the way out), and `interleave` is the codeword count `I` of one codeblock, the symbols of
 * the `I` codewords alternating on the wire so a channel burst lands spread across them. In
 * every family both ends know that the leading `pad` symbols of each codeword are zero and
 * neither sends them. The adapters stage each codeword into the kernel's block with the
 * padding restored, call it with `pad`, and copy the wire region back out; the kernel owns the
 * arithmetic and the checks.
 */
namespace gr::blocks::fec {

namespace detail {

//! The longest unshortened block any family here works on, which sizes the staging buffers.
inline constexpr std::size_t kRsMaxBlockSymbols = gr::fec::ReedSolomonCcsds255_223::kBlock;

//! Stage @p word into the kernel's own block type, run it, and copy the result back out. The
//! copies keep one type-erased surface over the GF(64) and GF(256) families.
template<typename TKernel>
inline void rsEncodeThrough(std::span<std::uint8_t> word, std::size_t pad) {
    typename TKernel::Block block{};
    std::copy_n(word.begin(), TKernel::kBlock, block.begin());
    TKernel::encode(block, pad);
    std::copy_n(block.begin(), TKernel::kBlock, word.begin());
}

template<typename TKernel>
[[nodiscard]] inline gr::fec::RsResult rsDecodeThrough(std::span<std::uint8_t> word, std::size_t pad) {
    typename TKernel::Block block{};
    std::copy_n(word.begin(), TKernel::kBlock, block.begin());
    const gr::fec::RsResult result = TKernel::decode(block, pad);
    std::copy_n(block.begin(), TKernel::kBlock, word.begin());
    return result;
}

//! One Reed-Solomon code: its geometry and the kernel entry points the settings select.
struct RsCode {
    std::size_t  roots = 0UZ;   //!< parity symbols per codeword
    std::size_t  block = 0UZ;   //!< symbols of the unshortened block, 63 or 255
    std::uint8_t mask  = 0xFFU; //!< the bits of an item a symbol occupies; the ones above are ignored on read, emitted zero
    bool         ccsds = false; //!< a CCSDS profile, whose basis and interleave rules apply

    void (*encode)(std::span<std::uint8_t>, std::size_t)              = nullptr;
    gr::fec::RsResult (*decode)(std::span<std::uint8_t>, std::size_t) = nullptr;
};

template<typename TKernel>
[[nodiscard]] inline RsCode rsCodeOver(std::uint8_t mask, bool ccsds) {
    return {TKernel::kRoots, TKernel::kBlock, mask, ccsds, &rsEncodeThrough<TKernel>, &rsDecodeThrough<TKernel>};
}

//! The code the `code` and `roots` settings name: a CCSDS profile by name, or the validated
//! GF(64) family by parity count. A `roots` staged beside a profile must equal the profile's
//! own parity count — one question, one answer.
[[nodiscard]] inline RsCode reedSolomonCode(const std::string& codeName, gr::Size_t roots) {
    if (codeName == "ccsds_255_223" || codeName == "ccsds_255_239") {
        const std::size_t profileRoots = codeName == "ccsds_255_223" ? 32UZ : 16UZ;
        if (roots != 0U && static_cast<std::size_t>(roots) != profileRoots) {
            throw gr::exception(std::format("code '{}' carries {} parity symbols and roots says {}: one question, two answers", codeName, profileRoots, roots));
        }
        return profileRoots == 32UZ ? rsCodeOver<gr::fec::ReedSolomonCcsds255_223>(0xFFU, true) : rsCodeOver<gr::fec::ReedSolomonCcsds255_239>(0xFFU, true);
    }
    if (!codeName.empty()) {
        throw gr::exception(std::format("code must be empty, 'ccsds_255_223' or 'ccsds_255_239', got '{}'; there is no free-form code tuple", codeName));
    }

    switch (roots) {
    case 8U: return rsCodeOver<gr::fec::ReedSolomon6<8UZ>>(0x3FU, false);
    case 12U: return rsCodeOver<gr::fec::ReedSolomon6<12UZ>>(0x3FU, false);
    case 16U: return rsCodeOver<gr::fec::ReedSolomon6<16UZ>>(0x3FU, false);
    default: break;
    }
    if (roots == 0U) {
        throw gr::exception("roots has no default and must be set to 8, 12 or 16: the parity count is half of what names a code, and the two ends of a link cannot guess it; code names a CCSDS profile instead");
    }
    throw gr::exception(std::format("roots must be 8, 12 or 16, the validated instantiations of the GF(64) code, got {}", roots));
}

//! Whether the `basis` setting selects the dual-basis recoding. A CCSDS profile requires the
//! choice — 131.0-B-5 4.3.9.1 mandates `dual` while a substantial flown population runs
//! `conventional`, a wrong choice decodes nothing, and neither end can guess — and the GF(64)
//! family has no dual-basis convention for the setting to select.
[[nodiscard]] inline bool reedSolomonDual(const RsCode& code, const std::string& basis) {
    if (!code.ccsds) {
        if (!basis.empty()) {
            throw gr::exception(std::format("basis selects a CCSDS profile's symbol convention, and the GF(64) family has none; got '{}'", basis));
        }
        return false;
    }
    if (basis == "conventional") {
        return false;
    }
    if (basis == "dual") {
        return true;
    }
    if (basis.empty()) {
        throw gr::exception("basis has no default under a CCSDS profile: 4.3.9.1 mandates 'dual' while part of the flown population uses 'conventional', a wrong choice decodes nothing, and neither end can guess");
    }
    throw gr::exception(std::format("basis must be 'conventional' or 'dual', got '{}'", basis));
}

//! The interleave depth, validated against the family: under a CCSDS profile the depths 4.3.5.1
//! allows, which are 1, 2, 3, 4, 5 and 8; otherwise 1 to 8.
[[nodiscard]] inline std::size_t reedSolomonDepth(const RsCode& code, gr::Size_t interleave) {
    const std::size_t depth = static_cast<std::size_t>(interleave);
    if (code.ccsds && !gr::fec::ccsdsInterleaveDepthAllowed(depth)) {
        throw gr::exception(std::format("interleave must be one of 4.3.5.1's depths 1, 2, 3, 4, 5 or 8, got {}", interleave));
    }
    if (!code.ccsds && (depth < 1UZ || depth > 8UZ)) {
        throw gr::exception(std::format("interleave must be 1 to 8, got {}", interleave));
    }
    return depth;
}

//! The shortening @p pad describes, checked against the room the code leaves for information.
[[nodiscard]] inline std::size_t reedSolomonPad(gr::Size_t pad, const RsCode& code) {
    if (static_cast<std::size_t>(pad) + code.roots >= code.block) {
        throw gr::exception(std::format("pad is the shortening in leading zero symbols and must be below {}, the information the code carries unshortened, got {}", code.block - code.roots, pad));
    }
    return static_cast<std::size_t>(pad);
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::fec::RsEncode)

struct RsEncode : Block<RsEncode> {
    using Description = Doc<R""(
@brief Reed-Solomon encode over the GF(64) family or a CCSDS GF(256) profile: information-symbol records in, codeword
records out, one for one.

Each input record carries a whole number of codeblocks, so its length must be a nonzero multiple of
`(block - roots - pad) * interleave` symbols, `block` being 63 in the GF(64) family and 255 under a CCSDS profile.
Each group of `block - roots - pad` symbols becomes one codeword of `block - pad` symbols on the wire, and the
`interleave` codewords of a codeblock alternate symbol by symbol there. A GF(64) item holds its symbol in the low six
bits, the bits above ignored on read and emitted zero; a CCSDS symbol is the whole byte, dual-basis where `basis`
says so.

An encoder has no status to report, so the record's metadata crosses unchanged; its signal name and its single-map
shape follow it, and the output record's extent names its own length. RsDecode is the counterpart.

A record whose length is not such a multiple is dropped and counted in `nRecordsRefused`, `stop()` states the total,
and the record that follows is processed normally, so a misaligned record costs one record rather than the stream.
)"">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "roots", Doc<"parity symbols per codeword: 8, 12 or 16 in the GF(64) family; under a CCSDS profile it may stay unset or must equal the profile's count">, Visible> roots{};
    Annotated<gr::Size_t, "pad", Doc<"the shortening in leading zero symbols neither end transmits; must leave the code room for information">>                                              pad = 0U;
    Annotated<std::string, "code", Doc<"'' selects the GF(64) family that roots names; 'ccsds_255_223' and 'ccsds_255_239' are the CCSDS profiles over GF(256)">, Visible>                   code{};
    Annotated<std::string, "basis", Doc<"'conventional' or 'dual', required under a CCSDS profile and empty for the GF(64) family, which has no dual basis">>                                basis{};
    Annotated<gr::Size_t, "interleave", Doc<"I, the codewords one record's codeblock holds; 4.3.5.1's {1,2,3,4,5,8} under a CCSDS profile, 1 to 8 otherwise">>                               interleave = 1U;

    GR_MAKE_REFLECTABLE(RsEncode, in, out, roots, pad, code, basis, interleave);

    detail::RsCode _code{};
    std::size_t    _pad        = 0UZ;
    std::size_t    _depth      = 1UZ;
    bool           _dual       = false;
    bool           _configured = false;

    std::vector<std::uint8_t> _infoBlock{}; //!< one codeblock's information symbols, in wire order
    std::vector<std::uint8_t> _infoWords{}; //!< the same symbols gathered codeword by codeword
    std::vector<std::uint8_t> _wireWords{}; //!< the encoded codewords, before interleaving back out

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords        = 0ULL; ///< records published on `out`
    std::uint64_t nCodewords      = 0ULL; ///< codewords those records carry
    std::uint64_t nRecordsRefused = 0ULL; ///< records whose length was not a multiple of the information length

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        const detail::RsCode named = detail::reedSolomonCode(code.value, roots.value);
        const bool           dual  = detail::reedSolomonDual(named, basis.value);
        const std::size_t    depth = detail::reedSolomonDepth(named, interleave.value);
        _pad                       = detail::reedSolomonPad(pad.value, named); // every setting is checked before any is kept
        _code                      = named;
        _dual                      = dual;
        _depth                     = depth;
        _configured                = true;
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 3UZ> counters{{{"records", nRecords}, {"codewords", nCodewords}, {"records refused", nRecordsRefused}}};
        detail::report("gr::blocks::fec::RsEncode", this->name, counters);
    }

    //! Information symbols in one codeword, which is also the input length one codeword consumes.
    [[nodiscard]] std::size_t infoSymbols() const noexcept { return _code.block - _code.roots - _pad; }

    //! Symbols one codeword occupies on the wire, the unshortened block minus the shortening.
    [[nodiscard]] std::size_t wireSymbols() const noexcept { return _code.block - _pad; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        const std::size_t info = infoSymbols();
        const std::size_t wire = wireSymbols();
        _infoBlock.resize(info * _depth);
        _infoWords.resize(info * _depth);
        _wireWords.resize(wire * _depth);

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record  = inSpan[consumed];
            const std::size_t            symbols = record.signal_values.size();
            if (symbols == 0UZ || symbols % (info * _depth) != 0UZ) {
                ++nRecordsRefused;
                continue;
            }
            const std::size_t codeblocks = symbols / (info * _depth);

            DataSet<std::uint8_t> coded;
            coded.signal_values.resize(codeblocks * wire * _depth);
            for (std::size_t cb = 0UZ; cb < codeblocks; ++cb) {
                // the information arrives in wire order — dual-basis strings where the profile says so,
                // the codewords interleaved — and the recoding is per symbol, so it comes off first
                for (std::size_t i = 0UZ; i < info * _depth; ++i) {
                    const std::uint8_t symbol = static_cast<std::uint8_t>(record.signal_values[cb * info * _depth + i] & _code.mask);
                    _infoBlock[i]             = _dual ? gr::fec::CcsdsDualBasis::fromDual(symbol) : symbol;
                }
                gr::fec::deinterleaveCodewords(_infoBlock, _infoWords, info, _depth);

                for (std::size_t w = 0UZ; w < _depth; ++w) {
                    std::array<std::uint8_t, detail::kRsMaxBlockSymbols> buffer{};
                    std::copy_n(_infoWords.begin() + static_cast<std::ptrdiff_t>(w * info), info, buffer.begin() + static_cast<std::ptrdiff_t>(_pad));
                    _code.encode(std::span(buffer.data(), _code.block), _pad);
                    std::copy_n(buffer.begin() + static_cast<std::ptrdiff_t>(_pad), wire, _wireWords.begin() + static_cast<std::ptrdiff_t>(w * wire));
                }

                const std::span<std::uint8_t> outBlock(coded.signal_values.data() + cb * wire * _depth, wire * _depth);
                gr::fec::interleaveCodewords(_wireWords, outBlock, wire, _depth);
                if (_dual) {
                    for (std::uint8_t& symbol : outBlock) {
                        symbol = gr::fec::CcsdsDualBasis::toDual(symbol);
                    }
                }
            }
            const std::size_t words = codeblocks * _depth;
            coded.extents.push_back(static_cast<std::int32_t>(coded.signal_values.size()));
            coded.signal_names.emplace_back(record.signal_names.empty() ? std::string("fec") : record.signal_names[0UZ]);
            coded.timing_events.resize(1UZ);
            coded.meta_information.resize(1UZ);
            if (!record.meta_information.empty()) {
                coded.meta_information[0UZ] = record.meta_information[0UZ]; // the record's facts carry through
            }

            ++nRecords;
            nCodewords += words;
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

GR_REGISTER_BLOCK(gr::blocks::fec::RsDecode)

struct RsDecode : Block<RsDecode> {
    using Description = Doc<R""(
@brief Reed-Solomon decode over the GF(64) family or a CCSDS GF(256) profile: codeword records in, information-symbol
records out, one for one, with the code's verdict in metadata.

Each input record carries a whole number of codeblocks, so its length must be a nonzero multiple of
`(block - pad) * interleave` symbols, `block` being 63 in the GF(64) family and 255 under a CCSDS profile, and each
codeword becomes `block - roots - pad` information symbols. A record whose length fails that test is dropped and
counted as RsEncode drops one.

The verdict is carried in metadata: `corrected_errors` gains this record's per-codeword error sum and
`uncorrectable_errors` the count of codewords the kernel reported invalid, each added to whatever the key already
carried, so a chain of correcting stages reports one total rather than its last stage's share. Every other key crosses
verbatim, and a record arriving without a metadata map gains one to carry the two status keys.

A codeword whose correction lands inside the padding is counted in `nPadCorrupted` and in `uncorrectable_errors` as
well: such a word violates the shortening both ends agreed on. The metadata carries the two status keys and the
separate counter keeps the distinction visible. Information symbols are emitted for such a codeword as for any other.
)"">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "roots", Doc<"parity symbols per codeword: 8, 12 or 16 in the GF(64) family; under a CCSDS profile it may stay unset or must equal the profile's count">, Visible> roots{};
    Annotated<gr::Size_t, "pad", Doc<"the shortening in leading zero symbols neither end transmits; must leave the code room for information">>                                              pad = 0U;
    Annotated<std::string, "code", Doc<"'' selects the GF(64) family that roots names; 'ccsds_255_223' and 'ccsds_255_239' are the CCSDS profiles over GF(256)">, Visible>                   code{};
    Annotated<std::string, "basis", Doc<"'conventional' or 'dual', required under a CCSDS profile and empty for the GF(64) family, which has no dual basis">>                                basis{};
    Annotated<gr::Size_t, "interleave", Doc<"I, the codewords one record's codeblock holds; 4.3.5.1's {1,2,3,4,5,8} under a CCSDS profile, 1 to 8 otherwise">>                               interleave = 1U;

    GR_MAKE_REFLECTABLE(RsDecode, in, out, roots, pad, code, basis, interleave);

    detail::RsCode _code{};
    std::size_t    _pad        = 0UZ;
    std::size_t    _depth      = 1UZ;
    bool           _dual       = false;
    bool           _configured = false;

    std::vector<std::uint8_t> _codeblock{}; //!< one codeblock's wire symbols, the recoding taken off
    std::vector<std::uint8_t> _codewords{}; //!< the same symbols gathered codeword by codeword
    std::vector<std::uint8_t> _infoWords{}; //!< the corrected information regions, codeword by codeword

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords                = 0ULL; ///< records published on `out`
    std::uint64_t nCodewords              = 0ULL; ///< codewords those records carried
    std::uint64_t nRecordsRefused         = 0ULL; ///< records whose length was not a multiple of the wire length
    std::uint64_t nCorrectedErrors        = 0ULL; ///< symbols the kernel corrected, totaled over every codeword
    std::uint64_t nUncorrectableCodewords = 0ULL; ///< codewords the kernel reported invalid
    std::uint64_t nPadCorrupted           = 0ULL; ///< codewords whose correction fell in the shortening's padding

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        const detail::RsCode named = detail::reedSolomonCode(code.value, roots.value);
        const bool           dual  = detail::reedSolomonDual(named, basis.value);
        const std::size_t    depth = detail::reedSolomonDepth(named, interleave.value);
        _pad                       = detail::reedSolomonPad(pad.value, named); // every setting is checked before any is kept
        _code                      = named;
        _dual                      = dual;
        _depth                     = depth;
        _configured                = true;
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 6UZ> counters{{{"records", nRecords}, {"codewords", nCodewords}, {"records refused", nRecordsRefused}, //
            {"corrected errors", nCorrectedErrors}, {"uncorrectable codewords", nUncorrectableCodewords}, {"pad corrupted", nPadCorrupted}}};
        detail::report("gr::blocks::fec::RsDecode", this->name, counters);
    }

    //! Information symbols one codeword yields.
    [[nodiscard]] std::size_t infoSymbols() const noexcept { return _code.block - _code.roots - _pad; }

    //! Symbols one codeword occupies on the wire, which is also the input length one codeword consumes.
    [[nodiscard]] std::size_t wireSymbols() const noexcept { return _code.block - _pad; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        const std::size_t info = infoSymbols();
        const std::size_t wire = wireSymbols();
        _codeblock.resize(wire * _depth);
        _codewords.resize(wire * _depth);
        _infoWords.resize(info * _depth);

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record  = inSpan[consumed];
            const std::size_t            symbols = record.signal_values.size();
            if (symbols == 0UZ || symbols % (wire * _depth) != 0UZ) {
                ++nRecordsRefused;
                continue;
            }
            const std::size_t codeblocks = symbols / (wire * _depth);
            const std::size_t words      = codeblocks * _depth;

            DataSet<std::uint8_t> recovered;
            recovered.signal_values.resize(words * info);
            gr::Size_t corrected     = 0U;
            gr::Size_t uncorrectable = 0U;
            gr::Size_t padCorrupted  = 0U;
            for (std::size_t cb = 0UZ; cb < codeblocks; ++cb) {
                // the recoding is per symbol and the interleaving per index, so the basis comes off
                // the whole codeblock first and the deinterleave sees plain field elements
                for (std::size_t i = 0UZ; i < wire * _depth; ++i) {
                    const std::uint8_t symbol = static_cast<std::uint8_t>(record.signal_values[cb * wire * _depth + i] & _code.mask);
                    _codeblock[i]             = _dual ? gr::fec::CcsdsDualBasis::fromDual(symbol) : symbol;
                }
                gr::fec::deinterleaveCodewords(_codeblock, _codewords, wire, _depth);

                for (std::size_t w = 0UZ; w < _depth; ++w) {
                    std::array<std::uint8_t, detail::kRsMaxBlockSymbols> buffer{};
                    std::copy_n(_codewords.begin() + static_cast<std::ptrdiff_t>(w * wire), wire, buffer.begin() + static_cast<std::ptrdiff_t>(_pad));
                    const gr::fec::RsResult result = _code.decode(std::span(buffer.data(), _code.block), _pad);
                    std::copy_n(buffer.begin() + static_cast<std::ptrdiff_t>(_pad), info, _infoWords.begin() + static_cast<std::ptrdiff_t>(w * info));
                    corrected += result.errors;
                    if (result.pad_corrupted) {
                        ++padCorrupted;
                    }
                    if (!result.valid) { // a corrupted padding already reads as invalid, and is counted here once
                        ++uncorrectable;
                    }
                }

                // the transfer frame's octets are themselves dual-basis strings, so the information
                // leaves in the interleaved wire order with the recoding put back on
                const std::span<std::uint8_t> outBlock(recovered.signal_values.data() + cb * info * _depth, info * _depth);
                gr::fec::interleaveCodewords(_infoWords, outBlock, info, _depth);
                if (_dual) {
                    for (std::uint8_t& symbol : outBlock) {
                        symbol = gr::fec::CcsdsDualBasis::toDual(symbol);
                    }
                }
            }
            recovered.extents.push_back(static_cast<std::int32_t>(recovered.signal_values.size()));
            recovered.signal_names.emplace_back(record.signal_names.empty() ? std::string("fec") : record.signal_names[0UZ]);
            recovered.timing_events.resize(1UZ);
            recovered.meta_information.resize(1UZ);
            property_map& map = recovered.meta_information[0UZ];
            if (!record.meta_information.empty()) {
                map = record.meta_information[0UZ]; // the record's facts carry through, the status keys over them
            }
            const gr::Size_t priorCorrected     = metaOr<gr::Size_t>(map, "corrected_errors", 0U);
            const gr::Size_t priorUncorrectable = metaOr<gr::Size_t>(map, "uncorrectable_errors", 0U);
            map["corrected_errors"]             = gr::Size_t{priorCorrected + corrected};
            map["uncorrectable_errors"]         = gr::Size_t{priorUncorrectable + uncorrectable};

            ++nRecords;
            nCodewords += words;
            nCorrectedErrors += corrected;
            nUncorrectableCodewords += uncorrectable;
            nPadCorrupted += padCorrupted;
            outSpan[made] = std::move(recovered);
            ++made;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    template<typename V>
    [[nodiscard]] static V metaOr(const property_map& map, const char* key, V fallback) {
        if (const auto entry = map.find(property_map::key_type(key)); entry != map.end()) {
            return entry->second.value_or(V(fallback));
        }
        return fallback;
    }
};

} // namespace gr::blocks::fec

#endif // GNURADIO_FEC_RS_BLOCKS_HPP
