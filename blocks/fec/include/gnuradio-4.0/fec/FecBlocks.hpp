#ifndef GNURADIO_FEC_FEC_BLOCKS_HPP
#define GNURADIO_FEC_FEC_BLOCKS_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <format>
#include <string>
#include <string_view>
#include <utility>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/fec/Bch.hpp>
#include <gnuradio-4.0/algorithm/fec/Golay.hpp>
#include <gnuradio-4.0/algorithm/fec/Hamming.hpp>

#include <gnuradio-4.0/fec/RecordShape.hpp>

/**
 * The record-native adapters over the binary block codes in `gnuradio-4.0/algorithm/fec`.
 *
 * The carrier is a record of bits: a `DataSet<std::uint8_t>` with one bit per item, the low bit
 * holding the value and the bits above it ignored on read and emitted zero. That is the
 * convention the bit-stream blocks already speak, so an encoder or a decoder joins a chain of
 * packers, correlators and extractors without a conversion between them.
 *
 * Item `i` of a codeword is the codeword's transmitted bit `i`. The kernels hold that bit in
 * word bit `n - 1 - i`, the first transmitted bit taking the word's most significant position,
 * and the same rule maps a record's information bits onto the right-aligned information word a
 * kernel accepts and returns. The adapters pack items into a word, call the kernel and unpack
 * its answer; nothing else about the layout is decided here.
 */
namespace gr::blocks::fec {

namespace detail {

//! What a binary decoder reports, in the one shape the six forms share.
struct BinaryResult {
    std::uint32_t info   = 0U;    //!< the recovered information bits, right-aligned
    unsigned      errors = 0U;    //!< bits corrected, or the distance found when the decode failed
    bool          valid  = false; //!< the word lay inside the code's correcting radius
};

//! One binary form: its dimensions and the kernel entry points the `code` setting selects.
struct BinaryCode {
    std::size_t n = 0UZ; //!< bits in a codeword
    std::size_t k = 0UZ; //!< information bits a codeword carries

    std::uint64_t (*encode)(std::uint32_t) = nullptr;
    BinaryResult (*decode)(std::uint64_t)  = nullptr;
};

//! The form @p name spells, refusing the empty default and reporting through the graph's exception type.
[[nodiscard]] inline BinaryCode binaryCode(std::string_view name) {
    if (name == "golay24") {
        return {24UZ, 12UZ, //
            [](std::uint32_t info) -> std::uint64_t { return gr::fec::golay24Encode(static_cast<std::uint16_t>(info)); },
            [](std::uint64_t word) -> BinaryResult {
                const gr::fec::GolayResult r = gr::fec::golay24Decode(static_cast<std::uint32_t>(word));
                return {r.info, r.errors, r.valid};
            }};
    }
    if (name == "golay23") {
        return {23UZ, 12UZ, //
            [](std::uint32_t info) -> std::uint64_t { return gr::fec::golay23Encode(static_cast<std::uint16_t>(info)); },
            [](std::uint64_t word) -> BinaryResult {
                const gr::fec::GolayResult r = gr::fec::golay23Decode(static_cast<std::uint32_t>(word));
                return {r.info, r.errors, r.valid};
            }};
    }
    if (name == "golay18") {
        return {18UZ, 6UZ, //
            [](std::uint32_t info) -> std::uint64_t { return gr::fec::golay18Encode(static_cast<std::uint8_t>(info)); },
            [](std::uint64_t word) -> BinaryResult {
                const gr::fec::GolayResult r = gr::fec::golay18Decode(static_cast<std::uint32_t>(word));
                return {r.info, r.errors, r.valid};
            }};
    }
    if (name == "hamming15") {
        return {15UZ, 11UZ, //
            [](std::uint32_t info) -> std::uint64_t { return gr::fec::hamming1511Encode(static_cast<std::uint16_t>(info)); },
            [](std::uint64_t word) -> BinaryResult {
                const gr::fec::HammingResult r = gr::fec::hamming1511Decode(static_cast<std::uint16_t>(word));
                return {r.info, r.errors, r.valid};
            }};
    }
    if (name == "hamming10") {
        return {10UZ, 6UZ, //
            [](std::uint32_t info) -> std::uint64_t { return gr::fec::hamming1063Encode(static_cast<std::uint8_t>(info)); },
            [](std::uint64_t word) -> BinaryResult {
                const gr::fec::HammingResult r = gr::fec::hamming1063Decode(static_cast<std::uint16_t>(word));
                return {r.info, r.errors, r.valid};
            }};
    }
    if (name == "bch63") {
        return {63UZ, 16UZ, //
            [](std::uint32_t info) -> std::uint64_t { return gr::fec::bch63Encode(static_cast<std::uint16_t>(info)); },
            [](std::uint64_t word) -> BinaryResult {
                const gr::fec::Bch63Result r = gr::fec::bch63Decode(word);
                return {r.info, r.errors, r.valid};
            }};
    }
    if (name == "bch15_11") {
        return {15UZ, 11UZ, //
            [](std::uint32_t info) -> std::uint64_t { return gr::fec::Bch15_11::encode(static_cast<std::uint16_t>(info)); },
            [](std::uint64_t word) -> BinaryResult {
                const gr::fec::Bch15Result r = gr::fec::Bch15_11::decode(static_cast<std::uint16_t>(word));
                return {r.info, r.errors, r.valid};
            }};
    }
    if (name == "bch15_7") {
        return {15UZ, 7UZ, //
            [](std::uint32_t info) -> std::uint64_t { return gr::fec::Bch15_7::encode(static_cast<std::uint16_t>(info)); },
            [](std::uint64_t word) -> BinaryResult {
                const gr::fec::Bch15Result r = gr::fec::Bch15_7::decode(static_cast<std::uint16_t>(word));
                return {r.info, r.errors, r.valid};
            }};
    }
    if (name == "bch15_5") {
        return {15UZ, 5UZ, //
            [](std::uint32_t info) -> std::uint64_t { return gr::fec::Bch15_5::encode(static_cast<std::uint16_t>(info)); },
            [](std::uint64_t word) -> BinaryResult {
                const gr::fec::Bch15Result r = gr::fec::Bch15_5::decode(static_cast<std::uint16_t>(word));
                return {r.info, r.errors, r.valid};
            }};
    }
    if (name.empty()) {
        throw gr::exception("code has no default and must be set to 'golay24', 'golay23', 'golay18', 'hamming15', 'hamming10', 'bch63', 'bch15_11', 'bch15_7' or 'bch15_5': no code is universal, and a default would be an interoperability assumption nobody made");
    }
    throw gr::exception(std::format("code must be 'golay24', 'golay23', 'golay18', 'hamming15', 'hamming10', 'bch63', 'bch15_11', 'bch15_7' or 'bch15_5', got '{}'", name));
}

//! Read @p k items as one information or codeword field, the first item the most significant bit.
[[nodiscard]] inline std::uint64_t packBits(const std::uint8_t* items, std::size_t k) noexcept {
    std::uint64_t word = 0ULL;
    for (std::size_t i = 0UZ; i < k; ++i) {
        word = (word << 1U) | (items[i] & 1U);
    }
    return word;
}

//! Write the low @p k bits of @p word as items, the most significant bit first.
inline void unpackBits(std::uint64_t word, std::size_t k, std::uint8_t* items) noexcept {
    for (std::size_t i = 0UZ; i < k; ++i) {
        items[i] = static_cast<std::uint8_t>((word >> (k - 1UZ - i)) & 1ULL);
    }
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::fec::FecEncode)

struct FecEncode : Block<FecEncode> {
    using Description = Doc<R""(
@brief Binary block encode: bit records in, codeword records out, one for one, under the code `code` names.

Each input record carries a whole number of information words, so its length must be a nonzero multiple of the code's
`k`, and each group of `k` bits becomes one `n`-bit codeword. The codewords follow one another in the output record in
the order their information arrived.

An encoder has no status to report, so the record's metadata crosses unchanged; its signal name and its single-map
shape follow it, and the output record's extent names its own length. FecDecode is the counterpart.

A record whose length is not a multiple of `k` is dropped and counted in `nRecordsRefused`, `stop()` states the total,
and the record that follows is processed normally, so a misaligned record costs one record rather than the stream.
)"">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<std::string, "code", Doc<"'golay24', 'golay23', 'golay18', 'hamming15', 'hamming10', 'bch63', 'bch15_11', 'bch15_7' or 'bch15_5'; required, with no default">, Visible> code{};

    GR_MAKE_REFLECTABLE(FecEncode, in, out, code);

    detail::BinaryCode _code{};
    bool               _configured = false;

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords        = 0ULL; ///< records published on `out`
    std::uint64_t nCodewords      = 0ULL; ///< codewords those records carry
    std::uint64_t nRecordsRefused = 0ULL; ///< records whose length was not a multiple of the code's k

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        _code       = detail::binaryCode(code.value);
        _configured = true;
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 3UZ> counters{{{"records", nRecords}, {"codewords", nCodewords}, {"records refused", nRecordsRefused}}};
        detail::report("gr::blocks::fec::FecEncode", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record = inSpan[consumed];
            const std::size_t            bits   = record.signal_values.size();
            if (bits == 0UZ || bits % _code.k != 0UZ) {
                ++nRecordsRefused;
                continue;
            }
            const std::size_t words = bits / _code.k;

            DataSet<std::uint8_t> coded;
            coded.signal_values.resize(words * _code.n);
            for (std::size_t w = 0UZ; w < words; ++w) {
                const std::uint64_t info = detail::packBits(record.signal_values.data() + w * _code.k, _code.k);
                detail::unpackBits(_code.encode(static_cast<std::uint32_t>(info)), _code.n, coded.signal_values.data() + w * _code.n);
            }
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

GR_REGISTER_BLOCK(gr::blocks::fec::FecDecode)

struct FecDecode : Block<FecDecode> {
    using Description = Doc<R""(
@brief Binary block decode: codeword records in, information records out, one for one, with the code's verdict in
metadata.

Each input record carries a whole number of codewords, so its length must be a nonzero multiple of the code's `n`, and
each `n`-bit codeword becomes `k` information bits. A record whose length fails that test is dropped and counted as
FecEncode drops one.

The verdict is carried in metadata: `corrected_errors` gains this record's per-codeword error sum and
`uncorrectable_errors` the count of codewords the kernel reported invalid, each added to whatever the key already
carried, so a chain of correcting stages reports one total rather than its last stage's share. Every other key crosses
verbatim, and a record arriving without a metadata map gains one to carry the two status keys.

An uncorrectable codeword's information bits are emitted unmodified like any other's: the kernel returns its best
decode and the counts say what it is worth. Only the forms carrying an overall parity bit or an unassigned syndrome
report a failure at all; the perfect codes report every word as valid.
)"">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<std::string, "code", Doc<"'golay24', 'golay23', 'golay18', 'hamming15', 'hamming10', 'bch63', 'bch15_11', 'bch15_7' or 'bch15_5'; required, with no default">, Visible> code{};

    GR_MAKE_REFLECTABLE(FecDecode, in, out, code);

    detail::BinaryCode _code{};
    bool               _configured = false;

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords                = 0ULL; ///< records published on `out`
    std::uint64_t nCodewords              = 0ULL; ///< codewords those records carried
    std::uint64_t nRecordsRefused         = 0ULL; ///< records whose length was not a multiple of the code's n
    std::uint64_t nCorrectedErrors        = 0ULL; ///< bits the kernel corrected, totaled over every codeword
    std::uint64_t nUncorrectableCodewords = 0ULL; ///< codewords the kernel reported invalid

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        _code       = detail::binaryCode(code.value);
        _configured = true;
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 5UZ> counters{{{"records", nRecords}, {"codewords", nCodewords}, {"records refused", nRecordsRefused}, //
            {"corrected errors", nCorrectedErrors}, {"uncorrectable codewords", nUncorrectableCodewords}}};
        detail::report("gr::blocks::fec::FecDecode", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record = inSpan[consumed];
            const std::size_t            bits   = record.signal_values.size();
            if (bits == 0UZ || bits % _code.n != 0UZ) {
                ++nRecordsRefused;
                continue;
            }
            const std::size_t words = bits / _code.n;

            DataSet<std::uint8_t> info;
            info.signal_values.resize(words * _code.k);
            gr::Size_t corrected     = 0U;
            gr::Size_t uncorrectable = 0U;
            for (std::size_t w = 0UZ; w < words; ++w) {
                const detail::BinaryResult result = _code.decode(detail::packBits(record.signal_values.data() + w * _code.n, _code.n));
                detail::unpackBits(result.info, _code.k, info.signal_values.data() + w * _code.k);
                corrected += result.errors;
                if (!result.valid) {
                    ++uncorrectable;
                }
            }
            info.extents.push_back(static_cast<std::int32_t>(info.signal_values.size()));
            info.signal_names.emplace_back(record.signal_names.empty() ? std::string("fec") : record.signal_names[0UZ]);
            info.timing_events.resize(1UZ);
            info.meta_information.resize(1UZ);
            property_map& map = info.meta_information[0UZ];
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

#endif // GNURADIO_FEC_FEC_BLOCKS_HPP
