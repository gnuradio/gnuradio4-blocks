#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/algorithm/digital/HeaderFormat.hpp>
#include <gnuradio-4.0/digital/AccessCodeCorrelator.hpp>
#include <gnuradio-4.0/digital/LengthHeaderFramer.hpp>
#include <gnuradio-4.0/digital/PacketFramer.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::digital::AccessCodeCorrelator;
using gr::blocks::digital::LengthHeaderFramer;
using gr::blocks::digital::PacketFramer;
using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;

using Record = gr::DataSet<std::uint8_t>;

constexpr std::uint64_t kSyncWord = 0xACDDA4E2F28C20FCULL;
constexpr std::size_t   kSyncBits = 64UZ;

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
    [[nodiscard]] std::size_t below(std::size_t bound) noexcept { return next() % bound; }
};

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

[[nodiscard]] std::string codeString(std::uint64_t word, std::size_t bits) {
    std::string text(bits, '0');
    for (std::size_t i = 0UZ; i < bits; ++i) {
        text[i] = ((word >> (bits - 1UZ - i)) & 1ULL) != 0ULL ? '1' : '0';
    }
    return text;
}

/// One packet as the chain carries it: a flat item array with its extent, its name and its own metadata.
[[nodiscard]] Record recordOf(std::vector<std::uint8_t> items, gr::property_map meta = {}) {
    Record record;
    record.signal_values = std::move(items);
    record.extents.push_back(static_cast<std::int32_t>(record.signal_values.size()));
    record.signal_names.emplace_back("payload");
    record.signal_quantities.emplace_back("");
    record.signal_units.emplace_back("");
    record.meta_information.push_back(std::move(meta));
    record.timing_events.emplace_back();
    return record;
}

[[nodiscard]] std::vector<std::uint8_t> randomBytes(std::size_t count, Rng& rng) {
    std::vector<std::uint8_t> bytes(count);
    for (std::uint8_t& byte : bytes) {
        byte = static_cast<std::uint8_t>(rng.next() & 0xFFULL);
    }
    return bytes;
}

[[nodiscard]] gr::Size_t metaSize(const Record& record, std::string_view key) {
    if (record.meta_information.empty()) {
        return std::numeric_limits<gr::Size_t>::max();
    }
    const auto entry = record.meta_information.front().find(gr::property_map::key_type(key));
    return entry == record.meta_information.front().end() ? std::numeric_limits<gr::Size_t>::max() : entry->second.value_or(std::numeric_limits<gr::Size_t>::max());
}

[[nodiscard]] std::uint64_t metaU64(const Record& record, std::string_view key) {
    if (record.meta_information.empty()) {
        return ~0ULL;
    }
    const auto entry = record.meta_information.front().find(gr::property_map::key_type(key));
    return entry == record.meta_information.front().end() ? ~0ULL : entry->second.value_or(std::uint64_t{~0ULL});
}

/// @brief One drive of the framer over @p records, everything published in one call.
[[nodiscard]] std::vector<Record> frameAll(LengthHeaderFramer& block, std::span<const Record> records) {
    std::vector<Record> scratch(std::max(records.size(), 1UZ));
    InputSpan<Record>   inSpan{records};
    OutputSpan<Record>  outSpan{std::span<Record>(scratch)};
    std::ignore = block.processBulk(inSpan, outSpan);

    std::vector<Record> out;
    for (std::size_t i = 0UZ; i < outSpan.count; ++i) {
        out.push_back(std::move(scratch[i]));
    }
    return out;
}

/// @brief The framed records laid end to end, which is what `gr::blocks::basic::DataSetToStream` puts on the wire. It
/// is done by hand here because this test drives the framer through spans rather than a scheduler, and what it is
/// checking is the round trip of items and metadata rather than that block's own tag machinery.
[[nodiscard]] std::vector<std::uint8_t> concatenated(std::span<const Record> framed) {
    std::vector<std::uint8_t> stream;
    for (const Record& record : framed) {
        stream.insert(stream.end(), record.signal_values.begin(), record.signal_values.end());
    }
    return stream;
}

/// @brief The receiving half of the chain: the detector, then the framer with the transmit side's own settings.
[[nodiscard]] std::vector<gr::DataSet<std::uint8_t>> received(std::span<const std::uint8_t> stream, gr::property_map framerSettings) {
    AccessCodeCorrelator<std::uint8_t> detector = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", codeString(kSyncWord, kSyncBits)}});
    const auto                         tagged   = gr::blocks::testing::span::run<std::uint8_t>(detector, stream, 4096UZ);

    PacketFramer<std::uint8_t> framer = make<PacketFramer<std::uint8_t>>(std::move(framerSettings));
    return gr::blocks::testing::span::runVariable<gr::DataSet<std::uint8_t>>(framer, std::span<const std::uint8_t>(tagged.samples), 0UZ, 64UZ, std::span<const gr::Tag>(tagged.tags)).samples;
}

/// @brief The settings both ends of a chain carry, with the transmit-only ones added by the caller.
[[nodiscard]] gr::property_map layout(std::string_view name, std::string_view covers, gr::Size_t lengthBits, gr::Size_t unit, gr::Size_t check, gr::Size_t prefix, gr::Size_t bound) {
    return gr::property_map{{"header_format", std::string(name)}, {"max_payload_items", bound}, {"length_bits", lengthBits}, //
        {"count_covers", std::string(covers)}, {"count_unit_items", unit}, {"check_items", check}, {"frame_prefix_items", prefix}};
}

[[nodiscard]] gr::property_map with(gr::property_map settings, gr::property_map extra) {
    for (auto& [key, value] : extra) {
        settings.insert_or_assign(key, value);
    }
    return settings;
}

/// @brief What @p call complains about, which is where a rejection has to name the value that caused it.
template<typename TCall>
[[nodiscard]] std::string complaint(TCall&& call) {
    try {
        call();
    } catch (const std::exception& error) {
        return std::string(error.what());
    }
    return std::string{};
}

} // namespace

const boost::ut::suite<"length header framer"> lengthHeaderFramerTests = [] {
    using namespace boost::ut;

    "the transmit side writes the field value the count arithmetic names"_test = [] {
        // the worked example the other end reads: a 34-byte record whose last two bytes are a CRC-16, under a 16-bit
        // field that counts the payload alone, writes 32 and puts 272 items on the wire
        Rng                       rng{};
        const gr::property_map    settings = with(layout("length_plain", "payload", 16U, 8U, 16U, 0U, 4096U), {{"sync_word", codeString(kSyncWord, kSyncBits)}, {"payload_pack_bits", static_cast<gr::Size_t>(8)}});
        LengthHeaderFramer        framer   = make<LengthHeaderFramer>(settings);
        const std::vector<Record> sent{recordOf(randomBytes(34UZ, rng))};

        const std::vector<Record> framed = frameAll(framer, std::span<const Record>(sent));
        expect(eq(framed.size(), 1UZ));
        if (framed.empty()) {
            return;
        }

        expect(eq(framed[0UZ].signal_values.size(), kSyncBits + 16UZ + 272UZ)) << "sync word, header and the unpacked payload";
        expect(eq(metaSize(framed[0UZ], "header_items"), gr::Size_t{16})) << "the record says how many items the header occupied";
        const std::span<const std::uint8_t> header(framed[0UZ].signal_values.data() + kSyncBits, 16UZ);
        expect(eq(gr::digital::readHeaderField(header, 0UZ, 16UZ), std::uint32_t{32})) << "(272 - 16) / 8 = 32, which is what the field carries";
        expect(eq(framer.nRecords, 1ULL));
        expect(eq(framer.nItemsEmitted, kSyncBits + 16UZ + 272UZ));
    };

    "transmit and receive are one chain, over both realizations and every convention"_test = [] {
        struct Row {
            const char* name;
            const char* covers;
            gr::Size_t  lengthBits;
            gr::Size_t  unit;
            gr::Size_t  check;
            gr::Size_t  prefix;
            const char* byteOrder;
            const char* flagsAt;
        };
        const Row kRows[]{{"length_plain", "payload", 16U, 8U, 16U, 0U, "big", "high"}, //
            {"length_plain", "payload_and_check", 16U, 8U, 0U, 0U, "big", "high"},      //
            {"length_plain", "whole_frame", 16U, 8U, 0U, 0U, "big", "high"},            //
            {"length_plain", "payload_and_check", 16U, 8U, 0U, 0U, "little", "high"},   //
            {"length_golay24", "payload_and_check", 8U, 8U, 0U, 0U, "big", "high"},     //
            {"length_golay24", "payload", 8U, 8U, 16U, 0U, "big", "low"}};

        for (const Row& row : kRows) {
            const bool             golay    = std::string_view(row.name) == "length_golay24";
            const gr::property_map both     = layout(row.name, row.covers, row.lengthBits, row.unit, row.check, row.prefix, 4096U);
            gr::property_map       transmit = with(both, {{"sync_word", codeString(kSyncWord, kSyncBits)}, {"payload_pack_bits", static_cast<gr::Size_t>(8)}});
            gr::property_map       receive  = both;
            if (golay) {
                transmit.insert_or_assign("flags_position", std::string(row.flagsAt));
                receive.insert_or_assign("flags_position", std::string(row.flagsAt));
            } else {
                transmit.insert_or_assign("byte_order", std::string(row.byteOrder));
                receive.insert_or_assign("byte_order", std::string(row.byteOrder));
            }

            Rng                 rng{};
            std::vector<Record> sent;
            for (std::size_t which = 0UZ; which < 60UZ; ++which) {
                // the check field is carried on the wire and counted or not by the convention, so the record has to be
                // long enough to hold it whatever the convention does with the number
                const std::size_t bytes = static_cast<std::size_t>(row.check) / 8UZ + 1UZ + rng.below(30UZ);
                gr::property_map  meta{{"source_id", std::string("qa")}};
                if (golay) {
                    meta.insert_or_assign(gr::property_map::key_type("header_flags"), std::uint64_t{which % 16UZ});
                }
                sent.push_back(recordOf(randomBytes(bytes, rng), std::move(meta)));
            }

            LengthHeaderFramer        framer = make<LengthHeaderFramer>(transmit);
            const std::vector<Record> framed = frameAll(framer, std::span<const Record>(sent));
            expect(eq(framed.size(), sent.size())) << std::format("{} / {} / {}: every record is framed", row.name, row.covers, golay ? row.flagsAt : row.byteOrder);
            expect(eq(framer.nRecordsRefused, 0ULL));
            expect(eq(framer.nLengthUnrepresentable, 0ULL));
            expect(eq(framer.nMetaRefused, 0ULL));

            const std::vector<std::uint8_t>              wire = concatenated(std::span<const Record>(framed));
            const std::vector<gr::DataSet<std::uint8_t>> back = received(std::span<const std::uint8_t>(wire), receive);
            expect(eq(back.size(), sent.size())) << std::format("{} / {}: every frame is recovered", row.name, row.covers);

            for (std::size_t which = 0UZ; which < std::min(back.size(), sent.size()); ++which) {
                // the receiving end works in wire items, so the comparison is against the record unpacked the same way
                std::vector<std::uint8_t> expected(sent[which].signal_values.size() * 8UZ);
                for (std::size_t byte = 0UZ; byte < sent[which].signal_values.size(); ++byte) {
                    for (std::size_t bit = 0UZ; bit < 8UZ; ++bit) {
                        expected[byte * 8UZ + bit] = static_cast<std::uint8_t>((sent[which].signal_values[byte] >> (7UZ - bit)) & 1U);
                    }
                }
                expect(that % (back[which].signal_values == expected)) << std::format("{} / {}: record {}", row.name, row.covers, which);
                if (golay) {
                    expect(eq(metaU64(back[which], "header_flags"), std::uint64_t{which % 16UZ})) << std::format("{}: record {} flags", row.covers, which);
                    expect(eq(metaU64(back[which], "header_corrected_errors"), 0ULL));
                }
            }
        }
    };

    "each refusal is counted under its own cause and publishes nothing"_test = [] {
        Rng rng{};

        { // empty, and above the bound
            LengthHeaderFramer        framer = make<LengthHeaderFramer>(with(layout("length_plain", "payload_and_check", 16U, 8U, 0U, 0U, 64U), {{"payload_pack_bits", static_cast<gr::Size_t>(8)}}));
            const std::vector<Record> sent{recordOf({}), recordOf(randomBytes(16UZ, rng)), recordOf(randomBytes(4UZ, rng))};
            const std::vector<Record> framed = frameAll(framer, std::span<const Record>(sent));
            expect(eq(framed.size(), 1UZ)) << "only the record that fits is framed";
            expect(eq(framer.nRecordsRefused, 2ULL)) << "an empty record and one of 128 wire items against a bound of 64";
            expect(eq(framer.nLengthUnrepresentable, 0ULL));
        }

        { // an item count that is not a whole number of counted units cannot be framed by a length field at all
            LengthHeaderFramer        framer = make<LengthHeaderFramer>(layout("length_plain", "payload_and_check", 16U, 8U, 0U, 0U, 4096U));
            const std::vector<Record> sent{recordOf(std::vector<std::uint8_t>(12UZ, std::uint8_t{1})), recordOf(std::vector<std::uint8_t>(16UZ, std::uint8_t{1}))};
            const std::vector<Record> framed = frameAll(framer, std::span<const Record>(sent));
            expect(eq(framed.size(), 1UZ));
            expect(eq(framer.nLengthUnrepresentable, 1ULL)) << "12 wire items is not a whole number of 8-item units";
            expect(eq(framer.nMetaRefused, 0ULL)) << "and the cause is attributed to the length and not to the metadata";
        }

        { // a flag value too wide for the field the layout leaves for it
            const gr::property_map    settings = with(layout("length_golay24", "payload_and_check", 8U, 8U, 0U, 0U, 4096U), {{"payload_pack_bits", static_cast<gr::Size_t>(8)}});
            LengthHeaderFramer        framer   = make<LengthHeaderFramer>(settings);
            const std::vector<Record> sent{recordOf(randomBytes(8UZ, rng), {{"header_flags", std::uint64_t{99}}}), recordOf(randomBytes(8UZ, rng), {{"header_flags", std::uint64_t{9}}})};
            const std::vector<Record> framed = frameAll(framer, std::span<const Record>(sent));
            expect(eq(framed.size(), 1UZ));
            expect(eq(framer.nMetaRefused, 1ULL)) << "eight length bits leave four flag bits, and 99 does not fit them";
            expect(eq(framer.nLengthUnrepresentable, 0ULL)) << "the length was representable, so the cause is the metadata";
        }
    };

    "an empty sync word emits none, and a malformed one is refused"_test = [] {
        Rng                       rng{};
        LengthHeaderFramer        framer = make<LengthHeaderFramer>(with(layout("length_plain", "payload_and_check", 16U, 8U, 0U, 0U, 4096U), {{"payload_pack_bits", static_cast<gr::Size_t>(8)}}));
        const std::vector<Record> sent{recordOf(randomBytes(8UZ, rng))};
        const std::vector<Record> framed = frameAll(framer, std::span<const Record>(sent));
        expect(eq(framed.size(), 1UZ));
        expect(eq(framed[0UZ].signal_values.size(), 16UZ + 64UZ)) << "the header and the payload alone, for a chain whose preamble is generated upstream";

        const std::string tooLong = complaint([] { std::ignore = make<LengthHeaderFramer>(with(layout("length_plain", "payload_and_check", 16U, 8U, 0U, 0U, 4096U), {{"sync_word", std::string(65UZ, '1')}})); });
        expect(tooLong.contains("65")) << tooLong;
        const std::string notBinary = complaint([] { std::ignore = make<LengthHeaderFramer>(with(layout("length_plain", "payload_and_check", 16U, 8U, 0U, 0U, 4096U), {{"sync_word", std::string("0101x101")}})); });
        expect(notBinary.contains("0101x101")) << notBinary;
    };

    "max_payload_items is required and the block is inert without it"_test = [] {
        expect(throws([] {
            LengthHeaderFramer block{}; // no settings at all, so nothing is staged and nothing is validated until start
            block.start();
        })) << "the bound the receiving end refuses above is the bound this end must not send";

        Rng                       rng{};
        LengthHeaderFramer        block{};
        const std::vector<Record> sent{recordOf(randomBytes(8UZ, rng))};
        std::vector<Record>       scratch(1UZ);
        InputSpan<Record>         inSpan{std::span<const Record>(sent)};
        OutputSpan<Record>        outSpan{std::span<Record>(scratch)};
        expect(that % (block.processBulk(inSpan, outSpan) == gr::work::Status::ERROR));
        expect(eq(inSpan.consumed, 0UZ));
    };

    "the layout settings are the framer's, refused in the same places"_test = [] {
        const auto refuses = [](gr::property_map settings, std::string_view offender, std::string_view why) {
            const std::string message = complaint([&settings] { std::ignore = make<LengthHeaderFramer>(settings); });
            expect(!message.empty()) << why;
            expect(message.contains(offender)) << std::format("{}: \"{}\" is missing from \"{}\"", why, offender, message);
        };

        refuses({{"max_payload_items", 256U}, {"length_bits", 16U}}, "length_bits", "a width belongs to the length formats");
        refuses({{"max_payload_items", 256U}, {"header_format", std::string("length_plain")}, {"length_bits", 16U}}, "count_covers", "and count_covers has no default");
        refuses({{"max_payload_items", 256U}, {"header_format", std::string("length_golay24")}, {"length_bits", 13U}, {"count_covers", std::string("payload")}}, "13", "a Golay header carries twelve information bits");
        refuses({{"max_payload_items", 256U}, {"header_format", std::string("length_plain")}, {"length_bits", 16U}, {"count_covers", std::string("payload")}, {"payload_pack_bits", 9U}}, "9", "an item unpacks into at most eight bits");

        expect(nothrow([] { std::ignore = make<LengthHeaderFramer>({{"max_payload_items", 256U}}); })) << "and the realizations in use need none of them";
    };
};

int main() { /* not needed for UT */ }
