/* The gate for the console renderer.
 *
 * The records here are what `ModeSDecode` produced from frames built to the standard, so a case states what the
 * pair does end to end and cannot drift from the decoder's own key spellings. What is asserted is the line, because
 * the line is this block's whole contract: the substance of the record is asserted next door.
 */
#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <format>
#include <memory_resource>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/adsb/AdsbPrinter.hpp>
#include <gnuradio-4.0/adsb/ModeS.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::adsb::AdsbPrinter;
using gr::blocks::adsb::ModeSDecode;
using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;

using Record = gr::DataSet<std::uint8_t>;

constexpr float kRate = 2.0e6f;

/// @brief A long Mode S frame under construction, written by the standard's bit numbers.
struct Frame {
    std::vector<std::uint8_t> octets = std::vector<std::uint8_t>(14UZ, std::uint8_t{0});

    Frame& put(std::size_t first, std::size_t count, std::uint32_t value) {
        for (std::size_t i = 0UZ; i < count; ++i) {
            const std::size_t   bit  = first - 1UZ + i;
            const std::uint32_t one  = (value >> (count - 1UZ - i)) & 1U;
            const auto          mask = static_cast<std::uint8_t>(0x80U >> (bit % 8UZ));
            octets[bit / 8UZ]        = static_cast<std::uint8_t>((octets[bit / 8UZ] & static_cast<std::uint8_t>(~mask)) | (one != 0U ? mask : std::uint8_t{0}));
        }
        return *this;
    }
};

[[nodiscard]] Frame squitter(std::uint32_t address, std::uint32_t typeCode) {
    Frame frame;
    frame.put(1UZ, 5UZ, 17U);
    frame.put(6UZ, 3UZ, 5U);
    frame.put(9UZ, 24UZ, address);
    frame.put(33UZ, 5UZ, typeCode);
    return frame;
}

[[nodiscard]] Record recordOf(const Frame& frame, double seconds, std::optional<bool> crcOk = std::optional<bool>{true}) {
    Record record;
    record.signal_values = frame.octets;
    record.extents.push_back(static_cast<std::int32_t>(record.signal_values.size()));
    record.signal_names.emplace_back("payload");
    record.signal_quantities.emplace_back("");
    record.signal_units.emplace_back("");
    record.timing_events.emplace_back();
    record.meta_information.emplace_back();

    gr::property_map& meta = record.meta_information[0UZ];
    meta.insert_or_assign(gr::property_map::key_type("sample_start"), gr::pmt::Value(static_cast<std::uint64_t>(seconds * static_cast<double>(kRate))));
    meta.insert_or_assign(gr::property_map::key_type("sample_rate"), gr::pmt::Value(kRate));
    meta.insert_or_assign(gr::property_map::key_type("sequence"), gr::pmt::Value(std::uint64_t{7}));
    if (crcOk.has_value()) {
        meta.insert_or_assign(gr::property_map::key_type("crc_ok"), gr::pmt::Value(*crcOk));
    }
    return record;
}

/// @brief @p frames through `ModeSDecode` and then `AdsbPrinter`, as the lines and the records the printer published.
struct Printed {
    std::vector<std::string> lines{};
    std::vector<Record>      records{};
    std::uint64_t            named   = 0ULL;
    std::uint64_t            evicted = 0ULL;
};

[[nodiscard]] Printed print(std::vector<Record> records, gr::property_map settings = {}) {
    ModeSDecode decoder;
    decoder.settings().init();
    std::ignore = decoder.settings().applyStagedParameters();
    decoder.start();

    std::vector<Record> decoded(records.size());
    InputSpan<Record>   decoderIn{std::span<const Record>(records)};
    OutputSpan<Record>  decoderOut{std::span<Record>(decoded)};
    std::ignore = decoder.processBulk(decoderIn, decoderOut);
    decoded.resize(decoderOut.count);

    AdsbPrinter printer(std::move(settings));
    printer.settings().init();
    std::ignore = printer.settings().applyStagedParameters();
    printer.start();

    std::vector<Record> rendered(decoded.size());
    InputSpan<Record>   printerIn{std::span<const Record>(decoded)};
    OutputSpan<Record>  printerOut{std::span<Record>(rendered)};
    std::ignore = printer.processBulk(printerIn, printerOut);

    Printed result;
    result.records.assign(rendered.begin(), rendered.begin() + static_cast<std::ptrdiff_t>(printerOut.count));
    for (const Record& record : result.records) {
        result.lines.emplace_back(record.signal_values.begin(), record.signal_values.end());
    }
    result.named   = printer.nNamed;
    result.evicted = printer.nEvicted;
    return result;
}

[[nodiscard]] std::optional<std::string> readText(const gr::property_map& meta, std::string_view key) {
    const auto entry = meta.find(gr::property_map::key_type(key));
    if (entry == meta.end()) {
        return std::nullopt;
    }
    const std::pmr::string* value = entry->second.get_if<std::pmr::string>();
    return value == nullptr ? std::nullopt : std::optional<std::string>{std::string(*value)};
}

/// @brief An identification message from @p address spelling @p characters, padded to eight.
[[nodiscard]] Frame identityFrame(std::uint32_t address, std::span<const std::uint32_t> characters) {
    Frame frame = squitter(address, 4U);
    frame.put(38UZ, 3UZ, 3U);
    for (std::size_t i = 0UZ; i < 8UZ; ++i) {
        frame.put(41UZ + 6UZ * i, 6UZ, i < characters.size() ? characters[i] : 32U);
    }
    return frame;
}

/// `AAL160` and `UAL597` in the six-bit set: letters are 1 to 26 and digits 48 to 57.
constexpr std::array<std::uint32_t, 6UZ> kAal160{1U, 1U, 12U, 49U, 54U, 48U};
constexpr std::array<std::uint32_t, 6UZ> kUal597{21U, 1U, 12U, 53U, 57U, 55U};

} // namespace

const boost::ut::suite<"adsb-printer"> printerTests = [] {
    using namespace boost::ut;

    "each message renders the line its fields describe, and the time comes off the frame"_test = [] {
        Frame position = squitter(0xAB0969U, 11U);
        position.put(41UZ, 12UZ, 0xC90U); // Q set, 39 000 ft
        position.put(54UZ, 1UZ, 0U).put(55UZ, 17UZ, 8836U).put(72UZ, 17UZ, 36676U);

        Frame velocity = squitter(0xA9A901U, 19U);
        velocity.put(38UZ, 3UZ, 1U).put(46UZ, 1UZ, 0U).put(47UZ, 10UZ, 341U).put(57UZ, 1UZ, 1U).put(58UZ, 10UZ, 78U);
        velocity.put(68UZ, 1UZ, 1U).put(69UZ, 1UZ, 1U).put(70UZ, 9UZ, 17U);

        const Printed printed = print({recordOf(identityFrame(0xAB0969U, kAal160), 29.768), recordOf(position, 29.781), recordOf(velocity, 29.782)});
        expect(eq(printed.lines.size(), 3UZ));
        expect(eq(printed.lines[0UZ], std::string("   29.768  DF17  AB0969  ident   callsign AAL160")));
        expect(eq(printed.lines[1UZ], std::string("   29.781  DF17  AB0969  airpos  alt 39000 ft  [AAL160]")));
        expect(eq(printed.lines[2UZ], std::string("   29.782  DF17  A9A901  veloc   gs 349 kt  track 103 deg  vs -1024 ft/min")));
        expect(eq(printed.named, 1ULL)) << "the position line is the one that took a remembered name";
    };

    "a resolved position joins the altitude on the line"_test = [] {
        // the two halves of one pair, an aircraft over 42.40448 N 71.34696 W, encoded to the standard
        Frame even = squitter(0xAB0969U, 11U);
        even.put(41UZ, 12UZ, 0xC90U).put(54UZ, 1UZ, 0U).put(55UZ, 17UZ, 8836U).put(72UZ, 17UZ, 36676U);
        Frame odd = squitter(0xAB0969U, 11U);
        odd.put(41UZ, 12UZ, 0xC90U).put(54UZ, 1UZ, 1U).put(55UZ, 17UZ, 124469U).put(72UZ, 17UZ, 62653U);

        // the even half arrives second, so it is the newer one the global decode resolves against
        const Printed printed = print({recordOf(odd, 29.781), recordOf(even, 30.781)});
        expect(eq(printed.lines.size(), 2UZ));
        expect(eq(printed.lines[0UZ], std::string("   29.781  DF17  AB0969  airpos  alt 39000 ft")));
        expect(eq(printed.lines[1UZ], std::string("   30.781  DF17  AB0969  airpos  alt 39000 ft  lat 42.40448  lon -71.34696")));
    };

    "a frame the decoder would not read prints its protocol and its octets"_test = [] {
        Frame frame = squitter(0xA0B803U, 19U);
        frame.put(38UZ, 3UZ, 1U);

        const Printed printed = print({recordOf(frame, 1.5, std::optional<bool>{false}), recordOf(frame, 2.5, std::optional<bool>{})});
        expect(eq(printed.lines.size(), 2UZ));
        expect(printed.lines[0UZ].starts_with("    1.500  mode_s/unchecked  8DA0B803")) << printed.lines[0UZ];
        expect(printed.lines[1UZ].starts_with("    2.500  mode_s/short  8DA0B803")) << printed.lines[1UZ];
        expect(eq(printed.named, 0ULL));
    };

    "an unhandled type code is named rather than guessed at"_test = [] {
        const Printed printed = print({recordOf(squitter(0xA0B803U, 31U), 29.768), recordOf(squitter(0xA0B803U, 6U), 29.958)});
        expect(eq(printed.lines.size(), 2UZ));
        expect(eq(printed.lines[0UZ], std::string("   29.768  DF17  A0B803  tc 31 (operational_status)")));
        expect(eq(printed.lines[1UZ], std::string("   29.958  DF17  A0B803  tc 6 (surface_position)")));
    };

    "the text record carries the line, the input's keys and a protocol of its own"_test = [] {
        const Printed printed = print({recordOf(identityFrame(0xAB0969U, kAal160), 29.768)});
        expect(eq(printed.records.size(), 1UZ));
        const gr::property_map& meta = printed.records[0UZ].meta_information.at(0UZ);
        expect(eq(readText(meta, "protocol").value_or(std::string("<absent>")), std::string("text/adsb")));
        expect(eq(readText(meta, "mode_s_icao").value_or(std::string("<absent>")), std::string("AB0969")));
        expect(eq(readText(meta, "adsb_callsign").value_or(std::string("<absent>")), std::string("AAL160")));
        expect(eq(printed.records[0UZ].signal_names.at(0UZ), std::string("text")));
        expect(eq(static_cast<std::size_t>(printed.records[0UZ].extents.at(0UZ)), printed.lines[0UZ].size()));
        expect(!printed.lines[0UZ].ends_with("\n")) << "the newline is the consumer's";
    };

    "the name table is bounded, and the address it gives up loses its name"_test = [] {
        // two names into a table that holds one: the first is gone by the time its second frame arrives
        const Printed printed = print({recordOf(identityFrame(0xAB0969U, kAal160), 1.0),    //
                                          recordOf(identityFrame(0xA0B803U, kUal597), 2.0), //
                                          recordOf(squitter(0xA0B803U, 31U), 3.0),          //
                                          recordOf(squitter(0xAB0969U, 31U), 4.0)},
            {{"table_size", gr::Size_t{1}}});
        expect(eq(printed.lines.size(), 4UZ));
        expect(eq(printed.lines[2UZ], std::string("    3.000  DF17  A0B803  tc 31 (operational_status)  [UAL597]")));
        expect(eq(printed.lines[3UZ], std::string("    4.000  DF17  AB0969  tc 31 (operational_status)")));
        expect(eq(printed.evicted, 1ULL));
    };

    "an empty span says so rather than reporting progress it did not make"_test = [] {
        AdsbPrinter printer;
        printer.settings().init();
        std::ignore = printer.settings().applyStagedParameters();
        printer.start();

        std::vector<Record> none;
        std::vector<Record> room(4UZ);
        InputSpan<Record>   inSpan{std::span<const Record>(none)};
        OutputSpan<Record>  outSpan{std::span<Record>(room)};
        expect(printer.processBulk(inSpan, outSpan) == gr::work::Status::INSUFFICIENT_INPUT_ITEMS);
        expect(eq(outSpan.count, 0UZ));
        expect(!printer.to_stdout.value) << "a block in a graph writes to its port, not to a terminal, unless asked";
    };
};

int main() { /* not needed for UT */ }
