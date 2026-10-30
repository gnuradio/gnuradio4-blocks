#include <boost/ut.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/digital/DelimiterFramer.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_delimiter_framer {

using gr::blocks::digital::DelimiterFramer;
using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;

using Record = gr::DataSet<std::uint8_t>;

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

/// One packet as the chain carries it: a flat byte array with its extent, its name and its own metadata.
[[nodiscard]] Record recordOf(std::vector<std::uint8_t> bytes, gr::property_map meta = {}) {
    Record record;
    record.signal_values = std::move(bytes);
    record.extents.push_back(static_cast<std::int32_t>(record.signal_values.size()));
    record.signal_names.emplace_back("payload");
    record.signal_quantities.emplace_back("");
    record.signal_units.emplace_back("");
    record.meta_information.push_back(std::move(meta));
    record.timing_events.emplace_back();
    return record;
}

/// The HDLC profile: the flag, bit stuffing, and each payload byte unpacked least significant bit first.
[[nodiscard]] gr::property_map hdlcFraming(gr::Size_t bound) {
    return {{"end_delimiter", std::string("01111110")}, {"transparency", std::string("bit_stuffing")}, {"max_payload_items", bound}, //
        {"payload_pack_bits", gr::Size_t{8}}, {"payload_bit_order", std::string("LsbFirst")}};
}

[[nodiscard]] gr::property_map slipFraming(gr::Size_t bound) {
    return {{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}, {"transparency", std::string("byte_escape")}, //
        {"escape_item", gr::Size_t{0xDB}}, {"escape_map", std::vector<gr::Size_t>{0xDCU, 0xC0U, 0xDDU, 0xDBU}}, {"max_payload_items", bound}};
}

/// What one drive of the framer produced.
struct Framed {
    std::vector<Record> out{};
    std::size_t         consumed = 0UZ;
};

/// Drives the framer over @p records in windows of @p chunk records, with @p room records of output space.
[[nodiscard]] Framed feed(DelimiterFramer& block, std::span<const Record> records, std::size_t chunk = 0UZ, bool connected = true, std::size_t room = 64UZ) {
    Framed              result;
    const std::size_t   stride = chunk == 0UZ ? std::max(records.size(), 1UZ) : chunk;
    std::vector<Record> scratch(room);

    for (std::size_t base = 0UZ; base < records.size();) {
        const std::size_t  count = std::min(stride, records.size() - base);
        InputSpan<Record>  inSpan(records.subspan(base, count), base);
        OutputSpan<Record> outSpan(connected ? std::span<Record>(scratch) : std::span<Record>{}, 0UZ, nullptr, connected);

        std::ignore = block.processBulk(inSpan, outSpan);
        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            result.out.push_back(std::move(scratch[k]));
        }
        result.consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ) {
            break;
        }
        base += inSpan.consumed;
    }
    return result;
}

} // namespace qa_delimiter_framer

using namespace qa_delimiter_framer;

const boost::ut::suite<"delimiter framer"> delimiterFramerTests = [] {
    using namespace boost::ut;

    "the adapter conforms: publish discipline and chunk independence"_test = [] {
        const std::vector<Record> records{recordOf({0x11U, 0x22U}, {{"packet", gr::Size_t{7}}, {"origin", std::string("qa")}}), //
            recordOf({0xC0U, 0xDBU}, {{"packet", gr::Size_t{8}}}), recordOf({0x33U})};

        // the same records come out whatever the window is, because a record is framed whole or not at all
        std::vector<std::vector<std::uint8_t>> reference;
        for (const std::size_t chunk : {1UZ, 2UZ, 0UZ}) {
            DelimiterFramer block  = make<DelimiterFramer>(slipFraming(64U));
            const Framed    framed = feed(block, std::span<const Record>(records), chunk);
            expect(eq(framed.out.size(), records.size())) << std::format("chunk {}", chunk);
            std::vector<std::vector<std::uint8_t>> produced;
            for (const Record& record : framed.out) {
                produced.push_back(record.signal_values);
            }
            if (reference.empty()) {
                reference = produced;
            } else {
                expect(that % (produced == reference)) << std::format("chunk {}", chunk);
            }
        }

        // an unconnected output publishes nothing and still consumes, which is the Async contract stated explicitly
        DelimiterFramer silent = make<DelimiterFramer>(slipFraming(64U));
        const Framed    none   = feed(silent, std::span<const Record>(records), 0UZ, false);
        expect(eq(none.out.size(), 0UZ));
        expect(eq(none.consumed, records.size()));
        expect(eq(silent.nRecords, 3ULL)) << "the frames were built and counted; only the publish was skipped";

        // room for fewer records than arrived: the rest stay in the buffer for the next call
        DelimiterFramer     narrow = make<DelimiterFramer>(slipFraming(64U));
        std::vector<Record> scratch(2UZ);
        InputSpan<Record>   inSpan(std::span<const Record>(records), 0UZ);
        OutputSpan<Record>  outSpan(std::span<Record>(scratch), 0UZ, nullptr, true);
        std::ignore = narrow.processBulk(inSpan, outSpan);
        expect(eq(outSpan.count, 2UZ));
        expect(eq(inSpan.consumed, 2UZ)) << "only the records the port had room for were consumed";
    };

    "without its two required settings the block is inert if driven"_test = [] {
        const std::vector<Record> records{recordOf({0x11U, 0x22U})};
        DelimiterFramer           block = make<DelimiterFramer>({{"max_payload_items", gr::Size_t{16}}});
        std::vector<Record>       scratch(4UZ);
        InputSpan<Record>         inSpan(std::span<const Record>(records), 0UZ);
        OutputSpan<Record>        outSpan(std::span<Record>(scratch), 0UZ, nullptr, true);
        const gr::work::Status    status = block.processBulk(inSpan, outSpan);
        expect(status == gr::work::Status::ERROR);
        expect(eq(inSpan.consumed, 0UZ));
        expect(eq(outSpan.count, 0UZ));
    };

    "an idle framer answers that it waits for input, not progress"_test = [] {
        DelimiterFramer block = make<DelimiterFramer>(hdlcFraming(1024U));
        block.start();
        std::vector<Record> room(2UZ);
        InputSpan<Record>   inSpan(std::span<const Record>{}, 0UZ);
        OutputSpan<Record>  outSpan(std::span<Record>(room), 0UZ, nullptr, true);
        expect(block.processBulk(inSpan, outSpan) == gr::work::Status::INSUFFICIENT_INPUT_ITEMS);
    };
};

int main() { /* not needed for UT */ }
