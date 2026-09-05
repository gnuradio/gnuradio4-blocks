#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <format>
#include <limits>
#include <numeric>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/algorithm/digital/HeaderFormat.hpp>
#include <gnuradio-4.0/digital/AccessCodeCorrelator.hpp>
#include <gnuradio-4.0/digital/PacketFramer.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::digital::AccessCodeCorrelator;
using gr::blocks::digital::PacketFramer;
using Packet = gr::DataSet<std::uint8_t>;

constexpr std::uint64_t kSyncWord = 0xACDDA4E2F28C20FCULL;
constexpr std::size_t   kSyncBits = 64UZ;

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

void appendSync(std::vector<std::uint8_t>& stream) {
    for (std::size_t i = 0UZ; i < kSyncBits; ++i) {
        stream.push_back(static_cast<std::uint8_t>((kSyncWord >> (kSyncBits - 1UZ - i)) & 1ULL));
    }
}

/// One framed packet on the wire: sync word, header, payload. Returns the payload it wrote.
[[nodiscard]] std::vector<std::uint8_t> appendPacket(std::vector<std::uint8_t>& stream, const gr::digital::HeaderFormat& format, std::size_t payloadItems, std::uint64_t packetNumber, Rng& rng) {
    appendSync(stream);

    const std::size_t                                      headerItems = gr::digital::headerItemsOf(format);
    std::array<std::uint8_t, gr::digital::kMaxHeaderItems> header{};
    const bool                                             written = gr::digital::formatHeader<std::uint8_t>(format, payloadItems, gr::property_map{{"packet_number", packetNumber}}, std::span<std::uint8_t>(header.data(), headerItems));
    if (!written) {
        throw gr::exception(std::format("the format refused a payload of {} items", payloadItems));
    }
    stream.insert(stream.end(), header.begin(), std::next(header.begin(), static_cast<std::ptrdiff_t>(headerItems)));

    std::vector<std::uint8_t> payload(payloadItems);
    for (std::uint8_t& item : payload) {
        item = static_cast<std::uint8_t>(rng.next() & 1ULL);
    }
    stream.insert(stream.end(), payload.begin(), payload.end());
    return payload;
}

/// Detector then framer, at the given feed granularity.
struct Framed {
    std::vector<Packet> packets{};
    std::size_t         offered  = 0UZ;
    std::size_t         consumed = 0UZ;
};

[[nodiscard]] Framed frame(std::span<const std::uint8_t> stream, const gr::property_map& framerSettings, std::size_t feed = 0UZ, std::span<const gr::Tag> extraTags = {}) {
    AccessCodeCorrelator<std::uint8_t> detector = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", codeString(kSyncWord, kSyncBits)}});
    const auto                         tagged   = gr::blocks::testing::span::run<std::uint8_t>(detector, stream, 4096UZ);

    std::vector<gr::Tag> tags = tagged.tags;
    tags.insert(tags.end(), extraTags.begin(), extraTags.end());
    std::ranges::sort(tags, {}, &gr::Tag::index);

    PacketFramer<std::uint8_t> framer = make<PacketFramer<std::uint8_t>>(framerSettings);
    const auto                 seen   = gr::blocks::testing::span::runVariable<Packet>(framer, std::span<const std::uint8_t>(tagged.samples), feed, 64UZ, std::span<const gr::Tag>(tags));

    return {seen.samples, tagged.samples.size(), seen.consumed};
}

/// @brief One length-header configuration, in both the routes a chain configures it by: the kernel type the test
/// builds by hand from the layout, and the setting strings the block builds its own from.
struct LengthRow {
    const char*              name;
    const char*              covers;
    gr::digital::CountCovers coversValue;
    std::size_t              lengthBits;
    gr::Size_t               unitItems;
    gr::Size_t               checkItems;
    gr::Size_t               prefixItems;
    const char*              byteOrder = "big";
    const char*              flagsAt   = "high";
};

[[nodiscard]] gr::digital::LengthCount countOf(const LengthRow& row) { return gr::digital::LengthCount{row.coversValue, static_cast<std::size_t>(row.unitItems), static_cast<std::size_t>(row.checkItems), static_cast<std::size_t>(row.prefixItems)}; }

[[nodiscard]] gr::digital::HeaderFormat formatOf(const LengthRow& row) {
    if (std::string_view(row.name) == "length_plain") {
        const auto order = std::string_view(row.byteOrder) == "little" ? gr::digital::HeaderByteOrder::Little : gr::digital::HeaderByteOrder::Big;
        return gr::digital::HeaderFormat{gr::digital::LengthPlainHeader{row.lengthBits, order, countOf(row)}};
    }
    const auto flags = std::string_view(row.flagsAt) == "low" ? gr::digital::FlagsPosition::Low : gr::digital::FlagsPosition::High;
    return gr::digital::HeaderFormat{gr::digital::LengthGolay24Header{row.lengthBits, flags, countOf(row)}};
}

[[nodiscard]] gr::property_map settingsOf(const LengthRow& row, gr::Size_t maxPayload) {
    gr::property_map settings{{"header_format", std::string(row.name)}, {"max_payload_items", maxPayload}, //
        {"length_bits", static_cast<gr::Size_t>(row.lengthBits)}, {"count_covers", std::string(row.covers)}, {"count_unit_items", row.unitItems}, {"check_items", row.checkItems}, {"frame_prefix_items", row.prefixItems}};
    if (std::string_view(row.name) == "length_plain") {
        settings.insert_or_assign("byte_order", std::string(row.byteOrder));
    } else {
        settings.insert_or_assign("flags_position", std::string(row.flagsAt));
    }
    return settings;
}

/// @brief One framed packet whose header carries @p extra rather than a packet number, with @p flips bits of the
/// header inverted after it is written. Returns the payload it wrote.
[[nodiscard]] std::vector<std::uint8_t> appendFramed(std::vector<std::uint8_t>& stream, const gr::digital::HeaderFormat& format, std::size_t payloadItems, const gr::property_map& extra, Rng& rng, std::span<const std::size_t> flips = {}) {
    appendSync(stream);

    const std::size_t                                      headerItems = gr::digital::headerItemsOf(format);
    std::array<std::uint8_t, gr::digital::kMaxHeaderItems> header{};
    if (!gr::digital::formatHeader<std::uint8_t>(format, payloadItems, extra, std::span<std::uint8_t>(header.data(), headerItems))) {
        throw gr::exception(std::format("the format refused a payload of {} items", payloadItems));
    }
    for (const std::size_t at : flips) {
        header[at % headerItems] ^= std::uint8_t{1};
    }
    stream.insert(stream.end(), header.begin(), std::next(header.begin(), static_cast<std::ptrdiff_t>(headerItems)));

    std::vector<std::uint8_t> payload(payloadItems);
    for (std::uint8_t& item : payload) {
        item = static_cast<std::uint8_t>(rng.next() & 1ULL);
    }
    stream.insert(stream.end(), payload.begin(), payload.end());
    return payload;
}

[[nodiscard]] std::uint64_t metaU64(const Packet& packet, std::string_view key) {
    if (packet.meta_information.empty()) {
        return ~0ULL;
    }
    const auto entry = packet.meta_information.front().find(gr::property_map::key_type(key));
    return entry == packet.meta_information.front().end() ? ~0ULL : entry->second.value_or(std::uint64_t{~0ULL});
}

/// `sync_errors` is a gr::Size_t, being a Hamming distance rather than a stream offset.
[[nodiscard]] gr::Size_t metaSize(const Packet& packet, std::string_view key) {
    if (packet.meta_information.empty()) {
        return std::numeric_limits<gr::Size_t>::max();
    }
    const auto entry = packet.meta_information.front().find(gr::property_map::key_type(key));
    return entry == packet.meta_information.front().end() ? std::numeric_limits<gr::Size_t>::max() : entry->second.value_or(std::numeric_limits<gr::Size_t>::max());
}

[[nodiscard]] std::string metaString(const Packet& packet, std::string_view key) {
    if (packet.meta_information.empty()) {
        return {};
    }
    const auto entry = packet.meta_information.front().find(gr::property_map::key_type(key));
    return entry == packet.meta_information.front().end() ? std::string{} : std::string(entry->second.value_or(std::string_view{}));
}

[[nodiscard]] bool metaBool(const Packet& packet, std::string_view key) {
    if (packet.meta_information.empty()) {
        return false;
    }
    const auto entry = packet.meta_information.front().find(gr::property_map::key_type(key));
    return entry != packet.meta_information.front().end() && entry->second.value_or(false);
}

[[nodiscard]] const Packet& at(const std::vector<Packet>& packets, std::size_t index) {
    if (index >= packets.size()) {
        throw gr::exception(std::format("expected more than {} packets, got {}", index, packets.size()));
    }
    return packets[index];
}

} // namespace

const boost::ut::suite<"packet framer"> packetFramerTests = [] {
    using namespace boost::ut;

    "every header format round trips a thousand random lengths"_test = [] {
        struct Row {
            const char*               name;
            gr::digital::HeaderFormat format;
            std::size_t               longest;
        };
        const Row kRows[] = {{"length_crc", gr::digital::HeaderFormat{gr::digital::LengthCrcHeader{}}, 4095UZ}, //
            {"length_repeated", gr::digital::HeaderFormat{gr::digital::LengthRepeatedHeader{}}, 4095UZ}};

        for (const Row& row : kRows) {
            Rng                                    rng{};
            std::vector<std::uint8_t>              stream;
            std::vector<std::vector<std::uint8_t>> sent;
            std::vector<std::uint64_t>             numbers;
            for (std::size_t which = 0UZ; which < 1000UZ; ++which) {
                const std::size_t   length = 1UZ + rng.below(row.longest);
                const std::uint64_t number = which % 4096UZ;
                sent.push_back(appendPacket(stream, row.format, length, number, rng));
                numbers.push_back(number);
            }

            const Framed framed = frame(std::span<const std::uint8_t>(stream), {{"header_format", std::string(row.name)}, {"max_payload_items", 4095U}});
            expect(eq(framed.packets.size(), sent.size())) << row.name;

            for (std::size_t which = 0UZ; which < std::min(framed.packets.size(), sent.size()); ++which) {
                const Packet& packet = at(framed.packets, which);
                expect(eq(packet.signal_values.size(), sent[which].size())) << std::format("{}: packet {} length", row.name, which);
                expect(that % (packet.signal_values == sent[which])) << std::format("{}: packet {} bytes", row.name, which);
                expect(eq(static_cast<std::size_t>(packet.extents.at(0)), sent[which].size()));
                expect(that % (packet.signal_names == std::vector<std::string>{"payload"}));
                expect(eq(metaString(packet, "trigger_name"), std::string("access_code")));
                expect(eq(metaSize(packet, "sync_errors"), gr::Size_t{0}));
                expect(metaBool(packet, "header_ok"));
                if (std::string_view(row.name) == "length_crc") {
                    expect(eq(metaU64(packet, "packet_number"), numbers[which])) << std::format("packet {} number", which);
                }
            }
        }
    };

    "sample_start is the absolute offset of the first payload item"_test = [] {
        Rng                             rng{};
        std::vector<std::uint8_t>       stream;
        std::vector<std::size_t>        starts;
        const gr::digital::HeaderFormat format{gr::digital::LengthCrcHeader{}};
        const std::size_t               headerItems = gr::digital::headerItemsOf(format);
        for (std::size_t which = 0UZ; which < 20UZ; ++which) {
            starts.push_back(stream.size() + kSyncBits + headerItems);
            std::ignore = appendPacket(stream, format, 1UZ + 37UZ * which, which, rng);
        }

        const Framed framed = frame(std::span<const std::uint8_t>(stream), {{"max_payload_items", 4095U}});
        expect(eq(framed.packets.size(), starts.size()));
        for (std::size_t which = 0UZ; which < std::min(framed.packets.size(), starts.size()); ++which) {
            expect(eq(metaU64(at(framed.packets, which), "sample_start"), static_cast<std::uint64_t>(starts[which]))) << std::format("packet {}", which);
        }
    };

    "a fixed-length format needs no header on the wire"_test = [] {
        constexpr std::size_t                  kLength = 96UZ;
        Rng                                    rng{};
        std::vector<std::uint8_t>              stream;
        std::vector<std::vector<std::uint8_t>> sent;
        const gr::digital::HeaderFormat        format{gr::digital::FixedLengthHeader{kLength}};
        for (std::size_t which = 0UZ; which < 12UZ; ++which) {
            sent.push_back(appendPacket(stream, format, kLength, 0ULL, rng));
        }

        const Framed framed = frame(std::span<const std::uint8_t>(stream), {{"header_format", std::string("fixed_length")}, {"fixed_payload_items", static_cast<gr::Size_t>(kLength)}, {"max_payload_items", 1024U}});
        expect(eq(framed.packets.size(), sent.size()));
        for (std::size_t which = 0UZ; which < std::min(framed.packets.size(), sent.size()); ++which) {
            expect(that % (at(framed.packets, which).signal_values == sent[which])) << std::format("packet {}", which);
        }
    };

    "the packets do not change with the feed granularity, and no tag is duplicated"_test = [] {
        Rng                                    rng{};
        std::vector<std::uint8_t>              stream;
        std::vector<std::vector<std::uint8_t>> sent;
        const gr::digital::HeaderFormat        format{gr::digital::LengthCrcHeader{}};
        for (std::size_t which = 0UZ; which < 8UZ; ++which) {
            sent.push_back(appendPacket(stream, format, 50UZ + 30UZ * which, which, rng));
        }

        // one marker tag inside every payload, at a position that a small feed must straddle
        const std::size_t    headerItems = gr::digital::headerItemsOf(format);
        std::vector<gr::Tag> markers;
        std::size_t          cursor = 0UZ;
        for (std::size_t which = 0UZ; which < sent.size(); ++which) {
            const std::size_t payloadStart = cursor + kSyncBits + headerItems;
            markers.emplace_back(payloadStart + 7UZ, gr::property_map{{"marker", static_cast<gr::Size_t>(which)}});
            cursor = payloadStart + sent[which].size();
        }

        const gr::property_map settings{{"max_payload_items", 4095U}};
        const Framed           whole = frame(std::span<const std::uint8_t>(stream), settings, 0UZ, std::span<const gr::Tag>(markers));
        expect(eq(whole.packets.size(), sent.size()));

        for (const std::size_t feed : {1UZ, 3UZ, 17UZ, 4096UZ}) {
            const Framed framed = frame(std::span<const std::uint8_t>(stream), settings, feed, std::span<const gr::Tag>(markers));
            expect(eq(framed.packets.size(), sent.size())) << std::format("feed {}", feed);

            for (std::size_t which = 0UZ; which < std::min(framed.packets.size(), sent.size()); ++which) {
                const Packet& packet = at(framed.packets, which);
                expect(that % (packet.signal_values == sent[which])) << std::format("feed {}, packet {}", feed, which);

                std::size_t markerCount = 0UZ;
                for (const auto& [index, map] : packet.timing_events.at(0)) {
                    if (map.contains(gr::property_map::key_type("marker"))) {
                        ++markerCount;
                        expect(eq(index, static_cast<std::ptrdiff_t>(7))) << std::format("feed {}, packet {}: relative to the first payload item", feed, which);
                    }
                }
                expect(eq(markerCount, 1UZ)) << std::format("feed {}, packet {}: exactly once, including across a call boundary", feed, which);
            }
        }
    };

    "a hostile header costs one item and does not wedge"_test = [] {
        const gr::digital::HeaderFormat format{gr::digital::LengthCrcHeader{}};
        const std::size_t               headerItems = gr::digital::headerItemsOf(format);
        Rng                             rng{};

        struct Case {
            const char* what;
            std::size_t claimed;
        };
        const Case kCases[] = {{"a length of zero", 0UZ}, {"a length above max_payload_items", 4000UZ}};

        for (const Case& hostile : kCases) {
            std::vector<std::uint8_t> stream;
            appendSync(stream);
            std::array<std::uint8_t, gr::digital::kMaxHeaderItems> header{};
            expect(gr::digital::formatHeader<std::uint8_t>(format, hostile.claimed, {}, std::span<std::uint8_t>(header.data(), headerItems)));
            stream.insert(stream.end(), header.begin(), std::next(header.begin(), static_cast<std::ptrdiff_t>(headerItems)));
            const std::vector<std::uint8_t> genuine = appendPacket(stream, format, 64UZ, 3ULL, rng);

            const Framed framed = frame(std::span<const std::uint8_t>(stream), {{"max_payload_items", 256U}});
            expect(eq(framed.packets.size(), 1UZ)) << std::format("{}: the packet after it is recovered intact", hostile.what);
            if (!framed.packets.empty()) {
                expect(that % (at(framed.packets, 0UZ).signal_values == genuine)) << hostile.what;
            }
        }

        // ten thousand corrupted headers, and the block is still producing packets at the end
        std::vector<std::uint8_t> stream;
        for (std::size_t which = 0UZ; which < 10000UZ; ++which) {
            appendSync(stream);
            for (std::size_t k = 0UZ; k < headerItems; ++k) {
                stream.push_back(static_cast<std::uint8_t>(rng.next() & 1ULL));
            }
        }
        const std::vector<std::uint8_t> survivor = appendPacket(stream, format, 128UZ, 9ULL, rng);
        const Framed                    framed   = frame(std::span<const std::uint8_t>(stream), {{"max_payload_items", 256U}});
        expect(ge(framed.packets.size(), 1UZ)) << "still producing after ten thousand corrupted headers";
        expect(that % (framed.packets.back().signal_values == survivor)) << "and the genuine packet at the end is intact";
    };

    "the block consumes exactly what it used, one item per call"_test = [] {
        Rng                             rng{};
        std::vector<std::uint8_t>       stream;
        const gr::digital::HeaderFormat format{gr::digital::LengthCrcHeader{}};
        while (stream.size() < 100000UZ) {
            std::ignore = appendPacket(stream, format, 200UZ, 1ULL, rng);
        }

        const Framed framed = frame(std::span<const std::uint8_t>(stream), {{"max_payload_items", 4095U}}, 1UZ);
        expect(ge(framed.packets.size(), 100UZ)) << "detections throughout";
        expect(le(framed.consumed, framed.offered)) << "never more than was offered";
        expect(ge(framed.consumed + 400UZ, framed.offered)) << std::format("consumed {} of {} offered, the remainder being one partial packet", framed.consumed, framed.offered);
    };

    "trigger_to_header moves the header away from the tag"_test = [] {
        constexpr std::size_t           kGap = 24UZ;
        const gr::digital::HeaderFormat format{gr::digital::LengthCrcHeader{}};
        const std::size_t               headerItems = gr::digital::headerItemsOf(format);
        Rng                             rng{};

        std::vector<std::uint8_t> stream;
        appendSync(stream);
        for (std::size_t k = 0UZ; k < kGap; ++k) {
            stream.push_back(0U); // a guard region the detector's tag points at and the framer must skip
        }
        std::array<std::uint8_t, gr::digital::kMaxHeaderItems> header{};
        expect(gr::digital::formatHeader<std::uint8_t>(format, 40UZ, {}, std::span<std::uint8_t>(header.data(), headerItems)));
        stream.insert(stream.end(), header.begin(), std::next(header.begin(), static_cast<std::ptrdiff_t>(headerItems)));
        std::vector<std::uint8_t> payload(40UZ);
        for (std::uint8_t& item : payload) {
            item = static_cast<std::uint8_t>(rng.next() & 1ULL);
        }
        stream.insert(stream.end(), payload.begin(), payload.end());

        const Framed aligned = frame(std::span<const std::uint8_t>(stream), {{"max_payload_items", 256U}, {"trigger_to_header", static_cast<gr::Size_t>(kGap)}});
        expect(eq(aligned.packets.size(), 1UZ));
        if (!aligned.packets.empty()) {
            expect(that % (at(aligned.packets, 0UZ).signal_values == payload));
        }

        const Framed misaligned = frame(std::span<const std::uint8_t>(stream), {{"max_payload_items", 256U}});
        expect(eq(misaligned.packets.size(), 0UZ)) << "without the offset the header is read out of the guard region and rejected";
    };

    "only a matching trigger_name starts a packet"_test = [] {
        Rng                             rng{};
        std::vector<std::uint8_t>       stream;
        const gr::digital::HeaderFormat format{gr::digital::LengthCrcHeader{}};
        const std::vector<std::uint8_t> payload = appendPacket(stream, format, 64UZ, 0ULL, rng);

        expect(eq(frame(std::span<const std::uint8_t>(stream), {{"max_payload_items", 256U}, {"trigger", std::string("access_code")}}).packets.size(), 1UZ));
        expect(eq(frame(std::span<const std::uint8_t>(stream), {{"max_payload_items", 256U}, {"trigger", std::string("preamble")}}).packets.size(), 0UZ)) << "a name that does not match starts nothing";
        expect(eq(frame(std::span<const std::uint8_t>(stream), {{"max_payload_items", 256U}}).packets.size(), 1UZ)) << "an empty trigger accepts any trigger tag";
        std::ignore = payload;
    };

    "degenerate parameters throw"_test = [] {
        expect(throws([] {
            PacketFramer<std::uint8_t> block{}; // no settings at all, so nothing is staged and nothing is validated
            block.start();
        })) << "max_payload_items is required and has no default: a block without it refuses to start";
        expect(throws([] {
            PacketFramer<std::uint8_t> block(gr::property_map{{"max_payload_items", 0U}}); // 0 is the default, so the batch moves nothing and there is no callback to refuse in
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
            block.start();
        })) << "and stating the unset value explicitly leaves it unset";
        expect(throws([] { std::ignore = make<PacketFramer<std::uint8_t>>({{"max_payload_items", 256U}, {"header_format", std::string("counter")}}); }));
        expect(throws([] { std::ignore = make<PacketFramer<std::uint8_t>>({{"max_payload_items", 256U}, {"header_format", std::string("fixed_length")}}); })) << "a fixed length of zero is not a length";
        expect(throws([] { std::ignore = make<PacketFramer<std::uint8_t>>({{"max_payload_items", 256U}, {"header_format", std::string("fixed_length")}, {"fixed_payload_items", 300U}}); })) << "and it must fit the maximum";
        expect(nothrow([] { std::ignore = make<PacketFramer<std::uint8_t>>({{"max_payload_items", 256U}}); }));
        expect(nothrow([] { std::ignore = make<PacketFramer<std::uint8_t>>({{"max_payload_items", 256U}, {"header_format", std::string("length_repeated")}}); }));
    };
};

const boost::ut::suite<"length headers"> lengthHeaderTests = [] {
    using namespace boost::ut;

    "the three count conventions all frame the worked example's 272 items"_test = [] {
        // one 34-byte payload region whose last two bytes are a CRC-16, behind a 16-bit length field: the three
        // conventions write 32, 34 and 36 and every one of them has to extract the same 272 items
        constexpr std::size_t kPayloadItems = 272UZ;
        const LengthRow       kRows[]{{"length_plain", "payload", gr::digital::CountCovers::Payload, 16UZ, 8U, 16U, 0U}, //
                  {"length_plain", "payload_and_check", gr::digital::CountCovers::PayloadAndCheck, 16UZ, 8U, 0U, 0U},    //
                  {"length_plain", "whole_frame", gr::digital::CountCovers::WholeFrame, 16UZ, 8U, 0U, 16U}};
        const std::uint32_t   kWritten[]{32U, 34U, 36U};

        for (std::size_t which = 0UZ; which < std::size(kRows); ++which) {
            const gr::digital::HeaderFormat format = formatOf(kRows[which]);
            std::array<std::uint8_t, 32UZ>  header{};
            expect(gr::digital::formatHeader<std::uint8_t>(format, kPayloadItems, {}, std::span<std::uint8_t>(header.data(), 16UZ))) << kRows[which].covers;
            expect(eq(gr::digital::readHeaderField(std::span<const std::uint8_t>(header.data(), 16UZ), 0UZ, 16UZ), kWritten[which])) << std::format("{} writes the field value {}", kRows[which].covers, kWritten[which]);

            Rng                             rng{};
            std::vector<std::uint8_t>       stream;
            const std::vector<std::uint8_t> sent   = appendFramed(stream, format, kPayloadItems, {}, rng);
            const Framed                    framed = frame(std::span<const std::uint8_t>(stream), settingsOf(kRows[which], 4096U));
            expect(eq(framed.packets.size(), 1UZ)) << kRows[which].covers;
            if (!framed.packets.empty()) {
                expect(eq(at(framed.packets, 0UZ).signal_values.size(), kPayloadItems)) << kRows[which].covers;
                expect(that % (at(framed.packets, 0UZ).signal_values == sent)) << kRows[which].covers;
            }
        }

        // and the wrong setting on the same wire bits is wrong by arithmetic rather than merely warned about
        std::array<std::uint8_t, 32UZ> header{};
        expect(gr::digital::formatHeader<std::uint8_t>(formatOf(kRows[0UZ]), kPayloadItems, {}, std::span<std::uint8_t>(header.data(), 16UZ)));
        const auto asCheck = gr::digital::parseHeader<std::uint8_t>(formatOf(kRows[1UZ]), std::span<const std::uint8_t>(header.data(), 16UZ));
        expect(asCheck.has_value() && asCheck->payloadItems == 256UZ) << "a payload frame read as payload_and_check hands the checker a record two bytes short";

        expect(gr::digital::formatHeader<std::uint8_t>(formatOf(kRows[2UZ]), kPayloadItems, {}, std::span<std::uint8_t>(header.data(), 16UZ)));
        const auto asPayload = gr::digital::parseHeader<std::uint8_t>(formatOf(kRows[0UZ]), std::span<const std::uint8_t>(header.data(), 16UZ));
        expect(asPayload.has_value() && asPayload->payloadItems == 304UZ) << "and a whole_frame frame read as payload over-runs into the next one";
    };

    "both realizations round trip random lengths under every convention"_test = [] {
        const LengthRow kRows[]{{"length_plain", "payload", gr::digital::CountCovers::Payload, 16UZ, 8U, 16U, 0U},        //
            {"length_plain", "payload_and_check", gr::digital::CountCovers::PayloadAndCheck, 16UZ, 8U, 0U, 0U},           //
            {"length_plain", "whole_frame", gr::digital::CountCovers::WholeFrame, 16UZ, 8U, 0U, 0U},                      //
            {"length_plain", "payload_and_check", gr::digital::CountCovers::PayloadAndCheck, 16UZ, 1U, 0U, 0U, "little"}, //
            {"length_golay24", "payload", gr::digital::CountCovers::Payload, 8UZ, 8U, 16U, 0U, "big", "high"},            //
            {"length_golay24", "payload_and_check", gr::digital::CountCovers::PayloadAndCheck, 8UZ, 8U, 0U, 0U, "big", "low"}};

        for (const LengthRow& row : kRows) {
            const gr::digital::HeaderFormat        format = formatOf(row);
            const std::size_t                      unit   = static_cast<std::size_t>(row.unitItems);
            Rng                                    rng{};
            std::vector<std::uint8_t>              stream;
            std::vector<std::vector<std::uint8_t>> sent;
            std::vector<std::uint64_t>             flags;
            for (std::size_t which = 0UZ; which < 200UZ; ++which) {
                // the item count has to be a whole number of counted units above the check field, which is what a
                // length field can express at all
                const std::size_t      payloadItems = static_cast<std::size_t>(row.checkItems) + unit * (1UZ + rng.below(60UZ));
                const std::uint64_t    flagValue    = std::string_view(row.name) == "length_golay24" ? which % 16UZ : 0ULL;
                const gr::property_map extra        = std::string_view(row.name) == "length_golay24" ? gr::property_map{{"header_flags", flagValue}} : gr::property_map{};
                sent.push_back(appendFramed(stream, format, payloadItems, extra, rng));
                flags.push_back(flagValue);
            }

            const Framed framed = frame(std::span<const std::uint8_t>(stream), settingsOf(row, 4096U));
            expect(eq(framed.packets.size(), sent.size())) << std::format("{} / {}", row.name, row.covers);
            for (std::size_t which = 0UZ; which < std::min(framed.packets.size(), sent.size()); ++which) {
                const Packet& packet = at(framed.packets, which);
                expect(that % (packet.signal_values == sent[which])) << std::format("{} / {}: packet {}", row.name, row.covers, which);
                if (std::string_view(row.name) == "length_golay24") {
                    expect(eq(metaU64(packet, "header_corrected_errors"), 0ULL)) << std::format("{}: an uncorrupted header corrects nothing", row.covers);
                    expect(eq(metaU64(packet, "header_flags"), flags[which])) << std::format("{}: packet {} flags", row.covers, which);
                } else {
                    expect(eq(metaU64(packet, "header_corrected_errors"), ~0ULL)) << "a bare length field carries no correction count";
                    expect(eq(metaU64(packet, "header_flags"), ~0ULL)) << "and no flags";
                }
            }
        }
    };

    "the Golay header corrects three errors and counts them, and refuses a fourth"_test = [] {
        const LengthRow                 row{"length_golay24", "payload_and_check", gr::digital::CountCovers::PayloadAndCheck, 8UZ, 8U, 0U, 0U, "big", "high"};
        const gr::digital::HeaderFormat format = formatOf(row);

        for (std::size_t errors = 0UZ; errors <= 3UZ; ++errors) {
            Rng                       rng{};
            std::vector<std::uint8_t> stream;
            std::vector<std::size_t>  flips;
            for (std::size_t k = 0UZ; k < errors; ++k) {
                flips.push_back(3UZ + 7UZ * k); // three distinct positions inside the 24-item codeword
            }
            const std::vector<std::uint8_t> sent   = appendFramed(stream, format, 64UZ, gr::property_map{{"header_flags", std::uint64_t{5}}}, rng, std::span<const std::size_t>(flips));
            const Framed                    framed = frame(std::span<const std::uint8_t>(stream), settingsOf(row, 4096U));

            expect(eq(framed.packets.size(), 1UZ)) << std::format("{} bit errors are within the code's correcting radius", errors);
            if (!framed.packets.empty()) {
                const Packet& packet = at(framed.packets, 0UZ);
                expect(that % (packet.signal_values == sent)) << std::format("{} bit errors: the payload is intact", errors);
                expect(eq(metaU64(packet, "header_corrected_errors"), static_cast<std::uint64_t>(errors))) << std::format("{} bit errors are counted as {}", errors, errors);
                expect(eq(metaU64(packet, "header_flags"), std::uint64_t{5})) << "and the flags survive the correction";
            }
        }

        // four is beyond the radius: the code detects it, the header is refused, one item is consumed, and the genuine
        // packet behind it is still recovered
        Rng                            rng{};
        std::vector<std::uint8_t>      stream;
        const std::vector<std::size_t> four{3UZ, 10UZ, 17UZ, 21UZ};
        std::ignore                             = appendFramed(stream, format, 64UZ, {}, rng, std::span<const std::size_t>(four));
        const std::vector<std::uint8_t> genuine = appendFramed(stream, format, 96UZ, {}, rng);

        const Framed framed = frame(std::span<const std::uint8_t>(stream), settingsOf(row, 4096U));
        expect(eq(framed.packets.size(), 1UZ)) << "a header carrying four errors is refused rather than framing garbage";
        if (!framed.packets.empty()) {
            expect(that % (at(framed.packets, 0UZ).signal_values == genuine)) << "and the packet after it is recovered intact";
        }
    };

    "an over-long claim is refused and the genuine packet behind it survives"_test = [] {
        const LengthRow                 row{"length_plain", "payload_and_check", gr::digital::CountCovers::PayloadAndCheck, 16UZ, 8U, 0U, 0U};
        const gr::digital::HeaderFormat format = formatOf(row);

        Rng                       rng{};
        std::vector<std::uint8_t> stream;
        appendSync(stream);
        std::array<std::uint8_t, 32UZ> header{};
        expect(gr::digital::formatHeader<std::uint8_t>(format, 4000UZ, {}, std::span<std::uint8_t>(header.data(), 16UZ))) << "the layout can express it; the graph's bound is what refuses it";
        stream.insert(stream.end(), header.begin(), std::next(header.begin(), 16));
        const std::vector<std::uint8_t> genuine = appendFramed(stream, format, 64UZ, {}, rng);

        const Framed framed = frame(std::span<const std::uint8_t>(stream), settingsOf(row, 256U));
        expect(eq(framed.packets.size(), 1UZ)) << "a claim above max_payload_items costs one item, not the stream";
        if (!framed.packets.empty()) {
            expect(that % (at(framed.packets, 0UZ).signal_values == genuine));
        }

        // a zero-length claim is the format's own refusal as well as the framer's, and composes with it the same way
        std::vector<std::uint8_t> zeroStream;
        appendSync(zeroStream);
        expect(gr::digital::formatHeader<std::uint8_t>(format, 0UZ, {}, std::span<std::uint8_t>(header.data(), 16UZ)));
        zeroStream.insert(zeroStream.end(), header.begin(), std::next(header.begin(), 16));
        Rng                             zeroRng{};
        const std::vector<std::uint8_t> after = appendFramed(zeroStream, format, 48UZ, {}, zeroRng);
        const Framed                    zero  = frame(std::span<const std::uint8_t>(zeroStream), settingsOf(row, 256U));
        expect(eq(zero.packets.size(), 1UZ));
        if (!zero.packets.empty()) {
            expect(that % (at(zero.packets, 0UZ).signal_values == after));
        }
    };

    "the new formats do not change with the feed granularity"_test = [] {
        const LengthRow kRows[]{{"length_plain", "payload", gr::digital::CountCovers::Payload, 16UZ, 8U, 16U, 0U}, //
            {"length_golay24", "payload_and_check", gr::digital::CountCovers::PayloadAndCheck, 8UZ, 8U, 0U, 0U, "big", "high"}};

        for (const LengthRow& row : kRows) {
            const gr::digital::HeaderFormat        format = formatOf(row);
            Rng                                    rng{};
            std::vector<std::uint8_t>              stream;
            std::vector<std::vector<std::uint8_t>> sent;
            for (std::size_t which = 0UZ; which < 8UZ; ++which) {
                sent.push_back(appendFramed(stream, format, static_cast<std::size_t>(row.checkItems) + 8UZ * (6UZ + 4UZ * which), {}, rng));
            }

            for (const std::size_t feed : {1UZ, 3UZ, 17UZ, 4096UZ}) {
                const Framed framed = frame(std::span<const std::uint8_t>(stream), settingsOf(row, 4096U), feed);
                expect(eq(framed.packets.size(), sent.size())) << std::format("{}, feed {}", row.name, feed);
                for (std::size_t which = 0UZ; which < std::min(framed.packets.size(), sent.size()); ++which) {
                    expect(that % (at(framed.packets, which).signal_values == sent[which])) << std::format("{}, feed {}, packet {}", row.name, feed, which);
                }
            }
        }
    };

    "every setting a layout does not own is refused rather than ignored"_test = [] {
        const auto refuses = [](gr::property_map settings, std::string_view offender, std::string_view why) {
            std::string message;
            try {
                std::ignore = make<PacketFramer<std::uint8_t>>(std::move(settings));
            } catch (const std::exception& error) {
                message = error.what();
            }
            expect(!message.empty()) << why;
            expect(message.contains(offender)) << std::format("{}: \"{}\" is missing from \"{}\"", why, offender, message);
        };

        const gr::property_map crc{{"max_payload_items", 256U}};
        refuses(gr::property_map{{"max_payload_items", 256U}, {"length_bits", 16U}}, "length_bits", "a width belongs to the length formats");
        refuses(gr::property_map{{"max_payload_items", 256U}, {"count_covers", std::string("payload")}}, "count_covers", "and so does what the field counts");
        refuses(gr::property_map{{"max_payload_items", 256U}, {"count_unit_items", 1U}}, "count_unit_items", "and the unit");
        refuses(gr::property_map{{"max_payload_items", 256U}, {"check_items", 16U}}, "check_items", "and the check field");
        refuses(gr::property_map{{"max_payload_items", 256U}, {"frame_prefix_items", 16U}}, "frame_prefix_items", "and the prefix");
        refuses(gr::property_map{{"max_payload_items", 256U}, {"byte_order", std::string("little")}}, "byte_order", "byte_order belongs to length_plain alone");
        refuses(gr::property_map{{"max_payload_items", 256U}, {"flags_position", std::string("low")}}, "flags_position", "and flags_position to length_golay24 alone");
        refuses(gr::property_map{{"max_payload_items", 256U}, {"fixed_payload_items", 64U}}, "fixed_payload_items", "and a fixed length to fixed_length alone");

        const gr::property_map plain{{"max_payload_items", 256U}, {"header_format", std::string("length_plain")}, {"count_covers", std::string("payload_and_check")}};
        refuses(plain, "length_bits", "length_bits is required by the length formats and has no default");
        refuses(gr::property_map{{"max_payload_items", 256U}, {"header_format", std::string("length_plain")}, {"length_bits", 16U}}, "count_covers", "and count_covers has none either");
        refuses(gr::property_map{{"max_payload_items", 256U}, {"header_format", std::string("length_plain")}, {"length_bits", 16U}, {"count_covers", std::string("everything")}}, "everything", "an unrecognized convention names itself");
        refuses(gr::property_map{{"max_payload_items", 256U}, {"header_format", std::string("length_plain")}, {"length_bits", 33U}, {"count_covers", std::string("payload_and_check")}}, "33", "a field wider than the framer's header buffer");
        refuses(gr::property_map{{"max_payload_items", 256U}, {"header_format", std::string("length_plain")}, {"length_bits", 12U}, {"count_covers", std::string("payload_and_check")}, {"byte_order", std::string("little")}}, "12", "a byte order over a field that is not whole bytes");
        refuses(gr::property_map{{"max_payload_items", 256U}, {"header_format", std::string("length_golay24")}, {"length_bits", 13U}, {"count_covers", std::string("payload_and_check")}}, "13", "a Golay header carries twelve information bits");
        refuses(gr::property_map{{"max_payload_items", 256U}, {"header_format", std::string("length_plain")}, {"length_bits", 16U}, {"count_covers", std::string("payload_and_check")}, {"count_unit_items", 65U}}, "65", "a counted unit above 64 items");
        refuses(gr::property_map{{"max_payload_items", 256U}, {"header_format", std::string("length_plain")}, {"length_bits", 16U}, {"count_covers", std::string("payload_and_check")}, {"check_items", 16U}}, "16", "a check field under a convention that counts it");
        refuses(gr::property_map{{"max_payload_items", 256U}, {"header_format", std::string("length_plain")}, {"length_bits", 16U}, {"count_covers", std::string("payload")}, {"frame_prefix_items", 16U}}, "16", "and a frame prefix under one that does not");

        expect(nothrow([&crc] { std::ignore = make<PacketFramer<std::uint8_t>>(crc); })) << "and a settings vector naming no layout-specific setting leaves the realizations in use alone";
    };

    "kMaxHeaderItems is still 32"_test = [] {
        static_assert(gr::digital::kMaxHeaderItems == 32UZ);
        static_assert(gr::digital::LengthPlainHeader::kMaxLengthBits <= gr::digital::kMaxHeaderItems);
        static_assert(gr::digital::LengthGolay24Header::kCodewordItems <= gr::digital::kMaxHeaderItems);
        expect(true) << "the widened variant does not grow the framer's fixed header array";
    };
};

int main() { /* not needed for UT */ }
