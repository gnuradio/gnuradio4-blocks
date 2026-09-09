/* The synthetic gate for the Mode S message decoder.
 *
 * Every frame here is built field by field from the standard rather than copied from a capture, so a case states
 * what the standard says and not what one recording happened to contain. The parity is the framer's business and this
 * block reads the framer's verdict, so a record carries a `crc_ok` key rather than a computed parity field.
 *
 * The compact position case encodes a chosen latitude and longitude with its own implementation of RTCA
 * DO-260B A.1.7.1 and asks the block to give the position back, which pins the pairing arithmetic against the
 * standard's equations rather than against itself.
 */
#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <memory_resource>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/YamlPmt.hpp>

#include <gnuradio-4.0/adsb/ModeS.hpp>
#include <gnuradio-4.0/basic/DataSetToPacket.hpp>
#include <gnuradio-4.0/basic/PacketToDataSet.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_mode_s_decode {

using gr::blocks::adsb::ModeSDecode;
using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;

using Record = gr::DataSet<std::uint8_t>;

constexpr float kRate = 2.0e6f; ///< the rate the reference capture was taken at, so a sample index reads as a time

/// @brief A long Mode S frame under construction, written by the standard's bit numbers: bit 1 is octet 0's top bit.
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

/// @brief An extended squitter of type code @p typeCode from address @p address, with nothing else written yet.
[[nodiscard]] Frame squitter(std::uint32_t address, std::uint32_t typeCode) {
    Frame frame;
    frame.put(1UZ, 5UZ, 17U);       // DF 17
    frame.put(6UZ, 3UZ, 5U);        // CA, the transponder capability
    frame.put(9UZ, 24UZ, address);  // AA, the announced address
    frame.put(33UZ, 5UZ, typeCode); // the ME field's type code
    return frame;
}

/// @brief The record the framer publishes for @p frame: the octets, and the account it gives of how they were taken.
///
/// `mode_s_format` is deliberately wrong here. The decoder re-asserts it from the octets, so a case that reads
/// the right value back has shown the block reads the frame rather than the key.
[[nodiscard]] Record recordOf(const Frame& frame, std::uint64_t sampleStart, std::optional<bool> crcOk = std::optional<bool>{true}, std::uint64_t sequence = 0ULL) {
    Record record;
    record.signal_values = frame.octets;
    record.extents.push_back(static_cast<std::int32_t>(record.signal_values.size()));
    record.signal_names.emplace_back("payload");
    record.signal_quantities.emplace_back("");
    record.signal_units.emplace_back("");
    record.timing_events.emplace_back();
    record.meta_information.emplace_back();

    gr::property_map& meta = record.meta_information[0UZ];
    meta.insert_or_assign(gr::property_map::key_type("trigger_name"), gr::pmt::Value(std::string("mode_s")));
    meta.insert_or_assign(gr::property_map::key_type("sample_start"), gr::pmt::Value(sampleStart));
    meta.insert_or_assign(gr::property_map::key_type("sequence"), gr::pmt::Value(sequence));
    meta.insert_or_assign(gr::property_map::key_type("sample_rate"), gr::pmt::Value(kRate));
    meta.insert_or_assign(gr::property_map::key_type("mode_s_format"), gr::pmt::Value(gr::Size_t{99}));
    meta.insert_or_assign(gr::property_map::key_type("crc_remainder"), gr::pmt::Value(gr::Size_t{0}));
    meta.insert_or_assign(gr::property_map::key_type("preamble_strong"), gr::pmt::Value(1.5f));
    if (crcOk.has_value()) {
        meta.insert_or_assign(gr::property_map::key_type("crc_ok"), gr::pmt::Value(*crcOk));
    }
    return record;
}

/// @brief What one `processBulk` of a decoder produced and counted.
struct Decoded {
    std::vector<Record> records{};
    std::uint64_t       frames       = 0ULL;
    std::uint64_t       decoded      = 0ULL;
    std::uint64_t       positions    = 0ULL;
    std::uint64_t       pairsExpired = 0ULL;
    std::uint64_t       unchecked    = 0ULL;
    std::uint64_t       evicted      = 0ULL;
};

[[nodiscard]] Decoded decode(std::vector<Record> records, gr::property_map settings = {}) {
    ModeSDecode block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();

    std::vector<Record> room(records.size());
    InputSpan<Record>   inSpan{std::span<const Record>(records)};
    OutputSpan<Record>  outSpan{std::span<Record>(room)};
    std::ignore = block.processBulk(inSpan, outSpan);

    Decoded result;
    result.records.assign(room.begin(), room.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
    result.frames       = block.nFrames;
    result.decoded      = block.nDecoded;
    result.positions    = block.nPositions;
    result.pairsExpired = block.nPairsExpired;
    result.unchecked    = block.nUnchecked;
    result.evicted      = block.nEvicted;
    return result;
}

/// @brief The metadata of the record at @p index, or an empty map where there is none.
[[nodiscard]] const gr::property_map& metaOf(const Decoded& decoded, std::size_t index) {
    static const gr::property_map empty{};
    return index < decoded.records.size() && !decoded.records[index].meta_information.empty() ? decoded.records[index].meta_information[0UZ] : empty;
}

/// @brief The value of @p key read at the exact type @p T, which is how a wrong type reads as absent.
template<typename T>
[[nodiscard]] std::optional<T> read(const gr::property_map& meta, std::string_view key) {
    const auto entry = meta.find(gr::property_map::key_type(key));
    if (entry == meta.end()) {
        return std::nullopt;
    }
    const T* value = entry->second.template get_if<T>();
    return value == nullptr ? std::nullopt : std::optional<T>{*value};
}

/// @brief The same for a string, which a metadata map holds in its own allocator's string type.
[[nodiscard]] std::optional<std::string> readText(const gr::property_map& meta, std::string_view key) {
    const std::optional<std::pmr::string> value = read<std::pmr::string>(meta, key);
    return value.has_value() ? std::optional<std::string>{std::string(*value)} : std::nullopt;
}

[[nodiscard]] std::string textOf(const gr::property_map& meta, std::string_view key) { return readText(meta, key).value_or(std::string("<absent>")); }

[[nodiscard]] bool has(const gr::property_map& meta, std::string_view key) { return meta.contains(gr::property_map::key_type(key)); }

// ─── compact position reporting, encoded here from the standard so the decode is judged against it ────────────────

/// @brief RTCA DO-260B A.1.7.2.2's NL, written out again so the encoder below does not lean on the block's copy.
[[nodiscard]] std::int32_t zones(double degrees) {
    const double from = std::abs(degrees);
    if (from >= 87.0) {
        return from > 87.0 ? 1 : 2;
    }
    const double cosine = std::cos(std::numbers::pi / 180.0 * from);
    return std::min(59, static_cast<std::int32_t>(std::floor(2.0 * std::numbers::pi / std::acos(1.0 - (1.0 - std::cos(std::numbers::pi / 30.0)) / (cosine * cosine)))));
}

[[nodiscard]] double flooredMod(double value, double modulus) { return value - modulus * std::floor(value / modulus); }

/// @brief The seventeen-bit encoded halves of @p latitude and @p longitude in format @p format, RTCA DO-260B A.1.7.1.
[[nodiscard]] std::pair<std::uint32_t, std::uint32_t> encodeCpr(double latitude, double longitude, std::uint32_t format) {
    constexpr double kScale     = 131072.0;
    const auto       index      = static_cast<double>(format);
    const double     latSpacing = 360.0 / (60.0 - index);
    const auto       encodedLat = static_cast<std::uint32_t>(std::floor(kScale * flooredMod(latitude, latSpacing) / latSpacing + 0.5)) % 131072U;

    const double resolvedLat = latSpacing * (std::floor(latitude / latSpacing) + static_cast<double>(encodedLat) / kScale);
    const double lonSpacing  = 360.0 / static_cast<double>(std::max(zones(resolvedLat) - static_cast<std::int32_t>(format), 1));
    const auto   encodedLon  = static_cast<std::uint32_t>(std::floor(kScale * flooredMod(longitude, lonSpacing) / lonSpacing + 0.5)) % 131072U;
    return {encodedLat, encodedLon};
}

/// @brief An airborne position squitter at @p latitude and @p longitude in CPR format @p format, altitude code @p ac12.
[[nodiscard]] Frame positionFrame(std::uint32_t address, double latitude, double longitude, std::uint32_t format, std::uint32_t ac12) {
    const auto [encodedLat, encodedLon] = encodeCpr(latitude, longitude, format);
    Frame frame                         = squitter(address, 11U); // type code 11, an airborne position with barometric altitude
    frame.put(41UZ, 12UZ, ac12);
    frame.put(54UZ, 1UZ, format);
    frame.put(55UZ, 17UZ, encodedLat);
    frame.put(72UZ, 17UZ, encodedLon);
    return frame;
}

// ─── graph-side blocks, for the case that carries a decoded record across the packet boundary ─────────────────────

struct RecordSource : gr::Block<RecordSource> {
    gr::PortOut<Record> out;
    GR_MAKE_REFLECTABLE(RecordSource, out);
    std::vector<Record> _records{};
    std::size_t         _emitted = 0UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t nRecords = std::min(outSpan.size(), _records.size() - _emitted);
        for (std::size_t k = 0UZ; k < nRecords; ++k) {
            outSpan[k] = _records[_emitted + k];
        }
        _emitted += nRecords;
        outSpan.publish(nRecords);
        return _emitted >= _records.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

template<typename TItem>
struct Collector : gr::Block<Collector<TItem>> {
    gr::PortIn<TItem, gr::Async> in;
    GR_MAKE_REFLECTABLE(Collector, in);
    std::vector<TItem> _items{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (std::size_t k = 0UZ; k < inSpan.size(); ++k) {
            _items.push_back(inSpan[k]);
        }
        const std::size_t taken = inSpan.size();
        std::ignore             = inSpan.consume(taken);
        return taken == 0UZ ? gr::work::Status::INSUFFICIENT_INPUT_ITEMS : gr::work::Status::OK;
    }
};

/// @brief Runs a graph to completion under the simple scheduler, stopping it rather than hanging if it wedges.
///
/// @p collect runs while the scheduler still owns the graph, because the references `emplaceBlock` returned point
/// into blocks the scheduler destroys with itself.
template<typename TCollect>
void runGraph(gr::Graph flow, TCollect&& collect) {
    gr::scheduler::Simple<> scheduler;
    boost::ut::expect(scheduler.exchange(std::move(flow)).has_value());
    std::atomic<bool> done{false};
    std::thread       runner([&scheduler, &done] {
        std::ignore = scheduler.runAndWait();
        done        = true;
    });
    const auto        start = std::chrono::steady_clock::now();
    while (!done.load() && std::chrono::steady_clock::now() - start < std::chrono::seconds(30)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!done.load()) {
        scheduler.requestStop();
        boost::ut::expect(false) << "the graph did not finish within thirty seconds";
    }
    runner.join();
    collect();
}

// The keys this block writes, by the type each of them holds, so that one case can carry all of them at once.
constexpr std::array<std::string_view, 6UZ> kStringKeys{"protocol", "mode_s_icao", "adsb_message", "adsb_callsign", "adsb_airspeed_type", "adsb_vertical_rate_source"};
constexpr std::array<std::string_view, 7UZ> kSizeKeys{"mode_s_format", "adsb_type_code", "adsb_emitter_category", "adsb_cpr_format", "adsb_cpr_latitude", "adsb_cpr_longitude", "adsb_velocity_subtype"};
constexpr std::array<std::string_view, 5UZ> kDoubleKeys{"adsb_latitude", "adsb_longitude", "adsb_ground_speed_kt", "adsb_track_deg", "adsb_heading_deg"};
constexpr std::array<std::string_view, 2UZ> kIntegerKeys{"adsb_altitude_ft", "adsb_vertical_rate_fpm"};

/// @brief Every key of @p before found in @p after at its own type, the doubles to @p relative, counting comparisons.
[[nodiscard]] std::size_t sameKeys(const gr::property_map& before, const gr::property_map& after, double relative) {
    using namespace boost::ut;
    std::size_t compared = 0UZ;
    for (const std::string_view key : kStringKeys) {
        if (const auto value = readText(before, key); value.has_value()) {
            expect(eq(textOf(after, key), *value)) << key;
            ++compared;
        }
    }
    for (const std::string_view key : kSizeKeys) {
        if (const auto value = read<gr::Size_t>(before, key); value.has_value()) {
            expect(eq(read<gr::Size_t>(after, key).value_or(std::numeric_limits<gr::Size_t>::max()), *value)) << key;
            ++compared;
        }
    }
    for (const std::string_view key : kDoubleKeys) {
        if (const auto value = read<double>(before, key); value.has_value()) {
            const double got = read<double>(after, key).value_or(-1.0e300);
            expect(std::abs(got - *value) <= relative * std::max(1.0, std::abs(*value))) << key << std::format(": {} against {}", got, *value);
            ++compared;
        }
    }
    for (const std::string_view key : kIntegerKeys) {
        if (const auto value = read<std::int32_t>(before, key); value.has_value()) {
            expect(eq(read<std::int32_t>(after, key).value_or(std::numeric_limits<std::int32_t>::min()), *value)) << key;
            ++compared;
        }
    }
    return compared;
}

} // namespace qa_mode_s_decode

using namespace qa_mode_s_decode;

const boost::ut::suite<"adsb"> modeSTests = [] {
    using namespace boost::ut;
    using gr::blocks::basic::DataSetToPacket;
    using gr::blocks::basic::PacketToDataSet;

    "an identification message names the aircraft, and the format is read from the octets not the key"_test = [] {
        // RTCA DO-260B 2.2.3.2: type code 4, an emitter category and eight six-bit characters spelling `AAL160  `
        constexpr std::array<std::uint32_t, 8UZ> kCharacters{1U, 1U, 12U, 49U, 54U, 48U, 32U, 32U};
        Frame                                    frame = squitter(0xAB0969U, 4U);
        frame.put(38UZ, 3UZ, 3U);
        for (std::size_t i = 0UZ; i < kCharacters.size(); ++i) {
            frame.put(41UZ + 6UZ * i, 6UZ, kCharacters[i]);
        }

        const Decoded           decoded = decode({recordOf(frame, 59'562'000ULL)});
        const gr::property_map& meta    = metaOf(decoded, 0UZ);
        expect(eq(decoded.records.size(), 1UZ));
        expect(eq(decoded.decoded, 1ULL));
        expect(eq(textOf(meta, "protocol"), std::string("mode_s/adsb")));
        expect(eq(read<gr::Size_t>(meta, "mode_s_format").value_or(0U), gr::Size_t{17}));
        expect(eq(textOf(meta, "mode_s_icao"), std::string("AB0969")));
        expect(eq(read<gr::Size_t>(meta, "adsb_type_code").value_or(0U), gr::Size_t{4}));
        expect(eq(textOf(meta, "adsb_message"), std::string("identification")));
        expect(eq(textOf(meta, "adsb_callsign"), std::string("AAL160")));
        expect(eq(read<gr::Size_t>(meta, "adsb_emitter_category").value_or(0U), gr::Size_t{3}));

        // the framer's own facts cross beneath the decode, and the payload is the frame it was given
        expect(eq(read<std::uint64_t>(meta, "sample_start").value_or(0ULL), 59'562'000ULL));
        expect(eq(read<float>(meta, "sample_rate").value_or(0.f), kRate));
        expect(eq(textOf(meta, "trigger_name"), std::string("mode_s")));
        expect(std::ranges::equal(decoded.records[0UZ].signal_values, frame.octets));
    };

    "the twenty-five foot altitude field decodes where its Q bit is set"_test = [] {
        // RTCA DO-260B 2.2.3.2: Q set, the remaining eleven bits are 25 ft increments offset by -1000 ft.
        // 39 000 ft is increment 1600, whose eleven bits sit either side of the Q bit at field position 8.
        constexpr std::uint32_t kIncrements = 1600U;
        constexpr std::uint32_t kAltitude   = ((kIncrements >> 4U) << 5U) | (1U << 4U) | (kIncrements & 0x0FU);

        const Decoded           decoded = decode({recordOf(positionFrame(0xAB0969U, 42.4, -71.3, 0U, kAltitude), 0ULL)});
        const gr::property_map& meta    = metaOf(decoded, 0UZ);
        expect(eq(read<std::int32_t>(meta, "adsb_altitude_ft").value_or(0), 39000));
        expect(read<bool>(meta, "adsb_altitude_q").value_or(false));
        expect(eq(textOf(meta, "adsb_message"), std::string("airborne_position")));
    };

    "the hundred foot code decodes where the Q bit is clear, which transponders in service still send"_test = [] {
        // ICAO Annex 10 Volume IV, the AC code with Q clear. 0x2E2 is C1 C2 C4 = 011 over a 500 ft step of 9: the
        // step is odd, so the C digit counts the other way and 2 reads as 4, giving 500*9 + 400 - 1300 = 3600 ft.
        // 0x0E2 differs in one C pulse alone and is the next 100 ft up.
        constexpr std::array<std::pair<std::uint32_t, std::int32_t>, 2UZ> kCodes{{{0x2E2U, 3600}, {0x0E2U, 3700}}};
        for (const auto& [code, feet] : kCodes) {
            const Decoded           decoded = decode({recordOf(positionFrame(0xAA1BAAU, 42.4, -71.3, 0U, code), 0ULL)});
            const gr::property_map& meta    = metaOf(decoded, 0UZ);
            expect(eq(read<std::int32_t>(meta, "adsb_altitude_ft").value_or(0), feet)) << std::format("altitude code 0x{:03X}", code);
            expect(!read<bool>(meta, "adsb_altitude_q").value_or(true));
        }

        // three of the eight C codes are unassigned; one of them states no altitude, and the record carries the rest
        const Decoded           unassigned = decode({recordOf(positionFrame(0xAA1BAAU, 42.4, -71.3, 0U, 0x062U), 0ULL)});
        const gr::property_map& meta       = metaOf(unassigned, 0UZ);
        expect(!has(meta, "adsb_altitude_ft"));
        expect(has(meta, "adsb_cpr_latitude"));
    };

    "an even and an odd half resolve the position they were encoded from, to the format's own resolution"_test = [] {
        constexpr double kLatitude  = 42.40448;
        constexpr double kLongitude = -71.34696;

        const auto [evenLat, evenLon] = encodeCpr(kLatitude, kLongitude, 0U);
        const auto [oddLat, oddLon]   = encodeCpr(kLatitude, kLongitude, 1U);

        const Decoded decoded = decode({recordOf(positionFrame(0xAB0969U, kLatitude, kLongitude, 0U, 0xC90U), 0ULL), //
            recordOf(positionFrame(0xAB0969U, kLatitude, kLongitude, 1U, 0xC90U), static_cast<std::uint64_t>(kRate), std::optional<bool>{true}, 1ULL)});

        expect(eq(decoded.records.size(), 2UZ));
        expect(eq(decoded.positions, 1ULL));

        // the first half names no position and says so by carrying its own encoded coordinates and nothing more
        const gr::property_map& first = metaOf(decoded, 0UZ);
        expect(!has(first, "adsb_latitude"));
        expect(eq(read<gr::Size_t>(first, "adsb_cpr_format").value_or(9U), gr::Size_t{0}));
        expect(eq(read<gr::Size_t>(first, "adsb_cpr_latitude").value_or(0U), static_cast<gr::Size_t>(evenLat)));
        expect(eq(read<gr::Size_t>(first, "adsb_cpr_longitude").value_or(0U), static_cast<gr::Size_t>(evenLon)));

        // the second completes the pair; 2^-17 of a zone is about 4.6e-5 degrees, which bounds what may be asked back
        const gr::property_map& second = metaOf(decoded, 1UZ);
        expect(eq(read<gr::Size_t>(second, "adsb_cpr_format").value_or(9U), gr::Size_t{1}));
        expect(eq(read<gr::Size_t>(second, "adsb_cpr_latitude").value_or(0U), static_cast<gr::Size_t>(oddLat)));
        expect(eq(read<gr::Size_t>(second, "adsb_cpr_longitude").value_or(0U), static_cast<gr::Size_t>(oddLon)));
        expect(std::abs(read<double>(second, "adsb_latitude").value_or(0.0) - kLatitude) < 1.0e-4) << read<double>(second, "adsb_latitude").value_or(0.0);
        expect(std::abs(read<double>(second, "adsb_longitude").value_or(0.0) - kLongitude) < 1.0e-4) << read<double>(second, "adsb_longitude").value_or(0.0);
    };

    "a half with no complement names no position, and a pair older than the limit is refused"_test = [] {
        constexpr double kLatitude  = 51.99;
        constexpr double kLongitude = 4.375;

        // one odd half alone: the encoded coordinates cross, the position does not
        const Decoded lone = decode({recordOf(positionFrame(0x400000U, kLatitude, kLongitude, 1U, 0xC90U), 0ULL)});
        expect(eq(lone.positions, 0ULL));
        expect(eq(lone.pairsExpired, 0ULL));
        expect(!has(metaOf(lone, 0UZ), "adsb_latitude"));
        expect(has(metaOf(lone, 0UZ), "adsb_cpr_longitude"));

        // the two halves twenty seconds apart against a ten second limit: counted as expired and not resolved
        const auto    twenty  = static_cast<std::uint64_t>(20.0 * static_cast<double>(kRate));
        const Decoded expired = decode({recordOf(positionFrame(0x400000U, kLatitude, kLongitude, 0U, 0xC90U), 0ULL), //
                                           recordOf(positionFrame(0x400000U, kLatitude, kLongitude, 1U, 0xC90U), twenty, std::optional<bool>{true}, 1ULL)},
            {{"pair_seconds", 10.f}});
        expect(eq(expired.positions, 0ULL));
        expect(eq(expired.pairsExpired, 1ULL));
        expect(!has(metaOf(expired, 1UZ), "adsb_latitude"));

        // the same pair inside a limit that admits it does resolve, so the refusal was the limit and not the pair
        const Decoded admitted = decode({recordOf(positionFrame(0x400000U, kLatitude, kLongitude, 0U, 0xC90U), 0ULL), //
                                            recordOf(positionFrame(0x400000U, kLatitude, kLongitude, 1U, 0xC90U), twenty, std::optional<bool>{true}, 1ULL)},
            {{"pair_seconds", 30.f}});
        expect(eq(admitted.positions, 1ULL));
        expect(std::abs(read<double>(metaOf(admitted, 1UZ), "adsb_latitude").value_or(0.0) - kLatitude) < 1.0e-4);
    };

    "the ground referenced velocity message gives a speed, a track and a vertical rate"_test = [] {
        // RTCA DO-260B 2.2.3.2 type code 19 subtype 1: east and north components, each one greater than the value it
        // states, with a sign bit before it; the vertical rate is 64 ft/min a count, its source bit set for
        // barometric. 300 kt east and 200 kt south is 360.6 kt on a track of 123.7 degrees.
        Frame frame = squitter(0xA9A901U, 19U);
        frame.put(38UZ, 3UZ, 1U);
        frame.put(46UZ, 1UZ, 0U).put(47UZ, 10UZ, 301U);                  // east, 300 kt
        frame.put(57UZ, 1UZ, 1U).put(58UZ, 10UZ, 201U);                  // south, 200 kt
        frame.put(68UZ, 1UZ, 1U).put(69UZ, 1UZ, 1U).put(70UZ, 9UZ, 17U); // barometric, down, 1024 ft/min

        const Decoded           decoded = decode({recordOf(frame, 29'092'000ULL)});
        const gr::property_map& meta    = metaOf(decoded, 0UZ);
        expect(eq(textOf(meta, "adsb_message"), std::string("airborne_velocity")));
        expect(eq(read<gr::Size_t>(meta, "adsb_velocity_subtype").value_or(0U), gr::Size_t{1}));
        expect(std::abs(read<double>(meta, "adsb_ground_speed_kt").value_or(0.0) - std::hypot(300.0, 200.0)) < 1.0e-9);
        expect(std::abs(read<double>(meta, "adsb_track_deg").value_or(0.0) - 123.69006752598) < 1.0e-6);
        expect(eq(read<std::int32_t>(meta, "adsb_vertical_rate_fpm").value_or(0), -1024));
        expect(eq(textOf(meta, "adsb_vertical_rate_source"), std::string("barometric")));
        expect(!has(meta, "adsb_airspeed_kt"));
    };

    "the air referenced velocity message gives an airspeed and a heading, and the supersonic subtype scales both"_test = [] {
        Frame frame = squitter(0xA9A901U, 19U);
        frame.put(38UZ, 3UZ, 3U);                                       // subtype 3, air referenced, subsonic
        frame.put(46UZ, 1UZ, 1U).put(47UZ, 10UZ, 256U);                 // heading available, a quarter turn from north
        frame.put(57UZ, 1UZ, 0U).put(58UZ, 10UZ, 251U);                 // indicated airspeed, 250 kt
        frame.put(68UZ, 1UZ, 0U).put(69UZ, 1UZ, 0U).put(70UZ, 9UZ, 2U); // geometric, up, 64 ft/min

        const Decoded           decoded = decode({recordOf(frame, 0ULL)});
        const gr::property_map& meta    = metaOf(decoded, 0UZ);
        expect(eq(read<gr::Size_t>(meta, "adsb_velocity_subtype").value_or(0U), gr::Size_t{3}));
        expect(std::abs(read<double>(meta, "adsb_airspeed_kt").value_or(0.0) - 250.0) < 1.0e-9);
        expect(eq(textOf(meta, "adsb_airspeed_type"), std::string("ias")));
        expect(std::abs(read<double>(meta, "adsb_heading_deg").value_or(0.0) - 90.0) < 1.0e-9);
        expect(eq(read<std::int32_t>(meta, "adsb_vertical_rate_fpm").value_or(0), 64));
        expect(eq(textOf(meta, "adsb_vertical_rate_source"), std::string("geometric")));
        expect(!has(meta, "adsb_ground_speed_kt"));

        // subtype 4 is the same message in units of four knots, which is the whole of the difference
        frame.put(38UZ, 3UZ, 4U);
        const Decoded fast = decode({recordOf(frame, 0ULL)});
        expect(std::abs(read<double>(metaOf(fast, 0UZ), "adsb_airspeed_kt").value_or(0.0) - 1000.0) < 1.0e-9);
    };

    "a frame the parity did not vouch for crosses undecoded, and the next one decodes"_test = [] {
        Frame velocity = squitter(0xA9A901U, 19U);
        velocity.put(38UZ, 3UZ, 1U).put(46UZ, 1UZ, 0U).put(47UZ, 10UZ, 301U).put(57UZ, 1UZ, 0U).put(58UZ, 10UZ, 201U);

        const Decoded decoded = decode({recordOf(velocity, 0ULL, std::optional<bool>{false}), // a long frame whose remainder was not zero
            recordOf(velocity, 100ULL, std::optional<bool>{}),                                // a short-format nomination, which carries no parity
            recordOf(velocity, 200ULL)});
        expect(eq(decoded.records.size(), 3UZ));
        expect(eq(decoded.unchecked, 2ULL));
        expect(eq(decoded.decoded, 1ULL));

        constexpr std::array<std::pair<std::size_t, std::string_view>, 2UZ> kRefused{{{0UZ, "mode_s/unchecked"}, {1UZ, "mode_s/short"}}};
        for (const auto& [index, protocol] : kRefused) {
            const gr::property_map& meta = metaOf(decoded, index);
            expect(eq(textOf(meta, "protocol"), std::string(protocol)));
            expect(!has(meta, "mode_s_icao"));
            expect(!has(meta, "adsb_type_code"));
            expect(!has(meta, "adsb_ground_speed_kt"));
            // the framer's own account crosses whole, including the key this block would have re-asserted
            expect(eq(read<gr::Size_t>(meta, "mode_s_format").value_or(0U), gr::Size_t{99}));
            expect(std::ranges::equal(decoded.records[index].signal_values, velocity.octets));
        }
        expect(eq(textOf(metaOf(decoded, 2UZ), "protocol"), std::string("mode_s/adsb")));
    };

    "the pairing table is bounded, and the address it gives up is the one least recently seen"_test = [] {
        constexpr double kLatitude  = 10.0;
        constexpr double kLongitude = 20.0;

        // three addresses through a table that holds two: the first is gone by the time its odd half arrives
        std::vector<Record> records;
        records.push_back(recordOf(positionFrame(0x000001U, kLatitude, kLongitude, 0U, 0xC90U), 0ULL));
        records.push_back(recordOf(positionFrame(0x000002U, kLatitude, kLongitude, 0U, 0xC90U), 100ULL, std::optional<bool>{true}, 1ULL));
        records.push_back(recordOf(positionFrame(0x000003U, kLatitude, kLongitude, 0U, 0xC90U), 200ULL, std::optional<bool>{true}, 2ULL));
        records.push_back(recordOf(positionFrame(0x000001U, kLatitude, kLongitude, 1U, 0xC90U), 300ULL, std::optional<bool>{true}, 3ULL));
        records.push_back(recordOf(positionFrame(0x000003U, kLatitude, kLongitude, 1U, 0xC90U), 400ULL, std::optional<bool>{true}, 4ULL));

        const Decoded decoded = decode(std::move(records), {{"table_size", gr::Size_t{2}}});
        expect(eq(decoded.evicted, 2ULL));   // one to admit the third address, one to admit the first back
        expect(eq(decoded.positions, 1ULL)); // only the address still held when its complement arrived
        expect(!has(metaOf(decoded, 3UZ), "adsb_latitude"));
        expect(has(metaOf(decoded, 4UZ), "adsb_latitude"));
    };

    "a setting outside its range is refused by name and leaves the block at the value it had"_test = [] {
        const std::array<std::pair<std::string_view, gr::pmt::Value>, 3UZ> kRefused{{{"pair_seconds", gr::pmt::Value(0.f)}, {"pair_seconds", gr::pmt::Value(std::numeric_limits<float>::quiet_NaN())}, {"table_size", gr::pmt::Value(gr::Size_t{0})}}};
        for (const auto& [key, value] : kRefused) {
            ModeSDecode block;
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
            bool threw  = false;
            try {
                std::ignore = block.settings().set(gr::property_map{{gr::property_map::key_type(key), value}});
                std::ignore = block.settings().activateContext();
                std::ignore = block.settings().applyStagedParameters();
            } catch (const gr::exception& error) {
                threw = true;
                expect(that % (std::string(error.message).find(std::string(key)) != std::string::npos)) << key;
            }
            expect(threw) << key;
            expect(block.pair_seconds.value > 0.f) << key;
            expect(gt(block.table_size.value, gr::Size_t{0})) << key;
        }
    };

    "every decoded field survives the record to packet boundary and the metadata encoder, at its own type"_test = [] {
        constexpr double kLatitude  = -33.86;
        constexpr double kLongitude = 151.21;

        // `QAN     `, three characters of the six-bit set and five pads
        constexpr std::array<std::uint32_t, 3UZ> kCharacters{17U, 1U, 14U};
        Frame                                    identity = squitter(0x7C4A08U, 1U);
        for (std::size_t i = 0UZ; i < 8UZ; ++i) {
            identity.put(41UZ + 6UZ * i, 6UZ, i < kCharacters.size() ? kCharacters[i] : 32U);
        }

        Frame velocity = squitter(0x7C4A08U, 19U);
        velocity.put(38UZ, 3UZ, 1U).put(46UZ, 1UZ, 0U).put(47UZ, 10UZ, 301U).put(57UZ, 1UZ, 1U).put(58UZ, 10UZ, 201U).put(68UZ, 1UZ, 1U).put(69UZ, 1UZ, 1U).put(70UZ, 9UZ, 17U);

        const Decoded decoded = decode({recordOf(identity, 0ULL),                                                           //
            recordOf(positionFrame(0x7C4A08U, kLatitude, kLongitude, 0U, 0x2E2U), 100ULL, std::optional<bool>{true}, 1ULL), //
            recordOf(positionFrame(0x7C4A08U, kLatitude, kLongitude, 1U, 0x2E2U), 200ULL, std::optional<bool>{true}, 2ULL), //
            recordOf(velocity, 300ULL, std::optional<bool>{true}, 3ULL)});
        expect(eq(decoded.records.size(), 4UZ));
        expect(eq(decoded.positions, 1ULL));
        expect(eq(textOf(metaOf(decoded, 0UZ), "adsb_callsign"), std::string("QAN")));

        // the same records through the stock record-to-packet pair, which is the road out of a process
        gr::Graph flow;
        auto&     source = flow.emplaceBlock<RecordSource>();
        source._records  = decoded.records;
        auto& toPacket   = flow.emplaceBlock<DataSetToPacket<std::uint8_t>>();
        auto& toRecord   = flow.emplaceBlock<PacketToDataSet<std::uint8_t>>();
        auto& collector  = flow.emplaceBlock<Collector<Record>>();
        expect(flow.connect<"out", "in">(source, toPacket).has_value());
        expect(flow.connect<"out", "in">(toPacket, toRecord).has_value());
        expect(flow.connect<"out", "in">(toRecord, collector).has_value());

        std::vector<Record> returned;
        runGraph(std::move(flow), [&returned, &collector] { returned = collector._items; });
        expect(eq(returned.size(), decoded.records.size()));

        std::size_t compared = 0UZ;
        for (std::size_t k = 0UZ; k < returned.size() && k < decoded.records.size(); ++k) {
            expect(!returned[k].meta_information.empty()) << "a packet carries exactly one metadata map, and so does the record it becomes";
            const gr::property_map& after = returned[k].meta_information.at(0UZ);
            compared += sameKeys(metaOf(decoded, k), after, 0.0); // nothing is rounded on this road, so nothing may differ
            expect(std::ranges::equal(returned[k].signal_values, decoded.records[k].signal_values)) << std::format("payload {}", k);

            // and the metadata is a wire format: the encoder the transports use writes and reads back every value.
            // Its floating point carries six significant digits, which sets the tolerance.
            const std::string written = gr::pmt::yaml::serialize(after);
            const auto        reread  = gr::pmt::yaml::deserialize(written);
            expect(reread.has_value()) << written;
            if (reread.has_value()) {
                compared += sameKeys(after, *reread, 1.0e-5);
            }
        }
        expect(gt(compared, 40UZ)) << "the comparison covered too few keys to say anything";
    };
};

int main() { /* not needed for UT */ }
