#ifndef GNURADIO_ADSB_MODE_S_HPP
#define GNURADIO_ADSB_MODE_S_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <map>
#include <memory_resource>
#include <numbers>
#include <optional>
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

#include <gnuradio-4.0/adsb/RecordFields.hpp>

/**
 * The message layer of the 1090 MHz Mode S downlink, as a record adapter over `DataSet<std::uint8_t>`.
 *
 * A frame reaches `ModeSDecode` after `gr::blocks::digital::PpmFramer` at the `mode_s` profile has read it out of a
 * magnitude stream and checked its parity, so what this block sees is 7 or 14 whole octets and the framer's account
 * of how they were taken. The framing is ICAO Annex 10 Volume IV; the extended squitter's message formats are
 * RTCA DO-260B, whose ME field occupies bits 33 to 88 of a long frame.
 *
 * The decoded fields are metadata and the payload stays the octets, so nothing the frame said is lost and a consumer
 * that speaks the protocol reads the same record as one that does not. Every value is a scalar of the record-metadata
 * vocabulary's transportable types — an integer, a double, a bool or a string — because a metadata map that crosses a
 * process is a wire format, and a sequence does not survive one.
 *
 * Bit numbering follows the standard: bit 1 is the most significant bit of the first octet.
 */
namespace gr::blocks::adsb {

namespace detail {

/// @brief @p count bits of @p frame starting at bit number @p first, one based. Bits past the frame read as zero.
[[nodiscard]] inline std::uint32_t field(std::span<const std::uint8_t> frame, std::size_t first, std::size_t count) noexcept {
    std::uint32_t value = 0U;
    for (std::size_t i = 0UZ; i < count; ++i) {
        const std::size_t  bit   = first - 1UZ + i;
        const std::uint8_t octet = bit / 8UZ < frame.size() ? frame[bit / 8UZ] : std::uint8_t{0};
        value                    = (value << 1U) | ((octet >> (7UZ - bit % 8UZ)) & 1U);
    }
    return value;
}

/// @brief The six-bit character set of the identification message, RTCA DO-260B 2.2.3.2. A code the set leaves
///        unassigned is spelled `#` here and is dropped rather than rendered.
constexpr std::string_view kIdentityCharacters = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######";

/// @brief The eight-character aircraft identification of a type code 1 to 4 message, its trailing pad removed.
[[nodiscard]] inline std::string identityOf(std::span<const std::uint8_t> frame) {
    std::string name;
    for (std::size_t i = 0UZ; i < 8UZ; ++i) {
        const char character = kIdentityCharacters[field(frame, 41UZ + 6UZ * i, 6UZ)];
        if (character != '#') {
            name.push_back(character);
        }
    }
    while (!name.empty() && name.back() == ' ') {
        name.pop_back();
    }
    return name;
}

/// @brief The pressure altitude in feet the AC field's 100 ft code names, or nothing when the code is unassigned.
///
/// ICAO Annex 10 Volume IV, the AC altitude code with its Q bit clear. The twelve bits are, most significant first,
/// C1 A1 C2 A2 C4 A4 B1 Q B2 D2 B4 D4: the thirteen-bit surveillance field without its M bit, and with D1 absent
/// because that pulse is only needed above 62 700 ft. The altitude is a count of 500 ft steps in a reflected binary
/// code on D2 D4 A1 A2 A4 B1 B2 B4, and a 100 ft digit within the step in the three C pulses, which count the other
/// way on an odd step. Three of the eight C codes are unassigned and the code floor is -1200 ft.
[[nodiscard]] inline std::optional<std::int32_t> gillhamAltitude(std::uint32_t ac12) noexcept {
    const auto pulse = [ac12](std::size_t position) { return (ac12 >> (12UZ - position)) & 1U; };

    // the five assigned C codes, C1 C2 C4 read as a three-bit number, and the 100 ft digit each of them names
    constexpr std::array<std::pair<std::uint32_t, std::int32_t>, 5UZ> kHundreds{{{0b001U, 1}, {0b011U, 2}, {0b010U, 3}, {0b110U, 4}, {0b100U, 5}}};
    const std::uint32_t                                               code  = (pulse(1UZ) << 2U) | (pulse(3UZ) << 1U) | pulse(5UZ);
    const auto                                                        digit = std::ranges::find(kHundreds, code, [](const auto& entry) { return entry.first; });
    if (digit == kHundreds.end()) {
        return std::nullopt;
    }

    // the 500 ft steps: D2 D4 A1 A2 A4 B1 B2 B4 at their positions in the field, most significant pulse first
    constexpr std::array<std::size_t, 8UZ> kStepPulses{10UZ, 12UZ, 2UZ, 4UZ, 6UZ, 7UZ, 9UZ, 11UZ};
    std::int32_t                           steps   = 0;
    std::uint32_t                          running = 0U;
    for (const std::size_t position : kStepPulses) {
        running ^= pulse(position); // a reflected binary code decodes as the running parity of its leading bits
        steps = (steps << 1) | static_cast<std::int32_t>(running);
    }

    const std::int32_t hundreds = steps % 2 == 0 ? digit->second : 6 - digit->second;
    const std::int32_t feet     = 500 * steps + 100 * hundreds - 1300;
    return feet >= -1200 ? std::optional<std::int32_t>{feet} : std::nullopt;
}

/// @brief The barometric altitude in feet of the twelve-bit AC field, or nothing where the field states none.
///
/// RTCA DO-260B 2.2.3.2, the altitude of an airborne position message. The Q bit at field position 8 selects the
/// coding: set, the remaining eleven bits are 25 ft increments offset by -1000 ft; clear, the field is the 100 ft
/// code above, which transponders in service do send and a decoder reading only the Q set case reports as absent.
[[nodiscard]] inline std::optional<std::int32_t> baroAltitude(std::uint32_t ac12) noexcept {
    if (ac12 == 0U) {
        return std::nullopt; // the all-zero code states that no altitude is available
    }
    if (((ac12 >> 4U) & 1U) != 0U) {
        const auto increments = static_cast<std::int32_t>(((ac12 >> 5U) << 4U) | (ac12 & 0x0FU));
        return increments * 25 - 1000;
    }
    return gillhamAltitude(ac12);
}

/// @brief The number of longitude zones at latitude @p degrees, RTCA DO-260B A.1.7.2.2's NL function.
///
/// The equation is the definition; the two poleward cases and the ceiling of 59 are the values it is defined to take
/// where it degenerates. The ceiling matters: at the equator the arc cosine's argument is its own cosine, so the
/// quotient is exactly 60 and rounding alone decides whether the floor reads 59 or 60.
[[nodiscard]] inline std::int32_t longitudeZones(double degrees) noexcept {
    constexpr double kZones = 15.0; // NZ, the number of geographic latitude zones in a quadrant
    const double     from   = std::abs(degrees);
    if (from >= 87.0) {
        return from > 87.0 ? 1 : 2;
    }
    const double cosine    = std::cos(std::numbers::pi / 180.0 * from);
    const double numerator = 1.0 - std::cos(std::numbers::pi / (2.0 * kZones));
    return std::min(59, static_cast<std::int32_t>(std::floor(2.0 * std::numbers::pi / std::acos(1.0 - numerator / (cosine * cosine)))));
}

/// @brief Floored modulo, which is the remainder the compact position arithmetic is defined over.
///
/// `std::fmod` keeps the sign of its left operand, so a negative zone index there puts the aircraft a few hundred
/// degrees from where it is rather than one zone over.
[[nodiscard]] inline double flooredMod(double value, double modulus) noexcept { return value - modulus * std::floor(value / modulus); }

/// @brief One half of a compact position pair, held until its complement arrives.
struct CprHalf {
    std::uint32_t latitude  = 0U;  ///< the encoded latitude, seventeen bits
    std::uint32_t longitude = 0U;  ///< the encoded longitude, seventeen bits
    double        at        = 0.0; ///< seconds into the stream, from the frame's own sample position
    bool          have      = false;
};

/// @brief The globally unambiguous position an even and an odd half name, RTCA DO-260B A.1.7.4.
///
/// Nothing is returned when the two halves resolve into different longitude zones, which is what a pair straddling a
/// zone boundary does: the pair says nothing rather than something wrong, and the next pair resolves.
[[nodiscard]] inline std::optional<std::pair<double, double>> globalPosition(const CprHalf& even, const CprHalf& odd) noexcept {
    constexpr double kScale = 131072.0; // 2^17, the encoded fraction of a zone

    const double evenLatitude = static_cast<double>(even.latitude) / kScale;
    const double oddLatitude  = static_cast<double>(odd.latitude) / kScale;
    const double index        = std::floor(59.0 * evenLatitude - 60.0 * oddLatitude + 0.5);

    double fromEven = (360.0 / 60.0) * (flooredMod(index, 60.0) + evenLatitude);
    double fromOdd  = (360.0 / 59.0) * (flooredMod(index, 59.0) + oddLatitude);
    if (fromEven >= 270.0) {
        fromEven -= 360.0; // the encoding's southern half, which the zone index states as the range above 270 degrees
    }
    if (fromOdd >= 270.0) {
        fromOdd -= 360.0;
    }
    if (longitudeZones(fromEven) != longitudeZones(fromOdd)) {
        return std::nullopt;
    }

    const bool         evenNewer = even.at >= odd.at;
    const double       latitude  = evenNewer ? fromEven : fromOdd;
    const std::int32_t zones     = longitudeZones(latitude);

    const double evenLongitude = static_cast<double>(even.longitude) / kScale;
    const double oddLongitude  = static_cast<double>(odd.longitude) / kScale;
    const double zoneIndex     = std::floor(evenLongitude * static_cast<double>(zones - 1) - oddLongitude * static_cast<double>(zones) + 0.5);
    const auto   width         = static_cast<double>(std::max(evenNewer ? zones : zones - 1, 1));

    double longitude = (360.0 / width) * (flooredMod(zoneIndex, width) + (evenNewer ? evenLongitude : oddLongitude));
    if (longitude >= 180.0) {
        longitude -= 360.0;
    }
    return std::pair{latitude, longitude};
}

/// @brief The message an extended squitter's type code names, RTCA DO-260B 2.2.3.2.
[[nodiscard]] inline std::string_view messageOf(std::uint32_t typeCode) noexcept {
    if (typeCode == 0U) {
        return "no_position";
    }
    if (typeCode <= 4U) {
        return "identification";
    }
    if (typeCode <= 8U) {
        return "surface_position";
    }
    if (typeCode <= 18U) {
        return "airborne_position";
    }
    if (typeCode == 19U) {
        return "airborne_velocity";
    }
    if (typeCode <= 22U) {
        return "airborne_position_gnss"; // the same layout with a height above the ellipsoid, which this block leaves undecoded
    }
    if (typeCode == 28U) {
        return "aircraft_status";
    }
    if (typeCode == 29U) {
        return "target_state";
    }
    if (typeCode == 31U) {
        return "operational_status";
    }
    return "reserved";
}

/// @brief The address the announced-address formats carry in bits 9 to 32: the all-call reply and the two squitters.
///
/// The remaining downlink formats put the address under the parity rather than in a field, so a frame of one of them
/// states no address this block may read.
[[nodiscard]] inline bool announcesAddress(std::uint32_t downlinkFormat) noexcept { return downlinkFormat == 11U || downlinkFormat == 17U || downlinkFormat == 18U; }

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::adsb::ModeSDecode)

/*!
@brief One Mode S frame in, the same octets out with the message read into typed metadata.

The record's items are the frame's octets, unchanged, so the parity the framer checked still covers what the record
carries and a consumer that wants the frame itself has it. Everything the message says is a metadata key of a
transportable scalar type — never a rendered line, which fixes a language and a field order at the producer where no
consumer can change them.

A frame is decoded only where the input record's `crc_ok` is true. The framer writes that key on long frames alone,
so a short-format nomination, which carries no self-checking parity, and a long frame whose remainder was not zero
both cross with their own metadata otherwise untouched, `protocol` naming which of the two they are. Reading fields out of
octets nothing vouched for is how a decoder reports an aircraft that is not there.

`protocol` names the stream's sub-kind from general to specific: `mode_s/adsb` on a decoded extended squitter,
`mode_s` on any other admitted frame, and `mode_s/unchecked` or `mode_s/short` on the two the framer publishes only
when asked. `mode_s_format` is re-asserted here from the octets under the spelling the framer already writes, so the
downlink format has one key and not two.

Compact position reporting needs two frames: an even and an odd half of one pair name a position, one half alone
names none. The halves are held per aircraft in a table of `table_size` addresses, the address least recently seen
giving way when it is full, and a pair whose halves are further apart in time than `pair_seconds` is refused and
counted. A frame whose pair does not resolve still carries its own encoded halves under `adsb_cpr_*`, so the
position is a consumer's to compute if it knows more than this block does.
*/
struct ModeSDecode : Block<ModeSDecode> {
    using Description = Doc<R""(
@brief Mode S message decode: one framer record in, its octets out with the downlink message as typed metadata.

The layer above `gr::blocks::digital::PpmFramer`. The framing is ICAO Annex 10 Volume IV and the extended squitter's
message formats are RTCA DO-260B: the downlink format, the announced address, and on formats 17 and 18 the type
code and the message it names — the aircraft identification, the barometric altitude in both of its codings, the
compact position halves and the position a pair of them resolves to, and the airborne velocity.

Every decoded field is a metadata key holding an integer, a double, a bool or a string, and the payload stays the
frame's octets. A record therefore crosses a process as a packet with nothing lost and nothing to parse twice.
)"">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<float, "pair_seconds", Unit<"s">, Doc<"greatest age difference between the two halves of a compact position pair; must be finite and greater than zero">, Visible>             pair_seconds{10.f};
    Annotated<gr::Size_t, "table_size", Doc<"aircraft addresses the position pairing table holds; the least recently seen gives way when it is full, and it must be at least one">, Visible> table_size{256U};

    GR_MAKE_REFLECTABLE(ModeSDecode, in, out, pair_seconds, table_size);

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nFrames       = 0ULL; ///< records that arrived
    std::uint64_t nDecoded      = 0ULL; ///< records whose octets the parity vouched for and whose message was read
    std::uint64_t nPositions    = 0ULL; ///< positions a pair of compact position halves resolved
    std::uint64_t nPairsExpired = 0ULL; ///< complete pairs refused because their halves were further apart than the limit
    std::uint64_t nUnchecked    = 0ULL; ///< records crossing undecoded, the parity having vouched for nothing
    std::uint64_t nEvicted      = 0ULL; ///< aircraft the pairing table gave up to make room

    /// @brief What one aircraft's address has accumulated, and when the table last touched it.
    struct Aircraft {
        detail::CprHalf even{};
        detail::CprHalf odd{};
        std::uint64_t   touched = 0ULL;
    };

    std::map<std::uint32_t, Aircraft> _aircraft{};
    std::uint64_t                     _touch = 0ULL; ///< monotonic, so the least recently seen address is the smallest
    float                             _pair  = 10.f;
    std::size_t                       _bound = 256UZ;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void reset() {
        _aircraft.clear();
        _touch = 0ULL;
    }

    void rebuild() {
        if (!std::isfinite(pair_seconds.value) || pair_seconds.value <= 0.f) {
            const float rejected = pair_seconds.value;
            pair_seconds.value   = _pair; // the block keeps working at the limit it already had
            throw gr::exception(std::format("pair_seconds is {}; it must be finite and greater than zero", rejected));
        }
        if (table_size.value == 0U) {
            table_size.value = static_cast<gr::Size_t>(_bound);
            throw gr::exception("table_size is zero; a pairing table that holds no aircraft can resolve no position");
        }
        _pair  = pair_seconds.value;
        _bound = static_cast<std::size_t>(table_size.value);
        while (_aircraft.size() > _bound) {
            evictOldest();
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t moved = std::min(inSpan.size(), outSpan.size());
        for (std::size_t i = 0UZ; i < moved; ++i) {
            outSpan[i] = decode(inSpan[i]);
        }
        std::ignore = inSpan.consume(moved);
        outSpan.publish(moved);
        if (moved == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

    /// @brief The record @p record becomes: its octets, its own facts, and what the message says where this reads it.
    [[nodiscard]] DataSet<std::uint8_t> decode(const DataSet<std::uint8_t>& record) {
        ++nFrames;

        DataSet<std::uint8_t> made;
        made.signal_values = record.signal_values;
        made.extents.push_back(static_cast<std::int32_t>(made.signal_values.size()));
        made.signal_names.emplace_back(record.signal_names.empty() ? std::string("mode_s") : record.signal_names[0UZ]);
        made.signal_quantities.emplace_back("");
        made.signal_units.emplace_back("");
        made.timing_events = record.timing_events;
        made.timing_events.resize(1UZ);
        made.meta_information.resize(1UZ);
        made.timestamp = record.timestamp;

        property_map& meta = made.meta_information[0UZ];
        if (!record.meta_information.empty()) {
            meta = record.meta_information[0UZ]; // the framer's facts carry through, this block's keys over them
        }

        const std::optional<bool> checked = detail::read<bool>(meta, "crc_ok");
        if (!checked.has_value() || !*checked) {
            ++nUnchecked;
            meta.insert_or_assign(property_map::key_type("protocol"), pmt::Value(std::string(checked.has_value() ? "mode_s/unchecked" : "mode_s/short")));
            return made;
        }

        const std::span<const std::uint8_t> frame          = std::span<const std::uint8_t>(made.signal_values);
        const std::uint32_t                 downlinkFormat = detail::field(frame, 1UZ, 5UZ);
        ++nDecoded;
        meta.insert_or_assign(property_map::key_type("mode_s_format"), pmt::Value(gr::Size_t{downlinkFormat}));
        if (detail::announcesAddress(downlinkFormat)) {
            meta.insert_or_assign(property_map::key_type("mode_s_icao"), pmt::Value(std::format("{:06X}", detail::field(frame, 9UZ, 24UZ))));
        }
        if (downlinkFormat != 17U && downlinkFormat != 18U) {
            meta.insert_or_assign(property_map::key_type("protocol"), pmt::Value(std::string("mode_s")));
            return made;
        }

        meta.insert_or_assign(property_map::key_type("protocol"), pmt::Value(std::string("mode_s/adsb")));
        readExtendedSquitter(frame, meta);
        return made;
    }

    /// @brief The ME field of a long frame, bits 33 to 88, written into @p meta as the message it names.
    void readExtendedSquitter(std::span<const std::uint8_t> frame, property_map& meta) {
        const std::uint32_t typeCode = detail::field(frame, 33UZ, 5UZ);
        meta.insert_or_assign(property_map::key_type("adsb_type_code"), pmt::Value(gr::Size_t{typeCode}));
        meta.insert_or_assign(property_map::key_type("adsb_message"), pmt::Value(std::string(detail::messageOf(typeCode))));

        if (typeCode >= 1U && typeCode <= 4U) {
            // RTCA DO-260B 2.2.3.2, aircraft identification: an emitter category and eight six-bit characters
            meta.insert_or_assign(property_map::key_type("adsb_emitter_category"), pmt::Value(gr::Size_t{detail::field(frame, 38UZ, 3UZ)}));
            meta.insert_or_assign(property_map::key_type("adsb_callsign"), pmt::Value(detail::identityOf(frame)));
            return;
        }
        if (typeCode >= 9U && typeCode <= 18U) {
            readAirbornePosition(frame, meta);
            return;
        }
        if (typeCode == 19U) {
            readAirborneVelocity(frame, meta);
        }
    }

    /// @brief The airborne position message, RTCA DO-260B 2.2.3.2: an altitude and one half of a position pair.
    void readAirbornePosition(std::span<const std::uint8_t> frame, property_map& meta) {
        const std::uint32_t altitudeCode = detail::field(frame, 41UZ, 12UZ);
        meta.insert_or_assign(property_map::key_type("adsb_altitude_q"), pmt::Value(((altitudeCode >> 4U) & 1U) != 0U));
        if (const std::optional<std::int32_t> feet = detail::baroAltitude(altitudeCode); feet.has_value()) {
            meta.insert_or_assign(property_map::key_type("adsb_altitude_ft"), pmt::Value(*feet));
        }

        // the CPR format bit at 54 says which half this is, and the two seventeen-bit encoded coordinates follow it
        const std::uint32_t format = detail::field(frame, 54UZ, 1UZ);
        detail::CprHalf     half{.latitude = detail::field(frame, 55UZ, 17UZ), .longitude = detail::field(frame, 72UZ, 17UZ), .at = 0.0, .have = true};
        meta.insert_or_assign(property_map::key_type("adsb_cpr_format"), pmt::Value(gr::Size_t{format}));
        meta.insert_or_assign(property_map::key_type("adsb_cpr_latitude"), pmt::Value(gr::Size_t{half.latitude}));
        meta.insert_or_assign(property_map::key_type("adsb_cpr_longitude"), pmt::Value(gr::Size_t{half.longitude}));

        const std::optional<std::string> address = detail::readText(meta, "mode_s_icao");
        const std::optional<double>      when    = detail::timeOf(meta);
        if (!address.has_value() || !when.has_value()) {
            return; // a half with no address to pair it against, or no time to age it by, resolves nothing
        }
        half.at = *when;

        Aircraft& state                         = touch(detail::field(frame, 9UZ, 24UZ));
        (format != 0U ? state.odd : state.even) = half;
        if (!state.even.have || !state.odd.have) {
            return;
        }
        if (std::abs(state.even.at - state.odd.at) > static_cast<double>(_pair)) {
            ++nPairsExpired;
            return;
        }
        if (const auto position = detail::globalPosition(state.even, state.odd); position.has_value()) {
            ++nPositions;
            meta.insert_or_assign(property_map::key_type("adsb_latitude"), pmt::Value(position->first));
            meta.insert_or_assign(property_map::key_type("adsb_longitude"), pmt::Value(position->second));
        }
    }

    /// @brief The airborne velocity message, RTCA DO-260B 2.2.3.2 type code 19, subtypes 1 to 4.
    ///
    /// Subtypes 1 and 2 are ground referenced and give a velocity as its east and north components; 3 and 4 are air
    /// referenced and give an airspeed and a magnetic heading. The supersonic pair, 2 and 4, states both in units of
    /// four knots. All four carry the same vertical rate.
    void readAirborneVelocity(std::span<const std::uint8_t> frame, property_map& meta) {
        const std::uint32_t subtype = detail::field(frame, 38UZ, 3UZ);
        meta.insert_or_assign(property_map::key_type("adsb_velocity_subtype"), pmt::Value(gr::Size_t{subtype}));
        const double knot = subtype == 2U || subtype == 4U ? 4.0 : 1.0;

        if (subtype == 1U || subtype == 2U) {
            const std::uint32_t east  = detail::field(frame, 47UZ, 10UZ);
            const std::uint32_t north = detail::field(frame, 58UZ, 10UZ);
            if (east != 0U && north != 0U) { // either component zero is the message's "velocity unavailable"
                const double eastward  = (detail::field(frame, 46UZ, 1UZ) != 0U ? -1.0 : 1.0) * (static_cast<double>(east) - 1.0) * knot;
                const double northward = (detail::field(frame, 57UZ, 1UZ) != 0U ? -1.0 : 1.0) * (static_cast<double>(north) - 1.0) * knot;
                meta.insert_or_assign(property_map::key_type("adsb_ground_speed_kt"), pmt::Value(std::hypot(eastward, northward)));
                meta.insert_or_assign(property_map::key_type("adsb_track_deg"), pmt::Value(detail::flooredMod(std::atan2(eastward, northward) * 180.0 / std::numbers::pi, 360.0)));
            }
        } else if (subtype == 3U || subtype == 4U) {
            if (const std::uint32_t airspeed = detail::field(frame, 58UZ, 10UZ); airspeed != 0U) {
                meta.insert_or_assign(property_map::key_type("adsb_airspeed_kt"), pmt::Value((static_cast<double>(airspeed) - 1.0) * knot));
                meta.insert_or_assign(property_map::key_type("adsb_airspeed_type"), pmt::Value(std::string(detail::field(frame, 57UZ, 1UZ) != 0U ? "tas" : "ias")));
            }
            if (detail::field(frame, 46UZ, 1UZ) != 0U) { // the heading status bit; clear, the heading field states nothing
                meta.insert_or_assign(property_map::key_type("adsb_heading_deg"), pmt::Value(static_cast<double>(detail::field(frame, 47UZ, 10UZ)) * 360.0 / 1024.0));
            }
        }

        if (const std::uint32_t rate = detail::field(frame, 70UZ, 9UZ); rate != 0U) {
            const std::int32_t feetPerMinute = (detail::field(frame, 69UZ, 1UZ) != 0U ? -1 : 1) * (static_cast<std::int32_t>(rate) - 1) * 64;
            meta.insert_or_assign(property_map::key_type("adsb_vertical_rate_fpm"), pmt::Value(feetPerMinute));
            meta.insert_or_assign(property_map::key_type("adsb_vertical_rate_source"), pmt::Value(std::string(detail::field(frame, 68UZ, 1UZ) != 0U ? "barometric" : "geometric")));
        }
    }

    /// @brief The table entry for @p address, made room for at the bound by giving up the least recently seen address.
    [[nodiscard]] Aircraft& touch(std::uint32_t address) {
        const auto found = _aircraft.find(address);
        if (found == _aircraft.end() && _aircraft.size() >= _bound) {
            evictOldest();
        }
        Aircraft& state = _aircraft[address];
        state.touched   = ++_touch;
        return state;
    }

    void evictOldest() {
        const auto oldest = std::ranges::min_element(_aircraft, {}, [](const auto& entry) { return entry.second.touched; });
        if (oldest != _aircraft.end()) {
            _aircraft.erase(oldest);
            ++nEvicted;
        }
    }
};

} // namespace gr::blocks::adsb

#endif // include guard
