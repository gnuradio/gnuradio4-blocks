#ifndef GNURADIO_ADSB_ADSB_PRINTER_HPP
#define GNURADIO_ADSB_ADSB_PRINTER_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <format>
#include <iterator>
#include <map>
#include <memory_resource>
#include <optional>
#include <print>
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
 * The console renderer of a decoded Mode S stream: one line of text per record, as a record of its own.
 *
 * It reads only the typed keys `ModeSDecode` writes and never the octets, except to show a frame nothing decoded.
 * The line is the payload of the record it publishes, so a console, a file and a network peer are three consumers of
 * one stream and the rendering happens once.
 */
namespace gr::blocks::adsb {

GR_REGISTER_BLOCK(gr::blocks::adsb::AdsbPrinter)

/*!
@brief One decoded record in, the line that describes it out as the payload of a text record.

The line is

    `   29.781  DF17  AB0969  airpos  alt 39000 ft  lat 42.40448  lon -71.34696  [AAL160]`

— the time in seconds from the frame's own `sample_start` and `sample_rate`, the downlink format, the address, what
the message said, and the aircraft's identification in brackets where an earlier frame from that address gave one.
A record the decoder left unread prints its protocol and its octets instead, because that is all it may be believed
about. The published record carries no trailing newline; `to_stdout` writes one after each line it prints.

The identifications are remembered per address in a table of `table_size` entries, the address least recently seen
giving way when it is full. That memory is the only state here: a line is otherwise a function of one record.

`protocol` is `text/adsb` — a rendered line, from ADS-B — and every key of the input record crosses beneath it, so a
consumer can still filter on the address or the altitude it was rendered from. The reserved `source_id` is not
written: the vocabulary gives that key to the receiver, as the operator's name for it, and a producer claiming it
would displace what the receiver has to say.
*/
struct AdsbPrinter : Block<AdsbPrinter> {
    using Description = Doc<R""(
@brief Renders each decoded Mode S record as one line of text, published as a record and optionally written to stdout.

The consumer of `gr::blocks::adsb::ModeSDecode`. It reads that block's typed metadata keys, never the octets, so the
line and a network peer's structured view of the same record cannot disagree. `to_stdout` additionally writes each
line to standard output, which is what a headless runner with no consumer attached wants.
)"">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<bool, "to_stdout", Doc<"also write each line to standard output, for a run with no consumer attached">, Visible>                                                        to_stdout{false};
    Annotated<gr::Size_t, "table_size", Doc<"addresses whose identification is remembered; the least recently seen gives way when it is full, and it must be at least one">, Visible> table_size{256U};

    GR_MAKE_REFLECTABLE(AdsbPrinter, in, out, to_stdout, table_size);

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nLines   = 0ULL; ///< lines rendered and published
    std::uint64_t nNamed   = 0ULL; ///< lines that carried an identification the table remembered
    std::uint64_t nEvicted = 0ULL; ///< addresses the identification table gave up to make room

    /// @brief One remembered identification and when the table last touched it.
    struct Named {
        std::string   callsign{};
        std::uint64_t touched = 0ULL;
    };

    std::map<std::string, Named> _named{};
    std::uint64_t                _touch = 0ULL;
    std::size_t                  _bound = 256UZ;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void reset() {
        _named.clear();
        _touch = 0ULL;
    }

    void rebuild() {
        if (table_size.value == 0U) {
            table_size.value = static_cast<gr::Size_t>(_bound);
            throw gr::exception("table_size is zero; a table that remembers no address can name no aircraft");
        }
        _bound = static_cast<std::size_t>(table_size.value);
        while (_named.size() > _bound) {
            evictOldest();
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t moved = std::min(inSpan.size(), outSpan.size());
        for (std::size_t i = 0UZ; i < moved; ++i) {
            outSpan[i] = render(inSpan[i]);
        }
        std::ignore = inSpan.consume(moved);
        outSpan.publish(moved);
        if (to_stdout.value && moved > 0UZ) {
            std::fflush(stdout);
        }
        if (moved == 0UZ) {
            // frames are minutes apart on a quiet band, so an empty span is the normal case and has to say so:
            // OK with nothing consumed and nothing published is what the scheduler reports as a stalled block
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

    /// @brief The text record @p record becomes: the line as its items, and every fact of the input beneath it.
    [[nodiscard]] DataSet<std::uint8_t> render(const DataSet<std::uint8_t>& record) {
        const property_map  none{};
        const property_map& meta = record.meta_information.empty() ? none : record.meta_information[0UZ];
        const std::string   line = lineOf(record, meta);
        ++nLines;
        if (to_stdout.value) {
            std::println("{}", line);
        }

        DataSet<std::uint8_t> made;
        made.signal_values.assign(line.begin(), line.end());
        made.extents.push_back(static_cast<std::int32_t>(made.signal_values.size()));
        made.signal_names.emplace_back("text");
        made.signal_quantities.emplace_back("");
        made.signal_units.emplace_back("");
        made.timing_events.resize(1UZ);
        made.meta_information.resize(1UZ);
        made.meta_information[0UZ] = meta;
        made.meta_information[0UZ].insert_or_assign(property_map::key_type("protocol"), pmt::Value(std::string("text/adsb")));
        made.timestamp = record.timestamp;
        return made;
    }

    [[nodiscard]] static std::string textOf(const property_map& meta, std::string_view key) { return detail::readText(meta, key).value_or(std::string{}); }

    [[nodiscard]] static std::string hexOf(std::span<const std::uint8_t> octets) {
        std::string text;
        text.reserve(octets.size() * 2UZ);
        for (const std::uint8_t octet : octets) {
            std::format_to(std::back_inserter(text), "{:02X}", octet);
        }
        return text;
    }

    [[nodiscard]] std::string lineOf(const DataSet<std::uint8_t>& record, const property_map& meta) {
        const double                    seconds = detail::timeOf(meta).value_or(0.0);
        const std::optional<gr::Size_t> format  = detail::read<gr::Size_t>(meta, "mode_s_format");
        if (!format.has_value()) {
            // a frame the decoder would not read: its protocol and its octets are all that may be said about it
            const std::string protocol = textOf(meta, "protocol");
            return std::format("{:9.3f}  {}  {}", seconds, protocol.empty() ? std::string("mode_s") : protocol, hexOf(std::span<const std::uint8_t>(record.signal_values)));
        }

        const std::string address    = textOf(meta, "mode_s_icao");
        const std::string identified = remember(address, detail::readText(meta, "adsb_callsign"));
        // an identification message's own line already names the aircraft, so the bracket would say it twice
        const bool bracket = !identified.empty() && textOf(meta, "adsb_message") != "identification";
        if (bracket) {
            ++nNamed;
        }
        return std::format("{:9.3f}  DF{}  {}  {}{}", seconds, *format, address.empty() ? std::string("------") : address, detailOf(meta), bracket ? std::format("  [{}]", identified) : std::string{});
    }

    [[nodiscard]] static std::string detailOf(const property_map& meta) {
        const std::string message = textOf(meta, "adsb_message");
        if (message.empty()) {
            return textOf(meta, "protocol"); // an admitted frame of a format that carries no ADS-B message
        }
        if (message == "identification") {
            return std::format("ident   callsign {}", textOf(meta, "adsb_callsign"));
        }
        if (message == "airborne_position") {
            const std::optional<std::int32_t> feet      = detail::read<std::int32_t>(meta, "adsb_altitude_ft");
            std::string                       text      = feet.has_value() ? std::format("airpos  alt {} ft", *feet) : std::string("airpos  alt n/a");
            const std::optional<double>       latitude  = detail::read<double>(meta, "adsb_latitude");
            const std::optional<double>       longitude = detail::read<double>(meta, "adsb_longitude");
            if (latitude.has_value() && longitude.has_value()) {
                std::format_to(std::back_inserter(text), "  lat {:.5f}  lon {:.5f}", *latitude, *longitude);
            }
            return text;
        }
        if (message == "airborne_velocity") {
            return std::format("veloc   {}", velocityOf(meta));
        }
        const std::optional<gr::Size_t> typeCode = detail::read<gr::Size_t>(meta, "adsb_type_code");
        return typeCode.has_value() ? std::format("tc {} ({})", *typeCode, message) : message;
    }

    [[nodiscard]] static std::string velocityOf(const property_map& meta) {
        std::string                 text;
        const std::optional<double> ground = detail::read<double>(meta, "adsb_ground_speed_kt");
        const std::optional<double> air    = detail::read<double>(meta, "adsb_airspeed_kt");
        if (ground.has_value()) {
            text = std::format("gs {:.0f} kt", *ground);
            if (const std::optional<double> track = detail::read<double>(meta, "adsb_track_deg"); track.has_value()) {
                std::format_to(std::back_inserter(text), "  track {:.0f} deg", *track);
            }
        } else if (air.has_value()) {
            text = std::format("{} {:.0f} kt", textOf(meta, "adsb_airspeed_type"), *air);
        } else {
            text = "velocity unavailable";
        }
        if (const std::optional<double> heading = detail::read<double>(meta, "adsb_heading_deg"); heading.has_value()) {
            std::format_to(std::back_inserter(text), "  heading {:.0f} deg", *heading);
        }
        if (const std::optional<std::int32_t> vertical = detail::read<std::int32_t>(meta, "adsb_vertical_rate_fpm"); vertical.has_value()) {
            std::format_to(std::back_inserter(text), "  vs {:+d} ft/min", *vertical);
        }
        return text;
    }

    /// @brief The identification known for @p address, updated first where @p callsign states one.
    [[nodiscard]] std::string remember(const std::string& address, const std::optional<std::string>& callsign) {
        if (address.empty()) {
            return callsign.value_or(std::string{});
        }
        const auto found = _named.find(address);
        if (found == _named.end() && !callsign.has_value()) {
            return {};
        }
        if (found == _named.end() && _named.size() >= _bound) {
            evictOldest();
        }
        Named& entry = _named[address];
        if (callsign.has_value()) {
            entry.callsign = *callsign;
        }
        entry.touched = ++_touch;
        return entry.callsign;
    }

    void evictOldest() {
        const auto oldest = std::ranges::min_element(_named, {}, [](const auto& entry) { return entry.second.touched; });
        if (oldest != _named.end()) {
            _named.erase(oldest);
            ++nEvicted;
        }
    }
};

} // namespace gr::blocks::adsb

#endif // include guard
