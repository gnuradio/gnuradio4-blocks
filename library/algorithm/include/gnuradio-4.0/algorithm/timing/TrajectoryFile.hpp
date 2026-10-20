#ifndef GNURADIO_ALGORITHM_TRAJECTORY_FILE_HPP
#define GNURADIO_ALGORITHM_TRAJECTORY_FILE_HPP

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <format>
#include <fstream>
#include <ios>
#include <limits>
#include <optional>
#include <source_location>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/algorithm/timing/FrequencySchedule.hpp>

/**
 * @brief The version-1 trajectory table, a text file of knots, and the two schedules made from it.
 *
 * A pass has hundreds to thousands of knots. A file carries them with a header that states their units and time
 * scale.
 *
 * ```
 * #!gr4-trajectory 1
 * columns    time range_m range_rate_m_s
 * time_scale utc_iso8601
 * carrier_hz 437000000
 * source     sgp4 over a 2500 km pass
 * 2026-09-02T10:00:00Z          2500000.0  -7000.0
 * 2026-09-02T10:00:20.5Z        2360000.0  -6900.0
 * ```
 *
 * The version is in-band, on the first line, and that line holds only the version. The `#!` prefix identifies
 * the file to `file(1)` and to a person who opens it by mistake. The integer after the space is the **format**
 * version. A reader built for version 1 refuses any other value by number and does not guess. A later version
 * that only adds optional headers still increments it. A reader that ignored unknown headers would also ignore
 * the one that mattered, with no error.
 *
 * Every refusal names its line. The line number is the part of a parse failure an operator can act on.
 * `time_scale` and `carrier_hz` each take one value. `carrier_hz` is in hertz. A unit or a comment after a value
 * is refused.
 *
 * **The loader does not differentiate or integrate**, and that follows from the math. Both schedules are
 * piecewise linear and therefore continuous. The derivative of a continuous piecewise-linear function is
 * piecewise *constant*, with a jump at every knot. The integral is piecewise *quadratic*. Neither fits the
 * class `PiecewiseLinearSchedule` represents. Either conversion would have to approximate, and the error is not
 * small. The secant slope of a range table misstates the range rate by up to half the rate's change over a
 * segment. With knots every 20 s and a range acceleration of 12 m/s^2 the secant slope is off by 120 m/s, 175 Hz
 * at 437 MHz. The loader therefore only converts units. A caller that needs both schedules supplies both columns.
 *
 * The loader is **not** on the hot path. It runs once, outside the sample path. Its cost is bounded by the
 * file's size. It makes one pass over the text with no backtracking. It allocates nothing per line beyond the
 * vectors it fills.
 */
namespace gr::timing {

/**
 * @brief The contents of one trajectory file, as a frequency schedule and a delay schedule.
 *
 * A schedule is present exactly when the file named the column that produces it. A file with only `range_m`
 * yields no `frequency`. A caller that needs one learns which column is missing. It does not receive a table
 * derived from the wrong quantity.
 */
struct Trajectory {
    std::optional<FrequencySchedule> frequency;                                            ///< from `offset_hz` verbatim, or from `range_rate_m_s` through `offsetFor`
    std::optional<DelaySchedule>     delay;                                                ///< from `range_m` through `delayFor`, in seconds
    double                           carrier_hz{std::numeric_limits<double>::quiet_NaN()}; ///< the `carrier_hz` header, or NaN when the file stated none
    std::string                      source;                                               ///< the `source` header, or empty
};

namespace detail {

/// The quantities a knot line may carry beyond its time, in the order `columns` named them.
enum class TrajectoryQuantity : std::uint8_t { offset_hz, range_m, range_rate_m_s };

[[nodiscard]] inline std::string_view trajectoryQuantityName(TrajectoryQuantity q) noexcept {
    switch (q) {
    case TrajectoryQuantity::offset_hz: return "offset_hz";
    case TrajectoryQuantity::range_m: return "range_m";
    case TrajectoryQuantity::range_rate_m_s: return "range_rate_m_s";
    }
    return "";
}

[[nodiscard]] inline std::unexpected<gr::Error> trajectoryRefuse(std::size_t line, std::string_view what, std::string_view offending, std::source_location loc = std::source_location::current()) { return std::unexpected(gr::Error(std::format("gr::timing::loadTrajectory: line {}: {} — {}", line, what, offending), loc)); }

/// @brief Lines of the text, with `\r` stripped from each end. A file from another platform then loads.
///
/// A `\r` in the middle of a line is *not* stripped. It then fails the byte check below. Such a line is not
/// the text it claims to be.
struct TrajectoryLines {
    std::string_view text;
    std::size_t      pos    = 0UZ;
    std::size_t      number = 0UZ;

    [[nodiscard]] bool next(std::string_view& out) noexcept {
        if (pos >= text.size()) {
            return false;
        }
        ++number;
        const std::size_t nl  = text.find('\n', pos);
        std::string_view  raw = (nl == std::string_view::npos) ? text.substr(pos) : text.substr(pos, nl - pos);
        pos                   = (nl == std::string_view::npos) ? text.size() : nl + 1UZ;
        if (!raw.empty() && raw.back() == '\r') {
            raw.remove_suffix(1UZ);
        }
        out = raw;
        return true;
    }
};

/// @brief Whitespace-separated fields of one line, without allocating.
struct TrajectoryFields {
    std::string_view line;
    std::size_t      pos = 0UZ;

    [[nodiscard]] static bool blank(char c) noexcept { return c == ' ' || c == '\t'; }

    [[nodiscard]] bool next(std::string_view& out) noexcept {
        while (pos < line.size() && blank(line[pos])) {
            ++pos;
        }
        if (pos >= line.size()) {
            return false;
        }
        const std::size_t start = pos;
        while (pos < line.size() && !blank(line[pos])) {
            ++pos;
        }
        out = line.substr(start, pos - start);
        return true;
    }

    /// The rest of the line from the current position, with leading blanks dropped. A header value runs to the
    /// end of the line.
    [[nodiscard]] std::string_view rest() const noexcept {
        std::size_t at = pos;
        while (at < line.size() && blank(line[at])) {
            ++at;
        }
        return line.substr(at);
    }
};

/// The file is US-ASCII. A byte outside the printable range, other than tab, is refused. A recording passed in
/// place of a table then gives one refusal and not ten thousand.
[[nodiscard]] inline std::expected<void, gr::Error> trajectoryCheckBytes(std::size_t line, std::string_view raw) {
    for (const char c : raw) {
        const auto byte = static_cast<unsigned char>(c);
        if (byte != '\t' && (byte < 0x20U || byte > 0x7EU)) {
            return trajectoryRefuse(line, std::format("byte 0x{:02X} is outside the printable US-ASCII the format admits", byte), raw);
        }
    }
    return {};
}

/// A header key begins with a letter or an underscore. A knot's time begins with a digit or a sign under either
/// time scale. The first character separates the two. The header block therefore ends at the first knot line,
/// and an unknown key on a line that is plainly a header is still refused.
[[nodiscard]] inline bool trajectoryLooksLikeKey(std::string_view token) noexcept {
    if (token.empty()) {
        return false;
    }
    const char c = token.front();
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
}

/// One leading `+` is dropped when a digit, or a point where @p point allows one, follows it. A second sign
/// after it stays and is refused.
[[nodiscard]] inline std::string_view trajectoryUnsigned(std::string_view field, bool point) noexcept {
    if (field.size() < 2UZ || field.front() != '+') {
        return field;
    }
    const char next = field[1];
    return ((next >= '0' && next <= '9') || (point && next == '.')) ? field.substr(1UZ) : field;
}

/// A finite decimal number with an optional sign. @p name is the quantity the refusal names.
[[nodiscard]] inline std::expected<double, gr::Error> trajectoryValue(std::size_t line, std::string_view field, std::string_view name) {
    const std::string_view number = trajectoryUnsigned(field, true);
    double                 out    = 0.;
    const auto [end, ec]          = std::from_chars(number.data(), number.data() + number.size(), out, std::chars_format::general);
    if (ec != std::errc{} || end != number.data() + number.size()) {
        return trajectoryRefuse(line, std::format("`{}` is not a number, or carries trailing characters", name), field);
    }
    if (!std::isfinite(out)) {
        return trajectoryRefuse(line, std::format("`{}` is not finite", name), field);
    }
    return out;
}

/// `unix_ns`, an optional sign followed by decimal digits only. A nanosecond count does not pass through a
/// floating type. There is no exponent form and no decimal point.
[[nodiscard]] inline std::expected<std::int64_t, gr::Error> trajectoryUnixNs(std::size_t line, std::string_view field) {
    const std::string_view digits = trajectoryUnsigned(field, false);
    std::int64_t           out    = 0;
    const auto [end, ec]          = std::from_chars(digits.data(), digits.data() + digits.size(), out, 10);
    if (ec == std::errc::result_out_of_range) {
        return trajectoryRefuse(line, "the unix_ns time overflows the signed 64-bit nanosecond axis", field);
    }
    if (ec != std::errc{} || end != digits.data() + digits.size()) {
        return trajectoryRefuse(line, "the unix_ns time is not a decimal integer", field);
    }
    return out;
}

[[nodiscard]] inline bool trajectoryDigits(std::string_view field, std::size_t at, std::size_t count) noexcept {
    if (at + count > field.size()) {
        return false;
    }
    for (std::size_t i = 0UZ; i < count; ++i) {
        const char c = field[at + i];
        if (c < '0' || c > '9') {
            return false;
        }
    }
    return true;
}

[[nodiscard]] inline int trajectoryInt(std::string_view field, std::size_t at, std::size_t count) noexcept {
    int out = 0;
    for (std::size_t i = 0UZ; i < count; ++i) {
        out = out * 10 + (field[at + i] - '0');
    }
    return out;
}

/**
 * @brief `YYYY-MM-DDThh:mm:ss[.f{1,9}]Z` to nanoseconds since the Unix epoch, in integers throughout.
 *
 * Every field has a fixed width. `T` and `Z` are required and literal. No timezone offset form is accepted.
 * Honoring one would need a rule about which offsets are legal, and this format has no such rule. A producer
 * with an offset converts before writing.
 *
 * The fractional part is parsed as an **integer** and not as a fraction. The digits after the point are read
 * as a decimal integer and scaled by `10^(9-d)` for `d` digits. `.5` is exactly 500000000 ns, and
 * `.000000001` is exactly 1 ns. More than nine digits are refused, because truncating them would lose data
 * with no error.
 *
 * A seconds field of `60` is refused with its reason named. The axis is a plain nanosecond count, and a leap
 * second has no representation on it.
 */
[[nodiscard]] inline std::expected<std::int64_t, gr::Error> trajectoryIso8601Ns(std::size_t line, std::string_view field) {
    constexpr std::size_t kFixed = 19UZ; // YYYY-MM-DDThh:mm:ss

    const bool shaped = field.size() > kFixed                                     //
                        && trajectoryDigits(field, 0UZ, 4UZ) && field[4] == '-'   //
                        && trajectoryDigits(field, 5UZ, 2UZ) && field[7] == '-'   //
                        && trajectoryDigits(field, 8UZ, 2UZ) && field[10] == 'T'  //
                        && trajectoryDigits(field, 11UZ, 2UZ) && field[13] == ':' //
                        && trajectoryDigits(field, 14UZ, 2UZ) && field[16] == ':' && trajectoryDigits(field, 17UZ, 2UZ);
    if (!shaped) {
        return trajectoryRefuse(line, "the time is not YYYY-MM-DDThh:mm:ss[.f{1,9}]Z at fixed field widths", field);
    }

    std::int64_t fraction = 0;
    if (field[kFixed] == '.') {
        const std::size_t first = kFixed + 1UZ;
        std::size_t       last  = first;
        while (last < field.size() && field[last] >= '0' && field[last] <= '9') {
            ++last;
        }
        const std::size_t digits = last - first;
        if (digits == 0UZ) {
            return trajectoryRefuse(line, "the fractional second carries no digits", field);
        }
        if (digits > 9UZ) {
            return trajectoryRefuse(line, std::format("the fractional second carries {} digits, past the nine a nanosecond axis holds", digits), field);
        }
        if (last + 1UZ != field.size() || field[last] != 'Z') {
            return trajectoryRefuse(line, "the time does not end in the literal Z, and no offset form is accepted", field);
        }
        fraction = trajectoryInt(field, first, digits);
        for (std::size_t i = digits; i < 9UZ; ++i) {
            fraction *= 10;
        }
    } else if (field[kFixed] != 'Z' || field.size() != kFixed + 1UZ) {
        return trajectoryRefuse(line, "the time does not end in the literal Z, and no offset form is accepted", field);
    }

    const int second = trajectoryInt(field, 17UZ, 2UZ);
    if (second == 60) {
        return trajectoryRefuse(line, "a seconds field of 60 names a leap second, which a plain nanosecond axis cannot represent", field);
    }

    const int year   = trajectoryInt(field, 0UZ, 4UZ);
    const int month  = trajectoryInt(field, 5UZ, 2UZ);
    const int day    = trajectoryInt(field, 8UZ, 2UZ);
    const int hour   = trajectoryInt(field, 11UZ, 2UZ);
    const int minute = trajectoryInt(field, 14UZ, 2UZ);

    const std::chrono::year_month_day date{std::chrono::year{year}, std::chrono::month{static_cast<unsigned>(month)}, std::chrono::day{static_cast<unsigned>(day)}};
    if (!date.ok() || hour > 23 || minute > 59 || second > 59) {
        return trajectoryRefuse(line, "a calendar field is out of range", field);
    }

    // 4-digit years reach 9999, and the axis reaches 2262-04-11. The product below overflows first. It is
    // therefore checked against the axis and not against the year. In the axis's last second only the fraction
    // up to its last nanosecond fits.
    constexpr std::int64_t kMaxSeconds  = std::numeric_limits<std::int64_t>::max() / 1'000'000'000LL;
    constexpr std::int64_t kMaxFraction = std::numeric_limits<std::int64_t>::max() % 1'000'000'000LL;

    const std::int64_t seconds = std::chrono::sys_days(date).time_since_epoch().count() * 86'400LL + hour * 3'600LL + minute * 60LL + second;
    if (seconds > kMaxSeconds || seconds < -kMaxSeconds) {
        return trajectoryRefuse(line, std::format("the instant is {} s from the epoch, outside the +/-{} s the signed 64-bit nanosecond axis holds", seconds, kMaxSeconds), field);
    }
    if (seconds == kMaxSeconds && fraction > kMaxFraction) {
        return trajectoryRefuse(line, std::format("the instant is {} ns past second {}, beyond the last nanosecond the signed 64-bit nanosecond axis holds", fraction, seconds), field);
    }
    return seconds * 1'000'000'000LL + fraction;
}

} // namespace detail

/**
 * @brief Load a version-1 trajectory table from its text.
 *
 * A pure function of its argument, with no global state and no locale dependence. It uses only
 * `std::from_chars` and `std::chrono`. `strtod` under a comma locale is the kind of defect this avoids.
 *
 * A refused file is an error with the line number in its message. No partial result escapes. The schedules are
 * constructed after the whole file has parsed.
 */
[[nodiscard]] inline std::expected<Trajectory, gr::Error> loadTrajectory(std::string_view text) {
    using detail::TrajectoryQuantity;
    using detail::trajectoryRefuse;

    detail::TrajectoryLines lines{text};
    std::string_view        raw;

    if (!lines.next(raw)) {
        return trajectoryRefuse(1UZ, "the file is empty and carries no version line", "");
    }
    if (const auto bytes = detail::trajectoryCheckBytes(1UZ, raw); !bytes) {
        return std::unexpected(bytes.error());
    }
    {
        constexpr std::string_view kMagic = "#!gr4-trajectory ";
        if (!raw.starts_with(kMagic)) {
            return trajectoryRefuse(1UZ, "the first line is not `#!gr4-trajectory <n>`", raw);
        }
        const std::string_view digits  = raw.substr(kMagic.size());
        int                    version = 0;
        const auto [end, ec]           = std::from_chars(digits.data(), digits.data() + digits.size(), version, 10);
        if (digits.empty() || ec != std::errc{} || end != digits.data() + digits.size()) {
            return trajectoryRefuse(1UZ, "the first line is not `#!gr4-trajectory <n>`", raw);
        }
        if (version != 1) {
            return trajectoryRefuse(1UZ, std::format("format version {} — this reader reads version 1", version), raw);
        }
    }

    std::optional<std::size_t> columnsLine;
    std::optional<std::size_t> timeScaleLine;
    std::optional<std::size_t> carrierLine;
    std::optional<std::size_t> sourceLine;

    std::vector<TrajectoryQuantity> quantities;
    bool                            isoTime   = false;
    double                          carrierHz = std::numeric_limits<double>::quiet_NaN();
    std::string                     source;

    std::string_view pendingKnot;
    std::size_t      pendingLine = 0UZ;
    bool             havePending = false;

    while (lines.next(raw)) {
        if (const auto bytes = detail::trajectoryCheckBytes(lines.number, raw); !bytes) {
            return std::unexpected(bytes.error());
        }

        detail::TrajectoryFields fields{raw};
        std::string_view         key;
        if (!fields.next(key) || key.front() == '#') {
            continue;
        }
        if (!detail::trajectoryLooksLikeKey(key)) {
            pendingKnot = raw;
            pendingLine = lines.number;
            havePending = true;
            break;
        }

        const auto once = [&](std::optional<std::size_t>& seen) -> std::expected<void, gr::Error> {
            if (seen.has_value()) {
                return trajectoryRefuse(lines.number, std::format("header key `{}` was already given on line {}", key, *seen), raw);
            }
            seen = lines.number;
            return {};
        };

        if (key == "columns") {
            if (const auto first = once(columnsLine); !first) {
                return std::unexpected(first.error());
            }
            std::string_view first;
            if (!fields.next(first) || first != "time") {
                return trajectoryRefuse(lines.number, "`columns` must begin with `time`", raw);
            }
            std::string_view name;
            while (fields.next(name)) {
                TrajectoryQuantity quantity{};
                if (name == "offset_hz") {
                    quantity = TrajectoryQuantity::offset_hz;
                } else if (name == "range_m") {
                    quantity = TrajectoryQuantity::range_m;
                } else if (name == "range_rate_m_s") {
                    quantity = TrajectoryQuantity::range_rate_m_s;
                } else {
                    return trajectoryRefuse(lines.number, std::format("`columns` names the unknown quantity `{}`", name), raw);
                }
                if (std::ranges::find(quantities, quantity) != quantities.end()) {
                    return trajectoryRefuse(lines.number, std::format("`columns` names `{}` twice", name), raw);
                }
                // The two are the same quantity through `offsetFor`. A file with both holds either a redundancy
                // or a contradiction. No rule can pick the right one.
                const bool clash = (quantity == TrajectoryQuantity::offset_hz && std::ranges::find(quantities, TrajectoryQuantity::range_rate_m_s) != quantities.end()) //
                                   || (quantity == TrajectoryQuantity::range_rate_m_s && std::ranges::find(quantities, TrajectoryQuantity::offset_hz) != quantities.end());
                if (clash) {
                    return trajectoryRefuse(lines.number, "`columns` names both `offset_hz` and `range_rate_m_s`, which are one quantity in two units", raw);
                }
                quantities.push_back(quantity);
            }
            if (quantities.empty()) {
                return trajectoryRefuse(lines.number, "`columns` names no quantity beyond `time`, so the table carries nothing", raw);
            }
        } else if (key == "time_scale") {
            if (const auto first = once(timeScaleLine); !first) {
                return std::unexpected(first.error());
            }
            std::string_view scale;
            if (!fields.next(scale) || (scale != "utc_iso8601" && scale != "unix_ns")) {
                return trajectoryRefuse(lines.number, "`time_scale` is `utc_iso8601` or `unix_ns`", raw);
            }
            if (std::string_view extra; fields.next(extra)) {
                return trajectoryRefuse(lines.number, "`time_scale` carries a field after its value", raw);
            }
            isoTime = scale == "utc_iso8601";
        } else if (key == "carrier_hz") {
            if (const auto first = once(carrierLine); !first) {
                return std::unexpected(first.error());
            }
            std::string_view value;
            if (!fields.next(value)) {
                return trajectoryRefuse(lines.number, "`carrier_hz` carries no value", raw);
            }
            const auto carrier = detail::trajectoryValue(lines.number, value, "carrier_hz");
            if (!carrier) {
                return std::unexpected(carrier.error());
            }
            if (std::string_view extra; fields.next(extra)) {
                return trajectoryRefuse(lines.number, "`carrier_hz` carries a field after its value", raw);
            }
            carrierHz = *carrier;
            if (!(carrierHz > 0.)) {
                return trajectoryRefuse(lines.number, std::format("`carrier_hz` is {}, and a carrier is positive", carrierHz), raw);
            }
        } else if (key == "source") {
            if (const auto first = once(sourceLine); !first) {
                return std::unexpected(first.error());
            }
            source = std::string(fields.rest());
        } else {
            return trajectoryRefuse(lines.number, std::format("unknown header key `{}`", key), raw);
        }
    }

    const std::size_t firstKnotLine = havePending ? pendingLine : lines.number + 1UZ;
    if (!columnsLine.has_value()) {
        return trajectoryRefuse(firstKnotLine, "the header block ended without a `columns` key", "");
    }
    if (!timeScaleLine.has_value()) {
        return trajectoryRefuse(firstKnotLine, "the header block ended without a `time_scale` key", "");
    }
    const bool needsCarrier = std::ranges::find(quantities, TrajectoryQuantity::range_rate_m_s) != quantities.end();
    if (needsCarrier && !carrierLine.has_value()) {
        return trajectoryRefuse(firstKnotLine, "`columns` names `range_rate_m_s`, which needs a `carrier_hz` header to become a frequency", "");
    }

    const std::size_t nFields = quantities.size() + 1UZ;

    std::vector<std::int64_t> times;
    std::vector<double>       offsets;
    std::vector<double>       delays;
    std::size_t               previousLine = 0UZ;

    const auto knot = [&](std::size_t number, std::string_view line) -> std::expected<void, gr::Error> {
        detail::TrajectoryFields fields{line};
        std::string_view         field;
        std::size_t              count = 0UZ;
        std::string_view         seen[4];
        while (fields.next(field)) {
            if (count < 4UZ) {
                seen[count] = field;
            }
            ++count;
        }
        if (count != nFields) {
            return trajectoryRefuse(number, std::format("{} fields against the {} `columns` declares", count, nFields), line);
        }

        const std::expected<std::int64_t, gr::Error> time = isoTime ? detail::trajectoryIso8601Ns(number, seen[0]) : detail::trajectoryUnixNs(number, seen[0]);
        if (!time) {
            return std::unexpected(time.error());
        }
        const std::int64_t t_ns = *time;
        if (!times.empty() && t_ns <= times.back()) {
            return trajectoryRefuse(number, std::format("the time {} ns is at or before line {}'s {} ns — times must strictly increase", t_ns, previousLine, times.back()), line);
        }
        times.push_back(t_ns);

        for (std::size_t i = 0UZ; i < quantities.size(); ++i) {
            const auto parsed = detail::trajectoryValue(number, seen[i + 1UZ], detail::trajectoryQuantityName(quantities[i]));
            if (!parsed) {
                return std::unexpected(parsed.error());
            }
            const double value = *parsed;
            switch (quantities[i]) {
            case TrajectoryQuantity::offset_hz: offsets.push_back(value); break;
            case TrajectoryQuantity::range_rate_m_s: {
                const double offset = offsetFor(value, carrierHz);
                if (!std::isfinite(offset)) {
                    return trajectoryRefuse(number, std::format("`range_rate_m_s` {} at the {} Hz carrier gives an offset that is not finite", value, carrierHz), line);
                }
                offsets.push_back(offset);
                break;
            }
            case TrajectoryQuantity::range_m:
                if (value < 0.) {
                    return trajectoryRefuse(number, std::format("`range_m` is {}, and a slant range is not negative", value), line);
                }
                delays.push_back(delayFor(value));
                break;
            }
        }
        previousLine = number;
        return {};
    };

    if (havePending) {
        if (const auto first = knot(pendingLine, pendingKnot); !first) {
            return std::unexpected(first.error());
        }
    }
    while (lines.next(raw)) {
        if (const auto bytes = detail::trajectoryCheckBytes(lines.number, raw); !bytes) {
            return std::unexpected(bytes.error());
        }
        detail::TrajectoryFields fields{raw};
        std::string_view         first;
        if (!fields.next(first) || first.front() == '#') {
            continue;
        }
        if (const auto added = knot(lines.number, raw); !added) {
            return std::unexpected(added.error());
        }
    }

    if (times.size() < PiecewiseLinearSchedule::kMinKnots || times.size() > PiecewiseLinearSchedule::kMaxKnots) {
        return trajectoryRefuse(lines.number, std::format("{} knots — the table holds between {} and {}", times.size(), PiecewiseLinearSchedule::kMinKnots, PiecewiseLinearSchedule::kMaxKnots), "");
    }

    Trajectory out;
    out.carrier_hz = carrierHz;
    out.source     = std::move(source);
    try {
        // The schedule constructors throw for the tables the checks above refuse. The catch returns such a throw as
        // the error.
        if (!offsets.empty()) {
            out.frequency.emplace(times, offsets);
        }
        if (!delays.empty()) {
            out.delay.emplace(times, delays);
        }
    } catch (const gr::exception& refused) {
        return std::unexpected(gr::Error(refused));
    }
    return out;
}

/// @brief The same, from a file. The message includes the path, and a failure names the file it tried to open.
[[nodiscard]] inline std::expected<Trajectory, gr::Error> loadTrajectoryFile(const std::filesystem::path& path, std::source_location loc = std::source_location::current()) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        return std::unexpected(gr::Error(std::format("gr::timing::loadTrajectoryFile: cannot read `{}`", path.string()), loc));
    }
    std::ostringstream text;
    text << stream.rdbuf();
    return loadTrajectory(text.str()).transform_error([&path, loc](const gr::Error& error) { return gr::Error(std::format("gr::timing::loadTrajectoryFile: `{}`: {}", path.string(), error.message), loc); });
}

/**
 * @brief The largest radial velocity the two halves of one trajectory disagree by, in meters per second.
 *
 * A file may carry both a range column and a range-rate column. They are independent quantities and can
 * disagree. The loader does not compare the two. This function does and returns the largest disagreement.
 *
 * Both schedules are evaluated at every knot time of both. The frequency schedule gives `-offset*c/f_c`. The
 * delay schedule gives `c * dtau/dt` on the segment that holds that time. The function returns the largest
 * absolute difference. The caller applies the tolerance that suits its producer.
 *
 * A consistent pair does not give zero. The size of the figure follows from the arithmetic. Take a
 * piecewise-linear table sampled every `h` seconds from a curve whose second derivative is bounded by `a`. Its
 * secant slope differs from the true derivative by at most `a*h/2`. A consistent pair stays within this bound.
 * A carrier that is not positive and finite is an error.
 */
[[nodiscard]] inline std::expected<double, gr::Error> worstRangeRateMismatch(const DelaySchedule& delay, const FrequencySchedule& frequency, double carrierHz, std::source_location loc = std::source_location::current()) {
    if (!(carrierHz > 0.) || !std::isfinite(carrierHz)) {
        return std::unexpected(gr::Error(std::format("gr::timing::worstRangeRateMismatch: carrier {} Hz — the carrier must be positive and finite", carrierHz), loc));
    }

    double worst = 0.;
    for (const std::span<const std::int64_t> knots : {delay.times(), frequency.times()}) {
        for (const std::int64_t t_ns : knots) {
            const double fromFrequency = -frequency.offsetAt(t_ns) * kSpeedOfLight / carrierHz;
            const double fromDelay     = kSpeedOfLight * delay.segmentSlopePerSecond(t_ns);
            worst                      = std::max(worst, std::abs(fromFrequency - fromDelay));
        }
    }
    return worst;
}

} // namespace gr::timing

#endif // GNURADIO_ALGORITHM_TRAJECTORY_FILE_HPP
