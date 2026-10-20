#ifndef GNURADIO_ALGORITHM_SAMPLE_CLOCK_HPP
#define GNURADIO_ALGORITHM_SAMPLE_CLOCK_HPP

#include <cmath>
#include <cstdint>
#include <expected>
#include <format>
#include <limits>
#include <source_location>

#include <gnuradio-4.0/Message.hpp>

namespace gr::timing {

namespace detail {

/**
 * @brief A 128-bit unsigned magnitude in two 64-bit limbs.
 *
 * The exact products this header needs exceed 64 bits and must not be rounded. The build uses
 * `-Wpedantic -Werror`, and `__int128` is not ISO C++. Only the operations the clock performs are provided. This is
 * not a general wide-integer type.
 */
struct U128 {
    std::uint64_t hi{0ULL};
    std::uint64_t lo{0ULL};
};

/// The exact 64x64 product, schoolbook over 32-bit halves.
[[nodiscard]] constexpr U128 mul(std::uint64_t a, std::uint64_t b) noexcept {
    constexpr std::uint64_t kMask = 0xFFFFFFFFULL;

    const std::uint64_t aLo = a & kMask;
    const std::uint64_t aHi = a >> 32;
    const std::uint64_t bLo = b & kMask;
    const std::uint64_t bHi = b >> 32;

    const std::uint64_t ll     = aLo * bLo;
    const std::uint64_t lh     = aLo * bHi;
    const std::uint64_t hl     = aHi * bLo;
    const std::uint64_t hh     = aHi * bHi;
    const std::uint64_t middle = (ll >> 32) + (lh & kMask) + (hl & kMask);

    return U128{hh + (lh >> 32) + (hl >> 32) + (middle >> 32), (middle << 32) | (ll & kMask)};
}

[[nodiscard]] constexpr U128 add(U128 a, std::uint64_t b) noexcept {
    const std::uint64_t lo = a.lo + b;
    return U128{a.hi + (lo < a.lo ? 1ULL : 0ULL), lo};
}

/// `a - b`, where the caller has established that `a >= b`.
[[nodiscard]] constexpr U128 sub(U128 a, std::uint64_t b) noexcept { return U128{a.hi - (a.lo < b ? 1ULL : 0ULL), a.lo - b}; }

[[nodiscard]] constexpr bool less(U128 a, std::uint64_t b) noexcept { return a.hi == 0ULL && a.lo < b; }

struct DivResult {
    U128          quotient{};
    std::uint64_t remainder{0ULL};
};

/**
 * @brief `a / d` and `a % d`, exact, for `0 < d < 2^63`.
 *
 * The high limb divides first and leaves a remainder below `d`. The low limb then runs a 64-step
 * shift-subtract over that remainder. The shifted remainder needs no 65th bit, because `d` is below
 * `2^63`. `SampleClock` therefore validates both of its divisors against that bound. A clock divides
 * once per span and not once per sample. `NsWalk` is the per-sample path.
 */
[[nodiscard]] constexpr DivResult divide(U128 a, std::uint64_t d) noexcept {
    DivResult     out{};
    std::uint64_t remainder = 0ULL;
    if (a.hi != 0ULL) {
        out.quotient.hi = a.hi / d;
        remainder       = a.hi % d;
    }

    std::uint64_t quotient = 0ULL;
    for (int bit = 63; bit >= 0; --bit) {
        remainder = (remainder << 1) | ((a.lo >> bit) & 1ULL);
        quotient <<= 1;
        if (remainder >= d) {
            remainder -= d;
            quotient |= 1ULL;
        }
    }
    out.quotient.lo = quotient;
    out.remainder   = remainder;
    return out;
}

} // namespace detail

/// The whole sample and the exact fraction of one, as `indexOf` returns them for a time.
struct IndexAt {
    std::uint64_t index{0ULL};         ///< the last sample at or before the given time
    std::uint64_t remainder_num{0ULL}; ///< how far past that sample the time falls, in [0, remainder_den)
    std::uint64_t remainder_den{1ULL}; ///< 10^9 * rate_den, the exact denominator of that fraction

    /// The fraction as a `double`, for a caller whose arithmetic is already floating point. It is a number in
    /// [0, 1) and not an index or a nanosecond count. It needs none of the range the integers above protect.
    [[nodiscard]] double fraction() const noexcept { return static_cast<double>(remainder_num) / static_cast<double>(remainder_den); }

    [[nodiscard]] bool operator==(const IndexAt&) const noexcept = default;
};

/**
 * @brief The map between a sample index and a wall-clock time, exact in integers.
 *
 * A stream's sample index counts samples without drift. A timestamp is the common scale on which two streams
 * compare. This type converts between the two, exactly. The rate is an exact rational `rate_num/rate_den`. For
 * example, 250 kS/s is `250000/1`. An audio rate trimmed by 1 ppm is `44100044100/1000000`, which no `double`
 * sample rate can express. No index or nanosecond count passes through a `double` here. At `2^53 + 1` a `double`
 * index is already short by one. Every conversion below is integer arithmetic in 128 bits.
 *
 * The exact map, as a rational, is
 *
 *     t(index) = anchor_ns + ((index - anchor_index) * 10^9 * rate_den + anchor_rem) / rate_num
 *
 * `timeOf` is its **floor**, the last whole nanosecond at or before that sample. The
 * floor holds on both sides of the anchor. At 250 kS/s the sample one before the anchor reads `anchor_ns - 4000`.
 * A rate whose period is not a whole number of nanoseconds rounds down and not toward the anchor. `indexOf` is
 * the same floor in the other direction. It returns the last sample at or before a time. The sub-sample
 * remainder is returned exactly beside it, and the caller need not derive it again.
 *
 * `anchor_rem` is the anchor's position within its nanosecond, a numerator over `rate_num`. That remainder makes
 * `rebase` exact. Without it, moving the anchor to an index whose time is not a whole nanosecond would shift
 * every later conversion by up to one nanosecond. `rebase` moves the anchor and leaves the map unchanged. A clock
 * built by hand leaves `anchor_rem` at zero, which places the anchor exactly on its nanosecond.
 *
 * Every member function is const. `rebase` and `withRate` return a new clock. A member written directly skips the
 * constructor's checks. `withRate` starts a new rate at a stated index. A rate change is a new anchor at the change
 * point. The clock then represents the discontinuity at that sample. It does not spread the change over the
 * samples on either side.
 *
 * Time is nanoseconds since the Unix epoch. The kernel counts nanoseconds and applies no leap second or time zone.
 *
 * Domain. Every product below is exact while `|index - anchor_index| * 10^9 * rate_den` and
 * `|t_ns - anchor_ns| * rate_num` stay under `2^127`. The rate bounds and the `std::int64_t` nanosecond range
 * together guarantee this. The nanosecond result must itself fit `std::int64_t`, about +/-292 years around the
 * epoch.
 */
struct SampleClock {
    /// `10^9 * rate_den` is the numerator of a sample period in nanoseconds and the divisor of `indexOf`.
    /// `rate_num` is the divisor of `timeOf`. Both must stay below `2^63`, and `detail::divide` then needs no
    /// 65th bit. This caps the denominator at `(2^63 - 1) / 10^9`.
    static constexpr std::uint64_t kNsPerSecond = 1'000'000'000ULL;
    static constexpr std::uint64_t kMaxRateNum  = (1ULL << 63) - 1ULL;
    static constexpr std::uint64_t kMaxRateDen  = kMaxRateNum / kNsPerSecond;

    /// The steps per hertz of the grid `clockForRateHz` rounds a rate to, and the slowest and fastest rate it takes.
    /// The fastest rate is the largest whole hertz whose numerator on that grid stays within `kMaxRateNum`.
    static constexpr std::uint64_t kRateStepsPerHz = 1'000'000ULL;
    static constexpr double        kMinRateHz      = 1. / static_cast<double>(kRateStepsPerHz);
    static constexpr double        kMaxRateHz      = static_cast<double>(kMaxRateNum / kRateStepsPerHz);

    /// A caller keeps an anchor time within 2^62 ns of the epoch. Every time within 2^62 ns of such an anchor then
    /// fits the signed nanosecond axis. The positive bound is 2116-02-20T23:53:38.427387904Z.
    static constexpr std::int64_t kMaxAnchorNs = std::int64_t{1} << 62;

    std::uint64_t anchor_index{0ULL}; ///< the sample the anchor time belongs to
    std::int64_t  anchor_ns{0LL};     ///< nanoseconds since the Unix epoch, floored
    std::uint64_t rate_num{1ULL};     ///< samples per second, numerator
    std::uint64_t rate_den{1ULL};     ///< samples per second, denominator
    std::uint64_t anchor_rem{0ULL};   ///< the anchor's position within `anchor_ns` as a numerator over `rate_num`, in [0, rate_num)

    constexpr SampleClock() noexcept = default;

    SampleClock(std::uint64_t anchorIndex, std::int64_t anchorNs, std::uint64_t rateNum, std::uint64_t rateDen, std::uint64_t anchorRem = 0ULL) : anchor_index(anchorIndex), anchor_ns(anchorNs), rate_num(rateNum), rate_den(rateDen), anchor_rem(anchorRem) {
        if (rateNum == 0ULL || rateDen == 0ULL) {
            throw gr::exception(std::format("gr::timing::SampleClock: rate {}/{} — neither term may be zero", rateNum, rateDen));
        }
        if (rateNum > kMaxRateNum || rateDen > kMaxRateDen) {
            throw gr::exception(std::format("gr::timing::SampleClock: rate {}/{} is outside the exact domain — the numerator must not exceed {} and the denominator not {}", rateNum, rateDen, kMaxRateNum, kMaxRateDen));
        }
        if (anchorRem >= rateNum) {
            throw gr::exception(std::format("gr::timing::SampleClock: anchor remainder {} must be below the rate numerator {}", anchorRem, rateNum));
        }
    }

    [[nodiscard]] bool operator==(const SampleClock&) const noexcept = default;

    /// The numerator of one sample period in nanoseconds. The denominator is `rate_num`.
    [[nodiscard]] constexpr std::uint64_t periodNsNum() const noexcept { return kNsPerSecond * rate_den; }

    /// The rate as a `double`, for radian arithmetic that is floating point anyway. It is not for an index or
    /// a nanosecond count. The rational above serves those.
    [[nodiscard]] constexpr double rateHz() const noexcept { return static_cast<double>(rate_num) / static_cast<double>(rate_den); }

    /// The offset from `anchor_ns` split into whole nanoseconds and the exact remainder within the last one.
    struct NsSplit {
        std::int64_t  whole{0LL}; ///< nanoseconds relative to `anchor_ns`, floored
        std::uint64_t rem{0ULL};  ///< the position within the nanosecond as a numerator over `rate_num`, in [0, rate_num)
    };

    [[nodiscard]] NsSplit offsetFrom(std::uint64_t index) const noexcept {
        const std::uint64_t period = periodNsNum();
        if (index >= anchor_index) {
            const detail::DivResult split = detail::divide(detail::add(detail::mul(index - anchor_index, period), anchor_rem), rate_num);
            return NsSplit{static_cast<std::int64_t>(split.quotient.lo), split.remainder};
        }

        // Below the anchor the numerator is negative, and a floor is not a truncation. The quotient rounds away
        // from zero whenever the division leaves a remainder. The remainder gives the distance to the next whole
        // nanosecond up.
        const detail::U128 product = detail::mul(anchor_index - index, period);
        if (detail::less(product, anchor_rem)) {
            return NsSplit{0LL, anchor_rem - product.lo};
        }
        const detail::DivResult split = detail::divide(detail::sub(product, anchor_rem), rate_num);
        if (split.remainder == 0ULL) {
            return NsSplit{-static_cast<std::int64_t>(split.quotient.lo), 0ULL};
        }
        return NsSplit{-static_cast<std::int64_t>(split.quotient.lo) - 1LL, rate_num - split.remainder};
    }

    /// The last whole nanosecond at or before sample `index`.
    [[nodiscard]] std::int64_t timeOf(std::uint64_t index) const noexcept { return anchor_ns + offsetFrom(index).whole; }

    /// The last sample at or before `t_ns`, with the exact fraction of a sample by which the time passes it. A time
    /// before sample 0 or past sample `2^64 - 1` is an error.
    [[nodiscard]] std::expected<IndexAt, gr::Error> indexOf(std::int64_t t_ns, std::source_location loc = std::source_location::current()) const {
        const std::uint64_t period = periodNsNum();
        // The distance from the anchor needs up to 64 bits. Two's complement makes the unsigned difference exact.
        const bool          after    = t_ns >= anchor_ns;
        const std::uint64_t distance = after ? static_cast<std::uint64_t>(t_ns) - static_cast<std::uint64_t>(anchor_ns) : static_cast<std::uint64_t>(anchor_ns) - static_cast<std::uint64_t>(t_ns);

        detail::U128 numerator{};
        bool         negative = false;
        if (after) {
            const detail::U128 scaled = detail::mul(distance, rate_num);
            if (detail::less(scaled, anchor_rem)) {
                numerator = detail::U128{0ULL, anchor_rem - scaled.lo};
                negative  = true;
            } else {
                numerator = detail::sub(scaled, anchor_rem);
            }
        } else {
            numerator = detail::add(detail::mul(distance, rate_num), anchor_rem);
            negative  = true;
        }

        const detail::DivResult split = detail::divide(numerator, period);
        if (!negative) {
            if (split.quotient.hi != 0ULL || split.quotient.lo > std::numeric_limits<std::uint64_t>::max() - anchor_index) {
                return std::unexpected(gr::Error(std::format("gr::timing::SampleClock::indexOf: {} ns is past the last sample index {}", t_ns, std::numeric_limits<std::uint64_t>::max()), loc));
            }
            return IndexAt{anchor_index + split.quotient.lo, split.remainder, period};
        }
        const bool partial = split.remainder != 0ULL;
        if (split.quotient.hi != 0ULL || split.quotient.lo > anchor_index || (partial && split.quotient.lo == anchor_index)) {
            return std::unexpected(gr::Error(std::format("gr::timing::SampleClock::indexOf: {} ns is before sample 0 at {} ns", t_ns, timeOf(0ULL)), loc));
        }
        if (!partial) {
            return IndexAt{anchor_index - split.quotient.lo, 0ULL, period};
        }
        return IndexAt{anchor_index - split.quotient.lo - 1ULL, period - split.remainder, period};
    }

    /// The same map anchored at `index`. It is exact, because `anchor_rem` keeps what the floor drops.
    [[nodiscard]] SampleClock rebase(std::uint64_t index) const {
        const NsSplit at = offsetFrom(index);
        return SampleClock(index, anchor_ns + at.whole, rate_num, rate_den, at.rem);
    }

    /// @brief A new clock running at `rateNum/rateDen` from sample `atIndex`, anchored at that sample's time.
    ///
    /// The change index is a required argument. A rate change takes effect at the sample where it happens.
    /// Keeping the old anchor would apply the new rate to every sample since that anchor. The anchor's
    /// sub-nanosecond position is re-expressed over the new numerator and truncated. That is the one inexact step
    /// in this type. The anchor moves earlier by less than 1/rateNum of a nanosecond. A rate the constructor
    /// refuses is an error here.
    [[nodiscard]] std::expected<SampleClock, gr::Error> withRate(std::uint64_t rateNum, std::uint64_t rateDen, std::uint64_t atIndex, std::source_location loc = std::source_location::current()) const {
        if (rateNum == 0ULL || rateDen == 0ULL) {
            return std::unexpected(gr::Error(std::format("gr::timing::SampleClock::withRate: rate {}/{} — neither term may be zero", rateNum, rateDen), loc));
        }
        if (rateNum > kMaxRateNum || rateDen > kMaxRateDen) {
            return std::unexpected(gr::Error(std::format("gr::timing::SampleClock::withRate: rate {}/{} is outside the exact domain — the numerator must not exceed {} and the denominator not {}", rateNum, rateDen, kMaxRateNum, kMaxRateDen), loc));
        }
        const NsSplit       at       = offsetFrom(atIndex);
        const std::uint64_t rescaled = detail::divide(detail::mul(at.rem, rateNum), rate_num).quotient.lo;
        return SampleClock(atIndex, anchor_ns + at.whole, rateNum, rateDen, rescaled);
    }

    /**
     * @brief A forward walk over consecutive sample times, exact and division-free.
     *
     * `timeOf` costs a 128-bit division. A walk gives the same times over a span of samples without that
     * division per sample. Write the period as `period = whole * rate_num + carry`. The time of sample
     * `index + k` is then `timeOf(index) + k * whole`, plus one nanosecond each time an accumulator of `carry`
     * passes `rate_num`. A step is one addition and at most one subtraction. `advance()` reproduces
     * `timeOf(index + k)` exactly. A caller that splits a span into chunks reads the same times as one that
     * does not, because both are the same function of the absolute index.
     */
    struct NsWalk {
        std::int64_t  t_ns{0LL};     ///< `timeOf` of the sample the walk stands on
        std::uint64_t rem{0ULL};     ///< that sample's position within `t_ns`, over `modulus`
        std::uint64_t whole{0ULL};   ///< whole nanoseconds a sample advances
        std::uint64_t carry{0ULL};   ///< the advance beyond those, over `modulus`
        std::uint64_t modulus{1ULL}; ///< `rate_num`

        void advance() noexcept {
            t_ns += static_cast<std::int64_t>(whole);
            rem += carry;
            if (rem >= modulus) {
                rem -= modulus;
                ++t_ns;
            }
        }
    };

    [[nodiscard]] NsWalk walkFrom(std::uint64_t index) const noexcept {
        const NsSplit       at     = offsetFrom(index);
        const std::uint64_t period = periodNsNum();
        return NsWalk{anchor_ns + at.whole, at.rem, period / rate_num, period % rate_num, rate_num};
    }
};

static_assert(SampleClock::kMaxRateDen * SampleClock::kNsPerSecond <= SampleClock::kMaxRateNum, "10^9 * rate_den is a divisor and must stay inside the same bound as the numerator");

/**
 * @brief A clock at @p rateHz on the microhertz grid, anchored at (@p anchorIndex, @p anchorNs).
 *
 * This function brings a floating rate onto the exact time axis. The rate is rounded to a 10^-6 Hz grid.
 * After T seconds the rounding moves a sample's time by about 5e-7 * T / rateHz seconds, 54 ns after ten
 * hours at 333 kS/s. A rate outside `[SampleClock::kMinRateHz, SampleClock::kMaxRateHz]`, or a rate that is not
 * finite and positive, is an error. A caller can wrap the message with the name of its setting.
 */
[[nodiscard]] inline std::expected<SampleClock, gr::Error> clockForRateHz(double rateHz, std::uint64_t anchorIndex, std::int64_t anchorNs, std::source_location loc = std::source_location::current()) {
    if (!(rateHz >= SampleClock::kMinRateHz) || !(rateHz <= SampleClock::kMaxRateHz)) {
        return std::unexpected(gr::Error(std::format("gr::timing::clockForRateHz: {} Hz is outside the [{:g}, {:g}] Hz the exact time axis allows", rateHz, SampleClock::kMinRateHz, SampleClock::kMaxRateHz), loc));
    }
    return SampleClock(anchorIndex, anchorNs, static_cast<std::uint64_t>(std::llround(rateHz * static_cast<double>(SampleClock::kRateStepsPerHz))), SampleClock::kRateStepsPerHz);
}

} // namespace gr::timing

#endif // GNURADIO_ALGORITHM_SAMPLE_CLOCK_HPP
