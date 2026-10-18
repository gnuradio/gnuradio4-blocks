#ifndef GNURADIO_ALGORITHM_PHASE_UNWRAP_HPP
#define GNURADIO_ALGORITHM_PHASE_UNWRAP_HPP

#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <limits>
#include <numbers>
#include <optional>
#include <source_location>
#include <span>
#include <string_view>

#include <gnuradio-4.0/Message.hpp>

namespace gr::measurement {

/// Where the count and the reported phase start.
enum class UnwrapOrigin : std::uint8_t {
    first_sample, ///< the count starts at zero, and the first phase is the first sample's own
    zero          ///< the count starts at zero, and the first sample's phase is subtracted from every phase
};

/// @brief The setting text of an origin, `first_sample` or `zero`.
[[nodiscard]] inline constexpr std::string_view unwrapOriginName(UnwrapOrigin origin) noexcept { return origin == UnwrapOrigin::zero ? "zero" : "first_sample"; }

/// @brief The origin named by a setting's text, `first_sample` or `zero`, or no value for any other text.
[[nodiscard]] inline constexpr std::optional<UnwrapOrigin> unwrapOriginFrom(std::string_view name) noexcept {
    if (name == "first_sample") {
        return UnwrapOrigin::first_sample;
    }
    if (name == "zero") {
        return UnwrapOrigin::zero;
    }
    return std::nullopt;
}

/**
 * @brief Unwraps the phase of a complex stream into a whole-turn count and a residual on `[-pi, pi)`.
 *
 * `process()` takes complex samples and writes, for each one, the whole turns since the origin as an `std::int64_t`
 * and the residual phase as a `float` on `[-pi, pi)`. It never adds the two. The count is exact, and the residual
 * keeps a `float`'s spacing near `pi`, at most `2.38e-7 rad`, for any run length. At `10^9` cycles a `float` sum of
 * total radians has a spacing of 81.5 cycles, and a `double` sum one of `9.54e-7 rad`. `unwrappedRadians()` adds the
 * two parts in `double` for the current sample. `process()` returns a `gr::Error` and processes nothing when an output
 * span is shorter than the input.
 *
 * The unwrap is correct while the true phase advances by less than `pi` per sample, that is while `|f| < fs/2`. At or
 * above that bound two turn counts give the same wrapped difference, and the count follows the alias. A sample with a
 * non-finite component has no phase. `process()` repeats the current count and residual for it, and the next finite
 * sample takes the shortest step from the last finite one. Over a gap in which the phase moves by `pi` or more, that
 * step takes the alias.
 *
 * The origin sets where the reported phase starts, as `UnwrapOrigin` states. `maxStepFraction`, in `(0, 1]`, sets the
 * threshold of a suspect step, and the constructor throws `gr::exception` outside that range. `markDiscontinuity()`
 * returns the count to zero, and the next sample sets the origin again.
 *
 * `nSuspectSteps()` counts the steps whose magnitude exceeds `maxStepFraction * pi` and every step across a gap.
 * `lastSuspectIndex()` gives the sample index of the last one. `nNonFiniteSamples()` counts the samples with a
 * non-finite component, and `nResets()` the calls to `markDiscontinuity()`. A turn past the `std::int64_t` limit
 * leaves the count at the limit, and `nSaturations()` counts it. At `fs/2` and 1 MS/s the limit lies 584 554 years
 * away. `reset()` clears the origin, the count and every counter.
 */
class CycleUnwrapper {
public:
    static constexpr float  kPi    = std::numbers::pi_v<float>;
    static constexpr double kTwoPi = 2. * std::numbers::pi_v<double>;

    CycleUnwrapper() = default;

    /// @brief Throws `gr::exception` when @p maxStepFraction is not in `(0, 1]`.
    explicit CycleUnwrapper(UnwrapOrigin origin, double maxStepFraction = 0.9) : _origin(origin), _maxStepFraction(maxStepFraction) {
        if (!(maxStepFraction > 0.) || !(maxStepFraction <= 1.)) {
            throw gr::exception(std::format("gr::measurement::CycleUnwrapper: max step fraction {} is not in (0, 1]", maxStepFraction));
        }
    }

    /// @brief Clears the origin, the count and every counter.
    void reset() noexcept {
        _started   = false;
        _cycles    = 0LL;
        _phase     = 0.f;
        _base      = 0.f;
        _index     = 0ULL;
        _suspect   = 0ULL;
        _resets    = 0ULL;
        _saturate  = 0ULL;
        _nonFinite = 0ULL;
        _inGap     = false;
        _lastSuspect.reset();
    }

    /**
     * @brief Marks a gap in the stream. The count returns to zero, and the next sample sets the origin again.
     *
     * Adds one to `nResets()`. The other counters and the sample index keep their values, where `reset()` clears
     * them.
     */
    void markDiscontinuity() noexcept {
        _started = false;
        _inGap   = false;
        _cycles  = 0LL;
        _phase   = 0.f;
        _base    = 0.f;
        ++_resets;
    }

    /**
     * @brief Writes one `(cycles, phase)` pair per input sample.
     *
     * @param in     the complex stream
     * @param cycles whole turns since the origin
     * @param phase  the residual, in radians on `[-pi, pi)`
     *
     * Returns a `gr::Error` and processes nothing when an output span is shorter than the input.
     */
    [[nodiscard]] std::expected<void, gr::Error> process(std::span<const std::complex<float>> in, std::span<std::int64_t> cycles, std::span<float> phase, std::source_location loc = std::source_location::current()) {
        if (cycles.size() < in.size() || phase.size() < in.size()) {
            return std::unexpected(gr::Error(std::format("gr::measurement::CycleUnwrapper::process: {} samples against {} cycles and {} phases", in.size(), cycles.size(), phase.size()), loc));
        }

        const float suspect = static_cast<float>(_maxStepFraction) * kPi;
        for (std::size_t k = 0UZ; k < in.size(); ++k) {
            if (!std::isfinite(in[k].real()) || !std::isfinite(in[k].imag())) {
                ++_nonFinite;
                _inGap    = _started;
                cycles[k] = _cycles;
                phase[k]  = _phase;
                ++_index;
                continue;
            }
            const float raw = std::atan2(in[k].imag(), in[k].real());
            if (!_started) {
                _started = true;
                _base    = (_origin == UnwrapOrigin::zero) ? raw : 0.f;
                _phase   = wrap(raw - _base);
            } else {
                const float current = wrap(raw - _base);
                const float delta   = current - _phase;
                const float step    = (delta > kPi) ? (delta - 2.f * kPi) : ((delta < -kPi) ? (delta + 2.f * kPi) : delta);
                if (delta > kPi) {
                    turn(-1LL);
                } else if (delta < -kPi) {
                    turn(+1LL);
                }
                if (_inGap || std::abs(step) > suspect) {
                    ++_suspect;
                    _lastSuspect = _index;
                }
                _inGap = false;
                _phase = current;
            }
            cycles[k] = _cycles;
            phase[k]  = _phase;
            ++_index;
        }
        return {};
    }

    [[nodiscard]] std::int64_t cycles() const noexcept { return _cycles; }
    [[nodiscard]] float        phase() const noexcept { return _phase; }

    /// @brief `2*pi*cycles + phase` of the current sample in `double`.
    [[nodiscard]] double unwrappedRadians() const noexcept { return kTwoPi * static_cast<double>(_cycles) + static_cast<double>(_phase); }

    [[nodiscard]] bool                         started() const noexcept { return _started; }
    [[nodiscard]] UnwrapOrigin                 origin() const noexcept { return _origin; }
    [[nodiscard]] double                       maxStepFraction() const noexcept { return _maxStepFraction; }
    [[nodiscard]] std::uint64_t                processed() const noexcept { return _index; }
    [[nodiscard]] std::uint64_t                nSuspectSteps() const noexcept { return _suspect; }
    [[nodiscard]] std::uint64_t                nResets() const noexcept { return _resets; }
    [[nodiscard]] std::uint64_t                nSaturations() const noexcept { return _saturate; }
    [[nodiscard]] std::uint64_t                nNonFiniteSamples() const noexcept { return _nonFinite; }
    [[nodiscard]] std::optional<std::uint64_t> lastSuspectIndex() const noexcept { return _lastSuspect; }

private:
    /// `atan2` returns `[-pi, pi]`. Under the `zero` origin, `raw - _base` spans `[-2*pi, 2*pi]`.
    [[nodiscard]] static float wrap(float radians) noexcept {
        float out = radians;
        while (out >= kPi) {
            out -= 2.f * kPi;
        }
        while (out < -kPi) {
            out += 2.f * kPi;
        }
        return out;
    }

    void turn(std::int64_t by) noexcept {
        if ((by > 0 && _cycles == std::numeric_limits<std::int64_t>::max()) || (by < 0 && _cycles == std::numeric_limits<std::int64_t>::min())) {
            ++_saturate;
            return;
        }
        _cycles += by;
    }

    UnwrapOrigin                 _origin{UnwrapOrigin::first_sample};
    double                       _maxStepFraction{0.9};
    bool                         _started{false};
    std::int64_t                 _cycles{0LL};
    float                        _phase{0.f};
    float                        _base{0.f};
    std::uint64_t                _index{0ULL};
    std::uint64_t                _suspect{0ULL};
    std::uint64_t                _resets{0ULL};
    std::uint64_t                _saturate{0ULL};
    std::uint64_t                _nonFinite{0ULL};
    bool                         _inGap{false}; ///< a non-finite sample followed the last finite one
    std::optional<std::uint64_t> _lastSuspect{};
};

} // namespace gr::measurement

#endif // GNURADIO_ALGORITHM_PHASE_UNWRAP_HPP
