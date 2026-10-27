#ifndef GNURADIO_MEASUREMENT_PHASE_UNWRAP_HPP
#define GNURADIO_MEASUREMENT_PHASE_UNWRAP_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
#include <expected>
#include <format>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/measurement/PhaseUnwrap.hpp>

namespace gr::blocks::measurement {

namespace detail {
/// @brief The validator of `max_step_fraction`: true above 0.
inline constexpr auto kMaxStepFractionPositive = [](double value) { return value > 0.0; };
} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::measurement::PhaseUnwrap)

/**
 * @brief Unwraps the phase of a complex stream into an exact `int64` cycle count and a `float` residual.
 *
 * The block tracks the phase of a complex stream over a run of any length. For each input sample it writes the whole
 * turns since the origin to `cycles` and the residual phase on `[-pi, pi)` to `phase`. It never adds the two. The
 * count stays exact, and the residual keeps a `float`'s precision. `unwrappedRadians()` adds them in `double` for the
 * current sample. The first call publishes a `gr:signal_unit` tag on each output, an empty unit on `cycles` and `rad`
 * on `phase`.
 *
 * The unwrap is correct while the true phase advances by less than `pi` per sample (`|f| < fs/2`). A step whose
 * magnitude exceeds `max_step_fraction * pi` is suspect. A sample with a non-finite component repeats the current
 * count and residual. The next finite sample takes the shortest step from the last finite one, and that step counts as
 * suspect whatever its size.
 *
 * `origin` 'first_sample' reports the first sample's own phase, and 'zero' subtracts the first sample's phase from
 * every phase. A change of `origin` sets the count to zero, and the next sample sets the origin again. A change of
 * `max_step_fraction` applies from the next sample. With `reset_on_discontinuity` set, a `gr:n_dropped_samples` tag
 * sets the count to zero at the tagged sample, and that sample sets the origin again.
 *
 * `nSuspectSteps()` counts the suspect steps, and `lastSuspectIndex()` gives the stream index of the last one.
 * `nNonFiniteSamples()` counts the samples with a non-finite component, and `nResets()` the resets of either cause.
 * `nSaturations()` counts the turns lost at the `int64` limit of the count.
 */
struct PhaseUnwrap : Block<PhaseUnwrap> {
    using Description = Doc<R""(
@brief Unwraps the phase of a complex stream into an exact int64 cycle count and a float residual on [-pi, pi).

The block writes one cycle count and one residual per input sample on two ports and never adds them.
`nSuspectSteps()` counts the steps whose magnitude exceeds `max_step_fraction * pi` and each step across non-finite
samples. With `reset_on_discontinuity` set, a `gr:n_dropped_samples` tag sets the count to zero.
)"">;

    PortIn<std::complex<float>> in;
    PortOut<std::int64_t>       cycles;
    PortOut<float>              phase;

    Annotated<std::string, "origin", Visible, Doc<"phase origin, 'first_sample' or 'zero'">>                                                                          origin                 = std::string("first_sample");
    Annotated<double, "max_step_fraction", Visible, Doc<"suspect-step threshold as a fraction of pi, in (0, 1]">, Limits<0.0, 1.0, detail::kMaxStepFractionPositive>> max_step_fraction      = 0.9;
    Annotated<bool, "reset_on_discontinuity", Visible, Doc<"reset the cycle count at a gr:n_dropped_samples tag">>                                                    reset_on_discontinuity = true;

    GR_MAKE_REFLECTABLE(PhaseUnwrap, in, cycles, phase, origin, max_step_fraction, reset_on_discontinuity);

    static constexpr double kPi    = std::numbers::pi_v<double>;
    static constexpr double kTwoPi = 2. * std::numbers::pi_v<double>;

    gr::measurement::CycleUnwrapper _unwrapper{};
    gr::measurement::UnwrapOrigin   _origin{gr::measurement::UnwrapOrigin::first_sample};
    double                          _maxStepFraction{0.9};
    bool                            _resetOnDiscontinuity{true};

    bool          _havePrev{false};
    bool          _inGap{false}; ///< the samples since the last finite one are non-finite
    double        _prevCycles{0.};
    double        _prevPhase{0.};
    std::uint64_t _streamAt{0ULL};

    std::atomic<std::uint64_t> _nSuspect{0ULL};
    std::atomic<std::uint64_t> _nResets{0ULL};
    std::atomic<std::uint64_t> _nNonFinite{0ULL};
    std::atomic<std::int64_t>  _lastSuspect{-1LL}; ///< -1 until the first suspect step

    bool _taggedUnits{false}; ///< the `gr:signal_unit` tags are published

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        gr::measurement::UnwrapOrigin nextOrigin = _origin;
        if (newSettings.contains("origin")) {
            const auto parsed = gr::measurement::unwrapOriginFrom(origin.value);
            if (!parsed.has_value()) {
                throw gr::exception(std::format("origin must be 'first_sample' or 'zero', got '{}'", origin.value));
            }
            nextOrigin = *parsed;
        }
        const bool changesFraction = newSettings.contains("max_step_fraction");

        if (nextOrigin != _origin) {
            // A new origin sets the count to zero. The next sample sets the origin again.
            _origin    = nextOrigin;
            _unwrapper = gr::measurement::CycleUnwrapper(_origin, 1.0);
            _havePrev  = false;
            _inGap     = false;
            _nResets.fetch_add(1ULL, std::memory_order_relaxed);
        }
        if (changesFraction) {
            _maxStepFraction = max_step_fraction.value;
        }
        if (newSettings.contains("reset_on_discontinuity")) {
            _resetOnDiscontinuity = reset_on_discontinuity.value;
        }
    }

    void start() {
        // The kernel's threshold is `pi`, and no wrapped step exceeds it. The block tests the live `max_step_fraction`
        // against the kernel's output.
        _unwrapper = gr::measurement::CycleUnwrapper(_origin, 1.0);
        _havePrev  = false;
        _inGap     = false;
        _streamAt  = 0ULL;
        _nSuspect.store(0ULL, std::memory_order_relaxed);
        _nResets.store(0ULL, std::memory_order_relaxed);
        _nNonFinite.store(0ULL, std::memory_order_relaxed);
        _lastSuspect.store(-1LL, std::memory_order_relaxed);
        _taggedUnits = false;
    }

    /// @brief The unwrapped phase of the current sample in radians. Read it on the thread that runs the block.
    [[nodiscard]] double unwrappedRadians() const noexcept { return _unwrapper.unwrappedRadians(); }

    [[nodiscard]] std::uint64_t                nSuspectSteps() const noexcept { return _nSuspect.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t                nResets() const noexcept { return _nResets.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t                nSaturations() const noexcept { return _unwrapper.nSaturations(); }
    [[nodiscard]] std::uint64_t                nNonFiniteSamples() const noexcept { return _nNonFinite.load(std::memory_order_relaxed); }
    [[nodiscard]] std::optional<std::uint64_t> lastSuspectIndex() const noexcept {
        const std::int64_t value = _lastSuspect.load(std::memory_order_relaxed);
        return value < 0 ? std::nullopt : std::optional<std::uint64_t>(static_cast<std::uint64_t>(value));
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& cyclesSpan, OutputSpanLike auto& phaseSpan) {
        const std::size_t n = std::min({inSpan.size(), cyclesSpan.size(), phaseSpan.size()});
        if (n == 0UZ) {
            cyclesSpan.publish(0UZ);
            phaseSpan.publish(0UZ);
            std::ignore = inSpan.consume(0UZ);
            return inSpan.size() > 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
        }

        if (!_taggedUnits) {
            cyclesSpan.publishTag(property_map{{std::pmr::string(gr::tag::SIGNAL_UNIT.key()), pmt::Value(std::string(""))}}, 0UZ);
            phaseSpan.publishTag(property_map{{std::pmr::string(gr::tag::SIGNAL_UNIT.key()), pmt::Value(std::string("rad"))}}, 0UZ);
            _taggedUnits = true;
        }

        const std::uint64_t base = _streamAt;

        std::vector<std::size_t> resetOffsets;
        if (_resetOnDiscontinuity) {
            for (const gr::Tag& tag : inSpan.rawTags) {
                const std::uint64_t at = static_cast<std::uint64_t>(tag.index);
                if (at < base || at >= base + n) {
                    continue;
                }
                if (tag.map.find(gr::tag::N_DROPPED_SAMPLES.key()) == tag.map.end()) {
                    continue;
                }
                const std::uint64_t offset = at - base;
                resetOffsets.push_back(static_cast<std::size_t>(offset));
            }
            std::ranges::sort(resetOffsets);
            resetOffsets.erase(std::ranges::unique(resetOffsets).begin(), resetOffsets.end());
        }

        std::span<const std::complex<float>> input(inSpan.data(), n);
        std::span<std::int64_t>              cyclesOut(cyclesSpan.data(), n);
        std::span<float>                     phaseOut(phaseSpan.data(), n);

        std::size_t at   = 0UZ;
        const auto  fail = [&](const gr::Error& error) {
            cyclesSpan.publish(0UZ);
            phaseSpan.publish(0UZ);
            std::ignore = inSpan.consume(0UZ);
            this->emitErrorMessage("processBulk", error);
            return work::Status::ERROR;
        };
        for (const std::size_t offset : resetOffsets) {
            if (offset > at) {
                if (const auto done = processSlice(input.subspan(at, offset - at), cyclesOut.subspan(at, offset - at), phaseOut.subspan(at, offset - at), base + at); !done) {
                    return fail(done.error());
                }
                at = offset;
            }
            _unwrapper.markDiscontinuity();
            _nResets.fetch_add(1ULL, std::memory_order_relaxed);
            _havePrev = false;
            _inGap    = false;
        }
        if (at < n) {
            if (const auto done = processSlice(input.subspan(at), cyclesOut.subspan(at), phaseOut.subspan(at), base + at); !done) {
                return fail(done.error());
            }
        }

        _streamAt += n;
        cyclesSpan.publish(n);
        phaseSpan.publish(n);
        std::ignore = inSpan.consume(n);
        return work::Status::OK;
    }

private:
    [[nodiscard]] std::expected<void, gr::Error> processSlice(std::span<const std::complex<float>> input, std::span<std::int64_t> cyclesOut, std::span<float> phaseOut, std::uint64_t baseIndex) {
        const std::uint64_t nonFiniteBefore = _unwrapper.nNonFiniteSamples();
        if (auto unwrapped = _unwrapper.process(input, cyclesOut, phaseOut); !unwrapped) {
            return unwrapped;
        }
        _nNonFinite.fetch_add(_unwrapper.nNonFiniteSamples() - nonFiniteBefore, std::memory_order_relaxed);

        const double threshold = _maxStepFraction * kPi;
        for (std::size_t k = 0UZ; k < input.size(); ++k) {
            if (!std::isfinite(input[k].real()) || !std::isfinite(input[k].imag())) {
                _inGap = _havePrev;
                continue;
            }
            const double curCycles = static_cast<double>(cyclesOut[k]);
            const double curPhase  = static_cast<double>(phaseOut[k]);
            if (_havePrev) {
                const double step = kTwoPi * (curCycles - _prevCycles) + (curPhase - _prevPhase);
                if (_inGap || std::abs(step) > threshold) {
                    _nSuspect.fetch_add(1ULL, std::memory_order_relaxed);
                    _lastSuspect.store(static_cast<std::int64_t>(baseIndex + k), std::memory_order_relaxed);
                }
            }
            _inGap      = false;
            _havePrev   = true;
            _prevCycles = curCycles;
            _prevPhase  = curPhase;
        }
        return {};
    }
};

} // namespace gr::blocks::measurement

#endif // GNURADIO_MEASUREMENT_PHASE_UNWRAP_HPP
