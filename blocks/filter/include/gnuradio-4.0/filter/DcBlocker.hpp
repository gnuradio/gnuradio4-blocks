#ifndef GNURADIO_DC_BLOCKER_HPP
#define GNURADIO_DC_BLOCKER_HPP

#include <algorithm>
#include <array>
#include <complex>
#include <concepts>
#include <cstdint>
#include <format>
#include <limits>
#include <optional>
#include <span>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>
#include <gnuradio-4.0/filter/TagDelay.hpp>

namespace gr::blocks::filter {

namespace detail {

template<typename T>
struct RealOf {
    using type = T;
};
template<typename TReal>
struct RealOf<std::complex<TReal>> {
    using type = TReal;
};

/// The real values in one sample: one for a real sample, two for a complex one.
template<typename T>
inline constexpr std::size_t kPartsOf = std::same_as<T, typename RealOf<T>::type> ? 1UZ : 2UZ;

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::filter::DcBlocker, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct DcBlocker : Block<DcBlocker<T>>, detail::DelayedTagFilter<DcBlocker<T>, T, T> {
    using Description = Doc<R""(
@brief Removes DC with a cascade of boxcar averages, a linear-phase notch with an exact integer delay.

The block removes a DC offset from a real or complex stream where a linear phase is worth a delay. It gives one sample
out for each one in, of the input's type, delayed by `group_delay` samples. Every tag moves with all its keys by the
same delay. A tag on input `i` leaves on output `i + group_delay`. A tag whose output lies past the stream's last output
leaves at the end-of-stream index, one past that output.

A cascade of boxcar averages of length `D` estimates DC. The block subtracts the estimate from a copy of the input
delayed by the cascade's group delay. The response is exactly `|1 - A(f)^stages|`, where `stages` is the number of
boxcars and `A(f) = sin(pi*D*f/fs) / (D*sin(pi*f/fs))`. It is zero at DC and returns to 1 at every multiple of `fs/D`.

`length` sets `D`, at least two, and a longer `length` narrows the notch. `long_form`, the default, takes four boxcars
and a delay of `2D-2`. The short form takes two boxcars and a delay of `D-1`. The long form is both the flatter and the
narrower of the two. At `D = 32` its -3 dB corner sits at `0.419 * fs/D`, against `0.573 * fs/D` for the short form. The
block refuses a change of `length` or `long_form` while it runs. A change while it is stopped rebuilds the filter with
an empty history. The block writes `group_delay` itself and refuses a different `group_delay` unless the same change
sets `length` or `long_form`.

Each boxcar's running sum is updated incrementally and recomputed exactly every `reseed_interval` samples, oldest sample
first. The recomputation stops a long float run from drifting without bound. A NaN or infinite input sample makes the
output non-finite until every boxcar has recomputed its sum after the value left its history. The output is finite again
fewer than `stages * (reseed_interval + length)` samples after that input. `reseed_interval` is at least one and may
change while the block runs. `reset()` empties the history. From an empty history the first `2 * group_delay` outputs
carry a start-up transient.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "length", Doc<"boxcar length D">, Limits<2U, std::numeric_limits<gr::Size_t>::max()>>                                           length          = 32U;
    Annotated<bool, "long_form", Doc<"four boxcars and delay 2D-2 when true, two and D-1 when false">>                                                    long_form       = true;
    Annotated<gr::Size_t, "reseed_interval", Doc<"samples between exact recomputations of the sums">, Limits<1U, std::numeric_limits<gr::Size_t>::max()>> reseed_interval = 4096U;
    Annotated<gr::Size_t, "group_delay", Doc<"derived pipeline delay in samples, D-1 or 2D-2">>                                                           group_delay     = 62U;

    GR_MAKE_REFLECTABLE(DcBlocker, in, out, length, long_form, reseed_interval, group_delay);

    using Real                          = typename detail::RealOf<T>::type;
    static constexpr std::size_t kParts = detail::kPartsOf<T>;

    std::vector<Real>              _history{}; ///< the last `D` inputs of each boxcar, boxcar after boxcar, `kParts` values a sample
    std::array<Real, 4UZ * kParts> _sums{};    ///< the running sum of each boxcar
    std::vector<Real>              _delay{};   ///< the delay line, `kParts` values a sample
    std::size_t                    _nStages     = 4UZ;
    std::size_t                    _length      = 32UZ; ///< `D`, the samples each boxcar holds
    std::size_t                    _delayLength = 62UZ; ///< the samples the delay line holds
    std::size_t                    _cursor      = 0UZ;  ///< the history slot every boxcar writes next
    std::size_t                    _delayCursor = 0UZ;  ///< the line slot of the sample due now
    Real                           _divisor{32};        ///< `D` in the sample's real type
    std::uint64_t                  _offset      = 0ULL; ///< samples filtered since the pipeline was built
    gr::Size_t                     _untilReseed = 0U;   ///< samples before the next exact recomputation of the sums

    bool       _running       = false;
    gr::Size_t _builtLength   = length.value;          ///< the accepted `length`, which the pipeline is built for
    bool       _builtLongForm = long_form.value;       ///< the accepted `long_form`, which the pipeline is built for
    gr::Size_t _interval      = reseed_interval.value; ///< the accepted `reseed_interval`

    /// Builds the pipeline for the default settings.
    explicit DcBlocker(property_map init = {}) : Block<DcBlocker<T>>(std::move(init)) { build(_builtLength, _builtLongForm); }

    void start() {
        _running = true;
        this->tagsStart();
    }

    void stop() { _running = false; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const bool       shapeWritten = newSettings.contains("length") || newSettings.contains("long_form");
        const gr::Size_t builtDelay   = static_cast<gr::Size_t>(_delayLength);
        if (!shapeWritten && newSettings.contains("group_delay") && group_delay.value != builtDelay) {
            throw gr::exception(std::format("group_delay follows from length and long_form, which give {}", builtDelay));
        }
        const gr::Size_t nextLength   = newSettings.contains("length") ? length.value : _builtLength;
        const bool       nextLongForm = newSettings.contains("long_form") ? long_form.value : _builtLongForm;
        const gr::Size_t nextInterval = newSettings.contains("reseed_interval") ? reseed_interval.value : _interval;
        const bool       rebuild      = nextLength != _builtLength || nextLongForm != _builtLongForm;
        if (rebuild && _running) {
            throw gr::exception(std::format("length and long_form are fixed while the block runs; stop it to change them from {} and {}", _builtLength, _builtLongForm));
        }
        _interval = nextInterval;
        if (rebuild) {
            build(nextLength, nextLongForm);
        }
        // the sums are recomputed on every sample whose offset since the build is a multiple of the interval
        const gr::Size_t phase = static_cast<gr::Size_t>(_offset % _interval);
        _untilReseed           = phase == 0U ? 0U : _interval - phase;
        group_delay            = static_cast<gr::Size_t>(_delayLength);
    }

    void reset() { build(_builtLength, _builtLongForm); }

    [[nodiscard]] std::size_t tagDecimation() const noexcept { return 1UZ; }

    /// The delay every tag moves by, in half samples.
    [[nodiscard]] std::optional<std::uint64_t> twiceTagDelay() const noexcept { return 2ULL * static_cast<std::uint64_t>(_delayLength); }

    void filterSamples(std::span<const T> input, std::span<T> output) noexcept {
        const std::size_t nSamples = std::min(input.size(), output.size());
        const Real*       from     = reinterpret_cast<const Real*>(input.data());
        Real*             to       = reinterpret_cast<Real*>(output.data());

        for (std::size_t done = 0UZ; done < nSamples;) {
            if (_untilReseed == 0U) {
                reseedOne(from + done * kParts, to + done * kParts);
                _untilReseed = _interval - 1U;
                ++done;
                continue;
            }
            const std::size_t run = std::min(nSamples - done, static_cast<std::size_t>(_untilReseed));
            if (_nStages == 4UZ) {
                filterRun<4UZ>(from + done * kParts, to + done * kParts, run);
            } else {
                filterRun<2UZ>(from + done * kParts, to + done * kParts, run);
            }
            _untilReseed -= static_cast<gr::Size_t>(run);
            done += run;
        }
        _offset += static_cast<std::uint64_t>(nSamples);
    }

private:
    [[nodiscard]] static constexpr std::size_t delayOf(gr::Size_t newLength, bool newLongForm) noexcept { return (newLongForm ? 2UZ : 1UZ) * (static_cast<std::size_t>(newLength) - 1UZ); }

    /// Filters @p nSamples samples with no recomputation among them. Each boxcar's sum adds the entering sample less the
    /// leaving one.
    template<std::size_t kStages>
    void filterRun(const Real* from, Real* to, std::size_t nSamples) noexcept {
        std::array<Real, kStages * kParts> sums{};
        std::copy_n(_sums.begin(), sums.size(), sums.begin());
        Real*             history     = _history.data();
        Real*             line        = _delay.data();
        const std::size_t boxcar      = _length;
        const std::size_t stride      = _length * kParts;
        const std::size_t delayLength = _delayLength;
        const Real        divisor     = _divisor;
        std::size_t       cursor      = _cursor;
        std::size_t       due         = _delayCursor;

        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            std::array<Real, kParts> value{};
            std::copy_n(from + i * kParts, kParts, value.begin());
            Real* slot = history + cursor * kParts;
            for (std::size_t s = 0UZ; s < kStages; ++s, slot += stride) {
                for (std::size_t p = 0UZ; p < kParts; ++p) {
                    const Real dropped = slot[p];
                    slot[p]            = value[p];
                    sums[s * kParts + p] += value[p] - dropped;
                    value[p] = sums[s * kParts + p] / divisor;
                }
            }
            cursor        = cursor + 1UZ == boxcar ? 0UZ : cursor + 1UZ;
            Real* aligned = line + due * kParts;
            for (std::size_t p = 0UZ; p < kParts; ++p) {
                const Real sample  = from[i * kParts + p];
                to[i * kParts + p] = aligned[p] - value[p];
                aligned[p]         = sample;
            }
            due = due + 1UZ == delayLength ? 0UZ : due + 1UZ;
        }

        std::copy_n(sums.begin(), sums.size(), _sums.begin());
        _cursor      = cursor;
        _delayCursor = due;
    }

    /// Filters one sample and recomputes each boxcar's sum from its history, oldest sample first.
    void reseedOne(const Real* from, Real* to) noexcept {
        const std::size_t        next = _cursor + 1UZ == _length ? 0UZ : _cursor + 1UZ; // the oldest slot once the sample is in
        std::array<Real, kParts> value{};
        std::copy_n(from, kParts, value.begin());
        for (std::size_t s = 0UZ; s < _nStages; ++s) {
            Real* stage = _history.data() + s * _length * kParts;
            std::copy_n(value.begin(), kParts, stage + _cursor * kParts);
            for (std::size_t p = 0UZ; p < kParts; ++p) {
                Real sum{};
                for (std::size_t k = next; k < _length; ++k) {
                    sum += stage[k * kParts + p];
                }
                for (std::size_t k = 0UZ; k < next; ++k) {
                    sum += stage[k * kParts + p];
                }
                _sums[s * kParts + p] = sum;
                value[p]              = sum / _divisor;
            }
        }
        _cursor       = next;
        Real* aligned = _delay.data() + _delayCursor * kParts;
        for (std::size_t p = 0UZ; p < kParts; ++p) {
            const Real sample = from[p];
            to[p]             = aligned[p] - value[p];
            aligned[p]        = sample;
        }
        _delayCursor = _delayCursor + 1UZ == _delayLength ? 0UZ : _delayCursor + 1UZ;
    }

    void build(gr::Size_t newLength, bool newLongForm) {
        const std::size_t nStages     = newLongForm ? 4UZ : 2UZ;
        const std::size_t boxcar      = static_cast<std::size_t>(newLength);
        const std::size_t delayLength = delayOf(newLength, newLongForm);
        _history.assign(nStages * kParts * boxcar, Real{});
        _delay.assign(kParts * delayLength, Real{});
        _sums.fill(Real{});
        _nStages       = nStages;
        _length        = boxcar;
        _delayLength   = delayLength;
        _divisor       = static_cast<Real>(boxcar);
        _cursor        = 0UZ;
        _delayCursor   = 0UZ;
        _offset        = 0ULL;
        _untilReseed   = 0U;
        _builtLength   = newLength;
        _builtLongForm = newLongForm;
        group_delay    = static_cast<gr::Size_t>(delayLength);
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_DC_BLOCKER_HPP
