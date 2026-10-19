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

/// A length-D moving average by running sum. A step with `reseed` set recomputes the sum exactly from the history.
template<typename T>
struct Boxcar {
    std::vector<T> _history{};
    std::size_t    _cursor = 0UZ;
    T              _sum{};

    void configure(std::size_t length) {
        _history.assign(length, T{});
        _cursor = 0UZ;
        _sum    = T{};
    }

    [[nodiscard]] T step(T sample, bool reseed) noexcept {
        const T dropped   = _history[_cursor];
        _history[_cursor] = sample;
        _cursor           = _cursor + 1UZ == _history.size() ? 0UZ : _cursor + 1UZ;

        if (reseed) {
            T sum{};
            for (std::size_t k = 0UZ; k < _history.size(); ++k) { // ascending, oldest first
                sum += _history[(_cursor + k) % _history.size()];
            }
            _sum = sum;
        } else {
            _sum += sample - dropped;
        }
        return _sum / static_cast<typename RealOf<T>::type>(_history.size());
    }
};

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

    std::vector<detail::Boxcar<T>> _stages{};
    std::vector<T>                 _delay{};
    std::size_t                    _cursor = 0UZ;
    std::uint64_t                  _offset = 0ULL;

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
        const gr::Size_t builtDelay   = static_cast<gr::Size_t>(_delay.size());
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
        group_delay = static_cast<gr::Size_t>(_delay.size());
    }

    void reset() { build(_builtLength, _builtLongForm); }

    [[nodiscard]] std::size_t tagDecimation() const noexcept { return 1UZ; }

    /// The delay every tag moves by, in half samples.
    [[nodiscard]] std::optional<std::uint64_t> twiceTagDelay() const noexcept { return 2ULL * static_cast<std::uint64_t>(_delay.size()); }

    void filterSamples(std::span<const T> input, std::span<T> output) noexcept {
        const std::size_t nSamples = std::min(input.size(), output.size());
        const std::size_t interval = _interval;

        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            const bool reseed = _offset % interval == 0ULL;
            T          value  = input[i];
            for (detail::Boxcar<T>& stage : _stages) {
                value = stage.step(value, reseed);
            }
            const T aligned = _delay[_cursor];
            _delay[_cursor] = input[i];
            _cursor         = _cursor + 1UZ == _delay.size() ? 0UZ : _cursor + 1UZ;
            output[i]       = aligned - value;
            ++_offset;
        }
    }

private:
    [[nodiscard]] static constexpr std::size_t delayOf(gr::Size_t newLength, bool newLongForm) noexcept { return (newLongForm ? 2UZ : 1UZ) * (static_cast<std::size_t>(newLength) - 1UZ); }

    void build(gr::Size_t newLength, bool newLongForm) {
        _stages.assign(newLongForm ? 4UZ : 2UZ, detail::Boxcar<T>{});
        for (detail::Boxcar<T>& stage : _stages) {
            stage.configure(newLength);
        }
        // The line holds exactly the group delay in slots. `_delay[_cursor]` holds the sample due now.
        _delay.assign(delayOf(newLength, newLongForm), T{});
        _cursor        = 0UZ;
        _offset        = 0ULL;
        _builtLength   = newLength;
        _builtLongForm = newLongForm;
        group_delay    = static_cast<gr::Size_t>(_delay.size());
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_DC_BLOCKER_HPP
