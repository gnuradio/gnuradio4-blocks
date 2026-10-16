#ifndef GNURADIO_RATE_PROBE_HPP
#define GNURADIO_RATE_PROBE_HPP

#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::RateProbe, [T], [ std::uint8_t, std::int16_t, std::int32_t, std::uint32_t, float, double, std::complex<float>, std::complex<double> ])

template<typename T>
struct RateProbe : Block<RateProbe<T>> {
    using Description = Doc<R""(
@brief Measures the rate at which samples arrive at its input, in samples per second.

The block consumes every sample it receives. `rate` is the arrival rate at this block. A `Throttle` upstream sets
that rate when no block between them changes the rate. Behind a source that runs freely, `rate` reports how fast the
graph runs.

The first interval opens at the work call that delivers the first samples, and each interval counts the samples
that arrive after it opens. A measurement takes place in a work call once `min_update_ms` have passed since the
interval opened. It divides the samples received in that interval by the interval's length on a steady clock and
opens the next interval. `rate_now` holds the quotient. `rate` takes the first result as it is, then moves toward
each new result by the weight `alpha`. Before the first measurement `rate` and `rate_now` are 0. While no samples
arrive, both keep their last values. `n_samples` counts the samples received since start and wraps at 2^32.

After each measurement the block copies `rate`, `rate_now` and `n_samples` into its settings, where a settings Get
reads them. It also sends one notification on `msgOut` with the endpoint `rate`, carrying the same three values.
)"">;

    using Clock = std::chrono::steady_clock;

    PortIn<T> in;

    Annotated<float, "alpha", Doc<"weight of the newest measurement in `rate`, in (0, 1]">, Visible>           alpha         = 0.15f;
    Annotated<float, "min_update_ms", Unit<"ms">, Doc<"shortest interval between two measurements">, Visible>  min_update_ms = 500.f;
    Annotated<double, "rate", Unit<"Hz">, Doc<"smoothed arrival rate in samples per second">, Visible>         rate          = 0.0;
    Annotated<double, "rate_now", Unit<"Hz">, Doc<"arrival rate over the last measurement interval">, Visible> rate_now      = 0.0;
    Annotated<gr::Size_t, "n_samples", Doc<"samples received since start">, Visible>                           n_samples     = 0U;

    GR_MAKE_REFLECTABLE(RateProbe, in, alpha, min_update_ms, rate, rate_now, n_samples);

    Clock::time_point _intervalStart   = Clock::now();
    std::uint64_t     _intervalSamples = 0ULL;
    bool              _measuredOnce    = false;
    bool              _receiving       = false;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (!(alpha > 0.f && alpha <= 1.f)) {
            throw gr::exception(std::format("alpha must be in (0, 1], got {}", alpha.value));
        }
        if (!(min_update_ms >= 0.f) || !std::isfinite(min_update_ms)) {
            throw gr::exception(std::format("min_update_ms must be finite and at least 0, got {}", min_update_ms.value));
        }
    }

    void start() {
        _intervalStart   = Clock::now();
        _intervalSamples = 0ULL;
        _measuredOnce    = false;
        _receiving       = false;
        rate             = 0.0;
        rate_now         = 0.0;
        n_samples        = 0U;
        std::ignore      = this->updateActiveParameters({"rate", "rate_now", "n_samples"});
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan) {
        const std::size_t nSamples = inSpan.size();
        std::ignore                = inSpan.consume(nSamples);
        n_samples                  = static_cast<gr::Size_t>(n_samples.value + nSamples);
        if (nSamples == 0UZ) {
            return work::Status::OK;
        }

        const Clock::time_point now = Clock::now();
        if (!_receiving) {
            _receiving     = true;
            _intervalStart = now;
            return work::Status::OK;
        }
        _intervalSamples += nSamples;
        const std::chrono::duration<double> elapsed = now - _intervalStart;
        if (elapsed.count() > 0.0 && elapsed.count() * 1e3 >= static_cast<double>(min_update_ms)) {
            measure(now, elapsed.count());
        }
        return work::Status::OK;
    }

private:
    void measure(Clock::time_point now, double seconds) {
        rate_now            = static_cast<double>(_intervalSamples) / seconds;
        const double weight = static_cast<double>(alpha.value);
        rate                = _measuredOnce ? weight * rate_now.value + (1.0 - weight) * rate.value : rate_now.value;
        _measuredOnce       = true;

        _intervalSamples = 0ULL;
        _intervalStart   = now;

        std::ignore = this->updateActiveParameters({"rate", "rate_now", "n_samples"});
        this->emitMessage("rate", {{"rate", rate.value}, {"rate_now", rate_now.value}, {"n_samples", n_samples.value}});
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_RATE_PROBE_HPP
