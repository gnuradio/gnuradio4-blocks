#ifndef GNURADIO_THROTTLE_HPP
#define GNURADIO_THROTTLE_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <thread>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/LifeCycle.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/basic/NamespaceCompatibility.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::Throttle, [T], [ float, std::complex<float>, std::int16_t, std::uint8_t ])

template<typename T>
struct Throttle : Block<Throttle<T>, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief Paces a stream to a configured samples per second against an absolute deadline schedule.

For a flowgraph with no clock of its own. It is not a sample-clock authority: it makes a long-run average rate. The
wait is taken in slices with the lifecycle state re-read between them, so a stop request is not held up by a long
sleep. The output port is optional; unconnected, the block still paces consumption, so it can hang off a stream
without paying for the copy. An incoming `sample_rate` tag retunes the block, though a rate the caller has set
explicitly is out of the auto-update set and no longer follows one.

The block is 1:1, so every input tag key passes through at its own offset, `sample_rate` carrying this block's value.
)"">;

    PortIn<T>            in;
    PortOut<T, Optional> out;

    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"target average rate; setting it restarts the pacing schedule">> sample_rate         = 1e6f;
    Annotated<gr::Size_t, "max_items_per_chunk", Doc<"upper bound on the items handled between sleeps; 0 is none">>  max_items_per_chunk = 0U;
    Annotated<double, "max_sleep_s", Unit<"s">, Doc<"longest single sleep; longer waits are taken in slices">>       max_sleep_s         = 0.1;

    GR_MAKE_REFLECTABLE(Throttle, in, out, sample_rate, max_items_per_chunk, max_sleep_s);

    using Clock = std::chrono::steady_clock;

    Clock::time_point _origin = Clock::now();
    std::uint64_t     _total  = 0ULL;
    double            _period = 1e-6; // seconds per sample, held in double so a long run does not drift on a rounded tick
    Clock::duration   _slice  = std::chrono::milliseconds(100);

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (!(sample_rate > 0.f) || !std::isfinite(sample_rate)) {
            throw gr::exception(std::format("sample_rate must be positive and finite, got {}", sample_rate.value));
        }
        if (!(max_sleep_s > 0.0) || !std::isfinite(max_sleep_s)) {
            throw gr::exception(std::format("max_sleep_s must be positive and finite, got {}", max_sleep_s.value));
        }
        _period = 1.0 / static_cast<double>(sample_rate);
        _slice  = seconds(max_sleep_s);
        if (newSettings.contains("sample_rate")) {
            restart();
        }
    }

    void start() { restart(); }

    void reset() { restart(); }

    void restart() noexcept {
        _origin = Clock::now();
        _total  = 0ULL;
    }

    /// @brief Samples paced since the pacing origin; the tests read it to confirm a rate change restarted the schedule.
    [[nodiscard]] std::uint64_t paced() const noexcept { return _total; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t cap   = max_items_per_chunk == 0U ? inSpan.size() : std::min(inSpan.size(), static_cast<std::size_t>(max_items_per_chunk.value));
        const std::size_t count = outSpan.isConnected ? std::min(cap, outSpan.size()) : cap;
        if (outSpan.isConnected) {
            std::copy_n(inSpan.begin(), count, outSpan.begin());
        }

        waitUntilDue(count);
        _total += count;
        std::ignore = inSpan.consume(count);
        outSpan.publish(outSpan.isConnected ? count : 0UZ);
        return work::Status::OK;
    }

private:
    [[nodiscard]] static Clock::duration seconds(double value) noexcept { return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(value)); }

    void waitUntilDue(std::size_t count) {
        const Clock::time_point deadline = _origin + seconds(_period * static_cast<double>(_total + count));
        Clock::time_point       now      = Clock::now();
        while (deadline > now && !lifecycle::isShuttingDown(this->state())) {
            std::this_thread::sleep_until(std::min(deadline, now + _slice));
            now = Clock::now();
        }
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_THROTTLE_HPP
