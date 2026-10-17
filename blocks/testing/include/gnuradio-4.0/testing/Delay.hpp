#ifndef GNURADIO_TESTING_DELAY_HPP
#define GNURADIO_TESTING_DELAY_HPP

#include <algorithm>
#include <chrono>
#include <thread>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

namespace gr::blocks::testing {

GR_REGISTER_BLOCK(gr::blocks::testing::Delay, [T], [float])

// Holds the stream back for delay_ms after start(). The first call sleeps on the calling thread until the hold ends.
// It sleeps in slices of kHoldSlice and returns DONE, passing nothing, once the block is asked to stop.
// isBlocking() reports that the first call waits inside work().
// Under NoTagPropagation the framework forwards nothing. The block republishes every input tag itself.
template<typename T>
struct Delay : Block<Delay<T>, NoTagPropagation> {
    using clock = std::chrono::steady_clock;

    static constexpr std::chrono::milliseconds kHoldSlice{5};

    PortIn<T>  in;
    PortOut<T> out;
    uint32_t   delay_ms = 0;

    GR_MAKE_REFLECTABLE(Delay, in, out, delay_ms);

    bool              _waiting = true;
    clock::time_point _start_time;

    [[nodiscard]] constexpr bool isBlocking() const noexcept { return true; }

    gr::work::Status processBulk(InputSpanLike auto& input, OutputSpanLike auto& output) {
        if (_waiting) {
            const clock::time_point end = _start_time + std::chrono::milliseconds(delay_ms);
            for (clock::time_point now = clock::now(); now < end; now = clock::now()) {
                if (lifecycle::isShuttingDown(this->state())) {
                    std::ignore = input.consume(0UZ);
                    output.publish(0UZ);
                    return work::Status::DONE;
                }
                std::this_thread::sleep_until(std::min(end, now + kHoldSlice));
            }
            _waiting = false;
        }

        const auto n = std::min(input.size(), output.size());
        for (const Tag& tag : input.rawTags) {
            if (tag.index >= input.streamIndex && tag.index - input.streamIndex < n) {
                output.publishTag(tag.map, tag.index - input.streamIndex);
            }
        }
        std::ranges::copy(input.first(n), output.begin());
        std::ignore = input.consume(n);
        output.publish(n);
        return work::Status::OK;
    }

    void start() noexcept {
        _waiting    = true;
        _start_time = clock::now();
    }
};

} // namespace gr::blocks::testing

#endif // GNURADIO_TESTING_DELAY_HPP
