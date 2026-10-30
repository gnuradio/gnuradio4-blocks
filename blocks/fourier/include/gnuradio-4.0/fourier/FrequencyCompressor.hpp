#ifndef GNURADIO_FREQUENCY_COMPRESSOR_HPP
#define GNURADIO_FREQUENCY_COMPRESSOR_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <expected>
#include <format>
#include <limits>
#include <span>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/fourier/PhaseVocoder.hpp>

namespace gr::blocks::fourier {

namespace detail {

/// The longest analysis frame accepted.
inline constexpr gr::Size_t kMaxCompressorFrame = 1U << 20U;

/// @brief The validator of `frame`: true for a power of two.
inline constexpr auto kPowerOfTwo = [](gr::Size_t value) { return value != 0U && (value & (value - 1U)) == 0U; };

/// @brief True for a sample whose two parts are finite.
inline constexpr auto kFiniteSample = [](const std::complex<float>& sample) { return std::isfinite(sample.real()) && std::isfinite(sample.imag()); };

/// @brief True for a sample with a part that is not finite.
inline constexpr auto kNonFiniteSample = [](const std::complex<float>& sample) { return !kFiniteSample(sample); };

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::fourier::FrequencyCompressor)

struct FrequencyCompressor : Block<FrequencyCompressor, NoTagPropagation> {
    using Description = Doc<R""(
@brief Divides every frequency of a complex stream by `divisor` and keeps the duration.

The output is the input's spectrum at 1/`divisor` of its frequencies. A band and its harmonics keep their frequency
ratios. A phase vocoder compresses time by `divisor`, and the output rate is `sample_rate / divisor`.

`frame` is the analysis length, a power of two. `fs / frame` is the finest frequency step, and `frame / fs` the time a
short event spreads over and the latency above a divisor of 1. `divisor` is at most `frame / 4`. A `divisor` of 1
copies the input with no latency.

With `sample_rate` above zero, the block publishes `sample_rate / divisor` on its first output sample, and again on the
first sample made after that rate changes. A change of `divisor` or `frame` drops the frame in progress and starts a
new one. The output made before the change still goes out, ahead of the first sample of the new plan. At end of stream
the samples short of a frame are dropped.

Input tags are dropped. An upstream `sample_rate` tag does not set `sample_rate`. A sample with a part that is not
finite enters the vocoder as zero. Both ports are asynchronous, and each call publishes the hops the vocoder completes.
)"">;

    PortIn<std::complex<float>, Async>  in;
    PortOut<std::complex<float>, Async> out;

    Annotated<gr::Size_t, "divisor", Visible, Doc<"frequency divisor, 1 to copy the input">, Limits<1U, detail::kMaxCompressorFrame / 4U>>                              divisor     = 1U;
    Annotated<gr::Size_t, "frame", Doc<"analysis frame in samples, a power of two">, Limits<4U, detail::kMaxCompressorFrame, detail::kPowerOfTwo>>                      frame       = 4096U;
    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"input sample rate, 0 to leave the output rate unstated">, Limits<0.f, std::numeric_limits<float>::max()>> sample_rate = 0.f;

    GR_MAKE_REFLECTABLE(FrequencyCompressor, in, out, divisor, frame, sample_rate);

    /// The settings the block computes on.
    struct Accepted {
        gr::Size_t divisor    = 1U;
        gr::Size_t frame      = 4096U;
        float      sampleRate = 0.f;
    };

    gr::algorithm::PhaseVocoder      _vocoder;
    std::vector<std::complex<float>> _queue;
    std::vector<std::complex<float>> _made;
    std::vector<std::complex<float>> _finite; ///< the call's input with non-finite samples set to zero
    Accepted                         _accepted{divisor.value, frame.value, sample_rate.value};
    float                            _queueRate  = 0.f;  ///< the output rate of the samples in `_queue`, 0 for none
    float                            _statedRate = -1.f; ///< the output rate last stated, 0 for none, -1 before the first output
    bool                             _running    = false;

    /// One call's input bound, so a burst of buffered input cannot queue unbounded output.
    static constexpr std::size_t kMaxTake = 1UZ << 16;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        Accepted next = _accepted;
        if (newSettings.contains("divisor")) {
            next.divisor = divisor.value;
        }
        if (newSettings.contains("frame")) {
            next.frame = frame.value;
        }
        if (newSettings.contains("sample_rate")) {
            next.sampleRate = sample_rate.value;
        }
        if (next.divisor > next.frame / 4U) {
            throw gr::exception(std::format("divisor ({}) must not exceed a quarter of the frame ({}): the synthesis hop would vanish", next.divisor, next.frame));
        }
        const bool replanned = next.divisor != _accepted.divisor || next.frame != _accepted.frame;
        _accepted            = next;
        if (_running && replanned) {
            replan();
        }
    }

    /// @brief Plans the vocoder for the accepted frame and divisor and drops the frame in progress. The queued output
    /// stays.
    void replan() {
        const auto frameSize = static_cast<std::size_t>(_accepted.frame);
        const auto n         = static_cast<std::size_t>(_accepted.divisor);
        // A quarter-frame analysis hop is the overlap at which the Hann window is well behaved.
        // Shrinking it to a multiple of the divisor keeps the synthesis hop whole. A whole
        // synthesis hop keeps the output rate exactly fs over the divisor.
        std::size_t hopIn = frameSize / 4UZ;
        hopIn -= hopIn % n;
        if (const std::expected<void, gr::Error> planned = _vocoder.configure(frameSize, hopIn, hopIn / n); !planned) {
            throw gr::exception(planned.error().message, planned.error().sourceLocation);
        }
        _made.clear();
    }

    void start() {
        _queue.clear();
        replan();
        _statedRate = -1.f;
        _running    = true;
    }

    void stop() { _running = false; }

    /// The output rate the accepted settings give, 0 for none.
    [[nodiscard]] float acceptedRate() const noexcept { return _accepted.sampleRate > 0.f ? _accepted.sampleRate / static_cast<float>(_accepted.divisor) : 0.f; }

    /// @brief States @p rate on the first sample of @p outSpan when it differs from the rate last stated.
    void stateRate(OutputSpanLike auto& outSpan, float rate) {
        if (rate == _statedRate) {
            return;
        }
        if (rate > 0.f) {
            outSpan.publishTag(property_map{{static_cast<std::pmr::string>(gr::tag::SAMPLE_RATE), rate}}, 0UZ);
        }
        _statedRate = rate;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (_accepted.divisor == 1U && _queue.empty()) { // passthrough, with nothing to do and no latency to add
            const std::size_t n = std::min(inSpan.size(), outSpan.size());
            std::copy_n(inSpan.begin(), n, outSpan.begin());
            std::ignore = inSpan.consume(n);
            if (n > 0UZ) {
                stateRate(outSpan, acceptedRate());
            }
            outSpan.publish(n);
            return n > 0UZ ? work::Status::OK : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        // Input is taken only while nothing waits to go out. The queue stays shallow, and back pressure reaches the
        // source.
        if (_queue.empty()) {
            const std::size_t take = std::min(inSpan.size(), kMaxTake);
            if (take == 0UZ) {
                std::ignore = inSpan.consume(0UZ);
                outSpan.publish(0UZ);
                return work::Status::INSUFFICIENT_INPUT_ITEMS;
            }
            std::span<const std::complex<float>> taken(inSpan.data(), take);
            if (!std::ranges::all_of(taken, detail::kFiniteSample)) {
                _finite.assign(taken.begin(), taken.end());
                std::ranges::replace_if(_finite, detail::kNonFiniteSample, std::complex<float>{});
                taken = _finite;
            }
            _made.clear();
            _vocoder.process(taken, _made);
            _queue.insert(_queue.end(), _made.begin(), _made.end());
            _queueRate  = acceptedRate();
            std::ignore = inSpan.consume(take);
        } else {
            std::ignore = inSpan.consume(0UZ);
        }

        const std::size_t n = std::min(_queue.size(), outSpan.size());
        std::copy_n(_queue.begin(), n, outSpan.begin());
        _queue.erase(_queue.begin(), _queue.begin() + static_cast<std::ptrdiff_t>(n));
        if (n > 0UZ) {
            stateRate(outSpan, _queueRate);
        }
        outSpan.publish(n);
        return n > 0UZ ? work::Status::OK : work::Status::INSUFFICIENT_INPUT_ITEMS;
    }
};

} // namespace gr::blocks::fourier

#endif // GNURADIO_FREQUENCY_COMPRESSOR_HPP
