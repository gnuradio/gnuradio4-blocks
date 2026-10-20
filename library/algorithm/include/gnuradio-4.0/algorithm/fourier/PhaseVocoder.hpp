#ifndef GNURADIO_PHASE_VOCODER_HPP
#define GNURADIO_PHASE_VOCODER_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <expected>
#include <format>
#include <numbers>
#include <source_location>
#include <span>
#include <vector>

#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>

namespace gr::algorithm {

/**
 * @brief Time-scale modification of a complex stream that keeps every partial coherent.
 *
 * Frames of `frameSize` samples are taken every `analysisHop` samples under a Hann window. They are
 * resynthesized every `synthesisHop` samples under the same window. The stream is replayed at
 * `analysisHop / synthesisHop` times its original pace, and its spectral content is unchanged. Each
 * bin's synthesis phase advances by the bin's *measured* frequency and not by its center frequency
 * alone. The measured frequency is the center plus a correction. The correction is the principal
 * value of the analysis phase advance's deviation from the center's advance, divided by the
 * analysis hop. A partial that lies between bins then stays coherent from frame to frame. The
 * first frame takes its analysis phase unchanged. A stream that begins mid-tone therefore begins
 * coherent and not at an arbitrary phase offset. Each bin's synthesis phase is reduced to within
 * one turn every frame. The output holds its quality however long the stream runs.
 *
 * The magnitude spectrum is not modified. No two bins collide in the output, and no amplitude
 * ambiguity arises. The overlap-add is normalized by the summed squared window actually applied.
 * This preserves the level at any hop pair. Equal hops replay the stream exactly. The identity case
 * is a plain copy with no latency.
 *
 * State carries across `process` calls. The call boundaries are not visible in the output. A stream
 * fed whole and the same stream fed one sample at a time produce identical results.
 */
struct PhaseVocoder {
    std::size_t frameSize    = 0UZ;
    std::size_t analysisHop  = 0UZ;
    std::size_t synthesisHop = 0UZ;

    /**
     * @brief Size the engine. Returns an error for a shape the algorithm cannot run, and then changes nothing.
     *
     * The frame must be a power of two, the transform's size. Both hops must be positive and no
     * longer than the frame. A hop above the frame would leave gaps that no window covers.
     */
    [[nodiscard]] std::expected<void, gr::Error> configure(std::size_t frame, std::size_t hopIn, std::size_t hopOut, std::source_location location = std::source_location::current()) {
        if (frame == 0UZ || (frame & (frame - 1UZ)) != 0UZ) {
            return std::unexpected(gr::Error(std::format("PhaseVocoder: frame is {}; it must be a power of two", frame), location));
        }
        if (hopIn == 0UZ || hopOut == 0UZ || hopIn > frame || hopOut > frame) {
            return std::unexpected(gr::Error(std::format("PhaseVocoder: hops are {} and {}; they must be positive and no longer than the frame, {}", hopIn, hopOut, frame), location));
        }
        frameSize    = frame;
        analysisHop  = hopIn;
        synthesisHop = hopOut;

        _window.resize(frame);
        for (std::size_t i = 0UZ; i < frame; ++i) {
            _window[i] = static_cast<float>(0.5 - 0.5 * std::cos(2.0 * std::numbers::pi * static_cast<double>(i) / static_cast<double>(frame)));
        }
        _scratch.resize(frame);
        _spectrum.resize(frame);
        _synth.resize(frame);
        _frameOut.resize(frame);
        reset();
        return {};
    }

    /// @brief Clear all stream state. The next call starts empty, as after construction.
    void reset() {
        _pending.clear();
        _lastPhase.assign(frameSize, 0.0);
        _sumPhase.assign(frameSize, 0.0);
        _acc.assign(frameSize, std::complex<float>{0.f, 0.f});
        _accWindow.assign(frameSize, 0.f);
        _primed = false;
    }

    /**
     * @brief Consume a block of input, appending however much output it completes.
     *
     * In the steady state the output count is `synthesisHop / analysisHop` times the input count.
     * A final partial frame stays pending. A window that is not full resolves no frequency.
     */
    void process(std::span<const std::complex<float>> in, std::vector<std::complex<float>>& out) {
        if (analysisHop == synthesisHop) { // identity, replaying at the same pace gives the input
            out.insert(out.end(), in.begin(), in.end());
            return;
        }
        _pending.insert(_pending.end(), in.begin(), in.end());

        std::size_t consumed = 0UZ;
        while (_pending.size() - consumed >= frameSize) {
            analyzeResynthesize(_pending.data() + consumed, out);
            consumed += analysisHop;
        }
        _pending.erase(_pending.begin(), _pending.begin() + static_cast<std::ptrdiff_t>(consumed));
    }

private:
    void analyzeResynthesize(const std::complex<float>* src, std::vector<std::complex<float>>& out) {
        for (std::size_t i = 0UZ; i < frameSize; ++i) {
            _scratch[i] = src[i] * _window[i];
        }
        _fft.compute(_scratch, _spectrum);

        const double expectedPerBin = 2.0 * std::numbers::pi * static_cast<double>(analysisHop) / static_cast<double>(frameSize);
        for (std::size_t k = 0UZ; k < frameSize; ++k) {
            const double magnitude = static_cast<double>(std::abs(_spectrum[k]));
            const double phase     = static_cast<double>(std::arg(_spectrum[k]));
            if (!_primed) {
                _lastPhase[k] = phase;
                _sumPhase[k]  = phase;
            } else {
                const double deviation = principal(phase - _lastPhase[k] - expectedPerBin * static_cast<double>(k));
                const double omega     = 2.0 * std::numbers::pi * static_cast<double>(k) / static_cast<double>(frameSize) + deviation / static_cast<double>(analysisHop);
                _lastPhase[k]          = phase;
                _sumPhase[k]           = std::remainder(_sumPhase[k] + omega * static_cast<double>(synthesisHop), 2.0 * std::numbers::pi);
            }
            _synth[k] = std::polar(static_cast<float>(magnitude), static_cast<float>(_sumPhase[k]));
        }
        _primed = true;

        inverse(_synth, _frameOut);
        for (std::size_t i = 0UZ; i < frameSize; ++i) {
            _acc[i] += _frameOut[i] * _window[i];
            _accWindow[i] += _window[i] * _window[i];
        }

        // The leading synthesis hop receives no further contribution and is final. Normalizing by
        // the window energy actually applied keeps the level independent of the overlap.
        for (std::size_t i = 0UZ; i < synthesisHop; ++i) {
            out.push_back(_accWindow[i] > 1e-12f ? _acc[i] / _accWindow[i] : std::complex<float>{0.f, 0.f});
        }
        std::rotate(_acc.begin(), _acc.begin() + static_cast<std::ptrdiff_t>(synthesisHop), _acc.end());
        std::rotate(_accWindow.begin(), _accWindow.begin() + static_cast<std::ptrdiff_t>(synthesisHop), _accWindow.end());
        std::fill(_acc.end() - static_cast<std::ptrdiff_t>(synthesisHop), _acc.end(), std::complex<float>{0.f, 0.f});
        std::fill(_accWindow.end() - static_cast<std::ptrdiff_t>(synthesisHop), _accWindow.end(), 0.f);
    }

    /// @brief Wrap to (-pi, pi].
    [[nodiscard]] static double principal(double angle) noexcept { return angle - 2.0 * std::numbers::pi * std::round(angle / (2.0 * std::numbers::pi)); }

    /// @brief Inverse transform as conjugate, forward transform, conjugate and scale.
    void inverse(const std::vector<std::complex<float>>& in, std::vector<std::complex<float>>& out) {
        for (std::size_t i = 0UZ; i < frameSize; ++i) {
            _scratch[i] = std::conj(in[i]);
        }
        _fft.compute(_scratch, _spectrum);
        const float scale = 1.f / static_cast<float>(frameSize);
        for (std::size_t i = 0UZ; i < frameSize; ++i) {
            out[i] = std::conj(_spectrum[i]) * scale;
        }
    }

    std::vector<float>               _window;
    std::vector<std::complex<float>> _pending;
    std::vector<double>              _lastPhase;
    std::vector<double>              _sumPhase;
    std::vector<std::complex<float>> _acc;
    std::vector<float>               _accWindow;
    bool                             _primed = false;

    FFT<std::complex<float>>         _fft;
    std::vector<std::complex<float>> _scratch, _spectrum, _synth, _frameOut;
};

} // namespace gr::algorithm

#endif // GNURADIO_PHASE_VOCODER_HPP
