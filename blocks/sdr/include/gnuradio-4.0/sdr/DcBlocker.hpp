#ifndef GNURADIO_SDR_DC_BLOCKER_HPP
#define GNURADIO_SDR_DC_BLOCKER_HPP

#include <numbers>

namespace gr::blocks::sdr {

// First-order DC blocker for one real stream: y[n] = x[n] - x[n-1] + r*y[n-1], with the pole
// r = 1 - 2*pi*fc/fs. Samples are float; the recurrence runs in double.
//
// The double state is the point of this class. A radio blocks DC far below its sample rate (10 Hz on 2 MS/s is
// fc/fs = 5e-6), and a second-order section in direct form II holds x/a(1) in its state under a constant input,
// which at that ratio is 5e8 for a 0.5 DC term where a float's step is 32; the output is then the difference of
// numbers that no longer resolve the signal. This form builds no such state (|y| stays within the range of the
// signal itself), but 1 - r is 3.1e-5, so the recurrence still needs more than a float's seven digits to carry
// the pole.
//
// A cutoff is realizable while 0 < r < 1, that is while 2*pi*fc < fs. Beyond that, r <= 0 is no longer a
// high-pass and r <= -1 (fc >= fs/pi) diverges; the bound is stricter than Nyquist and far above any cutoff
// that blocks DC. A cutoff outside it, or a sample rate that is not positive, leaves the blocker passing
// samples through unchanged rather than degrading the stream.
class DcBlocker {
public:
    DcBlocker() noexcept = default;
    DcBlocker(float cutoffHz, float sampleRate) noexcept { setCutoff(cutoffHz, sampleRate); }

    // true when the cutoff was taken, false when the blocker is left passing samples through; the state is
    // cleared either way.
    bool setCutoff(float cutoffHz, float sampleRate) noexcept {
        reset();
        const double r = 1.0 - 2.0 * std::numbers::pi * static_cast<double>(cutoffHz) / static_cast<double>(sampleRate);
        _blocking      = (cutoffHz > 0.f) && (sampleRate > 0.f) && (r > 0.0) && (r < 1.0);
        _r             = _blocking ? r : 0.0;
        return _blocking;
    }

    [[nodiscard]] float processOne(float sample) noexcept {
        if (!_blocking) {
            return sample;
        }
        const double x = static_cast<double>(sample);
        const double y = x - _xPrev + _r * _yPrev;
        _xPrev         = x;
        _yPrev         = y;
        return static_cast<float>(y);
    }

    // forgets the past samples; the cutoff is kept
    void reset() noexcept {
        _xPrev = 0.0;
        _yPrev = 0.0;
    }

private:
    double _r        = 0.0;
    double _xPrev    = 0.0;
    double _yPrev    = 0.0;
    bool   _blocking = false;
};

} // namespace gr::blocks::sdr

#endif // GNURADIO_SDR_DC_BLOCKER_HPP
