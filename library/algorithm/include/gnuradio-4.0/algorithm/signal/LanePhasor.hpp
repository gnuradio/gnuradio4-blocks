#ifndef GNURADIO_ALGORITHM_LANEPHASOR_HPP
#define GNURADIO_ALGORITHM_LANEPHASOR_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <numbers>
#include <span>

namespace gr::signal {

/**
 * @brief A complex exponential at a constant radians-per-sample increment, applied to a span (mix) or written
 * alone (fill), sixteen samples at a time.
 *
 * The accuracy depends on the absolute sample index and never on where a call began: seeds fall on a grid counted
 * from the stream start, so a stream handed over whole and the same stream split anywhere are as exact as each
 * other. Where the compiler contracts to fused multiply-add, the group loop and the loop for a partial group can
 * round differently, so the outputs can differ in the last bits by where calls cut.
 *
 * A serial phasor is a dependency chain per sample. Sixteen lanes advancing together by one group's step carry
 * none, and the compiler turns the group into whole-vector loops at whatever width the target has, with plain C++
 * and no switch on the instruction set.
 *
 * Every `ReseedInterval` samples the lanes are seeded exactly from one double-precision sine and cosine, rotated
 * by sixteen cached lane offsets. The recurrence drifts in magnitude as well as in phase, and the seed bounds
 * both. For `F = float` over 2^26 samples the worst error is 5.2e-7 rad in phase and 7.0e-7 in magnitude at 256
 * samples between seeds, 1.7e-6 and 2.0e-6 at 1024, and 5.9e-6 and 6.7e-6 at 4096: it grows nearly in proportion
 * to the interval. The spur this puts in a downconverter's output, measured through a decimating filter at
 * decimations of 4, 25 and 1000, is -143 dBc at 256, -131 dBc at 1024 and -119 dBc at 4096 (an eight-lane form at
 * 256 gives -139 dBc), with the stopband within 0.02 dB at every interval. A stage that needs a clean stopband
 * keeps the default of 256; a transmit rotator has no stopband to protect and can take a longer interval, which
 * spends less on seeds.
 *
 * The phase is radians, held in double; the exponential is exp(+j*phase), so a positive increment rotates
 * counter-clockwise.
 */
template<std::floating_point F, std::size_t ReseedInterval = 256UZ>
class LanePhasor {
public:
    /// how many lanes advance together
    static constexpr std::size_t kLanes = 16UZ;
    /// how many samples pass between exact seeds
    static constexpr std::size_t kReseedInterval = ReseedInterval;
    static_assert(kReseedInterval % kLanes == 0UZ, "a lane group must never straddle a re-seed boundary");

    LanePhasor() { setLaneOffsets(); }

    /// The stream restarts at `phase` with the given increment.
    void configure(double increment, double phase) noexcept {
        _increment = increment;
        _seedPhase = wrap(phase);
        _position  = 0UZ;
        setLaneOffsets();
    }

    /// The phase runs on from where it is, so a retune moves the rate and never the phase a listener hears.
    void setIncrement(double increment) noexcept {
        if (increment == _increment) {
            return;
        }
        _seedPhase = phase();
        _position  = 0UZ;
        _increment = increment;
        setLaneOffsets();
    }

    /// The next sample is exp(j*phase).
    void setPhase(double phase) noexcept {
        _seedPhase  = wrap(phase);
        _position   = 0UZ;
        _lanesValid = false;
    }

    /// The phase of the next sample, wrapped to [-pi, pi].
    [[nodiscard]] double phase() const noexcept { return wrap(_seedPhase + static_cast<double>(_position) * _increment); }

    [[nodiscard]] double increment() const noexcept { return _increment; }

    /// out[k] = in[k] * exp(j*phase_k) for as many samples as `in` holds. `out` is at least as long as `in`; it may
    /// be `in` itself but must not overlap it partly.
    void mix(std::span<const std::complex<F>> in, std::span<std::complex<F>> out) noexcept { run<false>(in.data(), out.data(), in.size()); }

    /// out[k] = exp(j*phase_k), bit for bit the mix of a span of ones.
    void fill(std::span<std::complex<F>> out) noexcept { run<true>(nullptr, out.data(), out.size()); }

private:
    [[nodiscard]] static double wrap(double phase) noexcept { return std::remainder(phase, 2.0 * std::numbers::pi); }

    // exp(j*w*increment) for each lane w, and the step one group advances by. A seed is then one sine and one
    // cosine plus sixteen products in double, whose error is 1e-16, rather than thirty-two calls; the lanes are as
    // exact as F holds.
    void setLaneOffsets() noexcept {
        for (std::size_t w = 0UZ; w < kLanes; ++w) {
            _offsetRe[w] = std::cos(static_cast<double>(w) * _increment);
            _offsetIm[w] = std::sin(static_cast<double>(w) * _increment);
        }
        _stepRe     = static_cast<F>(std::cos(static_cast<double>(kLanes) * _increment));
        _stepIm     = static_cast<F>(std::sin(static_cast<double>(kLanes) * _increment));
        _lanesValid = false;
    }

    // `re[w]`, `im[w]` become exp(j*(_seedPhase + (g + w)*increment)) where `g` is `_position` rounded down to a
    // multiple of kLanes. Anchoring the group to the absolute position rather than to where a call happened to
    // start gives the output its chunk independence: advancing the group by however many samples a call had left
    // over would give a different rounding path for a different chunking.
    void seedLanes(F* re, F* im) const noexcept {
        const double base = _seedPhase + static_cast<double>(_position - _position % kLanes) * _increment;
        const double c    = std::cos(base);
        const double s    = std::sin(base);
        for (std::size_t w = 0UZ; w < kLanes; ++w) {
            re[w] = static_cast<F>(c * _offsetRe[w] - s * _offsetIm[w]);
            im[w] = static_cast<F>(s * _offsetRe[w] + c * _offsetIm[w]);
        }
    }

    // Advances the whole group by one group's worth, which is the only step it ever takes: a partial group is
    // served from the lanes already standing, never by stepping them part-way.
    void advanceLanes(F* re, F* im) const noexcept {
        for (std::size_t w = 0UZ; w < kLanes; ++w) {
            const F a = re[w];
            const F b = im[w];
            re[w]     = a * _stepRe - b * _stepIm;
            im[w]     = a * _stepIm + b * _stepRe;
        }
    }

    // One group of output from one group of input. `xg` and `yg` are distinct runs of memory, which the caller
    // guarantees by copying the input first when `out` is `in`.
    static void mixGroup(const F* xg, F* yg, const F* re, const F* im) noexcept {
        for (std::size_t w = 0UZ; w < kLanes; ++w) {
            const F xr        = xg[2UZ * w];
            const F xi        = xg[2UZ * w + 1UZ];
            yg[2UZ * w]       = xr * re[w] - xi * im[w];
            yg[2UZ * w + 1UZ] = xr * im[w] + xi * re[w];
        }
    }

    // The one loop under mix and fill; `kFill` drops the input and writes the lanes.
    //
    // Samples are read and written through the value type rather than through std::complex, so a group is two
    // contiguous runs the compiler loads and stores whole. The group is held in locals for the whole call and
    // written back once: as members reached through `this` the compiler cannot prove that a store to the output
    // does not land on them, and it reloads both arrays per sample. Whole groups are split out with a trip count
    // it can see, which is most of what lets the loop vectorize. When `out` is `in`, a group of input is copied to
    // a local before the group of output is written, so the loop needs no no-alias promise; otherwise the input is
    // read where it lies, which spares a copy that costs registers on a target with few of them.
    //
    // A call that ends part-way through a group leaves the group where it stands and resumes at the same lane.
    template<bool kFill>
    void run(const std::complex<F>* in, std::complex<F>* out, std::size_t n) noexcept {
        alignas(64) F re[kLanes];
        alignas(64) F im[kLanes];
        if (_lanesValid) {
            std::copy_n(_laneRe, kLanes, re);
            std::copy_n(_laneIm, kLanes, im);
        } else {
            seedLanes(re, im);
            _lanesValid = true;
        }

        const F*   x       = reinterpret_cast<const F*>(in);
        F*         y       = reinterpret_cast<F*>(out);
        const bool inPlace = static_cast<const void*>(in) == static_cast<const void*>(out);

        for (std::size_t i = 0UZ; i < n;) {
            if (_position == kReseedInterval) {
                _seedPhase = wrap(_seedPhase + static_cast<double>(kReseedInterval) * _increment);
                _position  = 0UZ;
                seedLanes(re, im);
            }
            const std::size_t lane0 = _position % kLanes;

            if (lane0 == 0UZ) {
                const std::size_t groups = std::min((n - i) / kLanes, (kReseedInterval - _position) / kLanes);
                for (std::size_t g = 0UZ; g < groups; ++g) {
                    F* yg = y + 2UZ * i;
                    if constexpr (kFill) {
                        for (std::size_t w = 0UZ; w < kLanes; ++w) {
                            yg[2UZ * w]       = re[w];
                            yg[2UZ * w + 1UZ] = im[w];
                        }
                    } else if (inPlace) {
                        alignas(64) F xg[2UZ * kLanes];
                        std::copy_n(x + 2UZ * i, 2UZ * kLanes, xg);
                        mixGroup(xg, yg, re, im);
                    } else {
                        mixGroup(x + 2UZ * i, yg, re, im);
                    }
                    advanceLanes(re, im);
                    _position += kLanes;
                    i += kLanes;
                }
                if (groups > 0UZ) {
                    continue;
                }
            }

            const std::size_t take = std::min({kLanes - lane0, n - i, kReseedInterval - _position});
            for (std::size_t w = 0UZ; w < take; ++w) {
                if constexpr (kFill) {
                    y[2UZ * (i + w)]       = re[lane0 + w];
                    y[2UZ * (i + w) + 1UZ] = im[lane0 + w];
                } else {
                    const F xr             = x[2UZ * (i + w)];
                    const F xi             = x[2UZ * (i + w) + 1UZ];
                    y[2UZ * (i + w)]       = xr * re[lane0 + w] - xi * im[lane0 + w];
                    y[2UZ * (i + w) + 1UZ] = xr * im[lane0 + w] + xi * re[lane0 + w];
                }
            }
            _position += take;
            i += take;
            if (_position % kLanes == 0UZ && _position != kReseedInterval) {
                advanceLanes(re, im);
            }
        }

        std::copy_n(re, kLanes, _laneRe);
        std::copy_n(im, kLanes, _laneIm);
    }

    double      _increment  = 0.0;
    double      _seedPhase  = 0.0;
    std::size_t _position   = 0UZ;
    bool        _lanesValid = false;
    F           _stepRe     = F{1};
    F           _stepIm     = F{0};
    double      _offsetRe[kLanes]{};
    double      _offsetIm[kLanes]{};
    alignas(64) F _laneRe[kLanes]{};
    alignas(64) F _laneIm[kLanes]{};
};

} // namespace gr::signal

#endif // GNURADIO_ALGORITHM_LANEPHASOR_HPP
