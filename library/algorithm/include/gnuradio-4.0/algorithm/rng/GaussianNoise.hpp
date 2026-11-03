#ifndef GNURADIO_ALGORITHM_GAUSSIAN_NOISE_HPP
#define GNURADIO_ALGORITHM_GAUSSIAN_NOISE_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <utility>

#include <gnuradio-4.0/meta/simd_math.hpp>

#include <gnuradio-4.0/algorithm/rng/Xoshiro256pp.hpp>

namespace gr::rng {

/**
 * @brief Generates Gaussian-distributed samples via the Marsaglia polar method.
 *
 * Produces pairs of independent N(0,1) variates per rejection cycle, caching the spare.
 * complexSample() returns Option B convention: nI, nQ ~ N(0, 1/2) so E[|n|^2] = 1.
 * fill()/fillComplex() use local PRNG state for bulk throughput.
 * fillComplex<float>() takes the log in simd vectors through gr::meta::log, within 2 ulp of the exact log.
 * It draws the same points as complexSample() and leaves the generator in the same state.
 * @see Marsaglia polar method: https://en.wikipedia.org/wiki/Marsaglia_polar_method
 */
template<std::floating_point F>
struct GaussianNoise {
    Xoshiro256pp& _rng;
    F             _spare{};
    bool          _hasSpare = false;

    explicit constexpr GaussianNoise(Xoshiro256pp& rng) noexcept : _rng(rng) {}

    void reset() noexcept { _hasSpare = false; }

    constexpr F operator()() noexcept {
        if (_hasSpare) {
            _hasSpare = false;
            return _spare;
        }

        F u, v, s;
        do {
            u = _rng.uniformM11<F>();
            v = _rng.uniformM11<F>();
            s = u * u + v * v;
        } while (s >= F(1) || s == F(0));

        const F factor = std::sqrt(F(-2) * std::log(s) / s);
        _spare         = v * factor;
        _hasSpare      = true;
        return u * factor;
    }

    constexpr std::complex<F> complexSample() noexcept {
        constexpr F scale = F(1) / std::numbers::sqrt2_v<F>;
        return {(*this)() * scale, (*this)() * scale};
    }

    void fill(std::span<F> out, F amplitude = F(1), F offset = F(0)) noexcept {
        auto s0 = _rng._state[0], s1 = _rng._state[1], s2 = _rng._state[2], s3 = _rng._state[3];
        // the polar method yields variates in pairs, so a call ending on an odd count leaves a spare; resuming
        // from it keeps the draw sequence the same however the stream is split into calls
        bool hasSpare = _hasSpare;
        F    spare    = _spare;
        for (auto& sample : out) {
            if (hasSpare) {
                hasSpare = false;
                sample   = amplitude * spare + offset;
                continue;
            }
            F u, v, s;
            do {
                u = Xoshiro256pp::toUniformM11<F>(Xoshiro256pp::next(s0, s1, s2, s3));
                v = Xoshiro256pp::toUniformM11<F>(Xoshiro256pp::next(s0, s1, s2, s3));
                s = u * u + v * v;
            } while (s >= F(1) || s == F(0));
            const F factor = std::sqrt(F(-2) * std::log(s) / s);
            spare          = v * factor;
            hasSpare       = true;
            sample         = amplitude * (u * factor) + offset;
        }
        _rng._state[0] = s0;
        _rng._state[1] = s1;
        _rng._state[2] = s2;
        _rng._state[3] = s3;
        _hasSpare      = hasSpare;
        _spare         = spare;
    }

    void fillComplex(std::span<std::complex<F>> out, F amplitude = F(1), F offset = F(0)) noexcept {
        if constexpr (std::same_as<F, float>) {
            fillComplexVectors(out, amplitude, offset, _rng._state);
        } else {
            constexpr F scale = F(1) / std::numbers::sqrt2_v<F>;
            auto        s0 = _rng._state[0], s1 = _rng._state[1], s2 = _rng._state[2], s3 = _rng._state[3];
            for (auto& sample : out) {
                const auto [g1, g2] = polarPair(s0, s1, s2, s3);
                sample              = {amplitude * (g1 * scale) + offset, amplitude * (g2 * scale)};
            }
            _rng._state[0] = s0;
            _rng._state[1] = s1;
            _rng._state[2] = s2;
            _rng._state[3] = s3;
        }
        _hasSpare = false;
    }

private:
    // the accepted point (u, v) of one rejection cycle and s = u^2 + v^2
    struct PolarDraw {
        F u;
        F v;
        F s;
    };

    static PolarDraw polarDraw(std::uint64_t& s0, std::uint64_t& s1, std::uint64_t& s2, std::uint64_t& s3) noexcept {
        F u, v, s;
        do {
            u = Xoshiro256pp::toUniformM11<F>(Xoshiro256pp::next(s0, s1, s2, s3));
            v = Xoshiro256pp::toUniformM11<F>(Xoshiro256pp::next(s0, s1, s2, s3));
            s = u * u + v * v;
        } while (s >= F(1) || s == F(0));
        return {u, v, s};
    }

    static std::pair<F, F> polarPair(std::uint64_t& s0, std::uint64_t& s1, std::uint64_t& s2, std::uint64_t& s3) noexcept {
        const auto [u, v, s] = polarDraw(s0, s1, s2, s3);
        const F factor       = std::sqrt(F(-2) * std::log(s) / s);
        return {u * factor, v * factor};
    }

    // Draws up to kBlock points serially, then scales them in simd vectors. The lanes past the last point hold
    // u = v = 0 and s = 1. Each lane depends on its own point alone.
    static void fillComplexVectors(std::span<std::complex<float>> out, float amplitude, float offset, std::uint64_t (&state)[4]) noexcept {
        using V                      = vir::stdx::native_simd<float>;
        constexpr std::size_t kBlock = 256UZ;
        static_assert(kBlock % V::size() == 0UZ);
        constexpr float           scale = 1.f / std::numbers::sqrt2_v<float>;
        std::array<float, kBlock> u{};
        std::array<float, kBlock> v{};
        std::array<float, kBlock> s{};
        auto                      s0 = state[0], s1 = state[1], s2 = state[2], s3 = state[3];
        for (std::size_t at = 0UZ; at < out.size(); at += kBlock) {
            const std::size_t n = std::min(kBlock, out.size() - at);
            for (std::size_t k = 0UZ; k < n; ++k) {
                const PolarDraw draw = polarDraw(s0, s1, s2, s3);
                u[k]                 = draw.u;
                v[k]                 = draw.v;
                s[k]                 = draw.s;
            }
            const std::size_t padded = (n + V::size() - 1UZ) / V::size() * V::size();
            std::fill(u.begin() + static_cast<std::ptrdiff_t>(n), u.begin() + static_cast<std::ptrdiff_t>(padded), 0.f);
            std::fill(v.begin() + static_cast<std::ptrdiff_t>(n), v.begin() + static_cast<std::ptrdiff_t>(padded), 0.f);
            std::fill(s.begin() + static_cast<std::ptrdiff_t>(n), s.begin() + static_cast<std::ptrdiff_t>(padded), 1.f);
            for (std::size_t k = 0UZ; k < padded; k += V::size()) {
                const V r(s.data() + k, vir::stdx::element_aligned);
                const V factor = vir::stdx::sqrt(V(-2.f) * gr::meta::log(r) / r);
                const V re     = V(amplitude) * ((V(u.data() + k, vir::stdx::element_aligned) * factor) * V(scale)) + V(offset);
                const V im     = V(amplitude) * ((V(v.data() + k, vir::stdx::element_aligned) * factor) * V(scale));
                re.copy_to(u.data() + k, vir::stdx::element_aligned);
                im.copy_to(v.data() + k, vir::stdx::element_aligned);
            }
            for (std::size_t k = 0UZ; k < n; ++k) {
                out[at + k] = {u[k], v[k]};
            }
        }
        state[0] = s0;
        state[1] = s1;
        state[2] = s2;
        state[3] = s3;
    }
};

} // namespace gr::rng

#endif // GNURADIO_ALGORITHM_GAUSSIAN_NOISE_HPP
