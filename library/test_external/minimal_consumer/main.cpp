#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include <gnuradio-4.0/MemoryAllocators.hpp>
#include <gnuradio-4.0/algorithm/fourier/SimdFFT.hpp>
#include <gnuradio-4.0/algorithm/rng/Xoshiro256pp.hpp>

// SimdFFT's public headers include vir-simd. This program needs an installation that provides them.
int main() {
    gr::rng::Xoshiro256pp rng(0);
    if (rng() != std::uint64_t{0x53175d61490b23dfULL}) {
        return 1;
    }

    // the transform of a unit impulse is one in every bin
    gr::algorithm::SimdFFT<float, gr::algorithm::Transform::Complex, 64> fft;
    std::vector<float, gr::allocator::Aligned<float>>                    in(128UZ);
    std::vector<float, gr::allocator::Aligned<float>>                    out(128UZ);
    in[0] = 1.0f;
    fft.transform(gr::algorithm::forward, gr::algorithm::ordered, in, out);
    for (std::size_t k = 0UZ; k < 64UZ; ++k) {
        if (std::abs(out[2UZ * k] - 1.0f) > 1e-4f || std::abs(out[2UZ * k + 1UZ]) > 1e-4f) {
            return 2;
        }
    }
    return 0;
}
