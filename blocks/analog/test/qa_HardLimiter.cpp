#include <boost/ut.hpp>

#include <array>
#include <bit>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <random>
#include <vector>

#include <gnuradio-4.0/analog/HardLimiter.hpp>

namespace {

/// @brief Finite samples: every pair of parts from the signed zeros, one, the smallest subnormal and the smallest and
/// largest normals of either sign, then random finite bit patterns and Gaussian noise.
[[nodiscard]] std::vector<std::complex<float>> finiteSamples() {
    using L                                = std::numeric_limits<float>;
    const std::array<float, 10UZ>    parts = {0.f, -0.f, 1.f, -1.f, L::denorm_min(), -L::denorm_min(), L::min(), -L::min(), L::max(), -L::max()};
    std::vector<std::complex<float>> samples;
    for (const float re : parts) {
        for (const float im : parts) {
            samples.emplace_back(re, im);
        }
    }
    std::mt19937                                 rng(20261103U);
    std::uniform_int_distribution<std::uint32_t> bits;
    std::normal_distribution<float>              noise(0.f, 0.3f);
    while (samples.size() < 8192UZ) {
        const std::complex<float> pattern(std::bit_cast<float>(bits(rng)), std::bit_cast<float>(bits(rng)));
        if (std::isfinite(pattern.real()) && std::isfinite(pattern.imag())) {
            samples.push_back(pattern);
        }
        samples.emplace_back(noise(rng), noise(rng));
    }
    return samples;
}

/// @brief The number of float steps between `a` and `b`, the two zeros one value.
[[nodiscard]] std::int64_t floatSteps(float a, float b) {
    const auto ordered = [](float x) {
        const std::int32_t bits = std::bit_cast<std::int32_t>(x);
        return bits < 0 ? -std::int64_t{bits & 0x7fffffff} : std::int64_t{bits};
    };
    return std::abs(ordered(a) - ordered(b));
}

} // namespace

const boost::ut::suite<"HardLimiter"> hardLimiterTests = [] {
    using namespace boost::ut;
    using gr::blocks::analog::HardLimiter;
    using CF = std::complex<float>;

    static_assert(gr::HasConstProcessOneFunction<HardLimiter<CF>>, "stateless: may sit anywhere in a fused run");
    static_assert(gr::HasConstProcessOneFunction<HardLimiter<float>>, "the real form too");

    "every output has unit magnitude and the input's exact phase"_test = [] {
        HardLimiter<CF> block;
        for (const float magnitude : {1e-30f, 1e-6f, 0.001f, 0.5f, 1.0f, 42.0f, 1e12f, 1e30f}) {
            for (int step = 0; step < 16; ++step) {
                const float phase = static_cast<float>(step) * 0.3926991f - 3.0f;
                const CF    in    = std::polar(magnitude, phase);
                const CF    out   = block.processOne(in);
                expect(lt(std::abs(std::abs(out) - 1.0f), 1e-6f)) << "unit magnitude";
                expect(lt(std::abs(std::arg(out * std::conj(in))), 1e-6f)) << "the phase is untouched";
            }
        }
    };

    "zero passes as zero, never a non-finite value"_test = [] {
        HardLimiter<CF> complexBlock;
        const CF        zc = complexBlock.processOne(CF{0.f, 0.f});
        expect(eq(zc.real(), 0.f) && eq(zc.imag(), 0.f));

        HardLimiter<float> realBlock;
        expect(eq(realBlock.processOne(0.f), 0.f));
        expect(eq(realBlock.processOne(-0.f), 0.f));
    };

    "each part is within one float step of the unit sample computed in double, and a zero sample gives zero"_test = [] {
        HardLimiter<CF>       block;
        const std::vector<CF> samples       = finiteSamples();
        std::size_t           doubleOutside = 0UZ;
        std::size_t           zeros         = 0UZ;
        for (const CF sample : samples) {
            const CF out = block.processOne(sample);
            if (sample == CF{}) {
                zeros += static_cast<std::size_t>(out.real() != 0.f || out.imag() != 0.f);
                continue;
            }
            const std::complex<double> wide(sample);
            const std::complex<double> unit = wide / std::abs(wide);
            doubleOutside += static_cast<std::size_t>(floatSteps(out.real(), static_cast<float>(unit.real())) > 1 || floatSteps(out.imag(), static_cast<float>(unit.imag())) > 1);
        }
        expect(eq(zeros, 0UZ)) << "zero samples with a nonzero output";
        expect(eq(doubleOutside, 0UZ)) << "samples more than one float step from the double std result";
    };

    "the real form is the signum"_test = [] {
        HardLimiter<float> block;
        expect(eq(block.processOne(0.001f), 1.f));
        expect(eq(block.processOne(1e30f), 1.f));
        expect(eq(block.processOne(-0.001f), -1.f));
        expect(eq(block.processOne(-1e30f), -1.f));
    };
};

int main() { /* not needed for UT */ }
