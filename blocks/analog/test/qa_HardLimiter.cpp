#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <random>
#include <span>
#include <tuple>
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

/// @brief The block's output for `in`, through its bulk function in pieces of `chunk` samples.
[[nodiscard]] std::vector<std::complex<float>> limit(const std::vector<std::complex<float>>& in, std::size_t chunk) {
    gr::blocks::analog::HardLimiter<std::complex<float>> block;
    std::vector<std::complex<float>>                     out(in.size());
    for (std::size_t base = 0UZ; base < in.size(); base += chunk) {
        const std::size_t count = std::min(chunk, in.size() - base);
        std::ignore             = block.processBulk(std::span<const std::complex<float>>(in).subspan(base, count), std::span<std::complex<float>>(out).subspan(base, count));
    }
    return out;
}

/// @brief The unit sample with the parts and their squares in double, the exact result for a sample whose float square is out of range.
[[nodiscard]] std::complex<float> doubleUnit(std::complex<float> sample) {
    const double re  = static_cast<double>(sample.real());
    const double im  = static_cast<double>(sample.imag());
    const double sum = re * re + im * im;
    if (!(sum > 0.0)) {
        return {};
    }
    const double inverse = 1.0 / std::sqrt(sum);
    return {static_cast<float>(re * inverse), static_cast<float>(im * inverse)};
}

[[nodiscard]] bool sameBits(float a, float b) { return std::bit_cast<std::uint32_t>(a) == std::bit_cast<std::uint32_t>(b); }

/// @brief `count` samples whose squared magnitude is `sum`, at random phases, then the two on the axes.
[[nodiscard]] std::vector<std::complex<float>> ring(double sum, std::size_t count, std::mt19937& rng) {
    std::uniform_real_distribution<double> phase(-3.14159265358979, 3.14159265358979);
    const double                           magnitude = std::sqrt(sum);
    std::vector<std::complex<float>>       samples;
    for (std::size_t i = 0UZ; i < count; ++i) {
        const double angle = phase(rng);
        samples.emplace_back(static_cast<float>(magnitude * std::cos(angle)), static_cast<float>(magnitude * std::sin(angle)));
    }
    samples.emplace_back(static_cast<float>(magnitude), 0.f);
    samples.emplace_back(0.f, -static_cast<float>(magnitude));
    return samples;
}

/// @brief The largest distance in float steps of a part of `out` from the same part of the unit sample computed in double.
[[nodiscard]] std::int64_t worstSteps(const std::vector<std::complex<float>>& in, const std::vector<std::complex<float>>& out) {
    std::int64_t worst = 0;
    for (std::size_t i = 0UZ; i < in.size(); ++i) {
        const std::complex<double> wide(in[i]);
        const std::complex<double> unit = wide / std::abs(wide);
        worst                           = std::max({worst, floatSteps(out[i].real(), static_cast<float>(unit.real())), floatSteps(out[i].imag(), static_cast<float>(unit.imag()))});
    }
    return worst;
}

} // namespace

const boost::ut::suite<"HardLimiter"> hardLimiterTests = [] {
    using namespace boost::ut;
    using gr::blocks::analog::HardLimiter;
    using CF = std::complex<float>;

    static_assert(gr::HasProcessBulkFunction<HardLimiter<CF>>, "the complex<float> form runs over a span");
    static_assert(gr::HasConstProcessOneFunction<HardLimiter<float>>, "the real form too");

    "every output has unit magnitude and the input's phase, from the subnormal range to the top of the float range"_test = [] {
        const float        tiny = std::numeric_limits<float>::denorm_min();
        const float        top  = std::numeric_limits<float>::max();
        std::vector<CF>    in;
        std::vector<float> magnitudes = {tiny, 3.f * tiny, 1e-40f, 1e-30f, 1e-20f, 1e-15f, 1e-6f, 0.001f, 0.5f, 1.0f, 42.0f, 1e12f, 1e15f, 1e20f, 1e30f, 1e38f, 0.5f * top};
        for (const float magnitude : magnitudes) {
            for (int step = 0; step < 16; ++step) {
                const float phase = static_cast<float>(step) * 0.3926991f - 3.0f;
                // a subnormal magnitude holds few bits: its parts are built, then the exact phase is read back
                in.emplace_back(magnitude * std::cos(phase), magnitude * std::sin(phase));
            }
        }
        in.emplace_back(top, top);
        in.emplace_back(-top, top);
        const std::vector<CF> out = limit(in, in.size());
        for (std::size_t i = 0UZ; i < in.size(); ++i) {
            const std::complex<double> wide(in[i]);
            if (wide == std::complex<double>{}) {
                continue; // a magnitude below the smallest subnormal rounds both parts to zero
            }
            const std::complex<double> unit(out[i]);
            expect(lt(std::abs(std::abs(unit) - 1.0), 3e-7)) << "unit magnitude within 3e-7, sample" << i;
            const std::complex<double> reference = wide / std::abs(wide);
            expect(lt(std::abs(unit - reference), 5e-7)) << "the phase is kept, sample" << i;
        }
    };

    "the edge inputs give today's results: zero, subnormal, very large, infinite and NaN"_test = [] {
        const float           inf  = std::numeric_limits<float>::infinity();
        const float           nan  = std::numeric_limits<float>::quiet_NaN();
        const float           tiny = std::numeric_limits<float>::denorm_min();
        const float           top  = std::numeric_limits<float>::max();
        const std::vector<CF> in   = {CF{0.f, 0.f}, CF{-0.f, -0.f}, CF{tiny, 0.f}, CF{0.f, -tiny}, CF{tiny, tiny}, CF{top, top}, CF{-top, 0.f}, CF{nan, 1.f}, CF{1.f, nan}, CF{nan, nan}, CF{inf, 1.f}, CF{-inf, 0.f}, CF{inf, inf}, CF{0.f, -inf}, CF{1e-30f, 1e-30f}, CF{1e-20f, 0.f}};
        for (const std::size_t chunk : {std::size_t{1}, std::size_t{5}, in.size()}) {
            const std::vector<CF> out = limit(in, chunk);
            for (std::size_t i = 0UZ; i < in.size(); ++i) {
                const CF want = doubleUnit(in[i]);
                // NaN parts have no ordering: the bits of the result are compared with the double code's
                expect(sameBits(out[i].real(), want.real()) && sameBits(out[i].imag(), want.imag())) << "edge sample" << i << "chunk" << chunk;
            }
        }
        const std::vector<CF> out = limit(in, in.size());
        expect(out[0] == CF{} && out[1] == CF{}) << "zero stays zero";
        expect(eq(out[2].real(), 1.f) && eq(out[2].imag(), 0.f)) << "a subnormal part keeps its direction";
        expect(std::isnan(out[10].real()) && eq(out[10].imag(), 0.f)) << "an infinite part gives what the double code gives";
        expect(eq(out[7].real(), 0.f) && eq(out[7].imag(), 0.f)) << "a NaN sample gives zero";
    };

    "each part is within four float steps of the unit sample computed in double, and a zero sample gives zero"_test = [] {
        const std::vector<CF> samples       = finiteSamples();
        const std::vector<CF> out           = limit(samples, samples.size());
        std::size_t           doubleOutside = 0UZ;
        std::size_t           zeros         = 0UZ;
        for (std::size_t i = 0UZ; i < samples.size(); ++i) {
            if (samples[i] == CF{}) {
                zeros += static_cast<std::size_t>(out[i].real() != 0.f || out[i].imag() != 0.f);
                continue;
            }
            const std::complex<double> wide(samples[i]);
            const std::complex<double> unit = wide / std::abs(wide);
            doubleOutside += static_cast<std::size_t>(floatSteps(out[i].real(), static_cast<float>(unit.real())) > 4 || floatSteps(out[i].imag(), static_cast<float>(unit.imag())) > 4);
        }
        expect(eq(zeros, 0UZ)) << "zero samples with a nonzero output";
        expect(eq(doubleOutside, 0UZ)) << "samples more than four float steps from the double std result";
    };

    "within the range the parts are within four float steps, at sums near each end of it"_test = [] {
        std::mt19937    rng(20261113U);
        std::vector<CF> in;
        for (const double sum : {1.05e-30, 2e-30, 1e-28, 1e-20, 1e20, 1e28, 0.95e30}) {
            const std::vector<CF> part = ring(sum, 2000UZ, rng);
            in.insert(in.end(), part.begin(), part.end());
        }
        expect(le(worstSteps(in, limit(in, in.size())), 4)) << "float steps from the double result";
        expect(le(worstSteps(in, limit(in, 300UZ)), 4)) << "float steps from the double result, chunks of 300";
    };

    "the range ends at squared magnitudes 1e-30 and 1e30: outside it the double code's bits, inside it the float path"_test = [] {
        std::mt19937 rng(7U);
        const auto   doubleBits = [](const std::vector<CF>& in, const std::vector<CF>& out) {
            std::size_t same = 0UZ;
            for (std::size_t i = 0UZ; i < in.size(); ++i) {
                const CF want = doubleUnit(in[i]);
                same += static_cast<std::size_t>(sameBits(out[i].real(), want.real()) && sameBits(out[i].imag(), want.imag()));
            }
            return same;
        };
        for (const double sum : {0.9e-30, 1.1e30}) {
            const std::vector<CF> in  = ring(sum, 500UZ, rng);
            const std::vector<CF> out = limit(in, in.size());
            expect(eq(doubleBits(in, out), in.size())) << "samples outside the range that are not the double code's, sum" << sum;
        }
        for (const double sum : {1.1e-30, 0.9e30}) {
            const std::vector<CF> in  = ring(sum, 500UZ, rng);
            const std::vector<CF> out = limit(in, in.size());
            expect(le(worstSteps(in, out), 4)) << "float steps from the double result, sum" << sum;
            expect(lt(doubleBits(in, out), in.size())) << "samples inside the range are all the double code's: the float path is not taken, sum" << sum;
        }
    };

    "an output shorter than the input limits that many samples and writes nothing past it"_test = [] {
        const std::vector<CF> in    = finiteSamples();
        const std::vector<CF> whole = limit(in, in.size());
        const CF              sentinel{7.f, 7.f};
        std::vector<CF>       out(in.size(), sentinel);
        HardLimiter<CF>       block;
        std::ignore       = block.processBulk(std::span<const CF>(in), std::span<CF>(out).first(300UZ));
        std::size_t wrong = 0UZ;
        for (std::size_t i = 0UZ; i < in.size(); ++i) {
            const CF want = i < 300UZ ? whole[i] : sentinel;
            wrong += static_cast<std::size_t>(!sameBits(out[i].real(), want.real()) || !sameBits(out[i].imag(), want.imag()));
        }
        expect(eq(wrong, 0UZ)) << "samples that differ from the limited input or the untouched sentinel";
    };

    "any chunking gives the whole-span result, with an edge sample at each tile boundary"_test = [] {
        std::vector<CF> in  = finiteSamples();
        const float     nan = std::numeric_limits<float>::quiet_NaN();
        for (const std::size_t at : {std::size_t{0}, std::size_t{255}, std::size_t{256}, std::size_t{257}, std::size_t{511}, std::size_t{512}, in.size() - 1UZ}) {
            in[at] = (at % 2UZ == 0UZ) ? CF{0.f, 0.f} : CF{nan, 2.f};
        }
        const std::vector<CF> whole = limit(in, in.size());
        for (const std::size_t chunk : {std::size_t{1}, std::size_t{3}, std::size_t{255}, std::size_t{256}, std::size_t{257}, std::size_t{1000}}) {
            const std::vector<CF> pieces = limit(in, chunk);
            std::size_t           differ = 0UZ;
            for (std::size_t i = 0UZ; i < in.size(); ++i) {
                differ += static_cast<std::size_t>(!sameBits(pieces[i].real(), whole[i].real()) || !sameBits(pieces[i].imag(), whole[i].imag()));
            }
            expect(eq(differ, 0UZ)) << "samples that differ from the whole-span result, chunk" << chunk;
        }
        expect(eq(whole[0].real(), 0.f) && eq(whole[256].imag(), 0.f)) << "the zeros at tile starts";
    };

    "an empty span is a no-op"_test = [] {
        HardLimiter<CF> block;
        std::vector<CF> none;
        expect(block.processBulk(std::span<const CF>(none), std::span<CF>(none)) == gr::work::Status::OK);
    };

    "the real form is the signum"_test = [] {
        HardLimiter<float> block;
        expect(eq(block.processOne(0.f), 0.f));
        expect(eq(block.processOne(-0.f), 0.f));
        expect(eq(block.processOne(0.001f), 1.f));
        expect(eq(block.processOne(1e30f), 1.f));
        expect(eq(block.processOne(-0.001f), -1.f));
        expect(eq(block.processOne(-1e30f), -1.f));
    };
};

int main() { /* not needed for UT */ }
