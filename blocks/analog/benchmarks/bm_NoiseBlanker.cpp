#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/analog/NoiseBlanker.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::analog::NoiseBlanker;
using gr::blocks::testing::bench::Arm;

namespace test = gr::blocks::testing::span;

using CF = std::complex<float>;

constexpr std::size_t kSamples = 1UZ << 22;
constexpr std::size_t kChunk   = 4096UZ;
constexpr std::size_t kRepeats = 7UZ;

[[nodiscard]] NoiseBlanker<CF> make(gr::property_map settings) {
    NoiseBlanker<CF> block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

/// The two shapes this block rejects, kept as arms so neither the square root nor the uncensored tracker can return unnoticed.
struct Reference {
    double alpha;
    double threshold;
    double power = 0.0;
    bool   root  = false;

    void sweep(std::span<const CF> input, std::span<CF> output) noexcept {
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            const double re    = static_cast<double>(input[i].real());
            const double im    = static_cast<double>(input[i].imag());
            const double value = root ? std::sqrt(re * re + im * im) : re * re + im * im;
            power += alpha * (value - power); // uncensored and before the comparison, the order that desensitizes
            output[i] = value > threshold * power ? CF{} : input[i];
        }
    }
};

} // namespace

int main() {
    std::vector<CF> x(kSamples);
    std::uint64_t   state = 0x243f6a8885a308d3ULL;
    for (CF& sample : x) {
        state += 0x9e3779b97f4a7c15ULL;
        std::uint64_t z = state;
        z               = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z               = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        z ^= z >> 31;
        const double phase = 2.0 * std::numbers::pi * static_cast<double>(z >> 11) * 0x1.0p-53;
        sample             = CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
    }
    std::vector<CF> y(kSamples);

    NoiseBlanker<CF> full = make({{"enabled", true}, {"sample_rate", 96000.f}});
    Reference        magnitude{full._alpha, std::sqrt(full._threshold), 0.0, true};
    Reference        squared{full._alpha, full._threshold, 0.0, false};

    std::vector<Arm> arms;
    arms.emplace_back("plain span copy, the 1:1 floor", kSamples, [&x, &y] {
        std::copy_n(x.begin(), kSamples, y.begin());
        return static_cast<double>(y[kSamples / 2UZ].real());
    });
    arms.emplace_back("nb_abs: std::abs, uncensored tracker", kSamples, [&magnitude, &x, &y] {
        magnitude.sweep(std::span<const CF>(x), std::span<CF>(y));
        return static_cast<double>(y[kSamples / 2UZ].real());
    });
    arms.emplace_back("nb_sq: squared magnitude, same shape", kSamples, [&squared, &x, &y] {
        squared.sweep(std::span<const CF>(x), std::span<CF>(y));
        return static_cast<double>(y[kSamples / 2UZ].real());
    });
    arms.emplace_back("nb_full: the block, with the censored tracker", kSamples, [&full, &x, &y] {
        for (std::size_t base = 0UZ; base < kSamples; base += kChunk) {
            test::InputSpan<CF>  inSpan(std::span<const CF>(x).subspan(base, kChunk), base);
            test::OutputSpan<CF> outSpan(std::span<CF>(y).subspan(base, kChunk), base);
            std::ignore = full.processBulk(inSpan, outSpan);
        }
        return static_cast<double>(y[kSamples / 2UZ].real());
    });

    gr::blocks::testing::bench::report(std::span<Arm>(arms), kRepeats);
}
