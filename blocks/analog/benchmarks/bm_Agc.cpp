#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <cmath>
#include <complex>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/analog/Agc.hpp>

namespace {

constexpr std::size_t kSamplesPerCall = 65536UZ;
constexpr std::size_t kIterations     = 512UZ;

std::vector<std::complex<float>> makeFadingBurst() {
    std::vector<std::complex<float>> signal(kSamplesPerCall);
    for (std::size_t i = 0UZ; i < signal.size(); ++i) {
        const double magnitude = 0.2 + 0.8 * std::abs(std::sin(2.0 * std::numbers::pi * 7.0 * static_cast<double>(i) / static_cast<double>(kSamplesPerCall)));
        const double phase     = 2.0 * std::numbers::pi * 0.031 * static_cast<double>(i);
        signal[i]              = std::complex<float>(static_cast<float>(magnitude * std::cos(phase)), static_cast<float>(magnitude * std::sin(phase)));
    }
    return signal;
}

gr::blocks::analog::Agc<std::complex<float>> makeAgc(gr::property_map settings) {
    gr::blocks::analog::Agc<std::complex<float>> agc(std::move(settings));
    agc.settings().init();
    std::ignore = agc.settings().applyStagedParameters();
    return agc;
}

} // namespace

int main() {
    const std::vector<std::complex<float>> input = makeFadingBurst();
    std::vector<std::complex<float>>       output(kSamplesPerCall);

    auto tracking = makeAgc({{"sample_rate", 48000.f}, {"attack_s", 0.005}, {"decay_s", 0.5}});
    gr::blocks::testing::bench::report("Agc<complex<float>>", kSamplesPerCall, kIterations, [&tracking, &input, &output] {
        std::ignore = tracking.processBulk(std::span<const std::complex<float>>(input), std::span<std::complex<float>>(output));
        return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
    });

    auto manual = makeAgc({{"enabled", false}, {"gain_db", 6.0}});
    gr::blocks::testing::bench::report("Agc<complex<float>> manual gain", kSamplesPerCall, kIterations, [&manual, &input, &output] {
        std::ignore = manual.processBulk(std::span<const std::complex<float>>(input), std::span<std::complex<float>>(output));
        return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
    });
}
