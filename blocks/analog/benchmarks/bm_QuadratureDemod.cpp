#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <cmath>
#include <complex>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/analog/QuadratureDemod.hpp>

namespace {

constexpr std::size_t kSamplesPerCall = 65536UZ;
constexpr std::size_t kIterations     = 512UZ;

std::vector<std::complex<float>> makeToneBurst() {
    std::vector<std::complex<float>> signal(kSamplesPerCall);
    for (std::size_t i = 0UZ; i < signal.size(); ++i) {
        const double phase = 2.0 * std::numbers::pi * 0.037 * static_cast<double>(i);
        signal[i]          = std::complex<float>(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
    }
    return signal;
}

} // namespace

int main() {
    const std::vector<std::complex<float>>     input = makeToneBurst();
    std::vector<float>                         output(kSamplesPerCall);
    gr::blocks::analog::QuadratureDemod<float> demod;
    demod.settings().init();
    std::ignore = demod.settings().applyStagedParameters();

    gr::blocks::testing::bench::report("QuadratureDemod<float>", kSamplesPerCall, kIterations, [&demod, &input, &output] {
        std::ignore = demod.processBulk(std::span<const std::complex<float>>(input), std::span<float>(output));
        return static_cast<double>(output[kSamplesPerCall / 2UZ]);
    });
}
