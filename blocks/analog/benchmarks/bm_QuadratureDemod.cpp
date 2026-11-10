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

template<typename T>
std::vector<std::complex<T>> makeToneBurst() {
    std::vector<std::complex<T>> signal(kSamplesPerCall);
    for (std::size_t i = 0UZ; i < signal.size(); ++i) {
        const double phase = 2.0 * std::numbers::pi * 0.037 * static_cast<double>(i);
        signal[i]          = std::complex<T>(static_cast<T>(std::cos(phase)), static_cast<T>(std::sin(phase)));
    }
    return signal;
}

template<typename T>
void benchmark(const char* name) {
    const std::vector<std::complex<T>>     input = makeToneBurst<T>();
    std::vector<T>                         output(kSamplesPerCall);
    gr::blocks::analog::QuadratureDemod<T> demod;
    demod.settings().init();
    std::ignore = demod.settings().applyStagedParameters();

    gr::blocks::testing::bench::report(name, kSamplesPerCall, kIterations, [&demod, &input, &output] {
        std::ignore = demod.processBulk(std::span<const std::complex<T>>(input), std::span<T>(output));
        return static_cast<double>(output[kSamplesPerCall / 2UZ]);
    });
}

} // namespace

int main() {
    benchmark<float>("QuadratureDemod<float>");
    benchmark<double>("QuadratureDemod<double>");
}
