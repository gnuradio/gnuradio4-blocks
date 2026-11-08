#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <cmath>
#include <complex>
#include <concepts>
#include <format>
#include <numbers>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/analog/Agc.hpp>

namespace {

constexpr std::size_t kSamplesPerCall = 65536UZ;
constexpr std::size_t kIterations     = 512UZ;

template<typename T>
std::vector<T> makeFadingBurst() {
    std::vector<T> signal(kSamplesPerCall);
    for (std::size_t i = 0UZ; i < signal.size(); ++i) {
        const double magnitude = 0.2 + 0.8 * std::abs(std::sin(2.0 * std::numbers::pi * 7.0 * static_cast<double>(i) / static_cast<double>(kSamplesPerCall)));
        const double phase     = 2.0 * std::numbers::pi * 0.031 * static_cast<double>(i);
        if constexpr (std::same_as<T, float>) {
            signal[i] = static_cast<float>(magnitude * std::cos(phase));
        } else {
            signal[i] = T(static_cast<float>(magnitude * std::cos(phase)), static_cast<float>(magnitude * std::sin(phase)));
        }
    }
    return signal;
}

template<typename T>
gr::blocks::analog::Agc<T> makeAgc(gr::property_map settings) {
    gr::blocks::analog::Agc<T> agc(std::move(settings));
    agc.settings().init();
    std::ignore = agc.settings().applyStagedParameters();
    return agc;
}

template<typename T>
void benchType(const char* typeName) {
    const std::vector<T> input = makeFadingBurst<T>();
    std::vector<T>       output(kSamplesPerCall);

    const auto run = [&](const std::string& label, gr::property_map settings) {
        auto agc = makeAgc<T>(std::move(settings));
        gr::blocks::testing::bench::report(label, kSamplesPerCall, kIterations, [&agc, &input, &output] {
            std::ignore = agc.processBulk(std::span<const T>(input), std::span<T>(output));
            return static_cast<double>(std::abs(output[kSamplesPerCall / 2UZ]));
        });
    };
    run(std::format("Agc<{}>", typeName), {{"sample_rate", 48000.f}, {"attack_s", 0.005}, {"decay_s", 0.5}});
    run(std::format("Agc<{}> decimation 4", typeName), {{"sample_rate", 48000.f}, {"attack_s", 0.005}, {"decay_s", 0.5}, {"update_decimation", gr::Size_t(4)}});
    run(std::format("Agc<{}> manual gain", typeName), {{"enabled", false}, {"gain_db", 6.0}});
}

} // namespace

int main() {
    benchType<std::complex<float>>("complex<float>");
    benchType<float>("float");
}
