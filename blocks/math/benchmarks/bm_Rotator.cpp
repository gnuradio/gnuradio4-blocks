#include <benchmark.hpp>

#include <complex>
#include <format>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/math/Rotator.hpp>

namespace {

template<typename T>
std::vector<T> generateTone(std::size_t nSamples, double frequency) {
    std::vector<T> signal(nSamples);
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        const double angle = 2. * std::numbers::pi * frequency * static_cast<double>(i);
        signal[i]          = T(static_cast<typename T::value_type>(std::cos(angle)), static_cast<typename T::value_type>(std::sin(angle)));
    }
    return signal;
}

template<typename T>
void initialize(gr::blocks::math::Rotator<T>& rotator) {
    rotator.settings().init();
    std::ignore = rotator.settings().applyStagedParameters();
}

template<typename T>
void benchmarkRotator() {
    using namespace boost::ut::reflection;

    constexpr std::size_t nSamples     = 65536UZ;
    constexpr std::size_t nRepetitions = 200UZ;
    constexpr std::size_t nWarmUp      = 50UZ;

    const std::vector<T> input = generateTone<T>(nSamples, 0.011);
    std::vector<T>       output(nSamples);

    for (std::size_t chunk : {1024UZ, 4096UZ, 16384UZ, nSamples}) {
        gr::blocks::math::Rotator<T> rotator({{"frequency_shift", 0.13f}, {"sample_rate", 1.f}});
        initialize(rotator);

        // the chunk size is opaque to the optimizer, as a scheduler's run-time chunk size is
        std::size_t opaqueChunk = chunk;
        ::benchmark::force_to_memory(opaqueChunk);

        auto pass = [&rotator, &input, &output, opaqueChunk] {
            for (std::size_t i = 0UZ; i < nSamples; i += opaqueChunk) {
                std::ignore = rotator.processBulk(std::span<const T>(input.data() + i, opaqueChunk), std::span<T>(output.data() + i, opaqueChunk));
            }
            ::benchmark::force_to_memory(output);
        };

        for (std::size_t i = 0UZ; i < nWarmUp; ++i) {
            pass();
        }
        ::benchmark::benchmark<nRepetitions>(std::format("{} - {} samples per call", type_name<T>(), chunk), nSamples) = pass;
    }

    ::benchmark::results::add_separator();
}

} // namespace

inline const boost::ut::suite _rotator_bm_tests = [] {
    benchmarkRotator<std::complex<float>>();
    benchmarkRotator<std::complex<double>>();
};

int main() { /* not needed by the UT framework */ }
