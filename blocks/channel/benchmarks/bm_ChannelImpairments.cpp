#include <complex>
#include <cstdint>
#include <span>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/channel/Awgn.hpp>
#include <gnuradio-4.0/channel/CarrierImpairments.hpp>
#include <gnuradio-4.0/channel/Nonlinearity.hpp>
#include <gnuradio-4.0/channel/Quantizer.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

namespace {

using gr::blocks::channel::AwgnChannel;
using gr::blocks::channel::IqImbalance;
using gr::blocks::channel::Nonlinearity;
using gr::blocks::channel::PhaseNoise;
using gr::blocks::channel::Quantizer;
using CF = std::complex<float>;

constexpr std::size_t kSamplesPerCall = 65536UZ;
constexpr std::size_t kRepeats        = 9UZ;
constexpr float       kSampleRate     = 1.0e6f;

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

template<typename T>
[[nodiscard]] std::vector<T> noise() {
    std::vector<T> data(kSamplesPerCall);
    std::uint64_t  state = 0x243f6a8885a308d3ULL;
    const auto     next  = [&state] {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return 0.5f * (2.f * static_cast<float>(state % 1024ULL) / 1024.f - 1.f);
    };
    for (T& sample : data) {
        if constexpr (std::is_same_v<T, CF>) {
            const float re = next();
            sample         = CF(re, next());
        } else {
            sample = next();
        }
    }
    return data;
}

/// The per-sample blocks are driven through the same span the bulk blocks take, so every arm reports the cost of
/// covering one call's worth of samples and the two shapes are directly comparable.
template<typename TBlock, typename T>
[[nodiscard]] double runPerSample(const TBlock& block, std::span<const T> input, std::span<T> output) {
    for (std::size_t k = 0UZ; k < input.size(); ++k) {
        output[k] = block.processOne(input[k]);
    }
    return static_cast<double>(std::real(output[kSamplesPerCall / 2UZ]));
}

} // namespace

int main() {
    const std::vector<CF>    complexInput = noise<CF>();
    const std::vector<float> realInput    = noise<float>();
    std::vector<CF>          complexOutput(kSamplesPerCall);
    std::vector<float>       realOutput(kSamplesPerCall);

    AwgnChannel<CF>    awgnComplex = make<AwgnChannel<CF>>({{"noise_power", 0.01}, {"seed", std::uint64_t{12345ULL}}});
    AwgnChannel<float> awgnReal    = make<AwgnChannel<float>>({{"noise_power", 0.01}, {"seed", std::uint64_t{12345ULL}}});
    PhaseNoise<CF>     phaseNoise  = make<PhaseNoise<CF>>({{"sample_rate", kSampleRate}, {"linewidth", 100.0}, {"seed", std::uint64_t{12345ULL}}});
    awgnComplex.start();
    awgnReal.start();
    phaseNoise.start();

    const IqImbalance<CF>  imbalance = make<IqImbalance<CF>>({{"amplitude_imbalance_db", 0.5}, {"phase_imbalance", 0.02}});
    const Nonlinearity<CF> rapp      = make<Nonlinearity<CF>>({{"model", "Rapp"}, {"saturation", 1.0}, {"smoothness", 2.0}});
    const Nonlinearity<CF> saleh     = make<Nonlinearity<CF>>({{"model", "Saleh"}});
    const Quantizer<CF>    quantizer = make<Quantizer<CF>>({{"bits", gr::Size_t{12U}}, {"full_scale", 1.0}});

    std::vector<gr::blocks::testing::bench::Arm> arms{
        {"AwgnChannel, complex",
            [&] {
                std::ignore = awgnComplex.processBulk(std::span<const CF>(complexInput), std::span<CF>(complexOutput));
                return static_cast<double>(complexOutput[kSamplesPerCall / 2UZ].real());
            }},
        {"AwgnChannel, real",
            [&] {
                std::ignore = awgnReal.processBulk(std::span<const float>(realInput), std::span<float>(realOutput));
                return static_cast<double>(realOutput[kSamplesPerCall / 2UZ]);
            }},
        {"PhaseNoise",
            [&] {
                std::ignore = phaseNoise.processBulk(std::span<const CF>(complexInput), std::span<CF>(complexOutput));
                return static_cast<double>(complexOutput[kSamplesPerCall / 2UZ].real());
            }},
        {"IqImbalance", [&] { return runPerSample(imbalance, std::span<const CF>(complexInput), std::span<CF>(complexOutput)); }},
        {"Nonlinearity, Rapp", [&] { return runPerSample(rapp, std::span<const CF>(complexInput), std::span<CF>(complexOutput)); }},
        {"Nonlinearity, Saleh", [&] { return runPerSample(saleh, std::span<const CF>(complexInput), std::span<CF>(complexOutput)); }},
        {"Quantizer, 12 bits", [&] { return runPerSample(quantizer, std::span<const CF>(complexInput), std::span<CF>(complexOutput)); }},
    };

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kSamplesPerCall, kRepeats);
}
