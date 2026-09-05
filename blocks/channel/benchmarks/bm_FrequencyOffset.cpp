#include <complex>
#include <cstdint>
#include <numbers>
#include <span>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/algorithm/signal/Phasor.hpp>
#include <gnuradio-4.0/channel/CarrierImpairments.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

namespace {

using gr::blocks::channel::FrequencyOffset;
using CF = std::complex<float>;

constexpr std::size_t kSamplesPerCall = 65536UZ;
constexpr std::size_t kRepeats        = 9UZ;
constexpr float       kSampleRate     = 1.0e6f;
constexpr double      kOffsetHz       = 1.0e3;
constexpr double      kDriftHzPerS    = 50.0;

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

[[nodiscard]] std::vector<CF> noise() {
    std::vector<CF> data(kSamplesPerCall);
    std::uint64_t   state = 0x243f6a8885a308d3ULL;
    for (CF& sample : data) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        sample = CF(2.f * static_cast<float>(state % 1024ULL) / 1024.f - 1.f, 2.f * static_cast<float>((state >> 20U) % 1024ULL) / 1024.f - 1.f);
    }
    return data;
}

} // namespace

int main() {
    const std::vector<CF> input = noise();
    std::vector<CF>       output(kSamplesPerCall);

    FrequencyOffset<CF> steady  = make<FrequencyOffset<CF>>({{"sample_rate", kSampleRate}, {"frequency_offset", kOffsetHz}, {"drift", 0.0}});
    FrequencyOffset<CF> drifted = make<FrequencyOffset<CF>>({{"sample_rate", kSampleRate}, {"frequency_offset", kOffsetHz}, {"drift", kDriftHzPerS}});

    // the schedule the drift path used to build and hand to fillModulated sample by sample, kept as an arm so the
    // cost it replaced is measured in the same process as the cost that replaced it
    constexpr double    twoPi = 2. * std::numbers::pi_v<double>;
    const double        fs    = static_cast<double>(kSampleRate);
    std::vector<double> increments(kSamplesPerCall);
    for (std::size_t k = 0UZ; k < kSamplesPerCall; ++k) {
        increments[k] = twoPi * (kOffsetHz + kDriftHzPerS * ((static_cast<double>(k) + 0.5) / fs)) / fs;
    }
    gr::signal::Phasor<float> modulated;
    modulated.configure(twoPi * kOffsetHz / fs, 0.);

    std::vector<gr::blocks::testing::bench::Arm> arms{
        {"FrequencyOffset, no drift (plain mix)",
            [&] {
                std::ignore = steady.processBulk(std::span<const CF>(input), std::span<CF>(output));
                return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
            }},
        {"FrequencyOffset, drift (chirp)",
            [&] {
                std::ignore = drifted.processBulk(std::span<const CF>(input), std::span<CF>(output));
                return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
            }},
        {"the schedule it replaced (mixModulated)",
            [&] {
                modulated.mixModulated(std::span<const double>(increments), std::span<const CF>(input), std::span<CF>(output));
                return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
            }},
    };

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kSamplesPerCall, kRepeats);
}
