#include <cmath>
#include <complex>
#include <cstdint>
#include <span>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/channel/CarrierImpairments.hpp>
#include <gnuradio-4.0/channel/DopplerShift.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

namespace {

using gr::blocks::channel::DopplerShift;
using gr::blocks::channel::FrequencyOffset;
using CF = std::complex<float>;

constexpr std::size_t kSamplesPerCall = 65536UZ;
constexpr std::size_t kRepeats        = 9UZ;
constexpr float       kSampleRate     = 1.0e6f;
constexpr double      kOffsetHz       = 1.0e3;
constexpr double      kDriftHzPerS    = 50.0;

/// Built in place. A block with a measurement slot is not movable and lives behind a pointer.
template<typename TBlock>
[[nodiscard]] std::unique_ptr<TBlock> make(gr::property_map settings) {
    auto block = std::make_unique<TBlock>(std::move(settings));
    block->settings().init();
    std::ignore = block->settings().applyStagedParameters();
    if constexpr (requires { block->start(); }) { // DopplerShift takes its origin in start(), and FrequencyOffset has none
        block->start();
    }
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

/// A ten-minute pass shape at @p knots knots. The curve is the same at every table size. The per-sample cost can
/// then be compared across table sizes.
[[nodiscard]] gr::property_map schedule(std::size_t knots) {
    std::vector<std::int64_t> times(knots);
    std::vector<double>       offsets(knots);
    for (std::size_t k = 0UZ; k < knots; ++k) {
        const double t = 600. * static_cast<double>(k) / static_cast<double>(knots - 1UZ);
        times[k]       = static_cast<std::int64_t>(std::llround(t * 1e9));
        offsets[k]     = -10'000. * std::tanh((t - 300.) / 60.);
    }
    return {{"sample_rate", kSampleRate}, {"schedule_times_ns", times}, {"schedule_offsets_hz", offsets}, {"direction", std::string("apply")}};
}

} // namespace

int main() {
    const std::vector<CF> input = noise();
    std::vector<CF>       output(kSamplesPerCall);

    auto steady  = make<FrequencyOffset<CF>>({{"sample_rate", kSampleRate}, {"frequency_offset", kOffsetHz}, {"drift", 0.0}});
    auto drifted = make<FrequencyOffset<CF>>({{"sample_rate", kSampleRate}, {"frequency_offset", kOffsetHz}, {"drift", kDriftHzPerS}});
    auto two     = make<DopplerShift<CF>>(schedule(2UZ));
    auto dense   = make<DopplerShift<CF>>(schedule(1000UZ));

    std::vector<gr::blocks::testing::bench::Arm> arms{
        {"FrequencyOffset, no drift (plain mix)",
            [&] {
                std::ignore = steady->processBulk(std::span<const CF>(input), std::span<CF>(output));
                return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
            }},
        {"FrequencyOffset, drift (chirp)",
            [&] {
                std::ignore = drifted->processBulk(std::span<const CF>(input), std::span<CF>(output));
                return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
            }},
        {"DopplerShift, 2-knot schedule",
            [&] {
                std::ignore = two->processBulk(std::span<const CF>(input), std::span<CF>(output));
                return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
            }},
        {"DopplerShift, 1000-knot schedule",
            [&] {
                std::ignore = dense->processBulk(std::span<const CF>(input), std::span<CF>(output));
                return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
            }},
    };

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kSamplesPerCall, kRepeats);
}
