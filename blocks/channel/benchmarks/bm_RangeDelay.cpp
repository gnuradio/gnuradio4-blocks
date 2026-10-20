#include <complex>
#include <cstdint>
#include <memory>
#include <span>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/algorithm/filter/FractionalDelay.hpp>
#include <gnuradio-4.0/algorithm/timing/FrequencySchedule.hpp>
#include <gnuradio-4.0/algorithm/timing/SampleClock.hpp>
#include <gnuradio-4.0/channel/RangeDelay.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

namespace {

using gr::blocks::channel::RangeDelay;
using CF = std::complex<float>;

constexpr std::size_t kSamplesPerCall = 65536UZ;
constexpr std::size_t kRepeats        = 9UZ;
constexpr float       kSampleRate     = 1.0e6f;

/// A hundred samples of delay moving to a hundred and fifty over one second. No orbit reaches that range rate.
/// It bounds how hard the schedule walk is driven.
const std::vector<std::int64_t> kTimes{0LL, 1'000'000'000LL};
const std::vector<double>       kDelays{1.0e-4, 1.5e-4};

/// The block carries a measurement slot, so it lives where it is built.
[[nodiscard]] std::unique_ptr<RangeDelay<CF>> make(std::size_t bank, int order) {
    auto block = std::make_unique<RangeDelay<CF>>(gr::property_map{{"schedule_times_ns", kTimes}, {"schedule_delays_s", kDelays}, {"sample_rate", kSampleRate}, {"order", order}, {"bank_size", gr::Size_t{static_cast<std::uint32_t>(bank)}}});
    block->settings().init();
    std::ignore = block->settings().applyStagedParameters();
    block->start();
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

    auto nearest = make(32UZ, 0);
    auto linear  = make(32UZ, 1);
    auto cubic   = make(32UZ, 3);
    auto wide    = make(128UZ, 1);

    // The schedule walk and the conversion of its seconds to fixed-point samples, without interpolation. This is
    // the share of the per-sample cost outside the bank.
    const gr::timing::DelaySchedule schedule{std::span<const std::int64_t>(kTimes), std::span<const double>(kDelays)};
    const gr::timing::SampleClock   clock = gr::timing::clockForRateHz(static_cast<double>(kSampleRate), 0ULL, 0LL).value();
    std::vector<double>             seconds(kSamplesPerCall);
    std::vector<std::uint64_t>      fixed(kSamplesPerCall);
    std::uint64_t                   walkAt = 0ULL;

    std::vector<gr::blocks::testing::bench::Arm> arms{
        {"RangeDelay, L = 32, q = 0",
            [&] {
                std::ignore = nearest->processBulk(std::span<const CF>(input), std::span<CF>(output));
                return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
            }},
        {"RangeDelay, L = 32, q = 1",
            [&] {
                std::ignore = linear->processBulk(std::span<const CF>(input), std::span<CF>(output));
                return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
            }},
        {"RangeDelay, L = 32, q = 3",
            [&] {
                std::ignore = cubic->processBulk(std::span<const CF>(input), std::span<CF>(output));
                return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
            }},
        {"RangeDelay, L = 128, q = 1",
            [&] {
                std::ignore = wide->processBulk(std::span<const CF>(input), std::span<CF>(output));
                return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
            }},
        {"the schedule walk and the Q32 conversion alone",
            [&] {
                schedule.valuesFor(clock, walkAt, std::span<double>(seconds));
                std::ignore = gr::filter::fractionalDelayQ32(std::span<const double>(seconds), static_cast<double>(kSampleRate), std::span<std::uint64_t>(fixed));
                walkAt += kSamplesPerCall;
                return static_cast<double>(fixed[kSamplesPerCall / 2UZ]);
            }},
    };

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kSamplesPerCall, kRepeats);
}
