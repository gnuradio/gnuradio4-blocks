#include <benchmark.hpp>

#include <gnuradio-4.0/algorithm/timing/FrequencySchedule.hpp>
#include <gnuradio-4.0/algorithm/timing/SampleClock.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <string_view>
#include <vector>

namespace {

/// A pass-shaped table over ten minutes, in the form a LEO Doppler profile takes as schedule input.
struct Profile {
    std::vector<std::int64_t> times;
    std::vector<double>       offsets;
};

constexpr std::size_t kSamples = 65'536UZ;
constexpr std::size_t kRepeats = 200UZ;

[[nodiscard]] Profile leoProfile(std::size_t nKnots) {
    constexpr std::int64_t kSpanNs = 600'000'000'000LL;

    Profile profile;
    profile.times.resize(nKnots);
    profile.offsets.resize(nKnots);
    for (std::size_t i = 0UZ; i < nKnots; ++i) {
        const double u     = static_cast<double>(i) / static_cast<double>(nKnots - 1UZ);
        profile.times[i]   = static_cast<std::int64_t>(static_cast<double>(kSpanNs) * u);
        profile.offsets[i] = -10'000. * std::tanh(6. * (u - 0.5));
    }
    return profile;
}

} // namespace

/// The benchmark tests that the per-sample cost does not depend on the size of the table. The segment walk is a
/// cursor over times that only increase. It advances at most once per knot over a whole span. A per-sample binary
/// search would cost log2(knots) per sample, and the rows for 100 and 10000 knots would differ by a clear step. Each
/// row is one sample's worth of work, and the three rows are directly comparable.
void benchPhaseIncrements() {
    using namespace benchmark;
    using boost::ut::operator""_test;

    "phaseIncrementsFor: flat in the knot count"_test = [] {
        const gr::timing::SampleClock clock(0ULL, 0LL, 48'000ULL, 1ULL);
        std::vector<double>           increments(kSamples);

        for (const std::size_t nKnots : {100UZ, 1'000UZ, 10'000UZ}) {
            const Profile                       profile = leoProfile(nKnots);
            const gr::timing::FrequencySchedule schedule(profile.times, profile.offsets);

            const auto name                                                    = std::format("phaseIncrementsFor, {:5} knots, N={}", nKnots, kSamples);
            ::benchmark::benchmark<kRepeats>(std::string_view(name), kSamples) = [&] { schedule.phaseIncrementsFor(clock, 1'000'000ULL, increments); };
        }
        ::benchmark::results::add_separator();
    };

    // The same span read one sample per call. Each call rebuilds the cursor by a search. The walk avoids this cost.
    // This row is the closest this kernel comes to a per-sample search.
    "phaseIncrementsFor: one sample per call"_test = [] {
        const gr::timing::SampleClock clock(0ULL, 0LL, 48'000ULL, 1ULL);
        std::vector<double>           increments(kSamples);

        for (const std::size_t nKnots : {100UZ, 1'000UZ, 10'000UZ}) {
            const Profile                       profile = leoProfile(nKnots);
            const gr::timing::FrequencySchedule schedule(profile.times, profile.offsets);

            const auto name                                                = std::format("one call per sample,  {:5} knots, N={}", nKnots, kSamples);
            ::benchmark::benchmark<10UZ>(std::string_view(name), kSamples) = [&] {
                for (std::size_t k = 0UZ; k < kSamples; ++k) {
                    schedule.phaseIncrementsFor(clock, 1'000'000ULL + k, std::span<double>(increments).subspan(k, 1UZ));
                }
            };
        }
        ::benchmark::results::add_separator();
    };
}

inline const boost::ut::suite<"frequency schedule benchmarks"> _frequency_schedule_bm = [] { benchPhaseIncrements(); };

int main() { /* not needed by the UT framework */ }
