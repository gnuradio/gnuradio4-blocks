#include <array>
#include <complex>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/channel/Multipath.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

namespace {

using gr::blocks::channel::FadingChannel;
using CF = std::complex<float>;

constexpr std::size_t kSamplesPerCall = 65536UZ;
constexpr std::size_t kRepeats        = 9UZ;

/// The rows the cost actually varies over: the sinusoid count is the inner loop, and the tap count multiplies it.
struct Row {
    const char* name;
    gr::Size_t  sinusoids;
    std::size_t taps;
};

constexpr std::array<Row, 5UZ> kRows{Row{"1 tap, 8 sinusoids", 8U, 1UZ}, Row{"1 tap, 16 sinusoids", 16U, 1UZ}, Row{"1 tap, 32 sinusoids", 32U, 1UZ}, Row{"4 taps, 16 sinusoids", 16U, 4UZ}, Row{"9 taps, 16 sinusoids", 16U, 9UZ}};

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
    return block;
}

[[nodiscard]] gr::property_map settingsFor(const Row& row) {
    std::vector<gr::Size_t> delays(row.taps);
    std::vector<double>     powers(row.taps);
    for (std::size_t m = 0UZ; m < row.taps; ++m) {
        delays[m] = static_cast<gr::Size_t>(3UZ * m); // spread over the line rather than packed at its head
        powers[m] = -3.0 * static_cast<double>(m);
    }
    return {{"sample_rate", 1.0e6f}, {"max_doppler", 100.0}, {"n_sinusoids", row.sinusoids}, {"delays", delays}, {"powers_db", powers}, {"seed", std::uint64_t{12345ULL}}};
}

[[nodiscard]] std::vector<CF> noise() {
    std::vector<CF> data(kSamplesPerCall);
    std::uint64_t   state = 0x243f6a8885a308d3ULL;
    for (CF& sample : data) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        const float re = 2.f * static_cast<float>(state % 1024ULL) / 1024.f - 1.f;
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        const float im = 2.f * static_cast<float>(state % 1024ULL) / 1024.f - 1.f;
        sample         = CF(re, im);
    }
    return data;
}

} // namespace

int main() {
    const std::vector<CF> input = noise();
    std::vector<CF>       output(kSamplesPerCall);

    std::vector<FadingChannel<CF>> channels;
    for (const Row& row : kRows) {
        channels.push_back(make<FadingChannel<CF>>(settingsFor(row)));
    }

    std::vector<gr::blocks::testing::bench::Arm> arms;
    for (std::size_t which = 0UZ; which < kRows.size(); ++which) {
        arms.push_back({std::format("FadingChannel, {}", kRows[which].name), [&, which] {
                            std::ignore = channels[which].processBulk(std::span<const CF>(input), std::span<CF>(output));
                            return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
                        }});
    }

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kSamplesPerCall, kRepeats);
}
