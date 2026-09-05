#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/algorithm/digital/Ppm.hpp>

namespace {

using gr::digital::PpmFrame;
using gr::digital::PpmScanner;

constexpr std::size_t kSamplesPerCall = 65536UZ;
constexpr std::size_t kRepeats        = 9UZ;

/// The preamble in half-microsecond slots: pulses at 0.0, 1.0, 3.5 and 4.5 microseconds.
constexpr std::array<int, 16> kPreamble{1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0};

/// The anchor frame, a published extended squitter whose parity is the bare CRC.
constexpr std::array<std::uint8_t, 14> kAnchor{0x8DU, 0x48U, 0x40U, 0xD6U, 0x20U, 0x2CU, 0xC3U, 0x71U, 0xC3U, 0x2CU, 0xE0U, 0x57U, 0x60U, 0x98U};

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
    [[nodiscard]] double uniform() noexcept { return static_cast<double>((next() >> 11U) + 1ULL) * 0x1p-53; }
};

/// Rayleigh magnitudes, the envelope a complex Gaussian channel presents with no signal in it.
[[nodiscard]] std::vector<float> noiseStream(std::size_t count) {
    Rng                rng;
    std::vector<float> samples(count);
    for (float& value : samples) {
        value = static_cast<float>(std::sqrt(-std::log(rng.uniform())));
    }
    return samples;
}

/// The anchor repeated with 40 microseconds of silence between bursts, at @p slot samples a slot.
[[nodiscard]] std::vector<float> anchorStream(std::size_t count, std::size_t slot) {
    std::vector<float> samples;
    samples.reserve(count + (16UZ + 224UZ) * slot);
    const auto emit = [&samples, slot](float value) { samples.insert(samples.end(), slot, value); };
    while (samples.size() < count) {
        for (const int pulse : kPreamble) {
            emit(pulse != 0 ? 1.F : 0.F);
        }
        for (std::size_t bit = 0UZ; bit < 112UZ; ++bit) {
            const bool one = ((kAnchor[bit / 8UZ] >> (7U - bit % 8UZ)) & 1U) != 0U;
            emit(one ? 1.F : 0.F);
            emit(one ? 0.F : 1.F);
        }
        samples.insert(samples.end(), 80UZ * slot, 0.F); // 40 us of silence, the gap between two replies
    }
    samples.resize(count);
    return samples;
}

[[nodiscard]] PpmScanner scannerAt(std::size_t slot) {
    PpmScanner scanner;
    scanner.prepare(gr::digital::modeS(), slot);
    return scanner;
}

} // namespace

int main() {
    // one arm per (input, samples per slot); the scanner decides every position whose window lies in the call
    std::vector<std::vector<float>> streams;
    std::vector<PpmScanner>         scanners;
    std::vector<std::string>        labels;
    for (const std::size_t slot : {1UZ, 4UZ}) {
        streams.push_back(noiseStream(kSamplesPerCall));
        scanners.push_back(scannerAt(slot));
        labels.push_back(std::format("Rayleigh noise, S = {}", slot));

        streams.push_back(anchorStream(kSamplesPerCall, slot));
        scanners.push_back(scannerAt(slot));
        labels.push_back(std::format("anchor repeated with a 40 us gap, S = {}", slot));
    }

    std::vector<gr::blocks::testing::bench::Arm> arms;
    std::vector<float>                           copyOut(kSamplesPerCall);
    const std::vector<float>&                    baseline = streams.front();
    arms.push_back({"span copy, the floor", [&] {
                        std::ranges::copy(baseline, copyOut.begin());
                        return static_cast<double>(copyOut[kSamplesPerCall / 2UZ]);
                    }});

    for (std::size_t which = 0UZ; which < scanners.size(); ++which) {
        arms.push_back({labels[which], [&, which] {
                            const std::size_t done = scanners[which].consume(std::span<const float>(streams[which]), [](const PpmFrame&) {});
                            return static_cast<double>(done);
                        }});
    }

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kSamplesPerCall, kRepeats);
}
