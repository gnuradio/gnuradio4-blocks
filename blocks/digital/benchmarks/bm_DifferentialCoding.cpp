#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <numbers>
#include <span>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/digital/DifferentialCoding.hpp>

namespace {

using gr::blocks::digital::DifferentialDecoder;
using gr::blocks::digital::DifferentialEncoder;
using gr::blocks::digital::DifferentialPhasor;
using CF = std::complex<float>;

constexpr std::size_t kSamplesPerCall = 65536UZ;
constexpr std::size_t kRepeats        = 9UZ;

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

[[nodiscard]] std::vector<std::uint8_t> symbols(std::uint32_t modulus) {
    std::vector<std::uint8_t> data(kSamplesPerCall);
    std::uint64_t             state = 0x9e3779b97f4a7c15ULL;
    for (std::uint8_t& symbol : data) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        symbol = static_cast<std::uint8_t>(state % modulus);
    }
    return data;
}

[[nodiscard]] std::vector<CF> tone() {
    std::vector<CF> signal(kSamplesPerCall);
    for (std::size_t i = 0UZ; i < signal.size(); ++i) {
        const double phase = 2.0 * std::numbers::pi * 0.037 * static_cast<double>(i);
        signal[i]          = CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
    }
    return signal;
}

/// @brief The decoder written with a carried state word instead of a shifted span, as a comparison arm.
[[nodiscard]] double serialDecode(std::span<const std::uint8_t> input, std::span<std::uint8_t> output, std::uint32_t modulus) {
    std::uint32_t previous = 0U;
    for (std::size_t i = 0UZ; i < input.size(); ++i) {
        const std::uint32_t symbol     = static_cast<std::uint32_t>(input[i]);
        const std::uint32_t difference = symbol + modulus - previous;
        output[i]                      = static_cast<std::uint8_t>(difference >= modulus ? difference - modulus : difference);
        previous                       = symbol;
    }
    return static_cast<double>(output[kSamplesPerCall / 2UZ]);
}

} // namespace

int main() {
    constexpr std::array<std::uint32_t, 4UZ> kModuli{2U, 4U, 256U, 100U};

    std::vector<std::vector<std::uint8_t>> inputs;
    for (const std::uint32_t modulus : kModuli) {
        inputs.push_back(symbols(modulus));
    }
    const std::vector<CF> signal = tone();

    std::vector<std::uint8_t> byteOut(kSamplesPerCall);
    std::vector<CF>           complexOut(kSamplesPerCall);

    std::vector<DifferentialEncoder<std::uint8_t>> encoders;
    std::vector<DifferentialDecoder<std::uint8_t>> decoders;
    for (const std::uint32_t modulus : kModuli) {
        encoders.push_back(make<DifferentialEncoder<std::uint8_t>>({{"modulus", modulus}}));
        decoders.push_back(make<DifferentialDecoder<std::uint8_t>>({{"modulus", modulus}}));
    }
    DifferentialPhasor<float> phasor = make<DifferentialPhasor<float>>();

    std::vector<gr::blocks::testing::bench::Arm> arms;
    for (std::size_t which = 0UZ; which < kModuli.size(); ++which) {
        arms.push_back({std::format("DifferentialEncoder<uint8_t>, M={}", kModuli[which]), [&, which] {
                            std::ignore = encoders[which].processBulk(std::span<const std::uint8_t>(inputs[which]), std::span<std::uint8_t>(byteOut));
                            return static_cast<double>(byteOut[kSamplesPerCall / 2UZ]);
                        }});
    }
    for (std::size_t which = 0UZ; which < kModuli.size(); ++which) {
        arms.push_back({std::format("DifferentialDecoder<uint8_t>, M={}", kModuli[which]), [&, which] {
                            std::ignore = decoders[which].processBulk(std::span<const std::uint8_t>(inputs[which]), std::span<std::uint8_t>(byteOut));
                            return static_cast<double>(byteOut[kSamplesPerCall / 2UZ]);
                        }});
    }
    for (const std::size_t which : {1UZ, 3UZ}) {
        arms.push_back({std::format("  the same decode, carried state, M={}", kModuli[which]), [&, which] { return serialDecode(std::span<const std::uint8_t>(inputs[which]), std::span<std::uint8_t>(byteOut), kModuli[which]); }});
    }
    arms.push_back({"DifferentialPhasor<float>", [&] {
                        std::ignore = phasor.processBulk(std::span<const CF>(signal), std::span<CF>(complexOut));
                        return static_cast<double>(complexOut[kSamplesPerCall / 2UZ].real());
                    }});

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kSamplesPerCall, kRepeats);
}
