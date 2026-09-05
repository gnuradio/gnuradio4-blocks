#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <numbers>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/algorithm/digital/Constellation.hpp>
#include <gnuradio-4.0/digital/ConstellationBlocks.hpp>

namespace {

using gr::blocks::digital::ConstellationDecoder;
using gr::blocks::digital::ConstellationEncoder;
using gr::blocks::digital::ConstellationSoftDecoder;
using gr::blocks::digital::SymbolMap;
using CF = std::complex<float>;

constexpr std::size_t kSymbolsPerCall = 65536UZ;
constexpr std::size_t kRepeats        = 9UZ;

struct Row {
    const char* name;
    gr::Size_t  arity;
    std::size_t bits;
};

constexpr std::array<Row, 5UZ> kRows{Row{"bpsk", 2U, 1UZ}, Row{"qpsk", 4U, 2UZ}, Row{"psk8", 8U, 3UZ}, Row{"qam", 16U, 4UZ}, Row{"qam", 64U, 6UZ}};

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

[[nodiscard]] gr::property_map settingsOf(const Row& row) { return {{"constellation", std::string(row.name)}, {"arity", row.arity}}; }

[[nodiscard]] gr::digital::Constellation<float> kernelOf(const Row& row) {
    const std::string_view name(row.name);
    if (name == "qam") {
        return gr::digital::Constellation<float>::qam(row.arity);
    }
    if (name == "bpsk") {
        return gr::digital::Constellation<float>::bpsk();
    }
    if (name == "qpsk") {
        return gr::digital::Constellation<float>::qpsk();
    }
    return gr::digital::Constellation<float>::psk8();
}

[[nodiscard]] std::vector<CF> cloud() {
    std::vector<CF>  samples(kSymbolsPerCall);
    std::uint64_t    state = 0x9e3779b97f4a7c15ULL;
    constexpr double kStep = 2.0 / static_cast<double>(1ULL << 32U);
    for (CF& sample : samples) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        const double re = kStep * static_cast<double>(state >> 32U) - 1.0;
        const double im = kStep * static_cast<double>(state & 0xFFFFFFFFULL) - 1.0;
        sample          = CF(static_cast<float>(1.6 * re), static_cast<float>(1.6 * im));
    }
    return samples;
}

[[nodiscard]] std::vector<std::uint8_t> symbols() {
    std::vector<std::uint8_t> data(kSymbolsPerCall);
    std::uint64_t             state = 0x243f6a8885a308d3ULL;
    for (std::uint8_t& symbol : data) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        symbol = static_cast<std::uint8_t>(state & 0xFFU);
    }
    return data;
}

} // namespace

int main() {
    const std::vector<CF>           input = cloud();
    const std::vector<std::uint8_t> data  = symbols();

    std::vector<std::uint8_t> byteOut(kSymbolsPerCall);
    std::vector<CF>           complexOut(kSymbolsPerCall);
    std::vector<float>        softOut(kSymbolsPerCall * 6UZ);

    std::vector<ConstellationDecoder<float>>       hard;
    std::vector<ConstellationSoftDecoder<float>>   maxLog;
    std::vector<ConstellationSoftDecoder<float>>   exact;
    std::vector<ConstellationEncoder<float>>       encoders;
    std::vector<gr::digital::Constellation<float>> kernels;
    for (const Row& row : kRows) {
        hard.push_back(make<ConstellationDecoder<float>>(settingsOf(row)));
        maxLog.push_back(make<ConstellationSoftDecoder<float>>(settingsOf(row)));
        gr::property_map exactSettings  = settingsOf(row);
        exactSettings["soft_algorithm"] = std::string("exact");
        exact.push_back(make<ConstellationSoftDecoder<float>>(exactSettings));
        encoders.push_back(make<ConstellationEncoder<float>>(settingsOf(row)));
        kernels.push_back(kernelOf(row));
    }
    SymbolMap mapper = make<SymbolMap>({{"map", std::vector<std::uint8_t>(256UZ, 7U)}});

    std::vector<gr::blocks::testing::bench::Arm> arms;
    for (std::size_t which = 0UZ; which < kRows.size(); ++which) {
        arms.push_back({std::format("ConstellationDecoder, {}-{}", kRows[which].name, kRows[which].arity), [&, which] {
                            std::ignore = hard[which].processBulk(std::span<const CF>(input), std::span<std::uint8_t>(byteOut));
                            return static_cast<double>(byteOut[kSymbolsPerCall / 2UZ]);
                        }});
    }
    for (std::size_t which = 0UZ; which < kRows.size(); ++which) {
        const std::size_t bits = kRows[which].bits;
        arms.push_back({std::format("ConstellationSoftDecoder max_log, {}-{}", kRows[which].name, kRows[which].arity), [&, which, bits] {
                            std::ignore = maxLog[which].processBulk(std::span<const CF>(input), std::span<float>(softOut.data(), kSymbolsPerCall * bits));
                            return static_cast<double>(softOut[kSymbolsPerCall / 2UZ]);
                        }});
    }
    for (std::size_t which = 0UZ; which < kRows.size(); ++which) {
        const std::size_t bits = kRows[which].bits;
        arms.push_back({std::format("  the same, full M-point form, {}-{}", kRows[which].name, kRows[which].arity), [&, which, bits] {
                            for (std::size_t i = 0UZ; i < kSymbolsPerCall; ++i) {
                                kernels[which].softDecisionsExhaustive(input[i], 1.0f, std::span<float>(softOut.data() + i * bits, bits));
                            }
                            return static_cast<double>(softOut[kSymbolsPerCall / 2UZ]);
                        }});
    }
    for (std::size_t which = 0UZ; which < kRows.size(); ++which) {
        const std::size_t bits = kRows[which].bits;
        arms.push_back({std::format("ConstellationSoftDecoder exact, {}-{}", kRows[which].name, kRows[which].arity), [&, which, bits] {
                            std::ignore = exact[which].processBulk(std::span<const CF>(input), std::span<float>(softOut.data(), kSymbolsPerCall * bits));
                            return static_cast<double>(softOut[kSymbolsPerCall / 2UZ]);
                        }});
    }
    arms.push_back({"ConstellationEncoder, qam-64", [&] {
                        std::ignore = encoders[4].processBulk(std::span<const std::uint8_t>(data), std::span<CF>(complexOut));
                        return static_cast<double>(complexOut[kSymbolsPerCall / 2UZ].real());
                    }});
    arms.push_back({"SymbolMap", [&] {
                        std::ignore = mapper.processBulk(std::span<const std::uint8_t>(data), std::span<std::uint8_t>(byteOut));
                        return static_cast<double>(byteOut[kSymbolsPerCall / 2UZ]);
                    }});

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kSymbolsPerCall, kRepeats);
}
