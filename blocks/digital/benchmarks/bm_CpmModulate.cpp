#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <array>
#include <complex>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/algorithm/digital/CpmPulse.hpp>
#include <gnuradio-4.0/digital/CpmModulate.hpp>

namespace {

using gr::blocks::digital::CpmModulate;
using gr::digital::CpmPulse;
using gr::digital::CpmPulseShape;
using CF = std::complex<float>;

constexpr std::size_t kSamplesPerSymbol = 8UZ;
constexpr std::size_t kSymbolsPerCall   = 8192UZ;
constexpr std::size_t kSamplesPerCall   = kSymbolsPerCall * kSamplesPerSymbol;
constexpr std::size_t kRepeats          = 9UZ;

struct Row {
    const char*   name;
    CpmPulseShape shape;
    gr::Size_t    length;
};

constexpr std::array<Row, 4UZ> kRows{Row{"rect, L=1", CpmPulseShape::Rect, 1U}, Row{"rect, L=3", CpmPulseShape::Rect, 3U}, Row{"raised_cosine, L=3", CpmPulseShape::RaisedCosine, 3U}, Row{"gaussian, L=3", CpmPulseShape::Gaussian, 3U}};

[[nodiscard]] const char* shapeName(CpmPulseShape shape) {
    switch (shape) {
    case CpmPulseShape::Rect: return "rect";
    case CpmPulseShape::RaisedCosine: return "raised_cosine";
    case CpmPulseShape::Gaussian: return "gaussian";
    }
    return "rect";
}

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

/// A PAM-4 symbol stream on the odd grid, the widest grid the sweeps use.
[[nodiscard]] std::vector<float> symbols() {
    std::vector<float> data(kSymbolsPerCall);
    std::uint64_t      state = 0x243f6a8885a308d3ULL;
    for (float& symbol : data) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        symbol = 2.f * static_cast<float>(state % 4ULL) - 3.f;
    }
    return data;
}

} // namespace

int main() {
    const std::vector<float> input = symbols();

    std::vector<CF>     complexOut(kSamplesPerCall);
    std::vector<double> incrementOut(kSamplesPerCall);

    std::vector<CpmModulate<float>> modulators;
    std::vector<CpmPulse<float>>    kernels;
    for (const Row& row : kRows) {
        modulators.push_back(make<CpmModulate<float>>({{"samples_per_symbol", static_cast<gr::Size_t>(kSamplesPerSymbol)}, {"pulse", std::string(shapeName(row.shape))}, {"pulse_length", row.length}}));
        CpmPulse<float> kernel;
        kernel.configure(row.shape, static_cast<std::size_t>(row.length), kSamplesPerSymbol, 0.5, 0.3);
        kernels.push_back(std::move(kernel));
    }

    std::vector<gr::blocks::testing::bench::Arm> arms;
    for (std::size_t which = 0UZ; which < kRows.size(); ++which) {
        arms.push_back({std::format("CpmPulse increments, {}", kRows[which].name), [&, which] {
                            kernels[which].incrementsFor(std::span<const float>(input), std::span<double>(incrementOut));
                            return incrementOut[kSamplesPerCall / 2UZ];
                        }});
    }
    for (std::size_t which = 0UZ; which < kRows.size(); ++which) {
        arms.push_back({std::format("CpmModulate, {}", kRows[which].name), [&, which] {
                            std::ignore = modulators[which].processBulk(std::span<const float>(input), std::span<CF>(complexOut));
                            return static_cast<double>(complexOut[kSamplesPerCall / 2UZ].real());
                        }});
    }

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kSamplesPerCall, kRepeats);
}
