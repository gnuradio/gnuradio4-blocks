#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

#include <complex>
#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <vector>

#include <gnuradio-4.0/filter/ArbitraryRateResampler.hpp>

namespace {

using gr::blocks::filter::ArbitraryRateResampler;
using CF       = std::complex<float>;
namespace test = gr::blocks::testing::span;

/// Four representative rates, at both useful interpolation orders. `B` scales as `1/r` below
/// unity, so the multiply-accumulates per input sample are flat there and rise with `r` above it.
struct Shape {
    const char* label;
    double      rate;
};

constexpr Shape kShapes[] = {{"0.2", 0.2}, {"19/24", 19.0 / 24.0}, {"48/44.1", 48.0 / 44.1}, {"1.7", 1.7}};

constexpr std::size_t kBank    = 32UZ;
constexpr std::size_t kPass    = 1UZ << 18;
constexpr std::size_t kRepeats = 5UZ;

struct Noise {
    std::uint64_t state = 0x2545F4914F6CDD1DULL;

    [[nodiscard]] float next() noexcept {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return static_cast<float>(static_cast<double>(state >> 11) / 9007199254740992.0 * 2.0 - 1.0);
    }
};

[[nodiscard]] std::vector<CF> noise(std::size_t n) {
    Noise           source{};
    std::vector<CF> out(n);
    for (CF& v : out) {
        v = CF{source.next(), source.next()};
    }
    return out;
}

struct Bed {
    ArbitraryRateResampler<CF> block;
    std::vector<CF>            input;
    std::vector<CF>            output;

    Bed(const Shape& shape, gr::Size_t order) : block({{"rate", shape.rate}, {"bank_size", static_cast<gr::Size_t>(kBank)}, {"interpolation_order", order}}), input(noise(kPass)) {
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
        output.resize(block.outputsFor(kPass) + 8UZ);
    }

    [[nodiscard]] double run() {
        test::InputSpan<CF>  inSpan(std::span<const CF>(input), 0UZ, {}, false);
        test::OutputSpan<CF> outSpan(std::span<CF>(output), 0UZ, nullptr, true, false);
        std::ignore = block.processBulk(inSpan, outSpan);
        return static_cast<double>(std::abs(output[outSpan.count / 2UZ]));
    }
};

} // namespace

int main() {
    std::vector<std::unique_ptr<Bed>>            beds;
    std::vector<gr::blocks::testing::bench::Arm> arms;

    for (const gr::Size_t order : {1U, 3U}) {
        for (const Shape& shape : kShapes) {
            beds.push_back(std::make_unique<Bed>(shape, order));
            const double perOutput = static_cast<double>(order + 1U) * static_cast<double>(beds.back()->block.tapsPerArm());
            arms.emplace_back(std::format("r={} q={} B={}", shape.label, order, beds.back()->block.tapsPerArm()), kPass, perOutput * beds.back()->block.realizedRate(), [raw = beds.back().get()] { return raw->run(); });
        }
    }

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kRepeats);
}
