#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/basic/IqCorrection.hpp>

namespace {

using gr::blocks::basic::DcOffsetCorrect;
using gr::blocks::basic::IqSwap;
using gr::blocks::testing::bench::Arm;

using CF = std::complex<float>;

constexpr std::size_t kSamples = 1UZ << 22;
constexpr std::size_t kRepeats = 9UZ;

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

} // namespace

// The per-sample cost of both blocks, disabled and enabled, beside a span copy. Run it pinned to one core.
int main() {
    std::vector<CF> x(kSamples);
    for (std::size_t i = 0UZ; i < x.size(); ++i) { // an offset tone, so the tracker has something to track
        const double phase = 2.0 * std::numbers::pi * 0.031 * static_cast<double>(i);
        x[i]               = CF(static_cast<float>(0.7 * std::cos(phase) + 0.05), static_cast<float>(0.7 * std::sin(phase) - 0.02));
    }
    std::vector<CF> y(kSamples);
    std::vector<CF> silence(kSamples, CF{});

    IqSwap<CF>          swapOff = make<IqSwap<CF>>({});
    IqSwap<CF>          swapOn  = make<IqSwap<CF>>({{"enabled", true}});
    DcOffsetCorrect<CF> dcOff   = make<DcOffsetCorrect<CF>>({{"sample_rate", 25e6f}, {"tau", 1.0}});
    DcOffsetCorrect<CF> dcOn    = make<DcOffsetCorrect<CF>>({{"enabled", true}, {"sample_rate", 25e6f}, {"tau", 1.0}});
    DcOffsetCorrect<CF> device  = make<DcOffsetCorrect<CF>>({{"enabled", true}, {"sample_rate", 10e6f}, {"tau", 1.0}});

    const auto sweep = [&x, &y](auto& block) {
        for (std::size_t i = 0UZ; i < kSamples; ++i) {
            y[i] = block.processOne(x[i]);
        }
        return static_cast<double>(y[kSamples / 2UZ].real());
    };

    // The cost and the saving of the subnormal flush, at a device rate. On a live stream the flush and the hold on a
    // non-finite sample add compares and conditional stores. `asItStood` runs the recursion without either. On silence
    // that recursion decays the estimate below the smallest normal after 7 054 006 510 zeros, 705 s at 10 MS/s. It
    // then stays on the smallest subnormal, where every operation costs a microcode assist. The flushed block stays on
    // an exact zero. The last two arms start from those two states.
    const double alpha     = device._alpha;
    const double rest      = device._oneMinusAlpha;
    const auto   asItStood = [&y, alpha, rest](std::span<const CF> input, double seed) {
        double re = seed;
        double im = seed;
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            const double real = static_cast<double>(input[i].real());
            const double imag = static_cast<double>(input[i].imag());
            re                = alpha * real + rest * re;
            im                = alpha * imag + rest * im;
            y[i]              = CF(static_cast<float>(real - re), static_cast<float>(imag - im));
        }
        return static_cast<double>(y[kSamples / 2UZ].real());
    };
    const auto quiet = [&silence, &y](auto& block) {
        block.reset();
        for (std::size_t i = 0UZ; i < kSamples; ++i) {
            y[i] = block.processOne(silence[i]);
        }
        return static_cast<double>(y[kSamples / 2UZ].real());
    };

    std::vector<Arm> arms;
    arms.emplace_back("plain span copy, the 1:1 floor", kSamples, [&x, &y] {
        std::copy_n(x.begin(), kSamples, y.begin());
        return static_cast<double>(y[kSamples / 2UZ].real());
    });
    arms.emplace_back("IqSwap disabled", kSamples, [&sweep, &swapOff] { return sweep(swapOff); });
    arms.emplace_back("IqSwap enabled", kSamples, [&sweep, &swapOn] { return sweep(swapOn); });
    arms.emplace_back("DcOffsetCorrect disabled", kSamples, [&sweep, &dcOff] { return sweep(dcOff); });
    arms.emplace_back("DcOffsetCorrect enabled", kSamples, [&sweep, &dcOn] { return sweep(dcOn); });
    arms.emplace_back("at 10 MS/s, signal, the recursion as it stood", kSamples, [&asItStood, &x] { return asItStood(std::span<const CF>(x), 0.0); });
    arms.emplace_back("at 10 MS/s, signal, with the flush", kSamples, [&sweep, &device] { return sweep(device); });
    arms.emplace_back("at 10 MS/s, silence, the recursion as it stood", kSamples, [&asItStood, &silence] { return asItStood(std::span<const CF>(silence), 1e-310); });
    arms.emplace_back("at 10 MS/s, silence, with the flush", kSamples, [&quiet, &device] { return quiet(device); });

    gr::blocks::testing::bench::report(std::span<Arm>(arms), kRepeats);
}
