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

using gr::blocks::basic::DcIqCorrection;
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

// The per-sample cost of IqSwap off and on and of DcIqCorrection in each of its four states, beside a span copy. Run it
// pinned to one core.
int main() {
    std::vector<CF> x(kSamples);
    for (std::size_t i = 0UZ; i < x.size(); ++i) { // an offset tone, so the tracker has something to track
        const double phase = 2.0 * std::numbers::pi * 0.031 * static_cast<double>(i);
        x[i]               = CF(static_cast<float>(0.7 * std::cos(phase) + 0.05), static_cast<float>(0.7 * std::sin(phase) - 0.02));
    }
    std::vector<CF> y(kSamples);
    std::vector<CF> silence(kSamples, CF{});

    IqSwap<CF>         swapOff = make<IqSwap<CF>>({});
    IqSwap<CF>         swapOn  = make<IqSwap<CF>>({{"enabled", true}});
    DcIqCorrection<CF> dcOff   = make<DcIqCorrection<CF>>({{"sample_rate", 25e6f}, {"tau", 1.0}});
    DcIqCorrection<CF> dcOn    = make<DcIqCorrection<CF>>({{"enable_dc", true}, {"sample_rate", 25e6f}, {"tau", 1.0}});
    DcIqCorrection<CF> iqOn    = make<DcIqCorrection<CF>>({{"enable_iq", true}, {"sample_rate", 25e6f}, {"tau", 1.0}});
    DcIqCorrection<CF> bothOn  = make<DcIqCorrection<CF>>({{"enable_dc", true}, {"enable_iq", true}, {"sample_rate", 25e6f}, {"tau", 1.0}});
    DcIqCorrection<CF> device  = make<DcIqCorrection<CF>>({{"enable_dc", true}, {"sample_rate", 10e6f}, {"tau", 1.0}});
    DcIqCorrection<CF> cycling = make<DcIqCorrection<CF>>({{"sample_rate", 25e6f}, {"tau", 1.0}});

    const auto sweep = [&x, &y](auto& block) {
        for (std::size_t i = 0UZ; i < kSamples; ++i) {
            y[i] = block.processOne(x[i]);
        }
        return static_cast<double>(y[kSamples / 2UZ].real());
    };
    const auto bulk = [&x, &y](auto& block) {
        std::ignore = block.processBulk(std::span<const CF>(x), std::span<CF>(y));
        return static_cast<double>(y[kSamples / 2UZ].real());
    };

    // The subnormal flush at a device rate. `asItStood` runs the per-sample recursion with neither the flush nor the
    // hold on a non-finite sample. The block arms run `processBulk`, which has both. On silence that recursion decays
    // the estimate below the smallest normal after 7 054 006 510 zeros, 705 s at 10 MS/s. It then stays on the smallest
    // subnormal, where every operation costs a microcode assist. The flushed block stays on an exact zero. The last two
    // arms start from those two states.
    const double alpha     = device.alpha.value;
    const double rest      = 1.0 - device.alpha.value;
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
    // Steps through the four states, each switch changed through the settings, every 2^16 samples.
    const auto cycle = [&x, &y, &cycling] {
        constexpr std::size_t kStretch = 1UZ << 16;
        for (std::size_t start = 0UZ; start < kSamples; start += kStretch) {
            const std::size_t state = start / kStretch % 4UZ;
            std::ignore             = cycling.settings().setStaged({{"enable_dc", (state & 1UZ) != 0UZ}, {"enable_iq", (state & 2UZ) != 0UZ}});
            std::ignore             = cycling.settings().applyStagedParameters();
            std::ignore             = cycling.processBulk(std::span<const CF>(x).subspan(start, kStretch), std::span<CF>(y).subspan(start, kStretch));
        }
        return static_cast<double>(y[kSamples / 2UZ].real());
    };
    const auto quiet = [&silence, &y](auto& block) {
        block.reset();
        std::ignore = block.processBulk(std::span<const CF>(silence), std::span<CF>(y));
        return static_cast<double>(y[kSamples / 2UZ].real());
    };

    std::vector<Arm> arms;
    arms.emplace_back("plain span copy, the 1:1 floor", kSamples, [&x, &y] {
        std::copy_n(x.begin(), kSamples, y.begin());
        return static_cast<double>(y[kSamples / 2UZ].real());
    });
    arms.emplace_back("IqSwap disabled", kSamples, [&sweep, &swapOff] { return sweep(swapOff); });
    arms.emplace_back("IqSwap enabled", kSamples, [&sweep, &swapOn] { return sweep(swapOn); });
    arms.emplace_back("DcIqCorrection both off", kSamples, [&bulk, &dcOff] { return bulk(dcOff); });
    arms.emplace_back("DcIqCorrection DC only", kSamples, [&bulk, &dcOn] { return bulk(dcOn); });
    arms.emplace_back("DcIqCorrection IQ only", kSamples, [&bulk, &iqOn] { return bulk(iqOn); });
    arms.emplace_back("DcIqCorrection DC and IQ", kSamples, [&bulk, &bothOn] { return bulk(bothOn); });
    arms.emplace_back("DcIqCorrection cycling the four states", kSamples, [&cycle] { return cycle(); });
    arms.emplace_back("at 10 MS/s, signal, the recursion as it stood", kSamples, [&asItStood, &x] { return asItStood(std::span<const CF>(x), 0.0); });
    arms.emplace_back("at 10 MS/s, signal, with the flush", kSamples, [&bulk, &device] { return bulk(device); });
    arms.emplace_back("at 10 MS/s, silence, the recursion as it stood", kSamples, [&asItStood, &silence] { return asItStood(std::span<const CF>(silence), 1e-310); });
    arms.emplace_back("at 10 MS/s, silence, with the flush", kSamples, [&quiet, &device] { return quiet(device); });

    gr::blocks::testing::bench::report(std::span<Arm>(arms), kRepeats);
}
