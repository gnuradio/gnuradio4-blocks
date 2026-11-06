#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <limits>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/basic/IqCorrection.hpp>

namespace {

using gr::blocks::basic::FrontEndCorrection;
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

/// The DC removal that `DcOffsetCorrect` has on main, as a block runs it in a chain: a `processBulk` that runs the
/// recursion over the span with a `double` average of each component in locals, holds it on a non-finite sample and
/// flushes a subnormal component once at the end of the call. It stands for main's block in the chain of two that
/// `FrontEndCorrection` replaces.
struct MainDcOffsetCorrect {
    double alpha = 0.0;
    double rest  = 1.0;
    double real  = 0.0;
    double imag  = 0.0;

    explicit MainDcOffsetCorrect(double rate, double tau) : alpha(1.0 / (1.0 + tau * rate)), rest(1.0 - alpha) {}

    void processBulk(std::span<const CF> input, std::span<CF> output) noexcept {
        double estimate     = real;
        double estimateImag = imag;
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            const double re     = static_cast<double>(input[i].real());
            const double im     = static_cast<double>(input[i].imag());
            const double nextRe = alpha * re + rest * estimate;
            const double nextIm = alpha * im + rest * estimateImag;
            if (std::isfinite(nextRe) && std::isfinite(nextIm)) {
                estimate     = nextRe;
                estimateImag = nextIm;
            }
            output[i] = CF(static_cast<float>(re - estimate), static_cast<float>(im - estimateImag));
        }
        real = flushed(estimate);
        imag = flushed(estimateImag);
    }

    [[nodiscard]] static double flushed(double value) noexcept { return std::abs(value) < std::numeric_limits<double>::min() ? 0.0 : value; }
};

} // namespace

// The per-sample cost of IqSwap off and on, of main's IqSwap and DC removal in series, and of FrontEndCorrection in each
// of its eight states, beside a span copy. Run it pinned to one core.
int main() {
    std::vector<CF> x(kSamples);
    for (std::size_t i = 0UZ; i < x.size(); ++i) { // an offset tone, so the tracker has something to track
        const double phase = 2.0 * std::numbers::pi * 0.031 * static_cast<double>(i);
        x[i]               = CF(static_cast<float>(0.7 * std::cos(phase) + 0.05), static_cast<float>(0.7 * std::sin(phase) - 0.02));
    }
    std::vector<CF> y(kSamples);
    std::vector<CF> silence(kSamples, CF{});

    IqSwap<CF> swapOff = make<IqSwap<CF>>({});
    IqSwap<CF> swapOn  = make<IqSwap<CF>>({{"enabled", true}});
    // The states by name, in bit order swap, IQ, DC. Each starts from the same settings.
    const auto                              makeState = [](bool exchange, bool iqOn, bool dcOn) { return make<FrontEndCorrection<CF>>({{"swap_iq", exchange}, {"enable_iq", iqOn}, {"enable_dc", dcOn}, {"sample_rate", 25e6f}, {"tau", 1.0}}); };
    std::array<FrontEndCorrection<CF>, 8UZ> states{makeState(false, false, false), makeState(false, false, true), makeState(false, true, false), makeState(false, true, true), makeState(true, false, false), makeState(true, false, true), makeState(true, true, false), makeState(true, true, true)};
    FrontEndCorrection<CF>                  device    = make<FrontEndCorrection<CF>>({{"enable_dc", true}, {"sample_rate", 10e6f}, {"tau", 1.0}});
    FrontEndCorrection<CF>                  cycling   = make<FrontEndCorrection<CF>>({{"sample_rate", 25e6f}, {"tau", 1.0}});
    IqSwap<CF>                              chainSwap = make<IqSwap<CF>>({{"enabled", true}});
    MainDcOffsetCorrect                     chainDc(25e6, 1.0);

    const auto sweep = [&x, &y]<typename TBlock>(TBlock& block) {
        if constexpr (gr::HasProcessBulkFunction<TBlock>) {
            std::ignore = block.processBulk(std::span<const CF>(x), std::span<CF>(y));
        } else {
            for (std::size_t i = 0UZ; i < kSamples; ++i) {
                y[i] = block.processOne(x[i]);
            }
        }
        return static_cast<double>(y[kSamples / 2UZ].real());
    };
    const auto bulk = [&x, &y](auto& block) {
        std::ignore = block.processBulk(std::span<const CF>(x), std::span<CF>(y));
        return static_cast<double>(y[kSamples / 2UZ].real());
    };

    // The subnormal flush at a device rate. `asItStood` runs the recursion with neither the flush nor the hold on a
    // non-finite sample. The block arms run `processBulk`, which has both. On silence that recursion decays
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
    // Steps through the eight states, each switch changed through the settings, every 2^16 samples.
    const auto cycle = [&x, &y, &cycling] {
        constexpr std::size_t kStretch = 1UZ << 16;
        for (std::size_t start = 0UZ; start < kSamples; start += kStretch) {
            const std::size_t state = start / kStretch % 8UZ;
            std::ignore             = cycling.settings().setStaged({{"enable_dc", (state & 1UZ) != 0UZ}, {"enable_iq", (state & 2UZ) != 0UZ}, {"swap_iq", (state & 4UZ) != 0UZ}});
            std::ignore             = cycling.settings().applyStagedParameters();
            std::ignore             = cycling.processBulk(std::span<const CF>(x).subspan(start, kStretch), std::span<CF>(y).subspan(start, kStretch));
        }
        return static_cast<double>(y[kSamples / 2UZ].real());
    };
    // Main's two blocks in series, each run over the span in turn, as two blocks of a chain see the stream.
    const auto chain = [&x, &y, &chainSwap, &chainDc] {
        for (std::size_t i = 0UZ; i < kSamples; ++i) {
            y[i] = chainSwap.processOne(x[i]);
        }
        chainDc.processBulk(std::span<const CF>(y), std::span<CF>(y));
        return static_cast<double>(y[kSamples / 2UZ].real());
    };
    // Main's DcOffsetCorrect alone, run over the span as one block.
    MainDcOffsetCorrect aloneDc(25e6, 1.0);
    const auto          dcAlone = [&x, &y, &aloneDc] {
        aloneDc.processBulk(std::span<const CF>(x), std::span<CF>(y));
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
    arms.emplace_back("main: DcOffsetCorrect enabled alone", kSamples, [&dcAlone] { return dcAlone(); });
    arms.emplace_back("main: IqSwap enabled, then DcOffsetCorrect enabled, in series", kSamples, [&chain] { return chain(); });
    arms.emplace_back("FrontEndCorrection all off", kSamples, [&bulk, &states] { return bulk(states[0UZ]); });
    arms.emplace_back("FrontEndCorrection DC only", kSamples, [&bulk, &states] { return bulk(states[1UZ]); });
    arms.emplace_back("FrontEndCorrection IQ only", kSamples, [&bulk, &states] { return bulk(states[2UZ]); });
    arms.emplace_back("FrontEndCorrection IQ and DC", kSamples, [&bulk, &states] { return bulk(states[3UZ]); });
    arms.emplace_back("FrontEndCorrection swap only", kSamples, [&bulk, &states] { return bulk(states[4UZ]); });
    arms.emplace_back("FrontEndCorrection swap and DC", kSamples, [&bulk, &states] { return bulk(states[5UZ]); });
    arms.emplace_back("FrontEndCorrection swap and IQ", kSamples, [&bulk, &states] { return bulk(states[6UZ]); });
    arms.emplace_back("FrontEndCorrection swap, IQ and DC", kSamples, [&bulk, &states] { return bulk(states[7UZ]); });
    arms.emplace_back("FrontEndCorrection cycling the eight states", kSamples, [&cycle] { return cycle(); });
    arms.emplace_back("at 10 MS/s, signal, the recursion as it stood", kSamples, [&asItStood, &x] { return asItStood(std::span<const CF>(x), 0.0); });
    arms.emplace_back("at 10 MS/s, signal, with the flush", kSamples, [&bulk, &device] { return bulk(device); });
    arms.emplace_back("at 10 MS/s, silence, the recursion as it stood", kSamples, [&asItStood, &silence] { return asItStood(std::span<const CF>(silence), 1e-310); });
    arms.emplace_back("at 10 MS/s, silence, with the flush", kSamples, [&quiet, &device] { return quiet(device); });

    gr::blocks::testing::bench::report(std::span<Arm>(arms), kRepeats);
}
