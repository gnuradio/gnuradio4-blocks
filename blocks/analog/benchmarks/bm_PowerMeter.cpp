#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/analog/PowerMeter.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::analog::PowerMeter;
using gr::blocks::testing::bench::Arm;

using CF = std::complex<float>;

constexpr std::size_t kSamples = 1UZ << 22;
constexpr std::size_t kChunk   = 4096UZ;
constexpr std::size_t kRepeats = 7UZ;

void init(PowerMeter<CF>& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
}

} // namespace

int main() {
    std::vector<CF> x(kSamples);
    for (std::size_t i = 0UZ; i < x.size(); ++i) {
        const double phase = 2.0 * std::numbers::pi * 0.031 * static_cast<double>(i);
        x[i]               = CF(static_cast<float>(0.7 * std::cos(phase)), static_cast<float>(0.7 * std::sin(phase)));
    }

    PowerMeter<CF> narrow({{"sample_rate", 96000.f}}); // 600-sample segments, a 9600-sample window
    PowerMeter<CF> wide({{"sample_rate", 25e6f}});     // 156250-sample segments, a 2 500 000-sample window
    PowerMeter<CF> polled({{"sample_rate", 96000.f}});
    PowerMeter<CF> wired({{"sample_rate", 96000.f}});
    init(narrow);
    init(wide);
    init(polled);
    init(wired);

    // `connected` separates the sample path of a polling graph from the path that also builds a record per window.
    // The sweep advances by what the meter consumed, as a scheduler does.
    std::vector<gr::DataSet<float>> records(4UZ);
    const auto                      sweep = [&x, &records](PowerMeter<CF>& meter, bool poll, bool connected) {
        namespace test    = gr::blocks::testing::span;
        double      read  = 0.0;
        std::size_t calls = 0UZ;
        for (std::size_t base = 0UZ; base < kSamples;) {
            const std::size_t                    count = std::min(kChunk, kSamples - base);
            test::InputSpan<CF>                  inSpan{std::span<const CF>(x).subspan(base, count)};
            test::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(records)};
            outSpan.isConnected = connected;
            std::ignore         = meter.processBulk(inSpan, outSpan);
            if (poll && calls % 24UZ == 0UZ) { // 10 Hz against 4096-sample calls at 96 kHz
                read += static_cast<double>(meter.level());
            }
            ++calls;
            if (inSpan.consumed == 0UZ) {
                break;
            }
            base += inSpan.consumed;
        }
        return poll ? read : meter.linear_power();
    };

    std::vector<Arm> arms;
    arms.emplace_back("|x|^2 accumulation, the floor", kSamples, [&x] {
        double total = 0.0;
        for (const CF& sample : x) {
            total += static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
        }
        return total;
    });
    arms.emplace_back("PowerMeter, 96 kHz window", kSamples, [&sweep, &narrow] { return sweep(narrow, false, false); });
    arms.emplace_back("PowerMeter, 25 MS/s window", kSamples, [&sweep, &wide] { return sweep(wide, false, false); });
    arms.emplace_back("PowerMeter, 96 kHz, level() at 10 Hz", kSamples, [&sweep, &polled] { return sweep(polled, true, false); });
    arms.emplace_back("PowerMeter, 96 kHz, record port wired", kSamples, [&sweep, &wired] { return sweep(wired, false, true); });

    gr::blocks::testing::bench::report(std::span<Arm>(arms), kRepeats);
}
