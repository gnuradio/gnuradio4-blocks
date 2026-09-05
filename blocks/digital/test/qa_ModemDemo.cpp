#include <boost/ut.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/digital/Constellation.hpp>
#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>

#include <gnuradio-4.0/algorithm/channel/NoisePower.hpp>
#include <gnuradio-4.0/channel/Awgn.hpp>
#include <gnuradio-4.0/channel/CarrierImpairments.hpp>
#include <gnuradio-4.0/digital/ConstellationBlocks.hpp>
#include <gnuradio-4.0/digital/SignalQuality.hpp>

#include <gnuradio-4.0/filter/DesignedFilter.hpp>
#include <gnuradio-4.0/filter/RationalResampler.hpp>
#include <gnuradio-4.0/sync/CostasLoop.hpp>
#include <gnuradio-4.0/sync/SymbolSync.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::channel::AwgnChannel;
using gr::blocks::channel::FrequencyOffset;
using gr::blocks::digital::EvmMeter;
using gr::blocks::filter::DesignedFilter;
using gr::blocks::filter::RationalResampler;
using gr::blocks::sync::CostasLoop;
using gr::blocks::sync::SymbolSync;
using gr::testing::ProcessFunction;
using gr::testing::TagSink;
using gr::testing::TagSource;

using CF = std::complex<float>;

constexpr std::size_t kSps     = 4UZ;
constexpr double      kRolloff = 0.35;
constexpr std::size_t kSymbols = 20000UZ;
constexpr std::size_t kSettle  = 2000UZ; ///< symbols the loops spend acquiring, dropped before any figure is taken

/// A seeded QPSK symbol stream at unit power, and the labels that produced it.
struct Payload {
    std::vector<CF>           symbols;
    std::vector<std::uint8_t> labels;
};

[[nodiscard]] Payload payload(std::size_t count, std::uint64_t seed) {
    const auto    map = gr::digital::Constellation<float>::qpsk();
    Payload       out{std::vector<CF>(count), std::vector<std::uint8_t>(count)};
    std::uint64_t state = seed;
    for (std::size_t i = 0UZ; i < count; ++i) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        const auto label = static_cast<std::uint8_t>(state & 3ULL);
        out.labels[i]    = label;
        out.symbols[i]   = map.point(label);
    }
    return out;
}

/**
 * @brief The modem: shaping, an impairment channel, matched filtering, timing and carrier recovery.
 *
 * @param esN0_db          noise the channel adds, as symbol energy over noise density
 * @param frequencyOffset  carrier offset in cycles per sample of the shaped stream
 * @param phaseOffset      constant carrier phase at the stream start
 */
[[nodiscard]] std::vector<CF> throughTheModem(const Payload& sent, double esN0_db, double frequencyOffset, double phaseOffset, std::uint64_t seed) {
    gr::Graph  graph;
    const auto values = gr::Tensor<CF>(sent.symbols.begin(), sent.symbols.end());

    const auto shaping = gr::filter::fir::design::rootRaisedCosine(11UZ * kSps + 1UZ, static_cast<double>(kSps), kRolloff, static_cast<double>(kSps));

    auto& source  = graph.emplaceBlock<TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(sent.symbols.size())}, {"values", values}, {"sample_rate", 1.f}});
    auto& shape   = graph.emplaceBlock<RationalResampler<CF>>({{"interpolation", static_cast<gr::Size_t>(kSps)}, {"decimation", gr::Size_t(1)}, {"taps", shaping}});
    auto& carrier = graph.emplaceBlock<FrequencyOffset<CF>>({{"sample_rate", 1.f}, {"frequency_offset", frequencyOffset}, {"phase_offset", phaseOffset}});
    auto& noise   = graph.emplaceBlock<AwgnChannel<CF>>({{"noise_power", static_cast<double>(gr::channel::noisePowerFor(static_cast<float>(esN0_db), 1.f, static_cast<float>(kSps)))}, {"seed", seed}});
    auto& matched = graph.emplaceBlock<DesignedFilter<CF, float>>({{"profile", std::string("root_raised_cosine")}, {"sample_rate", static_cast<float>(kSps)}, {"symbol_rate", 1.0}, {"alpha", kRolloff}, {"taps", static_cast<gr::Size_t>(11UZ * kSps + 1UZ)}});
    // Gardner rather than the default Mueller and Muller: it reads no decisions, so it recovers timing while the
    // constellation is still turning, which is the order this chain needs with the carrier loop behind it.
    auto& timing = graph.emplaceBlock<SymbolSync<CF>>({{"samples_per_symbol", static_cast<double>(kSps)}, {"detector", std::string("Gardner")}, {"rolloff", kRolloff}, {"interpolator", std::string("mmse8")}, {"noise_bandwidth", 0.002}});
    auto& costas = graph.emplaceBlock<CostasLoop>({{"order", gr::Size_t(4)}, {"noise_bandwidth", 0.005}});
    auto& sink   = graph.emplaceBlock<TagSink<CF, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

    boost::ut::expect(graph.connect<"out", "in">(source, shape).has_value());
    boost::ut::expect(graph.connect<"out", "in">(shape, carrier).has_value());
    boost::ut::expect(graph.connect<"out", "in">(carrier, noise).has_value());
    boost::ut::expect(graph.connect<"out", "in">(noise, matched).has_value());
    boost::ut::expect(graph.connect<"out", "in">(matched, timing).has_value());
    boost::ut::expect(graph.connect<"out", "in">(timing, costas).has_value());
    boost::ut::expect(graph.connect<"out", "in">(costas, sink).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    const auto run = scheduler.runAndWait();
    boost::ut::expect(run.has_value()) << (run.has_value() ? std::string{} : run.error().message);
    return {sink._samples.begin(), sink._samples.end()};
}

/**
 * @brief The bit error rate of the recovered symbols against the transmitted labels.
 *
 * A carrier loop on a four-fold constellation locks to any of four rotations and the chain's delay is the sum of two
 * filters and whatever phase the timing loop settled on, so both are measured rather than derived: the best of the
 * four rotations over a window of lags is the honest reading.
 */
struct Recovery {
    double         ber     = 1.0;
    std::ptrdiff_t lag     = 0;
    std::size_t    turns   = 0UZ;
    std::size_t    counted = 0UZ;
};

[[nodiscard]] Recovery bitErrorRate(const Payload& sent, std::span<const CF> received) {
    const auto map = gr::digital::Constellation<float>::qpsk();
    Recovery   best;

    for (std::size_t turns = 0UZ; turns < 4UZ; ++turns) {
        const double angle = 0.5 * std::numbers::pi * static_cast<double>(turns);
        const CF     turn(static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle)));
        for (std::ptrdiff_t lag = -40; lag <= 40; ++lag) {
            std::size_t errors  = 0UZ;
            std::size_t counted = 0UZ;
            for (std::size_t k = kSettle; k < received.size(); ++k) {
                const std::ptrdiff_t index = static_cast<std::ptrdiff_t>(k) + lag;
                if (index < 0 || index >= static_cast<std::ptrdiff_t>(sent.labels.size())) {
                    continue;
                }
                const auto decided = map.hardDecision(received[k] * turn);
                const auto wanted  = sent.labels[static_cast<std::size_t>(index)];
                errors += static_cast<std::size_t>(std::popcount(static_cast<unsigned>(decided ^ wanted)));
                ++counted;
            }
            if (counted > 0UZ) {
                const double rate = static_cast<double>(errors) / (2. * static_cast<double>(counted));
                if (rate < best.ber) {
                    best = Recovery{rate, lag, turns, counted};
                }
            }
        }
    }
    return best;
}

/// The error vector magnitude of a recovered stream, read from the block that owns the definition.
[[nodiscard]] double evmPercentOf(std::span<const CF> received, std::size_t turns) {
    const double angle = 0.5 * std::numbers::pi * static_cast<double>(turns);
    const CF     turn(static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle)));

    std::vector<CF> rotated(received.size() - kSettle);
    for (std::size_t k = 0UZ; k < rotated.size(); ++k) {
        rotated[k] = received[k + kSettle] * turn;
    }

    EvmMeter<float> meter;
    meter.constellation = std::string("qpsk");
    meter.window        = static_cast<gr::Size_t>(rotated.size());
    meter.start();
    // the meter carries an optional record port now; this reading is taken through the poll, so nothing is wired
    namespace test = gr::blocks::testing::span;
    std::vector<gr::DataSet<float>>      made(2UZ);
    test::InputSpan<CF>                  inSpan{std::span<const CF>(rotated)};
    test::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(made)};
    std::ignore = meter.processBulk(inSpan, outSpan);
    return meter.evmPercent();
}

} // namespace

const boost::ut::suite<"ModemDemo"> modemDemoTests = [] {
    using namespace boost::ut;

    "a clean channel recovers every symbol"_test = [] {
        const Payload sent     = payload(kSymbols, 0x243F6A8885A308D3ULL);
        const auto    received = throughTheModem(sent, 60.0, 0.0, 0.0, 0xC0FFEEULL);
        expect(gt(received.size(), kSymbols / 2UZ)) << std::format("the chain returned {} symbols", received.size());

        const Recovery recovery = bitErrorRate(sent, received);
        const double   evm      = evmPercentOf(received, recovery.turns);
        std::println("ModemDemo clean: {} symbols, BER {:.3e} at lag {} and {} quarter turns, EVM {:.3f}%", received.size(), recovery.ber, recovery.lag, recovery.turns, evm);

        expect(eq(recovery.ber, 0.0)) << "a noiseless channel leaves no bit in error";
        expect(lt(evm, 3.0)) << std::format("the residual error vector is {:.3f}% of the reference amplitude", evm);
    };

    "a carrier offset is tracked and the constellation closes"_test = [] {
        const Payload sent     = payload(kSymbols, 0x243F6A8885A308D3ULL);
        const auto    received = throughTheModem(sent, 60.0, 1e-4, 0.0, 0xC0FFEEULL);

        const Recovery recovery = bitErrorRate(sent, received);
        const double   evm      = evmPercentOf(received, recovery.turns);
        std::println("ModemDemo offset: BER {:.3e} at lag {} and {} quarter turns, EVM {:.3f}%", recovery.ber, recovery.lag, recovery.turns, evm);

        expect(lt(recovery.ber, 1e-4)) << "the carrier loop holds the constellation still";
        expect(lt(evm, 12.0)) << std::format("the residual error vector is {:.3f}%", evm); // 8.58% measured: the loop tracks the offset but does not null it
    };

    "the bit error rate follows the noise"_test = [] {
        const Payload sent = payload(kSymbols, 0x243F6A8885A308D3ULL);

        double previous = 1.0;
        for (const double esN0_db : {6.0, 9.0, 12.0, 15.0}) {
            const auto     received = throughTheModem(sent, esN0_db, 1e-5, 0.0, 0xC0FFEEULL);
            const Recovery recovery = bitErrorRate(sent, received);
            const double   evm      = evmPercentOf(received, recovery.turns);

            // Coherent QPSK on a Gray map carries one bit per axis, so the reference is the antipodal bound at the
            // same energy per bit, which for QPSK equals the energy per symbol halved.
            const double ebN0      = std::pow(10., esN0_db / 10.) / 2.;
            const double reference = 0.5 * std::erfc(std::sqrt(ebN0));
            std::println("ModemDemo Es/N0 {:4.1f} dB: BER {:.3e}, coherent reference {:.3e}, ratio {:.2f}, EVM {:.2f}%", esN0_db, recovery.ber, reference, recovery.ber / reference, evm);

            expect(lt(recovery.ber, previous)) << std::format("Es/N0 {} dB: the rate falls as the noise does", esN0_db);
            expect(lt(recovery.ber, 8. * reference)) << std::format("Es/N0 {} dB: BER {:.3e} against eight times the coherent bound", esN0_db, recovery.ber);
            previous = recovery.ber;
        }
    };
};

int main() { /* not needed for UT */ }
