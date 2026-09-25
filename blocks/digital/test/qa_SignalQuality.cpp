#include <boost/ut.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <iterator>
#include <print>
#include <span>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/channel/Awgn.hpp>
#include <gnuradio-4.0/digital/SignalQuality.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

#include <gnuradio-4.0/algorithm/digital/Constellation.hpp>

namespace {
/// @brief Drives a meter over `input` to exhaustion and returns the records it published, which most cases ignore.
///
/// The meter leaves the last symbol of a call unconsumed so the framework's end-of-stream epilogue has a span to run
/// on, and takes at most as many symbols as the output span has room for the records they close. Calling until the
/// span is spent is therefore what a scheduler does, and what makes one call here mean one stretch of stream.
template<typename TBlock>
std::vector<gr::DataSet<float>> drive(TBlock& block, std::span<const std::complex<float>> input) {
    namespace test = gr::blocks::testing::span;
    std::vector<gr::DataSet<float>> records;

    for (std::size_t base = 0UZ; base < input.size();) {
        std::vector<gr::DataSet<float>>      made(4UZ);
        test::InputSpan<std::complex<float>> inSpan(input.subspan(base));
        test::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(made)};
        std::ignore = block.processBulk(inSpan, outSpan);
        records.insert(records.end(), std::make_move_iterator(made.begin()), std::make_move_iterator(made.begin() + static_cast<std::ptrdiff_t>(outSpan.count)));
        if (inSpan.consumed == 0UZ) {
            break;
        }
        base += inSpan.consumed;
    }
    return records;
}

/// @brief What a meter emits when the stream ends here: the trailing symbols folded in, then the record covering the
/// window in progress with the count that actually reached it.
template<typename TBlock>
std::vector<gr::DataSet<float>> finish(TBlock& block, std::span<const std::complex<float>> trailing) {
    namespace test = gr::blocks::testing::span;
    std::vector<gr::DataSet<float>>      made(4UZ);
    test::InputSpan<std::complex<float>> inSpan(trailing);
    test::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(made)};
    std::ignore = block.processEpilogue(inSpan, outSpan);
    made.resize(outSpan.count);
    return made;
}
} // namespace

namespace {

using gr::blocks::channel::AwgnChannel;
using gr::blocks::digital::EvmMeter;
using gr::blocks::digital::SnrEstimator;
using gr::digital::Constellation;

using CF = std::complex<float>;

/// Both meters hold atomics and are therefore not movable, so a block is built in place and initialized by reference.
template<typename TBlock>
void init(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

template<typename TBlock>
void restage(TBlock& block, gr::property_map settings) {
    std::ignore = block.settings().set(std::move(settings));
    std::ignore = block.settings().activateContext();
    std::ignore = block.settings().applyStagedParameters();
}

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
};

/// One complex sample per symbol, drawn uniformly from the point set and already at the constellation's own scale.
[[nodiscard]] std::vector<CF> symbolStream(const Constellation<float>& constellation, std::size_t count, std::uint64_t seed) {
    Rng             rng{seed};
    std::vector<CF> stream(count);
    for (CF& symbol : stream) {
        symbol = constellation.point(static_cast<std::uint8_t>(rng.next() % constellation.size()));
    }
    return stream;
}

/**
 * @brief The noise power that realizes @p snrDb against unit-power symbols.
 *
 * The channel's complex convention is `nI, nQ ~ N(0, 1/2)`, so `E[|n|^2]` is `noise_power` exactly. The symbol streams
 * here carry unit mean power at one sample per symbol, which makes the wanted noise power `10^(-snr/10)`;
 * `gr::channel::noisePowerFor()` would be handed a symbol energy and an oversampling of one and return the same number, so the
 * operating point is stated directly in the units the channel is parameterized in.
 */
[[nodiscard]] double noisePowerAt(double snrDb) { return std::pow(10., -0.1 * snrDb); }

[[nodiscard]] std::vector<CF> throughAwgn(std::span<const CF> symbols, double noisePower, std::uint64_t seed) {
    std::vector<CF> received(symbols.size());
    AwgnChannel<CF> channel({{"noise_power", noisePower}, {"seed", seed}});
    init(channel);
    std::ignore = channel.processBulk(symbols, std::span<CF>(received));
    return received;
}

/// Push @p stream through @p block in call sizes taken cyclically from @p pattern.
template<typename TBlock>
void feed(TBlock& block, std::span<const CF> stream, std::span<const std::size_t> pattern) {
    std::size_t at   = 0UZ;
    std::size_t step = 0UZ;
    while (at < stream.size()) {
        const std::size_t count = std::min(pattern[step % pattern.size()], stream.size() - at);
        std::ignore             = drive(block, stream.subspan(at, count));
        at += count;
        ++step;
    }
}

/// @brief A noisy QPSK stream at a stated signal-to-noise ratio, for the record-port cases below.
[[nodiscard]] std::vector<CF> noisyStream(double snrDb, std::size_t count) {
    const Constellation<float> qpsk    = gr::digital::constellationFromName<float>("qpsk", 4UZ, 0.f, 0U, {}, "power");
    const auto                 symbols = symbolStream(qpsk, count, 20260901ULL);
    return throughAwgn(std::span<const CF>(symbols), noisePowerAt(snrDb), 4242ULL);
}

[[nodiscard]] double snrReading(std::span<const CF> received, const std::string& method, std::size_t window, gr::property_map extra = {}) {
    gr::property_map settings{{"method", method}, {"window", static_cast<gr::Size_t>(window)}};
    settings.insert(extra.begin(), extra.end());
    SnrEstimator<float> block(std::move(settings));
    init(block);
    std::ignore = drive(block, received);
    return block.snrDb();
}

/// `E[|s|^4] / E[|s|^2]^2` over the point set, the quantity the m2m4 derivation takes to be one.
[[nodiscard]] double kurtosisOf(const Constellation<float>& constellation) {
    double secondMoment = 0.;
    double fourthMoment = 0.;
    for (const CF& point : constellation.points()) {
        const double power = static_cast<double>(point.real()) * static_cast<double>(point.real()) + static_cast<double>(point.imag()) * static_cast<double>(point.imag());
        secondMoment += power;
        fourthMoment += power * power;
    }
    const double count = static_cast<double>(constellation.points().size());
    secondMoment /= count;
    fourthMoment /= count;
    return fourthMoment / (secondMoment * secondMoment);
}

/// What m2m4 reports for unit-power symbols of kurtosis @p kurtosis in noise of power @p noisePower: the estimator
/// recovers `sqrt(2 - kurtosis)` of the signal power and charges the remainder to the noise.
[[nodiscard]] double m2m4Prediction(double kurtosis, double noisePower) {
    const double signal = std::sqrt(2. - kurtosis);
    return 10. * std::log10(signal / (1. + noisePower - signal));
}

[[nodiscard]] bool allFinite(const EvmMeter<float>& block) { return std::isfinite(block.evmRms()) && std::isfinite(block.evmPercent()) && std::isfinite(block.evmDb()) && std::isfinite(block.merDb()) && std::isfinite(block.evmPeak()) && std::isfinite(block.coverage()); }

[[nodiscard]] bool allFinite(const SnrEstimator<float>& block) { return std::isfinite(block.snrLinear()) && std::isfinite(block.snrDb()) && std::isfinite(block.coverage()); }

/// The sorted, deduplicated set of readings @p block publishes as it closes one window after another over @p stream,
/// whose length is a whole number of windows. Every value a reader can observe once the first window has closed is a
/// member of this set, whatever call sizes the stream is delivered in.
template<typename TBlock, typename TRead>
[[nodiscard]] std::vector<double> closedWindowValues(TBlock& block, std::span<const CF> stream, std::size_t window, TRead read) {
    std::vector<double> values;
    for (std::size_t at = 0UZ; at + window <= stream.size(); at += window) {
        std::ignore = drive(block, stream.subspan(at, window));
        values.push_back(read(block));
    }
    std::ranges::sort(values);
    const auto duplicates = std::ranges::unique(values);
    values.erase(duplicates.begin(), duplicates.end());
    return values;
}

} // namespace

const boost::ut::suite<"signal quality"> signalQualityTests = [] {
    using namespace boost::ut;

    "a known signal-to-noise ratio reads back"_test = [] {
        constexpr std::size_t kSymbols = 200000UZ;

        // The m2m4 envelopes are measured rather than derived: each is roughly twice the error the fixed seed of that
        // row actually produces, with a floor of 0.05 dB so that a row whose realized error happens to land near zero
        // asserts the estimator's accuracy rather than a coincidence of one noise realization. The error at every row
        // is dominated by the realized noise power of a 200000-symbol draw, which is about 0.01 dB wide.
        //
        // `decisionExcess` is how far the decision-directed reading sits above truth on the rows where decision errors
        // dominate. From 10 dB up the reading is held within 0.2 dB either way instead.
        struct Point {
            double        snrDb;
            std::uint64_t seed;
            double        m2m4Envelope;
            double        decisionExcess;
        };
        constexpr Point kGrid[]{
            {0.0, 0x13198a2e03707344ULL, 0.05, 1.763},
            {5.0, 0xa4093822299f31d0ULL, 0.05, 0.495},
            {10.0, 0x082efa98ec4e6c89ULL, 0.05, 0.0},
            {15.0, 0x452821e638d01377ULL, 0.07, 0.0},
            {20.0, 0xbe5466cf34e90c6cULL, 0.05, 0.0},
            {30.0, 0xc0ac29b7c97c50ddULL, 0.05, 0.0},
        };

        const std::vector<CF> transmitted = symbolStream(Constellation<float>::qpsk(), kSymbols, 0x243f6a8885a308d3ULL);
        std::println("QPSK, {} symbols per window", kSymbols);
        for (const auto& [snrDb, seed, m2m4Envelope, decisionExcess] : kGrid) {
            const std::vector<CF> received = throughAwgn(std::span<const CF>(transmitted), noisePowerAt(snrDb), seed);
            const double          blind    = snrReading(std::span<const CF>(received), "m2m4", kSymbols);
            const double          directed = snrReading(std::span<const CF>(received), "decision_directed", kSymbols);
            std::println("  {:4.1f} dB: m2m4 {:8.4f} ({:+.4f})   decision_directed {:8.4f} ({:+.4f})", snrDb, blind, blind - snrDb, directed, directed - snrDb);

            expect(lt(std::abs(blind - snrDb), m2m4Envelope)) << std::format("m2m4 at {} dB read {:.4f}", snrDb, blind);
            if (snrDb >= 10.0) {
                expect(lt(std::abs(directed - snrDb), 0.2)) << std::format("decision_directed at {} dB read {:.4f}", snrDb, directed);
            } else {
                expect(approx(directed - snrDb, decisionExcess, 0.1)) << std::format("decision_directed at {} dB read {:.4f}, {:+.4f} above truth", snrDb, directed, directed - snrDb);
            }
        }

        const std::vector<CF> atZeroDb = throughAwgn(std::span<const CF>(transmitted), noisePowerAt(kGrid[0].snrDb), kGrid[0].seed);
        SnrEstimator<float>   counted({{"method", std::string("m2m4")}, {"window", static_cast<gr::Size_t>(kSymbols)}});
        init(counted);
        std::ignore = drive(counted, std::span<const CF>(atZeroDb));
        expect(eq(counted.nWindows(), std::uint64_t{1})) << "one window of 200000 symbols closed";
        expect(eq(counted.nDegenerate(), std::uint64_t{0})) << "a window this long admits positive signal power even at 0 dB";
    };

    "modulation error ratio and a decision-directed signal-to-noise ratio are one quantity"_test = [] {
        constexpr std::size_t kWindows[]{4096UZ, 32768UZ};
        constexpr double      kLevels[]{0.0, 5.0, 10.0, 20.0, 30.0};

        // Both blocks divide the same sum of squared decision errors by the same count and the same reference power,
        // one taking 20*log10 of the square root of the ratio and the other -10*log10 of its reciprocal. The readings
        // therefore differ by units in the last place of a double, at most 1.8e-15 dB on this grid, and 1e-12 dB is a
        // margin of more than five hundred over that.
        constexpr double kTolerance = 1e-12;

        double worst = 0.;
        for (const std::size_t window : kWindows) {
            const std::vector<CF> transmitted = symbolStream(Constellation<float>::qpsk(), 8UZ * window, 0x243f6a8885a308d3ULL);
            for (const double snrDb : kLevels) {
                const std::vector<CF> received = throughAwgn(std::span<const CF>(transmitted), noisePowerAt(snrDb), 0x9216d5d98979fb1bULL);

                EvmMeter<float> evm({{"window", static_cast<gr::Size_t>(window)}});
                init(evm);
                std::ignore = drive(evm, std::span<const CF>(received));

                SnrEstimator<float> snr({{"method", std::string("decision_directed")}, {"window", static_cast<gr::Size_t>(window)}});
                init(snr);
                std::ignore = drive(snr, std::span<const CF>(received));

                const double difference = std::abs(evm.merDb() - snr.snrDb());
                worst                   = std::max(worst, difference);
                expect(lt(difference, kTolerance)) << std::format("window {} at {} dB: merDb {:.15g} against snrDb {:.15g}", window, snrDb, evm.merDb(), snr.snrDb());
                expect(eq(evm.merDb(), -evm.evmDb())) << "the modulation error ratio is the error vector magnitude with the opposite sign";
            }
        }
        std::println("mer against a decision-directed snr: worst difference {:.3e} dB over {} points", worst, std::size(kWindows) * std::size(kLevels));
    };

    "no reader returns an infinity or a not-a-number"_test = [] {
        const std::vector<CF> exact = symbolStream(Constellation<float>::qpsk(), 4096UZ, 0x243f6a8885a308d3ULL);

        EvmMeter<float> evm({{"window", 1024U}});
        init(evm);
        std::ignore = drive(evm, std::span<const CF>(exact));
        expect(eq(evm.evmRms(), 0.0)) << "a symbol sitting on its own constellation point contributes exactly no error";
        expect(eq(evm.evmPeak(), 0.0));
        expect(eq(evm.evmPercent(), 0.0));
        expect(approx(evm.evmDb(), -400.0, 1e-9)) << "the floor of 1e-20 rather than negative infinity";
        expect(eq(evm.merDb(), -evm.evmDb()));
        expect(that % allFinite(evm));

        for (const std::string& method : {std::string("m2m4"), std::string("decision_directed")}) {
            for (const double ceiling : {100.0, 60.0}) {
                SnrEstimator<float> snr({{"method", method}, {"window", 1024U}, {"ceiling_db", ceiling}});
                init(snr);
                std::ignore = drive(snr, std::span<const CF>(exact));
                expect(approx(snr.snrDb(), ceiling, 1e-9)) << std::format("{}: noiseless symbols clamp at ceiling_db", method);
                expect(that % allFinite(snr)) << method;
                expect(eq(snr.nWindows(), std::uint64_t{4}));
                expect(eq(snr.nDegenerate(), std::uint64_t{0}));
            }
        }

        EvmMeter<float> freshEvm({{"window", 1024U}});
        init(freshEvm);
        expect(eq(freshEvm.evmRms(), 0.0));
        expect(approx(freshEvm.evmDb(), -400.0, 1e-9));
        expect(eq(freshEvm.coverage(), 0.0)) << "a window that has seen nothing covers none of itself";
        expect(that % allFinite(freshEvm));

        SnrEstimator<float> freshSnr({{"window", 1024U}});
        init(freshSnr);
        expect(eq(freshSnr.snrLinear(), 0.0));
        expect(eq(freshSnr.snrDb(), -freshSnr.ceiling_db.value)) << "an empty window reads the negative ceiling rather than negative infinity";
        expect(eq(freshSnr.coverage(), 0.0));
        expect(eq(freshSnr.nWindows(), std::uint64_t{0}));
        expect(eq(freshSnr.nDegenerate(), std::uint64_t{0}));
        expect(that % allFinite(freshSnr));

        // Short windows at a negative signal-to-noise ratio drive 2*M2^2 - M4 below zero often enough to exercise the
        // path that reports zero, and the reading stays finite through it.
        const std::vector<CF> noisy = throughAwgn(std::span<const CF>(symbolStream(Constellation<float>::qpsk(), 200000UZ, 0x2ff24fa2ee84e39cULL)), noisePowerAt(-6.0), 0xbe5466cf34e90c6cULL);
        SnrEstimator<float>   blind({{"method", std::string("m2m4")}, {"window", 64U}});
        init(blind);
        std::ignore = drive(blind, std::span<const CF>(noisy));
        std::println("m2m4 at -6 dB over 64-symbol windows: {} of {} windows admitted no positive signal power", blind.nDegenerate(), blind.nWindows());
        expect(gt(blind.nDegenerate(), std::uint64_t{0}));
        expect(le(blind.nDegenerate(), blind.nWindows()));
        expect(that % allFinite(blind));
    };

    "m2m4 on 16QAM is biased by the constellation's kurtosis"_test = [] {
        constexpr std::size_t kSymbols = 200000UZ;

        // The readings are measured; the envelope is what the bias must stay inside so that a change to either block
        // cannot move it unnoticed. The bias is a property of the point set and not of the noise, so it does not
        // shrink with a longer window.
        struct Level {
            double        snrDb;
            std::uint64_t seed;
            double        reading;
        };
        constexpr Level kLevels[]{
            {10.0, 0x9216d5d98979fb25ULL, 4.752},
            {15.0, 0x9216d5d98979fb2aULL, 6.017},
        };
        constexpr double kEnvelope = 0.10;

        const Constellation<float> constellation = Constellation<float>::qam(16UZ);
        const double               kurtosis      = kurtosisOf(constellation);
        const std::vector<CF>      transmitted   = symbolStream(constellation, kSymbols, 0x452821e638d01377ULL);
        std::println("16QAM has a signal kurtosis of {:.6f} where m2m4 assumes one", kurtosis);

        for (const auto& [snrDb, seed, reading] : kLevels) {
            const std::vector<CF> received  = throughAwgn(std::span<const CF>(transmitted), noisePowerAt(snrDb), seed);
            const double          blind     = snrReading(std::span<const CF>(received), "m2m4", kSymbols, {{"constellation", std::string("qam")}, {"arity", 16U}});
            const double          predicted = m2m4Prediction(kurtosis, noisePowerAt(snrDb));
            std::println("  {:4.1f} dB: m2m4 {:8.4f}, bias {:+.4f} dB (the kurtosis predicts {:.4f})", snrDb, blind, blind - snrDb, predicted);

            expect(approx(blind, reading, kEnvelope)) << std::format("m2m4 on 16QAM at {} dB read {:.4f}", snrDb, blind);
            expect(approx(blind, predicted, kEnvelope)) << std::format("m2m4 on 16QAM at {} dB is the kurtosis bias and nothing else", snrDb);
            expect(lt(blind, snrDb)) << "the assumed kurtosis of one understates the signal power of a 16QAM stream";
        }
    };

    "coverage and the counters follow the window"_test = [] {
        const std::vector<CF> received = throughAwgn(std::span<const CF>(symbolStream(Constellation<float>::qpsk(), 8192UZ, 0x082efa98ec4e6c89ULL)), noisePowerAt(12.0), 0x77ULL);

        EvmMeter<float> evm({{"window", 1000U}});
        init(evm);
        expect(eq(evm.coverage(), 0.0));
        for (const std::size_t at : {1UZ, 250UZ, 500UZ, 999UZ}) {
            EvmMeter<float> partial({{"window", 1000U}});
            init(partial);
            std::ignore = drive(partial, std::span<const CF>(received).first(at));
            expect(approx(partial.coverage(), static_cast<double>(at) / 1000., 1e-12)) << std::format("after {} of 1000 symbols", at);
            expect(lt(partial.coverage(), 1.0));
            expect(that % allFinite(partial));
        }
        std::ignore = drive(evm, std::span<const CF>(received).first(1000UZ));
        expect(eq(evm.coverage(), 1.0)) << "a closed window covers itself";

        SnrEstimator<float> snr({{"method", std::string("m2m4")}, {"window", 1000U}});
        init(snr);
        std::ignore = drive(snr, std::span<const CF>(received).first(999UZ));
        expect(lt(snr.coverage(), 1.0));
        expect(eq(snr.nWindows(), std::uint64_t{0})) << "nothing has closed yet";
        std::ignore = drive(snr, std::span<const CF>(received).subspan(999UZ, 1UZ));
        expect(eq(snr.coverage(), 1.0));
        expect(eq(snr.nWindows(), std::uint64_t{1}));

        std::ignore = drive(snr, std::span<const CF>(received).subspan(1000UZ, 7000UZ));
        expect(eq(snr.nWindows(), std::uint64_t{8}));
        const double before = snr.snrDb();
        expect(that % std::isfinite(before));

        restage(snr, {{"constellation", std::string("qam")}, {"arity", 16U}});
        expect(eq(snr.nWindows(), std::uint64_t{0})) << "a settings change restarts the window and the counters with it";
        expect(eq(snr.nDegenerate(), std::uint64_t{0}));
        expect(eq(snr.coverage(), 0.0));
        expect(eq(snr.snrLinear(), 0.0));
        expect(that % allFinite(snr));

        restage(evm, {{"window", 512U}});
        expect(eq(evm.coverage(), 0.0));
        expect(eq(evm.evmRms(), 0.0));
        expect(eq(evm.evmPeak(), 0.0));
        expect(that % allFinite(evm));
    };

    "a reader in another thread sees whole windows"_test = [] {
        constexpr std::size_t kWindow = 1000UZ;
        constexpr std::size_t kStream = 400000UZ;
        constexpr std::size_t kChunk  = 251UZ;
        constexpr std::size_t kPiece  = 4000UZ;

        // The level alternates every four windows, so the published sequence spans a wide range and a reader that
        // mixed two windows would land between the levels rather than on one of them.
        const std::vector<CF> transmitted = symbolStream(Constellation<float>::qpsk(), kStream, 0x13198a2e03707344ULL);
        std::vector<CF>       received(kStream);
        for (std::size_t at = 0UZ; at < kStream; at += kPiece) {
            const double          snrDb = (at / kPiece) % 2UZ == 0UZ ? 25.0 : 3.0;
            const std::vector<CF> piece = throughAwgn(std::span<const CF>(transmitted).subspan(at, kPiece), noisePowerAt(snrDb), 0xc0ac29b7c97c50ddULL + at);
            std::ranges::copy(piece, received.begin() + static_cast<std::ptrdiff_t>(at));
        }

        EvmMeter<float> evmReference({{"window", static_cast<gr::Size_t>(kWindow)}});
        init(evmReference);
        const std::vector<double> evmValues = closedWindowValues(evmReference, std::span<const CF>(received), kWindow, [](const EvmMeter<float>& block) { return block.evmDb(); });

        SnrEstimator<float> snrReference({{"method", std::string("decision_directed")}, {"window", static_cast<gr::Size_t>(kWindow)}});
        init(snrReference);
        const std::vector<double> snrValues = closedWindowValues(snrReference, std::span<const CF>(received), kWindow, [](const SnrEstimator<float>& block) { return block.snrDb(); });

        EvmMeter<float> evm({{"window", static_cast<gr::Size_t>(kWindow)}});
        init(evm);
        SnrEstimator<float> snr({{"method", std::string("decision_directed")}, {"window", static_cast<gr::Size_t>(kWindow)}});
        init(snr);

        // The reader starts once the first window has closed, so every value it can observe is a closed window's.
        std::ignore = drive(evm, std::span<const CF>(received).first(kWindow));
        std::ignore = drive(snr, std::span<const CF>(received).first(kWindow));

        std::atomic<bool>          running{true};
        std::atomic<bool>          ready{false};
        std::atomic<std::uint64_t> polls{0ULL};
        std::atomic<std::uint64_t> rejected{0ULL};

        std::thread reader([&] {
            std::uint64_t seen = 0ULL;
            std::uint64_t bad  = 0ULL;
            ready.store(true, std::memory_order_release);
            while (running.load(std::memory_order_relaxed)) {
                const double evmDb = evm.evmDb();
                const double snrDb = snr.snrDb();
                ++seen;
                const bool evmOk = std::isfinite(evmDb) && evmDb >= evmValues.front() && evmDb <= evmValues.back() && std::binary_search(evmValues.begin(), evmValues.end(), evmDb);
                const bool snrOk = std::isfinite(snrDb) && snrDb >= snrValues.front() && snrDb <= snrValues.back() && std::binary_search(snrValues.begin(), snrValues.end(), snrDb);
                bad += evmOk && snrOk ? 0ULL : 1ULL;
            }
            polls.store(seen, std::memory_order_relaxed);
            rejected.store(bad, std::memory_order_relaxed);
        });

        while (!ready.load(std::memory_order_acquire)) {
        }
        for (std::size_t at = kWindow; at < kStream; at += kChunk) {
            const std::size_t count = std::min(kChunk, kStream - at);
            std::ignore             = drive(evm, std::span<const CF>(received).subspan(at, count));
            std::ignore             = drive(snr, std::span<const CF>(received).subspan(at, count));
        }
        running.store(false, std::memory_order_relaxed);
        reader.join();

        std::println("reader: {} polls over {} windows, evmDb in [{:.4f}, {:.4f}], snrDb in [{:.4f}, {:.4f}]", polls.load(), snr.nWindows(), evmValues.front(), evmValues.back(), snrValues.front(), snrValues.back());
        expect(gt(polls.load(), std::uint64_t{0})) << "the reader ran while the block processed";
        expect(eq(rejected.load(), std::uint64_t{0})) << "every value the reader saw was finite, inside the range the stream produces, and one the block published";
        expect(eq(snr.nWindows(), std::uint64_t{kStream / kWindow}));
    };

    "the readings do not depend on the chunking"_test = [] {
        constexpr std::size_t kSymbols = 30000UZ;
        constexpr std::size_t kWindows[]{1024UZ, 4096UZ, 7000UZ};
        const std::size_t     partitionA[]{1UZ, 2UZ, 3UZ, 5UZ, 8UZ, 13UZ, 21UZ, 34UZ, 4096UZ, 7UZ};
        const std::size_t     partitionB[]{4095UZ, 1UZ, 999UZ, 2UZ, 17UZ, 64UZ};

        const std::vector<CF> received = throughAwgn(std::span<const CF>(symbolStream(Constellation<float>::qpsk(), kSymbols, 0x2ff24fa2ee84e39cULL)), noisePowerAt(10.0), 0x082efa98ec4e6c89ULL);
        const std::size_t     whole[]{kSymbols};

        for (const std::size_t window : kWindows) {
            EvmMeter<float> evmReference({{"window", static_cast<gr::Size_t>(window)}});
            init(evmReference);
            feed(evmReference, std::span<const CF>(received), std::span<const std::size_t>(whole));

            for (const std::span<const std::size_t> partition : {std::span<const std::size_t>(partitionA), std::span<const std::size_t>(partitionB)}) {
                EvmMeter<float> evm({{"window", static_cast<gr::Size_t>(window)}});
                init(evm);
                feed(evm, std::span<const CF>(received), partition);
                const bool identical = evm.evmRms() == evmReference.evmRms() && evm.evmPercent() == evmReference.evmPercent() && evm.evmDb() == evmReference.evmDb() && evm.merDb() == evmReference.merDb() && evm.evmPeak() == evmReference.evmPeak() && evm.coverage() == evmReference.coverage();
                expect(that % identical) << std::format("EvmMeter, window {}, partition starting at {}", window, partition.front());
            }

            for (const std::string& method : {std::string("m2m4"), std::string("decision_directed")}) {
                SnrEstimator<float> snrReference({{"method", method}, {"window", static_cast<gr::Size_t>(window)}});
                init(snrReference);
                feed(snrReference, std::span<const CF>(received), std::span<const std::size_t>(whole));

                for (const std::span<const std::size_t> partition : {std::span<const std::size_t>(partitionA), std::span<const std::size_t>(partitionB)}) {
                    SnrEstimator<float> snr({{"method", method}, {"window", static_cast<gr::Size_t>(window)}});
                    init(snr);
                    feed(snr, std::span<const CF>(received), partition);
                    const bool identical = snr.snrLinear() == snrReference.snrLinear() && snr.snrDb() == snrReference.snrDb() && snr.coverage() == snrReference.coverage() && snr.nWindows() == snrReference.nWindows() && snr.nDegenerate() == snrReference.nDegenerate();
                    expect(that % identical) << std::format("SnrEstimator {}, window {}, partition starting at {}", method, window, partition.front());
                }
            }
        }
    };

    "degenerate settings"_test = [] {
        // A throwing applyStagedParameters() leaves the offending value staged, so each case gets a block of its own.
        expect(throws([] {
            EvmMeter<float> rejected({{"mode", std::string("known_reference")}});
            init(rejected);
        })) << "decision_directed is the only mode the block implements";
        expect(throws([] {
            EvmMeter<float> rejected({{"window", 0U}});
            init(rejected);
        }));
        expect(throws([] {
            EvmMeter<float> rejected({{"constellation", std::string("dqpsk")}});
            init(rejected);
        }));

        expect(throws([] {
            SnrEstimator<float> rejected({{"method", std::string("m2m6")}});
            init(rejected);
        }));
        expect(throws([] {
            SnrEstimator<float> rejected({{"method", std::string("")}});
            init(rejected);
        }));
        expect(throws([] {
            SnrEstimator<float> rejected({{"window", 0U}});
            init(rejected);
        }));
        expect(throws([] {
            SnrEstimator<float> rejected({{"ceiling_db", 0.0}});
            init(rejected);
        })) << "ceiling_db is a magnitude";
        expect(throws([] {
            SnrEstimator<float> rejected({{"ceiling_db", -1.0}});
            init(rejected);
        }));
        expect(throws([] {
            SnrEstimator<float> rejected({{"constellation", std::string("dqpsk")}});
            init(rejected);
        })) << "the point set is validated under m2m4 too, which does not read it";

        expect(throws([] {
            SnrEstimator<float> live({{"window", 1024U}});
            init(live);
            restage(live, {{"window", 0U}});
        })) << "on a live change as well";
    };

    "the record port publishes a closed window, and states it in symbols"_test = [] {
        const auto scene = noisyStream(20.0, 4096UZ * 2UZ);

        EvmMeter<float> evm({{"constellation", std::string("qpsk")}, {"window", gr::Size_t{1024U}}});
        evm.settings().init();
        std::ignore = evm.settings().applyStagedParameters();
        evm.start();

        const auto records = drive(evm, std::span<const CF>(scene));
        expect(!records.empty()) << "a window closed inside the call, so a record went out";
        if (records.empty()) {
            return;
        }
        // Every closed window is its own record now, so the one that matches a poll after the run is the last.
        const auto& record = records.back();
        expect(eq(record.signal_names.size(), 6UZ));
        expect(eq(record.signal_names[0UZ], std::string("evm_rms")));
        expect(eq(record.signal_units[2UZ], std::string("dB")));
        expect(approx(static_cast<double>(record.signal_values[0UZ]), evm.evmRms(), 1e-6)) << "the record carries the reading the poller sees";

        const auto& meta = record.meta_information[0UZ];
        expect(meta.find(std::pmr::string("sample_rate")) == meta.end()) << "a symbol-domain meter states no sample rate rather than a zero one";
        expect(meta.find(std::pmr::string("sample_start")) != meta.end());
        expect(meta.find(std::pmr::string("index_unit")) != meta.end()) << "and says what its index counts";
    };

    "a meter with its record port unconnected measures exactly as before"_test = [] {
        const auto scene = noisyStream(20.0, 4096UZ);

        SnrEstimator<float> connected({{"method", std::string("m2m4")}, {"window", gr::Size_t{1024U}}});
        connected.settings().init();
        std::ignore = connected.settings().applyStagedParameters();
        connected.start();
        std::ignore = drive(connected, std::span<const CF>(scene));

        // the same block driven through a span that reports itself unconnected, which is what an unwired port is
        SnrEstimator<float> bare({{"method", std::string("m2m4")}, {"window", gr::Size_t{1024U}}});
        bare.settings().init();
        std::ignore = bare.settings().applyStagedParameters();
        bare.start();
        {
            namespace test        = gr::blocks::testing::span;
            std::size_t published = 0UZ;
            for (std::size_t base = 0UZ; base < scene.size();) {
                std::vector<gr::DataSet<float>>      made(4UZ);
                test::InputSpan<CF>                  inSpan{std::span<const CF>(scene).subspan(base)};
                test::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(made), 0UZ, nullptr, false};
                std::ignore = bare.processBulk(inSpan, outSpan);
                published += outSpan.count;
                if (inSpan.consumed == 0UZ) {
                    break;
                }
                base += inSpan.consumed;
            }
            expect(eq(published, 0UZ)) << "an unconnected port has nothing published to it";
        }
        expect(eq(connected.snrLinear(), bare.snrLinear())) << "the reading does not depend on whether anyone is listening";
    };

    "several windows closing in one call are several records, each stamped at its own first symbol"_test = [] {
        constexpr std::size_t kWindow  = 512UZ;
        constexpr std::size_t kWindows = 8UZ;
        const auto            scene    = noisyStream(20.0, kWindow * kWindows + 1UZ); // one spare, since a call holds one back

        EvmMeter<float> evm({{"constellation", std::string("qpsk")}, {"window", gr::Size_t{kWindow}}});
        init(evm);

        namespace test = gr::blocks::testing::span;
        std::vector<gr::DataSet<float>>      made(16UZ);
        test::InputSpan<CF>                  inSpan{std::span<const CF>(scene)};
        test::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(made)};
        std::ignore = evm.processBulk(inSpan, outSpan);

        expect(eq(outSpan.count, kWindows)) << "eight windows closed inside the one call and each is its own record";
        for (std::size_t w = 0UZ; w < std::min(outSpan.count, kWindows); ++w) {
            const auto& meta  = made[w].meta_information[0UZ];
            const auto  entry = meta.find(std::pmr::string("sample_start"));
            expect(entry != meta.end()) << "record " << w << " states no place in the stream";
            if (entry != meta.end()) {
                const auto* index = entry->second.get_if<std::uint64_t>();
                expect(index != nullptr && *index == w * kWindow) << "record " << w << " is stamped at its own window's first symbol";
            }
        }
    };

    "a stream ending mid-window emits a final record marked with the symbols it covers"_test = [] {
        constexpr std::size_t kWindow  = 1024UZ;
        constexpr std::size_t kPartial = 300UZ;
        const auto            scene    = noisyStream(20.0, kWindow + kPartial);

        SnrEstimator<float> snr({{"method", std::string("decision_directed")}, {"window", gr::Size_t{kWindow}}});
        init(snr);

        const auto during = drive(snr, std::span<const CF>(scene));
        expect(eq(during.size(), 1UZ)) << "one whole window closed while the stream ran";

        const auto last = finish(snr, std::span<const CF>{});
        expect(eq(last.size(), 1UZ)) << "and the window in progress is reported rather than discarded";
        if (last.empty()) {
            return;
        }
        expect(approx(static_cast<double>(last.front().signal_values[2UZ]), static_cast<double>(kPartial) / static_cast<double>(kWindow), 1e-4)) << "the final record states the fraction of a window it actually covers";
        const auto& meta  = last.front().meta_information[0UZ];
        const auto  entry = meta.find(std::pmr::string("sample_start"));
        expect(entry != meta.end());
        if (entry != meta.end()) {
            const auto* index = entry->second.get_if<std::uint64_t>();
            expect(index != nullptr && *index == kWindow) << "and starts where the last whole window ended";
        }
    };

    "an idle meter answers that it waits for input, not progress"_test = [] {
        namespace test = gr::blocks::testing::span;
        EvmMeter<float> evm({{"constellation", std::string("qpsk")}});
        init(evm);
        SnrEstimator<float> snr({{"method", std::string("m2m4")}});
        init(snr);
        std::vector<gr::DataSet<float>> room(4UZ);

        test::InputSpan<CF>                  evmIn{std::span<const CF>{}};
        test::OutputSpan<gr::DataSet<float>> evmOut{std::span<gr::DataSet<float>>(room)};
        expect(evm.processBulk(evmIn, evmOut) == gr::work::Status::INSUFFICIENT_INPUT_ITEMS) << "EvmMeter";

        test::InputSpan<CF>                  snrIn{std::span<const CF>{}};
        test::OutputSpan<gr::DataSet<float>> snrOut{std::span<gr::DataSet<float>>(room)};
        expect(snr.processBulk(snrIn, snrOut) == gr::work::Status::INSUFFICIENT_INPUT_ITEMS) << "SnrEstimator";
    };
};

int main() { /* not needed for UT */ }
