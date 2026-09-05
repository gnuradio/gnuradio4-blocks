#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/digital/CpmPulse.hpp>

#include <gnuradio-4.0/algorithm/channel/NoisePower.hpp>
#include <gnuradio-4.0/analog/QuadratureDemod.hpp>
#include <gnuradio-4.0/channel/Awgn.hpp>
#include <gnuradio-4.0/digital/CpmModulate.hpp>
#include <gnuradio-4.0/digital/PamSymbols.hpp>
#include <gnuradio-4.0/filter/DesignedFilter.hpp>
#include <gnuradio-4.0/sync/SymbolSync.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace {

using gr::blocks::analog::QuadratureDemod;
using gr::blocks::channel::AwgnChannel;
using gr::blocks::digital::CpmModulate;
using gr::blocks::digital::PamSlicer;
using gr::blocks::filter::DesignedFilter;
using gr::blocks::sync::SymbolSync;
using gr::testing::ProcessFunction;
using gr::testing::TagSink;
using gr::testing::TagSource;

using CF = std::complex<float>;

constexpr double kPi = std::numbers::pi;

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    if constexpr (requires { block.start(); }) {
        block.start();
    }
    return block;
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

/// A deterministic symbol stream on the odd M-PAM grid the modulator reads.
[[nodiscard]] std::vector<float> pamStream(std::size_t count, std::size_t levels, std::uint64_t seed) {
    Rng                rng{seed};
    std::vector<float> symbols(count);
    for (float& symbol : symbols) {
        const std::size_t rank = rng.next() % levels;
        symbol                 = static_cast<float>(2.0 * static_cast<double>(rank) - static_cast<double>(levels - 1UZ));
    }
    return symbols;
}

/// @brief Drive the modulator over plain spans, `sps` outputs reserved per symbol.
///
/// @p chunks cycles the number of symbols each call is handed, so one symbol stream can be pushed through in any
/// partition; an empty list is the whole stream in one call.
[[nodiscard]] std::vector<CF> modulate(CpmModulate<float>& block, std::span<const float> symbols, std::size_t sps, std::span<const std::size_t> chunks = {}) {
    std::vector<CF> output(symbols.size() * sps);
    std::size_t     base  = 0UZ;
    std::size_t     which = 0UZ;
    while (base < symbols.size()) {
        const std::size_t asked = chunks.empty() ? symbols.size() : chunks[which % chunks.size()];
        const std::size_t count = std::min(std::max(asked, 1UZ), symbols.size() - base);
        std::ignore             = block.processBulk(symbols.subspan(base, count), std::span<CF>(output.data() + base * sps, count * sps));
        base += count;
        ++which;
    }
    return output;
}

/**
 * @brief The output the kernel and the phase accumulator jointly define, computed independently of the block.
 *
 * A bare `CpmPulse` produces the per-sample increments, and the phase is their running sum reduced to `[-pi, pi]`
 * after every step, which is what `gr::signal::Phasor::modulate` computes. The block starts from phase zero, so the
 * sum starts there too.
 */
[[nodiscard]] std::vector<CF> phaseOracle(std::span<const float> symbols, std::size_t sps, std::string_view shape, std::size_t symbolSpan, double h, double bt) {
    gr::digital::CpmPulse<float> pulse{};
    pulse.configure(gr::digital::cpmPulseShapeFrom(shape), symbolSpan, sps, h, bt);

    std::vector<double> increments(symbols.size() * sps);
    pulse.incrementsFor(symbols, std::span<double>(increments));

    constexpr double twoPi = 2.0 * kPi;
    std::vector<CF>  output(increments.size());
    double           phase = 0.0;
    for (std::size_t k = 0UZ; k < increments.size(); ++k) {
        output[k] = CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
        phase += increments[k];
        if (phase > kPi || phase < -kPi) {
            phase = std::remainder(phase, twoPi);
        }
    }
    return output;
}

struct Design {
    const char* pulse;
    std::size_t symbolSpan;
    double      modulationIndex;
    std::size_t samplesPerSymbol;
    std::size_t levels;
};

/// The three families, each at a full-response and a partial-response length.
constexpr Design kDesigns[] = {{"rect", 1UZ, 0.5, 8UZ, 2UZ}, {"rect", 3UZ, 0.7, 5UZ, 4UZ}, {"raised_cosine", 2UZ, 0.5, 8UZ, 4UZ}, {"raised_cosine", 4UZ, 0.25, 6UZ, 2UZ}, {"gaussian", 3UZ, 0.5, 8UZ, 2UZ}, {"gaussian", 5UZ, 0.32, 4UZ, 4UZ}};

[[nodiscard]] gr::property_map settingsOf(const Design& design) { return {{"modulation_index", design.modulationIndex}, {"samples_per_symbol", static_cast<gr::Size_t>(design.samplesPerSymbol)}, {"pulse", std::string(design.pulse)}, {"pulse_length", static_cast<gr::Size_t>(design.symbolSpan)}, {"bt", 0.3}}; }

/**
 * @brief The whole receive chain, from the symbol stream to one label per recovered symbol.
 *
 * @param channelBandwidth cutoff of the filter ahead of the discriminator, in symbol rates; zero hands the
 *        discriminator the whole sampled band, which is the shape the row was specified with
 */
[[nodiscard]] std::vector<std::uint8_t> receiveThroughNoise(std::span<const float> symbols, std::size_t sps, double h, double esN0_db, std::uint64_t seed, double channelBandwidth) {
    gr::Graph  graph;
    const auto values = gr::Tensor<float>(symbols.begin(), symbols.end());
    auto&      source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(symbols.size())}, {"values", values}, {"sample_rate", 1.0f}});

    auto& modulator = graph.emplaceBlock<CpmModulate<float>>({{"samples_per_symbol", static_cast<gr::Size_t>(sps)}, {"modulation_index", h}, {"pulse", std::string("rect")}, {"pulse_length", gr::Size_t(1)}});
    auto& channel   = graph.emplaceBlock<AwgnChannel<CF>>({{"noise_power", static_cast<double>(gr::channel::noisePowerFor(static_cast<float>(esN0_db), 1.0f, static_cast<float>(sps)))}, {"seed", seed}});
    auto& demod     = graph.emplaceBlock<QuadratureDemod<float>>({{"gain", static_cast<float>(static_cast<double>(sps) / (kPi * h))}});
    auto& lowpass   = graph.emplaceBlock<DesignedFilter<float, float>>({{"profile", std::string("lowpass")}, {"sample_rate", static_cast<float>(sps)}, {"cutoff", 0.5}, {"transition_width", 0.5}});
    auto& timing    = graph.emplaceBlock<SymbolSync<float>>({{"samples_per_symbol", static_cast<double>(sps)}, {"detector", std::string("MuellerMuller")}, {"constellation", std::string("bpsk")}, {"interpolator", std::string("mmse8")}, {"noise_bandwidth", 0.002}});
    auto& slicer    = graph.emplaceBlock<PamSlicer<float>>({{"n_levels", gr::Size_t(2)}});
    auto& sink      = graph.emplaceBlock<TagSink<std::uint8_t, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

    boost::ut::expect(graph.connect<"out", "in">(source, modulator).has_value());
    boost::ut::expect(graph.connect<"out", "in">(modulator, channel).has_value());
    if (channelBandwidth > 0.) { // the band the discriminator sees is the band its noise comes from
        auto& front = graph.emplaceBlock<DesignedFilter<CF, float>>({{"profile", std::string("lowpass")}, {"sample_rate", static_cast<float>(sps)}, {"cutoff", channelBandwidth}, {"transition_width", 0.5 * channelBandwidth}});
        boost::ut::expect(graph.connect<"out", "in">(channel, front).has_value());
        boost::ut::expect(graph.connect<"out", "in">(front, demod).has_value());
    } else {
        boost::ut::expect(graph.connect<"out", "in">(channel, demod).has_value());
    }
    boost::ut::expect(graph.connect<"out", "in">(demod, lowpass).has_value());
    boost::ut::expect(graph.connect<"out", "in">(lowpass, timing).has_value());
    boost::ut::expect(graph.connect<"out", "in">(timing, slicer).has_value());
    boost::ut::expect(graph.connect<"out", "in">(slicer, sink).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    const auto run = scheduler.runAndWait();
    boost::ut::expect(run.has_value()) << (run.has_value() ? std::string{} : run.error().message);
    return {sink._samples.begin(), sink._samples.end()};
}

/**
 * @brief The bit error rate of @p decided against @p symbols, at the lag that minimizes it.
 *
 * The chain's total delay is the filter's group delay plus whatever offset the timing loop settled on, so the lag is
 * measured rather than derived; @p settle drops the symbols the loop spends acquiring.
 */
[[nodiscard]] std::pair<double, std::ptrdiff_t> bitErrorRate(std::span<const float> symbols, std::span<const std::uint8_t> decided, std::size_t settle) {
    double         best = 1.0;
    std::ptrdiff_t at   = 0;
    for (std::ptrdiff_t lag = -32; lag <= 32; ++lag) {
        std::size_t errors  = 0UZ;
        std::size_t counted = 0UZ;
        for (std::size_t k = settle; k < decided.size(); ++k) {
            const std::ptrdiff_t index = static_cast<std::ptrdiff_t>(k) + lag;
            if (index < 0 || index >= static_cast<std::ptrdiff_t>(symbols.size())) {
                continue;
            }
            const std::uint8_t wanted = symbols[static_cast<std::size_t>(index)] > 0.f ? 1U : 0U;
            errors += decided[k] != wanted ? 1UZ : 0UZ;
            ++counted;
        }
        if (counted < decided.size() / 2UZ) {
            continue;
        }
        const double rate = static_cast<double>(errors) / static_cast<double>(counted);
        if (rate < best) {
            best = rate;
            at   = lag;
        }
    }
    return {best, at};
}

} // namespace

const boost::ut::suite<"CpmModulate"> cpmModulateTests = [] {
    using namespace boost::ut;

    "the output is the kernel's increments accumulated, sample for sample"_test = [] {
        for (const Design& design : kDesigns) {
            const std::vector<float> symbols = pamStream(4000UZ, design.levels, 0xfeedfaceULL);

            CpmModulate<float>    block  = make<CpmModulate<float>>(settingsOf(design));
            const std::vector<CF> got    = modulate(block, symbols, design.samplesPerSymbol);
            const std::vector<CF> wanted = phaseOracle(symbols, design.samplesPerSymbol, design.pulse, design.symbolSpan, design.modulationIndex, 0.3);

            std::size_t mismatches = 0UZ;
            std::string first;
            for (std::size_t k = 0UZ; k < got.size(); ++k) {
                if (got[k] == wanted[k]) {
                    continue;
                }
                ++mismatches;
                if (first.empty()) {
                    first = std::format(" first at {}: ({:a}, {:a}) against ({:a}, {:a})", k, got[k].real(), got[k].imag(), wanted[k].real(), wanted[k].imag());
                }
            }
            expect(eq(mismatches, 0UZ)) << std::format("{} L = {}, h = {}, sps = {}: bit-identical to the kernel, not approximate;{}", design.pulse, design.symbolSpan, design.modulationIndex, design.samplesPerSymbol, first);
        }
    };

    "the envelope holds at the stated amplitude"_test = [] {
        // cos^2 + sin^2 is not exactly one in float, so the envelope is flat to the rounding of the two components,
        // one part in 2^24, and the tolerance sits just above that floor.
        constexpr double kTolerance = 1e-7;

        for (const double amplitude : {1.0, 0.5}) {
            for (const char* shape : {"rect", "raised_cosine", "gaussian"}) {
                const std::vector<float> symbols = pamStream(20000UZ, 4UZ, 0x2545F4914F6CDD1DULL);

                CpmModulate<float>    block  = make<CpmModulate<float>>({{"modulation_index", 0.7}, {"samples_per_symbol", gr::Size_t(8)}, {"pulse", std::string(shape)}, {"pulse_length", gr::Size_t(3)}, {"amplitude", static_cast<float>(amplitude)}});
                const std::vector<CF> output = modulate(block, symbols, 8UZ);

                double worst = 0.0;
                for (const CF& sample : output) {
                    worst = std::max(worst, std::abs(static_cast<double>(std::abs(sample)) / amplitude - 1.0));
                }
                std::println("CpmModulate envelope at amplitude {} ({}): worst relative deviation {:.3e}", amplitude, shape, worst);
                expect(lt(worst, kTolerance)) << std::format("amplitude {} ({}): worst relative deviation {:.3e}", amplitude, shape, worst);
            }
        }
    };

    "a discriminator scaled by sps/(pi*h) hands back the transmitted grid"_test = [] {
        constexpr std::size_t kSps       = 8UZ;
        constexpr double      kTolerance = 1e-6;

        for (const std::size_t levels : {2UZ, 4UZ}) {
            for (const double h : {0.5, 0.7}) {
                const std::vector<float> symbols = pamStream(2000UZ, levels, 0x123456789abcdefULL);

                CpmModulate<float>    modulator = make<CpmModulate<float>>({{"modulation_index", h}, {"samples_per_symbol", static_cast<gr::Size_t>(kSps)}, {"pulse", std::string("rect")}, {"pulse_length", gr::Size_t(1)}});
                const std::vector<CF> modulated = modulate(modulator, symbols, kSps);

                QuadratureDemod<float> demod = make<QuadratureDemod<float>>({{"gain", static_cast<float>(static_cast<double>(kSps) / (kPi * h))}});
                std::vector<float>     discriminated(modulated.size());
                std::ignore = demod.processBulk(std::span<const CF>(modulated), std::span<float>(discriminated));

                // The alignment is measured rather than derived. A differential discriminator reports the phase
                // advance between samples n-1 and n at index n, so the window carrying symbol s sits at some offset
                // from s*sps; the sweep below finds which offset that is.
                std::vector<double>      deviations(kSps + 1UZ, 0.0);
                std::vector<std::size_t> aligned;
                for (std::size_t offset = 0UZ; offset <= kSps; ++offset) {
                    for (std::size_t s = 1UZ; s + 1UZ < symbols.size(); ++s) {
                        double mean = 0.0;
                        for (std::size_t k = 0UZ; k < kSps; ++k) {
                            mean += static_cast<double>(discriminated[s * kSps + offset + k]);
                        }
                        deviations[offset] = std::max(deviations[offset], std::abs(mean / static_cast<double>(kSps) - static_cast<double>(symbols[s])));
                    }
                    if (deviations[offset] < kTolerance) {
                        aligned.push_back(offset);
                    }
                }

                const std::size_t best = static_cast<std::size_t>(std::ranges::distance(deviations.begin(), std::ranges::min_element(deviations)));
                std::println("CpmModulate discriminator M = {}, h = {}: symbol means match at offset {}, worst error there {:.2e}, at the neighboring offset {:.2e}", levels, h, best, deviations[best], deviations[best == 0UZ ? 1UZ : best - 1UZ]);
                expect(that % (aligned == std::vector<std::size_t>{1UZ})) << std::format("M = {}, h = {}: one window reproduces the grid and it is one sample past the symbol boundary; deviations {}", levels, h, deviations);
            }
        }
    };

    "the output does not depend on how the symbol stream is split"_test = [] {
        constexpr std::size_t kFirst[]  = {1UZ, 7UZ, 3UZ, 64UZ, 2UZ};
        constexpr std::size_t kSecond[] = {997UZ, 5UZ, 1UZ, 13UZ};

        for (const char* shape : {"rect", "raised_cosine", "gaussian"}) {
            const std::vector<float> symbols = pamStream(3000UZ, 4UZ, 0xdeadbeefULL);
            const gr::property_map   settings{{"modulation_index", 0.6}, {"samples_per_symbol", gr::Size_t(6)}, {"pulse", std::string(shape)}, {"pulse_length", gr::Size_t(4)}, {"bt", 0.5}};

            CpmModulate<float>    whole  = make<CpmModulate<float>>(settings);
            const std::vector<CF> wanted = modulate(whole, symbols, 6UZ);

            CpmModulate<float> first  = make<CpmModulate<float>>(settings);
            CpmModulate<float> second = make<CpmModulate<float>>(settings);
            expect(that % (modulate(first, symbols, 6UZ, std::span<const std::size_t>(kFirst)) == wanted)) << std::format("{}: the first partition", shape);
            expect(that % (modulate(second, symbols, 6UZ, std::span<const std::size_t>(kSecond)) == wanted)) << std::format("{}: the second partition", shape);
        }
    };

    "one symbol becomes samples_per_symbol outputs, and a tag moves with them"_test = [] {
        constexpr gr::Size_t  kSymbols = 200U;
        constexpr std::size_t kSps     = 4UZ;
        constexpr float       kRate    = 1000.0f;
        constexpr std::size_t kAt[]    = {3UZ, 17UZ, 64UZ};

        gr::Graph  graph;
        const auto values = gr::Tensor<float>(static_cast<std::size_t>(kSymbols), 1.0f);
        auto&      source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", kSymbols}, {"values", values}, {"sample_rate", kRate}});
        for (const std::size_t at : kAt) {
            source._tags.emplace_back(at, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("symbol")}});
        }

        auto& modulator = graph.emplaceBlock<CpmModulate<float>>({{"samples_per_symbol", static_cast<gr::Size_t>(kSps)}, {"modulation_index", 0.5}, {"pulse", std::string("rect")}});
        auto& sink      = graph.emplaceBlock<TagSink<CF, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

        expect(graph.connect<"out", "in">(source, modulator).has_value());
        expect(graph.connect<"out", "in">(modulator, sink).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        expect(eq(sink._samples.size(), static_cast<std::size_t>(kSymbols) * kSps)) << "the resampling ratio is the setting, over the whole stream";

        std::vector<std::size_t> offsets;
        std::vector<float>       rates;
        for (const gr::Tag& seenTag : sink._tags) {
            if (seenTag.map.contains(gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()))) {
                offsets.push_back(seenTag.index);
            }
            if (const auto found = seenTag.map.find(gr::property_map::key_type(gr::tag::SAMPLE_RATE.shortKey())); found != seenTag.map.end()) {
                if (const float* rate = found->second.get_if<float>(); rate != nullptr) {
                    rates.push_back(*rate);
                }
            }
        }

        expect(that % (offsets == std::vector<std::size_t>{kSps * kAt[0], kSps * kAt[1], kSps * kAt[2]})) << "a tag at input symbol t arrives at output sps*t";
        expect(eq(rates.size(), 1UZ)) << "the stream carries one rate";
        expect(that % (rates == std::vector<float>{kRate * static_cast<float>(kSps)})) << "the rate downstream is the rate the block publishes at";
    };

    "settings outside the kernel's range are refused"_test = [] {
        // Every case builds its own block: a settings change that throws leaves the offending value staged, so the
        // same block would refuse the next attempt whatever it asked for.
        expect(throws([] { std::ignore = make<CpmModulate<float>>({{"samples_per_symbol", gr::Size_t(1)}}); })) << "two samples is the fewest a phase trajectory can be drawn on";
        expect(throws([] { std::ignore = make<CpmModulate<float>>({{"pulse_length", gr::Size_t(0)}}); })) << "a pulse spans at least one symbol";
        expect(throws([] { std::ignore = make<CpmModulate<float>>({{"pulse_length", gr::Size_t(9)}}); })) << "and at most eight";
        expect(throws([] { std::ignore = make<CpmModulate<float>>({{"modulation_index", 0.0}}); })) << "a zero index turns the carrier nowhere";
        expect(throws([] { std::ignore = make<CpmModulate<float>>({{"modulation_index", 5.0}}); })) << "four rotations per symbol is the ceiling";
        expect(throws([] { std::ignore = make<CpmModulate<float>>({{"pulse", std::string("gmsk")}}); })) << "the pulse names three families";
        expect(throws([] { std::ignore = make<CpmModulate<float>>({{"pulse", std::string("gaussian")}, {"bt", 0.0}}); })) << "the gaussian pulse reads bt and needs it positive";

        // The refusals leave nothing behind: a block built afterwards runs.
        CpmModulate<float>    block  = make<CpmModulate<float>>({{"samples_per_symbol", gr::Size_t(4)}, {"modulation_index", 0.5}});
        const std::vector<CF> output = modulate(block, pamStream(64UZ, 2UZ, 0xabcdefULL), 4UZ);
        expect(eq(output.size(), 256UZ));
        expect(approx(static_cast<double>(std::abs(output.back())), 1.0, 1e-6));
    };

    "a discriminator receiver's bit error rate against the noncoherent reference"_test = [] {
        constexpr std::size_t kSps     = 8UZ;
        constexpr double      kH       = 0.5;
        constexpr std::size_t kSymbols = 40000UZ;
        constexpr std::size_t kSettle  = 200UZ;

        struct Point {
            double esN0_db;
            double cap; ///< the bit error rate this operating point must stay under
        };
        // The caps sit roughly a factor of two above what the chain measures, which is the width a change in the
        // modulator, the discriminator or the timing loop would have to exceed to be caught here.
        constexpr Point kPoints[] = {{8.0, 0.30}, {10.0, 0.16}, {12.0, 0.055}, {14.0, 0.011}};

        const std::vector<float> symbols = pamStream(kSymbols, 2UZ, 0x243F6A8885A308D3ULL);

        double previous = 1.0;
        for (const Point& point : kPoints) {
            const std::vector<std::uint8_t> decided = receiveThroughNoise(symbols, kSps, kH, point.esN0_db, 0xC0FFEEULL, 0.);
            const auto [rate, lag]                  = bitErrorRate(symbols, decided, kSettle);

            // The transmitted grid carries one bit per symbol, so Es/N0 and Eb/N0 are the same number here.
            const double reference = 0.5 * std::exp(-std::pow(10.0, point.esN0_db / 10.0) / 2.0);
            std::println("CpmModulate 2-FSK at Es/N0 {:4.1f} dB: {} symbols recovered, BER {:.3e} at lag {}, noncoherent reference {:.3e}, ratio {:.1f}", point.esN0_db, decided.size(), rate, lag, reference, rate / reference);

            const double recovered = static_cast<double>(decided.size()) / static_cast<double>(kSymbols);
            expect(lt(std::abs(recovered - 1.0), 0.02)) << std::format("Es/N0 {} dB: the timing loop returns a symbol per transmitted symbol, {} of them", point.esN0_db, decided.size());
            expect(lt(rate, point.cap)) << std::format("Es/N0 {} dB: BER {:.3e} against a cap of {}", point.esN0_db, rate, point.cap);
            expect(gt(rate, reference)) << std::format("Es/N0 {} dB: a discriminator does not beat the noncoherent bound", point.esN0_db);
            expect(lt(rate, previous)) << std::format("Es/N0 {} dB: the rate falls as the noise does", point.esN0_db);
            previous = rate;
        }
    };
    "a channel filter puts the same receiver at the noncoherent bound"_test = [] {
        constexpr std::size_t kSps              = 8UZ;
        constexpr double      kH                = 0.5;
        constexpr std::size_t kSymbols          = 40000UZ;
        constexpr std::size_t kSettle           = 200UZ;
        constexpr double      kChannelBandwidth = 0.6; ///< symbol rates, the cutoff the packaged demod recipe carries

        // A discriminator's noise is the noise in the band it is handed. Unfiltered at eight samples per symbol that
        // band is eight times the symbol rate, which puts every operating point below the click threshold and gives
        // the curve a different exponent from the reference rather than a fixed distance from it. The same chain
        // behind a channel filter sits at or under the noncoherent bound.
        constexpr double kEsN0[] = {6.0, 8.0, 10.0, 12.0};

        const std::vector<float> symbols = pamStream(kSymbols, 2UZ, 0x243F6A8885A308D3ULL);

        double previous = 1.0;
        for (const double esN0_db : kEsN0) {
            const std::vector<std::uint8_t> decided = receiveThroughNoise(symbols, kSps, kH, esN0_db, 0xC0FFEEULL, kChannelBandwidth);
            const auto [rate, lag]                  = bitErrorRate(symbols, decided, kSettle);
            const double reference                  = 0.5 * std::exp(-std::pow(10.0, esN0_db / 10.0) / 2.0);
            std::println("CpmModulate 2-FSK behind a channel filter at Es/N0 {:4.1f} dB: BER {:.3e} at lag {}, noncoherent reference {:.3e}, ratio {:.2f}", esN0_db, rate, lag, reference, rate / reference);

            expect(lt(rate, 2.0 * reference)) << std::format("Es/N0 {} dB: BER {:.3e} against twice the noncoherent reference, {:.3e}", esN0_db, rate, 2.0 * reference);
            expect(lt(rate, previous)) << std::format("Es/N0 {} dB: the rate falls as the noise does", esN0_db);
            previous = rate;
        }
    };
};

int main() { /* not needed for UT */ }
