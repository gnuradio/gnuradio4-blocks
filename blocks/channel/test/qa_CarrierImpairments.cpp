#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <format>
#include <numbers>
#include <print>
#include <span>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/channel/CarrierImpairments.hpp>
#include <gnuradio-4.0/sync/LoopCommon.hpp>
#include <gnuradio-4.0/sync/Pll.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include "EstimateKeys.hpp"

namespace {

using C                 = std::complex<float>;
constexpr double kTwoPi = 2. * std::numbers::pi_v<double>;

template<typename Block>
[[nodiscard]] Block configured(gr::property_map settings) {
    Block block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

[[nodiscard]] std::vector<C> tone(std::size_t n, double cyclesPerSample) {
    std::vector<C> x(n);
    for (std::size_t k = 0UZ; k < n; ++k) {
        const double phase = kTwoPi * cyclesPerSample * static_cast<double>(k);
        x[k]               = C(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
    }
    return x;
}

/// Mean frequency, in cycles per sample, from the average phase step across a span.
[[nodiscard]] double meanFrequency(std::span<const C> x) {
    std::complex<double> accumulated{0., 0.};
    for (std::size_t k = 1UZ; k < x.size(); ++k) {
        accumulated += std::complex<double>(x[k]) * std::conj(std::complex<double>(x[k - 1UZ]));
    }
    return std::arg(accumulated) / kTwoPi;
}

template<typename Block>
[[nodiscard]] std::vector<C> runChunked(Block& block, std::span<const C> input, std::size_t chunk) {
    std::vector<C> out(input.size());
    for (std::size_t i = 0UZ; i < input.size(); i += chunk) {
        const std::size_t n = std::min(chunk, input.size() - i);
        std::ignore         = block.processBulk(input.subspan(i, n), std::span<C>(out.data() + i, n));
    }
    return out;
}

/// the carrier loop's estimates and output for one run of source -> TMiddle -> PllCarrierTracking
struct LoopRun {
    std::vector<float> frequency; ///< rad/sample, after each sample's update
    std::vector<float> phase;     ///< rad, in force on each sample
    std::vector<C>     derotated;
};

template<typename TMiddle>
[[nodiscard]] LoopRun runIntoLoop(std::span<const C> input, std::vector<gr::Tag> tags, gr::property_map middleSettings = {}) {
    using gr::testing::ProcessFunction;

    gr::Graph graph;
    auto&     source = graph.emplaceBlock<gr::testing::TagSource<C, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(input.size())}, {"values", gr::Tensor<C>(input.begin(), input.end())}, {"sample_rate", 1.f}, {"mark_tag", false}});
    source._tags     = std::move(tags);

    auto& middle        = graph.emplaceBlock<TMiddle>(std::move(middleSettings));
    auto& loop          = graph.emplaceBlock<gr::blocks::sync::PllCarrierTracking>({});
    auto& outSink       = graph.emplaceBlock<gr::testing::TagSink<C, ProcessFunction::USE_PROCESS_BULK>>({});
    auto& frequencySink = graph.emplaceBlock<gr::testing::TagSink<float, ProcessFunction::USE_PROCESS_BULK>>({});
    auto& phaseSink     = graph.emplaceBlock<gr::testing::TagSink<float, ProcessFunction::USE_PROCESS_BULK>>({});
    boost::ut::expect(graph.connect<"out", "in">(source, middle).has_value());
    boost::ut::expect(graph.connect<"out", "in">(middle, loop).has_value());
    boost::ut::expect(graph.connect<"out", "in">(loop, outSink).has_value());
    boost::ut::expect(graph.connect<"freq", "in">(loop, frequencySink).has_value());
    boost::ut::expect(graph.connect<"phase", "in">(loop, phaseSink).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value());
    return {std::vector<float>(frequencySink._samples.begin(), frequencySink._samples.end()), std::vector<float>(phaseSink._samples.begin(), phaseSink._samples.end()), std::vector<C>(outSink._samples.begin(), outSink._samples.end())};
}

[[nodiscard]] float meanOf(std::span<const float> values) {
    double sum = 0.;
    for (const float value : values) {
        sum += static_cast<double>(value);
    }
    return values.empty() ? 0.f : static_cast<float>(sum / static_cast<double>(values.size()));
}

} // namespace

const boost::ut::suite<"carrier impairments"> carrierTests = [] {
    using namespace boost::ut;
    using gr::blocks::channel::FrequencyOffset;
    using gr::blocks::channel::IqImbalance;
    using gr::blocks::channel::PhaseNoise;

    // the offset lands where it was asked to
    "a frequency offset moves the tone by exactly that much"_test = [] {
        constexpr std::size_t nSamples = 65536UZ;
        constexpr double      fs       = 1.0e6;
        const auto            input    = tone(nSamples, 0.01);

        for (const double offset : {-137000., -1000., 0., 250., 12345.}) {
            auto         block = configured<FrequencyOffset<C>>({{"sample_rate", static_cast<float>(fs)}, {"frequency_offset", offset}});
            const auto   out   = runChunked(block, std::span<const C>(input), nSamples);
            const double want  = 0.01 + offset / fs;
            expect(lt(std::abs(meanFrequency(out) - want), 1e-9)) << std::format("offset {} Hz: measured {:g} cyc/sample, wanted {:g}", offset, meanFrequency(out), want);
        }
    };

    // the drift is the frequency's own slope, measured across two disjoint windows
    "drift moves the frequency by rate times the elapsed time"_test = [] {
        constexpr std::size_t nSamples = 200'000UZ;
        constexpr double      fs       = 1.0e6;
        // the excursion must stay well inside +-fs/2: the phase-step estimator wraps beyond Nyquist, and at
        // 5 MHz/s this window would sweep 900 kHz and alias
        constexpr double     rate = 1.0e5; // Hz/s
        const std::vector<C> input(nSamples, C(1.f, 0.f));

        auto       block = configured<FrequencyOffset<C>>({{"sample_rate", static_cast<float>(fs)}, {"frequency_offset", 0.0}, {"drift", rate}});
        const auto out   = runChunked(block, std::span<const C>(input), nSamples);

        constexpr std::size_t window = 20'000UZ;
        const double          early  = meanFrequency(std::span<const C>(out).subspan(0UZ, window)) * fs;
        const double          late   = meanFrequency(std::span<const C>(out).subspan(nSamples - window, window)) * fs;
        const double          dt     = static_cast<double>(nSamples - window) / fs;
        expect(lt(std::abs((late - early) / (rate * dt) - 1.), 0.001)) << std::format("measured {:g} Hz of drift over {:g} s, expected {:g}", late - early, dt, rate * dt);
    };

    // splitting must be bit-identical on both the fixed and the drifting path
    "frequency offset is bit-identical however the stream is split"_test = [] {
        constexpr std::size_t nSamples = 20'000UZ;
        const auto            input    = tone(nSamples, 0.013);

        for (const double drift : {0.0, 5.0e6}) {
            const gr::property_map settings{{"sample_rate", 1.0e6f}, {"frequency_offset", 1234.0}, {"drift", drift}};
            auto                   reference = configured<FrequencyOffset<C>>(settings);
            const auto             want      = runChunked(reference, std::span<const C>(input), nSamples);
            for (const std::size_t chunk : {1UZ, 3UZ, 64UZ, 4095UZ}) {
                auto block = configured<FrequencyOffset<C>>(settings);
                expect(runChunked(block, std::span<const C>(input), chunk) == want) << std::format("drift {}, {}-sample calls", drift, chunk);
            }
        }
    };

    "a zero offset with no drift is a pure passthrough of the tone"_test = [] {
        const auto input = tone(4096UZ, 0.017);
        auto       block = configured<FrequencyOffset<C>>({{"sample_rate", 1.0e6f}, {"frequency_offset", 0.0}});
        const auto out   = runChunked(block, std::span<const C>(input), 4096UZ);
        for (std::size_t k = 0UZ; k < input.size(); ++k) {
            expect(lt(std::abs(std::complex<double>(out[k]) - std::complex<double>(input[k])), 1e-6)) << std::format("at {}", k);
        }
    };

    "the sample rate must be positive"_test = [] {
        expect(throws([] { std::ignore = configured<FrequencyOffset<C>>({{"sample_rate", 0.f}}); }));
        expect(throws([] { std::ignore = configured<PhaseNoise<C>>({{"sample_rate", -1.f}}); }));
    };

    // the walk's step variance is what the stated linewidth means
    "phase-noise step variance matches 2 pi linewidth / fs"_test = [] {
        constexpr std::size_t nSamples  = 1'000'000UZ;
        constexpr double      fs        = 1.0e6;
        constexpr double      linewidth = 1000.0;
        const std::vector<C>  input(nSamples, C(1.f, 0.f));

        auto       block = configured<PhaseNoise<C>>({{"sample_rate", static_cast<float>(fs)}, {"linewidth", linewidth}, {"seed", std::uint64_t(5)}});
        const auto out   = runChunked(block, std::span<const C>(input), nSamples);

        // with a constant input the output phase IS the walk, so its increments are the steps
        double sumSquared = 0.;
        for (std::size_t k = 1UZ; k < nSamples; ++k) {
            const auto step = std::complex<double>(out[k]) * std::conj(std::complex<double>(out[k - 1UZ]));
            sumSquared += std::arg(step) * std::arg(step);
        }
        const double measured = sumSquared / static_cast<double>(nSamples - 1UZ);
        const double expected = kTwoPi * linewidth / fs;
        std::println("PhaseNoise step variance: measured {:g}, expected {:g}", measured, expected);
        expect(lt(std::abs(measured / expected - 1.), 0.02)) << std::format("measured {:g}, expected {:g}", measured, expected);
    };

    // a rotation cannot change magnitude
    "phase noise preserves power exactly"_test = [] {
        constexpr std::size_t nSamples = 100'000UZ;
        const std::vector<C>  input(nSamples, C(0.75f, -0.25f));
        auto                  block = configured<PhaseNoise<C>>({{"sample_rate", 1.0e6f}, {"linewidth", 5000.0}, {"seed", std::uint64_t(2)}});
        const auto            out   = runChunked(block, std::span<const C>(input), nSamples);

        const double inputPower = static_cast<double>(std::norm(input[0]));
        double       worst      = 0.;
        for (const C v : out) {
            worst = std::max(worst, std::abs(static_cast<double>(std::norm(v)) / inputPower - 1.));
        }
        expect(lt(10. * std::log10(1. + worst), 0.01)) << std::format("worst power deviation {:g} dB", 10. * std::log10(1. + worst));
    };

    "phase noise is bit-identical however the stream is split"_test = [] {
        constexpr std::size_t  nSamples = 20'000UZ;
        const auto             input    = tone(nSamples, 0.011);
        const gr::property_map settings{{"sample_rate", 1.0e6f}, {"linewidth", 2000.0}, {"seed", std::uint64_t(7)}};

        auto       reference = configured<PhaseNoise<C>>(settings);
        const auto want      = runChunked(reference, std::span<const C>(input), nSamples);
        for (const std::size_t chunk : {1UZ, 2UZ, 3UZ, 7UZ, 4095UZ}) {
            auto block = configured<PhaseNoise<C>>(settings);
            expect(runChunked(block, std::span<const C>(input), chunk) == want) << std::format("{}-sample calls", chunk);
        }
    };

    // a disabled impairment must copy rather than rotate by zero
    "linewidth 0 is bit-exact passthrough"_test = [] {
        const auto input = tone(1024UZ, 0.021);
        auto       block = configured<PhaseNoise<C>>({{"sample_rate", 1.0e6f}, {"linewidth", 0.0}});
        expect(runChunked(block, std::span<const C>(input), 1024UZ) == input) << "a disabled impairment must copy";
    };

    // the measured image matches the closed-form image rejection
    "the image power matches the closed-form image rejection"_test = [] {
        constexpr std::size_t nSamples = 65536UZ;
        constexpr double      cycles   = 0.05;
        const auto            input    = tone(nSamples, cycles);

        for (const auto [gainDb, phase] : std::vector<std::pair<double, double>>{{0.0, 0.0}, {0.5, 0.0}, {0.0, 0.02}, {1.0, 0.05}, {-0.7, -0.03}}) {
            auto block = configured<IqImbalance<C>>({{"amplitude_imbalance_db", gainDb}, {"phase_imbalance", phase}});
            if (gainDb == 0.0 && phase == 0.0) {
                std::vector<C> out(nSamples);
                for (std::size_t k = 0UZ; k < nSamples; ++k) {
                    out[k] = block.processOne(input[k]);
                }
                expect(out == input) << "the identity must be bit-exact passthrough";
                continue;
            }

            // measure the wanted and image lines by correlating against each
            std::complex<double> wanted{0., 0.}, image{0., 0.};
            for (std::size_t k = 0UZ; k < nSamples; ++k) {
                const C                    v     = block.processOne(input[k]);
                const double               angle = kTwoPi * cycles * static_cast<double>(k);
                const std::complex<double> sample(v);
                wanted += sample * std::polar(1., -angle);
                image += sample * std::polar(1., angle);
            }
            const double measured = 10. * std::log10(std::norm(wanted) / std::norm(image));
            expect(lt(std::abs(measured - block.imageRejectionDb()), 0.1)) //
                << std::format("gain {} dB, phase {} rad: measured IRR {:.2f} dB, closed form {:.2f} dB", gainDb, phase, measured, block.imageRejectionDb());
        }
    };

    "IQ imbalance is stateless, so any split is identical"_test = [] {
        const auto     input = tone(4096UZ, 0.03);
        auto           block = configured<IqImbalance<C>>({{"amplitude_imbalance_db", 0.8}, {"phase_imbalance", 0.04}});
        std::vector<C> out(input.size());
        for (std::size_t k = 0UZ; k < input.size(); ++k) {
            out[k] = block.processOne(input[k]);
        }
        auto           second = configured<IqImbalance<C>>({{"amplitude_imbalance_db", 0.8}, {"phase_imbalance", 0.04}});
        std::vector<C> again(input.size());
        for (std::size_t k = input.size(); k-- > 0UZ;) { // deliberately out of order: nothing may carry
            again[k] = second.processOne(input[k]);
        }
        expect(out == again) << "a stateless map must not depend on sample order";
    };

    "each impairment drops the estimate keys it invalidates and forwards an unrelated key"_test = [] {
        using gr::blocks::channel::test::keysCrossing;
        using gr::blocks::channel::test::keysOtherThan;
        using gr::blocks::sync::kFreqEstKey;
        using gr::blocks::sync::kPhaseEstKey;

        static_assert(std::ranges::equal(FrequencyOffset<C>::DroppedTagKeysControl::kKeys, std::array{kFreqEstKey, kPhaseEstKey}), "the dropped keys are the loop family's own");
        static_assert(std::ranges::equal(PhaseNoise<C>::DroppedTagKeysControl::kKeys, std::array{kPhaseEstKey}));
        static_assert(std::ranges::equal(IqImbalance<C>::DroppedTagKeysControl::kKeys, std::array{kPhaseEstKey}));
        static_assert(gr::block::kUnfilteredTagPropagationAdmissible<FrequencyOffset<C>> && gr::block::kUnfilteredTagPropagationAdmissible<PhaseNoise<C>> && gr::block::kUnfilteredTagPropagationAdmissible<IqImbalance<C>>);

        expect(keysCrossing<gr::testing::Copy<C>>() == keysOtherThan({})) << "every test key crosses a block that drops none";
        expect(keysCrossing<FrequencyOffset<C>>({{"frequency_offset", 0.01}}) == keysOtherThan({kFreqEstKey, kPhaseEstKey})) << "FrequencyOffset";
        expect(keysCrossing<PhaseNoise<C>>({{"linewidth", 0.001}}) == keysOtherThan({kPhaseEstKey})) << "PhaseNoise";
        expect(keysCrossing<IqImbalance<C>>({{"phase_imbalance", 0.04}}) == keysOtherThan({kPhaseEstKey})) << "IqImbalance";
    };

    "a stale freq_est stops at a frequency offset, and the loop behind it acquires on its own"_test = [] {
        constexpr std::size_t      kSamples = 4096UZ;
        constexpr double           kInput   = 0.002; // cycles/sample
        constexpr double           kOffset  = 0.003; // cycles/sample, at sample_rate 1
        const float                stale    = static_cast<float>(kTwoPi * kInput);
        const float                truth    = static_cast<float>(kTwoPi * (kInput + kOffset));
        const auto                 input    = tone(kSamples, kInput);
        const auto                 shifted  = tone(kSamples, kInput + kOffset);
        const std::vector<gr::Tag> estimate{gr::Tag{0UZ, gr::property_map{{gr::property_map::key_type{gr::blocks::sync::kFreqEstKey}, stale}}}};

        const LoopRun relayed = runIntoLoop<gr::testing::Copy<C>>(std::span<const C>(shifted), estimate);
        expect(eq(relayed.frequency.size(), kSamples));
        expect(approx(relayed.frequency[0UZ], stale, 1e-3f)) << "a block that forwards the key steers the loop to the stale estimate";

        const gr::property_map offsetSettings{{"frequency_offset", kOffset}};
        const LoopRun          tagged   = runIntoLoop<FrequencyOffset<C>>(std::span<const C>(input), estimate, offsetSettings);
        const LoopRun          untagged = runIntoLoop<FrequencyOffset<C>>(std::span<const C>(input), {}, offsetSettings);
        expect(eq(tagged.frequency.size(), kSamples));
        expect(gt(std::abs(tagged.frequency[0UZ] - stale), 0.5f * std::abs(truth - stale))) << "the loop starts away from the stale estimate";
        expect(tagged.frequency == untagged.frequency && tagged.derotated == untagged.derotated) << "the loop runs as it does with no tag at all";
        expect(approx(meanOf(std::span<const float>(tagged.frequency).last(512UZ)), truth, 1e-3f)) << "and settles on the shifted carrier";
    };

    "a stale phase_est stops at phase noise, and the loop behind it acquires on its own"_test = [] {
        constexpr std::size_t      kSamples = 4096UZ;
        constexpr float            kStale   = 2.f; // rad, far from the carrier's phase
        const auto                 input    = tone(kSamples, 0.);
        const std::vector<gr::Tag> estimate{gr::Tag{16UZ, gr::property_map{{gr::property_map::key_type{gr::blocks::sync::kPhaseEstKey}, kStale}}}};

        const LoopRun relayed = runIntoLoop<gr::testing::Copy<C>>(std::span<const C>(input), estimate);
        expect(eq(relayed.phase.size(), kSamples));
        expect(approx(relayed.phase[16UZ], kStale, 1e-6f)) << "a block that forwards the key sets the loop phase to the stale estimate";

        const gr::property_map noiseSettings{{"linewidth", 1e-4}, {"seed", std::uint64_t{7U}}};
        const LoopRun          tagged   = runIntoLoop<PhaseNoise<C>>(std::span<const C>(input), estimate, noiseSettings);
        const LoopRun          untagged = runIntoLoop<PhaseNoise<C>>(std::span<const C>(input), {}, noiseSettings);
        expect(eq(tagged.phase.size(), kSamples));
        expect(gt(std::abs(tagged.phase[16UZ] - kStale), 1.f)) << "the loop phase stays away from the stale estimate";
        expect(tagged.phase == untagged.phase && tagged.derotated == untagged.derotated) << "the loop runs as it does with no tag at all";
        float worst = 0.f;
        for (const C& sample : std::span<const C>(tagged.derotated).last(512UZ)) {
            worst = std::max(worst, std::abs(std::arg(sample)));
        }
        expect(lt(worst, 0.5f)) << "and tracks the noisy carrier";
    };
};

int main() { /* not needed for UT */ }
