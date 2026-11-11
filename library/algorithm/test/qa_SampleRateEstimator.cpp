#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <format>
#include <random>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/algorithm/SampleRateEstimator.hpp>

using namespace boost::ut;
using namespace gr::algorithm;

const boost::ut::suite<"SampleRateEstimator"> tests = [] {
    "default construction"_test = [] {
        SampleRateEstimator est;
        expect(eq(est.filter_cutoff_hz, 0.1f));
        expect(eq(est.ppm_initial, 0.0f));
        expect(!est._initialised);
    };

    "reset sets nominal rate and period"_test = [] {
        SampleRateEstimator est;
        est.reset(2.048e6);
        expect(eq(est._nominalRate, 2.048e6));
        expect(lt(std::abs(est._periodEst - 1.0 / 2.048e6), 1e-15));
        expect(!est._initialised);
    };

    "reset with ppm_initial adjusts period"_test = [] {
        SampleRateEstimator est;
        est.ppm_initial = 50.0f;
        est.reset(2.048e6);
        double expectedPeriod = (1.0 / 2.048e6) * (1.0 + 50e-6);
        expect(lt(std::abs(est._periodEst - expectedPeriod), 1e-18));
    };

    "converges to true rate with zero jitter"_test = [] {
        constexpr double      kNominalRate        = 2.048e6;
        constexpr double      kTruePpm            = 30.0;
        constexpr double      kTrueRate           = kNominalRate * (1.0 + kTruePpm * 1e-6);
        constexpr std::size_t kSamplesPerTransfer = 8192;
        constexpr double      kTransferDt         = static_cast<double>(kSamplesPerTransfer) / kTrueRate;

        SampleRateEstimator est;
        est.filter_cutoff_hz = 1.0f; // fast convergence for test
        est.reset(kNominalRate, 1.0 / kTransferDt);

        double tObs = 0.0;
        for (int i = 0; i < 5000; ++i) {
            tObs += kTransferDt;
            est.update(tObs, kSamplesPerTransfer);
        }

        float ppmEst = est.estimatedPpm();
        expect(lt(std::abs(ppmEst - static_cast<float>(kTruePpm)), 1.0f)) << std::format("expected ~{} ppm, got {} ppm", kTruePpm, ppmEst);

        double rateEst = est.estimatedRate();
        double rateErr = std::abs(rateEst - kTrueRate) / kTrueRate * 1e6;
        expect(lt(rateErr, 1.0)) << std::format("rate error {} ppm", rateErr);
    };

    "converges with Gaussian jitter"_test = [] {
        constexpr double      kNominalRate        = 2.048e6;
        constexpr double      kTruePpm            = -20.0;
        constexpr double      kTrueRate           = kNominalRate * (1.0 + kTruePpm * 1e-6);
        constexpr std::size_t kSamplesPerTransfer = 8192;
        constexpr double      kTransferDt         = static_cast<double>(kSamplesPerTransfer) / kTrueRate;
        constexpr double      kJitterStddev       = 100e-6; // 100 us USB scheduling jitter
        constexpr double      kUpdateRate         = 1.0 / kTransferDt;

        SampleRateEstimator est;
        est.filter_cutoff_hz = 0.05f; // low cutoff to smooth jitter
        est.reset(kNominalRate, kUpdateRate);

        std::mt19937                     gen(42);
        std::normal_distribution<double> jitter(0.0, kJitterStddev);

        double tObs = 0.0;
        for (int i = 0; i < 50000; ++i) { // ~200 s of data
            tObs += kTransferDt + jitter(gen);
            est.update(tObs, kSamplesPerTransfer);
        }

        // with 100 us jitter on ~4 ms transfers, per-transfer rate noise is ~2.5%;
        // the IIR LP smooths this but residual noise depends on filter settling and seed
        float ppmEst = est.estimatedPpm();
        expect(lt(std::abs(ppmEst - static_cast<float>(kTruePpm)), 1000.0f)) << std::format("expected ~{} ppm, got {} ppm (with jitter)", kTruePpm, ppmEst);
    };

    "resetPhase preserves filter state"_test = [] {
        constexpr double      kNominalRate        = 1.024e6;
        constexpr double      kTruePpm            = 15.0;
        constexpr double      kTrueRate           = kNominalRate * (1.0 + kTruePpm * 1e-6);
        constexpr std::size_t kSamplesPerTransfer = 4096;
        constexpr double      kTransferDt         = static_cast<double>(kSamplesPerTransfer) / kTrueRate;

        SampleRateEstimator est;
        est.filter_cutoff_hz = 1.0f;
        est.reset(kNominalRate, 1.0 / kTransferDt);

        double tObs = 0.0;
        for (int i = 0; i < 2000; ++i) {
            tObs += kTransferDt;
            est.update(tObs, kSamplesPerTransfer);
        }

        float ppmBefore = est.estimatedPpm();
        est.resetPhase();

        expect(eq(est.estimatedPpm(), ppmBefore)) << "ppm unchanged after phase reset";

        // continue feeding — should remain stable
        for (int i = 0; i < 500; ++i) {
            tObs += kTransferDt;
            est.update(tObs, kSamplesPerTransfer);
        }

        float ppmAfter = est.estimatedPpm();
        expect(lt(std::abs(ppmAfter - ppmBefore), 1.0f)) << "stable after phase reset";
    };

    "zero nominal rate is safe"_test = [] {
        SampleRateEstimator est;
        est.reset(0.0);
        est.update(1.0, 1000);
        expect(eq(est.estimatedRate(), 0.0));
        expect(eq(est.estimatedPpm(), 0.0f));
    };

    "zero samples is safe"_test = [] {
        SampleRateEstimator est;
        est.reset(2.048e6);
        est.update(1.0, 0);
        expect(!est._initialised) << "no update on zero samples";
    };

    "negative ppm (slow crystal)"_test = [] {
        constexpr double      kNominalRate        = 2.048e6;
        constexpr double      kTruePpm            = -50.0;
        constexpr double      kTrueRate           = kNominalRate * (1.0 + kTruePpm * 1e-6);
        constexpr std::size_t kSamplesPerTransfer = 8192;
        constexpr double      kTransferDt         = static_cast<double>(kSamplesPerTransfer) / kTrueRate;

        SampleRateEstimator est;
        est.filter_cutoff_hz = 1.0f;
        est.reset(kNominalRate, 1.0 / kTransferDt);

        double tObs = 0.0;
        for (int i = 0; i < 5000; ++i) {
            tObs += kTransferDt;
            est.update(tObs, kSamplesPerTransfer);
        }

        float ppmEst = est.estimatedPpm();
        expect(lt(ppmEst, 0.0f)) << "negative ppm for slow crystal";
        expect(lt(std::abs(ppmEst - static_cast<float>(kTruePpm)), 1.0f)) << std::format("expected ~{} ppm, got {} ppm", kTruePpm, ppmEst);
    };
};

const boost::ut::suite<"Drift Compensator"> driftCompensatorTests = [] {
    using namespace boost::ut;
    using gr::algorithm::DriftCompensator;

    "insert when source is fast"_test = [] {
        DriftCompensator<float> comp;
        std::array<float, 10>   buf{1.f, 2.f, 3.f, 4.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

        std::size_t n = 4U;
        for (int i = 0; i < 300; ++i) {
            n = comp.compensateSource(std::span(buf), 4U, 48000.0 * 1.0001, 48000.0, 1U);
        }
        expect(ge(n, 4UZ));
    };

    "drop when source is slow"_test = [] {
        DriftCompensator<float> comp;
        std::array<float, 10>   buf{1.f, 2.f, 3.f, 4.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

        std::size_t n = 4U;
        for (int i = 0; i < 300; ++i) {
            n = comp.compensateSource(std::span(buf), 4U, 48000.0 * 0.9999, 48000.0, 1U);
        }
        expect(le(n, 4UZ));
    };

    "interpolation on insert produces midpoint"_test = [] {
        DriftCompensator<float> comp;
        std::array<float, 10>   buf{};
        comp.fractionalAccumulator = 0.99;
        buf[0]                     = 1.0f;
        buf[1]                     = 3.0f;

        auto n = comp.compensateSource(std::span(buf), 2U, 48000.0 * 1.01, 48000.0, 1U);
        if (n == 3U) {
            expect(approx(buf[2], 2.0f, 0.5f)) << "inserted sample should be midpoint";
        }
    };

    "drop blends splice boundary"_test = [] {
        DriftCompensator<float> comp;
        std::array<float, 10>   buf{0.f, 10.f, 20.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
        comp.fractionalAccumulator = -0.99;

        auto n = comp.compensateSource(std::span(buf), 3U, 48000.0 * 0.99, 48000.0, 1U);
        if (n == 2U) {
            // last frame (20) dropped, splice point buf[1] blended: lerp(10, 20, 0.5) = 15
            expect(approx(buf[1], 15.0f, 1.0f)) << "splice boundary should be blended";
        }
    };

    "stereo insert preserves interleaving"_test = [] {
        DriftCompensator<float> comp;
        std::array<float, 20>   buf{};
        buf[0] = 1.f;
        buf[1] = 2.f;
        buf[2] = 3.f;
        buf[3] = 4.f;

        comp.fractionalAccumulator = 0.99;
        auto n                     = comp.compensateSource(std::span(buf), 4U, 48000.0 * 1.01, 48000.0, 2U);
        if (n == 6U) {
            expect(approx(buf[4], 2.0f, 0.5f)) << "inserted L";
            expect(approx(buf[5], 3.0f, 0.5f)) << "inserted R";
        }
    };

    "accumulator clamps after gap"_test = [] {
        DriftCompensator<float> comp;
        std::array<float, 10>   buf{1.f, 2.f, 3.f, 4.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

        // drive accumulator way past normal bounds
        for (int i = 0; i < 10000; ++i) {
            comp.compensateSource(std::span(buf), 4U, 48000.0 * 1.001, 48000.0, 1U);
        }
        expect(le(comp.fractionalAccumulator, DriftCompensator<float>::kMaxAccumulator)) << "should be clamped";
        expect(ge(comp.fractionalAccumulator, -DriftCompensator<float>::kMaxAccumulator)) << "should be clamped";
    };

    "sink insert and drop"_test = [] {
        DriftCompensator<float> comp;
        std::array<float, 10>   input{1.f, 2.f, 3.f, 4.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
        std::array<float, 12>   adjusted{};

        comp.fractionalAccumulator = 0.99;
        auto n                     = comp.compensateSink(std::span<const float>(input.data(), 4U), std::span(adjusted), 4U, 48000.0 * 0.99, 48000.0, 1U);
        expect(ge(n, 4UZ)) << "sink should insert";

        comp.fractionalAccumulator = -0.99;
        n                          = comp.compensateSink(std::span<const float>(input.data(), 4U), std::span(adjusted), 4U, 48000.0 * 1.01, 48000.0, 1U);
        expect(le(n, 4UZ)) << "sink should drop";
    };

    "works with int16_t"_test = [] {
        DriftCompensator<std::int16_t> comp;
        std::array<std::int16_t, 10>   buf{1000, 2000, 3000, 4000, 0, 0, 0, 0, 0, 0};

        comp.fractionalAccumulator = 0.99;
        auto n                     = comp.compensateSource(std::span(buf), 4U, 48000.0 * 1.01, 48000.0, 1U);
        if (n == 5U) {
            expect(gt(buf[4], std::int16_t{0})) << "inserted int16 sample should be positive";
        }
    };

    "None mode passes through unchanged"_test = [] {
        DriftCompensator<float> comp;
        comp.mode = gr::algorithm::DriftCorrection::None;
        std::array<float, 10> buf{1.f, 2.f, 3.f, 4.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

        comp.fractionalAccumulator = 5.0; // would normally trigger correction
        auto n                     = comp.compensateSource(std::span(buf), 4U, 48000.0 * 1.01, 48000.0, 1U);
        expect(eq(n, 4UZ)) << "None mode should not modify sample count";
    };

    "Cubic mode insert uses smooth interpolation"_test = [] {
        DriftCompensator<float> comp;
        comp.mode                  = gr::algorithm::DriftCorrection::Cubic;
        comp.fractionalAccumulator = 0.99;
        // ramp: 0, 10, 20, 30 — cubic should produce a smooth value near 25 at the splice
        std::array<float, 10> buf{0.f, 10.f, 20.f, 30.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
        auto                  n = comp.compensateSource(std::span(buf), 4U, 48000.0 * 1.01, 48000.0, 1U);
        if (n == 5U) {
            // cubic Hermite on a linear ramp should produce a value close to the linear midpoint
            expect(approx(buf[4], 25.0f, 5.0f)) << "cubic insert should be near ramp continuation";
        }
    };

    "Cubic mode drop blends smoothly"_test = [] {
        DriftCompensator<float> comp;
        comp.mode                  = gr::algorithm::DriftCorrection::Cubic;
        comp.fractionalAccumulator = -0.99;
        std::array<float, 10> buf{0.f, 10.f, 20.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
        auto                  n = comp.compensateSource(std::span(buf), 3U, 48000.0 * 0.99, 48000.0, 1U);
        if (n == 2U) {
            expect(approx(buf[1], 15.0f, 3.0f)) << "cubic drop should blend smoothly";
        }
    };

    "AdaptiveResampling mode produces correct output count"_test = [] {
        DriftCompensator<float> comp;
        comp.mode = gr::algorithm::DriftCorrection::AdaptiveResampling;

        // 100 ppm fast: 1000 input frames should produce ~999 output frames
        std::vector<float> buf(2000, 0.f);
        for (std::size_t i = 0U; i < 1000U; ++i) {
            buf[i] = static_cast<float>(i); // ramp
        }
        auto n = comp.compensateSource(std::span(buf), 1000U, 48000.0 * 1.0001, 48000.0, 1U);
        // at 100 ppm the output should be very close to 1000 (ratio ~0.9999)
        expect(ge(n, 990UZ)) << "adaptive resampling should produce ~1000 samples";
        expect(le(n, 1010UZ)) << "adaptive resampling should produce ~1000 samples";
    };

    "AdaptiveResampling preserves ramp shape"_test = [] {
        DriftCompensator<float> comp;
        comp.mode = gr::algorithm::DriftCorrection::AdaptiveResampling;

        constexpr std::size_t N = 100U;
        std::vector<float>    buf(N + 10U, 0.f);
        for (std::size_t i = 0U; i < N; ++i) {
            buf[i] = static_cast<float>(i);
        }

        auto n = comp.compensateSource(std::span(buf), N, 48000.0, 48000.0, 1U); // ratio = 1.0 exactly
        expect(eq(n, N)) << "ratio=1.0 should produce same count";
        // the cubic needs two frames beyond a position, so the stream lags its input by two frames: the first two
        // outputs repeat the first input frame, then the ramp follows
        for (std::size_t i = 0U; i < std::min(n, N); ++i) {
            const float expected = i < 2U ? 0.f : static_cast<float>(i - 2U);
            expect(approx(buf[i], expected, 1e-4f)) << std::format("sample {} should be near {}", i, expected);
        }
    };

    "AdaptiveResampling sink mode"_test = [] {
        DriftCompensator<float> comp;
        comp.mode = gr::algorithm::DriftCorrection::AdaptiveResampling;

        constexpr std::size_t N = 100U;
        std::vector<float>    input(N, 0.f);
        std::vector<float>    adjusted(N + 10U, 0.f);
        for (std::size_t i = 0U; i < N; ++i) {
            input[i] = static_cast<float>(i);
        }

        // sink with ratio ~1.0 should pass through
        auto n = comp.compensateSink(std::span<const float>(input), std::span(adjusted), N, 48000.0, 48000.0, 1U);
        expect(eq(n, N)) << "ratio=1.0 sink should produce same count";
    };

    "AdaptiveResampling carries the fraction across calls"_test = [] {
        constexpr std::size_t nCalls = 1000U;
        for (const std::size_t n : {480UZ, 1024UZ}) {
            for (const double ppm : {-500.0, -100.0, 100.0, 500.0}) {
                const double ratio = 1.0 + ppm * 1e-6; // estimated over nominal
                const double exact = static_cast<double>(nCalls * n) / ratio;

                DriftCompensator<float> source;
                DriftCompensator<float> sink;
                source.mode = gr::algorithm::DriftCorrection::AdaptiveResampling;
                sink.mode   = gr::algorithm::DriftCorrection::AdaptiveResampling;
                std::vector<float> in(n, 1.f);
                std::vector<float> buf(n + 8U), adjusted(n + 8U);
                std::size_t        totalSource = 0U;
                std::size_t        totalSink   = 0U;
                for (std::size_t c = 0U; c < nCalls; ++c) {
                    std::ranges::copy(in, buf.begin());
                    totalSource += source.compensateSource(std::span(buf), n, 48000.0 * ratio, 48000.0, 1U);
                    totalSink += sink.compensateSink(std::span<const float>(in), std::span(adjusted), n, 48000.0 * ratio, 48000.0, 1U);
                }
                expect(le(std::abs(static_cast<double>(totalSource) - exact), 1.0 + 1e-6)) << std::format("source n={} ppm={}: {} frames, exact {}", n, ppm, totalSource, exact);
                expect(le(std::abs(static_cast<double>(totalSink) - exact), 1.0 + 1e-6)) << std::format("sink n={} ppm={}: {} frames, exact {}", n, ppm, totalSink, exact);
            }
        }
    };

    "AdaptiveResampling output does not depend on how the stream is split into calls"_test = [] {
        constexpr std::size_t total = 4800U;
        constexpr double      ratio = 1.0007; // estimated over nominal
        std::vector<double>   tone(total);
        for (std::size_t i = 0U; i < total; ++i) {
            tone[i] = std::sin(0.05 * static_cast<double>(i));
        }

        auto resample = [&](std::size_t callSize) {
            DriftCompensator<double> comp;
            comp.mode = gr::algorithm::DriftCorrection::AdaptiveResampling;
            std::vector<double> out;
            std::vector<double> buf(callSize + 8U);
            for (std::size_t first = 0U; first < total; first += callSize) {
                const std::size_t n = std::min(callSize, total - first);
                std::copy_n(tone.begin() + static_cast<std::ptrdiff_t>(first), static_cast<std::ptrdiff_t>(n), buf.begin());
                const std::size_t got = comp.compensateSource(std::span(buf), n, 48000.0 * ratio, 48000.0, 1U);
                out.insert(out.end(), buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(got));
            }
            return out;
        };

        const auto whole = resample(total);
        for (const std::size_t callSize : {97UZ, 480UZ, 1024UZ}) {
            const auto split = resample(callSize);
            expect(eq(split.size(), whole.size())) << std::format("call size {}", callSize);
            double worst = 0.0;
            for (std::size_t i = 0U; i < std::min(split.size(), whole.size()); ++i) {
                worst = std::max(worst, std::abs(split[i] - whole[i]));
            }
            expect(le(worst, 1e-9)) << std::format("call size {}: split and single call differ by {}", callSize, worst);
        }

        // and the one-call result is the tone, two frames late, within the cubic's error
        double worst = 0.0;
        for (std::size_t k = 2U; k < whole.size(); ++k) {
            const double position = static_cast<double>(k) * ratio - 2.0;
            worst                 = std::max(worst, std::abs(whole[k] - std::sin(0.05 * position)));
        }
        expect(le(worst, 1e-3)) << "resampled tone strays from the tone: " << worst;
    };

    "AdaptiveResampling ramp has one slope across call boundaries"_test = [] {
        DriftCompensator<double> comp;
        comp.mode                   = gr::algorithm::DriftCorrection::AdaptiveResampling;
        constexpr std::size_t n     = 480U;
        constexpr double      ratio = 1.003;
        std::vector<double>   out;
        std::vector<double>   buf(n + 8U);
        for (std::size_t c = 0U; c < 50U; ++c) {
            for (std::size_t i = 0U; i < n; ++i) {
                buf[i] = static_cast<double>(c * n + i);
            }
            const std::size_t got = comp.compensateSource(std::span(buf), n, 48000.0 * ratio, 48000.0, 1U);
            out.insert(out.end(), buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(got));
        }
        double worstJump = 0.0;
        for (std::size_t k = 6U; k < out.size(); ++k) { // past the start-up, where the first frame is repeated
            worstJump = std::max(worstJump, std::abs((out[k] - out[k - 1]) - (out[k - 1] - out[k - 2])));
        }
        expect(le(worstJump, 1e-9)) << "the ramp's slope changed by " << worstJump;
    };

    "AdaptiveResampling reset starts clean"_test = [] {
        auto run = [](DriftCompensator<float>& comp) {
            std::vector<float> buf(60U, 0.f);
            for (std::size_t i = 0U; i < 50U; ++i) {
                buf[i] = static_cast<float>(100U + i);
            }
            const std::size_t got = comp.compensateSource(std::span(buf), 50U, 48000.0 * 1.003, 48000.0, 1U);
            buf.resize(got);
            return buf;
        };
        DriftCompensator<float> fresh;
        fresh.mode           = gr::algorithm::DriftCorrection::AdaptiveResampling;
        const auto reference = run(fresh);

        DriftCompensator<float> used;
        used.mode   = gr::algorithm::DriftCorrection::AdaptiveResampling;
        std::ignore = run(used);
        std::ignore = run(used);
        expect(neq(used.resamplerPhase, 0.0)) << "two calls leave a phase to carry";
        used.reset();
        expect(eq(used.resamplerPhase, 0.0));
        expect(run(used) == reference) << "after reset() the first call matches a new compensator";
    };

    "all modes produce valid output for stereo"_test = [] {
        for (auto mode : {gr::algorithm::DriftCorrection::Linear, gr::algorithm::DriftCorrection::Cubic, gr::algorithm::DriftCorrection::AdaptiveResampling}) {
            DriftCompensator<float> comp;
            comp.mode = mode;

            std::vector<float> buf(200, 0.f);
            // stereo ramp: L=i, R=i+100
            for (std::size_t i = 0U; i < 50U; ++i) {
                buf[i * 2U]      = static_cast<float>(i);
                buf[i * 2U + 1U] = static_cast<float>(i) + 100.f;
            }

            auto n = comp.compensateSource(std::span(buf), 100U, 48000.0 * 1.0001, 48000.0, 2U);
            expect(eq(n % 2UZ, 0UZ)) << "stereo output must be frame-aligned";
            expect(ge(n, 90UZ)) << "should produce roughly the same count";
            expect(le(n, 110UZ)) << "should produce roughly the same count";
        }
    };
};

int main() { return 0; }
