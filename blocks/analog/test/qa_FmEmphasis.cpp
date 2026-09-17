#include <boost/ut.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/analog/FmEmphasis.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace {

using gr::blocks::analog::FmDeemphasis;
using gr::blocks::analog::FmPreemphasis;

constexpr double kPi = std::numbers::pi;

template<typename TBlock>
TBlock makeEmphasis(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

template<std::floating_point T = double>
FmDeemphasis<T> makeDeemphasis(float sampleRate, double tau) {
    return makeEmphasis<FmDeemphasis<T>>({{"sample_rate", sampleRate}, {"tau", tau}});
}

template<std::floating_point T = double>
FmPreemphasis<T> makePreemphasis(float sampleRate, double tau, double highCorner = 0.0) {
    return makeEmphasis<FmPreemphasis<T>>({{"sample_rate", sampleRate}, {"tau", tau}, {"high_corner", highCorner}});
}

template<typename T, typename TBlock>
T filterOne(TBlock& block, T value) {
    T output{};
    std::ignore = block.processBulk(std::span<const T>(&value, 1UZ), std::span<T>(&output, 1UZ));
    return output;
}

// steady state of a real LTI section driven by cos and sin of the same frequency is H(w) * e^{jwn},
// so the magnitude response is the hypotenuse of the two outputs taken at the same sample index
template<typename FCosinePath, typename FSinePath>
double steadyStateGain(FCosinePath&& cosinePath, FSinePath&& sinePath, double frequency, double sampleRate, std::size_t warmUp = 4000UZ) {
    const double omega = 2.0 * kPi * frequency / sampleRate;
    double       cosineOut{};
    double       sineOut{};
    for (std::size_t n = 0UZ; n < warmUp; ++n) {
        const double phase = omega * static_cast<double>(n);
        cosineOut          = cosinePath(std::cos(phase));
        sineOut            = sinePath(std::sin(phase));
    }
    return std::hypot(cosineOut, sineOut);
}

double deemphasisGain(float sampleRate, double tau, double frequency) {
    auto cosine = makeDeemphasis(sampleRate, tau);
    auto sine   = makeDeemphasis(sampleRate, tau);
    return steadyStateGain([&cosine](double x) { return filterOne(cosine, x); }, [&sine](double x) { return filterOne(sine, x); }, frequency, static_cast<double>(sampleRate));
}

double preemphasisGain(float sampleRate, double tau, double highCorner, double frequency) {
    auto cosine = makePreemphasis(sampleRate, tau, highCorner);
    auto sine   = makePreemphasis(sampleRate, tau, highCorner);
    return steadyStateGain([&cosine](double x) { return filterOne(cosine, x); }, [&sine](double x) { return filterOne(sine, x); }, frequency, static_cast<double>(sampleRate));
}

double cascadeGain(float sampleRate, double tau, double highCorner, double frequency) {
    auto cosinePre = makePreemphasis(sampleRate, tau, highCorner);
    auto cosineDe  = makeDeemphasis(sampleRate, tau);
    auto sinePre   = makePreemphasis(sampleRate, tau, highCorner);
    auto sineDe    = makeDeemphasis(sampleRate, tau);
    return steadyStateGain([&](double x) { return filterOne(cosineDe, filterOne(cosinePre, x)); }, [&](double x) { return filterOne(sineDe, filterOne(sinePre, x)); }, frequency, static_cast<double>(sampleRate));
}

double prewarp(double frequency, double sampleRate) { return std::tan(kPi * frequency / sampleRate); }

double cornerFromTau(double tau) { return 1.0 / (2.0 * kPi * tau); }

template<typename TBlock>
std::vector<double> filterChunked(TBlock& block, const std::vector<double>& input, std::size_t chunkSize) {
    const std::size_t   stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    std::vector<double> output(input.size());
    for (std::size_t base = 0UZ; base < input.size(); base += stride) {
        const std::size_t count = std::min(stride, input.size() - base);
        std::ignore             = block.processBulk(std::span<const double>(input).subspan(base, count), std::span<double>(output).subspan(base, count));
    }
    return output;
}

/// @brief The offsets at which a key outside `gr::tag::kDefaultTags` reaches the sink through @p TBlock.
template<typename TIn, typename TOut, typename TBlock>
[[nodiscard]] std::vector<std::size_t> privateTagOffsets(gr::property_map settings) {
    using gr::testing::ProcessFunction;
    using gr::testing::TagSink;
    using gr::testing::TagSource;

    const gr::property_map::key_type key{"private_key"};
    const gr::pmt::Value             value{std::string("carried")};

    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<TIn, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", gr::Size_t(2048)}, {"mark_tag", false}});
    for (const std::size_t at : {7UZ, 300UZ, 1000UZ}) {
        source._tags.emplace_back(at, gr::property_map{{key, value}});
    }
    auto& block = graph.emplaceBlock<TBlock>(std::move(settings));
    auto& sink  = graph.emplaceBlock<TagSink<TOut, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

    boost::ut::expect(graph.connect<"out", "in">(source, block).has_value());
    boost::ut::expect(graph.connect<"out", "in">(block, sink).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value());

    std::vector<std::size_t> offsets;
    for (const gr::Tag& tag : sink._tags) {
        if (const auto found = tag.map.find(key); found != tag.map.end() && found->second == value) {
            offsets.push_back(tag.index);
        }
    }
    return offsets;
}

} // namespace

const boost::ut::suite<"FmEmphasis"> fmEmphasisTests = [] {
    using namespace boost::ut;

    "coefficient anchors pin the prewarp and normalization conventions"_test = [] {
        const auto de48_75 = makeDeemphasis(48000.f, 75e-6);
        expect(approx(de48_75._section.b0, 0.12264454924040032, 1e-11));
        expect(approx(de48_75._section.b1, 0.12264454924040032, 1e-11));
        expect(approx(de48_75._section.p, 0.7547109015191994, 1e-9));

        const auto de48_50 = makeDeemphasis(48000.f, 50e-6);
        expect(approx(de48_50._section.b0, 0.17450929522806247, 1e-10));
        expect(approx(de48_50._section.b1, 0.17450929522806247, 1e-10));
        expect(approx(de48_50._section.p, 0.6509814095438752, 1e-9));

        const auto de96_75 = makeDeemphasis(96000.f, 75e-6);
        expect(approx(de96_75._section.b0, 0.06503284897603764, 1e-11));
        expect(approx(de96_75._section.b1, 0.06503284897603764, 1e-11));
        expect(approx(de96_75._section.p, 0.8699343020479247, 1e-9));

        const auto pre48_75 = makePreemphasis(48000.f, 75e-6);
        expect(approx(pre48_75._section.b0, 7.290729370612733, 1e-8));
        expect(approx(pre48_75._section.b1, -5.5023929360276425, 1e-8));
        expect(approx(pre48_75._section.p, -0.7883364345850924, 1e-9));

        const auto defaults = makeEmphasis<FmDeemphasis<double>>({});
        expect(approx(defaults._section.b0, 0.12264454924040032, 1e-11)) << "a block left at its defaults must still design its filter";
    };

    "an out-of-range high corner falls back to 0.925 * Nyquist"_test = [] {
        const auto defaulted  = makePreemphasis(48000.f, 75e-6);
        const auto explicitly = makePreemphasis(48000.f, 75e-6, 22200.0);
        const auto tooHigh    = makePreemphasis(48000.f, 75e-6, 1e9);

        expect(eq(defaulted._section.b0, explicitly._section.b0));
        expect(eq(defaulted._section.p, explicitly._section.p));
        expect(eq(tooHigh._section.b0, explicitly._section.b0));
        expect(eq(tooHigh._section.p, explicitly._section.p));
    };

    "both filters have unity DC gain"_test = []<typename T> {
        for (const float sampleRate : {48000.f, 96000.f}) {
            for (const double tau : {50e-6, 75e-6}) {
                auto deemphasis  = makeDeemphasis<T>(sampleRate, tau);
                auto preemphasis = makePreemphasis<T>(sampleRate, tau);

                T deemphasized{};
                T preemphasized{};
                for (std::size_t n = 0UZ; n < 200UZ; ++n) {
                    deemphasized  = filterOne(deemphasis, T(1));
                    preemphasized = filterOne(preemphasis, T(1));
                }
                expect(approx(static_cast<double>(deemphasized), 1.0, 1e-6)) << "de-emphasis DC gain at fs=" << sampleRate;
                expect(approx(static_cast<double>(preemphasized), 1.0, 1e-6)) << "pre-emphasis DC gain at fs=" << sampleRate;
            }
        }
    } | std::tuple<float, double>{};

    "prewarping puts the de-emphasis corner exactly at 1/(2*pi*tau)"_test = [] {
        for (const float sampleRate : {48000.f, 96000.f, 192000.f}) {
            for (const double tau : {50e-6, 75e-6}) {
                const double corner = cornerFromTau(tau);
                expect(approx(deemphasisGain(sampleRate, tau, corner), 0.70711, 1e-4)) << "corner gain at fs=" << sampleRate << " tau=" << tau;
            }
        }
    };

    "magnitude response follows the closed forms"_test = [] {
        constexpr float  sampleRate = 48000.f;
        constexpr double tau        = 75e-6;
        const double     rate       = static_cast<double>(sampleRate);
        const double     kLow       = prewarp(cornerFromTau(tau), rate);
        const double     kHigh      = prewarp(22200.0, rate);

        for (const double frequency : {100.0, 1000.0, cornerFromTau(tau), 5000.0, 15000.0, 20000.0}) {
            const double t        = prewarp(frequency, rate);
            const double expected = 1.0 / std::sqrt(1.0 + (t / kLow) * (t / kLow));
            expect(approx(deemphasisGain(sampleRate, tau, frequency) / expected, 1.0, 1e-4)) << "de-emphasis at " << frequency << " Hz";
        }

        for (const double frequency : {100.0, 500.0, 5000.0, 10000.0, 20000.0}) {
            const double t        = prewarp(frequency, rate);
            const double expected = std::sqrt(1.0 + (t / kLow) * (t / kLow)) / std::sqrt(1.0 + (t / kHigh) * (t / kHigh));
            expect(approx(preemphasisGain(sampleRate, tau, 0.0, frequency) / expected, 1.0, 1e-4)) << "pre-emphasis at " << frequency << " Hz";
        }

        expect(approx(deemphasisGain(sampleRate, tau, 1000.0), 0.9054155745528311, 1e-6));
        expect(approx(deemphasisGain(sampleRate, tau, cornerFromTau(tau)), 0.7071067811865475, 1e-6));
        expect(approx(deemphasisGain(sampleRate, tau, 15000.0), 0.0929991584122098, 1e-6));
    };

    "pre-emphasis into de-emphasis is exactly a single pole at the high corner"_test = [] {
        constexpr float  sampleRate = 48000.f;
        constexpr double tau        = 75e-6;
        constexpr double highCorner = 22200.0;

        expect(approx(cascadeGain(sampleRate, tau, highCorner, 1000.0) / 0.9999699113340105, 1.0, 1e-6));
        expect(approx(cascadeGain(sampleRate, tau, highCorner, 15000.0) / 0.984671395284593, 1.0, 1e-6));
        expect(approx(cascadeGain(sampleRate, tau, highCorner, 23000.0) / 0.4844514701633041, 1.0, 1e-6));

        const double kHigh = prewarp(highCorner, static_cast<double>(sampleRate));
        for (const double frequency : {200.0, 4000.0, 18000.0}) {
            const double t        = prewarp(frequency, static_cast<double>(sampleRate));
            const double expected = 1.0 / std::sqrt(1.0 + (t / kHigh) * (t / kHigh));
            expect(approx(cascadeGain(sampleRate, tau, highCorner, frequency) / expected, 1.0, 1e-6)) << "cascade at " << frequency << " Hz";
        }
    };

    "the pre-emphasis pole is negative and inside the unit circle"_test = [] {
        expect(lt(makePreemphasis(48000.f, 75e-6)._section.p, 0.0)) << "a high corner above fs/4 must give a negative pole";
        for (const float sampleRate : {8000.f, 48000.f, 96000.f, 192000.f}) {
            for (const double tau : {50e-6, 75e-6}) {
                expect(lt(std::abs(makePreemphasis(sampleRate, tau)._section.p), 1.0)) << "unstable pole at fs=" << sampleRate << " tau=" << tau;
                expect(lt(std::abs(makeDeemphasis(sampleRate, tau)._section.p), 1.0)) << "unstable pole at fs=" << sampleRate << " tau=" << tau;
            }
        }
    };

    "retuning tau mid-stream keeps the state and does not click"_test = [] {
        auto deemphasis  = makeDeemphasis(48000.f, 75e-6);
        auto preemphasis = makePreemphasis(48000.f, 75e-6);

        double lastDeemphasized{};
        double lastPreemphasized{};
        for (std::size_t n = 0UZ; n < 500UZ; ++n) {
            lastDeemphasized  = filterOne(deemphasis, 1.0);
            lastPreemphasized = filterOne(preemphasis, 1.0);
        }

        std::ignore = deemphasis.settings().setStaged({{"tau", 50e-6}});
        std::ignore = deemphasis.settings().applyStagedParameters();
        std::ignore = preemphasis.settings().setStaged({{"tau", 50e-6}});
        std::ignore = preemphasis.settings().applyStagedParameters();

        expect(approx(deemphasis._section.p, 0.6509814095438752, 1e-9)) << "the retune must have taken effect";
        for (std::size_t n = 0UZ; n < 50UZ; ++n) {
            const double deemphasized  = filterOne(deemphasis, 1.0);
            const double preemphasized = filterOne(preemphasis, 1.0);
            expect(lt(std::abs(deemphasized - lastDeemphasized), 1e-6)) << "de-emphasis click at n=" << n;
            expect(lt(std::abs(preemphasized - lastPreemphasized), 1e-6)) << "pre-emphasis click at n=" << n;
            lastDeemphasized  = deemphasized;
            lastPreemphasized = preemphasized;
        }
    };

    "unusable parameters are rejected at settings time"_test = [] {
        expect(throws([] { std::ignore = makeDeemphasis(-1.f, 75e-6); })) << "negative sample_rate";
        expect(throws([] { std::ignore = makeDeemphasis(0.f, 75e-6); })) << "zero sample_rate";
        expect(throws([] { std::ignore = makeDeemphasis(48000.f, -75e-6); })) << "negative tau";
        expect(throws([] { std::ignore = makeDeemphasis(2000.f, 75e-6); })) << "corner at or above Nyquist";
        expect(throws([] { std::ignore = makePreemphasis(-1.f, 75e-6); })) << "negative sample_rate";
        expect(throws([] { std::ignore = makePreemphasis(48000.f, -75e-6); })) << "negative tau";
        // zero tau is no longer unusable: it is the bypass, pinned in its own case below
        expect(throws([] { std::ignore = makePreemphasis(2000.f, 75e-6); })) << "corner at or above Nyquist";
    };

    "output does not depend on chunking"_test = [] {
        std::vector<double> input(5000UZ);
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            input[i] = std::sin(2.0 * kPi * 997.0 * static_cast<double>(i) / 48000.0) + 0.25 * std::sin(2.0 * kPi * 11000.0 * static_cast<double>(i) / 48000.0);
        }

        auto                      deemphasisReference  = makeDeemphasis(48000.f, 75e-6);
        auto                      preemphasisReference = makePreemphasis(48000.f, 75e-6);
        const std::vector<double> deemphasized         = filterChunked(deemphasisReference, input, 0UZ);
        const std::vector<double> preemphasized        = filterChunked(preemphasisReference, input, 0UZ);

        for (const std::size_t chunkSize : {1UZ, 7UZ, 4096UZ}) {
            auto deemphasis  = makeDeemphasis(48000.f, 75e-6);
            auto preemphasis = makePreemphasis(48000.f, 75e-6);
            expect(std::ranges::equal(filterChunked(deemphasis, input, chunkSize), deemphasized)) << "de-emphasis chunk size " << chunkSize;
            expect(std::ranges::equal(filterChunked(preemphasis, input, chunkSize), preemphasized)) << "pre-emphasis chunk size " << chunkSize;
        }
    };

    "a non-reserved tag key rides through both halves at its own offset"_test = [] {
        const std::vector<std::size_t> expected{7UZ, 300UZ, 1000UZ};
        expect(that % (privateTagOffsets<float, float, FmDeemphasis<float>>({{"sample_rate", 48000.f}}) == expected)) << "de-emphasis keeps a key the auto-forward set does not name";
        expect(that % (privateTagOffsets<float, float, FmPreemphasis<float>>({{"sample_rate", 48000.f}}) == expected)) << "pre-emphasis keeps a key the auto-forward set does not name";
    };

    "tau = 0 is a bypass on both halves, bit for bit and at the same index"_test = [] {
        constexpr float kRate = 48000.f;

        std::vector<float> input(4096UZ);
        std::uint64_t      state = 0x243f6a8885a308d3ULL;
        for (float& sample : input) {
            state ^= state << 13U;
            state ^= state >> 7U;
            state ^= state << 17U;
            sample = 2.f * static_cast<float>(state % 2048ULL) / 2048.f - 1.f;
        }

        const auto through = [&](auto& block) {
            std::vector<float> output(input.size());
            std::ignore = block.processBulk(std::span<const float>(input), std::span<float>(output));
            return output;
        };

        FmDeemphasis<float> deemph({{"sample_rate", kRate}, {"tau", 0.0}});
        deemph.settings().init();
        std::ignore          = deemph.settings().applyStagedParameters();
        const auto deemphOut = through(deemph);

        FmPreemphasis<float> preemph({{"sample_rate", kRate}, {"tau", 0.0}});
        preemph.settings().init();
        std::ignore           = preemph.settings().applyStagedParameters();
        const auto preemphOut = through(preemph);

        for (std::size_t k = 0UZ; k < input.size(); ++k) {
            expect(deemphOut[k] == input[k]) << "de-emphasis bypass differs at " << k;
            expect(preemphOut[k] == input[k]) << "pre-emphasis bypass differs at " << k;
        }

        // the same spelling on both, which is what lets one parameter turn the pair off together
        expect(nothrow([&] {
            FmPreemphasis<float> pair({{"sample_rate", kRate}, {"tau", 0.0}, {"high_corner", 12000.0}});
            pair.settings().init();
            std::ignore = pair.settings().applyStagedParameters();
        })) << "a bypassed pre-emphasis ignores its high corner rather than refusing it";

        // a negative or non-finite tau is still refused: zero is a convention, not a relaxation
        const auto refused = [kRate](double tau) {
            FmDeemphasis<float> block({{"sample_rate", kRate}, {"tau", tau}});
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        };
        expect(throws([&] { refused(-1e-6); }));
        expect(throws([&] { refused(std::numeric_limits<double>::quiet_NaN()); }));
    };

    "a run of exact zeros leaves the state at zero, not at a subnormal"_test = []<typename T> {
        // A squelch upstream writes exact zeros, and the recursion then decays by the pole every sample: at 96 kHz and
        // tau = 75 us, p = 0.8699, so a float state passes below its smallest normal after 627 samples and a double
        // after 5083. Without the flush it lands on the smallest subnormal and stays there -- 1.4e-45 * 0.87 rounds
        // back to 1.4e-45 -- and every sample after that costs an x86 microcode assist.
        constexpr std::size_t kZeros = 8000UZ;

        auto deemphasis  = makeDeemphasis<T>(96000.f, 75e-6);
        auto preemphasis = makePreemphasis<T>(96000.f, 75e-6);
        std::ignore      = filterOne<T>(deemphasis, T(1));
        std::ignore      = filterOne<T>(preemphasis, T(1));

        const std::vector<T> zeros(kZeros, T{});
        std::vector<T>       deemphasized(kZeros);
        std::vector<T>       preemphasized(kZeros);
        std::ignore = deemphasis.processBulk(std::span<const T>(zeros), std::span<T>(deemphasized));
        std::ignore = preemphasis.processBulk(std::span<const T>(zeros), std::span<T>(preemphasized));

        expect(eq(std::fpclassify(deemphasis._lastOutput), FP_ZERO)) << "the de-emphasis state is exactly zero";
        expect(eq(std::fpclassify(preemphasis._lastOutput), FP_ZERO)) << "the pre-emphasis state is exactly zero";
        expect(eq(std::fpclassify(deemphasized.back()), FP_ZERO)) << "and so is what it puts out";
        expect(eq(std::fpclassify(preemphasized.back()), FP_ZERO));

        const auto subnormals = [](const std::vector<T>& values) { return std::ranges::count_if(values, [](T value) { return std::fpclassify(value) == FP_SUBNORMAL; }); };
        expect(eq(subnormals(deemphasized), std::ptrdiff_t{0})) << "no output on the run is subnormal";
        expect(eq(subnormals(preemphasized), std::ptrdiff_t{0}));
    } | std::tuple<float, double>{};

    "nanoseconds per sample on a run of zeros"_test = [] {
        if (std::getenv("ENABLE_BENCHMARK_TESTS") == nullptr) {
            return; // opt-in: a throughput figure belongs to a controlled run, not to every ctest invocation
        }
        using Clock = std::chrono::steady_clock;

        constexpr std::size_t    kZeros = 100000UZ;
        constexpr int            kRuns  = 5;
        const std::vector<float> zeros(kZeros, 0.f);
        std::vector<float>       output(kZeros);

        const auto section = gr::blocks::analog::detail::designDeemphasis(96000.0, 75e-6);
        const auto b0      = static_cast<float>(section.b0);
        const auto b1      = static_cast<float>(section.b1);
        const auto p       = static_cast<float>(section.p);

        double without = 1e30;
        for (int run = 0; run < kRuns; ++run) { // the recursion as it stood, so the assists are what the two arms differ by
            float      lastInput  = 0.f;
            float      lastOutput = 1.f;
            const auto start      = Clock::now();
            for (std::size_t i = 0UZ; i < kZeros; ++i) {
                const float sample = zeros[i];
                lastOutput         = b0 * sample + b1 * lastInput + p * lastOutput;
                lastInput          = sample;
                output[i]          = lastOutput;
            }
            without = std::min(without, static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count()) / static_cast<double>(kZeros));
        }
        expect(eq(std::fpclassify(output.back()), FP_SUBNORMAL)) << "the arm without the flush ends on a subnormal, which is the cost being measured";

        double with = 1e30;
        for (int run = 0; run < kRuns; ++run) {
            auto block       = makeDeemphasis<float>(96000.f, 75e-6);
            std::ignore      = filterOne<float>(block, 1.f);
            const auto start = Clock::now();
            std::ignore      = block.processBulk(std::span<const float>(zeros), std::span<float>(output));
            with             = std::min(with, static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count()) / static_cast<double>(kZeros));
        }
        std::println("FmDeemphasis<float>, {} zeros after a signal: {:.3f} ns/sample without the flush, {:.3f} ns/sample with it", kZeros, without, with);
    };
};

int main() { /* not needed for ut */ }
