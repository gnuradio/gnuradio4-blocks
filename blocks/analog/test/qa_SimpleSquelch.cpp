#include <boost/ut.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/analog/Squelch.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::analog::PowerSquelch;
using gr::blocks::analog::SimpleSquelch;
namespace spans = gr::blocks::testing::span;

using CF = std::complex<float>;

template<typename TBlock>
[[nodiscard]] TBlock makeBlock(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

template<typename T>
[[nodiscard]] std::vector<T> run(SimpleSquelch<T>& block, std::span<const T> input, std::size_t chunkSize = 0UZ) {
    std::vector<T>    output(input.size());
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    for (std::size_t base = 0UZ; base < input.size(); base += stride) {
        const std::size_t count = std::min(stride, input.size() - base);
        std::ignore             = block.processBulk(input.subspan(base, count), std::span<T>(output).subspan(base, count));
    }
    return output;
}

/// @brief A fading tone that crosses the threshold in both directions several times.
[[nodiscard]] std::vector<CF> fadingTone(std::size_t nSamples) {
    std::vector<CF> signal(nSamples);
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        const double magnitude = 0.5 * (1.0 + std::sin(2.0 * std::numbers::pi * 3.0 * static_cast<double>(i) / static_cast<double>(nSamples)));
        const double phase     = 2.0 * std::numbers::pi * 0.031 * static_cast<double>(i);
        signal[i]              = CF{static_cast<float>(magnitude * std::cos(phase)), static_cast<float>(magnitude * std::sin(phase))};
    }
    return signal;
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

const boost::ut::suite<"SimpleSquelch"> simpleSquelchTests = [] {
    using namespace boost::ut;

    "it agrees with PowerSquelch everywhere except the sample that closes a burst"_test = [] {
        const gr::property_map detector = {{"alpha", 0.01}, {"threshold_db", -12.0}};
        const std::vector<CF>  input    = fadingTone(6000UZ);

        SimpleSquelch<CF>     simple = makeBlock<SimpleSquelch<CF>>(detector);
        const std::vector<CF> plain  = run<CF>(simple, std::span<const CF>(input));

        gr::property_map ramped = detector;
        ramped["ramp"]          = 0U;
        ramped["gate"]          = false;
        PowerSquelch<CF> power  = makeBlock<PowerSquelch<CF>>(ramped);
        const auto       ramp   = spans::run<PowerSquelch<CF>, CF>(power, std::span<const CF>(input));

        expect(eq(ramp.samples.size(), plain.size()));

        // The ramp machine passes the sample the detector closed on at full amplitude, so the end-of-burst tag lands on
        // signal. A memoryless reference gates that sample instead, so the two disagree on exactly the tagged samples.
        std::vector<std::size_t> differing;
        for (std::size_t i = 0UZ; i < plain.size(); ++i) {
            if (plain[i] != ramp.samples[i]) {
                differing.push_back(i);
            }
        }
        expect(gt(differing.size(), 0UZ)) << "the test signal must close the squelch at least once";
        expect(that % (differing == ramp.offsetsOf(gr::blocks::analog::detail::kSquelchEndOfBurst))) << "one detector, so nothing else can differ";
        for (const std::size_t at : differing) {
            expect(eq(ramp.samples[at], input[at])) << "PowerSquelch passes the closing sample whole at " << at;
            expect(eq(plain[at], CF{})) << "SimpleSquelch, which has no tag to place, zeroes it at " << at;
        }
    };

    "it is 1:1 and emits nothing but samples"_test = [] {
        SimpleSquelch<float>     block = makeBlock<SimpleSquelch<float>>({{"alpha", 0.5}, {"threshold_db", -20.0}});
        const std::vector<float> input(1000UZ, 0.5f);
        for (const std::size_t chunkSize : {1UZ, 7UZ, 333UZ, 4096UZ}) {
            SimpleSquelch<float> chunked = makeBlock<SimpleSquelch<float>>({{"alpha", 0.5}, {"threshold_db", -20.0}});
            expect(eq(run<float>(chunked, std::span<const float>(input), chunkSize).size(), input.size())) << "chunk " << chunkSize;
        }
        expect(eq(run<float>(block, std::span<const float>(input)).size(), input.size()));
    };

    "output does not depend on chunking"_test = [] {
        const gr::property_map settings = {{"alpha", 0.001}, {"threshold_db", -12.0}};
        const std::vector<CF>  input    = fadingTone(6000UZ);

        SimpleSquelch<CF>     reference = makeBlock<SimpleSquelch<CF>>(settings);
        const std::vector<CF> want      = run<CF>(reference, std::span<const CF>(input));

        for (const std::size_t chunkSize : {1UZ, 7UZ, 4096UZ}) {
            SimpleSquelch<CF>     block = makeBlock<SimpleSquelch<CF>>(settings);
            const std::vector<CF> got   = run<CF>(block, std::span<const CF>(input), chunkSize);
            expect(std::ranges::equal(got, want)) << "chunk size " << chunkSize << " must be bit-identical";
        }
    };

    "the gate is hard, and the threshold is a power level"_test = [] {
        SimpleSquelch<float>     block = makeBlock<SimpleSquelch<float>>({{"alpha", 1.0}, {"threshold_db", -6.02}});
        const std::vector<float> loud(4UZ, 0.6f);
        const std::vector<float> quiet(4UZ, 0.45f);

        const std::vector<float> passed = run<float>(block, std::span<const float>(loud));
        expect(std::ranges::equal(passed, loud)) << "above the threshold the samples come through untouched";
        const std::vector<float> blocked = run<float>(block, std::span<const float>(quiet));
        expect(std::ranges::all_of(blocked, [](float sample) { return sample == 0.f; })) << "below it they are exactly zero";
        expect(!block.unmuted.value);
    };

    "a run of exact zeros leaves the detector at zero, not at a subnormal"_test = [] {
        // The detector keeps a double, so a squelched stream has to stay silent a long time to walk it into the
        // subnormals: at the default alpha = 1e-4 it takes 7 083 610 samples, some 148 s at 48 kHz. Only the pole sets
        // the count, so the case runs at alpha = 1e-2, where it is 70 485, and the property is the same one.
        constexpr std::size_t kZeros = 80000UZ;

        SimpleSquelch<float>     block = makeBlock<SimpleSquelch<float>>({{"alpha", 1e-2}, {"threshold_db", -150.0}});
        const std::vector<float> loud(1000UZ, 1.f);
        std::ignore = run<float>(block, std::span<const float>(loud));
        expect(gt(block._detector._power, 0.9)) << "the average is up before the silence starts";

        const std::vector<float> zeros(kZeros, 0.f);
        const std::vector<float> got = run<float>(block, std::span<const float>(zeros), 4096UZ);

        expect(eq(std::fpclassify(block._detector._power), FP_ZERO)) << "the tracked power is exactly zero, not the smallest subnormal";
        expect(eq(std::ranges::count_if(got, [](float sample) { return std::fpclassify(sample) == FP_SUBNORMAL; }), std::ptrdiff_t{0})) << "and no output on the run is subnormal";
    };

    "the flush changes nothing above the subnormal range"_test = [] {
        // The recursion as it stood, sample by sample against the block, over a step and the decay that follows it. At
        // alpha = 0.5 the state passes 1e-300 after 997 zeros and cannot be flushed before 1022, so every sample
        // compared here is one the flush had no part in.
        constexpr double      kAlpha = 0.5;
        constexpr std::size_t kStep  = 8UZ;
        constexpr std::size_t kDecay = 997UZ;

        SimpleSquelch<float> block     = makeBlock<SimpleSquelch<float>>({{"alpha", kAlpha}, {"threshold_db", -150.0}});
        double               reference = 0.0;
        bool                 identical = true;

        for (std::size_t i = 0UZ; i < kStep + kDecay; ++i) {
            const float sample = i < kStep ? 1.f : 0.f;
            reference          = (1.0 - kAlpha) * reference + kAlpha * static_cast<double>(sample) * static_cast<double>(sample);

            float made  = 0.f;
            std::ignore = block.processBulk(std::span<const float>(&sample, 1UZ), std::span<float>(&made, 1UZ));
            identical   = identical && block._detector._power == reference && made == (reference >= block._detector._threshold ? sample : 0.f);
        }
        expect(identical) << "state and output agree bit for bit with the unflushed recursion at every sample";
        expect(lt(reference, 1e-300)) << "and the comparison ran until the state was below 1e-300";
    };

    "nanoseconds per sample"_test = [] {
        if (std::getenv("ENABLE_BENCHMARK_TESTS") == nullptr) {
            return; // opt-in: a throughput figure belongs to a controlled run, not to every ctest invocation
        }
        using Clock = std::chrono::steady_clock;

        const std::vector<CF> x = fadingTone(1UZ << 16);
        std::vector<CF>       yComplex(x.size());
        std::vector<float>    xReal(x.size());
        std::vector<float>    yReal(x.size());
        std::ranges::transform(x, xReal.begin(), [](CF sample) { return sample.real(); });

        SimpleSquelch<CF>    complexBlock = makeBlock<SimpleSquelch<CF>>({{"alpha", 1e-3}, {"threshold_db", -12.0}});
        SimpleSquelch<float> realBlock    = makeBlock<SimpleSquelch<float>>({{"alpha", 1e-3}, {"threshold_db", -12.0}});
        constexpr int        kRepeats     = 7;

        double bestComplex = 1e30, worstComplex = 0.0, bestReal = 1e30, worstReal = 0.0;
        for (int repeat = 0; repeat < kRepeats; ++repeat) { // arms interleaved, so a thermal drift moves both of them
            auto start   = Clock::now();
            std::ignore  = complexBlock.processBulk(std::span<const CF>(x), std::span<CF>(yComplex));
            double ns    = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count()) / static_cast<double>(x.size());
            bestComplex  = std::min(bestComplex, ns);
            worstComplex = std::max(worstComplex, ns);

            start       = Clock::now();
            std::ignore = realBlock.processBulk(std::span<const float>(xReal), std::span<float>(yReal));
            ns          = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count()) / static_cast<double>(x.size());
            bestReal    = std::min(bestReal, ns);
            worstReal   = std::max(worstReal, ns);
        }
        std::println("SimpleSquelch<complex<float>>: best {:.3f} ns/sample, spread {:.3f} ns", bestComplex, worstComplex - bestComplex);
        std::println("SimpleSquelch<float>: best {:.3f} ns/sample, spread {:.3f} ns", bestReal, worstReal - bestReal);
    };

    "a non-reserved tag key rides through at its own offset"_test = [] {
        const std::vector<std::size_t> offsets = privateTagOffsets<float, float, SimpleSquelch<float>>({{"threshold_db", -150.0}});
        expect(that % (offsets == std::vector<std::size_t>{7UZ, 300UZ, 1000UZ})) << "the pass-all policy keeps a key the auto-forward set does not name";
    };
};

int main() { /* tests are automatically registered and run */ }
