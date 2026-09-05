#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/analog/QuadratureDemod.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

// The tolerances below are set by float rounding of the complex product, not by the arctangent.
// A reference vector produced by a 256-entry interpolated arctangent table carries that table's error
// against an exact atan2, 6.1e-7 rad mean and 1.31e-6 rad maximum, so comparing against one needs a
// per-sample tolerance of gain * 3e-6. Asserting a tighter agreement would be asserting that this
// block reproduced the table, which it must not.

namespace {

using gr::blocks::analog::QuadratureDemod;

constexpr double kPi = std::numbers::pi;

std::vector<std::complex<float>> tone(std::size_t nSamples, double frequency, double sampleRate, double amplitude = 1.0) {
    std::vector<std::complex<float>> signal(nSamples);
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        const double phase = 2.0 * kPi * frequency * static_cast<double>(i) / sampleRate;
        signal[i]          = std::complex<float>(static_cast<float>(amplitude * std::cos(phase)), static_cast<float>(amplitude * std::sin(phase)));
    }
    return signal;
}

QuadratureDemod<float> makeDemod(float gain) {
    QuadratureDemod<float> demod({{"gain", gain}});
    demod.settings().init();
    std::ignore = demod.settings().applyStagedParameters();
    return demod;
}

void demodulateInto(QuadratureDemod<float>& demod, std::span<const std::complex<float>> input, std::span<float> output, std::size_t chunkSize = 0UZ) {
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    for (std::size_t base = 0UZ; base < input.size(); base += stride) {
        const std::size_t count = std::min(stride, input.size() - base);
        std::ignore             = demod.processBulk(input.subspan(base, count), output.subspan(base, count));
    }
}

std::vector<float> demodulate(const std::vector<std::complex<float>>& input, float gain, std::size_t chunkSize = 0UZ) {
    QuadratureDemod<float> demod = makeDemod(gain);
    std::vector<float>     output(input.size());
    demodulateInto(demod, input, output, chunkSize);
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

const boost::ut::suite<"QuadratureDemod"> quadratureDemodTests = [] {
    using namespace boost::ut;

    "constant-frequency tone fixes scale and sign"_test = [] {
        constexpr float          gain     = static_cast<float>(4.0 / kPi);
        const std::vector<float> positive = demodulate(tone(64UZ, 1000.0, 8000.0), gain);
        const std::vector<float> negative = demodulate(tone(64UZ, -1000.0, 8000.0), gain);

        expect(eq(positive[0], 0.0f)) << "first output sample must be exactly zero";
        expect(eq(negative[0], 0.0f)) << "first output sample must be exactly zero";
        for (std::size_t i = 1UZ; i < positive.size(); ++i) {
            expect(approx(positive[i], 1.0f, 1e-5f)) << "positive offset at i=" << i;
            expect(approx(negative[i], -1.0f, 1e-5f)) << "negative offset at i=" << i;
        }
    };

    "round trip against an accumulated-phase modulator"_test = [] {
        constexpr double      sensitivity = 0.5;
        constexpr std::size_t nSamples    = 512UZ;

        std::vector<float>               source(nSamples);
        std::vector<std::complex<float>> modulated(nSamples);
        double                           phase = 0.0;
        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            source[i] = static_cast<float>(0.7 * std::sin(2.0 * kPi * 3.0 * static_cast<double>(i) / static_cast<double>(nSamples)));
            phase += sensitivity * static_cast<double>(source[i]);
            modulated[i] = std::complex<float>(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
        }

        const std::vector<float> recovered = demodulate(modulated, static_cast<float>(1.0 / sensitivity));
        for (std::size_t i = 1UZ; i < nSamples; ++i) {
            expect(approx(recovered[i], source[i], 1e-5f)) << "round trip at i=" << i;
        }
    };

    "output is invariant under input amplitude"_test = [] {
        const std::vector<float> unity = demodulate(tone(256UZ, 700.0, 8000.0, 1.0), 1.0f);
        const std::vector<float> tiny  = demodulate(tone(256UZ, 700.0, 8000.0, 1e-3), 1.0f);
        const std::vector<float> huge  = demodulate(tone(256UZ, 700.0, 8000.0, 1e+3), 1.0f);

        for (std::size_t i = 1UZ; i < unity.size(); ++i) {
            expect(approx(tiny[i], unity[i], 1e-5f)) << "1e-3 scaling at i=" << i;
            expect(approx(huge[i], unity[i], 1e-5f)) << "1e+3 scaling at i=" << i;
        }
    };

    "zero input produces zero output"_test = [] {
        const std::vector<std::complex<float>> silence(128UZ, std::complex<float>{});
        const std::vector<float>               output = demodulate(silence, 3.0f);

        for (std::size_t i = 0UZ; i < output.size(); ++i) {
            expect(eq(output[i], 0.0f)) << "silence at i=" << i;
            expect(!std::isnan(output[i])) << "NaN at i=" << i;
        }
    };

    "phase advance either side of pi wraps and is never unwrapped"_test = [] {
        const std::vector<float> belowPi  = demodulate(tone(32UZ, 0.4999, 1.0), 1.0f);
        const std::vector<float> abovePi  = demodulate(tone(32UZ, 0.5001, 1.0), 1.0f);
        const auto               expected = static_cast<float>(0.9998 * kPi);

        for (std::size_t i = 1UZ; i < belowPi.size(); ++i) {
            expect(approx(belowPi[i], expected, 1e-3f)) << "just below pi at i=" << i;
            expect(approx(abovePi[i], -expected, 1e-3f)) << "just above pi at i=" << i;
            expect(le(std::abs(abovePi[i]), static_cast<float>(kPi))) << "output must stay inside (-pi, pi]";
        }
    };

    "output does not depend on chunking"_test = [] {
        const std::vector<std::complex<float>> input     = tone(5000UZ, 137.0, 8000.0);
        const std::vector<float>               reference = demodulate(input, 1.5f);

        for (const std::size_t chunkSize : {1UZ, 3UZ, 17UZ, 4096UZ}) {
            const std::vector<float> chunked = demodulate(input, 1.5f, chunkSize);
            expect(std::ranges::equal(chunked, reference)) << "chunk size " << chunkSize << " must be bit-identical";
        }
    };

    "gain is live and does not disturb the phase history"_test = [] {
        const std::vector<std::complex<float>> input = tone(64UZ, 900.0, 8000.0);
        constexpr std::size_t                  split = 32UZ;

        QuadratureDemod<float> fixedGain   = makeDemod(1.0f);
        QuadratureDemod<float> changedGain = makeDemod(1.0f);

        std::vector<float> reference(input.size());
        std::vector<float> retuned(input.size());
        demodulateInto(fixedGain, std::span(input).first(split), std::span(reference).first(split));
        demodulateInto(changedGain, std::span(input).first(split), std::span(retuned).first(split));
        expect(std::ranges::equal(std::span(reference).first(split), std::span(retuned).first(split)));

        std::ignore = changedGain.settings().setStaged({{"gain", 2.0f}});
        std::ignore = changedGain.settings().applyStagedParameters();

        const std::size_t remaining = input.size() - split;
        demodulateInto(fixedGain, std::span(input).subspan(split, remaining), std::span(reference).subspan(split, remaining));
        demodulateInto(changedGain, std::span(input).subspan(split, remaining), std::span(retuned).subspan(split, remaining));
        for (std::size_t i = split; i < input.size(); ++i) {
            expect(eq(retuned[i], 2.0f * reference[i])) << "gain change must scale exactly at i=" << i;
        }
    };

    "a non-reserved tag key rides through at its own offset"_test = [] {
        const std::vector<std::size_t> offsets = privateTagOffsets<std::complex<float>, float, QuadratureDemod<float>>({{"gain", 1.0f}});
        expect(that % (offsets == std::vector<std::size_t>{7UZ, 300UZ, 1000UZ})) << "the pass-all policy keeps a key the auto-forward set does not name";
    };
};

int main() { /* not needed for ut */ }
