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
#include <gnuradio-4.0/analog/Agc.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace {

using gr::blocks::analog::Agc;

constexpr double kPi = std::numbers::pi;

template<typename T>
Agc<T> makeAgc(gr::property_map settings) {
    Agc<T> agc(std::move(settings));
    agc.settings().init();
    std::ignore = agc.settings().applyStagedParameters();
    return agc;
}

template<typename T>
void runAgc(Agc<T>& agc, std::span<const T> input, std::span<T> output, std::size_t chunkSize = 0UZ) {
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    for (std::size_t base = 0UZ; base < input.size(); base += stride) {
        const std::size_t count = std::min(stride, input.size() - base);
        std::ignore             = agc.processBulk(input.subspan(base, count), output.subspan(base, count));
    }
}

std::vector<std::complex<float>> phasor(std::size_t nSamples, double magnitude, double normalizedFrequency = 1000.0 / 48000.0) {
    std::vector<std::complex<float>> signal(nSamples);
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        const double phase = 2.0 * kPi * normalizedFrequency * static_cast<double>(i);
        signal[i]          = std::complex<float>(static_cast<float>(magnitude * std::cos(phase)), static_cast<float>(magnitude * std::sin(phase)));
    }
    return signal;
}

std::vector<float> sine(std::size_t nSamples, double amplitude, double normalizedFrequency = 1000.0 / 48000.0) {
    std::vector<float> signal(nSamples);
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        signal[i] = static_cast<float>(amplitude * std::sin(2.0 * kPi * normalizedFrequency * static_cast<double>(i)));
    }
    return signal;
}

std::vector<std::complex<float>> processPhasor(Agc<std::complex<float>>& agc, std::size_t nSamples, double magnitude) {
    const std::vector<std::complex<float>> input = phasor(nSamples, magnitude);
    std::vector<std::complex<float>>       output(nSamples);
    runAgc(agc, std::span<const std::complex<float>>(input), std::span<std::complex<float>>(output));
    return output;
}

std::vector<float> processSine(Agc<float>& agc, std::size_t nSamples, double amplitude) {
    const std::vector<float> input = sine(nSamples, amplitude);
    std::vector<float>       output(nSamples);
    runAgc(agc, std::span<const float>(input), std::span<float>(output));
    return output;
}

// index, after a magnitude step, at which the output level error has decayed to 20/e dB
std::size_t stepSettlingIndex(float sampleRate, double attackSeconds, double decaySeconds, double startMagnitude, double steppedMagnitude) {
    Agc<std::complex<float>> agc = makeAgc<std::complex<float>>({{"sample_rate", sampleRate}, {"reference_db", 0.0}, {"attack_s", attackSeconds}, {"decay_s", decaySeconds}, {"max_gain_db", 120.0}, {"min_gain_db", -120.0}});

    const std::size_t settleCount = static_cast<std::size_t>(20.0 * std::max(attackSeconds, decaySeconds) * static_cast<double>(sampleRate));
    std::ignore                   = processPhasor(agc, settleCount, startMagnitude);

    const std::vector<std::complex<float>> stepped = processPhasor(agc, settleCount, steppedMagnitude);
    const double                           target  = 20.0 / std::numbers::e;
    for (std::size_t i = 0UZ; i < stepped.size(); ++i) {
        if (std::abs(20.0 * std::log10(static_cast<double>(std::abs(stepped[i])))) <= target) {
            return i;
        }
    }
    return stepped.size();
}

double peakDecibel(std::span<const float> samples) {
    float peak = 0.f;
    for (const float sample : samples) {
        peak = std::max(peak, std::abs(sample));
    }
    return 20.0 * std::log10(std::max(static_cast<double>(peak), 1e-30));
}

double rmsDecibel(std::span<const float> samples) {
    double sum = 0.0;
    for (const float sample : samples) {
        sum += static_cast<double>(sample) * static_cast<double>(sample);
    }
    return 10.0 * std::log10(std::max(sum / static_cast<double>(samples.size()), 1e-30));
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

const boost::ut::suite<"Agc"> agcTests = [] {
    using namespace boost::ut;

    "the inline log2 and exp2 match the library within tolerance"_test = [] {
        for (double exponent = -24.0; exponent <= 24.0; exponent += 0.37) {
            const float value = static_cast<float>(std::pow(10.0, exponent));
            expect(lt(std::abs(gr::blocks::analog::detail::fastLog2(value) - static_cast<float>(std::log2(static_cast<double>(value)))), 1e-5f)) << "fastLog2 at 1e" << exponent;
        }
        for (double exponent = -40.0; exponent <= 40.0; exponent += 0.13) {
            const double exact = std::exp2(exponent);
            expect(lt(std::abs(static_cast<double>(gr::blocks::analog::detail::fastExp2(static_cast<float>(exponent))) - exact) / exact, 4e-6)) << "fastExp2 at " << exponent;
        }
    };

    "the loop settles at reference_db for any input level"_test = [] {
        constexpr double  decaySeconds = 0.05;
        const std::size_t nSamples     = static_cast<std::size_t>(10.0 * decaySeconds * 48000.0);

        for (const double magnitude : {1.0, 0.01, 10.0}) {
            Agc<std::complex<float>> agc = makeAgc<std::complex<float>>({{"sample_rate", 48000.f}, {"reference_db", 0.0}, {"attack_s", 0.005}, {"decay_s", decaySeconds}, {"max_gain_db", 120.0}, {"min_gain_db", -120.0}});

            const std::vector<std::complex<float>> output = processPhasor(agc, nSamples, magnitude);
            expect(approx(static_cast<double>(std::abs(output.back())), 1.0, 1e-3)) << "settled magnitude at m=" << magnitude;
            expect(approx(agc.gain_db.value, -20.0 * std::log10(magnitude), 0.01)) << "settled gain at m=" << magnitude;
        }
    };

    "the decay time constant is the configured one"_test = [] {
        constexpr double decaySeconds = 0.05;
        const double     expected     = decaySeconds * 48000.0;
        const auto       measured     = static_cast<double>(stepSettlingIndex(48000.f, 0.005, decaySeconds, 1.0, 0.1));
        expect(lt(std::abs(measured - expected) / expected, 0.05)) << "decay settling at index " << measured << " vs " << expected;
    };

    "the attack time constant is the configured one"_test = [] {
        constexpr double attackSeconds = 0.01;
        const double     expected      = attackSeconds * 48000.0;
        const auto       measured      = static_cast<double>(stepSettlingIndex(48000.f, attackSeconds, 0.05, 1.0, 10.0));
        expect(lt(std::abs(measured - expected) / expected, 0.05)) << "attack settling at index " << measured << " vs " << expected;
    };

    "the settling time does not depend on the signal level"_test = [] {
        const auto loud  = static_cast<double>(stepSettlingIndex(48000.f, 0.005, 0.05, 1.0, 0.1));
        const auto quiet = static_cast<double>(stepSettlingIndex(48000.f, 0.005, 0.05, 0.001, 0.0001));
        expect(lt(std::abs(loud - quiet) / loud, 0.02)) << "a 60 dB level change moved the time constant: " << loud << " vs " << quiet;
    };

    "the settling time does not depend on the sample rate"_test = [] {
        const double atLowRate  = static_cast<double>(stepSettlingIndex(48000.f, 0.005, 0.05, 1.0, 0.1)) / 48000.0;
        const double atHighRate = static_cast<double>(stepSettlingIndex(192000.f, 0.005, 0.05, 1.0, 0.1)) / 192000.0;
        expect(lt(std::abs(atLowRate - atHighRate) / atLowRate, 0.01)) << "settling times " << atLowRate << " s vs " << atHighRate << " s";
    };

    "the gain never leaves the clamps"_test = [] {
        Agc<std::complex<float>>         rising = makeAgc<std::complex<float>>({{"sample_rate", 48000.f}, {"reference_db", 0.0}, {"decay_s", 0.01}, {"max_gain_db", 6.0}, {"min_gain_db", -20.0}});
        const auto                       quiet  = phasor(20000UZ, 1e-6);
        std::vector<std::complex<float>> risingOut(quiet.size());
        for (std::size_t base = 0UZ; base < quiet.size(); base += 250UZ) {
            std::ignore = rising.processBulk(std::span<const std::complex<float>>(quiet).subspan(base, 250UZ), std::span<std::complex<float>>(risingOut).subspan(base, 250UZ));
            expect(le(rising.gain_db.value, 6.0)) << "gain exceeded max_gain_db at " << base;
        }
        expect(eq(rising.gain_db.value, 6.0)) << "the gain must rest exactly on the clamp";

        Agc<std::complex<float>>         falling = makeAgc<std::complex<float>>({{"sample_rate", 48000.f}, {"reference_db", 0.0}, {"attack_s", 0.01}, {"max_gain_db", 60.0}, {"min_gain_db", -6.0}});
        const auto                       loud    = phasor(20000UZ, 1e6);
        std::vector<std::complex<float>> fallingOut(loud.size());
        for (std::size_t base = 0UZ; base < loud.size(); base += 250UZ) {
            std::ignore = falling.processBulk(std::span<const std::complex<float>>(loud).subspan(base, 250UZ), std::span<std::complex<float>>(fallingOut).subspan(base, 250UZ));
            expect(ge(falling.gain_db.value, -6.0)) << "gain fell below min_gain_db at " << base;
        }
        expect(eq(falling.gain_db.value, -6.0)) << "the gain must rest exactly on the clamp";
    };

    "the gate freezes the gain below its threshold"_test = [] {
        Agc<std::complex<float>> agc = makeAgc<std::complex<float>>({{"sample_rate", 48000.f}, {"reference_db", 0.0}, {"decay_s", 0.05}, {"gate_threshold_db", -40.0}, {"max_gain_db", 120.0}, {"min_gain_db", -120.0}});

        std::ignore = processPhasor(agc, 100000UZ, 1e-3);
        expect(eq(agc.gain_db.value, 0.0)) << "a -60 dB input must not move a gate set at -40 dB";

        const std::complex<float> audible{0.1f, 0.f};
        std::complex<float>       demoted{};
        std::ignore = agc.processBulk(std::span<const std::complex<float>>(&audible, 1UZ), std::span<std::complex<float>>(&demoted, 1UZ));
        expect(gt(agc.gain_db.value, 0.0)) << "the loop must move on the first sample above the gate";
    };

    "manual mode applies gain_db exactly and leaves it alone"_test = [] {
        Agc<std::complex<float>> agc = makeAgc<std::complex<float>>({{"enabled", false}, {"gain_db", 12.0}});

        const auto                       input = phasor(64UZ, 0.3);
        std::vector<std::complex<float>> output(input.size());
        runAgc(agc, std::span<const std::complex<float>>(input), std::span<std::complex<float>>(output));

        const auto expectedGain = static_cast<float>(std::pow(10.0, 0.6));
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            expect(eq(output[i].real(), input[i].real() * expectedGain)) << "manual gain at i=" << i;
            expect(eq(output[i].imag(), input[i].imag() * expectedGain)) << "manual gain at i=" << i;
        }
        expect(eq(agc.gain_db.value, 12.0)) << "manual mode must not move the gain";
    };

    "switching to manual mode is bumpless"_test = [] {
        constexpr std::size_t    settleCount = 20000UZ;
        Agc<std::complex<float>> agc         = makeAgc<std::complex<float>>({{"sample_rate", 48000.f}, {"reference_db", 0.0}, {"decay_s", 0.01}, {"attack_s", 0.001}});

        const auto                       input = phasor(settleCount + 8UZ, 1.0);
        std::vector<std::complex<float>> output(input.size());
        runAgc(agc, std::span<const std::complex<float>>(input).first(settleCount), std::span<std::complex<float>>(output).first(settleCount));

        std::ignore = agc.settings().setStaged({{"enabled", false}});
        std::ignore = agc.settings().applyStagedParameters();
        runAgc(agc, std::span<const std::complex<float>>(input).subspan(settleCount, 8UZ), std::span<std::complex<float>>(output).subspan(settleCount, 8UZ));

        const float outputStep = std::abs(output[settleCount] - output[settleCount - 1UZ]);
        const float inputStep  = std::abs(input[settleCount] - input[settleCount - 1UZ]);
        expect(le(outputStep, inputStep + 1e-4f)) << "mode switch stepped the output by " << outputStep << " against an input step of " << inputStep;
    };

    "output does not depend on chunking"_test = [] {
        std::vector<std::complex<float>> input(5000UZ);
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            const double magnitude = 0.2 + 0.8 * std::abs(std::sin(2.0 * kPi * 3.0 * static_cast<double>(i) / static_cast<double>(input.size())));
            const double phase     = 2.0 * kPi * 0.031 * static_cast<double>(i);
            input[i]               = std::complex<float>(static_cast<float>(magnitude * std::cos(phase)), static_cast<float>(magnitude * std::sin(phase)));
        }

        const gr::property_map           settings = {{"sample_rate", 48000.f}, {"reference_db", -6.0}, {"decay_s", 0.02}, {"attack_s", 0.002}};
        Agc<std::complex<float>>         agc      = makeAgc<std::complex<float>>(settings);
        std::vector<std::complex<float>> reference(input.size());
        runAgc(agc, std::span<const std::complex<float>>(input), std::span<std::complex<float>>(reference));

        for (const std::size_t chunkSize : {1UZ, 3UZ, 17UZ, 4096UZ}) {
            Agc<std::complex<float>>         chunkedAgc = makeAgc<std::complex<float>>(settings);
            std::vector<std::complex<float>> chunked(input.size());
            runAgc(chunkedAgc, std::span<const std::complex<float>>(input), std::span<std::complex<float>>(chunked), chunkSize);
            expect(std::ranges::equal(chunked, reference)) << "chunk size " << chunkSize << " must be bit-identical";
        }
    };

    "silence stays silent and finite"_test = [] {
        Agc<std::complex<float>> agc = makeAgc<std::complex<float>>({{"sample_rate", 48000.f}});

        const std::vector<std::complex<float>> silence(5000UZ, std::complex<float>{});
        std::vector<std::complex<float>>       output(silence.size());
        runAgc(agc, std::span<const std::complex<float>>(silence), std::span<std::complex<float>>(output));

        bool allZero = true;
        for (const std::complex<float>& sample : output) {
            allZero = allZero && sample.real() == 0.f && sample.imag() == 0.f && std::isfinite(sample.real()) && std::isfinite(sample.imag());
        }
        expect(allZero) << "an all-zero input must produce an all-zero, finite output";
        expect(ge(agc.gain_db.value, -20.0));
        expect(le(agc.gain_db.value, 60.0));
    };

    "the real instantiation regulates the peak, not the RMS"_test = [] {
        constexpr double  decaySeconds = 0.05;
        const std::size_t nSamples     = static_cast<std::size_t>(10.0 * decaySeconds * 48000.0);
        const std::size_t oneCycle     = 48UZ;

        Agc<float>               loud     = makeAgc<float>({{"sample_rate", 48000.f}, {"reference_db", 0.0}, {"attack_s", 0.001}, {"decay_s", decaySeconds}, {"gate_threshold_db", -40.0}, {"max_gain_db", 120.0}, {"min_gain_db", -120.0}});
        const std::vector<float> loudOut  = processSine(loud, nSamples, 1.0);
        const auto               loudTail = std::span<const float>(loudOut).last(oneCycle);
        expect(lt(std::abs(peakDecibel(loudTail)), 1.0)) << "the settled peak must sit at the reference, measured " << peakDecibel(loudTail) << " dB";
        expect(lt(rmsDecibel(loudTail), peakDecibel(loudTail) - 1.5)) << "an RMS detector would have settled the RMS at the reference instead";

        Agc<float>               quiet     = makeAgc<float>({{"sample_rate", 48000.f}, {"reference_db", 0.0}, {"attack_s", 0.001}, {"decay_s", decaySeconds}, {"gate_threshold_db", -80.0}, {"max_gain_db", 120.0}, {"min_gain_db", -120.0}});
        const std::vector<float> quietOut  = processSine(quiet, nSamples, 0.01);
        const auto               quietTail = std::span<const float>(quietOut).last(oneCycle);
        expect(lt(std::abs(peakDecibel(quietTail) - peakDecibel(loudTail)), 0.05)) << "a 40 dB quieter sinusoid must settle at the same peak level";

        Agc<float> stepping                 = makeAgc<float>({{"sample_rate", 48000.f}, {"reference_db", 0.0}, {"attack_s", 0.001}, {"decay_s", decaySeconds}, {"gate_threshold_db", -60.0}, {"max_gain_db", 120.0}, {"min_gain_db", -120.0}});
        std::ignore                         = processSine(stepping, nSamples, 1.0);
        const std::vector<float> steppedOut = processSine(stepping, nSamples, 0.1);
        const std::size_t        oneTau     = static_cast<std::size_t>(decaySeconds * 48000.0);
        expect(gt(peakDecibel(std::span<const float>(steppedOut).subspan(oneTau, oneCycle)), -10.0)) << "most of the 20 dB step must be recovered within one decay_s";
        expect(lt(std::abs(peakDecibel(std::span<const float>(steppedOut).subspan(3UZ * oneTau, oneCycle)) - peakDecibel(loudTail)), 1.0)) << "the peak must be back at the reference by three decay_s";
    };

    "unusable parameters are rejected at settings time"_test = [] {
        expect(throws([] { std::ignore = makeAgc<std::complex<float>>({{"sample_rate", 0.f}}); })) << "zero sample_rate";
        expect(throws([] { std::ignore = makeAgc<std::complex<float>>({{"attack_s", 0.0}}); })) << "zero attack_s";
        expect(throws([] { std::ignore = makeAgc<std::complex<float>>({{"decay_s", -1.0}}); })) << "negative decay_s";
        expect(throws([] { std::ignore = makeAgc<std::complex<float>>({{"max_gain_db", -30.0}, {"min_gain_db", -20.0}}); })) << "inverted clamps";
        expect(throws([] { std::ignore = makeAgc<std::complex<float>>({{"update_decimation", gr::Size_t(0)}}); })) << "zero update_decimation";
    };

    "a non-reserved tag key rides through at its own offset"_test = [] {
        const std::vector<std::size_t> offsets = privateTagOffsets<float, float, Agc<float>>({{"sample_rate", 48000.f}});
        expect(that % (offsets == std::vector<std::size_t>{7UZ, 300UZ, 1000UZ})) << "the pass-all policy keeps a key the auto-forward set does not name";
    };
};

int main() { /* not needed for ut */ }
