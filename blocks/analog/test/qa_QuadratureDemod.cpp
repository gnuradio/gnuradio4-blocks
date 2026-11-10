#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <numbers>
#include <random>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/analog/QuadratureDemod.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

// The tolerances of the cases that compare against a reference vector are set by float rounding of the complex
// product, not by the arctangent.
// A reference vector produced by a 256-entry interpolated arctangent table carries that table's error
// against an exact atan2, 6.1e-7 rad mean and 1.31e-6 rad maximum, so comparing against one needs a
// per-sample tolerance of gain * 3e-6. Asserting a tighter agreement would be asserting that this
// block reproduced the table, which it must not.
//
// The arctangent itself is gr::math::scaledAtan2, within 4 ulp of the exact result (a bit-equal match to std::atan2
// is not promised; qa_Atan2 pins the primitive). The cases at the end measure the block against a long double
// reference on inputs whose products are exact in the sample type, split streams and the FM residual.

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

/// @brief |value - reference| in units of the spacing of T at the reference; zero when both are the same zero.
template<typename T>
double errorInUlp(T value, long double reference) {
    const T rounded = static_cast<T>(reference);
    if (rounded == T(0)) {
        return value == T(0) ? 0.0 : std::numeric_limits<double>::infinity();
    }
    const T spacing = std::nextafter(std::abs(rounded), std::numeric_limits<T>::infinity()) - std::abs(rounded);
    return static_cast<double>(std::fabs(static_cast<long double>(value) - reference) / static_cast<long double>(spacing));
}

/// @brief The block's per-sample form: std::atan2 on the same products.
std::vector<float> demodulateWithStdAtan2(const std::vector<std::complex<float>>& input) {
    std::vector<float>  output(input.size());
    std::complex<float> previous{};
    for (std::size_t i = 0UZ; i < input.size(); ++i) {
        const float real = input[i].real() * previous.real() + input[i].imag() * previous.imag();
        const float imag = input[i].imag() * previous.real() - input[i].real() * previous.imag();
        output[i]        = std::atan2(imag, real);
        previous         = input[i];
    }
    return output;
}

/// @brief 20 log10 of the residual's rms over the reference's rms.
double residualDb(const std::vector<float>& measured, const std::vector<double>& reference) {
    double residual = 0.0;
    double signal   = 0.0;
    for (std::size_t i = 1UZ; i < reference.size(); ++i) {
        const double error = static_cast<double>(measured[i]) - reference[i];
        residual += error * error;
        signal += reference[i] * reference[i];
    }
    return 10.0 * std::log10(residual / signal);
}

/// @brief Frequency modulation of one tone, rounded to float as a receiver's input is; @p reference is the instantaneous
/// frequency recovered in double from those float samples.
std::vector<std::complex<float>> fmTone(std::size_t nSamples, double sampleRate, double deviation, double toneHz, std::vector<double>& reference) {
    std::vector<std::complex<float>> signal(nSamples);
    reference.assign(nSamples, 0.0);
    double phase = 0.0;
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        phase += 2.0 * kPi * deviation / sampleRate * std::sin(2.0 * kPi * toneHz * static_cast<double>(i) / sampleRate);
        signal[i] = std::complex<float>(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
    }
    for (std::size_t i = 1UZ; i < nSamples; ++i) {
        const std::complex<double> a(static_cast<double>(signal[i].real()), static_cast<double>(signal[i].imag()));
        const std::complex<double> b(static_cast<double>(signal[i - 1UZ].real()), static_cast<double>(signal[i - 1UZ].imag()));
        const std::complex<double> product = a * std::conj(b);
        reference[i]                       = std::atan2(product.imag(), product.real());
    }
    return signal;
}

/// @brief Where a tag key outside the `gr:` domain lands after @p TBlock, as the sink sees it.
template<typename TIn, typename TOut, typename TBlock>
[[nodiscard]] std::vector<std::size_t> privateTagOffsets(gr::property_map settings) {
    using gr::blocks::testing::ProcessFunction;
    using gr::blocks::testing::TagSink;
    using gr::blocks::testing::TagSource;

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

/// @brief Test values between samples of (1, 0), so that each product is a test value or its conjugate, exact: every
/// pair of parts from the signed zeros, the smallest subnormal, one, the smallest and largest normals, the infinities
/// and a NaN, then random bit patterns and Gaussian noise.
[[nodiscard]] std::vector<std::complex<float>> productTestInput() {
    using L                                = std::numeric_limits<float>;
    const std::array<float, 13UZ>    parts = {0.f, -0.f, L::denorm_min(), -L::denorm_min(), L::min(), -L::min(), 1.f, -1.f, L::max(), -L::max(), L::infinity(), -L::infinity(), L::quiet_NaN()};
    std::vector<std::complex<float>> values;
    for (const float re : parts) {
        for (const float im : parts) {
            values.emplace_back(re, im);
        }
    }
    std::mt19937                                 rng(20261103U);
    std::uniform_int_distribution<std::uint32_t> bits;
    std::normal_distribution<float>              noise(0.f, 0.3f);
    for (std::size_t i = 0UZ; i < 2048UZ; ++i) {
        values.emplace_back(std::bit_cast<float>(bits(rng)), std::bit_cast<float>(bits(rng)));
        values.emplace_back(noise(rng), noise(rng));
    }
    std::vector<std::complex<float>> input;
    for (const std::complex<float> value : values) {
        input.emplace_back(1.f, 0.f);
        input.push_back(value);
    }
    return input;
}

/// @brief True for two equal values of the same sign and for two NaNs.
[[nodiscard]] bool sameValue(double a, double b) { return (std::isnan(a) && std::isnan(b)) || (a == b && std::signbit(a) == std::signbit(b)); }

/// @brief One float ulp at the magnitude of `value`.
[[nodiscard]] double floatUlp(double value) {
    using L       = std::numeric_limits<float>;
    const int exp = std::max(std::ilogb(value), L::min_exponent - 1);
    return std::ldexp(1.0, exp - (L::digits - 1));
}

static_assert(QuadratureDemod<float>::keepsTagOffsets(), "keeps tag offsets: no Resampling<> or Stride<>, no asynchronous port, no tag policy and no forwardTags() override");

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

    // gr::meta::atan2 is within 2 ulp of the exact arctangent; the double std::atan2 of the product stands for the
    // exact value, its own error far below a float ulp
    "each output is within 2 ulp of std::atan2 of its product, and a zero product gives 0, at every chunk size"_test = [] {
        const std::vector<std::complex<float>> input = productTestInput();
        const std::vector<float>               whole = demodulate(input, 1.0f);

        std::size_t         outside = 0UZ;
        std::complex<float> prior{};
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            const std::complex<float> sample = input[i];
            const float               real   = sample.real() * prior.real() + sample.imag() * prior.imag();
            const float               imag   = sample.imag() * prior.real() - sample.real() * prior.imag();
            prior                            = sample;
            const double output              = static_cast<double>(whole[i]);
            if (real == 0.f && imag == 0.f) {
                outside += static_cast<std::size_t>(!sameValue(output, 0.0));
                continue;
            }
            const double reference = std::atan2(static_cast<double>(imag), static_cast<double>(real));
            if (std::isnan(reference) || reference == 0.0) {
                outside += static_cast<std::size_t>(!sameValue(output, reference));
            } else {
                outside += static_cast<std::size_t>(!(std::abs(output - reference) <= 2.0 * floatUlp(reference)));
            }
        }
        expect(eq(outside, 0UZ)) << "outputs outside the bound";

        std::size_t chunkMismatches = 0UZ;
        for (const std::size_t chunkSize : {1UZ, 3UZ, 255UZ, 257UZ}) {
            const std::vector<float> chunked = demodulate(input, 1.0f, chunkSize);
            for (std::size_t i = 0UZ; i < input.size(); ++i) {
                chunkMismatches += static_cast<std::size_t>(!sameValue(static_cast<double>(chunked[i]), static_cast<double>(whole[i])));
            }
        }
        expect(eq(chunkMismatches, 0UZ)) << "chunked outputs that differ from one call";
    };

    "the first output after a reset is 0 for a sample in every quadrant"_test = [] {
        for (const std::complex<float> first : {std::complex<float>{1.f, 1.f}, std::complex<float>{-1.f, 1.f}, std::complex<float>{-1.f, -1.f}, std::complex<float>{1.f, -1.f}}) {
            const std::vector<float> output = demodulate({first, first}, 2.0f);
            expect(eq(output[0], 0.0f) && !std::signbit(output[0])) << "first output for " << first.real() << ", " << first.imag();
        }
    };

    "a non-reserved tag key rides through at its own offset"_test = [] {
        const std::vector<std::size_t> offsets = privateTagOffsets<std::complex<float>, float, QuadratureDemod<float>>({{"gain", 1.0f}});
        expect(that % (offsets == std::vector<std::size_t>{7UZ, 300UZ, 1000UZ})) << "the default forwarder keeps a key outside the gr: domain";
    };

    "the block's arctangent on exact products stays within 4 ulp across scales"_test = [] {
        std::mt19937                       rng(777);
        std::uniform_int_distribution<int> digit(-1023, 1023);
        for (const int scaleExponent : {0, 30, -40, -70}) {
            const float                          scale = std::ldexp(1.0f, scaleExponent);
            std::vector<std::complex<float>>     input(4096UZ);
            std::vector<std::complex<long long>> exact(input.size());
            for (std::size_t i = 0UZ; i < input.size(); ++i) {
                exact[i] = {digit(rng), digit(rng)};
                input[i] = {static_cast<float>(exact[i].real()) * scale, static_cast<float>(exact[i].imag()) * scale};
            }
            input[100] = {0.0f, 0.0f}; // the axes and zero
            exact[100] = {0, 0};
            input[200] = {scale, 0.0f};
            exact[200] = {1, 0};
            input[201] = {scale, 0.0f};
            exact[201] = {1, 0};
            input[202] = {-scale, 0.0f};
            exact[202] = {-1, 0};
            input[203] = {0.0f, scale};
            exact[203] = {0, 1};

            const std::vector<float> output = demodulate(input, 1.0f);
            double                   worst  = 0.0;
            for (std::size_t i = 1UZ; i < input.size(); ++i) {
                const long long productReal = exact[i].real() * exact[i - 1UZ].real() + exact[i].imag() * exact[i - 1UZ].imag();
                const long long productImag = exact[i].imag() * exact[i - 1UZ].real() - exact[i].real() * exact[i - 1UZ].imag();
                if (productReal == 0 && productImag == 0) { // a zero product has a sign the integers lack
                    continue;
                }
                worst = std::max(worst, errorInUlp(output[i], std::atan2(static_cast<long double>(productImag), static_cast<long double>(productReal))));
            }
            boost::ut::log << "exact products at scale 2^" << scaleExponent << ": largest error " << worst << " ulp\n";
            expect(le(worst, 4.0)) << "largest error in ulp at sample scale 2^" << scaleExponent;
        }
    };

    "a split stream gives the values of one call"_test = [] {
        std::mt19937                     rng(4242);
        std::normal_distribution<float>  noise(0.0f, 1.0f);
        std::vector<std::complex<float>> input = tone(3001UZ, 137.0, 8000.0);
        for (auto& sample : input) {
            sample += std::complex<float>(0.3f * noise(rng), 0.3f * noise(rng));
        }
        const std::vector<float> whole = demodulate(input, 1.5f);
        for (const std::size_t chunkSize : {1UZ, 7UZ, 256UZ, 257UZ, 1000UZ}) {
            expect(std::ranges::equal(demodulate(input, 1.5f, chunkSize), whole)) << "chunk size " << chunkSize << " must be bit-identical";
        }

        QuadratureDemod<double> demod({{"gain", 1.5}});
        demod.settings().init();
        std::ignore = demod.settings().applyStagedParameters();
        std::vector<std::complex<double>> wide(input.begin(), input.end());
        std::vector<double>               wholeWide(wide.size());
        std::ignore = demod.processBulk(wide, wholeWide);
        for (const std::size_t chunkSize : {1UZ, 7UZ, 256UZ, 257UZ, 1000UZ}) {
            QuadratureDemod<double> chunked({{"gain", 1.5}});
            chunked.settings().init();
            std::ignore = chunked.settings().applyStagedParameters();
            std::vector<double> pieces(wide.size());
            for (std::size_t base = 0UZ; base < wide.size(); base += chunkSize) {
                const std::size_t count = std::min(chunkSize, wide.size() - base);
                std::ignore             = chunked.processBulk(std::span<const std::complex<double>>(wide).subspan(base, count), std::span<double>(pieces).subspan(base, count));
            }
            expect(std::ranges::equal(pieces, wholeWide)) << "double, chunk size " << chunkSize << " must be bit-identical";
        }
    };

    "an FM tone's residual against the double reference is far below the tone"_test = [] {
        struct Case {
            const char* name;
            double      sampleRate;
            double      deviation;
            double      toneHz;
        };
        for (const Case& fm : {Case{"narrowband", 48000.0, 5000.0, 1000.0}, Case{"wideband", 240000.0, 75000.0, 15000.0}}) {
            std::vector<double>                    reference;
            const std::vector<std::complex<float>> input   = fmTone(48000UZ, fm.sampleRate, fm.deviation, fm.toneHz, reference);
            const double                           changed = residualDb(demodulate(input, 1.0f), reference);
            const double                           before  = residualDb(demodulateWithStdAtan2(input), reference);
            boost::ut::log << fm.name << " FM residual: " << before << " dB with std::atan2, " << changed << " dB with scaledAtan2\n";
            expect(le(changed, -120.0)) << fm.name << ": residual below the tone";
        }
    };
};

int main() { /* not needed for ut */ }
