#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <limits>
#include <memory>
#include <numbers>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/YamlPmt.hpp>

#include <gnuradio-4.0/basic/IqCorrection.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace {

using gr::blocks::basic::DcOffsetCorrect;
using gr::blocks::basic::IqSwap;
using gr::blocks::testing::ProcessFunction;
using gr::blocks::testing::TagSink;
using gr::blocks::testing::TagSource;

using CF = std::complex<float>;

constexpr double kPi = std::numbers::pi;

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

template<typename TBlock>
void apply(TBlock& block, gr::property_map settings) {
    std::ignore = block.settings().setStaged(std::move(settings));
    std::ignore = block.settings().applyStagedParameters();
}

/// Stages and applies `settings`. True when every key is refused.
template<typename TBlock>
[[nodiscard]] bool refused(TBlock& block, gr::property_map settings) {
    const std::size_t keys = settings.size();
    std::ignore            = block.settings().setStaged(std::move(settings));
    return block.settings().applyStagedParameters().failedParameters.size() == keys;
}

/// Drives a processOne block over `input`, one sample at a time.
template<typename TBlock>
[[nodiscard]] std::vector<CF> drive(TBlock& block, std::span<const CF> input) {
    std::vector<CF> output(input.size());
    for (std::size_t i = 0UZ; i < input.size(); ++i) {
        output[i] = block.processOne(input[i]);
    }
    return output;
}

/// The value `settings().get()` returns for `key`, or NaN when it holds none as a `double`.
template<typename TBlock>
[[nodiscard]] double reported(TBlock& block, const char* key) {
    const gr::property_map active = block.settings().get();
    const auto             found  = active.find(gr::property_map::key_type{key});
    if (found == active.end() || found->second.template get_if<double>() == nullptr) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return *found->second.template get_if<double>();
}

/// The writable settings, round-tripped through YAML.
template<typename TBlock>
[[nodiscard]] gr::property_map savedSettings(TBlock& block) {
    gr::property_map stored;
    const auto&      writable = block.settings().writableMembers();
    for (const auto& [key, value] : block.settings().get()) {
        if (writable.contains(std::string(key))) {
            stored.insert_or_assign(key, value);
        }
    }
    return gr::pmt::yaml::deserialize(gr::pmt::yaml::serialize(stored)).value_or(gr::property_map{});
}

/// A block built from `settings`, after `init()`.
template<typename TBlock>
[[nodiscard]] std::unique_ptr<TBlock> loaded(const gr::property_map& settings) {
    auto block = std::make_unique<TBlock>(settings);
    block->init(std::make_shared<gr::Sequence>());
    return block;
}

[[nodiscard]] std::vector<CF> flat(std::size_t count, CF value) { return std::vector<CF>(count, value); }

[[nodiscard]] std::vector<CF> tone(std::size_t count, double amplitude, double normalizedFrequency) {
    std::vector<CF> samples(count);
    for (std::size_t i = 0UZ; i < count; ++i) {
        const double phase = 2.0 * kPi * normalizedFrequency * static_cast<double>(i);
        samples[i]         = CF(static_cast<float>(amplitude * std::cos(phase)), static_cast<float>(amplitude * std::sin(phase)));
    }
    return samples;
}

struct Random {
    std::uint64_t state = 0x243f6a8885a308d3ULL;

    [[nodiscard]] double gaussian() noexcept { // Box-Muller over a splitmix stream, deterministic on every machine
        const double u1 = uniform();
        const double u2 = uniform();
        return std::sqrt(-2.0 * std::log(u1 + 1e-300)) * std::cos(2.0 * kPi * u2);
    }

    [[nodiscard]] double uniform() noexcept {
        state += 0x9e3779b97f4a7c15ULL;
        std::uint64_t z = state;
        z               = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z               = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        z ^= z >> 31;
        return static_cast<double>(z >> 11) * 0x1.0p-53;
    }
};

[[nodiscard]] std::vector<CF> noise(std::size_t count, std::uint64_t seed, double offset = 0.0) {
    Random          rng{seed};
    std::vector<CF> samples(count);
    for (CF& sample : samples) { // unit rms per component pair, each component with variance 1/2
        sample = CF(static_cast<float>(rng.gaussian() * std::numbers::sqrt2 / 2.0 + offset), static_cast<float>(rng.gaussian() * std::numbers::sqrt2 / 2.0 + offset));
    }
    return samples;
}

/// `tau` that yields exactly the wanted `alpha` through `alpha = 1/(1 + tau*fs)`.
[[nodiscard]] constexpr double tauFor(double alpha, double sampleRate) { return (1.0 / alpha - 1.0) / sampleRate; }

[[nodiscard]] double meanMagnitude(std::span<const CF> samples) {
    double total = 0.0;
    for (const CF& sample : samples) {
        total += std::hypot(static_cast<double>(sample.real()), static_cast<double>(sample.imag()));
    }
    return total / static_cast<double>(samples.size());
}

struct Marker {
    const char*    key;
    std::size_t    at;
    gr::pmt::Value value;
};

/// Seven keys at six offsets. `t0` is not in `gr::tag::kDefaultTags`, and the default forwarder of both blocks keeps it.
/// Built per call, because the suite runs at process exit, when namespace-scope objects are already destroyed.
[[nodiscard]] std::array<Marker, 7UZ> markers() {
    return {{
        {"trigger_name", 0UZ, gr::pmt::Value(std::string("alpha"))},
        {"trigger_time", 1UZ, gr::pmt::Value(std::uint64_t{111})},
        {"trigger_offset", 1UZ, gr::pmt::Value(0.5f)},
        {"num_channels", 37UZ, gr::pmt::Value(gr::Size_t{3})},
        {"rx_overflow", 512UZ, gr::pmt::Value(true)},
        {"signal_name", 900UZ, gr::pmt::Value(std::string("iq"))},
        {"t0", 1200UZ, gr::pmt::Value(std::string("private"))},
    }};
}

constexpr std::size_t kAbsent = std::numeric_limits<std::size_t>::max();

/// What a scheduled run of `DcOffsetCorrect<CF>` delivered, and the `tau` it held at the end.
struct TaggedRun {
    bool            ran = false;
    std::vector<CF> samples;
    double          tau         = 0.0;
    double          reportedTau = 0.0;
};

/// Runs a counted stream carrying `tags` through an enabled `DcOffsetCorrect<CF>` into a sink.
[[nodiscard]] TaggedRun runTagged(const std::vector<gr::Tag>& tags) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 4000U}, {"mark_tag", false}});
    source._tags     = tags;
    auto& dc         = graph.emplaceBlock<DcOffsetCorrect<CF>>({{"enabled", true}});
    auto& sink       = graph.emplaceBlock<TagSink<CF, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

    TaggedRun run;
    if (!graph.connect<"out", "in">(source, dc).has_value() || !graph.connect<"out", "in">(dc, sink).has_value()) {
        return run;
    }
    gr::scheduler::Simple scheduler;
    run.ran         = scheduler.exchange(std::move(graph)).has_value() && scheduler.runAndWait().has_value();
    run.samples     = std::vector<CF>(sink._samples.begin(), sink._samples.end());
    run.tau         = dc.tau.value;
    run.reportedTau = reported(dc, "tau");
    return run;
}

} // namespace

static_assert(gr::block::kUnfilteredTagPropagationAdmissible<IqSwap<CF>>, "passes the every-key predicate: no Resampling<> or Stride<>, no asynchronous port, no tag policy other than UnfilteredTagPropagation and no forwardTags() override");
static_assert(gr::block::kUnfilteredTagPropagationAdmissible<DcOffsetCorrect<CF>>);
static_assert(gr::block::kUnfilteredTagPropagationAdmissible<DcOffsetCorrect<float>>);

const boost::ut::suite<"IqCorrection"> iqCorrectionTests = [] {
    using namespace boost::ut;

    static_assert(gr::HasConstProcessOneFunction<IqSwap<CF>>, "IqSwap writes no state");
    static_assert(!gr::HasConstProcessOneFunction<DcOffsetCorrect<CF>>, "DcOffsetCorrect updates its estimate");
    static_assert(!gr::HasConstProcessOneFunction<DcOffsetCorrect<float>>, "the real form carries the same estimate");

    "the swap exchanges the components, bit for bit"_test = [] {
        IqSwap<CF>            block = make<IqSwap<CF>>({{"enabled", true}});
        const std::vector<CF> x{CF(1.f, 2.f), CF(-3.f, 5.f), CF(0.f, -7.f), CF(1e-30f, 1e30f)};
        const std::vector<CF> want{CF(2.f, 1.f), CF(5.f, -3.f), CF(-7.f, 0.f), CF(1e30f, 1e-30f)};
        expect(that % (drive(block, std::span<const CF>(x)) == want)) << "asymmetric components: a conjugation, a negation and a no-op each fail here";
    };

    "the swap is j*conj to zero ULP"_test = [] {
        IqSwap<CF>            block = make<IqSwap<CF>>({{"enabled", true}});
        const std::vector<CF> x{CF(1.f, 2.f), CF(-3.f, 5.f), CF(0.f, -7.f), CF(1e-30f, 1e30f)};
        const std::vector<CF> got = drive(block, std::span<const CF>(x));
        for (std::size_t i = 0UZ; i < x.size(); ++i) {
            const CF rotated = CF(0.f, 1.f) * std::conj(x[i]);
            expect(eq(got[i].real(), rotated.real()) && eq(got[i].imag(), rotated.imag())) << std::format("sample {}: the two spectral mirrors differ only by the fixed 90 degrees", i);
        }
    };

    "the swap disabled is identity and enabled twice is identity"_test = [] {
        const std::vector<CF> x = noise(1000000UZ, 0x13198a2e03707344ULL);

        IqSwap<CF> off = make<IqSwap<CF>>();
        expect(that % (drive(off, std::span<const CF>(x)) == x)) << "disabled is a bit-exact pass-through";

        IqSwap<CF>            first  = make<IqSwap<CF>>({{"enabled", true}});
        IqSwap<CF>            second = make<IqSwap<CF>>({{"enabled", true}});
        const std::vector<CF> once   = drive(first, std::span<const CF>(x));
        expect(that % (drive(second, std::span<const CF>(once)) == x)) << "the swap is an involution";
    };

    "the null at DC is structural, at every alpha"_test = [] {
        constexpr float kRate = 96000.f;
        for (const double wanted : {0.5, 0.1, 1e-5}) {
            DcOffsetCorrect<CF> block = make<DcOffsetCorrect<CF>>({{"enabled", true}, {"sample_rate", kRate}, {"tau", tauFor(wanted, static_cast<double>(kRate))}});
            expect(approx(block.alpha.value, wanted, wanted * 1e-9)) << "alpha comes from tau and the rate";

            const std::size_t     count = static_cast<std::size_t>(1.5 * std::log(1e-6 / std::numbers::sqrt2) / std::log1p(-wanted)) + 64UZ;
            const std::vector<CF> y     = drive(block, std::span<const CF>(flat(count, CF(1.f, 1.f))));
            expect(lt(meanMagnitude(std::span<const CF>(y).last(64UZ)), 1e-6)) << std::format("alpha {}: H(1) = 0 exactly, so a constant is nulled", wanted);
        }

        // alpha = 1e-8 needs 1.4e9 samples to reach 1e-6 by simulation. H(1) = 0 gives the closed form below, and
        // the test asserts it. The decay is exactly (1-alpha)^(n+1), and its limit is zero.
        constexpr double      kAlpha = 1e-8;
        DcOffsetCorrect<CF>   slow   = make<DcOffsetCorrect<CF>>({{"enabled", true}, {"sample_rate", kRate}, {"tau", tauFor(kAlpha, static_cast<double>(kRate))}});
        const std::vector<CF> y      = drive(slow, std::span<const CF>(flat(1000000UZ, CF(1.f, 1.f))));
        for (const std::size_t at : {0UZ, 1UZ, 1000UZ, 999999UZ}) {
            const double want = std::pow(1.0 - kAlpha, static_cast<double>(at + 1UZ));
            expect(approx(static_cast<double>(y[at].real()), want, 1e-6)) << std::format("alpha 1e-8, sample {}", at);
        }
    };

    "the corner is where the time constant says and the passband gain is not one"_test = [] {
        constexpr float kRate = 96000.f;
        for (const double seconds : {0.01, 0.1}) {
            DcOffsetCorrect<CF> block  = make<DcOffsetCorrect<CF>>({{"enabled", true}, {"sample_rate", kRate}, {"tau", seconds}});
            const double        alpha  = block.alpha;
            const std::size_t   settle = static_cast<std::size_t>(30.0 / alpha);

            const gr::property_map settings{{"enabled", true}, {"sample_rate", kRate}, {"tau", seconds}};
            DcOffsetCorrect<CF>    atCorner  = make<DcOffsetCorrect<CF>>(settings);
            DcOffsetCorrect<CF>    atMidband = make<DcOffsetCorrect<CF>>(settings);
            const std::vector<CF>  corner    = drive(atCorner, std::span<const CF>(tone(settle + 4096UZ, 1.0, alpha / (2.0 * kPi))));
            const std::vector<CF>  midband   = drive(atMidband, std::span<const CF>(tone(settle + 4096UZ, 1.0, 0.25)));

            const double ratio = meanMagnitude(std::span<const CF>(corner).last(4096UZ)) / meanMagnitude(std::span<const CF>(midband).last(4096UZ));
            expect(approx(ratio, 1.0 / std::numbers::sqrt2, 0.02 / std::numbers::sqrt2)) << std::format("tau {} s: the -3 dB point sits at alpha*fs/(2*pi) = {:.4f} Hz", seconds, block.corner_hz.value);

            DcOffsetCorrect<CF> atNyquist = make<DcOffsetCorrect<CF>>(settings);
            std::vector<CF>     alternating(settle + 4096UZ);
            for (std::size_t i = 0UZ; i < alternating.size(); ++i) {
                alternating[i] = CF(i % 2UZ == 0UZ ? 1.f : -1.f, 0.f);
            }
            const std::vector<CF> nyquist = drive(atNyquist, std::span<const CF>(alternating));
            expect(approx(meanMagnitude(std::span<const CF>(nyquist).last(4096UZ)), 2.0 * (1.0 - alpha) / (2.0 - alpha), 1e-6)) << "the closed form, which a passband-gain normalization would break";
        }
    };

    "the estimate settles on the closed form"_test = [] {
        constexpr float  kRate   = 96000.f;
        constexpr double kOffset = 0.05;
        for (const double product : {1e3, 1e5}) {
            const double        alpha = 1.0 / (1.0 + product);
            DcOffsetCorrect<CF> block = make<DcOffsetCorrect<CF>>({{"enabled", true}, {"sample_rate", kRate}, {"tau", product / static_cast<double>(kRate)}});

            // The closed form, on a noise-free offset. Unit-rms noise cannot resolve this to 2 %. The tracker's own
            // output noise is sqrt(alpha/2) per component, which at fs*tau = 1e3 is 0.022 against an offset of 0.05.
            const std::size_t count = static_cast<std::size_t>(6.0 * product);
            std::ignore             = drive(block, std::span<const CF>(flat(count, CF(static_cast<float>(kOffset), 0.f))));

            DcOffsetCorrect<CF> stepping   = make<DcOffsetCorrect<CF>>({{"enabled", true}, {"sample_rate", kRate}, {"tau", product / static_cast<double>(kRate)}});
            std::size_t         reached    = 0UZ;
            std::size_t         ninetyNine = 0UZ;
            for (std::size_t n = 1UZ; n <= count; ++n) {
                std::ignore           = stepping.processOne(CF(static_cast<float>(kOffset), 0.f));
                const double fraction = stepping.dcEstimate().real() / kOffset;
                if (reached == 0UZ && fraction >= 1.0 - 1.0 / std::numbers::e) {
                    reached = n;
                }
                if (ninetyNine == 0UZ && fraction >= 0.99) {
                    ninetyNine = n;
                }
            }
            const double wantNinetyNine = std::log(0.01) / std::log1p(-alpha);
            expect(approx(static_cast<double>(reached), product, 0.02 * product)) << std::format("fs*tau {}: 1 - 1/e after one time constant", product);
            expect(approx(static_cast<double>(ninetyNine), wantNinetyNine, 0.02 * wantNinetyNine)) << "and 99 % where 1 - (1-alpha)^n predicts";
            expect(lt(std::abs(block.dcEstimate().real() - kOffset), kOffset * 1e-2)) << "converged";

            DcOffsetCorrect<CF> noisy = make<DcOffsetCorrect<CF>>({{"enabled", true}, {"sample_rate", kRate}, {"tau", product / static_cast<double>(kRate)}});
            std::ignore               = drive(noisy, std::span<const CF>(noise(count, 0x2ff29ULL + static_cast<std::uint64_t>(product), kOffset)));
            const double sigma        = std::sqrt(alpha / 2.0);
            expect(lt(std::abs(noisy.dcEstimate().real() - kOffset), 4.0 * sigma)) << "and on unit-rms noise it lands inside four of its own standard deviations";
        }
    };

    "the float32 trap, pinned"_test = [] {
        constexpr double kProduct   = 33554432.0; // 2^25
        const float      complement = 1.f - 1.f / (1.f + static_cast<float>(kProduct));
        expect(eq(complement, 1.f)) << "in float32 the pole lands on the unit circle here and the recursion integrates";

        constexpr float     kRate = 96000.f;
        DcOffsetCorrect<CF> block = make<DcOffsetCorrect<CF>>({{"enabled", true}, {"sample_rate", kRate}, {"tau", kProduct / static_cast<double>(kRate)}});
        expect(neq(1.0 - block.alpha.value, 1.0)) << "in double it does not";

        const std::size_t count = static_cast<std::size_t>(4.0 * kProduct);
        const CF          dc(0.05f, 0.f);
        for (std::size_t i = 0UZ; i < count; ++i) {
            std::ignore = block.processOne(dc);
        }
        const double want = 0.05 * (1.0 - std::exp(-4.0));
        expect(approx(block.dcEstimate().real(), want, 0.1 * want)) << std::format("four time constants: {:.7f} against the float32 direct form's 0.2172", block.dcEstimate().real());
    };

    "the estimate survives a toggle and a tau change"_test = [] {
        constexpr float     kRate    = 96000.f;
        constexpr double    kProduct = 1000.0;
        DcOffsetCorrect<CF> block    = make<DcOffsetCorrect<CF>>({{"enabled", true}, {"sample_rate", kRate}, {"tau", kProduct / static_cast<double>(kRate)}});

        std::ignore            = drive(block, std::span<const CF>(flat(static_cast<std::size_t>(10.0 * kProduct), CF(0.05f, 0.f))));
        const double converged = block.dcEstimate().real();
        expect(approx(converged, 0.05, 1e-4));

        apply(block, {{"enabled", false}});
        std::ignore = drive(block, std::span<const CF>(flat(static_cast<std::size_t>(10.0 * kProduct), CF(0.9f, 0.f))));
        expect(eq(block.dcEstimate().real(), converged)) << "a disabled block does not track";

        apply(block, {{"enabled", true}});
        const CF first = block.processOne(CF(0.05f, 0.f));
        expect(lt(std::abs(static_cast<double>(first.real())), 1e-4)) << "so re-enabling nulls the spike at once rather than over tau";

        apply(block, {{"tau", 10.0 * kProduct / static_cast<double>(kRate)}});
        expect(approx(block.dcEstimate().real(), converged, 1e-4)) << "a tau change keeps the converged estimate";
        expect(approx(block.alpha.value, 1.0 / (1.0 + 10.0 * kProduct), 1e-12)) << "only how fast it chases";
    };

    "the real form is the complex real lane, bit for bit"_test = [] {
        constexpr float       kRate     = 96000.f;
        const std::vector<CF> complexIn = noise(200000UZ, 0x452821e638d01377ULL, 0.05);

        for (const bool on : {false, true}) {
            const gr::property_map settings{{"enabled", on}, {"sample_rate", kRate}, {"tau", 0.01}};
            DcOffsetCorrect<CF>    complexBlock = make<DcOffsetCorrect<CF>>(settings);
            DcOffsetCorrect<float> realBlock    = make<DcOffsetCorrect<float>>(settings);
            for (std::size_t i = 0UZ; i < complexIn.size(); ++i) {
                // The imaginary component is zeroed, and the complex block's real lane sees the
                // same arithmetic. Both estimates then update from the same values.
                const CF    fromComplex = complexBlock.processOne(CF(complexIn[i].real(), 0.f));
                const float fromReal    = realBlock.processOne(complexIn[i].real());
                expect(eq(fromComplex.real(), fromReal)) << std::format("enabled={} sample {}", on, i);
                if (fromComplex.real() != fromReal) {
                    return;
                }
            }
            expect(eq(complexBlock.dcEstimate().real(), realBlock.dcEstimate().real())) << "one estimator, two spellings";
            expect(eq(realBlock.dcEstimate().imag(), 0.0)) << "a real stream reports no imaginary estimate";
        }
    };

    "the real form nulls a constant"_test = [] {
        DcOffsetCorrect<float> block = make<DcOffsetCorrect<float>>({{"enabled", true}, {"sample_rate", 96000.f}, {"tau", tauFor(0.1, 96000.0)}});
        float                  last  = 1.f;
        for (std::size_t i = 0UZ; i < 4096UZ; ++i) {
            last = block.processOne(1.f);
        }
        expect(lt(std::abs(static_cast<double>(last)), 1e-6)) << "H(1) = 0, so a constant envelope level is nulled";
    };

    "every key rides through both blocks, the private one included"_test = [] {
        const std::array<Marker, 7UZ> kMarkers = markers();
        gr::Graph                     graph;
        auto&                         source = graph.emplaceBlock<TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 4000U}, {"mark_tag", false}});
        for (const Marker& marker : kMarkers) {
            source._tags.emplace_back(marker.at, gr::property_map{{gr::property_map::key_type{marker.key}, marker.value}});
        }
        source._tags.emplace_back(200UZ, gr::property_map{{gr::property_map::key_type{"sample_rate"}, gr::pmt::Value(192000.f)}});
        std::ranges::sort(source._tags, std::ranges::less{}, &gr::Tag::index);

        auto& swap = graph.emplaceBlock<IqSwap<CF>>({{"enabled", true}});
        auto& dc   = graph.emplaceBlock<DcOffsetCorrect<CF>>({{"enabled", true}, {"sample_rate", 48000.f}, {"tau", 1.0}});
        auto& sink = graph.emplaceBlock<TagSink<CF, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

        expect(graph.connect<"out", "in">(source, swap).has_value());
        expect(graph.connect<"out", "in">(swap, dc).has_value());
        expect(graph.connect<"out", "in">(dc, sink).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        std::vector<std::size_t> offsets(kMarkers.size(), kAbsent);
        float                    declaredRate = 0.f;
        for (const gr::Tag& receivedTag : sink._tags) {
            for (std::size_t which = 0UZ; which < kMarkers.size(); ++which) {
                const auto found = receivedTag.map.find(gr::property_map::key_type{kMarkers[which].key});
                if (found != receivedTag.map.end() && found->second == kMarkers[which].value) {
                    offsets[which] = receivedTag.index;
                }
            }
            if (receivedTag.index == 200UZ) {
                if (const auto found = receivedTag.map.find(gr::property_map::key_type{"sample_rate"}); found != receivedTag.map.end()) {
                    declaredRate = found->second.get_if<float>() != nullptr ? *found->second.get_if<float>() : 0.f;
                }
            }
        }

        for (std::size_t which = 0UZ; which < kMarkers.size(); ++which) {
            expect(eq(offsets[which], kMarkers[which].at)) << std::format("{} rides through two 1:1 blocks unmoved", kMarkers[which].key);
        }
        expect(eq(declaredRate, 48000.f)) << "DcOffsetCorrect carries the reserved key, so the rate declared downstream becomes its own";
    };

    "degenerate settings"_test = [] {
        expect(throws([] { std::ignore = make<DcOffsetCorrect<CF>>({{"tau", 0.0}}); }));
        expect(throws([] { std::ignore = make<DcOffsetCorrect<CF>>({{"tau", -1.0}}); }));
        expect(throws([] { std::ignore = make<DcOffsetCorrect<CF>>({{"sample_rate", 0.f}}); }));
        expect(throws([] { std::ignore = make<DcOffsetCorrect<CF>>({{"sample_rate", -96000.f}}); }));
        expect(throws([] { std::ignore = make<DcOffsetCorrect<CF>>({{"sample_rate", std::numeric_limits<float>::quiet_NaN()}}); }));
        expect(throws([] { std::ignore = make<DcOffsetCorrect<CF>>({{"sample_rate", std::numeric_limits<float>::infinity()}}); }));

        DcOffsetCorrect<CF> block = make<DcOffsetCorrect<CF>>({{"sample_rate", 96000.f}, {"tau", 1.0}});
        expect(refused(block, {{"tau", 0.0}})) << "and on a live change, not only at construction";
        expect(refused(block, {{"sample_rate", -1.f}}));
        expect(eq(block.tau.value, 1.0) && eq(block.sample_rate.value, 96000.f)) << "the members keep the accepted values";

        for (const double seconds : {1e-9, 1e-3, 1.0, 1e6}) {
            for (const float rate : {1.f, 96000.f, 25e6f}) {
                DcOffsetCorrect<CF> valid = make<DcOffsetCorrect<CF>>({{"sample_rate", rate}, {"tau", seconds}});
                expect(gt(valid.alpha.value, 0.0) && le(valid.alpha.value, 1.0)) << std::format("alpha stays in (0, 1] at tau {} and fs {}", seconds, rate);
            }
        }
    };

    "a refused change leaves the block computing on the settings it accepted"_test = [] {
        // The reference sees only the accepted changes. After every refusal both blocks run the same samples, and the
        // outputs must agree bit for bit.
        const std::vector<CF>  x = noise(30000UZ, 0x3f84d5b5b5470917ULL, 0.05);
        const gr::property_map initial{{"enabled", true}, {"sample_rate", 96000.f}, {"tau", 0.01}};
        DcOffsetCorrect<CF>    block     = make<DcOffsetCorrect<CF>>(initial);
        DcOffsetCorrect<CF>    reference = make<DcOffsetCorrect<CF>>(initial);
        const auto             same      = [&](std::size_t from, std::size_t count) {
            const std::span<const CF> part = std::span<const CF>(x).subspan(from, count);
            return std::ranges::equal(drive(block, part), drive(reference, part));
        };
        expect(same(0UZ, 5000UZ));

        expect(refused(block, {{"tau", 0.0}})) << "the limits refuse tau = 0";
        expect(eq(block.tau.value, 0.01) && eq(reported(block, "tau"), 0.01)) << "the member and settings().get() keep the accepted tau";
        expect(nothrow([&block] { apply(block, {{"sample_rate", 48000.f}}); })) << "a valid rate is accepted";
        apply(reference, {{"sample_rate", 48000.f}});
        expect(eq(block.alpha.value, 1.0 / (1.0 + 0.01 * 48000.0))) << "alpha follows the accepted tau and the new rate";
        expect(same(5000UZ, 5000UZ)) << "after a refused tau";

        expect(throws([&block] { apply(block, {{"tau", 1e304}}); })) << "a tau whose product with the rate overflows is refused";
        expect(eq(reported(block, "tau"), 1e304)) << "the refused product leaves its tau in the member, and settings().get() returns it";
        expect(nothrow([&block] { apply(block, {{"tau", 1e304}}); })) << "the same value sent again alone equals its member and passes with no refusal";
        expect(same(10000UZ, 5000UZ)) << "after a refused product, the estimate keeps tracking at the accepted alpha";

        std::ignore      = block.settings().setStaged({{"enabled", false}, {"tau", 0.0}});
        const auto mixed = block.settings().applyStagedParameters();
        expect(eq(mixed.failedParameters.size(), 1UZ) && mixed.failedParameters.contains(gr::property_map::key_type{"tau"})) << "the limits refuse tau alone";
        expect(!block._enabled) << "and the toggle in the same application is accepted";
        apply(reference, {{"enabled", false}});
        expect(same(15000UZ, 2000UZ)) << "disabled";
        apply(block, {{"enabled", true}});
        apply(reference, {{"enabled", true}});

        expect(throws([&block] { apply(block, {{"alpha", 0.5}}); })) << "alpha follows from tau and sample_rate and takes no write";
        expect(throws([&block] { apply(block, {{"corner_hz", 1.0}}); })) << "nor does corner_hz";
        expect(eq(reported(block, "alpha"), 0.5)) << "settings().get() returns the written value until the next accepted application";
        expect(same(17000UZ, 5000UZ)) << "the block computes on the derived alpha";
        apply(block, {{"tau", 0.02}});
        apply(reference, {{"tau", 0.02}});
        expect(eq(block.alpha.value, reference.alpha.value) && eq(block.corner_hz.value, reference.corner_hz.value)) << "an accepted application writes both derived values again";

        expect(refused(block, {{"tau", -1.0}}));
        expect(refused(block, {{"tau", -1.0}})) << "the same value sent again is refused again, since the member kept the accepted tau";
        expect(refused(block, {{"sample_rate", std::numeric_limits<float>::quiet_NaN()}}));
        expect(eq(block.sample_rate.value, 48000.f)) << "a refused NaN never reaches the member";
        expect(same(22000UZ, 8000UZ)) << "after the repeated refusals";
    };

    "a block made from the settings saved from a configured block computes as that block"_test = [] {
        DcOffsetCorrect<CF>    configured = make<DcOffsetCorrect<CF>>({{"enabled", true}, {"sample_rate", 2.4e6f}, {"tau", 0.01}});
        const gr::property_map stored     = savedSettings(configured);
        expect(stored.contains(gr::property_map::key_type{"alpha"}) && stored.contains(gr::property_map::key_type{"corner_hz"})) << "the saved settings carry the derived values";

        const auto reloaded = loaded<DcOffsetCorrect<CF>>(stored);
        expect(reloaded->state() == gr::lifecycle::State::INITIALISED) << "the block accepts the derived values it computes itself";
        expect(eq(reloaded->alpha.value, configured.alpha.value) && eq(reloaded->corner_hz.value, configured.corner_hz.value));
        const std::vector<CF> x = noise(4000UZ, 0x5be0cd19137e2179ULL, 0.05);
        expect(std::ranges::equal(drive(*reloaded, x), drive(configured, x))) << "and computes on the saved settings";

        gr::property_map edited = stored;
        edited.insert_or_assign(gr::property_map::key_type{"alpha"}, 0.5);
        expect(loaded<DcOffsetCorrect<CF>>(edited)->state() == gr::lifecycle::State::ERROR) << "a saved alpha other than the derived one is refused";
    };

    "a tau outside its limits on a tag mid-run leaves the run going on the accepted tau"_test = [] {
        const gr::property_map accepted{{"tau", 0.01}};
        const TaggedRun        clean = runTagged({gr::Tag{0UZ, accepted}});
        const TaggedRun        got   = runTagged({gr::Tag{0UZ, accepted}, gr::Tag{2000UZ, gr::property_map{{"tau", -1.0}}}, gr::Tag{3000UZ, gr::property_map{{"tau", std::numeric_limits<double>::quiet_NaN()}}}});
        expect(clean.ran && eq(clean.samples.size(), 4000UZ)) << "the reference run";
        expect(got.ran) << "the refusals go out as plain messages, and the run completes";
        expect(eq(got.samples.size(), 4000UZ));
        expect(std::ranges::equal(got.samples, clean.samples)) << "the block computes on the accepted tau from the refused tags on";
        expect(eq(got.tau, 0.01) && eq(got.reportedTau, 0.01)) << "the member and settings().get() keep the accepted tau";
    };

    "a sample with a NaN or infinite component leaves the estimate where it was"_test = [] {
        constexpr float        kNan = std::numeric_limits<float>::quiet_NaN();
        constexpr float        kInf = std::numeric_limits<float>::infinity();
        constexpr std::size_t  kAt  = 10000UZ;
        const std::vector<CF>  x    = noise(20000UZ, 0x7c9e3f2a1b5d4c86ULL, 0.05);
        const gr::property_map settings{{"enabled", true}, {"sample_rate", 96000.f}, {"tau", 0.001}};
        const auto             sameFloat = [](float got, float want) { return (std::isnan(got) && std::isnan(want)) || got == want; };

        for (const CF bad : {CF(kNan, 0.1f), CF(0.1f, kInf), CF(-kInf, kNan)}) {
            DcOffsetCorrect<CF> block           = make<DcOffsetCorrect<CF>>(settings);
            DcOffsetCorrect<CF> reference       = make<DcOffsetCorrect<CF>>(settings);
            std::ignore                         = drive(block, std::span<const CF>(x).first(kAt));
            const std::complex<double> estimate = block.dcEstimate();
            const CF                   atBad    = block.processOne(bad);
            expect(block.dcEstimate() == estimate) << std::format("({}, {}) leaves the estimate where it was", bad.real(), bad.imag());
            expect(sameFloat(atBad.real(), static_cast<float>(static_cast<double>(bad.real()) - estimate.real())) && sameFloat(atBad.imag(), static_cast<float>(static_cast<double>(bad.imag()) - estimate.imag()))) << "the sample itself leaves less the held estimate";

            const std::vector<CF> after = drive(block, std::span<const CF>(x).subspan(kAt));
            const std::vector<CF> want  = drive(reference, std::span<const CF>(x));
            expect(std::ranges::equal(after, std::span<const CF>(want).subspan(kAt))) << "every later output is the one the stream without that sample gives";
        }

        DcOffsetCorrect<float> block     = make<DcOffsetCorrect<float>>(settings);
        DcOffsetCorrect<float> reference = make<DcOffsetCorrect<float>>(settings);
        bool                   same      = true;
        for (std::size_t i = 0UZ; i < x.size(); ++i) {
            if (i == kAt) {
                std::ignore = block.processOne(kNan);
            }
            same = same && block.processOne(x[i].real()) == reference.processOne(x[i].real());
        }
        expect(same) << "the real form holds its estimate the same way";
    };

    "a run of exact zeros leaves the estimate at zero, not at a subnormal"_test = [] {
        // The estimate is a double, and the pole at a device rate is very close to one. At 10 MS/s with tau = 1 s,
        // alpha = 1e-7. An estimate of 0.05 then needs 7 054 006 510 zeros, 705 s of stream, to reach the smallest
        // normal. Only the pole sets the count. The case runs at tau*fs = 9, where alpha = 0.1. The count is 6 696
        // for a real component at 0.05 and 6 691 for an imaginary one at 0.03.
        constexpr float       kRate  = 96000.f;
        constexpr std::size_t kZeros = 8000UZ;

        DcOffsetCorrect<CF> block = make<DcOffsetCorrect<CF>>({{"enabled", true}, {"sample_rate", kRate}, {"tau", tauFor(0.1, static_cast<double>(kRate))}});
        std::ignore               = drive(block, std::span<const CF>(flat(500UZ, CF(0.05f, 0.03f))));
        expect(approx(block.dcEstimate().real(), 0.05, 1e-6) && approx(block.dcEstimate().imag(), 0.03, 1e-6)) << "both components are tracking before the silence starts";

        const std::vector<CF> got = drive(block, std::span<const CF>(flat(kZeros, CF{})));

        expect(eq(std::fpclassify(block.dcEstimate().real()), FP_ZERO)) << "the real estimate is exactly zero, not the smallest subnormal";
        expect(eq(std::fpclassify(block.dcEstimate().imag()), FP_ZERO)) << "and so is the imaginary one, the two being flushed separately";

        // The port is float. The output underflows to exactly zero once the estimate passes 7e-46, some 160 samples
        // into the decay. The double state reaches its own smallest normal much later. The float subnormals near
        // 1e-40 come from the estimate passing through that range. The tail must not hold a subnormal left by a state
        // that stopped decaying.
        const std::span<const CF> tail      = std::span<const CF>(got).last(kZeros / 2UZ);
        const auto                subnormal = [](CF sample) { return std::fpclassify(sample.real()) == FP_SUBNORMAL || std::fpclassify(sample.imag()) == FP_SUBNORMAL; };
        expect(eq(std::ranges::count_if(tail, subnormal), std::ptrdiff_t{0})) << "no output in the tail is subnormal";
        expect(std::ranges::all_of(tail, [](CF sample) { return sample == CF{}; })) << "every one of them is exactly zero";
    };

    "the flush changes nothing above the subnormal range"_test = [] {
        // The recursion without the flush, sample by sample against the block, over a step and the decay after it.
        // The comparison stops once both components are below 1e-300. At alpha = 0.1 that takes some 6 528 zeros.
        // The flush cannot fire before 6 696. The flush acts on no compared sample, and the non-zero final estimate
        // confirms it.
        constexpr float       kRate = 96000.f;
        constexpr std::size_t kStep = 500UZ;

        DcOffsetCorrect<CF> block = make<DcOffsetCorrect<CF>>({{"enabled", true}, {"sample_rate", kRate}, {"tau", tauFor(0.1, static_cast<double>(kRate))}});
        const double        alpha = block._alpha; // the block's own coefficients, so only the flush can differ
        const double        rest  = block._oneMinusAlpha;

        double      real      = 0.0;
        double      imag      = 0.0;
        std::size_t compared  = 0UZ;
        bool        identical = true;
        while (identical && compared < 20000UZ && (compared < kStep || real >= 1e-300 || imag >= 1e-300)) {
            const CF     sample = compared < kStep ? CF(0.05f, 0.03f) : CF{};
            const double x      = static_cast<double>(sample.real());
            const double y      = static_cast<double>(sample.imag());
            real                = alpha * x + rest * real;
            imag                = alpha * y + rest * imag;

            const CF made = block.processOne(sample);
            identical     = block.dcEstimate().real() == real && block.dcEstimate().imag() == imag && made == CF(static_cast<float>(x - real), static_cast<float>(y - imag));
            ++compared;
        }
        expect(identical) << std::format("the estimate and the output agree bit for bit with the unflushed recursion at every sample, {} of them", compared);
        expect(lt(real, 1e-300) && lt(imag, 1e-300)) << "and the comparison ran until both components were below 1e-300";
        expect(neq(block.dcEstimate().real(), 0.0) && neq(block.dcEstimate().imag(), 0.0)) << "with the flush still not fired, so none of it was flushed arithmetic";
    };
};

int main() { /* not needed for UT */ }
