#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <iterator>
#include <limits>
#include <memory>
#include <numbers>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/YamlPmt.hpp>

#include <gnuradio-4.0/basic/IqCorrection.hpp>
#include <gnuradio-4.0/sync/Pll.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace {

using gr::blocks::basic::DcOffsetCorrect;
using gr::blocks::basic::IqSwap;
using gr::blocks::sync::PllCarrierTracking;
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

/// Seven keys at six offsets. `t0` is not in `gr::tag::kDefaultTags`, and both blocks forward it.
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

/// A unit carrier of `frequency` radians per sample from sample `start` on, with phase `phase + frequency*n` at sample
/// `n`. The samples before `start` are zero.
[[nodiscard]] std::vector<CF> burst(std::size_t count, std::size_t start, double frequency, double phase) {
    std::vector<CF> samples(count);
    for (std::size_t i = start; i < count; ++i) {
        samples[i] = std::polar(1.f, static_cast<float>(std::remainder(phase + frequency * static_cast<double>(i), 2.0 * kPi)));
    }
    return samples;
}

/// The value of `key` in `map` as a `double`, or NaN when it holds no `float` or `double`.
[[nodiscard]] double estimate(const gr::property_map& map, const char* key) {
    const auto found = map.find(gr::property_map::key_type{key});
    if (found == map.end()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (const double* value = found->second.get_if<double>(); value != nullptr) {
        return *value;
    }
    if (const float* value = found->second.get_if<float>(); value != nullptr) {
        return static_cast<double>(*value);
    }
    return std::numeric_limits<double>::quiet_NaN();
}

/// What a carrier-tracking loop behind an enabled `IqSwap<CF>` put out: its derotated samples, its phase on each sample
/// before the update, its frequency after it, and the tags on its derotated output.
struct LoopRun {
    bool                 ran = false;
    std::vector<CF>      derotated;
    std::vector<float>   phase;
    std::vector<float>   frequency;
    std::vector<gr::Tag> tags;
};

/// Runs `samples`, with `tag` on them, through an enabled `IqSwap<CF>` into a `PllCarrierTracking`.
[[nodiscard]] LoopRun runSwapIntoLoop(const std::vector<CF>& samples, const gr::Tag& tag) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(samples.size())}, {"values", gr::Tensor<CF>(samples.begin(), samples.end())}});
    source._tags     = {tag};
    auto& swap       = graph.emplaceBlock<IqSwap<CF>>({{"enabled", true}});
    auto& loop       = graph.emplaceBlock<PllCarrierTracking>();
    auto& derotated  = graph.emplaceBlock<TagSink<CF, ProcessFunction::USE_PROCESS_ONE>>({{"name", "derotated"}});
    auto& phase      = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "phase"}});
    auto& frequency  = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "frequency"}});

    LoopRun run;
    if (!graph.connect<"out", "in">(source, swap).has_value() || !graph.connect<"out", "in">(swap, loop).has_value() || !graph.connect<"out", "in">(loop, derotated).has_value() || !graph.connect<"phase", "in">(loop, phase).has_value() || !graph.connect<"freq", "in">(loop, frequency).has_value()) {
        return run;
    }
    gr::scheduler::Simple scheduler;
    run.ran       = scheduler.exchange(std::move(graph)).has_value() && scheduler.runAndWait().has_value();
    run.derotated = std::vector<CF>(derotated._samples.begin(), derotated._samples.end());
    run.phase     = std::vector<float>(phase._samples.begin(), phase._samples.end());
    run.frequency = std::vector<float>(frequency._samples.begin(), frequency._samples.end());
    run.tags      = derotated._tags;
    return run;
}

/// The tags a sink receives at the index of `tag` when 1000 samples carrying it pass through an `IqSwap<CF>`.
[[nodiscard]] std::vector<gr::Tag> swapTags(bool enabled, const gr::Tag& tag) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 1000U}, {"mark_tag", false}});
    source._tags     = {tag};
    auto& swap       = graph.emplaceBlock<IqSwap<CF>>({{"enabled", enabled}});
    auto& sink       = graph.emplaceBlock<TagSink<CF, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

    std::vector<gr::Tag> received;
    if (!graph.connect<"out", "in">(source, swap).has_value() || !graph.connect<"out", "in">(swap, sink).has_value()) {
        return received;
    }
    gr::scheduler::Simple scheduler;
    if (!scheduler.exchange(std::move(graph)).has_value() || !scheduler.runAndWait().has_value()) {
        return received;
    }
    std::ranges::copy_if(sink._tags, std::back_inserter(received), [&tag](const gr::Tag& t) { return t.index == tag.index; });
    return received;
}

/// The samples and tags a sink gets behind an `IqSwap<CF>` that takes `kSwapCount` samples of CF(1, 2).
struct SwapRun {
    bool                 ran = false;
    std::vector<CF>      samples;
    std::vector<gr::Tag> tags;
};

constexpr std::size_t kSwapCount = 1000UZ;

/// Runs `kSwapCount` samples of CF(1, 2), with `tags` on them, through an `IqSwap<CF>` built with `settings`.
[[nodiscard]] SwapRun swapRun(const gr::property_map& settings, std::vector<gr::Tag> tags) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(kSwapCount)}, {"values", gr::Tensor<CF>{CF(1.f, 2.f)}}});
    source._tags     = std::move(tags);
    auto& swap       = graph.emplaceBlock<IqSwap<CF>>(settings);
    auto& sink       = graph.emplaceBlock<TagSink<CF, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

    SwapRun run;
    if (!graph.connect<"out", "in">(source, swap).has_value() || !graph.connect<"out", "in">(swap, sink).has_value()) {
        return run;
    }
    gr::scheduler::Simple scheduler;
    run.ran     = scheduler.exchange(std::move(graph)).has_value() && scheduler.runAndWait().has_value();
    run.samples = std::vector<CF>(sink._samples.begin(), sink._samples.end());
    run.tags    = sink._tags;
    return run;
}

/// Runs `kSwapCount` samples of CF(1, 2), with `tag` on them, through an `IqSwap<CF>` that starts as `enabled`.
[[nodiscard]] SwapRun swapRun(bool enabled, const gr::Tag& tag) { return swapRun(gr::property_map{{"enabled", enabled}}, {tag}); }

} // namespace

/// Publishes `kSwapCount` samples of CF(1, 2) and ends the stream. `endTag` sits on the position after the last sample.
struct SourceTaggingItsEnd : gr::Block<SourceTaggingItsEnd> {
    gr::PortOut<CF> out;

    GR_MAKE_REFLECTABLE(SourceTaggingItsEnd, out);

    gr::property_map endTag;
    std::size_t      _sent = 0UZ;

    gr::work::Status processBulk(gr::OutputSpanLike auto& output) {
        const std::size_t n = std::min(output.size(), kSwapCount - _sent);
        std::ranges::fill(std::span(output.data(), n), CF(1.f, 2.f));
        _sent += n;
        if (_sent == kSwapCount) {
            output.publishTag(endTag, n);
        }
        output.publish(n);
        return _sent == kSwapCount ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

/// Passes its samples on and records the tags it is handed on the call that ends the stream, which no sample carries.
struct EndTagRecorder : gr::Block<EndTagRecorder> {
    gr::PortIn<CF>  in;
    gr::PortOut<CF> out;

    GR_MAKE_REFLECTABLE(EndTagRecorder, in, out);

    std::vector<gr::Tag> _endTags;

    [[nodiscard]] constexpr CF processOne(CF sample) const noexcept { return sample; }

    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& /*outputSpans*/, std::size_t processedIn) {
        if (processedIn != 0UZ) {
            return;
        }
        gr::for_each_reader_span(
            [this](auto& inSpan) {
                for (const gr::Tag& tag : inSpan.rawTags) {
                    if (tag.index >= inSpan.streamIndex) {
                        _endTags.push_back(tag);
                    }
                }
            },
            inputSpans);
    }
};

namespace {

/// The tags `EndTagRecorder` is handed on the final call when `endTag` sits on the position after the last sample of a
/// stream through an enabled `IqSwap<CF>`.
[[nodiscard]] std::vector<gr::Tag> swapEndTags(const gr::property_map& endTag, bool& ran) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<SourceTaggingItsEnd>();
    source.endTag    = endTag;
    auto& swap       = graph.emplaceBlock<IqSwap<CF>>({{"enabled", true}});
    auto& recorder   = graph.emplaceBlock<EndTagRecorder>();
    auto& sink       = graph.emplaceBlock<TagSink<CF, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

    if (!graph.connect<"out", "in">(source, swap).has_value() || !graph.connect<"out", "in">(swap, recorder).has_value() || !graph.connect<"out", "in">(recorder, sink).has_value()) {
        return {};
    }
    gr::scheduler::Simple scheduler;
    ran = scheduler.exchange(std::move(graph)).has_value() && scheduler.runAndWait().has_value();
    return recorder._endTags;
}

} // namespace

static_assert(IqSwap<CF>::hasForwardTagsOverride(), "IqSwap forwards its tags itself, mapping the carrier estimates");
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

    "an enabled swap hands a carrier loop the estimates of the swapped carrier"_test = [] {
        constexpr std::size_t kStart     = 400UZ;
        constexpr std::size_t kCount     = 2400UZ;
        constexpr double      kFrequency = 0.1;
        constexpr double      kPhase     = 0.2;
        const double          tagged     = std::remainder(kPhase + kFrequency * static_cast<double>(kStart), 2.0 * kPi); // the carrier's phase on the tagged sample
        const double          swapped    = kPi / 2.0 - tagged;                                                           // the phase of j*conj(x) there
        const std::vector<CF> carrier    = burst(kCount, kStart, kFrequency, kPhase);

        const gr::property_map estimates{{"phase_est", tagged}, {"freq_est", kFrequency}};
        const gr::property_map trigger{{"trigger_name", std::string("burst")}, {"trigger_meta_info", estimates}};
        const LoopRun          top    = runSwapIntoLoop(carrier, gr::Tag{kStart, estimates});
        const LoopRun          nested = runSwapIntoLoop(carrier, gr::Tag{kStart, trigger});
        for (const auto& [where, ran] : {std::pair{"at the top level", &top}, std::pair{"inside trigger_meta_info", &nested}}) {
            const LoopRun& run = *ran;
            expect(fatal(run.ran && run.derotated.size() == kCount && run.phase.size() == kCount && run.frequency.size() == kCount)) << where;

            expect(lt(std::abs(std::remainder(static_cast<double>(run.phase[kStart]) - swapped, 2.0 * kPi)), 1e-5)) << std::format("{}: the loop takes the swapped phase pi/2 - p on the tagged sample", where);
            expect(lt(std::abs(static_cast<double>(run.frequency[kStart]) + kFrequency), 1e-4)) << std::format("{}: the loop takes the negated frequency on the tagged sample", where);
            float worst = 0.f;
            for (std::size_t i = kStart; i < kCount; ++i) {
                worst = std::max(worst, std::abs(std::arg(run.derotated[i])));
            }
            expect(lt(worst, 1e-3f)) << std::format("{}: the loop holds the swapped carrier from the tagged sample on, worst residual {} rad", where, worst);
        }

        const auto meta = std::ranges::find_if(nested.tags, [](const gr::Tag& t) { return t.map.contains(gr::property_map::key_type{"trigger_meta_info"}); });
        expect(fatal(meta != nested.tags.end())) << "the loop forwards the trigger's map";
        const gr::property_map* forwarded = meta->map.at(gr::property_map::key_type{"trigger_meta_info"}).get_if<gr::property_map>();
        expect(fatal(forwarded != nullptr));
        expect(eq(estimate(*forwarded, "phase_est"), swapped)) << "the trigger's phase leaves for the swapped carrier";
        expect(eq(estimate(*forwarded, "freq_est"), -kFrequency)) << "the trigger's frequency leaves negated";
    };

    "a disabled swap forwards carrier estimates unchanged"_test = [] {
        constexpr double       kPhase     = 0.5;
        constexpr float        kFrequency = 0.25f;
        const gr::property_map estimates{{"phase_est", kPhase}, {"freq_est", kFrequency}, {"amp_est", 2.f}};
        gr::property_map       tagged = estimates;
        tagged.insert_or_assign(gr::property_map::key_type{"trigger_meta_info"}, estimates);

        const std::vector<gr::Tag> passed  = swapTags(false, gr::Tag{100UZ, tagged});
        const std::vector<gr::Tag> swapped = swapTags(true, gr::Tag{100UZ, tagged});
        expect(fatal(eq(passed.size(), 1UZ) && eq(swapped.size(), 1UZ)));
        expect(eq(passed[0].index, 100UZ) && eq(swapped[0].index, 100UZ)) << "the tag stays on its sample";
        expect(passed[0].map == tagged) << "disabled, every key leaves as it arrived";

        gr::property_map swappedEstimates{{"phase_est", kPi / 2.0 - kPhase}, {"freq_est", -kFrequency}, {"amp_est", 2.f}};
        gr::property_map want = swappedEstimates;
        want.insert_or_assign(gr::property_map::key_type{"trigger_meta_info"}, swappedEstimates);
        expect(swapped[0].map == want) << "enabled, both copies of the phase and frequency change, of the same type, and the amplitude stays";
        expect(passed[0].map != swapped[0].map) << "the two arms differ";
    };

    "a tag's setting keys leave with the block's own values through a disabled and an enabled swap"_test = [] {
        const gr::property_map rejected{{"enabled", std::string("not a bool")}, {"phase_est", 0.5}};
        for (const bool on : {false, true}) {
            const SwapRun run   = swapRun(on, gr::Tag{100UZ, rejected});
            const auto    found = std::ranges::find_if(run.tags, [](const gr::Tag& t) { return t.index == 100UZ && t.map.contains(gr::property_map::key_type{"enabled"}); });
            expect(fatal(run.ran && found != run.tags.end())) << on;
            const bool* kept = found->map.at(gr::property_map::key_type{"enabled"}).get_if<bool>();
            expect(fatal(kept != nullptr)) << on << ": the key leaves with the block's value, of the block's type";
            expect(eq(*kept, on)) << on << ": and that value is the block's own";
            expect(eq(estimate(found->map, "phase_est"), on ? kPi / 2.0 - 0.5 : 0.5)) << on << ": the estimate follows the block's state";
        }
    };

    "a mapped phase is reduced to [-pi, pi]"_test = [] {
        for (const double phase : {-3.0, 3.0, -2.0 * kPi + 0.25, 7.0}) {
            gr::property_map tagged{{"phase_est", phase}, {"trigger_meta_info", gr::property_map{{"phase_est", phase}}}};
            const SwapRun    run   = swapRun(true, gr::Tag{100UZ, tagged});
            const auto       found = std::ranges::find_if(run.tags, [](const gr::Tag& t) { return t.index == 100UZ && t.map.contains(gr::property_map::key_type{"phase_est"}); });
            expect(fatal(run.ran && found != run.tags.end())) << phase;
            const double mapped = estimate(found->map, "phase_est");
            expect(le(std::abs(mapped), kPi)) << std::format("{} leaves as {} inside [-pi, pi]", phase, mapped);
            expect(lt(std::abs(std::remainder(mapped - (kPi / 2.0 - phase), 2.0 * kPi)), 1e-12)) << std::format("{} leaves as pi/2 - p modulo a turn", phase);
            const gr::property_map* meta = found->map.at(gr::property_map::key_type{"trigger_meta_info"}).get_if<gr::property_map>();
            expect(fatal(meta != nullptr));
            expect(eq(estimate(*meta, "phase_est"), mapped)) << "the nested copy is reduced alike";
        }
        const SwapRun single = swapRun(true, gr::Tag{100UZ, gr::property_map{{"phase_est", -3.f}}});
        const auto    found  = std::ranges::find_if(single.tags, [](const gr::Tag& t) { return t.index == 100UZ && t.map.contains(gr::property_map::key_type{"phase_est"}); });
        expect(fatal(single.ran && found != single.tags.end()));
        expect(le(std::abs(estimate(found->map, "phase_est")), kPi)) << "a float phase is reduced as well";
    };

    "an estimate on the position after the last sample is mapped once on the call that ends the stream"_test = [] {
        const gr::property_map tagged{{"phase_est", 0.5}, {"freq_est", 0.25}, {"end_of_stream", true}};
        bool                   ran  = false;
        const auto             tags = swapEndTags(tagged, ran);
        expect(fatal(ran)) << "the graph ran";
        const auto hasEstimate = [](const gr::Tag& t) { return t.map.contains(gr::property_map::key_type{"phase_est"}); };
        expect(fatal(eq(std::ranges::count_if(tags, hasEstimate), std::ptrdiff_t{1}))) << "the estimate reaches the next block once";
        const auto found = std::ranges::find_if(tags, hasEstimate);
        expect(eq(found->index, kSwapCount)) << "on the position after the last sample";
        expect(eq(estimate(found->map, "phase_est"), kPi / 2.0 - 0.5)) << "mapped";
        expect(eq(estimate(found->map, "freq_est"), -0.25)) << "mapped";
    };

    "a tag that turns the swap on or off maps the estimate it carries to match its sample"_test = [] {
        // Tags do not update a key set at construction, so the swap is built without `enabled`.
        const SwapRun run = swapRun(gr::property_map{}, {gr::Tag{100UZ, gr::property_map{{"enabled", true}, {"phase_est", 0.5}, {"freq_est", 0.25}}}, gr::Tag{200UZ, gr::property_map{{"enabled", false}, {"phase_est", 0.5}, {"freq_est", 0.25}}}});
        expect(fatal(run.ran && run.samples.size() == kSwapCount));
        expect(run.samples[99] == CF(1.f, 2.f)) << "the sample before the first tag passes as it arrived";
        expect(run.samples[100] == CF(2.f, 1.f)) << "the tag at 100 turns the exchange on at its own sample";
        expect(run.samples[200] == CF(1.f, 2.f)) << "the tag at 200 turns it off at its own sample";
        for (const auto& [index, on] : {std::pair{100UZ, true}, std::pair{200UZ, false}}) {
            const auto found = std::ranges::find_if(run.tags, [index](const gr::Tag& t) { return t.index == index && t.map.contains(gr::property_map::key_type{"phase_est"}); });
            expect(fatal(found != run.tags.end())) << index;
            expect(eq(estimate(found->map, "phase_est"), on ? kPi / 2.0 - 0.5 : 0.5)) << index << ": the estimate is mapped when its sample is exchanged";
            expect(eq(estimate(found->map, "freq_est"), on ? -0.25 : 0.25)) << index;
            const bool* kept = found->map.at(gr::property_map::key_type{"enabled"}).get_if<bool>();
            expect(fatal(kept != nullptr) && eq(*kept, on)) << index << ": the key leaves with the state its sample ran with";
        }
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
