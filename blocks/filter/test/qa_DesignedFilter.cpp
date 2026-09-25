#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <numbers>
#include <print>
#include <span>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/filter/DesignedFilter.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::filter::DesignedFilter;
using CF       = std::complex<float>;
namespace test = gr::blocks::testing::span;

template<typename TSample, typename TTap>
[[nodiscard]] DesignedFilter<TSample, TTap> make(gr::property_map settings) {
    DesignedFilter<TSample, TTap> block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
    return block;
}

template<typename TBlock>
void apply(TBlock& block, gr::property_map settings) {
    std::ignore = block.settings().setStaged(std::move(settings));
    std::ignore = block.settings().applyStagedParameters();
}

template<typename TSample, typename TTap>
[[nodiscard]] auto run(DesignedFilter<TSample, TTap>& block, const std::vector<TSample>& input, std::size_t chunkOutputs = 0UZ) {
    const std::size_t m = block.decimation;
    return test::runDecimating<typename DesignedFilter<TSample, TTap>::TOut>(block, std::span<const TSample>(input), chunkOutputs * m, m).samples;
}

/// The tap-count law these Hamming-window designs are pinned against: the count grows with the rate and shrinks with
/// the transition width, and is odd by construction.
[[nodiscard]] constexpr int hammingLaw(double fs, double tw) { return static_cast<int>(53.0 * fs / (22.0 * tw)) | 1; }

/// A key per tag, so a tag that survives is visible wherever it lands.
[[nodiscard]] gr::property_map tagKey(std::size_t which) {
    gr::property_map map;
    map.insert_or_assign(gr::property_map::key_type{std::format("tag{}", which)}, static_cast<gr::Size_t>(which));
    return map;
}

/// @brief The index of the sample of largest magnitude, the first of several equal ones.
template<typename T>
[[nodiscard]] std::size_t peakIndex(std::span<const T> y) {
    std::size_t at = 0UZ;
    for (std::size_t k = 1UZ; k < y.size(); ++k) {
        if (std::abs(y[k]) > std::abs(y[at])) {
            at = k;
        }
    }
    return at;
}

template<typename T>
[[nodiscard]] double worstDelta(std::span<const T> a, std::span<const T> b) {
    double worst = 0.0;
    for (std::size_t i = 0UZ; i < a.size(); ++i) {
        worst = std::max(worst, static_cast<double>(std::abs(a[i] - b[i])));
    }
    return worst;
}

} // namespace

const boost::ut::suite<"DesignedFilter"> designedFilterTests = [] {
    using namespace boost::ut;
    using gr::algorithm::window::Type;

    "design parity: the block's taps are the library designer's, exactly"_test = [] {
        // The audio low-pass shape (fs 240k, 17k cutoff, 2k transition, Hamming at the pinned law).
        {
            const int  n     = hammingLaw(240000.0, 2000.0);
            auto       block = make<float, float>({{"profile", std::string("lowpass")}, {"sample_rate", 240000.f}, {"cutoff", 17000.0}, {"transition_width", 2000.0}, {"window", std::string("Hamming")}, {"taps", static_cast<gr::Size_t>(n)}});
            const auto want  = gr::filter::fir::design::designLowpass({.sampleRate = 240000.0, .cutoff = 17000.0, .transitionWidth = 2000.0, .window = Type::Hamming, .taps = n});
            expect(eq(block._designed.size(), want.size()));
            expect(eq(worstDelta(std::span<const float>(block._designed), std::span<const float>(want)), 0.0)) << "same designer, same floats";
            expect(eq(static_cast<std::size_t>(block.designed_taps.value), want.size())) << "the observable states the length";
        }
        // The sharp CW complex band (fs 96k, 575..825 Hz, 20 Hz transition — the receiver's largest filter).
        {
            const int  n     = hammingLaw(96000.0, 20.0);
            auto       block = make<CF, CF>({{"profile", std::string("complex_bandpass")}, {"sample_rate", 96000.f}, {"cutoff", 575.0}, {"high_cutoff", 825.0}, {"transition_width", 20.0}, {"window", std::string("Hamming")}, {"taps", static_cast<gr::Size_t>(n)}});
            const auto want  = gr::filter::fir::design::designComplexBandpass({.sampleRate = 96000.0, .cutoff = 575.0, .highCutoff = 825.0, .transitionWidth = 20.0, .window = Type::Hamming, .taps = n});
            expect(eq(block._designed.size(), want.size()));
            expect(eq(worstDelta(std::span<const CF>(block._designed), std::span<const CF>(want)), 0.0));
            expect(eq(static_cast<std::size_t>(n), want.size())) << "the CW length is the pinned 11563";
        }
        // A root-raised-cosine at the RDS matched-filter shape.
        {
            auto       block = make<float, float>({{"profile", std::string("root_raised_cosine")}, {"sample_rate", 19000.f}, {"symbol_rate", 1187.5}, {"alpha", 1.0}, {"taps", gr::Size_t(151)}});
            const auto want  = gr::filter::fir::design::designRootRaisedCosine(151, 19000.0, 1187.5, 1.0);
            expect(eq(block._designed.size(), want.size()));
            expect(eq(worstDelta(std::span<const float>(block._designed), std::span<const float>(want)), 0.0));
        }
        // A transition-estimated length (taps = 0) follows the library's own estimate.
        {
            auto              block = make<float, float>({{"profile", std::string("lowpass")}, {"sample_rate", 96000.f}, {"cutoff", 10000.0}, {"transition_width", 2000.0}});
            const std::size_t want  = gr::filter::fir::design::tapCountOf({.sampleRate = 96000.0, .cutoff = 10000.0, .transitionWidth = 2000.0});
            expect(eq(block._designed.size(), want));
        }
    };

    "behavioral: the low-pass passes and stops what its edges say"_test = [] {
        constexpr double fs    = 240000.0;
        auto             block = make<float, float>({{"profile", std::string("lowpass")}, {"sample_rate", static_cast<float>(fs)}, {"cutoff", 17000.0}, {"transition_width", 2000.0}, {"window", std::string("Hamming")}, {"taps", static_cast<gr::Size_t>(hammingLaw(fs, 2000.0))}});

        const std::size_t  n = 65536UZ;
        std::vector<float> in(n);
        for (std::size_t i = 0UZ; i < n; ++i) {
            const double t = static_cast<double>(i) / fs;
            in[i]          = static_cast<float>(std::sin(2.0 * std::numbers::pi * 1000.0 * t) + std::sin(2.0 * std::numbers::pi * 30000.0 * t));
        }
        const auto out = run(block, in);

        const auto level = [&](double hz) {
            std::complex<double> acc{};
            for (std::size_t i = 4096UZ; i < out.size(); ++i) {
                acc += static_cast<double>(out[i]) * std::exp(std::complex<double>(0.0, -2.0 * std::numbers::pi * hz * static_cast<double>(i) / fs));
            }
            return 2.0 * std::abs(acc) / static_cast<double>(out.size() - 4096UZ);
        };
        const double inBand  = level(1000.0);
        const double stopped = level(30000.0);
        expect(gt(inBand, 0.97) && lt(inBand, 1.03)) << "the passband carries the stated gain";
        expect(lt(20.0 * std::log10(stopped / inBand), -50.0)) << "the stopband rejects";
    };

    "behavioral: the complex band-pass rejects the image and holds unity at center"_test = [] {
        constexpr double fs    = 96000.0;
        auto             block = make<CF, CF>({{"profile", std::string("complex_bandpass")}, {"sample_rate", static_cast<float>(fs)}, {"cutoff", 300.0}, {"high_cutoff", 3000.0}, {"transition_width", 200.0}, {"window", std::string("Hamming")}, {"taps", static_cast<gr::Size_t>(hammingLaw(fs, 200.0))}});

        // The design's own contract: unity gain at band center.
        const double         fc = (300.0 + 3000.0) / 2.0 / fs;
        std::complex<double> acc{};
        for (std::size_t i = 0UZ; i < block._designed.size(); ++i) {
            acc += std::complex<double>(block._designed[i]) * std::exp(std::complex<double>(0.0, -2.0 * std::numbers::pi * fc * static_cast<double>(i)));
        }
        expect(lt(std::abs(std::abs(acc) - 1.0), 1.0e-3)) << "unity gain at band center";

        const std::size_t n = 65536UZ;
        std::vector<CF>   in(n);
        const auto        tone = [&](double hz, std::size_t i) { return std::exp(std::complex<double>(0.0, 2.0 * std::numbers::pi * hz * static_cast<double>(i) / fs)); };
        for (std::size_t i = 0UZ; i < n; ++i) {
            in[i] = CF(tone(1000.0, i) + tone(-1000.0, i) + tone(6000.0, i));
        }
        const auto out = run(block, in);

        const auto level = [&](double hz) {
            std::complex<double> acc2{};
            for (std::size_t i = 4096UZ; i < out.size(); ++i) {
                acc2 += std::complex<double>(out[i]) * std::exp(std::complex<double>(0.0, -2.0 * std::numbers::pi * hz * static_cast<double>(i) / fs));
            }
            return std::abs(acc2) / static_cast<double>(out.size() - 4096UZ);
        };
        const double inBand = level(1000.0);
        expect(gt(inBand, 0.97) && lt(inBand, 1.03)) << "the passband carries unity";
        expect(lt(20.0 * std::log10(level(-1000.0) / inBand), -50.0)) << "the image side rejects";
        expect(lt(20.0 * std::log10(level(6000.0) / inBand), -50.0)) << "the stopband rejects";
    };

    "a real stream with complex taps comes out as an analytic tone"_test = [] {
        // A real tone of amplitude A carries A/2 at each of +f and -f, so the surviving positive-frequency copy
        // leaves a band-pass normalized to `gain` at its center at A * gain / 2: the gain of two puts it back at A.
        constexpr double fs    = 48000.0;
        constexpr double tone  = 19000.0;
        const int        n     = hammingLaw(fs, 100.0);
        auto             block = make<float, CF>({{"profile", std::string("complex_bandpass")}, {"sample_rate", static_cast<float>(fs)}, {"cutoff", 18800.0}, {"high_cutoff", 19200.0}, {"transition_width", 100.0}, {"gain", 2.0}, {"window", std::string("Hamming")}, {"taps", static_cast<gr::Size_t>(n)}});
        const auto       want  = gr::filter::fir::design::designComplexBandpass({.sampleRate = fs, .cutoff = 18800.0, .highCutoff = 19200.0, .transitionWidth = 100.0, .gain = 2.0, .window = Type::Hamming, .taps = n});
        expect(eq(block._designed.size(), want.size()));
        expect(eq(worstDelta(std::span<const CF>(block._designed), std::span<const CF>(want)), 0.0)) << "the taps are the library designer's for a real sample type too";

        const std::size_t  count = 65536UZ;
        std::vector<float> in(count);
        for (std::size_t i = 0UZ; i < count; ++i) {
            in[i] = static_cast<float>(std::cos(2.0 * std::numbers::pi * tone * static_cast<double>(i) / fs));
        }
        const auto out = run(block, in);
        expect(eq(out.size(), count));

        const auto level = [&](double hz) {
            std::complex<double> acc{};
            for (std::size_t i = 4096UZ; i < out.size(); ++i) {
                acc += std::complex<double>(out[i]) * std::exp(std::complex<double>(0.0, -2.0 * std::numbers::pi * hz * static_cast<double>(i) / fs));
            }
            return std::abs(acc) / static_cast<double>(out.size() - 4096UZ);
        };
        const double inBand = level(tone);
        expect(gt(inBand, 0.97) && lt(inBand, 1.03)) << "the analytic tone carries the input amplitude";
        expect(lt(20.0 * std::log10(level(-tone) / inBand), -50.0)) << "the input's negative-frequency copy is rejected";

        expect(throws([] { std::ignore = make<float, CF>({{"profile", std::string("bandpass")}, {"sample_rate", 48000.f}, {"cutoff", 18800.0}, {"high_cutoff", 19200.0}, {"transition_width", 100.0}}); })) << "a real profile refuses the complex tap type";
    };

    "a live edge change redesigns; an unrelated write does not"_test = [] {
        auto block = make<float, float>({{"profile", std::string("lowpass")}, {"sample_rate", 96000.f}, {"cutoff", 10000.0}, {"transition_width", 2000.0}});

        const gr::Size_t   firstLength = block.designed_taps;
        std::vector<float> in(4096UZ, 1.0f);
        std::ignore = run(block, in);

        apply(block, {{"transition_width", 1000.0}});
        expect(gt(block.designed_taps.value, firstLength)) << "a narrower transition designs a longer filter";

        const gr::Size_t after = block.designed_taps;
        apply(block, {{"name", std::string("renamed")}});
        expect(eq(block.designed_taps.value, after)) << "an unrelated setting does not redesign";

        std::ignore = run(block, in);
        // A constant through a redesigned unity-DC-gain low-pass stays a constant: the stream continued.
        auto tail = run(block, in);
        expect(lt(std::abs(static_cast<double>(tail.back()) - 1.0), 1e-3)) << "the stream continues through the redesign";
    };

    "a stated rate wins over stream tags; the stream's tags still ride through"_test = [] {
        using gr::testing::ProcessFunction;
        using gr::testing::TagSink;
        using gr::testing::TagSource;

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 65536U}, {"mark_tag", false}});
        source._tags.emplace_back(1024UZ, gr::property_map{{gr::property_map::key_type{"sample_rate"}, gr::pmt::Value(48000.f)}});

        auto& filter = graph.emplaceBlock<DesignedFilter<float, float>>({{"profile", std::string("lowpass")}, {"sample_rate", 96000.f}, {"cutoff", 10000.0}, {"transition_width", 2000.0}});
        auto& sink   = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", std::string("TagSink")}});

        expect(graph.connect<"out", "in">(source, filter).has_value());
        expect(graph.connect<"out", "in">(filter, sink).has_value());

        const std::size_t at96k = gr::filter::fir::design::tapCountOf({.sampleRate = 96000.0, .cutoff = 10000.0, .transitionWidth = 2000.0});

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        // The framework's explicit-set rule: configuring sample_rate removed it from the
        // auto-update set, so the stream's 48 kHz declaration does not redesign the filter.
        // A redesign to a new rate arrives by restaging the setting, which the settings-path
        // leg above proves.
        expect(eq(static_cast<std::size_t>(filter.designed_taps.value), at96k)) << "the stated rate keeps the design";

        bool sawUpstream = false;
        for (const gr::Tag& seenTag : sink._tags) {
            if (const auto found = seenTag.map.find(gr::property_map::key_type{"sample_rate"}); found != seenTag.map.end()) {
                const float forwarded = found->second.value_or(0.f);
                if (seenTag.index >= 1024UZ && forwarded == 48000.f) {
                    sawUpstream = true; // decimation 1 forwards the upstream declaration unchanged
                }
            }
        }
        expect(sawUpstream) << "the upstream rate tag rode through at its offset";
    };

    "a tag leaves on the output that carries its sample's energy"_test = [] {
        // 41 designed taps delay by 20 input samples, so input i lands on output (i + 20) / M. A call takes 60 inputs,
        // and each input below sits in the first call's last output group with a delayed position on a whole output.
        constexpr std::size_t kDelay = 20UZ;
        constexpr std::size_t kChunk = 60UZ;
        struct Row {
            gr::Size_t  m;
            std::size_t at;
        };
        constexpr Row kRows[] = {{1U, 59UZ}, {3U, 58UZ}};
        for (const auto& [m, at] : kRows) {
            auto block = make<float, float>({{"profile", std::string("lowpass")}, {"sample_rate", 96000.f}, {"cutoff", 10000.0}, {"taps", gr::Size_t(41)}, {"decimation", m}});
            expect(eq(block.designed_taps.value, 41U));
            expect(eq(block.groupDelaySamples(), static_cast<double>(kDelay)));

            std::vector<float> x(240UZ, 0.0f);
            x[at] = 1.0f;
            const std::vector<gr::Tag> tags{gr::Tag{at, tagKey(0)}};
            const auto                 got  = test::runDecimating<float>(block, std::span<const float>(x), kChunk, m, std::span<const gr::Tag>(tags));
            const std::size_t          want = (at + kDelay) / m;

            expect(ge(want, kChunk / m)) << "the delayed output belongs to a later call than the tag's input";
            expect(that % (got.offsetsOf("tag0") == std::vector<std::size_t>{want})) << std::format("M = {}: input {} leaves on output {} and on no other", m, at, want);
            expect(eq(peakIndex(std::span<const float>(got.samples)), want)) << std::format("M = {}: the impulse at input {} peaks on output {}", m, at, want);
        }

        // a complex band-pass is a modulated low-pass: its magnitude stays symmetric and its delay (N-1)/2
        auto band = make<CF, CF>({{"profile", std::string("complex_bandpass")}, {"sample_rate", 96000.f}, {"cutoff", 5000.0}, {"high_cutoff", 15000.0}, {"taps", gr::Size_t(41)}});
        expect(eq(band.groupDelaySamples(), static_cast<double>(kDelay)));
        std::vector<CF> z(240UZ, CF{});
        z[59] = CF{1.0f, 0.0f};
        const std::vector<gr::Tag> bandTags{gr::Tag{59UZ, tagKey(0)}};
        const auto                 banded = test::runDecimating<CF>(band, std::span<const CF>(z), kChunk, 1UZ, std::span<const gr::Tag>(bandTags));
        expect(that % (banded.offsetsOf("tag0") == std::vector<std::size_t>{79UZ}));
        expect(eq(peakIndex(std::span<const CF>(banded.samples)), 79UZ));

        // a Hilbert transformer has no center tap: the tag sits on the zero between the two equal peaks, the output
        // that pairs with the input delayed by (N-1)/2
        auto hilbert = make<float, float>({{"profile", std::string("hilbert")}, {"sample_rate", 48000.f}, {"taps", gr::Size_t(31)}});
        expect(eq(hilbert.groupDelaySamples(), 15.0));
        std::vector<float> h(240UZ, 0.0f);
        h[59] = 1.0f;
        const std::vector<gr::Tag> hilbertTags{gr::Tag{59UZ, tagKey(0)}};
        const auto                 shifted = test::runDecimating<float>(hilbert, std::span<const float>(h), kChunk, 1UZ, std::span<const gr::Tag>(hilbertTags));
        expect(that % (shifted.offsetsOf("tag0") == std::vector<std::size_t>{74UZ}));
        expect(eq(shifted.samples[74], 0.0f));
        expect(lt(std::abs(std::abs(shifted.samples[73]) - std::abs(shifted.samples[75])), 1e-6f));
        const std::size_t peak = peakIndex(std::span<const float>(shifted.samples));
        expect(peak == 73UZ || peak == 75UZ) << "the largest outputs flank the tag";
    };

    "refusals fire by name and leave the previous design running"_test = [] {
        expect(throws([] { std::ignore = make<float, float>({{"profile", std::string("no_such_profile")}, {"sample_rate", 96000.f}, {"cutoff", 1000.0}, {"transition_width", 100.0}}); }));
        expect(throws([] { std::ignore = make<float, float>({{"profile", std::string("complex_bandpass")}, {"sample_rate", 96000.f}, {"cutoff", 300.0}, {"high_cutoff", 3000.0}, {"transition_width", 100.0}}); })) << "a complex profile refuses the real tap type";
        expect(throws([] { std::ignore = make<CF, CF>({{"profile", std::string("lowpass")}, {"sample_rate", 96000.f}, {"cutoff", 1000.0}, {"transition_width", 100.0}}); })) << "a real profile refuses the complex tap type";
        expect(throws([] { std::ignore = make<float, float>({{"profile", std::string("root_raised_cosine")}, {"sample_rate", 19000.f}, {"symbol_rate", 1187.5}}); })) << "an RRC with no stated length";
        expect(throws([] { std::ignore = make<float, float>({{"profile", std::string("root_raised_cosine")}, {"sample_rate", 19000.f}, {"taps", gr::Size_t(151)}}); })) << "an RRC with no symbol rate";
        expect(throws([] { std::ignore = make<float, float>({{"profile", std::string("lowpass")}, {"sample_rate", 0.f}, {"cutoff", 1000.0}, {"transition_width", 100.0}}); }));

        auto             block    = make<float, float>({{"profile", std::string("lowpass")}, {"sample_rate", 96000.f}, {"cutoff", 10000.0}, {"transition_width", 2000.0}});
        const gr::Size_t designed = block.designed_taps;
        expect(throws([&block] { apply(block, {{"profile", std::string("no_such_profile")}}); })) << "a live refusal throws";
        expect(eq(block.designed_taps.value, designed)) << "and the previous design keeps running";
        // The refused value is what the member holds, so stating it again moves nothing and is not a second
        // request. The refusal is not absorbed either: the design is stale until one succeeds, and a start refuses it.
        expect(nothrow([&block] { apply(block, {{"profile", std::string("no_such_profile")}}); })) << "re-stating the value in force is not a new request";
        expect(eq(block.designed_taps.value, designed)) << "and the previous design is still the one running";
        expect(throws([&block] { block.start(); })) << "the stale design is refused at the next start";
    };
};

const boost::ut::suite<"the hilbert profile"> hilbertTests = [] {
    using namespace boost::ut;

    /// @brief The image rejection of the analytic signal a Hilbert branch and a delayed direct branch build, in dB:
    /// the power at `-f` against the power at `+f`, both read by direct evaluation of the DFT at that one frequency.
    /// The delay is the design's own group delay, which is an integer because the length is odd.
    const auto imageRejectionDb = [](const std::vector<float>& hilbert, const std::vector<float>& direct, std::size_t warmup, double normalized) {
        std::complex<double> positive{};
        std::complex<double> negative{};
        const std::size_t    count = hilbert.size() - warmup;
        for (std::size_t n = 0UZ; n < count; ++n) {
            const std::complex<double> z{static_cast<double>(direct[warmup + n]), static_cast<double>(hilbert[warmup + n])};
            const double               phase = 2.0 * std::numbers::pi * normalized * static_cast<double>(n);
            positive += z * std::complex<double>{std::cos(phase), -std::sin(phase)};
            negative += z * std::complex<double>{std::cos(phase), std::sin(phase)};
        }
        return 20.0 * std::log10(std::abs(positive) / std::abs(negative));
    };

    "the taps are antisymmetric, odd, and zero at the center"_test = [] {
        for (const gr::Size_t length : {31U, 63U, 127U}) {
            auto block = make<float, float>({{"profile", std::string("hilbert")}, {"sample_rate", 48000.f}, {"taps", length}});
            expect(eq(block.designed_taps.value, length)) << "the stated length is the designed length";
            expect(eq(block._designed.size(), static_cast<std::size_t>(length)));

            const std::size_t mid = (block._designed.size() - 1UZ) / 2UZ;
            expect(that % (block._designed[mid] == 0.0f)) << "a type III design has no center tap";
            double worst = 0.0;
            for (std::size_t i = 0UZ; i < mid; ++i) {
                worst = std::max(worst, static_cast<double>(std::abs(block._designed[i] + block._designed[block._designed.size() - 1UZ - i])));
            }
            expect(that % (worst < 1e-7)) << std::format("taps {}: antisymmetric to {:.3e}", length, worst);
            for (std::size_t i = 0UZ; i < mid; ++i) {
                if (((mid - i) % 2UZ) == 0UZ) {
                    expect(that % (block._designed[i] == 0.0f)) << "every even offset from the center is structurally zero";
                }
            }
        }
    };

    "the usable band's ripple is what the length buys"_test = [] {
        // scanHilbert reports the worst departure from unity over [lowEdge, 0.5 - lowEdge]. The departure is a
        // property of the PRODUCT taps * lowEdge -- how many taps the design spends below the edge -- so a longer
        // design buys a proportionally lower edge and nothing else, which is the trade the profile makes the caller
        // state a length for. Measured through a Kaiser window shaped for 60 dB, which is the block's default.
        struct Row {
            gr::Size_t taps;
            double     lowEdge;
            double     ripple;
        };
        const Row kRows[]{{31U, 0.05, 0.030}, {63U, 0.05, 0.0009}, {63U, 0.025, 0.023}, {127U, 0.025, 0.0011}, {127U, 0.0125, 0.020}, {127U, 0.05, 0.0006}};

        std::vector<double> nearEdge; // taps * lowEdge about 1.55
        std::vector<double> wellIn;   // taps * lowEdge about 3.15; the 6.35 row is asserted but belongs to neither
        for (const Row& row : kRows) {
            auto         block  = make<float, float>({{"profile", std::string("hilbert")}, {"sample_rate", 48000.f}, {"taps", row.taps}});
            const auto   levels = gr::filter::fir::design::scanHilbert(block._designed, row.lowEdge);
            const double worst  = std::max(levels.maxMag - 1.0, 1.0 - levels.minMag);
            expect(that % (worst <= row.ripple)) << std::format("taps {}, low edge {}: ripple {:.4f} against {:.4f}", row.taps, row.lowEdge, worst, row.ripple);
            const double spent = static_cast<double>(row.taps) * row.lowEdge;
            if (spent < 2.0) {
                nearEdge.push_back(worst);
            } else if (spent < 4.5) {
                wellIn.push_back(worst);
            }
        }

        for (const std::vector<double>& group : {nearEdge, wellIn}) {
            const auto [low, high] = std::ranges::minmax(group);
            expect(that % (high <= 2.0 * low)) << std::format("one product, one ripple: {:.4f} to {:.4f} across the group", low, high);
        }
    };

    "a real tone becomes an analytic one, its image far down"_test = [imageRejectionDb] {
        constexpr double      kRate  = 48000.0;
        constexpr std::size_t kTaps  = 127UZ;
        constexpr std::size_t kCount = 8192UZ;
        constexpr double      kFloor = 60.0;

        // Bell 202's two tones and one in the middle of the band, all inside the design's usable range at this length
        for (const double tone : {1200.0, 1700.0, 2200.0, 12000.0}) {
            auto               block = make<float, float>({{"profile", std::string("hilbert")}, {"sample_rate", static_cast<float>(kRate)}, {"taps", static_cast<gr::Size_t>(kTaps)}});
            std::vector<float> real(kCount);
            for (std::size_t n = 0UZ; n < kCount; ++n) {
                real[n] = static_cast<float>(std::cos(2.0 * std::numbers::pi * tone * static_cast<double>(n) / kRate));
            }

            const std::vector<float> imag = run<float, float>(block, real);
            expect(eq(imag.size(), kCount));

            // the direct branch is the input delayed by the design's own group delay, which is (taps - 1) / 2 samples
            const std::size_t  delay = (kTaps - 1UZ) / 2UZ;
            std::vector<float> direct(kCount, 0.0f);
            std::copy(real.begin(), real.end() - static_cast<std::ptrdiff_t>(delay), direct.begin() + static_cast<std::ptrdiff_t>(delay));

            const double rejection = imageRejectionDb(imag, direct, kTaps, tone / kRate);
            expect(that % (rejection >= kFloor)) << std::format("{:.0f} Hz: the negative-frequency image is {:.1f} dB down, against a floor of {:.1f}", tone, rejection, kFloor);
        }
    };

    "the profile refuses a length it cannot use"_test = [] {
        expect(throws([] { std::ignore = make<float, float>({{"profile", std::string("hilbert")}, {"sample_rate", 48000.f}}); })) << "there is no transition-width estimate for a Hilbert transformer";
        expect(throws([] { std::ignore = make<float, float>({{"profile", std::string("hilbert")}, {"sample_rate", 48000.f}, {"taps", 64U}}); })) << "an even length has no whole-sample group delay";
        expect(throws([] { std::ignore = make<CF, CF>({{"profile", std::string("hilbert")}, {"sample_rate", 48000.f}, {"taps", 63U}}); })) << "the taps are real, so the profile belongs to the real tap type";
        expect(nothrow([] { std::ignore = make<float, float>({{"profile", std::string("hilbert")}, {"sample_rate", 48000.f}, {"taps", 63U}}); }));
    };
};

int main() { /* not needed for UT */ }
