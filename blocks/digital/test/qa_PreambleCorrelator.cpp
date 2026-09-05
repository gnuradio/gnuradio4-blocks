#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <numbers>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/digital/PreambleCorrelator.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::digital::PreambleCorrelator;
using CF = std::complex<float>;

constexpr double kPi = std::numbers::pi;

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
    [[nodiscard]] double uniform() noexcept { return static_cast<double>(next() >> 11U) / static_cast<double>(1ULL << 53U); }
};

/// A raised-cosine pulse, so an oversampled preamble is band limited and its correlation peak is smooth.
[[nodiscard]] double raisedCosine(double t, double rolloff) {
    if (std::abs(t) < 1e-9) {
        return 1.0;
    }
    const double denominator = 1.0 - 4.0 * rolloff * rolloff * t * t;
    if (std::abs(denominator) < 1e-9) {
        return 0.5 * kPi / 4.0 * std::sin(kPi / (2.0 * rolloff)) / (kPi / (2.0 * rolloff));
    }
    return std::sin(kPi * t) / (kPi * t) * std::cos(kPi * rolloff * t) / denominator;
}

/// A preamble at `sps` samples per symbol: pulse-shaped, so its correlation peak spans several samples.
[[nodiscard]] std::vector<CF> oversampledWord(std::size_t symbols, std::size_t sps, std::uint64_t seed = 0x55aa55aaULL) {
    Rng             rng{seed};
    std::vector<CF> amplitude(symbols);
    for (CF& value : amplitude) {
        const double angle = kPi * 0.25 + kPi * 0.5 * static_cast<double>(rng.next() & 3ULL);
        value              = CF(static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle)));
    }
    const std::size_t length = symbols * sps;
    std::vector<CF>   word(length, CF{});
    for (std::size_t m = 0UZ; m < symbols; ++m) {
        for (std::size_t k = 0UZ; k < length; ++k) {
            const double weight = raisedCosine((static_cast<double>(k) - static_cast<double>(m * sps)) / static_cast<double>(sps), 0.35);
            word[k] += amplitude[m] * static_cast<float>(weight);
        }
    }
    return word;
}

/// A QPSK-valued sync word: constant modulus, so its energy is exactly its length.
[[nodiscard]] std::vector<CF> syncWord(std::size_t length, std::uint64_t seed = 0x243f6a8885a308d3ULL) {
    Rng             rng{seed};
    std::vector<CF> word(length);
    for (CF& symbol : word) {
        const double angle = kPi * 0.25 + kPi * 0.5 * static_cast<double>(rng.next() & 3ULL);
        symbol             = CF(static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle)));
    }
    return word;
}

[[nodiscard]] std::vector<float> interleaved(std::span<const CF> word) {
    std::vector<float> flat(2UZ * word.size());
    for (std::size_t k = 0UZ; k < word.size(); ++k) {
        flat[2UZ * k]       = word[k].real();
        flat[2UZ * k + 1UZ] = word[k].imag();
    }
    return flat;
}

/// A quiet stream with @p word planted at @p at, rotated by @p phase and scaled by @p amplitude.
[[nodiscard]] std::vector<CF> plant(std::span<const CF> word, std::size_t at, std::size_t total, double phase = 0.0, double amplitude = 1.0) {
    std::vector<CF> stream(total, CF{});
    const CF        spin(static_cast<float>(amplitude * std::cos(phase)), static_cast<float>(amplitude * std::sin(phase)));
    for (std::size_t k = 0UZ; k < word.size(); ++k) {
        stream[at + k] = word[k] * spin;
    }
    return stream;
}

[[nodiscard]] float metaFloat(const gr::Tag& tag, std::string_view key) {
    const auto outer = tag.map.find(gr::property_map::key_type(gr::tag::TRIGGER_META_INFO.shortKey()));
    if (outer == tag.map.end()) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    const auto* nested = outer->second.get_if<gr::property_map>();
    if (nested == nullptr) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    const auto inner = nested->find(gr::property_map::key_type(key));
    return inner == nested->end() ? std::numeric_limits<float>::quiet_NaN() : inner->second.value_or(std::numeric_limits<float>::quiet_NaN());
}

[[nodiscard]] float triggerOffsetOf(const gr::Tag& tag) {
    const auto entry = tag.map.find(gr::property_map::key_type(gr::tag::TRIGGER_OFFSET.shortKey()));
    return entry == tag.map.end() ? std::numeric_limits<float>::quiet_NaN() : entry->second.value_or(std::numeric_limits<float>::quiet_NaN());
}

[[nodiscard]] std::vector<std::size_t> tagOffsets(const std::vector<gr::Tag>& tags) {
    std::vector<std::size_t> where;
    for (const gr::Tag& tag : tags) {
        where.push_back(tag.index);
    }
    return where;
}

[[nodiscard]] std::string join(const std::vector<std::size_t>& values) {
    std::string out;
    for (const std::size_t v : values) {
        out += std::format("{}{}", out.empty() ? "" : ", ", v);
    }
    return out;
}

/// A key of its own per planted tag, so a forwarded tag stays distinguishable from a detection.
[[nodiscard]] gr::property_map tagKey(std::size_t which) {
    gr::property_map map;
    map.insert_or_assign(gr::property_map::key_type{std::format("tag{}", which)}, static_cast<gr::Size_t>(which));
    return map;
}

/// The value of a band-limited signal at a fractional time, so a preamble can be planted off the sample grid.
[[nodiscard]] CF interpolate(std::span<const CF> samples, double t) {
    double real = 0.0;
    double imag = 0.0;
    for (std::size_t k = 0UZ; k < samples.size(); ++k) {
        const double x      = t - static_cast<double>(k);
        const double weight = std::abs(x) < 1e-12 ? 1.0 : std::sin(kPi * x) / (kPi * x);
        real += weight * static_cast<double>(samples[k].real());
        imag += weight * static_cast<double>(samples[k].imag());
    }
    return CF(static_cast<float>(real), static_cast<float>(imag));
}

} // namespace

const boost::ut::suite<"preamble correlator"> preambleTests = [] {
    using namespace boost::ut;

    "the tag sits on the sync word's first symbol and the output lags by N"_test = [] {
        constexpr std::size_t kLength = 32UZ;
        constexpr std::size_t kAt     = 100UZ;
        const std::vector<CF> word    = syncWord(kLength);
        const std::vector<CF> stream  = plant(std::span<const CF>(word), kAt, 400UZ);

        PreambleCorrelator<float> block = make<PreambleCorrelator<float>>({{"sequence", interleaved(std::span<const CF>(word))}, {"threshold", 0.5f}});
        expect(eq(block.delaySamples(), kLength));

        std::vector<float> magnitude;
        const auto         seen = gr::blocks::testing::span::run3<CF, float>(block, std::span<const CF>(stream), 64UZ, magnitude);

        expect(eq(seen.tags.size(), 1UZ));
        expect(that % (tagOffsets(seen.tags) == std::vector<std::size_t>{kAt + kLength})) << "the peak is on the last sync symbol; the stream is delayed so the first one lines up with it";
        expect(eq(seen.samples.size(), stream.size()));
        for (std::size_t i = 0UZ; i < kLength; ++i) {
            expect(that % (seen.samples[i] == CF{})) << "history starts zeroed, so the first N outputs are silence";
        }
        for (std::size_t i = kLength; i < stream.size(); ++i) {
            expect(that % (seen.samples[i] == stream[i - kLength])) << std::format("out[{}] is in[{}]", i, i - kLength);
        }
        expect(that % (seen.samples[kAt + kLength] == stream[kAt])) << "the tagged output sample is the sync word's first symbol";

        const auto peak = static_cast<std::size_t>(std::distance(magnitude.begin(), std::ranges::max_element(magnitude)));
        expect(eq(peak, kAt + kLength)) << "corr peaks at the tag's own index, so a corr capture lines up with the tagged sample";
    };

    "phase, amplitude and sub-sample timing come out of the peak"_test = [] {
        constexpr std::size_t    kLength = 32UZ;
        const std::vector<CF>    word    = syncWord(kLength);
        const std::vector<float> flat    = interleaved(std::span<const CF>(word));

        for (std::size_t turn = 0UZ; turn < 16UZ; ++turn) {
            const double phase = -kPi + 2.0 * kPi * static_cast<double>(turn) / 16.0 + 0.05;
            for (const double amplitude : {0.1, 1.0, 10.0}) {
                const std::vector<CF> stream = plant(std::span<const CF>(word), 80UZ, 300UZ, phase, amplitude);

                // absolute mode is a fraction of the noiseless unit-amplitude peak, so a sweep over amplitude has to
                // carry the amplitude into the threshold — which is the scale dependence the mode is documented to have
                PreambleCorrelator<float> block = make<PreambleCorrelator<float>>({{"sequence", flat}, {"threshold", static_cast<float>(0.5 * amplitude * amplitude)}, {"samples_per_symbol", 64.0f}});
                std::vector<float>        magnitude;
                const auto                seen = gr::blocks::testing::span::run3<CF, float>(block, std::span<const CF>(stream), 0UZ, magnitude);

                expect(eq(seen.tags.size(), 1UZ)) << std::format("phase {:.3f}, amplitude {}", phase, amplitude);
                if (seen.tags.empty()) {
                    continue;
                }
                expect(le(std::abs(static_cast<double>(metaFloat(seen.tags[0], "phase_est")) - phase), 1e-3)) << std::format("phase_est at {:.3f} rad, amplitude {}", phase, amplitude);
                expect(le(std::abs(static_cast<double>(metaFloat(seen.tags[0], "amp_est")) - amplitude) / amplitude, 0.01)) << std::format("amp_est at amplitude {}", amplitude);
                expect(le(std::abs(static_cast<double>(triggerOffsetOf(seen.tags[0]))), 0.01)) << "an on-grid preamble has no sub-sample offset";
            }
        }
    };

    "the sub-sample estimate recovers a fractional delay"_test = [] {
        // An oversampled preamble is the precondition: at one sample per symbol a random sequence's autocorrelation is
        // a spike one sample wide and no three-point fit can resolve inside it; at four samples per symbol the peak
        // spans several samples and the parabolic vertex means something. `samples_per_symbol` exists for that case.
        constexpr std::size_t    kSps    = 4UZ;
        constexpr std::size_t    kLength = 8UZ * kSps;
        const std::vector<CF>    word    = oversampledWord(8UZ, kSps);
        const std::vector<float> flat    = interleaved(std::span<const CF>(word));

        double worst = 0.0;
        for (const double delay : {-0.4, -0.2, 0.0, 0.2, 0.4}) {
            std::vector<CF> stream(400UZ, CF{});
            for (std::size_t i = 0UZ; i < 200UZ; ++i) { // the whole band-limited waveform, resampled off the grid
                stream[80UZ + i] = interpolate(std::span<const CF>(word), static_cast<double>(i) - delay);
            }

            PreambleCorrelator<float> block = make<PreambleCorrelator<float>>({{"sequence", flat}, {"threshold", 0.5f}, {"samples_per_symbol", static_cast<float>(kSps)}});
            std::vector<float>        magnitude;
            const auto                seen = gr::blocks::testing::span::run3<CF, float>(block, std::span<const CF>(stream), 0UZ, magnitude);

            expect(ge(seen.tags.size(), 1UZ)) << std::format("delay {}", delay);
            if (seen.tags.empty()) {
                continue;
            }
            const double recovered = static_cast<double>(seen.tags[0].index) + static_cast<double>(triggerOffsetOf(seen.tags[0])) - static_cast<double>(80UZ + kLength);
            worst                  = std::max(worst, std::abs(recovered - delay));
            // A parabola through three squared magnitudes compresses a large offset: the squaring sharpens the peak
            // faster than a parabola does, so the vertex sits inside the true one and the error grows with |delta|.
            // Measured here: 0.05 samples out to a fifth of a sample, 0.10 at four tenths.
            expect(le(std::abs(recovered - delay), std::abs(delay) <= 0.2 ? 0.05 : 0.11)) << std::format("delay {} recovered as {:.4f}", delay, recovered);
        }
        expect(le(worst, 0.11)) << std::format("worst sub-sample error {:.4f} samples", worst);
    };

    "the sub-sample offset is always finite and inside half a sample"_test = [] {
        // The flat-top guard is not reachable through the block: a detection needs `b > c` strictly, so the
        // denominator `a - 2b + c` is at most `-(b - c)` and can only vanish when `c` is within an ulp of `b`.
        // What is assertable is the property the guard exists for, over noise that reaches every shape of peak.
        constexpr std::size_t kLength = 16UZ;
        const std::vector<CF> word    = syncWord(kLength, 0xdeadbeefULL);

        Rng             rng{};
        std::vector<CF> stream(20000UZ);
        for (CF& sample : stream) {
            sample = CF(static_cast<float>(2.0 * rng.uniform() - 1.0), static_cast<float>(2.0 * rng.uniform() - 1.0));
        }

        PreambleCorrelator<float> block = make<PreambleCorrelator<float>>({{"sequence", interleaved(std::span<const CF>(word))}, {"threshold_mode", std::string("relative")}, {"threshold", 2.0f}});
        std::vector<float>        magnitude;
        const auto                seen = gr::blocks::testing::span::run3<CF, float>(block, std::span<const CF>(stream), 997UZ, magnitude);

        expect(ge(seen.tags.size(), 10UZ)) << "a relative threshold on noise fires often, which this test needs";
        for (const gr::Tag& seenTag : seen.tags) {
            const float offset = triggerOffsetOf(seenTag);
            expect(std::isfinite(offset)) << "never a NaN";
            expect(le(std::abs(offset), 0.5f)) << "clamped to half a sample";
        }
    };

    "the detection does not move with the span size"_test = [] {
        constexpr std::size_t    kLength = 16UZ;
        const std::vector<CF>    word    = syncWord(kLength, 0x1234ULL);
        const std::vector<float> flat    = interleaved(std::span<const CF>(word));

        for (std::size_t at : {0UZ, 1UZ, 2UZ, 37UZ, 200UZ, 480UZ}) {
            const std::vector<CF> stream = plant(std::span<const CF>(word), at, 512UZ);

            PreambleCorrelator<float>      reference = make<PreambleCorrelator<float>>({{"sequence", flat}, {"threshold", 0.5f}});
            std::vector<float>             magnitude;
            const std::vector<std::size_t> want = tagOffsets(gr::blocks::testing::span::run3<CF, float>(reference, std::span<const CF>(stream), 0UZ, magnitude).tags);
            expect(eq(want.size(), 1UZ)) << std::format("planted at {}", at);

            for (const std::size_t chunk : {1UZ, 2UZ, 3UZ, 17UZ, 4096UZ}) {
                PreambleCorrelator<float> block = make<PreambleCorrelator<float>>({{"sequence", flat}, {"threshold", 0.5f}});
                std::vector<float>        chunkedMagnitude;
                const auto                seen = gr::blocks::testing::span::run3<CF, float>(block, std::span<const CF>(stream), chunk, chunkedMagnitude);
                expect(that % (tagOffsets(seen.tags) == want)) << std::format("planted at {}, chunk {}", at, chunk);
                expect(that % (chunkedMagnitude == magnitude)) << std::format("the correlation is chunk independent too, at {} chunk {}", at, chunk);
            }
        }
    };

    "one symbol of dead time suppresses the adjacent re-trigger"_test = [] {
        constexpr std::size_t kLength = 8UZ;
        const std::vector<CF> word(kLength, CF(1.0f, 0.0f)); // a constant sequence, whose correlation is broad
        std::vector<CF>       stream(200UZ, CF{});
        for (std::size_t i = 60UZ; i < 60UZ + 40UZ; ++i) {
            stream[i] = CF(1.0f, 0.0f);
        }

        PreambleCorrelator<float> tight = make<PreambleCorrelator<float>>({{"sequence", interleaved(std::span<const CF>(word))}, {"threshold", 0.5f}, {"samples_per_symbol", 1.0f}});
        PreambleCorrelator<float> wide  = make<PreambleCorrelator<float>>({{"sequence", interleaved(std::span<const CF>(word))}, {"threshold", 0.5f}, {"samples_per_symbol", 64.0f}});
        std::vector<float>        magnitude;
        const std::size_t         tightCount = gr::blocks::testing::span::run3<CF, float>(tight, std::span<const CF>(stream), 0UZ, magnitude).tags.size();
        const std::size_t         wideCount  = gr::blocks::testing::span::run3<CF, float>(wide, std::span<const CF>(stream), 0UZ, magnitude).tags.size();
        expect(ge(tightCount, 1UZ));
        expect(le(wideCount, tightCount)) << std::format("{} detections at 64 samples of dead time against {} at one", wideCount, tightCount);
        expect(eq(wideCount, 1UZ)) << "a dead time longer than the burst leaves exactly one detection";
    };

    "the corr port is optional and costs nothing when unwired"_test = [] {
        constexpr std::size_t kLength = 16UZ;
        const std::vector<CF> word    = syncWord(kLength, 0x777ULL);
        const std::vector<CF> stream  = plant(std::span<const CF>(word), 50UZ, 200UZ);

        PreambleCorrelator<float> wired   = make<PreambleCorrelator<float>>({{"sequence", interleaved(std::span<const CF>(word))}, {"threshold", 0.5f}});
        PreambleCorrelator<float> unwired = make<PreambleCorrelator<float>>({{"sequence", interleaved(std::span<const CF>(word))}, {"threshold", 0.5f}});
        std::vector<float>        withCorr;
        std::vector<float>        withoutCorr;
        const auto                a = gr::blocks::testing::span::run3<CF, float>(wired, std::span<const CF>(stream), 32UZ, withCorr, true);
        const auto                b = gr::blocks::testing::span::run3<CF, float>(unwired, std::span<const CF>(stream), 32UZ, withoutCorr, false);

        expect(that % (tagOffsets(a.tags) == tagOffsets(b.tags)));
        expect(that % (a.samples == b.samples));
        expect(eq(withoutCorr.size(), 0UZ));
        expect(eq(withCorr.size(), stream.size()));
    };

    "a detection and a forwarded tag leave a port as one ordered sequence"_test = [] {
        // The sync word ends on input 63, so its detection belongs at output 64: at a chunk of 32 that is the first
        // output of the following call, the same call that releases a forwarded tag held for a later offset. Both
        // reach the port, in offset order — a port rejects an index below the last one it took.
        constexpr std::size_t      kLength = 16UZ;
        constexpr std::size_t      kAt     = 48UZ;
        const std::vector<CF>      word    = syncWord(kLength, 0x5151ULL);
        const std::vector<CF>      stream  = plant(std::span<const CF>(word), kAt, 256UZ);
        const std::vector<gr::Tag> planted{gr::Tag{50UZ, tagKey(0)}, gr::Tag{100UZ, tagKey(1)}};

        const std::vector<std::size_t> want{kAt + kLength, 50UZ + kLength, 100UZ + kLength};
        for (const std::size_t chunk : {0UZ, 7UZ, 32UZ}) {
            PreambleCorrelator<float> block = make<PreambleCorrelator<float>>({{"sequence", interleaved(std::span<const CF>(word))}, {"threshold", 0.5f}});
            std::vector<float>        magnitude;
            std::vector<gr::Tag>      corrTags;
            const auto                seen = gr::blocks::testing::span::run3<CF, float>(block, std::span<const CF>(stream), chunk, magnitude, true, std::span<const gr::Tag>(planted), 0UZ, &corrTags);

            expect(that % (tagOffsets(seen.tags) == want)) << std::format("chunk {}: out published [{}] against [{}]", chunk, join(tagOffsets(seen.tags)), join(want));
            expect(that % std::ranges::is_sorted(tagOffsets(seen.tags))) << std::format("chunk {}: the indices a port takes never decrease", chunk);
            expect(that % (tagOffsets(corrTags) == want)) << std::format("chunk {}: corr shares the alignment and the sequence", chunk);
        }
    };

    "a held tag is published once per port, whatever the port count"_test = [] {
        // A chunk well short of the sequence length holds the tag over several calls, and a second connected port
        // adds nothing to what is held: each port takes the tag exactly once.
        constexpr std::size_t kLength = 32UZ;
        const std::vector<CF> word    = syncWord(kLength, 0x2222ULL);
        const std::vector<CF> stream(200UZ, CF{}); // silent, so the only tags on the ports are the forwarded ones

        const std::vector<gr::Tag> planted{gr::Tag{40UZ, tagKey(0)}};
        PreambleCorrelator<float>  block = make<PreambleCorrelator<float>>({{"sequence", interleaved(std::span<const CF>(word))}, {"threshold", 0.5f}});

        std::vector<float>   magnitude;
        std::vector<gr::Tag> corrTags;
        const auto           seen = gr::blocks::testing::span::run3<CF, float>(block, std::span<const CF>(stream), 8UZ, magnitude, true, std::span<const gr::Tag>(planted), 0UZ, &corrTags);

        expect(that % (tagOffsets(seen.tags) == std::vector<std::size_t>{72UZ})) << std::format("out published [{}]", join(tagOffsets(seen.tags)));
        expect(that % (tagOffsets(corrTags) == std::vector<std::size_t>{72UZ})) << std::format("corr published [{}]", join(tagOffsets(corrTags)));
    };

    "an arriving trigger_name tag does not retune the emitted label"_test = [] {
        // Every writable member is in the auto-update set, so a setting named for a reserved key follows any tag of
        // that key. The label is named trigger_label for exactly this reason.
        constexpr std::size_t kLength = 16UZ;
        constexpr std::size_t kAt     = 48UZ;
        const std::vector<CF> word    = syncWord(kLength, 0x5151ULL);
        const std::vector<CF> stream  = plant(std::span<const CF>(word), kAt, 256UZ);

        const gr::property_map::key_type triggerName{gr::tag::TRIGGER_NAME.shortKey()};
        const std::vector<gr::Tag>       planted{gr::Tag{4UZ, gr::property_map{{triggerName, gr::pmt::Value(std::string("access_code"))}}}};

        PreambleCorrelator<float> block = make<PreambleCorrelator<float>>({{"sequence", interleaved(std::span<const CF>(word))}, {"threshold", 0.5f}, {"trigger_label", std::string("preamble")}});
        block.settings().autoUpdate(planted[0UZ]);
        std::ignore = block.settings().applyStagedParameters();
        expect(eq(block.trigger_label.value, std::string("preamble"))) << "the arriving tag drives no setting of this block";

        std::vector<float>   magnitude;
        std::vector<gr::Tag> corrTags;
        const auto           seen = gr::blocks::testing::span::run3<CF, float>(block, std::span<const CF>(stream), 8UZ, magnitude, true, std::span<const gr::Tag>(planted), 0UZ, &corrTags);

        std::vector<std::string> labels;
        for (const gr::Tag& seenTag : seen.tags) {
            if (const auto found = seenTag.map.find(triggerName); found != seenTag.map.end()) {
                labels.push_back(found->second.value_or(std::string{}));
            }
        }
        expect(that % (labels == std::vector<std::string>{std::string("access_code"), std::string("preamble")})) << "the passing label is forwarded as it arrived and the detection carries the configured one";
    };

    "degenerate parameters throw"_test = [] {
        const std::vector<float> flat = interleaved(std::span<const CF>(syncWord(8UZ)));
        expect(throws([] {
            PreambleCorrelator<float> block{}; // no settings at all, so nothing is staged and nothing is validated
            block.start();
        })) << "there is no default sequence: a block without one refuses to start";
        expect(throws([] { std::ignore = make<PreambleCorrelator<float>>({{"sequence", std::vector<float>{1.0f, 0.0f, 1.0f}}}); })) << "an odd count is not a list of re,im pairs";
        expect(throws([&flat] { std::ignore = make<PreambleCorrelator<float>>({{"sequence", flat}, {"threshold", 0.0f}}); }));
        expect(throws([&flat] { std::ignore = make<PreambleCorrelator<float>>({{"sequence", flat}, {"threshold", -1.0f}}); }));
        expect(throws([&flat] { std::ignore = make<PreambleCorrelator<float>>({{"sequence", flat}, {"threshold_mode", std::string("relative")}, {"threshold", 1.0f}}); })) << "a multiple of the mean below one is not a threshold";
        expect(throws([&flat] { std::ignore = make<PreambleCorrelator<float>>({{"sequence", flat}, {"threshold_mode", std::string("dynamic")}}); }));
        expect(throws([&flat] { std::ignore = make<PreambleCorrelator<float>>({{"sequence", flat}, {"samples_per_symbol", 0.5f}}); }));
        expect(throws([] { std::ignore = make<PreambleCorrelator<float>>({{"sequence", std::vector<float>{0.0f, 0.0f, 0.0f, 0.0f}}}); })) << "a zero-energy sequence has no threshold that means anything";
        expect(nothrow([&flat] { std::ignore = make<PreambleCorrelator<float>>({{"sequence", flat}, {"threshold_mode", std::string("relative")}, {"threshold", 1.0001f}}); }));
    };

    "an absolute threshold is a fraction of the noiseless peak"_test = [] {
        constexpr std::size_t    kLength = 16UZ;
        const std::vector<CF>    word    = syncWord(kLength, 0xabcULL);
        const std::vector<float> flat    = interleaved(std::span<const CF>(word));

        for (const double amplitude : {0.5, 1.0, 2.0}) {
            const std::vector<CF>     stream = plant(std::span<const CF>(word), 40UZ, 200UZ, 0.0, amplitude);
            PreambleCorrelator<float> block  = make<PreambleCorrelator<float>>({{"sequence", flat}, {"threshold", 0.9f}});
            std::vector<float>        magnitude;
            const auto                seen = gr::blocks::testing::span::run3<CF, float>(block, std::span<const CF>(stream), 0UZ, magnitude);
            if (amplitude >= 1.0) {
                expect(eq(seen.tags.size(), 1UZ)) << std::format("amplitude {} clears 0.9 of the unit-amplitude peak", amplitude);
            } else {
                expect(eq(seen.tags.size(), 0UZ)) << "an AGC is a stated precondition: half amplitude is a quarter of the peak power and does not clear it";
            }
        }
    };
};

int main() { /* not needed for UT */ }
