#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <numbers>
#include <span>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/filter/ArbitraryRateResampler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::filter::ArbitraryRateResampler;
using CF       = std::complex<float>;
namespace test = gr::blocks::testing::span;

/// A 64-bit stream offset narrowed to an index; a template, so the cast stands where `std::size_t` is 32 bits and is
/// not a useless cast where the two are the same 64-bit type.
template<std::integral To, std::integral From>
[[nodiscard]] constexpr To narrowIndex(From value) noexcept {
    return static_cast<To>(value);
}

constexpr double kNineteenTwentyFourths = 19.0 / 24.0;

template<typename T>
[[nodiscard]] ArbitraryRateResampler<T> makeResampler(gr::property_map settings) {
    ArbitraryRateResampler<T> block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
    return block;
}

/**
 * @brief The design law's prototype at a stated length rather than a searched one.
 *
 * The search is what `designArbitraryResampler` is for and what its own suite pins; here the length is stated at twice
 * Kaiser's estimate so the targets are met with margin, and the costly search at `minRate = 0.2` stays out of tests
 * that are about ports, settings and tags rather than about filter design.
 */
[[nodiscard]] std::vector<float> prototypeFor(std::size_t bank, double minRate, double rolloff = 0.2) {
    const double      stopEdge = 0.5 * std::min(1.0, minRate) / static_cast<double>(bank);
    const double      passEdge = (1.0 - rolloff) * stopEdge;
    const std::size_t length   = (2UZ * gr::filter::fir::design::kaiserLength(60.0, stopEdge - passEdge)) | 1UZ;

    std::vector<float> taps = gr::filter::fir::design::kaiserLowpass(length, 0.5 * (passEdge + stopEdge), 60.0);
    for (float& v : taps) {
        v *= static_cast<float>(bank);
    }
    return taps;
}

/// @brief The design law's prototype at an exact length, so the delay `(length - 1) / 2` can be put on a chosen grid.
[[nodiscard]] std::vector<float> prototypeOfLength(std::size_t bank, double minRate, std::size_t length) {
    const double       stopEdge = 0.5 * std::min(1.0, minRate) / static_cast<double>(bank);
    const double       passEdge = 0.8 * stopEdge;
    std::vector<float> taps     = gr::filter::fir::design::kaiserLowpass(length, 0.5 * (passEdge + stopEdge), 60.0);
    for (float& v : taps) {
        v *= static_cast<float>(bank);
    }
    return taps;
}

/// @brief The phase argument of `mapArbitraryOffset` that adds a symmetric prototype's delay `(N-1)/2` to a regime starting at phase zero.
[[nodiscard]] std::int64_t delayedPhase(const std::vector<float>& prototype) { return -static_cast<std::int64_t>(prototype.size() - 1UZ) * (std::int64_t{1} << (gr::filter::kArbitraryFractionBits - 1)); }

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

struct Noise {
    std::uint64_t state = 0x2545F4914F6CDD1DULL;

    [[nodiscard]] float next() noexcept {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return static_cast<float>(static_cast<double>(state >> 11) / 9007199254740992.0 * 2.0 - 1.0);
    }
};

template<typename T>
[[nodiscard]] std::vector<T> noise(std::size_t n, std::uint64_t seed) {
    Noise          source{seed};
    std::vector<T> out(n);
    for (T& v : out) {
        if constexpr (std::same_as<T, CF>) {
            v = CF{source.next(), source.next()};
        } else {
            v = source.next();
        }
    }
    return out;
}

[[nodiscard]] std::uint64_t stepFor(std::size_t bank, double rate) { return static_cast<std::uint64_t>(std::llround(static_cast<double>(bank) / rate * static_cast<double>(gr::filter::kArbitraryOne))); }

[[nodiscard]] gr::property_map tagKey(std::size_t which) {
    gr::property_map map;
    map.insert_or_assign(gr::property_map::key_type{std::format("tag{}", which)}, static_cast<gr::Size_t>(which));
    return map;
}

[[nodiscard]] gr::property_map tagWithRate(std::size_t which, float rateIn) {
    gr::property_map map = tagKey(which);
    map.insert_or_assign(gr::property_map::key_type{gr::tag::SAMPLE_RATE.shortKey()}, gr::pmt::Value(rateIn));
    return map;
}

/// @brief The `sample_rate` values of the published tags carrying @p key, in publication order.
[[nodiscard]] std::vector<float> ratesOf(const std::vector<gr::Tag>& tags, std::string_view key) {
    const gr::property_map::key_type wanted{key};
    const gr::property_map::key_type rateKey{gr::tag::SAMPLE_RATE.shortKey()};

    std::vector<float> rates;
    for (const gr::Tag& tag : tags) {
        if (!tag.map.contains(wanted)) {
            continue;
        }
        if (const auto found = tag.map.find(rateKey); found != tag.map.end()) {
            rates.push_back(found->second.value_or(0.f));
        }
    }
    return rates;
}

[[nodiscard]] std::size_t countOwnKeys(const gr::Tag& tag) {
    return static_cast<std::size_t>(std::ranges::count_if(tag.map, [](const auto& entry) { return std::string_view(entry.first).starts_with("tag"); }));
}

[[nodiscard]] std::string join(const std::vector<std::size_t>& values) {
    std::string out;
    for (const std::size_t v : values) {
        out += std::format("{}{}", out.empty() ? "" : ", ", v);
    }
    return out;
}

/**
 * @brief An `Async` port driven one call at a time, so a settings change can be placed between two calls.
 *
 * `test::runAsync` drives a run from end to end, and a run cannot be cut in two here: the sample position, the tag
 * cursor and the output offsets all have to carry across the call a `rate` change falls between, and two runs restart
 * every one of them. Each call is that helper's — @p arriving further samples are offered, everything not yet consumed
 * is offered again, @p room output slots are free, and `forwardTags` runs before `processBulk` as the framework runs it.
 */
template<typename T>
struct AsyncRun {
    std::span<const T> input;
    test::TagCursor    cursor;
    test::Capture<T>   result{};
    std::size_t        fed = 0UZ;

    AsyncRun(std::span<const T> samples, std::span<const gr::Tag> tags) : input(samples), cursor(tags) {}

    [[nodiscard]] bool more() const noexcept { return result.consumed < input.size(); }

    template<typename TBlock>
    void call(TBlock& block, std::size_t arriving, std::size_t room) {
        fed                      = std::min(input.size(), fed + arriving);
        const std::size_t at     = result.consumed;
        const std::size_t window = fed - at;
        std::vector<T>    scratch(room);

        test::InputSpan<T>  inSpan(input.subspan(at, window), at, cursor.window(at, window), false);
        test::OutputSpan<T> outSpan(std::span<T>(scratch.data(), room), result.samples.size(), &result.tags, true, false);

        auto inputs  = std::tie(inSpan);
        auto outputs = std::tie(outSpan);
        block.forwardTags(inputs, outputs, window);
        std::ignore = block.processBulk(inSpan, outSpan);
        cursor.retire(inSpan);

        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        result.consumed += inSpan.consumed;
    }
};

} // namespace

const boost::ut::suite<"arbitrary resampler"> arbitraryResamplerTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::testing;
    using gr::filter::mapArbitraryOffset;
    using gr::filter::mapResampledOffset;

    "the realized ratio is exact and reported"_test = [] {
        constexpr std::size_t    kBank = 32UZ;
        const std::vector<float> taps  = prototypeFor(kBank, 0.2);

        struct Row {
            double      requested;
            const char* label;
        };
        const Row kRows[] = {{0.2, "1/5"}, {kNineteenTwentyFourths, "19/24"}, {48.0 / 44.1, "48/44.1"}, {0.7912345678, "no small form"}, {1.0 / std::numbers::pi, "1/pi"}};

        for (const Row& row : kRows) {
            ArbitraryRateResampler<float> block = makeResampler<float>({{"rate", row.requested}, {"bank_size", static_cast<gr::Size_t>(kBank)}, {"taps", taps}});

            const double want = static_cast<double>(kBank) * static_cast<double>(gr::filter::kArbitraryOne) / static_cast<double>(stepFor(kBank, row.requested));
            expect(that % (block.realizedRate() == want)) << row.label << ": realizedRate() is L*2^32/step to the last bit";
            expect(lt(std::abs(block.realizedRate() - row.requested) / row.requested, 1.2e-10)) << row.label << ": within the bound F = 32 sets";
        }

        // L/r a whole number is the case that is realized exactly, and 32/0.2 = 160 is one
        ArbitraryRateResampler<float> exact = makeResampler<float>({{"rate", 0.2}, {"bank_size", static_cast<gr::Size_t>(kBank)}, {"taps", taps}});
        expect(that % (exact.realizedRate() == 0.2)) << "0.2 at L = 32 is exactly 0.2";
    };

    "the counts are exact in both directions"_test = [] {
        constexpr std::size_t kBank = 16UZ;
        for (const double rate : {0.05, 0.2, 1.0, 1.7, 20.0}) {
            ArbitraryRateResampler<float> block = makeResampler<float>({{"rate", rate}, {"bank_size", static_cast<gr::Size_t>(kBank)}, {"taps", prototypeFor(kBank, rate)}});
            expect(eq(block.outputsFor(0UZ), 0UZ)) << rate;
            for (std::size_t n = 1UZ; n <= 100000UZ; n = n * 7UZ + 1UZ) {
                expect(le(block.inputsFor(block.outputsFor(n)), n + 1UZ)) << rate << ": inputsFor(outputsFor(" << n << "))";
                expect(ge(block.outputsFor(block.inputsFor(n)), n)) << rate << ": outputsFor(inputsFor(" << n << "))";
            }
        }
    };

    "feeding n samples yields outputsFor(n) outputs"_test = [] {
        constexpr std::size_t kBank = 32UZ;
        for (const double rate : {0.2, kNineteenTwentyFourths, 1.7}) {
            ArbitraryRateResampler<CF> block = makeResampler<CF>({{"rate", rate}, {"bank_size", static_cast<gr::Size_t>(kBank)}, {"taps", prototypeFor(kBank, rate)}});

            const std::vector<CF> x    = noise<CF>(8192UZ, 0xBF58476D1CE4E5B9ULL);
            const std::size_t     want = block.outputsFor(x.size());
            const auto            got  = test::runAsync<CF>(block, std::span<const CF>(x), 0UZ, want + 8UZ);
            expect(eq(got.samples.size(), want)) << rate;
            expect(eq(got.consumed, x.size())) << rate << ": every sample offered is consumed";
        }
    };

    "chunk independence is bit-identical"_test = [] {
        constexpr std::size_t kBank = 32UZ;
        for (const double rate : {0.2, kNineteenTwentyFourths, 1.7}) {
            const std::vector<float> taps = prototypeFor(kBank, rate);
            for (const gr::Size_t order : {1U, 3U}) {
                const gr::property_map settings{{"rate", rate}, {"bank_size", static_cast<gr::Size_t>(kBank)}, {"interpolation_order", order}, {"taps", taps}};

                ArbitraryRateResampler<CF> whole     = makeResampler<CF>(settings);
                const std::vector<CF>      x         = noise<CF>(4096UZ, 0x2545F4914F6CDD1DULL);
                const auto                 reference = test::runAsync<CF>(whole, std::span<const CF>(x), 0UZ, 4UZ * x.size() + 8UZ);
                expect(gt(reference.samples.size(), 0UZ));

                for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 4096UZ}) {
                    ArbitraryRateResampler<CF> block = makeResampler<CF>(settings);
                    const auto                 got   = test::runAsync<CF>(block, std::span<const CF>(x), chunk, chunk * 4UZ + 8UZ);
                    expect(eq(got.samples.size(), reference.samples.size())) << std::format("r = {}, q = {}, chunk {}: output count", rate, order, chunk);
                    expect(std::ranges::equal(got.samples, reference.samples)) << std::format("r = {}, q = {}, chunk {}: not bit-identical", rate, order, chunk);
                }
            }
        }
    };

    "a constant comes out a constant"_test = [] {
        constexpr std::size_t kBank = 32UZ;
        for (const double rate : {0.2, kNineteenTwentyFourths, 1.0, 48.0 / 44.1, 1.7}) {
            ArbitraryRateResampler<float> block = makeResampler<float>({{"rate", rate}, {"bank_size", static_cast<gr::Size_t>(kBank)}, {"taps", prototypeFor(kBank, rate)}});

            const std::vector<float> x   = std::vector<float>(8192UZ, 1.0f);
            const auto               got = test::runAsync<float>(block, std::span<const float>(x), 0UZ, block.outputsFor(x.size()) + 8UZ);
            expect(gt(got.samples.size(), 100UZ)) << rate;

            double worst = 0.0;
            for (std::size_t k = got.samples.size() / 2UZ; k < got.samples.size() - 1UZ; ++k) {
                worst = std::max(worst, std::abs(static_cast<double>(got.samples[k]) - 1.0));
            }
            expect(lt(worst, 0.0116)) << std::format("r = {}: settles {:g} from unity, past the 0.1 dB ripple target", rate, worst);
        }
    };

    "the offsets agree with the exact rational map where the step is whole"_test = [] {
        // `step` is a whole number of interpolated samples exactly when L*m/l is an integer, and there the arbitrary
        // map has to be mapResampledOffset — 1/5 at any L, 19/24 at L = 19
        for (const auto& [bank, l, m] : {std::tuple<std::size_t, std::uint64_t, std::uint64_t>{32UZ, 1ULL, 5ULL}, {19UZ, 19ULL, 24ULL}}) {
            const double        rate = static_cast<double>(l) / static_cast<double>(m);
            const std::uint64_t step = stepFor(bank, rate);
            expect(eq(step & gr::filter::kArbitraryMask, 0ULL)) << l << "/" << m << ": the step is whole";
            for (std::uint64_t i = 0ULL; i < 60ULL; ++i) {
                expect(eq(mapArbitraryOffset(i, bank, step, 0), mapResampledOffset(i, l, m))) << l << "/" << m << ": offset " << i;
            }
        }
    };

    "tags land where the rate change puts them"_test = [] {
        constexpr std::size_t kBank = 32UZ;
        for (const double rate : {0.2, 1.7}) {
            constexpr std::size_t kTags = 12UZ;
            gr::Graph             graph;

            auto& source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 2000U}, {"mark_tag", false}});
            for (std::size_t i = 0UZ; i < kTags; ++i) {
                source._tags.emplace_back(i, tagKey(i));
            }
            const std::vector<float> prototype = prototypeFor(kBank, rate);
            auto&                    resampler = graph.emplaceBlock<ArbitraryRateResampler<float>>({{"rate", rate}, {"bank_size", static_cast<gr::Size_t>(kBank)}, {"taps", prototype}});
            auto&                    sink      = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

            expect(graph.connect<"out", "in">(source, resampler).has_value());
            expect(graph.connect<"out", "in">(resampler, sink).has_value());

            gr::scheduler::Simple scheduler;
            expect(scheduler.exchange(std::move(graph)).has_value());
            expect(scheduler.runAndWait().has_value());

            const std::uint64_t      step = stepFor(kBank, rate);
            std::vector<std::size_t> want;
            for (std::size_t i = 0UZ; i < kTags; ++i) {
                const std::size_t at = narrowIndex<std::size_t>(mapArbitraryOffset(i, kBank, step, delayedPhase(prototype)));
                if (want.empty() || want.back() != at) {
                    want.push_back(at);
                }
            }

            std::vector<std::size_t> got;
            std::size_t              keysSeen = 0UZ;
            for (const gr::Tag& seenTag : sink._tags) {
                const std::size_t mine = countOwnKeys(seenTag);
                if (mine > 0UZ) {
                    got.push_back(seenTag.index);
                    keysSeen += mine;
                }
            }
            expect(that % (got == want)) << std::format("r = {}: output offsets [{}] against [{}]", rate, join(got), join(want));
            expect(eq(keysSeen, kTags)) << "r = " << rate << ": every tag survives, none merged away";
            if (rate < 1.0) {
                expect(lt(got.size(), kTags)) << "below unity several inputs share an output offset";
            } else {
                expect(eq(got.size(), kTags)) << "above unity consecutive tags land on distinct, increasing offsets";
            }
        }
    };

    "a forwarded sample_rate is multiplied by the rate"_test = [] {
        constexpr std::size_t kBank   = 32UZ;
        constexpr float       kRateIn = 480000.f;

        for (const double rate : {0.2, 1.7}) {
            gr::Graph graph;

            auto& source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 2000U}, {"mark_tag", false}});
            source._tags.emplace_back(200UZ, gr::property_map{{gr::property_map::key_type{gr::tag::SAMPLE_RATE.shortKey()}, gr::pmt::Value(kRateIn)}});

            auto& resampler = graph.emplaceBlock<ArbitraryRateResampler<float>>({{"rate", rate}, {"bank_size", static_cast<gr::Size_t>(kBank)}, {"taps", prototypeFor(kBank, rate)}});
            auto& sink      = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

            expect(graph.connect<"out", "in">(source, resampler).has_value());
            expect(graph.connect<"out", "in">(resampler, sink).has_value());

            gr::scheduler::Simple scheduler;
            expect(scheduler.exchange(std::move(graph)).has_value());
            expect(scheduler.runAndWait().has_value());

            std::vector<float> rates;
            for (const gr::Tag& seenTag : sink._tags) {
                if (seenTag.index == 0UZ) {
                    continue; // the source announces its own rate at offset 0
                }
                if (const auto found = seenTag.map.find(gr::property_map::key_type{gr::tag::SAMPLE_RATE.shortKey()}); found != seenTag.map.end()) {
                    rates.push_back(found->second.value_or(0.f));
                }
            }
            expect(that % (rates == std::vector<float>{static_cast<float>(rate * static_cast<double>(kRateIn))})) << std::format("r = {}: downstream reads the rate of the stream it is handed", rate);
        }
    };

    "a tag whose output falls past the call is published later"_test = [] {
        constexpr std::size_t         kBank     = 32UZ;
        const std::vector<float>      prototype = prototypeFor(kBank, 0.2);
        ArbitraryRateResampler<float> block     = makeResampler<float>({{"rate", 0.2}, {"bank_size", static_cast<gr::Size_t>(kBank)}, {"taps", prototype}});

        const std::vector<float>   x = noise<float>(400UZ, 0xD6E8FEB86659FD93ULL);
        const std::vector<gr::Tag> tags{gr::Tag{40UZ, tagKey(0)}, gr::Tag{41UZ, tagKey(1)}, gr::Tag{42UZ, tagKey(2)}};

        // one output slot a call, so every tag past the first output has to be held rather than moved
        const auto got = test::runAsync<float>(block, std::span<const float>(x), 8UZ, 1UZ, std::span<const gr::Tag>(tags));

        const std::uint64_t step = stepFor(kBank, 0.2);
        for (std::size_t i = 0UZ; i < 3UZ; ++i) {
            const std::vector<std::size_t> at{narrowIndex<std::size_t>(mapArbitraryOffset(40ULL + i, kBank, step, delayedPhase(prototype)))};
            expect(that % (got.offsetsOf(std::format("tag{}", i)) == at)) << "tag " << i << " held, not dropped and not moved";
        }
    };

    "two tags at one input offset both come through"_test = [] {
        constexpr std::size_t         kBank     = 32UZ;
        const std::vector<float>      prototype = prototypeFor(kBank, 0.5);
        ArbitraryRateResampler<float> block     = makeResampler<float>({{"rate", 0.5}, {"bank_size", static_cast<gr::Size_t>(kBank)}, {"taps", prototype}});

        const std::vector<float>   x = noise<float>(400UZ, 0x94D049BB133111EBULL);
        const std::vector<gr::Tag> tags{gr::Tag{40UZ, tagKey(0)}, gr::Tag{40UZ, tagKey(1)}, gr::Tag{41UZ, tagKey(2)}};

        // one output slot a call, so the window is offered again and again: a tag has to survive being presented
        // several times and still map exactly once
        const auto got = test::runAsync<float>(block, std::span<const float>(x), 8UZ, 1UZ, std::span<const gr::Tag>(tags));

        const std::uint64_t step = stepFor(kBank, 0.5);
        const std::size_t   at   = narrowIndex<std::size_t>(mapArbitraryOffset(40ULL, kBank, step, delayedPhase(prototype)));
        expect(that % (got.offsetsOf("tag0") == std::vector<std::size_t>{at}));
        expect(that % (got.offsetsOf("tag1") == std::vector<std::size_t>{at})) << "a second tag at an index already mapped is a tag of its own, not a repeat";
        expect(that % (got.offsetsOf("tag2") == std::vector<std::size_t>{narrowIndex<std::size_t>(mapArbitraryOffset(41ULL, kBank, step, delayedPhase(prototype)))}));
    };

    "a rate change keeps the position and re-origins the map"_test = [] {
        constexpr std::size_t         kBank     = 32UZ;
        const std::vector<float>      prototype = prototypeFor(kBank, 0.4);
        ArbitraryRateResampler<float> block     = makeResampler<float>({{"rate", 0.5}, {"bank_size", static_cast<gr::Size_t>(kBank)}, {"min_rate", 0.4}, {"taps", prototype}});

        const std::vector<float>   x = noise<float>(1000UZ, 0x94D049BB133111EBULL);
        const std::vector<gr::Tag> early{gr::Tag{100UZ, tagKey(0)}};
        const auto                 head = test::runAsync<float>(block, std::span<const float>(x).first(500UZ), 0UZ, 400UZ, std::span<const gr::Tag>(early));

        const std::uint64_t oldStep = stepFor(kBank, 0.5);
        const std::size_t   placed  = narrowIndex<std::size_t>(mapArbitraryOffset(100ULL, kBank, oldStep, delayedPhase(prototype)));
        expect(that % (head.offsetsOf("tag0") == std::vector<std::size_t>{placed}));

        const std::int64_t before = block.kernel().phase();
        const std::size_t  arm    = block.tapsPerArm();
        std::ignore               = block.settings().setStaged({{"rate", 0.75}});
        std::ignore               = block.settings().applyStagedParameters();

        expect(that % (block.kernel().phase() == before)) << "the step changed, not where the block is";
        expect(lt(std::abs(block.realizedRate() - 0.75) / 0.75, 1.2e-10)) << "and realizedRate() reports the new value";
        expect(eq(block.tapsPerArm(), arm)) << "the prototype is untouched: supplied taps are the caller's";

        const auto tail = test::runAsync<float>(block, std::span<const float>(x).subspan(500UZ), 0UZ, 600UZ, {}, 500UZ);
        expect(gt(tail.samples.size(), 0UZ)) << "and the stream continues with no gap";
        expect(eq(tail.consumed, 500UZ));
        expect(that % (head.offsetsOf("tag0") == std::vector<std::size_t>{placed})) << "tags placed before the change keep their offsets";
    };

    "a tag waiting behind the consumed prefix is mapped by the rate that consumes it"_test = [] {
        constexpr std::size_t  kBank   = 32UZ;
        constexpr float        kRateIn = 480000.f;
        constexpr double       kAfter  = 1.0;
        const gr::property_map settings{{"rate", 0.5}, {"bank_size", static_cast<gr::Size_t>(kBank)}, {"taps", prototypeFor(kBank, 0.5)}};

        // 300 samples, so the output input 80 lands on after the prototype's delay of about 72 inputs is produced
        const std::vector<float>   x = noise<float>(300UZ, 0x9E3779B97F4A7C15ULL);
        const std::vector<gr::Tag> tags{gr::Tag{80UZ, tagWithRate(0, kRateIn)}};

        // The probe: a span of 100 is visible from the first call and one output slot is free, so the call consumes a
        // prefix far short of input 80 and the tag is still waiting when `rate` changes under it.
        ArbitraryRateResampler<float> probe = makeResampler<float>(settings);
        AsyncRun<float>               probeRun{std::span<const float>(x), std::span<const gr::Tag>(tags)};
        probeRun.call(probe, 100UZ, 1UZ);

        const std::size_t prefix = probeRun.result.consumed;
        expect(gt(prefix, 0UZ)) << "the first call makes progress";
        expect(lt(prefix, 80UZ)) << "and stops well short of the tag's sample";
        expect(probeRun.result.offsetsOf("tag0").empty()) << "so the tag cannot have been published yet";

        std::ignore = probe.settings().setStaged({{"rate", kAfter}});
        std::ignore = probe.settings().applyStagedParameters();
        for (std::size_t guard = 0UZ; probeRun.more() && guard < 200UZ; ++guard) { // eight outputs a call over what is left
            probeRun.call(probe, 100UZ, 8UZ);
        }
        expect(!probeRun.more()) << "the probe consumes everything it was given";

        // The control: the same block and the same change at the same sample, but four samples arrive per call, so
        // the tag first becomes visible long after the change and has no earlier regime to be mapped under.
        ArbitraryRateResampler<float> control = makeResampler<float>(settings);
        AsyncRun<float>               controlRun{std::span<const float>(x), std::span<const gr::Tag>(tags)};
        controlRun.call(control, 4UZ, 1UZ);
        expect(eq(controlRun.result.consumed, prefix)) << "the two runs stand at the same sample when the rate changes";

        std::ignore = control.settings().setStaged({{"rate", kAfter}});
        std::ignore = control.settings().applyStagedParameters();
        for (std::size_t guard = 0UZ; controlRun.more() && guard < 200UZ; ++guard) {
            controlRun.call(control, 4UZ, 8UZ);
        }
        expect(!controlRun.more()) << "and so does the control";

        const std::vector<std::size_t> want = controlRun.result.offsetsOf("tag0");
        const std::vector<std::size_t> got  = probeRun.result.offsetsOf("tag0");
        expect(eq(want.size(), 1UZ)) << "the control publishes the tag exactly once";
        expect(that % (got == want)) << std::format("held past the change the tag lands at [{}], first seen after it at [{}]", join(got), join(want));

        const std::vector<float> rate{static_cast<float>(kAfter * static_cast<double>(kRateIn))};
        // the rate crosses as its own tag, unmoved, since it states a property of the stream and tag0 marks a position
        expect(that % (ratesOf(controlRun.result.tags, "sample_rate") == rate));
        expect(that % (ratesOf(probeRun.result.tags, "sample_rate") == rate)) << "and the rate is the one of the stream the block hands on where it is published";
    };

    "a tag that states a property of the stream crosses unmoved, and a trigger moves by the delay"_test = [] {
        // the rate describes output 0 as it describes input 0; the trigger leaves on the output nearest the prototype's
        // delay at the interpolated rate
        constexpr std::size_t    kBank     = 32UZ;
        const std::vector<float> prototype = prototypeFor(kBank, 0.5);
        for (const double rate : {0.5, 1.7}) {
            ArbitraryRateResampler<float> block = makeResampler<float>({{"rate", rate}, {"bank_size", static_cast<gr::Size_t>(kBank)}, {"taps", prototype}});
            gr::property_map              opening; // a source's opening tag: the stream's rate and the trigger of its first sample
            opening.insert_or_assign(gr::property_map::key_type{"sample_rate"}, 48000.0f);
            opening.insert_or_assign(gr::property_map::key_type{"trigger_time"}, std::uint64_t{1000});
            const std::vector<gr::Tag> tags{gr::Tag{0UZ, opening}};
            const std::vector<float>   x(400UZ, 0.0f);
            const auto                 got = test::runAsync<float>(block, std::span<const float>(x), 8UZ, 4UZ, std::span<const gr::Tag>(tags));

            expect(that % (got.offsetsOf("sample_rate") == std::vector<std::size_t>{0UZ})) << std::format("r = {}: the rate stays on output 0", rate);
            expect(that % (got.offsetsOf("trigger_time") == std::vector<std::size_t>{narrowIndex<std::size_t>(mapArbitraryOffset(0ULL, kBank, stepFor(kBank, rate), delayedPhase(prototype)))})) << std::format("r = {}: the trigger moves by the delay", rate);
        }
    };

    "a tag leaves on the output that carries its sample's energy"_test = [] {
        constexpr std::size_t kBank = 32UZ;

        // A prototype of 2*37*32 + 1 taps delays by 37 whole input samples. At a rate of 0.5 the step is 64 interpolated
        // samples, so an odd input i lands exactly on output (i + 37) / 2.
        const std::vector<float> onGrid = prototypeOfLength(kBank, 0.5, 2UZ * 37UZ * kBank + 1UZ);
        for (const std::size_t at : {9UZ, 63UZ}) {
            ArbitraryRateResampler<float> block = makeResampler<float>({{"rate", 0.5}, {"bank_size", static_cast<gr::Size_t>(kBank)}, {"taps", onGrid}});
            expect(eq(block.groupDelaySamples(), 37.0));

            std::vector<float> x(400UZ, 0.0f);
            x[at] = 1.0f;
            const std::vector<gr::Tag> tags{gr::Tag{at, tagKey(0)}};
            const auto                 got  = test::runAsync<float>(block, std::span<const float>(x), 8UZ, 4UZ, std::span<const gr::Tag>(tags));
            const std::size_t          want = (at + 37UZ) / 2UZ;

            expect(that % (got.offsetsOf("tag0") == std::vector<std::size_t>{want})) << std::format("r = 0.5: input {} leaves on output {} and on no other", at, want);
            expect(eq(peakIndex(std::span<const float>(got.samples)), want)) << std::format("r = 0.5: the impulse at input {} peaks on output {}", at, want);
        }

        // Above unity the outputs fall between the input samples: the tag lands on the output nearest the delayed position.
        const std::vector<float>      wide  = prototypeFor(kBank, 1.0);
        ArbitraryRateResampler<float> block = makeResampler<float>({{"rate", 1.7}, {"bank_size", static_cast<gr::Size_t>(kBank)}, {"taps", wide}});
        constexpr std::size_t         kAt   = 50UZ;
        const double                  delay = 0.5 * static_cast<double>(wide.size() - 1UZ) / static_cast<double>(kBank);
        const std::size_t             want  = narrowIndex<std::size_t>(std::llround((static_cast<double>(kAt) + delay) * block.realizedRate()));

        std::vector<float> x(400UZ, 0.0f);
        x[kAt] = 1.0f;
        const std::vector<gr::Tag> tags{gr::Tag{kAt, tagKey(0)}};
        const auto                 got = test::runAsync<float>(block, std::span<const float>(x), 8UZ, 4UZ, std::span<const gr::Tag>(tags));
        expect(that % (got.offsetsOf("tag0") == std::vector<std::size_t>{want})) << std::format("r = 1.7: input {} leaves on output {} and on no other", kAt, want);
        expect(eq(peakIndex(std::span<const float>(got.samples)), want)) << std::format("r = 1.7: the impulse at input {} peaks on output {}", kAt, want);
    };

    "an asymmetric supplied prototype moves its tags by the centroid of its energy"_test = [] {
        // a bank of one at a rate of one is a plain FIR filter: a lone tap at 3 of 5 delays by 3, not the 2 the length
        // alone gives
        ArbitraryRateResampler<float> block = makeResampler<float>({{"rate", 1.0}, {"bank_size", 1U}, {"taps", std::vector<float>{0.0f, 0.0f, 0.0f, 1.0f, 0.0f}}});
        expect(eq(block.groupDelaySamples(), 3.0));

        std::vector<float> x(64UZ, 0.0f);
        x[20] = 1.0f;
        const std::vector<gr::Tag> tags{gr::Tag{20UZ, tagKey(0)}};
        const auto                 got = test::runAsync<float>(block, std::span<const float>(x), 8UZ, 4UZ, std::span<const gr::Tag>(tags));
        expect(eq(peakIndex(std::span<const float>(got.samples)), 23UZ));
        expect(that % (got.offsetsOf("tag0") == std::vector<std::size_t>{23UZ}));
    };

    "the prototype is rebuilt when the rate falls below it"_test = [] {
        constexpr std::size_t         kBank = 32UZ;
        ArbitraryRateResampler<float> block = makeResampler<float>({{"rate", 1.0}, {"bank_size", static_cast<gr::Size_t>(kBank)}});

        const std::size_t designed = block.tapsPerArm();
        expect(gt(designed, 30UZ)) << "the designed prototype is about 37 taps an arm at 60 dB";

        // a constant into the block, so a rebuild that dropped the window would show as a dip rather than a seam
        const std::vector<float> x    = std::vector<float>(4000UZ, 1.0f);
        const auto               head = test::runAsync<float>(block, std::span<const float>(x), 0UZ, 5000UZ);
        expect(gt(head.samples.size(), 1000UZ));

        std::ignore = block.settings().setStaged({{"rate", 0.5}});
        std::ignore = block.settings().applyStagedParameters();

        expect(gt(block.tapsPerArm(), designed)) << "the prototype was rebuilt for the lower rate, not left cut for the old one";

        const auto tail = test::runAsync<float>(block, std::span<const float>(x), 0UZ, 5000UZ);
        expect(gt(tail.samples.size(), 100UZ));
        double worst = 0.0;
        for (std::size_t k = tail.samples.size() / 2UZ; k < tail.samples.size() - 1UZ; ++k) {
            worst = std::max(worst, std::abs(static_cast<double>(tail.samples[k]) - 1.0));
        }
        expect(lt(worst, 0.0116)) << std::format("the window was carried, not zeroed: settles {:g} from unity", worst);

        // a rate that rises back inside the designed band leaves the prototype alone
        const std::size_t widened = block.tapsPerArm();
        std::ignore               = block.settings().setStaged({{"rate", 0.9}});
        std::ignore               = block.settings().applyStagedParameters();
        expect(eq(block.tapsPerArm(), widened)) << "the prototype is narrower than the higher rate needs but cannot alias, so it is kept";
    };

    "a tone above the new output Nyquist is suppressed"_test = [] {
        constexpr std::size_t         kBank = 32UZ;
        ArbitraryRateResampler<float> block = makeResampler<float>({{"rate", 0.5}, {"bank_size", static_cast<gr::Size_t>(kBank)}});

        std::vector<float> x(1UZ << 14);
        for (std::size_t n = 0UZ; n < x.size(); ++n) { // 0.35 cycles per input sample: inside the input band, above the output Nyquist
            x[n] = static_cast<float>(std::cos(2.0 * std::numbers::pi * 0.35 * static_cast<double>(n)));
        }
        const auto got = test::runAsync<float>(block, std::span<const float>(x), 0UZ, block.outputsFor(x.size()) + 8UZ);

        double peak = 0.0;
        for (std::size_t k = got.samples.size() / 4UZ; k < got.samples.size(); ++k) {
            peak = std::max(peak, std::abs(static_cast<double>(got.samples[k])));
        }
        expect(lt(20.0 * std::log10(peak), -59.0)) << std::format("the alias comes out at {:.2f} dB, and the unscaled prototype passes it at 0 dB", 20.0 * std::log10(peak));
    };

    "a rate of one is not a pass-through"_test = [] {
        constexpr std::size_t         kBank = 32UZ;
        ArbitraryRateResampler<float> block = makeResampler<float>({{"rate", 1.0}, {"bank_size", static_cast<gr::Size_t>(kBank)}, {"taps", prototypeFor(kBank, 1.0)}});

        std::vector<float> x(4096UZ); // a tone well inside the passband, whose amplitude the arm-0 response has to keep
        for (std::size_t n = 0UZ; n < x.size(); ++n) {
            x[n] = static_cast<float>(std::cos(2.0 * std::numbers::pi * 0.05 * static_cast<double>(n)));
        }
        const auto got = test::runAsync<float>(block, std::span<const float>(x), 0UZ, 5000UZ);
        expect(gt(got.samples.size(), 3000UZ));

        bool   same = true;
        double peak = 0.0;
        for (std::size_t k = 1000UZ; k < got.samples.size(); ++k) {
            same = same && got.samples[k] == x[k];
            peak = std::max(peak, std::abs(static_cast<double>(got.samples[k])));
        }
        expect(!same) << "arm 0 is a unit-gain filter and not the identity; nobody may add a fast path that changes the answer";
        expect(lt(std::abs(peak - 1.0), 0.0116)) << std::format("and it is a unit-gain one: peak {:g}", peak);
    };

    "degenerate settings"_test = [] {
        expect(throws([] { std::ignore = makeResampler<float>({{"rate", 0.0}}); }));
        expect(throws([] { std::ignore = makeResampler<float>({{"rate", -1.0}}); }));
        expect(throws([] { std::ignore = makeResampler<float>({{"interpolation_order", 2U}}); })) << "0, 1 or 3";
        expect(throws([] { std::ignore = makeResampler<float>({{"rate", 1e30}, {"bank_size", 32U}, {"taps", std::vector<float>{1.0f}}}); })) << "a rate above L*2^32 rounds the step to nothing";

        // a bank of one is legal and is a plain fractional-delay filter
        ArbitraryRateResampler<float> single = makeResampler<float>({{"rate", 0.5}, {"bank_size", 1U}, {"taps", std::vector<float>{1.0f, 0.5f, 0.25f}}});
        expect(eq(single.bankSize(), 1UZ));
        expect(eq(single.tapsPerArm(), 3UZ));

        // and a prototype shorter than the bank is zero-padded and works
        ArbitraryRateResampler<float> stubby = makeResampler<float>({{"rate", 1.0}, {"bank_size", 8U}, {"taps", std::vector<float>{8.0f, 4.0f}}});
        expect(eq(stubby.tapsPerArm(), 1UZ));
        const std::vector<float> x   = noise<float>(64UZ, 0xA24BAED4963EE407ULL);
        const auto               got = test::runAsync<float>(stubby, std::span<const float>(x), 0UZ, 128UZ);
        expect(gt(got.samples.size(), 50UZ));
    };
};

int main() { /* tests are automatically registered and run */ }
