#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/filter/FirFilter.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::filter::FirFilter;
using CF       = std::complex<float>;
using CD       = std::complex<double>;
namespace test = gr::blocks::testing::span;

template<typename TSample, typename TTap>
[[nodiscard]] FirFilter<TSample, TTap> makeFir(gr::property_map settings) {
    FirFilter<TSample, TTap> block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
    return block;
}

template<typename TSample, typename TTap>
[[nodiscard]] auto run(FirFilter<TSample, TTap>& block, const std::vector<TSample>& input, std::size_t chunkOutputs = 0UZ) {
    const std::size_t m = block.decimation;
    return test::runDecimating<typename FirFilter<TSample, TTap>::TOut>(block, std::span<const TSample>(input), chunkOutputs * m, m);
}

[[nodiscard]] constexpr CD wide(float v) noexcept { return {static_cast<double>(v), 0.0}; }
[[nodiscard]] constexpr CD wide(CF v) noexcept { return {static_cast<double>(v.real()), static_cast<double>(v.imag())}; }

/// @brief `y[n] = sum_i taps[i]*x[n-i]`, `out[k] = y[k*m]`, evaluated in double against the definition.
template<typename TSample, typename TTap>
[[nodiscard]] std::vector<CD> definition(std::span<const TSample> x, std::span<const TTap> h, std::size_t m) {
    std::vector<CD> out(x.size() / m);
    for (std::size_t k = 0UZ; k < out.size(); ++k) {
        const std::size_t at = k * m;
        CD                acc{};
        for (std::size_t i = 0UZ; i <= at && i < h.size(); ++i) {
            acc += wide(h[i]) * wide(x[at - i]);
        }
        out[k] = acc;
    }
    return out;
}

/// @brief A deterministic stand-in for randomness: the same numbers on every machine and every run.
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

/// @brief An asymmetric tap set, so that a stray reversal or conjugation cannot survive the comparison.
template<typename TTap>
[[nodiscard]] std::vector<TTap> asymmetric(std::size_t n) {
    std::vector<TTap> h(n);
    for (std::size_t i = 0UZ; i < n; ++i) {
        if constexpr (std::same_as<TTap, CF>) {
            h[i] = CF{static_cast<float>(i + 1UZ), static_cast<float>(2UZ * i + 1UZ)};
        } else {
            h[i] = static_cast<float>(i + 1UZ);
        }
    }
    return h;
}

template<typename T>
[[nodiscard]] T impulse() {
    if constexpr (std::same_as<T, CF>) {
        return CF{1.0f, 0.0f};
    } else {
        return 1.0f;
    }
}

[[nodiscard]] double worstRelative(std::span<const CF> got, std::span<const CD> want) {
    double peak  = 0.0;
    double worst = 0.0;
    for (const CD& v : want) {
        peak = std::max(peak, std::abs(v));
    }
    for (std::size_t k = 0UZ; k < want.size(); ++k) {
        worst = std::max(worst, std::abs(wide(got[k]) - want[k]));
    }
    return peak > 0.0 ? worst / peak : worst;
}

[[nodiscard]] double worstRelative(std::span<const float> got, std::span<const CD> want) {
    std::vector<CF> lifted(got.size());
    std::ranges::transform(got, lifted.begin(), [](float v) { return CF{v, 0.0f}; });
    return worstRelative(std::span<const CF>(lifted), want);
}

/// A key per tag, so a tag that survives is visible even where several of them share an output offset.
[[nodiscard]] gr::property_map tagKey(std::size_t which) {
    gr::property_map map;
    map.insert_or_assign(gr::property_map::key_type{std::format("tag{}", which)}, static_cast<gr::Size_t>(which));
    return map;
}

[[nodiscard]] std::size_t countOwnKeys(const gr::Tag& tag) {
    return static_cast<std::size_t>(std::ranges::count_if(tag.map, [](const auto& entry) { return std::string_view(entry.first).starts_with("tag"); }));
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

/// @brief The absolute offsets at which @p sink received a tag carrying @p key.
template<typename TSink>
[[nodiscard]] std::vector<std::size_t> sinkOffsetsOf(const TSink& sink, std::string_view key) {
    std::vector<std::size_t> offsets;
    for (const gr::Tag& seenTag : sink._tags) {
        if (seenTag.map.contains(gr::property_map::key_type{key})) {
            offsets.push_back(seenTag.index);
        }
    }
    return offsets;
}

[[nodiscard]] std::string join(const std::vector<std::size_t>& values) {
    std::string out;
    for (const std::size_t v : values) {
        out += std::format("{}{}", out.empty() ? "" : ", ", v);
    }
    return out;
}

} // namespace

const boost::ut::suite<"fir filter"> firFilterTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::testing;
    using gr::filter::mapResampledOffset;

    "the impulse response is the taps"_test = []<typename TPair>() {
        using TSample = typename TPair::first_type;
        using TTap    = typename TPair::second_type;
        using TOut    = typename FirFilter<TSample, TTap>::TOut;

        const std::vector<TTap> h = asymmetric<TTap>(5UZ);
        for (const std::size_t at : {0UZ, 1UZ, 7UZ, 1000UZ}) {
            FirFilter<TSample, TTap> block = makeFir<TSample, TTap>({{"taps", h}, {"decimation", 1U}});

            std::vector<TSample> x(at + 2UZ * h.size(), TSample{});
            x[at] = impulse<TSample>();

            const auto y = run(block, x);
            expect(eq(y.samples.size(), x.size()));
            for (std::size_t i = 0UZ; i < h.size(); ++i) {
                expect(that % (y.samples[at + i] == static_cast<TOut>(h[i]))) << std::format("impulse at {}: output {} is not taps[{}]", at, at + i, i);
            }
            for (std::size_t k = 0UZ; k < at; ++k) {
                expect(that % (y.samples[k] == TOut{})) << "nothing before the impulse";
            }
        }
    } | std::tuple<std::pair<float, float>, std::pair<CF, float>, std::pair<float, CF>, std::pair<CF, CF>>{};

    "against the definition"_test = []<typename TPair>() {
        using TSample = typename TPair::first_type;
        using TTap    = typename TPair::second_type;

        const std::vector<TTap>    h = noise<TTap>(37UZ, 0x9E3779B97F4A7C15ULL);
        const std::vector<TSample> x = noise<TSample>(2560UZ, 0xBF58476D1CE4E5B9ULL);

        for (const gr::Size_t m : {1U, 2U, 10U, 64U}) {
            FirFilter<TSample, TTap> block = makeFir<TSample, TTap>({{"taps", h}, {"decimation", m}});
            const auto               y     = run(block, x);
            const std::vector<CD>    want  = definition<TSample, TTap>(x, h, m);
            using TOut                     = typename FirFilter<TSample, TTap>::TOut;

            expect(eq(y.samples.size(), want.size())) << "M = " << m;
            expect(lt(worstRelative(std::span<const TOut>(y.samples), std::span<const CD>(want)), 1e-6)) << "M = " << m << ": against a double evaluation of the definition";
        }
    } | std::tuple<std::pair<float, float>, std::pair<CF, float>, std::pair<float, CF>, std::pair<CF, CF>>{};

    "chunk independence is bit-identical"_test = []<typename TPair>() {
        using TSample = typename TPair::first_type;
        using TTap    = typename TPair::second_type;

        const std::vector<TTap>    h = noise<TTap>(37UZ, 0x94D049BB133111EBULL);
        const std::vector<TSample> x = noise<TSample>(40960UZ, 0x2545F4914F6CDD1DULL);

        for (const gr::Size_t m : {1U, 10U}) {
            FirFilter<TSample, TTap> whole     = makeFir<TSample, TTap>({{"taps", h}, {"decimation", m}});
            const auto               reference = run(whole, x);

            for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 4096UZ}) {
                FirFilter<TSample, TTap> block = makeFir<TSample, TTap>({{"taps", h}, {"decimation", m}});
                const auto               got   = run(block, x, chunk);
                expect(gt(got.samples.size(), 0UZ)) << "chunk " << chunk;
                expect(std::ranges::equal(got.samples, std::span(reference.samples).first(got.samples.size()))) << std::format("M = {}, chunk {}: not bit-identical", m, chunk);
            }
        }
    } | std::tuple<std::pair<float, float>, std::pair<CF, float>, std::pair<float, CF>, std::pair<CF, CF>>{};

    "the decimation phase"_test = [] {
        // a single unit tap at M = 10 keeps input q if and only if q is a multiple of ten, and keeps it at q/10
        FirFilter<float, float> keep = makeFir<float, float>({{"taps", std::vector<float>{1.0f}}, {"decimation", 10U}});
        std::vector<float>      x(200UZ, 0.0f);
        for (std::size_t q = 0UZ; q < x.size(); ++q) {
            x[q] = static_cast<float>(q) + 1.0f;
        }
        const auto kept = run(keep, x);
        expect(eq(kept.samples.size(), 20UZ));
        for (std::size_t k = 0UZ; k < kept.samples.size(); ++k) {
            expect(that % (kept.samples[k] == x[k * 10UZ])) << "output " << k << " is input " << k * 10UZ;
        }

        // and an impulse at input 0 against an N-tap set appears in output k with weight taps[k*M], which pins the
        // phase and the tap indexing together
        const std::vector<float> h     = asymmetric<float>(35UZ);
        FirFilter<float, float>  block = makeFir<float, float>({{"taps", h}, {"decimation", 10U}});
        std::vector<float>       delta(200UZ, 0.0f);
        delta[0]            = 1.0f;
        const auto response = run(block, delta);
        for (std::size_t k = 0UZ; k < response.samples.size(); ++k) {
            const float want = k * 10UZ < h.size() ? h[k * 10UZ] : 0.0f;
            expect(that % (response.samples[k] == want)) << "output " << k << " carries taps[" << k * 10UZ << "]";
        }
    };

    "tags land where the decimation puts them"_test = [] {
        constexpr std::size_t kTags = 22UZ;
        constexpr std::size_t kM    = 10UZ;
        gr::Graph             graph;

        auto& source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 4000U}, {"mark_tag", false}});
        for (std::size_t i = 0UZ; i < kTags; ++i) {
            source._tags.emplace_back(i, tagKey(i));
        }
        auto& filter = graph.emplaceBlock<FirFilter<float, float>>({{"taps", std::vector<float>{1.0f}}, {"decimation", static_cast<gr::Size_t>(kM)}});
        auto& sink   = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

        expect(graph.connect<"out", "in">(source, filter).has_value());
        expect(graph.connect<"out", "in">(filter, sink).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        // the worked tag map at M = 10, written out: 0,0,0,0,0,1,...,1,2,...,2
        constexpr std::size_t    kWorked[kTags] = {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2};
        std::vector<std::size_t> want;
        for (std::size_t i = 0UZ; i < kTags; ++i) {
            expect(eq(mapResampledOffset(i, 1ULL, kM), static_cast<std::uint64_t>(kWorked[i]))) << "input " << i;
            if (want.empty() || want.back() != kWorked[i]) {
                want.push_back(kWorked[i]);
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
        expect(that % (got == want)) << std::format("output offsets [{}] against [{}]", join(got), join(want));
        expect(eq(keysSeen, kTags)) << "every tag survives, in input order, none merged away";
    };

    "a forwarded sample_rate is divided by the decimation"_test = [] {
        constexpr std::size_t kM       = 10UZ;
        constexpr float       kRateIn  = 480000.f;
        constexpr float       kRateOut = kRateIn / static_cast<float>(kM);
        gr::Graph             graph;

        auto& source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 4000U}, {"mark_tag", false}});
        source._tags.emplace_back(200UZ, gr::property_map{{gr::property_map::key_type{gr::tag::SAMPLE_RATE.shortKey()}, gr::pmt::Value(kRateIn)}});

        auto& filter = graph.emplaceBlock<FirFilter<float, float>>({{"taps", std::vector<float>{1.0f}}, {"decimation", static_cast<gr::Size_t>(kM)}});
        auto& sink   = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

        expect(graph.connect<"out", "in">(source, filter).has_value());
        expect(graph.connect<"out", "in">(filter, sink).has_value());

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
        expect(that % (rates == std::vector<float>{kRateOut})) << "downstream reads the rate of the stream it is handed, not the one this block was fed";
    };

    "a tag whose output falls past the call is published later"_test = [] {
        constexpr std::size_t kM = 10UZ;

        FirFilter<float, float>  block = makeFir<float, float>({{"taps", std::vector<float>{1.0f}}, {"decimation", static_cast<gr::Size_t>(kM)}});
        const std::vector<float> x(100UZ, 0.25f);

        // input 15 maps to output 2, and arrives in the call that publishes output 1 — so it has to be held
        const std::vector<gr::Tag> tags{gr::Tag{15UZ, tagKey(0)}, gr::Tag{16UZ, tagKey(1)}};
        const auto                 got = test::runDecimating<float>(block, std::span<const float>(x), kM, kM, std::span<const gr::Tag>(tags));

        expect(eq(got.samples.size(), 10UZ));
        expect(eq(got.tags.size(), 2UZ)) << "held, not dropped";
        for (const gr::Tag& seenTag : got.tags) {
            expect(eq(seenTag.index, 2UZ)) << "both land on output 2, in input order";
        }
        expect(that % got.tags[0].map.contains(gr::property_map::key_type{"tag0"}));
        expect(that % got.tags[1].map.contains(gr::property_map::key_type{"tag1"}));
    };

    "the offset map is exact past 2^24 and 2^53"_test = [] {
        expect(eq(mapResampledOffset((1ULL << 24) + 1ULL, 1ULL, 1ULL), (1ULL << 24) + 1ULL));
        expect(eq(mapResampledOffset((1ULL << 53) + 1ULL, 1ULL, 1ULL), (1ULL << 53) + 1ULL));
        expect(eq(mapResampledOffset((1ULL << 53) + 1ULL, 1ULL, 2ULL), (1ULL << 52) + 1ULL));
    };

    "a taps change preserves the alignment"_test = [] {
        struct Row {
            std::size_t before, after;
        };
        constexpr Row kRows[] = {{8UZ, 13UZ}, {13UZ, 5UZ}, {8UZ, 8UZ}};
        for (const Row& row : kRows) {
            const std::vector<float> before = asymmetric<float>(row.before);
            const std::vector<float> after  = asymmetric<float>(row.after);
            const std::vector<float> x      = noise<float>(400UZ, 0xD6E8FEB86659FD93ULL);
            constexpr std::size_t    kSeam  = 200UZ;

            FirFilter<float, float> block = makeFir<float, float>({{"taps", before}, {"decimation", 1U}});
            const auto              head  = test::runDecimating<float>(block, std::span<const float>(x).first(kSeam), 0UZ, 1UZ);
            expect(eq(head.samples.size(), kSeam));

            std::ignore = block.settings().setStaged({{"taps", after}});
            std::ignore = block.settings().applyStagedParameters();

            const auto tail = test::runDecimating<float>(block, std::span<const float>(x).subspan(kSeam), 0UZ, 1UZ);
            expect(eq(tail.samples.size(), x.size() - kSeam)) << "the output count is unchanged by the tap change, and the first call after it produces";

            // past the point where the retained history covers the new window, the output is exactly the definition over the
            // actual input: nothing skipped, nothing repeated.
            const std::size_t settled = row.after > row.before ? row.after - row.before : 0UZ;
            for (std::size_t k = kSeam + settled; k < x.size(); ++k) {
                double want = 0.0;
                for (std::size_t i = 0UZ; i < after.size() && i <= k; ++i) {
                    want += static_cast<double>(after[i]) * static_cast<double>(x[k - i]);
                }
                expect(lt(std::abs(static_cast<double>(tail.samples[k - kSeam]) - want), 2e-4)) << std::format("{} -> {} taps: output {}", row.before, row.after, k);
            }

            // and inside the transient the deficit is the zero fill, not a shift: the same sum with the
            // unavailable history taken as zero
            for (std::size_t k = kSeam; k < kSeam + settled; ++k) {
                double want = 0.0;
                for (std::size_t i = 0UZ; i < after.size() && i <= k; ++i) {
                    if (k - i + row.before >= kSeam + 1UZ) {
                        want += static_cast<double>(after[i]) * static_cast<double>(x[k - i]);
                    }
                }
                expect(lt(std::abs(static_cast<double>(tail.samples[k - kSeam]) - want), 2e-4)) << std::format("{} -> {} taps: transient output {}", row.before, row.after, k);
            }
        }
    };

    "a decimation change re-origins the tag map"_test = [] {
        FirFilter<float, float>  block = makeFir<float, float>({{"taps", std::vector<float>{1.0f}}, {"decimation", 10U}});
        const std::vector<float> x(400UZ, 0.5f);

        const std::vector<gr::Tag> early{gr::Tag{5UZ, tagKey(0)}, gr::Tag{15UZ, tagKey(1)}};
        const auto                 head = test::runDecimating<float>(block, std::span<const float>(x).first(200UZ), 100UZ, 10UZ, std::span<const gr::Tag>(early));
        expect(eq(head.samples.size(), 20UZ));
        expect(that % (head.offsetsOf("tag0") == std::vector<std::size_t>{1UZ}));
        expect(that % (head.offsetsOf("tag1") == std::vector<std::size_t>{2UZ}));

        std::ignore = block.settings().setStaged({{"decimation", 4U}});
        std::ignore = block.settings().applyStagedParameters();

        // the new origin is input 200 / output 20, so input 208 is two outputs past it
        const std::vector<gr::Tag> late{gr::Tag{208UZ, tagKey(2)}};
        const auto                 tail = test::runDecimating<float>(block, std::span<const float>(x).subspan(200UZ), 100UZ, 4UZ, std::span<const gr::Tag>(late), 200UZ, 20UZ);
        expect(eq(tail.samples.size(), 50UZ)) << "200 further inputs at M = 4";
        expect(that % (tail.offsetsOf("tag2") == std::vector<std::size_t>{22UZ})) << "mapped from the new origin, not rescaled from zero";
    };

    "a tag that states a property of the stream crosses unmoved, and a trigger moves by the delay"_test = [] {
        // the rate describes output 0 as it describes input 0; the trigger leaves on the output that carries input 0's
        // energy, 15 samples later for 31 symmetric taps
        const std::vector<float> h      = gr::filter::fir::design::kaiserLowpass(31, 0.1, 60.0);
        constexpr std::size_t    kDelay = 15UZ;
        for (const std::size_t m : {1UZ, 4UZ}) {
            FirFilter<float, float> block = makeFir<float, float>({{"taps", h}, {"decimation", static_cast<gr::Size_t>(m)}});
            gr::property_map        opening; // a source's opening tag: the stream's rate and the trigger of its first sample
            opening.insert_or_assign(gr::property_map::key_type{"sample_rate"}, 48000.0f);
            opening.insert_or_assign(gr::property_map::key_type{"trigger_time"}, std::uint64_t{1000});
            const std::vector<gr::Tag> tags{gr::Tag{0UZ, opening}};
            const std::vector<float>   x(256UZ, 0.0f);
            const auto                 got = test::runDecimating<float>(block, std::span<const float>(x), 64UZ, m, std::span<const gr::Tag>(tags));

            expect(that % (got.offsetsOf("sample_rate") == std::vector<std::size_t>{0UZ})) << std::format("M = {}: the rate stays on output 0", m);
            expect(that % (got.offsetsOf("trigger_time") == std::vector<std::size_t>{(2UZ * kDelay + m) / (2UZ * m)})) << std::format("M = {}: the trigger moves by the delay", m);
        }
    };

    "a tag leaves on the output that carries its sample's energy"_test = [] {
        // a symmetric set of 31 taps delays by 15 input samples, so input i lands on output (i + 15) / M; each input
        // below is one whose delayed position is a whole output, and a call takes 64 inputs
        const std::vector<float> h      = gr::filter::fir::design::kaiserLowpass(31, 0.1, 60.0);
        constexpr std::size_t    kDelay = 15UZ;
        constexpr std::size_t    kChunk = 64UZ;

        struct Row {
            std::size_t m, at;
        };
        constexpr Row kRows[] = {{1UZ, 5UZ}, {1UZ, 62UZ}, {1UZ, 63UZ}, {4UZ, 5UZ}, {4UZ, 61UZ}};
        for (const auto& [m, at] : kRows) {
            FirFilter<float, float> block = makeFir<float, float>({{"taps", h}, {"decimation", static_cast<gr::Size_t>(m)}});
            expect(eq(block.groupDelaySamples(), static_cast<double>(kDelay)));

            std::vector<float> x(256UZ, 0.0f);
            x[at] = 1.0f;
            const std::vector<gr::Tag> tags{gr::Tag{at, tagKey(0)}};
            const auto                 got  = test::runDecimating<float>(block, std::span<const float>(x), kChunk, m, std::span<const gr::Tag>(tags));
            const std::size_t          want = (at + kDelay) / m;

            expect(that % (got.offsetsOf("tag0") == std::vector<std::size_t>{want})) << std::format("M = {}: input {} leaves on output {} and on no other", m, at, want);
            expect(eq(peakIndex(std::span<const float>(got.samples)), want)) << std::format("M = {}: the impulse at input {} peaks on output {}", m, at, want);
            if (at + m >= kChunk) { // the tag rides the first call's last output group
                expect(ge(want, kChunk / m)) << "the delayed output belongs to a later call than the tag's input";
            }
        }
    };

    "an even-length set delays by a half sample, and the half rounds up"_test = [] {
        // {1, 2, 2, 1} delays by 1.5: the impulse at 63 comes out 1, 2, 2, 1 on outputs 63 to 66, and the tag goes to the
        // later of the two equal peaks, in the call after the one that carried it
        FirFilter<float, float> block = makeFir<float, float>({{"taps", std::vector<float>{1.0f, 2.0f, 2.0f, 1.0f}}, {"decimation", 1U}});
        expect(eq(block.groupDelaySamples(), 1.5));

        std::vector<float> x(128UZ, 0.0f);
        x[63] = 1.0f;
        const std::vector<gr::Tag> tags{gr::Tag{63UZ, tagKey(0)}};
        const auto                 got = test::runDecimating<float>(block, std::span<const float>(x), 64UZ, 1UZ, std::span<const gr::Tag>(tags));

        expect(eq(got.samples[64], got.samples[65])) << "the energy is split evenly between outputs 64 and 65";
        expect(that % (got.offsetsOf("tag0") == std::vector<std::size_t>{65UZ}));
    };

    "an asymmetric set moves its tags by the centroid of its energy"_test = [] {
        struct Row {
            std::vector<float> taps;
            std::size_t        delay; // where the impulse comes out largest
        };
        // a lone tap at 5 of 9 is a pure delay of 5, not the 4 the length alone gives; {4, 2, 1} has its centroid at 2/7
        // and delays by none
        const Row rows[] = {{{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}, 5UZ}, {{4.0f, 2.0f, 1.0f}, 0UZ}};
        for (const Row& row : rows) {
            FirFilter<float, float> block = makeFir<float, float>({{"taps", row.taps}, {"decimation", 1U}});
            expect(eq(block.groupDelaySamples(), static_cast<double>(row.delay)));

            std::vector<float> x(128UZ, 0.0f);
            x[62] = 1.0f;
            const std::vector<gr::Tag> tags{gr::Tag{62UZ, tagKey(0)}};
            const auto                 got = test::runDecimating<float>(block, std::span<const float>(x), 64UZ, 1UZ, std::span<const gr::Tag>(tags));

            expect(eq(peakIndex(std::span<const float>(got.samples)), 62UZ + row.delay));
            expect(that % (got.offsetsOf("tag0") == std::vector<std::size_t>{62UZ + row.delay})) << row.taps.size() << " taps";
        }
    };

    "a tag whose delayed output lies past the end of the stream is not published"_test = [] {
        constexpr gr::Size_t kSamples = 1000U;
        gr::Graph            graph;

        // 31 taps delay by 15: input 900 leaves on output 915, and input 990 would leave on output 1005, which a stream of
        // 1000 samples never produces
        auto& source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", kSamples}, {"mark_tag", false}});
        source._tags.emplace_back(900UZ, tagKey(0));
        source._tags.emplace_back(990UZ, tagKey(1));
        auto& filter = graph.emplaceBlock<FirFilter<float, float>>({{"taps", gr::filter::fir::design::kaiserLowpass(31, 0.1, 60.0)}, {"decimation", 1U}});
        auto& sink   = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

        expect(graph.connect<"out", "in">(source, filter).has_value());
        expect(graph.connect<"out", "in">(filter, sink).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        expect(eq(sink._samples.size(), static_cast<std::size_t>(kSamples)));
        expect(that % (sinkOffsetsOf(sink, "tag0") == std::vector<std::size_t>{915UZ})) << "a delayed tag inside the stream arrives through the scheduler";
        expect(that % sinkOffsetsOf(sink, "tag1").empty()) << "the stream ends with the tag's energy still inside the filter";
    };

    "a taps change moves no held tag, and a later tag never lands ahead of it"_test = [] {
        const std::vector<float> longer  = gr::filter::fir::design::kaiserLowpass(31, 0.1, 60.0); // delays by 15
        const std::vector<float> shorter = gr::filter::fir::design::kaiserLowpass(5, 0.1, 60.0);  // delays by 2
        const std::vector<float> x       = noise<float>(400UZ, 0xD1B54A32D192ED03ULL);

        FirFilter<float, float>    block = makeFir<float, float>({{"taps", longer}, {"decimation", 1U}});
        const std::vector<gr::Tag> early{gr::Tag{195UZ, tagKey(0)}};
        const auto                 head = test::runDecimating<float>(block, std::span<const float>(x).first(200UZ), 50UZ, 1UZ, std::span<const gr::Tag>(early));
        expect(that % head.offsetsOf("tag0").empty()) << "input 195 leaves on output 210, which the first 200 inputs do not produce";

        std::ignore = block.settings().setStaged({{"taps", shorter}});
        std::ignore = block.settings().applyStagedParameters();
        expect(eq(block.groupDelaySamples(), 2.0));

        // under the new delay input 201 maps to 203, ahead of the held tag, and input 250 to 252
        const std::vector<gr::Tag> late{gr::Tag{201UZ, tagKey(1)}, gr::Tag{250UZ, tagKey(2)}};
        const auto                 tail = test::runDecimating<float>(block, std::span<const float>(x).subspan(200UZ), 50UZ, 1UZ, std::span<const gr::Tag>(late), 200UZ, 200UZ);
        expect(that % (tail.offsetsOf("tag0") == std::vector<std::size_t>{210UZ})) << "held across the change, at the output it crossed with";
        expect(that % (tail.offsetsOf("tag1") == std::vector<std::size_t>{210UZ})) << "placed with the held tag, never ahead of it";
        expect(that % (tail.offsetsOf("tag2") == std::vector<std::size_t>{252UZ})) << "and past it, by the new delay";

        std::vector<std::size_t> order;
        for (const gr::Tag& published : tail.tags) {
            order.push_back(published.index);
        }
        expect(std::ranges::is_sorted(order)) << std::format("published in index order: [{}]", join(order));
        expect(!tail.tags.empty() && tail.tags.front().map.contains(gr::property_map::key_type{"tag0"})) << "the tag that crossed first leaves first";
    };

    "the frequency-translating identity"_test = [] {
        constexpr int            kTaps = 143;
        constexpr gr::Size_t     kD    = 10U;
        const double             w0    = 2.0 * std::numbers::pi * 0.2375;
        const double             phi   = w0 * 0.5 * static_cast<double>(kTaps - 1);
        const std::vector<float> h     = gr::filter::fir::design::kaiserLowpass(kTaps, 0.02, 60.0);

        std::vector<CF> composite(h.size());
        for (std::size_t i = 0UZ; i < h.size(); ++i) {
            composite[i] = CF{static_cast<float>(static_cast<double>(h[i]) * std::cos(w0 * static_cast<double>(i))), static_cast<float>(static_cast<double>(h[i]) * std::sin(w0 * static_cast<double>(i)))};
        }

        const std::vector<float> x = noise<float>(20000UZ, 0xA24BAED4963EE407ULL);

        FirFilter<float, CF> folded = makeFir<float, CF>({{"taps", composite}, {"decimation", kD}});
        auto                 y      = run(folded, x);
        for (std::size_t k = 0UZ; k < y.samples.size(); ++k) { // derotate at the output rate, from the same initial phase
            const double angle = -(phi + w0 * static_cast<double>(kD) * static_cast<double>(k));
            y.samples[k] *= CF{static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle))};
        }

        std::vector<CF> rotated(x.size()); // the same constant folded into the input rotator's initial phase
        for (std::size_t n = 0UZ; n < x.size(); ++n) {
            const double angle = -(phi + w0 * static_cast<double>(n));
            rotated[n]         = CF{static_cast<float>(static_cast<double>(x[n]) * std::cos(angle)), static_cast<float>(static_cast<double>(x[n]) * std::sin(angle))};
        }
        FirFilter<CF, float> composed = makeFir<CF, float>({{"taps", h}, {"decimation", kD}});
        const auto           v        = run(composed, rotated);

        expect(eq(y.samples.size(), v.samples.size()));
        double rms   = 0.0;
        double worst = 0.0;
        for (std::size_t k = 0UZ; k < v.samples.size(); ++k) {
            rms += static_cast<double>(std::norm(v.samples[k]));
            worst = std::max(worst, static_cast<double>(std::abs(y.samples[k] - v.samples[k])));
        }
        rms = std::sqrt(rms / static_cast<double>(v.samples.size()));
        expect(lt(worst, 1e-5 * rms)) << std::format("the fold and the composition are the same filter: residual {:g} against rms {:g}", worst, rms);
    };

    "degenerate settings"_test = [] {
        expect(throws([] { std::ignore = makeFir<float, float>({{"taps", std::vector<float>{1.0f}}, {"decimation", 0U}}); }));
        expect(throws([] { std::ignore = makeFir<float, float>({{"taps", std::vector<float>{}}, {"decimation", 1U}}); }));

        FirFilter<float, float> live = makeFir<float, float>({{"taps", std::vector<float>{1.0f}}, {"decimation", 1U}});
        expect(throws([&live] {
            std::ignore = live.settings().setStaged({{"decimation", 0U}});
            std::ignore = live.settings().applyStagedParameters();
        })) << "and on a live change";

        // a unit tap at M = 1 is a bit-exact pass-through, tags included
        FirFilter<CF, float>       through = makeFir<CF, float>({{"taps", std::vector<float>{1.0f}}, {"decimation", 1U}});
        const std::vector<CF>      x       = noise<CF>(500UZ, 0x3C79AC492BA7B653ULL);
        const std::vector<gr::Tag> tags{gr::Tag{7UZ, tagKey(0)}, gr::Tag{300UZ, tagKey(1)}};
        const auto                 got = test::runDecimating<CF>(through, std::span<const CF>(x), 100UZ, 1UZ, std::span<const gr::Tag>(tags));
        expect(that % (got.samples == x)) << "bit for bit";
        expect(that % (got.offsetsOf("tag0") == std::vector<std::size_t>{7UZ}));
        expect(that % (got.offsetsOf("tag1") == std::vector<std::size_t>{300UZ}));
    };
};

int main() { /* tests are automatically registered and run */ }
