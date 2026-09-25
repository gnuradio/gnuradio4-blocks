#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdlib>
#include <format>
#include <numeric>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/filter/RationalResampler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::filter::RationalResampler;
namespace test = gr::blocks::testing::span;

/// A 64-bit stream offset narrowed to an index; a template, so the cast stands where `std::size_t` is 32 bits and is
/// not a useless cast where the two are the same 64-bit type.
template<std::integral To, std::integral From>
[[nodiscard]] constexpr To narrowIndex(From value) noexcept {
    return static_cast<To>(value);
}

template<typename T>
[[nodiscard]] RationalResampler<T> makeResampler(gr::property_map settings) {
    RationalResampler<T> block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

/// Feed @p chunks whole chunks of `decimation` samples, which is what the framework hands a resampler.
template<typename T>
[[nodiscard]] std::vector<T> run(RationalResampler<T>& block, const std::vector<T>& input) {
    const std::size_t m = block.input_chunk_size;
    const std::size_t l = block.output_chunk_size;
    const std::size_t n = input.size() / m;

    std::vector<T> output(n * l);
    for (std::size_t k = 0UZ; k < n; ++k) {
        std::ignore = block.processBulk(std::span<const T>(input.data() + k * m, m), std::span<T>(output.data() + k * l, l));
    }
    return output;
}

/**
 * @brief Drive the block over @p input in calls of @p chunks whole `M`-in `L`-out chunks, which the framework hands it.
 *
 * @p startIn and @p startOut put the run at an absolute stream position, which is what the tag-offset tests need and
 * what no scheduler-driven test can reach: after a mid-stream ratio change the outputs already produced are neither
 * `startIn` nor a fixed multiple of it.
 */
template<typename T>
[[nodiscard]] test::Capture<T> runChunks(RationalResampler<T>& block, std::span<const T> input, std::size_t chunks, std::span<const gr::Tag> tags = {}, std::size_t startIn = 0UZ, std::size_t startOut = 0UZ) {
    const std::size_t m      = block.input_chunk_size;
    const std::size_t l      = block.output_chunk_size;
    const std::size_t stride = std::max(chunks, 1UZ) * m;

    test::Capture<T> result;
    std::vector<T>   scratch(stride / m * l);
    for (std::size_t base = 0UZ; base + m <= input.size(); base += stride) {
        const std::size_t take  = std::min(stride, ((input.size() - base) / m) * m);
        const std::size_t made  = take / m * l;
        const auto        first = std::ranges::lower_bound(tags, startIn + base, std::ranges::less{}, &gr::Tag::index);
        const auto        last  = std::ranges::lower_bound(tags, startIn + base + take, std::ranges::less{}, &gr::Tag::index);

        test::InputSpan<T>  inSpan(input.subspan(base, take), startIn + base, std::span<const gr::Tag>(first, last));
        test::OutputSpan<T> outSpan(std::span<T>(scratch.data(), made), startOut + base / m * l, &result.tags);

        auto inputs  = std::tie(inSpan);
        auto outputs = std::tie(outSpan);
        block.forwardTags(inputs, outputs, take);
        std::ignore = block.processBulk(std::span<const T>(inSpan), std::span<T>(outSpan));

        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(made));
        result.consumed += take;
    }
    return result;
}

/// A key per tag, so a tag that survives is visible even where two of them share an output offset.
constexpr const char* kTagKeys[] = {"t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7"};

[[nodiscard]] gr::property_map tagKey(std::size_t which) { return {{kTagKeys[which], static_cast<gr::Size_t>(which)}}; }

[[nodiscard]] std::string join(const std::vector<std::size_t>& values) {
    std::string out;
    for (const std::size_t v : values) {
        out += std::format("{}{}", out.empty() ? "" : ", ", v);
    }
    return out;
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

[[nodiscard]] std::size_t countOwnKeys(const gr::Tag& tag) {
    return static_cast<std::size_t>(std::ranges::count_if(tag.map, [](const auto& entry) { return std::ranges::contains(kTagKeys, entry.first); }));
}

} // namespace

const boost::ut::suite<"rational resampler"> rationalResamplerTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::testing;
    using gr::filter::mapResampledOffset;

    "the ratio is exact and the chunk sizes say so"_test = [] {
        struct Row {
            gr::Size_t l, m, chunkIn, chunkOut;
        };
        constexpr Row kRows[] = {{1U, 1U, 1U, 1U}, {3U, 2U, 2U, 3U}, {2U, 3U, 3U, 2U}, {1U, 4U, 4U, 1U}, {4U, 5U, 5U, 4U}, {6U, 4U, 2U, 3U}};
        for (const Row& row : kRows) {
            RationalResampler<float> block = makeResampler<float>({{"interpolation", row.l}, {"decimation", row.m}});
            expect(eq(block.input_chunk_size.value, row.chunkIn)) << row.l << "/" << row.m << ": the ratio is reduced where the taps are designed";
            expect(eq(block.output_chunk_size.value, row.chunkOut)) << row.l << "/" << row.m;

            const std::vector<float> x(1000UZ * row.chunkIn, 0.25f);
            expect(eq(run(block, x).size(), 1000UZ * row.chunkOut)) << row.l << "/" << row.m << ": no drift over a thousand chunks";
        }
    };

    "a designed filter carries the gain of L"_test = [] {
        for (const gr::Size_t l : {1U, 2U, 3U, 4U}) {
            RationalResampler<float> block = makeResampler<float>({{"interpolation", l}, {"decimation", 1U}});

            const std::vector<float> x(4000UZ, 1.0f);
            const std::vector<float> y = run(block, x);
            expect(gt(y.size(), 0UZ));

            // past the rise out of the zeroed history a constant comes out a constant of the same
            // size: zero-stuffing divides the amplitude by L and the design puts it back
            double worst = 0.0;
            for (std::size_t k = y.size() / 2UZ; k < y.size(); ++k) {
                worst = std::max(worst, std::abs(static_cast<double>(y[k]) - 1.0));
            }
            expect(lt(worst, 0.002)) << "interpolating by " << l << ": settles " << worst << " from unity";
        }
    };

    "supplied taps are used as given, gain included"_test = [] {
        // A single unit tap at 2/1 is the zero-stuffing itself: no filter, no factor of two. That is
        // the asymmetry stated at the `taps` parameter.
        RationalResampler<float> block = makeResampler<float>({{"interpolation", 2U}, {"decimation", 1U}, {"taps", std::vector<float>{1.0f}}});
        expect(eq(block.input_chunk_size.value, 1U));
        expect(eq(block.output_chunk_size.value, 2U));

        const std::vector<float> x{1.0f, 2.0f, 3.0f, 4.0f};
        const std::vector<float> y = run(block, x);
        expect(eq(y.size(), 8UZ));
        expect(that % (y == std::vector<float>{1.0f, 0.0f, 2.0f, 0.0f, 3.0f, 0.0f, 4.0f, 0.0f})) << "no filter is designed over supplied taps, so the stuffed zeros remain";

        // and a ratio with a common factor is not reduced when the taps came from outside
        RationalResampler<float> unreduced = makeResampler<float>({{"interpolation", 6U}, {"decimation", 4U}, {"taps", std::vector<float>{1.0f}}});
        expect(eq(unreduced.input_chunk_size.value, 4U));
        expect(eq(unreduced.output_chunk_size.value, 6U));
    };

    "a ratio of one with a unit tap is a pass-through"_test = [] {
        RationalResampler<float> block = makeResampler<float>({{"interpolation", 1U}, {"decimation", 1U}, {"taps", std::vector<float>{1.0f}}});

        std::vector<float> x(500UZ);
        for (std::size_t i = 0UZ; i < x.size(); ++i) {
            x[i] = static_cast<float>(i) * 0.013f - 3.0f;
        }
        expect(that % (run(block, x) == x)) << "bit for bit";

        // an empty tap vector at a ratio of one is the same pass-through, not an error
        RationalResampler<float> designed = makeResampler<float>({{"interpolation", 1U}, {"decimation", 1U}});
        expect(that % (run(designed, x) == x));
    };

    "changing the ratio mid-stream rebuilds and keeps producing"_test = [] {
        RationalResampler<float> block = makeResampler<float>({{"interpolation", 1U}, {"decimation", 1U}, {"taps", std::vector<float>{1.0f}}});

        const std::vector<float> x(600UZ, 0.5f);
        expect(eq(run(block, x).size(), 600UZ));

        std::ignore = block.settings().setStaged({{"interpolation", 3U}, {"decimation", 2U}});
        std::ignore = block.settings().applyStagedParameters();

        expect(eq(block.input_chunk_size.value, 2U)) << "the new ratio is announced to the scheduler";
        expect(eq(block.output_chunk_size.value, 3U));
        expect(eq(run(block, x).size(), 900UZ)) << "and the block keeps producing at it";
    };

    "a ratio change re-origins the tag map"_test = [] {
        RationalResampler<float> block = makeResampler<float>({{"interpolation", 1U}, {"decimation", 1U}, {"taps", std::vector<float>{1.0f}}});
        const std::vector<float> x(1500UZ, 0.5f);

        const std::vector<gr::Tag> early{gr::Tag{100UZ, tagKey(0)}};
        const auto                 head = runChunks<float>(block, std::span<const float>(x).first(600UZ), 50UZ, std::span<const gr::Tag>(early));
        expect(eq(head.samples.size(), 600UZ));
        expect(that % (head.offsetsOf("t0") == std::vector<std::size_t>{100UZ}));

        std::ignore = block.settings().setStaged({{"interpolation", 3U}, {"decimation", 2U}});
        std::ignore = block.settings().applyStagedParameters();
        expect(eq(block.input_chunk_size.value, 2U));

        // the new origin is input 600 / output 600, so input 604 is four inputs past it and 3/2 puts it six outputs on
        const std::vector<gr::Tag> rising = {gr::Tag{604UZ, tagKey(1)}};
        const auto                 middle = runChunks<float>(block, std::span<const float>(x).subspan(600UZ, 600UZ), 50UZ, std::span<const gr::Tag>(rising), 600UZ, 600UZ);
        expect(eq(middle.samples.size(), 900UZ)) << "600 further inputs at 3/2";
        expect(that % (middle.offsetsOf("t1") == std::vector<std::size_t>{606UZ})) << "mapped from the new origin, not rescaled from zero";

        std::ignore = block.settings().setStaged({{"interpolation", 1U}, {"decimation", 3U}});
        std::ignore = block.settings().applyStagedParameters();

        // and on the way down: the origin is now input 1200 / output 1500, so input 1215 is five outputs past it
        const std::vector<gr::Tag> falling = {gr::Tag{1215UZ, tagKey(2)}};
        const auto                 tail    = runChunks<float>(block, std::span<const float>(x).subspan(1200UZ), 50UZ, std::span<const gr::Tag>(falling), 1200UZ, 1500UZ);
        expect(eq(tail.samples.size(), 100UZ)) << "300 further inputs at 1/3";
        expect(that % (tail.offsetsOf("t2") == std::vector<std::size_t>{1505UZ})) << "and a falling ratio does not pile its tags on the first output of the call";
    };

    "a forwarded sample_rate is multiplied by the ratio in force where the tag crossed"_test = [] {
        const gr::property_map::key_type rateKey{gr::tag::SAMPLE_RATE.shortKey()};
        constexpr float                  kRateIn = 480000.f;

        const auto ratesOf = [&rateKey](const test::Capture<float>& capture) {
            std::vector<float> rates;
            for (const gr::Tag& seenTag : capture.tags) {
                if (const auto found = seenTag.map.find(rateKey); found != seenTag.map.end()) {
                    rates.push_back(found->second.value_or(0.f));
                }
            }
            return rates;
        };
        const auto rateTag = [&rateKey](std::size_t at) { return gr::Tag{at, gr::property_map{{rateKey, gr::pmt::Value(kRateIn)}}}; };

        RationalResampler<float> block = makeResampler<float>({{"interpolation", 1U}, {"decimation", 1U}, {"taps", std::vector<float>{1.0f}}});
        const std::vector<float> x(1500UZ, 0.5f);

        const std::vector<gr::Tag> early{rateTag(100UZ)};
        const auto                 head = runChunks<float>(block, std::span<const float>(x).first(600UZ), 50UZ, std::span<const gr::Tag>(early));
        expect(that % (ratesOf(head) == std::vector<float>{kRateIn})) << "a ratio of one leaves the rate alone";

        std::ignore = block.settings().setStaged({{"interpolation", 3U}, {"decimation", 2U}});
        std::ignore = block.settings().applyStagedParameters();

        const std::vector<gr::Tag> rising = {rateTag(604UZ)};
        const auto                 middle = runChunks<float>(block, std::span<const float>(x).subspan(600UZ, 600UZ), 50UZ, std::span<const gr::Tag>(rising), 600UZ, 600UZ);
        expect(that % (ratesOf(middle) == std::vector<float>{kRateIn * 3.f / 2.f})) << "3/2 raises the rate downstream reads";

        std::ignore = block.settings().setStaged({{"interpolation", 1U}, {"decimation", 3U}});
        std::ignore = block.settings().applyStagedParameters();

        const std::vector<gr::Tag> falling = {rateTag(1215UZ)};
        const auto                 tail    = runChunks<float>(block, std::span<const float>(x).subspan(1200UZ), 50UZ, std::span<const gr::Tag>(falling), 1200UZ, 1500UZ);
        expect(that % (ratesOf(tail) == std::vector<float>{kRateIn / 3.f})) << "and 1/3 lowers it";
    };

    "a held sample_rate keeps the ratio that was in force when it crossed"_test = [] {
        const gr::property_map::key_type rateKey{gr::tag::SAMPLE_RATE.shortKey()};
        constexpr float                  kRateIn = 480000.f;

        RationalResampler<float> block = makeResampler<float>({{"interpolation", 1U}, {"decimation", 3U}, {"taps", std::vector<float>{1.0f}}});
        const std::vector<float> x(12UZ, 0.5f);

        // input 5 maps to output 2, one past the last output of the call that carries it, so the tag is held
        const std::vector<gr::Tag> tags{gr::Tag{5UZ, gr::property_map{{rateKey, gr::pmt::Value(kRateIn)}}}};
        const auto                 head = runChunks<float>(block, std::span<const float>(x).first(6UZ), 1UZ, std::span<const gr::Tag>(tags));
        expect(eq(head.tags.size(), 0UZ)) << "its output is not produced yet";

        std::ignore = block.settings().setStaged({{"interpolation", 3U}, {"decimation", 1U}});
        std::ignore = block.settings().applyStagedParameters();

        const auto tail = runChunks<float>(block, std::span<const float>(x).subspan(6UZ), 1UZ, {}, 6UZ, 2UZ);
        expect(eq(tail.tags.size(), 1UZ));
        if (tail.tags.size() == 1UZ) {
            const auto found = tail.tags[0UZ].map.find(rateKey);
            expect(that % (found != tail.tags[0UZ].map.end()));
            expect(eq(found->second.value_or(0.f), kRateIn / 3.f)) << "scaled where it arrived, not at the ratio that replaced it";
        }
    };

    "a rebuild that moves no tag keeps the one in flight"_test = [] {
        RationalResampler<float> block = makeResampler<float>({{"interpolation", 1U}, {"decimation", 3U}, {"taps", std::vector<float>{1.0f}}});
        const std::vector<float> x(12UZ, 0.5f);

        // input 5 maps to output 2, which is one past the last output of the call that carries it
        const std::vector<gr::Tag> tags{gr::Tag{5UZ, tagKey(0)}};
        const auto                 head = runChunks<float>(block, std::span<const float>(x).first(6UZ), 1UZ, std::span<const gr::Tag>(tags));
        expect(eq(head.samples.size(), 2UZ));
        expect(that % (head.offsetsOf("t0") == std::vector<std::size_t>{})) << "its output is not produced yet, so it is held";

        std::ignore = block.settings().setStaged({{"rolloff", 0.3f}});
        std::ignore = block.settings().applyStagedParameters();

        const auto tail = runChunks<float>(block, std::span<const float>(x).subspan(6UZ), 1UZ, {}, 6UZ, 2UZ);
        expect(eq(tail.samples.size(), 2UZ));
        expect(that % (tail.offsetsOf("t0") == std::vector<std::size_t>{2UZ})) << "a rebuild for a key that moves no tag keeps the held one, at the offset it already had";
    };

    "a tag that states a property of the stream crosses unmoved, and a trigger moves by the delay"_test = [] {
        // the rate describes output 0 as it describes input 0; the trigger leaves on output round(d / M), d being the
        // prototype's delay at the interpolated rate
        for (const auto& [l, m] : {std::pair<gr::Size_t, gr::Size_t>{3U, 2U}, std::pair<gr::Size_t, gr::Size_t>{1U, 4U}}) {
            RationalResampler<float> block = makeResampler<float>({{"interpolation", l}, {"decimation", m}});
            const std::size_t        twice = narrowIndex<std::size_t>(std::llround(2.0 * block.groupDelaySamples() * static_cast<double>(l)));
            gr::property_map         opening; // a source's opening tag: the stream's rate and the trigger of its first sample
            opening.insert_or_assign(gr::property_map::key_type{"sample_rate"}, 48000.0f);
            opening.insert_or_assign(gr::property_map::key_type{"trigger_time"}, std::uint64_t{1000});
            const std::vector<gr::Tag> tags{gr::Tag{0UZ, opening}};
            const std::vector<float>   x(400UZ * m, 0.0f);
            const auto                 got = runChunks<float>(block, std::span<const float>(x), 10UZ, std::span<const gr::Tag>(tags));

            expect(that % (got.offsetsOf("sample_rate") == std::vector<std::size_t>{0UZ})) << std::format("{}/{}: the rate stays on output 0", l, m);
            expect(that % (got.offsetsOf("trigger_time") == std::vector<std::size_t>{(twice + m) / (2UZ * m)})) << std::format("{}/{}: the trigger moves by the delay", l, m);
        }
    };

    "a tag leaves on the output that carries its sample's energy"_test = [] {
        // The designed prototype delays by d interpolated samples, and input i lands on output (i*L + d) / M. Each input
        // below puts that position on a whole output. Inputs 19 and 39 sit in the last chunk of the first call, and the
        // outputs they land on belong to a later call.
        struct Row {
            gr::Size_t  l, m;
            std::size_t at;
        };
        constexpr Row         kRows[] = {{3U, 2U, 19UZ}, {3U, 2U, 1UZ}, {1U, 4U, 39UZ}};
        constexpr std::size_t kChunks = 10UZ; // M-input chunks a call
        for (const auto& [l, m, at] : kRows) {
            RationalResampler<float> block = makeResampler<float>({{"interpolation", l}, {"decimation", m}});
            const std::size_t        d     = narrowIndex<std::size_t>(std::llround(block.groupDelaySamples() * static_cast<double>(l)));
            expect(eq((at * l + d) % m, 0UZ)) << "the fixture puts the delayed position on a whole output";
            const std::size_t want = (at * l + d) / m;

            std::vector<float> x(400UZ * m, 0.0f);
            x[at] = 1.0f;
            const std::vector<gr::Tag> tags{gr::Tag{at, tagKey(0)}};
            const auto                 got = runChunks<float>(block, std::span<const float>(x), kChunks, std::span<const gr::Tag>(tags));

            expect(that % (got.offsetsOf("t0") == std::vector<std::size_t>{want})) << std::format("{}/{}: input {} leaves on output {} and on no other", l, m, at, want);
            expect(eq(peakIndex(std::span<const float>(got.samples)), want)) << std::format("{}/{}: the impulse at input {} peaks on output {}", l, m, at, want);
        }
    };

    "an asymmetric supplied prototype moves its tags by the centroid of its energy"_test = [] {
        // a lone tap at 4 of 6 at 2/1 is a delay of four interpolated samples, not the 2.5 the length alone gives:
        // input i comes out on output 2i + 4
        RationalResampler<float> block = makeResampler<float>({{"interpolation", 2U}, {"decimation", 1U}, {"taps", std::vector<float>{0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f}}});
        expect(eq(block.groupDelaySamples(), 2.0)) << "four interpolated samples are two input samples";

        std::vector<float> x(40UZ, 0.0f);
        x[9] = 1.0f;
        const std::vector<gr::Tag> tags{gr::Tag{9UZ, tagKey(0)}};
        const auto                 got = runChunks<float>(block, std::span<const float>(x), 10UZ, std::span<const gr::Tag>(tags));
        expect(eq(peakIndex(std::span<const float>(got.samples)), 22UZ));
        expect(that % (got.offsetsOf("t0") == std::vector<std::size_t>{22UZ})) << "held out of the call that carried input 9, and published on output 22";
    };

    "a ratio change moves no held tag, and a later tag never lands ahead of it"_test = [] {
        // a lone tap at 20 delays by 20 at 1/1: input 95 is held for output 115
        std::vector<float> lone(21UZ, 0.0f);
        lone.back()                    = 1.0f;
        RationalResampler<float> block = makeResampler<float>({{"interpolation", 1U}, {"decimation", 1U}, {"taps", lone}});
        expect(eq(block.groupDelaySamples(), 20.0));

        const std::vector<float>   x(300UZ, 0.5f);
        const std::vector<gr::Tag> early{gr::Tag{95UZ, tagKey(0)}};
        const auto                 head = runChunks<float>(block, std::span<const float>(x).first(100UZ), 10UZ, std::span<const gr::Tag>(early));
        expect(that % head.offsetsOf("t0").empty()) << "output 115 is not produced by the first 100 inputs";

        // 3/2 with a unit tap and no delay, from the new origin input 100 / output 100: input 102 maps to output 103,
        // ahead of the held tag, and input 140 to output 160
        std::ignore = block.settings().setStaged({{"interpolation", 3U}, {"decimation", 2U}, {"taps", std::vector<float>{1.0f}}});
        std::ignore = block.settings().applyStagedParameters();
        expect(eq(block.groupDelaySamples(), 0.0));

        const std::vector<gr::Tag> late{gr::Tag{102UZ, tagKey(1)}, gr::Tag{140UZ, tagKey(2)}};
        const auto                 tail = runChunks<float>(block, std::span<const float>(x).subspan(100UZ, 100UZ), 10UZ, std::span<const gr::Tag>(late), 100UZ, 100UZ);
        expect(eq(tail.samples.size(), 150UZ));
        expect(that % (tail.offsetsOf("t0") == std::vector<std::size_t>{115UZ})) << "held across the change, at the output it crossed with";
        expect(that % (tail.offsetsOf("t1") == std::vector<std::size_t>{115UZ})) << "placed with the held tag, never ahead of it";
        expect(that % (tail.offsetsOf("t2") == std::vector<std::size_t>{160UZ})) << "and past it, by the new ratio";

        std::vector<std::size_t> order;
        for (const gr::Tag& published : tail.tags) {
            order.push_back(published.index);
        }
        expect(std::ranges::is_sorted(order)) << std::format("published in index order: [{}]", join(order));
        expect(!tail.tags.empty() && tail.tags.front().map.contains(gr::property_map::key_type{"t0"})) << "the tag that crossed first leaves first";
    };

    "degenerate settings"_test = [] {
        expect(throws([] { std::ignore = makeResampler<float>({{"interpolation", 0U}, {"decimation", 1U}}); }));
        expect(throws([] { std::ignore = makeResampler<float>({{"interpolation", 1U}, {"decimation", 0U}}); }));

        // fewer taps than branches is padded and works
        RationalResampler<float> block = makeResampler<float>({{"interpolation", 4U}, {"decimation", 1U}, {"taps", std::vector<float>{0.25f, 0.5f}}});
        const std::vector<float> x{1.0f};
        expect(that % (run(block, x) == std::vector<float>{0.25f, 0.5f, 0.0f, 0.0f}));
    };

    "the group delay is stated in input samples"_test = [] {
        RationalResampler<float> block = makeResampler<float>({{"interpolation", 1U}, {"decimation", 4U}});
        expect(approx(block.groupDelaySamples(), 73.0, 1e-9)) << "147 designed taps at L = 1";

        RationalResampler<float> interpolating = makeResampler<float>({{"interpolation", 3U}, {"decimation", 2U}});
        expect(approx(interpolating.groupDelaySamples(), 55.0 / 3.0, 1e-9)) << "111 taps at L = 3, and not an integer";
    };

    "tags land where the rate change puts them"_test = [] {
        constexpr std::size_t kTags = 8UZ;
        struct Case {
            gr::Size_t l, m;
        };
        constexpr Case kCases[] = {{3U, 2U}, {1U, 3U}, {147U, 160U}};
        for (const auto& [l, m] : kCases) {
            gr::Graph graph;

            auto& source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 4000U}, {"mark_tag", false}});
            for (std::size_t i = 0UZ; i < kTags; ++i) {
                source._tags.emplace_back(i, tagKey(i));
            }
            auto& resampler = graph.emplaceBlock<RationalResampler<float>>({{"interpolation", l}, {"decimation", m}, {"taps", std::vector<float>{1.0f}}});
            auto& sink      = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

            expect(graph.connect<"out", "in">(source, resampler).has_value());
            expect(graph.connect<"out", "in">(resampler, sink).has_value());

            gr::scheduler::Simple scheduler;
            expect(scheduler.exchange(std::move(graph)).has_value());
            expect(scheduler.runAndWait().has_value());

            // the offsets the map says, with the tags that share one arriving together rather than
            // being merged away: a decimating block that de-duplicates them discards data. The
            // source's own metadata tag rides along at offset zero and is not what is counted here.
            std::vector<std::size_t> want;
            for (std::size_t i = 0UZ; i < kTags; ++i) {
                const std::size_t at = narrowIndex<std::size_t>(mapResampledOffset(i, l, m));
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
            expect(that % (got == want)) << std::format("{}/{}: output offsets [{}] against [{}]", l, m, join(got), join(want));
            expect(eq(keysSeen, kTags)) << l << "/" << m << ": every tag survives, none merged away";
        }
    };

    "the L=3 M=2 offsets are the table"_test = [] {
        constexpr std::uint64_t kWant[] = {0ULL, 2ULL, 3ULL, 5ULL, 6ULL, 8ULL, 9ULL, 11ULL};
        for (std::uint64_t i = 0ULL; i < 8ULL; ++i) {
            expect(eq(mapResampledOffset(i, 3ULL, 2ULL), kWant[i]));
        }
    };
};

int main() { /* tests are automatically registered and run */ }
