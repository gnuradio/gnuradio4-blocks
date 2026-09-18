#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/basic/StreamToDataSet.hpp>
#include <gnuradio-4.0/digital/AccessCodeCorrelator.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::digital::AccessCodeCorrelator;
using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;

/// A widely deployed default access code, named here for interoperability and not as a default of this block.
constexpr std::uint64_t kInteropAccessCode = 0xACDDA4E2F28C20FCULL;

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

template<typename TBlock>
void restage(TBlock& block, gr::property_map settings) {
    std::ignore = block.settings().set(std::move(settings));
    std::ignore = block.settings().activateContext();
    std::ignore = block.settings().applyStagedParameters();
}

[[nodiscard]] std::string codeString(std::uint64_t word, std::size_t bits) {
    std::string text(bits, '0');
    for (std::size_t i = 0UZ; i < bits; ++i) {
        text[i] = ((word >> (bits - 1UZ - i)) & 1ULL) != 0ULL ? '1' : '0';
    }
    return text;
}

template<typename T>
[[nodiscard]] std::vector<T> itemsOf(std::string_view bits, T one = T{1}, T zero = T{0}) {
    std::vector<T> items(bits.size());
    for (std::size_t i = 0UZ; i < bits.size(); ++i) {
        items[i] = bits[i] == '1' ? one : zero;
    }
    return items;
}

/// The offsets the block tagged, in order.
template<typename T>
[[nodiscard]] std::vector<std::size_t> offsets(AccessCodeCorrelator<T>& block, std::span<const T> input, std::size_t chunkSize = 0UZ) {
    const auto               seen = gr::blocks::testing::span::run<T>(block, input, chunkSize);
    std::vector<std::size_t> where;
    for (const gr::Tag& tag : seen.tags) {
        where.push_back(tag.index);
    }
    return where;
}

[[nodiscard]] gr::Size_t syncErrorsOf(const gr::Tag& tag) {
    const auto outer = tag.map.find(gr::property_map::key_type(gr::tag::TRIGGER_META_INFO.shortKey()));
    if (outer == tag.map.end()) {
        return std::numeric_limits<gr::Size_t>::max();
    }
    const auto* nested = outer->second.get_if<gr::property_map>();
    if (nested == nullptr) {
        return std::numeric_limits<gr::Size_t>::max();
    }
    const auto inner = nested->find(gr::property_map::key_type("sync_errors"));
    return inner == nested->end() ? std::numeric_limits<gr::Size_t>::max() : inner->second.value_or(std::numeric_limits<gr::Size_t>::max());
}

struct Rng {
    std::uint64_t state = 0x243f6a8885a308d3ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
};

/// A zero stream with @p word planted at @p at, as unpacked bits.
[[nodiscard]] std::vector<std::uint8_t> plant(std::uint64_t word, std::size_t bits, std::size_t at, std::size_t total) {
    std::vector<std::uint8_t> stream(total, 0U);
    for (std::size_t i = 0UZ; i < bits; ++i) {
        stream[at + i] = static_cast<std::uint8_t>((word >> (bits - 1UZ - i)) & 1ULL);
    }
    return stream;
}

/// @brief A zero stream of @p total items with @p word's @p n bits planted @p spacing items apart from @p at.
[[nodiscard]] std::vector<std::uint8_t> plantStrided(std::uint64_t word, std::size_t n, std::size_t spacing, std::size_t at, std::size_t total) {
    std::vector<std::uint8_t> stream(total, std::uint8_t{0});
    for (std::size_t i = 0UZ; i < n; ++i) {
        stream[at + i * spacing] = static_cast<std::uint8_t>((word >> (n - 1UZ - i)) & 1ULL);
    }
    return stream;
}

/// @brief `sum_{i<=t} C(n,i) / 2^n`: the chance a uniformly random window of @p n items lies within @p t of any one
/// word. Computed here from the binomial recurrence rather than transcribed, so the expectation the run is judged
/// against is derived by the test and not by the block.
[[nodiscard]] long double falseAlarmProbability(std::size_t n, std::size_t t) {
    long double total = 0.0L;
    long double term  = 1.0L; // C(n, 0)
    for (std::size_t i = 0UZ; i <= t; ++i) {
        if (i > 0UZ) {
            term *= static_cast<long double>(n - i + 1UZ) / static_cast<long double>(i);
        }
        total += term;
    }
    return std::ldexp(total, -static_cast<int>(n));
}

/// The worst aperiodic autocorrelation sidelobe of a bipolar code, and its longest run of equal bits.
[[nodiscard]] std::pair<int, std::size_t> codeStatistics(std::uint64_t word, std::size_t bits) {
    std::vector<int> bipolar(bits);
    for (std::size_t i = 0UZ; i < bits; ++i) {
        bipolar[i] = ((word >> (bits - 1UZ - i)) & 1ULL) != 0ULL ? 1 : -1;
    }
    int worst = 0;
    for (std::size_t shift = 1UZ; shift < bits; ++shift) {
        int sum = 0;
        for (std::size_t i = 0UZ; i + shift < bits; ++i) {
            sum += bipolar[i] * bipolar[i + shift];
        }
        worst = std::max(worst, std::abs(sum));
    }
    std::size_t longest = 1UZ;
    std::size_t run     = 1UZ;
    for (std::size_t i = 1UZ; i < bits; ++i) {
        run     = bipolar[i] == bipolar[i - 1UZ] ? run + 1UZ : 1UZ;
        longest = std::max(longest, run);
    }
    return {worst, longest};
}

/// @brief Run the block in a graph over @p stream with @p incoming planted at the source, and report what the sink saw.
[[nodiscard]] std::vector<gr::Tag> throughGraph(gr::property_map settings, std::span<const std::uint8_t> stream, std::span<const gr::Tag> incoming) {
    using gr::testing::ProcessFunction;
    using gr::testing::TagSink;
    using gr::testing::TagSource;

    gr::Graph  graph;
    const auto values = gr::Tensor<std::uint8_t>(stream.begin(), stream.end());
    auto&      source = graph.emplaceBlock<TagSource<std::uint8_t, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(stream.size())}, {"values", values}});
    source._tags.assign(incoming.begin(), incoming.end());
    std::ranges::sort(source._tags, std::ranges::less{}, &gr::Tag::index);

    auto& block = graph.emplaceBlock<AccessCodeCorrelator<std::uint8_t>>(std::move(settings));
    auto& sink  = graph.emplaceBlock<TagSink<std::uint8_t, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

    boost::ut::expect(graph.connect<"out", "in">(source, block).has_value());
    boost::ut::expect(graph.connect<"out", "in">(block, sink).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value());
    return sink._tags;
}

} // namespace

const boost::ut::suite<"access code correlator"> accessCodeTests = [] {
    using namespace boost::ut;

    "the tag sits on the first item after the code"_test = [] {
        // the worked case: input 1,0,1,1,1,1,0,1,1 against "1011" tags at 4 and 9
        AccessCodeCorrelator<std::uint8_t> block  = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string("1011")}});
        const std::vector<std::uint8_t>    stream = itemsOf<std::uint8_t>("1011110110");
        expect(that % (offsets<std::uint8_t>(block, std::span<const std::uint8_t>(stream)) == std::vector<std::size_t>{4UZ, 9UZ}));
    };

    "overlapping detections both fire"_test = [] {
        // "1011" planted at 0 and at 3, which share their boundary item: tags 3 apart
        AccessCodeCorrelator<std::uint8_t> block  = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string("1011")}});
        const std::vector<std::uint8_t>    stream = itemsOf<std::uint8_t>("10110110");
        expect(that % (offsets<std::uint8_t>(block, std::span<const std::uint8_t>(stream)) == std::vector<std::size_t>{4UZ, 7UZ}));
    };

    "the stream itself passes through unchanged"_test = [] {
        AccessCodeCorrelator<std::uint8_t> block  = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string("1011")}});
        const std::vector<std::uint8_t>    stream = plant(kInteropAccessCode, 64UZ, 100UZ, 300UZ);
        const auto                         seen   = gr::blocks::testing::span::run<std::uint8_t>(block, std::span<const std::uint8_t>(stream), 7UZ);
        expect(that % (seen.samples == stream)) << "1:1 passthrough, out[i] == in[i]";
        expect(eq(seen.consumed, stream.size()));
    };

    "max_errors is an inclusive bound on errors, not a count of matching bits"_test = [] {
        Rng                 rng{};
        const std::uint64_t code = rng.next() & 0xFFFFFFFFULL;
        expect(gt(static_cast<std::size_t>(std::popcount(code)), 8UZ)) << "the planted code must be far from the zero stream around it";

        for (gr::Size_t bound = 0U; bound <= 4U; ++bound) {
            AccessCodeCorrelator<std::uint8_t> block = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", codeString(code, 32UZ)}, {"max_errors", bound}});

            for (const gr::Size_t flips : {bound, bound + 1U}) {
                std::uint64_t corrupted = code;
                for (gr::Size_t k = 0U; k < flips; ++k) {
                    corrupted ^= 1ULL << (2U * k); // distinct bits, so the distance is exactly `flips`
                }
                const std::vector<std::uint8_t>    stream = plant(corrupted, 32UZ, 40UZ, 140UZ);
                AccessCodeCorrelator<std::uint8_t> fresh  = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", codeString(code, 32UZ)}, {"max_errors", bound}});
                const auto                         seen   = gr::blocks::testing::span::run<std::uint8_t>(fresh, std::span<const std::uint8_t>(stream));

                if (flips == bound) {
                    expect(eq(seen.tags.size(), 1UZ)) << std::format("t={} bound={}: {} errors is within the bound", flips, bound, flips);
                    if (!seen.tags.empty()) {
                        expect(eq(seen.tags[0].index, 72UZ)) << "the first item after the code";
                        expect(eq(syncErrorsOf(seen.tags[0]), bound)) << "the Hamming distance travels in trigger_meta_info";
                    }
                } else {
                    expect(eq(seen.tags.size(), 0UZ)) << std::format("bound={}: {} errors is one too many", bound, flips);
                }
            }
            std::ignore = block;
        }
    };

    "degenerate parameters throw"_test = [] {
        expect(throws([] {
            AccessCodeCorrelator<std::uint8_t> block{}; // no settings at all, so nothing is staged and nothing is validated
            block.start();
        })) << "there is no default access code: a block without one refuses to start";
        { // and until it starts it is inert: it detects nothing and consumes nothing
            AccessCodeCorrelator<std::uint8_t>                  block{};
            const std::vector<std::uint8_t>                     zeros(32UZ, 0U);
            std::vector<std::uint8_t>                           out(32UZ);
            std::vector<gr::Tag>                                published;
            gr::blocks::testing::span::InputSpan<std::uint8_t>  inSpan{std::span<const std::uint8_t>(zeros)};
            gr::blocks::testing::span::OutputSpan<std::uint8_t> outSpan{std::span<std::uint8_t>(out), 0UZ, &published};
            expect(that % (block.processBulk(inSpan, outSpan) == gr::work::Status::ERROR));
            expect(eq(inSpan.consumed, 0UZ));
            expect(eq(published.size(), 0UZ));
        }
        expect(throws([] {
            auto block = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string("")}}); // the empty code is the default, so the batch moves nothing
            block.start();
        })) << "and stating the empty code explicitly is the same block as one without it";
        expect(throws([] { std::ignore = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string(129UZ, '1')}}); })) << "129 bits do not fit the two limbs";
        expect(throws([] { std::ignore = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string("1011")}, {"max_errors", 5U}}); })) << "a bound above the length matches everything";
        expect(throws([] { std::ignore = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string("10 11")}}); })) << "a typo must not silently become a different code";
        expect(throws([] { std::ignore = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string("1012")}}); }));
        expect(nothrow([] { std::ignore = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string(64UZ, '1')}, {"max_errors", 64U}}); })) << "64 bits and a bound of 64 are both legal";
        expect(nothrow([] { std::ignore = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string(128UZ, '1')}, {"max_errors", 128U}}); })) << "and so are both limbs full, with a bound to match";
    };

    "nothing is compared before the register holds real bits"_test = [] {
        const std::vector<std::uint8_t> zeros(160UZ, 0U);
        for (const std::size_t length : {1UZ, 4UZ, 17UZ, 64UZ, 65UZ, 128UZ}) {
            AccessCodeCorrelator<std::uint8_t> block = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string(length, '0')}});
            const std::vector<std::size_t>     where = offsets<std::uint8_t>(block, std::span<const std::uint8_t>(zeros));
            expect(!where.empty()) << std::format("a {}-bit code of zeros matches a zero stream", length);
            if (where.empty()) {
                continue;
            }
            expect(eq(where.front(), length)) << std::format("a {}-bit code cannot match before item {}", length, length);
        }
    };

    "a longer code mid-stream re-arms the warm-up"_test = [] {
        const std::vector<std::uint8_t>    zeros(64UZ, 0U);
        AccessCodeCorrelator<std::uint8_t> block  = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string(4UZ, '0')}});
        const std::vector<std::size_t>     before = offsets<std::uint8_t>(block, std::span<const std::uint8_t>(zeros));
        expect(!before.empty());
        expect(eq(before.empty() ? 0UZ : before.front(), 4UZ));

        restage(block, {{"access_code", std::string(16UZ, '0')}});
        const std::vector<std::size_t> where = offsets<std::uint8_t>(block, std::span<const std::uint8_t>(zeros));
        expect(!where.empty());
        expect(eq(where.empty() ? 0UZ : where.front(), 16UZ)) << "the counter is reset by the code change, so no stale bit is compared";
    };

    "a soft bit is sliced at zero and a NaN slices to zero"_test = [] {
        const std::string code(8UZ, '1');

        AccessCodeCorrelator<float> positiveZero = make<AccessCodeCorrelator<float>>({{"access_code", code}});
        AccessCodeCorrelator<float> negativeZero = make<AccessCodeCorrelator<float>>({{"access_code", code}});
        AccessCodeCorrelator<float> ones         = make<AccessCodeCorrelator<float>>({{"access_code", code}});
        const std::vector<float>    plusZero(40UZ, +0.0f);
        const std::vector<float>    minusZero(40UZ, -0.0f);
        const std::vector<float>    plusOne(40UZ, 1.0f);

        const std::vector<std::size_t> fromPlusZero  = offsets<float>(positiveZero, std::span<const float>(plusZero));
        const std::vector<std::size_t> fromMinusZero = offsets<float>(negativeZero, std::span<const float>(minusZero));
        const std::vector<std::size_t> fromOnes      = offsets<float>(ones, std::span<const float>(plusOne));
        expect(that % (fromPlusZero == fromOnes)) << "+0.0 slices to one, because x >= 0 means one";
        expect(that % (fromMinusZero == fromOnes)) << "-0.0 slices to one too: -0.0 >= 0 is true";

        const std::string           zeroCode(8UZ, '0');
        AccessCodeCorrelator<float> fromNaN      = make<AccessCodeCorrelator<float>>({{"access_code", zeroCode}});
        AccessCodeCorrelator<float> fromNegative = make<AccessCodeCorrelator<float>>({{"access_code", zeroCode}});
        const std::vector<float>    nans(40UZ, std::numeric_limits<float>::quiet_NaN());
        const std::vector<float>    negatives(40UZ, -1.0f);
        expect(that % (offsets<float>(fromNaN, std::span<const float>(nans)) == offsets<float>(fromNegative, std::span<const float>(negatives)))) << "a NaN slices to zero: every IEEE comparison with it is false";
    };

    "the tags do not move with the chunk size"_test = [] {
        const std::vector<std::uint8_t>    stream = plant(kInteropAccessCode, 64UZ, 137UZ, 600UZ);
        AccessCodeCorrelator<std::uint8_t> whole  = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", codeString(kInteropAccessCode, 64UZ)}, {"max_errors", 4U}});
        const std::vector<std::size_t>     want   = offsets<std::uint8_t>(whole, std::span<const std::uint8_t>(stream));
        expect(that % (want == std::vector<std::size_t>{201UZ})) << "one detection, at the first item after the 64-bit code";

        for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 4096UZ}) {
            AccessCodeCorrelator<std::uint8_t> block = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", codeString(kInteropAccessCode, 64UZ)}, {"max_errors", 4U}});
            expect(that % (offsets<std::uint8_t>(block, std::span<const std::uint8_t>(stream), chunk) == want)) << std::format("chunk {}", chunk);
        }
    };

    "the interoperability access code is balanced and its autocorrelation sidelobe is low"_test = [] {
        expect(eq(static_cast<std::size_t>(std::popcount(kInteropAccessCode)), 32UZ)) << "32 ones and 32 zeros";
        const auto [sidelobe, longestRun] = codeStatistics(kInteropAccessCode, 64UZ);
        expect(eq(longestRun, 6UZ));
        expect(eq(sidelobe, 11)) << "worst aperiodic autocorrelation sidelobe, against a peak of 64";
    };

    "a detector and StreamToDataSet are a complete fixed-length framing chain"_test = [] {
        constexpr std::size_t kPayload = 800UZ;
        constexpr std::size_t kFirst   = 64UZ;
        constexpr std::size_t kSpacing = 1200UZ;

        std::vector<std::uint8_t> stream(4000UZ, 0U);
        std::vector<std::size_t>  planted;
        for (std::size_t at = kFirst; at + 64UZ + kPayload < stream.size(); at += kSpacing) {
            for (std::size_t i = 0UZ; i < 64UZ; ++i) {
                stream[at + i] = static_cast<std::uint8_t>((kInteropAccessCode >> (63UZ - i)) & 1ULL);
            }
            planted.push_back(at + 64UZ);
        }
        expect(ge(planted.size(), 3UZ));

        AccessCodeCorrelator<std::uint8_t> detector = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", codeString(kInteropAccessCode, 64UZ)}});
        const auto                         tagged   = gr::blocks::testing::span::run<std::uint8_t>(detector, std::span<const std::uint8_t>(stream), 256UZ);
        expect(eq(tagged.tags.size(), planted.size()));

        // the existing extractor, unchanged, with the filter naming this block's trigger_name
        auto extractor = make<gr::blocks::basic::StreamFilterImpl<std::uint8_t, false>>({{"filter", std::string("[access_code]")}, {"n_pre", 0U}, {"n_post", static_cast<gr::Size_t>(kPayload)}});

        std::vector<gr::DataSet<std::uint8_t>> packets;
        std::vector<gr::DataSet<std::uint8_t>> scratch(4UZ);
        std::size_t                            consumed = 0UZ;
        while (consumed < tagged.samples.size()) {
            // The scheduler breaks an input span at every tag, so StreamToDataSet only ever sees one at relative
            // index 0, and a driver that hands it a tag mid-span is testing nothing.
            std::size_t count = std::min(64UZ, tagged.samples.size() - consumed);
            if (const auto next = std::ranges::upper_bound(tagged.tags, consumed, std::ranges::less{}, &gr::Tag::index); next != tagged.tags.end()) {
                count = std::min(count, next->index - consumed);
            }
            const auto first = std::ranges::lower_bound(tagged.tags, consumed, std::ranges::less{}, &gr::Tag::index);
            const auto last  = std::ranges::lower_bound(tagged.tags, consumed + count, std::ranges::less{}, &gr::Tag::index);

            InputSpan<std::uint8_t>               inSpan(std::span<const std::uint8_t>(tagged.samples).subspan(consumed, count), consumed, std::span<const gr::Tag>(first, last));
            OutputSpan<gr::DataSet<std::uint8_t>> outSpan{std::span<gr::DataSet<std::uint8_t>>(scratch)};
            std::ignore = extractor.processBulk(inSpan, outSpan);
            for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
                packets.push_back(std::move(scratch[k]));
            }
            if (inSpan.consumed == 0UZ) {
                break;
            }
            consumed += inSpan.consumed;
        }

        expect(eq(packets.size(), planted.size())) << "one DataSet per planted code — no new extractor block is needed";
        for (std::size_t which = 0UZ; which < packets.size(); ++which) {
            expect(eq(packets[which].signal_values.size(), kPayload)) << std::format("packet {} is n_post items long", which);
            expect(that % (packets[which].signal_values == std::vector<std::uint8_t>(std::next(stream.begin(), static_cast<std::ptrdiff_t>(planted[which])), std::next(stream.begin(), static_cast<std::ptrdiff_t>(planted[which] + kPayload))))) << std::format("packet {} is the payload that followed the code", which);
        }
    };

    "a non-reserved tag key rides through at its own offset"_test = [] {
        const gr::property_map::key_type key{"private_key"};
        const gr::pmt::Value             value{std::string("carried")};

        const std::vector<std::uint8_t> stream = plant(kInteropAccessCode, 64UZ, 100UZ, 600UZ);
        std::vector<gr::Tag>            incoming;
        for (const std::size_t at : {11UZ, 400UZ}) {
            incoming.emplace_back(at, gr::property_map{{key, value}});
        }

        const std::vector<gr::Tag> seen = throughGraph({{"access_code", codeString(kInteropAccessCode, 64UZ)}}, std::span<const std::uint8_t>(stream), std::span<const gr::Tag>(incoming));

        std::vector<std::size_t> where;
        for (const gr::Tag& seenTag : seen) {
            if (const auto found = seenTag.map.find(key); found != seenTag.map.end() && found->second == value) {
                where.push_back(seenTag.index);
            }
        }
        expect(that % (where == std::vector<std::size_t>{11UZ, 400UZ})) << "the pass-all policy keeps a key the auto-forward set does not name, beside this block's own detections";
    };

    "an upstream trigger_name passes unrewritten and the block's own detections carry the configured label"_test = [] {
        const gr::property_map::key_type triggerName{gr::tag::TRIGGER_NAME.shortKey()};
        const gr::pmt::Value             upstream{std::string("preamble")};

        // one planted code, so exactly one detection, and an upstream trigger tag well clear of it
        const std::vector<std::uint8_t> stream = plant(kInteropAccessCode, 64UZ, 100UZ, 600UZ);
        const std::vector<gr::Tag>      incoming{gr::Tag{11UZ, gr::property_map{{triggerName, upstream}}}};

        const std::vector<gr::Tag> seen = throughGraph({{"access_code", codeString(kInteropAccessCode, 64UZ)}, {"trigger_label", std::string("access_code")}}, std::span<const std::uint8_t>(stream), std::span<const gr::Tag>(incoming));

        std::vector<std::pair<std::size_t, std::string>> labels;
        for (const gr::Tag& seenTag : seen) {
            if (const auto found = seenTag.map.find(triggerName); found != seenTag.map.end()) {
                labels.emplace_back(seenTag.index, found->second.value_or(std::string{}));
            }
        }
        const std::vector<std::pair<std::size_t, std::string>> want{{11UZ, std::string("preamble")}, {164UZ, std::string("access_code")}};
        expect(that % (labels == want)) << "the setting names the emitted label, so a passing trigger_name is not rewritten to it";
    };

    "dibit items match symbol-wise: a symbol wrong in both bits costs one"_test = [] {
        // P25's C4FM frame sync, 48 bits as 24 dibit symbols.
        const std::string sync = codeString(0x5575F5FF77FFULL, 48UZ);

        const auto dibitsOf = [](std::string_view bits) {
            std::vector<std::uint8_t> items(bits.size() / 2UZ);
            for (std::size_t i = 0UZ; i < items.size(); ++i) {
                items[i] = static_cast<std::uint8_t>(((bits[2UZ * i] == '1' ? 2U : 0U) | (bits[2UZ * i + 1UZ] == '1' ? 1U : 0U)));
            }
            return items;
        };

        std::vector<std::uint8_t> stream(40UZ, 0U);
        const auto                code = dibitsOf(sync);
        stream.insert(stream.begin() + 20, code.begin(), code.end());

        auto exact = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", sync}, {"bits_per_item", gr::Size_t(2)}});
        exact.start();
        expect(that % (offsets<std::uint8_t>(exact, std::span<const std::uint8_t>(stream)) == std::vector{44UZ})) << "the tag sits at the first payload item";

        // One symbol inverted in BOTH bits: two bit errors, one symbol error.
        std::vector<std::uint8_t> damaged = stream;
        damaged[20UZ + 7UZ] ^= 0x3U;
        auto strict = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", sync}, {"bits_per_item", gr::Size_t(2)}});
        strict.start();
        expect(that % offsets<std::uint8_t>(strict, std::span<const std::uint8_t>(damaged)).empty()) << "exact match refuses one wrong symbol";
        auto lenient = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", sync}, {"bits_per_item", gr::Size_t(2)}, {"max_errors", gr::Size_t(1)}});
        lenient.start();
        expect(that % (offsets<std::uint8_t>(lenient, std::span<const std::uint8_t>(damaged)) == std::vector{44UZ})) << "one symbol of slack accepts it, both bits notwithstanding";
    };

    "a symbol astride the two limbs is still one symbol"_test = [] {
        // 66 bits at three per item put the register's top symbol at bits 63..65, across the limb boundary, and the
        // top symbol is the FIRST transmitted item. Its middle bit is bit 64 — the high limb's lowest — so a flip
        // there must collapse across the boundary and cost one symbol, as must all three bits together.
        std::string code;
        for (std::size_t i = 0UZ; i < 22UZ; ++i) {
            code += (i % 2UZ == 0UZ) ? "101" : "010"; // alternating symbols, so a shifted window misses by many
        }
        std::vector<std::uint8_t> stream(60UZ, 0U);
        for (std::size_t i = 0UZ; i < 22UZ; ++i) {
            stream[20UZ + i] = (i % 2UZ == 0UZ) ? std::uint8_t{5U} : std::uint8_t{2U};
        }
        auto exact = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", code}, {"bits_per_item", gr::Size_t(3)}});
        exact.start();
        expect(that % (offsets<std::uint8_t>(exact, std::span<const std::uint8_t>(stream)) == std::vector{42UZ})) << "the whole 66-bit word matches exactly";

        for (const std::uint8_t damagedItem : {std::uint8_t{5U ^ 2U}, std::uint8_t{5U ^ 7U}}) {
            std::vector<std::uint8_t> damaged = stream;
            damaged[20UZ]                     = damagedItem;
            auto strict                       = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", code}, {"bits_per_item", gr::Size_t(3)}});
            strict.start();
            expect(that % offsets<std::uint8_t>(strict, std::span<const std::uint8_t>(damaged)).empty()) << "exact match refuses the wrong symbol";
            auto lenient = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", code}, {"bits_per_item", gr::Size_t(3)}, {"max_errors", gr::Size_t(1)}});
            lenient.start();
            const auto seen = gr::blocks::testing::span::run<std::uint8_t>(lenient, std::span<const std::uint8_t>(damaged));
            expect(eq(seen.tags.size(), 1UZ)) << std::format("item {:#x}: one symbol of slack accepts it", damagedItem);
            if (!seen.tags.empty()) {
                expect(eq(syncErrorsOf(seen.tags.front()), gr::Size_t{1})) << "however many of its bits are wrong, the straddling symbol costs one";
            }
        }
    };

    "one bit per item still slices any non-zero byte to one"_test = [] {
        // 0x02 has a zero low bit; the symbol path would read it as 0, the slice reads 1.
        std::vector<std::uint8_t> stream(24UZ, 0U);
        for (std::size_t i = 0UZ; i < 4UZ; ++i) {
            stream[12UZ + i] = 0x02U;
        }
        auto block = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string("1111")}});
        block.start();
        expect(that % (offsets<std::uint8_t>(block, std::span<const std::uint8_t>(stream)) == std::vector{16UZ})) << "the slice semantics are unchanged at one bit per item";
    };

    "the symbol width refusals fire by name"_test = [] {
        expect(throws([] { std::ignore = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string("1010")}, {"bits_per_item", gr::Size_t(0)}}); })) << "zero bits per item";
        expect(throws([] { std::ignore = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string(18UZ, '1')}, {"bits_per_item", gr::Size_t(9)}}); })) << "nine bits per item";
        expect(throws([] { std::ignore = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string("10101")}, {"bits_per_item", gr::Size_t(2)}}); })) << "five bits are not whole dibits";
        expect(throws([] { std::ignore = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string("1010")}, {"bits_per_item", gr::Size_t(2)}, {"max_errors", gr::Size_t(3)}}); })) << "a bound past the symbol count matches everything";
        expect(throws([] { std::ignore = make<AccessCodeCorrelator<float>>({{"access_code", std::string("1010")}, {"bits_per_item", gr::Size_t(2)}}); })) << "a soft stream carries one bit per item";
    };
};

const boost::ut::suite<"distributed sync words"> distributedSyncTests = [] {
    using namespace boost::ut;

    // A code of `n` items at stride `s` is one row of an `s` by `n` block interleaver, which is what makes the two
    // AO-40 geometries what they are. Every number below follows from `n` and `s` and none is transcribed.
    struct Geometry {
        const char* what;
        std::size_t items;
        std::size_t spacing;
        std::size_t frame;
        std::size_t lag;
    };
    static constexpr Geometry kAo40Long{"AO-40 long", 65UZ, 80UZ, 5200UZ, 5121UZ};
    static constexpr Geometry kAo40Short{"AO-40 short", 52UZ, 51UZ, 2652UZ, 2602UZ};

    "the AO-40 geometries are arithmetic, and both words fit the two-limb register"_test = [] {
        for (const Geometry& form : {kAo40Long, kAo40Short}) {
            expect(eq(form.items * form.spacing, form.frame)) << std::format("{}: the frame is n*s items", form.what);
            expect(eq((form.items - 1UZ) * form.spacing + 1UZ, form.lag)) << std::format("{}: the lag is (n-1)*s + 1", form.what);

            // the sync word is one interleaver row: transmitted position i*s decomposes as c*rows + r with rows = s,
            // so r = 0 and c = i, and its deinterleaved index r*cols + c is i. The sync word is the frame's first n
            // items in order, which is why a window at offset n contains none of it.
            const std::size_t rows = form.spacing;
            const std::size_t cols = form.items;
            expect(eq(rows * cols, form.frame)) << std::format("{}: the rectangle is rows = s, cols = n", form.what);
            for (std::size_t i = 0UZ; i < form.items; ++i) {
                const std::size_t transmitted  = i * form.spacing;
                const std::size_t deinterleave = (transmitted % rows) * cols + transmitted / rows;
                expect(eq(deinterleave, i)) << std::format("{}: sync item {} deinterleaves to index {}", form.what, i, i);
            }
        }

        // both forms run whole: the long one exercises the register's second limb. The words are synthetic, so
        // only the geometry is under test and not any published sync word.
        for (const Geometry& form : {kAo40Long, kAo40Short}) {
            Rng         rng;
            std::string word(form.items, '0');
            for (char& bit : word) {
                bit = (rng.next() & 1ULL) != 0ULL ? '1' : '0';
            }
            const std::size_t         at = 11UZ;
            std::vector<std::uint8_t> stream(at + form.lag + 32UZ, std::uint8_t{0});
            for (std::size_t i = 0UZ; i < form.items; ++i) {
                stream[at + i * form.spacing] = word[i] == '1' ? std::uint8_t{1} : std::uint8_t{0};
            }
            AccessCodeCorrelator<std::uint8_t> block = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", word}, {"stride", static_cast<gr::Size_t>(form.spacing)}});
            expect(that % (offsets<std::uint8_t>(block, std::span<const std::uint8_t>(stream)) == std::vector<std::size_t>{at + form.lag})) //
                << std::format("{}: one tag, at the item after the code's last", form.what);
        }
        expect(nothrow([] { std::ignore = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string(128UZ, '1')}, {"stride", gr::Size_t{80}}}); })) << "128 items are the limbs' width";
        const std::string message = [] {
            try {
                std::ignore = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string(129UZ, '1')}, {"stride", gr::Size_t{80}}});
            } catch (const std::exception& error) {
                return std::string(error.what());
            }
            return std::string{};
        }();
        expect(message.contains("129")) << message;
    };

    "a strided word is found at the item the arithmetic names"_test = [] {
        struct Case {
            std::size_t   items;
            std::size_t   spacing;
            std::uint64_t word;
        };
        const Case kCases[]{{4UZ, 3UZ, 0b1011ULL}, {8UZ, 5UZ, 0b10110100ULL}, {kAo40Short.items, kAo40Short.spacing, 0x000BADC0FFEE1234ULL}};

        for (const Case& row : kCases) {
            const std::size_t lag   = (row.items - 1UZ) * row.spacing + 1UZ;
            const std::size_t at    = 7UZ;
            const std::size_t total = at + lag + 40UZ;

            const std::vector<std::uint8_t> stream = plantStrided(row.word, row.items, row.spacing, at, total);
            const gr::property_map          base{{"access_code", codeString(row.word, row.items)}, {"stride", static_cast<gr::Size_t>(row.spacing)}};

            AccessCodeCorrelator<std::uint8_t> block = make<AccessCodeCorrelator<std::uint8_t>>(base);
            expect(that % (offsets<std::uint8_t>(block, std::span<const std::uint8_t>(stream)) == std::vector<std::size_t>{at + lag})) //
                << std::format("n = {}, s = {}: one tag, at the item after the code's last", row.items, row.spacing);
            expect(eq(block.nDetections, 1ULL));

            // the same stream through the delayed output: the tag index does not move, the stream under it does
            AccessCodeCorrelator<std::uint8_t> delayed = make<AccessCodeCorrelator<std::uint8_t>>([&base] {
                gr::property_map settings = base;
                settings.insert_or_assign("tag_at", std::string("code_start"));
                return settings;
            }());
            const auto                         seen    = gr::blocks::testing::span::run<std::uint8_t>(delayed, std::span<const std::uint8_t>(stream));
            expect(eq(seen.tags.size(), 1UZ)) << std::format("n = {}, s = {}", row.items, row.spacing);
            if (!seen.tags.empty()) {
                expect(eq(seen.tags.front().index, at + lag)) << "the tag names the code's first item, which the lag puts at the same index";
                expect(eq(seen.samples[at + lag], stream[at])) << "and the item under the tag is the code's first";
            }
            for (std::size_t j = lag; j < total; ++j) {
                expect(that % (seen.samples[j] == stream[j - lag])) << std::format("n = {}, s = {}: out[{}] == in[{}]", row.items, row.spacing, j, j - lag);
            }
            for (std::size_t j = 0UZ; j < lag; ++j) {
                expect(that % (seen.samples[j] == std::uint8_t{0})) << "the first D output items are the ring's zeros";
            }
        }
    };

    "a strided word survives up to max_errors wrong items and no more"_test = [] {
        constexpr std::size_t   kItems     = 32UZ;
        constexpr std::size_t   kSpacing   = 17UZ;
        constexpr std::uint64_t kWord      = 0xB4D1CE73ULL;
        constexpr std::size_t   kTolerated = 5UZ;
        const std::size_t       lag        = (kItems - 1UZ) * kSpacing + 1UZ;

        const gr::property_map settings{{"access_code", codeString(kWord, kItems)}, {"stride", static_cast<gr::Size_t>(kSpacing)}, {"max_errors", static_cast<gr::Size_t>(kTolerated)}};

        for (const std::size_t wrong : {0UZ, 1UZ, 3UZ, kTolerated, kTolerated + 1UZ}) {
            std::vector<std::uint8_t> stream = plantStrided(kWord, kItems, kSpacing, 3UZ, 3UZ + lag + 8UZ);
            for (std::size_t k = 0UZ; k < wrong; ++k) {
                const std::size_t at = 3UZ + (2UZ + 5UZ * k) * kSpacing; // distinct code items, spread through the word
                stream[at] ^= std::uint8_t{1};
            }

            AccessCodeCorrelator<std::uint8_t> block = make<AccessCodeCorrelator<std::uint8_t>>(settings);
            const auto                         seen  = gr::blocks::testing::span::run<std::uint8_t>(block, std::span<const std::uint8_t>(stream));
            if (wrong <= kTolerated) {
                expect(eq(seen.tags.size(), 1UZ)) << std::format("{} wrong items is within the bound", wrong);
                if (!seen.tags.empty()) {
                    expect(eq(syncErrorsOf(seen.tags.front()), static_cast<gr::Size_t>(wrong))) << "and the distance travels in sync_errors";
                }
            } else {
                expect(eq(seen.tags.size(), 0UZ)) << std::format("{} wrong items is one too many", wrong);
            }
        }
    };

    "no comparison exists before item (n-1)*s + 1"_test = [] {
        // max_errors at the word's full length makes every comparison match, so the first tag is the warm-up boundary
        // itself and nothing before it can be excused as a near miss
        constexpr std::size_t kItems = 8UZ;
        for (const std::size_t spacing : {1UZ, 3UZ, 5UZ, 40UZ}) {
            const std::size_t         lag = (kItems - 1UZ) * spacing + 1UZ;
            Rng                       rng{};
            std::vector<std::uint8_t> stream(lag + 30UZ);
            for (std::uint8_t& item : stream) {
                item = static_cast<std::uint8_t>(rng.next() & 1ULL);
            }

            AccessCodeCorrelator<std::uint8_t> block = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", codeString(0xA5ULL, kItems)}, {"stride", static_cast<gr::Size_t>(spacing)}, {"max_errors", static_cast<gr::Size_t>(kItems)}});
            const std::vector<std::size_t>     where = offsets<std::uint8_t>(block, std::span<const std::uint8_t>(stream));
            expect(!where.empty()) << std::format("s = {}: a bound of n symbols matches everything, so every warm position fires", spacing);
            if (!where.empty()) {
                expect(eq(where.front(), lag)) << std::format("s = {}: the first comparison is at item {}", spacing, lag);
                expect(eq(where.size(), stream.size() - lag)) << std::format("s = {}: and every item from there on", spacing);
            }
        }

        // a mid-stream change to the code or the stride starts the warm-up again, which is what says the counter is
        // the generalization's and not a leftover
        AccessCodeCorrelator<std::uint8_t> block = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", codeString(0xA5ULL, kItems)}, {"stride", gr::Size_t{5}}, {"max_errors", static_cast<gr::Size_t>(kItems)}});
        std::vector<std::uint8_t>          stream(80UZ, std::uint8_t{1});
        expect(eq(offsets<std::uint8_t>(block, std::span<const std::uint8_t>(stream)).front(), 36UZ)) << "7 * 5 + 1";
        restage(block, {{"stride", gr::Size_t{3}}});
        expect(eq(offsets<std::uint8_t>(block, std::span<const std::uint8_t>(stream)).front(), 22UZ)) << "7 * 3 + 1, counted from the change and not from the stream's start";
        restage(block, {{"access_code", codeString(0x5AULL, kItems)}});
        expect(eq(offsets<std::uint8_t>(block, std::span<const std::uint8_t>(stream)).front(), 22UZ)) << "and a new code warms up too";
    };

    "the false-alarm count is the binomial tail, and stride does not move it"_test = [] {
        constexpr std::size_t kItems     = 52UZ;
        constexpr std::size_t kTolerated = 14UZ;
        constexpr std::size_t kLength    = 1000000UZ;

        const long double perPosition = falseAlarmProbability(kItems, kTolerated);
        expect(that % (perPosition > 5.9e-4L && perPosition < 6.1e-4L)) << std::format("the tail at n = 52, t = 14 is 5.976e-04, computed as {:.4e}", static_cast<double>(perPosition));

        Rng                       rng{};
        std::vector<std::uint8_t> stream(kLength);
        for (std::uint8_t& item : stream) {
            item = static_cast<std::uint8_t>(rng.next() & 1ULL);
        }

        struct Arm {
            std::size_t spacing;
            double      sigmas;
            const char* why;
        };
        // consecutive strided trials read disjoint item sets at s >= n, so the count is close to a true binomial;
        // consecutive contiguous trials share n-1 items, so the same mean carries a wider spread
        const Arm kArms[]{{kAo40Short.spacing, 3.0, "disjoint trials"}, {1UZ, 5.0, "overlapping trials"}};

        for (const Arm& arm : kArms) {
            const std::size_t lag      = (kItems - 1UZ) * arm.spacing + 1UZ;
            const std::size_t trials   = kLength - lag;
            const double      expected = static_cast<double>(trials) * static_cast<double>(perPosition);
            const double      spread   = arm.sigmas * std::sqrt(expected);

            AccessCodeCorrelator<std::uint8_t> block = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", codeString(0x000BADC0FFEE1234ULL, kItems)}, //
                {"stride", static_cast<gr::Size_t>(arm.spacing)}, {"max_errors", static_cast<gr::Size_t>(kTolerated)}});
            std::ignore                              = gr::blocks::testing::span::run<std::uint8_t>(block, std::span<const std::uint8_t>(stream));

            const auto seen = static_cast<double>(block.nDetections);
            expect(that % (std::abs(seen - expected) <= spread)) //
                << std::format("s = {} ({}): {} detections in {} trials against {:.1f} expected, {:.1f} allowed", arm.spacing, arm.why, block.nDetections, trials, expected, spread);
        }
    };

    "the settings a strided word needs are refused where they have no meaning"_test = [] {
        const auto refuses = [](gr::property_map settings, std::string_view offender, std::string_view why) {
            std::string message;
            try {
                std::ignore = make<AccessCodeCorrelator<std::uint8_t>>(std::move(settings));
            } catch (const std::exception& error) {
                message = error.what();
            }
            expect(!message.empty()) << why;
            expect(message.contains(offender)) << std::format("{}: \"{}\" is missing from \"{}\"", why, offender, message);
        };

        refuses({{"access_code", std::string("10110100")}, {"stride", gr::Size_t{0}}}, "0", "a stride of zero is not a spacing");
        refuses({{"access_code", std::string("10110100")}, {"stride", gr::Size_t{4097}}}, "4097", "and one above the bounded-state cap");
        refuses({{"access_code", std::string("10110100")}, {"tag_at", std::string("frame_start")}}, "frame_start", "an unrecognized position names itself");

        expect(nothrow([] { std::ignore = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string("10110100")}, {"stride", gr::Size_t{4096}}}); })) << "and the cap itself is legal";
        expect(nothrow([] { std::ignore = make<AccessCodeCorrelator<float>>({{"access_code", std::string("10110100")}, {"stride", gr::Size_t{80}}, {"tag_at", std::string("code_start")}}); })) << "a soft stream strides too";
    };
};

const boost::ut::suite<"the encoded sync marker"> encodedMarkerTests = [] {
    using namespace boost::ut;

    "the encoded sync marker is derived, state-independent, and matches its check value"_test = [] {
        const std::string ccsds = gr::blocks::digital::syncword::ccsdsEncodedAsm("ccsds");
        expect(eq(ccsds.size(), 52UZ)) << "2 * (32 - 6) symbols survive the truncation";
        expect(eq(ccsds, std::string("1000000111001001011100011010101001110011110100111110"))) << "the derivation reproduces the recorded word";

        // the dsn convention emits the same two streams with the symbol pair exchanged
        const std::string dsn     = gr::blocks::digital::syncword::ccsdsEncodedAsm("nasa_dsn");
        bool              swapped = dsn.size() == ccsds.size();
        for (std::size_t t = 0UZ; swapped && 2UZ * t + 1UZ < ccsds.size(); ++t) {
            swapped = dsn[2UZ * t] == ccsds[2UZ * t + 1UZ] && dsn[2UZ * t + 1UZ] == ccsds[2UZ * t];
        }
        expect(swapped) << "one code, two symbol orders";

        // the uninverted twin differs from ccsds in exactly the second symbol of every pair
        const std::string plain  = gr::blocks::digital::syncword::ccsdsEncodedAsm("ccsds_uninverted");
        bool              masked = plain.size() == ccsds.size();
        for (std::size_t t = 0UZ; masked && 2UZ * t + 1UZ < ccsds.size(); ++t) {
            masked = plain[2UZ * t] == ccsds[2UZ * t] && plain[2UZ * t + 1UZ] != ccsds[2UZ * t + 1UZ];
        }
        expect(masked) << "the inversion is the whole of the difference";

        expect(throws([] { std::ignore = gr::blocks::digital::syncword::ccsdsEncodedAsm("voyager"); })) << "an unknown convention lists the ones there are";
    };
};

int main() { /* not needed for UT */ }
