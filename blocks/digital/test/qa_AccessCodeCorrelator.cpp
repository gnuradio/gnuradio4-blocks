#include <boost/ut.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/digital/AccessCodeCorrelator.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::digital::AccessCodeCorrelator;

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

static_assert(AccessCodeCorrelator<std::uint8_t>::keepsTagOffsets(), "keeps tag offsets: no Resampling<> or Stride<>, no asynchronous port, no tag policy and no forwardTags() override");
static_assert(AccessCodeCorrelator<float>::keepsTagOffsets());

} // namespace

const boost::ut::suite<"access code correlator"> accessCodeTests = [] {
    using namespace boost::ut;

    "until it starts the block is inert"_test = [] {
        AccessCodeCorrelator<std::uint8_t>                  block{};
        const std::vector<std::uint8_t>                     zeros(32UZ, 0U);
        std::vector<std::uint8_t>                           out(32UZ);
        std::vector<gr::Tag>                                published;
        gr::blocks::testing::span::InputSpan<std::uint8_t>  inSpan{std::span<const std::uint8_t>(zeros)};
        gr::blocks::testing::span::OutputSpan<std::uint8_t> outSpan{std::span<std::uint8_t>(out), 0UZ, &published};
        expect(that % (block.processBulk(inSpan, outSpan) == gr::work::Status::ERROR));
        expect(eq(inSpan.consumed, 0UZ));
        expect(eq(published.size(), 0UZ));
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

    "the AO-40 geometries are arithmetic"_test = [] {
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
    };

    "a change to the code or the stride starts the warm-up again"_test = [] {
        // max_errors at the word's full length makes every comparison match. The first tag is then the warm-up
        // boundary, counted from the latest change.
        constexpr std::size_t kItems = 8UZ;
        const auto            first  = [](const std::vector<std::size_t>& where) { return where.empty() ? 0UZ : where.front(); };

        AccessCodeCorrelator<std::uint8_t> block = make<AccessCodeCorrelator<std::uint8_t>>({{"access_code", codeString(0xA5ULL, kItems)}, {"stride", gr::Size_t{5}}, {"max_errors", static_cast<gr::Size_t>(kItems)}});
        std::vector<std::uint8_t>          stream(80UZ, std::uint8_t{1});
        expect(eq(first(offsets<std::uint8_t>(block, std::span<const std::uint8_t>(stream))), 36UZ)) << "7 * 5 + 1";
        restage(block, {{"stride", gr::Size_t{3}}});
        expect(eq(first(offsets<std::uint8_t>(block, std::span<const std::uint8_t>(stream))), 22UZ)) << "7 * 3 + 1, counted from the change and not from the stream's start";
        restage(block, {{"access_code", codeString(0x5AULL, kItems)}});
        expect(eq(first(offsets<std::uint8_t>(block, std::span<const std::uint8_t>(stream))), 22UZ)) << "and a new code warms up too";
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
