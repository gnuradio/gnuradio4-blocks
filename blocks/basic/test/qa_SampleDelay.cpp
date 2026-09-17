#include <boost/ut.hpp>

#include <algorithm>
#include <chrono>
#include <complex>
#include <cstdint>
#include <format>
#include <functional>
#include <numeric>
#include <print>
#include <ranges>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/basic/SampleDelay.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace {

using gr::blocks::basic::SampleDelay;
using gr::blocks::testing::Copy;
using gr::testing::ProcessFunction;
using gr::testing::TagSink;
using gr::testing::TagSource;

using CF = std::complex<float>;

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

template<typename T>
void setDelay(SampleDelay<T>& block, gr::Size_t delay) {
    std::ignore = block.settings().setStaged({{"delay", delay}});
    std::ignore = block.settings().applyStagedParameters();
}

/// The pair `ReaderSpan::tags()` yields: a signed offset against the span's own base, and a reference to the map.
struct ToRelIndexMapRef {
    std::size_t base = 0UZ;

    [[nodiscard]] std::pair<std::ptrdiff_t, std::reference_wrapper<const gr::property_map>> operator()(const gr::Tag& tag) const noexcept { //
        return {static_cast<std::ptrdiff_t>(tag.index) - static_cast<std::ptrdiff_t>(base), std::cref(tag.map)};
    }
};

// A ReaderSpanLike/WriterSpanLike pair, so forwardTags can be driven at an exact chunk size and across a settings
// change without standing up a graph. The graph tests below cover the framework's own path.
template<typename T>
struct InSpan : std::span<const T> {
    using value_type = T;

    std::span<const gr::Tag> rawTags{};
    std::size_t              streamIndex = 0UZ;
    bool                     isConnected = true;
    bool                     isSync      = true;

    InSpan(std::span<const T> samples, std::size_t at, std::span<const gr::Tag> tags) : std::span<const T>(samples), rawTags(tags), streamIndex(at) {}

    constexpr bool consume(std::size_t) noexcept { return true; }

    [[nodiscard]] auto tags(std::size_t) const { return rawTags | std::views::transform(ToRelIndexMapRef{streamIndex}); }
};

template<typename T>
struct OutSpan : std::span<T> {
    using value_type = T;

    std::vector<gr::Tag>* sink        = nullptr;
    std::size_t           streamIndex = 0UZ;
    bool                  isConnected = true;
    bool                  isSync      = true;

    OutSpan(std::span<T> samples, std::size_t at, std::vector<gr::Tag>* published) : std::span<T>(samples), sink(published), streamIndex(at) {}

    constexpr void publish(std::size_t) noexcept {}

    void publishTag(const gr::property_map& tagData, std::size_t tagOffset = 0UZ) { sink->push_back(gr::Tag{streamIndex + tagOffset, tagData}); }
};

template<typename T>
struct Result {
    std::vector<T>       samples{};
    std::vector<gr::Tag> tags{};
};

/// @brief Drive @p block over @p input in chunks of @p chunkSize, calling @p before with the absolute offset of each.
template<typename T, typename FBefore>
[[nodiscard]] Result<T> drive(SampleDelay<T>& block, std::span<const T> input, std::size_t chunkSize, std::span<const gr::Tag> tags, FBefore&& before) {
    Result<T>         result;
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    std::vector<T>    scratch(stride);

    for (std::size_t base = 0UZ; base < input.size(); base += stride) {
        const std::size_t count = std::min(stride, input.size() - base);
        before(base);

        const auto first = std::ranges::lower_bound(tags, base, std::ranges::less{}, &gr::Tag::index);
        const auto last  = std::ranges::lower_bound(tags, base + count, std::ranges::less{}, &gr::Tag::index);

        InSpan<T>  inSpan(input.subspan(base, count), base, std::span<const gr::Tag>(first, last));
        OutSpan<T> outSpan(std::span<T>(scratch.data(), count), result.samples.size(), &result.tags);
        auto       inSpans  = std::tie(inSpan);
        auto       outSpans = std::tie(outSpan);

        block.forwardTags(inSpans, outSpans, count);
        std::ignore = block.processBulk(std::span<const T>(input.subspan(base, count)), std::span<T>(scratch.data(), count));
        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(count));
    }
    return result;
}

template<typename T>
[[nodiscard]] std::vector<T> run(SampleDelay<T>& block, std::span<const T> input, std::size_t chunkSize = 0UZ) {
    return drive<T>(block, input, chunkSize, {}, [](std::size_t) {}).samples;
}

template<typename T>
[[nodiscard]] std::vector<T> ramp(std::size_t count) {
    std::vector<T> values(count);
    for (std::size_t i = 0UZ; i < count; ++i) {
        values[i] = static_cast<T>(i);
    }
    return values;
}

/// xoshiro-flavored splitmix, so "random" here is the same bits on every box and every run.
struct Random {
    std::uint64_t state = 0x243f6a8885a308d3ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state += 0x9e3779b97f4a7c15ULL;
        std::uint64_t z = state;
        z               = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z               = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        return z ^ (z >> 31);
    }
};

template<typename T>
[[nodiscard]] std::vector<T> noise(std::size_t count, std::uint64_t seed) {
    Random         rng{seed};
    std::vector<T> values(count);
    for (T& value : values) {
        if constexpr (gr::meta::complex_like<T>) {
            using Real = typename T::value_type;
            value      = T(static_cast<Real>(static_cast<std::int32_t>(rng.next() >> 40)), static_cast<Real>(static_cast<std::int32_t>(rng.next() >> 40)));
        } else {
            value = static_cast<T>(rng.next() >> 40);
        }
    }
    return values;
}

struct Marker {
    const char*    key;
    std::size_t    at;
    gr::pmt::Value value;
};

/// Six keys at five offsets; the first five are `gr::tag::kDefaultTags`, `t0` is not and is what a default-forwarding
/// neighbor drops. The table is a function-local static rather than a namespace-scope one: a `pmt::Value` holding a
/// string owns memory from the polymorphic resource, the suites run from the unit-test runner's destructor, and a
/// table destroyed earlier in that sequence leaves those values dangling.
[[nodiscard]] const std::array<Marker, 6UZ>& markers() {
    static const std::array<Marker, 6UZ> table{{
        {"trigger_name", 0UZ, gr::pmt::Value(std::string("alpha"))},
        {"trigger_time", 1UZ, gr::pmt::Value(std::uint64_t{111})},
        {"trigger_offset", 1UZ, gr::pmt::Value(0.5f)},
        {"num_channels", 37UZ, gr::pmt::Value(gr::Size_t{3})},
        {"rx_overflow", 512UZ, gr::pmt::Value(true)},
        {"t0", 900UZ, gr::pmt::Value(std::string("private"))},
    }};
    return table;
}

constexpr std::size_t kAbsent = std::numeric_limits<std::size_t>::max();

[[nodiscard]] std::vector<std::size_t> offsetsOf(const std::vector<gr::Tag>& tags) {
    std::vector<std::size_t> offsets(markers().size(), kAbsent);
    for (const gr::Tag& tag : tags) {
        for (std::size_t which = 0UZ; which < markers().size(); ++which) {
            const auto found = tag.map.find(gr::property_map::key_type{markers()[which].key});
            if (found != tag.map.end() && found->second == markers()[which].value) {
                offsets[which] = tag.index;
            }
        }
    }
    return offsets;
}

/// @brief Run the markers through a SampleDelay, optionally followed by a default-forwarding neighbor.
[[nodiscard]] std::vector<std::size_t> markersThrough(gr::Size_t delay, bool withNeighbor) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 8000U}, {"mark_tag", false}});
    for (const Marker& marker : markers()) {
        source._tags.emplace_back(marker.at, gr::property_map{{gr::property_map::key_type{marker.key}, marker.value}});
    }
    auto& block = graph.emplaceBlock<SampleDelay<float>>({{"delay", delay}});
    auto& sink  = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

    boost::ut::expect(graph.connect<"out", "in">(source, block).has_value());
    if (withNeighbor) {
        auto& neighbor = graph.emplaceBlock<Copy<float>>();
        boost::ut::expect(graph.connect<"out", "in">(block, neighbor).has_value());
        boost::ut::expect(graph.connect<"out", "in">(neighbor, sink).has_value());
    } else {
        boost::ut::expect(graph.connect<"out", "in">(block, sink).has_value());
    }

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value());
    return offsetsOf(sink._tags);
}

} // namespace

const boost::ut::suite<"SampleDelay"> sampleDelayTests = [] {
    using namespace boost::ut;

    "an impulse comes out delay samples later and nowhere else"_test = [] {
        for (const gr::Size_t delay : {0U, 1U, 7U, 80U, 4096U}) {
            for (const std::size_t position : {0UZ, 1UZ, 1000UZ}) {
                SampleDelay<float> block = make<SampleDelay<float>>({{"delay", delay}});

                std::vector<float> x(6000UZ, 0.f);
                x[position]                    = 1.f;
                const std::vector<float> y     = run<float>(block, x, 137UZ);
                const std::size_t        where = static_cast<std::size_t>(std::ranges::find(y, 1.f) - y.begin());

                expect(eq(where, position + delay)) << std::format("delay {}, impulse at {}", delay, position);
                expect(eq(std::ranges::count(y, 0.f), static_cast<std::ptrdiff_t>(y.size()) - 1)) << "and nowhere else";
            }
        }
    };

    "delay zero is a bit-exact pass-through for every registered type"_test = []<typename T>() {
        SampleDelay<T>       block = make<SampleDelay<T>>();
        const std::vector<T> x     = noise<T>(1000000UZ, 0x13198a2e03707344ULL);
        expect(that % (run<T>(block, x, 4096UZ) == x)) << "delay 0 copies";
    } | std::tuple<std::uint8_t, std::int16_t, std::int32_t, float, double, CF, std::complex<double>>{};

    "the first delay outputs are exactly zero"_test = [] {
        for (const gr::Size_t delay : {1U, 7U, 80U, 4096U}) {
            SampleDelay<CF>       block = make<SampleDelay<CF>>({{"delay", delay}});
            const std::vector<CF> x(8192UZ, CF{1.f, 1.f});
            const std::vector<CF> y = run<CF>(block, x, 999UZ);

            expect(eq(std::ranges::count(std::span<const CF>(y).first(delay), CF{}), static_cast<std::ptrdiff_t>(delay))) << std::format("delay {}: the primed line", delay);
            expect(eq(std::ranges::count(std::span<const CF>(y).subspan(delay), CF{1.f, 1.f}), static_cast<std::ptrdiff_t>(y.size() - delay))) << "and the signal from there on";
        }
    };

    "n inputs make n outputs, across a delay change too"_test = [] {
        for (const gr::Size_t delay : {0U, 1U, 80U, 4096U}) {
            for (const std::size_t chunk : {1UZ, 3UZ, 1000UZ}) {
                SampleDelay<float>       block = make<SampleDelay<float>>({{"delay", delay}});
                const std::vector<float> x     = ramp<float>(5000UZ);
                expect(eq(run<float>(block, x, chunk).size(), x.size())) << std::format("delay {}, chunk {}", delay, chunk);
            }
        }

        SampleDelay<float>       block = make<SampleDelay<float>>({{"delay", 8U}});
        const std::vector<float> x     = ramp<float>(4000UZ);
        const std::vector<float> y     = drive<float>(block, x, 7UZ, {}, [&block](std::size_t base) {
            if (base == 700UZ) {
                setDelay(block, 900U);
            } else if (base == 2100UZ) {
                setDelay(block, 3U);
            }
        }).samples;
        expect(eq(y.size(), x.size())) << "a delay change is still one output per input";
    };

    "the output does not depend on the chunking"_test = [] {
        constexpr gr::Size_t  kDelay    = 80U;
        SampleDelay<CF>       reference = make<SampleDelay<CF>>({{"delay", kDelay}});
        const std::vector<CF> x         = noise<CF>(30000UZ, 0xa4093822299f31d0ULL);
        const std::vector<CF> want      = run<CF>(reference, x, 4096UZ);

        for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 79UZ, 80UZ, 81UZ, 4096UZ}) {
            SampleDelay<CF> block = make<SampleDelay<CF>>({{"delay", kDelay}});
            expect(that % (run<CF>(block, x, chunk) == want)) << std::format("chunk {}", chunk);
        }
    };

    "a delay change inserts zeros or drops samples and stays 1:1"_test = [] {
        constexpr std::size_t    kSeam = 200UZ;
        const std::vector<float> x     = ramp<float>(400UZ);

        SampleDelay<float>       lengthen = make<SampleDelay<float>>({{"delay", 8U}});
        const std::vector<float> up       = drive<float>(lengthen, x, 10UZ, {}, [&lengthen](std::size_t base) {
            if (base == kSeam) {
                setDelay(lengthen, 24U);
            }
        }).samples;

        expect(eq(up.size(), x.size()));
        expect(eq(up[kSeam - 1UZ], static_cast<float>(kSeam - 1UZ - 8UZ))) << "the last sample before the seam";
        for (std::size_t k = 0UZ; k < 16UZ; ++k) {
            expect(eq(up[kSeam + k], 0.f)) << std::format("16 zeros are inserted, at {}", k);
        }
        for (std::size_t k = 0UZ; k < 8UZ; ++k) {
            expect(eq(up[kSeam + 16UZ + k], static_cast<float>(kSeam - 8UZ + k))) << "the ramp resumes where it had reached, nothing repeated";
        }
        expect(eq(up[kSeam + 24UZ], static_cast<float>(kSeam))) << "and continues";

        SampleDelay<float>       shorten = make<SampleDelay<float>>({{"delay", 24U}});
        const std::vector<float> down    = drive<float>(shorten, x, 10UZ, {}, [&shorten](std::size_t base) {
            if (base == kSeam) {
                setDelay(shorten, 8U);
            }
        }).samples;

        expect(eq(down.size(), x.size()));
        expect(eq(down[kSeam - 1UZ], static_cast<float>(kSeam - 1UZ - 24UZ))) << "the last sample before the seam";
        expect(eq(down[kSeam], static_cast<float>(kSeam - 8UZ))) << "the output jumps forward by exactly 16";
        for (std::size_t k = 0UZ; k < 8UZ; ++k) {
            expect(eq(down[kSeam + k], static_cast<float>(kSeam - 8UZ + k)));
        }
    };

    "tags come out delay samples later"_test = [] {
        for (const gr::Size_t delay : {0U, 1U, 4096U}) {
            const std::vector<std::size_t> got = markersThrough(delay, false);
            for (std::size_t which = 0UZ; which < markers().size(); ++which) {
                expect(eq(got[which], markers()[which].at + delay)) << std::format("delay {}, key {}", delay, markers()[which].key);
            }
        }
    };

    "the non-reserved key survives this block and not its neighbor"_test = [] {
        const std::vector<std::size_t> alone = markersThrough(7U, false);
        expect(eq(alone[5UZ], markers()[5UZ].at + 7UZ)) << "a block doing its own forwarding republishes what it saw";

        const std::vector<std::size_t> behind = markersThrough(7U, true);
        expect(eq(behind[5UZ], kAbsent)) << "and the next default-forwarding block drops it, which is the seam";
        for (std::size_t which = 0UZ; which < 5UZ; ++which) {
            expect(neq(behind[which], kAbsent)) << std::format("the reserved key {} still gets through", markers()[which].key);
        }
    };

    "a delay change carries the held tags with their samples"_test = [] {
        const gr::property_map     marker{{gr::property_map::key_type{"trigger_name"}, gr::pmt::Value(std::string("x"))}};
        const gr::property_map     other{{gr::property_map::key_type{"trigger_time"}, gr::pmt::Value(std::uint64_t{7})}};
        const std::vector<gr::Tag> tags{gr::Tag{10UZ, marker}, gr::Tag{30UZ, other}};
        const std::vector<float>   x = ramp<float>(120UZ);

        SampleDelay<float>       lengthen = make<SampleDelay<float>>({{"delay", 24U}});
        const Result<float>      up       = drive<float>(lengthen, x, 4UZ, std::span<const gr::Tag>(tags), [&lengthen](std::size_t base) {
            if (base == 32UZ) {
                setDelay(lengthen, 32U);
            }
        });
        std::vector<std::size_t> where;
        for (const gr::Tag& published : up.tags) {
            where.push_back(published.index);
        }
        expect(that % (where == std::vector<std::size_t>{42UZ, 62UZ})) << "a pending tag comes out D_new after its sample, not D_old";

        SampleDelay<float>  shorten = make<SampleDelay<float>>({{"delay", 24U}});
        const Result<float> down    = drive<float>(shorten, x, 4UZ, std::span<const gr::Tag>(tags), [&shorten](std::size_t base) {
            if (base == 32UZ) {
                setDelay(shorten, 8U);
            }
        });
        where.clear();
        for (const gr::Tag& published : down.tags) {
            where.push_back(published.index);
        }
        expect(that % (where == std::vector<std::size_t>{38UZ})) << "the tag whose sample the shortening discarded goes with it; the survivor comes out at 30 + 8";
    };

    "degenerate settings"_test = [] {
        SampleDelay<std::complex<double>>       block = make<SampleDelay<std::complex<double>>>({{"delay", 16U}});
        const std::vector<std::complex<double>> x(64UZ, std::complex<double>{1.0, 0.0});
        std::ignore = run<std::complex<double>>(block, x, 64UZ);

#ifndef __SANITIZE_ADDRESS__
        // The line has no upper bound of its own, so a delay this large is refused by the allocator rather than by
        // the block. The address sanitizer's allocator aborts on a request it cannot serve instead of reporting it,
        // so this case is asked only of a build whose allocator answers by throwing.
        expect(throws([&block] { setDelay(block, std::numeric_limits<gr::Size_t>::max()); })) << "a line larger than the address space throws the allocator's own exception";

        const std::vector<std::complex<double>> y = run<std::complex<double>>(block, x, 64UZ);
        expect(eq(std::ranges::count(y, std::complex<double>{}), 0)) << "and leaves the previous line intact: still delaying by 16, still primed";
#endif

        SampleDelay<float>       keep = make<SampleDelay<float>>({{"delay", 8U}});
        const std::vector<float> ones(32UZ, 1.f);
        std::ignore = run<float>(keep, ones, 32UZ);
        std::ignore = keep.settings().setStaged({{"delay", gr::Size_t{8}}});
        std::ignore = keep.settings().applyStagedParameters();

        const std::vector<float> after = run<float>(keep, ones, 32UZ);
        expect(eq(std::ranges::count(after, 0.f), 0)) << "a settings map that does not change delay does not reset the line";
    };

    "the cost stays inside 1.5x a span copy at every delay"_test = [] {
        using Clock                   = std::chrono::steady_clock;
        constexpr std::size_t kLength = 1UZ << 22;
        constexpr std::size_t kChunk  = 4096UZ;
        constexpr int         kRuns   = 7;
        constexpr gr::Size_t  kDelays[]{1U, 80U, 4095U, 65536U};

        const std::vector<float> x = noise<float>(kLength, 0x082efa98ec4e6c89ULL);
        std::vector<float>       y(kLength);

        std::vector<SampleDelay<float>> blocks;
        for (const gr::Size_t delay : kDelays) {
            blocks.push_back(make<SampleDelay<float>>({{"delay", delay}}));
        }

        std::vector<double> best(std::size(kDelays) + 1UZ, 1e300);
        std::vector<double> worst(std::size(kDelays) + 1UZ, 0.0);
        for (int repeat = 0; repeat <= kRuns; ++repeat) { // interleaved, so a thermal excursion lands on all the arms
            for (std::size_t arm = 0UZ; arm <= std::size(kDelays); ++arm) {
                const auto start = Clock::now();
                for (std::size_t base = 0UZ; base < kLength; base += kChunk) {
                    if (arm == 0UZ) {
                        std::copy_n(x.begin() + static_cast<std::ptrdiff_t>(base), kChunk, y.begin() + static_cast<std::ptrdiff_t>(base));
                    } else {
                        std::ignore = blocks[arm - 1UZ].processBulk(std::span<const float>(x).subspan(base, kChunk), std::span<float>(y).subspan(base, kChunk));
                    }
                }
                const double ns = std::chrono::duration<double, std::nano>(Clock::now() - start).count() / static_cast<double>(kLength);
                if (repeat > 0) {
                    best[arm]  = std::min(best[arm], ns);
                    worst[arm] = std::max(worst[arm], ns);
                }
            }
        }

        // The tight 1.5x bound is asserted under ENABLE_BENCHMARK_TESTS, where the harness controls placement. The
        // loose 3x holds otherwise and still catches a shifted line, which costs 5.6x.
        const double bound = std::getenv("ENABLE_BENCHMARK_TESTS") != nullptr ? 1.5 : 3.0;

        std::println("span copy {:.3f} ns/sample (spread {:.3f})", best[0UZ], worst[0UZ] - best[0UZ]);
        for (std::size_t arm = 1UZ; arm <= std::size(kDelays); ++arm) {
            const double ratio = best[arm] / best[0UZ];
            std::println("delay {:>6} {:.3f} ns/sample (spread {:.3f}), {:.2f}x the copy", kDelays[arm - 1UZ], best[arm], worst[arm] - best[arm], ratio);
            expect(lt(ratio, bound)) << std::format("delay {} costs {:.2f}x a span copy; a shifted line reaches 5.6x here", kDelays[arm - 1UZ], ratio);
        }
    };
};

int main() { /* not needed for UT */ }
