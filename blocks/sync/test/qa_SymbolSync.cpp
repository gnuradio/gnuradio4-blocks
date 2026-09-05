#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <numbers>
#include <random>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/sync/SymbolSync.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

// The interpolator's normal equations, its response accuracy and its differentiator belong to
// gr::sync::MmseInterpolatorBank, and the ten-row gain table, the S-curves and the self-noise figures belong to
// gr::sync::TimingErrorDetector; both are pinned by their own QA. What is tested here is the block: the branch-major
// partition it owns, the scheduling, the position accumulator's integer tag arithmetic, the two steering tags, and the
// closed loop the four pieces make between them.

namespace {

using gr::blocks::sync::SymbolSync;
namespace test = gr::blocks::testing::span;

using CF = std::complex<float>;
using CD = std::complex<double>;

constexpr double kPi   = std::numbers::pi;
constexpr double kRoot = std::numbers::sqrt2 / 2.0;

template<typename T>
struct FiniteSource : gr::Block<FiniteSource<T>> {
    gr::PortOut<T> out;
    GR_MAKE_REFLECTABLE(FiniteSource, out);
    std::vector<T>                 _data;
    std::size_t                    _pos{0UZ};
    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) noexcept {
        const std::size_t n = std::min(outSpan.size(), _data.size() - _pos);
        std::copy_n(_data.begin() + static_cast<std::ptrdiff_t>(_pos), n, outSpan.begin());
        outSpan.publish(n);
        _pos += n;
        return _pos == _data.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

template<typename T>
struct CountSink : gr::Block<CountSink<T>> {
    gr::PortIn<T> in;
    GR_MAKE_REFLECTABLE(CountSink, in);
    std::atomic<std::size_t>       _count{0UZ};
    [[nodiscard]] gr::work::Status processBulk(std::span<const T> input) noexcept {
        _count += input.size();
        return gr::work::Status::OK;
    }
};

/// @brief Hands on at most a drawn number of samples a call, so the span sizes the block behind it sees are this
/// block's and not the scheduler's. Zero-length calls do not draw, so the sequence follows the samples.
template<typename T>
struct ChunkLimiter : gr::Block<ChunkLimiter<T>> {
    gr::PortIn<T, gr::Async>  in;
    gr::PortOut<T, gr::Async> out;
    GR_MAKE_REFLECTABLE(ChunkLimiter, in, out);

    std::uint64_t _state = 1ULL;
    std::size_t   _fixed = 0UZ; ///< non-zero: every call takes this many; zero: a seeded draw in [1, _max]
    std::size_t   _max   = 4096UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) {
        const std::size_t room = std::min(inSpan.size(), outSpan.size());
        if (room == 0UZ) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        if (_fixed == 0UZ) {
            _state = _state * 6364136223846793005ULL + 1442695040888963407ULL;
        }
        const std::uint32_t draw = static_cast<std::uint32_t>(_state >> 33); // 31 bits, so the draw is exact where std::size_t is 32 bits too
        const std::size_t   n    = std::min(room, _fixed != 0UZ ? _fixed : 1UZ + draw % _max);
        std::copy_n(inSpan.begin(), n, outSpan.begin());
        std::ignore = inSpan.consume(n);
        outSpan.publish(n);
        return gr::work::Status::OK;
    }
};

template<typename T>
struct ValueSink : gr::Block<ValueSink<T>> {
    gr::PortIn<T> in;
    GR_MAKE_REFLECTABLE(ValueSink, in);
    std::vector<T>                 _values{};
    [[nodiscard]] gr::work::Status processBulk(std::span<const T> input) {
        _values.insert(_values.end(), input.begin(), input.end());
        return gr::work::Status::OK;
    }
};

template<typename T>
[[nodiscard]] SymbolSync<T> make(gr::property_map settings = {}) {
    SymbolSync<T> block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

template<typename T>
[[nodiscard]] auto drive(SymbolSync<T>& block, std::span<const T> input, std::size_t feed = 0UZ, std::size_t room = 0UZ, std::span<const gr::Tag> tags = {}, std::size_t at = 0UZ) {
    return test::runPortsVariable<3UZ, T>(block, input, feed, room, std::array<bool, 3UZ>{true, true, true}, tags, at);
}

/// @brief The raised cosine, the pulse a root-raised-cosine transmitter matched by a root-raised-cosine receiver leaves.
[[nodiscard]] double raisedCosineAt(double t, double alpha) {
    const double edge = 2.0 * alpha * t;
    if (alpha > 0.0 && std::abs(std::abs(edge) - 1.0) < 1e-9) {
        return 0.5 * alpha * std::sin(kPi / (2.0 * alpha));
    }
    return gr::filter::fir::design::sincPi(t) * std::cos(kPi * alpha * t) / (1.0 - edge * edge);
}

/// @brief Unit-power QPSK or BPSK on an ideal Nyquist channel at @p sps samples per symbol, sampled @p offset late.
[[nodiscard]] std::vector<CF> nyquistPsk(std::size_t nSamples, double sps, double alpha, bool qpsk, double offset, std::uint32_t seed) {
    constexpr int     kSpan    = 12;
    const std::size_t nSymbols = static_cast<std::size_t>(static_cast<double>(nSamples) / sps) + 2UZ * kSpan + 2UZ;

    std::mt19937                       rng(seed);
    std::uniform_int_distribution<int> pick(0, 3);
    std::vector<CD>                    symbols(nSymbols);
    for (std::size_t k = 0UZ; k < nSymbols; ++k) {
        const int choice = pick(rng);
        symbols[k]       = qpsk ? CD{kRoot * ((choice & 1) ? 1.0 : -1.0), kRoot * ((choice & 2) ? 1.0 : -1.0)} : CD{(choice & 1) ? 1.0 : -1.0, 0.0};
    }

    std::vector<CF> signal(nSamples);
    for (std::size_t n = 0UZ; n < nSamples; ++n) {
        const double time   = (static_cast<double>(n) - offset) / sps;
        const auto   center = static_cast<std::ptrdiff_t>(std::floor(time));
        CD           sum{};
        for (std::ptrdiff_t k = center - kSpan; k <= center + kSpan; ++k) {
            if (k >= 0 && static_cast<std::size_t>(k) < nSymbols) {
                sum += symbols[static_cast<std::size_t>(k)] * raisedCosineAt(time - static_cast<double>(k), alpha);
            }
        }
        signal[n] = static_cast<CF>(sum);
    }
    return signal;
}

/// @brief Unit-power PAM4 on an ideal Nyquist channel at @p sps samples per symbol, sampled @p offset late.
[[nodiscard]] std::vector<float> nyquistPam4(std::size_t nSamples, double sps, double alpha, double offset, std::uint32_t seed) {
    constexpr int     kSpan    = 12;
    constexpr double  kInner   = 0.4472135954999579; // 1/sqrt(5): unit average power over the four levels
    const std::size_t nSymbols = static_cast<std::size_t>(static_cast<double>(nSamples) / sps) + 2UZ * kSpan + 2UZ;

    std::mt19937                       rng(seed);
    std::uniform_int_distribution<int> pick(0, 3);
    std::vector<double>                symbols(nSymbols);
    for (double& s : symbols) {
        s = static_cast<double>(2 * pick(rng) - 3) * kInner;
    }

    std::vector<float> signal(nSamples);
    for (std::size_t n = 0UZ; n < nSamples; ++n) {
        const double time   = (static_cast<double>(n) - offset) / sps;
        const auto   center = static_cast<std::ptrdiff_t>(std::floor(time));
        double       sum    = 0.0;
        for (std::ptrdiff_t k = center - kSpan; k <= center + kSpan; ++k) {
            if (k >= 0 && static_cast<std::size_t>(k) < nSymbols) {
                sum += symbols[static_cast<std::size_t>(k)] * raisedCosineAt(time - static_cast<double>(k), alpha);
            }
        }
        signal[n] = static_cast<float>(sum);
    }
    return signal;
}

/// @brief The mean distance from a unit-power PAM4 sample to the nearest of its four levels.
[[nodiscard]] double meanPamError(std::span<const float> samples, std::size_t from) {
    constexpr double kInner = 0.4472135954999579;
    double           sum    = 0.0;
    for (std::size_t i = from; i < samples.size(); ++i) {
        const double magnitude = std::abs(static_cast<double>(samples[i]));
        const double level     = magnitude > 2.0 * kInner ? 3.0 * kInner : kInner;
        sum += std::abs(magnitude - level);
    }
    return sum / static_cast<double>(samples.size() - from);
}

/// @brief The mean distance from a unit-power QPSK sample to its own decision.
[[nodiscard]] double meanErrorVector(std::span<const CF> samples, std::size_t from) {
    double sum = 0.0;
    for (std::size_t i = from; i < samples.size(); ++i) {
        const CD decision{samples[i].real() > 0.f ? kRoot : -kRoot, samples[i].imag() > 0.f ? kRoot : -kRoot};
        sum += std::abs(static_cast<CD>(samples[i]) - decision);
    }
    return sum / static_cast<double>(samples.size() - from);
}

[[nodiscard]] double meanOfTail(std::span<const float> values, std::size_t count) {
    double sum = 0.0;
    for (std::size_t i = values.size() - count; i < values.size(); ++i) {
        sum += static_cast<double>(values[i]);
    }
    return sum / static_cast<double>(count);
}

/// @brief The anchors a free-running accumulator lands on: `base + L/2 - 1`, `base` advanced by a constant period.
[[nodiscard]] std::vector<std::uint64_t> anchorsOf(double period, std::uint64_t anchorDelay, std::size_t count) {
    std::vector<std::uint64_t> anchors(count);
    std::uint64_t              base = 0ULL;
    double                     mu   = 0.0;
    for (std::size_t i = 0UZ; i < count; ++i) {
        anchors[i]            = base + anchorDelay;
        const double advanced = mu + period;
        const double whole    = std::floor(advanced);
        mu                    = advanced - whole;
        base += static_cast<std::uint64_t>(whole);
    }
    return anchors;
}

/// @brief The output index a tag at input offset @p at belongs on: the first anchor that has reached it.
[[nodiscard]] std::size_t placementOf(std::span<const std::uint64_t> anchors, std::uint64_t at) { return static_cast<std::size_t>(std::ranges::distance(anchors.begin(), std::ranges::lower_bound(anchors, at))); }

[[nodiscard]] gr::Tag markerTag(std::size_t at, double which) { return gr::Tag{at, gr::property_map{{"marker", which}}}; }

} // namespace

const boost::ut::suite<"SymbolSync"> symbolSyncTests = [] {
    using namespace boost::ut;

    "the polyphase partition is the zero-stuffed convolution"_test = [] {
        constexpr std::size_t kArms   = 16UZ;
        constexpr std::size_t kBranch = 12UZ;

        const std::vector<float> prototype = gr::filter::fir::design::rootRaisedCosine(kArms * kBranch - 1UZ, static_cast<double>(kArms) * 3.0, 0.35, static_cast<double>(kArms));
        const std::vector<float> branches  = gr::blocks::sync::partitionBranchMajor(prototype, kArms);
        expect(eq(branches.size(), kArms * kBranch));

        std::mt19937                          rng(11U);
        std::uniform_real_distribution<float> noise(-1.f, 1.f);
        std::vector<float>                    x(64UZ);
        for (float& value : x) {
            value = noise(rng);
        }

        std::vector<float> stuffed(x.size() * kArms, 0.f);
        for (std::size_t k = 0UZ; k < x.size(); ++k) {
            stuffed[k * kArms] = x[k];
        }

        double worst = 0.0;
        for (std::size_t i = kBranch; i + 2UZ < x.size(); ++i) {
            for (std::size_t branch = 0UZ; branch < kArms; ++branch) {
                double direct = 0.0;
                for (std::size_t m = 0UZ; m < prototype.size(); ++m) {
                    const std::size_t at = i * kArms + branch;
                    if (m <= at && at - m < stuffed.size()) {
                        direct += static_cast<double>(prototype[m]) * static_cast<double>(stuffed[at - m]);
                    }
                }
                double partitioned = 0.0;
                for (std::size_t m = 0UZ; m < kBranch; ++m) {
                    partitioned += static_cast<double>(branches[branch * kBranch + m]) * static_cast<double>(x[i - kBranch + 1UZ + m]);
                }
                worst = std::max(worst, std::abs(direct - partitioned));
            }
        }
        expect(lt(worst, 1e-6)) << "branch-major partition against the zero-stuffed convolution, worst " << worst;
    };

    "a delta prototype makes the polyphase path plain sample selection"_test = [] {
        constexpr std::size_t kArms   = 8UZ;
        constexpr std::size_t kBranch = 6UZ;

        std::vector<float> prototype(kArms * kBranch, 0.f);
        prototype[(kBranch / 2UZ) * kArms] = static_cast<float>(kArms);

        const std::vector<float> branches = gr::blocks::sync::partitionBranchMajor(prototype, kArms);
        for (std::size_t m = 0UZ; m < kBranch; ++m) {
            const float expected = m == kBranch / 2UZ - 1UZ ? static_cast<float>(kArms) : 0.f;
            expect(eq(branches[m], expected)) << "branch 0 tap " << m;
        }
        for (std::size_t branch = 1UZ; branch < kArms; ++branch) {
            for (std::size_t m = 0UZ; m < kBranch; ++m) {
                expect(eq(branches[branch * kBranch + m], 0.f)) << "a delta lives on one branch only";
            }
        }
    };

    "the detector gain reaches the kernel per input sample"_test = [] {
        for (const double sps : {2.0, 4.0, 8.0}) {
            SymbolSync<CF> block = make<CF>({{"samples_per_symbol", sps}, {"rolloff", 0.35}});
            const double   kted  = gr::sync::muellerMullerGain(0.35);
            expect(lt(std::abs(block.loop().detectorGain() - kted / sps), 1e-12)) << "per input sample at sps=" << sps;
        }

        SymbolSync<CF> modified = make<CF>({{"detector", std::string("ModifiedMuellerMuller")}, {"rolloff", 0.35}, {"samples_per_symbol", 4.0}});
        expect(lt(std::abs(modified.loop().detectorGain() - 2.0 * gr::sync::muellerMullerGain(0.35) / 4.0), 1e-12)) << "the modified form is exactly twice";

        SymbolSync<CF> given = make<CF>({{"detector", std::string("Gardner")}, {"detector_gain", 1.06739}, {"samples_per_symbol", 4.0}});
        expect(lt(std::abs(given.loop().detectorGain() - 1.06739 / 4.0), 1e-12)) << "a measured Kted divides by sps too";
    };

    "the interpolation clock is the least common multiple"_test = [] {
        for (const auto& [name, outputs, expected] : std::array<std::tuple<std::string, gr::Size_t, int>, 5UZ>{{{"MuellerMuller", 1U, 1}, {"MuellerMuller", 2U, 2}, {"Gardner", 1U, 2}, {"Gardner", 2U, 2}, {"Gardner", 3U, 6}}}) {
            SymbolSync<CF> block = make<CF>({{"detector", name}, {"outputs_per_symbol", outputs}, {"detector_gain", 1.0}, {"samples_per_symbol", 8.0}});
            expect(eq(block.interpolationsPerSymbol(), expected)) << name << " at " << outputs << " outputs per symbol";
        }
    };

    "degenerate parameters throw"_test = [] {
        expect(throws([] { std::ignore = make<CF>({{"samples_per_symbol", 1.0}}); })) << "samples_per_symbol must exceed 1";
        expect(throws([] { std::ignore = make<CF>({{"samples_per_symbol", 0.5}}); })) << "nor below it";
        expect(throws([] { std::ignore = make<CF>({{"outputs_per_symbol", gr::Size_t(0)}}); })) << "outputs_per_symbol must be positive";
        expect(throws([] { std::ignore = make<CF>({{"samples_per_symbol", 4.0}, {"max_deviation", 4.0}}); })) << "a deviation that reaches sps allows a non-positive period";
        expect(throws([] { std::ignore = make<CF>({{"detector_gain", -1.0}}); })) << "a negative gain is a sign error";
        expect(throws([] { std::ignore = make<CF>({{"detector", std::string("Gardner")}, {"rolloff", 0.0}}); })) << "a blind detector has no gain without excess bandwidth";
        expect(throws([] { std::ignore = make<CF>({{"detector", std::string("SignalTimesSlopeMl")}, {"rolloff", 0.0}}); })) << "nor has the maximum-likelihood form";
        expect(throws([] { std::ignore = make<CF>({{"interpolator", std::string("cubic")}}); })) << "an unknown interpolator name";
        expect(throws([] { std::ignore = make<CF>({{"constellation", std::string("8psk")}}); })) << "an unknown constellation";
        expect(throws([] { std::ignore = make<float>({{"constellation", std::string("qpsk")}}); })) << "a real stream cannot carry qpsk";
        expect(throws([] { std::ignore = make<CF>({{"detector", std::string("SignalTimesSlopeMl")}, {"detector_gain", 1.0}, {"interpolator", std::string("polyphase")}}); })) << "the polyphase path has no differentiator";

        expect(nothrow([] { std::ignore = make<CF>({{"samples_per_symbol", 4.0}, {"max_deviation", 3.999}}); })) << "just inside the bound is legal";
    };

    "every detector starts on the rolloff alone"_test = [] {
        // Each name takes its gain from the raised-cosine channel the `rolloff` setting describes, so a graph that
        // names a detector and shapes with a matching root-raised cosine needs no `detector_gain` at all.
        for (std::string name : {"MuellerMuller", "ModifiedMuellerMuller", "ZeroCrossing", "Gardner", "EarlyLate", "SignalTimesSlopeMl", "SignumTimesSlopeMl"}) {
            expect(nothrow([&name] { std::ignore = make<CF>({{"detector", name}}); })) << name << " starts without an explicit gain";
        }
    };

    "the position accumulator places tags at the first anchor that reaches them"_test = [] {
        for (const double sps : {4.0, 4.3}) {
            const std::vector<std::uint64_t> anchors = anchorsOf(sps, 3ULL, 64UZ);
            expect(eq(anchors[0], 3ULL)) << "the first instant sits L/2 - 1 into the stream";

            std::vector<gr::Tag> tags;
            for (const std::size_t at : {0UZ, 3UZ, 4UZ, 7UZ, 8UZ, 20UZ, 41UZ, 100UZ}) {
                tags.push_back(markerTag(at, static_cast<double>(at)));
            }

            const std::vector<CF> silence(400UZ, CF{});
            SymbolSync<CF>        block   = make<CF>({{"samples_per_symbol", sps}});
            const auto            tracked = drive<CF>(block, std::span<const CF>(silence), 0UZ, 0UZ, std::span<const gr::Tag>(tags));

            expect(eq(tracked.tags.size(), tags.size())) << "every tag survives, none is merged away";
            for (std::size_t i = 0UZ; i < tags.size(); ++i) {
                expect(eq(tracked.tags[i].index, placementOf(std::span<const std::uint64_t>(anchors), tags[i].index))) << "sps=" << sps << " tag at input " << tags[i].index;
            }
        }
    };

    "tag placement is exact at an absolute offset a double could not hold"_test = [] {
        if constexpr (sizeof(std::size_t) != 8UZ) {
            return; // the offsets below are past 2^53, which a 32-bit std::size_t does not hold at all
        }
        constexpr std::size_t kFar = static_cast<std::size_t>((1ULL << 53U) + 1ULL);

        const std::vector<std::uint64_t> anchors = anchorsOf(4.0, 3ULL, 64UZ);
        std::vector<gr::Tag>             tags;
        for (const std::size_t at : {0UZ, 5UZ, 6UZ, 6UZ, 33UZ}) {
            tags.push_back(markerTag(kFar + at, static_cast<double>(at)));
        }

        const std::vector<CF> silence(200UZ, CF{});
        SymbolSync<CF>        block   = make<CF>({{"samples_per_symbol", 4.0}});
        const auto            tracked = drive<CF>(block, std::span<const CF>(silence), 0UZ, 0UZ, std::span<const gr::Tag>(tags), kFar);

        expect(eq(tracked.tags.size(), tags.size())) << "multiplicity is preserved";
        for (std::size_t i = 0UZ; i < tags.size(); ++i) {
            expect(eq(tracked.tags[i].index, placementOf(std::span<const std::uint64_t>(anchors), tags[i].index - kFar))) << "far tag " << i;
        }
    };

    "time_est repositions the interpolator onto the instant it names"_test = [] {
        constexpr std::size_t kSamples = 260UZ;
        constexpr std::size_t kAt      = 200UZ;
        const std::vector<CF> silence(kSamples, CF{});

        SymbolSync<CF> plain    = make<CF>({{"samples_per_symbol", 4.0}});
        const auto     baseline = drive<CF>(plain, std::span<const CF>(silence));

        for (const double wanted : {0.25, -0.75}) {
            const std::vector<gr::Tag> tags{gr::Tag{kAt, gr::property_map{{"time_est", wanted}}}};

            SymbolSync<CF> block   = make<CF>({{"samples_per_symbol", 4.0}});
            const auto     tracked = drive<CF>(block, std::span<const CF>(silence), 0UZ, 0UZ, std::span<const gr::Tag>(tags));

            const double instant  = static_cast<double>(block._base) + 3.0 + block._mu;
            const double residual = std::remainder(instant - (static_cast<double>(kAt) + wanted), 4.0);
            expect(lt(std::abs(residual), 0.04)) << "the instant lattice must contain " << (static_cast<double>(kAt) + wanted) << ", got " << instant;
            expect(eq(tracked.tags.size(), 0UZ)) << "time_est is consumed, not forwarded";
            expect(eq(block.ignoredTagPayloads(), 0ULL));
        }

        const double untagged = static_cast<double>(plain._base) + 3.0 + plain._mu;
        expect(gt(std::abs(std::remainder(untagged - (static_cast<double>(kAt) + 0.25), 4.0)), 0.2)) << "and the untagged run must not already be there";
        expect(gt(baseline.samples.size(), 50UZ));
    };

    "clock_est sets the average period as well, and an out-of-range payload changes nothing"_test = [] {
        constexpr std::size_t kSamples = 260UZ;
        const std::vector<CF> silence(kSamples, CF{});

        const std::vector<gr::Tag> good{gr::Tag{200UZ, gr::property_map{{"clock_est", std::vector<double>{0.5, 4.25}}}}};
        SymbolSync<CF>             block   = make<CF>({{"samples_per_symbol", 4.0}});
        const auto                 tracked = drive<CF>(block, std::span<const CF>(silence), 0UZ, 0UZ, std::span<const gr::Tag>(good));

        expect(lt(std::abs(block.loop().frequency() - 4.25), 1e-12)) << "the period the tag names is the tracked period";
        expect(lt(std::abs(static_cast<double>(tracked.aux[2].back()) - 4.25), 1e-6)) << "and the average_period port says so";
        expect(eq(tracked.tags.size(), 0UZ)) << "clock_est is consumed";

        SymbolSync<CF> reference = make<CF>({{"samples_per_symbol", 4.0}});
        const auto     plain     = drive<CF>(reference, std::span<const CF>(silence));

        for (const auto& payload : std::array<gr::property_map, 3UZ>{gr::property_map{{"time_est", 5.0}}, gr::property_map{{"time_est", std::string("soon")}}, gr::property_map{{"clock_est", std::vector<double>{0.25, 99.0}}}}) {
            const std::vector<gr::Tag> tags{gr::Tag{200UZ, payload}};
            SymbolSync<CF>             ignored = make<CF>({{"samples_per_symbol", 4.0}});
            const auto                 result  = drive<CF>(ignored, std::span<const CF>(silence), 0UZ, 0UZ, std::span<const gr::Tag>(tags));

            expect(gt(ignored.ignoredTagPayloads(), 0ULL)) << "a rejected payload is counted";
            expect(eq(result.samples.size(), plain.samples.size())) << "and leaves the run identical";
            expect(eq(ignored._base, reference._base)) << "bit-identical position";
            expect(eq(ignored._mu, reference._mu));
        }
    };

    "trigger names whose presets to honor, and trigger_presets turns them all off"_test = [] {
        // A stream can carry more than one timing producer, so a block has to be able to say which one it acts on.
        // Every arm is read off the position the run leaves, not off a status flag.
        constexpr std::size_t kSamples = 260UZ;
        const std::vector<CF> silence(kSamples, CF{});

        const auto steering = [](std::string_view label) {
            gr::property_map map{{"clock_est", std::vector<double>{0.5, 4.25}}};
            if (!label.empty()) {
                map[gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey())] = std::string(label);
            }
            return std::vector<gr::Tag>{gr::Tag{200UZ, map}};
        };

        SymbolSync<CF> untagged  = make<CF>({{"samples_per_symbol", 4.0}});
        const auto     freeRun   = drive<CF>(untagged, std::span<const CF>(silence));
        const double   plainBase = static_cast<double>(untagged._base) + untagged._mu;

        SymbolSync<CF> presetOnly = make<CF>({{"samples_per_symbol", 4.0}});
        const auto     presetRun  = drive<CF>(presetOnly, std::span<const CF>(silence), 0UZ, 0UZ, std::span<const gr::Tag>(steering("")));
        const double   presetBase = static_cast<double>(presetOnly._base) + presetOnly._mu;
        expect(gt(std::abs(presetBase - plainBase), 0.1)) << "an honored preset moves the position, or nothing below distinguishes the arms";

        for (const auto& [label, honored] : std::array<std::pair<std::string_view, bool>, 2UZ>{{{"preamble", true}, {"other", false}}}) {
            SymbolSync<CF> gated  = make<CF>({{"samples_per_symbol", 4.0}, {"trigger", std::string("preamble")}});
            const auto     result = drive<CF>(gated, std::span<const CF>(silence), 0UZ, 0UZ, std::span<const gr::Tag>(steering(label)));
            const double   base   = static_cast<double>(gated._base) + gated._mu;

            expect(lt(std::abs(base - (honored ? presetBase : plainBase)), 1e-12)) << "trigger 'preamble' against a tag named '" << label << "'";
            expect(eq(gated.ignoredTagPayloads(), honored ? 0ULL : 1ULL)) << "a payload meant for another producer is counted, not silently dropped";
            expect(that % (result.samples == (honored ? presetRun.samples : freeRun.samples))) << "and the stream it leaves is the one that arm's own run leaves, sample for sample";
            expect(eq(result.tags.size(), 1UZ)) << "and the trigger_name it carried rides through as an ordinary tag";
        }

        for (const std::string_view label : {"preamble", "other"}) {
            SymbolSync<CF> open = make<CF>({{"samples_per_symbol", 4.0}});
            std::ignore         = drive<CF>(open, std::span<const CF>(silence), 0UZ, 0UZ, std::span<const gr::Tag>(steering(label)));
            expect(lt(std::abs(static_cast<double>(open._base) + open._mu - presetBase), 1e-12)) << "an empty trigger honors a timing tag whatever name it carries, '" << label << "'";
            expect(eq(open.ignoredTagPayloads(), 0ULL));

            SymbolSync<CF> refusing = make<CF>({{"samples_per_symbol", 4.0}, {"trigger_presets", false}});
            const auto     ignored  = drive<CF>(refusing, std::span<const CF>(silence), 0UZ, 0UZ, std::span<const gr::Tag>(steering(label)));
            expect(lt(std::abs(static_cast<double>(refusing._base) + refusing._mu - plainBase), 1e-12)) << "trigger_presets false ignores every payload, '" << label << "'";
            expect(eq(refusing.ignoredTagPayloads(), 1ULL)) << "and counts it";
            expect(eq(ignored.samples.size(), freeRun.samples.size()));
        }
    };

    "a timing tag whose target the position has passed is ignored and counted"_test = [] {
        // The state after a request that cannot be honored is the state that was there. Applying it with the advance
        // clamped at zero instead would force `mu` to zero and clear the detector history at a phase nobody asked for,
        // destroying a position the loop already holds in order to honor a request it cannot reach.
        constexpr std::size_t kSamples = 300UZ;
        const std::vector<CF> silence(kSamples, CF{});

        for (const bool withPeriod : {true, false}) {
            const auto payload = [withPeriod](double instant) {
                gr::property_map map;
                if (withPeriod) {
                    map[gr::property_map::key_type("clock_est")] = std::vector<double>{instant, 4.0};
                } else {
                    map[gr::property_map::key_type("time_est")] = instant;
                }
                return map;
            };

            // the first tag lands the instant on 201, which puts the window base two samples behind it; the second
            // names an instant at 200, which the anchor has then already gone past
            const std::vector<gr::Tag> both{gr::Tag{200UZ, payload(1.0)}, gr::Tag{201UZ, payload(-1.0)}};
            const std::vector<gr::Tag> first{gr::Tag{200UZ, payload(1.0)}};

            SymbolSync<CF> reachable = make<CF>({{"samples_per_symbol", 4.0}});
            const auto     honored   = drive<CF>(reachable, std::span<const CF>(silence), 0UZ, 0UZ, std::span<const gr::Tag>(first));

            SymbolSync<CF> stale  = make<CF>({{"samples_per_symbol", 4.0}});
            const auto     result = drive<CF>(stale, std::span<const CF>(silence), 0UZ, 0UZ, std::span<const gr::Tag>(both));

            expect(eq(stale.ignoredTagPayloads(), 1ULL)) << "the unreachable payload is counted, period carried: " << withPeriod;
            expect(eq(reachable.ignoredTagPayloads(), 0ULL)) << "and the reachable one is not";
            expect(eq(stale._base, reachable._base)) << "the position is the one the honored tag left";
            expect(that % (stale._mu == reachable._mu));
            expect(lt(std::abs(stale.loop().frequency() - reachable.loop().frequency()), 1e-15));
            expect(eq(result.samples.size(), honored.samples.size()));
            expect(that % (result.samples == honored.samples)) << "and the output is bit-identical to the run without it";
        }

        // A tag whose offset itself precedes the base cannot be built through this block: a tag is collected only
        // once, on the first call that sees it, and that call's window starts past everything already consumed, so
        // the offset is always at or ahead of the base. What can be behind is the instant the payload names, which is
        // the case above.
        SymbolSync<CF>             single = make<CF>({{"samples_per_symbol", 4.0}});
        const std::vector<gr::Tag> late{gr::Tag{200UZ, gr::property_map{{"clock_est", std::vector<double>{-1.0, 4.0}}}}};
        std::ignore = drive<CF>(single, std::span<const CF>(silence), 0UZ, 0UZ, std::span<const gr::Tag>(late));
        expect(eq(single.ignoredTagPayloads(), 0ULL)) << "a single reachable tag is honored however early in its symbol it names an instant";
    };

    "a preset re-times a sample instead of taking one out of the stream"_test = [] {
        // The instant a tag names lies inside one interpolation step. Snapping the position at the step whose own
        // instant that measurement names re-times the sample that step takes; snapping a step early takes the sample
        // out of the stream instead, and the anchor before the preset then sits two symbols from the anchor after it.
        // On a burst link the missing sample is the one at the boundary the tag marks, which a differential decoder
        // downstream turns into a wrong bit. Markers on every input sample are what make the gap readable: the run of
        // markers sharing an output offset is the number of input samples one anchor stands past the one before it.
        constexpr double      kPeriod = 8.0;
        constexpr std::size_t kFirst  = 160UZ;
        constexpr std::size_t kLast   = 260UZ;
        const std::vector<CF> silence(400UZ, CF{});

        for (const std::size_t at : {201UZ, 202UZ, 203UZ, 204UZ}) {
            for (const double fraction : {-0.5, 0.0, 0.5}) {
                std::vector<gr::Tag> tags;
                for (std::size_t i = kFirst; i <= kLast; ++i) {
                    tags.push_back(markerTag(i, static_cast<double>(i)));
                    if (i == at) {
                        tags.push_back(gr::Tag{i, gr::property_map{{"clock_est", std::vector<double>{fraction, kPeriod}}}});
                    }
                }

                SymbolSync<CF> block   = make<CF>({{"samples_per_symbol", kPeriod}});
                const auto     tracked = drive<CF>(block, std::span<const CF>(silence), 0UZ, 0UZ, std::span<const gr::Tag>(tags));

                expect(eq(tracked.tags.size(), kLast - kFirst + 1UZ)) << "every marker rides through and the timing payload is consumed";
                expect(eq(block.ignoredTagPayloads(), 0ULL)) << "the payload is honored, or the arm asserts nothing";

                const double instant = static_cast<double>(block._base) + 3.0 + block._mu;
                expect(lt(std::abs(std::remainder(instant - (static_cast<double>(at) + fraction), kPeriod)), 1e-9)) << "the lattice the run leaves contains the instant named, tag at " << at << " " << fraction;

                std::size_t longest = 0UZ;
                std::size_t run     = 0UZ;
                bool        whole   = false; // the window opens mid-gap, so the first run is not one
                for (std::size_t i = 0UZ; i < tracked.tags.size(); ++i) {
                    if (i > 0UZ && tracked.tags[i].index != tracked.tags[i - 1UZ].index) {
                        if (whole) {
                            longest = std::max(longest, run);
                        }
                        whole = true;
                        run   = 0UZ;
                    }
                    ++run;
                }
                expect(gt(longest, 0UZ)) << "the window has to span several anchors for the gap to mean anything";
                expect(lt(longest, static_cast<std::size_t>(1.5 * kPeriod))) << "no anchor stands a period and a half past the one before it, tag at " << at << " " << fraction;
            }
        }
    };

    "the output is bit-identical and the same length under any chunking"_test = [] {
        constexpr std::size_t kSamples = 4000UZ;
        const std::vector<CF> input    = nyquistPsk(kSamples, 4.0, 0.35, true, 1.7, 21U);

        for (const auto& [name, gain, outputs] : std::array<std::tuple<std::string, double, gr::Size_t>, 3UZ>{{{"MuellerMuller", 0.0, 1U}, {"MuellerMuller", 0.0, 2U}, {"EarlyLate", 1.06743, 1U}}}) {
            const gr::property_map settings{{"samples_per_symbol", 4.0}, {"detector", name}, {"detector_gain", gain}, {"outputs_per_symbol", outputs}};

            SymbolSync<CF> whole     = make<CF>(settings);
            const auto     reference = drive<CF>(whole, std::span<const CF>(input));
            expect(gt(reference.samples.size(), 800UZ)) << name;

            for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 4096UZ}) {
                SymbolSync<CF> block   = make<CF>(settings);
                const auto     chunked = drive<CF>(block, std::span<const CF>(input), chunk, chunk);

                expect(eq(chunked.samples.size(), reference.samples.size())) << name << " at chunk " << chunk << ": the output count";
                expect(std::ranges::equal(chunked.samples, reference.samples)) << name << " at chunk " << chunk << ": bit-identical output";
                expect(std::ranges::equal(chunked.aux[0], reference.aux[0])) << name << " at chunk " << chunk << ": and the error port";
            }
        }
    };

    "an exact clock produces one output per symbol"_test = [] {
        constexpr std::size_t kSymbols = 2000UZ;
        const std::vector<CF> input    = nyquistPsk(kSymbols * 4UZ, 4.0, 0.35, true, 0.0, 22U);

        for (const gr::Size_t outputs : {gr::Size_t(1), gr::Size_t(2)}) {
            SymbolSync<CF>    block    = make<CF>({{"samples_per_symbol", 4.0}, {"outputs_per_symbol", outputs}});
            const std::size_t produced = drive<CF>(block, std::span<const CF>(input)).samples.size();
            const std::size_t wanted   = kSymbols * static_cast<std::size_t>(outputs);
            expect(le(wanted - produced, 2UZ * static_cast<std::size_t>(outputs))) << "produced " << produced << " against " << wanted;

            for (const std::size_t chunk : {1UZ, 17UZ, 4096UZ}) {
                SymbolSync<CF> chunkedBlock = make<CF>({{"samples_per_symbol", 4.0}, {"outputs_per_symbol", outputs}});
                expect(eq(drive<CF>(chunkedBlock, std::span<const CF>(input), chunk, chunk).samples.size(), produced)) << "the count is identical at chunk " << chunk;
            }
        }
    };

    "the loop settles on the true period with no steady-state error"_test = [] {
        constexpr std::size_t kSymbols = 6000UZ;

        for (const double truth : {4.0, 4.02, 3.98}) {
            const std::vector<CF> input = nyquistPsk(static_cast<std::size_t>(static_cast<double>(kSymbols) * truth), truth, 0.35, true, 0.0, 23U);

            SymbolSync<CF> block   = make<CF>({{"samples_per_symbol", 4.0}, {"noise_bandwidth", 0.01}, {"damping", 1.0}});
            const auto     tracked = drive<CF>(block, std::span<const CF>(input));
            expect(gt(tracked.samples.size(), 4800UZ)) << "true period " << truth;

            const double settled = meanOfTail(std::span<const float>(tracked.aux[2]), 2000UZ);
            expect(lt(std::abs(settled - truth) / truth, 5e-6)) << "settled " << settled << " against " << truth << ", " << 1e6 * std::abs(settled - truth) / truth << " ppm";

            const double evm = meanErrorVector(std::span<const CF>(tracked.samples), tracked.samples.size() - 2000UZ);
            expect(lt(evm, 0.005)) << "tail error vector " << evm << " at true period " << truth;
        }
    };

    "every detector closes the loop under its own sign convention"_test = [] {
        constexpr std::size_t kSymbols = 2500UZ;
        const std::vector<CF> input    = nyquistPsk(static_cast<std::size_t>(static_cast<double>(kSymbols) * 4.02), 4.02, 0.35, true, 1.1, 29U);

        constexpr std::array<std::pair<std::string_view, double>, 7UZ> kDetectors{{{"MuellerMuller", 1.77355}, {"ModifiedMuellerMuller", 3.54714}, {"ZeroCrossing", 2.60278}, {"Gardner", 1.06739}, {"EarlyLate", 1.06743}, {"SignalTimesSlopeMl", 1.0}, {"SignumTimesSlopeMl", 1.0}}};

        std::array<double, 7UZ> jitter{};
        for (std::size_t which = 0UZ; which < kDetectors.size(); ++which) {
            const auto& [name, gain] = kDetectors[which];

            SymbolSync<CF> block   = make<CF>({{"samples_per_symbol", 4.0}, {"detector", std::string(name)}, {"detector_gain", gain}, {"noise_bandwidth", 0.02}});
            const auto     tracked = drive<CF>(block, std::span<const CF>(input));
            expect(gt(tracked.samples.size(), 1800UZ)) << name;

            const double settled = meanOfTail(std::span<const float>(tracked.aux[2]), 2000UZ);
            expect(lt(std::abs(settled - 4.02) / 4.02, 2e-3)) << name << " settled at " << settled << " against 4.02";

            jitter[which] = meanErrorVector(std::span<const CF>(tracked.samples), tracked.samples.size() - 2000UZ);
            expect(lt(jitter[which], 0.1)) << name << " must leave a sliceable output, got " << jitter[which];
        }

        // Gardner and early-late are the same S-curve; what separates them is the noise about it, and early-late's is
        // 3.3 times larger. It shows up here as residual timing jitter and it is why the documentation prefers Gardner.
        expect(lt(jitter[3], jitter[4])) << "Gardner " << jitter[3] << " against early-late " << jitter[4];
        expect(lt(jitter[0], jitter[3])) << "and the decision-directed M&M is quieter than either";
    };

    "pam4 decisions drive the decision-directed loop on a real stream"_test = [] {
        constexpr std::size_t    kSymbols = 4000UZ;
        const std::vector<float> input    = nyquistPam4(static_cast<std::size_t>(static_cast<double>(kSymbols) * 4.02), 4.02, 0.35, 1.1, 31U);

        SymbolSync<float> block   = make<float>({{"samples_per_symbol", 4.0}, {"constellation", std::string("pam4")}, {"detector", std::string("ZeroCrossing")}, {"detector_gain", 2.60278}, {"noise_bandwidth", 0.02}});
        const auto        tracked = drive<float>(block, std::span<const float>(input));
        expect(gt(tracked.samples.size(), 3300UZ));

        const double settled = meanOfTail(std::span<const float>(tracked.aux[2]), 2000UZ);
        expect(lt(std::abs(settled - 4.02) / 4.02, 2e-3)) << "settled " << settled << " against 4.02";

        const double error = meanPamError(std::span<const float>(tracked.samples), tracked.samples.size() - 2000UZ);
        expect(lt(error, 0.1)) << "the four-level output slices, mean distance " << error;

        expect(throws([] { std::ignore = make<CF>({{"constellation", std::string("pam4")}}); })) << "pam4 refuses a complex stream";
        expect(throws([] { std::ignore = make<float>({{"constellation", std::string("qpsk")}}); })) << "qpsk still refuses a real one";
    };

    "the symbol instant comes first and the side ports repeat across the group"_test = [] {
        constexpr std::size_t kSymbols = 4000UZ;
        const std::vector<CF> input    = nyquistPsk(kSymbols * 4UZ, 4.0, 0.35, true, 1.4, 30U);

        SymbolSync<CF> block   = make<CF>({{"samples_per_symbol", 4.0}, {"outputs_per_symbol", gr::Size_t(2)}});
        const auto     tracked = drive<CF>(block, std::span<const CF>(input));
        expect(gt(tracked.samples.size(), 6000UZ));

        std::vector<CF> onInstant;
        std::vector<CF> between;
        for (std::size_t i = tracked.samples.size() - 2000UZ; i < tracked.samples.size(); ++i) {
            (i % 2UZ == 0UZ ? onInstant : between).push_back(tracked.samples[i]);
        }
        expect(lt(meanErrorVector(std::span<const CF>(onInstant), 0UZ), 0.01)) << "output 2k is the symbol instant and must be sliceable";
        expect(gt(meanErrorVector(std::span<const CF>(between), 0UZ), 0.1)) << "output 2k+1 is half a period away and must not be";

        for (std::size_t i = 2UZ; i + 2UZ < tracked.aux[0].size(); i += 2UZ) {
            expect(eq(tracked.aux[0][i], tracked.aux[0][i + 1UZ])) << "one error per symbol, repeated across the group at index " << i;
            expect(eq(tracked.aux[1][i], tracked.aux[1][i + 1UZ])) << "and one instantaneous period";
        }
    };

    "the optional ports may be left unwired"_test = [] {
        const std::vector<CF> input = nyquistPsk(4000UZ, 4.0, 0.35, true, 0.9, 31U);

        SymbolSync<CF> wired   = make<CF>({{"samples_per_symbol", 4.0}});
        const auto     withAll = drive<CF>(wired, std::span<const CF>(input));

        SymbolSync<CF> bare    = make<CF>({{"samples_per_symbol", 4.0}});
        const auto     minimal = test::runPortsVariable<3UZ, CF>(bare, std::span<const CF>(input), 0UZ, 0UZ, std::array<bool, 3UZ>{false, false, false});

        expect(std::ranges::equal(minimal.samples, withAll.samples)) << "an unwired side port changes nothing about the stream";
        expect(eq(minimal.aux[0].size(), 0UZ)) << "and nothing is published on it";
    };

    "acquisition from half a symbol takes about a thousand symbols"_test = [] {
        constexpr std::size_t kSymbols = 3000UZ;
        const std::vector<CF> input    = nyquistPsk(kSymbols * 4UZ, 4.0, 0.35, true, 2.0, 24U);

        SymbolSync<CF> block   = make<CF>({{"samples_per_symbol", 4.0}, {"noise_bandwidth", 0.01}, {"damping", 1.0}});
        const auto     tracked = drive<CF>(block, std::span<const CF>(input));
        expect(gt(tracked.samples.size(), 2500UZ));

        const double start = meanErrorVector(std::span<const CF>(tracked.samples).subspan(0UZ, 50UZ), 0UZ);
        expect(gt(start, 0.05)) << "half a symbol of timing error has to show up as one, got " << start;

        const std::span<const CF> settled = std::span<const CF>(tracked.samples).subspan(2000UZ, 50UZ);
        expect(lt(meanErrorVector(settled, 0UZ), 0.01)) << "and be gone within 2000 symbols, got " << meanErrorVector(settled, 0UZ);
    };

    "the blind detectors track through a carrier the decision-directed ones lose"_test = [] {
        constexpr std::size_t kSymbols = 3000UZ;
        const std::vector<CF> input    = nyquistPsk(kSymbols * 4UZ, 4.02, 0.35, true, 1.3, 25U);

        std::vector<CF> rotated(input.size());
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            rotated[i] = input[i] * static_cast<CF>(std::polar(1.0, 0.37 + 2.0 * kPi * 0.0009 * static_cast<double>(i)));
        }

        SymbolSync<CF> gardner = make<CF>({{"samples_per_symbol", 4.0}, {"detector", std::string("Gardner")}, {"detector_gain", 1.06739}, {"noise_bandwidth", 0.02}});
        const auto     blind   = drive<CF>(gardner, std::span<const CF>(rotated));
        const double   period  = meanOfTail(std::span<const float>(blind.aux[2]), 2000UZ);
        expect(lt(std::abs(period - 4.02) / 4.02, 2e-4)) << "a blind detector locks the clock whatever the carrier does, got " << period;

        SymbolSync<CF> decided = make<CF>({{"samples_per_symbol", 4.0}, {"noise_bandwidth", 0.02}});
        const auto     lost    = drive<CF>(decided, std::span<const CF>(rotated));
        expect(gt(std::abs(meanOfTail(std::span<const float>(lost.aux[2]), 2000UZ) - 4.02) / 4.02, 2e-4)) << "and a decision-directed one must not, or the documented distinction is meaningless";
    };

    "the polyphase path matched-filters and still tracks"_test = [] {
        constexpr std::size_t    kSymbols = 4000UZ;
        constexpr int            kSps     = 4;
        const std::vector<float> shaping  = gr::filter::fir::design::rootRaisedCosine(static_cast<std::size_t>(12 * kSps + 1), static_cast<double>(kSps), 0.35);

        std::mt19937                       rng(26U);
        std::uniform_int_distribution<int> pick(0, 3);
        std::vector<CD>                    shaped(kSymbols * static_cast<std::size_t>(kSps), CD{});
        for (std::size_t k = 0UZ; k + 16UZ < kSymbols; ++k) {
            const int choice = pick(rng);
            const CD  symbol{kRoot * ((choice & 1) ? 1.0 : -1.0), kRoot * ((choice & 2) ? 1.0 : -1.0)};
            for (std::size_t j = 0UZ; j < shaping.size(); ++j) {
                const std::size_t at = k * static_cast<std::size_t>(kSps) + j;
                if (at < shaped.size()) {
                    shaped[at] += symbol * static_cast<double>(shaping[j]);
                }
            }
        }
        std::vector<CF> input(shaped.size());
        for (std::size_t i = 0UZ; i < shaped.size(); ++i) {
            input[i] = static_cast<CF>(shaped[i]);
        }

        SymbolSync<CF> block = make<CF>({{"samples_per_symbol", 4.0}, {"interpolator", std::string("polyphase")}, {"rolloff", 0.35}, {"polyphase_arms", gr::Size_t(128)}, {"detector", std::string("Gardner")}, {"detector_gain", 1.06739}, {"noise_bandwidth", 0.02}});
        expect(eq(block.windowLength(), 44UZ)) << "ceil(11*sps) rounded up to even, which is 44 at sps 4 and not 11";
        expect(eq(block.branches().size(), 44UZ * 128UZ));

        const auto tracked = drive<CF>(block, std::span<const CF>(input));
        expect(gt(tracked.samples.size(), 3000UZ));
        expect(lt(std::abs(meanOfTail(std::span<const float>(tracked.aux[2]), 1000UZ) - 4.0), 0.01)) << "the clock is exact and the loop must say so";
    };

    "changing samples_per_symbol restarts the loop and leaves the output offsets continuous"_test = [] {
        const std::vector<CF> input = nyquistPsk(6000UZ, 4.0, 0.35, true, 0.0, 27U);

        SymbolSync<CF> block  = make<CF>({{"samples_per_symbol", 4.0}});
        const auto     first  = drive<CF>(block, std::span<const CF>(input).subspan(0UZ, 3000UZ));
        const double   before = block.loop().frequency();
        expect(gt(first.samples.size(), 600UZ));

        std::ignore = block.settings().setStaged({{"samples_per_symbol", 5.0}});
        std::ignore = block.settings().applyStagedParameters();
        expect(lt(std::abs(block.loop().frequency() - 5.0), 1e-12)) << "the tracked period restarts at the new nominal, was " << before;
        expect(eq(block._positioned, true)) << "and the stream position is not restarted";

        const auto second = drive<CF>(block, std::span<const CF>(input).subspan(3000UZ), 0UZ, 0UZ, {}, 3000UZ);
        expect(gt(second.samples.size(), 400UZ)) << "and the block keeps producing";
        expect(gt(block._base, 5000ULL)) << "from where it left off, not from zero";
    };

    "a real stream runs the same machinery"_test = [] {
        constexpr std::size_t kSymbols     = 3000UZ;
        const std::vector<CF> complexInput = nyquistPsk(kSymbols * 4UZ, 4.01, 0.35, false, 1.1, 28U);

        std::vector<float> input(complexInput.size());
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            input[i] = complexInput[i].real();
        }

        SymbolSync<float> block   = make<float>({{"samples_per_symbol", 4.0}, {"noise_bandwidth", 0.02}});
        const auto        tracked = drive<float>(block, std::span<const float>(input));
        expect(gt(tracked.samples.size(), 2500UZ));
        expect(lt(std::abs(meanOfTail(std::span<const float>(tracked.aux[2]), 1000UZ) - 4.01) / 4.01, 1e-4)) << "bpsk on a real stream locks the same way";
    };

    "the shared constellation spelling names the same slicer as this block's own"_test = [] {
        // The constellation-carrying blocks say `psk` with an `arity`; this one says `qpsk` or `bpsk`. Both are
        // accepted, and a name that resolves to the same shape has to behave identically rather than merely be
        // tolerated, so the two are driven over the same input and compared.
        const auto input = nyquistPsk(16384UZ, 4.0, 0.35, true, 0.3, 7U);

        SymbolSync<CF> native = make<CF>({{"samples_per_symbol", 4.0}, {"rolloff", 0.35}, {"constellation", std::string("qpsk")}});
        SymbolSync<CF> shared = make<CF>({{"samples_per_symbol", 4.0}, {"rolloff", 0.35}, {"constellation", std::string("psk")}, {"arity", gr::Size_t{4}}});

        const auto fromNative = drive<CF>(native, std::span<const CF>(input));
        const auto fromShared = drive<CF>(shared, std::span<const CF>(input));
        expect(gt(fromNative.samples.size(), 2000UZ)) << "the run produced something to compare";
        expect(std::ranges::equal(fromNative.samples, fromShared.samples)) << "psk with arity 4 is the qpsk slicer, so the output is the same sample for sample";

        // arity 2 is the binary decision a real stream carries, and a width the detectors cannot slice is refused
        // rather than sliced wrongly
        expect(nothrow([] { std::ignore = make<float>({{"constellation", std::string("psk")}, {"arity", gr::Size_t{2}}}); }));
        expect(throws([] { std::ignore = make<CF>({{"constellation", std::string("psk")}, {"arity", gr::Size_t{8}}}); })) << "8-PSK has no hard decision here";
        expect(throws([] { std::ignore = make<CF>({{"constellation", std::string("qam")}, {"arity", gr::Size_t{16}}}); })) << "the shared names this slicer cannot carry are refused, not ignored";
    };

    "under a scheduler the symbol count is the stream's, not the chunking's"_test = [] {
        // The chunking test above drives processBulk directly and so never meets the port minimum, which is what a
        // scheduler enforces: a minimum wider than the loop's own production condition refuses input the loop could
        // have used, and at the end of a stream the refused samples are gone. How many are left over there is set by
        // where the call boundaries fell, so the count would follow the scheduler. Every detector runs, because the
        // detectors differ in how far past the base they read and that distance is half the minimum.
        const auto through = []<typename T>(std::type_identity<T>, const gr::property_map& settings, const std::vector<T>& data, std::size_t fixed, std::uint64_t seed, std::size_t max) {
            gr::Graph flow;
            auto&     src = flow.emplaceBlock<FiniteSource<T>>();
            src._data     = data;
            auto& limit   = flow.emplaceBlock<ChunkLimiter<T>>();
            limit._fixed  = fixed;
            limit._state  = seed;
            limit._max    = max;
            auto& sync    = flow.emplaceBlock<SymbolSync<T>>(settings);
            auto& sink    = flow.emplaceBlock<ValueSink<T>>();
            expect(flow.connect<"out", "in">(src, limit).has_value());
            expect(flow.connect<"out", "in">(limit, sync).has_value());
            expect(flow.connect<"out", "in">(sync, sink).has_value());

            gr::scheduler::Simple<> scheduler;
            expect(scheduler.exchange(std::move(flow)).has_value()) << fatal;
            expect(scheduler.runAndWait().has_value()) << fatal;
            return sink._values;
        };

        const auto agrees = [&through]<typename T>(std::type_identity<T> typeTag, const gr::property_map& settings, const std::vector<T>& data, std::string_view arm) {
            const std::vector<T> reference = through(typeTag, settings, data, 0UZ, 1ULL, 1UZ << 20);
            expect(gt(reference.size(), 600UZ)) << arm;

            for (const auto& [label, fixed, seed, max] : std::array<std::tuple<std::string_view, std::size_t, std::uint64_t, std::size_t>, 2UZ>{{{"1", 1UZ, 1ULL, 0UZ}, {"seeded", 0UZ, 424242ULL, 97UZ}}}) {
                const std::vector<T> chunked = through(typeTag, settings, data, fixed, seed, max);
                expect(eq(chunked.size(), reference.size())) << arm << " at chunk " << label << ": the count is the stream's";
                expect(std::ranges::equal(chunked, reference)) << arm << " at chunk " << label << ": bit-identical output";
            }
        };

        const std::vector<CF> input = nyquistPsk(3000UZ, 4.0, 0.35, true, 1.7, 41U);
        for (const std::string_view name : {"MuellerMuller", "ModifiedMuellerMuller", "ZeroCrossing", "Gardner", "EarlyLate", "SignalTimesSlopeMl", "SignumTimesSlopeMl"}) {
            const std::string arm = std::format("mmse8 {}", name);
            agrees(std::type_identity<CF>{}, {{"samples_per_symbol", 4.0}, {"detector", std::string(name)}, {"interpolator", std::string("mmse8")}}, input, arm);
        }

        // The minimum carries a `_polyphase ? 1 : 0` term, so the polyphase interpolator -- whose held window is
        // symbols rather than taps long -- runs too. It has no interpolating differentiator, so the two
        // maximum-likelihood detectors are refused there and are not offered to it. Early-late is the sharpest of
        // the five here: its reach past the base moves with the loop's period, so a minimum that bounded every
        // period the clamp allows would stand two samples clear of the reach and cost the last symbol of a
        // chunked stream.
        for (const std::string_view name : {"MuellerMuller", "ModifiedMuellerMuller", "ZeroCrossing", "Gardner", "EarlyLate"}) {
            const std::string arm = std::format("polyphase {}", name);
            agrees(std::type_identity<CF>{}, {{"samples_per_symbol", 4.0}, {"detector", std::string(name)}, {"interpolator", std::string("polyphase")}, {"detector_gain", 1.0}}, input, arm);
        }

        // and the real sample type, which is what a post-detection chain reads
        const std::vector<float> pam = nyquistPam4(3000UZ, 4.0, 0.35, 1.1, 43U);
        agrees(std::type_identity<float>{}, {{"samples_per_symbol", 4.0}, {"constellation", std::string("pam4")}, {"detector", std::string("ZeroCrossing")}, {"detector_gain", 2.60278}, {"noise_bandwidth", 0.02}}, pam, "pam4 real");
    };

    "a finite graph terminates, real and complex, all interpolators"_test = [] {
        // A source that ends publishes an end-of-stream marker; the block ends once fewer samples than its
        // input minimum remain before it. A block that instead waits for its held-back window to grow parks
        // the whole scheduler forever, so the assertion here is completion itself, on the scheduler's own
        // thread with a deadline, and the sample count only confirms the stream actually flowed.
        const auto finishes = []<typename T>(std::type_identity<T>, gr::property_map settings, std::vector<T> data) {
            gr::Graph flow;
            auto&     src = flow.emplaceBlock<FiniteSource<T>>();
            src._data     = std::move(data);
            auto& sync    = flow.emplaceBlock<SymbolSync<T>>(std::move(settings));
            auto& sink    = flow.emplaceBlock<CountSink<T>>();
            expect(flow.connect<"out", "in">(src, sync).has_value());
            expect(flow.connect<"out", "in">(sync, sink).has_value());

            gr::scheduler::Simple<> scheduler;
            expect(scheduler.exchange(std::move(flow)).has_value());
            std::atomic<bool> done{false};
            std::thread       runner([&scheduler, &done] {
                std::ignore = scheduler.runAndWait();
                done        = true;
            });
            const auto        start = std::chrono::steady_clock::now();
            while (!done.load() && std::chrono::steady_clock::now() - start < std::chrono::seconds(30)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            const bool finished = done.load();
            if (!finished) {
                scheduler.requestStop();
            }
            runner.join();
            return finished ? sink._count.load() : 0UZ;
        };

        std::vector<float> pam(4000UZ * 5UZ);
        std::mt19937       rng(99);
        for (std::size_t i = 0UZ; i < pam.size(); i += 5UZ) {
            const float level = 0.4472136f * static_cast<float>(2 * static_cast<int>(rng() & 3U) - 3);
            std::fill_n(pam.begin() + static_cast<std::ptrdiff_t>(i), 5UZ, level);
        }
        std::vector<CF> psk(2000UZ * 4UZ);
        for (std::size_t i = 0UZ; i < psk.size(); i += 4UZ) {
            std::fill_n(psk.begin() + static_cast<std::ptrdiff_t>(i), 4UZ, CF{(rng() & 1U) != 0U ? 1.f : -1.f, 0.f});
        }

        const std::size_t pamOut = finishes(std::type_identity<float>{}, {{"samples_per_symbol", 5.0}, {"constellation", std::string("pam4")}, {"detector", std::string("ZeroCrossing")}, {"detector_gain", 2.60278}, {"noise_bandwidth", 0.02}}, pam);
        expect(gt(pamOut, 3900UZ)) << "the pam4 zero-crossing graph ran to completion";

        const std::size_t pskOut = finishes(std::type_identity<CF>{}, {{"samples_per_symbol", 4.0}, {"noise_bandwidth", 0.02}}, psk);
        expect(gt(pskOut, 1900UZ)) << "the complex default-detector graph ran to completion";

        const std::size_t polyOut = finishes(std::type_identity<CF>{}, {{"samples_per_symbol", 4.0}, {"detector", std::string("Gardner")}, {"detector_gain", 1.0}, {"interpolator", std::string("polyphase")}, {"noise_bandwidth", 0.02}}, psk);
        expect(gt(polyOut, 1800UZ)) << "the polyphase graph, whose held window is symbols long, still ends";
    };
};

int main() { /* tests are statically registered */ }
