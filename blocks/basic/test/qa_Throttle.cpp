#include <boost/ut.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <limits>
#include <numeric>
#include <span>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/basic/Throttle.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::basic::Throttle;
namespace spans = gr::blocks::testing::span;

using Clock = std::chrono::steady_clock;

constexpr float  kRate    = 100000.f;
constexpr double kFullSet = 50000.0 / 100000.0; // half a second of paced samples

template<typename T>
[[nodiscard]] Throttle<T> makeThrottle(gr::property_map settings) {
    Throttle<T> block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

struct Pump {
    std::size_t consumed     = 0UZ;
    std::size_t produced     = 0UZ;
    std::size_t largestChunk = 0UZ;
    std::size_t calls        = 0UZ;
};

template<typename T, typename FHook>
Pump pump(Throttle<T>& block, std::span<const T> input, std::span<T> output, std::size_t chunkSize, bool connected, FHook&& perCall) {
    Pump result;
    while (result.consumed < input.size()) {
        const std::size_t    count = std::min(chunkSize, input.size() - result.consumed);
        spans::InputSpan<T>  inSpan{input.subspan(result.consumed, count), result.consumed};
        spans::OutputSpan<T> outSpan{output.subspan(result.produced, count), result.produced};
        outSpan.isConnected = connected;
        std::ignore         = block.processBulk(inSpan, outSpan);

        result.consumed += inSpan.consumed;
        result.produced += outSpan.count;
        result.largestChunk = std::max(result.largestChunk, inSpan.consumed);
        ++result.calls;
        perCall(result.calls);
        if (inSpan.consumed == 0UZ) {
            break;
        }
    }
    return result;
}

template<typename T>
Pump pump(Throttle<T>& block, std::span<const T> input, std::span<T> output, std::size_t chunkSize, bool connected = true) {
    return pump(block, input, output, chunkSize, connected, [](std::size_t) {});
}

template<typename T>
[[nodiscard]] std::vector<T> ramp(std::size_t nSamples) {
    std::vector<T> input(nSamples);
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        input[i] = static_cast<T>(i % 251UZ);
    }
    return input;
}

template<typename T, typename FBody>
[[nodiscard]] double timed(FBody&& body) {
    const auto start = Clock::now();
    body();
    return std::chrono::duration<double>(Clock::now() - start).count();
}

} // namespace

const boost::ut::suite<"Throttle"> throttleTests = [] {
    using namespace boost::ut;

    "every sample comes through untouched"_test = []<typename T>() {
        Throttle<T>          block = makeThrottle<T>({{"sample_rate", 1e9f}});
        const std::vector<T> input = ramp<T>(4096UZ);
        std::vector<T>       output(input.size());
        block.restart();

        const Pump result = pump<T>(block, std::span<const T>(input), std::span<T>(output), 333UZ);
        expect(eq(result.consumed, input.size()));
        expect(eq(result.produced, input.size())) << "connected, the block is exactly 1:1";
        expect(std::ranges::equal(output, input)) << "and copies";
    } | std::tuple<float, std::int16_t, std::uint8_t>{};

    "a complex stream comes through untouched too"_test = [] {
        using CF              = std::complex<float>;
        Throttle<CF>    block = makeThrottle<CF>({{"sample_rate", 1e9f}});
        std::vector<CF> input(2048UZ);
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            input[i] = CF{static_cast<float>(i), static_cast<float>(-static_cast<double>(i))};
        }
        std::vector<CF> output(input.size());
        block.restart();

        expect(eq(pump<CF>(block, std::span<const CF>(input), std::span<CF>(output), 512UZ).produced, input.size()));
        expect(std::ranges::equal(output, input));
    };

    "tags arrive at the offsets they left at"_test = [] {
        gr::Graph graph;
        auto&     source = graph.emplaceBlock<gr::blocks::testing::TagSource<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 2048U}, {"mark_tag", false}});
        for (const std::size_t at : {7UZ, 64UZ, 1000UZ}) {
            source._tags.emplace_back(at, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::format("probe{}", at)}});
        }
        // a key outside the auto-forward set, which the block's pass-all policy carries all the same
        source._tags.emplace_back(300UZ, gr::property_map{{gr::property_map::key_type{"private_key"}, gr::pmt::Value(std::string("carried"))}});
        std::ranges::sort(source._tags, std::ranges::less{}, &gr::Tag::index);

        auto& throttle = graph.emplaceBlock<Throttle<float>>({{"sample_rate", 1e9f}});
        auto& sink     = graph.emplaceBlock<gr::blocks::testing::TagSink<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

        expect(graph.connect<"out", "in">(source, throttle).has_value());
        expect(graph.connect<"out", "in">(throttle, sink).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        const gr::property_map::key_type probe{gr::tag::TRIGGER_NAME.shortKey()};
        const gr::property_map::key_type priv{"private_key"};
        std::vector<std::size_t>         seen;
        std::vector<std::size_t>         seenPrivate;
        for (const gr::Tag& seenTag : sink._tags) {
            if (seenTag.map.contains(probe)) {
                seen.push_back(seenTag.index);
            }
            if (const auto found = seenTag.map.find(priv); found != seenTag.map.end() && found->second == gr::pmt::Value(std::string("carried"))) {
                seenPrivate.push_back(seenTag.index);
            }
        }
        expect(that % (seen == std::vector<std::size_t>{7UZ, 64UZ, 1000UZ})) << "a 1:1 block moves no tag";
        expect(that % (seenPrivate == std::vector<std::size_t>{300UZ})) << "and a non-reserved key arrives once, at its own offset";
    };

    "the long-run average rate is the configured one"_test = [] {
        Throttle<float>          block = makeThrottle<float>({{"sample_rate", kRate}});
        const std::vector<float> input(50000UZ, 0.f);
        std::vector<float>       output(input.size());
        block.restart();

        const double elapsed = timed<float>([&] { std::ignore = pump<float>(block, std::span<const float>(input), std::span<float>(output), 4096UZ); });
        expect(ge(elapsed, 0.9 * kFullSet)) << "measured " << elapsed << " s against " << kFullSet;
    };

    "no chunk size releases a sample before its own deadline"_test = [] {
        constexpr double         kPaced = 20000.0 / static_cast<double>(kRate);
        const std::vector<float> input(20000UZ, 0.f);
        std::vector<float>       output(input.size());

        for (const std::size_t chunkSize : {256UZ, 8192UZ}) {
            Throttle<float> block = makeThrottle<float>({{"sample_rate", kRate}});
            block.restart();
            const double elapsed = timed<float>([&] { std::ignore = pump<float>(block, std::span<const float>(input), std::span<float>(output), chunkSize); });
            // the deadline is the block's own clock read, so a run that is not early is a property of the block and
            // not of the machine; how much later than the deadline it lands is the machine's and is not asserted
            expect(ge(elapsed, 0.9 * kPaced)) << "chunks of " << chunkSize << " took " << elapsed << " s against " << kPaced;
        }
    };

    "a stall does not release the backlog early"_test = [] {
        constexpr double         kStall = 0.15;
        constexpr double         kPaced = 20000.0 / static_cast<double>(kRate);
        const std::vector<float> input(20000UZ, 0.f);
        std::vector<float>       output(input.size());

        Throttle<float> stalled = makeThrottle<float>({{"sample_rate", kRate}});
        stalled.restart();
        Pump         result;
        const double interrupted = timed<float>([&] {
            result = pump<float>(stalled, std::span<const float>(input), std::span<float>(output), 1000UZ, true, [kStall](std::size_t call) {
                if (call == 10UZ) {
                    std::this_thread::sleep_for(std::chrono::duration<double>(kStall));
                }
            });
        });

        expect(eq(result.consumed, input.size())) << "a stall costs nothing but time";
        expect(ge(interrupted, 0.9 * kPaced)) << "and the samples it made overdue still wait for their own deadlines: " << interrupted << " s";
    };

    "max_items_per_chunk bounds what one invocation handles"_test = [] {
        Throttle<float>          block = makeThrottle<float>({{"sample_rate", 1e9f}, {"max_items_per_chunk", 64U}});
        const std::vector<float> input(2000UZ, 0.f);
        std::vector<float>       output(input.size());
        block.restart();

        const Pump result = pump<float>(block, std::span<const float>(input), std::span<float>(output), 1000UZ);
        expect(eq(result.largestChunk, 64UZ)) << "no invocation may take more than the cap";
        expect(eq(result.consumed, input.size())) << "and a short chunk is still handled rather than waited for";
    };

    "a rate change restarts the schedule"_test = [] {
        Throttle<float>          block = makeThrottle<float>({{"sample_rate", 10000.f}});
        const std::vector<float> input(10000UZ, 0.f);
        std::vector<float>       output(input.size());
        block.restart();

        std::ignore = pump<float>(block, std::span<const float>(input).first(1000UZ), std::span<float>(output), 1000UZ);
        expect(eq(block.paced(), 1000ULL));

        std::ignore = block.settings().setStaged({{"sample_rate", kRate}});
        std::ignore = block.settings().applyStagedParameters();
        expect(eq(block.paced(), 0ULL)) << "the origin and the count both move to the moment of the change";

        const double elapsed = timed<float>([&] { std::ignore = pump<float>(block, std::span<const float>(input), std::span<float>(output), 4096UZ); });
        expect(ge(elapsed, 0.07)) << "the second phase must be paced at the new rate, not released instantly: " << elapsed << " s";
    };

    "an unconnected output still paces the consumption"_test = [] {
        Throttle<float>          block = makeThrottle<float>({{"sample_rate", kRate}});
        const std::vector<float> input(20000UZ, 0.f);
        std::vector<float>       output(input.size());
        block.restart();

        Pump         result;
        const double elapsed = timed<float>([&] { result = pump<float>(block, std::span<const float>(input), std::span<float>(output), 4096UZ, false); });
        expect(eq(result.consumed, input.size()));
        expect(eq(result.produced, 0UZ)) << "nothing is published when nothing is listening";
        expect(ge(elapsed, 0.9 * 0.2)) << "and the consumption is paced all the same: " << elapsed << " s";
    };

    "a stop reaches a sleeping block rather than waiting out its deadline"_test = [] {
        Throttle<float>          block = makeThrottle<float>({{"sample_rate", 1.f}, {"max_sleep_s", 0.05}});
        const std::vector<float> input(100UZ, 0.f); // a hundred seconds of deadline, slept in slices of one twentieth
        std::vector<float>       output(input.size());
        block.restart();

        std::atomic<bool> returned{false};
        std::jthread      pumping([&] {
            std::ignore = pump<float>(block, std::span<const float>(input), std::span<float>(output), 100UZ);
            returned.store(true, std::memory_order_release);
        });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        block.requestStop();

        // The bound is a hang guard and nothing else: a block that takes the stop returns at once, and one that slept
        // through it would hold the call for the hundred seconds of its own deadline. Nothing is read from how far
        // below the bound the return lands.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
        while (!returned.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        expect(returned.load(std::memory_order_acquire)) << "the call returns on the stop rather than on the last sample's deadline";
    };

    "a stop releases only the samples that are due by then"_test = [] {
        constexpr double         kRateHz = 1.0;
        Throttle<float>          block   = makeThrottle<float>({{"sample_rate", static_cast<float>(kRateHz)}, {"max_sleep_s", 0.05}});
        const std::vector<float> input(100UZ, 0.f); // a hundred seconds of deadline offered in one chunk
        std::vector<float>       output(input.size());
        block.restart();
        const auto started = Clock::now();

        Pump              result;
        std::atomic<bool> returned{false};
        {
            std::jthread pumping([&] {
                result = pump<float>(block, std::span<const float>(input), std::span<float>(output), 100UZ);
                returned.store(true, std::memory_order_release);
            });
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            block.requestStop();

            const auto guard = Clock::now() + std::chrono::seconds(30);
            while (!returned.load(std::memory_order_acquire) && Clock::now() < guard) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            expect(returned.load(std::memory_order_acquire)) << "the call returns on the stop";
        }

        // the bound is what the whole call could have made due, however long the machine took over it: at one sample
        // per second and a stop fifty milliseconds in, that is none of them
        const double      held      = std::chrono::duration<double>(Clock::now() - started).count();
        const std::size_t dueByThen = static_cast<std::size_t>(std::floor(held * kRateHz));
        expect(le(result.consumed, dueByThen)) << "consumed " << result.consumed << " of 100 in " << held << " s";
        expect(eq(result.produced, result.consumed)) << "and published exactly what it consumed";
        expect(lt(result.consumed, input.size())) << "the rest of the chunk stays unconsumed";
    };

    "unusable parameters are rejected at settings time"_test = [] {
        expect(throws([] { std::ignore = makeThrottle<float>({{"sample_rate", 0.f}}); })) << "zero rate";
        expect(throws([] { std::ignore = makeThrottle<float>({{"sample_rate", -1.f}}); })) << "negative rate";
        expect(throws([] { std::ignore = makeThrottle<float>({{"sample_rate", std::numeric_limits<float>::quiet_NaN()}}); })) << "NaN rate";
        expect(throws([] { std::ignore = makeThrottle<float>({{"sample_rate", std::numeric_limits<float>::infinity()}}); })) << "infinite rate";
        expect(throws([] { std::ignore = makeThrottle<float>({{"max_sleep_s", 0.0}}); })) << "zero slice";
        expect(throws([] { std::ignore = makeThrottle<float>({{"max_sleep_s", -1.0}}); })) << "negative slice";
    };
};

int main() { /* tests are automatically registered and run */ }
