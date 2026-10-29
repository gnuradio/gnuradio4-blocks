#include <boost/ut.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <limits>
#include <memory>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/digital/Constellation.hpp>
#include <gnuradio-4.0/algorithm/timing/FrequencySchedule.hpp>
#include <gnuradio-4.0/channel/Awgn.hpp>
#include <gnuradio-4.0/channel/DopplerShift.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace {

using gr::blocks::channel::AwgnChannel;
using gr::blocks::channel::DopplerShift;
using gr::blocks::channel::SchedulePosition;

using C                 = std::complex<float>;
constexpr double kTwoPi = 2. * std::numbers::pi_v<double>;

/// A staged and started block, held by pointer because a block carrying a measurement slot is pinned where it is built.
template<typename TBlock>
[[nodiscard]] std::unique_ptr<TBlock> configured(gr::property_map settings) {
    auto block = std::make_unique<TBlock>(std::move(settings));
    block->settings().init();
    std::ignore = block->settings().applyStagedParameters();
    block->start();
    return block;
}

/// @brief Instantaneous frequency in Hz over a window of @p width steps centered on sample @p center.
///
/// The window is symmetric. A linear frequency ramp puts its phase steps symmetrically about the center step. The
/// argument of the vector sum is then that center step exactly. The estimate therefore measures the phasor's own
/// error.
[[nodiscard]] double instantaneousFrequency(std::span<const C> x, std::size_t center, std::size_t width, double fs) {
    const std::size_t    first = center - width / 2UZ;
    std::complex<double> accumulated{0., 0.};
    for (std::size_t k = first; k < first + width; ++k) {
        accumulated += std::complex<double>(static_cast<double>(x[k + 1UZ].real()), static_cast<double>(x[k + 1UZ].imag())) * std::conj(std::complex<double>(static_cast<double>(x[k].real()), static_cast<double>(x[k].imag())));
    }
    return std::arg(accumulated) / kTwoPi * fs;
}

template<typename TBlock>
[[nodiscard]] std::vector<C> runChunked(TBlock& block, std::span<const C> input, std::size_t chunk) {
    std::vector<C> out(input.size());
    for (std::size_t i = 0UZ; i < input.size(); i += chunk) {
        const std::size_t n = std::min(chunk, input.size() - i);
        std::ignore         = block.processBulk(input.subspan(i, n), std::span<C>(out.data() + i, n));
    }
    return out;
}

[[nodiscard]] std::vector<C> ones(std::size_t n) { return std::vector<C>(n, C(1.f, 0.f)); }

/// @brief Applies @p change to a running block and reports a refusal by the block or by the limits.
template<typename TBlock>
[[nodiscard]] bool refusedLive(TBlock& block, gr::property_map change) {
    std::ignore = block.settings().setStaged(std::move(change));
    try {
        return !block.settings().applyStagedParameters().failedParameters.empty();
    } catch (const std::exception&) {
        return true;
    }
}

/// @brief Applies @p change to a running block. Returns the block's refusal, empty when the block takes the change.
template<typename TBlock>
[[nodiscard]] std::string refusalLive(TBlock& block, gr::property_map change) {
    std::ignore = block.settings().setStaged(std::move(change));
    try {
        std::ignore = block.settings().applyStagedParameters();
    } catch (const gr::exception& refusal) {
        return refusal.message;
    }
    return {};
}

/**
 * @brief The rehearsal profile, a ten-minute LEO pass at 437 MHz with three knots per minute.
 *
 * A pass has an S-curve shape. The offset is high and nearly flat while the satellite closes, steep through closest
 * approach, and low and flat as it recedes. The curve is `-A*tanh((t - T/2)/tau)` with `A` = 10 kHz and `tau` =
 * 60 s. The ends sit within a hertz of `+/-A`. The slope through the center is `A/tau` = 167 Hz/s. The sign follows
 * the convention `gr::timing::offsetFor` pins, where closing reads high.
 */
struct Pass {
    std::vector<std::int64_t> times{};
    std::vector<double>       offsets{};
};

[[nodiscard]] Pass leoPass(double seconds = 600., double amplitudeHz = 10'000., double tauSeconds = 60.) {
    Pass         pass;
    const int    knots = static_cast<int>(std::llround(seconds / 20.)) + 1; // three knots a minute
    const double half  = 0.5 * seconds;
    for (int k = 0; k <= knots; ++k) {
        const double t = seconds * static_cast<double>(k) / static_cast<double>(knots);
        pass.times.push_back(static_cast<std::int64_t>(std::llround(t * 1e9)));
        pass.offsets.push_back(-amplitudeHz * std::tanh((t - half) / tauSeconds));
    }
    return pass;
}

struct Bits {
    std::uint64_t state = 0x9E3779B97F4A7C15ULL;

    [[nodiscard]] std::uint8_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return static_cast<std::uint8_t>(state & 1ULL);
    }
};

} // namespace

const boost::ut::suite<"doppler shift"> dopplerShiftTests = [] {
    using namespace boost::ut;

    // The ramp is answered at its knots, between them, and beyond its ends
    "a two-knot ramp reads the interpolant everywhere"_test = [] {
        constexpr double      fs       = 48'000.;
        constexpr std::size_t nSamples = 96'000UZ; // two seconds
        constexpr double      kStart   = 1'000.;
        constexpr double      kEnd     = 5'000.;

        const std::vector<std::int64_t> times{0LL, 1'000'000'000LL};
        const std::vector<double>       offsets{kStart, kEnd};

        auto       block = configured<DopplerShift<C>>({{"sample_rate", static_cast<float>(fs)}, {"schedule_times_ns", times}, {"schedule_offsets_hz", offsets}, {"direction", std::string("apply")}});
        const auto input = ones(nSamples);
        const auto out   = runChunked(*block, std::span<const C>(input), nSamples);

        constexpr std::size_t kWindow = 4096UZ;
        struct Probe {
            std::size_t center;
            double      wanted;
            const char* what;
        };
        const Probe probes[] = {
            {24'000UZ, 3'000., "the midpoint is the linear interpolant"},
            {12'000UZ, 2'000., "a quarter along"},
            {36'000UZ, 4'000., "three quarters along"},
            {72'000UZ, kEnd, "past the last knot the end value holds"},
        };

        double worst = 0.;
        for (const Probe& probe : probes) {
            const double measured = instantaneousFrequency(std::span<const C>(out), probe.center, kWindow, fs);
            worst                 = std::max(worst, std::abs(measured - probe.wanted));
            expect(lt(std::abs(measured - probe.wanted), 1e-3)) << std::format("{}: measured {:.9f} Hz, wanted {:.9f}", probe.what, measured, probe.wanted);
        }
        std::println("[doppler] two-knot ramp: worst discriminator disagreement {:.3e} Hz over a {} sample window at {} Hz", worst, kWindow, fs);

        // the knots themselves, read through the block's own observable and not the discriminator
        auto atStart = configured<DopplerShift<C>>({{"sample_rate", static_cast<float>(fs)}, {"schedule_times_ns", times}, {"schedule_offsets_hz", offsets}});
        expect(that % (atStart->currentOffsetHz() == kStart)) << "at the first knot the first value, exactly";
        expect(atStart->schedulePosition() == SchedulePosition::Inside);

        auto beforeStart = configured<DopplerShift<C>>({{"sample_rate", static_cast<float>(fs)}, {"schedule_times_ns", times}, {"schedule_offsets_hz", offsets}, {"anchor_ns", std::int64_t{-500'000'000}}});
        expect(beforeStart->schedulePosition() == SchedulePosition::Before);
        expect(that % (beforeStart->currentOffsetHz() == kStart)) << "before the table the first value holds, never an extrapolation";

        auto afterEnd = configured<DopplerShift<C>>({{"sample_rate", static_cast<float>(fs)}, {"schedule_times_ns", times}, {"schedule_offsets_hz", offsets}, {"anchor_ns", std::int64_t{2'000'000'000}}});
        expect(afterEnd->schedulePosition() == SchedulePosition::After);
        expect(that % (afterEnd->currentOffsetHz() == kEnd)) << "after the table the last value holds";
    };

    // The two seats are each other's inverse
    "apply then correct with the same table returns the signal"_test = [] {
        constexpr double      fs       = 48'000.;
        constexpr std::size_t nSamples = 200'000UZ;

        const Pass pass = leoPass(4., 8'000., 1.);

        std::vector<C> input(nSamples);
        Bits           bits;
        for (std::size_t k = 0UZ; k < nSamples; ++k) {
            input[k] = C(bits.next() != 0U ? 1.f : -1.f, bits.next() != 0U ? 1.f : -1.f) * 0.70710678f;
        }

        auto applied   = configured<DopplerShift<C>>({{"sample_rate", static_cast<float>(fs)}, {"schedule_times_ns", pass.times}, {"schedule_offsets_hz", pass.offsets}, {"direction", std::string("apply")}});
        auto corrected = configured<DopplerShift<C>>({{"sample_rate", static_cast<float>(fs)}, {"schedule_times_ns", pass.times}, {"schedule_offsets_hz", pass.offsets}, {"direction", std::string("correct")}});

        const auto shifted  = runChunked(*applied, std::span<const C>(input), 8192UZ);
        const auto restored = runChunked(*corrected, std::span<const C>(shifted), 8192UZ);

        double worst = 0.;
        for (std::size_t k = 0UZ; k < nSamples; ++k) {
            worst = std::max(worst, static_cast<double>(std::abs(restored[k] - input[k])));
        }
        std::println("[doppler] apply then correct: worst round-trip error {:.3e} on unit-magnitude samples over {} samples", worst, nSamples);
        // two float phasor multiplies plus one float cos/sin each. The bound is a few ULP of one, not of the run.
        expect(lt(worst, 4e-6)) << std::format("worst {:.3e}", worst);
    };

    // An offset that would alias is refused, and the knot is named
    "an offset at or past half the sample rate is refused naming the knot"_test = [] {
        constexpr double fs = 48'000.;

        const std::vector<std::int64_t> times{0LL, 1'000'000'000LL, 2'000'000'000LL};
        const std::vector<double>       offsets{1'000., 24'000., -500.}; // knot 1 sits exactly on fs/2

        bool        threw = false;
        std::string what;
        try {
            std::ignore = configured<DopplerShift<C>>({{"sample_rate", static_cast<float>(fs)}, {"schedule_times_ns", times}, {"schedule_offsets_hz", offsets}});
        } catch (const std::exception& error) {
            threw = true;
            what  = error.what();
        }
        expect(threw) << "an aliased shift is refused rather than wrapped";
        expect(what.contains("knot 1")) << std::format("the message names the offending knot: {}", what);

        // one hertz inside the limit is a shift, not an alias
        const std::vector<double> inside{1'000., 23'999., -500.};
        expect(nothrow([&] { std::ignore = configured<DopplerShift<C>>({{"sample_rate", static_cast<float>(fs)}, {"schedule_times_ns", times}, {"schedule_offsets_hz", inside}}); }));

        // and the kernel's own refusals still name their knot through the block
        const std::vector<std::int64_t> backwards{0LL, 2'000'000'000LL, 1'000'000'000LL};
        expect(throws([&] { std::ignore = configured<DopplerShift<C>>({{"sample_rate", static_cast<float>(fs)}, {"schedule_times_ns", backwards}, {"schedule_offsets_hz", inside}}); }));
        expect(throws([&] { std::ignore = configured<DopplerShift<C>>({{"sample_rate", static_cast<float>(fs)}, {"schedule_times_ns", times}, {"schedule_offsets_hz", std::vector<double>{1., 2.}}}); }));
        expect(throws([&] { std::ignore = configured<DopplerShift<C>>({{"sample_rate", static_cast<float>(fs)}, {"schedule_times_ns", times}, {"schedule_offsets_hz", offsets}, {"direction", std::string("undo")}}); }));
    };

    // A refused rate leaves the output of a running block as it was
    "a refused change leaves the shift on the values it accepted"_test = [] {
        constexpr float       fs = 1'000.f;
        constexpr std::size_t n  = 1'000UZ;

        const gr::property_map table{{"sample_rate", fs}, {"schedule_times_ns", std::vector<std::int64_t>{0LL, 100'000'000'000LL}}, {"schedule_offsets_hz", std::vector<double>{100., 100.}}};
        const auto             input     = ones(2UZ * n);
        auto                   unchanged = configured<DopplerShift<C>>(table);
        const std::vector<C>   reference = runChunked(*unchanged, std::span<const C>(input), n);

        // 150 Hz puts the 100 Hz knot past half the rate. The table check refuses the pair.
        auto           block = configured<DopplerShift<C>>(table);
        std::vector<C> out(2UZ * n);
        std::ignore = block->processBulk(std::span<const C>(input).first(n), std::span<C>(out).first(n));
        expect(refusedLive(*block, {{"sample_rate", 150.f}})) << "a knot at or past half the new rate refuses the rate";
        std::ignore = block->processBulk(std::span<const C>(input).subspan(n), std::span<C>(out).subspan(n));
        expect(std::ranges::equal(out, reference)) << "the refused rate never reaches the samples";
        expect(that % (block->currentOffsetHz() == unchanged->currentOffsetHz()));

        // A rate the time axis cannot carry is refused by the limits, and the member keeps the accepted rate.
        auto limited = configured<DopplerShift<C>>(table);
        for (const float refused : {0.f, -1.f, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN(), 1e30f}) {
            expect(refusedLive(*limited, {{"sample_rate", refused}})) << std::format("sample_rate {}", refused);
            expect(that % (limited->sample_rate.value == fs)) << std::format("sample_rate {}: the member keeps {} Hz", refused, fs);
        }
    };

    // A refused value stays in its member. Staging it again beside the change that fits it takes the value
    "a refused rate staged again beside a table that fits it reaches the samples"_test = [] {
        constexpr std::size_t n = 1'000UZ;

        auto           block = configured<DopplerShift<C>>({{"sample_rate", 1'000.f}, {"schedule_times_ns", std::vector<std::int64_t>{0LL, 100'000'000'000LL}}, {"schedule_offsets_hz", std::vector<double>{100., 100.}}});
        const auto     input = ones(n);
        std::vector<C> out(n);
        expect(refusedLive(*block, {{"sample_rate", 150.f}})) << "150 Hz puts the 100 Hz knot past half the rate";
        expect(that % (block->sample_rate.value == 150.f)) << "and the member keeps the refused value";
        expect(!refusedLive(*block, {{"sample_rate", 150.f}, {"schedule_offsets_hz", std::vector<double>{50., 50.}}})) << "the same rate beside a table that fits it";
        std::ignore = block->processBulk(std::span<const C>(input), std::span<C>(out));
        expect(that % (block->sample_rate.value == 150.f));
        expect(that % (std::abs(instantaneousFrequency(out, n / 2UZ, 64UZ, 150.) - 50.) < 1e-3)) << "the output is the 50 Hz knot at 150 Hz";
    };

    // A refused value held in its member refuses a change that names another setting
    "a held refused rate refuses an unrelated change until a valid rate replaces it"_test = [] {
        auto              block = configured<DopplerShift<C>>({{"sample_rate", 1'000.f}, {"schedule_times_ns", std::vector<std::int64_t>{0LL, 100'000'000'000LL}}, {"schedule_offsets_hz", std::vector<double>{100., 100.}}});
        const std::string held  = refusalLive(*block, {{"sample_rate", 150.f}});
        expect(fatal(!held.empty())) << "150 Hz puts the 100 Hz knot past half the rate";

        const std::string unrelated = refusalLive(*block, {{"anchor_ns", std::int64_t{7}}});
        expect(unrelated == held) << std::format("a new anchor time is refused with the held rate's message: {}", unrelated);
        expect(that % (block->anchor_ns.value == 7LL)) << "the member holds the new anchor time";
        expect(that % (block->anchorNs() == 0LL)) << "and the block keeps the accepted one";

        expect(refusalLive(*block, {{"sample_rate", 1'000.f}}).empty()) << "a valid rate is taken";
        expect(that % (block->anchorNs() == 7LL)) << "and the anchor time staged beside the held rate with it";
    };

    // The chunk boundaries cannot change a sample
    "the output is bit-identical however the stream is chunked"_test = [] {
        constexpr double      fs       = 48'000.;
        constexpr std::size_t nSamples = 60'000UZ;

        const Pass           pass      = leoPass(2., 6'000., 0.5);
        const auto           input     = ones(nSamples);
        auto                 whole     = configured<DopplerShift<C>>({{"sample_rate", static_cast<float>(fs)}, {"schedule_times_ns", pass.times}, {"schedule_offsets_hz", pass.offsets}});
        const std::vector<C> reference = runChunked(*whole, std::span<const C>(input), nSamples);

        for (const std::size_t chunk : {1UZ, 7UZ, 1000UZ, 12345UZ}) {
            auto       block  = configured<DopplerShift<C>>({{"sample_rate", static_cast<float>(fs)}, {"schedule_times_ns", pass.times}, {"schedule_offsets_hz", pass.offsets}});
            const auto result = runChunked(*block, std::span<const C>(input), chunk);
            expect(std::ranges::equal(result, reference)) << std::format("chunk {}: bit-identical, not near", chunk);
        }
    };

    // the observables belong to whatever thread polls them, not to the one that advances the stream
    "a reader on another thread sees whole values while the block runs"_test = [] {
        constexpr double      fs       = 48'000.;
        constexpr std::size_t kChunk   = 512UZ;
        constexpr std::size_t kChunks  = 400UZ;
        constexpr std::size_t kHalf    = kChunks / 2UZ;
        constexpr std::size_t nSamples = kChunk * kChunks;

        const Pass             pass    = leoPass(4., 8'000., 1.); // 4 s of table under 4.27 s of stream, so the end is held too
        const double           lowest  = *std::ranges::min_element(pass.offsets);
        const double           highest = *std::ranges::max_element(pass.offsets);
        const gr::property_map table{{"sample_rate", static_cast<float>(fs)}, {"schedule_times_ns", pass.times}, {"schedule_offsets_hz", pass.offsets}};
        const auto             input = ones(nSamples);

        // A block of the same table, driven straight through on one thread, gives the reference values. They are the
        // values at the sample the reader is held on and at the end of the stream. The cross-thread reads are
        // compared against that reference, not against themselves.
        auto alone          = configured<DopplerShift<C>>(table);
        std::ignore         = runChunked(*alone, std::span<const C>(input).first(kHalf * kChunk), kChunk);
        const double atHalf = alone->currentOffsetHz();
        std::ignore         = runChunked(*alone, std::span<const C>(input).subspan(kHalf * kChunk), kChunk);
        const double atEnd  = alone->currentOffsetHz();
        expect(that % (atHalf != atEnd)) << "the schedule must actually move over the run for a mid-stream read to say anything";

        auto                 reference = configured<DopplerShift<C>>(table);
        const std::vector<C> wanted    = runChunked(*reference, std::span<const C>(input), kChunk);

        auto           block = configured<DopplerShift<C>>(table);
        std::vector<C> output(nSamples);

        std::atomic<std::size_t> produced{0UZ};
        std::atomic<bool>        looked{false};
        std::atomic<bool>        finished{false};

        std::thread writer([&] {
            for (std::size_t k = 0UZ; k < kChunks; ++k) {
                std::ignore = block->processBulk(std::span<const C>(input.data() + k * kChunk, kChunk), std::span<C>(output.data() + k * kChunk, kChunk));
                produced.store(k + 1UZ, std::memory_order_release);
                while (k + 1UZ == kHalf && !looked.load(std::memory_order_acquire)) {
                    // One look is held open on a moving stream. The comparison below is then against a running
                    // block, not against the state the last call left.
                }
            }
            finished.store(true, std::memory_order_release);
        });

        std::uint64_t previous   = 0ULL;
        std::size_t   reads      = 0UZ;
        std::size_t   backwards  = 0UZ;
        std::size_t   outOfBand  = 0UZ;
        std::size_t   unplaced   = 0UZ;
        std::uint64_t heldAt     = 0ULL;
        double        heldOffset = 0.;
        for (;;) {
            const bool             done   = finished.load(std::memory_order_acquire);
            const double           offset = block->currentOffsetHz();
            const SchedulePosition where  = block->schedulePosition();
            const std::uint64_t    at     = block->position();
            ++reads;
            outOfBand += (std::isfinite(offset) && offset >= lowest && offset <= highest) ? 0UZ : 1UZ;
            unplaced += (where == SchedulePosition::Unarmed) ? 1UZ : 0UZ;
            backwards += (at >= previous && at <= nSamples) ? 0UZ : 1UZ;
            previous = at;

            if (!looked.load(std::memory_order_acquire) && produced.load(std::memory_order_acquire) >= kHalf) {
                heldAt     = block->position();
                heldOffset = block->currentOffsetHz();
                looked.store(true, std::memory_order_release);
            }
            if (done) { // read after the flag, so the last pass is on the finished stream
                break;
            }
        }
        writer.join();

        std::println("[doppler] observables: {} cross-thread reads over {} samples in {} calls", reads, nSamples, kChunks);
        expect(eq(outOfBand, 0UZ)) << "every offset read lies inside the table's own range, never half of one publish and half of the next";
        expect(eq(backwards, 0UZ)) << "the position only ever moves forward, and never past the stream";
        expect(eq(unplaced, 0UZ)) << "a hand-set anchor is armed from the first call, so no read is Unarmed";
        expect(that % (heldAt == kHalf * kChunk)) << "the held look landed on the sample the writer was parked at";
        expect(that % (heldOffset == atHalf)) << std::format("and read exactly what one thread reports there: {:.9f} against {:.9f}", heldOffset, atHalf);
        expect(that % (block->position() == nSamples));
        expect(that % (block->currentOffsetHz() == atEnd)) << "and the last publish carries the stream's last value";
        expect(std::ranges::equal(output, wanted)) << "publishing once a call leaves the sample path bit-identical";
    };

    "a frequency tag passes through untouched in both directions"_test = [] {
        using gr::blocks::testing::ProcessFunction;
        using gr::blocks::testing::TagSink;
        using gr::blocks::testing::TagSource;

        constexpr double kTuned = 437'000'000.;
        const Pass       pass   = leoPass(2., 6'000., 0.5);

        for (const std::string& seat : {std::string("apply"), std::string("correct")}) {
            gr::Graph graph;
            auto&     source = graph.emplaceBlock<TagSource<C, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", gr::Size_t(4096)}, {"mark_tag", false}});
            source._tags.emplace_back(0UZ, gr::property_map{{gr::property_map::key_type{gr::tag::FREQUENCY.shortKey()}, gr::pmt::Value(kTuned)}});
            auto& block = graph.emplaceBlock<DopplerShift<C>>({{"sample_rate", 48'000.f}, {"schedule_times_ns", pass.times}, {"schedule_offsets_hz", pass.offsets}, {"direction", seat}});
            auto& sink  = graph.emplaceBlock<TagSink<C, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

            expect(graph.connect<"out", "in">(source, block).has_value());
            expect(graph.connect<"out", "in">(block, sink).has_value());

            gr::scheduler::Simple<> scheduler;
            expect(scheduler.exchange(std::move(graph)).has_value());
            expect(scheduler.runAndWait().has_value());

            std::size_t seen = 0UZ;
            for (const gr::Tag& receivedTag : sink._tags) {
                const auto found = receivedTag.map.find(gr::tag::FREQUENCY.shortKey());
                if (found == receivedTag.map.end()) {
                    continue;
                }
                ++seen;
                const double* value = found->second.get_if<double>();
                expect(value != nullptr);
                expect(that % (*value == kTuned)) << seat << ": the impairment does not move what the stream is centered on";
            }
            expect(eq(seen, 1UZ)) << seat;
        }
    };

    // A ten-minute pass applied and corrected around a modem
    "a corrected pass decodes at the same error rate as no pass at all"_test = [] {
        constexpr double      fs       = 21'000.; // above 2*10 kHz, so the pass's peak is a shift and not an alias
        constexpr std::size_t kSeconds = 600UZ;
        constexpr std::size_t kChunk   = 65'536UZ;
        constexpr double      kNoise   = 0.35; // total complex noise power, at which BPSK of unit energy reads about 1e-2
        const std::size_t     kSymbols = static_cast<std::size_t>(fs) * kSeconds;

        const Pass                              pass          = leoPass(static_cast<double>(kSeconds));
        const gr::digital::Constellation<float> constellation = gr::digital::Constellation<float>::bpsk();

        const auto start = std::chrono::steady_clock::now();

        const auto measure = [&](bool withPass) {
            Bits                      bits;
            std::vector<std::uint8_t> sent(kChunk);
            std::vector<std::uint8_t> got(kChunk);
            std::vector<C>            symbols(kChunk);
            std::vector<C>            scratch(kChunk);

            auto applied = configured<DopplerShift<C>>({{"sample_rate", static_cast<float>(fs)}, {"schedule_times_ns", pass.times}, {"schedule_offsets_hz", pass.offsets}, {"direction", std::string("apply")}});
            auto noise   = configured<AwgnChannel<C>>({{"noise_power", kNoise}, {"seed", std::uint64_t{20260902}}});
            auto removed = configured<DopplerShift<C>>({{"sample_rate", static_cast<float>(fs)}, {"schedule_times_ns", pass.times}, {"schedule_offsets_hz", pass.offsets}, {"direction", std::string("correct")}});

            std::uint64_t errors = 0ULL;
            for (std::size_t at = 0UZ; at < kSymbols; at += kChunk) {
                const std::size_t n = std::min(kChunk, kSymbols - at);
                for (std::size_t k = 0UZ; k < n; ++k) {
                    sent[k]    = bits.next();
                    symbols[k] = constellation.point(sent[k]);
                }
                if (withPass) {
                    std::ignore = applied->processBulk(std::span<const C>(symbols.data(), n), std::span<C>(scratch.data(), n));
                    std::ignore = noise->processBulk(std::span<const C>(scratch.data(), n), std::span<C>(symbols.data(), n));
                    std::ignore = removed->processBulk(std::span<const C>(symbols.data(), n), std::span<C>(scratch.data(), n));
                } else {
                    std::ignore = noise->processBulk(std::span<const C>(symbols.data(), n), std::span<C>(scratch.data(), n));
                }
                constellation.hardDecisions(std::span<const C>(scratch.data(), n), std::span<std::uint8_t>(got.data(), n));
                for (std::size_t k = 0UZ; k < n; ++k) {
                    errors += (got[k] != sent[k]) ? 1ULL : 0ULL;
                }
            }
            return static_cast<double>(errors) / static_cast<double>(kSymbols);
        };

        const double withPass = measure(true);
        const double clean    = measure(false);
        const double elapsed  = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

        // The correction rotates the noise as well as the signal. The two runs see the same noise distribution but
        // not the same realization. The agreement is statistical, and its bound is the counting error. At this rate
        // over this many symbols, one standard deviation is sqrt(N*p)/N, about 2.8e-5 in the rate.
        const double sigma = std::sqrt(clean * static_cast<double>(kSymbols)) / static_cast<double>(kSymbols);
        std::println("[doppler] corrected pass: {} symbols over a {} s pass at {} Hz — BER {:.6f} corrected against {:.6f} with no pass, difference {:.2e}, one sigma {:.2e}, {:.2f} s", kSymbols, kSeconds, fs, withPass, clean, withPass - clean, sigma, elapsed);
        expect(gt(clean, 1e-3)) << "the reference run must actually be making errors for the comparison to mean anything";
        expect(lt(std::abs(withPass - clean), 5. * sigma)) << std::format("BER {:.6f} corrected against {:.6f}", withPass, clean);
    };
};

int main() { /* not needed for UT */ }
