#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/channel/CarrierImpairments.hpp>
#include <gnuradio-4.0/digital/LinearEqualizer.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::channel::FrequencyOffset;
using gr::blocks::digital::LinearEqualizer;
using gr::testing::ProcessFunction;
using gr::testing::TagSink;
using gr::testing::TagSource;

using CF = std::complex<float>;

/// The reference channel every scene below is built on, its taps one symbol apart.
constexpr std::array<CF, 3UZ> kChannel{CF(1.0f, 0.0f), CF(0.4f, 0.0f), CF(0.0f, 0.2f)};

constexpr gr::Size_t  kNumTaps    = 11U;
constexpr std::size_t kGroupDelay = (kNumTaps - 1U) / 2U; ///< the input samples a spike at the center of `kNumTaps` taps delays by

/// A block carrying a seqlock is neither copyable nor movable, so every one below is built where it is used.
template<typename TBlock>
void init(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    if constexpr (requires { block.start(); }) {
        block.start();
    }
}

/// Build and initialize a block that is then discarded, for the settings a scene only needs accepted or refused.
template<typename TBlock>
void construct(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    init(block);
}

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
};

/// A deterministic unit-power QPSK stream, so every scene below is reproducible from its seed alone.
[[nodiscard]] std::vector<CF> qpsk(std::size_t count, std::uint64_t seed) {
    Rng             rng{seed};
    constexpr float kLevel = 0.70710678f;
    std::vector<CF> symbols(count);
    for (CF& symbol : symbols) {
        const std::uint64_t bits = rng.next();
        symbol                   = CF((bits & 1ULL) != 0ULL ? kLevel : -kLevel, (bits & 2ULL) != 0ULL ? kLevel : -kLevel);
    }
    return symbols;
}

/// `y[n] = sum_k h[k] * x[n-k]`, the same length as the input and taking the history ahead of the stream as zero.
[[nodiscard]] std::vector<CF> throughFilter(std::span<const CF> input, std::span<const CF> impulse) {
    std::vector<CF> output(input.size());
    for (std::size_t n = 0UZ; n < input.size(); ++n) {
        CF accumulator{};
        for (std::size_t k = 0UZ; k < impulse.size() && k <= n; ++k) {
            accumulator += impulse[k] * input[n - k];
        }
        output[n] = accumulator;
    }
    return output;
}

/// Zero stuffing, which puts a symbol stream on the sample grid a fractionally spaced equalizer reads.
[[nodiscard]] std::vector<CF> upsample(std::span<const CF> symbols, std::size_t factor) {
    std::vector<CF> stream(symbols.size() * factor);
    for (std::size_t n = 0UZ; n < symbols.size(); ++n) {
        stream[n * factor] = symbols[n];
    }
    return stream;
}

[[nodiscard]] std::vector<float> interleave(std::span<const CF> symbols) {
    std::vector<float> flat(2UZ * symbols.size());
    for (std::size_t i = 0UZ; i < symbols.size(); ++i) {
        flat[2UZ * i]       = symbols[i].real();
        flat[2UZ * i + 1UZ] = symbols[i].imag();
    }
    return flat;
}

[[nodiscard]] std::vector<CF> deinterleave(std::span<const float> flat) {
    std::vector<CF> symbols(flat.size() / 2UZ);
    for (std::size_t i = 0UZ; i < symbols.size(); ++i) {
        symbols[i] = CF(flat[2UZ * i], flat[2UZ * i + 1UZ]);
    }
    return symbols;
}

[[nodiscard]] bool allFinite(std::span<const CF> samples) {
    return std::ranges::all_of(samples, [](const CF& sample) { return std::isfinite(sample.real()) && std::isfinite(sample.imag()); });
}

/**
 * @brief The off-peak power of the combined channel-and-equalizer response over its peak, in dB.
 *
 * Convolving the taps with the channel gives the response from the transmitted symbols to the equalizer output. At two
 * samples per symbol the decimation keeps every second point of that convolution, the one whose newest input is the
 * second sample of a symbol's pair.
 */
[[nodiscard]] double residualIsiDb(std::span<const float> interleavedTaps, std::span<const CF> channel, std::size_t sps) {
    const std::vector<CF> taps = deinterleave(interleavedTaps);
    std::vector<CF>       response(taps.size() + channel.size() - 1UZ);
    for (std::size_t i = 0UZ; i < taps.size(); ++i) {
        for (std::size_t k = 0UZ; k < channel.size(); ++k) {
            response[i + k] += taps[i] * channel[k];
        }
    }

    double peak  = 0.0;
    double total = 0.0;
    for (std::size_t m = sps - 1UZ; m < response.size(); m += sps) {
        const double power = static_cast<double>(std::norm(response[m]));
        peak               = std::max(peak, power);
        total += power;
    }
    return 10.0 * std::log10((total - peak) / peak);
}

/// The mean squared error of the output against the transmitted symbols at a stated decision delay, over a tail.
[[nodiscard]] double meanSquaredError(std::span<const CF> output, std::span<const CF> symbols, std::size_t delay, std::size_t from) {
    double      total = 0.0;
    std::size_t count = 0UZ;
    for (std::size_t s = std::max(from, delay); s < output.size(); ++s) {
        total += static_cast<double>(std::norm(output[s] - symbols[s - delay]));
        ++count;
    }
    return count == 0UZ ? 0.0 : total / static_cast<double>(count);
}

/// `sum_i w[i] * x[newest-i]` accumulated in double and rounded once, which is the arithmetic a frozen filter runs.
[[nodiscard]] CF staticFir(std::span<const CF> taps, std::span<const CF> history, std::size_t newest) {
    double real = 0.0;
    double imag = 0.0;
    for (std::size_t i = 0UZ; i < taps.size() && i <= newest; ++i) {
        const double tapReal   = static_cast<double>(taps[i].real());
        const double tapImag   = static_cast<double>(taps[i].imag());
        const double inputReal = static_cast<double>(history[newest - i].real());
        const double inputImag = static_cast<double>(history[newest - i].imag());
        real += tapReal * inputReal - tapImag * inputImag;
        imag += tapReal * inputImag + tapImag * inputReal;
    }
    return CF(static_cast<float>(real), static_cast<float>(imag));
}

/// A frame-structured scene: what was sent, what the channel delivered, and the sequence every frame opens with.
struct Scene {
    std::vector<CF>      symbols{};
    std::vector<CF>      received{};
    std::vector<float>   training{};
    std::vector<gr::Tag> triggers{};
};

[[nodiscard]] Scene frameScene(std::size_t frames, std::size_t frameLength, std::size_t trainingLength, std::uint64_t seed) {
    Scene                 scene;
    std::vector<CF>       symbols = qpsk(frames * frameLength, seed);
    const std::vector<CF> known   = qpsk(trainingLength, seed + 1ULL);
    for (std::size_t f = 0UZ; f < frames; ++f) {
        std::ranges::copy(known, std::next(symbols.begin(), static_cast<std::ptrdiff_t>(f * frameLength)));
        scene.triggers.emplace_back(f * frameLength, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("frame")}});
    }
    scene.received = throughFilter(std::span<const CF>(symbols), std::span<const CF>(kChannel));
    scene.training = interleave(std::span<const CF>(known));
    scene.symbols  = std::move(symbols);
    return scene;
}

/// What the sink saw at the far end of `source -> equalizer -> sink`, with the block's observables at the end of it.
struct Run {
    std::vector<CF>      samples{};
    std::vector<gr::Tag> tags{};
    std::vector<float>   taps{};
    double               errorPower = 0.0;
    std::uint64_t        resets     = 0ULL;
    std::uint64_t        bursts     = 0ULL;
};

/**
 * @brief One scheduler-driven run, with @p incoming planted at the source.
 *
 * The equalizer sits directly behind the source because a block forwards only the tag keys it owns itself, and the
 * settings key that engages `freeze` is owned by the equalizer alone.
 */
[[nodiscard]] Run throughGraph(gr::property_map settings, std::span<const CF> input, std::span<const gr::Tag> incoming = {}) {
    gr::Graph  graph;
    const auto values = gr::Tensor<CF>(input.begin(), input.end());
    auto&      source = graph.emplaceBlock<TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(input.size())}, {"values", values}, {"mark_tag", false}});
    source._tags.assign(incoming.begin(), incoming.end());
    std::ranges::sort(source._tags, std::ranges::less{}, &gr::Tag::index);

    auto& equalizer = graph.emplaceBlock<LinearEqualizer<float>>(std::move(settings));
    auto& sink      = graph.emplaceBlock<TagSink<CF, ProcessFunction::USE_PROCESS_BULK>>({{"name", "TagSink"}});

    boost::ut::expect(graph.connect<"out", "in">(source, equalizer).has_value());
    boost::ut::expect(graph.connect<"out", "in">(equalizer, sink).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    const auto finished = scheduler.runAndWait();
    boost::ut::expect(finished.has_value()) << (finished.has_value() ? std::string{} : finished.error().message);

    return {std::vector<CF>(sink._samples.begin(), sink._samples.end()), sink._tags, equalizer.taps(), equalizer.errorPower(), equalizer.nResets(), equalizer.nTrainingBursts()};
}

/**
 * @brief One run over `processBulk` directly, @p chunks cycling the output symbols each call is handed.
 *
 * A scheduler chooses its own chunk boundaries, so the scenes that read the taps between calls, that state the call
 * boundaries themselves, or that change `freeze` part way through drive the block over `TestSpans` instead of a graph.
 * An empty @p chunks is the whole stream in one call; @p tapTrace, when given, collects the `taps` observable after
 * every call.
 */
[[nodiscard]] std::vector<CF> drive(LinearEqualizer<float>& equalizer, std::span<const CF> input, std::size_t sps, std::span<const gr::Tag> incoming = {}, std::span<const std::size_t> chunks = {}, std::vector<std::vector<float>>* tapTrace = nullptr) {
    namespace test = gr::blocks::testing::span;

    const std::size_t    nSymbols = input.size() / sps;
    std::vector<CF>      output(nSymbols);
    std::vector<gr::Tag> published;
    std::size_t          done  = 0UZ;
    std::size_t          which = 0UZ;

    while (done < nSymbols) {
        const std::size_t asked = chunks.empty() ? nSymbols : std::max(chunks[which % chunks.size()], 1UZ);
        const std::size_t take  = std::min(asked, nSymbols - done);
        const std::size_t base  = done * sps;
        const std::size_t count = take * sps;
        const auto        first = std::ranges::lower_bound(incoming, base, std::ranges::less{}, &gr::Tag::index);
        const auto        last  = std::ranges::lower_bound(incoming, base + count, std::ranges::less{}, &gr::Tag::index);

        test::InputSpan<CF>  inSpan(input.subspan(base, count), base, std::span<const gr::Tag>(first, last));
        test::OutputSpan<CF> outSpan(std::span<CF>(output.data() + done, take), done, &published);
        std::ignore = equalizer.processBulk(inSpan, outSpan);
        if (tapTrace != nullptr) {
            tapTrace->push_back(equalizer.taps());
        }
        done += take;
        ++which;
    }
    return output;
}

/**
 * @brief A run whose output room, not its input window, limits every call.
 *
 * @p window input samples are offered per call while only @p room output symbols are reserved, so `processBulk` takes
 * the smaller of the two and leaves an input tail unconsumed, which the next call offers again. This is the path the
 * block's own `std::min` against the output span serves, and the one where a trigger sitting in that tail has to wait
 * for the call that consumes it rather than count once per call it is offered in.
 */
[[nodiscard]] std::vector<CF> driveOutputLimited(LinearEqualizer<float>& equalizer, std::span<const CF> input, std::size_t sps, std::span<const gr::Tag> incoming, std::size_t window, std::size_t room) {
    namespace test = gr::blocks::testing::span;

    std::vector<CF>      output(input.size() / sps);
    std::vector<gr::Tag> published;
    std::size_t          consumed = 0UZ;
    std::size_t          done     = 0UZ;

    while (consumed + sps <= input.size()) {
        const std::size_t offered = std::min(window, input.size() - consumed);
        const std::size_t reserve = std::min(room, output.size() - done);
        const auto        first   = std::ranges::lower_bound(incoming, consumed, std::ranges::less{}, &gr::Tag::index);
        const auto        last    = std::ranges::lower_bound(incoming, consumed + offered, std::ranges::less{}, &gr::Tag::index);

        test::InputSpan<CF>  inSpan(input.subspan(consumed, offered), consumed, std::span<const gr::Tag>(first, last));
        test::OutputSpan<CF> outSpan(std::span<CF>(output.data() + done, reserve), done, &published);
        std::ignore = equalizer.processBulk(inSpan, outSpan);

        if (inSpan.consumed == 0UZ) {
            break;
        }
        consumed += inSpan.consumed;
        done += inSpan.consumed / sps;
    }
    return output;
}

/// The offsets at which a tag carrying @p key reached the sink.
[[nodiscard]] std::vector<std::size_t> offsetsOf(std::span<const gr::Tag> tags, std::string_view key) {
    std::vector<std::size_t> where;
    for (const gr::Tag& tag : tags) {
        if (tag.map.contains(gr::property_map::key_type(key))) {
            where.push_back(tag.index);
        }
    }
    return where;
}

} // namespace

const boost::ut::suite<"linear equalizer"> linearEqualizerTests = [] {
    using namespace boost::ut;

    "a trained equalizer inverts the channel and counts its training runs"_test = [] {
        constexpr std::size_t kFrames      = 2UZ;
        constexpr std::size_t kFrameLength = 1000UZ;
        constexpr std::size_t kTraining    = 64UZ;
        constexpr std::size_t kConverged   = 1200UZ;

        const Scene scene = frameScene(kFrames, kFrameLength, kTraining, 1234ULL);
        const Run   run   = throughGraph({{"num_taps", kNumTaps}, {"algorithm", std::string("lms")}, {"reference", std::string("training")}, {"step_size", 0.01}, {"training_sequence", scene.training}}, std::span<const CF>(scene.received), std::span<const gr::Tag>(scene.triggers));

        expect(eq(run.samples.size(), scene.symbols.size())) << "one output symbol per input at one sample per symbol";
        expect(eq(run.bursts, 2ULL)) << "one training run per frame";
        expect(eq(run.resets, 0ULL));

        const double mse = meanSquaredError(run.samples, scene.symbols, kGroupDelay, kConverged);
        const double isi = residualIsiDb(run.taps, kChannel, 1UZ);
        std::println("trained inversion: MSE {:.4e} ({:.2f} dB), residual ISI {:.2f} dB, error_power {:.4e}", mse, 10.0 * std::log10(mse), isi, run.errorPower);

        expect(lt(isi, -20.0)) << std::format("residual intersymbol interference {:.2f} dB", isi);
        expect(lt(mse, 0.005)) << std::format("mean squared error {:.4e} over the tail of the second frame", mse);
        expect(that % (run.errorPower > 0.25 * mse && run.errorPower < 4.0 * mse)) << std::format("the error_power observable reads {:.4e} against a measured {:.4e}", run.errorPower, mse);
    };

    "with no trigger the block adapts on decisions and stays finite"_test = [] {
        const Scene scene = frameScene(2UZ, 1000UZ, 64UZ, 1234ULL);
        const Run   run   = throughGraph({{"num_taps", kNumTaps}, {"algorithm", std::string("lms")}, {"reference", std::string("training")}, {"step_size", 0.01}, {"training_sequence", scene.training}}, std::span<const CF>(scene.received));

        expect(eq(run.samples.size(), scene.symbols.size()));
        expect(eq(run.bursts, 0ULL)) << "no trigger ever arrived, so no training run ever started";
        expect(eq(run.resets, 0ULL));
        expect(allFinite(run.samples)) << "the degraded path is still a stream of finite samples";

        std::vector<float> spike(2UZ * kNumTaps, 0.0f);
        spike[2UZ * kGroupDelay] = 1.0f;
        expect(that % (run.taps != spike)) << "the taps moved, so the block adapted on its own decisions";

        std::println("no trigger: MSE {:.4e}, residual ISI {:.2f} dB", meanSquaredError(run.samples, scene.symbols, kGroupDelay, 1200UZ), residualIsiDb(run.taps, kChannel, 1UZ));
    };

    "every trigger starts a training run, however the calls fall around it"_test = [] {
        constexpr std::size_t kSymbols  = 600UZ;
        constexpr std::size_t kCalls[]  = {1UZ, 3UZ, 5UZ, 6UZ, 16UZ, 997UZ};
        constexpr std::size_t kRooms[]  = {20UZ, 7UZ, 1UZ}; ///< output symbols reserved where the room is the limit
        constexpr std::size_t kWindow   = 200UZ;            ///< input samples offered per call against that room
        constexpr std::size_t kTraining = 4UZ;              ///< short, so one run ends well before the next trigger arrives

        // Triggers closer together than the group delay the block adds before a run begins. A scheduler breaks a chunk
        // at every tag, so the chunk carrying the earlier trigger is shorter than that delay and the symbol its run
        // starts at falls beyond the chunk; the run has to be carried to the following call rather than dropped there.
        const std::vector<std::vector<std::size_t>> kPlacements{{0UZ, 500UZ}, {0UZ, 3UZ}, {0UZ, 1UZ, 2UZ}, {7UZ, 9UZ}, {0UZ, 1UZ, 2UZ, 3UZ, 4UZ, 5UZ}, {0UZ, 100UZ, 200UZ}};

        const std::vector<CF>    symbols  = qpsk(kSymbols, 4242ULL);
        const std::vector<CF>    received = throughFilter(std::span<const CF>(symbols), std::span<const CF>(kChannel));
        const std::vector<float> training = interleave(std::span<const CF>(symbols).first(kTraining));

        for (const std::vector<std::size_t>& placement : kPlacements) {
            std::vector<gr::Tag> triggers;
            for (const std::size_t at : placement) {
                triggers.emplace_back(at, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("frame")}});
            }
            const auto wanted = placement.size();

            for (const gr::Size_t sps : {1U, 2U}) {
                const gr::property_map settings{{"num_taps", kNumTaps}, {"algorithm", std::string("lms")}, {"reference", std::string("training")}, {"step_size", 0.01}, {"samples_per_symbol", sps}, {"training_sequence", training}};

                for (const std::size_t call : kCalls) {
                    const std::size_t      partition[]{call};
                    LinearEqualizer<float> equalizer(settings);
                    init(equalizer);
                    std::ignore = drive(equalizer, std::span<const CF>(received), static_cast<std::size_t>(sps), std::span<const gr::Tag>(triggers), std::span<const std::size_t>(partition));
                    expect(eq(equalizer.nTrainingBursts(), wanted)) << std::format("sps {}, {} symbols per call, triggers at {}", sps, call, placement);
                }

                const Run run = throughGraph(settings, std::span<const CF>(received), std::span<const gr::Tag>(triggers));
                expect(eq(run.bursts, wanted)) << std::format("sps {}, through the scheduler, triggers at {}", sps, placement);

                // and the same count when the output room is the limit: the triggers the call could not reach wait for
                // the call that consumes them instead of counting once per call they are offered in
                for (const std::size_t room : kRooms) {
                    LinearEqualizer<float> equalizer(settings);
                    init(equalizer);
                    std::ignore = driveOutputLimited(equalizer, std::span<const CF>(received), static_cast<std::size_t>(sps), std::span<const gr::Tag>(triggers), kWindow, room);
                    expect(eq(equalizer.nTrainingBursts(), wanted)) << std::format("sps {}, a {}-sample window with room for {} symbols, triggers at {}", sps, kWindow, room, placement);
                }
            }
        }
    };

    "the modulus rule converges from a cold start without a reference"_test = [] {
        constexpr std::size_t kSymbols  = 8000UZ;
        constexpr double      kCrossing = -15.0;
        constexpr std::size_t kOneAtATime[]{1UZ}; ///< one symbol per call, so the tap trace is per symbol
        const std::vector<CF> symbols  = qpsk(kSymbols, 777ULL);
        const std::vector<CF> received = throughFilter(std::span<const CF>(symbols), std::span<const CF>(kChannel));

        LinearEqualizer<float> equalizer({{"num_taps", kNumTaps}, {"algorithm", std::string("cma")}, {"step_size", 0.003}});
        init(equalizer);
        std::vector<std::vector<float>> trace;
        const std::vector<CF>           output = drive(equalizer, std::span<const CF>(received), 1UZ, {}, std::span<const std::size_t>(kOneAtATime), &trace);

        std::size_t acquired = trace.size();
        for (std::size_t s = 0UZ; s < trace.size(); ++s) {
            if (residualIsiDb(trace[s], kChannel, 1UZ) < kCrossing) {
                acquired = s + 1UZ;
                break;
            }
        }

        double      modulus = 0.0;
        std::size_t counted = 0UZ;
        for (std::size_t s = kSymbols / 2UZ; s < output.size(); ++s) {
            const double excess = static_cast<double>(std::norm(output[s])) - 1.0;
            modulus += excess * excess;
            ++counted;
        }
        modulus /= static_cast<double>(counted);
        const double isi = residualIsiDb(equalizer.taps(), kChannel, 1UZ);

        std::println("modulus rule: mean (|y|^2 - 1)^2 {:.4e}, residual ISI {:.2f} dB, {} symbols to cross {:.0f} dB", modulus, isi, acquired, kCrossing);

        // the rule drives the output modulus and leaves the phase free, so a converged filter still carries an
        // arbitrary rotation and nothing here is asserted against the transmitted symbols
        expect(lt(modulus, 0.02)) << std::format("mean squared modulus error {:.4e}", modulus);
        expect(lt(isi, -20.0)) << std::format("residual intersymbol interference {:.2f} dB", isi);
        expect(lt(acquired, 2000UZ)) << std::format("{} symbols to first cross {:.0f} dB", acquired, kCrossing);
        expect(eq(equalizer.nTrainingBursts(), 0ULL)) << "the modulus rule reads no reference at all";
    };

    "decision-directed adaptation tracks a rotating channel that freezing cannot"_test = [] {
        constexpr std::size_t kSymbols  = 8000UZ;
        constexpr std::size_t kFreezeAt = 2000UZ;
        constexpr std::size_t kTail     = 6000UZ;
        constexpr double      kRotation = 3e-5; ///< cycles per symbol, a rate the loop follows and a held filter does not

        const Scene scene = frameScene(1UZ, kSymbols, 64UZ, 4242ULL);

        // the impairment is built into the scene rather than placed in the graph, because a block standing between the
        // source and the equalizer forwards only the tag keys it owns itself and `freeze` is not one of them
        FrequencyOffset<CF> rotator({{"sample_rate", 1.0f}, {"frequency_offset", kRotation}});
        init(rotator);
        std::vector<CF> rotated(scene.received.size());
        std::ignore = rotator.processBulk(std::span<const CF>(scene.received), std::span<CF>(rotated));

        std::vector<double> mse(2UZ);
        for (std::size_t which = 0UZ; which < 2UZ; ++which) {
            std::vector<gr::Tag> tags = scene.triggers;
            if (which == 1UZ) {
                tags.emplace_back(kFreezeAt, gr::property_map{{"freeze", true}});
            }
            const Run run = throughGraph({{"num_taps", kNumTaps}, {"algorithm", std::string("lms")}, {"reference", std::string("training")}, {"step_size", 0.01}, {"training_sequence", scene.training}}, std::span<const CF>(rotated), std::span<const gr::Tag>(tags));
            expect(eq(run.bursts, 1ULL));
            expect(allFinite(run.samples));
            mse[which] = meanSquaredError(run.samples, scene.symbols, kGroupDelay, kTail);
        }

        std::println("rotating channel at {:.1e} cycles per symbol: tracking MSE {:.4e}, frozen MSE {:.4e}", kRotation, mse[0], mse[1]);
        expect(lt(mse[0], mse[1])) << std::format("tracking {:.4e} against frozen {:.4e}", mse[0], mse[1]);
    };

    "fractional spacing beats symbol spacing on a half-sample timing offset"_test = [] {
        constexpr std::size_t kSymbols  = 12000UZ;
        constexpr std::size_t kTraining = 4000UZ;
        constexpr std::size_t kTail     = 8000UZ;

        const std::vector<CF> symbols = qpsk(kSymbols, 31337ULL);

        // A two-tap linear interpolator at the two-sample rate, whose transmit pulse is the triangle it produces: the
        // odd phase of the oversampled stream carries the symbols cleanly and the even phase sits half a symbol away
        // from them, which is the timing offset a symbol-spaced equalizer has to live with.
        const std::vector<CF> pulse{CF(0.5f, 0.0f), CF(1.0f, 0.0f), CF(0.5f, 0.0f)};
        const std::vector<CF> channelAtTwo{CF(1.0f, 0.0f), CF(0.0f, 0.0f), CF(0.4f, 0.0f), CF(0.0f, 0.0f), CF(0.0f, 0.2f)};
        const std::vector<CF> shaped   = throughFilter(std::span<const CF>(upsample(std::span<const CF>(symbols), 2UZ)), std::span<const CF>(pulse));
        const std::vector<CF> received = throughFilter(std::span<const CF>(shaped), std::span<const CF>(channelAtTwo));

        std::vector<CF> offsetPhase;
        for (std::size_t m = 0UZ; m < received.size(); m += 2UZ) {
            offsetPhase.push_back(received[m]);
        }

        const std::vector<float>   training = interleave(std::span<const CF>(symbols).first(kTraining));
        const std::vector<gr::Tag> triggers{gr::Tag{0UZ, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("frame")}}}};

        // a trigger at input t starts the training run at output (t + group delay) / sps, so the reference the block
        // adapts against sits at a decision delay of the group delay counted in symbols, which is what is measured here
        std::vector<double> mse(2UZ);
        for (const gr::Size_t sps : {1U, 2U}) {
            const std::span<const CF> input = sps == 1U ? std::span<const CF>(offsetPhase) : std::span<const CF>(received);
            const Run                 run   = throughGraph({{"num_taps", kNumTaps}, {"algorithm", std::string("lms")}, {"reference", std::string("training")}, {"step_size", 0.01}, {"samples_per_symbol", sps}, {"training_sequence", training}}, input, std::span<const gr::Tag>(triggers));

            expect(eq(run.samples.size(), kSymbols));
            expect(eq(run.bursts, 1ULL));
            mse[sps - 1U] = meanSquaredError(run.samples, symbols, kGroupDelay / sps, kTail);
        }

        std::println("half-sample timing offset: symbol-spaced MSE {:.4e}, fractionally spaced MSE {:.4e}", mse[0], mse[1]);
        expect(lt(mse[1], mse[0])) << std::format("fractionally spaced {:.4e} against symbol spaced {:.4e}", mse[1], mse[0]);
    };

    "a step size past the stable range resets the taps rather than streaming infinities"_test = [] {
        constexpr double      kStepSize = 0.5;
        const std::vector<CF> symbols   = qpsk(4000UZ, 5150ULL);
        const std::vector<CF> received  = throughFilter(std::span<const CF>(symbols), std::span<const CF>(kChannel));

        const Run run = throughGraph({{"num_taps", kNumTaps}, {"algorithm", std::string("lms")}, {"step_size", kStepSize}}, std::span<const CF>(received));

        std::println("step size {}: {} resets over {} symbols", kStepSize, run.resets, run.samples.size());
        expect(gt(run.resets, gr::Size_t{0})) << std::format("a step size of {} walks the taps past the divergence bound", kStepSize);
        expect(allFinite(run.samples)) << "no sample of the run is a NaN or an infinity";
    };

    "freeze holds the taps exactly and leaves a static filter running"_test = [] {
        constexpr std::size_t kSymbols = 4000UZ;
        constexpr std::size_t kAdapted = 2000UZ; ///< symbols the second filter adapts over before it is held
        constexpr std::size_t kInCalls[]{37UZ};  ///< an irregular call length, so the taps are read back on many calls
        const std::vector<CF> symbols  = qpsk(kSymbols, 20250830ULL);
        const std::vector<CF> received = throughFilter(std::span<const CF>(symbols), std::span<const CF>(kChannel));

        const auto matchesStaticFir = [&received](std::span<const CF> output, std::span<const float> interleavedTaps, std::size_t from) {
            const std::vector<CF> taps   = deinterleave(interleavedTaps);
            std::size_t           differ = 0UZ;
            for (std::size_t s = 0UZ; s < output.size(); ++s) {
                differ += output[s] != staticFir(taps, std::span<const CF>(received), from + s) ? 1UZ : 0UZ;
            }
            return differ;
        };

        LinearEqualizer<float> held({{"num_taps", kNumTaps}, {"algorithm", std::string("lms")}, {"step_size", 0.01}, {"freeze", true}});
        init(held);
        const std::vector<float>        start = held.taps();
        std::vector<std::vector<float>> trace;
        const std::vector<CF>           output = drive(held, std::span<const CF>(received), 1UZ, {}, std::span<const std::size_t>(kInCalls), &trace);

        expect(that % std::ranges::all_of(trace, [&start](const std::vector<float>& taps) { return taps == start; })) << "held from the start, the taps are the same bits at every call";
        expect(eq(held.nResets(), 0ULL));
        expect(eq(matchesStaticFir(output, start, 0UZ), 0UZ)) << "and the output is that filter run over the input, bit for bit";

        // the same claim about a set of taps that is not the initial spike: adapt for a while, then hold
        LinearEqualizer<float> converged({{"num_taps", kNumTaps}, {"algorithm", std::string("lms")}, {"step_size", 0.01}});
        init(converged);
        std::ignore = drive(converged, std::span<const CF>(received).first(kAdapted), 1UZ, {}, std::span<const std::size_t>(kInCalls));

        const std::vector<float> frozen = converged.taps();
        converged.freeze                = true;
        trace.clear();
        const std::vector<CF> afterFreeze = drive(converged, std::span<const CF>(received).subspan(kAdapted), 1UZ, {}, std::span<const std::size_t>(kInCalls), &trace);

        expect(that % (frozen != start)) << "the taps that were held had moved away from the spike first";
        expect(that % std::ranges::all_of(trace, [&frozen](const std::vector<float>& taps) { return taps == frozen; })) << "adaptation stopped where freeze was set";
        expect(eq(matchesStaticFir(afterFreeze, frozen, kAdapted), 0UZ)) << "and filtering continued, bit for bit, on the taps it stopped at";
    };

    "one output symbol per samples_per_symbol inputs, and a tag at input t lands at output t/sps"_test = [] {
        constexpr std::size_t kInputs   = 512UZ;
        constexpr std::size_t kOffset[] = {0UZ, 8UZ, 100UZ, 301UZ};
        const std::vector<CF> input     = qpsk(kInputs, 9ULL);

        std::vector<gr::Tag> triggers;
        for (const std::size_t at : kOffset) {
            triggers.emplace_back(at, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("frame")}});
        }

        for (const gr::Size_t sps : {1U, 2U}) {
            LinearEqualizer<float> block({{"num_taps", kNumTaps}, {"samples_per_symbol", sps}});
            init(block);
            expect(eq(static_cast<std::size_t>(block.input_chunk_size.value), static_cast<std::size_t>(sps)));
            expect(eq(static_cast<std::size_t>(block.output_chunk_size.value), 1UZ));

            const Run run = throughGraph({{"num_taps", kNumTaps}, {"samples_per_symbol", sps}}, std::span<const CF>(input), std::span<const gr::Tag>(triggers));
            expect(eq(run.samples.size(), kInputs / static_cast<std::size_t>(sps))) << std::format("sps {}", sps);

            std::vector<std::size_t> wanted;
            for (const std::size_t at : kOffset) {
                wanted.push_back(at / static_cast<std::size_t>(sps));
            }
            expect(that % (offsetsOf(run.tags, gr::tag::TRIGGER_NAME.shortKey()) == wanted)) << std::format("sps {}: a tag at input t arrives at output t/sps", sps);
        }
    };

    "the output and the taps do not depend on how the input was divided into calls"_test = [] {
        constexpr std::size_t kGrowing[]{1UZ, 2UZ, 3UZ, 5UZ, 8UZ, 13UZ, 1UZ, 64UZ};
        constexpr std::size_t kCoarse[]{997UZ, 1UZ, 17UZ, 3UZ};
        const std::vector<CF> symbols  = qpsk(4000UZ, 20250830ULL);
        const std::vector<CF> received = throughFilter(std::span<const CF>(symbols), std::span<const CF>(kChannel));

        for (const gr::Size_t sps : {1U, 2U}) {
            const gr::property_map settings{{"num_taps", kNumTaps}, {"algorithm", std::string("lms")}, {"step_size", 0.01}, {"samples_per_symbol", sps}};

            auto run = [&](std::span<const std::size_t> partition) {
                LinearEqualizer<float> equalizer(settings);
                init(equalizer);
                std::vector<CF> output = drive(equalizer, std::span<const CF>(received), static_cast<std::size_t>(sps), {}, partition);
                return std::pair<std::vector<CF>, std::vector<float>>{std::move(output), equalizer.taps()};
            };

            const auto whole  = run({});
            const auto first  = run(std::span<const std::size_t>(kGrowing));
            const auto second = run(std::span<const std::size_t>(kCoarse));

            expect(that % (first.first == whole.first)) << std::format("sps {}: the output over a growing partition", sps);
            expect(that % (first.second == whole.second)) << std::format("sps {}: the taps over a growing partition", sps);
            expect(that % (second.first == whole.first)) << std::format("sps {}: the output over a coarse partition", sps);
            expect(that % (second.second == whole.second)) << std::format("sps {}: the taps over a coarse partition", sps);
        }
    };

    "degenerate settings throw"_test = [] {
        // a throwing applyStagedParameters() leaves the offending value staged, so every case gets its own block
        expect(throws([] { construct<LinearEqualizer<float>>({{"num_taps", 10U}}); })) << "an even tap count leaves the spike without a center";
        expect(throws([] { construct<LinearEqualizer<float>>({{"num_taps", 0U}}); }));
        expect(throws([] { construct<LinearEqualizer<float>>({{"num_taps", 129U}}); })) << "128 taps is the longest filter the kernel accepts";
        expect(throws([] { construct<LinearEqualizer<float>>({{"samples_per_symbol", 0U}}); }));
        expect(throws([] { construct<LinearEqualizer<float>>({{"samples_per_symbol", 3U}}); })) << "the spacing is symbol or fractional, nothing else";
        expect(throws([] { construct<LinearEqualizer<float>>({{"step_size", 0.0}}); }));
        expect(throws([] { construct<LinearEqualizer<float>>({{"step_size", 1.0}}); })) << "the step size is open at both ends of (0, 1)";
        expect(throws([] { construct<LinearEqualizer<float>>({{"algorithm", std::string("rls")}}); }));
        expect(throws([] { construct<LinearEqualizer<float>>({{"reference", std::string("blind")}}); }));
        expect(throws([] { construct<LinearEqualizer<float>>({{"reference", std::string("training")}}); })) << "a training reference needs a sequence to adapt against";
        expect(throws([] { construct<LinearEqualizer<float>>({{"reference", std::string("training")}, {"training_sequence", std::vector<float>{1.0f, 0.0f, 1.0f}}}); })) << "the sequence is interleaved re,im and needs an even count";

        expect(nothrow([] { construct<LinearEqualizer<float>>(); }));
        expect(nothrow([] { construct<LinearEqualizer<float>>({{"algorithm", std::string("nlms")}}); }));
        expect(nothrow([] { construct<LinearEqualizer<float>>({{"num_taps", 127U}, {"samples_per_symbol", 2U}}); }));
    };
};

int main() { /* not needed for UT */ }
