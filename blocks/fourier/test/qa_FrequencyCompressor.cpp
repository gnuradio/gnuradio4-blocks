#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <format>
#include <limits>
#include <numbers>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/fourier/FrequencyCompressor.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using CF = std::complex<float>;

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

/// Takes at most `_per` samples a call. Output queues inside the block it reads from.
template<typename T>
struct CollectSink : gr::Block<CollectSink<T>> {
    gr::PortIn<T> in;
    GR_MAKE_REFLECTABLE(CollectSink, in);
    std::vector<T>                 _got;
    std::size_t                    _per = std::numeric_limits<std::size_t>::max();
    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& input) {
        const std::size_t n = std::min(_per, input.size());
        _got.insert(_got.end(), input.begin(), input.begin() + static_cast<std::ptrdiff_t>(n));
        std::ignore = input.consume(n);
        return gr::work::Status::OK;
    }
};

[[nodiscard]] std::vector<CF> tone(std::size_t count, double cycles) {
    std::vector<CF> outData(count);
    for (std::size_t n = 0UZ; n < count; ++n) {
        const double phase = 2.0 * std::numbers::pi * cycles * static_cast<double>(n);
        outData[n]         = CF{static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))};
    }
    return outData;
}

/// The frequency of @p x in cycles per sample, from the mean lag-one product.
[[nodiscard]] double cyclesPerSample(std::span<const CF> x) {
    std::complex<double> lag{};
    for (std::size_t i = 0UZ; i + 1UZ < x.size(); ++i) {
        lag += std::complex<double>(x[i + 1UZ]) * std::conj(std::complex<double>(x[i]));
    }
    return std::arg(lag) / (2.0 * std::numbers::pi);
}

/// The outputs the vocoder completes from @p n input samples: one synthesis hop per whole analysis frame.
[[nodiscard]] std::size_t completedOutputs(std::size_t n, std::size_t frame, std::size_t divisor) {
    const std::size_t hopIn  = frame / 4UZ - (frame / 4UZ) % divisor;
    const std::size_t hopOut = hopIn / divisor;
    return n < frame ? 0UZ : ((n - frame) / hopIn + 1UZ) * hopOut;
}

/// @brief One call into the block. Returns what it published and any tag it placed.
struct Call {
    std::size_t          made{};
    std::size_t          consumed{};
    std::vector<gr::Tag> tags{};
};

/// The settings change is placed exactly between two calls. The scheduler cannot be asked to change a setting at a
/// chosen sample. A test that waited for one would depend on timing.
[[nodiscard]] Call drive(gr::blocks::fourier::FrequencyCompressor& block, std::span<const CF> input, std::span<CF> output) {
    namespace span = gr::blocks::testing::span;
    std::vector<gr::Tag> published;
    span::InputSpan<CF>  inSpan{input};
    span::OutputSpan<CF> outSpan{output, 0UZ, &published};
    std::ignore = block.processBulk(inSpan, outSpan);
    return Call{.made = outSpan.count, .consumed = inSpan.consumed, .tags = published};
}

/// Drives the block until a call publishes output, at most @p calls times. Returns that call.
[[nodiscard]] Call driveToOutput(gr::blocks::fourier::FrequencyCompressor& block, std::span<const CF> input, std::span<CF> output, std::size_t calls = 8UZ) {
    Call last;
    for (std::size_t call = 0UZ; call < calls && last.made == 0UZ; ++call) {
        last = drive(block, input, output);
    }
    return last;
}

/// @brief Stages a live settings change and applies it, as work() does. Returns the refusal, empty when none.
[[nodiscard]] std::string applyLive(gr::blocks::fourier::FrequencyCompressor& block, gr::property_map changes) {
    try {
        std::ignore = block.settings().setStaged(std::move(changes));
        return block.settings().applyStagedParameters().refusal;
    } catch (const gr::exception& error) {
        return error.message;
    }
}

/// The rates the tags in @p tags state, under either spelling of the key.
[[nodiscard]] std::vector<float> ratesIn(std::span<const gr::Tag> tags) {
    std::vector<float> rates;
    for (const gr::Tag& tag : tags) {
        for (const auto& [key, value] : tag.map) {
            if (std::string_view(key) == gr::tag::SAMPLE_RATE.shortKey() || std::string_view(key) == gr::tag::SAMPLE_RATE.key()) {
                rates.push_back(value.value_or(std::numeric_limits<float>::quiet_NaN()));
            }
        }
    }
    return rates;
}

/// A block brought up with @p settings, started, outside a graph.
void bringUp(gr::blocks::fourier::FrequencyCompressor& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

/// Run source, compressor and sink to the end of the stream on the Simple scheduler, with the sink taking at most
/// @p perCall samples a call.
[[nodiscard]] std::vector<CF> runGraph(gr::property_map settings, std::vector<CF> data, std::size_t perCall = std::numeric_limits<std::size_t>::max()) {
    gr::Graph flow;
    auto&     src = flow.emplaceBlock<FiniteSource<CF>>();
    src._data     = std::move(data);
    auto& comp    = flow.emplaceBlock<gr::blocks::fourier::FrequencyCompressor>(std::move(settings));
    auto& sink    = flow.emplaceBlock<CollectSink<CF>>();
    sink._per     = perCall;
    boost::ut::expect(flow.connect<"out", "in">(src, comp).has_value());
    boost::ut::expect(flow.connect<"out", "in">(comp, sink).has_value());

    gr::scheduler::Simple<> scheduler;
    boost::ut::expect(scheduler.exchange(std::move(flow)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value()) << "the graph ends on its own";
    return sink._got;
}

} // namespace

const boost::ut::suite<"FrequencyCompressor"> frequencyCompressorTests = [] {
    using namespace boost::ut;
    using gr::blocks::fourier::FrequencyCompressor;

    "a finite graph divides the stream and ends on its own"_test = [] {
        constexpr std::size_t kFrame   = 1024UZ;
        constexpr std::size_t kDivisor = 8UZ;
        const std::size_t     n        = 128UZ * kFrame + 100UZ;
        const auto            got      = runGraph({{"divisor", gr::Size_t(kDivisor)}, {"frame", gr::Size_t(kFrame)}}, tone(n, 0.2));

        expect(eq(got.size(), completedOutputs(n, kFrame, kDivisor))) << "every completed hop arrives, and the samples short of a frame do not";
        double power = 0.0;
        for (std::size_t i = got.size() / 2UZ; i < got.size(); ++i) {
            power += static_cast<double>(std::norm(got[i]));
        }
        power /= static_cast<double>(got.size() - got.size() / 2UZ);
        expect(lt(std::abs(10.0 * std::log10(power)), 0.5)) << std::format("settled level {} dB against the unit tone", 10.0 * std::log10(power));
    };

    "output queued at the end of the stream reaches a slow sink"_test = [] {
        constexpr std::size_t kFrame   = 1024UZ;
        constexpr std::size_t kDivisor = 8UZ;
        const std::size_t     n        = 300'000UZ;
        const auto            got      = runGraph({{"divisor", gr::Size_t(kDivisor)}, {"frame", gr::Size_t(kFrame)}}, tone(n, 0.2), 16UZ);
        expect(eq(got.size(), completedOutputs(n, kFrame, kDivisor))) << "a sink that takes sixteen samples a call still receives every completed hop";
    };

    "divisor 1 is a bit-exact passthrough"_test = [] {
        const auto x   = tone(10000UZ, 0.05);
        const auto got = runGraph({{"divisor", gr::Size_t(1)}}, x);
        expect(that % (got == x)) << "every sample as it arrived";
    };

    "a tone at f cycles per input sample leaves at f cycles per output sample"_test = [] {
        constexpr std::size_t kFrame   = 1024UZ;
        constexpr std::size_t kDivisor = 8UZ;
        for (const double cycles : {0.2, -0.13, 0.0371}) {
            FrequencyCompressor block({{"divisor", gr::Size_t(kDivisor)}, {"frame", gr::Size_t(kFrame)}});
            bringUp(block);
            const auto      input = tone(1UZ << 16, cycles);
            std::vector<CF> output(1UZ << 16);
            const Call      call = drive(block, input, output);
            expect(fatal(gt(call.made, 2UZ * kFrame)));
            const double got = cyclesPerSample(std::span<const CF>(output).subspan(call.made / 4UZ, call.made / 2UZ));
            // A hundredth of a bin of the analysis frame. The output rate is the input rate over the divisor. The tone's
            // frequency in hertz is divided by the divisor.
            expect(lt(std::abs(got - cycles), 0.01 / static_cast<double>(kFrame))) << std::format("{:+.4f} cycles in, {:+.6f} cycles out", cycles, got);
        }
    };

    "each refused setting throws and names the setting"_test = [] {
        const auto refusal = [](gr::property_map settings) {
            try {
                FrequencyCompressor block(settings);
                bringUp(block);
            } catch (const gr::exception& error) {
                return error.message;
            }
            return std::string{};
        };
        const std::string zero = refusal({{"divisor", gr::Size_t(0)}});
        expect(zero.contains("divisor")) << "divisor 0: " << zero;
        const std::string notPower = refusal({{"frame", gr::Size_t(1000)}});
        expect(notPower.contains("frame")) << "a frame that is not a power of two: " << notPower;
        const std::string pastQuarter = refusal({{"divisor", gr::Size_t(512)}, {"frame", gr::Size_t(1024)}});
        expect(pastQuarter.contains("divisor") && pastQuarter.contains("frame")) << "a divisor past a quarter frame: " << pastQuarter;
        const std::string negativeRate = refusal({{"sample_rate", -48000.f}});
        expect(negativeRate.contains("sample_rate")) << "a negative rate: " << negativeRate;
    };

    "a refused live value leaves the block on its accepted settings, and a later change is taken"_test = [] {
        constexpr float     kRate = 48000.f;
        FrequencyCompressor block({{"divisor", gr::Size_t(4)}, {"frame", gr::Size_t(1024)}, {"sample_rate", kRate}});
        bringUp(block);

        const auto      input = tone(1UZ << 16, 0.05);
        std::vector<CF> output(1UZ << 16);
        expect(fatal(gt(driveToOutput(block, input, output).made, 0UZ)));

        const auto onAccepted = [&block](std::string_view refused) {
            expect(eq(block.divisor.value, gr::Size_t(4))) << refused;
            expect(eq(block.frame.value, gr::Size_t(1024))) << refused;
        };

        expect(!applyLive(block, {{"divisor", gr::Size_t(0)}}).empty());
        onAccepted("after divisor 0");
        Call next = drive(block, input, output);
        expect(gt(next.made, 0UZ)) << "the block keeps producing";
        expect(ratesIn(next.tags).empty()) << "and states no rate, the accepted rate being stated already";

        expect(!applyLive(block, {{"frame", gr::Size_t(1000)}}).empty());
        onAccepted("after frame 1000");
        expect(!applyLive(block, {{"divisor", gr::Size_t(512)}}).empty());
        onAccepted("after a divisor past a quarter frame");

        const std::string later = applyLive(block, {{"sample_rate", 2.f * kRate}});
        expect(later.empty()) << "a valid change after the refusals is taken: " << later;
        next             = driveToOutput(block, input, output);
        const auto rates = ratesIn(next.tags);
        expect(eq(rates.size(), 1UZ)) << "a new input rate is stated on the next output";
        if (rates.size() == 1UZ) {
            expect(eq(rates[0], 2.f * kRate / 4.f)) << "as the new input rate over the accepted divisor";
        }
    };

    "the output carries one sample rate, its own"_test = [] {
        constexpr float      kRate    = 48000.f;
        constexpr gr::Size_t kDivisor = 4U;

        gr::Graph flow;
        auto&     src = flow.emplaceBlock<gr::blocks::testing::TagSource<CF>>({{"n_samples_max", gr::Size_t{65536U}}, {"mark_tag", false}, {"values", gr::Tensor<CF>(gr::data_from, tone(64UZ, 0.25))}});
        src._tags.emplace_back(0UZ, gr::property_map{{gr::property_map::key_type{"sample_rate"}, gr::pmt::Value(kRate)}});
        src._tags.emplace_back(30000UZ, gr::property_map{{gr::property_map::key_type{"sample_rate"}, gr::pmt::Value(2.f * kRate)}});
        auto& comp = flow.emplaceBlock<FrequencyCompressor>({{"divisor", kDivisor}, {"frame", gr::Size_t(1024)}, {"sample_rate", kRate}});
        auto& sink = flow.emplaceBlock<gr::blocks::testing::TagSink<CF, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>({});
        expect(flow.connect<"out", "in">(src, comp).has_value());
        expect(flow.connect<"out", "in">(comp, sink).has_value());

        gr::scheduler::Simple<> scheduler;
        expect(scheduler.exchange(std::move(flow)).has_value());
        expect(scheduler.runAndWait().has_value());

        expect(gt(sink._samples.size(), 0UZ));
        expect(eq(comp.sample_rate.value, kRate)) << "an upstream rate tag does not set sample_rate";
        const auto rates = ratesIn(sink._tags);
        expect(eq(rates.size(), 1UZ)) << "one rate reaches the output";
        if (rates.size() == 1UZ) {
            expect(eq(rates[0], kRate / static_cast<float>(kDivisor))) << "the block's own output rate";
        }
    };

    "a non-finite input sample leaves the output finite"_test = [] {
        FrequencyCompressor block({{"divisor", gr::Size_t(4)}, {"frame", gr::Size_t(256)}});
        bringUp(block);

        auto input          = tone(1UZ << 14, 0.1);
        input[5000]         = CF(std::numeric_limits<float>::quiet_NaN(), 0.f);
        input[5001]         = CF(0.f, std::numeric_limits<float>::infinity());
        const auto isFinite = [](const CF& sample) { return std::isfinite(sample.real()) && std::isfinite(sample.imag()); };

        std::vector<CF> output(1UZ << 14);
        const Call      dirty = drive(block, input, output);
        expect(gt(dirty.made, 0UZ));
        expect(std::ranges::all_of(std::span<const CF>(output).first(dirty.made), isFinite)) << "the call that took the bad samples";

        const auto clean = tone(1UZ << 14, 0.1);
        const Call after = drive(block, clean, output);
        expect(gt(after.made, 0UZ));
        expect(std::ranges::all_of(std::span<const CF>(output).first(after.made), isFinite)) << "and the clean call after it";
    };

    "the divisor moves while the block runs, re-plans, and states the new rate"_test = [] {
        constexpr float     kRate = 48000.f;
        FrequencyCompressor block({{"divisor", gr::Size_t(4)}, {"frame", gr::Size_t(1024)}, {"sample_rate", kRate}});
        bringUp(block);

        const auto      input = tone(1UZ << 16, 0.05);
        std::vector<CF> output(1UZ << 16);

        // run until the first samples come out, which is where the block states its configured rate
        const Call first = driveToOutput(block, input, output);
        expect(first.made > 0UZ) << "the block produces output at divisor 4";
        const auto firstRates = ratesIn(first.tags);
        expect(eq(firstRates.size(), 1UZ)) << "and states the rate it was configured for";
        if (firstRates.size() == 1UZ) {
            expect(approx(firstRates[0], kRate / 4.f, 1e-3f));
        }

        expect(applyLive(block, {{"divisor", gr::Size_t(8)}}).empty());
        expect(eq(block.divisor.value, gr::Size_t(8))) << "the change is accepted rather than refused";

        const Call after = driveToOutput(block, input, output);
        expect(after.made > 0UZ) << "and the block keeps producing under the new divisor";
        const auto afterRates = ratesIn(after.tags);
        expect(eq(afterRates.size(), 1UZ)) << "the new rate is stated once, on the first sample after the change";
        if (afterRates.size() == 1UZ) {
            expect(approx(afterRates[0], kRate / 8.f, 1e-3f)) << "which is the input rate over the new divisor";
        }

        const Call quiet = drive(block, input, output);
        expect(quiet.tags.empty()) << "and only once: a rate that has not moved is not restated";
    };

    // A sink that takes sixteen samples leaves most of a call's output queued in the block when the divisor moves.
    "a live divisor change keeps the output already made for a slow sink"_test = [] {
        constexpr float       kRate  = 48000.f;
        constexpr std::size_t kFrame = 1024UZ;
        const auto            input  = tone(1UZ << 16, 0.05);
        std::vector<CF>       output(1UZ << 16);

        FrequencyCompressor unchanged({{"divisor", gr::Size_t(4)}, {"frame", gr::Size_t(kFrame)}, {"sample_rate", kRate}});
        bringUp(unchanged);
        const Call reference = drive(unchanged, input, output);
        expect(fatal(eq(reference.made, completedOutputs(input.size(), kFrame, 4UZ))));
        const std::vector<CF> made(output.begin(), output.begin() + static_cast<std::ptrdiff_t>(reference.made));

        for (const gr::Size_t next : {gr::Size_t(8), gr::Size_t(1)}) {
            FrequencyCompressor block({{"divisor", gr::Size_t(4)}, {"frame", gr::Size_t(kFrame)}, {"sample_rate", kRate}});
            bringUp(block);
            std::vector<CF> got(made.size());
            const Call      first = drive(block, input, std::span<CF>(got).first(16UZ));
            expect(fatal(eq(first.made, 16UZ)));
            expect(applyLive(block, {{"divisor", next}}).empty());

            const Call queued = drive(block, input, std::span<CF>(got).subspan(16UZ));
            expect(eq(queued.consumed, 0UZ)) << std::format("divisor {}: the queued output goes out before new input is taken", next);
            expect(eq(queued.made, made.size() - 16UZ)) << std::format("divisor {}: every queued sample", next);
            expect(that % (got == made)) << std::format("divisor {}: the sink receives what an unchanged block makes", next);
            expect(ratesIn(queued.tags).empty()) << std::format("divisor {}: the queued samples keep the rate already stated", next);

            const Call fresh = driveToOutput(block, input, output);
            expect(gt(fresh.consumed, 0UZ));
            expect(that % (ratesIn(fresh.tags) == std::vector<float>{kRate / static_cast<float>(next)})) << std::format("divisor {}: the new rate starts with the new plan's output", next);
        }
    };

    "the frame moves while the block runs, and says nothing about the rate"_test = [] {
        FrequencyCompressor block({{"divisor", gr::Size_t(4)}, {"frame", gr::Size_t(1024)}, {"sample_rate", 48000.f}});
        bringUp(block);

        const auto      input = tone(1UZ << 16, 0.05);
        std::vector<CF> output(1UZ << 16);
        for (std::size_t call = 0UZ; call < 8UZ; ++call) {
            std::ignore = drive(block, input, output);
        }

        expect(applyLive(block, {{"frame", gr::Size_t(2048)}}).empty());
        expect(eq(block.frame.value, gr::Size_t(2048))) << "a new analysis frame is accepted while running";

        const Call next = driveToOutput(block, input, output);
        expect(next.made > 0UZ) << "the block re-plans and keeps producing";
        expect(next.tags.empty()) << "a frame change moves the resolution and states no rate";
    };

    "the output rate follows the divisor across a live change"_test = [] {
        const auto      input = tone(1UZ << 16, 0.05);
        std::vector<CF> output(1UZ << 16);

        const auto ratioAt = [&](gr::Size_t first, gr::Size_t second) {
            FrequencyCompressor block({{"divisor", first}, {"frame", gr::Size_t(1024)}});
            bringUp(block);
            if (second != first) {
                for (std::size_t call = 0UZ; call < 4UZ; ++call) {
                    std::ignore = drive(block, input, output);
                }
                expect(applyLive(block, {{"divisor", second}}).empty());
            }
            std::size_t made = 0UZ;
            std::size_t took = 0UZ;
            for (std::size_t call = 0UZ; call < 24UZ; ++call) {
                const Call c = drive(block, input, output);
                made += c.made;
                took += c.consumed;
            }
            return took > 0UZ ? static_cast<double>(made) / static_cast<double>(took) : 0.;
        };

        expect(std::abs(ratioAt(gr::Size_t(4), gr::Size_t(4)) - 0.25) < 0.02) << "a divisor of four produces a quarter of the samples";
        expect(std::abs(ratioAt(gr::Size_t(4), gr::Size_t(8)) - 0.125) < 0.02) << "and after a change to eight, an eighth of them";
    };
};

int main() { /* not needed for UT */ }
