#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <memory>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/filter/Channelizer.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

#include "StreamEndSink.hpp"

namespace {

using namespace std::string_literals;

using CF = std::complex<float>;
using CD = std::complex<double>;

using gr::blocks::filter::PolyphaseChannelizer;
using gr::blocks::filter::PolyphaseSynthesizer;
using gr::blocks::testing::ProcessFunction;
using gr::blocks::testing::TagSink;
using gr::blocks::testing::TagSource;

namespace test = gr::blocks::testing::span;

using ChannelSink = TagSink<CF, ProcessFunction::USE_PROCESS_ONE>;

/// A block brought up the way the framework brings one up, held by pointer so a refused set-up leaves nothing behind.
template<typename TBlock>
[[nodiscard]] std::unique_ptr<TBlock> make(gr::property_map settings) {
    auto block = std::make_unique<TBlock>(std::move(settings));
    block->settings().init();
    std::ignore = block->settings().applyStagedParameters();
    block->start();
    return block;
}

/// Restages settings on a block that is already running, the way a live change arrives.
template<typename TBlock>
void apply(TBlock& block, gr::property_map settings) {
    std::ignore = block.settings().setStaged(std::move(settings));
    std::ignore = block.settings().applyStagedParameters();
}

/// Stages @p settings and applies them. Returns the refusal, empty when the block took every value.
template<typename TBlock>
[[nodiscard]] std::string refusalOf(TBlock& block, gr::property_map settings) {
    try {
        std::ignore = block.settings().setStaged(std::move(settings));
        return block.settings().applyStagedParameters().refusal;
    } catch (const gr::exception& error) {
        return error.message;
    }
}

/**
 * @brief One period of the tone at `spacings / over` channel spacings of a `channels`-wide bank.
 *
 * A rational offset keeps the period finite. A cycling source then repeats the tone exactly. An `over` of one puts
 * the tone at a channel center. Any other value offsets it. The offset separates a family that concentrates a
 * bin-centered tone from one that isolates a band.
 */
[[nodiscard]] std::vector<CF> tonePeriod(std::size_t channels, std::size_t spacings, std::size_t over = 1UZ) {
    const std::size_t length = channels * over;
    std::vector<CF>   period(length);
    for (std::size_t n = 0UZ; n < length; ++n) {
        const double angle = 2.0 * std::numbers::pi * static_cast<double>((spacings * n) % length) / static_cast<double>(length);
        period[n]          = CF(static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle)));
    }
    return period;
}

/// A deterministic broadband sequence that fills every channel, so a reconstruction figure repeats run to run.
[[nodiscard]] std::vector<CF> broadband(std::size_t nSamples) {
    std::vector<CF> samples(nSamples);
    std::uint64_t   state = 0x2545f4914f6cdd1dULL;
    for (CF& sample : samples) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        const double re = static_cast<double>(state >> 40U) / 8388608.0 - 1.0;
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        const double im = static_cast<double>(state >> 40U) / 8388608.0 - 1.0;
        sample          = CF(static_cast<float>(re), static_cast<float>(im));
    }
    return samples;
}

/// The mean power of a channel over the samples past `skip`, by which point the prototype has filled.
[[nodiscard]] double meanPower(std::span<const CF> samples, std::size_t skip) {
    if (samples.size() <= skip) {
        return 0.0;
    }
    double sum = 0.0;
    for (std::size_t i = skip; i < samples.size(); ++i) {
        sum += static_cast<double>(std::norm(samples[i]));
    }
    return sum / static_cast<double>(samples.size() - skip);
}

/**
 * @brief The worst channel at least two away from `bin`, relative to `bin` itself, in dB.
 *
 * Two away, the bank's isolation is its own and not the crossover it shares with its neighbor. The distance wraps
 * because channel `M-1` sits next to channel zero.
 */
[[nodiscard]] double leakageDb(std::span<const std::vector<CF>> samples, std::size_t bin, std::size_t skip) {
    const double selected = meanPower(samples[bin], skip);
    double       worst    = 0.0;
    for (std::size_t channel = 0UZ; channel < samples.size(); ++channel) {
        const std::size_t apart = channel > bin ? channel - bin : bin - channel;
        if (std::min(apart, samples.size() - apart) >= 2UZ) {
            worst = std::max(worst, meanPower(samples[channel], skip));
        }
    }
    return 10.0 * std::log10(worst / selected);
}

/// The output offsets of the tags in `tags` that carry `key`.
[[nodiscard]] std::vector<std::size_t> offsetsOf(std::span<const gr::Tag> tags, std::string_view key) {
    std::vector<std::size_t> offsets;
    for (const gr::Tag& tag : tags) {
        if (tag.map.contains(typename gr::property_map::key_type{key})) {
            offsets.push_back(tag.index);
        }
    }
    return offsets;
}

/// Everything the sinks of one channelizer run collected.
struct BankRun {
    std::vector<std::vector<CF>>      samples{};
    std::vector<std::vector<gr::Tag>> tags{};
};

/**
 * @brief Runs a channelizer in a graph with a source cycling `values`, the bank, and one sink per channel.
 *
 * The channel count comes from the bank's own port vector after the settings are applied. A graph that wires a
 * runtime-sized collection has only that count to work with.
 */
[[nodiscard]] BankRun runInGraph(gr::property_map settings, std::span<const CF> values, gr::Size_t nSamples, std::span<const gr::Tag> inputTags = {}) {
    using namespace boost::ut;

    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<CF>>({{"n_samples_max", nSamples}, {"mark_tag", false}, {"values", gr::Tensor<CF>(gr::data_from, values)}});
    source._tags.assign(inputTags.begin(), inputTags.end());

    auto& bank = graph.emplaceBlock<PolyphaseChannelizer<float>>(std::move(settings));
    expect(graph.connect(source, "out"s, bank, "in"s).has_value()) << "the single input connects";

    const std::size_t         channels = bank.out.size();
    std::vector<ChannelSink*> sinks;
    for (std::size_t channel = 0UZ; channel < channels; ++channel) {
        sinks.push_back(std::addressof(graph.emplaceBlock<ChannelSink>({})));
        expect(graph.connect(bank, "out#"s + std::to_string(channel), *sinks[channel], "in"s).has_value()) << std::format("output {} of {} connects", channel, channels);
    }

    gr::scheduler::Simple<> scheduler;
    expect(scheduler.exchange(std::move(graph)).has_value());
    expect(scheduler.runAndWait().has_value());

    BankRun run;
    for (const ChannelSink* sink : sinks) {
        run.samples.emplace_back(sink->_samples.begin(), sink->_samples.end());
        run.tags.push_back(sink->_tags);
    }
    return run;
}

/// What one cascade produced, with the length of the prototype both halves ran.
struct CascadeRun {
    std::vector<CF> samples{};
    std::size_t     prototypeLength = 0UZ;
};

/// Runs a channelizer straight into a synthesizer of the same shape and returns what the far end produced.
[[nodiscard]] CascadeRun runRoundTrip(const gr::property_map& settings, std::span<const CF> values, gr::Size_t nSamples) {
    using namespace boost::ut;

    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<CF>>({{"n_samples_max", nSamples}, {"mark_tag", false}, {"values", gr::Tensor<CF>(gr::data_from, values)}});
    auto&     bank   = graph.emplaceBlock<PolyphaseChannelizer<float>>(settings);
    auto&     synth  = graph.emplaceBlock<PolyphaseSynthesizer<float>>(settings);
    auto&     sink   = graph.emplaceBlock<ChannelSink>({});

    expect(graph.connect(source, "out"s, bank, "in"s).has_value());
    const std::size_t channels = bank.out.size();
    expect(eq(synth.in.size(), channels)) << "the two banks agree on their port counts";
    for (std::size_t channel = 0UZ; channel < channels; ++channel) {
        expect(graph.connect(bank, "out#"s + std::to_string(channel), synth, "in#"s + std::to_string(channel)).has_value()) << std::format("channel {} crosses the cascade", channel);
    }
    expect(graph.connect(synth, "out"s, sink, "in"s).has_value());

    const std::size_t prototypeLength = bank.designed_taps.value.size();

    gr::scheduler::Simple<> scheduler;
    expect(scheduler.exchange(std::move(graph)).has_value());
    expect(scheduler.runAndWait().has_value());

    return CascadeRun{std::vector<CF>(sink._samples.begin(), sink._samples.end()), prototypeLength};
}

/**
 * @brief The reconstruction residual of `output` against `input` in dB, with the best lag and complex gain fitted out.
 *
 * A cascade of two banks delays by the prototype's span and scales by the cascade's own gain, neither of which is an
 * error. The residual is the error left after both are removed, taken over a window clear of the pipeline's fill
 * and flush at either end.
 */
[[nodiscard]] double reconstructionDb(std::span<const CF> input, std::span<const CF> output, std::size_t maxLag) {
    const std::size_t guard = maxLag + 64UZ;
    if (input.size() <= 2UZ * guard || output.size() < input.size()) {
        return 0.0; // too short to fit a window, reported as a residual no target accepts
    }
    const std::size_t first = guard;
    const std::size_t count = input.size() - 2UZ * guard;

    double reference = 0.0;
    for (std::size_t i = 0UZ; i < count; ++i) {
        reference += static_cast<double>(std::norm(input[first + i]));
    }

    double best = 1.0;
    for (std::size_t lag = 0UZ; lag <= maxLag; ++lag) {
        CD correlation{};
        for (std::size_t i = 0UZ; i < count; ++i) {
            correlation += CD(output[first + i + lag]) * std::conj(CD(input[first + i]));
        }
        const CD gain = correlation / reference;
        if (std::abs(gain) == 0.0) {
            continue;
        }
        double residual = 0.0;
        for (std::size_t i = 0UZ; i < count; ++i) {
            residual += std::norm(CD(output[first + i + lag]) - gain * CD(input[first + i]));
        }
        best = std::min(best, residual / (std::norm(gain) * reference));
    }
    return 10.0 * std::log10(std::max(best, 1.0e-30));
}

/**
 * @brief Drives the bank's `processBulk` over `input`, taking the chunk sizes from `chunks` in turn.
 *
 * Each chunk is truncated to a whole number of commutator steps. The framework passes the same chunks under
 * `input_chunk_size`. An empty `chunks` presents the whole input in one call.
 */
[[nodiscard]] std::vector<std::vector<CF>> drive(PolyphaseChannelizer<float>& bank, std::span<const CF> input, std::span<const std::size_t> chunks) {
    const std::size_t channels = bank._channels;
    const std::size_t stride   = bank._stride;

    std::vector<std::vector<CF>> collected(channels);
    std::vector<std::vector<CF>> scratch(channels);
    std::size_t                  at   = 0UZ;
    std::size_t                  call = 0UZ;

    while (at < input.size()) {
        const std::size_t want = chunks.empty() ? input.size() : chunks[call % chunks.size()];
        const std::size_t take = (std::min(want, input.size() - at) / stride) * stride;
        if (take == 0UZ) {
            break;
        }
        ++call;

        std::vector<test::OutputSpan<CF>> outSpans;
        for (std::size_t channel = 0UZ; channel < channels; ++channel) {
            scratch[channel].assign(take / stride, CF{});
            outSpans.emplace_back(std::span<CF>(scratch[channel]), collected[channel].size());
        }

        test::InputSpan<CF>             inSpan(input.subspan(at, take), at);
        std::span<test::OutputSpan<CF>> outputs(outSpans);
        std::ignore = bank.processBulk(inSpan, outputs);

        for (std::size_t channel = 0UZ; channel < channels; ++channel) {
            collected[channel].insert(collected[channel].end(), scratch[channel].begin(), scratch[channel].begin() + static_cast<std::ptrdiff_t>(outSpans[channel].count));
        }
        at += inSpan.consumed;
    }
    return collected;
}

} // namespace

const boost::ut::suite<"Channelizer"> channelizerTests = [] {
    using namespace boost::ut;

    "a graph of vector outputs runs, every channel takes its share, and the tone lands on one of them"_test = [] {
        for (const auto& [channels, bin] : std::vector<std::pair<gr::Size_t, std::size_t>>{{4U, 1UZ}, {16U, 3UZ}}) {
            const std::size_t width    = static_cast<std::size_t>(channels);
            const gr::Size_t  nSamples = 128U * channels;
            const auto        run      = runInGraph({{"n_channels", channels}}, tonePeriod(width, bin), nSamples);

            expect(eq(run.samples.size(), width)) << std::format("a {}-channel bank offers {} sinks their own port", width, width);
            for (std::size_t channel = 0UZ; channel < width; ++channel) {
                expect(eq(run.samples[channel].size(), static_cast<std::size_t>(nSamples) / width)) << std::format("channel {} of {} received one sample per commutator step", channel, width);
            }

            const std::size_t settled  = run.samples[bin].size() / 4UZ;
            const double      selected = meanPower(run.samples[bin], settled);
            expect(gt(selected, 0.9) && lt(selected, 1.1)) << std::format("the tone arrives at unity on channel {} of {}", bin, width);

            double worst = 0.0;
            for (std::size_t channel = 0UZ; channel < width; ++channel) {
                if (channel != bin) {
                    worst = std::max(worst, meanPower(run.samples[channel], settled));
                }
            }
            // The default root-Nyquist family trades isolation for the flat sum a cascade needs. Its rejection narrows
            // with the bank, 61 dB at four channels against 68 at sixteen. The gate clears the narrower of them.
            expect(lt(10.0 * std::log10(worst / selected), -55.0)) << std::format("a {}-channel bank leaves the tone out of every other channel", width);
        }
    };

    "designed_taps holds the prototype that is running, and taps holds only what was supplied"_test = [] {
        constexpr gr::Size_t channels = 4U;

        // A bank asked for nothing but its width runs the root-Nyquist design, which is the family that also reconstructs.
        {
            const auto bank     = make<PolyphaseChannelizer<float>>({{"n_channels", channels}});
            const auto designed = gr::filter::fir::design::rootRaisedCosine(static_cast<int>(16U * channels) - 1, static_cast<double>(channels), 0.5, 1.0);

            expect(eq(bank->prototype.value, std::string("root_nyquist"))) << "the default family";
            expect(eq(bank->designed_taps.value.size(), static_cast<std::size_t>(16U * channels))) << "a span of sixteen spacings over four branches runs sixty-four taps";
            expect(std::ranges::equal(std::span(bank->designed_taps.value).first(designed.size()), designed)) << "at the default span of sixteen and rolloff of a half";
            expect(bank->taps.value.empty()) << "and with nothing in the supplied setting";
            // This family's length is the caller's span, not a search against attenuation_db. It is measured and not
            // aimed, and design_ok has nothing to fail against. stopband_db holds what the span achieved.
            expect(bank->design_ok.value) << "a family that was never given a target reports no failure to meet one";
            expect(lt(bank->stopband_db.value, 0.0)) << "and reports the rejection its span did achieve";
        }

        // Asked for the lowpass family, the observable holds the searched Kaiser the library designs.
        {
            const auto  design   = gr::filter::designChannelizerPrototype(static_cast<std::size_t>(channels), "lowpass", 80.0, 0.2, 16UZ, 1UZ).value();
            const auto  bank     = make<PolyphaseChannelizer<float>>({{"n_channels", channels}, {"prototype", std::string("lowpass")}, {"attenuation_db", 80.0}, {"transition", 0.2}});
            const auto  expected = ((design.taps.size() + channels - 1UZ) / channels) * channels;
            const auto& running  = bank->designed_taps.value;

            expect(eq(running.size(), expected)) << "the running prototype is the design zero-padded to a whole number of branches";
            expect(std::ranges::equal(std::span(running).first(design.taps.size()), design.taps)) << "and holds the library designer's own taps";
            expect(std::ranges::all_of(std::span(running).subspan(design.taps.size()), [](float tap) { return tap == 0.f; })) << "the padding is zero";
            expect(bank->taps.value.empty()) << "a design leaves the supplied prototype empty, which is what makes the design settings live";

            expect(bank->design_ok.value) << "the lowpass search reaches the target it was given";
            expect(le(bank->stopband_db.value, -80.0)) << "and the level it measured is at or past the eighty decibels asked for";
            expect(eq(bank->stopband_db.value, design.stopbandDb)) << "the block reports what the library measured";
            expect(eq(bank->ripple_db.value, design.rippleDb));
        }

        // The lowpass family is the one with a target, and it meets the default target on the path a caller takes to it.
        {
            for (const gr::Size_t width : {gr::Size_t(4), gr::Size_t(16)}) {
                const auto bank = make<PolyphaseChannelizer<float>>({{"n_channels", width}, {"prototype", std::string("lowpass")}, {"transition", 0.2}});
                expect(eq(bank->attenuation_db.value, 60.0)) << "the default stopband target";
                expect(bank->design_ok.value) << std::format("a {}-channel lowpass reaches the default target", width);
                expect(le(bank->stopband_db.value, -60.0)) << std::format("and measures at or past it, at {} channels", width);
            }
        }

        // A narrow transition needs a prototype longer than the search's starting estimate. The search must then grow
        // and cannot bisect down from the estimate.
        {
            const auto bank = make<PolyphaseChannelizer<float>>({{"n_channels", channels}, {"prototype", std::string("lowpass")}, {"transition", 0.05}, {"attenuation_db", 80.0}});
            expect(bank->design_ok.value) << "the search grows to meet a target its estimate falls short of";
            expect(le(bank->stopband_db.value, -80.0)) << "and lands at or past the eighty decibels asked for";
        }

        // The alias band depends on the commutator stride. Decimating by M/oversample folds every multiple of
        // oversample/M onto baseband. A stride of M/2 puts the stop edge a whole spacing further out, where the same
        // prototype is far lower. One filter gives two measurements.
        {
            constexpr gr::Size_t wide = 16U;

            const auto critical    = make<PolyphaseChannelizer<float>>({{"n_channels", wide}});
            const auto oversampled = make<PolyphaseChannelizer<float>>({{"n_channels", wide}, {"oversample", gr::Size_t(2)}});

            const auto criticalDesign    = gr::filter::designChannelizerPrototype(static_cast<std::size_t>(wide), "root_nyquist", 60.0, 0.5, 16UZ, 1UZ).value();
            const auto oversampledDesign = gr::filter::designChannelizerPrototype(static_cast<std::size_t>(wide), "root_nyquist", 60.0, 0.5, 16UZ, 2UZ).value();

            expect(std::ranges::equal(critical->designed_taps.value, oversampled->designed_taps.value)) << "the stride does not change the prototype";
            expect(eq(critical->stopband_db.value, criticalDesign.stopbandDb)) << "critically sampled the block reports the library's figure for a stride of M";
            expect(eq(oversampled->stopband_db.value, oversampledDesign.stopbandDb)) << "oversampled by two it reports the figure for a stride of M/2";
            expect(lt(oversampled->stopband_db.value, critical->stopband_db.value - 25.0)) << "which is where the second commutator stride earns its keep";
            expect(critical->design_ok.value && oversampled->design_ok.value) << "neither was aimed at a target, so neither reports missing one";
        }

        // A harder lowpass is a longer one, in both of the settings that drive its length.
        {
            const auto base    = make<PolyphaseChannelizer<float>>({{"n_channels", channels}, {"prototype", std::string("lowpass")}, {"attenuation_db", 80.0}, {"transition", 0.2}});
            const auto sharper = make<PolyphaseChannelizer<float>>({{"n_channels", channels}, {"prototype", std::string("lowpass")}, {"attenuation_db", 80.0}, {"transition", 0.05}});
            const auto deeper  = make<PolyphaseChannelizer<float>>({{"n_channels", channels}, {"prototype", std::string("lowpass")}, {"attenuation_db", 120.0}, {"transition", 0.2}});
            expect(gt(sharper->designed_taps.value.size(), base->designed_taps.value.size())) << "a narrower transition designs a longer prototype";
            expect(gt(deeper->designed_taps.value.size(), base->designed_taps.value.size())) << "a deeper stopband designs a longer prototype";
        }

        // The root-Nyquist family takes its length from the span it covers instead, and its shape from the excess bandwidth.
        {
            constexpr gr::Size_t span = 16U;

            const auto  bank     = make<PolyphaseChannelizer<float>>({{"n_channels", channels}, {"prototype", std::string("root_nyquist")}, {"span", span}, {"transition", 0.5}});
            const auto  designed = gr::filter::fir::design::rootRaisedCosine(static_cast<int>(span * channels) - 1, static_cast<double>(channels), 0.5, 1.0);
            const auto& running  = bank->designed_taps.value;

            expect(eq(running.size(), static_cast<std::size_t>(span * channels))) << "a span of sixteen spacings over four branches runs sixty-four taps";
            expect(std::ranges::equal(std::span(running).first(designed.size()), designed)) << "and holds the designer's own taps";
            expect(bank->taps.value.empty()) << "the second family designs on the same terms";

            const auto wider = make<PolyphaseChannelizer<float>>({{"n_channels", channels}, {"prototype", std::string("root_nyquist")}, {"span", gr::Size_t(32)}, {"transition", 0.5}});
            expect(gt(wider->designed_taps.value.size(), running.size())) << "a wider span designs a longer prototype";
        }

        // Supplied, the prototype runs zero-padded to a whole number of branches and the setting keeps the taps as they arrived.
        {
            const std::vector<float> supplied{0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.4f, 0.3f, 0.2f, 0.1f, 0.05f};
            const auto               bank    = make<PolyphaseChannelizer<float>>({{"n_channels", channels}, {"taps", supplied}});
            const auto&              running = bank->designed_taps.value;

            expect(eq(running.size(), 12UZ)) << "ten taps across four branches pad to twelve";
            expect(std::ranges::equal(std::span(running).first(supplied.size()), supplied)) << "the supplied taps arrive unchanged";
            expect(std::ranges::all_of(std::span(running).subspan(supplied.size()), [](float tap) { return tap == 0.f; })) << "the padding is zero";
            expect(std::ranges::equal(bank->taps.value, supplied)) << "and the setting still reads back exactly the ten that were supplied";

            // A supplied prototype is measured on the same terms as a designed one. It has no target. The two figures
            // are the response of the supplied vector at the alias edge the stride implies.
            const auto measured = gr::filter::measureChannelizerPrototype(supplied, static_cast<std::size_t>(channels), 0.5, 1UZ).value();
            expect(bank->design_ok.value) << "a supplied prototype is not reported as missing a target it never had";
            expect(eq(bank->stopband_db.value, measured.stopbandDb)) << "the block reports what the library measured of the taps it was handed";
            expect(eq(bank->ripple_db.value, measured.rippleDb));

            // The stride moves the alias edge for a supplied prototype exactly as it does for a designed one.
            const auto strided = make<PolyphaseChannelizer<float>>({{"n_channels", channels}, {"taps", supplied}, {"oversample", gr::Size_t(2)}});
            expect(lt(strided->stopband_db.value, bank->stopband_db.value - 3.0)) << "the same taps measure lower against the wider stop edge";
        }
    };

    "a design setting changed on a running block redesigns, and designed_taps follows"_test = [] {
        constexpr gr::Size_t channels = 16U;

        auto bank = make<PolyphaseChannelizer<float>>({{"n_channels", channels}});

        const auto lengthOf = [](const gr::property_map& settings) {
            const auto fresh = make<PolyphaseChannelizer<float>>(settings);
            return fresh->designed_taps.value.size();
        };

        // Started on the defaults, the bank runs the root-raised cosine its span asks for.
        const std::size_t asRootNyquist = bank->designed_taps.value.size();
        expect(eq(asRootNyquist, static_cast<std::size_t>(16U * channels))) << "the default family takes its length from the span";

        // The family switches, and the bank runs the Kaiser its transition and attenuation ask for instead.
        apply(*bank, {{"prototype", std::string("lowpass")}, {"attenuation_db", 80.0}, {"transition", 0.2}});
        const std::size_t asLowpass = bank->designed_taps.value.size();
        expect(neq(asLowpass, asRootNyquist)) << "the live change reached the running bank";
        expect(eq(asLowpass, lengthOf({{"n_channels", channels}, {"prototype", std::string("lowpass")}, {"attenuation_db", 80.0}, {"transition", 0.2}}))) << "and left it where a fresh block at those settings starts";

        // In the lowpass family the transition is a width, and a narrower one designs a longer filter.
        apply(*bank, {{"transition", 0.05}});
        expect(gt(bank->designed_taps.value.size(), asLowpass)) << "a narrower transition lengthens the running prototype";
        apply(*bank, {{"transition", 0.2}, {"attenuation_db", 120.0}});
        expect(gt(bank->designed_taps.value.size(), asLowpass)) << "so does a deeper stopband";

        // The boxcar family is one channel long and reads neither of the two settings the other families design from.
        apply(*bank, {{"prototype", std::string("boxcar")}});
        expect(eq(bank->designed_taps.value.size(), static_cast<std::size_t>(channels))) << "the boxcar prototype is exactly one channel long";
        const float flat = 1.f / static_cast<float>(channels);
        expect(std::ranges::all_of(bank->designed_taps.value, [flat](float tap) { return tap == flat; })) << "and flat across it at unit gain";
        apply(*bank, {{"attenuation_db", 60.0}, {"span", gr::Size_t(8)}});
        expect(eq(bank->designed_taps.value.size(), static_cast<std::size_t>(channels))) << "neither the stopband target nor the span moves it";

        // Back in the root-Nyquist family, the span is live.
        apply(*bank, {{"prototype", std::string("root_nyquist")}, {"span", gr::Size_t(16)}, {"transition", 0.5}});
        expect(eq(bank->designed_taps.value.size(), asRootNyquist)) << "the family switches back to the design it started on";
        apply(*bank, {{"span", gr::Size_t(32)}});
        expect(eq(bank->designed_taps.value.size(), static_cast<std::size_t>(32U * channels))) << "a wider span lengthens the running prototype";

        // The root-Nyquist family sets its rejection by the span it covers and does not read the stopband target.
        const std::vector<float> beforeAttenuation = bank->designed_taps.value;
        apply(*bank, {{"attenuation_db", 40.0}});
        expect(std::ranges::equal(bank->designed_taps.value, beforeAttenuation)) << "a stopband target does not reach the root-Nyquist design";
        apply(*bank, {{"attenuation_db", 120.0}});
        expect(std::ranges::equal(bank->designed_taps.value, beforeAttenuation)) << "in either direction";

        // So is the transition, which is the rolloff here and changes the shape at a fixed length.
        const std::vector<float> beforeRolloff = bank->designed_taps.value;
        apply(*bank, {{"transition", 0.25}});
        expect(eq(bank->designed_taps.value.size(), beforeRolloff.size())) << "a different rolloff keeps the span's length";
        expect(!std::ranges::equal(bank->designed_taps.value, beforeRolloff)) << "and changes the taps";

        expect(bank->taps.value.empty()) << "none of it wrote a prototype into the supplied setting";

        // A supplied prototype outranks the design settings. Clearing it gives control back to them.
        const std::vector<float> supplied(64UZ, 0.5f);
        apply(*bank, {{"taps", supplied}});
        expect(eq(bank->designed_taps.value.size(), supplied.size())) << "the supplied prototype runs";
        apply(*bank, {{"transition", 0.5}});
        expect(std::ranges::equal(std::span(bank->designed_taps.value).first(supplied.size()), supplied)) << "and a design setting does not displace it";
        apply(*bank, {{"taps", std::vector<float>{}}});
        expect(gt(bank->designed_taps.value.size(), supplied.size())) << "clearing it returns the bank to its designed prototype";
    };

    "an input tag lands on channel zero at its own step, and nowhere else"_test = [] {
        constexpr gr::Size_t channels = 4U;
        constexpr gr::Size_t nSamples = 4096U;
        const std::size_t    stride   = static_cast<std::size_t>(channels);

        const std::vector<std::size_t> at{400UZ, 1024UZ, 2000UZ};
        std::vector<gr::Tag>           inputTags;
        for (const std::size_t index : at) {
            inputTags.emplace_back(index, gr::property_map{{gr::property_map::key_type{"marker"}, gr::pmt::Value(static_cast<gr::Size_t>(index))}});
        }

        const auto run = runInGraph({{"n_channels", channels}}, tonePeriod(static_cast<std::size_t>(channels), 0UZ), nSamples, inputTags);

        const auto seen = offsetsOf(run.tags[0], "marker");
        expect(eq(seen.size(), at.size())) << "channel zero carries every input tag exactly once";
        for (std::size_t i = 0UZ; i < std::min(seen.size(), at.size()); ++i) {
            expect(eq(seen[i], at[i] / stride)) << std::format("the tag at input {} arrives at output {}", at[i], at[i] / stride);
        }

        for (std::size_t channel = 1UZ; channel < run.tags.size(); ++channel) {
            expect(eq(offsetsOf(run.tags[channel], "marker").size(), 0UZ)) << std::format("channel {} carries no copy of it", channel);
        }
    };

    "a forwarded sample_rate states the channel's rate, not the bank's input rate"_test = [] {
        constexpr gr::Size_t channels  = 4U;
        constexpr gr::Size_t nSamples  = 4096U;
        constexpr float      inputRate = 48000.f;

        for (const gr::Size_t oversample : {1U, 2U}) {
            const std::size_t stride      = static_cast<std::size_t>(channels) / static_cast<std::size_t>(oversample);
            const float       channelRate = inputRate / static_cast<float>(stride);

            std::vector<gr::Tag> inputTags;
            inputTags.emplace_back(512UZ, gr::property_map{{gr::property_map::key_type{"sample_rate"}, gr::pmt::Value(inputRate)}});

            const auto run = runInGraph({{"n_channels", channels}, {"oversample", oversample}}, tonePeriod(static_cast<std::size_t>(channels), 0UZ), nSamples, inputTags);

            const auto* forwarded = std::ranges::find_if(run.tags[0], [](const gr::Tag& entry) { return entry.map.contains(gr::property_map::key_type{"sample_rate"}); }).base();
            expect(forwarded != run.tags[0].data() + run.tags[0].size()) << std::format("oversample {}: channel zero carries the rate tag", oversample);
            if (forwarded == run.tags[0].data() + run.tags[0].size()) {
                continue;
            }

            const auto* rate = forwarded->map.at(gr::property_map::key_type{"sample_rate"}).get_if<float>();
            expect(rate != nullptr) << "the forwarded rate is still a float";
            if (rate != nullptr) {
                expect(eq(*rate, channelRate)) << std::format("oversample {}: a bank fed {} Hz runs each channel at {} Hz, so the tag reads {}", oversample, inputRate, channelRate, *rate);
            }
            expect(eq(forwarded->index, 512UZ / stride)) << "and it lands at the step the input sample fell in";
        }
    };

    "a cascade through the synthesizer reconstructs on the defaults, and each family sits where its trade puts it"_test = [] {
        constexpr gr::Size_t channels = 16U;
        constexpr gr::Size_t nSamples = 16384U;

        const auto input = broadband(static_cast<std::size_t>(nSamples));

        // The default root-Nyquist prototype sums flat across the bank, which is the condition the two halves cancel
        // under. Every setting behind the figure below is a default except the commutator stride reconstruction needs.
        {
            const auto run = runRoundTrip({{"n_channels", channels}, {"oversample", gr::Size_t(2)}}, input, nSamples);
            expect(eq(run.samples.size(), static_cast<std::size_t>(nSamples))) << "the cascade neither loses nor invents samples";
            expect(std::ranges::all_of(run.samples, [](const CF& sample) { return std::isfinite(sample.real()) && std::isfinite(sample.imag()); })) << "and stays finite throughout";

            const double residual = reconstructionDb(input, run.samples, 2UZ * run.prototypeLength);
            std::println("channelizer round trip, default prototype, oversampled by two: {:.1f} dB", residual);
            expect(lt(residual, -55.0)) << "the reconstruction residual clears -55 dB on the defaults alone";

            // Critically sampled, the same prototype aliases inside each channel and the pair leaves a residual.
            const auto   critical         = runRoundTrip({{"n_channels", channels}}, input, nSamples);
            const double criticalResidual = reconstructionDb(input, critical.samples, 2UZ * critical.prototypeLength);
            std::println("channelizer round trip, default prototype, critically sampled: {:.1f} dB", criticalResidual);
            expect(gt(criticalResidual, -30.0)) << "a critically sampled root-Nyquist pair carries a signal without reconstructing it";
        }

        // The lowpass family crosses over 6 dB down, where two neighbors sum to half the power. That 3 dB dip caps what
        // a cascade of two of them can return, however deep the stopband goes. The commutator stride is the same.
        {
            const auto run = runRoundTrip({{"n_channels", channels}, {"prototype", std::string("lowpass")}, {"transition", 0.2}, {"oversample", gr::Size_t(2)}}, input, nSamples);
            expect(eq(run.samples.size(), static_cast<std::size_t>(nSamples))) << "the lowpass cascade holds its rate too";
            expect(std::ranges::all_of(run.samples, [](const CF& sample) { return std::isfinite(sample.real()) && std::isfinite(sample.imag()); })) << "and stays finite throughout";

            const double residual = reconstructionDb(input, run.samples, 2UZ * run.prototypeLength);
            std::println("channelizer round trip, lowpass family, oversampled by two: {:.1f} dB", residual);
            expect(gt(residual, -30.0)) << "the lowpass family carries a signal through without reconstructing it";
        }

        // A prototype exactly one channel long makes the bank a block transform whose inverse is its own synthesis. The
        // cascade cancels to the arithmetic's floor without the commutator stride the other families need.
        {
            const auto run = runRoundTrip({{"n_channels", channels}, {"prototype", std::string("boxcar")}}, input, nSamples);
            expect(eq(run.prototypeLength, static_cast<std::size_t>(channels))) << "the prototype is one channel long";
            expect(eq(run.samples.size(), static_cast<std::size_t>(nSamples))) << "the critically sampled cascade holds its rate";
            expect(std::ranges::all_of(run.samples, [](const CF& sample) { return std::isfinite(sample.real()) && std::isfinite(sample.imag()); })) << "and stays finite throughout";

            const double residual = reconstructionDb(input, run.samples, 2UZ * run.prototypeLength);
            std::println("channelizer round trip, boxcar family, critically sampled: {:.1f} dB", residual);
            expect(lt(residual, -100.0)) << "the boxcar family is the one that reconstructs critically sampled";
        }

        // The cost is isolation. A tone a quarter of a spacing off center spreads across the bank, and the default
        // family holds it in. The two figures pin the trade from both ends.
        {
            constexpr std::size_t bin     = 4UZ;
            constexpr gr::Size_t  nTone   = 128U * channels;
            const auto            offset  = tonePeriod(static_cast<std::size_t>(channels), 4UZ * bin + 1UZ, 4UZ);
            const std::size_t     settled = static_cast<std::size_t>(nTone) / static_cast<std::size_t>(channels) / 4UZ;

            const auto   boxcar     = runInGraph({{"n_channels", channels}, {"prototype", std::string("boxcar")}}, offset, nTone);
            const double boxcarLeak = leakageDb(boxcar.samples, bin, settled);
            std::println("channelizer leakage two channels out, boxcar family: {:.1f} dB", boxcarLeak);
            expect(gt(boxcarLeak, -40.0)) << "the boxcar family isolates almost nothing, which is what it trades away";

            const auto   dflt     = runInGraph({{"n_channels", channels}}, offset, nTone);
            const double dfltLeak = leakageDb(dflt.samples, bin, settled);
            std::println("channelizer leakage two channels out, default prototype: {:.1f} dB", dfltLeak);
            expect(lt(dfltLeak, -55.0)) << "the default family holds an off-center tone inside its own channel";
            expect(lt(dfltLeak, boxcarLeak - 30.0)) << "and is at least 30 dB better at it than the boxcar";
        }
    };

    "the settings that cannot be built are refused"_test = [] {
        const auto refused = [](gr::property_map settings) { std::ignore = make<PolyphaseChannelizer<float>>(std::move(settings)); };

        expect(throws([&refused] { refused({{"n_channels", gr::Size_t(0)}}); })) << "a bank of no channels";
        expect(throws([&refused] { refused({{"n_channels", gr::Size_t(1)}}); })) << "a bank of one channel is not a bank";
        expect(throws([&refused] { refused({{"n_channels", gr::Size_t(257)}}); })) << "one channel past the widest bank";
        expect(throws([&refused] { refused({{"n_channels", gr::Size_t(4)}, {"oversample", gr::Size_t(0)}}); })) << "a commutator that does not advance";
        expect(throws([&refused] { refused({{"n_channels", gr::Size_t(4)}, {"oversample", gr::Size_t(3)}}); })) << "an oversampling factor the stride cannot take";
        expect(throws([&refused] { refused({{"n_channels", gr::Size_t(5)}, {"oversample", gr::Size_t(2)}}); })) << "an oversampled bank of odd width";
        expect(throws([&refused] { refused({{"n_channels", gr::Size_t(4)}, {"transition", 0.0}}); })) << "a transition of nothing";
        expect(throws([&refused] { refused({{"n_channels", gr::Size_t(4)}, {"transition", 1.0}}); })) << "a transition of a whole channel spacing";
        expect(throws([&refused] { refused({{"n_channels", gr::Size_t(4)}, {"attenuation_db", 0.0}}); })) << "a stopband target of zero";
        expect(throws([&refused] { refused({{"n_channels", gr::Size_t(4)}, {"attenuation_db", -20.0}}); })) << "a negative stopband target";
        expect(throws([&refused] { refused({{"n_channels", gr::Size_t(4)}, {"prototype", std::string("bandpass")}}); })) << "a family none of the designers offers";
        expect(throws([&refused] { refused({{"n_channels", gr::Size_t(4)}, {"prototype", std::string("root_nyquist")}, {"span", gr::Size_t(1)}}); })) << "a root-Nyquist span of one spacing";
        expect(throws([&refused] { refused({{"n_channels", gr::Size_t(4)}, {"prototype", std::string("root_nyquist")}, {"span", gr::Size_t(0)}}); })) << "a root-Nyquist span of nothing";

        // The refusal names every family a caller could have meant instead.
        std::string offered;
        try {
            refused({{"n_channels", gr::Size_t(4)}, {"prototype", std::string("bandpass")}});
        } catch (const std::exception& error) {
            offered = error.what();
        }
        for (const std::string_view family : {"root_nyquist", "lowpass", "boxcar"}) {
            expect(offered.find(family) != std::string::npos) << std::format("the refusal offers '{}'", family);
        }

        // The same shapes are refused by the synthesizer, which shares the settings and the checks.
        const auto refusedSynth = [](gr::property_map settings) { std::ignore = make<PolyphaseSynthesizer<float>>(std::move(settings)); };
        expect(throws([&refusedSynth] { refusedSynth({{"n_channels", gr::Size_t(1)}}); }));
        expect(throws([&refusedSynth] { refusedSynth({{"n_channels", gr::Size_t(5)}, {"oversample", gr::Size_t(2)}}); }));
        expect(throws([&refusedSynth] { refusedSynth({{"n_channels", gr::Size_t(4)}, {"prototype", std::string("bandpass")}}); }));
        expect(throws([&refusedSynth] { refusedSynth({{"n_channels", gr::Size_t(4)}, {"prototype", std::string("root_nyquist")}, {"span", gr::Size_t(1)}}); }));

        // A span below two is only the root-Nyquist designer's business, so the lowpass family takes one without complaint.
        const auto narrowSpan = make<PolyphaseChannelizer<float>>({{"n_channels", gr::Size_t(4)}, {"prototype", std::string("lowpass")}, {"span", gr::Size_t(1)}});
        expect(gt(narrowSpan->designed_taps.value.size(), 1UZ)) << "the lowpass design is unaffected by the span";

        // An accepted set-up plumbs the ports and takes the commutator stride its width and oversampling state.
        const auto oversampled = make<PolyphaseChannelizer<float>>({{"n_channels", gr::Size_t(8)}, {"oversample", gr::Size_t(2)}});
        expect(eq(oversampled->out.size(), 8UZ)) << "an accepted width plumbs that many ports";
        expect(eq(oversampled->_stride, 4UZ)) << "and an oversampled commutator advances half a bank";
    };

    "the split of a run into calls does not change its output"_test = [] {
        constexpr gr::Size_t channels = 8U;

        const auto input = broadband(8192UZ);

        auto       whole   = make<PolyphaseChannelizer<float>>({{"n_channels", channels}});
        const auto oneCall = drive(*whole, input, {});

        // Chunk sizes in commutator steps, uneven and cycling, so successive calls start at differing phases.
        const std::size_t              stride = whole->_stride;
        const std::vector<std::size_t> chunks{1UZ * stride, 13UZ * stride, 2UZ * stride, 57UZ * stride, 5UZ * stride, 3UZ * stride};

        auto       pieces      = make<PolyphaseChannelizer<float>>({{"n_channels", channels}});
        const auto manyCalls   = drive(*pieces, input, chunks);
        const auto expectedOut = input.size() / stride;

        expect(eq(oneCall.size(), static_cast<std::size_t>(channels)));
        expect(eq(manyCalls.size(), static_cast<std::size_t>(channels)));
        for (std::size_t channel = 0UZ; channel < static_cast<std::size_t>(channels); ++channel) {
            expect(eq(oneCall[channel].size(), expectedOut)) << std::format("one call fills channel {}", channel);
            expect(eq(manyCalls[channel].size(), expectedOut)) << std::format("many calls fill channel {} to the same length", channel);
            expect(std::ranges::equal(oneCall[channel], manyCalls[channel])) << std::format("channel {} is identical either way", channel);
        }
    };

    "each setting range is checked before a design, and the span bound is the root-Nyquist family's alone"_test = [] {
        const auto refusalAt = [](gr::property_map settings) {
            try {
                std::ignore = make<PolyphaseChannelizer<float>>(std::move(settings));
            } catch (const gr::exception& error) {
                return error.message;
            }
            return std::string{};
        };

        const std::string tooWide = refusalAt({{"n_channels", gr::Size_t(1024)}});
        expect(tooWide.contains("n_channels")) << "a width past the widest bank is refused by name: " << tooWide;

        const std::vector<float> supplied{0.1f, 0.2f, 0.3f, 0.4f, 0.3f, 0.2f, 0.1f};
        expect(!refusalAt({{"n_channels", gr::Size_t(4)}, {"taps", supplied}, {"transition", 1.5}}).empty()) << "a transition past a whole spacing, with taps supplied";
        const std::string noGain = refusalAt({{"n_channels", gr::Size_t(4)}, {"taps", std::vector<float>{1.f, -1.f}}});
        expect(noGain.contains("taps") && noGain.contains("no DC gain")) << "supplied taps with no DC gain are refused by name: " << noGain;

        expect(refusalAt({{"n_channels", gr::Size_t(4)}, {"prototype", std::string("lowpass")}, {"span", gr::Size_t(100)}}).empty()) << "the lowpass family does not read the span";
        expect(!refusalAt({{"n_channels", gr::Size_t(4)}, {"prototype", std::string("root_nyquist")}, {"span", gr::Size_t(65)}}).empty()) << "a root-Nyquist span past 64";
    };

    "taps copied from designed_taps measure as the design that made them"_test = [] {
        auto         bank             = make<PolyphaseChannelizer<float>>({{"n_channels", gr::Size_t(8)}, {"prototype", std::string("lowpass")}, {"transition", 0.2}});
        const auto   design           = gr::filter::designChannelizerPrototype(8UZ, "lowpass", 60.0, 0.2).value();
        const double designedStopband = bank->stopband_db.value;
        const double designedRipple   = bank->ripple_db.value;
        expect(fatal(gt(bank->designed_taps.value.size(), design.taps.size()))) << "the running prototype carries padding the design does not";

        expect(refusalOf(*bank, {{"taps", bank->designed_taps.value}}).empty());
        expect(approx(bank->stopband_db.value, designedStopband, 0.01)) << std::format("the padded copy reads {:.2f} dB against the design's {:.2f} dB", bank->stopband_db.value, designedStopband);
        expect(approx(bank->ripple_db.value, designedRipple, 0.01));
    };

    "a refused change leaves the bank on the settings it last accepted, and a later change is taken"_test = [] {
        constexpr gr::Size_t channels = 16U;

        auto                     bank     = make<PolyphaseChannelizer<float>>({{"n_channels", channels}});
        const std::vector<float> running  = bank->designed_taps.value;
        const double             stopband = bank->stopband_db.value;
        const std::size_t        stride   = bank->_stride;

        const auto onAccepted = [&](std::string_view refused) {
            expect(eq(bank->n_channels.value, channels)) << refused;
            expect(eq(bank->oversample.value, gr::Size_t(1))) << refused;
            expect(eq(bank->transition.value, 0.5)) << refused;
            expect(eq(bank->prototype.value, std::string("root_nyquist"))) << refused;
            expect(eq(bank->span.value, gr::Size_t(16))) << refused;
            expect(bank->taps.value.empty()) << refused;
            expect(std::ranges::equal(bank->designed_taps.value, running)) << refused;
            expect(eq(bank->stopband_db.value, stopband)) << refused << ": stopband_db reports the design that runs";
            expect(eq(bank->_stride, stride)) << refused;
            expect(eq(bank->out.size(), static_cast<std::size_t>(channels))) << refused;
        };

        expect(!refusalOf(*bank, {{"oversample", gr::Size_t(3)}}).empty());
        onAccepted("after oversample 3");
        expect(!refusalOf(*bank, {{"transition", 1.5}}).empty());
        onAccepted("after transition 1.5");
        expect(!refusalOf(*bank, {{"prototype", std::string("bandpass")}}).empty());
        onAccepted("after an unknown family");
        expect(!refusalOf(*bank, {{"n_channels", gr::Size_t(5)}, {"oversample", gr::Size_t(2)}}).empty());
        onAccepted("after an oversampled odd width");
        expect(!refusalOf(*bank, {{"span", gr::Size_t(1)}}).empty());
        onAccepted("after a root-Nyquist span of one");
        expect(!refusalOf(*bank, {{"taps", std::vector<float>{1.f, -1.f}}}).empty());
        onAccepted("after taps with no DC gain");

        expect(refusalOf(*bank, {{"span", gr::Size_t(8)}}).empty()) << "a valid change after the refusals is taken";
        const auto design = gr::filter::designChannelizerPrototype(static_cast<std::size_t>(channels), "root_nyquist", 60.0, 0.5, 8UZ, 1UZ).value();
        expect(eq(bank->designed_taps.value.size(), 8UZ * static_cast<std::size_t>(channels))) << "the bank runs the span it was given";
        expect(eq(bank->stopband_db.value, design.stopbandDb)) << "and reports that design";
        expect(nothrow([&bank] { bank->start(); })) << "a restart builds nothing that was refused";

        auto synth = make<PolyphaseSynthesizer<float>>({{"n_channels", channels}});
        expect(!refusalOf(*synth, {{"oversample", gr::Size_t(3)}}).empty());
        expect(eq(synth->oversample.value, gr::Size_t(1))) << "the synthesizer keeps its accepted stride";
        expect(eq(synth->_stride, static_cast<std::size_t>(channels)));
        expect(refusalOf(*synth, {{"span", gr::Size_t(8)}}).empty()) << "and takes a valid change after it";
        expect(eq(synth->designed_taps.value.size(), 8UZ * static_cast<std::size_t>(channels)));
    };

    "a forwarded rate reaches every channel, and the bank's own setting keys stay behind"_test = [] {
        constexpr gr::Size_t channels = 4U;
        constexpr gr::Size_t nSamples = 4096U;
        const std::size_t    stride   = static_cast<std::size_t>(channels);
        const auto           rateKey  = gr::property_map::key_type{"sample_rate"};

        std::vector<gr::Tag> inputTags;
        inputTags.emplace_back(512UZ, gr::property_map{{rateKey, gr::pmt::Value(48000.f)}});
        inputTags.emplace_back(1024UZ, gr::property_map{{rateKey, gr::pmt::Value(96000.0)}});
        inputTags.emplace_back(2000UZ, gr::property_map{{gr::property_map::key_type{"marker"}, gr::pmt::Value(gr::Size_t(1))}, {gr::property_map::key_type{"n_channels"}, gr::pmt::Value(channels)}});

        const auto run = runInGraph({{"n_channels", channels}}, tonePeriod(static_cast<std::size_t>(channels), 0UZ), nSamples, inputTags);

        for (std::size_t channel = 0UZ; channel < run.tags.size(); ++channel) {
            const auto rates = offsetsOf(run.tags[channel], "sample_rate");
            expect(eq(rates.size(), 2UZ)) << std::format("channel {} carries both rate tags", channel);
            for (const gr::Tag& entry : run.tags[channel]) {
                if (!entry.map.contains(rateKey)) {
                    continue;
                }
                const gr::pmt::Value& rate = entry.map.at(rateKey);
                if (entry.index == 512UZ / stride) {
                    expect(rate.get_if<float>() != nullptr && *rate.get_if<float>() == 48000.f / static_cast<float>(stride)) << std::format("channel {}: a float rate is divided by the stride", channel);
                } else {
                    expect(eq(entry.index, 1024UZ / stride));
                    expect(rate.get_if<double>() != nullptr && *rate.get_if<double>() == 96000.0 / static_cast<double>(stride)) << std::format("channel {}: a double rate is divided by the stride", channel);
                }
            }
        }

        const auto marked = std::ranges::find_if(run.tags[0], [](const gr::Tag& entry) { return entry.map.contains(gr::property_map::key_type{"marker"}); });
        expect(marked != run.tags[0].end()) << "channel zero carries the marker";
        if (marked != run.tags[0].end()) {
            expect(eq(marked->index, 2000UZ / stride));
            expect(!marked->map.contains(gr::property_map::key_type{"n_channels"})) << "without the key the bank owns as a setting";
        }
    };

    "a stream that ends inside a step drops the partial step and lands its tags at the end"_test = [] {
        constexpr gr::Size_t channels = 4U;
        constexpr gr::Size_t nSamples = 515U;
        const std::size_t    stride   = static_cast<std::size_t>(channels);
        const std::size_t    steps    = static_cast<std::size_t>(nSamples) / stride;

        std::vector<gr::Tag> inputTags;
        for (const std::size_t index : {400UZ, 513UZ}) {
            inputTags.emplace_back(index, gr::property_map{{gr::property_map::key_type{"marker"}, gr::pmt::Value(static_cast<gr::Size_t>(index))}});
        }

        // A tag at the end-of-stream index sits on no sample. Channel zero goes to a sink that reads the tag ring.
        gr::Graph graph;
        auto&     source = graph.emplaceBlock<TagSource<CF>>({{"n_samples_max", nSamples}, {"mark_tag", false}, {"values", gr::Tensor<CF>(gr::data_from, tonePeriod(static_cast<std::size_t>(channels), 0UZ))}});
        source._tags     = inputTags;
        auto& bank       = graph.emplaceBlock<PolyphaseChannelizer<float>>({{"n_channels", channels}});
        auto& endSink    = graph.emplaceBlock<gr::blocks::filter::testing::StreamEndSink<CF>>();
        expect(graph.connect(source, "out"s, bank, "in"s).has_value());
        expect(graph.connect(bank, "out#0"s, endSink, "in"s).has_value());
        std::vector<ChannelSink*> others;
        for (std::size_t channel = 1UZ; channel < stride; ++channel) {
            others.push_back(std::addressof(graph.emplaceBlock<ChannelSink>({})));
            expect(graph.connect(bank, "out#"s + std::to_string(channel), *others.back(), "in"s).has_value());
        }

        gr::scheduler::Simple<> scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        expect(eq(endSink._samples.size(), steps)) << "channel 0 holds the whole steps only";
        for (std::size_t channel = 1UZ; channel < stride; ++channel) {
            expect(eq(others[channel - 1UZ]->_samples.size(), steps)) << std::format("channel {} holds the whole steps only", channel);
        }
        const auto seen = endSink.offsetsOf("marker");
        expect(eq(seen.size(), 2UZ)) << "both tags reach channel zero";
        if (seen.size() == 2UZ) {
            expect(eq(seen[0], 400UZ / stride)) << "a tag inside a whole step lands at its step";
            expect(eq(seen[1], steps)) << "a tag inside the dropped tail lands at the end of the stream";
        }
    };

    "the synthesizer block runs the kernel its settings name"_test = [] {
        constexpr std::size_t channels = 8UZ;
        constexpr std::size_t steps    = 256UZ;

        for (const gr::Size_t oversample : {gr::Size_t(1), gr::Size_t(2)}) {
            auto synth = make<PolyphaseSynthesizer<float>>({{"n_channels", static_cast<gr::Size_t>(channels)}, {"oversample", oversample}});
            expect(fatal(eq(synth->in.size(), channels)));
            const std::size_t stride = channels / static_cast<std::size_t>(oversample);
            expect(eq(synth->_stride, stride));

            gr::filter::PolyphaseSynthesizer<float> kernel;
            expect(kernel.configure(channels, synth->designed_taps.value, static_cast<std::size_t>(oversample)).has_value());

            std::vector<std::vector<CF>> channelData;
            for (std::size_t channel = 0UZ; channel < channels; ++channel) {
                auto data = broadband(steps + channel);
                data.erase(data.begin(), data.begin() + static_cast<std::ptrdiff_t>(channel));
                channelData.push_back(std::move(data));
            }

            std::vector<test::InputSpan<CF>> inSpans;
            for (const std::vector<CF>& data : channelData) {
                inSpans.emplace_back(std::span<const CF>(data));
            }
            std::span<test::InputSpan<CF>> inputs(inSpans);
            std::vector<CF>                got(steps * stride);
            std::ignore = synth->processBulk(inputs, std::span<CF>(got));

            std::vector<CF> want(steps * stride);
            std::vector<CF> step(channels);
            for (std::size_t s = 0UZ; s < steps; ++s) {
                for (std::size_t channel = 0UZ; channel < channels; ++channel) {
                    step[channel] = channelData[channel][s];
                }
                kernel.step(step, std::span<CF>(want).subspan(s * stride, stride));
            }
            expect(std::ranges::equal(got, want)) << std::format("oversample {}: the block's output is the kernel's", oversample);
        }
    };

    "a width change that reaches a running graph is refused, and the bank keeps its ports"_test = [] {
        constexpr gr::Size_t channels = 4U;

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<TagSource<CF>>({{"n_samples_max", gr::Size_t(8192)}, {"mark_tag", false}, {"values", gr::Tensor<CF>(gr::data_from, tonePeriod(static_cast<std::size_t>(channels), 1UZ))}});
        source._tags.emplace_back(1000UZ, gr::property_map{{gr::property_map::key_type{"n_channels"}, gr::pmt::Value(gr::Size_t(8))}});

        // The width stays at its default. A setting named at construction does not follow tags.
        auto& bank = graph.emplaceBlock<PolyphaseChannelizer<float>>({});
        expect(fatal(eq(bank.n_channels.value, channels)));
        expect(graph.connect(source, "out"s, bank, "in"s).has_value());
        for (std::size_t channel = 0UZ; channel < static_cast<std::size_t>(channels); ++channel) {
            auto& sink = graph.emplaceBlock<ChannelSink>({});
            expect(graph.connect(bank, "out#"s + std::to_string(channel), sink, "in"s).has_value());
        }

        gr::scheduler::Simple<> scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        std::ignore = scheduler.runAndWait();

        expect(eq(bank.n_channels.value, channels)) << "the setting keeps the accepted width";
        expect(eq(bank.out.size(), static_cast<std::size_t>(channels))) << "the connected ports stay as they were";
        expect(eq(bank._channels, static_cast<std::size_t>(channels))) << "and the bank runs the width it started with";
    };
};

int main() { /* not needed for UT */ }
