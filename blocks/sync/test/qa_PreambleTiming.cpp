#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <random>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tensor.hpp>

#include <gnuradio-4.0/algorithm/channel/NoisePower.hpp>
#include <gnuradio-4.0/algorithm/digital/CpmPulse.hpp>
#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>

#include <gnuradio-4.0/sync/PreambleTiming.hpp>
#include <gnuradio-4.0/sync/SymbolSync.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

// The estimator, its bound, the detection threshold and the lattice belong to gr::sync::PreambleToneEstimator and are
// pinned by qa_PreambleTone. What is tested here is the block: that one burst yields one tag, that the tag names the
// sample and the phase it says it does, that neither depends on where the scheduler put its call boundaries, and that
// the loop downstream of it starts the burst already on the symbol.

namespace qa_preamble_timing {

using gr::blocks::sync::PreambleTiming;
using gr::blocks::sync::SymbolSync;
namespace test = gr::blocks::testing::span;

using CD = std::complex<double>;

constexpr float       kSampleRate = 48000.f;
constexpr float       kSymbolRate = 9600.f;
constexpr double      kModIndex   = 0.5;
constexpr double      kBt         = 0.4;
constexpr std::size_t kSps        = 5UZ;
constexpr std::size_t kSpan       = 3UZ;
constexpr std::size_t kPreamble   = 24UZ;
constexpr std::size_t kFlagBits   = 8UZ;
constexpr std::size_t kDataBits   = 200UZ;
constexpr double      kPi         = std::numbers::pi;

/// @brief The instant the chain puts symbol zero at: the causal Gaussian pulse's own center, in input samples.
constexpr double kAnchor = static_cast<double>(kSpan * kSps - 1UZ) / 2.0;

struct Rng {
    std::mt19937_64                  engine;
    std::normal_distribution<double> normal{0.0, 1.0};

    explicit Rng(std::uint64_t seed) : engine(seed) {}
    [[nodiscard]] double        gaussian() { return normal(engine); }
    [[nodiscard]] std::uint64_t bits() { return engine(); }
    [[nodiscard]] std::size_t   below(std::size_t bound) { return engine() % bound; }
};

[[nodiscard]] std::vector<CD> modulate(std::span<const float> symbols) {
    gr::digital::CpmPulse<float> pulse;
    pulse.configure(gr::digital::CpmPulseShape::Gaussian, kSpan, kSps, kModIndex, kBt);

    std::vector<double> increments(symbols.size() * kSps);
    std::ignore = pulse.incrementsFor(symbols, std::span<double>(increments));

    std::vector<CD> baseband(increments.size());
    double          phase = 0.0;
    for (std::size_t k = 0UZ; k < increments.size(); ++k) {
        phase += increments[k];
        baseband[k] = std::polar(1.0, phase);
    }
    return baseband;
}

[[nodiscard]] std::vector<float> designedLowpass(double cutoffSymbolRates) {
    gr::filter::fir::design::FilterSpec spec;
    spec.sampleRate      = static_cast<double>(kSampleRate);
    spec.cutoff          = cutoffSymbolRates * static_cast<double>(kSymbolRate);
    spec.transitionWidth = 0.5 * cutoffSymbolRates * static_cast<double>(kSymbolRate);
    return gr::filter::fir::design::designLowpass(spec);
}

template<typename T>
[[nodiscard]] std::vector<T> filtered(std::span<const T> signal, std::span<const float> taps) {
    const std::size_t lead = (taps.size() - 1UZ) / 2UZ;
    std::vector<T>    out(signal.size());
    for (std::size_t n = 0UZ; n < signal.size(); ++n) {
        const std::size_t index = n + lead;
        const std::size_t first = index + 1UZ > signal.size() ? index + 1UZ - signal.size() : 0UZ;
        const std::size_t last  = std::min(taps.size(), index + 1UZ);
        T                 sum{};
        for (std::size_t k = first; k < last; ++k) {
            sum += static_cast<double>(taps[k]) * signal[index - k];
        }
        out[n] = sum;
    }
    return out;
}

/// @brief The recipe's chain from the complex baseband to the stream the block sits on, with noise added at @p esN0Db.
[[nodiscard]] std::vector<float> receiveChain(std::span<const CD> baseband, double esN0Db, Rng* rng) {
    static const std::vector<float> channel = designedLowpass(0.6);
    static const std::vector<float> post    = designedLowpass(0.5);

    std::vector<CD> noisy(baseband.begin(), baseband.end());
    if (rng != nullptr) {
        const double power = gr::channel::noisePowerFor(esN0Db, 1.0, static_cast<double>(kSps));
        const double scale = std::sqrt(0.5 * power);
        for (CD& sample : noisy) {
            sample += CD{scale * rng->gaussian(), scale * rng->gaussian()};
        }
    }

    const std::vector<CD> band = filtered<CD>(std::span<const CD>(noisy), std::span<const float>(channel));
    const double          gain = static_cast<double>(kSps) / (kPi * kModIndex);

    std::vector<double> discriminated(band.size(), 0.0);
    for (std::size_t n = 1UZ; n < band.size(); ++n) {
        discriminated[n] = gain * std::arg(band[n] * std::conj(band[n - 1UZ]));
    }
    const std::vector<double> smoothed = filtered<double>(std::span<const double>(discriminated), std::span<const float>(post));

    std::vector<float> stream(smoothed.size());
    std::ranges::transform(smoothed, stream.begin(), [](double v) { return static_cast<float>(v); });
    return stream;
}

/// @brief One AIS-shaped slot as channel symbols: the alternating training tone, the start flag and NRZI-coded data.
///
/// The NRZI encoder starts on the training sequence's last symbol, so the flag's leading zero is one more transition
/// and the tone runs unbroken into the frame, which is what the standard's slot layout produces.
[[nodiscard]] std::vector<float> slotSymbols(Rng& rng, std::size_t dataBits = kDataBits) {
    constexpr std::array<std::uint8_t, kFlagBits> kFlag{{0U, 1U, 1U, 1U, 1U, 1U, 1U, 0U}};

    std::vector<std::uint8_t> channel;
    std::uint8_t              level = 0U;
    for (std::size_t k = 0UZ; k < kPreamble; ++k) {
        channel.push_back(level);
        level = static_cast<std::uint8_t>(level ^ 1U);
    }
    std::uint8_t previous = channel.back();
    const auto   encode   = [&channel, &previous](std::uint8_t bit) {
        previous = static_cast<std::uint8_t>(bit != 0U ? previous : (previous ^ 1U));
        channel.push_back(previous);
    };
    for (const std::uint8_t bit : kFlag) {
        encode(bit);
    }
    std::uint64_t pool = 0ULL;
    for (std::size_t k = 0UZ; k < dataBits; ++k) {
        if (k % 64UZ == 0UZ) {
            pool = rng.bits();
        }
        encode(static_cast<std::uint8_t>((pool >> (k % 64UZ)) & 1ULL));
    }

    std::vector<float> symbols(channel.size());
    std::ranges::transform(channel, symbols.begin(), [](std::uint8_t bit) { return bit != 0U ? 1.f : -1.f; });
    return symbols;
}

/// Where each slot sits in the stream the scene builds, so a tag can be checked against its own slot.
struct Slot {
    std::size_t first = 0UZ; ///< the burst's first sample
    std::size_t flag  = 0UZ; ///< the last sample of its start flag
};

struct Scene {
    std::vector<float> stream{};
    std::vector<Slot>  slots{};
};

/// @brief @p count slots at a random symbol phase inside noise-only gaps, taken through the receive chain.
[[nodiscard]] Scene bursts(std::size_t count, std::uint64_t seed, double esN0Db, std::size_t gapSymbols = 40UZ) {
    Rng             rng(seed);
    std::vector<CD> baseband(gapSymbols * kSps, CD{});
    Scene           scene;
    for (std::size_t k = 0UZ; k < count; ++k) {
        const std::vector<float> symbols = slotSymbols(rng);
        const std::vector<CD>    burst   = modulate(std::span<const float>(symbols));
        baseband.insert(baseband.end(), rng.below(kSps), CD{});
        const std::size_t first = baseband.size();
        baseband.insert(baseband.end(), burst.begin(), burst.end());
        baseband.insert(baseband.end(), gapSymbols * kSps, CD{});
        scene.slots.push_back({first, first + (kPreamble + kFlagBits) * kSps});
    }
    Rng noise(seed ^ 0x5EEDULL);
    scene.stream = receiveChain(std::span<const CD>(baseband), esN0Db, &noise);
    return scene;
}

/// One run of the block: the stream it passed through, the statistic beside it, and the tags it published.
struct Capture {
    std::vector<float>   samples{};
    std::vector<float>   statistic{};
    std::vector<gr::Tag> tags{};
    std::size_t          consumed = 0UZ;
};

/// @brief Drive the block over @p input with @p feed samples arriving per call and @p room outputs allowed per call.
[[nodiscard]] Capture drive(PreambleTiming<float>& block, std::span<const float> input, std::size_t feed = 0UZ, std::size_t room = 0UZ, bool wantStatistic = true, std::span<const gr::Tag> tags = {}) {
    Capture           result;
    const std::size_t arriving = feed == 0UZ ? std::max(input.size(), 1UZ) : feed;
    const std::size_t allowed  = room == 0UZ ? std::max(input.size(), 1UZ) : room;

    std::vector<float> main(allowed);
    std::vector<float> side(allowed);

    std::size_t consumed = 0UZ;
    std::size_t fed      = 0UZ;
    while (consumed < input.size()) {
        fed              = std::min(input.size(), fed + arriving);
        const auto first = std::ranges::lower_bound(tags, consumed, std::ranges::less{}, &gr::Tag::index);
        const auto last  = std::ranges::lower_bound(tags, fed, std::ranges::less{}, &gr::Tag::index);

        test::InputSpan<float>  inSpan(input.subspan(consumed, fed - consumed), consumed, std::span<const gr::Tag>(first, last));
        test::OutputSpan<float> outSpan(std::span<float>(main.data(), allowed), result.samples.size(), &result.tags);
        test::OutputSpan<float> sideSpan(wantStatistic ? std::span<float>(side.data(), allowed) : std::span<float>{}, result.statistic.size(), nullptr, wantStatistic);

        std::ignore = block.processBulk(inSpan, outSpan, sideSpan);

        result.samples.insert(result.samples.end(), main.begin(), main.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        result.statistic.insert(result.statistic.end(), side.begin(), side.begin() + static_cast<std::ptrdiff_t>(sideSpan.count));
        result.consumed += inSpan.consumed;
        consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && fed == input.size()) {
            break;
        }
    }
    return result;
}

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

[[nodiscard]] gr::property_map defaults(std::size_t preambleSymbols) { return {{"sample_rate", kSampleRate}, {"symbol_rate", kSymbolRate}, {"preamble_symbols", static_cast<gr::Size_t>(preambleSymbols)}}; }

/// @brief A source that ends, so a finite graph terminates.
struct FiniteSource : gr::Block<FiniteSource> {
    gr::PortOut<float> out;
    GR_MAKE_REFLECTABLE(FiniteSource, out);
    std::vector<float> _data{};
    std::size_t        _pos = 0UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) noexcept {
        const std::size_t n = std::min(outSpan.size(), _data.size() - _pos);
        std::copy_n(_data.begin() + static_cast<std::ptrdiff_t>(_pos), n, outSpan.begin());
        outSpan.publish(n);
        _pos += n;
        return _pos == _data.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

/// @brief Hands on a drawn number of samples a call, so the block downstream sees call boundaries it did not choose.
struct Chunker : gr::Block<Chunker> {
    gr::PortIn<float>  in;
    gr::PortOut<float> out;
    GR_MAKE_REFLECTABLE(Chunker, in, out);
    std::uint64_t _state = 0x1234ULL;
    std::size_t   _fixed = 0UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) noexcept {
        std::size_t wanted = _fixed;
        if (wanted == 0UZ) {
            _state ^= _state << 13U;
            _state ^= _state >> 7U;
            _state ^= _state << 17U;
            wanted = 1UZ + static_cast<std::size_t>(_state % 97ULL);
        }
        const std::size_t n = std::min({wanted, inSpan.size(), outSpan.size()});
        std::copy_n(inSpan.begin(), n, outSpan.begin());
        std::ignore = inSpan.consume(n);
        outSpan.publish(n);
        if (n > 0UZ) {
            return gr::work::Status::OK;
        }
        return outSpan.size() == 0UZ ? gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS : gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
    }
};

/// @brief Keeps every sample and every tag offset a run produced.
struct TagSink : gr::Block<TagSink> {
    gr::PortIn<float> in;
    GR_MAKE_REFLECTABLE(TagSink, in);
    std::vector<float>   _samples{};
    std::vector<gr::Tag> _tags{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const gr::Tag& tag : inSpan.rawTags) {
            if (tag.index >= _samples.size()) {
                _tags.push_back(tag);
            }
        }
        _samples.insert(_samples.end(), inSpan.begin(), inSpan.end());
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

[[nodiscard]] std::vector<double> clockPayload(const gr::property_map& map) {
    const auto found = map.find(gr::property_map::key_type("clock_est"));
    if (found == map.end()) {
        return {};
    }
    const auto* narrow = found->second.get_if<gr::Tensor<float>>();
    if (narrow == nullptr) {
        return {};
    }
    std::vector<double> values;
    for (std::size_t k = 0UZ; k < narrow->size(); ++k) {
        values.push_back(static_cast<double>((*narrow)[k]));
    }
    return values;
}

} // namespace qa_preamble_timing

using namespace qa_preamble_timing;

const boost::ut::suite<"PreambleTiming"> preambleTimingTests = [] {
    using namespace boost::ut;

    "preamble_symbols zero makes the block a wire"_test = [] {
        // The recipe dialect has no conditional, so the stage is carried unconditionally and a continuous link
        // disables it.
        Rng                      rng(0x11ULL);
        const std::vector<float> symbols = slotSymbols(rng);
        const std::vector<float> stream  = receiveChain(std::span<const CD>(modulate(std::span<const float>(symbols))), 0.0, nullptr);

        PreambleTiming<float> block   = make<PreambleTiming<float>>(defaults(0UZ));
        const Capture         through = drive(block, std::span<const float>(stream));

        expect(eq(block.nDetections, 0ULL)) << "a disabled block declares nothing";
        expect(eq(through.samples.size(), stream.size())) << "and passes the stream through sample for sample";
        expect(eq(through.tags.size(), 0UZ));
        expect(that % (through.samples == stream)) << "unchanged, bit for bit";
        expect(eq(block.in.min_samples, 1UZ)) << "and holds nothing back";
    };

    "one burst yields one tag, inside its own slot"_test = [] {
        // The tag has to name a sample between the burst's first and its start flag's last: earlier is a preset taken
        // from noise, later is a preset the loop has already passed.
        constexpr std::size_t kSlots = 200UZ;

        const Scene           scene = bursts(kSlots, 0xA15ULL, 20.0);
        PreambleTiming<float> block = make<PreambleTiming<float>>(defaults(kPreamble));
        const Capture         run   = drive(block, std::span<const float>(scene.stream));

        expect(eq(block.nDetections, static_cast<std::uint64_t>(kSlots))) << "one tag a burst, and no burst missed";
        expect(eq(run.tags.size(), kSlots)) << "and every one of them reached the output";

        std::size_t placed = 0UZ;
        for (std::size_t k = 0UZ; k < std::min(run.tags.size(), scene.slots.size()); ++k) {
            const Slot& slot = scene.slots[k];
            placed += run.tags[k].index >= slot.first && run.tags[k].index <= slot.flag ? 1UZ : 0UZ;
        }
        expect(eq(placed, std::min(run.tags.size(), scene.slots.size()))) << "every tag sits between its burst's first sample and its start flag's last";
    };

    "the tag carries the keys, the types and the units the contract states"_test = [] {
        // Read off the tag, not off the code: the period is asserted equal to the configured constant on a stream
        // whose true rate is deliberately one per cent off, which is what proves it is a configuration constant and
        // not a measurement.
        Rng                      rng(0x37ULL);
        const std::vector<float> symbols = slotSymbols(rng);
        const std::vector<float> stream  = receiveChain(std::span<const CD>(modulate(std::span<const float>(symbols))), 20.0, &rng);

        PreambleTiming<float> unnamed  = make<PreambleTiming<float>>(defaults(kPreamble));
        const Capture         untagged = drive(unnamed, std::span<const float>(stream));
        expect(eq(untagged.tags.size(), 1UZ)) << fatal;
        expect(untagged.tags[0].map.find(gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey())) == untagged.tags[0].map.end()) << "an unnamed detector writes no trigger, so a framer downstream does not resynchronize on a timing preset";
        expect(untagged.tags[0].map.find(gr::property_map::key_type(gr::tag::TRIGGER_OFFSET.shortKey())) == untagged.tags[0].map.end()) << "and none of the trigger pair without the other";
        expect(eq(clockPayload(untagged.tags[0].map).size(), 2UZ)) << "the timing payload is written either way";

        gr::property_map withLabel                             = defaults(kPreamble);
        withLabel[gr::property_map::key_type("trigger_label")] = std::string("preamble");

        PreambleTiming<float> block = make<PreambleTiming<float>>(withLabel);
        const Capture         run   = drive(block, std::span<const float>(stream));
        expect(eq(run.tags.size(), 1UZ)) << fatal;

        const gr::property_map& map   = run.tags[0].map;
        const auto              named = map.find(gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()));
        expect(named != map.end()) << fatal;
        const auto* label = named->second.get_if<std::pmr::string>();
        expect(label != nullptr) << "trigger_name is a string" << fatal;
        expect(eq(std::string(label->data(), label->size()), std::string("preamble")));

        const auto delay = map.find(gr::property_map::key_type(gr::tag::TRIGGER_OFFSET.shortKey()));
        expect(delay != map.end()) << fatal;
        const auto* seconds = delay->second.get_if<float>();
        expect(seconds != nullptr) << "trigger_offset is a float, and the key declares seconds" << fatal;
        expect(lt(std::abs(static_cast<double>(*seconds)), 0.5 / static_cast<double>(kSampleRate) + 1e-12)) << "half a sample either side, in the seconds the key declares";

        const std::vector<double> payload = clockPayload(map);
        expect(eq(payload.size(), 2UZ)) << "clock_est is a two-element real tensor" << fatal;
        expect(lt(std::abs(payload[0]), 0.5 + 1e-9)) << "the offset is half an input sample either side";
        expect(lt(std::abs(payload[1] - static_cast<double>(kSampleRate / kSymbolRate)), 1e-6)) << "and the period is the nominal one";
        expect(lt(std::abs(static_cast<double>(*seconds) * static_cast<double>(kSampleRate) - payload[0]), 1e-5)) << "the two encodings are one number in two units";
        expect(map.find(gr::property_map::key_type("time_est")) == map.end()) << "clock_est and time_est are never both written";

        // the same stream read as though the link ran one per cent fast: the tag still carries the configured period
        constexpr float       kOffRate = kSymbolRate * 1.01f;
        PreambleTiming<float> mistuned = make<PreambleTiming<float>>({{"sample_rate", kSampleRate}, {"symbol_rate", kOffRate}, {"preamble_symbols", static_cast<gr::Size_t>(kPreamble)}});
        const Capture         drifted  = drive(mistuned, std::span<const float>(stream));
        expect(ge(drifted.tags.size(), 1UZ)) << "a one per cent rate error still declares the preamble" << fatal;
        const std::vector<double> other = clockPayload(drifted.tags[0].map);
        expect(eq(other.size(), 2UZ)) << fatal;
        expect(lt(std::abs(other[1] - static_cast<double>(kSampleRate / kOffRate)), 1e-5)) << "the period in the tag is the configured constant, never a measurement";

        PreambleTiming<float> plain = make<PreambleTiming<float>>({{"sample_rate", kSampleRate}, {"symbol_rate", kSymbolRate}, {"preamble_symbols", static_cast<gr::Size_t>(kPreamble)}, {"preset_period", false}});
        const Capture         bare  = drive(plain, std::span<const float>(stream));
        expect(eq(bare.tags.size(), 1UZ)) << fatal;
        const auto offset = bare.tags[0].map.find(gr::property_map::key_type("time_est"));
        expect(offset != bare.tags[0].map.end()) << "preset_period false writes time_est" << fatal;
        expect(offset->second.get_if<float>() != nullptr) << "time_est is a float in input samples";
        expect(bare.tags[0].map.find(gr::property_map::key_type("clock_est")) == bare.tags[0].map.end()) << "and no clock_est beside it";
    };

    "the tag does not move with the call boundaries"_test = [] {
        // The direct driver never consults the port minimum, so the second leg runs the block under a scheduler
        // behind a block that hands on a drawn number of samples a call, which is the only way a progress requirement
        // that is wrong shows up.
        const Scene scene = bursts(6UZ, 0x38ULL, 20.0);

        std::vector<std::size_t> offsets;
        std::vector<double>      phases;
        for (const std::size_t chunk : {1UZ, 7UZ, 4096UZ}) {
            PreambleTiming<float> block = make<PreambleTiming<float>>(defaults(kPreamble));
            const Capture         run   = drive(block, std::span<const float>(scene.stream), chunk, chunk);

            std::vector<std::size_t> theseOffsets;
            std::vector<double>      thesePhases;
            for (const gr::Tag& seenTag : run.tags) {
                theseOffsets.push_back(seenTag.index);
                const std::vector<double> payload = clockPayload(seenTag.map);
                thesePhases.push_back(payload.empty() ? 0.0 : payload[0]);
            }
            expect(eq(run.samples.size(), scene.stream.size() - block.in.min_samples + 1UZ)) << "the hold-back is the only latency, chunk " << chunk;
            expect(that % (run.samples == std::vector<float>(scene.stream.begin(), scene.stream.begin() + static_cast<std::ptrdiff_t>(run.samples.size())))) << "and the stream itself is the input, sample for sample, chunk " << chunk;
            if (chunk == 1UZ) {
                offsets = theseOffsets;
                phases  = thesePhases;
                expect(eq(offsets.size(), 6UZ)) << "six bursts, six tags" << fatal;
            } else {
                expect(that % (theseOffsets == offsets)) << "tag offsets do not move with the chunking, chunk " << chunk;
                expect(that % (thesePhases == phases)) << "and neither do the payloads, chunk " << chunk;
            }
        }

        for (const std::size_t fixed : {0UZ, 1UZ, 7UZ}) {
            gr::Graph flow;
            auto&     source = flow.emplaceBlock<FiniteSource>();
            source._data     = scene.stream;
            auto& chunker    = flow.emplaceBlock<Chunker>();
            chunker._fixed   = fixed;
            auto& block      = flow.emplaceBlock<PreambleTiming<float>>(defaults(kPreamble));
            auto& sink       = flow.emplaceBlock<TagSink>();
            expect(flow.connect<"out", "in">(source, chunker).has_value());
            expect(flow.connect<"out", "in">(chunker, block).has_value());
            expect(flow.connect<"out", "in">(block, sink).has_value());

            gr::scheduler::Simple<> scheduler;
            expect(scheduler.exchange(std::move(flow)).has_value());
            std::atomic<bool> done{false};
            std::thread       runner([&scheduler, &done] {
                std::ignore = scheduler.runAndWait();
                done        = true;
            });
            const auto        start = std::chrono::steady_clock::now();
            while (!done.load() && std::chrono::steady_clock::now() - start < std::chrono::seconds(60)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            const bool finished = done.load();
            if (!finished) {
                scheduler.requestStop();
            }
            runner.join();
            expect(finished) << "the graph terminates: the block's port minimum has to be a progress guarantee, not a worst case";

            std::vector<std::size_t> scheduled;
            for (const gr::Tag& seenTag : sink._tags) {
                if (seenTag.map.contains(gr::property_map::key_type("clock_est"))) {
                    scheduled.push_back(seenTag.index);
                }
            }
            expect(that % (scheduled == offsets)) << "the scheduler's own chunking places every tag where the direct driver did";
        }
    };

    "the preset lands the loop on the symbol before the first data bit"_test = [] {
        // Measured on SymbolSync's own position, not inferred from a decode count: the loop's initial error without a
        // tag is uniform on half a symbol either side, 0.289 symbol RMS, and what the preset has to do is replace
        // that with the estimator's own accuracy before the data starts.
        constexpr std::size_t kRuns  = 60UZ;
        constexpr std::size_t kFirst = kPreamble + kFlagBits + 8UZ; // the ninth symbol after the training sequence
        constexpr std::size_t kGap   = 40UZ;

        for (const double bandwidth : {0.002, 0.010, 0.020}) {
            double presetSquares = 0.0;
            double plainSquares  = 0.0;
            for (std::size_t run = 0UZ; run < kRuns; ++run) {
                Rng                      rng(0x41ULL + run);
                const std::vector<float> symbols = slotSymbols(rng);
                const std::size_t        jitter  = rng.below(kSps);

                std::vector<CD>       baseband(kGap * kSps + jitter, CD{});
                const std::size_t     first = baseband.size();
                const std::vector<CD> burst = modulate(std::span<const float>(symbols));
                baseband.insert(baseband.end(), burst.begin(), burst.end());
                baseband.insert(baseband.end(), kGap * kSps, CD{});

                Rng                      noise(0x9E11ULL + run);
                const std::vector<float> stream = receiveChain(std::span<const CD>(baseband), 20.0, &noise);
                const double             truth  = static_cast<double>(first) + kAnchor + static_cast<double>(kFirst) * static_cast<double>(kSps);

                PreambleTiming<float> detector = make<PreambleTiming<float>>(defaults(kPreamble));
                const Capture         tagged   = drive(detector, std::span<const float>(stream), 0UZ, 0UZ, false);

                for (const bool preset : {true, false}) {
                    SymbolSync<float>    loop = make<SymbolSync<float>>({{"samples_per_symbol", static_cast<double>(kSps)}, {"noise_bandwidth", bandwidth}, {"constellation", std::string("bpsk")}});
                    std::vector<gr::Tag> steering;
                    if (preset) {
                        steering = tagged.tags;
                    }

                    // one output a call, so the position the block leaves is the instant of the output that follows it
                    std::vector<double> instants;
                    std::size_t         consumed = 0UZ;
                    std::vector<float>  one(1UZ);
                    while (consumed < tagged.samples.size()) {
                        const auto              begin = std::ranges::lower_bound(steering, consumed, std::ranges::less{}, &gr::Tag::index);
                        test::InputSpan<float>  inSpan(std::span<const float>(tagged.samples).subspan(consumed), consumed, std::span<const gr::Tag>(begin, steering.end()));
                        test::OutputSpan<float> outSpan(std::span<float>(one), 0UZ, nullptr);
                        test::OutputSpan<float> a(std::span<float>{}, 0UZ, nullptr, false);
                        test::OutputSpan<float> b(std::span<float>{}, 0UZ, nullptr, false);
                        test::OutputSpan<float> c(std::span<float>{}, 0UZ, nullptr, false);
                        std::ignore = loop.processBulk(inSpan, outSpan, a, b, c);
                        instants.push_back(static_cast<double>(loop._base) + 3.0 + loop._mu);
                        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ) {
                            break;
                        }
                        consumed += inSpan.consumed;
                    }

                    double nearest = 1e18;
                    for (const double instant : instants) {
                        if (std::abs(instant - truth) < std::abs(nearest - truth)) {
                            nearest = instant;
                        }
                    }
                    const double error = (nearest - truth) / static_cast<double>(kSps);
                    (preset ? presetSquares : plainSquares) += error * error;
                }
            }
            const double preset = std::sqrt(presetSquares / static_cast<double>(kRuns));
            const double plain  = std::sqrt(plainSquares / static_cast<double>(kRuns));
            expect(lt(preset, 0.06)) << "the preset puts the loop on the symbol before the data starts";
            expect(lt(preset, 0.5 * plain)) << "and the untagged run is the acquisition the preset removes";
        }
    };
};

int main() { /* tests are automatically registered and run */ }
