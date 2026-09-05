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
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/adsb/PpmFramer.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_ppm_framer {

using gr::blocks::adsb::PpmFramer;
using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;
using gr::digital::PpmCounters;
using gr::digital::PpmFrame;
using gr::digital::PpmOutcome;
using gr::digital::PpmScanner;

using Record = gr::DataSet<std::uint8_t>;

/// The two instantiations: magnitudes, as this framer has always taken them, and the samples the magnitudes came from.
using MagFramer = PpmFramer<float>;
using IqFramer  = PpmFramer<std::complex<float>>;

/// The five published 1090 MHz extended squitters, whose parity fields are the bare CRC.
constexpr std::array<std::string_view, 5> kPublished{"8D4840D6202CC371C32CE0576098", "8D40621D58C382D690C8AC2863A7", "8D40621D58C386435CC412692AD6", "8D485020994409940838175B284F", "8D40621D58C3862590C412D77427"};

/// The one this suite anchors on; downlink format 17, address 0x4840D6.
constexpr std::string_view kAnchor = kPublished[0UZ];

/// The preamble in half-microsecond slots: pulses at 0.0, 1.0, 3.5 and 4.5 microseconds.
constexpr std::array<int, 16> kPreamble{1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0};

[[nodiscard]] std::vector<std::uint8_t> octetsOf(std::string_view hex) {
    std::vector<std::uint8_t> octets;
    octets.reserve(hex.size() / 2UZ);
    const auto nibble = [](char c) { return static_cast<std::uint8_t>(c <= '9' ? c - '0' : (c | 0x20) - 'a' + 10); };
    for (std::size_t i = 0UZ; i + 1UZ < hex.size(); i += 2UZ) {
        octets.push_back(static_cast<std::uint8_t>((nibble(hex[i]) << 4U) | nibble(hex[i + 1UZ])));
    }
    return octets;
}

/// @brief The samples one frame occupies: sixteen preamble slots, then two slots a bit, each slot @p slot samples.
[[nodiscard]] std::vector<float> render(std::span<const std::uint8_t> frame, std::size_t slot = 1UZ) {
    const std::size_t  bits = frame.size() * 8UZ;
    std::vector<float> wave;
    wave.reserve((16UZ + 2UZ * bits) * slot);
    const auto emit = [&wave, slot](float value) { wave.insert(wave.end(), slot, value); };
    for (const int pulse : kPreamble) {
        emit(pulse != 0 ? 1.F : 0.F);
    }
    for (std::size_t bit = 0UZ; bit < bits; ++bit) {
        const bool one = ((frame[bit / 8UZ] >> (7U - bit % 8UZ)) & 1U) != 0U;
        emit(one ? 1.F : 0.F);
        emit(one ? 0.F : 1.F);
    }
    return wave;
}

void append(std::vector<float>& into, std::span<const float> more) { into.insert(into.end(), more.begin(), more.end()); }
void appendZeros(std::vector<float>& into, std::size_t count) { into.insert(into.end(), count, 0.F); }

/// @brief One frame followed by a long frame's worth of silence, which is what a position needs before it is decided.
[[nodiscard]] std::vector<float> padded(std::span<const std::uint8_t> frame, std::size_t slot = 1UZ) {
    std::vector<float> wave = render(frame, slot);
    appendZeros(wave, (16UZ + 224UZ) * slot);
    return wave;
}

/// @brief A frame whose trailing parity field is the bare CRC over its message octets, built from @p message.
[[nodiscard]] std::vector<std::uint8_t> withParity(std::vector<std::uint8_t> message, std::size_t octets, std::uint64_t addressXor = 0ULL) {
    PpmScanner scanner;
    scanner.prepare(gr::digital::modeS(), 1UZ);
    message.resize(octets, std::uint8_t{0});
    const std::uint64_t parity = scanner.remainderOf(std::span<const std::uint8_t>(message)) ^ addressXor;
    message[octets - 3UZ]      = static_cast<std::uint8_t>(parity >> 16U);
    message[octets - 2UZ]      = static_cast<std::uint8_t>(parity >> 8U);
    message[octets - 1UZ]      = static_cast<std::uint8_t>(parity);
    return message;
}

/// One nomination as the kernel reported it, with the octets copied out of the scanner.
struct Seen {
    std::vector<std::uint8_t> octets{};
    std::size_t               bits      = 0UZ;
    std::uint32_t             format    = 0U;
    std::uint64_t             remainder = 0ULL;
    float                     strong    = 0.F;
    float                     weak      = 0.F;
    std::size_t               position  = 0UZ;
    PpmOutcome                outcome   = PpmOutcome::Admitted;
};

/// What one kernel run reported and counted.
struct Scanned {
    std::vector<Seen> frames{};
    PpmCounters       counters{};
};

/**
 * @brief Drives the kernel over @p data presenting @p chunk fresh samples at a time.
 *
 * The undecided tail is presented again with whatever arrives next, which is exactly the contract `consume()`
 * states and what the block does with its own working buffer.
 */
[[nodiscard]] Scanned scan(std::span<const float> data, std::size_t chunk = 0UZ, std::size_t slot = 1UZ, float threshold = 2.F) {
    PpmScanner scanner;
    scanner.prepare(gr::digital::modeS(), slot);
    scanner.threshold = threshold;

    Scanned           result;
    const std::size_t stride = chunk == 0UZ ? std::max(data.size(), 1UZ) : chunk;
    std::size_t       base   = 0UZ;
    std::size_t       fed    = std::min(data.size(), stride);
    while (true) {
        scanner.seek(base);
        const std::size_t done = scanner.consume(data.subspan(base, fed - base), [&result](const PpmFrame& frame) { result.frames.push_back({std::vector<std::uint8_t>(frame.octets.begin(), frame.octets.end()), frame.bits, frame.format, frame.remainder, frame.strong, frame.weak, frame.position, frame.outcome}); });
        base += done;
        if (fed == data.size()) {
            if (done == 0UZ) {
                break;
            }
        } else {
            fed = std::min(data.size(), fed + stride);
        }
    }
    result.counters = scanner.counters;
    return result;
}

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
    return block;
}

/// The settings every block case shares, at the 2 MS/s the canonical receivers deliver.
[[nodiscard]] gr::property_map modeSSettings(float rate = 2.0e6F) { return {{"profile", std::string("mode_s")}, {"sample_rate", rate}}; }

/// What one drive of the block produced, with the counters it reported.
struct Framed {
    std::vector<Record> records{};
    std::uint64_t       samples = 0ULL, candidates = 0ULL, nominations = 0ULL, admitted = 0ULL, crcFailed = 0ULL, shortFormat = 0ULL, published = 0ULL, tagsDropped = 0ULL, rateRefused = 0ULL, tailDropped = 0ULL, overrunDropped = 0ULL;
    std::size_t         held = 0UZ; ///< samples still in the working buffer when the run ended, before stop() drops them
};

/// @brief Drives the block over @p input in windows of @p chunk samples, with @p tags planted at absolute offsets.
///
/// @p room is the records the sink offers per call, and @p connected drives the block with its output port left
/// unconnected, which is what a graph does when nothing is wired to `out`.
template<typename T>
[[nodiscard]] Framed feed(PpmFramer<T>& block, std::span<const T> input, std::size_t chunk = 0UZ, std::span<const gr::Tag> tags = {}, std::size_t room = 64UZ, bool connected = true) {
    Framed              result;
    const std::size_t   stride = chunk == 0UZ ? std::max(input.size(), 1UZ) : chunk;
    std::vector<Record> scratch(room);

    for (std::size_t base = 0UZ; base < input.size();) {
        const std::size_t count = std::min(stride, input.size() - base);
        const auto        first = std::ranges::lower_bound(tags, base, std::ranges::less{}, &gr::Tag::index);
        const auto        last  = std::ranges::lower_bound(tags, base + count, std::ranges::less{}, &gr::Tag::index);

        InputSpan<T>       inSpan(input.subspan(base, count), base, std::span<const gr::Tag>(first, last));
        OutputSpan<Record> outSpan(connected ? std::span<Record>(scratch) : std::span<Record>{}, 0UZ, nullptr, connected);
        std::ignore = block.processBulk(inSpan, outSpan);
        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            result.records.push_back(std::move(scratch[k]));
        }
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ) {
            break;
        }
        base += inSpan.consumed;
    }
    result.held = block._work.size();

    result.samples        = block.nSamples;
    result.candidates     = block.nCandidates;
    result.nominations    = block.nNominations;
    result.admitted       = block.nAdmitted;
    result.crcFailed      = block.nCrcFailed;
    result.shortFormat    = block.nShortFormat;
    result.published      = block.nPublished;
    result.tagsDropped    = block.nTagsDropped;
    result.rateRefused    = block.nRateRefused;
    result.tailDropped    = block.nTailDropped;
    result.overrunDropped = block.nOverrunDropped;
    return result;
}

[[nodiscard]] const gr::property_map& metaOf(const Record& record) {
    static const gr::property_map kEmpty{};
    return record.meta_information.empty() ? kEmpty : record.meta_information.front();
}

[[nodiscard]] std::uint64_t metaU64(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? ~0ULL : entry->second.value_or(std::uint64_t{~0ULL});
}

[[nodiscard]] gr::Size_t metaSize(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::numeric_limits<gr::Size_t>::max() : entry->second.value_or(std::numeric_limits<gr::Size_t>::max());
}

[[nodiscard]] std::string metaString(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::string{} : std::string(entry->second.value_or(std::string_view{}));
}

[[nodiscard]] float metaFloat(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? -1.F : entry->second.value_or(-1.F);
}

[[nodiscard]] bool metaBool(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry != map.end() && entry->second.value_or(false);
}

[[nodiscard]] bool metaHas(const Record& record, std::string_view key) { return metaOf(record).contains(gr::property_map::key_type(key)); }

template<typename T>
struct FiniteSource : gr::Block<FiniteSource<T>> {
    gr::PortOut<T> out;
    GR_MAKE_REFLECTABLE(FiniteSource, out);
    std::vector<T> _data{};
    std::size_t    _pos = 0UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) noexcept {
        const std::size_t n = std::min(outSpan.size(), _data.size() - _pos);
        std::copy_n(_data.begin() + static_cast<std::ptrdiff_t>(_pos), n, outSpan.begin());
        outSpan.publish(n);
        _pos += n;
        return _pos == _data.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<Record, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);
    std::vector<Record> _records{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const auto& record : inSpan) {
            _records.push_back(record);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/// @brief Runs a graph to completion under the simple scheduler, stopping it rather than hanging if it wedges.
template<typename TCollect>
void runGraph(gr::Graph flow, TCollect&& collect) {
    gr::scheduler::Simple<> scheduler;
    boost::ut::expect(scheduler.exchange(std::move(flow)).has_value());
    std::atomic<bool> done{false};
    std::thread       runner([&scheduler, &done] {
        std::ignore = scheduler.runAndWait();
        done        = true;
    });
    const auto        start = std::chrono::steady_clock::now();
    while (!done.load() && std::chrono::steady_clock::now() - start < std::chrono::seconds(30)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!done.load()) {
        scheduler.requestStop();
        boost::ut::expect(false) << "the graph did not finish within thirty seconds";
    }
    runner.join();
    collect();
}

/// @brief Runs @p samples through a `PpmFramer` under the scheduler and hands back what the sink saw.
template<typename T = float>
[[nodiscard]] std::vector<Record> scheduled(std::vector<T> samples, gr::property_map settings) {
    gr::Graph flow;
    auto&     source = flow.emplaceBlock<FiniteSource<T>>();
    source._data     = std::move(samples);
    auto& framer     = flow.emplaceBlock<PpmFramer<T>>(std::move(settings));
    auto& sink       = flow.emplaceBlock<RecordSink>();
    boost::ut::expect(flow.connect<"out", "in">(source, framer).has_value());
    boost::ut::expect(flow.connect<"out", "in">(framer, sink).has_value());

    std::vector<Record> records;
    runGraph(std::move(flow), [&records, &sink] { records = sink._records; });
    return records;
}

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
    /// A number in (0, 1].
    [[nodiscard]] double uniform() noexcept { return static_cast<double>((next() >> 11U) + 1ULL) * 0x1p-53; }
};

/// @brief Rayleigh magnitudes of unit mean power, which is what a complex Gaussian channel presents to the framer.
[[nodiscard]] std::vector<float> rayleigh(std::size_t count, std::uint64_t seed) {
    Rng                rng{seed};
    std::vector<float> noise(count);
    for (float& value : noise) {
        value = static_cast<float>(std::sqrt(-std::log(rng.uniform())));
    }
    return noise;
}

/// @brief Fine samples a slot in the rendered waveform, so an offset of an eighth of a sample is an exact grid step.
constexpr std::size_t kGrid = 32UZ;

/// @brief Slots of silence the renderer puts before the burst, so the scan has positions on either side of it.
constexpr std::size_t kLead = 8UZ;

/// @brief The carrier every rendered sample sits on, so nothing downstream can pass by reading a real part.
constexpr float kCarrier = 0.7F;

/// @brief One magnitude stream as the samples it could have been read from, unchanged in magnitude.
[[nodiscard]] std::vector<std::complex<float>> asIq(std::span<const float> magnitudes) {
    std::vector<std::complex<float>> wave;
    wave.reserve(magnitudes.size());
    const std::complex<float> carrier = std::polar(1.F, kCarrier);
    for (const float value : magnitudes) {
        wave.push_back(value * carrier);
    }
    return wave;
}

/**
 * @brief One frame as complex baseband, one sample a slot, sitting @p tau samples away from the sample it is nearest.
 *
 * Rectangular pulses are laid on a grid of `kGrid` samples a slot and averaged over one slot, which is the front end
 * this framer is written for: each sample is half a microsecond of signal integrated. Sampling that grid at one
 * sample a slot delays the burst by @p tau, leaving the frame's first slot at index `lead + tau` — so a scanner
 * reading it at the integer position `lead` has to look @p tau samples ahead to find the shape whole.
 *
 * @p tau in [-0.5, 0.5), a multiple of `1 / kGrid`, so the grid is sampled and never resampled.
 */
[[nodiscard]] std::vector<std::complex<float>> renderIq(std::span<const std::uint8_t> frame, float tau, std::size_t lead = kLead, std::size_t trail = 260UZ) {
    const std::size_t  bits = frame.size() * 8UZ;
    std::vector<float> slots;
    slots.reserve(lead + 16UZ + 2UZ * bits + trail);
    slots.insert(slots.end(), lead, 0.F);
    for (const int pulse : kPreamble) {
        slots.push_back(pulse != 0 ? 1.F : 0.F);
    }
    for (std::size_t bit = 0UZ; bit < bits; ++bit) {
        const bool one = ((frame[bit / 8UZ] >> (7U - bit % 8UZ)) & 1U) != 0U;
        slots.push_back(one ? 1.F : 0.F);
        slots.push_back(one ? 0.F : 1.F);
    }
    slots.insert(slots.end(), trail, 0.F);

    // the running mean over one slot of the grid; the sum is at most `kGrid` ones and the divisor a power of two, so
    // every rendered value is an exact multiple of 1 / kGrid and two equal offsets render bit for bit alike
    std::vector<float> mean(slots.size() * kGrid, 0.F);
    float              running = 0.F;
    for (std::size_t m = 0UZ; m < mean.size(); ++m) {
        running += slots[m / kGrid];
        if (m >= kGrid) {
            running -= slots[(m - kGrid) / kGrid];
        }
        mean[m] = running / static_cast<float>(kGrid);
    }

    const auto                       shift   = static_cast<std::ptrdiff_t>(std::llround(static_cast<double>(tau) * static_cast<double>(kGrid)));
    const std::complex<float>        carrier = std::polar(1.F, kCarrier);
    std::vector<std::complex<float>> wave;
    wave.reserve(slots.size());
    for (std::size_t n = 0UZ; n < slots.size(); ++n) {
        const std::ptrdiff_t at = static_cast<std::ptrdiff_t>((n + 1UZ) * kGrid) - shift - 1;
        if (at < 0 || at >= static_cast<std::ptrdiff_t>(mean.size())) {
            break;
        }
        wave.push_back(mean[static_cast<std::size_t>(at)] * carrier);
    }
    return wave;
}

/// @brief The eight sub-sample offsets a `phase_steps` of eight searches, which is the grid the renderer is swept on.
[[nodiscard]] constexpr float offsetOf(std::size_t step) noexcept { return -0.5F + static_cast<float>(step) / 8.F; }

} // namespace qa_ppm_framer

using namespace qa_ppm_framer;

const boost::ut::suite<"ppm framer"> ppmFramerTests = [] {
    using namespace boost::ut;

    "the anchor decodes identically at every chunking"_test = [] {
        const std::vector<std::uint8_t> anchor = octetsOf(kAnchor);
        std::vector<float>              stream;
        appendZeros(stream, 3UZ); // the first chunk boundaries then fall inside the preamble and inside a bit
        append(stream, render(std::span<const std::uint8_t>(anchor)));
        appendZeros(stream, 37UZ);
        expect(eq(stream.size(), 3UZ + 240UZ + 37UZ));

        for (const std::size_t chunk : {1UZ, 7UZ, 240UZ, 241UZ, 0UZ}) {
            const Scanned run = scan(std::span<const float>(stream), chunk);
            const auto    at  = std::format("chunk {}", chunk);
            expect(eq(run.frames.size(), 1UZ)) << at;
            expect(eq(run.counters.nominations, 1ULL)) << at;
            expect(eq(run.counters.admitted, 1ULL)) << at;
            expect(eq(run.counters.nominations, run.counters.admitted + run.counters.crcFailed + run.counters.shortFormat)) << at;
            if (run.frames.size() != 1UZ) {
                continue;
            }
            expect(that % (run.frames[0UZ].octets == anchor)) << at;
            expect(eq(run.frames[0UZ].bits, 112UZ)) << at;
            expect(eq(run.frames[0UZ].format, 17U)) << at;
            expect(eq(run.frames[0UZ].remainder, 0ULL)) << at;
            expect(eq(run.frames[0UZ].position, 3UZ)) << at;
            expect(run.frames[0UZ].outcome == PpmOutcome::Admitted) << at;
            expect(eq(run.frames[0UZ].strong, 1.F)) << at;
            expect(eq(run.frames[0UZ].weak, 0.F)) << at;
        }

        // and the block over the same stream reports the same identity
        MagFramer    block  = make<MagFramer>(modeSSettings());
        const Framed framed = feed(block, std::span<const float>(stream), 13UZ);
        expect(eq(framed.records.size(), 1UZ));
        expect(eq(framed.nominations, framed.admitted + framed.crcFailed + framed.shortFormat));
        expect(eq(framed.nominations, 1ULL));
        expect(eq(framed.published, 1ULL));
        expect(eq(framed.samples, stream.size()));
        expect(eq(framed.tailDropped, std::uint64_t{37})) << "the positions whose window never completed are dropped and counted";
    };

    "an unconnected output scans and counts, and does not hoard the stream"_test = [] {
        const std::vector<std::uint8_t> anchor = octetsOf(kAnchor);
        std::vector<float>              stream;
        appendZeros(stream, 3UZ);
        append(stream, render(std::span<const std::uint8_t>(anchor)));
        appendZeros(stream, 37UZ);

        MagFramer    block = make<MagFramer>(modeSSettings());
        const Framed seen  = feed(block, std::span<const float>(stream), 13UZ, {}, 64UZ, false);
        expect(eq(seen.records.size(), 0UZ)) << "nothing is written where nothing is connected";
        expect(eq(seen.samples, stream.size()));
        expect(eq(seen.nominations, 1ULL)) << "the scan runs whether or not anyone reads it";
        expect(eq(seen.admitted, 1ULL));
        expect(eq(seen.published, 1ULL));
        expect(eq(seen.held, 37UZ)) << "only the positions whose window never completed stay in the working buffer";
        expect(eq(seen.tailDropped, std::uint64_t{37}));
    };

    "the working buffer is bounded when the sink stops taking records"_test = [] {
        // three hundred anchors end to end, with one record of room a call: the sink saturates on the first frame and
        // the rest of the span has nowhere to go, so the buffer would grow with the stream if nothing bounded it
        const std::vector<std::uint8_t> anchor = octetsOf(kAnchor);
        const std::vector<float>        one    = render(std::span<const std::uint8_t>(anchor));
        std::vector<float>              stream;
        for (std::size_t k = 0UZ; k < 300UZ; ++k) {
            append(stream, std::span<const float>(one));
        }
        expect(eq(stream.size(), 72000UZ));

        MagFramer         block    = make<MagFramer>(modeSSettings());
        const Framed      seen     = feed(block, std::span<const float>(stream), 0UZ, {}, 1UZ);
        const std::size_t capacity = 240UZ + (1UZ << 16U); // one window and the reserve, both fixed at start()
        expect(eq(seen.records.size(), 1UZ)) << "the sink had room for one";
        expect(eq(seen.samples, stream.size()));
        expect(le(seen.held, capacity)) << "the buffer holds one window and its reserve, and no more of the stream";
        expect(eq(seen.overrunDropped, std::uint64_t{stream.size() - 240UZ - capacity})) << "the samples past the bound are dropped and counted";
        expect(eq(seen.nominations, seen.admitted + seen.crcFailed + seen.shortFormat));
    };

    "the five published frames admit in sequence"_test = [] {
        std::vector<float>       stream;
        std::vector<std::size_t> starts;
        for (const std::string_view hex : kPublished) {
            appendZeros(stream, 80UZ); // 40 us of silence between bursts
            starts.push_back(stream.size());
            const std::vector<std::uint8_t> frame = octetsOf(hex);
            append(stream, render(std::span<const std::uint8_t>(frame)));
        }
        appendZeros(stream, 80UZ);

        const std::vector<Record> records = scheduled(stream, modeSSettings());
        expect(eq(records.size(), 5UZ)) << "every published frame is a bare-CRC extended squitter";
        for (std::size_t k = 0UZ; k < std::min(records.size(), starts.size()); ++k) {
            const Record& record = records[k];
            const auto    at     = std::format("frame {}", k);
            expect(that % (record.signal_values == octetsOf(kPublished[k]))) << at;
            expect(that % (record.signal_names == std::vector<std::string>{"payload"})) << at;
            expect(eq(static_cast<std::size_t>(record.extents.at(0)), 14UZ)) << at;
            expect(eq(record.timestamp, std::int64_t{0})) << at;
            expect(eq(metaString(record, "trigger_name"), std::string("mode_s"))) << at;
            expect(eq(metaString(record, "protocol"), std::string("mode_s"))) << at << ": the framing the record was read under";
            expect(eq(metaSize(record, "mode_s_format"), gr::Size_t{17})) << at;
            expect(eq(metaSize(record, "crc_remainder"), gr::Size_t{0})) << at;
            expect(metaBool(record, "crc_ok")) << at;
            expect(eq(metaU64(record, "sample_start"), starts[k] + 16UZ)) << at << ": the first data bit's first sample";
            expect(eq(metaU64(record, "sequence"), k)) << at;
            expect(eq(metaFloat(record, "sample_rate"), 2.0e6F)) << at;
            expect(eq(metaFloat(record, "preamble_strong"), 1.F)) << at;
            expect(eq(metaFloat(record, "preamble_weak"), 0.F)) << at;
        }
    };

    "both gates are real: the preamble's shape and the parity"_test = [] {
        const std::vector<std::uint8_t> anchor = octetsOf(kAnchor);

        for (const std::size_t pulse : {0UZ, 2UZ, 7UZ, 9UZ}) {
            std::vector<float> stream = render(std::span<const std::uint8_t>(anchor));
            stream[pulse]             = 0.F;
            for (const float threshold : {1.01F, 2.F, 100.F}) {
                const Scanned run = scan(std::span<const float>(stream), 0UZ, 1UZ, threshold);
                expect(eq(run.counters.nominations, 0ULL)) << std::format("pulse slot {} zeroed at threshold {}", pulse, threshold);
            }
        }

        // exchanging one bit's two half periods inverts that bit, and the remainder leaves zero
        std::vector<float> flipped = render(std::span<const std::uint8_t>(anchor));
        std::swap(flipped[16UZ + 2UZ * 40UZ], flipped[16UZ + 2UZ * 40UZ + 1UZ]);
        const Scanned damaged = scan(std::span<const float>(flipped));
        expect(eq(damaged.counters.nominations, 1ULL));
        expect(eq(damaged.counters.crcFailed, 1ULL));
        expect(eq(damaged.counters.admitted, 0ULL));

        MagFramer    silent    = make<MagFramer>(modeSSettings());
        const Framed byDefault = feed(silent, std::span<const float>(flipped));
        expect(eq(byDefault.records.size(), 0UZ)) << "a frame the parity refuses is not a record";
        expect(eq(byDefault.crcFailed, 1ULL));

        gr::property_map unchecked  = modeSSettings();
        unchecked["emit_unchecked"] = true;
        MagFramer    loud           = make<MagFramer>(unchecked);
        const Framed published      = feed(loud, std::span<const float>(flipped));
        expect(eq(published.records.size(), 1UZ));
        if (published.records.size() == 1UZ) {
            expect(eq(published.records[0UZ].signal_values.size(), 14UZ));
            expect(!metaBool(published.records[0UZ], "crc_ok"));
            expect(metaHas(published.records[0UZ], "crc_ok"));
            expect(neq(metaSize(published.records[0UZ], "crc_remainder"), gr::Size_t{0}));
        }

        std::vector<float> blinded = render(std::span<const std::uint8_t>(anchor));
        blinded[7UZ]               = 0.F;
        MagFramer    stillSilent   = make<MagFramer>(unchecked);
        const Framed nothing       = feed(stillSilent, std::span<const float>(blinded));
        expect(eq(nothing.records.size(), 0UZ)) << "emit_unchecked publishes refused parity, not an absent preamble";
        expect(eq(nothing.nominations, 0ULL));
    };

    "an admitted frame contains its own window and a refused one yields a sample"_test = [] {
        const std::vector<std::uint8_t> anchor = octetsOf(kAnchor);

        std::vector<float> pair = render(std::span<const std::uint8_t>(anchor));
        append(pair, render(std::span<const std::uint8_t>(anchor)));
        const Scanned back = scan(std::span<const float>(pair));
        expect(eq(back.counters.nominations, 2ULL));
        expect(eq(back.counters.admitted, 2ULL));
        expect(eq(back.frames.size(), 2UZ));
        if (back.frames.size() == 2UZ) {
            expect(eq(back.frames[0UZ].position, 0UZ));
            expect(eq(back.frames[1UZ].position, 240UZ));
            std::vector<std::uint8_t> both = back.frames[0UZ].octets;
            both.insert(both.end(), back.frames[1UZ].octets.begin(), back.frames[1UZ].octets.end());
            expect(eq(both.size(), 28UZ));
        }

        // a nomination whose parity refuses it must not hide the real frame inside its window: its data region is the
        // anchor's own preamble pattern repeated, which reads as a long frame and reduces to a non-zero remainder
        std::vector<float> decoy;
        for (const int pulse : kPreamble) {
            decoy.push_back(pulse != 0 ? 1.F : 0.F);
        }
        for (std::size_t k = 0UZ; k < 24UZ; ++k) { // 24 half-slot pairs read as bits and set the long-format bit
            decoy.push_back(1.F);
            decoy.push_back(0.F);
        }
        std::vector<float> stream = decoy;
        const std::size_t  at     = stream.size();
        append(stream, render(std::span<const std::uint8_t>(anchor)));
        expect(lt(at, 240UZ)) << "the decoy's window still covers the anchor's first sample";

        const Scanned run = scan(std::span<const float>(stream));
        expect(ge(run.counters.crcFailed, 1ULL)) << "the decoy nominates and its parity refuses it";
        expect(eq(run.counters.admitted, 1ULL));
        bool found = false;
        for (const Seen& frame : run.frames) {
            if (frame.outcome == PpmOutcome::Admitted) {
                found = frame.position == at && frame.octets == anchor;
            }
        }
        expect(found) << "the refused nomination advanced one sample and did not consume the real frame";
    };

    "a short frame is counted, published only on request, and carries no crc_ok"_test = [] {
        // downlink format 11, all-call reply, with a zero interrogator identifier: its parity is the bare CRC
        const std::vector<std::uint8_t> allCall = withParity({0x58U, 0x40U, 0xD6U, 0x11U}, 7UZ);
        const std::vector<float>        stream  = padded(std::span<const std::uint8_t>(allCall));
        expect(eq(render(std::span<const std::uint8_t>(allCall)).size(), 16UZ + 112UZ)) << "a short frame is 56 bits over two slots each";

        MagFramer    quiet  = make<MagFramer>(modeSSettings());
        const Framed silent = feed(quiet, std::span<const float>(stream));
        expect(eq(silent.records.size(), 0UZ));
        expect(eq(silent.shortFormat, 1ULL));
        expect(eq(silent.nominations, 1ULL));

        gr::property_map settings = modeSSettings();
        settings["emit_short"]    = true;
        MagFramer    loud         = make<MagFramer>(settings);
        const Framed shown        = feed(loud, std::span<const float>(stream));
        expect(eq(shown.records.size(), 1UZ));
        if (shown.records.size() == 1UZ) {
            expect(eq(shown.records[0UZ].signal_values.size(), 7UZ));
            expect(that % (shown.records[0UZ].signal_values == allCall));
            expect(eq(metaSize(shown.records[0UZ], "mode_s_format"), gr::Size_t{11}));
            expect(eq(metaSize(shown.records[0UZ], "crc_remainder"), gr::Size_t{0}));
            expect(!metaHas(shown.records[0UZ], "crc_ok")) << "the question has no answer on a short frame";
        }

        // downlink format 4, whose parity is the CRC XORed with the aircraft address
        for (const std::uint64_t address : {0x4840D6ULL, 0xABCDEFULL}) {
            const std::vector<std::uint8_t> reply  = withParity({0x20U, 0x00U, 0x11U, 0x22U}, 7UZ, address);
            MagFramer                       framer = make<MagFramer>(settings);
            const Framed                    seen   = feed(framer, std::span<const float>(padded(std::span<const std::uint8_t>(reply))));
            expect(eq(seen.records.size(), 1UZ)) << std::format("address {:06X}", address);
            if (seen.records.size() == 1UZ) {
                expect(eq(metaSize(seen.records[0UZ], "mode_s_format"), gr::Size_t{4}));
                expect(eq(metaSize(seen.records[0UZ], "crc_remainder"), static_cast<gr::Size_t>(address))) << "the reduction is linear, so the XORed address is the remainder";
            }
        }
    };

    "the format field's top bit selects the length"_test = [] {
        gr::property_map settings = modeSSettings();
        settings["emit_short"]    = true;

        struct Case {
            std::uint8_t lead;
            std::size_t  octets;
            gr::Size_t   format;
        };
        for (const Case& sample : {Case{0x80U, 14UZ, 16U}, Case{0x78U, 7UZ, 15U}, Case{0xC0U, 14UZ, 24U}}) {
            std::vector<std::uint8_t> message(sample.octets, std::uint8_t{0});
            message[0UZ]                          = sample.lead;
            message[1UZ]                          = 0x5AU;
            const std::vector<std::uint8_t> frame = withParity(std::move(message), sample.octets);
            MagFramer                       block = make<MagFramer>(settings);
            const Framed                    seen  = feed(block, std::span<const float>(padded(std::span<const std::uint8_t>(frame))));
            const auto                      at    = std::format("format {}", sample.format);
            expect(eq(seen.records.size(), 1UZ)) << at;
            if (seen.records.size() != 1UZ) {
                continue;
            }
            expect(eq(seen.records[0UZ].signal_values.size(), sample.octets)) << at;
            expect(eq(metaSize(seen.records[0UZ], "mode_s_format"), sample.format)) << at;
            expect(eq(metaSize(seen.records[0UZ], "crc_remainder"), gr::Size_t{0})) << at;
        }
    };

    "the rate sets the slot width, and a rate that is not a whole slot is refused"_test = [] {
        const std::vector<std::uint8_t> anchor = octetsOf(kAnchor);
        for (const std::size_t slot : {2UZ, 4UZ}) {
            const float               rate    = 2.0e6F * static_cast<float>(slot);
            const std::vector<Record> records = scheduled(render(std::span<const std::uint8_t>(anchor), slot), modeSSettings(rate));
            const auto                at      = std::format("{} samples a slot", slot);
            expect(eq(records.size(), 1UZ)) << at;
            if (records.size() == 1UZ) {
                expect(that % (records[0UZ].signal_values == anchor)) << at;
                expect(eq(metaU64(records[0UZ], "sample_start"), 16UZ * slot)) << at;
                expect(eq(metaFloat(records[0UZ], "sample_rate"), rate)) << at;
            }
        }

        for (const float rate : {2.4e6F, 3.0e6F, 0.F}) {
            MagFramer block = MagFramer(modeSSettings(rate));
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
            expect(throws([&block] { block.start(); })) << std::format("{} Hz is not an even multiple of 1 MHz", rate);
        }
        MagFramer bare = MagFramer({{"sample_rate", 2.0e6F}});
        bare.settings().init();
        std::ignore = bare.settings().applyStagedParameters();
        expect(throws([&bare] { bare.start(); })) << "profile is required and has no default";

        // a rate tag mid-stream moves the geometry for the samples after it; one that is not a whole slot is refused
        const std::vector<float> first = render(std::span<const std::uint8_t>(anchor), 1UZ);
        std::vector<float>       stream(first);
        appendZeros(stream, 300UZ);
        const std::size_t        change = stream.size();
        const std::vector<float> second = render(std::span<const std::uint8_t>(anchor), 2UZ);
        append(stream, second);
        appendZeros(stream, 600UZ);

        const auto                 rateTag = [](std::size_t at, float rate) { return gr::Tag{at, gr::property_map{{gr::property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()), gr::pmt::Value(rate)}}}; };
        const std::vector<gr::Tag> tags{rateTag(100UZ, 3.0e6F), rateTag(change, 4.0e6F)};
        MagFramer                  block = make<MagFramer>(modeSSettings());
        const Framed               seen  = feed(block, std::span<const float>(stream), 64UZ, std::span<const gr::Tag>(tags));
        expect(eq(seen.rateRefused, 1ULL)) << "3 MS/s leaves the previous rate standing";
        expect(eq(seen.records.size(), 2UZ));
        expect(eq(seen.admitted, 2ULL)) << "one anchor under each geometry, and the change does not forget the first";
        expect(eq(seen.published, 2ULL));
        expect(eq(seen.nominations, seen.admitted + seen.crcFailed + seen.shortFormat)) << "the counter identity holds across a rate change";
        if (seen.records.size() == 2UZ) {
            expect(eq(metaFloat(seen.records[0UZ], "sample_rate"), 2.0e6F));
            expect(eq(metaFloat(seen.records[1UZ], "sample_rate"), 4.0e6F));
            expect(eq(metaU64(seen.records[1UZ], "sample_start"), change + 32UZ));
        }
    };

    "noise nominates rarely and publishes nothing"_test = [] {
        const std::vector<float> noise = rayleigh(2000000UZ, 0x5EEDULL);

        const Scanned atDefault = scan(std::span<const float>(noise));
        expect(eq(atDefault.counters.admitted, 0ULL)) << "no false frame survives the parity";
        expect(lt(atDefault.counters.nominations, 20ULL)) << "noise alone nominates a handful of candidates at most";

        const Scanned atLow = scan(std::span<const float>(noise), 0UZ, 1UZ, 1.5F);
        expect(gt(atLow.counters.nominations, atDefault.counters.nominations)) << "a lower threshold nominates more often";
        expect(eq(atLow.counters.admitted, 0ULL));

        MagFramer    block = make<MagFramer>(modeSSettings());
        const Framed seen  = feed(block, std::span<const float>(noise), 65536UZ);
        expect(eq(seen.published, 0ULL));
        expect(eq(seen.nominations, atDefault.counters.nominations)) << "the block counts what the kernel nominated";
    };

    "tags inside a frame become timing events, and the rest are counted dropped"_test = [] {
        const std::vector<std::uint8_t> anchor = octetsOf(kAnchor);
        std::vector<float>              stream;
        appendZeros(stream, 20UZ);
        append(stream, render(std::span<const std::uint8_t>(anchor)));
        appendZeros(stream, 300UZ);

        const std::vector<gr::Tag> tags{
            gr::Tag{5UZ, {{"before", std::string("dropped")}}},       //
            gr::Tag{20UZ + 4UZ, {{"preamble", std::string("kept")}}}, //
            gr::Tag{20UZ + 40UZ, {{"payload", std::string("kept")}}}, //
        };
        MagFramer    block = make<MagFramer>(modeSSettings());
        const Framed seen  = feed(block, std::span<const float>(stream), 32UZ, std::span<const gr::Tag>(tags));
        expect(eq(seen.records.size(), 1UZ));
        expect(eq(seen.tagsDropped, 1ULL)) << "the tag in the leading zeros belongs to no frame";
        if (seen.records.size() != 1UZ) {
            return;
        }
        const auto& events = seen.records[0UZ].timing_events.at(0UZ);
        expect(eq(events.size(), 2UZ));
        if (events.size() != 2UZ) {
            return;
        }
        expect(eq(events[0UZ].first, std::ptrdiff_t{-12})) << "a preamble tag precedes the payload, and is not clamped";
        expect(events[0UZ].second.contains(gr::property_map::key_type("preamble"))) << "every key verbatim";
        expect(eq(events[1UZ].first, std::ptrdiff_t{24}));
        expect(events[1UZ].second.contains(gr::property_map::key_type("payload")));
    };

    "the counters are complete after every call, and stop() only reports them"_test = [] {
        // GR4 runs a block's stop() on the thread that asked for the stop: requestStop() reaches
        // SchedulerBase::stop(), which walks the graph calling changeStateTo(REQUESTED_STOP), and that invokes this
        // block's stop() while its own worker may still be inside processBulk. So stop() may read the counters and
        // nothing else, which means the counters have to be complete before it is called.
        const std::vector<std::uint8_t> anchor = octetsOf(kAnchor);
        std::vector<float>              stream;
        appendZeros(stream, 20UZ);
        append(stream, render(std::span<const std::uint8_t>(anchor)));
        appendZeros(stream, 37UZ);
        const std::vector<gr::Tag> tags{gr::Tag{5UZ, {{"before", std::string("dropped")}}}};

        MagFramer           block = make<MagFramer>(modeSSettings());
        std::vector<Record> scratch(64UZ);
        for (std::size_t base = 0UZ; base < stream.size();) {
            const std::size_t count = std::min(32UZ, stream.size() - base);
            const auto        first = std::ranges::lower_bound(tags, base, std::ranges::less{}, &gr::Tag::index);
            const auto        last  = std::ranges::lower_bound(tags, base + count, std::ranges::less{}, &gr::Tag::index);

            InputSpan<float>   inSpan(std::span<const float>(stream).subspan(base, count), base, std::span<const gr::Tag>(first, last));
            OutputSpan<Record> outSpan(std::span<Record>(scratch), 0UZ, nullptr, true);
            std::ignore = block.processBulk(inSpan, outSpan);
            base += inSpan.consumed;
        }

        expect(eq(block.nTailDropped, std::uint64_t{37})) << "the undecided remainder is counted by the thread that owns the buffer";
        expect(eq(block.nTagsDropped, std::uint64_t{1})) << "the tag no frame carried is counted by the thread that owns the list";
    };

    "with one phase step the samples decide exactly what their magnitudes do"_test = [] {
        const std::vector<std::uint8_t> anchor = octetsOf(kAnchor);
        std::vector<float>              stream;
        appendZeros(stream, 3UZ);
        append(stream, render(std::span<const std::uint8_t>(anchor)));
        appendZeros(stream, 37UZ);
        const std::vector<std::complex<float>> samples = asIq(std::span<const float>(stream));

        gr::property_map single = modeSSettings();
        single["phase_steps"]   = gr::Size_t{1U};

        MagFramer    mag   = make<MagFramer>(modeSSettings());
        const Framed byMag = feed(mag, std::span<const float>(stream), 13UZ);
        IqFramer     iq    = make<IqFramer>(single);
        const Framed byIq  = feed(iq, std::span<const std::complex<float>>(samples), 13UZ);

        expect(eq(byIq.records.size(), byMag.records.size()));
        expect(eq(byIq.records.size(), 1UZ));
        expect(eq(byIq.candidates, byIq.nominations)) << "with no fine step to refuse a candidate, every candidate is a nomination";
        expect(eq(byIq.nominations, byMag.nominations));
        expect(eq(byIq.admitted, byMag.admitted));
        expect(eq(byIq.samples, byMag.samples));
        expect(eq(byIq.tailDropped, byMag.tailDropped)) << "one phase step leaves the kernel the window it had before there was a fine step";
        expect(eq(byIq.held, byMag.held));
        if (byIq.records.size() != 1UZ || byMag.records.size() != 1UZ) {
            return;
        }
        expect(that % (byIq.records[0UZ].signal_values == byMag.records[0UZ].signal_values));
        expect(eq(metaU64(byIq.records[0UZ], "sample_start"), metaU64(byMag.records[0UZ], "sample_start")));
        expect(eq(metaFloat(byIq.records[0UZ], "preamble_strong"), metaFloat(byMag.records[0UZ], "preamble_strong")));
        expect(eq(metaFloat(byIq.records[0UZ], "sample_phase"), 0.F)) << "no offset was searched, so none is reported";
        expect(eq(metaFloat(byMag.records[0UZ], "sample_phase"), 0.F)) << "a magnitude stream carries nothing to align to";
    };

    "a frame that falls between two samples"_test = [] {
        // Without the fine step a frame is decided where the samples put it. At one sample a slot a pulse offset by
        // tau spills |tau| of itself into the slot behind it, and that slot is the shape test's own denominator, so
        // the ratio the nomination is made on falls to (1 - |tau|) / |tau| and a threshold of two keeps the offsets
        // out to a quarter of a sample and no further.
        constexpr std::array<bool, 8UZ> kPlainAdmits{false, false, true, true, true, true, true, false};

        const std::vector<std::uint8_t> anchor = octetsOf(kAnchor);
        std::array<bool, 8UZ>           plain{};
        std::array<bool, 8UZ>           fine{};

        for (std::size_t step = 0UZ; step < 8UZ; ++step) {
            const float                            tau     = offsetOf(step);
            const std::vector<std::complex<float>> samples = renderIq(std::span<const std::uint8_t>(anchor), tau);
            for (const gr::Size_t steps : {gr::Size_t{1U}, gr::Size_t{8U}}) {
                const auto       at      = std::format("offset {:+.3f} at {} phase steps", tau, steps);
                gr::property_map setting = modeSSettings();
                setting["phase_steps"]   = steps;
                IqFramer     block       = make<IqFramer>(setting);
                const Framed seen        = feed(block, std::span<const std::complex<float>>(samples), 97UZ);

                (steps == 1U ? plain : fine)[step] = seen.records.size() == 1UZ;
                expect(ge(seen.candidates, seen.nominations)) << at << ": a nomination is a candidate the fine step kept";
                if (steps == 1U) {
                    expect(eq(seen.candidates, seen.nominations)) << at;
                }
                expect(le(seen.records.size(), 1UZ)) << at << ": one frame was rendered";
                for (const Record& record : seen.records) {
                    expect(that % (record.signal_values == anchor)) << at;
                    expect(eq(metaFloat(record, "sample_rate"), 2.0e6F)) << at;
                    const float phase = metaFloat(record, "sample_phase");
                    expect(ge(phase, -0.5F)) << at;
                    expect(lt(phase, 0.5F)) << at;
                    if (steps == 1U) {
                        expect(eq(phase, 0.F)) << at << ": one step is the integer position itself";
                    }
                    // the burst's first data sample sits at `kLead + 16 + tau`, and the record's timing is the
                    // integer start plus the offset, never the integer start alone
                    const float timing = static_cast<float>(metaU64(record, "sample_start")) + phase;
                    expect(lt(std::abs(timing - (static_cast<float>(kLead + 16UZ) + tau)), 0.5F)) << at << ": sample_start plus sample_phase is where the frame is";
                }
            }
        }

        const auto listed = [](const std::array<bool, 8UZ>& set) {
            std::string text;
            for (std::size_t step = 0UZ; step < set.size(); ++step) {
                if (set[step]) {
                    std::format_to(std::back_inserter(text), "{}{:+.3f}", text.empty() ? "" : " ", offsetOf(step));
                }
            }
            return text.empty() ? std::string("none") : text;
        };
        expect(that % (plain == kPlainAdmits)) << std::format("the integer decision admitted {}", listed(plain));
        for (std::size_t step = 0UZ; step < 8UZ; ++step) {
            expect(!plain[step] || fine[step]) << std::format("the fine step refused offset {:+.3f}, which the integer decision kept", offsetOf(step));
        }
    };

    "the tail, a tag and the buffer's bound hold on the complex instantiation"_test = [] {
        const std::vector<std::uint8_t> anchor = octetsOf(kAnchor);
        std::vector<float>              stream;
        appendZeros(stream, 20UZ);
        append(stream, render(std::span<const std::uint8_t>(anchor)));
        appendZeros(stream, 37UZ);
        const std::vector<std::complex<float>> samples = asIq(std::span<const float>(stream));
        const std::vector<gr::Tag>             tags{
            gr::Tag{5UZ, {{"before", std::string("dropped")}}},       //
            gr::Tag{20UZ + 40UZ, {{"payload", std::string("kept")}}}, //
        };

        gr::property_map single = modeSSettings();
        single["phase_steps"]   = gr::Size_t{1U};
        IqFramer     block      = make<IqFramer>(single);
        const Framed seen       = feed(block, std::span<const std::complex<float>>(samples), 32UZ, std::span<const gr::Tag>(tags));
        expect(eq(seen.records.size(), 1UZ));
        expect(eq(seen.tailDropped, std::uint64_t{37})) << "the positions whose window never completed are dropped and counted";
        expect(eq(seen.tagsDropped, 1ULL)) << "the tag in the leading zeros belongs to no frame";
        if (seen.records.size() == 1UZ) {
            const auto& events = seen.records[0UZ].timing_events.at(0UZ);
            expect(eq(events.size(), 1UZ));
            if (events.size() == 1UZ) {
                expect(eq(events[0UZ].first, std::ptrdiff_t{24}));
                expect(events[0UZ].second.contains(gr::property_map::key_type("payload")));
            }
        }

        // three hundred anchors end to end with one record of room a call: the sink saturates on the first frame and
        // the buffer would grow with the stream if nothing bounded it, and the magnitudes beside it with the buffer
        const std::vector<float> one = render(std::span<const std::uint8_t>(anchor));
        std::vector<float>       many;
        for (std::size_t k = 0UZ; k < 300UZ; ++k) {
            append(many, std::span<const float>(one));
        }
        const std::vector<std::complex<float>> loud = asIq(std::span<const float>(many));

        IqFramer          bounded  = make<IqFramer>(modeSSettings());
        const Framed      capped   = feed(bounded, std::span<const std::complex<float>>(loud), 0UZ, {}, 1UZ);
        const std::size_t capacity = bounded._scanner.phasedWindowSamples() + (1UZ << 16U);
        expect(eq(capped.records.size(), 1UZ)) << "the sink had room for one";
        expect(eq(capped.samples, std::uint64_t{loud.size()}));
        expect(le(capped.held, capacity)) << "the buffer holds one window and its reserve, and no more of the stream";
        expect(eq(capped.held, bounded._mags.size())) << "the samples and their magnitudes are trimmed together";
        expect(gt(capped.overrunDropped, 0ULL)) << "the samples past the bound are dropped and counted";
        expect(le(capped.overrunDropped + std::uint64_t{capped.held}, std::uint64_t{loud.size()})) << "nothing is counted dropped twice, and nothing both held and dropped";
    };
};

int main() { /* not needed for UT */ }
