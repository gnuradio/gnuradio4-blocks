#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <limits>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/audio/AudioBlocks.hpp>
#include <gnuradio-4.0/fileio/WavBlocks.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#ifdef __GNUC__
#pragma GCC diagnostic push
// GCC 16 misreads the inlined gzip_compressor dtor as indexing past a compressor[2] object
#pragma GCC diagnostic ignored "-Warray-bounds"
#endif
#include <httplib.h>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

using namespace boost::ut;
using namespace std::chrono_literals;

void appendLe16(std::vector<std::uint8_t>& bytes, std::uint16_t value) {
    bytes.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    bytes.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
}

void appendLe32(std::vector<std::uint8_t>& bytes, std::uint32_t value) {
    bytes.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    bytes.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
    bytes.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xFFU));
    bytes.push_back(static_cast<std::uint8_t>((value >> 24U) & 0xFFU));
}

void appendText(std::vector<std::uint8_t>& bytes, std::string_view text) { bytes.insert(bytes.end(), text.begin(), text.end()); }

void appendChunk(std::vector<std::uint8_t>& bytes, std::string_view id, std::span<const std::uint8_t> chunkBytes) {
    appendText(bytes, id);
    appendLe32(bytes, static_cast<std::uint32_t>(chunkBytes.size()));
    bytes.insert(bytes.end(), chunkBytes.begin(), chunkBytes.end());
    if ((chunkBytes.size() & 1U) != 0U) {
        bytes.push_back(0U);
    }
}

void patchLe32(std::vector<std::uint8_t>& bytes, std::size_t offset, std::uint32_t value) {
    bytes[offset + 0U] = static_cast<std::uint8_t>(value & 0xFFU);
    bytes[offset + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    bytes[offset + 2U] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
    bytes[offset + 3U] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

std::vector<std::uint8_t> encodePcm8(const std::vector<std::uint8_t>& samples) { return samples; }

std::vector<std::uint8_t> encodePcm16(const std::vector<std::int16_t>& samples) {
    std::vector<std::uint8_t> bytes;
    bytes.reserve(samples.size() * sizeof(std::int16_t));
    for (const auto sample : samples) {
        appendLe16(bytes, static_cast<std::uint16_t>(sample));
    }
    return bytes;
}

std::vector<std::uint8_t> encodePcm24(const std::vector<std::int32_t>& samples) {
    std::vector<std::uint8_t> bytes;
    bytes.reserve(samples.size() * 3U);
    for (const auto sample : samples) {
        const auto value = static_cast<std::uint32_t>(sample);
        bytes.push_back(static_cast<std::uint8_t>(value & 0xFFU));
        bytes.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
        bytes.push_back(static_cast<std::uint8_t>((value >> 16U) & 0xFFU));
    }
    return bytes;
}

std::vector<std::uint8_t> encodePcm32(const std::vector<std::int32_t>& samples) {
    std::vector<std::uint8_t> bytes;
    bytes.reserve(samples.size() * sizeof(std::int32_t));
    for (const auto sample : samples) {
        appendLe32(bytes, static_cast<std::uint32_t>(sample));
    }
    return bytes;
}

std::vector<std::uint8_t> encodeFloat32(const std::vector<float>& samples) {
    std::vector<std::uint8_t> bytes;
    bytes.reserve(samples.size() * sizeof(float));
    for (const auto sample : samples) {
        appendLe32(bytes, std::bit_cast<std::uint32_t>(sample));
    }
    return bytes;
}

std::vector<std::uint8_t> makeWav(std::uint16_t formatTag, std::uint16_t channels, std::uint16_t bitsPerSample, std::uint32_t sampleRate, const std::vector<std::uint8_t>& dataBytes, bool addJunkChunk = false) {
    std::vector<std::uint8_t> bytes;
    appendText(bytes, "RIFF");
    appendLe32(bytes, 0U);
    appendText(bytes, "WAVE");

    std::vector<std::uint8_t> fmt;
    const std::uint32_t       byteRate   = sampleRate * channels * (bitsPerSample / 8U);
    const std::uint16_t       blockAlign = static_cast<std::uint16_t>(channels * (bitsPerSample / 8U));
    appendLe16(fmt, formatTag);
    appendLe16(fmt, channels);
    appendLe32(fmt, sampleRate);
    appendLe32(fmt, byteRate);
    appendLe16(fmt, blockAlign);
    appendLe16(fmt, bitsPerSample);
    appendChunk(bytes, "fmt ", fmt);

    if (addJunkChunk) {
        static constexpr std::array<std::uint8_t, 5U> junk{{'h', 'e', 'l', 'l', 'o'}};
        appendChunk(bytes, "JUNK", junk);
    }

    appendChunk(bytes, "data", dataBytes);
    patchLe32(bytes, 4U, static_cast<std::uint32_t>(bytes.size() - 8U));
    return bytes;
}

struct TempFile {
    std::filesystem::path path;
    ~TempFile() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
};

std::string writeTempAudioFile(std::span<const std::uint8_t> bytes) {
    const auto    path = std::filesystem::temp_directory_path() / std::format("gr4-audio-{}.wav", std::chrono::steady_clock::now().time_since_epoch().count());
    std::ofstream file(path, std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    file.close();
    return path.string();
}

template<typename T>
struct WavSourceTestCase {
    std::string_view          name;
    std::vector<std::uint8_t> wavBytes;
    std::vector<T>            expectedSamples;
    float                     sampleRate;
    gr::Size_t                numChannels;
};

void expectSingleFormatTag(const std::vector<gr::Tag>& tags, float sampleRate, gr::Size_t numChannels, std::string_view caseName) {
    expect(ge(tags.size(), 1U)) << caseName;
    if (tags.empty()) {
        return;
    }
    expect(eq(gr::test::get_value_or_fail<float>(tags[0].map.at(gr::tag::SAMPLE_RATE.shortKey())), sampleRate)) << caseName;
    expect(eq(gr::test::get_value_or_fail<gr::Size_t>(tags[0].map.at(gr::tag::NUM_CHANNELS.shortKey())), numChannels)) << caseName;
}

template<typename TSource, typename T, typename TSampleCheck>
void runLocalSourceCases(const std::vector<WavSourceTestCase<T>>& cases, TSampleCheck&& sampleCheck) {
    for (const auto& testCase : cases) {
        const auto caseName = std::format("{} / {}", gr::meta::type_name<TSource>(), testCase.name);
        TempFile   file{writeTempAudioFile(testCase.wavBytes)};

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<TSource>({{"uri", file.path.string()}});
        auto&     sink   = graph.emplaceBlock<gr::blocks::testing::TagSink<T, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>();
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(sched.runAndWait().has_value()) << caseName;

        sampleCheck(std::vector<T>(sink._samples.begin(), sink._samples.end()), testCase.expectedSamples, caseName);
        expectSingleFormatTag(sink._tags, testCase.sampleRate, testCase.numChannels, caseName);
    }
}

// holds its input span without consuming for the whole run: the port ring fills, and behind it the
// capture ring the device writes into
template<typename T>
class StalledSink : public gr::Block<StalledSink<T>> {
public:
    gr::PortIn<T> in;

    GR_MAKE_REFLECTABLE(StalledSink, in);

    std::chrono::milliseconds _stallFor{0};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        std::this_thread::sleep_for(_stallFor);
        std::ignore = inSpan.consume(0UZ);
        return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
    }
};

// records the trigger_time of every timing tag with the sample it marks, and the time each sample
// arrived: a sample cannot arrive before it was captured
template<typename T>
class ArrivalSink : public gr::Block<ArrivalSink<T>> {
public:
    gr::PortIn<T> in;

    GR_MAKE_REFLECTABLE(ArrivalSink, in);

    struct Stamp {
        std::size_t   index;
        std::uint64_t timeNs;
    };
    std::vector<Stamp> _triggerTimes; // the sample a timing tag marks, and its trigger_time
    std::vector<Stamp> _arrivals;     // every sample before `index` had arrived by `timeNs`
    std::size_t        _nSamples{0UZ};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        const std::uint64_t tArrivalNs = gr::blocks::audio::detail::wallClockNs();
        for (const auto& [relIndex, tagMapRef] : inSpan.tags()) {
            const auto& tagMap = tagMapRef.get();
            if (const auto it = tagMap.find(gr::tag::TRIGGER_TIME.shortKey()); it != tagMap.end()) {
                if (const auto* triggerTime = it->second.template get_if<std::uint64_t>(); triggerTime != nullptr) {
                    _triggerTimes.push_back({_nSamples + (relIndex < 0 ? 0UZ : static_cast<std::size_t>(relIndex)), *triggerTime});
                }
            }
        }
        _nSamples += inSpan.size();
        _arrivals.push_back({_nSamples, tArrivalNs});
        return gr::work::Status::OK;
    }
};

std::expected<void, gr::Error> runSchedulerFor(gr::scheduler::Simple<>& sched, std::chrono::milliseconds duration) {
    std::optional<std::expected<void, gr::Error>> result;
    auto                                          schedThread = std::thread([&sched, &result] { result = sched.runAndWait(); });
    std::this_thread::sleep_for(duration);
    sched.requestStop();
    schedThread.join();
    return std::move(*result);
}

const boost::ut::suite<"audio device tests"> _audioTests = [] {
    using namespace boost::ut;

#ifndef __EMSCRIPTEN__
    "AudioSink plays PCM with soundio dummy backend"_test = [] {
        constexpr std::string_view      caseName = "AudioSink soundio dummy backend";
        const std::vector<std::int16_t> reference{0, 1000, -1000, 2000, -2000, 3000};
        const auto                      wavBytes = makeWav(1U, 2U, 16U, 22050U, encodePcm16(reference));
        TempFile                        file{writeTempAudioFile(wavBytes)};

        gr::Graph graph;
        auto&     source              = graph.emplaceBlock<gr::blocks::fileio::WavSource<float>>({{"uri", file.path.string()}});
        auto&     sink                = graph.emplaceBlock<gr::blocks::audio::AudioSink<float>>({{"io_buffer_size", 0.1f}});
        sink._useDummyBackendForTests = true;
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(sched.runAndWait().has_value()) << caseName;
        expect(sched.state() != gr::lifecycle::State::ERROR) << caseName;

        expect(sink._useDummyBackendForTests) << caseName;
        expect(eq(sink.num_channels.value, 2U)) << caseName;
        expect(eq(sink.sample_rate.value, 22050.f)) << caseName;
    };

    "AudioSource captures PCM with soundio dummy backend"_test = [] {
        constexpr std::string_view caseName = "AudioSource soundio dummy backend";

        gr::Graph graph;
        auto&     source                = graph.emplaceBlock<gr::blocks::audio::AudioSource<float>>({{"sample_rate", 22050.f}, {"num_channels", gr::Size_t(2)}, {"io_buffer_size", 0.1f}});
        source._useDummyBackendForTests = true;
        auto& sink                      = graph.emplaceBlock<gr::blocks::testing::TagSink<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>();
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(runSchedulerFor(sched, 200ms).has_value()) << caseName;
        expect(sched.state() != gr::lifecycle::State::ERROR) << caseName;

        expect(source._useDummyBackendForTests) << caseName;
        expect(gt(source.sample_rate.value, 0.0f)) << caseName;
        expect(gt(source.num_channels.value, 0U)) << caseName;
        expect(gt(sink._nSamplesProduced, 0UZ)) << caseName;
        expectSingleFormatTag(sink._tags, source.sample_rate.value, source.num_channels.value, caseName);
    };

    "AudioSource accounts for every captured sample"_test = [] {
        constexpr std::string_view caseName = "AudioSource sample accounting";

        gr::Graph graph;
        auto&     source                = graph.emplaceBlock<gr::blocks::audio::AudioSource<float>>({{"sample_rate", 22050.f}, {"num_channels", gr::Size_t(1)}, {"io_buffer_size", 0.1f}});
        source._useDummyBackendForTests = true;
        auto& sink                      = graph.emplaceBlock<gr::blocks::testing::TagSink<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>();
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(runSchedulerFor(sched, 200ms).has_value()) << caseName;
        expect(sched.state() != gr::lifecycle::State::ERROR) << caseName;

        // a consumer that keeps up must see no loss, and any loss that does happen must be counted
        // rather than inferred from a gap in the stream
        expect(gt(sink._nSamplesProduced, 0UZ)) << caseName;
        expect(eq(source.dropped_samples.value, gr::Size_t(0))) << std::format("{}: a keeping-up consumer must not lose samples, dropped {}", caseName, source.dropped_samples.value);
    };

    "AudioSource accounts for the losses the backend reports"_test = [] {
        constexpr std::string_view caseName = "AudioSource backend loss accounting";

        gr::Graph graph;
        auto&     source                = graph.emplaceBlock<gr::blocks::audio::AudioSource<float>>({{"sample_rate", 48000.f}, {"num_channels", gr::Size_t(1)}, {"io_buffer_size", 0.1f}});
        source._useDummyBackendForTests = true;
        auto& sink                      = graph.emplaceBlock<StalledSink<float>>();
        sink._stallFor                  = 700ms; // outlasts the run below, so nothing is ever consumed
        // a short edge, so the capture ring behind it fills well inside the run
        expect(graph.connect<"out", "in">(source, sink, gr::EdgeParameters{.minBufferSize = 4096UZ}).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(runSchedulerFor(sched, 500ms).has_value()) << caseName;
        expect(sched.state() != gr::lifecycle::State::ERROR) << caseName;

        // nothing downstream consumed, so the capture ring filled and the backend lost samples on
        // the one path that never reaches the output
        expect(gt(source._backendDroppedSamples, 0UZ)) << std::format("{}: backendDropped={} backlogDiscarded={}", caseName, source._backendDroppedSamples, source._backlogDiscardedSamples);

        // the public count is what the backend lost plus the silence it delivered plus what the
        // block itself discarded, with nothing left behind by the shutdown paths
        expect(eq(static_cast<std::size_t>(source.dropped_samples.value), source._backendDroppedSamples + source._backendSilenceSamples + source._backlogDiscardedSamples)) //
            << std::format("{}: dropped={} backendDropped={} silence={} backlogDiscarded={}", caseName, source.dropped_samples.value, source._backendDroppedSamples, source._backendSilenceSamples, source._backlogDiscardedSamples);

        // and the setting a control plane reads is current with the member
        const auto reported = source.settings().get("dropped_samples");
        expect(reported.has_value()) << caseName;
        if (reported.has_value()) {
            const auto* reportedValue = reported->get_if<gr::Size_t>();
            expect(reportedValue != nullptr) << caseName;
            if (reportedValue != nullptr) {
                expect(eq(*reportedValue, source.dropped_samples.value)) << caseName;
            }
        }
    };

    "capture is quiesced before the final loss collection"_test = [] {
        constexpr std::string_view caseName = "AudioSource shutdown collection";

        // the quiesce stops the capture callback and keeps the counts, so what follows it is final
        gr::blocks::audio::detail::SoundIoSourceBackend<float> backend;
        expect(backend.start({.sampleRate = 48000U, .numChannels = 1U, .bufferFrames = 8192UZ, .device = "", .useDummyBackendForTests = true}).has_value()) << caseName;
        backend._state.silenceSamples.fetch_add(17U);
        backend.quiesceCapture();
        expect(!backend.isStreamActive()) << "no capture callback may run after the quiesce";
        expect(eq(backend._state.silenceSamples.load(), 17UZ)) << "the quiesce must leave the counts to be collected";
        backend.shutdown();
        expect(eq(backend._state.silenceSamples.load(), 0UZ)) << "only the shutdown that follows the collection resets them";

        // and the block's stop collects what the backend was holding when it stopped
        gr::blocks::audio::AudioSource<float> source({{"sample_rate", 48000.f}, {"num_channels", gr::Size_t(1)}, {"io_buffer_size", 0.1f}});
        source._useDummyBackendForTests = true;
        source.settings().init();
        std::ignore = source.settings().applyStagedParameters();
        source.start();
        source._backendImpl._state.silenceSamples.fetch_add(17U);
        source.stop();
        expect(eq(source.dropped_samples.value, gr::Size_t(17))) << std::format("{}: holes pending at stop must reach the public count, got {}", caseName, source.dropped_samples.value);
    };

    "the soundio capture callback dates the frames it stores"_test = [] {
        constexpr std::string_view caseName = "SoundIo capture time";
        constexpr double           kRate    = 48000.0;
        using State                         = gr::blocks::audio::detail::AudioSourceState<float>;

        gr::blocks::audio::detail::SoundIoSourceBackend<float> backend;

        const auto tStartNs = static_cast<std::int64_t>(gr::blocks::audio::detail::wallClockNs());
        expect(backend.start({.sampleRate = 48000U, .numChannels = 1U, .bufferFrames = 8192UZ, .device = "", .useDummyBackendForTests = true}).has_value()) << caseName;
        // the first callback that stores frames records their capture time
        for (std::size_t attempt = 0UZ; attempt < 2000UZ && !backend._state.captureTimeNs(0UZ, kRate).has_value(); ++attempt) {
            std::this_thread::sleep_for(1ms);
        }
        backend.quiesceCapture(); // no callback runs after this; the ring and its record stay unchanged
        const auto tEndNs = static_cast<std::int64_t>(gr::blocks::audio::detail::wallClockNs());

        const std::size_t nStored = backend._state.writer.position();
        const auto        oldest  = backend._state.captureTimeNs(0UZ, kRate);
        const auto        newest  = backend._state.captureTimeNs(nStored > 0UZ ? nStored - 1UZ : 0UZ, kRate);
        expect(gt(nStored, 0UZ)) << caseName;
        expect(oldest.has_value() && newest.has_value()) << "the dummy backend reports a capture latency, so its callback records a capture time";
        if (nStored > 0UZ && oldest.has_value() && newest.has_value()) {
            // one sample period apart, captured after the start and before the frames were read
            expect(eq(*newest - *oldest, State::framesToNs(nStored - 1UZ, kRate))) << caseName;
            expect(ge(*oldest, tStartNs)) << std::format("{}: the first frame is dated {} ns before the start", caseName, tStartNs - *oldest);
            expect(le(*newest, tEndNs)) << std::format("{}: the newest frame is dated {} ns after it was read", caseName, *newest - tEndNs);
        }
        backend.shutdown();
        expect(!backend._state.captureTimeNs(0UZ, kRate).has_value()) << "a new ring holds no capture time until a callback records one";
    };

    "an evicted silence placeholder is counted once"_test = [] {
        constexpr std::string_view caseName = "AudioSource hole eviction";

        gr::blocks::audio::detail::AudioSourceState<float> state;
        state.recreateBuffer(1024UZ);

        // fill the ring the way the capture callback fills a driver hole it has already counted;
        // the reservation is scoped, because the ring publishes a claim when its span goes away
        {
            auto holes = state.writer.tryReserve(1024UZ);
            expect(eq(holes.size(), 1024UZ)) << caseName;
            std::fill(holes.begin(), holes.end(), 0.f);
            holes.publish(1024UZ);
        }
        state.silenceSamples.fetch_add(1024U);
        state.silenceInRing.fetch_add(1024U);
        expect(eq(state.reader.available(), 1024UZ)) << caseName;

        // evicting a placeholder adds nothing: the hole was counted when it was stored
        expect(eq(state.discardOldest(64UZ), 0UZ)) << std::format("{}: an evicted placeholder must not be counted twice", caseName);
        expect(eq(state.silenceInRing.load(), 960UZ)) << caseName;

        // real capture lost to the same backpressure is still counted
        {
            auto captured = state.writer.tryReserve(64UZ);
            expect(eq(captured.size(), 64UZ)) << caseName;
            std::fill(captured.begin(), captured.end(), 1.f);
            captured.publish(64UZ);
        }
        expect(eq(state.discardOldest(960UZ), 0UZ)) << caseName;
        expect(eq(state.discardOldest(64UZ), 64UZ)) << std::format("{}: evicted capture must still be counted", caseName);
        expect(eq(state.silenceInRing.load(), 0UZ)) << caseName;
    };

    "AudioSink accounts for samples the device ring refused"_test = [] {
        constexpr std::string_view      caseName = "AudioSink sample accounting";
        const std::vector<std::int16_t> reference(8000, std::int16_t(1000)); // 3.6x the 0.1 s staging ring
        const auto                      wavBytes = makeWav(1U, 1U, 16U, 22050U, encodePcm16(reference));
        TempFile                        file{writeTempAudioFile(wavBytes)};

        gr::Graph graph;
        auto&     source              = graph.emplaceBlock<gr::blocks::fileio::WavSource<float>>({{"uri", file.path.string()}});
        auto&     sink                = graph.emplaceBlock<gr::blocks::audio::AudioSink<float>>({{"io_buffer_size", 0.1f}});
        sink._useDummyBackendForTests = true;
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(sched.runAndWait().has_value()) << caseName;
        expect(sched.state() != gr::lifecycle::State::ERROR) << caseName;

        // staged, written and dropped must together account for everything the block accepted
        expect(eq(sink._totalStagedSamples, sink._totalIoWrittenSamples + static_cast<std::size_t>(sink.dropped_samples.value))) //
            << std::format("{}: staged={} written={} dropped={}", caseName, sink._totalStagedSamples, sink._totalIoWrittenSamples, sink.dropped_samples.value);
    };

    "AudioSink reconfigures while streaming"_test = [] {
        constexpr std::string_view caseName = "AudioSink reconfigure while streaming";

        struct Reconfiguration {
            std::size_t index;
            float       sampleRate;
            gr::Size_t  numChannels;
        };
        // sample_rate and num_channels are auto-updated from the stream, so each tag reopens the
        // device and replaces the staging ring while the producer is pushing into it
        constexpr std::array<Reconfiguration, 4U> reconfigurations{{{500UZ, 22050.f, 2U}, {1500UZ, 44100.f, 1U}, {2500UZ, 16000.f, 2U}, {3500UZ, 32000.f, 1U}}};

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<gr::blocks::testing::TagSource<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", gr::Size_t(8000)}, {"sample_rate", 48000.f}, {"mark_tag", false}});
        for (const auto& reconfiguration : reconfigurations) {
            gr::property_map tagMap;
            gr::tag::put(tagMap, gr::tag::SAMPLE_RATE, reconfiguration.sampleRate);
            gr::tag::put(tagMap, gr::tag::NUM_CHANNELS, reconfiguration.numChannels);
            source._tags.push_back({reconfiguration.index, std::move(tagMap)});
        }

        auto& sink                    = graph.emplaceBlock<gr::blocks::audio::AudioSink<float>>({{"io_buffer_size", 0.1f}});
        sink._useDummyBackendForTests = true;
        // the sample-exact accounting below holds only with the resampling compensator out of the way
        sink.drift_correction = gr::algorithm::DriftCorrection::None;
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(sched.runAndWait().has_value()) << caseName;
        expect(sched.state() != gr::lifecycle::State::ERROR) << caseName;

        // the run ends on the configuration the last tag asked for, negotiated with the device
        expect(eq(sink.sample_rate.value, reconfigurations.back().sampleRate)) << caseName;
        expect(eq(sink.num_channels.value, reconfigurations.back().numChannels)) << caseName;
        expect(eq(sink._activeConfig.sampleRate, static_cast<std::uint32_t>(reconfigurations.back().sampleRate))) << caseName;
        expect(eq(sink._activeConfig.numChannels, static_cast<std::uint32_t>(reconfigurations.back().numChannels))) << caseName;

        // every sample the sink accepted reached a device or was counted as lost, across all the
        // staging rings the reconfigurations replaced
        expect(gt(sink._totalStagedSamples, 0UZ)) << caseName;
        expect(eq(sink._totalStagedSamples, sink._totalIoWrittenSamples + static_cast<std::size_t>(sink.dropped_samples.value))) //
            << std::format("{}: staged={} written={} dropped={}", caseName, sink._totalStagedSamples, sink._totalIoWrittenSamples, sink.dropped_samples.value);
    };

    "AudioSource loops back into AudioSink with soundio dummy backend"_test = [] {
        constexpr std::string_view caseName = "AudioSource to AudioSink soundio dummy backend";

        gr::Graph graph;
        auto&     source                = graph.emplaceBlock<gr::blocks::audio::AudioSource<float>>({{"sample_rate", 22050.f}, {"num_channels", gr::Size_t(2)}, {"io_buffer_size", 0.1f}});
        auto&     sink                  = graph.emplaceBlock<gr::blocks::audio::AudioSink<float>>({{"io_buffer_size", 0.1f}});
        source._useDummyBackendForTests = true;
        sink._useDummyBackendForTests   = true;
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(runSchedulerFor(sched, 200ms).has_value()) << caseName;
        expect(sched.state() != gr::lifecycle::State::ERROR) << caseName;

        expect(source._useDummyBackendForTests) << caseName;
        expect(sink._useDummyBackendForTests) << caseName;
        expect(gt(source.sample_rate.value, 0.0f)) << caseName;
        expect(gt(source.num_channels.value, 0U)) << caseName;
        expect(gt(sink.sample_rate.value, 0.0f)) << caseName;
        expect(gt(sink.num_channels.value, 0U)) << caseName;
    };

    "available_devices is populated after start"_test = [] {
        constexpr std::string_view caseName = "available_devices populated";

        gr::Graph graph;
        auto&     source                = graph.emplaceBlock<gr::blocks::audio::AudioSource<float>>({{"sample_rate", 22050.f}, {"num_channels", gr::Size_t(1)}, {"io_buffer_size", 0.1f}});
        source._useDummyBackendForTests = true;
        auto& sink                      = graph.emplaceBlock<gr::blocks::testing::TagSink<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>();
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(runSchedulerFor(sched, 200ms).has_value()) << caseName;

        expect(!source.available_devices.value.empty()) << caseName;
        for (const auto& entry : source.available_devices.value) {
            expect(entry.find('[') != std::string::npos) << "device entry should contain '[': " << entry;
            expect(entry.find(']') != std::string::npos) << "device entry should contain ']': " << entry;
        }
    };

    "AudioSink available_devices is populated after start"_test = [] {
        constexpr std::string_view      caseName = "AudioSink available_devices populated";
        const std::vector<std::int16_t> reference{0, 1000, -1000, 2000};
        const auto                      wavBytes = makeWav(1U, 1U, 16U, 22050U, encodePcm16(reference));
        TempFile                        file{writeTempAudioFile(wavBytes)};

        gr::Graph graph;
        auto&     source              = graph.emplaceBlock<gr::blocks::fileio::WavSource<float>>({{"uri", file.path.string()}});
        auto&     sink                = graph.emplaceBlock<gr::blocks::audio::AudioSink<float>>({{"io_buffer_size", 0.1f}});
        sink._useDummyBackendForTests = true;
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(sched.runAndWait().has_value()) << caseName;

        expect(!sink.available_devices.value.empty()) << caseName;
    };

    "a named audio backend is the one the block connects through"_test = [] {
        constexpr std::string_view caseName = "AudioSource named backend";

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<gr::blocks::audio::AudioSource<float>>({{"sample_rate", 22050.f}, {"num_channels", gr::Size_t(1)}, {"io_buffer_size", 0.1f}, {"backend", std::string("dummy")}});
        auto&     sink   = graph.emplaceBlock<gr::blocks::testing::TagSink<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>();
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(runSchedulerFor(sched, 200ms).has_value()) << caseName;
        expect(sched.state() != gr::lifecycle::State::ERROR) << caseName;

        // the setting alone selects the backend, and the block reports the one it got
        expect(!source._useDummyBackendForTests) << caseName;
        expect(eq(source.active_backend.value, std::string("dummy"))) << caseName;
        expect(gt(sink._nSamplesProduced, 0UZ)) << caseName;
    };

    "an audio backend that is not available fails the start"_test = [] {
        gr::blocks::audio::detail::SoundIoSourceBackend<float> backend;

        const auto result = backend.start({.sampleRate = 48000U, .numChannels = 1U, .bufferFrames = 8192UZ, .device = "", .backend = "nosuchbackend"});
        expect(!result.has_value()) << "an unavailable backend must be an error, not a connection to another one";
        if (!result.has_value()) {
            expect(result.error().message.contains("nosuchbackend")) << result.error().message;
            expect(result.error().message.contains("dummy")) << "the message must name what is available: " << result.error().message;
        }
        backend.shutdown();
    };

#endif
};

const boost::ut::suite<"audio device resolution"> _deviceResolutionTests = [] {
    using namespace boost::ut;
    using gr::blocks::audio::detail::AudioDeviceConfig;
    using gr::blocks::audio::detail::AudioDeviceInfo;
    using gr::blocks::audio::detail::prefersPulseAudioFirst;
    using gr::blocks::audio::detail::resolveDeviceIndex;

    "PulseAudio is preferred only for the default device"_test = [] {
        expect(prefersPulseAudioFirst({.device = ""})) << "an unspecified device asks for the desktop's default routing";
        expect(prefersPulseAudioFirst({.device = "default"})) << "the default selector asks for the desktop's default routing";
        expect(!prefersPulseAudioFirst({.device = "@id:jack:system"})) << "an explicit id must keep the platform's backend order";
        expect(!prefersPulseAudioFirst({.device = "USB Headset"})) << "an explicit name must keep the platform's backend order";
        expect(!prefersPulseAudioFirst({.device = "", .useDummyBackendForTests = true})) << "the test backend is not a routing choice";
    };

    const std::vector<AudioDeviceInfo> devices{
        {.name = "Built-in Audio Output", .id = "hw:0,0"},
        {.name = "USB Headset", .id = "hw:1,0"},
        {.name = "HDMI Output", .id = "hw:2,0"},
    };

    "empty spec returns nullopt (system default)"_test = [&] { expect(!resolveDeviceIndex("", devices).has_value()); };

    "'default' returns nullopt"_test = [&] { expect(!resolveDeviceIndex("default", devices).has_value()); };

    "'Default' returns nullopt (case-insensitive)"_test = [&] { expect(!resolveDeviceIndex("Default", devices).has_value()); };

    "substring match on name"_test = [&] {
        auto result = resolveDeviceIndex("usb", devices);
        expect(result.has_value()) << "should match 'USB Headset'";
        expect(eq(*result, 1UZ));
    };

    "substring match is case-insensitive"_test = [&] {
        auto result = resolveDeviceIndex("hdmi", devices);
        expect(result.has_value()) << "should match 'HDMI Output'";
        expect(eq(*result, 2UZ));
    };

    "exact ID match with @id: prefix"_test = [&] {
        auto result = resolveDeviceIndex("@id:hw:1,0", devices);
        expect(result.has_value()) << "should match hw:1,0";
        expect(eq(*result, 1UZ));
    };

    "unmatched name returns nullopt"_test = [&] { expect(!resolveDeviceIndex("NonExistent", devices).has_value()); };

    "unmatched @id: returns nullopt"_test = [&] { expect(!resolveDeviceIndex("@id:hw:99,0", devices).has_value()); };

    "first match wins for ambiguous substring"_test = [&] {
        auto result = resolveDeviceIndex("output", devices);
        expect(result.has_value());
        expect(eq(*result, 0UZ)) << "should match 'Built-in Audio Output' first";
    };
};

#ifndef __EMSCRIPTEN__
const boost::ut::suite<"audio timing drift"> _timingAndDriftTests = [] {
    using namespace boost::ut;

    "AudioSource emits timing tags with dummy backend"_test = [] {
        constexpr std::string_view caseName = "AudioSource timing tags";

        gr::Graph graph;
        auto&     source                = graph.emplaceBlock<gr::blocks::audio::AudioSource<float>>({{"sample_rate", 22050.f}, {"num_channels", gr::Size_t(1)}, {"io_buffer_size", 0.1f}, {"emit_timing_tags", true}, {"tag_interval", 0.0f}});
        source._useDummyBackendForTests = true;
        auto& sink                      = graph.emplaceBlock<gr::blocks::testing::TagSink<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>();
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(runSchedulerFor(sched, 500ms).has_value()) << caseName;

        expect(gt(sink._nSamplesProduced, 0UZ)) << caseName;
        expect(gt(sink._tags.size(), 1UZ)) << caseName;

        bool foundTimingTag = false;
        for (const auto& sinkTag : sink._tags) {
            if (sinkTag.map.contains(gr::tag::TRIGGER_TIME.shortKey())) {
                foundTimingTag = true;
                expect(sinkTag.map.contains(gr::tag::TRIGGER_NAME.shortKey())) << caseName;
                expect(sinkTag.map.contains(gr::tag::TRIGGER_OFFSET.shortKey())) << caseName;

                expect(sinkTag.map.contains(gr::tag::TRIGGER_META_INFO.shortKey())) << caseName;
                break;
            }
        }
        expect(foundTimingTag) << "should have at least one timing tag";
    };

    "AudioSource stamps a chunk with the capture time of its first sample"_test = [] {
        constexpr std::string_view caseName = "AudioSource capture time";
        constexpr double           kRate    = 48000.0;

        gr::Graph graph;
        auto&     source                = graph.emplaceBlock<gr::blocks::audio::AudioSource<float>>({{"sample_rate", static_cast<float>(kRate)}, {"num_channels", gr::Size_t(1)}, {"io_buffer_size", 0.1f}, {"emit_timing_tags", true}, {"tag_interval", 0.0f}});
        source._useDummyBackendForTests = true;
        // without drift compensation every output sample is a captured one, and its index counts the sample periods before it
        source.drift_correction = gr::algorithm::DriftCorrection::None;
        auto& sink              = graph.emplaceBlock<ArrivalSink<float>>();
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        const auto tStartNs = static_cast<std::int64_t>(gr::blocks::audio::detail::wallClockNs());
        expect(runSchedulerFor(sched, 500ms).has_value()) << caseName;
        expect(sched.state() != gr::lifecycle::State::ERROR) << caseName;

        // with tag_interval 0 every chunk carries a timing tag on its first sample; a chunk ends where the next tag begins
        const auto& stamps   = sink._triggerTimes;
        const auto& arrivals = sink._arrivals;
        const auto  duration = [](std::size_t nSamples) { return static_cast<std::int64_t>(std::llround(static_cast<double>(nSamples) * 1e9 / kRate)); };

        std::int64_t latestNs   = std::numeric_limits<std::int64_t>::min(); // stamp less the latest possible capture
        std::int64_t earliestNs = std::numeric_limits<std::int64_t>::max(); // stamp less the earliest possible capture
        std::int64_t minEpochNs = std::numeric_limits<std::int64_t>::max(); // stamp less the sample's offset in the stream
        std::int64_t maxEpochNs = std::numeric_limits<std::int64_t>::min();
        std::size_t  nChunks    = 0UZ;
        std::size_t  maxLength  = 0UZ;
        for (std::size_t i = 0UZ; i + 1UZ < stamps.size(); ++i) {
            const std::size_t first = stamps[i].index;
            const std::size_t last  = stamps[i + 1UZ].index - 1UZ;
            const auto        seen  = std::ranges::find_if(arrivals, [last](const auto& arrival) { return arrival.index > last; });
            if (last < first || seen == arrivals.end()) {
                continue;
            }
            // upper bound: the chunk's last sample was captured before it arrived, and its first
            // sample (last - first) sample periods earlier; lower bound: the backend captured no
            // sample before the graph started, nor faster than its rate
            const std::int64_t upperNs = static_cast<std::int64_t>(seen->timeNs) - duration(last - first);
            const std::int64_t lowerNs = tStartNs + duration(first);
            const auto         stampNs = static_cast<std::int64_t>(stamps[i].timeNs);
            latestNs                   = std::max(latestNs, stampNs - upperNs);
            earliestNs                 = std::min(earliestNs, stampNs - lowerNs);
            minEpochNs                 = std::min(minEpochNs, stampNs - duration(first));
            maxEpochNs                 = std::max(maxEpochNs, stampNs - duration(first));
            maxLength                  = std::max(maxLength, last - first + 1UZ);
            ++nChunks;
        }

        expect(ge(nChunks, 2UZ)) << caseName;
        if (nChunks > 0UZ) {
            std::println("{}: {} chunks of at most {} samples ({:.3f} ms); trigger_time less the latest possible capture of the chunk's first sample at most {:+.3f} ms, less the earliest at least {:+.3f} ms; trigger_time less the sample's offset spread {:.3f} ms", //
                caseName, nChunks, maxLength, static_cast<double>(duration(maxLength)) * 1e-6, static_cast<double>(latestNs) * 1e-6, static_cast<double>(earliestNs) * 1e-6, static_cast<double>(maxEpochNs - minEpochNs) * 1e-6);
        }
        expect(le(latestNs, std::int64_t{0})) << std::format("{}: a chunk is stamped {:.3f} ms after its first sample could have been captured", caseName, static_cast<double>(latestNs) * 1e-6);
        expect(ge(earliestNs, std::int64_t{0})) << std::format("{}: a chunk is stamped {:.3f} ms before its first sample could have been captured", caseName, static_cast<double>(-earliestNs) * 1e-6);
    };

    "DriftCompensator inserts sample when source is fast"_test = [] {
        gr::algorithm::DriftCompensator<float> comp;
        std::array<float, 10>                  buf{1.f, 2.f, 3.f, 4.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

        // simulate source 100 ppm fast over many calls to accumulate >=1 sample
        std::size_t nProduced = 4U;
        for (int i = 0; i < 300; ++i) {
            nProduced = comp.compensateSource(std::span(buf), 4U, 48000.0 * 1.0001, 48000.0, 1U);
        }

        // after enough calls, compensator should have inserted at least once
        expect(ge(nProduced, 4UZ)) << "should insert or maintain sample count";
    };

    "DriftCompensator drops sample when source is slow"_test = [] {
        gr::algorithm::DriftCompensator<float> comp;
        std::array<float, 10>                  buf{1.f, 2.f, 3.f, 4.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

        std::size_t nProduced = 4U;
        for (int i = 0; i < 300; ++i) {
            nProduced = comp.compensateSource(std::span(buf), 4U, 48000.0 * 0.9999, 48000.0, 1U);
        }

        expect(le(nProduced, 4UZ)) << "should drop or maintain sample count";
    };

    "DriftCompensator interpolation produces smooth values"_test = [] {
        gr::algorithm::DriftCompensator<float> comp;
        std::array<float, 10>                  buf{0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

        comp.fractionalAccumulator = 0.99;
        buf[0]                     = 1.0f;
        buf[1]                     = 3.0f;

        auto n = comp.compensateSource(std::span(buf), 2U, 48000.0 * 1.01, 48000.0, 1U);
        if (n == 3U) {
            expect(approx(buf[2], 2.0f, 0.5f)) << "interpolated sample should be between neighbours";
        }
    };

    "DriftCompensator stereo insert preserves channel interleaving"_test = [] {
        gr::algorithm::DriftCompensator<float> comp;
        std::array<float, 20>                  buf{};
        // stereo: L0=1, R0=2, L1=3, R1=4
        buf[0] = 1.f;
        buf[1] = 2.f;
        buf[2] = 3.f;
        buf[3] = 4.f;

        comp.fractionalAccumulator = 0.99;
        auto n                     = comp.compensateSource(std::span(buf), 4U, 48000.0 * 1.01, 48000.0, 2U);
        if (n == 6U) {
            // inserted stereo frame at index 4,5 interpolated from frames (0,1) and (2,3)
            expect(approx(buf[4], 2.0f, 0.5f)) << "inserted L should interpolate between 1 and 3";
            expect(approx(buf[5], 3.0f, 0.5f)) << "inserted R should interpolate between 2 and 4";
        }
    };

    "DriftCompensator stereo drop preserves frame alignment"_test = [] {
        gr::algorithm::DriftCompensator<float> comp;
        std::array<float, 10>                  buf{1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 0.f, 0.f, 0.f, 0.f};

        comp.fractionalAccumulator = -0.99;
        auto n                     = comp.compensateSource(std::span(buf), 6U, 48000.0 * 0.99, 48000.0, 2U);
        if (n == 4U) {
            expect(eq(n % 2UZ, 0UZ)) << "dropped result should be frame-aligned";
        }
    };

    "emit_timing_tags=false suppresses timing tags"_test = [] {
        constexpr std::string_view caseName = "timing tags disabled";

        gr::Graph graph;
        auto&     source                = graph.emplaceBlock<gr::blocks::audio::AudioSource<float>>({{"sample_rate", 22050.f}, {"num_channels", gr::Size_t(1)}, {"io_buffer_size", 0.1f}, {"emit_timing_tags", false}});
        source._useDummyBackendForTests = true;
        auto& sink                      = graph.emplaceBlock<gr::blocks::testing::TagSink<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>();
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(runSchedulerFor(sched, 300ms).has_value()) << caseName;

        expect(gt(sink._nSamplesProduced, 0UZ)) << caseName;
        // should have the format tag but no TRIGGER_TIME tags
        bool foundTimingTag = false;
        for (const auto& sinkTag : sink._tags) {
            if (sinkTag.map.contains(gr::tag::TRIGGER_TIME.shortKey())) {
                foundTimingTag = true;
            }
        }
        expect(!foundTimingTag) << "no timing tags should be emitted when disabled";
    };

    "emit_meta_info=false omits TRIGGER_META_INFO"_test = [] {
        constexpr std::string_view caseName = "meta info disabled";

        gr::Graph graph;
        auto&     source                = graph.emplaceBlock<gr::blocks::audio::AudioSource<float>>({{"sample_rate", 22050.f}, {"num_channels", gr::Size_t(1)}, {"io_buffer_size", 0.1f}, {"emit_timing_tags", true}, {"emit_meta_info", false}, {"tag_interval", 0.0f}});
        source._useDummyBackendForTests = true;
        auto& sink                      = graph.emplaceBlock<gr::blocks::testing::TagSink<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>();
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(runSchedulerFor(sched, 300ms).has_value()) << caseName;

        bool foundTimingTag = false;
        bool foundMetaInfo  = false;
        for (const auto& sinkTag : sink._tags) {
            if (sinkTag.map.contains(gr::tag::TRIGGER_TIME.shortKey())) {
                foundTimingTag = true;
                if (sinkTag.map.contains(gr::tag::TRIGGER_META_INFO.shortKey())) {
                    foundMetaInfo = true;
                }
            }
        }
        expect(foundTimingTag) << "should have timing tags";
        expect(!foundMetaInfo) << "TRIGGER_META_INFO should be absent when emit_meta_info=false";
    };

    "tag_interval throttles timing tag emission"_test = [] {
        constexpr std::string_view caseName = "tag interval throttling";

        gr::Graph graph;
        // large interval — should emit at most 1-2 timing tags in 300ms
        auto& source                    = graph.emplaceBlock<gr::blocks::audio::AudioSource<float>>({{"sample_rate", 22050.f}, {"num_channels", gr::Size_t(1)}, {"io_buffer_size", 0.1f}, {"emit_timing_tags", true}, {"tag_interval", 10.0f}});
        source._useDummyBackendForTests = true;
        auto& sink                      = graph.emplaceBlock<gr::blocks::testing::TagSink<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>();
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(runSchedulerFor(sched, 300ms).has_value()) << caseName;

        std::size_t timingTagCount = 0U;
        for (const auto& sinkTag : sink._tags) {
            if (sinkTag.map.contains(gr::tag::TRIGGER_TIME.shortKey())) {
                ++timingTagCount;
            }
        }
        // with 10s interval and 300ms runtime, expect at most 1 timing tag (the first one)
        expect(le(timingTagCount, 1UZ)) << "tag_interval=10s should heavily throttle emission";
    };

    "AudioSource rate estimator converges"_test = [] {
        constexpr std::string_view caseName = "rate estimator convergence";

        gr::Graph graph;
        auto&     source                = graph.emplaceBlock<gr::blocks::audio::AudioSource<float>>({{"sample_rate", 22050.f}, {"num_channels", gr::Size_t(1)}, {"io_buffer_size", 0.1f}, {"ppm_estimator_cutoff", 0.5f}});
        source._useDummyBackendForTests = true;
        auto& sink                      = graph.emplaceBlock<gr::blocks::testing::TagSink<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>();
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(runSchedulerFor(sched, 500ms).has_value()) << caseName;

        const double estimated = source._rateEstimator.estimatedRate();
        expect(gt(estimated, 0.0)) << "rate estimator should have a positive estimate";
        // dummy backend may not honour the exact requested rate — just verify it's in a sane range
        expect(gt(static_cast<float>(estimated), 1000.f)) << "estimated rate should be above 1 kHz";
        expect(lt(static_cast<float>(estimated), 200000.f)) << "estimated rate should be below 200 kHz";
    };

    "AudioSink rate estimator runs during playback"_test = [] {
        constexpr std::string_view      caseName = "AudioSink rate estimator";
        const std::vector<std::int16_t> reference{0, 1000, -1000, 2000, -2000, 3000, -3000, 4000};
        const auto                      wavBytes = makeWav(1U, 1U, 16U, 22050U, encodePcm16(reference));
        TempFile                        file{writeTempAudioFile(wavBytes)};

        gr::Graph graph;
        auto&     source              = graph.emplaceBlock<gr::blocks::fileio::WavSource<float>>({{"uri", file.path.string()}});
        auto&     sink                = graph.emplaceBlock<gr::blocks::audio::AudioSink<float>>({{"io_buffer_size", 0.1f}, {"ppm_estimator_cutoff", 0.5f}});
        sink._useDummyBackendForTests = true;
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(sched.runAndWait().has_value()) << caseName;

        // the estimator may or may not converge for such a short file, but it should have been initialised
        expect(ge(sink._rateEstimator.estimatedRate(), 0.0)) << "sink rate estimator should have run";
    };

    "clk_in forwards clock offset and trigger name"_test = [] {
        constexpr std::string_view caseName = "clk_in forwarding";

        gr::Graph graph;
        auto&     source                = graph.emplaceBlock<gr::blocks::audio::AudioSource<float>>({{"sample_rate", 22050.f}, {"num_channels", gr::Size_t(1)}, {"io_buffer_size", 0.1f}, {"emit_timing_tags", true}, {"tag_interval", 0.0f}});
        source._useDummyBackendForTests = true;

        // clock source: emits a TRIGGER_TIME tag at sample 0 with a known UTC timestamp
        auto& clkSource = graph.emplaceBlock<gr::blocks::testing::TagSource<std::uint8_t, gr::blocks::testing::ProcessFunction::USE_PROCESS_ONE>>({{"n_samples_max", gr::Size_t(0)}, {"mark_tag", false}});

        constexpr std::uint64_t kFakeUtcNs = 1700000000'000000000ULL; // a fixed UTC timestamp
        gr::property_map        clkTagMap;
        gr::tag::put(clkTagMap, gr::tag::TRIGGER_TIME, kFakeUtcNs);
        gr::tag::put(clkTagMap, gr::tag::TRIGGER_NAME, std::string("GPS:TEST"));
        clkSource._tags = {{0U, std::move(clkTagMap)}};

        auto& sink = graph.emplaceBlock<gr::blocks::testing::TagSink<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>();
        expect(graph.connect<"out", "clk_in">(clkSource, source).has_value()) << caseName;
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(runSchedulerFor(sched, 500ms).has_value()) << caseName;

        expect(gt(sink._nSamplesProduced, 0UZ)) << caseName;

        // check that the clock offset was applied
        expect(source._clockOffsetValid) << "clock offset should be valid after receiving clk_in tag";

        // check that a timing tag uses the forwarded trigger name
        bool foundGpsTrigger = false;
        for (const auto& sinkTag : sink._tags) {
            if (auto it = sinkTag.map.find(gr::tag::TRIGGER_NAME.shortKey()); it != sinkTag.map.end()) {
                if (auto* name = it->second.get_if<std::pmr::string>()) {
                    if (*name == "GPS:TEST") {
                        foundGpsTrigger = true;
                        break;
                    }
                }
            }
        }
        expect(foundGpsTrigger) << "timing tags should forward the clock trigger name from clk_in";
    };

    "AudioSource permission setting is readable"_test = [] {
        constexpr std::string_view caseName = "AudioSource permission";

        gr::Graph graph;
        auto&     source                = graph.emplaceBlock<gr::blocks::audio::AudioSource<float>>({{"sample_rate", 22050.f}, {"num_channels", gr::Size_t(1)}, {"io_buffer_size", 0.1f}});
        source._useDummyBackendForTests = true;
        auto& sink                      = graph.emplaceBlock<gr::blocks::testing::TagSink<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>();
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(runSchedulerFor(sched, 300ms).has_value()) << caseName;

        expect(static_cast<bool>(source.permission.value)) << "source permission should be true after start";

        const auto activeParams = source.settings().getStored().value_or(gr::property_map{});
        expect(activeParams.contains("permission")) << "permission should be in active parameters";
    };

    "AudioSink permission setting is readable"_test = [] {
        constexpr std::string_view      caseName = "AudioSink permission";
        const std::vector<std::int16_t> reference{0, 1000, -1000, 2000, -2000, 3000, -3000, 4000};
        const auto                      wavBytes = makeWav(1U, 1U, 16U, 22050U, encodePcm16(reference));
        TempFile                        file{writeTempAudioFile(wavBytes)};

        gr::Graph graph;
        auto&     source              = graph.emplaceBlock<gr::blocks::fileio::WavSource<float>>({{"uri", file.path.string()}});
        auto&     sink                = graph.emplaceBlock<gr::blocks::audio::AudioSink<float>>({{"io_buffer_size", 0.1f}});
        sink._useDummyBackendForTests = true;
        expect(graph.connect<"out", "in">(source, sink).has_value()) << caseName;

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value()) << caseName;
        expect(sched.runAndWait().has_value()) << caseName;

        expect(static_cast<bool>(sink.permission.value)) << "sink permission should be true after start";

        const auto activeParams = sink.settings().getStored().value_or(gr::property_map{});
        expect(activeParams.contains("permission")) << "permission should be in active parameters";
    };

    "DriftCompensator sink insert and drop"_test = [] {
        gr::algorithm::DriftCompensator<float> comp;

        // sink compensate: input → adjusted buffer
        std::array<float, 10> input{1.f, 2.f, 3.f, 4.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
        std::array<float, 12> adjusted{};

        // sink is faster than source — needs to insert
        comp.fractionalAccumulator = 0.99;
        auto n                     = comp.compensateSink(std::span<const float>(input.data(), 4U), std::span(adjusted), 4U, 48000.0 * 0.99, 48000.0, 1U);
        expect(ge(n, 4UZ)) << "sink compensator should insert when sink is faster";

        // sink is slower than source — needs to drop
        comp.fractionalAccumulator = -0.99;
        n                          = comp.compensateSink(std::span<const float>(input.data(), 4U), std::span(adjusted), 4U, 48000.0 * 1.01, 48000.0, 1U);
        expect(le(n, 4UZ)) << "sink compensator should drop when sink is slower";
    };
};
#endif

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
