#ifndef GNURADIO_AUDIO_BACKENDS_HPP
#define GNURADIO_AUDIO_BACKENDS_HPP

#include <gnuradio-4.0/CircularBuffer.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include <gnuradio-4.0/audio/NamespaceCompatibility.hpp>

namespace gr::blocks::audio::detail {

template<typename T>
concept AudioSample = std::same_as<T, float> || std::same_as<T, std::int16_t>;

struct AudioDeviceConfig {
    std::uint32_t sampleRate{0U};
    std::uint32_t numChannels{0U};
    std::size_t   bufferFrames{0U};
    std::string   device;
    std::string   backend; // empty or 'auto': the device selector chooses; otherwise the named backend
    bool          useDummyBackendForTests{false};
};

struct AudioStreamFormat {
    std::uint32_t sampleRate{0U};
    std::uint32_t numChannels{0U};
};

struct AudioDeviceInfo {
    std::string name;
    std::string id;
};

[[nodiscard]] inline std::uint64_t wallClockNs() { return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()); }

[[nodiscard]] inline bool caseInsensitiveContains(std::string_view haystack, std::string_view needle) {
    if (needle.empty() || needle.size() > haystack.size()) {
        return needle.empty();
    }
    return std::ranges::search(haystack, needle, [](char a, char b) { return std::tolower(static_cast<unsigned char>(a)) == std::tolower(static_cast<unsigned char>(b)); }).begin() != haystack.end();
}

[[nodiscard]] inline bool caseInsensitiveEquals(std::string_view lhs, std::string_view rhs) {
    return std::ranges::equal(lhs, rhs, [](char a, char b) { return std::tolower(static_cast<unsigned char>(a)) == std::tolower(static_cast<unsigned char>(b)); });
}

[[nodiscard]] inline std::string asciiToLower(std::string_view text) {
    std::string lowered(text);
    std::ranges::transform(lowered, lowered.begin(), [](char c) { return static_cast<char>(std::tolower(static_cast<unsigned char>(c))); });
    return lowered;
}

// an unset or 'auto' backend leaves the choice to the device selector
[[nodiscard]] inline bool isAutoBackend(std::string_view spec) { return spec.empty() || caseInsensitiveEquals(spec, "auto"); }

[[nodiscard]] inline bool isDefaultDevice(std::string_view spec) {
    if (spec.empty()) {
        return true;
    }
    return caseInsensitiveContains("default", spec) && spec.size() <= 7U;
}

// which connection the device selector asks for: an unspecified device wants the desktop's default
// routing, which PulseAudio owns, while an explicit selector names a device inside one backend, so
// that case keeps the platform's own backend order and the named device stays reachable
[[nodiscard]] inline bool prefersPulseAudioFirst(const AudioDeviceConfig& config) noexcept { return !config.useDummyBackendForTests && isDefaultDevice(config.device); }

[[nodiscard]] inline std::optional<std::size_t> resolveDeviceIndex(std::string_view deviceSpec, std::span<const AudioDeviceInfo> devices) {
    if (isDefaultDevice(deviceSpec)) {
        return std::nullopt; // caller uses system default
    }

    constexpr std::string_view kIdPrefix = "@id:";
    if (deviceSpec.starts_with(kIdPrefix)) {
        const auto targetId = deviceSpec.substr(kIdPrefix.size());
        for (std::size_t i = 0U; i < devices.size(); ++i) {
            if (devices[i].id == targetId) {
                return i;
            }
        }
        return std::nullopt;
    }

    for (std::size_t i = 0U; i < devices.size(); ++i) {
        if (caseInsensitiveContains(devices[i].name, deviceSpec)) {
            return i;
        }
    }
    return std::nullopt;
}

[[nodiscard]] inline std::vector<std::string> formatDeviceList(std::span<const AudioDeviceInfo> devices) {
    std::vector<std::string> result;
    result.reserve(devices.size());
    for (const auto& dev : devices) {
        result.push_back(std::format("{} [{}]", dev.name, dev.id));
    }
    return result;
}

template<AudioSample T>
struct AudioStateBase {
    using SampleBuffer = gr::CircularBuffer<T, std::dynamic_extent, gr::ProducerType::Single>;
    using SampleWriter = decltype(std::declval<SampleBuffer&>().new_writer());
    using SampleReader = decltype(std::declval<SampleBuffer&>().new_reader());

    std::atomic<bool>        stopRequested{false};
    std::atomic<std::size_t> overflowCount{0U};
    std::atomic<std::size_t> underrunCount{0U};
    std::atomic<std::size_t> droppedSamples{0U}; // offered by the device (or the producer) but not stored
    std::atomic<std::size_t> silenceSamples{0U}; // silence stored in place of holes the driver reported with no data behind them
    std::atomic<std::size_t> silenceInRing{0U};  // of that silence, what is still in the ring, neither delivered nor evicted
    SampleBuffer             buffer{1U};
    SampleWriter             writer{buffer.new_writer()};
    SampleReader             reader{buffer.new_reader()};

    [[nodiscard]] static std::size_t bufferCapacitySamples(std::size_t numChannels, std::size_t bufferFrames) { return std::max<std::size_t>(1U, numChannels) * std::max<std::size_t>(1U, bufferFrames); }

    void recreateBuffer(std::size_t capacitySamples) {
        buffer = SampleBuffer(std::max<std::size_t>(1U, capacitySamples));
        writer = buffer.new_writer();
        reader = buffer.new_reader();
        overflowCount.store(0U, std::memory_order_relaxed);
        underrunCount.store(0U, std::memory_order_relaxed);
        droppedSamples.store(0U, std::memory_order_relaxed);
        silenceSamples.store(0U, std::memory_order_relaxed);
        silenceInRing.store(0U, std::memory_order_relaxed);
    }
};

[[nodiscard]] inline std::size_t wholeFrameSamples(std::size_t sampleCount, std::size_t channelCount) {
    const std::size_t alignedChannelCount = std::max<std::size_t>(1U, channelCount);
    return sampleCount - (sampleCount % alignedChannelCount);
}

template<AudioSample T>
struct AudioSinkState : AudioStateBase<T> {
    using AudioStateBase<T>::reader;
    using AudioStateBase<T>::stopRequested;
    using AudioStateBase<T>::writer;

    // returns what reached the ring; the caller must not consume more than that from its own staging
    template<typename InputSpan>
    [[nodiscard]] std::size_t writeFromInput(const InputSpan& inSpan, std::size_t channelCount) {
        if (stopRequested.load(std::memory_order_acquire)) {
            return 0U;
        }

        const std::size_t alignedChannelCount = std::max<std::size_t>(1U, channelCount);
        const std::size_t nFrameSamples       = wholeFrameSamples(inSpan.size(), alignedChannelCount);
        if (nFrameSamples == 0U) {
            return 0U;
        }

        std::size_t available = writer.available();
        if (available == 0U) {
            return 0U;
        }
        available = wholeFrameSamples(available, alignedChannelCount);
        if (available == 0U) {
            return 0U;
        }

        const std::size_t nChunk     = std::min(nFrameSamples, available);
        auto              writerSpan = writer.tryReserve(nChunk);
        if (writerSpan.empty()) {
            return 0U;
        }

        const std::size_t published = wholeFrameSamples(writerSpan.size(), alignedChannelCount);
        if (published == 0U) {
            return 0U;
        }

        std::copy_n(inSpan.begin(), static_cast<std::ptrdiff_t>(published), writerSpan.begin());
        writerSpan.publish(published);
        return published;
    }

    void readPlanarFloat(float* output, std::size_t frameCount, std::size_t channelCount) {
        if (output == nullptr || frameCount == 0U) {
            return;
        }

        const auto toFloatSample = [](T value) {
            if constexpr (std::same_as<T, float>) {
                return value;
            } else {
                constexpr float kScale = 1.0f / 32768.0f;
                return static_cast<float>(value) * kScale;
            }
        };

        const std::size_t alignedChannelCount = std::max<std::size_t>(1U, channelCount);
        const std::size_t available           = reader.available();
        const std::size_t alignedAvailable    = wholeFrameSamples(available, alignedChannelCount);
        const std::size_t sampleCount         = frameCount * alignedChannelCount;
        const std::size_t nRead               = std::min(sampleCount, alignedAvailable);
        const std::size_t readFrames          = nRead / alignedChannelCount;

        if (nRead > 0U) {
            auto read = reader.get(nRead);
            for (std::size_t frame = 0U; frame < readFrames; ++frame) {
                for (std::size_t channel = 0U; channel < alignedChannelCount; ++channel) {
                    output[channel * frameCount + frame] = toFloatSample(read[frame * alignedChannelCount + channel]);
                }
            }
            std::ignore = read.consume(nRead);
        }

        if (readFrames < frameCount) {
            this->underrunCount.fetch_add(1U, std::memory_order_relaxed);
        }
        for (std::size_t channel = 0U; channel < alignedChannelCount; ++channel) {
            std::fill_n(output + static_cast<std::ptrdiff_t>(channel * frameCount + readFrames), static_cast<std::ptrdiff_t>(frameCount - readFrames), 0.0f);
        }
    }
};

template<AudioSample T>
struct AudioSourceState : AudioStateBase<T> {
    using AudioStateBase<T>::reader;
    using AudioStateBase<T>::stopRequested;
    using AudioStateBase<T>::writer;

    static constexpr std::int64_t kNoCaptureTime = std::numeric_limits<std::int64_t>::min();
    static constexpr std::size_t  kMaxSegments   = 16U;

    // a run of ring samples the device delivered with no loss between them: the ring position of
    // its first sample, and the capture time of that frame on the wallClockNs() clock, extrapolated
    // at the nominal rate from the newest frame of the run whose capture time was measured, or
    // kNoCaptureTime until one is. A loss ends the run. The frames stored after the loss open the
    // next segment, and the frames before it keep their time.
    struct CaptureSegment {
        std::atomic<std::size_t>  firstSample{0U};
        std::atomic<std::int64_t> firstFrameNs{kNoCaptureTime};
    };

    // segments [segmentsRetired, segmentsOpened) are live, in ring order, each at its index modulo
    // kMaxSegments; the writing side opens them, the reading side retires them
    std::array<CaptureSegment, kMaxSegments> segments{};
    std::atomic<std::size_t>                 segmentsOpened{1U};
    std::atomic<std::size_t>                 segmentsRetired{0U};
    std::atomic<std::size_t>                 frameSamples{1U}; // samples per ring frame, as the latest record states
    std::atomic<bool>                        lossPending{false};
    std::optional<std::size_t>               overdueSegmentStart; // writing side only: a loss whose segment found no free slot

    void recreateBuffer(std::size_t capacitySamples) {
        AudioStateBase<T>::recreateBuffer(capacitySamples);
        for (auto& segment : segments) {
            segment.firstSample.store(0U, std::memory_order_relaxed);
            segment.firstFrameNs.store(kNoCaptureTime, std::memory_order_relaxed);
        }
        segmentsOpened.store(1U, std::memory_order_relaxed);
        segmentsRetired.store(0U, std::memory_order_relaxed);
        frameSamples.store(1U, std::memory_order_relaxed);
        lossPending.store(false, std::memory_order_relaxed);
        overdueSegmentStart.reset();
    }

    // capture was lost after the ring's newest sample; safe from any thread
    void noteCaptureLoss() { lossPending.store(true, std::memory_order_release); }

    // on the writing side, before samples are published: after a loss they open a new segment
    void beginSegmentAfterLoss() {
        if (lossPending.exchange(false, std::memory_order_acquire)) {
            overdueSegmentStart = writer.position();
        }
        if (overdueSegmentStart.has_value()) {
            openSegment(*overdueSegmentStart);
        }
    }

    // the newest frame offered to the ring was captured at newestCaptureNs. A loss after that frame
    // leaves it out of the ring, and nothing is recorded then, nor while its segment waits for a slot.
    void recordCaptureTime(std::int64_t newestCaptureNs, std::size_t channelCount, double sampleRate) {
        if (!(sampleRate > 0.0) || lossPending.load(std::memory_order_acquire)) {
            return;
        }
        if (overdueSegmentStart.has_value()) {
            openSegment(*overdueSegmentStart);
            if (overdueSegmentStart.has_value()) {
                return;
            }
        }
        const std::size_t samplesPerFrame = std::max<std::size_t>(1U, channelCount);
        CaptureSegment&   segment         = segments[(segmentsOpened.load(std::memory_order_relaxed) - 1U) % kMaxSegments];
        const std::size_t nFrames         = (writer.position() - segment.firstSample.load(std::memory_order_relaxed)) / samplesPerFrame;
        if (nFrames == 0U) {
            return;
        }
        frameSamples.store(samplesPerFrame, std::memory_order_relaxed);
        segment.firstFrameNs.store(newestCaptureNs - framesToNs(nFrames - 1U, sampleRate), std::memory_order_release);
    }

    // the capture time of ring frame `frame`, counted from the ring's start, where one was measured
    // for its segment; valid for an unread frame and for the first frame of the latest read
    [[nodiscard]] std::optional<std::int64_t> captureTimeNs(std::size_t frame, double sampleRate) const {
        if (!(sampleRate > 0.0)) {
            return std::nullopt;
        }
        const std::size_t retired = segmentsRetired.load(std::memory_order_relaxed);
        for (std::size_t index = segmentsOpened.load(std::memory_order_acquire); index > retired; --index) {
            const CaptureSegment& segment      = segments[(index - 1U) % kMaxSegments];
            const std::int64_t    firstFrameNs = segment.firstFrameNs.load(std::memory_order_acquire);
            const std::size_t     firstFrame   = segment.firstSample.load(std::memory_order_relaxed) / frameSamples.load(std::memory_order_relaxed);
            if (firstFrame <= frame) {
                if (firstFrameNs == kNoCaptureTime) {
                    return std::nullopt;
                }
                return firstFrameNs + framesToNs(frame - firstFrame, sampleRate);
            }
        }
        return std::nullopt;
    }

    [[nodiscard]] static std::int64_t framesToNs(std::size_t nFrames, double sampleRate) { return static_cast<std::int64_t>(std::llround(static_cast<double>(nFrames) * 1e9 / sampleRate)); }

    [[nodiscard]] std::size_t writePlanarFloat(const float* input, std::size_t frameCount, std::size_t inputChannels, std::size_t outputChannels) {
        if (stopRequested.load(std::memory_order_acquire) || input == nullptr || frameCount == 0U) {
            return 0U;
        }

        const auto fromFloatSample = [](float value) {
            if constexpr (std::same_as<T, float>) {
                return value;
            } else {
                const float clamped = std::clamp(value, -1.0f, 1.0f);
                if (clamped <= -1.0f) {
                    return std::numeric_limits<std::int16_t>::min();
                }
                const auto scaled = std::lround(static_cast<double>(clamped) * static_cast<double>(std::numeric_limits<std::int16_t>::max()));
                return static_cast<std::int16_t>(std::clamp<long>(static_cast<long>(scaled), std::numeric_limits<std::int16_t>::min(), std::numeric_limits<std::int16_t>::max()));
            }
        };

        const std::size_t alignedOutputChannels = std::max<std::size_t>(1U, outputChannels);
        const std::size_t alignedAvailable      = wholeFrameSamples(writer.available(), alignedOutputChannels);
        const std::size_t requested             = frameCount * alignedOutputChannels;
        const std::size_t nSamplesToWrite       = std::min(requested, alignedAvailable);
        if (nSamplesToWrite == 0U) {
            if (requested > 0U) {
                this->overflowCount.fetch_add(1U, std::memory_order_relaxed);
            }
            noteCaptureLoss();
            return 0U;
        }

        auto writeSpan = writer.tryReserve(nSamplesToWrite);
        if (writeSpan.empty()) {
            noteCaptureLoss();
            return 0U;
        }

        const std::size_t published   = wholeFrameSamples(writeSpan.size(), alignedOutputChannels);
        const std::size_t chunkFrames = published / alignedOutputChannels;
        for (std::size_t frame = 0U; frame < chunkFrames; ++frame) {
            for (std::size_t channel = 0U; channel < alignedOutputChannels; ++channel) {
                const float value                                  = channel < inputChannels ? input[channel * frameCount + frame] : 0.0f;
                writeSpan[frame * alignedOutputChannels + channel] = fromFloatSample(value);
            }
        }

        beginSegmentAfterLoss();
        writeSpan.publish(published);
        if (published < requested) {
            noteCaptureLoss();
        }
        return published;
    }

    // a stored hole was counted as loss when the driver reported it, and the ring is FIFO, so what
    // leaves takes the oldest placeholders with it and is never counted a second time
    [[nodiscard]] std::size_t takeStoredSilence(std::size_t nLeaving) {
        std::size_t stored = this->silenceInRing.load(std::memory_order_relaxed);
        std::size_t taken  = std::min(nLeaving, stored);
        while (taken > 0U && !this->silenceInRing.compare_exchange_weak(stored, stored - taken, std::memory_order_relaxed)) {
            taken = std::min(nLeaving, stored);
        }
        return taken;
    }

    // evicts the oldest samples when downstream cannot take them, and returns what that lost which
    // was not counted before: the placeholders among them already were
    [[nodiscard]] std::size_t discardOldest(std::size_t nSamples) {
        retireSegments();
        auto              span       = reader.get(std::min(nSamples, reader.available()));
        const std::size_t nDiscarded = span.size();
        std::ignore                  = span.consume(nDiscarded);
        return nDiscarded - takeStoredSilence(nDiscarded);
    }

    [[nodiscard]] std::size_t readToOutput(std::span<T> output, std::size_t channelCount) {
        retireSegments();
        const std::size_t alignedOutputSize = wholeFrameSamples(output.size(), channelCount);
        const std::size_t alignedAvailable  = wholeFrameSamples(reader.available(), channelCount);
        const std::size_t nSamplesToRead    = std::min(alignedOutputSize, alignedAvailable);
        if (nSamplesToRead == 0U) {
            return 0U;
        }

        auto readSpan = reader.get(nSamplesToRead);
        if (readSpan.empty()) {
            return 0U;
        }

        std::copy_n(readSpan.begin(), static_cast<std::ptrdiff_t>(readSpan.size()), output.begin());
        const std::size_t nRead = readSpan.size();
        std::ignore             = readSpan.consume(nRead);
        std::ignore             = takeStoredSilence(nRead); // delivered placeholders leave the ring too
        return nRead;
    }

private:
    // opens a segment at ring sample firstSample, or reuses the latest one when no sample was stored
    // since it opened; with every slot live, the latest segment spans the loss and loses its time
    void openSegment(std::size_t firstSample) {
        const std::size_t opened = segmentsOpened.load(std::memory_order_relaxed);
        CaptureSegment&   latest = segments[(opened - 1U) % kMaxSegments];
        if (latest.firstSample.load(std::memory_order_relaxed) == firstSample) {
            latest.firstFrameNs.store(kNoCaptureTime, std::memory_order_relaxed);
            overdueSegmentStart.reset();
            return;
        }
        if (opened - segmentsRetired.load(std::memory_order_acquire) < kMaxSegments) {
            CaptureSegment& next = segments[opened % kMaxSegments];
            next.firstSample.store(firstSample, std::memory_order_relaxed);
            next.firstFrameNs.store(kNoCaptureTime, std::memory_order_relaxed);
            segmentsOpened.store(opened + 1U, std::memory_order_release);
            overdueSegmentStart.reset();
            return;
        }
        latest.firstFrameNs.store(kNoCaptureTime, std::memory_order_relaxed);
    }

    // retires the segments that end at or before the oldest unread sample; the segment of the frame
    // read first by the previous read stays live until this call
    void retireSegments() {
        const std::size_t oldestUnread = reader.position();
        const std::size_t opened       = segmentsOpened.load(std::memory_order_acquire);
        std::size_t       retired      = segmentsRetired.load(std::memory_order_relaxed);
        while (retired + 1U < opened && segments[(retired + 1U) % kMaxSegments].firstSample.load(std::memory_order_relaxed) <= oldestUnread) {
            ++retired;
        }
        segmentsRetired.store(retired, std::memory_order_release);
    }
};

} // namespace gr::blocks::audio::detail

#endif // GNURADIO_AUDIO_BACKENDS_HPP
