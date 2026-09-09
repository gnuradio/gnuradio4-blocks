#ifndef GNURADIO_AUDIO_BLOCKS_HPP
#define GNURADIO_AUDIO_BLOCKS_HPP

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/algorithm/SampleRateEstimator.hpp>
#include <gnuradio-4.0/audio/AudioBackends.hpp>
#include <gnuradio-4.0/audio/EmscriptenAudioBackend.hpp>
#include <gnuradio-4.0/audio/SoundIoBackend.hpp>
#include <gnuradio-4.0/fileio/WavBlocks.hpp>
#include <gnuradio-4.0/meta/formatter.hpp>
#include <gnuradio-4.0/meta/reflection.hpp>
#include <gnuradio-4.0/thread/thread_pool.hpp>

#include <algorithm>
#include <atomic>
#include <cstring>
#include <expected>
#include <format>
#include <mutex>
#include <print>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <gnuradio-4.0/audio/NamespaceCompatibility.hpp>

namespace gr::blocks::audio {

GR_REGISTER_BLOCK(gr::blocks::audio::AudioSource, [T], [ float, int16_t ])

template<detail::AudioSample T>
struct AudioSource : gr::Block<AudioSource<T>> {
    using Description = Doc<R""(Capture interleaved PCM samples from the platform audio input device.

`AudioSource` is intentionally PCM-only. It captures microphone or default input-device samples and
publishes them downstream as interleaved float or signed 16-bit PCM.
Publishes timing tags with estimated sample rate and optional GPS/PPS clock discipline.)"">;

    gr::PortIn<std::uint8_t, gr::Optional> clk_in;
    gr::PortOut<T>                         out;

    gr::Annotated<float, "sample_rate", gr::Visible, gr::Unit<"Hz">, gr::Doc<"Requested capture sample rate. Updated to the active stream rate after start.">>                                                                                                                                                        sample_rate    = 48000.f;
    gr::Annotated<gr::Size_t, "num_channels", gr::Visible, gr::Doc<"Requested interleaved channel count. Updated to the active stream channel count after start.">>                                                                                                                                                   num_channels   = 1U;
    gr::Annotated<float, "io_buffer_size", gr::Visible, gr::Unit<"s">, gr::Limits<0.1f, 10.f>, gr::Doc<"I/O buffer size in seconds">>                                                                                                                                                                                 io_buffer_size = 5.0f;
    gr::Annotated<std::string, "device", gr::Visible, gr::Doc<"Device selector: empty or 'default' for system default, substring match on name, or '@id:...' for exact ID. A default selection prefers PulseAudio, an explicit one keeps the platform's own backend order.">>                                         device;
    gr::Annotated<std::string, "backend", gr::Visible, gr::Doc<"Audio backend: 'auto' (default) leaves the choice to the device selector, or name one of the platform's backends, e.g. 'pulseaudio', 'jack', 'alsa', 'dummy'. A backend that is not available is a start error, never a connection to another one.">> backend = std::string("auto");
    gr::Annotated<std::vector<std::string>, "available_devices", gr::Doc<"Detected audio input devices in 'name [id]' format">>                                                                                                                                                                                       available_devices;
    gr::Annotated<std::string, "active_backend", gr::Doc<"Read-only: the backend the block connected through">>                                                                                                                                                                                                       active_backend;
    gr::Annotated<bool, "emit_timing_tags", gr::Doc<"Emit timing tags with timestamps and rate estimates">>                                                                                                                                                                                                           emit_timing_tags = true;
    gr::Annotated<bool, "emit_meta_info", gr::Doc<"Include metadata in timing tags">>                                                                                                                                                                                                                                 emit_meta_info   = true;
    gr::Annotated<float, "tag_interval", gr::Unit<"s">, gr::Doc<"Minimum interval between timing tags">>                                                                                                                                                                                                              tag_interval     = 1.0f;
    gr::Annotated<std::string, "trigger_name", gr::Doc<"Trigger name for free-running (no external clock) mode">>                                                                                                                                                                                                     trigger_name     = std::string("AUDIO_WALLCLOCK");
    gr::Annotated<float, "ppm_estimator_cutoff", gr::Unit<"Hz">, gr::Doc<"Low-pass cutoff for sample rate estimator">>                                                                                                                                                                                                ppm_estimator_cutoff =
#if defined(__EMSCRIPTEN__)
        0.01f;
#else
        0.1f;
#endif
    gr::Annotated<algorithm::DriftCorrection, "drift_correction", gr::Doc<"Drift compensation mode: None, Linear, Cubic, or AdaptiveResampling">> drift_correction = algorithm::DriftCorrection::Linear;
    gr::Annotated<bool, "permission", gr::Doc<"Read-only: whether microphone/input device permission has been granted">>                          permission       = false;
    gr::Annotated<gr::Size_t, "dropped_samples", gr::Doc<"Read-only: captured samples lost, to a full ring or to a silence-filled driver hole">>  dropped_samples  = 0U;
    bool                                                                                                                                          _useDummyBackendForTests{false};

    GR_MAKE_REFLECTABLE(AudioSource, clk_in, out, sample_rate, num_channels, io_buffer_size, device, backend, available_devices, active_backend, emit_timing_tags, emit_meta_info, tag_interval, trigger_name, ppm_estimator_cutoff, drift_correction, permission, dropped_samples);

    using gr::Block<AudioSource<T>>::Block;
#if defined(__EMSCRIPTEN__)
    using BackendImpl = detail::EmscriptenAudioWorkletSourceBackend<T>;
#else
    using BackendImpl = detail::SoundIoSourceBackend<T>;
#endif

    BackendImpl                    _backendImpl{};
    bool                           _ioThreadDone{true};
    bool                           _failed{false};
    bool                           _formatTagPending{true};
    detail::AudioDeviceConfig      _activeConfig{};
    algorithm::SampleRateEstimator _rateEstimator;
    algorithm::DriftCompensator<T> _driftCompensator;
    std::uint64_t                  _lastTagTimeNs{0U};
    std::int64_t                   _clockOffsetNs{0};
    bool                           _clockOffsetValid{false};
    std::string                    _clockTriggerName;
    std::size_t                    _lastReportedOverflows{0U};
    std::size_t                    _backendDroppedSamples{0U};   // capture the device offered that the ring could not take
    std::size_t                    _backendSilenceSamples{0U};   // silence the driver put in place of capture it had lost
    std::size_t                    _backlogDiscardedSamples{0U}; // backlog dropped here because downstream stalled
    std::uint64_t                  _lastDropLogNs{0U};

    struct IoThreadGuard {
        bool& done;
        ~IoThreadGuard() { gr::atomic_ref(done).wait(false); }
    };
    IoThreadGuard _ioGuard{_ioThreadDone};

    void start() {
        if (auto result = initialiseBackend(); !result) {
            this->emitErrorMessage("AudioSource::start()", result.error());
            _backendImpl.shutdown();
            _failed = true;
            return;
        }
        gr::atomic_ref(_ioThreadDone).store_release(false);
        gr::thread_pool::Manager::defaultIoPool()->execute([this]() { ioReadLoop(); });
    }

    void stop() {
        gr::atomic_ref(_ioThreadDone).wait(false);
        // a capture callback still in flight would add to the counts after they were taken, and
        // shutdown() drops them with the ring, so capture is quiesced before the final collection
        _backendImpl.quiesceCapture();
        collectBackendLosses();
        _backendImpl.shutdown();
    }

    gr::work::Result work(std::size_t requestedWork = std::numeric_limits<std::size_t>::max()) noexcept {
        if (!gr::lifecycle::isActive(this->state())) {
            return {requestedWork, 0UZ, gr::work::Status::DONE};
        }
        if (gr::atomic_ref(_ioThreadDone).load_acquire()) {
            this->requestStop();
            return {requestedWork, 0UZ, gr::work::Status::DONE};
        }
        return {requestedWork, 1UZ, gr::work::Status::OK};
    }

    void ioReadLoop() {
        gr::thread_pool::thread::setThreadName(std::format("audio-src:{}", this->name.value));

        auto& outWriter = out.streamWriter();
        auto& clkReader = clk_in.streamReader();
        auto& clkTagRdr = clk_in.tagReader();

        std::size_t channelCount = std::max<std::size_t>(1U, static_cast<std::size_t>(num_channels.value));

        while (gr::lifecycle::isActive(this->state())) {
            this->applyChangedSettings();

            if (_failed) {
                // retry: quiesce capture, take its final counts, then shut down and re-initialise
                _backendImpl.quiesceCapture();
                collectBackendLosses();
                _backendImpl.shutdown();
                std::this_thread::sleep_for(std::chrono::seconds(2));
                if (!gr::lifecycle::isActive(this->state())) {
                    break;
                }
                if (auto result = initialiseBackend(); !result) {
                    this->emitErrorMessage("AudioSource::ioReadLoop()", result.error());
                    continue;
                }
                channelCount = std::max<std::size_t>(1U, static_cast<std::size_t>(num_channels.value));
            }

            if (auto pollResult = _backendImpl.poll(); !pollResult) {
                this->emitErrorMessage("AudioSource::ioReadLoop()", pollResult.error());
                _failed = true;
                continue;
            }
            // the capture callback loses samples exactly when this loop cannot place them, so the
            // count is taken on every pass, not only where samples reach the output
            collectBackendLosses();

            const std::size_t available = _backendImpl._state.reader.available();
            if (available == 0U) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            const std::size_t nFrameAligned = detail::wholeFrameSamples(available, channelCount);
            if (nFrameAligned == 0U) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            drainClockInput(clkReader, clkTagRdr);
            publishSamples(outWriter, nFrameAligned, channelCount);
        }
        collectBackendLosses();

        this->publishEoS();
        gr::atomic_ref(_ioThreadDone).store_release(true);
        gr::atomic_ref(_ioThreadDone).notify_all();
    }

private:
    [[nodiscard]] std::uint32_t currentSampleRate() const {
        const auto rounded = std::lround(static_cast<double>(sample_rate.value));
        return rounded > 0L ? static_cast<std::uint32_t>(rounded) : 0U;
    }

    [[nodiscard]] std::uint32_t currentChannelCount() const { return num_channels.value > 0U ? static_cast<std::uint32_t>(num_channels.value) : 0U; }

    [[nodiscard]] std::size_t ioBufferSamples() const { return static_cast<std::size_t>(std::max(0.1f, io_buffer_size.value) * sample_rate.value * static_cast<float>(std::max(1U, num_channels.value))); }

    [[nodiscard]] std::size_t backendBufferFrames() const { return std::max<std::size_t>(8192UZ, ioBufferSamples()); }

    // the backend ring holds io_buffer_size seconds (240k samples at the 5 s default) against a far
    // smaller port ring, so the take is clamped to what the port ring can accept
    void publishSamples(auto& writer, std::size_t nFrameAligned, std::size_t channelCount) {
        const std::size_t headroom  = channelCount;
        const std::size_t available = writer.available();
        const std::size_t capacity  = available > headroom ? detail::wholeFrameSamples(available - headroom, channelCount) : 0UZ;
        const std::size_t nToTake   = std::min(nFrameAligned, capacity);

        if (nToTake == 0UZ) {
            // drop only what would overflow the backend ring, keeping the rest for the next call
            const std::size_t backendCapacity = _backendImpl._state.buffer.size();
            const std::size_t backlog         = _backendImpl._state.reader.available();
            if (backlog >= backendCapacity) {
                const std::size_t excess = detail::wholeFrameSamples(backlog - backendCapacity + channelCount, channelCount);
                // a silence placeholder was counted when the driver's hole was stored, so only the
                // capture that eviction loses for the first time is counted here
                const std::size_t nUncounted = _backendImpl._state.discardOldest(std::min(excess, backlog));
                _backlogDiscardedSamples += nUncounted;
                reportDiscardedSamples(nUncounted);
            }
            return;
        }

        auto outSpan = writer.template tryReserve<gr::SpanReleasePolicy::ProcessNone>(nToTake + headroom);
        if (outSpan.empty()) {
            return;
        }

        std::size_t nProduced = _backendImpl.readToOutput(std::span<T>(outSpan.data(), nToTake), channelCount);

        const bool streamActive = _backendImpl.isStreamActive();
        if (static_cast<bool>(permission.value) != streamActive) {
            permission = streamActive;
            this->settings().updateActiveParameters();
        }

        if (nProduced > 0U) {
            const std::uint64_t tNowNs = detail::wallClockNs();
            _rateEstimator.update(static_cast<double>(tNowNs) * 1e-9, nProduced / channelCount);

            const double nomRate       = static_cast<double>(sample_rate.value);
            const double estimatedRate = _rateEstimator.estimatedRate();
            const double clampedRate   = std::clamp(estimatedRate, nomRate * 0.9, nomRate * 1.1);
            if (clampedRate > 0.0) {
                nProduced = _driftCompensator.compensateSource(std::span<T>(outSpan.data(), outSpan.size()), nProduced, clampedRate, nomRate, channelCount);
            }

            if (_formatTagPending) {
                auto tagMap = property_map{};
                gr::tag::put(tagMap, gr::tag::SAMPLE_RATE, sample_rate.value);
                gr::tag::put(tagMap, gr::tag::NUM_CHANNELS, num_channels.value);
                gr::tag::put(tagMap, gr::tag::SIGNAL_NAME, std::string("audio_capture"));
                out.publishTag(std::move(tagMap), 0UZ);
                _formatTagPending = false;
            }

            if (emit_timing_tags.value) {
                maybeEmitTimingTag(tNowNs, nProduced, channelCount);
            }
        }

        const auto overflows = _backendImpl._state.overflowCount.load(std::memory_order_relaxed);
        if (overflows > _lastReportedOverflows) {
            std::println(stderr, "[AudioSource] device overflow #{} (silence-filled holes: {} samples)", overflows, _backendSilenceSamples);
            _lastReportedOverflows = overflows;
        }

        outSpan.publish(nProduced);
        this->progress->incrementAndGet();
        this->progress->notify_all();
    }

    // the backend counts what its ring refused and what it filled with silence; the two are disjoint
    // and both are capture the block cannot deliver, so both are collected into the block's count
    void collectBackendLosses() {
        const std::size_t dropped = _backendImpl._state.droppedSamples.exchange(0U, std::memory_order_relaxed);
        const std::size_t silence = _backendImpl._state.silenceSamples.exchange(0U, std::memory_order_relaxed);
        _backendDroppedSamples += dropped;
        _backendSilenceSamples += silence;
        reportDiscardedSamples(dropped + silence);
    }

    void reportDiscardedSamples(std::size_t nDiscarded) {
        if (nDiscarded == 0UZ) {
            return;
        }
        dropped_samples = dropped_samples.value + static_cast<gr::Size_t>(nDiscarded);
        this->settings().updateActiveParameters();

        constexpr std::uint64_t kLogIntervalNs = 1'000'000'000ULL;
        const std::uint64_t     tNowNs         = detail::wallClockNs();
        if (tNowNs - _lastDropLogNs >= kLogIntervalNs) {
            _lastDropLogNs = tNowNs;
            std::println(stderr, "[AudioSource] lost {} captured samples (total {})", nDiscarded, dropped_samples.value);
        }
    }

    void maybeEmitTimingTag(std::uint64_t tNowNs, std::size_t /*nSamples*/, std::size_t /*channelCount*/) {
        const auto intervalNs = static_cast<std::uint64_t>(tag_interval.value * 1e9f);
        if (tNowNs - _lastTagTimeNs < intervalNs) {
            return;
        }
        _lastTagTimeNs = tNowNs;

        const auto tUtcNs   = static_cast<std::uint64_t>(static_cast<std::int64_t>(tNowNs) + _clockOffsetNs);
        const bool hasClock = clk_in.isConnected() && !_clockTriggerName.empty();

        auto tagMap = out.makeTagMap();
        gr::tag::put(tagMap, gr::tag::TRIGGER_NAME, hasClock ? _clockTriggerName : trigger_name.value);
        gr::tag::put(tagMap, gr::tag::TRIGGER_TIME, tUtcNs);
        gr::tag::put(tagMap, gr::tag::TRIGGER_OFFSET, 0.0f);

        if (emit_meta_info.value) {
            auto metaInfo = out.makeTagMap();
            gr::tag::put(metaInfo, "trigger_source", std::string("AudioSource"));
            gr::tag::put(metaInfo, "clock_source", hasClock ? _clockTriggerName : std::string("wallclock"));
            gr::tag::put(metaInfo, "software_latency", static_cast<float>(_backendImpl.softwareLatency()));
            if (_rateEstimator.estimatedRate() > 0.0) {
                gr::tag::put(metaInfo, "sample_rate", static_cast<float>(_rateEstimator.estimatedRate()));
                gr::tag::put(metaInfo, "ppm_error", _rateEstimator.estimatedPpm());
            }
            if (_clockOffsetValid) {
                gr::tag::put(metaInfo, "clock_offset_ns", _clockOffsetNs);
            }
            gr::tag::put(tagMap, gr::tag::TRIGGER_META_INFO, std::move(metaInfo));
        }

        out.publishTag(std::move(tagMap), 0UZ);
    }

    void drainClockInput(auto& clkReader, auto& clkTagRdr) {
        if (!clk_in.isConnected()) {
            return;
        }

        auto nAvailable = clkReader.available();
        if (nAvailable == 0) {
            return;
        }

        auto        tagData       = clkTagRdr.get(clkTagRdr.available());
        std::size_t nTagsConsumed = 0;

        for (const auto& clkTag : tagData) {
            ++nTagsConsumed;

            if (auto it = clkTag.map.find(std::pmr::string(gr::tag::TRIGGER_TIME.shortKey())); it != clkTag.map.end()) {
                if (auto* timePtr = it->second.template get_if<std::uint64_t>()) {
                    auto triggerUtcNs = static_cast<std::int64_t>(*timePtr);

                    std::int64_t localNs      = 0;
                    bool         hasLocalTime = false;

                    if (auto metaIt = clkTag.map.find(std::pmr::string(gr::tag::TRIGGER_META_INFO.shortKey())); metaIt != clkTag.map.end()) {
                        if (auto* metaMap = metaIt->second.template get_if<property_map>()) {
                            if (auto ltIt = metaMap->find(std::pmr::string("local_time")); ltIt != metaMap->end()) {
                                if (auto* ltPtr = ltIt->second.template get_if<std::uint64_t>()) {
                                    localNs      = static_cast<std::int64_t>(*ltPtr);
                                    hasLocalTime = true;
                                }
                            }
                        }
                    }

                    _clockOffsetNs    = hasLocalTime ? (triggerUtcNs - localNs) : (triggerUtcNs - static_cast<std::int64_t>(detail::wallClockNs()));
                    _clockOffsetValid = true;
                }
            }

            if (auto it = clkTag.map.find(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey())); it != clkTag.map.end()) {
                if (auto* namePtr = it->second.template get_if<std::pmr::string>()) {
                    if (!namePtr->empty()) {
                        _clockTriggerName = std::string(*namePtr);
                    }
                }
            }
        }

        std::ignore  = tagData.consume(nTagsConsumed);
        auto clkSpan = clkReader.get(nAvailable);
        std::ignore  = clkSpan.consume(nAvailable);
    }

    [[nodiscard]] std::expected<void, gr::Error> initialiseBackend() {
        const detail::AudioDeviceConfig config{.sampleRate = currentSampleRate(), .numChannels = currentChannelCount(), .bufferFrames = backendBufferFrames(), .device = device.value, .backend = backend.value, .useDummyBackendForTests = _useDummyBackendForTests};
        auto                            result = _backendImpl.start(config);
        if (!result) {
            return std::unexpected(result.error());
        }

        if (result->sampleRate != config.sampleRate && config.sampleRate != 0U) {
            this->emitErrorMessage("AudioSource::start()", gr::Error(std::format("requested sample rate {} Hz, device negotiated {} Hz", config.sampleRate, result->sampleRate)));
        }

        available_devices = _backendImpl._availableDevices;
        active_backend    = _backendImpl.activeBackendName();
        sample_rate       = static_cast<float>(result->sampleRate);
        num_channels      = static_cast<gr::Size_t>(result->numChannels);
        _activeConfig     = {.sampleRate = result->sampleRate, .numChannels = result->numChannels, .bufferFrames = backendBufferFrames(), .device = device.value, .backend = backend.value};
        _formatTagPending = true;
        _failed           = false;
        _lastTagTimeNs    = 0U;
        _clockOffsetNs    = 0;
        _clockOffsetValid = false;
        _clockTriggerName.clear();
        _driftCompensator.mode = drift_correction.value;
        _driftCompensator.reset();
        _lastReportedOverflows = 0U;

        const double expectedChunkRate  = static_cast<double>(result->sampleRate) / static_cast<double>(backendBufferFrames());
        _rateEstimator.filter_cutoff_hz = ppm_estimator_cutoff.value;
        _rateEstimator.reset(static_cast<double>(result->sampleRate), expectedChunkRate);

        return {};
    }
};

static_assert(gr::BlockLike<AudioSource<float>>);

GR_REGISTER_BLOCK(gr::blocks::audio::AudioSink, [T], [ float, int16_t ])

template<detail::AudioSample T>
struct AudioSink : gr::Block<AudioSink<T>> {
    using Description = Doc<R""(Play interleaved PCM samples on the platform audio output device.

`AudioSink` is intentionally PCM-only. It does not parse container formats or fetch files.
Pair it with `WavSource` or any other block that already produces decoded PCM.
Publishes timing tags with estimated consumption rate and software latency.)"">;

    gr::PortIn<T> in;

    gr::Annotated<float, "sample_rate", gr::Visible, gr::Unit<"Hz">, gr::Doc<"PCM sample rate. Updated automatically; not intended to be set by the user.">>                                                                                                                                                          sample_rate    = 48000.f;
    gr::Annotated<gr::Size_t, "num_channels", gr::Visible, gr::Doc<"PCM interleaved channel count. Updated automatically; not intended to be set by the user.">>                                                                                                                                                      num_channels   = 1U;
    gr::Annotated<float, "io_buffer_size", gr::Visible, gr::Unit<"s">, gr::Limits<0.1f, 10.f>, gr::Doc<"I/O staging buffer size in seconds">>                                                                                                                                                                         io_buffer_size = 5.0f;
    gr::Annotated<std::string, "device", gr::Visible, gr::Doc<"Device selector: empty or 'default' for system default, substring match on name, or '@id:...' for exact ID. A default selection prefers PulseAudio, an explicit one keeps the platform's own backend order.">>                                         device;
    gr::Annotated<std::string, "backend", gr::Visible, gr::Doc<"Audio backend: 'auto' (default) leaves the choice to the device selector, or name one of the platform's backends, e.g. 'pulseaudio', 'jack', 'alsa', 'dummy'. A backend that is not available is a start error, never a connection to another one.">> backend = std::string("auto");
    gr::Annotated<std::vector<std::string>, "available_devices", gr::Doc<"Detected audio output devices in 'name [id]' format">>                                                                                                                                                                                      available_devices;
    gr::Annotated<std::string, "active_backend", gr::Doc<"Read-only: the backend the block connected through">>                                                                                                                                                                                                       active_backend;
    gr::Annotated<float, "ppm_estimator_cutoff", gr::Unit<"Hz">, gr::Doc<"Low-pass cutoff for sample rate estimator">>                                                                                                                                                                                                ppm_estimator_cutoff =
#if defined(__EMSCRIPTEN__)
        0.01f;
#else
        0.1f;
#endif
    gr::Annotated<algorithm::DriftCorrection, "drift_correction", gr::Doc<"Drift compensation mode: None, Linear, Cubic, or AdaptiveResampling">> drift_correction = algorithm::DriftCorrection::Linear;
    gr::Annotated<bool, "debug_console", gr::Doc<"Log diagnostic info to stderr">>                                                                debug_console    = false;
    gr::Annotated<bool, "permission", gr::Doc<"Read-only: whether audio output device/context is active (not suspended)">>                        permission       = false;
    gr::Annotated<gr::Size_t, "dropped_samples", gr::Doc<"Read-only: samples dropped because the staging buffer was full">>                       dropped_samples  = 0U;
    bool                                                                                                                                          _useDummyBackendForTests{false};

    GR_MAKE_REFLECTABLE(AudioSink, in, sample_rate, num_channels, io_buffer_size, device, backend, available_devices, active_backend, ppm_estimator_cutoff, drift_correction, debug_console, permission, dropped_samples);

    using gr::Block<AudioSink<T>>::Block;
#if defined(__EMSCRIPTEN__)
    using BackendImpl = detail::EmscriptenAudioWorkletSinkBackend<T>;
#else
    using BackendImpl = detail::SoundIoSinkBackend<T>;
#endif

    using StagingBuffer = gr::CircularBuffer<T, std::dynamic_extent, gr::ProducerType::Single>;
    using StagingWriter = decltype(std::declval<StagingBuffer&>().new_writer());
    using StagingReader = decltype(std::declval<StagingBuffer&>().new_reader());

    // what the I/O thread needs from the settings, handed over with the reconfigure request: the I/O
    // thread reads no setting, so nothing it uses can change under it while the scheduler thread writes
    struct IoConfig {
        detail::AudioDeviceConfig  device{};
        std::size_t                stagingCapacity{1U};
        float                      ioBufferSeconds{5.f};
        float                      ppmEstimatorCutoff{0.1f};
        algorithm::DriftCorrection driftCorrection{algorithm::DriftCorrection::Linear};
        bool                       debugConsole{false};
    };

    // what the device negotiated, handed back for the scheduler thread to publish as settings
    struct NegotiatedFormat {
        float                    sampleRate{0.f};
        gr::Size_t               numChannels{0U};
        std::vector<std::string> availableDevices{};
        std::string              backendName{};
        bool                     pending{false};
    };

    BackendImpl                    _backendImpl{};
    bool                           _failed{false};
    detail::AudioDeviceConfig      _activeConfig{};
    std::mutex                     _deviceMutex;
    std::shared_mutex              _ringMutex;   // unique: the I/O thread replaces the staging generation; shared: the producer holds one generation across a reservation
    std::mutex                     _configMutex; // the configuration handed to the I/O thread and the format handed back
    algorithm::SampleRateEstimator _rateEstimator;
    algorithm::DriftCompensator<T> _driftCompensator;
    double                         _smoothedFillLevel{0.5};
    std::size_t                    _bufferCapacity{0U};
    std::size_t                    _lastReportedUnderruns{0U};
    StagingBuffer                  _stagingBuffer{1U};
    StagingWriter                  _stagingWriter{_stagingBuffer.new_writer()};
    StagingReader                  _stagingReader{_stagingBuffer.new_reader()};
    std::vector<T>                 _driftAdjustedBuffer{};
    bool                           _ioThreadDone{true};
    bool                           _ioStopRequested{false};
    bool                           _ioAbortRequested{false};
    bool                           _reconfigureRequested{false};
    bool                           _streamActive{false};
    IoConfig                       _pendingIoConfig{};
    NegotiatedFormat               _negotiated{};
    IoConfig                       _ioConfig{};
    std::atomic<std::size_t>       _uncollectedDropped{0U};
    std::size_t                    _ioDroppedSamples{0U};
    std::size_t                    _totalStagedSamples{0U};
    std::size_t                    _totalIoWrittenSamples{0U};
    std::uint64_t                  _lastDropLogNs{0U};

    // declared last so it destructs first: the I/O task must be joined before any state it touches
    struct IoThreadGuard {
        AudioSink& parent;
        ~IoThreadGuard() {
            gr::atomic_ref(parent._ioStopRequested).store_release(true);
            gr::atomic_ref(parent._ioAbortRequested).store_release(true);
            gr::atomic_ref(parent._ioThreadDone).wait(false);
        }
    };

    void start() {
        std::lock_guard deviceLock(_deviceMutex);
        gr::atomic_ref(_reconfigureRequested).store_release(false);
        if (auto result = initialiseBackendUnlocked(makeIoConfig()); !result) {
            failUnlocked("AudioSink::start()", result.error());
            return;
        }
        applyNegotiatedFormat();
        // start I/O thread that drains the staging buffer into the backend
        gr::atomic_ref(_ioAbortRequested).store_release(false);
        gr::atomic_ref(_ioStopRequested).store_release(false);
        gr::atomic_ref(_ioThreadDone).store_release(false);
        gr::thread_pool::Manager::defaultIoPool()->execute([this]() { ioWriteLoop(); });
    }

    // the drain the I/O thread runs after a stop request is bounded by io_buffer_size + 2 s, so this
    // must wait at least that long or shutdownDevice() frees the backend under a live thread
    void stop() {
        gr::atomic_ref(_ioStopRequested).store_release(true);
        const auto stopDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int>(io_buffer_size.value * 1000.f + 3000.f));
        while (!gr::atomic_ref(_ioThreadDone).load_acquire() && std::chrono::steady_clock::now() < stopDeadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!gr::atomic_ref(_ioThreadDone).load_acquire()) {
            gr::atomic_ref(_ioAbortRequested).store_release(true);
            gr::atomic_ref(_ioThreadDone).wait(false);
        }
        shutdownDevice();
        applyNegotiatedFormat();
        collectDroppedSamples();
    }

    // reconfiguring rebuilds the rings and the stream the I/O thread holds spans into and opens a
    // device (hundreds of ms), so only the request is recorded here; the whole configuration the I/O
    // thread will need is taken here, on the thread that owns the settings
    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        IoConfig         requested = makeIoConfig();
        std::shared_lock ringLock(_ringMutex);
        if (_activeConfig.sampleRate == 0U) {
            return;
        }
        if (requested.device.sampleRate == _activeConfig.sampleRate && requested.device.numChannels == _activeConfig.numChannels && requested.device.device == _activeConfig.device && requested.device.backend == _activeConfig.backend) {
            return;
        }
        {
            std::lock_guard configLock(_configMutex);
            _pendingIoConfig = std::move(requested);
        }
        gr::atomic_ref(_reconfigureRequested).store_release(true);
    }

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        if (inSpan.empty()) {
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        applyNegotiatedFormat();
        collectDroppedSamples();

        const bool streamActive = gr::atomic_ref(_streamActive).load_acquire();
        if (static_cast<bool>(permission.value) != streamActive) {
            permission = streamActive;
            this->settings().updateActiveParameters();
        }

        // the I/O thread replaces the whole staging generation; the shared lock holds the replacement
        // off across the reservation, so no span outlives the ring it was taken from
        std::shared_lock ringLock(_ringMutex);
        if (_failed) {
            std::ignore = inSpan.consume(0U);
            return gr::work::Status::ERROR;
        }

        const std::size_t channelCount  = std::max<std::size_t>(1U, static_cast<std::size_t>(_activeConfig.numChannels));
        const std::size_t nFrameSamples = inSpan.size() - (inSpan.size() % channelCount);
        if (nFrameSamples == 0U) {
            std::ignore = inSpan.consume(0U);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        // push to staging buffer — consume only what fits (natural backpressure to scheduler)
        const std::size_t stagingAvail = _stagingWriter.available();
        const std::size_t toPush       = detail::wholeFrameSamples(std::min(nFrameSamples, stagingAvail), channelCount);
        std::size_t       nPushed      = 0U;
        if (toPush > 0U) {
            auto span = _stagingWriter.tryReserve(toPush);
            if (!span.empty()) {
                nPushed = detail::wholeFrameSamples(span.size(), channelCount);
                std::copy_n(inSpan.begin(), static_cast<std::ptrdiff_t>(nPushed), span.begin());
                span.publish(nPushed);
            }
        }
        _totalStagedSamples += nPushed;

        std::ignore = inSpan.consume(nPushed);
        return nPushed > 0U ? gr::work::Status::OK : gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS;
    }

private:
    [[nodiscard]] std::uint32_t currentSampleRate() const {
        const auto rounded = std::lround(static_cast<double>(sample_rate.value));
        return rounded > 0L ? static_cast<std::uint32_t>(rounded) : 0U;
    }

    [[nodiscard]] std::uint32_t currentChannelCount() const { return num_channels.value > 0U ? static_cast<std::uint32_t>(num_channels.value) : 0U; }

    [[nodiscard]] std::size_t ioBufferSamples() const { return static_cast<std::size_t>(std::max(0.1f, io_buffer_size.value) * sample_rate.value * static_cast<float>(std::max(1U, num_channels.value))); }

    [[nodiscard]] std::size_t backendBufferFrames() const { return std::max<std::size_t>(8192UZ, ioBufferSamples()); }

    // only the scheduler thread reads the settings; every value the I/O thread needs is taken here
    [[nodiscard]] IoConfig makeIoConfig() const {
        return IoConfig{.device = {.sampleRate = currentSampleRate(), .numChannels = currentChannelCount(), .bufferFrames = backendBufferFrames(), .device = device.value, .backend = backend.value, .useDummyBackendForTests = _useDummyBackendForTests}, //
            .stagingCapacity    = std::max<std::size_t>(1U, ioBufferSamples()),
            .ioBufferSeconds    = io_buffer_size.value,
            .ppmEstimatorCutoff = ppm_estimator_cutoff.value,
            .driftCorrection    = drift_correction.value,
            .debugConsole       = debug_console.value};
    }

    // the negotiated format reaches the settings from the scheduler thread, never from the I/O thread
    void applyNegotiatedFormat() {
        NegotiatedFormat negotiated;
        {
            std::lock_guard configLock(_configMutex);
            if (!_negotiated.pending) {
                return;
            }
            negotiated          = std::move(_negotiated);
            _negotiated.pending = false;
        }
        sample_rate       = negotiated.sampleRate;
        num_channels      = negotiated.numChannels;
        available_devices = std::move(negotiated.availableDevices);
        active_backend    = std::move(negotiated.backendName);
        this->settings().updateActiveParameters();
    }

    void ioWriteLoop() {
        gr::thread_pool::thread::setThreadName(std::format("audio-sink:{}", this->name.value));

        std::size_t channelCount = std::max<std::size_t>(1U, static_cast<std::size_t>(_activeConfig.numChannels));
        double      nominalRate  = static_cast<double>(_activeConfig.sampleRate);

        // pre-fill: wait for staging buffer to accumulate ~50ms before feeding the backend
        const std::size_t prefillSamples  = static_cast<std::size_t>(nominalRate * 0.05) * channelCount;
        const auto        prefillDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
        while (!gr::atomic_ref(_ioStopRequested).load_acquire() && std::chrono::steady_clock::now() < prefillDeadline) {
            if (_stagingReader.available() >= prefillSamples) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        while (!gr::atomic_ref(_ioStopRequested).load_acquire()) {
            if (gr::atomic_ref(_reconfigureRequested).load_acquire()) {
                applyPendingReconfigure();
                channelCount = std::max<std::size_t>(1U, static_cast<std::size_t>(_activeConfig.numChannels));
                nominalRate  = static_cast<double>(_activeConfig.sampleRate);
            }

            if (auto pollResult = _backendImpl.poll(); !pollResult) {
                if (_ioConfig.debugConsole) {
                    std::println(stderr, "[AudioSink] poll error: {}", pollResult.error().message);
                }
                break;
            }

            // wait for backend to be ready (WASM AudioWorklet may take time to initialise)
            const bool streamActive = _backendImpl.isStreamActive();
            gr::atomic_ref(_streamActive).store_release(streamActive);
            if (!streamActive) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            const std::size_t stagingAvail = _stagingReader.available();
            const std::size_t backendSpace = _backendImpl._state.writer.available();
            if (stagingAvail == 0U || backendSpace == 0U) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // read only as much from staging as the backend can accept
            const std::size_t nToTransfer = detail::wholeFrameSamples(std::min(stagingAvail, backendSpace), channelCount);
            if (nToTransfer == 0U) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            auto readSpan = _stagingReader.get(nToTransfer);
            if (readSpan.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // fill-level servo on the backend's ring buffer
            const std::size_t backendAvail = _backendImpl._state.reader.available();
            const double      fillRatio    = _bufferCapacity > 0U ? static_cast<double>(backendAvail) / static_cast<double>(_bufferCapacity) : 0.5;
            constexpr double  kEmaAlpha    = 0.01;
            _smoothedFillLevel             = _smoothedFillLevel * (1.0 - kEmaAlpha) + fillRatio * kEmaAlpha;

            const double     fillError  = _smoothedFillLevel - 0.5;
            constexpr double kServoGain = 0.001;
            const double     servoRatio = 1.0 + fillError * kServoGain;

            // apply drift compensation
            const std::size_t nFrameAligned = detail::wholeFrameSamples(readSpan.size(), channelCount);
            if (_driftAdjustedBuffer.size() < nFrameAligned + channelCount) {
                _driftAdjustedBuffer.resize(nFrameAligned + channelCount);
            }
            const std::size_t nAdjusted = _driftCompensator.compensateSink(std::span<const T>(readSpan.begin(), nFrameAligned), std::span<T>(_driftAdjustedBuffer.data(), nFrameAligned + channelCount), nFrameAligned, nominalRate * servoRatio, nominalRate, channelCount);

            const std::size_t nBackendWritten = _backendImpl._state.writeFromInput(std::span<const T>(_driftAdjustedBuffer.data(), nAdjusted), channelCount);
            _totalIoWrittenSamples += nBackendWritten;
            // the drift compensator may resample, so what did not reach the backend is measured
            // against its output, not against the staging samples that produced it
            if (nBackendWritten < nAdjusted) {
                reportDroppedSamples(nAdjusted - nBackendWritten);
            }

            const std::uint64_t tNowNs = detail::wallClockNs();
            _rateEstimator.update(static_cast<double>(tNowNs) * 1e-9, nFrameAligned / channelCount);

            std::ignore = readSpan.consume(nFrameAligned);

            // check underrun — throttle log to once per second
            const auto underruns = _backendImpl._state.underrunCount.load(std::memory_order_relaxed);
            if (underruns > _lastReportedUnderruns && _ioConfig.debugConsole) {
                const auto  now     = std::chrono::steady_clock::now();
                static auto lastLog = now;
                if (now - lastLog >= std::chrono::seconds(1)) {
                    std::println(stderr, "[AudioSink] buffer underrun: total {}", underruns);
                    lastLog = now;
                }
            }
            _lastReportedUnderruns = underruns;
        }

        // drain staging → backend, then wait for playout (audio callback consuming the backend buffer)
        const auto drainDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int>(_ioConfig.ioBufferSeconds * 1000.f + 2000.f));

        // phase 1: transfer all remaining staging samples into the backend ring buffer
        while (_stagingReader.available() > 0U && !gr::atomic_ref(_ioAbortRequested).load_acquire() && std::chrono::steady_clock::now() < drainDeadline) {
            if (auto pollResult = _backendImpl.poll(); !pollResult) {
                break;
            }
            if (!_backendImpl.isStreamActive()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            const std::size_t remaining = _stagingReader.available();
            const std::size_t aligned   = detail::wholeFrameSamples(remaining, channelCount);
            if (aligned == 0U) {
                break;
            }

            const std::size_t backendSpace = _backendImpl._state.writer.available();
            if (backendSpace == 0U) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            const std::size_t toWrite = detail::wholeFrameSamples(std::min(aligned, backendSpace), channelCount);
            if (toWrite == 0U) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            auto readSpan = _stagingReader.get(toWrite);
            if (readSpan.empty()) {
                break;
            }
            const auto nDrained = _backendImpl._state.writeFromInput(std::span<const T>(readSpan.begin(), readSpan.size()), channelCount);
            _totalIoWrittenSamples += nDrained;
            if (nDrained < readSpan.size()) {
                reportDroppedSamples(readSpan.size() - nDrained);
            }
            std::ignore = readSpan.consume(readSpan.size());
        }
        // whatever the drain could not place is lost with the ring, so it is counted as dropped
        reportDroppedSamples(_stagingReader.available());

        // phase 2: wait for the audio callback to consume the backend buffer (playout)
        while (_backendImpl._state.reader.available() > channelCount && _backendImpl.isStreamActive() && !gr::atomic_ref(_ioAbortRequested).load_acquire() && std::chrono::steady_clock::now() < drainDeadline) {
            if (auto pollResult = _backendImpl.poll(); !pollResult) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (_ioConfig.debugConsole) {
            std::unique_lock  ringLock(_ringMutex);
            const std::size_t underruns = _backendImpl._state.underrunCount.load(std::memory_order_relaxed);
            const std::size_t overflows = _backendImpl._state.overflowCount.load(std::memory_order_relaxed);
            std::println(stderr, "[AudioSink] I/O thread done: staged={} ioWritten={} dropped={} backendAvail={} underruns={} overflows={}", _totalStagedSamples, _totalIoWrittenSamples, _ioDroppedSamples, _backendImpl._state.reader.available(), underruns, overflows);
        }
        gr::atomic_ref(_ioThreadDone).store_release(true);
        gr::atomic_ref(_ioThreadDone).notify_all();
    }

    // the I/O thread only counts; the setting and its active parameters belong to the scheduler thread
    void reportDroppedSamples(std::size_t nDropped) {
        if (nDropped == 0UZ) {
            return;
        }
        _ioDroppedSamples += nDropped;
        _uncollectedDropped.fetch_add(nDropped, std::memory_order_relaxed);
    }

    void collectDroppedSamples() {
        const std::size_t nDropped = _uncollectedDropped.exchange(0U, std::memory_order_relaxed);
        if (nDropped == 0UZ) {
            return;
        }
        dropped_samples = dropped_samples.value + static_cast<gr::Size_t>(nDropped);
        this->settings().updateActiveParameters();

        constexpr std::uint64_t kLogIntervalNs = 1'000'000'000ULL;
        const std::uint64_t     tNowNs         = detail::wallClockNs();
        if (tNowNs - _lastDropLogNs >= kLogIntervalNs) {
            _lastDropLogNs = tNowNs;
            std::println(stderr, "[AudioSink] dropped {} samples (total {}) — the device ring could not take them", nDropped, dropped_samples.value);
        }
    }

    void failUnlocked(std::string_view endpoint, gr::Error error) {
        this->emitErrorMessage(endpoint, error);
        _backendImpl.shutdown();
        gr::atomic_ref(_streamActive).store_release(false);
        std::unique_lock ringLock(_ringMutex);
        _failed = true;
    }

    // runs on the I/O thread between spans, so no span the I/O thread holds into the rings or the
    // stream being replaced is live; the producer is held off by the ring lock the replacement takes
    void applyPendingReconfigure() {
        std::lock_guard deviceLock(_deviceMutex);
        gr::atomic_ref(_reconfigureRequested).store_release(false);
        IoConfig pending;
        {
            std::lock_guard configLock(_configMutex);
            pending = _pendingIoConfig;
        }
        if (pending.device.sampleRate == _activeConfig.sampleRate && pending.device.numChannels == _activeConfig.numChannels && pending.device.device == _activeConfig.device && pending.device.backend == _activeConfig.backend) {
            return;
        }
        if (auto result = initialiseBackendUnlocked(pending); !result) {
            failUnlocked("AudioSink::applyPendingReconfigure()", result.error());
        }
    }

    [[nodiscard]] std::expected<void, gr::Error> initialiseBackendUnlocked(const IoConfig& requested) {
        auto result = _backendImpl.start(requested.device);
        if (!result) {
            return std::unexpected(result.error());
        }

        const auto& actual = *result;
        if (actual.sampleRate != requested.device.sampleRate && requested.device.sampleRate != 0U) {
            this->emitErrorMessage("AudioSink::start()", gr::Error(std::format("requested sample rate {} Hz, device negotiated {} Hz", requested.device.sampleRate, actual.sampleRate)));
        }

        _ioConfig              = requested;
        _lastReportedUnderruns = 0U;
        _smoothedFillLevel     = 0.5;
        _bufferCapacity        = detail::AudioStateBase<T>::bufferCapacitySamples(actual.numChannels, requested.device.bufferFrames);
        _driftCompensator.mode = requested.driftCorrection;
        _driftCompensator.reset();

        const double expectedChunkRate  = static_cast<double>(actual.sampleRate) / static_cast<double>(std::max<std::size_t>(1U, requested.device.bufferFrames));
        _rateEstimator.filter_cutoff_hz = requested.ppmEstimatorCutoff;
        _rateEstimator.reset(static_cast<double>(actual.sampleRate), expectedChunkRate);

        // the producer holds spans into the staging ring, so the generation is replaced under the lock
        // that excludes it, and what the old ring still held is counted as lost
        {
            std::unique_lock ringLock(_ringMutex);
            reportDroppedSamples(_stagingReader.available());
            _stagingBuffer = StagingBuffer(requested.stagingCapacity);
            _stagingWriter = _stagingBuffer.new_writer();
            _stagingReader = _stagingBuffer.new_reader();
            _activeConfig  = {.sampleRate = actual.sampleRate, .numChannels = actual.numChannels, .bufferFrames = requested.device.bufferFrames, .device = requested.device.device, .backend = requested.device.backend};
            _failed        = false;

            std::lock_guard configLock(_configMutex);
            _negotiated = {.sampleRate = static_cast<float>(actual.sampleRate), .numChannels = static_cast<gr::Size_t>(actual.numChannels), .availableDevices = _backendImpl._availableDevices, .backendName = _backendImpl.activeBackendName(), .pending = true};
        }
        gr::atomic_ref(_streamActive).store_release(_backendImpl.isStreamActive());

        return {};
    }

    void shutdownDevice() {
        std::lock_guard deviceLock(_deviceMutex);
        _backendImpl.shutdown();
        gr::atomic_ref(_streamActive).store_release(false);
    }

    IoThreadGuard _ioGuard{*this};
};

static_assert(gr::BlockLike<AudioSink<float>>);

} // namespace gr::blocks::audio

#endif // GNURADIO_AUDIO_BLOCKS_HPP
