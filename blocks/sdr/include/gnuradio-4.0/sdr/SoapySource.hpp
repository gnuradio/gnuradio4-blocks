#ifndef GNURADIO_SOAPY_SOURCE_HPP
#define GNURADIO_SOAPY_SOURCE_HPP

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/thread/thread_pool.hpp>

#include <gnuradio-4.0/algorithm/SampleRateEstimator.hpp>

#include <gnuradio-4.0/sdr/DcBlocker.hpp>
#include <gnuradio-4.0/sdr/SoapyRaiiWrapper.hpp>
#include <gnuradio-4.0/sdr/ReceiveLease.hpp>

#include <array>
#include <future>
#include <limits>
#include <mutex>
#include <optional>
#include <set>

#include <chrono>
#include <thread>

namespace gr::blocks::sdr {

namespace detail {
inline bool equalWithinOnePercent(const auto& a, const auto& b) {
    return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), [](double x, double y) { return std::abs(x - y) <= 0.01 * std::max(std::abs(x), std::abs(y)); });
}
inline std::uint64_t wallClockNs() { return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count()); }
} // namespace detail

GR_REGISTER_BLOCK("gr::blocks::sdr::SoapySource", gr::blocks::sdr::SoapySimpleSource, [T], [ uint8_t, int16_t, std::complex<float> ])
GR_REGISTER_BLOCK("gr::blocks::sdr::SoapyDualSource", gr::blocks::sdr::SoapyDualSource, [T], [ uint8_t, int16_t, std::complex<float> ])

template<typename T, std::size_t nPorts = std::dynamic_extent>
struct SoapySource : Block<SoapySource<T, nPorts>> {
    using Description = Doc<R"(SoapySDR source block for SDR hardware.
Supports single and multi-channel RX via SoapySDR's device-agnostic API.
Uses a dedicated IO task to decouple USB/hardware latency from the scheduler.

finite_samples optionally terminates at an exact accepted sample count per channel.
epoch_samples optionally pauses after one bounded acquisition, retaining RX ownership
until a greater configuration_id starts the next epoch, or finish/cancel releases it.
Timestamp fencing and stream_restart are explicit policies with different evidence;
neither qualifies RF settling. source_status and per-read tags distinguish caller
requests, driver readbacks, source acceptance and release. Source completion does
not acknowledge downstream processing or archival. See docs/SOAPY_ACQUISITION.md.

Operating modes:
  clk_in connected: forwards external timing tags (GPS/PPS) with clock-offset interpolation
  clk_in disconnected: emits best-effort wall-clock timestamps on every chunk

What reaches the device: sample_rate and frequency always, because the block publishes both as stream
metadata; every other device setting only when the caller gives it a value, which leaves the device's own
gain, bandwidth, antenna, correction and AGC state unchanged. A setting the caller gives is applied
whatever that value is, so gain_mode=false switches AGC off on a device whose AGC starts on and
frequency_correction=0 resets a device's ppm. A block restored from a fully serialized settings map has
been given every setting that map holds, and a setting changed while the block runs is given by that
change.

Gain: rx_gains is one overall value per channel, which the driver distributes across its gain elements.
On a driver whose elements are attenuations that distribution is not monotonic in gain. rx_gain_elements
names the elements directly (IFGR: 20) and is applied in the order the driver lists them, after the AGC
state, because a driver may refuse a gain write while its AGC is on. Where both are given, the elements
are applied last and take precedence.

A read takes at most half the output buffer, whatever max_chunk_size is.

Tested with RTL-SDR and LimeSDR drivers.)">;

    using TSizeChecker = Limits<std::uint32_t{1}, std::numeric_limits<std::uint32_t>::max(), [](std::uint32_t x) { return std::has_single_bit(x); }>;
    using TBasePort    = PortOut<T>;
    using TPortType    = std::conditional_t<nPorts == 1U, TBasePort, std::conditional_t<nPorts == std::dynamic_extent, std::vector<TBasePort>, std::array<TBasePort, nPorts>>>;

    gr::PortIn<std::uint8_t, Optional> clk_in;
    TPortType                          out;

    Annotated<std::string, "device", Visible, Doc<"SoapySDR driver name">>                                                                 device;
    Annotated<std::string, "device_parameter", Visible, Doc<"additional driver parameters">>                                               device_parameter;
    Annotated<double, "master_clock_rate", Unit<"Hz">, Doc<"device master clock rate (0 = auto, set before sample_rate)">>                 master_clock_rate = 0.0;
    Annotated<std::string, "clock_source", Doc<"clock reference source (e.g. internal, external, gpsdo)">>                                 clock_source;
    Annotated<float, "sample_rate", Unit<"Hz">, Visible, Doc<"ADC sample rate (SigMF core:sample_rate)">>                                  sample_rate  = 1'000'000.f;
    Annotated<gr::Size_t, "num_channels", Visible, Doc<"number of RX channels (SigMF core:num_channels)">>                                 num_channels = 1U;
    Annotated<std::vector<std::string>, "rx_antennae", Visible, Doc<"per-channel RX antenna selection">>                                   rx_antennae;
    Annotated<std::vector<double>, "frequency", Unit<"Hz">, Visible, Doc<"per-channel center frequency (SigMF core:frequency)">>           frequency = initDefaultValues(107'000'000.);
    Annotated<std::vector<double>, "rx_bandwidths", Unit<"Hz">, Visible, Doc<"per-channel RX RF bandwidth (0 = the driver's own choice)">> rx_bandwidths;
    Annotated<std::vector<double>, "rx_gains", Unit<"dB">, Visible, Doc<"per-channel overall RX gain, distributed by the driver">>         rx_gains;
    Annotated<property_map, "rx_gain_elements", Visible, Doc<"RX gain in dB per named gain element, e.g. IFGR: 20">>                       rx_gain_elements{};
    Annotated<bool, "gain_mode", Doc<"enable automatic gain control (AGC)">>                                                               gain_mode            = false;
    Annotated<double, "frequency_correction", Unit<"ppm">, Doc<"crystal oscillator drift compensation">>                                   frequency_correction = 0.0;
    Annotated<bool, "dc_offset_mode", Doc<"enable hardware automatic DC offset removal">>                                                  dc_offset_mode       = false;
    Annotated<std::vector<double>, "dc_offset", Doc<"manual DC offset correction [I0,Q0,I1,Q1,...] per channel">>                          dc_offset;
    Annotated<std::vector<double>, "iq_balance", Doc<"manual IQ balance correction [I0,Q0,I1,Q1,...] per channel">>                        iq_balance;
    Annotated<std::string, "time_source", Doc<"PPS/GPS time reference (e.g. external, gpsdo)">>                                            time_source;
    Annotated<double, "reference_clock_rate", Unit<"Hz">, Doc<"reference oscillator rate (0 = auto)">>                                     reference_clock_rate = 0.0;
    Annotated<std::string, "stream_args", Doc<"SoapySDR stream kwargs (comma-separated key=value)">>                                       stream_args;
    Annotated<std::string, "tune_args", Doc<"per-channel tuning kwargs (comma-separated key=value)">>                                      tune_args;
    Annotated<std::string, "frontend_mapping", Doc<"logical-to-physical channel mapping">>                                                 frontend_mapping;
    Annotated<std::string, "device_settings", Doc<"device-level settings (comma-separated key=value)">>                                    device_settings;

    Annotated<std::uint32_t, "max_chunk_size", Doc<"max samples per read (ideally N x 512)">, Visible, TSizeChecker> max_chunk_size       = 512U << 4U;
    Annotated<std::uint32_t, "max_time_out_us", Unit<"us">, Doc<"SoapySDR polling timeout">>                         max_time_out_us      = 1'000;
    Annotated<gr::Size_t, "max_overflow_count", Doc<"max consecutive overflows before stop (0 = disable)">>          max_overflow_count   = 10U;
    Annotated<gr::Size_t, "max_fragment_count", Doc<"max consecutive fragments before stop (0 = disable)">>          max_fragment_count   = 100U;
    Annotated<bool, "verbose_overflow", Doc<"log each overflow event">>                                              verbose_overflow     = false;
    Annotated<bool, "log_device_state", Doc<"log what the device holds once, after setup">>                          log_device_state     = true;
    Annotated<std::string, "trigger_name", Doc<"tag gr:trigger_name for free-running wallclock mode">>               trigger_name         = std::string("SDR_WALLCLOCK");
    Annotated<bool, "emit_timing_tags", Doc<"emit timing tags on every chunk">>                                      emit_timing_tags     = true;
    Annotated<bool, "emit_meta_info", Doc<"include device/clock metadata in timing tags">>                           emit_meta_info       = true;
    Annotated<float, "tag_interval", Unit<"s">, Doc<"minimum interval between timing tags (0 = every chunk)">>       tag_interval         = 1.0f;
    Annotated<bool, "dc_blocker_enabled", Doc<"IIR high-pass to remove DC offset (complex<float> only)">>            dc_blocker_enabled   = false;
    Annotated<float, "dc_blocker_cutoff", Unit<"Hz">, Doc<"DC blocker high-pass cutoff frequency">>                  dc_blocker_cutoff    = 10.f;
    Annotated<float, "ppm_estimator_cutoff", Unit<"Hz">, Doc<"LP cutoff for sample-rate estimator (0 = disable)">>   ppm_estimator_cutoff = 0.f;
    Annotated<float, "ppm_tag_threshold", Doc<"emit corrected frequency/rate when ppm drift exceeds this">>          ppm_tag_threshold    = 0.1f;

    Annotated<std::vector<gr::Size_t>, "channels", Doc<"physical RX channel indices; empty selects 0..num_channels-1">> channels;
    Annotated<std::string, "frequency_component", Doc<"explicit driver component; empty uses legacy RF selection in ordinary streaming and complete-center tuning in acquisition">> frequency_component;
    Annotated<bool, "require_fresh_samples", Doc<"require post-configuration hardware clock fence and timestamped reads">> require_fresh_samples = false;
    Annotated<bool, "require_receive_ownership", Doc<"require canonical identity and a supported cooperative local RX lease">> require_receive_ownership = false;
    Annotated<std::uint64_t, "configuration_id", Doc<"caller correlation ID; wait for source_status before another configuration">> configuration_id = 0;
    Annotated<std::uint64_t, "finite_samples", Doc<"terminating sample target per channel; 0 means unlimited">> finite_samples = 0;
    Annotated<std::uint32_t, "max_stale_reads", Doc<"maximum discarded reads at a timestamp freshness fence">> max_stale_reads = 100;
    Annotated<std::uint32_t, "max_timeout_count", Doc<"consecutive timeouts in acquisition mode; unused in ordinary streaming">> max_timeout_count = 1000;
    Annotated<std::uint64_t, "epoch_samples", Doc<"opt-in bounded persistent epochs; requires selected freshness policy and nonzero increasing configuration IDs">> epoch_samples = 0;
    Annotated<std::string, "epoch_freshness", Doc<"bounded epochs: timestamp (strict device-clock fence) or stream_restart (new driver stream; no timestamp claim)">> epoch_freshness = "timestamp";
    Annotated<bool, "finish", Doc<"final normal termination from a completed paused epoch">> finish = false;
    Annotated<bool, "cancel", Doc<"cancel acquisition at the next source I/O boundary">> cancel = false;
    Annotated<property_map, "source_status", Doc<"source-owned configuration/readback/terminal snapshot; writes are ignored">> source_status;

    GR_MAKE_REFLECTABLE(SoapySource, clk_in, out, device, device_parameter, master_clock_rate, clock_source, sample_rate, num_channels, rx_antennae, frequency, rx_bandwidths, rx_gains, rx_gain_elements, gain_mode, frequency_correction, dc_offset_mode, dc_offset, iq_balance, time_source, reference_clock_rate, stream_args, tune_args, frontend_mapping, device_settings, max_chunk_size, max_time_out_us, max_overflow_count, max_fragment_count, verbose_overflow, log_device_state, trigger_name, emit_timing_tags, emit_meta_info, tag_interval, dc_blocker_enabled, dc_blocker_cutoff, ppm_estimator_cutoff, ppm_tag_threshold, channels, frequency_component, require_fresh_samples, require_receive_ownership, configuration_id, finite_samples, max_stale_reads, max_timeout_count, epoch_samples, epoch_freshness, finish, cancel, source_status);

    soapy::Device                               _device{};
    soapy::Device::Stream<T, SOAPY_SDR_RX>      _rxStream{};
    soapy::Kwargs                               _devKwargs{};
    bool                                        _ioThreadDone = true;
    std::atomic<gr::Size_t>                     _overflowCount{0U};
    std::atomic<gr::Size_t>                     _fragmentCount{0U};
    std::int64_t                                _clockOffsetNs    = 0;
    bool                                        _clockOffsetValid = false;
    std::string                                 _clockTriggerName;
    std::uint64_t                               _lastTagTimeNs  = 0UL;
    DcBlocker                                   _dcFilterI;
    DcBlocker                                   _dcFilterQ;
    algorithm::SampleRateEstimator              _rateEstimator;
    float                                       _ppmLastEmitted   = 0.0f;
    bool                                        _clockEosReceived = false;
    std::atomic<bool>                           _ioThreadStarted{false};
    std::atomic<bool>                           _ioStopRequested{false};
    std::atomic<bool>                           _dcFilterDirty{false};
    std::atomic<bool>                           _rateEstimatorDirty{false};
    std::atomic<bool>                           _activationFailed{false};
    gr::Error                                   _activationError{}; // written before _activationFailed is set
    soapy::detail::DeviceRegistry::Registration _activation;

    // Only the I/O owner mutates phase, evidence and handles once launched.
    // stop() first cancels activation and joins that owner, then uses the same
    // idempotent cleanup. The guard is last, so it runs before owned resources.
    enum class Phase { Idle, Applying, Acquiring, Quiescing, Paused, Releasing, Released };
    Phase _phase = Phase::Idle;
    ReceiveLease _receiveLease;
    property_map _status;
    // A fixed-size summary, never a queue: retain endpoints and aggregate every
    // rejected continuous discontinuity until IQ carries it or EOS reports it.
    struct PendingDiscontinuity {
        property_map firstRead, lastRead;
        std::uint64_t count = 0, overflows = 0;
        std::int32_t flags = 0;
        bool saturated = false;
        void record(const property_map& raw) {
            if (count == 0) { firstRead = raw; }
            lastRead = raw;
            auto increment = [&](std::uint64_t& n) {
                if (n == std::numeric_limits<std::uint64_t>::max()) { saturated = true; }
                else { ++n; }
            };
            increment(count);
            if (*raw.at("soapy_read_result").template get_if<std::int32_t>() == SOAPY_SDR_OVERFLOW) { increment(overflows); }
            flags |= *raw.at("soapy_read_flags").template get_if<std::int32_t>();
        }
        property_map evidence() const {
            return {{"rejected_read_count", count}, {"overflow_read_count", overflows}, {"flags_or", flags},
                {"count_saturated", saturated}, {"details_coalesced", count > 2}, {"first_read", firstRead}, {"last_read", lastRead}};
        }
    } _pendingDiscontinuity;
    std::vector<property_map> _readbacks;
    std::vector<double> _actualRates;
    std::string _identity, _sourceIdentity;
    std::uint64_t _epoch = 0, _streamGeneration = 0, _run = 0;
    std::size_t _mtu = 0;
    std::uint64_t _target = 0;
    std::uint64_t _sampleIndex = 0, _epochSamples = 0, _epochStart = 0, _readOrdinal = 0, _appliedId = 0;
    std::uint32_t _staleReads = 0, _timeouts = 0;
    std::optional<std::int64_t> _freshBoundary;
    std::optional<long double> _expectedTime;
    bool _boundedMode = false, _fresh = false, _streamActive = false, _releaseFailed = false;
    std::atomic<bool> _failed{false};
    std::atomic<bool> _cancelRequested{false}; // native cancellation observed before the I/O owner exists
    std::future<void> _ioCompletion;
    std::recursive_mutex _lifecycleMutex; // startup configuration versus external cleanup
    inline static thread_local SoapySource* _ioOwner = nullptr;

    bool restartFreshness() const { return _boundedMode && epoch_freshness.value == "stream_restart"; }
    bool acquisitionMode() const { return _boundedMode || finite_samples != 0 || require_fresh_samples; }
    std::uint64_t sampleTarget() const { return _target; }
    gr::Size_t channel(std::size_t i) const { return channels->empty() ? static_cast<gr::Size_t>(i) : channels->at(i); }

    void setPhase(Phase phase) {
        _phase = phase;
        constexpr std::array names{"idle", "applying", "acquiring", "quiescing", "paused", "releasing", "released"};
        _status.insert_or_assign("epoch_state", std::string(names[static_cast<std::size_t>(phase)]));
    }
    void syncStatus() {
        source_status = _status;
        this->updateActiveParameters({"source_status"});
    }
    void terminal(std::string_view cause, std::string_view detail = {}, bool failure = false) {
        // Retain the admission outcome independently of later cleanup failures.
        _status.try_emplace("acquisition_cause", std::string(cause));
        _status.try_emplace("acquisition_detail", std::string(detail));
        // A cleanup error may replace success/cancellation, never an earlier error.
        if (!_status.contains("terminal_cause") || (failure && !_failed.load())) {
            _status.insert_or_assign("terminal_cause", std::string(cause));
            _status.insert_or_assign("terminal_detail", std::string(detail));
        }
        if (failure) { _failed.store(true, std::memory_order_release); }
        _ioStopRequested.store(true, std::memory_order_release);
    }
    [[noreturn]] void configurationError(std::string_view where, const auto& error) {
        const auto reason = std::format("{}: {}", where, error);
        _status.insert_or_assign("configuration_state", std::string("failed"));
        terminal("configuration_failure", reason, true);
        throw gr::exception(reason);
    }
    void releaseError(std::string_view operation, const gr::Error& error) {
        if (_phase == Phase::Applying) { _status.insert_or_assign("configuration_state", std::string("failed")); }
        _releaseFailed = true; // latched for the entire object lifetime
        auto it = _status.find("release_errors");
        property_map errors = it == _status.end() ? property_map{} : *it->second.template get_if<property_map>();
        const auto previous = errors.find(operation);
        property_map history = previous == errors.end() ? property_map{} : *previous->second.template get_if<property_map>();
        history.insert_or_assign(std::pmr::string(std::to_string(history.size())), error.message);
        errors.insert_or_assign(std::pmr::string(operation), std::move(history));
        _status.insert_or_assign("release_errors", std::move(errors));
        terminal("hardware_release_failure", error.message, true);
    }
    bool deactivateRx() {
        if (!_streamActive) { return true; }
        if (auto r = _rxStream.deactivate(); !r) {
            releaseError("deactivate", r.error());
            return false; // still uncertain; final cleanup may try deactivation once more
        }
        _streamActive = false;
        return true;
    }
    bool closeRx() {
        if (auto r = _rxStream.close(); !r) {
            releaseError("close_stream", r.error());
            return false;
        }
        return true;
    }
    void cleanup() {
        if (_phase == Phase::Released) { return; }
        setPhase(Phase::Releasing);
        std::ignore = deactivateRx();
        std::ignore = closeRx();
        if (auto r = _device.close(); !r) {
            releaseError("unmake", r.error());
        } else {
            // The source's reference is released even if a TX reference remains.
            _status.insert_or_assign("device_handle_released", *r);
        }
        _status.insert_or_assign("device_reference_released", true);
        if (_releaseFailed) {
            const bool held = _receiveLease.held();
            _receiveLease.quarantine();
            _status.insert_or_assign("ownership", std::string(held ? "quarantined_until_exit_or_exec" : "release_uncertain"));
        } else if (!_receiveLease.reset()) {
            releaseError("lease_close", gr::Error("receive lease close failed; release uncertain"));
            _status.insert_or_assign("ownership", std::string("release_uncertain"));
        }
        _status.insert_or_assign("ownership_released", !_releaseFailed);
        _status.insert_or_assign("source_terminated", true);
        _status.insert_or_assign("target_reached", sampleTarget() != 0 && _epochSamples == sampleTarget());
        setPhase(Phase::Released);
        const std::size_t terminalOutputs = [&] { if constexpr (nPorts == 1) { return 1UZ; } else { return out.size(); } }();
        syncStatus();
        for (std::size_t i = 0; i < terminalOutputs; ++i) {
            auto& port = output(i);
            property_map map{{"source_terminal", _status}};
            tag::put(map, tag::END_OF_STREAM, true);
            const auto lost = port.nEoSTagsLost;
            port.publishEoSTag(std::move(map), 0UZ);
            if (port.nEoSTagsLost != lost) { terminal("provenance_failure", "terminal tag publication failed", true); }
        }
        syncStatus();
    }

    property_map epochReceipt(bool completed) const {
        property_map receipt = _status;
        receipt.erase("epoch_completion");
        receipt.erase("epoch_failure");
        receipt.insert_or_assign("accepted_samples", _epochSamples);
        receipt.insert_or_assign("expected_samples", sampleTarget());
        receipt.insert_or_assign("global_sample_start", _epochStart);
        receipt.insert_or_assign("global_sample_stop", _sampleIndex);
        receipt.insert_or_assign("target_reached", sampleTarget() != 0 && _epochSamples == sampleTarget());
        receipt.insert_or_assign("completed", completed);
        receipt.insert_or_assign("quiescent", _phase == Phase::Paused);
        receipt.insert_or_assign("paused", _phase == Phase::Paused);
        receipt.insert_or_assign("source_terminated", false);
        receipt.insert_or_assign("ownership_released", false);
        return receipt;
    }
    void pauseEpoch() {
        setPhase(Phase::Quiescing);
        if (!deactivateRx()) { return; }
        setPhase(Phase::Paused);
        _status.insert_or_assign("epoch_completion", epochReceipt(true));
    }
    void validateConfiguration() {
        if (epoch_freshness.value != "timestamp" && epoch_freshness.value != "stream_restart") {
            configurationError("freshness", "unknown policy");
        }
        if (epoch_freshness.value == "stream_restart" && !_boundedMode) { configurationError("freshness", "stream_restart requires epoch_samples"); }
        if (_boundedMode && (finite_samples != 0 || !require_fresh_samples || configuration_id == 0 || epoch_samples == 0)) {
            configurationError("epochs", "requires epoch_samples>0, finite_samples=0, configuration_id>0 and require_fresh_samples=true");
        }
        if ((_boundedMode ? epoch_samples.value : finite_samples.value) > std::numeric_limits<std::uint64_t>::max() - _sampleIndex) { configurationError("samples", "sample ordinal overflow"); }
        // Continuous polling retains its previous timeout range and unlimited retries.
        if (acquisitionMode() && (max_time_out_us == 0 || max_timeout_count == 0 || max_stale_reads == 0)) {
            configurationError("limits", "acquisition requires positive timeout and stale-read budgets");
        }
        if (static_cast<std::uint64_t>(max_time_out_us.value) > static_cast<std::uint64_t>(std::numeric_limits<long>::max())) {
            configurationError("timeout", "polling timeout exceeds driver API range");
        }
        if (!std::isfinite(sample_rate.value) || sample_rate <= 0 || frequency->empty() ||
            std::ranges::any_of(frequency.value, [](double f) { return !std::isfinite(f); })) { configurationError("RF", "finite frequency and positive sample rate required"); }
        if constexpr (nPorts != std::dynamic_extent) {
            num_channels = static_cast<gr::Size_t>(nPorts);
            this->updateActiveParameters({"num_channels"});
        }
        if (num_channels == 0 || (!channels->empty() && channels->size() != num_channels)) { configurationError("channels", "channel count mismatch"); }
        if constexpr (nPorts == std::dynamic_extent) {
            if (out.size() != num_channels) { configurationError("channels", "output port count must match num_channels before start"); }
        }
        auto selected = channels.value;
        std::ranges::sort(selected);
        if (std::adjacent_find(selected.begin(), selected.end()) != selected.end()) { configurationError("channels", "duplicate RX channel"); }
    }

    template<typename U>
    static property_map indexed(const std::vector<U>& values) {
        property_map result;
        for (std::size_t i = 0; i < values.size(); ++i) { result.insert_or_assign(std::pmr::string(std::to_string(i)), values[i]); }
        return result;
    }

    void recordRequest() {
        _status.insert_or_assign("configuration_id", configuration_id.value);
        _status.insert_or_assign("configuration_state", std::string("applying"));
        _status.insert_or_assign("requested", property_map{{"frequency", indexed(frequency.value)}, {"sample_rate", sample_rate.value},
            {"rx_bandwidths", indexed(rx_bandwidths.value)}, {"rx_gains", indexed(rx_gains.value)},
            {"clock_source", clock_source.value}, {"time_source", time_source.value}, {"master_clock_rate", master_clock_rate.value},
            {"reference_clock_rate", reference_clock_rate.value}, {"channels", indexed(channels.value)}, {"num_channels", num_channels.value},
            {"frequency_component", frequency_component.value}, {"tune_args", tune_args.value}, {"rx_antennae", indexed(rx_antennae.value)},
            {"rx_gain_elements", rx_gain_elements.value}, {"gain_mode", gain_mode.value}, {"frequency_correction", frequency_correction.value},
            {"dc_offset_mode", dc_offset_mode.value}, {"dc_offset", indexed(dc_offset.value)}, {"iq_balance", indexed(iq_balance.value)},
            {"device_settings", device_settings.value}, {"frontend_mapping", frontend_mapping.value},
            {"finite_samples", finite_samples.value}, {"epoch_samples", epoch_samples.value},
            {"require_fresh_samples", require_fresh_samples.value}, {"epoch_freshness", epoch_freshness.value}});
    }

    void captureConfiguration() {
        _readbacks.clear();
        _actualRates.clear();
        property_map hardware;
        for (const auto& [key, value] : _device.hardwareInfo()) { hardware.insert_or_assign(std::pmr::string(key), value); }
        _status.insert_or_assign("device_info", hardware);
        _status.insert_or_assign("source_identity", _sourceIdentity);
        _status.insert_or_assign("hardware_identity", _identity);
        _status.insert_or_assign("source_instance", std::string(this->unique_name));
        _status.insert_or_assign("configuration_id", configuration_id.value);

        const auto& requested = *_status.at("requested").template get_if<property_map>();
        // "applied" records setter completion, not measured hardware values.
        property_map applied{{"frequency", indexed(frequency.value)}, {"sample_rate", sample_rate.value}};
        for (std::string_view key : {"rx_bandwidths", "rx_gains"}) {
            if ((key == "rx_bandwidths" && !rx_bandwidths->empty()) || (key == "rx_gains" && !rx_gains->empty())) {
                applied.insert_or_assign(std::pmr::string(key), requested.at(std::pmr::string(key)));
            }
        }
        if (!clock_source->empty()) { applied.insert_or_assign("clock_source", clock_source.value); }
        if (!time_source->empty()) { applied.insert_or_assign("time_source", time_source.value); }
        if (master_clock_rate > 0) { applied.insert_or_assign("master_clock_rate", master_clock_rate.value); }
        if (reference_clock_rate > 0) { applied.insert_or_assign("reference_clock_rate", reference_clock_rate.value); }

        property_map readbacks;
        auto get = [](property_map& map, std::string_view key, auto&& read, bool positive = false) {
            const auto result = soapy::Device::readback(read);
            bool valid = result.has_value();
            if constexpr (std::is_arithmetic_v<typename decltype(result)::value_type>) {
                valid = valid && std::isfinite(*result) && (!positive || *result > 0);
            } else {
                valid = valid && !result->empty();
            }
            map.insert_or_assign(std::pmr::string(std::string(key) + "_available"), valid);
            if (valid) { map.insert_or_assign(std::pmr::string(key), *result); }
        };
        get(readbacks, "clock_source", [&] { return _device.getClockSource(); });
        get(readbacks, "time_source", [&] { return _device.getTimeSource(); });
        get(readbacks, "master_clock_rate_hz", [&] { return _device.getMasterClockRate(); }, true);
        get(readbacks, "reference_clock_rate_hz", [&] { return _device.getReferenceClockRate(); }, true);
        for (std::size_t i = 0; i < num_channels; ++i) {
            property_map actual{{"channel", channel(i)}};
            get(actual, "actual_center_frequency_hz", [&] { return _device.getCenterFrequency(SOAPY_SDR_RX, channel(i)); });
            get(actual, "actual_sample_rate_hz", [&] { return _device.getSampleRate(SOAPY_SDR_RX, channel(i)); }, true);
            get(actual, "actual_bandwidth_hz", [&] { return _device.getBandwidth(SOAPY_SDR_RX, channel(i)); }, true);
            if (!_device.listAvailableGainElements(SOAPY_SDR_RX, channel(i)).empty()) {
                get(actual, "actual_gain_db", [&] { return _device.getGain(SOAPY_SDR_RX, channel(i)); });
            } else { actual.insert_or_assign("actual_gain_db_available", false); }
            _actualRates.push_back(actual.contains("actual_sample_rate_hz") ? *actual.at("actual_sample_rate_hz").template get_if<double>() : 0.0);
            if (actual.contains("actual_center_frequency_hz")) {
                actual.insert_or_assign("frequency_error_hz", *actual.at("actual_center_frequency_hz").template get_if<double>() - frequency->at(std::min(i, frequency->size() - 1)));
            }
            readbacks.insert_or_assign(std::pmr::string(std::to_string(i)), actual);
            _readbacks.push_back(std::move(actual));
        }
        _status.insert_or_assign("readback", std::move(readbacks));
        _status.insert_or_assign("readback_configuration_id", configuration_id.value);
        _freshBoundary.reset();
        _fresh = false;
        if (require_fresh_samples) {
            std::optional<std::int64_t> boundary;
            if (!restartFreshness()) {
                auto observed = _device.hardwareTime();
                if (!observed) { configurationError("freshness", observed.error()); }
                boundary = *observed;
            }
            if (std::ranges::any_of(_actualRates, [](double rate) { return rate <= 0.0; }) ||
                std::ranges::any_of(_readbacks, [](const auto& rb) { return !rb.contains("actual_center_frequency_hz"); })) {
                configurationError("freshness", "frequency/rate readback unavailable");
            }
            // A Soapy multi-channel read has one timestamp and one sample count.
            // Its freshness/continuity fence cannot describe independently paced
            // channels. Fail closed rather than attest channel 0's clock for all.
            if (std::ranges::any_of(_actualRates, [&](double rate) { return rate != _actualRates.front(); })) {
                configurationError("freshness", "a timestamped multi-channel stream requires equal actual sample rates");
            }
            _freshBoundary = boundary;
            if (boundary) _status.insert_or_assign("freshness_boundary_ns", *boundary);
            else _status.erase("freshness_boundary_ns");
        } else { _status.erase("freshness_boundary_ns"); }
        _status.insert_or_assign("configuration_state", std::string("applied"));
        _status.insert_or_assign("applied", std::move(applied));
        _status.insert_or_assign("applied_configuration_id", configuration_id.value);
        _appliedId = configuration_id;
        ++_epoch;
        _epochStart = _sampleIndex;
        setPhase(Phase::Acquiring);
        if (_boundedMode) {
            _fragmentCount.store(0U, std::memory_order_relaxed);
            _dcFilterDirty.store(true, std::memory_order_release);
            _rateEstimatorDirty.store(true, std::memory_order_release);
            _lastTagTimeNs = 0;
        }
        _epochSamples = 0;
        _status.insert_or_assign("epoch_samples", std::uint64_t{0});
        _staleReads = 0;
        _status.insert_or_assign("discarded_stale_reads", std::uint64_t{0});
        _timeouts = 0;
        _expectedTime.reset();
        _status.insert_or_assign("configuration_revision", _epoch);
        _status.insert_or_assign("sdr_epoch", _epoch);
        _status.insert_or_assign("run_id", _run);
        _status.insert_or_assign("freshness", std::string(require_fresh_samples ? "pending" : "unverified"));
        _status.insert_or_assign("epoch_freshness", epoch_freshness.value);
        _status.insert_or_assign("analog_settling", std::string("unqualified"));
        _status.insert_or_assign("driver_control_deadlines", std::string("not_provided_by_SoapySDR"));
        _status.insert_or_assign("ownership_scope", std::string("cooperative_same_host_and_user_after_device_open"));
        _status.insert_or_assign("shared_tx_configuration", std::string("requires_driver_qualification"));
    }

    std::optional<Message> defaultsProperty(std::string_view property, Message message) {
        if (lifecycle::isActive(this->state())) {
            message.data = std::unexpected(gr::Error("SoapySource defaults may only be stored/reset while stopped"));
            return message;
        }
        return property == block::property::kResetDefaults ? this->propertyCallbackResetDefaults(property, std::move(message)) : this->propertyCallbackStoreDefaults(property, std::move(message));
    }
    void settingsChanged(const property_map& /*oldSettings*/, property_map& changed, property_map& forwarded) {
        forwarded.clear(); // publish valid driver readbacks at the first admitted sample instead
        if (changed.contains("source_status")) { source_status = _status; }
        if (!_device.get()) { return; }
        if (cancel) { terminal("cancelled"); return; }
        try {
            static const std::set<std::string_view> rfKeys{"frequency", "sample_rate", "rx_antennae", "rx_bandwidths", "rx_gains", "rx_gain_elements", "gain_mode", "frequency_correction", "dc_offset_mode", "dc_offset", "iq_balance", "device_settings", "clock_source", "time_source", "master_clock_rate", "reference_clock_rate", "configuration_id", "require_fresh_samples", "frequency_component", "tune_args", "finite_samples", "epoch_samples"};
            const bool configure = std::ranges::any_of(changed, [&](const auto& kv) { return rfKeys.contains(kv.first); });
            // Record the attempted request even when admission rejects it. Do not
            // mutate the prior accepted boundary or immutable completion receipt.
            if (configure) { recordRequest(); }
            for (std::string_view key : {"device", "device_parameter", "channels", "num_channels", "stream_args", "frontend_mapping", "require_receive_ownership", "epoch_freshness", "finite_samples"}) {
                if (changed.contains(key)) { configurationError("settings", "topology/ownership/freshness policy changes require stop and restart"); }
            }
            if ((epoch_samples != 0) != _boundedMode) { configurationError("epochs", "mode changes require stop and restart"); }
            if (finish) {
                if (!_boundedMode || _phase != Phase::Paused || configure) { configurationError("finish", "requires a paused epoch without configuration changes"); }
                terminal("normal_completion");
                return;
            }
            if (configure && sampleTarget() != 0 && (!_boundedMode || _phase != Phase::Paused || !changed.contains("configuration_id") || configuration_id <= _appliedId)) {
                configurationError("configuration", "wait for epoch completion, then supply a strictly increasing configuration_id");
            }
            validateConfiguration();
            if (configure) {
                _target = _boundedMode ? epoch_samples.value : finite_samples.value;
                setPhase(Phase::Applying);
                _epochStart = _sampleIndex;
                _epochSamples = 0;
                _status.insert_or_assign("epoch_samples", std::uint64_t{0});
                _status.erase("last_read");
                _status.erase("freshness_boundary_ns");
                _status.insert_or_assign("freshness", std::string("pending"));
                if (!deactivateRx()) { return; }
                if (restartFreshness() && !closeRx()) { return; }
                const bool clockChanged = changed.contains("clock_source") || changed.contains("time_source") || changed.contains("master_clock_rate") || changed.contains("reference_clock_rate");
                if (clockChanged) { applyClockConfig(); }
                if (changed.contains("device_settings")) { applyDeviceSettings(); }
                const bool rateChanged = clockChanged || changed.contains("sample_rate");
                if (rateChanged) { applySampleRate(); }
                if (rateChanged || changed.contains("rx_bandwidths")) { applyBandwidth(); }
                if (rateChanged || changed.contains("frequency") || changed.contains("frequency_component") || changed.contains("tune_args") || _boundedMode) { applyFrequency(); }
                if (changed.contains("rx_antennae")) { applyAntenna(); }
                if (changed.contains("gain_mode")) { applyGainMode(); }
                if (changed.contains("rx_gains")) { applyGain(); }
                if (changed.contains("rx_gain_elements")) { applyGainElements(); }
                if (changed.contains("frequency_correction")) { applyFrequencyCorrection(); }
                if (changed.contains("dc_offset_mode")) { applyDcOffsetMode(); }
                if (changed.contains("dc_offset")) { applyDcOffset(); }
                if (changed.contains("iq_balance")) { applyIqBalance(); }
                if (restartFreshness()) { setupRxStream(); }
                captureConfiguration();
                _dcFilterDirty.store(true, std::memory_order_release);
                _rateEstimatorDirty.store(true, std::memory_order_release);
                if (cancellationPending()) { terminal("cancelled"); return; }
                if (!ioActive()) { terminal("explicit_stop"); return; }
                // A failed activation may have side effects. Attempt deactivation
                // during cleanup even when the driver reports activation failure.
                _streamActive = true;
                if (auto r = _rxStream.activate(); !r) { configurationError("activate", r.error()); }
                _mtu = _rxStream.mtu();
                _status.insert_or_assign("stream_mtu", static_cast<std::uint64_t>(_mtu));
            }
            if (changed.contains("dc_blocker_cutoff") || changed.contains("dc_blocker_enabled")) { _dcFilterDirty.store(true, std::memory_order_release); }
            if (changed.contains("ppm_estimator_cutoff") || changed.contains("max_chunk_size")) { _rateEstimatorDirty.store(true, std::memory_order_release); }
        } catch (const std::exception& e) { terminal("configuration_failure", e.what(), true); }
        catch (...) { terminal("configuration_failure", "unknown configuration exception", true); }
        source_status = _status;
    }

    struct IoThreadGuard {
        SoapySource* self;
        ~IoThreadGuard() {
            // Core's base destructor must not dispatch derived stop() after our
            // members have died. This transition runs the hook while they live.
            if (lifecycle::isActive(self->state())) { self->requestStop(); }
            self->stop(); // also join/cancel in states whose transition runs no hook
        }
    };
    IoThreadGuard _ioGuard{this};

    void start() {
        const auto lock = std::lock_guard(_lifecycleMutex);
        if (_releaseFailed) { throw gr::exception("previous hardware release is uncertain; source cannot restart"); }
        if (_ioThreadStarted.load(std::memory_order_acquire) && !gr::atomic_ref(_ioThreadDone).load_acquire()) {
            throw gr::exception("source must stop its previous I/O task before starting");
        }
        if (_ioCompletion.valid()) { _ioCompletion.wait(); }
        if (_device.get()) { throw gr::exception("source must release its previous run before starting"); }
        this->propertyCallbacks.insert_or_assign(block::property::kResetDefaults, static_cast<BlockBase::PropertyCallback>(&SoapySource::defaultsProperty));
        this->propertyCallbacks.insert_or_assign(block::property::kStoreDefaults, static_cast<BlockBase::PropertyCallback>(&SoapySource::defaultsProperty));
        _status.clear();
        _pendingDiscontinuity = {};
        _failed.store(false, std::memory_order_relaxed);
        _cancelRequested.store(false, std::memory_order_relaxed);
        _ioThreadStarted.store(false, std::memory_order_relaxed);
        _ioStopRequested.store(false, std::memory_order_relaxed);
        _activationFailed.store(false, std::memory_order_relaxed);
        _sampleIndex = _epochSamples = _epochStart = _readOrdinal = _appliedId = 0;
        _boundedMode = false;
        _streamActive = false;
        _target = 0;
        _status = {{"schema_revision", std::uint64_t{1}}, {"run_id", ++_run}, {"source_instance", std::string(this->unique_name)},
            {"published_samples", std::uint64_t{0}}, {"epoch_samples", std::uint64_t{0}}, {"source_terminated", false}, {"ownership_released", false}};
        setPhase(Phase::Applying);
        try {
            // Core only applies constructor staging in init(). A restart does
            // not re-run init(); consume stopped staging with no device/I/O owner
            // before interpreting intent, policy, topology or acquisition limits.
            const auto applied = this->settings().applyStagedParameters();
            if (!applied.failedParameters.empty()) { configurationError("settings", applied.refusal); }
            if (!applied.appliedParameters.empty()) {
                this->notifyListeners(block::property::kStagedSetting, applied.appliedParameters);
                this->notifyListeners(block::property::kSetting, this->settings().get());
            }
            _boundedMode = epoch_samples != 0;
            _target = _boundedMode ? epoch_samples.value : finite_samples.value;
            recordRequest();
            if (finish || cancel) { configurationError("start", "clear finish/cancel before starting"); }
            validateConfiguration();
            _overflowCount.store(0U, std::memory_order_relaxed);
            _fragmentCount.store(0U, std::memory_order_relaxed);
            _clockOffsetNs = 0;
            _clockOffsetValid = false;
            _clockTriggerName.clear();
            _lastTagTimeNs = 0;
            _ppmLastEmitted = 0;
            _clockEosReceived = false;
            _dcFilterDirty.store(false, std::memory_order_relaxed);
            _rateEstimatorDirty.store(false, std::memory_order_relaxed);
            rebuildDcFilter();
            reinitDevice();
            captureConfiguration();
            rebuildRateEstimator();
            syncStatus();
            _activation.activate([this] {
                try {
                    if (cancellationPending()) { terminal("cancelled"); return; }
                    if (!ioActive()) { terminal("explicit_stop"); return; }
                    _streamActive = true;
                    if (auto r = _rxStream.activate(); !r) {
                        _activationError = r.error();
                        _activationFailed.store(true, std::memory_order_release);
                        return;
                    }
                    _mtu = _rxStream.mtu();
                    _status.insert_or_assign("stream_mtu", static_cast<std::uint64_t>(_mtu));
                    _rateEstimatorDirty.store(true, std::memory_order_release);
                    auto task = std::make_shared<std::packaged_task<void()>>([this] { ioReadLoop(); });
                    _ioCompletion = task->get_future();
                    gr::atomic_ref(_ioThreadDone).store_release(false);
                    _ioThreadStarted.store(true, std::memory_order_release);
                    thread_pool::Manager::defaultIoPool()->execute([task] { (*task)(); });
                } catch (const std::exception& e) {
                    _ioThreadStarted.store(false, std::memory_order_release);
                    gr::atomic_ref(_ioThreadDone).store_release(true);
                    gr::atomic_ref(_ioThreadDone).notify_all();
                    _activationError = gr::Error(e.what());
                    _activationFailed.store(true, std::memory_order_release);
                }
            });
            if (_activationFailed.load(std::memory_order_acquire)) { configurationError("activate", _activationError); }
        } catch (...) {
            _status.insert_or_assign("configuration_state", std::string("failed"));
            terminal("configuration_failure", "source start failed", true);
            stop();
            throw;
        }
    }
    void stop() {
        _ioStopRequested.store(true, std::memory_order_release);
        if (_ioOwner == this) { return; } // the owner exits through its own cleanup, never joins itself
        const auto lock = std::lock_guard(_lifecycleMutex);
        _activation.reset(); // also synchronizes any deferred activation callback
        if (_ioCompletion.valid()) { _ioCompletion.wait(); } // includes the task's final notifications
        if (_phase == Phase::Idle || _phase == Phase::Released) { return; }
        // Only after the gate and task have settled may this thread own status
        // and handles. A pending callback may have started an I/O task meanwhile.
        if (_activationFailed.load(std::memory_order_acquire)) {
            _status.insert_or_assign("configuration_state", std::string("failed"));
            terminal("configuration_failure", _activationError.message, true);
        }
        if (!_status.contains("terminal_cause")) { terminal(cancellationPending() ? "cancelled" : "explicit_stop"); }
        cleanup();
        syncStatus();
    }
    [[noreturn]] void failStart(std::string_view reason, std::source_location location = std::source_location::current()) {
        _status.insert_or_assign("configuration_state", std::string("failed"));
        terminal("configuration_failure", reason, true);
        throw gr::exception(reason, location); // start() owns cleanup, including partial setup
    }
    work::Result work(std::size_t requestedWork = std::numeric_limits<std::size_t>::max()) noexcept {
        if (lifecycle::isActive(this->state()) && !_ioThreadStarted.load(std::memory_order_acquire) && stagedCancellationPending()) {
            // Do not apply settings or mutate status alongside deferred driver
            // activation. Latch intent, then let stop() cancel/wait through the
            // registration gate and join any task before transferring ownership.
            _cancelRequested.store(true, std::memory_order_release);
            this->requestStop();
        } else if (_activationFailed.load(std::memory_order_acquire)) {
            this->requestStop();
        } else if (lifecycle::isActive(this->state()) &&
            ((this->disconnect_on_done && this->hasNoDownStreamConnectedChildren()) ||
             (_ioThreadStarted.load(std::memory_order_acquire) && gr::atomic_ref(_ioThreadDone).load_acquire()) ||
             (!_ioThreadStarted.load(std::memory_order_acquire) && _ioStopRequested.load(std::memory_order_acquire)))) {
            this->requestStop();
        }
        return {requestedWork, 0UZ, !lifecycle::isActive(this->state()) ? (_failed.load(std::memory_order_acquire) ? work::Status::ERROR : work::Status::DONE) : work::Status::OK};
    }
    bool stagedCancellationPending() {
        const auto pending = this->settings().stagedParameters();
        const auto it = pending.find("cancel");
        return it != pending.end() && it->second.value_or(false);
    }
    bool cancellationPending() { return _cancelRequested.load(std::memory_order_acquire) || cancel || stagedCancellationPending(); }
    [[nodiscard]] bool ioActive() const noexcept { return lifecycle::isActive(this->state()) && !_ioStopRequested.load(std::memory_order_acquire); }

    [[nodiscard]] static std::size_t readSize(std::size_t maxChunkSize, std::size_t outputCapacity, std::size_t room, std::size_t mtu) noexcept {
        const std::size_t count = std::min({maxChunkSize, std::max(outputCapacity / 2UZ, 1UZ), room});
        return mtu > 0 && count >= mtu ? count - count % mtu : count;
    }
    auto& output(std::size_t i) {
        if constexpr (nPorts == 1) { std::ignore = i; return out; }
        else { return out[i]; }
    }
    std::size_t outputCapacity() {
        std::size_t capacity = std::numeric_limits<std::size_t>::max();
        for (std::size_t i = 0; i < num_channels; ++i) { capacity = std::min(capacity, output(i).streamWriter().buffer().size()); }
        return capacity;
    }

    // Driver results always enter status, including rejected reads. Only accepted
    // data gets stream provenance; no IQ staging or secondary publication queue.
    property_map readEvidence(int result, int flags, std::int64_t timeNs, std::size_t requested) {
        const bool hasTime = (flags & SOAPY_SDR_HAS_TIME) != 0;
        const bool discontinuity = result == SOAPY_SDR_OVERFLOW || result == SOAPY_SDR_CORRUPTION || (flags & SOAPY_SDR_END_ABRUPT) ||
            (result > 0 && hasTime && _expectedTime && std::abs(static_cast<long double>(timeNs) - *_expectedTime) > 1.5L);
        property_map raw{{"soapy_read_result", static_cast<std::int32_t>(result)}, {"soapy_read_flags", static_cast<std::int32_t>(flags)},
            {"soapy_time_ns", timeNs}, {"has_device_time", hasTime}, {"read_requested", static_cast<std::uint64_t>(requested)},
            {"read_ordinal", _readOrdinal++}, {"sample_index", _sampleIndex}, {"epoch_sample_index", _epochSamples},
            {"source_identity", _sourceIdentity}, {"hardware_identity", _identity}, {"source_instance", std::string(this->unique_name)}, {"run_id", _run},
            {"configuration_id", _appliedId}, {"applied_configuration_id", _appliedId}, {"configuration_revision", _epoch}, {"sdr_epoch", _epoch},
            {"device_clock", std::string(hasTime ? "SoapySDR default hardware clock (not UTC)" : "unknown")},
            {"epoch_freshness", epoch_freshness.value}, {"stream_generation", _streamGeneration}, {"freshness_verified", false},
            {"stream_boundary_verified", restartFreshness() && _streamActive}, {"samples_admitted", false}, {"host_receive_time_ns", detail::wallClockNs()}};
        if (hasTime) { raw.insert_or_assign("device_time_ns", timeNs); }
        if (discontinuity) { raw.insert_or_assign("discontinuity", true); }
        if (result == SOAPY_SDR_OVERFLOW) { tag::put(raw, tag::RX_OVERFLOW, true); }
        return raw;
    }
    bool admitRead(property_map& raw, int result, int flags, std::int64_t timeNs, std::size_t requested) {
        if (cancellationPending()) { terminal("cancelled"); return false; }
        if (!ioActive()) { return false; }
        if (raw.contains("discontinuity") && acquisitionMode()) { terminal("stream_discontinuity", "timestamp/driver discontinuity", true); return false; }
        if (result <= 0) {
            if (flags & SOAPY_SDR_END_BURST) { terminal(sampleTarget() ? "premature_end_burst" : "normal_completion", "empty end of burst", sampleTarget() != 0); }
            else if (result == SOAPY_SDR_TIMEOUT || result == 0) {
                if (acquisitionMode() && ++_timeouts >= max_timeout_count) { terminal("timeout", "read timeout limit", true); }
            } else if (raw.contains("discontinuity") && acquisitionMode()) { terminal("stream_discontinuity", "driver reported discontinuity", true); }
            else if (!handleStreamError(result)) { terminal("driver_failure", std::format("readStream: {}", result), true); }
            return false;
        }
        _timeouts = 0;
        if (static_cast<std::size_t>(result) > requested) { terminal("driver_failure", "read exceeds reserved span", true); return false; }
        if (require_fresh_samples && !restartFreshness()) {
            if (!(flags & SOAPY_SDR_HAS_TIME) || !_freshBoundary) { terminal("freshness_failure", "missing timestamp/fence", true); return false; }
            if (timeNs < *_freshBoundary) {
                _status.insert_or_assign("discarded_stale_reads", static_cast<std::uint64_t>(++_staleReads));
                if (flags & SOAPY_SDR_END_BURST) { terminal("premature_end_burst", "burst ended at stale fence", true); }
                else if (_staleReads >= max_stale_reads) { terminal("freshness_failure", "stale read limit", true); }
                return false;
            }
        }
        handleStreamFlags(flags);
        if (!ioActive()) { terminal("driver_failure", "fragment limit", true); return false; }
        _fresh = require_fresh_samples && !restartFreshness();
        raw.insert_or_assign("freshness_verified", _fresh);
        raw.insert_or_assign("samples_admitted", true);
        _status.insert_or_assign("freshness", std::string(restartFreshness() ? "stream_restart" : _fresh ? "timestamp_verified" : "unverified"));
        _expectedTime.reset();
        if ((flags & SOAPY_SDR_HAS_TIME) && !_actualRates.empty() && _actualRates[0] > 0) {
            _expectedTime = static_cast<long double>(timeNs) + static_cast<long double>(result) * 1.e9L / _actualRates[0];
        }
        return true;
    }
    void accountSamples(std::size_t count, int flags) {
        _sampleIndex += count;
        _epochSamples += count;
        _status.insert_or_assign("published_samples", _sampleIndex);
        _status.insert_or_assign("epoch_samples", _epochSamples);
        if (sampleTarget() && _epochSamples == sampleTarget()) {
            if (_boundedMode) { pauseEpoch(); }
            else { terminal("normal_completion"); }
        } else if (flags & SOAPY_SDR_END_BURST) {
            terminal(sampleTarget() ? "premature_end_burst" : "normal_completion", {}, sampleTarget() != 0);
        }
    }
    void ioReadLoop() {
        const auto previousOwner = std::exchange(_ioOwner, this);
        on_scope_exit restoreOwner = [previousOwner] { _ioOwner = previousOwner; };
        thread_pool::thread::setThreadName(std::format("soapy:{}", this->name.value));
        auto& clkReader = clk_in.streamReader();
        auto& clkTags = clk_in.tagReader();
        try {
            const auto capacity = outputCapacity();
            using Span = decltype(output(0).streamWriter().template tryReserve<SpanReleasePolicy::ProcessNone>(0UZ));
            std::vector<Span> spans;
            spans.reserve(num_channels);
            while (ioActive()) {
                spans.clear(); // discard any unadmitted reservation before applying settings or waiting
                // Preserve a staged batch arriving during a slow driver setter.
                if (!this->settings().stagedParameters().empty()) { this->settings().setChanged(true); }
                this->applyChangedSettings(false);
                syncStatus();
                if (!ioActive()) { break; }
                drainClockInput(clkReader, clkTags);
                if (!ioActive()) { break; }
                if (!this->settings().stagedParameters().empty()) { continue; }
                if (_phase == Phase::Paused) {
                    _status.insert_or_assign("waiting_for", std::string("configuration"));
                    std::this_thread::sleep_for(std::chrono::microseconds(200));
                    continue;
                }
                std::size_t room = capacity;
                bool tagRoom = true;
                for (std::size_t i = 0; i < num_channels; ++i) {
                    room = std::min(room, output(i).streamWriter().available());
                    // One read tag includes host timing and pending discontinuities.
                    // Keep Core's EOS reserve without increasing tag/IQ density.
                    tagRoom = tagRoom && output(i).tagWriter().available() >= 2UZ;
                }
                auto count = readSize(max_chunk_size, capacity, room, _mtu); // refreshed after activation/recreation
                if (sampleTarget()) { count = static_cast<std::size_t>(std::min<std::uint64_t>(count, sampleTarget() - _epochSamples)); }
                _status.insert_or_assign("output_room", static_cast<std::uint64_t>(room));
                _status.insert_or_assign("waiting_for", std::string(!tagRoom ? "metadata" : count == 0 ? "samples" : "driver"));
                if (count == 0 || !tagRoom) {
                    std::this_thread::sleep_for(std::chrono::microseconds(200));
                    continue;
                }
                applyDirtyFlags();
                spans.clear();
                for (std::size_t i = 0; i < num_channels; ++i) { spans.push_back(output(i).streamWriter().template tryReserve<SpanReleasePolicy::ProcessNone>(count)); }
                if (std::ranges::any_of(spans, [](const auto& span) { return span.empty(); })) { continue; }
                int flags = 0;
                long long timeNs = 0;
                const int result = _rxStream.readStreamIntoBufferList(flags, timeNs, static_cast<long>(max_time_out_us.value), spans);
                auto raw = readEvidence(result, flags, timeNs, count);
                const bool admitted = admitRead(raw, result, flags, timeNs, count);
                _status.insert_or_assign("last_read", raw);
                if (!admitted) {
                    if (!acquisitionMode() && raw.contains("discontinuity")) {
                        _pendingDiscontinuity.record(raw);
                        _status.insert_or_assign("pending_discontinuity", _pendingDiscontinuity.evidence());
                    }
                    continue;
                }
                _overflowCount.store(0U, std::memory_order_relaxed);
                const auto samples = static_cast<std::size_t>(result);
                const auto wallNs = detail::wallClockNs();
                if (ppm_estimator_cutoff > 0.f) { _rateEstimator.update(static_cast<double>(wallNs) * 1e-9, samples); }
                property_map timing;
                if (emit_timing_tags) {
                    const auto interval = static_cast<std::uint64_t>(tag_interval.value * 1e9f);
                    if (interval == 0 || _lastTagTimeNs == 0 || wallNs - _lastTagTimeNs >= interval) {
                        timing = timingMetadata(samples, wallNs);
                        _lastTagTimeNs = wallNs;
                    }
                }
                // Publish metadata before making samples visible on any channel.
                for (std::size_t i = 0; i < num_channels; ++i) {
                    auto map = raw;
                    for (const auto& [key, val] : timing) { map.insert_or_assign(key, val); }
                    if (!timing.empty() && !frequency->empty()) {
                        const double requestedFrequency = frequency.value[std::min(i, frequency->size() - 1)];
                        if (emit_meta_info) {
                            auto& meta = *map.at(tag::TRIGGER_META_INFO.key()).template get_if<property_map>();
                            tag::put(meta, "requested_center_frequency_hz", requestedFrequency);
                            tag::put(meta, "channel", static_cast<gr::Size_t>(channel(i)));
                        }
                        if (auto ppm = timing.find("ppm_error"); ppm != timing.end()) {
                            tag::put(map, "estimated_center_frequency_hz", requestedFrequency * (1.0 + static_cast<double>(*ppm->second.template get_if<float>()) * 1e-6));
                        }
                    }
                    if (_pendingDiscontinuity.count != 0) {
                        map.insert_or_assign("preceding_discontinuity", _pendingDiscontinuity.evidence());
                        map.insert_or_assign("discontinuity", true);
                        if (_pendingDiscontinuity.overflows != 0) { tag::put(map, tag::RX_OVERFLOW, true); }
                    }
                    for (const auto& [key, val] : _readbacks[i]) { map.insert_or_assign(key, val); }
                    if (_readbacks[i].contains("actual_sample_rate_hz")) { tag::put(map, tag::SAMPLE_RATE, static_cast<float>(*_readbacks[i].at("actual_sample_rate_hz").template get_if<double>())); }
                    if (_readbacks[i].contains("actual_center_frequency_hz")) { tag::put(map, tag::FREQUENCY, *_readbacks[i].at("actual_center_frequency_hz").template get_if<double>()); }
                    const auto dropped = output(i).nTagsDropped;
                    output(i).publishTag(std::move(map), 0UZ);
                    if (output(i).nTagsDropped != dropped) { terminal("provenance_failure", "read tag publication failed", true); }
                }
                if (_failed.load()) {
                    raw.insert_or_assign("samples_admitted", false);
                    _status.insert_or_assign("last_read", std::move(raw));
                    break;
                }
                if constexpr (nPorts == 1 && std::is_same_v<T, std::complex<float>>) {
                    if (dc_blocker_enabled) { applyDcBlocker(spans[0].data(), samples); }
                }
                for (auto& span : spans) { span.publish(samples); }
                // Reservations must commit before pausing/publishing a receipt.
                spans.clear();
                _pendingDiscontinuity = {};
                _status.erase("pending_discontinuity");
                accountSamples(samples, flags);
                this->progress->incrementAndGet();
                this->progress->notify_all();
            }
        } catch (const std::exception& e) { terminal("source_failure", e.what(), true); }
        catch (...) { terminal("source_failure", "unknown I/O exception", true); }
        if (!_status.contains("terminal_cause")) { terminal(cancellationPending() ? "cancelled" : "explicit_stop"); }
        if (_boundedMode && (_phase != Phase::Paused || _failed.load())) { _status.insert_or_assign("epoch_failure", epochReceipt(false)); }
        cleanup();
        if (clk_in.isConnected()) { std::ignore = clk_in.disconnect(); }
        syncStatus();
        this->progress->incrementAndGet();
        this->progress->notify_all();
        gr::atomic_ref(_ioThreadDone).store_release(true);
        gr::atomic_ref(_ioThreadDone).notify_all();
    }

    /// Takes the clock input's tags and samples. A trigger time sets the offset of the clock from the wall clock, a
    /// trigger name sets the name the timing tags carry, and an end-of-stream tag ends the read loop. The samples are
    /// discarded. The read loop calls this on every pass, as the end-of-stream tag can arrive alone after the last
    /// sample.
    void drainClockInput(auto& clkReader, auto& clkTagRdr) {
        if (!clk_in.isConnected()) {
            return;
        }

        auto        tagData       = clkTagRdr.get(clkTagRdr.available());
        std::size_t nTagsConsumed = 0UZ;

        for (const auto& clkTag : tagData) {
            ++nTagsConsumed;
            if (auto it = clkTag.map.find(std::pmr::string(tag::TRIGGER_TIME.key())); it != clkTag.map.end()) {
                if (auto* timePtr = it->second.template get_if<std::uint64_t>()) {
                    auto triggerUtcNs = static_cast<std::int64_t>(*timePtr);
                    _clockOffsetNs    = triggerUtcNs - static_cast<std::int64_t>(detail::wallClockNs());
                    _clockOffsetValid = true;
                }
            }
            if (auto it = clkTag.map.find(std::pmr::string(tag::TRIGGER_NAME.key())); it != clkTag.map.end()) {
                if (auto* namePtr = it->second.template get_if<std::pmr::string>()) {
                    if (!namePtr->empty()) {
                        _clockTriggerName = std::string(*namePtr);
                    }
                }
            }
            static const std::pmr::string kEosKey(tag::END_OF_STREAM.key());
            if (clkTag.map.contains(kEosKey)) {
                _clockEosReceived = true;
            }
        }

        std::ignore = tagData.consume(nTagsConsumed);
        if (const std::size_t nAvailable = clkReader.available(); nAvailable > 0UZ) {
            auto clkSpan = clkReader.get(nAvailable);
            std::ignore  = clkSpan.consume(nAvailable);
        }

        if (_clockEosReceived) {
            _ioStopRequested.store(true, std::memory_order_release);
        }
    }

    property_map timingMetadata(std::size_t nSamples, std::uint64_t tWallNs) {
        auto tUtcLastNs      = static_cast<std::int64_t>(tWallNs) + _clockOffsetNs;
        auto chunkDurationNs = static_cast<std::int64_t>(static_cast<double>(nSamples - 1) / static_cast<double>(sample_rate.value) * 1e9);
        auto tUtcFirstNs     = static_cast<std::uint64_t>(std::max(std::int64_t{0}, tUtcLastNs - chunkDurationNs));

        bool hasClock = clk_in.isConnected() && !_clockTriggerName.empty();

        property_map tagMap;
        tag::put(tagMap, tag::TRIGGER_NAME, hasClock ? _clockTriggerName : trigger_name.value);
        tag::put(tagMap, tag::TRIGGER_TIME, tUtcFirstNs);
        tag::put(tagMap, tag::TRIGGER_OFFSET, 0.f);

        if (emit_meta_info) {
            property_map metaInfo;
            tag::put(metaInfo, "trigger_source", std::string("SoapySource"));

            std::string clockSource = (_clockOffsetValid && !_clockTriggerName.empty()) ? _clockTriggerName : std::string("wallclock");
            tag::put(metaInfo, "clock_source", std::move(clockSource));

            if (_clockOffsetValid) {
                tag::put(metaInfo, "clock_offset_ns", _clockOffsetNs);
            }

            tag::put(metaInfo, "requested_sample_rate_hz", static_cast<double>(sample_rate.value));
            tag::put(tagMap, tag::TRIGGER_META_INFO, std::move(metaInfo));
        }

        if (_rateEstimator._initialised && ppm_estimator_cutoff > 0.f) {
            float ppmNow = _rateEstimator.estimatedPpm();
            tag::put(tagMap, "estimated_sample_rate_hz", _rateEstimator.estimatedRate());
            tag::put(tagMap, "ppm_error", ppmNow);
            _ppmLastEmitted = ppmNow;
        }

        return tagMap;
    }

    void reinitDevice() {
        _devKwargs = soapy::Kwargs{{"driver", device.value}};
        if (!device_parameter->empty()) {
            _devKwargs.merge(soapy::parseKwargsString(device_parameter.value));
        }
        _activation = soapy::detail::DeviceRegistry::Registration(_devKwargs);

        auto devResult = soapy::Device::make(_devKwargs);
        if (!devResult) {
            failStart(devResult.error().message);
        }
        _device = std::move(*devResult);
        const auto info = _device.hardwareInfo();
        auto serial = info.find("serial");
        // Soapy UHD exposes the opened motherboard's serial as mboard_serial.
        // Use only driver-reported hardware info, never the selector kwargs.
        if (serial == info.end() || serial->second.empty()) { serial = info.find("mboard_serial"); }
        const auto hardware = _device.hardwareKey();
        _identity = (serial != info.end() && !serial->second.empty() && !hardware.empty()) ? std::format("{}:{}:{}", hardware.size(), hardware, serial->second) : "";
        // Source provenance is still available when the driver reports no hardware
        // identity. This labels the block instance, never a fabricated radio serial.
        _sourceIdentity = _identity.empty() ? std::format("instance:{}", this->unique_name) : _identity;
        _status.insert_or_assign("source_identity", _sourceIdentity);
        _status.insert_or_assign("hardware_identity", _identity);
        auto claim = _receiveLease.acquire(_identity);
        _status.insert_or_assign("ownership", std::string(claim ? "local_rx_exclusive" : "unavailable"));
        if (!claim) { _status.insert_or_assign("ownership_unavailable_reason", claim.error()); }
        if (!claim && (require_receive_ownership || (!_receiveLease.backendUnavailable() && !_identity.empty()))) { failStart(claim.error()); }

        std::size_t nChannelMax    = _device.getNumChannels(SOAPY_SDR_RX);
        std::size_t nChannelNeeded = (nPorts != std::dynamic_extent) ? nPorts : static_cast<std::size_t>(num_channels.value);
        if (nChannelMax < nChannelNeeded || (!channels->empty() && std::ranges::any_of(channels.value, [&](auto ch) { return ch >= nChannelMax; }))) {
            failStart(std::format("channel mismatch: need {} but device has {}", nChannelNeeded, nChannelMax));
        }

        // The order is what the drivers require: the frontend mapping decides which physical channel an
        // index names, a device setting may share a control with a gain element, a driver may derive its
        // filter from the sample rate, and a driver may refuse a gain write while its AGC is on.
        applyFrontendMapping();
        applyClockConfig();
        applyDeviceSettings();
        applySampleRate();
        applyBandwidth();
        applyAntenna();
        applyFrequency();
        // A bool or a zero carries no "not given" value of its own, so what the caller wrote decides
        // whether these three reach the device. A later change carries that decision in itself.
        if (isSetByCaller("frequency_correction")) {
            applyFrequencyCorrection();
        }
        if (isSetByCaller("gain_mode")) {
            applyGainMode();
        }
        applyGain();
        applyGainElements();
        if (isSetByCaller("dc_offset_mode")) {
            applyDcOffsetMode();
        }
        applyDcOffset();
        applyIqBalance();
        logDeviceState();

        setupRxStream();
    }

    void setupRxStream() {
        auto        supportedFormats = _device.getStreamFormats(SOAPY_SDR_RX, channel(0));
        const char* requestedFormat  = soapy::detail::toSoapySDRFormat<T>();
        if (!supportedFormats.empty() && std::ranges::find(supportedFormats, std::string(requestedFormat)) == supportedFormats.end()) {
            failStart(std::format("format '{}' not supported (available: {})", requestedFormat, gr::join(supportedFormats, ", ")));
        }

        std::vector<gr::Size_t> channelIndices(num_channels);
        for (std::size_t i = 0; i < channelIndices.size(); ++i) { channelIndices[i] = channel(i); }
        soapy::Kwargs parsedStreamArgs = stream_args->empty() ? soapy::Kwargs{} : soapy::parseKwargsString(stream_args.value);
        auto          streamResult     = _device.setupStream<T, SOAPY_SDR_RX>(channelIndices, parsedStreamArgs);
        if (!streamResult) {
            failStart(std::format("{} (requested: {})", streamResult.error(), requestedFormat));
        }
        _rxStream = std::move(*streamResult);
        _mtu = 0;
        _status.erase("stream_mtu");
        _status.insert_or_assign("stream_generation", ++_streamGeneration);
        _rateEstimatorDirty.store(true, std::memory_order_release);
    }

    void applyClockConfig() {
        if (!clock_source->empty()) {
            if (auto r = _device.setClockSource(clock_source.value); !r) {
                configurationError("applyClockConfig()", r.error());
            }
        }
        if (!time_source->empty()) {
            if (auto r = _device.setTimeSource(time_source.value); !r) {
                configurationError("applyClockConfig()", r.error());
            }
        }
        if (master_clock_rate > 0.0) {
            if (auto r = _device.setMasterClockRate(master_clock_rate); !r) {
                configurationError("applyClockConfig()", r.error());
            }
        }
        if (reference_clock_rate > 0.0) {
            if (auto r = _device.setReferenceClockRate(reference_clock_rate); !r) {
                configurationError("applyClockConfig()", r.error());
            }
        }
    }

    void applySampleRate() {
        for (gr::Size_t i = 0U; i < num_channels; i++) {
            if (auto r = _device.setSampleRate(SOAPY_SDR_RX, channel(i), static_cast<double>(sample_rate)); !r) {
                configurationError("applySampleRate()", r.error());
            }
        }
    }

    // A driver accepts an antenna name it does not have without reporting an error, so the name is checked
    // against the device's own list first.
    void applyAntenna() {
        if (rx_antennae->empty()) {
            return;
        }
        for (gr::Size_t i = 0U; i < num_channels; i++) {
            const auto& ant = rx_antennae->at(std::min(static_cast<std::size_t>(i), rx_antennae->size() - 1UZ));
            if (ant.empty()) {
                continue;
            }
            const auto available = _device.listAvailableAntennas(SOAPY_SDR_RX, channel(i));
            if (!available.empty() && std::ranges::find(available, ant) == available.end()) {
                configurationError("applyAntenna()", std::format("channel {} has no antenna '{}' (device has: {})", i, ant, gr::join(available, ", ")));
            }
            if (auto r = _device.setAntenna(SOAPY_SDR_RX, channel(i), ant); !r) {
                configurationError("applyAntenna()", r.error());
            }
        }
    }

    // Complete-center tuning lets the driver coordinate RF and baseband. A named
    // component is an explicit opt-in for drivers whose generic distribution is unsuitable.
    void applyFrequency() {
        if (frequency->empty()) {
            return;
        }
        const soapy::Kwargs tuneArgs = tune_args->empty() ? soapy::Kwargs{} : soapy::parseKwargsString(tune_args.value);
        for (gr::Size_t i = 0U; i < num_channels; i++) {
            double     freq       = frequency->at(std::min(static_cast<std::size_t>(i), frequency->size() - 1UZ));
            auto component = frequency_component.value;
            if (component.empty() && !acquisitionMode()) {
                const auto components = _device.listFrequencyComponents(SOAPY_SDR_RX, channel(i));
                if (std::ranges::find(components, std::string("RF")) != components.end()) { component = "RF"; }
            }
            const auto result = component.empty() ? _device.setCenterFrequency(SOAPY_SDR_RX, channel(i), freq, tuneArgs) : _device.setFrequencyComponent(SOAPY_SDR_RX, channel(i), component, freq, tuneArgs);
            if (!result) {
                configurationError("applyFrequency()", result.error());
            }
        }
    }

    void applyBandwidth() {
        if (rx_bandwidths->empty()) {
            return;
        }
        for (gr::Size_t i = 0U; i < num_channels; i++) {
            double bw = rx_bandwidths->at(std::min(static_cast<std::size_t>(i), rx_bandwidths->size() - 1UZ));
            if (auto r = _device.setBandwidth(SOAPY_SDR_RX, channel(i), bw); !r) {
                configurationError("applyBandwidth()", r.error());
            }
        }
    }

    void applyGain() {
        if (rx_gains->empty()) {
            return;
        }
        for (gr::Size_t i = 0U; i < num_channels; i++) {
            double g = rx_gains->at(std::min(static_cast<std::size_t>(i), rx_gains->size() - 1UZ));
            if (!withinRange(_device.getGainRange(SOAPY_SDR_RX, channel(i)), g)) {
                configurationError("applyGain()", std::format("channel {}: {} dB is outside the device's overall gain range {}", i, g, _device.getGainRange(SOAPY_SDR_RX, channel(i))));
            }
            if (auto r = _device.setGain(SOAPY_SDR_RX, channel(i), g); !r) {
                configurationError("applyGain()", r.error());
            }
        }
    }

    // Elements are applied in the order the driver lists them, which is the order the driver itself uses when
    // it distributes an overall gain across them.
    void applyGainElements() {
        if (rx_gain_elements->empty()) {
            return;
        }
        for (gr::Size_t i = 0U; i < num_channels; i++) {
            const auto available = _device.listAvailableGainElements(SOAPY_SDR_RX, channel(i));
            for (const auto& name : available) {
                const auto entry = rx_gain_elements->find(name);
                if (entry == rx_gain_elements->end()) {
                    continue;
                }
                const auto gain = pmt::convert_safely<double>(entry->second);
                if (!gain) {
                    configurationError("applyGainElements()", std::format("gain element '{}' needs a number: {}", name, gain.error()));
                }
                if (!withinRange(_device.getGainRange(SOAPY_SDR_RX, channel(i), name), *gain)) {
                    configurationError("applyGainElements()", std::format("channel {}: {} dB is outside element '{}' range {}", i, *gain, name, _device.getGainRange(SOAPY_SDR_RX, channel(i), name)));
                }
                if (auto r = _device.setGain(SOAPY_SDR_RX, channel(i), *gain, name); !r) {
                    configurationError("applyGainElements()", r.error());
                }
            }
            for (const auto& entry : rx_gain_elements.value) {
                if (std::ranges::find(available, std::string(entry.first)) == available.end()) {
                    configurationError("applyGainElements()", std::format("channel {} has no gain element '{}' (device has: {})", i, entry.first, gr::join(available, ", ")));
                }
            }
        }
    }

    void applyGainMode() {
        for (gr::Size_t i = 0U; i < num_channels; i++) {
            if (!_device.hasAutomaticGainControl(SOAPY_SDR_RX, channel(i))) {
                continue;
            }
            if (auto r = _device.setAutomaticGainControl(SOAPY_SDR_RX, channel(i), gain_mode); !r) {
                configurationError("applyGainMode()", r.error());
            }
        }
    }

    void applyFrequencyCorrection() {
        for (gr::Size_t i = 0U; i < num_channels; i++) {
            if (!_device.hasFrequencyCorrection(SOAPY_SDR_RX, channel(i))) {
                continue;
            }
            if (auto r = _device.setFrequencyCorrection(SOAPY_SDR_RX, channel(i), frequency_correction); !r) {
                configurationError("applyFrequencyCorrection()", r.error());
            }
        }
    }

    void applyDcOffsetMode() {
        for (gr::Size_t i = 0U; i < num_channels; i++) {
            if (!_device.hasDCOffsetMode(SOAPY_SDR_RX, channel(i))) {
                continue;
            }
            if (auto r = _device.setDCOffsetMode(SOAPY_SDR_RX, channel(i), dc_offset_mode); !r) {
                configurationError("applyDcOffsetMode()", r.error());
            }
        }
    }

    void applyDcOffset() {
        if (dc_offset->empty()) {
            return;
        }
        for (gr::Size_t i = 0U; i < num_channels; i++) {
            if (!_device.hasDCOffset(SOAPY_SDR_RX, channel(i))) {
                continue;
            }
            auto idx = static_cast<std::size_t>(i) * 2UZ;
            if (idx + 1UZ >= dc_offset->size()) {
                break;
            }
            if (auto r = _device.setDCOffset(SOAPY_SDR_RX, channel(i), dc_offset->at(idx), dc_offset->at(idx + 1UZ)); !r) {
                configurationError("applyDcOffset()", r.error());
            }
        }
    }

    void applyIqBalance() {
        if (iq_balance->empty()) {
            return;
        }
        for (gr::Size_t i = 0U; i < num_channels; i++) {
            if (!_device.hasIQBalance(SOAPY_SDR_RX, channel(i))) {
                continue;
            }
            auto idx = static_cast<std::size_t>(i) * 2UZ;
            if (idx + 1UZ >= iq_balance->size()) {
                break;
            }
            if (auto r = _device.setIQBalance(SOAPY_SDR_RX, channel(i), iq_balance->at(idx), iq_balance->at(idx + 1UZ)); !r) {
                configurationError("applyIqBalance()", r.error());
            }
        }
    }

    void applyFrontendMapping() {
        if (!frontend_mapping->empty()) {
            auto result = soapy::Device::readback([&] { _device.setFrontendMapping(SOAPY_SDR_RX, frontend_mapping.value); return true; });
            if (!result) { configurationError("frontend_mapping", result.error()); }
        }
    }

    // A key the device does not have is ignored by the driver rather than reported, so keys are checked
    // against the device's own list. That list is read only when the caller supplied settings: on some
    // drivers getSettingInfo() reopens the device.
    void applyDeviceSettings() {
        if (device_settings->empty()) {
            return;
        }
        std::vector<std::string> known;
        for (const auto& info : _device.getSettingInfo()) {
            known.push_back(info.key);
        }
        for (const auto& [key, value] : soapy::parseKwargsString(device_settings.value)) {
            if (std::ranges::find(known, key) == known.end()) {
                configurationError("applyDeviceSettings()", known.empty() ? std::format("the device has no settings, so '{}' cannot be written", key) : std::format("the device has no setting '{}' (it has: {})", key, gr::join(known, ", ")));
            }
            if (auto r = _device.writeSetting(key, value); !r) {
                configurationError("applyDeviceSettings()", r.error());
            }
        }
    }

    // The caller's keys are the writable members the settings' auto-update set no longer holds: that set
    // starts as every writable member and a write removes a key from it whatever value the key carries, so
    // a map that names every writable member, which is what a fully serialized block is loaded from, leaves
    // it empty. Its emptiness therefore cannot stand for "nothing was written": settings that hold no
    // parameters at all have an empty set for the other reason, and the count of stored parameter sets,
    // which a written map and a constructed block both raise above zero, is what tells the two apart.
    [[nodiscard]] bool isSetByCaller(std::string_view key) {
        if (this->settings().getNStoredParameters() == 0U) {
            return false;
        }
        return !this->settings().autoUpdateParameters().contains(std::string(key));
    }

    // A driver that reports a degenerate range gives nothing to check against.
    [[nodiscard]] static bool withinRange(const soapy::Range& range, double value) { return range.maximum <= range.minimum || (value >= range.minimum && value <= range.maximum); }

    void logDeviceState() {
        if (!log_device_state) {
            return;
        }
        for (gr::Size_t i = 0U; i < num_channels; i++) {
            std::string gains;
            for (const auto& name : _device.listAvailableGainElements(SOAPY_SDR_RX, channel(i))) {
                gains += std::format("{}{}={:g}", gains.empty() ? "" : " ", name, _device.getGain(SOAPY_SDR_RX, channel(i), name));
            }
            std::string written;
            for (const auto& [key, value] : device_settings->empty() ? soapy::Kwargs{} : soapy::parseKwargsString(device_settings.value)) {
                written += std::format("{}{}={}", written.empty() ? "" : " ", key, _device.readSetting(key));
            }
            std::println(stderr, "[SoapySource] {} ch{}: rate={:g} freq={:g} bw={:g} antenna={} agc={} gain={:g} [{}] corr={:g} settings [{}]", device.value, i, _device.getSampleRate(SOAPY_SDR_RX, channel(i)), _device.getCenterFrequency(SOAPY_SDR_RX, channel(i)), _device.getBandwidth(SOAPY_SDR_RX, channel(i)), _device.getAntenna(SOAPY_SDR_RX, channel(i)), _device.isAutomaticGainControl(SOAPY_SDR_RX, channel(i)), _device.getGain(SOAPY_SDR_RX, channel(i)), gains, _device.getFrequencyCorrection(SOAPY_SDR_RX, channel(i)), written);
        }
    }

    void rebuildDcFilter() {
        if constexpr (std::is_same_v<T, std::complex<float>>) {
            if (dc_blocker_enabled && dc_blocker_cutoff > 0.f && sample_rate > 0.f) {
                _dcFilterI.setCutoff(dc_blocker_cutoff.value, sample_rate.value);
                _dcFilterQ.setCutoff(dc_blocker_cutoff.value, sample_rate.value);
            }
        }
    }

    void rebuildRateEstimator() {
        if (ppm_estimator_cutoff > 0.f) {
            // one update per read, each read at the size readSize() gives when every output has room
            const std::size_t nRead         = readSize(max_chunk_size, outputCapacity(), outputCapacity(), _mtu);
            double            nomRate       = static_cast<double>(sample_rate.value);
            double            updateHz      = (nomRate > 0.0) ? (nomRate / static_cast<double>(nRead)) : 250.0;
            _rateEstimator.filter_cutoff_hz = ppm_estimator_cutoff;
            _rateEstimator.reset(nomRate, updateHz);
        }
    }

    void applyDcBlocker(std::complex<float>* samples, std::size_t nSamples) {
        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            float filteredI = _dcFilterI.processOne(samples[i].real());
            float filteredQ = _dcFilterQ.processOne(samples[i].imag());
            samples[i]      = {filteredI, filteredQ};
        }
    }

    bool handleStreamError(int ret) {
        switch (ret) {
        case SOAPY_SDR_OVERFLOW: {
            auto count = _overflowCount.fetch_add(1U, std::memory_order_relaxed) + 1U;
            if (verbose_overflow) {
                std::println(stderr, "[SoapySource] OVERFLOW #{}", count);
            }

            if (max_overflow_count > 0 && count >= max_overflow_count) {
                terminal("stream_discontinuity", "overflow limit", true);
                _ioStopRequested.store(true, std::memory_order_release);
                return false;
            }
            return true;
        }
        case SOAPY_SDR_CORRUPTION:
            terminal("driver_failure", "CORRUPTION", true);
            _ioStopRequested.store(true, std::memory_order_release);
            return false;
        default:
            terminal("driver_failure", std::format("stream error: {}", ret), true);
            _ioStopRequested.store(true, std::memory_order_release);
            return false;
        }
    }

    void handleStreamFlags(int flags) {
        if (max_fragment_count > 0 && (flags & SOAPY_SDR_MORE_FRAGMENTS)) {
            auto count = _fragmentCount.fetch_add(1U, std::memory_order_relaxed) + 1U;
            if (count > max_fragment_count) {
                terminal("stream_discontinuity", "fragment limit", true);
                _ioStopRequested.store(true, std::memory_order_release);
            }
        } else {
            _fragmentCount.store(0U, std::memory_order_relaxed);
        }
    }

    void applyDirtyFlags() {
        if (_dcFilterDirty.exchange(false, std::memory_order_acquire)) {
            rebuildDcFilter();
        }
        if (_rateEstimatorDirty.exchange(false, std::memory_order_acquire)) {
            rebuildRateEstimator();
        }
    }

    template<typename U>
    static std::vector<U> initDefaultValues(U initialValue) {
        if constexpr (nPorts != std::dynamic_extent) {
            return std::vector<U>(nPorts, initialValue);
        } else {
            return std::vector<U>(1, initialValue);
        }
    }
};

template<typename T>
using SoapySimpleSource = SoapySource<T, 1UZ>;
template<typename T>
using SoapyDualSource = SoapySource<T, 2UZ>;

} // namespace gr::blocks::sdr

#endif // GNURADIO_SOAPY_SOURCE_HPP
