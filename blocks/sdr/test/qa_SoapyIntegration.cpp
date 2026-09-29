#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <complex>
#include <cstdlib>
#include <filesystem>
#include <numbers>
#include <numeric>
#include <optional>
#include <print>
#include <string>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/basic/ClockSource.hpp>
#include <gnuradio-4.0/common/USBDevice.hpp>
#include <gnuradio-4.0/sdr/LoopbackDevice.hpp>
#include <gnuradio-4.0/sdr/SoapySink.hpp>
#include <gnuradio-4.0/sdr/SoapySource.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

using namespace boost::ut;
using CF32 = std::complex<float>;

// Keeps the first sample of each input chunk. The block takes its input in whole chunks only. Up to one chunk less one
// sample stays unread in the buffer ahead of it after each call.
struct KeepFirstOfChunk : gr::Block<KeepFirstOfChunk, gr::Resampling<>> {
    gr::PortIn<CF32>  in;
    gr::PortOut<CF32> out;

    GR_MAKE_REFLECTABLE(KeepFirstOfChunk, in, out);

    gr::work::Status processBulk(gr::InputSpanLike auto& input, gr::OutputSpanLike auto& output) {
        const auto ratio = static_cast<std::size_t>(input_chunk_size.value);
        for (std::size_t i = 0UZ; i < output.size(); ++i) {
            output[i] = input[i * ratio];
        }
        return gr::work::Status::OK;
    }
};

namespace {

namespace soapy = gr::blocks::sdr::soapy;

// The block and the test reach one device object by opening it with the same arguments, so what the device
// recorded while the graph ran is readable through the SoapySDR API. The probe is opened before the graph
// and held for the whole run, which keeps the device alive after the block releases it.
soapy::Kwargs loopbackKwargs(const std::string& driver, const std::string& parameters) {
    soapy::Kwargs kwargs{{"driver", driver}};
    kwargs.merge(soapy::parseKwargsString(parameters));
    return kwargs;
}

std::vector<std::string> callLog(const soapy::Device& device) {
    const std::string        log = device.readSetting("call_log");
    std::vector<std::string> calls;
    std::size_t              pos = 0UZ;
    while (pos < log.size()) {
        const auto next = log.find(';', pos);
        const auto end  = (next == std::string::npos) ? log.size() : next;
        if (end > pos) {
            calls.push_back(log.substr(pos, end - pos));
        }
        if (next == std::string::npos) {
            break;
        }
        pos = next + 1UZ;
    }
    return calls;
}

std::optional<std::size_t> callIndex(const std::vector<std::string>& calls, std::string_view fragment) {
    for (std::size_t i = 0UZ; i < calls.size(); ++i) {
        if (calls[i].find(fragment) != std::string::npos) {
            return i;
        }
    }
    return std::nullopt;
}

bool sawCall(const std::vector<std::string>& calls, std::string_view fragment) { return callIndex(calls, fragment).has_value(); }

auto runWithWatchdog(auto& sched, std::chrono::seconds timeout = std::chrono::seconds{6}) {
    auto watchdog = std::jthread([&sched, timeout](std::stop_token stoken) {
        auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline && !stoken.stop_requested()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!stoken.stop_requested()) {
            sched.requestStop();
        }
    });
    auto ret      = sched.runAndWait();
    return ret;
}

// A block that stopped itself from its own io thread parks that thread inside its own stop(), waiting for the
// thread that is doing the waiting. Neither the watchdog above nor the graph's teardown, which waits for the io
// thread too, can release it, so the whole case runs on a thread of its own: one that outlives the bound is
// already wedged, and ending the process with the reason makes that a failure in seconds rather than the test
// harness's timeout.
void withinBound(std::chrono::milliseconds bound, std::string_view what, auto&& body) {
    std::atomic<bool> finished{false};

    auto runner = std::thread([&finished, &body] {
        body();
        finished.store(true, std::memory_order_release);
    });

    const auto deadline = std::chrono::steady_clock::now() + bound;
    while (!finished.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!finished.load(std::memory_order_acquire)) {
        std::println(stderr, "[qa_SoapyIntegration] {}: still running {} ms after it should have finished", what, bound.count());
        std::_Exit(1);
    }

    runner.join();
}

// The number of users of a device that have registered and not yet finished configuring it. The device's
// streams are activated when it reaches zero.
std::size_t pendingUsers(const soapy::Kwargs& kwargs) {
    auto  lock    = std::lock_guard(soapy::detail::DeviceRegistry::mutex());
    auto& devices = soapy::detail::DeviceRegistry::devices();
    auto  it      = devices.find(kwargs);
    return it == devices.end() ? 0UZ : it->second.pendingUsers;
}

bool sawError(gr::MsgPortIn& port, std::string_view fragment) {
    auto& reader   = port.streamReader();
    auto  messages = reader.get<gr::SpanReleasePolicy::ProcessAll>(reader.available());
    return std::ranges::any_of(messages, [fragment](const gr::Message& message) { return !message.data.has_value() && message.data.error().message.contains(fragment); });
}

// The receive-only loopback turns its tone by 2*pi*frequency/sampleRate from one sample to the next, so a stream that
// holds every sample once and in order shows that step between every pair of neighbors.
std::optional<std::size_t> firstPhaseBreak(const auto& samples, double frequency, double sampleRate) {
    const double step = 2.0 * std::numbers::pi * frequency / sampleRate;
    for (std::size_t i = 1UZ; i < samples.size(); ++i) {
        const auto turn = std::complex<double>(samples[i]) * std::conj(std::complex<double>(samples[i - 1UZ]));
        if (std::abs(std::arg(turn) - step) > 1e-3) {
            return i;
        }
    }
    return std::nullopt;
}

std::vector<std::size_t> tagIndices(const std::vector<gr::Tag>& tags, std::string_view key) {
    std::vector<std::size_t> indices;
    for (const auto& tag : tags) {
        if (tag.map.contains(std::pmr::string(key))) {
            indices.push_back(tag.index);
        }
    }
    return indices;
}

// true when the indices are 0, nRead, 2 * nRead and so on, one for each read of nRead samples
bool evenlySpaced(const std::vector<std::size_t>& indices, std::size_t nRead) {
    for (std::size_t i = 0UZ; i < indices.size(); ++i) {
        if (indices[i] != i * nRead) {
            return false;
        }
    }
    return !indices.empty();
}

// A pattern of prime length, so that no chunk, read or write size is a multiple of it. The imaginary parts are
// distinct and within full scale, so each sample shows its place in the pattern; every seventh real part lies beyond
// full scale, on alternating sides.
std::vector<CF32> placeMarkedPattern(std::size_t length) {
    std::vector<CF32> values(length);
    for (std::size_t k = 0UZ; k < length; ++k) {
        const float place = -0.99f + 1.98f * static_cast<float>(k) / static_cast<float>(length);
        const float real  = (k % 7UZ != 0UZ) ? 0.5f * place : ((k % 2UZ == 0UZ) ? 1.5f : -2.25f);
        values[k]         = CF32{real, place};
    }
    return values;
}

// What a sink sends for the first n samples of the repeated pattern: each part saturated to full scale and, with a
// ramp time, each sample scaled by the envelope of a linear taper started as the sink starts its own, followed by
// the ramp-down the sink sends from its last sample when it stops.
std::vector<CF32> expectedTransmission(const std::vector<CF32>& values, std::size_t n, std::optional<float> rampTime, float sampleRate) {
    const auto atFullScale = [](CF32 sample) { return CF32{std::clamp(sample.real(), -1.0f, 1.0f), std::clamp(sample.imag(), -1.0f, 1.0f)}; };

    gr::algorithm::BurstTaper<float> taper;
    if (rampTime.has_value()) {
        std::ignore = taper.configure(gr::algorithm::TaperType::Linear, *rampTime, sampleRate, 1.0f);
        std::ignore = taper.setTarget(true);
    }
    std::vector<CF32> sent(n);
    for (std::size_t i = 0UZ; i < n; ++i) {
        sent[i] = atFullScale(rampTime.has_value() ? values[i % values.size()] * taper.processOne() : values[i % values.size()]);
    }
    if (rampTime.has_value()) {
        const CF32 last = sent.back();
        std::ignore     = taper.setTarget(false, true);
        for (std::size_t k = 0UZ, nRamp = taper.rampLength(); k < nRamp; ++k) {
            sent.push_back(atFullScale(last * taper.processOne()));
        }
    }
    return sent;
}

std::optional<std::size_t> firstDifference(const std::vector<CF32>& received, const std::vector<CF32>& expected) {
    const auto mismatch = std::ranges::mismatch(received, expected);
    if (mismatch.in1 == received.end() && mismatch.in2 == expected.end()) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(mismatch.in1 - received.begin());
}

// Sends each channel's pattern through a sink on the loopback device and returns what the device's receiver holds
// once the sink has stopped, which in loopback mode is every sample the sink sent. The probe opens the device with
// the sink's arguments first, so the two share it and the device outlives the sink.
template<std::size_t nChannels>
requires(nChannels == 1UZ || nChannels == 2UZ)
std::array<std::vector<CF32>, nChannels> transmitThroughLoopback(const std::string& parameters, gr::property_map settings, const std::array<std::vector<CF32>, nChannels>& patterns, gr::Size_t nSamples) {
    using gr::blocks::testing::ProcessFunction;
    using Source = gr::blocks::testing::TagSource<CF32, ProcessFunction::USE_PROCESS_BULK>;

    auto probe = soapy::Device::make(loopbackKwargs("loopback", parameters));
    expect(fatal(probe.has_value())) << "the probe must open the device the sink will open";

    settings.insert_or_assign(std::pmr::string("device"), std::string("loopback"));
    settings.insert_or_assign(std::pmr::string("device_parameter"), parameters);
    settings.insert_or_assign(std::pmr::string("num_channels"), static_cast<gr::Size_t>(nChannels));

    gr::Graph                      flow;
    auto&                          sink = flow.emplaceBlock<gr::blocks::sdr::SoapySink<CF32, nChannels>>(std::move(settings));
    std::array<Source*, nChannels> sources{};
    for (std::size_t ch = 0UZ; ch < nChannels; ++ch) {
        sources[ch] = std::addressof(flow.emplaceBlock<Source>({{"n_samples_max", nSamples}, {"values", gr::Tensor<CF32>(patterns[ch].begin(), patterns[ch].end())}}));
    }
    if constexpr (nChannels == 1UZ) {
        expect(flow.connect<"out", "in">(*sources[0], sink).has_value());
    } else {
        expect(flow.connect<"out", "in#0">(*sources[0], sink).has_value());
        expect(flow.connect<"out", "in#1">(*sources[1], sink).has_value());
    }

    gr::scheduler::Simple<> sched;
    expect(sched.exchange(std::move(flow)).has_value());
    expect(runWithWatchdog(sched, std::chrono::seconds{10}).has_value());

    std::vector<gr::Size_t> channels(nChannels);
    std::iota(channels.begin(), channels.end(), gr::Size_t{0});
    auto receiver = probe->setupStream<CF32, SOAPY_SDR_RX>(channels);
    expect(fatal(receiver.has_value()));
    expect(receiver->activate().has_value());

    std::array<std::vector<CF32>, nChannels> received;
    std::vector<std::vector<CF32>>           buffers(nChannels, std::vector<CF32>(4096UZ));
    int                                      nRead = 0;
    do {
        int       flags  = 0;
        long long timeNs = 0;
        nRead            = receiver->readStreamIntoBufferList(flags, timeNs, 0L, buffers);
        for (std::size_t ch = 0UZ; ch < nChannels && nRead > 0; ++ch) {
            received[ch].insert(received[ch].end(), buffers[ch].begin(), buffers[ch].begin() + nRead);
        }
    } while (nRead > 0);
    std::ignore = receiver->deactivate();
    return received;
}

} // namespace

const boost::ut::suite<"SoapySource + Loopback"> integrationTests = [] {
    using namespace gr;
    using namespace gr::blocks::sdr;
    using namespace gr::blocks::testing;
    using Sched = gr::scheduler::Simple<>;

    "single-channel rxOnly delivers samples"_test = [] {
        gr::Graph            flow;
        constexpr gr::Size_t nSamples = 10000;

        auto& source = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({
            {"device", "loopback"},
            {"device_parameter", std::string("device_mode=rx_only")},
            {"sample_rate", 1e6f},
            {"frequency", std::vector{100e3}},
            {"rx_gains", std::vector{0.}},
        });
        auto& sink   = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", nSamples}});
        expect(flow.connect<"out", "in">(source, sink).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(runWithWatchdog(sched).has_value());
        expect(eq(sink.count.value, nSamples));
    };

    "2-channel rxOnly delivers to both sinks"_test = [] {
        gr::Graph            flow;
        constexpr gr::Size_t nSamples = 5000;

        auto& source = flow.emplaceBlock<SoapySource<CF32, 2UZ>>({
            {"device", "loopback"},
            {"device_parameter", std::string("device_mode=rx_only,num_channels=2")},
            {"sample_rate", 1e6f},
            {"num_channels", gr::Size_t{2}},
            {"frequency", std::vector{100e3, 200e3}},
            {"rx_gains", std::vector{0., 0.}},
        });
        auto& sink1  = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", nSamples}});
        auto& sink2  = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", nSamples}});
        expect(flow.connect<"out#0", "in">(source, sink1).has_value());
        expect(flow.connect<"out#1", "in">(source, sink2).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(runWithWatchdog(sched).has_value());
        expect(eq(sink1.count.value, nSamples)) << "channel 0";
        expect(eq(sink2.count.value, nSamples)) << "channel 1";
    };

    "rxOnly propagates tags through the graph"_test = [] {
        gr::Graph            flow;
        constexpr gr::Size_t nSamples = 10000;

        auto& source = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({
            {"device", "loopback"}, {"device_parameter", std::string("device_mode=rx_only")}, {"sample_rate", 1e6f}, {"frequency", std::vector{433.92e6}}, {"rx_gains", std::vector{0.}}, {"emit_timing_tags", true}, {"emit_meta_info", true}, {"tag_interval", 0.f}, // tag every chunk
        });
        auto& sink   = flow.emplaceBlock<TagSink<CF32, ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_expected", nSamples},
            {"log_tags", true},
            {"log_samples", false},
        });
        expect(flow.connect<"out", "in">(source, sink).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(runWithWatchdog(sched).has_value());
        expect(ge(sink._nSamplesProduced, nSamples));
        expect(!sink._tags.empty()) << "should receive at least one tag";
    };

    "clk_in forwards external timing tags"_test = [] {
        gr::Graph            flow;
        constexpr gr::Size_t nSamples = 10000;
        constexpr float      rate     = 1e6f;

        auto& clock = flow.emplaceBlock<gr::blocks::basic::ClockSource<std::uint8_t>>({
            {"sample_rate", rate},
            {"n_samples_max", gr::Size_t{0}}, // unlimited — stopped by scheduler
            {"chunk_size", gr::Size_t{100}},
        });
        clock.tags  = {
            {0, {{std::pmr::string(gr::tag::TRIGGER_NAME.shortKey()), std::pmr::string("GPS_PPS")}, {std::pmr::string(gr::tag::TRIGGER_TIME.shortKey()), std::uint64_t{1'000'000'000ULL}}}},
        };

        auto& source = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({
            {"device", "loopback"},
            {"device_parameter", std::string("device_mode=rx_only")},
            {"sample_rate", rate},
            {"frequency", std::vector{100e3}},
            {"rx_gains", std::vector{0.}},
            {"emit_timing_tags", true},
            {"emit_meta_info", true},
            {"tag_interval", 0.f},
        });

        auto& sink = flow.emplaceBlock<TagSink<CF32, ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_expected", nSamples},
            {"log_tags", true},
            {"log_samples", false},
        });

        expect(flow.connect<"out", "clk_in">(clock, source).has_value());
        expect(flow.connect<"out", "in">(source, sink).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(runWithWatchdog(sched).has_value());
        expect(ge(sink._nSamplesProduced, nSamples));
        expect(!sink._tags.empty()) << "should receive timing tags";

        // verify at least one tag carries the forwarded clock name
        bool foundClockTag = false;
        for (const auto& tag : sink._tags) {
            auto nameIt = tag.map.find(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey()));
            if (nameIt != tag.map.end()) {
                if (auto* name = nameIt->second.get_if<std::pmr::string>()) {
                    if (*name == "GPS_PPS") {
                        foundClockTag = true;
                        break;
                    }
                }
            }
        }
        expect(foundClockTag) << "at least one timing tag should carry the clk_in trigger name";
    };

    "clk_in EoS stops SoapySource"_test = [] {
        gr::Graph            flow;
        constexpr gr::Size_t nClockSamples = 500;
        constexpr float      rate          = 1e6f;

        auto& clock = flow.emplaceBlock<gr::blocks::basic::ClockSource<std::uint8_t>>({
            {"sample_rate", rate},
            {"n_samples_max", nClockSamples},
            {"chunk_size", gr::Size_t{100}},
        });

        auto& source = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({
            {"device", "loopback"},
            {"device_parameter", std::string("device_mode=rx_only")},
            {"sample_rate", rate},
            {"frequency", std::vector{100e3}},
            {"rx_gains", std::vector{0.}},
        });
        auto& sink   = flow.emplaceBlock<TagSink<CF32, ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_expected", gr::Size_t{0}}, // unlimited — stopped by EoS
            {"log_tags", true},
            {"log_samples", false},
        });

        expect(flow.connect<"out", "clk_in">(clock, source).has_value());
        expect(flow.connect<"out", "in">(source, sink).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        auto ret = runWithWatchdog(sched, std::chrono::seconds{4});
        expect(ret.has_value());
        expect(gt(sink._nSamplesProduced, gr::Size_t{0})) << "should have received samples before clk_in EoS";
    };

    "DC blocker removes DC offset from rxOnly tone"_test = [] {
        gr::Graph            flow;
        constexpr gr::Size_t nSamples = 50000; // need enough for IIR to settle

        auto& source = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({
            {"device", "loopback"}, {"device_parameter", std::string("device_mode=rx_only")}, {"sample_rate", 1e6f}, {"frequency", std::vector{0.}}, // DC tone (frequency=0 → constant {1,0})
            {"rx_gains", std::vector{0.}}, {"dc_blocker_enabled", true}, {"dc_blocker_cutoff", 100.f},                                               // aggressive cutoff for fast settling
        });
        auto& sink   = flow.emplaceBlock<TagSink<CF32, ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_expected", nSamples},
            {"log_tags", false},
            {"log_samples", true},
        });
        expect(flow.connect<"out", "in">(source, sink).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(runWithWatchdog(sched).has_value());
        expect(ge(sink._nSamplesProduced, nSamples));

        // the last quarter of samples should have near-zero DC after the filter settles
        auto   totalSamples = sink._samples.size();
        double sumMag       = 0.0;
        auto   startIdx     = totalSamples * 3 / 4;
        for (std::size_t i = startIdx; i < totalSamples; ++i) {
            sumMag += static_cast<double>(std::abs(sink._samples[i]));
        }
        auto avgMag = static_cast<float>(sumMag / static_cast<double>(totalSamples - startIdx));
        expect(lt(avgMag, 0.1f)) << std::format("DC blocker should suppress DC tone, avg magnitude: {}", avgMag);
    };

    "timing tags emitted at configured interval"_test = [] {
        gr::Graph            flow;
        constexpr gr::Size_t nSamples = 50000;
        constexpr float      rate     = 1e6f;

        auto& source = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({
            {"device", "loopback"}, {"device_parameter", std::string("device_mode=rx_only")}, {"sample_rate", rate}, {"frequency", std::vector{100e3}}, {"rx_gains", std::vector{0.}}, {"emit_timing_tags", true}, {"tag_interval", 0.01f}, // 10 ms between tags → expect ~5 tags in 50 ms of data
        });
        auto& sink   = flow.emplaceBlock<TagSink<CF32, ProcessFunction::USE_PROCESS_BULK>>({
            {"n_samples_expected", nSamples},
            {"log_tags", true},
            {"log_samples", false},
        });
        expect(flow.connect<"out", "in">(source, sink).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(runWithWatchdog(sched).has_value());
        expect(ge(sink._nSamplesProduced, nSamples));
        expect(ge(sink._tags.size(), 2UZ)) << std::format("expected multiple timing tags, got {}", sink._tags.size());
    };

    "loopback mode TX→RX through scheduler"_test = [] {
        gr::Graph            flow;
        constexpr gr::Size_t nSamples = 10000;

        // in default loopback mode, rxOnly tone is NOT active — need TX data
        // without a SoapySink, the loopback buffer stays empty → expect TIMEOUT → scheduler stops via watchdog
        // this test verifies the scheduler handles the loopback-without-TX gracefully
        auto& source = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({
            {"device", "loopback"},
            {"sample_rate", 1e6f},
            {"frequency", std::vector{100e3}},
            {"rx_gains", std::vector{0.}},
        });
        auto& sink   = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", nSamples}});
        expect(flow.connect<"out", "in">(source, sink).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());

        // short watchdog — loopback without TX will not deliver samples
        auto ret = runWithWatchdog(sched, std::chrono::seconds{3});
        expect(ret.has_value());
        expect(lt(sink.count.value, nSamples)) << "loopback without TX should not deliver all samples";
    };
};

const boost::ut::suite<"SoapySource read path"> readPathTests = [] {
    using namespace gr;
    using namespace gr::blocks::sdr;
    using namespace gr::blocks::testing;
    using Sched = gr::scheduler::Simple<>;

    static constexpr float  kRate      = 1e6f;
    static constexpr double kFrequency = 100e3;

    struct Received {
        std::vector<CF32>    samples;
        std::vector<gr::Tag> tags;
        std::size_t          maxChunkSize   = 0UZ;
        std::size_t          outputCapacity = 0UZ;
    };

    // Without its rate limit the receive-only loopback hands over a read as soon as it is asked for one, so the
    // source's output fills and every read waits for room. A timing tag marks every read.
    auto receive = [](const std::string& parameters, property_map extraSettings, gr::Size_t nSamples) {
        gr::Graph    flow;
        property_map settings{{"device", "loopback"}, {"device_parameter", parameters}, {"device_settings", std::string("simulate_timing=false")}, {"sample_rate", kRate}, {"frequency", std::vector{kFrequency}}, {"emit_timing_tags", true}, {"tag_interval", 0.f}};
        for (auto& [key, value] : extraSettings) {
            settings.insert_or_assign(key, value);
        }
        auto& source = flow.emplaceBlock<SoapySource<CF32, 1UZ>>(std::move(settings));
        auto& sink   = flow.emplaceBlock<TagSink<CF32, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_expected", nSamples}, {"log_tags", true}, {"log_samples", true}});
        expect(flow.connect<"out", "in">(source, sink).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(runWithWatchdog(sched).has_value());
        return Received{.samples = std::vector<CF32>(sink._samples.begin(), sink._samples.end()), .tags = sink._tags, .maxChunkSize = source.max_chunk_size, .outputCapacity = source.out.bufferSize()};
    };

    "every sample arrives once and in order, and a read's tags mark its first sample"_test = [&receive] {
        constexpr std::size_t kOverflowEvery = 4UZ;
        constexpr gr::Size_t  nSamples       = 200'000U;

        const auto received = receive(std::format("device_mode=rx_only,overflow_every={}", kOverflowEvery), {}, nSamples);
        expect(ge(received.samples.size(), std::size_t{nSamples}));
        const auto brokenAt = firstPhaseBreak(received.samples, kFrequency, static_cast<double>(kRate));
        expect(!brokenAt.has_value()) << std::format("the tone breaks at sample {}", brokenAt.value_or(0UZ));

        // The loopback's MTU is its buffer size, larger than max_chunk_size. A read is max_chunk_size.
        const std::size_t nRead  = received.maxChunkSize;
        const auto        timing = tagIndices(received.tags, gr::tag::TRIGGER_TIME.shortKey());
        expect(evenlySpaced(timing, nRead)) << std::format("{} timing tags, {} samples per read", timing.size(), nRead);

        // every kOverflowEvery-th read reports an overflow and delivers nothing, so its tag sits where the next read
        // starts, after the kOverflowEvery - 1 reads before it
        const auto overflows = tagIndices(received.tags, "rx_overflow");
        expect(!overflows.empty()) << "the device reported overflows";
        for (std::size_t i = 0UZ; i < overflows.size(); ++i) {
            expect(eq(overflows[i], (i + 1UZ) * (kOverflowEvery - 1UZ) * nRead)) << std::format("overflow tag {}", i);
        }
    };

    "a read is at most max_chunk_size samples"_test = [&receive] {
        constexpr std::uint32_t kMaxChunkSize = 2048U;
        const auto              received      = receive("device_mode=rx_only", {{"max_chunk_size", kMaxChunkSize}}, 20'000U);
        expect(!firstPhaseBreak(received.samples, kFrequency, static_cast<double>(kRate)).has_value());
        expect(evenlySpaced(tagIndices(received.tags, gr::tag::TRIGGER_TIME.shortKey()), kMaxChunkSize));
    };

    // The output buffer is no whole number of these reads. A read that finds less room than a full read is shorter,
    // and still a whole number of MTUs where the room holds one.
    "a read is a whole number of MTUs where max_chunk_size holds one"_test = [&receive] {
        constexpr std::size_t kMtu     = 3000UZ; // the loopback reports its buffer size as its MTU
        const auto            received = receive(std::format("device_mode=rx_only,buffer_size={}", kMtu), {}, 60'000U);
        expect(!firstPhaseBreak(received.samples, kFrequency, static_cast<double>(kRate)).has_value());
        const std::size_t nRead  = received.maxChunkSize - received.maxChunkSize % kMtu;
        const auto        timing = tagIndices(received.tags, gr::tag::TRIGGER_TIME.shortKey());
        expect(fatal(ge(timing.size(), 2UZ)));
        expect(eq(timing[0], 0UZ));
        expect(eq(timing[1], nRead)) << "the first read finds the whole buffer free";
        for (std::size_t i = 1UZ; i < timing.size(); ++i) {
            const std::size_t length = timing[i] - timing[i - 1UZ];
            expect(le(length, nRead) and (length % kMtu == 0UZ or length < kMtu)) << std::format("read {} of {} samples", i - 1UZ, length);
        }
    };

    "a read is at most half the output buffer"_test = [&receive] {
        // The loopback's MTU is its buffer size, larger than half the output buffer. A read is half the output buffer.
        const auto received = receive("device_mode=rx_only", {{"max_chunk_size", std::uint32_t{1U << 20U}}}, 200'000U);
        expect(!firstPhaseBreak(received.samples, kFrequency, static_cast<double>(kRate)).has_value());
        const std::size_t nRead = received.outputCapacity / 2UZ;
        expect(evenlySpaced(tagIndices(received.tags, gr::tag::TRIGGER_TIME.shortKey()), nRead)) << std::format("{} samples per read", nRead);
    };

    // A decimator by ten leaves up to nine samples unread in the source's output buffer. A read of the whole buffer
    // would wait for those samples, which the decimator takes only with the next read's.
    "a consumer that leaves part of its input chunk unread does not stop a read of max_chunk_size 65536"_test = [] {
        static constexpr gr::Size_t kDecimated = 50'000U;

        gr::Graph flow;
        auto&     source    = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({
            {"device", "loopback"},
            {"device_parameter", std::string("device_mode=rx_only")},
            {"device_settings", std::string("simulate_timing=false")},
            {"sample_rate", kRate},
            {"frequency", std::vector{kFrequency}},
            {"max_chunk_size", std::uint32_t{65536U}},
            {"emit_timing_tags", false},
        });
        auto&     decimator = flow.emplaceBlock<KeepFirstOfChunk>({{"input_chunk_size", gr::Size_t{10U}}, {"output_chunk_size", gr::Size_t{1U}}});
        auto&     sink      = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", kDecimated}});
        expect(flow.connect<"out", "in">(source, decimator).has_value());
        expect(flow.connect<"out", "in">(decimator, sink).has_value());

        withinBound(std::chrono::seconds{5}, "a source feeding a decimator by ten", [&flow, &sink] {
            Sched sched;
            expect(sched.exchange(std::move(flow)).has_value());
            expect(sched.runAndWait().has_value());
            expect(eq(sink.count.value, kDecimated));
        });
    };

    // A consumer whose input chunk exceeds half the output buffer waits while fewer samples than its chunk are unread.
    // The output then has room, and a read fills it.
    "a consumer whose input chunk exceeds half the output buffer does not stop a read of max_chunk_size 65536"_test = [] {
        static constexpr gr::Size_t kChunk   = 40'000U;
        static constexpr gr::Size_t kOutputs = 100U;

        gr::Graph flow;
        auto&     source  = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({
            {"device", "loopback"},
            {"device_parameter", std::string("device_mode=rx_only")},
            {"device_settings", std::string("simulate_timing=false")},
            {"sample_rate", kRate},
            {"frequency", std::vector{kFrequency}},
            {"max_chunk_size", std::uint32_t{65536U}},
            {"emit_timing_tags", false},
        });
        auto&     reducer = flow.emplaceBlock<KeepFirstOfChunk>({{"input_chunk_size", kChunk}, {"output_chunk_size", gr::Size_t{1U}}});
        auto&     sink    = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", kOutputs}});
        expect(flow.connect<"out", "in">(source, reducer).has_value());
        expect(flow.connect<"out", "in">(reducer, sink).has_value());

        withinBound(std::chrono::seconds{5}, "a source feeding a consumer of 40000-sample chunks", [&flow, &source, &sink] {
            Sched sched;
            expect(sched.exchange(std::move(flow)).has_value());
            expect(sched.runAndWait().has_value());
            expect(eq(sink.count.value, kOutputs));
            const std::size_t capacity = source.out.bufferSize();
            expect(lt(capacity / 2UZ, std::size_t{kChunk}) and le(std::size_t{kChunk}, capacity)) << std::format("the chunk lies between half the output buffer of {} samples and all of it", capacity);
        });
    };

    "both channels of a two-channel read arrive whole, in order and marked together"_test = [] {
        constexpr gr::Size_t nSamples = 100'000U;
        constexpr double     kSecond  = 250e3;

        gr::Graph flow;
        auto&     source = flow.emplaceBlock<SoapySource<CF32, 2UZ>>({
            {"device", "loopback"},
            {"device_parameter", std::string("device_mode=rx_only,num_channels=2")},
            {"device_settings", std::string("simulate_timing=false")},
            {"sample_rate", kRate},
            {"num_channels", gr::Size_t{2}},
            {"frequency", std::vector{kFrequency, kSecond}},
            {"emit_timing_tags", true},
            {"tag_interval", 0.f},
        });
        auto&     sink0  = flow.emplaceBlock<TagSink<CF32, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_expected", nSamples}, {"log_tags", true}, {"log_samples", true}});
        auto&     sink1  = flow.emplaceBlock<TagSink<CF32, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_expected", nSamples}, {"log_tags", true}, {"log_samples", true}});
        expect(flow.connect<"out#0", "in">(source, sink0).has_value());
        expect(flow.connect<"out#1", "in">(source, sink1).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(runWithWatchdog(sched).has_value());

        const std::size_t nRead = source.max_chunk_size;
        for (const auto& [sink, frequency] : {std::pair{&sink0, kFrequency}, std::pair{&sink1, kSecond}}) {
            expect(ge(sink->_samples.size(), std::size_t{nSamples}));
            const auto brokenAt = firstPhaseBreak(sink->_samples, frequency, static_cast<double>(kRate));
            expect(!brokenAt.has_value()) << std::format("the {} Hz tone breaks at sample {}", frequency, brokenAt.value_or(0UZ));
            expect(evenlySpaced(tagIndices(sink->_tags, gr::tag::TRIGGER_TIME.shortKey()), nRead)) << std::format("{} Hz channel", frequency);
        }
    };
};

const boost::ut::suite<"SoapySink write path"> writePathTests = [] {
    using namespace gr;

    static constexpr float      kRate     = 1e6f;
    static constexpr float      kRampTime = 1e-3f;
    static constexpr gr::Size_t kSamples  = 100'000U;

    // The receiver's buffer holds every sample a run sends, and the device takes at most 3001 samples a write, so
    // writes end short of the chunks the sink hands it.
    static const std::string kDevice = "buffer_size=131072,max_write_samples=3001";

    const property_map taperOn{{"burst_taper_enabled", true}, {"burst_ramp_time", kRampTime}, {"burst_taper_type", std::string("Linear")}};

    "every sample reaches the device once and in order, each part saturated to full scale"_test = [] {
        const auto values   = placeMarkedPattern(997UZ);
        const auto received = transmitThroughLoopback<1UZ>(kDevice, {{"sample_rate", kRate}}, {values}, kSamples);
        const auto expected = expectedTransmission(values, kSamples, std::nullopt, kRate);
        expect(eq(received[0].size(), expected.size()));
        const auto differsAt = firstDifference(received[0], expected);
        expect(!differsAt.has_value()) << std::format("the device received another sample at {}", differsAt.value_or(0UZ));
    };

    "with the taper on, every sample reaches the device once, in order and tapered, and the ramp-down follows"_test = [&taperOn] {
        property_map settings = taperOn;
        settings.insert_or_assign(std::pmr::string("sample_rate"), kRate);

        const auto values   = placeMarkedPattern(997UZ);
        const auto received = transmitThroughLoopback<1UZ>(kDevice, settings, {values}, kSamples);
        const auto expected = expectedTransmission(values, kSamples, kRampTime, kRate);
        expect(eq(received[0].size(), expected.size()));
        const auto differsAt = firstDifference(received[0], expected);
        expect(!differsAt.has_value()) << std::format("the device received another sample at {}", differsAt.value_or(0UZ));
    };

    "both channels reach the device whole and in order, tapered alike, with the taper on and off"_test = [&taperOn] {
        const std::array patterns{placeMarkedPattern(997UZ), placeMarkedPattern(1009UZ)};
        for (const bool tapered : {false, true}) {
            property_map settings = tapered ? taperOn : property_map{};
            settings.insert_or_assign(std::pmr::string("sample_rate"), kRate);
            settings.insert_or_assign(std::pmr::string("frequency"), std::vector{100e3, 200e3});

            const auto received = transmitThroughLoopback<2UZ>("num_channels=2," + kDevice, settings, patterns, kSamples);
            for (std::size_t ch = 0UZ; ch < patterns.size(); ++ch) {
                const auto expected = expectedTransmission(patterns[ch], kSamples, tapered ? std::optional{kRampTime} : std::nullopt, kRate);
                expect(eq(received[ch].size(), expected.size())) << std::format("channel {}, taper {}", ch, tapered);
                const auto differsAt = firstDifference(received[ch], expected);
                expect(!differsAt.has_value()) << std::format("channel {}, taper {}: the device received another sample at {}", ch, tapered, differsAt.value_or(0UZ));
            }
        }
    };
};

const boost::ut::suite<"SoapySink + SoapySource shared device"> txRxTests = [] {
    using namespace gr;
    using namespace gr::blocks::sdr;
    using namespace gr::blocks::testing;
    using Sched = gr::scheduler::Simple<>;

    "TX→RX round-trip through shared loopback device"_test = [] {
        gr::Graph            flow;
        constexpr gr::Size_t nSamples = 10000;

        auto& txSource = flow.emplaceBlock<gr::blocks::basic::ClockSource<CF32>>({
            {"n_samples_max", nSamples},
            {"sample_rate", 1e6f},
            {"chunk_size", gr::Size_t{1024}},
        });
        auto& sink     = flow.emplaceBlock<SoapySink<CF32, 1UZ>>({
            {"device", "loopback"},
            {"sample_rate", 1e6f},
            {"frequency", std::vector{100e3}},
            {"tx_gains", std::vector{0.}},
        });
        auto& source   = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({
            {"device", "loopback"},
            {"sample_rate", 1e6f},
            {"frequency", std::vector{100e3}},
            {"rx_gains", std::vector{0.}},
        });
        auto& rxSink   = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", nSamples}});

        expect(flow.connect<"out", "in">(txSource, sink).has_value());
        expect(flow.connect<"out", "in">(source, rxSink).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(runWithWatchdog(sched).has_value());
        expect(eq(rxSink.count.value, nSamples)) << std::format("expected {} samples, got {}", nSamples, rxSink.count.value);
    };

    "shared device handle verified via Device::make()"_test = [] {
        auto dev1 = soapy::Device::make({{"driver", "loopback"}});
        auto dev2 = soapy::Device::make({{"driver", "loopback"}});
        expect(dev1.has_value());
        expect(dev2.has_value());
        expect(eq(dev1->get(), dev2->get())) << "same kwargs should return same device handle";
    };

    "device handle survives after one user resets"_test = [] {
        auto dev1 = soapy::Device::make({{"driver", "loopback"}});
        auto dev2 = soapy::Device::make({{"driver", "loopback"}});
        expect(dev1.has_value());
        expect(dev2.has_value());
        auto* rawPtr = dev1->get();
        dev1->reset();
        expect(eq(dev1->get(), static_cast<SoapySDRDevice*>(nullptr)));
        expect(eq(dev2->get(), rawPtr)) << "second handle should still be valid";
    };

    "SoapySink standalone with txOnly loopback"_test = [] {
        gr::Graph            flow;
        constexpr gr::Size_t nSamples = 5000;

        auto& txSource = flow.emplaceBlock<gr::blocks::basic::ClockSource<CF32>>({
            {"n_samples_max", nSamples},
            {"sample_rate", 1e6f},
            {"chunk_size", gr::Size_t{512}},
        });
        auto& sink     = flow.emplaceBlock<SoapySink<CF32, 1UZ>>({
            {"device", "loopback"},
            {"device_parameter", std::string("device_mode=tx_only")},
            {"sample_rate", 1e6f},
            {"frequency", std::vector{100e3}},
            {"tx_gains", std::vector{0.}},
        });
        expect(flow.connect<"out", "in">(txSource, sink).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(runWithWatchdog(sched).has_value());
    };

    "2-channel TX→RX round-trip"_test = [] {
        gr::Graph            flow;
        constexpr gr::Size_t nSamples = 5000;

        auto& txSrc1  = flow.emplaceBlock<gr::blocks::basic::ClockSource<CF32>>({
            {"n_samples_max", nSamples},
            {"sample_rate", 1e6f},
            {"chunk_size", gr::Size_t{512}},
        });
        auto& txSrc2  = flow.emplaceBlock<gr::blocks::basic::ClockSource<CF32>>({
            {"n_samples_max", nSamples},
            {"sample_rate", 1e6f},
            {"chunk_size", gr::Size_t{512}},
        });
        auto& sink    = flow.emplaceBlock<SoapySink<CF32, 2UZ>>({
            {"device", "loopback"},
            {"device_parameter", std::string("num_channels=2")},
            {"sample_rate", 1e6f},
            {"num_channels", gr::Size_t{2}},
            {"frequency", std::vector{100e3, 200e3}},
            {"tx_gains", std::vector{0., 0.}},
        });
        auto& source  = flow.emplaceBlock<SoapySource<CF32, 2UZ>>({
            {"device", "loopback"},
            {"device_parameter", std::string("num_channels=2")},
            {"sample_rate", 1e6f},
            {"num_channels", gr::Size_t{2}},
            {"frequency", std::vector{100e3, 200e3}},
            {"rx_gains", std::vector{0., 0.}},
        });
        auto& rxSink1 = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", nSamples}});
        auto& rxSink2 = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", nSamples}});

        expect(flow.connect<"out", "in#0">(txSrc1, sink).has_value());
        expect(flow.connect<"out", "in#1">(txSrc2, sink).has_value());
        expect(flow.connect<"out#0", "in">(source, rxSink1).has_value());
        expect(flow.connect<"out#1", "in">(source, rxSink2).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(runWithWatchdog(sched, std::chrono::seconds{20}).has_value());
        constexpr auto kMinExpected = static_cast<gr::Size_t>(nSamples * 0.9);
        expect(ge(rxSink1.count.value, kMinExpected)) << std::format("channel 0: got {}, expected at least {}", rxSink1.count.value, kMinExpected);
        expect(ge(rxSink2.count.value, kMinExpected)) << std::format("channel 1: got {}, expected at least {}", rxSink2.count.value, kMinExpected);
    };

    "2-channel TX with a small staging buffer keeps both channels fed"_test = [] {
        gr::Graph            flow;
        constexpr gr::Size_t nSamples = 20000;

        auto& txSrc1  = flow.emplaceBlock<gr::blocks::basic::ClockSource<CF32>>({{"n_samples_max", nSamples}, {"sample_rate", 1e6f}, {"chunk_size", gr::Size_t{128}}});
        auto& txSrc2  = flow.emplaceBlock<gr::blocks::basic::ClockSource<CF32>>({{"n_samples_max", nSamples}, {"sample_rate", 1e6f}, {"chunk_size", gr::Size_t{128}}});
        auto& sink    = flow.emplaceBlock<SoapySink<CF32, 2UZ>>({
            {"device", "loopback"},
            {"device_parameter", std::string("num_channels=2")},
            {"sample_rate", 1e6f},
            {"num_channels", gr::Size_t{2}},
            {"max_chunk_size", std::uint32_t{512}},
        });
        auto& source  = flow.emplaceBlock<SoapySource<CF32, 2UZ>>({
            {"device", "loopback"},
            {"device_parameter", std::string("num_channels=2")},
            {"sample_rate", 1e6f},
            {"num_channels", gr::Size_t{2}},
        });
        auto& rxSink1 = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", nSamples}});
        auto& rxSink2 = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", nSamples}});

        expect(flow.connect<"out", "in#0">(txSrc1, sink).has_value());
        expect(flow.connect<"out", "in#1">(txSrc2, sink).has_value());
        expect(flow.connect<"out#0", "in">(source, rxSink1).has_value());
        expect(flow.connect<"out#1", "in">(source, rxSink2).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(runWithWatchdog(sched, std::chrono::seconds{20}).has_value());

        // a per-channel accounting error starves one channel while the other keeps flowing
        constexpr auto kMinExpected = static_cast<gr::Size_t>(nSamples * 0.9);
        expect(ge(rxSink1.count.value, kMinExpected)) << std::format("channel 0: got {}", rxSink1.count.value);
        expect(ge(rxSink2.count.value, kMinExpected)) << std::format("channel 1: got {}", rxSink2.count.value);
    };
};

const boost::ut::suite<"LimeSDR hardware"> limeTests = [] {
    using namespace gr;
    using namespace gr::blocks::sdr;
    using namespace gr::blocks::testing;
    using Sched = gr::scheduler::Simple<>;

    auto limeAvailable = [] {
        auto devices = soapy::Device::enumerate({{"driver", "lime"}});
        return !devices.empty();
    };

    auto resetLimeUsb = [] {
#if defined(__linux__)
        auto devices = gr::blocks::common::enumerateUSBDevices(std::array{
            gr::blocks::common::USBDeviceId{0x1D50, 0x6108, "LimeSDR-USB"},
        });
        for (const auto& dev : devices) {
            gr::blocks::common::USBDevice usbDev;
            if (auto r = usbDev.open(dev); r) {
                std::ignore = usbDev.reset();
            }
        }
        if (!devices.empty()) {
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
#endif
    };

    "full-duplex TX+RX on shared LimeSDR device"_test = [&] {
        if (!limeAvailable()) {
            std::println(stderr, "[SKIP] no LimeSDR device found");
            return;
        }
        resetLimeUsb();

        gr::Graph            flow;
        constexpr float      rate     = 1e6f;
        constexpr gr::Size_t nSamples = static_cast<gr::Size_t>(rate * 10);

        auto& txSource = flow.emplaceBlock<gr::blocks::basic::ClockSource<CF32>>({
            {"n_samples_max", nSamples},
            {"sample_rate", rate},
            {"chunk_size", gr::Size_t{4096}},
        });
        auto& sink     = flow.emplaceBlock<SoapySink<CF32, 1UZ>>({
            {"device", "lime"},
            {"sample_rate", rate},
            {"frequency", std::vector{433.92e6}},
            {"tx_antennae", std::vector<std::string>{"BAND1"}},
            {"tx_bandwidths", std::vector{5e6}},
            {"tx_gains", std::vector{20.}},
        });
        auto& source   = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({
            {"device", "lime"},
            {"sample_rate", rate},
            {"frequency", std::vector{433.92e6}},
            {"rx_antennae", std::vector<std::string>{"LNAW"}},
            {"rx_bandwidths", std::vector{5e6}},
            {"rx_gains", std::vector{20.}},
            {"max_overflow_count", gr::Size_t{0}},
            {"verbose_overflow", true},
        });
        auto& rxSink   = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", nSamples}});

        expect(flow.connect<"out", "in">(txSource, sink).has_value());
        expect(flow.connect<"out", "in">(source, rxSink).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        auto ret = runWithWatchdog(sched, std::chrono::seconds{15});
        if (!ret.has_value()) {
            expect(false) << std::format("scheduler error: {}", ret.error());
        }

        std::println("LimeSDR full-duplex: RX received {} of {} samples", rxSink.count.value, nSamples);
        expect(eq(rxSink.count.value, nSamples));
    };

    "2-channel duplex on LimeSDR"_test = [&] {
        if (!limeAvailable()) {
            std::println(stderr, "[SKIP] no LimeSDR device found");
            return;
        }
        resetLimeUsb();

        gr::Graph            flow;
        constexpr float      rate     = 1e6f;
        constexpr gr::Size_t nSamples = static_cast<gr::Size_t>(rate * 10);

        auto& txSrc1  = flow.emplaceBlock<gr::blocks::basic::ClockSource<CF32>>({
            {"n_samples_max", nSamples},
            {"sample_rate", rate},
            {"chunk_size", gr::Size_t{4096}},
        });
        auto& txSrc2  = flow.emplaceBlock<gr::blocks::basic::ClockSource<CF32>>({
            {"n_samples_max", nSamples},
            {"sample_rate", rate},
            {"chunk_size", gr::Size_t{4096}},
        });
        auto& sink    = flow.emplaceBlock<SoapySink<CF32, 2UZ>>({
            {"device", "lime"},
            {"sample_rate", rate},
            {"num_channels", gr::Size_t{2}},
            {"frequency", std::vector{433.92e6, 433.92e6}},
            {"tx_antennae", std::vector<std::string>{"BAND1", "BAND1"}},
            {"tx_bandwidths", std::vector{5e6, 5e6}},
            {"tx_gains", std::vector{20., 20.}},
        });
        auto& source  = flow.emplaceBlock<SoapySource<CF32, 2UZ>>({
            {"device", "lime"},
            {"sample_rate", rate},
            {"num_channels", gr::Size_t{2}},
            {"frequency", std::vector{433.92e6, 433.92e6}},
            {"rx_antennae", std::vector<std::string>{"LNAW", "LNAW"}},
            {"rx_bandwidths", std::vector{5e6, 5e6}},
            {"rx_gains", std::vector{20., 20.}},
            {"max_overflow_count", gr::Size_t{0}},
            {"verbose_overflow", true},
        });
        auto& rxSink1 = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", nSamples}});
        auto& rxSink2 = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", nSamples}});

        expect(flow.connect<"out", "in#0">(txSrc1, sink).has_value());
        expect(flow.connect<"out", "in#1">(txSrc2, sink).has_value());
        expect(flow.connect<"out#0", "in">(source, rxSink1).has_value());
        expect(flow.connect<"out#1", "in">(source, rxSink2).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        auto ret = runWithWatchdog(sched, std::chrono::seconds{20});
        expect(ret.has_value());

        constexpr auto kMinExpected = static_cast<gr::Size_t>(nSamples * 0.9);
        std::println("LimeSDR 2-ch duplex: ch0={}, ch1={} of {} samples (min {})", rxSink1.count.value, rxSink2.count.value, nSamples, kMinExpected);
        expect(ge(rxSink1.count.value, kMinExpected)) << "channel 0";
        expect(ge(rxSink2.count.value, kMinExpected)) << "channel 1";
    };
};

int main() {
    if (!std::getenv("SOAPY_SDR_PLUGIN_PATH")) {
        std::error_code ec;
        auto            exePath = std::filesystem::read_symlink("/proc/self/exe", ec);
        if (ec) {
            std::println(stderr, "[qa_SoapyIntegration] SOAPY_SDR_PLUGIN_PATH not set and /proc/self/exe unreadable ({}) — loopback tests will fail", ec.message());
            return 0;
        }

        auto modulePath = exePath.parent_path() / "soapy_modules";
        if (std::filesystem::exists(modulePath)) {
            setenv("SOAPY_SDR_PLUGIN_PATH", modulePath.c_str(), 0);
        } else {
            std::println(stderr, "[qa_SoapyIntegration] SOAPY_SDR_PLUGIN_PATH not set and {} not found — loopback tests will fail", modulePath.string());
        }
    }
    // SoapySDR loads every module it finds on these paths the first time a device is made or enumerated
    const auto modules = soapy::getSoapySDRModules();
    std::println(stderr, "[qa_SoapyIntegration] {} SoapySDR module(s): {}", modules.size(), gr::join(modules, ", "));
}

const boost::ut::suite<"SoapySink BurstTaper"> taperTests = [] {
    using namespace gr::blocks::sdr;
    namespace loopback = gr::blocks::sdr::loopback;

    "taper shapes TX envelope through loopback"_test = [] {
        constexpr float       kSampleRate = 100'000.f;
        constexpr float       kRampTime   = 0.01f; // 10 ms → 1000 ramp samples
        constexpr std::size_t kRampLen    = static_cast<std::size_t>(kRampTime * kSampleRate);
        constexpr std::size_t kTotalTx    = kRampLen * 3; // ramp-up + flat + margin

        auto device = loopback::DeviceRegistry::findOrCreate(80, {{"driver", "loopback#80"}, {"num_channels", "1"}});

        auto* txStream = device->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, {0});
        auto* rxStream = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, {0});
        device->activateStream(txStream);
        device->activateStream(rxStream);

        gr::algorithm::BurstTaper<float> taper(gr::algorithm::TaperType::RaisedCosine, kRampTime, kSampleRate);
        std::ignore = taper.setTarget(true);

        std::vector<CF32> txBuf(kTotalTx, CF32{1.0f, 0.0f});
        taper.applyInPlace(std::span<float>(reinterpret_cast<float*>(txBuf.data()), kTotalTx));
        // applyInPlace operates on float; for CF32 we need per-sample application
        // redo with manual per-sample taper
        taper.reset();
        std::ignore = taper.setTarget(true);
        for (auto& s : txBuf) {
            s *= taper.processOne();
        }

        const void* txPtr  = txBuf.data();
        int         flags  = 0;
        long long   timeNs = 0;
        int         txRet  = device->writeStream(txStream, &txPtr, kTotalTx, flags, timeNs);
        expect(ge(txRet, 0)) << "writeStream failed";

        std::vector<CF32> rxBuf(kTotalTx);
        void*             rxPtr = rxBuf.data();
        int               rxRet = device->readStream(rxStream, &rxPtr, kTotalTx, flags, timeNs);
        expect(eq(static_cast<std::size_t>(rxRet), kTotalTx)) << "readStream short read";

        // verify ramp-up: first kRampLen samples have increasing magnitude
        float prevMag   = 0.f;
        bool  monotonic = true;
        for (std::size_t i = 1UZ; i < kRampLen; ++i) {
            float mag = std::abs(rxBuf[i]);
            if (mag < prevMag - 1e-6f) {
                monotonic = false;
            }
            prevMag = mag;
        }
        expect(monotonic) << "ramp-up should be monotonically increasing";
        expect(lt(std::abs(rxBuf[0]), 0.1f)) << "first sample should be near zero";
        expect(gt(std::abs(rxBuf[kRampLen - 1]), 0.9f)) << "last ramp sample should be near 1.0";

        // verify flat region: samples after ramp should be ~1.0
        for (std::size_t i = kRampLen; i < kRampLen * 2; ++i) {
            expect(gt(std::abs(rxBuf[i]), 0.99f)) << std::format("flat sample {} should be ~1.0", i);
        }

        device->deactivateStream(txStream);
        device->deactivateStream(rxStream);
    };

    "SoapySink with taper enabled passes samples through loopback"_test = [] {
        using Sched              = gr::scheduler::Simple<>;
        constexpr float kRate    = 1e6f;
        gr::Size_t      nSamples = 50'000;
        gr::Graph       flow;

        auto& clockSrc = flow.emplaceBlock<gr::blocks::basic::ClockSource<CF32>>({
            {"sample_rate", kRate},
            {"n_samples_max", nSamples},
        });
        auto& txSink   = flow.emplaceBlock<SoapySink<CF32, 1UZ>>({
            {"device", "loopback"},
            {"sample_rate", kRate},
            {"frequency", std::vector{107e6}},
            {"burst_taper_enabled", true},
            {"burst_ramp_time", 0.001f},
            {"burst_taper_type", std::string("Linear")},
        });
        auto& source   = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({
            {"device", "loopback"},
            {"sample_rate", kRate},
            {"frequency", std::vector{107e6}},
        });
        auto& rxSink   = flow.emplaceBlock<gr::blocks::testing::CountingSink<CF32>>({{"n_samples_max", nSamples}});

        expect(flow.connect<"out", "in">(clockSrc, txSink).has_value());
        expect(flow.connect<"out", "in">(source, rxSink).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(runWithWatchdog(sched, std::chrono::seconds{10}).has_value());
        expect(ge(rxSink.count.value, nSamples * 4U / 5U)) << std::format("expected ~{} RX samples, got {}", nSamples, rxSink.count);
    };

    "taper disabled passes unmodified signal"_test = [] {
        constexpr std::size_t kN = 64;

        auto  device   = loopback::DeviceRegistry::findOrCreate(82, {{"driver", "loopback#82"}, {"num_channels", "1"}});
        auto* txStream = device->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, {0});
        auto* rxStream = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, {0});
        device->activateStream(txStream);
        device->activateStream(rxStream);

        std::vector<CF32> txBuf(kN, CF32{0.5f, -0.25f});
        const void*       txPtr  = txBuf.data();
        int               flags  = 0;
        long long         timeNs = 0;
        int               txRet  = device->writeStream(txStream, &txPtr, kN, flags, timeNs);
        expect(eq(txRet, static_cast<int>(kN)));

        std::vector<CF32> rxBuf(kN);
        void*             rxPtr = rxBuf.data();
        int               rxRet = device->readStream(rxStream, &rxPtr, kN, flags, timeNs);
        expect(eq(rxRet, static_cast<int>(kN)));

        for (std::size_t i = 0UZ; i < kN; ++i) {
            expect(approx(rxBuf[i].real(), 0.5f, 1e-5f)) << std::format("sample {} real", i);
            expect(approx(rxBuf[i].imag(), -0.25f, 1e-5f)) << std::format("sample {} imag", i);
        }

        device->deactivateStream(txStream);
        device->deactivateStream(rxStream);
    };
};

const boost::ut::suite<"SoapySink shutdown"> shutdownTests = [] {
    using namespace gr::blocks::sdr;
    using Sched = gr::scheduler::Simple<>;

    "a device that accepts nothing cannot hold up stop()"_test = [] {
        constexpr float kRate      = 1e6f;
        constexpr auto  kRunTime   = std::chrono::seconds{1};
        constexpr auto  kStopBound = std::chrono::seconds{10};
        gr::Graph       flow;

        // the loopback routes TX into an RX ring that no source drains, so once the ring is full every
        // write reports a timeout and accepts nothing: the shutdown drain and the safety ramp-down both
        // face a device that never makes progress
        auto& clockSrc = flow.emplaceBlock<gr::blocks::basic::ClockSource<CF32>>({
            {"sample_rate", kRate},
            {"n_samples_max", gr::Size_t{0}},
            {"chunk_size", gr::Size_t{1024}},
        });
        auto& txSink   = flow.emplaceBlock<SoapySink<CF32, 1UZ>>({
            {"device", "loopback"},
            {"device_parameter", std::string("buffer_size=4096")},
            {"sample_rate", kRate},
            {"max_chunk_size", std::uint32_t{1024}},
            {"burst_taper_enabled", true},
            {"burst_ramp_time", 0.01f},
            {"burst_taper_type", std::string("Linear")},
        });
        expect(flow.connect<"out", "in">(clockSrc, txSink).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        const auto start = std::chrono::steady_clock::now();
        expect(runWithWatchdog(sched, kRunTime).has_value());
        const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();

        expect(lt(elapsedMs, std::chrono::milliseconds(kRunTime + kStopBound).count())) << std::format("stop() took {} ms after a {} s run", elapsedMs, kRunTime.count());
        expect(txSink._rampAbandoned.load()) << "the safety ramp-down should give up on a device that accepts nothing";
        expect(gt(txSink._stalledWrites.load(), 0UZ)) << "giving up implies stalled writes were counted";
    };

    "a device that accepts one sample per call still ramps down"_test = [] {
        constexpr float         kRate        = 100e3f;
        constexpr float         kRampTime    = 0.001f;
        constexpr std::uint32_t kTimeOutUs   = 100'000U; // a long timeout buys a short stall budget
        constexpr auto          kRampSamples = static_cast<std::size_t>(kRampTime * kRate);
        gr::Graph               flow;

        auto& clockSrc = flow.emplaceBlock<gr::blocks::basic::ClockSource<CF32>>({
            {"sample_rate", kRate},
            {"n_samples_max", gr::Size_t{2000}},
            {"chunk_size", gr::Size_t{64}},
        });
        auto& txSink   = flow.emplaceBlock<SoapySink<CF32, 1UZ>>({
            {"device", "loopback"},
            {"device_parameter", std::string("device_mode=tx_only,max_write_samples=1")},
            {"sample_rate", kRate},
            {"max_chunk_size", std::uint32_t{64}},
            {"max_time_out_us", kTimeOutUs},
            {"burst_taper_enabled", true},
            {"burst_ramp_time", kRampTime},
            {"burst_taper_type", std::string("Linear")},
        });
        expect(flow.connect<"out", "in">(clockSrc, txSink).has_value());

        // every ramp sample needs its own write, so a budget spent on progressing writes would run out first
        expect(gt(kRampSamples, txSink.stalledWriteBudget())) << std::format("{} ramp samples against a budget of {}", kRampSamples, txSink.stalledWriteBudget());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(runWithWatchdog(sched, std::chrono::seconds{10}).has_value());

        expect(!txSink._rampAbandoned.load()) << "the ramp-down should survive more writes than the stall budget";
        expect(eq(txSink._stalledWrites.load(), 0UZ)) << "a device that always takes a sample never stalls";
    };
};

const boost::ut::suite<"SoapySink underflow"> underflowTests = [] {
    using namespace gr::blocks::sdr;
    using Sched = gr::scheduler::Simple<>;

    // max_underflow_count bounds a run of underflows, not a transmission: a link that starves once an hour would
    // otherwise stop the block on its tenth hour, which is what a cumulative count means at the default of ten.
    "a write the device takes clears the underflow count"_test = [] {
        constexpr float kRate = 100e3f;
        gr::Graph       flow;

        // every second write reports an underflow and takes nothing, so the device starves the transmitter again
        // and again without ever starving it twice in a row. The limit itself is off: what is under test is the
        // count it reads.
        auto& clockSrc = flow.emplaceBlock<gr::blocks::basic::ClockSource<CF32>>({
            {"sample_rate", kRate},
            {"n_samples_max", gr::Size_t{2000}},
            {"chunk_size", gr::Size_t{64}},
        });
        auto& txSink   = flow.emplaceBlock<SoapySink<CF32, 1UZ>>({
            {"device", "loopback"},
            {"device_parameter", std::string("device_mode=tx_only,underflow_every=2")},
            {"sample_rate", kRate},
            {"max_chunk_size", std::uint32_t{64}},
            {"max_underflow_count", gr::Size_t{0}},
        });
        expect(flow.connect<"out", "in">(clockSrc, txSink).has_value());

        Sched sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(runWithWatchdog(sched, std::chrono::seconds{10}).has_value());

        expect(lt(txSink._underflowCount.load(), gr::Size_t{2})) << std::format("{} underflows stand after a run in which a write the device took followed every one of them", txSink._underflowCount.load());
    };

    "a sink that reaches its underflow limit stops"_test = [] {
        withinBound(std::chrono::seconds{5}, "a sink at its underflow limit", [] {
            constexpr float      kRate         = 100e3f;
            constexpr gr::Size_t kMaxUnderflow = 3U;
            gr::Graph            flow;

            // every write reports an underflow and takes nothing, so the limit is reached in as many writes
            auto& txSource = flow.emplaceBlock<gr::blocks::testing::ConstantSource<CF32>>({{"n_samples_max", std::uint64_t{2000}}});
            auto& txSink   = flow.emplaceBlock<SoapySink<CF32, 1UZ>>({
                {"device", "loopback"},
                {"device_parameter", std::string("device_mode=tx_only,underflow_every=1")},
                {"sample_rate", kRate},
                {"max_chunk_size", std::uint32_t{64}},
                {"max_underflow_count", kMaxUnderflow},
            });
            expect(flow.connect<"out", "in">(txSource, txSink).has_value());

            Sched sched;
            expect(sched.exchange(std::move(flow)).has_value());
            // the sink reports the limit as a block error, which the scheduler may still be holding when the run
            // ends: what is under test is that the run ends at all, and that the transmit thread is what ended it
            std::ignore = sched.runAndWait();

            expect(gr::atomic_ref(txSink._ioThreadDone).load_acquire()) << "the transmit thread left the device";
            expect(ge(txSink._underflowCount.load(), kMaxUnderflow)) << std::format("{} underflows stand against a limit of {}", txSink._underflowCount.load(), kMaxUnderflow);
        });
    };
};

const boost::ut::suite<"SoapySource overflow"> overflowTests = [] {
    using namespace gr::blocks::sdr;
    using namespace gr::blocks::testing;
    using Sched = gr::scheduler::Simple<>;

    "a source that reaches its overflow limit stops"_test = [] {
        withinBound(std::chrono::seconds{5}, "a source at its overflow limit", [] {
            constexpr gr::Size_t kMaxOverflow = 3U;
            gr::Graph            flow;

            // every read reports an overflow and delivers nothing, so the limit is reached in as many reads
            auto& source = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({
                {"device", "loopback"},
                {"device_parameter", std::string("device_mode=rx_only,overflow_every=1")},
                {"sample_rate", 1e6f},
                {"frequency", std::vector{100e3}},
                {"rx_gains", std::vector{0.}},
                {"max_overflow_count", kMaxOverflow},
            });
            auto& sink   = flow.emplaceBlock<TagSink<CF32, ProcessFunction::USE_PROCESS_BULK>>({
                {"n_samples_expected", gr::Size_t{0}}, // unlimited — stopped by the end of stream the source publishes
                {"log_tags", false},
                {"log_samples", false},
            });
            expect(flow.connect<"out", "in">(source, sink).has_value());

            Sched sched;
            expect(sched.exchange(std::move(flow)).has_value());
            // the source reports the limit as a block error, which the scheduler may still be holding when the run
            // ends: what is under test is that the run ends at all, and that the receive thread is what ended it
            std::ignore = sched.runAndWait();

            expect(gr::atomic_ref(source._ioThreadDone).load_acquire()) << "the receive thread left the device";
            expect(ge(source._overflowCount.load(), kMaxOverflow)) << std::format("{} overflows stand against a limit of {}", source._overflowCount.load(), kMaxOverflow);
        });
    };
};

const boost::ut::suite<"SoapySource device configuration"> configurationTests = [] {
    using namespace gr;
    using namespace gr::blocks::sdr;
    using namespace gr::blocks::testing;
    using Sched = gr::scheduler::Simple<>;

    // An SDRplay-like frontend: two gain elements that are reductions, an AGC that starts on, and an IF gain
    // the device refuses while that AGC is on.
    constexpr const char* kSdrplayLike = "device_mode=rx_only,gain_elements=IFGR:20:59|RFGR:0:9,agc_default=on,refuse_under_agc=IFGR";

    auto runSource = [](const std::string& driver, const std::string& parameters, property_map extraSettings) {
        auto probe = soapy::Device::make(loopbackKwargs(driver, parameters));
        expect(probe.has_value()) << "the probe must open the device the block will open";
        std::ignore = probe->writeSetting("call_log", "");

        gr::Graph    flow;
        property_map settings{{"device", driver}, {"device_parameter", parameters}, {"sample_rate", 1e6f}, {"frequency", std::vector{100e3}}};
        for (auto& [key, value] : extraSettings) {
            settings.insert_or_assign(key, value);
        }
        auto& source = flow.emplaceBlock<SoapySource<CF32, 1UZ>>(std::move(settings));
        auto& sink   = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", std::uint64_t{4096}}});
        expect(flow.connect<"out", "in">(source, sink).has_value());

        // a subscriber keeps a block's error report a message: a scheduler with none throws it instead
        gr::MsgPortIn fromScheduler;
        Sched         sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(sched.msgOut.connect(fromScheduler).has_value());
        expect(runWithWatchdog(sched).has_value());
        auto calls = callLog(*probe);
        return std::make_pair(std::move(*probe), std::move(calls));
    };

    "a device the caller asks nothing of keeps its own frontend"_test = [&] {
        auto [device, calls] = runSource("loopback", kSdrplayLike, {});
        expect(!sawCall(calls, "setGain(")) << "no gain was asked for";
        expect(!sawCall(calls, "setGainMode(")) << "no AGC state was asked for";
        expect(!sawCall(calls, "setBandwidth(")) << "no bandwidth was asked for";
        expect(!sawCall(calls, "setAntenna(")) << "no antenna was asked for";
        expect(!sawCall(calls, "setFrequencyCorrection(")) << "no correction was asked for";
        expect(sawCall(calls, "setSampleRate(RX,0,")) << "the block always sets the rate it publishes";
        expect(sawCall(calls, "setFrequency(RX,0,RF,")) << "the block always sets the frequency it publishes";
        expect(device.isAutomaticGainControl(SOAPY_SDR_RX, 0)) << "the device kept the AGC it started with";
        expect(approx(device.getGain(SOAPY_SDR_RX, 0, "IFGR"), 20.0, 1e-9)) << "the device kept its own IF gain";
    };

    "gain_mode=false switches the AGC off on a device that starts with it on"_test = [&] {
        auto [device, calls] = runSource("loopback", kSdrplayLike, {{"gain_mode", false}});
        expect(sawCall(calls, "setGainMode(RX,0,false)")) << "a setting the caller gives is applied whatever its value";
        expect(!device.isAutomaticGainControl(SOAPY_SDR_RX, 0));
    };

    // A block restored from a serialized graph is constructed from a map that names every writable
    // setting, so each of those values is the caller's, including the ones that equal a default.
    "a fully serialized load applies the values it carries, defaults included"_test = [&] {
        property_map full;
        {
            gr::Graph   reference;
            auto&       block    = reference.emplaceBlock<SoapySource<CF32, 1UZ>>();
            const auto& writable = block.settings().writableMembers();
            for (const auto& [key, value] : block.settings().get()) {
                if (writable.contains(std::string(key))) {
                    full.insert_or_assign(key, value);
                }
            }
        }
        expect(full.contains("gain_mode") && full.contains("frequency_correction") && full.contains("dc_offset_mode")) << "the map holds every writable setting";
        full.insert_or_assign(std::pmr::string("device"), std::string("loopback"));
        full.insert_or_assign(std::pmr::string("device_parameter"), std::string(kSdrplayLike));
        full.insert_or_assign(std::pmr::string("sample_rate"), 1e6f);
        full.insert_or_assign(std::pmr::string("frequency"), std::vector{100e3});
        full.insert_or_assign(std::pmr::string("gain_mode"), false);
        full.insert_or_assign(std::pmr::string("frequency_correction"), 0.0);
        full.insert_or_assign(std::pmr::string("dc_offset_mode"), false);

        auto [device, calls] = runSource("loopback", kSdrplayLike, full);
        expect(sawCall(calls, "setGainMode(RX,0,false)")) << "an AGC state the map holds is written even where it equals the default";
        expect(!device.isAutomaticGainControl(SOAPY_SDR_RX, 0)) << "the device did not keep the AGC it started with";
        expect(sawCall(calls, "setFrequencyCorrection(RX,0,0)")) << "a zero correction the map holds is written";
        expect(sawCall(calls, "setDCOffsetMode(RX,0,false)")) << "a DC offset mode the map holds is written";
        expect(sawCall(calls, "setSampleRate(RX,0,")) << "the rate and the frequency are written as always";
        expect(sawCall(calls, "setFrequency(RX,0,RF,"));
        expect(!sawCall(calls, "setGain(")) << "a setting whose own value says 'not given' is still not given";
        expect(!sawCall(calls, "setAntenna(")) << "a setting whose own value says 'not given' is still not given";
    };

    "a setting written while the block runs reaches the device"_test = [&] {
        auto probe = soapy::Device::make(loopbackKwargs("loopback", "device_mode=rx_only"));
        expect(probe.has_value());
        std::ignore = probe->writeSetting("call_log", "");
        expect(!probe->isAutomaticGainControl(SOAPY_SDR_RX, 0)) << "the device starts with its AGC off";

        gr::Graph flow;
        auto&     source = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({{"device", "loopback"}, {"device_parameter", std::string("device_mode=rx_only")}, {"sample_rate", 1e6f}, {"frequency", std::vector{100e3}}});
        auto&     sink   = flow.emplaceBlock<CountingSink<CF32>>();
        expect(flow.connect<"out", "in">(source, sink).has_value());

        gr::MsgPortIn fromScheduler;
        Sched         sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(sched.msgOut.connect(fromScheduler).has_value());

        // the AGC state is written only once the device is open, so what reaches it is the change and not
        // the map the block was constructed with, which never named the key
        auto writer = std::jthread([&](std::stop_token stoken) {
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{3};
            const auto waitFor  = [&](std::string_view fragment) {
                while (!stoken.stop_requested() && std::chrono::steady_clock::now() < deadline && !sawCall(callLog(*probe), fragment)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds{5});
                }
            };
            waitFor("setFrequency(RX,0,RF,");
            std::ignore = source.settings().setStaged({{"gain_mode", true}});
            waitFor("setGainMode(RX,0,true)");
            sched.requestStop();
        });

        expect(runWithWatchdog(sched).has_value());
        writer.request_stop();
        expect(sawCall(callLog(*probe), "setGainMode(RX,0,true)")) << "the AGC state the caller wrote while the block ran reached the device";
        expect(probe->isAutomaticGainControl(SOAPY_SDR_RX, 0)) << "the device took it";
    };

    "a per-element gain reaches the element the caller named, and no other"_test = [&] {
        auto [device, calls] = runSource("loopback", kSdrplayLike, {{"gain_mode", false}, {"rx_gain_elements", property_map{{"IFGR", 20.0}}}});
        const auto agc       = callIndex(calls, "setGainMode(RX,0,false)");
        const auto ifGain    = callIndex(calls, "setGain(RX,0,IFGR,20)");
        expect(agc.has_value() && ifGain.has_value()) << "both calls reached the device";
        expect(lt(*agc, *ifGain)) << "the AGC state precedes the gain the device would otherwise refuse";
        expect(!sawCall(calls, "refusedUnderAgc")) << "the device did not refuse the gain";
        expect(!sawCall(calls, "setGain(RX,0,RFGR,")) << "an element the caller did not name is untouched";
        expect(approx(device.getGain(SOAPY_SDR_RX, 0, "IFGR"), 20.0, 1e-9));
        expect(approx(device.getGain(SOAPY_SDR_RX, 0, "RFGR"), 0.0, 1e-9));
    };

    "an overall gain under a starting AGC is what the per-element setting replaces"_test = [&] {
        auto [device, calls] = runSource("loopback", kSdrplayLike, {{"rx_gains", std::vector{10.0}}});
        expect(sawCall(calls, "refusedUnderAgc(IFGR)")) << "the device refuses the IF half while its AGC is on";
        expect(sawCall(calls, "setGain(RX,0,RFGR,")) << "the remainder reaches the other element";
    };

    "a gain element the device does not have is refused"_test = [&] {
        auto [device, calls] = runSource("loopback", kSdrplayLike, {{"gain_mode", false}, {"rx_gain_elements", property_map{{"LNA", 3.0}}}});
        expect(!sawCall(calls, "setGain(")) << "nothing is written when the element is unknown";
    };

    "a gain outside the element's range is refused"_test = [&] {
        auto [device, calls] = runSource("loopback", kSdrplayLike, {{"gain_mode", false}, {"rx_gain_elements", property_map{{"RFGR", 40.0}}}});
        expect(!sawCall(calls, "setGain(RX,0,RFGR,")) << "40 is outside the element's 0..9";
    };

    "a device setting the device does not have is refused"_test = [&] {
        auto [device, calls] = runSource("loopback", kSdrplayLike, {{"device_settings", std::string("bogus_key=1")}});
        expect(!sawCall(calls, "writeSetting(bogus_key")) << "an unknown key is not written";
    };

    "a device setting the device has is written"_test = [&] {
        auto [device, calls] = runSource("loopback", kSdrplayLike, {{"device_settings", std::string("channel_model=passthrough")}});
        expect(sawCall(calls, "writeSetting(channel_model,passthrough)"));
    };

    "tuning names the RF component and leaves the correction alone"_test = [&] {
        const std::string parameters = "device_mode=rx_only,frequency_components=RF|CORR,frequency_step=1000";
        auto [device, calls]         = runSource("loopback", parameters, {{"frequency", std::vector{100'000'500.}}});
        expect(sawCall(calls, "setFrequency(RX,0,RF,")) << "the RF component is named";
        expect(!sawCall(calls, "setFrequency(RX,0,CORR,")) << "the tuning residual is not written as a correction";
        expect(approx(device.getFrequencyCorrection(SOAPY_SDR_RX, 0), 0.0, 1e-9));
    };

    "frequency_correction=0 resets a device that holds one"_test = [&] {
        auto probe = soapy::Device::make(loopbackKwargs("loopback", "device_mode=rx_only"));
        expect(probe.has_value());
        std::ignore = probe->setFrequencyCorrection(SOAPY_SDR_RX, 0, 12.0);

        gr::Graph flow;
        auto&     source = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({{"device", "loopback"}, {"device_parameter", std::string("device_mode=rx_only")}, {"sample_rate", 1e6f}, {"frequency", std::vector{100e3}}, {"frequency_correction", 0.0}});
        auto&     sink   = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", std::uint64_t{4096}}});
        expect(flow.connect<"out", "in">(source, sink).has_value());

        gr::MsgPortIn fromScheduler;
        Sched         sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(sched.msgOut.connect(fromScheduler).has_value());
        expect(runWithWatchdog(sched).has_value());
        expect(approx(probe->getFrequencyCorrection(SOAPY_SDR_RX, 0), 0.0, 1e-9)) << "a correction the caller set to zero reaches the device";
    };

    "the frontend mapping is set before any per-channel call"_test = [&] {
        auto [device, calls] = runSource("loopback", kSdrplayLike, {{"frontend_mapping", std::string("0:0")}});
        expect(!calls.empty());
        expect(calls.front().starts_with("setFrontendMapping")) << "a mapping decides which physical channel an index names";
    };

    "an antenna the device does not have is refused"_test = [&] {
        auto [device, calls] = runSource("loopback", "device_mode=rx_only,antennas=A|B", {{"rx_antennae", std::vector<std::string>{"Hi-Z"}}});
        expect(!sawCall(calls, "setAntenna(")) << "nothing is written when the antenna is unknown";
        expect(eq(device.getAntenna(SOAPY_SDR_RX, 0), std::string("A"))) << "the device kept the antenna it started with";
    };

    "an antenna the device has is selected"_test = [&] {
        auto [device, calls] = runSource("loopback", "device_mode=rx_only,antennas=A|B", {{"rx_antennae", std::vector<std::string>{"B"}}});
        expect(sawCall(calls, "setAntenna(RX,0,B)"));
        expect(eq(device.getAntenna(SOAPY_SDR_RX, 0), std::string("B")));
    };

    "tune_args reach the tuning call"_test = [&] {
        auto [device, calls] = runSource("loopback", "device_mode=rx_only", {{"tune_args", std::string("CORR=IGNORE")}});
        expect(sawCall(calls, "tuneArg(CORR,IGNORE)")) << "the tuning kwargs the caller gave reach setFrequency";
    };

    "the bandwidth reaches the device only when the caller sets one"_test = [&] {
        auto [withoutDevice, without] = runSource("loopback", "device_mode=rx_only", {});
        expect(!sawCall(without, "setBandwidth("));
        auto [withDevice, with] = runSource("loopback", "device_mode=rx_only", {{"rx_bandwidths", std::vector{200e3}}});
        expect(sawCall(with, "setBandwidth(RX,0,200000)"));
    };
};

// A block that cannot start fails the run, and runAndWait() returns. Every case runs within a bound: a block that
// stops itself inside start() leaves the blocks around it waiting forever. Each graph has a scheduler subscriber,
// which keeps a block's error report a message.
const boost::ut::suite<"Soapy blocks that cannot start"> startFailureTests = [] {
    using namespace gr;
    using namespace gr::blocks::sdr;
    using namespace gr::blocks::testing;
    using Sched = gr::scheduler::Simple<>;
    using gr::lifecycle::State;

    // Another user registers on the device before the graph starts and settles when the trigger sink receives its
    // first tag. The scheduler calls work() only after every block of the graph has started. The block's
    // activation therefore runs inside the trigger sink's work(), off the block's own start().
    auto addLateSettlingUser = [](gr::Graph& flow, soapy::detail::DeviceRegistry::Registration& otherUser, std::atomic<bool>& settled) {
        auto& trigger = flow.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", gr::Size_t{1}}});
        trigger._tags.emplace_back(0UZ, gr::property_map{{"settle", true}});
        auto& settle        = flow.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_BULK>>({{"log_samples", false}});
        settle._tagCallback = [&otherUser, &settled](const gr::Tag&) {
            otherUser.activate({});
            settled.store(true);
        };
        expect(flow.connect<"out", "in">(trigger, settle).has_value());
    };

    "a source whose driver no loaded module provides fails the run"_test = [] {
        withinBound(std::chrono::seconds{5}, "a source without a driver", [] {
            gr::Graph flow;
            auto&     source = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({{"device", "gr4-no-such-driver"}, {"sample_rate", 1e6f}});
            auto&     sink   = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", gr::Size_t{1000}}});
            expect(flow.connect<"out", "in">(source, sink).has_value());

            gr::MsgPortIn fromScheduler;
            Sched         sched;
            expect(sched.exchange(std::move(flow)).has_value());
            expect(sched.msgOut.connect(fromScheduler).has_value());
            const auto result = sched.runAndWait();

            expect(!result.has_value() && result.error().message.contains("device creation failed")) << "the run fails with the reason";
            expect(source.state() == State::ERROR) << "start() threw";
            expect(eq(pendingUsers(loopbackKwargs("gr4-no-such-driver", "")), 0UZ)) << "the start released its place among the device's users";
        });
    };

    "a source that needs more channels than the device has fails the run"_test = [] {
        withinBound(std::chrono::seconds{5}, "a source short of channels", [] {
            const std::string parameters = "device_mode=rx_only,num_channels=1";
            gr::Graph         flow;
            auto&             source = flow.emplaceBlock<SoapySource<CF32, 2UZ>>({{"device", "loopback"}, {"device_parameter", parameters}, {"sample_rate", 1e6f}, {"num_channels", gr::Size_t{2}}});
            auto&             sink0  = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", gr::Size_t{1000}}});
            auto&             sink1  = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", gr::Size_t{1000}}});
            expect(flow.connect<"out#0", "in">(source, sink0).has_value());
            expect(flow.connect<"out#1", "in">(source, sink1).has_value());

            gr::MsgPortIn fromScheduler;
            Sched         sched;
            expect(sched.exchange(std::move(flow)).has_value());
            expect(sched.msgOut.connect(fromScheduler).has_value());
            const auto result = sched.runAndWait();

            expect(!result.has_value() && result.error().message.contains("channel mismatch: need 2 but device has 1")) << "the run fails with the reason";
            expect(source.state() == State::ERROR) << "start() threw";
            expect(source._device.get() == nullptr) << "the start released the device it had opened";
            expect(eq(pendingUsers(loopbackKwargs("loopback", parameters)), 0UZ)) << "the start released its place among the device's users";
        });
    };

    "a source whose device refuses to activate the stream fails the run from start()"_test = [] {
        withinBound(std::chrono::seconds{5}, "a source the device will not start", [] {
            const std::string parameters = "device_mode=rx_only,refuse_activation=true";
            gr::Graph         flow;
            auto&             source = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({{"device", "loopback"}, {"device_parameter", parameters}, {"sample_rate", 1e6f}});
            auto&             sink   = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", gr::Size_t{1000}}});
            expect(flow.connect<"out", "in">(source, sink).has_value());

            gr::MsgPortIn fromScheduler;
            Sched         sched;
            expect(sched.exchange(std::move(flow)).has_value());
            expect(sched.msgOut.connect(fromScheduler).has_value());
            const auto result = sched.runAndWait();

            expect(!result.has_value() && result.error().message.contains("activate(")) << "the run fails with the reason";
            expect(source.state() == State::ERROR) << "the device was the source's alone, so the activation ran inside start(), which threw";
            expect(source._device.get() == nullptr) << "the start released the device it had opened";
        });
    };

    "a source whose device refuses to activate the stream after start() has returned ends the run from work()"_test = [&addLateSettlingUser] {
        withinBound(std::chrono::seconds{5}, "a source the device will not start later", [&addLateSettlingUser] {
            const std::string                           parameters = "device_mode=rx_only,refuse_activation=true";
            soapy::detail::DeviceRegistry::Registration otherUser(loopbackKwargs("loopback", parameters));
            std::atomic<bool>                           settled{false};

            gr::Graph flow;
            auto&     source = flow.emplaceBlock<SoapySource<CF32, 1UZ>>({{"device", "loopback"}, {"device_parameter", parameters}, {"sample_rate", 1e6f}});
            auto&     sink   = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", gr::Size_t{1000}}});
            expect(flow.connect<"out", "in">(source, sink).has_value());
            addLateSettlingUser(flow, otherUser, settled);

            gr::MsgPortIn fromScheduler;
            Sched         sched;
            expect(sched.exchange(std::move(flow)).has_value());
            expect(sched.msgOut.connect(fromScheduler).has_value());
            const auto result = sched.runAndWait();

            expect(settled.load()) << "the other user settled while the graph ran";
            expect(!result.has_value()) << "the run fails";
            expect(sched.state() == State::ERROR) << "a block answered ERROR";
            expect(source.state() != State::ERROR) << "the source's start() returned before the device refused the stream";
            expect(sawError(fromScheduler, "activate(")) << "the reason reached the scheduler's subscriber";
            expect(source._device.get() == nullptr) << "stopping the run released the device";
        });
    };

    "a sink whose driver no loaded module provides fails the run"_test = [] {
        withinBound(std::chrono::seconds{5}, "a sink without a driver", [] {
            gr::Graph flow;
            auto&     source = flow.emplaceBlock<ConstantSource<CF32>>();
            auto&     sink   = flow.emplaceBlock<SoapySink<CF32, 1UZ>>({{"device", "gr4-no-such-driver"}, {"sample_rate", 1e6f}});
            expect(flow.connect<"out", "in">(source, sink).has_value());

            gr::MsgPortIn fromScheduler;
            Sched         sched;
            expect(sched.exchange(std::move(flow)).has_value());
            expect(sched.msgOut.connect(fromScheduler).has_value());
            const auto result = sched.runAndWait();

            expect(!result.has_value() && result.error().message.contains("device creation failed")) << "the run fails with the reason";
            expect(sink.state() == State::ERROR) << "start() threw";
            expect(eq(pendingUsers(loopbackKwargs("gr4-no-such-driver", "")), 0UZ)) << "the start released its place among the device's users";
        });
    };

    "a sink that needs more channels than the device has fails the run"_test = [] {
        withinBound(std::chrono::seconds{5}, "a sink short of channels", [] {
            const std::string parameters = "device_mode=tx_only,num_channels=1";
            gr::Graph         flow;
            auto&             source0 = flow.emplaceBlock<ConstantSource<CF32>>();
            auto&             source1 = flow.emplaceBlock<ConstantSource<CF32>>();
            auto&             sink    = flow.emplaceBlock<SoapySink<CF32, 2UZ>>({{"device", "loopback"}, {"device_parameter", parameters}, {"sample_rate", 1e6f}, {"num_channels", gr::Size_t{2}}});
            expect(flow.connect<"out", "in#0">(source0, sink).has_value());
            expect(flow.connect<"out", "in#1">(source1, sink).has_value());

            gr::MsgPortIn fromScheduler;
            Sched         sched;
            expect(sched.exchange(std::move(flow)).has_value());
            expect(sched.msgOut.connect(fromScheduler).has_value());
            const auto result = sched.runAndWait();

            expect(!result.has_value() && result.error().message.contains("TX channel mismatch: need 2 but device has 1")) << "the run fails with the reason";
            expect(sink.state() == State::ERROR) << "start() threw";
            expect(sink._device.get() == nullptr) << "the start released the device it had opened";
            expect(eq(pendingUsers(loopbackKwargs("loopback", parameters)), 0UZ)) << "the start released its place among the device's users";
        });
    };

    "a sink whose device refuses to activate the stream fails the run from start()"_test = [] {
        withinBound(std::chrono::seconds{5}, "a sink the device will not start", [] {
            const std::string parameters = "device_mode=tx_only,refuse_activation=true";
            gr::Graph         flow;
            auto&             source = flow.emplaceBlock<ConstantSource<CF32>>();
            auto&             sink   = flow.emplaceBlock<SoapySink<CF32, 1UZ>>({{"device", "loopback"}, {"device_parameter", parameters}, {"sample_rate", 1e6f}});
            expect(flow.connect<"out", "in">(source, sink).has_value());

            gr::MsgPortIn fromScheduler;
            Sched         sched;
            expect(sched.exchange(std::move(flow)).has_value());
            expect(sched.msgOut.connect(fromScheduler).has_value());
            const auto result = sched.runAndWait();

            expect(!result.has_value() && result.error().message.contains("activate(")) << "the run fails with the reason";
            expect(sink.state() == State::ERROR) << "the device was the sink's alone, so the activation ran inside start(), which threw";
            expect(sink._device.get() == nullptr) << "the start released the device it had opened";
        });
    };

    "a sink whose device refuses to activate the stream after start() has returned ends the run from work()"_test = [&addLateSettlingUser] {
        withinBound(std::chrono::seconds{5}, "a sink the device will not start later", [&addLateSettlingUser] {
            const std::string                           parameters = "device_mode=tx_only,refuse_activation=true";
            soapy::detail::DeviceRegistry::Registration otherUser(loopbackKwargs("loopback", parameters));
            std::atomic<bool>                           settled{false};

            gr::Graph flow;
            auto&     source = flow.emplaceBlock<ConstantSource<CF32>>();
            auto&     sink   = flow.emplaceBlock<SoapySink<CF32, 1UZ>>({{"device", "loopback"}, {"device_parameter", parameters}, {"sample_rate", 1e6f}});
            expect(flow.connect<"out", "in">(source, sink).has_value());
            addLateSettlingUser(flow, otherUser, settled);

            gr::MsgPortIn fromScheduler;
            Sched         sched;
            expect(sched.exchange(std::move(flow)).has_value());
            expect(sched.msgOut.connect(fromScheduler).has_value());
            const auto result = sched.runAndWait();

            expect(settled.load()) << "the other user settled while the graph ran";
            expect(!result.has_value()) << "the run fails";
            expect(sched.state() == State::ERROR) << "a block answered ERROR";
            expect(sink.state() != State::ERROR) << "the sink's start() returned before the device refused the stream";
            expect(sawError(fromScheduler, "activate(")) << "the reason reached the scheduler's subscriber";
            expect(sink._device.get() == nullptr) << "stopping the run released the device";
        });
    };
};
