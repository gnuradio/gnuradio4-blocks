#include <boost/ut.hpp>

#include <complex>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <string>
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
        expect(ge(rxSink.count, nSamples * 0.8)) << std::format("expected ~{} RX samples, got {}", nSamples, rxSink.count);
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
        auto& sink   = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", gr::Size_t{4096}}});
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
        auto&     sink   = flow.emplaceBlock<CountingSink<CF32>>({{"n_samples_max", gr::Size_t{4096}}});
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
