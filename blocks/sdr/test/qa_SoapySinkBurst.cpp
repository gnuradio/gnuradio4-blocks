#include <boost/ut.hpp>

#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <complex>
#include <cstdint>
#include <format>
#include <functional>
#include <iterator>
#include <limits>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/sdr/SoapySink.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

/**
 * @brief The writes SoapySink sends the device at the transmit burst tags `tx_sob`, `tx_eob` and `tx_time`.
 *
 * The sink writes to the loopback device, which records every write with the count asked for, the count taken and
 * the flags and time the write carried. The test opens the device with the block's own arguments and gets the same
 * device object. It reads that record through the SoapySDR API. Every case names the loopback driver alone.
 */

using namespace boost::ut;
using CF32 = std::complex<float>;

namespace {

namespace soapy = gr::blocks::sdr::soapy;

using TaggedSource = gr::blocks::testing::TagSource<CF32, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>;

constexpr std::uint64_t kFirstBurstTimeNs  = 1'720'000'000'000'000'000ULL;
constexpr std::uint64_t kSecondBurstTimeNs = 1'720'000'000'250'000'000ULL;

/// one writeStream call as the device recorded it, placed in the stream by the samples taken before it
struct Write {
    std::size_t first     = 0UZ;
    std::size_t requested = 0UZ;
    std::size_t taken     = 0UZ;
    int         flags     = 0;
    long long   timeNs    = 0LL;

    [[nodiscard]] bool endsBurst() const { return (flags & SOAPY_SDR_END_BURST) != 0; }
    [[nodiscard]] bool hasTime() const { return (flags & SOAPY_SDR_HAS_TIME) != 0; }
};

struct TimedSample {
    std::size_t   index  = 0UZ;
    std::uint64_t timeNs = 0ULL;
};

std::vector<Write> writeLog(const soapy::Device& device) {
    const std::string  log = device.readSetting("write_log");
    std::vector<Write> writes;
    std::size_t        first = 0UZ;
    std::size_t        pos   = 0UZ;
    while (pos < log.size()) {
        const std::size_t end = std::min(log.find(';', pos), log.size());
        Write             write{.first = first, .requested = 0UZ, .taken = 0UZ, .flags = 0, .timeNs = 0LL};
        const char*       cursor = log.data() + pos;
        const char*       last   = log.data() + end;
        cursor                   = std::from_chars(cursor, last, write.requested).ptr + 1;
        cursor                   = std::from_chars(cursor, last, write.taken).ptr + 1;
        cursor                   = std::from_chars(cursor, last, write.flags).ptr + 1;
        std::ignore              = std::from_chars(cursor, last, write.timeNs);
        writes.push_back(write);
        first += write.taken;
        pos = end + 1UZ;
    }
    return writes;
}

gr::Tag burstStart(std::size_t index) { return {index, {{gr::tag::TX_SOB.shortKey(), true}}}; }
gr::Tag timedBurstStart(std::size_t index, std::uint64_t timeNs) { return {index, {{gr::tag::TX_SOB.shortKey(), true}, {gr::tag::TX_TIME.shortKey(), timeNs}}}; }
gr::Tag burstEnd(std::size_t index) { return {index, {{gr::tag::TX_EOB.shortKey(), true}}}; }

// the text of every error message on the port that contains filter
std::vector<std::string> errorReports(gr::MsgPortIn& port, std::string_view filter) {
    std::vector<std::string> reports;
    auto&                    reader   = port.streamReader();
    auto                     messages = reader.get<gr::SpanReleasePolicy::ProcessAll>(reader.available());
    for (const gr::Message& message : messages) {
        if (!message.data.has_value()) {
            continue;
        }
        if (const auto it = message.data->find(std::string_view("error")); it != message.data->end()) {
            if (auto text = it->second.value_or(std::string()); text.contains(filter)) {
                reports.push_back(std::move(text));
            }
        }
    }
    return reports;
}

// The source ends the stream, and the run ends with it. The watchdog bounds a sink that fails to stop.
bool runToEnd(gr::scheduler::Simple<>& sched) {
    std::atomic<bool> stoppedByWatchdog{false};
    auto              watchdog = std::jthread([&sched, &stoppedByWatchdog](std::stop_token stoken) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{10};
        while (std::chrono::steady_clock::now() < deadline && !stoken.stop_requested()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!stoken.stop_requested()) {
            stoppedByWatchdog.store(true);
            sched.requestStop();
        }
    });
    const bool        ran      = sched.runAndWait().has_value();
    watchdog.request_stop();
    watchdog.join();
    return ran && !stoppedByWatchdog.load();
}

// Streams nSamples through a one-port sink to a transmit-only loopback device and returns the writes it received.
// With reports, it also collects the messages that report an ignored burst tag.
std::vector<Write> transmit(std::string parameters, std::size_t nSamples, std::vector<gr::Tag> tags, gr::property_map extraSettings = {}, std::vector<std::string>* reports = nullptr) {
    parameters = "device_mode=tx_only,record_writes=true" + (parameters.empty() ? std::string() : "," + parameters);
    soapy::Kwargs kwargs{{"driver", "loopback"}};
    kwargs.merge(soapy::parseKwargsString(parameters));
    auto probe = soapy::Device::make(kwargs);
    expect(fatal(probe.has_value())) << "the probe must open the device the block will open";
    std::ignore = probe->writeSetting("write_log", "");

    gr::Graph flow;
    auto&     source = flow.emplaceBlock<TaggedSource>({{"n_samples_max", static_cast<gr::Size_t>(nSamples)}, {"mark_tag", false}});
    source._tags     = std::move(tags);

    gr::property_map settings{{"device", std::string("loopback")}, {"device_parameter", parameters}, {"sample_rate", 1e6f}};
    for (auto& [key, value] : extraSettings) {
        settings.insert_or_assign(key, value);
    }
    auto& sink = flow.emplaceBlock<gr::blocks::sdr::SoapySink<CF32, 1UZ>>(std::move(settings));
    expect(fatal(flow.connect<"out", "in">(source, sink).has_value()));

    gr::scheduler::Simple<> sched;
    gr::MsgPortIn           fromScheduler;
    expect(fatal(sched.exchange(std::move(flow)).has_value()));
    if (reports != nullptr) {
        expect(fatal(sched.msgOut.connect(fromScheduler).has_value()));
    }
    expect(runToEnd(sched)) << "the stream ends and the sink stops by itself";
    if (reports != nullptr) {
        *reports = errorReports(fromScheduler, "tx_");
    }
    return writeLog(*probe);
}

[[nodiscard]] std::size_t samplesTaken(const std::vector<Write>& writes) {
    std::size_t total = 0UZ;
    for (const Write& write : writes) {
        total += write.taken;
    }
    return total;
}

/// Sends n samples of one value, and ends its stream once mayEnd holds.
struct GatedSource : gr::Block<GatedSource> {
    gr::PortOut<CF32> out;

    GR_MAKE_REFLECTABLE(GatedSource, out);

    std::size_t           n = 0UZ;
    std::function<bool()> mayEnd;
    std::size_t           _sent = 0UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& output) {
        const std::size_t count = std::min(output.size(), n - _sent);
        std::fill_n(output.begin(), count, CF32{0.5f, 0.f});
        output.publish(count);
        _sent += count;
        if (_sent < n || (mayEnd && !mayEnd())) {
            return gr::work::Status::OK;
        }
        return gr::work::Status::DONE;
    }
};

/// Passes its input through. When its input ends and publishes is set, it publishes tx_eob = true at the end-of-stream
/// index, one past the last sample.
struct EndBurstAtStreamEnd : gr::Block<EndBurstAtStreamEnd> {
    gr::PortIn<CF32>  in;
    gr::PortOut<CF32> out;

    GR_MAKE_REFLECTABLE(EndBurstAtStreamEnd, in, out);

    bool publishes = true;

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& input, gr::OutputSpanLike auto& output) {
        std::ranges::copy(input, output.begin());
        return gr::work::Status::OK;
    }

    [[nodiscard]] gr::work::Status processEpilogue(gr::InputSpanLike auto& /*input*/, gr::OutputSpanLike auto& output) {
        if (publishes && !gr::lifecycle::isShuttingDown(this->state())) {
            output.publishTag(gr::property_map{{gr::tag::TX_EOB.shortKey(), true}}, 0UZ);
        }
        output.publish(0UZ);
        return gr::work::Status::OK;
    }
};

// Streams nSamples on each of nPorts inputs through EndBurstAtStreamEnd to a sink on a transmit-only loopback device,
// and returns the writes the device received. Only the last input carries the tx_eob. Each source ends its stream once
// mayEnd holds for the device. With held, the device takes no sample until the sink has stopped running. With reports,
// it also collects the error messages that mention END_BURST.
template<std::size_t nPorts = 1UZ>
requires(nPorts == 1UZ || nPorts == 2UZ)
std::vector<Write> transmitToStreamEnd(std::size_t nSamples, bool held, std::function<bool(soapy::Device&)> mayEnd = {}, std::vector<std::string>* reports = nullptr) {
    const std::string parameters = std::format("device_mode=tx_only,num_channels={},record_writes=true", nPorts);
    soapy::Kwargs     kwargs{{"driver", "loopback"}};
    kwargs.merge(soapy::parseKwargsString(parameters));
    auto probe = soapy::Device::make(kwargs);
    expect(fatal(probe.has_value())) << "the probe must open the device the block will open";
    std::ignore = probe->writeSetting("write_log", "");
    std::ignore = probe->writeSetting("hold_writes", held ? "true" : "false");

    gr::Graph flow;
    auto&     sink = flow.emplaceBlock<gr::blocks::sdr::SoapySink<CF32, nPorts>>({{"device", std::string("loopback")}, {"device_parameter", parameters}, {"sample_rate", 1e6f}, {"num_channels", static_cast<gr::Size_t>(nPorts)}});
    for (std::size_t port = 0UZ; port < nPorts; ++port) {
        auto& source = flow.emplaceBlock<GatedSource>();
        source.n     = nSamples;
        if (mayEnd) {
            source.mayEnd = [&device = *probe, mayEnd] { return mayEnd(device); };
        }
        auto& relay     = flow.emplaceBlock<EndBurstAtStreamEnd>();
        relay.publishes = port + 1UZ == nPorts;
        expect(fatal(flow.connect<"out", "in">(source, relay).has_value()));
        if constexpr (nPorts == 1UZ) {
            expect(fatal(flow.connect<"out", "in">(relay, sink).has_value()));
        } else {
            const bool connected = port == 0UZ ? flow.connect<"out", "in#0">(relay, sink).has_value() : flow.connect<"out", "in#1">(relay, sink).has_value();
            expect(fatal(connected));
        }
    }

    // a held device takes no sample until the sink stops running, and by then the sink has staged every sample
    auto release = std::jthread([&device = *probe, &sink, held](std::stop_token stoken) {
        if (!held) {
            return;
        }
        while (!stoken.stop_requested() && !gr::lifecycle::isShuttingDown(sink.state())) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        std::ignore = device.writeSetting("hold_writes", "false");
    });

    gr::scheduler::Simple<> sched;
    gr::MsgPortIn           fromScheduler;
    expect(fatal(sched.exchange(std::move(flow)).has_value()));
    if (reports != nullptr) {
        expect(fatal(sched.msgOut.connect(fromScheduler).has_value()));
    }
    expect(runToEnd(sched)) << "the stream ends and the sink stops by itself";
    release.request_stop();
    release.join();
    if (reports != nullptr) {
        *reports = errorReports(fromScheduler, "END_BURST");
    }
    return writeLog(*probe);
}

// The source ends its stream once the device has taken nSamples.
bool deviceTookAll(const soapy::Device& device, std::size_t nSamples) { return samplesTaken(writeLog(device)) >= nSamples; }

// The device took every sample in writes without END_BURST, then one zero sample with END_BURST.
void expectOwedEndBurst(std::string_view scenario, const std::vector<Write>& writes, std::size_t nSamples) {
    expect(eq(samplesTaken(writes), nSamples + 1UZ)) << std::format("{}: the device took every sample once and one sample after them", scenario);
    expect(fatal(!writes.empty()));
    const Write& last = writes.back();
    expect(eq(last.first, nSamples) && eq(last.requested, 1UZ) && eq(last.taken, 1UZ)) << std::format("{}: the last write holds one sample after the last sample", scenario);
    expect(last.endsBurst()) << std::format("{}: the last write carries END_BURST", scenario);
    expect(std::none_of(writes.begin(), std::prev(writes.end()), [](const Write& write) { return write.endsBurst(); })) << std::format("{}: no earlier write ends a burst", scenario);
}

// A write asks the device for a range of samples. It carries END_BURST exactly when the range ends at a burst's last
// sample. It carries HAS_TIME with the burst's time exactly when the range starts at a timed sample. No range reaches
// past a burst's last sample, and no range holds a timed sample after its own first sample. Every burst's last sample
// goes out in a write that carries END_BURST, and every timed sample in a write that carries its time.
void expectBurstWrites(std::string_view scenario, const std::vector<Write>& writes, std::size_t nSamples, const std::vector<std::size_t>& lastSamples, const std::vector<TimedSample>& timedSamples) {
    expect(eq(samplesTaken(writes), nSamples)) << std::format("{}: the device took every sample once", scenario);

    for (const Write& write : writes) {
        const std::size_t requestedEnd = write.first + write.requested;
        const bool        endsAtLast   = std::ranges::contains(lastSamples, requestedEnd - 1UZ);
        const auto        timed        = std::ranges::find(timedSamples, write.first, &TimedSample::index);
        expect(eq(write.flags & ~(SOAPY_SDR_END_BURST | SOAPY_SDR_HAS_TIME), 0)) << std::format("{}: the write at {} carries no other flag", scenario, write.first);
        expect(eq(write.endsBurst(), endsAtLast)) << std::format("{}: the write asking for [{}, {}) carries END_BURST exactly when {} is a burst's last sample", scenario, write.first, requestedEnd, requestedEnd - 1UZ);
        expect(eq(write.hasTime(), timed != timedSamples.end())) << std::format("{}: the write at {} carries HAS_TIME exactly when it starts a timed burst", scenario, write.first);
        const long long expectedTime = timed != timedSamples.end() ? static_cast<long long>(timed->timeNs) : 0LL;
        expect(eq(write.timeNs, expectedTime)) << std::format("{}: the write at {} carries the time of its first sample and no other", scenario, write.first);
        for (const std::size_t last : lastSamples) {
            expect(!(write.first <= last && last + 1UZ < requestedEnd)) << std::format("{}: the write asking for [{}, {}) runs past the burst's last sample {}", scenario, write.first, requestedEnd, last);
        }
        for (const TimedSample& sample : timedSamples) {
            expect(!(write.first < sample.index && sample.index < requestedEnd)) << std::format("{}: the write asking for [{}, {}) runs into the timed sample {}", scenario, write.first, requestedEnd, sample.index);
        }
    }

    for (const std::size_t last : lastSamples) {
        const bool ended = std::ranges::any_of(writes, [last](const Write& write) { return write.endsBurst() && write.taken > 0UZ && write.first + write.taken == last + 1UZ; });
        expect(ended) << std::format("{}: the burst's last sample {} is taken by a write that carries END_BURST", scenario, last);
    }
    for (const TimedSample& sample : timedSamples) {
        const bool started = std::ranges::any_of(writes, [&sample](const Write& write) { return write.hasTime() && write.taken > 0UZ && write.first == sample.index; });
        expect(started) << std::format("{}: the timed sample {} is taken by a write that carries its time", scenario, sample.index);
    }
}

} // namespace

const boost::ut::suite<"SoapySink transmit bursts"> burstTests = [] {
    "a stream without burst tags reaches the device with no flag and no time"_test = [] {
        constexpr std::size_t kSamples = 5000UZ;
        const auto            writes   = transmit("", kSamples, {});
        expectBurstWrites("untagged", writes, kSamples, {}, {});
    };

    "tx_sob asks nothing of the device"_test = [] {
        constexpr std::size_t kSamples = 3000UZ;
        const auto            writes   = transmit("", kSamples, {burstStart(0UZ), burstStart(1000UZ)});
        expectBurstWrites("tx_sob alone", writes, kSamples, {}, {});
    };

    "a burst that ends mid-stream ends at its last sample"_test = [] {
        constexpr std::size_t kSamples = 3000UZ;
        constexpr std::size_t kLast    = 1233UZ;
        const auto            writes   = transmit("", kSamples, {burstStart(0UZ), burstEnd(kLast)});
        expectBurstWrites("one burst", writes, kSamples, {kLast}, {});
    };

    "two bursts in one buffer each end at their own last sample"_test = [] {
        constexpr std::size_t kSamples = 600UZ;
        const auto            writes   = transmit("", kSamples, {burstStart(0UZ), burstEnd(199UZ), burstStart(200UZ), burstEnd(599UZ)});
        expectBurstWrites("two bursts", writes, kSamples, {199UZ, 599UZ}, {});
        expect(eq(writes.back().first + writes.back().taken, kSamples)) << "the stream's last write ends the second burst";
        expect(writes.back().endsBurst());
    };

    "a timed burst starts at its time"_test = [] {
        constexpr std::size_t kSamples = 1500UZ;
        const auto            writes   = transmit("", kSamples, {timedBurstStart(300UZ, kFirstBurstTimeNs), burstEnd(899UZ), timedBurstStart(900UZ, kSecondBurstTimeNs), burstEnd(1499UZ)});
        expectBurstWrites("timed bursts", writes, kSamples, {899UZ, 1499UZ}, {{300UZ, kFirstBurstTimeNs}, {900UZ, kSecondBurstTimeNs}});
    };

    "a tx_time of a signed or narrower integer type times the burst"_test = [] {
        constexpr std::size_t   kSamples = 1200UZ;
        constexpr std::int64_t  kSigned  = 1'720'000'000'000'000'000LL;
        constexpr std::uint32_t kNarrow  = 4'000'000'000U;
        const auto              writes   = transmit("", kSamples, {{300UZ, {{gr::tag::TX_TIME.shortKey(), kSigned}}}, burstEnd(599UZ), {600UZ, {{gr::tag::TX_TIME.shortKey(), kNarrow}}}, burstEnd(1199UZ)});
        expectBurstWrites("integer times", writes, kSamples, {599UZ, 1199UZ}, {{300UZ, static_cast<std::uint64_t>(kSigned)}, {600UZ, std::uint64_t{kNarrow}}});
    };

    "a burst tag of another type is ignored and reported once"_test = [] {
        constexpr std::size_t    kSamples = 1000UZ;
        std::vector<std::string> reports;
        const auto               writes = transmit("", kSamples, {{100UZ, {{gr::tag::TX_EOB.shortKey(), std::int32_t{1}}}}, {300UZ, {{gr::tag::TX_TIME.shortKey(), 1.5}}}, {500UZ, {{gr::tag::TX_TIME.shortKey(), std::int64_t{-1}}}}}, {}, &reports);
        expectBurstWrites("mistyped tags", writes, kSamples, {}, {});
        expect(fatal(eq(reports.size(), 1UZ))) << "one report for the run";
        expect(reports.front().contains("tx_eob") && reports.front().contains("100")) << reports.front();
    };

    "a tx_time above the largest device time is ignored and reported"_test = [] {
        constexpr std::size_t    kSamples  = 1000UZ;
        constexpr auto           kLargest  = static_cast<std::uint64_t>(std::numeric_limits<long long>::max());
        constexpr std::uint64_t  kTooLarge = kLargest + 1U;
        std::vector<std::string> reports;
        const auto               writes = transmit("", kSamples, {{200UZ, {{gr::tag::TX_TIME.shortKey(), kTooLarge}}}, {600UZ, {{gr::tag::TX_TIME.shortKey(), kLargest}}}}, {}, &reports);
        expectBurstWrites("times at and above the largest", writes, kSamples, {}, {{600UZ, kLargest}});
        expect(fatal(eq(reports.size(), 1UZ))) << "one report for the run";
        expect(reports.front().contains("tx_time") && reports.front().contains("200")) << reports.front();
    };

    "a device that takes part of a write still ends the burst at its last sample"_test = [] {
        constexpr std::size_t kSamples = 300UZ;
        const auto            writes   = transmit("max_write_samples=7", kSamples, {burstStart(0UZ), burstEnd(99UZ), timedBurstStart(100UZ, kFirstBurstTimeNs), burstEnd(250UZ)});
        expectBurstWrites("short writes", writes, kSamples, {99UZ, 250UZ}, {{100UZ, kFirstBurstTimeNs}});
        expect(std::ranges::all_of(writes, [](const Write& write) { return write.taken <= 7UZ; })) << "the device took at most seven samples per write";
    };

    "the shutdown ramp-down still follows a transmission that ends without tx_eob"_test = [] {
        constexpr std::size_t kSamples = 2000UZ;
        const auto            writes   = transmit("", kSamples, {}, {{"burst_taper_enabled", true}, {"burst_ramp_time", 0.001f}, {"burst_taper_type", std::string("Linear")}});
        expect(gt(samplesTaken(writes), kSamples)) << "the ramp-down follows the stream's last sample";
        expect(std::ranges::none_of(writes, &Write::endsBurst)) << "no write ends a burst";
    };

    "no ramp-down follows a transmission that ended at tx_eob"_test = [] {
        constexpr std::size_t kSamples = 2000UZ;
        const auto            writes   = transmit("", kSamples, {burstStart(0UZ), burstEnd(kSamples - 1UZ)}, {{"burst_taper_enabled", true}, {"burst_ramp_time", 0.001f}, {"burst_taper_type", std::string("Linear")}});
        expectBurstWrites("ended burst with taper", writes, kSamples, {kSamples - 1UZ}, {});
        expect(fatal(!writes.empty()));
        expect(writes.back().endsBurst()) << "the device's last write is the one that ended the burst";
    };

    "a tx_eob at the end of the stream ends the burst at the last sample"_test = [] {
        constexpr std::size_t kSamples = 3000UZ;
        const auto            writes   = transmitToStreamEnd(kSamples, true);
        expectBurstWrites("tx_eob at the end of the stream", writes, kSamples, {kSamples - 1UZ}, {});
        expect(fatal(!writes.empty()));
        expect(writes.back().taken > 0UZ && writes.back().endsBurst()) << "the write that takes the last sample carries END_BURST";
    };

    "a tx_eob at the end of the stream ends a burst the device already took with one zero sample"_test = [] {
        constexpr std::size_t kSamples = 3000UZ;
        const auto            writes   = transmitToStreamEnd(kSamples, false, [](soapy::Device& device) { return deviceTookAll(device, kSamples); });
        expectOwedEndBurst("already taken", writes, kSamples);
    };

    "a tx_eob at the end of one channel's stream ends the burst on a two-port sink"_test = [] {
        constexpr std::size_t kSamples = 3000UZ;
        const auto            marked   = transmitToStreamEnd<2UZ>(kSamples, true);
        expectBurstWrites("two ports, not yet taken", marked, kSamples, {kSamples - 1UZ}, {});
        expect(fatal(!marked.empty()));
        expect(marked.back().taken > 0UZ && marked.back().endsBurst()) << "the write that takes the last sample carries END_BURST";

        const auto owed = transmitToStreamEnd<2UZ>(kSamples, false, [](soapy::Device& device) { return deviceTookAll(device, kSamples); });
        expectOwedEndBurst("two ports, already taken", owed, kSamples);
    };

    "a zero sample the device does not take is reported once"_test = [] {
        constexpr std::size_t    kSamples = 3000UZ;
        std::vector<std::string> reports;
        // the device takes every sample, then holds every write to the end of the run
        const auto writes = transmitToStreamEnd(
            kSamples, false,
            [](soapy::Device& device) {
                if (!deviceTookAll(device, kSamples)) {
                    return false;
                }
                std::ignore = device.writeSetting("hold_writes", "true");
                return true;
            },
            &reports);
        expect(eq(samplesTaken(writes), kSamples)) << "the device took every sample and no zero sample";
        expect(std::ranges::none_of(writes, &Write::endsBurst)) << "no write ends the burst";
        expect(fatal(eq(reports.size(), 1UZ))) << "one report for the run";
        expect(reports.front().contains(std::format("END_BURST after sample {} was not sent", kSamples - 1UZ))) << reports.front();
    };

    "a tx_eob at the end of an empty stream ends nothing"_test = [] {
        const auto writes = transmitToStreamEnd(0UZ, false);
        expect(writes.empty()) << "the device receives no write";
    };

    "a burst tag on one channel ends the write of every channel"_test = [] {
        constexpr std::size_t kSamples   = 1000UZ;
        const std::string     parameters = "device_mode=tx_only,num_channels=2,record_writes=true";
        soapy::Kwargs         kwargs{{"driver", "loopback"}};
        kwargs.merge(soapy::parseKwargsString(parameters));
        auto probe = soapy::Device::make(kwargs);
        expect(fatal(probe.has_value()));
        std::ignore = probe->writeSetting("write_log", "");

        gr::Graph flow;
        auto&     tagged   = flow.emplaceBlock<TaggedSource>({{"n_samples_max", static_cast<gr::Size_t>(kSamples)}, {"mark_tag", false}});
        auto&     untagged = flow.emplaceBlock<TaggedSource>({{"n_samples_max", static_cast<gr::Size_t>(kSamples)}, {"mark_tag", false}});
        tagged._tags       = {timedBurstStart(0UZ, kFirstBurstTimeNs), burstEnd(499UZ), timedBurstStart(500UZ, kSecondBurstTimeNs), burstEnd(999UZ)};
        auto& sink         = flow.emplaceBlock<gr::blocks::sdr::SoapySink<CF32, 2UZ>>({{"device", std::string("loopback")}, {"device_parameter", parameters}, {"sample_rate", 1e6f}, {"num_channels", gr::Size_t{2}}});
        expect(fatal(flow.connect<"out", "in#0">(tagged, sink).has_value()));
        expect(fatal(flow.connect<"out", "in#1">(untagged, sink).has_value()));

        gr::scheduler::Simple<> sched;
        expect(fatal(sched.exchange(std::move(flow)).has_value()));
        expect(runToEnd(sched)) << "the streams end and the sink stops by itself";
        expectBurstWrites("two channels", writeLog(*probe), kSamples, {499UZ, 999UZ}, {{0UZ, kFirstBurstTimeNs}, {500UZ, kSecondBurstTimeNs}});
    };
};

int main() { /* not needed for UT */ }
