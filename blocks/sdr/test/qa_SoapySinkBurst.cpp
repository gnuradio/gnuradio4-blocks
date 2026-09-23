#include <boost/ut.hpp>

#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <complex>
#include <cstdint>
#include <format>
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
 * @brief What SoapySink hands the device at the transmit burst tags `tx_sob`, `tx_eob` and `tx_time`.
 *
 * The sink writes to the loopback device, which records every write with the count asked for, the count taken and
 * the flags and time the write carried. The test opens the device with the block's own arguments, which reaches the
 * same device object, and reads that record through the SoapySDR API. Every case names the loopback driver alone.
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
        Write             write{.first = first};
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

// The source ends the stream, so the run ends by itself; the watchdog only bounds a sink that fails to stop.
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
std::vector<Write> transmit(std::string parameters, std::size_t nSamples, std::vector<gr::Tag> tags, gr::property_map extraSettings = {}) {
    parameters = "device_mode=tx_only" + (parameters.empty() ? std::string() : "," + parameters);
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
    expect(fatal(sched.exchange(std::move(flow)).has_value()));
    expect(runToEnd(sched)) << "the stream ends and the sink stops by itself";
    return writeLog(*probe);
}

[[nodiscard]] std::size_t samplesTaken(const std::vector<Write>& writes) {
    std::size_t total = 0UZ;
    for (const Write& write : writes) {
        total += write.taken;
    }
    return total;
}

// The device sees END_BURST on a write exactly when the samples it asked the device to take end at a burst's last
// sample, and HAS_TIME with the burst's time exactly when they start at a timed burst's first; no write asks for
// samples on both sides of either, every burst's last sample is taken by a write that carries END_BURST, and every
// timed sample by a write that carries its time.
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

    "a burst tag on one channel ends the write of every channel"_test = [] {
        constexpr std::size_t kSamples   = 1000UZ;
        const std::string     parameters = "device_mode=tx_only,num_channels=2";
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
