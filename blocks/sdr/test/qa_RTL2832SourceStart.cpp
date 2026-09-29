#include <boost/ut.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <complex>
#include <cstdint>
#include <deque>
#include <expected>
#include <format>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/sdr/RTL2832Source.hpp>

// Cases that never open a USB device, apart from qa_RTL2832Source, which opens the first dongle attached. A device index
// past any count a bus can hold is refused after the enumeration, which reads descriptors and opens nothing. The stream
// cases give the source a model of the dongle in place of RTL2832Device.

namespace {

using namespace std::chrono_literals;

// the run's result, or nothing when the run was still going after `bound` and had to be stopped
std::optional<std::expected<void, gr::Error>> runBounded(gr::scheduler::Simple<>& sched, std::chrono::milliseconds bound) {
    std::optional<std::expected<void, gr::Error>> result;
    std::atomic<bool>                             ended{false};
    auto                                          schedThread = std::thread([&sched, &result, &ended] {
        result = sched.runAndWait();
        ended.store(true, std::memory_order_release);
    });
    const auto                                    deadline    = std::chrono::steady_clock::now() + bound;
    while (!ended.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(10ms);
    }
    const bool endedInTime = ended.load(std::memory_order_acquire);
    if (!endedInTime) {
        sched.requestStop();
    }
    schedThread.join();
    return endedInTime ? result : std::nullopt;
}

// A model of a dongle read through a queue of transfers. Every sample it hands out holds the number of changes (a tune,
// a sample rate or a correction) made before the sample was taken. As RTL2832Device does, it keeps kQueued transfers
// queued, each filled when it was queued; setCenterFrequency and discardStream drop the queue, and setSampleRate and
// setFreqCorrection leave it. It hands out kReadsBeforeChange transfers, then none until a change arrives, then more up
// to kReads in all.
struct QueuedDongle {
    using Result      = std::expected<void, std::string>;
    using ValueResult = std::expected<double, std::string>;

    static constexpr std::size_t kQueued            = 16UZ;
    static constexpr std::size_t kTransferBytes     = 1024UZ;
    static constexpr std::size_t kReadsBeforeChange = 40UZ;
    static constexpr std::size_t kReads             = 120UZ;

    std::string              _deviceName            = "queued dongle";
    bool                     _opened                = false;
    std::uint8_t             _changes               = 0U;
    bool                     _changedWhileStreaming = false;
    std::deque<std::uint8_t> _queue; // what each queued transfer holds
    std::size_t              _staleReads = 0UZ;
    std::atomic<std::size_t> _reads{0UZ};

    [[nodiscard]] bool isOpen() const { return _opened; }
    Result             open(std::uint32_t /*deviceIndex*/) {
        _opened = true;
        return {};
    }
    void close() {
        _opened = false;
        _queue.clear();
    }
    ValueResult setSampleRate(float rate) {
        change();
        return static_cast<double>(rate);
    }
    ValueResult setCenterFrequency(double frequency) {
        change();
        std::ignore = discardStream();
        return frequency;
    }
    Result setGainMode(bool /*autoGain*/) { return {}; }
    Result setAgcMode(bool /*on*/) { return {}; }
    Result setTunerGain(float /*gainDb*/) { return {}; }
    Result setFreqCorrection(std::int32_t /*ppm*/) {
        change();
        return {};
    }
    Result resetBuffer() { return {}; }
    Result discardStream() {
        _queue.clear();
        return {};
    }

    std::expected<std::size_t, std::string> readBulk(std::uint8_t* dst, std::size_t maxLen) {
        const std::size_t nRead = _reads.load(std::memory_order_relaxed);
        if (nRead >= kReads || (nRead >= kReadsBeforeChange && !_changedWhileStreaming)) {
            return 0UZ;
        }
        if (_queue.empty()) {
            _queue.assign(kQueued, _changes);
        }
        const std::uint8_t held = _queue.front();
        _queue.pop_front();
        _queue.push_back(_changes);
        if (held != _changes) {
            ++_staleReads;
        }
        const std::size_t nBytes = std::min(maxLen, kTransferBytes);
        std::fill_n(dst, nBytes, held);
        _reads.store(nRead + 1UZ, std::memory_order_release);
        return nBytes;
    }

private:
    void change() {
        ++_changes;
        _changedWhileStreaming = _changedWhileStreaming || _reads.load(std::memory_order_relaxed) > 0UZ;
    }
};

struct StreamAroundChange {
    std::vector<std::uint8_t> samples;
    std::vector<gr::Tag>      tags;
    std::size_t               staleReads = 0UZ;
    std::uint8_t              changes    = 0U;
};

// Runs the source on the model until it has handed out kReadsBeforeChange transfers, applies the change, and returns
// what the sink received once the model has handed out all kReads.
std::optional<StreamAroundChange> streamAroundChange(gr::property_map change) {
    using Source = gr::blocks::sdr::RTL2832Source<std::uint8_t, QueuedDongle>;
    using Sink   = gr::blocks::testing::TagSink<std::uint8_t, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>;

    gr::Graph graph;
    auto&     source = graph.emplaceBlock<Source>({{"sample_rate", 2.048e6f}, {"emit_timing_tags", false}, {"polling_period", std::uint32_t{1U}}});
    auto&     sink   = graph.emplaceBlock<Sink>({{"n_samples_expected", static_cast<gr::Size_t>(QueuedDongle::kReads * QueuedDongle::kTransferBytes)}});
    if (!graph.connect<"out", "in">(source, sink).has_value()) {
        return std::nullopt;
    }

    gr::scheduler::Simple<> sched;
    if (!sched.exchange(std::move(graph)).has_value()) {
        return std::nullopt;
    }
    auto       changer = std::jthread([&source, &change](std::stop_token stoken) {
        const auto deadline = std::chrono::steady_clock::now() + 3s;
        while (!stoken.stop_requested() && std::chrono::steady_clock::now() < deadline && source._device._reads.load(std::memory_order_acquire) < QueuedDongle::kReadsBeforeChange) {
            std::this_thread::sleep_for(1ms);
        }
        std::ignore = source.settings().setStaged(std::move(change));
    });
    const auto result  = runBounded(sched, 5s);
    changer.request_stop();
    changer.join();
    if (!result.has_value() || !result->has_value()) {
        return std::nullopt;
    }
    return StreamAroundChange{.samples = std::vector<std::uint8_t>(sink._samples.begin(), sink._samples.end()), .tags = sink._tags, .staleReads = source._device._staleReads, .changes = source._device._changes};
}

// the index of the first tag that carries key with the value, or nothing
template<typename TValue>
std::optional<std::size_t> taggedAt(const std::vector<gr::Tag>& tags, std::string_view key, TValue value) {
    for (const gr::Tag& tag : tags) {
        if (const auto it = tag.map.find(key); it != tag.map.end()) {
            if (const TValue* held = it->second.template get_if<TValue>(); held != nullptr && *held == value) {
                return tag.index;
            }
        }
    }
    return std::nullopt;
}

} // namespace

const boost::ut::suite<"RTL2832Source start"> _rtlStartTests = [] {
    using namespace boost::ut;

    // with a reader on the scheduler's messages, the scheduler forwards an error message and does not end the run on
    // it: the run ends only if the source's start fails it
    "an RTL2832Source whose device cannot be opened fails the run of a host that reads messages"_test = [] {
        gr::Graph graph;
        auto&     source = graph.emplaceBlock<gr::blocks::sdr::RTL2832Source<std::complex<float>>>({{"device_index", std::uint32_t{1000U}}});
        auto&     sink   = graph.emplaceBlock<gr::blocks::testing::TagSink<std::complex<float>, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>();
        expect(graph.connect<"out", "in">(source, sink).has_value());

        gr::MsgPortIn           fromScheduler;
        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value());
        expect(sched.msgOut.connect(fromScheduler).has_value());
        // the io thread retries a refused open every 2 s for a device lost during a run, and in the browser build also for
        // the first open
        const auto result = runBounded(sched, 5s);
        expect(fatal(result.has_value())) << "the run must end by itself, not wait for a device that is not there";
        expect(fatal(!result->has_value())) << "a source that cannot open its device must fail the run";
        expect(result->error().message.contains("open failed")) << result->error().message;
    };
};

const boost::ut::suite<"RTL2832Source stream around a change"> _rtlChangeTests = [] {
    using namespace boost::ut;

    // no sample taken before the change follows the tag that announces it, and the change is the last one the model saw
    auto expectNothingStaleAfter = [](std::string_view what, const StreamAroundChange& stream, std::size_t tagIndex) {
        expect(fatal(lt(tagIndex, stream.samples.size()))) << std::format("{}: samples follow the tag", what);
        const auto stale = std::ranges::find_if(stream.samples.begin() + static_cast<std::ptrdiff_t>(tagIndex), stream.samples.end(), [&stream](std::uint8_t sample) { return sample != stream.changes; });
        expect(stale == stream.samples.end()) << std::format("{}: sample {} after the tag at {} was taken before the change", what, stale - stream.samples.begin(), tagIndex);
        expect(neq(stream.samples.front(), stream.changes)) << std::format("{}: the stream starts before the change", what);
    };

    "a retune drops the samples taken before it"_test = [&expectNothingStaleAfter] {
        const auto stream = streamAroundChange({{"frequency", 101.0e6}});
        expect(fatal(stream.has_value())) << "the run ends by itself";
        expect(eq(stream->staleReads, 0UZ)) << "no read after the retune hands out a transfer queued before it";
        const auto tagIndex = taggedAt(stream->tags, "frequency", 101.0e6);
        expect(fatal(tagIndex.has_value())) << "the retune is tagged";
        expectNothingStaleAfter("retune", *stream, *tagIndex);
    };

    "a sample rate change drops the samples taken before it"_test = [&expectNothingStaleAfter] {
        const auto stream = streamAroundChange({{"sample_rate", 1.024e6f}});
        expect(fatal(stream.has_value())) << "the run ends by itself";
        expect(eq(stream->staleReads, 0UZ)) << "no read after the rate change hands out a transfer queued before it";
        const auto tagIndex = taggedAt(stream->tags, "sample_rate", 1.024e6f);
        expect(fatal(tagIndex.has_value())) << "the rate change is tagged";
        expectNothingStaleAfter("sample rate change", *stream, *tagIndex);
    };

    "a correction change drops the samples taken before it"_test = [] {
        const auto stream = streamAroundChange({{"ppm_correction", std::int32_t{12}}});
        expect(fatal(stream.has_value())) << "the run ends by itself";
        expect(eq(stream->staleReads, 0UZ)) << "no read after the correction change hands out a transfer queued before it";
        expect(std::ranges::contains(stream->samples, stream->changes)) << "samples taken after the change arrive";
    };
};

int main() { /* not needed for UT */ }
