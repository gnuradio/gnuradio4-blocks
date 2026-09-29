#include <boost/ut.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <complex>
#include <cstdint>
#include <deque>
#include <expected>
#include <format>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <gnuradio-4.0/BlockRegistration.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/sdr/RTL2832Source.hpp>

// Cases that never open a USB device, apart from qa_RTL2832Source, which opens the first dongle attached. A device index
// past any count a bus can hold is refused after the enumeration, which reads descriptors and opens nothing. The stream
// cases give the source a model of the dongle through setDevice().

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
// to kReads in all. A discard asked to fail closes the model and returns an error, as RTL2832Device does. The model
// counts its opens and every call that reaches it while it is closed.
struct QueuedDongle : gr::blocks::sdr::RTL2832DeviceBase {
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
    std::atomic<bool>        _failNextDiscard{false};
    std::size_t              _opens            = 0UZ;
    std::size_t              _callsWhileClosed = 0UZ;

    [[nodiscard]] bool             isOpen() const override { return _opened; }
    [[nodiscard]] std::string_view deviceName() const override { return _deviceName; }
    Result                         open(std::uint32_t /*deviceIndex*/) override {
        _opened = true;
        ++_opens;
        return {};
    }
    void close() override {
        _opened = false;
        _queue.clear();
    }
    ValueResult setSampleRate(float rate) override {
        reach();
        change();
        return static_cast<double>(rate);
    }
    ValueResult setCenterFrequency(double frequency) override {
        reach();
        change();
        if (auto discarded = discardStream(); !discarded) {
            return std::unexpected(discarded.error());
        }
        return frequency;
    }
    Result setGainMode(bool /*autoGain*/) override {
        reach();
        return {};
    }
    Result setAgcMode(bool /*on*/) override {
        reach();
        return {};
    }
    Result setTunerGain(float /*gainDb*/) override {
        reach();
        return {};
    }
    Result setFreqCorrection(std::int32_t /*ppm*/) override {
        reach();
        change();
        return {};
    }
    Result resetBuffer() override {
        reach();
        return {};
    }
    Result discardStream() override {
        reach();
        if (_failNextDiscard.exchange(false)) {
            close();
            return std::unexpected(std::string("discard failed"));
        }
        _queue.clear();
        return {};
    }

    std::expected<std::size_t, std::string> readBulk(std::uint8_t* dst, std::size_t maxLen) override {
        if (!_opened) {
            ++_callsWhileClosed;
            return std::unexpected(std::string("not open"));
        }
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
    void reach() {
        if (!_opened) {
            ++_callsWhileClosed;
        }
    }
    void change() {
        ++_changes;
        _changedWhileStreaming = _changedWhileStreaming || _reads.load(std::memory_order_relaxed) > 0UZ;
    }
};

struct StreamAroundChange {
    std::vector<std::uint8_t> samples;
    std::vector<gr::Tag>      tags;
    std::size_t               staleReads       = 0UZ;
    std::uint8_t              changes          = 0U;
    std::size_t               opens            = 0UZ;
    std::size_t               callsWhileClosed = 0UZ;
    std::vector<std::string>  errors;
};

// the text of every error message on the port
std::vector<std::string> errorMessages(gr::MsgPortIn& port) {
    std::vector<std::string> errors;
    auto&                    reader   = port.streamReader();
    auto                     messages = reader.get<gr::SpanReleasePolicy::ProcessAll>(reader.available());
    for (const gr::Message& message : messages) {
        if (!message.data.has_value()) {
            errors.push_back(message.data.error().message);
        }
    }
    return errors;
}

// Runs an RTL2832Source<std::uint8_t> on the model until the model has handed out kReadsBeforeChange transfers, applies
// the change, and returns what the sink received once the model has handed out all kReads. The scheduler's messages
// have a reader, so an error the source reports does not end the run.
std::optional<StreamAroundChange> streamAroundChange(gr::property_map change, bool failDiscard = false) {
    using Source = gr::blocks::sdr::RTL2832Source<std::uint8_t>;
    using Sink   = gr::blocks::testing::TagSink<std::uint8_t, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>;

    auto          owned  = std::make_unique<QueuedDongle>();
    QueuedDongle& dongle = *owned;
    gr::Graph     graph;
    auto&         source = graph.emplaceBlock<Source>({{"sample_rate", 2.048e6f}, {"emit_timing_tags", false}, {"polling_period", std::uint32_t{1U}}});
    auto&         sink   = graph.emplaceBlock<Sink>({{"n_samples_expected", static_cast<gr::Size_t>(QueuedDongle::kReads * QueuedDongle::kTransferBytes)}});
    if (!source.setDevice(std::move(owned)).has_value() || !graph.connect<"out", "in">(source, sink).has_value()) {
        return std::nullopt;
    }

    gr::MsgPortIn           fromScheduler;
    gr::scheduler::Simple<> sched;
    if (!sched.exchange(std::move(graph)).has_value() || !sched.msgOut.connect(fromScheduler).has_value()) {
        return std::nullopt;
    }
    auto       changer = std::jthread([&source, &dongle, &change, failDiscard](std::stop_token stoken) {
        const auto deadline = std::chrono::steady_clock::now() + 3s;
        while (!stoken.stop_requested() && std::chrono::steady_clock::now() < deadline && dongle._reads.load(std::memory_order_acquire) < QueuedDongle::kReadsBeforeChange) {
            std::this_thread::sleep_for(1ms);
        }
        dongle._failNextDiscard.store(failDiscard);
        std::ignore = source.settings().setStaged(std::move(change));
    });
    const auto result  = runBounded(sched, 5s);
    changer.request_stop();
    changer.join();
    if (!result.has_value() || !result->has_value()) {
        return std::nullopt;
    }
    return StreamAroundChange{
        .samples          = std::vector<std::uint8_t>(sink._samples.begin(), sink._samples.end()),
        .tags             = sink._tags,
        .staleReads       = dongle._staleReads,
        .changes          = dongle._changes,
        .opens            = dongle._opens,
        .callsWhileClosed = dongle._callsWhileClosed,
        .errors           = errorMessages(fromScheduler),
    };
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

    // a generated registration unit derives the registry key and the alias this way
    "RTL2832Source<uint8_t> is registered under one key, and its default name names no device type"_test = [] {
        using Source                                = gr::blocks::sdr::RTL2832Source<std::uint8_t>;
        constexpr gr::BlockFactory  factory         = [](gr::property_map parameters) -> std::unique_ptr<gr::BlockModel> { return std::make_unique<gr::BlockWrapper<Source>>(std::move(parameters)); };
        const gr::BlockRegistration registration    = gr::makeBlockRegistration<Source, "gr::blocks::sdr::RTL2832Source<uint8_t>">(factory);
        const std::string           kRegisteredName = "gr::blocks::sdr::RTL2832Source<uint8>";
        expect(eq(registration.name, registration.alias)) << "the type name is the alias";
        std::ignore = gr::insertBlockFactory(gr::globalBlockRegistry(), registration); // false where the library registered it first

        std::vector<std::string> keys;
        for (const std::string& key : gr::globalBlockRegistry().keys()) {
            if (key.contains("RTL2832Source<uint8")) {
                keys.push_back(key);
            }
        }
        expect(fatal(eq(keys.size(), 1UZ))) << "one registry key";
        expect(eq(keys.front(), kRegisteredName));

        Source source(gr::property_map{});
        expect(eq(source.name.value, kRegisteredName));
        expect(std::string_view(source.unique_name).starts_with(kRegisteredName + "#")) << std::string_view(source.unique_name);
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

    "a failed discard leaves the device closed, and the source reopens it and streams on"_test = [&expectNothingStaleAfter] {
        const auto stream = streamAroundChange({{"sample_rate", 1.024e6f}}, true);
        expect(fatal(stream.has_value())) << "the run ends by itself";
        expect(eq(stream->opens, 2UZ)) << "the source reopens the device once";
        expect(eq(stream->callsWhileClosed, 0UZ)) << "no call reaches the closed device";
        expect(fatal(eq(stream->errors.size(), 1UZ))) << "one error";
        expect(stream->errors.front().contains("discardStream")) << stream->errors.front();
        const auto tagIndex = taggedAt(stream->tags, "sample_rate", 1.024e6f);
        expect(fatal(tagIndex.has_value())) << "the rate change is tagged";
        expectNothingStaleAfter("failed discard", *stream, *tagIndex);
    };
};

int main() { /* not needed for UT */ }
