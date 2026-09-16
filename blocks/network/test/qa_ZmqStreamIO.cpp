#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <mutex>
#include <random>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <zmq.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/network/ZmqStreamIO.hpp>

// The block exists to hear a publisher it did not build, so the assertions below meet it at a socket rather than at
// a span: a raw ZeroMQ publisher in the test sends exactly what a GR 3.10 `zmq_pub_sink` sends, and what the graph
// produces is compared against what was published. Nothing here sleeps to wait for a message; every wait ends on
// the message, on a subscription, or on a deadline that fails the test.

namespace qa_zmq_stream_io {

using namespace std::chrono_literals;
using gr::blocks::network::ZmqStreamSource;

using Sample = std::complex<float>;

constexpr std::uint64_t kBound       = 1ULL << 20U; // max_message_bytes, comfortably past anything published here
constexpr std::size_t   kSampleBytes = sizeof(Sample);

// A fixed ipc:// path is a parallel-ctest hazard and an AF_UNIX path is bounded at about a hundred bytes, so the
// name is short, unique per process and unique per use, and the socket file is removed when the test is done.
[[nodiscard]] std::string uniqueEndpointPath() {
    static const std::uint64_t salt = [] {
        std::random_device device;
        return (static_cast<std::uint64_t>(device()) << 32U) | static_cast<std::uint64_t>(device());
    }();
    static std::atomic<std::uint32_t> counter{0U};
    return std::format("{}/gr4-zsio-{:016x}-{}.sock", std::filesystem::temp_directory_path().string(), salt, counter.fetch_add(1U));
}

struct Endpoint {
    std::string path{uniqueEndpointPath()};
    std::string uri{std::format("ipc://{}", path)};

    Endpoint()                           = default;
    Endpoint(const Endpoint&)            = delete;
    Endpoint& operator=(const Endpoint&) = delete;
    ~Endpoint() { std::filesystem::remove(std::filesystem::path(path)); }
};

/// @brief The publisher this block exists for, in the shape `gr-zeromq` puts on the wire: one message per buffer of
/// items, nothing else in it, and the `key` as a frame of its own where one is set.
///
/// XPUB rather than PUB for one reason: a publisher discards everything it sends before a subscription has reached
/// it, and XPUB delivers that subscription as a message. Waiting for it is what makes "every sample published
/// arrives" an exact assertion here rather than a race against a duration someone guessed.
struct Publisher {
    zmq::context_t context{1};
    zmq::socket_t  socket{context, zmq::socket_type::xpub};

    explicit Publisher(const std::string& uri) {
        socket.set(zmq::sockopt::rcvtimeo, 5000);
        socket.set(zmq::sockopt::sndtimeo, 5000);
        socket.set(zmq::sockopt::linger, 0);
        socket.set(zmq::sockopt::sndhwm, 4096);
        socket.bind(uri);
    }

    Publisher(const Publisher&)            = delete;
    Publisher& operator=(const Publisher&) = delete;

    /// @brief Wait for a subscriber's subscription to reach this socket. False means the deadline won.
    [[nodiscard]] bool awaitSubscriber() {
        zmq::message_t subscription;
        if (!socket.recv(subscription, zmq::recv_flags::none).has_value()) {
            return false;
        }
        // an XPUB reports a subscription as 0x01 followed by the prefix, and a cancellation as 0x00
        return subscription.size() >= 1UZ && *static_cast<const std::uint8_t*>(subscription.data()) == 1U;
    }

    void publish(std::span<const std::byte> payload, std::string_view key = {}) {
        if (!key.empty()) {
            zmq::message_t frame(key.data(), key.size());
            std::ignore = socket.send(frame, zmq::send_flags::sndmore);
        }
        zmq::message_t message(payload.data(), payload.size());
        std::ignore = socket.send(message, zmq::send_flags::none);
    }
};

/// @brief `n` samples counting up from @p first, distinct in both components, so a check can name which samples
/// came back and in what order rather than only how many.
[[nodiscard]] std::vector<Sample> countingRun(std::size_t first, std::size_t n) {
    std::vector<Sample> samples(n);
    for (std::size_t i = 0UZ; i < n; ++i) {
        samples[i] = Sample{static_cast<float>(first + i), -static_cast<float>(first + i)};
    }
    return samples;
}

[[nodiscard]] std::span<const std::byte> asBytes(std::span<const Sample> samples) { return std::as_bytes(samples); }

struct SampleVectorSink : gr::Block<SampleVectorSink> {
    gr::PortIn<Sample> in;
    GR_MAKE_REFLECTABLE(SampleVectorSink, in);

    mutable std::mutex  _mutex;
    std::vector<Sample> _samples{};

    [[nodiscard]] std::size_t count() const {
        std::lock_guard lock(_mutex);
        return _samples.size();
    }

    [[nodiscard]] std::vector<Sample> take() const {
        std::lock_guard lock(_mutex);
        return _samples;
    }

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        {
            std::lock_guard lock(_mutex);
            for (std::size_t i = 0UZ; i < inSpan.size(); ++i) {
                _samples.push_back(inSpan[i]);
            }
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

struct GraphRunner {
    gr::scheduler::Simple<> scheduler;
    std::thread             worker;
    std::atomic<bool>       finished{false};

    explicit GraphRunner(gr::Graph&& graph) {
        boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
        worker = std::thread([this] {
            std::ignore = scheduler.runAndWait();
            finished.store(true);
        });
    }

    GraphRunner(const GraphRunner&)            = delete;
    GraphRunner& operator=(const GraphRunner&) = delete;

    void stop() {
        scheduler.requestStop();
        if (worker.joinable()) {
            worker.join();
        }
    }

    ~GraphRunner() { stop(); }
};

/// @brief Wait until @p ready holds, ending on the condition rather than on a duration. False means the deadline won.
template<typename F>
[[nodiscard]] bool waitFor(F&& ready, std::chrono::milliseconds deadline = 5000ms) {
    const auto until = std::chrono::steady_clock::now() + deadline;
    while (std::chrono::steady_clock::now() < until) {
        if (ready()) {
            return true;
        }
        std::this_thread::yield();
    }
    return ready();
}

} // namespace qa_zmq_stream_io

using namespace qa_zmq_stream_io;

const boost::ut::suite<"ZmqStreamSource"> zmqStreamSourceTests = [] {
    using namespace boost::ut;

    "the settings that gate start() are checked before any socket opens"_test = [] {
        const Endpoint endpoint;
        // a block constructed with no settings at all never passes through settingsChanged, so start() is the gate
        // that has to hold
        const auto refuses = [](gr::property_map settings, std::string_view what) {
            gr::Graph graph;
            auto&     block = graph.emplaceBlock<ZmqStreamSource<Sample>>(std::move(settings));
            expect(throws<gr::exception>([&block] { block.start(); })) << what;
        };
        refuses({{"endpoint", std::string("")}, {"max_message_bytes", kBound}}, "an empty endpoint");
        refuses({{"endpoint", endpoint.uri}, {"max_message_bytes", kBound}, {"pattern", std::string("pub")}}, "a pattern no source runs");
        refuses({{"endpoint", endpoint.uri}}, "a zero message bound");
        refuses({{"endpoint", endpoint.uri}, {"max_message_bytes", kBound}, {"queue_bytes", std::uint64_t{4ULL}}}, "a queue too small for one sample");

        // an endpoint with no transport prefix is the common typo, and the diagnostic names it
        gr::Graph   graph;
        auto&       source = graph.emplaceBlock<ZmqStreamSource<Sample>>({{"endpoint", std::string("127.0.0.1:5555")}, {"max_message_bytes", kBound}});
        std::string message;
        try {
            source.start();
        } catch (const gr::exception& error) {
            message = error.message;
        }
        expect(message.contains("names no transport")) << "the missing-prefix hint is missing: " << message;
    };

    // The property the block exists for: what a stock publisher sends is what the graph gets, sample for sample and
    // in order, over message boundaries the graph never sees.
    "a publisher's buffers cross unchanged"_test = [] {
        constexpr std::size_t kPerMessage = 512UZ;
        constexpr std::size_t kMessages   = 64UZ;

        const Endpoint endpoint;
        Publisher      publisher(endpoint.uri);

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<ZmqStreamSource<Sample>>({{"endpoint", endpoint.uri}, {"max_message_bytes", kBound}});
        auto&     collector = graph.emplaceBlock<SampleVectorSink>();
        expect(graph.connect<"out", "in">(source, collector).has_value());

        GraphRunner runner(std::move(graph));
        expect(publisher.awaitSubscriber()) << "the block never subscribed";

        const std::vector<Sample> sent = countingRun(0UZ, kPerMessage * kMessages);
        for (std::size_t message = 0UZ; message < kMessages; ++message) {
            publisher.publish(asBytes(std::span(sent).subspan(message * kPerMessage, kPerMessage)));
        }
        expect(waitFor([&collector, &sent] { return collector.count() >= sent.size(); })) << std::format("only {} of {} samples crossed", collector.count(), sent.size());
        runner.stop();

        const std::vector<Sample> received = collector.take();
        expect(eq(received.size(), sent.size()));
        expect(std::ranges::equal(received, sent)) << "the stream that arrived is not the stream that was published";

        const auto counted = source.counters();
        expect(eq(counted.messagesReceived, std::uint64_t{kMessages}));
        expect(eq(counted.samplesReceived, std::uint64_t{sent.size()}));
        expect(eq(counted.samplesDropped, std::uint64_t{0ULL})) << "a queue with room to spare shed samples";
        expect(eq(counted.messagesRefused, std::uint64_t{0ULL}));
    };

    // A publisher with a `key` prefixes it as a frame of its own. That frame is the subscription, not the signal.
    "a keyed publisher's key is not samples"_test = [] {
        const Endpoint endpoint;
        Publisher      publisher(endpoint.uri);

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<ZmqStreamSource<Sample>>({{"endpoint", endpoint.uri}, {"max_message_bytes", kBound}, {"topic", std::string("iq")}});
        auto&     collector = graph.emplaceBlock<SampleVectorSink>();
        expect(graph.connect<"out", "in">(source, collector).has_value());

        GraphRunner runner(std::move(graph));
        expect(publisher.awaitSubscriber()) << "the block never subscribed";

        const std::vector<Sample> sent = countingRun(1000UZ, 256UZ);
        publisher.publish(asBytes(sent), "iq");
        expect(waitFor([&collector, &sent] { return collector.count() >= sent.size(); })) << "the keyed message never reached the graph";
        runner.stop();

        expect(std::ranges::equal(collector.take(), sent)) << "the key frame reached the stream as samples";
        expect(eq(source.counters().messagesRefused, std::uint64_t{0ULL}));
    };

    // A payload that is not a whole number of samples cannot be spliced into the stream: doing so would leave every
    // sample after it half a sample out of step, permanently and silently.
    "a ragged message is refused rather than spliced in"_test = [] {
        const Endpoint endpoint;
        Publisher      publisher(endpoint.uri);

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<ZmqStreamSource<Sample>>({{"endpoint", endpoint.uri}, {"max_message_bytes", kBound}});
        auto&     collector = graph.emplaceBlock<SampleVectorSink>();
        expect(graph.connect<"out", "in">(source, collector).has_value());

        GraphRunner runner(std::move(graph));
        expect(publisher.awaitSubscriber()) << "the block never subscribed";

        const std::array<std::byte, 5> ragged{std::byte{0x01}, std::byte{0x02}, std::byte{0x03}, std::byte{0x04}, std::byte{0x05}};
        publisher.publish(ragged);
        expect(waitFor([&source] { return source.counters().messagesRefused >= 1ULL; })) << "five bytes were taken as samples";

        const std::vector<Sample> sent = countingRun(0UZ, 256UZ);
        publisher.publish(asBytes(sent));
        expect(waitFor([&collector, &sent] { return collector.count() >= sent.size(); })) << "the stream after the refusal never reached the graph";
        runner.stop();

        expect(std::ranges::equal(collector.take(), sent)) << "the refused bytes shifted the stream that followed them";
        expect(eq(source.counters().samplesReceived, std::uint64_t{sent.size()})) << "the refused message was counted as samples";
    };

    // A publisher outpacing the graph is the case the queue is bounded for, and which end it sheds is a behavior
    // rather than an implementation detail: the newest samples are the ones still worth having.
    "a publisher that outpaces the graph loses its oldest samples"_test = [] {
        constexpr std::size_t kQueueSamples = 64UZ;
        constexpr std::size_t kPerMessage   = 128UZ;
        constexpr std::size_t kMessages     = 8UZ;

        const Endpoint endpoint;
        Publisher      publisher(endpoint.uri);

        // driven without a graph, so nothing drains the queue and the overflow rule is the only thing acting on it
        ZmqStreamSource<Sample> source({{"endpoint", endpoint.uri}, {"max_message_bytes", kBound}, {"queue_bytes", std::uint64_t{kQueueSamples * kSampleBytes}}});
        source.settings().init();
        std::ignore = source.settings().applyStagedParameters();
        source.start();
        expect(publisher.awaitSubscriber()) << "the block never subscribed";

        const std::vector<Sample> sent = countingRun(0UZ, kPerMessage * kMessages);
        for (std::size_t message = 0UZ; message < kMessages; ++message) {
            publisher.publish(asBytes(std::span(sent).subspan(message * kPerMessage, kPerMessage)));
        }
        expect(waitFor([&source, &sent] { return source.counters().samplesReceived >= sent.size(); })) << "the source did not read everything the publisher sent";

        const auto          counted = source.counters();
        std::vector<Sample> survivors(kQueueSamples);
        {
            // read where they are held: with no graph there is no processBulk to take them out
            std::lock_guard lock(source._mutex);
            expect(eq(source._ring.size(), kQueueSamples * kSampleBytes)) << "the queue holds its capacity and no more";
            source._ring.peek(reinterpret_cast<std::byte*>(survivors.data()), survivors.size() * kSampleBytes);
        }
        source.stop();

        expect(eq(counted.samplesDropped, std::uint64_t{sent.size() - kQueueSamples})) << "each sample past the bound displaced one";
        expect(std::ranges::equal(survivors, std::span(sent).last(kQueueSamples))) << "what survived is not the newest samples";
        expect(eq(counted.messagesRefused, std::uint64_t{0ULL})) << "a shed sample is not a refused message";
    };
};

int main() { /* not needed for UT */ }
