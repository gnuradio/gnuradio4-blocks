#include <boost/ut.hpp>

#include <atomic>
#include <chrono>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <future>
#include <memory>
#include <random>
#include <span>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>

#include <gnuradio-4.0/basic/DataSink.hpp>

// The module's example graphs, run as what they describe: two halves of one flowgraph, in two schedulers, talking
// over a socket. They are loaded through the tree's own YAML importer, which is the route a graph editor and a host
// application take, so an example that no longer loads fails here rather than in front of a new user.
//
// The endpoint in the files is a fixed ipc:// path, because a file a person reads has to name a real endpoint. The
// test rewrites it to a path unique to this process before loading, since two ctest runs sharing one socket file
// would be a hazard that has nothing to do with what is under test.

namespace {

using namespace std::chrono_literals;
using Level = float; // what the receiving graph's DataSink hands back: the magnitude of each link sample

constexpr auto kDeliveryTimeout = 10s;

/// @brief The two endpoints written in the example files, and what they are replaced by here.
constexpr std::string_view kStreamEndpoint  = "ipc:///tmp/gr4-stream-link.sock";
constexpr std::string_view kMessageEndpoint = "ipc:///tmp/gr4-message-link.sock";

[[nodiscard]] std::string uniqueEndpoint(std::string_view role) {
    static const std::uint64_t salt = [] {
        std::random_device device;
        return (static_cast<std::uint64_t>(device()) << 32U) | static_cast<std::uint64_t>(device());
    }();
    return std::format("ipc://{}/gr4-spg-{}-{:016x}.sock", std::filesystem::temp_directory_path().string(), role, salt);
}

void substitute(std::string& text, std::string_view from, std::string_view to) {
    for (std::size_t at = text.find(from); at != std::string::npos; at = text.find(from, at)) {
        text.replace(at, from.size(), to);
        at += to.size();
    }
}

[[nodiscard]] std::string readGraph(std::string_view fileName, std::string_view stream, std::string_view messages) {
    std::ifstream      file(std::format("{}/{}", EXAMPLE_GRAPHS_PATH, fileName), std::ios::binary);
    std::ostringstream content;
    content << file.rdbuf();
    std::string text = content.str();
    substitute(text, kStreamEndpoint, stream);
    substitute(text, kMessageEndpoint, messages);
    return text;
}

/// @brief A scheduler running its graph on its own thread, stopped when the test is done with it.
struct Runner {
    gr::scheduler::Simple<>                     scheduler;
    std::future<std::expected<void, gr::Error>> finished;

    explicit Runner(gr::Graph&& graph) {
        boost::ut::expect(scheduler.exchange(std::move(graph)).has_value()) << boost::ut::fatal;
        finished = std::async(std::launch::async, [this] { return scheduler.runAndWait(); });
    }

    Runner(const Runner&)            = delete;
    Runner& operator=(const Runner&) = delete;

    void stop() {
        if (finished.valid()) {
            scheduler.requestStop();
            const auto result = finished.get();
            boost::ut::expect(result.has_value()) << (result.has_value() ? std::string{} : result.error().message);
        }
    }

    ~Runner() { stop(); }
};

} // namespace

const boost::ut::suite<"stream link example graphs"> streamLinkExampleTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::basic;

    // Both halves carry the message pair beside the stream pair, so this also gates the case a unit test cannot
    // reach: two blocks whose only ports are the framework's message ports, inside a running graph.
    "the two example halves load, run in one process and carry the stream and its rate across"_test = [] {
        constexpr std::size_t kWanted = 65536UZ;

        const std::string stream   = uniqueEndpoint("iq");
        const std::string messages = uniqueEndpoint("msg");
        // loadGrc throws when a file no longer loads, which is the failure this case exists to catch first
        auto receiver = gr::loadGrc(gr::globalPluginLoader(), readGraph("stream_link_rx.yaml", stream, messages));
        auto sender   = gr::loadGrc(gr::globalPluginLoader(), readGraph("stream_link_tx.yaml", stream, messages));

        // the receiver first: its `pull` socket connects, and the sender's `push` waits in its mute state for it
        Runner receiving(std::move(*receiver));
        Runner sending(std::move(*sender));

        std::shared_ptr<StreamingPoller<Level>> poller;
        std::vector<Level>                      samples;
        std::vector<gr::Tag>                    tags;

        const auto deadline = std::chrono::steady_clock::now() + kDeliveryTimeout;
        bool       enough   = false;
        while (!enough && std::chrono::steady_clock::now() < deadline) {
            if (poller == nullptr) {
                poller = globalDataSinkRegistry().getStreamingPoller<Level>(DataSinkQuery::signalName("link_level"), {.overflowPolicy = OverflowPolicy::Drop});
                std::this_thread::sleep_for(2ms);
                continue;
            }
            while (poller->process([&samples, &tags](std::span<const Level> data, std::span<const gr::Tag> arrived) {
                for (const gr::Tag& seenTag : arrived) {
                    tags.emplace_back(samples.size() + seenTag.index, seenTag.map);
                }
                samples.insert(samples.end(), data.begin(), data.end());
            })) {
            }
            enough = samples.size() >= kWanted;
            if (!enough) {
                std::this_thread::sleep_for(2ms);
            }
        }
        sending.stop();
        receiving.stop();

        expect(enough) << std::format("{} of {} samples crossed the link", samples.size(), kWanted) << fatal;
        expect(poller != nullptr && poller->droppedSampleCount.load() == 0UZ) << "the poller, not the link, lost samples";

        // The generator's amplitude is 1, so every magnitude that crossed is inside it and the run is not all
        // zeros. A payload cut or spliced somewhere in the link produces neither, which no count of samples shows.
        Level highest = 0.0f;
        for (const Level sample : samples) {
            expect(le(sample, 1.0f + 1e-5f)) << std::format("a magnitude of {:.6f} arrived, past the generator's amplitude", sample);
            highest = std::max(highest, sample);
        }
        expect(ge(highest, 0.5f)) << "every sample that crossed was near zero; the payload did not cross intact";

        // the rate the sender stated crosses as a tag, which is how the receiving half knows what it is holding
        bool rateSeen = false;
        for (const gr::Tag& seenTag : tags) {
            const auto entry = seenTag.map.find(gr::property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()));
            if (entry != seenTag.map.end() && entry->second.get_if<float>() != nullptr) {
                rateSeen = true;
                expect(eq(*entry->second.get_if<float>(), 2000000.0f)) << "the rate arrived, but not the one the sender stated";
            }
        }
        expect(rateSeen) << "no sample_rate tag reached the receiving half";
    };
};

int main() { /* not needed for UT */ }
