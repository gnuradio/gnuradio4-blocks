#include <boost/ut.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <format>
#include <mutex>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <zmq.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/YamlPmt.hpp>

#include <gnuradio-4.0/network/StreamPacketIO.hpp>

// A transport is only proved by a peer it did not build, so the wire-shape assertions below meet the blocks at a
// socket: a raw ZeroMQ socket in the test reads what the sink published and writes what the source has to read,
// including the losses and the malformed messages a peer can produce and the blocks cannot. The end-to-end
// assertions run both blocks in one process over an ipc:// endpoint, which is what a flowgraph cut in two is.
//
// Everything here runs PUSH/PULL rather than PUB/SUB. A publisher discards what it sends before a subscription has
// reached it, so a PUB test either races or needs an XPUB peer to wait on; PUSH blocks in its mute state instead and
// loses nothing, which makes "every sample crossed" an exact assertion. The PUB path is the same code below the
// pattern and is covered by qa_ZmqPacketIO.
//
// Nothing here sleeps to wait for data; every wait ends on the data or on a deadline that fails the test.

namespace qa_stream_packet_io {

using namespace std::chrono_literals;
using gr::blocks::network::StreamPacketSink;
using gr::blocks::network::StreamPacketSource;

using Sample = std::complex<float>;

constexpr std::uint64_t kBound = 1ULL << 20U; // max_message_bytes, comfortably past anything sent here

// ─── endpoints ────────────────────────────────────────────────────────────────────────────────────────────────────
// A fixed ipc:// path is a parallel-ctest hazard and an AF_UNIX path is bounded at about a hundred bytes, so the
// name is short, unique per process and unique per use, and the socket file is removed when the test is done.

[[nodiscard]] std::string uniqueEndpointPath() {
    static const std::uint64_t salt = [] {
        std::random_device device;
        return (static_cast<std::uint64_t>(device()) << 32U) | static_cast<std::uint64_t>(device());
    }();
    static std::atomic<std::uint32_t> counter{0U};
    return std::format("{}/gr4-spio-{:016x}-{}.sock", std::filesystem::temp_directory_path().string(), salt, counter.fetch_add(1U));
}

struct Endpoint {
    std::string path{uniqueEndpointPath()};
    std::string uri{std::format("ipc://{}", path)};

    Endpoint()                           = default;
    Endpoint(const Endpoint&)            = delete;
    Endpoint& operator=(const Endpoint&) = delete;
    ~Endpoint() { std::filesystem::remove(std::filesystem::path(path)); }
};

// ─── a raw ZeroMQ peer, which is what proves the format ───────────────────────────────────────────────────────────

/// @brief One envelope as three frames; the topic frame is added when it is sent.
struct WireEnvelope {
    std::vector<std::uint8_t> header{};
    std::string               metadata{};
    std::vector<std::uint8_t> payload{};
};

/// @brief The envelope this pair puts on the wire, built here from the kernel and the key names alone.
[[nodiscard]] WireEnvelope makeEnvelope(std::span<const Sample> items, std::uint64_t sequence, std::uint64_t position, std::optional<float> rate = std::nullopt, std::span<const std::pair<std::uint64_t, gr::property_map>> tags = {}) {
    gr::property_map map;
    map.insert_or_assign(gr::property_map::key_type("sequence"), gr::pmt::Value(sequence));
    map.insert_or_assign(gr::property_map::key_type("stream_position"), gr::pmt::Value(position));
    if (rate.has_value()) {
        map.insert_or_assign(gr::property_map::key_type("sample_rate"), gr::pmt::Value(*rate));
    }
    if (!tags.empty()) {
        gr::Tensor<gr::pmt::Value> carried;
        for (const auto& [offset, tagMap] : tags) {
            gr::property_map entry;
            entry.insert_or_assign(gr::property_map::key_type("offset"), gr::pmt::Value(offset));
            entry.insert_or_assign(gr::property_map::key_type("map"), gr::pmt::Value(tagMap));
            carried.push_back(gr::pmt::Value(std::move(entry)));
        }
        map.insert_or_assign(gr::property_map::key_type("packet_tags"), gr::pmt::Value(std::move(carried)));
    }

    WireEnvelope envelope;
    envelope.metadata = gr::pmt::yaml::serialize(map);

    gr::network::EnvelopeHeader header;
    header.item_type     = gr::network::kItemTypeCode<Sample>;
    header.item_size     = static_cast<std::uint8_t>(sizeof(Sample));
    header.item_count    = static_cast<std::uint32_t>(items.size());
    header.payload_bytes = static_cast<std::uint32_t>(items.size() * sizeof(Sample));
    header.meta_bytes    = static_cast<std::uint32_t>(envelope.metadata.size());
    const auto encoded   = gr::network::encodeHeader(header);
    envelope.header.assign(encoded.begin(), encoded.end());

    const auto raw = std::as_bytes(items);
    envelope.payload.resize(raw.size());
    if (!raw.empty()) {
        std::memcpy(envelope.payload.data(), raw.data(), raw.size());
    }
    return envelope;
}

struct RawPeer {
    zmq::context_t context{1};
    zmq::socket_t  socket;

    RawPeer(zmq::socket_type type, const std::string& uri, bool bindHere) : socket(context, type) {
        socket.set(zmq::sockopt::rcvtimeo, 5000);
        socket.set(zmq::sockopt::sndtimeo, 5000);
        socket.set(zmq::sockopt::linger, 0);
        socket.set(zmq::sockopt::sndhwm, 4096);
        if (bindHere) {
            socket.bind(uri);
        } else {
            socket.connect(uri);
        }
    }

    RawPeer(const RawPeer&)            = delete;
    RawPeer& operator=(const RawPeer&) = delete;

    /// @brief Send one four-frame message; @p payloadTrim removes bytes from the payload frame alone.
    void send(const WireEnvelope& envelope, std::size_t payloadTrim = 0UZ) {
        const std::size_t bytes = envelope.payload.size() - std::min(payloadTrim, envelope.payload.size());
        std::ignore             = socket.send(zmq::message_t(std::string_view("")), zmq::send_flags::sndmore);
        std::ignore             = socket.send(zmq::message_t(envelope.header.data(), envelope.header.size()), zmq::send_flags::sndmore);
        std::ignore             = socket.send(zmq::message_t(envelope.metadata.data(), envelope.metadata.size()), zmq::send_flags::sndmore);
        std::ignore             = socket.send(zmq::message_t(envelope.payload.data(), bytes), zmq::send_flags::none);
    }

    /// @brief One whole multipart message, or nothing when the receive timeout expired.
    [[nodiscard]] std::optional<std::vector<std::vector<std::uint8_t>>> receive() {
        std::vector<std::vector<std::uint8_t>> frames;
        zmq::message_t                         part;
        if (!socket.recv(part, zmq::recv_flags::none).has_value()) {
            return std::nullopt;
        }
        while (true) {
            const std::uint8_t* begin = static_cast<const std::uint8_t*>(part.data());
            frames.emplace_back(begin, begin + part.size());
            if (socket.get(zmq::sockopt::rcvmore) == 0) {
                break;
            }
            if (!socket.recv(part, zmq::recv_flags::none).has_value()) {
                break;
            }
        }
        return frames;
    }
};

// ─── the graph ends the tests drive ───────────────────────────────────────────────────────────────────────────────

/// @brief A finite run of samples with tags at chosen indices, emitted and then idle so the test owns the teardown.
struct TaggedRunSource : gr::Block<TaggedRunSource, gr::NoTagPropagation> {
    gr::PortOut<Sample> out;

    GR_MAKE_REFLECTABLE(TaggedRunSource, out);

    std::vector<Sample>                                   _samples{};
    std::vector<std::pair<std::size_t, gr::property_map>> _tags{};
    std::size_t                                           _next = 0UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t room = std::min(outSpan.size(), _samples.size() - _next);
        if (room == 0UZ) {
            outSpan.publish(0UZ);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS; // idle rather than done: the test owns the teardown
        }
        std::ranges::copy(std::span(_samples).subspan(_next, room), outSpan.begin());
        for (const auto& [index, map] : _tags) {
            if (index >= _next && index < _next + room) {
                outSpan.publishTag(map, index - _next);
            }
        }
        _next += room;
        outSpan.publish(room);
        return gr::work::Status::OK;
    }
};

/// @brief Everything that arrived, with each tag at the absolute sample index it stood on.
struct TaggedVectorSink : gr::Block<TaggedVectorSink, gr::NoTagPropagation> {
    gr::PortIn<Sample> in;

    GR_MAKE_REFLECTABLE(TaggedVectorSink, in);

    mutable std::mutex                                    _mutex;
    std::vector<Sample>                                   _samples{};
    std::vector<std::pair<std::size_t, gr::property_map>> _tags{};

    [[nodiscard]] std::size_t count() const {
        std::lock_guard lock(_mutex);
        return _samples.size();
    }

    [[nodiscard]] std::vector<Sample> samples() const {
        std::lock_guard lock(_mutex);
        return _samples;
    }

    [[nodiscard]] std::vector<std::pair<std::size_t, gr::property_map>> tags() const {
        std::lock_guard lock(_mutex);
        return _tags;
    }

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        {
            std::lock_guard lock(_mutex);
            for (const auto& [relIndex, tagMapRef] : inSpan.tags()) {
                const std::size_t offset = relIndex < 0 ? 0UZ : static_cast<std::size_t>(relIndex);
                if (offset < inSpan.size()) {
                    _tags.emplace_back(_samples.size() + offset, tagMapRef.get());
                }
            }
            _samples.insert(_samples.end(), inSpan.begin(), inSpan.end());
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

struct GraphRunner {
    gr::scheduler::Simple<> scheduler;
    std::thread             worker;

    explicit GraphRunner(gr::Graph&& graph) {
        boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
        worker = std::thread([this] { std::ignore = scheduler.runAndWait(); });
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

/// @brief `n` samples counting up from @p first, distinct in both components, so a check can name which samples came
/// back and in what order rather than only how many.
[[nodiscard]] std::vector<Sample> countingRun(std::size_t first, std::size_t n) {
    std::vector<Sample> samples(n);
    for (std::size_t i = 0UZ; i < n; ++i) {
        samples[i] = Sample{static_cast<float>(first + i), -static_cast<float>(first + i)};
    }
    return samples;
}

[[nodiscard]] gr::property_map namedTag(std::string name, std::uint64_t ordinal) {
    gr::property_map map;
    map.insert_or_assign(gr::property_map::key_type("trigger_name"), gr::pmt::Value(std::move(name)));
    map.insert_or_assign(gr::property_map::key_type("sequence"), gr::pmt::Value(ordinal));
    return map;
}

[[nodiscard]] const gr::pmt::Value* find(const gr::property_map& map, std::string_view key) {
    const auto entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? nullptr : &entry->second;
}

/// @brief A receiving graph and the collector inside it, kept together because the test reads both.
struct Receiver {
    gr::Graph                   graph{};
    StreamPacketSource<Sample>* source    = nullptr;
    TaggedVectorSink*           collector = nullptr;

    explicit Receiver(const std::string& uri, gr::property_map extra = {}) {
        gr::property_map settings{{"endpoint", uri}, {"max_message_bytes", kBound}, {"pattern", std::string("pull")}};
        for (auto& [key, value] : extra) {
            settings.insert_or_assign(key, value);
        }
        source    = &graph.emplaceBlock<StreamPacketSource<Sample>>(std::move(settings));
        collector = &graph.emplaceBlock<TaggedVectorSink>();
        boost::ut::expect(graph.connect<"out", "in">(*source, *collector).has_value());
    }
};

} // namespace qa_stream_packet_io

using namespace qa_stream_packet_io;

const boost::ut::suite<"StreamPacketSink"> streamPacketSinkTests = [] {
    using namespace boost::ut;

    "the settings that gate start() are checked before any socket opens"_test = [] {
        const Endpoint endpoint;
        const auto     refuses = [](gr::property_map settings, std::string_view what) {
            gr::Graph graph;
            auto&     block = graph.emplaceBlock<StreamPacketSink<Sample>>(std::move(settings));
            expect(throws<gr::exception>([&block] { block.start(); })) << what;
        };
        refuses({{"endpoint", std::string("")}}, "an empty endpoint");
        refuses({{"endpoint", endpoint.uri}, {"pattern", std::string("sub")}}, "a pattern no sink runs");
        refuses({{"endpoint", endpoint.uri}, {"overflow", std::string("wait")}}, "an overflow rule that is neither");
        refuses({{"endpoint", endpoint.uri}, {"max_message_bytes", std::uint64_t{0ULL}}}, "a zero message bound");
    };

    // What the sink puts in the metadata frame is the whole of what makes a stream reassemblable at the far end, so
    // it is read here off the wire rather than through the source that would agree with it by construction.
    "the metadata frame states where the chunk sits, what tags stand in it and how fast it runs"_test = [] {
        constexpr std::size_t kItems = 64UZ;

        const Endpoint endpoint;
        RawPeer        peer(zmq::socket_type::pull, endpoint.uri, false);

        gr::Graph graph;
        auto&     producer = graph.emplaceBlock<TaggedRunSource>();
        auto&     sink     = graph.emplaceBlock<StreamPacketSink<Sample>>({{"endpoint", endpoint.uri}, {"pattern", std::string("push")}, {"max_items", static_cast<gr::Size_t>(kItems)}});
        producer._samples  = countingRun(0UZ, 4UZ * kItems);
        producer._tags     = {{0UZ, gr::property_map{{gr::property_map::key_type("sample_rate"), gr::pmt::Value(48000.0f)}}}, {kItems + 3UZ, namedTag("interior", 7ULL)}};
        expect(graph.connect<"out", "in">(producer, sink).has_value());

        GraphRunner runner(std::move(graph));

        // The packet boundaries are not the test's to predict: the framework cuts the sink's input span at each tag,
        // so a chunk is at most max_items and often fewer. What the wire has to state exactly is where each chunk
        // sits, which is asserted against the item counts the headers themselves carry.
        std::uint64_t                                           expectedPosition = 0ULL;
        std::uint64_t                                           packets          = 0ULL;
        std::vector<std::pair<std::uint64_t, gr::property_map>> carried; // absolute sample index, tag map
        while (expectedPosition < 4ULL * kItems) {
            const auto frames = peer.receive();
            expect(frames.has_value()) << std::format("the stream stopped after {} samples", expectedPosition);
            if (!frames.has_value()) {
                break;
            }
            expect(eq((*frames).size(), 4UZ));
            const auto header = gr::network::decodeHeader((*frames)[1UZ]);
            expect(header.has_value()) << "the header a peer has to read did not decode";
            const auto parsed = gr::pmt::yaml::deserialize(std::string_view(reinterpret_cast<const char*>((*frames)[2UZ].data()), (*frames)[2UZ].size()));
            expect(parsed.has_value()) << "the metadata frame did not parse";
            if (!header.has_value() || !parsed.has_value()) {
                break;
            }

            const gr::pmt::Value* sequence = find(*parsed, "sequence");
            const gr::pmt::Value* position = find(*parsed, "stream_position");
            const gr::pmt::Value* rate     = find(*parsed, "sample_rate");
            expect(sequence != nullptr && sequence->get_if<std::uint64_t>() != nullptr) << "no sequence on the wire";
            expect(position != nullptr && position->get_if<std::uint64_t>() != nullptr) << "no stream_position on the wire";
            expect(rate != nullptr && rate->get_if<float>() != nullptr) << "the rate the stream stated did not cross";
            if (sequence == nullptr || position == nullptr || rate == nullptr) {
                break;
            }
            expect(eq(*sequence->get_if<std::uint64_t>(), packets)) << "the sequence does not count packets";
            expect(eq(*position->get_if<std::uint64_t>(), expectedPosition)) << "stream_position does not count the samples that went before";
            expect(eq(*rate->get_if<float>(), 48000.0f)) << "a packet after the tag that set the rate does not state it";
            expect(le(std::size_t{header->item_count}, kItems)) << "a chunk is larger than max_items";

            if (const gr::pmt::Value* tags = find(*parsed, "packet_tags"); tags != nullptr) {
                const auto* list = tags->get_if<gr::Tensor<gr::pmt::Value>>();
                expect(list != nullptr) << "packet_tags is not a list";
                if (list != nullptr) {
                    for (const auto& element : *list) {
                        const auto* fields = element.get_if<gr::property_map>();
                        expect(fields != nullptr) << "a packet_tags entry is not a map";
                        if (fields == nullptr) {
                            continue;
                        }
                        const gr::pmt::Value* offset = find(*fields, "offset");
                        const gr::pmt::Value* map    = find(*fields, "map");
                        expect(offset != nullptr && offset->get_if<std::uint64_t>() != nullptr) << "a packet_tags entry states no offset";
                        expect(map != nullptr && map->get_if<gr::property_map>() != nullptr) << "a packet_tags entry carries no map";
                        if (offset == nullptr || map == nullptr || offset->get_if<std::uint64_t>() == nullptr || map->get_if<gr::property_map>() == nullptr) {
                            continue;
                        }
                        expect(lt(*offset->get_if<std::uint64_t>(), std::uint64_t{header->item_count})) << "a tag offset points past its own payload";
                        carried.emplace_back(expectedPosition + *offset->get_if<std::uint64_t>(), *map->get_if<gr::property_map>());
                    }
                }
            }
            expectedPosition += header->item_count;
            ++packets;
        }
        runner.stop();

        // the two tags the run put in, at the absolute samples they stood on, reconstructed from stream_position and
        // the offsets alone — which is the arithmetic a peer that did not build this sink has to be able to do
        expect(eq(carried.size(), std::size_t{2UZ})) << "not every tag crossed";
        if (carried.size() != 2UZ) {
            return;
        }
        expect(eq(carried[0UZ].first, std::uint64_t{0ULL})) << "the tag at the stream's first sample moved";
        expect(eq(carried[1UZ].first, kItems + 3UZ)) << "the interior tag did not land on the sample it stood on";
        const gr::pmt::Value* name = find(carried[1UZ].second, "trigger_name");
        expect(name != nullptr && name->get_if<std::pmr::string>() != nullptr) << "the tag's map did not cross";
        if (name != nullptr && name->get_if<std::pmr::string>() != nullptr) {
            expect(std::string_view(*name->get_if<std::pmr::string>()) == std::string_view("interior"));
        }
    };

    // `queue_bytes` is a bound the send queue keeps rather than a size it aims at: a chunk whose envelope is larger
    // than the whole queue is refused, because shedding what is queued cannot make room for it. The hole it leaves
    // is the one a refused chunk always leaves, stated in both counters.
    "a chunk larger than queue_bytes is refused rather than queued"_test = [] {
        constexpr std::uint64_t kQueueBytes = 64ULL; // below even a one-sample envelope, so how the span is cut cannot matter
        constexpr std::size_t   kItems      = 64UZ;

        const Endpoint endpoint;
        gr::Graph      graph;
        auto&          producer = graph.emplaceBlock<TaggedRunSource>();
        producer._samples       = countingRun(0UZ, kItems);
        // push with no peer, so nothing leaves the queue and what it holds is what the test reads
        auto& sink = graph.emplaceBlock<StreamPacketSink<Sample>>({{"endpoint", endpoint.uri}, {"pattern", std::string("push")}, {"max_items", static_cast<gr::Size_t>(kItems)}, {"queue_bytes", kQueueBytes}});
        expect(graph.connect<"out", "in">(producer, sink).has_value());

        GraphRunner runner(std::move(graph));
        expect(waitFor([&sink] { return sink.counters().overQueueBytes >= 1ULL; })) << "the oversize chunk was not refused by the queue";
        const std::uint64_t queued = sink._sender.queuedBytes();
        runner.stop();

        expect(le(queued, kQueueBytes)) << "the send queue held" << queued << "bytes against a bound of" << kQueueBytes;
        const auto counted = sink.counters();
        expect(ge(counted.overQueueBytes, std::uint64_t{1ULL}));
        expect(ge(counted.samplesRejected, std::uint64_t{kItems})) << "the samples the refused chunk held were not counted";
        expect(eq(counted.packetsRejected, std::uint64_t{0ULL})) << "the chunk is inside max_message_bytes; the queue is what refused it";
        expect(eq(counted.packetsSent, std::uint64_t{0ULL})) << "no peer was connected, so nothing can have been sent";
        expect(eq(counted.droppedOnOverflow, std::uint64_t{0ULL})) << "nothing queued was shed to make room for what cannot fit";
    };
};

const boost::ut::suite<"StreamPacketSource"> streamPacketSourceTests = [] {
    using namespace boost::ut;

    "the settings that gate start() are checked before any socket opens"_test = [] {
        const Endpoint endpoint;
        const auto     refuses = [](gr::property_map settings, std::string_view what) {
            gr::Graph graph;
            auto&     block = graph.emplaceBlock<StreamPacketSource<Sample>>(std::move(settings));
            expect(throws<gr::exception>([&block] { block.start(); })) << what;
        };
        refuses({{"endpoint", std::string("")}, {"max_message_bytes", kBound}}, "an empty endpoint");
        refuses({{"endpoint", endpoint.uri}, {"max_message_bytes", kBound}, {"pattern", std::string("pub")}}, "a pattern no source runs");
        refuses({{"endpoint", endpoint.uri}}, "a zero message bound");
    };

    // The property the pair exists for: the stream and the tags standing in it come out the far end unchanged, at the
    // same absolute sample indices they went in at.
    "a tagged stream crosses two processes' worth of blocks unchanged"_test = [] {
        constexpr std::size_t kSamples = 4096UZ;

        const Endpoint endpoint;
        Receiver       receiver(endpoint.uri);
        GraphRunner    receiving(std::move(receiver.graph));

        gr::Graph graph;
        auto&     producer = graph.emplaceBlock<TaggedRunSource>();
        auto&     sink     = graph.emplaceBlock<StreamPacketSink<Sample>>({{"endpoint", endpoint.uri}, {"pattern", std::string("push")}, {"max_items", 512U}});
        producer._samples  = countingRun(0UZ, kSamples);
        producer._tags     = {{0UZ, namedTag("start", 0ULL)}, {513UZ, namedTag("interior", 1ULL)}, {2048UZ, namedTag("middle", 2ULL)}, {kSamples - 1UZ, namedTag("last", 3ULL)}};
        expect(graph.connect<"out", "in">(producer, sink).has_value());
        GraphRunner sending(std::move(graph));

        expect(waitFor([&receiver] { return receiver.collector->count() >= kSamples; })) << std::format("only {} of {} samples crossed", receiver.collector->count(), kSamples);
        sending.stop();
        receiving.stop();

        expect(std::ranges::equal(receiver.collector->samples(), producer._samples)) << "the stream that arrived is not the stream that was sent";

        const auto arrived = receiver.collector->tags();
        expect(eq(arrived.size(), producer._tags.size())) << "not every tag crossed";
        for (std::size_t i = 0UZ; i < std::min(arrived.size(), producer._tags.size()); ++i) {
            expect(eq(arrived[i].first, producer._tags[i].first)) << std::format("tag {} landed at sample {} rather than {}", i, arrived[i].first, producer._tags[i].first);
            const gr::pmt::Value* name = find(arrived[i].second, "trigger_name");
            expect(name != nullptr) << "the tag's map did not cross";
            if (name != nullptr) {
                const auto* text = name->get_if<std::pmr::string>();
                expect(text != nullptr && std::string_view(*text) == std::string_view(*producer._tags[i].second.find(gr::property_map::key_type("trigger_name"))->second.get_if<std::pmr::string>()));
            }
        }
        expect(eq(receiver.source->counters().messagesRefused, std::uint64_t{0ULL}));
        expect(eq(receiver.source->counters().sequenceGaps, std::uint64_t{0ULL})) << "PUSH/PULL lost a packet";
    };

    // The framework hands a block with no forward tag propagation one tag at relative index 0, because it cuts the
    // span at the next tag; where tags stand closer together than the port's minimum request it cannot, and the span
    // carries several at their own offsets. Both are the sink's one code path, and this is the second case.
    "several tags inside one input span keep their offsets"_test = [] {
        constexpr std::size_t kSamples = 2048UZ;

        const Endpoint endpoint;
        Receiver       receiver(endpoint.uri);
        GraphRunner    receiving(std::move(receiver.graph));

        gr::Graph graph;
        auto&     producer = graph.emplaceBlock<TaggedRunSource>();
        auto&     sink     = graph.emplaceBlock<StreamPacketSink<Sample>>({{"endpoint", endpoint.uri}, {"pattern", std::string("push")}});
        // a minimum the tags sit inside, so the framework cannot cut the span at each of them
        sink.in.min_samples = 512UZ;
        producer._samples   = countingRun(0UZ, kSamples);
        producer._tags      = {{16UZ, namedTag("a", 0ULL)}, {32UZ, namedTag("b", 1ULL)}, {48UZ, namedTag("c", 2ULL)}};
        expect(graph.connect<"out", "in">(producer, sink).has_value());
        GraphRunner sending(std::move(graph));

        expect(waitFor([&receiver] { return receiver.collector->count() >= kSamples; })) << "the run never crossed";
        sending.stop();
        receiving.stop();

        const auto arrived = receiver.collector->tags();
        expect(eq(arrived.size(), std::size_t{3UZ})) << "a tag inside a span carrying several was dropped";
        for (std::size_t i = 0UZ; i < std::min(arrived.size(), producer._tags.size()); ++i) {
            expect(eq(arrived[i].first, producer._tags[i].first)) << std::format("tag {} landed at sample {} rather than {}", i, arrived[i].first, producer._tags[i].first);
        }
    };

    // max_items is what makes packet size a number rather than a consequence of buffer sizing, and the receiver has
    // to put the pieces back in order.
    "max_items cuts one span into several packets and they arrive in order"_test = [] {
        constexpr std::size_t kSamples   = 1000UZ;
        constexpr gr::Size_t  kPerPacket = 100U;

        const Endpoint endpoint;
        Receiver       receiver(endpoint.uri);
        GraphRunner    receiving(std::move(receiver.graph));

        gr::Graph graph;
        auto&     producer = graph.emplaceBlock<TaggedRunSource>();
        auto&     sink     = graph.emplaceBlock<StreamPacketSink<Sample>>({{"endpoint", endpoint.uri}, {"pattern", std::string("push")}, {"max_items", kPerPacket}});
        producer._samples  = countingRun(0UZ, kSamples);
        expect(graph.connect<"out", "in">(producer, sink).has_value());
        GraphRunner sending(std::move(graph));

        expect(waitFor([&receiver] { return receiver.collector->count() >= kSamples; })) << "the run never crossed";
        sending.stop();
        receiving.stop();

        expect(std::ranges::equal(receiver.collector->samples(), producer._samples)) << "the pieces were not reassembled in order";
        // at least one packet per max_items, since where the scheduler cut the span the sink cut again; the exact
        // count is the scheduler's business and asserting it would be asserting the scheduler's chunking
        expect(ge(receiver.source->counters().packetsPublished, std::uint64_t{kSamples / kPerPacket})) << "the run crossed in fewer packets than max_items allows";
        expect(eq(sink.counters().packetsSent, receiver.source->counters().packetsPublished)) << "a packet the sink sent did not arrive";
    };

    // The one property that governs the design: a hole in the stream is announced, never spliced. Two of three
    // packets are sent by a peer that simply never sends the middle one, which is what a PUB drop looks like from
    // this end and is exact where a real drop is not.
    "a lost packet becomes one gap tag naming the count and the span"_test = [] {
        constexpr std::size_t kItems = 128UZ;

        const Endpoint endpoint;
        Receiver       receiver(endpoint.uri);
        GraphRunner    receiving(std::move(receiver.graph));
        RawPeer        peer(zmq::socket_type::push, endpoint.uri, true);

        const std::vector<Sample> first = countingRun(0UZ, kItems);
        const std::vector<Sample> third = countingRun(2UZ * kItems, kItems);
        peer.send(makeEnvelope(first, 0ULL, 0ULL, 48000.0f));
        peer.send(makeEnvelope(third, 2ULL, 2ULL * kItems, 48000.0f));

        expect(waitFor([&receiver] { return receiver.collector->count() >= 2UZ * kItems; })) << "the two packets that were sent never both arrived";
        receiving.stop();

        std::vector<Sample> expected = first;
        expected.insert(expected.end(), third.begin(), third.end());
        expect(std::ranges::equal(receiver.collector->samples(), expected)) << "a sample outside the gap was lost or duplicated";

        const auto counted = receiver.source->counters();
        expect(eq(counted.sequenceGaps, std::uint64_t{1ULL})) << "the gap was not detected";
        expect(eq(counted.packetsLost, std::uint64_t{1ULL}));
        expect(eq(counted.samplesLost, std::uint64_t{kItems}));

        const auto arrived = receiver.collector->tags();
        // one tag at the stream's first sample for the rate, and exactly one gap tag at the first sample after the hole
        std::vector<std::pair<std::size_t, gr::property_map>> gaps;
        for (const auto& seenTag : arrived) {
            if (find(seenTag.second, "n_dropped_samples") != nullptr) {
                gaps.push_back(seenTag);
            }
        }
        expect(eq(gaps.size(), std::size_t{1UZ})) << "a gap is announced exactly once";
        if (gaps.size() != 1UZ) {
            return;
        }
        expect(eq(gaps[0UZ].first, kItems)) << "the gap tag does not stand at the first sample after the hole";
        const gr::pmt::Value* dropped  = find(gaps[0UZ].second, "n_dropped_samples");
        const gr::pmt::Value* lost     = find(gaps[0UZ].second, "packets_lost");
        const gr::pmt::Value* position = find(gaps[0UZ].second, "stream_position");
        expect(dropped != nullptr && dropped->get_if<gr::Size_t>() != nullptr);
        expect(lost != nullptr && lost->get_if<std::uint64_t>() != nullptr);
        expect(position != nullptr && position->get_if<std::uint64_t>() != nullptr);
        if (dropped == nullptr || lost == nullptr || position == nullptr) {
            return;
        }
        expect(eq(static_cast<std::size_t>(*dropped->get_if<gr::Size_t>()), kItems)) << "the gap's length is wrong";
        expect(eq(*lost->get_if<std::uint64_t>(), std::uint64_t{1ULL})) << "the gap's packet count is wrong";
        expect(eq(*position->get_if<std::uint64_t>(), static_cast<std::uint64_t>(kItems))) << "the gap does not name where the hole starts";
    };

    // A payload frame that is not a whole number of items cannot be spliced into the stream: doing so would leave
    // every sample after it out of step, permanently and silently.
    "a ragged message is refused and does not shift the stream"_test = [] {
        constexpr std::size_t kItems = 64UZ;

        const Endpoint endpoint;
        Receiver       receiver(endpoint.uri);
        GraphRunner    receiving(std::move(receiver.graph));
        RawPeer        peer(zmq::socket_type::push, endpoint.uri, true);

        const std::vector<Sample> ragged = countingRun(1000UZ, kItems);
        peer.send(makeEnvelope(ragged, 0ULL, 0ULL), 3UZ); // three bytes short of a sample boundary
        expect(waitFor([&receiver] { return receiver.source->counters().messagesRefused >= 1ULL; })) << "a ragged payload was taken as samples";

        const std::vector<Sample> sound = countingRun(0UZ, kItems);
        peer.send(makeEnvelope(sound, 1ULL, static_cast<std::uint64_t>(kItems)));
        expect(waitFor([&receiver] { return receiver.collector->count() >= kItems; })) << "the stream after the refusal never arrived";
        receiving.stop();

        expect(std::ranges::equal(receiver.collector->samples(), sound)) << "the refused bytes shifted the stream that followed them";
        expect(eq(receiver.source->counters().messagesRefused, std::uint64_t{1ULL}));
        expect(eq(receiver.source->counters().packetsPublished, std::uint64_t{1ULL}));
    };

    // PUSH/PULL is the pattern to pair with a chain that must see every sample, and its whole claim is that it
    // discards nothing.
    "PUSH and PULL lose nothing over a run"_test = [] {
        constexpr std::size_t kSamples = 20000UZ;

        const Endpoint endpoint;
        Receiver       receiver(endpoint.uri);
        GraphRunner    receiving(std::move(receiver.graph));

        gr::Graph graph;
        auto&     producer = graph.emplaceBlock<TaggedRunSource>();
        auto&     sink     = graph.emplaceBlock<StreamPacketSink<Sample>>({{"endpoint", endpoint.uri}, {"pattern", std::string("push")}, {"max_items", 256U}, {"overflow", std::string("backpressure")}});
        producer._samples  = countingRun(0UZ, kSamples);
        expect(graph.connect<"out", "in">(producer, sink).has_value());
        GraphRunner sending(std::move(graph));

        expect(waitFor([&receiver] { return receiver.collector->count() >= kSamples; }, 10000ms)) << std::format("only {} of {} samples crossed", receiver.collector->count(), kSamples);
        sending.stop();
        receiving.stop();

        expect(std::ranges::equal(receiver.collector->samples(), producer._samples)) << "a lossless pattern lost or reordered samples";
        const auto counted = receiver.source->counters();
        expect(eq(counted.sequenceGaps, std::uint64_t{0ULL}));
        expect(eq(counted.droppedByBackpressure, std::uint64_t{0ULL}));
        expect(eq(sink.counters().droppedOnOverflow, std::uint64_t{0ULL})) << "backpressure shed an envelope";
    };

    // The receive queue keeps the same bound, and states it the way a source states every other loss: the arrival is
    // discarded and counted, because a source has no upstream to refuse.
    "an arrival larger than queue_bytes never enters the queue"_test = [] {
        constexpr std::uint64_t kQueueBytes = 256ULL;
        constexpr std::size_t   kWide       = 64UZ; // 512 payload bytes, which no queue of kQueueBytes can hold
        constexpr std::size_t   kNarrow     = 8UZ;

        const Endpoint endpoint;
        // driven without a graph, so nothing drains the receive queue and the queue itself is what the test reads
        StreamPacketSource<Sample> source({{"endpoint", endpoint.uri}, {"pattern", std::string("pull")}, {"max_message_bytes", kBound}, {"queue_bytes", kQueueBytes}});
        source.settings().init();
        std::ignore = source.settings().applyStagedParameters();
        source.start();

        RawPeer peer(zmq::socket_type::push, endpoint.uri, true);
        peer.send(makeEnvelope(countingRun(0UZ, kWide), 0ULL, 0ULL));
        expect(waitFor([&source] { return source.counters().overQueueBytes >= 1ULL; })) << "the oversize message was not refused by the queue";
        std::uint64_t afterOversize = 0ULL;
        std::size_t   heldOversize  = 0UZ;
        {
            std::lock_guard lock(source._mutex);
            afterOversize = source._queuedBytes;
            heldOversize  = source._queue.size();
        }

        peer.send(makeEnvelope(countingRun(0UZ, kNarrow), 1ULL, static_cast<std::uint64_t>(kWide)));
        expect(waitFor([&source] {
            std::lock_guard lock(source._mutex);
            return !source._queue.empty();
        })) << "the message that fits never reached the queue";
        std::uint64_t afterSmall = 0ULL;
        {
            std::lock_guard lock(source._mutex);
            afterSmall = source._queuedBytes;
        }
        source.stop();

        expect(eq(heldOversize, 0UZ)) << "the oversize message was queued";
        expect(eq(afterOversize, std::uint64_t{0ULL})) << "the queue accounted a message it must not hold";
        expect(le(afterSmall, kQueueBytes)) << "the receive queue held" << afterSmall << "bytes against a bound of" << kQueueBytes;
        const auto counted = source.counters();
        expect(eq(counted.overQueueBytes, std::uint64_t{1ULL}));
        expect(eq(counted.droppedByBackpressure, std::uint64_t{0ULL})) << "a message that cannot fit is not an overflow";
        expect(eq(counted.messagesRefused, std::uint64_t{0ULL})) << "the message is inside max_message_bytes; the queue is what refused it";
    };
};

int main() { /* not needed for UT */ }
