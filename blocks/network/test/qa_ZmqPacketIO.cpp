#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <format>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <zmq.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>

#include <gnuradio-4.0/network/ZmqPacketIO.hpp>

// A transport is only proved by a peer it did not build. So the assertions below meet the blocks at a socket rather
// than at a span: a raw ZeroMQ socket in the test stands in for the far process, the golden envelope is compared
// frame by frame against bytes derived by hand, and the delivery properties are asserted through what actually came
// off the wire. Nothing here sleeps to wait for a message; every wait ends on the message or on a deadline that
// fails the test.

namespace {

using namespace std::chrono_literals;
using gr::blocks::network::ZmqPacketSink;

// ─── endpoints ────────────────────────────────────────────────────────────────────────────────────────────────────
// A fixed ipc:// path is a parallel-ctest hazard, and an AF_UNIX path is bounded at about a hundred bytes, so the
// name is short, unique per process and unique per use, and the socket file is removed when the test is done with it.

[[nodiscard]] std::string uniqueEndpointPath() {
    static const std::uint64_t salt = [] {
        std::random_device device;
        return (static_cast<std::uint64_t>(device()) << 32U) | static_cast<std::uint64_t>(device());
    }();
    static std::atomic<std::uint32_t> counter{0U};
    return std::format("{}/gr4-zpio-{:016x}-{}.sock", std::filesystem::temp_directory_path().string(), salt, counter.fetch_add(1U));
}

struct Endpoint {
    std::string path{uniqueEndpointPath()};
    std::string uri{std::format("ipc://{}", path)};

    Endpoint()                           = default;
    Endpoint(const Endpoint&)            = delete;
    Endpoint& operator=(const Endpoint&) = delete;
    ~Endpoint() { std::filesystem::remove(std::filesystem::path(path)); }
};

// ─── a raw ZeroMQ peer, which is the whole point ──────────────────────────────────────────────────────────────────

struct RawPeer {
    zmq::context_t context{1};
    zmq::socket_t  socket;

    RawPeer(zmq::socket_type type, const std::string& uri, bool bindHere) : socket(context, type) {
        socket.set(zmq::sockopt::rcvtimeo, 3000);
        socket.set(zmq::sockopt::sndtimeo, 3000);
        socket.set(zmq::sockopt::linger, 0);
        if (type == zmq::socket_type::sub) {
            socket.set(zmq::sockopt::subscribe, "");
        }
        if (bindHere) {
            socket.bind(uri);
        } else {
            socket.connect(uri);
        }
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

// ─── a source of prepared packets, which never ends so the test decides when the graph stops ──────────────────────

template<typename T>
struct PacketVectorSource : gr::Block<PacketVectorSource<T>> {
    gr::PortOut<gr::Packet<T>> out;

    GR_MAKE_REFLECTABLE(PacketVectorSource, out);

    std::vector<gr::Packet<T>> _packets{};
    bool                       _repeat = false;
    std::size_t                _next   = 0UZ;
    std::atomic<std::uint64_t> _emitted{0ULL};

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (_packets.empty() || (!_repeat && _next >= _packets.size())) {
            outSpan.publish(0UZ);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS; // idle rather than done: the test owns the teardown
        }
        const std::size_t room = _repeat ? outSpan.size() : std::min(outSpan.size(), _packets.size() - _next);
        for (std::size_t i = 0UZ; i < room; ++i) {
            outSpan[i] = _packets[(_next + i) % _packets.size()];
        }
        _next += room;
        _emitted.fetch_add(static_cast<std::uint64_t>(room));
        outSpan.publish(room);
        return room == 0UZ ? gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS : gr::work::Status::OK;
    }
};

template<typename T>
struct PacketVectorSink : gr::Block<PacketVectorSink<T>> {
    gr::PortIn<gr::Packet<T>> in;

    GR_MAKE_REFLECTABLE(PacketVectorSink, in);

    mutable std::mutex         _mutex;
    std::vector<gr::Packet<T>> _packets{};

    [[nodiscard]] std::size_t count() const {
        std::lock_guard lock(_mutex);
        return _packets.size();
    }

    [[nodiscard]] std::vector<gr::Packet<T>> take() const {
        std::lock_guard lock(_mutex);
        return _packets;
    }

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        {
            std::lock_guard lock(_mutex);
            for (std::size_t i = 0UZ; i < inSpan.size(); ++i) {
                _packets.push_back(inSpan[i]);
            }
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

// ─── running a graph beside the test's own socket ─────────────────────────────────────────────────────────────────

struct GraphRunner {
    gr::scheduler::Simple<> scheduler;
    std::thread             worker;
    std::atomic<bool>       finished{false};
    std::string             failure{};

    explicit GraphRunner(gr::Graph&& graph) {
        boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
        worker = std::thread([this] {
            const auto result = scheduler.runAndWait();
            if (!result.has_value()) {
                failure = std::format("{}", result.error());
            }
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

// ─── the golden message of the specification's anchor A ───────────────────────────────────────────────────────────

constexpr std::array<std::uint8_t, 32> kGoldenHeader{0x47U, 0x52U, 0x34U, 0x50U, 0x01U, 0x00U, 0x00U, 0x20U, //
    0x06U, 0x00U, 0x01U, 0x01U, 0x05U, 0x00U, 0x00U, 0x00U,                                                  //
    0x05U, 0x00U, 0x00U, 0x00U, 0x58U, 0x00U, 0x00U, 0x00U,                                                  //
    0x00U, 0x00U, 0x00U, 0x00U, 0x45U, 0xD6U, 0xE3U, 0xBAU};

constexpr std::string_view kGoldenMetadata = "\ncrc_ok: !!bool true\nprotocol:  \"ax25\"\nsample_start: !!uint64 4096\nsequence: !!uint64 7\n";

[[nodiscard]] gr::Packet<std::uint8_t> goldenPacket() {
    gr::Packet<std::uint8_t> packet;
    packet.signal_values = {0x01U, 0x02U, 0x03U, 0x04U, 0x05U};
    packet.meta_information.resize(1UZ);
    gr::property_map& map = packet.meta_information[0UZ];
    map.insert_or_assign(gr::property_map::key_type("crc_ok"), gr::pmt::Value(true));
    map.insert_or_assign(gr::property_map::key_type("protocol"), gr::pmt::Value(std::string("ax25")));
    map.insert_or_assign(gr::property_map::key_type("sample_start"), gr::pmt::Value(static_cast<std::uint64_t>(4096)));
    map.insert_or_assign(gr::property_map::key_type("sequence"), gr::pmt::Value(static_cast<std::uint64_t>(7)));
    return packet;
}

/// @brief A packet of @p nItems bytes counting up from @p first, carrying @p sequence when it is stated.
[[nodiscard]] gr::Packet<std::uint8_t> countingPacket(std::size_t nItems, std::uint8_t first, std::optional<std::uint64_t> sequence = std::nullopt) {
    gr::Packet<std::uint8_t> packet;
    packet.signal_values.resize(nItems);
    for (std::size_t i = 0UZ; i < nItems; ++i) {
        packet.signal_values[i] = static_cast<std::uint8_t>(first + i);
    }
    packet.meta_information.resize(1UZ);
    packet.meta_information[0UZ].insert_or_assign(gr::property_map::key_type("source_id"), gr::pmt::Value(std::string("qa")));
    if (sequence.has_value()) {
        packet.meta_information[0UZ].insert_or_assign(gr::property_map::key_type("sequence"), gr::pmt::Value(*sequence));
    }
    return packet;
}

[[nodiscard]] std::uint64_t sequenceOf(std::span<const std::uint8_t> metadataFrame) {
    const auto parsed = gr::pmt::yaml::deserialize(std::string_view(reinterpret_cast<const char*>(metadataFrame.data()), metadataFrame.size()));
    boost::ut::expect(parsed.has_value()) << "a published metadata frame did not parse";
    if (!parsed.has_value()) {
        return 0ULL;
    }
    const auto entry = parsed->find("sequence");
    boost::ut::expect(entry != parsed->end()) << "an emitted envelope carried no sequence";
    return entry == parsed->end() ? 0ULL : entry->second.value_or<std::uint64_t>(0ULL);
}

/// @brief The message bound every source below runs under, well above anything these tests send.
constexpr std::uint64_t kBound = 1ULL << 20U;

[[nodiscard]] std::string stringValue(const gr::property_map& map, std::string_view key) {
    const auto entry = map.find(key);
    if (entry == map.end()) {
        return {};
    }
    const std::pmr::string* value = entry->second.get_if<std::pmr::string>();
    return value == nullptr ? std::string{} : std::string(value->begin(), value->end());
}

[[nodiscard]] std::string reasonOf(const gr::Packet<std::uint8_t>& packet) { //
    return packet.meta_information.empty() ? std::string{} : stringValue(packet.meta_information[0UZ], "discard_reason");
}

/// @brief A packet's metadata as the bytes the emitter writes for it, which compares keys, order and value types at once.
[[nodiscard]] std::string serializedMeta(const gr::Packet<std::uint8_t>& packet) { //
    return packet.meta_information.empty() ? std::string{} : gr::pmt::yaml::serialize(packet.meta_information[0UZ]);
}

// ─── a raw sender, so a message can be malformed in ways no sink would produce ────────────────────────────────────
// PUSH rather than PUB throughout: a subscription propagates asynchronously and would make "the peer sent exactly n
// messages" a race, while PUSH blocks until a peer is ready and discards nothing, so the counts below are exact.

struct RawSender {
    zmq::context_t context{1};
    zmq::socket_t  socket;

    /// @brief A PUSH peer of the source under test. With @p reconnects clear the socket stays disconnected once its
    /// connection is closed, which is what makes a disconnection performed at the far end observable from this one.
    explicit RawSender(const std::string& uri, std::chrono::milliseconds sendTimeout = 3000ms, bool reconnects = true) : socket(context, zmq::socket_type::push) {
        socket.set(zmq::sockopt::sndtimeo, static_cast<int>(sendTimeout.count()));
        socket.set(zmq::sockopt::linger, 1000);
        if (!reconnects) {
            socket.set(zmq::sockopt::reconnect_ivl, -1);
        }
        socket.connect(uri);
    }

    /// @brief Send one multipart message, reporting rather than failing when the socket has no peer to place it on.
    /// A PUSH socket tests for one on the first part only, so a refusal leaves nothing half-written.
    [[nodiscard]] bool trySendFrames(std::span<const std::vector<std::uint8_t>> frames) {
        for (std::size_t i = 0UZ; i < frames.size(); ++i) {
            const zmq::send_flags flags = i + 1UZ < frames.size() ? zmq::send_flags::sndmore : zmq::send_flags::none;
            if (!socket.send(zmq::buffer(frames[i]), flags).has_value()) {
                return false;
            }
        }
        return true;
    }

    void sendFrames(std::span<const std::vector<std::uint8_t>> frames) { //
        boost::ut::expect(trySendFrames(frames)) << "the raw sender could not place a message";
    }
};

/// @brief Whether a socket's connection has been closed under it, taken from libzmq's own event stream for that
/// socket rather than inferred. A send that fails says only that a message could not be placed, which a filled send
/// queue also produces; the event says what happened and when.
struct Disconnection : zmq::monitor_t {
    bool seen = false;

    void on_event_disconnected(const zmq_event_t& /*event*/, const char* /*address*/) override { seen = true; }

    [[nodiscard]] bool happened() {
        std::ignore = check_event(0);
        return seen;
    }
};

[[nodiscard]] std::vector<std::uint8_t> bytesOf(std::string_view text) { //
    return std::vector<std::uint8_t>(reinterpret_cast<const std::uint8_t*>(text.data()), reinterpret_cast<const std::uint8_t*>(text.data()) + text.size());
}

/// @brief Recompute the header CRC after a test has edited a covered byte, so a field refusal is reached rather than
/// the CRC catching the edit first.
void reseal(std::vector<std::uint8_t>& header) {
    const std::uint32_t crc = static_cast<std::uint32_t>(gr::network::kHeaderCrc.compute(std::span<const std::uint8_t>(header.data(), 28UZ)));
    header[28UZ]            = static_cast<std::uint8_t>(crc & 0xFFU);
    header[29UZ]            = static_cast<std::uint8_t>((crc >> 8U) & 0xFFU);
    header[30UZ]            = static_cast<std::uint8_t>((crc >> 16U) & 0xFFU);
    header[31UZ]            = static_cast<std::uint8_t>((crc >> 24U) & 0xFFU);
}

[[nodiscard]] std::vector<std::uint8_t> headerBytes(std::uint16_t itemType, std::uint8_t itemSize, std::uint32_t itemCount, std::uint32_t payloadBytes, std::uint32_t metaBytes) {
    gr::network::EnvelopeHeader header;
    header.item_type     = itemType;
    header.item_size     = itemSize;
    header.item_count    = itemCount;
    header.payload_bytes = payloadBytes;
    header.meta_bytes    = metaBytes;
    const auto encoded   = gr::network::encodeHeader(header);
    return std::vector<std::uint8_t>(encoded.begin(), encoded.end());
}

/// @brief A well-formed uint8 envelope of @p payload carrying @p sequence and @p sourceId.
[[nodiscard]] std::vector<std::vector<std::uint8_t>> uint8Envelope(std::span<const std::uint8_t> payload, std::uint64_t sequence, std::string_view sourceId = "peer") {
    gr::property_map map;
    map.insert_or_assign(gr::property_map::key_type("sequence"), gr::pmt::Value(sequence));
    map.insert_or_assign(gr::property_map::key_type("source_id"), gr::pmt::Value(std::string(sourceId)));
    const std::string metadata = gr::pmt::yaml::serialize(map);
    return {std::vector<std::uint8_t>{}, headerBytes(6U, 1U, static_cast<std::uint32_t>(payload.size()), static_cast<std::uint32_t>(payload.size()), static_cast<std::uint32_t>(metadata.size())), bytesOf(metadata), std::vector<std::uint8_t>(payload.begin(), payload.end())};
}

} // namespace

const boost::ut::suite<"ZmqPacketSink"> zmqPacketSinkTests = [] {
    using namespace boost::ut;

    // The criterion the tier gate's second process stands in for: an envelope verified against a socket that is not
    // one of these blocks is the smallest thing that proves the format is a format rather than a convention between
    // two copies of one implementation.
    "the envelope on the wire is the golden one"_test = [] {
        const Endpoint endpoint;
        RawPeer        peer(zmq::socket_type::sub, endpoint.uri, false);

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<PacketVectorSource<std::uint8_t>>();
        source._packets  = {goldenPacket()};
        source._repeat   = true; // a subscription propagates asynchronously, so the publisher repeats until one lands
        auto& sink       = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}});
        expect(graph.connect<"out", "in">(source, sink).has_value());

        GraphRunner runner(std::move(graph));
        const auto  frames = peer.receive();
        runner.stop();

        expect(frames.has_value()) << "no envelope arrived at a raw SUB socket";
        if (!frames.has_value()) {
            return;
        }
        expect(eq(frames->size(), 4UZ)) << "an envelope is four frames";
        if (frames->size() != 4UZ) {
            return;
        }
        expect(eq((*frames)[0UZ].size(), 0UZ)) << "the default topic is a zero-length frame 0";
        expect(std::ranges::equal((*frames)[1UZ], kGoldenHeader)) << "frame 1 is not the specified 32 bytes";
        expect(std::ranges::equal((*frames)[2UZ], std::span<const std::uint8_t>(reinterpret_cast<const std::uint8_t*>(kGoldenMetadata.data()), kGoldenMetadata.size()))) << "frame 2 is not the specified metadata";
        expect(std::ranges::equal((*frames)[3UZ], std::array<std::uint8_t, 5>{0x01U, 0x02U, 0x03U, 0x04U, 0x05U})) << "frame 3 is not the payload";
        // every packet the sink emits states its sequence, and this one stated its own
        expect(eq(sequenceOf((*frames)[2UZ]), std::uint64_t{7ULL}));
        expect(ge(sink.nSequenceDeclined, sink.counters().packetsSent)) << "the sink wrote its own counter over a stated sequence";
    };

    // Every envelope the sink emits carries `sequence`: those that state one keep it, those that do not receive the
    // sink's counter, and the counter advances by one per accepted packet either way.
    "the sink's sequence guarantee"_test = [] {
        const Endpoint endpoint;
        RawPeer        peer(zmq::socket_type::pull, endpoint.uri, true); // push blocks rather than drops, so nothing is lost

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<PacketVectorSource<std::uint8_t>>();
        source._packets  = {countingPacket(4UZ, 0U), countingPacket(4UZ, 10U, 900ULL), countingPacket(4UZ, 20U), countingPacket(4UZ, 30U, 901ULL), countingPacket(4UZ, 40U)};
        auto& sink       = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", false}, {"pattern", std::string("push")}, {"overflow", std::string("backpressure")}});
        expect(graph.connect<"out", "in">(source, sink).has_value());

        GraphRunner                runner(std::move(graph));
        std::vector<std::uint64_t> sequences;
        for (std::size_t i = 0UZ; i < 5UZ; ++i) {
            const auto frames = peer.receive();
            expect(frames.has_value()) << "envelope" << i << "did not arrive";
            if (!frames.has_value()) {
                break;
            }
            expect(eq(frames->size(), 4UZ));
            sequences.push_back(sequenceOf((*frames)[2UZ]));
        }
        runner.stop();

        expect(eq(sequences.size(), 5UZ));
        if (sequences.size() == 5UZ) {
            expect(eq(sequences[0UZ], std::uint64_t{0ULL})) << "the first unstated packet takes the sink's counter";
            expect(eq(sequences[1UZ], std::uint64_t{900ULL})) << "a stated sequence must survive";
            expect(eq(sequences[2UZ], std::uint64_t{2ULL})) << "the counter advances per accepted packet, stated or not";
            expect(eq(sequences[3UZ], std::uint64_t{901ULL}));
            expect(eq(sequences[4UZ], std::uint64_t{4ULL}));
        }
        expect(eq(sink.nSequenceDeclined, std::uint64_t{2ULL}));
        expect(eq(sink.counters().packetsSent, std::uint64_t{5ULL}));
    };

    // The vocabulary crosses verbatim, and a key at the wrong type is counted rather than dropped: the value's author
    // is in another process, so dropping the key would erase the only evidence that the peer is misconfigured.
    "the vocabulary is copied and a mistype is counted"_test = [] {
        const Endpoint endpoint;
        RawPeer        peer(zmq::socket_type::pull, endpoint.uri, true);

        gr::Packet<std::uint8_t> packet = countingPacket(2UZ, 1U, 5ULL);
        packet.meta_information[0UZ].insert_or_assign(gr::property_map::key_type("sample_rate"), gr::pmt::Value(48000.0)); // declared float
        packet.meta_information[0UZ].insert_or_assign(gr::property_map::key_type("n_pre"), gr::pmt::Value(static_cast<gr::Size_t>(7)));

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<PacketVectorSource<std::uint8_t>>();
        source._packets  = {packet};
        auto& sink       = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", false}, {"pattern", std::string("push")}});
        expect(graph.connect<"out", "in">(source, sink).has_value());

        GraphRunner runner(std::move(graph));
        const auto  frames = peer.receive();
        runner.stop();

        expect(frames.has_value());
        if (!frames.has_value()) {
            return;
        }
        const auto parsed = gr::pmt::yaml::deserialize(std::string_view(reinterpret_cast<const char*>((*frames)[2UZ].data()), (*frames)[2UZ].size()));
        expect(parsed.has_value());
        if (parsed.has_value()) {
            expect(parsed->contains("sample_rate")) << "a mistyped vocabulary key must arrive, not vanish";
            expect(parsed->at("sample_rate").get_if<double>() != nullptr) << "the wrong type crosses at the type it carries";
            expect(parsed->contains("n_pre")) << "a producer-private key crosses unfiltered";
            expect(parsed->contains("source_id"));
        }
        expect(eq(sink.nMetaKeysMistyped, std::uint64_t{1ULL}));
    };

    // The carrier's timestamp has no header field, so it crosses as a producer-private key rather than being lost
    // silently; default_value has neither and is counted instead.
    "the carrier fields"_test = [] {
        const Endpoint endpoint;
        RawPeer        peer(zmq::socket_type::pull, endpoint.uri, true);

        gr::Packet<std::uint8_t> stamped = countingPacket(2UZ, 1U, 1ULL);
        stamped.timestamp                = 1'700'000'000'000'000'000;
        gr::Packet<std::uint8_t> plain   = countingPacket(2UZ, 3U, 2ULL);
        gr::Packet<std::uint8_t> padded  = countingPacket(2UZ, 5U, 3ULL);
        padded.default_value             = 0xEEU;

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<PacketVectorSource<std::uint8_t>>();
        source._packets  = {stamped, plain, padded};
        auto& sink       = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", false}, {"pattern", std::string("push")}});
        expect(graph.connect<"out", "in">(source, sink).has_value());

        GraphRunner                   runner(std::move(graph));
        std::vector<gr::property_map> maps;
        for (std::size_t i = 0UZ; i < 3UZ; ++i) {
            const auto frames = peer.receive();
            expect(frames.has_value()) << "envelope" << i << "did not arrive";
            if (!frames.has_value()) {
                break;
            }
            const auto parsed = gr::pmt::yaml::deserialize(std::string_view(reinterpret_cast<const char*>((*frames)[2UZ].data()), (*frames)[2UZ].size()));
            expect(parsed.has_value());
            maps.push_back(parsed.has_value() ? *parsed : gr::property_map{});
        }
        runner.stop();

        expect(eq(maps.size(), 3UZ));
        if (maps.size() == 3UZ) {
            expect(maps[0UZ].contains("packet_timestamp")) << "a non-zero timestamp must cross";
            expect(eq(maps[0UZ].at("packet_timestamp").value_or<std::int64_t>(0), std::int64_t{1'700'000'000'000'000'000}));
            expect(!maps[1UZ].contains("packet_timestamp")) << "a zero timestamp adds no key at all";
            expect(!maps[2UZ].contains("packet_timestamp"));
        }
        expect(eq(sink.nTimestampsCarried, std::uint64_t{1ULL}));
        expect(eq(sink.nDefaultValuesDropped, std::uint64_t{1ULL}));
    };

    // A packet whose envelope would exceed the bound leaves by the failure port with a named reason, and it does not
    // advance the sequence counter, so a refusal never renumbers the stream.
    "an oversize packet is rejected, not truncated"_test = [] {
        const Endpoint endpoint;
        RawPeer        peer(zmq::socket_type::pull, endpoint.uri, true);

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<PacketVectorSource<std::uint8_t>>();
        source._packets  = {countingPacket(4096UZ, 0U), countingPacket(4UZ, 1U)};
        auto& sink       = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", false}, {"pattern", std::string("push")}, {"max_message_bytes", static_cast<std::uint64_t>(512)}});
        auto& reject     = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(source, sink).has_value());
        expect(graph.connect<"reject", "in">(sink, reject).has_value());

        GraphRunner runner(std::move(graph));
        const auto  frames = peer.receive();
        expect(waitFor([&reject] { return reject.count() >= 1UZ; }));
        runner.stop();

        expect(frames.has_value()) << "the admissible packet did not arrive";
        if (frames.has_value()) {
            expect(eq((*frames)[3UZ].size(), 4UZ)) << "the packet that arrived is the small one";
            expect(eq(sequenceOf((*frames)[2UZ]), std::uint64_t{0ULL})) << "a refused packet must not advance the counter";
        }
        expect(eq(sink.nRejectedPackets, std::uint64_t{1ULL}));
        const std::vector<gr::Packet<std::uint8_t>> refused = reject.take();
        expect(eq(refused.size(), 1UZ));
        if (refused.size() == 1UZ) {
            const std::pmr::string* reason = refused[0UZ].meta_information[0UZ].at("discard_reason").get_if<std::pmr::string>();
            expect(reason != nullptr);
            expect(reason != nullptr && std::string_view(*reason) == "over_max_message_bytes");
        }
    };

    "drop_oldest sheds the stalest envelope and never stalls the graph"_test = [] {
        const Endpoint endpoint;

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<PacketVectorSource<std::uint8_t>>();
        source._packets  = {countingPacket(8UZ, 0U)};
        source._repeat   = true;
        // push with no peer, so nothing leaves the queue and the queue is the only thing that can give
        auto& sink = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("push")}, {"queue_messages", static_cast<gr::Size_t>(4)}});
        expect(graph.connect<"out", "in">(source, sink).has_value());

        GraphRunner runner(std::move(graph));
        expect(waitFor([&sink] { return sink.counters().droppedOnOverflow > 100ULL; })) << "drop_oldest did not shed a full queue";
        const std::uint64_t emitted = source._emitted.load();
        runner.stop();

        expect(gt(emitted, std::uint64_t{100ULL})) << "the upstream block was starved although the sink was told to drop";
        expect(eq(sink.nBackpressureStalls, std::uint64_t{0ULL})) << "drop_oldest must not stall";
        expect(eq(sink.counters().packetsSent, std::uint64_t{0ULL})) << "no peer was connected, so nothing was sent";
    };

    "backpressure stops consuming instead of dropping"_test = [] {
        const Endpoint endpoint;

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<PacketVectorSource<std::uint8_t>>();
        source._packets  = {countingPacket(8UZ, 0U)};
        source._repeat   = true;
        auto& sink       = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("push")}, {"overflow", std::string("backpressure")}, {"queue_messages", static_cast<gr::Size_t>(4)}});
        expect(graph.connect<"out", "in">(source, sink).has_value());

        GraphRunner runner(std::move(graph));
        expect(waitFor([&sink] { return sink.nBackpressureStalls > 10ULL; })) << "backpressure never stalled";
        const std::uint64_t emitted = source._emitted.load();
        runner.stop();

        expect(eq(sink.counters().droppedOnOverflow, std::uint64_t{0ULL})) << "backpressure must lose nothing";
        expect(lt(emitted, std::uint64_t{100000ULL})) << "an upstream block behind a stalled sink is bounded by the buffer, not free-running";
    };

    // The lossless configuration: nothing is sent while no PULL peer exists, nothing is lost, and the moment a peer
    // connects every packet arrives in order.
    "push with backpressure loses nothing while no peer exists"_test = [] {
        const Endpoint endpoint;

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<PacketVectorSource<std::uint8_t>>();
        for (std::uint8_t i = 0U; i < 8U; ++i) {
            source._packets.push_back(countingPacket(4UZ, static_cast<std::uint8_t>(i * 10U), static_cast<std::uint64_t>(i)));
        }
        auto& sink = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("push")}, {"overflow", std::string("backpressure")}});
        expect(graph.connect<"out", "in">(source, sink).has_value());

        GraphRunner runner(std::move(graph));
        expect(waitFor([&source] { return source._emitted.load() >= 8ULL; })) << "the source never got its packets out";
        expect(eq(sink.counters().droppedOnOverflow, std::uint64_t{0ULL}));

        RawPeer                    peer(zmq::socket_type::pull, endpoint.uri, false);
        std::vector<std::uint64_t> sequences;
        for (std::size_t i = 0UZ; i < 8UZ; ++i) {
            const auto frames = peer.receive();
            expect(frames.has_value()) << "envelope" << i << "did not arrive after the peer connected";
            if (!frames.has_value()) {
                break;
            }
            sequences.push_back(sequenceOf((*frames)[2UZ]));
        }
        runner.stop();

        expect(eq(sequences.size(), 8UZ));
        expect(std::ranges::is_sorted(sequences)) << "a PUSH socket preserves order per peer";
        for (std::size_t i = 0UZ; i < sequences.size(); ++i) {
            expect(eq(sequences[i], static_cast<std::uint64_t>(i)));
        }
        expect(eq(sink.counters().droppedAtStop, std::uint64_t{0ULL}));
    };

    "validation and lifecycle"_test = [] {
        const Endpoint endpoint;
        // the block refuses to start rather than binding to nothing: a block constructed with no settings at all is
        // never passed through settingsChanged, so start() is the gate that has to hold
        const auto refuses = [](gr::property_map settings, std::string_view what) {
            gr::Graph graph;
            auto&     block = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>(std::move(settings));
            expect(throws<gr::exception>([&block] { block.start(); })) << what;
        };
        refuses({{"endpoint", std::string("")}}, "an empty endpoint");
        refuses({{"endpoint", endpoint.uri}, {"pattern", std::string("req")}}, "an unknown pattern");
        refuses({{"endpoint", endpoint.uri}, {"overflow", std::string("block")}}, "an unknown overflow policy");
        refuses({{"endpoint", endpoint.uri}, {"max_message_bytes", static_cast<std::uint64_t>(0)}}, "a zero message bound");
        refuses({{"endpoint", endpoint.uri}, {"queue_messages", static_cast<gr::Size_t>(0)}}, "a zero queue depth");

        // an endpoint with no transport prefix is the common typo, and the diagnostic names it
        gr::Graph   graph;
        auto&       sink = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>({{"endpoint", std::string("127.0.0.1:5555")}});
        std::string message;
        try {
            sink.start();
        } catch (const gr::exception& error) {
            message = error.message;
        }
        expect(message.contains("names no transport")) << "the missing-prefix hint is missing: " << message;
    };

    "a socket setting cannot change under a running graph"_test = [] {
        const Endpoint endpoint;
        const Endpoint other;

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<PacketVectorSource<std::uint8_t>>();
        source._packets  = {countingPacket(4UZ, 0U)};
        source._repeat   = true;
        auto& sink       = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("push")}});
        expect(graph.connect<"out", "in">(source, sink).has_value());

        GraphRunner runner(std::move(graph));
        expect(waitFor([&source] { return source._emitted.load() > 0ULL; }));
        std::string message;
        try {
            sink.endpoint.value = other.uri;
            sink.rebuild();
        } catch (const gr::exception& error) {
            message = error.message;
        }
        runner.stop();
        expect(message.contains("endpoint")) << "a live endpoint change was not refused by name: " << message;
    };

    // A stalled peer must not make teardown unbounded: the queue is discarded, counted, and the graph comes down.
    "teardown is bounded and counts what it discards"_test = [] {
        const Endpoint endpoint;

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<PacketVectorSource<std::uint8_t>>();
        source._packets  = {countingPacket(8UZ, 0U)};
        source._repeat   = true;
        auto& sink       = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("push")}, {"overflow", std::string("backpressure")}, {"queue_messages", static_cast<gr::Size_t>(4)}});
        expect(graph.connect<"out", "in">(source, sink).has_value());

        const auto    started       = std::chrono::steady_clock::now();
        std::uint64_t droppedAtStop = 0ULL;
        std::uint64_t packetsSent   = 0ULL;
        {
            GraphRunner runner(std::move(graph));
            expect(waitFor([&sink] { return sink.nBackpressureStalls > 0ULL; })) << "the queue never filled";
            runner.stop();
            // the runner owns the scheduler, which owns the graph and so the block: what the sink counted is read
            // inside this scope, because after it the block is gone
            droppedAtStop = sink.counters().droppedAtStop;
            packetsSent   = sink.counters().packetsSent;
        }
        const auto elapsed = std::chrono::steady_clock::now() - started;
        expect(elapsed < 5000ms) << "teardown against a peerless PUSH socket was not bounded";
        // where the I/O thread stood when the stop arrived decides whether one envelope was already in its hand, so
        // the count is the queue depth plus at most that one, and nothing beyond it can have gone anywhere
        expect(ge(droppedAtStop, std::uint64_t{1ULL})) << "the discarded queue was not counted";
        expect(le(droppedAtStop, std::uint64_t{5ULL})) << "more was discarded than the queue could hold";
        expect(eq(packetsSent, std::uint64_t{0ULL})) << "no peer was connected, so nothing can have been sent";
    };

    // `queue_bytes` is a bound the queue keeps rather than a size it aims at: an envelope larger than the whole
    // queue is refused, because shedding what is queued cannot make room for it.
    "an envelope larger than queue_bytes leaves the sink by reject"_test = [] {
        constexpr std::uint64_t kQueueBytes = 256ULL;
        const Endpoint          endpoint;

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<PacketVectorSource<std::uint8_t>>();
        source._packets  = {countingPacket(4UZ, 1U), countingPacket(4096UZ, 0U)};
        // push with no peer, so nothing leaves the queue and what it holds is what the test reads
        auto& sink   = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("push")}, {"queue_bytes", kQueueBytes}});
        auto& reject = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(source, sink).has_value());
        expect(graph.connect<"reject", "in">(sink, reject).has_value());

        GraphRunner runner(std::move(graph));
        expect(waitFor([&reject] { return reject.count() >= 1UZ; })) << "the oversize packet never reached the failure port";
        const std::uint64_t queued = sink._sender.queuedBytes();
        runner.stop();

        expect(le(queued, kQueueBytes)) << "the send queue held" << queued << "bytes against a bound of" << kQueueBytes;
        const auto counted = sink.counters();
        expect(eq(counted.overQueueBytes, std::uint64_t{1ULL}));
        expect(eq(counted.rejectedPackets, std::uint64_t{0ULL})) << "the packet is inside max_message_bytes; the queue is what refused it";
        expect(eq(counted.droppedOnOverflow, std::uint64_t{0ULL})) << "nothing queued was shed to make room for what cannot fit";
        const std::vector<gr::Packet<std::uint8_t>> refused = reject.take();
        expect(eq(refused.size(), 1UZ));
        if (refused.size() == 1UZ) {
            expect(eq(reasonOf(refused[0UZ]), std::string("over_queue_bytes")));
            expect(eq(refused[0UZ].signal_values.size(), 4096UZ)) << "a refused packet is republished whole";
        }
    };

    "start, stop and start again leave nothing behind"_test = [] {
        const Endpoint endpoint;
        gr::Graph      graph;
        auto&          sink = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("push")}});
        for (std::size_t i = 0UZ; i < 10UZ; ++i) {
            expect(nothrow([&sink] { sink.start(); })) << "start" << i;
            expect(nothrow([&sink] { sink.stop(); })) << "stop" << i;
        }
    };
};

const boost::ut::suite<"ZmqPacketSource"> zmqPacketSourceTests = [] {
    using namespace boost::ut;
    using gr::blocks::network::ZmqPacketSource;

    // The loopback the tier gate stands on, in the configuration that discards nothing, so "every packet arrives
    // once, in order, with its payload and its map" is an exact assertion rather than a probabilistic one.
    "loopback over a real socket, lossless"_test = [] {
        const Endpoint endpoint;

        std::vector<gr::Packet<std::uint8_t>> sent;
        for (std::uint32_t i = 0U; i < 100U; ++i) {
            const std::size_t nItems = (i == 17U) ? 1UZ : (1UZ + (i % 23UZ)); // one packet of a single item, deliberately
            sent.push_back(countingPacket(nItems, static_cast<std::uint8_t>(i)));
        }

        gr::Graph graph;
        auto&     producer = graph.emplaceBlock<PacketVectorSource<std::uint8_t>>();
        producer._packets  = sent;
        auto& sink         = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("push")}, {"overflow", std::string("backpressure")}});
        auto& source       = graph.emplaceBlock<ZmqPacketSource<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", false}, {"pattern", std::string("pull")}, {"max_message_bytes", kBound}});
        auto& collector    = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(producer, sink).has_value());
        expect(graph.connect<"out", "in">(source, collector).has_value());

        GraphRunner runner(std::move(graph));
        expect(waitFor([&collector] { return collector.count() >= 100UZ; })) << "only" << collector.count() << "of 100 packets crossed";
        runner.stop();

        const std::vector<gr::Packet<std::uint8_t>> received = collector.take();
        expect(eq(received.size(), 100UZ));
        for (std::size_t i = 0UZ; i < std::min(received.size(), sent.size()); ++i) {
            expect(std::ranges::equal(received[i].signal_values, sent[i].signal_values)) << "packet" << i << "payload";
            expect(eq(received[i].meta_information.size(), 1UZ));
            if (!received[i].meta_information.empty()) {
                expect(eq(stringValue(received[i].meta_information[0UZ], "source_id"), std::string("qa"))) << "packet" << i << "source_id";
                expect(eq(received[i].meta_information[0UZ].at("sequence").value_or<std::uint64_t>(~0ULL), static_cast<std::uint64_t>(i))) << "packet" << i << "sequence";
            }
        }
        expect(eq(source.nSequenceGaps, std::uint64_t{0ULL})) << "a lossless link reported loss";
        expect(eq(source.nDroppedByBackpressure, std::uint64_t{0ULL}));
        expect(eq(source.nPacketsPublished, std::uint64_t{100ULL}));
    };

    // The same loopback, with the sink's input port bounded to a fixed number of items per call. What crosses is what
    // the whole stream in one call produces — the same packets, the same metadata and the same number of them —
    // wherever the call boundaries fall, which is the property that makes the pair safe under any scheduler.
    "the crossing does not depend on the sink's input chunk size"_test = [] {
        std::vector<gr::Packet<std::uint8_t>> sent;
        for (std::uint32_t i = 0U; i < 100U; ++i) {
            const std::size_t nItems = (i == 17U) ? 1UZ : (1UZ + (i % 23UZ)); // one packet of a single item, deliberately
            sent.push_back(countingPacket(nItems, static_cast<std::uint8_t>(i)));
        }

        // `room` bounds the items one call at the sink may take; 0 stands for the whole stream in one call, which the
        // port's own lower bound makes exact and the edge's buffer is sized to hold
        const auto crossing = [&sent](std::size_t room) {
            const Endpoint endpoint;

            gr::Graph graph;
            auto&     producer = graph.emplaceBlock<PacketVectorSource<std::uint8_t>>();
            producer._packets  = sent;
            auto& sink         = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("push")}, {"overflow", std::string("backpressure")}});
            auto& source       = graph.emplaceBlock<ZmqPacketSource<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", false}, {"pattern", std::string("pull")}, {"max_message_bytes", kBound}});
            auto& collector    = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
            if (room == 0UZ) {
                sink.in.min_samples = sent.size();
            } else {
                sink.in.max_samples = room;
            }
            expect(graph.connect<"out", "in">(producer, sink, gr::EdgeParameters{.minBufferSize = 256UZ}).has_value());
            expect(graph.connect<"out", "in">(source, collector).has_value());

            GraphRunner runner(std::move(graph));
            expect(waitFor([&collector] { return collector.count() >= 100UZ; })) << std::format("room {}: only {} of 100 packets crossed", room, collector.count());
            runner.stop();
            return collector.take();
        };

        const std::vector<gr::Packet<std::uint8_t>> whole = crossing(0UZ);
        expect(eq(whole.size(), 100UZ)) << "the whole stream in one call did not cross";

        for (const std::size_t room : {1UZ, 3UZ, 17UZ, 4096UZ}) {
            const std::vector<gr::Packet<std::uint8_t>> chunked = crossing(room);
            expect(eq(chunked.size(), whole.size())) << std::format("room {}: multiplicity", room);
            for (std::size_t i = 0UZ; i < std::min(chunked.size(), whole.size()); ++i) {
                expect(std::ranges::equal(chunked[i].signal_values, sent[i].signal_values)) << std::format("room {}: packet {} is not what was sent", room, i);
                expect(std::ranges::equal(chunked[i].signal_values, whole[i].signal_values)) << std::format("room {}: packet {} items", room, i);
                expect(eq(serializedMeta(chunked[i]), serializedMeta(whole[i]))) << std::format("room {}: packet {} metadata", room, i);
            }
        }
    };

    // The default configuration. A subscription propagates asynchronously and a PUB socket drops in its mute state,
    // so what is asserted here is what PUB actually guarantees: whatever arrives is in order, unduplicated, and
    // carries the payload its sequence names.
    "loopback over the default pub/sub pair"_test = [] {
        const Endpoint endpoint;

        gr::Graph graph;
        auto&     producer = graph.emplaceBlock<PacketVectorSource<std::uint8_t>>();
        for (std::uint32_t i = 0U; i < 64U; ++i) {
            producer._packets.push_back(countingPacket(4UZ, static_cast<std::uint8_t>(i)));
        }
        producer._repeat = true;
        auto& sink       = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}});
        auto& source     = graph.emplaceBlock<ZmqPacketSource<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", false}, {"max_message_bytes", kBound}});
        auto& collector  = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(producer, sink).has_value());
        expect(graph.connect<"out", "in">(source, collector).has_value());

        GraphRunner runner(std::move(graph));
        expect(waitFor([&collector] { return collector.count() >= 32UZ; })) << "nothing crossed the default pub/sub pair";
        runner.stop();

        const std::vector<gr::Packet<std::uint8_t>> received = collector.take();
        expect(ge(received.size(), 32UZ));
        std::uint64_t previous = 0ULL;
        for (std::size_t i = 0UZ; i < received.size(); ++i) {
            expect(eq(received[i].meta_information.size(), 1UZ));
            if (received[i].meta_information.empty()) {
                continue;
            }
            const std::uint64_t sequence = received[i].meta_information[0UZ].at("sequence").value_or<std::uint64_t>(~0ULL);
            if (i > 0UZ) {
                expect(gt(sequence, previous)) << "a PUB link duplicated or reordered a packet";
            }
            previous = sequence;
            expect(eq(received[i].signal_values.size(), 4UZ));
            expect(eq(received[i].signal_values[0UZ], static_cast<std::uint8_t>(sequence % 64ULL))) << "packet" << sequence << "carried another packet's payload";
        }
    };

    // Refusal is observable three ways at once, and the third is that a mismatched peer publishing at rate does not
    // turn a version mismatch into a log flood.
    "a future wire version is refused observably"_test = [] {
        const Endpoint endpoint;

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<ZmqPacketSource<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("pull")}, {"max_message_bytes", kBound}});
        auto&     collector = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        auto&     refusals  = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(source, collector).has_value());
        expect(graph.connect<"reject", "in">(source, refusals).has_value());

        GraphRunner                            runner(std::move(graph));
        RawSender                              sender(endpoint.uri);
        const std::array<std::uint8_t, 5>      payload{1U, 2U, 3U, 4U, 5U};
        std::vector<std::vector<std::uint8_t>> frames = uint8Envelope(payload, 1ULL);
        frames[1UZ][4UZ]                              = 2U; // a wire version above this reader's
        reseal(frames[1UZ]);
        sender.sendFrames(frames);
        expect(waitFor([&refusals] { return refusals.count() >= 1UZ; })) << "a refused envelope never reached the failure port";
        sender.sendFrames(frames);
        expect(waitFor([&refusals] { return refusals.count() >= 2UZ; })) << "the second refusal never arrived";
        runner.stop();

        expect(eq(collector.count(), 0UZ)) << "a refused envelope must not appear on out";
        expect(eq(source.nRefusedVersion, std::uint64_t{2ULL})) << "the counter must see both, however few are logged";
        const std::vector<gr::Packet<std::uint8_t>> refused = refusals.take();
        expect(eq(refused.size(), 2UZ));
        if (!refused.empty()) {
            const std::uint64_t total = 32ULL + frames[2UZ].size() + frames[3UZ].size();
            expect(eq(reasonOf(refused[0UZ]), std::string("future_version")));
            expect(eq(refused[0UZ].meta_information[0UZ].at("envelope_bytes_total").value_or<std::uint64_t>(0ULL), total));
            expect(eq(refused[0UZ].meta_information[0UZ].at("envelope_bytes_kept").value_or<std::uint64_t>(0ULL), std::min(total, std::uint64_t{256ULL})));
            expect(eq(refused[0UZ].meta_information[0UZ].at("envelope_frames").value_or<gr::Size_t>(0U), 4U));
            expect(eq(refused[0UZ].signal_values.size(), static_cast<std::size_t>(std::min(total, std::uint64_t{256ULL}))));
            expect(std::ranges::equal(std::span<const std::uint8_t>(refused[0UZ].signal_values).first(32UZ), frames[1UZ])) << "the refusal did not keep the header it refused";
        }
    };

    // Eight malformations in one run: the graph keeps running, nothing escapes as an exception, each leaves by the
    // failure port with its own name, and the counters add up.
    "truncation and malformation do not stop the graph"_test = [] {
        const Endpoint endpoint;

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<ZmqPacketSource<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("pull")}, {"max_message_bytes", kBound}});
        auto&     collector = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        auto&     refusals  = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(source, collector).has_value());
        expect(graph.connect<"reject", "in">(source, refusals).has_value());

        GraphRunner                       runner(std::move(graph));
        RawSender                         sender(endpoint.uri);
        const std::array<std::uint8_t, 4> payload{1U, 2U, 3U, 4U};
        const auto                        good = uint8Envelope(payload, 1ULL);

        sender.sendFrames(std::vector<std::vector<std::uint8_t>>{good[0UZ], good[1UZ], good[2UZ]});                       // three frames
        sender.sendFrames(std::vector<std::vector<std::uint8_t>>{good[0UZ], good[1UZ], good[2UZ], good[3UZ], good[3UZ]}); // five frames

        std::vector<std::vector<std::uint8_t>> shortHeader = good;
        shortHeader[1UZ].resize(31UZ);
        sender.sendFrames(shortHeader);

        std::vector<std::vector<std::uint8_t>> metaTooLong = good;
        metaTooLong[1UZ]                                   = headerBytes(6U, 1U, 4U, 4U, static_cast<std::uint32_t>(good[2UZ].size() + 100U));
        sender.sendFrames(metaTooLong);

        std::vector<std::vector<std::uint8_t>> payloadTooLong = good;
        payloadTooLong[1UZ]                                   = headerBytes(6U, 1U, 8U, 8U, static_cast<std::uint32_t>(good[2UZ].size()));
        sender.sendFrames(payloadTooLong);

        const std::string_view                 unclosed = "[unclosed";
        std::vector<std::vector<std::uint8_t>> badYaml  = good;
        badYaml[2UZ]                                    = bytesOf(unclosed);
        badYaml[1UZ]                                    = headerBytes(6U, 1U, 4U, 4U, static_cast<std::uint32_t>(unclosed.size()));
        sender.sendFrames(badYaml);

        const std::string_view                 twoDocuments = "---\na: !!uint64 1\n---\nb: !!uint64 2\n";
        std::vector<std::vector<std::uint8_t>> multiDoc     = good;
        multiDoc[2UZ]                                       = bytesOf(twoDocuments);
        multiDoc[1UZ]                                       = headerBytes(6U, 1U, 4U, 4U, static_cast<std::uint32_t>(twoDocuments.size()));
        sender.sendFrames(multiDoc);

        std::vector<std::vector<std::uint8_t>> wrongItem = good;
        wrongItem[1UZ]                                   = headerBytes(10U, 4U, 1U, 4U, static_cast<std::uint32_t>(good[2UZ].size())); // Float32
        sender.sendFrames(wrongItem);

        expect(waitFor([&refusals] { return refusals.count() >= 8UZ; })) << "only" << refusals.count() << "of 8 malformed messages were reported";
        expect(!runner.finished.load()) << "a malformed message stopped the graph";
        sender.sendFrames(good); // and the reader is still able to take a legal message afterwards
        expect(waitFor([&collector] { return collector.count() >= 1UZ; })) << "the reader did not recover";
        runner.stop();

        expect(eq(source.nBadFrameCount, std::uint64_t{2ULL}));
        expect(eq(source.nShortHeader, std::uint64_t{1ULL}));
        expect(eq(source.nLengthMismatch, std::uint64_t{2ULL}));
        expect(eq(source.nBadMetadata, std::uint64_t{2ULL}));
        expect(eq(source.nItemTypeMismatch, std::uint64_t{1ULL}));
        expect(eq(source.nPacketsPublished, std::uint64_t{1ULL}));

        std::vector<std::string> reasons;
        for (const gr::Packet<std::uint8_t>& packet : refusals.take()) {
            reasons.push_back(reasonOf(packet));
        }
        expect(ge(reasons.size(), 8UZ));
        if (reasons.size() >= 8UZ) {
            expect(eq(reasons[0UZ], std::string("frame_count")));
            expect(eq(reasons[1UZ], std::string("frame_count")));
            expect(eq(reasons[2UZ], std::string("short_header")));
            expect(eq(reasons[3UZ], std::string("length_mismatch")));
            expect(eq(reasons[4UZ], std::string("length_mismatch")));
            expect(eq(reasons[5UZ], std::string("bad_metadata")));
            expect(eq(reasons[6UZ], std::string("bad_metadata")));
            expect(eq(reasons[7UZ], std::string("item_type_mismatch")));
        }
    };

    // Neither end of a ZeroMQ link can count what the transport dropped, so loss is reconstructed from what arrives.
    "loss is detected from a sequence gap"_test = [] {
        const Endpoint endpoint;

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<ZmqPacketSource<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("pull")}, {"max_message_bytes", kBound}, {"max_tracked_sources", static_cast<gr::Size_t>(2)}});
        auto&     collector = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(source, collector).has_value());

        GraphRunner                       runner(std::move(graph));
        RawSender                         sender(endpoint.uri);
        const std::array<std::uint8_t, 2> payload{7U, 8U};
        for (const std::uint64_t sequence : {100ULL, 101ULL, 105ULL, 106ULL, 109ULL}) {
            sender.sendFrames(uint8Envelope(payload, sequence, "alpha"));
        }
        expect(waitFor([&collector] { return collector.count() >= 5UZ; })) << "the gapped stream did not arrive";
        expect(eq(source.nSequenceGaps, std::uint64_t{2ULL}));
        expect(eq(source.nPacketsLost, std::uint64_t{5ULL}));

        // a lower sequence is a producer restart, not loss
        sender.sendFrames(uint8Envelope(payload, 3ULL, "alpha"));
        expect(waitFor([&collector] { return collector.count() >= 6UZ; }));
        expect(eq(source.nSequenceResets, std::uint64_t{1ULL}));
        expect(eq(source.nPacketsLost, std::uint64_t{5ULL})) << "a restart was counted as loss";

        // a second producer is tracked on its own, and its first envelope is never a gap however high it starts
        sender.sendFrames(uint8Envelope(payload, 5000ULL, "beta"));
        expect(waitFor([&collector] { return collector.count() >= 7UZ; }));
        expect(eq(source.nSequenceGaps, std::uint64_t{2ULL})) << "a late joiner was reported as loss";
        expect(eq(source.nSourcesUntracked, std::uint64_t{0ULL}));

        // and the tracker is bounded: the third distinct id evicts the least recently seen one
        sender.sendFrames(uint8Envelope(payload, 1ULL, "gamma"));
        expect(waitFor([&collector] { return collector.count() >= 8UZ; }));
        runner.stop();
        expect(eq(source.nSourcesUntracked, std::uint64_t{1ULL}));
        expect(eq(source.envelopesReceived(), std::uint64_t{8ULL}));
    };

    // The vocabulary crosses verbatim at this end too, and the carrier's timestamp arrives in the field it belongs
    // in rather than as a second spelling in the map.
    "the vocabulary and the carrier fields arrive"_test = [] {
        const Endpoint endpoint;

        gr::Packet<std::uint8_t> stamped = countingPacket(3UZ, 1U, 11ULL);
        stamped.timestamp                = -1;
        stamped.meta_information[0UZ].insert_or_assign(gr::property_map::key_type("sample_rate"), gr::pmt::Value(48000.0)); // declared float
        stamped.meta_information[0UZ].insert_or_assign(gr::property_map::key_type("n_pre"), gr::pmt::Value(static_cast<gr::Size_t>(7)));
        gr::Packet<std::uint8_t> plain = countingPacket(3UZ, 4U, 12ULL);

        gr::Graph graph;
        auto&     producer = graph.emplaceBlock<PacketVectorSource<std::uint8_t>>();
        producer._packets  = {stamped, plain};
        auto& sink         = graph.emplaceBlock<ZmqPacketSink<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("push")}, {"overflow", std::string("backpressure")}});
        auto& source       = graph.emplaceBlock<ZmqPacketSource<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", false}, {"pattern", std::string("pull")}, {"max_message_bytes", kBound}});
        auto& collector    = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(producer, sink).has_value());
        expect(graph.connect<"out", "in">(source, collector).has_value());

        GraphRunner runner(std::move(graph));
        expect(waitFor([&collector] { return collector.count() >= 2UZ; }));
        runner.stop();

        const std::vector<gr::Packet<std::uint8_t>> received = collector.take();
        expect(eq(received.size(), 2UZ));
        if (received.size() != 2UZ) {
            return;
        }
        expect(eq(received[0UZ].timestamp, std::int64_t{-1})) << "the carrier's timestamp did not cross";
        expect(!received[0UZ].meta_information[0UZ].contains("packet_timestamp")) << "the crossing key was left in the map";
        expect(eq(received[1UZ].timestamp, std::int64_t{0}));
        expect(received[0UZ].meta_information[0UZ].contains("sample_rate")) << "a mistyped vocabulary key must arrive, not vanish";
        expect(received[0UZ].meta_information[0UZ].at("sample_rate").get_if<double>() != nullptr);
        expect(received[0UZ].meta_information[0UZ].contains("n_pre")) << "a producer-private key crosses unfiltered";
        expect(eq(received[0UZ].default_value, std::uint8_t{0U}));
        expect(eq(source.nTimestampsCarried, std::uint64_t{1ULL}));
        expect(eq(source.nMetaKeysMistyped, std::uint64_t{1ULL}));
        expect(eq(sink.nTimestampsCarried, std::uint64_t{1ULL}));
    };

    // An unwired failure port costs nothing: the same refusals are counted and the admitted output is unchanged.
    "an unconnected reject port changes nothing"_test = [] {
        const Endpoint endpoint;

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<ZmqPacketSource<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("pull")}, {"max_message_bytes", kBound}});
        auto&     collector = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(source, collector).has_value());

        GraphRunner                            runner(std::move(graph));
        RawSender                              sender(endpoint.uri);
        const std::array<std::uint8_t, 2>      payload{9U, 10U};
        std::vector<std::vector<std::uint8_t>> future = uint8Envelope(payload, 1ULL);
        future[1UZ][4UZ]                              = 2U;
        reseal(future[1UZ]);
        for (std::size_t i = 0UZ; i < 4UZ; ++i) {
            sender.sendFrames(future);
        }
        sender.sendFrames(uint8Envelope(payload, 2ULL));
        expect(waitFor([&collector] { return collector.count() >= 1UZ; })) << "an unwired reject port wedged the queue";
        runner.stop();

        expect(eq(source.nRefusedVersion, std::uint64_t{4ULL}));
        expect(eq(collector.count(), 1UZ));
    };

    // `max_message_bytes` is pushed into libzmq as well as checked here, so a message larger than the bound is refused
    // by the library while it is still reading the frame's length and nothing is ever allocated for it. That the block
    // never saw it is asserted where it can be: no refusal is produced and the block's own bound never fires, which is
    // what would happen instead if the library had let the message through. What the peer sees is its connection
    // closed under it, and the source serves the next peer that connects.
    "an oversize message costs the peer its connection and reaches no counter"_test = [] {
        const Endpoint endpoint;

        gr::Graph graph;
        auto&     source    = graph.emplaceBlock<ZmqPacketSource<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("pull")}, {"max_message_bytes", static_cast<std::uint64_t>(4096)}});
        auto&     collector = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        auto&     refusals  = graph.emplaceBlock<PacketVectorSink<std::uint8_t>>();
        expect(graph.connect<"out", "in">(source, collector).has_value());
        expect(graph.connect<"reject", "in">(source, refusals).has_value());

        GraphRunner runner(std::move(graph));

        const std::array<std::uint8_t, 8> small{1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};
        {
            // the peer below is not reconnected by libzmq for any reason, a failed first attempt included, so it may
            // not connect before the endpoint exists
            expect(waitFor([&endpoint] { return std::filesystem::exists(std::filesystem::path(endpoint.path)); })) << "the source never bound its endpoint";

            // a peer libzmq will not reconnect for, so the disconnection stays observable instead of being repaired
            // before it can be seen, and a short send timeout so an unplaceable message says so rather than waiting
            RawSender     peer(endpoint.uri, 100ms, false);
            Disconnection closed;
            closed.init(peer.socket, "inproc://oversize.monitor", ZMQ_EVENT_DISCONNECTED);

            peer.sendFrames(uint8Envelope(small, 1ULL, "peer"));
            expect(waitFor([&collector] { return collector.count() >= 1UZ; })) << "the link carried nothing before the oversize message";

            const std::vector<std::uint8_t> oversize(8192UZ, 0xA5U); // one frame above the bound, and the envelope with it
            expect(peer.trySendFrames(uint8Envelope(oversize, 2ULL, "peer"))) << "the oversize message would not leave the peer's own socket";

            expect(waitFor([&closed] { return closed.happened(); })) << "the oversize message did not cost the peer its connection";

            // and the connection is not replaced, so the peer runs out of anywhere to place a message. The socket is
            // asked whether it can still send rather than being made to prove it by sending: a send queue filled by
            // repeated attempts would refuse too, and that refusal would say nothing about the connection. The
            // pipe outlives the event by a little, so the wait is on the socket's own writability.
            expect(waitFor([&peer] { return (peer.socket.get(zmq::sockopt::events) & ZMQ_POLLOUT) == 0; })) << "the peer's socket still had somewhere to place a message";
            expect(!peer.trySendFrames(uint8Envelope(small, 3ULL, "peer"))) << "the peer could still send after its connection was closed";
        }

        RawSender rejoined(endpoint.uri);
        rejoined.sendFrames(uint8Envelope(small, 1ULL, "rejoined"));
        expect(waitFor([&collector] {
            const std::vector<gr::Packet<std::uint8_t>> arrived = collector.take();
            return std::ranges::any_of(arrived, [](const gr::Packet<std::uint8_t>& packet) { return !packet.meta_information.empty() && stringValue(packet.meta_information[0UZ], "source_id") == "rejoined"; });
        })) << "the source did not serve a peer that connected after the disconnection";
        runner.stop();

        expect(eq(source.nOverMax, std::uint64_t{0ULL})) << "the block's own bound fired, so libzmq passed the oversize message up to it";
        expect(eq(refusals.count(), 0UZ)) << "the oversize message reached the block";
        for (const gr::Packet<std::uint8_t>& packet : collector.take()) {
            expect(le(packet.signal_values.size(), 8UZ)) << "the oversize payload was delivered";
        }
    };

    "validation and lifecycle"_test = [] {
        const Endpoint endpoint;
        const auto     refuses = [](gr::property_map settings, std::string_view what) {
            gr::Graph graph;
            auto&     block = graph.emplaceBlock<ZmqPacketSource<std::uint8_t>>(std::move(settings));
            expect(throws<gr::exception>([&block] { block.start(); })) << what;
        };
        refuses({{"endpoint", std::string("")}, {"max_message_bytes", kBound}}, "an empty endpoint");
        refuses({{"endpoint", endpoint.uri}}, "an unset max_message_bytes");
        refuses({{"endpoint", endpoint.uri}, {"max_message_bytes", static_cast<std::uint64_t>(0)}}, "a zero max_message_bytes");
        refuses({{"endpoint", endpoint.uri}, {"max_message_bytes", kBound}, {"pattern", std::string("req")}}, "an unknown pattern");
        refuses({{"endpoint", endpoint.uri}, {"max_message_bytes", kBound}, {"max_tracked_sources", static_cast<gr::Size_t>(0)}}, "a zero source-tracking bound");
        refuses({{"endpoint", endpoint.uri}, {"max_message_bytes", static_cast<std::uint64_t>(64)}, {"max_reject_bytes", static_cast<gr::Size_t>(256)}}, "a refusal copy larger than a message");

        gr::Graph   graph;
        auto&       source = graph.emplaceBlock<ZmqPacketSource<std::uint8_t>>({{"endpoint", std::string("127.0.0.1:5555")}, {"max_message_bytes", kBound}});
        std::string message;
        try {
            source.start();
        } catch (const gr::exception& error) {
            message = error.message;
        }
        expect(message.contains("names no transport")) << "the missing-prefix hint is missing: " << message;

        gr::Graph   second;
        auto&       running = second.emplaceBlock<ZmqPacketSource<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("pull")}, {"max_message_bytes", kBound}});
        std::string refusal;
        running.start();
        try {
            running.max_message_bytes.value = 4096ULL;
            running.rebuild();
        } catch (const gr::exception& error) {
            refusal = error.message;
        }
        running.stop();
        expect(refusal.contains("max_message_bytes")) << "a live change to a socket setting was not refused by name: " << refusal;
    };

    // The receive queue keeps the same bound, and states it the way a source states every other loss: the arrival is
    // discarded and counted, because a source has no upstream to refuse.
    "an arrival larger than queue_bytes never enters the queue"_test = [] {
        constexpr std::uint64_t kQueueBytes = 256ULL;
        const Endpoint          endpoint;

        // driven without a graph, so nothing drains the receive queue and the queue itself is what the test reads
        ZmqPacketSource<std::uint8_t> source({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("pull")}, {"max_message_bytes", kBound}, {"queue_bytes", kQueueBytes}});
        source.settings().init();
        std::ignore = source.settings().applyStagedParameters();
        source.start();

        RawSender                       peer(endpoint.uri);
        const std::vector<std::uint8_t> wide(4096UZ, 0xA5U);
        peer.sendFrames(uint8Envelope(wide, 1ULL));
        expect(waitFor([&source] { return source.nOverQueueBytes >= 1ULL; })) << "the oversize message was not refused by the queue";
        std::uint64_t afterOversize = 0ULL;
        std::size_t   heldOversize  = 0UZ;
        {
            std::lock_guard lock(source._mutex);
            afterOversize = source._queuedBytes;
            heldOversize  = source._queue.size();
        }

        const std::array<std::uint8_t, 4> small{1U, 2U, 3U, 4U};
        peer.sendFrames(uint8Envelope(small, 2ULL));
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
        expect(eq(source.nOverQueueBytes, std::uint64_t{1ULL}));
        expect(eq(source.nDroppedByBackpressure, std::uint64_t{0ULL})) << "a message that cannot fit is not an overflow";
        expect(eq(source.nOverMax, std::uint64_t{0ULL})) << "the message is inside max_message_bytes; the queue is what refused it";
    };

    "start, stop and start again leave nothing behind"_test = [] {
        const Endpoint endpoint;
        gr::Graph      graph;
        auto&          source = graph.emplaceBlock<ZmqPacketSource<std::uint8_t>>({{"endpoint", endpoint.uri}, {"bind", true}, {"pattern", std::string("pull")}, {"max_message_bytes", kBound}});
        for (std::size_t i = 0UZ; i < 10UZ; ++i) {
            expect(nothrow([&source] { source.start(); })) << "start" << i;
            expect(nothrow([&source] { source.stop(); })) << "stop" << i;
        }
    };
};

int main() { /* not needed for UT */ }
