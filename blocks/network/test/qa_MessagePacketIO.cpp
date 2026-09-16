#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <memory>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <zmq.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/YamlPmt.hpp>

#include <gnuradio-4.0/network/MessagePacketIO.hpp>

// The blocks are driven here without a scheduler, because what they do is not stream work: the sink's whole job is
// what happens when the message plane hands it a span, and the source's is what happens when the scheduler calls
// processScheduledMessages() on it, and both are one call each. A graph adds a thread and a teardown race and proves
// nothing further about the format; the two-graph case is qa_StreamPacketGraphs's, through the YAML examples.
//
// PUSH/PULL throughout: a publisher discards what it sends before a subscription reaches it, so a PUB test either
// races or needs an XPUB peer to wait on, while PUSH blocks in its mute state and loses nothing. That is what makes
// "every message crossed" an exact assertion here.

namespace {

using namespace std::chrono_literals;
using gr::blocks::network::MessagePacketSink;
using gr::blocks::network::MessagePacketSource;

constexpr std::uint64_t kBound = 1ULL << 20U;

[[nodiscard]] std::string uniqueEndpointPath() {
    static const std::uint64_t salt = [] {
        std::random_device device;
        return (static_cast<std::uint64_t>(device()) << 32U) | static_cast<std::uint64_t>(device());
    }();
    static std::atomic<std::uint32_t> counter{0U};
    return std::format("{}/gr4-mpio-{:016x}-{}.sock", std::filesystem::temp_directory_path().string(), salt, counter.fetch_add(1U));
}

struct Endpoint {
    std::string path{uniqueEndpointPath()};
    std::string uri{std::format("ipc://{}", path)};

    Endpoint()                           = default;
    Endpoint(const Endpoint&)            = delete;
    Endpoint& operator=(const Endpoint&) = delete;
    ~Endpoint() { std::filesystem::remove(std::filesystem::path(path)); }
};

/// @brief A block started outside a graph, which is how both halves are driven here.
template<typename TBlock>
[[nodiscard]] std::unique_ptr<TBlock> started(gr::property_map settings) {
    auto block = std::make_unique<TBlock>(std::move(settings));
    block->settings().init();
    std::ignore = block->settings().applyStagedParameters();
    block->start();
    return block;
}

/// @brief The message the crossing is judged on: every field set, a nested map and a string among the payload.
[[nodiscard]] gr::Message richMessage(std::string endpoint, std::uint64_t ordinal) {
    gr::property_map nested;
    nested.insert_or_assign(gr::property_map::key_type("depth"), gr::pmt::Value(std::uint64_t{2ULL}));
    nested.insert_or_assign(gr::property_map::key_type("label"), gr::pmt::Value(std::string("inner")));

    gr::property_map data;
    data.insert_or_assign(gr::property_map::key_type("ordinal"), gr::pmt::Value(ordinal));
    data.insert_or_assign(gr::property_map::key_type("name"), gr::pmt::Value(std::string("a string with spaces")));
    data.insert_or_assign(gr::property_map::key_type("rate"), gr::pmt::Value(48000.0f));
    data.insert_or_assign(gr::property_map::key_type("nested"), gr::pmt::Value(std::move(nested)));

    gr::Message message;
    message.protocol        = gr::message::defaultClientProtocol;
    message.cmd             = gr::message::Command::Set;
    message.serviceName     = "far::block";
    message.clientRequestID = "request-7";
    message.endpoint        = std::move(endpoint);
    message.rbac            = "token";
    message.data            = std::move(data);
    return message;
}

/// @brief Compare two messages field for field, so a failure names the field rather than the object.
void expectSame(const gr::Message& arrived, const gr::Message& sent) {
    using namespace boost::ut;
    expect(arrived.protocol == sent.protocol) << std::format("protocol '{}' rather than '{}'", arrived.protocol, sent.protocol);
    expect(arrived.cmd == sent.cmd) << "the command did not cross";
    expect(arrived.serviceName == sent.serviceName) << std::format("serviceName '{}' rather than '{}'", arrived.serviceName, sent.serviceName);
    expect(arrived.clientRequestID == sent.clientRequestID) << "the client request id did not cross";
    expect(arrived.endpoint == sent.endpoint) << std::format("endpoint '{}' rather than '{}'", arrived.endpoint, sent.endpoint);
    expect(arrived.rbac == sent.rbac) << "the RBAC field did not cross";
    expect(arrived.data.has_value() == sent.data.has_value()) << "one carries a payload and the other an error";
    if (arrived.data.has_value() && sent.data.has_value()) {
        expect(arrived.data.value() == sent.data.value()) << std::format("the payload arrived as {}", arrived.data.value());
    }
}

/// @brief Wait until @p ready holds, ending on the condition rather than on a duration.
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

/// @brief A source pumped the way the scheduler pumps it, with everything it emits collected.
struct Listener {
    gr::MsgPortIn            port{};
    std::vector<gr::Message> received{};

    void pump(MessagePacketSource& source) {
        source.processScheduledMessages();
        const auto available = port.streamReader().available();
        if (available == 0UZ) {
            return;
        }
        gr::ReaderSpanLike auto span = port.streamReader().get<gr::SpanReleasePolicy::ProcessAll>(available);
        received.insert(received.end(), span.begin(), span.end());
    }

    /// @brief Pump until @p wanted messages have been collected. False means the deadline won.
    [[nodiscard]] bool pumpUntil(MessagePacketSource& source, std::size_t wanted) {
        return waitFor([this, &source, wanted] {
            pump(source);
            return received.size() >= wanted;
        });
    }
};

/// @brief One four-frame message built here rather than by the sink, which is what proves the format.
struct RawPeer {
    zmq::context_t context{1};
    zmq::socket_t  socket;

    explicit RawPeer(const std::string& uri) : socket(context, zmq::socket_type::push) {
        socket.set(zmq::sockopt::sndtimeo, 5000);
        socket.set(zmq::sockopt::linger, 0);
        socket.bind(uri);
    }

    RawPeer(const RawPeer&)            = delete;
    RawPeer& operator=(const RawPeer&) = delete;

    /// @brief Publish @p message under @p sequence; @p items forges a payload this pair never sends.
    void send(const gr::Message& message, std::uint64_t sequence, std::uint32_t items = 0U) {
        const gr::property_map map      = gr::blocks::network::detail::messagepacket::encode(message, sequence);
        const std::string      metadata = gr::pmt::yaml::serialize(map);

        gr::network::EnvelopeHeader header;
        header.item_type                        = gr::network::kItemTypeCode<std::uint8_t>;
        header.item_size                        = 1U;
        header.item_count                       = items;
        header.payload_bytes                    = items;
        header.meta_bytes                       = static_cast<std::uint32_t>(metadata.size());
        const auto                      encoded = gr::network::encodeHeader(header);
        const std::vector<std::uint8_t> payload(items, 0U);

        std::ignore = socket.send(zmq::message_t(std::string_view("")), zmq::send_flags::sndmore);
        std::ignore = socket.send(zmq::message_t(encoded.data(), encoded.size()), zmq::send_flags::sndmore);
        std::ignore = socket.send(zmq::message_t(metadata.data(), metadata.size()), zmq::send_flags::sndmore);
        std::ignore = socket.send(zmq::message_t(payload.data(), payload.size()), zmq::send_flags::none);
    }
};

} // namespace

const boost::ut::suite<"MessagePacketSink"> messagePacketSinkTests = [] {
    using namespace boost::ut;

    "the settings that gate start() are checked before any socket opens"_test = [] {
        const Endpoint endpoint;
        const auto     refuses = [](gr::property_map settings, std::string_view what) {
            MessagePacketSink block(std::move(settings));
            expect(throws<gr::exception>([&block] { block.start(); })) << what;
        };
        refuses({{"endpoint", std::string("")}}, "an empty endpoint");
        refuses({{"endpoint", endpoint.uri}, {"pattern", std::string("sub")}}, "a pattern no sink runs");
        refuses({{"endpoint", endpoint.uri}, {"max_message_bytes", std::uint64_t{0ULL}}}, "a zero message bound");
    };

    // A message addressed to the sink itself is how a host application reads and changes the sink's own settings, so
    // it is answered here rather than put on the wire; everything else is the far half's business.
    "a message naming the sink is answered locally, not forwarded"_test = [] {
        const Endpoint endpoint;
        auto           sink = started<MessagePacketSink>({{"endpoint", endpoint.uri}, {"pattern", std::string("push")}});

        gr::Message mine;
        mine.cmd         = gr::message::Command::Get;
        mine.serviceName = sink->unique_name;
        mine.endpoint    = gr::block::property::kHeartbeat;
        mine.data        = gr::property_map{};

        const gr::Message theirs = richMessage("far/endpoint", 0ULL);
        const std::array  batch{mine, theirs};
        sink->processMessages(sink->msgIn, batch);
        sink->stop();

        const auto counted = sink->counters();
        expect(eq(counted.messagesKeptLocal, std::uint64_t{1ULL})) << "the sink forwarded a message addressed to itself";
        expect(eq(counted.messagesForwarded, std::uint64_t{1ULL})) << "the sink kept a message addressed elsewhere";
    };
};

const boost::ut::suite<"MessagePacketSource"> messagePacketSourceTests = [] {
    using namespace boost::ut;

    "the settings that gate start() are checked before any socket opens"_test = [] {
        const Endpoint endpoint;
        const auto     refuses = [](gr::property_map settings, std::string_view what) {
            MessagePacketSource block(std::move(settings));
            expect(throws<gr::exception>([&block] { block.start(); })) << what;
        };
        refuses({{"endpoint", std::string("")}, {"max_message_bytes", kBound}}, "an empty endpoint");
        refuses({{"endpoint", endpoint.uri}, {"max_message_bytes", kBound}, {"pattern", std::string("pub")}}, "a pattern no source runs");
        refuses({{"endpoint", endpoint.uri}}, "a zero message bound");
    };

    // The property the pair exists for: what one graph's message plane carried, the other's carries, field for field.
    "a message crosses two processes' worth of blocks unchanged"_test = [] {
        const Endpoint endpoint;
        auto           source = started<MessagePacketSource>({{"endpoint", endpoint.uri}, {"pattern", std::string("pull")}, {"max_message_bytes", kBound}});
        Listener       listener;
        expect(source->msgOut.connect(listener.port).has_value());

        auto              sink = started<MessagePacketSink>({{"endpoint", endpoint.uri}, {"pattern", std::string("push")}});
        const gr::Message sent = richMessage("settings/sample_rate", 42ULL);
        const std::array  batch{sent};
        sink->processMessages(sink->msgIn, batch);

        expect(listener.pumpUntil(*source, 1UZ)) << "the message never reached the far plane";
        sink->stop();
        source->stop();

        expect(eq(listener.received.size(), std::size_t{1UZ}));
        if (listener.received.size() == 1UZ) {
            expectSame(listener.received[0UZ], sent);
        }
        expect(eq(source->counters().messagesRefused, std::uint64_t{0ULL}));
        expect(eq(source->counters().gapsAnnounced, std::uint64_t{0ULL}));
    };

    // A lost Set leaves a block holding a setting nobody asked for, so the loss is a message of its own.
    "a lost message becomes one gap message naming the count and where"_test = [] {
        const Endpoint endpoint;
        auto           source = started<MessagePacketSource>({{"endpoint", endpoint.uri}, {"pattern", std::string("pull")}, {"max_message_bytes", kBound}});
        Listener       listener;
        expect(source->msgOut.connect(listener.port).has_value());

        RawPeer peer(endpoint.uri);
        peer.send(richMessage("first", 0ULL), 0ULL);
        peer.send(richMessage("third", 2ULL), 2ULL);

        expect(listener.pumpUntil(*source, 3UZ)) << "the gap message and the two that framed it did not all arrive";
        source->stop();

        expect(eq(listener.received.size(), std::size_t{3UZ}));
        if (listener.received.size() != 3UZ) {
            return;
        }
        expect(listener.received[0UZ].endpoint == std::string("first"));
        const gr::Message& gap = listener.received[1UZ];
        expect(gap.endpoint == std::string(gr::blocks::network::detail::messagepacket::kGapEndpoint)) << std::format("the gap was announced on '{}'", gap.endpoint);
        expect(gap.cmd == gr::message::Command::Notify);
        expect(gap.serviceName == source->unique_name) << "the gap message does not name the block that saw it";
        expect(gap.data.has_value()) << "the gap message carries no payload";
        if (gap.data.has_value()) {
            const auto lost     = gap.data.value().find(gr::property_map::key_type("messages_lost"));
            const auto sequence = gap.data.value().find(gr::property_map::key_type("sequence"));
            expect(lost != gap.data.value().end() && lost->second.get_if<std::uint64_t>() != nullptr);
            expect(sequence != gap.data.value().end() && sequence->second.get_if<std::uint64_t>() != nullptr);
            if (lost != gap.data.value().end() && lost->second.get_if<std::uint64_t>() != nullptr) {
                expect(eq(*lost->second.get_if<std::uint64_t>(), std::uint64_t{1ULL})) << "the gap's count is wrong";
            }
            if (sequence != gap.data.value().end() && sequence->second.get_if<std::uint64_t>() != nullptr) {
                expect(eq(*sequence->second.get_if<std::uint64_t>(), std::uint64_t{1ULL})) << "the gap does not name the first message it swallowed";
            }
        }
        expect(listener.received[2UZ].endpoint == std::string("third")) << "the message after the gap did not follow it";

        const auto counted = source->counters();
        expect(eq(counted.gapsAnnounced, std::uint64_t{1ULL}));
        expect(eq(counted.messagesLost, std::uint64_t{1ULL}));
        expect(eq(counted.messagesEmitted, std::uint64_t{2ULL})) << "the gap announcement is not one of the messages received";
    };

    // This pair's packets carry no payload items, so one that does was written by a peer of another kind.
    "a packet carrying payload items is refused and does not shift the stream"_test = [] {
        const Endpoint endpoint;
        auto           source = started<MessagePacketSource>({{"endpoint", endpoint.uri}, {"pattern", std::string("pull")}, {"max_message_bytes", kBound}});
        Listener       listener;
        expect(source->msgOut.connect(listener.port).has_value());

        RawPeer peer(endpoint.uri);
        peer.send(richMessage("forged", 0ULL), 0ULL, 8U);
        expect(waitFor([&source] { return source->counters().messagesRefused >= 1ULL; })) << "a payload-carrying packet was taken as a message";

        const gr::Message sound = richMessage("sound", 1ULL);
        peer.send(sound, 1ULL);
        expect(listener.pumpUntil(*source, 1UZ)) << "the message after the refusal never arrived";
        source->stop();

        expect(eq(listener.received.size(), std::size_t{1UZ}));
        if (listener.received.size() == 1UZ) {
            expectSame(listener.received[0UZ], sound);
        }
        expect(eq(source->counters().messagesRefused, std::uint64_t{1ULL}));
        expect(eq(source->counters().gapsAnnounced, std::uint64_t{0ULL})) << "a refused packet is not a gap; nothing was lost between what arrived";
    };

    // PUSH/PULL is the pattern for a control path that must see every message, and its whole claim is that it
    // discards nothing.
    "PUSH and PULL lose no message over a run"_test = [] {
        constexpr std::size_t kMessages = 200UZ;

        const Endpoint endpoint;
        auto           source = started<MessagePacketSource>({{"endpoint", endpoint.uri}, {"pattern", std::string("pull")}, {"max_message_bytes", kBound}});
        Listener       listener;
        expect(source->msgOut.connect(listener.port).has_value());

        auto                     sink = started<MessagePacketSink>({{"endpoint", endpoint.uri}, {"pattern", std::string("push")}});
        std::vector<gr::Message> sent;
        sent.reserve(kMessages);
        for (std::size_t i = 0UZ; i < kMessages; ++i) {
            sent.push_back(richMessage(std::format("endpoint/{}", i), static_cast<std::uint64_t>(i)));
        }
        sink->processMessages(sink->msgIn, sent);

        expect(listener.pumpUntil(*source, kMessages)) << std::format("only {} of {} messages crossed", listener.received.size(), kMessages);
        sink->stop();
        source->stop();

        expect(eq(listener.received.size(), kMessages));
        for (std::size_t i = 0UZ; i < std::min(listener.received.size(), kMessages); ++i) {
            expect(listener.received[i].endpoint == sent[i].endpoint) << std::format("message {} arrived as '{}'", i, listener.received[i].endpoint);
        }
        expect(eq(source->counters().gapsAnnounced, std::uint64_t{0ULL})) << "a lossless pattern lost a message";
        expect(eq(sink->counters().messagesForwarded, std::uint64_t{kMessages}));
    };
};

int main() { /* not needed for UT */ }
