#include <boost/ut.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/zeromq/ZmqPubSink.hpp>
#include <gnuradio-4.0/zeromq/ZmqPullSource.hpp>
#include <gnuradio-4.0/zeromq/ZmqPushSink.hpp>
#include <gnuradio-4.0/zeromq/ZmqRepSink.hpp>
#include <gnuradio-4.0/zeromq/ZmqReqSource.hpp>
#include <gnuradio-4.0/zeromq/ZmqSubSource.hpp>

using namespace boost::ut;
using namespace gr::blocks::zeromq;
namespace zd = gr::blocks::zeromq::detail;
namespace gr::blocks::zeromq::detail {
struct ZmqTransportTestAccess {
    static void          shutdown(ZmqSocketTransport& transport) { transport._context.shutdown(); }
    static zmq::socket_t peer(ZmqSocketTransport& transport, zmq::socket_type type) { return zmq::socket_t{transport._context, type}; }
};
} // namespace gr::blocks::zeromq::detail
namespace {
constexpr auto none = gr::SpanReleasePolicy::ProcessNone;

template<typename T>
T value(int n) {
    if constexpr (std::same_as<T, gr::pmt::Value>) {
        return gr::pmt::Value{static_cast<std::uint8_t>(n)};
    } else if constexpr (std::same_as<T, std::vector<float>>) {
        return {static_cast<float>(n), static_cast<float>(n + 10)};
    } else {
        return static_cast<T>(n);
    }
}

template<typename T>
std::vector<std::uint8_t> frame(int n) {
    auto bytes = zd::serialize_tag_header(10, {{10, gr::pmt::Value{"label"}, gr::pmt::Value{n}, {}}});
    auto v     = value<T>(n);
    if constexpr (std::same_as<T, gr::pmt::Value>) {
        auto payload = zd::serialize_pmt(v, PmtWireFormat::GR4_YAML_V1);
        bytes.insert(bytes.end(), payload.begin(), payload.end());
    } else if constexpr (std::same_as<T, std::vector<float>>) {
        zd::append_bytes(bytes, v.data(), v.size() * sizeof(float));
    } else {
        zd::append_bytes(bytes, &v, sizeof(v));
    }
    return bytes;
}
void send(zmq::socket_t& peer, const std::vector<std::uint8_t>& bytes) { expect(peer.send(zmq::buffer(bytes), zmq::send_flags::dontwait).has_value()); }

template<template<typename> typename Source, typename T>
void refusal_prefix() {
    Source<T> source;
    source.endpoint  = "inproc://refusal";
    source.bind      = true;
    source.linger    = 0;
    source.timeout   = 0;
    source.pass_tags = true;
    source.start();
    constexpr bool sub  = std::same_as<Source<T>, ZmqSubSource<T>>;
    auto           peer = zd::ZmqTransportTestAccess::peer(source._transport, sub ? zmq::socket_type::xpub : zmq::socket_type::push);
    peer.set(zmq::sockopt::linger, 0);
    peer.set(zmq::sockopt::rcvtimeo, 1000);
    peer.connect(source.last_endpoint());
    if constexpr (sub) {
        zmq::message_t subscription;
        const auto     deadline   = std::chrono::steady_clock::now() + std::chrono::seconds{2};
        bool           subscribed = false;
        while (!subscribed && std::chrono::steady_clock::now() < deadline) {
            std::ignore = source._transport.wait_readable(0);
            subscribed  = peer.recv(subscription, zmq::recv_flags::dontwait).has_value();
        }
        expect(subscribed >> fatal);
    }
    gr::PortIn<T> reader;
    expect(source.out.connect(reader).has_value());
    auto work = [&](std::size_t capacity) {
        auto output = source.out.template reserve<none>(capacity);
        expect(source.processBulk(output) == gr::work::Status::OK);
    };
    // inproc enqueues synchronously into the receiving pipe. All four messages
    // are queued before this single invocation; no sleep or scheduler race.
    send(peer, {0});
    send(peer, frame<T>(1));
    send(peer, {0});
    send(peer, frame<T>(2));
    work(4);
    expect(eq(source.receive_statistics().messages, std::uint64_t{4}));
    expect(eq(reader.streamReader().available(), 2UZ));
    if (reader.streamReader().available() == 2) {
        auto input = reader.template get<none>(2);
        expect(input[0] == value<T>(1));
        expect(input[1] == value<T>(2));
        if constexpr (std::same_as<T, gr::pmt::Value>) {
            expect(input[0].value_type() == value<T>(1).value_type());
            expect(input[1].value_type() == value<T>(2).value_type());
        }
        std::size_t count = 0;
        for (const auto& [index, tag] : input.tags()) {
            expect(eq(index, static_cast<std::ptrdiff_t>(count)));
            expect(tag.get().at("value") == gr::pmt::Value{static_cast<int>(count + 1)});
            ++count;
        }
        expect(eq(count, 2UZ));
        std::ignore = input.consume(2);
        input.consumeTags(2);
    }
    // Refusals consume the work budget, never output capacity. The valid frame
    // after eight refusals must remain for the next work call.
    for (std::size_t i = 0; i < zd::kMaxMessagesPerWork; ++i) {
        send(peer, {0});
    }
    send(peer, frame<T>(3));
    work(2);
    expect(eq(reader.streamReader().available(), 0UZ));
    expect(eq(source.receive_statistics().refused_messages, std::uint64_t{10}));
    work(2);
    expect(eq(reader.streamReader().available(), 1UZ));
    if (reader.streamReader().available() == 1) {
        auto input = reader.template get<none>(1);
        expect(input[0] == value<T>(3));
        std::ignore = input.consume(1);
    }
    source.stop();
}

template<template<typename> typename Sink, typename T>
void message_tags_retry(const std::string& key = {}, std::size_t tagged_item = 1) {
    Sink<T> sink;
    sink.endpoint      = "inproc://retry";
    sink.timeout       = 0;
    sink.linger        = 0;
    sink.hwm           = 2;
    sink.pass_tags     = true;
    constexpr bool pub = std::same_as<Sink<T>, ZmqPubSink<T>>;
    if constexpr (pub) {
        sink.drop_on_hwm = false;
        sink.key         = key;
    }
    sink.start();
    auto peer = zd::ZmqTransportTestAccess::peer(sink._transport, pub ? zmq::socket_type::sub : zmq::socket_type::pull);
    peer.set(zmq::sockopt::linger, 0);
    peer.set(zmq::sockopt::rcvhwm, 1);
    peer.set(zmq::sockopt::rcvtimeo, 1000);
    if constexpr (pub) {
        peer.set(zmq::sockopt::subscribe, key);
    }
    peer.connect(sink.last_endpoint());
    // Pump the inproc attach/subscription commands before starting the batch.
    expect(sink._transport.wait_writable(1000));
    if constexpr (pub) {
        expect(sink._transport.socket().get(zmq::sockopt::socket_type) == zmq::socket_type::pub);
        // A received probe proves the subscriber is established. Drain it fully.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{2};
        bool       ready    = false;
        while (!ready && std::chrono::steady_clock::now() < deadline) {
            auto guard  = sink._transport.acquire_operation_guard();
            std::ignore = sink._transport.socket().send(zmq::buffer(key), zmq::send_flags::dontwait);
            zmq::message_t probe;
            ready = peer.recv(probe, zmq::recv_flags::dontwait).has_value();
        }
        expect(ready >> fatal);
        zmq::message_t extra;
        while (peer.recv(extra, zmq::recv_flags::dontwait)) {
        }
    }
    gr::PortOut<T> writer;
    expect(writer.connect(sink.in).has_value());
    {
        auto input = writer.template reserve<none>(6);
        for (int i = 0; i < 6; ++i) {
            input[static_cast<std::size_t>(i)] = value<T>(i + 1);
        }
        input.publishTag(gr::property_map{{"key", "only"}, {"value", 42}}, tagged_item);
        input.publish(6);
    }
    std::size_t received = 0, tag_count = 0;
    bool        partial  = false;
    const auto  deadline = std::chrono::steady_clock::now() + std::chrono::seconds{2};
    while (received < 6 && std::chrono::steady_clock::now() < deadline) {
        const auto before = sink.in.streamReader().available();
        if (before) {
            {
                auto input = sink.in.template get<none>(before);
                expect(sink.processBulk(input) == gr::work::Status::OK);
                // Match scheduler tag consumption to the accepted input prefix.
                input.consumeTags(input.nRequestedSamplesToConsume());
            }
            const auto after = sink.in.streamReader().available();
            if (received == 0) {
                expect(after > 0 && after < before);
                // No draining: the exact refused item must stay in the GR4 buffer.
                {
                    auto input = sink.in.template get<none>(after);
                    expect(sink.processBulk(input) == gr::work::Status::OK);
                }
                expect(eq(sink.in.streamReader().available(), after));
                partial = true;
            }
        }
        zmq::message_t msg;
        while (peer.recv(msg, zmq::recv_flags::dontwait)) {
            if (!key.empty()) {
                expect(std::string_view(static_cast<const char*>(msg.data()), msg.size()) == key);
                expect(peer.recv(msg).has_value() >> fatal);
            }
            std::uint64_t                       offset = 0;
            std::vector<zd::ZmqTagHeaderRecord> tags;
            const auto                          consumed_bytes = zd::parse_tag_header(static_cast<const std::uint8_t*>(msg.data()), msg.size(), offset, tags);
            // Decode payload independently of the receiving blocks.
            if constexpr (std::same_as<T, gr::pmt::Value>) {
                expect(zd::deserialize_pmt(static_cast<const std::uint8_t*>(msg.data()) + consumed_bytes, msg.size() - consumed_bytes, PmtWireFormat::GR4_YAML_V1) == value<T>(static_cast<int>(received + 1)));
            } else {
                T v((msg.size() - consumed_bytes) / sizeof(float));
                std::memcpy(v.data(), static_cast<const std::uint8_t*>(msg.data()) + consumed_bytes, v.size() * sizeof(float));
                expect(v == value<T>(static_cast<int>(received + 1)));
            }
            for (const auto& tag : tags) {
                expect(eq(received, tagged_item));
                expect(eq(tag.offset, std::uint64_t{0}));
                expect(tag.value == gr::pmt::Value{42});
                ++tag_count;
            }
            ++received;
        }
    }
    expect(partial);
    expect(eq(received, 6UZ));
    expect(eq(tag_count, 1UZ));
    expect(eq(sink.in.streamReader().available(), 0UZ));
    expect(sink.send_statistics().refused_messages > 0);
    sink.stop();
}

template<typename T>
void req_refusal_recovery() {
    ZmqReqSource<T> source;
    source.endpoint  = "inproc://req-recovery";
    source.bind      = true;
    source.timeout   = 0;
    source.linger    = 0;
    source.pass_tags = true;
    source.start();
    auto peer = zd::ZmqTransportTestAccess::peer(source._transport, zmq::socket_type::rep);
    peer.set(zmq::sockopt::linger, 0);
    peer.set(zmq::sockopt::rcvtimeo, 1000);
    peer.connect(source.last_endpoint());
    gr::PortIn<T> reader;
    expect(source.out.connect(reader).has_value());
    int accepted = 0;
    for (const int n : {0, 0, 1, 0, 2}) {
        if (!source._req_pending) {
            const std::uint32_t count = 1;
            expect(source._transport.wait_writable(1000));
            expect(source._transport.socket().send(zmq::buffer(&count, sizeof(count))).has_value());
            source._req_pending       = true;
            source._req_pending_since = std::chrono::steady_clock::now();
        }
        zmq::message_t request;
        expect(peer.recv(request).has_value() >> fatal);
        send(peer, n == 0 ? std::vector<std::uint8_t>{0} : frame<T>(n));
        {
            auto output = source.out.template reserve<none>(1);
            expect(source.processBulk(output) == gr::work::Status::OK);
        }
        expect(eq(reader.streamReader().available(), n == 0 ? 0UZ : 1UZ));
        if (n && reader.streamReader().available()) {
            auto input = reader.template get<none>(1);
            expect(input[0] == value<T>(n));
            if constexpr (std::same_as<T, gr::pmt::Value>) {
                expect(input[0].value_type() == value<T>(n).value_type());
            }
            std::size_t tags = 0;
            for (const auto& [index, map] : input.tags()) {
                expect(eq(index, std::ptrdiff_t{0}));
                expect(map.get().at("value") == gr::pmt::Value{n});
                ++tags;
            }
            expect(eq(tags, 1UZ));
            std::ignore = input.consume(1);
            ++accepted;
        }
    }
    expect(eq(accepted, 2));
    expect(eq(source.receive_statistics().refused_messages, std::uint64_t{3}));
    source.stop();
}

template<typename Span, typename Error>
struct ThrowingTags : Span {
    explicit ThrowingTags(Span&& span) : Span(std::move(span)) {}
    void publishTag(const gr::property_map&, std::size_t) { throw Error{}; }
};
struct PublicationError : std::runtime_error {
    PublicationError() : std::runtime_error("injected publication error") {}
};

template<template<typename> typename Source, typename T, typename Error>
void publication_error() {
    Source<T> source;
    source.endpoint  = "inproc://exception";
    source.bind      = true;
    source.timeout   = 0;
    source.linger    = 0;
    source.pass_tags = true;
    source.start();
    constexpr bool req  = std::same_as<Source<T>, ZmqReqSource<T>>;
    constexpr bool sub  = std::same_as<Source<T>, ZmqSubSource<T>>;
    auto           peer = zd::ZmqTransportTestAccess::peer(source._transport, req ? zmq::socket_type::rep : sub ? zmq::socket_type::xpub : zmq::socket_type::push);
    peer.set(zmq::sockopt::linger, 0);
    peer.set(zmq::sockopt::rcvtimeo, 1000);
    peer.connect(source.last_endpoint());
    if constexpr (req) {
        const std::uint32_t count = 1;
        expect(source._transport.wait_writable(1000));
        expect(source._transport.socket().send(zmq::buffer(&count, sizeof(count))).has_value());
        source._req_pending = true;
        zmq::message_t request;
        expect(peer.recv(request).has_value() >> fatal);
    } else if constexpr (sub) {
        std::ignore = source._transport.wait_readable(0);
        zmq::message_t subscription;
        expect(peer.recv(subscription).has_value() >> fatal);
    }
    send(peer, frame<T>(1));
    gr::PortIn<T> reader;
    expect(source.out.connect(reader).has_value());
    bool propagated = false;
    {
        auto                                span = source.out.template reserve<none>(1);
        ThrowingTags<decltype(span), Error> output{std::move(span)};
        try {
            std::ignore = source.processBulk(output);
        } catch (const Error&) {
            propagated = true;
        }
    }
    expect(propagated);
    expect(eq(source.receive_statistics().refused_messages, std::uint64_t{0}));
    source.stop();
}

void rep_request_limit() {
    ZmqRepSink<float> sink;
    sink.endpoint = "tcp://127.0.0.1:0";
    sink.timeout  = 20;
    sink.linger   = 0;
    sink.start();
    expect(eq(sink._transport.socket().get(zmq::sockopt::maxmsgsize), std::int64_t{4096}));
    gr::PortOut<float> writer;
    expect(writer.connect(sink.in).has_value());
    {
        auto output = writer.reserve<none>(2);
        output[0]   = 11.f;
        output[1]   = 22.f;
        output.publish(2);
    }
    zmq::context_t context{1};
    auto           transact = [&](bool malformed, float expected) {
        zmq::socket_t peer{context, zmq::socket_type::req};
        peer.set(zmq::sockopt::linger, 0);
        peer.set(zmq::sockopt::rcvtimeo, 1000);
        peer.connect(sink.last_endpoint());
        const std::uint32_t count = 1;
        expect(peer.send(zmq::buffer(&count, malformed ? 3 : sizeof(count))).has_value());
        const auto     deadline = std::chrono::steady_clock::now() + std::chrono::seconds{2};
        zmq::message_t reply;
        bool           received = false;
        while (!received && std::chrono::steady_clock::now() < deadline) {
            {
                auto input = sink.in.get<none>(sink.in.streamReader().available());
                expect(sink.processBulk(input) == gr::work::Status::OK);
            }
            received = peer.recv(reply, zmq::recv_flags::dontwait).has_value();
        }
        expect(received >> fatal);
        expect(eq(reply.size(), malformed ? 0UZ : sizeof(float)));
        if (!malformed && reply.size() == sizeof(float)) {
            float v;
            std::memcpy(&v, reply.data(), sizeof(v));
            expect(eq(v, expected));
        }
    };
    transact(true, 0);
    expect(eq(sink.in.streamReader().available(), 2UZ));
    transact(false, 11);
    expect(eq(sink.in.streamReader().available(), 1UZ));
    {
        zmq::socket_t peer{context, zmq::socket_type::req};
        peer.set(zmq::sockopt::linger, 0);
        peer.set(zmq::sockopt::reconnect_ivl, -1);
        expect(eq(zmq_socket_monitor(peer.handle(), "inproc://oversize-monitor", ZMQ_EVENT_DISCONNECTED), 0));
        zmq::socket_t monitor{context, zmq::socket_type::pair};
        monitor.set(zmq::sockopt::rcvtimeo, 2000);
        monitor.connect("inproc://oversize-monitor");
        peer.connect(sink.last_endpoint());
        std::vector<std::uint8_t> oversized(8192, 0);
        expect(peer.send(zmq::buffer(oversized)).has_value());
        zmq::message_t event;
        expect(monitor.recv(event).has_value() >> fatal);
        std::uint16_t type = 0;
        if (event.size() >= sizeof(type)) {
            std::memcpy(&type, event.data(), sizeof(type));
        }
        expect(eq(type, static_cast<std::uint16_t>(ZMQ_EVENT_DISCONNECTED)));
        {
            auto input = sink.in.get<none>(1);
            expect(sink.processBulk(input) == gr::work::Status::OK);
        }
        expect(eq(sink.in.streamReader().available(), 1UZ));
    }
    // MAXMSGSIZE disconnects the offending connection below application decode.
    transact(false, 22);
    expect(eq(sink.in.streamReader().available(), 0UZ));
    sink.stop();
}
struct FailingResource : std::pmr::memory_resource {
    void* do_allocate(std::size_t, std::size_t) override { throw std::bad_alloc{}; }
    void  do_deallocate(void*, std::size_t, std::size_t) override {}
    bool  do_is_equal(const std::pmr::memory_resource& other) const noexcept override { return this == &other; }
};
struct DefaultResourceGuard {
    std::pmr::memory_resource* previous;
    explicit DefaultResourceGuard(std::pmr::memory_resource* resource) : previous(std::pmr::set_default_resource(resource)) {}
    ~DefaultResourceGuard() { std::pmr::set_default_resource(previous); }
};
void decode_allocation_error() {
    ZmqPullSource<gr::pmt::Value> source;
    source.endpoint = "inproc://allocation";
    source.bind     = true;
    source.timeout  = 0;
    source.linger   = 0;
    source.start();
    auto peer = zd::ZmqTransportTestAccess::peer(source._transport, zmq::socket_type::push);
    peer.connect(source.last_endpoint());
    send(peer, zd::serialize_pmt(gr::pmt::Value{"value requiring a map allocation"}, PmtWireFormat::GR4_YAML_V1));
    gr::PortIn<gr::pmt::Value> reader;
    expect(source.out.connect(reader).has_value());
    bool propagated = false;
    {
        auto            output = source.out.reserve<none>(1);
        FailingResource resource;
        {
            DefaultResourceGuard guard{&resource};
            try {
                std::ignore = source.processBulk(output);
            } catch (const std::bad_alloc&) {
                propagated = true;
            }
        }
    }
    expect(propagated);
    expect(eq(source.receive_statistics().refused_messages, std::uint64_t{0}));
    source.stop();
}
} // namespace
const suite remediation = [] {
    "Transport failure propagates instead of becoming a refusal"_test = [] {
        ZmqPullSource<float> source;
        source.endpoint = "inproc://transport-failure";
        source.bind     = true;
        source.linger   = 0;
        source.start();
        zd::ZmqTransportTestAccess::shutdown(source._transport);
        bool propagated = false;
        {
            auto output = source.out.reserve<gr::SpanReleasePolicy::ProcessNone>(1);
            try {
                std::ignore = source.processBulk(output);
            } catch (const zmq::error_t& error) {
                propagated = error.num() == ETERM;
            }
        }
        expect(propagated);
        expect(eq(source.receive_statistics().refused_messages, std::uint64_t{0}));
        source.stop();
    };
    "Decoder allocation failure propagates"_test                  = decode_allocation_error;
    "Peer-controlled lengths are rejected before allocation"_test = [] {
        const std::vector<std::uint8_t> huge_yaml{255, 255, 255, 255};
        auto                            huge_tags = zd::serialize_tag_header(0, {});
        std::fill(huge_tags.begin() + 11, huge_tags.end(), std::uint8_t{255});
        bool            yaml_refused = false, tags_refused = false;
        FailingResource resource;
        {
            DefaultResourceGuard guard{&resource};
            try {
                std::ignore = zd::deserialize_pmt(huge_yaml.data(), huge_yaml.size(), PmtWireFormat::GR4_YAML_V1);
            } catch (const std::runtime_error&) {
                yaml_refused = true;
            }
            std::uint64_t                       offset = 0;
            std::vector<zd::ZmqTagHeaderRecord> tags;
            try {
                std::ignore = zd::parse_tag_header(huge_tags.data(), huge_tags.size(), offset, tags);
            } catch (const std::runtime_error&) {
                tags_refused = true;
            }
        }
        expect(yaml_refused);
        expect(tags_refused);
    };
    "ZmqReqSource gr::pmt::Value propagates PublicationError"_test      = [] { publication_error<ZmqReqSource, gr::pmt::Value, PublicationError>(); };
    "ZmqReqSource gr::pmt::Value propagates std::bad_alloc"_test        = [] { publication_error<ZmqReqSource, gr::pmt::Value, std::bad_alloc>(); };
    "ZmqReqSource std::vector<float> propagates PublicationError"_test  = [] { publication_error<ZmqReqSource, std::vector<float>, PublicationError>(); };
    "ZmqReqSource std::vector<float> propagates std::bad_alloc"_test    = [] { publication_error<ZmqReqSource, std::vector<float>, std::bad_alloc>(); };
    "ZmqReqSource float propagates PublicationError"_test               = [] { publication_error<ZmqReqSource, float, PublicationError>(); };
    "ZmqReqSource float propagates std::bad_alloc"_test                 = [] { publication_error<ZmqReqSource, float, std::bad_alloc>(); };
    "ZmqSubSource gr::pmt::Value propagates PublicationError"_test      = [] { publication_error<ZmqSubSource, gr::pmt::Value, PublicationError>(); };
    "ZmqSubSource gr::pmt::Value propagates std::bad_alloc"_test        = [] { publication_error<ZmqSubSource, gr::pmt::Value, std::bad_alloc>(); };
    "ZmqSubSource std::vector<float> propagates PublicationError"_test  = [] { publication_error<ZmqSubSource, std::vector<float>, PublicationError>(); };
    "ZmqSubSource std::vector<float> propagates std::bad_alloc"_test    = [] { publication_error<ZmqSubSource, std::vector<float>, std::bad_alloc>(); };
    "ZmqSubSource float propagates PublicationError"_test               = [] { publication_error<ZmqSubSource, float, PublicationError>(); };
    "ZmqSubSource float propagates std::bad_alloc"_test                 = [] { publication_error<ZmqSubSource, float, std::bad_alloc>(); };
    "ZmqPullSource gr::pmt::Value propagates PublicationError"_test     = [] { publication_error<ZmqPullSource, gr::pmt::Value, PublicationError>(); };
    "ZmqPullSource gr::pmt::Value propagates std::bad_alloc"_test       = [] { publication_error<ZmqPullSource, gr::pmt::Value, std::bad_alloc>(); };
    "ZmqPullSource std::vector<float> propagates PublicationError"_test = [] { publication_error<ZmqPullSource, std::vector<float>, PublicationError>(); };
    "ZmqPullSource std::vector<float> propagates std::bad_alloc"_test   = [] { publication_error<ZmqPullSource, std::vector<float>, std::bad_alloc>(); };
    "ZmqPullSource float propagates PublicationError"_test              = [] { publication_error<ZmqPullSource, float, PublicationError>(); };
    "ZmqPullSource float propagates std::bad_alloc"_test                = [] { publication_error<ZmqPullSource, float, std::bad_alloc>(); };
    "REQ PMT refusal recovery"_test                                     = [] { req_refusal_recovery<gr::pmt::Value>(); };
    "REQ vector refusal recovery"_test                                  = [] { req_refusal_recovery<std::vector<float>>(); };
    "REQ scalar refusal recovery"_test                                  = [] { req_refusal_recovery<float>(); };
    "REP limits inbound requests before allocation"_test                = rep_request_limit;
    "Tag maps accept only the documented envelope"_test                 = [] {
        expect(!zd::tag_record_from_property_map(gr::property_map{{"native", 42}}));
        expect(!zd::tag_record_from_property_map(gr::property_map{{"key", "x"}}));
        auto tag = zd::tag_record_from_property_map(gr::property_map{{"key", "x"}, {"value", 42}, {"ignored", 99}});
        expect(tag.has_value() >> fatal);
        expect(tag->key == gr::pmt::Value{"x"});
        expect(tag->value == gr::pmt::Value{42});
        expect(tag->srcid == gr::pmt::Value{});
    };
    "Push PMT per-message tags and retry"_test = [] {
        message_tags_retry<ZmqPushSink, gr::pmt::Value>();
        message_tags_retry<ZmqPushSink, gr::pmt::Value>("", 4);
    };
    "Push vector per-message tags and retry"_test = [] {
        message_tags_retry<ZmqPushSink, std::vector<float>>();
        message_tags_retry<ZmqPushSink, std::vector<float>>("", 4);
    };
    "Pub PMT per-message tags and retry"_test = [] {
        message_tags_retry<ZmqPubSink, gr::pmt::Value>();
        message_tags_retry<ZmqPubSink, gr::pmt::Value>("", 4);
    };
    "Pub vector per-message tags and retry"_test = [] {
        message_tags_retry<ZmqPubSink, std::vector<float>>();
        message_tags_retry<ZmqPubSink, std::vector<float>>("", 4);
    };
    "Pub topic PMT per-message tags and retry"_test = [] { message_tags_retry<ZmqPubSink, gr::pmt::Value>("topic"); };
    "Pull PMT refusal prefix"_test                  = [] { refusal_prefix<ZmqPullSource, gr::pmt::Value>(); };
    "Sub PMT refusal prefix"_test                   = [] { refusal_prefix<ZmqSubSource, gr::pmt::Value>(); };
    "Pull vector refusal prefix"_test               = [] { refusal_prefix<ZmqPullSource, std::vector<float>>(); };
    "Sub vector refusal prefix"_test                = [] { refusal_prefix<ZmqSubSource, std::vector<float>>(); };
    "Pull scalar refusal prefix"_test               = [] { refusal_prefix<ZmqPullSource, float>(); };
    "Sub scalar refusal prefix"_test                = [] { refusal_prefix<ZmqSubSource, float>(); };
};
int main() { return boost::ut::cfg<boost::ut::override>.run(); }
