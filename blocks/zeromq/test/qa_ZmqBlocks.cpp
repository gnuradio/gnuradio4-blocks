#include <boost/ut.hpp>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <complex>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <functional>
#include <future>
#include <limits>
#include <map>
#include <memory_resource>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/LifeCycle.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>
#include <gnuradio-4.0/zeromq/ZmqPubSink.hpp>
#include <gnuradio-4.0/zeromq/ZmqPullSource.hpp>
#include <gnuradio-4.0/zeromq/ZmqPushSink.hpp>
#include <gnuradio-4.0/zeromq/ZmqRepSink.hpp>
#include <gnuradio-4.0/zeromq/ZmqReqSource.hpp>
#include <gnuradio-4.0/zeromq/ZmqSubSource.hpp>
#include <gnuradio-4.0/zeromq/detail/ZmqPmtCodec.hpp>
#include <gnuradio-4.0/zeromq/detail/ZmqTagHeaders.hpp>
#include <gnuradio-4.0/zeromq/detail/pmt_legacy_codec.hpp>
#include <zmq.hpp>

using namespace boost::ut;
namespace zdetail = gr::blocks::zeromq::detail;

using PmtBlock = gr::blocks::zeromq::ZmqPushSink<gr::pmt::Value>;
static_assert(gr::refl::reflectable<PmtBlock>);
static_assert(std::same_as<gr::refl::data_member_type<PmtBlock, "pmt_wire_format">, gr::blocks::zeromq::PmtWireFormat>);

namespace {

using namespace std::chrono_literals;

gr::property_map make_props(std::initializer_list<std::pair<std::string_view, gr::pmt::Value>> init) {
    gr::property_map out;
    auto*            mr = out.get_allocator().resource();
    for (const auto& [key, value] : init) {
        out.emplace(gr::pmt::Value::Map::value_type{std::pmr::string(key.data(), key.size(), mr), value});
    }
    return out;
}

std::string endpoint_for(int offset) {
    std::ignore = offset;
    return "tcp://127.0.0.1:0";
}

class TestEndpoint {
public:
    TestEndpoint(std::string endpoint) : _resolve([endpoint = std::move(endpoint)] { return endpoint; }) {}

    template<typename Block>
    static TestEndpoint bound_by(Block& block) {
        return TestEndpoint([&block] {
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{2};
            while (std::chrono::steady_clock::now() < deadline) {
                try {
                    const auto endpoint = block.last_endpoint();
                    if (!endpoint.empty() && !endpoint.ends_with(":0")) {
                        return endpoint;
                    }
                } catch (...) {
                }
                std::this_thread::sleep_for(std::chrono::milliseconds{1});
            }
            throw std::runtime_error("timed out waiting for a ZeroMQ block to bind");
        });
    }

    [[nodiscard]] std::string resolve() const { return _resolve(); }

private:
    explicit TestEndpoint(std::function<std::string()> resolve) : _resolve(std::move(resolve)) {}
    std::function<std::string()> _resolve;
};

bool wait_for_subscription(zmq::socket_t& socket) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{2};
    while (std::chrono::steady_clock::now() < deadline) {
        zmq::pollitem_t items[] = {{static_cast<void*>(socket), 0, ZMQ_POLLIN, 0}};
        zmq::poll(items, 1, std::chrono::milliseconds{10});
        if ((items[0].revents & ZMQ_POLLIN) != 0) {
            zmq::message_t subscription;
            return bool(socket.recv(subscription));
        }
    }
    return false;
}

bool endpoint_can_be_bound(const std::string& endpoint, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        try {
            zmq::context_t context{1};
            zmq::socket_t  socket{context, zmq::socket_type::pair};
            socket.set(zmq::sockopt::linger, 0);
            socket.bind(endpoint);
            socket.unbind(endpoint);
            return true;
        } catch (const zmq::error_t& error) {
            if (error.num() != EADDRINUSE) {
                throw;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    return false;
}

template<typename T>
std::vector<std::uint8_t> to_bytes(const std::vector<T>& payload) {
    std::vector<std::uint8_t> bytes(payload.size() * sizeof(T));
    if (!payload.empty()) {
        std::memcpy(bytes.data(), payload.data(), bytes.size());
    }
    return bytes;
}

std::thread spawn_push_sender(TestEndpoint endpoint, std::vector<std::uint8_t> bytes, std::chrono::milliseconds delay = std::chrono::milliseconds{0}) {
    return std::thread([endpoint = std::move(endpoint), bytes = std::vector<std::vector<std::uint8_t>>{std::move(bytes)}, delay] {
        const auto resolved_endpoint = endpoint.resolve();
        std::this_thread::sleep_for(delay);
        zmq::context_t ctx{1};
        zmq::socket_t  socket{ctx, zmq::socket_type::push};
        socket.connect(resolved_endpoint);
        std::this_thread::sleep_for(std::chrono::milliseconds{10});

        for (const auto& message_bytes : bytes) {
            zmq::message_t msg(message_bytes.size());
            if (!message_bytes.empty()) {
                std::memcpy(msg.data(), message_bytes.data(), message_bytes.size());
            }
            [[maybe_unused]] const bool ok = bool(socket.send(msg, zmq::send_flags::none));
        }
    });
}

std::thread spawn_push_sender(TestEndpoint endpoint, std::vector<std::vector<std::uint8_t>> messages, std::chrono::milliseconds delay = std::chrono::milliseconds{0}) {
    return std::thread([endpoint = std::move(endpoint), messages = std::move(messages), delay] {
        const auto resolved_endpoint = endpoint.resolve();
        std::this_thread::sleep_for(delay);
        zmq::context_t ctx{1};
        zmq::socket_t  socket{ctx, zmq::socket_type::push};
        socket.connect(resolved_endpoint);
        std::this_thread::sleep_for(std::chrono::milliseconds{10});

        for (const auto& message_bytes : messages) {
            zmq::message_t msg(message_bytes.size());
            if (!message_bytes.empty()) {
                std::memcpy(msg.data(), message_bytes.data(), message_bytes.size());
            }
            [[maybe_unused]] const bool ok = bool(socket.send(msg, zmq::send_flags::none));
        }
    });
}

std::thread spawn_push_multipart_sender(TestEndpoint endpoint, std::vector<std::vector<std::vector<std::uint8_t>>> messages, std::chrono::milliseconds delay = std::chrono::milliseconds{0}) {
    return std::thread([endpoint = std::move(endpoint), messages = std::move(messages), delay] {
        const auto resolved_endpoint = endpoint.resolve();
        std::this_thread::sleep_for(delay);
        zmq::context_t ctx{1};
        zmq::socket_t  socket{ctx, zmq::socket_type::push};
        socket.connect(resolved_endpoint);
        std::this_thread::sleep_for(std::chrono::milliseconds{10});

        for (const auto& frames : messages) {
            for (std::size_t i = 0; i < frames.size(); ++i) {
                const auto&    bytes = frames[i];
                zmq::message_t frame(bytes.size());
                if (!bytes.empty()) {
                    std::memcpy(frame.data(), bytes.data(), bytes.size());
                }
                const auto                  flags = i + 1 < frames.size() ? zmq::send_flags::sndmore : zmq::send_flags::none;
                [[maybe_unused]] const bool ok    = bool(socket.send(frame, flags));
            }
        }
    });
}

std::future<std::vector<std::vector<std::uint8_t>>> spawn_pull_receiver(TestEndpoint endpoint, std::size_t min_messages, std::chrono::milliseconds timeout = std::chrono::milliseconds{2000}) {
    return std::async(std::launch::async, [endpoint = std::move(endpoint), min_messages, timeout] {
        const auto     resolved_endpoint = endpoint.resolve();
        zmq::context_t ctx{1};
        zmq::socket_t  socket{ctx, zmq::socket_type::pull};
        socket.connect(resolved_endpoint);

        std::vector<std::vector<std::uint8_t>> messages;
        const auto                             deadline = std::chrono::steady_clock::now() + timeout;
        while (messages.size() < min_messages && std::chrono::steady_clock::now() < deadline) {
            zmq::pollitem_t items[] = {{static_cast<void*>(socket), 0, ZMQ_POLLIN, 0}};
            zmq::poll(&items[0], 1, std::chrono::milliseconds{25});
            if ((items[0].revents & ZMQ_POLLIN) == 0) {
                continue;
            }

            zmq::message_t msg;
            if (socket.recv(msg)) {
                const auto* first = static_cast<const std::uint8_t*>(msg.data());
                messages.emplace_back(first, first + msg.size());
            }
        }
        return messages;
    });
}

std::future<std::vector<std::vector<std::uint8_t>>> spawn_sub_receiver(TestEndpoint endpoint, std::string subscribe_key, std::chrono::milliseconds timeout = std::chrono::milliseconds{2000}) {
    return std::async(std::launch::async, [endpoint = std::move(endpoint), subscribe_key = std::move(subscribe_key), timeout] {
        const auto     resolved_endpoint = endpoint.resolve();
        zmq::context_t ctx{1};
        zmq::socket_t  socket{ctx, zmq::socket_type::sub};
        socket.set(zmq::sockopt::subscribe, subscribe_key);
        socket.connect(resolved_endpoint);

        std::vector<std::vector<std::uint8_t>> frames;
        const auto                             deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            zmq::pollitem_t items[] = {{static_cast<void*>(socket), 0, ZMQ_POLLIN, 0}};
            zmq::poll(&items[0], 1, std::chrono::milliseconds{25});
            if ((items[0].revents & ZMQ_POLLIN) == 0) {
                continue;
            }

            while (true) {
                zmq::message_t msg;
                if (!socket.recv(msg)) {
                    break;
                }
                const auto* first = static_cast<const std::uint8_t*>(msg.data());
                frames.emplace_back(first, first + msg.size());
                if (!socket.get(zmq::sockopt::rcvmore)) {
                    return frames;
                }
            }
        }
        return frames;
    });
}

std::thread spawn_pub_sender(TestEndpoint endpoint, std::string topic, std::vector<std::uint8_t> payload, std::chrono::milliseconds delay = std::chrono::milliseconds{0}) {
    return std::thread([endpoint = std::move(endpoint), topic = std::move(topic), payload = std::move(payload), delay] {
        const auto resolved_endpoint = endpoint.resolve();
        std::this_thread::sleep_for(delay);
        zmq::context_t ctx{1};
        zmq::socket_t  socket{ctx, zmq::socket_type::xpub};
        socket.set(zmq::sockopt::linger, 0);
        socket.connect(resolved_endpoint);

        if (!wait_for_subscription(socket)) {
            return;
        }
        if (!topic.empty()) {
            zmq::message_t key_msg(topic.size());
            std::memcpy(key_msg.data(), topic.data(), topic.size());
            [[maybe_unused]] const bool ok_key = bool(socket.send(key_msg, zmq::send_flags::sndmore));
        }
        zmq::message_t msg(payload.size());
        if (!payload.empty()) {
            std::memcpy(msg.data(), payload.data(), payload.size());
        }
        [[maybe_unused]] const bool ok_payload = bool(socket.send(msg, zmq::send_flags::none));
    });
}

[[maybe_unused]] std::thread spawn_pub_sender(TestEndpoint endpoint, std::string topic, std::vector<std::vector<std::uint8_t>> payloads, std::chrono::milliseconds delay = std::chrono::milliseconds{0}) {
    return std::thread([endpoint = std::move(endpoint), topic = std::move(topic), payloads = std::move(payloads), delay] {
        const auto resolved_endpoint = endpoint.resolve();
        std::this_thread::sleep_for(delay);
        zmq::context_t ctx{1};
        zmq::socket_t  socket{ctx, zmq::socket_type::xpub};
        socket.set(zmq::sockopt::linger, 0);
        socket.connect(resolved_endpoint);

        if (!wait_for_subscription(socket)) {
            return;
        }
        for (const auto& payload : payloads) {
            if (!topic.empty()) {
                zmq::message_t key_msg(topic.size());
                std::memcpy(key_msg.data(), topic.data(), topic.size());
                [[maybe_unused]] const bool ok_key = bool(socket.send(key_msg, zmq::send_flags::sndmore));
            }
            zmq::message_t msg(payload.size());
            if (!payload.empty()) {
                std::memcpy(msg.data(), payload.data(), payload.size());
            }
            [[maybe_unused]] const bool ok_payload = bool(socket.send(msg, zmq::send_flags::none));
        }
    });
}

std::thread spawn_pub_multipart_sender(TestEndpoint endpoint, std::string topic, std::vector<std::vector<std::vector<std::uint8_t>>> payloads, std::chrono::milliseconds delay = std::chrono::milliseconds{0}) {
    return std::thread([endpoint = std::move(endpoint), topic = std::move(topic), payloads = std::move(payloads), delay] {
        const auto resolved_endpoint = endpoint.resolve();
        std::this_thread::sleep_for(delay);
        zmq::context_t ctx{1};
        zmq::socket_t  socket{ctx, zmq::socket_type::xpub};
        socket.set(zmq::sockopt::linger, 0);
        socket.connect(resolved_endpoint);

        if (!wait_for_subscription(socket)) {
            return;
        }
        for (const auto& payload : payloads) {
            std::vector<std::vector<std::uint8_t>> frames;
            if (!topic.empty()) {
                frames.push_back(std::vector<std::uint8_t>(topic.begin(), topic.end()));
            }
            frames.insert(frames.end(), payload.begin(), payload.end());
            for (std::size_t i = 0; i < frames.size(); ++i) {
                zmq::message_t frame(frames[i].size());
                if (!frames[i].empty()) {
                    std::memcpy(frame.data(), frames[i].data(), frames[i].size());
                }
                const auto                  flags = i + 1 < frames.size() ? zmq::send_flags::sndmore : zmq::send_flags::none;
                [[maybe_unused]] const bool ok    = bool(socket.send(frame, flags));
            }
        }
    });
}

std::future<uint32_t> spawn_rep_responder(TestEndpoint endpoint, std::vector<std::uint8_t> payload, std::chrono::milliseconds delay = std::chrono::milliseconds{0}) {
    return std::async(std::launch::async, [endpoint = std::move(endpoint), payload = std::move(payload), delay] {
        const auto resolved_endpoint = endpoint.resolve();
        std::this_thread::sleep_for(delay);
        zmq::context_t ctx{1};
        zmq::socket_t  socket{ctx, zmq::socket_type::rep};
        socket.connect(resolved_endpoint);

        const auto deadline      = std::chrono::steady_clock::now() + std::chrono::milliseconds{4000};
        uint32_t   request_count = 0;
        while (std::chrono::steady_clock::now() < deadline) {
            zmq::pollitem_t items[] = {{static_cast<void*>(socket), 0, ZMQ_POLLIN, 0}};
            zmq::poll(&items[0], 1, std::chrono::milliseconds{25});
            if ((items[0].revents & ZMQ_POLLIN) == 0) {
                continue;
            }

            zmq::message_t request;
            if (!socket.recv(request)) {
                continue;
            }
            if (request.size() >= sizeof(uint32_t)) {
                std::memcpy(&request_count, request.data(), sizeof(uint32_t));
            }

            zmq::message_t reply(payload.size());
            if (!payload.empty()) {
                std::memcpy(reply.data(), payload.data(), payload.size());
            }
            [[maybe_unused]] const bool ok = bool(socket.send(reply, zmq::send_flags::none));
            return request_count;
        }
        return request_count;
    });
}

std::future<std::vector<std::vector<std::uint8_t>>> spawn_req_client(TestEndpoint endpoint, std::vector<uint32_t> request_counts, std::chrono::milliseconds delay = std::chrono::milliseconds{0}) {
    return std::async(std::launch::async, [endpoint = std::move(endpoint), request_counts = std::move(request_counts), delay] {
        const auto resolved_endpoint = endpoint.resolve();
        std::this_thread::sleep_for(delay);
        zmq::context_t ctx{1};
        zmq::socket_t  socket{ctx, zmq::socket_type::req};
        socket.connect(resolved_endpoint);

        std::vector<std::vector<std::uint8_t>> replies;
        const auto                             deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{4000};
        for (const auto request_count : request_counts) {
            zmq::message_t request(sizeof(request_count));
            std::memcpy(request.data(), &request_count, sizeof(request_count));
            if (!socket.send(request, zmq::send_flags::none)) {
                break;
            }

            while (std::chrono::steady_clock::now() < deadline) {
                zmq::pollitem_t items[] = {{static_cast<void*>(socket), 0, ZMQ_POLLIN, 0}};
                zmq::poll(&items[0], 1, std::chrono::milliseconds{25});
                if ((items[0].revents & ZMQ_POLLIN) == 0) {
                    continue;
                }

                zmq::message_t reply;
                if (socket.recv(reply)) {
                    const auto* first = static_cast<const std::uint8_t*>(reply.data());
                    replies.emplace_back(first, first + reply.size());
                }
                break;
            }
        }
        return replies;
    });
}

template<typename SchedulerT>
bool run_with_timeout(SchedulerT& sched, std::chrono::milliseconds timeout) {
    std::mutex                                    mu;
    std::condition_variable                       cv;
    std::optional<std::expected<void, gr::Error>> result;
    bool                                          done = false;

    std::thread runner([&] {
        auto res = sched.runAndWait();
        {
            std::lock_guard lk(mu);
            result = std::move(res);
            done   = true;
        }
        cv.notify_one();
    });

    {
        std::unique_lock lk(mu);
        if (!cv.wait_for(lk, timeout, [&] { return done; })) {
            std::ignore = sched.changeStateTo(gr::lifecycle::State::REQUESTED_STOP);
            lk.unlock();
            if (runner.joinable()) {
                runner.join();
            }
            return false;
        }
    }

    if (runner.joinable()) {
        runner.join();
    }
    return result.has_value() && result->has_value();
}

template<typename SchedulerT, typename Predicate>
bool run_until_then_stop(SchedulerT& sched, Predicate&& done, std::chrono::milliseconds timeout, bool require_successful_result = true) {
    std::optional<std::expected<void, gr::Error>> result;

    std::thread runner([&] { result = sched.runAndWait(); });

    const auto deadline       = std::chrono::steady_clock::now() + timeout;
    bool       predicate_done = false;
    while (std::chrono::steady_clock::now() < deadline) {
        if (done()) {
            predicate_done = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }

    std::ignore = sched.changeStateTo(gr::lifecycle::State::REQUESTED_STOP);

    if (runner.joinable()) {
        runner.join();
    }

    return predicate_done && result.has_value() && (!require_successful_result || result->has_value());
}

template<typename ZmqSource>
bool source_stop_is_prompt(int endpoint_offset) {
    gr::Graph fg;
    auto&     source = fg.emplaceBlock<ZmqSource>(make_props({
        {"endpoint", gr::pmt::Value(endpoint_for(endpoint_offset))},
        {"timeout", gr::pmt::Value(2000)},
        {"bind", gr::pmt::Value(true)},
        {"linger", gr::pmt::Value(0)},
    }));
    auto&     sink   = fg.emplaceBlock<gr::testing::CountingSink<float>>();
    if (!fg.connect<"out", "in">(source, sink).has_value()) {
        return false;
    }
    gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
    if (!sched.exchange(std::move(fg)).has_value()) {
        return false;
    }
    const auto started = std::chrono::steady_clock::now();
    std::ignore        = run_until_then_stop(sched, [] { return false; }, 150ms);
    return std::chrono::steady_clock::now() - started < 600ms;
}

template<typename T>
struct DelayedCountingSource : gr::Block<DelayedCountingSource<T>> {
    gr::PortOut<T> out;
    gr::Size_t     n_samples_max    = 0;
    gr::Size_t     count            = 0;
    gr::Size_t     startup_delay_ms = 50;
    bool           started          = false;

    GR_MAKE_REFLECTABLE(DelayedCountingSource, out, n_samples_max, count, startup_delay_ms);

    [[nodiscard]] T processOne() noexcept {
        if (!started) {
            started = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(startup_delay_ms));
        }
        ++count;
        if (n_samples_max > 0 && count >= n_samples_max) {
            this->requestStop();
        }
        return static_cast<T>(count);
    }
};

template<typename T>
struct AtomicCountingSource : gr::Block<AtomicCountingSource<T>> {
    gr::PortOut<T> out;
    gr::Size_t     count = 0;

    GR_MAKE_REFLECTABLE(AtomicCountingSource, out, count);

    [[nodiscard]] T processOne() noexcept {
        const auto value = std::atomic_ref(count).fetch_add(1, std::memory_order_acq_rel) + 1;
        return static_cast<T>(value);
    }
};

template<typename T>
struct ConstantSource : gr::Block<ConstantSource<T>> {
    gr::PortOut<T> out;
    T              value{};
    bool           emitted          = false;
    gr::Size_t     startup_delay_ms = 50;

    GR_MAKE_REFLECTABLE(ConstantSource, out, value, startup_delay_ms);

    [[nodiscard]] T processOne() {
        if (emitted) {
            this->requestStop();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(startup_delay_ms));
        emitted = true;
        return value;
    }
};

template<typename T>
struct RepeatingSource : gr::Block<RepeatingSource<T>> {
    gr::PortOut<T> out;
    T              value{};
    gr::Size_t     startup_delay_ms = 0;
    bool           started          = false;

    GR_MAKE_REFLECTABLE(RepeatingSource, out, value, startup_delay_ms);

    [[nodiscard]] T processOne() {
        if (!started) {
            started = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(startup_delay_ms));
        }
        return value;
    }
};

template<typename ZmqSink>
bool sink_stop_is_prompt(int endpoint_offset) {
    gr::Graph fg;
    auto&     source = fg.emplaceBlock<RepeatingSource<float>>();
    source.value     = 1.f;
    auto& sink       = fg.emplaceBlock<ZmqSink>(make_props({
        {"endpoint", gr::pmt::Value(endpoint_for(endpoint_offset))},
        {"timeout", gr::pmt::Value(2000)},
        {"bind", gr::pmt::Value(true)},
        {"linger", gr::pmt::Value(0)},
    }));
    if (!fg.connect<"out", "in">(source, sink).has_value()) {
        return false;
    }
    gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
    if (!sched.exchange(std::move(fg)).has_value()) {
        return false;
    }
    const auto started = std::chrono::steady_clock::now();
    std::ignore        = run_until_then_stop(sched, [] { return false; }, 150ms);
    return std::chrono::steady_clock::now() - started < 600ms;
}

template<typename T>
struct FiniteHoldingSource : gr::Block<FiniteHoldingSource<T>> {
    gr::PortOut<T> out;
    std::vector<T> values;
    std::size_t    emitted = 0;

    GR_MAKE_REFLECTABLE(FiniteHoldingSource, out, values);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& output) {
        const auto n = std::min(output.size(), values.size() - emitted);
        std::copy_n(values.begin() + static_cast<std::ptrdiff_t>(emitted), n, output.begin());
        emitted += n;
        output.publish(n);
        return gr::work::Status::OK;
    }
};

template<typename T>
struct DelayedDefaultSource : gr::Block<DelayedDefaultSource<T>> {
    gr::PortOut<T> out;
    gr::Size_t     startup_delay_ms = 0;
    bool           started          = false;

    GR_MAKE_REFLECTABLE(DelayedDefaultSource, out, startup_delay_ms);

    [[nodiscard]] T processOne() {
        if (!started) {
            started = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(startup_delay_ms));
        }
        this->requestStop();
        return T{};
    }
};

template<typename T>
struct VectorSink : gr::Block<VectorSink<T>> {
    gr::PortIn<std::vector<T>> in;
    gr::Size_t                 n_samples_max = 0;
    gr::Size_t                 count         = 0;
    gr::Size_t                 last_size     = 0;

    GR_MAKE_REFLECTABLE(VectorSink, in, n_samples_max, count, last_size);

    void processOne(const std::vector<T>& v) noexcept {
        ++count;
        last_size = static_cast<gr::Size_t>(v.size());
        if (n_samples_max > 0 && count >= n_samples_max) {
            this->requestStop();
        }
    }
};

template<typename T>
struct RecordingScalarSink : gr::Block<RecordingScalarSink<T>> {
    gr::PortIn<T>  in;
    gr::Size_t     n_samples_max = 0;
    std::vector<T> received;

    GR_MAKE_REFLECTABLE(RecordingScalarSink, in, n_samples_max);

    void processOne(const T& value) {
        received.push_back(value);
        if (n_samples_max > 0 && received.size() >= n_samples_max) {
            this->requestStop();
        }
    }
};

struct PmtSink : gr::Block<PmtSink> {
    gr::PortIn<gr::pmt::Value> in;
    gr::Size_t                 n_samples_max = 0;
    gr::Size_t                 count         = 0;

    GR_MAKE_REFLECTABLE(PmtSink, in, n_samples_max, count);

    void processOne(const gr::pmt::Value&) noexcept {
        ++count;
        if (n_samples_max > 0 && count >= n_samples_max) {
            this->requestStop();
        }
    }
};

template<typename T>
struct SequenceSource : gr::Block<SequenceSource<T>> {
    gr::PortOut<T> out;
    std::vector<T> values;
    std::size_t    index            = 0;
    gr::Size_t     startup_delay_ms = 50;
    bool           started          = false;

    GR_MAKE_REFLECTABLE(SequenceSource, out, values, startup_delay_ms);

    [[nodiscard]] T processOne() {
        if (!started) {
            started = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(startup_delay_ms));
        }
        if (index >= values.size()) {
            this->requestStop();
            return T{};
        }

        auto value = values[index++];
        if (index >= values.size()) {
            this->requestStop();
        }
        return value;
    }
};

struct RecordingPmtSink : gr::Block<RecordingPmtSink> {
    gr::PortIn<gr::pmt::Value>  in;
    gr::Size_t                  n_samples_max = 0;
    std::vector<gr::pmt::Value> received;

    GR_MAKE_REFLECTABLE(RecordingPmtSink, in, n_samples_max);

    void processOne(const gr::pmt::Value& value) {
        received.push_back(value);
        if (n_samples_max > 0 && received.size() >= n_samples_max) {
            this->requestStop();
        }
    }
};

template<typename T>
struct TaggedSequenceSource : gr::Block<TaggedSequenceSource<T>> {
    gr::PortOut<T>                                        out;
    std::vector<T>                                        values;
    std::vector<std::pair<std::size_t, gr::property_map>> tags;
    std::size_t                                           index            = 0;
    gr::Size_t                                            startup_delay_ms = 50;
    bool                                                  started          = false;

    GR_MAKE_REFLECTABLE(TaggedSequenceSource, out, values, startup_delay_ms);

    [[nodiscard]] T processOne() {
        if (!started) {
            started = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(startup_delay_ms));
        }
        if (index >= values.size()) {
            this->requestStop();
            return T{};
        }

        for (const auto& [tag_index, tag_map] : tags) {
            if (tag_index == index) {
                this->publishTag(tag_map, 0UZ);
            }
        }

        auto value = values[index++];
        if (index >= values.size()) {
            this->requestStop();
        }
        return value;
    }
};

template<typename T>
struct TaggedBulkSource : gr::Block<TaggedBulkSource<T>> {
    gr::PortOut<T>                                        out;
    std::vector<T>                                        values;
    std::vector<std::pair<std::size_t, gr::property_map>> tags;
    std::size_t                                           emitted          = 0;
    gr::Size_t                                            startup_delay_ms = 50;
    bool                                                  started          = false;

    GR_MAKE_REFLECTABLE(TaggedBulkSource, out, values, startup_delay_ms);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& output) {
        if (!started) {
            started = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(startup_delay_ms));
        }

        if (emitted >= values.size()) {
            output.publish(0UZ);
            return gr::work::Status::DONE;
        }

        const auto available = output.size();
        const auto remaining = values.size() - emitted;
        const auto n         = std::min(available, remaining);
        if (n == 0) {
            return gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        for (std::size_t i = 0; i < n; ++i) {
            output[i] = values[emitted + i];
        }
        for (const auto& [tag_index, tag_map] : tags) {
            if (tag_index >= emitted && tag_index < emitted + n) {
                output.publishTag(tag_map, tag_index - emitted);
            }
        }
        emitted += n;
        output.publish(n);
        return emitted >= values.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

struct RecordingTaggedSink : gr::Block<RecordingTaggedSink> {
    gr::PortIn<float>                                     in;
    gr::Size_t                                            n_samples_max = 0;
    std::vector<float>                                    received;
    std::vector<std::pair<std::size_t, gr::property_map>> tags;
    std::size_t                                           consumed_total = 0;

    GR_MAKE_REFLECTABLE(RecordingTaggedSink, in, n_samples_max);

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& input) {
        const auto base_offset = consumed_total;
        for (const auto& value : input) {
            received.push_back(value);
        }
        for (const auto& [relIndex, tagMapRef] : input.tags()) {
            if (relIndex >= 0) {
                tags.emplace_back(base_offset + static_cast<std::size_t>(relIndex), tagMapRef.get());
            }
        }
        consumed_total += input.size();
        if (n_samples_max > 0 && received.size() >= n_samples_max) {
            this->requestStop();
        }
        return gr::work::Status::OK;
    }
};

static gr::property_map wire_tag(std::string_view key, gr::pmt::Value value, gr::pmt::Value srcid = gr::pmt::Value{}) { return make_props({{"key", gr::pmt::Value(std::string(key))}, {"value", std::move(value)}, {"srcid", std::move(srcid)}}); }

std::vector<gr::pmt::Value> make_pmt_fixtures() {
    gr::pmt::Value::Map metadata{std::pmr::get_default_resource()};
    metadata.emplace("scheme", gr::pmt::Value("gr4"));
    metadata.emplace("version", gr::pmt::Value(int32_t{4}));

    gr::pmt::Value::Map nested{std::pmr::get_default_resource()};
    nested.emplace("title", gr::pmt::Value("telemetry"));
    nested.emplace("samples", gr::pmt::Value(gr::Tensor<float>(gr::data_from, std::vector<float>{1.f, 2.f, 3.f, 4.f})));
    nested.emplace("meta", gr::pmt::Value(std::move(metadata)));

    std::vector<gr::pmt::Value> items;
    items.emplace_back(false);
    items.emplace_back(true);
    items.emplace_back(int8_t{-7});
    items.emplace_back(int16_t{-1234});
    items.emplace_back(int32_t{123456});
    items.emplace_back(int64_t{249387429783478});
    items.emplace_back(uint8_t{201});
    items.emplace_back(uint16_t{54321});
    items.emplace_back(uint32_t{3456789012U});
    items.emplace_back(uint64_t{9876543210123456789ULL});
    items.emplace_back(float{3.25f});
    items.emplace_back(double{456.789});
    items.emplace_back(std::complex<float>{1.25f, -2.5f});
    items.emplace_back(std::complex<double>{123.456, -789.321});
    items.emplace_back(std::string{"example"});
    items.emplace_back(gr::Tensor<uint8_t>(gr::data_from, std::vector<uint8_t>{1, 2, 3, 4}));
    items.emplace_back(gr::Tensor<int8_t>(gr::data_from, std::vector<int8_t>{-1, -2, -3, -4}));
    items.emplace_back(gr::Tensor<uint16_t>(gr::data_from, std::vector<uint16_t>{1000, 2000, 3000}));
    items.emplace_back(gr::Tensor<int16_t>(gr::data_from, std::vector<int16_t>{-1000, -2000, 3000}));
    items.emplace_back(gr::Tensor<uint32_t>(gr::data_from, std::vector<uint32_t>{10, 20, 30}));
    items.emplace_back(gr::Tensor<int32_t>(gr::data_from, std::vector<int32_t>{-10, 20, -30}));
    items.emplace_back(gr::Tensor<uint64_t>(gr::data_from, std::vector<uint64_t>{10000000000ULL, 20000000000ULL}));
    items.emplace_back(gr::Tensor<int64_t>(gr::data_from, std::vector<int64_t>{-10000000000LL, 20000000000LL}));
    items.emplace_back(gr::Tensor<float>(gr::data_from, std::vector<float>{3.5f, 4.5f, 5.5f}));
    items.emplace_back(gr::Tensor<double>(gr::data_from, std::vector<double>{6.5, 7.5, 8.5}));
    items.emplace_back(gr::Tensor<std::complex<float>>(gr::data_from, std::vector<std::complex<float>>{
                                                                          {1.5f, -2.5f},
                                                                          {3.5f, -4.5f},
                                                                      }));
    items.emplace_back(gr::Tensor<std::complex<double>>(gr::data_from, std::vector<std::complex<double>>{
                                                                           {1.25, -2.75},
                                                                           {3.125, -4.875},
                                                                       }));
    items.emplace_back(std::move(nested));
    items.emplace_back(gr::Tensor<gr::pmt::Value>(gr::data_from, std::vector<gr::pmt::Value>{gr::pmt::Value(int32_t{123}), gr::pmt::Value(456.789)}));
    items.emplace_back(gr::Tensor<gr::pmt::Value>(gr::data_from, std::vector<gr::pmt::Value>{
                                                                     gr::pmt::Value(int32_t{1}),
                                                                     gr::pmt::Value(gr::Tensor<gr::pmt::Value>(gr::data_from,
                                                                         std::vector<gr::pmt::Value>{
                                                                             gr::pmt::Value(int32_t{2}),
                                                                             gr::pmt::Value(gr::Tensor<gr::pmt::Value>(gr::data_from, std::vector<gr::pmt::Value>{gr::pmt::Value(int32_t{3}), gr::pmt::Value{}})),
                                                                         })),
                                                                 }));
    items.emplace_back(gr::Tensor<gr::pmt::Value>(gr::data_from, std::vector<gr::pmt::Value>{gr::pmt::Value(int32_t{7}), gr::pmt::Value("nested"), gr::pmt::Value(gr::Tensor<float>(gr::data_from, std::vector<float>{9.f, 10.f}))}));
    return items;
}

std::thread spawn_rep_sequence_responder(TestEndpoint endpoint, std::vector<std::vector<std::uint8_t>> payloads, std::chrono::milliseconds delay = std::chrono::milliseconds{0}) {
    return std::thread([endpoint = std::move(endpoint), payloads = std::move(payloads), delay] {
        const auto resolved_endpoint = endpoint.resolve();
        std::this_thread::sleep_for(delay);
        zmq::context_t ctx{1};
        zmq::socket_t  socket{ctx, zmq::socket_type::rep};
        socket.connect(resolved_endpoint);

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{4000};
        for (const auto& payload : payloads) {
            while (std::chrono::steady_clock::now() < deadline) {
                zmq::pollitem_t items[] = {{static_cast<void*>(socket), 0, ZMQ_POLLIN, 0}};
                zmq::poll(&items[0], 1, std::chrono::milliseconds{25});
                if ((items[0].revents & ZMQ_POLLIN) == 0) {
                    continue;
                }

                zmq::message_t request;
                if (!socket.recv(request)) {
                    break;
                }

                zmq::message_t reply(payload.size());
                if (!payload.empty()) {
                    std::memcpy(reply.data(), payload.data(), payload.size());
                }
                [[maybe_unused]] const bool ok = bool(socket.send(reply, zmq::send_flags::none));
                break;
            }
        }
    });
}

std::thread spawn_rep_multipart_responder(TestEndpoint endpoint, std::vector<std::vector<std::vector<std::uint8_t>>> replies, std::chrono::milliseconds delay = std::chrono::milliseconds{0}) {
    return std::thread([endpoint = std::move(endpoint), replies = std::move(replies), delay] {
        const auto resolved_endpoint = endpoint.resolve();
        std::this_thread::sleep_for(delay);
        zmq::context_t ctx{1};
        zmq::socket_t  socket{ctx, zmq::socket_type::rep};
        socket.connect(resolved_endpoint);

        for (const auto& reply_frames : replies) {
            zmq::pollitem_t items[] = {{static_cast<void*>(socket), 0, ZMQ_POLLIN, 0}};
            zmq::poll(&items[0], 1, std::chrono::milliseconds{2000});
            if ((items[0].revents & ZMQ_POLLIN) == 0) {
                return;
            }
            zmq::message_t request;
            if (!socket.recv(request)) {
                return;
            }
            for (std::size_t i = 0; i < reply_frames.size(); ++i) {
                const auto&    bytes = reply_frames[i];
                zmq::message_t frame(bytes.size());
                if (!bytes.empty()) {
                    std::memcpy(frame.data(), bytes.data(), bytes.size());
                }
                const auto flags = i + 1 < reply_frames.size() ? zmq::send_flags::sndmore : zmq::send_flags::none;
                if (!socket.send(frame, flags)) {
                    return;
                }
            }
        }
    });
}

} // namespace

const suite ZmqBlocksTests = [] {
    "Pull source receives scalar raw payload"_test = [] {
        gr::Graph fg;
        using T                        = float;
        constexpr gr::Size_t n_samples = 16;
        const auto           endpoint  = endpoint_for(0);

        auto& pull = fg.emplaceBlock<gr::blocks::zeromq::ZmqPullSource<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
        }));
        auto& sink = fg.emplaceBlock<gr::testing::CountingSink<T>>(make_props({
            {"n_samples_max", gr::pmt::Value(n_samples)},
        }));

        expect(fg.connect<"out", "in">(pull, sink).has_value());

        auto sender = spawn_push_sender(TestEndpoint::bound_by(pull), to_bytes(std::vector<T>{1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f, 10.f, 11.f, 12.f, 13.f, 14.f, 15.f, 16.f}));

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.count >= n_samples; }, std::chrono::milliseconds(2000)));
        expect(eq(sink.count, n_samples));

        if (sender.joinable()) {
            sender.join();
        }
    };

    "Pull source receives vector<complex<float>> messages"_test = [] {
        gr::Graph fg;
        using T                          = std::complex<float>;
        constexpr gr::Size_t n_messages  = 4;
        constexpr gr::Size_t payload_len = 16;
        const auto           endpoint    = endpoint_for(1);

        auto& pull = fg.emplaceBlock<gr::blocks::zeromq::ZmqPullSource<std::vector<T>>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
        }));
        auto& sink = fg.emplaceBlock<VectorSink<T>>(make_props({
            {"n_samples_max", gr::pmt::Value(n_messages)},
        }));

        expect(fg.connect<"out", "in">(pull, sink).has_value());

        std::vector<T> payload(payload_len);
        for (gr::Size_t i = 0; i < payload_len; ++i) {
            payload[i] = T{static_cast<float>(i + 1), static_cast<float>(i + 2)};
        }
        std::vector<std::vector<std::uint8_t>> messages;
        for (gr::Size_t i = 0; i < n_messages; ++i) {
            messages.push_back(to_bytes(payload));
        }
        auto sender = spawn_push_sender(TestEndpoint::bound_by(pull), std::move(messages));

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.count >= n_messages; }, std::chrono::milliseconds(2000)));
        expect(eq(sink.count, n_messages));
        expect(eq(sink.last_size, payload_len));

        if (sender.joinable()) {
            sender.join();
        }
    };

    "Pull source receives pmt::Value messages"_test = [] {
        gr::Graph            fg;
        constexpr gr::Size_t n_messages = 4;
        const auto           endpoint   = endpoint_for(2);

        auto& pull = fg.emplaceBlock<gr::blocks::zeromq::ZmqPullSource<gr::pmt::Value>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
        }));
        auto& sink = fg.emplaceBlock<PmtSink>(make_props({
            {"n_samples_max", gr::pmt::Value(n_messages)},
        }));

        expect(fg.connect<"out", "in">(pull, sink).has_value());

        std::vector<float>                     vec{1.0f, 2.0f, 3.0f, 4.0f};
        auto                                   serialized = zdetail::serialize_pmt(gr::pmt::Value(gr::Tensor<float>(gr::data_from, vec)), gr::blocks::zeromq::PmtWireFormat::GR4_YAML_V1);
        std::vector<std::vector<std::uint8_t>> messages(n_messages, serialized);
        auto                                   sender = spawn_push_sender(TestEndpoint::bound_by(pull), std::move(messages));

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.count >= n_messages; }, std::chrono::milliseconds(2000)));
        expect(eq(sink.count, n_messages));

        if (sender.joinable()) {
            sender.join();
        }
    };

    "Pull source times out without inbound data"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(3);

        auto& source = fg.emplaceBlock<gr::blocks::zeromq::ZmqPullSource<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
        }));
        auto& sink   = fg.emplaceBlock<gr::testing::CountingSink<float>>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(1))},
        }));

        expect(fg.connect<"out", "in">(source, sink).has_value());

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(!run_with_timeout(sched, std::chrono::milliseconds(300)));
    };

    "Pull source stops with timeout above poll slice"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(40);
        auto&      source   = fg.emplaceBlock<gr::blocks::zeromq::ZmqPullSource<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(250)},
            {"bind", gr::pmt::Value(true)},
        }));
        auto&      sink     = fg.emplaceBlock<RecordingScalarSink<float>>();
        expect(fg.connect<"out", "in">(source, sink).has_value());

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        const auto started = std::chrono::steady_clock::now();
        expect(!run_until_then_stop(sched, [] { return false; }, std::chrono::milliseconds{150}));
        expect(std::chrono::steady_clock::now() - started < std::chrono::seconds{1});
    };

    "Push sink sends scalar stream payload"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(4);

        auto& source   = fg.emplaceBlock<DelayedCountingSource<float>>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(0))},
            {"startup_delay_ms", gr::pmt::Value(static_cast<gr::Size_t>(50))},
        }));
        auto& push     = fg.emplaceBlock<gr::blocks::zeromq::ZmqPushSink<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
        }));
        auto  receiver = spawn_pull_receiver(TestEndpoint::bound_by(push), 1);

        expect(fg.connect<"out", "in">(source, push).has_value());

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return receiver.wait_for(std::chrono::milliseconds{0}) == std::future_status::ready; }, std::chrono::milliseconds(2000)));
        expect(source.count >= static_cast<gr::Size_t>(1));
        const auto stats = push.send_statistics();
        expect(stats.messages >= std::uint64_t{1});
        expect(stats.items >= std::uint64_t{1});
        expect(eq(stats.dropped_messages, std::uint64_t{0}));

        auto messages = receiver.get();
        expect(eq(messages.size(), static_cast<std::size_t>(1)));
        if (!messages.empty()) {
            expect(messages.front().size() >= sizeof(float));
            expect(eq(messages.front().size() % sizeof(float), static_cast<std::size_t>(0)));
        }
    };

    "Pull source flushes oversized raw message across spans"_test = [] {
        gr::Graph fg;
        using T             = float;
        const auto endpoint = endpoint_for(5);

        auto& pull = fg.emplaceBlock<gr::blocks::zeromq::ZmqPullSource<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
        }));
        auto& sink = fg.emplaceBlock<gr::testing::CountingSink<T>>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(16))},
        }));

        expect(fg.connect<"out", "in">(pull, sink).has_value());

        auto sender = spawn_push_sender(TestEndpoint::bound_by(pull), to_bytes(std::vector<T>{1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f, 10.f, 11.f, 12.f, 13.f, 14.f, 15.f, 16.f}));

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.count >= static_cast<gr::Size_t>(16); }, std::chrono::milliseconds(2000)));
        expect(eq(sink.count, static_cast<gr::Size_t>(16)));

        if (sender.joinable()) {
            sender.join();
        }
    };

    "Pull source discards many extra multipart frames"_test = [] {
        gr::Graph fg;
        using T             = float;
        const auto endpoint = endpoint_for(41);
        auto&      pull     = fg.emplaceBlock<gr::blocks::zeromq::ZmqPullSource<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
        }));
        auto&      sink     = fg.emplaceBlock<RecordingScalarSink<T>>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(1))},
        }));
        expect(fg.connect<"out", "in">(pull, sink).has_value());
        std::vector<std::vector<std::uint8_t>> extra_frames;
        extra_frames.reserve(128);
        for (std::size_t i = 0; i < 128; ++i) {
            extra_frames.push_back(to_bytes(std::vector<T>{static_cast<T>(i)}));
        }
        auto sender = spawn_push_multipart_sender(TestEndpoint::bound_by(pull), {std::move(extra_frames), {to_bytes(std::vector<T>{4.f})}});

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.received.size() >= 1; }, std::chrono::milliseconds(2000)));
        expect(eq(sink.received.size(), static_cast<std::size_t>(1)));
        if (!sink.received.empty()) {
            expect(eq(sink.received[0], 4.f));
        }
        sender.join();
    };

    "Pull source refuses an invalid raw payload and receives the next message"_test = [] {
        gr::Graph fg;
        using T             = float;
        const auto endpoint = endpoint_for(6);

        auto& pull = fg.emplaceBlock<gr::blocks::zeromq::ZmqPullSource<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
        }));
        auto& sink = fg.emplaceBlock<RecordingScalarSink<T>>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(1))},
        }));

        expect(fg.connect<"out", "in">(pull, sink).has_value());

        auto sender = spawn_push_sender(TestEndpoint::bound_by(pull), std::vector<std::vector<std::uint8_t>>{
                                                                          {0x01, 0x02, 0x03, 0x04, 0x05, 0x06},
                                                                          to_bytes(std::vector<T>{42.f}),
                                                                      });

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.received.size() >= 1; }, std::chrono::milliseconds(2000)));
        expect(sink.received == std::vector<T>{42.f});
        const auto stats = pull.receive_statistics();
        expect(eq(stats.messages, std::uint64_t{2}));
        expect(eq(stats.items, std::uint64_t{1}));
        expect(eq(stats.refused_messages, std::uint64_t{1}));

        if (sender.joinable()) {
            sender.join();
        }
    };

    "Pull source refuses malformed PMT and receives the next message"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(51);
        auto&      pull     = fg.emplaceBlock<gr::blocks::zeromq::ZmqPullSource<gr::pmt::Value>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"pmt_wire_format", gr::pmt::Value("GR3")},
        }));
        auto&      sink     = fg.emplaceBlock<PmtSink>(make_props({{"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(1))}}));
        expect(fg.connect<"out", "in">(pull, sink).has_value());

        auto                                                                 sender = spawn_push_sender(TestEndpoint::bound_by(pull), std::vector<std::vector<std::uint8_t>>{
                                                                          {0xFF},
                                                                          legacy_pmt::serialize_to_legacy(gr::pmt::Value(int32_t{42})),
                                                                      });
        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.count >= 1; }, std::chrono::milliseconds(2000)));
        const auto stats = pull.receive_statistics();
        expect(eq(stats.messages, std::uint64_t{2}));
        expect(eq(stats.items, std::uint64_t{1}));
        expect(eq(stats.refused_messages, std::uint64_t{1}));
        sender.join();
    };

    "Pub sink emits keyed multipart raw payload"_test = [] {
        gr::Graph         fg;
        const auto        endpoint = endpoint_for(7);
        const std::string key      = "telemetry.";

        auto& source   = fg.emplaceBlock<DelayedCountingSource<float>>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(3))},
            {"startup_delay_ms", gr::pmt::Value(static_cast<gr::Size_t>(1000))},
        }));
        auto& pub      = fg.emplaceBlock<gr::blocks::zeromq::ZmqPubSink<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"key", gr::pmt::Value(key)},
        }));
        auto  receiver = spawn_sub_receiver(TestEndpoint::bound_by(pub), key);

        expect(fg.connect<"out", "in">(source, pub).has_value());

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        const bool completed = run_until_then_stop(sched, [&] { return receiver.wait_for(std::chrono::milliseconds{0}) == std::future_status::ready; }, std::chrono::milliseconds(2000));
        expect(completed);
        if (completed) {
            auto frames = receiver.get();
            expect(eq(frames.size(), static_cast<std::size_t>(2)));
            if (frames.size() == 2) {
                expect(eq(std::string_view(reinterpret_cast<const char*>(frames[0].data()), frames[0].size()), key));
                expect(eq(frames[1].size() % sizeof(float), static_cast<std::size_t>(0)));
            }
        }

        expect(source.count >= static_cast<gr::Size_t>(1));
        const auto stats = pub.send_statistics();
        expect(stats.messages >= std::uint64_t{1});
        expect(stats.items >= std::uint64_t{1});
        expect(eq(stats.refused_messages, std::uint64_t{0}));
    };

    "Sub source receives keyed raw payload"_test = [] {
        gr::Graph fg;
        using T                    = float;
        const auto        endpoint = endpoint_for(8);
        const std::string key      = "telemetry.";

        auto& sub  = fg.emplaceBlock<gr::blocks::zeromq::ZmqSubSource<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"key", gr::pmt::Value(key)},
        }));
        auto& sink = fg.emplaceBlock<gr::testing::CountingSink<T>>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(3))},
        }));

        expect(fg.connect<"out", "in">(sub, sink).has_value());

        auto sender = spawn_pub_sender(TestEndpoint::bound_by(sub), key, to_bytes(std::vector<T>{1.f, 2.f, 3.f}));

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.count >= static_cast<gr::Size_t>(3); }, std::chrono::milliseconds(2000)));
        expect(eq(sink.count, static_cast<gr::Size_t>(3)));

        if (sender.joinable()) {
            sender.join();
        }
    };

    "Sub source refuses an invalid raw payload and receives the next message"_test = [] {
        gr::Graph fg;
        using T                    = float;
        const auto        endpoint = endpoint_for(52);
        const std::string key      = "telemetry.";
        auto&             sub      = fg.emplaceBlock<gr::blocks::zeromq::ZmqSubSource<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"key", gr::pmt::Value(key)},
        }));
        auto&             sink     = fg.emplaceBlock<RecordingScalarSink<T>>(make_props({{"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(1))}}));
        expect(fg.connect<"out", "in">(sub, sink).has_value());

        auto                                                                 sender = spawn_pub_sender(TestEndpoint::bound_by(sub), key,
                                                                            std::vector<std::vector<std::uint8_t>>{
                {0x01, 0x02, 0x03, 0x04, 0x05, 0x06},
                to_bytes(std::vector<T>{43.f}),
            });
        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.received.size() >= 1; }, std::chrono::milliseconds(2000)));
        expect(!sink.received.empty());
        if (!sink.received.empty()) {
            expect(eq(sink.received.front(), 43.f));
        }
        const auto stats = sub.receive_statistics();
        expect(stats.messages >= std::uint64_t{2});
        expect(eq(stats.items, std::uint64_t{1}));
        expect(stats.refused_messages >= std::uint64_t{1});
        sender.join();
    };

    "Sub source refuses malformed PMT and receives the next message"_test = [] {
        gr::Graph         fg;
        const auto        endpoint = endpoint_for(53);
        const std::string key      = "pmt.";
        auto&             sub      = fg.emplaceBlock<gr::blocks::zeromq::ZmqSubSource<gr::pmt::Value>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"key", gr::pmt::Value(key)},
            {"pmt_wire_format", gr::pmt::Value("GR3")},
        }));
        auto&             sink     = fg.emplaceBlock<PmtSink>(make_props({{"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(1))}}));
        expect(fg.connect<"out", "in">(sub, sink).has_value());

        auto                                                                 sender = spawn_pub_sender(TestEndpoint::bound_by(sub), key,
                                                                            std::vector<std::vector<std::uint8_t>>{
                {0xFF},
                legacy_pmt::serialize_to_legacy(gr::pmt::Value(int32_t{44})),
            });
        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.count >= 1; }, std::chrono::milliseconds(2000)));
        const auto stats = sub.receive_statistics();
        expect(stats.messages >= std::uint64_t{2});
        expect(eq(stats.items, std::uint64_t{1}));
        expect(stats.refused_messages >= std::uint64_t{1});
        sender.join();
    };

    "Sub source discards extra multipart frames"_test = [] {
        gr::Graph fg;
        using T                    = float;
        const auto        endpoint = endpoint_for(42);
        const std::string key      = "telemetry";
        auto&             sub      = fg.emplaceBlock<gr::blocks::zeromq::ZmqSubSource<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"key", gr::pmt::Value(key)},
        }));
        auto&             sink     = fg.emplaceBlock<RecordingScalarSink<T>>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(1))},
        }));
        expect(fg.connect<"out", "in">(sub, sink).has_value());
        auto sender = spawn_pub_multipart_sender(TestEndpoint::bound_by(sub), key, {{to_bytes(std::vector<T>{1.f}), to_bytes(std::vector<T>{2.f})}, {to_bytes(std::vector<T>{4.f})}});

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.received.size() >= 1; }, std::chrono::milliseconds(2000)));
        expect(eq(sink.received.size(), static_cast<std::size_t>(1)));
        if (!sink.received.empty()) {
            expect(eq(sink.received[0], 4.f));
        }
        sender.join();
    };

    "Sub source subscribes to topic prefix"_test = [] {
        gr::Graph fg;
        using T                         = float;
        const auto        endpoint      = endpoint_for(9);
        const std::string subscribe_key = "telemetry.";
        const std::string published_key = "telemetry.stream";

        auto& sub  = fg.emplaceBlock<gr::blocks::zeromq::ZmqSubSource<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"key", gr::pmt::Value(subscribe_key)},
        }));
        auto& sink = fg.emplaceBlock<gr::testing::CountingSink<T>>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(2))},
        }));

        expect(fg.connect<"out", "in">(sub, sink).has_value());

        auto sender = spawn_pub_sender(TestEndpoint::bound_by(sub), published_key, to_bytes(std::vector<T>{4.f, 5.f}));

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.count >= static_cast<gr::Size_t>(2); }, std::chrono::milliseconds(2000)));
        expect(eq(sink.count, static_cast<gr::Size_t>(2)));

        if (sender.joinable()) {
            sender.join();
        }
    };

    "Sub source rejects wrong topic"_test = [] {
        gr::Graph fg;
        using T             = float;
        const auto endpoint = endpoint_for(10);

        auto& sub  = fg.emplaceBlock<gr::blocks::zeromq::ZmqSubSource<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"key", gr::pmt::Value(std::string("telemetry."))},
        }));
        auto& sink = fg.emplaceBlock<gr::testing::CountingSink<T>>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(1))},
        }));

        expect(fg.connect<"out", "in">(sub, sink).has_value());

        auto sender = spawn_pub_sender(TestEndpoint::bound_by(sub), std::string("wrong."), to_bytes(std::vector<T>{6.f}));

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(!run_with_timeout(sched, std::chrono::milliseconds(300)));
        expect(eq(sink.count, static_cast<gr::Size_t>(0)));

        if (sender.joinable()) {
            sender.join();
        }
    };

    "Sub source with empty key receives all topics"_test = [] {
        gr::Graph fg;
        using T             = float;
        const auto endpoint = endpoint_for(11);

        auto& sub  = fg.emplaceBlock<gr::blocks::zeromq::ZmqSubSource<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"key", gr::pmt::Value(std::string())},
        }));
        auto& sink = fg.emplaceBlock<gr::testing::CountingSink<T>>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(2))},
        }));

        expect(fg.connect<"out", "in">(sub, sink).has_value());

        auto sender = spawn_pub_sender(TestEndpoint::bound_by(sub), std::string("telemetry.stream"), to_bytes(std::vector<T>{7.f, 8.f}));

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.count >= static_cast<gr::Size_t>(2); }, std::chrono::milliseconds(2000)));
        expect(eq(sink.count, static_cast<gr::Size_t>(2)));

        if (sender.joinable()) {
            sender.join();
        }
    };

    "Req source receives raw payloads"_test = [] {
        gr::Graph fg;
        using T             = float;
        const auto endpoint = endpoint_for(14);

        auto& source = fg.emplaceBlock<gr::blocks::zeromq::ZmqReqSource<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(100)},
            {"bind", gr::pmt::Value(true)},
        }));
        auto& sink   = fg.emplaceBlock<gr::testing::CountingSink<T>>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(5))},
        }));

        expect(fg.connect<"out", "in">(source, sink).has_value());

        auto responder = spawn_rep_responder(TestEndpoint::bound_by(source), to_bytes(std::vector<T>{1.f, 2.f, 3.f, 4.f, 5.f}));

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.count >= static_cast<gr::Size_t>(5); }, std::chrono::milliseconds(2000)));
        expect(eq(sink.count, static_cast<gr::Size_t>(5)));

        expect(responder.valid());
        if (responder.valid()) {
            expect(responder.get() >= 1U);
        }
    };

    "Req source refuses an invalid raw reply and requests the next one"_test = [] {
        gr::Graph fg;
        using T             = float;
        const auto endpoint = endpoint_for(54);
        auto&      source   = fg.emplaceBlock<gr::blocks::zeromq::ZmqReqSource<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(100)},
            {"bind", gr::pmt::Value(true)},
        }));
        auto&      sink     = fg.emplaceBlock<RecordingScalarSink<T>>(make_props({{"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(1))}}));
        expect(fg.connect<"out", "in">(source, sink).has_value());

        auto                                                                 responder = spawn_rep_sequence_responder(TestEndpoint::bound_by(source), {
                                                                                          {0x01, 0x02, 0x03, 0x04, 0x05, 0x06},
                                                                                          to_bytes(std::vector<T>{45.f}),
                                                                                      });
        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.received.size() >= 1; }, std::chrono::milliseconds(2000)));
        expect(sink.received == std::vector<T>{45.f});
        const auto stats = source.receive_statistics();
        expect(eq(stats.messages, std::uint64_t{2}));
        expect(eq(stats.items, std::uint64_t{1}));
        expect(eq(stats.refused_messages, std::uint64_t{1}));
        responder.join();
    };

    "Req source refuses malformed PMT and requests the next one"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(55);
        auto&      source   = fg.emplaceBlock<gr::blocks::zeromq::ZmqReqSource<gr::pmt::Value>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(100)},
            {"bind", gr::pmt::Value(true)},
            {"pmt_wire_format", gr::pmt::Value("GR3")},
        }));
        auto&      sink     = fg.emplaceBlock<PmtSink>(make_props({{"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(1))}}));
        expect(fg.connect<"out", "in">(source, sink).has_value());

        auto                                                                 responder = spawn_rep_sequence_responder(TestEndpoint::bound_by(source), {
                                                                                          {0xFF},
                                                                                          legacy_pmt::serialize_to_legacy(gr::pmt::Value(int32_t{46})),
                                                                                      });
        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.count >= 1; }, std::chrono::milliseconds(2000)));
        const auto stats = source.receive_statistics();
        expect(eq(stats.messages, std::uint64_t{2}));
        expect(eq(stats.items, std::uint64_t{1}));
        expect(eq(stats.refused_messages, std::uint64_t{1}));
        responder.join();
    };

    "Req source discards extra multipart frames"_test = [] {
        gr::Graph fg;
        using T             = float;
        const auto endpoint = endpoint_for(44);
        auto&      source   = fg.emplaceBlock<gr::blocks::zeromq::ZmqReqSource<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
        }));
        auto&      sink     = fg.emplaceBlock<RecordingScalarSink<T>>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(1))},
        }));
        expect(fg.connect<"out", "in">(source, sink).has_value());
        auto responder = spawn_rep_multipart_responder(TestEndpoint::bound_by(source), {{to_bytes(std::vector<T>{1.f}), to_bytes(std::vector<T>{2.f})}, {to_bytes(std::vector<T>{4.f})}});

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.received.size() >= 1; }, std::chrono::milliseconds(2000)));
        expect(eq(sink.received.size(), static_cast<std::size_t>(1)));
        if (!sink.received.empty()) {
            expect(eq(sink.received[0], 4.f));
        }
        responder.join();
    };

    "Push sink emits legacy PMT bytes for supported fixtures"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(19);
        const auto fixtures = make_pmt_fixtures();

        auto& source            = fg.emplaceBlock<SequenceSource<gr::pmt::Value>>();
        source.values           = fixtures;
        source.startup_delay_ms = 250;
        auto& push              = fg.emplaceBlock<gr::blocks::zeromq::ZmqPushSink<gr::pmt::Value>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"pmt_wire_format", gr::pmt::Value("GR3")},
        }));
        auto  receiver          = spawn_pull_receiver(TestEndpoint::bound_by(push), fixtures.size());

        expect(fg.connect<"out", "in">(source, push).has_value());

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        const bool completed = run_until_then_stop(sched, [&] { return receiver.wait_for(std::chrono::milliseconds{0}) == std::future_status::ready; }, std::chrono::milliseconds(3000));
        expect(completed);
        if (completed) {
            auto messages = receiver.get();
            expect(eq(messages.size(), fixtures.size()));
            if (messages.size() == fixtures.size()) {
                for (std::size_t i = 0; i < fixtures.size(); ++i) {
                    expect(messages[i] == legacy_pmt::serialize_to_legacy(fixtures[i]));
                }
            }
        }
    };

    "Pull source receives supported legacy PMT fixtures"_test = [] {
        const auto fixtures = make_pmt_fixtures();
        for (std::size_t i = 0; i < fixtures.size(); ++i) {
            const auto serialized = legacy_pmt::serialize_to_legacy(fixtures[i]);
            const auto roundtrip  = legacy_pmt::deserialize_from_legacy(serialized.data(), serialized.size());
            expect(legacy_pmt::serialize_to_legacy(roundtrip) == serialized);
        }
    };

    "Pull source rejects invalid legacy PMT payload"_test = [] {
        const std::vector<std::uint8_t> invalid{0xFF};
        bool                            threw = false;
        try {
            std::ignore = legacy_pmt::deserialize_from_legacy(invalid.data(), invalid.size());
        } catch (...) {
            threw = true;
        }
        expect(threw);
    };

    "PMT wire format defaults to GR4 YAML v1"_test = [] {
        PmtBlock block;
        expect(block.pmt_wire_format == gr::blocks::zeromq::PmtWireFormat::GR4_YAML_V1);
        expect(eq(gr::meta::enumName(gr::blocks::zeromq::PmtWireFormat::GR4_YAML_V1).value(), std::string_view{"GR4_YAML_V1"}));
        expect(eq(gr::meta::enumName(gr::blocks::zeromq::PmtWireFormat::GR3).value(), std::string_view{"GR3"}));

        const auto fixtures = make_pmt_fixtures();
        for (std::size_t index = 0; index < fixtures.size(); ++index) {
            const auto& value         = fixtures[index];
            const auto  yaml_v1_bytes = zdetail::serialize_pmt(value, gr::blocks::zeromq::PmtWireFormat::GR4_YAML_V1);
            const auto  decoded       = zdetail::deserialize_pmt(yaml_v1_bytes.data(), yaml_v1_bytes.size(), gr::blocks::zeromq::PmtWireFormat::GR4_YAML_V1);
            expect(decoded.value_type() == value.value_type()) << "fixture " << index;
            expect(decoded.container_type() == value.container_type()) << "fixture " << index;
            if (index < 15UZ) {
                expect(decoded == value) << "scalar fixture " << index;
            } else if (value.container_type() != gr::pmt::Value::ContainerType::Map) {
                expect(zdetail::serialize_pmt(decoded, gr::blocks::zeromq::PmtWireFormat::GR4_YAML_V1) == yaml_v1_bytes) << "container fixture " << index;
            }
        }

        const auto& value     = fixtures.front();
        const auto  gr3_bytes = zdetail::serialize_pmt(value, gr::blocks::zeromq::PmtWireFormat::GR3);
        expect(zdetail::deserialize_pmt(gr3_bytes.data(), gr3_bytes.size(), gr::blocks::zeromq::PmtWireFormat::GR3) == value);

        const auto                               gr4_header = zdetail::serialize_tag_header(0, {{0, gr::pmt::Value("key"), value, gr::pmt::Value("src")}});
        std::uint64_t                            offset     = 0;
        std::vector<zdetail::ZmqTagHeaderRecord> tags;
        std::ignore = zdetail::parse_tag_header(gr4_header.data(), gr4_header.size(), offset, tags);
        expect(eq(tags.size(), static_cast<std::size_t>(1)));
        if (tags.size() == 1) {
            expect(eq(tags.front().key.value_or(std::string_view{}), std::string_view{"key"}));
            expect(tags.front().value == value);
            expect(eq(tags.front().srcid.value_or(std::string_view{}), std::string_view{"src"}));
        }

        bool truncated_threw = false;
        try {
            const std::vector<std::uint8_t> truncated{0, 0, 0, 4, 'x'};
            std::ignore = zdetail::deserialize_pmt(truncated.data(), truncated.size(), gr::blocks::zeromq::PmtWireFormat::GR4_YAML_V1);
        } catch (const std::runtime_error& error) {
            truncated_threw = std::string_view(error.what()).find("truncated") != std::string_view::npos;
        }
        expect(truncated_threw);

        bool wrong_format_threw = false;
        try {
            std::ignore = zdetail::deserialize_pmt(gr3_bytes.data(), gr3_bytes.size(), gr::blocks::zeromq::PmtWireFormat::GR4_YAML_V1);
        } catch (const std::runtime_error& error) {
            wrong_format_threw = !std::string_view(error.what()).empty();
        }
        expect(wrong_format_threw);
    };

    "ZMQ rejects negative timeouts"_test = [] {
        bool threw = false;
        try {
            zdetail::ZmqSocketTransport::require_nonnegative_timeout(-1);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        expect(threw);
    };

    "ZMQ applies and validates the inbound message-size limit"_test = [] {
        bool threw = false;
        try {
            zdetail::ZmqSocketTransport::require_positive_max_message_size(0);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        expect(threw);

        zdetail::ZmqSocketTransport transport{zmq::socket_type::pull};
        transport.open(endpoint_for(57), true, 0, -1, false, 4096);
        auto operation = transport.acquire_operation_guard();
        expect(eq(transport.socket().get(zmq::sockopt::maxmsgsize), std::int64_t{4096}));
    };

    "PUB drop_on_hwm selects the no-drop socket mode"_test = [] {
        for (const bool drop_on_hwm : {true, false}) {
            gr::blocks::zeromq::ZmqPubSink<float> pub;
            pub.endpoint    = endpoint_for(drop_on_hwm ? 58 : 59);
            pub.bind        = true;
            pub.linger      = 0;
            pub.hwm         = 1;
            pub.drop_on_hwm = drop_on_hwm;
            pub.start();
            {
                auto operation = pub._transport.acquire_operation_guard();
                expect(pub._transport.socket().get(zmq::sockopt::socket_type) == zmq::socket_type::pub);
            }
            pub.stop();
        }
    };

    "ZMQ honors timeout beyond poll slice"_test = [] {
        zdetail::ZmqSocketTransport transport{zmq::socket_type::pull};
        transport.open(endpoint_for(46), true, 0, -1, false);

        const auto started = std::chrono::steady_clock::now();
        expect(!transport.wait_readable(250));
        const auto elapsed = std::chrono::steady_clock::now() - started;
        expect(elapsed >= std::chrono::milliseconds{200});
        expect(elapsed < std::chrono::milliseconds{600});
    };

    "ZMQ codecs reject truncated buffers"_test = [] {
        const auto tag_header = zdetail::serialize_tag_header(0, {});
        for (std::size_t size = 0; size < tag_header.size(); ++size) {
            std::uint64_t                            offset = 0;
            std::vector<zdetail::ZmqTagHeaderRecord> tags;
            bool                                     threw = false;
            try {
                std::ignore = zdetail::parse_tag_header(tag_header.data(), size, offset, tags);
            } catch (...) {
                threw = true;
            }
            expect(threw);
        }

        const auto pmt = legacy_pmt::serialize_to_legacy(gr::pmt::Value(std::uint32_t{0x12345678U}));
        for (std::size_t size = 0; size < pmt.size(); ++size) {
            bool threw = false;
            try {
                std::ignore = legacy_pmt::deserialize_from_legacy(pmt.data(), size);
            } catch (...) {
                threw = true;
            }
            expect(threw);
        }

        const std::vector<std::uint8_t> huge_vector{0x0A, 0x04, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00};
        bool                            huge_threw = false;
        try {
            std::ignore = legacy_pmt::deserialize_from_legacy(huge_vector.data(), huge_vector.size());
        } catch (...) {
            huge_threw = true;
        }
        expect(huge_threw);

        auto trailing = legacy_pmt::serialize_to_legacy(gr::pmt::Value(true));
        trailing.push_back(0);
        bool trailing_threw = false;
        try {
            std::ignore = legacy_pmt::deserialize_from_legacy(trailing.data(), trailing.size());
        } catch (...) {
            trailing_threw = true;
        }
        expect(trailing_threw);

        bool long_string_threw = false;
        try {
            std::ignore = legacy_pmt::serialize_to_legacy(gr::pmt::Value(std::string(65536, 'x')));
        } catch (...) {
            long_string_threw = true;
        }
        expect(long_string_threw);
    };

    "REP tag selection does not commit on header failure"_test = [] {
        std::vector<zdetail::ZmqTagHeaderRecord> pending;
        pending.push_back({0, gr::pmt::Value(std::string(65536, 'x')), gr::pmt::Value(1), gr::pmt::Value{}});
        const auto selected = zdetail::select_pending_tags(pending, 1);
        bool       threw    = false;
        try {
            std::ignore = zdetail::serialize_tag_header(0, selected, gr::blocks::zeromq::PmtWireFormat::GR3);
        } catch (...) {
            threw = true;
        }
        expect(threw);
        expect(eq(pending.size(), static_cast<std::size_t>(1)));
        expect(eq(pending[0].offset, 0ULL));
    };

    "Rep sink honors smaller and equal request counts"_test = [] {
        gr::Graph fg;
        using T             = float;
        const auto endpoint = endpoint_for(22);

        auto& source = fg.emplaceBlock<DelayedCountingSource<T>>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(5))},
            {"startup_delay_ms", gr::pmt::Value(static_cast<gr::Size_t>(100))},
        }));
        auto& rep    = fg.emplaceBlock<gr::blocks::zeromq::ZmqRepSink<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(100)},
            {"bind", gr::pmt::Value(true)},
        }));

        expect(fg.connect<"out", "in">(source, rep).has_value());

        auto client = spawn_req_client(TestEndpoint::bound_by(rep), std::vector<uint32_t>{2U, 3U});

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return client.wait_for(std::chrono::milliseconds{0}) == std::future_status::ready; }, std::chrono::milliseconds(3000)));

        auto replies = client.get();
        expect(eq(replies.size(), static_cast<std::size_t>(2)));
        if (replies.size() == 2) {
            expect(eq(replies[0].size(), 2U * sizeof(T)));
            expect(eq(replies[1].size(), 3U * sizeof(T)));
            const auto* first  = reinterpret_cast<const T*>(replies[0].data());
            const auto* second = reinterpret_cast<const T*>(replies[1].data());
            expect(eq(first[0], 1.f));
            expect(eq(first[1], 2.f));
            expect(eq(second[0], 3.f));
            expect(eq(second[1], 4.f));
            expect(eq(second[2], 5.f));
        }
        const auto stats = rep.send_statistics();
        expect(eq(stats.messages, std::uint64_t{2}));
        expect(eq(stats.items, std::uint64_t{5}));
        expect(eq(stats.refused_messages, std::uint64_t{0}));
    };

    "Rep sink replies to zero count without consuming data"_test = [] {
        gr::Graph fg;
        using T             = float;
        const auto endpoint = endpoint_for(43);
        auto&      source   = fg.emplaceBlock<RepeatingSource<T>>(make_props({
            {"startup_delay_ms", gr::pmt::Value(static_cast<gr::Size_t>(10))},
        }));
        source.value        = 7.f;
        auto& rep           = fg.emplaceBlock<gr::blocks::zeromq::ZmqRepSink<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
        }));
        expect(fg.connect<"out", "in">(source, rep).has_value());
        auto client = spawn_req_client(TestEndpoint::bound_by(rep), std::vector<uint32_t>{0U, 1U});

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return client.wait_for(std::chrono::milliseconds{0}) == std::future_status::ready; }, std::chrono::milliseconds(2000)));
        auto replies = client.get();
        expect(eq(replies.size(), static_cast<std::size_t>(2)));
        if (replies.size() == 2) {
            expect(eq(replies[0].size(), static_cast<std::size_t>(0)));
            expect(eq(replies[1].size(), sizeof(T)));
            if (replies[1].size() == sizeof(T)) {
                T first = 0;
                std::memcpy(&first, replies[1].data(), sizeof(first));
                expect(eq(first, 7.f));
            }
        }
    };

    "Rep sink emits a valid empty tag-framed zero reply"_test = [] {
        gr::Graph fg;
        using T             = float;
        const auto endpoint = endpoint_for(45);
        auto&      source   = fg.emplaceBlock<RepeatingSource<T>>(make_props({
            {"startup_delay_ms", gr::pmt::Value(static_cast<gr::Size_t>(10))},
        }));
        source.value        = 7.f;
        auto& rep           = fg.emplaceBlock<gr::blocks::zeromq::ZmqRepSink<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"pass_tags", gr::pmt::Value(true)},
        }));
        expect(fg.connect<"out", "in">(source, rep).has_value());
        auto client = spawn_req_client(TestEndpoint::bound_by(rep), std::vector<uint32_t>{0U, 1U});

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return client.wait_for(std::chrono::milliseconds{0}) == std::future_status::ready; }, std::chrono::milliseconds(2000)));
        auto replies = client.get();
        expect(eq(replies.size(), static_cast<std::size_t>(2)));
        if (replies.size() == 2) {
            for (std::size_t i = 0; i < replies.size(); ++i) {
                std::uint64_t                            header_offset = 0;
                std::vector<zdetail::ZmqTagHeaderRecord> tags;
                const auto                               consumed = zdetail::parse_tag_header(replies[i].data(), replies[i].size(), header_offset, tags);
                expect(eq(header_offset, 0ULL));
                expect(tags.empty());
                expect(eq(replies[i].size() - consumed, i == 0 ? static_cast<std::size_t>(0) : sizeof(T)));
            }
        }
    };

    "Rep vector sink replies when queue is empty"_test = [] {
        gr::Graph fg;
        using T             = std::vector<float>;
        const auto endpoint = endpoint_for(47);

        auto& source = fg.emplaceBlock<DelayedDefaultSource<T>>(make_props({
            {"startup_delay_ms", gr::pmt::Value(static_cast<gr::Size_t>(250))},
        }));
        auto& rep    = fg.emplaceBlock<gr::blocks::zeromq::ZmqRepSink<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
        }));
        expect(fg.connect<"out", "in">(source, rep).has_value());
        auto client = spawn_req_client(TestEndpoint::bound_by(rep), std::vector<uint32_t>{1U});

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return client.wait_for(std::chrono::milliseconds{0}) == std::future_status::ready; }, std::chrono::milliseconds{2000}));
        const auto replies = client.get();
        expect(eq(replies.size(), static_cast<std::size_t>(1)));
        if (replies.size() == 1) {
            expect(replies[0].empty());
        }
    };

    "Rep PMT sink replies when queue is empty"_test = [] {
        gr::Graph fg;
        using T             = gr::pmt::Value;
        const auto endpoint = endpoint_for(48);

        auto& source = fg.emplaceBlock<DelayedDefaultSource<T>>(make_props({
            {"startup_delay_ms", gr::pmt::Value(static_cast<gr::Size_t>(250))},
        }));
        auto& rep    = fg.emplaceBlock<gr::blocks::zeromq::ZmqRepSink<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
        }));
        expect(fg.connect<"out", "in">(source, rep).has_value());
        auto client = spawn_req_client(TestEndpoint::bound_by(rep), std::vector<uint32_t>{1U});

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return client.wait_for(std::chrono::milliseconds{0}) == std::future_status::ready; }, std::chrono::milliseconds{2000}));
        const auto replies = client.get();
        expect(eq(replies.size(), static_cast<std::size_t>(1)));
        if (replies.size() == 1) {
            expect(replies[0] == zdetail::serialize_pmt(gr::pmt::Value{}, gr::blocks::zeromq::PmtWireFormat::GR4_YAML_V1));
        }
    };

    "Rep sink caps larger request counts"_test = [] {
        gr::Graph fg;
        using T             = float;
        const auto endpoint = endpoint_for(23);

        auto& source = fg.emplaceBlock<DelayedCountingSource<T>>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(5))},
            {"startup_delay_ms", gr::pmt::Value(static_cast<gr::Size_t>(50))},
        }));
        auto& rep    = fg.emplaceBlock<gr::blocks::zeromq::ZmqRepSink<T>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(100)},
            {"bind", gr::pmt::Value(true)},
        }));

        expect(fg.connect<"out", "in">(source, rep).has_value());

        auto client = spawn_req_client(TestEndpoint::bound_by(rep), std::vector<uint32_t>{7U});

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return client.wait_for(std::chrono::milliseconds{0}) == std::future_status::ready; }, std::chrono::milliseconds(3000)));

        auto replies = client.get();
        expect(eq(replies.size(), static_cast<std::size_t>(1)));
        if (replies.size() == 1) {
            expect(eq(replies[0].size(), 5U * sizeof(T)));
        }
    };

    "Rep sink emits legacy PMT bytes for supported fixtures"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(24);
        const auto fixtures = make_pmt_fixtures();

        auto& source            = fg.emplaceBlock<SequenceSource<gr::pmt::Value>>();
        source.values           = fixtures;
        source.startup_delay_ms = 250;
        auto& rep               = fg.emplaceBlock<gr::blocks::zeromq::ZmqRepSink<gr::pmt::Value>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(100)},
            {"bind", gr::pmt::Value(true)},
            {"pmt_wire_format", gr::pmt::Value("GR3")},
        }));

        expect(fg.connect<"out", "in">(source, rep).has_value());

        auto client = spawn_req_client(TestEndpoint::bound_by(rep), std::vector<uint32_t>(fixtures.size(), 1U));

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return client.wait_for(std::chrono::milliseconds{0}) == std::future_status::ready; }, std::chrono::milliseconds(3000)));

        auto replies = client.get();
        expect(eq(replies.size(), fixtures.size()));
        if (replies.size() == fixtures.size()) {
            for (std::size_t i = 0; i < fixtures.size(); ++i) {
                expect(replies[i] == legacy_pmt::serialize_to_legacy(fixtures[i]));
            }
        }
    };

    "Req source receives legacy PMT payloads"_test = [] {
        gr::Graph                              fg;
        const auto                             endpoint = endpoint_for(25);
        const auto                             fixtures = make_pmt_fixtures();
        std::vector<std::vector<std::uint8_t>> payloads;
        payloads.reserve(fixtures.size());
        for (const auto& fixture : fixtures) {
            payloads.push_back(legacy_pmt::serialize_to_legacy(fixture));
        }

        auto& source = fg.emplaceBlock<gr::blocks::zeromq::ZmqReqSource<gr::pmt::Value>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(100)},
            {"bind", gr::pmt::Value(true)},
            {"pmt_wire_format", gr::pmt::Value("GR3")},
        }));
        auto& sink   = fg.emplaceBlock<RecordingPmtSink>(make_props({
            {"n_samples_max", gr::pmt::Value(fixtures.size())},
        }));

        expect(fg.connect<"out", "in">(source, sink).has_value());

        auto responder = spawn_rep_sequence_responder(TestEndpoint::bound_by(source), std::move(payloads));

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.received.size() >= fixtures.size(); }, std::chrono::milliseconds(3000)));
        expect(eq(sink.received.size(), fixtures.size()));
        if (sink.received.size() == fixtures.size()) {
            for (std::size_t i = 0; i < fixtures.size(); ++i) {
                expect(legacy_pmt::serialize_to_legacy(sink.received[i]) == legacy_pmt::serialize_to_legacy(fixtures[i]));
            }
        }

        if (responder.joinable()) {
            responder.join();
        }
    };

    "Pub/Sub PMT payload uses legacy serialization"_test = [] {
        gr::Graph         fg;
        const auto        endpoint = endpoint_for(12);
        const std::string key      = "pmt.";

        auto& source  = fg.emplaceBlock<SequenceSource<gr::pmt::Value>>();
        source.values = {
            gr::pmt::Value(gr::Tensor<float>(gr::data_from, std::vector<float>{1.f, 2.f, 3.f, 4.f})),
            gr::pmt::Value(gr::Tensor<float>(gr::data_from, std::vector<float>{1.f, 2.f, 3.f, 4.f})),
            gr::pmt::Value(gr::Tensor<float>(gr::data_from, std::vector<float>{1.f, 2.f, 3.f, 4.f})),
        };
        source.startup_delay_ms = 1000;
        auto& pub               = fg.emplaceBlock<gr::blocks::zeromq::ZmqPubSink<gr::pmt::Value>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"key", gr::pmt::Value(key)},
            {"pmt_wire_format", gr::pmt::Value("GR3")},
        }));
        auto  receiver          = spawn_sub_receiver(TestEndpoint::bound_by(pub), key);

        expect(fg.connect<"out", "in">(source, pub).has_value());

        const auto expected = legacy_pmt::serialize_to_legacy(source.values.front());

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        const bool completed = run_until_then_stop(sched, [&] { return receiver.wait_for(std::chrono::milliseconds{0}) == std::future_status::ready; }, std::chrono::milliseconds(4000));
        expect(completed);
        if (completed) {
            auto frames = receiver.get();
            expect(eq(frames.size(), static_cast<std::size_t>(2)));
            if (frames.size() == 2) {
                expect(eq(std::string_view(reinterpret_cast<const char*>(frames[0].data()), frames[0].size()), key));
                expect(frames[1] == expected);
            }
        }
    };

    "Sub source receives legacy PMT payload"_test = [] {
        gr::Graph         fg;
        const auto        endpoint = endpoint_for(13);
        const std::string key      = "pmt.";

        auto& sub  = fg.emplaceBlock<gr::blocks::zeromq::ZmqSubSource<gr::pmt::Value>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"key", gr::pmt::Value(key)},
            {"pmt_wire_format", gr::pmt::Value("GR3")},
        }));
        auto& sink = fg.emplaceBlock<PmtSink>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(1))},
        }));

        expect(fg.connect<"out", "in">(sub, sink).has_value());

        auto payload = legacy_pmt::serialize_to_legacy(gr::pmt::Value(gr::Tensor<float>(gr::data_from, std::vector<float>{1.f, 2.f, 3.f, 4.f})));
        auto sender  = spawn_pub_sender(TestEndpoint::bound_by(sub), key, std::move(payload));

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.count >= static_cast<gr::Size_t>(1); }, std::chrono::milliseconds(2000)));
        expect(eq(sink.count, static_cast<gr::Size_t>(1)));

        if (sender.joinable()) {
            sender.join();
        }
    };

    "Push sink emits tag headers"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(26);

        auto& source            = fg.emplaceBlock<TaggedSequenceSource<float>>();
        source.values           = std::vector<float>(100, 1.f);
        source.startup_delay_ms = 1000;
        for (std::size_t i = 0; i < source.values.size(); ++i) {
            source.tags.emplace_back(i, wire_tag("alpha", gr::pmt::Value(int32_t{11}), gr::pmt::Value("src-a")));
        }
        auto& push     = fg.emplaceBlock<gr::blocks::zeromq::ZmqPushSink<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"pass_tags", gr::pmt::Value(true)},
        }));
        auto  receiver = spawn_pull_receiver(TestEndpoint::bound_by(push), 1);

        expect(fg.connect<"out", "in">(source, push).has_value());

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return receiver.wait_for(std::chrono::milliseconds{0}) == std::future_status::ready; }, std::chrono::milliseconds(3000)));

        auto messages = receiver.get();
        expect(eq(messages.size(), static_cast<std::size_t>(1)));
        if (messages.size() == 1) {
            std::uint64_t                            header_offset = 0;
            std::vector<zdetail::ZmqTagHeaderRecord> tags;
            const auto                               consumed = zdetail::parse_tag_header(messages[0].data(), messages[0].size(), header_offset, tags);
            expect(eq(header_offset, 0ULL));
            expect(eq(tags.size(), static_cast<std::size_t>(1)));
            if (tags.size() == 1) {
                expect(eq(tags[0].offset, 0ULL));
                expect(eq(tags[0].key.value_or(std::string{}), std::string{"alpha"}));
                expect(eq(tags[0].value.value_or<int32_t>(0), 11));
                expect(eq(tags[0].srcid.value_or(std::string{}), std::string{"src-a"}));
                const auto* payload      = messages[0].data() + consumed;
                const auto  payload_size = messages[0].size() - consumed;
                expect(eq(payload_size, 1UL * sizeof(float)));
                const auto* floats = reinterpret_cast<const float*>(payload);
                expect(eq(floats[0], 1.f));
            }
        }
    };

    "Push sink preserves multiple tags at one offset"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(260);

        auto& source            = fg.emplaceBlock<TaggedBulkSource<float>>();
        source.values           = {1.f};
        source.startup_delay_ms = 500;
        source.tags             = {
            {0UZ, wire_tag("alpha", gr::pmt::Value(int32_t{11}), gr::pmt::Value("src-a"))},
            {0UZ, wire_tag("beta", gr::pmt::Value(int32_t{22}), gr::pmt::Value("src-b"))},
        };
        auto& push     = fg.emplaceBlock<gr::blocks::zeromq::ZmqPushSink<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"pass_tags", gr::pmt::Value(true)},
        }));
        auto  receiver = spawn_pull_receiver(TestEndpoint::bound_by(push), 1);

        expect(fg.connect<"out", "in">(source, push).has_value());

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return receiver.wait_for(std::chrono::milliseconds{0}) == std::future_status::ready; }, std::chrono::milliseconds(3000)));

        auto messages = receiver.get();
        expect(eq(messages.size(), static_cast<std::size_t>(1)));
        if (messages.size() == 1) {
            std::uint64_t                            header_offset = 0;
            std::vector<zdetail::ZmqTagHeaderRecord> tags;
            const auto                               consumed = zdetail::parse_tag_header(messages[0].data(), messages[0].size(), header_offset, tags);
            expect(eq(header_offset, 0ULL));
            expect(eq(tags.size(), static_cast<std::size_t>(2)));
            if (tags.size() == 2) {
                expect(eq(tags[0].offset, 0ULL));
                expect(eq(tags[0].key.value_or(std::string{}), std::string{"alpha"}));
                expect(eq(tags[0].value.value_or<int32_t>(0), 11));
                expect(eq(tags[1].offset, 0ULL));
                expect(eq(tags[1].key.value_or(std::string{}), std::string{"beta"}));
                expect(eq(tags[1].value.value_or<int32_t>(0), 22));
            }
            const auto payload_size = messages[0].size() - consumed;
            expect(eq(payload_size, 1UL * sizeof(float)));
        }
    };

    "Pull source receives tag headers"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(27);
        auto&      source   = fg.emplaceBlock<gr::blocks::zeromq::ZmqPullSource<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"pass_tags", gr::pmt::Value(true)},
        }));
        auto&      sink     = fg.emplaceBlock<RecordingTaggedSink>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(4))},
        }));

        expect(fg.connect<"out", "in">(source, sink).has_value());

        std::vector<zdetail::ZmqTagHeaderRecord> tags = {
            {0, gr::pmt::Value("alpha"), gr::pmt::Value(int32_t{11}), gr::pmt::Value("src-a")},
            {2, gr::pmt::Value("beta"), gr::pmt::Value(int32_t{22}), gr::pmt::Value("src-b")},
        };
        auto                      header  = zdetail::serialize_tag_header(0, tags);
        auto                      payload = to_bytes(std::vector<float>{1.f, 2.f, 3.f, 4.f});
        std::vector<std::uint8_t> message;
        message.reserve(header.size() + payload.size());
        message.insert(message.end(), header.begin(), header.end());
        message.insert(message.end(), payload.begin(), payload.end());
        auto sender = spawn_push_sender(TestEndpoint::bound_by(source), std::move(message));

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.received.size() >= 4; }, std::chrono::milliseconds(3000)));
        expect(eq(sink.received.size(), static_cast<std::size_t>(4)));
        expect(eq(sink.tags.size(), static_cast<std::size_t>(2)));
        if (sink.tags.size() == 2) {
            expect(eq(sink.tags[0].first, static_cast<std::size_t>(0)));
            expect(eq(sink.tags[0].second.at("key").value_or(std::string{}), std::string{"alpha"}));
            expect(eq(sink.tags[0].second.at("value").value_or<int32_t>(0), 11));
            expect(eq(sink.tags[1].first, static_cast<std::size_t>(2)));
            expect(eq(sink.tags[1].second.at("key").value_or(std::string{}), std::string{"beta"}));
            expect(eq(sink.tags[1].second.at("value").value_or<int32_t>(0), 22));
        }

        if (sender.joinable()) {
            sender.join();
        }
    };

    "Pull source receives multiple tags at one offset"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(270);
        auto&      source   = fg.emplaceBlock<gr::blocks::zeromq::ZmqPullSource<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"pass_tags", gr::pmt::Value(true)},
        }));
        auto&      sink     = fg.emplaceBlock<RecordingTaggedSink>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(1))},
        }));

        expect(fg.connect<"out", "in">(source, sink).has_value());

        std::vector<zdetail::ZmqTagHeaderRecord> tags = {
            {0, gr::pmt::Value("alpha"), gr::pmt::Value(int32_t{11}), gr::pmt::Value("src-a")},
            {0, gr::pmt::Value("beta"), gr::pmt::Value(int32_t{22}), gr::pmt::Value("src-b")},
        };
        auto                      header  = zdetail::serialize_tag_header(0, tags);
        auto                      payload = to_bytes(std::vector<float>{1.f});
        std::vector<std::uint8_t> message;
        message.reserve(header.size() + payload.size());
        message.insert(message.end(), header.begin(), header.end());
        message.insert(message.end(), payload.begin(), payload.end());
        auto sender = spawn_push_sender(TestEndpoint::bound_by(source), std::move(message));

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.received.size() >= 1; }, std::chrono::milliseconds(3000)));
        expect(eq(sink.received.size(), static_cast<std::size_t>(1)));
        expect(eq(sink.tags.size(), static_cast<std::size_t>(2)));
        if (sink.tags.size() == 2) {
            expect(eq(sink.tags[0].first, static_cast<std::size_t>(0)));
            expect(eq(sink.tags[0].second.at("key").value_or(std::string{}), std::string{"alpha"}));
            expect(eq(sink.tags[0].second.at("value").value_or<int32_t>(0), 11));
            expect(eq(sink.tags[1].first, static_cast<std::size_t>(0)));
            expect(eq(sink.tags[1].second.at("key").value_or(std::string{}), std::string{"beta"}));
            expect(eq(sink.tags[1].second.at("value").value_or<int32_t>(0), 22));
        }

        if (sender.joinable()) {
            sender.join();
        }
    };

    "Pub sink emits tag headers"_test = [] {
        gr::Graph         fg;
        const auto        endpoint = endpoint_for(28);
        const std::string key      = "tag.";

        auto& source            = fg.emplaceBlock<TaggedSequenceSource<float>>();
        source.values           = std::vector<float>(100, 1.f);
        source.startup_delay_ms = 1000;
        for (std::size_t i = 0; i < source.values.size(); ++i) {
            source.tags.emplace_back(i, wire_tag("alpha", gr::pmt::Value(int32_t{11}), gr::pmt::Value("src-a")));
        }
        auto& pub      = fg.emplaceBlock<gr::blocks::zeromq::ZmqPubSink<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"pass_tags", gr::pmt::Value(true)},
            {"key", gr::pmt::Value(key)},
        }));
        auto  receiver = spawn_sub_receiver(TestEndpoint::bound_by(pub), key);

        expect(fg.connect<"out", "in">(source, pub).has_value());

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        const bool completed = run_until_then_stop(sched, [&] { return receiver.wait_for(std::chrono::milliseconds{0}) == std::future_status::ready; }, std::chrono::milliseconds(3000));
        expect(completed);
        if (completed) {
            auto frames = receiver.get();
            expect(eq(frames.size(), static_cast<std::size_t>(2)));
            if (frames.size() == 2) {
                expect(eq(std::string_view(reinterpret_cast<const char*>(frames[0].data()), frames[0].size()), key));
                std::uint64_t                            header_offset = 0;
                std::vector<zdetail::ZmqTagHeaderRecord> tags;
                const auto                               consumed = zdetail::parse_tag_header(frames[1].data(), frames[1].size(), header_offset, tags);
                expect(eq(header_offset, 0ULL));
                expect(eq(tags.size(), static_cast<std::size_t>(1)));
                if (tags.size() == 1) {
                    expect(eq(tags[0].offset, 0ULL));
                    expect(eq(tags[0].key.value_or(std::string{}), std::string{"alpha"}));
                    expect(eq(tags[0].value.value_or<int32_t>(0), 11));
                }
                const auto payload_size = frames[1].size() - consumed;
                expect(eq(payload_size, 1UL * sizeof(float)));
            }
        }
    };

    "Sub source receives tag headers"_test = [] {
        gr::Graph         fg;
        const auto        endpoint = endpoint_for(29);
        const std::string key      = "tag.";

        auto& sub  = fg.emplaceBlock<gr::blocks::zeromq::ZmqSubSource<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
            {"pass_tags", gr::pmt::Value(true)},
            {"key", gr::pmt::Value(key)},
        }));
        auto& sink = fg.emplaceBlock<RecordingTaggedSink>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(4))},
        }));

        expect(fg.connect<"out", "in">(sub, sink).has_value());

        std::vector<zdetail::ZmqTagHeaderRecord> tags = {
            {0, gr::pmt::Value("alpha"), gr::pmt::Value(int32_t{11}), gr::pmt::Value("src-a")},
            {3, gr::pmt::Value("beta"), gr::pmt::Value(int32_t{22}), gr::pmt::Value("src-b")},
        };
        auto                      header  = zdetail::serialize_tag_header(0, tags);
        auto                      payload = to_bytes(std::vector<float>{1.f, 2.f, 3.f, 4.f});
        std::vector<std::uint8_t> message;
        message.reserve(header.size() + payload.size());
        message.insert(message.end(), header.begin(), header.end());
        message.insert(message.end(), payload.begin(), payload.end());
        auto sender = spawn_pub_sender(TestEndpoint::bound_by(sub), key, std::move(message));

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.received.size() >= 4; }, std::chrono::milliseconds(3000)));
        expect(eq(sink.received.size(), static_cast<std::size_t>(4)));
        expect(eq(sink.tags.size(), static_cast<std::size_t>(2)));
        if (sink.tags.size() == 2) {
            expect(eq(sink.tags[0].first, static_cast<std::size_t>(0)));
            expect(eq(sink.tags[0].second.at("key").value_or(std::string{}), std::string{"alpha"}));
            expect(eq(sink.tags[0].second.at("value").value_or<int32_t>(0), 11));
            expect(eq(sink.tags[1].first, static_cast<std::size_t>(3)));
            expect(eq(sink.tags[1].second.at("key").value_or(std::string{}), std::string{"beta"}));
            expect(eq(sink.tags[1].second.at("value").value_or<int32_t>(0), 22));
        }

        if (sender.joinable()) {
            sender.join();
        }
    };

    "Rep sink emits tag headers"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(30);

        auto& source            = fg.emplaceBlock<TaggedBulkSource<float>>();
        source.values           = {1.f, 2.f, 3.f, 4.f};
        source.startup_delay_ms = 250;
        source.tags             = {
            {0UZ, wire_tag("alpha", gr::pmt::Value(int32_t{11}), gr::pmt::Value("src-a"))},
        };
        auto& rep    = fg.emplaceBlock<gr::blocks::zeromq::ZmqRepSink<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(2000)},
            {"bind", gr::pmt::Value(true)},
            {"pass_tags", gr::pmt::Value(true)},
        }));
        auto  client = spawn_req_client(TestEndpoint::bound_by(rep), std::vector<uint32_t>{4U});

        expect(fg.connect<"out", "in">(source, rep).has_value());

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return client.wait_for(std::chrono::milliseconds{0}) == std::future_status::ready; }, std::chrono::milliseconds(3000)));

        auto replies = client.get();
        expect(eq(replies.size(), static_cast<std::size_t>(1)));
        if (replies.size() == 1) {
            std::uint64_t                            header_offset = 0;
            std::vector<zdetail::ZmqTagHeaderRecord> tags;
            const auto                               consumed = zdetail::parse_tag_header(replies[0].data(), replies[0].size(), header_offset, tags);
            expect(eq(header_offset, 0ULL));
            expect(eq(tags.size(), static_cast<std::size_t>(1)));
            if (tags.size() == 1) {
                expect(eq(tags[0].offset, 0ULL));
                expect(eq(tags[0].key.value_or(std::string{}), std::string{"alpha"}));
                expect(eq(tags[0].value.value_or<int32_t>(0), 11));
            }
            const auto payload_size = replies[0].size() - consumed;
            expect(eq(payload_size, 4UL * sizeof(float)));
        }
    };

    "Req source receives tag headers"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(31);
        auto&      source   = fg.emplaceBlock<gr::blocks::zeromq::ZmqReqSource<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(100)},
            {"bind", gr::pmt::Value(true)},
            {"pass_tags", gr::pmt::Value(true)},
        }));
        auto&      sink     = fg.emplaceBlock<RecordingTaggedSink>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(4))},
        }));

        expect(fg.connect<"out", "in">(source, sink).has_value());

        std::vector<zdetail::ZmqTagHeaderRecord> tags = {
            {0, gr::pmt::Value("alpha"), gr::pmt::Value(int32_t{11}), gr::pmt::Value("src-a")},
            {2, gr::pmt::Value("beta"), gr::pmt::Value(int32_t{22}), gr::pmt::Value("src-b")},
        };
        auto                      header  = zdetail::serialize_tag_header(0, tags);
        auto                      payload = to_bytes(std::vector<float>{1.f, 2.f, 3.f, 4.f});
        std::vector<std::uint8_t> message;
        message.reserve(header.size() + payload.size());
        message.insert(message.end(), header.begin(), header.end());
        message.insert(message.end(), payload.begin(), payload.end());
        auto responder = spawn_rep_sequence_responder(TestEndpoint::bound_by(source), {std::move(message)});

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.received.size() >= 4; }, std::chrono::milliseconds(3000)));
        expect(eq(sink.received.size(), static_cast<std::size_t>(4)));
        expect(eq(sink.tags.size(), static_cast<std::size_t>(2)));
        if (sink.tags.size() == 2) {
            expect(eq(sink.tags[0].first, static_cast<std::size_t>(0)));
            expect(eq(sink.tags[0].second.at("key").value_or(std::string{}), std::string{"alpha"}));
            expect(eq(sink.tags[0].second.at("value").value_or<int32_t>(0), 11));
            expect(eq(sink.tags[1].first, static_cast<std::size_t>(2)));
            expect(eq(sink.tags[1].second.at("key").value_or(std::string{}), std::string{"beta"}));
            expect(eq(sink.tags[1].second.at("value").value_or<int32_t>(0), 22));
        }

        if (responder.joinable()) {
            responder.join();
        }
    };

    "Runtime REP applies upstream backpressure without a requester"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(60);
        auto&      source   = fg.emplaceBlock<AtomicCountingSource<float>>();
        auto&      rep      = fg.emplaceBlock<gr::blocks::zeromq::ZmqRepSink<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
        }));
        expect(fg.connect<"out", "in">(source, rep).has_value());

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        auto runner = std::async(std::launch::async, [&] { return sched.runAndWait(); });
        std::this_thread::sleep_for(std::chrono::milliseconds{350});
        const auto first = std::atomic_ref(source.count).load(std::memory_order_acquire);
        std::this_thread::sleep_for(std::chrono::milliseconds{150});
        const auto second       = std::atomic_ref(source.count).load(std::memory_order_acquire);
        const auto stop_started = std::chrono::steady_clock::now();
        std::ignore             = sched.changeStateTo(gr::lifecycle::State::REQUESTED_STOP);
        expect(runner.wait_for(std::chrono::seconds{1}) == std::future_status::ready);
        if (runner.wait_for(std::chrono::seconds{0}) == std::future_status::ready) {
            expect(runner.get().has_value());
        }
        expect(std::chrono::steady_clock::now() - stop_started < std::chrono::milliseconds{500});
        expect(second - first < static_cast<gr::Size_t>(1024));
        expect(!rep._pending_reply.has_value());
        expect(eq(rep._pending_reply_items, static_cast<std::size_t>(0)));
    };

    "Runtime REP serves finite input after a delayed request"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(61);
        auto&      source   = fg.emplaceBlock<FiniteHoldingSource<float>>();
        source.values       = {1.f, 2.f, 3.f};
        auto& rep           = fg.emplaceBlock<gr::blocks::zeromq::ZmqRepSink<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(25)},
            {"bind", gr::pmt::Value(true)},
        }));
        expect(fg.connect<"out", "in">(source, rep).has_value());
        auto client = spawn_req_client(TestEndpoint::bound_by(rep), {3U});

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return client.wait_for(0ms) == std::future_status::ready; }, 2s));
        const auto replies = client.get();
        expect(eq(replies.size(), static_cast<std::size_t>(1)));
        if (replies.size() == 1) {
            expect(replies[0] == to_bytes(std::vector<float>{1.f, 2.f, 3.f}));
        }
    };

    "Runtime REP drains malformed multipart request and serves the next request"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(62);
        auto&      source   = fg.emplaceBlock<FiniteHoldingSource<float>>();
        source.values       = {9.f};
        auto& rep           = fg.emplaceBlock<gr::blocks::zeromq::ZmqRepSink<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(50)},
            {"bind", gr::pmt::Value(true)},
        }));
        expect(fg.connect<"out", "in">(source, rep).has_value());

        auto client = std::async(std::launch::async, [endpoint = TestEndpoint::bound_by(rep)] {
            const auto     resolved_endpoint = endpoint.resolve();
            zmq::context_t ctx{1};
            zmq::socket_t  req{ctx, zmq::socket_type::req};
            req.set(zmq::sockopt::rcvtimeo, 1500);
            req.connect(resolved_endpoint);
            uint32_t       count = 1;
            zmq::message_t first(sizeof(count));
            std::memcpy(first.data(), &count, sizeof(count));
            if (!req.send(first, zmq::send_flags::sndmore)) {
                return false;
            }
            zmq::message_t extra(1);
            if (!req.send(extra, zmq::send_flags::none)) {
                return false;
            }
            zmq::message_t malformed_reply;
            if (!req.recv(malformed_reply) || malformed_reply.size() != 0) {
                return false;
            }
            zmq::message_t valid(sizeof(count));
            std::memcpy(valid.data(), &count, sizeof(count));
            if (!req.send(valid, zmq::send_flags::none)) {
                return false;
            }
            zmq::message_t reply;
            if (!req.recv(reply) || reply.size() != sizeof(float)) {
                return false;
            }
            float value = 0;
            std::memcpy(&value, reply.data(), sizeof(value));
            return value == 9.f;
        });

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return client.wait_for(0ms) == std::future_status::ready; }, 3s));
        expect(client.get());
    };

    "Runtime REQ resets after peer loss and reaches replacement REP"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(63);
        auto&      req      = fg.emplaceBlock<gr::blocks::zeromq::ZmqReqSource<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(100)},
            {"bind", gr::pmt::Value(true)},
        }));
        auto&      sink     = fg.emplaceBlock<RecordingScalarSink<float>>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(1))},
        }));
        expect(fg.connect<"out", "in">(req, sink).has_value());

        auto peers = std::async(std::launch::async, [endpoint = TestEndpoint::bound_by(req)] {
            const auto     resolved_endpoint = endpoint.resolve();
            zmq::context_t ctx{1};
            {
                zmq::socket_t lost{ctx, zmq::socket_type::rep};
                lost.set(zmq::sockopt::rcvtimeo, 1500);
                lost.connect(resolved_endpoint);
                zmq::message_t request;
                if (!lost.recv(request)) {
                    return false;
                }
            }
            std::this_thread::sleep_for(200ms);
            zmq::socket_t replacement{ctx, zmq::socket_type::rep};
            replacement.set(zmq::sockopt::rcvtimeo, 2000);
            replacement.connect(resolved_endpoint);
            zmq::message_t request;
            if (!replacement.recv(request)) {
                return false;
            }
            const float    value = 17.f;
            zmq::message_t reply(sizeof(value));
            std::memcpy(reply.data(), &value, sizeof(value));
            return bool(replacement.send(reply, zmq::send_flags::none));
        });

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        expect(run_until_then_stop(sched, [&] { return sink.received.size() == 1; }, 4s));
        expect(peers.get());
        expect(eq(sink.received.size(), static_cast<std::size_t>(1)));
        if (!sink.received.empty()) {
            expect(eq(sink.received[0], 17.f));
        }
    };

    "Runtime REQ empty replies are bounded and useful output returns promptly"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(64);
        auto&      req      = fg.emplaceBlock<gr::blocks::zeromq::ZmqReqSource<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(2000)},
            {"bind", gr::pmt::Value(true)},
        }));
        auto&      sink     = fg.emplaceBlock<RecordingScalarSink<float>>(make_props({
            {"n_samples_max", gr::pmt::Value(static_cast<gr::Size_t>(1))},
        }));
        expect(fg.connect<"out", "in">(req, sink).has_value());
        std::atomic<bool>                                                    done{false};
        std::atomic<std::size_t>                                             transactions{0};
        const auto                                                           bound_endpoint = TestEndpoint::bound_by(req);
        std::thread                                                          peer([&, bound_endpoint] {
            const auto     resolved_endpoint = bound_endpoint.resolve();
            zmq::context_t ctx{1};
            zmq::socket_t  rep{ctx, zmq::socket_type::rep};
            rep.set(zmq::sockopt::rcvtimeo, 100);
            rep.connect(resolved_endpoint);
            bool first = true;
            while (!done.load(std::memory_order_acquire)) {
                zmq::message_t request;
                if (!rep.recv(request)) {
                    continue;
                }
                zmq::message_t reply(first ? sizeof(float) : 0);
                if (first) {
                    const float value = 23.f;
                    std::memcpy(reply.data(), &value, sizeof(value));
                    first = false;
                }
                if (rep.send(reply, zmq::send_flags::none)) {
                    ++transactions;
                }
            }
                                                                 });
        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        const auto started = std::chrono::steady_clock::now();
        expect(run_until_then_stop(sched, [&] { return sink.received.size() == 1; }, 2s));
        done.store(true, std::memory_order_release);
        peer.join();
        expect(std::chrono::steady_clock::now() - started < 1s);
        expect(eq(sink.received.size(), static_cast<std::size_t>(1)));
        expect(transactions.load() >= 1);
    };

    "Runtime deadline is shared by REQ writable and readable waits"_test = [] {
        const auto                  endpoint = endpoint_for(65);
        zdetail::ZmqSocketTransport transport{zmq::socket_type::req};
        transport.open(endpoint, true, 0, -1, false);
        const auto                        resolved_endpoint = transport.last_endpoint();
        auto                              peer              = std::async(std::launch::async, [resolved_endpoint] {
            std::this_thread::sleep_for(100ms);
            zmq::context_t ctx{1};
            zmq::socket_t  rep{ctx, zmq::socket_type::rep};
            rep.connect(resolved_endpoint);
            zmq::message_t request;
            std::ignore = rep.recv(request);
            std::this_thread::sleep_for(400ms);
        });
        auto                              operation         = transport.acquire_operation_guard();
        const auto                        started           = std::chrono::steady_clock::now();
        const zdetail::InvocationDeadline deadline{250};
        expect(transport.wait_writable(deadline, true, [] { return false; }));
        uint32_t       count = 1;
        zmq::message_t request(sizeof(count));
        std::memcpy(request.data(), &count, sizeof(count));
        expect(transport.socket().send(request, zmq::send_flags::dontwait).has_value());
        expect(!transport.wait_readable(deadline, true, [] { return false; }));
        const auto elapsed = std::chrono::steady_clock::now() - started;
        expect(elapsed >= 200ms);
        expect(elapsed < 400ms);
        operation.unlock();
        transport.close();
        peer.wait();
    };

    "Runtime lifecycle cancellation interrupts all six block families"_test = [] {
        expect(source_stop_is_prompt<gr::blocks::zeromq::ZmqPullSource<float>>(66));
        expect(source_stop_is_prompt<gr::blocks::zeromq::ZmqReqSource<float>>(67));
        expect(source_stop_is_prompt<gr::blocks::zeromq::ZmqSubSource<float>>(68));
        expect(sink_stop_is_prompt<gr::blocks::zeromq::ZmqPushSink<float>>(69));
        expect(sink_stop_is_prompt<gr::blocks::zeromq::ZmqRepSink<float>>(82));
        expect(sink_stop_is_prompt<gr::blocks::zeromq::ZmqPubSink<float>>(83));
    };

    "Runtime continuous zero replies yield and stop promptly"_test = [] {
        gr::Graph  fg;
        const auto endpoint = endpoint_for(84);
        auto&      req      = fg.emplaceBlock<gr::blocks::zeromq::ZmqReqSource<float>>(make_props({
            {"endpoint", gr::pmt::Value(endpoint)},
            {"timeout", gr::pmt::Value(2000)},
            {"bind", gr::pmt::Value(true)},
            {"linger", gr::pmt::Value(0)},
        }));
        auto&      sink     = fg.emplaceBlock<RecordingScalarSink<float>>();
        expect(fg.connect<"out", "in">(req, sink).has_value());
        std::atomic<bool>                                                    done{false};
        std::atomic<std::size_t>                                             transactions{0};
        const auto                                                           bound_endpoint = TestEndpoint::bound_by(req);
        std::thread                                                          peer([&, bound_endpoint] {
            const auto     resolved_endpoint = bound_endpoint.resolve();
            zmq::context_t ctx{1};
            zmq::socket_t  rep{ctx, zmq::socket_type::rep};
            rep.set(zmq::sockopt::linger, 0);
            rep.set(zmq::sockopt::rcvtimeo, 100);
            rep.connect(resolved_endpoint);
            while (!done.load(std::memory_order_acquire)) {
                zmq::message_t request;
                if (!rep.recv(request)) {
                    continue;
                }
                zmq::message_t reply(0);
                if (rep.send(reply, zmq::send_flags::none)) {
                    ++transactions;
                }
            }
                                                                 });
        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(fg)).has_value());
        const auto started = std::chrono::steady_clock::now();
        std::ignore        = run_until_then_stop(sched, [] { return false; }, 250ms);
        done.store(true, std::memory_order_release);
        peer.join();
        expect(std::chrono::steady_clock::now() - started < 700ms);
        expect(transactions.load(std::memory_order_acquire) > 1);
        expect(sink.received.empty());

        gr::Graph  rep_graph;
        const auto rep_endpoint = endpoint_for(85);
        auto&      rep_source   = rep_graph.emplaceBlock<RepeatingSource<float>>();
        rep_source.value        = 3.f;
        auto& rep               = rep_graph.emplaceBlock<gr::blocks::zeromq::ZmqRepSink<float>>(make_props({
            {"endpoint", gr::pmt::Value(rep_endpoint)},
            {"timeout", gr::pmt::Value(2000)},
            {"bind", gr::pmt::Value(true)},
            {"linger", gr::pmt::Value(0)},
        }));
        expect(rep_graph.connect<"out", "in">(rep_source, rep).has_value());
        std::atomic<bool>                                                    client_done{false};
        std::atomic<std::size_t>                                             rep_transactions{0};
        const auto                                                           rep_bound_endpoint = TestEndpoint::bound_by(rep);
        std::thread                                                          client([&, rep_bound_endpoint] {
            const auto     resolved_endpoint = rep_bound_endpoint.resolve();
            zmq::context_t ctx{1};
            zmq::socket_t  req_socket{ctx, zmq::socket_type::req};
            req_socket.set(zmq::sockopt::linger, 0);
            req_socket.set(zmq::sockopt::rcvtimeo, 300);
            req_socket.connect(resolved_endpoint);
            while (!client_done.load(std::memory_order_acquire)) {
                uint32_t       count = 0;
                zmq::message_t request(sizeof(count));
                std::memcpy(request.data(), &count, sizeof(count));
                if (!req_socket.send(request, zmq::send_flags::none)) {
                    break;
                }
                zmq::message_t reply;
                if (!req_socket.recv(reply)) {
                    break;
                }
                ++rep_transactions;
            }
                                                                 });
        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> rep_sched;
        expect(rep_sched.exchange(std::move(rep_graph)).has_value());
        const auto rep_started = std::chrono::steady_clock::now();
        expect(run_until_then_stop(rep_sched, [&] { return rep_transactions.load(std::memory_order_acquire) >= 4; }, 1s));
        client_done.store(true, std::memory_order_release);
        client.join();
        expect(std::chrono::steady_clock::now() - rep_started < 1500ms);
        expect(rep_transactions.load(std::memory_order_acquire) > 1);
    };

    "Runtime same instance restart creates fresh sockets for all families"_test = [] {
        auto restart = []<typename Block>(Block& block, int first_offset, int second_offset) {
            block.endpoint = endpoint_for(first_offset);
            block.bind     = true;
            block.linger   = 0;
            block.start();
            const auto first_endpoint = block.last_endpoint();
            expect(!first_endpoint.ends_with(":0"));
            block.stop();
            expect(endpoint_can_be_bound(first_endpoint, 1s));
            block.endpoint = endpoint_for(second_offset);
            block.start();
            const auto second_endpoint = block.last_endpoint();
            expect(!second_endpoint.ends_with(":0"));
            expect(second_endpoint != first_endpoint);
            block.stop();
        };
        gr::blocks::zeromq::ZmqPullSource<float> pull;
        gr::blocks::zeromq::ZmqPushSink<float>   push;
        gr::blocks::zeromq::ZmqReqSource<float>  req;
        gr::blocks::zeromq::ZmqRepSink<float>    rep;
        gr::blocks::zeromq::ZmqSubSource<float>  sub;
        gr::blocks::zeromq::ZmqPubSink<float>    pub;
        restart(pull, 70, 71);
        restart(push, 72, 73);
        restart(req, 74, 75);
        restart(rep, 76, 77);
        restart(sub, 78, 79);
        restart(pub, 80, 81);
        expect(req._pending_items.empty());
        expect(!req._req_pending);
        expect(!rep._pending_reply.has_value());
        expect(pull._pending_items.empty());
        expect(sub._pending_items.empty());
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
