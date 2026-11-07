// Socket-independent decoder/placement qualification. IQ publication is always
// through a real GR4 scheduler. This is deliberately NOT a wire-transfer test.
#include <chrono>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/network/StreamPacketIO.hpp>
#include <iostream>
#include <thread>
using C      = std::complex<float>;
using Source = gr::blocks::network::StreamPacketSource<C>;
namespace gr::blocks::network {
struct StreamPacketTestAccess {
    static void inject(Source& source, std::uint64_t seq, std::uint64_t pos, std::size_t count, const std::string& id = "execution-A", bool badTag = false) {
        gr::property_map map{{"sequence", seq}, {"stream_position", pos}, {"stream_id", id}, {"initial_position", std::uint64_t{0}}, {"stream_origin", std::uint64_t{0}}, {"sample_rate", 48000.f}};
        if (badTag) {
            map["packet_tags"] = std::string("malformed");
        }
        auto                        metadata = gr::pmt::yaml::serialize(map);
        gr::network::EnvelopeHeader header;
        header.item_type     = gr::network::kItemTypeCode<C>;
        header.item_size     = sizeof(C);
        header.item_count    = static_cast<std::uint32_t>(count);
        header.payload_bytes = static_cast<std::uint32_t>(count * sizeof(C));
        header.meta_bytes    = static_cast<std::uint32_t>(metadata.size());
        auto           bytes = gr::network::encodeHeader(header);
        std::vector<C> samples(count);
        for (std::size_t i = 0; i < count; ++i) {
            samples[i] = C(float(seq * 100 + i), -float(seq * 100 + i));
        }
        std::array<zmq::message_t, 4> parts{zmq::message_t{}, zmq::message_t(bytes.data(), bytes.size()), zmq::message_t(metadata.data(), metadata.size()), zmq::message_t(samples.data(), samples.size() * sizeof(C))};
        source.handleMessage(parts, 4, bytes.size() + metadata.size() + samples.size() * sizeof(C));
    }
};
} // namespace gr::blocks::network
struct Collector : gr::Block<Collector, gr::NoTagPropagation> {
    gr::PortIn<C> in;
    GR_MAKE_REFLECTABLE(Collector, in);
    std::mutex                                            mutex;
    std::vector<C>                                        samples;
    std::vector<std::pair<std::size_t, gr::property_map>> placements;
    gr::work::Status                                      processBulk(gr::InputSpanLike auto& input) {
        std::lock_guard lock(mutex);
        for (const auto& [offset, tag] : input.tags()) {
            if (offset >= 0 && std::size_t(offset) < input.size()) {
                auto it = tag.get().find("stream_packet");
                if (it != tag.get().end()) {
                    placements.emplace_back(samples.size() + std::size_t(offset), it->second.template value_or(gr::property_map{}));
                }
            }
        }
        samples.insert(samples.end(), input.begin(), input.end());
        return gr::work::Status::OK;
    }
    std::size_t count() {
        std::lock_guard lock(mutex);
        return samples.size();
    }
};
void check(bool b, const char* why) {
    if (!b) {
        throw std::runtime_error(why);
    }
}
std::uint64_t number(const gr::property_map& m, const char* key) { return m.at(key).value_or(std::uint64_t{0}); }
std::string   word(const gr::property_map& m, const char* key) { return std::string(m.at(key).value_or(std::string_view{})); }
int           main() {
    try {
        gr::Graph graph;
        auto&     source       = graph.emplaceBlock<Source>({{"endpoint", std::string("inproc://placement-no-peer")}, {"pattern", std::string("pull")}, {"publish_positions", true}, {"max_message_bytes", std::uint64_t{65536}}, {"queue_messages", 2U}, {"queue_bytes", std::uint64_t{4096}}});
        source.out.max_samples = 3; // every four-sample packet crosses multiple work calls
        auto& sink             = graph.emplaceBlock<Collector>();
        check(graph.connect<"out", "in">(source, sink).has_value(), "connection");
        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> scheduler;
        check(scheduler.exchange(std::move(graph)).has_value(), "graph exchange");
        bool         ok = false;
        std::jthread runner([&] { ok = scheduler.runAndWait().has_value(); });
        struct Stop {
            decltype(scheduler)& s;
            std::jthread&        t;
            ~Stop() {
                s.requestStop();
                if (t.joinable()) {
                    t.join();
                }
            }
        } stop{scheduler, runner};
        auto wait = [&](auto predicate) {
            auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
            while (!predicate()) {
                check(std::chrono::steady_clock::now() < deadline, "progress timeout");
                std::this_thread::yield();
            }
        };
        wait([&] { return scheduler.workerStarted(); });
        std::size_t expected = 0;
        auto        send     = [&](std::uint64_t seq, std::uint64_t pos, const std::string& id = "execution-A", bool bad = false) {
            gr::blocks::network::StreamPacketTestAccess::inject(source, seq, pos, 4, id, bad);
            expected += 4;
            wait([&] { return sink.count() == expected; });
        };
        constexpr std::uint64_t big = 9007199254740995ULL;
        send(7, big);
        send(9, big + 8);
        send(12, big + 15);
        send(14, big + 19); // sequence-only discontinuity last
        send(0, 0);
        send(1, 0);
        send(2, 4, "execution-B");
        send(3, 8, "execution-B", true);
        send(5, 8 + (1ULL << 33), "execution-B");
        {
            scheduler.requestWorkQuiescence();
            struct Release {
                decltype(scheduler)& s;
                ~Release() { s.releaseWorkQuiescence(); }
            } release{scheduler};
            // Three decoded packets into depth two: the existing queue drops oldest.
            for (std::uint64_t seq = 6; seq < 9; ++seq) {
                gr::blocks::network::StreamPacketTestAccess::inject(source, seq, 8 + (1ULL << 33) + 4 * (seq - 4), 4, "execution-B");
            }
            std::lock_guard lock(source._mutex);
            check(source._queue.size() == 2 && source._queuedBytes <= 4096, "receive queue unbounded");
        }
        expected += 8;
        wait([&] { return sink.count() == expected; });
        scheduler.requestStop();
        runner.join();
        check(ok, "scheduler failed");
        const auto& p = sink.placements;
        check(p.size() == 11, "packet placement tags missing or duplicated across chunks");
        for (std::size_t i = 0; i < p.size(); ++i) {
            check(p[i].first == 4 * i, "placement moved");
            check(number(p[i].second, "sample_count") == 4, "wrong packet count");
            auto seq = number(p[i].second, "sequence");
            for (std::size_t k = 0; k < 4; ++k) {
                check(sink.samples[4 * i + k] == C(float(seq * 100 + k), -float(seq * 100 + k)), "IQ changed");
            }
        }
        check(number(p[0].second, "position") == big && word(p[0].second, "event") == "initial", "initial precision/placement");
        check(p[0].second.at("unavailable_before").value_or(false), "late attachment hidden");
        check(number(p[1].second, "missing_begin") == big + 4 && number(p[1].second, "missing_end") == big + 8, "first gap");
        check(number(p[2].second, "missing_begin") == big + 12 && number(p[2].second, "missing_end") == big + 15, "disjoint partial chunk gap");
        check(number(p[3].second, "missing_begin") == number(p[3].second, "missing_end") && number(p[3].second, "packets_lost") == 1, "sequence-only loss");
        check(word(p[4].second, "event") == "sequence_reset", "reset hidden");
        check(word(p[5].second, "event") == "stream_rewind", "rewind hidden");
        check(word(p[6].second, "event") == "stream_changed" && word(p[6].second, "stream_id") == "execution-B", "generation hidden");
        check(number(p[7].second, "tags_malformed") == 1 && word(p[7].second, "event") == "contiguous", "tag loss confused with sample loss");
        check(number(p[8].second, "missing_end") - number(p[8].second, "missing_begin") == (1ULL << 33) - 4, "large gap truncated");
        check(number(p[9].second, "missing_end") - number(p[9].second, "missing_begin") == 8, "overflow gap wrong");
        check(source.counters().droppedByBackpressure == 1, "overflow accounting");
        std::cout << "PASS: 11 indexed packets, 44 bit-identical samples; initial >2^53, disjoint/partial gaps, sequence-only loss, reset, rewind, identity change, tag loss, >2^32 gap, bounded receive overflow; real GR4 scheduling, decoder injection (no wire qualification)\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
