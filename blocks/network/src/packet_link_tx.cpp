// The transmitting half of the packet-link acceptance pair: a byte stream becomes fixed-length records, each record
// is protected by a CRC, and each protected record leaves this process as one versioned four-frame ZeroMQ envelope.
// Every block in the graph is a stock block; this file builds the graph, states its settings and reports the block
// counters at exit, and does no signal work of its own.
//
// A second branch flattens the same records back to a byte stream and counts it, so the record length the transport
// branch carries is stated a second time by a block that has nothing to do with the transport.
//
// The run ends on SIGTERM, on end of stream, or on a deadline, whichever comes first, and the normal end is the
// first. Closing a ZeroMQ socket discards whatever the peer has not yet read — the linger period covers what is
// still inside this process, not what is already in the peer's buffers — so a transmitter that stops as soon as it
// has handed over its last envelope truncates the stream by however far the receiver was lagging. The transmitter is
// therefore asked for more records than the receiver will take, is throttled to the receiver by a one-envelope send
// queue under backpressure, and is stopped from outside once the receiver has finished.

#include <atomic>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <fstream>
#include <print>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <pthread.h>
#include <signal.h>
#include <time.h>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>

#include <gnuradio-4.0/basic/DataSetToPacket.hpp>
#include <gnuradio-4.0/basic/DataSetToStream.hpp>
#include <gnuradio-4.0/basic/StreamToDataSet.hpp>
#include <gnuradio-4.0/digital/CrcBlocks.hpp>
#include <gnuradio-4.0/network/ZmqPacketIO.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace {

using namespace std::chrono_literals;

/// @brief The trigger name the source stamps at every record boundary and the record cutter selects on.
constexpr std::string_view kRecordTrigger = "record";

/// @brief How long the transmitter runs before it stops itself, when nothing else has stopped it.
constexpr std::chrono::milliseconds kRunDeadline = 20s;

/// @brief How long each wait for a stop signal blocks before the deadline and the graph's own state are re-read.
constexpr long kSignalWaitNanoseconds = 50'000'000L;

/// @brief The largest envelope the link admits, well above what any record here produces.
constexpr std::uint64_t kMaxMessageBytes = 1048576ULL;

[[nodiscard]] bool parseNumber(std::string_view text, std::uint32_t& value) {
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}

/// @brief Append one `key=value` line to the report both the process prints and the peer reads.
void report(std::string& text, std::string_view key, std::uint64_t value) { std::format_to(std::back_inserter(text), "{}={}\n", key, value); }

void report(std::string& text, std::string_view key, std::string_view value) { std::format_to(std::back_inserter(text), "{}={}\n", key, value); }

} // namespace

int main(int argc, char** argv) {
    if (argc != 5) {
        std::println(stderr, "usage: {} <endpoint> <records> <record_bytes> <report_file>", argc > 0 ? argv[0] : "packet_link_tx");
        return 2;
    }
    const std::string endpoint(argv[1]);
    const std::string reportFile(argv[4]);

    std::uint32_t nRecords    = 0U;
    std::uint32_t recordBytes = 0U;
    if (!parseNumber(argv[2], nRecords) || !parseNumber(argv[3], recordBytes) || nRecords == 0U || recordBytes == 0U) {
        std::println(stderr, "records and record_bytes must be positive integers");
        return 2;
    }

    using namespace gr::blocks;

    // Every thread inherits this mask, so the stop signal is delivered to the wait below rather than to whichever
    // thread the kernel picks. It is installed before any thread exists.
    sigset_t stopSignals;
    sigemptyset(&stopSignals);
    sigaddset(&stopSignals, SIGTERM);
    sigaddset(&stopSignals, SIGINT);
    if (pthread_sigmask(SIG_BLOCK, &stopSignals, nullptr) != 0) {
        std::println(stderr, "packet_link_tx: cannot block the stop signals");
        return 3;
    }

    gr::Graph graph;

    // A deterministic byte ramp: sample i is i modulo 256, so the vector a peer must recover is stated by the sample
    // index alone. One trigger tag per record boundary is what turns that stream into records downstream.
    auto& source = graph.emplaceBlock<testing::TagSource<std::uint8_t, testing::ProcessFunction::USE_PROCESS_BULK>>({
        {"name", std::string("byte ramp")},
        {"n_samples_max", nRecords * recordBytes},
        {"sample_rate", 1.0e6f},
        {"signal_name", std::string("packet_link")},
        {"signal_unit", std::string("byte")},
        {"mark_tag", false},
        {"repeat_tags", false},
        {"verbose_console", false},
    });
    source._tags.reserve(nRecords);
    for (std::uint32_t record = 0U; record < nRecords; ++record) {
        gr::property_map map;
        map.insert_or_assign(gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), gr::pmt::Value(std::string(kRecordTrigger)));
        source._tags.emplace_back(static_cast<std::size_t>(record) * static_cast<std::size_t>(recordBytes), std::move(map));
    }

    // One trigger name and no stop name is the cutter's single-trigger mode: the record is the trigger sample plus
    // `n_post` - 1 samples after it, and `n_max` bounds what a record may ever become.
    auto& records = graph.emplaceBlock<basic::StreamToDataSet<std::uint8_t>>({
        {"name", std::string("record cutter")},
        {"filter", std::string(kRecordTrigger)},
        {"n_pre", 0U},
        {"n_post", recordBytes},
        {"n_max", recordBytes},
        {"signal_name", std::string("packet_link")},
        {"signal_unit", std::string("byte")},
    });

    // CRC-32/ISO-HDLC over the whole record, appended most significant byte first, with the computed value and the
    // width written into the record's metadata so a receiver is told what to expect by the stream.
    auto& crc = graph.emplaceBlock<digital::CrcAppend>({
        {"name", std::string("record crc")},
        {"width", 32U},
        {"crc_byte_order", std::string("big")},
    });

    auto& packets = graph.emplaceBlock<basic::DataSetToPacket<std::uint8_t>>({
        {"name", std::string("record to packet")},
        {"signal_index", 0U},
        {"protocol_label", std::string("packet_link")},
        {"source_label", std::string("packet_link_tx")},
    });

    // `push` is the lossless pattern and is what makes the run's counts exact: it round-robins rather than fanning
    // out, it discards nothing, and it waits for a peer instead of publishing into an empty room, so the receiver may
    // start before or after this process. A one-envelope send queue under backpressure then couples the whole graph
    // to the receiver's own pace, which bounds how far ahead of the receiver this process can get.
    auto& wire = graph.emplaceBlock<network::ZmqPacketSink<std::uint8_t>>({
        {"name", std::string("packet wire")},
        {"endpoint", endpoint},
        {"bind", true},
        {"pattern", std::string("push")},
        {"overflow", std::string("backpressure")},
        {"queue_messages", 1U},
        {"send_hwm", 16U},
        {"max_message_bytes", kMaxMessageBytes},
    });

    // The reference branch: the same records, flattened back to bytes and counted. It reads the records the transport
    // branch reads, so the byte count it reports is a second statement about what the wire carried.
    auto& flatten = graph.emplaceBlock<basic::DataSetToStream<std::uint8_t>>({
        {"name", std::string("record to stream")},
        {"signal_index", 0U},
        {"boundary_label", std::string("")},
    });
    auto& counted = graph.emplaceBlock<testing::CountingSink<std::uint8_t>>({
        {"name", std::string("sent bytes")},
        {"n_samples_max", 0U},
    });

    const auto connect = [](auto result, std::string_view what) {
        if (!result.has_value()) {
            std::println(stderr, "packet_link_tx: cannot connect {}: {}", what, result.error());
            std::exit(3);
        }
    };
    connect(graph.connect<"out", "in">(source, records), "the source to the record cutter");
    connect(graph.connect<"out", "in">(records, crc), "the record cutter to the CRC");
    connect(graph.connect<"out", "in">(crc, packets), "the CRC to the packet converter");
    connect(graph.connect<"out", "in">(packets, wire), "the packet converter to the wire");
    connect(graph.connect<"out", "in">(crc, flatten), "the CRC to the stream converter");
    connect(graph.connect<"out", "in">(flatten, counted), "the stream converter to the byte counter");

    gr::scheduler::Simple<> scheduler;
    if (const auto exchanged = scheduler.exchange(std::move(graph)); !exchanged.has_value()) {
        std::println(stderr, "packet_link_tx: cannot initialize the scheduler: {}", exchanged.error());
        return 3;
    }

    std::atomic<bool> finished{false};
    std::string       failure;
    std::thread       worker([&scheduler, &failure, &finished] {
        const auto result = scheduler.runAndWait();
        if (!result.has_value()) {
            failure = std::format("{}", result.error());
        }
        finished.store(true, std::memory_order_release);
    });

    // Wait for whichever comes first: the stop signal, the graph finishing on its own, or the deadline. Each wait
    // blocks in the kernel for a bounded time, so the loop costs nothing while the graph runs.
    const auto deadline = std::chrono::steady_clock::now() + kRunDeadline;
    bool       signaled = false;
    while (!finished.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        const timespec wait{.tv_sec = 0, .tv_nsec = kSignalWaitNanoseconds};
        const int      received = sigtimedwait(&stopSignals, nullptr, &wait);
        if (received > 0) {
            signaled = true;
            break;
        }
    }
    const bool timedOut = !signaled && !finished.load(std::memory_order_acquire);
    scheduler.requestStop();
    worker.join();

    std::string text;
    report(text, "role", "tx");
    report(text, "records_offered", nRecords);
    report(text, "record_bytes", recordBytes);
    report(text, "packets_sent", wire.nPacketsSent);
    report(text, "bytes_sent", wire.nBytesSent);
    report(text, "rejected_packets", wire.nRejectedPackets);
    report(text, "dropped_on_overflow", wire.nDroppedOnOverflow);
    report(text, "sequence_declined", wire.nSequenceDeclined);
    report(text, "meta_keys_mistyped", wire.nMetaKeysMistyped);
    report(text, "dropped_at_stop", wire.nDroppedAtStop);
    report(text, "send_errors", wire.nSendErrors);
    report(text, "records_rejected", packets.nRejectedRecords);
    report(text, "meta_keys_dropped", packets.nMetaKeysDropped);
    report(text, "timing_events_dropped", packets.nDroppedTimingEvents);
    report(text, "stream_bytes", counted.count);
    report(text, "stopped_by", signaled ? std::string_view("signal") : (timedOut ? std::string_view("deadline") : std::string_view("end_of_stream")));
    report(text, "scheduler_error", failure.empty() ? std::string_view("none") : std::string_view(failure));
    std::print("{}", text);

    std::ofstream out(reportFile, std::ios::binary | std::ios::trunc);
    out << text;
    out.close();
    if (!out) {
        std::println(stderr, "packet_link_tx: cannot write the report to '{}'", reportFile);
        return 3;
    }
    if (!failure.empty()) {
        return 4;
    }
    return timedOut ? 5 : 0;
}
