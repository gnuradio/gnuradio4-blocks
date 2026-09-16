// The receiving half of the packet-link acceptance pair: versioned four-frame ZeroMQ envelopes become packets, and
// what a packet carried — payload bytes, the record metadata and the sequence — is reported so a peer can check the
// crossing without a packet capture. Every block in the graph is a stock block; this file builds the graph, states
// its settings and writes out what the graph produced.
//
// The packets are also converted back to records and checked: `PacketToDataSet` gives the wire's payload the carrier
// `CrcCheck` reads, so a payload corrupted after its CRC was computed leaves by a fail port in this process rather
// than being a difference a reader has to compute for itself. The passing records go on through `DataSetToStream` to
// a byte counter, which is a second statement of what the link delivered intact.
//
// The graph ends on its own: the packet sink stops the scheduler at the requested count, so the process is a finite
// run rather than a listener a peer has to kill. Refused envelopes take the source's `reject` port to a second sink,
// so a refusal is a record this process reports rather than a message that vanished.

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

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/basic/DataSetToStream.hpp>
#include <gnuradio-4.0/basic/PacketToDataSet.hpp>
#include <gnuradio-4.0/digital/CrcBlocks.hpp>
#include <gnuradio-4.0/network/ZmqPacketIO.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace {

using namespace std::chrono_literals;

/// @brief How long the receiver waits for the requested packets before it stops the graph and reports what it has.
constexpr std::chrono::milliseconds kReceiveDeadline = 20s;

/// @brief The largest envelope the link admits; libzmq is given the same bound, so nothing larger is allocated.
constexpr std::uint64_t kMaxMessageBytes = 1048576ULL;

[[nodiscard]] bool parseNumber(std::string_view text, std::uint32_t& value) {
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}

void report(std::string& text, std::string_view key, std::uint64_t value) { std::format_to(std::back_inserter(text), "{}={}\n", key, value); }

void report(std::string& text, std::string_view key, std::string_view value) { std::format_to(std::back_inserter(text), "{}={}\n", key, value); }

/// @brief A metadata value read at the type the record-metadata vocabulary declares for it, or a stated absence.
[[nodiscard]] std::string textOf(const gr::property_map& map, std::string_view key) {
    const auto entry = map.find(key);
    if (entry == map.end()) {
        return "absent";
    }
    if (const std::pmr::string* text = entry->second.get_if<std::pmr::string>(); text != nullptr) {
        return std::string(text->begin(), text->end());
    }
    return "mistyped";
}

[[nodiscard]] std::string flagOf(const gr::property_map& map, std::string_view key) {
    const auto entry = map.find(key);
    if (entry == map.end()) {
        return "absent";
    }
    if (const bool* value = entry->second.get_if<bool>(); value != nullptr) {
        return *value ? "true" : "false";
    }
    return "mistyped";
}

[[nodiscard]] std::string numberOf(const gr::property_map& map, std::string_view key) {
    const auto entry = map.find(key);
    if (entry == map.end()) {
        return "absent";
    }
    if (const std::uint64_t* value = entry->second.get_if<std::uint64_t>(); value != nullptr) {
        return std::format("{}", *value);
    }
    return "mistyped";
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 5) {
        std::println(stderr, "usage: {} <endpoint> <packets> <recovered_file> <report_file>", argc > 0 ? argv[0] : "packet_link_rx");
        return 2;
    }
    const std::string endpoint(argv[1]);
    const std::string recoveredFile(argv[3]);
    const std::string reportFile(argv[4]);

    std::uint32_t nPackets = 0U;
    if (!parseNumber(argv[2], nPackets) || nPackets == 0U) {
        std::println(stderr, "packets must be a positive integer");
        return 2;
    }

    using namespace gr::blocks;
    using PacketSink = testing::TagSink<gr::Packet<std::uint8_t>, testing::ProcessFunction::USE_PROCESS_ONE>;
    using RecordSink = testing::TagSink<gr::DataSet<std::uint8_t>, testing::ProcessFunction::USE_PROCESS_ONE>;

    gr::Graph graph;

    // `pull` against the transmitter's `push`: the pattern discards nothing and fair-queues, so the packets this
    // process reports are every packet the wire delivered, in the order it delivered them.
    auto& wire = graph.emplaceBlock<network::ZmqPacketSource<std::uint8_t>>({
        {"name", std::string("packet wire")},
        {"endpoint", endpoint},
        {"bind", false},
        {"pattern", std::string("pull")},
        {"max_message_bytes", kMaxMessageBytes},
        {"max_reject_bytes", 256U},
    });

    // The check branch, bounded at the same packet count as the packet sink. The bound is what makes the run finite
    // and the branch's counts exact: the graph ends when every consumer of the wire's output is done, so a branch
    // that ran forever would keep the source alive past the sink that ends the run, and a branch that ran an
    // unpredictable number of packets would report counts nothing could assert.
    auto& bound = graph.emplaceBlock<testing::HeadBlock<gr::Packet<std::uint8_t>>>({
        {"name", std::string("checked packets")},
        {"n_samples_max", nPackets},
    });

    auto& records = graph.emplaceBlock<basic::PacketToDataSet<std::uint8_t>>({
        {"name", std::string("packet to record")},
        {"signal_label", std::string("payload")},
    });

    // The same six-tuple and the same byte order the transmitter's `CrcAppend` used, and it is told nothing else: the
    // CRC that crossed sits in the payload's own trailing bytes, so the check is against what arrived.
    auto& crc = graph.emplaceBlock<digital::CrcCheck>({
        {"name", std::string("record crc")},
        {"width", 32U},
        {"crc_byte_order", std::string("big")},
    });

    auto& flatten     = graph.emplaceBlock<basic::DataSetToStream<std::uint8_t>>({
        {"name", std::string("record to stream")},
        {"signal_index", 0U},
        {"boundary_label", std::string("")},
    });
    auto& verified    = graph.emplaceBlock<testing::CountingSink<std::uint8_t>>({
        {"name", std::string("verified bytes")},
        {"n_samples_max", 0U},
    });
    auto& okRecords   = graph.emplaceBlock<RecordSink>({
        {"name", std::string("verified records")},
        {"n_samples_expected", 0U},
        {"log_tags", false},
        {"log_samples", true},
        {"verbose_console", false},
    });
    auto& failed      = graph.emplaceBlock<RecordSink>({
        {"name", std::string("failed records")},
        {"n_samples_expected", 0U},
        {"log_tags", false},
        {"log_samples", true},
        {"verbose_console", false},
    });
    auto& unconverted = graph.emplaceBlock<PacketSink>({
        {"name", std::string("unconvertible packets")},
        {"n_samples_expected", 0U},
        {"log_tags", false},
        {"log_samples", true},
        {"verbose_console", false},
    });

    // The refusal sink is placed before the packet sink so that a work cycle in which both ports publish services the
    // refusals first: the packet sink is what ends the run, and a refused envelope has to be recorded before it does.
    auto& refusals = graph.emplaceBlock<PacketSink>({
        {"name", std::string("refused envelopes")},
        {"n_samples_expected", 0U},
        {"log_tags", false},
        {"log_samples", true},
        {"verbose_console", false},
    });
    auto& received = graph.emplaceBlock<PacketSink>({
        {"name", std::string("received packets")},
        {"n_samples_expected", nPackets},
        {"log_tags", false},
        {"log_samples", true},
        {"verbose_console", false},
    });

    const auto connect = [](auto result, std::string_view what) {
        if (!result.has_value()) {
            std::println(stderr, "packet_link_rx: cannot connect {}: {}", what, result.error());
            std::exit(3);
        }
    };
    connect(graph.connect<"out", "in">(wire, bound), "the wire to the check branch's bound");
    connect(graph.connect<"out", "in">(bound, records), "the bound to the record converter");
    connect(graph.connect<"out", "in">(records, crc), "the record converter to the CRC");
    connect(graph.connect<"reject", "in">(records, unconverted), "the converter's refusals to their sink");
    connect(graph.connect<"ok", "in">(crc, okRecords), "the verified records to their sink");
    connect(graph.connect<"ok", "in">(crc, flatten), "the verified records to the stream converter");
    connect(graph.connect<"fail", "in">(crc, failed), "the failing records to their sink");
    connect(graph.connect<"out", "in">(flatten, verified), "the stream converter to the byte counter");
    connect(graph.connect<"reject", "in">(wire, refusals), "the wire's refusals to the refusal sink");
    connect(graph.connect<"out", "in">(wire, received), "the wire to the packet sink");

    gr::scheduler::Simple<> scheduler;
    if (const auto exchanged = scheduler.exchange(std::move(graph)); !exchanged.has_value()) {
        std::println(stderr, "packet_link_rx: cannot initialize the scheduler: {}", exchanged.error());
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

    // The sink ends the run at the requested count; the deadline exists so a peer that never sends leaves a report
    // and an exit code behind rather than a process to kill.
    const auto deadline = std::chrono::steady_clock::now() + kReceiveDeadline;
    while (!finished.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::yield();
    }
    const bool timedOut = !finished.load(std::memory_order_acquire);
    scheduler.requestStop();
    worker.join();

    std::vector<std::uint8_t> recovered;
    std::string               packetLines;
    for (std::size_t i = 0UZ; i < received._samples.size(); ++i) {
        const gr::Packet<std::uint8_t>& packet = received._samples[i];
        recovered.insert(recovered.end(), packet.signal_values.begin(), packet.signal_values.end());
        const gr::property_map& map = packet.meta_information.empty() ? gr::property_map{} : packet.meta_information[0UZ];
        std::format_to(std::back_inserter(packetLines), "packet={} items={} sequence={} crc_value={} protocol={} source_id={}\n", //
            i, packet.signal_values.size(), numberOf(map, "sequence"), numberOf(map, "crc_value"), textOf(map, "protocol"), textOf(map, "source_id"));
    }

    std::string text;
    report(text, "role", "rx");
    report(text, "packets_expected", nPackets);
    report(text, "packets_received", received._samples.size());
    report(text, "recovered_bytes", recovered.size());
    report(text, "envelopes_received", wire.nEnvelopesReceived);
    report(text, "packets_published", wire.nPacketsPublished);
    report(text, "bytes_received", wire.nBytesReceived);
    report(text, "bad_frame_count", wire.nBadFrameCount);
    report(text, "short_header", wire.nShortHeader);
    report(text, "bad_magic", wire.nBadMagic);
    report(text, "refused_version", wire.nRefusedVersion);
    report(text, "bad_byte_order", wire.nBadByteOrder);
    report(text, "bad_header_bytes", wire.nBadHeaderBytes);
    report(text, "bad_header_crc", wire.nBadHeaderCrc);
    report(text, "unsupported_item_type", wire.nUnsupportedItemType);
    report(text, "bad_item_size", wire.nBadItemSize);
    report(text, "bad_payload_length", wire.nBadPayloadLength);
    report(text, "unknown_meta_encoding", wire.nUnknownMetaEncoding);
    report(text, "unknown_flags", wire.nUnknownFlags);
    report(text, "item_type_mismatch", wire.nItemTypeMismatch);
    report(text, "length_mismatch", wire.nLengthMismatch);
    report(text, "over_max", wire.nOverMax);
    report(text, "bad_metadata", wire.nBadMetadata);
    report(text, "sequence_gaps", wire.nSequenceGaps);
    report(text, "packets_lost", wire.nPacketsLost);
    report(text, "sequence_resets", wire.nSequenceResets);
    report(text, "sources_untracked", wire.nSourcesUntracked);
    report(text, "dropped_by_backpressure", wire.nDroppedByBackpressure);
    report(text, "meta_keys_mistyped", wire.nMetaKeysMistyped);
    report(text, "timestamps_carried", wire.nTimestampsCarried);
    report(text, "refusals_recorded", refusals._samples.size());
    report(text, "conversion_rejected", records.nRejectedPackets);
    report(text, "conversion_meta_keys_dropped", records.nMetaKeysDropped);
    report(text, "conversion_names_synthesized", records.nSignalNamesSynthesized);
    report(text, "unconverted_recorded", unconverted._samples.size());
    report(text, "crc_ok_records", okRecords._samples.size());
    report(text, "crc_failed_records", failed._samples.size());
    report(text, "crc_verified_bytes", static_cast<std::uint64_t>(verified.count));
    for (std::size_t i = 0UZ; i < refusals._samples.size(); ++i) {
        const gr::Packet<std::uint8_t>& refused = refusals._samples[i];
        const gr::property_map&         map     = refused.meta_information.empty() ? gr::property_map{} : refused.meta_information[0UZ];
        std::format_to(std::back_inserter(text), "refusal={} reason={} bytes_kept={}\n", i, textOf(map, "discard_reason"), numberOf(map, "envelope_bytes_kept"));
    }
    // The keys here are spelled apart from the per-packet ones deliberately: a reader collecting every `sequence` in
    // this report is collecting what the link delivered, and a failing record must not be counted twice in it.
    for (std::size_t i = 0UZ; i < failed._samples.size(); ++i) {
        const gr::DataSet<std::uint8_t>& record = failed._samples[i];
        const gr::property_map&          map    = record.meta_information.empty() ? gr::property_map{} : record.meta_information[0UZ];
        std::format_to(std::back_inserter(text), "crc_failure={} failed_items={} failed_crc_ok={} failed_sequence={}\n", i, record.signal_values.size(), flagOf(map, "crc_ok"), numberOf(map, "sequence"));
    }
    text += packetLines;
    report(text, "timed_out", timedOut ? std::string_view("yes") : std::string_view("no"));
    report(text, "scheduler_error", failure.empty() ? std::string_view("none") : std::string_view(failure));
    std::print("{}", text);

    std::ofstream payload(recoveredFile, std::ios::binary | std::ios::trunc);
    payload.write(reinterpret_cast<const char*>(recovered.data()), static_cast<std::streamsize>(recovered.size()));
    payload.close();
    if (!payload) {
        std::println(stderr, "packet_link_rx: cannot write the recovered bytes to '{}'", recoveredFile);
        return 3;
    }
    std::ofstream out(reportFile, std::ios::binary | std::ios::trunc);
    out << text;
    out.close();
    if (!out) {
        std::println(stderr, "packet_link_rx: cannot write the report to '{}'", reportFile);
        return 3;
    }
    if (!failure.empty()) {
        return 4;
    }
    return timedOut ? 5 : 0;
}
