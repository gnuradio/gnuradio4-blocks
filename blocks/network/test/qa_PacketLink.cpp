#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <future>
#include <limits>
#include <map>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <zmq.hpp>

#include <gnuradio-4.0/YamlPmt.hpp>
#include <gnuradio-4.0/algorithm/digital/Crc.hpp>
#include <gnuradio-4.0/network/PacketEnvelope.hpp>

// The tier's acceptance gate. Two executables built beside this file are launched as real, separate processes and
// exchange byte packets over a ZeroMQ endpoint: the transmitter cuts a known byte vector into fixed-length records,
// appends a CRC to each and publishes each record as one versioned envelope; the receiver takes envelopes off the
// wire and writes back what it recovered. Nothing here reaches into either process — the assertions are made on the
// files they wrote, on the counters they printed, and on what a raw ZeroMQ socket in this test puts on the wire.
//
// The failure legs are injected from that raw socket, because a fault a peer cannot produce is not a fault the link
// will ever meet. Each one asserts an exact count, and each is followed by traffic the receiver must still accept:
// a refusal that stops the link is as bad as a refusal that is silent.
//
// The last of those legs is a payload corrupted after its CRC was computed, which the transport is right to deliver.
// The receiver converts every packet back to a record and runs it through `CrcCheck`, so the corruption is caught by
// a block on the receiver's own fail port rather than by arithmetic this test does afterwards, and the assertion is
// the count on that port.

extern char** environ;

namespace {

using namespace std::chrono_literals;

constexpr std::uint32_t kRecords = 100U; ///< records the receiver takes and this test asserts on
/// @brief Records the transmitter is asked for, well past what the receiver takes.
///
/// Closing a ZeroMQ socket discards whatever the peer has not yet read, so the transmitter's socket has to still be
/// open when the receiver finishes. It is given far more work than the receiver will consume and is throttled to the
/// receiver's own pace by a one-envelope send queue under backpressure, so it is still sending when the receiver
/// stops, and it is then stopped from here.
constexpr std::uint32_t kOfferedRecords = 4U * kRecords;
constexpr std::uint32_t kRecordBytes    = 250U; ///< payload bytes per record, before the CRC is appended
constexpr std::uint32_t kCrcBytes       = 4U;
constexpr std::uint32_t kPacketBytes    = kRecordBytes + kCrcBytes;

/// @brief The CRC the transmitter's `CrcAppend` is configured for: CRC-32/ISO-HDLC, appended most significant first.
const gr::digital::Crc kRecordCrc{32U, 0x04C11DB7ULL, 0xFFFFFFFFULL, 0xFFFFFFFFULL, true, true};

/// @brief How long a child may take before the test kills it and fails. Both processes normally finish in well under
/// a second; the bound exists so a wedged link fails the suite rather than hanging it.
constexpr std::chrono::milliseconds kChildDeadline = 30s;

// ─── endpoints and workspace ──────────────────────────────────────────────────────────────────────────────────────
// An AF_UNIX path is bounded at about a hundred bytes and a fixed path is a parallel-ctest hazard, so every name is
// short, salted once per process and unique per use, and everything is removed when the test is done with it.

[[nodiscard]] std::string salt() {
    static const std::string value = [] {
        std::random_device device;
        return std::format("{:08x}", device());
    }();
    return value;
}

[[nodiscard]] std::uint32_t nextOrdinal() {
    static std::atomic<std::uint32_t> counter{0U};
    return counter.fetch_add(1U);
}

struct Endpoint {
    std::string path{std::format("{}/gr4-plink-{}-{}.sock", std::filesystem::temp_directory_path().string(), salt(), nextOrdinal())};
    std::string uri{std::format("ipc://{}", path)};

    Endpoint()                           = default;
    Endpoint(const Endpoint&)            = delete;
    Endpoint& operator=(const Endpoint&) = delete;
    ~Endpoint() { std::filesystem::remove(std::filesystem::path(path)); }
};

/// @brief A directory for what the two processes write, removed whole when the case ends.
struct Workspace {
    std::filesystem::path directory{std::filesystem::temp_directory_path() / std::format("gr4-plink-{}-{}", salt(), nextOrdinal())};

    Workspace() { std::filesystem::create_directories(directory); }
    Workspace(const Workspace&)            = delete;
    Workspace& operator=(const Workspace&) = delete;
    ~Workspace() {
        std::error_code ignored;
        std::filesystem::remove_all(directory, ignored);
    }

    [[nodiscard]] std::string file(std::string_view name) const { return (directory / name).string(); }
};

// ─── running the two executables as real processes ────────────────────────────────────────────────────────────────

struct Child {
    ::pid_t     pid = -1;
    std::string label;

    Child(std::string_view executable, const std::vector<std::string>& arguments, const std::string& logFile) : label(executable) {
        ::posix_spawn_file_actions_t actions;
        ::posix_spawn_file_actions_init(&actions);
        ::posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, logFile.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
        ::posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);

        std::vector<std::string> owned;
        owned.reserve(arguments.size() + 1UZ);
        owned.emplace_back(executable);
        owned.insert(owned.end(), arguments.begin(), arguments.end());
        std::vector<char*> argv;
        argv.reserve(owned.size() + 1UZ);
        for (std::string& argument : owned) {
            argv.push_back(argument.data());
        }
        argv.push_back(nullptr);

        const int spawned = ::posix_spawn(&pid, std::string(executable).c_str(), &actions, nullptr, argv.data(), environ);
        ::posix_spawn_file_actions_destroy(&actions);
        boost::ut::expect(spawned == 0) << "cannot launch" << executable << ":" << std::strerror(spawned);
        if (spawned != 0) {
            pid = -1;
        }
    }

    Child(const Child&)            = delete;
    Child& operator=(const Child&) = delete;

    /// @brief Ask the child to stop, which is how the transmitter is ended once the receiver has what it came for.
    void requestStop() const {
        if (pid >= 0) {
            ::kill(pid, SIGTERM);
        }
    }

    /// @brief The child's exit status, or a negative signal number when the deadline had to end it.
    [[nodiscard]] int wait(std::chrono::milliseconds deadline = kChildDeadline) {
        if (pid < 0) {
            return -1;
        }
        const ::pid_t child  = pid;
        auto          reaped = std::async(std::launch::async, [child] {
            int status = 0;
            ::waitpid(child, &status, 0);
            return status;
        });
        if (reaped.wait_for(deadline) != std::future_status::ready) {
            ::kill(child, SIGKILL);
            boost::ut::expect(false) << label << "did not finish within the deadline";
        }
        const int status = reaped.get();
        pid              = -1;
        return WIFEXITED(status) ? WEXITSTATUS(status) : -WTERMSIG(status);
    }

    ~Child() {
        if (pid >= 0) {
            ::kill(pid, SIGKILL);
            int status = 0;
            ::waitpid(pid, &status, 0);
        }
    }
};

// ─── what the processes wrote ─────────────────────────────────────────────────────────────────────────────────────

/// @brief One process's report, as the `key=value` lines it prints and writes. Repeated keys keep every value.
struct Report {
    std::multimap<std::string, std::string, std::less<>> entries;

    [[nodiscard]] std::string text(std::string_view key) const {
        const auto found = entries.find(key);
        return found == entries.end() ? std::string{} : found->second;
    }

    [[nodiscard]] std::uint64_t number(std::string_view key) const {
        const std::string value = text(key);
        return value.empty() ? std::numeric_limits<std::uint64_t>::max() : std::strtoull(value.c_str(), nullptr, 10);
    }

    [[nodiscard]] std::vector<std::string> all(std::string_view key) const {
        std::vector<std::string> values;
        const auto [first, last] = entries.equal_range(key);
        for (auto it = first; it != last; ++it) {
            values.push_back(it->second);
        }
        return values;
    }
};

[[nodiscard]] Report readReport(const std::string& path) {
    Report        report;
    std::ifstream in(path);
    std::string   line;
    while (std::getline(in, line)) {
        // a line is either `key=value` or a space-separated run of them, which is how the per-packet lines are shaped
        std::size_t at = 0UZ;
        while (at < line.size()) {
            const std::size_t space  = std::min(line.find(' ', at), line.size());
            const std::size_t equals = line.find('=', at);
            if (equals == std::string::npos || equals > space) {
                break;
            }
            report.entries.emplace(line.substr(at, equals - at), line.substr(equals + 1UZ, space - equals - 1UZ));
            at = space + 1UZ;
        }
    }
    return report;
}

[[nodiscard]] std::vector<std::uint8_t> readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return std::vector<std::uint8_t>((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

// ─── a raw ZeroMQ peer, which is what makes a fault the link cannot produce for itself reachable ──────────────────

struct RawSender {
    zmq::context_t context{1};
    zmq::socket_t  socket;

    explicit RawSender(const std::string& uri) : socket(context, zmq::socket_type::push) {
        socket.set(zmq::sockopt::sndtimeo, 5000);
        socket.set(zmq::sockopt::linger, 5000);
        socket.bind(uri);
    }

    void send(std::span<const std::vector<std::uint8_t>> frames) {
        for (std::size_t i = 0UZ; i < frames.size(); ++i) {
            const zmq::send_flags flags = i + 1UZ < frames.size() ? zmq::send_flags::sndmore : zmq::send_flags::none;
            boost::ut::expect(socket.send(zmq::buffer(frames[i]), flags).has_value()) << "the raw sender could not place frame" << i;
        }
    }
};

[[nodiscard]] std::vector<std::uint8_t> bytesOf(std::string_view text) { //
    return std::vector<std::uint8_t>(reinterpret_cast<const std::uint8_t*>(text.data()), reinterpret_cast<const std::uint8_t*>(text.data()) + text.size());
}

/// @brief Recompute the header CRC after an edit, so a field refusal is reached rather than the CRC catching it first.
void reseal(std::vector<std::uint8_t>& header) {
    const std::uint32_t crc = static_cast<std::uint32_t>(gr::network::kHeaderCrc.compute(std::span<const std::uint8_t>(header.data(), 28UZ)));
    for (std::size_t i = 0UZ; i < 4UZ; ++i) {
        header[28UZ + i] = static_cast<std::uint8_t>((crc >> (8U * i)) & 0xFFU);
    }
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

/// @brief The metadata the transmitter's records carry, as a peer would have to state it.
[[nodiscard]] std::string recordMetadata(std::uint64_t sequence, std::uint64_t crcValue) {
    gr::property_map map;
    map.insert_or_assign(gr::property_map::key_type("crc_value"), gr::pmt::Value(crcValue));
    map.insert_or_assign(gr::property_map::key_type("crc_width"), gr::pmt::Value(std::uint64_t{32ULL}));
    map.insert_or_assign(gr::property_map::key_type("protocol"), gr::pmt::Value(std::string("packet_link")));
    map.insert_or_assign(gr::property_map::key_type("sequence"), gr::pmt::Value(sequence));
    map.insert_or_assign(gr::property_map::key_type("source_id"), gr::pmt::Value(std::string("raw_peer")));
    return gr::pmt::yaml::serialize(map);
}

/// @brief A four-frame envelope of @p payload, well formed unless a leg below edits it afterwards.
[[nodiscard]] std::vector<std::vector<std::uint8_t>> envelopeOf(std::span<const std::uint8_t> payload, std::uint64_t sequence, std::uint64_t crcValue) {
    const std::string metadata = recordMetadata(sequence, crcValue);
    return {std::vector<std::uint8_t>{}, headerBytes(6U, 1U, static_cast<std::uint32_t>(payload.size()), static_cast<std::uint32_t>(payload.size()), static_cast<std::uint32_t>(metadata.size())), //
        bytesOf(metadata), std::vector<std::uint8_t>(payload.begin(), payload.end())};
}

// ─── the vector the link is asked to carry ────────────────────────────────────────────────────────────────────────

/// @brief Record @p index of the transmitter's stream: the byte ramp cut at the record boundary, CRC appended.
[[nodiscard]] std::vector<std::uint8_t> expectedRecord(std::uint32_t index) {
    std::vector<std::uint8_t> record(kRecordBytes);
    for (std::uint32_t i = 0U; i < kRecordBytes; ++i) {
        record[i] = static_cast<std::uint8_t>((index * kRecordBytes + i) & 0xFFU);
    }
    const std::uint64_t crc = kRecordCrc.compute(record);
    for (std::uint32_t i = 0U; i < kCrcBytes; ++i) {
        record.push_back(static_cast<std::uint8_t>((crc >> (8U * (kCrcBytes - 1U - i))) & 0xFFU));
    }
    return record;
}

/// @brief The CRC a receiver recomputes over a recovered record, which is what makes the appended field checkable.
[[nodiscard]] std::uint64_t crcOfPayload(std::span<const std::uint8_t> record) { //
    return kRecordCrc.compute(record.first(record.size() - kCrcBytes));
}

[[nodiscard]] std::uint64_t trailingCrc(std::span<const std::uint8_t> record) {
    std::uint64_t value = 0ULL;
    for (std::size_t i = record.size() - kCrcBytes; i < record.size(); ++i) {
        value = (value << 8U) | static_cast<std::uint64_t>(record[i]);
    }
    return value;
}

} // namespace

const boost::ut::suite<"packet link acceptance"> packetLinkTests = [] {
    using namespace boost::ut;

    // The whole contract in one run: a known byte vector leaves one process as records under a CRC and inside a
    // versioned envelope, and arrives at another process built separately, byte for byte, with its type, its length,
    // its metadata and its sequence intact.
    "two processes exchange a known byte vector, byte for byte"_test = [] {
        const Endpoint    endpoint;
        const Workspace   workspace;
        const std::string recoveredFile = workspace.file("recovered.bin");
        const std::string txReport      = workspace.file("tx.report");
        const std::string rxReport      = workspace.file("rx.report");

        // `push` waits for a peer rather than publishing into an empty room, so neither order of these two lines can
        // lose a packet and the counts below are exact whichever process reaches its socket first.
        Child receiver(GR4_PACKET_LINK_RX, {endpoint.uri, std::format("{}", kRecords), recoveredFile, rxReport}, workspace.file("rx.log"));
        Child transmitter(GR4_PACKET_LINK_TX, {endpoint.uri, std::format("{}", kOfferedRecords), std::format("{}", kRecordBytes), txReport}, workspace.file("tx.log"));

        // the receiver ends its own run at the packet it was asked for; the transmitter is ended afterwards, so its
        // socket is open for the whole of the receiver's run and nothing is discarded underneath it
        expect(eq(receiver.wait(), 0)) << "the receiver did not exit cleanly";
        transmitter.requestStop();
        expect(eq(transmitter.wait(), 0)) << "the transmitter did not exit cleanly";

        const Report tx = readReport(txReport);
        const Report rx = readReport(rxReport);

        expect(eq(tx.text("role"), std::string("tx")));
        expect(ge(tx.number("packets_sent"), std::uint64_t{kRecords}));
        expect(eq(tx.number("rejected_packets"), std::uint64_t{0ULL}));
        expect(eq(tx.number("dropped_on_overflow"), std::uint64_t{0ULL}));
        expect(eq(tx.number("send_errors"), std::uint64_t{0ULL}));
        expect(eq(tx.number("records_rejected"), std::uint64_t{0ULL}));
        expect(eq(tx.number("meta_keys_dropped"), std::uint64_t{0ULL}));
        // the record-to-stream branch flattened at least the records the receiver took, at the stated record length
        expect(ge(tx.number("stream_bytes"), std::uint64_t{kRecords} * kPacketBytes));
        expect(eq(tx.number("stream_bytes") % kPacketBytes, std::uint64_t{0ULL}));
        expect(eq(tx.text("scheduler_error"), std::string("none")));

        expect(eq(rx.text("role"), std::string("rx")));
        expect(eq(rx.text("timed_out"), std::string("no")));
        expect(eq(rx.text("scheduler_error"), std::string("none")));
        expect(eq(rx.number("packets_received"), std::uint64_t{kRecords}));
        // the source publishes whatever the wire delivered, and the transmitter's margin is still arriving while the
        // graph tears down, so the published count is the requested one or the margin past it — never less
        expect(ge(rx.number("packets_published"), std::uint64_t{kRecords}));
        expect(ge(rx.number("envelopes_received"), std::uint64_t{kRecords}));
        expect(eq(rx.number("recovered_bytes"), std::uint64_t{kRecords} * kPacketBytes));
        for (const std::string_view counter : {"bad_frame_count", "short_header", "bad_magic", "refused_version", "bad_byte_order", "bad_header_bytes", "bad_header_crc", //
                 "unsupported_item_type", "bad_item_size", "bad_payload_length", "unknown_meta_encoding", "unknown_flags", "item_type_mismatch", "length_mismatch",       //
                 "over_max", "bad_metadata", "refusals_recorded"}) {
            expect(eq(rx.number(counter), std::uint64_t{0ULL})) << "a clean link refused something:" << counter;
        }
        // loss is reconstructed at the receiver from the sequence the sink guarantees, so a lossless run says so
        expect(eq(rx.number("sequence_gaps"), std::uint64_t{0ULL}));
        expect(eq(rx.number("packets_lost"), std::uint64_t{0ULL}));
        expect(eq(rx.number("sequence_resets"), std::uint64_t{0ULL}));
        expect(eq(rx.number("dropped_by_backpressure"), std::uint64_t{0ULL}));
        expect(eq(rx.number("meta_keys_mistyped"), std::uint64_t{0ULL}));

        // the receiver's own check branch: every packet becomes a record again, and a clean link fails no CRC
        expect(eq(rx.number("crc_failed_records"), std::uint64_t{0ULL})) << "a clean link failed a CRC at the receiver";
        expect(eq(rx.number("conversion_rejected"), std::uint64_t{0ULL}));
        expect(eq(rx.number("unconverted_recorded"), std::uint64_t{0ULL}));
        expect(eq(rx.number("conversion_meta_keys_dropped"), std::uint64_t{0ULL})) << "every vocabulary key crossed at the width it is declared";
        expect(eq(rx.number("conversion_names_synthesized"), std::uint64_t{0ULL})) << "the transmitter's own signal name survived the crossing, so none had to be invented";
        expect(eq(rx.number("crc_ok_records"), std::uint64_t{kRecords})) << "every packet the receiver took was checked and passed";
        // The stream leg is asserted as whole records and no more than the checked ones rather than as an equality:
        // a block still holding input when the graph reaches its end is stopped rather than drained, and the stream
        // converter emits one record per call, so it can finish a record short of what the CRC passed to it.
        expect(gt(rx.number("crc_verified_bytes"), std::uint64_t{0ULL}));
        expect(eq(rx.number("crc_verified_bytes") % kPacketBytes, std::uint64_t{0ULL})) << "the stream leg carries whole records";
        expect(le(rx.number("crc_verified_bytes"), rx.number("crc_ok_records") * kPacketBytes));

        const std::vector<std::uint8_t> recovered = readFile(recoveredFile);
        expect(eq(recovered.size(), std::size_t{kRecords} * kPacketBytes)) << "the receiver recovered a different number of bytes";
        if (recovered.size() != std::size_t{kRecords} * kPacketBytes) {
            return;
        }

        // the vector is stated here rather than read back from the transmitter, so a link that is self-consistently
        // wrong is caught: the payload is the byte ramp cut at the record boundary and the trailing field is the CRC
        // of exactly those bytes
        for (std::uint32_t record = 0U; record < kRecords; ++record) {
            const std::span<const std::uint8_t> arrived(recovered.data() + record * kPacketBytes, kPacketBytes);
            const std::vector<std::uint8_t>     expected = expectedRecord(record);
            expect(std::ranges::equal(arrived, expected)) << "record" << record << "is not the stated vector";
            expect(eq(trailingCrc(arrived), crcOfPayload(arrived))) << "record" << record << "does not carry a CRC over its own payload";
        }

        // the record metadata that has to survive the crossing, read back from what the receiver reported per packet
        const std::vector<std::string> sequences = rx.all("sequence");
        const std::vector<std::string> lengths   = rx.all("items");
        const std::vector<std::string> crcs      = rx.all("crc_value");
        const std::vector<std::string> protocols = rx.all("protocol");
        const std::vector<std::string> sources   = rx.all("source_id");
        expect(eq(sequences.size(), std::size_t{kRecords}));
        expect(eq(crcs.size(), std::size_t{kRecords}));
        if (sequences.size() != std::size_t{kRecords} || crcs.size() != std::size_t{kRecords}) {
            return;
        }
        for (std::uint32_t record = 0U; record < kRecords; ++record) {
            expect(eq(sequences[record], std::format("{}", record))) << "sequence is not continuous at packet" << record;
            expect(eq(lengths[record], std::format("{}", kPacketBytes)));
            expect(eq(crcs[record], std::format("{}", crcOfPayload(std::span<const std::uint8_t>(recovered.data() + record * kPacketBytes, kPacketBytes))))) //
                << "the crc_value that crossed is not the CRC of the payload that crossed, at packet" << record;
            expect(eq(protocols[record], std::string("packet_link")));
            expect(eq(sources[record], std::string("packet_link_tx")));
        }
    };

    // Four faults no correct sender produces, each counted under its own name, each followed by traffic the receiver
    // still accepts. The last accepted packet carries a payload corrupted after its CRC was computed, which the
    // transport is right to deliver and which the CRC that crossed with it is what makes detectable.
    "injected faults are refused by name, counted, and the link continues"_test = [] {
        const Endpoint    endpoint;
        const Workspace   workspace;
        const std::string recoveredFile = workspace.file("recovered.bin");
        const std::string rxReport      = workspace.file("rx.report");

        constexpr std::uint32_t nAccepted = 4U;
        RawSender               peer(endpoint.uri);
        Child                   receiver(GR4_PACKET_LINK_RX, {endpoint.uri, std::format("{}", nAccepted), recoveredFile, rxReport}, workspace.file("rx.log"));

        const std::vector<std::uint8_t> sample = expectedRecord(0U);

        // a single flipped bit in a covered byte, left unsealed: the header CRC runs before any remaining field is
        // believed, so the corruption lands on one name rather than on whichever field the bit happened to be in
        auto corruptHeader = envelopeOf(sample, 1000ULL, crcOfPayload(sample));
        corruptHeader[1UZ][12UZ] ^= 0x01U;
        peer.send(corruptHeader);

        // a version this reader does not implement, sealed so the version is what refuses it
        auto futureVersion      = envelopeOf(sample, 1001ULL, crcOfPayload(sample));
        futureVersion[1UZ][4UZ] = 2U;
        reseal(futureVersion[1UZ]);
        peer.send(futureVersion);

        // a metadata frame that is not a document, with the header's length field agreeing with the frame it describes
        const std::string                              broken      = "[unclosed";
        const std::array<std::vector<std::uint8_t>, 4> badMetadata = {std::vector<std::uint8_t>{}, headerBytes(6U, 1U, static_cast<std::uint32_t>(sample.size()), static_cast<std::uint32_t>(sample.size()), static_cast<std::uint32_t>(broken.size())), bytesOf(broken), sample};
        peer.send(badMetadata);

        // a well-formed envelope of a different item type, which a receiver fixed at one type refuses rather than
        // reinterprets: this is the whole of "unambiguous type" as a fault
        const std::string                              floatMeta = recordMetadata(1003ULL, 0ULL);
        const std::array<std::vector<std::uint8_t>, 4> wrongType = {std::vector<std::uint8_t>{}, headerBytes(10U, 4U, 8U, 32U, static_cast<std::uint32_t>(floatMeta.size())), bytesOf(floatMeta), std::vector<std::uint8_t>(32UZ, 0U)};
        peer.send(wrongType);

        // the traffic the link must still carry, ending with a payload corrupted after its CRC was computed
        std::vector<std::vector<std::uint8_t>> accepted;
        for (std::uint32_t i = 0U; i < nAccepted; ++i) {
            accepted.push_back(expectedRecord(i));
        }
        std::vector<std::uint64_t> statedCrc;
        for (std::uint32_t i = 0U; i < nAccepted; ++i) {
            statedCrc.push_back(crcOfPayload(accepted[i]));
        }
        accepted[nAccepted - 1U][7UZ] ^= 0xFFU; // corrupted in flight: the CRC field still states the original
        for (std::uint32_t i = 0U; i < nAccepted; ++i) {
            peer.send(envelopeOf(accepted[i], 2000ULL + i, statedCrc[i]));
        }

        expect(eq(receiver.wait(), 0)) << "the receiver did not exit cleanly";
        const Report rx = readReport(rxReport);

        expect(eq(rx.text("timed_out"), std::string("no")));
        expect(eq(rx.number("envelopes_received"), std::uint64_t{nAccepted} + 4ULL)) << "an injected message did not reach the receiver at all";
        expect(eq(rx.number("packets_received"), std::uint64_t{nAccepted})) << "the link did not continue past the refusals";
        expect(eq(rx.number("bad_header_crc"), std::uint64_t{1ULL}));
        expect(eq(rx.number("refused_version"), std::uint64_t{1ULL}));
        expect(eq(rx.number("bad_metadata"), std::uint64_t{1ULL}));
        expect(eq(rx.number("item_type_mismatch"), std::uint64_t{1ULL}));
        for (const std::string_view counter : {"bad_frame_count", "short_header", "bad_magic", "bad_byte_order", "bad_header_bytes", "unsupported_item_type", //
                 "bad_item_size", "bad_payload_length", "unknown_meta_encoding", "unknown_flags", "length_mismatch", "over_max"}) {
            expect(eq(rx.number(counter), std::uint64_t{0ULL})) << "an injected fault was counted under the wrong name:" << counter;
        }
        expect(eq(rx.number("sequence_gaps"), std::uint64_t{0ULL})) << "a refused envelope must not look like loss";
        expect(eq(rx.number("packets_lost"), std::uint64_t{0ULL}));

        // the refusals are records, not just counters: the raw bytes reach the reject port with the reason on them
        expect(eq(rx.number("refusals_recorded"), std::uint64_t{4ULL}));
        const std::vector<std::string> reasons = rx.all("reason");
        expect(eq(reasons.size(), std::size_t{4UZ}));
        if (reasons.size() == 4UZ) {
            expect(eq(reasons[0UZ], std::string("bad_header_crc")));
            expect(eq(reasons[1UZ], std::string("future_version")));
            expect(eq(reasons[2UZ], std::string("bad_metadata")));
            expect(eq(reasons[3UZ], std::string("item_type_mismatch")));
        }

        const std::vector<std::uint8_t> recovered = readFile(recoveredFile);
        expect(eq(recovered.size(), std::size_t{nAccepted} * kPacketBytes));
        if (recovered.size() != std::size_t{nAccepted} * kPacketBytes) {
            return;
        }
        for (std::uint32_t i = 0U; i < nAccepted; ++i) {
            const std::span<const std::uint8_t> arrived(recovered.data() + i * kPacketBytes, kPacketBytes);
            expect(std::ranges::equal(arrived, accepted[i])) << "accepted packet" << i << "is not the payload that was sent";
            // the three intact records verify against the CRC that crossed with them; the corrupted one does not, and
            // that difference is the whole value of carrying the CRC across the process boundary
            const bool intact = crcOfPayload(arrived) == statedCrc[i];
            expect(eq(intact, i + 1U != nAccepted)) << "packet" << i << "verified against the wrong expectation";
        }
        const std::vector<std::string> crcs = rx.all("crc_value");
        expect(eq(crcs.size(), std::size_t{nAccepted}));
        if (crcs.size() == std::size_t{nAccepted}) {
            for (std::uint32_t i = 0U; i < nAccepted; ++i) {
                expect(eq(crcs[i], std::format("{}", statedCrc[i]))) << "the crc_value a peer stated did not cross unchanged, at packet" << i;
            }
        }

        // the leg the link could not build before: the damage is detected by a block in the receiver's own graph, the
        // failing record leaves by a fail port exactly once, and the three intact ones go on through
        expect(eq(rx.number("conversion_rejected"), std::uint64_t{0ULL})) << "a corrupted payload is still a convertible packet";
        expect(eq(rx.number("unconverted_recorded"), std::uint64_t{0ULL}));
        expect(eq(rx.number("crc_failed_records"), std::uint64_t{1ULL})) << "the corrupted payload did not leave by the fail port exactly once";
        expect(eq(rx.number("crc_ok_records"), std::uint64_t{nAccepted - 1U})) << "and the link did not carry the other three onward";
        expect(gt(rx.number("crc_verified_bytes"), std::uint64_t{0ULL}));
        expect(eq(rx.number("crc_verified_bytes") % kPacketBytes, std::uint64_t{0ULL}));
        expect(le(rx.number("crc_verified_bytes"), std::uint64_t{nAccepted - 1U} * kPacketBytes));
        // the raw peer states no signal_name, so every record takes the converter's label: a counted, stated fill
        expect(eq(rx.number("conversion_names_synthesized"), std::uint64_t{nAccepted}));

        const std::vector<std::string> failedFlags = rx.all("failed_crc_ok");
        const std::vector<std::string> failedItems = rx.all("failed_items");
        const std::vector<std::string> failedSeq   = rx.all("failed_sequence");
        expect(eq(failedFlags.size(), std::size_t{1UZ}));
        if (failedFlags.size() == 1UZ) {
            expect(eq(failedFlags[0UZ], std::string("false"))) << "the failing record carries the reason it failed";
            expect(eq(failedItems[0UZ], std::format("{}", kPacketBytes))) << "and the whole payload, damage included";
            expect(eq(failedSeq[0UZ], std::format("{}", 2000ULL + nAccepted - 1U))) << "and it is the packet this test corrupted, not another";
        }
    };
};

int main() { /* boost::ut runs the suites above */ }
