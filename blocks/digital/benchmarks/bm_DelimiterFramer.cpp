#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/digital/DelimiterFramer.hpp>

namespace {

using gr::blocks::digital::DelimiterFramer;
using Record = gr::DataSet<std::uint8_t>;

constexpr std::size_t kItemsPerCall = 65536UZ;
constexpr std::size_t kRepeats      = 9UZ;

/// The reader and writer spans a `processBulk` needs, with no tag handling: the measurement is the record path.
struct TagSpan : std::span<const gr::Tag> {
    using value_type = gr::Tag;
    bool consume(std::size_t) const noexcept { return true; }
};

struct TagWriteSpan : std::span<gr::Tag> {
    using value_type = gr::Tag;
    void publish(std::size_t) const noexcept {}
};

template<typename T>
struct ReaderSpan : std::span<const T> {
    using value_type = T;

    TagSpan     rawTags{};
    std::size_t streamIndex = 0UZ;
    std::size_t consumed    = 0UZ;
    bool        isConnected = true;
    bool        isSync      = true;

    explicit ReaderSpan(std::span<const T> items) : std::span<const T>(items) {}

    bool consume(std::size_t nItems) noexcept {
        consumed = nItems;
        return true;
    }
    void consumeTags(std::size_t) noexcept {}

    [[nodiscard]] std::vector<std::pair<std::ptrdiff_t, std::reference_wrapper<const gr::property_map>>> tags() const { return {}; }
    [[nodiscard]] std::vector<std::pair<std::ptrdiff_t, std::reference_wrapper<const gr::property_map>>> tags(std::size_t) const { return {}; }
};

template<typename T>
struct WriterSpan : std::span<T> {
    using value_type = T;

    TagWriteSpan tags{};
    std::size_t  streamIndex = 0UZ;
    std::size_t  count       = 0UZ;
    bool         isConnected = true;
    bool         isSync      = true;

    explicit WriterSpan(std::span<T> items, bool connected = true) : std::span<T>(items), isConnected(connected) {}

    void publish(std::size_t nItems) noexcept { count = nItems; }
    void publishTag(const gr::property_map&, std::size_t = 0UZ) noexcept {}
};

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
};

/// One call's worth of payload, cut into records of @p frameItems items each, at @p mask bits an item.
[[nodiscard]] std::vector<Record> records(std::size_t frameItems, std::uint64_t mask) {
    Rng                 rng;
    std::vector<Record> batch;
    batch.reserve(kItemsPerCall / frameItems);
    while (batch.size() * frameItems + frameItems <= kItemsPerCall) {
        Record record;
        record.signal_values.resize(frameItems);
        for (std::uint8_t& item : record.signal_values) {
            item = static_cast<std::uint8_t>(rng.next() & mask);
        }
        record.extents.push_back(static_cast<std::int32_t>(frameItems));
        record.signal_names.emplace_back("payload");
        record.signal_quantities.emplace_back("");
        record.signal_units.emplace_back("");
        record.meta_information.emplace_back();
        record.timing_events.emplace_back();
        batch.push_back(std::move(record));
    }
    return batch;
}

} // namespace

int main() {
    const std::vector<std::size_t> kFrameLengths{8UZ, 64UZ, 1024UZ};

    std::vector<Record>       framed(kItemsPerCall / kFrameLengths.front() + 1UZ);
    std::vector<std::uint8_t> copyOut(kItemsPerCall);

    // the floor every arm is read against: one span copy of the same item count
    std::vector<gr::blocks::testing::bench::Arm> arms;
    const std::vector<std::uint8_t>              floor(kItemsPerCall, 0x5AU);
    arms.push_back({"span copy, the floor", [&] {
                        std::ranges::copy(floor, copyOut.begin());
                        return static_cast<double>(copyOut[kItemsPerCall / 2UZ]);
                    }});

    struct Mode {
        const char*      label;
        gr::property_map settings;
        std::uint64_t    mask;
    };
    const std::vector<Mode> kModes{
        {"none, 8 bits/item, d=1", {{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}, {"max_payload_items", gr::Size_t{4096}}}, 0xFFULL},
        {"bit_stuffing, 1 bit/item, d=8", {{"end_delimiter", std::string("01111110")}, {"transparency", std::string("bit_stuffing")}, {"max_payload_items", gr::Size_t{4096}}}, 0x01ULL},
        {"byte_escape, 8 bits/item, d=1",
            {{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}, {"transparency", std::string("byte_escape")}, //
                {"escape_item", gr::Size_t{0xDB}}, {"escape_map", std::vector<gr::Size_t>{0xDCU, 0xC0U, 0xDDU, 0xDBU}}, {"max_payload_items", gr::Size_t{4096}}},
            0xFFULL},
    };

    std::vector<std::vector<Record>> batches;
    std::vector<DelimiterFramer>     blocks;
    std::vector<std::string>         labels;
    for (const Mode& mode : kModes) {
        for (const std::size_t frameItems : kFrameLengths) {
            batches.push_back(records(frameItems, mode.mask));
            blocks.push_back(make<DelimiterFramer>(mode.settings));
            labels.push_back(std::format("{}, frame={}", mode.label, frameItems));
        }
    }
    for (std::size_t which = 0UZ; which < blocks.size(); ++which) {
        arms.push_back({labels[which], [&, which] {
                            ReaderSpan<Record> inSpan{std::span<const Record>(batches[which])};
                            WriterSpan<Record> outSpan{std::span<Record>(framed)};
                            std::ignore = blocks[which].processBulk(inSpan, outSpan);
                            return static_cast<double>(outSpan.count);
                        }});
    }

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kItemsPerCall, kRepeats);
}
