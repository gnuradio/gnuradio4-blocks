#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <array>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/digital/CrcBlocks.hpp>

namespace {

using gr::blocks::digital::CrcAppend;
using gr::blocks::digital::CrcCheck;
using Record = gr::DataSet<std::uint8_t>;

constexpr std::size_t kBytesPerCall = 65536UZ;
constexpr std::size_t kRepeats      = 9UZ;

constexpr std::array<gr::Size_t, 4UZ>  kWidths{8U, 16U, 32U, 64U};
constexpr std::array<std::size_t, 3UZ> kLengths{16UZ, 256UZ, 4096UZ};

/// The two span shapes a record-oriented processBulk needs. The tag members are empty by
/// construction: neither block reads or writes a stream tag, and a `DataSet` carries its annotation in its own map.
struct TagSpan : std::span<const gr::Tag> {
    using value_type = gr::Tag;
    bool consume(std::size_t) const noexcept { return true; }
};

struct TagWriteSpan : std::span<gr::Tag> {
    using value_type = gr::Tag;
    void publish(std::size_t) const noexcept {}
};

struct ReaderSpan : std::span<const Record> {
    using value_type = Record;

    TagSpan     rawTags{};
    std::size_t consumed    = 0UZ;
    bool        isConnected = true;
    bool        isSync      = true;

    explicit ReaderSpan(std::span<const Record> records) : std::span<const Record>(records) {}

    bool consume(std::size_t nRecords) noexcept {
        consumed = nRecords;
        return true;
    }
    void consumeTags(std::size_t) noexcept {}

    [[nodiscard]] std::span<const gr::Tag> tags() const noexcept { return rawTags; }
    [[nodiscard]] std::span<const gr::Tag> tags(std::size_t) const noexcept { return rawTags; }
};

struct WriterSpan : std::span<Record> {
    using value_type = Record;

    TagWriteSpan tags{};
    std::size_t  count       = 0UZ;
    bool         isConnected = true;
    bool         isSync      = true;

    explicit WriterSpan(std::span<Record> records) : std::span<Record>(records) {}

    void publish(std::size_t nRecords) noexcept { count = nRecords; }
    void publishTag(const gr::property_map&, std::size_t = 0UZ) noexcept {}
};

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

[[nodiscard]] gr::property_map settingsOf(gr::Size_t width, bool reflected) { return {{"width", width}, {"poly", std::uint64_t{0x04C11DB7}}, {"initial_value", std::uint64_t{0xFFFFFFFF}}, {"final_xor", std::uint64_t{0xFFFFFFFF}}, {"input_reflected", reflected}, {"result_reflected", reflected}}; }

[[nodiscard]] std::vector<Record> records(std::size_t length) {
    std::vector<Record> batch(kBytesPerCall / length);
    std::uint64_t       state = 0x9e3779b97f4a7c15ULL;
    for (Record& record : batch) {
        record.signal_values.resize(length);
        for (std::uint8_t& byte : record.signal_values) {
            state ^= state << 13U;
            state ^= state >> 7U;
            state ^= state << 17U;
            byte = static_cast<std::uint8_t>(state & 0xFFU);
        }
        record.extents.push_back(static_cast<std::int32_t>(length));
        record.signal_names.emplace_back("payload");
        record.meta_information.emplace_back();
        record.timing_events.emplace_back();
    }
    return batch;
}

[[nodiscard]] std::vector<Record> protect(CrcAppend& block, const std::vector<Record>& input) {
    std::vector<Record> output(input.size());
    ReaderSpan          inSpan{std::span<const Record>(input)};
    WriterSpan          outSpan{std::span<Record>(output)};
    std::ignore = block.processBulk(inSpan, outSpan);
    output.resize(outSpan.count);
    return output;
}

} // namespace

int main() {
    std::vector<std::vector<Record>> payloads;
    for (const std::size_t length : kLengths) {
        payloads.push_back(records(length));
    }

    struct Arm {
        gr::Size_t  width;
        bool        reflected;
        std::size_t lengthIndex;
    };

    std::vector<Arm>                 plan;
    std::vector<CrcAppend>           appenders;
    std::vector<CrcCheck>            checkers;
    std::vector<std::vector<Record>> guarded;
    for (const gr::Size_t width : kWidths) {
        for (const bool reflected : {false, true}) {
            for (std::size_t li = 0UZ; li < kLengths.size(); ++li) {
                plan.push_back({width, reflected, li});
                appenders.push_back(make<CrcAppend>(settingsOf(width, reflected)));
                checkers.push_back(make<CrcCheck>(settingsOf(width, reflected)));
                guarded.push_back(protect(appenders.back(), payloads[li]));
            }
        }
    }

    std::vector<std::vector<Record>> appendScratch(plan.size());
    std::vector<std::vector<Record>> okScratch(plan.size());
    std::vector<std::vector<Record>> failScratch(plan.size());
    for (std::size_t which = 0UZ; which < plan.size(); ++which) {
        appendScratch[which].resize(payloads[plan[which].lengthIndex].size());
        okScratch[which].resize(guarded[which].size());
        failScratch[which].resize(guarded[which].size());
    }

    std::vector<gr::blocks::testing::bench::Arm> arms;
    for (std::size_t which = 0UZ; which < plan.size(); ++which) {
        const Arm& arm = plan[which];
        arms.push_back({std::format("CrcAppend  w={:<2} {} len={}", arm.width, arm.reflected ? "reflected  " : "unreflected", kLengths[arm.lengthIndex]), [&, which] {
                            const std::vector<Record>& input = payloads[plan[which].lengthIndex];
                            ReaderSpan                 inSpan{std::span<const Record>(input)};
                            WriterSpan                 outSpan{std::span<Record>(appendScratch[which])};
                            std::ignore = appenders[which].processBulk(inSpan, outSpan);
                            return static_cast<double>(appendScratch[which].front().signal_values.back());
                        }});
    }
    for (std::size_t which = 0UZ; which < plan.size(); ++which) {
        const Arm& arm = plan[which];
        arms.push_back({std::format("CrcCheck   w={:<2} {} len={}", arm.width, arm.reflected ? "reflected  " : "unreflected", kLengths[arm.lengthIndex]), [&, which] {
                            ReaderSpan inSpan{std::span<const Record>(guarded[which])};
                            WriterSpan okSpan{std::span<Record>(okScratch[which])};
                            WriterSpan failSpan{std::span<Record>(failScratch[which])};
                            std::ignore = checkers[which].processBulk(inSpan, okSpan, failSpan);
                            return static_cast<double>(okSpan.count);
                        }});
    }

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kBytesPerCall, kRepeats);
}
