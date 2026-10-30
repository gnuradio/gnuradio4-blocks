#include <boost/ut.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/digital/DelimiterExtractor.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::digital::DelimiterExtractor;
using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;

template<typename T>
using Record = gr::DataSet<T>;

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
    [[nodiscard]] std::size_t below(std::size_t bound) noexcept { return next() % bound; }
};

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

/// One item per '0'/'1' character.
[[nodiscard]] std::vector<std::uint8_t> itemsOf(std::string_view bits) {
    std::vector<std::uint8_t> items;
    items.reserve(bits.size());
    for (const char character : bits) {
        items.push_back(static_cast<std::uint8_t>(character == '1' ? 1 : 0));
    }
    return items;
}

/// What one drive of the block produced on each of its two ports.
template<typename T>
struct Extracted {
    std::vector<Record<T>> out{};
    std::vector<Record<T>> reject{};
    std::size_t            consumed = 0UZ;
};

/**
 * @brief Drives the block over @p input in windows of @p chunk items, with @p tags planted at absolute offsets.
 *
 * Tags from up to 32 items before the window are presented again at a negative relative index, which is what the
 * framework does to a block holding a partial record across calls, so the filter that skips them is exercised.
 */
template<typename T>
[[nodiscard]] Extracted<T> feed(DelimiterExtractor<T>& block, std::span<const T> input, std::size_t chunk = 0UZ, std::span<const gr::Tag> tags = {}, bool rejectConnected = true, std::size_t room = 64UZ) {
    Extracted<T>           result;
    const std::size_t      stride = chunk == 0UZ ? std::max(input.size(), 1UZ) : chunk;
    std::vector<Record<T>> outScratch(room);
    std::vector<Record<T>> rejectScratch(room);

    for (std::size_t base = 0UZ; base < input.size();) {
        const std::size_t count    = std::min(stride, input.size() - base);
        const std::size_t lookback = base > 32UZ ? base - 32UZ : 0UZ;
        const auto        first    = std::ranges::lower_bound(tags, lookback, std::ranges::less{}, &gr::Tag::index);
        const auto        last     = std::ranges::lower_bound(tags, base + count, std::ranges::less{}, &gr::Tag::index);

        InputSpan<T>          inSpan(input.subspan(base, count), base, std::span<const gr::Tag>(first, last));
        OutputSpan<Record<T>> outSpan(std::span<Record<T>>(outScratch), 0UZ, nullptr, true);
        OutputSpan<Record<T>> rejectSpan(rejectConnected ? std::span<Record<T>>(rejectScratch) : std::span<Record<T>>{}, 0UZ, nullptr, rejectConnected);

        std::ignore = block.processBulk(inSpan, outSpan, rejectSpan);

        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            result.out.push_back(std::move(outScratch[k]));
        }
        for (std::size_t k = 0UZ; k < rejectSpan.count; ++k) {
            result.reject.push_back(std::move(rejectScratch[k]));
        }
        result.consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && rejectSpan.count == 0UZ) {
            break;
        }
        base += inSpan.consumed;
    }
    return result;
}

template<typename T>
[[nodiscard]] const gr::property_map& metaOf(const Record<T>& record) {
    static const gr::property_map kEmpty{};
    return record.meta_information.empty() ? kEmpty : record.meta_information.front();
}

template<typename T>
[[nodiscard]] std::uint64_t metaU64(const Record<T>& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? ~0ULL : entry->second.value_or(std::uint64_t{~0ULL});
}

template<typename T>
[[nodiscard]] gr::Size_t metaSize(const Record<T>& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::numeric_limits<gr::Size_t>::max() : entry->second.value_or(std::numeric_limits<gr::Size_t>::max());
}

template<typename T>
[[nodiscard]] std::string metaString(const Record<T>& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::string{} : std::string(entry->second.value_or(std::string_view{}));
}

/// The HDLC framing of anchor A over bit items.
[[nodiscard]] gr::property_map hdlcFraming(gr::Size_t bound) { return {{"end_delimiter", std::string("01111110")}, {"transparency", std::string("bit_stuffing")}, {"max_payload_items", bound}}; }

/// `feed` for the boundary sweep, which produces tens of thousands of records and needs none of them kept.
template<typename T>
[[nodiscard]] std::size_t feedCounting(DelimiterExtractor<T>& block, std::span<const T> input, std::size_t chunk, std::size_t room) {
    const std::size_t      stride = chunk == 0UZ ? std::max(input.size(), 1UZ) : chunk;
    std::vector<Record<T>> outScratch(room);
    std::vector<Record<T>> rejectScratch(room);
    std::size_t            consumed = 0UZ;

    for (std::size_t base = 0UZ; base < input.size();) {
        const std::size_t     count = std::min(stride, input.size() - base);
        InputSpan<T>          inSpan(input.subspan(base, count), base);
        OutputSpan<Record<T>> outSpan(std::span<Record<T>>(outScratch), 0UZ, nullptr, true);
        OutputSpan<Record<T>> rejectSpan(std::span<Record<T>>(rejectScratch), 0UZ, nullptr, true);

        std::ignore = block.processBulk(inSpan, outSpan, rejectSpan);
        consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && rejectSpan.count == 0UZ) {
            break;
        }
        base += inSpan.consumed;
    }
    return consumed;
}

} // namespace

const boost::ut::suite<"delimiter extractor"> delimiterExtractorTests = [] {
    using namespace boost::ut;

    "the records do not change with the chunk size"_test = [] {
        // anchor A extended to two frames: 0x3F with one stuffed bit, then 0x00 with none
        const std::vector<std::uint8_t> stream = itemsOf(std::string("01111110") + "111110100" + "01111110" + "00000000" + "01111110");

        for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 4096UZ}) {
            DelimiterExtractor<std::uint8_t> block     = make<DelimiterExtractor<std::uint8_t>>(hdlcFraming(1024U));
            const Extracted<std::uint8_t>    extracted = feed<std::uint8_t>(block, std::span<const std::uint8_t>(stream), chunk);
            expect(eq(extracted.out.size(), 2UZ)) << std::format("chunk {}", chunk);
            expect(eq(extracted.reject.size(), 0UZ)) << std::format("chunk {}", chunk);
            if (extracted.out.size() != 2UZ) {
                continue;
            }
            expect(that % (extracted.out[0UZ].signal_values == std::vector<std::uint8_t>{1U, 1U, 1U, 1U, 1U, 1U, 0U, 0U})) << std::format("chunk {}", chunk);
            expect(that % (extracted.out[1UZ].signal_values == std::vector<std::uint8_t>(8UZ, 0U))) << std::format("chunk {}", chunk);
            expect(eq(metaU64(extracted.out[0UZ], "sample_start"), std::uint64_t{8})) << std::format("chunk {}", chunk);
            expect(eq(metaU64(extracted.out[1UZ], "sample_start"), std::uint64_t{25})) << std::format("chunk {}", chunk);
            expect(eq(metaSize(extracted.out[0UZ], "stuffing_removed"), gr::Size_t{1})) << std::format("chunk {}", chunk);
            expect(eq(metaSize(extracted.out[1UZ], "stuffing_removed"), gr::Size_t{0})) << std::format("chunk {}", chunk);
            expect(eq(metaU64(extracted.out[0UZ], "sequence"), std::uint64_t{0})) << std::format("chunk {}", chunk);
            expect(eq(metaU64(extracted.out[1UZ], "sequence"), std::uint64_t{1})) << std::format("chunk {}", chunk);
            expect(eq(metaString(extracted.out[0UZ], "trigger_name"), std::string("delimiter"))) << std::format("chunk {}", chunk);
            expect(that % (extracted.out[0UZ].signal_names == std::vector<std::string>{"payload"})) << std::format("chunk {}", chunk);
            expect(eq(static_cast<std::size_t>(extracted.out[0UZ].extents.at(0)), 8UZ)) << std::format("chunk {}", chunk);
        }
    };

    "interior tags become the same timing events at every chunk size"_test = [] {
        // anchor A: raw payload items 8 to 16, of which item 13 is the stuffed bit
        const std::vector<std::uint8_t> stream = itemsOf(std::string("01111110") + "111110100" + "01111110");
        std::vector<gr::Tag>            tags;
        tags.emplace_back(0UZ, gr::property_map{{"marker", gr::Size_t{99}}}); // outside any frame
        for (std::size_t at = 8UZ; at <= 16UZ; ++at) {
            tags.emplace_back(at, gr::property_map{{"marker", static_cast<gr::Size_t>(at)}});
        }

        for (const std::size_t chunk : {1UZ, 5UZ, 4096UZ}) {
            DelimiterExtractor<std::uint8_t> block     = make<DelimiterExtractor<std::uint8_t>>(hdlcFraming(1024U));
            const Extracted<std::uint8_t>    extracted = feed<std::uint8_t>(block, std::span<const std::uint8_t>(stream), chunk, std::span<const gr::Tag>(tags));
            expect(eq(extracted.out.size(), 1UZ)) << std::format("chunk {}", chunk);
            if (extracted.out.size() != 1UZ) {
                continue;
            }
            const auto& events = extracted.out[0UZ].timing_events.at(0);
            expect(eq(events.size(), 9UZ)) << std::format("chunk {}: one event per payload item, and no tag twice", chunk);
            const std::vector<std::ptrdiff_t> wanted{0, 1, 2, 3, 4, 5, 5, 6, 7};
            for (std::size_t i = 0UZ; i < std::min(events.size(), wanted.size()); ++i) {
                expect(eq(events[i].first, wanted[i])) << std::format("chunk {}: event {}", chunk, i);
            }
            expect(eq(block.nTagsOnRemovedItems, 1ULL)) << std::format("chunk {}: the tag on the stuffed bit lands on the next surviving item", chunk);
            expect(eq(block.nTagsOutsideFrames, 1ULL)) << std::format("chunk {}: the tag before the first flag is outside every frame", chunk);
        }
    };

    "without its two required settings the block is inert if driven"_test = [] {
        const auto inert = [](gr::property_map settings) {
            DelimiterExtractor<std::uint8_t>  block = make<DelimiterExtractor<std::uint8_t>>(std::move(settings));
            const std::vector<std::uint8_t>   stream{0xC0U, 0x11U, 0x22U, 0xC0U};
            std::vector<Record<std::uint8_t>> outScratch(4UZ);
            std::vector<Record<std::uint8_t>> rejectScratch(4UZ);

            InputSpan<std::uint8_t>          inSpan(std::span<const std::uint8_t>(stream), 0UZ);
            OutputSpan<Record<std::uint8_t>> outSpan(std::span<Record<std::uint8_t>>(outScratch), 0UZ, nullptr, true);
            OutputSpan<Record<std::uint8_t>> rejectSpan(std::span<Record<std::uint8_t>>(rejectScratch), 0UZ, nullptr, true);
            const gr::work::Status           status = block.processBulk(inSpan, outSpan, rejectSpan);
            boost::ut::expect(status == gr::work::Status::ERROR);
            boost::ut::expect(boost::ut::eq(inSpan.consumed, 0UZ));
            boost::ut::expect(boost::ut::eq(outSpan.count, 0UZ));
        };

        inert({{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}});
        inert({{"max_payload_items", gr::Size_t{16}}});
    };

    "delimiters at every boundary, in all three transparency modes"_test = [] {
        struct Mode {
            const char* transparency;
            gr::Size_t  bitsPerItem;
            const char* shortest;
            const char* longest;
        };
        // the 64-bit bit-stuffing delimiter carries one run of six ones and no zero arriving at exactly five
        static const std::string kLongStuffed = std::string("01111110") + [] {
            std::string tail;
            for (std::size_t i = 0UZ; i < 28UZ; ++i) {
                tail += "01";
            }
            return tail;
        }();
        const Mode kModes[]{{"none", 1U, "1", "01111110"}, {"bit_stuffing", 1U, "01111110", kLongStuffed.c_str()}, {"none", 8U, "11000000", "1100000011000000"}};

        Rng rng{};
        for (const Mode& mode : kModes) {
            for (const char* delimiter : {mode.shortest, mode.longest}) {
                const std::string         text(delimiter);
                const std::size_t         items = text.size() / static_cast<std::size_t>(mode.bitsPerItem);
                std::vector<std::uint8_t> pattern;
                if (mode.bitsPerItem == 1U) {
                    pattern = itemsOf(text);
                } else {
                    for (std::size_t at = 0UZ; at < text.size(); at += 8UZ) {
                        unsigned value = 0U;
                        for (unsigned bit = 0U; bit < 8U; ++bit) {
                            value = (value << 1U) | (text[at + bit] == '1' ? 1U : 0U);
                        }
                        pattern.push_back(static_cast<std::uint8_t>(value));
                    }
                }

                // a delimiter at the very first item, at the very last, and at random positions in between
                std::vector<std::uint8_t> stream;
                stream.reserve(100'000UZ + items);
                stream.insert(stream.end(), pattern.begin(), pattern.end());
                while (stream.size() < 100'000UZ - items) {
                    if (rng.below(64UZ) == 0UZ) {
                        stream.insert(stream.end(), pattern.begin(), pattern.end());
                    } else {
                        stream.push_back(static_cast<std::uint8_t>(rng.next() & (mode.bitsPerItem == 1U ? 1ULL : 0x7FULL)));
                    }
                }
                stream.insert(stream.end(), pattern.begin(), pattern.end());

                for (const std::size_t chunk : {1UZ, 2UZ, 3UZ, 17UZ, 4096UZ}) {
                    const gr::property_map           settings{{"end_delimiter", text}, {"bits_per_item", mode.bitsPerItem}, {"transparency", std::string(mode.transparency)}, {"max_payload_items", gr::Size_t{512}}};
                    DelimiterExtractor<std::uint8_t> block    = make<DelimiterExtractor<std::uint8_t>>(settings);
                    const std::size_t                consumed = feedCounting<std::uint8_t>(block, std::span<const std::uint8_t>(stream), chunk, 512UZ);
                    expect(eq(consumed, stream.size())) << std::format("{} at {} items, chunk {}", mode.transparency, items, chunk);
                }
            }
        }
    };
};

int main() { /* not needed for UT */ }
