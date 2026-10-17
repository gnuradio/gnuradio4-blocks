#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <memory>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/ofdm/CarrierAllocator.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_ofdm_allocator {

using gr::blocks::ofdm::CarrierAllocator;
using CF       = std::complex<float>;
namespace shim = gr::blocks::testing::span;

constexpr gr::Size_t kFft = 64U;

/// The test numerology: 64 carriers, 52 of them occupied, 4 of those pilots.
[[nodiscard]] std::vector<std::int32_t> pilotCarriers() { return {-21, -7, 7, 21}; }

[[nodiscard]] std::vector<std::int32_t> dataCarriers() {
    std::vector<std::int32_t> carriers;
    const auto                pilots = pilotCarriers();
    for (std::int32_t c = -26; c <= 26; ++c) {
        if (c == 0 || std::ranges::find(pilots, c) != pilots.end()) {
            continue;
        }
        carriers.push_back(c);
    }
    return carriers;
}

[[nodiscard]] std::vector<float> interleave(std::span<const CF> values) {
    std::vector<float> flat(2UZ * values.size());
    for (std::size_t k = 0UZ; k < values.size(); ++k) {
        flat[2UZ * k]       = values[k].real();
        flat[2UZ * k + 1UZ] = values[k].imag();
    }
    return flat;
}

/// A stream of distinct samples. A misplaced carrier shows which sample it holds.
[[nodiscard]] std::vector<CF> ramp(std::size_t count) {
    std::vector<CF> data(count);
    for (std::size_t k = 0UZ; k < count; ++k) {
        data[k] = CF(static_cast<float>(k + 1UZ), -static_cast<float>(k + 1UZ));
    }
    return data;
}

/// @brief Drives the block through span shims and keeps the stream index across calls.
struct Driver {
    CarrierAllocator&            block;
    std::size_t                  outRoom = 128UZ;
    std::size_t                  at      = 0UZ; ///< stream index of the next sample
    std::vector<gr::DataSet<CF>> records{};

    /// Feeds @p input in chunks. Each call starts at the first sample the previous call did not consume. Returns the
    /// samples the block consumed.
    std::size_t feed(std::span<const CF> input, std::size_t chunk) {
        std::vector<gr::DataSet<CF>> scratch(outRoom);
        std::size_t                  consumed = 0UZ;
        std::size_t                  fed      = 0UZ;
        while (consumed < input.size()) {
            fed = std::min(input.size(), fed + chunk);
            shim::InputSpan<CF>               inSpan(input.subspan(consumed, fed - consumed), at + consumed);
            shim::OutputSpan<gr::DataSet<CF>> outSpan(std::span<gr::DataSet<CF>>(scratch.data(), outRoom));
            std::ignore = block.processBulk(inSpan, outSpan);
            collect(scratch, outSpan.count);
            consumed += inSpan.consumed;
            if (inSpan.consumed == 0UZ && fed == input.size()) {
                break; // the block takes nothing more while its output is full
            }
        }
        at += consumed;
        return consumed;
    }

    /// Runs the end-of-stream epilogue over @p tail, with room for @p room records.
    void end(std::span<const CF> tail, std::size_t room) {
        std::vector<gr::DataSet<CF>>      scratch(room);
        shim::InputSpan<CF>               inSpan(tail, at);
        shim::OutputSpan<gr::DataSet<CF>> outSpan(std::span<gr::DataSet<CF>>(scratch.data(), room));
        std::ignore = block.processEpilogue(inSpan, outSpan);
        collect(scratch, outSpan.count);
    }

    void collect(std::vector<gr::DataSet<CF>>& scratch, std::size_t count) {
        for (std::size_t k = 0UZ; k < count; ++k) {
            records.push_back(std::move(scratch[k]));
        }
    }
};

/// @brief Drives the block over @p input in chunks, then runs the end-of-stream epilogue over the samples it left.
[[nodiscard]] std::vector<gr::DataSet<CF>> drive(CarrierAllocator& block, std::span<const CF> input, std::size_t chunk, std::size_t outRoom = 128UZ) {
    Driver            driver{block, outRoom};
    const std::size_t consumed = driver.feed(input, chunk);
    driver.end(input.subspan(consumed), outRoom);
    return std::move(driver.records);
}

[[nodiscard]] CarrierAllocator make(gr::property_map settings) {
    CarrierAllocator block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
    return block;
}

/// @brief Applies @p settings between two calls.
void change(CarrierAllocator& block, gr::property_map settings) {
    std::ignore = block.settings().setStaged(std::move(settings));
    std::ignore = block.settings().applyStagedParameters();
}

/// @brief The refusal's message, or an empty string.
template<typename TFunction>
[[nodiscard]] std::string refusal(TFunction&& function) {
    try {
        function();
    } catch (const gr::exception& e) {
        return std::string(e.what());
    }
    return {};
}

[[nodiscard]] std::string signalName(const gr::DataSet<CF>& record) { return record.signal_names.empty() ? std::string{} : record.signal_names[0UZ]; }

/// @brief Runs a source of @p nInput samples into the block and a record counter under the framework's scheduler. A
/// @p recordEdge of 0 keeps the default edge. Returns the records the counter received and the block's `nTruncated()`.
[[nodiscard]] std::pair<std::size_t, std::uint64_t> runToCounter(std::size_t nInput, gr::property_map settings, std::size_t recordEdge) {
    using gr::blocks::testing::ProcessFunction;
    gr::Graph graph;
    auto&     source   = graph.emplaceBlock<gr::blocks::testing::TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(nInput)}, {"mark_tag", false}});
    auto&     allocate = graph.emplaceBlock<CarrierAllocator>(std::move(settings));
    auto&     counter  = graph.emplaceBlock<gr::blocks::testing::CountingSink<gr::DataSet<CF>>>({});
    boost::ut::expect(graph.connect<"out", "in">(source, allocate).has_value());
    boost::ut::expect((recordEdge > 0UZ ? graph.connect<"out", "in">(allocate, counter, gr::EdgeParameters{.minBufferSize = recordEdge}) : graph.connect<"out", "in">(allocate, counter)).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value());
    return {static_cast<std::size_t>(counter.count.value), allocate.nTruncated()};
}

[[nodiscard]] std::uint64_t meta(const gr::DataSet<CF>& record, std::string_view key) {
    const auto entry = record.meta_information[0UZ].find(std::pmr::string(key));
    if (entry == record.meta_information[0UZ].end()) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    const auto* value = entry->second.get_if<std::uint64_t>();
    return value == nullptr ? std::numeric_limits<std::uint64_t>::max() : *value;
}

[[nodiscard]] std::string kindOf(const gr::DataSet<CF>& record) {
    const auto  entry = record.meta_information[0UZ].find(std::pmr::string("symbol_kind"));
    const auto* value = entry->second.get_if<std::pmr::string>();
    return value == nullptr ? std::string{} : std::string(value->begin(), value->end());
}

const boost::ut::suite<"OFDM carrier allocator"> _allocator = [] {
    using namespace boost::ut;

    "a symbol holds data, pilots and guards in the bins of their signed carriers"_test = [] {
        const auto            data   = dataCarriers();
        const auto            pilots = pilotCarriers();
        const std::vector<CF> pilotValues{CF(1.f, 0.f), CF(0.f, 1.f), CF(-1.f, 0.f), CF(0.f, -1.f)};

        CarrierAllocator block = make({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilots}, {"pilot_symbols", interleave(pilotValues)}});

        const std::vector<CF> input   = ramp(data.size());
        const auto            records = drive(block, std::span<const CF>(input), 7UZ);

        expect(eq(records.size(), 1UZ)) << "48 data carriers in, one symbol out";
        const auto& values = records[0UZ].signal_values;
        expect(eq(values.size(), static_cast<std::size_t>(kFft)));

        for (std::size_t k = 0UZ; k < data.size(); ++k) {
            const std::size_t bin = gr::ofdm::CarrierMap::binOf(kFft, data[k]).value();
            expect(values[bin] == input[k]) << std::format("data carrier {} landed in bin {}", data[k], bin);
        }
        for (std::size_t p = 0UZ; p < pilots.size(); ++p) {
            const std::size_t bin = gr::ofdm::CarrierMap::binOf(kFft, pilots[p]).value();
            expect(values[bin] == pilotValues[p]) << std::format("pilot carrier {} landed in bin {}", pilots[p], bin);
        }

        expect(values[0UZ] == CF{}) << "DC is a guard in this numerology and holds nothing";
        expect(values[32UZ] == CF{}) << "bin 32 is carrier -32, the Nyquist carrier, and is guarded here";
        expect(values[27UZ] == CF{}) << "carrier +27 is outside the occupied band";
        expect(values[37UZ] == CF{}) << "carrier -27 is outside the occupied band";

        // the record's axis states the carrier of each bin
        expect(records[0UZ].axis_values[0UZ][0UZ] == CF(0.f, 0.f));
        expect(records[0UZ].axis_values[0UZ][1UZ] == CF(1.f, 0.f));
        expect(records[0UZ].axis_values[0UZ][32UZ] == CF(-32.f, 0.f));
        expect(records[0UZ].axis_values[0UZ][63UZ] == CF(-1.f, 0.f));
    };

    "+fft_len/2 is refused, not folded onto the Nyquist bin"_test = [] {
        std::vector<std::int32_t> data = dataCarriers();
        data.push_back(32);
        expect(throws([&data] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", data}}); }));
    };

    "the pilot cycle runs at two levels"_test = [] {
        const auto      data   = dataCarriers();
        const auto      pilots = pilotCarriers();
        std::vector<CF> cycle(6UZ); // a length that is not a multiple of n_pilots rotates the pilots
        for (std::size_t k = 0UZ; k < cycle.size(); ++k) {
            cycle[k] = CF(static_cast<float>(k + 1UZ), 0.f);
        }

        CarrierAllocator block = make({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilots}, {"pilot_symbols", interleave(cycle)}});

        const std::vector<CF> input   = ramp(3UZ * data.size());
        const auto            records = drive(block, std::span<const CF>(input), 100UZ);
        expect(eq(records.size(), 3UZ));

        for (std::size_t s = 0UZ; s < 3UZ; ++s) {
            for (std::size_t p = 0UZ; p < pilots.size(); ++p) {
                const std::size_t bin      = gr::ofdm::CarrierMap::binOf(kFft, pilots[p]).value();
                const std::size_t expected = (s * pilots.size() + p) % cycle.size();
                expect(records[s].signal_values[bin] == cycle[expected]) << std::format("symbol {} pilot slot {}", s, p);
            }
        }
    };

    "sync words open a frame verbatim and the cadence repeats"_test = [] {
        const auto      data = dataCarriers();
        std::vector<CF> word(kFft);
        for (std::size_t k = 0UZ; k < word.size(); ++k) {
            word[k] = CF(static_cast<float>(k), 0.5f);
        }

        CarrierAllocator block = make({{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{2U}}});

        const std::vector<CF> input   = ramp(4UZ * data.size());
        const auto            records = drive(block, std::span<const CF>(input), 13UZ);

        expect(eq(records.size(), 6UZ)) << "two frames of one sync word and two data symbols";
        for (const std::size_t at : {0UZ, 3UZ}) {
            expect(eq(kindOf(records[at]), std::string("sync")));
            expect(std::ranges::equal(records[at].signal_values, word)) << "the sync word is emitted verbatim";
            expect(eq(meta(records[at], "symbol_in_frame"), std::uint64_t{0}));
        }
        expect(eq(meta(records[1UZ], "symbol_in_frame"), std::uint64_t{1}));
        expect(eq(meta(records[2UZ], "symbol_in_frame"), std::uint64_t{2}));
        expect(eq(meta(records[3UZ], "frame_index"), std::uint64_t{1}));
        expect(eq(meta(records[5UZ], "symbol_index"), std::uint64_t{5}));
    };

    "the same stream chunked differently gives the same records"_test = [] {
        const auto            data   = dataCarriers();
        const auto            pilots = pilotCarriers();
        const std::vector<CF> pilotValues{CF(1.f, 0.f), CF(0.f, 1.f), CF(-1.f, 0.f)};
        std::vector<CF>       word(kFft, CF(0.25f, -0.25f));

        const gr::property_map settings{{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilots}, {"pilot_symbols", interleave(pilotValues)}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{3U}}};

        const std::vector<CF>        input = ramp(7UZ * data.size() + 5UZ); // ends mid-symbol and mid-frame
        std::vector<gr::DataSet<CF>> reference;
        for (const std::size_t chunk : {1UZ, 7UZ, 48UZ, 4096UZ}) {
            CarrierAllocator block   = make(settings);
            auto             records = drive(block, std::span<const CF>(input), chunk);
            if (reference.empty()) {
                reference = std::move(records);
                expect(gt(reference.size(), 0UZ));
                continue;
            }
            expect(eq(records.size(), reference.size())) << std::format("chunk {}", chunk);
            for (std::size_t r = 0UZ; r < records.size(); ++r) {
                expect(std::ranges::equal(records[r].signal_values, reference[r].signal_values)) << std::format("chunk {} record {}", chunk, r);
                expect(eq(meta(records[r], "symbol_index"), meta(reference[r], "symbol_index")));
                expect(eq(meta(records[r], "pad_carriers"), meta(reference[r], "pad_carriers")));
            }
        }
    };

    "end of stream pads the open symbol and the open frame, counted and stated"_test = [] {
        const auto      data = dataCarriers();
        std::vector<CF> word(kFft, CF(1.f, 0.f));

        CarrierAllocator block = make({{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{3U}}});

        const std::size_t     partial = 5UZ;
        const std::vector<CF> input   = ramp(data.size() + partial); // one whole symbol and a fragment
        const auto            records = drive(block, std::span<const CF>(input), 9UZ);

        expect(eq(records.size(), 4UZ)) << "the sync word plus the frame's three data symbols";
        expect(eq(meta(records[1UZ], "pad_carriers"), std::uint64_t{0}));
        expect(eq(meta(records[2UZ], "pad_carriers"), data.size() - partial));
        expect(eq(meta(records[3UZ], "pad_carriers"), data.size()));
        expect(eq(block.nPadded(), 2UZ * data.size() - partial));

        // the padded carriers are zero, and the carriers that arrived are untouched
        const std::size_t bin = gr::ofdm::CarrierMap::binOf(kFft, data[partial]).value();
        expect(records[2UZ].signal_values[bin] == CF{});
        const std::size_t kept = gr::ofdm::CarrierMap::binOf(kFft, data[partial - 1UZ]).value();
        expect(records[2UZ].signal_values[kept] == input[data.size() + partial - 1UZ]);
    };

    "an unframed stream pads only the open symbol"_test = [] {
        const auto            data    = dataCarriers();
        CarrierAllocator      block   = make({{"fft_len", kFft}, {"data_carriers", data}});
        const std::vector<CF> input   = ramp(data.size() + 2UZ);
        const auto            records = drive(block, std::span<const CF>(input), 5UZ);

        expect(eq(records.size(), 2UZ));
        expect(eq(meta(records[1UZ], "pad_carriers"), data.size() - 2UZ));
        expect(eq(block.nPadded(), data.size() - 2UZ));
    };

    "a numerology or a frame the block cannot serve is refused by name"_test = [] {
        const auto data  = dataCarriers();
        const auto names = [](std::string_view setting, auto&& function) {
            const std::string message = refusal(function);
            expect(message.contains(setting)) << std::format("the refusal names {}: '{}'", setting, message);
        };

        const std::string limits = refusal([&data] { std::ignore = make({{"fft_len", gr::Size_t{63U}}, {"data_carriers", data}}); });
        expect(limits.contains("fft_len") && limits.contains("outside the limits")) << std::format("the limits of fft_len refuse 63: '{}'", limits);
        names("data_carriers", [&data] {
            CarrierAllocator block = make({{"fft_len", kFft}, {"data_carriers", data}});
            change(block, {{"data_carriers", std::vector<std::int32_t>{}}});
        });
        names("pilot_carriers", [&data] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", std::vector<std::int32_t>{1}}}); });
        names("pilot_symbols", [&data] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilotCarriers()}}); });
        names("pilot_symbols", [&data] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilotCarriers()}, {"pilot_symbols", std::vector<float>{1.f, 0.f, 0.f, 0.f}}}); });
        names("sync_words", [&data] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", std::vector<float>(2UZ * kFft - 4UZ, 0.f)}, {"frame_len", gr::Size_t{2U}}}); });
        names("sync_words", [&data] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", std::vector<float>(2UZ * kFft, 0.f)}}); });
    };

    "processBulk takes the whole span, and the epilogue alone pads"_test = [] {
        const auto            data  = dataCarriers();
        CarrierAllocator      block = make({{"fft_len", kFft}, {"data_carriers", data}});
        const std::vector<CF> input = ramp(data.size() + 2UZ);

        Driver driver{block};
        expect(eq(driver.feed(std::span<const CF>(input), 4096UZ), input.size())) << "no sample is held back";
        expect(eq(driver.records.size(), 1UZ));
        driver.end({}, 128UZ);
        expect(eq(driver.records.size(), 2UZ)) << "an epilogue over an empty span emits the open symbol";
        expect(eq(meta(driver.records[1UZ], "pad_carriers"), data.size() - 2UZ));
    };

    "an end-of-stream output with too little room ends the frame short and counts what it lacks"_test = [] {
        const auto      data = dataCarriers();
        std::vector<CF> word(kFft, CF(1.f, 0.f));

        CarrierAllocator      block = make({{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{3U}}});
        const std::vector<CF> input = ramp(data.size() + 5UZ);

        Driver driver{block};
        std::ignore = driver.feed(std::span<const CF>(input), 4096UZ);
        expect(eq(driver.records.size(), 2UZ)) << "the sync word and the first data symbol";
        driver.end({}, 1UZ);
        expect(eq(driver.records.size(), 3UZ)) << "room for the open symbol alone";
        expect(eq(block.nTruncated(), std::uint64_t{1})) << "the frame lacks its third data symbol";
        expect(eq(block.nPadded(), data.size() - 5UZ)) << "only emitted records count as padded";
    };

    /**
     * One sync word and the largest `frame_len` make a frame of 2^32 records, one past the largest 32-bit value. On
     * every target the stream's five samples open the frame. The end of the stream emits three records into a room of
     * three and counts the rest as lacking.
     */
    "a frame of 2^32 records is completed and counted on every target"_test = [] {
        const std::vector<CF>   word(kFft, CF(1.f, 0.f));
        constexpr std::uint64_t kRecords = std::uint64_t{1} + std::numeric_limits<gr::Size_t>::max();
        constexpr std::size_t   kRoom    = 3UZ;
        CarrierAllocator        block    = make({{"fft_len", kFft}, {"data_carriers", dataCarriers()}, {"sync_words", interleave(word)}, {"frame_len", std::numeric_limits<gr::Size_t>::max()}});
        const std::vector<CF>   input    = ramp(5UZ);
        Driver                  driver{block};
        std::ignore = driver.feed(std::span<const CF>(input), 4096UZ);
        expect(eq(driver.records.size(), 0UZ)) << "no whole data symbol, and no record yet";
        driver.end({}, kRoom);
        expect(eq(driver.records.size(), kRoom)) << "the sync word and two data symbols fill the room";
        expect(eq(driver.records.size() + block.nTruncated(), kRecords)) << "the records out and the records lacking make the frame";
    };

    "the end of stream through the framework completes the frame into the room the output has, and counts what it lacks"_test = [] {
        // One sync word and 200 data symbols make a frame of 201 records. The stream carries one symbol and 5 samples.
        const auto              data = dataCarriers();
        const std::vector<CF>   word(kFft, CF(1.f, 0.f));
        const gr::property_map  settings{{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{200U}}};
        constexpr std::uint64_t kFrame = 201ULL;

        const auto [records, truncated] = runToCounter(data.size() + 5UZ, settings, 0UZ);
        std::println("default record edge: {} records out, {} counted as lacking", records, truncated);
        expect(gt(truncated, 0ULL)) << "the default record edge holds fewer records than the frame";
        expect(eq(static_cast<std::uint64_t>(records) + truncated, kFrame)) << "the records out and the records lacking make the frame";

        const auto [wideRecords, wideTruncated] = runToCounter(data.size() + 5UZ, settings, 1024UZ);
        expect(eq(static_cast<std::uint64_t>(wideRecords), kFrame)) << "a record edge of 1024 takes the whole frame";
        expect(eq(wideTruncated, 0ULL));
    };

    "a numerology the block already runs on keeps the pending samples, the open frame and the indices"_test = [] {
        const auto             data   = dataCarriers();
        const auto             pilots = pilotCarriers();
        const std::vector<CF>  pilotValues{CF(1.f, 0.f), CF(0.f, 1.f), CF(-1.f, 0.f)};
        const std::vector<CF>  word(kFft, CF(0.25f, -0.25f));
        const gr::property_map settings{{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilots}, {"pilot_symbols", interleave(pilotValues)}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{3U}}};
        const std::vector<CF>  input = ramp(5UZ * data.size());

        CarrierAllocator reference = make(settings);
        const auto       expected  = drive(reference, std::span<const CF>(input), 4096UZ);

        CarrierAllocator  block = make(settings);
        Driver            driver{block};
        const std::size_t half = data.size() + data.size() / 2UZ;
        expect(eq(driver.feed(std::span<const CF>(input).first(half), 4096UZ), half)) << "processBulk takes the whole span";
        // The refusal leaves the empty list in the member. The second change then reaches the block with the carriers
        // it already runs on.
        expect(refusal([&block] { change(block, {{"data_carriers", std::vector<std::int32_t>{}}}); }).contains("data_carriers"));
        change(block, {{"data_carriers", data}});
        expect(eq(driver.feed(std::span<const CF>(input).subspan(half), 4096UZ), input.size() - half));
        driver.end({}, 128UZ);

        expect(eq(driver.records.size(), expected.size()));
        for (std::size_t r = 0UZ; r < std::min(driver.records.size(), expected.size()); ++r) {
            expect(std::ranges::equal(driver.records[r].signal_values, expected[r].signal_values)) << std::format("record {}", r);
            for (const std::string_view key : {"symbol_index", "frame_index", "symbol_in_frame", "sample_start", "pad_carriers"}) {
                expect(eq(meta(driver.records[r], key), meta(expected[r], key))) << std::format("record {} {}", r, key);
            }
        }
    };

    "pilot_symbols and signal_name changes apply from the next record and keep the frame"_test = [] {
        const auto            data   = dataCarriers();
        const auto            pilots = pilotCarriers();
        const std::vector<CF> first{CF(1.f, 0.f), CF(0.f, 1.f), CF(-1.f, 0.f), CF(0.f, -1.f)};
        const std::vector<CF> second{CF(2.f, 0.f), CF(0.f, 2.f), CF(-2.f, 0.f), CF(0.f, -2.f)};
        const std::vector<CF> word(kFft, CF(0.5f, 0.5f));
        const std::vector<CF> input = ramp(3UZ * data.size());

        CarrierAllocator  block = make({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilots}, {"pilot_symbols", interleave(first)}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{3U}}});
        Driver            driver{block};
        const std::size_t half = data.size() + data.size() / 2UZ;
        std::ignore            = driver.feed(std::span<const CF>(input).first(half), 4096UZ);
        expect(eq(driver.records.size(), 2UZ)) << "the sync word and the first data symbol";
        change(block, {{"pilot_symbols", interleave(second)}, {"signal_name", std::string("renamed")}});
        std::ignore = driver.feed(std::span<const CF>(input).subspan(half), 4096UZ);

        expect(eq(driver.records.size(), 4UZ)) << "one frame of a sync word and three data symbols";
        expect(eq(block.nDropped(), std::uint64_t{0}));
        for (std::size_t r = 0UZ; r < std::min(driver.records.size(), 4UZ); ++r) {
            const auto& record = driver.records[r];
            expect(eq(meta(record, "symbol_index"), std::uint64_t{r}));
            expect(eq(meta(record, "frame_index"), std::uint64_t{0}));
            expect(eq(meta(record, "symbol_in_frame"), std::uint64_t{r}));
            expect(eq(signalName(record), std::string(r < 2UZ ? "ofdm_symbol" : "renamed"))) << std::format("record {}", r);
            if (r == 0UZ) {
                continue;
            }
            const std::vector<CF>& cycle = r < 2UZ ? first : second;
            for (std::size_t p = 0UZ; p < pilots.size(); ++p) {
                const std::size_t bin = gr::ofdm::CarrierMap::binOf(kFft, pilots[p]).value();
                expect(record.signal_values[bin] == cycle[p]) << std::format("record {} pilot slot {}", r, p);
            }
            for (std::size_t k = 0UZ; k < data.size(); ++k) {
                const std::size_t bin = gr::ofdm::CarrierMap::binOf(kFft, data[k]).value();
                expect(record.signal_values[bin] == input[(r - 1UZ) * data.size() + k]) << std::format("record {} data carrier {}", r, data[k]);
            }
        }
    };

    "a new numerology ends the open frame, keeps the samples that fit and counts the samples dropped"_test = [] {
        const auto            data = dataCarriers();
        const std::vector<CF> word(kFft, CF(0.5f, 0.5f));
        const std::size_t     half  = data.size() + data.size() / 2UZ; // one symbol and 24 samples
        const std::vector<CF> input = ramp(half + 32UZ);

        for (const std::size_t nNew : {16UZ, 32UZ}) {
            std::vector<std::int32_t> carriers; // +1, -1, +2, -2, ...
            for (std::size_t k = 0UZ; k < nNew; ++k) {
                const auto magnitude = static_cast<std::int32_t>(k / 2UZ + 1UZ);
                carriers.push_back(k % 2UZ == 0UZ ? magnitude : -magnitude);
            }
            CarrierAllocator block = make({{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{3U}}});
            Driver           driver{block};
            std::ignore = driver.feed(std::span<const CF>(input).first(half), 4096UZ);
            expect(eq(driver.records.size(), 2UZ));
            change(block, {{"data_carriers", carriers}});
            std::ignore = driver.feed(std::span<const CF>(input).subspan(half), 4096UZ);

            const bool        dropped = nNew < 24UZ;
            const std::size_t first   = dropped ? half : data.size(); // the input index of the new frame's first data sample
            expect(eq(block.nDropped(), dropped ? std::uint64_t{24} : std::uint64_t{0})) << std::format("{} new data carriers", nNew);
            expect(ge(driver.records.size(), 4UZ)) << "a new frame of a sync word and a data symbol";
            if (driver.records.size() < 4UZ) {
                continue;
            }
            const auto& sync = driver.records[2UZ];
            const auto& next = driver.records[3UZ];
            expect(eq(kindOf(sync), std::string("sync")));
            expect(eq(meta(sync, "frame_index"), std::uint64_t{1})) << "the open frame ended";
            expect(eq(meta(sync, "symbol_in_frame"), std::uint64_t{0}));
            expect(eq(meta(sync, "symbol_index"), std::uint64_t{2})) << "the indices continue";
            expect(eq(meta(next, "symbol_index"), std::uint64_t{3}));
            expect(eq(meta(next, "sample_start"), std::uint64_t{first}));
            const std::size_t bin = gr::ofdm::CarrierMap::binOf(kFft, carriers[0UZ]).value();
            expect(next.signal_values[bin] == input[first]) << std::format("{} new data carriers", nNew);
        }
    };

    "a refused setting leaves the block on the settings it last accepted"_test = [] {
        const auto            data = dataCarriers();
        const std::vector<CF> word(kFft, CF(0.5f, 0.5f));
        const std::vector<CF> input = ramp(4UZ * data.size());

        CarrierAllocator block = make({{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{2U}}});
        expect(refusal([&block] { change(block, {{"frame_len", gr::Size_t{0U}}}); }).contains("sync_words")) << "an unframed stream with sync words";
        const auto records = drive(block, std::span<const CF>(input), 4096UZ);
        expect(eq(records.size(), 6UZ)) << "two frames of a sync word and two data symbols";
        for (const std::size_t at : {0UZ, 3UZ}) {
            if (at < records.size()) {
                expect(eq(kindOf(records[at]), std::string("sync")));
            }
        }
    };

    /**
     * Each arm sends a refused pair of settings, then a valid change of another key. The records equal those of a run
     * that took the valid change alone.
     */
    "a valid change after a refused application leaves the refused values out"_test = [] {
        const auto             data = dataCarriers();
        const std::vector<CF>  word(kFft, CF(0.5f, 0.5f));
        const std::vector<CF>  input = ramp(7UZ * data.size());
        const std::size_t      split = data.size() + data.size() / 2UZ; // a sync word, one data symbol and 24 samples
        const gr::property_map framed{{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{2U}}};
        struct Arm {
            std::string_view name;
            gr::property_map refused;
            gr::property_map valid;
        };
        const std::vector<Arm> arms{
            {"frame_len 0 with sync words, then an empty sync_words", {{"frame_len", gr::Size_t{0U}}, {"signal_name", std::string("refused")}}, {{"sync_words", std::vector<float>{}}}},
            {"an empty data_carriers, then frame_len 3", {{"data_carriers", std::vector<std::int32_t>{}}, {"signal_name", std::string("refused")}}, {{"frame_len", gr::Size_t{3U}}}},
        };
        for (const Arm& arm : arms) {
            const auto run = [&](bool refuseFirst) {
                CarrierAllocator block = make(framed);
                Driver           driver{block};
                std::ignore = driver.feed(std::span<const CF>(input).first(split), 4096UZ);
                if (refuseFirst) {
                    expect(!refusal([&block, &arm] { change(block, arm.refused); }).empty()) << std::format("{}: the first application is refused", arm.name);
                }
                expect(refusal([&block, &arm] { change(block, arm.valid); }).empty()) << std::format("{}: the valid change applies", arm.name);
                std::ignore = driver.feed(std::span<const CF>(input).subspan(split), 4096UZ);
                driver.end({}, 128UZ);
                return std::move(driver.records);
            };
            const auto expected = run(false);
            const auto got      = run(true);
            expect(eq(got.size(), expected.size())) << arm.name;
            for (std::size_t r = 0UZ; r < std::min(got.size(), expected.size()); ++r) {
                expect(std::ranges::equal(got[r].signal_values, expected[r].signal_values)) << std::format("{}: record {}", arm.name, r);
                expect(eq(signalName(got[r]), signalName(expected[r]))) << std::format("{}: record {} signal name", arm.name, r);
                for (const std::string_view key : {"symbol_index", "frame_index", "symbol_in_frame", "sample_start", "pad_carriers"}) {
                    expect(eq(meta(got[r], key), meta(expected[r], key))) << std::format("{}: record {} {}", arm.name, r, key);
                }
            }
        }
    };

    "a signal_name change alone reaches the next record"_test = [] {
        const auto            data  = dataCarriers();
        CarrierAllocator      block = make({{"fft_len", kFft}, {"data_carriers", data}});
        const std::vector<CF> input = ramp(3UZ * data.size());
        const std::size_t     split = data.size() + 10UZ;

        Driver driver{block};
        std::ignore = driver.feed(std::span<const CF>(input).first(split), 4096UZ);
        expect(eq(driver.records.size(), 1UZ));
        change(block, {{"signal_name", std::string("renamed")}});
        std::ignore = driver.feed(std::span<const CF>(input).subspan(split), 4096UZ);
        driver.end({}, 128UZ);

        expect(eq(driver.records.size(), 3UZ));
        for (std::size_t r = 0UZ; r < driver.records.size(); ++r) {
            expect(eq(signalName(driver.records[r]), std::string(r == 0UZ ? "ofdm_symbol" : "renamed"))) << std::format("record {}", r);
        }
    };

    "a frame_len change to 0 ends the open frame, and the pilot cycle counts from the next data symbol"_test = [] {
        const auto             data   = dataCarriers();
        const auto             pilots = pilotCarriers();
        const std::vector<CF>  cycle{CF(1.f, 0.f), CF(2.f, 0.f), CF(3.f, 0.f), CF(4.f, 0.f), CF(5.f, 0.f), CF(6.f, 0.f)}; // rotates over four pilots
        const std::vector<CF>  word(kFft, CF(0.5f, 0.5f));
        const std::vector<CF>  input = ramp(5UZ * data.size());
        const gr::property_map unframed{{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilots}, {"pilot_symbols", interleave(cycle)}};
        gr::property_map       framed = unframed;
        framed["sync_words"]          = interleave(word);
        framed["frame_len"]           = gr::Size_t{3U};

        // The data symbols after the change match a stream that starts unframed at the same sample.
        CarrierAllocator reference = make(unframed);
        const auto       expected  = drive(reference, std::span<const CF>(input).subspan(2UZ * data.size()), 4096UZ);
        expect(eq(expected.size(), 3UZ));

        CarrierAllocator  block = make(framed);
        Driver            driver{block};
        const std::size_t split = 2UZ * data.size() + data.size() / 2UZ; // two data symbols and 24 samples
        std::ignore             = driver.feed(std::span<const CF>(input).first(split), 4096UZ);
        expect(eq(driver.records.size(), 3UZ)) << "the sync word and two data symbols";
        change(block, {{"frame_len", gr::Size_t{0U}}, {"sync_words", std::vector<float>{}}});
        std::ignore = driver.feed(std::span<const CF>(input).subspan(split), 4096UZ);
        driver.end({}, 128UZ);

        expect(eq(driver.records.size(), 3UZ + expected.size()));
        const std::size_t after = driver.records.size() > 3UZ ? driver.records.size() - 3UZ : 0UZ;
        for (std::size_t r = 0UZ; r < std::min(after, expected.size()); ++r) {
            const auto& record = driver.records[3UZ + r];
            expect(std::ranges::equal(record.signal_values, expected[r].signal_values)) << std::format("data symbol {} after the change: data and pilots", r);
            expect(eq(meta(record, "symbol_in_frame"), meta(expected[r], "symbol_in_frame"))) << std::format("data symbol {} after the change", r);
            expect(eq(meta(record, "frame_index"), std::uint64_t{1})) << "the change ends frame 0";
        }
    };

    "a block left at its default settings starts, and its calls report the missing numerology"_test = [] {
        CarrierAllocator block(gr::property_map{});
        block.settings().init();
        std::ignore  = block.settings().applyStagedParameters();
        bool started = true;
        try {
            block.start();
        } catch (const gr::exception&) {
            started = false;
        }
        expect(started) << "start() throws nothing";
        if (!started) {
            return;
        }

        const std::vector<CF>             input = ramp(100UZ);
        std::vector<gr::DataSet<CF>>      scratch(4UZ);
        shim::InputSpan<CF>               inSpan(std::span<const CF>(input), 0UZ);
        shim::OutputSpan<gr::DataSet<CF>> outSpan{std::span<gr::DataSet<CF>>(scratch)};
        expect(block.processBulk(inSpan, outSpan) == gr::work::Status::ERROR);
        expect(eq(inSpan.consumed, 0UZ));
        expect(eq(outSpan.count, 0UZ));

        shim::InputSpan<CF>               tail(std::span<const CF>(input), 0UZ);
        shim::OutputSpan<gr::DataSet<CF>> outTail{std::span<gr::DataSet<CF>>(scratch)};
        expect(block.processEpilogue(tail, outTail) == gr::work::Status::ERROR);
        expect(eq(outTail.count, 0UZ));
    };

    "a block without a numerology takes signal_name alone, and its first records carry that name"_test = [] {
        const auto       data  = dataCarriers();
        CarrierAllocator block = make({});
        expect(refusal([&block] { change(block, {{"signal_name", std::string("named")}}); }).empty()) << "signal_name alone";
        expect(refusal([&block, &data] { change(block, {{"data_carriers", data}}); }).empty()) << "the data carriers build a numerology";

        const std::vector<CF> input   = ramp(2UZ * data.size());
        const auto            records = drive(block, std::span<const CF>(input), 4096UZ);
        expect(eq(records.size(), 2UZ));
        for (std::size_t r = 0UZ; r < records.size(); ++r) {
            expect(eq(signalName(records[r]), std::string("named"))) << std::format("record {}", r);
        }
    };

    "a block without a numerology takes fft_len alone, and the data carriers then build a numerology of that length"_test = [] {
        static constexpr gr::Size_t kWide = 128U;
        const auto                  data  = dataCarriers();
        CarrierAllocator            block = make({});
        expect(refusal([&block] { change(block, {{"fft_len", kWide}}); }).empty()) << "fft_len alone";
        expect(refusal([&block, &data] { change(block, {{"data_carriers", data}}); }).empty()) << "the data carriers build a numerology";

        const std::vector<CF> input   = ramp(data.size());
        const auto            records = drive(block, std::span<const CF>(input), 4096UZ);
        expect(eq(records.size(), 1UZ));
        if (records.empty()) {
            return;
        }
        expect(eq(records[0UZ].signal_values.size(), std::size_t{kWide})) << "a record of 128 bins";
        for (std::size_t k = 0UZ; k < data.size() && records[0UZ].signal_values.size() == kWide; ++k) {
            const std::size_t bin = gr::ofdm::CarrierMap::binOf(kWide, data[k]).value();
            expect(records[0UZ].signal_values[bin] == input[k]) << std::format("data carrier {} in its bin of 128", data[k]);
        }
    };

    "a refused pilot_symbols that holds a NaN is refused again when it is sent again"_test = [] {
        const auto            data   = dataCarriers();
        const auto            pilots = pilotCarriers();
        const std::vector<CF> cycle{CF(1.f, 0.f), CF(0.f, 1.f), CF(-1.f, 0.f), CF(0.f, -1.f)};
        CarrierAllocator      block = make({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilots}, {"pilot_symbols", interleave(cycle)}});

        std::vector<float> withNaN = interleave(cycle);
        withNaN[2UZ]               = std::numeric_limits<float>::quiet_NaN();
        for (int attempt = 0; attempt < 2; ++attempt) {
            expect(refusal([&block, &withNaN] { change(block, {{"pilot_symbols", withNaN}}); }).contains("pilot_symbols")) << std::format("attempt {}", attempt);
        }

        const std::vector<CF> input   = ramp(data.size());
        const auto            records = drive(block, std::span<const CF>(input), 4096UZ);
        expect(eq(records.size(), 1UZ));
        for (std::size_t p = 0UZ; p < pilots.size() && !records.empty(); ++p) {
            const std::size_t bin = gr::ofdm::CarrierMap::binOf(kFft, pilots[p]).value();
            expect(records[0UZ].signal_values[bin] == cycle[p]) << std::format("pilot slot {} keeps the accepted cycle", p);
        }
    };

    "a sync word that holds a value that is not finite is refused by name, and again when it is sent again"_test = [] {
        const auto data = dataCarriers();
        for (const CF bad : {CF(std::numeric_limits<float>::quiet_NaN(), 0.f), CF(0.f, std::numeric_limits<float>::infinity())}) {
            std::vector<CF> word(kFft, CF(0.5f, 0.5f));
            word[5UZ]                 = bad;
            const std::string message = refusal([&data, &word] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{2U}}}); });
            expect(message.contains("sync_words") && message.contains("bin 5 of word 0")) << std::format("({}, {}): '{}'", bad.real(), bad.imag(), message);
        }

        CarrierAllocator block = make({{"fft_len", kFft}, {"data_carriers", data}, {"frame_len", gr::Size_t{2U}}});
        std::vector<CF>  word(kFft, CF(0.5f, 0.5f));
        word[5UZ] = CF(std::numeric_limits<float>::quiet_NaN(), 0.f);
        for (int attempt = 0; attempt < 2; ++attempt) {
            expect(refusal([&block, &word] { change(block, {{"sync_words", interleave(word)}}); }).contains("sync_words")) << std::format("attempt {}", attempt);
        }
    };

    /**
     * A tag sets `fft_len` mid-run. The framework refuses 63, a value outside its limits, and the run completes with the
     * records of a run without the tag. In a third run the block takes 128 from the tag, and the records after the tag
     * hold 128 bins.
     */
    "an fft_len outside its limits on a tag is refused, and the run goes on"_test = [] {
        using gr::blocks::testing::ProcessFunction;
        const auto            data   = dataCarriers();
        const std::vector<CF> input  = ramp(6UZ * data.size());
        const std::size_t     tagged = 2UZ * data.size() + 10UZ;

        struct Run {
            bool                         completed = false;
            std::vector<gr::DataSet<CF>> records{};
            gr::Size_t                   member   = 0U;
            gr::Size_t                   reported = 0U;
        };
        const auto run = [&data, &input, tagged](std::optional<gr::Size_t> fftLength) {
            gr::Graph graph;
            auto&     source = graph.emplaceBlock<gr::blocks::testing::TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(input.size())}, {"values", gr::Tensor<CF>(input.begin(), input.end())}, {"mark_tag", false}});
            if (fftLength.has_value()) {
                source._tags = {gr::Tag{tagged, gr::property_map{{"fft_len", *fftLength}}}};
            }
            // fft_len stays out of the constructor's settings, at its default of 64, and the tag sets it.
            auto& allocate = graph.emplaceBlock<CarrierAllocator>({{"data_carriers", data}});
            auto& sink     = graph.emplaceBlock<gr::blocks::testing::TagSink<gr::DataSet<CF>, ProcessFunction::USE_PROCESS_BULK>>({{"log_samples", true}, {"log_tags", false}});
            expect(graph.connect<"out", "in">(source, allocate).has_value());
            expect(graph.connect<"out", "in">(allocate, sink).has_value());

            gr::scheduler::Simple scheduler;
            expect(scheduler.exchange(std::move(graph)).has_value());
            Run result;
            result.completed = scheduler.runAndWait().has_value();
            result.records.assign(sink._samples.begin(), sink._samples.end());
            result.member                                = allocate.fft_len.value;
            const std::optional<gr::pmt::Value> reported = allocate.settings().get("fft_len");
            result.reported                              = reported.has_value() && reported->get_if<gr::Size_t>() != nullptr ? *reported->get_if<gr::Size_t>() : 0U;
            return result;
        };

        const Run reference = run(std::nullopt);
        expect(reference.completed);
        expect(eq(reference.records.size(), 6UZ));

        const Run refused = run(gr::Size_t{63U});
        expect(refused.completed) << "the refusal ends no run";
        expect(eq(refused.member, kFft)) << "the member keeps the accepted value";
        expect(eq(refused.reported, kFft)) << "settings().get() returns the accepted value";
        expect(eq(refused.records.size(), reference.records.size()));
        for (std::size_t r = 0UZ; r < std::min(refused.records.size(), reference.records.size()); ++r) {
            expect(std::ranges::equal(refused.records[r].signal_values, reference.records[r].signal_values)) << std::format("record {} on 64 bins", r);
        }

        const Run taken = run(gr::Size_t{128U});
        expect(taken.completed);
        expect(eq(taken.member, gr::Size_t{128U})) << "the tag reaches the setting";
        expect(!taken.records.empty() && taken.records.back().signal_values.size() == 128UZ) << "the last record holds 128 bins";
    };

    /**
     * A reset to defaults arrives between two calls. The records after it match those of a block started at the same
     * input sample, and `nDropped()` counts the samples the block held.
     */
    "a reset to defaults mid-run restarts the indices and counts the samples it drops"_test = [] {
        const auto             data = dataCarriers();
        const std::vector<CF>  word(kFft, CF(0.5f, 0.5f));
        const std::vector<CF>  input = ramp(4UZ * data.size());
        const std::size_t      split = data.size() + 20UZ; // a sync word and a data symbol out, and 20 samples held
        const gr::property_map settings{{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{2U}}};

        // The reset restores the settings the constructor names.
        CarrierAllocator block(settings);
        block.init(std::make_shared<gr::Sequence>());
        block.start();
        Driver driver{block};
        expect(eq(driver.feed(std::span<const CF>(input).first(split), 4096UZ), split));
        expect(eq(driver.records.size(), 2UZ)) << "the sync word and the first data symbol";
        expect(nothrow([&block] { block.settings().resetDefaults(); })) << "the reset moves no setting";
        expect(eq(block.nDropped(), std::uint64_t{20})) << "the samples the block held";
        std::ignore = driver.feed(std::span<const CF>(input).subspan(split), 4096UZ);
        driver.end({}, 128UZ);

        CarrierAllocator fresh = make(settings);
        Driver           started{fresh, 128UZ, split};
        std::ignore = started.feed(std::span<const CF>(input).subspan(split), 4096UZ);
        started.end({}, 128UZ);

        const std::size_t after = driver.records.size() > 2UZ ? driver.records.size() - 2UZ : 0UZ;
        expect(eq(after, started.records.size()));
        for (std::size_t r = 0UZ; r < std::min(after, started.records.size()); ++r) {
            const auto& record = driver.records[2UZ + r];
            expect(std::ranges::equal(record.signal_values, started.records[r].signal_values)) << std::format("record {} after the reset", r);
            for (const std::string_view key : {"symbol_index", "frame_index", "symbol_in_frame", "sample_start", "pad_carriers"}) {
                expect(eq(meta(record, key), meta(started.records[r], key))) << std::format("record {} after the reset: {}", r, key);
            }
        }
        expect(eq(block.nPadded(), fresh.nPadded()));
        expect(eq(block.nDropped(), std::uint64_t{20}));
    };

    /**
     * The defaults name half the data carriers, and a change names all of them. The block then holds more samples than
     * a symbol of the defaults takes. A reset to defaults drops them as a change of numerology does, and `nDropped()`
     * keeps that count across the reset.
     */
    "a reset to defaults that moves the numerology keeps the count of the samples it drops"_test = [] {
        const auto                      all = dataCarriers();
        const std::vector<std::int32_t> half(all.begin(), all.begin() + static_cast<std::ptrdiff_t>(all.size() / 2UZ));
        const std::size_t               held  = half.size() + 4UZ;
        const std::vector<CF>           input = ramp(held);

        CarrierAllocator block({{"fft_len", kFft}, {"data_carriers", half}});
        block.init(std::make_shared<gr::Sequence>());
        block.start();
        change(block, {{"data_carriers", all}});
        Driver driver{block};
        expect(eq(driver.feed(std::span<const CF>(input), 4096UZ), held));
        expect(driver.records.empty()) << "a symbol of all the data carriers is not yet whole";
        expect(nothrow([&block] { block.settings().resetDefaults(); }));
        expect(block.data_carriers.value == half) << "the reset restores the defaults";
        expect(eq(block.nDropped(), std::uint64_t{held})) << "the samples the change of numerology dropped";
    };
};

} // namespace qa_ofdm_allocator

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
