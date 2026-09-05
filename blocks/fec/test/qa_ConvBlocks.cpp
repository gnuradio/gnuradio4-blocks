#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/fec/ConvBlocks.hpp>

/*
 * The adapters are exercised through the scheduler rather than by calling them directly, because
 * what they promise is a record contract: one frame in, one frame out, in order, with the decode's
 * account of the channel accumulating in metadata and a record that is not a frame counted rather
 * than absorbed.
 *
 * The kernel's own correctness is established beside it and is not restated here. What these tests
 * pin is the layer this module owns: the item order of a coded frame, the arithmetic that turns a
 * record's length into an information length and back, the metadata vocabulary, and the counters.
 */
namespace qa_conv_blocks {

using gr::blocks::fec::ConvEncode;
using gr::blocks::fec::ViterbiDecode;
using gr::blocks::fec::ViterbiDecodeSoft;

using Record = gr::DataSet<std::uint8_t>;
using Soft   = gr::DataSet<float>;

//! The classic constraint length 7 rate 1/2 code, and the hand-computable constraint length 3 pair.
[[nodiscard]] gr::property_map classic() { return {{"constraint_length", gr::Size_t{7U}}, {"polynomials", std::vector<gr::Size_t>{0171U, 0133U}}}; }
[[nodiscard]] gr::property_map shortest() { return {{"constraint_length", gr::Size_t{3U}}, {"polynomials", std::vector<gr::Size_t>{07U, 05U}}}; }

std::uint64_t rng = 0x9E3779B97F4A7C15ULL;

[[nodiscard]] std::uint64_t next() {
    rng = rng * 6364136223846793005ULL + 1442695040888963407ULL;
    return rng >> 17U;
}

//! @p count seeded random bits.
[[nodiscard]] std::vector<std::uint8_t> randomBits(std::size_t count) {
    std::vector<std::uint8_t> bits(count);
    for (std::size_t i = 0UZ; i < count; ++i) {
        bits[i] = static_cast<std::uint8_t>(next() & 1ULL);
    }
    return bits;
}

//! A record of @p values, shaped as a record producer shapes one.
template<typename T>
[[nodiscard]] gr::DataSet<T> record(std::vector<T> values, gr::property_map meta = {}) {
    gr::DataSet<T> r;
    r.signal_values = std::move(values);
    r.extents.push_back(static_cast<std::int32_t>(r.signal_values.size()));
    r.signal_names.emplace_back("fec");
    r.timing_events.resize(1UZ);
    if (!meta.empty()) {
        r.meta_information.push_back(std::move(meta));
    }
    return r;
}

//! A coded record as soft values, a one becoming +1 and a zero -1: full confidence either way.
[[nodiscard]] std::vector<float> asSoft(const std::vector<std::uint8_t>& coded) {
    std::vector<float> values(coded.size());
    for (std::size_t i = 0UZ; i < coded.size(); ++i) {
        values[i] = ((coded[i] & 1U) != 0U) ? 1.0F : -1.0F;
    }
    return values;
}

//! A record's count under @p key, with the key's absence reported rather than defaulted away.
template<typename T>
[[nodiscard]] gr::Size_t metaCount(const gr::DataSet<T>& r, const char* key) {
    boost::ut::expect(!r.meta_information.empty()) << "the record carries a metadata map";
    if (r.meta_information.empty()) {
        return 0U;
    }
    const auto& map   = r.meta_information[0UZ];
    const auto  entry = map.find(gr::property_map::key_type(key));
    boost::ut::expect(entry != map.end()) << key;
    return entry == map.end() ? 0U : entry->second.value_or(gr::Size_t{0U});
}

template<typename T>
struct RecordSource : gr::Block<RecordSource<T>> {
    gr::PortOut<gr::DataSet<T>, gr::Async> out;
    GR_MAKE_REFLECTABLE(RecordSource, out);
    std::vector<gr::DataSet<T>>    _records;
    std::size_t                    _pos{0UZ};
    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(outSpan.size(), _records.size() - _pos);
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = _records[_pos + i];
        }
        outSpan.publish(n);
        _pos += n;
        return _pos == _records.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<Record, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);
    std::vector<Record>            _records;
    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const auto& r : inSpan) {
            _records.push_back(r);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

//! Flips the bits at fixed item positions of every record passing through, so the damage a chain
//! sees is the same on every run and on every schedule.
struct BitFlipper : gr::Block<BitFlipper> {
    gr::PortIn<Record, gr::Async>  in;
    gr::PortOut<Record, gr::Async> out;
    GR_MAKE_REFLECTABLE(BitFlipper, in, out);
    std::vector<std::size_t>       _positions;
    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(inSpan.size(), outSpan.size());
        for (std::size_t i = 0UZ; i < n; ++i) {
            Record damaged = inSpan[i];
            for (const std::size_t p : _positions) {
                if (p < damaged.signal_values.size()) {
                    damaged.signal_values[p] = static_cast<std::uint8_t>(damaged.signal_values[p] ^ 1U);
                }
            }
            outSpan[i] = std::move(damaged);
        }
        std::ignore = inSpan.consume(n);
        outSpan.publish(n);
        return n == 0UZ ? gr::work::Status::INSUFFICIENT_INPUT_ITEMS : gr::work::Status::OK;
    }
};

//! Run @p flow to completion under the simple scheduler, then call @p after while its blocks are
//! still alive. A run that has not finished inside the timeout is stopped and reported.
template<typename After>
void runFlow(gr::Graph&& flow, After&& after) {
    gr::scheduler::Simple<> scheduler;
    boost::ut::expect(scheduler.exchange(std::move(flow)).has_value());
    std::atomic<bool> done{false};
    std::thread       runner([&scheduler, &done] {
        std::ignore = scheduler.runAndWait();
        done        = true;
    });
    const auto        start = std::chrono::steady_clock::now();
    while (!done.load() && std::chrono::steady_clock::now() - start < std::chrono::seconds(30)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!done.load()) {
        scheduler.requestStop();
    }
    runner.join();
    boost::ut::expect(done.load());
    after();
}

//! Run @p records through one block configured by @p settings, handing the block to @p inspect
//! before the graph is torn down.
template<typename TBlock, typename TIn, typename Inspect>
[[nodiscard]] std::vector<Record> runBlock(gr::property_map settings, std::vector<gr::DataSet<TIn>> records, Inspect&& inspect) {
    gr::Graph flow;
    auto&     src = flow.emplaceBlock<RecordSource<TIn>>();
    src._records  = std::move(records);
    auto& coder   = flow.emplaceBlock<TBlock>(std::move(settings));
    auto& sink    = flow.emplaceBlock<RecordSink>();
    boost::ut::expect(flow.connect<"out", "in">(src, coder).has_value());
    boost::ut::expect(flow.connect<"out", "in">(coder, sink).has_value());

    std::vector<Record> published;
    runFlow(std::move(flow), [&] {
        inspect(coder);
        published = std::move(sink._records);
    });
    return published;
}

template<typename TBlock, typename TIn>
[[nodiscard]] std::vector<Record> runBlock(gr::property_map settings, std::vector<gr::DataSet<TIn>> records) {
    return runBlock<TBlock, TIn>(std::move(settings), std::move(records), [](const TBlock&) {});
}

//! Encode @p info under @p settings and hand back the single coded record it becomes.
[[nodiscard]] Record encoded(gr::property_map settings, const std::vector<std::uint8_t>& info) {
    std::vector<Record> out = runBlock<ConvEncode, std::uint8_t>(std::move(settings), {record<std::uint8_t>(info)});
    boost::ut::expect(boost::ut::eq(out.size(), 1UZ));
    return out.empty() ? Record{} : out[0UZ];
}

//! Flip the bits at @p positions of @p r.
void flipBits(Record& r, const std::vector<std::size_t>& positions) {
    for (const std::size_t p : positions) {
        boost::ut::expect(p < r.signal_values.size());
        if (p < r.signal_values.size()) {
            r.signal_values[p] = static_cast<std::uint8_t>(r.signal_values[p] ^ 1U);
        }
    }
}

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

} // namespace qa_conv_blocks

using namespace qa_conv_blocks;

int main() {
    using namespace boost::ut;

    "the code settings are required and are checked before anything runs"_test = [] {
        expect(throws([] {
            ConvEncode block = make<ConvEncode>({{"constraint_length", gr::Size_t{0U}}, {"polynomials", std::vector<gr::Size_t>{}}});
            block.start();
        })) << "a convolutional code has no default: the empty spelling is the declared one, so it reaches start(), which refuses";
        expect(throws([] { std::ignore = make<ViterbiDecode>({{"constraint_length", gr::Size_t{2U}}, {"polynomials", std::vector<gr::Size_t>{03U, 02U}}}); })) << "below three there is no code";
        expect(throws([] { std::ignore = make<ViterbiDecode>({{"constraint_length", gr::Size_t{10U}}, {"polynomials", std::vector<gr::Size_t>{01151U, 01753U}}}); })) << "above nine the family stops";
        expect(throws([] { std::ignore = make<ConvEncode>({{"constraint_length", gr::Size_t{7U}}, {"polynomials", std::vector<gr::Size_t>{0171U}}}); })) << "one generator is not a rate the family carries";
        expect(throws([] { std::ignore = make<ConvEncode>({{"constraint_length", gr::Size_t{7U}}, {"polynomials", std::vector<gr::Size_t>{0171U, 0171U}}}); })) << "the same generator twice is not two of them";
        expect(throws([] { std::ignore = make<ConvEncode>({{"constraint_length", gr::Size_t{7U}}, {"polynomials", std::vector<gr::Size_t>{0171U, 0333U}}}); })) << "a generator wider than the register";
        expect(throws([] { std::ignore = make<ConvEncode>({{"constraint_length", gr::Size_t{7U}}, {"polynomials", std::vector<gr::Size_t>{0171U, 0U}}}); })) << "a zero generator";
        expect(nothrow([] { std::ignore = make<ViterbiDecodeSoft>(classic()); })) << "the classic pair is a code";
    };

    // The impulse response through a record is the generator polynomials, least
    // significant bit first, in the item order a coded record carries.
    "an impulse through a coded record is the code's generator polynomials"_test = [] {
        struct Anchor {
            gr::property_map               settings;
            std::size_t                    length;
            std::array<std::uint32_t, 2UZ> generators;
        };
        const std::array<Anchor, 2UZ> anchors{{{classic(), 7UZ, {0171U, 0133U}}, {shortest(), 3UZ, {07U, 05U}}}};

        for (const Anchor& anchor : anchors) {
            std::vector<std::uint8_t> impulse(anchor.length, 0U);
            impulse[0UZ] = 1U;

            const Record coded = encoded(anchor.settings, impulse);
            expect(eq(coded.signal_values.size(), (anchor.length + anchor.length - 1UZ) * 2UZ)) << "K" << anchor.length;

            const std::size_t steps = impulse.size() + anchor.length - 1UZ;
            for (std::size_t step = 0UZ; step < steps; ++step) {
                for (std::size_t j = 0UZ; j < 2UZ; ++j) {
                    // Past the register's width the impulse has left it and the output is zero.
                    const std::uint8_t want = (step < anchor.length) ? static_cast<std::uint8_t>((anchor.generators[j] >> step) & 1U) : std::uint8_t{0U};
                    expect(eq(coded.signal_values[step * 2UZ + j], want)) << "K" << anchor.length << "step" << step << "polynomial" << j;
                }
            }
        }
    };

    "a frame round-trips through the encode and the hard decode"_test = [] {
        const std::vector<std::uint8_t> info  = randomBits(100UZ);
        const Record                    coded = encoded(classic(), info);
        expect(eq(coded.signal_values.size(), (100UZ + 6UZ) * 2UZ));

        const auto out = runBlock<ViterbiDecode, std::uint8_t>(classic(), {coded}, [](const ViterbiDecode& block) {
            expect(eq(block.nRecords, std::uint64_t{1ULL}));
            expect(eq(block.nInfoBits, std::uint64_t{100ULL}));
            expect(eq(block.nCorrectedErrors, std::uint64_t{0ULL}));
        });
        expect(eq(out.size(), 1UZ));
        if (out.size() != 1UZ) {
            return;
        }
        expect(std::ranges::equal(out[0UZ].signal_values, info));
        expect(eq(metaCount(out[0UZ], "corrected_errors"), gr::Size_t{0U}));

        // A Viterbi decode cannot refuse, so the key that would always be zero is not written.
        const auto& map = out[0UZ].meta_information[0UZ];
        expect(map.find(gr::property_map::key_type("uncorrectable_errors")) == map.end()) << "there is no refusal to report";
    };

    "a frame round-trips through the encode and the soft decode"_test = [] {
        const std::vector<std::uint8_t> info  = randomBits(100UZ);
        const Record                    coded = encoded(classic(), info);

        const auto out = runBlock<ViterbiDecodeSoft, float>(classic(), {record<float>(asSoft(coded.signal_values))}, [](const ViterbiDecodeSoft& block) {
            expect(eq(block.nRecords, std::uint64_t{1ULL}));
            expect(eq(block.nCorrectedErrors, std::uint64_t{0ULL}));
        });
        expect(eq(out.size(), 1UZ));
        if (out.size() == 1UZ) {
            expect(std::ranges::equal(out[0UZ].signal_values, info));
            expect(eq(metaCount(out[0UZ], "corrected_errors"), gr::Size_t{0U}));
        }
    };

    "the soft decode reads a sign as the bit and a zero as an erasure"_test = [] {
        const std::vector<std::uint8_t> info  = randomBits(100UZ);
        const Record                    coded = encoded(classic(), info);

        std::vector<float> values = asSoft(coded.signal_values);
        values[11UZ]              = -values[11UZ]; // a confident value that is simply wrong
        values[93UZ]              = -values[93UZ];
        for (const std::size_t p : {40UZ, 41UZ, 42UZ, 43UZ}) {
            values[p] = 0.0F; // an erasure, which the correlation weighs at nothing
        }

        const auto out = runBlock<ViterbiDecodeSoft, float>(classic(), {record<float>(values)});
        expect(eq(out.size(), 1UZ));
        if (out.size() == 1UZ) {
            expect(std::ranges::equal(out[0UZ].signal_values, info)) << "two wrong values and four erasures are inside the code's budget";
        }
    };

    // First part: what the record already carried, and what the decode adds to it.
    "the decode adds its distance to the record's count and carries every other key verbatim"_test = [] {
        const std::vector<std::uint8_t> info  = randomBits(100UZ);
        Record                          coded = encoded(classic(), info);
        flipBits(coded, {9UZ, 120UZ}); // two errors far enough apart that each is corrected alone

        coded.meta_information.resize(1UZ);
        coded.meta_information[0UZ]["corrected_errors"] = gr::Size_t{3U};
        coded.meta_information[0UZ]["provenance"]       = std::string("earlier stage");
        coded.meta_information[0UZ]["sample_start"]     = std::uint64_t{4096ULL};

        const auto out = runBlock<ViterbiDecode, std::uint8_t>(classic(), {coded});
        expect(eq(out.size(), 1UZ));
        if (out.size() != 1UZ) {
            return;
        }
        expect(std::ranges::equal(out[0UZ].signal_values, info));
        expect(eq(metaCount(out[0UZ], "corrected_errors"), gr::Size_t{5U})) << "three carried in plus the two this decode disagreed with";

        const auto& map = out[0UZ].meta_information[0UZ];
        const auto  who = map.find(gr::property_map::key_type("provenance"));
        expect(who != map.end());
        if (who != map.end()) {
            expect(eq(who->second.value_or(std::string{}), std::string("earlier stage")));
        }
        const auto at = map.find(gr::property_map::key_type("sample_start"));
        expect(at != map.end());
        if (at != map.end()) {
            expect(eq(at->second.value_or(std::uint64_t{0ULL}), std::uint64_t{4096ULL}));
        }
        expect(eq(out[0UZ].signal_names.size(), 1UZ)) << "the record's signal name follows it through";
    };

    "a record arriving without a metadata map gains one for the decode's account"_test = [] {
        Record coded = encoded(classic(), randomBits(20UZ));
        coded.meta_information.clear();
        const auto out = runBlock<ViterbiDecode, std::uint8_t>(classic(), {coded});
        expect(eq(out.size(), 1UZ));
        if (out.size() == 1UZ) {
            expect(eq(out[0UZ].meta_information.size(), 1UZ));
            expect(eq(metaCount(out[0UZ], "corrected_errors"), gr::Size_t{0U}));
        }
    };

    // Second part: a record that is not a frame is dropped, counted, and costs only
    // itself.
    "an empty record is refused by the encode, and the record after it is encoded"_test = [] {
        const std::vector<std::uint8_t> good = randomBits(50UZ);
        const auto                      out  = runBlock<ConvEncode, std::uint8_t>(classic(),                       //
                                  {record<std::uint8_t>(std::vector<std::uint8_t>{}), record<std::uint8_t>(good)}, //
                                  [](const ConvEncode& block) {
                expect(eq(block.nRecordsRefused, std::uint64_t{1ULL}));
                expect(eq(block.nRecords, std::uint64_t{1ULL}));
                expect(eq(block.nInfoBits, std::uint64_t{50ULL}));
            });
        expect(eq(out.size(), 1UZ)) << "only the record that was a frame was published";
        if (out.size() == 1UZ) {
            expect(eq(out[0UZ].signal_values.size(), (50UZ + 6UZ) * 2UZ));
        }
    };

    "a record that is not a whole frame is refused by the decode"_test = [] {
        const Record coded     = encoded(classic(), randomBits(20UZ));
        Record       truncated = coded;
        truncated.signal_values.pop_back();                                                // an odd length is not a whole number of steps
        const Record tailOnly = record<std::uint8_t>(std::vector<std::uint8_t>(12UZ, 0U)); // the tail alone, carrying nothing

        const auto out = runBlock<ViterbiDecode, std::uint8_t>(classic(), {truncated, tailOnly, coded}, [](const ViterbiDecode& block) {
            expect(eq(block.nRecordsRefused, std::uint64_t{2ULL}));
            expect(eq(block.nRecords, std::uint64_t{1ULL}));
        });
        expect(eq(out.size(), 1UZ));
        if (out.size() == 1UZ) {
            expect(eq(out[0UZ].signal_values.size(), 20UZ)) << "the frame that was whole decoded normally";
        }
    };

    "a soft record that is not a whole frame is refused by the soft decode"_test = [] {
        const Record       coded  = encoded(classic(), randomBits(20UZ));
        std::vector<float> values = asSoft(coded.signal_values);
        std::vector<float> odd    = values;
        odd.pop_back();

        const auto out = runBlock<ViterbiDecodeSoft, float>(classic(), {record<float>(odd), record<float>(values)}, [](const ViterbiDecodeSoft& block) {
            expect(eq(block.nRecordsRefused, std::uint64_t{1ULL}));
            expect(eq(block.nRecords, std::uint64_t{1ULL}));
        });
        expect(eq(out.size(), 1UZ));
    };

    // Third part: encode, damage and decode in one graph, with the records staying in
    // order and every count exact.
    "an encode, a deterministic bit flipper and a decode compose in one graph"_test = [] {
        constexpr std::size_t                  records  = 3UZ;
        constexpr std::size_t                  infoBits = 100UZ;
        std::vector<std::vector<std::uint8_t>> info(records);
        std::vector<Record>                    sources;
        sources.reserve(records);
        for (std::size_t r = 0UZ; r < records; ++r) {
            info[r] = randomBits(infoBits);
            sources.push_back(record<std::uint8_t>(info[r], {{"seq", static_cast<gr::Size_t>(r)}}));
        }

        gr::Graph flow;
        auto&     src    = flow.emplaceBlock<RecordSource<std::uint8_t>>();
        src._records     = std::move(sources);
        auto& coder      = flow.emplaceBlock<ConvEncode>(classic());
        auto& noise      = flow.emplaceBlock<BitFlipper>();
        noise._positions = {5UZ, 61UZ, 150UZ}; // three errors, each alone inside the code's budget
        auto& decoder    = flow.emplaceBlock<ViterbiDecode>(classic());
        auto& sink       = flow.emplaceBlock<RecordSink>();
        expect(flow.connect<"out", "in">(src, coder).has_value());
        expect(flow.connect<"out", "in">(coder, noise).has_value());
        expect(flow.connect<"out", "in">(noise, decoder).has_value());
        expect(flow.connect<"out", "in">(decoder, sink).has_value());

        std::vector<Record> out;
        runFlow(std::move(flow), [&] {
            expect(eq(coder.nRecords, std::uint64_t{records}));
            expect(eq(coder.nInfoBits, std::uint64_t{records * infoBits}));
            expect(eq(decoder.nRecords, std::uint64_t{records}));
            expect(eq(decoder.nInfoBits, std::uint64_t{records * infoBits}));
            expect(eq(decoder.nCorrectedErrors, std::uint64_t{records * 3UZ}));
            out = std::move(sink._records);
        });

        expect(eq(out.size(), records));
        if (out.size() != records) {
            return;
        }
        for (std::size_t r = 0UZ; r < records; ++r) {
            expect(std::ranges::equal(out[r].signal_values, info[r])) << "record" << r;
            expect(eq(metaCount(out[r], "seq"), static_cast<gr::Size_t>(r))) << "the records leave in the order they arrived";
            expect(eq(metaCount(out[r], "corrected_errors"), gr::Size_t{3U})) << "record" << r;
        }
    };

    "a named convention is the explicit settings it supplies"_test = [] {
        const std::vector<std::uint8_t> info = randomBits(96UZ);

        // the two ccsds spellings differ in exactly the second output's complement
        const Record byName     = encoded({{"code", std::string("ccsds")}}, info);
        const Record uninverted = encoded({{"code", std::string("ccsds_uninverted")}}, info);
        expect(eq(byName.signal_values.size(), uninverted.signal_values.size()));
        bool firstEqual       = true;
        bool secondComplement = true;
        for (std::size_t t = 0UZ; 2UZ * t + 1UZ < byName.signal_values.size(); ++t) {
            firstEqual       = firstEqual && byName.signal_values[2UZ * t] == uninverted.signal_values[2UZ * t];
            secondComplement = secondComplement && byName.signal_values[2UZ * t + 1UZ] == (uninverted.signal_values[2UZ * t + 1UZ] ^ 1U);
        }
        expect(firstEqual) << "G1's output is shared";
        expect(secondComplement) << "G2's output carries the convention's inversion";

        // an explicit setting staged beside the name must agree with it
        expect(nothrow([] { std::ignore = make<ConvEncode>({{"code", std::string("ccsds")}, {"constraint_length", gr::Size_t{7U}}}); })) << "agreement is not a conflict";
        expect(nothrow([] { std::ignore = make<ConvEncode>({{"code", std::string("ccsds")}, {"invert_outputs", gr::Size_t{0b10U}}}); }));
        expect(throws([] { std::ignore = make<ConvEncode>({{"code", std::string("ccsds")}, {"constraint_length", gr::Size_t{5U}}}); })) << "one question, two answers";
        expect(throws([] { std::ignore = make<ConvEncode>({{"code", std::string("ccsds")}, {"polynomials", std::vector<gr::Size_t>{0171U, 0133U}}}); })) << "the standard's own octal spelling is the reversed code, and disagrees";
        expect(throws([] { std::ignore = make<ConvEncode>({{"code", std::string("ccsds")}, {"invert_outputs", gr::Size_t{0b01U}}}); }));
        expect(throws([] { std::ignore = make<ViterbiDecode>({{"code", std::string("voyager")}}); })) << "an unknown name is refused, listing the ones there are";
    };

    "inversion is a relabelling"_test = [] {
        const std::vector<std::uint8_t> info  = randomBits(80UZ);
        const Record                    plain = encoded(classic(), info);

        gr::property_map inverted  = classic();
        inverted["invert_outputs"] = gr::Size_t{0b01U};
        const Record flipped       = encoded(inverted, info);
        bool         firstFlipped  = true;
        bool         secondEqual   = true;
        for (std::size_t t = 0UZ; 2UZ * t + 1UZ < plain.signal_values.size(); ++t) {
            firstFlipped = firstFlipped && flipped.signal_values[2UZ * t] == (plain.signal_values[2UZ * t] ^ 1U);
            secondEqual  = secondEqual && flipped.signal_values[2UZ * t + 1UZ] == plain.signal_values[2UZ * t + 1UZ];
        }
        expect(firstFlipped) << "the masked output is emitted complemented";
        expect(secondEqual) << "the unmasked one is untouched";

        const auto out = runBlock<ViterbiDecode, std::uint8_t>(inverted, {flipped});
        expect(eq(out.size(), 1UZ));
        if (!out.empty()) {
            expect(std::ranges::equal(out[0UZ].signal_values, info)) << "decode under the same mask reproduces the data";
            expect(eq(metaCount(out[0UZ], "corrected_errors"), gr::Size_t{0U})) << "at the uninverted decode's distance";
        }
    };

    "an open record decodes every step and drops no tail"_test = [] {
        const std::vector<std::uint8_t> info  = randomBits(120UZ);
        const Record                    coded = encoded(classic(), info);

        gr::property_map open = classic();
        open["termination"]   = std::string("open");
        const auto out        = runBlock<ViterbiDecode, std::uint8_t>(open, {coded});
        expect(eq(out.size(), 1UZ));
        if (!out.empty()) {
            expect(eq(out[0UZ].signal_values.size(), 126UZ)) << "every step of an open record carries an information bit";
            expect(std::ranges::equal(std::span(out[0UZ].signal_values).first(120UZ), info)) << "the data is where the terminated decode put it";
            expect(that % std::ranges::count(std::span(out[0UZ].signal_values).subspan(120UZ), std::uint8_t{1U}) == 0) << "and the tail decodes as the zeros it is";
        }

        // a record too short to be a terminated frame is still an open record
        const std::vector<std::uint8_t> twoBits(coded.signal_values.begin(), coded.signal_values.begin() + 2);
        const auto                      oneStep = runBlock<ViterbiDecode, std::uint8_t>(open, {record<std::uint8_t>(twoBits)});
        expect(eq(oneStep.size(), 1UZ)) << "one step is a record under 'open'";
        std::ignore = runBlock<ViterbiDecode, std::uint8_t>(classic(), {record<std::uint8_t>(twoBits)}, [](const ViterbiDecode& block) { //
            expect(eq(block.nRecordsRefused, std::uint64_t{1ULL})) << "and a counted refusal under 'terminated'";
        });

        expect(throws([] {
            std::ignore = make<ViterbiDecode>([] {
                auto s           = classic();
                s["termination"] = std::string("running");
                return s;
            }());
        })) << "an unrecognized mode names its two values";
        expect(nothrow([] {
            std::ignore = make<ViterbiDecodeSoft>([] {
                auto s           = classic();
                s["termination"] = std::string("open");
                return s;
            }());
        })) << "the soft decoder takes the mode too";
    };

    return 0;
}
