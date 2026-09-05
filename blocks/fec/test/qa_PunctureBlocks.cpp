#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/fec/ConvBlocks.hpp>
#include <gnuradio-4.0/fec/PunctureBlocks.hpp>

/*
 * What the puncturing pair promises is index arithmetic over a stated mask, so the tests pin the
 * arithmetic where it is checkable by hand and the composition where it has to hold end to end.
 *
 * The transcription anchors come first: the two published patterns of the (171, 133) code, walked
 * by hand over a short coded frame, name the exact item positions that survive. Those lists are
 * written out rather than computed, because a list computed the way the block computes it would
 * only agree with itself.
 *
 * The rest is composition. A punctured frame decoded back to its information proves that the
 * erasures went back where the deletions were, since a phase error of a single position destroys
 * the frame; the noisy leg proves the decoder's metric still accounts for the channel over a
 * punctured word; and the counted drops prove that a record the pattern cannot describe costs only
 * itself.
 */
namespace qa_puncture_blocks {

using gr::blocks::fec::ConvEncode;
using gr::blocks::fec::Depuncture;
using gr::blocks::fec::Puncture;
using gr::blocks::fec::ViterbiDecodeSoft;

using Record = gr::DataSet<std::uint8_t>;
using Soft   = gr::DataSet<float>;

//! The classic constraint length 7 rate 1/2 code, and the hand-computable constraint length 3 pair.
[[nodiscard]] gr::property_map classic() { return {{"constraint_length", gr::Size_t{7U}}, {"polynomials", std::vector<gr::Size_t>{0171U, 0133U}}}; }
[[nodiscard]] gr::property_map shortest() { return {{"constraint_length", gr::Size_t{3U}}, {"polynomials", std::vector<gr::Size_t>{07U, 05U}}}; }

//! The two published punctures: "1110" takes a rate 1/2 code to rate 2/3, "111001" to rate 3/4.
[[nodiscard]] gr::property_map masked(const char* spelling) { return {{"pattern", std::string(spelling)}}; }

std::uint64_t rng = 0xD1B54A32D192ED03ULL;

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

/**
 * A seeded Gaussian source: a xorshift generator feeding the Box-Muller transform.
 *
 * The channel a punctured frame is measured over has to be the same channel on every run and on
 * every machine, so the noise is built here from an integer state rather than drawn from a library
 * whose sequence is free to change under it.
 */
class Awgn {
public:
    explicit Awgn(std::uint64_t seed) noexcept : _state(seed | 1ULL) {}

    //! A standard normal deviate. Box-Muller makes two at a time and the second is kept.
    [[nodiscard]] double normal() noexcept {
        if (_hasSpare) {
            _hasSpare = false;
            return _spare;
        }
        const double radius = std::sqrt(-2.0 * std::log(uniform()));
        const double angle  = 2.0 * std::numbers::pi * uniform();
        _spare              = radius * std::sin(angle);
        _hasSpare           = true;
        return radius * std::cos(angle);
    }

private:
    //! A value in (0, 1], the open end held away from zero so the logarithm stays finite.
    [[nodiscard]] double uniform() noexcept {
        _state ^= _state << 13U;
        _state ^= _state >> 7U;
        _state ^= _state << 17U;
        return static_cast<double>((_state >> 11U) + 1ULL) * 0x1.0p-53;
    }

    std::uint64_t _state;
    bool          _hasSpare = false;
    double        _spare    = 0.0;
};

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

template<typename T>
struct RecordSink : gr::Block<RecordSink<T>> {
    gr::PortIn<gr::DataSet<T>, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);
    std::vector<gr::DataSet<T>>    _records;
    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const auto& r : inSpan) {
            _records.push_back(r);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/**
 * The channel between the two halves of a punctured chain: a coded bit record becomes soft values
 * on the decoder's convention, a one at +1 and a zero at -1, with Gaussian noise of standard
 * deviation `_sigma` added to each.
 *
 * The noise is seeded from the record's index rather than carried across records, so the damage a
 * given frame takes is the same whatever order or batch size the scheduler runs the graph in. The
 * block counts the values whose sign the noise moved, which is what makes the noisy leg a statement
 * about a frame that was really damaged.
 */
struct SoftChannel : gr::Block<SoftChannel> {
    gr::PortIn<Record, gr::Async> in;
    gr::PortOut<Soft, gr::Async>  out;
    GR_MAKE_REFLECTABLE(SoftChannel, in, out);

    static constexpr std::uint64_t kStride = 0x9E3779B97F4A7C15ULL; ///< one record's step through the seed space

    double        _sigma = 0.0;
    std::uint64_t _seed  = 0x2545F4914F6CDD1DULL;
    std::uint64_t _index = 0ULL;
    std::size_t   _flips = 0UZ; ///< values the noise carried across zero, over every record so far

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(inSpan.size(), outSpan.size());
        for (std::size_t i = 0UZ; i < n; ++i) {
            const Record&      coded  = inSpan[i];
            std::vector<float> values = asSoft(coded.signal_values);
            Awgn               noise(_seed + _index * kStride);
            ++_index;
            for (float& value : values) {
                const float clean = value;
                value             = static_cast<float>(static_cast<double>(clean) + noise.normal() * _sigma);
                if ((clean > 0.0F) != (value > 0.0F)) {
                    ++_flips;
                }
            }
            outSpan[i] = record<float>(std::move(values), coded.meta_information.empty() ? gr::property_map{} : coded.meta_information[0UZ]);
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
template<typename TBlock, typename TIn, typename TOut, typename Inspect>
[[nodiscard]] std::vector<gr::DataSet<TOut>> runBlock(gr::property_map settings, std::vector<gr::DataSet<TIn>> records, Inspect&& inspect) {
    gr::Graph flow;
    auto&     src = flow.emplaceBlock<RecordSource<TIn>>();
    src._records  = std::move(records);
    auto& stage   = flow.emplaceBlock<TBlock>(std::move(settings));
    auto& sink    = flow.emplaceBlock<RecordSink<TOut>>();
    boost::ut::expect(flow.connect<"out", "in">(src, stage).has_value());
    boost::ut::expect(flow.connect<"out", "in">(stage, sink).has_value());

    std::vector<gr::DataSet<TOut>> published;
    runFlow(std::move(flow), [&] {
        inspect(stage);
        published = std::move(sink._records);
    });
    return published;
}

template<typename TBlock, typename TIn, typename TOut>
[[nodiscard]] std::vector<gr::DataSet<TOut>> runBlock(gr::property_map settings, std::vector<gr::DataSet<TIn>> records) {
    return runBlock<TBlock, TIn, TOut>(std::move(settings), std::move(records), [](const TBlock&) {});
}

//! Encode @p info under @p settings and hand back the single coded record it becomes.
[[nodiscard]] Record encoded(gr::property_map settings, const std::vector<std::uint8_t>& info) {
    std::vector<Record> out = runBlock<ConvEncode, std::uint8_t, std::uint8_t>(std::move(settings), {record<std::uint8_t>(info)});
    boost::ut::expect(boost::ut::eq(out.size(), 1UZ));
    return out.empty() ? Record{} : out[0UZ];
}

//! The items of @p values at @p positions, which the anchors spell out by hand.
template<typename T>
[[nodiscard]] std::vector<T> at(const std::vector<T>& values, const std::vector<std::size_t>& positions) {
    std::vector<T> picked;
    picked.reserve(positions.size());
    for (const std::size_t p : positions) {
        boost::ut::expect(p < values.size());
        picked.push_back(values[p < values.size() ? p : 0UZ]);
    }
    return picked;
}

//! The deleted positions of @p coded whose bit is a one: what a clean depunctured word must
//! disagree with the transmitted word about, since an erasure sign-slices to zero.
[[nodiscard]] std::size_t erasedOnes(const std::vector<std::uint8_t>& coded, const std::string& mask) {
    std::size_t count = 0UZ;
    for (std::size_t i = 0UZ; i < coded.size(); ++i) {
        if (mask[i % mask.size()] == '0' && (coded[i] & 1U) != 0U) {
            ++count;
        }
    }
    return count;
}

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

//! Construct, apply and start: a refusal a batch that moved no value cannot reach at apply time is reached here.
template<typename TBlock>
void started(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

//! One end-to-end run of encode, puncture, channel and soft decode, and what came back out.
struct ChainResult {
    std::vector<Record> info;    ///< the frames sent
    std::vector<Record> decoded; ///< the frames the decoder published
    std::uint64_t       kept   = 0ULL;
    std::uint64_t       filled = 0ULL;
    std::size_t         flips  = 0UZ;
};

//! Send @p frames seeded frames of @p infoBits bits through
//! ConvEncode -> Puncture -> channel -> Depuncture -> ViterbiDecodeSoft in one scheduler graph.
[[nodiscard]] ChainResult runChain(const char* mask, std::size_t frames, std::size_t infoBits, double sigma) {
    ChainResult                            result;
    std::vector<Record>                    sources;
    std::vector<std::vector<std::uint8_t>> info(frames);
    sources.reserve(frames);
    for (std::size_t f = 0UZ; f < frames; ++f) {
        info[f] = randomBits(infoBits);
        sources.push_back(record<std::uint8_t>(info[f], {{"seq", static_cast<gr::Size_t>(f)}}));
        result.info.push_back(sources.back());
    }

    gr::Graph flow;
    auto&     src    = flow.emplaceBlock<RecordSource<std::uint8_t>>();
    src._records     = std::move(sources);
    auto& coder      = flow.emplaceBlock<ConvEncode>(classic());
    auto& puncture   = flow.emplaceBlock<Puncture>(masked(mask));
    auto& channel    = flow.emplaceBlock<SoftChannel>();
    channel._sigma   = sigma;
    auto& depuncture = flow.emplaceBlock<Depuncture>(masked(mask));
    auto& decoder    = flow.emplaceBlock<ViterbiDecodeSoft>(classic());
    auto& sink       = flow.emplaceBlock<RecordSink<std::uint8_t>>();
    boost::ut::expect(flow.connect<"out", "in">(src, coder).has_value());
    boost::ut::expect(flow.connect<"out", "in">(coder, puncture).has_value());
    boost::ut::expect(flow.connect<"out", "in">(puncture, channel).has_value());
    boost::ut::expect(flow.connect<"out", "in">(channel, depuncture).has_value());
    boost::ut::expect(flow.connect<"out", "in">(depuncture, decoder).has_value());
    boost::ut::expect(flow.connect<"out", "in">(decoder, sink).has_value());

    runFlow(std::move(flow), [&] {
        result.kept    = puncture.nBits;
        result.filled  = depuncture.nBits;
        result.flips   = channel._flips;
        result.decoded = std::move(sink._records);
    });
    return result;
}

} // namespace qa_puncture_blocks

using namespace qa_puncture_blocks;

int main() {
    using namespace boost::ut;

    // First part: the pattern is checked before a record arrives — at the apply where it moved a value,
    // and at start where it named its own default and so moved none.
    "the pattern setting is required and is checked before anything runs"_test = [] {
        expect(throws([] { started<Puncture>(masked("")); })) << "an empty pattern names no rate match";
        expect(throws([] { started<Depuncture>(masked("")); })) << "an empty pattern names no rate match";
        expect(throws([] { std::ignore = make<Puncture>(masked("1102")); })) << "a pattern spells keep and delete only";
        expect(throws([] { std::ignore = make<Depuncture>(masked("11 0")); })) << "a pattern spells keep and delete only";
        expect(throws([] { std::ignore = make<Puncture>(masked("0000")); })) << "a pattern that deletes everything carries nothing";
        expect(throws([] { std::ignore = make<Depuncture>(masked("0")); })) << "a pattern that deletes everything carries nothing";
        expect(nothrow([] { std::ignore = make<Puncture>(masked("1110")); })) << "the published rate 2/3 puncture";
        expect(nothrow([] { std::ignore = make<Depuncture>(masked("111001")); })) << "the published rate 3/4 puncture";
    };

    // The two published patterns, walked by hand over a short coded frame.
    "the published patterns keep the positions the mask names"_test = [] {
        // "1110" over four steps of a rate 1/2 code: eight coded bits, the fourth of every four
        // deleted, which is the rate 2/3 puncture.
        const Record twoThirds = encoded(shortest(), randomBits(2UZ));
        expect(eq(twoThirds.signal_values.size(), 8UZ));
        const auto keptTwoThirds = runBlock<Puncture, std::uint8_t, std::uint8_t>(masked("1110"), {twoThirds}, [](const Puncture& block) {
            expect(eq(block.nRecords, std::uint64_t{1ULL}));
            expect(eq(block.nBits, std::uint64_t{6ULL}));
        });
        expect(eq(keptTwoThirds.size(), 1UZ));
        if (keptTwoThirds.size() == 1UZ) {
            expect(std::ranges::equal(keptTwoThirds[0UZ].signal_values, at(twoThirds.signal_values, {0UZ, 1UZ, 2UZ, 4UZ, 5UZ, 6UZ})));
        }

        // "111001" over six steps: twelve coded bits, the fourth, fifth, tenth and eleventh
        // deleted, which is the rate 3/4 puncture.
        const Record threeQuarters = encoded(shortest(), randomBits(4UZ));
        expect(eq(threeQuarters.signal_values.size(), 12UZ));
        const auto keptThreeQuarters = runBlock<Puncture, std::uint8_t, std::uint8_t>(masked("111001"), {threeQuarters}, [](const Puncture& block) { expect(eq(block.nBits, std::uint64_t{8ULL})); });
        expect(eq(keptThreeQuarters.size(), 1UZ));
        if (keptThreeQuarters.size() == 1UZ) {
            expect(std::ranges::equal(keptThreeQuarters[0UZ].signal_values, at(threeQuarters.signal_values, {0UZ, 1UZ, 2UZ, 5UZ, 6UZ, 7UZ, 8UZ, 11UZ})));
        }
    };

    // The phase is the record's, so a record that ends mid-period costs the next one
    // nothing.
    "the pattern phase resets at every record"_test = [] {
        const std::vector<std::uint8_t> bits = randomBits(10UZ); // ten positions over a period of four
        const auto                      out  = runBlock<Puncture, std::uint8_t, std::uint8_t>(masked("1110"), {record<std::uint8_t>(bits), record<std::uint8_t>(bits)}, [](const Puncture& block) {
            expect(eq(block.nRecords, std::uint64_t{2ULL}));
            expect(eq(block.nBits, std::uint64_t{16ULL}));
        });
        expect(eq(out.size(), 2UZ));
        if (out.size() != 2UZ) {
            return;
        }
        expect(std::ranges::equal(out[0UZ].signal_values, at(bits, {0UZ, 1UZ, 2UZ, 4UZ, 5UZ, 6UZ, 8UZ, 9UZ})));
        expect(std::ranges::equal(out[0UZ].signal_values, out[1UZ].signal_values)) << "identical records puncture identically";
    };

    "a depunctured record puts the erasures back where the deletions were"_test = [] {
        const Record       coded = encoded(shortest(), randomBits(2UZ)); // eight coded bits
        std::vector<float> kept  = asSoft(at(coded.signal_values, {0UZ, 1UZ, 2UZ, 4UZ, 5UZ, 6UZ}));

        const auto out = runBlock<Depuncture, float, float>(masked("1110"), {record<float>(kept)}, [](const Depuncture& block) {
            expect(eq(block.nRecords, std::uint64_t{1ULL}));
            expect(eq(block.nBits, std::uint64_t{8ULL}));
        });
        expect(eq(out.size(), 1UZ));
        if (out.size() != 1UZ) {
            return;
        }
        const std::vector<float> filled = asSoft(coded.signal_values);
        for (std::size_t i = 0UZ; i < 8UZ; ++i) {
            const float want = (i % 4UZ == 3UZ) ? 0.0F : filled[i];
            expect(eq(out[0UZ].signal_values[i], want)) << "position" << i;
        }
    };

    // The whole row in one graph, at both published rates, on a clean channel.
    "a frame round-trips through the punctured chain at rate 2/3 and rate 3/4"_test = [] {
        struct Leg {
            const char*   mask;
            std::uint64_t kept; // punctured bits per frame, from 204 coded bits
        };
        // 96 information bits terminate into 204 coded bits, a whole number of periods of both
        // published patterns: 51 of "1110" and 34 of "111001".
        const std::array<Leg, 2UZ> legs{{{"1110", 153ULL}, {"111001", 136ULL}}};

        for (const Leg& leg : legs) {
            const ChainResult result = runChain(leg.mask, 3UZ, 96UZ, 0.0);
            expect(eq(result.decoded.size(), 3UZ)) << leg.mask;
            expect(eq(result.kept, 3ULL * leg.kept)) << leg.mask;
            expect(eq(result.filled, 3ULL * 204ULL)) << leg.mask;
            if (result.decoded.size() != 3UZ) {
                continue;
            }
            for (std::size_t f = 0UZ; f < 3UZ; ++f) {
                expect(std::ranges::equal(result.decoded[f].signal_values, result.info[f].signal_values)) << leg.mask << "frame" << f;
                expect(eq(metaCount(result.decoded[f], "seq"), static_cast<gr::Size_t>(f))) << "the frames leave in the order they arrived";

                // On a clean channel the decoded path is the transmitted word, so the distance the
                // decoder reports is exactly the deleted positions that carried a one: an erasure
                // sign-slices to zero, and the pattern says where every one of them is.
                const Record      coded  = encoded(classic(), result.info[f].signal_values);
                const std::size_t erased = erasedOnes(coded.signal_values, std::string(leg.mask));
                expect(eq(metaCount(result.decoded[f], "corrected_errors"), static_cast<gr::Size_t>(erased))) << leg.mask << "frame" << f;
            }
        }
    };

    // The same chain over a mild seeded channel.
    "a punctured frame survives a mild noisy channel and the decoder still accounts for it"_test = [] {
        const ChainResult result = runChain("1110", 2UZ, 96UZ, 0.4);
        expect(eq(result.decoded.size(), 2UZ));
        expect(result.flips > 0UZ) << "the channel really moved a sign, so the frame was damaged";
        if (result.decoded.size() != 2UZ) {
            return;
        }
        std::size_t charged = 0UZ;
        for (std::size_t f = 0UZ; f < 2UZ; ++f) {
            expect(std::ranges::equal(result.decoded[f].signal_values, result.info[f].signal_values)) << "frame" << f;
            const gr::Size_t reported = metaCount(result.decoded[f], "corrected_errors");
            expect(reported > gr::Size_t{0U}) << "frame" << f;

            // The decode is exact, so the path the distance is measured against is the transmitted
            // word. What the sign-sliced input disagrees with it about is then the deleted positions
            // that carried a one, which the pattern fixes, plus the values the channel moved across
            // zero. Subtracting the first leaves the second, and it has to be what the channel did.
            const Record      coded  = encoded(classic(), result.info[f].signal_values);
            const std::size_t erased = erasedOnes(coded.signal_values, std::string("1110"));
            expect(static_cast<std::size_t>(reported) >= erased) << "frame" << f;
            charged += static_cast<std::size_t>(reported) - erased;
        }
        expect(eq(charged, result.flips)) << "the distance charges the channel for exactly the signs it moved";
    };

    // Second part: what neither block can describe is counted rather than absorbed,
    // and the record after it goes through.
    "an empty record is refused by the puncture, and the record after it is punctured"_test = [] {
        const std::vector<std::uint8_t> good = randomBits(8UZ);
        const auto                      out  = runBlock<Puncture, std::uint8_t, std::uint8_t>(masked("1110"),      //
                                  {record<std::uint8_t>(std::vector<std::uint8_t>{}), record<std::uint8_t>(good)}, //
                                  [](const Puncture& block) {
                expect(eq(block.nRecordsRefused, std::uint64_t{1ULL}));
                expect(eq(block.nRecords, std::uint64_t{1ULL}));
                expect(eq(block.nBits, std::uint64_t{6ULL}));
            });
        expect(eq(out.size(), 1UZ)) << "only the record the pattern could describe was published";
    };

    "a record inconsistent with the pattern is refused by the depuncture"_test = [] {
        // A coded frame of ten bits is not a whole number of periods of "1110", so the eight bits
        // it punctures to name no length to expand back to: the depuncture counts it and moves on.
        const Record              shortFrame = encoded(shortest(), randomBits(3UZ)); // ten coded bits
        const std::vector<Record> punctured  = runBlock<Puncture, std::uint8_t, std::uint8_t>(masked("1110"), {shortFrame});
        expect(eq(punctured.size(), 1UZ));
        if (punctured.size() != 1UZ) {
            return;
        }
        expect(eq(punctured[0UZ].signal_values.size(), 8UZ)) << "eight kept bits, and eight is not a multiple of three";

        const Soft inconsistent = record<float>(asSoft(punctured[0UZ].signal_values));
        const Soft empty        = record<float>(std::vector<float>{});
        const Soft whole        = record<float>(asSoft(std::vector<std::uint8_t>(6UZ, 1U)));

        const auto out = runBlock<Depuncture, float, float>(masked("1110"), {inconsistent, empty, whole}, [](const Depuncture& block) {
            expect(eq(block.nRecordsRefused, std::uint64_t{2ULL}));
            expect(eq(block.nRecords, std::uint64_t{1ULL}));
            expect(eq(block.nBits, std::uint64_t{8ULL}));
        });
        expect(eq(out.size(), 1UZ));
        if (out.size() == 1UZ) {
            expect(eq(out[0UZ].signal_values.size(), 8UZ)) << "six kept values are two whole periods";
        }
    };

    // The record's own facts cross both blocks, and neither writes over them.
    "both blocks carry a record's metadata verbatim and write nothing"_test = [] {
        gr::property_map meta;
        meta["corrected_errors"] = gr::Size_t{3U};
        meta["provenance"]       = std::string("earlier stage");
        meta["sample_start"]     = std::uint64_t{4096ULL};

        const auto punctured = runBlock<Puncture, std::uint8_t, std::uint8_t>(masked("1110"), {record<std::uint8_t>(randomBits(8UZ), meta)});
        expect(eq(punctured.size(), 1UZ));
        if (punctured.size() != 1UZ) {
            return;
        }
        const auto filled = runBlock<Depuncture, float, float>(masked("1110"), {record<float>(asSoft(punctured[0UZ].signal_values), meta)});
        expect(eq(filled.size(), 1UZ));
        if (filled.size() != 1UZ) {
            return;
        }

        for (const gr::property_map& map : {punctured[0UZ].meta_information[0UZ], filled[0UZ].meta_information[0UZ]}) {
            expect(eq(map.size(), 3UZ)) << "neither block has a status to add";
            const auto errors = map.find(gr::property_map::key_type("corrected_errors"));
            expect(errors != map.end());
            if (errors != map.end()) {
                expect(eq(errors->second.value_or(gr::Size_t{0U}), gr::Size_t{3U})) << "what an earlier stage counted is not touched";
            }
            const auto who = map.find(gr::property_map::key_type("provenance"));
            expect(who != map.end());
            if (who != map.end()) {
                expect(eq(who->second.value_or(std::string{}), std::string("earlier stage")));
            }
            const auto at_ = map.find(gr::property_map::key_type("sample_start"));
            expect(at_ != map.end());
            if (at_ != map.end()) {
                expect(eq(at_->second.value_or(std::uint64_t{0ULL}), std::uint64_t{4096ULL}));
            }
        }
        expect(eq(punctured[0UZ].signal_names.size(), 1UZ)) << "the record's signal name follows it through";
        expect(eq(filled[0UZ].signal_names.size(), 1UZ));
    };

    "a record arriving without a metadata map gains one and nothing else"_test = [] {
        Record bare = record<std::uint8_t>(randomBits(8UZ));
        bare.meta_information.clear();
        const auto out = runBlock<Puncture, std::uint8_t, std::uint8_t>(masked("1110"), {bare});
        expect(eq(out.size(), 1UZ));
        if (out.size() == 1UZ) {
            expect(eq(out[0UZ].meta_information.size(), 1UZ));
            expect(eq(out[0UZ].meta_information[0UZ].size(), 0UZ)) << "the puncture has nothing to say";
        }
    };

    return 0;
}
