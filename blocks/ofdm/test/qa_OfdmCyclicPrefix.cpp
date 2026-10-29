#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
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
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/ofdm/CarrierAllocator.hpp>
#include <gnuradio-4.0/ofdm/CyclicPrefix.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_ofdm_cp {

using gr::blocks::ofdm::CpInsert;
using gr::blocks::ofdm::CpRemove;
using CF       = std::complex<float>;
namespace shim = gr::blocks::testing::span;

constexpr std::size_t kFft = 64UZ;

/// The reference inverse transform: the library FFT the block uses, scaled the same way.
[[nodiscard]] std::vector<CF> inverseTransform(std::span<const CF> spectrum) {
    gr::algorithm::FFT<CF, CF, gr::algorithm::Direction::Backward> inverse{};
    std::vector<CF>                                                time(spectrum.size());
    inverse.compute(spectrum, std::span<CF>(time));
    const float scale = 1.f / static_cast<float>(spectrum.size());
    for (CF& value : time) {
        value *= scale;
    }
    return time;
}

[[nodiscard]] std::vector<CF> forwardTransform(std::span<const CF> time) {
    gr::algorithm::FFT<CF, CF, gr::algorithm::Direction::Forward> forward{};
    std::vector<CF>                                               spectrum(time.size());
    forward.compute(time, std::span<CF>(spectrum));
    return spectrum;
}

/// A frequency-domain symbol of distinct values. A record without @p symbolInFrame carries no `symbol_in_frame`.
[[nodiscard]] gr::DataSet<CF> symbol(std::size_t index, std::optional<std::uint64_t> symbolInFrame, std::size_t fftLength = kFft) {
    std::vector<CF> values(fftLength);
    for (std::size_t k = 0UZ; k < fftLength; ++k) {
        values[k] = CF(static_cast<float>(index + 1UZ) * 0.125f * static_cast<float>(k + 1UZ), 0.25f * static_cast<float>(k) - static_cast<float>(index));
    }
    std::vector<int> carriers(fftLength);
    for (std::size_t bin = 0UZ; bin < fftLength; ++bin) {
        carriers[bin] = gr::ofdm::CarrierMap::carrierOf(fftLength, bin).value();
    }
    gr::property_map meta;
    if (symbolInFrame.has_value()) {
        meta[std::pmr::string("symbol_in_frame")] = gr::pmt::Value(*symbolInFrame);
    }
    return gr::blocks::ofdm::detail::makeSymbolRecord(std::move(values), std::span<const int>(carriers), "data", "test", std::move(meta));
}

struct Stream {
    std::vector<CF>      samples{};
    std::vector<gr::Tag> tags{};
};

/// @brief Drives `CpInsert` until it has consumed every record.
[[nodiscard]] Stream insert(CpInsert& block, std::span<const gr::DataSet<CF>> records, std::size_t outRoom) {
    Stream          result;
    std::vector<CF> scratch(outRoom);

    std::size_t consumed = 0UZ;
    while (consumed < records.size()) {
        shim::InputSpan<gr::DataSet<CF>> inSpan(records.subspan(consumed), consumed);
        shim::OutputSpan<CF>             outSpan(std::span<CF>(scratch.data(), outRoom), result.samples.size(), &result.tags);
        std::ignore = block.processBulk(inSpan, outSpan);
        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ) {
            break;
        }
        consumed += inSpan.consumed;
    }
    return result;
}

/// @brief Drives `CpRemove` over samples @p begin to @p end of @p input, @p chunk samples a call, with each tag at its
/// absolute index.
[[nodiscard]] std::vector<gr::DataSet<CF>> remove(CpRemove& block, const Stream& input, std::size_t begin, std::size_t end, std::size_t chunk, std::size_t outRoom = 64UZ) {
    std::vector<gr::DataSet<CF>> records;
    std::vector<gr::DataSet<CF>> scratch(outRoom);

    std::size_t consumed = begin;
    std::size_t fed      = begin;
    while (consumed < end) {
        fed              = std::min(end, fed + chunk);
        const auto first = std::ranges::lower_bound(input.tags, consumed, std::ranges::less{}, &gr::Tag::index);
        const auto last  = std::ranges::lower_bound(input.tags, fed, std::ranges::less{}, &gr::Tag::index);

        shim::InputSpan<CF>               inSpan(std::span<const CF>(input.samples).subspan(consumed, fed - consumed), consumed, std::span<const gr::Tag>(first, last));
        shim::OutputSpan<gr::DataSet<CF>> outSpan(std::span<gr::DataSet<CF>>(scratch.data(), outRoom));
        std::ignore = block.processBulk(inSpan, outSpan);
        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            records.push_back(std::move(scratch[k]));
        }
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && fed == end) {
            break;
        }
        consumed += inSpan.consumed;
    }
    return records;
}

[[nodiscard]] std::vector<gr::DataSet<CF>> remove(CpRemove& block, const Stream& input, std::size_t chunk, std::size_t outRoom = 64UZ) { return remove(block, input, 0UZ, input.samples.size(), chunk, outRoom); }

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
    return block;
}

/// @brief Applies @p settings between two calls.
template<typename TBlock>
void change(TBlock& block, gr::property_map settings) {
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

[[nodiscard]] std::string kindOf(const gr::DataSet<CF>& record) {
    const auto  entry = record.meta_information[0UZ].find(std::pmr::string("symbol_kind"));
    const auto* value = entry == record.meta_information[0UZ].end() ? nullptr : entry->second.get_if<std::pmr::string>();
    return value == nullptr ? std::string{} : std::string(value->begin(), value->end());
}

/// A stream of @p nFrames frames of @p perFrame symbols each, from `CpInsert` with a prefix of 16.
[[nodiscard]] Stream frames(std::size_t nFrames, std::size_t perFrame) {
    std::vector<gr::DataSet<CF>> records;
    for (std::size_t s = 0UZ; s < nFrames * perFrame; ++s) {
        records.push_back(symbol(s, s % perFrame));
    }
    CpInsert block = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
    return insert(block, std::span<const gr::DataSet<CF>>(records), 4096UZ);
}

/// @brief Runs a source, a `CarrierAllocator`, `CpInsert` and a sink under the framework's scheduler. Returns the
/// samples the sink received.
[[nodiscard]] std::size_t runToSink(std::size_t nInput, std::size_t sampleEdge) {
    using gr::blocks::testing::ProcessFunction;
    std::vector<std::int32_t> carriers;
    for (std::int32_t carrier = -26; carrier <= 26; ++carrier) {
        if (carrier != 0) {
            carriers.push_back(carrier);
        }
    }

    gr::Graph graph;
    auto&     source   = graph.emplaceBlock<gr::blocks::testing::TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(nInput)}, {"mark_tag", false}});
    auto&     allocate = graph.emplaceBlock<gr::blocks::ofdm::CarrierAllocator>({{"fft_len", gr::Size_t{kFft}}, {"data_carriers", carriers}});
    auto&     insert   = graph.emplaceBlock<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
    auto&     sink     = graph.emplaceBlock<gr::blocks::testing::TagSink<CF, ProcessFunction::USE_PROCESS_BULK>>({{"log_samples", false}, {"log_tags", false}});
    boost::ut::expect(graph.connect<"out", "in">(source, allocate).has_value());
    boost::ut::expect(graph.connect<"out", "in">(allocate, insert).has_value());
    boost::ut::expect(graph.connect<"out", "in">(insert, sink, gr::EdgeParameters{.minBufferSize = sampleEdge}).has_value());

    gr::scheduler::Simple<> scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value());
    return static_cast<std::size_t>(sink._nSamplesProduced);
}

/// The stream a prefix cycle and a list of symbols must produce, built without the block.
[[nodiscard]] std::vector<CF> expectedStream(std::span<const gr::DataSet<CF>> records, std::span<const gr::Size_t> cycle) {
    std::vector<CF> stream;
    for (std::size_t s = 0UZ; s < records.size(); ++s) {
        const std::vector<CF> time   = inverseTransform(records[s].signal_values);
        const std::size_t     prefix = static_cast<std::size_t>(cycle[s % cycle.size()]);
        stream.insert(stream.end(), time.end() - static_cast<std::ptrdiff_t>(prefix), time.end());
        stream.insert(stream.end(), time.begin(), time.end());
    }
    return stream;
}

const boost::ut::suite<"OFDM cyclic prefix"> _cyclicPrefix = [] {
    using namespace boost::ut;

    "the prefix is the symbol's own tail, for a scalar and for a cycle"_test = [] {
        for (const std::vector<gr::Size_t>& cycle : {std::vector<gr::Size_t>{16U}, std::vector<gr::Size_t>{16U, 4U, 8U}}) {
            std::vector<gr::DataSet<CF>> records;
            for (std::size_t s = 0UZ; s < 6UZ; ++s) {
                records.push_back(symbol(s, s));
            }
            CpInsert     block  = make<CpInsert>({{"cp_len", cycle}});
            const Stream stream = insert(block, std::span<const gr::DataSet<CF>>(records), 4096UZ);

            const std::vector<CF> expected = expectedStream(std::span<const gr::DataSet<CF>>(records), std::span<const gr::Size_t>(cycle));
            expect(eq(stream.samples.size(), expected.size()));
            expect(std::ranges::equal(stream.samples, expected)) << "the emitted stream is bit-identical to the prefix algebra";
            expect(eq(stream.tags.size(), 1UZ)) << "one frame start, at symbol_in_frame 0";
            expect(eq(stream.tags[0UZ].index, 0UZ));
        }
    };

    "the prefix cycle restarts at each frame"_test = [] {
        const std::vector<gr::Size_t> cycle{16U, 4U};
        std::vector<gr::DataSet<CF>>  records;
        for (std::size_t frame = 0UZ; frame < 2UZ; ++frame) {
            for (std::size_t s = 0UZ; s < 3UZ; ++s) {
                records.push_back(symbol(3UZ * frame + s, s)); // symbol_in_frame runs 0,1,2 in both frames
            }
        }
        CpInsert     block  = make<CpInsert>({{"cp_len", cycle}});
        const Stream stream = insert(block, std::span<const gr::DataSet<CF>>(records), 4096UZ);

        // 16, 4, 16 in both frames. The cycle follows symbol_in_frame, not the record count.
        const std::size_t expectedLength = 6UZ * kFft + 2UZ * (16UZ + 4UZ + 16UZ);
        expect(eq(stream.samples.size(), expectedLength));
        expect(eq(stream.tags.size(), 2UZ));
        expect(eq(stream.tags[1UZ].index, 3UZ * kFft + 16UZ + 4UZ + 16UZ));
    };

    "CpInsert into CpRemove is exact identity on the kept samples, at any chunking"_test = [] {
        for (const std::vector<gr::Size_t>& cycle : {std::vector<gr::Size_t>{16U}, std::vector<gr::Size_t>{16U, 4U, 8U}}) {
            std::vector<gr::DataSet<CF>> records;
            for (std::size_t s = 0UZ; s < 3UZ; ++s) {
                records.push_back(symbol(s, s));
            }
            CpInsert     insertBlock = make<CpInsert>({{"cp_len", cycle}});
            const Stream stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records), 4096UZ);

            // Each cut holds the samples the cadence names, read from the stream itself.
            std::vector<std::vector<CF>> expected;
            std::size_t                  at = 0UZ;
            for (std::size_t s = 0UZ; s < records.size(); ++s) {
                const std::size_t prefix = static_cast<std::size_t>(cycle[s % cycle.size()]);
                expected.push_back(std::vector<CF>(stream.samples.begin() + static_cast<std::ptrdiff_t>(at + prefix), stream.samples.begin() + static_cast<std::ptrdiff_t>(at + prefix + kFft)));
                at += prefix + kFft;
            }

            for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 80UZ, 4096UZ}) {
                CpRemove   removeBlock = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", cycle}, {"frame_len", gr::Size_t{3U}}});
                const auto out         = remove(removeBlock, stream, chunk);
                expect(eq(out.size(), records.size())) << std::format("chunk {}", chunk);
                for (std::size_t s = 0UZ; s < out.size(); ++s) {
                    const std::vector<CF> reference = forwardTransform(std::span<const CF>(expected[s]));
                    expect(std::ranges::equal(out[s].signal_values, reference)) << std::format("chunk {} symbol {}", chunk, s);
                }
            }
        }
    };

    "the round trip returns the symbol it started from"_test = [] {
        std::vector<gr::DataSet<CF>> records;
        for (std::size_t s = 0UZ; s < 3UZ; ++s) {
            records.push_back(symbol(s, s));
        }
        CpInsert     insertBlock = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        const Stream stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records), 4096UZ);
        CpRemove     removeBlock = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{3U}}});
        const auto   out         = remove(removeBlock, stream, 37UZ);

        expect(eq(out.size(), 3UZ));
        double worst = 0.;
        for (std::size_t s = 0UZ; s < out.size(); ++s) {
            for (std::size_t k = 0UZ; k < kFft; ++k) {
                worst = std::max(worst, static_cast<double>(std::abs(out[s].signal_values[k] - records[s].signal_values[k])));
            }
        }
        std::println("round trip through the inverse and forward transforms: worst bin error {:.3e}", worst);
        expect(lt(worst, 1e-3)) << "the scaled backward transform is the forward one's inverse";
    };

    "a negative timing_offset rotates carrier k by 2*pi*k*offset/fft_len"_test = [] {
        constexpr std::int32_t       kOffset = -4;
        std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ)};

        CpInsert     insertBlock = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        const Stream stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records), 4096UZ);
        CpRemove     removeBlock = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{1U}}, {"timing_offset", kOffset}});
        const auto   out         = remove(removeBlock, stream, 11UZ);

        expect(eq(out.size(), 1UZ));
        double worst = 0.;
        for (std::size_t bin = 0UZ; bin < kFft; ++bin) {
            const int    carrier = gr::ofdm::CarrierMap::carrierOf(kFft, bin).value();
            const double phase   = 2. * std::numbers::pi * static_cast<double>(carrier) * static_cast<double>(kOffset) / static_cast<double>(kFft);
            const CF     want    = records[0UZ].signal_values[bin] * CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
            worst                = std::max(worst, static_cast<double>(std::abs(out[0UZ].signal_values[bin] - want)));
        }
        std::println("timing_offset {} against the stated slope 2*pi*k*offset/fft_len: worst bin error {:.3e}", kOffset, worst);
        expect(lt(worst, 1e-3)) << "the bias into the prefix is a pure per-carrier rotation";
    };

    "between frames the block discards, and counts what it discarded"_test = [] {
        std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ), symbol(1UZ, 1UZ)};
        CpInsert                     insertBlock = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        Stream                       stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records), 4096UZ);

        // silence before the frame and after it, outside the cadence
        Stream padded;
        padded.samples.assign(25UZ, CF{});
        padded.samples.insert(padded.samples.end(), stream.samples.begin(), stream.samples.end());
        padded.samples.insert(padded.samples.end(), 40UZ, CF{});
        padded.tags.push_back(gr::Tag{25UZ, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("ofdm_frame")}}});

        CpRemove   block = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{2U}}});
        const auto out   = remove(block, padded, 23UZ);

        expect(eq(out.size(), 2UZ));
        expect(eq(block.nDiscarded(), std::uint64_t{65})) << "25 samples before the trigger and 40 after the frame";
        expect(eq(gr::blocks::ofdm::detail::metaCount(out[1UZ], "discarded_samples", 0ULL), std::uint64_t{25})) << "the record states the samples discarded before it was cut";
    };

    "a trigger label the block does not carry starts no frame"_test = [] {
        std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ)};
        CpInsert                     insertBlock = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}, {"trigger_label", std::string("other")}});
        const Stream                 stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records), 4096UZ);

        CpRemove labeled = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{1U}}, {"trigger_label", std::string("ofdm_frame")}});
        expect(eq(remove(labeled, stream, 100UZ).size(), 0UZ));
        expect(eq(labeled.nDiscarded(), stream.samples.size()));

        CpRemove any = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{1U}}, {"trigger_label", std::string("")}});
        expect(eq(remove(any, stream, 100UZ).size(), 1UZ)) << "an empty trigger_label accepts any trigger";
    };

    /**
     * Two frames of four symbols, then 400 samples of silence. Triggers of another label lie in the first frame's
     * second cut and in the silence. At the default trigger_label they open no frame, and the records match the run
     * without them. An empty trigger_label takes both, and each opens a frame whose first cut starts cp_len after it.
     */
    "a trigger of another label opens no frame at the default trigger_label"_test = [] {
        constexpr std::size_t kSymbol = 16UZ + kFft;
        Stream                clean   = frames(2UZ, 4UZ);
        clean.samples.insert(clean.samples.end(), 400UZ, CF{});
        expect(eq(clean.tags.size(), 2UZ)) << "one trigger per frame from CpInsert";

        const std::vector<std::size_t> others{kSymbol + 30UZ, 8UZ * kSymbol + 50UZ};
        Stream                         tagged = clean;
        for (const std::size_t at : others) {
            tagged.tags.push_back(gr::Tag{at, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("wallclock")}, {gr::tag::TRIGGER_TIME.shortKey(), std::uint64_t{1}}, {gr::tag::TRIGGER_OFFSET.shortKey(), 0.f}}});
        }
        std::ranges::stable_sort(tagged.tags, std::ranges::less{}, &gr::Tag::index);

        const auto starts = [](const std::vector<gr::DataSet<CF>>& records) {
            std::vector<std::uint64_t> list;
            for (const gr::DataSet<CF>& record : records) {
                list.push_back(gr::blocks::ofdm::detail::metaCount(record, "sample_start", std::numeric_limits<std::uint64_t>::max()));
            }
            return list;
        };
        const gr::property_map framing{{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"n_sync", gr::Size_t{1U}}, {"frame_len", gr::Size_t{3U}}};
        for (const std::size_t chunk : {7UZ, 4096UZ}) {
            CpRemove   reference = make<CpRemove>(framing);
            const auto expected  = remove(reference, clean, chunk);
            expect(eq(expected.size(), 8UZ)) << std::format("chunk {}: two frames of four symbols", chunk);

            CpRemove   block = make<CpRemove>(framing);
            const auto out   = remove(block, tagged, chunk);
            expect(starts(out) == starts(expected)) << std::format("chunk {}: every record on the grid, {} against {}", chunk, starts(out), starts(expected));
            expect(eq(block.nDiscarded(), reference.nDiscarded())) << std::format("chunk {}", chunk);
            for (std::size_t k = 0UZ; k < std::min(out.size(), expected.size()); ++k) {
                expect(std::ranges::equal(out[k].signal_values, expected[k].signal_values)) << std::format("chunk {} record {}", chunk, k);
            }

            gr::property_map anyLabel = framing;
            anyLabel["trigger_label"] = std::string("");
            CpRemove   any            = make<CpRemove>(anyLabel);
            const auto taken          = starts(remove(any, tagged, chunk));
            for (const std::size_t at : others) {
                expect(std::ranges::find(taken, at + 16UZ) != taken.end()) << std::format("chunk {}: an empty trigger_label opens a frame at {}", chunk, at);
            }
        }
    };

    "edge windowing lowers the out-of-band power, measured"_test = [] {
        // 24 occupied carriers of 64. The guard band is wide, and the shoulder shows well away from the band edge. A
        // Hann window on the analysis keeps its own leakage below the signal's.
        constexpr std::size_t        kOccupiedHalf = 12UZ;
        constexpr std::size_t        kSymbols      = 48UZ;
        std::vector<gr::DataSet<CF>> records;
        std::vector<int>             carriers(kFft);
        for (std::size_t bin = 0UZ; bin < kFft; ++bin) {
            carriers[bin] = gr::ofdm::CarrierMap::carrierOf(kFft, bin).value();
        }
        for (std::size_t s = 0UZ; s < kSymbols; ++s) {
            std::vector<CF> values(kFft, CF{});
            for (std::size_t k = 1UZ; k <= kOccupiedHalf; ++k) {
                const float sign = ((s + k) % 2UZ) == 0UZ ? 1.f : -1.f;
                values[k]        = CF(sign, -sign);
                values[kFft - k] = CF(-sign, sign);
            }
            records.push_back(gr::blocks::ofdm::detail::makeSymbolRecord(std::move(values), std::span<const int>(carriers), "data", "test", //
                gr::property_map{{std::pmr::string("symbol_in_frame"), gr::pmt::Value(static_cast<std::uint64_t>(s))}}));
        }

        constexpr std::size_t kAnalysis = 2048UZ;
        std::vector<float>    hann(kAnalysis);
        for (std::size_t n = 0UZ; n < kAnalysis; ++n) {
            hann[n] = static_cast<float>(0.5 * (1.0 - std::cos(2. * std::numbers::pi * static_cast<double>(n) / static_cast<double>(kAnalysis))));
        }

        const auto shoulderDb = [&records, &hann](gr::Size_t edge) {
            CpInsert     block  = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}, {"window_len", edge}});
            const Stream stream = insert(block, std::span<const gr::DataSet<CF>>(records), 16384UZ);

            std::vector<CF> segment(kAnalysis);
            for (std::size_t n = 0UZ; n < kAnalysis; ++n) {
                segment[n] = stream.samples[256UZ + n] * hann[n];
            }
            const std::vector<CF> spectrum = forwardTransform(std::span<const CF>(segment));

            // The occupied band is |carrier| <= 12 of 64. Everything past |carrier| 20 is the shoulder. The carriers
            // between are left out, and the transition counts as neither.
            constexpr std::size_t inEdge  = kOccupiedHalf * kAnalysis / kFft;
            constexpr std::size_t outEdge = 20UZ * kAnalysis / kFft;
            double                inBand  = 0.;
            double                out     = 0.;
            for (std::size_t bin = 0UZ; bin < kAnalysis; ++bin) {
                const std::size_t distance = std::min(bin, kAnalysis - bin);
                const double      power    = static_cast<double>(std::norm(spectrum[bin]));
                if (distance <= inEdge) {
                    inBand += power;
                } else if (distance >= outEdge) {
                    out += power;
                }
            }
            return 10. * std::log10(out / inBand);
        };

        const double plain = shoulderDb(0U);
        for (const gr::Size_t edge : {gr::Size_t{4U}, gr::Size_t{8U}, gr::Size_t{16U}}) {
            const double windowed = shoulderDb(edge);
            std::println("out-of-band power relative to in-band: {:.2f} dB unwindowed, {:.2f} dB with a {}-sample raised-cosine edge, a {:.2f} dB improvement", plain, windowed, edge, plain - windowed);
            expect(lt(windowed, plain)) << "a raised-cosine edge lowers the shoulder";
        }
    };

    "a prefix or an offset the algebra cannot hold is refused by name"_test = [] {
        const auto names = [](std::string_view setting, auto&& function) {
            const std::string message = refusal(function);
            expect(message.contains(setting)) << std::format("the refusal names {}: '{}'", setting, message);
        };
        names("timing_offset", [] { std::ignore = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"timing_offset", std::int32_t{-17}}}); });
        const std::string limits = refusal([] { std::ignore = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"timing_offset", std::int32_t{1}}}); });
        expect(limits.contains("timing_offset") && limits.contains("outside the limits")) << std::format("the limits of timing_offset refuse 1: '{}'", limits);
        names("cp_len", [] { std::ignore = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{}}}); });
        names("cp_len", [] { std::ignore = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{80U}}}); });
        names("window_len", [] { std::ignore = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{4U}}, {"window_len", gr::Size_t{8U}}}); });
    };

    "a record shorter than its prefix is dropped and counted"_test = [] {
        const std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ), symbol(1UZ, 1UZ, 8UZ), symbol(2UZ, 1UZ)};
        CpInsert                           block  = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        const Stream                       stream = insert(block, std::span<const gr::DataSet<CF>>(records), 4096UZ);

        const std::vector<gr::DataSet<CF>> kept{records[0UZ], records[2UZ]};
        const std::vector<gr::Size_t>      cycle{16U};
        expect(std::ranges::equal(stream.samples, expectedStream(std::span<const gr::DataSet<CF>>(kept), std::span<const gr::Size_t>(cycle)))) << "the stream holds the two whole symbols";
        expect(eq(block.nMalformed(), std::uint64_t{1}));
        expect(eq(block.nUnmarked(), std::uint64_t{0}));
    };

    "a record without symbol_in_frame takes the next cycle position and is counted"_test = [] {
        const std::vector<gr::Size_t>      cycle{16U, 4U, 8U};
        const std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ), symbol(1UZ, std::nullopt), symbol(2UZ, 2UZ)};
        CpInsert                           block  = make<CpInsert>({{"cp_len", cycle}});
        const Stream                       stream = insert(block, std::span<const gr::DataSet<CF>>(records), 4096UZ);

        expect(std::ranges::equal(stream.samples, expectedStream(std::span<const gr::DataSet<CF>>(records), std::span<const gr::Size_t>(cycle)))) << "the unmarked record takes prefix 4, the second entry";
        expect(eq(block.nUnmarked(), std::uint64_t{1}));
        expect(eq(block.nMalformed(), std::uint64_t{0}));
    };

    /**
     * The second record carries a `symbol_in_frame` of 2^32, a value wider than 32 bits. 2^32 is 1 modulo 3, and the
     * record takes the second prefix of the cycle. The third record carries none and takes the third. Neither starts a
     * frame.
     */
    "a symbol_in_frame wider than 32 bits sets the cycle position whole and starts no frame"_test = [] {
        constexpr std::uint64_t kWide = std::uint64_t{1} << 32U;
        static_assert(kWide % 3U == 1U, "expectedStream() takes the prefix by record index, and the marked record is index 1");
        const std::vector<gr::Size_t>      cycle{16U, 4U, 8U};
        const std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ), symbol(1UZ, kWide), symbol(2UZ, std::nullopt)};
        CpInsert                           block  = make<CpInsert>({{"cp_len", cycle}});
        const Stream                       stream = insert(block, std::span<const gr::DataSet<CF>>(records), 4096UZ);

        expect(std::ranges::equal(stream.samples, expectedStream(std::span<const gr::DataSet<CF>>(records), std::span<const gr::Size_t>(cycle)))) << "the prefixes 16, 4 and 8";
        expect(eq(stream.tags.size(), 1UZ)) << "one frame start, at the record of symbol_in_frame 0";
    };

    "CpInsert publishes the last record whole through the framework's end of stream, whatever room its output has"_test = [] {
        // 52 data carriers make one symbol of 64 + 16 samples. The sink's edge of 100 samples fills before the last
        // symbol has gone out at some stream lengths, and the block then publishes the rest on later calls.
        constexpr std::size_t kSymbolSamples = kFft + 16UZ;
        std::size_t           short_         = 0UZ;
        for (std::size_t nSymbols = 1UZ; nSymbols <= 40UZ; ++nSymbols) {
            const std::size_t received = runToSink(52UZ * nSymbols, 100UZ);
            if (received != nSymbols * kSymbolSamples) {
                ++short_;
                std::println("{} symbols: the sink received {} of {} samples", nSymbols, received, nSymbols * kSymbolSamples);
            }
        }
        expect(eq(short_, 0UZ)) << "every stream length reaches the sink whole";
    };

    "a trigger on the cut's first sample opens its frame once, whatever room the output has"_test = [] {
        // Two frames of one symbol each. With timing_offset at -cp_len the cut starts on the trigger's own sample.
        std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ), symbol(1UZ, 0UZ)};
        CpInsert                     insertBlock = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        const Stream                 stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records), 4096UZ);
        expect(eq(stream.tags.size(), 2UZ));

        for (const std::size_t outRoom : {1UZ, 64UZ}) {
            CpRemove   block = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{1U}}, {"timing_offset", std::int32_t{-16}}});
            const auto out   = remove(block, stream, 4096UZ, outRoom);
            expect(eq(out.size(), 2UZ)) << std::format("output room {}", outRoom);
            for (std::size_t r = 0UZ; r < out.size(); ++r) {
                expect(eq(gr::blocks::ofdm::detail::metaCount(out[r], "frame_index", 99ULL), std::uint64_t{r})) << std::format("output room {} record {}", outRoom, r);
            }
        }
    };

    "a cp_len change keeps the windowed edge"_test = [] {
        std::vector<gr::DataSet<CF>> records;
        for (std::size_t s = 0UZ; s < 4UZ; ++s) {
            records.push_back(symbol(s, s));
        }
        // The cycle 16, 16, 12, 12 gives the four symbols the prefixes the change gives them.
        CpInsert     reference = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U, 16U, 12U, 12U}}, {"window_len", gr::Size_t{4U}}});
        const Stream expected  = insert(reference, std::span<const gr::DataSet<CF>>(records), 4096UZ);

        CpInsert block  = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}, {"window_len", gr::Size_t{4U}}});
        Stream   stream = insert(block, std::span<const gr::DataSet<CF>>(records).first(2UZ), 4096UZ);
        change(block, {{"cp_len", std::vector<gr::Size_t>{12U}}});
        const Stream rest = insert(block, std::span<const gr::DataSet<CF>>(records).subspan(2UZ), 4096UZ);
        stream.samples.insert(stream.samples.end(), rest.samples.begin(), rest.samples.end());
        expect(eq(stream.samples.size(), expected.samples.size()));
        expect(std::ranges::equal(stream.samples, expected.samples)) << "the third symbol's head still adds the second symbol's extension";
    };

    "a cut setting the block already runs on keeps the partial cut, and a signal_name change reaches the next record"_test = [] {
        std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ), symbol(1UZ, 1UZ)};
        CpInsert                     insertBlock = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        const Stream                 stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records), 4096UZ);
        const gr::property_map       settings{{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{2U}}};

        CpRemove   reference = make<CpRemove>(settings);
        const auto whole     = remove(reference, stream, 4096UZ);
        expect(eq(whole.size(), 2UZ));

        for (const bool renamed : {false, true}) {
            // 100 samples hold the first symbol and the second symbol's prefix, with 4 samples of its cut
            CpRemove block = make<CpRemove>(settings);
            auto     out   = remove(block, stream, 0UZ, 100UZ, 4096UZ);
            if (renamed) {
                change(block, {{"signal_name", std::string("renamed")}});
            } else {
                // The refusal leaves -20 in the member. The second change then reaches the block with the offset it
                // already runs on.
                expect(refusal([&block] { change(block, {{"timing_offset", std::int32_t{-20}}}); }).contains("timing_offset"));
                change(block, {{"timing_offset", std::int32_t{0}}});
            }
            const auto rest = remove(block, stream, 100UZ, stream.samples.size(), 4096UZ);
            out.insert(out.end(), rest.begin(), rest.end());

            expect(eq(out.size(), 2UZ)) << std::format("renamed {}", renamed);
            expect(eq(block.nDiscarded(), std::uint64_t{0})) << std::format("renamed {}", renamed);
            if (out.size() == 2UZ && whole.size() == 2UZ) {
                expect(std::ranges::equal(out[1UZ].signal_values, whole[1UZ].signal_values)) << "the cut in hand is complete and unchanged";
                expect(eq(signalName(out[0UZ]), std::string("ofdm_symbol")));
                expect(eq(signalName(out[1UZ]), std::string(renamed ? "renamed" : "ofdm_symbol")));
            }
        }
    };

    "a refused setting leaves the block on the settings it last accepted"_test = [] {
        std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ), symbol(1UZ, 1UZ)};

        CpInsert     reference = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        const Stream expected  = insert(reference, std::span<const gr::DataSet<CF>>(records), 4096UZ);

        CpInsert insertBlock = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        Stream   stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records).first(1UZ), 4096UZ);
        expect(refusal([&insertBlock] { change(insertBlock, {{"cp_len", std::vector<gr::Size_t>{}}}); }).contains("cp_len"));
        const Stream second = insert(insertBlock, std::span<const gr::DataSet<CF>>(records).subspan(1UZ), 4096UZ);
        stream.samples.insert(stream.samples.end(), second.samples.begin(), second.samples.end());
        expect(std::ranges::equal(stream.samples, expected.samples)) << "CpInsert keeps a prefix of 16";

        CpRemove removeBlock = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{2U}}});
        expect(refusal([&removeBlock] { change(removeBlock, {{"timing_offset", std::int32_t{-20}}}); }).contains("timing_offset"));
        const auto out = remove(removeBlock, expected, 37UZ);
        expect(eq(out.size(), 2UZ)) << "CpRemove keeps a timing_offset of 0";
        if (out.size() == 2UZ) {
            const std::vector<CF> cut(expected.samples.begin() + 16, expected.samples.begin() + 16 + static_cast<std::ptrdiff_t>(kFft));
            expect(std::ranges::equal(out[0UZ].signal_values, forwardTransform(std::span<const CF>(cut))));
        }
    };

    /**
     * The block applies none of the values of a settings application that throws. It keeps its trigger and framing
     * settings, which no check refuses, when they arrive beside a refused value.
     */
    "a refused application leaves every setting the block reads as it was"_test = [] {
        const std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ), symbol(1UZ, 1UZ), symbol(2UZ, 0UZ), symbol(3UZ, 1UZ)};
        const auto                         tagList = [](const Stream& stream) {
            std::vector<std::string> list;
            for (const gr::Tag& tag : stream.tags) {
                const auto entry = tag.map.find(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey()));
                list.push_back(std::format("{}@{}", entry == tag.map.end() ? std::string{} : entry->second.value_or(std::string{}), tag.index));
            }
            return list;
        };

        CpInsert     insertReference = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        const Stream expected        = insert(insertReference, std::span<const gr::DataSet<CF>>(records), 4096UZ);
        expect(eq(expected.tags.size(), 2UZ)) << "one trigger per frame";

        CpInsert insertBlock = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        Stream   stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records).first(2UZ), 4096UZ);
        expect(refusal([&insertBlock] { change(insertBlock, {{"emit_trigger", false}, {"trigger_label", std::string("other")}, {"window_len", gr::Size_t{99U}}}); }).contains("window_len"));
        const Stream rest = insert(insertBlock, std::span<const gr::DataSet<CF>>(records).subspan(2UZ), 4096UZ);
        for (gr::Tag tag : rest.tags) { // the second call counts its output from zero
            tag.index += stream.samples.size();
            stream.tags.push_back(std::move(tag));
        }
        stream.samples.insert(stream.samples.end(), rest.samples.begin(), rest.samples.end());
        expect(std::ranges::equal(stream.samples, expected.samples)) << "CpInsert keeps its prefix";
        expect(tagList(stream) == tagList(expected)) << std::format("CpInsert keeps emit_trigger and trigger_label: {} against {}", tagList(stream), tagList(expected));

        const gr::property_map framing{{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{2U}}, {"trigger_label", std::string("ofdm_frame")}};
        CpRemove               removeReference = make<CpRemove>(framing);
        const auto             whole           = remove(removeReference, expected, 4096UZ);
        expect(eq(whole.size(), 4UZ)) << "two frames of two symbols";

        // 100 samples hold the first frame's first symbol and part of its second
        CpRemove removeBlock = make<CpRemove>(framing);
        auto     out         = remove(removeBlock, expected, 0UZ, 100UZ, 4096UZ);
        expect(refusal([&removeBlock] { change(removeBlock, {{"n_sync", gr::Size_t{1U}}, {"frame_len", gr::Size_t{1U}}, {"trigger_label", std::string("other")}, {"timing_offset", std::int32_t{-20}}}); }).contains("timing_offset"));
        const auto after = remove(removeBlock, expected, 100UZ, expected.samples.size(), 4096UZ);
        out.insert(out.end(), after.begin(), after.end());
        expect(eq(out.size(), whole.size())) << "CpRemove keeps frame_len and trigger_label";
        for (std::size_t k = 0UZ; k < std::min(out.size(), whole.size()); ++k) {
            expect(std::ranges::equal(out[k].signal_values, whole[k].signal_values)) << std::format("record {}", k);
            expect(out[k].meta_information[0UZ] == whole[k].meta_information[0UZ]) << std::format("record {}: CpRemove keeps n_sync", k);
        }
    };

    /**
     * Each arm refuses a `CpInsert` or `CpRemove` application before a valid change of another key. The output equals
     * that of the valid change alone.
     */
    "a valid change after a refused application leaves the refused values out"_test = [] {
        const std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ), symbol(1UZ, 1UZ), symbol(2UZ, 0UZ), symbol(3UZ, 1UZ)};
        const auto                         tagList = [](const std::vector<gr::Tag>& tags) {
            std::vector<std::string> list;
            for (const gr::Tag& tag : tags) {
                const auto entry = tag.map.find(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey()));
                list.push_back(std::format("{}@{}", entry == tag.map.end() ? std::string{} : entry->second.value_or(std::string{}), tag.index));
            }
            return list;
        };
        struct InsertArm {
            std::string_view name;
            gr::property_map refused;
            gr::property_map valid;
        };
        const std::vector<InsertArm> insertArms{
            {"CpInsert: window_len 20 with emit_trigger and trigger_label, then cp_len 24", {{"emit_trigger", false}, {"trigger_label", std::string("other")}, {"window_len", gr::Size_t{20U}}}, {{"cp_len", std::vector<gr::Size_t>{24U}}}},
            {"CpInsert: an empty cp_len with emit_trigger, then trigger_label", {{"cp_len", std::vector<gr::Size_t>{}}, {"emit_trigger", false}}, {{"trigger_label", std::string("x")}}},
        };
        for (const InsertArm& arm : insertArms) {
            const auto run = [&records, &arm](bool refuseFirst) {
                CpInsert block  = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
                Stream   stream = insert(block, std::span<const gr::DataSet<CF>>(records).first(2UZ), 4096UZ);
                if (refuseFirst) {
                    expect(!refusal([&block, &arm] { change(block, arm.refused); }).empty()) << std::format("{}: the first application is refused", arm.name);
                }
                expect(refusal([&block, &arm] { change(block, arm.valid); }).empty()) << std::format("{}: the valid change applies", arm.name);
                const Stream rest = insert(block, std::span<const gr::DataSet<CF>>(records).subspan(2UZ), 4096UZ);
                for (gr::Tag tag : rest.tags) { // the second call counts its output from zero
                    tag.index += stream.samples.size();
                    stream.tags.push_back(std::move(tag));
                }
                stream.samples.insert(stream.samples.end(), rest.samples.begin(), rest.samples.end());
                return stream;
            };
            const Stream expected = run(false);
            const Stream got      = run(true);
            expect(std::ranges::equal(got.samples, expected.samples)) << std::format("{}: the samples", arm.name);
            expect(tagList(got.tags) == tagList(expected.tags)) << std::format("{}: {} against {}", arm.name, tagList(got.tags), tagList(expected.tags));
        }

        CpInsert     source = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        const Stream stream = insert(source, std::span<const gr::DataSet<CF>>(records), 4096UZ);
        struct RemoveArm {
            std::string_view name;
            gr::property_map refused;
            gr::property_map valid;
        };
        const std::vector<RemoveArm> removeArms{
            {"CpRemove: timing_offset -20 with n_sync and trigger_label, then signal_name", {{"n_sync", gr::Size_t{1U}}, {"trigger_label", std::string("other")}, {"timing_offset", std::int32_t{-20}}}, {{"signal_name", std::string("renamed")}}},
            {"CpRemove: timing_offset -20, then cp_len 32", {{"timing_offset", std::int32_t{-20}}}, {{"cp_len", std::vector<gr::Size_t>{32U}}}},
        };
        const gr::property_map framing{{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{2U}}};
        for (const RemoveArm& arm : removeArms) {
            const auto run = [&stream, &framing, &arm](bool refuseFirst) {
                // The split at sample 100 falls 4 samples into the second symbol's cut.
                CpRemove block = make<CpRemove>(framing);
                auto     out   = remove(block, stream, 0UZ, 100UZ, 4096UZ);
                if (refuseFirst) {
                    expect(refusal([&block, &arm] { change(block, arm.refused); }).contains("timing_offset")) << std::format("{}: the first application is refused", arm.name);
                    const std::optional<gr::pmt::Value> reported = block.settings().get("timing_offset");
                    expect(reported.has_value() && reported->get_if<std::int32_t>() != nullptr && *reported->get_if<std::int32_t>() == -20) << "settings().get() returns the refused value";
                    expect(refusal([&block] { change(block, {{"timing_offset", std::int32_t{-20}}}); }).empty()) << "the refused value sent again alone passes";
                }
                expect(refusal([&block, &arm] { change(block, arm.valid); }).empty()) << std::format("{}: the valid change applies", arm.name);
                const auto rest = remove(block, stream, 100UZ, stream.samples.size(), 4096UZ);
                out.insert(out.end(), rest.begin(), rest.end());
                return std::pair{out, block.nDiscarded()};
            };
            const auto [expected, expectedDiscarded] = run(false);
            const auto [got, gotDiscarded]           = run(true);
            expect(gt(expected.size(), 0UZ)) << arm.name;
            expect(eq(got.size(), expected.size())) << std::format("{}: the records", arm.name);
            expect(eq(gotDiscarded, expectedDiscarded)) << std::format("{}: the discarded samples", arm.name);
            for (std::size_t k = 0UZ; k < std::min(got.size(), expected.size()); ++k) {
                expect(std::ranges::equal(got[k].signal_values, expected[k].signal_values)) << std::format("{}: record {}", arm.name, k);
                expect(got[k].meta_information[0UZ] == expected[k].meta_information[0UZ]) << std::format("{}: record {} meta", arm.name, k);
                expect(eq(signalName(got[k]), signalName(expected[k]))) << std::format("{}: record {} signal name", arm.name, k);
            }
        }
    };

    "an fft_len change during a cut ends the frame, and the partial cut counts as discarded"_test = [] {
        // The cut of 64 holds 50 samples when fft_len becomes 32.
        Stream input;
        input.samples.assign(214UZ, CF{1.f, 0.f});
        input.tags.push_back(gr::Tag{0UZ, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("ofdm_frame")}}});
        input.tags.push_back(gr::Tag{166UZ, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("ofdm_frame")}}});

        CpRemove block = make<CpRemove>({{"fft_len", gr::Size_t{64U}}, {"cp_len", std::vector<gr::Size_t>{16U}}});
        expect(eq(remove(block, input, 0UZ, 66UZ, 4096UZ).size(), 0UZ));
        change(block, {{"fft_len", gr::Size_t{32U}}});

        expect(eq(remove(block, input, 66UZ, 166UZ, 4096UZ).size(), 0UZ)) << "no frame is open until the next trigger";
        expect(eq(block.nDiscarded(), std::uint64_t{150})) << "the 50 samples of the partial cut and the 100 after it";

        const auto out = remove(block, input, 166UZ, input.samples.size(), 4096UZ);
        expect(eq(out.size(), 1UZ)) << "the next trigger opens a frame cut at the new length";
        if (!out.empty()) {
            expect(eq(out[0UZ].signal_values.size(), 32UZ));
            expect(eq(gr::blocks::ofdm::detail::metaCount(out[0UZ], "fft_len", 0ULL), std::uint64_t{32}));
            expect(lt(std::abs(out[0UZ].signal_values[0UZ] - CF(32.f, 0.f)), 1e-4f)) << "a cut of ones puts 32 on DC";
        }
    };

    "a second run of the graph gives CpInsert's first output"_test = [] {
        using gr::blocks::testing::ProcessFunction;
        std::vector<std::int32_t> carriers;
        for (std::int32_t carrier = -26; carrier <= 26; ++carrier) {
            if (carrier != 0) {
                carriers.push_back(carrier);
            }
        }

        gr::Graph graph;
        auto&     source   = graph.emplaceBlock<gr::blocks::testing::TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", gr::Size_t{3U * 52U}}, {"mark_tag", false}});
        auto&     allocate = graph.emplaceBlock<gr::blocks::ofdm::CarrierAllocator>({{"fft_len", gr::Size_t{kFft}}, {"data_carriers", carriers}});
        auto&     prefix   = graph.emplaceBlock<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U, 8U}}, {"window_len", gr::Size_t{4U}}});
        auto&     sink     = graph.emplaceBlock<gr::blocks::testing::TagSink<CF, ProcessFunction::USE_PROCESS_BULK>>({{"log_samples", true}, {"log_tags", false}});
        expect(graph.connect<"out", "in">(source, allocate).has_value());
        expect(graph.connect<"out", "in">(allocate, prefix).has_value());
        expect(graph.connect<"out", "in">(prefix, sink).has_value());

        gr::scheduler::Simple<> scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());
        const std::vector<CF> first(sink._samples.begin(), sink._samples.end());
        expect(scheduler.runAndWait().has_value());
        const std::vector<CF> second(sink._samples.begin(), sink._samples.end());

        expect(eq(first.size(), 3UZ * kFft + 16UZ + 8UZ + 16UZ)) << "three symbols with the prefixes 16, 8 and 16";
        expect(std::ranges::equal(second, first)) << "the second run repeats the first, sample for sample";
    };

    "with frame_len 0 a frame runs until the next trigger, sync symbols and data symbols alike"_test = [] {
        const Stream stream = frames(2UZ, 3UZ);
        expect(eq(stream.tags.size(), 2UZ));

        CpRemove   block = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"n_sync", gr::Size_t{1U}}, {"frame_len", gr::Size_t{0U}}});
        const auto out   = remove(block, stream, 37UZ);

        expect(eq(out.size(), 6UZ)) << "two frames of one sync symbol and two data symbols";
        expect(eq(block.nDiscarded(), std::uint64_t{0}));
        for (std::size_t r = 0UZ; r < out.size(); ++r) {
            expect(eq(kindOf(out[r]), std::string(r % 3UZ == 0UZ ? "sync" : "data"))) << std::format("record {}", r);
            expect(eq(gr::blocks::ofdm::detail::metaCount(out[r], "frame_index", 99ULL), std::uint64_t{r / 3UZ})) << std::format("record {}", r);
            expect(eq(gr::blocks::ofdm::detail::metaCount(out[r], "symbol_in_frame", 99ULL), std::uint64_t{r % 3UZ})) << std::format("record {}", r);
        }
    };

    /**
     * `n_sync` 1 and the largest `frame_len` make a frame of 2^32 symbols, one past the largest 32-bit value. On every
     * target the block cuts the stream's symbols as one frame.
     */
    "a frame of 2^32 symbols runs past its first symbol on every target"_test = [] {
        constexpr std::size_t kSymbols = 3UZ;
        CpRemove              block    = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"n_sync", gr::Size_t{1U}}, {"frame_len", std::numeric_limits<gr::Size_t>::max()}});
        const auto            out      = remove(block, frames(1UZ, kSymbols), 37UZ);
        expect(eq(out.size(), kSymbols)) << "every symbol of the stream is cut";
        expect(eq(block.nDiscarded(), std::uint64_t{0}));
    };

    "a lowered frame_len or n_sync ends the open frame at once, and its partial cut counts as discarded"_test = [] {
        // One frame of four symbols, 80 samples each. 200 samples hold two symbols and 24 samples of the third cut.
        const Stream stream = frames(1UZ, 4UZ);
        struct Arm {
            std::string_view name;
            gr::property_map framing;
            gr::property_map changed;
            std::size_t      records;
            std::uint64_t    discarded;
        };
        const std::vector<Arm> arms{
            {"frame_len 4 to 1", {{"frame_len", gr::Size_t{4U}}}, {{"frame_len", gr::Size_t{1U}}}, 2UZ, 24ULL + 120ULL},
            {"n_sync 2 to 0, frame_len 2", {{"n_sync", gr::Size_t{2U}}, {"frame_len", gr::Size_t{2U}}}, {{"n_sync", gr::Size_t{0U}}}, 2UZ, 24ULL + 120ULL},
            {"frame_len 4 to 3, a length the frame has not reached", {{"frame_len", gr::Size_t{4U}}}, {{"frame_len", gr::Size_t{3U}}}, 3UZ, 80ULL},
        };
        for (const Arm& arm : arms) {
            gr::property_map settings = arm.framing;
            settings["fft_len"]       = gr::Size_t{kFft};
            settings["cp_len"]        = std::vector<gr::Size_t>{16U};
            CpRemove block            = make<CpRemove>(settings);
            auto     out              = remove(block, stream, 0UZ, 200UZ, 4096UZ);
            expect(eq(out.size(), 2UZ)) << arm.name;
            change(block, arm.changed);
            const auto rest = remove(block, stream, 200UZ, stream.samples.size(), 4096UZ);
            out.insert(out.end(), rest.begin(), rest.end());

            expect(eq(out.size(), arm.records)) << arm.name;
            expect(eq(block.nDiscarded(), arm.discarded)) << arm.name;
        }
    };

    "a trigger during a cut opens its frame, and the partial cut counts as discarded"_test = [] {
        // With frame_len 0 the cadence runs past the first frame's two symbols. The 40 samples of silence after it hold
        // the next symbol's prefix and 24 samples of its cut when the second trigger arrives.
        const Stream frame = frames(1UZ, 2UZ);
        Stream       input;
        input.samples = frame.samples;
        input.samples.insert(input.samples.end(), 40UZ, CF{});
        input.samples.insert(input.samples.end(), frame.samples.begin(), frame.samples.end());
        for (const std::size_t at : {0UZ, frame.samples.size() + 40UZ}) {
            input.tags.push_back(gr::Tag{at, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("ofdm_frame")}}});
        }

        CpRemove   block = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}});
        const auto out   = remove(block, input, 37UZ);

        expect(eq(out.size(), 4UZ)) << "two frames of two symbols";
        expect(eq(block.nDiscarded(), std::uint64_t{24}));
        if (out.size() == 4UZ) {
            expect(eq(gr::blocks::ofdm::detail::metaCount(out[2UZ], "frame_index", 99ULL), std::uint64_t{1}));
            expect(eq(gr::blocks::ofdm::detail::metaCount(out[2UZ], "discarded_samples", 99ULL), std::uint64_t{24})) << "the second frame's first record states the dropped cut";
        }
    };

    "the end of the stream drops the partial cut, and its samples count as discarded"_test = [] {
        // A trigger on sample 25 opens a frame without a frame length. The stream ends 14 samples into the third cut.
        using gr::blocks::testing::ProcessFunction;
        constexpr std::size_t kTrigger = 25UZ;
        constexpr std::size_t kPartial = 14UZ;
        constexpr std::size_t kLength  = kTrigger + 2UZ * (16UZ + kFft) + 16UZ + kPartial;

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<gr::blocks::testing::TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(kLength)}, {"mark_tag", false}});
        source._tags     = {gr::Tag{kTrigger, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("ofdm_frame")}}}};
        auto& cut        = graph.emplaceBlock<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}});
        auto& sink       = graph.emplaceBlock<gr::blocks::testing::CountingSink<gr::DataSet<CF>>>({});
        expect(graph.connect<"out", "in">(source, cut).has_value());
        expect(graph.connect<"out", "in">(cut, sink).has_value());

        gr::scheduler::Simple<> scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());
        expect(eq(sink.count.value, std::uint64_t{2})) << "two whole symbols";
        expect(eq(cut.nDiscarded(), std::uint64_t{kTrigger + kPartial})) << "the samples before the trigger and the partial cut";
    };

    /**
     * A trigger on sample 25 opens a frame with `frame_len` 0. A tag 40 samples into the third symbol sets
     * `timing_offset`. The framework refuses 1, a value outside its limits, and the run completes with the six records
     * of a run without the tag. In a third run the block takes -4 from the tag, and the change ends the open frame
     * after two records.
     */
    "a timing_offset outside its limits on a tag is refused, and the run goes on"_test = [] {
        using gr::blocks::testing::ProcessFunction;
        static constexpr std::size_t kTrigger = 25UZ;
        static constexpr std::size_t kTagged  = kTrigger + 2UZ * (16UZ + kFft) + 40UZ;
        static constexpr std::size_t kLength  = kTrigger + 6UZ * (16UZ + kFft) + 10UZ;

        struct Run {
            bool          completed = false;
            std::uint64_t records   = 0ULL;
            std::uint64_t discarded = 0ULL;
            std::int32_t  member    = 99;
            std::int32_t  reported  = 99;
        };
        const auto run = [](std::optional<std::int32_t> offset) {
            gr::Graph graph;
            auto&     source = graph.emplaceBlock<gr::blocks::testing::TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(kLength)}, {"mark_tag", false}});
            source._tags     = {gr::Tag{kTrigger, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("ofdm_frame")}}}};
            if (offset.has_value()) {
                source._tags.push_back(gr::Tag{kTagged, gr::property_map{{"timing_offset", *offset}}});
            }
            // The constructor names no settings. fft_len stays 64 and cp_len 16, and timing_offset stays 0 until the
            // tag sets it.
            auto& cut  = graph.emplaceBlock<CpRemove>(gr::property_map{});
            auto& sink = graph.emplaceBlock<gr::blocks::testing::CountingSink<gr::DataSet<CF>>>({});
            expect(graph.connect<"out", "in">(source, cut).has_value());
            expect(graph.connect<"out", "in">(cut, sink).has_value());

            gr::scheduler::Simple<> scheduler;
            expect(scheduler.exchange(std::move(graph)).has_value());
            Run result;
            result.completed                             = scheduler.runAndWait().has_value();
            result.records                               = sink.count.value;
            result.discarded                             = cut.nDiscarded();
            result.member                                = cut.timing_offset.value;
            const std::optional<gr::pmt::Value> reported = cut.settings().get("timing_offset");
            result.reported                              = reported.has_value() && reported->get_if<std::int32_t>() != nullptr ? *reported->get_if<std::int32_t>() : 99;
            return result;
        };

        const Run reference = run(std::nullopt);
        expect(reference.completed);
        expect(eq(reference.records, std::uint64_t{6}));

        const Run refused = run(std::int32_t{1});
        expect(refused.completed) << "the refusal ends no run";
        expect(eq(refused.member, std::int32_t{0})) << "the member keeps the accepted value";
        expect(eq(refused.reported, std::int32_t{0})) << "settings().get() returns the accepted value";
        expect(eq(refused.records, reference.records)) << "the frame runs on with the accepted timing_offset";
        expect(eq(refused.discarded, reference.discarded));

        const Run taken = run(std::int32_t{-4});
        expect(taken.completed);
        expect(eq(taken.member, std::int32_t{-4})) << "the tag reaches the setting";
        expect(eq(taken.records, std::uint64_t{2})) << "the change ends the open frame";
    };
};

} // namespace qa_ofdm_cp

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
