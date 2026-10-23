#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <print>
#include <span>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/ofdm/CarrierAllocator.hpp>
#include <gnuradio-4.0/ofdm/ChannelEqualizer.hpp>
#include <gnuradio-4.0/ofdm/CyclicPrefix.hpp>
#include <gnuradio-4.0/ofdm/SchmidlCoxSync.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::ofdm::CarrierAllocator;
using gr::blocks::ofdm::ChannelEqualizer;
using gr::blocks::ofdm::CpInsert;
using gr::blocks::ofdm::CpRemove;
using gr::blocks::ofdm::SchmidlCoxSync;
using gr::blocks::testing::bench::Arm;
using CF       = std::complex<float>;
namespace shim = gr::blocks::testing::span;

constexpr gr::Size_t  kFft     = 64U;
constexpr gr::Size_t  kCp      = 16U;
constexpr std::size_t kSymbol  = static_cast<std::size_t>(kFft + kCp);
constexpr std::size_t kStream  = 1UZ << 16;          ///< samples a call, for the stream-rate arms
constexpr std::size_t kSymbols = 512UZ;              ///< symbols a call, for the record arms
constexpr std::size_t kCarried = kSymbols * kSymbol; ///< the stream those symbols are worth, the record arms' denominator
constexpr std::size_t kRepeats = 9UZ;

template<typename TBlock>
void init(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

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

[[nodiscard]] std::vector<CF> sounding() {
    std::vector<CF> word(kFft, CF{});
    std::uint32_t   state = 987654321U;
    const auto      place = [&word, &state](std::int32_t carrier) {
        state                                                    = state * 1664525U + 1013904223U;
        word[gr::ofdm::CarrierMap::binOf(kFft, carrier).value()] = CF(((state >> 16U) & 1U) != 0U ? 0.7071f : -0.7071f, ((state >> 17U) & 1U) != 0U ? 0.7071f : -0.7071f);
    };
    for (const std::int32_t carrier : dataCarriers()) {
        place(carrier);
    }
    for (const std::int32_t carrier : pilotCarriers()) {
        place(carrier);
    }
    return word;
}

/// A stream with no repeated halves. The sync's metric opens no plateau, and the arm measures the steady-state cost of
/// the two sliding sums, not the cost of a detection.
[[nodiscard]] std::vector<CF> noise(std::size_t count) {
    std::vector<CF> data(count);
    std::uint32_t   state = 2026U;
    for (CF& sample : data) {
        state          = state * 1664525U + 1013904223U;
        const float re = static_cast<float>(static_cast<std::int32_t>(state >> 8U) & 0xFFFF) / 32768.f - 1.f;
        state          = state * 1664525U + 1013904223U;
        const float im = static_cast<float>(static_cast<std::int32_t>(state >> 8U) & 0xFFFF) / 32768.f - 1.f;
        sample         = CF(re, im);
    }
    return data;
}

/// @brief One call of a passthrough block over the whole sample vector, at a stream position that keeps advancing.
///
/// The blocks work in absolute stream indices. Each call therefore starts where the previous call ended.
template<typename TBlock>
[[nodiscard]] double sweepStream(TBlock& block, std::span<const CF> input, std::span<CF> output, std::size_t& at) {
    shim::InputSpan<CF>  inSpan(input, at);
    shim::OutputSpan<CF> outSpan(output, at);
    std::ignore = block.processBulk(inSpan, outSpan);
    at += inSpan.consumed;
    return static_cast<double>(output[0UZ].real());
}

} // namespace

int main() {
    const std::vector<CF> stream = noise(kStream);
    std::vector<CF>       sink(kStream);

    const auto      data   = dataCarriers();
    const auto      pilots = pilotCarriers();
    const auto      known  = sounding();
    std::vector<CF> pilotValues{CF(1.f, 0.f), CF(1.f, 0.f), CF(1.f, 0.f), CF(-1.f, 0.f)};

    // --- the stream-rate arms: the sync's two sliding sums ---
    SchmidlCoxSync bare({{"fft_len", kFft}, {"cp_len", kCp}, {"correct_cfo", false}});
    SchmidlCoxSync derotating({{"fft_len", kFft}, {"cp_len", kCp}, {"correct_cfo", true}});
    SchmidlCoxSync wide({{"fft_len", gr::Size_t{1024U}}, {"cp_len", gr::Size_t{128U}}, {"correct_cfo", false}});
    init(bare);
    init(derotating);
    init(wide);

    std::size_t bareAt       = 0UZ;
    std::size_t derotatingAt = 0UZ;
    std::size_t wideAt       = 0UZ;

    std::vector<Arm> streamArms{
        {"copy of the stream, the floor",
            [&stream, &sink] {
                std::ranges::copy(stream, sink.begin());
                return static_cast<double>(sink[0UZ].real());
            }},
        {"SchmidlCoxSync 64, no correction", [&] { return sweepStream(bare, std::span<const CF>(stream), std::span<CF>(sink), bareAt); }},
        {"SchmidlCoxSync 64, derotating", [&] { return sweepStream(derotating, std::span<const CF>(stream), std::span<CF>(sink), derotatingAt); }},
        {"SchmidlCoxSync 1024, no correction", [&] { return sweepStream(wide, std::span<const CF>(stream), std::span<CF>(sink), wideAt); }},
    };

    std::println("ns per stream sample. The sliding sums cost O(1) per sample at any fft_len.");
    gr::blocks::testing::bench::report(std::span<Arm>(streamArms), kStream, kRepeats);

    // --- the record arms, each divided by the stream its symbols occupy ---
    CarrierAllocator allocator({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilots}, {"pilot_symbols", interleave(std::span<const CF>(pilotValues))}});
    CpInsert         prefix({{"cp_len", std::vector<gr::Size_t>{kCp}}, {"emit_trigger", false}});
    CpInsert         windowed({{"cp_len", std::vector<gr::Size_t>{kCp}}, {"window_len", gr::Size_t{8U}}, {"emit_trigger", false}});
    CpRemove         remove({{"fft_len", kFft}, {"cp_len", std::vector<gr::Size_t>{kCp}}});
    ChannelEqualizer cpe({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilots}, {"pilot_symbols", interleave(std::span<const CF>(pilotValues))}, //
        {"sync_word", interleave(std::span<const CF>(known))}, {"n_sync", gr::Size_t{1U}}, {"sync_index", gr::Size_t{0U}}, {"tracking", std::string("cpe")}});
    ChannelEqualizer interp({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilots}, {"pilot_symbols", interleave(std::span<const CF>(pilotValues))}, //
        {"sync_word", interleave(std::span<const CF>(known))}, {"n_sync", gr::Size_t{1U}}, {"sync_index", gr::Size_t{0U}}, {"tracking", std::string("cpe_interp")}});
    init(allocator);
    init(prefix);
    init(windowed);
    init(remove);
    init(cpe);
    init(interp);

    // one symbol of slack. The allocator holds a sample back for its end-of-stream epilogue. A payload of exactly
    // kSymbols symbols would leave the last record of the vector default-constructed.
    const std::vector<CF>        payload = noise((kSymbols + 1UZ) * data.size());
    std::vector<gr::DataSet<CF>> records(kSymbols);
    std::vector<gr::DataSet<CF>> equalized(kSymbols);
    {
        shim::InputSpan<CF>               inSpan(std::span<const CF>(payload), 0UZ);
        shim::OutputSpan<gr::DataSet<CF>> outSpan{std::span<gr::DataSet<CF>>(records)};
        std::ignore = allocator.processBulk(inSpan, outSpan);
    }
    init(allocator);

    std::vector<CF> carried(kCarried + kSymbol);
    {
        shim::InputSpan<gr::DataSet<CF>> inSpan(std::span<const gr::DataSet<CF>>(records), 0UZ);
        shim::OutputSpan<CF>             outSpan{std::span<CF>(carried)};
        std::ignore = prefix.processBulk(inSpan, outSpan);
    }
    // init resets the prefix block, which keeps state from the call above
    init(prefix);

    std::vector<gr::DataSet<CF>> recordSink(kSymbols);
    std::vector<CF>              streamSink(kCarried + kSymbol);
    std::vector<gr::Tag>         trigger{gr::Tag{0UZ, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("ofdm_frame")}}}};
    std::size_t                  removeAt = 0UZ;

    shim::OutputSpan<gr::DataSet<float>> unwiredChannel(std::span<gr::DataSet<float>>{}, 0UZ, nullptr, false);

    const auto sweepRecords = [&recordSink, &unwiredChannel](auto& block, std::span<const gr::DataSet<CF>> input) {
        shim::InputSpan<gr::DataSet<CF>>  inSpan(input, 0UZ);
        shim::OutputSpan<gr::DataSet<CF>> outSpan{std::span<gr::DataSet<CF>>(recordSink)};
        std::ignore = block.processBulk(inSpan, outSpan, unwiredChannel);
        return static_cast<double>(recordSink[0UZ].signal_values[0UZ].real());
    };

    std::vector<Arm> recordArms{
        {"CarrierAllocator, 48 of 64 carriers",
            [&] {
                shim::InputSpan<CF>               inSpan(std::span<const CF>(payload), 0UZ);
                shim::OutputSpan<gr::DataSet<CF>> outSpan{std::span<gr::DataSet<CF>>(equalized)};
                std::ignore = allocator.processBulk(inSpan, outSpan);
                return static_cast<double>(equalized[0UZ].signal_values[0UZ].real());
            }},
        {"CpInsert, inverse transform and prefix",
            [&] {
                shim::InputSpan<gr::DataSet<CF>> inSpan(std::span<const gr::DataSet<CF>>(records), 0UZ);
                shim::OutputSpan<CF>             outSpan{std::span<CF>(streamSink)};
                std::ignore = prefix.processBulk(inSpan, outSpan);
                return static_cast<double>(streamSink[0UZ].real());
            }},
        {"CpInsert, with an 8-sample edge",
            [&] {
                shim::InputSpan<gr::DataSet<CF>> inSpan(std::span<const gr::DataSet<CF>>(records), 0UZ);
                shim::OutputSpan<CF>             outSpan{std::span<CF>(streamSink)};
                std::ignore = windowed.processBulk(inSpan, outSpan);
                return static_cast<double>(streamSink[0UZ].real());
            }},
        {"CpRemove, cut and forward transform",
            [&] {
                const std::span<const gr::Tag>    arriving = removeAt == 0UZ ? std::span<const gr::Tag>(trigger) : std::span<const gr::Tag>{};
                shim::InputSpan<CF>               inSpan(std::span<const CF>(carried).first(kCarried), removeAt, arriving);
                shim::OutputSpan<gr::DataSet<CF>> outSpan{std::span<gr::DataSet<CF>>(recordSink)};
                std::ignore = remove.processBulk(inSpan, outSpan);
                removeAt += inSpan.consumed;
                return static_cast<double>(recordSink[0UZ].signal_values[0UZ].real());
            }},
        {"ChannelEqualizer, cpe", [&] { return sweepRecords(cpe, std::span<const gr::DataSet<CF>>(records)); }},
        {"ChannelEqualizer, cpe_interp", [&] { return sweepRecords(interp, std::span<const gr::DataSet<CF>>(records)); }},
    };

    std::println("");
    std::println("ns per stream sample. Every arm divides by the {} stream samples of the {} symbols it moves.", kCarried, kSymbols);
    std::println("A symbol is {} samples: {} of transform and {} of prefix.", kSymbol, kFft, kCp);
    gr::blocks::testing::bench::report(std::span<Arm>(recordArms), kCarried, kRepeats);
}
