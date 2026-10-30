#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/channel/Awgn.hpp>
#include <gnuradio-4.0/channel/CarrierImpairments.hpp>
#include <gnuradio-4.0/ofdm/CarrierAllocator.hpp>
#include <gnuradio-4.0/ofdm/CyclicPrefix.hpp>
#include <gnuradio-4.0/ofdm/SchmidlCoxSync.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_ofdm_sync {

using gr::blocks::channel::AwgnChannel;
using gr::blocks::channel::FrequencyOffset;
using gr::blocks::ofdm::CarrierAllocator;
using gr::blocks::ofdm::CpInsert;
using gr::blocks::ofdm::SchmidlCoxSync;
using CF       = std::complex<float>;
namespace shim = gr::blocks::testing::span;

constexpr gr::Size_t  kFft   = 64U;
constexpr gr::Size_t  kCp    = 16U;
constexpr std::size_t kLead  = 137UZ; ///< silence before the frame, so the trigger is not at sample zero
constexpr std::size_t kFrame = 4UZ;

constexpr std::uint64_t kWrap = std::uint64_t{1} << 32U; ///< the stream index where a 32-bit `std::size_t` wraps to 0

/// @brief The stream index @p before samples ahead of 2^32, for @p before from 1. A run that starts there reaches 2^32 at its
/// input @p before.
[[nodiscard]] std::size_t aheadOfWrap(std::size_t before) { return std::size_t{std::numeric_limits<std::uint32_t>::max()} - (before - 1UZ); }

/// @brief The distance from @p start to each tag that carries @p key. A `std::size_t` difference is exact across a wrap.
[[nodiscard]] std::vector<std::size_t> offsetsOf(std::span<const gr::Tag> tags, std::string_view key, std::size_t start) {
    std::vector<std::size_t> offsets;
    for (const gr::Tag& tag : tags) {
        if (tag.map.contains(std::pmr::string(key))) {
            offsets.push_back(tag.index - start);
        }
    }
    return offsets;
}

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    if constexpr (requires { block.start(); }) {
        block.start();
    }
    return block;
}

/// @brief Stages and applies @p settings. Returns the refusal, or an empty string.
[[nodiscard]] std::string apply(SchmidlCoxSync& block, const gr::property_map& settings) {
    std::ignore = block.settings().setStaged(settings);
    try {
        return block.settings().applyStagedParameters().refusal;
    } catch (const gr::exception& e) {
        return std::string(e.what());
    }
}

/// @brief The transform, the preamble's prefix, the occupied carriers `-edge..edge` without DC, and the pilots among them.
struct Numerology {
    gr::Size_t                fft;
    gr::Size_t                cp;
    std::int32_t              edge;
    std::vector<std::int32_t> pilots;
};

[[nodiscard]] Numerology narrow() { return {kFft, kCp, 26, {-21, -7, 7, 21}}; }

[[nodiscard]] std::vector<std::int32_t> dataCarriers(const Numerology& numerology) {
    std::vector<std::int32_t> carriers;
    for (std::int32_t c = -numerology.edge; c <= numerology.edge; ++c) {
        if (c == 0 || std::ranges::find(numerology.pilots, c) != numerology.pilots.end()) {
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

/// @brief One transmitted frame, a Schmidl-Cox preamble and `kFrame` QPSK-loaded data symbols, with silence in front.
///
/// The frame's first sample, the first sample of the preamble's own prefix, is at index `kLead`. Every timing
/// assertion here is measured against it.
[[nodiscard]] std::vector<CF> transmit(const Numerology& numerology = narrow()) {
    const auto      data     = dataCarriers(numerology);
    const auto      preamble = gr::ofdm::schmidlCoxPreamble(static_cast<std::size_t>(numerology.fft), 2UZ * static_cast<std::size_t>(numerology.edge), 0xC0FFEEULL).value();
    std::vector<CF> pilotValues{CF(1.f, 0.f), CF(1.f, 0.f), CF(1.f, 0.f), CF(-1.f, 0.f)};

    CarrierAllocator allocator = make<CarrierAllocator>({{"fft_len", numerology.fft}, {"data_carriers", data}, {"pilot_carriers", numerology.pilots}, //
        {"pilot_symbols", interleave(pilotValues)}, {"sync_words", interleave(preamble)}, {"frame_len", static_cast<gr::Size_t>(kFrame)}});

    std::vector<CF> payload(kFrame * data.size());
    std::uint32_t   state = 12345U;
    for (CF& symbol : payload) {
        state          = state * 1664525U + 1013904223U;
        const float re = ((state >> 16U) & 1U) != 0U ? 0.70710678f : -0.70710678f;
        const float im = ((state >> 17U) & 1U) != 0U ? 0.70710678f : -0.70710678f;
        symbol         = CF(re, im);
    }

    std::vector<gr::DataSet<CF>> records;
    std::vector<gr::DataSet<CF>> scratch(64UZ);
    {
        shim::InputSpan<CF>               inSpan(std::span<const CF>(payload), 0UZ);
        shim::OutputSpan<gr::DataSet<CF>> outSpan{std::span<gr::DataSet<CF>>(scratch)};
        std::ignore = allocator.processBulk(inSpan, outSpan);
        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            records.push_back(std::move(scratch[k]));
        }
        shim::InputSpan<CF>               tail(std::span<const CF>(payload).last(payload.size() - inSpan.consumed), inSpan.consumed);
        shim::OutputSpan<gr::DataSet<CF>> outTail{std::span<gr::DataSet<CF>>(scratch)};
        std::ignore = allocator.processEpilogue(tail, outTail);
        for (std::size_t k = 0UZ; k < outTail.count; ++k) {
            records.push_back(std::move(scratch[k]));
        }
    }

    CpInsert        cp = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{numerology.cp}}});
    std::vector<CF> stream(kLead, CF{});
    std::vector<CF> body(8192UZ);
    {
        shim::InputSpan<gr::DataSet<CF>> inSpan(std::span<const gr::DataSet<CF>>(records), 0UZ);
        shim::OutputSpan<CF>             outSpan{std::span<CF>(body)};
        std::ignore = cp.processBulk(inSpan, outSpan);
        stream.insert(stream.end(), body.begin(), body.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
    }
    // two symbols of trailing silence, longer than the lag: the delayed output then carries the whole frame
    stream.insert(stream.end(), 2UZ * static_cast<std::size_t>(numerology.fft + numerology.cp), CF{});
    return stream;
}

[[nodiscard]] double meanPower(std::span<const CF> stream) {
    double sum = 0.;
    for (const CF& sample : stream) {
        sum += static_cast<double>(std::norm(sample));
    }
    return sum / static_cast<double>(stream.size());
}

/// The energy floor of the metric, set from the scene's signal: a quarter of the energy that a window of fft_len/2
/// preamble samples holds.
[[nodiscard]] float energyFloor(std::span<const CF> stream, const Numerology& numerology = narrow()) {
    const double perSample = meanPower(stream.subspan(kLead, static_cast<std::size_t>(numerology.cp + numerology.fft)));
    return static_cast<float>(0.25 * perSample * static_cast<double>(numerology.fft / 2U));
}

struct Detection {
    std::vector<std::int64_t>  biases{}; ///< trigger sample minus the constructed frame start
    std::vector<double>        cfo{};
    std::vector<std::uint32_t> plateau{}; ///< samples the metric stayed above the threshold
};

[[nodiscard]] Detection sync(SchmidlCoxSync& block, std::span<const CF> stream, std::size_t chunk, std::vector<CF>* corrected = nullptr) {
    const auto result = shim::run<CF>(block, stream, chunk);
    Detection  found;
    for (const gr::Tag& tag : result.tags) {
        const auto entry = tag.map.find(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey()));
        if (entry == tag.map.end()) {
            continue;
        }
        found.biases.push_back(static_cast<std::int64_t>(tag.index) - static_cast<std::int64_t>(block.delaySamples()) - static_cast<std::int64_t>(kLead));
        const auto meta = tag.map.find(std::pmr::string(gr::tag::TRIGGER_META_INFO.shortKey()));
        if (meta != tag.map.end()) {
            if (const auto* map = meta->second.get_if<gr::property_map>(); map != nullptr) {
                const auto value = map->find(std::pmr::string("cfo_fractional"));
                if (value != map->end()) {
                    if (const auto* eps = value->second.get_if<float>(); eps != nullptr) {
                        found.cfo.push_back(static_cast<double>(*eps));
                    }
                }
                const auto width = map->find(std::pmr::string("plateau_samples"));
                if (width != map->end()) {
                    if (const auto* samples = width->second.get_if<gr::Size_t>(); samples != nullptr) {
                        found.plateau.push_back(*samples);
                    }
                }
            }
        }
    }
    if (corrected != nullptr) {
        *corrected = result.samples;
    }
    return found;
}

/// @brief The samples and tags of a run in the framework's order of calls.
struct Driven {
    std::vector<CF>      samples{};
    std::vector<gr::Tag> tags{};
};

/**
 * @brief Drives the block in the framework's order: `forwardTags()` then `processBulk()` on each chunk, and the
 * end-of-stream call last.
 *
 * The tag cursor hands each call the tags up to the end of its window and retires them by the rule a released span
 * applies. The end-of-stream call passes `processedIn` 0 and spans of no samples. Its input span holds every tag the
 * cursor has not retired.
 */
struct Drive {
    SchmidlCoxSync&     block;
    std::span<const CF> input;
    shim::TagCursor     cursor;
    Driven              out{};
    std::size_t         position = 0UZ;

    void to(std::size_t end, std::size_t chunk) {
        std::vector<CF> scratch(chunk);
        while (position < end) {
            const std::size_t    count = std::min(chunk, end - position);
            shim::InputSpan<CF>  inSpan(input.subspan(position, count), position, cursor.window(position, count));
            shim::OutputSpan<CF> outSpan(std::span<CF>(scratch.data(), count), position, &out.tags);
            auto                 inputs  = std::tie(inSpan);
            auto                 outputs = std::tie(outSpan);
            block.forwardTags(inputs, outputs, count);
            std::ignore = block.processBulk(inSpan, outSpan);
            cursor.retire(inSpan);
            out.samples.insert(out.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
            position += count;
        }
    }

    void finish() {
        shim::InputSpan<CF>  inSpan(std::span<const CF>{}, position, cursor.window(position, std::numeric_limits<std::size_t>::max() - position));
        shim::OutputSpan<CF> outSpan(std::span<CF>{}, position, &out.tags);
        auto                 inputs  = std::tie(inSpan);
        auto                 outputs = std::tie(outSpan);
        block.forwardTags(inputs, outputs, 0UZ);
    }
};

/// @brief The `mark` and `trigger_name` tags of a run, as `key=value@index` in the order the block published them.
[[nodiscard]] std::vector<std::string> tagList(std::span<const gr::Tag> tags) {
    std::vector<std::string> list;
    for (const gr::Tag& tag : tags) {
        for (const auto& [key, value] : tag.map) {
            if (key == "mark" || key == std::pmr::string(gr::tag::TRIGGER_NAME.shortKey())) {
                list.push_back(std::format("{}={}@{}", std::string(key), value.value_or(std::string{}), tag.index));
            }
        }
    }
    return list;
}

/// @brief The residual offset a stream still carries, read off the preamble's own repeated halves.
///
/// The preamble at `at` is `cp_len` prefix samples followed by two identical halves. Correlating the halves gives
/// `pi * eps`, as in the block's own estimator.
[[nodiscard]] double residualCfo(std::span<const CF> stream, std::size_t at) {
    const std::size_t    half  = static_cast<std::size_t>(kFft) / 2UZ;
    const std::size_t    start = at + static_cast<std::size_t>(kCp);
    std::complex<double> sum{};
    for (std::size_t m = 0UZ; m < half; ++m) {
        sum += std::conj(std::complex<double>(stream[start + m])) * std::complex<double>(stream[start + m + half]);
    }
    return std::atan2(sum.imag(), sum.real()) / std::numbers::pi;
}

const boost::ut::suite<"OFDM Schmidl-Cox sync"> _sync = [] {
    using namespace boost::ut;

    const std::vector<CF> clean = transmit();
    const float           floor = energyFloor(std::span<const CF>(clean));

    /**
     * The exact-correlation plateau runs from the frame's first sample `t0` to `cp_len` samples later, and its
     * midpoint is `t0 + cp_len/2`. The threshold crossings are wider than that plateau, and their position depends on
     * the preamble's energy over time. At `d = t0 - delta` the correlation holds the energy of the window's last
     * `L - delta` samples, and the reference holds all `L`. The rising edge is early by the samples that carry the
     * first fifth of the symbol's energy. The falling edge is late by the matching count at the other end, plus the
     * incoherent terms against the next symbol. The sequence sets this bias. The test asserts that it stays
     * inside a quarter of the prefix and that it does not depend on how the stream is chunked.
     */
    "the trigger lands within a quarter prefix of the constructed frame start, at any chunking"_test = [&clean, floor] {
        std::vector<std::int64_t> biases;
        for (const std::size_t chunk : {1UZ, 5UZ, 64UZ, 331UZ, 4096UZ}) {
            SchmidlCoxSync  block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}});
            const Detection found = sync(block, std::span<const CF>(clean), chunk);
            expect(eq(found.biases.size(), 1UZ)) << std::format("chunk {}: one preamble, one trigger", chunk);
            if (found.biases.empty()) {
                continue;
            }
            biases.push_back(found.biases[0UZ]);
            expect(le(4 * std::abs(found.biases[0UZ]), static_cast<std::int64_t>(kCp))) << std::format("chunk {}: plateau-midpoint bias", chunk);
            expect(eq(block.nShortPlateaus(), std::uint64_t{0}));
            if (chunk == 4096UZ) {
                std::println("plateau-midpoint timing bias, no impairment: {:+} samples of a {}-sample prefix, the crossings {} samples wide against the {}-sample exact plateau", //
                    found.biases[0UZ], kCp, found.plateau[0UZ], kCp + 1U);
            }
        }
        expect(eq(std::ranges::count(biases, biases.front()), std::ssize(biases))) << "the same stream chunked differently triggers at the same sample";
    };

    /**
     * A prefix much shorter than the transform leaves the crossings far wider than the exact plateau. The sample that
     * ends such a crossing arrives long after the frame's first sample, and a call that starts at that sample has
     * already published every output before it.
     */
    "a short prefix loses no frame at any chunk size"_test = [] {
        for (const Numerology& numerology : {Numerology{256U, 16U, 100, {-75, -25, 25, 75}}, Numerology{512U, 32U, 200, {-150, -50, 50, 150}}}) {
            const std::vector<CF>     stream = transmit(numerology);
            const float               rFloor = energyFloor(std::span<const CF>(stream), numerology);
            std::vector<std::int64_t> biases;
            std::uint32_t             width = 0U;
            for (std::size_t chunk = 1UZ; chunk <= 64UZ; ++chunk) {
                SchmidlCoxSync  block = make<SchmidlCoxSync>({{"fft_len", numerology.fft}, {"cp_len", numerology.cp}, {"r_floor", rFloor}, {"correct_cfo", false}});
                const Detection found = sync(block, std::span<const CF>(stream), chunk);
                expect(eq(found.biases.size(), 1UZ)) << std::format("fft_len {} cp_len {} chunk {}: one preamble, one trigger", numerology.fft, numerology.cp, chunk);
                expect(eq(block.nLate(), std::uint64_t{0})) << std::format("fft_len {} cp_len {} chunk {}: no detection is late", numerology.fft, numerology.cp, chunk);
                if (!found.biases.empty()) {
                    biases.push_back(found.biases[0UZ]);
                    width = found.plateau[0UZ];
                }
            }
            expect(eq(biases.size(), 64UZ));
            if (biases.empty()) {
                continue;
            }
            expect(eq(std::ranges::count(biases, biases.front()), std::ssize(biases))) << "the same stream chunked differently triggers at the same sample";
            std::println("fft_len {} cp_len {}: trigger bias {:+} samples, the crossings {} samples wide against the {}-sample exact plateau", numerology.fft, numerology.cp, biases.front(), width, numerology.cp + 1U);
        }
    };

    "a crossing wider than any preamble's plateau emits no trigger"_test = [&clean, floor] {
        // a tone repeats at every lag, and M stays at 1 for as long as it lasts
        const auto      amplitude = static_cast<float>(std::sqrt(meanPower(std::span<const CF>(clean).subspan(kLead, static_cast<std::size_t>(kCp + kFft)))));
        std::vector<CF> tone(kLead, CF{});
        for (std::size_t n = 0UZ; n < 1000UZ; ++n) {
            tone.push_back(std::polar(amplitude, static_cast<float>(0.1 * std::numbers::pi * static_cast<double>(n))));
        }
        tone.insert(tone.end(), 256UZ, CF{});
        for (const std::size_t chunk : {1UZ, 64UZ, 4096UZ}) {
            SchmidlCoxSync block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}});
            std::ignore          = shim::run<CF>(block, std::span<const CF>(tone), chunk);
            expect(eq(block.nTriggers(), std::uint64_t{0})) << std::format("chunk {}", chunk);
            expect(eq(block.nWidePlateaus(), std::uint64_t{1})) << std::format("chunk {}: the tone's crossing is refused", chunk);
        }
    };

    "silence emits no trigger"_test = [floor] {
        const std::vector<CF> quiet(2048UZ, CF{});
        SchmidlCoxSync        block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}});
        std::ignore                 = shim::run<CF>(block, std::span<const CF>(quiet), 256UZ);
        expect(eq(block.nTriggers(), std::uint64_t{0}));
    };

    "a NaN or infinite sample costs no later frame"_test = [&clean, floor] {
        std::vector<CF> twice = clean;
        twice.insert(twice.end(), clean.begin(), clean.end());
        for (const CF bad : {CF(std::numeric_limits<float>::quiet_NaN(), 0.f), CF(std::numeric_limits<float>::infinity(), 0.f)}) {
            std::vector<CF> input = twice;
            input[10UZ]           = bad; // in the silence before the first frame
            SchmidlCoxSync block  = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}});
            std::ignore           = shim::run<CF>(block, std::span<const CF>(input), 64UZ);
            expect(eq(block.nTriggers(), std::uint64_t{2})) << std::format("{} at sample 10: two frames, two triggers", bad.real());
        }
    };

    "the fractional offset is estimated and corrected to under a thousandth of a spacing"_test = [&clean, floor] {
        for (const double eps : {-0.4, 0.25}) {
            // at a sample_rate of fft_len one subcarrier spacing is one hertz, and frequency_offset equals eps
            FrequencyOffset<CF> impair  = make<FrequencyOffset<CF>>({{"sample_rate", static_cast<float>(kFft)}, {"frequency_offset", eps}});
            const auto          shifted = shim::run<CF>(impair, std::span<const CF>(clean), 512UZ);
            const double        truth   = residualCfo(std::span<const CF>(shifted.samples), kLead);

            SchmidlCoxSync  block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}});
            std::vector<CF> corrected;
            const Detection found = sync(block, std::span<const CF>(shifted.samples), 97UZ, &corrected);

            expect(eq(found.cfo.size(), 1UZ)) << std::format("eps {}", eps);
            if (found.cfo.empty()) {
                continue;
            }
            const double estimateError = std::abs(found.cfo[0UZ] - eps);
            const double residual      = std::abs(residualCfo(std::span<const CF>(corrected), kLead + block.delaySamples()));
            std::println("eps {:+.2f}: the stream carries {:+.6f}, the estimate is {:+.6f} (error {:.2e}), the corrected stream leaves {:.2e}", eps, truth, found.cfo[0UZ], estimateError, residual);
            expect(lt(estimateError, 1e-3)) << std::format("eps {}: the estimate", eps);
            expect(lt(residual, 1e-3)) << std::format("eps {}: what the correction left", eps);
            expect(le(4 * std::abs(found.biases[0UZ]), static_cast<std::int64_t>(kCp))) << std::format("eps {}: an offset does not move the timing", eps);
        }
    };

    "detection statistics through additive noise at 10 dB"_test = [&clean, floor] {
        constexpr std::size_t kTrials = 200UZ;
        const double          signal  = meanPower(std::span<const CF>(clean).subspan(kLead, kFrame * static_cast<std::size_t>(kCp + kFft)));
        const double          noise   = signal / 10.; // 10 dB, both quadratures together

        std::size_t               detected = 0UZ;
        std::size_t               spurious = 0UZ;
        std::vector<std::int64_t> biases;
        std::vector<double>       errors;
        for (std::size_t trial = 0UZ; trial < kTrials; ++trial) {
            AwgnChannel<CF> channel = make<AwgnChannel<CF>>({{"noise_power", noise}, {"seed", std::uint64_t{trial + 1UZ}}});
            const auto      noisy   = shim::run<CF>(channel, std::span<const CF>(clean), 1024UZ);

            SchmidlCoxSync  block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}});
            const Detection found = sync(block, std::span<const CF>(noisy.samples), 512UZ);
            if (found.biases.empty()) {
                continue;
            }
            ++detected;
            spurious += found.biases.size() - 1UZ;
            biases.push_back(found.biases[0UZ]);
            errors.push_back(found.cfo.empty() ? 0. : found.cfo[0UZ]);
        }

        std::ranges::sort(biases);
        double mean = 0.;
        for (const std::int64_t bias : biases) {
            mean += static_cast<double>(bias);
        }
        mean /= static_cast<double>(biases.size());
        double variance = 0.;
        double worstCfo = 0.;
        for (std::size_t k = 0UZ; k < biases.size(); ++k) {
            variance += (static_cast<double>(biases[k]) - mean) * (static_cast<double>(biases[k]) - mean);
            worstCfo = std::max(worstCfo, std::abs(errors[k]));
        }
        variance /= static_cast<double>(biases.size());

        std::println("AWGN at 10 dB, {} trials: detected {}, spurious {}; timing bias mean {:+.2f} samples, sigma {:.2f}, range [{}, {}]; worst |cfo| {:.4f} spacings", //
            kTrials, detected, spurious, mean, std::sqrt(variance), biases.front(), biases.back(), worstCfo);

        expect(eq(detected, kTrials)) << "every frame is found at 10 dB";
        expect(eq(spurious, 0UZ)) << "and found once";
        expect(lt(4. * std::abs(mean), static_cast<double>(kCp))) << "the plateau midpoint stays inside a quarter of the prefix";
        expect(lt(std::sqrt(variance), 3.0)) << "and its spread is a small fraction of the prefix";
        expect(lt(worstCfo, 0.1)) << "no trial's fractional estimate is out by a tenth of a spacing";
    };

    /**
     * Two frames back to back put the second preamble `kFrame + 1` symbols after the first. A dead time shorter than
     * that finds both preambles, and a longer one refuses the second.
     */
    "the dead time refuses a preamble inside it"_test = [&clean, floor] {
        const std::size_t frameSamples = (kFrame + 1UZ) * static_cast<std::size_t>(kCp + kFft);
        const auto        frame        = std::span<const CF>(clean).subspan(kLead, frameSamples);
        std::vector<CF>   stream(kLead, CF{});
        stream.insert(stream.end(), frame.begin(), frame.end());
        stream.insert(stream.end(), frame.begin(), frame.end());
        stream.insert(stream.end(), clean.size() - kLead - frameSamples, CF{});

        for (const auto& [gap, triggers] : {std::pair{gr::Size_t{1U}, 2UZ}, std::pair{static_cast<gr::Size_t>(kFrame + 2UZ), 1UZ}}) {
            SchmidlCoxSync  block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"min_gap", gap}});
            const Detection found = sync(block, std::span<const CF>(stream), 256UZ);
            expect(eq(found.biases.size(), triggers)) << std::format("min_gap {}: two preambles {} samples apart", gap, frameSamples);
            if (found.biases.size() == 2UZ) {
                expect(le(4 * std::abs(found.biases[1UZ] - std::ssize(frame)), static_cast<std::int64_t>(kCp))) << "the second trigger lands within a quarter prefix of its frame";
            }
        }
    };

    "a burst that ends with the stream leaves its last delaySamples() inputs in the delay line"_test = [&clean, floor] {
        const std::span<const CF>  burst = std::span<const CF>(clean).first(kLead + (kFrame + 1UZ) * static_cast<std::size_t>(kCp + kFft));
        const std::vector<gr::Tag> inTags{gr::Tag{burst.size() - 1UZ, gr::property_map{{"mark", std::string("last")}}}};
        SchmidlCoxSync             block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}});
        Drive                      drive{block, burst, shim::TagCursor(std::span<const gr::Tag>(inTags))};
        drive.to(burst.size(), 64UZ);
        drive.finish();

        const std::size_t         delay   = block.delaySamples();
        const std::span<const CF> out     = drive.out.samples;
        const auto                trigger = std::ranges::find_if(drive.out.tags, [](const gr::Tag& tag) { return tag.map.contains(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey())); });
        expect(eq(out.size(), burst.size())) << "one output per input";
        expect(std::ranges::all_of(out.first(delay), [](const CF& sample) { return sample == CF{}; })) << "the delay line starts empty";
        expect(std::ranges::equal(out.subspan(delay), burst.first(burst.size() - delay))) << "each input comes out delaySamples() later";
        expect(trigger != drive.out.tags.end()) << "the frame's trigger is output";
        if (trigger == drive.out.tags.end()) {
            return;
        }
        expect(le(4 * std::abs(static_cast<std::int64_t>(trigger->index - delay) - static_cast<std::int64_t>(kLead)), static_cast<std::int64_t>(kCp)));
        const std::vector<std::string> expected{std::format("trigger_name=ofdm_frame@{}", trigger->index), std::format("mark=last@{}", burst.size())};
        expect(tagList(drive.out.tags) == expected) << std::format("the tag on the last input leaves at the end-of-stream index: {}", tagList(drive.out.tags));
    };

    /**
     * In the framework's order of calls, an input tag near the frame's start leaves on its own sample `delaySamples()`
     * later. A tag held past the last output and a tag at the end-of-stream index leave at that index in the order of
     * their samples.
     */
    "input tags keep their samples beside a detection, at any chunking"_test = [&clean, floor] {
        const std::size_t          end = clean.size();
        const std::vector<gr::Tag> inTags{gr::Tag{kLead + 10UZ, gr::property_map{{"mark", std::string("frame")}}}, //
            gr::Tag{end - 5UZ, gr::property_map{{"mark", std::string("tail")}}},                                   //
            gr::Tag{end, gr::property_map{{"mark", std::string("end")}}},                                          //
            gr::Tag{end, gr::property_map{{static_cast<std::pmr::string>(gr::tag::END_OF_STREAM), true}}}};
        for (const std::size_t chunk : {1UZ, 7UZ, 64UZ, 4096UZ}) {
            SchmidlCoxSync block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}});
            Drive          drive{block, std::span<const CF>(clean), shim::TagCursor(std::span<const gr::Tag>(inTags))};
            drive.to(end, chunk);
            drive.finish();

            const auto trigger = std::ranges::find_if(drive.out.tags, [](const gr::Tag& tag) { return tag.map.contains(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey())); });
            expect(trigger != drive.out.tags.end()) << std::format("chunk {}: the frame's trigger is output", chunk);
            if (trigger == drive.out.tags.end()) {
                continue;
            }
            const std::vector<std::string> expected{std::format("trigger_name=ofdm_frame@{}", trigger->index), std::format("mark=frame@{}", kLead + 10UZ + block.delaySamples()), //
                std::format("mark=tail@{}", end), std::format("mark=end@{}", end)};
            expect(tagList(drive.out.tags) == expected) << std::format("chunk {}: {} against {}", chunk, tagList(drive.out.tags), expected);
            expect(std::ranges::none_of(drive.out.tags, [](const gr::Tag& tag) { return tag.map.contains(static_cast<std::pmr::string>(gr::tag::END_OF_STREAM)); })) << std::format("chunk {}: the input's end_of_stream key is dropped", chunk);
        }
    };

    /**
     * Input triggers named "ofdm_frame" sit on the frame's first sample, 20 samples later beside a `mark` key, and at
     * the end of the stream. One named "burst" sits 30 samples into the frame. The block drops the trigger keys of each
     * input tag with its own label and forwards the `mark` key. A trigger with the other label passes.
     */
    "an input trigger with the block's own trigger_label loses its trigger keys, at any chunking"_test = [&clean, floor] {
        const std::size_t          end  = clean.size();
        const std::pmr::string     name = gr::tag::TRIGGER_NAME.shortKey();
        const std::vector<gr::Tag> inTags{gr::Tag{kLead, gr::property_map{{name, std::string("ofdm_frame")}, {gr::tag::TRIGGER_OFFSET.shortKey(), 0.f}}}, //
            gr::Tag{kLead + 20UZ, gr::property_map{{name, std::string("ofdm_frame")}, {"mark", std::string("kept")}}},                                    //
            gr::Tag{kLead + 30UZ, gr::property_map{{name, std::string("burst")}}}, gr::Tag{end, gr::property_map{{name, std::string("ofdm_frame")}}}};
        const auto                 named = [&name](std::span<const gr::Tag> tags, const std::string& label) {
            return std::ranges::count_if(tags, [&name, &label](const gr::Tag& tag) {
                const auto entry = tag.map.find(name);
                return entry != tag.map.end() && entry->second.value_or(std::string{}) == label;
            });
        };
        for (const std::size_t chunk : {7UZ, 4096UZ}) {
            for (const std::string own : {"ofdm_frame", "burst"}) {
                const std::string other = own == "ofdm_frame" ? "burst" : "ofdm_frame";
                gr::property_map  settings{{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}};
                if (own != "ofdm_frame") {
                    settings["trigger_label"] = own;
                }
                SchmidlCoxSync block = make<SchmidlCoxSync>(settings);
                Drive          drive{block, std::span<const CF>(clean), shim::TagCursor(std::span<const gr::Tag>(inTags))};
                drive.to(end, chunk);
                drive.finish();

                expect(eq(block.nTriggers(), std::uint64_t{1}));
                expect(eq(named(drive.out.tags, own), std::ptrdiff_t{1})) << std::format("chunk {}, trigger_label {}: the block's own trigger alone", chunk, own);
                expect(eq(named(drive.out.tags, other), own == "ofdm_frame" ? std::ptrdiff_t{1} : std::ptrdiff_t{3})) << std::format("chunk {}, trigger_label {}: every {} trigger", chunk, own, other);
                const auto mark = std::ranges::find_if(drive.out.tags, [](const gr::Tag& tag) { return tag.map.contains(std::pmr::string("mark")); });
                expect(mark != drive.out.tags.end()) << std::format("chunk {}, trigger_label {}: the mark is forwarded", chunk, own);
                if (mark != drive.out.tags.end()) {
                    expect(eq(mark->index, kLead + 20UZ + block.delaySamples()));
                    expect(eq(mark->map.size(), own == "ofdm_frame" ? 1UZ : 2UZ)) << std::format("chunk {}, trigger_label {}: the keys beside the mark", chunk, own);
                }
            }
        }
    };

    /**
     * At sample 240 the block has confirmed the frame that starts at 137 and has not yet output its trigger's sample.
     * It also holds the input tag on sample 150. A change of a key the lag does not depend on keeps all of that.
     */
    "a threshold, min_gap or r_floor change keeps the delay line, the held detection and the held tags"_test = [&clean, floor] {
        const std::vector<gr::Tag> inTags{gr::Tag{150UZ, gr::property_map{{"mark", std::string("held")}}}};
        constexpr std::size_t      kSplit = 240UZ;

        struct Run {
            Driven        out{};
            std::uint64_t triggersAtSplit = 0ULL;
        };
        const auto run = [&clean, &inTags, floor](const gr::property_map& change) {
            SchmidlCoxSync block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}});
            Drive          drive{block, std::span<const CF>(clean), shim::TagCursor(std::span<const gr::Tag>(inTags))};
            drive.to(kSplit, 64UZ);
            Run result;
            result.triggersAtSplit = block.nTriggers();
            if (!change.empty()) {
                std::ignore = block.settings().setStaged(change);
                std::ignore = block.settings().applyStagedParameters();
            }
            drive.to(clean.size(), 64UZ);
            drive.finish();
            result.out = std::move(drive.out);
            return result;
        };

        const Run  reference = run({});
        const auto expected  = tagList(reference.out.tags);
        const auto trigger   = std::ranges::find_if(reference.out.tags, [](const gr::Tag& tag) { return tag.map.contains(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey())); });
        expect(eq(reference.triggersAtSplit, std::uint64_t{1})) << "the detection is confirmed before the change";
        expect(trigger != reference.out.tags.end() && trigger->index >= kSplit) << "and its sample is output after it";
        expect(eq(expected.size(), 2UZ)) << "one trigger and one input tag";
        for (const gr::property_map& change : {gr::property_map{{"threshold", 0.65f}}, gr::property_map{{"min_gap", gr::Size_t{2U}}}, gr::property_map{{"r_floor", 0.5f * floor}}}) {
            const Run  changed = run(change);
            const auto key     = std::string(change.begin()->first);
            expect(changed.out.samples == reference.out.samples) << std::format("{}: the delay line", key);
            expect(tagList(changed.out.tags) == expected) << std::format("{}: the held detection and tag, {} against {}", key, tagList(changed.out.tags), expected);
        }
    };

    /**
     * The frame at sample 137 carries a fifth of a subcarrier spacing. At sample 240 the block has confirmed the frame
     * and has not yet output its trigger's sample. A change of `correct_cfo` there applies from the next output
     * sample. The output from 240 on equals that of a block that held the new value from the start.
     */
    "a correct_cfo change before a held trigger applies to that trigger's frame"_test = [&clean, floor] {
        constexpr double          kEps    = 0.2;
        FrequencyOffset<CF>       impair  = make<FrequencyOffset<CF>>({{"sample_rate", static_cast<float>(kFft)}, {"frequency_offset", kEps}});
        const auto                shifted = shim::run<CF>(impair, std::span<const CF>(clean), 512UZ);
        const std::span<const CF> input(shifted.samples);
        constexpr std::size_t     kSplit = 240UZ;

        struct Run {
            Driven        out{};
            std::uint64_t triggersAtSplit = 0ULL;
            std::size_t   delay           = 0UZ;
        };
        const auto run = [input, floor](bool initial, std::optional<bool> change) {
            SchmidlCoxSync block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", initial}});
            Drive          drive{block, input, shim::TagCursor(std::span<const gr::Tag>{})};
            drive.to(kSplit, 64UZ);
            Run result;
            result.triggersAtSplit = block.nTriggers();
            if (change.has_value()) {
                expect(apply(block, {{"correct_cfo", *change}}).empty()) << "the change applies";
            }
            drive.to(input.size(), 64UZ);
            result.delay = block.delaySamples();
            result.out   = std::move(drive.out);
            return result;
        };
        const auto after = [](const Run& result) { return std::span<const CF>(result.out.samples).subspan(kSplit); };

        const Run on       = run(true, std::nullopt);
        const Run off      = run(false, std::nullopt);
        const Run enabled  = run(false, true);
        const Run disabled = run(true, false);

        const auto trigger = std::ranges::find_if(enabled.out.tags, [](const gr::Tag& tag) { return tag.map.contains(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey())); });
        expect(eq(enabled.triggersAtSplit, std::uint64_t{1})) << "the detection is confirmed before the change";
        expect(trigger != enabled.out.tags.end() && trigger->index >= kSplit) << "and its sample is output after it";
        for (const Run* arm : {&off, &enabled, &disabled}) {
            expect(tagList(arm->out.tags) == tagList(on.out.tags)) << "every arm places the same trigger";
        }

        const std::size_t frameOut  = kLead + on.delay;
        const double      carried   = residualCfo(input, kLead);
        const double      untouched = residualCfo(std::span<const CF>(off.out.samples), frameOut);
        const double      corrected = residualCfo(std::span<const CF>(enabled.out.samples), frameOut);
        std::println("correct_cfo switched on at sample {}, before the trigger at {}: the stream carries {:+.6f}, the output leaves {:.2e}", kSplit, frameOut, carried, std::abs(corrected));
        expect(lt(std::abs(carried - kEps), 1e-3)) << "the stream carries the offset";
        expect(eq(untouched, carried)) << "the output of a block that never corrects carries it unchanged";
        expect(lt(std::abs(corrected), 1e-3)) << "the change corrects the held trigger's frame";
        expect(std::ranges::equal(after(enabled), after(on))) << "switched on: the output from the change on";
        expect(std::ranges::equal(after(disabled), after(off))) << "switched off: the output from the change on";
    };

    "settings the estimator cannot work under are refused"_test = [] {
        const auto refusal = [](const gr::property_map& settings) {
            try {
                std::ignore = make<SchmidlCoxSync>(settings);
            } catch (const gr::exception& e) {
                return std::string(e.what());
            }
            return std::string{};
        };
        expect(refusal({{"fft_len", kFft}, {"cp_len", gr::Size_t{80U}}}).contains("cp_len")) << "a prefix longer than the symbol";
        for (const auto& [key, settings] : {std::pair{"threshold", gr::property_map{{"threshold", 1.f}}}, std::pair{"r_floor", gr::property_map{{"r_floor", 0.f}}}, std::pair{"fft_len", gr::property_map{{"fft_len", gr::Size_t{100U}}}}}) {
            const std::string message = refusal(settings);
            expect(message.contains(key) && message.contains("outside the limits")) << std::format("the limits of {} refuse it: '{}'", key, message);
        }
    };

    /**
     * The block computes on the settings it last accepted, whether a limit or a check refuses the application. The
     * second frame after a refused application leaves with the same samples and the same trigger as without it.
     */
    "a refused setting changes nothing the block computes"_test = [&clean, floor] {
        std::vector<CF> twice = clean;
        twice.insert(twice.end(), clean.begin(), clean.end());
        FrequencyOffset<CF>       impair  = make<FrequencyOffset<CF>>({{"sample_rate", static_cast<float>(kFft)}, {"frequency_offset", 0.25}});
        const auto                shifted = shim::run<CF>(impair, std::span<const CF>(twice), 512UZ);
        const std::span<const CF> input(shifted.samples);
        const std::size_t         split = clean.size(); // the first frame's trigger is out, and the second frame is ahead

        struct Run {
            Driven        out{};
            std::uint64_t triggers = 0ULL;
        };
        const auto run = [input, split, floor](const gr::property_map& change, std::string_view key) {
            SchmidlCoxSync block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}});
            Drive          drive{block, input, shim::TagCursor(std::span<const gr::Tag>{})};
            drive.to(split, 64UZ);
            if (!change.empty()) {
                const std::string message = apply(block, change);
                expect(message.contains(key)) << std::format("the application is refused by {}: '{}'", key, message);
            }
            drive.to(input.size(), 64UZ);
            return Run{std::move(drive.out), block.nTriggers()};
        };

        const Run reference = run({}, {});
        expect(eq(reference.triggers, std::uint64_t{2}));
        struct Refused {
            std::string      name;
            std::string_view key;
            gr::property_map change;
        };
        const std::vector<Refused> refused{
            {"threshold 1.5, outside its limits", "threshold", {{"threshold", 1.5f}}},
            {"a NaN threshold, outside its limits", "threshold", {{"threshold", std::numeric_limits<float>::quiet_NaN()}}},
            {"fft_len 100, outside its limits", "fft_len", {{"fft_len", gr::Size_t{100U}}}},
            {"r_floor 0, outside its limits", "r_floor", {{"r_floor", 0.f}}},
            {"cp_len 80 with correct_cfo", "cp_len", {{"cp_len", gr::Size_t{80U}}, {"correct_cfo", false}}},
        };
        for (const auto& [name, key, change] : refused) {
            const Run  changed = run(change, key);
            const auto tags    = tagList(changed.out.tags);
            expect(eq(changed.triggers, reference.triggers)) << std::format("{}: the second frame is found", name);
            expect(changed.out.samples == reference.out.samples) << std::format("{}: the derotated stream", name);
            expect(tags == tagList(reference.out.tags)) << std::format("{}: {} against {}", name, tags, tagList(reference.out.tags));
        }
    };

    /**
     * Each arm refuses an application before a valid change of another key. The output and its tags equal those of
     * the valid change alone.
     */
    "a refused application is written back, and a valid change after it leaves the refused values out"_test = [&clean, floor] {
        std::vector<CF> twice = clean;
        twice.insert(twice.end(), clean.begin(), clean.end());
        FrequencyOffset<CF>       impair  = make<FrequencyOffset<CF>>({{"sample_rate", static_cast<float>(kFft)}, {"frequency_offset", 0.25}});
        const auto                shifted = shim::run<CF>(impair, std::span<const CF>(twice), 512UZ);
        const std::span<const CF> input(shifted.samples);
        const std::size_t         split = clean.size();

        struct Arm {
            std::string      name;
            gr::property_map refused;
            gr::property_map valid;
        };
        const std::vector<Arm> arms{
            {"fft_len 8 with threshold, then trigger_label", {{"fft_len", gr::Size_t{8U}}, {"threshold", 0.65f}}, {{"trigger_label", std::string("x")}}},
            {"cp_len 80 with trigger_label, then min_gap", {{"cp_len", gr::Size_t{80U}}, {"trigger_label", std::string("other")}}, {{"min_gap", gr::Size_t{2U}}}},
            {"cp_len 80 with correct_cfo, then fft_len 128", {{"cp_len", gr::Size_t{80U}}, {"correct_cfo", false}}, {{"fft_len", gr::Size_t{128U}}}},
        };
        for (const Arm& arm : arms) {
            const auto run = [input, split, floor, &arm](bool refuseFirst) {
                SchmidlCoxSync block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}});
                Drive          drive{block, input, shim::TagCursor(std::span<const gr::Tag>{})};
                drive.to(split, 64UZ);
                if (refuseFirst) {
                    std::ignore = block.settings().setStaged(arm.refused);
                    expect(throws([&block] { std::ignore = block.settings().applyStagedParameters(); })) << std::format("{}: the first application is refused", arm.name);
                }
                std::ignore = block.settings().setStaged(arm.valid);
                expect(nothrow([&block] { std::ignore = block.settings().applyStagedParameters(); })) << std::format("{}: the valid change applies", arm.name);
                drive.to(input.size(), 64UZ);
                return std::pair{std::move(drive.out), block.nTriggers()};
            };
            const auto [expected, expectedTriggers] = run(false);
            const auto [got, gotTriggers]           = run(true);
            expect(eq(gotTriggers, expectedTriggers)) << std::format("{}: the triggers", arm.name);
            expect(got.samples == expected.samples) << std::format("{}: the output stream", arm.name);
            expect(tagList(got.tags) == tagList(expected.tags)) << std::format("{}: {} against {}", arm.name, tagList(got.tags), tagList(expected.tags));
        }

        // A refused value is written back, and sent again alone it is refused again.
        SchmidlCoxSync block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}});
        expect(apply(block, {{"cp_len", gr::Size_t{80U}}}).contains("cp_len"));
        const std::optional<gr::pmt::Value> reported = block.settings().get("cp_len");
        expect(eq(block.cp_len.value, kCp) && reported.has_value() && reported->get_if<gr::Size_t>() != nullptr && *reported->get_if<gr::Size_t>() == kCp) << "the member and settings().get() return the accepted value";
        expect(apply(block, {{"cp_len", gr::Size_t{80U}}}).contains("cp_len")) << "the refused value sent again alone is refused again";

        // A NaN is outside the limits of threshold, and the framework refuses it each time it is sent.
        for (int attempt = 0; attempt < 2; ++attempt) {
            const std::string message = apply(block, {{"threshold", std::numeric_limits<float>::quiet_NaN()}});
            expect(message.contains("threshold") && message.contains("outside the limits")) << std::format("attempt {}: '{}'", attempt, message);
        }
        expect(eq(block.threshold.value, 0.6f)) << "the member keeps the accepted value";

        // The framework refuses the value outside its limits alone, and the other settings of the application apply.
        const std::string mixed = apply(block, {{"trigger_label", std::string("other")}, {"r_floor", 0.f}});
        expect(mixed.contains("r_floor") && !mixed.contains("trigger_label")) << mixed;
        expect(eq(block.trigger_label.value, std::string("other")));
        expect(eq(block.r_floor.value, floor));
    };

    /**
     * At sample 192 the delay line holds inputs 81 to 191, with tags on samples 85 and 150. A cp_len of 8 or 24 sets a
     * lag of 103 or 119 samples. The delay line keeps its newest inputs, and each tag leaves on its own sample. The
     * shorter lag drops inputs 81 to 88, and the tag on sample 85 leaves on the next output.
     */
    "a change of the lag keeps the held inputs and their tags"_test = [&clean, floor] {
        const std::vector<gr::Tag> inTags{gr::Tag{85UZ, gr::property_map{{"mark", std::string("dropped")}}}, //
            gr::Tag{150UZ, gr::property_map{{"mark", std::string("held")}}}, gr::Tag{500UZ, gr::property_map{{"mark", std::string("later")}}}};
        constexpr std::size_t      kSplit = 192UZ;
        for (const gr::Size_t prefix : {gr::Size_t{8U}, gr::Size_t{24U}}) {
            SchmidlCoxSync block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}});
            Drive          drive{block, std::span<const CF>(clean), shim::TagCursor(std::span<const gr::Tag>(inTags))};
            drive.to(kSplit, 64UZ);
            const std::size_t before = block.delaySamples();
            std::ignore              = block.settings().setStaged({{"cp_len", prefix}});
            std::ignore              = block.settings().applyStagedParameters();
            const std::size_t after  = block.delaySamples();
            drive.to(clean.size(), 64UZ);
            drive.finish();

            const std::size_t         kept  = std::min(before, after);
            const std::size_t         zeros = after - kept;
            const std::span<const CF> out   = drive.out.samples;
            expect(eq(after, 3UZ * static_cast<std::size_t>(kFft) / 2UZ + static_cast<std::size_t>(prefix) - 1UZ));
            expect(std::ranges::all_of(out.subspan(kSplit, zeros), [](const CF& sample) { return sample == CF{}; })) << std::format("cp_len {}: zeros ahead of the kept inputs", prefix);
            expect(std::ranges::equal(out.subspan(kSplit + zeros), std::span<const CF>(clean).subspan(kSplit - kept, clean.size() - kSplit - zeros))) << std::format("cp_len {}: each kept input comes out once, in order", prefix);

            std::vector<std::string> marks;
            for (const std::string& entry : tagList(drive.out.tags)) {
                if (entry.starts_with("mark=")) {
                    marks.push_back(entry);
                }
            }
            const std::vector<std::string> expected{std::format("mark=dropped@{}", std::max(kSplit, 85UZ + after)), std::format("mark=held@{}", 150UZ + after), std::format("mark=later@{}", 500UZ + after)};
            expect(marks == expected) << std::format("cp_len {}: {} against {}", prefix, marks, expected);
        }
    };

    "the counters count the whole run across a change of the lag"_test = [&clean, floor] {
        std::vector<CF> twice = clean;
        twice.insert(twice.end(), clean.begin(), clean.end());
        SchmidlCoxSync block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}});
        Drive          drive{block, std::span<const CF>(twice), shim::TagCursor(std::span<const gr::Tag>{})};
        drive.to(clean.size(), 64UZ);
        expect(eq(block.nTriggers(), std::uint64_t{1})) << "the first frame";
        std::ignore = block.settings().setStaged({{"cp_len", gr::Size_t{15U}}});
        std::ignore = block.settings().applyStagedParameters();
        drive.to(twice.size(), 64UZ);
        expect(eq(block.nTriggers(), std::uint64_t{2})) << "the second frame adds to the first";
        expect(eq(std::ranges::count_if(drive.out.tags, [](const gr::Tag& tag) { return tag.map.contains(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey())); }), std::ptrdiff_t{2})) << "both triggers are output";
    };

    "start() throws nothing after a refused setting"_test = [floor] {
        SchmidlCoxSync block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}});
        expect(apply(block, {{"cp_len", gr::Size_t{80U}}}).contains("cp_len")) << "the settings application refuses it";
        expect(nothrow([&block] { block.start(); })) << "a restart throws nothing";
    };

    "a block left at its default settings finds the frame"_test = [&clean] {
        SchmidlCoxSync  block = make<SchmidlCoxSync>({});
        const Detection found = sync(block, std::span<const CF>(clean), 64UZ);
        expect(eq(found.biases.size(), 1UZ));
        if (!found.biases.empty()) {
            expect(le(4 * std::abs(found.biases[0UZ]), static_cast<std::int64_t>(kCp)));
        }
    };

    /**
     * A tag on sample 10, ahead of the frame, sets `threshold`. The framework refuses 1.5, a value outside its limits,
     * and the run completes with the samples and the trigger of a run without the tag. In a third run the block takes
     * 0.65 from the tag.
     */
    "a threshold outside its limits on a tag is refused, and the run goes on"_test = [&clean, floor] {
        using gr::blocks::testing::ProcessFunction;
        struct Run {
            bool                     completed = false;
            std::vector<CF>          samples{};
            std::vector<std::size_t> triggers{};
            float                    member   = 0.f;
            float                    reported = 0.f;
        };
        const auto run = [&clean, floor](std::optional<float> level) {
            gr::Graph graph;
            auto&     source = graph.emplaceBlock<gr::blocks::testing::TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(clean.size())}, {"values", gr::Tensor<CF>(clean.begin(), clean.end())}, {"mark_tag", false}});
            if (level.has_value()) {
                source._tags = {gr::Tag{10UZ, gr::property_map{{"threshold", *level}}}};
            }
            // threshold stays out of the constructor's settings, at its default of 0.6, and the tag sets it.
            auto& detect = graph.emplaceBlock<SchmidlCoxSync>({{"r_floor", floor}});
            auto& sink   = graph.emplaceBlock<gr::blocks::testing::TagSink<CF, ProcessFunction::USE_PROCESS_BULK>>({{"log_samples", true}, {"log_tags", true}});
            expect(graph.connect<"out", "in">(source, detect).has_value());
            expect(graph.connect<"out", "in">(detect, sink).has_value());

            gr::scheduler::Simple scheduler;
            expect(scheduler.exchange(std::move(graph)).has_value());
            Run result;
            result.completed = scheduler.runAndWait().has_value();
            result.samples.assign(sink._samples.begin(), sink._samples.end());
            for (const gr::Tag& tag : sink._tags) {
                if (tag.map.contains(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey()))) {
                    result.triggers.push_back(tag.index);
                }
            }
            result.member                                = detect.threshold.value;
            const std::optional<gr::pmt::Value> reported = detect.settings().get("threshold");
            result.reported                              = reported.has_value() && reported->get_if<float>() != nullptr ? *reported->get_if<float>() : 0.f;
            return result;
        };

        const Run reference = run(std::nullopt);
        expect(reference.completed);
        expect(eq(reference.triggers.size(), 1UZ));

        const Run refused = run(1.5f);
        expect(refused.completed) << "the refusal ends no run";
        expect(eq(refused.member, 0.6f)) << "the member keeps the accepted value";
        expect(eq(refused.reported, 0.6f)) << "settings().get() returns the accepted value";
        expect(refused.samples == reference.samples) << "the delayed stream";
        expect(refused.triggers == reference.triggers) << std::format("the trigger: {} against {}", refused.triggers, reference.triggers);

        const Run taken = run(0.65f);
        expect(taken.completed);
        expect(eq(taken.member, 0.65f)) << "the tag reaches the setting";
    };

    /**
     * Two frames, and a reset to defaults at sample 240. The block has confirmed the first frame there and has not yet
     * output its trigger's sample. The block still holds the input tag on sample 150. From sample 240 on, the samples
     * match those of a block started at that sample: the zeros of an empty delay line, then the input. The held input
     * tag leaves on sample 240, the held detection counts as late, and the second frame's trigger follows. The counters
     * continue.
     */
    "a reset to defaults mid-run restarts the delay line and publishes the held tags on the next output"_test = [&clean, floor] {
        std::vector<CF> twice = clean;
        twice.insert(twice.end(), clean.begin(), clean.end());
        constexpr std::size_t      kSplit = 240UZ;
        const std::vector<gr::Tag> inTags{gr::Tag{150UZ, gr::property_map{{"mark", std::string("held")}}}};
        const gr::property_map     settings{{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}};

        // The settings the constructor names are the ones the reset puts back.
        SchmidlCoxSync block(settings);
        block.init(std::make_shared<gr::Sequence>());
        block.start();
        Drive drive{block, std::span<const CF>(twice), shim::TagCursor(std::span<const gr::Tag>(inTags))};
        drive.to(kSplit, 64UZ);
        expect(eq(block.nTriggers(), std::uint64_t{1})) << "the first frame is confirmed";
        expect(drive.out.tags.empty()) << "and the block holds its trigger and the input tag";
        expect(nothrow([&block] { block.settings().resetDefaults(); })) << "the reset moves no setting";
        expect(eq(block.nTriggers(), std::uint64_t{1})) << "the counters continue";
        expect(eq(block.nLate(), std::uint64_t{1})) << "the held detection is late";
        drive.to(twice.size(), 64UZ);

        SchmidlCoxSync fresh = make<SchmidlCoxSync>(settings);
        Drive          started{fresh, std::span<const CF>(twice), shim::TagCursor(std::span<const gr::Tag>{}), {}, kSplit};
        started.to(twice.size(), 64UZ);

        const std::span<const CF> after = std::span<const CF>(drive.out.samples).subspan(kSplit);
        expect(std::ranges::all_of(after.first(block.delaySamples()), [](const CF& sample) { return sample == CF{}; })) << "zeros in place of the inputs the delay line held";
        expect(std::ranges::equal(after, started.out.samples)) << "the output after the reset";
        std::vector<std::string> expected{std::format("mark=held@{}", kSplit)};
        std::ranges::copy(tagList(started.out.tags), std::back_inserter(expected));
        expect(eq(fresh.nTriggers(), std::uint64_t{1})) << "the second frame";
        expect(tagList(drive.out.tags) == expected) << std::format("the held tag, then the second frame's trigger: {} against {}", tagList(drive.out.tags), expected);
        expect(eq(block.nTriggers(), std::uint64_t{2}));
        expect(eq(block.nLate(), std::uint64_t{1}));
    };

    /**
     * A reset to defaults at sample 240 moves the held input tag on sample 150 to the next output. A cp_len of 8 then
     * shortens the lag from 111 to 103 samples. The tag still leaves on sample 240, the first output after both.
     */
    "a held tag leaves on the next output after a reset to defaults and a shorter lag"_test = [&clean, floor] {
        constexpr std::size_t      kSplit = 240UZ;
        const std::vector<gr::Tag> inTags{gr::Tag{150UZ, gr::property_map{{"mark", std::string("held")}}}};

        SchmidlCoxSync block({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}});
        block.init(std::make_shared<gr::Sequence>());
        block.start();
        Drive drive{block, std::span<const CF>(clean), shim::TagCursor(std::span<const gr::Tag>(inTags))};
        drive.to(kSplit, 64UZ);
        expect(drive.out.tags.empty()) << "the block holds the input tag";
        expect(nothrow([&block] { block.settings().resetDefaults(); }));
        expect(apply(block, {{"cp_len", gr::Size_t{8U}}}).empty()) << "the shorter lag applies";
        expect(eq(block.delaySamples(), 3UZ * static_cast<std::size_t>(kFft) / 2UZ + 8UZ - 1UZ));
        drive.to(clean.size(), 64UZ);
        drive.finish();

        std::vector<std::string> marks;
        for (const std::string& entry : tagList(drive.out.tags)) {
            if (entry.starts_with("mark=")) {
                marks.push_back(entry);
            }
        }
        const std::vector<std::string> expected{std::format("mark=held@{}", kSplit)};
        expect(marks == expected) << std::format("{} against {}", marks, expected);
    };

    /**
     * Two frames a stream apart, under two values of `min_gap`: the largest whose dead time stays below 2^32 samples,
     * and the least whose dead time passes 2^32. The first trigger lies past 2^32 samples less the shorter dead time.
     * On every target each dead time refuses the second frame.
     */
    "a dead time on either side of 2^32 samples refuses the second of two frames on every target"_test = [&clean, floor] {
        constexpr std::uint64_t kSymbol = std::uint64_t{kFft} + std::uint64_t{kCp};
        constexpr auto          kBelow  = static_cast<gr::Size_t>((kWrap - 1U) / kSymbol);
        constexpr auto          kAbove  = static_cast<gr::Size_t>(kWrap / kSymbol + 1U);
        static_assert(kBelow * kSymbol < kWrap && kAbove * kSymbol > kWrap);
        static_assert(kLead - kCp / 4U > kWrap - kBelow * kSymbol, "a trigger lands within a quarter prefix of its frame");

        std::vector<CF> twice = clean;
        twice.insert(twice.end(), clean.begin(), clean.end());
        for (const gr::Size_t gap : {kBelow, kAbove}) {
            SchmidlCoxSync  block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}, {"min_gap", gap}});
            const Detection found = sync(block, std::span<const CF>(twice), 64UZ);
            expect(eq(found.biases.size(), 1UZ)) << std::format("min_gap {}: a dead time of {} samples refuses the second frame", gap, std::uint64_t{gap} * kSymbol);
        }
    };

    /**
     * Two copies of the clean frame run with 2^32 at each input from `fft_len/2` before the first frame's first sample
     * to `delaySamples()` past its preamble symbol. A 32-bit `std::size_t` wraps there. Each run places both triggers
     * where the run from index 0 places them and counts nothing late.
     */
    "a frame at the 32-bit wrap of the stream index keeps its trigger on every target"_test = [&clean, floor] {
        const gr::property_map settings{{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}};
        const std::string_view trigger = gr::tag::TRIGGER_NAME.shortKey();
        std::vector<CF>        twice   = clean;
        twice.insert(twice.end(), clean.begin(), clean.end());

        SchmidlCoxSync reference = make<SchmidlCoxSync>(settings);
        const auto     expected  = offsetsOf(shim::run<CF>(reference, std::span<const CF>(twice), 64UZ).tags, trigger, 0UZ);
        expect(eq(expected.size(), 2UZ)) << "the run from index 0 finds both frames";

        const std::size_t        symbol = static_cast<std::size_t>(kFft + kCp);
        std::vector<std::string> misplaced;
        std::vector<std::size_t> late;
        for (std::size_t wrapAt = kLead - kFft / 2U; wrapAt < kLead + symbol + reference.delaySamples(); ++wrapAt) {
            const std::size_t start  = aheadOfWrap(wrapAt);
            SchmidlCoxSync    block  = make<SchmidlCoxSync>(settings);
            const auto        result = shim::run<CF>(block, std::span<const CF>(twice), 64UZ, 1UZ, {}, start);
            if (const auto found = offsetsOf(result.tags, trigger, start); found != expected) {
                misplaced.push_back(std::format("{}: {}", wrapAt, found));
            }
            if (block.nLate() != 0ULL) {
                late.push_back(wrapAt);
            }
        }
        expect(misplaced.empty()) << std::format("triggers against {}, by the input at 2^32: {}", expected, misplaced);
        expect(late.empty()) << std::format("a late detection with 2^32 at inputs {}", late);
    };

    /**
     * The second copy of the frame starts at 2^32, and the dead time of the first trigger runs across the wrap. Under
     * `min_gap` 1 it ends before the second frame. A dead time longer than one copy covers the second frame. Each run
     * places its triggers where the run from index 0 places them.
     */
    "a dead time across the 32-bit wrap of the stream index ends where it does from index 0"_test = [&clean, floor] {
        const std::string_view trigger = gr::tag::TRIGGER_NAME.shortKey();
        std::vector<CF>        twice   = clean;
        twice.insert(twice.end(), clean.begin(), clean.end());
        const auto longer = static_cast<gr::Size_t>(clean.size() / static_cast<std::size_t>(kFft + kCp) + 1UZ);

        for (const auto& [gap, triggers] : {std::pair{gr::Size_t{1U}, 2UZ}, std::pair{longer, 1UZ}}) {
            const gr::property_map settings{{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}, {"min_gap", gap}};
            SchmidlCoxSync         reference = make<SchmidlCoxSync>(settings);
            const auto             expected  = offsetsOf(shim::run<CF>(reference, std::span<const CF>(twice), 64UZ).tags, trigger, 0UZ);
            expect(eq(expected.size(), triggers)) << std::format("min_gap {}: the run from index 0", gap);

            const std::size_t start  = aheadOfWrap(clean.size());
            SchmidlCoxSync    block  = make<SchmidlCoxSync>(settings);
            const auto        result = shim::run<CF>(block, std::span<const CF>(twice), 64UZ, 1UZ, {}, start);
            const auto        found  = offsetsOf(result.tags, trigger, start);
            expect(found == expected) << std::format("min_gap {}: triggers at {} against {}", gap, found, expected);
        }
    };

    /**
     * An input tag on the frame's first sample leaves `delaySamples()` later. The call that hands the tag over ends
     * before input `handedTo`. 2^32 falls at each input from `handedTo + 1` to the tag's output. The test spans hand a
     * call only the tags below the end of its window, and that end must lie below the wrap.
     */
    "an input tag whose output passes the 32-bit wrap of the stream index keeps its sample"_test = [&clean, floor] {
        constexpr std::size_t  kChunk = 8UZ;
        const gr::property_map settings{{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}};
        const std::size_t      handedTo = (kLead / kChunk + 1UZ) * kChunk;
        const std::size_t      delay    = make<SchmidlCoxSync>(settings).delaySamples();
        const std::vector      expected{kLead + delay};

        std::vector<std::string> misplaced;
        for (std::size_t wrapAt = handedTo + 1UZ; wrapAt <= kLead + delay; ++wrapAt) {
            const std::size_t          start = aheadOfWrap(wrapAt);
            const std::vector<gr::Tag> inTags{gr::Tag{start + kLead, gr::property_map{{"mark", std::string("frame")}}}};
            SchmidlCoxSync             block  = make<SchmidlCoxSync>(settings);
            const auto                 result = shim::run<CF>(block, std::span<const CF>(clean), kChunk, 1UZ, std::span<const gr::Tag>(inTags), start);
            if (const auto found = offsetsOf(result.tags, "mark", start); found != expected) {
                misplaced.push_back(std::format("{}: {}", wrapAt, found));
            }
        }
        expect(misplaced.empty()) << std::format("the tag against {}, by the input at 2^32: {}", expected, misplaced);
    };
};

} // namespace qa_ofdm_sync

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
