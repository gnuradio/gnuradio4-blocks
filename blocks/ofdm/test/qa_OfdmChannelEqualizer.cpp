#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/algorithm/channel/DelayProfile.hpp>
#include <gnuradio-4.0/channel/Awgn.hpp>
#include <gnuradio-4.0/channel/CarrierImpairments.hpp>
#include <gnuradio-4.0/filter/FirFilter.hpp>
#include <gnuradio-4.0/ofdm/CarrierAllocator.hpp>
#include <gnuradio-4.0/ofdm/ChannelEqualizer.hpp>
#include <gnuradio-4.0/ofdm/CyclicPrefix.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_ofdm_equalizer {

using gr::blocks::channel::AwgnChannel;
using gr::blocks::channel::PhaseNoise;
using gr::blocks::filter::FirFilter;
using gr::blocks::ofdm::CarrierAllocator;
using gr::blocks::ofdm::ChannelEqualizer;
using gr::blocks::ofdm::CpInsert;
using gr::blocks::ofdm::CpRemove;
using CF       = std::complex<float>;
namespace shim = gr::blocks::testing::span;

constexpr gr::Size_t  kFft   = 64U;
constexpr gr::Size_t  kCp    = 16U;
constexpr gr::Size_t  kSync  = 2U; ///< a Schmidl-Cox preamble for timing, then the known symbol least squares reads
constexpr std::size_t kFrame = 20UZ;
constexpr std::size_t kSplit = 12UZ; ///< the two sync symbols and ten data symbols ahead of a change

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
[[nodiscard]] std::string apply(ChannelEqualizer& block, const gr::property_map& settings) {
    std::ignore = block.settings().setStaged(settings);
    try {
        return block.settings().applyStagedParameters().refusal;
    } catch (const gr::exception& e) {
        return std::string(e.what());
    }
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

[[nodiscard]] CF qpsk(std::uint32_t& state) {
    state          = state * 1664525U + 1013904223U;
    const float re = ((state >> 16U) & 1U) != 0U ? 0.70710678f : -0.70710678f;
    const float im = ((state >> 17U) & 1U) != 0U ? 0.70710678f : -0.70710678f;
    return CF(re, im);
}

/// The known full symbol for least squares at @p fftLength bins. Every occupied carrier carries a point.
[[nodiscard]] std::vector<CF> channelSounding(gr::Size_t fftLength = kFft) {
    std::vector<CF> word(fftLength, CF{});
    std::uint32_t   state = 987654321U;
    for (const std::int32_t carrier : dataCarriers()) {
        word[gr::ofdm::CarrierMap::binOf(fftLength, carrier).value()] = qpsk(state);
    }
    for (const std::int32_t carrier : pilotCarriers()) {
        word[gr::ofdm::CarrierMap::binOf(fftLength, carrier).value()] = qpsk(state);
    }
    return word;
}

[[nodiscard]] std::vector<CF> pilotCycle() { return {CF(1.f, 0.f), CF(1.f, 0.f), CF(1.f, 0.f), CF(-1.f, 0.f)}; }

/// Six values on four pilots. Consecutive data symbols read different values, by the symbol's data index.
[[nodiscard]] std::vector<CF> rotatingPilots() { return {CF(1.f, 0.f), CF(1.f, 0.f), CF(1.f, 0.f), CF(-1.f, 0.f), CF(0.f, 1.f), CF(0.f, -1.f)}; }

[[nodiscard]] gr::property_map numerology(const std::vector<CF>& pilots = pilotCycle()) {
    const auto      preamble = gr::ofdm::schmidlCoxPreamble(static_cast<std::size_t>(kFft), 52UZ, 0xC0FFEEULL).value();
    const auto      sounding = channelSounding();
    std::vector<CF> words(preamble.begin(), preamble.end());
    words.insert(words.end(), sounding.begin(), sounding.end());
    return {{"fft_len", kFft}, {"data_carriers", dataCarriers()}, {"pilot_carriers", pilotCarriers()}, //
        {"pilot_symbols", interleave(std::span<const CF>(pilots))}, {"sync_words", interleave(words)}, {"frame_len", static_cast<gr::Size_t>(kFrame)}};
}

[[nodiscard]] gr::property_map equalizerSettings(std::string tracking) {
    return {{"fft_len", kFft}, {"data_carriers", dataCarriers()}, {"pilot_carriers", pilotCarriers()}, //
        {"pilot_symbols", interleave(pilotCycle())}, {"sync_word", interleave(channelSounding())},     //
        {"n_sync", kSync}, {"sync_index", gr::Size_t{1U}}, {"tracking", std::move(tracking)}};
}

struct Modulated {
    std::vector<CF>              payload{};
    std::vector<gr::DataSet<CF>> records{};
};

/// @brief The records an allocator with @p settings makes of @p payload, the end of the stream included.
[[nodiscard]] std::vector<gr::DataSet<CF>> allocate(const gr::property_map& settings, std::span<const CF> payload) {
    CarrierAllocator allocator = make<CarrierAllocator>(settings);

    std::vector<gr::DataSet<CF>>      records;
    std::vector<gr::DataSet<CF>>      scratch(256UZ);
    shim::InputSpan<CF>               inSpan(payload, 0UZ);
    shim::OutputSpan<gr::DataSet<CF>> outSpan{std::span<gr::DataSet<CF>>(scratch)};
    std::ignore = allocator.processBulk(inSpan, outSpan);
    for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
        records.push_back(std::move(scratch[k]));
    }
    shim::InputSpan<CF>               tail(payload.last(payload.size() - inSpan.consumed), inSpan.consumed);
    shim::OutputSpan<gr::DataSet<CF>> outTail{std::span<gr::DataSet<CF>>(scratch)};
    std::ignore = allocator.processEpilogue(tail, outTail);
    for (std::size_t k = 0UZ; k < outTail.count; ++k) {
        records.push_back(std::move(scratch[k]));
    }
    return records;
}

/// @brief One frame of QPSK data through the allocator, with the payload it was given and the symbols it produced.
[[nodiscard]] Modulated modulate(std::size_t frames = 1UZ, const std::vector<CF>& pilots = pilotCycle()) {
    Modulated     result;
    std::uint32_t state = 24680U;
    result.payload.resize(frames * kFrame * dataCarriers().size());
    for (CF& symbol : result.payload) {
        symbol = qpsk(state);
    }
    result.records = allocate(numerology(pilots), std::span<const CF>(result.payload));
    return result;
}

struct Stream {
    std::vector<CF>      samples{};
    std::vector<gr::Tag> tags{};
};

[[nodiscard]] Stream toStream(std::span<const gr::DataSet<CF>> records) {
    CpInsert                         cp = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{kCp}}});
    Stream                           result;
    std::vector<CF>                  body(1UZ << 16);
    shim::InputSpan<gr::DataSet<CF>> inSpan(records, 0UZ);
    shim::OutputSpan<CF>             outSpan{std::span<CF>(body), 0UZ, &result.tags};
    std::ignore = cp.processBulk(inSpan, outSpan);
    result.samples.assign(body.begin(), body.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
    return result;
}

[[nodiscard]] std::vector<gr::DataSet<CF>> toRecords(const Stream& stream) {
    CpRemove block = make<CpRemove>({{"fft_len", kFft}, {"cp_len", std::vector<gr::Size_t>{kCp}}, {"n_sync", kSync}, {"frame_len", static_cast<gr::Size_t>(kFrame)}});

    std::vector<gr::DataSet<CF>>      records;
    std::vector<gr::DataSet<CF>>      scratch(256UZ);
    shim::InputSpan<CF>               inSpan(std::span<const CF>(stream.samples), 0UZ, std::span<const gr::Tag>(stream.tags));
    shim::OutputSpan<gr::DataSet<CF>> outSpan{std::span<gr::DataSet<CF>>(scratch)};
    std::ignore = block.processBulk(inSpan, outSpan);
    for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
        records.push_back(std::move(scratch[k]));
    }
    return records;
}

struct Equalized {
    std::vector<gr::DataSet<CF>>    symbols{};
    std::vector<gr::DataSet<float>> estimates{};
};

[[nodiscard]] Equalized equalize(ChannelEqualizer& block, std::span<const gr::DataSet<CF>> records, bool wantEstimate = false) {
    Equalized                       result;
    std::vector<gr::DataSet<CF>>    scratch(256UZ);
    std::vector<gr::DataSet<float>> estimateScratch(16UZ);

    shim::InputSpan<gr::DataSet<CF>>     inSpan(records, 0UZ);
    shim::OutputSpan<gr::DataSet<CF>>    outSpan{std::span<gr::DataSet<CF>>(scratch)};
    shim::OutputSpan<gr::DataSet<float>> channelSpan(wantEstimate ? std::span<gr::DataSet<float>>(estimateScratch) : std::span<gr::DataSet<float>>{}, 0UZ, nullptr, wantEstimate);
    std::ignore = block.processBulk(inSpan, outSpan, channelSpan);
    for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
        result.symbols.push_back(std::move(scratch[k]));
    }
    for (std::size_t k = 0UZ; k < channelSpan.count; ++k) {
        result.estimates.push_back(std::move(estimateScratch[k]));
    }
    return result;
}

[[nodiscard]] float metaFloat(const gr::DataSet<CF>& record, std::string_view key) {
    const auto entry = record.meta_information[0UZ].find(std::pmr::string(key));
    if (entry == record.meta_information[0UZ].end()) {
        return 0.f;
    }
    const auto* value = entry->second.get_if<float>();
    return value == nullptr ? 0.f : *value;
}

/// @brief The error vector magnitude of a whole run, in dB, against the nearest QPSK point.
[[nodiscard]] double errorVectorDb(std::span<const gr::DataSet<CF>> symbols) {
    const gr::digital::Constellation<float> points = gr::digital::Constellation<float>::qpsk();
    double                                  error  = 0.;
    double                                  signal = 0.;
    for (const gr::DataSet<CF>& symbol : symbols) {
        for (const CF& value : symbol.signal_values) {
            const CF nearest = points.point(points.hardDecision(value));
            error += static_cast<double>(std::norm(value - nearest));
            signal += static_cast<double>(std::norm(nearest));
        }
    }
    return 10. * std::log10(std::max(error / signal, 1e-30));
}

[[nodiscard]] std::size_t symbolErrors(std::span<const gr::DataSet<CF>> symbols, std::span<const CF> payload) {
    const gr::digital::Constellation<float> points = gr::digital::Constellation<float>::qpsk();
    std::size_t                             wrong  = 0UZ;
    std::size_t                             at     = 0UZ;
    for (const gr::DataSet<CF>& symbol : symbols) {
        for (const CF& value : symbol.signal_values) {
            if (at >= payload.size()) {
                return wrong;
            }
            if (points.hardDecision(value) != points.hardDecision(payload[at])) {
                ++wrong;
            }
            ++at;
        }
    }
    return wrong;
}

/// The static multipath profile. It has four paths over nine samples. At 64 samples per symbol that is a delay spread
/// of an eighth of the transform, well inside the prefix.
[[nodiscard]] std::vector<CF> multipathTaps() {
    constexpr double          rate = static_cast<double>(kFft);
    const std::vector<double> delays{0., 2. / rate, 5. / rate, 9. / rate};
    const std::vector<double> powers{0., -3., -6., -9.};
    return gr::channel::tapsFromProfile(std::span<const double>(delays), std::span<const double>(powers), rate, true);
}

/// @brief The records' stream after the static multipath, with no noise.
[[nodiscard]] Stream throughMultipath(std::span<const gr::DataSet<CF>> records) {
    const Stream      clean         = toStream(records);
    FirFilter<CF, CF> channelFilter = make<FirFilter<CF, CF>>({{"taps", multipathTaps()}});
    Stream            faded;
    faded.tags    = clean.tags;
    faded.samples = shim::run<CF>(channelFilter, std::span<const CF>(clean.samples), 4096UZ).samples;
    return faded;
}

/// @brief One frame's records with data symbol `d` turned by `2 pi k (d + 1) step / fft_len` on carrier `k`. A timing
/// drift of @p step samples per symbol gives this phase slope. The sync symbols stay as they are.
[[nodiscard]] std::vector<gr::DataSet<CF>> drifted(std::span<const gr::DataSet<CF>> records, double step) {
    std::vector<gr::DataSet<CF>> result(records.begin(), records.end());
    for (std::size_t s = static_cast<std::size_t>(kSync); s < result.size(); ++s) {
        const double delay = step * static_cast<double>(s - static_cast<std::size_t>(kSync) + 1UZ);
        for (std::size_t bin = 0UZ; bin < result[s].signal_values.size(); ++bin) {
            const int    carrier = gr::ofdm::CarrierMap::carrierOf(static_cast<std::size_t>(kFft), bin).value();
            const double phase   = 2. * std::numbers::pi * static_cast<double>(carrier) * delay / static_cast<double>(kFft);
            result[s].signal_values[bin] *= CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
        }
    }
    return result;
}

const boost::ut::suite<"OFDM channel equalizer"> _equalizer = [] {
    using namespace boost::ut;

    "on an ideal channel the data comes back as it went in"_test = [] {
        const Modulated  sent  = modulate();
        ChannelEqualizer block = make<ChannelEqualizer>(equalizerSettings("cpe"));
        const Equalized  got   = equalize(block, std::span<const gr::DataSet<CF>>(sent.records), true);

        expect(eq(got.symbols.size(), kFrame)) << "one output record per data symbol, and none for the sync words";
        expect(eq(block.nSkipped(), std::uint64_t{1})) << "the preamble is passed over; the sounding symbol is read";
        expect(eq(block.nUntrained(), std::uint64_t{0}));
        expect(eq(block.nEstimates(), std::uint64_t{1}));
        expect(eq(got.estimates.size(), 1UZ)) << "one channel-estimate record per frame";

        const auto  data  = dataCarriers();
        double      worst = 0.;
        std::size_t at    = 0UZ;
        for (const gr::DataSet<CF>& symbol : got.symbols) {
            expect(eq(symbol.signal_values.size(), data.size()));
            for (std::size_t k = 0UZ; k < data.size(); ++k) {
                worst = std::max(worst, static_cast<double>(std::abs(symbol.signal_values[k] - sent.payload[at + k])));
                expect(symbol.axis_values[0UZ][k] == CF(static_cast<float>(data[k]), 0.f)) << "the axis states the carrier each value came from";
            }
            at += data.size();
        }
        std::println("ideal channel: worst data-carrier error {:.3e}, EVM {:.1f} dB", worst, errorVectorDb(std::span<const gr::DataSet<CF>>(got.symbols)));
        expect(lt(worst, 1e-5)) << "the data reproduces to float rounding";

        // the estimate of an ideal channel is unit magnitude and zero phase on every occupied carrier
        const gr::DataSet<float>& estimate = got.estimates[0UZ];
        expect(eq(estimate.signal_values.size(), 2UZ * (data.size() + pilotCarriers().size())));
        for (std::size_t k = 0UZ; k < estimate.signal_values.size() / 2UZ; ++k) {
            expect(lt(std::abs(estimate.signal_values[k] - 1.f), 1e-5f));
        }
    };

    "zero forcing inverts static multipath, and noise raises the EVM within its bound"_test = [] {
        const Modulated sent  = modulate();
        const Stream    faded = throughMultipath(std::span<const gr::DataSet<CF>>(sent.records));

        {
            ChannelEqualizer block = make<ChannelEqualizer>(equalizerSettings("cpe"));
            const Equalized  got   = equalize(block, std::span<const gr::DataSet<CF>>(toRecords(faded)));
            const double     evm   = errorVectorDb(std::span<const gr::DataSet<CF>>(got.symbols));
            std::println("static multipath alone (0/-3/-6/-9 dB at 0/2/5/9 samples): EVM {:.1f} dB, {} symbol errors", evm, symbolErrors(std::span<const gr::DataSet<CF>>(got.symbols), std::span<const CF>(sent.payload)));
            expect(lt(evm, -60.)) << "the prefix makes the channel circular and zero forcing inverts it outright";
            expect(eq(symbolErrors(std::span<const gr::DataSet<CF>>(got.symbols), std::span<const CF>(sent.payload)), 0UZ));
        }

        double meanPower = 0.;
        for (const CF& sample : faded.samples) {
            meanPower += static_cast<double>(std::norm(sample));
        }
        meanPower /= static_cast<double>(faded.samples.size());

        for (const double snrDb : {20., 15.}) {
            AwgnChannel<CF> noise = make<AwgnChannel<CF>>({{"noise_power", meanPower / std::pow(10., snrDb / 10.)}, {"seed", std::uint64_t{7}}});
            Stream          noisy;
            noisy.tags    = faded.tags;
            noisy.samples = shim::run<CF>(noise, std::span<const CF>(faded.samples), 4096UZ).samples;

            ChannelEqualizer  block = make<ChannelEqualizer>(equalizerSettings("cpe"));
            const Equalized   got   = equalize(block, std::span<const gr::DataSet<CF>>(toRecords(noisy)));
            const double      evm   = errorVectorDb(std::span<const gr::DataSet<CF>>(got.symbols));
            const std::size_t wrong = symbolErrors(std::span<const gr::DataSet<CF>>(got.symbols), std::span<const CF>(sent.payload));
            std::println("the same multipath at {:.0f} dB SNR: EVM {:.1f} dB, {} of {} symbols wrong", snrDb, evm, wrong, kFrame * dataCarriers().size());
            expect(lt(evm, -snrDb + 12.)) << "zero forcing costs noise enhancement at the profile's nulls, bounded here";
        }
    };

    "cpe_interp keeps the least-squares estimate of static multipath"_test = [] {
        const Modulated sent    = modulate();
        const auto      records = toRecords(throughMultipath(std::span<const gr::DataSet<CF>>(sent.records)));

        ChannelEqualizer phaseOnly    = make<ChannelEqualizer>(equalizerSettings("cpe"));
        ChannelEqualizer interpolated = make<ChannelEqualizer>(equalizerSettings("cpe_interp"));
        const Equalized  reference    = equalize(phaseOnly, std::span<const gr::DataSet<CF>>(records));
        const Equalized  got          = equalize(interpolated, std::span<const gr::DataSet<CF>>(records));
        expect(eq(got.symbols.size(), kFrame));
        expect(eq(reference.symbols.size(), kFrame));

        double gap = 0.;
        for (std::size_t s = 0UZ; s < std::min(got.symbols.size(), reference.symbols.size()); ++s) {
            for (std::size_t k = 0UZ; k < got.symbols[s].signal_values.size(); ++k) {
                gap = std::max(gap, static_cast<double>(std::abs(got.symbols[s].signal_values[k] - reference.symbols[s].signal_values[k])));
            }
        }
        const std::size_t wrong = symbolErrors(std::span<const gr::DataSet<CF>>(got.symbols), std::span<const CF>(sent.payload));
        std::println("static multipath, cpe_interp against cpe: {} symbol errors, largest difference {:.2e}", wrong, gap);
        expect(eq(wrong, 0UZ)) << "the update keeps the channel's shape between the pilots";
        expect(lt(gap, 1e-4)) << "every pilot ratio of a static channel is 1, and the update leaves the estimate where least squares put it";
    };

    "phase noise is held by cpe tracking and is not held without it"_test = [] {
        const Modulated sent  = modulate();
        const Stream    clean = toStream(std::span<const gr::DataSet<CF>>(sent.records));

        const auto through = [&clean](double linewidth, const std::string& tracking) {
            PhaseNoise<CF> jitter = make<PhaseNoise<CF>>({{"sample_rate", static_cast<float>(kFft)}, {"linewidth", linewidth}, {"seed", std::uint64_t{2026}}});
            Stream         noisy;
            noisy.tags    = clean.tags;
            noisy.samples = shim::run<CF>(jitter, std::span<const CF>(clean.samples), 4096UZ).samples;

            ChannelEqualizer block = make<ChannelEqualizer>(equalizerSettings(tracking));
            return equalize(block, std::span<const gr::DataSet<CF>>(toRecords(noisy)));
        };

        constexpr double kLinewidth = 0.02; // hertz, against a subcarrier spacing of one, a slow oscillator
        const Equalized  tracked    = through(kLinewidth, "cpe");
        const Equalized  untracked  = through(kLinewidth, "none");

        const double      trackedEvm     = errorVectorDb(std::span<const gr::DataSet<CF>>(tracked.symbols));
        const double      untrackedEvm   = errorVectorDb(std::span<const gr::DataSet<CF>>(untracked.symbols));
        const std::size_t trackedWrong   = symbolErrors(std::span<const gr::DataSet<CF>>(tracked.symbols), std::span<const CF>(sent.payload));
        const std::size_t untrackedWrong = symbolErrors(std::span<const gr::DataSet<CF>>(untracked.symbols), std::span<const CF>(sent.payload));

        float lastCpe = 0.f;
        for (const gr::DataSet<CF>& symbol : tracked.symbols) {
            lastCpe = metaFloat(symbol, "cpe_rad");
        }
        std::println("phase noise of {} Hz against a 1 Hz spacing over {} symbols: cpe EVM {:.1f} dB with {} symbol errors, no tracking EVM {:.1f} dB with {} errors; the last symbol was turned by {:+.3f} rad", //
            kLinewidth, kFrame, trackedEvm, trackedWrong, untrackedEvm, untrackedWrong, lastCpe);

        // The symbol errors decide the comparison. Once the phase turns past a decision boundary, the nearest point is
        // a different point, and the error vector stops growing while every decision is wrong. cpe removes the phase
        // that changes from symbol to symbol. The phase noise within a symbol remains. It turns each carrier against
        // its neighbors, and no common phase removes it.
        expect(lt(trackedEvm, untrackedEvm)) << "tracking lowers the error vector";
        expect(eq(trackedWrong, 0UZ)) << "cpe holds the constellation";
        expect(gt(untrackedWrong, kFrame * dataCarriers().size() / 4UZ)) << "and without it the frame loses lock";
    };

    "cpe_interp follows a timing drift that cpe leaves on the outer carriers"_test = [] {
        const std::vector<CF> pilots  = rotatingPilots();
        const Modulated       sent    = modulate(1UZ, pilots);
        constexpr double      kStep   = 0.021; // samples of drift per symbol
        const auto            records = drifted(std::span<const gr::DataSet<CF>>(sent.records), kStep);

        const auto wrongWith = [&](const std::string& tracking) {
            gr::property_map settings = equalizerSettings(tracking);
            settings["pilot_symbols"] = interleave(std::span<const CF>(pilots));
            settings["alpha"]         = 0.5f;
            ChannelEqualizer  block   = make<ChannelEqualizer>(settings);
            const Equalized   got     = equalize(block, std::span<const gr::DataSet<CF>>(records));
            const std::size_t wrong   = symbolErrors(std::span<const gr::DataSet<CF>>(got.symbols), std::span<const CF>(sent.payload));
            std::println("timing drift of {} samples per symbol, tracking '{}': {} records, {} symbol errors", kStep, tracking, got.symbols.size(), wrong);
            return wrong;
        };
        const std::size_t phaseOnlyWrong    = wrongWith("cpe");
        const std::size_t interpolatedWrong = wrongWith("cpe_interp");

        // The four pilots sit in pairs at +-k. The drift gives them no common phase, and cpe leaves every carrier
        // turned. A QPSK point turned past pi/4 crosses a decision boundary.
        std::size_t turnedPast = 0UZ;
        for (std::size_t d = 0UZ; d < kFrame; ++d) {
            for (const std::int32_t carrier : dataCarriers()) {
                const double turn = 2. * std::numbers::pi * std::abs(static_cast<double>(carrier)) * kStep * static_cast<double>(d + 1UZ) / static_cast<double>(kFft);
                turnedPast += turn > std::numbers::pi / 4. ? 1UZ : 0UZ;
            }
        }
        expect(gt(turnedPast, 0UZ)) << "the drift turns the outer carriers of the last symbols past pi/4";
        expect(eq(phaseOnlyWrong, turnedPast)) << "cpe decides every carrier the drift turns past pi/4 wrong";
        expect(eq(interpolatedWrong, 0UZ)) << "cpe_interp follows the drift at the pilots and between them";
    };

    "a change of any setting but the numerology and the sync word keeps the channel estimate"_test = [] {
        const Modulated sent    = modulate();
        const auto      records = toRecords(throughMultipath(std::span<const gr::DataSet<CF>>(sent.records)));

        struct Outcome {
            std::size_t   wrong{0UZ};
            std::uint64_t untrained{0ULL};
        };
        const auto changedAfterSplit = [&](const gr::property_map& change) {
            ChannelEqualizer                       block = make<ChannelEqualizer>(equalizerSettings("cpe"));
            const std::span<const gr::DataSet<CF>> all(records);
            Equalized                              got = equalize(block, all.first(kSplit));
            std::ignore                                = block.settings().setStaged(change);
            const auto applied                         = block.settings().applyStagedParameters();
            expect(applied.failedParameters.empty()) << applied.refusal;
            const Equalized rest = equalize(block, all.subspan(kSplit));
            got.symbols.insert(got.symbols.end(), rest.symbols.begin(), rest.symbols.end());
            return Outcome{symbolErrors(std::span<const gr::DataSet<CF>>(got.symbols), std::span<const CF>(sent.payload)), block.nUntrained()};
        };

        // Eight pilot values in a cycle over four pilots give every symbol the same pilots as the four-value cycle. A
        // third sync symbol and a known symbol at index 0 leave the frame's remaining data symbols where they are.
        std::vector<CF> twice = pilotCycle();
        twice.insert(twice.end(), twice.begin(), twice.end());
        const std::vector<std::pair<std::string, gr::property_map>> kept{
            {"alpha", {{"alpha", 0.2f}}},
            {"tracking", {{"tracking", std::string("cpe_interp")}}},
            {"constellation", {{"constellation", std::string("psk")}, {"phase_offset", std::numbers::pi_v<float> / 4.f}}},
            {"pilot_symbols", {{"pilot_symbols", interleave(std::span<const CF>(twice))}}},
            {"n_sync", {{"n_sync", gr::Size_t{3U}}}},
            {"sync_index", {{"sync_index", gr::Size_t{0U}}}},
        };
        for (const auto& [name, change] : kept) {
            const Outcome outcome = changedAfterSplit(change);
            std::println("static multipath, {} changed after record {}: {} symbol errors, {} untrained", name, kSplit, outcome.wrong, outcome.untrained);
            expect(eq(outcome.wrong, 0UZ)) << name;
            expect(eq(outcome.untrained, std::uint64_t{0})) << name;
        }

        // A sync word change resets the estimate. The frame's remaining data symbols are equalized on the identity
        // channel.
        std::vector<CF> word = channelSounding();
        word[gr::ofdm::CarrierMap::binOf(kFft, 1).value()] *= -1.f;
        const Outcome reset = changedAfterSplit({{"sync_word", interleave(std::span<const CF>(word))}});
        std::println("static multipath, sync_word changed after record {}: {} symbol errors, {} untrained", kSplit, reset.wrong, reset.untrained);
        expect(eq(reset.untrained, std::uint64_t{records.size() - kSplit})) << "every data symbol after the change";
    };

    "a signal_name change reaches the next record"_test = [] {
        const Modulated                        sent = modulate();
        const std::span<const gr::DataSet<CF>> all(sent.records);
        ChannelEqualizer                       block = make<ChannelEqualizer>(equalizerSettings("cpe"));

        const Equalized before = equalize(block, all.first(static_cast<std::size_t>(kSync) + 1UZ));
        std::ignore            = block.settings().setStaged({{"signal_name", std::string("renamed")}});
        std::ignore            = block.settings().applyStagedParameters();
        const Equalized after  = equalize(block, all.subspan(static_cast<std::size_t>(kSync) + 1UZ));

        expect(eq(before.symbols.size(), 1UZ));
        expect(eq(after.symbols.size(), kFrame - 1UZ));
        for (const gr::DataSet<CF>& symbol : before.symbols) {
            expect(eq(symbol.signal_names[0UZ], std::string("ofdm_equalized")));
        }
        for (const gr::DataSet<CF>& symbol : after.symbols) {
            expect(eq(symbol.signal_names[0UZ], std::string("renamed")));
        }
        expect(eq(block.nUntrained(), std::uint64_t{0})) << "the name change keeps the channel estimate";
    };

    "a numerology the estimator cannot work on is refused"_test = [] {
        expect(throws([] {
            gr::property_map settings                          = equalizerSettings("cpe");
            std::vector<CF>  word                              = channelSounding();
            word[gr::ofdm::CarrierMap::binOf(kFft, 3).value()] = CF{};
            settings["sync_word"]                              = interleave(word);
            std::ignore                                        = make<ChannelEqualizer>(settings);
        })) << "a sync word silent on an occupied carrier";
        expect(throws([] {
            gr::property_map settings = equalizerSettings("cpe");
            settings["sync_index"]    = gr::Size_t{4U};
            std::ignore               = make<ChannelEqualizer>(settings);
        })) << "a sync index outside the frame's head";
        expect(throws([] {
            gr::property_map settings = equalizerSettings("wander");
            std::ignore               = make<ChannelEqualizer>(settings);
        })) << "a tracking mode the block has no rule for";
        std::string limits;
        try {
            gr::property_map settings = equalizerSettings("cpe");
            settings["alpha"]         = 0.f;
            std::ignore               = make<ChannelEqualizer>(settings);
        } catch (const gr::exception& e) {
            limits = e.what();
        }
        expect(limits.contains("alpha") && limits.contains("outside the limits")) << std::format("the limits of alpha refuse zero: '{}'", limits);
        for (const std::string tracking : {"cpe", "cpe_interp", "none"}) {
            expect(throws([&tracking] {
                gr::property_map settings = equalizerSettings(tracking);
                settings["pilot_symbols"] = std::vector<float>{1.f, 0.f, 1.f, 0.f, 1.f, 0.f, 0.f, 0.f};
                std::ignore               = make<ChannelEqualizer>(settings);
            })) << std::format("tracking '{}': a pilot value of zero", tracking);
        }
    };

    /**
     * The block equalizes on the settings it last accepted, whether a limit or a check refuses the application. Two
     * frames through static multipath and noise come out the same with a refused application after record 12 as without
     * it.
     */
    "a refused setting changes nothing the block computes"_test = [] {
        const Modulated sent  = modulate(2UZ);
        const Stream    faded = throughMultipath(std::span<const gr::DataSet<CF>>(sent.records));
        double          power = 0.;
        for (const CF& sample : faded.samples) {
            power += static_cast<double>(std::norm(sample));
        }
        power /= static_cast<double>(faded.samples.size());
        AwgnChannel<CF> noise = make<AwgnChannel<CF>>({{"noise_power", power / 100.}, {"seed", std::uint64_t{11}}});
        Stream          noisy;
        noisy.tags                                     = faded.tags;
        noisy.samples                                  = shim::run<CF>(noise, std::span<const CF>(faded.samples), 4096UZ).samples;
        const std::vector<gr::DataSet<CF>>     records = toRecords(noisy);
        const std::span<const gr::DataSet<CF>> all(records);

        struct Outcome {
            std::vector<gr::DataSet<CF>> symbols{};
            std::uint64_t                estimates{0ULL};
            float                        alpha{0.f};
        };
        const auto run = [all](const gr::property_map& change, std::string_view key) {
            gr::property_map settings = equalizerSettings("cpe_interp");
            settings["alpha"]         = 0.1f;
            ChannelEqualizer block    = make<ChannelEqualizer>(settings);
            Equalized        got      = equalize(block, all.first(kSplit));
            if (!change.empty()) {
                const std::string message = apply(block, change);
                expect(message.contains(key)) << std::format("the application is refused by {}: '{}'", key, message);
            }
            const Equalized rest = equalize(block, all.subspan(kSplit));
            got.symbols.insert(got.symbols.end(), rest.symbols.begin(), rest.symbols.end());
            return Outcome{std::move(got.symbols), block.nEstimates(), block.alpha.value};
        };

        const Outcome reference = run({}, {});
        expect(eq(reference.symbols.size(), 2UZ * kFrame));
        expect(eq(reference.estimates, std::uint64_t{2}));
        struct Refused {
            std::string      name;
            std::string_view key;
            gr::property_map change;
        };
        const std::vector<Refused> refused{
            {"sync_index 5", "sync_index", {{"sync_index", gr::Size_t{5U}}}},
            {"n_sync 1", "sync_index", {{"n_sync", gr::Size_t{1U}}}},
            {"tracking 'none' with sync_index 5", "sync_index", {{"tracking", std::string("none")}, {"sync_index", gr::Size_t{5U}}}},
            {"alpha 2, outside its limits", "alpha", {{"alpha", 2.f}}},
            {"alpha 0, outside its limits", "alpha", {{"alpha", 0.f}}},
            {"a NaN alpha, outside its limits", "alpha", {{"alpha", std::numeric_limits<float>::quiet_NaN()}}},
            {"fft_len 100, outside its limits", "fft_len", {{"fft_len", gr::Size_t{100U}}}},
        };
        for (const auto& [name, key, change] : refused) {
            const Outcome outcome = run(change, key);
            expect(eq(outcome.alpha, 0.1f)) << std::format("{}: the member alpha keeps the accepted value", name);
            expect(eq(outcome.estimates, reference.estimates)) << std::format("{}: one estimate per frame", name);
            expect(eq(outcome.symbols.size(), reference.symbols.size())) << std::format("{}: one record per data symbol", name);
            bool same = outcome.symbols.size() == reference.symbols.size();
            for (std::size_t s = 0UZ; same && s < outcome.symbols.size(); ++s) {
                same = outcome.symbols[s].signal_values == reference.symbols[s].signal_values;
            }
            expect(same) << std::format("{}: every equalized value", name);
        }
    };

    /**
     * Each arm pairs a refused application with a later valid change of another key. The records equal those of the
     * valid change alone.
     */
    "a valid change after a refused application leaves the refused values out"_test = [] {
        const Modulated                        sent    = modulate(2UZ);
        const std::vector<gr::DataSet<CF>>     records = toRecords(throughMultipath(std::span<const gr::DataSet<CF>>(sent.records)));
        const std::span<const gr::DataSet<CF>> all(records);

        std::vector<CF> twice = pilotCycle();
        twice.insert(twice.end(), twice.begin(), twice.end());
        struct Arm {
            std::string      name;
            gr::property_map refused;
            gr::property_map valid;
        };
        const std::vector<Arm> arms{
            {"sync_index 5 with tracking and signal_name, then alpha", {{"sync_index", gr::Size_t{5U}}, {"tracking", std::string("none")}, {"signal_name", std::string("refused")}}, {{"alpha", 0.2f}}},
            {"tracking 'wander' with signal_name, then pilot_symbols", {{"tracking", std::string("wander")}, {"signal_name", std::string("refused")}}, {{"pilot_symbols", interleave(std::span<const CF>(twice))}}},
        };
        for (const Arm& arm : arms) {
            const auto run = [all, &arm](bool refuseFirst) {
                ChannelEqualizer block = make<ChannelEqualizer>(equalizerSettings("cpe_interp"));
                Equalized        got   = equalize(block, all.first(kSplit));
                if (refuseFirst) {
                    std::ignore = block.settings().setStaged(arm.refused);
                    expect(throws([&block] { std::ignore = block.settings().applyStagedParameters(); })) << std::format("{}: the first application is refused", arm.name);
                }
                std::ignore = block.settings().setStaged(arm.valid);
                expect(nothrow([&block] { std::ignore = block.settings().applyStagedParameters(); })) << std::format("{}: the valid change applies", arm.name);
                const Equalized rest = equalize(block, all.subspan(kSplit));
                got.symbols.insert(got.symbols.end(), rest.symbols.begin(), rest.symbols.end());
                return std::move(got.symbols);
            };
            const auto expected = run(false);
            const auto got      = run(true);
            expect(eq(got.size(), expected.size())) << arm.name;
            for (std::size_t s = 0UZ; s < std::min(got.size(), expected.size()); ++s) {
                expect(got[s].signal_values == expected[s].signal_values) << std::format("{}: record {}", arm.name, s);
                expect(eq(got[s].signal_names[0UZ], expected[s].signal_names[0UZ])) << std::format("{}: record {} signal name", arm.name, s);
            }
        }

        // A NaN is outside the limits of alpha, and the framework refuses it each time it is sent.
        ChannelEqualizer block = make<ChannelEqualizer>(equalizerSettings("cpe_interp"));
        for (int attempt = 0; attempt < 2; ++attempt) {
            const std::string message = apply(block, {{"alpha", std::numeric_limits<float>::quiet_NaN()}});
            expect(message.contains("alpha") && message.contains("outside the limits")) << std::format("attempt {}: '{}'", attempt, message);
        }
        expect(eq(block.alpha.value, 0.1f)) << "the member keeps the accepted value";
    };

    "the counters count the whole run across a new sync word"_test = [] {
        const Modulated                        sent    = modulate(2UZ);
        const auto                             records = toRecords(throughMultipath(std::span<const gr::DataSet<CF>>(sent.records)));
        const std::span<const gr::DataSet<CF>> all(records);
        constexpr auto                         kFirst = static_cast<std::size_t>(kSync) + kFrame;

        ChannelEqualizer block = make<ChannelEqualizer>(equalizerSettings("cpe"));
        Equalized        got   = equalize(block, all.first(kFirst));
        expect(eq(block.nEstimates(), std::uint64_t{1}));
        std::vector<CF> word = channelSounding();
        word[gr::ofdm::CarrierMap::binOf(kFft, 1).value()] *= -1.f;
        std::ignore          = block.settings().setStaged({{"sync_word", interleave(std::span<const CF>(word))}});
        std::ignore          = block.settings().applyStagedParameters();
        const Equalized rest = equalize(block, all.subspan(kFirst));

        expect(eq(got.symbols.size() + rest.symbols.size(), 2UZ * kFrame));
        expect(eq(block.nEstimates(), std::uint64_t{2})) << "the second frame's estimate adds to the first";
        expect(eq(block.nSkipped(), std::uint64_t{2})) << "both preambles";
        expect(eq(block.nUntrained(), std::uint64_t{0})) << "the change falls between frames";
    };

    "a block left at its default settings starts, and its first call reports the missing numerology"_test = [] {
        ChannelEqualizer block(gr::property_map{});
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

        const Modulated                      sent = modulate();
        std::vector<gr::DataSet<CF>>         scratch(4UZ);
        shim::InputSpan<gr::DataSet<CF>>     inSpan(std::span<const gr::DataSet<CF>>(sent.records), 0UZ);
        shim::OutputSpan<gr::DataSet<CF>>    outSpan{std::span<gr::DataSet<CF>>(scratch)};
        shim::OutputSpan<gr::DataSet<float>> channelSpan(std::span<gr::DataSet<float>>{}, 0UZ, nullptr, false);
        expect(block.processBulk(inSpan, outSpan, channelSpan) == gr::work::Status::ERROR);
        expect(eq(inSpan.consumed, 0UZ));
        expect(eq(outSpan.count, 0UZ));
    };

    "a record whose length is not fft_len is dropped and counted"_test = [] {
        const Modulated  sent     = modulate(); // records of 64 bins
        gr::property_map settings = equalizerSettings("cpe");
        settings["fft_len"]       = gr::Size_t{128U};
        settings["sync_word"]     = interleave(std::span<const CF>(channelSounding(128U)));
        ChannelEqualizer block    = make<ChannelEqualizer>(settings);

        const Equalized got = equalize(block, std::span<const gr::DataSet<CF>>(sent.records), true);
        expect(eq(got.symbols.size(), 0UZ)) << "no equalized record";
        expect(eq(got.estimates.size(), 0UZ)) << "no channel record";
        expect(eq(block.nMalformed(), std::uint64_t{sent.records.size()})) << "every record counted";
        expect(eq(block.nEstimates(), std::uint64_t{0}));
    };

    "a block without a numerology takes fft_len alone, and the carriers then build a numerology of that length"_test = [] {
        constexpr gr::Size_t kWide = 128U;
        ChannelEqualizer     block = make<ChannelEqualizer>({});
        std::ignore                = block.settings().setStaged({{"fft_len", kWide}});
        expect(nothrow([&block] { std::ignore = block.settings().applyStagedParameters(); })) << "fft_len alone";
        std::ignore = block.settings().setStaged({{"data_carriers", dataCarriers()}, {"pilot_carriers", pilotCarriers()}, {"pilot_symbols", interleave(pilotCycle())}, {"sync_word", interleave(channelSounding(kWide))}});
        expect(nothrow([&block] { std::ignore = block.settings().applyStagedParameters(); })) << "the carriers and a sync word of 128 bins";

        // A frame of the known symbol and three data symbols at 128 bins, on an ideal channel.
        std::vector<CF> payload(3UZ * dataCarriers().size());
        std::uint32_t   state = 13579U;
        for (CF& symbol : payload) {
            symbol = qpsk(state);
        }
        const gr::property_map             wide{{"fft_len", kWide}, {"data_carriers", dataCarriers()}, {"pilot_carriers", pilotCarriers()}, {"pilot_symbols", interleave(pilotCycle())}, {"sync_words", interleave(channelSounding(kWide))}, {"frame_len", gr::Size_t{3U}}};
        const std::vector<gr::DataSet<CF>> records = allocate(wide, std::span<const CF>(payload));
        expect(eq(records.size(), 4UZ));

        const Equalized got = equalize(block, std::span<const gr::DataSet<CF>>(records));
        expect(eq(block.nMalformed(), std::uint64_t{0})) << "every record has the block's 128 bins";
        expect(eq(got.symbols.size(), 3UZ));
        expect(eq(symbolErrors(std::span<const gr::DataSet<CF>>(got.symbols), std::span<const CF>(payload)), 0UZ));
    };

    "a sync word with a non-finite value on an occupied carrier is refused"_test = [] {
        ChannelEqualizer block = make<ChannelEqualizer>(equalizerSettings("cpe"));

        std::vector<CF> onData                                = channelSounding();
        onData[gr::ofdm::CarrierMap::binOf(kFft, 3).value()]  = CF(std::numeric_limits<float>::quiet_NaN(), 0.f);
        std::vector<CF> onPilot                               = channelSounding();
        onPilot[gr::ofdm::CarrierMap::binOf(kFft, 7).value()] = CF(0.f, std::numeric_limits<float>::infinity());
        // A NaN equals no value, including the NaN its member holds. The framework stages the word again, and the block
        // refuses it again.
        const std::vector<std::pair<std::string, std::vector<CF>>> words{{"a NaN on data carrier 3", onData}, {"the same NaN sent again", onData}, {"an infinity on pilot carrier 7", onPilot}};
        for (const auto& [name, word] : words) {
            std::ignore = block.settings().setStaged({{"sync_word", interleave(std::span<const CF>(word))}});
            expect(throws([&block] { std::ignore = block.settings().applyStagedParameters(); })) << name;
        }
    };

    "a sync word with a NaN on a guard bin keeps the channel estimate across an alpha change"_test = [] {
        const Modulated                        sent    = modulate();
        const auto                             records = toRecords(throughMultipath(std::span<const gr::DataSet<CF>>(sent.records)));
        const std::span<const gr::DataSet<CF>> all(records);

        std::vector<CF> word                                = channelSounding();
        word[gr::ofdm::CarrierMap::binOf(kFft, 30).value()] = CF(std::numeric_limits<float>::quiet_NaN(), 0.f); // a guard
        gr::property_map settings                           = equalizerSettings("cpe");
        settings["sync_word"]                               = interleave(std::span<const CF>(word));
        ChannelEqualizer block                              = make<ChannelEqualizer>(settings);

        Equalized got = equalize(block, all.first(kSplit));
        std::ignore   = block.settings().setStaged({{"alpha", 0.2f}});
        expect(nothrow([&block] { std::ignore = block.settings().applyStagedParameters(); })) << "the alpha change applies";
        const Equalized rest = equalize(block, all.subspan(kSplit));
        got.symbols.insert(got.symbols.end(), rest.symbols.begin(), rest.symbols.end());

        expect(eq(got.symbols.size(), kFrame));
        expect(eq(block.nUntrained(), std::uint64_t{0})) << "the alpha change keeps the estimate";
        expect(eq(symbolErrors(std::span<const gr::DataSet<CF>>(got.symbols), std::span<const CF>(sent.payload)), 0UZ));
    };

    /**
     * The first data record carries a `symbol_in_frame` of 2^32 + 2, a value wider than 32 bits. The next record
     * carries none and takes the position after it, 2^32 + 3. Each equalized record states the position it read.
     */
    "a record without symbol_in_frame follows a 64-bit position whole"_test = [] {
        constexpr std::uint64_t      kWide = (std::uint64_t{1} << 32U) + std::uint64_t{kSync};
        const Modulated              sent  = modulate();
        std::vector<gr::DataSet<CF>> records(sent.records.begin(), sent.records.begin() + static_cast<std::ptrdiff_t>(kSync + 2U));
        records[kSync].meta_information[0UZ][std::pmr::string("symbol_in_frame")] = gr::pmt::Value(kWide);
        records[kSync + 1U].meta_information[0UZ].erase(std::pmr::string("symbol_in_frame"));

        ChannelEqualizer block = make<ChannelEqualizer>(equalizerSettings("cpe"));
        const Equalized  got   = equalize(block, std::span<const gr::DataSet<CF>>(records));
        expect(fatal(eq(got.symbols.size(), 2UZ)));
        expect(eq(gr::blocks::ofdm::detail::metaCount(got.symbols[0UZ], "symbol_in_frame", 0ULL), kWide));
        expect(eq(gr::blocks::ofdm::detail::metaCount(got.symbols[1UZ], "symbol_in_frame", 0ULL), kWide + 1ULL)) << "the unmarked record follows the whole position";
    };
};

} // namespace qa_ofdm_equalizer

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
