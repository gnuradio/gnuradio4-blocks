#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <limits>
#include <memory>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/algorithm/analysis/Autocorrelation.hpp>
#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/measurement/Autocorrelation.hpp>
#include <gnuradio-4.0/measurement/Detectors.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_autocorrelation {

using gr::blocks::measurement::Autocorrelation;
using gr::blocks::measurement::PeakDetect;
using CF = std::complex<float>;

constexpr double kPi    = std::numbers::pi;
constexpr double kTwoPi = 2. * std::numbers::pi;

/// The samples of one record of 32 half-overlapped windows of 4096 samples. A scene of this length or a little more
/// gives exactly one record.
constexpr std::size_t kOneRecord = 4096UZ + 31UZ * 2048UZ;

/// Emits a fixed sequence in bursts, then ends the stream. Each tag goes out on its stated sample,
/// inside a burst or at its start.
template<typename T>
struct BurstSource : gr::Block<BurstSource<T>> {
    gr::PortOut<T> out;

    std::vector<T> samples{};
    std::size_t    burst = 4096UZ;
    std::size_t    at    = 0UZ;

    std::vector<std::pair<std::size_t, gr::property_map>> tags{}; ///< each tag and the absolute sample it sits on, in order
    std::size_t                                           next = 0UZ;

    GR_MAKE_REFLECTABLE(BurstSource, out);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (at >= samples.size()) {
            outSpan.publish(0UZ);
            this->requestStop();
            return gr::work::Status::DONE;
        }
        const std::size_t take = std::min({burst, samples.size() - at, outSpan.size()});
        std::copy_n(samples.begin() + static_cast<std::ptrdiff_t>(at), take, outSpan.begin());
        while (next < tags.size() && tags[next].first < at + take) {
            outSpan.publishTag(tags[next].second, tags[next].first - at);
            ++next;
        }
        at += take;
        outSpan.publish(take);
        return gr::work::Status::OK;
    }
};

/// Keeps every record of a run.
struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<gr::DataSet<float>, gr::Async> in;

    std::vector<gr::DataSet<float>> records{};

    GR_MAKE_REFLECTABLE(RecordSink, in);

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        records.insert(records.end(), inSpan.begin(), inSpan.end());
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

[[nodiscard]] double metaNumber(const gr::DataSet<float>& record, std::string_view key, double fallback = -1.) {
    if (record.meta_information.empty()) {
        return fallback;
    }
    const auto& map = record.meta_information[0UZ];
    const auto  it  = map.find(std::pmr::string(key));
    if (it == map.end()) {
        return fallback;
    }
    if (const auto* asU64 = it->second.template get_if<std::uint64_t>()) {
        return static_cast<double>(*asU64);
    }
    if (const auto* asFloat = it->second.template get_if<float>()) {
        return static_cast<double>(*asFloat);
    }
    if (const auto* asDouble = it->second.template get_if<double>()) {
        return *asDouble;
    }
    if (const auto* asBool = it->second.template get_if<bool>()) {
        return *asBool ? 1. : 0.;
    }
    return fallback;
}

[[nodiscard]] std::string metaText(const gr::DataSet<float>& record, std::string_view key) {
    if (record.meta_information.empty()) {
        return {};
    }
    const auto& map = record.meta_information[0UZ];
    const auto  it  = map.find(std::pmr::string(key));
    if (it == map.end()) {
        return {};
    }
    const std::pmr::string* asString = it->second.template get_if<std::pmr::string>();
    return asString == nullptr ? std::string{} : std::string(*asString);
}

[[nodiscard]] bool hasKey(const gr::DataSet<float>& record, std::string_view key) { return !record.meta_information.empty() && record.meta_information[0UZ].contains(std::pmr::string(key)); }

/// @brief The records and the block's counters after one run.
template<typename T>
struct Run {
    std::vector<gr::DataSet<float>> records{};
    std::vector<gr::DataSet<float>> phase{};
    std::uint64_t                   nWindows{};
    std::uint64_t                   nRecords{};
    std::uint64_t                   nDegenerate{};
    std::uint64_t                   nNonFinite{};
    std::uint64_t                   nRecordsDropped{};
    std::uint64_t                   nPartialFlushes{};
    std::uint64_t                   nPartialWindows{};
    std::uint64_t                   nWindowResets{};
    std::uint64_t                   nRateRefused{};
    bool                            completed{}; ///< `runAndWait()` returned no error
    std::uint64_t                   nTailDropped{};
    std::uint64_t                   nSamples{};
};

/// @brief Runs `samples` through a source, `Autocorrelation<T>` and record sinks. The records and the counters are
/// read while the scheduler holds the graph.
template<typename T>
[[nodiscard]] Run<T> collect(gr::property_map settings, std::vector<T> samples, std::size_t burst = 65536UZ, bool withPhase = false, std::vector<std::pair<std::size_t, gr::property_map>> tags = {}) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<BurstSource<T>>();
    auto&     block  = graph.emplaceBlock<Autocorrelation<T>>(std::move(settings));
    auto&     sink   = graph.emplaceBlock<RecordSink>();
    source.samples   = std::move(samples);
    source.burst     = burst;
    source.tags      = std::move(tags);

    boost::ut::expect(graph.connect<"out", "in">(source, block).has_value());
    boost::ut::expect(graph.connect<"out", "in">(block, sink).has_value());
    RecordSink* phaseSink = nullptr;
    if (withPhase) {
        phaseSink = &graph.emplaceBlock<RecordSink>();
        boost::ut::expect(graph.connect<"phase", "in">(block, *phaseSink).has_value());
    }
    gr::scheduler::Simple scheduler;
    if (!scheduler.exchange(std::move(graph)).has_value()) {
        boost::ut::expect(false) << "the scheduler takes the graph";
        return {};
    }
    const bool completed = scheduler.runAndWait().has_value(); // the source ends the stream

    Run<T> result;
    result.completed       = completed;
    result.records         = sink.records;
    result.phase           = phaseSink == nullptr ? std::vector<gr::DataSet<float>>{} : phaseSink->records;
    result.nWindows        = block.nWindows();
    result.nRecords        = block.nRecords();
    result.nDegenerate     = block.nDegenerate();
    result.nNonFinite      = block.nNonFinite();
    result.nRecordsDropped = block.nRecordsDropped();
    result.nPartialFlushes = block.nPartialFlushes();
    result.nPartialWindows = block.nPartialWindows();
    result.nWindowResets   = block.nWindowResets();
    result.nRateRefused    = block.nRateRefused();
    result.nTailDropped    = block.nTailDropped();
    result.nSamples        = block.nSamples();
    return result;
}

/// @brief A block outside a graph, initialized and started as the scheduler starts one.
[[nodiscard]] std::unique_ptr<Autocorrelation<CF>> startedBlock(gr::property_map settings) {
    auto block = std::make_unique<Autocorrelation<CF>>(std::move(settings));
    block->settings().init();
    std::ignore = block->settings().applyStagedParameters();
    block->start();
    return block;
}

/// @brief Applies @p changes to a block by name, as a settings call does.
void applyByName(Autocorrelation<CF>& block, gr::property_map changes) {
    std::ignore = block.settings().set(std::move(changes));
    std::ignore = block.settings().activateContext();
    std::ignore = block.settings().applyStagedParameters();
}

/// @brief Stages @p changes on a block and applies them. Returns the settings the limits refused.
[[nodiscard]] gr::property_map refusedByLimits(Autocorrelation<CF>& block, gr::property_map changes) {
    std::ignore = block.settings().setStaged(std::move(changes));
    return block.settings().applyStagedParameters().failedParameters;
}

/// @brief What one direct call of the block returned and published.
struct Call {
    gr::work::Status                status{};
    std::size_t                     consumed{};
    std::vector<gr::DataSet<float>> records{};
    std::vector<gr::DataSet<float>> phase{};
};

/// @brief Calls `processBulk`, or `processEpilogue` when @p atEnd is set, over @p samples from stream index @p at. The
/// correlation port offers @p outRoom slots and the connected phase port @p phaseRoom.
[[nodiscard]] Call callBlock(Autocorrelation<CF>& block, std::span<const CF> samples, std::size_t at, std::size_t outRoom, std::size_t phaseRoom, bool atEnd = false) {
    std::vector<gr::DataSet<float>>                           outSlots(outRoom);
    std::vector<gr::DataSet<float>>                           phaseSlots(phaseRoom);
    gr::blocks::testing::span::InputSpan<CF>                  in(samples, at);
    gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> out{std::span<gr::DataSet<float>>(outSlots)};
    gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> phase{std::span<gr::DataSet<float>>(phaseSlots)};

    Call call;
    call.status   = atEnd ? block.processEpilogue(in, out, phase) : block.processBulk(in, out, phase);
    call.consumed = in.consumed;
    call.records.assign(outSlots.begin(), outSlots.begin() + static_cast<std::ptrdiff_t>(out.count));
    call.phase.assign(phaseSlots.begin(), phaseSlots.begin() + static_cast<std::ptrdiff_t>(phase.count));
    return call;
}

/// @brief The largest value over lags `[first, last]`, and where it sits.
[[nodiscard]] std::pair<std::size_t, double> peakOver(const gr::DataSet<float>& record, std::size_t first, std::size_t last) {
    std::size_t at   = first;
    double      best = -1.;
    for (std::size_t lag = first; lag <= last && lag < record.signal_values.size(); ++lag) {
        if (static_cast<double>(record.signal_values[lag]) > best) {
            best = static_cast<double>(record.signal_values[lag]);
            at   = lag;
        }
    }
    return {at, best};
}

struct Rng {
    std::uint64_t state = 0x243f6a8885a308d3ULL;

    [[nodiscard]] std::uint64_t next() {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
    [[nodiscard]] double uniform() { return (static_cast<double>(next() >> 11U) + 0.5) / static_cast<double>(1ULL << 53U); }
    [[nodiscard]] CF     gaussian() {
        const double radius = std::sqrt(-std::log(uniform()));
        const double angle  = kTwoPi * uniform();
        return CF(static_cast<float>(radius * std::cos(angle)), static_cast<float>(radius * std::sin(angle)));
    }
};

[[nodiscard]] std::vector<CF> complexNoise(std::size_t n, std::uint64_t seed) {
    Rng             rng{seed};
    std::vector<CF> out(n);
    for (CF& sample : out) {
        sample = rng.gaussian();
    }
    return out;
}

// ---- scenes -------------------------------------------------------------------------------------------------

/// @brief An OFDM stream of random quadrature symbols on every bin, transformed to time. Each symbol is preceded by a
/// copy of its own last `nCp` samples. The prefix puts an exact repeat at the useful symbol's length.
[[nodiscard]] std::vector<CF> ofdmScene(std::size_t nFft, std::size_t nCp, std::size_t symbols, std::uint64_t seed, double offsetPerSample = 0.) {
    Rng                                                                                              rng{seed};
    gr::algorithm::FFT<std::complex<float>, std::complex<float>, gr::algorithm::Direction::Backward> inverse{};
    std::vector<CF>                                                                                  bins(nFft);
    std::vector<CF>                                                                                  time(nFft);
    std::vector<CF>                                                                                  out;
    out.reserve(symbols * (nFft + nCp));

    for (std::size_t symbol = 0UZ; symbol < symbols; ++symbol) {
        for (CF& bin : bins) {
            const std::size_t quadrant = static_cast<std::size_t>(rng.next() % 4ULL);
            const float       re       = (quadrant & 1ULL) != 0UZ ? 1.f : -1.f;
            const float       im       = (quadrant & 2ULL) != 0UZ ? 1.f : -1.f;
            bin                        = CF(re, im);
        }
        inverse.compute(bins, time);
        out.insert(out.end(), time.end() - static_cast<std::ptrdiff_t>(nCp), time.end());
        out.insert(out.end(), time.begin(), time.end());
    }
    if (offsetPerSample != 0.) {
        for (std::size_t k = 0UZ; k < out.size(); ++k) {
            const double phase = kTwoPi * offsetPerSample * static_cast<double>(k);
            out[k] *= CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
        }
    }
    return out;
}

/// @brief A raised-cosine transmit pulse, `spanSymbols` symbols long at `sps` samples a symbol.
[[nodiscard]] std::vector<float> raisedCosine(std::size_t sps, double beta, std::size_t spanSymbols) {
    const std::size_t  taps   = spanSymbols * sps + 1UZ;
    const auto         center = static_cast<double>(taps - 1UZ) / 2.;
    std::vector<float> h(taps);
    for (std::size_t k = 0UZ; k < taps; ++k) {
        const double t = (static_cast<double>(k) - center) / static_cast<double>(sps);
        double       value{};
        if (std::abs(t) < 1e-12) {
            value = 1.;
        } else if (beta > 0. && std::abs(std::abs(2. * beta * t) - 1.) < 1e-9) {
            const double u = 1. / (2. * beta);
            value          = (kPi / 4.) * std::sin(kPi * u) / (kPi * u);
        } else {
            const double sinc = std::sin(kPi * t) / (kPi * t);
            value             = sinc * std::cos(kPi * beta * t) / (1. - 4. * beta * beta * t * t);
        }
        h[k] = static_cast<float>(value);
    }
    return h;
}

/// @brief Antipodal symbols at `sps` samples a symbol, shaped by @p pulse. An empty pulse holds each symbol flat, the
/// rectangular case.
[[nodiscard]] std::vector<CF> linearModulation(std::size_t symbols, std::size_t sps, std::span<const float> pulse, std::uint64_t seed) {
    Rng                rng{seed};
    std::vector<float> antipodal(symbols);
    for (float& symbol : antipodal) {
        symbol = (rng.next() & 1ULL) != 0ULL ? 1.f : -1.f;
    }
    if (pulse.empty()) {
        std::vector<CF> out(symbols * sps);
        for (std::size_t k = 0UZ; k < symbols; ++k) {
            for (std::size_t s = 0UZ; s < sps; ++s) {
                out[k * sps + s] = CF(antipodal[k], 0.f);
            }
        }
        return out;
    }
    std::vector<CF> out(symbols * sps, CF{});
    for (std::size_t k = 0UZ; k < symbols; ++k) {
        const std::size_t base = k * sps;
        for (std::size_t t = 0UZ; t < pulse.size(); ++t) {
            const std::size_t at = base + t;
            if (at < out.size()) {
                out[at] += CF(antipodal[k] * pulse[t], 0.f);
            }
        }
    }
    return out;
}

/// @brief The C4FM scene, four-level frequency modulation at 4800 symbols a second and 20 samples a symbol.
///
/// The dibits are shaped by a raised cosine of roll-off 0.2. The outer level deviates 1800 Hz and the inner 600 Hz,
/// and the shaped deviation is integrated into a phase. The raised cosine is the Nyquist pulse
/// `sinc(t/T) cos(pi a t/T) / (1 - (2 a t/T)^2)`, the transmit filter itself. A continuous-phase modulator's frequency
/// pulse of the same name is a different function.
[[nodiscard]] std::vector<CF> c4fmScene(std::size_t symbols, std::uint64_t seed) {
    constexpr std::size_t sps  = 20UZ;
    constexpr double      rate = 96000.;
    Rng                   rng{seed};

    const std::vector<float> pulse = raisedCosine(sps, 0.2, 8UZ);
    std::vector<double>      frequency(symbols * sps, 0.);
    for (std::size_t k = 0UZ; k < symbols; ++k) {
        const auto        rank      = static_cast<double>(rng.next() % 4ULL);
        const double      deviation = (2. * rank - 3.) * 600.; // the odd grid times 600 Hz, giving +/-1800 and +/-600
        const std::size_t base      = k * sps;
        for (std::size_t t = 0UZ; t < pulse.size(); ++t) {
            const std::size_t at = base + t;
            if (at < frequency.size()) {
                frequency[at] += deviation * static_cast<double>(pulse[t]);
            }
        }
    }
    std::vector<CF> out(frequency.size());
    double          phase = 0.;
    for (std::size_t k = 0UZ; k < frequency.size(); ++k) {
        phase += kTwoPi * frequency[k] / rate;
        out[k] = CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
    }
    return out;
}

/// @brief The valid part of the convolution of @p in with @p taps.
[[nodiscard]] std::vector<CF> applyFir(std::span<const CF> in, std::span<const float> taps) {
    if (in.size() < taps.size()) {
        return {};
    }
    std::vector<CF> out(in.size() - taps.size() + 1UZ);
    for (std::size_t k = 0UZ; k < out.size(); ++k) {
        CF sum{};
        for (std::size_t t = 0UZ; t < taps.size(); ++t) {
            sum += in[k + t] * taps[taps.size() - 1UZ - t];
        }
        out[k] = sum;
    }
    return out;
}

/// The Mode S preamble in half-microsecond slots, with pulses at 0.0, 1.0, 3.5 and 4.5 microseconds.
constexpr std::array<int, 16> kPreamble{1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0};

/// @brief One long Mode S frame at two slots a microsecond. Sixteen preamble slots come first, then two slots a bit.
/// A one has its pulse in the first half and a zero in the second.
[[nodiscard]] std::vector<CF> modeSFrame(Rng& rng, bool withPreamble, bool withData) {
    std::vector<CF> wave;
    if (withPreamble) {
        for (const int pulse : kPreamble) {
            wave.emplace_back(pulse != 0 ? 1.f : 0.f, 0.f);
        }
    }
    if (withData) {
        for (std::size_t bit = 0UZ; bit < 112UZ; ++bit) {
            const bool one = (rng.next() & 1ULL) != 0ULL;
            wave.emplace_back(one ? 1.f : 0.f, 0.f);
            wave.emplace_back(one ? 0.f : 1.f, 0.f);
        }
    }
    return wave;
}

/// @brief The scatter and threshold a record states, recomputed from the keys it publishes.
struct HandComputed {
    double scatter{};
    double threshold{};
};

/// @brief Recomputes the scatter and the threshold of a record of rectangular windows. A product at lag `tau` carries
/// one unit of weight per window that holds it, and the scatter of the lag is the root of the summed squared weights
/// over the summed weights.
[[nodiscard]] HandComputed recomputed(const gr::DataSet<float>& record) {
    const auto n       = static_cast<std::size_t>(metaNumber(record, "window_length"));
    const auto lags    = static_cast<std::size_t>(metaNumber(record, "max_lag"));
    const auto hop     = static_cast<std::size_t>(metaNumber(record, "hop"));
    const auto windows = static_cast<std::size_t>(metaNumber(record, "n_averaged"));
    const bool biased  = metaText(record, "normalization") == std::string("biased");
    const auto pFa     = metaNumber(record, "false_alarm_rate");
    const bool real    = metaNumber(record, "real_valued") > 0.5;
    boost::ut::expect(metaText(record, "window") == std::string("Rectangular")) << "the recomputation holds for a rectangular taper";

    std::vector<double> scatter(lags, 0.);
    double              largest = 0.;
    for (std::size_t lag = 1UZ; lag <= lags; ++lag) {
        auto squares = static_cast<double>(windows * (n - lag));
        for (std::size_t m = 1UZ; m < windows && m * hop < n - lag; ++m) {
            squares += 2. * static_cast<double>(windows - m) * static_cast<double>(n - lag - m * hop);
        }
        scatter[lag - 1UZ] = std::sqrt(squares) / static_cast<double>(windows * (biased ? n : n - lag));
        largest            = std::max(largest, scatter[lag - 1UZ]);
    }
    return HandComputed{
        .scatter   = largest,
        .threshold = gr::analysis::acfRecordThreshold(scatter, pFa, real),
    };
}

/// @brief The autocovariance of the envelope `|s|^2` of independent antipodal symbols shaped by @p pulse at @p sps
/// samples a symbol, averaged over the symbol phase, at lags `0..maxLag`.
///
/// The fourth moment of the symbols is one where the four indices pair up and zero otherwise. With
/// `m(t) = sum_k p(t - kT)^2` and `q(t, tau) = sum_k p(t - kT) p(t + tau - kT)`, the envelope correlation is
/// `m(t) m(t + tau) + 2 q(t, tau)^2 - 2 sum_k p(t - kT)^2 p(t + tau - kT)^2`. The square of the mean of `m` over a
/// symbol is subtracted.
[[nodiscard]] std::vector<double> shapedEnvelopeCovariance(std::span<const float> pulse, std::size_t sps, std::size_t maxLag) {
    const auto taps    = static_cast<std::ptrdiff_t>(pulse.size());
    const auto period  = static_cast<std::ptrdiff_t>(sps);
    const auto symbols = taps / period + 2;
    const auto at      = [&](std::ptrdiff_t index) { return index >= 0 && index < taps ? static_cast<double>(pulse[static_cast<std::size_t>(index)]) : 0.; };
    const auto power   = [&](std::ptrdiff_t t) {
        double sum = 0.;
        for (std::ptrdiff_t k = -symbols; k <= symbols; ++k) {
            sum += at(t - k * period) * at(t - k * period);
        }
        return sum;
    };
    double mean = 0.;
    for (std::ptrdiff_t t = 0; t < period; ++t) {
        mean += power(t) / static_cast<double>(period);
    }
    std::vector<double> covariance(maxLag + 1UZ, 0.);
    for (std::size_t lag = 0UZ; lag <= maxLag; ++lag) {
        const auto shift = static_cast<std::ptrdiff_t>(lag);
        double     sum   = 0.;
        for (std::ptrdiff_t t = 0; t < period; ++t) {
            double cross  = 0.;
            double fourth = 0.;
            for (std::ptrdiff_t k = -symbols; k <= symbols; ++k) {
                const double a = at(t - k * period);
                const double b = at(t + shift - k * period);
                cross += a * b;
                fourth += a * a * b * b;
            }
            sum += power(t) * power((t + shift) % period) + 2. * cross * cross - 2. * fourth;
        }
        covariance[lag] = sum / static_cast<double>(period) - mean * mean;
    }
    return covariance;
}

/// @brief Bartlett's factor for the variance of an estimated correlation coefficient at @p lag, over the coefficients
/// @p rho at lags `0..K`. The variance of the estimate is this factor over the count of independent samples.
[[nodiscard]] double bartlettFactor(std::span<const double> rho, std::size_t lag) {
    const auto coefficient = [&](std::ptrdiff_t k) {
        const auto index = static_cast<std::size_t>(k < 0 ? -k : k);
        return index < rho.size() ? rho[index] : 0.;
    };
    const auto   limit = static_cast<std::ptrdiff_t>(rho.size()) - 1;
    const auto   tau   = static_cast<std::ptrdiff_t>(lag);
    const double r     = coefficient(tau);
    double       sum   = 0.;
    for (std::ptrdiff_t k = -limit; k <= limit; ++k) {
        sum += coefficient(k) * coefficient(k) + coefficient(k + tau) * coefficient(k - tau) - 4. * r * coefficient(k) * coefficient(k - tau) + 2. * coefficient(k) * coefficient(k) * r * r;
    }
    return sum;
}

} // namespace qa_autocorrelation

using namespace boost::ut;
using namespace qa_autocorrelation;

const suite<"autocorrelation record and settings"> _acfRecord = [] {
    "a record states its lag axis, its rate and every metadata key"_test = [] {
        const std::vector<CF> input = complexNoise(40000UZ, 0x1111ULL);
        for (const float rate : {2.0e6f, 125000.f}) {
            const Run<CF> run = collect<CF>({{"window_length", gr::Size_t(4096)}, {"max_lag", gr::Size_t(100)}, {"kind", std::string("complex")}, {"n_averages", gr::Size_t(4)}, {"sample_rate", rate}}, input);
            expect(fatal(ge(run.records.size(), 1UZ))) << "the stream is long enough for at least one record";
            const gr::DataSet<float>& record = run.records[0UZ];

            expect(eq(record.axis_names.size(), 1UZ));
            expect(record.axis_names[0UZ] == std::string("Lag"));
            expect(record.axis_units[0UZ] == std::string("s"));
            expect(eq(record.extents.size(), 1UZ));
            expect(eq(record.extents[0UZ], 101));
            expect(eq(record.signal_values.size(), 101UZ));
            expect(record.signal_quantities[0UZ] == std::string("Autocorrelation"));
            expect(record.signal_names[0UZ] == std::string("acf"));
            for (std::size_t k = 0UZ; k <= 100UZ; ++k) {
                expect(record.axis_values[0UZ][k] == static_cast<float>(k) / rate) << std::format("lag {} at rate {}", k, rate);
            }
            expect(approx(metaNumber(record, "sample_rate"), static_cast<double>(rate), 1e-3));

            for (const std::string_view key : {"sample_rate", "sample_start", "sequence", "kind", "normalization", "window_length", "max_lag", "hop", "overlap", "window", "n_averaged", "n_samples", "n_pairs", "scatter_lag", "mean_removed", "real_valued", "zero_lag", "scatter", "detection_threshold", "false_alarm_rate", "peak_threshold_db", "transform_length"}) {
                expect(hasKey(record, key)) << std::format("the record carries '{}'", key);
            }
            expect(eq(static_cast<std::size_t>(metaNumber(record, "window_length")), 4096UZ));
            expect(eq(static_cast<std::size_t>(metaNumber(record, "max_lag")), 100UZ));
            expect(eq(static_cast<std::size_t>(metaNumber(record, "hop")), 2048UZ));
            expect(eq(static_cast<std::size_t>(metaNumber(record, "transform_length")), 8192UZ)) << "the next power of two at or above N + L";
            expect(eq(static_cast<std::size_t>(metaNumber(record, "n_averaged")), 4UZ));
            expect(eq(static_cast<std::size_t>(metaNumber(record, "n_samples")), 4096UZ + 3UZ * 2048UZ));
            expect(metaText(record, "kind") == std::string("complex"));
            expect(metaText(record, "normalization") == std::string("unbiased"));
            expect(metaText(record, "window") == std::string("Rectangular"));
            expect(eq(static_cast<std::size_t>(metaNumber(record, "sequence")), 0UZ));
            expect(record.signal_values[0UZ] == 1.f) << "lag zero reads exactly one";
        }
    };

    "each invalid setting is refused, and the lag refusal names both values"_test = [] {
        const auto refused = [](gr::property_map settings) {
            Autocorrelation<CF> block(std::move(settings));
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        };
        expect(throws([&] { refused({{"max_lag", gr::Size_t(4096)}, {"window_length", gr::Size_t(4096)}}); })) << "a lag of a whole window";
        expect(throws([&] { refused({{"max_lag", gr::Size_t(5000)}, {"window_length", gr::Size_t(4096)}}); })) << "a lag beyond a window";
        expect(throws([&] { refused({{"kind", std::string("envelop")}}); }));
        expect(throws([&] { refused({{"normalization", std::string("none")}}); }));
        expect(throws([&] { refused({{"window", std::string("NotAWindow")}}); }));
        expect(throws([&] { refused({{"sample_rate", 0.f}}); })) << "a stopped block refuses a rate of zero";
        expect(throws([&] { refused({{"sample_rate", std::numeric_limits<float>::quiet_NaN()}}); })) << "and a NaN rate";

        // values outside the declared limits
        const std::array<gr::property_map, 7UZ> outside{{
            {{"max_lag", gr::Size_t(0)}},
            {{"window_length", gr::Size_t(8)}},
            {{"n_averages", gr::Size_t(0)}},
            {{"overlap", 1.0}},
            {{"overlap", -0.1}},
            {{"false_alarm_rate", 0.0}},
            {{"false_alarm_rate", 1.0}},
        }};
        for (std::size_t r = 0UZ; r < outside.size(); ++r) {
            auto block = startedBlock({});
            expect(throws([&] { std::ignore = block->settings().set(outside[r]); })) << std::format("value {}: settings().set() refuses it at the call", r);
            expect(eq(refusedByLimits(*block, outside[r]).size(), 1UZ)) << std::format("value {}: the limits refuse a staged one", r);
            expect(eq(block->max_lag.value, gr::Size_t(1024)) && eq(block->window_length.value, gr::Size_t(4096)) && eq(block->n_averages.value, gr::Size_t(16)) && eq(block->overlap.value, 0.5) && eq(block->false_alarm_rate.value, 1e-3) && eq(block->sample_rate.value, 1.f)) << std::format("value {}: the members keep the accepted values", r);
        }

        // the lag refusal names both values
        try {
            refused({{"max_lag", gr::Size_t(4096)}, {"window_length", gr::Size_t(1024)}});
            expect(false) << "a lag beyond the window is refused";
        } catch (const std::exception& error) {
            const std::string what(error.what());
            expect(what.find("4096") != std::string::npos && what.find("1024") != std::string::npos) << std::format("the message names both values: {}", what);
        }
    };

    "the staged-restart keys are refused while the block runs and change once it stops"_test = [] {
        const gr::property_map base{{"window_length", gr::Size_t(1024)}, {"max_lag", gr::Size_t(64)}, {"sample_rate", 48000.f}};

        const std::array<gr::property_map, 7> kStaged{gr::property_map{{"window_length", gr::Size_t(2048)}}, gr::property_map{{"max_lag", gr::Size_t(128)}}, gr::property_map{{"kind", std::string("complex")}}, gr::property_map{{"normalization", std::string("biased")}}, gr::property_map{{"overlap", 0.25}}, gr::property_map{{"window", std::string("Hann")}}, gr::property_map{{"remove_mean", false}}};
        for (const gr::property_map& change : kStaged) {
            auto block = startedBlock(base);
            expect(throws([&] { applyByName(*block, change); })) << std::format("a running block refuses '{}'", change.begin()->first.c_str());
        }
        const std::array<gr::property_map, 5> kLive{gr::property_map{{"n_averages", gr::Size_t(4)}}, gr::property_map{{"false_alarm_rate", 1e-2}}, gr::property_map{{"emit_phase", true}}, gr::property_map{{"signal_name", std::string("r")}}, gr::property_map{{"sample_rate", 96000.f}}};
        for (const gr::property_map& change : kLive) {
            auto block = startedBlock(base);
            expect(nothrow([&] { applyByName(*block, change); })) << std::format("'{}' is live", change.begin()->first.c_str());
        }
        {
            auto block = startedBlock(base);
            block->stop();
            expect(nothrow([&] { applyByName(*block, {{"window_length", gr::Size_t(2048)}}); })) << "a stopped block takes a new window_length";
        }
        {
            auto block = startedBlock(base);
            expect(nothrow([&] { applyByName(*block, {{"sample_rate", 0.f}}); })) << "a running block counts a rate of zero";
            expect(eq(block->nRateRefused(), std::uint64_t{1ULL}));
            expect(block->sample_rate.value == 48000.f) << "and keeps the previous rate";
        }
        {
            // the limits refuse one key of a staged batch, and the other key applies
            auto                   block  = startedBlock(base);
            const gr::property_map failed = refusedByLimits(*block, {{"false_alarm_rate", 1.5}, {"n_averages", gr::Size_t(4)}});
            expect(eq(failed.size(), 1UZ) && failed.contains("false_alarm_rate")) << "the limits refuse false_alarm_rate alone";
            expect(eq(block->false_alarm_rate.value, 1e-3)) << "the member keeps the accepted value";
            const std::vector<CF> input = complexNoise(1024UZ + 3UZ * 512UZ, 0x7A7AULL);
            const Call            call  = callBlock(*block, std::span<const CF>(input), 0UZ, 4UZ, 0UZ);
            expect(fatal(eq(call.records.size(), 1UZ))) << "four windows make one record at the n_averages of the batch";
            expect(eq(static_cast<std::size_t>(metaNumber(call.records[0UZ], "n_averaged")), 4UZ));
            expect(approx(metaNumber(call.records[0UZ], "false_alarm_rate"), 1e-3, 1e-12)) << "on the accepted false_alarm_rate";
        }
    };

    "a refused setting changes nothing the block computes, later changes apply, and start() builds from the accepted values"_test = [] {
        constexpr std::size_t     length  = 1024UZ;
        const std::vector<CF>     input   = complexNoise(6UZ * length, 0x2D2DULL);
        const std::span<const CF> samples = std::span<const CF>(input);
        // without sample_rate among the settings, the rate tag below reaches it
        const gr::property_map geometry{{"window_length", gr::Size_t(length)}, {"max_lag", gr::Size_t(64)}, {"kind", std::string("complex")}, {"n_averages", gr::Size_t(2)}, {"overlap", 0.0}};
        const gr::Tag          rateTag{0UZ, gr::property_map{{gr::tag::SAMPLE_RATE.shortKey(), 96000.f}}};

        // the reference runs the same stream and changes without the refused application
        auto refused   = startedBlock(geometry);
        auto reference = startedBlock(geometry);
        for (Autocorrelation<CF>* block : {refused.get(), reference.get()}) {
            expect(fatal(eq(callBlock(*block, samples.first(2UZ * length), 0UZ, 4UZ, 0UZ).records.size(), 1UZ)));
        }
        try {
            applyByName(*refused, {{"window", std::string("Hann")}});
            expect(false) << "a running block refuses a new window";
        } catch (const std::exception& error) {
            expect(std::string_view(error.what()).find("window") != std::string_view::npos) << std::format("the refusal names the key: {}", error.what());
        }
        expect(refused->window.value == std::string("Hann")) << "the refused value stays in its member";
        const auto              stored = refused->settings().get(std::string("window"));
        const std::pmr::string* text   = stored.has_value() ? stored->get_if<std::pmr::string>() : nullptr;
        expect(text != nullptr && *text == "Hann") << "and settings().get() returns it";

        for (Autocorrelation<CF>* block : {refused.get(), reference.get()}) {
            expect(nothrow([&] { applyByName(*block, {{"n_averages", gr::Size_t(4)}}); })) << "a live change after the refusal applies";
            block->settings().autoUpdate(rateTag);
            expect(nothrow([&] { std::ignore = block->settings().applyStagedParameters(); })) << "and so does a rate tag";
        }
        const Call after = callBlock(*refused, samples.subspan(2UZ * length, 4UZ * length), 2UZ * length, 4UZ, 0UZ);
        const Call twin  = callBlock(*reference, samples.subspan(2UZ * length, 4UZ * length), 2UZ * length, 4UZ, 0UZ);
        expect(fatal(eq(after.records.size(), 1UZ)));
        expect(fatal(eq(twin.records.size(), 1UZ)));
        const gr::DataSet<float>& record = after.records[0UZ];
        expect(metaText(record, "window") == std::string("Rectangular")) << "the record names the window the kernel runs";
        expect(eq(static_cast<std::size_t>(metaNumber(record, "n_averaged")), 4UZ)) << "the live n_averages applies";
        expect(approx(metaNumber(record, "sample_rate"), 96000., 1e-3)) << "the tagged rate applies";
        expect(record.signal_values == twin.records[0UZ].signal_values) << "the refused value changes nothing the block computes";
        expect(record.meta_information[0UZ] == twin.records[0UZ].meta_information[0UZ]);

        // after a stop, both blocks build from their accepted values, and the refused member takes no part
        const auto restart = [&samples](Autocorrelation<CF>& block) {
            block.stop();
            expect(nothrow([&] { block.start(); }));
            const Call call = callBlock(block, samples.first(4UZ * length), 0UZ, 4UZ, 0UZ);
            expect(fatal(eq(call.records.size(), 1UZ))) << "four windows make one record at the live n_averages";
            expect(approx(metaNumber(call.records[0UZ], "sample_rate"), 96000., 1e-3));
            return call.records[0UZ];
        };
        const gr::DataSet<float> restarted = restart(*refused);
        expect(metaText(restarted, "window") == std::string("Rectangular")) << "start() builds from the accepted values";
        expect(restarted.signal_values == restart(*reference).signal_values) << "and computes what the reference computes";

        // the same value sent again alone moves nothing, and a stopped block takes the key once an application moves it again
        expect(nothrow([&] { applyByName(*refused, {{"window", std::string("Hann")}}); })) << "the value sent again passes with no refusal";
        expect(metaText(restart(*refused), "window") == std::string("Rectangular")) << "and changes nothing";
        refused->stop();
        expect(nothrow([&] { applyByName(*refused, {{"window", std::string("Rectangular")}}); }));
        expect(nothrow([&] { applyByName(*refused, {{"window", std::string("Hann")}}); }));
        const gr::DataSet<float> hannRecord = restart(*refused);
        expect(metaText(hannRecord, "window") == std::string("Hann")) << "a stopped block takes the key once it moves again";

        gr::property_map hann = geometry;
        hann.insert_or_assign(std::pmr::string("window"), std::string("Hann"));
        hann.insert_or_assign(std::pmr::string("n_averages"), gr::Size_t(4));
        hann.insert_or_assign(std::pmr::string("sample_rate"), 96000.f);
        auto       fresh     = startedBlock(hann);
        const Call freshCall = callBlock(*fresh, samples.first(4UZ * length), 0UZ, 4UZ, 0UZ);
        expect(fatal(eq(freshCall.records.size(), 1UZ)));
        expect(hannRecord.signal_values == freshCall.records[0UZ].signal_values) << "the block computes what a block built with Hann computes";
        expect(hannRecord.signal_values != restarted.signal_values) << "and the two tapers give different records";
    };

    "an out-of-range staged-restart value refused while running names its range, and the next start works"_test = [] {
        constexpr std::size_t                                              length = 1024UZ;
        const std::vector<CF>                                              input  = complexNoise(4UZ * length, 0x6262ULL);
        const gr::property_map                                             settings{{"window_length", gr::Size_t(length)}, {"max_lag", gr::Size_t(64)}, {"kind", std::string("complex")}, {"n_averages", gr::Size_t(4)}, {"overlap", 0.0}, {"sample_rate", 48000.f}};
        const std::array<std::pair<gr::property_map, std::string_view>, 2> kOutOfRange{{
            {gr::property_map{{"kind", std::string("bogus")}}, "'complex' or 'envelope'"},
            {gr::property_map{{"max_lag", gr::Size_t(2048)}}, "must be below window_length"},
        }};
        for (const auto& [change, range] : kOutOfRange) {
            auto block = startedBlock(settings);
            try {
                applyByName(*block, change);
                expect(false) << "a running block refuses an out-of-range value";
            } catch (const std::exception& error) {
                expect(std::string_view(error.what()).find(range) != std::string_view::npos) << std::format("the refusal names the range: {}", error.what());
            }
            block->stop();
            expect(nothrow([&] { block->start(); })) << std::format("start() after the refused '{}' works", change.begin()->first.c_str());
            const Call call = callBlock(*block, std::span<const CF>(input), 0UZ, 4UZ, 0UZ);
            expect(fatal(eq(call.records.size(), 1UZ)));
            expect(eq(static_cast<std::size_t>(metaNumber(call.records[0UZ], "window_length")), length)) << "on the accepted values";
            expect(eq(static_cast<std::size_t>(metaNumber(call.records[0UZ], "max_lag")), 64UZ));
            expect(metaText(call.records[0UZ], "kind") == std::string("complex"));
        }
    };

    "a rate change waits for room on both ports, and its short record reaches both"_test = [] {
        constexpr std::size_t     length  = 1024UZ;
        const std::vector<CF>     input   = complexNoise(4UZ * length, 0x3131ULL);
        const std::span<const CF> samples = std::span<const CF>(input);
        auto                      block   = startedBlock({{"window_length", gr::Size_t(length)}, {"max_lag", gr::Size_t(64)}, {"kind", std::string("complex")}, {"n_averages", gr::Size_t(4)}, {"overlap", 0.0}, {"emit_phase", true}, {"sample_rate", 48000.f}});

        const Call opening = callBlock(*block, samples.first(2UZ * length), 0UZ, 4UZ, 4UZ);
        expect(opening.records.empty() && opening.phase.empty()) << "two windows of four complete no record";
        applyByName(*block, {{"sample_rate", 96000.f}});

        const Call blocked = callBlock(*block, samples.subspan(2UZ * length), 2UZ * length, 4UZ, 0UZ);
        expect(blocked.status == gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS) << "the phase port has no room for the short record";
        expect(eq(blocked.consumed, 0UZ));
        expect(blocked.records.empty()) << "and the correlation port does not take it alone";

        const Call flushed = callBlock(*block, samples.subspan(2UZ * length), 2UZ * length, 4UZ, 4UZ);
        expect(flushed.status == gr::work::Status::OK);
        expect(eq(flushed.consumed, 2UZ * length));
        expect(fatal(eq(flushed.records.size(), 1UZ)));
        expect(fatal(eq(flushed.phase.size(), 1UZ))) << "the short record reaches both ports";
        for (const gr::DataSet<float>& record : {flushed.records[0UZ], flushed.phase[0UZ]}) {
            expect(eq(static_cast<std::size_t>(metaNumber(record, "n_averaged")), 2UZ));
            expect(approx(metaNumber(record, "sample_rate"), 48000., 1e-3)) << "on the old rate's axis";
        }
        expect(eq(block->nRecordsDropped(), std::uint64_t{0ULL}));
    };

    "an end-of-stream record the outputs have no room for is counted"_test = [] {
        constexpr std::size_t  length = 1024UZ;
        const std::vector<CF>  input  = complexNoise(2UZ * length, 0x5151ULL);
        const gr::property_map settings{{"window_length", gr::Size_t(length)}, {"max_lag", gr::Size_t(64)}, {"kind", std::string("complex")}, {"n_averages", gr::Size_t(4)}, {"overlap", 0.0}, {"emit_phase", true}, {"sample_rate", 48000.f}};
        for (const auto& [outRoom, phaseRoom] : {std::pair{0UZ, 0UZ}, std::pair{1UZ, 0UZ}, std::pair{1UZ, 1UZ}}) {
            auto block = startedBlock(settings);
            expect(callBlock(*block, std::span<const CF>(input), 0UZ, 4UZ, 4UZ).records.empty()) << "two windows of four complete no record";

            const Call last = callBlock(*block, {}, 2UZ * length, outRoom, phaseRoom, true);
            expect(eq(last.records.size(), outRoom)) << std::format("room {} and {}: the correlation record", outRoom, phaseRoom);
            expect(eq(last.phase.size(), phaseRoom)) << "the phase record";
            const std::uint64_t dropped = (1UZ - outRoom) + (1UZ - phaseRoom);
            expect(eq(block->nRecordsDropped(), dropped)) << "each record without room is counted";
            expect(eq(block->nPartialFlushes(), std::uint64_t{1ULL})) << "the accumulation is flushed once";

            const Call again = callBlock(*block, {}, 2UZ * length, 1UZ, 1UZ, true);
            expect(again.records.empty() && again.phase.empty()) << "a second epilogue publishes nothing";
        }
    };

    "a sample_rate tag inside a chunk takes effect at its own sample"_test = [] {
        constexpr std::size_t length = 1024UZ;
        constexpr std::size_t tagAt  = 4600UZ;
        const std::vector<CF> input  = complexNoise(16384UZ, 0x2222ULL);

        // Both rates arrive as tags, the first on the stream's first sample. The settings leave sample_rate unset. The
        // framework applies a tagged value only to a setting the block was not given by name.
        const gr::property_map first{{gr::tag::SAMPLE_RATE.shortKey(), 48000.f}};
        const gr::property_map second{{gr::tag::SAMPLE_RATE.shortKey(), 96000.f}};
        const gr::property_map geometry{{"window_length", gr::Size_t(length)}, {"max_lag", gr::Size_t(64)}, {"kind", std::string("complex")}, {"n_averages", gr::Size_t(8)}, {"overlap", 0.0}};
        for (const std::size_t burst : {512UZ, 4096UZ, 65536UZ}) {
            const Run<CF> run = collect<CF>(geometry, input, burst, false, {{0UZ, first}, {tagAt, second}});

            expect(eq(run.nWindowResets, std::uint64_t{1ULL})) << std::format("burst {}: the window straddling the change is discarded and counted", burst);
            expect(eq(run.nPartialFlushes, std::uint64_t{2ULL})) << "the change and the end of stream each flush a short record";
            expect(fatal(eq(run.records.size(), 3UZ)));
            const gr::DataSet<float>& flushed = run.records[0UZ];
            expect(eq(static_cast<std::size_t>(metaNumber(flushed, "n_averaged")), 4UZ)) << "four whole windows preceded the change";
            expect(approx(metaNumber(flushed, "sample_rate"), 48000., 1e-3)) << "carried on the old rate's axis";
            expect(approx(static_cast<double>(flushed.axis_values[0UZ][1UZ]), 1. / 48000., 1e-12));
            expect(eq(static_cast<std::size_t>(metaNumber(flushed, "n_samples")), length + 3UZ * length));
            const HandComputed hand = recomputed(flushed);
            expect(approx(metaNumber(flushed, "scatter"), hand.scatter, 1e-12)) << "the short record's scatter follows from its own keys";
            expect(approx(metaNumber(flushed, "detection_threshold"), hand.threshold, 1e-12));

            const gr::DataSet<float>& next = run.records[1UZ];
            expect(eq(static_cast<std::size_t>(metaNumber(next, "sample_start")), tagAt)) << std::format("burst {}: the new grid starts on the tag's sample", burst);
            expect(approx(metaNumber(next, "sample_rate"), 96000., 1e-3)) << "the next record carries the new rate";
            expect(eq(static_cast<std::size_t>(metaNumber(next, "n_averaged")), 8UZ));
            expect(eq(static_cast<std::size_t>(metaNumber(run.records[2UZ], "n_averaged")), (input.size() - tagAt - 8UZ * length) / length)) << "the end of stream flushes the windows after the full record";
            expect(eq(run.nTailDropped, (input.size() - tagAt) % length)) << "the samples past the last window";
            std::println("sample_rate tag, bursts of {}: flushed n_averaged={} at {} Hz, then n_averaged={} at {} Hz from sample {}", burst, metaNumber(flushed, "n_averaged"), metaNumber(flushed, "sample_rate"), metaNumber(next, "n_averaged"), metaNumber(next, "sample_rate"), metaNumber(next, "sample_start"));
        }

        // A tagged rate of zero is counted, and a tagged false_alarm_rate outside its limits is refused. The run
        // completes, and the accepted values stay. The valid arm shows that a false_alarm_rate tag reaches the block.
        const Run<CF> validRun = collect<CF>(geometry, input, 512UZ, false, {{0UZ, first}, {tagAt, gr::property_map{{"false_alarm_rate", 1e-2}}}});
        expect(validRun.completed);
        expect(fatal(eq(validRun.records.size(), 2UZ)));
        expect(approx(metaNumber(validRun.records[1UZ], "false_alarm_rate"), 1e-2, 1e-12)) << "a valid tag moves false_alarm_rate";
        for (const gr::property_map& badTag : {gr::property_map{{gr::tag::SAMPLE_RATE.shortKey(), 0.f}}, gr::property_map{{"false_alarm_rate", 1.5}}}) {
            const std::string key        = badTag.begin()->first.c_str();
            const Run<CF>     refusedRun = collect<CF>(geometry, input, 512UZ, false, {{0UZ, first}, {tagAt, badTag}});
            expect(refusedRun.completed) << std::format("{}: the run completes", key);
            expect(eq(refusedRun.records.size(), 2UZ)) << std::format("{}: two records of eight windows", key);
            expect(eq(refusedRun.nWindowResets, std::uint64_t{0ULL})) << std::format("{}: a refused value discards no window", key);
            expect(eq(refusedRun.nRateRefused, key == "sample_rate" ? std::uint64_t{1ULL} : std::uint64_t{0ULL})) << std::format("{}: the block counts a refused rate alone", key);
            for (const gr::DataSet<float>& record : refusedRun.records) {
                expect(approx(metaNumber(record, "sample_rate"), 48000., 1e-3) && approx(metaNumber(record, "false_alarm_rate"), 1e-3, 1e-12)) << std::format("{}: every record states the accepted values", key);
            }
        }
    };

    "records do not depend on the input chunking, and the window counters add up"_test = [] {
        constexpr std::size_t                sps     = 8UZ;
        constexpr std::size_t                symbols = 5000UZ;                                  // two records of eight windows and a flushed record of two
        constexpr std::size_t                windows = (symbols * sps - 4096UZ) / 2048UZ + 1UZ; // windows of 4096 advance by 2048
        constexpr std::size_t                partial = windows % 8UZ;
        const std::vector<float>             pulse   = raisedCosine(sps, 0.35, 8UZ);
        const std::vector<CF>                scene   = linearModulation(symbols, sps, std::span<const float>(pulse), 0x3333ULL);
        constexpr std::array<std::size_t, 6> kBursts{1UZ, 7UZ, 4095UZ, 4096UZ, 4097UZ, 1UZ << 20U};
        static_assert(partial != 0UZ, "the end of stream leaves a short record to flush");

        std::vector<gr::DataSet<float>> reference;
        for (const std::size_t burst : kBursts) {
            const Run<CF> run = collect<CF>({{"window_length", gr::Size_t(4096)}, {"max_lag", gr::Size_t(256)}, {"n_averages", gr::Size_t(8)}, {"sample_rate", static_cast<float>(sps * 1000UZ)}}, scene, burst);
            expect(fatal(eq(run.records.size(), windows / 8UZ + 1UZ))) << std::format("burst {}: the full records and the flushed one", burst);
            expect(eq(run.nWindows, static_cast<std::uint64_t>(windows))) << std::format("burst {}: every window was taken", burst);
            expect(eq(run.nPartialFlushes, std::uint64_t{1ULL})) << std::format("burst {}: the end of stream flushes one short record", burst);
            expect(eq(run.nPartialWindows, static_cast<std::uint64_t>(partial))) << std::format("burst {}: the windows past the last full record", burst);
            expect(eq(static_cast<std::size_t>(metaNumber(run.records.back(), "n_averaged")), partial)) << std::format("burst {}: the flushed record states its windows", burst);
            expect(eq(run.nWindows, 8ULL * run.nRecords + run.nPartialWindows + run.nDegenerate + run.nNonFinite)) << std::format("burst {}: the window counts add up", burst);
            expect(eq(run.nSamples, scene.size())) << std::format("burst {}: every sample was taken", burst);
            expect(eq(run.nTailDropped, (scene.size() - 4096UZ) % 2048UZ)) << std::format("burst {}: the dropped tail is the samples past the last half-overlapped window", burst);
            expect(eq(run.nRecordsDropped, std::uint64_t{0ULL}));
            if (reference.empty()) {
                reference = run.records;
                continue;
            }
            expect(eq(run.records.size(), reference.size())) << std::format("burst {}: the same number of records", burst);
            for (std::size_t r = 0UZ; r < std::min(run.records.size(), reference.size()); ++r) {
                expect(run.records[r].signal_values == reference[r].signal_values) << std::format("burst {}: record {} is identical", burst, r);
                expect(eq(metaNumber(run.records[r], "sample_start"), metaNumber(reference[r], "sample_start")));
            }
        }
    };

    "the phase port carries the complex kind's phase, and the envelope kind publishes none"_test = [] {
        const std::vector<CF> input     = complexNoise(40000UZ, 0x4444ULL);
        const Run<CF>         withPhase = collect<CF>({{"window_length", gr::Size_t(4096)}, {"max_lag", gr::Size_t(100)}, {"kind", std::string("complex")}, {"emit_phase", true}, {"n_averages", gr::Size_t(4)}, {"sample_rate", 48000.f}}, input, 65536UZ, true);
        expect(fatal(ge(withPhase.phase.size(), 1UZ))) << "the phase port is fed for the complex kind";
        expect(eq(withPhase.phase.size(), withPhase.records.size()));
        const gr::DataSet<float>& correlation = withPhase.records[0UZ];
        const gr::DataSet<float>& phase       = withPhase.phase[0UZ];
        expect(phase.extents == correlation.extents);
        expect(phase.axis_values == correlation.axis_values);
        expect(phase.signal_units[0UZ] == std::string("rad"));
        expect(phase.signal_quantities[0UZ] == std::string("Phase"));
        expect(phase.signal_names[0UZ] == std::string("acf_phase"));
        expect(phase.meta_information[0UZ] == correlation.meta_information[0UZ]) << "both records carry the same metadata";
        for (const float value : phase.signal_values) {
            expect(std::abs(value) <= static_cast<float>(kPi) + 1e-5f);
        }

        const Run<CF> envelope = collect<CF>({{"window_length", gr::Size_t(4096)}, {"max_lag", gr::Size_t(100)}, {"kind", std::string("envelope")}, {"emit_phase", true}, {"n_averages", gr::Size_t(4)}, {"sample_rate", 48000.f}}, input, 65536UZ, true);
        expect(envelope.phase.empty()) << "the envelope kind publishes no phase";
        expect(eq(envelope.records.size(), withPhase.records.size())) << "and the correlation port keeps its record count";
    };

    "without remove_mean the envelope kind publishes an infinite threshold and margin, and the complex kind finite ones"_test = [] {
        const std::vector<CF> input = complexNoise(4UZ * 1024UZ, 0x5A5AULL);
        for (const std::string_view kind : {"envelope", "complex"}) {
            const Run<CF> run = collect<CF>({{"window_length", gr::Size_t(1024)}, {"max_lag", gr::Size_t(16)}, {"kind", std::string(kind)}, {"remove_mean", false}, {"n_averages", gr::Size_t(4)}, {"overlap", 0.0}}, input);
            expect(fatal(eq(run.records.size(), 1UZ))) << std::format("{}: four windows make one record", kind);
            const gr::DataSet<float>& record    = run.records[0UZ];
            const double              threshold = metaNumber(record, "detection_threshold");
            const double              margin    = metaNumber(record, "peak_threshold_db");
            expect(metaNumber(record, "mean_removed") < 0.5) << std::format("{}: the record states the mean was kept", kind);
            if (kind == "envelope") {
                expect(std::isinf(threshold) && threshold > 0.) << std::format("envelope: detection_threshold {} is infinite", threshold);
                expect(std::isinf(margin) && margin > 0.) << std::format("envelope: peak_threshold_db {} is infinite", margin);
            } else {
                expect(std::isfinite(threshold) && threshold > 0.) << std::format("complex: detection_threshold {} is finite", threshold);
                expect(std::isfinite(margin) && margin > 0.) << std::format("complex: peak_threshold_db {} is finite", margin);
            }
        }
    };
};

const suite<"autocorrelation scenes"> _acfScenes = [] {
    "OFDM puts both kinds' peak at the useful symbol length, at the closed-form height"_test = [] {
        // A guard of a quarter symbol. One record of 200 half-overlapped windows covers the scene.
        constexpr std::size_t nFft    = 256UZ;
        constexpr std::size_t nCp     = nFft / 4UZ;
        constexpr std::size_t window  = 1024UZ;
        constexpr std::size_t lags    = nFft + nFft / 8UZ;
        constexpr std::size_t first   = nFft - nFft / 16UZ;
        constexpr std::size_t last    = nFft + nFft / 16UZ;
        constexpr std::size_t symbols = (window + 199UZ * window / 2UZ) / (nFft + nCp) + 1UZ;
        constexpr double      closed  = static_cast<double>(nCp) / static_cast<double>(nCp + nFft);
        const std::vector<CF> scene   = ofdmScene(nFft, nCp, symbols, 0x5555ULL);

        for (const std::string& kind : {std::string("complex"), std::string("envelope")}) {
            const Run<CF> run = collect<CF>({{"window_length", gr::Size_t(window)}, {"max_lag", gr::Size_t(lags)}, {"kind", kind}, {"normalization", std::string("unbiased")}, {"n_averages", gr::Size_t(200)}, {"sample_rate", 30.72e6f}}, scene);
            expect(fatal(ge(run.records.size(), 1UZ))) << kind;
            const gr::DataSet<float>& record = run.records[0UZ];
            const auto [at, height]          = peakOver(record, first, last);
            const double scatter             = metaNumber(record, "scatter");
            std::println("OFDM {} kind, maximum over [{}, {}] at lag {} reading {:.5f} against the closed form {:.5f} (scatter {:.5f})", kind, first, last, at, height, closed, scatter);
            expect(eq(at, nFft)) << std::format("{}: the maximum sits at the useful symbol's length", kind);
            expect(std::abs(height - closed) < 3. * scatter) << std::format("{}: {:.5f} against {:.5f} within three scatter units of {:.5f}", kind, height, closed, scatter);
            const HandComputed hand = recomputed(record);
            expect(approx(metaNumber(record, "detection_threshold"), hand.threshold, 1e-12)) << "the threshold matches the one recomputed from the published keys";
            expect(approx(metaNumber(record, "scatter"), hand.scatter, 1e-12));
            expect(height > metaNumber(record, "detection_threshold")) << "and the peak clears it";
            expect(metaNumber(record, "zero_lag") > 0.) << "the record keeps R(0)";
        }

        const Run<CF> biased = collect<CF>({{"window_length", gr::Size_t(window)}, {"max_lag", gr::Size_t(lags)}, {"kind", std::string("complex")}, {"normalization", std::string("biased")}, {"n_averages", gr::Size_t(200)}, {"sample_rate", 30.72e6f}}, scene);
        expect(fatal(ge(biased.records.size(), 1UZ)));
        const auto [biasedAt, biasedHeight] = peakOver(biased.records[0UZ], first, last);
        const double biasedClosed           = closed * (1. - static_cast<double>(nFft) / static_cast<double>(window));
        std::println("OFDM: under biased the same peak sits at lag {} reading {:.5f} against {:.5f}", biasedAt, biasedHeight, biasedClosed);
        expect(eq(biasedAt, nFft));
        expect(std::abs(biasedHeight - biasedClosed) < 3. * metaNumber(biased.records[0UZ], "scatter"));

        // A carrier offset leaves the magnitude unchanged and shows in the phase. A complex error of rms `s` on a
        // value of height `h` turns its phase by about `s / h` radians.
        constexpr double      offset      = 0.3 / static_cast<double>(nFft);
        const std::vector<CF> offsetScene = ofdmScene(nFft, nCp, symbols, 0x5555ULL, offset);
        const Run<CF>         withOffset  = collect<CF>({{"window_length", gr::Size_t(window)}, {"max_lag", gr::Size_t(lags)}, {"kind", std::string("complex")}, {"emit_phase", true}, {"n_averages", gr::Size_t(200)}, {"sample_rate", 30.72e6f}}, offsetScene, 65536UZ, true);
        expect(fatal(ge(withOffset.records.size(), 1UZ)));
        const auto [offsetAt, offsetHeight] = peakOver(withOffset.records[0UZ], first, last);
        const double offsetScatter          = metaNumber(withOffset.records[0UZ], "scatter");
        expect(eq(offsetAt, nFft));
        expect(std::abs(offsetHeight - closed) < 3. * offsetScatter) << "a carrier offset does not move the magnitude";
        expect(fatal(ge(withOffset.phase.size(), 1UZ)));
        const double measuredPhase = static_cast<double>(withOffset.phase[0UZ].signal_values[nFft]);
        const double expectedPhase = kTwoPi * offset * static_cast<double>(nFft);
        const double phaseBound    = 3. * offsetScatter / offsetHeight;
        std::println("OFDM: with a {:.4f}/T_u offset the phase at lag {} reads {:.5f} rad against {:.5f} (bound {:.5f})", 0.3, nFft, measuredPhase, expectedPhase, phaseBound);
        expect(std::abs(measuredPhase - expectedPhase) < phaseBound) << std::format("phase {:.5f} against {:.5f} within {:.5f}", measuredPhase, expectedPhase, phaseBound);
    };

    "a rectangular-pulse envelope publishes nothing, and a raised cosine on the same symbols does"_test = [] {
        constexpr std::size_t  sps     = 8UZ;
        constexpr std::size_t  symbols = (4096UZ + 7UZ * 2048UZ) / sps + 64UZ; // one record of eight windows
        const std::vector<CF>  flat    = linearModulation(symbols, sps, std::span<const float>{}, 0x6666ULL);
        const gr::property_map settings{{"window_length", gr::Size_t(4096)}, {"max_lag", gr::Size_t(256)}, {"kind", std::string("envelope")}, {"n_averages", gr::Size_t(8)}, {"sample_rate", static_cast<float>(sps * 1000UZ)}};

        const Run<CF> rectangular = collect<CF>(settings, flat);
        expect(rectangular.records.empty()) << "a constant-modulus envelope publishes nothing";
        expect(gt(rectangular.nDegenerate, std::uint64_t{0ULL}));
        expect(eq(rectangular.nDegenerate, rectangular.nWindows)) << "every window is degenerate";
        std::println("rectangular-pulse BPSK, {} windows, {} degenerate, {} records", rectangular.nWindows, rectangular.nDegenerate, rectangular.nRecords);

        const std::vector<float> pulse     = raisedCosine(sps, 0.35, 8UZ);
        const std::vector<CF>    shaped    = linearModulation(symbols, sps, std::span<const float>(pulse), 0x6666ULL);
        const Run<CF>            shapedRun = collect<CF>(settings, shaped);
        expect(fatal(ge(shapedRun.records.size(), 1UZ))) << "the same symbols through a raised cosine have an envelope to correlate";
        const auto [at, height]      = peakOver(shapedRun.records[0UZ], 4UZ, 12UZ);
        const double atPeriod        = static_cast<double>(shapedRun.records[0UZ].signal_values[sps]);
        const double shapedThreshold = metaNumber(shapedRun.records[0UZ], "detection_threshold");
        std::println("rectangular pulse: the same symbols through a raised cosine put a maximum at lag {} reading {:.4f}, and lag {} itself reads {:.4f} against the record's threshold {:.4f}", at, height, sps, atPeriod, shapedThreshold);
        expect(le(at > sps ? at - sps : sps - at, 1UZ)) << "within one lag step of one symbol period";
        expect(atPeriod > shapedThreshold) << "and the lag of one symbol period clears the threshold";
    };

    "shaped BPSK puts the envelope peak at one symbol period, at the height its pulse gives"_test = [] {
        constexpr std::size_t sps     = 8UZ;
        constexpr std::size_t symbols = kOneRecord / sps + 64UZ;
        constexpr std::size_t lags    = 256UZ;
        // At beta = 0 the periodic mean vanishes, and the two pulse-product terms keep the feature.
        for (const double beta : {0., 0.2, 0.35, 0.5}) {
            const std::vector<float>  pulse      = raisedCosine(sps, beta, 8UZ);
            const std::vector<CF>     scene      = linearModulation(symbols, sps, std::span<const float>(pulse), 0x7777ULL);
            const std::vector<double> covariance = shapedEnvelopeCovariance(pulse, sps, lags);
            std::vector<double>       rho(covariance.size());
            std::ranges::transform(covariance, rho.begin(), [&](double value) { return value / covariance[0UZ]; });
            std::size_t closedAt = 4UZ;
            for (std::size_t lag = 4UZ; lag <= 12UZ; ++lag) {
                closedAt = std::abs(rho[lag]) > std::abs(rho[closedAt]) ? lag : closedAt;
            }

            const Run<CF> run = collect<CF>({{"window_length", gr::Size_t(4096)}, {"max_lag", gr::Size_t(lags)}, {"kind", std::string("envelope")}, {"n_averages", gr::Size_t(32)}, {"sample_rate", static_cast<float>(sps * 1000UZ)}}, scene);
            expect(fatal(ge(run.records.size(), 1UZ))) << std::format("beta {}", beta);
            const gr::DataSet<float>& record = run.records[0UZ];
            const auto [at, height]          = peakOver(record, 4UZ, 12UZ);
            const double       atPeriod      = static_cast<double>(record.signal_values[sps]);
            const HandComputed hand          = recomputed(record);
            // the scatter of a white sequence, widened by Bartlett's factor for this envelope's own correlation
            const double spread = metaNumber(record, "scatter") * std::sqrt(bartlettFactor(rho, sps));
            std::println("shaped BPSK: beta {:.2f} puts the maximum at lag {} reading {:.4f}; lag {} reads {:.4f} against {:.4f} from the pulse (+/- {:.4f}), threshold {:.4f}", beta, at, height, sps, atPeriod, std::abs(rho[sps]), 3. * spread, hand.threshold);
            expect(eq(at, closedAt)) << std::format("beta {}: the maximum sits where the pulse puts it", beta);
            expect(std::abs(atPeriod - std::abs(rho[sps])) < 3. * spread) << std::format("beta {}: height {:.4f} against {:.4f} within {:.4f}", beta, atPeriod, std::abs(rho[sps]), 3. * spread);
            expect(approx(metaNumber(record, "detection_threshold"), hand.threshold, 1e-12)) << "the threshold matches the one recomputed from the published keys";
            expect(approx(metaNumber(record, "scatter"), hand.scatter, 1e-12));
            expect(atPeriod > hand.threshold);
        }
    };

    "C4FM publishes nothing unfiltered, and through a Gaussian filter it peaks at the symbol period"_test = [] {
        constexpr std::size_t  sps   = 20UZ;
        constexpr float        rate  = 96000.f;
        constexpr std::size_t  lags  = 128UZ;
        const std::vector<CF>  scene = c4fmScene(kOneRecord / sps + 64UZ, 0x8888ULL); // one record after the longest filter
        const gr::property_map settings{{"window_length", gr::Size_t(4096)}, {"max_lag", gr::Size_t(lags)}, {"kind", std::string("envelope")}, {"n_averages", gr::Size_t(32)}, {"sample_rate", rate}};

        const Run<CF> unfiltered = collect<CF>(settings, scene);
        expect(unfiltered.records.empty()) << "a constant-modulus waveform has no envelope to correlate";
        expect(eq(unfiltered.nDegenerate, unfiltered.nWindows));
        std::println("unfiltered C4FM, {} windows, all degenerate", unfiltered.nWindows);

        // A windowed-sinc channel filter rings at fs/(2 fc), and its envelope autocorrelation then shows that period.
        // A Gaussian filter does not ring.
        constexpr std::array<std::pair<double, std::size_t>, 4> kBandwidths{{{6250., 21UZ}, {5000., 25UZ}, {4000., 31UZ}, {3000., 41UZ}}};
        for (const auto& [bandwidth, taps] : kBandwidths) {
            const std::vector<float> channel  = gr::filter::fir::design::gaussianPulse(taps, static_cast<double>(sps), bandwidth / 4800.);
            const std::vector<CF>    filtered = applyFir(std::span<const CF>(scene), std::span<const float>(channel));
            const Run<CF>            run      = collect<CF>(settings, filtered);
            expect(fatal(ge(run.records.size(), 1UZ))) << std::format("{:.0f} Hz: a filter converts frequency modulation to amplitude modulation", bandwidth);
            const gr::DataSet<float>& record = run.records[0UZ];
            const auto [at, height]          = peakOver(record, sps - 6UZ, sps + 6UZ);
            const bool isLocal               = at > 0UZ && at + 1UZ < record.signal_values.size() && record.signal_values[at] > record.signal_values[at - 1UZ] && record.signal_values[at] > record.signal_values[at + 1UZ];
            std::println("C4FM: Gaussian channel filter at {:.0f} Hz ({} taps): maximum over [{}, {}] at lag {} reading {:.4f}, a local maximum: {}; threshold {:.4f}", bandwidth, channel.size(), sps - 6UZ, sps + 6UZ, at, height, isLocal, metaNumber(record, "detection_threshold"));
            if (bandwidth == 6250.) {
                expect(le(at > sps ? at - sps : sps - at, 1UZ)) << std::format("within one lag step of {} samples, 208.333 us at {} S/s", sps, static_cast<std::size_t>(rate));
                expect(isLocal) << "and it is a local maximum";
                expect(height > metaNumber(record, "detection_threshold")) << "and it clears the threshold";
            }
        }
    };

    "Mode S against its exact closed forms"_test = [] {
        // the preamble alone, a deterministic sequence with an exact expected value
        Rng                   rng{0x9999ULL};
        const std::vector<CF> preamble = modeSFrame(rng, true, false);
        expect(eq(preamble.size(), 16UZ));

        gr::analysis::AcfConfig preambleConfig{};
        preambleConfig.windowLength = 16UZ;
        preambleConfig.maxLag       = 8UZ;
        preambleConfig.kind         = gr::analysis::AcfKind::Envelope;
        preambleConfig.overlap      = 0.;
        preambleConfig.nAverages    = 1UZ;
        gr::analysis::Autocorrelation<CF> kernel;
        kernel.prepare(preambleConfig);
        std::vector<float> preambleMagnitude;
        std::ignore = kernel.process(std::span<const CF>(preamble), [&preambleMagnitude](const gr::analysis::AcfResult& result) { preambleMagnitude.assign(result.magnitude.begin(), result.magnitude.end()); });
        expect(eq(preambleMagnitude.size(), 9UZ));
        // The mean-removed sample correlation of the window at lag 2 is 3/7 of its zero lag. Under the unbiased
        // normalization the estimator adds 1/(N-1) = 1/15 to it.
        const double preambleExpected = 3. / 7. + 1. / 15.;
        std::println("Mode S: the preamble alone reads |R(1.0 us)|/R(0) = {:.6f} against 3/7 + 1/15 = {:.6f}", preambleMagnitude[2UZ], preambleExpected);
        expect(std::abs(static_cast<double>(preambleMagnitude[2UZ]) - preambleExpected) < 1e-5) << std::format("{:.7f} against {:.7f}", preambleMagnitude[2UZ], preambleExpected);

        // the data field alone, averaged over 200 seeded frames
        constexpr std::size_t frames = 200UZ;
        std::vector<CF>       dataFields;
        for (std::size_t frame = 0UZ; frame < frames; ++frame) {
            const std::vector<CF> field = modeSFrame(rng, false, true);
            dataFields.insert(dataFields.end(), field.begin(), field.end());
        }
        expect(eq(dataFields.size(), frames * 224UZ));
        const Run<CF> data = collect<CF>({{"window_length", gr::Size_t(224)}, {"max_lag", gr::Size_t(12)}, {"kind", std::string("envelope")}, {"overlap", 0.0}, {"n_averages", gr::Size_t(frames)}, {"sample_rate", 2.0e6f}}, dataFields);
        expect(fatal(ge(data.records.size(), 1UZ)));
        const gr::DataSet<float>& dataRecord = data.records[0UZ];
        const double              scatter    = metaNumber(dataRecord, "scatter");
        const double              atHalf     = static_cast<double>(dataRecord.signal_values[1UZ]);
        const double              atOne      = static_cast<double>(dataRecord.signal_values[2UZ]);
        std::println("Mode S: the data field alone reads |R(0.5 us)|/R(0) = {:.5f} against 0.5 and |R(1.0 us)|/R(0) = {:.5f} against 0, scatter {:.5f}", atHalf, atOne, scatter);
        const HandComputed hand = recomputed(dataRecord);
        expect(std::abs(atHalf - 0.5) < 3. * scatter) << std::format("{:.5f} against 0.5 within three scatter units of {:.5f}", atHalf, scatter);
        // The null is held to the record's own detection threshold. A zero-mean real estimate has a folded normal
        // magnitude. That magnitude exceeds three scatter units in about one realization in 370.
        expect(atOne < hand.threshold) << std::format("the derived null reads {:.5f} ({:.2f} scatter units) against the record's threshold {:.5f}", atOne, atOne / scatter, hand.threshold);
        expect(approx(metaNumber(dataRecord, "detection_threshold"), hand.threshold, 1e-12)) << "the threshold matches the one recomputed from the published keys";
        expect(atHalf > hand.threshold) << "the lag of 0.5 us clears the threshold in the same run";

        // the whole 240-sample frame blends the two in the ratio 224:16
        std::vector<CF> whole;
        for (std::size_t frame = 0UZ; frame < frames; ++frame) {
            const std::vector<CF> field = modeSFrame(rng, true, true);
            whole.insert(whole.end(), field.begin(), field.end());
        }
        const Run<CF> wholeRun = collect<CF>({{"window_length", gr::Size_t(240)}, {"max_lag", gr::Size_t(16)}, {"kind", std::string("envelope")}, {"overlap", 0.0}, {"n_averages", gr::Size_t(frames)}, {"sample_rate", 2.0e6f}}, whole);
        expect(fatal(ge(wholeRun.records.size(), 1UZ)));
        std::println("Mode S: the whole 240-sample frame reads |R(0.5 us)|/R(0) = {:.5f} and |R(1.0 us)|/R(0) = {:.5f}, scatter {:.5f}", wholeRun.records[0UZ].signal_values[1UZ], wholeRun.records[0UZ].signal_values[2UZ], metaNumber(wholeRun.records[0UZ], "scatter"));
    };

    "PeakDetect over the record, against the derived threshold and false-alarm rate"_test = [] {
        constexpr std::size_t    sps   = 8UZ;
        constexpr float          rate  = static_cast<float>(sps * 1000UZ);
        constexpr std::size_t    lags  = 256UZ;
        constexpr double         pFa   = 1e-3;
        const std::vector<float> pulse = raisedCosine(sps, 0.35, 8UZ);
        const std::vector<CF>    scene = linearModulation(kOneRecord / sps + 64UZ, sps, std::span<const float>(pulse), 0xAAAAULL);

        const gr::property_map shapedSettings{{"window_length", gr::Size_t(4096)}, {"max_lag", gr::Size_t(lags)}, {"kind", std::string("envelope")}, {"n_averages", gr::Size_t(32)}, {"false_alarm_rate", pFa}, {"sample_rate", rate}};

        /// @brief Runs the estimate into a peak detector at @p thresholdDb over the median. Returns the detections.
        const auto detect = [](gr::property_map settings, std::vector<CF> samples, double thresholdDb) {
            gr::Graph graph;
            auto&     source   = graph.emplaceBlock<BurstSource<CF>>();
            auto&     acf      = graph.emplaceBlock<Autocorrelation<CF>>(std::move(settings));
            auto&     detector = graph.emplaceBlock<PeakDetect>({{"threshold_db", thresholdDb}, {"reference", std::string("above_median")}, {"min_distance_hz", 0.0}});
            auto&     sink     = graph.emplaceBlock<RecordSink>();
            source.samples     = std::move(samples);
            expect(graph.connect<"out", "in">(source, acf).has_value());
            expect(graph.connect<"out", "in">(acf, detector).has_value());
            expect(graph.connect<"out", "in">(detector, sink).has_value());
            gr::scheduler::Simple scheduler;
            if (!scheduler.exchange(std::move(graph)).has_value()) {
                expect(false) << "the scheduler takes the graph";
                return std::vector<gr::DataSet<float>>{};
            }
            std::ignore = scheduler.runAndWait(); // the source ends the stream
            return std::move(sink.records);
        };

        /// @brief The distance in seconds from @p wanted to the nearest detection of @p records, and the detection count.
        const auto nearest = [](const std::vector<gr::DataSet<float>>& records, double wanted) {
            std::pair<double, std::size_t> result{1., 0UZ};
            if (records.empty()) {
                return result;
            }
            result.second = static_cast<std::size_t>(records[0UZ].extents[0UZ]);
            for (std::size_t k = 0UZ; k < result.second; ++k) {
                result.first = std::min(result.first, std::abs(static_cast<double>(records[0UZ].signal_values[k]) - wanted));
            }
            return result;
        };

        /// @brief The decibel margin of a record's largest value over its own median. A detector reading
        /// `above_median` compares this quantity against its threshold.
        const auto marginDb = [](const gr::DataSet<float>& record) {
            std::vector<float> sorted(record.signal_values);
            if (sorted.size() < 3UZ) {
                return 0.;
            }
            double top = 0.; // lag zero is one by construction and is left out
            for (std::size_t k = 1UZ; k < sorted.size(); ++k) {
                top = std::max(top, static_cast<double>(sorted[k]));
            }
            const std::size_t middle = sorted.size() / 2UZ;
            std::nth_element(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(middle), sorted.end());
            const double median = static_cast<double>(sorted[middle]);
            return median > 0. ? 10. * std::log10(top / median) : 0.;
        };

        // A sinusoidal envelope modulation at 8.4 samples has its correlation maximum at 8.4, between two published
        // lags. The detector's parabolic interpolation places it.
        //
        // This arm states its margin in place of the record's derived figure. The modulation correlates at every lag,
        // and the record holds the same cosine from lag 1 to lag L. The median of its magnitude stands
        // 10 log10(1/cos(pi/4)) = 1.51 dB under the peak, below a margin derived for a noise median. This arm tests the
        // interpolation, and the third arm tests the realized false-alarm rate.
        {
            constexpr double period    = 8.4;
            constexpr double statedDb  = 1.0;
            std::vector<CF>  modulated = complexNoise(kOneRecord, 0xCCCCULL);
            for (std::size_t k = 0UZ; k < modulated.size(); ++k) {
                modulated[k] *= static_cast<float>(std::sqrt(1. + 0.9 * std::cos(kTwoPi * static_cast<double>(k) / period)));
            }
            const Run<CF> plain = collect<CF>(shapedSettings, modulated);
            expect(fatal(ge(plain.records.size(), 1UZ)));

            const std::vector<gr::DataSet<float>> detections = detect(shapedSettings, modulated, statedDb);
            const auto [closest, count]                      = nearest(detections, period / static_cast<double>(rate));
            std::println("PeakDetect: a {:.1f}-sample sinusoidal envelope modulation stands {:.3f} dB over its record's own median against the derived {:.3f} dB; at a stated {:.1f} dB it gives {} detections and the nearest sits {:.4f} lag steps from the period after parabolic interpolation", period, marginDb(plain.records[0UZ]), metaNumber(plain.records[0UZ], "peak_threshold_db"), statedDb, count, closest * static_cast<double>(rate));
            expect(ge(detections.size(), 1UZ)) << "the injected period is reported as a detection";
            expect(closest <= 0.5 / static_cast<double>(rate)) << "within half a lag step";
        }

        // The symbol period of a shaped stream. Its envelope feature is a negative lobe, and the magnitude peaks
        // below the period. The detector fits a parabola to the decibel levels around the maximum. The expected
        // position is that fit over the levels the pulse gives, and its bound is the largest move of the fit when
        // each level moves by three of its own standard deviations.
        {
            const Run<CF> plain = collect<CF>(shapedSettings, scene);
            expect(fatal(ge(plain.records.size(), 1UZ)));
            const std::vector<double> covariance = shapedEnvelopeCovariance(pulse, sps, lags);
            std::vector<double>       rho(covariance.size());
            std::ranges::transform(covariance, rho.begin(), [&](double value) { return std::abs(value / covariance[0UZ]); });
            const auto peak   = static_cast<std::size_t>(std::distance(rho.begin(), std::max_element(rho.begin() + 4, rho.begin() + 13)));
            const auto vertex = [](double left, double center, double right) {
                const double l = 10. * std::log10(left);
                const double c = 10. * std::log10(center);
                const double r = 10. * std::log10(right);
                return 0.5 * (l - r) / (l - 2. * c + r);
            };
            const double expected = static_cast<double>(peak) + vertex(rho[peak - 1UZ], rho[peak], rho[peak + 1UZ]);
            const double spread   = 3. * metaNumber(plain.records[0UZ], "scatter") * std::sqrt(std::max({bartlettFactor(rho, peak - 1UZ), bartlettFactor(rho, peak), bartlettFactor(rho, peak + 1UZ)}));
            double       bound    = 0.;
            for (const double left : {-spread, spread}) {
                for (const double center : {-spread, spread}) {
                    for (const double right : {-spread, spread}) {
                        bound = std::max(bound, std::abs(static_cast<double>(peak) + vertex(rho[peak - 1UZ] + left, rho[peak] + center, rho[peak + 1UZ] + right) - expected));
                    }
                }
            }
            const double                          thresholdDb = metaNumber(plain.records[0UZ], "peak_threshold_db");
            const std::vector<gr::DataSet<float>> detections  = detect(shapedSettings, scene, thresholdDb);
            const auto [closest, count]                       = nearest(detections, expected / static_cast<double>(rate));
            std::println("PeakDetect: over shaped BPSK the record stands {:.3f} dB over its own median against the derived {:.3f} dB; {} detections, the closest sits {:.3f} lag steps from the {:.3f} the pulse gives (bound {:.3f})", marginDb(plain.records[0UZ]), thresholdDb, count, closest * static_cast<double>(rate), expected, bound);
            expect(ge(detections.size(), 1UZ)) << "the symbol period is reported as a detection";
            expect(closest <= bound / static_cast<double>(rate)) << std::format("the detection lies within {:.3f} lag steps of the position the pulse gives", bound);
        }

        // the realized false-alarm rate on noise alone
        {
            // The noise records average four windows. On one window of 128 samples the envelope estimate's tail is
            // heavier than the Gaussian tail the threshold models. At 16 lags the lag scatters are nearly equal, and the
            // margin must cover the scatter of the median. At 64 lags the unbiased scatter grows by the root of two.
            constexpr std::size_t noiseWindow  = 128UZ;
            constexpr std::size_t noiseRecords = 2000UZ;
            constexpr std::size_t averages     = 4UZ;
            for (const std::size_t noiseLags : {noiseWindow / 8UZ, noiseWindow / 2UZ}) {
                for (const std::string& kind : {std::string("complex"), std::string("envelope")}) {
                    const gr::property_map settings{{"window_length", gr::Size_t(noiseWindow)}, {"max_lag", gr::Size_t(noiseLags)}, {"kind", kind}, {"n_averages", gr::Size_t(averages)}, {"overlap", 0.0}, {"false_alarm_rate", pFa}, {"sample_rate", 48000.f}};
                    const Run<CF>          probe = collect<CF>(settings, complexNoise(noiseWindow * averages * 8UZ + 16UZ, 0xDDDDULL));
                    expect(fatal(ge(probe.records.size(), 1UZ)));
                    const double thresholdDb = metaNumber(probe.records[0UZ], "peak_threshold_db");

                    const std::vector<gr::DataSet<float>> detections = detect(settings, complexNoise(noiseRecords * averages * noiseWindow + 16UZ, 0xBBBBULL + noiseLags), thresholdDb);
                    const double                          measured   = static_cast<double>(detections.size()) / static_cast<double>(noiseRecords);
                    const double                          spread     = 3. * std::sqrt(pFa * (1. - pFa) / static_cast<double>(noiseRecords));
                    std::println("PeakDetect: {} kind, {} lags, at {:.3f} dB over the median, {} of {} noise records produced a detection: rate {:.5f} against the design {:.5f} (+/- {:.5f})", kind, noiseLags, thresholdDb, detections.size(), noiseRecords, measured, pFa, spread);
                    expect(measured <= pFa + spread) << std::format("{} at {} lags: realized rate {:.5f} against design {:.5f}", kind, noiseLags, measured, pFa);
                }
            }
        }
    };
};

int main() { /* not needed for UT */ }
