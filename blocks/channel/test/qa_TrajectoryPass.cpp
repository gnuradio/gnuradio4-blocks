#include <boost/ut.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <limits>
#include <memory>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/algorithm/timing/FrequencySchedule.hpp>
#include <gnuradio-4.0/channel/Awgn.hpp>
#include <gnuradio-4.0/channel/DopplerShift.hpp>
#include <gnuradio-4.0/channel/RangeDelay.hpp>

/*
 * The two-block pass. A trajectory's envelope delay and carrier shift are applied from one table and removed with
 * the same table. A modem sits around them and must decode as if there were no pass. The chain is apply-delay,
 * apply-shift, noise, correct-shift, correct-delay. The last applied is the first removed. Each correcting block
 * then sees the sample index its applying twin saw. The remaining non-commuting term is the delay block reading
 * its schedule tau seconds late. That term is far below a sample here.
 *
 * The modem is band-limited on purpose. A fractional delay line interpolates inside its design band and distorts
 * what lies beyond it. A full-band stream through two lines would measure the bank's stopband and not the blocks'
 * composition. Two samples a symbol under a root-raised-cosine pulse keep the occupied band at a third of the
 * sample rate. That band is inside the line's passband with margin.
 */

namespace {

using gr::blocks::channel::AwgnChannel;
using gr::blocks::channel::DopplerShift;
using gr::blocks::channel::RangeDelay;
using C = std::complex<float>;

constexpr double        kFs             = 48'000.;
constexpr double        kCarrierHz      = 437e6;
constexpr double        kSeconds        = 600.; // the ten-minute pass the table covers
constexpr double        kRunStart       = 240.; // the run's two minutes, centered on closest approach
constexpr double        kRunSeconds     = 120.;
constexpr double        kKnotSeconds    = 20.; // three knots a minute
constexpr double        kRangeAtClosest = 500e3;
constexpr double        kSpeed          = 7000.;
constexpr std::size_t   kSps            = 2UZ;
constexpr double        kRolloff        = 0.35;
constexpr std::size_t   kSpanSymbols    = 6UZ;
constexpr double        kNoise          = 0.35; // total complex noise power, at which BPSK of unit energy reads about 8e-3
constexpr std::size_t   kChunk          = 65'536UZ;
constexpr std::uint64_t kSeed           = 20260902ULL;

/// The blocks carry a measurement slot and atomic counters. They cannot move and stay where they are built.
template<typename TBlock>
[[nodiscard]] std::unique_ptr<TBlock> configured(gr::property_map settings) {
    auto block = std::make_unique<TBlock>(std::move(settings));
    block->settings().init();
    std::ignore = block->settings().applyStagedParameters();
    block->start();
    return block;
}

struct Trajectory {
    std::vector<std::int64_t> times{};
    std::vector<double>       delays{};
    std::vector<double>       offsets{};
};

/// A pass over a flat ground track. The slant range is sqrt(R0^2 + (v (t - T/2))^2), and its rate is
/// v (t - T/2) v / R. The pass closes through the first half and recedes through the second. The rate changes
/// fastest at the point of closest approach.
[[nodiscard]] Trajectory leoPass() {
    Trajectory pass;
    const int  knots = static_cast<int>(std::llround(kSeconds / kKnotSeconds));
    for (int k = 0; k <= knots; ++k) {
        const double t     = kSeconds * static_cast<double>(k) / static_cast<double>(knots);
        const double along = kSpeed * (t - 0.5 * kSeconds);
        const double range = std::sqrt(kRangeAtClosest * kRangeAtClosest + along * along);
        pass.times.push_back(std::llround(t * 1e9));
        pass.delays.push_back(gr::timing::delayFor(range));
        pass.offsets.push_back(gr::timing::offsetFor(kSpeed * along / range, kCarrierHz));
    }
    return pass;
}

/// The root-raised-cosine pulse at kSps samples a symbol, normalized to unit energy. A symbol's matched-filter peak
/// reads 1. The noise after the filter keeps its power per sample.
[[nodiscard]] std::vector<float> rrcTaps() {
    const std::size_t   taps = kSpanSymbols * kSps + 1UZ;
    std::vector<double> h(taps);
    const double        beta = kRolloff;
    const double        pi   = std::numbers::pi;
    for (std::size_t k = 0UZ; k < taps; ++k) {
        const double t = (static_cast<double>(k) - 0.5 * static_cast<double>(taps - 1UZ)) / static_cast<double>(kSps);
        if (std::abs(t) < 1e-9) {
            h[k] = 1. - beta + 4. * beta / pi;
        } else if (std::abs(std::abs(t) - 1. / (4. * beta)) < 1e-9) {
            h[k] = beta / std::sqrt(2.) * ((1. + 2. / pi) * std::sin(pi / (4. * beta)) + (1. - 2. / pi) * std::cos(pi / (4. * beta)));
        } else {
            h[k] = (std::sin(pi * t * (1. - beta)) + 4. * beta * t * std::cos(pi * t * (1. + beta))) / (pi * t * (1. - std::pow(4. * beta * t, 2.)));
        }
    }
    double energy = 0.;
    for (const double v : h) {
        energy += v * v;
    }
    std::vector<float> out(taps);
    for (std::size_t k = 0UZ; k < taps; ++k) {
        out[k] = static_cast<float>(h[k] / std::sqrt(energy));
    }
    return out;
}

struct Bits {
    std::uint64_t state = 0x9E3779B97F4A7C15ULL;

    [[nodiscard]] std::uint8_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return static_cast<std::uint8_t>(state & 1ULL);
    }
};

/// Bits at kSps samples a symbol through the pulse, produced a chunk at a time. Every sent bit is kept.
struct Transmitter {
    std::vector<float>        h = rrcTaps();
    std::vector<std::uint8_t> bits{};
    Bits                      source{};
    std::size_t               produced = 0UZ;

    void fill(std::span<C> out) {
        for (std::size_t i = 0UZ; i < out.size(); ++i) {
            const std::size_t s = produced + i;
            while (bits.size() * kSps <= s) {
                bits.push_back(source.next());
            }
            double acc = 0.;
            for (std::size_t j = 0UZ; j < h.size() && j <= s; ++j) {
                const std::size_t u = s - j;
                if (u % kSps == 0UZ) {
                    acc += static_cast<double>(h[j]) * (bits[u / kSps] != 0U ? 1. : -1.);
                }
            }
            out[i] = C(static_cast<float>(acc), 0.f);
        }
        produced += out.size();
    }
};

/// The streamed matched filter and hard decision. Symbol m is decided on filtered sample m * kSps + lag.
struct Receiver {
    std::vector<float>               h = rrcTaps();
    std::vector<C>                   history{};
    std::size_t                      consumed    = 0UZ;
    std::size_t                      lag         = 0UZ;
    std::size_t                      skipSymbols = 0UZ;
    const std::vector<std::uint8_t>* bits        = nullptr;
    std::uint64_t                    errors      = 0ULL;
    std::uint64_t                    counted     = 0ULL;

    void feed(std::span<const C> in) {
        const std::size_t H = h.size() - 1UZ;
        std::vector<C>    buf;
        buf.reserve(history.size() + in.size());
        buf.insert(buf.end(), history.begin(), history.end());
        buf.insert(buf.end(), in.begin(), in.end());
        const std::size_t base = history.size(); // buf index of the first sample of this call
        for (std::size_t i = 0UZ; i < in.size(); ++i) {
            const std::size_t g = consumed + i;
            if (g < lag || (g - lag) % kSps != 0UZ) {
                continue;
            }
            const std::size_t m = (g - lag) / kSps;
            if (m >= bits->size()) {
                break;
            }
            double acc = 0.;
            for (std::size_t j = 0UZ; j < h.size() && j <= base + i; ++j) {
                acc += static_cast<double>(h[j]) * static_cast<double>(buf[base + i - j].real());
            }
            if (m >= skipSymbols) {
                ++counted;
                errors += ((acc >= 0.) != ((*bits)[m] != 0U)) ? 1ULL : 0ULL;
            }
        }
        consumed += in.size();
        const std::size_t keep = std::min(H, buf.size());
        history.assign(buf.end() - static_cast<std::ptrdiff_t>(keep), buf.end());
    }

    [[nodiscard]] double ber() const noexcept { return counted == 0ULL ? 1. : static_cast<double>(errors) / static_cast<double>(counted); }
};

enum class Chain { clean, corrected, delayLeftIn };

struct Outcome {
    double        ber         = 1.;
    std::uint64_t counted     = 0ULL;
    double        lagSamples  = 0.;
    double        latency     = 0.;
    double        biasSamples = 0.;
    std::size_t   transient   = 0UZ;
};

[[nodiscard]] Outcome run(const Trajectory& pass, Chain chain, double seconds) {
    const std::size_t samples = static_cast<std::size_t>(kFs * seconds);
    Transmitter       tx;
    Receiver          rx;
    Outcome           outcome;

    const auto settings = [&](std::string direction, gr::property_map extra = {}) {
        gr::property_map map{{"sample_rate", static_cast<float>(kFs)}, {"schedule_times_ns", pass.times}, {"direction", std::move(direction)}, {"anchor_ns", static_cast<std::int64_t>(std::llround(kRunStart * 1e9))}};
        for (auto& [key, value] : extra) {
            map[key] = value;
        }
        return map;
    };

    auto applyDelay   = configured<RangeDelay<C>>(settings("apply", {{"schedule_delays_s", pass.delays}}));
    auto applyShift   = configured<DopplerShift<C>>(settings("apply", {{"schedule_offsets_hz", pass.offsets}}));
    auto noise        = configured<AwgnChannel<C>>({{"noise_power", kNoise}, {"seed", kSeed}});
    auto correctShift = configured<DopplerShift<C>>(settings("correct", {{"schedule_offsets_hz", pass.offsets}}));

    // The correcting line's bias makes the chain's whole lag an integer number of samples. The whole lag is both
    // commanded delays, which sum to the bias, plus both lines' latencies. The decision instant then lands on a
    // sample.
    auto         probe        = configured<RangeDelay<C>>(settings("correct", {{"schedule_delays_s", pass.delays}}));
    const double latency      = probe->latencySamples();
    const double maxDelay     = *std::max_element(pass.delays.begin(), pass.delays.end());
    const double biasSamples  = std::ceil(maxDelay * kFs + 2. * latency) - 2. * latency;
    auto         correctDelay = configured<RangeDelay<C>>(settings("correct", {{"schedule_delays_s", pass.delays}, {"bias_s", biasSamples / kFs}}));

    const std::size_t filterLag = tx.h.size() - 1UZ; // the pulse and the matched filter, each centered
    double            lag       = static_cast<double>(filterLag);
    if (chain == Chain::corrected) {
        lag += biasSamples + 2. * latency;
    } else if (chain == Chain::delayLeftIn) {
        lag += applyDelay->currentDelaySamples() + latency; // the best whole-sample alignment at the start of the run
    }
    outcome.lagSamples  = lag;
    outcome.latency     = latency;
    outcome.biasSamples = biasSamples;
    outcome.transient   = chain == Chain::clean ? 0UZ : applyDelay->historySamples() + correctDelay->historySamples();

    rx.lag         = static_cast<std::size_t>(std::llround(lag));
    rx.skipSymbols = (outcome.transient + rx.lag) / kSps + 1UZ; // the lines start with empty histories
    rx.bits        = &tx.bits;

    std::vector<C> a(kChunk);
    std::vector<C> b(kChunk);
    for (std::size_t at = 0UZ; at < samples; at += kChunk) {
        const std::size_t n = std::min(kChunk, samples - at);
        tx.fill(std::span<C>(a.data(), n));
        if (chain == Chain::clean) {
            std::ignore = noise->processBulk(std::span<const C>(a.data(), n), std::span<C>(b.data(), n));
            rx.feed(std::span<const C>(b.data(), n));
            continue;
        }
        std::ignore = applyDelay->processBulk(std::span<const C>(a.data(), n), std::span<C>(b.data(), n));
        std::ignore = applyShift->processBulk(std::span<const C>(b.data(), n), std::span<C>(a.data(), n));
        std::ignore = noise->processBulk(std::span<const C>(a.data(), n), std::span<C>(b.data(), n));
        std::ignore = correctShift->processBulk(std::span<const C>(b.data(), n), std::span<C>(a.data(), n));
        if (chain == Chain::corrected) {
            std::ignore = correctDelay->processBulk(std::span<const C>(a.data(), n), std::span<C>(b.data(), n));
            rx.feed(std::span<const C>(b.data(), n));
        } else {
            rx.feed(std::span<const C>(a.data(), n));
        }
    }
    outcome.ber     = rx.ber();
    outcome.counted = rx.counted;
    return outcome;
}

} // namespace

const boost::ut::suite<"trajectory pass"> trajectoryPassTests = [] {
    using namespace boost::ut;

    // The two blocks compose. A pass applied and corrected around a modem decodes as if there were no pass. The
    // same chain with the envelope delay left in does not decode at all. The run covers closest approach, where the
    // Doppler shift sweeps through zero at its fastest and the delay falls and rises again.
    "a corrected pass decodes at the clean error rate, and the delay left in destroys the link"_test = [] {
        const Trajectory pass = leoPass();

        // The run starts and ends on knots. The delay is linear between knots, and the knots inside the run bound it.
        double minDelay = std::numeric_limits<double>::max();
        double maxDelay = 0.;
        for (std::size_t k = 0UZ; k < pass.times.size(); ++k) {
            const double t = static_cast<double>(pass.times[k]) * 1e-9;
            if (t >= kRunStart && t <= kRunStart + kRunSeconds) {
                minDelay = std::min(minDelay, pass.delays[k]);
                maxDelay = std::max(maxDelay, pass.delays[k]);
            }
        }
        const double excursion = (maxDelay - minDelay) * kFs;
        const double radial    = kSpeed * kRunSeconds / 299'792'458. * kFs; // the straight-line bound a radial approach would move it

        const auto    start     = std::chrono::steady_clock::now();
        const Outcome corrected = run(pass, Chain::corrected, kRunSeconds);
        const Outcome clean     = run(pass, Chain::clean, kRunSeconds);
        const Outcome leftIn    = run(pass, Chain::delayLeftIn, 0.5 * kRunSeconds); // the delay falls through its whole excursion in the first minute
        const double  elapsed   = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

        // The correcting blocks rotate and interpolate the noise as well as the signal. The two runs draw from one noise
        // distribution and get different realizations. The agreement is statistical. Its bound is the counting error.
        const double sigma = std::sqrt(clean.ber * (1. - clean.ber) / static_cast<double>(clean.counted));
        std::println("[pass] corrected pass: {} symbols over {} s of the pass from {} s at {} Hz; envelope delay moves {:.1f} samples ({:.1f} for a radial approach at {} m/s), chain lag {:.0f} samples "
                     "(bias {:.5f} + 2 x latency {:.5f} + filters), transient {} samples skipped",
            clean.counted, kRunSeconds, kRunStart, kFs, excursion, radial, kSpeed, corrected.lagSamples, corrected.biasSamples, corrected.latency, corrected.transient);
        std::println("[pass] BER {:.6f} corrected against {:.6f} clean, difference {:.2e} at one sigma {:.2e}; delay left in {:.4f} over the first minute; {:.2f} s", corrected.ber, clean.ber, corrected.ber - clean.ber, sigma, leftIn.ber, elapsed);

        expect(gt(excursion, 10. * static_cast<double>(kSps))) << "the delay must move by many symbols over the run for a fixed alignment to be unable to serve";
        expect(lt(std::abs(corrected.biasSamples + 2. * corrected.latency - std::round(corrected.biasSamples + 2. * corrected.latency)), 1e-6)) << "the chain's lag is integral by construction";
        expect(gt(clean.ber, 1e-3)) << "the reference run must actually be making errors for the comparison to mean anything";
        expect(lt(std::abs(corrected.ber - clean.ber), 5. * sigma)) << std::format("BER {:.6f} corrected against {:.6f} clean", corrected.ber, clean.ber);
        expect(gt(leftIn.ber, 0.3)) << std::format("with the envelope delay left in the link must fail; it read {:.4f}", leftIn.ber);
        expect(gt(leftIn.ber, 10. * corrected.ber)) << "the negative control and the corrected chain must differ by more than their noise";
    };
};

int main() { /* not needed for UT */ }
