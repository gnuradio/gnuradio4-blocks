#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/algorithm/fourier/PhaseVocoder.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>

namespace {

using CF = std::complex<float>;

/// A complex exponential at `cycles` cycles per sample, unit amplitude.
[[nodiscard]] std::vector<CF> tone(std::size_t count, double cycles, double amplitude = 1.0) {
    std::vector<CF> out(count);
    for (std::size_t n = 0UZ; n < count; ++n) {
        const double phase = 2.0 * std::numbers::pi * cycles * static_cast<double>(n);
        out[n]             = static_cast<float>(amplitude) * CF{static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))};
    }
    return out;
}

/// The frequency (cycles per sample, in [0, 1)) of the strongest bin of a windowed transform.
[[nodiscard]] double strongestBin(std::span<const CF> x) {
    std::size_t size = 1UZ;
    while (size * 2UZ <= x.size()) {
        size *= 2UZ;
    }
    gr::algorithm::FFT<CF> fft;
    std::vector<CF>        windowed(size);
    for (std::size_t i = 0UZ; i < size; ++i) {
        const double w = 0.5 - 0.5 * std::cos(2.0 * std::numbers::pi * static_cast<double>(i) / static_cast<double>(size));
        windowed[i]    = x[x.size() - size + i] * static_cast<float>(w);
    }
    std::vector<CF> spectrum(size);
    fft.compute(windowed, spectrum);
    std::size_t best = 0UZ;
    for (std::size_t k = 1UZ; k < size; ++k) {
        if (std::abs(spectrum[k]) > std::abs(spectrum[best])) {
            best = k;
        }
    }
    return static_cast<double>(best) / static_cast<double>(size);
}

/// A complex tone's amplitude equals its RMS. The level is measured without window calibration.
[[nodiscard]] double rmsDb(std::span<const CF> x) {
    double power = 0.0;
    for (const CF v : x) {
        power += static_cast<double>(std::norm(v));
    }
    return 10.0 * std::log10(power / static_cast<double>(x.size()));
}

/// The energy outside the best-fit complex exponential at `cycles` cycles per sample, in dB below the energy of
/// the whole. The fit is one complex amplitude. A gain or a phase offset of the tone counts as part of the fit.
[[nodiscard]] double residualDb(std::span<const CF> x, double cycles) {
    std::complex<double> amplitude{};
    double               energy = 0.0;
    for (std::size_t n = 0UZ; n < x.size(); ++n) {
        const double turn = 2.0 * std::numbers::pi * std::fmod(cycles * static_cast<double>(n), 1.0);
        amplitude += std::complex<double>(x[n]) * std::polar(1.0, -turn);
        energy += static_cast<double>(std::norm(x[n]));
    }
    amplitude /= static_cast<double>(x.size());
    const double fitted = std::norm(amplitude) * static_cast<double>(x.size());
    return 10.0 * std::log10(std::max(energy - fitted, 1e-30) / energy);
}

/// The stream's samples `[first, first + count)` of a unit tone at `cycles` cycles per sample.
[[nodiscard]] std::vector<CF> toneFrom(std::uint64_t first, std::size_t count, double cycles) {
    std::vector<CF> out(count);
    for (std::size_t n = 0UZ; n < count; ++n) {
        const double phase = 2.0 * std::numbers::pi * std::fmod(cycles * static_cast<double>(first + n), 1.0);
        out[n]             = CF{static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))};
    }
    return out;
}

[[nodiscard]] std::vector<CF> run(gr::algorithm::PhaseVocoder& engine, std::span<const CF> input, std::size_t chunk = 0UZ) {
    std::vector<CF>   out;
    const std::size_t stride = chunk == 0UZ ? input.size() : chunk;
    for (std::size_t base = 0UZ; base < input.size(); base += stride) {
        engine.process(input.subspan(base, std::min(stride, input.size() - base)), out);
    }
    return out;
}

} // namespace

const boost::ut::suite<"PhaseVocoder"> phaseVocoderTests = [] {
    using namespace boost::ut;
    using gr::algorithm::PhaseVocoder;

    constexpr std::size_t kFrame = 1024UZ;
    constexpr std::size_t kHopIn = 256UZ;

    "a pure tone lands at its frequency divided, level held"_test = [] {
        for (const std::size_t divisor : {2UZ, 8UZ, 32UZ}) {
            PhaseVocoder engine;
            expect(engine.configure(kFrame, kHopIn, kHopIn / divisor).has_value());
            const double cycles = 0.21; // well inside the band at every divisor
            const auto   out    = run(engine, tone(64UZ * kFrame, cycles));

            // The output stream is declared at 1/divisor of the input rate. The divided frequency in output samples is
            // therefore the input frequency unchanged. Transposition shows as the same normalized frequency over
            // 1/divisor as many samples.
            const std::span<const CF> tail{std::span<const CF>(out).subspan(out.size() / 2UZ)};
            const double              peak = strongestBin(tail);
            const auto                bins = static_cast<double>(kFrame);
            expect(lt(std::abs(peak - cycles), 1.0 / bins)) << std::format("divisor {}: peak at {} for {}", divisor, peak, cycles);
            expect(lt(std::abs(rmsDb(tail)), 0.5)) << std::format("divisor {}: level {} dB against the unit tone", divisor, rmsDb(tail));
        }
    };

    "a long run stays as clean as a short one"_test = [] {
        // A hop pair near the frame size advances a bin's synthesis phase by hundreds of radians a frame. An unwrapped
        // phase grows past what a float resolves within a few thousand frames.
        constexpr std::size_t kDivisor = 2UZ;
        constexpr double      kCycles  = 215.4 / 1024.0; // between two bins
        constexpr std::size_t kEarly   = 256UZ;          // frames
        constexpr std::size_t kLate    = 16384UZ;
        PhaseVocoder          engine;
        expect(engine.configure(kFrame, kHopIn, kHopIn / kDivisor).has_value());

        std::uint64_t fed           = 0ULL;
        const auto    residualAfter = [&](std::size_t frames) {
            const std::uint64_t end = static_cast<std::uint64_t>(frames) * kHopIn;
            std::vector<CF>     out;
            while (fed < end) {
                const auto n = static_cast<std::size_t>(std::min<std::uint64_t>(1ULL << 16, end - fed));
                out.clear();
                engine.process(toneFrom(fed, n, kCycles), out);
                fed += n;
            }
            return residualDb(out, kCycles); // the last chunk's output
        };
        (void)residualAfter(kEarly / 4UZ); // the first frames carry the start-up
        const double early = residualAfter(kEarly);
        const double late  = residualAfter(kLate);
        expect(lt(early, -55.0)) << std::format("{} dB after {} frames: the tone stands clear of its residual", early, kEarly);
        expect(lt(late, early + 3.0)) << std::format("{} dB after {} frames against {} dB after {}", late, kLate, early, kEarly);
    };

    "transposition, not translation: a harmonic pair stays harmonic"_test = [] {
        PhaseVocoder engine;
        expect(engine.configure(kFrame, kHopIn, kHopIn / 8UZ).has_value());
        const double f = 0.03;
        auto         x = tone(64UZ * kFrame, f, 0.7);
        const auto   h = tone(64UZ * kFrame, 3.0 * f, 0.7);
        for (std::size_t i = 0UZ; i < x.size(); ++i) {
            x[i] += h[i];
        }
        const auto out = run(engine, x);

        // Each tone keeps its own normalized frequency, so the pair's ratio survives exactly
        // where an offset would have destroyed it.
        gr::algorithm::FFT<CF> fft;
        std::vector<CF>        tail(out.end() - static_cast<std::ptrdiff_t>(kFrame), out.end());
        std::vector<CF>        spectrum(kFrame);
        fft.compute(tail, spectrum);
        const auto   bin   = [&](double cycles) { return static_cast<double>(std::abs(spectrum[static_cast<std::size_t>(std::lround(cycles * static_cast<double>(kFrame)))])); };
        const double floor = static_cast<double>(std::abs(spectrum[kFrame / 2UZ]));
        expect(gt(bin(f), 100.0 * floor)) << "the fundamental stands";
        expect(gt(bin(3.0 * f), 100.0 * floor)) << "and its third harmonic beside it";
    };

    "the steady-state output count is the input count divided"_test = [] {
        for (const std::size_t divisor : {4UZ, 16UZ}) {
            PhaseVocoder engine;
            expect(engine.configure(kFrame, kHopIn, kHopIn / divisor).has_value());
            const std::size_t n    = 200UZ * kHopIn;
            const auto        out  = run(engine, tone(n, 0.1));
            const auto        want = static_cast<double>(n) / static_cast<double>(divisor);
            expect(lt(std::abs(static_cast<double>(out.size()) - want), static_cast<double>(kFrame))) << std::format("divisor {}: {} of {}", divisor, out.size(), want);
        }
    };

    "the chunking is not observable"_test = [] {
        const auto   x = tone(16UZ * kFrame, 0.13);
        PhaseVocoder reference;
        expect(reference.configure(kFrame, kHopIn, kHopIn / 4UZ).has_value());
        const auto want = run(reference, x);
        for (const std::size_t chunk : {1UZ, 7UZ, 997UZ}) {
            PhaseVocoder engine;
            expect(engine.configure(kFrame, kHopIn, kHopIn / 4UZ).has_value());
            expect(that % (run(engine, x, chunk) == want)) << std::format("chunk {}", chunk);
        }
    };

    "reset returns the engine to the primed-from-nothing state"_test = [] {
        const auto   x = tone(8UZ * kFrame, 0.07);
        PhaseVocoder engine;
        expect(engine.configure(kFrame, kHopIn, kHopIn / 4UZ).has_value());
        const auto first = run(engine, x);
        engine.reset();
        expect(that % (run(engine, x) == first)) << "the same stream replays to the same output";
    };

    "equal hops replay the stream exactly"_test = [] {
        PhaseVocoder engine;
        expect(engine.configure(kFrame, kHopIn, kHopIn).has_value());
        const auto x   = tone(3UZ * kFrame + 17UZ, 0.05);
        const auto out = run(engine, x);
        expect(that % (out == x)) << "identity is a copy, with no latency";
    };

    "refusals fire by name"_test = [] {
        PhaseVocoder engine;
        const auto   power = engine.configure(1000UZ, 250UZ, 25UZ);
        expect(fatal(!power.has_value())) << "frame must be a power of two";
        expect(power.error().message.contains("1000")) << power.error().message;
        const auto noHop = engine.configure(1024UZ, 0UZ, 1UZ);
        expect(fatal(!noHop.has_value())) << "hops must be positive";
        expect(noHop.error().message.contains("hops")) << noHop.error().message;
        const auto gaps = engine.configure(1024UZ, 2048UZ, 256UZ);
        expect(fatal(!gaps.has_value())) << "a hop past the frame leaves gaps";
        expect(gaps.error().message.contains("2048")) << gaps.error().message;
        expect(eq(engine.frameSize, 0UZ)) << "a refusal leaves the engine unsized";
    };
};

int main() { /* not needed for UT */ }
