#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <format>
#include <functional>
#include <limits>
#include <numbers>
#include <print>
#include <span>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/algorithm/signal/Phasor.hpp>

namespace {

constexpr double kTwoPi = 2. * std::numbers::pi_v<double>;

struct WindowError {
    double first = 0.; // worst phase error over the first window generated
    double last  = 0.; // ... and over the last one, however long the stream ran
};

/// Worst deviation of fill()'s output from exp(j*k*increment), measured on a stride within the first and the
/// last window of an nSamples-long stream. The two figures together are the no-drift property: the error is
/// bounded by the re-seed rounding and does not grow with stream length.
template<std::floating_point F>
[[nodiscard]] WindowError fillError(double increment, std::size_t nSamples, std::size_t window, std::size_t stride) {
    gr::signal::Phasor<F> phasor;
    phasor.configure(increment, 0.);

    std::vector<std::complex<F>> buffer(window);
    WindowError                  error;
    std::size_t                  produced = 0UZ;
    while (produced < nSamples) {
        const std::size_t n = std::min(window, nSamples - produced);
        phasor.fill(std::span<std::complex<F>>(buffer.data(), n));

        double worst = 0.;
        for (std::size_t k = 0UZ; k < n; k += stride) {
            const std::complex<double> want = std::polar(1., std::remainder(static_cast<double>(produced + k) * increment, kTwoPi));
            const std::complex<double> have = std::complex<double>(buffer[k]);
            worst                           = std::max(worst, std::abs(std::arg(have / want)));
        }
        if (produced == 0UZ) {
            error.first = worst;
        }
        error.last = worst;
        produced += n;
    }
    return error;
}

/// The modulated path's contract restated independently: exp(j*phase_k), where phase_k is the prefix sum of
/// the increments reduced to (-pi, pi] after each step.
template<std::floating_point F, std::floating_point FIncrement>
[[nodiscard]] std::vector<std::complex<F>> modulatedOracle(std::span<const FIncrement> increments, double initialPhase) {
    std::vector<std::complex<F>> out(increments.size());
    double                       phase = std::remainder(initialPhase, kTwoPi);
    for (std::size_t k = 0UZ; k < increments.size(); ++k) {
        out[k] = std::complex<F>(static_cast<F>(std::cos(phase)), static_cast<F>(std::sin(phase)));
        phase += static_cast<double>(increments[k]);
        if (phase > std::numbers::pi_v<double> || phase < -std::numbers::pi_v<double>) {
            phase = std::remainder(phase, kTwoPi);
        }
    }
    return out;
}

/// Deterministic non-trivial increment sequence; no RNG dependency, and the values span both signs.
[[nodiscard]] std::vector<float> rampIncrements(std::size_t n) {
    std::vector<float> increments(n);
    for (std::size_t k = 0UZ; k < n; ++k) {
        increments[k] = static_cast<float>(0.31 * std::sin(0.017 * static_cast<double>(k)) + 0.05);
    }
    return increments;
}

/// Worst spur outside the main lobe, in dB relative to the tone.
///
/// The window and the transform are BOTH double: a 4-term Blackman-Harris floors at its own -92 dB sidelobe
/// and a float transform at about -132 dBc, so either would measure the instrument rather than the phasor.
/// The 7-term minimum-sidelobe window below is at -180 dB, past where the float tone's own quantization sits.
[[nodiscard]] double worstSpurDbc(std::span<const std::complex<float>> tone) {
    const std::size_t n = tone.size();
    // 7-term minimum-sidelobe Blackman-Harris
    constexpr std::array<double, 7UZ> a{0.27105140069342, 0.43329793923448, 0.21812299954311, 0.06592544638803, 0.01081174209837, 0.00077658482522, 0.00001388721735};
    std::vector<std::complex<double>> windowed(n);
    for (std::size_t i = 0UZ; i < n; ++i) {
        const double x = kTwoPi * static_cast<double>(i) / static_cast<double>(n);
        double       w = a[0];
        for (std::size_t term = 1UZ; term < a.size(); ++term) {
            w += (term % 2UZ == 1UZ ? -1. : 1.) * a[term] * std::cos(static_cast<double>(term) * x);
        }
        windowed[i] = std::complex<double>(tone[i]) * w;
    }

    gr::algorithm::FFT<std::complex<double>> fft;
    std::vector<std::complex<double>>        spectrum(n);
    fft.compute(windowed, spectrum);

    std::size_t peak = 0UZ;
    for (std::size_t k = 1UZ; k < n; ++k) {
        if (std::abs(spectrum[k]) > std::abs(spectrum[peak])) {
            peak = k;
        }
    }

    // the 7-term window's main lobe is +-7 bins wide; everything beyond it is spur
    constexpr std::size_t kMainLobe = 8UZ;
    double                worst     = 0.;
    for (std::size_t k = 0UZ; k < n; ++k) {
        const std::size_t distance = std::min((k + n - peak) % n, (peak + n - k) % n);
        if (distance <= kMainLobe) {
            continue;
        }
        worst = std::max(worst, std::abs(spectrum[k]));
    }
    return 20. * std::log10(worst / std::abs(spectrum[peak]));
}

} // namespace

const boost::ut::suite<"Phasor"> phasorTests = [] {
    using namespace boost::ut;
    using gr::signal::Phasor;

    constexpr auto kFloatingTypes = std::tuple<float, double>{};

    // 10^8 samples is 24414 re-seed intervals; the increment is irrational so no re-seed ever
    // lands on a repeated phase
    "fill stays on the exact rotation and does not drift"_test = []<std::floating_point F> {
        constexpr std::size_t nSamples  = 100'000'000UZ;
        constexpr double      increment = 0.2718281828459045;
        // the worst over every increment swept below is 9.5e-7 rad in float and 4.4e-9 in double. The double
        // figure is the oracle's floor rather than the kernel's, since remainder(k*inc, 2pi) at k = 1e8 carries
        // about 3e-9 rad of its own rounding, so the float column is the one that shows the no-drift property.
        constexpr double bound = std::is_same_v<F, float> ? 3e-6 : 1e-7;

        const auto error = fillError<F>(increment, nSamples, 65536UZ, 997UZ);
        expect(lt(error.first, bound)) << std::format("first-window phase error {:g}", error.first);
        expect(lt(error.last, bound)) << std::format("last-window phase error {:g} after {} samples", error.last, nSamples);
        std::println("Phasor<{}> fill phase error over {} samples: first window {:g} rad, last window {:g} rad", //
            std::is_same_v<F, float> ? "float" : "double", nSamples, error.first, error.last);
    } | kFloatingTypes;

    "fill matches the definition across increment magnitudes"_test = []<std::floating_point F> {
        constexpr double bound = std::is_same_v<F, float> ? 3e-6 : 1e-8;
        for (const double increment : {1e-6, 1e-3, 0.05, 1.0, 3.0, -2.7, 6.0}) {
            const auto error = fillError<F>(increment, 1'000'000UZ, 65536UZ, 313UZ);
            expect(lt(error.last, bound)) << std::format("increment {}: phase error {:g}", increment, error.last);
        }
    } | kFloatingTypes;

    // the modulated path against an independent statement of the same accumulation rule
    "fillModulated is the reduced prefix sum, bit for bit"_test = []<std::floating_point F> {
        const auto increments = rampIncrements(10'000UZ);
        const auto want       = modulatedOracle<F>(std::span<const float>(increments), 0.4);

        Phasor<F> phasor;
        phasor.configure(0., 0.4);
        std::vector<std::complex<F>> have(increments.size());
        phasor.fillModulated(std::span<const float>(increments), std::span<std::complex<F>>(have));

        expect(eq(have.size(), want.size()));
        bool identical = true;
        for (std::size_t k = 0UZ; k < want.size(); ++k) {
            identical = identical && have[k].real() == want[k].real() && have[k].imag() == want[k].imag();
        }
        expect(identical) << "fillModulated must reproduce the prefix-sum oracle exactly";
    } | kFloatingTypes;

    "mixModulated multiplies the same phases into the stream"_test = []<std::floating_point F> {
        const auto increments = rampIncrements(4096UZ);
        const auto phases     = modulatedOracle<F>(std::span<const float>(increments), -1.1);

        std::vector<std::complex<F>> in(increments.size());
        for (std::size_t k = 0UZ; k < in.size(); ++k) {
            in[k] = std::complex<F>(static_cast<F>(0.5 + 0.25 * std::cos(0.003 * static_cast<double>(k))), static_cast<F>(-0.75));
        }

        Phasor<F> phasor;
        phasor.configure(0., -1.1);
        std::vector<std::complex<F>> have(in.size());
        phasor.mixModulated(std::span<const float>(increments), std::span<const std::complex<F>>(in), std::span<std::complex<F>>(have));

        for (std::size_t k = 0UZ; k < have.size(); ++k) {
            const std::complex<double> want = std::complex<double>(in[k]) * std::complex<double>(phases[k]);
            expect(lt(std::abs(std::complex<double>(have[k]) - want), 1e-6)) << std::format("mixModulated mismatch at {}", k);
        }
    } | kFloatingTypes;

    // the re-seed grid is stream-absolute, so a call boundary anywhere leaves the stream within the kernel's accuracy
    "fill and mix hold their accuracy at any call boundary"_test = []<std::floating_point F> {
        constexpr std::size_t nSamples  = 20'000UZ;
        constexpr double      increment = 0.2718281828459045;

        std::vector<std::complex<F>> in(nSamples);
        for (std::size_t k = 0UZ; k < nSamples; ++k) {
            in[k] = std::complex<F>(static_cast<F>(std::cos(0.00037 * static_cast<double>(k))), static_cast<F>(std::sin(0.00037 * static_cast<double>(k))));
        }

        const auto generateInChunks = [&](std::size_t chunk, bool doMix) {
            Phasor<F> phasor;
            phasor.configure(increment, 0.25);
            std::vector<std::complex<F>> out(nSamples);
            for (std::size_t i = 0UZ; i < nSamples; i += chunk) {
                const std::size_t n = std::min(chunk, nSamples - i);
                if (doMix) {
                    phasor.mix(std::span<const std::complex<F>>(in.data() + i, n), std::span<std::complex<F>>(out.data() + i, n));
                } else {
                    phasor.fill(std::span<std::complex<F>>(out.data() + i, n));
                }
            }
            return out;
        };

        for (const bool doMix : {false, true}) {
            constexpr double tolerance = std::is_same_v<F, float> ? 4e-6 : 1e-9;
            for (const std::size_t chunk : {1UZ, 3UZ, 64UZ, 1000UZ, 4095UZ, 4096UZ, 4097UZ, nSamples}) {
                const auto split = generateInChunks(chunk, doMix);
                double     worst = 0.;
                for (std::size_t k = 0UZ; k < nSamples; ++k) {
                    const std::complex<double> x    = doMix ? std::complex<double>(in[k]) : std::complex<double>(1., 0.);
                    const std::complex<double> want = x * std::polar(1., 0.25 + increment * static_cast<double>(k));
                    worst                           = std::max(worst, std::abs(std::complex<double>(split[k]) - want));
                }
                expect(lt(worst, tolerance)) << std::format("{} in {}-sample calls is {:.3e} from the double reference", doMix ? "mix" : "fill", chunk, worst);
            }
        }
    } | kFloatingTypes;

    "fillModulated agrees with itself at any call boundary"_test = []<std::floating_point F> {
        const auto increments = rampIncrements(9'999UZ);

        const auto generateInChunks = [&](std::size_t chunk) {
            Phasor<F> phasor;
            phasor.configure(0., 0.4);
            std::vector<std::complex<F>> out(increments.size());
            for (std::size_t i = 0UZ; i < increments.size(); i += chunk) {
                const std::size_t n = std::min(chunk, increments.size() - i);
                phasor.fillModulated(std::span<const float>(increments.data() + i, n), std::span<std::complex<F>>(out.data() + i, n));
            }
            return out;
        };

        const auto reference = generateInChunks(increments.size());
        for (const std::size_t chunk : {1UZ, 7UZ, 512UZ, 4096UZ}) {
            const auto split = generateInChunks(chunk);
            double     worst = 0.;
            for (std::size_t k = 0UZ; k < reference.size(); ++k) {
                worst = std::max(worst, std::abs(std::complex<double>(split[k]) - std::complex<double>(reference[k])));
            }
            expect(lt(worst, 4. * static_cast<double>(std::numeric_limits<F>::epsilon()))) << std::format("fillModulated in {}-sample calls is {:.3e} from one call", chunk, worst);
        }
    } | kFloatingTypes;

    "a chirp agrees with the modulated path it replaces, and is chunk independent"_test = []<std::floating_point F> {
        // A linear frequency sweep produces an arithmetic increment schedule, which is exactly what
        // fillModulated would be handed sample by sample. The chirp entry has to reproduce it without
        // evaluating a transcendental per sample, so the two are compared over the same schedule.
        constexpr std::size_t kSamples = 4096UZ;
        constexpr double      kStart   = 0.31;
        constexpr double      kStep    = 1.7e-6;

        std::vector<float> increments(kSamples);
        for (std::size_t k = 0UZ; k < kSamples; ++k) {
            increments[k] = static_cast<float>(kStart + static_cast<double>(k) * kStep);
        }

        Phasor<F> modulated;
        modulated.configure(0., 0.);
        std::vector<std::complex<F>> viaModulate(kSamples);
        modulated.fillModulated(std::span<const float>(increments), std::span<std::complex<F>>(viaModulate));

        const auto chirpInChunks = [&](std::size_t chunk) {
            Phasor<F> phasor;
            phasor.configure(kStart, 0.);
            std::vector<std::complex<F>> out(kSamples);
            for (std::size_t i = 0UZ; i < kSamples; i += chunk) {
                const std::size_t n = std::min(chunk, kSamples - i);
                phasor.fillChirp(kStep, std::span<std::complex<F>>(out.data() + i, n));
            }
            return out;
        };

        const auto reference = chirpInChunks(kSamples);

        // the two paths round differently - one accumulates a phase, the other a phasor - so they agree to a
        // tolerance rather than bit for bit, and the tolerance is what a float phasor can hold over the run
        double worst = 0.;
        for (std::size_t k = 0UZ; k < kSamples; ++k) {
            worst = std::max(worst, std::abs(std::complex<double>(reference[k]) - std::complex<double>(viaModulate[k])));
        }
        expect(lt(worst, 1e-4)) << std::format("the chirp and the modulated schedule differ by at most {:.3e}", worst);

        // chunk independence: a split chirp agrees with one call to a few units of F's epsilon
        for (const std::size_t chunk : {1UZ, 7UZ, 256UZ, 511UZ, 4096UZ}) {
            const auto split = chirpInChunks(chunk);
            double     most  = 0.;
            for (std::size_t k = 0UZ; k < kSamples; ++k) {
                most = std::max(most, std::abs(std::complex<double>(split[k]) - std::complex<double>(reference[k])));
            }
            expect(lt(most, 4. * static_cast<double>(std::numeric_limits<F>::epsilon()))) << std::format("fillChirp in {}-sample calls is {:.3e} from one call", chunk, most);
        }
    } | kFloatingTypes;

    "a running chirp reports its own phase and increment, and hands them on"_test = []<std::floating_point F> {
        using Cx                   = std::complex<F>;
        constexpr double kStart    = 0.1;
        constexpr double kPhase0   = 0.4;
        constexpr double kStep     = 0.001;
        constexpr double kStateTol = 1e-9;
        const double     sampleTol = std::is_same_v<F, float> ? 4e-6 : 1e-9;

        // the arithmetic schedule: sample n has increment start + n*step and phase phase0 + n*start + step*n*(n-1)/2
        const auto schedulePhase     = [&](double start, double phase0, double step, std::size_t n) { return std::remainder(phase0 + static_cast<double>(n) * start + step * static_cast<double>(n) * (static_cast<double>(n) - 1.) * 0.5, kTwoPi); };
        const auto scheduleIncrement = [&](double start, double step, std::size_t n) { return start + static_cast<double>(n) * step; };
        const auto phaseGap          = [](double a, double b) { return std::abs(std::remainder(a - b, kTwoPi)); };

        const std::size_t kInterval = Phasor<F>::kReseedInterval;
        for (const std::size_t n : {1UZ, 100UZ, kInterval - 1UZ, kInterval, kInterval + 1UZ, 3UZ * kInterval + 5UZ, 5000UZ}) {
            for (const std::size_t chunk : {n, 1UZ, 37UZ, kInterval}) {
                Phasor<F> phasor;
                phasor.configure(kStart, kPhase0);
                std::vector<Cx> out(n);
                for (std::size_t i = 0UZ; i < n; i += chunk) {
                    phasor.fillChirp(kStep, std::span<Cx>(out.data() + i, std::min(chunk, n - i)));
                }
                expect(lt(phaseGap(phasor.phase(), schedulePhase(kStart, kPhase0, kStep, n)), kStateTol)) << std::format("phase after {} samples in {}-sample calls: {}", n, chunk, phasor.phase());
                expect(lt(std::abs(phasor.increment() - scheduleIncrement(kStart, kStep, n)), kStateTol)) << std::format("increment after {} samples in {}-sample calls: {}", n, chunk, phasor.increment());

                // a fixed fill after the chirp starts on the schedule's next phase and runs at its next increment
                std::vector<Cx> next(2UZ);
                phasor.fill(std::span<Cx>(next));
                const double               p0    = schedulePhase(kStart, kPhase0, kStep, n);
                const double               inc   = scheduleIncrement(kStart, kStep, n);
                const std::complex<double> want0 = std::polar(1., p0);
                const std::complex<double> want1 = std::polar(1., p0 + inc);
                expect(lt(std::abs(std::complex<double>(next[0]) - want0), sampleTol)) << std::format("first fixed sample after {} chirp samples in {}-sample calls", n, chunk);
                expect(lt(std::abs(std::complex<double>(next[1]) - want1), sampleTol)) << std::format("second fixed sample after {} chirp samples in {}-sample calls", n, chunk);
            }
        }

        // advance during a chirp moves from the chirp's present phase by its present increment
        {
            constexpr std::size_t n = 300UZ;
            Phasor<F>             phasor;
            phasor.configure(kStart, kPhase0);
            std::vector<Cx> out(n);
            phasor.fillChirp(kStep, std::span<Cx>(out));
            phasor.advance(10);
            const double want = schedulePhase(kStart, kPhase0, kStep, n) + 10. * scheduleIncrement(kStart, kStep, n);
            expect(lt(phaseGap(phasor.phase(), want), kStateTol)) << "advance during a chirp";
            std::vector<Cx> next(1UZ);
            phasor.fill(std::span<Cx>(next));
            expect(lt(std::abs(std::complex<double>(next[0]) - std::polar(1., want)), sampleTol)) << "the sample after advance continues the chirp's phase";
        }

        // setIncrement during a chirp keeps the phase and takes the new increment
        {
            constexpr std::size_t n = 300UZ;
            Phasor<F>             phasor;
            phasor.configure(kStart, kPhase0);
            std::vector<Cx> out(n);
            phasor.fillChirp(kStep, std::span<Cx>(out));
            phasor.setIncrement(0.2);
            const double want = schedulePhase(kStart, kPhase0, kStep, n);
            expect(lt(phaseGap(phasor.phase(), want), kStateTol)) << "setIncrement during a chirp must keep the phase";
            expect(lt(std::abs(phasor.increment() - 0.2), kStateTol));
            std::vector<Cx> next(2UZ);
            phasor.fill(std::span<Cx>(next));
            expect(lt(std::abs(std::complex<double>(next[0]) - std::polar(1., want)), sampleTol)) << "the sample after setIncrement continues the chirp's phase";
            expect(lt(std::abs(std::complex<double>(next[1]) - std::polar(1., want + 0.2)), sampleTol));
        }

        // a chirp call with another step starts a new frame at the exact phase and increment reached
        for (const std::size_t n1 : {100UZ, kInterval, 700UZ}) {
            constexpr double      kStep2 = -0.0005;
            constexpr std::size_t n2     = 400UZ;
            Phasor<F>             phasor;
            phasor.configure(kStart, kPhase0);
            std::vector<Cx> first(n1);
            std::vector<Cx> second(n2);
            phasor.fillChirp(kStep, std::span<Cx>(first));
            phasor.fillChirp(kStep2, std::span<Cx>(second));
            const double p1 = schedulePhase(kStart, kPhase0, kStep, n1);
            const double i1 = scheduleIncrement(kStart, kStep, n1);
            expect(lt(std::abs(std::complex<double>(second[0]) - std::polar(1., p1)), sampleTol)) << std::format("first sample at the step change after {} samples", n1);
            for (const std::size_t k : {1UZ, 200UZ, n2 - 1UZ}) {
                expect(lt(std::abs(std::complex<double>(second[k]) - std::polar(1., schedulePhase(i1, p1, kStep2, k))), sampleTol)) << std::format("sample {} after a step change at {}", k, n1);
            }
            expect(lt(phaseGap(phasor.phase(), schedulePhase(i1, p1, kStep2, n2)), kStateTol)) << "phase after a step change";
            expect(lt(std::abs(phasor.increment() - scheduleIncrement(i1, kStep2, n2)), kStateTol)) << "increment after a step change";
        }

        // the chirp after a step change, run in calls of 1 and of 37 samples, stays on the schedule
        for (const std::size_t chunk : {1UZ, 37UZ}) {
            constexpr double      kStep2 = -0.0005;
            constexpr std::size_t n1     = 100UZ;
            constexpr std::size_t n2     = 700UZ;
            Phasor<F>             phasor;
            phasor.configure(kStart, kPhase0);
            std::vector<Cx> first(n1);
            std::vector<Cx> second(n2);
            phasor.fillChirp(kStep, std::span<Cx>(first));
            for (std::size_t i = 0UZ; i < n2; i += chunk) {
                phasor.fillChirp(kStep2, std::span<Cx>(second.data() + i, std::min(chunk, n2 - i)));
            }
            const double p1    = schedulePhase(kStart, kPhase0, kStep, n1);
            const double i1    = scheduleIncrement(kStart, kStep, n1);
            double       worst = 0.;
            for (std::size_t k = 0UZ; k < n2; ++k) {
                worst = std::max(worst, std::abs(std::complex<double>(second[k]) - std::polar(1., schedulePhase(i1, p1, kStep2, k))));
            }
            expect(lt(worst, sampleTol)) << std::format("a chirp after a step change in {}-sample calls is {:.3e} from the schedule", chunk, worst);
            expect(lt(phaseGap(phasor.phase(), schedulePhase(i1, p1, kStep2, n2)), kStateTol)) << std::format("phase after a step change in {}-sample calls", chunk);
            expect(lt(std::abs(phasor.increment() - scheduleIncrement(i1, kStep2, n2)), kStateTol)) << std::format("increment after a step change in {}-sample calls", chunk);
        }

        // setPhase during a chirp keeps the increment the chirp has reached
        {
            constexpr std::size_t n = 300UZ;
            Phasor<F>             phasor;
            phasor.configure(kStart, kPhase0);
            std::vector<Cx> out(n);
            phasor.fillChirp(kStep, std::span<Cx>(out));
            phasor.setPhase(1.);
            const double inc = scheduleIncrement(kStart, kStep, n);
            expect(lt(std::abs(phasor.increment() - inc), kStateTol)) << "setPhase during a chirp must keep the reached increment";
            std::vector<Cx> next(2UZ);
            phasor.fill(std::span<Cx>(next));
            expect(lt(std::abs(std::complex<double>(next[0]) - std::polar(1., 1.)), sampleTol)) << "the sample after setPhase is at the new phase";
            expect(lt(std::abs(std::complex<double>(next[1]) - std::polar(1., 1. + inc)), sampleTol)) << "the second sample after setPhase advances by the reached increment";
        }

        // a chirp, a fixed fill, then the chirp again: the second chirp starts at the fixed run's phase and increment
        {
            constexpr std::size_t n1 = 100UZ;
            constexpr std::size_t nf = 50UZ;
            constexpr std::size_t n2 = 100UZ;
            Phasor<F>             phasor;
            phasor.configure(kStart, kPhase0);
            std::vector<Cx> first(n1);
            std::vector<Cx> fixed(nf);
            std::vector<Cx> second(n2);
            phasor.fillChirp(kStep, std::span<Cx>(first));
            phasor.fill(std::span<Cx>(fixed));
            phasor.fillChirp(kStep, std::span<Cx>(second));
            const double i1    = scheduleIncrement(kStart, kStep, n1);
            const double p2    = schedulePhase(kStart, kPhase0, kStep, n1) + static_cast<double>(nf) * i1;
            double       worst = 0.;
            for (std::size_t k = 0UZ; k < n2; ++k) {
                worst = std::max(worst, std::abs(std::complex<double>(second[k]) - std::polar(1., schedulePhase(i1, p2, kStep, k))));
            }
            expect(lt(worst, sampleTol)) << std::format("the chirp after a fixed fill is {:.3e} from the schedule", worst);
            expect(lt(phaseGap(phasor.phase(), schedulePhase(i1, p2, kStep, n2)), kStateTol)) << "phase after chirp, fill, chirp";
            expect(lt(std::abs(phasor.increment() - scheduleIncrement(i1, kStep, n2)), kStateTol)) << "increment after chirp, fill, chirp";
        }
    } | kFloatingTypes;

    "a call that produces no sample leaves a chirp running"_test = []<std::floating_point F> {
        constexpr std::size_t kBefore = 37UZ;
        constexpr std::size_t kAfter  = 300UZ;
        constexpr double      kStart  = 0.2;
        constexpr double      kStep   = 0.001;

        using Cx             = std::complex<F>;
        const auto makeChirp = [&] {
            Phasor<F> phasor;
            phasor.configure(kStart, 0.3);
            return phasor;
        };

        std::vector<Cx> reference(kBefore + kAfter);
        {
            Phasor<F> phasor = makeChirp();
            phasor.fillChirp(kStep, std::span<Cx>(reference));
        }

        const std::vector<float> someIncrements(4UZ, 0.1f);
        const std::vector<Cx>    someSamples(4UZ, Cx(F(1), F(0)));
        std::vector<Cx>          someOut(4UZ);

        const std::array<std::pair<const char*, std::function<void(Phasor<F>&)>>, 8UZ> emptyCalls{{
            {"fill", [&](Phasor<F>& p) { p.fill({}); }},
            {"mix, no input", [&](Phasor<F>& p) { p.mix({}, std::span<Cx>(someOut)); }},
            {"mix, no output", [&](Phasor<F>& p) { p.mix(std::span<const Cx>(someSamples), {}); }},
            {"fillModulated, no increments", [&](Phasor<F>& p) { p.fillModulated(std::span<const float>{}, std::span<Cx>(someOut)); }},
            {"fillModulated, no output", [&](Phasor<F>& p) { p.fillModulated(std::span<const float>(someIncrements), {}); }},
            {"mixModulated, no increments", [&](Phasor<F>& p) { p.mixModulated(std::span<const float>{}, std::span<const Cx>(someSamples), std::span<Cx>(someOut)); }},
            {"mixModulated, no input", [&](Phasor<F>& p) { p.mixModulated(std::span<const float>(someIncrements), {}, std::span<Cx>(someOut)); }},
            {"mixModulated, no output", [&](Phasor<F>& p) { p.mixModulated(std::span<const float>(someIncrements), std::span<const Cx>(someSamples), {}); }},
        }};

        for (const auto& [name, call] : emptyCalls) {
            Phasor<F>       phasor = makeChirp();
            std::vector<Cx> out(kBefore + kAfter);
            phasor.fillChirp(kStep, std::span<Cx>(out.data(), kBefore));
            call(phasor);
            phasor.fillChirp(kStep, std::span<Cx>(out.data() + kBefore, kAfter));

            double worst = 0.;
            for (std::size_t k = 0UZ; k < out.size(); ++k) {
                worst = std::max(worst, std::abs(std::complex<double>(out[k]) - std::complex<double>(reference[k])));
            }
            expect(lt(worst, 4. * static_cast<double>(std::numeric_limits<F>::epsilon()))) << std::format("a chirp resumed after an empty {} is {:.3e} from the uninterrupted chirp", name, worst);
        }
    } | kFloatingTypes;

    "mix equals fill multiplied into the stream"_test = []<std::floating_point F> {
        constexpr std::size_t nSamples  = 8192UZ;
        constexpr double      increment = 0.13;

        std::vector<std::complex<F>> in(nSamples, std::complex<F>(F(0.75), F(-0.25)));

        Phasor<F> generator;
        generator.configure(increment, 1.0);
        std::vector<std::complex<F>> phases(nSamples);
        generator.fill(std::span<std::complex<F>>(phases));

        Phasor<F> mixer;
        mixer.configure(increment, 1.0);
        std::vector<std::complex<F>> mixed(nSamples);
        mixer.mix(std::span<const std::complex<F>>(in), std::span<std::complex<F>>(mixed));

        expect(approx(mixer.phase(), generator.phase(), 1e-12)) << "both paths must leave the same accumulated phase";
        for (std::size_t k = 0UZ; k < nSamples; ++k) {
            const std::complex<double> want = std::complex<double>(in[k]) * std::complex<double>(phases[k]);
            expect(lt(std::abs(std::complex<double>(mixed[k]) - want), 1e-6)) << std::format("mix mismatch at {}", k);
        }
    } | kFloatingTypes;

    "advance moves the phase by whole increments and is reversible"_test = [] {
        Phasor<double> phasor;
        phasor.configure(0.37, 0.1);

        phasor.advance(1);
        expect(approx(phasor.phase(), 0.47, 1e-12));
        phasor.advance(-1);
        expect(approx(phasor.phase(), 0.1, 1e-12)) << "stepping one increment forward and back must return the phase";

        phasor.advance(1000);
        expect(approx(phasor.phase(), std::remainder(0.1 + 1000. * 0.37, kTwoPi), 1e-12));
    };

    "a generated span leaves the phase where the next sample continues"_test = [] {
        constexpr double increment = 0.37;
        Phasor<double>   phasor;
        phasor.configure(increment, 0.);

        std::vector<std::complex<double>> first(100UZ);
        phasor.fill(std::span<std::complex<double>>(first));
        expect(approx(phasor.phase(), std::remainder(100. * increment, kTwoPi), 1e-12));

        std::vector<std::complex<double>> second(1UZ);
        phasor.fill(std::span<std::complex<double>>(second));
        const std::complex<double> want = std::polar(1., std::remainder(100. * increment, kTwoPi));
        expect(lt(std::abs(second[0] - want), 1e-12)) << "the sample after a span continues the same rotation";
    };

    "configure refuses non-finite values"_test = [] {
        Phasor<float> phasor;
        expect(throws([&] { phasor.configure(std::numeric_limits<double>::quiet_NaN(), 0.); }));
        expect(throws([&] { phasor.configure(0.1, std::numeric_limits<double>::infinity()); }));
        expect(nothrow([&] { phasor.configure(0.1, 0.2); }));
    };

    "spectral honesty: the worst spur stays below the stated floor"_test = [] {
        constexpr std::size_t nSamples = 65536UZ;
        // deliberately off-bin: 1000.37 cycles over the transform, so no bin holds the tone exactly
        const double increment = kTwoPi * 1000.37 / static_cast<double>(nSamples);

        gr::signal::Phasor<float> phasor;
        phasor.configure(increment, 0.);
        std::vector<std::complex<float>> tone(nSamples);
        phasor.fill(std::span<std::complex<float>>(tone));

        const double spurDbc = worstSpurDbc(tone);
        std::println("Phasor<float> worst spur at an off-bin tone: {:.1f} dBc", spurDbc);
        // the recurrence holds the worst spur near -147 dBc; the bound leaves margin for libm and transform
        // differences across platforms
        expect(lt(spurDbc, -130.)) << std::format("worst spur {:.1f} dBc", spurDbc);
    };
};

int main() { /* not needed for UT */ }
