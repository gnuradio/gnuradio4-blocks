#include <boost/ut.hpp>

#include <gnuradio-4.0/sdr/DcBlocker.hpp>

#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <format>
#include <numbers>
#include <print>

// The blocker is measured at the rate and the cutoff a radio uses, 10 Hz on 2 MS/s, and at a hundred times that
// cutoff. A tone of amplitude 0.3 at 100 kHz rides on a DC term of 0.5; what the blocker must do is remove the
// DC term and leave the tone where its own response puts it. No device and no SoapySDR are involved.
namespace {

using gr::blocks::sdr::DcBlocker;

constexpr double      kSampleRate  = 2.0e6;
constexpr double      kDcTerm      = 0.5;
constexpr double      kToneHz      = 1.0e5; // fs/20, so the tone is a 20-entry table and its own reference
constexpr double      kToneAmp     = 0.3;
constexpr std::size_t kTonePeriod  = 20UZ;
constexpr std::size_t kSamples     = 4'000'000UZ; // 2 s
constexpr std::size_t kSteadyStart = 1'000'000UZ; // 0.5 s, about 22 time constants of a 10 Hz corner
constexpr std::size_t kStepSamples = 1'000'000UZ; // 0.5 s of a constant input, for the settling time

std::array<std::complex<double>, kTonePeriod> toneTable() {
    std::array<std::complex<double>, kTonePeriod> table{};
    for (std::size_t k = 0UZ; k < kTonePeriod; ++k) {
        const double phase = 2.0 * std::numbers::pi * static_cast<double>(k) / static_cast<double>(kTonePeriod);
        table[k]           = kToneAmp * std::complex<double>(std::cos(phase), std::sin(phase));
    }
    return table;
}

double poleRadius(double cutoffHz) { return 1.0 - 2.0 * std::numbers::pi * cutoffHz / kSampleRate; }

// the exact response of y[n] = x[n] - x[n-1] + r*y[n-1] at one frequency
std::complex<double> response(double cutoffHz, double frequencyHz) {
    const std::complex<double> zInv = std::polar(1.0, -2.0 * std::numbers::pi * frequencyHz / kSampleRate);
    return (1.0 - zInv) / (1.0 - poleRadius(cutoffHz) * zInv);
}

struct ToneResult {
    double gainDb       = 0.0; // of the 100 kHz tone, measured
    double idealGainDb  = 0.0; // of the recurrence itself, computed
    double phaseDeg     = 0.0;
    double dcResidual   = 0.0; // |mean output| in the steady state
    double errorIdealDb = 0.0; // against the exactly filtered tone: what the implementation adds
    double errorToneDb  = 0.0; // against the tone as it went in: what the blocker costs the signal
};

// one blocker per part of the sample, which is how the radio sources run it
ToneResult measureTone(double cutoffHz) {
    const auto                 table = toneTable();
    const std::complex<double> h     = response(cutoffHz, kToneHz);

    DcBlocker blockerI(static_cast<float>(cutoffHz), static_cast<float>(kSampleRate));
    DcBlocker blockerQ(static_cast<float>(cutoffHz), static_cast<float>(kSampleRate));

    std::complex<double> sumOut{0.0, 0.0};
    std::complex<double> correlation{0.0, 0.0};
    double               toneNorm = 0.0;
    double               errIdeal = 0.0;
    double               errTone  = 0.0;

    for (std::size_t i = 0UZ; i < kSamples; ++i) {
        const std::complex<double> tone = table[i % kTonePeriod];
        const float                yI   = blockerI.processOne(static_cast<float>(kDcTerm + tone.real()));
        const float                yQ   = blockerQ.processOne(static_cast<float>(tone.imag()));
        if (i < kSteadyStart) {
            continue;
        }
        const std::complex<double> y{static_cast<double>(yI), static_cast<double>(yQ)};
        sumOut += y;
        correlation += y * std::conj(tone);
        toneNorm += std::norm(tone);
        errIdeal += std::norm(y - h * tone);
        errTone += std::norm(y - tone);
    }

    const double               steady = static_cast<double>(kSamples - kSteadyStart);
    const std::complex<double> gain   = correlation / toneNorm;

    ToneResult result;
    result.gainDb       = 20.0 * std::log10(std::abs(gain));
    result.idealGainDb  = 20.0 * std::log10(std::abs(h));
    result.phaseDeg     = std::arg(gain) * 180.0 / std::numbers::pi;
    result.dcResidual   = std::abs(sumOut / steady);
    result.errorIdealDb = 10.0 * std::log10(errIdeal / (std::norm(h) * toneNorm));
    result.errorToneDb  = 10.0 * std::log10(errTone / toneNorm);
    return result;
}

struct StepResult {
    double settleMs = 0.0;
    bool   settled  = false;
    double floorRms = 0.0; // over the last tenth of the run: a state that does not decay shows up here
};

// the transient a constant input leaves: the time after which the output stays under 1 % of the DC term
StepResult measureStep(double cutoffHz) {
    DcBlocker         blocker(static_cast<float>(cutoffHz), static_cast<float>(kSampleRate));
    const std::size_t tail    = kStepSamples / 10UZ;
    double            lastOut = -1.0;
    double            tailSq  = 0.0;

    for (std::size_t i = 0UZ; i < kStepSamples; ++i) {
        const double y = std::abs(static_cast<double>(blocker.processOne(static_cast<float>(kDcTerm))));
        if (!(y <= 0.01 * kDcTerm)) {
            lastOut = static_cast<double>(i);
        }
        if (i >= kStepSamples - tail) {
            tailSq += y * y;
        }
    }

    StepResult result;
    result.settled  = lastOut < static_cast<double>(kStepSamples - 1UZ);
    result.settleMs = (lastOut + 1.0) / kSampleRate * 1e3;
    result.floorRms = std::sqrt(tailSq / static_cast<double>(tail));
    return result;
}

} // namespace

const boost::ut::suite<"DC blocker"> dcBlockerTests = [] {
    using namespace boost::ut;

    "a 10 Hz cutoff on 2 MS/s leaves a 100 kHz tone alone and removes the DC term"_test = [] {
        const ToneResult tone = measureTone(10.0);
        const StepResult step = measureStep(10.0);
        std::println("fc = 10 Hz: gain {:+.6f} dB (ideal {:+.6f} dB), phase {:+.4f} deg, DC residual {:.3e}, error vs ideal {:.2f} dB, error vs tone {:.2f} dB, settles in {:.2f} ms, floor {:.3e}", tone.gainDb, tone.idealGainDb, tone.phaseDeg, tone.dcResidual, tone.errorIdealDb, tone.errorToneDb, step.settleMs, step.floorRms);

        expect(lt(std::abs(tone.gainDb), 0.01)) << std::format("the tone's gain is {:+.6f} dB", tone.gainDb);
        expect(lt(std::abs(tone.gainDb - tone.idealGainDb), 0.01)) << std::format("gain {:+.6f} dB against the recurrence's own {:+.6f} dB", tone.gainDb, tone.idealGainDb);
        expect(lt(tone.dcResidual, 1e-6)) << std::format("DC residual {:.3e} after 0.5 s", tone.dcResidual);
        expect(lt(tone.errorIdealDb, -60.0)) << std::format("error against the exactly filtered tone {:.2f} dB", tone.errorIdealDb);
        expect(lt(tone.errorToneDb, -60.0)) << std::format("error against the tone as it went in {:.2f} dB", tone.errorToneDb);
        expect(step.settled) << "the output never stays under 1 % of the DC term";
        expect(lt(step.settleMs, 100.0)) << std::format("settles to 1 % in {:.2f} ms", step.settleMs);
    };

    "a 1000 Hz cutoff keeps the same accuracy, at the response its own corner gives"_test = [] {
        const ToneResult tone = measureTone(1000.0);
        const StepResult step = measureStep(1000.0);
        std::println("fc = 1000 Hz: gain {:+.6f} dB (ideal {:+.6f} dB), phase {:+.4f} deg, DC residual {:.3e}, error vs ideal {:.2f} dB, error vs tone {:.2f} dB, settles in {:.2f} ms, floor {:.3e}", tone.gainDb, tone.idealGainDb, tone.phaseDeg, tone.dcResidual, tone.errorIdealDb, tone.errorToneDb, step.settleMs, step.floorRms);

        expect(lt(std::abs(tone.gainDb - tone.idealGainDb), 0.01)) << std::format("gain {:+.6f} dB against the recurrence's own {:+.6f} dB", tone.gainDb, tone.idealGainDb);
        expect(lt(tone.dcResidual, 1e-6)) << std::format("DC residual {:.3e} after 0.5 s", tone.dcResidual);
        expect(lt(tone.errorIdealDb, -60.0)) << std::format("error against the exactly filtered tone {:.2f} dB", tone.errorIdealDb);
        expect(step.settled) << "the output never stays under 1 % of the DC term";
        expect(lt(step.settleMs, 100.0)) << std::format("settles to 1 % in {:.2f} ms", step.settleMs);
    };

    "a cutoff the first-order form cannot realize is refused and the samples pass through"_test = [] {
        DcBlocker blocker;
        expect(!blocker.setCutoff(static_cast<float>(kSampleRate) / 4.f, static_cast<float>(kSampleRate))) << "a cutoff at fs/4 was taken";
        expect(!blocker.setCutoff(10.f, 0.f)) << "a sample rate of zero was taken";
        expect(!blocker.setCutoff(-10.f, static_cast<float>(kSampleRate))) << "a negative cutoff was taken";
        for (const float sample : {0.5f, -0.25f, 0.125f, 0.f}) {
            expect(eq(blocker.processOne(sample), sample)) << std::format("a refused blocker changed {}", sample);
        }
        expect(blocker.setCutoff(10.f, static_cast<float>(kSampleRate))) << "10 Hz on 2 MS/s was refused";
    };

    "reset forgets the past samples and keeps the cutoff"_test = [] {
        DcBlocker blocker(10.f, static_cast<float>(kSampleRate));
        for (std::size_t i = 0UZ; i < 1000UZ; ++i) {
            [[maybe_unused]] const float ignored = blocker.processOne(static_cast<float>(kDcTerm));
        }
        blocker.reset();
        // with no history, the first output of y[n] = x[n] - x[n-1] + r*y[n-1] is the input itself
        expect(eq(blocker.processOne(static_cast<float>(kDcTerm)), static_cast<float>(kDcTerm))) << "the state survived a reset";
    };
};

int main() { /* not needed for UT */ }
