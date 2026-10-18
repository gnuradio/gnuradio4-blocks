#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <limits>
#include <numbers>
#include <optional>
#include <print>
#include <span>
#include <vector>

#include <gnuradio-4.0/algorithm/measurement/PhaseUnwrap.hpp>

using namespace boost::ut;
using gr::measurement::CycleUnwrapper;
using gr::measurement::UnwrapOrigin;
using gr::measurement::unwrapOriginFrom;
using gr::measurement::unwrapOriginName;

namespace {

constexpr double kTwoPi = 2. * std::numbers::pi_v<double>;
using Complex           = std::complex<float>;

[[nodiscard]] bool longRun() { return std::getenv("ENABLE_LONG_TESTS") != nullptr; }

/// A unit tone at @p f0 cycles per sample. The phase of each sample is computed from its index in `double`, and the
/// input carries no accumulated error.
[[nodiscard]] std::vector<Complex> tone(double f0, std::size_t n, std::uint64_t first = 0ULL) {
    std::vector<Complex> out(n);
    for (std::size_t k = 0UZ; k < n; ++k) {
        const double phase = kTwoPi * f0 * static_cast<double>(first + k);
        out[k]             = Complex{static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))};
    }
    return out;
}

/// A `float` and a `double` sum of the same per-sample phase steps.
struct FloatComparison {
    float  asFloat{0.f};
    double asDouble{0.};
};

void accumulate(FloatComparison& into, float step) noexcept {
    into.asFloat += step;
    into.asDouble += static_cast<double>(step);
}

} // namespace

const boost::ut::suite<"PhaseUnwrap"> _phase_unwrap = [] {
    "the origin names parse, and the max step fraction is checked"_test = [] {
        expect(unwrapOriginFrom("first_sample").value() == UnwrapOrigin::first_sample);
        expect(unwrapOriginFrom("zero").value() == UnwrapOrigin::zero);
        expect(!unwrapOriginFrom("origin").has_value());
        expect(unwrapOriginName(UnwrapOrigin::zero) == "zero");
        expect(throws<gr::exception>([] { (void)CycleUnwrapper(UnwrapOrigin::zero, 0.); })) << "a max step fraction of zero is not in (0, 1]";
        expect(throws<gr::exception>([] { (void)CycleUnwrapper(UnwrapOrigin::zero, 1.5); }));
        expect(nothrow([] { (void)CycleUnwrapper(UnwrapOrigin::zero, 1.); })) << "a max step fraction of one is accepted";
    };

    "the count is exact where a float sum of the steps is off by more than a cycle"_test = [] {
        // At 0.4 cycles per sample the stream turns twice every five samples. After `5*C/2 + 1` samples it ends
        // exactly on turn `C`. No sample falls at +/-pi.
        // The long run takes 2.5e9 samples, past a 32-bit size_t, and counts them in std::uint64_t.
        const std::int64_t    turns   = longRun() ? 1'000'000'000LL : (1LL << 21);
        const std::uint64_t   samples = 5ULL * static_cast<std::uint64_t>(turns) / 2ULL + 1ULL;
        constexpr std::size_t kChunk  = 1UZ << 16;

        CycleUnwrapper  unwrapper;
        FloatComparison naive;

        std::vector<std::int64_t> cycles(kChunk);
        std::vector<float>        phase(kChunk);
        float                     previous = 0.f;
        for (std::uint64_t at = 0ULL; at < samples; at += kChunk) {
            const std::size_t          n  = static_cast<std::size_t>(std::min<std::uint64_t>(kChunk, samples - at));
            const std::vector<Complex> in = tone(0.4, n, at);
            expect(unwrapper.process(in, std::span<std::int64_t>(cycles).first(n), std::span<float>(phase).first(n)).has_value());
            for (std::size_t k = 0UZ; k < n; ++k) {
                if (at + k > 0ULL) {
                    const float delta = phase[k] - previous;
                    accumulate(naive, (delta > CycleUnwrapper::kPi) ? (delta - 2.f * CycleUnwrapper::kPi) : ((delta < -CycleUnwrapper::kPi) ? (delta + 2.f * CycleUnwrapper::kPi) : delta));
                }
                previous = phase[k];
            }
        }

        const double naiveCycles  = static_cast<double>(naive.asFloat) / kTwoPi;
        const double doubleCycles = naive.asDouble / kTwoPi;
        std::println("after {} samples: cycles() = {}, a float sum reads {:.3f} turns (off by {:.3f}), a double sum {:.6f} (off by {:.3e}){}", samples, unwrapper.cycles(), naiveCycles, naiveCycles - static_cast<double>(turns), doubleCycles, doubleCycles - static_cast<double>(turns), longRun() ? " [ENABLE_LONG_TESTS]" : "");

        expect(unwrapper.cycles() == turns) << "the integer count is exact at" << turns << "turns";
        expect(std::abs(static_cast<double>(unwrapper.phase())) < 4. * 2.38e-7) << "the residual is zero within four times a float's spacing at pi";
        expect(std::abs(naiveCycles - static_cast<double>(turns)) > 1.) << "a float sum of the same steps is off by more than one cycle";
        expect(unwrapper.nSuspectSteps() == 0ULL) << "0.4 cycles per sample steps by 0.8*pi, below the default 0.9*pi";
        expect(unwrapper.nSaturations() == 0ULL);
    };

    "above half the sample rate the count follows the alias"_test = [] {
        constexpr std::int64_t kTurns  = 1LL << 14;
        const std::size_t      samples = static_cast<std::size_t>(5LL * kTurns / 2LL) + 1UZ;

        CycleUnwrapper            forward;
        std::vector<std::int64_t> cycles(samples);
        std::vector<float>        phase(samples);
        expect(forward.process(tone(0.4, samples), cycles, phase).has_value());
        expect(forward.cycles() == kTurns) << "0.4 cycles per sample unwraps exactly";
        expect(forward.nSuspectSteps() == 0ULL);

        // 0.6 cycles per sample steps by 1.2*pi. The step wraps to -0.8*pi, the step of -0.4 cycles per sample. The
        // two tones give the same samples.
        CycleUnwrapper aliased;
        expect(aliased.process(tone(0.6, samples), cycles, phase).has_value());
        expect(aliased.cycles() == -kTurns) << "0.6 cycles per sample reads the alias at -0.4 exactly";
        expect(aliased.nSuspectSteps() == 0ULL) << "the wrapped step is 0.8*pi, below the default 0.9*pi, and no step is flagged";

        // At 0.47 cycles per sample the step is 0.94*pi, above the default 0.9*pi. The unwrap is still exact.
        CycleUnwrapper close;
        expect(close.process(tone(0.47, 1'000UZ), cycles, phase).has_value());
        expect(close.nSuspectSteps() == 999ULL) << "every step after the first is flagged";
        expect(close.lastSuspectIndex().value() == 999ULL) << "the last flagged step carries its absolute index";

        CycleUnwrapper quiet;
        expect(!quiet.lastSuspectIndex().has_value()) << "an unwrapper with no flagged step has no index";
    };

    "the two origins give the same differences between samples"_test = [] {
        constexpr std::size_t kSamples = 4'096UZ;
        const auto            in       = tone(0.13, kSamples, 3UZ); // the first sample's phase is not zero

        std::vector<std::int64_t> cyclesA(kSamples);
        std::vector<float>        phaseA(kSamples);
        CycleUnwrapper            asIs(UnwrapOrigin::first_sample);
        expect(asIs.process(in, cyclesA, phaseA).has_value());

        std::vector<std::int64_t> cyclesB(kSamples);
        std::vector<float>        phaseB(kSamples);
        CycleUnwrapper            zeroed(UnwrapOrigin::zero);
        expect(zeroed.process(in, cyclesB, phaseB).has_value());

        expect(phaseA[0] == std::atan2(in[0].imag(), in[0].real())) << "first_sample reports the first sample's phase";
        expect(phaseB[0] == 0.f) << "zero starts at exactly zero";
        expect(cyclesA[0] == 0LL && cyclesB[0] == 0LL);

        double worst = 0.;
        for (std::size_t k = 1UZ; k < kSamples; ++k) {
            const double a = kTwoPi * static_cast<double>(cyclesA[k] - cyclesA[k - 1UZ]) + static_cast<double>(phaseA[k] - phaseA[k - 1UZ]);
            const double b = kTwoPi * static_cast<double>(cyclesB[k] - cyclesB[k - 1UZ]) + static_cast<double>(phaseB[k] - phaseB[k - 1UZ]);
            worst          = std::max(worst, std::abs(a - b));
        }
        expect(lt(worst, 1e-6)) << "the two origins give the same phase differences, worst" << worst;
    };

    "a discontinuity sets the count to zero and counts the reset"_test = [] {
        constexpr std::size_t kSamples = 2'000UZ;

        CycleUnwrapper            unwrapper;
        std::vector<std::int64_t> cycles(kSamples);
        std::vector<float>        phase(kSamples);

        expect(unwrapper.process(tone(0.1, kSamples), cycles, phase).has_value());
        const std::int64_t before = unwrapper.cycles();
        expect(before > 100LL) << "the count has run up";
        expect(unwrapper.nResets() == 0ULL);

        unwrapper.markDiscontinuity();
        expect(unwrapper.cycles() == 0LL) << "the count returns to zero";
        expect(unwrapper.nResets() == 1ULL) << "nResets() counts the discontinuity";
        expect(!unwrapper.started());

        expect(unwrapper.process(tone(0.1, kSamples, 5'000UZ), cycles, phase).has_value());
        expect(unwrapper.cycles() == before) << "the second stretch counts its own turns";
        expect(unwrapper.nResets() == 1ULL);

        // Without a call to markDiscontinuity, the count runs on across the two calls.
        CycleUnwrapper carried;
        expect(carried.process(tone(0.1, kSamples), cycles, phase).has_value());
        expect(carried.process(tone(0.1, kSamples, kSamples), cycles, phase).has_value());
        expect(carried.cycles() == 2LL * before) << "an uninterrupted count keeps running";
        expect(carried.nResets() == 0ULL);
    };

    "any split of the stream gives the same output"_test = [] {
        constexpr std::size_t kSamples = 12'000UZ;
        const auto            in       = tone(0.37, kSamples);

        CycleUnwrapper            whole;
        std::vector<std::int64_t> cyclesRef(kSamples);
        std::vector<float>        phaseRef(kSamples);
        expect(whole.process(in, cyclesRef, phaseRef).has_value());

        for (const std::size_t chunk : {1UZ, 7UZ, 1'000UZ, 12'345UZ}) {
            CycleUnwrapper            pieced;
            std::vector<std::int64_t> cycles(kSamples);
            std::vector<float>        phase(kSamples);
            for (std::size_t at = 0UZ; at < kSamples; at += chunk) {
                const std::size_t n = std::min(chunk, kSamples - at);
                expect(pieced.process(std::span<const Complex>(in).subspan(at, n), std::span<std::int64_t>(cycles).subspan(at, n), std::span<float>(phase).subspan(at, n)).has_value());
            }
            expect(that % std::ranges::equal(cycles, cyclesRef)) << std::format("chunk {} changed the count", chunk);
            expect(that % std::ranges::equal(phase, phaseRef)) << std::format("chunk {} changed the phase", chunk);
            expect(pieced.nSuspectSteps() == whole.nSuspectSteps());
        }
    };

    // At 0.2 cycles per sample the wrapped phase crosses pi between samples 5m + 2 and 5m + 3. A non-finite sample at
    // either end of a crossing hides it.
    "a non-finite sample repeats the current outputs, is counted, and the turn across it is kept"_test = [] {
        constexpr std::size_t kSamples = 2'001UZ; // 400 turns at 0.2 cycles per sample
        const auto            clean    = tone(0.2, kSamples);

        std::vector<std::int64_t> cyclesRef(kSamples);
        std::vector<float>        phaseRef(kSamples);
        CycleUnwrapper            reference;
        expect(reference.process(clean, cyclesRef, phaseRef).has_value());
        expect(eq(reference.cycles(), std::int64_t{400LL}));
        expect(eq(reference.nNonFiniteSamples(), std::uint64_t{0ULL}));

        std::vector<Complex> spoiled = clean;
        spoiled[1'002UZ]             = Complex{std::numeric_limits<float>::quiet_NaN(), 0.f};
        spoiled[1'503UZ]             = Complex{std::numeric_limits<float>::infinity(), 0.f};

        std::vector<std::int64_t> cycles(kSamples);
        std::vector<float>        phase(kSamples);
        CycleUnwrapper            unwrapper;
        expect(unwrapper.process(spoiled, cycles, phase).has_value());
        std::println("a NaN at 1002 and an infinity at 1503: cycles {} against {}, nNonFiniteSamples {}", unwrapper.cycles(), reference.cycles(), unwrapper.nNonFiniteSamples());
        expect(eq(unwrapper.cycles(), reference.cycles())) << "both turns across the non-finite samples are counted";
        expect(eq(unwrapper.nNonFiniteSamples(), std::uint64_t{2ULL})) << "a NaN and an infinite sample are each counted";
        expect(eq(cycles[1'002UZ], cycles[1'001UZ]) && eq(phase[1'002UZ], phase[1'001UZ])) << "the NaN sample repeats the outputs before it";
        expect(eq(cycles[1'503UZ], cycles[1'502UZ]) && eq(phase[1'503UZ], phase[1'502UZ])) << "the infinite sample repeats the outputs before it";
        expect(std::ranges::equal(std::span<const std::int64_t>(cycles).last(400UZ), std::span<const std::int64_t>(cyclesRef).last(400UZ))) << "the count after the gaps is the count of the clean stream";
        expect(std::ranges::all_of(phase, [](float value) { return std::isfinite(value); })) << "every residual is finite";

        // At 0.3 cycles per sample the phase moves 0.6 cycles over a one-sample gap. The step across it takes the alias,
        // -0.4 cycles, and the count is one turn short. nNonFiniteSamples() and nSuspectSteps() record the gap.
        auto           fast = tone(0.3, kSamples);
        CycleUnwrapper fastReference;
        expect(fastReference.process(fast, cyclesRef, phaseRef).has_value());
        fast[1'001UZ] = Complex{std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};
        CycleUnwrapper fastSpoiled;
        expect(fastSpoiled.process(fast, cycles, phase).has_value());
        expect(eq(fastSpoiled.cycles(), fastReference.cycles() - 1LL)) << "a gap over which the phase moves more than pi loses a turn";
        expect(eq(fastSpoiled.nNonFiniteSamples(), std::uint64_t{1ULL}));
        expect(eq(fastReference.nSuspectSteps(), std::uint64_t{0ULL}) && eq(fastSpoiled.nSuspectSteps(), std::uint64_t{1ULL})) << "the step across the gap is counted, and no other";
        expect(fastSpoiled.lastSuspectIndex() == std::optional<std::uint64_t>{1'002ULL}) << "the counted step ends at the finite sample after the gap";

        // A non-finite first sample leaves the origin to the next finite one.
        auto late = tone(0.13, 16UZ, 3ULL);
        late[0UZ] = Complex{std::numeric_limits<float>::quiet_NaN(), 1.f};
        CycleUnwrapper zeroed(UnwrapOrigin::zero);
        expect(zeroed.process(late, std::span<std::int64_t>(cycles).first(16UZ), std::span<float>(phase).first(16UZ)).has_value());
        expect(eq(phase[0UZ], 0.f) && eq(cycles[0UZ], std::int64_t{0LL})) << "the non-finite first sample reads zero";
        expect(eq(phase[1UZ], 0.f)) << "the first finite sample sets the origin";
        expect(eq(zeroed.nNonFiniteSamples(), std::uint64_t{1ULL}));

        zeroed.reset();
        expect(eq(zeroed.nNonFiniteSamples(), std::uint64_t{0ULL})) << "reset() clears the count";
    };

    // At 0.04 cycles per sample a gap of three samples moves the phase by 0.32*pi, below the default 0.9*pi. The gap at
    // 599 and 600 spans two calls. A discontinuity follows the gap at 799, and the gap at 0 has no finite sample before
    // it.
    "each gap of non-finite samples counts once among the suspect steps"_test = [] {
        constexpr std::size_t kSamples = 1'000UZ;
        const auto            clean    = tone(0.04, kSamples);
        auto                  spoiled  = clean;
        for (const std::size_t at : {0UZ, 100UZ, 400UZ, 401UZ, 402UZ, 599UZ, 600UZ, 799UZ}) {
            spoiled[at] = Complex{std::numeric_limits<float>::quiet_NaN(), 0.f};
        }

        std::vector<std::int64_t> cycles(kSamples);
        std::vector<float>        phase(kSamples);
        const auto                piece = [&cycles, &phase](CycleUnwrapper& unwrapper, const std::vector<Complex>& in, std::size_t from, std::size_t to) { return unwrapper.process(std::span<const Complex>(in).subspan(from, to - from), std::span<std::int64_t>(cycles).subspan(from, to - from), std::span<float>(phase).subspan(from, to - from)).has_value(); };
        const auto                split = [&piece](CycleUnwrapper& unwrapper, const std::vector<Complex>& in) {
            const bool before = piece(unwrapper, in, 0UZ, 600UZ) && piece(unwrapper, in, 600UZ, 800UZ);
            unwrapper.markDiscontinuity();
            return before && piece(unwrapper, in, 800UZ, kSamples);
        };

        CycleUnwrapper reference;
        CycleUnwrapper unwrapper;
        expect(split(reference, clean) && split(unwrapper, spoiled));
        std::println("gaps at 0, 100, 400-402, 599-600 and 799: nSuspectSteps {}, last at {}, nNonFiniteSamples {}", unwrapper.nSuspectSteps(), unwrapper.lastSuspectIndex().value_or(0ULL), unwrapper.nNonFiniteSamples());
        expect(eq(reference.nSuspectSteps(), std::uint64_t{0ULL})) << "no step of the clean stream crosses the threshold";
        expect(eq(unwrapper.nNonFiniteSamples(), std::uint64_t{8ULL}));
        expect(eq(unwrapper.nSuspectSteps(), std::uint64_t{3ULL})) << "the gaps at 100, 400 to 402 and 599 to 600 count once each";
        expect(unwrapper.lastSuspectIndex() == std::optional<std::uint64_t>{601ULL}) << "the last counted step ends at sample 601";
    };

    "unwrappedRadians sums the two parts, and a short output is refused"_test = [] {
        constexpr std::size_t kSamples = 101UZ;

        CycleUnwrapper            unwrapper;
        std::vector<std::int64_t> cycles(kSamples);
        std::vector<float>        phase(kSamples);
        expect(unwrapper.process(tone(0.1, kSamples), cycles, phase).has_value());

        expect(unwrapper.cycles() == 10LL) << "0.1 cycles per sample over 100 steps is ten turns";
        expect(unwrapper.processed() == kSamples);
        expect(approx(unwrapper.unwrappedRadians(), kTwoPi * 10., 1e-5)) << "unwrappedRadians() sums the two parts of the current sample";

        std::vector<std::int64_t> tooFew(3UZ);
        const std::uint64_t       before  = unwrapper.processed();
        const auto                refused = unwrapper.process(tone(0.1, 4UZ), tooFew, phase);
        expect(fatal(!refused.has_value())) << "one output short of the samples";
        expect(refused.error().message.contains("4 samples against 3 cycles")) << refused.error().message;
        expect(eq(unwrapper.processed(), before)) << "a refused call processes nothing";

        unwrapper.reset();
        expect(unwrapper.cycles() == 0LL);
        expect(unwrapper.processed() == 0ULL);
        expect(unwrapper.nResets() == 0ULL) << "reset() clears the counters";
        expect(!unwrapper.started());

        // At 10^9 S/s, `2^63 - 1` cycles take 584.55 years. No test reaches the limit.
        expect(unwrapper.nSaturations() == 0ULL);
    };
};

int main() { /* not needed for UT */ }
