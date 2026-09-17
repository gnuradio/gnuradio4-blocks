#include <boost/ut.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <numbers>
#include <print>
#include <span>
#include <vector>

#include <gnuradio-4.0/analog/Squelch.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::analog::PowerSquelch;
namespace spans = gr::blocks::testing::span;

using CF = std::complex<float>;

constexpr float       kQuiet = 0.05f; // power 2.5e-3, below the -20 dB threshold these tests use
constexpr float       kLoud  = 1.0f;  // power 1.0, above it
constexpr std::size_t kRamp  = 16UZ;
constexpr std::size_t kLead = 32UZ, kBurst = 64UZ, kTail = 64UZ;

template<typename T>
[[nodiscard]] PowerSquelch<T> makeSquelch(gr::property_map settings) {
    PowerSquelch<T> block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

/// @brief An alpha of one makes the detector memoryless, so the gate-machine tests exercise the state machine and not the averager.
[[nodiscard]] gr::property_map gateSettings(gr::Size_t ramp, bool gate) { return {{"alpha", 1.0}, {"threshold_db", -20.0}, {"ramp", ramp}, {"gate", gate}}; }

[[nodiscard]] std::vector<CF> stepBurst(std::size_t lead = kLead, std::size_t burst = kBurst, std::size_t tail = kTail) {
    std::vector<CF> signal(lead + burst + tail, CF{kQuiet, 0.f});
    std::fill(signal.begin() + static_cast<std::ptrdiff_t>(lead), signal.begin() + static_cast<std::ptrdiff_t>(lead + burst), CF{kLoud, 0.f});
    return signal;
}

[[nodiscard]] std::vector<CF> steady(std::size_t nSamples, double magnitude) { return std::vector<CF>(nSamples, CF{static_cast<float>(magnitude), 0.f}); }

[[nodiscard]] double envelope(std::size_t r, std::size_t ramp) { return 0.5 * (1.0 - std::cos(std::numbers::pi * static_cast<double>(r) / static_cast<double>(ramp))); }

[[nodiscard]] std::vector<double> scaledBy(std::span<const CF> samples, double reference) {
    std::vector<double> values(samples.size());
    for (std::size_t i = 0UZ; i < samples.size(); ++i) {
        values[i] = static_cast<double>(samples[i].real()) / reference;
    }
    return values;
}

[[nodiscard]] std::vector<char> observeUnmuted(PowerSquelch<CF>& block, std::span<const CF> input) {
    std::vector<char> open(input.size());
    for (std::size_t i = 0UZ; i < input.size(); ++i) {
        std::ignore = spans::run<PowerSquelch<CF>, CF>(block, input.subspan(i, 1UZ));
        open[i]     = block.unmuted.value ? 1 : 0;
    }
    return open;
}

[[nodiscard]] bool allFinite(std::span<const CF> samples, double bound) {
    return std::ranges::all_of(samples, [bound](CF sample) { return std::isfinite(sample.real()) && std::isfinite(sample.imag()) && std::abs(static_cast<double>(sample.real())) <= bound; });
}

} // namespace

const boost::ut::suite<"PowerSquelch"> powerSquelchTests = [] {
    using namespace boost::ut;
    using gr::blocks::analog::detail::kSquelchEndOfBurst;
    using gr::blocks::analog::detail::kSquelchStartOfBurst;

    "the attack occupies exactly ramp samples and reaches unity on the last"_test = [] {
        PowerSquelch<CF> block  = makeSquelch<CF>(gateSettings(static_cast<gr::Size_t>(kRamp), false));
        const auto       input  = stepBurst();
        const auto       result = spans::run<PowerSquelch<CF>, CF>(block, std::span<const CF>(input));

        expect(eq(result.samples.size(), input.size())) << "gate = false is exactly 1:1";
        const std::vector<double> attack = scaledBy(std::span<const CF>(result.samples).subspan(kLead, kRamp), static_cast<double>(kLoud));
        for (std::size_t r = 1UZ; r <= kRamp; ++r) {
            expect(approx(attack[r - 1UZ], envelope(r, kRamp), 1e-6)) << "attack envelope at r=" << r;
        }
        expect(eq(attack.back(), 1.0)) << "the ramp reaches unity on its last sample";
        expect(approx(attack[0], 0.009607, 1e-6));
        expect(approx(attack[3], 0.146447, 1e-6));
        expect(eq(attack[7], 0.5));
        expect(approx(attack[11], 0.853553, 1e-6));
        expect(eq(static_cast<double>(result.samples[kLead + kRamp].real()), 1.0)) << "and stays there";
    };

    "the decay is the attack reversed, and both are strictly monotone"_test = [] {
        PowerSquelch<CF> block  = makeSquelch<CF>(gateSettings(static_cast<gr::Size_t>(kRamp), false));
        const auto       input  = stepBurst();
        const auto       result = spans::run<PowerSquelch<CF>, CF>(block, std::span<const CF>(input));

        const std::vector<double> attack = scaledBy(std::span<const CF>(result.samples).subspan(kLead, kRamp), static_cast<double>(kLoud));
        std::vector<double>       decay  = scaledBy(std::span<const CF>(result.samples).subspan(kLead + kBurst + 1UZ, kRamp), static_cast<double>(kQuiet));
        std::ranges::reverse(decay);

        for (std::size_t r = 0UZ; r < kRamp; ++r) {
            expect(approx(decay[r], attack[r], 1e-6)) << "decay reversed against attack at " << r;
        }
        expect(std::ranges::is_sorted(attack, std::ranges::less{})) << "the attack is monotone";
        expect(std::ranges::adjacent_find(attack) == attack.end()) << "and strictly so";
    };

    "the burst tags bracket the produced samples that carry signal"_test = [] {
        PowerSquelch<CF> open   = makeSquelch<CF>(gateSettings(static_cast<gr::Size_t>(kRamp), false));
        const auto       input  = stepBurst();
        const auto       result = spans::run<PowerSquelch<CF>, CF>(open, std::span<const CF>(input));

        expect(that % (result.offsetsOf(kSquelchStartOfBurst) == std::vector<std::size_t>{kLead})) << "sob sits on the env(1) sample, not after the ramp";
        expect(that % (result.offsetsOf(kSquelchEndOfBurst) == std::vector<std::size_t>{kLead + kBurst + kRamp})) << "eob sits on the final env(1) sample, not on the zero after it";

        PowerSquelch<CF>  gating   = makeSquelch<CF>(gateSettings(static_cast<gr::Size_t>(kRamp), true));
        const auto        gated    = spans::run<PowerSquelch<CF>, CF>(gating, std::span<const CF>(input));
        const std::size_t expected = kRamp + (kBurst - kRamp + 1UZ) + kRamp; // attack, the unmuted run, decay

        expect(eq(gated.samples.size(), expected)) << "gating produces exactly the burst";
        expect(eq(gated.consumed, input.size())) << "and consumes everything";
        expect(that % (gated.offsetsOf(kSquelchStartOfBurst) == std::vector<std::size_t>{0UZ}));
        expect(that % (gated.offsetsOf(kSquelchEndOfBurst) == std::vector<std::size_t>{expected - 1UZ}));
        expect(std::ranges::none_of(gated.samples, [](CF sample) { return sample.real() == 0.f; })) << "no zero reaches the output while gating";
    };

    "ramp = 0 makes both transitions instantaneous"_test = [] {
        PowerSquelch<CF> block  = makeSquelch<CF>(gateSettings(0U, false));
        const auto       input  = stepBurst();
        const auto       result = spans::run<PowerSquelch<CF>, CF>(block, std::span<const CF>(input));

        expect(that % (result.offsetsOf(kSquelchStartOfBurst) == std::vector<std::size_t>{kLead}));
        expect(that % (result.offsetsOf(kSquelchEndOfBurst) == std::vector<std::size_t>{kLead + kBurst}));
        expect(eq(static_cast<double>(result.samples[kLead].real()), 1.0)) << "the first passed sample is at full amplitude";
        expect(eq(static_cast<double>(result.samples[kLead + kBurst].real()), static_cast<double>(kQuiet))) << "and so is the sample the detector closed on";
        expect(eq(static_cast<double>(result.samples[kLead + kBurst + 1UZ].real()), 0.0));
    };

    "a one-sample detection still produces a full 2*ramp+1 burst"_test = [] {
        std::vector<CF> input(200UZ, CF{kQuiet, 0.f});
        input[40UZ] = CF{kLoud, 0.f};

        PowerSquelch<CF> block  = makeSquelch<CF>(gateSettings(static_cast<gr::Size_t>(kRamp), true));
        const auto       result = spans::run<PowerSquelch<CF>, CF>(block, std::span<const CF>(input));
        expect(eq(result.samples.size(), 2UZ * kRamp + 1UZ)) << "attack and decay run to completion whatever the detector does";
    };

    "a ramp canceled or rescaled mid-ramp stays finite and terminates"_test = [] {
        for (const gr::Size_t replacement : {0U, 8U}) {
            for (const std::size_t interrupt : {kLead + 10UZ, kLead + kBurst + 10UZ}) { // mid-attack, then mid-decay
                PowerSquelch<CF> block = makeSquelch<CF>(gateSettings(64U, false));
                const auto       input = stepBurst(kLead, kBurst, 400UZ);

                const auto before = spans::run<PowerSquelch<CF>, CF>(block, std::span<const CF>(input).first(interrupt));
                std::ignore       = block.settings().setStaged({{"ramp", replacement}});
                std::ignore       = block.settings().applyStagedParameters();
                const auto after  = spans::run<PowerSquelch<CF>, CF>(block, std::span<const CF>(input).subspan(interrupt));

                expect(allFinite(before.samples, 1.0)) << "ramp -> " << replacement << " at " << interrupt;
                expect(allFinite(after.samples, 1.0)) << "ramp -> " << replacement << " at " << interrupt;
                expect(!block.unmuted.value) << "the burst must terminate after the change";
                expect(eq(before.tags.size() + after.tags.size(), 2UZ)) << "one sob and one eob, neither lost nor duplicated";
            }
        }
    };

    "the observable is true exactly while open or attacking"_test = [] {
        PowerSquelch<CF>        block = makeSquelch<CF>(gateSettings(static_cast<gr::Size_t>(kRamp), false));
        const auto              input = stepBurst();
        const std::vector<char> open  = observeUnmuted(block, std::span<const CF>(input));

        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            const bool expected = i >= kLead && i < kLead + kBurst; // the sample that closes the gate is already in the Decay state
            expect(eq(open[i] != 0, expected)) << "unmuted at sample " << i;
        }
    };

    "output and tag offsets do not depend on chunking"_test = [] {
        const auto input = stepBurst();
        for (const bool gate : {false, true}) {
            PowerSquelch<CF> reference = makeSquelch<CF>(gateSettings(static_cast<gr::Size_t>(kRamp), gate));
            const auto       want      = spans::run<PowerSquelch<CF>, CF>(reference, std::span<const CF>(input));

            for (const std::size_t chunkSize : {1UZ, 5UZ, 17UZ, 4096UZ}) {
                PowerSquelch<CF> block = makeSquelch<CF>(gateSettings(static_cast<gr::Size_t>(kRamp), gate));
                const auto       got   = spans::run<PowerSquelch<CF>, CF>(block, std::span<const CF>(input), chunkSize);
                expect(std::ranges::equal(got.samples, want.samples)) << "gate=" << gate << " chunk " << chunkSize << " must be bit-identical";
                expect(that % (got.offsetsOf(kSquelchStartOfBurst) == want.offsetsOf(kSquelchStartOfBurst))) << "gate=" << gate << " chunk " << chunkSize;
                expect(that % (got.offsetsOf(kSquelchEndOfBurst) == want.offsetsOf(kSquelchEndOfBurst))) << "gate=" << gate << " chunk " << chunkSize;
            }
        }
    };

    "an input tag on a suppressed sample moves forward to the next produced one"_test = [] {
        const auto           input = stepBurst();
        std::vector<gr::Tag> tags;
        for (const std::size_t at : {0UZ, 5UZ, 31UZ, kLead, kLead + 3UZ}) {
            tags.push_back(gr::Tag{at, gr::property_map{{"probe", static_cast<gr::Size_t>(at)}}});
        }

        PowerSquelch<CF> open = makeSquelch<CF>(gateSettings(static_cast<gr::Size_t>(kRamp), false));
        const auto       kept = spans::run<PowerSquelch<CF>, CF>(open, std::span<const CF>(input), 0UZ, std::span<const gr::Tag>(tags));
        expect(that % (kept.offsetsOf("probe") == std::vector<std::size_t>{0UZ, 5UZ, 31UZ, kLead, kLead + 3UZ})) << "1:1 leaves every offset alone";

        PowerSquelch<CF> gating = makeSquelch<CF>(gateSettings(static_cast<gr::Size_t>(kRamp), true));
        const auto       moved  = spans::run<PowerSquelch<CF>, CF>(gating, std::span<const CF>(input), 7UZ, std::span<const gr::Tag>(tags));
        expect(that % (moved.offsetsOf("probe") == std::vector<std::size_t>{0UZ, 0UZ, 0UZ, 0UZ, 3UZ})) << "four suppressed tags land on the burst's first produced sample, in input order";
    };

    "an input tag is republished whole, non-reserved keys included"_test = [] {
        const auto                 input = stepBurst();
        const std::vector<gr::Tag> tags{gr::Tag{kLead + 8UZ, gr::property_map{{"sample_rate", 48000.f}, {"test_probe", static_cast<gr::Size_t>(7)}}}};

        PowerSquelch<CF> block = makeSquelch<CF>(gateSettings(static_cast<gr::Size_t>(kRamp), true));
        const auto       got   = spans::run<PowerSquelch<CF>, CF>(block, std::span<const CF>(input), 0UZ, std::span<const gr::Tag>(tags));

        // The framework's default forwarding keeps only the auto-forward keys and drops the rest. This
        // block republishes every key it was handed.
        expect(eq(got.offsetsOf("test_probe").size(), 1UZ)) << "a key no block declares still reaches the output";
        expect(that % (got.offsetsOf("sample_rate") == got.offsetsOf("test_probe"))) << "and rides the same output tag it arrived on";
    };

    "a passing sample_rate leaves carrying this block's own rate"_test = [] {
        constexpr float kUpstream = 192000.f;
        constexpr float kOwn      = 48000.f;

        const auto       input    = stepBurst();
        gr::property_map settings = gateSettings(static_cast<gr::Size_t>(kRamp), true);
        settings.insert_or_assign(gr::property_map::key_type{"sample_rate"}, gr::pmt::Value(kOwn));

        const std::vector<gr::Tag> tags{gr::Tag{kLead + 8UZ, gr::property_map{{"sample_rate", kUpstream}, {"test_probe", static_cast<gr::Size_t>(7)}}}};

        PowerSquelch<CF> block = makeSquelch<CF>(std::move(settings));
        const auto       got   = spans::run<PowerSquelch<CF>, CF>(block, std::span<const CF>(input), 0UZ, std::span<const gr::Tag>(tags));

        std::vector<float> rates;
        for (const gr::Tag& forwarded : got.tags) {
            if (const auto found = forwarded.map.find(gr::property_map::key_type{"sample_rate"}); found != forwarded.map.end()) {
                rates.push_back(found->second.value_or(0.f));
            }
        }
        expect(that % (rates == std::vector<float>{kOwn})) << "the block declares the reserved key, so what it forwards is its own value and not the upstream one";
        expect(eq(got.offsetsOf("test_probe").size(), 1UZ)) << "and the substitution does not cost the private key its passage";
    };

    "threshold_db reads back exactly as set"_test = [] {
        PowerSquelch<CF> block = makeSquelch<CF>({{"threshold_db", -37.5}});
        expect(approx(block.threshold_db.value, -37.5, 1e-6)) << "never round-tripped through the linear form";
    };

    "the steady-state decision is a power comparison"_test = [] {
        constexpr double kThreshold = -30.0;
        for (const double offsetDb : {0.5, -0.5}) {
            PowerSquelch<CF>      block  = makeSquelch<CF>({{"alpha", 0.1}, {"threshold_db", kThreshold}});
            const std::vector<CF> signal = steady(2000UZ, std::pow(10.0, (kThreshold + offsetDb) / 20.0));
            std::ignore                  = spans::run<PowerSquelch<CF>, CF>(block, std::span<const CF>(signal));
            expect(eq(block.unmuted.value, offsetDb > 0.0)) << "a signal " << offsetDb << " dB from the threshold";
        }

        // -6.02 dB is magnitude 0.5, which is what the /10 conversion says; the /20 one would put it at 0.7075
        PowerSquelch<CF>      boundary = makeSquelch<CF>({{"alpha", 1.0}, {"threshold_db", -6.02}});
        const std::vector<CF> above    = steady(4UZ, 0.6);
        const std::vector<CF> below    = steady(4UZ, 0.45);
        std::ignore                    = spans::run<PowerSquelch<CF>, CF>(boundary, std::span<const CF>(above));
        expect(boundary.unmuted.value) << "magnitude 0.6 is above a -6.02 dB power threshold";
        std::ignore = spans::run<PowerSquelch<CF>, CF>(boundary, std::span<const CF>(below));
        expect(!boundary.unmuted.value) << "and 0.45 is below it";
    };

    "averaging_time_s sets alpha from a time constant"_test = [] {
        PowerSquelch<CF> block = makeSquelch<CF>({{"sample_rate", 48000.f}, {"averaging_time_s", 0.02}});
        expect(approx(block.alpha.value, 1.0 - std::exp(-1.0 / 960.0), 1e-12)) << "1 - exp(-1/(fs*T)) at fs*T = 960";
        expect(approx(block.alpha.value, 1.0411243e-3, 1e-9)) << "the exact value of 1 - exp(-1/960), not a rounded one";
    };

    "the average starts at zero, so the block starts muted and takes time to open"_test = [] {
        constexpr double      kAlpha = 1e-4;
        PowerSquelch<CF>      block  = makeSquelch<CF>({{"alpha", kAlpha}, {"threshold_db", -20.0}});
        const std::vector<CF> first  = steady(1UZ, 1.0);
        const std::vector<CF> rest   = steady(static_cast<std::size_t>(3.0 / kAlpha), 1.0);

        std::ignore = spans::run<PowerSquelch<CF>, CF>(block, std::span<const CF>(first));
        expect(!block.unmuted.value) << "muted at sample 0";
        std::ignore = spans::run<PowerSquelch<CF>, CF>(block, std::span<const CF>(rest));
        expect(block.unmuted.value) << "and open within 3/alpha samples";
    };

    "unusable parameters are rejected at settings time"_test = [] {
        expect(throws([] { std::ignore = makeSquelch<CF>({{"alpha", 0.0}}); })) << "zero alpha";
        expect(throws([] { std::ignore = makeSquelch<CF>({{"alpha", 1.5}}); })) << "alpha above one";
        expect(throws([] { std::ignore = makeSquelch<CF>({{"averaging_time_s", 0.02}, {"sample_rate", 0.f}}); })) << "zero sample_rate with a time constant";
    };

    "nanoseconds per sample"_test = [] {
        if (std::getenv("ENABLE_BENCHMARK_TESTS") == nullptr) {
            return; // opt-in: a throughput figure belongs to a controlled run, not to every ctest invocation
        }
        using Clock = std::chrono::steady_clock;

        std::vector<CF> x(1UZ << 16);
        for (std::size_t i = 0UZ; i < x.size(); ++i) {
            const double magnitude = 0.2 + 0.8 * std::abs(std::sin(2.0 * std::numbers::pi * 11.0 * static_cast<double>(i) / static_cast<double>(x.size())));
            x[i]                   = CF{static_cast<float>(magnitude), static_cast<float>(0.5 * magnitude)};
        }
        std::vector<CF> y(x.size());

        struct Arm {
            const char*      label;
            gr::property_map settings;
        };
        const Arm     kArms[]  = {{"PowerSquelch open", {{"threshold_db", -60.0}}}, {"PowerSquelch ramping", {{"threshold_db", -12.0}, {"ramp", 64U}}}, {"PowerSquelch gating", {{"threshold_db", -12.0}, {"ramp", 64U}, {"gate", true}}}};
        constexpr int kRepeats = 7;

        std::vector<PowerSquelch<CF>> blocks;
        for (const Arm& arm : kArms) {
            blocks.push_back(makeSquelch<CF>(arm.settings));
        }

        std::vector<double> best(std::size(kArms), 1e30);
        std::vector<double> worst(std::size(kArms), 0.0);
        for (int repeat = 0; repeat < kRepeats; ++repeat) { // arms interleaved, so a thermal drift moves all of them
            for (std::size_t a = 0UZ; a < std::size(kArms); ++a) {
                spans::InputSpan<CF>  inSpan{std::span<const CF>(x)};
                spans::OutputSpan<CF> outSpan{std::span<CF>(y)};
                const auto            start = Clock::now();
                std::ignore                 = blocks[a].processBulk(inSpan, outSpan);
                const double ns             = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count()) / static_cast<double>(x.size());
                best[a]                     = std::min(best[a], ns);
                worst[a]                    = std::max(worst[a], ns);
            }
        }
        for (std::size_t a = 0UZ; a < std::size(kArms); ++a) {
            std::println("{}: best {:.3f} ns/sample, spread {:.3f} ns", kArms[a].label, best[a], worst[a] - best[a]);
        }
    };
};

int main() { /* tests are automatically registered and run */ }
