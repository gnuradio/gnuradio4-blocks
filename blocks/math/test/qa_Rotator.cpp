#include <algorithm>
#include <boost/ut.hpp>
#include <cmath>
#include <complex>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/math/Rotator.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/algorithm/ImChart.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace {

struct RotationError {
    double phase     = 0.;
    double magnitude = 0.;
};

// worst deviation of the block's output from exp(j*(k+1)*phase_increment) over a stream fed in `chunk`-sized calls
template<typename T>
RotationError rotationError(std::size_t nSamples, double increment, std::size_t chunk) {
    using value_t = typename T::value_type;

    gr::blocks::math::Rotator<T> rot({{"phase_increment", static_cast<value_t>(increment)}, {"initial_phase", value_t(0)}});
    rot.settings().init();
    std::ignore = rot.settings().applyStagedParameters();

    const double applied = static_cast<double>(rot.phase_increment);

    std::vector<T> input(nSamples);
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        const double angle = 0.00037 * static_cast<double>(i);
        input[i]           = T(static_cast<value_t>(std::cos(angle)), static_cast<value_t>(std::sin(angle)));
    }

    std::vector<T> output(nSamples);
    for (std::size_t i = 0UZ; i < nSamples; i += chunk) {
        const std::size_t n = std::min(chunk, nSamples - i);
        std::ignore         = rot.processBulk(std::span<const T>(input.data() + i, n), std::span<T>(output.data() + i, n));
    }

    RotationError worst;
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        const std::complex<double> want = std::complex<double>(input[i]) * std::polar(1., static_cast<double>(i + 1UZ) * applied);
        const std::complex<double> have = std::complex<double>(output[i]);
        worst.phase                     = std::max(worst.phase, std::abs(std::arg(have / want)));
        worst.magnitude                 = std::max(worst.magnitude, std::abs(std::abs(have) / std::abs(want) - 1.));
    }
    return worst;
}

template<typename T>
std::vector<std::complex<T>> execRotator(const std::vector<std::complex<T>>& input, const gr::property_map& initSettings) {
    gr::blocks::math::Rotator<std::complex<T>> rot(initSettings);
    rot.settings().init();
    std::ignore = rot.settings().applyStagedParameters(); // needed for unit-test only when executed outside a Scheduler/Graph

    std::vector<std::complex<T>> output(input.size());
    std::ignore = rot.processBulk(std::span<const std::complex<T>>(input), std::span<std::complex<T>>(output));
    return output;
}

template<typename T>
void plotTimeDomain(const std::vector<std::complex<T>>& dataIn, const std::vector<std::complex<T>>& dataOut, float fs, const std::string& label) {
    std::vector<float> time(dataOut.size());
    for (std::size_t i = 0UZ; i < dataOut.size(); i++) {
        time[i] = static_cast<float>(i) / fs;
    }

    std::vector<float> inRe(dataOut.size());
    std::vector<float> inIm(dataOut.size());
    std::vector<float> outRe(dataOut.size());
    std::vector<float> outIm(dataOut.size());
    for (std::size_t i = 0UZ; i < dataOut.size(); i++) {
        inRe[i]  = static_cast<float>(dataIn[i].real());
        inIm[i]  = static_cast<float>(dataIn[i].imag());
        outRe[i] = static_cast<float>(dataOut[i].real());
        outIm[i] = static_cast<float>(dataOut[i].imag());
    }

    // quick chart
    gr::graphs::ImChart<100, 15> chart({{0.0f, time.back()}, {-1.5f, +1.5f}});
    chart.axis_name_x = "Time [s]";
    chart.axis_name_y = "Amplitude [a.u.]";

    chart.draw(time, inRe, "Re(in)");
    chart.draw(time, inIm, "Im(in)");
    chart.draw(time, outRe, std::format("out: Re({})", label));
    chart.draw(time, outIm, std::format("out: Im({})", label));
    chart.draw();
}

} // end anonymous namespace

const boost::ut::suite<"basic math tests"> basicMath = [] {
    using namespace boost::ut;
    using namespace gr::blocks::math;

    constexpr auto kArithmeticTypes = std::tuple<std::complex<float>, std::complex<double>>{};

    if (std::getenv("DISABLE_SENSITIVE_TESTS") == nullptr) {
        // conditionally enable visual tests outside the CI
        boost::ext::ut::cfg<override> = {.tag = {"visual", "benchmarks"}};
    }

    "Rotator - basic test"_test = []<typename T> {
        using value_t          = typename T::value_type;
        value_t    phase_shift = std::numbers::pi_v<value_t> / value_t(2);
        Rotator<T> rot({{"phase_increment", phase_shift}, {"initial_phase", value_t(0)}, {"sample_rate", 1.f}});
        rot.settings().init();
        std::ignore = rot.settings().applyStagedParameters(); // needed for unit-test only when executed outside a Scheduler/Graph

        expect(approx(rot.frequency_shift, 0.25f, 1e-3f));
        expect(approx(rot.initial_phase, value_t(0), value_t(1e-3f)));

        const std::vector<T> input(8UZ, std::complex<value_t>(1, 0));
        std::vector<T>       output(8UZ);
        std::ignore = rot.processBulk(std::span<const T>(input), std::span<T>(output));

        for (std::size_t i = 0; i < 8; i++) {
            value_t wantAngle = value_t(i + 1) * phase_shift;
            value_t wantCos   = std::cos(wantAngle);
            value_t wantSin   = std::sin(wantAngle);

            expect(approx(output[i].real(), wantCos, value_t(1e-5))) << "rotator real mismatch i=" << i;
            expect(approx(output[i].imag(), wantSin, value_t(1e-5))) << "rotator imag mismatch i=" << i;
        }
    } | kArithmeticTypes;

    "Rotator - a passing frequency tag is retuned by the shift"_test = [] {
        using namespace gr::blocks::testing;
        using T = std::complex<float>;

        constexpr double     kCenter   = 100.0e6;
        constexpr float      kShift    = 12.5e3f;
        constexpr float      kRate     = 1.0e6f;
        constexpr gr::Size_t kNSamples = 64U;

        gr::Graph flow;
        auto&     source = flow.emplaceBlock<TagSource<T>>({{"n_samples_max", kNSamples}, {"sample_rate", kRate}, {"disconnect_on_done", false}});
        source._tags     = {gr::Tag{0UZ, {{"frequency", kCenter}}}};
        auto& rotator    = flow.emplaceBlock<Rotator<T>>({{"sample_rate", kRate}, {"frequency_shift", kShift}});
        auto& sink       = flow.emplaceBlock<TagSink<T, ProcessFunction::USE_PROCESS_ONE>>({{"disconnect_on_done", false}});
        expect(flow.connect<"out", "in">(source, rotator).has_value());
        expect(flow.connect<"out", "in">(rotator, sink).has_value());

        gr::scheduler::Simple sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(sched.runAndWait().has_value());

        std::vector<double> centers;
        for (const gr::Tag& sinkTag : sink._tags) {
            const auto it = sinkTag.map.find(gr::tag::FREQUENCY.shortKey());
            if (it != sinkTag.map.end()) {
                if (const double* center = it->second.get_if<double>(); center != nullptr) {
                    centers.push_back(*center);
                }
            }
        }
        expect(eq(centers.size(), 1UZ)) << "the center frequency must reach the sink exactly once";
        if (centers.size() == 1UZ) {
            expect(approx(centers.front(), kCenter - static_cast<double>(kShift), 1e-3)) << "rotating the spectrum up moves the announced center down by the same amount";
        }
    };

    "Rotator - sample_rate change preserves the commanded Hz shift"_test = [] {
        using T = std::complex<double>;
        Rotator<T> rot({{"frequency_shift", 10.f}, {"sample_rate", 1000.f}});
        rot.settings().init();
        std::ignore = rot.settings().applyStagedParameters();

        const double incrementAt1k = static_cast<double>(rot.phase_increment);
        expect(approx(incrementAt1k, 2.0 * std::numbers::pi * 10.0 / 1000.0, 1e-6));

        std::ignore = rot.settings().setStaged({{"sample_rate", 2000.f}});
        std::ignore = rot.settings().applyStagedParameters();

        expect(approx(static_cast<float>(rot.frequency_shift), 10.f, 1e-4f)) << "commanded shift must survive a sample_rate change";
        expect(approx(static_cast<double>(rot.phase_increment), incrementAt1k / 2.0, 1e-9)) << "increment must rescale with sample_rate";
    };

    "Rotator - settings change without initial_phase keeps phase continuity"_test = [] {
        using T = std::complex<double>;
        Rotator<T> rot({{"frequency_shift", 1.f}, {"sample_rate", 64.f}});
        rot.settings().init();
        std::ignore = rot.settings().applyStagedParameters();

        const std::vector<T> input(32UZ, T(1.0, 0.0));
        std::vector<T>       first(32UZ);
        std::ignore = rot.processBulk(std::span<const T>(input), std::span<T>(first));

        std::ignore = rot.settings().setStaged({{"frequency_shift", 1.f}});
        std::ignore = rot.settings().applyStagedParameters();

        std::vector<T> second(32UZ);
        std::ignore = rot.processBulk(std::span<const T>(input), std::span<T>(second));

        const T continued = first.back() * std::polar(1.0, static_cast<double>(rot.phase_increment));
        expect(approx(second.front().real(), continued.real(), 1e-9)) << "phase must not reset on a settings change that omits initial_phase";
        expect(approx(second.front().imag(), continued.imag(), 1e-9)) << "phase must not reset on a settings change that omits initial_phase";
    };

    "Rotator - initial_phase in the staged keys resets the accumulator"_test = [] {
        using T = std::complex<double>;
        Rotator<T> rot({{"frequency_shift", 1.f}, {"sample_rate", 64.f}});
        rot.settings().init();
        std::ignore = rot.settings().applyStagedParameters();

        const std::vector<T> input(32UZ, T(1.0, 0.0));
        std::vector<T>       discard(32UZ);
        std::ignore = rot.processBulk(std::span<const T>(input), std::span<T>(discard));

        std::ignore = rot.settings().setStaged({{"initial_phase", 0.0}});
        std::ignore = rot.settings().applyStagedParameters();

        std::vector<T> restarted(32UZ);
        std::ignore = rot.processBulk(std::span<const T>(input), std::span<T>(restarted));

        const double increment = static_cast<double>(rot.phase_increment);
        expect(approx(restarted.front().real(), std::cos(increment), 1e-9));
        expect(approx(restarted.front().imag(), std::sin(increment), 1e-9));
    };

    // 65536 samples is 16 re-seed intervals; the increments span slow, fast and near-pi rotations
    "Rotator - long stream stays on the exact rotation"_test = []<typename T> {
        constexpr std::size_t nSamples  = 65536UZ;
        constexpr double      tolerance = std::is_same_v<typename T::value_type, float> ? 1e-4 : 1e-9;

        for (double increment : {1e-4, 0.05, 1.0, 3.0, -2.7, 6.0}) {
            const auto error = rotationError<T>(nSamples, increment, nSamples);
            expect(lt(error.phase, tolerance)) << std::format("phase drift over {} samples, increment {}", nSamples, increment);
            expect(lt(error.magnitude, tolerance)) << std::format("magnitude drift over {} samples, increment {}", nSamples, increment);
        }
    } | kArithmeticTypes;

    // the same bound has to survive call boundaries that land off both the lane width and the re-seed interval
    "Rotator - odd chunk sizes stay on the exact rotation"_test = []<typename T> {
        constexpr std::size_t nSamples  = 65536UZ;
        constexpr double      tolerance = std::is_same_v<typename T::value_type, float> ? 1e-4 : 1e-9;

        for (std::size_t chunk : {1000UZ, 4095UZ, 4096UZ, 4097UZ, 16384UZ}) {
            const auto error = rotationError<T>(nSamples, 0.2718281828, chunk);
            expect(lt(error.phase, tolerance)) << std::format("phase drift over {} samples in {}-sample calls", nSamples, chunk);
            expect(lt(error.magnitude, tolerance)) << std::format("magnitude drift over {} samples in {}-sample calls", nSamples, chunk);
        }
    } | kArithmeticTypes;

    // short calls re-seed from '_accumulated_phase' every call, so its rounding is the floor here rather
    // than the loop's: in double that floor is flat, i.e. the bound does not grow with the call count
    "Rotator - short calls stay bounded as the stream grows"_test = []<typename T> {
        constexpr double tolerance = std::is_same_v<typename T::value_type, float> ? 1e-5 : 1e-9;

        for (std::size_t nSamples : {16384UZ, 65536UZ}) {
            for (std::size_t chunk : {1UZ, 3UZ, 64UZ}) {
                for (double increment : {0.2718281828, 6.0}) {
                    const auto error = rotationError<T>(nSamples, increment, chunk);
                    expect(lt(error.phase, tolerance)) << std::format("phase drift over {} samples in {}-sample calls, increment {}", nSamples, chunk, increment);
                    expect(lt(error.magnitude, tolerance)) << std::format("magnitude drift over {} samples in {}-sample calls, increment {}", nSamples, chunk, increment);
                }
            }
        }
    } | kArithmeticTypes;

    "Rotator - phase is continuous across a frequency change mid-stream"_test = [] {
        using T = std::complex<double>;
        Rotator<T> rot({{"frequency_shift", 2.f}, {"sample_rate", 64.f}});
        rot.settings().init();
        std::ignore = rot.settings().applyStagedParameters();

        const std::vector<T> input(4100UZ, T(1.0, 0.0)); // spans a re-seed boundary and ends mid-lane
        std::vector<T>       before(4100UZ);
        std::ignore = rot.processBulk(std::span<const T>(input), std::span<T>(before));

        std::ignore = rot.settings().setStaged({{"frequency_shift", -7.f}});
        std::ignore = rot.settings().applyStagedParameters();

        std::vector<T> after(4100UZ);
        std::ignore = rot.processBulk(std::span<const T>(input), std::span<T>(after));

        const T continued = before.back() * std::polar(1.0, static_cast<double>(rot.phase_increment));
        expect(approx(after.front().real(), continued.real(), 1e-9)) << "a frequency change must advance the phase, not restart it";
        expect(approx(after.front().imag(), continued.imag(), 1e-9)) << "a frequency change must advance the phase, not restart it";
    };

    "Rotator - one sample at a time rotates as one element of a bulk call"_test = [] {
        using T                      = std::complex<double>;
        constexpr std::size_t  kN    = 4100UZ; // spans a re-seed boundary and ends mid-lane
        const gr::property_map kInit = {{"frequency_shift", 2.f}, {"sample_rate", 64.f}};

        std::vector<T> input(kN);
        for (std::size_t i = 0UZ; i < kN; ++i) {
            const double angle = 0.00037 * static_cast<double>(i);
            input[i]           = T(std::cos(angle), std::sin(angle));
        }

        Rotator<T> bulk(kInit);
        bulk.settings().init();
        std::ignore = bulk.settings().applyStagedParameters();
        std::vector<T> bulkOut(kN);
        std::ignore = bulk.processBulk(std::span<const T>(input), std::span<T>(bulkOut));

        Rotator<T> stepwise(kInit);
        stepwise.settings().init();
        std::ignore  = stepwise.settings().applyStagedParameters();
        double worst = 0.;
        T      last{};
        for (std::size_t i = 0UZ; i < kN; ++i) {
            last  = stepwise.rotateOne(input[i]);
            worst = std::max(worst, std::abs(last - bulkOut[i]));
        }
        expect(lt(worst, 1e-9)) << std::format("the single-sample entry point must match the bulk one, worst deviation {}", worst);

        // the phase carries across a retune, and the two entry points share it
        const T    unit{1.0, 0.0};
        Rotator<T> retuned(kInit);
        retuned.settings().init();
        std::ignore = retuned.settings().applyStagedParameters();
        T before{};
        for (std::size_t i = 0UZ; i < kN; ++i) {
            before = retuned.rotateOne(unit);
        }

        std::ignore       = retuned.settings().setStaged({{"frequency_shift", -7.f}});
        std::ignore       = retuned.settings().applyStagedParameters();
        const T continued = before * std::polar(1.0, static_cast<double>(retuned.phase_increment));
        const T after     = retuned.rotateOne(unit);
        expect(approx(after.real(), continued.real(), 1e-9)) << "a retune must advance the phase, not restart it";
        expect(approx(after.imag(), continued.imag(), 1e-9)) << "a retune must advance the phase, not restart it";

        const std::vector<T> ones(4UZ, unit);
        std::vector<T>       tail(4UZ);
        std::ignore     = retuned.processBulk(std::span<const T>(ones), std::span<T>(tail));
        const T resumed = after * std::polar(1.0, static_cast<double>(retuned.phase_increment));
        expect(approx(tail.front().real(), resumed.real(), 1e-9)) << "a bulk call must resume the phase a single-sample call left";
        expect(approx(tail.front().imag(), resumed.imag(), 1e-9)) << "a bulk call must resume the phase a single-sample call left";
    };

    constexpr static float fs    = 100.0; // sampling rate
    constexpr static float tMax  = 2.0;   // seconds
    constexpr static auto  nSamp = static_cast<std::size_t>(fs * tMax);

    tag("visual") / "RotatorTest - DC->2 Hz shift"_test = [] {
        std::vector<std::complex<double>> input(nSamp, std::complex<double>(std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0));
        auto                              output = execRotator(input, {{"frequency_shift", +2.f}, {"sample_rate", fs}});
        plotTimeDomain(input, output, fs, "DC->+2 Hz");
    };

    tag("visual") / "RotatorTest - 0.5 Hz => shift +1.5 => 2 Hz"_test = [] {
        std::vector<std::complex<double>> input(nSamp);
        for (std::size_t i = 0; i < nSamp; i++) { // 0.5 Hz complex sinusoid
            double t     = static_cast<double>(i) / static_cast<double>(fs);
            double angle = 2.0 * std::numbers::pi * 0.5 * t; // 0.5 Hz
            input[i]     = {std::cos(angle), std::sin(angle)};
        }
        auto output = execRotator(input, {{"frequency_shift", +1.5f}, {"sample_rate", fs}});
        plotTimeDomain(input, output, fs, ".5->2 Hz");
    };

    tag("visual") / "RotatorTest - 2 Hz => shift -1.5 => 0.5 Hz"_test = [] {
        std::vector<std::complex<double>> input(nSamp);
        for (std::size_t i = 0; i < nSamp; i++) { // 2 Hz complex sinusoid
            double t     = static_cast<double>(i) / static_cast<double>(fs);
            double angle = 2.0 * std::numbers::pi * 2.0 * t; // 2 Hz
            input[i]     = {std::cos(angle), std::sin(angle)};
        }
        auto output = execRotator(input, {{"frequency_shift", -1.5f}, {"sample_rate", fs}});
        plotTimeDomain(input, output, fs, "2->.5 Hz");
    };
};

int main() { /* not needed for UT */ }
