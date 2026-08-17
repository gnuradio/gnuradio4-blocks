#include <boost/ut.hpp>
#include <cmath>
#include <complex>
#include <numbers>

#include <gnuradio-4.0/math/Rotator.hpp>

#include <gnuradio-4.0/algorithm/ImChart.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>

namespace {

template<typename T>
std::vector<std::complex<T>> execRotator(const std::vector<std::complex<T>>& input, const gr::property_map& initSettings) {
    gr::blocks::math::Rotator<std::complex<T>> rot(initSettings);
    rot.settings().init();
    std::ignore = rot.settings().applyStagedParameters(); // needed for unit-test only when executed outside a Scheduler/Graph

    std::vector<std::complex<T>> output(input.size());
    for (std::size_t i = 0; i < input.size(); i++) {
        output[i] = rot.processOne(input[i]);
    }
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

        std::vector<T> output(8UZ);
        for (std::size_t i = 0; i < 8; i++) {
            output[i] = rot.processOne(std::complex<value_t>(1, 0));
        }

        for (std::size_t i = 0; i < 8; i++) {
            value_t wantAngle = value_t(i + 1) * phase_shift;
            value_t wantCos   = std::cos(wantAngle);
            value_t wantSin   = std::sin(wantAngle);

            expect(approx(output[i].real(), wantCos, value_t(1e-5))) << "rotator real mismatch i=" << i;
            expect(approx(output[i].imag(), wantSin, value_t(1e-5))) << "rotator imag mismatch i=" << i;
        }
    } | kArithmeticTypes;

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

        std::vector<T> first(32UZ);
        for (std::size_t i = 0; i < first.size(); i++) {
            first[i] = rot.processOne(T(1.0, 0.0));
        }

        std::ignore = rot.settings().setStaged({{"frequency_shift", 1.f}});
        std::ignore = rot.settings().applyStagedParameters();

        std::vector<T> second(32UZ);
        for (std::size_t i = 0; i < second.size(); i++) {
            second[i] = rot.processOne(T(1.0, 0.0));
        }

        const T continued = first.back() * std::polar(1.0, static_cast<double>(rot.phase_increment));
        expect(approx(second.front().real(), continued.real(), 1e-9)) << "phase must not reset on a settings change that omits initial_phase";
        expect(approx(second.front().imag(), continued.imag(), 1e-9)) << "phase must not reset on a settings change that omits initial_phase";
    };

    "Rotator - initial_phase in the staged keys resets the accumulator"_test = [] {
        using T = std::complex<double>;
        Rotator<T> rot({{"frequency_shift", 1.f}, {"sample_rate", 64.f}});
        rot.settings().init();
        std::ignore = rot.settings().applyStagedParameters();

        for (std::size_t i = 0; i < 32UZ; i++) {
            std::ignore = rot.processOne(T(1.0, 0.0));
        }

        std::ignore = rot.settings().setStaged({{"initial_phase", 0.0}});
        std::ignore = rot.settings().applyStagedParameters();

        std::vector<T> restarted(32UZ);
        for (std::size_t i = 0; i < restarted.size(); i++) {
            restarted[i] = rot.processOne(T(1.0, 0.0));
        }

        const double increment = static_cast<double>(rot.phase_increment);
        expect(approx(restarted.front().real(), std::cos(increment), 1e-9));
        expect(approx(restarted.front().imag(), std::sin(increment), 1e-9));
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
