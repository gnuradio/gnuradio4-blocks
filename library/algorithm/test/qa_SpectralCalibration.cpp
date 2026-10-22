#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/algorithm/fourier/SpectralCalibration.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/algorithm/fourier/window.hpp>

namespace {

using gr::algorithm::fft::accumulatePowerSpectrum;
using gr::algorithm::fft::enbwBins;
using gr::algorithm::fft::integratePowerDensity;
using gr::algorithm::fft::spectralScale;
using CF = std::complex<float>;

constexpr float kSampleRate = 48000.f;

/// The windows the calibration is checked against, one per window family.
constexpr std::array<std::pair<std::string_view, gr::algorithm::window::Type>, 5UZ> kWindows{{
    {"Rectangular", gr::algorithm::window::Type::Rectangular},
    {"Hann", gr::algorithm::window::Type::Hann},
    {"Hamming", gr::algorithm::window::Type::Hamming},
    {"Blackman", gr::algorithm::window::Type::Blackman},
    {"BlackmanHarris", gr::algorithm::window::Type::BlackmanHarris},
}};

/// @brief The density of one windowed transform of `input`, computed with the kernel's scale and fold.
[[nodiscard]] std::vector<float> densityOf(std::span<const CF> input, std::span<const float> window, bool oneSided = false) {
    const std::size_t n = input.size();
    std::vector<CF>   windowed(n);
    for (std::size_t k = 0UZ; k < n; ++k) {
        windowed[k] = input[k] * window[k];
    }
    gr::algorithm::FFT<CF> transform;
    std::vector<CF>        spectrum(n);
    transform.compute(windowed, std::span<CF>(spectrum));

    const auto         scale = spectralScale(window, kSampleRate);
    std::vector<float> density(oneSided ? n / 2UZ + 1UZ : n, 0.f);
    accumulatePowerSpectrum(std::span<const CF>(spectrum), scale.density, oneSided, std::span<float>(density));
    return density;
}

[[nodiscard]] std::vector<CF> toneAtBin(std::size_t n, double bin, float amplitude = 1.f) {
    std::vector<CF>  data(n);
    constexpr double twoPi = 2. * std::numbers::pi;
    for (std::size_t k = 0UZ; k < n; ++k) {
        const double phase = twoPi * bin * static_cast<double>(k) / static_cast<double>(n);
        data[k]            = CF(amplitude * static_cast<float>(std::cos(phase)), amplitude * static_cast<float>(std::sin(phase)));
    }
    return data;
}

} // namespace

const boost::ut::suite<"SpectralCalibration"> calibrationTests = [] {
    using namespace boost::ut;

    "a rectangle has one bin of noise bandwidth, and every other window has more"_test = [] {
        const auto rect = gr::algorithm::window::create(gr::algorithm::window::Type::Rectangular, 1024UZ);
        expect(approx(enbwBins(std::span<const float>(rect)), 1.f, 1e-5f)) << "the rectangle's noise bandwidth is one bin";

        // the published figures: Hann 1.50, Hamming 1.36, Blackman 1.73, Blackman-Harris 2.00 bins
        const auto expected = std::vector<std::pair<gr::algorithm::window::Type, float>>{
            {gr::algorithm::window::Type::Hann, 1.50f},
            {gr::algorithm::window::Type::Hamming, 1.36f},
            {gr::algorithm::window::Type::Blackman, 1.73f},
            {gr::algorithm::window::Type::BlackmanHarris, 2.00f},
        };
        for (const auto& [type, figure] : expected) {
            const auto  window = gr::algorithm::window::create(type, 1024UZ);
            const float value  = enbwBins(std::span<const float>(window));
            expect(approx(value, figure, 0.01f)) << "the measured noise bandwidth must be the published one";
        }
    };

    "a full-scale tone's density integrates to 0 dBFS for every window"_test = [] {
        constexpr std::size_t n = 1024UZ;
        for (const auto& [name, type] : kWindows) {
            const auto  window   = gr::algorithm::window::create(type, n);
            const auto  density  = densityOf(std::span<const CF>(toneAtBin(n, 128.)), std::span<const float>(window));
            const auto  scale    = spectralScale(std::span<const float>(window), kSampleRate);
            const float integral = integratePowerDensity(std::span<const float>(density), scale.binWidthHz);
            const float db       = 10.f * std::log10(integral);
            expect(std::abs(db) < 0.05f) << name << " reads " << db << " dBFS for a full-scale tone";
        }
    };

    "a full-scale real sine integrates to half the power of a full-scale complex tone"_test = [] {
        // The density is referred to unit power. A real sine of amplitude 1 has the mean power 1/2.
        constexpr std::size_t n = 1024UZ;
        std::vector<CF>       real(n);
        for (std::size_t k = 0UZ; k < n; ++k) {
            real[k] = CF(static_cast<float>(std::cos(2. * std::numbers::pi * 128. * static_cast<double>(k) / static_cast<double>(n))), 0.f);
        }
        for (const auto& [name, type] : kWindows) {
            const auto  window   = gr::algorithm::window::create(type, n);
            const auto  scale    = spectralScale(std::span<const float>(window), kSampleRate);
            const auto  density  = densityOf(std::span<const CF>(real), std::span<const float>(window), true);
            const float integral = integratePowerDensity(std::span<const float>(density), scale.binWidthHz);
            expect(std::abs(integral - 0.5f) < 1e-4f) << name << " integrates a full-scale real sine to " << integral;
        }
    };

    "the tone correction recovers a tone's power from its own peak bin"_test = [] {
        constexpr std::size_t n = 1024UZ;
        for (const auto& [name, type] : kWindows) {
            const auto window  = gr::algorithm::window::create(type, n);
            const auto density = densityOf(std::span<const CF>(toneAtBin(n, 128.)), std::span<const float>(window));
            const auto scale   = spectralScale(std::span<const float>(window), kSampleRate);

            const float peak      = *std::ranges::max_element(density);
            const float tonePower = peak * scale.enbwHz();
            const float db        = 10.f * std::log10(tonePower);
            expect(std::abs(db) < 0.01f) << name << " reads " << db << " dBFS from its peak bin";
        }
    };

    "seeded noise of known power integrates to that power"_test = [] {
        constexpr std::size_t n         = 1024UZ;
        constexpr std::size_t nAverages = 64UZ;
        // Both parts are uniform on [-0.5, 0.5), and E[|x|^2] = 2 * (1/12). The test checks the generator's measured
        // power against this value.
        constexpr float kPower = 1.f / 6.f;

        const auto         window = gr::algorithm::window::create(gr::algorithm::window::Type::Hann, n);
        const auto         scale  = spectralScale(std::span<const float>(window), kSampleRate);
        std::vector<float> density(n, 0.f);

        std::uint64_t state = 0x9e3779b97f4a7c15ULL;
        const auto    next  = [&state] {
            state ^= state << 13U;
            state ^= state >> 7U;
            state ^= state << 17U;
            return static_cast<float>(static_cast<double>(state % 2048ULL) / 2048. - 0.5); // uniform on [-0.5, 0.5)
        };

        gr::algorithm::FFT<CF> transform;
        std::vector<CF>        windowed(n);
        std::vector<CF>        spectrum(n);
        double                 measuredPower = 0.;
        for (std::size_t segment = 0UZ; segment < nAverages; ++segment) {
            for (std::size_t k = 0UZ; k < n; ++k) {
                const CF sample = CF(next(), next());
                measuredPower += static_cast<double>(sample.real() * sample.real() + sample.imag() * sample.imag());
                windowed[k] = sample * window[k];
            }
            transform.compute(windowed, std::span<CF>(spectrum));
            accumulatePowerSpectrum(std::span<const CF>(spectrum), scale.density / static_cast<float>(nAverages), false, std::span<float>(density));
        }
        measuredPower /= static_cast<double>(n * nAverages);

        const float integral = integratePowerDensity(std::span<const float>(density), scale.binWidthHz);
        const float ratioDb  = 10.f * std::log10(integral / static_cast<float>(measuredPower));
        expect(std::abs(ratioDb) < 0.1f) << "the density integral is the sequence's own mean power, measured " << ratioDb << " dB apart";
        expect(std::abs(static_cast<float>(measuredPower) - kPower) < 0.005f) << "the generator's power is the expected 1/6";
    };

    "a one-sided density integrates to the same total as the two-sided one"_test = [] {
        constexpr std::size_t n      = 1024UZ;
        const auto            window = gr::algorithm::window::create(gr::algorithm::window::Type::Hann, n);
        const auto            scale  = spectralScale(std::span<const float>(window), kSampleRate);

        // A real input as complex samples with zero quadrature. Its spectrum is conjugate-symmetric.
        std::vector<CF> real(n);
        for (std::size_t k = 0UZ; k < n; ++k) {
            real[k] = CF(std::cos(2.f * std::numbers::pi_v<float> * 128.f * static_cast<float>(k) / static_cast<float>(n)), 0.f);
        }

        const auto twoSided = densityOf(std::span<const CF>(real), std::span<const float>(window), false);
        const auto oneSided = densityOf(std::span<const CF>(real), std::span<const float>(window), true);
        expect(eq(oneSided.size(), n / 2UZ + 1UZ)) << "a one-sided record keeps DC through Nyquist";

        const float whole = integratePowerDensity(std::span<const float>(twoSided), scale.binWidthHz);
        const float half  = integratePowerDensity(std::span<const float>(oneSided), scale.binWidthHz);
        expect(std::abs(10.f * std::log10(half / whole)) < 0.01f) << "the fold keeps the total power";
    };

    "an odd length has no Nyquist bin, and its one-sided density keeps the total"_test = [] {
        // At N 5 a tone sits in bin 2, the last bin kept. At N 1023 seeded noise reaches every bin, bin 511 included.
        for (const std::size_t n : {5UZ, 1023UZ}) {
            const std::vector<float> window(n, 1.f);
            const auto               scale = spectralScale(std::span<const float>(window), kSampleRate);

            std::vector<CF> real(n);
            std::uint64_t   state = 0x9e3779b97f4a7c15ULL;
            for (std::size_t k = 0UZ; k < n; ++k) {
                if (n == 5UZ) {
                    real[k] = CF(static_cast<float>(std::cos(2. * std::numbers::pi * 2. * static_cast<double>(k) / 5.)), 0.f);
                } else {
                    state ^= state << 13U;
                    state ^= state >> 7U;
                    state ^= state << 17U;
                    real[k] = CF(static_cast<float>(static_cast<double>(state % 2048ULL) / 2048. - 0.5), 0.f);
                }
            }
            const auto twoSided = densityOf(std::span<const CF>(real), std::span<const float>(window), false);
            const auto oneSided = densityOf(std::span<const CF>(real), std::span<const float>(window), true);
            expect(eq(oneSided.size(), n / 2UZ + 1UZ));

            const float whole = integratePowerDensity(std::span<const float>(twoSided), scale.binWidthHz);
            const float half  = integratePowerDensity(std::span<const float>(oneSided), scale.binWidthHz);
            expect(std::abs(half / whole - 1.f) < 1e-4f) << "at N " << n << " the one-sided total is " << half << " against " << whole;
        }
    };

    "an empty spectrum leaves the output unchanged, one-sided and two-sided"_test = [] {
        // The empty view sits at the end of a filled buffer. A read of its first element reads past the buffer.
        const std::vector<CF>     buffer(4UZ, CF(1.f, 1.f));
        const std::span<const CF> empty(buffer.data() + buffer.size(), 0UZ);
        const std::vector<float>  before = {1.f, 2.f, 3.f, 4.f};

        std::vector<float> oneSided = before;
        std::vector<float> twoSided = before;
        accumulatePowerSpectrum(empty, 1.f, true, std::span<float>(oneSided));
        accumulatePowerSpectrum(empty, 1.f, false, std::span<float>(twoSided));
        expect(oneSided == before) << "a one-sided accumulation of an empty spectrum changed the output";
        expect(twoSided == before) << "a two-sided accumulation of an empty spectrum changed the output";
    };

    "a degenerate window gives a NaN scale and no division by zero"_test = [] {
        const std::vector<float> zeros(16UZ, 0.f);
        const auto               scale = spectralScale(std::span<const float>(zeros), kSampleRate);
        expect(std::isnan(scale.density));
        expect(std::isnan(scale.enbwBins));

        const auto rect   = gr::algorithm::window::create(gr::algorithm::window::Type::Rectangular, 16UZ);
        const auto noRate = spectralScale(std::span<const float>(rect), 0.f);
        expect(std::isnan(noRate.density)) << "a zero sample rate gives a NaN density";
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
