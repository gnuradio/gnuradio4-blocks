#include <boost/ut.hpp>

#include <gnuradio-4.0/sdr/Saturate.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <print>
#include <random>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

// The saturation is measured on the kind of stream a transmit chain hands the sink: a random QPSK sequence through a
// root raised cosine, four samples a symbol at 0.35 excess bandwidth, normalized to unit average power. A pulse-shaped
// stream at that power runs past full scale on its peaks, which is the case the saturation exists for. No device and
// no SoapySDR are involved.
namespace {

using gr::blocks::sdr::saturateToFullScale;

constexpr double      kSamplesPerSymbol = 4.0;
constexpr double      kRolloff          = 0.35;
constexpr std::size_t kSampleStride     = 4UZ;
constexpr std::size_t kTaps             = 41UZ;
constexpr std::size_t kSymbols          = 250'000UZ;
constexpr std::size_t kSamples          = kSymbols * kSampleStride; // one million

// h(t) = [sin(pi t (1-a)) + 4 a t cos(pi t (1+a))] / [pi t (1 - (4 a t)^2)], the time in symbol periods, with both
// removable singularities in closed form: h(0) = 1 - a + 4a/pi, and at abs(4 a t) = 1,
// h = (a/sqrt2) [(1 + 2/pi) sin(pi/(4a)) + (1 - 2/pi) cos(pi/(4a))].
double rootRaisedCosineAt(double t, double alpha) {
    constexpr double pi = std::numbers::pi;
    if (t == 0.0) {
        return 1.0 - alpha + 4.0 * alpha / pi;
    }
    const double x = 4.0 * alpha * t;
    if (std::abs(1.0 - x * x) < 1.0e-6) {
        const double corner = pi / (4.0 * alpha);
        return (alpha / std::numbers::sqrt2) * ((1.0 + 2.0 / pi) * std::sin(corner) + (1.0 - 2.0 / pi) * std::cos(corner));
    }
    return (std::sin(pi * t * (1.0 - alpha)) + x * std::cos(pi * t * (1.0 + alpha))) / (pi * t * (1.0 - x * x));
}

// The taps carry no normalization: the unit-power scaling below sets the stream's amplitude and any gain in the taps
// cancels there.
std::vector<double> shapingTaps() {
    const auto          middle = static_cast<double>(kTaps - 1UZ) / 2.0;
    std::vector<double> taps(kTaps);
    for (std::size_t i = 0UZ; i < kTaps; ++i) {
        taps[i] = rootRaisedCosineAt((static_cast<double>(i) - middle) / kSamplesPerSymbol, kRolloff);
    }
    return taps;
}

std::vector<std::complex<float>> shapedStream() {
    const std::vector<double> taps = shapingTaps();

    constexpr double                                corner = 1.0 / std::numbers::sqrt2;
    constexpr std::array<std::complex<double>, 4UZ> points{std::complex<double>{corner, corner}, std::complex<double>{-corner, corner}, std::complex<double>{-corner, -corner}, std::complex<double>{corner, -corner}};

    std::mt19937                       generator(20260912U);
    std::uniform_int_distribution<int> draw(0, 3);
    std::vector<std::complex<double>>  symbols(kSymbols);
    for (auto& symbol : symbols) {
        symbol = points[static_cast<std::size_t>(draw(generator))];
    }

    // one symbol every kSampleStride samples, so only the taps whose index matches n modulo the stride contribute
    std::vector<std::complex<double>> shaped(kSamples);
    for (std::size_t n = 0UZ; n < kSamples; ++n) {
        std::complex<double> accumulator{};
        for (std::size_t k = n % kSampleStride; k < kTaps && k <= n; k += kSampleStride) {
            const std::size_t symbol = (n - k) / kSampleStride;
            if (symbol < kSymbols) {
                accumulator += taps[k] * symbols[symbol];
            }
        }
        shaped[n] = accumulator;
    }

    double power = 0.0;
    for (const auto& sample : shaped) {
        power += std::norm(sample);
    }
    const double scale = 1.0 / std::sqrt(power / static_cast<double>(kSamples));

    std::vector<std::complex<float>> stream(kSamples);
    for (std::size_t n = 0UZ; n < kSamples; ++n) {
        stream[n] = std::complex<float>(static_cast<float>(shaped[n].real() * scale), static_cast<float>(shaped[n].imag() * scale));
    }
    return stream;
}

struct StreamStats {
    double      meanPower     = 0.0;
    double      peakMagnitude = 0.0;
    double      peakPart      = 0.0; // the larger of the two parts, which is what a driver converts
    std::size_t partsPast     = 0UZ; // parts past full scale
    std::size_t samplesPast   = 0UZ; // samples with at least one part past full scale
    std::size_t nSamples      = 0UZ;
    std::size_t nNaN          = 0UZ;

    [[nodiscard]] double partFraction() const { return static_cast<double>(partsPast) / (2.0 * static_cast<double>(nSamples)); }
    [[nodiscard]] double sampleFraction() const { return static_cast<double>(samplesPast) / static_cast<double>(nSamples); }
};

StreamStats measure(const std::vector<std::complex<float>>& stream) {
    StreamStats stats;
    stats.nSamples = stream.size();
    for (const auto& sample : stream) {
        const double re = std::abs(static_cast<double>(sample.real()));
        const double im = std::abs(static_cast<double>(sample.imag()));
        stats.meanPower += static_cast<double>(std::norm(sample));
        stats.peakMagnitude = std::max(stats.peakMagnitude, std::sqrt(re * re + im * im));
        stats.peakPart      = std::max(stats.peakPart, std::max(re, im));
        stats.partsPast += static_cast<std::size_t>(re > 1.0) + static_cast<std::size_t>(im > 1.0);
        stats.samplesPast += static_cast<std::size_t>(re > 1.0 || im > 1.0);
        stats.nNaN += static_cast<std::size_t>(std::isnan(re)) + static_cast<std::size_t>(std::isnan(im));
    }
    stats.meanPower /= static_cast<double>(stream.size());
    return stats;
}

bool identical(float a, float b) { return std::bit_cast<std::uint32_t>(a) == std::bit_cast<std::uint32_t>(b); }

// what the function promises for one part: a part past full scale becomes the bound with its sign, and every other
// part, a NaN included, comes back bit-identical
void checkPart(float before, float after, std::string_view where) {
    using namespace boost::ut;
    if (std::isnan(before) || std::abs(before) <= 1.0f) {
        expect(identical(before, after)) << std::format("{}: {} came back as {}", where, before, after);
    } else {
        expect(identical(after, std::copysign(1.0f, before))) << std::format("{}: {} came back as {}", where, before, after);
    }
}

} // namespace

const boost::ut::suite<"full-scale saturation"> saturateTests = [] {
    using namespace boost::ut;

    // the cases in order: a signed zero, an ordinary sample, exactly full scale on both signs, one ulp past it on both
    // signs, well past it on both signs, the peak part the modem demo's stream reached, both infinities, a NaN in
    // either part, and the smallest magnitudes a float carries
    "a part past full scale becomes the bound and every other part is untouched"_test = [] {
        constexpr float                        infinity   = std::numeric_limits<float>::infinity();
        const float                            notANumber = std::numeric_limits<float>::quiet_NaN();
        constexpr std::size_t                  kNaNRealAt = 7UZ;
        constexpr std::size_t                  kNaNImagAt = 8UZ;
        std::vector<std::complex<float>>       samples    = {{0.0f, -0.0f}, {0.5f, -0.25f}, {1.0f, -1.0f}, {std::nextafter(1.0f, 2.0f), std::nextafter(-1.0f, -2.0f)}, {2.5f, -7.25f}, {1.117f, -1.117f}, {infinity, -infinity}, {notANumber, 0.75f}, {-0.75f, notANumber}, {std::numeric_limits<float>::denorm_min(), -1.0e-30f}};
        const std::vector<std::complex<float>> before     = samples;

        saturateToFullScale(samples);

        for (std::size_t i = 0UZ; i < samples.size(); ++i) {
            checkPart(before[i].real(), samples[i].real(), std::format("sample {} real", i));
            checkPart(before[i].imag(), samples[i].imag(), std::format("sample {} imaginary", i));
            expect(std::isnan(samples[i].real()) || std::abs(samples[i].real()) <= 1.0f) << std::format("sample {} real is {}", i, samples[i].real());
            expect(std::isnan(samples[i].imag()) || std::abs(samples[i].imag()) <= 1.0f) << std::format("sample {} imaginary is {}", i, samples[i].imag());
        }
        expect(std::isnan(samples[kNaNRealAt].real()) && std::isnan(samples[kNaNImagAt].imag())) << "a NaN part was replaced";

        const std::vector<std::complex<float>> once = samples;
        saturateToFullScale(samples);
        for (std::size_t i = 0UZ; i < samples.size(); ++i) {
            expect(identical(once[i].real(), samples[i].real()) && identical(once[i].imag(), samples[i].imag())) << std::format("sample {} moved on the second pass", i);
        }

        saturateToFullScale(std::span<std::complex<float>>{});
    };

    "a shaped unit-power stream runs past full scale and comes back inside it"_test = [] {
        const std::vector<std::complex<float>> stream = shapedStream();
        const StreamStats                      before = measure(stream);

        std::vector<std::complex<float>> saturated = stream;
        const auto                       started   = std::chrono::steady_clock::now();
        saturateToFullScale(saturated);
        const double elapsedMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();

        const StreamStats after = measure(saturated);
        std::println("{} samples at mean power {:.4f}: peak magnitude {:.4f}, peak part {:.4f}, {:.2f} % of parts and {:.2f} % of samples past full scale; saturated in {:.2f} ms ({:.2f} ns a sample), peak part after {:.6f}", stream.size(), before.meanPower, before.peakMagnitude, before.peakPart, before.partFraction() * 100.0, before.sampleFraction() * 100.0, elapsedMs, elapsedMs * 1.0e6 / static_cast<double>(stream.size()), after.peakPart);

        expect(lt(std::abs(before.meanPower - 1.0), 1.0e-6)) << std::format("mean power {:.6f}", before.meanPower);
        expect(gt(before.peakPart, 1.0)) << std::format("peak part {:.4f} never reaches full scale", before.peakPart);
        expect(gt(before.sampleFraction(), 0.05) && lt(before.sampleFraction(), 0.30)) << std::format("{:.2f} % of samples past full scale", before.sampleFraction() * 100.0);
        expect(gt(before.partFraction(), 0.02) && lt(before.partFraction(), 0.25)) << std::format("{:.2f} % of parts past full scale", before.partFraction() * 100.0);

        expect(le(after.peakPart, 1.0)) << std::format("peak part after saturation {:.6f}", after.peakPart);
        expect(eq(after.partsPast, 0UZ)) << std::format("{} parts still past full scale", after.partsPast);
        expect(eq(after.nNaN, 0UZ)) << std::format("{} NaN parts appeared", after.nNaN);

        std::size_t changed = 0UZ;
        std::size_t wrong   = 0UZ;
        for (std::size_t n = 0UZ; n < stream.size(); ++n) {
            for (const auto& part : {std::pair{stream[n].real(), saturated[n].real()}, std::pair{stream[n].imag(), saturated[n].imag()}}) {
                if (std::abs(part.first) <= 1.0f) {
                    wrong += static_cast<std::size_t>(!identical(part.first, part.second));
                } else {
                    changed += 1UZ;
                    wrong += static_cast<std::size_t>(!identical(part.second, std::copysign(1.0f, part.first)));
                }
            }
        }
        expect(eq(wrong, 0UZ)) << std::format("{} parts came back wrong", wrong);
        expect(eq(changed, before.partsPast)) << std::format("{} parts changed against {} past full scale", changed, before.partsPast);
        expect(lt(elapsedMs, 200.0)) << std::format("a million samples took {:.2f} ms", elapsedMs);
    };
};

int main() { /* not needed for UT */ }
