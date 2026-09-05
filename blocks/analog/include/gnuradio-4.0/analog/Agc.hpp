#ifndef GNURADIO_AGC_HPP
#define GNURADIO_AGC_HPP

#include <algorithm>
#include <bit>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <format>
#include <limits>
#include <span>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::analog {

namespace detail {

// std::clamp branches on its unordered comparisons; the hot loop needs the branch-free minss/maxss form
template<std::floating_point TValue>
[[nodiscard]] constexpr TValue clampOrdered(TValue value, TValue low, TValue high) noexcept {
    const TValue lifted = value < low ? low : value;
    return lifted > high ? high : lifted;
}

// log2 of a positive normal float: exponent field plus the atanh series for the mantissa, centered on
// [1/sqrt(2), sqrt(2)) so four terms hold the error below 5e-8, with no libm call and no table
[[nodiscard]] inline float fastLog2(float value) noexcept {
    const auto    bits     = std::bit_cast<std::uint32_t>(value);
    std::int32_t  exponent = static_cast<std::int32_t>(bits >> 23U) - 127;
    std::uint32_t mantissa = (bits & 0x007FFFFFU) | 0x3F800000U;
    if (mantissa >= 0x3FB504F3U) {
        mantissa -= 0x00800000U;
        ++exponent;
    }

    const float centered = std::bit_cast<float>(mantissa);
    const float z        = (centered - 1.0f) / (centered + 1.0f);
    const float z2       = z * z;
    return static_cast<float>(exponent) + z * (2.885390082f + z2 * (0.961796694f + z2 * (0.577078016f + z2 * 0.412198583f)));
}

// 2^value: the integer part is assembled straight into the exponent field, the fraction is a degree-5
// polynomial on [-0.5, 0.5]; evaluated in float the pair stays under 2e-6 relative
[[nodiscard]] inline float fastExp2(float value) noexcept {
    const float        clamped = clampOrdered(value, -126.0f, 126.0f);
    const std::int32_t whole   = static_cast<std::int32_t>(clamped + (clamped >= 0.0f ? 0.5f : -0.5f));
    const float        f       = clamped - static_cast<float>(whole);
    const float        poly    = 1.000000052f + f * (0.693147200f + f * (0.240222117f + f * (0.055503407f + f * (0.009670763f + f * 0.001339528f))));
    return std::bit_cast<float>(static_cast<std::uint32_t>(whole + 127) << 23U) * poly;
}

inline constexpr double kLog2PerDecibel        = 0.16609640474436813; // log2(10) / 20
inline constexpr double kDecibelPerHalfLog2    = 3.010299956639812;   // 10 / log2(10), i.e. dB from log2 of a squared magnitude
inline constexpr float  kMagnitudeFloorSquared = 1e-24f;              // squared form of the 1e-12 (-240 dB) level floor

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::analog::Agc, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct Agc : Block<Agc<T>, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief Holds the output level of a stream near a target by adapting a gain, with attack and decay in seconds.

The loop runs in the logarithmic domain, so its settling time is independent of both the signal level and the sample
rate. The gain in force for a sample is the one this sample's update produced. `update_decimation` runs the loop once
per K samples against a stream-absolute phase, so the output does not depend on how the scheduler chunked the stream.

The block is 1:1, so every input tag key passes through at its own offset, `sample_rate` carrying this block's value.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"stream rate; also updated by a sample_rate tag">>          sample_rate       = 48000.f;
    Annotated<bool, "enabled", Doc<"false freezes the gain and applies it as a manual gain">>                   enabled           = true;
    Annotated<double, "reference_db", Unit<"dB">, Doc<"target level of the output magnitude">>                  reference_db      = -6.0;
    Annotated<double, "attack_s", Unit<"s">, Doc<"time constant used when the gain must decrease">>             attack_s          = 0.005;
    Annotated<double, "decay_s", Unit<"s">, Doc<"time constant used when the gain must increase">>              decay_s           = 0.500;
    Annotated<double, "max_gain_db", Unit<"dB">, Doc<"upper clamp on the gain">>                                max_gain_db       = 60.0;
    Annotated<double, "min_gain_db", Unit<"dB">, Doc<"lower clamp on the gain">>                                min_gain_db       = -20.0;
    Annotated<double, "gate_threshold_db", Unit<"dB">, Doc<"input levels below this freeze the gain">>          gate_threshold_db = -150.0;
    Annotated<double, "gain_db", Unit<"dB">, Doc<"gain currently applied; writable to force a starting point">> gain_db           = 0.0;
    Annotated<gr::Size_t, "update_decimation", Doc<"update the gain every K-th sample; 1 is normative">>        update_decimation = 1U;

    GR_MAKE_REFLECTABLE(Agc, in, out, sample_rate, enabled, reference_db, attack_s, decay_s, max_gain_db, min_gain_db, gate_threshold_db, gain_db, update_decimation);

    double _gainDb{};
    // The loop coefficients are a function of the members, and a batch that moves no value never calls back, so
    // each states that function of the settings declared above it.
    double      _alphaAttack = loopCoefficient(attack_s);
    double      _alphaDecay  = loopCoefficient(decay_s);
    float       _manualGain{1.f};
    float       _appliedGain{1.f};
    std::size_t _updatePhase{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (!(sample_rate > 0.f) || !std::isfinite(sample_rate)) {
            throw gr::exception(std::format("sample_rate must be positive and finite, got {}", sample_rate.value));
        }
        if (!(attack_s > 0.0) || !(decay_s > 0.0)) {
            throw gr::exception(std::format("attack_s and decay_s must be positive, got {} and {}", attack_s.value, decay_s.value));
        }
        if (max_gain_db < min_gain_db) {
            throw gr::exception(std::format("max_gain_db ({}) must not be below min_gain_db ({})", max_gain_db.value, min_gain_db.value));
        }
        if (update_decimation < 1U) {
            throw gr::exception("update_decimation must be >= 1");
        }

        _alphaAttack = loopCoefficient(attack_s);
        _alphaDecay  = loopCoefficient(decay_s);

        if (newSettings.contains("gain_db")) {
            _gainDb = gain_db;
        }
        _gainDb     = std::clamp(_gainDb, min_gain_db.value, max_gain_db.value);
        gain_db     = _gainDb;
        _manualGain = static_cast<float>(std::pow(10.0, _gainDb / 20.0));
    }

    void reset() {
        _gainDb      = gain_db;
        _updatePhase = 0UZ;
    }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        const std::size_t nSamples = std::min(input.size(), output.size());

        if (!enabled) {
            const float manualGain = _manualGain;
            for (std::size_t i = 0UZ; i < nSamples; ++i) {
                output[i] = input[i] * manualGain;
            }
            _appliedGain = manualGain;
            return work::Status::OK;
        }

        const double reference = reference_db;
        const double gate      = gate_threshold_db;
        const double minGain   = min_gain_db;
        const double maxGain   = max_gain_db;
        const double attack    = _alphaAttack;
        const double decay     = _alphaDecay;

        const auto stepGain = [=](double gain, float magnitudeSquaredValue) noexcept {
            const double level = detail::kDecibelPerHalfLog2 * static_cast<double>(detail::fastLog2(std::max(magnitudeSquaredValue, detail::kMagnitudeFloorSquared)));
            const double error = level + gain - reference;
            const double alpha = level < gate ? 0.0 : (error > 0.0 ? attack : decay);
            return detail::clampOrdered(gain - alpha * error, minGain, maxGain);
        };

        double gainDb = _gainDb;
        if (update_decimation == 1U) {
            for (std::size_t i = 0UZ; i < nSamples; ++i) {
                const T sample = input[i];
                gainDb         = stepGain(gainDb, magnitudeSquared(sample));
                output[i]      = sample * detail::fastExp2(static_cast<float>(gainDb * detail::kLog2PerDecibel));
            }
        } else {
            const std::size_t stride = static_cast<std::size_t>(update_decimation.value);
            for (std::size_t i = 0UZ; i < nSamples;) {
                if (_updatePhase == 0UZ) {
                    gainDb       = stepGain(gainDb, magnitudeSquared(input[i]));
                    _appliedGain = detail::fastExp2(static_cast<float>(gainDb * detail::kLog2PerDecibel));
                }
                const std::size_t count = std::min(stride - _updatePhase, nSamples - i);
                for (std::size_t k = 0UZ; k < count; ++k) {
                    output[i + k] = input[i + k] * _appliedGain;
                }
                i += count;
                _updatePhase = (_updatePhase + count) % stride;
            }
        }

        _gainDb = gainDb;
        gain_db = _gainDb;
        return work::Status::OK;
    }

    [[nodiscard]] static constexpr float magnitudeSquared(T sample) noexcept {
        if constexpr (std::same_as<T, float>) {
            return sample * sample;
        } else {
            return sample.real() * sample.real() + sample.imag() * sample.imag();
        }
    }

    [[nodiscard]] double loopCoefficient(double tau) const noexcept {
        const double samples = tau * static_cast<double>(sample_rate);
        return samples < 1.0 ? 1.0 : std::clamp(1.0 - std::exp(-1.0 / samples), std::numeric_limits<double>::denorm_min(), 1.0);
    }
};

} // namespace gr::blocks::analog

#endif // GNURADIO_AGC_HPP
