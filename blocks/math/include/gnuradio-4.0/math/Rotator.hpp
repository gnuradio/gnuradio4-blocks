#ifndef GNURADIO_ROTATOR_HPP
#define GNURADIO_ROTATOR_HPP

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <complex>
#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>
#include <gnuradio-4.0/math/NamespaceCompatibility.hpp>
#include <gnuradio-4.0/meta/utils.hpp>
#include <numbers>
#include <optional>
#include <tuple>

namespace gr::blocks::math {

GR_REGISTER_BLOCK(gr::blocks::math::Rotator, [T], [std::complex<float>])

template<gr::meta::complex_like T>
struct Rotator : gr::Block<Rotator<T>> {
    using value_type  = typename T::value_type;
    using Description = Doc<R""(
@brief Rotator block shifts complex input samples by a given incremental phase every sample,
       thus effectively performing a frequency translation.

This block supports either `phase_increment` in radians per sample (x) or relative `frequency_shift` in Hz for a
given 'sample_rate' in Hz (N.B sample_rate is normalised to '1' by default).
A `frequency` key on a passing tag is retuned by the same shift, so it keeps describing the center of the stream it
is attached to.
 )"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<float, "sample rate", Doc<"signal sample rate">, Unit<"Hz">>                           sample_rate     = 1.f;
    Annotated<float, "frequency shift", Doc<"rel. frequency shift">, Unit<"Hz">>                     frequency_shift = 0.0f;
    Annotated<value_type, "phase_increment", Unit<"rad">, Doc<"how many radians to add per sample">> phase_increment{0};
    Annotated<value_type, "initial_phase", Unit<"rad">, Doc<"starting offset for each new chunk">>   initial_phase{0};

    // kept in double regardless of T: a value_type accumulator rounds the re-seed phase once per call,
    // which for 'complex<float>' is ~2.4e-7 rad and dominates every other error over a stream
    double _accumulated_phase{0.};

    GR_MAKE_REFLECTABLE(Rotator, in, out, sample_rate, frequency_shift, initial_phase, phase_increment);

    // 'frequency_shift' is the commanded quantity and survives a 'sample_rate' change;
    // setting 'phase_increment' instead commands the increment and re-derives the shift.
    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const bool haveShift     = newSettings.contains("frequency_shift");
        const bool haveIncrement = newSettings.contains("phase_increment");
        if (haveShift && haveIncrement) {
            throw gr::exception(std::format("cannot set both 'frequency_shift' and 'phase_increment' in new setting (XOR): {}", newSettings));
        }

        if (haveIncrement) {
            frequency_shift = static_cast<float>(phase_increment / (value_type(2) * std::numbers::pi_v<value_type>)) * sample_rate;
        } else if (haveShift || newSettings.contains("sample_rate")) {
            phase_increment = value_type(2) * static_cast<value_type>(std::numbers::pi_v<float> * frequency_shift / sample_rate);
        }

        if (newSettings.contains("initial_phase")) {
            _accumulated_phase = static_cast<double>(initial_phase);
        }
    }

    // rotating by 'frequency_shift' moves the spectrum, so the center frequency a passing tag announces moves by the
    // same amount in the opposite direction: what sits at the output's zero frequency arrived at -'frequency_shift'.
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t /*processedIn*/) {
        std::optional<property_map> cachedSettings;
        gr::for_each_reader_span(
            [&](auto& inSpan) {
                if (!inSpan.isSync || !inSpan.isConnected) {
                    return;
                }
                for (const auto& [relIndex, tagMapRef] : inSpan.tags(1UZ)) {
                    property_map forwarded = this->filterAndSubstituteTag(tagMapRef.get(), cachedSettings);
                    if (forwarded.empty()) {
                        continue;
                    }
                    retuneCenterFrequency(forwarded);
                    const std::size_t offset = static_cast<std::size_t>(std::max(std::ptrdiff_t(0), relIndex));
                    gr::for_each_writer_span([&forwarded, offset](auto& outSpan) { outSpan.publishTag(forwarded, offset); }, outputSpans);
                }
            },
            inputSpans);
    }

    void retuneCenterFrequency(property_map& tagMap) const noexcept {
        auto it = tagMap.find(gr::tag::FREQUENCY.shortKey());
        if (it == tagMap.end()) {
            return;
        }
        if (const double* asDouble = it->second.template get_if<double>(); asDouble != nullptr) {
            it->second = *asDouble - static_cast<double>(frequency_shift.value);
        } else if (const float* asFloat = it->second.template get_if<float>(); asFloat != nullptr) {
            it->second = *asFloat - frequency_shift.value;
        }
    }

    [[nodiscard]] constexpr work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        assert(output.size() >= input.size());

        // e^{j(phi + k*dphi)} == e^{j*phi} * (e^{j*dphi})^k: kLanes phasors advanced by (e^{j*dphi})^kLanes cover
        // kLanes consecutive samples with no dependency carried between them, so the sample loop vectorizes;
        // re-seeding from an exact phase keeps phase and magnitude drift bounded however long the stream is
        constexpr std::size_t kLanes          = 16UZ;
        constexpr std::size_t kReseedInterval = 4096UZ;
        constexpr double      twoPi           = 2. * std::numbers::pi_v<double>;

        const double                   startPhase = _accumulated_phase;
        const double                   phaseStep  = static_cast<double>(phase_increment);
        const std::complex<value_type> increment  = std::polar(value_type(1), static_cast<value_type>(phase_increment));
        const std::complex<double>     laneStep   = std::polar(1., static_cast<double>(kLanes) * phaseStep);
        const value_type               stepRe     = static_cast<value_type>(laneStep.real());
        const value_type               stepIm     = static_cast<value_type>(laneStep.imag());

        std::array<value_type, kLanes> laneRe;
        std::array<value_type, kLanes> laneIm;

        const std::size_t nSamples = input.size();
        for (std::size_t base = 0UZ; base < nSamples; base += kReseedInterval) {
            const std::complex<double> seed = std::polar(1., std::fmod(startPhase + static_cast<double>(base + 1UZ) * phaseStep, twoPi));
            laneRe[0UZ]                     = static_cast<value_type>(seed.real());
            laneIm[0UZ]                     = static_cast<value_type>(seed.imag());
            for (std::size_t w = 1UZ; w < kLanes; ++w) {
                laneRe[w] = laneRe[w - 1UZ] * increment.real() - laneIm[w - 1UZ] * increment.imag();
                laneIm[w] = laneRe[w - 1UZ] * increment.imag() + laneIm[w - 1UZ] * increment.real();
            }

            const std::size_t end = std::min(nSamples, base + kReseedInterval);
            std::size_t       i   = base;
            for (; i + kLanes <= end; i += kLanes) {
                for (std::size_t w = 0UZ; w < kLanes; ++w) {
                    const value_type re = input[i + w].real();
                    const value_type im = input[i + w].imag();
                    const value_type pr = laneRe[w];
                    const value_type pi = laneIm[w];
                    output[i + w]       = T(re * pr - im * pi, re * pi + im * pr);
                    laneRe[w]           = pr * stepRe - pi * stepIm;
                    laneIm[w]           = pr * stepIm + pi * stepRe;
                }
            }
            for (std::size_t w = 0UZ; i + w < end; ++w) {
                const value_type re = input[i + w].real();
                const value_type im = input[i + w].imag();
                output[i + w]       = T(re * laneRe[w] - im * laneIm[w], re * laneIm[w] + im * laneRe[w]);
            }
        }

        double phase = std::fmod(startPhase + static_cast<double>(nSamples) * phaseStep, twoPi);
        if (phase < 0.) {
            phase += twoPi;
        }
        _accumulated_phase = phase;

        return work::Status::OK;
    }

    // the entry point for callers that rotate one sample at a time. A block offers the framework
    // exactly one process function, so this cannot be called 'processOne' beside 'processBulk'; it
    // runs the same recurrence, and one call advances the phase as one element of a bulk call does.
    [[nodiscard]] constexpr T rotateOne(const T& inSample) noexcept {
        T outSample{};
        std::ignore = processBulk(std::span<const T>(&inSample, 1UZ), std::span<T>(&outSample, 1UZ));
        return outSample;
    }
};

} // namespace gr::blocks::math

#endif // GNURADIO_ROTATOR_HPP
