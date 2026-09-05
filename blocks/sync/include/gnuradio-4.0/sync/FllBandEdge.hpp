#ifndef GNURADIO_SYNC_FLL_BAND_EDGE_HPP
#define GNURADIO_SYNC_FLL_BAND_EDGE_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <numbers>
#include <tuple>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/sync/BandEdgeFilter.hpp>
#include <gnuradio-4.0/algorithm/sync/ControlLoop.hpp>

#include <gnuradio-4.0/sync/LoopCommon.hpp>

namespace gr::blocks::sync {

GR_REGISTER_BLOCK(gr::blocks::sync::FllBandEdge)

struct FllBandEdge : Block<FllBandEdge, NoTagPropagation> {
    using Description = Doc<R""(
@brief A band-edge frequency-locked loop: acquires a carrier offset an order of magnitude wider than a Costas loop can,
using no carrier, no data decisions and no symbol timing.

Two filters sitting on the two band edges of a signal shaped by a root-raised cosine of excess bandwidth `rolloff`
measure its frequency offset as `|yu|^2 - |yl|^2`, positive when the signal has drifted up. Its pull-in range is
therefore roughly the signal bandwidth rather than the loop bandwidth, and it leaves a phase error for a Costas loop
downstream to clean up. An AGC is a precondition: the discriminant's gain is `sps * P` with `P` the mean input power.

The frequency output is the estimate, not the correction, so a settled loop reads `+2*pi*offset` and a flowgraph
ported from a correction-signed FLL must negate it. The loop is first order and so has no damping parameter. At 45
taps this block costs about 2.4 times a Costas loop per sample, so a receiver that can disable it after acquisition
should.
)"">;

    PortIn<std::complex<float>>  in;
    PortOut<std::complex<float>> out;
    PortOut<float, Optional>     freq;
    PortOut<float, Optional>     phase;
    PortOut<float, Optional>     error;

    Annotated<double, "samples_per_symbol", Doc<"sps; need not be an integer and need not divide the tap count">>            samples_per_symbol      = 4.0;
    Annotated<double, "rolloff", Doc<"excess bandwidth of the transmit shaping filter, in [0, 1]">>                          rolloff                 = 0.35;
    Annotated<gr::Size_t, "filter_length", Doc<"taps in each band-edge filter; 30 to 70 is the useful range">>               filter_length           = 45U;
    Annotated<double, "noise_bandwidth", Doc<"first-order closed-loop noise bandwidth, normalized: Bn*T">>                   noise_bandwidth         = 0.01;
    Annotated<double, "detector_gain", Doc<"Kdet in error units per cycle/sample; 0 selects sps, its value at unit power">>  detector_gain           = 0.0;
    Annotated<float, "max_frequency", Unit<"rad/sample">, Doc<"upper clamp; 0 on both bounds selects the pull-in range">>    max_frequency           = 0.f;
    Annotated<float, "min_frequency", Unit<"rad/sample">, Doc<"lower clamp; 0 on both bounds selects the pull-in range">>    min_frequency           = 0.f;
    Annotated<bool, "normalized_discriminant", Doc<"divide the power difference by the sum: no AGC needed, gain tabulated">> normalized_discriminant = false;

    GR_MAKE_REFLECTABLE(FllBandEdge, in, out, freq, phase, error, samples_per_symbol, rolloff, filter_length, noise_bandwidth, detector_gain, max_frequency, min_frequency, normalized_discriminant);

    detail::CarrierLoop                                              _loop{0.01, 1.0, 1.0, -1.f, 1.f, gr::sync::LoopOrder::First};
    gr::sync::BandEdgeDiscriminant<gr::sync::BandEdgeForm::RealTaps> _discriminant{45, 4.0, 0.35};
    std::uint64_t                                                    _ignoredTags = 0ULL;

    /// The loop gains and the band-edge taps are derived from the members, and a batch that moves no value never
    /// calls back, so a block constructed at its declared defaults is born with the design its settings name.
    explicit FllBandEdge(property_map init = {}) : Block<FllBandEdge, NoTagPropagation>(std::move(init)) { design(); }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { design(); }

    void design() {
        if (!(samples_per_symbol > 0.0) || !std::isfinite(samples_per_symbol)) {
            throw gr::exception(std::format("samples_per_symbol must be positive and finite, got {}", samples_per_symbol.value));
        }
        if (!(rolloff >= 0.0) || !(rolloff <= 1.0)) {
            throw gr::exception(std::format("rolloff must lie in [0, 1], got {}", rolloff.value));
        }
        if (filter_length < 3U) {
            throw gr::exception(std::format("filter_length must be at least 3, got {}", filter_length.value));
        }
        if (!(detector_gain >= 0.0) || !std::isfinite(detector_gain)) {
            throw gr::exception(std::format("detector_gain must be finite and not negative; 0 selects samples_per_symbol, got {}", detector_gain.value));
        }

        const float limit = static_cast<float>(gr::sync::bandEdgeFrequencyLimit(samples_per_symbol, rolloff));
        const float upper = max_frequency == 0.f && min_frequency == 0.f ? +limit : max_frequency.value;
        const float lower = max_frequency == 0.f && min_frequency == 0.f ? -limit : min_frequency.value;

        detail::applyCarrierSettings(_loop, noise_bandwidth, 1.0, lower, upper);
        // The discriminant's gain is per cycle per sample and the loop's phase is in radians; the 2*pi is the conversion.
        _loop.setDetectorGain(effectiveDetectorGain() / (2.0 * std::numbers::pi));
        _discriminant = gr::sync::BandEdgeDiscriminant<gr::sync::BandEdgeForm::RealTaps>(static_cast<int>(filter_length), samples_per_symbol, rolloff);
    }

    void reset() {
        _loop.reset();
        _discriminant.reset();
        _ignoredTags = 0ULL;
    }

    /// @brief `Kdet` in error units per cycle per sample: what `detector_gain` says, or `sps` when it says nothing.
    [[nodiscard]] double effectiveDetectorGain() const noexcept { return detector_gain > 0.0 ? detector_gain.value : samples_per_symbol.value; }

    [[nodiscard]] std::uint64_t ignoredTagPayloads() const noexcept { return _ignoredTags; }

    [[nodiscard]] const detail::CarrierLoop& loop() const noexcept { return _loop; }

    [[nodiscard]] const gr::sync::BandEdgeFilters& filters() const noexcept { return _discriminant.filters(); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& freqSpan, OutputSpanLike auto& phaseSpan, OutputSpanLike auto& errorSpan) {
        std::size_t nSamples = std::min(inSpan.size(), outSpan.size());
        nSamples             = detail::syncCount(nSamples, freqSpan);
        nSamples             = detail::syncCount(nSamples, phaseSpan);
        nSamples             = detail::syncCount(nSamples, errorSpan);

        const bool wantFreq   = freqSpan.isConnected;
        const bool wantPhase  = phaseSpan.isConnected;
        const bool wantError  = errorSpan.isConnected;
        const bool normalized = normalized_discriminant;

        const auto&       rawTags = inSpan.rawTags;
        const std::size_t nTags   = rawTags.size();
        std::size_t       cursor  = 0UZ;

        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            while (cursor < nTags && rawTags[cursor].index <= inSpan.streamIndex + i) {
                if (rawTags[cursor].index >= inSpan.streamIndex) {
                    detail::routeTag(rawTags[cursor].map, _loop, _ignoredTags, [&](const property_map& forwarded) {
                        outSpan.publishTag(forwarded, i);
                        freqSpan.publishTag(forwarded, i);
                        phaseSpan.publishTag(forwarded, i);
                        errorSpan.publishTag(forwarded, i);
                    });
                }
                ++cursor;
            }

            const float                    loopPhase = _loop.phase();
            const std::complex<float>      y         = inSpan[i] * detail::derotator(loopPhase);
            const gr::sync::BandEdgePowers powers    = _discriminant.step(y);
            const float                    e         = normalized ? gr::sync::normalizedDiscriminant(powers) : gr::sync::discriminant(powers);
            std::ignore                              = _loop.step(e);

            outSpan[i] = y;
            if (wantFreq) {
                freqSpan[i] = _loop.frequency();
            }
            if (wantPhase) {
                phaseSpan[i] = loopPhase;
            }
            if (wantError) {
                errorSpan[i] = e;
            }
        }

        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        freqSpan.publish(wantFreq ? nSamples : 0UZ);
        phaseSpan.publish(wantPhase ? nSamples : 0UZ);
        errorSpan.publish(wantError ? nSamples : 0UZ);
        return work::Status::OK;
    }
};

} // namespace gr::blocks::sync

#endif // GNURADIO_SYNC_FLL_BAND_EDGE_HPP
