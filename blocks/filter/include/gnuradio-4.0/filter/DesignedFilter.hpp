#ifndef GNURADIO_DESIGNED_FILTER_HPP
#define GNURADIO_DESIGNED_FILTER_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <concepts>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>

#include <gnuradio-4.0/filter/FirFilter.hpp>
#include <gnuradio-4.0/filter/NamespaceCompatibility.hpp>

namespace gr::blocks::filter {

GR_REGISTER_BLOCK(gr::blocks::filter::DesignedFilter, ([T], [U]), [ float, std::complex<float> ], [ float, std::complex<float> ])

template<typename TSample, typename TTap = float>
requires((std::same_as<TSample, float> || std::same_as<TSample, std::complex<float>>) && (std::same_as<TTap, float> || std::same_as<TTap, std::complex<float>>))
struct DesignedFilter : Block<DesignedFilter<TSample, TTap>, Resampling<1UZ, 1UZ, false>>, detail::FirFilterCore<DesignedFilter<TSample, TTap>, TSample, TTap> {
    using TParent     = Block<DesignedFilter<TSample, TTap>, Resampling<1UZ, 1UZ, false>>;
    using TCore       = detail::FirFilterCore<DesignedFilter<TSample, TTap>, TSample, TTap>;
    using TOut        = typename TCore::TOut;
    using Description = Doc<R""(
@brief A FIR filter whose taps are designed at runtime from a stated specification.

Parameters in, taps out: the block owns a design specification as settings - profile, -6 dB cutoffs in hertz, a
transition width or an explicit odd length, a window or a Kaiser attenuation, gain, decimation - designs its taps in
`settingsChanged` through `gr::filter::fir::design`, and filters with the same machinery the raw-taps FirFilter runs. Every
design-affecting setting is live: a redesign keeps the input/output alignment exactly, and a `decimation` change moves
the phase origin. `taps` here is an observable count, never a setting - a chain supplies a passband, not a vector.

The tap type states which family the block designs: real taps carry `lowpass`, `highpass`, `bandpass`, `bandstop`,
`root_raised_cosine` and `hilbert`; complex taps carry `complex_bandpass` and `complex_bandstop`. A profile outside the
tap type's family is refused by name. `root_raised_cosine` requires an explicit `taps` length and a positive
`symbol_rate` - no transition-width estimate exists for it.

`hilbert` is the analytic-signal branch: antisymmetric type III taps, structurally zero at DC and at Nyquist, rotating
every positive frequency by -90 degrees. An analytic signal is this output paired with the input delayed by the group
delay `(taps - 1) / 2`, which is an integer because the length is odd. It requires an explicit odd `taps` and reads no
band edge - a Hilbert transformer covers the whole open band and how much of it is usable follows from the length
alone, which is what `gr::filter::fir::design::scanHilbert` measures. `cutoff` is therefore not read under this profile.

`BasicFilter` is the module's other design-from-parameters filter and the one to reach for where an IIR response
or a named window over a fixed length is enough; `FirFilter` is the one to reach for where the taps are already in
hand. This block is the FIR-only third: an arbitrary length, complex taps, and the Hilbert and root-raised-cosine
profiles neither of the others offers.

`sample_rate` is the reserved stream key under the framework's standard rule: explicitly configuring it removes it
from the auto-update set, so a stated rate wins over stream tags and a redesign to a new rate arrives by restaging the
setting; left unconfigured, a stream tag updates it and redesigns. A passing rate tag is forwarded divided by the
decimation, so downstream reads the rate of the stream this block hands it. A refused settings change leaves the
previous design running and is not absorbed: the member keeps the value that was refused, so re-stating it moves
nothing and is not a second request, but the design stays stale and the next start refuses on it.
)"">;

    PortIn<TSample> in;
    PortOut<TOut>   out;

    Annotated<std::string, "profile", Visible, Doc<"lowpass | highpass | bandpass | bandstop | hilbert | complex_bandpass | complex_bandstop | root_raised_cosine">>       profile          = "lowpass";
    Annotated<float, "sample_rate", Unit<"Hz">, Visible, Doc<"the design's rate reference; the reserved stream key, so a passing tag redesigns to it">>                    sample_rate      = 1.0f;
    Annotated<double, "cutoff", Unit<"Hz">, Visible, Doc<"-6 dB point; the lower edge of a band form">>                                                                    cutoff           = 0.0;
    Annotated<double, "high_cutoff", Unit<"Hz">, Visible, Doc<"-6 dB point, band forms only">>                                                                             high_cutoff      = 0.0;
    Annotated<double, "transition_width", Unit<"Hz">, Visible, Doc<"applied to every edge; sets the length unless taps is stated">>                                        transition_width = 0.0;
    Annotated<double, "gain", Doc<"the value of the design's own normalization reference">>                                                                                gain             = 1.0;
    Annotated<double, "attenuation_db", Unit<"dB">, Doc<"Kaiser only: what the window is shaped for">>                                                                     attenuation_db   = 60.0;
    Annotated<gr::algorithm::window::Type, "window", Doc<"the window the taps are shaped with; Kaiser takes its beta from attenuation_db unless window_param states one">> window           = gr::algorithm::window::Type::Kaiser;
    Annotated<double, "window_param", Doc<"NaN: Kaiser derives beta from attenuation_db, other windows take their own default">>                                           window_param     = std::numeric_limits<double>::quiet_NaN();
    Annotated<gr::Size_t, "taps", Doc<"0: length estimated from transition_width; otherwise the odd length used">>                                                         taps             = 0U;
    Annotated<double, "symbol_rate", Unit<"Hz">, Doc<"root_raised_cosine only">>                                                                                           symbol_rate      = 0.0;
    Annotated<double, "alpha", Doc<"root_raised_cosine only: excess bandwidth">>                                                                                           alpha            = 0.35;
    Annotated<gr::Size_t, "decimation", Doc<"M: input samples consumed per output; at least one">, Visible>                                                                decimation       = 1U;
    Annotated<gr::Size_t, "designed_taps", Doc<"observable: the length the last design produced">>                                                                         designed_taps    = 0U;

    GR_MAKE_REFLECTABLE(DesignedFilter, in, out, profile, sample_rate, cutoff, high_cutoff, transition_width, gain, attenuation_db, window, window_param, taps, symbol_rate, alpha, decimation, designed_taps);

    std::vector<TTap> _designed;
    bool              _designStale = true; /// the members have moved since `_designed` was built, a refused design included

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const bool live = this->coreLive();
        // Everything except the decimation is design-affecting; redesigning on an unrelated
        // key is harmless (the same inputs produce the same taps) but the alignment replay is
        // not free, so a named change is what triggers it. A design that threw built nothing and left the members
        // holding the value it refused, so the flag stays set: the design is stale until one succeeds, and the
        // next start refuses on it rather than running taps the members do not describe.
        static constexpr std::array<std::string_view, 12UZ> kDesignKeys{"profile", "sample_rate", "cutoff", "high_cutoff", "transition_width", "gain", "attenuation_db", "window", "window_param", "taps", "symbol_rate", "alpha"};
        _designStale = _designStale || !live || std::ranges::any_of(kDesignKeys, [&newSettings](std::string_view key) { return newSettings.contains(gr::property_map::key_type{key}); });

        const bool designChanged = _designStale;
        const bool rateChanged   = newSettings.contains("decimation");
        if (designChanged) {
            design();
            _designStale = false;
        }
        if (designChanged || rateChanged) {
            this->coreRebuild(std::span<const TTap>(_designed), decimation, live);
        }
        if (live && rateChanged) {
            this->coreMarkReorigin();
        }
    }

    void start() {
        if (_designStale) {
            design();
            _designStale = false;
        }
        this->coreStart(std::span<const TTap>(_designed), decimation);
    }

    /// @brief The delay of the symmetric design, in input samples.
    [[nodiscard]] double groupDelaySamples() const noexcept { return 0.5 * static_cast<double>(_designed.size() - 1UZ); }

private:
    void design() {
        if (!(sample_rate > 0.f) || !std::isfinite(sample_rate)) {
            throw gr::exception(std::format("sample_rate must be positive and finite, got {}", sample_rate.value));
        }
        gr::filter::fir::design::FilterSpec spec{
            .sampleRate      = static_cast<double>(sample_rate),
            .cutoff          = cutoff,
            .highCutoff      = high_cutoff,
            .transitionWidth = transition_width,
            .gain            = gain,
            .attenuationDb   = attenuation_db,
            .window          = window,
            .windowParam     = window_param,
            .taps            = static_cast<std::size_t>(taps.value),
        };

        const std::string_view which = profile.value;
        std::vector<TTap>      designed;
        if constexpr (std::same_as<TTap, float>) {
            if (which == "lowpass") {
                designed = gr::filter::fir::design::designLowpass(spec);
            } else if (which == "highpass") {
                designed = gr::filter::fir::design::designHighpass(spec);
            } else if (which == "bandpass") {
                designed = gr::filter::fir::design::designBandpass(spec);
            } else if (which == "bandstop") {
                designed = gr::filter::fir::design::designBandstop(spec);
            } else if (which == "root_raised_cosine") {
                if (taps.value == 0U) {
                    throw gr::exception("root_raised_cosine has no transition-width length estimate; state taps");
                }
                if (!(symbol_rate > 0.0)) {
                    throw gr::exception(std::format("root_raised_cosine needs a positive symbol_rate, got {}", symbol_rate.value));
                }
                designed = gr::filter::fir::design::designRootRaisedCosine(static_cast<std::size_t>(taps.value), static_cast<double>(sample_rate), symbol_rate, alpha, gain);
            } else if (which == "hilbert") {
                if (taps.value == 0U) {
                    throw gr::exception("hilbert has no transition-width length estimate: the usable band is a trade against the length, so state taps");
                }
                if (taps.value % 2U == 0U) {
                    throw gr::exception(std::format("hilbert is a type III antisymmetric design and needs an odd length, so that its group delay is a whole sample; got taps {}", taps.value));
                }
                designed = gr::filter::fir::design::hilbert(static_cast<std::size_t>(taps.value), gr::filter::fir::design::windowOf(spec), gain);
            } else if (which == "complex_bandpass" || which == "complex_bandstop") {
                throw gr::exception(std::format("profile '{}' needs the complex tap type", which));
            } else {
                throw gr::exception(std::format("unknown profile '{}'", which));
            }
        } else {
            if (which == "complex_bandpass") {
                designed = gr::filter::fir::design::designComplexBandpass(spec);
            } else if (which == "complex_bandstop") {
                designed = gr::filter::fir::design::designComplexBandstop(spec);
            } else if (which == "lowpass" || which == "highpass" || which == "bandpass" || which == "bandstop" || which == "root_raised_cosine" || which == "hilbert") {
                throw gr::exception(std::format("profile '{}' needs the real tap type", which));
            } else {
                throw gr::exception(std::format("unknown profile '{}'", which));
            }
        }

        _designed     = std::move(designed);
        designed_taps = static_cast<gr::Size_t>(_designed.size());
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_DESIGNED_FILTER_HPP
