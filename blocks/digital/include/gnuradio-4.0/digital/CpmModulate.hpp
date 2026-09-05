#ifndef GNURADIO_CPM_MODULATE_HPP
#define GNURADIO_CPM_MODULATE_HPP

#include <algorithm>
#include <array>
#include <complex>
#include <concepts>
#include <cstddef>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/digital/CpmPulse.hpp>
#include <gnuradio-4.0/algorithm/signal/Phasor.hpp>

namespace gr::blocks::digital {

GR_REGISTER_BLOCK(gr::blocks::digital::CpmModulate, [T], [float])

template<std::floating_point T>
struct CpmModulate : Block<CpmModulate<T>, Resampling<1UZ, 1UZ, false>> {
    using Description = Doc<R""(
@brief Continuous-phase modulation: real symbols on the M-PAM grid in, constant-envelope complex baseband out.

One symbol of amplitude `a` turns the carrier by exactly `pi * modulation_index * a` radians, spread over the
`pulse_length` symbols the frequency pulse spans. The phase is the running sum of the per-sample increments and is
never reset, so the output is continuous across every record and every call, and the envelope is constant by
construction.

`rect` at `pulse_length` 1 is CPFSK, which on two levels is classic FSK at deviation
`modulation_index / (2 * T_sym)`. `gaussian` is GFSK, and GMSK is that with two levels and `modulation_index` 0.5.
Longer pulses spread each symbol over its neighbors, narrowing the spectrum at the price of intersymbol
interference the receiver has to live with.

Symbols arrive as floats on the odd grid `±1, ±3, …, ±(M-1)`, the grid `PamSlicer` and `LevelTracker` decode.
Turning bits into that grid is an affine map on two levels and a Gray map above it, both of which belong upstream.
)"">;

    PortIn<T>                in;
    PortOut<std::complex<T>> out;

    Annotated<double, "modulation_index", Visible, Doc<"h: a symbol of amplitude a turns the carrier by pi*h*a radians">> modulation_index   = 0.5;
    Annotated<gr::Size_t, "samples_per_symbol", Visible, Doc<"output samples produced per input symbol; at least 2">>     samples_per_symbol = 8U;
    Annotated<std::string, "pulse", Visible, Doc<"frequency pulse: rect, raised_cosine or gaussian">>                     pulse              = std::string("rect");
    Annotated<gr::Size_t, "pulse_length", Doc<"L: symbols the pulse spans; 1 is full response, at most 8">>               pulse_length       = 1U;
    Annotated<double, "bt", Doc<"bandwidth-symbol-time product; read only by the gaussian pulse, 0.3 and 0.5 are usual">> bt                 = 0.3;
    Annotated<T, "amplitude", Doc<"the envelope magnitude the output holds">>                                             amplitude          = T(1);

    GR_MAKE_REFLECTABLE(CpmModulate, in, out, modulation_index, samples_per_symbol, pulse, pulse_length, bt, amplitude);

    gr::digital::CpmPulse<T> _pulse{};
    gr::signal::Phasor<T>    _phasor{};
    std::vector<double>      _increments{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        static constexpr std::array kDesignKeys{"modulation_index", "samples_per_symbol", "pulse", "pulse_length", "bt"};

        const bool designed = !_pulse.pulse().empty();
        if (!designed || std::ranges::any_of(kDesignKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            design();
        }
    }

    void start() {
        design();
        _pulse.reset();
        _phasor.setPhase(0.);
    }

    void design() {
        try {
            _pulse.configure(gr::digital::cpmPulseShapeFrom(pulse), static_cast<std::size_t>(pulse_length.value), static_cast<std::size_t>(samples_per_symbol.value), modulation_index, bt);
        } catch (const std::invalid_argument& error) {
            throw gr::exception(error.what()); // a graph reports a bad setting as a gr::exception, the kernel as an argument error
        }

        this->input_chunk_size  = 1U;
        this->output_chunk_size = samples_per_symbol;
    }

    /// @brief One symbol becomes `samples_per_symbol` outputs, so a tag at input `t` belongs at output `sps*t`.
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        const auto                                        sps = static_cast<std::size_t>(samples_per_symbol.value);
        std::optional<property_map>                       cachedSettings;
        std::vector<std::pair<std::size_t, property_map>> arriving;
        gr::for_each_reader_span(
            [&](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const auto& [relIndex, tagMap] : span.tags(processedIn)) {
                    if (relIndex < 0) { // already forwarded when it first arrived
                        continue;
                    }
                    auto forwarded = this->filterAndSubstituteTag(tagMap.get(), cachedSettings);
                    if (!forwarded.empty()) {
                        arriving.emplace_back(sps * static_cast<std::size_t>(relIndex), std::move(forwarded));
                    }
                }
            },
            inputSpans);
        if (arriving.empty()) {
            return;
        }
        gr::for_each_writer_span(
            [&arriving](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const auto& [offset, tagMap] : arriving) {
                    if (offset < span.size()) {
                        span.publishTag(tagMap, offset);
                    }
                }
            },
            outputSpans);
    }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<std::complex<T>> output) {
        const auto        sps      = static_cast<std::size_t>(samples_per_symbol.value);
        const std::size_t nSymbols = std::min(input.size(), output.size() / sps);
        const std::size_t nSamples = nSymbols * sps;

        if (_increments.size() < nSamples) { // grows to the largest chunk seen and stays there
            _increments.resize(nSamples);
        }
        const std::span<double> increments(_increments.data(), nSamples);

        _pulse.incrementsFor(input.first(nSymbols), increments);
        _phasor.fillModulated(std::span<const double>(increments), output.first(nSamples));

        if (amplitude != T(1)) { // scaling by exactly one is the identity, so the common case skips the pass
            const T gain = amplitude;
            std::ranges::transform(output.first(nSamples), output.begin(), [gain](std::complex<T> v) { return gain * v; });
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_CPM_MODULATE_HPP
