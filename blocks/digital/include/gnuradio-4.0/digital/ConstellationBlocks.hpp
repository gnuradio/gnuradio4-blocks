#ifndef GNURADIO_DIGITAL_CONSTELLATION_BLOCKS_HPP
#define GNURADIO_DIGITAL_CONSTELLATION_BLOCKS_HPP

#include <algorithm>
#include <array>
#include <complex>
#include <concepts>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/digital/Constellation.hpp>

#include <gnuradio-4.0/digital/ConstellationSettings.hpp>

namespace gr::blocks::digital {

GR_REGISTER_BLOCK(gr::blocks::digital::ConstellationDecoder, [T], [float])

template<std::floating_point F>
struct ConstellationDecoder : Block<ConstellationDecoder<F>, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief The hard decision: the label of the constellation point nearest each input sample.

The definition is the nearest point, ties to the lowest symbol, and that is what a `custom` constellation gets. A
geometry that admits a cheaper exact answer takes it - a square QAM slices per axis, a PSK constellation takes an
angle sector - resolved once, at construction, from the points themselves, with no virtual call.

The output byte is the point's label, and the labeling is Gray. The decision is exactly scale-invariant for PSK but
not for QAM, which compares against absolute levels - an AGC holding the input at the constellation's own scale is a
stated precondition of every QAM decision.

The block is one symbol in, one symbol out, so every input tag key passes through at its own offset.
)"">;

    PortIn<std::complex<F>> in;
    PortOut<std::uint8_t>   out;

    Annotated<std::string, "constellation", detail::ConstellationSettingsDoc, Visible> constellation = std::string("qpsk");
    Annotated<gr::Size_t, "arity", Unit<"symbols">, Doc<"M, for 'psk' and 'qam'">>     arity         = 4U;
    Annotated<F, "phase_offset", Unit<"rad">, Doc<"rotation of the 'psk' point set">>  phase_offset  = F{0};
    Annotated<gr::Size_t, "label_xor", Doc<"label offset of the 'psk' point set">>     label_xor     = 0U;
    Annotated<std::vector<F>, "points", Doc<"interleaved re,im for 'custom'">>         points{};
    Annotated<std::string, "normalization", Doc<"'power', 'amplitude' or 'none'">>     normalization = std::string("power");

    GR_MAKE_REFLECTABLE(ConstellationDecoder, in, out, constellation, arity, phase_offset, label_xor, points, normalization);

    gr::digital::Constellation<F> _constellation = gr::digital::Constellation<F>::qpsk();

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() { _constellation = detail::build<F>(constellation, arity, phase_offset, label_xor, std::span<const F>(points.value), normalization); }

    [[nodiscard]] std::size_t bitsPerSymbol() const { return _constellation.bitsPerSymbol(); }

    [[nodiscard]] work::Status processBulk(std::span<const std::complex<F>> input, std::span<std::uint8_t> output) {
        const std::size_t nSamples = std::min(input.size(), output.size());
        _constellation.hardDecisions(input.first(nSamples), output.first(nSamples));
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::digital::ConstellationSoftDecoder, [T], [float])

// The declared rate is the declared constellation's: qpsk carries two bits, and `rebuild()` follows the setting from
// there. A block whose construction moved no value is never called back, so the two defaults have to agree.
template<std::floating_point F>
struct ConstellationSoftDecoder : Block<ConstellationSoftDecoder<F>, Resampling<1UZ, 2UZ, false>> {
    using Description = Doc<R""(
@brief `bitsPerSymbol()` log-likelihood ratios per input symbol, most significant bit first.

A positive value means the bit is one, `ln(P(b=1|z)/P(b=0|z))`; much of the coding literature writes the ratio the
other way up, so anything crossing that boundary needs one negation.

`noise_power` is `N0`, and the soft decision compares a distance against it, so unlike the hard decision it is
scale-sensitive for every constellation: an AGC holding the input at the constellation's own scale is a stated
precondition. `max_log` is the default and needs no transcendental; it is exact for BPSK and QPSK and within about
`ln 2` elsewhere. The block is a 1:`m` interpolator whose ratio follows the constellation, so changing
`constellation` changes the rate.
)"">;

    PortIn<std::complex<F>> in;
    PortOut<F>              out;

    Annotated<std::string, "constellation", detail::ConstellationSettingsDoc, Visible>                             constellation = std::string("qpsk");
    Annotated<gr::Size_t, "arity", Unit<"symbols">, Doc<"M, for 'psk' and 'qam'">>                                 arity         = 4U;
    Annotated<F, "phase_offset", Unit<"rad">, Doc<"rotation of the 'psk' point set">>                              phase_offset  = F{0};
    Annotated<gr::Size_t, "label_xor", Doc<"label offset of the 'psk' point set">>                                 label_xor     = 0U;
    Annotated<std::vector<F>, "points", Doc<"interleaved re,im for 'custom'">>                                     points{};
    Annotated<std::string, "normalization", Doc<"'power', 'amplitude' or 'none'">>                                 normalization  = std::string("power");
    Annotated<F, "noise_power", Doc<"N0, the total complex noise power; linear, never dB, and strictly positive">> noise_power    = F{1};
    Annotated<std::string, "soft_algorithm", Doc<"'max_log', or 'exact' which is the true LLR and costs a log">>   soft_algorithm = std::string("max_log");

    GR_MAKE_REFLECTABLE(ConstellationSoftDecoder, in, out, constellation, arity, phase_offset, label_xor, points, normalization, noise_power, soft_algorithm);

    gr::digital::Constellation<F> _constellation = gr::digital::Constellation<F>::qpsk();
    gr::digital::SoftAlgorithm    _algorithm     = gr::digital::SoftAlgorithm::MaxLog;
    std::size_t                   _bits          = 2UZ;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        if (!(noise_power > F{0})) {
            throw gr::exception(std::format("noise_power is a linear total complex noise power and must be strictly positive, got {}", static_cast<double>(noise_power.value)));
        }
        if (soft_algorithm != "max_log" && soft_algorithm != "exact") {
            throw gr::exception(std::format("soft_algorithm must be 'max_log' or 'exact', got '{}'", soft_algorithm.value));
        }
        _algorithm     = soft_algorithm == "exact" ? gr::digital::SoftAlgorithm::Exact : gr::digital::SoftAlgorithm::MaxLog;
        _constellation = detail::build<F>(constellation, arity, phase_offset, label_xor, std::span<const F>(points.value), normalization);
        _bits          = _constellation.bitsPerSymbol();

        this->input_chunk_size  = 1U;
        this->output_chunk_size = static_cast<gr::Size_t>(_bits);
    }

    [[nodiscard]] std::size_t bitsPerSymbol() const noexcept { return _bits; }

    /// @brief One input symbol becomes `m` outputs, so a tag at input `t` belongs at output `m*t` — integer, in `std::size_t`.
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        const std::size_t                                 bits = _bits;
        std::vector<std::pair<std::size_t, property_map>> arriving;
        gr::for_each_reader_span(
            [&arriving, processedIn, bits](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const auto& [relIndex, tagMap] : span.tags(processedIn)) {
                    if (relIndex < 0) { // already forwarded when it first arrived
                        continue;
                    }
                    arriving.emplace_back(bits * static_cast<std::size_t>(relIndex), tagMap.get());
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

    [[nodiscard]] work::Status processBulk(std::span<const std::complex<F>> input, std::span<F> output) {
        const std::size_t nSymbols = std::min(input.size(), output.size() / _bits);
        _constellation.softDecisions(input.first(nSymbols), noise_power, output.first(nSymbols * _bits), _algorithm);
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::digital::ConstellationEncoder, [T], [float])

template<std::floating_point F>
struct ConstellationEncoder : Block<ConstellationEncoder<F>, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief The point for each input symbol: the modulator's map from labels to the complex plane.

A stream byte cannot read past the point list: the symbol is masked to the arity, so a BPSK encoder handed the byte 200
emits the point for `200 mod 2`. Reducing rather than throwing is the same rule the differential coder states, that a
stream value must not be able to stop a graph.

The points carry the normalization the constellation was built with, applied once, at construction, to the point list.
Nothing is scaled per sample and no scale factor is carried separately, which is why there is no method that
renormalizes in place.

The block is one symbol in, one symbol out, so every input tag key passes through at its own offset.
)"">;

    PortIn<std::uint8_t>     in;
    PortOut<std::complex<F>> out;

    Annotated<std::string, "constellation", detail::ConstellationSettingsDoc, Visible> constellation = std::string("qpsk");
    Annotated<gr::Size_t, "arity", Unit<"symbols">, Doc<"M, for 'psk' and 'qam'">>     arity         = 4U;
    Annotated<F, "phase_offset", Unit<"rad">, Doc<"rotation of the 'psk' point set">>  phase_offset  = F{0};
    Annotated<gr::Size_t, "label_xor", Doc<"label offset of the 'psk' point set">>     label_xor     = 0U;
    Annotated<std::vector<F>, "points", Doc<"interleaved re,im for 'custom'">>         points{};
    Annotated<std::string, "normalization", Doc<"'power', 'amplitude' or 'none'">>     normalization = std::string("power");

    GR_MAKE_REFLECTABLE(ConstellationEncoder, in, out, constellation, arity, phase_offset, label_xor, points, normalization);

    gr::digital::Constellation<F> _constellation = gr::digital::Constellation<F>::qpsk();

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() { _constellation = detail::build<F>(constellation, arity, phase_offset, label_xor, std::span<const F>(points.value), normalization); }

    [[nodiscard]] std::size_t bitsPerSymbol() const { return _constellation.bitsPerSymbol(); }

    [[nodiscard]] work::Status processBulk(std::span<const std::uint8_t> input, std::span<std::complex<F>> output) noexcept {
        const std::size_t nSamples = std::min(input.size(), output.size());
        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            output[i] = _constellation.point(input[i]);
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::digital::SymbolMap)

struct SymbolMap : Block<SymbolMap, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief A byte permutation: `out[n] = map[in[n]]`.

The table is 256 entries because the index is a byte, so stream data cannot make the lookup read out of range whatever
the constellation's arity, which is the reason not to shrink it to `M`. A supplied table shorter than 256 fills the
rest with identity; a longer one throws rather than being silently truncated.

Its use is the Gray-to-rotation permutation a differential chain needs: differential coding requires a rotation-ordered
labeling, `c[(s+1) mod M] = c[s] * exp(j*2*pi/M)`, which a Gray labeling is not for `M >= 4`. The permutation belongs
on the data — `symbol_map` before the differential encoder on transmit, after the differential decoder on receive —
rather than inside the constellation, which keeps the hard and the soft decision from disagreeing about which labeling
is in force. `Constellation::rotationToGray()` and `grayToRotation()` supply the tables.

The block is 1:1, so every input tag key passes through at its own offset.
)"">;

    PortIn<std::uint8_t>  in;
    PortOut<std::uint8_t> out;

    Annotated<std::vector<std::uint8_t>, "map", Doc<"up to 256 entries; a short table is identity-filled, a longer one throws">, Visible> map{};

    GR_MAKE_REFLECTABLE(SymbolMap, in, out, map);

    std::array<std::uint8_t, 256> _table = identityTable();

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        if (map.value.size() > _table.size()) {
            throw gr::exception(std::format("map holds {} entries and the index is a byte, so at most {} are meaningful; a longer table is a mistake rather than something to truncate", map.value.size(), _table.size()));
        }
        for (std::size_t i = 0UZ; i < _table.size(); ++i) {
            _table[i] = i < map.value.size() ? map.value[i] : static_cast<std::uint8_t>(i);
        }
    }

    [[nodiscard]] work::Status processBulk(std::span<const std::uint8_t> input, std::span<std::uint8_t> output) noexcept {
        const std::size_t nSamples = std::min(input.size(), output.size());
        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            output[i] = _table[input[i]];
        }
        return work::Status::OK;
    }

    [[nodiscard]] static constexpr std::array<std::uint8_t, 256> identityTable() noexcept {
        std::array<std::uint8_t, 256> table{};
        for (std::size_t i = 0UZ; i < table.size(); ++i) {
            table[i] = static_cast<std::uint8_t>(i);
        }
        return table;
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_CONSTELLATION_BLOCKS_HPP
