#ifndef GNURADIO_CHANNELIZER_HPP
#define GNURADIO_CHANNELIZER_HPP

#include <algorithm>
#include <complex>
#include <concepts>
#include <cstddef>
#include <expected>
#include <format>
#include <limits>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/algorithm/filter/PolyphaseChannelizer.hpp>

namespace gr::blocks::filter {

namespace detail {

/// The widest bank accepted.
inline constexpr gr::Size_t kMaxChannels = static_cast<gr::Size_t>(gr::filter::PolyphaseChannelizer<float>::kMaxChannels);

/// The longest root-Nyquist span accepted, in channel spacings.
inline constexpr std::size_t kMaxSpan = 64UZ;

/// @brief The validator of `transition`: true inside (0, 1).
inline constexpr auto kTransitionInside = [](double transition) { return transition > 0. && transition < 1.; };

/// @brief The validator of `attenuation_db`: true above 0.
inline constexpr auto kAttenuationPositive = [](double attenuationDb) { return attenuationDb > 0.; };

/// @brief Names every setting, for the first read of a block's settings.
inline constexpr auto kEverySetting = [](std::string_view) { return true; };

/// @brief True for a connected port.
inline constexpr auto kIsConnected = [](const auto& port) { return port.isConnected(); };

/// The settings a bank was last built from.
struct ChannelizerSettings {
    std::size_t        channels = 0UZ;
    std::vector<float> taps{};
    double             attenuationDb = 0.;
    double             transition    = 0.;
    std::string        prototype{};
    std::size_t        span       = 0UZ;
    std::size_t        oversample = 0UZ;

    [[nodiscard]] bool operator==(const ChannelizerSettings&) const = default;
};

/// @brief Copies the settings @p moved names from @p block into @p next.
template<typename TBlock, typename TMoved>
[[nodiscard]] ChannelizerSettings channelizerSettingsAfter(const TBlock& block, ChannelizerSettings next, TMoved moved) {
    if (moved("n_channels")) {
        next.channels = static_cast<std::size_t>(block.n_channels.value);
    }
    if (moved("taps")) {
        next.taps = block.taps.value;
    }
    if (moved("attenuation_db")) {
        next.attenuationDb = block.attenuation_db.value;
    }
    if (moved("transition")) {
        next.transition = block.transition.value;
    }
    if (moved("prototype")) {
        next.prototype = block.prototype.value;
    }
    if (moved("span")) {
        next.span = static_cast<std::size_t>(block.span.value);
    }
    if (moved("oversample")) {
        next.oversample = static_cast<std::size_t>(block.oversample.value);
    }
    return next;
}

/**
 * @brief Designs the prototype @p settings name, or measures the supplied one, with its stopband and ripple.
 *
 * A check across settings that fails throws `gr::exception` naming the settings.
 */
[[nodiscard]] inline gr::filter::ChannelizerDesign channelizerPrototype(const ChannelizerSettings& settings) {
    if (settings.prototype != "root_nyquist" && settings.prototype != "lowpass" && settings.prototype != "boxcar") {
        throw gr::exception(std::format("prototype must be 'root_nyquist', 'lowpass' or 'boxcar', got '{}'", settings.prototype));
    }
    if (settings.prototype == "root_nyquist" && (settings.span < 2UZ || settings.span > kMaxSpan)) {
        throw gr::exception(std::format("span is {}; a root_nyquist prototype covers 2 to {} channel spacings", settings.span, kMaxSpan));
    }
    if (settings.oversample == 2UZ && settings.channels % 2UZ != 0UZ) {
        throw gr::exception(std::format("n_channels is {}; oversample 2 needs an even channel count", settings.channels));
    }
    auto measured = settings.taps.empty() ? gr::filter::designChannelizerPrototype(settings.channels, settings.prototype, settings.attenuationDb, settings.transition, settings.span, settings.oversample) : gr::filter::measureChannelizerPrototype(settings.taps, settings.channels, settings.transition, settings.oversample);
    if (!measured) {
        throw gr::exception(measured.error().message, measured.error().sourceLocation);
    }
    return std::move(*measured);
}

/// @brief Divides each `sample_rate` value of @p tag by @p stride, float or double. Returns the divided entries.
[[nodiscard]] inline property_map divideRate(property_map& tag, std::size_t stride) {
    property_map rates;
    for (auto& [key, value] : tag) {
        if (std::string_view(key) != gr::tag::SAMPLE_RATE.shortKey() && std::string_view(key) != gr::tag::SAMPLE_RATE.key()) {
            continue;
        }
        if (const float* rate = value.get_if<float>(); rate != nullptr) {
            value = *rate / static_cast<float>(stride);
        } else if (const double* wide = value.get_if<double>(); wide != nullptr) {
            value = *wide / static_cast<double>(stride);
        } else {
            continue;
        }
        rates.insert_or_assign(key, value);
    }
    return rates;
}

/// The setting labels both banks carry.
using ChannelCountDoc = Doc<"channel count M, 2 to 256, fixed once a port is connected">;
using TapsDoc         = Doc<"prototype taps, empty to design one">;
using DesignedTapsDoc = Doc<"observable running prototype, zero-padded to whole branches">;
using StopbandDoc     = Doc<"observable worst level, alias edge to Nyquist, relative to DC gain">;
using RippleDoc       = Doc<"observable passband ripple from DC to the pass edge">;
using DesignOkDoc     = Doc<"observable, whether the lowpass design met its target">;
using AttenuationDoc  = Doc<"stopband target of the lowpass search">;
using TransitionDoc   = Doc<"excess bandwidth in channel spacings, lowpass transition or root-Nyquist rolloff">;
using PrototypeDoc    = Doc<"prototype family: 'root_nyquist', 'lowpass' or 'boxcar'">;
using SpanDoc         = Doc<"channel spacings the root_nyquist prototype covers, 2 to 64">;
using OversampleDoc   = Doc<"1 for critical sampling, 2 for a commutator stride of M/2">;

/// The settings text both banks carry.
inline constexpr gr::meta::fixed_string kChannelizerSettingsText = R""(

`n_channels` is M, from 2 to 256, and is fixed once a port is connected. When `taps` is empty, the bank designs a
prototype in the family `prototype` names. A prototype in `taps` runs in place of the design and stays as supplied.

`root_nyquist`, the default, is a root-raised cosine of rolloff `transition` that covers `span` channel spacings, 2 to
64. A longer span rejects more. Its squared responses sum flat across the bank. `lowpass` is a Kaiser lowpass cut off
at half a channel spacing. Its passband ends at (1 - `transition`)/2 channel spacings. Its stopband starts at the alias
edge, `oversample` channel spacings less the pass edge. The design takes the shortest length that rejects the stopband
by `attenuation_db` with at most 0.1 dB of passband ripple. `boxcar` is M taps of 1/M, and the bank becomes a plain
block transform. `span` applies to `root_nyquist` alone and `attenuation_db` to `lowpass` alone.

`oversample` 1 advances the commutator M samples per step. `oversample` 2 advances it M/2 samples and needs an even M.
`lowpass` isolates the most and does not reconstruct. `boxcar` reconstructs at either `oversample` and isolates little.
`root_nyquist` with `oversample` 2 does both.

A change to any of these settings designs the prototype again, rebuilds the bank and clears its history. A refused
change leaves the bank as it was.

`designed_taps` holds the prototype that runs, zero-padded to a whole number of branches. `stopband_db` is the
prototype's worst level from the alias edge to Nyquist. That band folds into each channel when the bank decimates.
`ripple_db` is the passband ripple. Both figures are measured before the padding, relative to the DC gain.
`design_ok` is false only when a `lowpass` design misses `attenuation_db` or the ripple limit.
)"";

/// @brief `Head` followed by `Tail` in one text. The `+` operators of `gr::meta::fixed_string` build their result
/// through a constructor that folds over every character, and Clang refuses a fold of more than 2048.
template<gr::meta::fixed_string Head, gr::meta::fixed_string Tail>
[[nodiscard]] consteval auto joinedText() {
    gr::meta::fixed_string<Head.size() + Tail.size()> text{};
    for (std::size_t i = 0UZ; i < Head.size(); ++i) {
        text[i] = Head[i];
    }
    for (std::size_t i = 0UZ; i < Tail.size(); ++i) {
        text[Head.size() + i] = Tail[i];
    }
    return text;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::filter::PolyphaseChannelizer, [T], [float])

template<std::floating_point F>
struct PolyphaseChannelizer : Block<PolyphaseChannelizer<F>, NoTagPropagation, Resampling<1UZ, 1UZ, false>> {
    using Description = Doc<detail::joinedText<R""(
@brief Splits one complex stream into `n_channels` equally spaced channels with a polyphase analysis filter bank.

Channel `k` carries the input's band around `k * fs / M`, shifted to baseband and decimated. Indices above `M/2` are
the negative frequencies in the usual transform order. Each output runs at `fs/M`, or twice that when the bank is
oversampled. Each step consumes `M / oversample` input samples and writes one sample to every channel.

An input tag lands on channel zero, on the output of the step that consumes its sample. A `sample_rate` key is divided
by the commutator stride and reaches every channel. The keys the bank holds as settings are dropped. At end of stream
the samples short of a whole step are dropped, and their tags land on channel zero at the end of the stream.)"",
        detail::kChannelizerSettingsText>()>;

    PortIn<std::complex<F>>               in;
    std::vector<PortOut<std::complex<F>>> out;

    Annotated<gr::Size_t, "n_channels", Visible, detail::ChannelCountDoc, Limits<gr::Size_t{2U}, detail::kMaxChannels>>                                    n_channels = 4U;
    Annotated<std::vector<float>, "taps", detail::TapsDoc>                                                                                                 taps{};
    Annotated<std::vector<float>, "designed_taps", detail::DesignedTapsDoc>                                                                                designed_taps{};
    Annotated<double, "stopband_db", Unit<"dB">, detail::StopbandDoc>                                                                                      stopband_db{};
    Annotated<double, "ripple_db", Unit<"dB">, detail::RippleDoc>                                                                                          ripple_db{};
    Annotated<bool, "design_ok", detail::DesignOkDoc>                                                                                                      design_ok{};
    Annotated<double, "attenuation_db", Unit<"dB">, detail::AttenuationDoc, Limits<0.0, std::numeric_limits<double>::max(), detail::kAttenuationPositive>> attenuation_db = 60.0;
    Annotated<double, "transition", detail::TransitionDoc, Limits<0.0, 1.0, detail::kTransitionInside>>                                                    transition     = 0.5;
    Annotated<std::string, "prototype", Visible, detail::PrototypeDoc>                                                                                     prototype      = std::string("root_nyquist");
    Annotated<gr::Size_t, "span", detail::SpanDoc>                                                                                                         span           = 16U;
    Annotated<gr::Size_t, "oversample", Visible, detail::OversampleDoc, Limits<gr::Size_t{1U}, gr::Size_t{2U}>>                                            oversample     = 1U;

    GR_MAKE_REFLECTABLE(PolyphaseChannelizer, in, out, n_channels, taps, designed_taps, stopband_db, ripple_db, design_ok, attenuation_db, transition, prototype, span, oversample);

    gr::filter::PolyphaseChannelizer<F> _bank{};
    std::vector<std::complex<F>>        _step{};
    detail::ChannelizerSettings         _accepted{};
    std::size_t                         _channels = 4UZ;
    std::size_t                         _stride   = 4UZ;

    /// Builds the bank and its `n_channels` output ports from the initial settings.
    explicit PolyphaseChannelizer(property_map init = {}) : Block<PolyphaseChannelizer<F>, NoTagPropagation, Resampling<1UZ, 1UZ, false>>(std::move(init)) { install(detail::channelizerSettingsAfter(*this, {}, detail::kEverySetting)); }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        detail::ChannelizerSettings next = detail::channelizerSettingsAfter(*this, _accepted, [&newSettings](std::string_view key) { return newSettings.contains(key); });
        if (next == _accepted) {
            return;
        }
        if (next.channels != _accepted.channels && std::ranges::any_of(out, detail::kIsConnected)) {
            throw gr::exception(std::format("n_channels is {}; the channel count cannot change once a port is connected", next.channels));
        }
        install(std::move(next));
    }

    void start() { _bank.reset(); }

    /// @brief Installs an analysis bank for @p next with its output ports and observables. Throws and changes nothing
    /// on a refusal.
    void install(detail::ChannelizerSettings next) {
        const gr::filter::ChannelizerDesign measured = detail::channelizerPrototype(next);
        gr::filter::PolyphaseChannelizer<F> bank;
        if (const std::expected<void, gr::Error> configured = bank.configure(next.channels, measured.taps, next.oversample); !configured) {
            throw gr::exception(configured.error().message, configured.error().sourceLocation);
        }

        _bank     = std::move(bank);
        _accepted = std::move(next);
        _channels = _bank.channels();
        _stride   = _bank.stride();
        _step.assign(_channels, std::complex<F>{});
        out.resize(_channels);
        designed_taps.value = std::vector<float>(_bank.prototype().begin(), _bank.prototype().end());
        stopband_db.value   = measured.stopbandDb;
        ripple_db.value     = measured.rippleDb;
        design_ok.value     = measured.ok;

        this->input_chunk_size  = static_cast<gr::Size_t>(_stride);
        this->output_chunk_size = 1U;
    }

    template<typename TOutputSpanType>
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, std::span<TOutputSpanType>& outputs) {
        const std::size_t stride   = _stride;
        const std::size_t channels = _channels;
        std::size_t       nSteps   = inSpan.size() / stride;
        for (const auto& port : outputs) {
            nSteps = std::min(nSteps, port.size());
        }

        for (std::size_t s = 0UZ; s < nSteps; ++s) {
            _bank.step(std::span<const std::complex<F>>(inSpan).subspan(s * stride, stride), _step);
            for (std::size_t c = 0UZ; c < channels; ++c) {
                outputs[c][s] = _step[c];
            }
        }

        for (const auto& [relIndex, tagMap] : inSpan.tags()) {
            if (relIndex < 0) {
                continue;
            }
            const std::size_t at = static_cast<std::size_t>(relIndex) / stride;
            if (at < nSteps) {
                forwardTag(tagMap.get(), outputs, at);
            }
        }

        std::ignore = inSpan.consume(nSteps * stride);
        for (auto& port : outputs) {
            port.publish(nSteps);
        }
        return work::Status::OK;
    }

    /// @brief At end of stream, forwards the tags of the samples short of a whole step to the end-of-stream index.
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        auto& outputs = std::get<0>(outputSpans);
        if (processedIn > 0UZ || std::ranges::any_of(outputs, [](const auto& output) { return output.size() > 0UZ; })) {
            return; // processBulk forwards the tags of the steps it takes
        }
        const std::pmr::string endOfStream = static_cast<std::pmr::string>(gr::tag::END_OF_STREAM);
        const auto&            input       = std::get<0>(inputSpans);
        for (const Tag& remaining : input.rawTags) {
            if (remaining.index < input.streamIndex) {
                continue;
            }
            property_map tag = remaining.map;
            tag.erase(endOfStream);
            forwardTag(std::move(tag), outputs, 0UZ);
        }
    }

    /// @brief Publishes @p tag at @p offset on channel zero, less the bank's setting keys, and its divided
    /// `sample_rate` on every other channel.
    template<typename TOutputs>
    void forwardTag(property_map tag, TOutputs& outputs, std::size_t offset) {
        if (outputs.empty()) {
            return;
        }
        const std::set<std::string>& owned = CtxSettings<PolyphaseChannelizer<F>>::allWritableMembers();
        std::erase_if(tag, [&owned](const auto& entry) { return owned.contains(std::string(entry.first)); });
        const property_map rates = detail::divideRate(tag, _stride);
        if (!tag.empty()) {
            outputs[0].publishTag(std::move(tag), offset);
        }
        if (!rates.empty()) {
            for (std::size_t c = 1UZ; c < outputs.size(); ++c) {
                outputs[c].publishTag(rates, offset);
            }
        }
    }
};

GR_REGISTER_BLOCK(gr::blocks::filter::PolyphaseSynthesizer, [T], [float])

template<std::floating_point F>
struct PolyphaseSynthesizer : Block<PolyphaseSynthesizer<F>, NoTagPropagation, Resampling<1UZ, 1UZ, false>> {
    using Description = Doc<detail::joinedText<R""(
@brief Combines `n_channels` equally spaced channels into one complex stream with a polyphase synthesis filter bank.

It is the transpose of the analysis bank, with the same settings and channel numbering. Channel `k` is placed at
`k * fs / M` of the output rate. Each step takes one sample per channel and writes `M / oversample` output samples.

Fed the channels of an analysis bank with the same settings, it returns that bank's input delayed by `((N-1)/S)*S`
samples and scaled by the cascade's gain. N is the length of `designed_taps`, and S is M / `oversample`. `boxcar`
reconstructs at either `oversample`, and `root_nyquist` with `oversample` 2. A `lowpass` prototype leaves a residual,
and so does a critically sampled `root_nyquist` pair.

Tags on the channel inputs are dropped, and the output carries no `sample_rate`.)"",
        detail::kChannelizerSettingsText>()>;

    std::vector<PortIn<std::complex<F>>> in;
    PortOut<std::complex<F>>             out;

    Annotated<gr::Size_t, "n_channels", Visible, detail::ChannelCountDoc, Limits<gr::Size_t{2U}, detail::kMaxChannels>>                                    n_channels = 4U;
    Annotated<std::vector<float>, "taps", detail::TapsDoc>                                                                                                 taps{};
    Annotated<std::vector<float>, "designed_taps", detail::DesignedTapsDoc>                                                                                designed_taps{};
    Annotated<double, "stopband_db", Unit<"dB">, detail::StopbandDoc>                                                                                      stopband_db{};
    Annotated<double, "ripple_db", Unit<"dB">, detail::RippleDoc>                                                                                          ripple_db{};
    Annotated<bool, "design_ok", detail::DesignOkDoc>                                                                                                      design_ok{};
    Annotated<double, "attenuation_db", Unit<"dB">, detail::AttenuationDoc, Limits<0.0, std::numeric_limits<double>::max(), detail::kAttenuationPositive>> attenuation_db = 60.0;
    Annotated<double, "transition", detail::TransitionDoc, Limits<0.0, 1.0, detail::kTransitionInside>>                                                    transition     = 0.5;
    Annotated<std::string, "prototype", Visible, detail::PrototypeDoc>                                                                                     prototype      = std::string("root_nyquist");
    Annotated<gr::Size_t, "span", detail::SpanDoc>                                                                                                         span           = 16U;
    Annotated<gr::Size_t, "oversample", Visible, detail::OversampleDoc, Limits<gr::Size_t{1U}, gr::Size_t{2U}>>                                            oversample     = 1U;

    GR_MAKE_REFLECTABLE(PolyphaseSynthesizer, in, out, n_channels, taps, designed_taps, stopband_db, ripple_db, design_ok, attenuation_db, transition, prototype, span, oversample);

    gr::filter::PolyphaseSynthesizer<F> _bank{};
    std::vector<std::complex<F>>        _step{};
    detail::ChannelizerSettings         _accepted{};
    std::size_t                         _channels = 4UZ;
    std::size_t                         _stride   = 4UZ;

    /// Builds the bank and its `n_channels` input ports from the initial settings.
    explicit PolyphaseSynthesizer(property_map init = {}) : Block<PolyphaseSynthesizer<F>, NoTagPropagation, Resampling<1UZ, 1UZ, false>>(std::move(init)) { install(detail::channelizerSettingsAfter(*this, {}, detail::kEverySetting)); }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        detail::ChannelizerSettings next = detail::channelizerSettingsAfter(*this, _accepted, [&newSettings](std::string_view key) { return newSettings.contains(key); });
        if (next == _accepted) {
            return;
        }
        if (next.channels != _accepted.channels && std::ranges::any_of(in, detail::kIsConnected)) {
            throw gr::exception(std::format("n_channels is {}; the channel count cannot change once a port is connected", next.channels));
        }
        install(std::move(next));
    }

    void start() { _bank.reset(); }

    /// @brief Installs a synthesis bank for @p next with its input ports and observables. Throws and changes nothing on
    /// a refusal.
    void install(detail::ChannelizerSettings next) {
        const gr::filter::ChannelizerDesign measured = detail::channelizerPrototype(next);
        gr::filter::PolyphaseSynthesizer<F> bank;
        if (const std::expected<void, gr::Error> configured = bank.configure(next.channels, measured.taps, next.oversample); !configured) {
            throw gr::exception(configured.error().message, configured.error().sourceLocation);
        }

        _bank     = std::move(bank);
        _accepted = std::move(next);
        _channels = _bank.channels();
        _stride   = _bank.stride();
        _step.assign(_channels, std::complex<F>{});
        in.resize(_channels);
        designed_taps.value = std::vector<float>(_bank.prototype().begin(), _bank.prototype().end());
        stopband_db.value   = measured.stopbandDb;
        ripple_db.value     = measured.rippleDb;
        design_ok.value     = measured.ok;

        this->input_chunk_size  = 1U;
        this->output_chunk_size = static_cast<gr::Size_t>(_stride);
    }

    template<typename TInputSpanType>
    [[nodiscard]] work::Status processBulk(std::span<TInputSpanType>& inputs, std::span<std::complex<F>> output) {
        const std::size_t stride   = _stride;
        const std::size_t channels = _channels;
        std::size_t       nSteps   = output.size() / stride;
        for (const auto& port : inputs) {
            nSteps = std::min(nSteps, port.size());
        }

        for (std::size_t s = 0UZ; s < nSteps; ++s) {
            for (std::size_t c = 0UZ; c < channels; ++c) {
                _step[c] = inputs[c][s];
            }
            _bank.step(_step, output.subspan(s * stride, stride));
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_CHANNELIZER_HPP
