#ifndef GNURADIO_SVD_DENOISER_HPP
#define GNURADIO_SVD_DENOISER_HPP

#include <algorithm>
#include <cstdint>
#include <optional>
#include <span>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/algorithm/filter/SvdFilter.hpp>

#include <gnuradio-4.0/filter/NamespaceCompatibility.hpp>
#include <gnuradio-4.0/filter/TagDelay.hpp>

namespace gr::blocks::filter {

using namespace gr;

GR_REGISTER_BLOCK(gr::blocks::filter::SvdDenoiser, [T], [ float, std::complex<float> ])

template<typename T>
struct SvdDenoiser : Block<SvdDenoiser<T>>, detail::DelayedTagFilter<SvdDenoiser<T>, T, T> {
    using Block<SvdDenoiser<T>>::Block;
    static_assert(std::floating_point<T> || gr::meta::complex_like<T>, "T must be floating_point or complex_like");

    using Description = Doc<R""(@brief SVD-based signal denoiser

Denoises signals using Singular Value Decomposition applied to a Hankel matrix
representation. Effective for removing broadband noise from periodic or
quasi-periodic signals while preserving signal structure.

Singular values are kept if ALL criteria are satisfied:
- count ≤ max_rank
- σ_i / σ_0 ≥ relative_threshold
- σ_i ≥ absolute_threshold
- cumulative energy ≤ energy_fraction × total energy

The output lags the input by `max((window_size-1)/2, hop-1)` samples, `hop` being the SVD recomputation interval in
samples. Every tag moves whole, every key with it, by that lag: a tag on input `i` leaves on output `i + lag`, the
output that estimates input `i`. A tag whose output lies past the stream's last output leaves at the end-of-stream
index, one past that output.
)"">; // clang-format off

    using RealT = gr::meta::fundamental_base_value_type_t<T>;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "window size", Doc<"analysis window size (samples)">>                   window_size = 64U;
    Annotated<gr::Size_t, "Hankel rows", Doc<"number of Hankel matrix rows (0 = window_size/2)">> hankel_rows = 0U;
    Annotated<gr::Size_t, "max rank", Doc<"maximum singular values to keep (max = no limit)">>    max_rank    = std::numeric_limits<gr::Size_t>::max();

    Annotated<RealT, "relative threshold", Doc<"minimum ratio σ_i/σ_0 to keep">, Unit<"ratio">>
        relative_threshold = std::numeric_limits<RealT>::epsilon();

    Annotated<RealT, "absolute threshold", Doc<"minimum absolute value of σ_i to keep">>
        absolute_threshold = std::numeric_limits<RealT>::epsilon();

    Annotated<RealT, "energy fraction", Doc<"fraction of total energy to retain (1.0 = all)">, Unit<"ratio">>
        energy_fraction = RealT(0.95);

    Annotated<RealT, "hop fraction", Doc<"SVD recomputation interval as fraction of window_size">, Unit<"ratio">>
        hop_fraction = RealT{0.25};

    GR_MAKE_REFLECTABLE(SvdDenoiser, in, out, window_size, hankel_rows, max_rank, relative_threshold, absolute_threshold, energy_fraction, hop_fraction);

private:
    algorithm::svd_filter::SvdDenoiser<T> _state;

    [[nodiscard]] algorithm::svd_filter::Config<RealT> buildConfig() const {
        return {
            .maxRank           = static_cast<std::size_t>(max_rank),
            .relativeThreshold = relative_threshold,
            .absoluteThreshold = absolute_threshold,
            .energyFraction    = energy_fraction,
            .hopFraction       = hop_fraction
        };
    }

public:
    void start() {
        _state = algorithm::svd_filter::SvdDenoiser<T>(
            static_cast<std::size_t>(window_size),
            static_cast<std::size_t>(hankel_rows),
            buildConfig());
        this->tagsStart();
    }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (newSettings.contains("window_size") || newSettings.contains("hankel_rows") ||
            newSettings.contains("max_rank") || newSettings.contains("relative_threshold") ||
            newSettings.contains("absolute_threshold") || newSettings.contains("energy_fraction") ||
            newSettings.contains("hop_fraction")) {
            _state.setParameters(
                static_cast<std::size_t>(window_size),
                static_cast<std::size_t>(hankel_rows),
                buildConfig());
        }
    }

    void reset() { _state.reset(); }

    [[nodiscard]] std::size_t tagDecimation() const noexcept { return 1UZ; }

    /// @brief The lag in half samples. Each SVD computation serves the next `hop` outputs from index `W-1-lag` of its
    /// window, oldest first; that index is `W-1-(W-1)/2`, or `W-hop` where a hop reaches past the window's middle.
    [[nodiscard]] std::optional<std::uint64_t> twiceTagDelay() const noexcept {
        const std::uint64_t window = _state.windowSize();
        const std::uint64_t hop    = _state.hopSize();
        const std::uint64_t first  = std::min(window - 1ULL - (window - 1ULL) / 2ULL, window > hop ? window - hop : 0ULL);
        return 2ULL * (window - 1ULL - first);
    }

    void filterSamples(std::span<const T> input, std::span<T> output) {
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            output[i] = _state.processOne(input[i]);
        }
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_SVD_DENOISER_HPP
