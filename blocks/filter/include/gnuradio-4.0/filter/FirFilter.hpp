#ifndef GNURADIO_FIR_FILTER_HPP
#define GNURADIO_FIR_FILTER_HPP

#include <algorithm>
#include <complex>
#include <concepts>
#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/filter/PolyphaseResampler.hpp>

#include <gnuradio-4.0/filter/NamespaceCompatibility.hpp>

namespace gr::blocks::filter {

namespace detail {

/**
 * @brief The FIR machinery a taps-driven block runs on, shared by every block that owns a tap
 * vector by whatever authority: the polyphase kernel binding, the newest `N-1` input history
 * that lets a live tap change keep the input/output alignment, and the decimation-aware tag
 * placement with its re-origination on a rate change.
 *
 * The deriving block owns the taps and the decimation setting and passes both into
 * `coreRebuild`/`coreStart`; the core owns everything downstream of them. `TDerived` reaches
 * back only for what belongs to the block framework: the chunk sizes and the forwarded
 * `sample_rate` scaling.
 */
template<typename TDerived, typename TSample, typename TTap>
struct FirFilterCore {
    using TKernel = gr::filter::PolyphaseResampler<TSample, TTap>;
    using TOut    = typename TKernel::output_type;

    std::optional<TKernel>                              _fir;
    std::size_t                                         _decimation = 1UZ;
    std::vector<TSample>                                _history; /// the newest N-1 input samples, oldest first, so a tap change can keep the alignment
    std::uint64_t                                       _inOrigin  = 0ULL;
    std::uint64_t                                       _outOrigin = 0ULL;
    bool                                                _reorigin  = false;
    std::vector<std::pair<std::uint64_t, property_map>> _pendingTags;

    [[nodiscard]] TDerived&       self() noexcept { return static_cast<TDerived&>(*this); }
    [[nodiscard]] const TDerived& self() const noexcept { return static_cast<const TDerived&>(*this); }

    [[nodiscard]] bool coreLive() const noexcept { return _fir.has_value(); }

    /**
     * @brief Install @p taps at @p decimation, keeping the input/output alignment when @p carryHistory.
     *
     * The kernel's window is its own, so the core keeps the newest `N-1` samples beside it and replays them into the
     * new kernel: the window that results is the newest history with any deficit zero-filled, the phase is back at the
     * next unconsumed sample, and output `k` is still `y[k*M]`. The replay is quadratic in the tap count at `M = 1`,
     * but it runs on the settings path and never on the sample path.
     */
    void coreRebuild(std::span<const TTap> taps, gr::Size_t decimation, bool carryHistory) {
        if (decimation < 1U) {
            throw gr::exception(std::format("decimation ({}) must be at least one", decimation));
        }
        if (taps.empty()) {
            throw gr::exception("taps is empty; a pass-through is taps = {1}");
        }

        const std::vector<TSample> carried = std::move(_history);

        _fir.emplace(1UZ, static_cast<std::size_t>(decimation), taps);
        _decimation = decimation;
        _history.assign(taps.size() - 1UZ, TSample{});

        self().input_chunk_size  = static_cast<gr::Size_t>(decimation);
        self().output_chunk_size = 1U;

        if (!carryHistory || carried.empty() || _history.empty()) {
            return;
        }
        const std::size_t keep = std::min(carried.size(), _history.size());
        std::copy(carried.end() - static_cast<std::ptrdiff_t>(keep), carried.end(), _history.end() - static_cast<std::ptrdiff_t>(keep));

        // whole chunks, so the replay leaves the phase at the next unconsumed sample exactly as the old kernel had it
        const std::size_t    pad = (_decimation - _history.size() % _decimation) % _decimation;
        std::vector<TSample> replay(pad + _history.size(), TSample{});
        std::copy(_history.begin(), _history.end(), replay.begin() + static_cast<std::ptrdiff_t>(pad));
        std::vector<TOut> discarded(replay.size() / _decimation);
        std::ignore = _fir->process(replay, discarded);
    }

    void coreStart(std::span<const TTap> taps, gr::Size_t decimation) {
        coreRebuild(taps, decimation, false);
        _pendingTags.clear();
        _inOrigin  = 0ULL;
        _outOrigin = 0ULL;
        _reorigin  = false;
    }

    /// @brief A decimation change applied between calls takes its new origin on the next call.
    void coreMarkReorigin() noexcept { _reorigin = true; }

    /**
     * @brief Place every input tag at the output offset the decimation puts it at, from the current phase origin.
     *
     * This replaces the framework's forwarding rather than adjusting it, and it is also where a `decimation` change
     * takes its new origin: the change is applied on the settings path, between calls, where neither absolute offset
     * is knowable. A `sample_rate` tag is divided by the decimation as it is taken in, so a tag that crossed before a
     * rate change carries the ratio that was in force when it crossed.
     */
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        if (_reorigin) {
            gr::for_each_reader_span(
                [this](auto& span) {
                    if (span.isSync && span.isConnected) {
                        _inOrigin = static_cast<std::uint64_t>(span.streamIndex);
                    }
                },
                inputSpans);
            gr::for_each_writer_span(
                [this](auto& span) {
                    if (span.isSync && span.isConnected) {
                        _outOrigin = static_cast<std::uint64_t>(span.streamIndex);
                    }
                },
                outputSpans);
            _reorigin = false;
        }

        std::vector<std::pair<std::uint64_t, property_map>> arriving;
        gr::for_each_reader_span(
            [&arriving, processedIn, this](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const auto& [relIndex, tagMap] : span.tags(processedIn)) {
                    if (relIndex < 0) { // a tag from before this window is one this block has already placed
                        continue;
                    }
                    const std::uint64_t at = static_cast<std::uint64_t>(span.streamIndex) + static_cast<std::uint64_t>(relIndex);
                    property_map        forwarded(tagMap.get());
                    self().scaleSampleRateByChunkRatio(forwarded); // the rate in force where the tag crossed, not where it is published
                    arriving.emplace_back(_outOrigin + gr::filter::mapResampledOffset(at - _inOrigin, 1ULL, _decimation), std::move(forwarded));
                }
            },
            inputSpans);

        if (arriving.empty() && _pendingTags.empty()) {
            return;
        }

        std::vector<std::pair<std::uint64_t, property_map>> deferred;
        gr::for_each_writer_span(
            [&](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                const std::uint64_t base = static_cast<std::uint64_t>(span.streamIndex);
                const std::uint64_t end  = base + span.size();

                const auto place = [&](const std::pair<std::uint64_t, property_map>& tag) {
                    if (tag.first >= end) { // its output is not in this call: hold it rather than move it
                        deferred.push_back(tag);
                        return;
                    }
                    span.publishTag(tag.second, static_cast<std::size_t>(tag.first > base ? tag.first - base : 0ULL));
                };
                for (const auto& tag : _pendingTags) {
                    place(tag);
                }
                for (const auto& tag : arriving) {
                    place(tag);
                }
            },
            outputSpans);

        _pendingTags = std::move(deferred);
    }

    [[nodiscard]] work::Status processBulk(std::span<const TSample> input, std::span<TOut> output) {
        const std::size_t made = _fir->process(input, output);
        if (made != output.size()) {
            throw gr::exception(std::format("{} input samples yield {} outputs, not the {} reserved", input.size(), made, output.size()));
        }
        rememberHistory(input);
        return work::Status::OK;
    }

private:
    /// @brief Keep the newest `N-1` samples, which is what a taps change needs and what the kernel does not hand back.
    void rememberHistory(std::span<const TSample> input) {
        const std::size_t keep = _history.size();
        if (keep == 0UZ) {
            return;
        }
        if (input.size() >= keep) {
            std::copy_n(input.end() - static_cast<std::ptrdiff_t>(keep), keep, _history.begin());
            return;
        }
        std::copy(_history.begin() + static_cast<std::ptrdiff_t>(input.size()), _history.end(), _history.begin());
        std::copy(input.begin(), input.end(), _history.end() - static_cast<std::ptrdiff_t>(input.size()));
    }
};

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::filter::FirFilter, ([T], [U]), [ float, std::complex<float> ], [ float, std::complex<float> ])

template<typename TSample, typename TTap = float>
requires((std::same_as<TSample, float> || std::same_as<TSample, std::complex<float>>) && (std::same_as<TTap, float> || std::same_as<TTap, std::complex<float>>))
struct FirFilter : Block<FirFilter<TSample, TTap>, Resampling<1UZ, 1UZ, false>>, detail::FirFilterCore<FirFilter<TSample, TTap>, TSample, TTap> {
    using TParent     = Block<FirFilter<TSample, TTap>, Resampling<1UZ, 1UZ, false>>;
    using TCore       = detail::FirFilterCore<FirFilter<TSample, TTap>, TSample, TTap>;
    using TOut        = typename TCore::TOut;
    using Description = Doc<R""(
@brief A FIR filter with arbitrary taps over real or complex samples, decimating by one or more.

`out[k] = sum_i taps[i] * x[k*decimation - i]`. The tap type is separate from the sample type and all four
combinations exist. There is no design path - taps come from `gr::filter::fir::design` - and an empty `taps` throws rather
than becoming a pass-through; a pass-through is `taps = {1}`.

A taps change preserves the input/output alignment exactly; changing `decimation` moves the phase origin. The group
delay is stated, not compensated. A forwarded `sample_rate` tag is divided by the decimation, so downstream reads the
rate of the stream this block hands it.
)"">;

    PortIn<TSample> in;
    PortOut<TOut>   out;

    Annotated<std::vector<TTap>, "taps", Doc<"the impulse response in natural order; taps[0] multiplies the newest sample. A real-valued tap set belongs in a real tap type">> taps{std::vector<TTap>{TTap{1}}};
    Annotated<gr::Size_t, "decimation", Doc<"M: input samples consumed per output; at least one">, Visible>                                                                    decimation = 1U;

    GR_MAKE_REFLECTABLE(FirFilter, in, out, taps, decimation);

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const bool live        = this->coreLive();
        const bool rateChanged = newSettings.contains("decimation");
        if (!live || rateChanged || newSettings.contains("taps")) {
            this->coreRebuild(std::span<const TTap>(taps.value), decimation, live);
        }
        if (live && rateChanged) {
            this->coreMarkReorigin();
        }
    }

    void start() { this->coreStart(std::span<const TTap>(taps.value), decimation); }

    /// @brief The delay of a symmetric design of this length, in input samples. An asymmetric tap set has no single group delay.
    [[nodiscard]] double groupDelaySamples() const noexcept { return 0.5 * static_cast<double>(taps.value.size() - 1UZ); }

    [[nodiscard]] std::size_t tapCount() const noexcept { return taps.value.size(); }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_FIR_FILTER_HPP
