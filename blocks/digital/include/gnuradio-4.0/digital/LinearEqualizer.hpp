#ifndef GNURADIO_LINEAR_EQUALIZER_HPP
#define GNURADIO_LINEAR_EQUALIZER_HPP

#include <algorithm>
#include <array>
#include <complex>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/MeasurementSlot.hpp>
#include <gnuradio-4.0/algorithm/digital/AdaptiveFir.hpp>
#include <gnuradio-4.0/algorithm/digital/Constellation.hpp>

#include <gnuradio-4.0/digital/ConstellationSettings.hpp>

namespace gr::blocks::digital {

GR_REGISTER_BLOCK(gr::blocks::digital::LinearEqualizer, [T], [float])

template<std::floating_point F>
struct LinearEqualizer : Block<LinearEqualizer<F>, Resampling<1UZ, 1UZ, false>> {
    using Description = Doc<R""(
@brief A linear adaptive equalizer: least mean squares against a reference, or blind on the modulus.

One output symbol per `samples_per_symbol` inputs. At one sample per symbol the taps are symbol spaced; at two they
are fractionally spaced, spanning `num_taps` input samples and half as many symbols, which is what lets the filter
correct a timing offset a symbol-spaced equalizer can only live with.

Three rules. `lms` walks the taps against the gradient of the squared error; `nlms` is the same with the step divided
by the energy in the tap window, so one step size suits a range of input levels; `cma` needs no reference at all,
driving the output modulus toward the constellation's own `E|a|^4 / E|a|^2`. Constant-modulus adaptation converges to
the channel inverse up to an arbitrary phase rotation, which downstream carrier recovery owns.

The reference for `lms` and `nlms` is either the decision — the constellation point nearest the output — or a stated
training sequence. A training run starts where a `trigger_name` tag marks a frame, offset by the filter's own group
delay, and lasts as long as the sequence; between runs the block continues on decisions. With no trigger ever
arriving the block adapts on decisions from the start, which is degraded rather than broken, and `n_training_bursts`
says which happened.

The taps start as a unit spike at the center, so an unadapted equalizer is a delay of `(num_taps-1)/2` input samples,
and that delay is what a converged one keeps. A step too large for the channel walks the taps away from any solution;
rather than stream a growing signal the block watches the tap energy, re-initializes past a bound, and counts it in
`n_resets`.

`freeze` stops adaptation and leaves filtering running, so a converged set of taps can be held against a stretch of
data that would mislead the loop.

What the block observes about itself while it runs — `taps()`, `errorPower()`, `nResets()` and `nTrainingBursts()` —
is read by polling those methods, from any thread, and each returns a value whole rather than one being written as it
is read. They are not settings: a settings map is served from a cache refreshed when settings apply, so a value that
changes on the sample path would read as its starting value forever. A record output port carrying the same
observations, on the measurement conventions, is what will make them readable outside C++.
)"">;

    PortIn<std::complex<F>>  in;
    PortOut<std::complex<F>> out;

    Annotated<gr::Size_t, "num_taps", Visible, Doc<"odd, 1 to 127; the group delay is (num_taps-1)/2 input samples">>       num_taps           = 11U;
    Annotated<std::string, "algorithm", Visible, Doc<"'lms', 'nlms' or 'cma'">>                                             algorithm          = std::string("lms");
    Annotated<std::string, "reference", Doc<"'decision_directed' or 'training'; the modulus rule reads neither">>           reference          = std::string("decision_directed");
    Annotated<double, "step_size", Visible, Doc<"mu, in (0, 1)">>                                                           step_size          = 0.01;
    Annotated<gr::Size_t, "samples_per_symbol", Visible, Doc<"1 for symbol spacing, 2 for fractional">>                     samples_per_symbol = 1U;
    Annotated<std::vector<F>, "training_sequence", Doc<"interleaved re,im symbols a training run adapts against">>          training_sequence{};
    Annotated<bool, "freeze", Doc<"live: adaptation stops, filtering continues">>                                           freeze           = false;
    Annotated<double, "divergence_bound", Doc<"tap energy past which the taps return to the spike and n_resets counts it">> divergence_bound = 100.0;

    Annotated<std::string, "constellation", detail::ConstellationSettingsDoc, Visible> constellation = std::string("qpsk");
    Annotated<gr::Size_t, "arity", Unit<"symbols">, Doc<"M, for 'psk' and 'qam'">>     arity         = 4U;
    Annotated<F, "phase_offset", Unit<"rad">, Doc<"rotation of the 'psk' point set">>  phase_offset  = F{0};
    Annotated<gr::Size_t, "label_xor", Doc<"label offset of the 'psk' point set">>     label_xor     = 0U;
    Annotated<std::vector<F>, "points", Doc<"interleaved re,im for 'custom'">>         points{};
    Annotated<std::string, "normalization", Doc<"'power', 'amplitude' or 'none'">>     normalization = std::string("power");

    GR_MAKE_REFLECTABLE(LinearEqualizer, in, out, num_taps, algorithm, reference, step_size, samples_per_symbol, training_sequence, freeze, divergence_bound, constellation, arity, phase_offset, label_xor, points, normalization);

    /// The scalar observables, in this order, so that a reader sees the three of one instant together.
    static constexpr std::size_t kErrorPower = 0UZ;
    static constexpr std::size_t kResets     = 1UZ;
    static constexpr std::size_t kBursts     = 2UZ;

    static constexpr std::size_t kTapValues = 2UZ * gr::digital::AdaptiveFir<F>::kMaxTaps;

    gr::measurement::MeasurementSlot<3UZ>        _scalars{};
    gr::measurement::MeasurementSlot<kTapValues> _tapValues{};
    std::array<double, kTapValues>               _tapStaging{};

    gr::digital::AdaptiveFir<F>   _filter{};
    gr::digital::Constellation<F> _constellation = gr::digital::Constellation<F>::qpsk();
    std::vector<std::complex<F>>  _training{};
    std::size_t                   _sps           = 1UZ;
    bool                          _trained       = false; ///< the reference comes from the training sequence
    std::size_t                   _trainingLeft  = 0UZ;
    std::size_t                   _trainingIndex = 0UZ;
    /// Output symbols from the next one at which a training run begins. A trigger arriving late in a chunk starts a
    /// run the chunk has no room to reach, so the wait is carried rather than recomputed against each call.
    std::vector<std::size_t> _pendingStarts{};
    double                   _errorPower = 0.0;
    std::size_t              _resets     = 0UZ;
    std::size_t              _bursts     = 0UZ;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        static constexpr std::array kDesignKeys{"num_taps", "algorithm", "reference", "step_size", "samples_per_symbol", "training_sequence", "divergence_bound", "constellation", "arity", "phase_offset", "label_xor", "points", "normalization"};

        const bool designed = _filter.size() > 0UZ && !_filter.taps().empty();
        if (!designed || std::ranges::any_of(kDesignKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            rebuild();
        }
    }

    void start() { rebuild(); }

    void rebuild() {
        if (reference != "decision_directed" && reference != "training") {
            throw gr::exception(std::format("reference must be 'decision_directed' or 'training', got '{}'", reference.value));
        }
        if (samples_per_symbol < 1U || samples_per_symbol > 2U) {
            throw gr::exception(std::format("samples_per_symbol is 1 or 2, got {}", samples_per_symbol.value));
        }
        if ((training_sequence.value.size() % 2UZ) != 0UZ) {
            throw gr::exception(std::format("training_sequence is interleaved re,im and needs an even count, got {}", training_sequence.value.size()));
        }
        _constellation = detail::build<F>(constellation, arity, phase_offset, label_xor, std::span<const F>(points.value), normalization);

        const auto rule = gr::digital::adaptiveAlgorithmFrom(algorithm);
        _filter.configure(static_cast<std::size_t>(num_taps.value), rule, step_size, gr::digital::modulusReference<F>(_constellation.points()), divergence_bound);

        _training.resize(training_sequence.value.size() / 2UZ);
        for (std::size_t i = 0UZ; i < _training.size(); ++i) {
            _training[i] = std::complex<F>(training_sequence.value[2UZ * i], training_sequence.value[2UZ * i + 1UZ]);
        }
        _trained = reference == "training" && rule != gr::digital::AdaptiveAlgorithm::Cma;
        if (_trained && _training.empty()) {
            throw gr::exception("reference 'training' needs a training_sequence to adapt against");
        }
        _sps = static_cast<std::size_t>(samples_per_symbol.value);

        this->input_chunk_size  = samples_per_symbol;
        this->output_chunk_size = 1U;

        _trainingLeft  = 0UZ;
        _trainingIndex = 0UZ;
        _pendingStarts.clear();
        _errorPower = 0.0;
        _resets     = 0UZ;
        _bursts     = 0UZ;
        publishObservables();
    }

    /// @brief One symbol out per `samples_per_symbol` in, so a tag at input `t` belongs at output `t / sps`.
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        const std::size_t                                 sps = _sps;
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
                        arriving.emplace_back(static_cast<std::size_t>(relIndex) / sps, std::move(forwarded));
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

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t sps      = _sps;
        const std::size_t nSymbols = std::min(inSpan.size() / sps, outSpan.size());

        // A frame's trigger marks an input sample; the output that carries it appears the filter's group delay later.
        // Only the tags inside the region this call consumes are read: one further on is offered again next call, and
        // reading it now would start its run once per call until its samples are finally taken.
        for (const auto& [relIndex, tagMap] : inSpan.tags(nSymbols * sps)) {
            if (relIndex < 0) {
                continue;
            }
            if (tagMap.get().contains(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey()))) {
                _pendingStarts.push_back((static_cast<std::size_t>(relIndex) + _filter.groupDelay()) / sps);
            }
        }
        std::ranges::sort(_pendingStarts);
        std::size_t nextStart = 0UZ;

        for (std::size_t s = 0UZ; s < nSymbols; ++s) {
            while (nextStart < _pendingStarts.size() && _pendingStarts[nextStart] <= s) {
                if (_trained) {
                    _trainingLeft  = _training.size();
                    _trainingIndex = 0UZ;
                    ++_bursts;
                }
                ++nextStart;
            }

            for (std::size_t p = 0UZ; p < sps; ++p) {
                _filter.push(inSpan[s * sps + p]);
            }
            const std::complex<F> y = _filter.output();
            outSpan[s]              = y;

            std::complex<F> want{};
            if (_trainingLeft > 0UZ) {
                want = _training[_trainingIndex];
                ++_trainingIndex;
                --_trainingLeft;
            } else {
                want = _constellation.point(_constellation.hardDecision(y));
            }

            const double errorRe = static_cast<double>(want.real()) - static_cast<double>(y.real());
            const double errorIm = static_cast<double>(want.imag()) - static_cast<double>(y.imag());
            _errorPower += (errorRe * errorRe + errorIm * errorIm - _errorPower) * kErrorSmoothing;

            if (!freeze) {
                _resets += _filter.adapt(y, want) ? 1UZ : 0UZ;
            }
        }

        // whatever did not fire waits on, counted from the first symbol of the next call
        _pendingStarts.erase(_pendingStarts.begin(), _pendingStarts.begin() + static_cast<std::ptrdiff_t>(nextStart));
        for (std::size_t& pending : _pendingStarts) {
            pending -= nSymbols;
        }

        std::ignore = inSpan.consume(nSymbols * sps);
        outSpan.publish(nSymbols);
        publishObservables();
        return work::Status::OK;
    }

    /// @brief The current taps, interleaved re,im. Callable from any thread.
    [[nodiscard]] std::vector<F> taps() const {
        const auto [values, count] = _tapValues.read();
        std::vector<F> interleaved(static_cast<std::size_t>(count));
        for (std::size_t i = 0UZ; i < interleaved.size(); ++i) {
            interleaved[i] = static_cast<F>(values[i]);
        }
        return interleaved;
    }

    /// @brief |e|^2 through a single pole of 256 symbols. Callable from any thread.
    [[nodiscard]] double errorPower() const noexcept { return _scalars.read().first[kErrorPower]; }

    /// @brief Times the taps were returned to the spike. Callable from any thread.
    [[nodiscard]] std::uint64_t nResets() const noexcept { return static_cast<std::uint64_t>(_scalars.read().first[kResets]); }

    /// @brief Training runs a trigger tag has started. Callable from any thread.
    [[nodiscard]] std::uint64_t nTrainingBursts() const noexcept { return static_cast<std::uint64_t>(_scalars.read().first[kBursts]); }

private:
    /// One pole over 256 symbols, which is long enough to read as a level and short enough to follow a channel.
    static constexpr double kErrorSmoothing = 1.0 / 256.0;

    void publishObservables() {
        const auto current = _filter.taps();
        _tapStaging.fill(0.);
        for (std::size_t i = 0UZ; i < current.size(); ++i) {
            _tapStaging[2UZ * i]       = static_cast<double>(current[i].real());
            _tapStaging[2UZ * i + 1UZ] = static_cast<double>(current[i].imag());
        }
        _tapValues.publish(_tapStaging, static_cast<std::uint64_t>(2UZ * current.size()));
        _scalars.publish({_errorPower, static_cast<double>(_resets), static_cast<double>(_bursts)}, 1ULL);
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_LINEAR_EQUALIZER_HPP
