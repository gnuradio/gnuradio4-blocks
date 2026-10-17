#ifndef GNURADIO_OFDM_CHANNEL_EQUALIZER_HPP
#define GNURADIO_OFDM_CHANNEL_EQUALIZER_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/digital/Constellation.hpp>
#include <gnuradio-4.0/digital/ConstellationSettings.hpp>

#include <gnuradio-4.0/ofdm/Numerology.hpp>

namespace gr::blocks::ofdm {

GR_REGISTER_BLOCK(gr::blocks::ofdm::ChannelEqualizer)

namespace detail {

/// @brief The validator of `alpha`: true above 0.
inline constexpr auto kAlphaPositive = [](float value) { return value > 0.f; };

} // namespace detail

/**
 * @brief Equalizes OFDM symbol records against a known sync symbol and emits their data carriers.
 *
 * In an OFDM receiver the block follows `CpRemove` and feeds a constellation decoder. Its input is one
 * `DataSet<std::complex<float>>` record per symbol, with `fft_len` values in FFT bin order. It reads `symbol_in_frame`
 * and `frame_index` from the record's meta. A record without `symbol_in_frame` follows the previous record's position.
 * A record of another length is dropped. The first `n_sync` symbols of a frame are sync symbols and give no output
 * record. For each data symbol the block emits one record of the equalized data carriers in `data_carriers` order, with
 * the signed carrier of each value as the axis and `signal_name` as the signal name. Its meta holds `fft_len`,
 * `symbol_kind` (`equalized`), `symbol_in_frame`, `frame_index`, `cpe_rad` (the common phase the block removed),
 * `evm_db` and `frame_evm_db`. `evm_db` is the symbol's error vector magnitude against the nearest point of the
 * constellation the settings name, and `frame_evm_db` is the same measure over the frame so far. The block forwards no
 * input tag.
 *
 * The optional `channel` port emits one `DataSet<float>` per estimate. It holds the magnitude and the phase of the
 * channel estimate `H` on each occupied carrier, against the signed carrier axis, and its meta holds `fft_len`,
 * `frame_index` and `sample_start` of the sync symbol.
 *
 * At the sync symbol at `sync_index` the block takes the least-squares estimate `H[c] = Y[c] / X[c]` on every occupied
 * carrier, where `X` is `sync_word`. It equalizes each data carrier by zero forcing, `Y[c] / H[c]`. `sync_word` holds
 * one whole symbol, `fft_len` interleaved re,im pairs. Until the first estimate the block equalizes on a channel of 1
 * on every carrier.
 *
 * `tracking` selects how the pilots refine the estimate on each data symbol. The pilot values follow the
 * `pilot_symbols` cycle of `CarrierAllocator` over the data index `symbol_in_frame - n_sync`.
 *
 * - `cpe` takes the common phase error the pilots agree on,
 *   `theta_s = angle( sum_p Y_s[p] * conj(H[p] * X_s[p]) )`, and applies `e^{-j theta_s}` to the whole symbol. It
 *   removes the common part of phase noise and of a residual frequency offset. It costs one arctangent per symbol.
 * - `cpe_interp` does the same and then follows the channel itself. After the common phase is removed, each pilot
 *   gives its ratio to the current estimate, `r[p] = Y_s[p] * e^{-j theta_s} / (H[p] * X_s[p])`. The ratios are
 *   interpolated across the occupied carriers, linearly in magnitude and in unwrapped phase. Each carrier's estimate
 *   is multiplied by `1 - alpha + alpha * r[c]`, a single-pole step toward `H[c] * r[c]`. On a static channel without
 *   noise every ratio is 1, and the estimate keeps its least-squares shape between the pilots. It costs one
 *   `std::polar` per carrier.
 * - `none` equalizes on the sync symbol's estimate alone.
 *
 * `cpe` and `cpe_interp` need at least one pilot carrier. `fft_len` is a power of two from 8 to 65536, the carrier
 * lists name distinct carriers in range, and, once the block holds a layout, `data_carriers` names at least one. Each
 * pilot value is finite and nonzero, and a layout with pilot carriers needs at least one pilot value. `sync_word` is
 * finite and nonzero on every occupied carrier, and `sync_index` is below `n_sync`. `tracking` is `cpe`, `cpe_interp`
 * or `none`. The constellation settings name a valid point set. The block refuses a change that breaks one of these
 * rules. Settings that meet the rules only together go in one application.
 *
 * A change of `fft_len`, `data_carriers`, `pilot_carriers` or `sync_word` resets the estimate to 1 on every carrier
 * until the next sync symbol. A change of any other setting keeps the estimate and applies from the next record.
 *
 * The default `data_carriers` is empty. Until a change names data carriers, `processBulk()` reports an error and
 * returns `ERROR`. Until then the block takes `pilot_carriers` and `tracking` unchecked against each other and checks
 * `pilot_symbols` and `sync_word` for an even length only. The change that names the data carriers checks them all.
 *
 * `nEstimates()` counts the estimates, one per frame. `nSkipped()` counts the sync symbols other than the one at
 * `sync_index`. `nUntrained()` counts the data symbols equalized while no estimate was in force. `nMalformed()` counts
 * the dropped records. `start()` sets the counters to 0 and resets the estimate.
 */
struct ChannelEqualizer : Block<ChannelEqualizer, NoTagPropagation> {
    using Description = Doc<"Equalizes OFDM symbol records against the frame's known sync symbol and emits the data carriers in data_carriers order. The block estimates the channel by least squares, tracks the common phase error on the pilots with an optional interpolated per-carrier update, and equalizes by zero forcing. Records carry cpe_rad, evm_db and frame_evm_db, and the optional channel port carries the estimate">;

    PortIn<DataSet<Complex>, Async>          in;
    PortOut<DataSet<Complex>, Async>         out;
    PortOut<DataSet<float>, Optional, Async> channel;

    Annotated<gr::Size_t, "fft_len", Visible, Doc<"bins per record, a power of two">, detail::FftLengthLimits>                                         fft_len = 64U;
    Annotated<std::vector<std::int32_t>, "data_carriers", Visible, Doc<"signed logical carrier indices the output holds, in order">>                   data_carriers{};
    Annotated<std::vector<std::int32_t>, "pilot_carriers", Visible, Doc<"signed logical carrier indices the tracking reads">>                          pilot_carriers{};
    Annotated<std::vector<float>, "pilot_symbols", Doc<"interleaved re,im pilot cycle of nonzero values">>                                             pilot_symbols{};
    Annotated<std::vector<float>, "sync_word", Doc<"interleaved re,im known symbol of fft_len carriers">>                                              sync_word{};
    Annotated<gr::Size_t, "n_sync", Visible, Doc<"sync symbols at a frame's head">>                                                                    n_sync      = 1U;
    Annotated<gr::Size_t, "sync_index", Doc<"index of the known symbol among the frame's sync symbols">>                                               sync_index  = 0U;
    Annotated<std::string, "tracking", Visible, Doc<"phase tracking mode: 'cpe', 'cpe_interp' or 'none'">>                                             tracking    = std::string("cpe");
    Annotated<float, "alpha", Visible, Doc<"single-pole coefficient of the 'cpe_interp' update, in (0, 1]">, Limits<0.f, 1.f, detail::kAlphaPositive>> alpha       = 0.1f;
    Annotated<std::string, "signal_name", Doc<"the emitted record's signal name">>                                                                     signal_name = std::string("ofdm_equalized");

    Annotated<std::string, "constellation", gr::blocks::digital::ConstellationSettingsDoc, Visible> constellation = std::string("qpsk");
    Annotated<gr::Size_t, "arity", Unit<"symbols">, Doc<"M, read by 'psk' and 'qam'">>              arity         = 4U;
    Annotated<float, "phase_offset", Unit<"rad">, Doc<"rotation of the 'psk' point set">>           phase_offset  = 0.f;
    Annotated<gr::Size_t, "label_xor", Doc<"label offset of the 'psk' point set">>                  label_xor     = 0U;
    Annotated<std::vector<float>, "points", Doc<"interleaved re,im points of 'custom'">>            points{};
    Annotated<std::string, "normalization", Doc<"point scaling: 'power', 'amplitude' or 'none'">>   normalization = std::string("power");

    GR_MAKE_REFLECTABLE(ChannelEqualizer, in, out, channel, fft_len, data_carriers, pilot_carriers, pilot_symbols, sync_word, n_sync, sync_index, tracking, alpha, signal_name, constellation, arity, phase_offset, label_xor, points, normalization);

    /// The two pilots around an occupied carrier and the carrier's interpolation weight between them. The table
    /// depends on the numerology alone.
    struct Between {
        std::size_t left{0UZ};  ///< pilot slot at or below this carrier
        std::size_t right{0UZ}; ///< pilot slot at or above it
        float       weight{0.f};
    };

    /// The settings the block computes on.
    struct Accepted {
        gr::Size_t                fftLength = 0U;
        std::vector<std::int32_t> dataCarriers{};
        std::vector<std::int32_t> pilotCarriers{};
        std::vector<float>        pilotSymbols{};
        std::vector<float>        syncWord{};
        std::uint64_t             nSync     = 0ULL;
        std::uint64_t             syncIndex = 0ULL;
        std::string               tracking{};
        float                     alpha = 0.f;
        std::string               signalName{};
        std::string               constellation{};
        gr::Size_t                arity       = 0U;
        float                     phaseOffset = 0.f;
        gr::Size_t                labelXor    = 0U;
        std::vector<float>        points{};
        std::string               normalization{};
    };

    Accepted                            _accepted = settingsAfter(Accepted{}, [](std::string_view) { return true; });
    std::optional<gr::ofdm::CarrierMap> _map{};
    gr::digital::Constellation<float>   _constellation = gr::digital::Constellation<float>::qpsk();

    std::vector<Complex>      _pilots{};    ///< `pilot_symbols` as complex values
    std::vector<Complex>      _sync{};      ///< the known symbol, whole
    std::vector<Complex>      _h{};         ///< the channel estimate, per bin
    std::vector<Complex>      _hInv{};      ///< its reciprocal
    std::vector<Complex>      _corrected{}; ///< one symbol after the common phase is taken out
    std::vector<Complex>      _values{};    ///< the data carriers of one output record
    std::vector<std::size_t>  _occupied{};  ///< occupied bins, ascending in signed carrier index
    std::vector<Between>      _between{};   ///< one entry per `_occupied` bin
    std::vector<int>          _dataAxis{};  ///< the signed carrier of each output value
    std::vector<int>          _occupiedAxis{};
    std::vector<float>        _pilotMagnitude{}; ///< |r| at each pilot, taken once per symbol instead of once per carrier
    std::vector<float>        _pilotAngle{};
    detail::SymbolRecordShape _shape{}; ///< the invariant part of a record

    bool          _interp        = false;
    bool          _track         = true;
    bool          _trained       = false;
    std::uint64_t _symbolInFrame = 0ULL; ///< the position of a record that carries no `symbol_in_frame`
    double        _frameError    = 0.;
    double        _frameSignal   = 0.;
    std::uint64_t _untrained     = 0ULL; ///< data symbols equalized with no estimate in force
    std::uint64_t _skipped       = 0ULL; ///< sync symbols other than the known one
    std::uint64_t _estimates     = 0ULL;
    std::uint64_t _malformed     = 0ULL; ///< records whose length is not `fft_len`

    /// @brief Data symbols equalized while no channel estimate is in force, on the identity channel.
    [[nodiscard]] std::uint64_t nUntrained() const noexcept { return _untrained; }
    /// @brief Sync symbols other than the one at `sync_index`.
    [[nodiscard]] std::uint64_t nSkipped() const noexcept { return _skipped; }
    /// @brief Least-squares estimates taken over the run, one per frame.
    [[nodiscard]] std::uint64_t nEstimates() const noexcept { return _estimates; }
    /// @brief Records whose length is not `fft_len`. The block drops them.
    [[nodiscard]] std::uint64_t nMalformed() const noexcept { return _malformed; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        take(settingsAfter(_accepted, [&newSettings](std::string_view key) { return newSettings.contains(key); }));
    }

    void start() { resetState(); }

    /// @brief Resets the channel estimate, the frame's position and error sums, and the counters.
    void resetState() {
        resetEstimate();
        _symbolInFrame = 0ULL;
        _frameError    = 0.;
        _frameSignal   = 0.;
        _untrained     = 0ULL;
        _skipped       = 0ULL;
        _estimates     = 0ULL;
        _malformed     = 0ULL;
    }

    /// @brief One record in, at most one record out, taking nothing the output has no room for.
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& channelSpan) {
        if (!_map.has_value()) {
            this->emitErrorMessage("processBulk", gr::Error("gr::blocks::ofdm::ChannelEqualizer: the block holds no numerology. Its default settings name no data carriers, and no settings application has built one"));
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            channelSpan.publish(0UZ);
            return work::Status::ERROR;
        }
        const bool        wantEstimate = channelSpan.isConnected;
        const std::size_t fftLength    = _map->fftLength();
        std::size_t       consumed     = 0UZ;
        std::size_t       made         = 0UZ;
        std::size_t       estimates    = 0UZ;

        while (consumed < inSpan.size()) {
            const DataSet<Complex>& record     = inSpan[consumed];
            const std::uint64_t     position   = detail::metaCount(record, detail::kSymbolInFrame, _symbolInFrame);
            const bool              isSync     = position < _accepted.nSync;
            const bool              wellFormed = record.signal_values.size() == fftLength;
            if (wellFormed && !isSync && made == outSpan.size()) {
                break; // no room for its equalized record, which leaves it on the input
            }
            if (wellFormed && isSync && position == _accepted.syncIndex && wantEstimate && estimates == channelSpan.size()) {
                break; // no room for its channel record
            }

            if (position == 0ULL) {
                _frameError  = 0.;
                _frameSignal = 0.;
            }
            if (!wellFormed) {
                ++_malformed;
            } else if (isSync) {
                if (position == _accepted.syncIndex) {
                    estimate(record);
                    if (wantEstimate) {
                        channelSpan[estimates] = channelRecord(record);
                        ++estimates;
                    }
                } else {
                    ++_skipped;
                }
            } else {
                equalize(outSpan[made], record, position - _accepted.nSync);
                ++made;
            }
            _symbolInFrame = position + 1ULL;
            ++consumed;
        }

        outSpan.publish(made);
        channelSpan.publish(estimates);
        std::ignore = inSpan.consume(consumed);
        if (consumed == 0UZ) {
            return inSpan.size() == 0UZ ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    /// @brief Copies the named members into @p next.
    template<typename Moved>
    [[nodiscard]] Accepted settingsAfter(Accepted next, Moved moved) const {
        if (moved("fft_len")) {
            next.fftLength = fft_len.value;
        }
        if (moved("data_carriers")) {
            next.dataCarriers = data_carriers.value;
        }
        if (moved("pilot_carriers")) {
            next.pilotCarriers = pilot_carriers.value;
        }
        if (moved("pilot_symbols")) {
            next.pilotSymbols = pilot_symbols.value;
        }
        if (moved("sync_word")) {
            next.syncWord = sync_word.value;
        }
        if (moved("n_sync")) {
            next.nSync = n_sync.value;
        }
        if (moved("sync_index")) {
            next.syncIndex = sync_index.value;
        }
        if (moved("tracking")) {
            next.tracking = tracking.value;
        }
        if (moved("alpha")) {
            next.alpha = alpha.value;
        }
        if (moved("signal_name")) {
            next.signalName = signal_name.value;
        }
        if (moved("constellation")) {
            next.constellation = constellation.value;
        }
        if (moved("arity")) {
            next.arity = arity.value;
        }
        if (moved("phase_offset")) {
            next.phaseOffset = phase_offset.value;
        }
        if (moved("label_xor")) {
            next.labelXor = label_xor.value;
        }
        if (moved("points")) {
            next.points = points.value;
        }
        if (moved("normalization")) {
            next.normalization = normalization.value;
        }
        return next;
    }

    /// @brief Checks @p next, then rebuilds the layout and resets the estimate where they change. With no layout and
    /// no data carriers in @p next, the block builds none.
    void take(Accepted next) {
        const bool                          newNumerology = next.fftLength != _accepted.fftLength || next.dataCarriers != _accepted.dataCarriers || next.pilotCarriers != _accepted.pilotCarriers;
        std::optional<gr::ofdm::CarrierMap> built{};
        if (newNumerology && (_map.has_value() || !next.dataCarriers.empty())) {
            built.emplace(detail::buildMap(next.fftLength, std::span<const std::int32_t>(next.dataCarriers), std::span<const std::int32_t>(next.pilotCarriers)));
        }
        const gr::ofdm::CarrierMap* map = built.has_value() ? &*built : (_map.has_value() ? &*_map : nullptr);

        std::vector<Complex> pilots = detail::complexFrom(std::span<const float>(next.pilotSymbols), "pilot_symbols");
        std::vector<Complex> known  = detail::complexFrom(std::span<const float>(next.syncWord), "sync_word");
        if (next.syncIndex >= next.nSync) {
            throw gr::exception(std::format("sync_index is {} and n_sync is {}: the known symbol has to be one of the frame's sync symbols", next.syncIndex, next.nSync));
        }
        if (next.tracking != "cpe" && next.tracking != "cpe_interp" && next.tracking != "none") {
            throw gr::exception(std::format("tracking must be 'cpe', 'cpe_interp' or 'none', got '{}'", next.tracking));
        }
        gr::digital::Constellation<float> decisions = gr::blocks::digital::constellationFromSettings<float>(next.constellation, next.arity, next.phaseOffset, next.labelXor, std::span<const float>(next.points), next.normalization);
        if (map != nullptr) {
            requireNumerology(*map, next, std::span<const Complex>(pilots), std::span<const Complex>(known));
        }
        const bool newEstimate = newNumerology || !sameValues(next.syncWord, _accepted.syncWord);

        // Every check has passed. The block changes from here.
        if (built.has_value()) {
            const std::size_t fftLength = built->fftLength();
            _dataAxis.assign(built->dataCarriers().begin(), built->dataCarriers().end());
            buildInterpolation(*built);
            _map.emplace(std::move(*built));
            _h.assign(fftLength, Complex{1.f, 0.f});
            _hInv.assign(fftLength, Complex{1.f, 0.f});
            _corrected.assign(fftLength, Complex{});
            _values.assign(_map->nData(), Complex{});
            _pilotMagnitude.assign(_map->nPilots(), 1.f);
            _pilotAngle.assign(_map->nPilots(), 0.f);
        }
        _pilots        = std::move(pilots);
        _sync          = std::move(known);
        _constellation = std::move(decisions);
        _accepted      = std::move(next);
        _interp        = _accepted.tracking == "cpe_interp";
        _track         = _accepted.tracking != "none";
        if (_map.has_value()) {
            _shape.build(std::span<const int>(_dataAxis), _accepted.signalName, property_map{{std::pmr::string(detail::kFftLenKey), pmt::Value(std::uint64_t{_map->fftLength()})}});
        }
        if (newEstimate) {
            resetEstimate();
        }
    }

    /// @brief Checks the pilots, the sync word and the tracking mode of @p next against the numerology @p map.
    static void requireNumerology(const gr::ofdm::CarrierMap& map, const Accepted& next, std::span<const Complex> pilots, std::span<const Complex> known) {
        detail::requireValid(map.validatePilotCycle(pilots));
        detail::requireValid(map.validateSyncWord(known));
        if (next.tracking != "none" && map.nPilots() == 0UZ) {
            throw gr::exception(std::format("tracking is '{}' but the numerology has no pilot carriers to track on", next.tracking));
        }
        const std::size_t fftLength = map.fftLength();
        for (const std::size_t bin : map.dataBins()) {
            requireKnown(known[bin], "data", fftLength, bin);
        }
        for (const std::size_t bin : map.pilotBins()) {
            requireKnown(known[bin], "pilot", fftLength, bin);
        }
    }

    /// @brief Refuses a zero or non-finite known value on the occupied carrier in @p bin. Least squares cannot estimate
    /// the channel against it. @p role names the carrier's role in the message.
    static void requireKnown(Complex value, std::string_view role, std::size_t fftLength, std::size_t bin) {
        if (value == Complex{}) {
            throw gr::exception(std::format("sync_word carries nothing on {} carrier {}, so the channel cannot be estimated there", role, detail::carrierAt(fftLength, bin)));
        }
        if (!std::isfinite(value.real()) || !std::isfinite(value.imag())) {
            throw gr::exception(std::format("sync_word carries ({}, {}) on {} carrier {}, and the channel can be estimated only against a finite value", value.real(), value.imag(), role, detail::carrierAt(fftLength, bin)));
        }
    }

    /// @brief True when @p a and @p b hold the same values. Two NaN values are the same value. `sync_word` can hold a
    /// NaN on a guard bin.
    [[nodiscard]] static bool sameValues(std::span<const float> a, std::span<const float> b) noexcept {
        return std::ranges::equal(a, b, [](float x, float y) { return x == y || (std::isnan(x) && std::isnan(y)); });
    }

    /// @brief Builds the table of the pilots around each occupied carrier and its weight between them.
    void buildInterpolation(const gr::ofdm::CarrierMap& map) {
        const std::size_t fftLength = map.fftLength();
        std::vector<int>  carriers(map.dataCarriers().begin(), map.dataCarriers().end());
        carriers.insert(carriers.end(), map.pilotCarriers().begin(), map.pilotCarriers().end());
        std::ranges::sort(carriers);

        std::vector<int>         pilotOrder(map.pilotCarriers().begin(), map.pilotCarriers().end());
        std::vector<std::size_t> slotOf(pilotOrder.size());
        for (std::size_t p = 0UZ; p < pilotOrder.size(); ++p) {
            slotOf[p] = p;
        }
        std::ranges::sort(slotOf, [&pilotOrder](std::size_t a, std::size_t b) { return pilotOrder[a] < pilotOrder[b]; });

        _occupied.clear();
        _occupiedAxis.clear();
        _between.clear();
        _occupied.reserve(carriers.size());
        _between.reserve(carriers.size());
        for (const int carrier : carriers) {
            _occupied.push_back(detail::binAt(fftLength, carrier));
            _occupiedAxis.push_back(carrier);

            Between where{};
            if (slotOf.empty()) {
                _between.push_back(where);
                continue;
            }
            std::size_t upper = 0UZ;
            while (upper < slotOf.size() && pilotOrder[slotOf[upper]] < carrier) {
                ++upper;
            }
            if (upper == 0UZ) { // below every pilot, holding the nearest pilot's value
                where = Between{slotOf.front(), slotOf.front(), 0.f};
            } else if (upper == slotOf.size()) {
                where = Between{slotOf.back(), slotOf.back(), 0.f};
            } else {
                const int low  = pilotOrder[slotOf[upper - 1UZ]];
                const int high = pilotOrder[slotOf[upper]];
                where          = Between{slotOf[upper - 1UZ], slotOf[upper], static_cast<float>(carrier - low) / static_cast<float>(high - low)};
            }
            _between.push_back(where);
        }
    }

    /// @brief Puts the identity channel in place of the estimate until the next sync symbol.
    void resetEstimate() {
        std::ranges::fill(_h, Complex{1.f, 0.f});
        std::ranges::fill(_hInv, Complex{1.f, 0.f});
        _trained = false;
    }

    /// @brief Least squares against the known symbol. Divides once and keeps the reciprocal.
    void estimate(const DataSet<Complex>& record) {
        const Complex* observed = record.signal_values.data();
        const Complex* known    = _sync.data();
        for (const std::size_t bin : _occupied) {
            const Complex h = detail::divide(observed[bin], known[bin]);
            _h[bin]         = h;
            _hInv[bin]      = detail::reciprocal(h);
        }
        _trained = true;
        ++_estimates;
    }

    /// @brief The pilot value symbol @p dataIndex carries in slot @p slot, by the two-level cycle.
    [[nodiscard]] Complex pilotOf(std::uint64_t dataIndex, std::size_t slot) const noexcept { return _pilots[gr::ofdm::CarrierMap::pilotSymbolIndex(dataIndex, slot, _map->nPilots(), _pilots.size())]; }

    void equalize(DataSet<Complex>& target, const DataSet<Complex>& record, std::uint64_t dataIndex) {
        if (!_trained) {
            ++_untrained;
        }
        const Complex*     observed  = record.signal_values.data();
        const std::size_t* pilotBins = _map->pilotBins().data();
        const std::size_t  nPilots   = _map->nPilots();

        double theta = 0.;
        if (_track) {
            std::complex<double> sum{};
            for (std::size_t p = 0UZ; p < nPilots; ++p) {
                const std::size_t bin      = pilotBins[p];
                const Complex     expected = detail::multiply(_h[bin], pilotOf(dataIndex, p));
                sum += std::complex<double>(observed[bin]) * std::conj(std::complex<double>(expected));
            }
            theta = std::atan2(sum.imag(), sum.real());
        }
        const Complex rotate(static_cast<float>(std::cos(-theta)), static_cast<float>(std::sin(-theta)));

        for (const std::size_t bin : _occupied) {
            _corrected[bin] = detail::multiply(observed[bin], rotate);
        }
        if (_interp) {
            interpolate(dataIndex);
        }

        const std::size_t* dataBins = _map->dataBins().data();
        const std::size_t  nData    = _map->nData();
        const Complex*     inverse  = _hInv.data();
        Complex*           values   = _values.data();
        double             error    = 0.;
        double             signal   = 0.;
        for (std::size_t k = 0UZ; k < nData; ++k) {
            const std::size_t bin = dataBins[k];
            const Complex     z   = detail::multiply(_corrected[bin], inverse[bin]);
            values[k]             = z;
            const Complex point   = _constellation.point(_constellation.hardDecision(z));
            error += static_cast<double>(std::norm(z - point));
            signal += static_cast<double>(std::norm(point));
        }
        _frameError += error;
        _frameSignal += signal;

        const double symbolEvm = signal > 0. ? 10. * std::log10(std::max(error / signal, 1e-30)) : 0.;
        const double frameEvm  = _frameSignal > 0. ? 10. * std::log10(std::max(_frameError / _frameSignal, 1e-30)) : 0.;

        property_map& meta                             = _shape.emitInto(target, std::span<const Complex>(_values), detail::kKindEqualized);
        meta[std::pmr::string(detail::kSymbolInFrame)] = pmt::Value(dataIndex + _accepted.nSync);
        meta[std::pmr::string(detail::kCpeRadKey)]     = pmt::Value(static_cast<float>(theta));
        meta[std::pmr::string(detail::kEvmDbKey)]      = pmt::Value(static_cast<float>(symbolEvm));
        meta[std::pmr::string(detail::kFrameEvmDbKey)] = pmt::Value(static_cast<float>(frameEvm));
        meta[std::pmr::string(detail::kFrameIndexKey)] = pmt::Value(detail::metaCount(record, detail::kFrameIndexKey, 0ULL));
    }

    /**
     * @brief Updates the channel estimate between sync words from the pilots, interpolated across the carriers and
     * smoothed.
     *
     * `_corrected` has the common phase removed. Magnitude and phase of the ratios are interpolated separately. A
     * linear interpolation of two complex values shrinks toward zero where their phases differ. A timing drift turns
     * the phase linearly across the carriers, and the phase interpolation follows it.
     */
    void interpolate(std::uint64_t dataIndex) {
        const std::size_t* pilotBins = _map->pilotBins().data();
        const std::size_t  nPilots   = _map->nPilots();
        for (std::size_t p = 0UZ; p < nPilots; ++p) {
            const std::size_t bin   = pilotBins[p];
            const Complex     ratio = detail::divide(_corrected[bin], detail::multiply(_h[bin], pilotOf(dataIndex, p)));
            _pilotMagnitude[p]      = std::abs(ratio);
            _pilotAngle[p]          = std::arg(ratio);
        }

        const float mix = _accepted.alpha;
        for (std::size_t k = 0UZ; k < _occupied.size(); ++k) {
            const Between& where    = _between[k];
            const float    lowMag   = _pilotMagnitude[where.left];
            const float    lowPhase = _pilotAngle[where.left];

            const float magnitude = lowMag + where.weight * (_pilotMagnitude[where.right] - lowMag);
            // the shorter way round, across the branch cut where the pair straddles it
            const float step  = std::remainder(_pilotAngle[where.right] - lowPhase, 2.f * std::numbers::pi_v<float>);
            const float phase = lowPhase + where.weight * step;

            const Complex     ratio = std::polar(magnitude, phase);
            const Complex     factor(1.f - mix + mix * ratio.real(), mix * ratio.imag());
            const std::size_t bin = _occupied[k];
            _h[bin]               = detail::multiply(_h[bin], factor);
            _hInv[bin]            = detail::reciprocal(_h[bin]);
        }
    }

    /// @brief The channel estimate as a measurement record, with magnitude and phase against the carrier axis.
    [[nodiscard]] DataSet<float> channelRecord(const DataSet<Complex>& source) const {
        const std::size_t count = _occupied.size();
        DataSet<float>    ds;

        ds.extents = {static_cast<std::int32_t>(count)};
        ds.layout  = gr::LayoutRight{};

        ds.axis_names = {"Carrier"};
        ds.axis_units = {"1"};
        ds.axis_values.resize(1UZ);
        ds.axis_values[0UZ].resize(count);
        for (std::size_t k = 0UZ; k < count; ++k) {
            ds.axis_values[0UZ][k] = static_cast<float>(_occupiedAxis[k]);
        }

        ds.signal_names      = {"ChannelMagnitude", "ChannelPhase"};
        ds.signal_quantities = {"Magnitude", "Phase"};
        ds.signal_units      = {"1", "rad"};
        ds.signal_values.resize(2UZ * count);
        for (std::size_t k = 0UZ; k < count; ++k) {
            ds.signal_values[k]         = std::abs(_h[_occupied[k]]);
            ds.signal_values[count + k] = std::arg(_h[_occupied[k]]);
        }
        ds.signal_ranges.resize(2UZ);
        for (std::size_t s = 0UZ; s < 2UZ; ++s) {
            const auto range    = std::minmax_element(ds.signal_values.begin() + static_cast<std::ptrdiff_t>(s * count), ds.signal_values.begin() + static_cast<std::ptrdiff_t>((s + 1UZ) * count));
            ds.signal_ranges[s] = {*range.first, *range.second};
        }

        const property_map meta{
            {std::pmr::string(detail::kFftLenKey), pmt::Value(std::uint64_t{_map->fftLength()})},
            {std::pmr::string(detail::kFrameIndexKey), pmt::Value(detail::metaCount(source, detail::kFrameIndexKey, 0ULL))},
            {std::pmr::string(detail::kSampleStartKey), pmt::Value(detail::metaCount(source, detail::kSampleStartKey, 0ULL))},
            {std::pmr::string("n_averaged"), pmt::Value(std::uint64_t{1})},
            {std::pmr::string("estimator"), pmt::Value(std::string("least_squares"))},
        };
        ds.meta_information.assign(2UZ, meta);
        ds.timing_events.resize(2UZ);
        ds.timestamp = 0;
        return ds;
    }
};

} // namespace gr::blocks::ofdm

#endif // GNURADIO_OFDM_CHANNEL_EQUALIZER_HPP
