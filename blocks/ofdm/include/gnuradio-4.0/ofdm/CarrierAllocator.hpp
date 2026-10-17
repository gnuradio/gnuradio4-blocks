#ifndef GNURADIO_OFDM_CARRIER_ALLOCATOR_HPP
#define GNURADIO_OFDM_CARRIER_ALLOCATOR_HPP

#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/ofdm/Numerology.hpp>

namespace gr::blocks::ofdm {

GR_REGISTER_BLOCK(gr::blocks::ofdm::CarrierAllocator)

/**
 * @brief Places a stream of data symbols onto OFDM symbol records, with pilots and sync words.
 *
 * The block opens the frequency-domain part of an OFDM transmitter. It follows a constellation mapper and feeds
 * `CpInsert`. Its input is the mapper's complex points, one per data carrier. Its output is one record per OFDM symbol,
 * a `DataSet<std::complex<float>>` of `fft_len` values in FFT bin order. The record's axis holds the signed carrier
 * index of each value, and its signal name is `signal_name`. Its meta holds `fft_len`, `symbol_kind` (`sync` or
 * `data`), `symbol_index` (the record's index in the stream), `frame_index`, `symbol_in_frame` (sync words included),
 * `sample_start` (the input index of the first sample of the data symbol the record holds or precedes, and on a
 * symbol of zero fill at end of stream the previous record's value) and `pad_carriers`. The block forwards no input
 * tag.
 *
 * `data_carriers` and `pilot_carriers` name carriers by signed index, from `-fft_len/2` to `fft_len/2 - 1`. Each data
 * symbol fills the data carriers with the next input samples, in list order. `pilot_symbols` and `sync_words` hold
 * interleaved re,im pairs. Pilot slot `p` of data symbol `s` takes entry `(s * n_pilots + p) % len` of the
 * `pilot_symbols` cycle, and a cycle of `n_pilots` values puts the same pilots on every symbol. `s` counts the data
 * symbols of the frame. Without framing it counts those since the start, the last reset or the change that ended
 * framing. Every other carrier is a guard and holds zero.
 *
 * With `frame_len` above 0, a frame holds `frame_len` data symbols and opens with the symbols of `sync_words`, emitted
 * as given. The block emits a frame's sync words when the frame's first data symbol is complete.
 * `gr::ofdm::schmidlCoxPreamble` builds a sync word that `SchmidlCoxSync` detects. With `frame_len` 0 the stream is
 * unframed, and `sync_words` is empty.
 *
 * At end of stream the block zero-fills the last partial symbol and adds zero symbols to complete an open frame.
 * `pad_carriers` holds the zero-filled data carriers of each record. These records take the room the output has at that
 * point, and a frame that needs more records ends short.
 *
 * `fft_len` is a power of two from 8 to 65536. The carrier lists name distinct carriers in range, and, once the block
 * holds a layout, `data_carriers` names at least one. Every pilot value is finite and nonzero, and a layout with pilot
 * carriers needs at least one pilot value. Every sync-word value is finite, and `sync_words` holds a whole number of
 * symbols. `sync_words` is empty while `frame_len` is 0. The block refuses a value that breaks one of these rules. Two
 * settings that meet the rules only together go in one application.
 *
 * A change of `fft_len`, `data_carriers` or `pilot_carriers` ends the open frame without padding. The data samples not
 * yet placed stay when they fit one symbol of the new layout, and the block drops them otherwise. A change of
 * `pilot_symbols` applies from the next data symbol, and a change of `sync_words` from the next sync word. A change of
 * `frame_len` sets the length of the open frame, and a frame that already holds that many data symbols ends at once. A
 * change from 0 opens a new frame, and a change to 0 ends the open one. A change of `signal_name` applies from the next
 * record. A reset to defaults ends the open frame, drops the data samples not yet placed and restarts the indices at 0.
 *
 * The default `data_carriers` is empty. Until a change names data carriers, `processBulk()` and `processEpilogue()`
 * report an error and return `ERROR`. Until then the block takes `pilot_carriers` unchecked and checks `pilot_symbols`
 * for an even length only. The change that names the data carriers checks both.
 *
 * `nPadded()` counts the zero-filled data carriers. `nTruncated()` counts the records a final frame lacked because the
 * output had no room for them. `nDropped()` counts the data samples that a change of layout or a reset dropped.
 * `start()` sets the counters and the indices to 0.
 */
struct CarrierAllocator : Block<CarrierAllocator, NoTagPropagation> {
    using Description = Doc<"Places a stream of complex data symbols onto OFDM symbol records of fft_len values in FFT bin order. The block fills the data carriers in list order and the pilot carriers from the pilot_symbols cycle. With frame_len above 0, the sync_words open each frame. At end of stream the block zero-fills the open symbol and frame">;

    PortIn<Complex>                  in;
    PortOut<DataSet<Complex>, Async> out;

    Annotated<gr::Size_t, "fft_len", Visible, Doc<"bins per record, a power of two">, detail::FftLengthLimits>                            fft_len = 64U;
    Annotated<std::vector<std::int32_t>, "data_carriers", Visible, Doc<"signed logical carrier indices the data stream fills, in order">> data_carriers{};
    Annotated<std::vector<std::int32_t>, "pilot_carriers", Visible, Doc<"signed logical carrier indices the pilot cycle fills">>          pilot_carriers{};
    Annotated<std::vector<float>, "pilot_symbols", Doc<"interleaved re,im pilot cycle of nonzero values">>                                pilot_symbols{};
    Annotated<std::vector<float>, "sync_words", Doc<"interleaved re,im, a whole number of fft_len-carrier symbols, emitted verbatim">>    sync_words{};
    Annotated<gr::Size_t, "frame_len", Visible, Doc<"data symbols per frame; 0 means unframed">>                                          frame_len   = 0U;
    Annotated<std::string, "signal_name", Doc<"the emitted record's signal name">>                                                        signal_name = std::string("ofdm_symbol");

    GR_MAKE_REFLECTABLE(CarrierAllocator, in, out, fft_len, data_carriers, pilot_carriers, pilot_symbols, sync_words, frame_len, signal_name);

    /// The settings the block computes on.
    struct Accepted {
        gr::Size_t                fftLength = 0U;
        std::vector<std::int32_t> dataCarriers{};
        std::vector<std::int32_t> pilotCarriers{};
        std::vector<float>        pilotSymbols{};
        std::vector<float>        syncWords{};
        gr::Size_t                frameLength = 0U;
        std::string               signalName{};
    };

    Accepted                            _accepted = settingsAfter(Accepted{}, [](std::string_view) { return true; });
    std::optional<gr::ofdm::CarrierMap> _map{};
    std::vector<Complex>                _pilots{};      ///< `pilot_symbols` as complex values
    std::vector<std::vector<Complex>>   _syncWords{};   ///< the sync words, one symbol each
    std::vector<int>                    _binCarriers{}; ///< the signed carrier each bin holds, which is the record's axis
    std::vector<Complex>                _pending{};     ///< data-carrier symbols not yet placed in a symbol
    std::vector<Complex>                _symbol{};      ///< one symbol under construction, reused across records
    detail::SymbolRecordShape           _shape{};       ///< the invariant part of a record

    std::size_t   _nData         = 0UZ;
    std::size_t   _frameLength   = 0UZ;  ///< the accepted `frame_len`
    std::size_t   _syncLeft      = 0UZ;  ///< sync words still owed to the frame being opened
    std::uint64_t _symbolIndex   = 0ULL; ///< records emitted since the last `reset()`
    std::uint64_t _frameIndex    = 0ULL;
    std::uint64_t _symbolInFrame = 0ULL; ///< the next record's position in its frame, sync words included
    std::size_t   _dataInFrame   = 0UZ;  ///< data symbols emitted into the open frame
    std::uint64_t _pilotPhase    = 0ULL; ///< the `s` of the pilot cycle, within the frame when framed, and since the start, the end of framing or a reset when not
    std::uint64_t _streamAt      = 0ULL; ///< absolute input index of `_pending`'s first sample
    std::uint64_t _padded        = 0ULL; ///< carriers zero-filled at end of stream, cumulative
    std::uint64_t _truncated     = 0ULL; ///< records the final frame lacked for want of output room
    std::uint64_t _dropped       = 0ULL; ///< data samples dropped by changes of numerology and by resets
    std::size_t   _padThisSymbol = 0UZ;
    bool          _flushed       = false;

    /// @brief The data-carrier slots zero-filled at end of stream, over the whole run.
    [[nodiscard]] std::uint64_t nPadded() const noexcept { return _padded; }

    /// @brief The records the final frame lacked because the output had no room for them at end of stream.
    [[nodiscard]] std::uint64_t nTruncated() const noexcept { return _truncated; }

    /// @brief The data samples dropped by changes of numerology and by resets to defaults, over the whole run.
    [[nodiscard]] std::uint64_t nDropped() const noexcept { return _dropped; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        take(settingsAfter(_accepted, [&newSettings](std::string_view key) { return newSettings.contains(key); }));
    }

    /// @brief Restarts the indices and the open frame. Drops the data samples not yet placed and counts them in
    /// `nDropped()`.
    void reset() {
        _dropped += _pending.size();
        _pending.clear();
        _syncLeft      = _syncWords.size();
        _symbolIndex   = 0ULL;
        _frameIndex    = 0ULL;
        _symbolInFrame = 0ULL;
        _dataInFrame   = 0UZ;
        _pilotPhase    = 0ULL;
        _streamAt      = 0ULL;
        _padThisSymbol = 0UZ;
        _flushed       = false;
    }

    /// @brief Restarts the indices, the open frame and the counters.
    void start() {
        reset();
        _padded    = 0ULL;
        _truncated = 0ULL;
        _dropped   = 0ULL;
    }

    /**
     * @brief Folds whole data symbols out of the stream.
     *
     * The sample that completes a symbol stays in the input until the output has taken every record of that symbol.
     */
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_map.has_value()) {
            return reportNoNumerology("processBulk", inSpan, outSpan);
        }
        Progress tally = fold(inSpan, inSpan.size(), outSpan);
        if (tally.outputFull && tally.taken > 0UZ && _pending.size() == _nData) {
            _pending.pop_back(); // the next call takes this sample again
            --tally.taken;
        }
        outSpan.publish(tally.made);
        std::ignore = inSpan.consume(tally.taken);
        if (tally.outputFull && tally.made == 0UZ && tally.taken == 0UZ) {
            return work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        return work::Status::OK;
    }

    /// @brief Folds the trailing samples at end of stream, then zero-fills the open symbol and any open frame.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_map.has_value()) {
            return reportNoNumerology("processEpilogue", inSpan, outSpan);
        }
        Progress tally = fold(inSpan, inSpan.size(), outSpan);
        if (!_flushed) {
            flush(tally, outSpan);
        }
        outSpan.publish(tally.made);
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
        if (moved("sync_words")) {
            next.syncWords = sync_words.value;
        }
        if (moved("frame_len")) {
            next.frameLength = frame_len.value;
        }
        if (moved("signal_name")) {
            next.signalName = signal_name.value;
        }
        return next;
    }

    /// @brief Checks @p next, then rebuilds the layout and moves the open frame to match it. A block without a layout
    /// builds none while @p next names no data carriers.
    void take(Accepted next) {
        const bool                          newNumerology = next.fftLength != _accepted.fftLength || next.dataCarriers != _accepted.dataCarriers || next.pilotCarriers != _accepted.pilotCarriers;
        std::optional<gr::ofdm::CarrierMap> built{};
        if (newNumerology && (_map.has_value() || !next.dataCarriers.empty())) {
            built.emplace(detail::buildMap(next.fftLength, std::span<const std::int32_t>(next.dataCarriers), std::span<const std::int32_t>(next.pilotCarriers)));
        }
        const gr::ofdm::CarrierMap* map = built.has_value() ? &*built : (_map.has_value() ? &*_map : nullptr);

        std::vector<Complex> pilots = detail::complexFrom(std::span<const float>(next.pilotSymbols), "pilot_symbols");
        if (map != nullptr) {
            detail::requireValid(map->validatePilotCycle(std::span<const Complex>(pilots)));
        }
        std::vector<std::vector<Complex>> words = detail::syncWordsFrom(std::span<const float>(next.syncWords), static_cast<std::size_t>(next.fftLength));
        if (next.frameLength == 0U && !words.empty()) {
            throw gr::exception(std::format("frame_len is 0, which is unframed, but sync_words holds {} symbols: a sync word marks a frame's start and an unframed stream has none", words.size()));
        }

        // Every check has passed. The block changes from here.
        const std::size_t oldFrameLength = _frameLength;
        const std::size_t syncSent       = _syncWords.size() - _syncLeft;
        const bool        renamed        = next.signalName != _accepted.signalName;
        _accepted                        = std::move(next);
        _pilots                          = std::move(pilots);
        _syncWords                       = std::move(words);
        _frameLength                     = static_cast<std::size_t>(_accepted.frameLength);

        // The pending samples and the open frame follow the new settings. Before `start()` both are empty, and
        // `start()` clears them.
        if (built.has_value()) {
            const std::size_t fftLength = built->fftLength();
            _binCarriers.resize(fftLength);
            for (std::size_t bin = 0UZ; bin < fftLength; ++bin) {
                _binCarriers[bin] = detail::carrierAt(fftLength, bin);
            }
            _symbol.assign(fftLength, Complex{});
            _nData = built->nData();
            _map.emplace(std::move(*built));
            buildShape();
            if (_pending.size() > _nData) {
                _dropped += _pending.size();
                _pending.clear();
            }
            if (oldFrameLength > 0UZ || _frameLength > 0UZ) {
                openNextFrame();
                return;
            }
        } else {
            if (renamed && _map.has_value()) {
                buildShape();
            }
            if (_frameLength != oldFrameLength && (oldFrameLength == 0UZ || _frameLength == 0UZ || _dataInFrame >= _frameLength)) {
                openNextFrame();
                return;
            }
        }
        if (_dataInFrame == 0UZ) { // the open frame's sync words continue with the new ones
            _syncLeft = _syncWords.size() > syncSent ? _syncWords.size() - syncSent : 0UZ;
        }
    }

    /// @brief Emits the error of a block that holds no numerology, and takes and makes nothing.
    [[nodiscard]] work::Status reportNoNumerology(std::string_view call, InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        this->emitErrorMessage(call, gr::Error("gr::blocks::ofdm::CarrierAllocator: the block holds no numerology. Its default settings name no data carriers, and no settings application has built one"));
        std::ignore = inSpan.consume(0UZ);
        outSpan.publish(0UZ);
        return work::Status::ERROR;
    }

    /// @brief The samples taken and the records made by one call.
    struct Progress {
        std::size_t taken      = 0UZ;
        std::size_t made       = 0UZ;
        bool        outputFull = false;
    };

    [[nodiscard]] Progress fold(InputSpanLike auto& inSpan, std::size_t offer, OutputSpanLike auto& outSpan) {
        Progress tally{};
        if (_nData == 0UZ) {
            return tally;
        }
        for (;;) {
            if (_pending.size() < _nData) {
                if (tally.taken == offer) {
                    return tally;
                }
                const std::size_t take  = std::min(_nData - _pending.size(), offer - tally.taken);
                const auto        first = inSpan.begin() + static_cast<std::ptrdiff_t>(tally.taken);
                if (_pending.empty()) {
                    _streamAt = static_cast<std::uint64_t>(inSpan.streamIndex) + static_cast<std::uint64_t>(tally.taken);
                }
                _pending.insert(_pending.end(), first, first + static_cast<std::ptrdiff_t>(take));
                tally.taken += take;
                continue;
            }
            // A whole data symbol is ready. Its frame's sync words go out first.
            if (tally.made == outSpan.size()) {
                tally.outputFull = true;
                return tally;
            }
            if (_syncLeft > 0UZ) {
                emitSyncWord(outSpan[tally.made]);
            } else {
                emitDataSymbol(outSpan[tally.made]);
            }
            ++tally.made;
        }
    }

    /// @brief Zero-fills what the stream did not deliver, into the room the output has, and counts what has no room.
    void flush(Progress& tally, OutputSpanLike auto& outSpan) {
        _flushed             = true;
        const bool framed    = _frameLength > 0UZ;
        const bool frameOpen = framed && (_dataInFrame > 0UZ || !_pending.empty());
        if (_pending.empty() && !frameOpen) {
            return;
        }

        // the sync words still owed, the open symbol, and the zero-filled symbols that complete the frame
        std::uint64_t owed = std::uint64_t{_syncLeft} + 1ULL + (framed ? _frameLength - _dataInFrame - 1UZ : 0UZ);
        _padThisSymbol     = _nData - _pending.size();
        _pending.resize(_nData, Complex{});

        while (tally.made < outSpan.size() && owed > 0ULL) {
            if (_syncLeft > 0UZ) {
                emitSyncWord(outSpan[tally.made]);
            } else {
                if (_pending.empty()) { // a whole symbol the frame needs and the stream did not carry
                    _pending.assign(_nData, Complex{});
                    _padThisSymbol = _nData;
                }
                emitDataSymbol(outSpan[tally.made]);
            }
            ++tally.made;
            --owed;
        }
        _truncated += owed;
    }

    void emitSyncWord(DataSet<Complex>& target) {
        const std::size_t index = _syncWords.size() - _syncLeft;
        --_syncLeft;
        stamp(target, _syncWords[index], detail::kKindSync, 0UZ);
        ++_symbolIndex;
        ++_symbolInFrame;
    }

    void emitDataSymbol(DataSet<Complex>& target) {
        const std::size_t  nPilots   = _map->nPilots();
        const std::size_t* dataBins  = _map->dataBins().data();
        const std::size_t* pilotBins = _map->pilotBins().data();
        const Complex*     data      = _pending.data();

        std::ranges::fill(_symbol, Complex{});
        for (std::size_t k = 0UZ; k < _nData; ++k) {
            _symbol[dataBins[k]] = data[k];
        }
        if (nPilots > 0UZ) {
            const Complex*    pilots = _pilots.data();
            const std::size_t cycle  = _pilots.size();
            for (std::size_t p = 0UZ; p < nPilots; ++p) {
                _symbol[pilotBins[p]] = pilots[gr::ofdm::CarrierMap::pilotSymbolIndex(_pilotPhase, p, nPilots, cycle)];
            }
        }

        stamp(target, _symbol, detail::kKindData, _padThisSymbol);
        _padded += static_cast<std::uint64_t>(_padThisSymbol);
        _padThisSymbol = 0UZ;
        _pending.clear();
        ++_symbolIndex;
        ++_symbolInFrame;
        ++_dataInFrame;
        ++_pilotPhase;
        if (_frameLength > 0UZ && _dataInFrame >= _frameLength) {
            openNextFrame();
        }
    }

    /// @brief Ends the open frame where it stands. The next record opens a new frame.
    void openNextFrame() {
        if (_symbolInFrame > 0ULL) {
            ++_frameIndex;
        }
        _symbolInFrame = 0ULL;
        _dataInFrame   = 0UZ;
        _pilotPhase    = 0ULL;
        _syncLeft      = _syncWords.size();
    }

    void buildShape() {
        const std::uint64_t fftLength = _symbol.size();
        _shape.build(std::span<const int>(_binCarriers), _accepted.signalName, property_map{{std::pmr::string(detail::kFftLenKey), pmt::Value(fftLength)}});
    }

    void stamp(DataSet<Complex>& target, const std::vector<Complex>& values, std::string_view kind, std::size_t padCarriers) const {
        property_map& meta                              = _shape.emitInto(target, std::span<const Complex>(values), kind);
        meta[std::pmr::string(detail::kSymbolIndexKey)] = pmt::Value(_symbolIndex);
        meta[std::pmr::string(detail::kFrameIndexKey)]  = pmt::Value(_frameIndex);
        meta[std::pmr::string(detail::kSymbolInFrame)]  = pmt::Value(_symbolInFrame);
        meta[std::pmr::string(detail::kSampleStartKey)] = pmt::Value(_streamAt);
        meta[std::pmr::string(detail::kPadCarriersKey)] = pmt::Value(static_cast<std::uint64_t>(padCarriers));
    }
};

} // namespace gr::blocks::ofdm

#endif // GNURADIO_OFDM_CARRIER_ALLOCATOR_HPP
