#ifndef GNURADIO_ADSB_PPM_FRAMER_HPP
#define GNURADIO_ADSB_PPM_FRAMER_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/digital/Ppm.hpp>

namespace gr::blocks::adsb {

GR_REGISTER_BLOCK(gr::blocks::adsb::PpmFramer, [T], [ float, std::complex<float> ])

/**
 * @brief The block over `gr::digital::PpmScanner`: magnitudes or samples in, one bounded record per published frame out.
 *
 * Thin by construction. It derives the samples per slot from the rate, holds what the scanner has not yet decided
 * about, turns each reported frame into a record, moves the tags that fell inside it, and counts. Every decision
 * about what a frame is belongs to the kernel.
 *
 * @tparam T `float` for a magnitude stream, `std::complex<float>` for the samples themselves. Only the complex
 *         instantiation can align a frame to the phase it sits on, because only it holds what an interpolation reads.
 */
template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct PpmFramer : Block<PpmFramer<T>, NoTagPropagation> {
    /// @brief Whether this instantiation carries the samples an interpolation needs, and so can align a frame's phase.
    static constexpr bool kAligns = std::same_as<T, std::complex<float>>;

    using Description = Doc<R""(
@brief Reads pulse-position frames out of a magnitude or complex stream, one `DataSet<uint8_t>` per frame.

The framer for a burst that arrives out of noise with no carrier before it and no stream after it. Pulse-position
modulation compares the two halves of one bit period with each other, so there is no clock to recover and no
amplitude to track, and a frame is read wherever the preamble's shape appears. `profile` selects the framing:
`mode_s` is ICAO Annex 10 Volume IV's 1090 MHz reply, four preamble pulses at 0.0, 1.0, 3.5 and 4.5 microseconds,
data from 8.0, and a frame of 56 or 112 bits chosen by the downlink format's top bit.

At `float` the input is magnitude, `|x|` and not `|x|^2`: the nomination is a ratio and the bit decision a
comparison, so both are free of any positive scaling, but `threshold` is a number on magnitude and would be its
square on power. `gr::blocks::basic::Abs<std::complex<float>>` produces the stream this instantiation wants on its
`abs` port.

At `std::complex<float>` the samples themselves arrive and every position is decided at the sub-sample phase its
preamble sits on. A pulse whose edges fall between two samples spills part of itself into the neighboring gap slot,
which is the shape test's own denominator, and at the two samples a bit period the common receivers deliver that is
the difference between a frame read and a frame missed. So a position first passes a coarse test at
`threshold * coarse_ratio`, and is then re-tested at `phase_steps` offsets spanning one sample period; the offset
whose least pulse slot stands highest above its greatest gap slot decides both the nomination and the bits, and
travels on the record as `sample_phase`. A frame's timing is then `sample_start + sample_phase`: an admitted frame
takes its window from the first position that passed, which can be the sample before the one its preamble sits
nearest, and the offset compensates. `phase_steps` and `coarse_ratio` are read only by this instantiation.

`sample_rate` must be an even integer multiple of 1 MHz, so a half-microsecond slot is a whole number of samples;
2 MS/s is the rate the common receivers deliver and anything else resamples upstream. A `sample_rate` tag overrides
the setting for the samples after it, and a tagged rate that is not such a multiple is a counted refusal that
leaves the previous rate standing.

The parity is checked here rather than downstream because the remainder decides containment: an admitted frame
consumes its whole window and no position inside it is nominated, while every other nomination advances the search
by one sample, so a false nomination cannot swallow the real frame behind it. The remainder travels on every
record — zero on an admitted frame, and the aircraft address on the formats that transmit the parity XORed with
one. Short frames carry no self-checking parity and CRC-failed long frames were never vouched for, so neither is
published unless `emit_short` or `emit_unchecked` asks for it.

The samples not yet decided about are held in a buffer whose bound is fixed at `start()` from the rate and the
framing, so a consumer that stops taking records cannot grow it with the stream: samples past that bound are dropped
and counted, oldest first.
)"">;

    PortIn<T>                             in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"input rate; required, and an even integer multiple of 1 MHz so a half-microsecond slot is a whole number of samples">, Visible>                            sample_rate = 0.F;
    Annotated<std::string, "profile", Doc<"the pulse-position framing: 'mode_s'; required, because a framing is an interoperability fact and has no default">, Visible>                                         profile{};
    Annotated<float, "threshold", Doc<"nomination ratio of the least preamble pulse slot to the greatest gap slot; must exceed one">, Visible>                                                                  threshold      = 2.F;
    Annotated<gr::Size_t, "phase_steps", Doc<"sub-sample offsets a candidate is re-tested at, spanning one sample period; one turns the fine step off. Read only by the complex instantiation">, Visible>       phase_steps    = 8U;
    Annotated<float, "coarse_ratio", Doc<"fraction of threshold the integer-position test admits a candidate at, before the fine step decides it; in (0, 1]. Read only by the complex instantiation">, Visible> coarse_ratio   = 0.5F;
    Annotated<bool, "emit_short", Doc<"publish short-format nominations, whose parity vouches for nothing, with their remainder attached">>                                                                     emit_short     = false;
    Annotated<bool, "emit_unchecked", Doc<"publish long frames whose remainder is not zero, with crc_ok false">>                                                                                                emit_unchecked = false;
    Annotated<std::string, "frame_label", Doc<"written under the record's trigger_name key">>                                                                                                                   frame_label    = std::string("mode_s");

    GR_MAKE_REFLECTABLE(PpmFramer, in, out, sample_rate, profile, threshold, phase_steps, coarse_ratio, emit_short, emit_unchecked, frame_label);

    // Counted, stated drops and totals. Plain members, read by the owning thread and by QA, reported once at stop().
    std::uint64_t nSamples        = 0ULL; ///< input samples consumed
    std::uint64_t nCandidates     = 0ULL; ///< positions whose preamble shape passed the coarse test, of which the nominations are the subset the fine step kept
    std::uint64_t nNominations    = 0ULL; ///< positions whose preamble shape passed the threshold
    std::uint64_t nAdmitted       = 0ULL; ///< long frames whose whole-frame remainder was zero
    std::uint64_t nCrcFailed      = 0ULL; ///< long frames whose remainder was not zero
    std::uint64_t nShortFormat    = 0ULL; ///< short frames, which carry no self-checking parity
    std::uint64_t nPublished      = 0ULL; ///< records produced for `out`, counted whether or not the port is connected
    std::uint64_t nTagsDropped    = 0ULL; ///< input tags that fell outside every published frame, including the ones still waiting
    std::uint64_t nRateRefused    = 0ULL; ///< sample_rate tags whose value is not an even multiple of 1 MHz
    std::uint64_t nTailDropped    = 0ULL; ///< samples never decided, because their window never completed, including the ones still held
    std::uint64_t nOverrunDropped = 0ULL; ///< samples dropped from the working buffer at its bound, because the sink stopped taking records

    /// @brief One input tag waiting for a frame that might contain it, at its absolute input index.
    struct Pending {
        std::size_t  at   = 0UZ;
        bool         used = false; ///< whether some published frame's span already carried it
        property_map map{};
    };

    /// @brief One arriving tag, as a position in the current span and a borrowed map.
    struct Arriving {
        std::size_t         at  = 0UZ;
        const property_map* map = nullptr;
    };

    /// @brief The pending list is bounded by fiat: several tags may share one index, and none of them sizes a buffer.
    static constexpr std::size_t kPendingCap = 4096UZ;
    /// @brief Room the working buffer keeps beyond one window, so a whole input span is appended without growing it.
    static constexpr std::size_t kWorkReserve = 1UZ << 16U;

    gr::digital::PpmConfig  _framing{};
    gr::digital::PpmScanner _scanner{};
    /// @brief What the kernel counted under the geometries it has already been reconfigured out of, which zero its own counters.
    gr::digital::PpmCounters _retired{};
    std::vector<T>           _work{};          ///< input not yet decided, followed by whatever arrived last
    std::vector<float>       _mags{};          ///< `_work`'s magnitudes, index for index; empty on the magnitude instantiation, where `_work` already is them
    std::size_t              _workStart = 0UZ; ///< absolute input index of `_work`'s first sample
    std::size_t              _slot      = 0UZ; ///< samples per half-bit-period slot, at the rate in force
    float                    _rate      = 0.F; ///< the rate in force, which a tag may replace
    std::vector<Pending>     _pending{};
    std::vector<Arriving>    _arriving{};
    std::uint64_t            _sequence = 0ULL;
    /// @brief The two drop totals as far as the working thread has already retired them; the counters add what is still held.
    std::uint64_t _tailBanked = 0ULL;
    std::uint64_t _tagsBanked = 0ULL;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() {
        if (profile.value.empty()) {
            throw gr::exception("profile is required and has no default: a pulse-position framing is an interoperability fact, and a silent one would be a silent assumption");
        }
        if (slotsFor(sample_rate.value) == 0UZ) {
            throw gr::exception(std::format("sample_rate must be an even integer multiple of 1 MHz so that a half-microsecond slot is a whole number of samples, got {} Hz; resample upstream instead", sample_rate.value));
        }
        rebuild();
        _work.reserve(_scanner.phasedWindowSamples() + kWorkReserve);
        if constexpr (kAligns) {
            _mags.reserve(_scanner.phasedWindowSamples() + kWorkReserve);
        }
        _pending.reserve(kPendingCap);
    }

    void reset() { rebuild(); }

    /// @brief Rebuilds the scanner from the settings, refusing everything the framing cannot be.
    void rebuild() {
        if (!profile.value.empty() && profile.value != std::string_view("mode_s")) {
            throw gr::exception(std::format("profile must be 'mode_s', got '{}'", profile.value));
        }
        if (threshold <= 1.F) {
            throw gr::exception(std::format("threshold is the ratio of the least pulse slot to the greatest gap and must exceed one, because a ratio of one or less nominates on flat noise, got {}", threshold));
        }
        if (phase_steps == 0U) {
            throw gr::exception("phase_steps is the number of sub-sample offsets a candidate is tested at and must be at least one, which is the offset zero the integer positions already are");
        }
        if (!(coarse_ratio > 0.F) || coarse_ratio > 1.F) {
            throw gr::exception(std::format("coarse_ratio scales the threshold the coarse test admits a candidate at and must lie in (0, 1], because a coarse test stricter than the fine one would refuse what the fine one would keep, got {}", coarse_ratio));
        }

        _work.clear();
        _mags.clear();
        _pending.clear();
        _workStart = 0UZ;
        _sequence  = 0ULL;
        nSamples = nCandidates = nNominations = nAdmitted = nCrcFailed = nShortFormat = nPublished = 0ULL;
        nTagsDropped = nRateRefused = nTailDropped = nOverrunDropped = 0ULL;
        _tailBanked = _tagsBanked = 0ULL;
        _retired                  = gr::digital::PpmCounters{};

        if (profile.value.empty()) {
            return;
        }
        _framing               = gr::digital::modeS();
        const std::size_t slot = slotsFor(sample_rate.value);
        arm();
        if (slot != 0UZ) {
            _slot = slot;
            _rate = sample_rate.value;
            _scanner.prepare(_framing, slot);
        }
    }

    /// @brief Puts the nomination settings onto the kernel. `prepare()` sizes the interpolator from them, so this precedes it.
    void arm() {
        // The fine step interpolates the samples themselves, and a magnitude stream holds none: one offset there is
        // the offset zero the integer positions already are, and leaves the kernel's window and its decisions as they
        // were before there was a fine step at all.
        _scanner.threshold   = threshold;
        _scanner.phaseSteps  = kAligns ? static_cast<std::size_t>(phase_steps.value) : 1UZ;
        _scanner.coarseRatio = kAligns ? coarse_ratio.value : 1.F;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_scanner.configured()) { // unconfigured: inert rather than scanning something arbitrary
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }
        const bool        outConnected = outSpan.isConnected;
        const std::size_t room         = outConnected ? outSpan.size() : std::numeric_limits<std::size_t>::max();
        if (room == 0UZ) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }

        _arriving.clear();
        for (const auto& [relIndex, tagMap] : inSpan.tags()) {
            // a block holding undecided samples across calls is shown an earlier call's tag again at a negative index
            if (relIndex >= 0 && static_cast<std::size_t>(relIndex) < inSpan.size()) {
                _arriving.push_back({static_cast<std::size_t>(relIndex), &tagMap.get()});
            }
        }
        std::ranges::stable_sort(_arriving, std::ranges::less{}, &Arriving::at);
        for (const Arriving& tag : _arriving) {
            if (_pending.size() >= kPendingCap) {
                ++_tagsBanked;
                continue;
            }
            _pending.push_back({inSpan.streamIndex + tag.at, false, *tag.map});
        }

        std::size_t onOut = 0UZ;
        const auto  sink  = [&](const gr::digital::PpmFrame& frame) {
            const bool publish = frame.admitted()                                                           //
                                 || (frame.outcome == gr::digital::PpmOutcome::CrcFailed && emit_unchecked) //
                                 || (frame.outcome == gr::digital::PpmOutcome::ShortFormat && emit_short);
            if (!publish || onOut >= room) {
                return;
            }
            DataSet<std::uint8_t> record = buildRecord(frame);
            if (outConnected) {
                outSpan[onOut] = std::move(record);
            }
            ++onOut;
            ++nPublished;
        };

        // one scan pass over everything undecided plus `arriving`, in slices small enough that the sink cannot overrun
        const auto scan = [&](std::span<const T> arriving) {
            _work.insert(_work.end(), arriving.begin(), arriving.end());
            if constexpr (kAligns) {
                // the kernel decides on magnitude and interpolates on the samples, so the two travel together; this
                // is the only place either grows, and the reserve taken at start() is what keeps it off the heap
                const std::size_t known = _mags.size();
                _mags.resize(_work.size());
                for (std::size_t j = 0UZ; j < arriving.size(); ++j) {
                    _mags[known + j] = std::abs(arriving[j]);
                }
            }
            // the span one position needs, guard included: with the fine step off the two are the same number
            const std::size_t window = _scanner.phasedWindowSamples();
            std::size_t       base   = 0UZ;
            while (onOut < room && _work.size() - base >= window) {
                // a slice of `window - 1 + k` samples holds k decidable positions and so at most k frames; when the
                // sink has room for at least as many frames as the buffer has samples, nothing can bind before its end
                const std::size_t available = _work.size() - base;
                const std::size_t headroom  = room - onOut;
                const std::size_t slice     = headroom >= available ? available : std::min(available, window - 1UZ + headroom);
                _scanner.seek(_workStart + base);
                std::size_t done = 0UZ;
                if constexpr (kAligns) {
                    done = _scanner.consume(std::span<const float>(_mags).subspan(base, slice), std::span<const T>(_work).subspan(base, slice), sink);
                } else {
                    done = _scanner.consume(std::span<const float>(_work).subspan(base, slice), sink);
                }
                if (done == 0UZ) {
                    break;
                }
                base += done;
            }
            // the buffer holds one window plus the reserve and no more, so a sink that stops taking records cannot
            // grow it with the stream; the oldest samples go, because the newest are the ones still worth deciding
            const std::size_t capacity = window + kWorkReserve;
            if (_work.size() - base > capacity) {
                const std::size_t excess = _work.size() - base - capacity;
                nOverrunDropped += excess;
                base += excess;
            }
            _work.erase(_work.begin(), _work.begin() + static_cast<std::ptrdiff_t>(base));
            if constexpr (kAligns) {
                _mags.erase(_mags.begin(), _mags.begin() + static_cast<std::ptrdiff_t>(base));
            }
            _workStart += base;
            prunePending(_workStart);
        };

        const std::span<const T> items(inSpan.begin(), inSpan.size());
        std::size_t              cut = 0UZ;
        for (const Arriving& tag : _arriving) {
            const auto entry = tag.map->find(property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()));
            if (entry == tag.map->end()) {
                continue;
            }
            const float* value = entry->second.template get_if<float>();
            if (value == nullptr) {
                continue;
            }
            const std::size_t slot = slotsFor(*value);
            if (slot == 0UZ) {
                ++nRateRefused; // the previous rate stands, because a fractional slot has no bit rule
                continue;
            }
            if (*value == _rate) {
                continue;
            }
            scan(items.subspan(cut, tag.at - cut));
            cut = tag.at;
            // the geometry changes at this sample, so nothing spanning it can be decided under either rate
            _tailBanked += _work.size();
            _work.clear();
            _mags.clear();
            _workStart = inSpan.streamIndex + tag.at;
            _slot      = slot;
            _rate      = *value;
            // preparing the kernel for the new geometry zeroes its counters, so the epoch that just closed is banked
            _retired.candidates += _scanner.counters.candidates;
            _retired.nominations += _scanner.counters.nominations;
            _retired.admitted += _scanner.counters.admitted;
            _retired.crcFailed += _scanner.counters.crcFailed;
            _retired.shortFormat += _scanner.counters.shortFormat;
            arm();
            _scanner.prepare(_framing, slot);
            prunePending(_workStart);
        }
        scan(items.subspan(cut));

        nSamples += inSpan.size();
        nCandidates  = _retired.candidates + _scanner.counters.candidates;
        nNominations = _retired.nominations + _scanner.counters.nominations;
        nAdmitted    = _retired.admitted + _scanner.counters.admitted;
        nCrcFailed   = _retired.crcFailed + _scanner.counters.crcFailed;
        nShortFormat = _retired.shortFormat + _scanner.counters.shortFormat;
        // what the buffer and the list still hold is what a stream ending here drops, and only this thread may look
        nTailDropped = _tailBanked + std::uint64_t{_work.size()};
        nTagsDropped = _tagsBanked + static_cast<std::uint64_t>(std::ranges::count(_pending, false, &Pending::used));

        const std::size_t consumed = inSpan.size();
        std::ignore                = inSpan.consume(consumed);
        outSpan.publish(outConnected ? onOut : 0UZ);
        return consumed == 0UZ && onOut == 0UZ ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::OK;
    }

    /// @brief The record a published frame becomes: its octets, its provenance, and the tags that fell inside it.
    [[nodiscard]] DataSet<std::uint8_t> buildRecord(const gr::digital::PpmFrame& frame) {
        const std::size_t preamble = _framing.preambleSlots * _slot;
        const std::size_t start    = frame.position + preamble;
        const std::size_t span     = preamble + 2UZ * frame.bits * _slot;

        DataSet<std::uint8_t> record;
        record.signal_values.assign(frame.octets.begin(), frame.octets.end());
        record.extents.push_back(static_cast<std::int32_t>(frame.octets.size()));
        record.signal_names.emplace_back("payload");
        record.signal_quantities.emplace_back("");
        record.signal_units.emplace_back("");
        record.meta_information.emplace_back();
        record.timing_events.emplace_back();
        record.timestamp = 0;

        property_map& meta = record.meta_information[0UZ];
        meta.insert_or_assign(property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()), pmt::Value(std::string_view(frame_label.value)));
        // the framing this record was read under; a decoder downstream names its own sub-kind over this one
        meta.insert_or_assign(property_map::key_type("protocol"), pmt::Value(std::string_view(profile.value)));
        meta.insert_or_assign(property_map::key_type("sample_start"), pmt::Value(static_cast<std::uint64_t>(start)));
        meta.insert_or_assign(property_map::key_type("sequence"), pmt::Value(_sequence));
        meta.insert_or_assign(property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()), pmt::Value(_rate));
        meta.insert_or_assign(property_map::key_type("mode_s_format"), pmt::Value(static_cast<gr::Size_t>(frame.format)));
        meta.insert_or_assign(property_map::key_type("crc_remainder"), pmt::Value(static_cast<gr::Size_t>(frame.remainder)));
        if (frame.outcome != gr::digital::PpmOutcome::ShortFormat) {
            // a short frame carries no self-checking parity, and a false there would read as a failure
            meta.insert_or_assign(property_map::key_type("crc_ok"), pmt::Value(frame.admitted()));
        }
        meta.insert_or_assign(property_map::key_type("preamble_strong"), pmt::Value(frame.strong));
        meta.insert_or_assign(property_map::key_type("preamble_weak"), pmt::Value(frame.weak));
        // the frame's timing is `sample_start + sample_phase` and not `sample_start` alone: an admitted frame takes
        // its window from the first position that passed, which can be the sample before the one its preamble sits
        // nearest, and this offset in [-0.5, 0.5) compensates. Zero wherever nothing was searched, so the sum is
        // the timing on either instantiation
        meta.insert_or_assign(property_map::key_type("sample_phase"), pmt::Value(frame.phase));

        auto& events = record.timing_events[0UZ];
        for (Pending& event : _pending) {
            if (event.at < frame.position || event.at >= frame.position + span) {
                continue;
            }
            event.used = true;
            // a tag on a preamble sample precedes the payload, so its index is negative and is not clamped
            events.emplace_back(static_cast<std::ptrdiff_t>(event.at) - static_cast<std::ptrdiff_t>(start), event.map);
        }

        ++_sequence;
        return record;
    }

    /// @brief Retires tags no later frame can carry, because every frame still to come starts at @p before or after.
    void prunePending(std::size_t before) {
        std::size_t retired = 0UZ;
        while (retired < _pending.size() && _pending[retired].at < before) {
            if (!_pending[retired].used) {
                ++_tagsBanked;
            }
            ++retired;
        }
        _pending.erase(_pending.begin(), _pending.begin() + static_cast<std::ptrdiff_t>(retired));
    }

    /// @brief The samples a half-microsecond slot spans at @p rate, or zero when the rate is not an even multiple of 1 MHz.
    [[nodiscard]] static std::size_t slotsFor(float rate) noexcept {
        if (!(rate > 0.F)) {
            return 0UZ;
        }
        const auto slots = static_cast<std::size_t>(std::llround(static_cast<double>(rate) / 2.0e6));
        if (slots == 0UZ) {
            return 0UZ;
        }
        return static_cast<float>(2.0e6 * static_cast<double>(slots)) == rate ? slots : 0UZ;
    }
};

} // namespace gr::blocks::adsb

#endif // GNURADIO_ADSB_PPM_FRAMER_HPP
