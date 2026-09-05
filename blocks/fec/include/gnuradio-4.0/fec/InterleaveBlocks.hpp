#ifndef GNURADIO_FEC_INTERLEAVE_BLOCKS_HPP
#define GNURADIO_FEC_INTERLEAVE_BLOCKS_HPP

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <format>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/fec/Interleaver.hpp>

#include <gnuradio-4.0/fec/RecordShape.hpp>

/**
 * The record-native adapters over the interleaver families in `gnuradio-4.0/algorithm/fec`.
 *
 * An interleaver spreads a burst. A channel that destroys a run of adjacent items hands a decoder
 * a run of adjacent errors, which is the one shape a code with a small correcting radius cannot
 * take; moving the items apart before the channel and back afterwards turns that run into single
 * errors scattered over a frame, which is the shape every code is designed for. Nothing here
 * reads an item's value, so the whole layer is addressing arithmetic and the item type is the
 * carrier's business rather than the family's.
 *
 * The carrier is the bit record the other adapters in this module speak: a
 * `DataSet<std::uint8_t>` with one item per bit or per symbol, the value untouched from one end
 * to the other. A symbol-width consumer packs before this block and unpacks after it — the
 * families move items and a wider item is a different packing, not a different interleaver.
 * The framed kinds also carry `DataSet<float>`, the soft record a soft-decision chain
 * deinterleaves before its decoder: a permutation never reads a value, so the float
 * registration is addressing arithmetic over a different carrier and nothing more.
 *
 * One block pair carries all three families, selected by an immutable `kind` setting. The
 * settings surfaces barely overlap, but the record contract, the counted drops and the counters
 * are identical across them, and three pairs would state that contract three times.
 *
 * - **`block`** takes `rows` and `cols`, and — on `Interleave` only — optionally the
 *   `window_offset` and `window_length` of a readout that covers only part of the interleaved
 *   frame. One record must hold a whole number of frames.
 * - **`convolutional`** takes `branches` and `unit_delay`. It is the one family here whose state
 *   crosses a record boundary: it is stream-shaped, any record length is valid, and splitting a
 *   stream into different records gives the same output items. The end-to-end
 *   interleave-to-deinterleave delay is `branches * (branches - 1) * unit_delay` items and the
 *   initial fill emits `fill_value` rather than swallowing items, so the output is 1:1 with the
 *   input from the first item.
 * - **`permutation`** takes an explicit `table`, required with no default: a published table is
 *   an interoperability constant and a default would be an interoperability assumption nobody
 *   made.
 *
 * **Metadata carry.** The framed kinds turn one record into one record and the record's facts
 * cross verbatim. The convolutional kind produces a span whose items came from several earlier
 * records, and the output record carries the facts of the record that held the *first* item of
 * that span — the only causal choice, since that is the record the span begins in. While the
 * delay lines are still filling, the span's first item came from the fill rather than from any
 * record, and the output then carries the facts of the record being consumed.
 *
 * The family and its shape are immutable configuration rather than live settings. Both ends of a
 * link agree on an interleaver before the first item, so changing one makes a different chain,
 * which is what rebuilding a graph is for.
 */
namespace gr::blocks::fec {

namespace detail {

//! What one output record inherits: the facts of the record the kernel drew its first item from.
struct InterleaverOrigin {
    std::size_t  begin = 0UZ; //!< the absolute item index at which that record's items started
    std::string  name{};      //!< its signal name, or the module's label where it named none
    property_map meta{};      //!< its metadata map, or an empty one where it carried none
};

//! The kernel the family settings name, reporting its refusals through the graph's exception type.
template<typename T>
[[nodiscard]] inline gr::fec::Interleaver<T> interleaverKernel(const std::string& kind, gr::Size_t rows, gr::Size_t cols, gr::Size_t windowOffset, gr::Size_t windowLength, //
    gr::Size_t branches, gr::Size_t unitDelay, const std::vector<gr::Size_t>& table, gr::Size_t fillValue) {
    if (kind.empty()) {
        throw gr::exception("kind has no default and must be set to 'block', 'convolutional' or 'permutation': the three families share a contract but not a shape, and neither end of a link can guess the other's");
    }
    if (fillValue > 0xFFU) {
        throw gr::exception(std::format("fill_value is one item of the byte carrier and must be 0 to 255, got {}", fillValue));
    }
    const T fill = static_cast<T>(fillValue);

    try {
        if (kind == "block") {
            const std::size_t frame  = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
            const std::size_t length = (windowLength == 0U) ? frame : static_cast<std::size_t>(windowLength);
            return gr::fec::Interleaver<T>::block(rows, cols, windowOffset, length, fill);
        }
        if (kind == "convolutional") {
            if constexpr (!std::same_as<T, std::uint8_t>) {
                throw gr::exception("kind 'convolutional' carries bytes only: its fill and its across-record state have no float semantics, and no consumer asks");
            } else {
                return gr::fec::Interleaver<T>::convolutional(branches, unitDelay, fill);
            }
        }
        if (kind == "permutation") {
            if (table.empty()) {
                throw gr::exception("table is required for kind 'permutation' and has no default: a published table is an interoperability constant, and a default table would be an interoperability assumption nobody made");
            }
            std::vector<std::size_t> indices(table.begin(), table.end());
            return gr::fec::Interleaver<T>::permutation(std::span<const std::size_t>(indices));
        }
    } catch (const std::invalid_argument& refusal) {
        throw gr::exception(std::format("kind '{}': {}", kind, refusal.what()));
    }
    throw gr::exception(std::format("kind must be 'block', 'convolutional' or 'permutation', got '{}'", kind));
}

/*!
 * @brief The state the convolutional kind needs beyond the kernel's own: which record each output
 * span begins in.
 *
 * The kernel holds the delay lines and the commutator, which is everything the items need. What it
 * cannot hold is the record vocabulary, because it has never heard of a record. The item at
 * absolute output position `p` left branch `p mod B`, so it entered the lines `delay(p mod B)`
 * items earlier; the record that held that item is the one whose facts the output span inherits.
 * Keeping the origins in a queue costs one entry per record in flight, which the latency bounds.
 */
class OriginQueue {
public:
    void reset() noexcept {
        _origins.clear();
        _items = 0UZ;
    }

    //! Note that @p record's items begin at the current stream position, and hand back that position.
    template<typename T>
    [[nodiscard]] std::size_t admit(const DataSet<T>& record) {
        const std::size_t begin = _items;
        _origins.push_back(InterleaverOrigin{begin, originName(record), originMeta(record)});
        _items += record.signal_values.size();
        return begin;
    }

    /*!
     * @brief The origin of the item at absolute output position @p start, given a delay of
     * @p delayItems items and a longest delay of @p maxDelay.
     *
     * A delay reaching back before the first item is the initial fill: no record held that item, and
     * the record being consumed is the only one there is to name.
     *
     * The queue is retired here rather than by walking it forward, because the delay is the
     * commutator's and therefore cycles: consecutive spans reach back by different amounts and the
     * source index is not monotonic. What no future span can reach is everything older than
     * `start - maxDelay`, and that is the only bound safe to drop by.
     */
    [[nodiscard]] const InterleaverOrigin& originAt(std::size_t start, std::size_t delayItems, std::size_t maxDelay) noexcept {
        const std::size_t horizon = (start > maxDelay) ? start - maxDelay : 0UZ;
        while (_origins.size() > 1UZ && _origins[1UZ].begin <= horizon) {
            _origins.pop_front();
        }
        if (delayItems > start) {
            return _origins.back(); // still filling: the span opens on fill values, which came from no record
        }
        const std::size_t source = start - delayItems;
        std::size_t       pick   = 0UZ;
        while (pick + 1UZ < _origins.size() && _origins[pick + 1UZ].begin <= source) {
            ++pick;
        }
        return _origins[pick];
    }

private:
    std::deque<InterleaverOrigin> _origins{};
    std::size_t                   _items{0UZ};
};

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::fec::Interleave, [T], [ std::uint8_t, float ])

/*!
@brief Interleave: item records in, the same items in the family's permuted order out.

The framed kinds — `block` and `permutation` — turn one record into one record. A record's length
must be a nonzero multiple of the family's frame, and each frame becomes `rows * cols` items, or
the window's length where `kind` is `block` and a window is set. A record whose length fails that
test is dropped and counted in `nRecordsRefused`, one line at `stop()` reports the total, and the
record after it is interleaved normally: a record producer cannot emit a misaligned record except
through a fault, and the counter is where such a fault becomes visible.

The `convolutional` kind is stream-shaped. Any nonzero length is a valid record, the output is 1:1
with the input from the first item, and the delay lines keep their contents across records, so the
same stream cut into different records interleaves identically. Only an empty record is refused.
It carries bytes only: a `float` block refuses it at configure, because its fill and its
across-record state have no float semantics and no consumer asks.

An interleaver has no status to report, so the record's facts cross unchanged; its signal name and
its single-map shape follow it, and the output record's extent names its own length. Which record's
facts a convolutional output inherits is the file's stated rule. See Deinterleave for the
counterpart that puts the items back.
*/
template<typename T>
requires std::same_as<T, std::uint8_t> || std::same_as<T, float>
struct Interleave : Block<Interleave<T>> {
    using Description = Doc<"interleave: item records in, the same items in the permuted order of the family the 'kind' setting names">;

    PortIn<DataSet<T>, Async>  in;
    PortOut<DataSet<T>, Async> out;

    Annotated<std::string, "kind", Doc<"'block', 'convolutional' or 'permutation'; there is no default, because the three families share a contract but not a shape">, Visible> kind{};
    Annotated<gr::Size_t, "rows", Doc<"kind 'block': rows of the rectangle, written row-major and read column-major">, Visible>                                                 rows          = 0U;
    Annotated<gr::Size_t, "cols", Doc<"kind 'block': columns of the rectangle; items adjacent at the input land 'rows' apart at the output">, Visible>                          cols          = 0U;
    Annotated<gr::Size_t, "window_offset", Doc<"kind 'block': first item of the column-major readout to emit">>                                                                 window_offset = 0U;
    Annotated<gr::Size_t, "window_length", Doc<"kind 'block': items of the readout to emit; 0 means the whole frame">>                                                          window_length = 0U;
    Annotated<gr::Size_t, "branches", Doc<"kind 'convolutional': delay lines B, at least two; branch b holds b * unit_delay cells">, Visible>                                   branches      = 0U;
    Annotated<gr::Size_t, "unit_delay", Doc<"kind 'convolutional': cells M per branch step; the end-to-end delay is B * (B - 1) * M items">, Visible>                           unit_delay    = 0U;
    Annotated<std::vector<gr::Size_t>, "table", Doc<"kind 'permutation': the gather out[i] = in[table[i]], every index below the frame size once; required, with no default">>  table{};
    Annotated<gr::Size_t, "fill_value", Doc<"the item value the convolutional fill emits, 0 to 255">>                                                                           fill_value = 0U;

    GR_MAKE_REFLECTABLE(Interleave, in, out, kind, rows, cols, window_offset, window_length, branches, unit_delay, table, fill_value);

    std::optional<gr::fec::Interleaver<T>> _kernel{};
    detail::OriginQueue                    _origins{};
    bool                                   _configured = false;

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords        = 0ULL; ///< records published on `out`
    std::uint64_t nItems          = 0ULL; ///< items those records carry
    std::uint64_t nRecordsRefused = 0ULL; ///< records whose length was not a whole number of frames

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        _kernel.emplace(detail::interleaverKernel<T>(kind.value, rows.value, cols.value, window_offset.value, window_length.value, branches.value, unit_delay.value, table.value, fill_value.value));
        _origins.reset();
        _configured = true; // only reached when the settings named a family the kernel accepts
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 3UZ> counters{{{"records", nRecords}, {"items", nItems}, {"records refused", nRecordsRefused}}};
        detail::report("gr::blocks::fec::Interleave", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) { // inert rather than permuting under a family nobody chose
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }
        const bool        stream   = _kernel->kind() == gr::fec::InterleaverKind::Convolutional;
        const std::size_t inFrame  = _kernel->frameSize();
        const std::size_t outFrame = _kernel->interleavedSize();

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<T>& record = inSpan[consumed];
            const std::size_t items  = record.signal_values.size();
            if (items == 0UZ || (!stream && items % inFrame != 0UZ)) {
                ++nRecordsRefused;
                continue;
            }

            DataSet<T> moved;
            moved.signal_values.resize(stream ? items : (items / inFrame) * outFrame);
            if (stream) {
                // The span opens at the stream position this record's items open at, so the branch the
                // commutator is on there is what dates the span's first item.
                const std::size_t start                 = _origins.admit(record);
                const std::size_t depth                 = (start % _kernel->branches()) * _kernel->unitDelay() * _kernel->branches();
                std::ignore                             = _kernel->interleave(record.signal_values, moved.signal_values);
                const detail::InterleaverOrigin& origin = _origins.originAt(start, depth, _kernel->latency());
                detail::carryOrigin(moved, origin.name, origin.meta);
            } else {
                std::ignore = _kernel->interleave(record.signal_values, moved.signal_values);
                detail::carryOrigin(moved, detail::originName(record), detail::originMeta(record));
            }

            ++nRecords;
            nItems += moved.signal_values.size();
            outSpan[made] = std::move(moved);
            ++made;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::fec::Deinterleave, [T], [ std::uint8_t, float ])

/*!
@brief Deinterleave: the interleaved items in, the original order out.

The inverse of Interleave under the same settings, item for item. The `convolutional` kind runs
the complementary delay set, so an interleave followed by a deinterleave delays the stream by
exactly `branches * (branches - 1) * unit_delay` items and changes nothing else about it; it
carries bytes only, as on Interleave.

`output_offset` and `output_length` window the DEINTERLEAVED frame: the emitted record is items
`[output_offset, output_offset + output_length)` of each frame, with a length of 0 meaning "to
the end of the frame", and the record's extent names the emitted length. A distributed sync word
occupies the deinterleaved frame's first items, so `output_offset` at the word's item count hands
the decoder the coded block alone. The window addresses a frame, so the stream-shaped
`convolutional` kind refuses it. Interleave's readout window is a different thing — a partial
readout of the INTERLEAVED frame on the transmit side — and deliberately has no counterpart here:
the pair's inverse relationship must not depend on two settings agreeing.

Length refusals, the counters and the metadata carry are Interleave's, unchanged.
*/
template<typename T>
requires std::same_as<T, std::uint8_t> || std::same_as<T, float>
struct Deinterleave : Block<Deinterleave<T>> {
    using Description = Doc<"deinterleave: the interleaved items in, the original order out, under the family the 'kind' setting names">;

    PortIn<DataSet<T>, Async>  in;
    PortOut<DataSet<T>, Async> out;

    Annotated<std::string, "kind", Doc<"'block', 'convolutional' or 'permutation'; there is no default, because the three families share a contract but not a shape">, Visible> kind{};
    Annotated<gr::Size_t, "rows", Doc<"kind 'block': rows of the rectangle, written row-major and read column-major">, Visible>                                                 rows          = 0U;
    Annotated<gr::Size_t, "cols", Doc<"kind 'block': columns of the rectangle; items adjacent at the input land 'rows' apart at the output">, Visible>                          cols          = 0U;
    Annotated<gr::Size_t, "output_offset", Doc<"framed kinds: items dropped from the front of each deinterleaved frame">>                                                       output_offset = 0U;
    Annotated<gr::Size_t, "output_length", Doc<"framed kinds: items kept of each deinterleaved frame; 0 means to the end of the frame">>                                        output_length = 0U;
    Annotated<gr::Size_t, "branches", Doc<"kind 'convolutional': delay lines B, at least two; branch b holds (B - 1 - b) * unit_delay cells on this side">, Visible>            branches      = 0U;
    Annotated<gr::Size_t, "unit_delay", Doc<"kind 'convolutional': cells M per branch step; the end-to-end delay is B * (B - 1) * M items">, Visible>                           unit_delay    = 0U;
    Annotated<std::vector<gr::Size_t>, "table", Doc<"kind 'permutation': the interleaver's table, inverted once at configure; required, with no default">>                      table{};
    Annotated<gr::Size_t, "fill_value", Doc<"the item value the convolutional fill emits, 0 to 255">>                                                                           fill_value = 0U;

    GR_MAKE_REFLECTABLE(Deinterleave, in, out, kind, rows, cols, output_offset, output_length, branches, unit_delay, table, fill_value);

    std::optional<gr::fec::Interleaver<T>> _kernel{};
    detail::OriginQueue                    _origins{};
    bool                                   _configured = false;
    std::size_t                            _outOffset  = 0UZ; //!< the resolved window, in items of the deinterleaved frame
    std::size_t                            _outLength  = 0UZ;
    std::vector<T>                         _whole{}; //!< the full deinterleaved frames, when only a window of them is emitted

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords        = 0ULL; ///< records published on `out`
    std::uint64_t nItems          = 0ULL; ///< items those records carry
    std::uint64_t nRecordsRefused = 0ULL; ///< records whose length was not a whole number of frames

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        _kernel.emplace(detail::interleaverKernel<T>(kind.value, rows.value, cols.value, 0U, 0U, branches.value, unit_delay.value, table.value, fill_value.value));
        if (_kernel->kind() == gr::fec::InterleaverKind::Convolutional) {
            if (output_offset.value != 0U || output_length.value != 0U) {
                throw gr::exception("the convolutional kind is stream-shaped and has no frame for an output window to address");
            }
            _outOffset = 0UZ;
            _outLength = 0UZ;
        } else {
            const std::size_t frame = _kernel->frameSize();
            if (static_cast<std::size_t>(output_offset.value) >= frame) {
                throw gr::exception(std::format("output_offset {} does not address the {}-item deinterleaved frame", output_offset.value, frame));
            }
            _outOffset = static_cast<std::size_t>(output_offset.value);
            _outLength = (output_length.value == 0U) ? frame - _outOffset : static_cast<std::size_t>(output_length.value);
            if (_outOffset + _outLength > frame) {
                throw gr::exception(std::format("output window [{}, {}) runs past the {}-item deinterleaved frame", _outOffset, _outOffset + _outLength, frame));
            }
        }
        _origins.reset();
        _configured = true; // only reached when the settings named a family the kernel accepts
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 3UZ> counters{{{"records", nRecords}, {"items", nItems}, {"records refused", nRecordsRefused}}};
        detail::report("gr::blocks::fec::Deinterleave", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) { // inert rather than permuting under a family nobody chose
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }
        const bool        stream   = _kernel->kind() == gr::fec::InterleaverKind::Convolutional;
        const std::size_t inFrame  = _kernel->interleavedSize();
        const std::size_t outFrame = _kernel->frameSize();

        const bool windowed = !stream && !(_outOffset == 0UZ && _outLength == outFrame);

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<T>& record = inSpan[consumed];
            const std::size_t items  = record.signal_values.size();
            if (items == 0UZ || (!stream && items % inFrame != 0UZ)) {
                ++nRecordsRefused;
                continue;
            }

            DataSet<T> moved;
            if (stream) {
                // The complementary delay set, so that the two sides together delay every item alike.
                moved.signal_values.resize(items);
                const std::size_t start                 = _origins.admit(record);
                const std::size_t depth                 = (_kernel->branches() - 1UZ - (start % _kernel->branches())) * _kernel->unitDelay() * _kernel->branches();
                std::ignore                             = _kernel->deinterleave(record.signal_values, moved.signal_values);
                const detail::InterleaverOrigin& origin = _origins.originAt(start, depth, _kernel->latency());
                detail::carryOrigin(moved, origin.name, origin.meta);
            } else if (windowed) {
                // deinterleave the whole frames, then emit the window of each: the window addresses the
                // deinterleaved order, which only exists once the frame is whole
                const std::size_t frames = items / inFrame;
                _whole.resize(frames * outFrame);
                std::ignore = _kernel->deinterleave(record.signal_values, _whole);
                moved.signal_values.resize(frames * _outLength);
                for (std::size_t f = 0UZ; f < frames; ++f) {
                    const T* from = _whole.data() + f * outFrame + _outOffset;
                    std::copy_n(from, _outLength, moved.signal_values.data() + f * _outLength);
                }
                detail::carryOrigin(moved, detail::originName(record), detail::originMeta(record));
            } else {
                moved.signal_values.resize((items / inFrame) * outFrame);
                std::ignore = _kernel->deinterleave(record.signal_values, moved.signal_values);
                detail::carryOrigin(moved, detail::originName(record), detail::originMeta(record));
            }

            ++nRecords;
            nItems += moved.signal_values.size();
            outSpan[made] = std::move(moved);
            ++made;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::fec

#endif // GNURADIO_FEC_INTERLEAVE_BLOCKS_HPP
