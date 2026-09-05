#ifndef GNURADIO_DIGITAL_BIT_PACKING_HPP
#define GNURADIO_DIGITAL_BIT_PACKING_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/digital/BitPacking.hpp>

namespace gr::blocks::digital {

namespace detail {

inline void requireFieldWidth(gr::Size_t bits, std::string_view setting) {
    if (bits < 1U || bits > 8U) {
        throw gr::exception(std::format("{} is a field width in bits and must be in [1, 8], got {}", setting, bits));
    }
}

/**
 * @brief The tag route the three bit-packing blocks share: the offset map, and the tags waiting for their output item.
 *
 * A tag marks an input item, that item's first bit is bit `t * bitsIn` of the stream, and the tag belongs on the
 * output item holding that bit — `floor(t * bitsIn / bitsOut)`, computed in integer arithmetic end to end. This
 * replaces the framework's forwarding rather than adjusting it, because the default publishes a tag at the output
 * index matching its input index, which is right only at a ratio of one.
 *
 * Holding is ordinary here rather than a corner case: at `1:8` packing seven of every eight tags map into the
 * current call and the eighth may not, and at `8:1` unpacking a tag near the end of a call routinely lands past the
 * output span. Order and multiplicity are preserved and no tag is deduplicated. Every key is republished verbatim,
 * which is what replacing the forwarder means, so a key the default forwarder would filter survives these blocks
 * and will not survive a default-forwarding neighbor.
 */
struct RepackTagRoute {
    std::vector<std::pair<std::uint64_t, property_map>> pending{};

    void clear() { pending.clear(); }

    template<typename TInputSpans, typename TOutputSpans>
    void forward(const gr::digital::BitRepack& cfg, TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        std::vector<std::pair<std::uint64_t, property_map>> arriving;
        gr::for_each_reader_span(
            [&arriving, &cfg, processedIn](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const auto& [relIndex, tagMap] : span.tags(processedIn)) {
                    if (relIndex < 0) {
                        // A tag from before this window is one this block has already placed. The framework's own
                        // forwarding clamps it to offset zero and republishes it, which would duplicate it.
                        continue;
                    }
                    const std::uint64_t at = static_cast<std::uint64_t>(span.streamIndex) + static_cast<std::uint64_t>(relIndex);
                    arriving.emplace_back(gr::digital::mapRepackedOffset(at, cfg.bitsIn, cfg.bitsOut), tagMap.get());
                }
            },
            inputSpans);

        if (arriving.empty() && pending.empty()) {
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
                    if (tag.first >= end) { // its output item is not in this call: hold it rather than move it
                        deferred.push_back(tag);
                        return;
                    }
                    span.publishTag(tag.second, static_cast<std::size_t>(tag.first > base ? tag.first - base : 0ULL));
                };
                for (const auto& tag : pending) {
                    place(tag);
                }
                for (const auto& tag : arriving) {
                    place(tag);
                }
            },
            outputSpans);

        pending = std::move(deferred);
    }
};

/// @brief Whole periods of the conversion; the framework hands exact chunks, so the counts agree or the contract is broken.
inline void convertWholePeriods(const gr::digital::BitRepack& cfg, std::span<const std::uint8_t> input, std::span<std::uint8_t> output, std::string_view block) {
    const std::size_t periods = input.size() / cfg.inChunk;
    if (periods * cfg.outChunk != output.size()) {
        throw gr::exception(std::format("{}: {} input items yield {} outputs, not the {} reserved", block, input.size(), periods * cfg.outChunk, output.size()));
    }
    gr::digital::repack(cfg, input, output);
}

/**
 * @brief The end of the stream: whole periods where there are any, then the trailing items shorter than one period.
 *
 * The tail primitive reads its bit positions off the period's own table, so it is given only what is left after the
 * whole periods are taken. Without @p pad the stranded bits are dropped, which is reported once because a drop must
 * not be silent.
 *
 * The framework reaches the end-of-stream hook once per stream whatever the last chunk left, and reserves a whole
 * output chunk for it either way, so an empty @p input is ordinary here: no period, no stranded bit, nothing written
 * and nothing reported. The count returned, never the room offered, is what the caller publishes.
 */
[[nodiscard]] inline std::size_t convertTail(const gr::digital::BitRepack& cfg, std::span<const std::uint8_t> input, std::span<std::uint8_t> output, bool pad, std::string_view block) {
    const std::size_t periods = std::min(input.size() / cfg.inChunk, output.size() / cfg.outChunk);
    const std::size_t taken   = periods * static_cast<std::size_t>(cfg.inChunk);
    const std::size_t made    = periods * static_cast<std::size_t>(cfg.outChunk);
    gr::digital::repack(cfg, input.first(taken), output.first(made));

    const std::size_t stranded  = (input.size() - taken) * static_cast<std::size_t>(cfg.bitsIn);
    const std::size_t remainder = stranded % static_cast<std::size_t>(cfg.bitsOut);
    if (!pad && remainder != 0UZ) {
        std::println(stderr, "{}: {} trailing bits are dropped at end of stream, short of one {}-bit output item", block, remainder, cfg.bitsOut);
    }
    return made + gr::digital::repackTail(cfg, input.subspan(taken), output.subspan(made), pad);
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::digital::PackBits)

struct PackBits : Block<PackBits, Resampling<1UZ, 1UZ, false>> {
    using TParent     = Block<PackBits, Resampling<1UZ, 1UZ, false>>;
    using Description = Doc<R""(
@brief Gathers `k` one-bit items into one `k`-bit item, `k` inputs to one output.

What follows a slicer: one bit per symbol arrives and a CRC, a scrambler or a file sink wants whole bytes. Only the
low bit of each input item is read, so a decision carrying anything in its upper bits contributes exactly what its
low bit says.

`bit_order` names which end of the `k`-bit field the first bit of the stream occupies: `msb_first` puts it at
position `k-1` and counts down, `lsb_first` at position `0` and counts up. The one-bit side has no order to state.
The order is a parameter rather than a convention, because the packing end and the unpacking end of a chain have to
agree on it explicitly.

`PackBits(k)` is `RepackBits(bits_in = 1, bits_out = k)` and shares its arithmetic; it exists because one width and
one order are easier to configure correctly than four settings. Nothing comes out until `k` items have arrived,
which is the declared chunk size rather than a startup rule, and a tag moves to the output item holding the bit it
marked, `floor(t / k)`, so `k` consecutive input tags share one output offset in input order. `sample_rate` is
forwarded unchanged: the item rate falls by `k` but the sample rate does not.

`flush_partial` emits the final incomplete item at end of stream, its unfilled positions zero; clearing it drops
those bits and reports how many. At `k = 1` the block is a copy of the low bit of every item and the setting is
inert.
)"">;

    PortIn<std::uint8_t>  in;
    PortOut<std::uint8_t> out;

    Annotated<gr::Size_t, "k", Unit<"bits">, Doc<"bits gathered into each output item, in [1, 8]">, Visible>                        k             = 8U;
    Annotated<gr::digital::BitOrder, "bit_order", Doc<"which end of the k-bit field the stream's first bit takes">>                 bit_order     = gr::digital::BitOrder::MsbFirst;
    Annotated<bool, "flush_partial", Doc<"emit the final incomplete output item with its unfilled positions zero; inert at k = 1">> flush_partial = true;

    GR_MAKE_REFLECTABLE(PackBits, in, out, k, bit_order, flush_partial);

    gr::digital::BitRepack _repack{};
    detail::RepackTagRoute _tagRoute{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        configureConversion();
        if (newSettings.contains("k") || newSettings.contains("bit_order")) {
            _tagRoute.clear();
        }
    }

    void start() {
        configureConversion(); // a block left at its defaults stages nothing, so this is the only call that reaches it
        _tagRoute.clear();
    }

    void reset() { _tagRoute.clear(); }

    void configureConversion() {
        detail::requireFieldWidth(k, "k");

        gr::digital::configureRepack(_repack, 1U, k, gr::digital::BitOrder::MsbFirst, bit_order);
        this->input_chunk_size  = _repack.inChunk;
        this->output_chunk_size = _repack.outChunk;
    }

    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        _tagRoute.forward(_repack, inputSpans, outputSpans, processedIn);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        detail::convertWholePeriods(_repack, std::span<const std::uint8_t>(inSpan.data(), inSpan.size()), std::span<std::uint8_t>(outSpan.data(), outSpan.size()), "gr::blocks::digital::PackBits");
        return work::Status::OK;
    }

    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        outSpan.publish(detail::convertTail(_repack, std::span<const std::uint8_t>(inSpan.data(), inSpan.size()), std::span<std::uint8_t>(outSpan.data(), outSpan.size()), flush_partial, "gr::blocks::digital::PackBits"));
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::digital::UnpackBits)

struct UnpackBits : Block<UnpackBits, Resampling<1UZ, 1UZ, false>> {
    using TParent     = Block<UnpackBits, Resampling<1UZ, 1UZ, false>>;
    using Description = Doc<R""(
@brief Spreads each `k`-bit item into `k` one-bit items, one input to `k` outputs.

What feeds a modulator: whole bytes arrive and a constellation mapper wants the bits one at a time, or `k` at a time
through a second stage. Only the low `k` bits of an input item are read, so an item carrying anything above them
contributes exactly what its low `k` bits say.

`bit_order` names which end of the `k`-bit field leaves first: `msb_first` emits position `k-1` first and counts
down, `lsb_first` emits position `0` first and counts up. The one-bit side has no order to state. The order is a parameter rather than a convention, because the
packing end and the unpacking end of a chain have to agree on it explicitly.

`UnpackBits(k)` is `RepackBits(bits_in = k, bits_out = 1)` and shares its arithmetic. One input item is a whole
period, so the block carries no state at all, every call is a whole number of items on both sides, and
`flush_partial` can never do anything — it is kept and documented as inert rather than hidden, because a setting
that sometimes does nothing is better than one that says when only in a table. A tag moves to the first of the `k`
items its own item became, `k * t`, so the map is exact and no two tags collide. `sample_rate` is forwarded
unchanged: the item rate rises by `k` but the sample rate does not.
)"">;

    PortIn<std::uint8_t>  in;
    PortOut<std::uint8_t> out;

    Annotated<gr::Size_t, "k", Unit<"bits">, Doc<"significant bits per input item, spread over k outputs; in [1, 8]">, Visible> k             = 8U;
    Annotated<gr::digital::BitOrder, "bit_order", Doc<"which end of the k-bit field leaves first">>                             bit_order     = gr::digital::BitOrder::MsbFirst;
    Annotated<bool, "flush_partial", Doc<"inert here: one input item is a whole period, so no input can trail one">>            flush_partial = true;

    GR_MAKE_REFLECTABLE(UnpackBits, in, out, k, bit_order, flush_partial);

    gr::digital::BitRepack _repack{};
    detail::RepackTagRoute _tagRoute{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        configureConversion();
        if (newSettings.contains("k") || newSettings.contains("bit_order")) {
            _tagRoute.clear();
        }
    }

    void start() {
        configureConversion(); // a block left at its defaults stages nothing, so this is the only call that reaches it
        _tagRoute.clear();
    }

    void reset() { _tagRoute.clear(); }

    void configureConversion() {
        detail::requireFieldWidth(k, "k");

        gr::digital::configureRepack(_repack, k, 1U, bit_order, gr::digital::BitOrder::MsbFirst);
        this->input_chunk_size  = _repack.inChunk;
        this->output_chunk_size = _repack.outChunk;
    }

    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        _tagRoute.forward(_repack, inputSpans, outputSpans, processedIn);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        detail::convertWholePeriods(_repack, std::span<const std::uint8_t>(inSpan.data(), inSpan.size()), std::span<std::uint8_t>(outSpan.data(), outSpan.size()), "gr::blocks::digital::UnpackBits");
        return work::Status::OK;
    }

    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        outSpan.publish(detail::convertTail(_repack, std::span<const std::uint8_t>(inSpan.data(), inSpan.size()), std::span<std::uint8_t>(outSpan.data(), outSpan.size()), flush_partial, "gr::blocks::digital::UnpackBits"));
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::digital::RepackBits)

struct RepackBits : Block<RepackBits, Resampling<1UZ, 1UZ, false>> {
    using TParent     = Block<RepackBits, Resampling<1UZ, 1UZ, false>>;
    using Description = Doc<R""(
@brief Regroups a stream of `bits_in`-bit fields into a stream of `bits_out`-bit fields, bit sequence unchanged.

An item is a byte carrying its significant bits in the low positions, and the bit stream of a port is the
concatenation of those fields taken in that port's bit order. `msb_first` puts the stream's first bit at the top of
the field, `lsb_first` at the bottom. The conversion is then one sentence: the output stream's bit sequence is the
input stream's bit sequence. Bit order is not byte order — it names the traversal inside one item, never the order
of items — and the two sides carry independent orders, so an MSB-first byte stream becomes LSB-first symbols in one
pass.

The conversion repeats every `lcm(bits_in, bits_out)` bits, which the block declares as its chunk sizes, so no bit
cursor is carried between calls and the output does not depend on how a scheduler divides the stream. Changing a
width or an order is a discontinuity: the change lands on a period boundary of the old configuration, every input
item consumed before it is grouped as the old configuration grouped it, and the pending tags are dropped.

A tag moves to the output item holding the first bit of the item it marked, `floor(t * bits_in / bits_out)`, in
integer arithmetic. Packing therefore puts several input tags on one output offset, in input order. Every key is
forwarded verbatim, including keys the framework's own forwarder would filter. `sample_rate` is forwarded unchanged
and is a stated limitation: the item rate changes by `bits_in / bits_out` but the sample rate does not, so rewriting
the key would claim the stream slowed down.

At end of stream fewer than one period of input can remain. `flush_partial` emits the final incomplete item,
zero-padded in the positions the later bits would have occupied; clearing it drops those bits and reports how many.
The setting is inert whenever `bits_out` divides `bits_in`, because a single input item is then a whole period and
nothing can trail it.
)"">;

    PortIn<std::uint8_t>  in;
    PortOut<std::uint8_t> out;

    Annotated<gr::Size_t, "bits_in", Unit<"bits">, Doc<"significant bits per input item, in [1, 8]">, Visible>                                              bits_in          = 1U;
    Annotated<gr::Size_t, "bits_out", Unit<"bits">, Doc<"significant bits per output item, in [1, 8]">, Visible>                                            bits_out         = 8U;
    Annotated<gr::digital::BitOrder, "input_bit_order", Doc<"traversal of each input field: MsbFirst reads position bits_in-1 first, LsbFirst position 0">> input_bit_order  = gr::digital::BitOrder::MsbFirst;
    Annotated<gr::digital::BitOrder, "output_bit_order", Doc<"traversal of each output field, independent of the input's">>                                 output_bit_order = gr::digital::BitOrder::MsbFirst;
    Annotated<bool, "flush_partial", Doc<"emit the final incomplete output item zero-padded; inert when bits_out divides bits_in">>                         flush_partial    = true;

    GR_MAKE_REFLECTABLE(RepackBits, in, out, bits_in, bits_out, input_bit_order, output_bit_order, flush_partial);

    gr::digital::BitRepack _repack{};
    detail::RepackTagRoute _tagRoute{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        static constexpr std::array kShapeKeys{"bits_in", "bits_out", "input_bit_order", "output_bit_order"};

        configureConversion();
        if (std::ranges::any_of(kShapeKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            _tagRoute.clear();
        }
    }

    void start() {
        configureConversion(); // a block left at its defaults stages nothing, so this is the only call that reaches it
        _tagRoute.clear();
    }

    void reset() { _tagRoute.clear(); }

    void configureConversion() {
        detail::requireFieldWidth(bits_in, "bits_in");
        detail::requireFieldWidth(bits_out, "bits_out");
        const gr::digital::BitOrder orderIn  = input_bit_order;
        const gr::digital::BitOrder orderOut = output_bit_order;

        gr::digital::configureRepack(_repack, bits_in, bits_out, orderIn, orderOut);
        this->input_chunk_size  = _repack.inChunk;
        this->output_chunk_size = _repack.outChunk;
    }

    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        _tagRoute.forward(_repack, inputSpans, outputSpans, processedIn);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        detail::convertWholePeriods(_repack, std::span<const std::uint8_t>(inSpan.data(), inSpan.size()), std::span<std::uint8_t>(outSpan.data(), outSpan.size()), "gr::blocks::digital::RepackBits");
        return work::Status::OK;
    }

    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        outSpan.publish(detail::convertTail(_repack, std::span<const std::uint8_t>(inSpan.data(), inSpan.size()), std::span<std::uint8_t>(outSpan.data(), outSpan.size()), flush_partial, "gr::blocks::digital::RepackBits"));
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::digital::RecordRepackBits)

/*!
@brief Regroups one bounded record of `bits_in`-bit fields into a record of `bits_out`-bit fields, bit sequence
unchanged.

The record form of RepackBits, for the chains that are records at the point the grouping changes: an extracted
codeblock of one-bit items packs into the byte record a symbol decoder consumes without flattening to a stream, so
the boundary and the record's facts survive the regrouping. The bit vocabulary is the stream family's — an item
carries its significant bits in the low positions, each side's order names the traversal inside one of its items,
and the output's bit sequence is the input's.

A record has an end where a stream has none, so the remainder rule is a record rule: a record whose bit count is
not a whole number of output fields is a counted, stated drop in `nRecordsRefused`, and the record after it is
regrouped normally — a record producer cuts records to its protocol's own lengths, so a remainder
says the configuration and the record disagree. The record's metadata, signal name and single-map shape cross
verbatim; an item-count change is what the extent is for.
*/
struct RecordRepackBits : Block<RecordRepackBits> {
    using Description = Doc<"record form of RepackBits: one record's bit fields regrouped, the boundary and the metadata kept">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "bits_in", Unit<"bits">, Doc<"significant bits per input item, in [1, 8]">, Visible>                                              bits_in          = 1U;
    Annotated<gr::Size_t, "bits_out", Unit<"bits">, Doc<"significant bits per output item, in [1, 8]">, Visible>                                            bits_out         = 8U;
    Annotated<gr::digital::BitOrder, "input_bit_order", Doc<"traversal of each input field: MsbFirst reads position bits_in-1 first, LsbFirst position 0">> input_bit_order  = gr::digital::BitOrder::MsbFirst;
    Annotated<gr::digital::BitOrder, "output_bit_order", Doc<"traversal of each output field, independent of the input's">>                                 output_bit_order = gr::digital::BitOrder::MsbFirst;

    GR_MAKE_REFLECTABLE(RecordRepackBits, in, out, bits_in, bits_out, input_bit_order, output_bit_order);

    gr::digital::BitOrder _inOrder    = gr::digital::BitOrder::MsbFirst;
    gr::digital::BitOrder _outOrder   = gr::digital::BitOrder::MsbFirst;
    bool                  _configured = false;

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords        = 0ULL; ///< records published on `out`
    std::uint64_t nRecordsRefused = 0ULL; ///< records whose bit count was not a whole number of output fields

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        if (bits_in < 1U || bits_in > 8U || bits_out < 1U || bits_out > 8U) {
            throw gr::exception(std::format("bits_in and bits_out are the significant bits of an item and must be in [1, 8], got {} and {}", bits_in.value, bits_out.value));
        }
        _inOrder    = input_bit_order;
        _outOrder   = output_bit_order;
        _configured = true;
    }

    void stop() {
        if (nRecordsRefused > 0ULL) {
            std::println(stderr, "gr::blocks::digital::RecordRepackBits '{}': {} record(s) refused, their bit count not a whole number of {}-bit fields", this->name, nRecordsRefused, bits_out.value);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) { // inert rather than regrouping under an order nobody chose
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        const auto widthIn  = static_cast<unsigned>(bits_in.value);
        const auto widthOut = static_cast<unsigned>(bits_out.value);

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record    = inSpan[consumed];
            const std::size_t            totalBits = record.signal_values.size() * widthIn;
            if (totalBits == 0UZ || totalBits % widthOut != 0UZ) {
                ++nRecordsRefused;
                continue;
            }

            DataSet<std::uint8_t> grouped = record; // the record's facts cross verbatim; the values are regrouped below
            grouped.signal_values.assign(totalBits / widthOut, 0U);
            if (!grouped.extents.empty()) {
                grouped.extents[0UZ] = static_cast<std::int32_t>(grouped.signal_values.size());
            }
            for (std::size_t bit = 0UZ; bit < totalBits; ++bit) {
                const std::size_t inItem  = bit / widthIn;
                const unsigned    inPos   = _inOrder == gr::digital::BitOrder::MsbFirst ? widthIn - 1U - static_cast<unsigned>(bit % widthIn) : static_cast<unsigned>(bit % widthIn);
                const std::size_t outItem = bit / widthOut;
                const unsigned    outPos  = _outOrder == gr::digital::BitOrder::MsbFirst ? widthOut - 1U - static_cast<unsigned>(bit % widthOut) : static_cast<unsigned>(bit % widthOut);
                grouped.signal_values[outItem] |= static_cast<std::uint8_t>(((record.signal_values[inItem] >> inPos) & 1U) << outPos);
            }

            ++nRecords;
            outSpan[made] = std::move(grouped);
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

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_BIT_PACKING_HPP
