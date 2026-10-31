#ifndef GNURADIO_BASIC_VECTOR_SOURCE_HPP
#define GNURADIO_BASIC_VECTOR_SOURCE_HPP

#include <algorithm>
#include <complex>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Exception.hpp>
#include <gnuradio-4.0/algorithm/dataset/IndexedMaps.hpp>

namespace gr::blocks::basic {

namespace detail {

/// Returns `count` as `std::size_t`. The caller has bounded it by a `std::size_t` value. For a `std::size_t` count the
/// helper applies no cast.
template<std::unsigned_integral Count>
[[nodiscard]] constexpr std::size_t boundedSize(Count count) noexcept {
    if constexpr (std::same_as<Count, std::size_t>) {
        return count;
    } else {
        return static_cast<std::size_t>(count);
    }
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::basic::VectorSource, [T], [ uint8_t, int16_t, uint16_t, int32_t, uint32_t, float, double, std::complex<float>, std::complex<double> ])

template<typename T>
struct VectorSource : Block<VectorSource<T>> {
    using Description = Doc<R""(@brief Emits the samples of `values` in order, `repeat` times, and then ends the stream.

The number of samples is `values` times `repeat`. An empty `values` or a `repeat` of 0 ends the stream at once.
`tags` maps a sample index, written as a decimal string, to the tags published at that sample. The entry at an index
is a map of tag keys and values, published as one tag, or a list of such maps, published as one tag each in list
order.
With `repeat_tags` false, an index counts the samples of all passes, and each tag is published once. An index equal
to the number of samples places its tags at the end-of-stream index.
With `repeat_tags` true, an index counts the samples of one pass, and its tags are published on every pass.
The block refuses a `tags` key that is not a sample index, two keys that name one index, and an entry that is
neither a map nor a list of maps. An index past the number of samples, or with `repeat_tags` an index at or past the
size of `values`, ends the run at the start with an error that names the index. While the block runs, it refuses a
change that leaves such an index.
A change of any setting while the block runs applies from the next call. The block recomputes the number of samples.
After `n` samples, a new `values` continues at its element `n` modulo its size. Without `repeat_tags`, `tags` indices
keep counting from the first sample.
A call emits every sample the output has room for, with the tags due at those samples.
The source refuses, at the start and while it runs, a `tags` setting that places more maps in a run of samples as
long as the output buffer than the tag buffer of the output holds. The tag buffer keeps one slot for the end-of-stream
tag, and the maps at the end-of-stream index count with the last sample. The error names the first index of the run.
Settings in place from the start and within this bound lose no tag while the reader takes the tags of each chunk with
its samples. A running change is checked alone: the tags placed under the earlier settings that still wait in the output
share the tag buffer with the new ones and can leave too few free tag slots. A downstream block that keeps a chunk's
later tags until its next call, such as one with a tag policy that forwards only some keys and a fixed chunk per call,
can also leave too few.
The output then drops the tags that do not fit and counts them in `out.nTagsDropped`. The source never waits for free
tag slots.
)"">;

    PortOut<T> out;

    Annotated<std::vector<T>, "values", Doc<"samples to emit, in order">>        values{};
    Annotated<std::uint64_t, "repeat", Doc<"number of passes over the values">>  repeat      = 1U;
    Annotated<bool, "repeat_tags", Doc<"publish the tags again on every pass">>  repeat_tags = false;
    Annotated<property_map, "tags", Doc<"tags to publish at each sample index">> tags{};

    GR_MAKE_REFLECTABLE(VectorSource, out, values, repeat, repeat_tags, tags);

    std::uint64_t                                       _emitted = 0U;
    bool                                                _failed  = false;
    std::vector<gr::dataset::IndexedMap<std::uint64_t>> _schedule{};

    void start() {
        _emitted = 0U;
        _failed  = false;
        if (const std::optional<std::string> error = scheduleError(_schedule)) {
            _failed = true;
            this->emitErrorMessage("start", *error);
        }
    }

    [[nodiscard]] work::Status processBulk(OutputSpanLike auto& outSpan) {
        if (_failed) {
            return work::Status::ERROR;
        }
        const std::optional<std::uint64_t> total = sampleCount();
        if (!total.has_value()) {
            return fail(countError());
        }
        const std::uint64_t left    = *total > _emitted ? *total - _emitted : 0U;
        const std::size_t   n       = detail::boundedSize(std::min<std::uint64_t>(outSpan.size(), left));
        const std::size_t   nValues = values->size();
        std::size_t         at      = n > 0UZ ? detail::boundedSize(_emitted % nValues) : 0UZ;
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = values.value[at];
            at         = at + 1UZ == nValues ? 0UZ : at + 1UZ;
        }
        visitTags(n, *total, [this, &outSpan](std::uint64_t index, const property_map& map) { outSpan.publishTag(map, detail::boundedSize(index - _emitted)); });
        outSpan.publish(n);
        _emitted += n;
        return _emitted < *total ? work::Status::OK : work::Status::DONE;
    }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        std::optional<std::vector<gr::dataset::IndexedMap<std::uint64_t>>> read;
        if (newSettings.contains("tags")) {
            std::expected<std::vector<gr::dataset::IndexedMap<std::uint64_t>>, std::string> parsed = gr::dataset::readIndexedMaps<std::uint64_t>(tags.value, "tags", "a sample index");
            if (!parsed.has_value()) {
                throw gr::exception(parsed.error());
            }
            read = std::move(*parsed);
        }
        if (lifecycle::isActive(this->state())) {
            if (const std::optional<std::string> error = scheduleError(read.has_value() ? *read : _schedule)) {
                throw gr::exception(*error);
            }
        }
        if (read.has_value()) {
            _schedule = std::move(*read);
        }
    }

    /// Returns `values` times `repeat`, or nothing when the product exceeds 64 bits.
    [[nodiscard]] std::optional<std::uint64_t> sampleCount() const noexcept {
        const std::uint64_t nValues = values->size();
        if (nValues != 0U && repeat.value > std::numeric_limits<std::uint64_t>::max() / nValues) {
            return std::nullopt;
        }
        return nValues * repeat.value;
    }

    [[nodiscard]] std::string countError() const { return std::format("{} values repeated {} times exceed a 64-bit sample count", values->size(), repeat.value); }

    /// Returns the error for a sample count past 64 bits or an index of `schedule` past the samples it can mark.
    [[nodiscard]] std::optional<std::string> misplacedTag(const std::vector<gr::dataset::IndexedMap<std::uint64_t>>& schedule) const {
        const std::optional<std::uint64_t> total = sampleCount();
        if (!total.has_value()) {
            return countError();
        }
        if (schedule.empty()) {
            return std::nullopt;
        }
        const std::uint64_t last = schedule.back().index;
        if (repeat_tags.value && last >= values->size()) {
            return std::format("tags entry at sample {} is past the {} values of one pass", last, values->size());
        }
        if (!repeat_tags.value && last > *total) {
            return std::format("tags entry at sample {} is past the {} samples", last, *total);
        }
        return std::nullopt;
    }

    /// Returns the error of `misplacedTag()` or of `crowdedRun()` for `schedule`.
    [[nodiscard]] std::optional<std::string> scheduleError(const std::vector<gr::dataset::IndexedMap<std::uint64_t>>& schedule) const {
        if (std::optional<std::string> error = misplacedTag(schedule)) {
            return error;
        }
        return crowdedRun(schedule);
    }

    /// Returns the error for a run of `out.bufferSize()` samples that starts at an index of `schedule` and holds more
    /// maps than the tag buffer of the output less its end-of-stream slot. The output holds at most that many unread
    /// samples. A schedule that passes at the start therefore fits the tag buffer while the reader takes the tags of
    /// each chunk with its samples. A running change is checked alone. Tags of the earlier schedule still in the output
    /// can crowd the tag buffer. A downstream block that keeps a chunk's later tags until its next call can fill it as
    /// well, and the output drops the excess. An unconnected output publishes no tag and passes. The caller has checked
    /// `schedule` with `misplacedTag()`.
    [[nodiscard]] std::optional<std::string> crowdedRun(const std::vector<gr::dataset::IndexedMap<std::uint64_t>>& schedule) const {
        const std::optional<std::uint64_t> total = sampleCount();
        if (!out.isConnected() || schedule.empty() || !total.has_value()) {
            return std::nullopt;
        }
        const std::size_t   ring  = out.tagWriter().buffer().size();
        const std::size_t   limit = ring > 0UZ ? ring - 1UZ : 0UZ;
        const std::uint64_t run   = out.bufferSize();
        const auto          error = [run, limit](std::uint64_t index) { return std::format("tags entry at sample {} starts a run of {} samples with more than the {} maps the output's tag buffer holds", index, run, limit); };
        if (!repeat_tags.value) {
            // The maps at the end-of-stream index go out with the last sample.
            const auto  at  = [last = *total](std::uint64_t index) { return index == last && last > 0U ? last - 1U : index; };
            std::size_t end = 0UZ;
            for (std::size_t first = 0UZ; first < schedule.size(); ++first) {
                if (first > 0UZ && schedule[first].index == schedule[first - 1UZ].index) {
                    continue;
                }
                const std::uint64_t from = at(schedule[first].index);
                while (end < schedule.size() && at(schedule[end].index) - from < run) {
                    ++end;
                }
                if (end - first > limit) {
                    return error(schedule[first].index);
                }
            }
            return std::nullopt;
        }
        // With `repeat_tags`, the maps repeat every pass. A run that starts in the first pass holds the most maps of all
        // the runs that start at its place in a pass.
        const std::uint64_t nValues = values->size();
        const std::uint64_t perPass = schedule.size();
        const auto          below   = [&schedule](std::uint64_t index) { return static_cast<std::uint64_t>(std::ranges::lower_bound(schedule, index, {}, &gr::dataset::IndexedMap<std::uint64_t>::index) - schedule.begin()); };
        if (*total == 0U) {
            return std::nullopt;
        }
        for (std::size_t first = 0UZ; first < schedule.size(); ++first) {
            if (first > 0UZ && schedule[first].index == schedule[first - 1UZ].index) {
                continue;
            }
            const std::uint64_t from   = schedule[first].index;
            const std::uint64_t length = std::min(run, *total - from);
            const std::uint64_t passes = length / nValues;
            const std::uint64_t stop   = from + length % nValues;
            if (passes > 0U && perPass > limit / passes) {
                return error(from);
            }
            const std::uint64_t maps = passes * perPass + (stop <= nValues ? below(stop) - first : perPass - first + below(stop - nValues));
            if (maps > limit) {
                return error(from);
            }
        }
        return std::nullopt;
    }

    /// Calls `visit` with the sample index and map of each tag due in the `n` samples from `_emitted`, in index order.
    /// Without `repeat_tags`, the tags at the end-of-stream index are due in the call that reaches the end.
    template<std::invocable<std::uint64_t, const property_map&> Visit>
    void visitTags(std::size_t n, std::uint64_t total, Visit&& visit) const {
        const std::uint64_t end = _emitted + n;
        if (!repeat_tags.value) {
            const bool ends = end >= total;
            auto       tag  = std::ranges::lower_bound(_schedule, _emitted, {}, &gr::dataset::IndexedMap<std::uint64_t>::index);
            for (; tag != _schedule.end() && (tag->index < end || (ends && tag->index == total)); ++tag) {
                visit(tag->index, tag->map);
            }
            return;
        }
        const std::uint64_t nValues = values->size();
        if (n == 0UZ || _schedule.empty()) {
            return;
        }
        for (std::uint64_t pass = _emitted - _emitted % nValues;; pass += nValues) {
            for (const gr::dataset::IndexedMap<std::uint64_t>& tag : _schedule) {
                if (tag.index >= nValues || pass + tag.index >= end) {
                    break;
                }
                if (pass + tag.index >= _emitted) {
                    visit(pass + tag.index, tag.map);
                }
            }
            if (end - pass <= nValues) {
                return;
            }
        }
    }

    [[nodiscard]] work::Status fail(const std::string& message) {
        _failed = true;
        this->emitErrorMessage("processBulk", message);
        return work::Status::ERROR;
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_BASIC_VECTOR_SOURCE_HPP
