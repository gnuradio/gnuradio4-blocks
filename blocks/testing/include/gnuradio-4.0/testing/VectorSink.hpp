#ifndef GNURADIO_TESTING_VECTOR_SINK_HPP
#define GNURADIO_TESTING_VECTOR_SINK_HPP

#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/meta/reflection.hpp>

namespace gr::blocks::testing {

GR_REGISTER_BLOCK(gr::blocks::testing::VectorSink, [T], [ uint8_t, int16_t, int32_t, uint32_t, float, double, std::complex<float>, std::complex<double> ])

template<typename T>
struct VectorSink : Block<VectorSink<T>> {
    using Description = Doc<R""(@brief Records the samples and tags it receives into memory.

Samples are available through `data()` and tags through `tags()`, with tag indices relative to the first recorded
sample. With `n_samples_max` > 0 the block records exactly that many samples and signals DONE, which terminates
graphs fed by unbounded sources. The record is cleared each time the block starts.)"">;

    PortIn<T> in;

    Annotated<std::uint64_t, "max samples", Doc<"count>=n_samples_max -> signal DONE (0: infinite)">> n_samples_max = 0UZ;

    GR_MAKE_REFLECTABLE(VectorSink, in, n_samples_max);

    mutable std::mutex _mutex{};
    std::vector<T>     _samples{};
    std::vector<Tag>   _tags{};
    std::uint64_t      _total_samples = 0ULL;

    void start() {
        std::lock_guard lock(_mutex);
        _samples.clear();
        _tags.clear();
        _total_samples = 0ULL;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& input) {
        std::size_t nTake = input.size();
        bool        limit = false;

        std::lock_guard lock(_mutex);
        if (n_samples_max > 0ULL) {
            const std::uint64_t max        = n_samples_max.value;
            const std::uint64_t nRemaining = max > _total_samples ? max - _total_samples : 0ULL;
            if (static_cast<std::uint64_t>(nTake) >= nRemaining) {
                nTake = static_cast<std::size_t>(nRemaining);
                limit = true;
            }
        }

        for (const auto& [relIndex, tagMapRef] : input.tags()) {
            if (relIndex < 0) {
                continue;
            }
            const std::size_t offset = static_cast<std::size_t>(relIndex);
            if (offset < nTake) {
                _tags.emplace_back(_samples.size() + offset, tagMapRef.get());
            }
        }
        _samples.insert(_samples.end(), input.begin(), input.begin() + static_cast<std::ptrdiff_t>(nTake));
        _total_samples += nTake;

        if (limit) {
            std::ignore = input.consume(nTake);
            return work::Status::DONE;
        }
        return work::Status::OK;
    }

    /// Receives trailing samples and pending end-of-stream user tags without framework control tags.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& input) {
        const work::Status status = processBulk(input);
        if (status == work::Status::DONE) {
            return status;
        }
        std::lock_guard lock(_mutex);
        for (const auto& [relIndex, tagMapRef] : input.tags()) {
            if (relIndex < 0) {
                continue;
            }
            const std::size_t offset = static_cast<std::size_t>(relIndex);
            if (offset >= input.size()) {
                property_map map = tagMapRef.get();
                map.erase("end_of_stream");
                map.erase("gr:end_of_stream");
                if (!map.empty()) {
                    _tags.emplace_back(_samples.size() + (offset - input.size()), std::move(map));
                }
            }
        }
        return status;
    }

    /// copy of the recorded samples; safe to call while the graph runs
    [[nodiscard]] std::vector<T> data() const {
        std::lock_guard lock(_mutex);
        return _samples;
    }

    /// copy of the recorded tags; safe to call while the graph runs
    [[nodiscard]] std::vector<Tag> tags() const {
        std::lock_guard lock(_mutex);
        return _tags;
    }

    /// moves the recorded samples out and leaves the record empty
    [[nodiscard]] std::vector<T> take() {
        std::lock_guard lock(_mutex);
        _tags.clear();
        return std::exchange(_samples, {});
    }

    [[nodiscard]] std::size_t size() const {
        std::lock_guard lock(_mutex);
        return _samples.size();
    }

    void clear() {
        std::lock_guard lock(_mutex);
        _samples.clear();
        _tags.clear();
    }
};

static_assert(gr::BlockLike<VectorSink<float>>);

} // namespace gr::blocks::testing

#endif // GNURADIO_TESTING_VECTOR_SINK_HPP
