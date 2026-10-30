#ifndef GNURADIO_BASIC_VECTOR_SINK_HPP
#define GNURADIO_BASIC_VECTOR_SINK_HPP

#include <complex>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/algorithm/dataset/IndexedMaps.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::VectorSink, [T], [ uint8_t, int16_t, uint16_t, int32_t, uint32_t, float, double, std::complex<float>, std::complex<double> ])

template<typename T>
struct VectorSink : Block<VectorSink<T>> {
    using Description = Doc<R""(@brief Records the samples and tags it receives and publishes the record in its settings when it stops.

The block records each sample in order, and each tag under the index of its sample, counted from the first sample
since the start. `tags` has the form of the `tags` setting of `VectorSource`. An index that received one tag holds
its map, and an index that received several holds the list of their maps in arrival order. A tag at the end of the
stream goes under the sample count, without the end-of-stream key. A recorded run fed to a source comes out
unchanged.
With `n_samples_max` above 0, the block stops after it records that many samples and the tags among them.
Starting the block clears the record. Stopping it writes the record to `data`, `tags` and `received`.
While the block runs, `data`, `tags` and `received` hold the empty record of the start. A change of `n_samples_max`
applies from the next call.
)"">;

    PortIn<T> in;

    Annotated<gr::Size_t, "n_samples_max", Doc<"sample limit, 0 is unlimited">> n_samples_max = 0U;
    Annotated<std::uint64_t, "received", Doc<"number of samples recorded">>     received      = 0U;
    Annotated<std::vector<T>, "data", Doc<"recorded samples, in order">>        data{};
    Annotated<property_map, "tags", Doc<"recorded tags keyed by sample index">> tags{};

    GR_MAKE_REFLECTABLE(VectorSink, in, n_samples_max, received, data, tags);

    std::vector<T> _data{};
    property_map   _tags{};

    void start() {
        _data.clear();
        _tags.clear();
        publish();
    }

    void stop() { publish(); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& input) {
        std::size_t n       = input.size();
        bool        limited = false;
        if (n_samples_max.value > 0U) {
            const std::size_t limit = n_samples_max.value;
            const std::size_t left  = limit > _data.size() ? limit - _data.size() : 0UZ;
            if (n >= left) {
                n       = left;
                limited = true;
            }
        }
        for (const auto& [relIndex, map] : input.tags()) {
            if (relIndex >= 0 && static_cast<std::size_t>(relIndex) < n) {
                gr::dataset::appendIndexedMap<std::uint64_t>(_tags, _data.size() + static_cast<std::size_t>(relIndex), map.get());
            }
        }
        _data.insert(_data.end(), input.begin(), input.begin() + static_cast<std::ptrdiff_t>(n));
        if (limited) {
            std::ignore = input.consume(n);
            return work::Status::DONE;
        }
        return work::Status::OK;
    }

    /// Records the stream's last samples, then the tags at or past its end, which no sample carries.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& input) {
        const work::Status status = processBulk(input);
        if (status != work::Status::OK) {
            return status;
        }
        const std::size_t end = input.streamIndex + input.size();
        for (const gr::Tag& tag : in.tagReader().get()) {
            if (tag.index < end) {
                continue;
            }
            property_map map = tag.map;
            map.erase(std::pmr::string(gr::tag::END_OF_STREAM.key()));
            map.erase(std::pmr::string(gr::tag::END_OF_STREAM.shortKey()));
            if (!map.empty()) {
                gr::dataset::appendIndexedMap<std::uint64_t>(_tags, _data.size() + (tag.index - end), std::move(map));
            }
        }
        return status;
    }

    /// Moves the record into the settings.
    void publish() {
        received    = _data.size();
        data.value  = std::exchange(_data, {});
        tags.value  = std::exchange(_tags, {});
        std::ignore = this->updateActiveParameters({"received", "data", "tags"});
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_BASIC_VECTOR_SINK_HPP
