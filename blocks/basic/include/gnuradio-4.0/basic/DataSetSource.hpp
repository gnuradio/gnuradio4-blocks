#ifndef GNURADIO_BASIC_DATASET_SOURCE_HPP
#define GNURADIO_BASIC_DATASET_SOURCE_HPP

#include <algorithm>
#include <cstddef>
#include <expected>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/algorithm/dataset/DataSetRecords.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::DataSetSource, [T], [ uint8_t, float ])

template<typename T>
struct DataSetSource : Block<DataSetSource<T>> {
    using Description = Doc<R""(@brief Emits the DataSets of `records` in order and then ends the stream.

`records` maps record indices to records. Each key is a record index written as a decimal string, and the indices run
from 0 without a gap. Each value is a map of DataSet fields: `signal_values`, `signal_names`, `signal_quantities`,
`signal_units`, `axis_names`, `extents`, `timestamp`, `meta_information` and `timing_events`. An empty map or a null
value is a record without fields.
A record without `signal_names` has one signal with an empty name. A record without `signal_quantities`,
`signal_units` or `axis_names` has none. A record without `extents` has one dimension that holds the values of one
signal, the value count divided by the signal count. A record without `timestamp` has the timestamp 0.
A `meta_information` map gives the DataSet one map, and a list of maps gives it one map per entry. A record without
`meta_information` carries no map.
`timing_events` holds one entry per signal. Each entry has the form of the `tags` setting of `VectorSource`: event
indices, written as decimal strings, each mapped to a map of keys and values or to a list of such maps. An event index
may be negative. A signal's events follow in index order. A record without `timing_events` carries one empty list of
events per signal. The DataSet fields `axis_units`, `axis_values` and `signal_ranges` stay empty.
The block reads `records` when it starts. These entries end the run with an error that names the key, or the record
index and the field: a key that is not a record index, a missing index, two keys that name one index, an entry that
is not a map, an unknown field, a field value that does not convert, and a value count the signal count does not
divide in a record without `extents`.
)"">;

    PortOut<DataSet<T>, Async> out;

    Annotated<property_map, "records", Doc<"DataSets to emit, keyed by record index">> records{};

    GR_MAKE_REFLECTABLE(DataSetSource, out, records);

    std::vector<DataSet<T>> _records{};
    std::size_t             _emitted = 0UZ;
    bool                    _failed  = false;

    void start() {
        _records.clear();
        _emitted = 0UZ;
        _failed  = false;

        std::expected<std::vector<property_map>, std::string> maps = gr::dataset::readRecordMaps(records.value);
        if (!maps.has_value()) {
            failAtStart(maps.error());
            return;
        }
        _records.reserve(maps->size());
        for (std::size_t i = 0UZ; i < maps->size(); ++i) {
            std::expected<gr::dataset::DataSetRecord<T>, std::string> record = gr::dataset::readDataSetRecord<T>(i, (*maps)[i]);
            if (!record.has_value()) {
                failAtStart(record.error());
                return;
            }
            _records.push_back(std::move(record->dataSet));
        }
    }

    [[nodiscard]] work::Status processBulk(OutputSpanLike auto& outSpan) {
        if (_failed) {
            return work::Status::ERROR;
        }
        const std::size_t n = std::min(outSpan.size(), _records.size() - _emitted);
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = _records[_emitted + i];
        }
        outSpan.publish(n);
        _emitted += n;
        return _emitted >= _records.size() ? work::Status::DONE : work::Status::OK;
    }

    void failAtStart(const std::string& message) {
        _failed = true;
        this->emitErrorMessage("start", message);
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_BASIC_DATASET_SOURCE_HPP
