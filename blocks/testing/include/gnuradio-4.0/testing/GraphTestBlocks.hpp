#ifndef GNURADIO_TESTING_GRAPH_TEST_BLOCKS_HPP
#define GNURADIO_TESTING_GRAPH_TEST_BLOCKS_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/meta/formatter.hpp>

namespace gr::blocks::testing {

GR_REGISTER_BLOCK(gr::blocks::testing::VectorSource, [T], [ uint8_t, int16_t, int32_t, float, double, std::complex<float> ])

template<typename T>
struct VectorSource : Block<VectorSource<T>> {
    using Description = Doc<R""(@brief Emits the samples of `values` in order and then ends the stream.

The whole of `values` is emitted `repeat` times. An empty `values` ends the stream at once.
The block publishes `tag_map` as a tag at every sample index in `tag_indices` below the sample count.
An empty `tag_map` publishes no tag.
)"">;

    PortOut<T> out;

    Annotated<std::vector<T>, "values", Doc<"samples to emit, in order">>                       values{};
    Annotated<gr::Size_t, "repeat", Doc<"number of passes over the values">>                    repeat = 1U;
    Annotated<std::vector<gr::Size_t>, "tag_indices", Doc<"sample indices that carry tag_map">> tag_indices{};
    Annotated<property_map, "tag_map", Doc<"keys and values of the published tag">>             tag_map{};

    GR_MAKE_REFLECTABLE(VectorSource, out, values, repeat, tag_indices, tag_map);

    std::size_t _emitted = 0UZ;

    void start() { _emitted = 0UZ; }

    [[nodiscard]] work::Status processBulk(OutputSpanLike auto& outSpan) {
        const std::size_t nValues = values->size();
        const std::size_t total   = nValues * static_cast<std::size_t>(repeat.value);
        const std::size_t n       = std::min(outSpan.size(), total > _emitted ? total - _emitted : 0UZ);
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = values.value[(_emitted + i) % nValues];
        }
        if (!tag_map->empty()) {
            std::vector<gr::Size_t> indices = tag_indices.value;
            std::ranges::sort(indices);
            indices.erase(std::ranges::unique(indices).begin(), indices.end());
            for (const gr::Size_t tagIndex : indices) {
                const std::size_t index = static_cast<std::size_t>(tagIndex);
                if (index >= _emitted && index < _emitted + n) {
                    outSpan.publishTag(tag_map.value, index - _emitted);
                }
            }
        }
        outSpan.publish(n);
        _emitted += n;
        return _emitted >= total ? work::Status::DONE : work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::testing::ExpectSink, [T], [ uint8_t, int16_t, int32_t, uint32_t, float, double, std::complex<float> ])

template<typename T>
struct ExpectSink : Block<ExpectSink<T>> {
    using Description = Doc<R""(@brief Compares the input stream with `expected` and reports the first difference as an error.

A sample matches when its distance from the expected sample is at most
`tolerance + relative_tolerance * |expected|`. A NaN matches a NaN, and an infinity matches the same infinity.
A complex sample with an infinite part matches only an equal sample.
The first sample that does not match ends the run with an error that names its index, its value and the
expected value. A stream shorter than `expected` ends with an error that names both counts. A sample past the
end of `expected` ends the run with an error that names its index.
A non-empty `tag_key` also checks the tags. The sample indices of the tags carrying that key must equal
`tag_indices` when the stream ends, each index counted once. An empty `tag_indices` requires that no tag
carries the key. A tag at the end-of-stream index, one past the last sample, has the sample count as its index.
)"">;

    PortIn<T> in;

    Annotated<std::vector<T>, "expected", Doc<"samples the input must carry, in order">>                  expected{};
    Annotated<double, "tolerance", Doc<"absolute deviation accepted per sample">>                         tolerance          = 0.0;
    Annotated<double, "relative_tolerance", Doc<"deviation accepted per sample, relative to |expected|">> relative_tolerance = 0.0;
    Annotated<gr::Size_t, "received", Doc<"samples received and matched">>                                received           = 0U;
    Annotated<std::string, "tag_key", Doc<"tag key whose indices are checked">>                           tag_key{};
    Annotated<std::vector<gr::Size_t>, "tag_indices", Doc<"sample indices of the tags carrying tag_key">> tag_indices{};

    GR_MAKE_REFLECTABLE(ExpectSink, in, expected, tolerance, relative_tolerance, received, tag_key, tag_indices);

    bool                     _failed = false;
    std::vector<std::size_t> _tagged{};

    void start() {
        received = 0U;
        _failed  = false;
        _tagged.clear();
    }

    void stop() {
        if (!_failed && static_cast<std::size_t>(received.value) != expected->size()) {
            _failed = true;
            this->emitErrorMessage("stop", std::format("the stream ended after {} samples, expected {}", received.value, expected->size()));
        }
        if (!_failed && !tag_key->empty()) {
            std::vector<std::size_t> wanted(tag_indices->begin(), tag_indices->end());
            std::ranges::sort(wanted);
            wanted.erase(std::ranges::unique(wanted).begin(), wanted.end());
            std::ranges::sort(_tagged);
            if (wanted != _tagged) {
                _failed = true;
                this->emitErrorMessage("stop", std::format("tags with key '{}' at {}, expected at {}", tag_key.value, _tagged, wanted));
            }
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& input) {
        if (_failed) {
            return work::Status::ERROR;
        }
        if (!tag_key->empty()) {
            recordTags(input);
        }
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            const std::size_t index = static_cast<std::size_t>(received.value) + i;
            if (index >= expected->size()) {
                received = static_cast<gr::Size_t>(index);
                return fail(std::format("sample {} is {}, beyond the {} expected samples", index, input[i], expected->size()));
            }
            if (!matches(input[i], expected.value[index])) {
                received = static_cast<gr::Size_t>(index);
                return fail(std::format("sample {} is {}, expected {}", index, input[i], expected.value[index]));
            }
        }
        received = static_cast<gr::Size_t>(received.value + input.size());
        return work::Status::OK;
    }

    /// The stream's last samples, then the tags at or past the end of the stream, which no sample carries.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& input) {
        const work::Status status = processBulk(input);
        if (status != work::Status::OK || tag_key->empty()) {
            return status;
        }
        const std::string prefixed = "gr:" + tag_key.value;
        const std::size_t end      = input.streamIndex + input.size();
        for (const gr::Tag& tag : in.tagReader().get()) {
            if (tag.index < end || (!tag.map.contains(tag_key.value) && !tag.map.contains(prefixed))) {
                continue;
            }
            const std::size_t index = static_cast<std::size_t>(received.value) + (tag.index - end);
            if (!std::ranges::contains(_tagged, index)) {
                _tagged.push_back(index);
            }
        }
        return status;
    }

    /// A tag at a negative relative index belongs to an earlier call and is skipped.
    void recordTags(InputSpanLike auto& input) {
        const std::string prefixed = "gr:" + tag_key.value;
        for (const auto& [relIndex, tagMap] : input.tags()) {
            if (relIndex < 0 || static_cast<std::size_t>(relIndex) >= input.size()) {
                continue;
            }
            const property_map& map = tagMap.get();
            if (!map.contains(tag_key.value) && !map.contains(prefixed)) {
                continue;
            }
            const std::size_t index = static_cast<std::size_t>(received.value) + static_cast<std::size_t>(relIndex);
            if (!std::ranges::contains(_tagged, index)) {
                _tagged.push_back(index);
            }
        }
    }

    [[nodiscard]] work::Status fail(const std::string& message) {
        _failed = true;
        this->emitErrorMessage("processBulk", message);
        return work::Status::ERROR;
    }

    [[nodiscard]] bool matches(const T& value, const T& reference) const {
        if constexpr (meta::complex_like<T>) {
            const bool valueNan     = std::isnan(value.real()) || std::isnan(value.imag());
            const bool referenceNan = std::isnan(reference.real()) || std::isnan(reference.imag());
            if (valueNan || referenceNan) {
                return valueNan && referenceNan;
            }
            const bool valueInf     = std::isinf(value.real()) || std::isinf(value.imag());
            const bool referenceInf = std::isinf(reference.real()) || std::isinf(reference.imag());
            if (valueInf || referenceInf) {
                return value == reference;
            }
            return within(std::abs(std::complex<double>(value) - std::complex<double>(reference)), std::abs(std::complex<double>(reference)));
        } else {
            if constexpr (std::floating_point<T>) {
                if (std::isnan(value) || std::isnan(reference)) {
                    return std::isnan(value) && std::isnan(reference);
                }
                if (std::isinf(value) || std::isinf(reference)) {
                    return value == reference;
                }
            }
            return within(std::abs(asDouble(value) - asDouble(reference)), std::abs(asDouble(reference)));
        }
    }

    [[nodiscard]] static double asDouble(const T& sample) {
        if constexpr (std::same_as<T, double>) {
            return sample;
        } else {
            return static_cast<double>(sample);
        }
    }

    [[nodiscard]] bool within(double distance, double magnitude) const { return distance <= tolerance.value + relative_tolerance.value * magnitude; }
};

} // namespace gr::blocks::testing

#endif // GNURADIO_TESTING_GRAPH_TEST_BLOCKS_HPP
