#ifndef GNURADIO_DROP_TAG_KEYS_HPP
#define GNURADIO_DROP_TAG_KEYS_HPP

#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::DropTagKeys, [T], [ std::uint8_t, std::int16_t, std::int32_t, float, double, std::complex<float>, std::complex<double> ])

template<typename T>
struct DropTagKeys : Block<DropTagKeys<T>> {
    using Description = Doc<R""(
@brief Passes every sample unchanged and removes the listed keys from every tag that passes.

A graph places the block where a key should stop. Each sample leaves as it arrived. Each tag leaves at the sample it
arrived on, with every key that `keys` lists removed. A tag left with no key is not published. A key is matched whole,
domain included: `gr:sample_rate` removes the rate. Keys inside a `gr:trigger_meta_info` map stay. A change of `keys`
applies to every tag the block forwards after it. A change of `keys` that a tag past the first sample of a chunk brings
applies from the next chunk on. The input's `min_samples` can keep a chunk from ending at such a tag.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<std::vector<std::string>, "keys", Doc<"tag keys to remove, each matched whole">, Visible> keys;

    GR_MAKE_REFLECTABLE(DropTagKeys, in, out, keys);

    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t /*processedIn*/) {
        const auto forward = [this, &outputSpans](const property_map& map, std::size_t offset) {
            property_map forwarded = this->filterAndSubstituteTag(map);
            std::erase_if(forwarded, [this](const auto& entry) { return std::ranges::contains(keys.value, std::string_view(entry.first)); });
            forwarded.erase(static_cast<std::pmr::string>(gr::tag::END_OF_STREAM));
            if (forwarded.empty()) {
                return;
            }
            gr::for_each_writer_span([&forwarded, offset](auto& outSpan) { outSpan.publishTag(forwarded, offset); }, outputSpans);
        };
        gr::for_each_reader_span(
            [&forward](auto& inSpan) {
                if (!inSpan.isSync || !inSpan.isConnected) {
                    return;
                }
                if (inSpan.size() == 0UZ) {
                    // the end of the stream: every tag from the read position on leaves at the end-of-stream index
                    for (const auto& [relIndex, tagMapRef] : inSpan.tags()) {
                        if (relIndex >= 0) {
                            forward(tagMapRef.get(), 0UZ);
                        }
                    }
                    return;
                }
                // each tag of the chunk leaves at its own sample
                // a tag behind the read position left with the previous chunk
                for (const auto& [relIndex, tagMapRef] : inSpan.tags(inSpan.size())) {
                    if (relIndex >= 0) {
                        forward(tagMapRef.get(), static_cast<std::size_t>(relIndex));
                    }
                }
            },
            inputSpans);
    }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        std::ranges::copy(input, output.begin());
        return work::Status::OK;
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_DROP_TAG_KEYS_HPP
