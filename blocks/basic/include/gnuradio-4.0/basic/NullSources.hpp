#ifndef GNURADIO_BASIC_NULL_SOURCES_HPP
#define GNURADIO_BASIC_NULL_SOURCES_HPP

#include <cstdint>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/meta/reflection.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::NullSource, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float>, gr::Packet<float>, gr::Tensor<float>, gr::DataSet<float> ])

template<typename T>
struct NullSource : Block<NullSource<T>> {
    using Description = Doc<R""(@brief Emits the value-initialized `T{}` on every output sample.

The block emits samples until the graph stops it.)"">;

    gr::PortOut<T> out;

    GR_MAKE_REFLECTABLE(NullSource, out);

    [[nodiscard]] constexpr T processOne() const noexcept { return T{}; }

    [[nodiscard]] constexpr auto processOne(meta::constexpr_value auto N) const noexcept
    requires std::is_arithmetic_v<T>
    {
        return meta::simdize<T, N>{};
    }
};

static_assert(gr::BlockLike<NullSource<float>>);

GR_REGISTER_BLOCK(gr::blocks::basic::ConstantSource, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float>, gr::Packet<float>, gr::Tensor<float>, gr::DataSet<float> ])

template<typename T>
struct ConstantSource : Block<ConstantSource<T>> {
    using value_t     = std::conditional_t<std::is_same_v<T, std::string>, std::string, meta::fundamental_base_value_type_t<T>>; // use base-type for types not supported by settings
    using Description = Doc<R""(@brief Emits `default_value` on every output sample.

The block counts the samples it emits in `count`. When `count` reaches a non-zero `n_samples_max`, the block emits that
sample and stops. With `n_samples_max` at zero, the block runs until the graph stops.)"">;

    gr::PortOut<T> out;

    Annotated<value_t, "default value", Visible, Doc<"default value for each sample">> default_value{};
    Annotated<std::uint64_t, "max samples", Doc<"sample limit, 0 is unlimited">>       n_samples_max = 0UZ;
    Annotated<std::uint64_t, "count", Doc<"sample count (diagnostics only)">>          count         = 0UZ;

    GR_MAKE_REFLECTABLE(ConstantSource, out, default_value, n_samples_max, count);

    void reset() { count = 0U; }

    [[nodiscard]] constexpr T processOne() noexcept {
        count++;
        if (n_samples_max > 0 && count >= n_samples_max) {
            this->requestStop();
        }
        return T(default_value.value);
    }
};

static_assert(gr::BlockLike<ConstantSource<float>>);

GR_REGISTER_BLOCK(gr::blocks::basic::HeadBlock, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float>, gr::Packet<float>, gr::Tensor<float>, gr::DataSet<float> ])

template<typename T>
struct HeadBlock : Block<HeadBlock<T>> {
    using Description = Doc<R""(@brief Copies input samples to the output and stops after `n_samples_max` samples.

`count` holds the samples copied. With `n_samples_max` at zero, the block copies every sample.)"">;

    gr::PortIn<T>  in;
    gr::PortOut<T> out;

    Annotated<std::uint64_t, "max samples", Doc<"sample limit, 0 is unlimited">> n_samples_max = 0UZ;
    Annotated<std::uint64_t, "count", Doc<"sample count (diagnostics only)">>    count         = 0UZ;

    GR_MAKE_REFLECTABLE(HeadBlock, in, out, n_samples_max, count);

    void reset() { count = 0U; }

    [[nodiscard]] constexpr auto processOne(T input) noexcept {
        count++;
        if (n_samples_max > 0 && count >= n_samples_max) {
            this->requestStop();
        }
        return input;
    }
};

static_assert(gr::BlockLike<HeadBlock<float>>);

GR_REGISTER_BLOCK(gr::blocks::basic::NullSink, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float>, gr::Packet<float>, gr::Tensor<float>, gr::DataSet<float> ])

template<typename T>
struct NullSink : Block<NullSink<T>> {
    using Description = Doc<R""(@brief Consumes every input sample and discards it.)"">;

    gr::PortIn<T> in;
    GR_MAKE_REFLECTABLE(NullSink, in);

    template<gr::meta::t_or_simd<T> V>
    void processOne(V) const noexcept { /* do nothing */ }
};

static_assert(gr::BlockLike<NullSink<float>>);

} // namespace gr::blocks::basic

#endif // GNURADIO_BASIC_NULL_SOURCES_HPP
