#ifndef GNURADIO_HARD_LIMITER_HPP
#define GNURADIO_HARD_LIMITER_HPP

#include <cmath>
#include <complex>
#include <concepts>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/meta/utils.hpp>

namespace gr::blocks::analog {

GR_REGISTER_BLOCK(gr::blocks::analog::HardLimiter, [T], [ std::complex<float>, float ])

template<typename T>
requires(gr::meta::complex_like<T> || std::floating_point<T>)
struct HardLimiter : Block<HardLimiter<T>> {
    using Description = Doc<R""(
@brief Scales every complex sample to unit magnitude, leaving only the phase; the signum on a real stream.

A constant-envelope demodulator wants only the phase, and removing the amplitude with no time constant is the point:
an automatic gain control in the same position moves its gain on a schedule of its own, and while a discriminator's
angle is immune to gain, the noise floor it produces between transmissions is not - a rising gain in dead air lifts
noise into a weak signal's range, and a symbol tracker downstream cannot tell the two apart. A fixed limiter has
nothing to drift with.

A sample of exactly zero has no direction and passes through as zero - never a non-finite value - which is what its
phase difference with anything already is. Stateless and 1:1, so it may sit anywhere in a fused run.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    GR_MAKE_REFLECTABLE(HardLimiter, in, out);

    [[nodiscard]] constexpr T processOne(T sample) const noexcept {
        if constexpr (gr::meta::complex_like<T>) {
            const auto magnitude = std::abs(sample);
            return magnitude > typename T::value_type{0} ? sample / magnitude : T{};
        } else {
            return sample > T{0} ? T{1} : (sample < T{0} ? T{-1} : T{0});
        }
    }
};

} // namespace gr::blocks::analog

#endif // GNURADIO_HARD_LIMITER_HPP
