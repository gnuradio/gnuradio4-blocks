#ifndef GNURADIO_ALGORITHM_FEC_GF2_POLYNOMIAL_HPP
#define GNURADIO_ALGORITHM_FEC_GF2_POLYNOMIAL_HPP

#include <cstdint>

namespace gr::fec {

/**
 * @brief The remainder of a polynomial over GF(2) divided by a generator, both as bit fields in which
 *        bit i is the coefficient of x^i.
 *
 * @p word holds @p length coefficients, bits `length - 1` down to zero. @p generator has degree
 * @p degree, its bit @p degree set. Every bit from `length - 1` down to @p degree is cleared by
 * subtracting the generator shifted under it, and the low @p degree bits are the remainder. A
 * systematic cyclic encoder appends `gf2Remainder(info << degree, ...)` to the information; a
 * syndrome is the remainder of a received word. @p length is at most 64 and @p degree below 64.
 */
[[nodiscard]] inline constexpr std::uint64_t gf2Remainder(std::uint64_t word, unsigned length, std::uint64_t generator, unsigned degree) noexcept {
    for (unsigned bit = length; bit-- > degree;) {
        if (((word >> bit) & 1U) != 0U) {
            word ^= generator << (bit - degree);
        }
    }
    return word & ((std::uint64_t{1} << degree) - 1U);
}

} // namespace gr::fec

#endif // GNURADIO_ALGORITHM_FEC_GF2_POLYNOMIAL_HPP
