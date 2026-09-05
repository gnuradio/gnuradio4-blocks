#ifndef GNURADIO_DIGITAL_DIFFERENTIAL_CODING_HPP
#define GNURADIO_DIGITAL_DIFFERENTIAL_CODING_HPP

#include <algorithm>
#include <array>
#include <bit>
#include <complex>
#include <concepts>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <string_view>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::digital {

/// @brief A symbol alphabet up to 65536 wide; wider is unregistered and would overflow the 32-bit working type.
template<typename T>
concept SymbolLike = std::unsigned_integral<T> && sizeof(T) <= sizeof(std::uint16_t);

namespace detail {

using Symbol = gr::Size_t;

enum class Coding : std::uint8_t { Differential, Nrzi };

[[nodiscard]] inline Coding codingFromName(std::string_view name) {
    if (name == "differential") {
        return Coding::Differential;
    }
    if (name == "nrzi") {
        return Coding::Nrzi;
    }
    throw gr::exception(std::format("coding must be 'differential' or 'nrzi', got '{}'", name));
}

/**
 * @brief The configuration both halves of the code share, and the one word of state each carries.
 *
 * `binary` marks a power-of-two `M`, where reducing an input symbol and reducing a difference are the same mask, so the
 * recursion needs no comparison and the decoder's difference needs no bias. Off a power of two the general path costs
 * one predicated subtract per symbol, plus one divide for a symbol that arrives out of range.
 */
struct CodingState {
    Symbol modulus = 2U;
    Symbol mask    = 1U;
    Symbol state   = 0U;
    Coding coding  = Coding::Differential;
    bool   binary  = true;
};

inline void configure(CodingState& coding, gr::Size_t modulus, std::string_view codingName, gr::Size_t initialState, std::uint64_t alphabet) {
    if (modulus < 2U) {
        throw gr::exception(std::format("modulus is an alphabet size and must be at least 2, got {}", modulus));
    }
    if (static_cast<std::uint64_t>(modulus) > alphabet) {
        throw gr::exception(std::format("modulus {} exceeds the {} values the sample type can hold", modulus, alphabet));
    }
    const Coding mode = codingFromName(codingName);
    if (mode == Coding::Nrzi && modulus != 2U) {
        throw gr::exception(std::format("coding 'nrzi' is a binary code and requires modulus 2, got {}", modulus));
    }
    if (initialState >= modulus) {
        throw gr::exception(std::format("initial_state {} must be below modulus {}", initialState, modulus));
    }

    coding.modulus = modulus;
    coding.mask    = modulus - 1U;
    coding.binary  = std::has_single_bit(modulus);
    coding.coding  = mode;
    coding.state   = initialState;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::digital::DifferentialEncoder, [T], [ std::uint8_t, std::uint16_t ])

template<SymbolLike T>
struct DifferentialEncoder : Block<DifferentialEncoder<T>, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief Puts the information in the change between symbols: `y[n] = (x[n] + y[n-1]) mod M`.

A coherent receiver recovers the carrier only up to a rotation, and differencing consecutive symbols cancels that
unknown constant. It costs one symbol of latency and a doubling of the symbol error rate.

The labeling must be rotation-ordered, `c[(s+1) mod M] = c[s] * exp(j*2*pi/M)`, which a Gray labeling is not for
`M >= 4`; the Gray permutation therefore belongs on the data ahead of this block, as `SymbolMap`. At `M = 2` the two
coincide. An input symbol at or above `modulus` is reduced rather than rejected, because a stream value must not be
able to stop a graph.

The block is 1:1, so every input tag key passes through at its own offset.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "modulus", Unit<"symbols">, Doc<"alphabet size M: 2 for BPSK, 4 for QPSK, 8 for 8PSK; exact for every M in [2, 2^bits(T)]">> modulus       = 2U;
    Annotated<std::string, "coding", Doc<"'differential', or 'nrzi' which is XNOR and transitions on a zero input bit">>                               coding        = std::string("differential");
    Annotated<gr::Size_t, "initial_state", Doc<"y[-1]; must be below modulus and must match the decoder's">>                                           initial_state = 0U;

    GR_MAKE_REFLECTABLE(DifferentialEncoder, in, out, modulus, coding, initial_state);

    detail::CodingState _coding{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { detail::configure(_coding, modulus, coding, initial_state, std::uint64_t{1} << (8UZ * sizeof(T))); }

    void reset() { _coding.state = initial_state; }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        const std::size_t nSamples = std::min(input.size(), output.size());
        detail::Symbol    state    = _coding.state;

        if (_coding.coding == detail::Coding::Nrzi) {
            for (std::size_t i = 0UZ; i < nSamples; ++i) {
                state     = ~(static_cast<detail::Symbol>(input[i]) ^ state) & detail::Symbol{1};
                output[i] = static_cast<T>(state);
            }
        } else if (_coding.binary) {
            const detail::Symbol mask = _coding.mask;
            for (std::size_t i = 0UZ; i < nSamples; ++i) {
                state     = (state + static_cast<detail::Symbol>(input[i])) & mask;
                output[i] = static_cast<T>(state);
            }
        } else {
            const detail::Symbol M = _coding.modulus;
            for (std::size_t i = 0UZ; i < nSamples; ++i) {
                detail::Symbol symbol = static_cast<detail::Symbol>(input[i]);
                if (symbol >= M) {
                    symbol %= M;
                }
                state += symbol;
                if (state >= M) {
                    state -= M;
                }
                output[i] = static_cast<T>(state);
            }
        }

        _coding.state = state;
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::digital::DifferentialDecoder, [T], [ std::uint8_t, std::uint16_t ])

template<SymbolLike T>
struct DifferentialDecoder : Block<DifferentialDecoder<T>, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief Recovers the differentially coded stream: `x[n] = (y[n] - y[n-1]) mod M`, exact for every M.

The inverse of DifferentialEncoder, and exact from sample 0 when both carry the same `initial_state`. `mod` here is
the mathematical modulus, so the reduction is written `(M + y[n] - y[n-1]) mod M`: the bias makes the operand
non-negative, and without it the unsigned `%` is right only when `M` is a power of two while the encoder stays
correct.

An input symbol at or above `modulus` is reduced rather than rejected. See DifferentialEncoder for `coding` and for
the rotation-ordered labeling the phase-ambiguity argument requires.

The block is 1:1, so every input tag key passes through at its own offset.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "modulus", Unit<"symbols">, Doc<"alphabet size M: 2 for BPSK, 4 for QPSK, 8 for 8PSK; exact for every M in [2, 2^bits(T)]">> modulus       = 2U;
    Annotated<std::string, "coding", Doc<"'differential', or 'nrzi' which is XNOR and transitions on a zero input bit">>                               coding        = std::string("differential");
    Annotated<gr::Size_t, "initial_state", Doc<"y[-1]; must be below modulus and must match the encoder's">>                                           initial_state = 0U;

    GR_MAKE_REFLECTABLE(DifferentialDecoder, in, out, modulus, coding, initial_state);

    detail::CodingState _coding{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { detail::configure(_coding, modulus, coding, initial_state, std::uint64_t{1} << (8UZ * sizeof(T))); }

    void reset() { _coding.state = initial_state; }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        const std::size_t nSamples = std::min(input.size(), output.size());
        if (nSamples == 0UZ) {
            return work::Status::OK;
        }

        if (_coding.coding == detail::Coding::Nrzi) {
            output[0UZ] = static_cast<T>(~(static_cast<detail::Symbol>(input[0UZ]) ^ _coding.state) & detail::Symbol{1});
            for (std::size_t i = 1UZ; i < nSamples; ++i) {
                output[i] = static_cast<T>(~(static_cast<detail::Symbol>(input[i]) ^ static_cast<detail::Symbol>(input[i - 1UZ])) & detail::Symbol{1});
            }
            _coding.state = static_cast<detail::Symbol>(input[nSamples - 1UZ]) & detail::Symbol{1};
        } else if (_coding.binary) {
            const detail::Symbol mask = _coding.mask;
            output[0UZ]               = static_cast<T>((static_cast<detail::Symbol>(input[0UZ]) - _coding.state) & mask);
            for (std::size_t i = 1UZ; i < nSamples; ++i) {
                output[i] = static_cast<T>((static_cast<detail::Symbol>(input[i]) - static_cast<detail::Symbol>(input[i - 1UZ])) & mask);
            }
            _coding.state = static_cast<detail::Symbol>(input[nSamples - 1UZ]) & mask;
        } else {
            const detail::Symbol              M     = _coding.modulus;
            constexpr std::size_t             kTile = 256UZ;
            std::array<detail::Symbol, kTile> reduced;
            detail::Symbol                    previous = _coding.state;

            for (std::size_t base = 0UZ; base < nSamples; base += kTile) {
                const std::size_t count = std::min(kTile, nSamples - base);
                for (std::size_t i = 0UZ; i < count; ++i) {
                    detail::Symbol symbol = static_cast<detail::Symbol>(input[base + i]);
                    if (symbol >= M) {
                        symbol %= M;
                    }
                    reduced[i] = symbol;
                }

                const detail::Symbol first = reduced[0UZ] + M - previous;
                output[base]               = static_cast<T>(first >= M ? first - M : first);
                for (std::size_t i = 1UZ; i < count; ++i) {
                    const detail::Symbol difference = reduced[i] + M - reduced[i - 1UZ];
                    output[base + i]                = static_cast<T>(difference >= M ? difference - M : difference);
                }
                previous = reduced[count - 1UZ];
            }
            _coding.state = previous;
        }

        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::digital::DifferentialPhasor, [T], [float])

template<std::floating_point T>
struct DifferentialPhasor : Block<DifferentialPhasor<T>, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief Differences the phase in the signal domain: `y[n] = x[n] * conj(x[n-1])`.

The non-coherent DPSK front end. A constant carrier offset cancels and the receiver needs no carrier loop, at a cost
of about 3 dB against the coherent one. Placed before the slicer it replaces the pairing of a Costas loop with
DifferentialDecoder rather than adding to it.

There is no normalization: the output magnitude is `|x[n]| * |x[n-1]|`, a product and not a square. A PSK slicer
decides on angle and does not care; anything that decides on radius needs an AGC between it and this block. The first
output is `x[0] * conj(0)`, exactly `0` by a stated rule; `reset()` restores it.

The block is 1:1, so every input tag key passes through at its own offset.
)"">;

    PortIn<std::complex<T>>  in;
    PortOut<std::complex<T>> out;

    GR_MAKE_REFLECTABLE(DifferentialPhasor, in, out);

    std::complex<T> _previous{};

    void reset() { _previous = std::complex<T>{}; }

    [[nodiscard]] work::Status processBulk(std::span<const std::complex<T>> input, std::span<std::complex<T>> output) noexcept {
        const std::size_t nSamples = std::min(input.size(), output.size());
        if (nSamples == 0UZ) {
            return work::Status::OK;
        }

        {
            const std::complex<T> sample = input[0UZ];
            output[0UZ]                  = std::complex<T>(sample.real() * _previous.real() + sample.imag() * _previous.imag(), sample.imag() * _previous.real() - sample.real() * _previous.imag());
        }

        const T* samples  = reinterpret_cast<const T*>(input.data());
        T*       products = reinterpret_cast<T*>(output.data());
        for (std::size_t i = 1UZ; i < nSamples; ++i) {
            const T sampleReal      = samples[2UZ * i];
            const T sampleImag      = samples[2UZ * i + 1UZ];
            const T priorReal       = samples[2UZ * i - 2UZ];
            const T priorImag       = samples[2UZ * i - 1UZ];
            products[2UZ * i]       = sampleReal * priorReal + sampleImag * priorImag;
            products[2UZ * i + 1UZ] = sampleImag * priorReal - sampleReal * priorImag;
        }

        _previous = input[nSamples - 1UZ];
        return work::Status::OK;
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_DIFFERENTIAL_CODING_HPP
