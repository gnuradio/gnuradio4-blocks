#ifndef GNURADIO_CONSTELLATION_SETTINGS_HPP
#define GNURADIO_CONSTELLATION_SETTINGS_HPP

#include <concepts>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string_view>

#include <gnuradio-4.0/annotated.hpp>
#include <gnuradio-4.0/meta/utils.hpp>

#include <gnuradio-4.0/algorithm/digital/Constellation.hpp>

namespace gr::blocks::digital::detail {

/**
 * @brief The constellation as a setting - a name and its parameters - rather than a shared mutable object.
 *
 * Each block owns its own value, built once per settings change from `constellation`, `arity`, `phase_offset`,
 * `label_xor`, `points` and `normalization`, so there is nothing to share between blocks and nothing to reallocate
 * while `work()` is reading it. The naming itself lives beside the kernel in the library; what this adds is the
 * settings types the blocks carry and the failure a block reports, since a graph reports a bad setting as a
 * `gr::exception` rather than as the library's own argument error.
 */
template<std::floating_point F>
[[nodiscard]] gr::digital::Constellation<F> build(std::string_view name, gr::Size_t arity, F phaseOffset, gr::Size_t labelXor, std::span<const F> interleavedPoints, std::string_view normalizationName) {
    try {
        return gr::digital::constellationFromName<F>(name, static_cast<std::size_t>(arity), phaseOffset, static_cast<std::uint8_t>(labelXor), interleavedPoints, normalizationName);
    } catch (const std::invalid_argument& error) {
        throw gr::exception(error.what());
    }
}

/// @brief The shared documentation of the six settings that name a constellation, quoted by each block that carries them.
using ConstellationSettingsDoc = Doc<R""(
`constellation` names one of `bpsk`, `qpsk`, `psk8`, `psk`, `qam` or `custom`. `arity` is M and is read by `psk` and
`qam` only; `phase_offset` and `label_xor` are read by `psk` only; `points` is the interleaved re,im list `custom`
reads. `normalization` is `power`, `amplitude` or `none`, and `power` is the default: the points are scaled to unit mean
power rather than to unit peak amplitude, which differ by 0.471 dB on 16QAM and not at all on any
constant-modulus constellation.
)"">;

} // namespace gr::blocks::digital::detail

#endif // GNURADIO_CONSTELLATION_SETTINGS_HPP
