#ifndef GNURADIO_ALGORITHM_CARRIER_MAP_HPP
#define GNURADIO_ALGORITHM_CARRIER_MAP_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <source_location>
#include <span>
#include <vector>

#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/algorithm/rng/Xoshiro256pp.hpp>

namespace gr::ofdm {

namespace detail {
/// The bin of a carrier already checked against the transform length.
[[nodiscard]] constexpr std::size_t binOfCarrier(std::size_t fftLength, int carrier) noexcept { return (carrier < 0) ? static_cast<std::size_t>(carrier + static_cast<int>(fftLength)) : static_cast<std::size_t>(carrier); }

/// The carrier of a bin already checked against the transform length.
[[nodiscard]] constexpr int carrierOfBin(std::size_t fftLength, std::size_t bin) noexcept {
    const int half  = static_cast<int>(fftLength / 2UZ);
    const int index = static_cast<int>(bin);
    return (index >= half) ? index - static_cast<int>(fftLength) : index;
}
} // namespace detail

/// The role of a carrier. A guard is every carrier the numerology does not name as data or pilot.
enum class Occupancy : std::uint8_t { Guard, Data, Pilot };

/**
 * @brief One OFDM numerology, with signed carrier indices resolved once into FFT bin order.
 *
 * A numerology names carriers by signed logical index. DC is 0, the carrier one spacing above it is +1, and the one
 * below it is -1. The range is `-fft_len/2` to `fft_len/2 - 1`. A transform holds the same symbol in bin order. DC is
 * at index 0, the positive carriers ascend from index 1, and the negative carriers fill the upper half. The map is
 * `bin = carrier` for a non-negative carrier and `bin = carrier + fft_len` for a negative one.
 *
 * The range is asymmetric. `-fft_len/2` is a carrier and falls on the Nyquist bin `fft_len/2`. `+fft_len/2` is outside
 * the range, and a numerology that names it is refused.
 *
 * A numerology is a data set and a pilot set. Every other carrier is a guard. The three sets are therefore disjoint and
 * cover all `fft_len` bins. The constructor checks that every carrier is in range, that neither set names a carrier
 * twice, and that the two sets share none. DC is an ordinary carrier. A numerology may use it or leave it as a guard,
 * and `dcOccupied()` reports which. The constructor throws `gr::exception` for a numerology it refuses. Every other
 * function that checks an argument returns a `std::expected` with a `gr::Error`.
 *
 * The class holds the addressing alone, and it does not change after construction.
 */
class CarrierMap {
public:
    /// The longest transform accepted. It bounds the per-bin tables a numerology allocates.
    static constexpr std::size_t kMaxFftLength = std::size_t{1} << 16;

    /// The FFT bin of a signed carrier index. The map depends on the transform length alone.
    [[nodiscard]] static std::expected<std::size_t, gr::Error> binOf(std::size_t fftLength, int carrier, std::source_location loc = std::source_location::current()) {
        return requireLength(fftLength, loc).and_then([&] { return requireCarrier(fftLength, carrier, loc); }).transform([&] { return detail::binOfCarrier(fftLength, carrier); });
    }

    /// The signed carrier index of an FFT bin.
    [[nodiscard]] static std::expected<int, gr::Error> carrierOf(std::size_t fftLength, std::size_t bin, std::source_location loc = std::source_location::current()) {
        if (auto length = requireLength(fftLength, loc); !length) {
            return std::unexpected(length.error());
        }
        if (bin >= fftLength) {
            return std::unexpected(gr::Error(std::format("gr::ofdm::CarrierMap: bin {} is outside a transform of {} bins", bin, fftLength), loc));
        }
        return detail::carrierOfBin(fftLength, bin);
    }

    CarrierMap(std::size_t fftLength, std::span<const int> dataCarriers, std::span<const int> pilotCarriers) : _fftLength(fftLength), _data(dataCarriers.begin(), dataCarriers.end()), _pilot(pilotCarriers.begin(), pilotCarriers.end()) {
        if (auto length = requireLength(fftLength); !length) {
            throw gr::exception(length.error().message);
        }
        if (_data.empty()) {
            throw gr::exception("gr::ofdm::CarrierMap: data_carriers is empty; a numerology needs at least one data carrier");
        }

        _occupancy.assign(fftLength, Occupancy::Guard);
        _dataBins.reserve(_data.size());
        _pilotBins.reserve(_pilot.size());

        claim(_data, _dataBins, Occupancy::Data, "data_carriers");
        claim(_pilot, _pilotBins, Occupancy::Pilot, "pilot_carriers");

        for (std::size_t bin = 0UZ; bin < fftLength; ++bin) {
            if (_occupancy[bin] == Occupancy::Guard) {
                _guard.push_back(detail::carrierOfBin(fftLength, bin));
                _guardBins.push_back(bin);
            }
        }
        _dcOccupied = _occupancy[0UZ] != Occupancy::Guard;
    }

    [[nodiscard]] std::size_t fftLength() const noexcept { return _fftLength; }
    [[nodiscard]] std::size_t nData() const noexcept { return _data.size(); }
    [[nodiscard]] std::size_t nPilots() const noexcept { return _pilot.size(); }
    [[nodiscard]] std::size_t occupiedCount() const noexcept { return _data.size() + _pilot.size(); }
    [[nodiscard]] bool        dcOccupied() const noexcept { return _dcOccupied; }

    [[nodiscard]] std::span<const int> dataCarriers() const noexcept { return _data; }
    [[nodiscard]] std::span<const int> pilotCarriers() const noexcept { return _pilot; }

    /// The guard carriers, ascending in signed index.
    [[nodiscard]] std::span<const int> guardCarriers() const noexcept { return _guard; }

    /// The bin of each data carrier, in `dataCarriers()` order.
    [[nodiscard]] std::span<const std::size_t> dataBins() const noexcept { return _dataBins; }
    [[nodiscard]] std::span<const std::size_t> pilotBins() const noexcept { return _pilotBins; }
    [[nodiscard]] std::span<const std::size_t> guardBins() const noexcept { return _guardBins; }

    /// The role of a bin. A bin outside the transform returns an error.
    [[nodiscard]] std::expected<Occupancy, gr::Error> occupancyOfBin(std::size_t bin, std::source_location loc = std::source_location::current()) const {
        if (bin >= _fftLength) {
            return std::unexpected(gr::Error(std::format("gr::ofdm::CarrierMap: bin {} is outside a transform of {} bins", bin, _fftLength), loc));
        }
        return _occupancy[bin];
    }

    [[nodiscard]] std::expected<Occupancy, gr::Error> occupancyOf(int carrier, std::source_location loc = std::source_location::current()) const {
        return requireCarrier(_fftLength, carrier, loc).transform([&] { return _occupancy[detail::binOfCarrier(_fftLength, carrier)]; });
    }

    /// Checks that a sync word holds one value per bin. A sync word is a whole symbol.
    [[nodiscard]] std::expected<void, gr::Error> validateSyncWord(std::span<const std::complex<float>> word, std::size_t index = 0UZ, std::source_location loc = std::source_location::current()) const {
        if (word.size() != _fftLength) {
            return std::unexpected(gr::Error(std::format("gr::ofdm::CarrierMap: sync word {} holds {} carriers, not the {} a symbol has", index, word.size(), _fftLength), loc));
        }
        return {};
    }

    /// Refuses an empty pilot-symbol cycle for a numerology with pilot carriers, and a zero or non-finite pilot value.
    /// `pilotSymbolIndex` makes no check of its own.
    [[nodiscard]] std::expected<void, gr::Error> validatePilotCycle(std::span<const std::complex<float>> cycle, std::source_location loc = std::source_location::current()) const {
        if (!_pilot.empty() && cycle.empty()) {
            return std::unexpected(gr::Error("gr::ofdm::CarrierMap: pilot_symbols is empty while the numerology has pilot carriers", loc));
        }
        for (std::size_t k = 0UZ; k < cycle.size(); ++k) {
            const std::complex<float> value = cycle[k];
            if (!std::isfinite(value.real()) || !std::isfinite(value.imag()) || value == std::complex<float>{}) {
                return std::unexpected(gr::Error(std::format("gr::ofdm::CarrierMap: pilot_symbols entry {} is ({}, {}); a pilot value must be finite and nonzero", k, value.real(), value.imag()), loc));
            }
        }
        return {};
    }

    /**
     * @brief The entry of the pilot-symbol vector that symbol `symbolIndex` reads for pilot slot `pilotSlot`.
     *
     * The slots of a symbol take consecutive entries. The next symbol continues where the previous one stopped, which
     * gives `pilot_symbols[(s * n_pilots + p) % len]`. A cycle whose length is a multiple of `n_pilots` gives every
     * symbol the same pilots. Any other length rotates them from symbol to symbol. `cycleLength` is above 0.
     *
     * `symbolIndex` is 64 bits wide on every target, and the entry is exact for every value. The function reduces `s`
     * modulo the cycle length before the multiply.
     */
    [[nodiscard]] static constexpr std::size_t pilotSymbolIndex(std::uint64_t symbolIndex, std::size_t pilotSlot, std::size_t nPilots, std::size_t cycleLength) noexcept {
        const std::uint64_t length = cycleLength;
        const std::uint64_t entry  = ((symbolIndex % length) * nPilots + pilotSlot) % length;
        return static_cast<std::size_t>(entry);
    }

    [[nodiscard]] constexpr std::size_t pilotSymbolIndex(std::uint64_t symbolIndex, std::size_t pilotSlot, std::size_t cycleLength) const noexcept { return pilotSymbolIndex(symbolIndex, pilotSlot, _pilot.size(), cycleLength); }

private:
    std::size_t              _fftLength{0UZ};
    std::vector<int>         _data{};
    std::vector<int>         _pilot{};
    std::vector<int>         _guard{};
    std::vector<std::size_t> _dataBins{};
    std::vector<std::size_t> _pilotBins{};
    std::vector<std::size_t> _guardBins{};
    std::vector<Occupancy>   _occupancy{};
    bool                     _dcOccupied{false};

    static std::expected<void, gr::Error> requireLength(std::size_t fftLength, std::source_location loc = std::source_location::current()) {
        if (fftLength < 4UZ || fftLength > kMaxFftLength) {
            return std::unexpected(gr::Error(std::format("gr::ofdm::CarrierMap: fft_len is {}; a transform holds between 4 and {} bins", fftLength, kMaxFftLength), loc));
        }
        if (fftLength % 2UZ != 0UZ) {
            return std::unexpected(gr::Error(std::format("gr::ofdm::CarrierMap: fft_len is {}; the map needs an even length", fftLength), loc));
        }
        return {};
    }

    static std::expected<void, gr::Error> requireCarrier(std::size_t fftLength, int carrier, std::source_location loc = std::source_location::current()) {
        const int half = static_cast<int>(fftLength / 2UZ);
        if (carrier < -half || carrier >= half) {
            return std::unexpected(gr::Error(std::format("gr::ofdm::CarrierMap: carrier {} is outside the range {} to {} an fft_len of {} spans", carrier, -half, half - 1, fftLength), loc));
        }
        return {};
    }

    void claim(std::span<const int> carriers, std::vector<std::size_t>& bins, Occupancy what, const char* parameter) {
        for (const int carrier : carriers) {
            if (auto inRange = requireCarrier(_fftLength, carrier); !inRange) {
                throw gr::exception(inRange.error().message);
            }
            const std::size_t bin = detail::binOfCarrier(_fftLength, carrier);
            if (_occupancy[bin] != Occupancy::Guard) {
                throw gr::exception(std::format("gr::ofdm::CarrierMap: {} names carrier {} twice, or after the other set already claimed it", parameter, carrier));
            }
            _occupancy[bin] = what;
            bins.push_back(bin);
        }
    }
};

/**
 * @brief A Schmidl-Cox preamble: one frequency-domain symbol whose time-domain upper half repeats its lower half.
 *
 * The inverse transform gives
 *
 *     x[n + N/2] = (1/N) * sum_k X[k] * exp(j*2*pi*k*n/N) * exp(j*pi*k)
 *
 * `exp(j*pi*k)` is `+1` on every even `k` and `-1` on every odd `k`. A symbol whose odd bins are all zero therefore has
 * `x[n + N/2] = x[n]` for every `n`, whatever values the even bins hold. The function occupies even carriers and leaves
 * the odd ones empty. `fftLength` is even. A carrier and its bin therefore have the same parity.
 *
 * The occupied band is the `occupied` carriers closest to DC: `-occupied/2` to `-1` and `1` to `occupied/2`. DC
 * carries nothing. Each even carrier of the band takes a QPSK point scaled by `sqrt(2)`, twice the power a fully
 * occupied symbol puts on one carrier. The preamble's total energy is `2 * evenCarrierCount`. It equals `occupied` when
 * `occupied/2` is even.
 *
 * The points come from a `gr::rng::Xoshiro256pp` seeded with `seed`. The same seed gives the same preamble.
 */
[[nodiscard]] inline std::expected<std::vector<std::complex<float>>, gr::Error> schmidlCoxPreamble(std::size_t fftLength, std::size_t occupied, std::uint64_t seed, std::source_location loc = std::source_location::current()) {
    if (fftLength < 4UZ || fftLength > CarrierMap::kMaxFftLength || fftLength % 2UZ != 0UZ) {
        return std::unexpected(gr::Error(std::format("gr::ofdm::schmidlCoxPreamble: fft_len is {}; it must be even and between 4 and {}", fftLength, CarrierMap::kMaxFftLength), loc));
    }
    if (occupied < 4UZ || occupied % 2UZ != 0UZ) {
        return std::unexpected(gr::Error(std::format("gr::ofdm::schmidlCoxPreamble: occupied is {}; it must be even and at least 4, so that each side of DC holds an even carrier", occupied), loc));
    }
    if (occupied > fftLength - 2UZ) {
        return std::unexpected(gr::Error(std::format("gr::ofdm::schmidlCoxPreamble: occupied is {}, more than an fft_len of {} holds with DC and the Nyquist bin left empty", occupied, fftLength), loc));
    }

    gr::rng::Xoshiro256pp            rng(seed);
    std::vector<std::complex<float>> symbol(fftLength, std::complex<float>{0.f, 0.f});
    const int                        half = static_cast<int>(occupied / 2UZ);
    for (int carrier = -half; carrier <= half; ++carrier) {
        if (carrier == 0 || (carrier % 2) != 0) {
            continue;
        }
        const std::uint64_t bits                         = rng();
        const float         real                         = ((bits & 1ULL) != 0ULL) ? 1.f : -1.f;
        const float         imag                         = ((bits & 2ULL) != 0ULL) ? 1.f : -1.f;
        symbol[detail::binOfCarrier(fftLength, carrier)] = std::complex<float>{real, imag};
    }
    return symbol;
}

} // namespace gr::ofdm

#endif // GNURADIO_ALGORITHM_CARRIER_MAP_HPP
