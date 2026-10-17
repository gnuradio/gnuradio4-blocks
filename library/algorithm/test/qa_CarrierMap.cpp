#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/algorithm/ofdm/CarrierMap.hpp>

using namespace boost::ut;
using gr::ofdm::CarrierMap;
using gr::ofdm::Occupancy;
using gr::ofdm::schmidlCoxPreamble;

namespace {

using Cplx = std::complex<float>;

/// A test numerology: 64 carriers, 52 of them occupied, 4 of those pilots.
struct Numerology {
    std::vector<int> data;
    std::vector<int> pilot{-21, -7, 7, 21};
};

[[nodiscard]] Numerology qaNumerology() {
    Numerology numerology;
    for (int carrier = -26; carrier <= 26; ++carrier) {
        if (carrier == 0 || carrier == -21 || carrier == -7 || carrier == 7 || carrier == 21) {
            continue;
        }
        numerology.data.push_back(carrier);
    }
    return numerology;
}

} // namespace

const boost::ut::suite<"CarrierMap"> carrierMapTests = [] {
    // Both ends of the range and the Nyquist bin. `+fft_len/2` is outside the range.
    "a signed carrier and its bin convert both ways"_test = [] {
        expect(eq(CarrierMap::binOf(64UZ, 0).value(), 0UZ)) << "DC is bin zero";
        expect(eq(CarrierMap::binOf(64UZ, 1).value(), 1UZ));
        expect(eq(CarrierMap::binOf(64UZ, 31).value(), 31UZ)) << "the highest positive carrier";
        expect(eq(CarrierMap::binOf(64UZ, -1).value(), 63UZ)) << "the negative carriers are the upper half";
        expect(eq(CarrierMap::binOf(64UZ, -31).value(), 33UZ));
        expect(eq(CarrierMap::binOf(64UZ, -32).value(), 32UZ)) << "-fft_len/2 is the Nyquist bin";

        expect(eq(CarrierMap::carrierOf(64UZ, 0UZ).value(), 0));
        expect(eq(CarrierMap::carrierOf(64UZ, 31UZ).value(), 31));
        expect(eq(CarrierMap::carrierOf(64UZ, 32UZ).value(), -32));
        expect(eq(CarrierMap::carrierOf(64UZ, 63UZ).value(), -1));

        for (const std::size_t fftLength : {4UZ, 8UZ, 64UZ, 128UZ, 1'024UZ}) {
            const int half = static_cast<int>(fftLength / 2UZ);
            for (int carrier = -half; carrier < half; ++carrier) {
                const std::size_t bin = CarrierMap::binOf(fftLength, carrier).value();
                expect(lt(bin, fftLength));
                expect(eq(CarrierMap::carrierOf(fftLength, bin).value(), carrier)) << std::format("fft_len {} carrier {} did not come back", fftLength, carrier);
            }
            expect(!CarrierMap::binOf(fftLength, half).has_value()) << "+fft_len/2 is not a carrier";
            expect(!CarrierMap::binOf(fftLength, -half - 1).has_value()) << "one below the lowest carrier";
            expect(!CarrierMap::carrierOf(fftLength, fftLength).has_value()) << "a bin past the transform";
        }
    };

    "the QA numerology's three sets are disjoint and complete"_test = [] {
        const Numerology numerology = qaNumerology();
        const CarrierMap map(64UZ, numerology.data, numerology.pilot);

        expect(eq(map.fftLength(), 64UZ));
        expect(eq(map.nData(), 48UZ));
        expect(eq(map.nPilots(), 4UZ));
        expect(eq(map.occupiedCount(), 52UZ)) << "64 carriers, 52 occupied, 4 pilots";
        expect(eq(map.guardCarriers().size(), 12UZ)) << "DC, -32..-27 and 27..31";
        expect(that % !map.dcOccupied()) << "this numerology guards DC";

        std::vector<int> claimed(64UZ, 0);
        for (const std::size_t bin : map.dataBins()) {
            claimed[bin] += 1;
        }
        for (const std::size_t bin : map.pilotBins()) {
            claimed[bin] += 1;
        }
        for (const std::size_t bin : map.guardBins()) {
            claimed[bin] += 1;
        }
        expect(that % std::ranges::all_of(claimed, [](int n) { return n == 1; })) << "every bin is claimed exactly once by exactly one set";

        // The bins follow the order of the carrier list.
        for (std::size_t i = 0UZ; i < map.nData(); ++i) {
            expect(eq(map.dataBins()[i], CarrierMap::binOf(64UZ, numerology.data[i]).value()));
        }
        expect(that % std::ranges::equal(map.pilotBins(), std::vector<std::size_t>{43UZ, 57UZ, 7UZ, 21UZ})) << "pilots at -21, -7, +7, +21";

        expect(that % (map.occupancyOfBin(0UZ) == Occupancy::Guard)) << "DC";
        expect(that % (map.occupancyOf(7) == Occupancy::Pilot));
        expect(that % (map.occupancyOf(1) == Occupancy::Data));
        expect(that % (map.occupancyOf(-27) == Occupancy::Guard));
        expect(that % (map.occupancyOf(-32) == Occupancy::Guard)) << "the Nyquist carrier is guard here";

        // A numerology may use DC, and `dcOccupied()` reports it.
        std::vector<int> withDc = numerology.data;
        withDc.push_back(0);
        const CarrierMap dcMap(64UZ, withDc, numerology.pilot);
        expect(that % dcMap.dcOccupied());
        expect(eq(dcMap.occupiedCount(), 53UZ));
        expect(eq(dcMap.guardCarriers().size(), 11UZ));
    };

    "the pilot cycle is read at two levels"_test = [] {
        for (const std::size_t nPilots : {1UZ, 4UZ, 7UZ}) {
            for (const std::size_t cycleLength : {1UZ, 4UZ, 6UZ, 13UZ}) {
                for (std::size_t symbol = 0UZ; symbol < 40UZ; ++symbol) {
                    for (std::size_t slot = 0UZ; slot < nPilots; ++slot) {
                        expect(eq(CarrierMap::pilotSymbolIndex(symbol, slot, nPilots, cycleLength), (symbol * nPilots + slot) % cycleLength)) << std::format("n_pilots {} len {} symbol {} slot {}", nPilots, cycleLength, symbol, slot);
                    }
                }
            }
        }

        // A cycle of 4 over 4 pilots gives every symbol the same four. A cycle of 6 rotates by two each symbol.
        expect(eq(CarrierMap::pilotSymbolIndex(0UZ, 2UZ, 4UZ, 4UZ), CarrierMap::pilotSymbolIndex(99UZ, 2UZ, 4UZ, 4UZ)));
        expect(eq(CarrierMap::pilotSymbolIndex(1UZ, 0UZ, 4UZ, 6UZ), 4UZ));
        expect(eq(CarrierMap::pilotSymbolIndex(2UZ, 0UZ, 4UZ, 6UZ), 2UZ));

        // A symbol index past 2^32 and one of 2^62 read the same entry as their residues modulo the cycle length. The
        // index is 64 bits wide. The first case therefore holds where std::size_t has 32 bits as well.
        for (const std::uint64_t huge : {(std::uint64_t{1} << 32) + std::uint64_t{5}, std::uint64_t{1} << 62}) {
            for (const std::size_t cycleLength : {3UZ, 6UZ, 13UZ}) {
                for (std::size_t slot = 0UZ; slot < 4UZ; ++slot) {
                    expect(eq(CarrierMap::pilotSymbolIndex(huge, slot, 4UZ, cycleLength), CarrierMap::pilotSymbolIndex(huge % cycleLength, slot, 4UZ, cycleLength))) << std::format("symbol {} len {} slot {}", huge, cycleLength, slot);
                }
            }
        }

        const Numerology numerology = qaNumerology();
        const CarrierMap map(64UZ, numerology.data, numerology.pilot);
        expect(eq(map.pilotSymbolIndex(3UZ, 1UZ, 8UZ), (3UZ * 4UZ + 1UZ) % 8UZ)) << "the member form reads n_pilots off the numerology";
        const std::vector<Cplx> cycle{Cplx(1.f, 0.f), Cplx(1.f, 0.f), Cplx(1.f, 0.f), Cplx(-1.f, 0.f)};
        expect(map.validatePilotCycle(cycle).has_value());
        expect(!map.validatePilotCycle(std::span<const Cplx>{}).has_value()) << "pilot carriers with no symbols to put on them";

        const CarrierMap noPilots(64UZ, numerology.data, std::span<const int>{});
        expect(noPilots.validatePilotCycle(std::span<const Cplx>{}).has_value()) << "no pilots, nothing to cycle";
    };

    "a zero or non-finite pilot value is refused"_test = [] {
        const Numerology numerology = qaNumerology();
        const CarrierMap map(64UZ, numerology.data, numerology.pilot);
        const float      nan = std::numeric_limits<float>::quiet_NaN();
        const float      inf = std::numeric_limits<float>::infinity();
        for (const Cplx bad : {Cplx(0.f, 0.f), Cplx(nan, 0.f), Cplx(0.f, inf)}) {
            const std::vector<Cplx> cycle{Cplx(1.f, 0.f), Cplx(1.f, 0.f), Cplx(1.f, 0.f), bad};
            const auto              checked = map.validatePilotCycle(cycle);
            expect(!checked.has_value()) << std::format("pilot value ({}, {})", bad.real(), bad.imag());
            if (!checked.has_value()) {
                expect(checked.error().message.find("pilot_symbols entry 3") != std::string::npos) << checked.error().message;
            }
        }
        const CarrierMap noPilots(64UZ, numerology.data, std::span<const int>{});
        expect(!noPilots.validatePilotCycle(std::vector<Cplx>{Cplx(0.f, 0.f)}).has_value()) << "a value is checked whether or not a carrier reads it";
    };

    "a numerology that does not describe a transform is refused"_test = [] {
        const std::vector<int> data{1, 2, 3};

        expect(throws<gr::exception>([&] { (void)CarrierMap(2UZ, data, std::span<const int>{}); })) << "an fft_len below four";
        expect(throws<gr::exception>([&] { (void)CarrierMap(65UZ, data, std::span<const int>{}); })) << "an odd fft_len";
        expect(throws<gr::exception>([&] { (void)CarrierMap(CarrierMap::kMaxFftLength + 2UZ, data, std::span<const int>{}); })) << "an fft_len past the ceiling";
        expect(throws<gr::exception>([] { (void)CarrierMap(64UZ, std::span<const int>{}, std::span<const int>{}); })) << "no data carriers";

        expect(throws<gr::exception>([] {
            const std::vector<int> outside{1, 32};
            (void)CarrierMap(64UZ, outside, std::span<const int>{});
        })) << "a carrier at +fft_len/2";
        expect(throws<gr::exception>([] {
            const std::vector<int> outside{-33, 1};
            (void)CarrierMap(64UZ, outside, std::span<const int>{});
        })) << "a carrier below -fft_len/2";
        expect(throws<gr::exception>([] {
            const std::vector<int> repeated{1, 2, 1};
            (void)CarrierMap(64UZ, repeated, std::span<const int>{});
        })) << "a data carrier named twice";
        expect(throws<gr::exception>([] {
            const std::vector<int> data2{1, 2};
            const std::vector<int> pilots{3, 3};
            (void)CarrierMap(64UZ, data2, pilots);
        })) << "a pilot carrier named twice";
        expect(throws<gr::exception>([] {
            const std::vector<int> data2{1, 2};
            const std::vector<int> pilots{2};
            (void)CarrierMap(64UZ, data2, pilots);
        })) << "a carrier that is both data and pilot";

        const Numerology numerology = qaNumerology();
        const CarrierMap map(64UZ, numerology.data, numerology.pilot);
        expect(map.validateSyncWord(std::vector<Cplx>(64UZ)).has_value());
        expect(!map.validateSyncWord(std::vector<Cplx>(63UZ)).has_value()) << "a sync word is a whole symbol";
        expect(!map.occupancyOfBin(64UZ).has_value());
        expect(!map.occupancyOf(32).has_value());
        expect(map.occupancyOf(32).error().message.contains("carrier 32 is outside the range -32 to 31")) << map.occupancyOf(32).error().message;
        expect(map.occupancyOfBin(64UZ).error().message.contains("bin 64 is outside a transform of 64 bins"));
        expect(map.validateSyncWord(std::vector<Cplx>(63UZ), 2UZ).error().message.contains("sync word 2 holds 63 carriers"));
        expect(CarrierMap::binOf(65UZ, 0).error().message.contains("the map needs an even length")) << "the length is checked before the carrier";
        expect(schmidlCoxPreamble(64UZ, 51UZ, 0ULL).error().message.contains("occupied is 51"));
    };

    // The test checks the half repeat through the library FFT. The kernel itself uses no transform.
    "the preamble's odd bins are empty and its time-domain halves match"_test = [] {
        constexpr std::size_t kFftLength = 64UZ;
        constexpr std::size_t kOccupied  = 52UZ;

        const std::vector<Cplx> preamble = schmidlCoxPreamble(kFftLength, kOccupied, 0xc0ffeeULL).value();
        expect(eq(preamble.size(), kFftLength));
        expect(eq(preamble[0UZ], Cplx{0.f, 0.f})) << "DC carries nothing";

        std::size_t occupiedBins = 0UZ;
        double      energy       = 0.;
        for (std::size_t bin = 0UZ; bin < kFftLength; ++bin) {
            const int carrier = CarrierMap::carrierOf(kFftLength, bin).value();
            energy += static_cast<double>(std::norm(preamble[bin]));
            if (bin % 2UZ != 0UZ) {
                expect(eq(preamble[bin], Cplx{0.f, 0.f})) << std::format("odd bin {} must be empty", bin);
                continue;
            }
            if (preamble[bin] != Cplx{0.f, 0.f}) {
                ++occupiedBins;
                expect(lt(std::abs(static_cast<double>(std::norm(preamble[bin])) - 2.), 1e-6)) << "a QPSK point scaled by sqrt(2)";
                expect(le(std::abs(carrier), 26)) << std::format("carrier {} is outside the occupied band", carrier);
                expect(neq(carrier, 0)) << "DC is not occupied";
            }
        }
        expect(eq(occupiedBins, 26UZ)) << "the even members of -26..-1 and 1..26";
        std::println("qa_CarrierMap: preamble energy {:.1f} over {} even carriers, against an occupancy of {}", energy, occupiedBins, kOccupied);
        expect(lt(std::abs(energy - static_cast<double>(kOccupied)), 1e-5)) << "the sqrt(2) scaling puts a fully occupied symbol's energy on half the carriers";

        gr::algorithm::FFT<Cplx, Cplx, gr::algorithm::Direction::Backward> inverse{};
        const std::vector<Cplx>                                            time = inverse.compute(preamble);
        expect(eq(time.size(), kFftLength));

        double worst = 0.;
        double scale = 0.;
        for (std::size_t n = 0UZ; n < kFftLength / 2UZ; ++n) {
            worst = std::max(worst, static_cast<double>(std::abs(time[n] - time[n + kFftLength / 2UZ])));
            scale = std::max(scale, static_cast<double>(std::abs(time[n])));
        }
        std::println("qa_CarrierMap: the two time-domain halves differ by at most {:.3e} against a peak of {:.3f}", worst, scale);
        expect(lt(worst, 1e-5 * scale)) << "the upper half must repeat the lower half";

        // One value on an odd bin breaks the repeat.
        std::vector<Cplx> spoiled            = preamble;
        spoiled[3UZ]                         = Cplx{1.f, 0.f};
        const std::vector<Cplx> spoiledTime  = inverse.compute(spoiled);
        double                  spoiledWorst = 0.;
        for (std::size_t n = 0UZ; n < kFftLength / 2UZ; ++n) {
            spoiledWorst = std::max(spoiledWorst, static_cast<double>(std::abs(spoiledTime[n] - spoiledTime[n + kFftLength / 2UZ])));
        }
        expect(gt(spoiledWorst, 1.)) << "one occupied odd bin breaks the repeat";

        // Another seed and other lengths repeat as well.
        for (const std::size_t fftLength : {32UZ, 64UZ, 256UZ}) {
            const std::vector<Cplx> other      = schmidlCoxPreamble(fftLength, fftLength - 12UZ, 7ULL).value();
            const std::vector<Cplx> otherTime  = inverse.compute(other);
            double                  otherWorst = 0.;
            double                  otherScale = 0.;
            for (std::size_t n = 0UZ; n < fftLength / 2UZ; ++n) {
                otherWorst = std::max(otherWorst, static_cast<double>(std::abs(otherTime[n] - otherTime[n + fftLength / 2UZ])));
                otherScale = std::max(otherScale, static_cast<double>(std::abs(otherTime[n])));
            }
            expect(lt(otherWorst, 1e-5 * otherScale)) << std::format("fft_len {} did not repeat", fftLength);
        }

        expect(that % !std::ranges::equal(preamble, schmidlCoxPreamble(kFftLength, kOccupied, 1ULL).value())) << "a different seed gives a different preamble";
        expect(that % std::ranges::equal(preamble, schmidlCoxPreamble(kFftLength, kOccupied, 0xc0ffeeULL).value())) << "the same seed gives the same preamble";
    };

    "a preamble that cannot be built is refused"_test = [] {
        expect(!schmidlCoxPreamble(63UZ, 52UZ, 0ULL).has_value()) << "an odd fft_len";
        expect(!schmidlCoxPreamble(2UZ, 2UZ, 0ULL).has_value()) << "an fft_len below four";
        expect(!schmidlCoxPreamble(64UZ, 51UZ, 0ULL).has_value()) << "an odd occupancy";
        expect(!schmidlCoxPreamble(64UZ, 2UZ, 0ULL).has_value()) << "too few to hold an even carrier each side";
        expect(!schmidlCoxPreamble(64UZ, 64UZ, 0ULL).has_value()) << "an occupancy that leaves no room for DC";
        expect(schmidlCoxPreamble(64UZ, 62UZ, 0ULL).has_value()) << "the widest occupancy that does";
    };
};

int main() { /* not needed for UT */ }
