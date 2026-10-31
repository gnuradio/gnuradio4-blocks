#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/digital/DifferentialCoding.hpp>

namespace {

using gr::blocks::digital::DifferentialDecoder;
using gr::blocks::digital::DifferentialEncoder;
using gr::blocks::digital::DifferentialPhasor;

using CF = std::complex<float>;

constexpr double      kPi    = std::numbers::pi;
constexpr std::size_t kEqual = std::numeric_limits<std::size_t>::max();

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }

    [[nodiscard]] std::uint32_t below(std::uint32_t bound) noexcept { return static_cast<std::uint32_t>(next() % bound); }
};

template<typename T>
[[nodiscard]] std::vector<T> randomSymbols(std::size_t count, std::uint32_t bound, std::uint64_t seed = 0x9e3779b97f4a7c15ULL) {
    Rng            rng{seed};
    std::vector<T> symbols(count);
    for (T& symbol : symbols) {
        symbol = static_cast<T>(rng.below(bound));
    }
    return symbols;
}

template<typename T, typename TBlock>
[[nodiscard]] std::vector<T> drive(TBlock& block, std::span<const T> input, std::size_t chunkSize = 0UZ) {
    std::vector<T>    output(input.size());
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    for (std::size_t base = 0UZ; base < input.size(); base += stride) {
        const std::size_t count = std::min(stride, input.size() - base);
        std::ignore             = block.processBulk(input.subspan(base, count), std::span<T>(output.data() + base, count));
    }
    return output;
}

template<typename T>
[[nodiscard]] std::size_t firstDifference(std::span<const T> a, std::span<const T> b) {
    if (a.size() != b.size()) {
        return std::min(a.size(), b.size());
    }
    for (std::size_t i = 0UZ; i < a.size(); ++i) {
        if (a[i] != b[i]) {
            return i;
        }
    }
    return kEqual;
}

/// @brief The mathematical modulus, written the long way so the test does not share the block's expression.
[[nodiscard]] std::uint32_t modulo(std::int64_t value, std::uint32_t modulus) {
    const std::int64_t m         = static_cast<std::int64_t>(modulus);
    const std::int64_t remainder = value % m;
    return static_cast<std::uint32_t>(remainder < 0 ? remainder + m : remainder);
}

/// @brief The naive difference: promoted to `unsigned int`, reduced, then stored into a byte.
[[nodiscard]] std::uint32_t promotedDifference(std::uint32_t a, std::uint32_t b, std::uint32_t modulus) {
    const std::uint32_t promoted = static_cast<std::uint32_t>(static_cast<int>(a) - static_cast<int>(b));
    return (promoted % modulus) & 0xffU;
}

[[nodiscard]] constexpr std::uint32_t gray(std::uint32_t index) noexcept { return index ^ (index >> 1U); }

[[nodiscard]] constexpr std::uint32_t ungray(std::uint32_t label) noexcept {
    for (std::uint32_t shift = 1U; shift < 32U; shift <<= 1U) {
        label ^= label >> shift;
    }
    return label;
}

/// @brief A rotation-ordered PSK constellation is `exp(j*2*pi*k/M)`; the Gray-labeled one puts label `L` at `ungray(L)`.
[[nodiscard]] CF psk(std::uint32_t index, std::uint32_t modulus) {
    const double angle = 2.0 * kPi * static_cast<double>(index) / static_cast<double>(modulus);
    return CF(static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle)));
}

[[nodiscard]] std::uint32_t decideIndex(CF sample, std::uint32_t modulus) {
    const double turns = std::atan2(static_cast<double>(sample.imag()), static_cast<double>(sample.real())) * static_cast<double>(modulus) / (2.0 * kPi);
    return modulo(std::llround(turns), modulus);
}

[[nodiscard]] std::uint32_t positionOf(std::uint32_t label, bool grayLabeled) { return grayLabeled ? ungray(label) : label; }

[[nodiscard]] std::uint32_t labelOf(std::uint32_t position, bool grayLabeled) { return grayLabeled ? gray(position) : position; }

/// @brief The permutation a `+2*pi/M` rotation of the whole constellation induces on the label.
[[nodiscard]] std::vector<std::uint32_t> rotationPermutation(std::uint32_t modulus, bool grayLabeled) {
    std::vector<std::uint32_t> permutation(modulus);
    for (std::uint32_t label = 0U; label < modulus; ++label) {
        const CF rotated   = psk(positionOf(label, grayLabeled), modulus) * psk(1U, modulus);
        permutation[label] = labelOf(decideIndex(rotated, modulus), grayLabeled);
    }
    return permutation;
}

/// @brief gray map, differential encode, transmit, rotate by `k`, hard-decide, differential decode, inverse gray map.
[[nodiscard]] std::vector<std::uint8_t> chain(std::span<const std::uint8_t> data, std::uint32_t modulus, std::uint32_t ambiguity, bool grayLabeled) {
    std::vector<std::uint8_t> mapped(data.size());
    for (std::size_t i = 0UZ; i < data.size(); ++i) {
        mapped[i] = static_cast<std::uint8_t>(gray(data[i]));
    }

    DifferentialEncoder<std::uint8_t> encoder = make<DifferentialEncoder<std::uint8_t>>({{"modulus", modulus}});
    const std::vector<std::uint8_t>   coded   = drive<std::uint8_t>(encoder, std::span<const std::uint8_t>(mapped));

    std::vector<std::uint8_t> decided(coded.size());
    for (std::size_t i = 0UZ; i < coded.size(); ++i) {
        const CF received = psk(positionOf(coded[i], grayLabeled), modulus) * psk(ambiguity, modulus);
        decided[i]        = static_cast<std::uint8_t>(labelOf(decideIndex(received, modulus), grayLabeled));
    }

    DifferentialDecoder<std::uint8_t> decoder   = make<DifferentialDecoder<std::uint8_t>>({{"modulus", modulus}});
    const std::vector<std::uint8_t>   recovered = drive<std::uint8_t>(decoder, std::span<const std::uint8_t>(decided));

    std::vector<std::uint8_t> bits(recovered.size());
    for (std::size_t i = 0UZ; i < recovered.size(); ++i) {
        bits[i] = static_cast<std::uint8_t>(ungray(recovered[i]));
    }
    return bits;
}

static_assert(DifferentialEncoder<std::uint8_t>::keepsTagOffsets(), "keeps tag offsets: no Resampling<> or Stride<>, no asynchronous port, no tag policy and no forwardTags() override");
static_assert(DifferentialEncoder<std::uint16_t>::keepsTagOffsets());
static_assert(DifferentialDecoder<std::uint8_t>::keepsTagOffsets());
static_assert(DifferentialDecoder<std::uint16_t>::keepsTagOffsets());
static_assert(DifferentialPhasor<float>::keepsTagOffsets());

} // namespace

const boost::ut::suite<"DifferentialCoding"> differentialCodingTests = [] {
    using namespace boost::ut;

    "the decoder agrees with the mathematical modulus over the whole range of M"_test = [] {
        std::size_t failed = 0UZ;
        for (std::uint32_t modulus = 2U; modulus <= 256U; ++modulus) {
            const std::vector<std::uint8_t> coded = randomSymbols<std::uint8_t>(512UZ, modulus, 0xa4093822299f31d0ULL + modulus);

            DifferentialDecoder<std::uint8_t> decoder = make<DifferentialDecoder<std::uint8_t>>({{"modulus", modulus}});
            const std::vector<std::uint8_t>   got     = drive<std::uint8_t>(decoder, std::span<const std::uint8_t>(coded));

            std::uint32_t previous = 0U;
            for (std::size_t i = 0UZ; i < coded.size(); ++i) {
                const std::uint32_t want = modulo(static_cast<std::int64_t>(coded[i]) - static_cast<std::int64_t>(previous), modulus);
                failed += got[i] != want ? 1UZ : 0UZ;
                previous = coded[i];
            }
        }
        expect(eq(failed, 0UZ)) << "the block's difference must be the mathematical modulus for every M in [2, 256]";
    };

    "the conditional subtract is the modulus, exhaustively"_test = [] {
        std::size_t mismatches = 0UZ;
        for (std::uint32_t modulus = 2U; modulus <= 256U; ++modulus) {
            for (std::uint32_t a = 0U; a < modulus; ++a) {
                for (std::uint32_t b = 0U; b < modulus; ++b) {
                    const std::uint32_t biased = a + modulus - b;
                    mismatches += (biased >= modulus ? biased - modulus : biased) != (a + modulus - b) % modulus ? 1UZ : 0UZ;
                }
            }
        }
        expect(eq(mismatches, 0UZ)) << "one bias and one predicated subtract must replace the divide exactly";
    };

    "the promoted difference is wrong for every modulus that does not divide 2^32"_test = [] {
        expect(eq(promotedDifference(0U, 1U, 3U), 0U)) << "the promoted form decodes y=0 against y=1 at M=3 as 0";
        expect(eq(modulo(-1, 3U), 2U)) << "and the answer is 2";

        for (const std::uint32_t modulus : {3U, 5U, 6U, 10U, 17U, 100U, 255U}) {
            std::size_t differing = 0UZ;
            for (std::uint32_t a = 0U; a < modulus; ++a) {
                for (std::uint32_t b = 0U; b < modulus; ++b) {
                    differing += promotedDifference(a, b, modulus) != modulo(static_cast<std::int64_t>(a) - static_cast<std::int64_t>(b), modulus) ? 1UZ : 0UZ;
                }
            }
            expect(gt(differing, 0UZ)) << std::format("M={} does not divide 2^32, so the promoted form must disagree somewhere", modulus);
        }

        for (const std::uint32_t modulus : {2U, 4U, 8U, 16U, 32U, 64U, 128U, 256U}) {
            std::size_t differing = 0UZ;
            for (std::uint32_t a = 0U; a < 256U; ++a) {
                for (std::uint32_t b = 0U; b < 256U; ++b) {
                    differing += promotedDifference(a, b, modulus) != modulo(static_cast<std::int64_t>(a) - static_cast<std::int64_t>(b), modulus) ? 1UZ : 0UZ;
                }
            }
            expect(eq(differing, 0UZ)) << std::format("M={} divides 2^32, so the promoted form happens to agree everywhere", modulus);
        }
    };

    "the promoted difference fails to invert the encoder end to end"_test = [] {
        for (const std::uint32_t modulus : {3U, 5U, 6U, 10U, 17U, 100U, 255U}) {
            const std::vector<std::uint8_t> data = randomSymbols<std::uint8_t>(4000UZ, modulus, 0x082efa98ec4e6c89ULL + modulus);

            DifferentialEncoder<std::uint8_t> encoder = make<DifferentialEncoder<std::uint8_t>>({{"modulus", modulus}});
            const std::vector<std::uint8_t>   coded   = drive<std::uint8_t>(encoder, std::span<const std::uint8_t>(data));

            std::size_t   wrong    = 0UZ;
            std::uint32_t previous = 0U;
            for (std::size_t i = 0UZ; i < coded.size(); ++i) {
                wrong += promotedDifference(coded[i], previous, modulus) != data[i] ? 1UZ : 0UZ;
                previous = coded[i];
            }
            expect(gt(wrong, 0UZ)) << std::format("M={}: the promoted form pairs a working encoder with a broken decoder", modulus);
        }

        for (const std::uint32_t modulus : {2U, 4U, 8U, 16U, 32U, 64U, 128U, 256U}) {
            const std::vector<std::uint8_t> data = randomSymbols<std::uint8_t>(4000UZ, modulus, 0x452821e638d01377ULL + modulus);

            DifferentialEncoder<std::uint8_t> encoder = make<DifferentialEncoder<std::uint8_t>>({{"modulus", modulus}});
            const std::vector<std::uint8_t>   coded   = drive<std::uint8_t>(encoder, std::span<const std::uint8_t>(data));

            std::size_t   wrong    = 0UZ;
            std::uint32_t previous = 0U;
            for (std::size_t i = 0UZ; i < coded.size(); ++i) {
                wrong += promotedDifference(coded[i], previous, modulus) != data[i] ? 1UZ : 0UZ;
                previous = coded[i];
            }
            expect(eq(wrong, 0UZ)) << std::format("M={}: which is why a suite of powers of two alone cannot see the failure", modulus);
        }
    };

    "the output does not depend on how the stream was chunked"_test = [] {
        for (const std::uint32_t modulus : {2U, 100U}) {
            const std::vector<std::uint8_t> data = randomSymbols<std::uint8_t>(10000UZ, modulus, 0xc0ac29b7c97c50ddULL + modulus);

            DifferentialEncoder<std::uint8_t> whole        = make<DifferentialEncoder<std::uint8_t>>({{"modulus", modulus}});
            DifferentialDecoder<std::uint8_t> wholeDecoder = make<DifferentialDecoder<std::uint8_t>>({{"modulus", modulus}});
            const std::vector<std::uint8_t>   coded        = drive<std::uint8_t>(whole, std::span<const std::uint8_t>(data));
            const std::vector<std::uint8_t>   decoded      = drive<std::uint8_t>(wholeDecoder, std::span<const std::uint8_t>(data));

            for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 4096UZ}) {
                DifferentialEncoder<std::uint8_t> encoder = make<DifferentialEncoder<std::uint8_t>>({{"modulus", modulus}});
                DifferentialDecoder<std::uint8_t> decoder = make<DifferentialDecoder<std::uint8_t>>({{"modulus", modulus}});
                expect(eq(firstDifference<std::uint8_t>(coded, drive<std::uint8_t>(encoder, std::span<const std::uint8_t>(data), chunk)), kEqual)) << std::format("encoder M={} chunk={}", modulus, chunk);
                expect(eq(firstDifference<std::uint8_t>(decoded, drive<std::uint8_t>(decoder, std::span<const std::uint8_t>(data), chunk)), kEqual)) << std::format("decoder M={} chunk={}", modulus, chunk);
            }
        }

        std::vector<CF> signal(10000UZ);
        Rng             rng{0x3f84d5b5b5470917ULL};
        for (CF& sample : signal) {
            const double angle     = 2.0 * kPi * static_cast<double>(rng.below(1000U)) / 1000.0;
            const double magnitude = 0.5 + static_cast<double>(rng.below(1000U)) / 1000.0;
            sample                 = CF(static_cast<float>(magnitude * std::cos(angle)), static_cast<float>(magnitude * std::sin(angle)));
        }

        DifferentialPhasor<float> whole     = make<DifferentialPhasor<float>>();
        const std::vector<CF>     reference = drive<CF>(whole, std::span<const CF>(signal));
        for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 4096UZ}) {
            DifferentialPhasor<float> phasor = make<DifferentialPhasor<float>>();
            expect(eq(firstDifference<CF>(reference, drive<CF>(phasor, std::span<const CF>(signal), chunk)), kEqual)) << std::format("phasor chunk={}", chunk);
        }
    };

    "reset returns both halves to initial_state"_test = [] {
        constexpr std::uint32_t         kModulus = 100U;
        const std::vector<std::uint8_t> data     = randomSymbols<std::uint8_t>(10000UZ, kModulus, 0x9216d5d98979fb1bULL);

        DifferentialEncoder<std::uint8_t> encoder = make<DifferentialEncoder<std::uint8_t>>({{"modulus", kModulus}});
        DifferentialDecoder<std::uint8_t> decoder = make<DifferentialDecoder<std::uint8_t>>({{"modulus", kModulus}});

        const std::vector<std::uint8_t> firstCoded   = drive<std::uint8_t>(encoder, std::span<const std::uint8_t>(data));
        const std::vector<std::uint8_t> firstDecoded = drive<std::uint8_t>(decoder, std::span<const std::uint8_t>(data));
        const std::vector<std::uint8_t> carried      = drive<std::uint8_t>(encoder, std::span<const std::uint8_t>(data));
        const std::vector<std::uint8_t> carriedBack  = drive<std::uint8_t>(decoder, std::span<const std::uint8_t>(data));

        expect(neq(firstDifference<std::uint8_t>(firstCoded, carried), kEqual)) << "without a reset the encoder continues from where it stopped";
        expect(neq(firstDifference<std::uint8_t>(firstDecoded, carriedBack), kEqual)) << "and so does the decoder, whose state is as much part of the contract as the encoder's";

        encoder.reset();
        decoder.reset();
        expect(eq(firstDifference<std::uint8_t>(firstCoded, drive<std::uint8_t>(encoder, std::span<const std::uint8_t>(data))), kEqual)) << "reset makes a run reproducible";
        expect(eq(firstDifference<std::uint8_t>(firstDecoded, drive<std::uint8_t>(decoder, std::span<const std::uint8_t>(data))), kEqual)) << "and resets both halves the same way";
    };

    "the rotation permutations are the measured tables"_test = [] {
        const std::vector<std::uint32_t> ordered4 = rotationPermutation(4U, false);
        const std::vector<std::uint32_t> gray4    = rotationPermutation(4U, true);
        const std::vector<std::uint32_t> gray8    = rotationPermutation(8U, true);

        expect(that % (ordered4 == std::vector<std::uint32_t>{1U, 2U, 3U, 0U})) << "a rotation-ordered labeling maps s to s+1 mod M";
        expect(that % (gray4 == std::vector<std::uint32_t>{1U, 3U, 0U, 2U})) << "a Gray labeling does not, which is why differencing cannot cancel it";
        expect(that % (gray8 == std::vector<std::uint32_t>{1U, 3U, 6U, 2U, 0U, 4U, 7U, 5U}));

        for (const std::uint32_t modulus : {2U, 4U, 8U, 16U}) {
            std::vector<std::uint32_t> want(modulus);
            for (std::uint32_t s = 0U; s < modulus; ++s) {
                want[s] = (s + 1U) % modulus;
            }
            expect(that % (rotationPermutation(modulus, false) == want)) << std::format("M={}", modulus);
        }
        expect(that % (rotationPermutation(2U, true) == std::vector<std::uint32_t>{1U, 0U})) << "at M=2 Gray and rotation-ordered coincide";
    };

    "a rotation-ordered chain is immune to the M-fold phase ambiguity"_test = [] {
        for (const std::uint32_t modulus : {2U, 4U, 8U}) {
            const std::vector<std::uint8_t> data = randomSymbols<std::uint8_t>(5000UZ, modulus, 0xd1310ba698dfb5acULL + modulus);
            for (std::uint32_t ambiguity = 0U; ambiguity < modulus; ++ambiguity) {
                const std::vector<std::uint8_t> recovered = chain(std::span<const std::uint8_t>(data), modulus, ambiguity, false);
                expect(eq(firstDifference<std::uint8_t>(std::span<const std::uint8_t>(data).subspan(1UZ), std::span<const std::uint8_t>(recovered).subspan(1UZ)), kEqual)) << std::format("M={} k={}: exact from sample 1", modulus, ambiguity);
            }
        }
    };

    "a Gray-labeled constellation loses the immunity, which is what the pre-differential map buys"_test = [] {
        for (const std::uint32_t modulus : {4U, 8U}) {
            const std::vector<std::uint8_t> data = randomSymbols<std::uint8_t>(5000UZ, modulus, 0x2ffd72dbd01adfb7ULL + modulus);

            const std::vector<std::uint8_t> aligned = chain(std::span<const std::uint8_t>(data), modulus, 0U, true);
            expect(eq(firstDifference<std::uint8_t>(std::span<const std::uint8_t>(data).subspan(1UZ), std::span<const std::uint8_t>(aligned).subspan(1UZ)), kEqual)) << std::format("M={}: with no ambiguity any labeling works", modulus);

            for (std::uint32_t ambiguity = 1U; ambiguity < modulus; ++ambiguity) {
                const std::vector<std::uint8_t> recovered = chain(std::span<const std::uint8_t>(data), modulus, ambiguity, true);
                expect(neq(firstDifference<std::uint8_t>(std::span<const std::uint8_t>(data).subspan(1UZ), std::span<const std::uint8_t>(recovered).subspan(1UZ)), kEqual)) << std::format("M={} k={}: the Gray permutation does not cancel", modulus, ambiguity);
            }
        }
    };

    "the phasor cancels a carrier offset of any size"_test = [] {
        std::vector<CF> signal(2048UZ);
        Rng             rng{0x4b7a70e9b5b32944ULL};
        for (CF& sample : signal) {
            const double angle = 2.0 * kPi * static_cast<double>(rng.below(997U)) / 997.0;
            sample             = CF(static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle)));
        }

        DifferentialPhasor<float> plain     = make<DifferentialPhasor<float>>();
        const std::vector<CF>     reference = drive<CF>(plain, std::span<const CF>(signal));

        for (std::size_t which = 0UZ; which < 10UZ; ++which) {
            const double omega = kPi * (static_cast<double>(rng.below(2000U)) / 1000.0 - 1.0);

            std::vector<CF> translated(signal.size());
            for (std::size_t i = 0UZ; i < signal.size(); ++i) {
                const double phase = omega * static_cast<double>(i);
                translated[i]      = signal[i] * CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
            }

            DifferentialPhasor<float> phasor = make<DifferentialPhasor<float>>();
            const std::vector<CF>     output = drive<CF>(phasor, std::span<const CF>(translated));
            const CF                  rotate = CF(static_cast<float>(std::cos(omega)), static_cast<float>(std::sin(omega)));

            double worst = 0.0;
            for (std::size_t i = 1UZ; i < output.size(); ++i) {
                worst = std::max(worst, static_cast<double>(std::abs(output[i] - reference[i] * rotate)));
            }
            expect(lt(worst, 1e-6)) << std::format("omega={}: the offset survives as one constant rotation", omega);
        }
    };
};

int main() { /* not needed for UT */ }
