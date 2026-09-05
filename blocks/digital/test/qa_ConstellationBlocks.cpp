#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <numbers>
#include <numeric>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/digital/Constellation.hpp>
#include <gnuradio-4.0/digital/ConstellationBlocks.hpp>
#include <gnuradio-4.0/digital/DifferentialCoding.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::digital::ConstellationDecoder;
using gr::blocks::digital::ConstellationEncoder;
using gr::blocks::digital::ConstellationSoftDecoder;
using gr::blocks::digital::DifferentialDecoder;
using gr::blocks::digital::DifferentialEncoder;
using gr::blocks::digital::SymbolMap;
using gr::digital::Constellation;
using gr::testing::ProcessFunction;
using gr::testing::TagSink;
using gr::testing::TagSource;

using CF = std::complex<float>;

constexpr double kPi = std::numbers::pi;

struct Named {
    const char* name;
    gr::Size_t  arity;
    std::size_t bits;
};

/// The five factories the blocks name, with the arity each needs and the bit count each carries.
constexpr Named kNamed[] = {{"bpsk", 2U, 1UZ}, {"qpsk", 4U, 2UZ}, {"psk8", 8U, 3UZ}, {"qam", 16U, 4UZ}, {"qam", 64U, 6UZ}};

[[nodiscard]] Constellation<float> kernelOf(const Named& row) {
    if (std::string_view(row.name) == "qam") {
        return Constellation<float>::qam(row.arity);
    }
    if (std::string_view(row.name) == "bpsk") {
        return Constellation<float>::bpsk();
    }
    if (std::string_view(row.name) == "qpsk") {
        return Constellation<float>::qpsk();
    }
    return Constellation<float>::psk8();
}

[[nodiscard]] gr::property_map settingsOf(const Named& row) { return {{"constellation", std::string(row.name)}, {"arity", row.arity}}; }

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

template<typename TBlock>
void restage(TBlock& block, gr::property_map settings) {
    std::ignore = block.settings().set(std::move(settings));
    std::ignore = block.settings().activateContext();
    std::ignore = block.settings().applyStagedParameters();
}

/// Drive a one-in one-out block over plain spans, `outPerIn` outputs reserved per input item.
template<typename TOut, typename TIn, typename TBlock>
[[nodiscard]] std::vector<TOut> drive(TBlock& block, std::span<const TIn> input, std::size_t chunkSize = 0UZ, std::size_t outPerIn = 1UZ) {
    std::vector<TOut> output(input.size() * outPerIn);
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    for (std::size_t base = 0UZ; base < input.size(); base += stride) {
        const std::size_t count = std::min(stride, input.size() - base);
        std::ignore             = block.processBulk(input.subspan(base, count), std::span<TOut>(output.data() + base * outPerIn, count * outPerIn));
    }
    return output;
}

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }

    [[nodiscard]] double uniform() noexcept { return static_cast<double>(next() >> 11U) / static_cast<double>(1ULL << 53U); }

    /// Box-Muller, so the samples reach well outside the constellation where a clamp or a sector goes wrong.
    [[nodiscard]] CF gaussian(double sigma) noexcept {
        const double u = std::max(uniform(), 1e-300);
        const double v = uniform();
        const double r = sigma * std::sqrt(-2.0 * std::log(u));
        return CF(static_cast<float>(r * std::cos(2.0 * kPi * v)), static_cast<float>(r * std::sin(2.0 * kPi * v)));
    }
};

[[nodiscard]] std::vector<CF> noiseCloud(std::size_t count, double sigma = 1.3) {
    Rng             rng{};
    std::vector<CF> samples(count);
    for (CF& sample : samples) {
        sample = rng.gaussian(sigma);
    }
    return samples;
}

/// The rotation-ordered point list of a Gray constellation: its own points, reordered, so no arithmetic differs.
[[nodiscard]] std::vector<float> rotationOrderedPoints(const Constellation<float>& gray) {
    const auto         table = gray.rotationToGray().value();
    std::vector<float> interleaved(2UZ * gray.size());
    for (std::size_t r = 0UZ; r < gray.size(); ++r) {
        const CF point             = gray.point(table[r]);
        interleaved[2UZ * r]       = point.real();
        interleaved[2UZ * r + 1UZ] = point.imag();
    }
    return interleaved;
}

[[nodiscard]] std::vector<std::uint8_t> tableOf(const std::array<std::uint8_t, 256>& table) { return {table.begin(), table.end()}; }

/**
 * @brief Whether nudging @p z by a hair on either axis changes the nearest point.
 *
 * The closed forms and `Nearest` are required to agree everywhere except on the boundaries, where the kernel's own
 * documentation says the two round in different domains and can differ by one level, each deterministically. A sample
 * whose nearest point survives a nudge of `1e-6` of the minimum distance is not on a boundary, and there the
 * agreement is unconditional.
 */
[[nodiscard]] bool sitsOnADecisionBoundary(const Constellation<float>& kernel, CF z) {
    const float        nudge = 1e-6f * kernel.minimumDistance();
    const std::uint8_t here  = kernel.nearestDecision(z);
    for (const CF& step : {CF(nudge, 0.0f), CF(-nudge, 0.0f), CF(0.0f, nudge), CF(0.0f, -nudge)}) {
        if (kernel.nearestDecision(z + step) != here) {
            return true;
        }
    }
    return false;
}

} // namespace

const boost::ut::suite<"constellation blocks"> constellationBlockTests = [] {
    using namespace boost::ut;

    "an encoder cannot be made to read past its point list"_test = [] {
        for (const Named& row : kNamed) {
            ConstellationEncoder<float> block  = make<ConstellationEncoder<float>>(settingsOf(row));
            const Constellation<float>  kernel = kernelOf(row);

            std::vector<std::uint8_t> everyByte(256UZ);
            std::iota(everyByte.begin(), everyByte.end(), std::uint8_t{0});
            const std::vector<CF> points = drive<CF>(block, std::span<const std::uint8_t>(everyByte));

            expect(eq(points.size(), 256UZ));
            for (std::size_t i = 0UZ; i < 256UZ; ++i) {
                expect(that % (points[i] == kernel.points()[i % row.arity])) << std::format("{}/{}: byte {} is the point for {} mod M", row.name, row.arity, i, i);
            }
        }
    };

    "symbol_map's table is 256 entries because the index is a byte"_test = [] {
        std::vector<std::uint8_t> everyByte(256UZ);
        std::iota(everyByte.begin(), everyByte.end(), std::uint8_t{0});

        SymbolMap identity = make<SymbolMap>();
        expect(that % (drive<std::uint8_t>(identity, std::span<const std::uint8_t>(everyByte)) == everyByte)) << "an unset table is the identity";

        SymbolMap                       shortTable = make<SymbolMap>({{"map", std::vector<std::uint8_t>{3U, 2U, 1U, 0U}}});
        const std::vector<std::uint8_t> shortOut   = drive<std::uint8_t>(shortTable, std::span<const std::uint8_t>(everyByte));
        expect(that % (std::vector<std::uint8_t>(shortOut.begin(), shortOut.begin() + 4) == std::vector<std::uint8_t>{3U, 2U, 1U, 0U}));
        for (std::size_t i = 4UZ; i < 256UZ; ++i) {
            expect(eq(static_cast<std::size_t>(shortOut[i]), i)) << "a short table is identity-filled, not truncated at four";
        }

        std::vector<std::uint8_t> reversed(256UZ);
        for (std::size_t i = 0UZ; i < 256UZ; ++i) {
            reversed[i] = static_cast<std::uint8_t>(255UZ - i);
        }
        SymbolMap full = make<SymbolMap>({{"map", reversed}});
        expect(that % (drive<std::uint8_t>(full, std::span<const std::uint8_t>(everyByte)) == reversed));

        std::vector<std::uint8_t> tooLong(257UZ, 0U);
        expect(throws([&tooLong] { std::ignore = make<SymbolMap>({{"map", tooLong}}); })) << "a table longer than the index range throws rather than being silently truncated";
    };

    "encoder then decoder is the identity for every symbol"_test = [] {
        for (const Named& row : kNamed) {
            ConstellationEncoder<float> encoder = make<ConstellationEncoder<float>>(settingsOf(row));
            ConstellationDecoder<float> decoder = make<ConstellationDecoder<float>>(settingsOf(row));

            std::vector<std::uint8_t> symbols(row.arity);
            std::iota(symbols.begin(), symbols.end(), std::uint8_t{0});
            const std::vector<CF>           points  = drive<CF>(encoder, std::span<const std::uint8_t>(symbols));
            const std::vector<std::uint8_t> decoded = drive<std::uint8_t>(decoder, std::span<const CF>(points));
            expect(that % (decoded == symbols)) << std::format("{}/{}: hardDecision(point(s)) == s", row.name, row.arity);
        }
    };

    "the decoder's closed form is the nearest point"_test = [] {
        const std::vector<CF> cloud = noiseCloud(200000UZ);
        for (const Named& row : kNamed) {
            ConstellationDecoder<float>     block   = make<ConstellationDecoder<float>>(settingsOf(row));
            const Constellation<float>      kernel  = kernelOf(row);
            const std::vector<std::uint8_t> decided = drive<std::uint8_t>(block, std::span<const CF>(cloud));

            std::size_t mismatches = 0UZ;
            std::size_t onBoundary = 0UZ;
            std::string first;
            for (std::size_t i = 0UZ; i < cloud.size(); ++i) {
                const std::uint8_t want = kernel.nearestDecision(cloud[i]);
                if (decided[i] == want) {
                    continue;
                }
                if (sitsOnADecisionBoundary(kernel, cloud[i])) {
                    ++onBoundary; // the documented measure-zero set: the two domains round it differently
                    continue;
                }
                ++mismatches;
                if (first.empty()) {
                    first = std::format(" first at {}: ({:a}, {:a}) gave {} against {}", i, cloud[i].real(), cloud[i].imag(), decided[i], want);
                }
            }
            expect(eq(mismatches, 0UZ)) << std::format("{}/{}: closed form against exhaustive search;{}", row.name, row.arity, first);
            expect(le(onBoundary, 2UZ)) << std::format("{}/{}: {} boundary samples out of {} — the disagreement is measure zero, not a region", row.name, row.arity, onBoundary, cloud.size());
        }
    };

    "noise_power is linear and strictly positive"_test = [] {
        expect(throws([] { std::ignore = make<ConstellationSoftDecoder<float>>({{"noise_power", 0.0f}}); }));
        expect(throws([] { std::ignore = make<ConstellationSoftDecoder<float>>({{"noise_power", -1.0f}}); }));
        expect(throws([] { std::ignore = make<ConstellationSoftDecoder<float>>({{"soft_algorithm", std::string("approx")}}); }));

        const std::vector<CF> cloud = noiseCloud(4096UZ);
        for (const Named& row : kNamed) {
            gr::property_map settings             = settingsOf(row);
            settings["noise_power"]               = 1.0f;
            ConstellationSoftDecoder<float> block = make<ConstellationSoftDecoder<float>>(settings);

            const std::vector<float> atOne = drive<float>(block, std::span<const CF>(cloud), 0UZ, row.bits);
            restage(block, {{"noise_power", 0.25f}});
            const std::vector<float> atQuarter = drive<float>(block, std::span<const CF>(cloud), 0UZ, row.bits);

            expect(eq(atOne.size(), cloud.size() * row.bits));
            std::size_t signChanges = 0UZ;
            std::size_t ratioErrors = 0UZ;
            for (std::size_t i = 0UZ; i < atOne.size(); ++i) {
                signChanges += std::signbit(atOne[i]) != std::signbit(atQuarter[i]) ? 1UZ : 0UZ;
                ratioErrors += atQuarter[i] != 4.0f * atOne[i] ? 1UZ : 0UZ;
            }
            expect(eq(signChanges, 0UZ)) << std::format("{}/{}: changing N0 changes no sign", row.name, row.arity);
            expect(eq(ratioErrors, 0UZ)) << std::format("{}/{}: the soft values scale by exactly the inverse ratio", row.name, row.arity);
        }
    };

    "under max_log the sign of every soft value is the hard decision's bit"_test = [] {
        const std::vector<CF> cloud = noiseCloud(50000UZ);
        for (const Named& row : kNamed) {
            ConstellationDecoder<float>     hard    = make<ConstellationDecoder<float>>(settingsOf(row));
            ConstellationSoftDecoder<float> soft    = make<ConstellationSoftDecoder<float>>(settingsOf(row));
            const std::vector<std::uint8_t> decided = drive<std::uint8_t>(hard, std::span<const CF>(cloud));
            const std::vector<float>        llr     = drive<float>(soft, std::span<const CF>(cloud), 0UZ, row.bits);

            std::size_t disagreements = 0UZ;
            for (std::size_t i = 0UZ; i < cloud.size(); ++i) {
                for (std::size_t k = 0UZ; k < row.bits; ++k) {
                    const bool fromSoft = llr[i * row.bits + k] > 0.0f;
                    const bool fromHard = ((decided[i] >> (row.bits - 1UZ - k)) & 1U) != 0U;
                    disagreements += fromSoft != fromHard ? 1UZ : 0UZ;
                }
            }
            expect(eq(disagreements, 0UZ)) << std::format("{}/{}: asserted exactly, not statistically", row.name, row.arity);
        }
    };

    "the soft decoder's rate is the constellation's bit count"_test = [] {
        for (const Named& row : kNamed) {
            ConstellationSoftDecoder<float> block = make<ConstellationSoftDecoder<float>>(settingsOf(row));
            expect(eq(static_cast<std::size_t>(block.input_chunk_size.value), 1UZ)) << row.name;
            expect(eq(static_cast<std::size_t>(block.output_chunk_size.value), row.bits)) << std::format("{}/{}", row.name, row.arity);
            expect(eq(block.bitsPerSymbol(), row.bits));
        }
    };

    "a tag at input t lands at output m*t, at a stream offset above 2^53"_test = [] {
        if constexpr (sizeof(std::size_t) != 8UZ) {
            return; // the offsets below are past 2^53, which a 32-bit std::size_t does not hold at all
        }
        constexpr std::size_t kBase     = static_cast<std::size_t>((1ULL << 53U) + 8ULL);
        constexpr std::size_t kOffset[] = {0UZ, 5UZ, 11UZ, 12UZ};

        for (const Named& row : kNamed) {
            std::vector<gr::Tag> incoming;
            for (const std::size_t at : kOffset) {
                incoming.emplace_back(kBase + at, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("symbol")}});
            }

            ConstellationSoftDecoder<float> block = make<ConstellationSoftDecoder<float>>(settingsOf(row));
            const std::vector<CF>           cloud = noiseCloud(16UZ);
            const auto                      seen  = gr::blocks::testing::span::run<float>(block, std::span<const CF>(cloud), 4UZ, row.bits, std::span<const gr::Tag>(incoming), kBase);

            expect(eq(seen.samples.size(), cloud.size() * row.bits)) << std::format("{}/{}", row.name, row.arity);
            expect(eq(seen.tags.size(), std::size(kOffset)));
            for (std::size_t i = 0UZ; i < seen.tags.size(); ++i) {
                expect(eq(seen.tags[i].index, row.bits * (kBase + kOffset[i]))) << std::format("{}/{}: tag {} at m*(base+{})", row.name, row.arity, i, kOffset[i]);
            }
        }
    };

    "every block is chunk independent"_test = [] {
        constexpr std::size_t kChunks[] = {1UZ, 3UZ, 17UZ, 4096UZ};
        const std::vector<CF> cloud     = noiseCloud(5000UZ);

        std::vector<std::uint8_t> symbols(5000UZ);
        Rng                       rng{};
        for (std::uint8_t& symbol : symbols) {
            symbol = static_cast<std::uint8_t>(rng.next() & 0xFFU);
        }

        for (const Named& row : kNamed) {
            ConstellationDecoder<float>     hard    = make<ConstellationDecoder<float>>(settingsOf(row));
            ConstellationSoftDecoder<float> soft    = make<ConstellationSoftDecoder<float>>(settingsOf(row));
            ConstellationEncoder<float>     encoder = make<ConstellationEncoder<float>>(settingsOf(row));
            SymbolMap                       mapper  = make<SymbolMap>({{"map", tableOf(kernelOf(row).rotationToGray().value_or(std::array<std::uint8_t, 256>{}))}});

            const std::vector<std::uint8_t> hardWhole    = drive<std::uint8_t>(hard, std::span<const CF>(cloud));
            const std::vector<float>        softWhole    = drive<float>(soft, std::span<const CF>(cloud), 0UZ, row.bits);
            const std::vector<CF>           encoderWhole = drive<CF>(encoder, std::span<const std::uint8_t>(symbols));
            const std::vector<std::uint8_t> mapperWhole  = drive<std::uint8_t>(mapper, std::span<const std::uint8_t>(symbols));

            for (const std::size_t chunk : kChunks) {
                expect(that % (drive<std::uint8_t>(hard, std::span<const CF>(cloud), chunk) == hardWhole)) << std::format("{}/{} decoder at chunk {}", row.name, row.arity, chunk);
                expect(that % (drive<float>(soft, std::span<const CF>(cloud), chunk, row.bits) == softWhole)) << std::format("{}/{} soft decoder at chunk {}", row.name, row.arity, chunk);
                expect(that % (drive<CF>(encoder, std::span<const std::uint8_t>(symbols), chunk) == encoderWhole)) << std::format("{}/{} encoder at chunk {}", row.name, row.arity, chunk);
                expect(that % (drive<std::uint8_t>(mapper, std::span<const std::uint8_t>(symbols), chunk) == mapperWhole)) << std::format("{}/{} symbol_map at chunk {}", row.name, row.arity, chunk);
            }
        }
    };

    "changing the constellation changes the soft decoder's rate mid-stream"_test = [] {
        const std::vector<CF>           cloud = noiseCloud(64UZ);
        ConstellationSoftDecoder<float> block = make<ConstellationSoftDecoder<float>>({{"constellation", std::string("qpsk")}});
        expect(eq(static_cast<std::size_t>(block.output_chunk_size.value), 2UZ));
        const std::vector<float> asQpsk = drive<float>(block, std::span<const CF>(cloud), 0UZ, 2UZ);
        expect(eq(asQpsk.size(), 128UZ));

        restage(block, {{"constellation", std::string("psk8")}, {"arity", 8U}});
        expect(eq(static_cast<std::size_t>(block.output_chunk_size.value), 3UZ)) << "the ratio follows the constellation";
        const std::vector<float> as8psk = drive<float>(block, std::span<const CF>(cloud), 7UZ, 3UZ);
        expect(eq(as8psk.size(), 192UZ)) << "whole symbols only: no partial symbol's worth of soft values";

        restage(block, {{"constellation", std::string("qam")}, {"arity", 64U}});
        expect(eq(static_cast<std::size_t>(block.output_chunk_size.value), 6UZ));
        expect(eq(drive<float>(block, std::span<const CF>(cloud), 5UZ, 6UZ).size(), 384UZ));
    };

    "degenerate constellation settings throw"_test = [] {
        expect(throws([] { std::ignore = make<ConstellationDecoder<float>>({{"constellation", std::string("dqpsk")}}); }));
        expect(throws([] { std::ignore = make<ConstellationDecoder<float>>({{"normalization", std::string("rms")}}); }));
        expect(throws([] { std::ignore = make<ConstellationDecoder<float>>({{"constellation", std::string("qam")}, {"arity", 32U}}); })) << "32QAM has no axis product and is not a factory";
        expect(throws([] { std::ignore = make<ConstellationDecoder<float>>({{"constellation", std::string("psk")}, {"arity", 6U}}); }));
        expect(throws([] { std::ignore = make<ConstellationDecoder<float>>({{"constellation", std::string("custom")}, {"points", std::vector<float>{1.0f}}}); }));
    };

    "power normalization is the default and amplitude is available"_test = [] {
        ConstellationEncoder<float> power     = make<ConstellationEncoder<float>>({{"constellation", std::string("qam")}, {"arity", 16U}});
        ConstellationEncoder<float> amplitude = make<ConstellationEncoder<float>>({{"constellation", std::string("qam")}, {"arity", 16U}, {"normalization", std::string("amplitude")}});

        std::vector<std::uint8_t> symbols(16UZ);
        std::iota(symbols.begin(), symbols.end(), std::uint8_t{0});
        const std::vector<CF> asPower     = drive<CF>(power, std::span<const std::uint8_t>(symbols));
        const std::vector<CF> asAmplitude = drive<CF>(amplitude, std::span<const std::uint8_t>(symbols));

        double meanPower = 0.0;
        for (const CF& point : asPower) {
            meanPower += static_cast<double>(std::norm(point));
        }
        expect(approx(meanPower / 16.0, 1.0, 1e-6)) << "power: mean symbol energy is one";
        expect(approx(static_cast<double>(std::abs(asAmplitude[0])) / static_cast<double>(std::abs(asPower[0])), 1.055728090, 1e-6)) << "16QAM carries 0.471 dB more under amplitude normalization";
    };

    "a differential chain recovers the data through any k*2*pi/M rotation"_test = [] {
        constexpr gr::Size_t kArities[] = {2U, 4U, 8U};
        Rng                  rng{};

        for (const gr::Size_t arity : kArities) {
            const Constellation<float> gray       = Constellation<float>::psk(arity, arity == 2U ? 0.0f : static_cast<float>(kPi / static_cast<double>(arity)), 0U);
            const std::vector<float>   rotation   = rotationOrderedPoints(gray);
            const auto                 toRotation = gray.grayToRotation().value();
            const auto                 toGray     = gray.rotationToGray().value();

            std::vector<std::uint8_t> data(2000UZ);
            for (std::uint8_t& symbol : data) {
                symbol = static_cast<std::uint8_t>(rng.next() % arity);
            }

            SymbolMap                         transmitMap = make<SymbolMap>({{"map", tableOf(toRotation)}});
            DifferentialEncoder<std::uint8_t> diffEncoder = make<DifferentialEncoder<std::uint8_t>>({{"modulus", arity}});
            ConstellationEncoder<float>       encoder     = make<ConstellationEncoder<float>>({{"constellation", std::string("custom")}, {"points", rotation}, {"normalization", std::string("none")}});

            const std::vector<std::uint8_t> mapped    = drive<std::uint8_t>(transmitMap, std::span<const std::uint8_t>(data));
            const std::vector<std::uint8_t> encoded   = drive<std::uint8_t>(diffEncoder, std::span<const std::uint8_t>(mapped));
            const std::vector<CF>           modulated = drive<CF>(encoder, std::span<const std::uint8_t>(encoded));

            for (gr::Size_t k = 0U; k < arity; ++k) {
                const double    turn = 2.0 * kPi * static_cast<double>(k) / static_cast<double>(arity);
                const CF        spin(static_cast<float>(std::cos(turn)), static_cast<float>(std::sin(turn)));
                std::vector<CF> received(modulated.size());
                std::ranges::transform(modulated, received.begin(), [spin](const CF& sample) { return sample * spin; });

                ConstellationDecoder<float>       decoder     = make<ConstellationDecoder<float>>({{"constellation", std::string("custom")}, {"points", rotation}, {"normalization", std::string("none")}});
                DifferentialDecoder<std::uint8_t> diffDecoder = make<DifferentialDecoder<std::uint8_t>>({{"modulus", arity}});
                SymbolMap                         receiveMap  = make<SymbolMap>({{"map", tableOf(toGray)}});

                const std::vector<std::uint8_t> sliced    = drive<std::uint8_t>(decoder, std::span<const CF>(received));
                const std::vector<std::uint8_t> differed  = drive<std::uint8_t>(diffDecoder, std::span<const std::uint8_t>(sliced));
                const std::vector<std::uint8_t> recovered = drive<std::uint8_t>(receiveMap, std::span<const std::uint8_t>(differed));

                std::size_t errors = 0UZ;
                for (std::size_t i = 1UZ; i < data.size(); ++i) { // symbol 0 carries the unknown initial rotation
                    errors += recovered[i] != data[i] ? 1UZ : 0UZ;
                }
                expect(eq(errors, 0UZ)) << std::format("M={}, rotated by {}*2*pi/M", arity, k);
            }
        }
    };

    "a 1:1 block passes a reserved tag at the same offset"_test = [] {
        gr::Graph graph;
        auto&     source = graph.emplaceBlock<TagSource<std::uint8_t, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 512U}, {"mark_tag", false}});
        source._tags.emplace_back(7UZ, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("packet")}});
        source._tags.emplace_back(64UZ, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("packet")}});

        auto& mapper = graph.emplaceBlock<SymbolMap>({{"map", std::vector<std::uint8_t>{1U, 0U}}});
        auto& sink   = graph.emplaceBlock<TagSink<std::uint8_t, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

        expect(graph.connect<"out", "in">(source, mapper).has_value());
        expect(graph.connect<"out", "in">(mapper, sink).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        std::vector<std::size_t> offsets;
        for (const gr::Tag& seenTag : sink._tags) {
            if (seenTag.map.contains(gr::property_map::key_type(gr::tag::TRIGGER_NAME.shortKey()))) {
                offsets.push_back(seenTag.index);
            }
        }
        expect(that % (offsets == std::vector<std::size_t>{7UZ, 64UZ})) << "a 1:1 block leaves a reserved key where it arrived";
    };

    "a non-reserved key survives the encoder, the hard decoder and the mapper"_test = [] {
        const gr::pmt::Value             carried{std::string("carried")};
        const gr::property_map::key_type priv{"private_key"};

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<TagSource<std::uint8_t, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 512U}, {"mark_tag", false}});
        source._tags.emplace_back(11UZ, gr::property_map{{priv, carried}});
        source._tags.emplace_back(300UZ, gr::property_map{{priv, carried}});

        auto& mapper  = graph.emplaceBlock<SymbolMap>({{"map", std::vector<std::uint8_t>{1U, 0U}}});
        auto& encoder = graph.emplaceBlock<ConstellationEncoder<float>>({{"constellation", std::string("qpsk")}, {"arity", 4U}});
        auto& decoder = graph.emplaceBlock<ConstellationDecoder<float>>({{"constellation", std::string("qpsk")}, {"arity", 4U}});
        auto& sink    = graph.emplaceBlock<TagSink<std::uint8_t, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

        expect(graph.connect<"out", "in">(source, mapper).has_value());
        expect(graph.connect<"out", "in">(mapper, encoder).has_value());
        expect(graph.connect<"out", "in">(encoder, decoder).has_value());
        expect(graph.connect<"out", "in">(decoder, sink).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        std::vector<std::size_t> offsets;
        for (const gr::Tag& seenTag : sink._tags) {
            if (const auto found = seenTag.map.find(priv); found != seenTag.map.end() && found->second == carried) {
                offsets.push_back(seenTag.index);
            }
        }
        expect(that % (offsets == std::vector<std::size_t>{11UZ, 300UZ})) << "three pass-all 1:1 blocks in series carry a private key unmoved";
    };
};

int main() { /* not needed for UT */ }
