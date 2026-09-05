#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/digital/BitPacking.hpp>

// A regrouping costs what the bits cost, not what the items cost: one period of `lcm(bitsIn, bitsOut)` bits is the
// same work however the widths on either side are arranged, so `8:3` and `3:8` are one number reported two ways.
// Every arm therefore carries both, and the item count per call is a whole number of periods for every width pair
// here, so the arms are directly comparable.
//
// The two eight-bit shapes have a form that moves eight bits with a multiply, a shift and a mask instead of eight
// shift-mask-shift-OR sequences. Each of them is measured beside the general loop it replaces, in both bit orders, so
// the fast path can be dropped if it is not one.

namespace {

using gr::blocks::digital::PackBits;
using gr::blocks::digital::RepackBits;
using gr::blocks::digital::UnpackBits;
using gr::blocks::testing::bench::Arm;
using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;

// a whole number of periods for every input chunk used below — 8, 3, 7 and 1 — so no arm converts a partial period
constexpr std::size_t kItemsPerCall = 65520UZ;
constexpr std::size_t kRepeats      = 9UZ;
constexpr std::size_t kMaxOutItems  = kItemsPerCall * 8UZ;

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
    return block;
}

[[nodiscard]] std::vector<std::uint8_t> randomItems() {
    std::vector<std::uint8_t> data(kItemsPerCall);
    std::uint64_t             state = 0x9e3779b97f4a7c15ULL;
    for (std::uint8_t& item : data) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        item = static_cast<std::uint8_t>(state);
    }
    return data;
}

[[nodiscard]] gr::digital::BitRepack kernelFor(unsigned bitsIn, unsigned bitsOut, gr::digital::BitOrder orderIn, gr::digital::BitOrder orderOut) {
    gr::digital::BitRepack config;
    gr::digital::configureRepack(config, bitsIn, bitsOut, orderIn, orderOut);
    return config;
}

} // namespace

int main() {
    using gr::digital::BitOrder;

    const std::vector<std::uint8_t> input = randomItems();
    std::vector<std::uint8_t>       output(kMaxOutItems);
    std::vector<std::uint8_t>       copied(kItemsPerCall);

    // everything is built before anything is timed, so an arm times the sample path and never a configuration
    PackBits   packMsb   = make<PackBits>({{"k", static_cast<gr::Size_t>(8)}, {"bit_order", std::string("MsbFirst")}});
    PackBits   packLsb   = make<PackBits>({{"k", static_cast<gr::Size_t>(8)}, {"bit_order", std::string("LsbFirst")}});
    UnpackBits unpackMsb = make<UnpackBits>({{"k", static_cast<gr::Size_t>(8)}, {"bit_order", std::string("MsbFirst")}});
    UnpackBits unpackLsb = make<UnpackBits>({{"k", static_cast<gr::Size_t>(8)}, {"bit_order", std::string("LsbFirst")}});

    struct Shape {
        unsigned    bitsIn;
        unsigned    bitsOut;
        BitOrder    orderOut;
        std::string label;
    };
    const std::array<Shape, 5UZ> shapes{Shape{8U, 3U, BitOrder::MsbFirst, "RepackBits 8:3"}, Shape{3U, 8U, BitOrder::MsbFirst, "RepackBits 3:8"}, Shape{5U, 7U, BitOrder::MsbFirst, "RepackBits 5:7, a 35-bit period"}, Shape{8U, 8U, BitOrder::MsbFirst, "RepackBits 8:8, equal orders"}, Shape{8U, 8U, BitOrder::LsbFirst, "RepackBits 8:8, unequal orders"}};

    std::vector<RepackBits> repackers;
    for (const Shape& shape : shapes) {
        repackers.push_back(make<RepackBits>({{"bits_in", static_cast<gr::Size_t>(shape.bitsIn)}, {"bits_out", static_cast<gr::Size_t>(shape.bitsOut)}, {"output_bit_order", std::string(gr::meta::enumName(shape.orderOut).value_or(""))}}));
    }

    std::vector<Arm> arms;

    /// @brief A block arm: one call over the whole input, at the exact number of output items the widths imply.
    const auto block = [&arms, &input, &output](std::string label, auto& converter, unsigned bitsIn, unsigned bitsOut) {
        const std::size_t outItems = kItemsPerCall * bitsIn / bitsOut;
        arms.push_back({std::move(label),
            [&converter, &input, &output, outItems] {
                InputSpan<std::uint8_t>  inSpan{std::span<const std::uint8_t>(input)};
                OutputSpan<std::uint8_t> outSpan(std::span<std::uint8_t>(output.data(), outItems));
                std::ignore = converter.processBulk(inSpan, outSpan);
                return static_cast<double>(output[outItems / 2UZ]);
            },
            kItemsPerCall * bitsIn});
    };

    /// @brief A kernel arm, which is how the two forms of the eight-bit shapes are put side by side.
    const auto kernel = [&arms, &input, &output](std::string label, const gr::digital::BitRepack& config, bool general) {
        const std::size_t outItems = kItemsPerCall * config.bitsIn / config.bitsOut;
        arms.push_back({std::move(label),
            [&config, &input, &output, outItems, general] {
                const std::span<const std::uint8_t> in(input);
                const std::span<std::uint8_t>       out(output.data(), outItems);
                if (general) {
                    gr::digital::detail::repackGeneral(config, in, out);
                } else {
                    gr::digital::repack(config, in, out);
                }
                return static_cast<double>(output[outItems / 2UZ]);
            },
            kItemsPerCall * config.bitsIn});
    };

    block("PackBits k=8, msb_first", packMsb, 1U, 8U);
    block("PackBits k=8, lsb_first", packLsb, 1U, 8U);
    block("UnpackBits k=8, msb_first", unpackMsb, 8U, 1U);
    block("UnpackBits k=8, lsb_first", unpackLsb, 8U, 1U);
    for (std::size_t i = 0UZ; i < shapes.size(); ++i) {
        block(shapes[i].label, repackers[i], shapes[i].bitsIn, shapes[i].bitsOut);
    }

    const std::array<gr::digital::BitRepack, 4UZ> eightBit{kernelFor(1U, 8U, BitOrder::MsbFirst, BitOrder::MsbFirst), kernelFor(1U, 8U, BitOrder::MsbFirst, BitOrder::LsbFirst), kernelFor(8U, 1U, BitOrder::MsbFirst, BitOrder::MsbFirst), kernelFor(8U, 1U, BitOrder::LsbFirst, BitOrder::MsbFirst)};
    const std::array<std::string_view, 4UZ>       eightBitLabels{"pack 1:8 msb_first", "pack 1:8 lsb_first", "unpack 8:1 msb_first", "unpack 8:1 lsb_first"};
    for (std::size_t i = 0UZ; i < eightBit.size(); ++i) {
        kernel(std::format("{}, 64-bit form", eightBitLabels[i]), eightBit[i], false);
        kernel("  the same, general loop", eightBit[i], true);
    }

    arms.push_back({"a plain span copy, the floor",
        [&input, &copied] {
            std::ranges::copy(input, copied.begin());
            return static_cast<double>(copied[kItemsPerCall / 2UZ]);
        },
        kItemsPerCall * 8UZ});

    gr::blocks::testing::bench::report(std::span<Arm>(arms), kItemsPerCall, kRepeats);
}
