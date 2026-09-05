#include <cstddef>
#include <cstdint>
#include <print>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/fec/InterleaveBlocks.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using Deinterleave = gr::blocks::fec::Deinterleave<std::uint8_t>;
using Interleave   = gr::blocks::fec::Interleave<std::uint8_t>;
using gr::blocks::testing::bench::Arm;
namespace shim = gr::blocks::testing::span;

using Record = gr::DataSet<std::uint8_t>;

constexpr std::size_t kItems   = 1UZ << 16; ///< items a record, which is one call's work
constexpr std::size_t kRepeats = 9UZ;

template<typename TBlock>
void init(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

[[nodiscard]] Record record(std::size_t items) {
    Record r;
    r.signal_values.resize(items);
    for (std::size_t i = 0UZ; i < items; ++i) {
        r.signal_values[i] = static_cast<std::uint8_t>((i * 37UZ + 11UZ) & 0xFFUZ);
    }
    r.extents.push_back(static_cast<std::int32_t>(items));
    r.signal_names.emplace_back("fec");
    r.timing_events.resize(1UZ);
    return r;
}

//! A seeded permutation of @p items, so the permutation kind gathers from everywhere rather than
//! walking a rectangle's stride and reading the cache lines a rectangle happens to reuse.
[[nodiscard]] std::vector<gr::Size_t> scatterTable(std::size_t items) {
    std::vector<gr::Size_t> table(items);
    for (std::size_t i = 0UZ; i < items; ++i) {
        table[i] = static_cast<gr::Size_t>(i);
    }
    std::uint32_t state = 987654321U;
    for (std::size_t i = items; i > 1UZ; --i) {
        state                     = state * 1664525U + 1013904223U;
        const std::size_t swapAt  = state % i;
        const gr::Size_t  carried = table[i - 1UZ];
        table[i - 1UZ]            = table[swapAt];
        table[swapAt]             = carried;
    }
    return table;
}

//! One call of a record-shaped block over one record, which is the shape a graph calls it in.
template<typename TBlock>
[[nodiscard]] double sweep(TBlock& block, std::span<const Record> input, std::span<Record> output) {
    shim::InputSpan<Record>  inSpan(input, 0UZ);
    shim::OutputSpan<Record> outSpan{output};
    std::ignore = block.processBulk(inSpan, outSpan);
    return static_cast<double>(output[0UZ].signal_values[0UZ]);
}

} // namespace

int main() {
    const std::vector<Record> input{record(kItems)};
    std::vector<Record>       sink(1UZ);

    Interleave   rectangle({{"kind", std::string("block")}, {"rows", gr::Size_t{256U}}, {"cols", gr::Size_t{256U}}});
    Deinterleave rectangleBack({{"kind", std::string("block")}, {"rows", gr::Size_t{256U}}, {"cols", gr::Size_t{256U}}});
    Interleave   scatter({{"kind", std::string("permutation")}, {"table", scatterTable(kItems)}});
    Interleave   forney({{"kind", std::string("convolutional")}, {"branches", gr::Size_t{16U}}, {"unit_delay", gr::Size_t{4U}}});
    Deinterleave forneyBack({{"kind", std::string("convolutional")}, {"branches", gr::Size_t{16U}}, {"unit_delay", gr::Size_t{4U}}});
    init(rectangle);
    init(rectangleBack);
    init(scatter);
    init(forney);
    init(forneyBack);

    std::vector<Arm> arms{
        {"a record copied, the floor",
            [&input, &sink] {
                sink[0UZ] = input[0UZ];
                return static_cast<double>(sink[0UZ].signal_values[0UZ]);
            }},
        {"Interleave, block 256 x 256", [&] { return sweep(rectangle, std::span<const Record>(input), std::span<Record>(sink)); }},
        {"Deinterleave, block 256 x 256", [&] { return sweep(rectangleBack, std::span<const Record>(input), std::span<Record>(sink)); }},
        {"Interleave, permutation of 65536", [&] { return sweep(scatter, std::span<const Record>(input), std::span<Record>(sink)); }},
        {"Interleave, convolutional B 16 M 4", [&] { return sweep(forney, std::span<const Record>(input), std::span<Record>(sink)); }},
        {"Deinterleave, convolutional B 16 M 4", [&] { return sweep(forneyBack, std::span<const Record>(input), std::span<Record>(sink)); }},
    };

    std::println("ns per item. The framed kinds are a gather against a resolved table and the convolutional kind is a");
    std::println("walk over B ring buffers, so the two are different memory patterns rather than different amounts of");
    std::println("arithmetic; the floor is the record copy the block cannot avoid making.");
    gr::blocks::testing::bench::report(std::span<Arm>(arms), kItems, kRepeats);
    return 0;
}
