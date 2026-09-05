#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <algorithm>
#include <array>
#include <complex>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/filter/FirFilter.hpp>

namespace {

using gr::blocks::filter::FirFilter;
using CF = std::complex<float>;

/// Five representative receiver shapes, in all four `(sample, tap)` combinations: the multiply
/// counts are `4 : 2 : 2 : 1`, and the table checks that the measured ratios match.
struct Shape {
    const char* label;
    std::size_t taps;
    std::size_t decimation;
};

constexpr Shape kShapes[] = {{"RDS-shaped", 143UZ, 10UZ}, {"downconverter", 137UZ, 25UZ}, {"channel filter", 233UZ, 1UZ}, {"CW filter", 11001UZ, 1UZ}, {"deep decimation", 143UZ, 64UZ}};

constexpr std::size_t kRepeats  = 5UZ;
constexpr double      kMacsGoal = 5e7; /// a pass is about this many multiply-accumulates, matching the sizing of the kernel's own table
constexpr std::size_t kMaxPass  = 1UZ << 18;

struct Noise {
    std::uint64_t state = 0x2545F4914F6CDD1DULL;

    [[nodiscard]] float next() noexcept {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return static_cast<float>(static_cast<double>(state >> 11) / 9007199254740992.0 * 2.0 - 1.0);
    }
};

template<typename T>
[[nodiscard]] std::vector<T> noise(std::size_t n, std::uint64_t seed) {
    Noise          source{seed};
    std::vector<T> out(n);
    for (T& v : out) {
        if constexpr (std::same_as<T, CF>) {
            v = CF{source.next(), source.next()};
        } else {
            v = source.next();
        }
    }
    return out;
}

[[nodiscard]] std::size_t passLength(const Shape& shape) {
    const std::size_t wanted = static_cast<std::size_t>(kMacsGoal * static_cast<double>(shape.decimation) / static_cast<double>(shape.taps));
    const std::size_t capped = std::min(kMaxPass, std::max(4096UZ, wanted));
    return (capped / shape.decimation) * shape.decimation; // whole chunks, which is what the framework hands the block
}

/// @brief One arm: a block of this shape and type pair, its input, its output, and the call that drives it.
template<typename TSample, typename TTap>
struct Bed {
    FirFilter<TSample, TTap>                             block;
    std::vector<TSample>                                 input;
    std::vector<typename FirFilter<TSample, TTap>::TOut> output;

    Bed(const Shape& shape) : block({{"taps", noise<TTap>(shape.taps, 0x9E3779B97F4A7C15ULL)}, {"decimation", static_cast<gr::Size_t>(shape.decimation)}}), input(noise<TSample>(passLength(shape), 0xBF58476D1CE4E5B9ULL)), output(passLength(shape) / shape.decimation) {
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
    }

    [[nodiscard]] double run() {
        std::ignore = block.processBulk(std::span<const TSample>(input), std::span<typename FirFilter<TSample, TTap>::TOut>(output));
        return static_cast<double>(std::abs(output[output.size() / 2UZ]));
    }
};

} // namespace

int main() {
    std::vector<gr::blocks::testing::bench::Arm> arms;

    const auto add = [&arms]<typename TSample, typename TTap>(const Shape& shape, const char* combination, std::unique_ptr<Bed<TSample, TTap>>& bed) {
        bed = std::make_unique<Bed<TSample, TTap>>(shape);
        arms.emplace_back(std::format("{} {} N={} M={}", combination, shape.label, shape.taps, shape.decimation), passLength(shape), static_cast<double>(shape.taps) / static_cast<double>(shape.decimation), [raw = bed.get()] { return raw->run(); });
    };

    std::vector<std::unique_ptr<Bed<float, float>>> ff(std::size(kShapes));
    std::vector<std::unique_ptr<Bed<CF, float>>>    cf(std::size(kShapes));
    std::vector<std::unique_ptr<Bed<float, CF>>>    fc(std::size(kShapes));
    std::vector<std::unique_ptr<Bed<CF, CF>>>       cc(std::size(kShapes));

    for (std::size_t s = 0UZ; s < std::size(kShapes); ++s) {
        add.template operator()<float, float>(kShapes[s], "ff", ff[s]);
        add.template operator()<CF, float>(kShapes[s], "cf", cf[s]);
        add.template operator()<float, CF>(kShapes[s], "fc", fc[s]);
        add.template operator()<CF, CF>(kShapes[s], "cc", cc[s]);
    }

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kRepeats);
}
