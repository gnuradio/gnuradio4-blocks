#include <benchmark.hpp>

#include <gnuradio-4.0/algorithm/filter/PolyphaseResampler.hpp>

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <random>
#include <span>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

constexpr std::size_t kSamples = 65'536UZ;
constexpr std::size_t kRepeats = 20UZ;

struct Setting {
    std::string_view   name;
    std::size_t        l = 1UZ;
    std::size_t        m = 1UZ;
    std::vector<float> taps;
};

/// The single-stage decimators `1/D` at 92 dB and 0.05 dB for two passband widths, then `designResampler`'s defaults.
[[nodiscard]] std::vector<Setting> settings() {
    std::vector<Setting> out;
    constexpr std::array kDecimators{std::tuple{"1/4 W0.24", 4UZ, 0.24}, std::tuple{"1/4 W0.50", 4UZ, 0.50}, std::tuple{"1/25 W0.24", 25UZ, 0.24}, std::tuple{"1/25 W0.50", 25UZ, 0.50}, std::tuple{"1/250 W0.24", 250UZ, 0.24}, std::tuple{"1/250 W0.50", 250UZ, 0.50}, std::tuple{"1/1000 W0.24", 1000UZ, 0.24}, std::tuple{"1/1000 W0.50", 1000UZ, 0.50}};
    for (const auto& [name, d, width] : kDecimators) {
        const std::array<std::size_t, 1UZ>      ladder{d};
        const gr::filter::StagedDecimatorDesign design = gr::filter::designDecimatorLadder(std::span<const std::size_t>(ladder), d, width, 92.0, 0.05);
        out.push_back({name, 1UZ, d, design.stages.front().taps});
    }
    constexpr std::array kResamplers{std::tuple{"4/1", 4UZ, 1UZ}, std::tuple{"1/2", 1UZ, 2UZ}, std::tuple{"3/2", 3UZ, 2UZ}, std::tuple{"1/25", 1UZ, 25UZ}, std::tuple{"147/160", 147UZ, 160UZ}};
    for (const auto& [name, l, m] : kResamplers) {
        out.push_back({name, l, m, gr::filter::designResampler(l, m).taps});
    }
    return out;
}

template<typename T>
[[nodiscard]] std::vector<T> noise(std::size_t n, std::uint32_t seed) {
    std::mt19937                          rng(seed);
    std::uniform_real_distribution<float> uniform(-1.0f, 1.0f);
    std::vector<T>                        out(n);
    for (T& v : out) {
        if constexpr (std::is_floating_point_v<T>) {
            v = static_cast<T>(uniform(rng));
        } else {
            using V        = typename T::value_type;
            const float re = uniform(rng);
            v              = T{static_cast<V>(re), static_cast<V>(uniform(rng))};
        }
    }
    return out;
}

/// Complex taps of the same magnitudes as the real ones, turned by a slow rotation.
template<typename TTap>
[[nodiscard]] std::vector<TTap> tapsAs(const std::vector<float>& taps) {
    if constexpr (std::is_floating_point_v<TTap>) {
        return std::vector<TTap>(taps.begin(), taps.end());
    } else {
        std::vector<TTap> out(taps.size());
        for (std::size_t i = 0UZ; i < taps.size(); ++i) {
            out[i] = TTap(std::polar(taps[i], 0.01f * static_cast<float>(i)));
        }
        return out;
    }
}

template<typename TSample, typename TTap>
void benchSetting(std::string_view type, const Setting& setting, std::size_t call = kSamples) {
    using Out = typename gr::filter::PolyphaseResampler<TSample, TTap>::output_type;

    const std::vector<TTap>                       taps = tapsAs<TTap>(setting.taps);
    const std::vector<TSample>                    in   = noise<TSample>(kSamples, 1U);
    gr::filter::PolyphaseResampler<TSample, TTap> resampler(setting.l, setting.m, std::span<const TTap>(taps));
    std::vector<Out>                              out(resampler.outputsFor(call) + setting.l);

    const std::size_t total                                         = kSamples / call * call;
    const auto        name                                          = std::format("{} {:<12} {:5} taps, {:4} a branch, {:6.1f} MAC/input, {:5} a call", type, setting.name, setting.taps.size(), resampler.branchLength(), static_cast<double>(resampler.taps()) / static_cast<double>(setting.m), call);
    ::benchmark::benchmark<kRepeats>(std::string_view(name), total) = [&] {
        for (std::size_t at = 0UZ; at < total; at += call) {
            std::ignore = resampler.process(std::span<const TSample>(in).subspan(at, call), std::span<Out>(out).first(resampler.outputsFor(call)));
        }
    };
}

} // namespace

/// The resampler's cost per input sample: complex samples on real taps at every setting, the other sample and tap
/// types at a decimator and an interpolator, and calls of a few chunks at 147/160, as an audio chain makes.
void benchPolyphaseResampler() {
    using namespace benchmark;
    using boost::ut::operator""_test;

    "PolyphaseResampler: ns per input sample"_test = [] {
        using CF                         = std::complex<float>;
        const std::vector<Setting> table = settings();
        for (const Setting& setting : table) {
            benchSetting<CF, float>("cf", setting);
        }
        ::benchmark::results::add_separator();
        for (const Setting& setting : table) {
            if (setting.name == "1/25 W0.24" || setting.name == "4/1") {
                benchSetting<CF, CF>("cc", setting);
                benchSetting<float, CF>("fc", setting);
                benchSetting<float, float>("ff", setting);
                benchSetting<std::complex<double>, double>("cd", setting);
            }
        }
        ::benchmark::results::add_separator();
        for (const Setting& setting : table) {
            if (setting.name == "147/160") {
                for (const std::size_t call : {160UZ, 480UZ}) {
                    benchSetting<CF, float>("cf", setting, call);
                    benchSetting<float, float>("ff", setting, call);
                }
            }
        }
        ::benchmark::results::add_separator();
    };
}

inline const boost::ut::suite<"polyphase resampler benchmarks"> _polyphase_resampler_bm = [] { benchPolyphaseResampler(); };

int main() { /* not needed by the UT framework */ }
