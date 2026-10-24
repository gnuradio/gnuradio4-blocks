#include <benchmark.hpp>

#include <gnuradio-4.0/algorithm/filter/HalfbandCascade.hpp>
#include <gnuradio-4.0/algorithm/filter/RealToComplex.hpp>

#include <complex>
#include <cstddef>
#include <format>
#include <random>
#include <span>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace {

using Complex = std::complex<float>;

constexpr std::size_t kSamples = 65'536UZ;
constexpr std::size_t kChunk   = 8'192UZ;
constexpr std::size_t kRepeats = 200UZ;

[[nodiscard]] std::vector<float> noise(std::size_t n) {
    std::mt19937                          rng(1U);
    std::uniform_real_distribution<float> uniform(-1.f, 1.f);
    std::vector<float>                    x(n);
    for (float& v : x) {
        v = uniform(rng);
    }
    return x;
}

} // namespace

/// The conversion's own loop against the same conversion through `HalfbandCascade`: the mix into a complex buffer,
/// then one halving stage over the same taps. Both read the input in chunks of 8192 samples, as a scheduler hands them
/// over, and the two rows of each length run twice, alternately. The sample count is the input's, so ops/s is input
/// samples per second.
void benchRealToComplex() {
    using namespace benchmark;
    using boost::ut::operator""_test;

    "RealToComplex: the conversion's loop and HalfbandCascade, per input sample"_test = [] {
        const std::vector<float> in = noise(kSamples);
        for (const auto& [rate, transition] : {std::pair{500'000.0, 20'000.0}, std::pair{48'000.0, 200.0}}) {
            gr::filter::RealToComplex conversion = gr::filter::RealToComplex::make(rate, transition).value();
            const std::size_t         nTaps      = conversion.taps().size();
            std::vector<Complex>      out(kChunk / 2UZ + 1UZ);

            const std::vector<std::vector<float>> stage{std::vector<float>(conversion.taps().begin(), conversion.taps().end())};
            gr::filter::HalfbandCascade           cascade(stage);
            cascade.primeWithSilence();
            std::vector<Complex> mixed(kChunk);
            std::vector<Complex> halved;
            halved.reserve(kChunk / 2UZ + 1UZ);

            for (int round = 0; round < 2; ++round) {
                const auto own                                                    = std::format("RealToComplex::process, {} taps", nTaps);
                ::benchmark::benchmark<kRepeats>(std::string_view(own), kSamples) = [&] {
                    for (std::size_t at = 0UZ; at < kSamples; at += kChunk) {
                        std::ignore = conversion.process(std::span(in).subspan(at, kChunk), std::span(out));
                    }
                    ::benchmark::force_to_memory(out);
                };

                const auto viaCascade                                                    = std::format("mix + HalfbandCascade,  {} taps", nTaps);
                ::benchmark::benchmark<kRepeats>(std::string_view(viaCascade), kSamples) = [&] {
                    for (std::size_t at = 0UZ; at < kSamples; at += kChunk) {
                        for (std::size_t i = 0UZ; i < kChunk; i += 4UZ) { // the mixer sequence 1, -j, -1, +j
                            mixed[i]       = Complex{in[at + i], 0.f};
                            mixed[i + 1UZ] = Complex{0.f, -in[at + i + 1UZ]};
                            mixed[i + 2UZ] = Complex{-in[at + i + 2UZ], 0.f};
                            mixed[i + 3UZ] = Complex{0.f, in[at + i + 3UZ]};
                        }
                        halved.clear();
                        std::ignore = cascade.push(mixed, halved);
                    }
                    ::benchmark::force_to_memory(halved);
                };
            }
            ::benchmark::results::add_separator();
        }
    };
}

inline const boost::ut::suite<"real to complex benchmarks"> _real_to_complex_bm = [] { benchRealToComplex(); };

int main() { /* not needed by the UT framework */ }
