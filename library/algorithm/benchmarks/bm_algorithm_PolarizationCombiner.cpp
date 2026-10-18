#include <benchmark.hpp>

#include <gnuradio-4.0/algorithm/measurement/PolarizationCombiner.hpp>
#include <gnuradio-4.0/algorithm/rng/GaussianNoise.hpp>
#include <gnuradio-4.0/algorithm/rng/Xoshiro256pp.hpp>

#include <complex>
#include <cstddef>
#include <format>
#include <string_view>
#include <tuple>
#include <vector>

namespace {

using Complex = std::complex<float>;

constexpr std::size_t kSamples = 65'536UZ;
constexpr std::size_t kRepeats = 200UZ;

} // namespace

/// Times `polarizationCombine` with and without the orthogonal output, and `BranchCovariance::add`. Per sample, the
/// combination costs two complex multiply-adds, and the orthogonal output two more. `solve()` runs once per window and
/// is not timed.
void benchPolarizationCombiner() {
    using namespace benchmark;
    using boost::ut::operator""_test;

    "PolarizationCombiner: ns/sample"_test = [] {
        gr::rng::Xoshiro256pp         rng(3ULL);
        gr::rng::GaussianNoise<float> noise(rng);

        std::vector<Complex> branch0(kSamples);
        std::vector<Complex> branch1(kSamples);
        noise.fillComplex(branch0);
        noise.fillComplex(branch1);

        gr::measurement::BranchCovariance covariance;
        std::ignore                                          = covariance.add(branch0, branch1);
        const gr::measurement::PolarizationEstimate estimate = covariance.solve().value();

        std::vector<Complex> out(kSamples);
        std::vector<Complex> ortho(kSamples);

        const auto combined                                                    = std::format("polarizationCombine, no orthogonal port, N={}", kSamples);
        ::benchmark::benchmark<kRepeats>(std::string_view(combined), kSamples) = [&] { std::ignore = gr::measurement::polarizationCombine(branch0, branch1, estimate, out); };
        const auto both                                                        = std::format("polarizationCombine, with it,           N={}", kSamples);
        ::benchmark::benchmark<kRepeats>(std::string_view(both), kSamples)     = [&] { std::ignore = gr::measurement::polarizationCombine(branch0, branch1, estimate, out, ortho); };

        const auto accumulate                                                    = std::format("BranchCovariance::add,                  N={}", kSamples);
        ::benchmark::benchmark<kRepeats>(std::string_view(accumulate), kSamples) = [&] {
            gr::measurement::BranchCovariance window;
            std::ignore = window.add(branch0, branch1);
            ::benchmark::force_to_memory(window);
        };
        ::benchmark::results::add_separator();
    };
}

inline const boost::ut::suite<"polarization benchmarks"> _polarization_bm = [] { benchPolarizationCombiner(); };

int main() { /* not needed by the UT framework */ }
