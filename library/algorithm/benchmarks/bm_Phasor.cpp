#include <benchmark.hpp>

#include <complex>
#include <concepts>
#include <cstddef>
#include <format>
#include <span>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/algorithm/signal/Phasor.hpp>

namespace {

// ns per sample of the fixed-increment entry points, in calls of 2 Ki and 64 Ki samples
template<std::floating_point F>
void benchPhasor(std::string_view typeName) {
    constexpr std::size_t nRepetitions = 500UZ;
    for (const std::size_t n : {2048UZ, 65536UZ}) {
        std::vector<std::complex<F>> in(n, std::complex<F>(F(1), F(0)));
        std::vector<std::complex<F>> out(n);
        {
            gr::signal::Phasor<F> phasor;
            phasor.configure(0.2718281828459045, 0.);
            const auto name                                                 = std::format("phasor fill - {:6}, {:5} samples per call", typeName, n);
            ::benchmark::benchmark<nRepetitions>(std::string_view(name), n) = [&] {
                phasor.fill(out);
                ::benchmark::force_to_memory(out);
            };
        }
        {
            gr::signal::Phasor<F> phasor;
            phasor.configure(0.2718281828459045, 0.);
            const auto name                                                 = std::format("phasor mix  - {:6}, {:5} samples per call", typeName, n);
            ::benchmark::benchmark<nRepetitions>(std::string_view(name), n) = [&] {
                phasor.mix(in, out);
                ::benchmark::force_to_memory(out);
            };
        }
    }
    ::benchmark::results::add_separator();
}

} // namespace

inline const boost::ut::suite _phasor_bm_tests = [] {
    benchPhasor<float>("float");
    benchPhasor<double>("double");
};

int main() { /* not needed by the UT framework */ }
