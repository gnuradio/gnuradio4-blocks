#include <boost/ut.hpp>

#include <cmath>
#include <complex>
#include <format>
#include <numbers>
#include <print>
#include <span>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/algorithm/rng/Xoshiro256pp.hpp>
#include <gnuradio-4.0/channel/Nonlinearity.hpp>
#include <gnuradio-4.0/channel/Quantizer.hpp>

namespace {

using C = std::complex<float>;

template<typename Block>
[[nodiscard]] Block configured(gr::property_map settings) {
    Block block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

} // namespace

const boost::ut::suite<"converter models"> converterTests = [] {
    using namespace boost::ut;
    using gr::blocks::channel::Nonlinearity;
    using gr::blocks::channel::Quantizer;

    // the closed form is the oracle, so the block is checked against the equation itself
    "Rapp reproduces its closed AM/AM curve and leaves phase alone"_test = [] {
        auto block = configured<Nonlinearity<C>>({{"model", "Rapp"}, {"saturation", 1.0}, {"smoothness", 2.0}});

        for (const double amplitude : {0.01, 0.1, 0.5, 0.9, 1.0, 1.5, 4.0}) {
            constexpr double phase   = 0.7;
            const C          sample  = C(static_cast<float>(amplitude * std::cos(phase)), static_cast<float>(amplitude * std::sin(phase)));
            const C          got     = block.processOne(sample);
            const double     ratio   = std::pow(amplitude / 1.0, 4.0); // (A/A_sat)^(2p), p = 2
            const double     wantAmp = amplitude / std::pow(1. + ratio, 0.25);

            expect(lt(std::abs(std::hypot(static_cast<double>(got.real()), static_cast<double>(got.imag())) - wantAmp), 1e-5)) //
                << std::format("Rapp amplitude at A={}", amplitude);
            expect(lt(std::abs(std::arg(std::complex<double>(got)) - phase), 1e-5)) //
                << std::format("Rapp must not rotate, A={}", amplitude);
        }
    };

    // well below saturation the amplifier is linear to the order the knee implies
    "Rapp is unity gain far below saturation"_test = [] {
        auto             block     = configured<Nonlinearity<C>>({{"model", "Rapp"}, {"saturation", 1.0}, {"smoothness", 2.0}});
        constexpr double amplitude = 0.01;
        const C          got       = block.processOne(C(static_cast<float>(amplitude), 0.f));
        // the leading error term is (A/A_sat)^(2p)/(2p) = 1e-8/4
        expect(lt(std::abs(static_cast<double>(got.real()) / amplitude - 1.), 1e-7)) << "Rapp must be linear far below saturation";
    };

    "Saleh reproduces both its AM/AM and AM/PM curves"_test = [] {
        constexpr double alphaA = 2.1587, betaA = 1.1517, alphaP = 4.0033, betaP = 9.1040;
        auto             block = configured<Nonlinearity<C>>({{"model", "Saleh"}});

        for (const double amplitude : {0.05, 0.2, 0.5, 1.0, 2.0}) {
            const C      got      = block.processOne(C(static_cast<float>(amplitude), 0.f));
            const double squared  = amplitude * amplitude;
            const double wantAmp  = alphaA * amplitude / (1. + betaA * squared);
            const double wantPhi  = alphaP * squared / (1. + betaP * squared);
            const auto   asDouble = std::complex<double>(got);

            expect(lt(std::abs(std::abs(asDouble) - wantAmp), 1e-5)) << std::format("Saleh AM/AM at A={}", amplitude);
            expect(lt(std::abs(std::arg(asDouble) - wantPhi), 1e-5)) << std::format("Saleh AM/PM at A={}", amplitude);
        }
    };

    "input backoff moves the operating point"_test = [] {
        // 20*log10(2), not the colloquial 6 dB: 6 dB is a factor of 0.50119, and asserting an exact halving
        // against it would be testing the rounding rather than the backoff
        const double halving  = 20. * std::log10(2.);
        auto         plain    = configured<Nonlinearity<C>>({{"model", "Rapp"}, {"saturation", 1.0}});
        auto         backedIn = configured<Nonlinearity<C>>({{"model", "Rapp"}, {"saturation", 1.0}, {"input_backoff_db", halving}});

        const C direct = plain.processOne(C(0.5f, 0.f));
        const C backed = backedIn.processOne(C(1.0f, 0.f));
        expect(lt(std::abs(static_cast<double>(direct.real()) - static_cast<double>(backed.real())), 1e-6)) << "backing the drive off by a factor of two must match driving at half the amplitude";
    };

    "a zero envelope passes through exactly"_test = [] {
        auto block = configured<Nonlinearity<C>>({{"model", "Saleh"}});
        expect(eq(block.processOne(C(0.f, 0.f)), C(0.f, 0.f)));
    };

    "nonlinearity refuses a non-positive saturation or smoothness"_test = [] {
        expect(throws([] { std::ignore = configured<Nonlinearity<C>>({{"saturation", 0.0}}); }));
        expect(throws([] { std::ignore = configured<Nonlinearity<C>>({{"smoothness", -1.0}}); }));
    };

    // split in two because the textbook SQNR formula and the property it rests on are not the
    // same claim. The property is that the quantization error power is delta^2/12, which holds exactly, at
    // every word length, for an input that visits the codes evenly.
    "quantizer error power is delta squared over twelve"_test = [] {
        constexpr std::size_t nSamples = 500'000UZ;
        for (const gr::Size_t bits : {8U, 12U, 16U}) {
            auto                  block = configured<Quantizer<float>>({{"bits", bits}, {"full_scale", 1.0}});
            gr::rng::Xoshiro256pp rng(0x51F0u);

            double errorPower = 0.;
            for (std::size_t k = 0UZ; k < nSamples; ++k) {
                const auto   value = static_cast<float>(0.9 * rng.uniformM11<double>());
                const double error = static_cast<double>(block.processOne(value)) - static_cast<double>(value);
                errorPower += error * error;
            }
            errorPower /= static_cast<double>(nSamples);

            const double delta    = std::pow(2., 1. - static_cast<double>(bits));
            const double expected = delta * delta / 12.;
            expect(lt(std::abs(errorPower / expected - 1.), 0.01)) << std::format("{} bits: error power {:g}, expected {:g}", bits, errorPower, expected);
        }
    };

    // The formula 6.02*B + 1.76 additionally assumes the error is uncorrelated with the signal, which a sine
    // satisfies only once it has plenty of codes to cross. A 0.9-of-full-scale sine sits within 0.15 dB of the
    // ideal at 12 and 16 bits, and 1.5 dB under it at 8, where 115 levels leave the error correlated with the
    // waveform and lift its power to 1.147 times delta^2/12. That is a property of quantizing a sine at this
    // depth, so each figure is pinned rather than derived, and a change in any of them is a real change.
    "quantizer SQNR on a sine, where the formula's assumptions hold and where they do not"_test = [] {
        constexpr std::size_t nSamples  = 200'000UZ;
        constexpr double      amplitude = 0.9;

        for (const auto& [bits, expectedDb, tolerance] : std::vector<std::tuple<gr::Size_t, double, double>>{{8U, 48.4, 0.2}, {12U, 73.09, 0.2}, {16U, 97.18, 0.2}}) {
            auto block = configured<Quantizer<float>>({{"bits", bits}, {"full_scale", 1.0}});

            double signalPower = 0., errorPower = 0.;
            for (std::size_t k = 0UZ; k < nSamples; ++k) {
                const double phase = 2. * std::numbers::pi_v<double> * 0.0123456789 * static_cast<double>(k);
                const auto   value = static_cast<float>(amplitude * std::sin(phase));
                const double error = static_cast<double>(block.processOne(value)) - static_cast<double>(value);
                signalPower += static_cast<double>(value) * static_cast<double>(value);
                errorPower += error * error;
            }
            const double delta    = std::pow(2., 1. - static_cast<double>(bits));
            const double measured = 10. * std::log10(signalPower / errorPower);
            const double ideal    = 10. * std::log10((amplitude * amplitude / 2.) / (delta * delta / 12.));
            std::println("Quantizer {:2} bits: sine SQNR {:.2f} dB (delta^2/12 ideal {:.2f}, full-scale textbook {:.2f})", bits, measured, ideal, 6.02 * static_cast<double>(bits) + 1.76);
            expect(lt(std::abs(measured - expectedDb), tolerance)) << std::format("{} bits: measured {:.2f} dB, expected {:.2f}", bits, measured, expectedDb);
        }
    };

    // an already quantized value must land back on the same code
    "quantizing twice equals quantizing once"_test = [] {
        auto block = configured<Quantizer<C>>({{"bits", gr::Size_t(10)}, {"full_scale", 1.0}});
        for (std::size_t k = 0UZ; k < 1000UZ; ++k) {
            const auto value = C(static_cast<float>(std::sin(0.017 * static_cast<double>(k))), static_cast<float>(std::cos(0.013 * static_cast<double>(k))));
            const C    once  = block.processOne(value);
            expect(eq(block.processOne(once), once)) << std::format("idempotence at {}", k);
        }
    };

    "values beyond full scale land exactly on the rails"_test = [] {
        auto             block = configured<Quantizer<float>>({{"bits", gr::Size_t(8)}, {"full_scale", 1.0}});
        constexpr double delta = 1.0 * 0.0078125; // full_scale * 2^(1-8)
        expect(lt(std::abs(static_cast<double>(block.processOne(5.f)) - 127. * delta), 1e-9)) << "positive rail";
        expect(lt(std::abs(static_cast<double>(block.processOne(-5.f)) - -128. * delta), 1e-9)) << "negative rail";
        // mid-tread: zero is a code, so it survives exactly
        expect(eq(block.processOne(0.f), 0.f));
    };

    "quantizer refuses an unusable word length or scale"_test = [] {
        expect(throws([] { std::ignore = configured<Quantizer<float>>({{"bits", gr::Size_t(1)}}); }));
        expect(throws([] { std::ignore = configured<Quantizer<float>>({{"bits", gr::Size_t(25)}}); }));
        expect(throws([] { std::ignore = configured<Quantizer<float>>({{"full_scale", 0.0}}); }));
    };
};

int main() { /* not needed for UT */ }
