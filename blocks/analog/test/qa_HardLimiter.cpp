#include <boost/ut.hpp>

#include <cmath>
#include <complex>
#include <vector>

#include <gnuradio-4.0/analog/HardLimiter.hpp>

const boost::ut::suite<"HardLimiter"> hardLimiterTests = [] {
    using namespace boost::ut;
    using gr::blocks::analog::HardLimiter;
    using CF = std::complex<float>;

    static_assert(gr::HasConstProcessOneFunction<HardLimiter<CF>>, "stateless: may sit anywhere in a fused run");
    static_assert(gr::HasConstProcessOneFunction<HardLimiter<float>>, "the real form too");

    "every output has unit magnitude and the input's exact phase"_test = [] {
        HardLimiter<CF> block;
        for (const float magnitude : {1e-30f, 1e-6f, 0.001f, 0.5f, 1.0f, 42.0f, 1e12f, 1e30f}) {
            for (int step = 0; step < 16; ++step) {
                const float phase = static_cast<float>(step) * 0.3926991f - 3.0f;
                const CF    in    = std::polar(magnitude, phase);
                const CF    out   = block.processOne(in);
                expect(lt(std::abs(std::abs(out) - 1.0f), 1e-6f)) << "unit magnitude";
                expect(lt(std::abs(std::arg(out * std::conj(in))), 1e-6f)) << "the phase is untouched";
            }
        }
    };

    "zero passes as zero, never a non-finite value"_test = [] {
        HardLimiter<CF> complexBlock;
        const CF        zc = complexBlock.processOne(CF{0.f, 0.f});
        expect(eq(zc.real(), 0.f) && eq(zc.imag(), 0.f));

        HardLimiter<float> realBlock;
        expect(eq(realBlock.processOne(0.f), 0.f));
        expect(eq(realBlock.processOne(-0.f), 0.f));
    };

    "the real form is the signum"_test = [] {
        HardLimiter<float> block;
        expect(eq(block.processOne(0.001f), 1.f));
        expect(eq(block.processOne(1e30f), 1.f));
        expect(eq(block.processOne(-0.001f), -1.f));
        expect(eq(block.processOne(-1e30f), -1.f));
    };
};

int main() { /* not needed for UT */ }
