#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <format>
#include <numbers>
#include <random>
#include <span>
#include <tuple>
#include <type_traits>
#include <vector>

#include <gnuradio-4.0/algorithm/signal/LanePhasor.hpp>

namespace {

constexpr double kTau = 2. * std::numbers::pi_v<double>;

struct Error {
    double phase = 0.;
    double mag   = 0.;
};

// The largest phase and magnitude error of `n` samples of fill against exp(j*phase_k) computed in double at each
// sample, taken in calls of `chunk`. Sample k is at startPhase + k*increment.
template<std::floating_point F, std::size_t ReseedInterval>
Error measure(gr::signal::LanePhasor<F, ReseedInterval>& p, double increment, double startPhase, std::size_t n, std::size_t chunk) {
    std::vector<std::complex<F>> out(chunk);
    Error                        e;
    for (std::size_t at = 0UZ; at < n; at += chunk) {
        const std::size_t take = std::min(chunk, n - at);
        p.fill(std::span<std::complex<F>>(out).first(take));
        for (std::size_t k = 0UZ; k < take; ++k) {
            const double               ph = std::remainder(startPhase + increment * static_cast<double>(at + k), kTau);
            const std::complex<double> y(out[k]);
            e.phase = std::max(e.phase, std::abs(std::arg(y * std::polar(1., -ph))));
            e.mag   = std::max(e.mag, std::abs(std::abs(y) - 1.));
        }
    }
    return e;
}

std::vector<std::complex<float>> noise(std::size_t n) {
    std::vector<std::complex<float>>      v(n);
    std::minstd_rand                      gen(3U);
    std::uniform_real_distribution<float> u(-1.f, 1.f);
    for (auto& x : v) {
        x = {u(gen), u(gen)};
    }
    return v;
}

template<typename F>
bool same(const std::vector<std::complex<F>>& a, const std::vector<std::complex<F>>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(std::complex<F>)) == 0;
}

// The largest distance of out[k] from in[k] * exp(j*(startPhase + k*increment)) taken in double, with `in` of ones when
// it is empty. A split stream holds the kernel's accuracy, and the last bits are the instruction set's.
template<typename F>
double worstDistance(const std::vector<std::complex<F>>& in, const std::vector<std::complex<F>>& out, double increment, double startPhase) {
    double worst = 0.;
    for (std::size_t k = 0UZ; k < out.size(); ++k) {
        const std::complex<double> x = in.empty() ? std::complex<double>(1., 0.) : std::complex<double>(in[k]);
        const std::complex<double> y = x * std::polar(1., std::remainder(startPhase + increment * static_cast<double>(k), kTau));
        worst                        = std::max(worst, std::abs(std::complex<double>(out[k]) - y));
    }
    return worst;
}

// the accuracy of a seeded stream of F at 256 samples between seeds: the phase and magnitude errors of the header
// bound the distance by about 1e-6 in float, and 2e-10 in double
template<typename F>
constexpr double kStreamTolerance = std::is_same_v<F, float> ? 4.0e-6 : 1.0e-9;

} // namespace

const boost::ut::suite<"LanePhasor"> lanePhasorTests = [] {
    using namespace boost::ut;
    using gr::signal::LanePhasor;

    // Over 2^26 samples at 256 samples between seeds the worst measured is 5.2e-7 in phase and 7.0e-7 in
    // magnitude; 1024 gives 1.7e-6 and 2.0e-6, and 4096 gives 5.9e-6 and 6.7e-6. A longer interval, or a seed that is
    // not exact, fails the 1e-6 bound.
    "float phase and magnitude over 2^26 samples stay within the seed's floor"_test = [] {
        constexpr double  bound = 1.0e-6;
        const double      inc   = kTau * -12.35e6 / 250.0e6;
        LanePhasor<float> p;
        p.configure(inc, 0.3);
        // the error is bounded by the seed, not by the length, so a run on which timing is not representative
        // (a sanitizer build) takes 2^22 samples, which still crosses 16384 seeds
        const std::size_t nLong = std::getenv("DISABLE_SENSITIVE_TESTS") != nullptr ? 1UZ << 22UZ : 1UZ << 26UZ;
        const Error       e     = measure(p, inc, 0.3, nLong, 65536UZ);
        expect(le(e.phase, bound)) << std::format("phase error {:.3e} rad over {} samples", e.phase, nLong);
        expect(le(e.mag, bound)) << std::format("magnitude error {:.3e} over {} samples", e.mag, nLong);

        for (const double f : {1.0e3, 83.3e6, -0.4e6}) {
            const double      inc2 = kTau * f / 250.0e6;
            LanePhasor<float> q;
            q.configure(inc2, -2.);
            const Error e2 = measure(q, inc2, -2., 1UZ << 22UZ, 65536UZ);
            expect(le(e2.phase, bound) && le(e2.mag, bound)) << std::format("increment of {:.0f} Hz: phase {:.3e}, magnitude {:.3e}", f, e2.phase, e2.mag);
        }
    };

    // In double the recurrence's own error is a few 1e-15, so the phase error measured here is the reference's: the
    // product increment*k reaches 1.1e5 radians at 2^18 samples, where a double step is 1.5e-11. Ten such steps
    // bound the phase; the magnitude has no such term.
    "double stays on the exact rotation"_test = [] {
        const double       inc = 0.4321;
        LanePhasor<double> p;
        p.configure(inc, 0.);
        const Error e = measure(p, inc, 0., 1UZ << 18UZ, 4099UZ);
        expect(le(e.phase, 1.5e-10) && le(e.mag, 1.0e-12)) << std::format("double: phase {:.3e}, magnitude {:.3e}", e.phase, e.mag);
    };

    // A group is anchored to the sample index. Cuts of 1 and 7 end inside groups, 16 on them, 1000 and 4096 across
    // the seed interval; the stream holds the kernel's accuracy at every cut.
    "mix holds its accuracy however it is cut"_test = [] {
        const auto   in  = noise(20011UZ);
        const double inc = 0.7317;
        for (const std::size_t chunk : {1UZ, 7UZ, 16UZ, 1000UZ, 4096UZ}) {
            LanePhasor<float> p;
            p.configure(inc, 1.1);
            std::vector<std::complex<float>> out(in.size());
            for (std::size_t at = 0UZ; at < in.size(); at += chunk) {
                const std::size_t take = std::min(chunk, in.size() - at);
                p.mix(std::span<const std::complex<float>>(in).subspan(at, take), std::span<std::complex<float>>(out).subspan(at, take));
            }
            const double worst = worstDistance(in, out, inc, 1.1);
            expect(lt(worst, kStreamTolerance<float>)) << std::format("calls of {}: the stream is {:.3e} from the double reference", chunk, worst);
        }
    };

    // The same holds for fill, for double, and with an empty call between any two others: a zero-length call writes
    // nothing, so it must leave the stream where it was.
    "fill holds its accuracy however it is cut, empty calls included"_test = [] {
        const double          inc = -1.9;
        constexpr std::size_t n   = 20011UZ;
        for (const std::size_t chunk : {1UZ, 7UZ, 16UZ, 1000UZ, 4096UZ}) {
            LanePhasor<float>  p;
            LanePhasor<double> pd;
            p.configure(inc, -0.7);
            pd.configure(inc, -0.7);
            std::vector<std::complex<float>>  out(n);
            std::vector<std::complex<double>> outD(n);
            for (std::size_t at = 0UZ; at < n; at += chunk) {
                const std::size_t take = std::min(chunk, n - at);
                p.fill(std::span<std::complex<float>>(out).subspan(at, 0UZ));
                p.fill(std::span<std::complex<float>>(out).subspan(at, take));
                pd.fill(std::span<std::complex<double>>(outD).subspan(at, take));
            }
            const double worst  = worstDistance({}, out, inc, -0.7);
            const double worstD = worstDistance({}, outD, inc, -0.7);
            expect(lt(worst, kStreamTolerance<float>)) << std::format("fill in calls of {} (with empty calls between): {:.3e} from the double reference", chunk, worst);
            expect(lt(worstD, kStreamTolerance<double>)) << std::format("double in calls of {}: {:.3e} from the double reference", chunk, worstD);
        }
    };

    // A change of increment moves the rate and never the phase. 1013 samples is not a multiple of a group or of the
    // interval, so the carry is taken part-way through both.
    "setIncrement keeps the phase"_test = [] {
        const double          inc1 = 0.2, inc2 = -0.9;
        constexpr std::size_t head = 1013UZ;
        LanePhasor<float>     p;
        p.configure(inc1, 0.5);
        std::vector<std::complex<float>> a(head);
        p.fill(a);
        const double expected = std::remainder(0.5 + inc1 * static_cast<double>(head), kTau);
        expect(lt(std::abs(std::remainder(p.phase() - expected, kTau)), 1.0e-12)) << "phase() reports where the stream has reached";
        p.setIncrement(inc2);
        expect(lt(std::abs(std::remainder(p.phase() - expected, kTau)), 1.0e-12)) << "setIncrement leaves the phase where it was";
        const Error e = measure(p, inc2, expected, 3000UZ, 3000UZ);
        expect(lt(e.phase, 1.0e-6) && lt(e.mag, 1.0e-6)) << std::format("the samples after a retune continue from that phase at the new rate: phase {:.3e}", e.phase);
        p.setIncrement(inc2);
        expect(lt(std::abs(std::remainder(p.phase() - std::remainder(expected + inc2 * 3000., kTau), kTau)), 1.0e-12)) << "setting the same increment again moves nothing";
    };

    "setPhase restarts the stream at the phase given"_test = [] {
        LanePhasor<float> p;
        p.configure(0.1, 0.);
        std::vector<std::complex<float>> a(37);
        p.fill(a);
        p.setPhase(2.);
        const Error e = measure(p, 0.1, 2., 2000UZ, 500UZ);
        expect(lt(e.phase, 1.0e-6)) << std::format("after setPhase the stream starts at that phase: error {:.3e}", e.phase);
    };

    // A call that ends on a seed boundary leaves the stream there: phase() reports the position, and the next sample is
    // the exact seed, as close to the double reference as F holds (a few units of F's epsilon), not a sample that
    // has drifted over an interval. Cuts one short of the interval, on it and one past it, on the first and the
    // second boundary, for the two intervals the library's users take.
    "a cut on a seed boundary leaves the next sample on the exact rotation"_test = []<typename Interval> {
        constexpr std::size_t R   = Interval::value;
        const double          inc = 0.7317, start = 1.1;
        for (const std::size_t boundary : {R, 2UZ * R}) {
            for (const std::size_t cut : {boundary - 1UZ, boundary, boundary + 1UZ}) {
                LanePhasor<float, R> p;
                p.configure(inc, start);
                std::vector<std::complex<float>> head(cut);
                p.fill(head);
                const double expected = std::remainder(start + inc * static_cast<double>(cut), kTau);
                expect(lt(std::abs(std::remainder(p.phase() - expected, kTau)), 1.0e-12)) << std::format("interval {}: phase() after {} samples", R, cut);
                std::vector<std::complex<float>> next(1UZ);
                p.fill(next);
                const double err = std::abs(std::arg(std::complex<double>(next[0]) * std::polar(1., -expected)));
                // from the boundary the samples are the seed and its lane offsets; just before it they have run the whole interval, and the drift grows with it
                expect(lt(err, cut < boundary ? 2.0e-7 + 2.5e-9 * static_cast<double>(R) : 3.0e-7)) << std::format("interval {}: sample {} is {:.3e} rad from the reference", R, cut, err);
            }
        }
    } | std::tuple<std::integral_constant<std::size_t, 256UZ>, std::integral_constant<std::size_t, 1024UZ>>{};

    // fill is mix of ones, exactly, from the middle of a group as well as from a start.
    "fill equals mix of ones bit for bit"_test = [] {
        const double                           inc = 1.3;
        const std::vector<std::complex<float>> ones(2051UZ, {1.f, 0.f});
        LanePhasor<float>                      a, b;
        a.configure(inc, 0.2);
        b.configure(inc, 0.2);
        std::vector<std::complex<float>> lead(21), x(ones.size()), y(ones.size());
        a.fill(lead);
        b.fill(lead);
        a.fill(x);
        b.mix(ones, y);
        expect(same(x, y)) << "fill equals mix of ones, bit for bit";
    };

    // The interval is a template parameter: a longer one changes the seeds and nothing about the stream's shape.
    "a longer interval gives a stream of the same shape and a larger floor"_test = [] {
        const double              inc = 0.7317;
        LanePhasor<float, 256UZ>  shortSeed;
        LanePhasor<float, 4096UZ> longSeed;
        shortSeed.configure(inc, 0.1);
        longSeed.configure(inc, 0.1);
        const Error eShort = measure(shortSeed, inc, 0.1, 1UZ << 22UZ, 65536UZ);
        const Error eLong  = measure(longSeed, inc, 0.1, 1UZ << 22UZ, 65536UZ);
        expect(lt(eLong.phase, 2.0e-5)) << std::format("4096 samples between seeds: phase error {:.3e}", eLong.phase);
        expect(gt(eLong.phase, eShort.phase)) << std::format("the error grows with the interval: {:.3e} against {:.3e}", eLong.phase, eShort.phase);
    };

    // in place: out may be in
    "mix may write over its input"_test = [] {
        const auto        in  = noise(1237UZ);
        const double      inc = 0.31;
        auto              ref = std::vector<std::complex<float>>(in.size());
        LanePhasor<float> a, b;
        a.configure(inc, 0.4);
        b.configure(inc, 0.4);
        a.mix(in, ref);
        auto inPlace = in;
        b.mix(inPlace, inPlace);
        expect(same(inPlace, ref)) << "mixing in place gives the stream of separate buffers";
    };
};

int main() { /* not needed for UT */ }
