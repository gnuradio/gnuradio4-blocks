#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <span>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/algorithm/math/Atan2.hpp>

using namespace boost::ut;
using gr::math::Atan2Form;

namespace {

template<typename T>
using FormFunction = void (*)(std::span<const T>, std::span<const T>, std::span<T>, T) noexcept;

template<typename T>
struct FormCase {
    const char*     name;
    Atan2Form       form;
    FormFunction<T> run;
    double          boundUlp;
};

template<typename T>
std::vector<FormCase<T>> allForms() {
    return {{"lanes", Atan2Form::Lanes, &gr::math::detail::scaledAtan2Lanes<T>, 2.0}, {"branchless", Atan2Form::Branchless, &gr::math::detail::scaledAtan2Branchless<T>, 4.0}, {"library", Atan2Form::Library, &gr::math::detail::scaledAtan2Library<T>, 2.0}};
}

/// @brief |value - reference| in units of the spacing of T at the reference; zero when both are the same zero.
template<typename T>
double errorInUlp(T value, long double reference) {
    const T rounded = static_cast<T>(reference);
    if (rounded == T(0)) {
        return value == T(0) ? 0.0 : std::numeric_limits<double>::infinity();
    }
    const T spacing = std::nextafter(std::abs(rounded), std::numeric_limits<T>::infinity()) - std::abs(rounded);
    return static_cast<double>(std::fabs(static_cast<long double>(value) - reference) / static_cast<long double>(spacing));
}

/// @brief Every quadrant, the axes and zero with both signs, subnormals, the smallest normal, the largest finite
/// values, the octant boundary and random pairs of mixed magnitude.
template<typename T>
void argumentPairs(std::vector<T>& y, std::vector<T>& x) {
    constexpr T big  = std::numeric_limits<T>::max();
    constexpr T tiny = std::numeric_limits<T>::denorm_min();
    constexpr T low  = std::numeric_limits<T>::min();
    for (const T yv : {T(0), -T(0), T(1), T(-1), tiny, -tiny, low, -low, big, -big, T(0.41421356237309503), T(2.4142135623730951)}) {
        for (const T xv : {T(0), -T(0), T(1), T(-1), tiny, -tiny, low, -low, big, -big}) {
            y.push_back(yv);
            x.push_back(xv);
        }
    }
    std::mt19937_64                        rng(12345);
    std::uniform_real_distribution<double> mantissa(-1.0, 1.0);
    const int                              span = std::numeric_limits<T>::max_exponent - 4;
    for (std::size_t i = 0UZ; i < 200000UZ; ++i) {
        const int ey = i % 4UZ == 0UZ ? static_cast<int>(rng() % static_cast<unsigned>(2 * span)) - span : static_cast<int>(rng() % 5U) - 2;
        const int ex = i % 4UZ == 1UZ ? static_cast<int>(rng() % static_cast<unsigned>(2 * span)) - span : static_cast<int>(rng() % 5U) - 2;
        y.push_back(static_cast<T>(std::ldexp(mantissa(rng), ey)));
        x.push_back(static_cast<T>(std::ldexp(mantissa(rng), ex)));
    }
    for (std::size_t i = 0UZ; i < 20000UZ; ++i) { // ratios across the octant boundary tan(pi/8)
        y.push_back(static_cast<T>(0.41421356237309503 * (1.0 + 1e-3 * mantissa(rng))));
        x.push_back(T(1));
    }
}

} // namespace

const boost::ut::suite<"Atan2"> atan2Tests = [] {
    "each form stays within its bound of a long double reference"_test = []<typename T> {
        std::vector<T> y;
        std::vector<T> x;
        argumentPairs<T>(y, x);
        for (const FormCase<T>& form : allForms<T>()) {
            std::vector<T> out(y.size());
            form.run(y, x, out, T(1));
            double worst = 0.0;
            for (std::size_t i = 0UZ; i < y.size(); ++i) {
                worst = std::max(worst, errorInUlp(out[i], std::atan2(static_cast<long double>(y[i]), static_cast<long double>(x[i]))));
            }
            boost::ut::log << sizeof(T) * 8UZ << "-bit " << form.name << " largest error " << worst << " ulp\n";
            expect(le(worst, form.boundUlp)) << form.name << ", " << sizeof(T) * 8UZ << "-bit samples";
        }
    } | std::tuple<float, double>{};

    "zeros, axes, infinities and NaN follow C's atan2 in every form"_test = []<typename T> {
        constexpr T          inf = std::numeric_limits<T>::infinity();
        constexpr T          nan = std::numeric_limits<T>::quiet_NaN();
        const std::vector<T> values{T(0), -T(0), T(1), T(-1), inf, -inf, nan};
        std::vector<T>       y;
        std::vector<T>       x;
        for (const T yv : values) {
            for (const T xv : values) {
                y.push_back(yv);
                x.push_back(xv);
            }
        }
        for (const FormCase<T>& form : allForms<T>()) {
            std::vector<T> out(y.size());
            form.run(y, x, out, T(1));
            for (std::size_t i = 0UZ; i < y.size(); ++i) {
                const T expected = std::atan2(y[i], x[i]);
                if (std::isnan(expected)) {
                    expect(std::isnan(out[i])) << form.name << " y=" << y[i] << " x=" << x[i];
                } else {
                    expect(le(errorInUlp(out[i], static_cast<long double>(expected)), 1.0)) << form.name << " y=" << y[i] << " x=" << x[i];
                    expect(eq(std::signbit(out[i]), std::signbit(expected))) << form.name << " sign of y=" << y[i] << " x=" << x[i];
                }
            }
        }
    } | std::tuple<float, double>{};

    "the scale multiplies the arctangent"_test = []<typename T> {
        std::vector<T> y;
        std::vector<T> x;
        argumentPairs<T>(y, x);
        std::vector<T> unscaled(y.size());
        std::vector<T> scaled(y.size());
        gr::math::scaledAtan2<T>(y, x, unscaled, T(1));
        gr::math::scaledAtan2<T>(y, x, scaled, T(-1.5));
        std::size_t differing = 0UZ;
        for (std::size_t i = 0UZ; i < y.size(); ++i) {
            differing += (std::isnan(unscaled[i]) ? !std::isnan(scaled[i]) : scaled[i] != T(-1.5) * unscaled[i]) ? 1UZ : 0UZ;
        }
        expect(eq(differing, 0UZ));
    } | std::tuple<float, double>{};

    "a stream cut anywhere gives the values of one call"_test = []<typename T> {
        std::vector<T> y;
        std::vector<T> x;
        argumentPairs<T>(y, x);
        y.resize(5001UZ);
        x.resize(5001UZ);
        for (const FormCase<T>& form : allForms<T>()) {
            std::vector<T> whole(y.size());
            form.run(y, x, whole, T(0.75));
            for (const std::size_t chunk : {1UZ, 7UZ, 256UZ, 257UZ, 1000UZ}) {
                std::vector<T> pieces(y.size());
                for (std::size_t base = 0UZ; base < y.size(); base += chunk) {
                    const std::size_t count = std::min(chunk, y.size() - base);
                    form.run(std::span<const T>(y).subspan(base, count), std::span<const T>(x).subspan(base, count), std::span<T>(pieces).subspan(base, count), T(0.75));
                }
                expect(std::ranges::equal(pieces, whole)) << form.name << ", " << sizeof(T) * 8UZ << "-bit, chunk " << chunk << " must be bit-identical";
            }
        }
    } | std::tuple<float, double>{};

    "the build takes the form its doc names"_test = []<typename T> {
#if defined(__clang__)
        static_assert(gr::math::atan2Form<T>() == Atan2Form::Branchless);
#else
        static_assert(gr::math::atan2Form<T>() != Atan2Form::Branchless);
#endif
    } | std::tuple<float, double>{};

    "spans of unequal length use the shortest"_test = [] {
        const std::vector<float> y(10UZ, 1.0f);
        const std::vector<float> x(4UZ, 1.0f);
        std::vector<float>       out(10UZ, -7.0f);
        gr::math::scaledAtan2<float>(y, x, out, 1.0f);
        for (std::size_t i = 0UZ; i < out.size(); ++i) {
            expect(i < 4UZ ? out[i] != -7.0f : out[i] == -7.0f) << "element " << i;
        }
    };
};

int main() { /* not needed for ut */ }
