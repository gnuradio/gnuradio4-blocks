#include <boost/ut.hpp>

// Opening gr::blocks::filter puts the name `filter` in an enclosing scope of gr::blocks::sdr, so an unqualified
// `filter::` in an sdr header binds to the block family rather than to the kernels of gr::filter. Including a
// filter block header before the sdr headers is the test: this translation unit only compiles while the sdr
// headers name every such kernel in full.
#include <gnuradio-4.0/filter/time_domain_filter.hpp>

#include <gnuradio-4.0/sdr/RTL2832Source.hpp>
#include <gnuradio-4.0/sdr/SoapySink.hpp>
#include <gnuradio-4.0/sdr/SoapySource.hpp>

#include <complex>
#include <type_traits>

// No device is opened here: the blocks are named, never constructed.
const boost::ut::suite<"SDR header order"> sdrHeaderOrderTests = [] {
    using namespace boost::ut;

    "both families are usable in one translation unit"_test = [] {
        static_assert(std::is_class_v<gr::blocks::filter::fir_filter<float>>);
        static_assert(std::is_class_v<gr::blocks::sdr::SoapySimpleSource<std::complex<float>>>);
        static_assert(std::is_class_v<gr::blocks::sdr::SoapySimpleSink<std::complex<float>>>);
        static_assert(std::is_class_v<gr::blocks::sdr::RTL2832Source<std::complex<float>>>);
        expect(true);
    };

    "the DC blocker the sdr sources hold is the module's own"_test = [] {
        static_assert(std::is_same_v<decltype(gr::blocks::sdr::SoapySimpleSource<std::complex<float>>::_dcFilterI), gr::blocks::sdr::DcBlocker>);
        static_assert(std::is_same_v<decltype(gr::blocks::sdr::RTL2832Source<std::complex<float>>::_dcFilterQ), gr::blocks::sdr::DcBlocker>);
        expect(true);
    };
};

int main() { /* not needed for UT */ }
