#include <boost/ut.hpp>

#include <gnuradio-4.0/basic/DataSink.hpp>
#include <gnuradio-4.0/basic/FunctionGenerator.hpp>
#include <gnuradio-4.0/basic/Selector.hpp>
#include <gnuradio-4.0/basic/SignalGenerator.hpp>

#include <gnuradio-4.0/GrBasicBlocks.hpp>

const boost::ut::suite AvailableBlockTests = [] {
    using namespace boost::ut;
    using namespace std::string_literals;

    gr::blocklib::initGrBasicBlocks(gr::globalBlockRegistry());

    "Registered"_test = [] {
        auto known = gr::globalBlockRegistry().keys();
        std::ranges::sort(known);
        std::vector<std::string> desired{
            //
            "gr::blocks::basic::DataSink<float32>"s,                //
            "gr::blocks::basic::DataSetSink<float32>"s,             //
            "gr::blocks::basic::FunctionGenerator<int16>"s,         //
            "gr::blocks::basic::FunctionGenerator<float32>"s,       //
            "gr::blocks::basic::Selector<int32>"s,                  //
            "gr::blocks::basic::Selector<float32>"s,                //
            "gr::blocks::basic::SignalGenerator<float32>"s,         //
            "gr::blocks::basic::SignalGenerator<complex<float32>>"s //
        };
        std::ranges::sort(desired);

        expect((std::ranges::includes(known, desired)));
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
