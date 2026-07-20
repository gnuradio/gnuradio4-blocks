#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

#include <cassert>
#include <iostream>

#include <gnuradio-4.0/GrBasicBlocks.hpp>

using namespace std::string_literals;

int main() {
    gr_blocklib_init_module_GrBasicBlocks(gr::globalBlockRegistry());

    auto known = gr::globalBlockRegistry().keys();
    std::ranges::sort(known);
    std::vector<std::string> desired{
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

    assert((std::ranges::includes(known, desired)));
    std::cout << "All ok\n";
}
