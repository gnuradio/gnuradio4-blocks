#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <iterator>

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

    std::vector<std::string> missing;
    std::ranges::set_difference(desired, known, std::back_inserter(missing));
    if (!missing.empty()) {
        for (const auto& name : missing) {
            std::cerr << "missing block registration: " << name << '\n';
        }
        return EXIT_FAILURE;
    }
    std::cout << "All ok\n";
}
