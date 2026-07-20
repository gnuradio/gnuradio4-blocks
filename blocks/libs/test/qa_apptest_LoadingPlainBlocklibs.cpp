#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

#include <cassert>
#include <print>
#include <string>

using namespace std::literals;

// This tests automatic loading of .so files that are not
// plugins, but ordinary dynamic block libraries -- they
// just need to be in a path that the PluginLoader searches in.
//
// This is intentionally not a ut test as it tests
// how a normal application would use the global block
// registry in the lifetime of main

int main() {
    gr::globalPluginLoader();

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

    assert((std::ranges::includes(known, desired)));
    std::println("All ok");
}
