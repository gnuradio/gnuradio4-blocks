#include <algorithm>
#include <string_view>

#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/GrBasicBlocks.hpp>

int main() {
    auto& registry = gr::globalBlockRegistry();
    gr::blocklib::initGrBasicBlocks(registry);
    const auto known = registry.keys();
    return std::ranges::find(known, std::string_view{"gr::blocks::basic::SignalGenerator<float32>"}) != known.end() ? 0 : 1;
}
