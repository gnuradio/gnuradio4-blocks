#include <boost/ut.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <expected>
#include <format>
#include <fstream>
#include <future>
#include <map>
#include <memory>
#include <print>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/YamlPmt.hpp>

#include <gnuradio-4.0/basic/DataSink.hpp>

#include <gnuradio-4.0/GrAnalogBlocks.hpp>
#include <gnuradio-4.0/GrBasicBlocks.hpp>
#include <gnuradio-4.0/GrFilterBlocks.hpp>
#include <gnuradio-4.0/GrMathBlocks.hpp>

#include "ExampleGraphs.hpp"

// Loads each example graph through the core's YAML importer, from the text embedded at configure time and, where the
// host file system is readable, from its installed path. Every block entry must be constructed under the id it names.

namespace {

using namespace std::chrono_literals;

constexpr std::string_view kFmRadioMono     = "fm_radio_mono.yaml";
constexpr auto             kDeliveryTimeout = 10s;

/// @brief One block entry of a graph file: the registry id and the block's name.
struct FileBlock {
    std::string id;
    std::string name;
};

/// @brief Registers the analog, basic, filter and math blocks in the global registry.
///
/// Each init call references its library: an as-needed or static link keeps a library only when the program uses a
/// symbol from it. A block registered twice keeps one entry.
void registerGraphFamilies() {
    gr::BlockRegistry& registry = gr::globalBlockRegistry();
    gr::blocklib::initGrAnalogBlocks(registry);
    gr::blocklib::initGrBasicBlocks(registry);
    gr::blocklib::initGrFilterBlocks(registry);
    gr::blocklib::initGrMathBlocks(registry);
}

/// @brief The embedded text of the example graph `fileName`.
[[nodiscard]] std::string_view graphText(std::string_view fileName) {
    const auto found = std::ranges::find(gr::examples::kExampleGraphs, fileName, &gr::examples::ExampleGraph::fileName);
    boost::ut::expect(found != gr::examples::kExampleGraphs.end()) << std::format("{} is not embedded", fileName) << boost::ut::fatal;
    return found->text;
}

#ifdef INSTALLED_EXAMPLE_GRAPHS_PATH
[[nodiscard]] std::string readFile(std::string_view directory, std::string_view fileName) {
    const std::string path = std::format("{}/{}", directory, fileName);
    std::ifstream     file(path, std::ios::binary);
    boost::ut::expect(file.is_open()) << std::format("cannot open {}", path) << boost::ut::fatal;
    std::ostringstream content;
    content << file.rdbuf();
    return content.str();
}
#endif

/// @brief The id and name of every block entry of a graph file, read with the parser the importer uses.
[[nodiscard]] std::vector<FileBlock> fileBlocks(std::string_view text) {
    using namespace boost::ut;
    const auto yaml = gr::pmt::yaml::deserialize(text);
    expect(yaml.has_value()) << "the graph file parses as YAML" << fatal;
    const auto blocksIt = yaml->find("blocks");
    expect(blocksIt != yaml->end()) << "the graph file has a blocks list" << fatal;
    const auto* entries = blocksIt->second.get_if<gr::Tensor<gr::pmt::Value>>();
    expect(entries != nullptr) << "the blocks entry is a list" << fatal;

    std::vector<FileBlock> result;
    for (const gr::pmt::Value& entry : *entries) {
        const auto* block = entry.get_if<gr::property_map>();
        expect(block != nullptr) << "a block entry is a map" << fatal;
        const auto idIt         = block->find("id");
        const auto parametersIt = block->find("parameters");
        expect(idIt != block->end() && parametersIt != block->end()) << "a block entry has an id and parameters" << fatal;
        const auto* parameters = parametersIt->second.get_if<gr::property_map>();
        expect(parameters != nullptr) << "a block's parameters are a map" << fatal;
        const auto nameIt = parameters->find("name");
        expect(nameIt != parameters->end()) << "a block entry has a name" << fatal;
        result.push_back({.id = std::string(idIt->second.value_or(std::string_view{})), .name = std::string(nameIt->second.value_or(std::string_view{}))});
    }
    return result;
}

/// @brief Loads a graph that is checked and never run.
///
/// A DataSink adds its signal_name to the global sink registry when its settings are applied, and removes it when it
/// stops. A sink that is loaded and destroyed without running stays registered under its name, and the next sink that
/// takes the name is refused. Every DataSink of a graph loaded here takes an empty name, which the registry skips.
[[nodiscard]] gr::meta::indirect<gr::Graph> loadForInspection(std::string_view text) {
    gr::BlockSettings overrides;
    for (const FileBlock& entry : fileBlocks(text)) {
        if (entry.id.starts_with("gr::blocks::basic::DataSink<")) {
            overrides.emplace(entry.name, gr::property_map{{"signal_name", std::string{}}});
        }
    }
    return gr::loadGrc(gr::globalPluginLoader(), text, overrides);
}

/// @brief Expects the loaded graph to hold one block per entry of the file, each under the name and id of its entry.
void expectBlocksUnderTheirIds(const gr::Graph& graph, std::string_view text) {
    using namespace boost::ut;
    const std::vector<FileBlock> entries  = fileBlocks(text);
    gr::BlockRegistry&           registry = gr::globalPluginLoader().registry();

    expect(eq(graph.blocks().size(), entries.size())) << "the graph holds one block per entry of the file";
    for (const FileBlock& entry : entries) {
        const auto found = std::ranges::find_if(graph.blocks(), [&entry](const auto& block) { return block->name() == entry.name; });
        expect(found != graph.blocks().end()) << std::format("no block is named '{}'", entry.name);
        if (found != graph.blocks().end()) {
            expect(eq(registry.typeName(*found), entry.id)) << std::format("block '{}'", entry.name);
        }
    }
}

/// @brief Runs the scheduler on its own thread until `collect` reports enough or the timeout passes, then stops it.
[[nodiscard]] bool runUntil(gr::scheduler::Simple<>& scheduler, auto collect) {
    auto       runner   = std::async(std::launch::async, [&scheduler] { return scheduler.runAndWait(); });
    const auto deadline = std::chrono::steady_clock::now() + kDeliveryTimeout;

    bool enough = false;
    while (!enough && std::chrono::steady_clock::now() < deadline) {
        enough = collect();
        if (!enough) {
            std::this_thread::sleep_for(2ms);
        }
    }

    scheduler.requestStop();
    const auto finished = runner.get();
    boost::ut::expect(finished.has_value()) << (finished.has_value() ? std::string{} : finished.error().message);
    return enough;
}

/// @brief A streaming poller on the float DataSink named `name`, taken from the loaded block itself.
///
/// The global sink registry finds a sink by its signal name and casts the stored pointer to the requested sink type.
/// The cast compares type information. A loaded graph's sink is built in the basic block library. Under libc++ that
/// library's type information for the sink differs from the program's, and the registry returns no poller. The block's
/// registry id confirms its type before the cast.
[[nodiscard]] std::shared_ptr<gr::blocks::basic::StreamingPoller<float>> attachToSink(gr::Graph& graph, std::string_view name, std::string_view signalName) {
    using namespace boost::ut;
    using gr::blocks::basic::DataSink;
    const auto found = std::ranges::find_if(graph.blocks(), [name](const auto& block) { return block->name() == name; });
    expect(found != graph.blocks().end()) << std::format("no block is named '{}'", name) << fatal;
    expect(eq(gr::globalPluginLoader().registry().typeName(*found), std::string_view{"gr::blocks::basic::DataSink<float32>"})) << std::format("block '{}'", name) << fatal;

    auto* sink = static_cast<DataSink<float>*>((*found)->raw());
    expect(eq(std::string_view{sink->signal_name.value}, signalName)) << std::format("the signal name of '{}'", name);
    return sink->getStreamingPoller({.overflowPolicy = gr::blocks::basic::OverflowPolicy::Drop});
}

} // namespace

const boost::ut::suite<"example graphs"> exampleGraphTests = [] {
    using namespace boost::ut;

    registerGraphFamilies();

    "every example graph loads with every block under its id"_test = [] {
        expect(std::ranges::contains(gr::examples::kExampleGraphs, kFmRadioMono, &gr::examples::ExampleGraph::fileName)) << "the graphs are embedded";
        for (const auto& [fileName, text] : gr::examples::kExampleGraphs) {
            std::println("loading {}", fileName);
            auto graph = loadForInspection(text);
            expectBlocksUnderTheirIds(*graph, text);
        }
    };

#ifdef INSTALLED_EXAMPLE_GRAPHS_PATH
    // ctest installs the files into a prefix inside the build tree before this binary runs
    "every example graph in the install layout is the source file and loads with every block under its id"_test = [] {
        for (const auto& [fileName, text] : gr::examples::kExampleGraphs) {
            const std::string installed = readFile(INSTALLED_EXAMPLE_GRAPHS_PATH, fileName);
            expect(eq(std::string_view{installed}, text)) << std::format("the installed {} is the source file", fileName);
            auto graph = loadForInspection(installed);
            expectBlocksUnderTheirIds(*graph, installed);
        }
    };
#endif

    "a copy naming a block type the registry does not hold fails to load"_test = [] {
        constexpr std::string_view kRegistered   = "gr::blocks::analog::QuadratureDemod<float32>";
        constexpr std::string_view kUnregistered = "gr::blocks::analog::NoSuchDemod<float32>";
        expect(!gr::globalPluginLoader().isBlockAvailable(kUnregistered)) << fatal;

        std::string text{graphText(kFmRadioMono)};
        const auto  at = text.find(kRegistered);
        expect(at != std::string::npos) << "the file names the discriminator's type" << fatal;
        text.replace(at, kRegistered.size(), kUnregistered);

        std::string refusal;
        try {
            auto graph = loadForInspection(text);
        } catch (const std::exception& error) {
            refusal = error.what();
        }
        expect(!refusal.empty()) << "the copy loaded" << fatal;
        expect(refusal.contains(kUnregistered)) << std::format("the refusal names the type: {}", refusal);
    };

    "fm_radio_mono.yaml runs headless and hands back demodulated audio"_test = [] {
        // The 15 kHz tone is a fifth of the 75 kHz deviation the discriminator gain is scaled to.
        // The audio settles at 0.2.
        constexpr std::size_t kWanted   = 8192UZ;
        constexpr std::size_t kSettled  = 4096UZ; // the tail, past the filters' start-up transients
        constexpr double      kExpected = 0.2;

        auto       graph  = gr::loadGrc(gr::globalPluginLoader(), graphText(kFmRadioMono));
        const auto poller = attachToSink(*graph, "audio_sink", "fm_audio");
        expect(poller != nullptr) << fatal;

        gr::scheduler::Simple<> scheduler;
        expect(scheduler.exchange(std::move(*graph)).has_value()) << fatal;

        std::vector<float> audio;
        const bool         delivered = runUntil(scheduler, [&poller, &audio] {
            while (poller->process([&audio](const auto& data) { audio.insert(audio.end(), data.begin(), data.end()); })) {
            }
            return audio.size() >= kWanted;
        });
        expect(delivered) << std::format("audio samples delivered within the timeout: {}", audio.size()) << fatal;

        double sum = 0.0;
        for (const float sample : std::span<const float>(audio).last(kSettled)) {
            sum += static_cast<double>(sample);
        }
        const double level = sum / static_cast<double>(kSettled);
        expect(lt(std::abs(level - kExpected), 0.02 * kExpected)) << std::format("audio settled at {:.5f}, expected {:.5f}", level, kExpected);
    };
};

// runs the suites before exit; under Emscripten the global block registry is destroyed by then
int main() { return boost::ut::cfg<boost::ut::override>.run(); }
