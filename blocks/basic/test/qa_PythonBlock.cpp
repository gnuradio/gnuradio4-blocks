// Python related includes need to be first. This unit holds the program's NumPy API table and interpreter runtime.
#define GR_PYTHON_RUNTIME_OWNER
#include <gnuradio-4.0/basic/PythonBlock.hpp>

#include <boost/ut.hpp>

#include <gnuradio-4.0/GrBasicBlocks.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <thread>

namespace {
/// runs five samples through a Python block the block library's registration makes
template<typename T>
std::vector<T> runLibraryBlock(std::string_view typeName, const std::string& pythonScript) {
    using namespace boost::ut;
    using namespace gr::blocks::testing;
    gr::Graph                       graph;
    auto&                           src    = graph.emplaceBlock<TagSource<T>>({{"n_samples_max", 5U}, {"mark_tag", false}});
    auto&                           sink   = graph.emplaceBlock<TagSink<T, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_expected", 5U}});
    std::shared_ptr<gr::BlockModel> python = gr::globalBlockRegistry().create(typeName, {{"n_inputs", 1U}, {"n_outputs", 1U}, {"pythonScript", pythonScript}});
    expect(python != nullptr) << std::format("the block library registers {}", typeName);
    if (python == nullptr) {
        return {};
    }
    graph.addBlock(python);
    expect(graph.connect(gr::graph::findBlock(graph, src).value(), "out", python, "inputs#0").has_value());
    expect(graph.connect(python, "outputs#0", gr::graph::findBlock(graph, sink).value(), "in").has_value());

    gr::scheduler::Simple sched;
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
    }
    expect(sched.runAndWait().has_value());
    return {sink._samples.begin(), sink._samples.end()};
}

/// makes the block library, a ramp source and a collecting sink loadable from a graph file
void registerGraphFileBlocks() {
    using namespace gr::blocks::testing;
    static const bool registered = [] {
        gr::BlockRegistry& registry = gr::globalBlockRegistry();
        gr::blocklib::initGrBasicBlocks(registry);
        return registry.insert<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>("=qa::RampSource") && registry.insert<TagSink<float, ProcessFunction::USE_PROCESS_BULK>>("=qa::CollectingSink");
    }();
    boost::ut::expect(registered) << "the test blocks reach the global registry";
}

/// the counts a FactorSink shares with the test thread
struct FactorProbe {
    static constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();

    std::atomic<std::size_t> nSamples{0UZ};
    std::atomic<std::size_t> firstScaled{kNone}; ///< index of the first sample that is its index times '_after'
    std::atomic<std::size_t> nUnexpected{0UZ};   ///< samples that fit neither factor in force
};

/// Checks a ramp scaled by one factor and then by another: sample i is i * _before up to the first sample that is
/// i * _after, and every sample from there on is i * _after.
template<typename T>
struct FactorSink : gr::Block<FactorSink<T>> {
    gr::PortIn<T> in;

    GR_MAKE_REFLECTABLE(FactorSink, in);

    std::shared_ptr<FactorProbe> _probe = std::make_shared<FactorProbe>();
    T                            _before{2};
    T                            _after{5};

    gr::work::Status processBulk(gr::InputSpanLike auto& input) {
        FactorProbe& probe = *_probe;
        std::size_t  index = probe.nSamples.load(std::memory_order_relaxed);
        for (const T sample : input) {
            const T    ramp   = static_cast<T>(index);
            const bool scaled = probe.firstScaled.load(std::memory_order_relaxed) != FactorProbe::kNone;
            if (!scaled && sample == ramp * _before) {
                // the first factor is still in force
            } else if (!scaled && sample == ramp * _after) {
                probe.firstScaled.store(index, std::memory_order_relaxed);
            } else if (!scaled || sample != ramp * _after) {
                probe.nUnexpected.fetch_add(1UZ, std::memory_order_relaxed);
            }
            ++index;
        }
        probe.nSamples.store(index, std::memory_order_release);
        return gr::work::Status::OK;
    }
};

/// waits until 'condition' holds; the time limit ends only a run that has already failed
template<typename Condition>
bool awaitCondition(Condition condition) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!condition()) {
        if (std::chrono::steady_clock::now() > deadline) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return true;
}

/// runs 'in' once through a block with one input and one output
std::vector<float> runOnce(gr::blocks::basic::PythonBlock<float>& block, std::vector<float> in) {
    std::vector<float>                  out(in.size());
    std::vector<std::span<const float>> ins{in};
    std::vector<std::span<float>>       outs{out};
    boost::ut::expect(block.processBulk(std::span(ins), std::span(outs)) == gr::work::Status::OK);
    return out;
}
} // namespace

const boost::ut::suite<"python::<C-API abstraction interfaces>"> pythonInterfaceTests = [] {
    using namespace boost::ut;
    using namespace gr::python;

    "numpyType<T>()"_test = [] {
        expect(numpyType<bool>() == NPY_BOOL);
        expect(numpyType<int8_t>() == NPY_BYTE);
        expect(numpyType<uint8_t>() == NPY_UBYTE);
        expect(numpyType<int16_t>() == NPY_SHORT);
        expect(numpyType<uint16_t>() == NPY_USHORT);
        expect(numpyType<int32_t>() == NPY_INT);
        expect(numpyType<uint32_t>() == NPY_UINT);
        expect(numpyType<int64_t>() == NPY_LONG);
        expect(numpyType<uint64_t>() == NPY_ULONG);
        expect(numpyType<float>() == NPY_FLOAT);
        expect(numpyType<double>() == NPY_DOUBLE);
        expect(numpyType<std::complex<float>>() == NPY_CFLOAT);
        expect(numpyType<std::complex<double>>() == NPY_CDOUBLE);
        expect(numpyType<char*>() == NPY_STRING);
        expect(numpyType<const char*>() == NPY_STRING);
        expect(numpyType<void>() == NPY_NOTYPE);
    };
};

const boost::ut::suite<"PythonBlock"> pythonBlockTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::basic;
    using namespace std::string_literals;
    using namespace std::string_view_literals;

    static_assert(gr::HasRequiredProcessFunction<gr::blocks::basic::PythonBlock<std::int32_t>>);
    static_assert(gr::HasProcessBulkFunction<gr::blocks::basic::PythonBlock<std::int32_t>>);
    static_assert(gr::HasRequiredProcessFunction<gr::blocks::basic::PythonBlock<float>>);
    static_assert(gr::HasProcessBulkFunction<gr::blocks::basic::PythonBlock<float>>);

    "nominal PoC"_test = [] {
        // Your Python script
        std::string pythonScript = R"(import time;
counter = 0

def process_bulk(ins, outs):
    global counter
    start = time.time()
    print('Start Python processing iteration: {}'.format(counter))
    # Print current settings
    settings = this_block.getSettings()
    print("Current settings:", settings)

    # tag handling
    if this_block.tagAvailable():
        tag = this_block.getTag()
        print('Tag:', tag)

    counter += 1
    # process the input->output samples
    for i in range(len(ins)):
        outs[i][:] = ins[i] * 2

    # Update settings with the counter
    settings["counter"] = str(counter)
    this_block.setSettings(settings)

    print('Stop Python processing - time: {} seconds'.format(time.time() - start))
)";

        PythonBlock<std::int32_t> myBlock({{"n_inputs", 3U}, {"n_outputs", 3U}, {"pythonScript", pythonScript}});
        myBlock.init(myBlock.progress); // needed for unit-test only when executed outside a Scheduler/Graph

        int                                        count = 0;
        std::vector<std::int32_t>                  data1 = {1, 2, 3};
        std::vector<std::int32_t>                  data2 = {4, 5, 6};
        std::vector<std::int32_t>                  out1(3);
        std::vector<std::int32_t>                  out2(3);
        std::vector<std::span<std::int32_t>>       outs    = {out1, out2};
        std::vector<std::span<const std::int32_t>> ins     = {data1, data2};
        std::span<std::span<const std::int32_t>>   spanIns = ins;
        for (const auto& span : ins) {
            std::println("InPort[{}] : [{}]", count++, gr::join(span, ", "));
        }
        std::println("");

        for (std::size_t i = 0; i < 3; i++) {
            std::println("C++ processing iteration: {}", i);
            std::vector<std::span<const std::int32_t>> constOuts(outs.begin(), outs.end());
            std::span<std::span<const std::int32_t>>   constSpanOuts = constOuts;
            std::span<std::span<std::int32_t>>         spanOuts      = outs;

            try {
                if (i == 0) {
                    myBlock.processBulk(spanIns, spanOuts);
                } else {
                    myBlock.processBulk(constSpanOuts, spanOuts);
                }
            } catch (const std::exception& ex) {
                std::println(stderr, "myBlock.processBulk(...) - threw unexpected exception:\n {}", ex.what());
                expect(false) << "nominal example should not throw";
            }

            std::println("C++ side got:");
            std::println("settings: {}", myBlock._settingsMap);
            for (const auto& span : outs) {
                std::println("OutPort[{}] : [{}]", count++, gr::join(span, ", "));
            }
            std::println("");
        }

        expect(eq(outs[0][0], 8)) << "out1[0] should be 8";
        expect(eq(outs[0][1], 16)) << "out1[1] should be 16";
        expect(eq(outs[0][2], 24)) << "out1[2] should be 24";

        expect(eq(outs[1][0], 32)) << "out2[0] should be 32";
        expect(eq(outs[1][1], 40)) << "out2[1] should be 40";
        expect(eq(outs[1][2], 48)) << "out2[2] should be 48";

        expect(eq(myBlock.getSettings().at("counter"), "3"s));
    };

    "Python SyntaxError"_test = [] {
        // Your Python script
        std::string pythonScript = R"(def process_bulk(ins, outs):

    # process the input->output samples
    for i in range(len(ins))     # <- (N.B. missing ':')
        outs[i][:] = ins[i] * 2
)";

        PythonBlock<std::int32_t> myBlock({{"n_inputs", 3U}, {"n_outputs", 3U}, {"pythonScript", pythonScript}});

        bool throws = false;
        try {
            myBlock.settings().init();
            std::ignore = myBlock.settings().applyStagedParameters(); // needed for unit-test only when executed outside a Scheduler/Graph
        } catch (const std::exception& ex) {
            throws = true;
            std::println("myBlock.processBulk(...) - correctly threw SyntaxError exception:\n {}", ex.what());
        }
        expect(throws) << "SyntaxError should throw";
    };

    "Python RuntimeWarning as exception"_test = [] {
        // Your Python script
        std::string pythonScript = R"(def process_bulk(ins, outs):

    # process the input->output samples
    for i in range(len(ins)):
        outs[i][:] = ins[i] * 2/0 # <- (N.B. division by zero)
)";

        PythonBlock<float> myBlock({{"n_inputs", 3U}, {"n_outputs", 3U}, {"pythonScript", pythonScript}});
        myBlock.init(myBlock.progress); // needed for unit-test only when executed outside a Scheduler/Graph

        std::vector<float>                  data1 = {1, 2, 3};
        std::vector<float>                  data2 = {4, 5, 6};
        std::vector<float>                  out1(3);
        std::vector<float>                  out2(3);
        std::vector<std::span<float>>       outs = {out1, out2};
        std::vector<std::span<const float>> ins  = {data1, data2};

        bool throws = false;
        try {
            myBlock.processBulk(std::span(ins), std::span(outs));
        } catch (const std::exception& ex) {
            throws = true;
            std::println("myBlock.processBulk(...) - correctly threw RuntimeWarning as exception:\n {}", ex.what());
        }
        expect(throws) << "RuntimeWarning should throw";
    };

    "Python Execution via Scheduler/Graph"_test = [] {
        std::string pythonScript = R"(def process_bulk(ins, outs):

    # process the input->output samples
    for i in range(len(ins)):
        outs[i][:] = ins[i] * 2
)";

        using namespace gr::blocks::testing;
        Graph graph;
        auto& src   = graph.emplaceBlock<TagSource<int32_t>>({{"n_samples_max", 5U}, {"mark_tag", false}});
        auto& block = graph.emplaceBlock<PythonBlock<int32_t>>({{"n_inputs", 1U}, {"n_outputs", 1U}, {"pythonScript", pythonScript}});
        auto& sink  = graph.emplaceBlock<TagSink<int32_t, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_expected", 5U}, {"verbose_console", true}});

        expect(graph.connect(src, "out", block, "inputs#0").has_value());
        expect(graph.connect(block, "outputs#0", sink, "in").has_value());

        gr::scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }

        bool throws = false;
        try {
            expect(sched.runAndWait().has_value());
        } catch (const std::exception& ex) {
            throws = true;
            std::println("sched.runAndWait() unexpectedly threw an exception:\n {}", ex.what());
        }
        expect(!throws);

        expect(eq(sink._nSamplesProduced, 5U)) << "sinkOne did not consume enough input samples";
        expect(eq(sink._samples, std::vector<std::int32_t>{0, 2, 4, 6, 8})) << std::format("mismatch of vector {}", sink._samples);
    };

    "Python Execution - Lifecycle method tests"_test = [] {
        std::string pythonScript = R"x(import os
counter = 0

# optional life-cycle methods - can be used to inform the block of the scheduling state
def start():
    global counter
    print("Python: invoked start")
    counter += 1

def stop():
    global counter
    print("Python: invoked stop")
    counter += 1

def pause():
    global counter
    counter += 1

def resume():
    global counter
    counter += 1

def reset():
    global counter
    counter += 1

# stream-based processing
def process_bulk(ins, outs):
    global counter
    assert counter == 4, "Counter is not equal to 4 (N.B. having called start(), pause(), resume(), reset() callback functions"

    print("Python: invoked process_bulk(..)")
    # process the input->output samples
    for i in range(len(ins)):
        outs[i][:] = ins[i] * 2
)x";

        using namespace gr::blocks::testing;
        Graph graph;
        auto& src   = graph.emplaceBlock<TagSource<float>>({{"n_samples_max", 5U}, {"mark_tag", false}});
        auto& block = graph.emplaceBlock<PythonBlock<float>>({{"n_inputs", 1U}, {"n_outputs", 1U}, {"pythonScript", pythonScript}});
        auto& sink  = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_expected", 5U}, {"verbose_console", true}});

        expect(graph.connect(src, "out", block, "inputs#0").has_value());
        expect(graph.connect(block, "outputs#0", sink, "in").has_value());

        gr::scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }

        block.pause();  // simplified calling
        block.resume(); // simplified calling
        block.reset();  // simplified calling
        bool throws = false;
        try {
            expect(sched.runAndWait().has_value());
        } catch (const std::exception& ex) {
            throws = true;
            std::println("sched.runAndWait() unexpectedly threw an exception:\n {}", ex.what());
        }
        expect(!throws);

        expect(eq(sink._nSamplesProduced, 5U)) << "sinkOne did not consume enough input samples";
        expect(eq(sink._samples, std::vector<float>{0.f, 2.f, 4.f, 6.f, 8.f})) << std::format("mismatch of vector {}", sink._samples);
    };

    "two blocks keep their own script names"_test = [] {
        // both scripts define 'process', 'process_bulk' and the same global; each block must call its own
        std::string doubling = R"(tag = "doubling"
def process(x):
    return x * 2

def process_bulk(ins, outs):
    this_block.setSettings({"ran": tag})
    for i in range(len(ins)):
        outs[i][:] = process(ins[i])
)";
        std::string offset   = R"(tag = "offset"
def process(x):
    return x + 100

def process_bulk(ins, outs):
    this_block.setSettings({"ran": tag})
    for i in range(len(ins)):
        outs[i][:] = process(ins[i])
)";

        using namespace gr::blocks::testing;
        Graph graph;
        auto& srcA   = graph.emplaceBlock<TagSource<int32_t>>({{"n_samples_max", 5U}, {"mark_tag", false}});
        auto& blockA = graph.emplaceBlock<PythonBlock<int32_t>>({{"n_inputs", 1U}, {"n_outputs", 1U}, {"pythonScript", doubling}});
        auto& sinkA  = graph.emplaceBlock<TagSink<int32_t, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_expected", 5U}});
        auto& srcB   = graph.emplaceBlock<TagSource<int32_t>>({{"n_samples_max", 5U}, {"mark_tag", false}});
        auto& blockB = graph.emplaceBlock<PythonBlock<int32_t>>({{"n_inputs", 1U}, {"n_outputs", 1U}, {"pythonScript", offset}});
        auto& sinkB  = graph.emplaceBlock<TagSink<int32_t, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_expected", 5U}});

        expect(graph.connect(srcA, "out", blockA, "inputs#0").has_value());
        expect(graph.connect(blockA, "outputs#0", sinkA, "in").has_value());
        expect(graph.connect(srcB, "out", blockB, "inputs#0").has_value());
        expect(graph.connect(blockB, "outputs#0", sinkB, "in").has_value());

        gr::scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sched.runAndWait().has_value());

        expect(eq(sinkA._samples, std::vector<std::int32_t>{0, 2, 4, 6, 8})) << std::format("first block: {}", sinkA._samples);
        expect(eq(sinkB._samples, std::vector<std::int32_t>{100, 101, 102, 103, 104})) << std::format("second block: {}", sinkB._samples);
        expect(eq(blockA.getSettings().at("ran"), "doubling"s)) << "the first script's 'this_block' names the first block";
        expect(eq(blockB.getSettings().at("ran"), "offset"s)) << "the second script's 'this_block' names the second block";
    };

    "Python block on a worker thread"_test = [] {
        // the multi-threaded scheduler calls process_bulk from its pool threads, and each call takes the interpreter lock
        std::string pythonScript = R"(def process_bulk(ins, outs):
    for i in range(len(ins)):
        outs[i][:] = ins[i] * 2
)";

        using namespace gr::blocks::testing;
        Graph graph;
        auto& src   = graph.emplaceBlock<TagSource<int32_t>>({{"n_samples_max", 5U}, {"mark_tag", false}});
        auto& block = graph.emplaceBlock<PythonBlock<int32_t>>({{"n_inputs", 1U}, {"n_outputs", 1U}, {"pythonScript", pythonScript}});
        auto& sink  = graph.emplaceBlock<TagSink<int32_t, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_expected", 5U}});

        expect(graph.connect(src, "out", block, "inputs#0").has_value());
        expect(graph.connect(block, "outputs#0", sink, "in").has_value());

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        if (auto ret = sched.exchange(std::move(graph)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sched.runAndWait().has_value());

        expect(eq(sink._samples, std::vector<std::int32_t>{0, 2, 4, 6, 8})) << std::format("mismatch of vector {}", sink._samples);
    };

    "a block made after another is destroyed"_test = [] {
        // NumPy refuses a second import into a process whose interpreter was finalized once
        std::string pythonScript = R"(import numpy as np

def process_bulk(ins, outs):
    for i in range(len(ins)):
        outs[i][:] = np.multiply(ins[i], 3)
)";

        auto makeAndRun = [&pythonScript] {
            PythonBlock<float> block({{"n_inputs", 1U}, {"n_outputs", 1U}, {"pythonScript", pythonScript}});
            block.init(block.progress); // needed for unit-test only when executed outside a Scheduler/Graph
            std::vector<float>                  in{1.f, 2.f, 3.f};
            std::vector<float>                  out(3UZ);
            std::vector<std::span<const float>> ins{in};
            std::vector<std::span<float>>       outs{out};
            expect(block.processBulk(std::span(ins), std::span(outs)) == gr::work::Status::OK);
            return out;
        };

        expect(eq(makeAndRun(), std::vector<float>{3.f, 6.f, 9.f}));
        expect(Py_IsInitialized() != 0) << "destroying the only block leaves the interpreter initialized";
        expect(eq(makeAndRun(), std::vector<float>{3.f, 6.f, 9.f})) << "a block made after the first one is destroyed";
    };

    "blocks of two value types from the block library"_test = [] {
        // the library compiles each value type in a unit of its own, and the int32 block runs before the float block
        gr::blocklib::initGrBasicBlocks(gr::globalBlockRegistry());
        const std::string pythonScript = R"(def process_bulk(ins, outs):
    for i in range(len(ins)):
        outs[i][:] = ins[i] * 2
)";
        expect(eq(runLibraryBlock<std::int32_t>("gr::blocks::basic::PythonBlock<int32>", pythonScript), std::vector<std::int32_t>{0, 2, 4, 6, 8}));
        expect(eq(runLibraryBlock<float>("gr::blocks::basic::PythonBlock<float32>", pythonScript), std::vector<float>{0.f, 2.f, 4.f, 6.f, 8.f}));
    };

    "script parameters from a graph file survive a save and a load"_test = [] {
        registerGraphFileBlocks();
        gr::PluginLoader& loader = gr::globalPluginLoader();

        // the script asserts the Python type of each parameter
        const std::string document = R"yaml(blocks:
  - id: qa::RampSource
    parameters:
      name: src
      n_samples_max: 5
      mark_tag: false
  - id: gr::blocks::basic::PythonBlock<float32>
    parameters:
      name: py
      n_inputs: 1
      n_outputs: 1
      script_parameters: {factor: 3, label: x, on: true}
      pythonScript: |
        def process_bulk(ins, outs, factor, label, on):
            assert type(factor) is int and type(label) is str and on is True, (factor, label, on)
            for i in range(len(ins)):
                outs[i][:] = ins[i] * factor
  - id: qa::CollectingSink
    parameters:
      name: sink
      n_samples_expected: 5
connections:
  - [src, 0, py, [0, 0]]
  - [py, [0, 0], sink, 0]
)yaml";

        auto findBlock = [](const gr::Graph& graph, std::string_view name) -> gr::BlockModel* {
            for (const auto& block : graph.blocks()) {
                if (block->name() == name) {
                    return block.get();
                }
            }
            return nullptr;
        };
        auto runGraph = [&findBlock](gr::meta::indirect<gr::Graph> graph) -> std::vector<float> {
            gr::BlockModel* sink = findBlock(*graph, "sink");
            if (sink == nullptr) {
                return {};
            }
            gr::scheduler::Simple sched;
            if (!sched.exchange(std::move(graph)).has_value()) {
                return {};
            }
            if (auto ran = sched.runAndWait(); !ran) {
                std::println(stderr, "graph run failed: {}", ran.error().message);
                return {};
            }
            const auto& samples = static_cast<gr::blocks::testing::TagSink<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>*>(sink->raw())->_samples;
            return {samples.begin(), samples.end()};
        };
        const gr::property_map expected{{"factor", std::int64_t{3}}, {"label", std::string("x")}, {"on", true}};

        auto              loaded = gr::loadGrc(loader, document);
        const std::string saved  = gr::saveGrc(loader, *loaded);
        expect(eq(runGraph(std::move(loaded)), std::vector<float>{0.f, 3.f, 6.f, 9.f, 12.f})) << "the graph file's parameters reach process_bulk";

        auto            reloaded = gr::loadGrc(loader, saved);
        gr::BlockModel* python   = findBlock(*reloaded, "py");
        expect(python != nullptr) << saved;
        if (python != nullptr) {
            const auto parameters = python->settings().get("script_parameters");
            expect(parameters.has_value() && *parameters == gr::pmt::Value(expected)) << std::format("the saved graph keeps each parameter and its type:\n{}", saved);
        }
        expect(eq(runGraph(std::move(reloaded)), std::vector<float>{0.f, 3.f, 6.f, 9.f, 12.f})) << "the saved graph runs alike";
    };

    "a settings message changes the parameters of a running script"_test = [] {
        // the script counts its loads and its calls; a change of the map alone does not run the script again
        std::string pythonScript = R"(loads = globals().get("loads", 0) + 1
calls = 0
calls_at_change = 0

def process_bulk(ins, outs, factor):
    global calls, calls_at_change
    calls += 1
    if factor == 5 and calls_at_change == 0:
        calls_at_change = calls
    this_block.setSettings({"loads": str(loads), "calls": str(calls), "calls_at_change": str(calls_at_change)})
    for i in range(len(ins)):
        outs[i][:] = ins[i] * factor
)";

        using namespace gr::blocks::testing;
        using enum gr::lifecycle::State;
        Graph graph;
        auto& src   = graph.emplaceBlock<TagSource<float>>({{"n_samples_max", 0U}, {"mark_tag", false}});
        auto& block = graph.emplaceBlock<PythonBlock<float>>({{"n_inputs", 1U}, {"n_outputs", 1U}, {"pythonScript", pythonScript}, {"script_parameters", gr::property_map{{"factor", 2}}}});
        auto& sink  = graph.emplaceBlock<FactorSink<float>>();
        expect(graph.connect(src, "out", block, "inputs#0").has_value());
        expect(graph.connect(block, "outputs#0", sink, "in").has_value());
        const std::shared_ptr<FactorProbe> probe = sink._probe;

        gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded> sched;
        expect(sched.exchange(std::move(graph)).has_value());
        gr::MsgPortOut toScheduler;
        expect(toScheduler.connect(sched.msgIn).has_value());
        expect(sched.changeStateTo(INITIALISED).has_value());
        expect(sched.changeStateTo(RUNNING).has_value());

        expect(awaitCondition([&probe] { return probe->nSamples.load(std::memory_order_acquire) > 0UZ; })) << "the graph runs";
        gr::sendMessage<gr::message::Command::Set>(toScheduler, block.unique_name, gr::block::property::kSetting, {{"script_parameters", gr::property_map{{"factor", 5}}}});
        const bool changed = awaitCondition([&probe] { return probe->firstScaled.load(std::memory_order_relaxed) != FactorProbe::kNone; });
        expect(changed) << "the new factor reaches the output";
        if (changed) {
            // one call produces at most one buffer of samples, so these samples include later calls
            const std::size_t later = probe->firstScaled.load(std::memory_order_relaxed) + 4UZ * gr::graph::defaultMinBufferSize(true);
            expect(awaitCondition([&probe, later] { return probe->nSamples.load(std::memory_order_acquire) > later; })) << "the graph runs on after the change";
        }
        expect(sched.changeStateTo(REQUESTED_STOP).has_value());
        expect(awaitCondition([&sched] { return sched.state() == STOPPED; })) << "the graph stops";

        expect(eq(probe->nUnexpected.load(), 0UZ)) << "each sample is its index times 2 before the change and times 5 after it";
        const auto& settings = block.getSettings();
        auto        count    = [&settings](const std::string& key) { return settings.contains(key) ? std::stoul(settings.at(key)) : 0UL; };
        expect(eq(count("loads"), 1UL)) << "the script ran once";
        expect(gt(count("calls_at_change"), 1UL)) << "the script was called before the change";
        expect(gt(count("calls"), count("calls_at_change"))) << "the call counter kept rising after the change";
    };

    "a tag changes the script parameters at its sample"_test = [] {
        // a tag updates a setting the block was not constructed with, so the factor starts as the script's default
        std::string pythonScript = R"(def process_bulk(ins, outs, factor=2):
    for i in range(len(ins)):
        outs[i][:] = ins[i] * factor
)";

        using namespace gr::blocks::testing;
        Graph graph;
        auto& src   = graph.emplaceBlock<TagSource<float>>({{"n_samples_max", 100U}, {"mark_tag", false}});
        auto& block = graph.emplaceBlock<PythonBlock<float>>({{"n_inputs", 1U}, {"n_outputs", 1U}, {"pythonScript", pythonScript}});
        auto& sink  = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_expected", 100U}});
        src._tags   = {gr::Tag{50UZ, {{"script_parameters", gr::property_map{{"factor", 5}}}}}};
        expect(graph.connect(src, "out", block, "inputs#0").has_value());
        expect(graph.connect(block, "outputs#0", sink, "in").has_value());

        gr::scheduler::Simple sched;
        expect(sched.exchange(std::move(graph)).has_value());
        expect(sched.runAndWait().has_value());

        std::vector<float> expected(100UZ);
        for (std::size_t i = 0UZ; i < expected.size(); ++i) {
            expected[i] = static_cast<float>(i) * (i < 50UZ ? 2.f : 5.f);
        }
        expect(eq(sink._samples, expected)) << "the factor changes at the tagged sample";
    };

    "an empty script_parameters map keeps the two-argument call"_test = [] {
        auto makeAndRun = [](const std::string& pythonScript) {
            PythonBlock<float> myBlock({{"n_inputs", 1U}, {"n_outputs", 1U}, {"pythonScript", pythonScript}});
            myBlock.init(myBlock.progress); // needed for unit-test only when executed outside a Scheduler/Graph
            const auto parameters = myBlock.settings().get("script_parameters");
            expect(parameters.has_value() && parameters->is_map() && parameters->get_if<gr::property_map>()->empty()) << "the block declares an empty map by default";
            return runOnce(myBlock, {1.f, 2.f, 3.f});
        };

        expect(eq(makeAndRun(R"(def process_bulk(ins, outs):
    for i in range(len(ins)):
        outs[i][:] = ins[i] * 2
)"),
            std::vector<float>{2.f, 4.f, 6.f}))
            << "a script that takes two arguments";
        expect(eq(makeAndRun(R"(def process_bulk(ins, outs, factor=3):
    for i in range(len(ins)):
        outs[i][:] = ins[i] * factor
)"),
            std::vector<float>{3.f, 6.f, 9.f}))
            << "a script whose third argument has a default";
    };

    "script parameters that process_bulk does not take are refused at init"_test = [] {
        // with a default for 'factor', the script runs and sets its flag wherever the block calls it
        const std::string pythonScript = R"(def process_bulk(ins, outs, factor=1):
    this_block.setSettings({"ran": "yes"})
    for i in range(len(ins)):
        outs[i][:] = ins[i] * factor
)";
        auto              initError    = [](const gr::property_map& parameters, const std::string& script) {
            PythonBlock<float> myBlock({{"n_inputs", 1U}, {"n_outputs", 1U}, {"pythonScript", script}, {"script_parameters", parameters}});
            try {
                myBlock.settings().init();
                std::ignore = myBlock.settings().applyStagedParameters(); // needed for unit-test only when executed outside a Scheduler/Graph
            } catch (const std::exception& ex) {
                return std::string(ex.what());
            }
            return std::string{};
        };

        const std::string unknownKey = initError({{"gain", 2}}, pythonScript);
        expect(unknownKey.contains("gain")) << std::format("a key process_bulk does not take is refused by name: '{}'", unknownKey);

        const std::string grid = initError({{"grid", gr::pmt::Value(gr::Tensor<float>(std::vector<std::size_t>{2UZ, 2UZ}))}}, R"(def process_bulk(ins, outs, **parameters):
    for i in range(len(ins)):
        outs[i][:] = ins[i]
)");
        expect(grid.contains("grid")) << std::format("a value with no Python form is refused by name: '{}'", grid);

        using namespace gr::blocks::testing;
        Graph graph;
        auto& src   = graph.emplaceBlock<TagSource<float>>({{"n_samples_max", 5U}, {"mark_tag", false}});
        auto& block = graph.emplaceBlock<PythonBlock<float>>({{"n_inputs", 1U}, {"n_outputs", 1U}, {"pythonScript", pythonScript}, {"script_parameters", gr::property_map{{"gain", 2}}}});
        auto& sink  = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_expected", 5U}});
        expect(graph.connect(src, "out", block, "inputs#0").has_value());
        expect(graph.connect(block, "outputs#0", sink, "in").has_value());

        gr::scheduler::Simple sched;
        std::string           reported;
        if (sched.exchange(std::move(graph)).has_value()) {
            if (auto ran = sched.runAndWait(); !ran) {
                reported = ran.error().message;
            }
        }
        expect(reported.contains("gain")) << std::format("the graph run fails and names the key: '{}'", reported);
        expect(!block.getSettings().contains("ran")) << "the script is never called";
    };

    "a refused change at run time keeps the previous parameters"_test = [] {
        std::string            pythonScript = R"(def process_bulk(ins, outs, factor):
    for i in range(len(ins)):
        outs[i][:] = ins[i] * factor
)";
        const gr::property_map previous{{"factor", 2}};

        PythonBlock<float> myBlock({{"n_inputs", 1U}, {"n_outputs", 1U}, {"pythonScript", pythonScript}, {"script_parameters", previous}});
        myBlock.init(myBlock.progress); // needed for unit-test only when executed outside a Scheduler/Graph
        gr::MsgPortIn fromBlock;
        expect(myBlock.msgOut.connect(fromBlock).has_value());
        expect(eq(runOnce(myBlock, {1.f, 2.f, 3.f}), std::vector<float>{2.f, 4.f, 6.f}));

        expect(myBlock.settings().setStaged({{"script_parameters", gr::property_map{{"gain", 2}}}}).empty()) << "the block declares script_parameters";
        bool throws = false;
        try {
            std::ignore = myBlock.settings().applyStagedParameters();
        } catch (const std::exception& ex) {
            throws = true;
            std::println(stderr, "applyStagedParameters() threw: {}", ex.what());
        }
        expect(!throws) << "a refusal at run time is reported, not thrown";

        const auto parameters = myBlock.settings().get("script_parameters");
        expect(parameters.has_value() && *parameters == gr::pmt::Value(previous)) << "the settings report the previous map";
        expect(eq(runOnce(myBlock, {1.f, 2.f, 3.f}), std::vector<float>{2.f, 4.f, 6.f})) << "the script keeps the previous factor";

        auto       messages = fromBlock.streamReader().get();
        const bool reported = std::ranges::any_of(messages, [](const gr::Message& message) { return !message.data.has_value() && message.data.error().message.contains("gain"); });
        std::ignore         = messages.consume(messages.size());
        expect(reported) << "the block reports the refused key on its message port";
    };

    "a script's change to a list parameter stays within one call"_test = [] {
        std::string        pythonScript = R"(def process_bulk(ins, outs, history):
    this_block.setSettings({"seen": str(len(history))})
    history.append(0)
    for i in range(len(ins)):
        outs[i][:] = ins[i]
)";
        PythonBlock<float> myBlock({{"n_inputs", 1U}, {"n_outputs", 1U}, {"pythonScript", pythonScript}, {"script_parameters", gr::property_map{{"history", std::vector<std::int64_t>{1}}}}});
        myBlock.init(myBlock.progress); // needed for unit-test only when executed outside a Scheduler/Graph
        std::ignore = runOnce(myBlock, {1.f});
        std::ignore = runOnce(myBlock, {1.f});
        expect(myBlock.getSettings().contains("seen") && myBlock.getSettings().at("seen") == "1") << "each call receives the list as the map holds it";
    };

    "setSettings refuses a key or a value that is not a string"_test = [] {
        // the script records the message of each refusal under a string key
        std::string pythonScript = R"(def process_bulk(ins, outs):
    for entry, record in (({"n": 1}, "refused_value"), ({2: "two"}, "refused_key")):
        try:
            this_block.setSettings(entry)
        except TypeError as error:
            this_block.setSettings({record: str(error)})
    for i in range(len(ins)):
        outs[i][:] = ins[i]
)";

        PythonBlock<float> myBlock({{"n_inputs", 1U}, {"n_outputs", 1U}, {"pythonScript", pythonScript}});
        myBlock.init(myBlock.progress); // needed for unit-test only when executed outside a Scheduler/Graph
        std::vector<float>                  in{1.f};
        std::vector<float>                  out(1UZ);
        std::vector<std::span<const float>> ins{in};
        std::vector<std::span<float>>       outs{out};
        expect(myBlock.processBulk(std::span(ins), std::span(outs)) == gr::work::Status::OK);

        const auto& settings = myBlock.getSettings();
        expect(settings.contains("refused_value") && settings.at("refused_value").contains("'n': 1")) << "a value that is not a string raises a TypeError naming its key";
        expect(settings.contains("refused_key") && settings.at("refused_key").contains("entry 2:")) << "a key that is not a string raises a TypeError naming the key";
        expect(!settings.contains("n")) << "a refused entry is not stored";
    };
};

int main() { /* tests are statically executed */ }
