// Python related includes need to be first. This unit holds the program's NumPy API table and interpreter runtime.
#define GR_PYTHON_RUNTIME_OWNER
#include <gnuradio-4.0/basic/PythonBlock.hpp>

#include <boost/ut.hpp>

#include <gnuradio-4.0/GrBasicBlocks.hpp>
#include <gnuradio-4.0/Graph.hpp>

#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

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
};

int main() { /* tests are statically executed */ }
