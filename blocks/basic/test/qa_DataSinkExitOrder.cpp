#include <boost/ut.hpp>

#include <gnuradio-4.0/Graph.hpp>

#include <gnuradio-4.0/basic/DataSink.hpp>

namespace {
// Built before main, ahead of the sink registry. The sinks main adds to it are destroyed at exit, after main returns
// and after every function-local static built during main.
gr::Graph exitGraph;
} // namespace

int main() {
    using namespace boost::ut;
    using gr::blocks::basic::DataSetSink;
    using gr::blocks::basic::DataSink;
    using gr::blocks::basic::DataSinkQuery;
    using gr::blocks::basic::globalDataSinkRegistry;

    // The case passes when the process exits cleanly: the graph destroys both sinks, still registered, at exit.
    "a graph of static storage duration destroys its registered sinks at exit"_test = [] {
        auto& sink        = exitGraph.emplaceBlock<DataSink<float>>({{"name", "exit_sink"}, {"signal_name", "exit signal"}});
        auto& dataSetSink = exitGraph.emplaceBlock<DataSetSink<float>>({{"name", "exit_data_set_sink"}, {"signal_name", "exit data set"}});
        expect(sink._registered && dataSetSink._registered) << "both sinks register when their settings apply" << fatal;
        expect(globalDataSinkRegistry().getStreamingPoller<float>(DataSinkQuery::signalName("exit signal")) != nullptr) << "the DataSink is found";
        expect(globalDataSinkRegistry().getDataSetPoller<float>(DataSinkQuery::signalName("exit data set")) != nullptr) << "the DataSetSink is found";
    };
}
