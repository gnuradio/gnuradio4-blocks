#include <boost/ut.hpp>

#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/LifeCycle.hpp>

#include <gnuradio-4.0/basic/DataSink.hpp>

#include <gnuradio-4.0/GrBasicBlocks.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <tuple>

namespace {
/// emplaces a sink whose type the block library compiles; the library's code registers the sink when its settings apply
const std::shared_ptr<gr::BlockModel>& emplaceLibrarySink(gr::Graph& graph, std::string_view typeName, std::string_view name, std::string_view signalName) {
    [[maybe_unused]] static const std::size_t nRegistered = gr::blocklib::initGrBasicBlocks(gr::globalBlockRegistry());
    boost::ut::expect(gr::globalBlockRegistry().contains(typeName)) << "the block library registers " << typeName;
    return graph.emplaceBlock(typeName, {{"name", std::string(name)}, {"signal_name", std::string(signalName)}});
}
} // namespace

const boost::ut::suite<"DataSinkRegistry"> dataSinkRegistryTests = [] {
    using namespace boost::ut;
    using gr::blocks::basic::DataSinkQuery;
    using gr::blocks::basic::globalDataSinkRegistry;

    "a DataSink of the block library is found by signal name and by sink name"_test = [] {
        gr::Graph   graph;
        const auto& sink = emplaceLibrarySink(graph, "gr::blocks::basic::DataSink<float32>", "library_sink", "library_signal");
        expect(sink != nullptr) << fatal;

        expect(globalDataSinkRegistry().getStreamingPoller<float>(DataSinkQuery::signalName("library_signal")) != nullptr) << "found by signal name";
        expect(globalDataSinkRegistry().getStreamingPoller<float>(DataSinkQuery::sinkName("library_sink")) != nullptr) << "found by sink name";
        expect(globalDataSinkRegistry().getStreamingPoller<double>(DataSinkQuery::signalName("library_signal")) == nullptr) << "refused for another sample type";
        expect(globalDataSinkRegistry().getStreamingPoller<double>(DataSinkQuery::sinkName("library_sink")) == nullptr) << "refused for another sample type";
        expect(globalDataSinkRegistry().getDataSetPoller<float>(DataSinkQuery::signalName("library_signal")) == nullptr) << "refused as a DataSetSink";

        expect(sink->changeStateTo(gr::lifecycle::State::REQUESTED_STOP).has_value());
        expect(globalDataSinkRegistry().getStreamingPoller<float>(DataSinkQuery::signalName("library_signal")) == nullptr) << "gone after the sink stops";
        expect(globalDataSinkRegistry().getStreamingPoller<float>(DataSinkQuery::sinkName("library_sink")) == nullptr) << "gone after the sink stops";
    };

    "a DataSetSink of the block library is found by signal name and by sink name"_test = [] {
        gr::Graph   graph;
        const auto& sink = emplaceLibrarySink(graph, "gr::blocks::basic::DataSetSink<float32>", "library_dataset_sink", "library_dataset_signal");
        expect(sink != nullptr) << fatal;

        expect(globalDataSinkRegistry().getDataSetPoller<float>(DataSinkQuery::signalName("library_dataset_signal")) != nullptr) << "found by signal name";
        expect(globalDataSinkRegistry().getDataSetPoller<float>(DataSinkQuery::sinkName("library_dataset_sink")) != nullptr) << "found by sink name";
        expect(globalDataSinkRegistry().getDataSetPoller<std::uint8_t>(DataSinkQuery::signalName("library_dataset_signal")) == nullptr) << "refused for another sample type";
        expect(globalDataSinkRegistry().getStreamingPoller<float>(DataSinkQuery::signalName("library_dataset_signal")) == nullptr) << "refused as a DataSink";

        expect(sink->changeStateTo(gr::lifecycle::State::REQUESTED_STOP).has_value());
        expect(globalDataSinkRegistry().getDataSetPoller<float>(DataSinkQuery::signalName("library_dataset_signal")) == nullptr) << "gone after the sink stops";
    };

    "a sink refused a taken signal name leaves the owner registered when it stops"_test = [] {
        gr::Graph   graph;
        const auto& owner = emplaceLibrarySink(graph, "gr::blocks::basic::DataSink<float32>", "owner_sink", "shared_signal");
        const auto& other = graph.emplaceBlock("gr::blocks::basic::DataSink<float32>", {{"name", std::string("other_sink")}});
        expect(owner != nullptr && other != nullptr) << fatal;

        // the other sink takes the owner's signal name at run time; the registry refuses it, and the other sink's
        // signal_name stays shared_signal
        expect(other->settings().setStaged({{"signal_name", std::string("shared_signal")}}).empty());
        const bool refused = [&other] {
            try {
                std::ignore = other->settings().applyStagedParameters();
            } catch (...) {
                return true;
            }
            return false;
        }();
        expect(refused) << "the registry refuses the taken name";
        expect(globalDataSinkRegistry().getStreamingPoller<float>(DataSinkQuery::sinkName("other_sink")) == nullptr) << "the other sink is not registered";

        expect(other->changeStateTo(gr::lifecycle::State::REQUESTED_STOP).has_value());
        expect(globalDataSinkRegistry().getStreamingPoller<float>(DataSinkQuery::signalName("shared_signal")) != nullptr) << "the owner is still found by signal name";
        expect(globalDataSinkRegistry().getStreamingPoller<float>(DataSinkQuery::sinkName("owner_sink")) != nullptr) << "the owner is still found by sink name";

        expect(owner->changeStateTo(gr::lifecycle::State::REQUESTED_STOP).has_value());
        expect(globalDataSinkRegistry().getStreamingPoller<float>(DataSinkQuery::signalName("shared_signal")) == nullptr) << "gone after the owner stops";
    };
};

int main() { /* tests are statically executed */ }
