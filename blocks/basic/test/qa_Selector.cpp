#include <unordered_set>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/meta/utils.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/basic/Selector.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

using namespace std::string_literals;

struct TestParams {
    gr::Size_t                                     nSamples;
    std::vector<std::pair<gr::Size_t, gr::Size_t>> mapping;
    std::vector<gr::Tensor<double>>                inValues;
    std::vector<gr::Tensor<double>>                outValues;
    std::vector<std::vector<gr::Tag>>              inTags;
    std::vector<std::vector<gr::Tag>>              outTags;
    gr::Size_t                                     monitorSource;
    std::vector<double>                            monitorValues;
    bool                                           backPressure;
    std::vector<gr::Size_t>                        nSamplesSelectorInput; // check back pressure
    bool                                           syncCombinedPorts{true};
    bool                                           ignoreOrder{false};
};

std::vector<gr::Tensor<double>> values(std::initializer_list<std::initializer_list<double>> data) {
    std::vector<gr::Tensor<double>> result;
    for (const auto tensorData : data) {
        result.emplace_back(gr::data_from, tensorData);
    }
    return result;
}

void execute_selector_test(TestParams params, std::source_location location = std::source_location::current()) {
    using namespace boost::ut;
    using namespace gr::blocks::testing;

    const gr::Size_t nSources = static_cast<gr::Size_t>(params.inValues.size());
    const gr::Size_t nSinks   = static_cast<gr::Size_t>(params.outValues.size());

    gr::Graph                                                       graph;
    std::vector<TagSource<double>*>                                 sources;
    std::vector<TagSink<double, ProcessFunction::USE_PROCESS_ONE>*> sinks;
    gr::blocks::basic::Selector<double>*                            selector;

    gr::Tensor<gr::Size_t> mapIn(gr::extents_from, {params.mapping.size()});
    gr::Tensor<gr::Size_t> mapOut(gr::extents_from, {params.mapping.size()});
    std::ranges::transform(params.mapping, mapIn.begin(), [](auto& p) { return p.first; });
    std::ranges::transform(params.mapping, mapOut.begin(), [](auto& p) { return p.second; });

    selector = std::addressof(graph.emplaceBlock<gr::blocks::basic::Selector<double>>({{"n_inputs", nSources}, {"n_outputs", nSinks}, {"map_in", mapIn}, {"map_out", mapOut}, {"back_pressure", params.backPressure}, {"sync_combined_ports", params.syncCombinedPorts}, {"disconnect_on_done", false}}));

    for (gr::Size_t i = 0; i < nSources; ++i) {
        sources.push_back(std::addressof(graph.emplaceBlock<TagSource<double>>({{"n_samples_max", params.nSamples}, {"values", params.inValues[i]}, {"disconnect_on_done", false}})));
        expect(sources[i]->settings().applyStagedParameters().forwardParameters.empty());
        sources[i]->_tags = params.inTags[i];
        expect(graph.connect(*sources[i], "out"s, *selector, "inputs#"s + std::to_string(i)).has_value());
    }

    for (gr::Size_t i = 0; i < nSinks; ++i) {
        sinks.push_back(std::addressof(graph.emplaceBlock<TagSink<double, ProcessFunction::USE_PROCESS_ONE>>({{"disconnect_on_done", false}})));
        expect(sinks[i]->settings().applyStagedParameters().forwardParameters.empty());
        expect(graph.connect(*selector, "outputs#"s + std::to_string(i), *sinks[i], "in"s).has_value());
    }

    TagSink<double, ProcessFunction::USE_PROCESS_ONE>* monitorSink = std::addressof(graph.emplaceBlock<TagSink<double, ProcessFunction::USE_PROCESS_ONE>>({{"disconnect_on_done", false}}));
    expect(monitorSink->settings().applyStagedParameters().forwardParameters.empty());
    expect(graph.connect(*selector, "monitor"s, *monitorSink, "in"s).has_value());

    gr::scheduler::Simple sched;
    if (auto ret = sched.exchange(std::move(graph)); !ret) {
        throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
    }
    expect(sched.runAndWait().has_value());

    for (std::size_t i = 0; i < selector->inputs.size(); i++) {
        expect(eq(selector->inputs[i].streamReader().available(), params.nSamplesSelectorInput[i]));
    }

    for (std::size_t i = 0; i < sinks.size(); i++) {
        if (params.ignoreOrder) {
            std::ranges::sort(sinks[i]->_samples);
            std::ranges::sort(params.outValues[i]);
        }
        expect(std::ranges::equal(sinks[i]->_samples, params.outValues[i])) //
            << std::format("called from {}:{} -- test failed:\nparams.outValues[i] i={} samples={} outValues={}", location.file_name(), location.line(), i, sinks[i]->_samples, params.outValues[i]);
    }

    for (std::size_t i = 0; i < sinks.size(); i++) {
        expect(equal_tag_lists(sinks[i]->_tags, params.outTags[i], {}));
    }
}

/// one select value per work call, so the Selector reaches its monitor path on every call rather than only the first
struct DrippingSelect : gr::Block<DrippingSelect> {
    gr::PortOut<gr::Size_t> out;

    GR_MAKE_REFLECTABLE(DrippingSelect, out);

    gr::Size_t  index    = 0U;
    std::size_t nMax     = 0UZ;
    std::size_t _emitted = 0UZ;

    gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (_emitted >= nMax) {
            outSpan.publish(0UZ);
            return gr::work::Status::DONE;
        }
        if (outSpan.size() == 0UZ) {
            outSpan.publish(0UZ);
            return gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        outSpan[0UZ] = index;
        _emitted++;
        outSpan.publish(1UZ);
        return gr::work::Status::OK;
    }
};

/// drains a bounded number of samples per call, so the monitor output backs up and bounds the selected source
struct ThrottledSink : gr::Block<ThrottledSink> {
    gr::PortIn<double> in;

    GR_MAKE_REFLECTABLE(ThrottledSink, in);

    std::size_t perCall = 2UZ;
    std::size_t nSeen   = 0UZ;

    gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        const std::size_t n = std::min(perCall, inSpan.size());
        nSeen += n;
        inSpan.consumeTags(n);
        std::ignore = inSpan.consume(n);
        return gr::work::Status::OK;
    }
};

const boost::ut::suite SelectorTest = [] {
    using namespace boost::ut;
    using namespace gr::blocks::basic;

    "Selector<T> constructor"_test = [] {
        Selector<double> block_nop({{"name", "block_nop"}});
        block_nop.init(block_nop.progress);
        expect(eq(block_nop.n_inputs, 0U));
        expect(eq(block_nop.n_outputs, 0U));
        expect(eq(block_nop.inputs.size(), 0U));
        expect(eq(block_nop.outputs.size(), 0U));
        expect(eq(block_nop._internalMappingInOut.size(), 0U));

        Selector<double> block({{"name", "block"}, {"n_inputs", 4U}, {"n_outputs", 3U}});
        block.init(block.progress);
        expect(eq(block.n_inputs, 4U));
        expect(eq(block.n_outputs, 3U));
        expect(eq(block.inputs.size(), 4U));
        expect(eq(block.outputs.size(), 3U));
        expect(eq(block._internalMappingInOut.size(), 0U));
    };

    "basic Selector<T>"_test = [] {
        using T = double;
        const Tensor<uint32_t> outputMap{data_from, {1U, 0U}};
        Selector<T>            block({{"n_inputs", 3U}, {"n_outputs", 2U}, {"map_in", std::vector<gr::Size_t>{0U, 1U}}, {"map_out", outputMap}}); // N.B. 3rd input is unconnected
        block.init(block.progress);
        expect(eq(block._internalMappingInOut.size(), 2U));

        using internal_mapping_t = decltype(block._internalMappingInOut);
        expect(block._internalMappingInOut == internal_mapping_t{{0U, {outputMap[0]}}, {1U, {outputMap[1]}}});
    };

    gr::Tag tag1{1, {{"key1", "value1"}}};
    gr::Tag tag2{2, {{"key2", "value2"}}};
    gr::Tag tag3{3, {{"key3", "value3"}}};

    // Tests without the back pressure

    "Selector<T> 1 to 1 mapping"_test = [tag1, tag2, tag3] {
        execute_selector_test({.nSamples = 5,                                                           //
            .mapping                     = {{0, 0}, {1, 1}, {2, 2}},                                    //
            .inValues                    = values({{1}, {2}, {3}}),                                     //
            .outValues                   = values({{1, 1, 1, 1, 1}, {2, 2, 2, 2, 2}, {3, 3, 3, 3, 3}}), //
            .inTags                      = {{tag1}, {tag2}, {tag3}},                                    //
            .outTags                     = {{tag1}, {tag2}, {tag3}},
            .monitorSource               = -1U, //
            .monitorValues               = {},  //
            .backPressure                = false,
            .nSamplesSelectorInput       = {0, 0, 0},
            .ignoreOrder                 = true});
    };

    "Selector<T> only one input used"_test = [tag1, tag2, tag3] {
        execute_selector_test({.nSamples = 5,                                 //
            .mapping                     = {{1, 1}},                          //
            .inValues                    = values({{1}, {2}, {3}}),           //
            .outValues                   = values({{}, {2, 2, 2, 2, 2}, {}}), //
            .inTags                      = {{tag1}, {tag2}, {tag3}},          //
            .outTags                     = {{}, {tag2}, {}},
            .monitorSource               = -1U, //
            .monitorValues               = {},  //
            .backPressure                = false,
            .nSamplesSelectorInput       = {0, 0, 0},
            .ignoreOrder                 = true});
    };

    "Selector<T> all for one synch_combined_ports = false"_test = [tag1, tag2, tag3] {
        const Tag newTag1{6, tag1.map};
        const Tag newTag2{10, tag2.map};
        const Tag newTag3{13, tag3.map};
        execute_selector_test({.nSamples = 5,                                                               //
            .mapping                     = {{0, 1}, {1, 1}, {2, 1}},                                        //
            .inValues                    = values({{1}, {2}, {3}}),                                         //
            .outValues                   = values({{}, {1, 2, 2, 3, 3, 3, 1, 1, 1, 1, 2, 2, 2, 3, 3}, {}}), //
            .inTags                      = {{tag1}, {tag2}, {tag3}},                                        //
            .outTags                     = {{}, {newTag1, newTag2, newTag3}, {}},
            .monitorSource               = -1U, //
            .monitorValues               = {},  //
            .backPressure                = false,
            .nSamplesSelectorInput       = {0, 0, 0},
            .syncCombinedPorts           = false,
            .ignoreOrder                 = false});
    };

    "Selector<T> all for one synch_combined_ports = true"_test = [tag1, tag2, tag3] {
        const Tag newTag1{3, tag1.map};
        const Tag newTag2{7, tag2.map};
        const Tag newTag3{11, tag3.map};
        execute_selector_test({.nSamples = 5,                                                               //
            .mapping                     = {{0, 1}, {1, 1}, {2, 1}},                                        //
            .inValues                    = values({{1}, {2}, {3}}),                                         //
            .outValues                   = values({{}, {1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3}, {}}), //
            .inTags                      = {{tag1}, {tag2}, {tag3}},                                        //
            .outTags                     = {{}, {newTag1, newTag2, newTag3}, {}},
            .monitorSource               = -1U, //
            .monitorValues               = {},  //
            .backPressure                = false,
            .nSamplesSelectorInput       = {0, 0, 0},
            .syncCombinedPorts           = true,
            .ignoreOrder                 = false});
    };

    "Selector<T> combined ports that exclude input 0"_test = [tag1, tag2, tag3] {
        const Tag newTag2{4, tag2.map};
        const Tag newTag3{7, tag3.map};
        execute_selector_test({.nSamples = 5,                                            //
            .mapping                     = {{1, 1}, {2, 1}},                             //
            .inValues                    = values({{1}, {2}, {3}}),                      //
            .outValues                   = values({{}, {2, 3, 2, 3, 2, 3, 2, 3, 2, 3}}), //
            .inTags                      = {{tag1}, {tag2}, {tag3}},                     //
            .outTags                     = {{}, {newTag2, newTag3}},
            .monitorSource               = -1U, //
            .monitorValues               = {},  //
            .backPressure                = false,
            .nSamplesSelectorInput       = {0, 0, 0},
            .syncCombinedPorts           = true,
            .ignoreOrder                 = false});
    };

    "Selector<T> one for all"_test = [tag1, tag2, tag3] {
        execute_selector_test({.nSamples = 5,                                                           //
            .mapping                     = {{1, 0}, {1, 1}, {1, 2}},                                    //
            .inValues                    = values({{1}, {2}, {3}}),                                     //
            .outValues                   = values({{2, 2, 2, 2, 2}, {2, 2, 2, 2, 2}, {2, 2, 2, 2, 2}}), //
            .inTags                      = {{tag1}, {tag2}, {tag3}},                                    //
            .outTags                     = {{tag2}, {tag2}, {tag2}},
            .monitorSource               = -1U, //
            .monitorValues               = {},  //
            .backPressure                = false,
            .nSamplesSelectorInput       = {0, 0, 0},
            .ignoreOrder                 = false});
    };

    // tests with the back pressure

    "Selector<T> 1 to 1 mapping, with back pressure"_test = [tag1, tag2, tag3] {
        execute_selector_test({.nSamples = 5,                                                           //
            .mapping                     = {{0, 0}, {1, 1}, {2, 2}},                                    //
            .inValues                    = values({{1}, {2}, {3}}),                                     //
            .outValues                   = values({{1, 1, 1, 1, 1}, {2, 2, 2, 2, 2}, {3, 3, 3, 3, 3}}), //
            .inTags                      = {{tag1}, {tag2}, {tag3}},                                    //
            .outTags                     = {{tag1}, {tag2}, {tag3}},
            .monitorSource               = -1U, //
            .monitorValues               = {},  //
            .backPressure                = true,
            .nSamplesSelectorInput       = {0, 0, 0},
            .ignoreOrder                 = false});
    };

    "Selector<T> only one input used, with back pressure"_test = [tag1, tag2, tag3] {
        execute_selector_test({.nSamples = 5,                                 //
            .mapping                     = {{1, 1}},                          //
            .inValues                    = values({{1}, {2}, {3}}),           //
            .outValues                   = values({{}, {2, 2, 2, 2, 2}, {}}), //
            .inTags                      = {{tag1}, {tag2}, {tag3}},          //
            .outTags                     = {{}, {tag2}, {}},
            .monitorSource               = -1U, //
            .monitorValues               = {},  //
            .backPressure                = true,
            .nSamplesSelectorInput       = {5, 0, 5},
            .ignoreOrder                 = false});
    };

    "Selector<T> all for one, with back pressure"_test = [tag1, tag2, tag3] {
        const Tag newTag1{3, tag1.map};
        const Tag newTag2{7, tag2.map};
        const Tag newTag3{11, tag3.map};
        execute_selector_test({.nSamples = 5,                                                               //
            .mapping                     = {{0, 1}, {1, 1}, {2, 1}},                                        //
            .inValues                    = values({{1}, {2}, {3}}),                                         //
            .outValues                   = values({{}, {1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3}, {}}), //
            .inTags                      = {{tag1}, {tag2}, {tag3}},                                        //
            .outTags                     = {{}, {newTag1, newTag2, newTag3}, {}},
            .monitorSource               = -1U, //
            .monitorValues               = {},  //
            .backPressure                = true,
            .nSamplesSelectorInput       = {0, 0, 0},
            .ignoreOrder                 = false});
    };

    "Selector<T> one for all, with back pressure"_test = [tag1, tag2, tag3] {
        execute_selector_test({.nSamples = 5,                                                           //
            .mapping                     = {{1, 0}, {1, 1}, {1, 2}},                                    //
            .inValues                    = values({{1}, {2}, {3}}),                                     //
            .outValues                   = values({{2, 2, 2, 2, 2}, {2, 2, 2, 2, 2}, {2, 2, 2, 2, 2}}), //
            .inTags                      = {{tag1}, {tag2}, {tag3}},                                    //
            .outTags                     = {{tag2}, {tag2}, {tag2}},
            .monitorSource               = -1U, //
            .monitorValues               = {},  //
            .backPressure                = true,
            .nSamplesSelectorInput       = {5, 0, 5},
            .ignoreOrder                 = false});
    };

    // Tests with a monitor

    "Selector<T> 1 to 1 mapping, with monitor, monitor source already mapped"_test = [tag1, tag2, tag3] {
        execute_selector_test({.nSamples = 5,                                                           //
            .mapping                     = {{0, 0}, {1, 1}, {2, 2}},                                    //
            .inValues                    = values({{1}, {2}, {3}}),                                     //
            .outValues                   = values({{1, 1, 1, 1, 1}, {2, 2, 2, 2, 2}, {3, 3, 3, 3, 3}}), //
            .inTags                      = {{tag1}, {tag2}, {tag3}},                                    //
            .outTags                     = {{tag1}, {tag2}, {tag3}},
            .monitorSource               = 0U,              // set monitor index
            .monitorValues               = {1, 1, 1, 1, 1}, //
            .backPressure                = false,
            .nSamplesSelectorInput       = {0, 0, 0},
            .ignoreOrder                 = false});
    };

    "Selector<T> only one input used, with monitor, monitor source not mapped"_test = [tag1, tag2, tag3] {
        execute_selector_test({.nSamples = 5,                                 //
            .mapping                     = {{1, 1}},                          //
            .inValues                    = values({{1}, {2}, {3}}),           //
            .outValues                   = values({{}, {2, 2, 2, 2, 2}, {}}), //
            .inTags                      = {{tag1}, {tag2}, {tag3}},          //
            .outTags                     = {{}, {tag2}, {}},
            .monitorSource               = 0U,              // set monitor index
            .monitorValues               = {1, 1, 1, 1, 1}, // monitor has values even if port is not mapped
            .backPressure                = false,
            .nSamplesSelectorInput       = {0, 0, 0},
            .ignoreOrder                 = false});
    };

    "Selector<T> all for one, with monitor, monitor source already mapped"_test = [tag1, tag2, tag3] {
        const Tag newTag1{3, tag1.map};
        const Tag newTag2{7, tag2.map};
        const Tag newTag3{11, tag3.map};
        execute_selector_test({.nSamples = 5,                                                               //
            .mapping                     = {{0, 1}, {1, 1}, {2, 1}},                                        //
            .inValues                    = values({{1}, {2}, {3}}),                                         //
            .outValues                   = values({{}, {1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3}, {}}), //
            .inTags                      = {{tag1}, {tag2}, {tag3}},                                        //
            .outTags                     = {{}, {newTag1, newTag2, newTag3}, {}},
            .monitorSource               = 1U,              // set monitor index
            .monitorValues               = {2, 2, 2, 2, 2}, //
            .backPressure                = false,
            .nSamplesSelectorInput       = {0, 0, 0},
            .ignoreOrder                 = true});
    };

    "Selector<T> one for all, with monitor, monitor source already mapped"_test = [tag1, tag2, tag3] {
        execute_selector_test({.nSamples = 5,                                                           //
            .mapping                     = {{1, 0}, {1, 1}, {1, 2}},                                    //
            .inValues                    = values({{1}, {2}, {3}}),                                     //
            .outValues                   = values({{2, 2, 2, 2, 2}, {2, 2, 2, 2, 2}, {2, 2, 2, 2, 2}}), //
            .inTags                      = {{tag1}, {tag2}, {tag3}},                                    //
            .outTags                     = {{tag2}, {tag2}, {tag2}},
            .monitorSource               = 1U,              //
            .monitorValues               = {2, 2, 2, 2, 2}, //
            .backPressure                = false,
            .nSamplesSelectorInput       = {0, 0, 0},
            .ignoreOrder                 = false});
    };

    "Selector<T> a mapped and selected source publishes each tag once under monitor backpressure"_test = [] {
        using namespace gr::blocks::testing;

        constexpr gr::Size_t   kNSamples      = 2048U;
        constexpr std::size_t  kMonitorBuffer = 512UZ; // shorter than the stream, so the monitor bounds the source repeatedly
        const std::vector<Tag> inTags{Tag{1UZ, {{"key1", "value1"}}}, Tag{700UZ, {{"key2", "value2"}}}, Tag{1500UZ, {{"key3", "value3"}}}};

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<TagSource<double>>({{"n_samples_max", kNSamples}, {"values", values({{1}})[0]}, {"disconnect_on_done", false}});
        source._tags     = inTags;

        auto& selectSource = graph.emplaceBlock<DrippingSelect>({{"disconnect_on_done", false}});
        selectSource.nMax  = 256UZ; // more calls than the stream needs, so every one of them reaches the monitor path

        auto& selector      = graph.emplaceBlock<Selector<double>>({{"n_inputs", 1U}, {"n_outputs", 1U}, {"map_in", std::vector<gr::Size_t>{0U}}, {"map_out", std::vector<gr::Size_t>{0U}}, {"back_pressure", false}, {"disconnect_on_done", false}});
        auto& sink          = graph.emplaceBlock<TagSink<double, ProcessFunction::USE_PROCESS_ONE>>({{"disconnect_on_done", false}});
        auto& monitorSink   = graph.emplaceBlock<ThrottledSink>({{"disconnect_on_done", false}});
        monitorSink.perCall = 64UZ;

        expect(graph.connect(source, "out"s, selector, "inputs#0"s).has_value());
        expect(graph.connect(selectSource, "out"s, selector, "select"s).has_value());
        expect(graph.connect(selector, "outputs#0"s, sink, "in"s).has_value());
        expect(graph.connect(selector, "monitor"s, monitorSink, "in"s, {.minBufferSize = kMonitorBuffer}).has_value());

        gr::scheduler::Simple sched;
        expect(sched.exchange(std::move(graph)).has_value());
        expect(sched.runAndWait().has_value());

        expect(eq(sink._samples.size(), static_cast<std::size_t>(kNSamples))) << "the mapped output must receive the whole stream";
        expect(gt(monitorSink.nSeen, kMonitorBuffer)) << "more samples than the monitor ring holds must have crossed it, or the source was never monitor-bound";
        expect(eq(sink._tags.size(), inTags.size())) << "a tag published against a count larger than the one consumed arrives a second time";
        for (const Tag& tag : inTags) {
            const auto nMatching = std::ranges::count_if(sink._tags, [&tag](const Tag& seen) { return seen.map == tag.map; });
            expect(eq(static_cast<std::size_t>(nMatching), 1UZ)) << std::format("the tag at input index {} must arrive exactly once", tag.index);
        }
    };
};

int main() { /* not needed for UT */ }
