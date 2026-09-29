#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/meta/UncertainValue.hpp>

#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/filter/FirFilter.hpp>
#include <gnuradio-4.0/filter/TagDelay.hpp>
#include <gnuradio-4.0/filter/time_domain_filter.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

#include "StreamEndSink.hpp"

/// @brief One sample through the `processBulk` of @p block.
template<typename TBlock, typename T>
[[nodiscard]] T filterOne(TBlock& block, T input) {
    T output{};
    std::ignore = block.processBulk(std::span<const T>(&input, 1UZ), std::span<T>(&output, 1UZ));
    return output;
}

/// @brief What two sinks see of a tagged stream that passed one block: every tag, and the tags on samples alone.
struct TaggedRun {
    bool                       ran     = false;
    std::size_t                samples = 0UZ;
    std::optional<std::size_t> endIndex;   ///< the index of the `end_of_stream` tag
    std::vector<gr::Tag>       tags;       ///< every tag, those at the end-of-stream index included
    std::vector<gr::Tag>       sampleTags; ///< the tags a sample-by-sample consumer sees

    [[nodiscard]] std::vector<std::size_t> offsetsOf(std::string_view key) const { return gr::blocks::filter::testing::offsetsOf(tags, key); }
    [[nodiscard]] std::vector<std::size_t> sampleOffsetsOf(std::string_view key) const { return gr::blocks::filter::testing::offsetsOf(sampleTags, key); }
};

/// @brief Run @p nSamples through a block of type @p TBlock made with @p settings: a trigger name at input @p mid, a
/// trigger time on the next-to-last input, and trigger information with a burst end on the last.
template<typename TBlock>
[[nodiscard]] TaggedRun runTagged(gr::property_map settings, gr::Size_t nSamples, std::size_t mid) {
    using namespace gr::blocks::testing;
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", nSamples}, {"mark_tag", false}});
    source._tags.emplace_back(mid, gr::property_map{{gr::property_map::key_type{"trigger_name"}, std::string("mid")}});
    source._tags.emplace_back(nSamples - 2UZ, gr::property_map{{gr::property_map::key_type{"trigger_time"}, std::uint64_t{1}}});
    source._tags.emplace_back(nSamples - 1UZ, gr::property_map{{gr::property_map::key_type{"trigger_meta_info"}, std::string("last")}, {gr::property_map::key_type{"tx_eob"}, true}});
    auto& block = graph.emplaceBlock<TBlock>(std::move(settings));
    auto  sinks = gr::blocks::filter::testing::connectEndSinks<float>(graph, block);

    TaggedRun run;
    if (!graph.connect<"out", "in">(source, block).has_value() || !sinks.has_value()) {
        return run;
    }
    gr::scheduler::Simple scheduler;
    run.ran        = scheduler.exchange(std::move(graph)).has_value() && scheduler.runAndWait().has_value();
    run.samples    = sinks->end._samples.size();
    run.endIndex   = sinks->end.endIndex();
    run.tags       = sinks->end._tags;
    run.sampleTags = sinks->samples._tags;
    run.ran        = run.ran && sinks->samples._samples.size() == run.samples;
    return run;
}

template<typename T, typename Range>
requires std::floating_point<T>
constexpr size_t estimate_settling_time(const Range& step_response, std::size_t offset = 0, T step_value = 1.0, T threshold = 0.001) {
    if (offset >= step_response.size()) {
        throw std::out_of_range("Offset is greater than the size of the step response.");
    }
    const T lower_bound = step_value - threshold;
    const T upper_bound = step_value + threshold;

    auto begin = step_response.begin() + static_cast<typename Range::difference_type>(offset);
    auto end   = step_response.end();

    auto it = std::find_if(begin, end, [lower_bound, upper_bound](T sample) { return sample >= lower_bound && sample <= upper_bound; });

    // If no such sample is found, return an error
    if (it == end) {
        throw gr::exception("No settling found within the given threshold.");
    }

    // Check if all subsequent samples stay within the acceptable range
    auto it_next = it;
    while (it_next != end) {
        it_next = std::find_if(it_next, end, [lower_bound, upper_bound](T sample) { return sample < lower_bound || sample > upper_bound; });

        if (it_next != end) {
            it = it_next++;
        }
    }

    // Return the settling time (or index)
    return static_cast<std::size_t>(std::distance(begin, it));
}

const boost::ut::suite SequenceTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::filter;

    "FIR and IIR general tests"_test = [] {
        Tensor<double> fir_coeffs(10, 0.1); // box car filter
        Tensor<double> iir_coeffs_b(data_from, {0.55, 0.0});
        Tensor<double> iir_coeffs_a(data_from, {1.0, -0.45});

        // Create FIR and IIR filter instances
        fir_filter<double> fir_filter;
        fir_filter.b = fir_coeffs;

        iir_filter<double, IIRForm::DF_I> iir_filter1;
        iir_filter1.b = iir_coeffs_b;
        iir_filter1.a = iir_coeffs_a;
        iir_filter<double, IIRForm::DF_II> iir_filter2;
        iir_filter2.b = iir_coeffs_b;
        iir_filter2.a = iir_coeffs_a;

        std::vector<double> fir_response;
        std::vector<double> iir_response1;
        std::vector<double> iir_response2;
        for (std::size_t i = 0UL; i < 20; ++i) {
            const double input = (i == 0) ? 0.0 : 1.0; // Step function

            fir_response.push_back(filterOne(fir_filter, input));
            iir_response1.push_back(iir_filter1.processOne(input));
            iir_response2.push_back(iir_filter1.processOne(input));
        }
        expect(eq(fir_response[0], 0.0));
        expect(eq(iir_response1[0], 0.0));
        expect(eq(iir_response2[0], 0.0));

        const std::size_t fir_settling_time  = estimate_settling_time<double>(fir_response);
        const std::size_t iir_settling_time1 = estimate_settling_time<double>(iir_response1);
        const std::size_t iir_settling_time2 = estimate_settling_time<double>(iir_response2);
        expect(eq(fir_settling_time, 10u)) << "FIR settling time";
        expect(eq(iir_settling_time1, 5u)) << "IIR (I) settling time";
        expect(eq(iir_settling_time2, 5u)) << "IIR (II) settling time";

        std::println("FIR      filter settling time: {} ms", fir_settling_time);
        std::println("IIR (I)  filter settling time: {} ms", iir_settling_time1);
        std::println("IIR (II) filter settling time: {} ms", iir_settling_time2);
    };

    "IIR equality tests"_test = [] {
        Tensor<double> iir_coeffs_b(data_from, {0.020083365564211, 0.040166731128423, 0.020083365564211});
        Tensor<double> iir_coeffs_a(data_from, {1.0, -1.561018075800718, 0.641351538057563});

        iir_filter<double, IIRForm::DF_I> iir_filter_I;
        iir_filter_I.b = iir_coeffs_b;
        iir_filter_I.a = iir_coeffs_a;
        iir_filter<double, IIRForm::DF_II> iir_filter_II;
        iir_filter_II.b = iir_coeffs_b;
        iir_filter_II.a = iir_coeffs_a;
        iir_filter<double, IIRForm::DF_I_TRANSPOSED> iir_filter_IT;
        iir_filter_IT.b = iir_coeffs_b;
        iir_filter_IT.a = iir_coeffs_a;
        iir_filter<double, IIRForm::DF_II_TRANSPOSED> iir_filter_IIT;
        iir_filter_IIT.b = iir_coeffs_b;
        iir_filter_IIT.a = iir_coeffs_a;

        constexpr double tolerance = 0.00001;
        for (std::size_t i = 0UL; i < 20; ++i) {
            const double input     = (i == 0) ? 0.0 : 1.0; // Step function
            const auto   form_I    = iir_filter_I.processOne(input);
            const auto   form_II   = iir_filter_II.processOne(input);
            const auto   form_I_T  = iir_filter_IT.processOne(input);
            const auto   form_II_T = iir_filter_IIT.processOne(input);
            expect(approx(form_II, form_I, tolerance)) << "direct form II";
            expect(approx(form_I_T, form_I, tolerance)) << "direct form I - transposed";
            expect(approx(form_II_T, form_I, tolerance)) << "direct form II - transposed";

#if defined(__GNUC__) && !defined(__OPTIMIZE__)
            std::print("input[{:2}]={}-> IIR= {:4.2f} (I) {:4.2f} (II) {:4.2f} (I-T) {:4.2f} (II-T)\n", //
                i, input, form_I, form_II, form_I_T, form_II_T);
#endif
        }
    };

    "a run of exact zeros leaves every form's state at zero, not at a subnormal"_test = [] {
        // The pole pair of this section has radius sqrt(0.6414) = 0.80, so a float state driven by exact zeros
        // passes below the smallest normal after 391 samples.
        Tensor<float> coeffs_b(data_from, {0.020083366f, 0.040166732f, 0.020083366f});
        Tensor<float> coeffs_a(data_from, {1.0f, -1.561018076f, 0.641351538f});

        const auto settled = [&](auto& filter) {
            filter.b = coeffs_b;
            filter.a = coeffs_a;
            for (std::size_t i = 0UZ; i < 64UZ; ++i) {
                std::ignore = filter.processOne(1.f);
            }
            float last = 0.f;
            for (std::size_t i = 0UZ; i < 4000UZ; ++i) {
                last = filter.processOne(0.f);
            }
            return last;
        };

        iir_filter<float, IIRForm::DF_I>             form_I;
        iir_filter<float, IIRForm::DF_II>            form_II;
        iir_filter<float, IIRForm::DF_I_TRANSPOSED>  form_I_T;
        iir_filter<float, IIRForm::DF_II_TRANSPOSED> form_II_T;
        expect(eq(std::fpclassify(settled(form_I)), FP_ZERO)) << "direct form I";
        expect(eq(std::fpclassify(settled(form_II)), FP_ZERO)) << "direct form II";
        expect(eq(std::fpclassify(settled(form_I_T)), FP_ZERO)) << "direct form I - transposed";
        expect(eq(std::fpclassify(settled(form_II_T)), FP_ZERO)) << "direct form II - transposed";
    };
};

template<typename T, gr::blocks::filter::FilterType type>
struct FilterTestParam {
    using value_type                  = T;
    static constexpr auto filter_type = type;
};

const boost::ut::suite<"Basic[Decimating]Filter"> BasicFilterTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::filter;
    using namespace std::string_literals;

    constexpr static auto maxOp = []<typename T>(const T a, const T b) -> bool { return std::abs(gr::value(a)) < std::abs(gr::value(b)); };

    constexpr static float       sampleRate     = 1000.0;
    constexpr static float       f_low          = 100.0;
    constexpr static std::size_t filterOrder    = 4;
    constexpr static std::size_t numSamples     = 1000;
    constexpr static std::size_t decimationRate = 5;

    "BasicFilter - Low-pass Filter Test"_test =
        []<typename TTestParameter>() {
            using T         = typename TTestParameter::value_type;
            using ValueType = meta::fundamental_base_value_type_t<T>;
            auto filterType = TTestParameter::filter_type;

            BasicFilter<T> filter;
            filter.filter_type       = filterType;
            filter.filter_response   = filter::Type::LOWPASS;
            filter.filter_order      = filterOrder;
            filter.f_low             = f_low;
            filter.sample_rate       = sampleRate;
            filter.iir_design_method = filter::iir::Design::CHEBYSHEV1;
            filter.fir_design_method = algorithm::window::Type::Hamming;
            filter.designFilter(); // triggers filter re-computation and setting of internal enums

            "verify in-band signal passes through"_test = [&filter] {
                std::vector<T> outputSignal;
                outputSignal.reserve(numSamples);
                T phase = 0;
                for (std::size_t i = 0UZ; i < 2 * numSamples; i++) {
                    // generate a sine wave signal with a frequency below the cutoff
                    phase += T{2} * std::numbers::pi_v<ValueType> * static_cast<ValueType>(50) / static_cast<ValueType>(sampleRate);
                    if (i < numSamples) { // ignore initial transient
                        std::ignore = filterOne(filter, gr::math::sin(phase));
                    } else {
                        outputSignal.push_back(filterOne(filter, gr::math::sin(phase)));
                    }
                }

                ValueType maxOutput = std::abs(gr::value(*std::ranges::max_element(outputSignal, maxOp)));
                expect(ge(maxOutput, static_cast<ValueType>(.9f))) << std::format("{} filter should pass in-band frequencies: max output {}", filter.filter_type, maxOutput);
            };

            "verify out-of-band signal is attenuated"_test = [&filter] {
                std::vector<T> outputSignal;
                outputSignal.reserve(numSamples);
                T phase = 0;
                for (std::size_t i = 0UZ; i < 2 * numSamples; i++) {
                    // generate a sine wave signal with a frequency below the cutoff
                    phase += T{2} * std::numbers::pi_v<ValueType> * static_cast<ValueType>(300) / static_cast<ValueType>(sampleRate);
                    if (i < numSamples) { // ignore initial transient
                        std::ignore = filterOne(filter, gr::math::sin(phase));
                    } else {
                        outputSignal.push_back(filterOne(filter, gr::math::sin(phase)));
                    }
                }

                ValueType maxOutput = std::abs(gr::value(*std::ranges::max_element(outputSignal, maxOp)));
                expect(le(maxOutput, static_cast<ValueType>(.2f))) << std::format("{} filter should attenuate out-of-band frequencies: max output {}", filter.filter_type, maxOutput);
            };
        } |
        std::tuple<FilterTestParam<float, FilterType::FIR>,          //
            FilterTestParam<double, FilterType::FIR>,                //
            FilterTestParam<UncertainValue<float>, FilterType::FIR>, //
            FilterTestParam<float, FilterType::IIR>,                 //
            FilterTestParam<double, FilterType::IIR>,                //
            FilterTestParam<UncertainValue<float>, FilterType::IIR>>{};

    "UncertainValue<double> header type smoke test"_test = [] {
        static_assert(BlockLike<BasicFilter<UncertainValue<double>>>);
        BasicFilter<UncertainValue<double>> filter;
        expect(filter.filter_type == FilterType::IIR);
    };

    "BasicDecimatingFilter - Low-pass Filter Test"_test = [](const FilterType& filterType) {
        using T = double;

        // Instantiate the BasicDecimatingFilter with the desired decimation rate
        BasicDecimatingFilter<T> filter;
        filter.filter_type       = filterType;
        filter.filter_response   = filter::Type::LOWPASS;
        filter.filter_order      = filterOrder;
        filter.f_low             = f_low;
        filter.sample_rate       = sampleRate;
        filter.iir_design_method = filter::iir::Design::CHEBYSHEV1;
        filter.fir_design_method = algorithm::window::Type::Hamming;
        filter.decimate          = decimationRate;
        filter.designFilter(); // triggers filter re-computation and setting of internal enums

        expect(eq(filter.input_chunk_size, decimationRate)) << "decimationRate type mismatch";

        "verify in-band signal passes through"_test = [&filter] {
            std::vector<T> inputSignal(numSamples);
            std::vector<T> outputSignal(numSamples / decimationRate);

            T    phase          = 0;
            auto generateSample = [&phase]() {
                // generate a sine wave signal with a frequency below the cutoff
                phase += 2 * std::numbers::pi_v<T> * static_cast<T>(50) / static_cast<T>(sampleRate);
                return std::sin(phase);
            };
            std::ranges::generate(inputSignal, generateSample);
            expect(filter.processBulk(inputSignal, outputSignal) == work::Status::OK) << "first processing failed";
            std::ranges::generate(inputSignal, generateSample);
            expect(filter.processBulk(inputSignal, outputSignal) == work::Status::OK) << "second processing failed";

            double sumSq = 0.0;
            for (const auto& v : outputSignal) {
                sumSq += static_cast<double>(v) * static_cast<double>(v);
            }
            double amplitude = std::sqrt(2.0 * sumSq / static_cast<double>(outputSignal.size()));
            expect(ge(amplitude, T{0.9})) << std::format("{} filter should pass in-band frequencies: amplitude {}", filter.filter_type, amplitude);
        };

        "verify out-of-band signal is attenuated"_test = [&filter] {
            std::vector<T> inputSignal(numSamples);
            std::vector<T> outputSignal(numSamples / decimationRate);

            T    phase          = 0;
            auto generateSample = [&phase]() {
                // generate a sine wave signal with a frequency above the cutoff
                phase += 2 * std::numbers::pi_v<T> * T(300) / T(sampleRate);
                return std::sin(phase);
            };
            std::ranges::generate(inputSignal, generateSample);
            expect(filter.processBulk(inputSignal, outputSignal) == work::Status::OK) << "first processing failed";
            std::ranges::generate(inputSignal, generateSample);
            expect(filter.processBulk(inputSignal, outputSignal) == work::Status::OK) << "second processing failed";

            double maxOutput = std::abs(*std::ranges::max_element(outputSignal, maxOp));
            expect(le(maxOutput, T{0.2})) << std::format("{} filter should attenuate out-of-band frequencies: max output {}", filter.filter_type, maxOutput);
        };
    } | std::vector<FilterType>({FilterType::FIR, FilterType::IIR});

    "Decimator - Low-pass Filter Test"_test = [] {
        using namespace gr::blocks::testing;
        using T = int;

        constexpr gr::Size_t decimationFactor = 10;

        gr::Graph flow;
        auto&     source    = flow.emplaceBlock<CountingSource<T>>({{"n_samples_max", 10 * decimationFactor}});
        auto&     decimator = flow.emplaceBlock<gr::blocks::filter::Decimator<T>>({{"decim", decimationFactor}});
        auto&     sink      = flow.emplaceBlock<CountingSink<T>>();
        expect(flow.connect<"out", "in">(source, decimator).has_value());
        expect(flow.connect<"out", "in">(decimator, sink).has_value());

        gr::scheduler::Simple<> sched;
        ;
        if (auto ret = sched.exchange(std::move(flow)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sched.runAndWait().has_value());

        expect(eq(decimator.decim, decimationFactor));
        expect(eq(decimator.output_chunk_size, static_cast<gr::Size_t>(1)));
        expect(eq(decimator.input_chunk_size, decimationFactor));

        expect(eq(sink.count, static_cast<gr::Size_t>(10)));
    };
};

const boost::ut::suite<"tag placement"> TagPlacementTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::filter;

    constexpr gr::Size_t  kSamples = 1000U;
    constexpr std::size_t kMid     = 100UZ;
    constexpr std::size_t kLast    = kSamples - 1UZ;

    "fir_filter moves a tag by its delay as FirFilter does, and a tag past the end leaves at the end-of-stream index"_test = [] {
        // 31 equal taps delay by 15: the trigger at input 100 leaves on output 115, the tags on the last two inputs one
        // past the last output
        const std::vector<float> taps(31UZ, 1.0f / 31.0f);
        const TaggedRun          got = runTagged<fir_filter<float>>({{"b", taps}}, kSamples, kMid);
        const TaggedRun          ref = runTagged<FirFilter<float>>({{"taps", taps}}, kSamples, kMid);
        expect(got.ran && ref.ran);
        expect(eq(got.samples, std::size_t{kSamples})) << "every output, and none past the last input";
        expect(that % (got.endIndex == std::optional<std::size_t>{std::size_t{kSamples}})) << "the stream ends one past the last output";
        expect(that % (got.offsetsOf("trigger_name") == std::vector<std::size_t>{kMid + 15UZ})) << "the trigger on the delayed sample";
        expect(that % (got.sampleOffsetsOf("trigger_name") == std::vector<std::size_t>{kMid + 15UZ})) << "where a sample-by-sample consumer sees it";
        for (const std::string_view key : {"trigger_time", "trigger_meta_info", "tx_eob"}) {
            expect(that % (got.offsetsOf(key) == std::vector<std::size_t>{std::size_t{kSamples}})) << std::format("{} past the end at the end-of-stream index", key);
            expect(that % got.sampleOffsetsOf(key).empty()) << std::format("{} on no sample", key);
        }
        for (const std::string_view key : {"trigger_name", "trigger_time", "trigger_meta_info", "tx_eob"}) {
            expect(that % (got.offsetsOf(key) == ref.offsetsOf(key))) << std::format("{} where FirFilter puts it", key);
        }
    };

    "BasicFilter and BasicDecimatingFilter in FIR mode move a tag by the design's delay as FirFilter does"_test = [] {
        gr::filter::FilterParameters params;
        params.order                   = 4U;
        params.fLow                    = 100.0;
        params.fHigh                   = 200.0;
        params.fs                      = 1000.0;
        const std::vector<float> taps  = gr::filter::fir::designFilter<float>(gr::filter::Type::LOWPASS, params, gr::algorithm::window::Type::Hamming).b;
        const std::uint64_t      twice = gr::blocks::filter::detail::twiceTapDelay(std::span<const float>(taps));
        expect(gt(twice, 4ULL)) << "the design delays the last two inputs past the last output";

        const gr::property_map design{{"filter_type", std::string("FIR")}, {"filter_response", std::string("LOWPASS")}, {"filter_order", gr::Size_t{4}}, {"f_low", 100.0f}, {"f_high", 200.0f}, {"sample_rate", 1000.0f}, {"fir_design_method", std::string("Hamming")}};
        // at M = 5 a stream of 1003 ends in a partial chunk of 3 inputs, which holds both end tags and makes no output
        for (const gr::Size_t decimation : {gr::Size_t{1}, gr::Size_t{5}}) {
            for (const gr::Size_t samples : {kSamples, gr::Size_t{kSamples + 3U}}) {
                gr::property_map settings = design;
                settings.insert_or_assign(gr::property_map::key_type{"decimate"}, decimation);
                const TaggedRun   got     = decimation == 1U ? runTagged<BasicFilter<float>>(settings, samples, kMid) : runTagged<BasicDecimatingFilter<float>>(settings, samples, kMid);
                const TaggedRun   ref     = runTagged<FirFilter<float>>({{"taps", taps}, {"decimation", decimation}}, samples, kMid);
                const std::size_t outputs = samples / decimation;
                const std::size_t mid     = gr::blocks::filter::detail::mapDelayedOffset(kMid, 1ULL, decimation, twice);
                const std::string label   = std::format("M = {}, {} inputs", decimation, samples);

                expect(got.ran && ref.ran);
                expect(eq(got.samples, outputs)) << label << ": every output, and none past the last input";
                expect(that % (got.endIndex == std::optional<std::size_t>{outputs})) << label << ": the stream ends one past the last output";
                expect(that % (got.offsetsOf("trigger_name") == std::vector<std::size_t>{mid})) << label << ": the trigger on the delayed sample";
                expect(that % (got.sampleOffsetsOf("trigger_name") == std::vector<std::size_t>{mid})) << label << ": where a sample-by-sample consumer sees it";
                for (const std::string_view key : {"trigger_time", "trigger_meta_info", "tx_eob"}) {
                    expect(that % (got.offsetsOf(key) == std::vector<std::size_t>{outputs})) << label << ": " << key << " past the end at the end-of-stream index";
                    expect(that % got.sampleOffsetsOf(key).empty()) << label << ": " << key << " on no sample";
                }
                for (const std::string_view key : {"trigger_name", "trigger_time", "trigger_meta_info", "tx_eob"}) {
                    expect(that % (got.offsetsOf(key) == ref.offsetsOf(key))) << label << ": " << key << " where FirFilter puts it";
                }
            }
        }
    };

    "BasicFilter in IIR mode leaves a tag on its own input"_test = [] {
        const TaggedRun got = runTagged<BasicFilter<float>>({{"filter_type", std::string("IIR")}, {"f_low", 100.0f}, {"sample_rate", 1000.0f}}, kSamples, kMid);
        expect(got.ran);
        expect(eq(got.samples, std::size_t{kSamples}));
        expect(that % (got.offsetsOf("trigger_name") == std::vector<std::size_t>{kMid})) << "the framework places the tag";
        expect(that % (got.offsetsOf("trigger_meta_info") == std::vector<std::size_t>{kLast}));
        expect(that % (got.sampleOffsetsOf("trigger_meta_info") == std::vector<std::size_t>{kLast})) << "on the sample a sample-by-sample consumer sees";
    };

    "tags an upstream block leaves at its end-of-stream index pass to fir_filter's and BasicFilter's end-of-stream index"_test = [] {
        // a FirFilter at M = 4 publishes the tags on the last inputs at index 250, where no sample is; fir_filter, and
        // BasicFilter in IIR mode through the framework's key filter, pass them to their own end-of-stream index
        namespace filter_test = gr::blocks::filter::testing;
        const std::vector<gr::Tag> tags{{400UZ, gr::property_map{{gr::property_map::key_type{"trigger_meta_info"}, std::string("mid")}}}, {990UZ, gr::property_map{{gr::property_map::key_type{"trigger_name"}, std::string("burst")}}}, {999UZ, gr::property_map{{gr::property_map::key_type{"tx_eob"}, true}}}};
        const gr::property_map     upstream{{"taps", gr::filter::fir::design::kaiserLowpass(31, 0.1, 60.0)}, {"decimation", 4U}};

        const auto fir = filter_test::runChained<FirFilter<float>, fir_filter<float>>(upstream, {{"b", std::vector<float>(31UZ, 1.0f / 31.0f)}}, 1000U, tags);
        filter_test::expectAtStreamEnd(fir, 250UZ, {"trigger_name", "tx_eob"}, "fir_filter");
        expect(eq(fir.sampleOffsetsOf("trigger_meta_info").size(), 1UZ)) << "fir_filter: a tag inside the stream reaches a sample";

        const auto iir = filter_test::runChained<FirFilter<float>, BasicFilter<float>>(upstream, {{"filter_type", std::string("IIR")}, {"f_low", 100.0f}, {"sample_rate", 1000.0f}}, 1000U, tags);
        filter_test::expectAtStreamEnd(iir, 250UZ, {"trigger_name"}, "BasicFilter, IIR");
        expect(that % iir.offsetsOf("tx_eob").empty()) << "BasicFilter, IIR: the framework's key filter drops a key it does not forward";
    };

    "BasicDecimatingFilter in IIR mode publishes the tags of its partial last chunk at the end-of-stream index"_test = [] {
        // at M = 5 a stream of 1003 ends in a partial chunk of 3 inputs, which holds both end tags; the framework
        // forwards them from no call, and they leave through its key filter one past the 200 outputs
        gr::property_map settings{{"filter_type", std::string("IIR")}, {"f_low", 100.0f}, {"sample_rate", 1000.0f}};
        settings.insert_or_assign(gr::property_map::key_type{"decimate"}, gr::Size_t{5});
        const TaggedRun got = runTagged<BasicDecimatingFilter<float>>(settings, kSamples + 3U, kMid);
        expect(got.ran);
        expect(eq(got.samples, 200UZ)) << "the whole chunks' outputs";
        expect(that % (got.endIndex == std::optional<std::size_t>{200UZ})) << "the stream ends one past the last output";
        for (const std::string_view key : {"trigger_time", "trigger_meta_info"}) {
            expect(that % (got.offsetsOf(key) == std::vector<std::size_t>{200UZ})) << std::format("{} at the end-of-stream index", key);
            expect(that % got.sampleOffsetsOf(key).empty()) << std::format("{} on no sample", key);
        }
        expect(that % got.offsetsOf("tx_eob").empty()) << "the framework's key filter drops a key it does not forward";
    };

    "a switch from FIR to IIR publishes the held tag ahead of the next call's tags"_test = [] {
        // the FIR design delays the trigger on input 99 past the first call's 100 outputs; the second call runs in IIR
        // mode and starts on a tag, which the framework places on output 100: the held trigger leaves there too
        namespace test = gr::blocks::testing::span;
        BasicFilter<float> block;
        block.filter_type       = FilterType::FIR;
        block.filter_response   = gr::filter::Type::LOWPASS;
        block.filter_order      = 4U;
        block.f_low             = 100.0f;
        block.sample_rate       = 1000.0f;
        block.fir_design_method = gr::algorithm::window::Type::Hamming;
        block.start();
        expect(gt(block.twiceTagDelay().value_or(0ULL), 0ULL)) << "the FIR design delays its tags";

        const std::vector<float>   input(100UZ, 0.0f);
        std::vector<float>         output(100UZ);
        test::Capture<float>       got;
        const std::vector<gr::Tag> firstTags{{99UZ, gr::property_map{{gr::property_map::key_type{"trigger_name"}, std::string("held")}}}};
        const std::vector<gr::Tag> secondTags{{100UZ, gr::property_map{{gr::property_map::key_type{"trigger_meta_info"}, std::string("next")}}}};
        for (const auto& [at, tags] : {std::pair{0UZ, std::span<const gr::Tag>(firstTags)}, std::pair{100UZ, std::span<const gr::Tag>(secondTags)}}) {
            if (at > 0UZ) {
                block.filter_type = FilterType::IIR;
                block.designFilter();
            }
            test::InputSpan<float>  inSpan(std::span<const float>(input), at, tags);
            test::OutputSpan<float> outSpan(std::span<float>(output), at, &got.tags);
            auto                    inputs  = std::tie(inSpan);
            auto                    outputs = std::tie(outSpan);
            block.forwardTags(inputs, outputs, input.size());
            std::ignore = block.processBulk(std::span<const float>(inSpan), std::span<float>(outSpan));
        }

        expect(that % (got.offsetsOf("trigger_name") == std::vector<std::size_t>{100UZ})) << "the held trigger on the IIR call's first output";
        expect(that % (got.offsetsOf("trigger_meta_info") == std::vector<std::size_t>{100UZ})) << "the framework's tag on its own input";
        expect(std::ranges::is_sorted(got.tags, std::ranges::less{}, &gr::Tag::index)) << "the tags published in index order";
    };
};

int main() { /* not needed for UT */ }
