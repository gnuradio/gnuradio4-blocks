#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <format>
#include <numbers>
#include <random>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/filter/SvdDenoiser.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

/// @brief One sample through the `processBulk` of @p block.
template<typename TBlock, typename T>
[[nodiscard]] T filterOne(TBlock& block, T input) {
    T output{};
    std::ignore = block.processBulk(std::span<const T>(&input, 1UZ), std::span<T>(&output, 1UZ));
    return output;
}

const boost::ut::suite<"SvdDenoiser Block"> svdDenoiserBlockTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::filter;

    "SvdDenoiser float default parameters"_test = [] {
        SvdDenoiser<float> denoiser;
        expect(eq(denoiser.window_size.value, static_cast<gr::Size_t>(64)));
        expect(eq(denoiser.energy_fraction.value, 0.95f));
        expect(eq(denoiser.hop_fraction.value, 0.25f));
    };

    "SvdDenoiser float processOne"_test = [] {
        SvdDenoiser<float> denoiser;
        denoiser.start();

        // With BoundaryPolicy::Default, history is pre-filled with zeros
        // Process a few samples and verify we get finite output
        std::vector<float> outputs;
        for (int i = 0; i < 100; ++i) {
            outputs.push_back(filterOne(denoiser, static_cast<float>(i)));
        }
        expect(eq(outputs.size(), 100UZ));
        expect(std::isfinite(outputs.back())) << "output should be finite";
    };

    "SvdDenoiser double processOne"_test = [] {
        SvdDenoiser<double> denoiser;
        denoiser.start();

        std::vector<double> outputs;
        for (int i = 0; i < 100; ++i) {
            outputs.push_back(filterOne(denoiser, static_cast<double>(i)));
        }
        expect(eq(outputs.size(), 100UZ));
        expect(std::isfinite(outputs.back())) << "output should be finite";
    };

    "SvdDenoiser denoising quality"_test = [] {
        constexpr std::size_t N  = 512UZ;
        constexpr double      fs = 1000.0;

        std::mt19937                     gen(42);
        std::normal_distribution<double> noise(0.0, 0.3);

        std::vector<double> clean(N), noisy(N);
        for (std::size_t i = 0UZ; i < N; ++i) {
            double t = static_cast<double>(i) / fs;
            clean[i] = std::sin(2.0 * std::numbers::pi * 50.0 * t);
            noisy[i] = clean[i] + noise(gen);
        }

        SvdDenoiser<double> denoiser;
        denoiser.window_size     = 64;
        denoiser.max_rank        = 3;
        denoiser.energy_fraction = 0.95;
        denoiser.start();

        std::vector<double> denoised(N);
        for (std::size_t i = 0UZ; i < N; ++i) {
            denoised[i] = filterOne(denoiser, noisy[i]);
        }

        const std::size_t filterDelay = (denoiser.window_size.value - 1UZ) / 2UZ;
        const std::size_t skipSamples = std::max(128UZ, filterDelay + denoiser.window_size.value);
        double            noisyRms = 0.0, denoisedRms = 0.0;
        std::size_t       count = 0UZ;

        for (std::size_t i = skipSamples; i < N; ++i) {
            noisyRms += std::pow(noisy[i] - clean[i], 2.0);
            if (i >= filterDelay) {
                denoisedRms += std::pow(denoised[i] - clean[i - filterDelay], 2.0);
                ++count;
            }
        }
        noisyRms    = std::sqrt(noisyRms / static_cast<double>(N - skipSamples));
        denoisedRms = count > 0UZ ? std::sqrt(denoisedRms / static_cast<double>(count)) : 0.0;

        expect(lt(denoisedRms, noisyRms)) << "denoised=" << denoisedRms << " noisy=" << noisyRms;
    };

    "SvdDenoiser settingsChanged"_test = [] {
        SvdDenoiser<float> denoiser;
        denoiser.window_size     = 32;
        denoiser.max_rank        = 2;
        denoiser.energy_fraction = 0.9f;
        denoiser.hop_fraction    = 0.5f;
        denoiser.settingsChanged({}, {{"window_size", 32U}, {"max_rank", static_cast<gr::Size_t>(2)}, {"energy_fraction", 0.9f}, {"hop_fraction", 0.5f}});

        expect(eq(denoiser.window_size.value, static_cast<gr::Size_t>(32)));
        expect(eq(denoiser.max_rank.value, static_cast<gr::Size_t>(2)));
        expect(eq(denoiser.energy_fraction.value, 0.9f));
        expect(eq(denoiser.hop_fraction.value, 0.5f));
    };

    "SvdDenoiser reset"_test = [] {
        SvdDenoiser<float> denoiser;
        denoiser.start();

        for (int i = 0; i < 100; ++i) {
            std::ignore = filterOne(denoiser, static_cast<float>(i));
        }
        denoiser.reset();

        // After reset, history is cleared and re-initialized with zeros
        float output = filterOne(denoiser, 1.0f);
        expect(std::isfinite(output)) << "output finite after reset";
    };
};

const boost::ut::suite<"SvdDenoiser complex support"> svdDenoiserComplexTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::filter;

    "complex float"_test = [] {
        using C = std::complex<float>;
        SvdDenoiser<C> denoiser;
        denoiser.window_size     = 32;
        denoiser.max_rank        = 2;
        denoiser.energy_fraction = 0.95f;
        denoiser.start();

        std::vector<C> output;
        for (std::size_t i = 0UZ; i < 100UZ; ++i) {
            float phase = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i) / 16.0f;
            output.push_back(filterOne(denoiser, C{std::cos(phase), std::sin(phase)}));
        }
        expect(eq(output.size(), 100UZ));
        expect(std::isfinite(output.back().real())) << "output should be finite";
    };

    "complex double"_test = [] {
        using C = std::complex<double>;
        SvdDenoiser<C> denoiser;
        denoiser.window_size     = 32;
        denoiser.max_rank        = 2;
        denoiser.energy_fraction = 0.95;
        denoiser.start();

        std::vector<C> output;
        for (std::size_t i = 0UZ; i < 100UZ; ++i) {
            double phase = 2.0 * std::numbers::pi * static_cast<double>(i) / 16.0;
            output.push_back(filterOne(denoiser, C{std::cos(phase), std::sin(phase)}));
        }
        expect(eq(output.size(), 100UZ));
        expect(std::isfinite(output.back().real())) << "output should be finite";
    };
};

const boost::ut::suite<"SvdDenoiser edge cases"> svdDenoiserEdgeCaseTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::filter;

    "hankel_rows parameter"_test = [] {
        "default hankel_rows = 0 uses window_size/2"_test = [] {
            SvdDenoiser<double> denoiser;
            denoiser.window_size = 64;
            denoiser.hankel_rows = 0; // default: use window_size / 2
            denoiser.start();

            std::vector<double> outputs;
            for (std::size_t i = 0UZ; i < 100UZ; ++i) {
                outputs.push_back(filterOne(denoiser, std::sin(2.0 * std::numbers::pi * static_cast<double>(i) / 16.0)));
            }
            expect(std::isfinite(outputs.back()));
        };

        "custom hankel_rows"_test = [] {
            SvdDenoiser<double> denoiser;
            denoiser.window_size = 32;
            denoiser.hankel_rows = 8; // custom value
            denoiser.start();

            std::vector<double> outputs;
            for (std::size_t i = 0UZ; i < 50UZ; ++i) {
                outputs.push_back(filterOne(denoiser, static_cast<double>(i)));
            }
            expect(std::isfinite(outputs.back()));
        };
    };

    "threshold parameters"_test = [] {
        "relative_threshold effect"_test = [] {
            SvdDenoiser<double> denoiser;
            denoiser.window_size        = 32;
            denoiser.relative_threshold = 0.1; // 10% of max singular value
            denoiser.start();

            std::vector<double> outputs;
            for (std::size_t i = 0UZ; i < 50UZ; ++i) {
                outputs.push_back(filterOne(denoiser, std::sin(2.0 * std::numbers::pi * static_cast<double>(i) / 8.0)));
            }
            expect(std::isfinite(outputs.back()));
        };

        "absolute_threshold effect"_test = [] {
            SvdDenoiser<double> denoiser;
            denoiser.window_size        = 32;
            denoiser.absolute_threshold = 0.01;
            denoiser.start();

            std::vector<double> outputs;
            for (std::size_t i = 0UZ; i < 50UZ; ++i) {
                outputs.push_back(filterOne(denoiser, std::sin(2.0 * std::numbers::pi * static_cast<double>(i) / 8.0)));
            }
            expect(std::isfinite(outputs.back()));
        };

        "combined thresholds"_test = [] {
            SvdDenoiser<double> denoiser;
            denoiser.window_size        = 32;
            denoiser.max_rank           = 3;
            denoiser.relative_threshold = 0.05;
            denoiser.absolute_threshold = 0.001;
            denoiser.energy_fraction    = 0.9;
            denoiser.start();

            std::vector<double> outputs;
            for (std::size_t i = 0UZ; i < 100UZ; ++i) {
                outputs.push_back(filterOne(denoiser, std::sin(2.0 * std::numbers::pi * static_cast<double>(i) / 16.0)));
            }
            expect(std::isfinite(outputs.back()));
        };
    };

    "minimum window_size"_test = [] {
        SvdDenoiser<float> denoiser;
        denoiser.window_size = 4; // very small window
        denoiser.max_rank    = 1;
        denoiser.start();

        std::vector<float> outputs;
        for (int i = 0; i < 20; ++i) {
            outputs.push_back(filterOne(denoiser, static_cast<float>(i)));
        }
        expect(std::isfinite(outputs.back()));
    };

    "hop_fraction variations"_test = [] {
        "small hop_fraction"_test = [] {
            SvdDenoiser<double> denoiser;
            denoiser.window_size  = 32;
            denoiser.hop_fraction = 0.1; // recompute SVD frequently
            denoiser.start();

            std::vector<double> outputs;
            for (std::size_t i = 0UZ; i < 50UZ; ++i) {
                outputs.push_back(filterOne(denoiser, static_cast<double>(i)));
            }
            expect(std::isfinite(outputs.back()));
        };

        "large hop_fraction"_test = [] {
            SvdDenoiser<double> denoiser;
            denoiser.window_size  = 32;
            denoiser.hop_fraction = 0.75; // recompute SVD less frequently
            denoiser.start();

            std::vector<double> outputs;
            for (std::size_t i = 0UZ; i < 50UZ; ++i) {
                outputs.push_back(filterOne(denoiser, static_cast<double>(i)));
            }
            expect(std::isfinite(outputs.back()));
        };
    };

    "near-constant input handling"_test = [] {
        // Pure constant input creates degenerate Hankel matrix, add tiny variation
        SvdDenoiser<double> denoiser;
        denoiser.window_size = 16;
        denoiser.max_rank    = 2;
        denoiser.start();

        std::vector<double> outputs;
        for (std::size_t i = 0UZ; i < 50UZ; ++i) {
            double input = 5.0 + 0.001 * std::sin(2.0 * std::numbers::pi * static_cast<double>(i) / 50.0);
            outputs.push_back(filterOne(denoiser, input));
        }

        for (const auto& val : outputs) {
            expect(std::isfinite(val));
        }
    };
};

const boost::ut::suite<"SvdDenoiser tag placement"> svdTagPlacementTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::filter;
    using namespace gr::blocks::testing;

    "a tag leaves on the output that estimates its input"_test = [] {
        // with every singular value kept the estimate is the input itself: the impulse the source puts under the tag
        // arrives on the lagged output. A window of 64 at the default hop lags by 31. A window of 16 whose hop is the
        // whole window lags by 15, where the library's delay() reads 7.
        constexpr gr::Size_t  kSamples = 200U;
        constexpr std::size_t kAt      = 50UZ;
        struct Case {
            gr::Size_t  window;
            float       hopFraction;
            std::size_t lag;
        };
        for (const Case& c : {Case{64U, 0.25f, 31UZ}, Case{16U, 1.0f, 15UZ}}) {
            gr::Graph graph;
            auto&     source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", kSamples}, {"mark_tag", true}});
            source._tags.emplace_back(kAt, gr::property_map{{gr::property_map::key_type{"trigger_name"}, std::string("impulse")}});
            source._tags.emplace_back(kSamples - 1UZ, gr::property_map{{gr::property_map::key_type{"trigger_meta_info"}, std::string("last")}, {gr::property_map::key_type{"tx_eob"}, true}});
            auto& block = graph.emplaceBlock<SvdDenoiser<float>>({{"window_size", c.window}, {"hop_fraction", c.hopFraction}, {"energy_fraction", 2.0f}});
            auto& sink  = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});
            expect(graph.connect<"out", "in">(source, block).has_value());
            expect(graph.connect<"out", "in">(block, sink).has_value());
            gr::scheduler::Simple scheduler;
            expect(scheduler.exchange(std::move(graph)).has_value());
            expect(scheduler.runAndWait().has_value());

            const auto offsetsOf = [&sink](std::string_view key) {
                std::vector<std::size_t> offsets;
                for (const gr::Tag& seen : sink._tags) {
                    if (seen.map.contains(gr::property_map::key_type{key})) {
                        offsets.push_back(seen.index);
                    }
                }
                return offsets;
            };
            const std::string label = std::format("W = {}, hop fraction {}", c.window, c.hopFraction);
            expect(eq(sink._samples.size(), std::size_t{kSamples})) << label << ": every output, and none past the last input";
            const auto head = std::span<const float>(sink._samples).first(3UZ * kAt);
            expect(eq(static_cast<std::size_t>(std::ranges::max_element(head) - head.begin()), kAt + c.lag)) << label << ": the impulse on the lagged output";
            expect(that % (offsetsOf("trigger_name") == std::vector<std::size_t>{kAt + c.lag})) << label << ": the tag with it";
            expect(that % (offsetsOf("tx_eob") == std::vector<std::size_t>{kSamples - 1UZ})) << label << ": a burst end past the end on the last output";
        }
    };
};

int main() { /* tests are automatically registered and run */ }
