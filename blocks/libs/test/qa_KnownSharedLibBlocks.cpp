#include <boost/ut.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

#include <iostream>

using namespace boost::ut;

using namespace std::string_view_literals;

#include <gnuradio-4.0/GrBasicBlocks.hpp>
#include <gnuradio-4.0/GrElectricalBlocks.hpp>
#include <gnuradio-4.0/GrFileIoBlocks.hpp>
#include <gnuradio-4.0/GrFilterBlocks.hpp>
#include <gnuradio-4.0/GrFourierBlocks.hpp>
#include <gnuradio-4.0/GrHttpBlocks.hpp>
#include <gnuradio-4.0/GrTestingBlocks.hpp>

#if __has_include(<gnuradio-4.0/GrAudioBlocks.hpp>)
#include <gnuradio-4.0/GrAudioBlocks.hpp>
#define GNURADIO4_HAVE_AUDIO_BLOCKS 1
#else
#define GNURADIO4_HAVE_AUDIO_BLOCKS 0
#endif

const boost::ut::suite TagTests = [] {
    auto&       registry = gr::globalBlockRegistry();
    std::size_t result   = 0UZ;
    result += gr::blocklib::initGrBasicBlocks(registry);
#if GNURADIO4_HAVE_AUDIO_BLOCKS
    result += gr::blocklib::initGrAudioBlocks(registry);
#endif
    result += gr::blocklib::initGrElectricalBlocks(registry);
    result += gr::blocklib::initGrFileIoBlocks(registry);
    result += gr::blocklib::initGrFilterBlocks(registry);
    result += gr::blocklib::initGrFourierBlocks(registry);
    result += gr::blocklib::initGrHttpBlocks(registry);
    result += gr::blocklib::initGrTestingBlocks(registry);
    if (result) {
        std::print("Warning: Failed to init {} blocks\n", result);
    }

    "CheckAvailableBlocks"_test = [&] {
        expect(gt(registry.keys().size(), 20UZ));

        expect(registry.contains("gr::blocks::basic::ClockSource"sv));
        expect(registry.contains("gr::blocks::testing::Delay<float32>"sv));
        expect(registry.contains("gr::blocks::testing::NullSource<float32>"sv));
        expect(registry.contains("gr::blocks::testing::NullSource<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::NullSource<gr::Packet<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::NullSource<gr::Tensor<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::NullSource<gr::DataSet<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::ConstantSource<float32>"sv));
        expect(registry.contains("gr::blocks::testing::ConstantSource<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::ConstantSource<gr::Packet<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::ConstantSource<gr::Tensor<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::ConstantSource<gr::DataSet<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::SlowSource<float32>"sv));
        expect(registry.contains("gr::blocks::testing::SlowSource<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::SlowSource<gr::Packet<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::SlowSource<gr::Tensor<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::SlowSource<gr::DataSet<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::CountingSource<float32>"sv));
        expect(registry.contains("gr::blocks::testing::CountingSource<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::Copy<float32>"sv));
        expect(registry.contains("gr::blocks::testing::Copy<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::Copy<gr::Packet<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::Copy<gr::Tensor<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::Copy<gr::DataSet<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::HeadBlock<float32>"sv));
        expect(registry.contains("gr::blocks::testing::HeadBlock<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::HeadBlock<gr::Packet<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::HeadBlock<gr::Tensor<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::HeadBlock<gr::DataSet<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::NullSink<float32>"sv));
        expect(registry.contains("gr::blocks::testing::NullSink<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::NullSink<gr::Packet<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::NullSink<gr::Tensor<float32>>"sv));
        expect(registry.contains("gr::blocks::testing::NullSink<gr::DataSet<float32>>"sv));
        expect(registry.contains("gr::blocks::fileio::BasicFileSink<float32>"sv));
        expect(registry.contains("gr::blocks::basic::Convert<float32, float32>"sv));
        expect(registry.contains("gr::blocks::basic::Convert<float32, int32>"sv));
        expect(registry.contains("gr::blocks::basic::ScalingConvert<float32, float32>"sv));
        expect(registry.contains("gr::blocks::basic::ScalingConvert<float32, int32>"sv));
        expect(registry.contains("gr::blocks::basic::DataSink<float32>"sv));
        expect(registry.contains("gr::blocks::basic::SchmittTrigger<float32, (gr::trigger::InterpolationMethod)0>"sv));
#if defined(_WIN32)
        expect(registry.contains("gr::blocks::electrical::PowerMetrics<float32, 3ull>"sv));
#else
        expect(registry.contains("gr::blocks::electrical::PowerMetrics<float32, 3ul>"sv));
#endif
        expect(registry.contains("gr::blocks::http::HttpSource"sv));
        expect(registry.contains("gr::blocks::http::HttpSink"sv));
        expect(registry.contains("gr::blocks::fileio::WavSource<float32>"sv));
        expect(registry.contains("gr::blocks::fileio::WavSink<float32>"sv));
#if GNURADIO4_HAVE_AUDIO_BLOCKS
        expect(registry.contains("gr::blocks::audio::AudioSink<float32>"sv));
#endif
        expect(registry.contains("gr::blocks::filter::fir_filter<float32>"sv));
        expect(registry.contains("gr::blocks::fourier::FFT<float32>"sv));
    };

    "CheckBlockInstantiations"_test = [&] {
        expect(registry.create("gr::blocks::testing::Delay<float32>"sv, {}) != nullptr);
        expect(registry.create("gr::blocks::fileio::WavSource<float32>"sv, {}) != nullptr);
#if GNURADIO4_HAVE_AUDIO_BLOCKS
        expect(registry.create("gr::blocks::audio::AudioSink<float32>"sv, {}) != nullptr);
#endif
        expect(registry.create("gr::blocks::basic::DataSink<float32>"sv, {}) != nullptr);
        expect(registry.create("gr::blocks::basic::ClockSource"sv, {}) != nullptr);
    };
};
int main() { /* not needed for UT */ }
