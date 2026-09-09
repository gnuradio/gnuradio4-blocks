#include <boost/ut.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

#include <iostream>

using namespace boost::ut;

using namespace std::string_view_literals;

#include <gnuradio-4.0/GrAnalogBlocks.hpp>
#include <gnuradio-4.0/GrBasicBlocks.hpp>
#include <gnuradio-4.0/GrChannelBlocks.hpp>
#include <gnuradio-4.0/GrDigitalBlocks.hpp>
#include <gnuradio-4.0/GrElectricalBlocks.hpp>
#include <gnuradio-4.0/GrFecBlocks.hpp>
#include <gnuradio-4.0/GrFileIoBlocks.hpp>
#include <gnuradio-4.0/GrFilterBlocks.hpp>
#include <gnuradio-4.0/GrFourierBlocks.hpp>
#include <gnuradio-4.0/GrHttpBlocks.hpp>
#include <gnuradio-4.0/GrSyncBlocks.hpp>
#include <gnuradio-4.0/GrTestingBlocks.hpp>

#if __has_include(<gnuradio-4.0/GrAudioBlocks.hpp>)
#include <gnuradio-4.0/GrAudioBlocks.hpp>
#define GNURADIO4_HAVE_AUDIO_BLOCKS 1
#else
#define GNURADIO4_HAVE_AUDIO_BLOCKS 0
#endif

// the network family is optional and its generated header outlives a reconfiguration that switches the family off,
// so the build system says whether the library is there rather than the build tree being asked
#ifndef GNURADIO4_HAVE_NETWORK_BLOCKS
#define GNURADIO4_HAVE_NETWORK_BLOCKS 0
#endif
#if GNURADIO4_HAVE_NETWORK_BLOCKS
#include <gnuradio-4.0/GrNetworkBlocks.hpp>
#endif

const boost::ut::suite TagTests = [] {
    auto&       registry = gr::globalBlockRegistry();
    std::size_t result   = 0UZ;
    result += gr::blocklib::initGrAnalogBlocks(registry);
    result += gr::blocklib::initGrBasicBlocks(registry);
    result += gr::blocklib::initGrChannelBlocks(registry);
    result += gr::blocklib::initGrDigitalBlocks(registry);
#if GNURADIO4_HAVE_AUDIO_BLOCKS
    result += gr::blocklib::initGrAudioBlocks(registry);
#endif
    result += gr::blocklib::initGrElectricalBlocks(registry);
    result += gr::blocklib::initGrFecBlocks(registry);
    result += gr::blocklib::initGrFileIoBlocks(registry);
    result += gr::blocklib::initGrFilterBlocks(registry);
    result += gr::blocklib::initGrFourierBlocks(registry);
    result += gr::blocklib::initGrHttpBlocks(registry);
#if GNURADIO4_HAVE_NETWORK_BLOCKS
    result += gr::blocklib::initGrNetworkBlocks(registry);
#endif
    result += gr::blocklib::initGrSyncBlocks(registry);
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
        expect(registry.contains("gr::blocks::basic::DataSetSink<float32>"sv));
        expect(registry.contains("gr::blocks::basic::DataSetSink<uint8>"sv));
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
        expect(registry.contains("gr::blocks::analog::QuadratureDemod<float32>"sv));
        expect(registry.contains("gr::blocks::analog::PowerSquelch<float32>"sv));
        expect(registry.contains("gr::blocks::analog::CtcssSquelch"sv));
        expect(registry.contains("gr::blocks::channel::AwgnChannel<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::channel::FadingChannel<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::channel::FrequencyOffset<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::channel::IqImbalance<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::channel::PhaseNoise<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::channel::Nonlinearity<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::channel::Quantizer<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::analog::HardLimiter<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::filter::DesignedFilter<float32, float32>"sv));
        expect(registry.contains("gr::blocks::filter::DesignedFilter<complex<float32>, float32>"sv));
        expect(registry.contains("gr::blocks::sync::PreambleTiming<float32>"sv));
        expect(registry.contains("gr::blocks::digital::PamSlicer<float32>"sv));
        expect(registry.contains("gr::blocks::digital::LevelTracker<float32>"sv));
        expect(registry.contains("gr::blocks::sync::PllCarrierTracking"sv));
        expect(registry.contains("gr::blocks::sync::PllFreqDet"sv));
        expect(registry.contains("gr::blocks::sync::PllRefOut"sv));
        expect(registry.contains("gr::blocks::sync::SymbolSync<float32>"sv));
        expect(registry.contains("gr::blocks::sync::CostasLoop"sv));
        expect(registry.contains("gr::blocks::sync::FllBandEdge"sv));
        expect(registry.contains("gr::blocks::basic::Throttle<float32>"sv));
        expect(registry.contains("gr::blocks::filter::fir_filter<float32>"sv));
        expect(registry.contains("gr::blocks::fourier::FFT<float32>"sv));
        // a stated name is the key a flowgraph file writes, and the block type it names binds every argument
        // that name already says, so the key carries the value type alone; the expansion the type resolves to
        // -- here BasicFilterProto<float32> -- stays the primary key
        expect(registry.contains("gr::blocks::filter::BasicFilterProto<float32>"sv));
        expect(registry.contains("gr::blocks::filter::BasicFilter<float32>"sv));
        expect(registry.contains("gr::blocks::filter::BasicDecimatingFilter<float32>"sv));
        expect(registry.contains("gr::blocks::filter::FrequencyEstimatorFrequencyDomainDecimating<float32>"sv));
        expect(registry.contains("gr::blocks::basic::SchmittTrigger<float32>"sv));
        expect(registry.contains("gr::blocks::basic::StreamToDataSet<float32>"sv));
        expect(registry.contains("gr::blocks::electrical::ThreePhasePowerMetrics<float32>"sv));
        expect(registry.contains("gr::blocks::testing::TagSource<float32>"sv));
        expect(registry.contains("gr::blocks::digital::PackBits"sv));
        expect(registry.contains("gr::blocks::digital::UnpackBits"sv));
        expect(registry.contains("gr::blocks::digital::RepackBits"sv));
        expect(registry.contains("gr::blocks::digital::RecordRepackBits"sv));
        expect(registry.contains("gr::blocks::digital::AdditiveScrambler<uint8>"sv));
        expect(registry.contains("gr::blocks::digital::AdditiveScrambler<gr::DataSet<uint8>>"sv));
        expect(registry.contains("gr::blocks::digital::MultiplicativeScrambler"sv));
        expect(registry.contains("gr::blocks::digital::MultiplicativeDescrambler"sv));
        expect(registry.contains("gr::blocks::digital::ManchesterEncoder"sv));
        expect(registry.contains("gr::blocks::digital::ManchesterDecoder"sv));
        expect(registry.contains("gr::blocks::digital::ManchesterCombine"sv));
        expect(registry.contains("gr::blocks::digital::CrcAppend"sv));
        expect(registry.contains("gr::blocks::digital::CrcCheck"sv));
        expect(registry.contains("gr::blocks::digital::AccessCodeCorrelator<float32>"sv));
        expect(registry.contains("gr::blocks::digital::PreambleCorrelator<float32>"sv));
        expect(registry.contains("gr::blocks::digital::PacketFramer<float32>"sv));
        expect(registry.contains("gr::blocks::digital::LengthHeaderFramer"sv));
        expect(registry.contains("gr::blocks::digital::DelimiterExtractor<float32>"sv));
        expect(registry.contains("gr::blocks::digital::DelimiterFramer"sv));
        expect(registry.contains("gr::blocks::fec::FecEncode"sv));
        expect(registry.contains("gr::blocks::fec::FecDecode"sv));
        expect(registry.contains("gr::blocks::fec::RsEncode"sv));
        expect(registry.contains("gr::blocks::fec::RsDecode"sv));
        expect(registry.contains("gr::blocks::fec::ConvEncode"sv));
        expect(registry.contains("gr::blocks::fec::ViterbiDecode"sv));
        expect(registry.contains("gr::blocks::fec::ViterbiDecodeSoft"sv));
        expect(registry.contains("gr::blocks::fec::Puncture"sv));
        expect(registry.contains("gr::blocks::fec::Depuncture"sv));
        expect(registry.contains("gr::blocks::fec::Interleave<uint8>"sv));
        expect(registry.contains("gr::blocks::fec::Deinterleave<uint8>"sv));
        expect(registry.contains("gr::blocks::basic::DataSetToStream<float32>"sv));
        expect(registry.contains("gr::blocks::basic::DataSetToPacket<float32>"sv));
        expect(registry.contains("gr::blocks::basic::PacketToDataSet<float32>"sv));
#if GNURADIO4_HAVE_NETWORK_BLOCKS
        expect(registry.contains("gr::blocks::network::ZmqPacketSink<float32>"sv));
        expect(registry.contains("gr::blocks::network::ZmqPacketSource<float32>"sv));
        expect(registry.contains("gr::blocks::network::ZmqStreamSource<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::network::StreamPacketSink<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::network::StreamPacketSource<complex<float32>>"sv));
        expect(registry.contains("gr::blocks::network::MessagePacketSink"sv));
        expect(registry.contains("gr::blocks::network::MessagePacketSource"sv));
#endif
    };

    "CheckBlockInstantiations"_test = [&] {
        expect(registry.create("gr::blocks::testing::Delay<float32>"sv, {}) != nullptr);
        expect(registry.create("gr::blocks::filter::BasicFilter<float32>"sv, {}) != nullptr);
        expect(registry.create("gr::blocks::filter::BasicDecimatingFilter<float32>"sv, {}) != nullptr);
        expect(registry.create("gr::blocks::filter::FrequencyEstimatorFrequencyDomainDecimating<float32>"sv, {}) != nullptr);
        expect(registry.create("gr::blocks::basic::StreamToDataSet<float32>"sv, {}) != nullptr);
        expect(registry.create("gr::blocks::digital::RepackBits"sv, {}) != nullptr);
        expect(registry.create("gr::blocks::digital::AdditiveScrambler<uint8>"sv, {}) != nullptr);
        expect(registry.create("gr::blocks::digital::ManchesterDecoder"sv, {}) != nullptr);
        expect(registry.create("gr::blocks::basic::DataSetToStream<float32>"sv, {}) != nullptr);
        expect(registry.create("gr::blocks::basic::DataSetToPacket<float32>"sv, {}) != nullptr);
        expect(registry.create("gr::blocks::basic::PacketToDataSet<float32>"sv, {}) != nullptr);
        expect(registry.create("gr::blocks::fileio::WavSource<float32>"sv, {}) != nullptr);
#if GNURADIO4_HAVE_AUDIO_BLOCKS
        expect(registry.create("gr::blocks::audio::AudioSink<float32>"sv, {}) != nullptr);
#endif
        expect(registry.create("gr::blocks::basic::DataSink<float32>"sv, {}) != nullptr);
        expect(registry.create("gr::blocks::channel::AwgnChannel<complex<float32>>"sv, {}) != nullptr);
        expect(registry.create("gr::blocks::basic::ClockSource"sv, {}) != nullptr);
    };
};
int main() { /* not needed for UT */ }
