#include <complex>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/digital/SignalQuality.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::digital::EvmMeter;
using gr::blocks::digital::SnrEstimator;
using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;
using CF     = std::complex<float>;
using Record = gr::DataSet<float>;

/// @brief One call of a meter over the whole input, with room for the records the windows close.
template<typename TBlock>
void drive(TBlock& block, std::span<const CF> input, std::vector<Record>& room) {
    InputSpan<CF>      inSpan(input);
    OutputSpan<Record> outSpan{std::span<Record>(room)};
    std::ignore = block.processBulk(inSpan, outSpan);
}

constexpr std::size_t kSamplesPerCall = 65536UZ;
constexpr std::size_t kRepeats        = 9UZ;

/// Both meters carry a seqlock and are therefore neither copyable nor movable.
template<typename TBlock>
[[nodiscard]] std::unique_ptr<TBlock> make(gr::property_map settings) {
    auto block = std::make_unique<TBlock>(std::move(settings));
    block->settings().init();
    std::ignore = block->settings().applyStagedParameters();
    block->start();
    return block;
}

[[nodiscard]] std::vector<CF> qpskWithNoise() {
    std::vector<CF> data(kSamplesPerCall);
    std::uint64_t   state = 0x243f6a8885a308d3ULL;
    const auto      next  = [&state] {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    };
    for (CF& sample : data) {
        const std::uint64_t bits   = next();
        const float         re     = (bits & 1ULL) ? 0.7071f : -0.7071f;
        const float         im     = (bits & 2ULL) ? 0.7071f : -0.7071f;
        const std::uint64_t jitter = next();
        sample                     = CF(re + 0.05f * (static_cast<float>(jitter % 1024ULL) / 512.f - 1.f), im + 0.05f * (static_cast<float>((jitter >> 20U) % 1024ULL) / 512.f - 1.f));
    }
    return data;
}

} // namespace

int main() {
    const std::vector<CF> input = qpskWithNoise();
    std::vector<Record>   room(kSamplesPerCall / 100UZ + 2UZ);

    auto evmQpsk    = make<EvmMeter<float>>({{"constellation", std::string("qpsk")}, {"window", gr::Size_t{1000U}}});
    auto evm16Qam   = make<EvmMeter<float>>({{"constellation", std::string("qam")}, {"arity", gr::Size_t{16U}}, {"window", gr::Size_t{1000U}}});
    auto snrMoments = make<SnrEstimator<float>>({{"method", std::string("m2m4")}, {"window", gr::Size_t{4096U}}});
    auto snrDecided = make<SnrEstimator<float>>({{"method", std::string("decision_directed")}, {"constellation", std::string("qpsk")}, {"window", gr::Size_t{4096U}}});

    std::vector<gr::blocks::testing::bench::Arm> arms{
        {"EvmMeter, qpsk",
            [&] {
                drive(*evmQpsk, std::span<const CF>(input), room);
                return evmQpsk->evmRms();
            }},
        {"EvmMeter, 16-qam",
            [&] {
                drive(*evm16Qam, std::span<const CF>(input), room);
                return evm16Qam->evmRms();
            }},
        {"SnrEstimator, m2m4",
            [&] {
                drive(*snrMoments, std::span<const CF>(input), room);
                return snrMoments->snrLinear();
            }},
        {"SnrEstimator, decision directed",
            [&] {
                drive(*snrDecided, std::span<const CF>(input), room);
                return snrDecided->snrLinear();
            }},
    };

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kSamplesPerCall, kRepeats);
}
