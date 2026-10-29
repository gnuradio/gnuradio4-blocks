#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <format>
#include <future>
#include <iterator>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <unistd.h>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/analog/PowerMeter.hpp>

#include <gnuradio-4.0/testing/Instrumentation.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {
/// @brief Drives the meter over `input` to exhaustion and returns the records it published. Most cases ignore them.
///
/// The meter takes at most as many samples as the output span has room for the records they close. A scheduler also
/// calls until the span is used up. One call here then means one stretch of stream.
template<typename TBlock, typename T>
std::vector<gr::DataSet<float>> drive(TBlock& block, std::span<const T> input) {
    namespace test = gr::blocks::testing::span;
    std::vector<gr::DataSet<float>> records;

    for (std::size_t base = 0UZ; base < input.size();) {
        std::vector<gr::DataSet<float>>      made(4UZ);
        test::InputSpan<T>                   inSpan(input.subspan(base));
        test::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(made)};
        std::ignore = block.processBulk(inSpan, outSpan);
        records.insert(records.end(), std::make_move_iterator(made.begin()), std::make_move_iterator(made.begin() + static_cast<std::ptrdiff_t>(outSpan.count)));
        if (inSpan.consumed == 0UZ) {
            break;
        }
        base += inSpan.consumed;
    }
    return records;
}

/// @brief What the meter emits when the stream ends here.
///
/// It folds in the trailing samples, then emits the record of the segments closed since the last one.
template<typename TBlock, typename T>
std::vector<gr::DataSet<float>> finish(TBlock& block, std::span<const T> trailing) {
    namespace test = gr::blocks::testing::span;
    std::vector<gr::DataSet<float>>      made(4UZ);
    test::InputSpan<T>                   inSpan(trailing);
    test::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(made)};
    std::ignore = block.processEpilogue(inSpan, outSpan);
    made.resize(outSpan.count);
    return made;
}
} // namespace

namespace {

using gr::blocks::analog::PowerMeter;
using gr::blocks::testing::ProcessFunction;
using gr::blocks::testing::TagSource;

using CF = std::complex<float>;

constexpr double kPi   = std::numbers::pi;
constexpr float  kRate = 96000.f;

/// The block holds atomics and is therefore not movable, so it is built in place and initialized by reference.
template<typename T>
void init(PowerMeter<T>& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
}

template<typename T>
void apply(PowerMeter<T>& block, gr::property_map settings) {
    std::ignore = block.settings().setStaged(std::move(settings));
    std::ignore = block.settings().applyStagedParameters();
}

template<typename T>
void feed(PowerMeter<T>& block, std::span<const T> input, std::size_t chunkSize = 0UZ) {
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    for (std::size_t base = 0UZ; base < input.size(); base += stride) {
        std::ignore = drive(block, input.subspan(base, std::min(stride, input.size() - base)));
    }
}

/// 0.01 normalized, a whole number of periods in a 600-sample segment and in a 9600-sample window alike.
[[nodiscard]] std::vector<CF> complexTone(std::size_t count, double amplitude, double normalized = 0.01) {
    std::vector<CF> samples(count);
    for (std::size_t i = 0UZ; i < count; ++i) {
        const double phase = 2.0 * kPi * normalized * static_cast<double>(i);
        samples[i]         = CF(static_cast<float>(amplitude * std::cos(phase)), static_cast<float>(amplitude * std::sin(phase)));
    }
    return samples;
}

[[nodiscard]] std::vector<float> realTone(std::size_t count, double amplitude, double normalized = 0.01) {
    std::vector<float> samples(count);
    for (std::size_t i = 0UZ; i < count; ++i) {
        samples[i] = static_cast<float>(amplitude * std::cos(2.0 * kPi * normalized * static_cast<double>(i)));
    }
    return samples;
}

struct Random {
    std::uint64_t state = 0x243f6a8885a308d3ULL;

    [[nodiscard]] double uniform() noexcept {
        state += 0x9e3779b97f4a7c15ULL;
        std::uint64_t z = state;
        z               = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z               = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        z ^= z >> 31;
        return static_cast<double>(z >> 11) * 0x1.0p-53;
    }

    [[nodiscard]] double gaussian() noexcept { return std::sqrt(-2.0 * std::log(uniform() + 1e-300)) * std::cos(2.0 * kPi * uniform()); }

    /// Circular complex Gaussian of unit mean power. Each component has variance one half.
    [[nodiscard]] CF sample() noexcept { return CF(static_cast<float>(gaussian() * 0.7071067811865476), static_cast<float>(gaussian() * 0.7071067811865476)); }
};

[[nodiscard]] std::vector<CF> noise(std::size_t count, std::uint64_t seed) {
    Random          rng{seed};
    std::vector<CF> samples(count);
    for (CF& sample : samples) {
        sample = rng.sample();
    }
    return samples;
}

/// @brief Mean and standard deviation of `level()` over @p trials independent windows of @p length noise samples.
[[nodiscard]] std::pair<double, double> levelStatistics(std::size_t length, std::size_t trials, std::uint64_t seed) {
    PowerMeter<CF> block({{"sample_rate", kRate}, {"window_time", static_cast<double>(length) / static_cast<double>(kRate)}, {"segments", 1U}});
    init(block);
    Random          rng{seed};
    std::vector<CF> window(length);

    double total  = 0.0;
    double square = 0.0;
    for (std::size_t trial = 0UZ; trial < trials; ++trial) {
        for (CF& sample : window) {
            sample = rng.sample();
        }
        block.reset();
        std::ignore        = drive(block, std::span<const CF>(window));
        const double level = static_cast<double>(block.level());
        total += level;
        square += level * level;
    }
    const double mean = total / static_cast<double>(trials);
    return {mean, std::sqrt(std::max(0.0, square / static_cast<double>(trials) - mean * mean))};
}

[[nodiscard]] bool longTestsEnabled() { return std::getenv("ENABLE_LONG_TESTS") != nullptr; }

/// @brief psi(n) - ln(n) and psi'(n) by their asymptotic series, to about 1e-12 for n >= 10.
[[nodiscard]] std::pair<double, double> digammaLessLogAndTrigamma(double n) {
    const double inverse = 1.0 / n;
    const double square  = inverse * inverse;
    const double psi     = -0.5 * inverse - square * (1.0 / 12.0 - square * (1.0 / 120.0 - square * (1.0 / 252.0 - square / 240.0)));
    const double psi1    = inverse + 0.5 * square + inverse * square * (1.0 / 6.0 - square * (1.0 / 30.0 - square * (1.0 / 42.0 - square / 30.0)));
    return {psi, psi1};
}

/// @brief Bias and standard deviation in dB of the level of @p n samples of unit-power complex Gaussian noise.
///
/// The sum of `n` values of |x|^2 is Gamma(n, 1). The log of its mean has mean psi(n) - ln(n) and variance psi'(n).
[[nodiscard]] std::pair<double, double> levelClosedForm(std::size_t n) {
    const auto [psi, psi1] = digammaLessLogAndTrigamma(static_cast<double>(n));
    const double toDb      = 10.0 / std::numbers::ln10;
    return {toDb * psi, toDb * std::sqrt(psi1)};
}

[[nodiscard]] std::uint64_t sampleStartOf(const gr::DataSet<float>& record) {
    const auto& meta  = record.meta_information[0UZ];
    const auto  entry = meta.find(std::pmr::string("sample_start"));
    if (entry == meta.end()) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    const auto* index = entry->second.get_if<std::uint64_t>();
    return index == nullptr ? std::numeric_limits<std::uint64_t>::max() : *index;
}

[[nodiscard]] std::uint64_t segmentsOf(const gr::DataSet<float>& record) {
    const auto& meta  = record.meta_information[0UZ];
    const auto  entry = meta.find(std::pmr::string("n_segments"));
    if (entry == meta.end()) {
        return 0ULL;
    }
    const auto* count = entry->second.get_if<std::uint64_t>();
    return count == nullptr ? 0ULL : *count;
}

[[nodiscard]] std::uint64_t windowSamplesOf(const gr::DataSet<float>& record) {
    const auto& meta  = record.meta_information[0UZ];
    const auto  entry = meta.find(std::pmr::string("window_samples"));
    if (entry == meta.end()) {
        return 0ULL;
    }
    const auto* count = entry->second.get_if<std::uint64_t>();
    return count == nullptr ? 0ULL : *count;
}

/// Ends the stream after `count` samples of unit power with no room for a record. Returns how many records went out.
///
/// The record of each window the samples close waits in the meter. The next call publishes it first.
std::size_t endWithNoRoom(PowerMeter<CF>& block, std::size_t count) {
    namespace test = gr::blocks::testing::span;
    const std::vector<CF>                x(count, CF(1.f, 0.f));
    std::vector<gr::DataSet<float>>      none;
    test::InputSpan<CF>                  tail{std::span<const CF>(x)};
    test::OutputSpan<gr::DataSet<float>> full{std::span<gr::DataSet<float>>(none)};
    std::ignore = block.processEpilogue(tail, full);
    return full.count;
}

/// @brief Sends file descriptor 2 to a temporary file until `release()`. `text()` reads what arrived.
class StderrCapture {
    std::FILE* _file  = std::tmpfile();
    int        _saved = -1;

public:
    StderrCapture() {
        if (_file != nullptr) {
            std::fflush(stderr);
            _saved = ::dup(STDERR_FILENO);
            ::dup2(::fileno(_file), STDERR_FILENO);
        }
    }
    StderrCapture(const StderrCapture&)            = delete;
    StderrCapture& operator=(const StderrCapture&) = delete;
    ~StderrCapture() {
        release();
        if (_file != nullptr) {
            std::fclose(_file);
        }
    }

    void release() {
        if (_saved >= 0) {
            std::fflush(stderr);
            ::dup2(_saved, STDERR_FILENO);
            ::close(_saved);
            _saved = -1;
        }
    }

    [[nodiscard]] std::string text() const {
        std::string out;
        if (_file == nullptr) {
            return out;
        }
        std::fflush(stderr);
        std::array<char, 4096UZ> chunk{};
        for (off_t at = 0;;) {
            const ssize_t n = ::pread(::fileno(_file), chunk.data(), chunk.size(), at);
            if (n <= 0) {
                return out;
            }
            out.append(chunk.data(), static_cast<std::size_t>(n));
            at += n;
        }
    }
};

} // namespace

// In an anonymous namespace, clang reports the friend function GR_MAKE_REFLECTABLE declares as unneeded.
namespace qa_power_meter {

/// Shared between a `HeldSource` and the test thread.
struct Hold {
    std::atomic<std::size_t> calls{0UZ};
    std::atomic<bool>        released{false};
};

/// @brief Publishes nothing and answers `OK` until released, then `count` samples of unit power and `DONE`.
///
/// While held it draws the runtime's report of a block that answers `OK` for a second without moving a sample.
struct HeldSource : gr::Block<HeldSource> {
    gr::PortOut<CF> out;
    GR_MAKE_REFLECTABLE(HeldSource, out);

    std::shared_ptr<Hold> hold;
    std::size_t           count = 0UZ;
    std::size_t           _sent = 0UZ;

    gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        hold->calls.fetch_add(1UZ);
        if (!hold->released.load()) {
            outSpan.publish(0UZ);
            return gr::work::Status::OK;
        }
        const std::size_t n = std::min(outSpan.size(), count - _sent);
        for (std::size_t k = 0UZ; k < n; ++k) {
            outSpan[k] = CF(1.f, 0.f);
        }
        _sent += n;
        outSpan.publish(n);
        return _sent >= count ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

/// @brief Publishes `count` samples, the first `split` of them at 0 dBFS and the rest at -20 dBFS, then `DONE`.
struct LevelSource : gr::Block<LevelSource> {
    gr::PortOut<CF> out;
    GR_MAKE_REFLECTABLE(LevelSource, out);

    std::size_t count = 0UZ;
    std::size_t split = 0UZ;
    std::size_t _sent = 0UZ;

    gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(outSpan.size(), count - _sent);
        for (std::size_t k = 0UZ; k < n; ++k) {
            outSpan[k] = _sent + k < split ? CF(1.f, 0.f) : CF(0.1f, 0.f);
        }
        _sent += n;
        outSpan.publish(n);
        return _sent >= count ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

/// @brief Keeps the records that reach it and counts the tags that arrive with them.
struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<gr::DataSet<float>> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);

    std::vector<gr::DataSet<float>> received;
    std::size_t                     tags = 0UZ;

    gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        tags += inSpan.rawTags.size();
        for (const auto& record : inSpan) {
            received.push_back(record);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/// @brief Counts the records that reach it.
struct RecordCounter : gr::Block<RecordCounter> {
    gr::PortIn<gr::DataSet<float>> in;
    GR_MAKE_REFLECTABLE(RecordCounter, in);

    std::size_t received = 0UZ;

    gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        received += inSpan.size();
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

} // namespace qa_power_meter

const boost::ut::suite<"PowerMeter"> powerMeterTests = [] {
    using namespace boost::ut;
    using namespace qa_power_meter;

    "the dBFS anchors are exact"_test = [] {
        struct Case {
            const char* what;
            double      amplitude;
            bool        isTone;
            double      want;
        };
        constexpr Case kCases[]{
            {"complex tone A=1.0", 1.0, true, 0.0},
            {"complex tone A=0.5", 0.5, true, -6.0206},
            {"complex tone A=0.1", 0.1, true, -20.0},
            {"complex DC A=1", 1.0, false, 0.0},
            {"complex DC A=0.25", 0.25, false, -12.0412},
        };
        for (const auto& [what, amplitude, isTone, want] : kCases) {
            PowerMeter<CF> block({{"sample_rate", kRate}});
            init(block);
            const std::vector<CF> x = isTone ? complexTone(9600UZ, amplitude) : std::vector<CF>(9600UZ, CF(static_cast<float>(amplitude), 0.f));
            feed<CF>(block, std::span<const CF>(x), 997UZ);
            expect(approx(static_cast<double>(block.level()), want, 1e-4)) << what;
        }

        PowerMeter<CF> rails({{"sample_rate", kRate}});
        init(rails);
        feed<CF>(rails, std::span<const CF>(std::vector<CF>(9600UZ, CF(1.f, 1.f))), 997UZ);
        expect(approx(static_cast<double>(rails.level()), 3.0103, 1e-4)) << "both rails at once is |x|^2 = 2, and the meter reads above 0 dBFS rather than clamping at it";
    };

    "a real sinusoid reads 3.0103 dB below the complex one, and a real DC reads the same"_test = [] {
        for (const double amplitude : {1.0, 0.5}) {
            PowerMeter<float> real({{"sample_rate", kRate}});
            init(real);
            const std::vector<float> x = realTone(9600UZ, amplitude);
            feed<float>(real, std::span<const float>(x), 997UZ);
            expect(approx(static_cast<double>(real.level()), 20.0 * std::log10(amplitude) - 3.0103, 1e-4)) << std::format("real tone A={}", amplitude);

            PowerMeter<CF> promoted({{"sample_rate", kRate}});
            init(promoted);
            std::vector<CF> lifted(x.size());
            std::ranges::transform(x, lifted.begin(), [](float sample) { return CF(sample, 0.f); });
            feed<CF>(promoted, std::span<const CF>(lifted), 997UZ);
            expect(eq(promoted.level(), real.level())) << "the same waveform through the complex port, bit for bit: there is no stray factor of two";
        }

        PowerMeter<float> dc({{"sample_rate", kRate}});
        init(dc);
        feed<float>(dc, std::span<const float>(std::vector<float>(9600UZ, 0.5f)), 997UZ);
        expect(approx(static_cast<double>(dc.level()), 20.0 * std::log10(0.5), 1e-4)) << "a real DC reads what a complex DC of the same amplitude reads";
    };

    "the window fills segment by segment and is already correct at the first"_test = [] {
        PowerMeter<CF> block({{"sample_rate", kRate}, {"window_time", 0.1}, {"segments", 16U}});
        init(block);
        expect(eq(block.segment_samples.value, 600U));
        expect(eq(block.window_samples.value, 9600U));
        expect(eq(block.coverage(), 0.f)) << "nothing accumulated yet";
        expect(eq(block.level(), static_cast<float>(block.floor_db.value))) << "and the floor rather than 0 dBFS, which is full scale";

        const std::vector<CF> x = complexTone(9600UZ, 1.0);
        feed<CF>(block, std::span<const CF>(x).first(599UZ));
        expect(eq(block.coverage(), 0.f)) << "still short of the first segment";

        for (std::size_t k = 1UZ; k <= 16UZ; ++k) {
            feed<CF>(block, std::span<const CF>(x).subspan(k == 1UZ ? 599UZ : (k - 1UZ) * 600UZ, k == 1UZ ? 1UZ : 600UZ));
            expect(approx(static_cast<double>(block.coverage()), static_cast<double>(k) / 16.0, 1e-6)) << std::format("after {} segments", k);
            expect(approx(static_cast<double>(block.level()), 0.0, 0.01)) << "the partial window is already the right reading for a deterministic signal";
        }
        expect(eq(block.coverage(), 1.f));
    };

    "the window is a boxcar, not a pole"_test = [] {
        PowerMeter<CF> block({{"sample_rate", kRate}});
        init(block);
        feed<CF>(block, std::span<const CF>(complexTone(9600UZ, 1.0)), 600UZ);
        expect(approx(static_cast<double>(block.level()), 0.0, 1e-4));

        const std::vector<CF> silence(600UZ, CF{});
        for (std::size_t retired = 1UZ; retired <= 16UZ; ++retired) {
            feed<CF>(block, std::span<const CF>(silence));
            const double remaining = static_cast<double>(16UZ - retired);
            if (remaining > 0.0) {
                expect(approx(static_cast<double>(block.level()), 10.0 * std::log10(remaining / 16.0), 0.01)) << std::format("{} of 16 segments still hold signal", remaining);
            } else {
                expect(eq(block.level(), static_cast<float>(block.floor_db.value))) << "exactly window_samples later the reading is the floor; a single pole never gets there";
            }
        }
    };

    "the partial-window statistics are the closed form"_test = [] {
        std::vector<std::size_t> lengths{100UZ};
        if (longTestsEnabled()) {
            lengths.insert(lengths.begin(), 10UZ);
            lengths.push_back(1000UZ);
        }
        constexpr std::size_t kTrials = 20000UZ;

        for (const std::size_t length : lengths) {
            const auto [bias, sigma]                 = levelClosedForm(length);
            const auto [measuredBias, measuredSigma] = levelStatistics(length, kTrials, 0x13198a2e03707344ULL + length);
            const double standardError               = sigma / std::sqrt(static_cast<double>(kTrials));
            expect(lt(std::abs(measuredBias - bias), 3.0 * standardError)) << std::format("N={}: bias {:.4f} against {:.4f}", length, measuredBias, bias);
            expect(lt(std::abs(measuredSigma - sigma), 3.0 * sigma / std::sqrt(2.0 * static_cast<double>(kTrials)))) << std::format("N={}: sigma {:.4f} against {:.4f}", length, measuredSigma, sigma);
        }
    };

    "the full-window spread is the closed form, 0.0443 dB"_test = [] {
        constexpr std::size_t kTrials            = 400UZ;
        const auto [bias, sigma]                 = levelClosedForm(9600UZ);
        const auto [measuredBias, measuredSigma] = levelStatistics(9600UZ, kTrials, 0xa4093822299f31d0ULL);
        const double sigmaError                  = sigma / std::sqrt(2.0 * static_cast<double>(kTrials));
        std::println("full window: bias {:.5f} dB, sigma {:.5f} dB over {} trials, closed form {:.5f} and {:.5f}", measuredBias, measuredSigma, kTrials, bias, sigma);
        expect(approx(sigma, 0.0443, 1e-4)) << "the closed form at 9600 samples";
        expect(lt(std::abs(measuredSigma - sigma), 3.0 * sigmaError)) << "the number a user sees on a 100 ms window";
        expect(lt(std::abs(measuredBias - bias), 3.0 * sigma / std::sqrt(static_cast<double>(kTrials)))) << "and its bias is the closed form's, a few thousandths of a dB";
    };

    "the reading does not depend on the chunking"_test = [] {
        constexpr std::size_t kLength = 30000UZ;
        const std::vector<CF> x       = noise(kLength, 0x082efa98ec4e6c89ULL);

        PowerMeter<CF> reference({{"sample_rate", kRate}});
        init(reference);
        std::vector<double> want(kLength);
        for (std::size_t i = 0UZ; i < kLength; ++i) {
            std::ignore = drive(reference, std::span<const CF>(x).subspan(i, 1UZ));
            want[i]     = reference.linear_power();
        }

        for (const std::size_t chunk : {3UZ, 17UZ, 600UZ, 601UZ, 4096UZ}) {
            PowerMeter<CF> block({{"sample_rate", kRate}});
            init(block);
            std::size_t fed   = 0UZ;
            bool        equal = true;
            while (fed < kLength) {
                const std::size_t count = std::min(chunk, kLength - fed);
                std::ignore             = drive(block, std::span<const CF>(x).subspan(fed, count));
                fed += count;
                equal = equal && block.linear_power() == want[fed - 1UZ];
            }
            expect(that % equal) << std::format("chunk {}: the segment boundary is an absolute offset, so the reading is bit-identical", chunk);
            expect(eq(block.level(), reference.level()));
        }
    };

    "no drift over a hundred million samples"_test = [] {
        constexpr std::size_t kBuffer = 600UZ * 1748UZ; // a whole number of segments, so repeated passes keep the segment boundaries aligned
        constexpr std::size_t kPasses = 96UZ;

        PowerMeter<CF> block({{"sample_rate", kRate}});
        init(block);
        const std::vector<CF> x = noise(kBuffer, 0x2ff24fa2ee84e39cULL);
        for (std::size_t pass = 0UZ; pass < kPasses; ++pass) {
            feed<CF>(block, std::span<const CF>(x), 4096UZ);
        }

        double fresh = 0.0;
        for (const CF& sample : std::span<const CF>(x).last(9600UZ)) {
            fresh += static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
        }
        fresh /= 9600.0;
        std::println("after {} samples: linear_power {:.15g} against a fresh mean of {:.15g}", kBuffer * kPasses, block.linear_power(), fresh);
        expect(lt(std::abs(block.linear_power() - fresh) / fresh, 1e-12)) << "nothing is ever subtracted, so there is no residue to accumulate";

        // A single 9600-sample window of noise has a 0.0443 dB spread of its own. The 0 dB check therefore needs a
        // signal whose power is not a random variable. The drift is checked above.
        PowerMeter<CF> tone({{"sample_rate", kRate}});
        init(tone);
        const std::vector<CF> steady = complexTone(kBuffer, 1.0);
        for (std::size_t pass = 0UZ; pass < kPasses; ++pass) {
            feed<CF>(tone, std::span<const CF>(steady), 4096UZ);
        }
        expect(lt(std::abs(static_cast<double>(tone.level())), 0.01)) << "and a unit tone still reads 0.000 dBFS a hundred million samples later";
    };

    "the floor is a clamp and not an epsilon"_test = [] {
        const auto readAt = [](double power, double floorDb) {
            PowerMeter<CF> block({{"sample_rate", kRate}, {"floor_db", floorDb}});
            init(block);
            feed<CF>(block, std::span<const CF>(std::vector<CF>(9600UZ, CF(static_cast<float>(std::sqrt(power)), 0.f))), 600UZ);
            return static_cast<double>(block.level());
        };

        expect(approx(readAt(1e-19, -200.0), -190.0, 1e-3)) << "an additive epsilon of 1e-20 would read -189.586 instead";
        expect(approx(readAt(1e-18, -200.0), -180.0, 1e-3));
        expect(approx(readAt(1e-16, -140.0), -140.0, 1e-9)) << "and below the floor the answer is exactly the floor";

        PowerMeter<CF> silent({{"sample_rate", kRate}});
        init(silent);
        feed<CF>(silent, std::span<const CF>(std::vector<CF>(9600UZ, CF{})), 600UZ);
        expect(eq(silent.level(), -200.f)) << "a true power of zero is the floor rather than negative infinity";
    };

    "a settings change resets the window, and floor_db alone does not"_test = [] {
        PowerMeter<CF> block({{"sample_rate", kRate}});
        init(block);
        feed<CF>(block, std::span<const CF>(complexTone(9600UZ, 1.0)), 600UZ);
        expect(eq(block.coverage(), 1.f));

        for (const gr::property_map& change : {gr::property_map{{"sample_rate", 48000.f}}, gr::property_map{{"window_time", 0.05}}, gr::property_map{{"segments", 8U}}}) {
            PowerMeter<CF> filled({{"sample_rate", kRate}});
            init(filled);
            feed<CF>(filled, std::span<const CF>(complexTone(9600UZ, 1.0)), 600UZ);
            apply<CF>(filled, change);
            expect(eq(filled.coverage(), 0.f)) << "a window rescaled into a different shape holds neither the old nor the new average";
            expect(eq(filled.level(), -200.f));
        }

        apply<CF>(block, {{"floor_db", -140.0}});
        expect(eq(block.coverage(), 1.f)) << "floor_db alone changes the clamp and nothing else";
        expect(approx(static_cast<double>(block.level()), 0.0, 1e-4));
    };

    "degenerate settings"_test = [] {
        expect(throws([] {
            PowerMeter<CF> rejected({{"sample_rate", 0.f}});
            init(rejected);
        }));
        expect(throws([] {
            PowerMeter<CF> rejected({{"sample_rate", -1.f}});
            init(rejected);
        }));
        expect(throws([] {
            PowerMeter<CF> rejected({{"sample_rate", std::numeric_limits<float>::infinity()}});
            init(rejected);
        }));
        expect(throws([] {
            PowerMeter<CF> rejected({{"window_time", 0.0}});
            init(rejected);
        }));
        expect(throws([] {
            PowerMeter<CF> rejected({{"window_time", -0.1}});
            init(rejected);
        }));
        expect(throws([] {
            PowerMeter<CF> rejected({{"segments", 0U}});
            init(rejected);
        }));

        PowerMeter<CF> block({{"sample_rate", kRate}});
        init(block);
        std::ignore       = block.settings().setStaged({{"segments", 0U}});
        const auto staged = block.settings().applyStagedParameters();
        expect(staged.failedParameters.contains(gr::property_map::key_type{"segments"})) << "the limits refuse a live change too";
        expect(eq(block.segments.value, 16U)) << "and the member keeps the accepted count";

        PowerMeter<CF> tiny({{"sample_rate", kRate}, {"window_time", 1e-9}, {"segments", 16U}});
        init(tiny);
        expect(eq(tiny.segment_samples.value, 1U)) << "a window too short for a segment is clamped to one sample";
        expect(eq(tiny.window_samples.value, 16U)) << "and the realized window is reported rather than silently wrong";

        PowerMeter<CF> single({{"sample_rate", kRate}, {"segments", 1U}});
        init(single);
        feed<CF>(single, std::span<const CF>(complexTone(9600UZ, 1.0)), 600UZ);
        expect(approx(static_cast<double>(single.level()), 0.0, 1e-4)) << "one segment is an integrate-and-dump and is legal";
    };

    "a window past 2^32 samples is reported whole, and one past 2^53 samples is refused"_test = [] {
        constexpr double kRateHz = 1e6;
        for (const double seconds : {4000.0, 5000.0}) {
            PowerMeter<CF> block({{"sample_rate", static_cast<float>(kRateHz)}, {"window_time", seconds}, {"segments", gr::Size_t{16U}}});
            init(block);
            const auto want = static_cast<std::uint64_t>(kRateHz * seconds);
            expect(eq(block.window_samples.value, want)) << std::format("{} s at {} Hz is {} samples", seconds, kRateHz, want);
            expect(eq(block.segment_samples.value * 16ULL, want)) << "the window is 16 whole segments";
            expect(eq(windowSamplesOf(block.makeRecord(1.0, 16U)), want)) << "and a record states the same window";
        }

        // At 1 Hz the window in samples equals window_time. 2^53 + 2 is the next double above 2^53.
        constexpr double kBound = 9007199254740992.0;
        PowerMeter<CF>   edge({{"sample_rate", 1.f}, {"window_time", kBound}, {"segments", gr::Size_t{16U}}});
        init(edge);
        expect(eq(edge.window_samples.value, static_cast<std::uint64_t>(kBound))) << "a window of 2^53 samples applies";
        expect(throws([] {
            PowerMeter<CF> rejected({{"sample_rate", 1.f}, {"window_time", kBound + 2.0}});
            init(rejected);
        })) << "one sample step past 2^53 is refused";

        PowerMeter<CF> block({{"sample_rate", static_cast<float>(kRateHz)}});
        init(block);
        const std::uint64_t window = block.window_samples.value;
        for (const double seconds : {1e10, 1e300}) {
            expect(throws([&block, seconds] { apply<CF>(block, {{"window_time", seconds}}); })) << std::format("{} s at {} Hz is refused", seconds, kRateHz);
            expect(eq(block.window_samples.value, window)) << "and the meter keeps its window";
        }
        expect(nothrow([&block] { apply<CF>(block, {{"floor_db", -90.0}}); })) << "the next change of another setting applies";
        expect(eq(block.window_samples.value, window));
    };

    "tags arrive at the sink, change nothing and cause no error"_test = [] {
        const auto readingOf = [](bool withTags) {
            gr::Graph graph;
            auto&     source = graph.emplaceBlock<TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 9600U}, {"mark_tag", false}});
            if (withTags) {
                source._tags.emplace_back(0UZ, gr::property_map{{gr::property_map::key_type{"trigger_name"}, gr::pmt::Value(std::string("alpha"))}});
                source._tags.emplace_back(1UZ, gr::property_map{{gr::property_map::key_type{"trigger_time"}, gr::pmt::Value(std::uint64_t{111})}});
                source._tags.emplace_back(1UZ, gr::property_map{{gr::property_map::key_type{"trigger_offset"}, gr::pmt::Value(0.5f)}});
                source._tags.emplace_back(37UZ, gr::property_map{{gr::property_map::key_type{"num_channels"}, gr::pmt::Value(gr::Size_t{3})}});
                source._tags.emplace_back(512UZ, gr::property_map{{gr::property_map::key_type{"rx_overflow"}, gr::pmt::Value(true)}});
                source._tags.emplace_back(900UZ, gr::property_map{{gr::property_map::key_type{"signal_name"}, gr::pmt::Value(std::string("iq"))}});
                source._tags.emplace_back(1200UZ, gr::property_map{{gr::property_map::key_type{"t0"}, gr::pmt::Value(std::string("private"))}});
            }

            auto& meter = graph.emplaceBlock<PowerMeter<CF>>({{"sample_rate", kRate}, {"window_time", 0.01}});
            auto& sink  = graph.emplaceBlock<RecordSink>();
            boost::ut::expect(graph.connect<"out", "in">(source, meter).has_value());
            boost::ut::expect(graph.connect<"records", "in">(meter, sink).has_value());

            gr::scheduler::Simple scheduler;
            boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
            boost::ut::expect(scheduler.runAndWait().has_value());
            return std::tuple{meter.level(), meter.coverage(), sink.received.size(), sink.tags};
        };

        const auto [taggedLevel, taggedCoverage, taggedRecords, taggedTags] = readingOf(true);
        const auto [plainLevel, plainCoverage, plainRecords, plainTags]     = readingOf(false);
        expect(eq(taggedLevel, plainLevel)) << "the meter interprets no tag: the reading with tags equals the one without";
        expect(eq(taggedCoverage, plainCoverage));
        expect(eq(taggedCoverage, 1.f)) << "and the run was long enough for the window to fill";
        expect(eq(taggedRecords, 10UZ)) << "ten windows of 960 samples, ten records";
        expect(eq(plainRecords, taggedRecords));
        expect(eq(taggedTags, 0UZ)) << "and the input tags stop at the meter: none reaches the records port";
        expect(eq(plainTags, 0UZ));
    };

    "the logarithm is not on the sample path"_test = [] {
        // The case measures the cost of the code the compiler generates for the meter, as a ratio to the accumulation
        // alone. An unoptimized or sanitizer-instrumented build generates different code and charges the two arms
        // differently. There the ratio is not the meter's.
        if (!gr::blocks::testing::kCostMeasurable) {
            std::println("the per-sample cost is measured only in an optimized build without a sanitizer");
            return;
        }

        using Clock                   = std::chrono::steady_clock;
        constexpr std::size_t kLength = 1UZ << 22;
        constexpr std::size_t kChunk  = 4096UZ;
        constexpr int         kRuns   = 7;

        const std::vector<CF> x = noise(kLength, 0x243f6a8885a308d3ULL);

        PowerMeter<CF> narrow({{"sample_rate", kRate}});
        init(narrow); // 600-sample segments
        PowerMeter<CF> wide({{"sample_rate", 25e6f}});
        init(wide); // 156250-sample segments
        double sink = 0.0;

        const auto sweep = [&x, &sink](PowerMeter<CF>& meter, bool poll) {
            for (std::size_t base = 0UZ; base < kLength; base += kChunk) {
                std::ignore = drive(meter, std::span<const CF>(x).subspan(base, kChunk));
                if (poll && (base / kChunk) % 24UZ == 0UZ) { // 10 Hz against 4096-sample calls at 96 kHz
                    sink += static_cast<double>(meter.level());
                }
            }
        };

        const std::array<std::pair<const char*, std::function<void()>>, 4UZ> arms{{
            {"|x|^2 accumulation only",
                [&x, &sink] {
                    double total = 0.0;
                    for (const CF& sample : x) {
                        total += static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
                    }
                    sink += total;
                }},
            {"PowerMeter, 96 kHz window", [&sweep, &narrow] { sweep(narrow, false); }},
            {"PowerMeter, 25 MS/s window", [&sweep, &wide] { sweep(wide, false); }},
            {"PowerMeter, 96 kHz, polled at 10 Hz", [&sweep, &narrow] { sweep(narrow, true); }},
        }};

        std::vector<double> best(arms.size(), 1e300);
        std::vector<double> worst(arms.size(), 0.0);
        for (int repeat = 0; repeat <= kRuns; ++repeat) {
            for (std::size_t arm = 0UZ; arm < arms.size(); ++arm) {
                const auto start = Clock::now();
                arms[arm].second();
                const double ns = std::chrono::duration<double, std::nano>(Clock::now() - start).count() / static_cast<double>(kLength);
                if (repeat > 0) {
                    best[arm]  = std::min(best[arm], ns);
                    worst[arm] = std::max(worst[arm], ns);
                }
            }
        }
        for (std::size_t arm = 0UZ; arm < arms.size(); ++arm) {
            std::println("{:<38} {:7.3f} ns/sample (spread {:.3f})", arms[arm].first, best[arm], worst[arm] - best[arm]);
        }
        std::println("[checksum {:g}]", sink);

        // With DISABLE_SENSITIVE_TESTS set, the test prints the figures and asserts nothing. With
        // ENABLE_BENCHMARK_TESTS set, it asserts the tight 1.5x bound. Otherwise it asserts the loose 3.0x bound. The
        // tight bound needs a harness that controls the run. An ordinary ctest run measures the scheduler as much as
        // this code. Either bound still catches a per-sample log10, which costs tens of times the accumulation and not
        // tens of percent.
        if (std::getenv("DISABLE_SENSITIVE_TESTS") != nullptr) {
            std::println("DISABLE_SENSITIVE_TESTS is set: the figures are printed and not asserted on");
            return;
        }
        const double bound = std::getenv("ENABLE_BENCHMARK_TESTS") != nullptr ? 1.5 : 3.0;

        expect(lt(best[1UZ] / best[0UZ], bound)) << "the 96 kHz window costs what accumulating |x|^2 costs";
        expect(lt(best[2UZ] / best[0UZ], bound)) << "and so does the 25 MS/s window: the memory is 16 doubles at either rate";
        expect(lt(best[3UZ] / best[1UZ], bound)) << "and polling level() at 10 Hz does not change it";
    };

    "the record port publishes a completed window, and an unconnected one publishes nothing"_test = [] {
        constexpr gr::Size_t kSegments = 4U;
        PowerMeter<CF>       block({{"sample_rate", kRate}, {"window_time", 0.001}, {"segments", kSegments}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();

        // two whole windows of a unit-power tone, so a record is due and its value is known
        const std::size_t window = static_cast<std::size_t>(block.window_samples.value);
        std::vector<CF>   x(2UZ * window, CF(1.f, 0.f));
        const auto        records = drive(block, std::span<const CF>(x));

        expect(!records.empty()) << "a completed window publishes a record";
        if (records.empty()) {
            return;
        }
        const auto& record = records.front();
        expect(eq(record.signal_names.size(), 3UZ));
        expect(eq(record.signal_names[0UZ], std::string("power")));
        expect(eq(record.signal_units[0UZ], std::string("dBFS")));
        expect(std::abs(record.signal_values[0UZ] - 0.f) < 1e-3f) << "a unit-power tone reads 0 dBFS";
        expect(std::abs(record.signal_values[1UZ] - 1.f) < 1e-3f) << "and 1 in linear power";
        const auto& meta = record.meta_information[0UZ];
        expect(meta.find(std::pmr::string("sample_rate")) != meta.end()) << "a sample-domain meter states its rate";
        expect(meta.find(std::pmr::string("sample_start")) != meta.end());

        PowerMeter<CF> bare({{"sample_rate", kRate}, {"window_time", 0.001}, {"segments", kSegments}});
        bare.settings().init();
        std::ignore = bare.settings().applyStagedParameters();
        {
            namespace test        = gr::blocks::testing::span;
            std::size_t published = 0UZ;
            for (std::size_t base = 0UZ; base < x.size();) {
                std::vector<gr::DataSet<float>>      made(4UZ);
                test::InputSpan<CF>                  inSpan{std::span<const CF>(x).subspan(base)};
                test::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(made), 0UZ, nullptr, false};
                std::ignore = bare.processBulk(inSpan, outSpan);
                published += outSpan.count;
                if (inSpan.consumed == 0UZ) {
                    break;
                }
                base += inSpan.consumed;
            }
            expect(eq(published, 0UZ)) << "an unconnected port has nothing published to it";
        }
        expect(eq(block.linear_power(), bare.linear_power())) << "and the reading does not depend on whether anyone is listening";
    };

    "several windows closing in one call are several records, each stamped at its own first sample"_test = [] {
        constexpr gr::Size_t  kSegments = 4U;
        constexpr std::size_t kWindows  = 8UZ;
        PowerMeter<CF>        block({{"sample_rate", kRate}, {"window_time", 0.001}, {"segments", kSegments}});
        init(block);

        const std::size_t window = static_cast<std::size_t>(block.window_samples.value);
        std::vector<CF>   x(kWindows * window, CF(1.f, 0.f));

        namespace test = gr::blocks::testing::span;
        std::vector<gr::DataSet<float>>      made(16UZ);
        test::InputSpan<CF>                  inSpan{std::span<const CF>(x)};
        test::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(made)};
        std::ignore = block.processBulk(inSpan, outSpan);

        expect(eq(outSpan.count, kWindows)) << "eight windows closed inside the one call and each is its own record";
        for (std::size_t w = 0UZ; w < std::min(outSpan.count, kWindows); ++w) {
            const auto& meta  = made[w].meta_information[0UZ];
            const auto  entry = meta.find(std::pmr::string("sample_start"));
            expect(entry != meta.end()) << "record " << w << " states no place in the stream";
            if (entry != meta.end()) {
                const auto* index = entry->second.get_if<std::uint64_t>();
                expect(index != nullptr && *index == w * window) << "record " << w << " is stamped at its own window's first sample";
            }
        }
    };

    "a stream ending mid-window emits a final record of the segments closed since the last record"_test = [] {
        constexpr gr::Size_t kSegments = 4U;
        PowerMeter<CF>       block({{"sample_rate", kRate}, {"window_time", 0.001}, {"segments", kSegments}});
        init(block);

        const std::size_t window  = static_cast<std::size_t>(block.window_samples.value);
        const std::size_t segment = static_cast<std::size_t>(block.segment_samples.value);
        std::vector<CF>   x(2UZ * window, CF(1.f, 0.f));
        x.insert(x.end(), 2UZ * segment + segment / 2UZ, CF(0.1f, 0.f)); // two segments at -20 dBFS and half of a third

        const auto during = drive(block, std::span<const CF>(x));
        expect(eq(during.size(), 2UZ)) << "two whole windows closed while the stream ran";

        const auto last = finish(block, std::span<const CF>{});
        expect(eq(last.size(), 1UZ)) << "and the segments closed after them are reported rather than discarded";
        if (last.empty()) {
            return;
        }
        expect(approx(static_cast<double>(last.front().signal_values[0UZ]), -20.0, 1e-4)) << "the record states the tail's level alone";
        expect(approx(static_cast<double>(last.front().signal_values[2UZ]), 0.5, 1e-6)) << "and its coverage, two segments of four";
        expect(eq(segmentsOf(last.front()), 2ULL));
        expect(eq(sampleStartOf(last.front()), 2UZ * window)) << "starting where the last whole window ended";
        expect(gt(static_cast<double>(block.level()), -3.0)) << "while the rolling reading still holds the previous window";
        expect(finish(block, std::span<const CF>{}).empty()) << "a second end of stream emits nothing";
    };

    "a stream ending with no segment closed since the last record emits no final record"_test = [] {
        for (const std::size_t extra : {0UZ, 5UZ}) {
            PowerMeter<CF> block({{"sample_rate", kRate}, {"window_time", 0.001}, {"segments", gr::Size_t{4U}}});
            init(block);
            const std::size_t     window = static_cast<std::size_t>(block.window_samples.value);
            const std::vector<CF> x(window + extra, CF(1.f, 0.f));
            expect(eq(drive(block, std::span<const CF>(x)).size(), 1UZ));
            expect(finish(block, std::span<const CF>{}).empty()) << std::format("{} samples past the record: no segment closed, no record", extra);
        }
    };

    "the end-of-stream record reaches a connected sink through a scheduler"_test = [] {
        gr::Graph graph;
        auto&     source = graph.emplaceBlock<LevelSource>();
        source.count     = 2400UZ;
        source.split     = 1920UZ;
        auto& meter      = graph.emplaceBlock<PowerMeter<CF>>({{"sample_rate", kRate}, {"window_time", 0.01}, {"segments", gr::Size_t{4U}}});
        auto& sink       = graph.emplaceBlock<RecordSink>();
        expect(graph.connect<"out", "in">(source, meter).has_value());
        expect(graph.connect<"records", "in">(meter, sink).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        expect(eq(sink.received.size(), 3UZ)) << "two windows of 960 samples, then the final record of the 480 after them";
        if (sink.received.size() != 3UZ) {
            return;
        }
        for (std::size_t w = 0UZ; w < 2UZ; ++w) {
            expect(approx(static_cast<double>(sink.received[w].signal_values[0UZ]), 0.0, 1e-4));
            expect(eq(sampleStartOf(sink.received[w]), 960UZ * w));
        }
        const auto& last = sink.received[2UZ];
        expect(approx(static_cast<double>(last.signal_values[0UZ]), -20.0, 1e-4)) << "the tail at -20 dBFS";
        expect(approx(static_cast<double>(last.signal_values[2UZ]), 0.5, 1e-6)) << "covering half the window";
        expect(eq(sampleStartOf(last), 1920ULL));
    };

    "a call takes every sample it is offered"_test = [] {
        namespace test = gr::blocks::testing::span;
        PowerMeter<CF> block({{"sample_rate", kRate}, {"window_time", 0.001}, {"segments", gr::Size_t{4U}}});
        init(block);
        const std::vector<CF>                x(777UZ, CF(1.f, 0.f));
        std::vector<gr::DataSet<float>>      made(16UZ);
        test::InputSpan<CF>                  inSpan{std::span<const CF>(x)};
        test::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(made)};
        expect(block.processBulk(inSpan, outSpan) == gr::work::Status::OK);
        expect(eq(inSpan.consumed, x.size())) << "nothing is held back for the end-of-stream epilogue";
        expect(eq(outSpan.count, 8UZ)) << "and the eight windows it closes go out";
    };

    "a rebuild keeps the stream index, and later records keep their place"_test = [] {
        PowerMeter<CF> block({{"sample_rate", kRate}, {"window_time", 0.001}, {"segments", gr::Size_t{4U}}});
        init(block);
        const auto before = drive(block, std::span<const CF>(std::vector<CF>(288UZ, CF(1.f, 0.f))));
        expect(eq(before.size(), 3UZ));

        apply<CF>(block, {{"window_time", 0.002}});
        expect(eq(block.window_samples.value, 192U));
        expect(eq(block.coverage(), 0.f)) << "the window starts empty";
        const auto after = drive(block, std::span<const CF>(std::vector<CF>(384UZ, CF(1.f, 0.f))));
        expect(eq(after.size(), 2UZ));
        for (std::size_t w = 0UZ; w < std::min(after.size(), 2UZ); ++w) {
            expect(eq(sampleStartOf(after[w]), 288UZ + 192UZ * w)) << "record " << w << " after the rebuild is stamped at its place in the stream";
        }

        block.start();
        const auto restarted = drive(block, std::span<const CF>(std::vector<CF>(192UZ, CF(1.f, 0.f))));
        expect(eq(restarted.size(), 1UZ));
        expect(eq(restarted.empty() ? 1ULL : sampleStartOf(restarted.front()), 0ULL)) << "start() alone counts the stream from zero";
    };

    "a record waiting for room survives a rebuild"_test = [] {
        PowerMeter<CF> block({{"sample_rate", kRate}, {"window_time", 0.001}, {"segments", gr::Size_t{4U}}});
        init(block);
        expect(eq(endWithNoRoom(block, 120UZ), 0UZ)) << "the window of the first 96 samples closes, and its record waits";

        apply<CF>(block, {{"window_time", 0.002}});
        const auto after = drive(block, std::span<const CF>(std::vector<CF>(192UZ, CF(0.1f, 0.f))));
        expect(eq(after.size(), 2UZ)) << "the waiting record, then the first window of the rebuild";
        if (after.size() == 2UZ) {
            expect(eq(sampleStartOf(after[0UZ]), 0ULL)) << "the waiting record keeps its place";
            expect(approx(static_cast<double>(after[0UZ].signal_values[0UZ]), 0.0, 1e-4)) << "and its level";
            expect(eq(sampleStartOf(after[1UZ]), 120ULL)) << "the rebuilt window starts at the rebuild's sample";
            expect(approx(static_cast<double>(after[1UZ].signal_values[0UZ]), -20.0, 1e-4));
        }
    };

    "a reset to defaults drops the unfinished window and keeps the stream index and the waiting records"_test = [] {
        PowerMeter<CF> block({{"sample_rate", kRate}, {"window_time", 0.001}, {"segments", gr::Size_t{4U}}});
        init(block);
        block.settings().storeDefaults(); // the defaults a graph stores after the constructor's map
        expect(eq(endWithNoRoom(block, 132UZ), 0UZ)) << "the window of the first 96 samples closes, and its record waits";
        expect(eq(block.coverage(), 1.f)) << "the reading covers a whole window before the reset";

        block.settings().resetDefaults();
        expect(eq(block.coverage(), 0.f)) << "the reset empties the window";
        const auto after = drive(block, std::span<const CF>(std::vector<CF>(96UZ, CF(0.1f, 0.f))));
        expect(eq(after.size(), 2UZ)) << "the waiting record, then the first window after the reset";
        if (after.size() == 2UZ) {
            expect(eq(sampleStartOf(after[0UZ]), 0ULL)) << "the waiting record keeps its place";
            expect(eq(sampleStartOf(after[1UZ]), 132ULL)) << "the next record starts at the first sample after the reset, counted from the stream's start";
            expect(approx(static_cast<double>(after[1UZ].signal_values[0UZ]), -20.0, 1e-4)) << "and holds none of the samples before it";
        }
    };

    "the limits refuse a value alone, and the meter computes on the accepted ones"_test = [] {
        PowerMeter<CF> block({{"sample_rate", kRate}, {"window_time", 0.001}, {"segments", gr::Size_t{4U}}});
        init(block);
        const auto rateOf = [](const gr::DataSet<float>& record) {
            const auto& meta  = record.meta_information[0UZ];
            const auto  entry = meta.find(std::pmr::string("sample_rate"));
            const auto* rate  = entry == meta.end() ? nullptr : entry->second.get_if<float>();
            return rate == nullptr ? -1.f : *rate;
        };
        const auto refused = [&block](gr::property_map change) {
            std::ignore = block.settings().setStaged(std::move(change));
            return block.settings().applyStagedParameters().failedParameters;
        };

        expect(refused({{"sample_rate", 0.f}}).contains(gr::property_map::key_type{"sample_rate"})) << "a rate of zero is outside the limits";
        const std::optional<gr::pmt::Value> held = block.settings().get("sample_rate");
        const float*                        kept = held.has_value() ? held->get_if<float>() : nullptr;
        expect(kept != nullptr && *kept == kRate) << "the member and the settings keep the accepted rate";
        expect(!refused({{"sample_rate", 0.f}}).empty()) << "sent again, it is refused again";
        const gr::property_map batch = refused({{"floor_db", -60.0}, {"window_time", -1.0}});
        expect(batch.contains(gr::property_map::key_type{"window_time"}) && !batch.contains(gr::property_map::key_type{"floor_db"})) << "each key of a batch is checked on its own";

        const auto records = drive(block, std::span<const CF>(std::vector<CF>(96UZ, CF(1e-4f, 0.f))));
        expect(eq(block.window_samples.value, 96U)) << "the window keeps its accepted size";
        expect(eq(records.size(), 1UZ));
        if (!records.empty()) {
            expect(eq(rateOf(records.front()), kRate)) << "a record states the accepted rate";
            expect(approx(static_cast<double>(records.front().signal_values[0UZ]), -60.0, 1e-6)) << "and the -80 dBFS input reads the floor the batch applied";
        }

        for (int attempt = 0; attempt < 2; ++attempt) {
            expect(!refused({{"window_time", std::numeric_limits<double>::quiet_NaN()}}).empty()) << "a NaN is outside every limit";
            expect(eq(block.window_time.value, 0.001)) << "and never reaches the member";
        }
        for (const double floor : {std::numeric_limits<double>::quiet_NaN(), -std::numeric_limits<double>::infinity()}) {
            expect(refused({{"floor_db", floor}}).contains(gr::property_map::key_type{"floor_db"})) << std::format("a floor of {} is outside the limits", floor);
            expect(eq(block.floor_db.value, -60.0)) << "the member keeps the accepted floor";
            expect(approx(static_cast<double>(block.level()), -60.0, 1e-6)) << "and the -80 dBFS window still reads it";
        }
        apply<CF>(block, {{"sample_rate", 48000.f}});
        expect(eq(block.window_samples.value, 48U)) << "a value inside the limits applies at once";
    };

    "a value outside its limits on a tag is refused, the run completes, and the meter keeps its window"_test = [] {
        static constexpr std::size_t kLength = 9600UZ;
        static constexpr std::size_t kAt     = 4800UZ;
        // A tag cannot set a key that the constructor's map names. The map leaves out `window_time`, which the tag sets.
        const auto runWith = [](std::optional<gr::property_map> change) {
            gr::Graph graph;
            auto&     source = graph.emplaceBlock<TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(kLength)}, {"mark_tag", false}});
            if (change.has_value()) {
                source._tags.emplace_back(kAt, *change);
            }
            auto& meter = graph.emplaceBlock<PowerMeter<CF>>({{"sample_rate", 9600.f}, {"segments", gr::Size_t{4U}}});
            auto& sink  = graph.emplaceBlock<RecordSink>();
            boost::ut::expect(graph.connect<"out", "in">(source, meter).has_value());
            boost::ut::expect(graph.connect<"records", "in">(meter, sink).has_value());

            gr::scheduler::Simple scheduler;
            boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
            const bool                                   ran = scheduler.runAndWait().has_value();
            std::vector<std::pair<std::uint64_t, float>> records;
            for (const auto& record : sink.received) {
                records.emplace_back(sampleStartOf(record), record.signal_values[0UZ]);
            }
            return std::tuple{ran, records, meter.window_time.value, meter.window_samples.value};
        };

        const auto [plainRan, plain, plainTime, plainWindow] = runWith(std::nullopt);
        expect(plainRan);
        expect(eq(plain.size(), 10UZ)) << "ten windows of 960 samples at the default 0.1 s";
        for (const double value : {-1.0, std::numeric_limits<double>::quiet_NaN()}) {
            const auto [ran, records, time, window] = runWith(gr::property_map{{"window_time", value}});
            expect(ran) << std::format("window_time {}: the refusal on a tag is a plain message, and the run completes", value);
            expect(that % (records == plain)) << std::format("window_time {}: every record, the ones after the tag included, is the record of the accepted window", value);
            expect(eq(time, plainTime)) << "the member keeps the accepted window time";
            expect(eq(window, plainWindow));
        }

        const auto [changedRan, changed, changedTime, changedWindow] = runWith(gr::property_map{{"window_time", 0.05}});
        expect(changedRan);
        expect(eq(changedTime, 0.05)) << "a value inside the limits applies";
        expect(eq(changedWindow, 480U));
        expect(eq(changed.size(), 15UZ)) << "five windows of 960 samples, then ten of 480 from the tag's sample on";
        if (changed.size() == 15UZ) {
            expect(eq(changed[5UZ].first, kAt));
            expect(eq(changed[14UZ].first, kAt + 9UZ * 480UZ));
        }
    };

    "the record's level is clamped exactly where the reader's is"_test = [] {
        PowerMeter<CF> block({{"sample_rate", kRate}, {"window_time", 0.001}, {"segments", gr::Size_t{4U}}, {"floor_db", -60.0}});
        init(block);

        const std::size_t window = static_cast<std::size_t>(block.window_samples.value);
        std::vector<CF>   quiet(window + 1UZ, CF(1e-6f, 0.f)); // -120 dBFS, far under the floor

        const auto records = drive(block, std::span<const CF>(quiet));
        expect(!records.empty()) << "a window closed, so a record went out";
        if (records.empty()) {
            return;
        }
        expect(approx(static_cast<double>(block.level()), -60.0, 1e-6)) << "the reader clamps at floor_db";
        expect(eq(records.front().signal_values[0UZ], block.level())) << "and the record states the same number rather than an unclamped one";
        expect(records.front().signal_values[1UZ] < 1e-9f) << "while the linear channel carries the raw ratio, unclamped";
    };

    "an empty input answers INSUFFICIENT_INPUT_ITEMS and moves nothing"_test = [] {
        namespace test = gr::blocks::testing::span;
        PowerMeter<CF> block({{"sample_rate", kRate}});
        init(block);

        for (const bool connected : {true, false}) {
            std::vector<gr::DataSet<float>>      made(4UZ);
            test::InputSpan<CF>                  inSpan{std::span<const CF>{}};
            test::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(made)};
            outSpan.isConnected           = connected;
            const gr::work::Status status = block.processBulk(inSpan, outSpan);
            expect(status == gr::work::Status::INSUFFICIENT_INPUT_ITEMS) << std::format("records port {}: an empty input is a wait for input, answered {}", connected ? "connected" : "unconnected", static_cast<int>(status));
            expect(eq(inSpan.consumed, 0UZ));
            expect(eq(outSpan.count, 0UZ));
        }
        expect(eq(block.coverage(), 0.f)) << "and the window holds nothing";
    };

    "a record with no room holds the input back and answers INSUFFICIENT_OUTPUT_ITEMS"_test = [] {
        namespace test = gr::blocks::testing::span;
        PowerMeter<CF> block({{"sample_rate", kRate}, {"window_time", 0.001}, {"segments", gr::Size_t{4U}}});
        init(block);

        const std::size_t     window = static_cast<std::size_t>(block.window_samples.value);
        const std::vector<CF> x(window, CF(1.f, 0.f));
        expect(drive(block, std::span<const CF>(x).first(window - 1UZ)).empty()) << "one sample short of the window's close";

        std::vector<gr::DataSet<float>>      none;
        test::InputSpan<CF>                  waiting{std::span<const CF>(x).subspan(window - 1UZ)};
        test::OutputSpan<gr::DataSet<float>> full{std::span<gr::DataSet<float>>(none)};
        expect(block.processBulk(waiting, full) == gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS) << "the next sample closes a window whose record has nowhere to go";
        expect(eq(waiting.consumed, 0UZ));
        expect(eq(full.count, 0UZ));

        std::vector<gr::DataSet<float>>      made(4UZ);
        test::InputSpan<CF>                  resumed{std::span<const CF>(x).subspan(window - 1UZ)};
        test::OutputSpan<gr::DataSet<float>> room{std::span<gr::DataSet<float>>(made)};
        expect(block.processBulk(resumed, room) == gr::work::Status::OK) << "with room the same input proceeds";
        expect(eq(resumed.consumed, 1UZ));
        expect(eq(room.count, 1UZ)) << "and the window's record goes out";
    };

    "a meter fed nothing for longer than the stall report's bound draws no report"_test = [] {
        auto hold = std::make_shared<Hold>();

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<HeldSource>();
        source.hold      = hold;
        source.count     = 9600UZ;
        auto& bare       = graph.emplaceBlock<PowerMeter<CF>>({{"sample_rate", kRate}});
        auto& recorded   = graph.emplaceBlock<PowerMeter<CF>>({{"sample_rate", kRate}});
        auto& records    = graph.emplaceBlock<RecordCounter>();
        expect(graph.connect<"out", "in">(source, bare).has_value());
        expect(graph.connect<"out", "in">(source, recorded).has_value());
        expect(graph.connect<"records", "in">(recorded, records).has_value());
        const std::string sourceLine   = std::format("'{}' keeps returning OK", source.unique_name);
        const std::string bareLine     = std::format("'{}' keeps returning OK", bare.unique_name);
        const std::string recordedLine = std::format("'{}' keeps returning OK", recorded.unique_name);

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());

        StderrCapture capture;
        auto          finished = std::async(std::launch::async, [&scheduler] { return scheduler.runAndWait(); });

        // The source's report marks a second in which it moved nothing and the meters received nothing. Two more of
        // its calls complete a pass in which each meter has waited at least as long.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (!capture.text().contains(sourceLine) && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        const std::size_t reportedAt = hold->calls.load();
        while (hold->calls.load() < reportedAt + 2UZ && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        hold->released.store(true);
        const bool        ran  = finished.get().has_value();
        const std::string text = capture.text();
        capture.release();

        expect(ran);
        expect(text.contains(sourceLine)) << "the capture sees the report the held source draws";
        expect(!text.contains(bareLine)) << "records port unconnected:\n" << text;
        expect(!text.contains(recordedLine)) << "records port connected:\n" << text;
        for (const PowerMeter<CF>* meter : {&bare, &recorded}) {
            expect(eq(meter->coverage(), 1.f)) << "each meter measures the stream that follows the wait";
            expect(approx(static_cast<double>(meter->level()), 0.0, 1e-4));
        }
        expect(eq(records.received, 1UZ)) << "and the connected one publishes the window that closed after it";
    };
};

int main() { /* not needed for UT */ }
