#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <print>
#include <ranges>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/filter/FractionalDelay.hpp>
#include <gnuradio-4.0/algorithm/timing/FrequencySchedule.hpp>
#include <gnuradio-4.0/channel/DopplerShift.hpp>
#include <gnuradio-4.0/channel/RangeDelay.hpp>
#include <gnuradio-4.0/testing/SettingsLimits.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace {

using gr::blocks::channel::DopplerShift;
using gr::blocks::channel::RangeDelay;
using gr::blocks::channel::SchedulePosition;

using C                 = std::complex<float>;
constexpr double kTwoPi = 2. * std::numbers::pi_v<double>;
constexpr double kC     = 299'792'458.;
constexpr double kRoll  = 0.2;
constexpr double kPi    = std::numbers::pi_v<double>;

/// True when the setting types @p Left and @p Right carry one non-empty documentation text.
template<typename Left, typename Right>
inline constexpr bool kSameDoc = !Left::documentation().empty() && Left::documentation() == Right::documentation();

/// The blocks carry a measurement slot and atomic counters, so they are pinned where they are built.
template<typename TBlock>
[[nodiscard]] std::unique_ptr<TBlock> configured(gr::property_map settings) {
    auto block = std::make_unique<TBlock>(std::move(settings));
    block->settings().init();
    std::ignore = block->settings().applyStagedParameters();
    block->start();
    return block;
}

/// @brief The message a staging refusal carries, or empty where the settings were accepted.
template<typename TBlock>
[[nodiscard]] std::string refusalOf(gr::property_map settings) {
    try {
        std::ignore = configured<TBlock>(std::move(settings));
    } catch (const std::exception& refusal) {
        return refusal.what();
    }
    return {};
}

//! One straight call over the whole input, the raw-span overload the anchor-free cases exercise.
template<typename TBlock, typename T>
[[nodiscard]] std::vector<T> runWhole(TBlock& block, std::span<const T> input) {
    std::vector<T> out(input.size());
    std::ignore = block.processBulk(input, std::span<T>(out));
    return out;
}

//! The same input in chunks, so a boundary that lands anywhere gives the same stream.
template<typename TBlock, typename T>
[[nodiscard]] std::vector<T> runChunked(TBlock& block, std::span<const T> input, std::size_t chunk) {
    std::vector<T> out(input.size());
    for (std::size_t i = 0UZ; i < input.size(); i += chunk) {
        const std::size_t n = std::min(chunk, input.size() - i);
        std::ignore         = block.processBulk(input.subspan(i, n), std::span<T>(out.data() + i, n));
    }
    return out;
}

// A ReaderSpanLike/OutputSpanLike pair. It drives the tag-aware overload at an exact chunk size, with tags at exact
// indices, without a graph. A graph chooses its own chunking.
struct TagReader : std::span<const gr::Tag> {
    using value_type = gr::Tag;

    explicit TagReader(std::span<const gr::Tag> tags = {}) : std::span<const gr::Tag>(tags) {}

    constexpr bool consume(std::size_t) noexcept { return true; }
};

struct TagWriter : std::span<gr::Tag> {
    using value_type = gr::Tag;

    constexpr void publish(std::size_t) noexcept {}
};

/// The pair `tags()` yields. It holds a signed offset against the span's base and a reference to the map.
struct ToRelIndexMapRef {
    std::size_t base = 0UZ;

    [[nodiscard]] std::pair<std::ptrdiff_t, std::reference_wrapper<const gr::property_map>> operator()(const gr::Tag& tag) const noexcept { //
        return {static_cast<std::ptrdiff_t>(tag.index) - static_cast<std::ptrdiff_t>(base), std::cref(tag.map)};
    }
};

template<typename T>
struct InSpan : std::span<const T> {
    using value_type = T;

    TagReader   rawTags{};
    std::size_t streamIndex = 0UZ;
    bool        isConnected = true;
    bool        isSync      = true;

    InSpan(std::span<const T> samples, std::size_t at, std::span<const gr::Tag> tags) : std::span<const T>(samples), rawTags(tags), streamIndex(at) {}

    constexpr bool consume(std::size_t) noexcept { return true; }
    constexpr void consumeTags(std::size_t) noexcept {}

    [[nodiscard]] auto tags() const { return rawTags | std::views::transform(ToRelIndexMapRef{streamIndex}); }
    [[nodiscard]] auto tags(std::size_t) const { return rawTags | std::views::transform(ToRelIndexMapRef{streamIndex}); }
};

template<typename T>
struct OutSpan : std::span<T> {
    using value_type = T;

    TagWriter             tags{};
    std::vector<gr::Tag>* sink        = nullptr;
    std::size_t           streamIndex = 0UZ;
    bool                  isConnected = true;
    bool                  isSync      = true;

    OutSpan(std::span<T> samples, std::size_t at, std::vector<gr::Tag>* published) : std::span<T>(samples), sink(published), streamIndex(at) {}

    constexpr void publish(std::size_t) noexcept {}

    void publishTag(const gr::property_map& tagData, std::size_t tagOffset = 0UZ) { sink->push_back(gr::Tag{streamIndex + tagOffset, tagData}); }
};

template<typename T>
struct Driven {
    std::vector<T>       samples{};
    std::vector<gr::Tag> tags{};
};

/// @brief Feed @p block through its tag-aware overload in chunks of @p chunk, with @p tags at their own indices.
template<typename T, typename TBlock>
[[nodiscard]] Driven<T> drive(TBlock& block, std::span<const T> input, std::span<const gr::Tag> tags, std::size_t chunk) {
    Driven<T>         result;
    const std::size_t stride = chunk == 0UZ ? std::max(input.size(), 1UZ) : chunk;
    result.samples.resize(input.size());

    for (std::size_t base = 0UZ; base < input.size(); base += stride) {
        const std::size_t count = std::min(stride, input.size() - base);
        const auto        first = std::ranges::lower_bound(tags, base, std::ranges::less{}, &gr::Tag::index);
        const auto        last  = std::ranges::lower_bound(tags, base + count, std::ranges::less{}, &gr::Tag::index);

        InSpan<T>  inSpan(input.subspan(base, count), base, std::span<const gr::Tag>(first, last));
        OutSpan<T> outSpan(std::span<T>(result.samples.data() + base, count), base, &result.tags);
        std::ignore = block.processBulk(inSpan, outSpan);
    }
    return result;
}

[[nodiscard]] gr::Tag triggerTag(std::size_t index, std::uint64_t timeNs) { //
    return gr::Tag{index, gr::property_map{{gr::property_map::key_type(gr::tag::TRIGGER_TIME.key()), timeNs}}};
}

[[nodiscard]] gr::Tag triggerTag(std::size_t index, std::uint64_t timeNs, float offsetSeconds) {
    return gr::Tag{index, gr::property_map{{gr::property_map::key_type(gr::tag::TRIGGER_TIME.key()), timeNs}, //
                              {gr::property_map::key_type(gr::tag::TRIGGER_OFFSET.key()), offsetSeconds}}};
}

/// The prototype's delivered passband ripple and stopband as linear amplitudes. They are the two floors under the
/// interpolation bound. Together they give the envelope a realized bank is held to.
[[nodiscard]] double rippleFloor(double rippleDb) { return std::pow(10., rippleDb / 20.) - 1.; }
[[nodiscard]] double stopbandFloor(double stopbandDb) { return std::pow(10., stopbandDb / 20.); }

/// @brief The slope of a least-squares line through @p y against its own index, closed form.
[[nodiscard]] double slopeOf(std::span<const double> y) {
    const double n     = static_cast<double>(y.size());
    const double meanX = 0.5 * (n - 1.);
    double       meanY = 0.;
    for (const double value : y) {
        meanY += value;
    }
    meanY /= n;

    double covariance = 0.;
    double variance   = 0.;
    for (std::size_t k = 0UZ; k < y.size(); ++k) {
        const double dx = static_cast<double>(k) - meanX;
        covariance += dx * (y[k] - meanY);
        variance += dx * dx;
    }
    return covariance / variance;
}

/// @brief The unwrapped phase of @p x against a reference tone at @p f0 cycles per sample, from @p skip on.
[[nodiscard]] std::vector<double> residualPhase(std::span<const C> x, double f0, std::size_t skip) {
    std::vector<double> residual(x.size() - skip);
    double              turns = 0.;
    double              last  = 0.;
    for (std::size_t k = skip; k < x.size(); ++k) {
        const double         phase = kTwoPi * f0 * static_cast<double>(k);
        std::complex<double> reference{std::cos(phase), std::sin(phase)};
        const double         wrapped = std::arg(std::complex<double>(static_cast<double>(x[k].real()), static_cast<double>(x[k].imag())) * std::conj(reference));
        if (k > skip) {
            const double step = wrapped - last;
            turns += (step > kPi) ? -kTwoPi : ((step < -kPi) ? kTwoPi : 0.);
        }
        last               = wrapped;
        residual[k - skip] = wrapped + turns;
    }
    return residual;
}

[[nodiscard]] std::vector<C> tone(double f0, std::size_t n) {
    std::vector<C> out(n);
    for (std::size_t k = 0UZ; k < n; ++k) {
        const double phase = kTwoPi * f0 * static_cast<double>(k);
        out[k]             = C(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
    }
    return out;
}

/// A smooth two-tone signal, band-limited far inside the interpolator's design band.
[[nodiscard]] std::vector<C> bandLimited(std::size_t n) {
    std::vector<C> signal(n);
    for (std::size_t k = 0UZ; k < n; ++k) {
        const double x = static_cast<double>(k);
        signal[k]      = C(static_cast<float>(0.6 * std::sin(0.031 * x) + 0.3 * std::cos(0.011 * x)), //
                 static_cast<float>(0.5 * std::cos(0.023 * x) - 0.2 * std::sin(0.007 * x)));
    }
    return signal;
}

/// The same amplitudes with no band limit. The energy reaches Nyquist, where no interpolator is designed.
[[nodiscard]] std::vector<C> fullBand(std::size_t n) {
    std::vector<C> signal(n);
    std::uint64_t  state = 0x243f6a8885a308d3ULL;
    for (C& sample : signal) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        sample = C(static_cast<float>(2. * static_cast<double>(state % 2048ULL) / 2048. - 1.), //
            static_cast<float>(2. * static_cast<double>((state >> 20U) % 2048ULL) / 2048. - 1.));
    }
    return signal;
}

[[nodiscard]] std::size_t peakIndex(std::span<const float> x) {
    std::size_t at = 0UZ;
    for (std::size_t k = 1UZ; k < x.size(); ++k) {
        if (std::abs(x[k]) > std::abs(x[at])) {
            at = k;
        }
    }
    return at;
}

/// @brief Stages @p change on a running block and applies it. True when the block or the limits refused it.
template<typename TBlock>
[[nodiscard]] bool refusedLive(TBlock& block, gr::property_map change) {
    std::ignore = block.settings().setStaged(std::move(change));
    try {
        return !block.settings().applyStagedParameters().failedParameters.empty();
    } catch (const std::exception&) {
        return true;
    }
}

/// @brief The message with which a running block refuses @p change, or empty where it applies the change.
template<typename TBlock>
[[nodiscard]] std::string refusalLive(TBlock& block, gr::property_map change) {
    std::ignore = block.settings().setStaged(std::move(change));
    try {
        std::ignore = block.settings().applyStagedParameters();
    } catch (const gr::exception& refusal) {
        return refusal.message;
    }
    return {};
}

/// @brief The samples of @p driven from @p from on.
template<typename T>
[[nodiscard]] std::vector<T> tail(const Driven<T>& driven, std::size_t from) {
    return std::vector<T>(driven.samples.begin() + static_cast<std::ptrdiff_t>(from), driven.samples.end());
}

/// Records every sample and every tag of its input, the tags at the end-of-stream index included. The sink reads its
/// input's tag ring when the stream ends.
template<typename T>
struct EndSink : gr::Block<EndSink<T>> {
    gr::PortIn<T> in;

    GR_MAKE_REFLECTABLE(EndSink, in);

    std::vector<T>       _samples;
    std::vector<gr::Tag> _tags;

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& input) {
        record(input);
        return gr::work::Status::OK;
    }

    [[nodiscard]] gr::work::Status processEpilogue(gr::InputSpanLike auto& input) {
        record(input);
        const std::size_t end = input.streamIndex + input.size();
        for (const gr::Tag& tag : in.tagReader().get()) {
            if (tag.index >= end) {
                _tags.push_back(tag);
            }
        }
        return gr::work::Status::OK;
    }

    void record(auto& input) {
        const std::size_t end = input.streamIndex + input.size();
        for (const gr::Tag& tag : input.rawTags) {
            if (tag.index >= input.streamIndex && tag.index < end) {
                _tags.push_back(tag);
            }
        }
        _samples.insert(_samples.end(), input.begin(), input.end());
    }

    [[nodiscard]] std::vector<std::size_t> indicesOf(std::string_view key) const {
        std::vector<std::size_t> indices;
        for (const gr::Tag& tag : _tags) {
            if (tag.map.contains(gr::property_map::key_type{key})) {
                indices.push_back(tag.index);
            }
        }
        return indices;
    }
};

} // namespace

int main() {
    using namespace boost::ut;

    // A constant delay at an integral total lag is a plain shift, to the interpolation bound
    "a constant delay reduces to an integer sample shift, latency included"_test = [] {
        // The line's own lag is fractional. A whole-sample compare after removing the group delay is not possible at
        // an arbitrary commanded delay. The commanded delay is chosen to make the total lag integral instead.
        // The interpolator runs at a fraction of an arm. The comparison against `out[n] = in[n - delay]` is exact.
        // That formula is the integer shift that defines `SampleDelay`.
        constexpr std::size_t kBank = 32UZ;
        const auto            probe = configured<RangeDelay<float>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{1., 1.}}, {"sample_rate", 1.f}, {"order", 1}, {"bank_size", gr::Size_t{kBank}}});

        const double latency   = probe->latencySamples();
        const double lag       = std::ceil(latency) + 7.; // an integer, and past the transient
        const double commanded = lag - latency;           // fractional, and the bank interpolates between arms

        auto block = configured<RangeDelay<float>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{commanded, commanded}}, {"sample_rate", 1.f}, {"order", 1}, {"bank_size", gr::Size_t{kBank}}});

        std::vector<float> signal(4096UZ);
        for (std::size_t k = 0UZ; k < signal.size(); ++k) {
            signal[k] = static_cast<float>(0.6 * std::sin(0.031 * static_cast<double>(k)) + 0.3 * std::sin(0.007 * static_cast<double>(k)));
        }
        const auto        out   = runWhole<RangeDelay<float>, float>(*block, std::span<const float>(signal));
        const std::size_t nSkip = block->historySamples() + 64UZ;
        const auto        shift = static_cast<std::size_t>(std::llround(lag));

        double worst = 0.;
        for (std::size_t k = nSkip; k + shift < signal.size(); ++k) {
            worst = std::max(worst, static_cast<double>(std::abs(out[k + shift] - signal[k])));
        }
        const double bound = gr::filter::arbitraryInterpolationError(kBank, kRoll, 1);
        std::println("[range] constant delay: L = {}, q = 1, lag {} samples ({} commanded + {:.6f} line): worst {:.3e} against the bound {:.3e}", kBank, lag, commanded, latency, worst, bound);
        expect(that % (worst < bound)) << std::format("the constant delay is a plain shift to the L = {} interpolation bound: {:.3e} against {:.3e}", kBank, worst, bound);
        expect(that % (block->nBankRebuilds() == 0ULL)) << "cutting the first bank is not a rebuild";
    };

    // The interpolation error stays inside the envelope the bank was cut for
    "a swept-frequency probe stays inside the interpolation envelope at every bank"_test = [] {
        struct Arm {
            std::size_t bank;
            int         order;
        };
        constexpr Arm         arms[]   = {{8UZ, 1}, {32UZ, 1}, {32UZ, 3}, {128UZ, 1}};
        constexpr std::size_t kSamples = 2048UZ;

        for (const Arm& arm : arms) {
            const auto designed = gr::filter::designFractionalDelay(arm.bank, kRoll);
            expect(designed.has_value() >> fatal);
            const gr::filter::ResamplerDesign& design = *designed;
            const double                       interp = gr::filter::arbitraryInterpolationError(arm.bank, kRoll, arm.order);
            const double                       bound  = interp + rippleFloor(design.rippleDb) + stopbandFloor(design.stopbandDb);

            double worstAll = 0.;
            for (std::size_t step = 0UZ; step < 33UZ; ++step) { // 33 fractional delays across one arm spacing and beyond
                const double      delay   = 4. + static_cast<double>(step) / 32.;
                auto              block   = configured<RangeDelay<C>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{delay, delay}}, {"sample_rate", 1.f}, {"order", arm.order}, {"bank_size", gr::Size_t{static_cast<std::uint32_t>(arm.bank)}}});
                const double      latency = block->latencySamples();
                const std::size_t nSkip   = block->historySamples() + 64UZ;

                for (const double f0 : {0.02, 0.10, 0.20, 0.30, 0.39}) { // across the passband, whose edge is (1 - rolloff)/2
                    block->start();
                    const auto in  = tone(f0, kSamples);
                    const auto out = runWhole<RangeDelay<C>, C>(*block, std::span<const C>(in));
                    for (std::size_t k = nSkip; k < kSamples; ++k) {
                        const double phase = kTwoPi * f0 * (static_cast<double>(k) - latency - delay);
                        const C      ideal{static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))};
                        worstAll = std::max(worstAll, static_cast<double>(std::abs(out[k] - ideal)));
                    }
                }
            }
            std::println("[range] swept probe: L = {:3}, q = {}: worst {:.6e} against the envelope {:.6e} = interpolation {:.6e} + ripple {:.6e} + stopband {:.6e}", arm.bank, arm.order, worstAll, bound, interp, rippleFloor(design.rippleDb), stopbandFloor(design.stopbandDb));
            expect(that % (worstAll < bound)) << std::format("L = {}, q = {} misses its own envelope: {:.6e} against {:.6e}", arm.bank, arm.order, worstAll, bound);
        }
    };

    // The consistency identity, on the measured pair
    "range delay and Doppler shift are two factors of one physics"_test = [] {
        constexpr double      fs       = 48'000.;
        constexpr double      fb       = 12'000.;
        constexpr double      fc       = 437e6;
        constexpr double      v        = -7'000.; // closing
        constexpr double      seconds  = 10.;
        constexpr std::size_t n        = static_cast<std::size_t>(fs * seconds);
        constexpr double      d0       = 0.05; // seconds, comfortably inside the reach and non-negative across the pass
        const double          slope    = v / kC;
        const double          expected = -fb * v / kC;                 // +0.2801938 Hz
        const double          carrier  = gr::timing::offsetFor(v, fc); // +10203.7257 Hz
        const double          ratio    = fb / fc;                      // 2.745995e-5

        // The envelope's half. A linear range ramp of rate v is a delay ramp of rate v/c. A tone through it emerges
        // at f_b*(1 - v/c) exactly. The derivation is closed form, not first order.
        auto       delayBlock = configured<RangeDelay<C>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, static_cast<std::int64_t>(seconds * 1e9)}}, {"schedule_delays_s", std::vector<double>{d0, d0 + slope * seconds}}, {"sample_rate", static_cast<float>(fs)}, {"order", 3}, {"bank_size", gr::Size_t{128U}}});
        const auto delayed    = runWhole<RangeDelay<C>, C>(*delayBlock, std::span<const C>(tone(fb / fs, n)));

        const std::size_t nSkip       = delayBlock->historySamples() + 1024UZ;
        const auto        residual    = residualPhase(std::span<const C>(delayed), fb / fs, nSkip);
        const double      measuredEnv = slopeOf(std::span<const double>(residual)) / kTwoPi * fs;

        // The carrier's half. The same range rate goes through `offsetFor`, and the shift is measured off the block.
        const std::vector<C> unmodulated(n, C(1.f, 0.f));
        auto                 shiftBlock      = configured<DopplerShift<C>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, static_cast<std::int64_t>(seconds * 1e9)}}, {"schedule_offsets_hz", std::vector<double>{carrier, carrier}}, {"sample_rate", static_cast<float>(fs)}});
        const auto           shifted         = runWhole<DopplerShift<C>, C>(*shiftBlock, std::span<const C>(unmodulated));
        const auto           carrierRes      = residualPhase(std::span<const C>(shifted), 0., 1024UZ);
        const double         measuredCarrier = slopeOf(std::span<const double>(carrierRes)) / kTwoPi * fs;

        const double measuredRatio = measuredEnv / measuredCarrier;
        std::println("[range] delay and Doppler: envelope shift {:.7f} Hz against {:.7f}; carrier shift {:.4f} Hz against {:.4f}; ratio {:.6e} against f_b/f_c {:.6e}", measuredEnv, expected, measuredCarrier, carrier, measuredRatio, ratio);

        expect(that % (measuredEnv > 0.)) << "a closing pass shifts the envelope's tone up";
        expect(that % (measuredCarrier > 0.)) << "and reads high on the carrier, which is the same sign convention";
        expect(that % (std::abs(measuredEnv - expected) < 0.01 * expected)) << std::format("the delay's derivative shifts the tone by -f_b v/c: {:.7f} against {:.7f}", measuredEnv, expected);
        expect(that % (std::abs(measuredCarrier - carrier) < 0.01 * carrier)) << std::format("the schedule shifts the carrier by offsetFor(v, f_c): {:.4f} against {:.4f}", measuredCarrier, carrier);
        expect(that % (std::abs(measuredRatio - ratio) < 0.01 * ratio)) << std::format("the two shifts stand in the ratio of the two frequencies: {:.6e} against {:.6e}", measuredRatio, ratio);
    };

    // The reported delay is the interpolant, at the knots, between them and held past the ends
    "the reported delay is the schedule's own interpolant, to the double contract"_test = [] {
        constexpr double kD0 = 1.0e-3;
        constexpr double kD1 = 3.0e-3;
        const auto       at  = [](std::int64_t timeNs) {
            return configured<RangeDelay<float>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{kD0, kD1}}, //
                {"sample_rate", 1.e6f}, {"anchor_index", std::uint64_t{0}}, {"anchor_ns", timeNs}});
        };

        double worst = 0.;
        for (int step = 0; step <= 10; ++step) { // the two knots and ten interior points
            const std::int64_t timeNs   = static_cast<std::int64_t>(step) * 100'000'000LL;
            const double       expected = kD0 + (kD1 - kD0) * static_cast<double>(timeNs) * 1e-9;
            const auto         block    = at(timeNs);
            worst                       = std::max(worst, std::abs(block->currentDelaySeconds() - expected));
            expect(that % (std::abs(block->currentDelaySeconds() - expected) <= 1e-15)) << std::format("at {} ns the interpolant is {:.17g}, read {:.17g}", timeNs, expected, block->currentDelaySeconds());
            expect(block->schedulePosition() == SchedulePosition::Inside);
            expect(that % (std::abs(block->currentDelaySamples() - expected * 1e6) <= 1e-9)) << "the same delay in samples at the block's own rate";
        }
        std::println("[range] reported delay: worst interpolant disagreement {:.3e} s over the knots and ten interior points", worst);

        const auto before = at(-500'000'000LL);
        expect(that % (before->currentDelaySeconds() == kD0)) << "before the table the first value holds, never an extrapolation";
        expect(before->schedulePosition() == SchedulePosition::Before);
        const auto after = at(2'000'000'000LL);
        expect(that % (after->currentDelaySeconds() == kD1)) << "after the table the last value holds";
        expect(after->schedulePosition() == SchedulePosition::After);
    };

    // Every derived refusal fires, and names the knot or the segment it fired on
    "the settings and the schedule's edges are refused where they cannot be meant"_test = [] {
        const auto ramp = [](double d0, double d1) { return gr::property_map{{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{d0, d1}}, {"sample_rate", 1.e6f}}; };

        const std::string negative = refusalOf<RangeDelay<float>>(ramp(-1e-6, -1e-6));
        expect(!negative.empty() && negative.contains("knot 0")) << std::format("a negative delay under apply names its knot: {}", negative);

        const std::string steep = refusalOf<RangeDelay<float>>(ramp(0.001, 2.001)); // two seconds of delay over one second of stream
        expect(!steep.empty() && steep.contains("segment 1")) << std::format("a slope past one sample per sample names its segment: {}", steep);

        const std::string tooFar = refusalOf<RangeDelay<float>>(ramp(2200., 2200.)); // 2.2e9 samples at 1 MS/s, past 2^31
        expect(!tooFar.empty() && tooFar.contains("knot 0")) << std::format("a delay past the fixed point's reach names its knot: {}", tooFar);

        const std::string backwards = refusalOf<RangeDelay<float>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 2'000'000'000LL, 1'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{1e-3, 1e-3, 1e-3}}, {"sample_rate", 1.e6f}});
        expect(!backwards.empty() && backwards.contains("knot 2")) << std::format("times that do not increase name the knot that turned back: {}", backwards);
        expect(backwards.contains("'schedule_times_ns'/'schedule_delays_s'")) << std::format("the schedule's refusal names the two settings: {}", backwards);

        const std::string unpaired = refusalOf<RangeDelay<float>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{1e-3, 1e-3, 1e-3}}, {"sample_rate", 1.e6f}});
        expect(!unpaired.empty() && unpaired.contains("must be paired")) << std::format("a table with more delays than times is refused: {}", unpaired);

        const std::string both = refusalOf<RangeDelay<float>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{1e-3, 1e-3}}, {"schedule_file", std::string("/nonexistent.gr4traj")}, {"sample_rate", 1.e6f}});
        expect(!both.empty() && both.contains("are both set")) << std::format("a file beside the vectors is refused: {}", both);

        const std::string missing = refusalOf<RangeDelay<float>>({{"schedule_file", std::string("/nonexistent.gr4traj")}, {"sample_rate", 1.e6f}});
        expect(missing.contains("RangeDelay: 'schedule_file': ") && missing.contains("/nonexistent.gr4traj")) << std::format("a file the loader refuses names the setting and the path: {}", missing);
        const std::string missingShift = refusalOf<DopplerShift<C>>({{"schedule_file", std::string("/nonexistent.gr4traj")}, {"sample_rate", 1.e6f}});
        expect(missingShift.contains("DopplerShift: 'schedule_file': ") && missingShift.contains("/nonexistent.gr4traj")) << std::format("the shift names the setting and the path too: {}", missingShift);

        const std::string offAxis = refusalOf<RangeDelay<float>>({{"sample_rate", 1.e30f}});
        expect(offAxis.contains("RangeDelay") && offAxis.contains("sample_rate")) << std::format("a rate off the exact time axis names the block and the setting: {}", offAxis);
        const std::string offAxisShift = refusalOf<DopplerShift<C>>({{"sample_rate", 1.e30f}});
        expect(offAxisShift.contains("DopplerShift") && offAxisShift.contains("sample_rate")) << std::format("the shift names them too: {}", offAxisShift);

        // and the settings that carry their own arithmetic
        expect(!refusalOf<RangeDelay<float>>([&] {
            auto s         = ramp(1e-3, 1e-3);
            s["bank_size"] = gr::Size_t{48U};
            return s;
        }())
                    .empty())
            << "a bank that is not a power of two";
        expect(!refusalOf<RangeDelay<float>>([&] {
            auto s     = ramp(1e-3, 1e-3);
            s["order"] = 2;
            return s;
        }())
                    .empty())
            << "an interpolation order the line has no arm blend for";
        expect(!refusalOf<RangeDelay<float>>([&] {
            auto s             = ramp(1e-3, 1e-3);
            s["anchor_source"] = std::string("on_tuesdays");
            return s;
        }())
                    .empty())
            << "an anchor source that is not one of the three";
        expect(!refusalOf<RangeDelay<float>>([&] {
            auto s         = ramp(1e-3, 1e-3);
            s["direction"] = std::string("undo");
            return s;
        }())
                    .empty())
            << "a seat that is neither apply nor correct";
        expect(refusalOf<RangeDelay<float>>(ramp(1e-3, 1e-3)).empty()) << "a flat, reachable, non-negative delay is a schedule";

        // a bias that is the whole commanded delay is held to the same two rules as a knot
        expect(!refusalOf<RangeDelay<float>>({{"bias_s", -1e-6}, {"sample_rate", 1.e6f}}).empty()) << "a negative bias with no table reaches the future";
        expect(!refusalOf<RangeDelay<float>>({{"bias_s", 2200.}, {"sample_rate", 1.e6f}}).empty()) << "a bias with no table is held to the line's reach";
        expect(refusalOf<RangeDelay<float>>({{"bias_s", 1e-3}, {"sample_rate", 1.e6f}}).empty()) << "a reachable bias with no table is a constant delay";

        // an anchor time past the trigger bound would carry later samples off the time axis
        const auto pastAxis = static_cast<std::int64_t>(gr::timing::ScheduleAnchor::kMaxTriggerTimeNs / 2ULL * 3ULL);
        expect(!refusalOf<RangeDelay<float>>({{"anchor_ns", pastAxis}}).empty()) << "an anchor_ns past the bound";
        expect(!refusalOf<DopplerShift<C>>({{"anchor_ns", pastAxis}}).empty()) << "the shift refuses it too";
        expect(refusalOf<RangeDelay<float>>({{"anchor_ns", static_cast<std::int64_t>(gr::timing::ScheduleAnchor::kMaxTriggerTimeNs)}}).empty()) << "the bound itself is an anchor";
    };

    // A combination whose prototype passes the design cap is refused from three numbers, before any design runs
    "a bank the design cap cannot hold is refused by name before it is designed"_test = [] {
        const auto settingsWith = [](gr::property_map extra) {
            gr::property_map s{{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{1e-3, 1e-3}}, {"sample_rate", 1.e6f}, {"order", 1}};
            for (auto& [key, value] : extra) {
                s.insert_or_assign(key, value);
            }
            return s;
        };
        const auto expectNamed = [](const std::string& refusal, std::size_t bank, double rolloff, double attenuationDb, std::string_view why) {
            const std::size_t estimate = gr::filter::fractionalDelayTapEstimate(bank, rolloff, attenuationDb);
            expect(that % (estimate > gr::filter::kFractionalDelayMaxTaps)) << std::format("{}: the estimate {} is past the cap", why, estimate);
            expect(refusal.contains("RangeDelay") && refusal.contains("'bank_size'") && refusal.contains("'rolloff'") && refusal.contains("'attenuation_db'")) << std::format("{}: the refusal names the block and the three settings: {}", why, refusal);
            expect(refusal.contains(std::format("needs about {} taps, past the {}", estimate, gr::filter::kFractionalDelayMaxTaps))) << std::format("{}: the refusal carries the estimate and the cap, so it came before the design: {}", why, refusal);
        };

        expectNamed(refusalOf<RangeDelay<float>>(settingsWith({{"bank_size", gr::Size_t{2048U}}})), 2048UZ, 0.2, 60.0, "2048 arms at the default");
        expectNamed(refusalOf<RangeDelay<float>>(settingsWith({{"bank_size", gr::Size_t{1024U}}, {"attenuation_db", 140.0}})), 1024UZ, 0.2, 140.0, "1024 arms at the top attenuation");
        expectNamed(refusalOf<RangeDelay<float>>(settingsWith({{"bank_size", gr::Size_t{32U}}, {"rolloff", 0.001}})), 32UZ, 0.001, 60.0, "a narrow transition on a small bank");

        // bank_size 0 derives the bank, and the refusal says so
        const std::size_t derived = gr::filter::arbitraryBankSize(120.0, 0.2, 1);
        const std::string refusal = refusalOf<RangeDelay<float>>(settingsWith({{"attenuation_db", 120.0}}));
        expectNamed(refusal, derived, 0.2, 120.0, "a derived bank at 120 dB");
        expect(refusal.contains(std::format("'bank_size' {} (derived)", derived))) << std::format("the derived bank is named as derived: {}", refusal);

        // the ripple limit makes a low attenuation long: Kaiser's estimate alone is 8559 taps, and the estimate is ten times that
        expectNamed(refusalOf<RangeDelay<float>>(settingsWith({{"bank_size", gr::Size_t{1024U}}, {"attenuation_db", 20.0}})), 1024UZ, 0.2, 20.0, "1024 arms at the lowest attenuation");

        // an explicit bank is told to lower itself, a derived one to change what it follows
        expect(!refusal.contains("lower 'bank_size'") && refusal.contains("raise 'order' to 3")) << std::format("a derived bank is advised through order: {}", refusal);
        const std::string explicitBank = refusalOf<RangeDelay<float>>(settingsWith({{"bank_size", gr::Size_t{2048U}}}));
        expect(explicitBank.contains("lower 'bank_size'") && !explicitBank.contains("raise 'order'")) << std::format("an explicit bank is advised to be smaller: {}", explicitBank);

        // no rolloff below one holds more than 8192 arms, so the limit of bank_size is 8192: the limit refuses 16384, not the cap check
        const std::string overLimit = refusalOf<RangeDelay<float>>(settingsWith({{"bank_size", gr::Size_t{16384U}}}));
        expect(!overLimit.empty() && !overLimit.contains("needs about")) << std::format("16384 arms are refused by the setting's limits: {}", overLimit);
    };

    // A bank the design cannot hold, refused while the block runs, leaves the line and its samples as they were
    "a refused bank leaves the accepted bank and the samples as they were"_test = [] {
        const gr::property_map settings{{"schedule_times_ns", std::vector<std::int64_t>{0LL, 100'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{0.01, 0.01}}, {"sample_rate", 1'000.f}, {"order", 1}, {"bank_size", gr::Size_t{32U}}};
        std::vector<float>     impulses(512UZ, 0.f);
        impulses[100UZ] = 1.f;
        impulses[356UZ] = 1.f;

        auto                     unchanged = configured<RangeDelay<float>>(settings);
        const std::vector<float> reference = runChunked<RangeDelay<float>, float>(*unchanged, std::span<const float>(impulses), 256UZ);

        auto               block = configured<RangeDelay<float>>(settings);
        std::vector<float> out(impulses.size());
        std::ignore               = block->processBulk(std::span<const float>(impulses).first(256UZ), std::span<float>(out).first(256UZ));
        const std::string refusal = refusalLive(*block, {{"bank_size", gr::Size_t{2048U}}});
        std::ignore               = block->processBulk(std::span<const float>(impulses).subspan(256UZ), std::span<float>(out).subspan(256UZ));
        expect(refusal.contains("needs about")) << std::format("the combination is refused by the cap check: {}", refusal);
        expect(std::ranges::equal(out, reference)) << "the samples are those of the unchanged block";
        expect(that % (block->nBankRebuilds() == 0ULL && block->currentDelaySamples() == unchanged->currentDelaySamples())) << "no bank was rebuilt and the delay is the accepted one";
    };

    // The check admits a combination the design can still miss. On a running block the miss is refused after the design
    // and the accepted bank stays in force. 32 arms at 60 dB and a rolloff of 0.00352 need 65.9k taps and the estimate
    // reads 65.2k.
    "a design that misses its target on a running block leaves the accepted bank and the samples as they were"_test = [] {
        const gr::property_map settings{{"schedule_times_ns", std::vector<std::int64_t>{0LL, 100'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{0.01, 0.01}}, {"sample_rate", 1'000.f}, {"order", 1}, {"bank_size", gr::Size_t{32U}}};
        std::vector<float>     impulses(512UZ, 0.f);
        impulses[100UZ] = 1.f;
        impulses[356UZ] = 1.f;
        expect(that % gr::filter::fractionalDelayReachable(32UZ, 0.00352, 60.0)) << "the check admits the combination, so only the design can refuse it";

        auto                     unchanged = configured<RangeDelay<float>>(settings);
        const std::vector<float> reference = runChunked<RangeDelay<float>, float>(*unchanged, std::span<const float>(impulses), 256UZ);

        auto               block = configured<RangeDelay<float>>(settings);
        std::vector<float> out(impulses.size());
        std::ignore               = block->processBulk(std::span<const float>(impulses).first(256UZ), std::span<float>(out).first(256UZ));
        const std::string refusal = refusalLive(*block, {{"rolloff", 0.00352}});
        std::ignore               = block->processBulk(std::span<const float>(impulses).subspan(256UZ), std::span<float>(out).subspan(256UZ));
        expect(refusal.contains("designs 65537 taps that deliver a stopband of")) << std::format("the design is refused with the stopband it delivers: {}", refusal);
        expect(!refusal.contains("needs about")) << std::format("it was not the cap check: {}", refusal);
        expect(std::ranges::equal(out, reference)) << "the samples are those of the unchanged block";
        expect(that % (block->nBankRebuilds() == 0ULL && block->currentDelaySamples() == unchanged->currentDelaySamples())) << "no bank was rebuilt and the delay is the accepted one";
    };

    // The check admits a combination the design can still miss; the block then refuses with what the design delivers
    "a design that misses its stopband is refused with the stopband it delivers"_test = [] {
        gr::filter::ResamplerDesign design;
        design.taps.assign(gr::filter::kFractionalDelayMaxTaps | 1UZ, 0.f);
        design.stopbandDb = -41.79;
        design.rippleDb   = 0.069;
        design.ok         = false;

        const std::optional<std::string> explicitBank = gr::blocks::channel::detail::designShortfall(2048UZ, false, 0.2, 60.0, design);
        expect(fatal(explicitBank.has_value()));
        expect(explicitBank->contains("RangeDelay") && explicitBank->contains("'bank_size' 2048 with") && explicitBank->contains("designs 65537 taps") && explicitBank->contains("stopband of -41.79 dB") && explicitBank->contains("ripple of 0.069 dB")) << std::format("the refusal names the settings and the delivered stopband and ripple: {}", *explicitBank);
        expect(explicitBank->contains("lower 'bank_size'")) << std::format("an explicit bank is advised to be smaller: {}", *explicitBank);

        const std::optional<std::string> derivedBank = gr::blocks::channel::detail::designShortfall(1024UZ, true, 0.2, 120.0, design);
        expect(fatal(derivedBank.has_value()));
        expect(derivedBank->contains("'bank_size' 1024 (derived)") && derivedBank->contains("raise 'order' to 3")) << std::format("a derived bank is marked and advised through its inputs: {}", *derivedBank);

        design.ok = true;
        expect(!gr::blocks::channel::detail::designShortfall(2048UZ, false, 0.2, 60.0, design).has_value()) << "a design that met its target is not refused";
    };

    // A setting both blocks have reads the same in both
    "the settings RangeDelay shares with DopplerShift carry one documentation text"_test = [] {
        using Range         = RangeDelay<float>;
        using Shift         = DopplerShift<C>;
        constexpr bool same = kSameDoc<decltype(Range::schedule_times_ns), decltype(Shift::schedule_times_ns)> && kSameDoc<decltype(Range::direction), decltype(Shift::direction)> && kSameDoc<decltype(Range::sample_rate), decltype(Shift::sample_rate)> && kSameDoc<decltype(Range::anchor_index), decltype(Shift::anchor_index)> && //
                              kSameDoc<decltype(Range::anchor_ns), decltype(Shift::anchor_ns)> && kSameDoc<decltype(Range::anchor_source), decltype(Shift::anchor_source)> && kSameDoc<decltype(Range::honor_trigger_offset), decltype(Shift::honor_trigger_offset)>;
        expect(that % same) << "schedule_times_ns, direction, sample_rate, anchor_index, anchor_ns, anchor_source and honor_trigger_offset";
        expect(that % (!kSameDoc<decltype(Range::schedule_file), decltype(Shift::schedule_file)>)) << "schedule_file names a different column in each block, so its texts differ";
    };

    // A range-only file drives the delay and refuses the shift, naming the column it has not got
    "a range-only trajectory file feeds RangeDelay and is refused by DopplerShift"_test = [] {
        const std::filesystem::path path = std::filesystem::temp_directory_path() / "qa_RangeDelay_range_only.gr4traj";
        {
            std::ofstream file(path);
            file << "#!gr4-trajectory 1\ncolumns time range_m\ntime_scale unix_ns\n";
            file << "0 300000\n1000000000 307000\n2000000000 314000\n";
        }

        auto block = configured<RangeDelay<float>>({{"schedule_file", path.string()}, {"sample_rate", 1.e6f}});
        expect(that % (std::abs(block->currentDelaySeconds() - 300000. / kC) < 1e-15)) << "the range column arrives as range/c, knot for knot";
        expect(block->schedulePosition() == SchedulePosition::Inside);

        const std::string refused = refusalOf<DopplerShift<C>>({{"schedule_file", path.string()}, {"sample_rate", 1.e6f}});
        expect(!refused.empty() && refused.contains("has no frequency column")) << std::format("the shift refuses a table it cannot be built from, naming the column: {}", refused);
        expect(refused.contains("offset_hz") && refused.contains("range_rate_m_s")) << std::format("and names the columns it looked for: {}", refused);

        std::filesystem::remove(path);
    };

    // Apply then correct recover the signal, to the commutation term and nothing hidden
    "apply then correct through the same table recover the signal"_test = [] {
        // The pair is not an exact inverse. The correcting block reads the schedule at its own stream position. The
        // signal reaching it left the first block one lag earlier. The two lookups differ by that lag times the
        // table's slope. The number is arithmetic and is printed below beside the measured value.
        constexpr double      fs = 100'000.;
        constexpr std::size_t n  = 8000UZ;

        const std::vector<std::int64_t> times{0LL, static_cast<std::int64_t>(1e9)};
        const std::vector<double>       delays{1.0e-4, 1.5e-4};
        const double                    slopePerSecond = (delays[1] - delays[0]) / 1.0;

        auto forward = configured<RangeDelay<C>>({{"schedule_times_ns", times}, {"schedule_delays_s", delays}, {"direction", std::string("apply")}, {"bias_s", 0.}, {"sample_rate", static_cast<float>(fs)}, {"order", 3}, {"bank_size", gr::Size_t{128U}}});

        // The two lines' latencies are fractional. A whole-sample comparison needs a bias that makes the total lag
        // integral. The causal rule needs a bias at or above the table's maximum. This bias meets both.
        const double latency     = forward->latencySamples();
        const double biasSamples = std::ceil(2. * latency) + 16. - 2. * latency;
        const auto   lag         = static_cast<std::size_t>(std::llround(2. * latency + biasSamples));

        const auto pair = [&](const std::vector<double>& table, std::span<const C> signal) {
            auto       first     = configured<RangeDelay<C>>({{"schedule_times_ns", times}, {"schedule_delays_s", table}, {"direction", std::string("apply")}, {"bias_s", 0.}, {"sample_rate", static_cast<float>(fs)}, {"order", 3}, {"bank_size", gr::Size_t{128U}}});
            auto       second    = configured<RangeDelay<C>>({{"schedule_times_ns", times}, {"schedule_delays_s", table}, {"direction", std::string("correct")}, {"bias_s", biasSamples / fs}, {"sample_rate", static_cast<float>(fs)}, {"order", 3}, {"bank_size", gr::Size_t{128U}}});
            const auto delayed   = runWhole<RangeDelay<C>, C>(*first, signal);
            const auto recovered = runWhole<RangeDelay<C>, C>(*second, std::span<const C>(delayed));

            const std::size_t nSkip = first->historySamples() + second->historySamples() + 128UZ;
            double            worst = 0.;
            for (std::size_t k = nSkip; k + lag < n; ++k) {
                worst = std::max(worst, static_cast<double>(std::abs(recovered[k + lag] - signal[k])));
            }
            return worst;
        };

        // The schedule-time mismatch of the pair, in samples of delay, from the scene's numbers. The correcting
        // block looks up the table `lag + tau` seconds after the first one. Over that gap the table moves by its
        // slope.
        const double lookupApartSeconds = (static_cast<double>(lag) + delays[1] * fs) / fs;
        const double commutationSamples = lookupApartSeconds * slopePerSecond * fs;
        // and the effect of a misalignment of that many samples on this signal, at most its slope per sample
        const double signalSlope = 0.6 * 0.031 + 0.3 * 0.011 + 0.5 * 0.023 + 0.2 * 0.007;

        // The control arm is the same pair over a table with no slope. Its commutation term is zero. It measures
        // the two passes' own interpolation error on this signal. That error is the other possible part of the
        // moving arm's residual. The control's error comes out the larger of the two. The cause is the bank's arms.
        // A constant delay keeps the bank on one arm, and its error is systematic. A moving delay walks every arm,
        // and the worst of that walk is smaller.
        const std::vector<C>      band = bandLimited(n);
        const std::vector<double> flat{delays[1], delays[1]};
        const double              floorWorst = pair(flat, std::span<const C>(band));
        const double              worstBand  = pair(delays, std::span<const C>(band));
        const double              bound      = floorWorst + commutationSamples * signalSlope;

        std::println("[range] apply then correct: band-limited worst {:.3e} against {:.3e} = the flat-table interpolation error {:.3e} + commutation {:.3e} samples * the signal's own slope {:.3e} = {:.3e}", worstBand, bound, floorWorst, commutationSamples, signalSlope, commutationSamples * signalSlope);
        expect(that % (worstBand < bound)) << std::format("the pair recovers a band-limited signal to the commutation term: {:.3e} against {:.3e}", worstBand, bound);
        expect(that % (worstBand > 0.)) << "the pair is not an exact inverse, and the residual is measured rather than assumed away";

        const std::vector<C> white      = fullBand(n);
        const double         worstWhite = pair(delays, std::span<const C>(white));
        std::println("[range] apply then correct: the same pair on a full-band signal reads {:.3e} — that signal has energy at Nyquist, where the bank is a stopband and not a delay, so the figure measures the interpolator outside its design band and is not the commutation term", worstWhite);
        expect(that % (worstWhite > 10. * worstBand)) << "a full-band signal is a different measurement, not a wider tolerance";
    };

    // A pass moves the envelope by hundreds of samples
    "a low-orbit pass moves the envelope delay by 672 samples, through the block"_test = [] {
        constexpr double fs      = 48'000.;
        constexpr double seconds = 600.;
        const double     change  = 7000. * seconds / kC; // 14.0097 ms
        const double     samples = change * fs;          // 672.5 at 48 kS/s

        const auto atTime = [&](std::int64_t timeNs) {
            return configured<RangeDelay<float>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, static_cast<std::int64_t>(seconds * 1e9)}}, {"schedule_delays_s", std::vector<double>{0., change}}, //
                {"sample_rate", static_cast<float>(fs)}, {"anchor_ns", timeNs}, {"order", 1}, {"bank_size", gr::Size_t{32U}}});
        };

        auto atStart = atTime(0LL);
        auto atEnd   = atTime(static_cast<std::int64_t>(seconds * 1e9));
        expect(that % (std::abs((atEnd->currentDelaySeconds() - atStart->currentDelaySeconds()) * 1e3 - 14.0097) < 1e-3)) << "the delay change is the pass geometry, read off the block";
        expect(that % (std::abs((atEnd->currentDelaySamples() - atStart->currentDelaySamples()) - 672.5) < 0.5)) << "672 samples is far past a symbol, so a chain without RangeDelay is visibly wrong";

        // and the stream moves by it too. One impulse sits at each end of the pass, and the outputs are that far apart.
        std::vector<float> impulse(2048UZ, 0.f);
        impulse[100UZ]        = 1.f;
        const auto   headOut  = runWhole<RangeDelay<float>, float>(*atStart, std::span<const float>(impulse));
        const auto   tailOut  = runWhole<RangeDelay<float>, float>(*atEnd, std::span<const float>(impulse));
        const double measured = static_cast<double>(peakIndex(std::span<const float>(tailOut))) - static_cast<double>(peakIndex(std::span<const float>(headOut)));
        std::println("[range] a {:.0f} s pass at 7 km/s moves the envelope {:.4f} ms, {:.1f} samples at {:.0f} S/s; the block's own impulse moved {:.0f}", seconds, change * 1e3, samples, fs, measured);
        expect(that % (std::abs(measured - samples) <= 1.)) << std::format("the impulse moved {} samples against the {:.1f} the pass commands", measured, samples);
    };

    // The chunk boundaries cannot change a sample, on the raw-span overload
    "the delayed stream is bit-identical however it is chunked"_test = [] {
        // a knot at exactly sample 1000, so a chunk boundary lands on it at chunk 1000
        constexpr double      fs = 1'000.;
        constexpr std::size_t n  = 20'000UZ;

        const gr::property_map settings{{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL, 4'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{0.01, 0.02, 0.005}}, //
            {"sample_rate", static_cast<float>(fs)}, {"order", 3}, {"bank_size", gr::Size_t{32U}}};

        const auto           input     = bandLimited(n);
        auto                 whole     = configured<RangeDelay<C>>(settings);
        const std::vector<C> reference = runChunked<RangeDelay<C>, C>(*whole, std::span<const C>(input), n);

        for (const std::size_t chunk : {1UZ, 7UZ, 1000UZ, 12345UZ}) {
            auto       block  = configured<RangeDelay<C>>(settings);
            const auto result = runChunked<RangeDelay<C>, C>(*block, std::span<const C>(input), chunk);
            expect(std::ranges::equal(result, reference)) << std::format("chunk {}: bit-identical, not near", chunk);
        }
    };

    // And on the tag-aware overload, with a chunk boundary on the trigger tag itself
    "both blocks are chunk independent through the tag-aware overload"_test = [] {
        constexpr double      fs = 1'000.;
        constexpr std::size_t n  = 8'000UZ;
        constexpr std::size_t at = 1'000UZ; // the trigger, and a chunk boundary at chunk 1000

        const std::vector<gr::Tag> tags{triggerTag(at, 500'000'000ULL), gr::Tag{2'000UZ, gr::property_map{{"marker", std::int64_t{7}}}}};
        const auto                 input = bandLimited(n);

        const gr::property_map delaySettings{{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL, 4'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{0.01, 0.02, 0.005}}, //
            {"sample_rate", static_cast<float>(fs)}, {"order", 3}, {"bank_size", gr::Size_t{32U}}, {"anchor_source", std::string("first_trigger")}};
        const gr::property_map shiftSettings{{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL, 4'000'000'000LL}}, {"schedule_offsets_hz", std::vector<double>{100., 200., -50.}}, //
            {"sample_rate", static_cast<float>(fs)}, {"anchor_source", std::string("first_trigger")}};

        auto       delayWhole = configured<RangeDelay<C>>(delaySettings);
        const auto delayRef   = drive<C>(*delayWhole, std::span<const C>(input), std::span<const gr::Tag>(tags), 0UZ);
        auto       shiftWhole = configured<DopplerShift<C>>(shiftSettings);
        const auto shiftRef   = drive<C>(*shiftWhole, std::span<const C>(input), std::span<const gr::Tag>(tags), 0UZ);
        expect(that % (delayRef.tags.size() == 2UZ)) << "the trigger stays where it arrived and the marker rides with its sample";

        for (const std::size_t chunk : {1UZ, 7UZ, 1000UZ, 12345UZ}) {
            auto       delayBlock = configured<RangeDelay<C>>(delaySettings);
            const auto delayed    = drive<C>(*delayBlock, std::span<const C>(input), std::span<const gr::Tag>(tags), chunk);
            expect(std::ranges::equal(delayed.samples, delayRef.samples)) << std::format("RangeDelay chunk {}: bit-identical samples", chunk);
            expect(that % (delayed.tags.size() == delayRef.tags.size())) << std::format("RangeDelay chunk {}: the same tags", chunk);
            for (std::size_t k = 0UZ; k < delayed.tags.size() && k < delayRef.tags.size(); ++k) {
                expect(that % (delayed.tags[k].index == delayRef.tags[k].index)) << std::format("RangeDelay chunk {}: tag {} at the same output sample", chunk, k);
            }

            auto       shiftBlock = configured<DopplerShift<C>>(shiftSettings);
            const auto shifted    = drive<C>(*shiftBlock, std::span<const C>(input), std::span<const gr::Tag>(tags), chunk);
            expect(std::ranges::equal(shifted.samples, shiftRef.samples)) << std::format("DopplerShift chunk {}: bit-identical samples", chunk);
        }
    };

    // A trigger anchor and a hand-set anchor at the same place are the same anchor
    "an armed trigger anchor and the setting that names the same place agree"_test = [] {
        constexpr double        fs  = 1'000.;
        constexpr std::size_t   n   = 6'000UZ;
        constexpr std::size_t   at  = 500UZ;
        constexpr std::uint64_t tNs = 250'000'000ULL;

        const auto             input = bandLimited(n);
        const gr::property_map table{{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL, 4'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{0.01, 0.02, 0.005}}, //
            {"sample_rate", static_cast<float>(fs)}, {"order", 3}, {"bank_size", gr::Size_t{32U}}};

        auto fromTrigger = configured<RangeDelay<C>>([&] {
            auto s             = table;
            s["anchor_source"] = std::string("first_trigger");
            return s;
        }());
        auto fromSetting = configured<RangeDelay<C>>([&] {
            auto s            = table;
            s["anchor_index"] = std::uint64_t{at};
            s["anchor_ns"]    = static_cast<std::int64_t>(tNs);
            return s;
        }());

        const std::vector<gr::Tag> tags{triggerTag(at, tNs)};
        const auto                 triggered = drive<C>(*fromTrigger, std::span<const C>(input), std::span<const gr::Tag>(tags), 512UZ);
        const auto                 hand      = drive<C>(*fromSetting, std::span<const C>(input), std::span<const gr::Tag>(tags), 512UZ);

        // Before the tag, the trigger-anchored block has no time and passes through. Its line has been fed nothing.
        // The two streams meet only after that line has filled. The transient is the history the line holds.
        const std::size_t settled = at + fromTrigger->historySamples();
        std::size_t       first   = 0UZ;
        for (std::size_t k = settled; k < n; ++k) {
            if (triggered.samples[k] != hand.samples[k]) {
                first = k;
                break;
            }
        }
        std::println("[range] trigger anchor: the trigger-armed line settles into the hand-set one after {} samples, its own history; first disagreement past that {}", fromTrigger->historySamples(), first);
        expect(that % (first == 0UZ)) << "past the arming transient the two anchors give the same stream, to the bit";
        expect(that % (fromTrigger->anchorIndex() == static_cast<std::uint64_t>(at))) << "the tag's own stream index becomes the anchor index";
        expect(that % (fromTrigger->anchorNs() == static_cast<std::int64_t>(tNs))) << "and the tag's value becomes the anchor time";

        // gr:trigger_offset, whose float spacing is 0.116 ns at 1 ms and 119.2 ns at 1 s, so both are exact here
        auto                       milli = configured<RangeDelay<C>>([&] {
            auto s             = table;
            s["anchor_source"] = std::string("first_trigger");
            return s;
        }());
        const std::vector<gr::Tag> milliTags{triggerTag(at, tNs, 1e-3f)};
        std::ignore = drive<C>(*milli, std::span<const C>(input).first(2048UZ), std::span<const gr::Tag>(milliTags), 512UZ);
        expect(that % (milli->anchorNs() == static_cast<std::int64_t>(tNs) + 1'000'000LL)) << "a millisecond of trigger_offset is exactly a million nanoseconds";

        auto                       whole = configured<RangeDelay<C>>([&] {
            auto s             = table;
            s["anchor_source"] = std::string("first_trigger");
            return s;
        }());
        const std::vector<gr::Tag> wholeTags{triggerTag(at, tNs, 1.f)};
        std::ignore = drive<C>(*whole, std::span<const C>(input).first(2048UZ), std::span<const gr::Tag>(wholeTags), 512UZ);
        expect(that % (whole->anchorNs() == static_cast<std::int64_t>(tNs) + std::llround(1e9 * static_cast<double>(1.f)))) << "and a second is exactly a billion, its own float spacing there being 119.2 ns";

        auto ignoring = configured<RangeDelay<C>>([&] {
            auto s                    = table;
            s["anchor_source"]        = std::string("first_trigger");
            s["honor_trigger_offset"] = false;
            return s;
        }());
        std::ignore   = drive<C>(*ignoring, std::span<const C>(input).first(2048UZ), std::span<const gr::Tag>(milliTags), 512UZ);
        expect(that % (ignoring->anchorNs() == static_cast<std::int64_t>(tNs))) << "and it is not read at all where honor_trigger_offset is off";
    };

    // The shift's half. The phasor restarts at the tag. The trigger anchor matches the setting on the schedule,
    // not on the phase.
    "the shift's trigger anchor agrees with the setting on the schedule, and states the phase it restarts"_test = [] {
        constexpr double        fs  = 1'000.;
        constexpr std::size_t   n   = 4'000UZ;
        constexpr std::size_t   at  = 500UZ;
        constexpr std::uint64_t tNs = 250'000'000ULL;

        const std::vector<C>   input(n, C(1.f, 0.f));
        const gr::property_map table{{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL, 4'000'000'000LL}}, {"schedule_offsets_hz", std::vector<double>{20., 40., -10.}}, {"sample_rate", static_cast<float>(fs)}};

        auto                       fromTrigger = configured<DopplerShift<C>>([&] {
            auto s             = table;
            s["anchor_source"] = std::string("first_trigger");
            return s;
        }());
        auto                       fromSetting = configured<DopplerShift<C>>([&] {
            auto s            = table;
            s["anchor_index"] = std::uint64_t{at};
            s["anchor_ns"]    = static_cast<std::int64_t>(tNs);
            return s;
        }());
        const std::vector<gr::Tag> tags{triggerTag(at, tNs)};
        const auto                 triggered = drive<C>(*fromTrigger, std::span<const C>(input), std::span<const gr::Tag>(tags), 512UZ);
        const auto                 hand      = drive<C>(*fromSetting, std::span<const C>(input), std::span<const gr::Tag>(tags), 512UZ);

        expect(that % (fromTrigger->anchorNs() == fromSetting->anchorNs())) << "the two anchors name the same instant";
        expect(that % (fromTrigger->currentOffsetHz() == fromSetting->currentOffsetHz())) << "so the schedule is read at the same place at the same sample";

        // The two streams are not bit-identical. The trigger-armed block passes through until the tag and restarts
        // its phase there. The pair differs by one constant rotation from the tag onward. The test asserts that the
        // rotation is constant, which means the schedules agree.
        const std::complex<double> reference = std::complex<double>(static_cast<double>(triggered.samples[at].real()), static_cast<double>(triggered.samples[at].imag())) * std::conj(std::complex<double>(static_cast<double>(hand.samples[at].real()), static_cast<double>(hand.samples[at].imag())));
        double                     worst     = 0.;
        for (std::size_t k = at; k < n; ++k) {
            const std::complex<double> ratio = std::complex<double>(static_cast<double>(triggered.samples[k].real()), static_cast<double>(triggered.samples[k].imag())) * std::conj(std::complex<double>(static_cast<double>(hand.samples[k].real()), static_cast<double>(hand.samples[k].imag())));
            worst                            = std::max(worst, std::abs(ratio - reference));
        }
        std::println("[range] shift trigger anchor: the trigger-armed and hand-set streams differ by one fixed rotation of {:.4f} rad, held to {:.3e} over {} samples", std::arg(reference), worst, n - at);
        expect(that % (worst < 1e-5)) << std::format("the difference is a constant rotation, not a drift: {:.3e}", worst);
        expect(that % (std::abs(std::abs(reference) - 1.) < 1e-5)) << "and a rotation, not a gain";
    };

    // A second tag moves no anchor, and the counters record it
    "a second trigger is ignored under first_trigger and re-anchors under every_trigger"_test = [] {
        constexpr double        fs   = 1'000.;
        constexpr std::size_t   n    = 4'000UZ;
        constexpr std::size_t   at   = 500UZ;
        constexpr std::size_t   at2  = 2'000UZ;
        constexpr std::uint64_t tNs  = 250'000'000ULL;
        constexpr std::uint64_t tNs2 = 900'000'000ULL;

        const std::vector<C> input(n, C(1.f, 0.f));
        // 1.5 Hz over the 1500 samples between the two tags is 2.25 turns. The continuous run's phase at the second
        // tag is a quarter turn from zero. A restart at that sample therefore shows in the output.
        const gr::property_map table{{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL, 4'000'000'000LL}}, {"schedule_offsets_hz", std::vector<double>{1.5, 1.5, 1.5}}, {"sample_rate", static_cast<float>(fs)}};

        const std::vector<gr::Tag> one{triggerTag(at, tNs)};
        const std::vector<gr::Tag> two{triggerTag(at, tNs), triggerTag(at2, tNs2)};

        auto       once   = configured<DopplerShift<C>>([&] {
            auto s             = table;
            s["anchor_source"] = std::string("first_trigger");
            return s;
        }());
        auto       twice  = configured<DopplerShift<C>>([&] {
            auto s             = table;
            s["anchor_source"] = std::string("first_trigger");
            return s;
        }());
        const auto single = drive<C>(*once, std::span<const C>(input), std::span<const gr::Tag>(one), 512UZ);
        const auto second = drive<C>(*twice, std::span<const C>(input), std::span<const gr::Tag>(two), 512UZ);
        expect(std::ranges::equal(single.samples, second.samples)) << "a second trigger under first_trigger leaves the stream where it was";
        expect(that % (twice->nIgnoredAnchors() == 1ULL)) << "and is counted rather than silently dropped";
        expect(that % (twice->anchorNs() == static_cast<std::int64_t>(tNs))) << "the anchor stays on the first tag";

        // every_trigger takes the second one, and says so by restarting the phase at that sample
        auto       every = configured<DopplerShift<C>>([&] {
            auto s             = table;
            s["anchor_source"] = std::string("every_trigger");
            return s;
        }());
        const auto burst = drive<C>(*every, std::span<const C>(input), std::span<const gr::Tag>(two), 512UZ);
        expect(that % (every->nReanchors() == 1ULL)) << "the second tag is a re-anchoring, counted";
        expect(that % (every->anchorNs() == static_cast<std::int64_t>(tNs2))) << "and the anchor moved to it";
        expect(!std::ranges::equal(burst.samples, single.samples)) << "the two modes must actually differ for the comparison to mean anything";
        expect(that % (std::abs(std::arg(std::complex<double>(static_cast<double>(burst.samples[at2].real()), static_cast<double>(burst.samples[at2].imag())))) < 0.02)) << "the phasor restarts at zero phase on the tag";
        expect(that % (std::abs(std::arg(std::complex<double>(static_cast<double>(single.samples[at2].real()), static_cast<double>(single.samples[at2].imag())))) > 0.5)) << "where the continuous run has accumulated a phase by then";

        // a trigger time off the nanosecond axis, and one that is not a nanosecond count at all
        const std::vector<gr::Tag> offAxis{gr::Tag{at, gr::property_map{{gr::property_map::key_type(gr::tag::TRIGGER_TIME.key()), std::uint64_t{1ULL << 63U}}}}};
        auto                       refusing   = configured<DopplerShift<C>>([&] {
            auto s             = table;
            s["anchor_source"] = std::string("first_trigger");
            return s;
        }());
        const auto                 refusedRun = drive<C>(*refusing, std::span<const C>(input), std::span<const gr::Tag>(offAxis), 512UZ);
        expect(that % (refusing->nRefusedAnchors() == 1ULL)) << "a trigger time past 2^63 - 1 ns is refused and counted";
        expect(!refusing->anchorArmed()) << "and leaves the anchor where it was, which is unarmed";
        expect(std::ranges::equal(refusedRun.samples, input)) << "so the stream passes through unshifted";

        const std::vector<gr::Tag> mistyped{gr::Tag{at, gr::property_map{{gr::property_map::key_type(gr::tag::TRIGGER_TIME.key()), double{2.5e17}}}}};
        auto                       unreadable = configured<DopplerShift<C>>([&] {
            auto s             = table;
            s["anchor_source"] = std::string("first_trigger");
            return s;
        }());
        std::ignore                           = drive<C>(*unreadable, std::span<const C>(input), std::span<const gr::Tag>(mistyped), 512UZ);
        expect(that % (unreadable->nRefusedAnchors() == 1ULL)) << "a trigger_time that is not a nanosecond count is refused, never read as one";
        expect(!unreadable->anchorArmed()) << "and never arms the anchor at the epoch";

        // the setting mode sees the tags too, and counts every one of them
        auto settingMode = configured<DopplerShift<C>>(table);
        std::ignore      = drive<C>(*settingMode, std::span<const C>(input), std::span<const gr::Tag>(two), 512UZ);
        expect(that % (settingMode->nIgnoredAnchors() == 2ULL)) << "a hand-set anchor beside a tagged source is visible in the count";
    };

    // A tag moves with the sample it marks, in the fixed point of the read cursor
    "a tag moves to the output sample that carries its input sample"_test = [] {
        constexpr std::size_t n      = 512UZ;
        constexpr double      kDelay = 10.25; // samples, at a rate of one sample per second

        auto         block   = configured<RangeDelay<float>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{kDelay, kDelay}}, {"sample_rate", 1.f}, {"order", 1}, {"bank_size", gr::Size_t{32U}}});
        const double latency = block->latencySamples();

        const std::vector<float>   input(n, 1.f);
        const std::vector<gr::Tag> tags{gr::Tag{100UZ, gr::property_map{{"marker", std::int64_t{1}}}}, //
            gr::Tag{200UZ, gr::property_map{{gr::property_map::key_type(gr::tag::FREQUENCY.key()), 437e6}}}};
        const auto                 driven = drive<float>(*block, std::span<const float>(input), std::span<const gr::Tag>(tags), 64UZ);

        const std::size_t expected = 100UZ + static_cast<std::size_t>(std::ceil(kDelay + latency));
        expect(that % (driven.tags.size() == 2UZ)) << "both tags are emitted exactly once";
        expect(that % (driven.tags[0].index == expected)) << std::format("the marker moves to the first output sample whose read position reaches its own: {} against {}", driven.tags[0].index, expected);
        expect(that % (driven.tags[1].index == 200UZ)) << "a reserved key describes the stream rather than a place in it, and stays where it arrived";

        // the same mapping however the stream is cut
        for (const std::size_t chunk : {1UZ, 7UZ, 1000UZ}) {
            auto       again  = configured<RangeDelay<float>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{kDelay, kDelay}}, {"sample_rate", 1.f}, {"order", 1}, {"bank_size", gr::Size_t{32U}}});
            const auto result = drive<float>(*again, std::span<const float>(input), std::span<const gr::Tag>(tags), chunk);
            expect(that % (result.tags.size() == 2UZ)) << std::format("chunk {}: both tags", chunk);
            expect(that % (result.tags[0].index == expected)) << std::format("chunk {}: the same output sample", chunk);
        }
    };

    // A delay that shrinks fast enough skips an input index, and the tags that land together are counted
    "tags that reach one output sample are attached together and counted"_test = [] {
        constexpr std::size_t n = 256UZ;
        // the delay falls 0.9 s per second at one sample per second, so the read position advances 1.9 per output
        auto         block   = configured<RangeDelay<float>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 100'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{100., 10.}}, {"sample_rate", 1.f}, {"order", 1}, {"bank_size", gr::Size_t{32U}}});
        const double latency = block->latencySamples();

        const std::vector<float> input(n, 1.f);
        std::vector<gr::Tag>     tags;
        for (const std::size_t index : {60UZ, 61UZ, 62UZ, 63UZ, 64UZ}) {
            tags.push_back(gr::Tag{index, gr::property_map{{"marker", static_cast<std::int64_t>(index)}}});
        }
        const auto driven = drive<float>(*block, std::span<const float>(input), std::span<const gr::Tag>(tags), 32UZ);

        // The read position in closed form is `k - D(k) - latency`. The table gives `D(k) = 100 - 0.9*k` seconds at
        // one sample per second, held at its last value past the last knot.
        const auto readPosition = [&](std::size_t k) { return static_cast<double>(k) - std::max(10., 100. - 0.9 * static_cast<double>(k)) - latency; };
        expect(that % (driven.tags.size() == tags.size())) << "every tag is emitted, none dropped by the skip";

        std::size_t coalesced = 0UZ; // output samples carrying more than one tag, the quantity the counter counts
        std::size_t run       = 0UZ;
        std::size_t lastIndex = 0UZ;
        for (std::size_t t = 0UZ; t < driven.tags.size() && t < tags.size(); ++t) {
            const std::size_t k = driven.tags[t].index;
            expect(that % (readPosition(k) >= static_cast<double>(tags[t].index))) << std::format("tag {} at output {} has its sample", t, k);
            expect(that % (readPosition(k - 1UZ) < static_cast<double>(tags[t].index))) << std::format("tag {} at output {} is the first that does", t, k);
            run = (t > 0UZ && k == lastIndex) ? run + 1UZ : 0UZ;
            if (run == 1UZ) {
                ++coalesced;
            }
            lastIndex = k;
        }
        std::println("[range] a read position advancing 1.9 samples per output puts {} of {} tags on a sample another already holds; the block counted {}", coalesced, tags.size(), block->nTagsCoalesced());
        expect(that % (coalesced > 0UZ)) << "the scene must actually coalesce for the counter to be measured";
        expect(that % (block->nTagsCoalesced() == static_cast<std::uint64_t>(coalesced))) << "and every such sample is counted once";
    };

    // The reserved keys' reports about the stream are counted and not acted on
    "a gap and a disagreeing rate are counted where they pass"_test = [] {
        constexpr std::size_t n     = 256UZ;
        auto                  block = configured<RangeDelay<float>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{1e-3, 1e-3}}, {"sample_rate", 1.e3f}, {"order", 1}, {"bank_size", gr::Size_t{32U}}});

        const std::vector<float>   input(n, 1.f);
        const std::vector<gr::Tag> tags{gr::Tag{50UZ, gr::property_map{{gr::property_map::key_type(gr::tag::N_DROPPED_SAMPLES.key()), gr::Size_t{17U}}}}, //
            gr::Tag{100UZ, gr::property_map{{gr::property_map::key_type(gr::tag::SAMPLE_RATE.key()), 2.e3f}}},                                            //
            gr::Tag{150UZ, gr::property_map{{gr::property_map::key_type(gr::tag::SAMPLE_RATE.key()), 1.e3f}}}};
        const auto                 driven = drive<float>(*block, std::span<const float>(input), std::span<const gr::Tag>(tags), 64UZ);

        expect(that % (block->nDiscontinuities() == 1ULL)) << "a dropped-sample marker is a gap the schedule's mapping cannot bridge, counted";
        expect(that % (block->nRateDisagreements() == 1ULL)) << "a stream rate that is not the staged one is counted";
        expect(that % (std::abs(block->currentDelaySamples() - 1e-3 * 1e3) < 1e-12)) << std::format("the staged rate converts the delay to samples: {}", block->currentDelaySamples());
        expect(that % (driven.tags.size() == 3UZ)) << "and all three are forwarded, at the index they arrived on";
        expect(that % (driven.tags[0].index == 50UZ && driven.tags[1].index == 100UZ && driven.tags[2].index == 150UZ));
    };

    // In a graph the framework reads the tags too, and the end of the stream releases the tags still waiting
    "a tag still waiting at the end of the stream leaves at the end-of-stream index"_test = [] {
        using gr::blocks::testing::ProcessFunction;
        using gr::blocks::testing::TagSource;

        constexpr gr::Size_t kSamples = 256U;
        constexpr float      kRate    = 1'000.f;
        constexpr double     kDelay   = 0.01; // ten samples, so a tag on input 250 has no output sample in the stream

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", kSamples}, {"mark_tag", false}, {"sample_rate", kRate}});
        source._tags     = {gr::Tag{50UZ, {{gr::property_map::key_type(gr::tag::N_DROPPED_SAMPLES.key()), gr::Size_t{17U}}}}, //
                gr::Tag{100UZ, {{gr::property_map::key_type(gr::tag::SAMPLE_RATE.key()), 2.f * kRate}}},                      //
                gr::Tag{150UZ, {{gr::property_map::key_type(gr::tag::SAMPLE_RATE.key()), kRate}}},                            //
                gr::Tag{250UZ, {{"marker", std::int64_t{7}}}}};
        auto& block      = graph.emplaceBlock<RangeDelay<float>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{kDelay, kDelay}}, {"sample_rate", kRate}, {"order", 1}, {"bank_size", gr::Size_t{32U}}});
        auto& sink       = graph.emplaceBlock<EndSink<float>>();
        expect(graph.connect<"out", "in">(source, block).has_value());
        expect(graph.connect<"out", "in">(block, sink).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        expect(that % (sink._samples.size() == static_cast<std::size_t>(kSamples))) << "every output";
        expect(that % (sink.indicesOf("marker") == std::vector<std::size_t>{static_cast<std::size_t>(kSamples)})) << "the marker leaves once, at the end-of-stream index";
        expect(that % (sink.indicesOf(gr::tag::N_DROPPED_SAMPLES.key()) == std::vector<std::size_t>{50UZ})) << "a reserved key stays on its sample";
        expect(that % (block.nRateDisagreements() == 1ULL)) << "the tagged rate that differs from the setting is counted";
        expect(that % (std::abs(block.currentDelaySamples() - kDelay * static_cast<double>(kRate)) < 1e-9)) << std::format("and the configured rate converts the delay: {} samples", block.currentDelaySamples());
    };

    // A change refused while the block runs leaves every sample where an unchanged block puts it
    "a refused change that names a rate leaves the delay at the accepted rate"_test = [] {
        const gr::property_map settings{{"schedule_times_ns", std::vector<std::int64_t>{0LL, 100'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{0.01, 0.01}}, {"sample_rate", 1'000.f}, {"order", 1}, {"bank_size", gr::Size_t{32U}}};
        std::vector<float>     impulses(512UZ, 0.f);
        impulses[100UZ] = 1.f;
        impulses[356UZ] = 1.f;

        auto                     unchanged = configured<RangeDelay<float>>(settings);
        const std::vector<float> reference = runChunked<RangeDelay<float>, float>(*unchanged, std::span<const float>(impulses), 256UZ);

        // The rate alone is valid. The direction in the same change is not, and the block refuses the whole change.
        for (const float rate : {500.f, 2'000.f}) {
            auto               block = configured<RangeDelay<float>>(settings);
            std::vector<float> out(impulses.size());
            const auto         first = block->processBulk(std::span<const float>(impulses).first(256UZ), std::span<float>(out).first(256UZ));
            expect(refusedLive(*block, {{"sample_rate", rate}, {"direction", std::string("undo")}})) << std::format("{} Hz beside an unknown direction", rate);
            const auto second = block->processBulk(std::span<const float>(impulses).subspan(256UZ), std::span<float>(out).subspan(256UZ));
            expect(first == gr::work::Status::OK && second == gr::work::Status::OK) << std::format("{} Hz: both calls run", rate);
            expect(std::ranges::equal(out, reference)) << std::format("{} Hz: the impulse leaves where the accepted rate puts it", rate);
            expect(that % (block->currentDelaySamples() == unchanged->currentDelaySamples())) << std::format("{} Hz: the reported delay is the accepted one", rate);
        }

        // A value outside a setting's limits is refused, and the member keeps the accepted value.
        auto               block = configured<RangeDelay<float>>(settings);
        std::vector<float> out(impulses.size());
        std::ignore = block->processBulk(std::span<const float>(impulses).first(256UZ), std::span<float>(out).first(256UZ));
        expect(refusedLive(*block, {{"sample_rate", 0.f}})) << "a rate of zero";
        expect(refusedLive(*block, {{"order", 2}})) << "an order the line has no blend for";
        expect(refusedLive(*block, {{"attenuation_db", 10.0}})) << "an attenuation below the design range";
        expect(refusedLive(*block, {{"rolloff", 1.0}})) << "a rolloff of one";
        expect(refusedLive(*block, {{"bank_size", gr::Size_t{48U}}})) << "a bank that is not a power of two";
        expect(that % (block->sample_rate.value == 1'000.f && block->order.value == 1 && block->attenuation_db.value == 60.0 && block->rolloff.value == 0.2 && block->bank_size.value == 32U)) << "every member keeps its accepted value";
        std::ignore = block->processBulk(std::span<const float>(impulses).subspan(256UZ), std::span<float>(out).subspan(256UZ));
        expect(std::ranges::equal(out, reference)) << "and the samples are those of the unchanged block";
        expect(gr::blocks::testing::settingsOutsideLimits(RangeDelay<float>{}).empty()) << "every default lies inside its limits";
        expect(gr::blocks::testing::settingsOutsideLimits(DopplerShift<C>{}).empty()) << "the shift's defaults too";
    };

    // A change that leaves the anchor alone keeps a trigger anchor where its tag put it
    "an accepted change that leaves the anchor alone keeps a trigger anchor armed"_test = [] {
        constexpr std::size_t      n     = 1'000UZ;
        const auto                 input = bandLimited(3UZ * n);
        const std::vector<gr::Tag> trigger{triggerTag(0UZ, 0ULL)};
        const std::vector<gr::Tag> none{};

        // A trigger at sample 0 and time 0 names the place the default setting names. The two blocks then agree to
        // the bit, and they must still agree after each change.
        const auto compare = [&]<typename TBlock>(gr::property_map table, gr::property_map tableChange, const char* what) {
            auto fromTrigger = configured<TBlock>([&] {
                auto s             = table;
                s["anchor_source"] = std::string("first_trigger");
                return s;
            }());
            auto fromSetting = configured<TBlock>(table);
            for (std::size_t part = 0UZ; part < 3UZ; ++part) {
                if (part == 1UZ) {
                    expect(!refusedLive(*fromTrigger, tableChange) && !refusedLive(*fromSetting, tableChange)) << what << ": a new table";
                } else if (part == 2UZ) {
                    expect(!refusedLive(*fromTrigger, {{"sample_rate", 2'000.f}}) && !refusedLive(*fromSetting, {{"sample_rate", 2'000.f}})) << what << ": a new rate";
                }
                const std::span<const C>       chunk = std::span<const C>(input).subspan(part * n, n);
                const std::span<const gr::Tag> tags  = part == 0UZ ? std::span<const gr::Tag>(trigger) : std::span<const gr::Tag>(none);
                const auto                     a     = drive<C>(*fromTrigger, chunk, tags, 256UZ);
                const auto                     b     = drive<C>(*fromSetting, chunk, tags, 256UZ);
                expect(std::ranges::equal(a.samples, b.samples)) << std::format("{}: part {} agrees to the bit", what, part);
                expect(fromTrigger->anchorArmed()) << std::format("{}: part {} is armed", what, part);
            }
        };
        compare.template operator()<RangeDelay<C>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 10'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{0.01, 0.01}}, {"sample_rate", 1'000.f}, {"order", 3}, {"bank_size", gr::Size_t{32U}}}, //
            {{"schedule_delays_s", std::vector<double>{0.02, 0.02}}}, "RangeDelay");
        compare.template operator()<DopplerShift<C>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 10'000'000'000LL}}, {"schedule_offsets_hz", std::vector<double>{100., 100.}}, {"sample_rate", 1'000.f}}, //
            {{"schedule_offsets_hz", std::vector<double>{120., 120.}}}, "DopplerShift");
    };

    // A refused change leaves a trigger anchor armed and the table in force
    "a refused change leaves a trigger anchor and its table in force"_test = [] {
        constexpr std::size_t      n = 1'000UZ;
        const std::vector<C>       input(2UZ * n, C(1.f, 0.f));
        const std::vector<gr::Tag> trigger{triggerTag(0UZ, 0ULL)};
        const gr::property_map     table{{"schedule_times_ns", std::vector<std::int64_t>{0LL, 100'000'000'000LL}}, {"schedule_offsets_hz", std::vector<double>{100., 100.}}, {"sample_rate", 1'000.f}, {"anchor_source", std::string("first_trigger")}};

        auto       unchanged = configured<DopplerShift<C>>(table);
        const auto reference = drive<C>(*unchanged, std::span<const C>(input), std::span<const gr::Tag>(trigger), n);

        auto       block = configured<DopplerShift<C>>(table);
        const auto first = drive<C>(*block, std::span<const C>(input).first(n), std::span<const gr::Tag>(trigger), n);
        expect(refusedLive(*block, {{"schedule_file", std::string("/nonexistent.gr4traj")}, {"schedule_times_ns", std::vector<std::int64_t>{}}, {"schedule_offsets_hz", std::vector<double>{}}})) << "a file that does not load";
        const auto second = drive<C>(*block, std::span<const C>(input).subspan(n), std::span<const gr::Tag>(), n);
        expect(block->anchorArmed()) << "the anchor stays armed";
        expect(std::ranges::equal(first.samples, std::span<const C>(reference.samples).first(n)));
        expect(std::ranges::equal(second.samples, std::span<const C>(reference.samples).subspan(n))) << "and the shift goes on as if no change had been staged";

        // Under every_trigger a re-anchor after a refused rate builds its clock at the accepted rate.
        auto every = [&] {
            auto s             = table;
            s["anchor_source"] = std::string("every_trigger");
            return s;
        }();
        const std::vector<gr::Tag> twice{triggerTag(0UZ, 0ULL), triggerTag(n, 500'000'000ULL)};
        auto                       steady   = configured<DopplerShift<C>>(every);
        const auto                 expected = drive<C>(*steady, std::span<const C>(input), std::span<const gr::Tag>(twice), n);
        auto                       moved    = configured<DopplerShift<C>>(every);
        std::ignore                         = drive<C>(*moved, std::span<const C>(input).first(n), std::span<const gr::Tag>(twice).first(1UZ), n);
        expect(refusedLive(*moved, {{"sample_rate", 0.f}})) << "a rate of zero";
        const std::vector<gr::Tag> next{triggerTag(0UZ, 500'000'000ULL)};
        const auto                 after = drive<C>(*moved, std::span<const C>(input).subspan(n), std::span<const gr::Tag>(next), n);
        expect(that % (moved->nReanchors() == 1ULL)) << "the second tag re-anchors";
        expect(std::ranges::equal(after.samples, std::span<const C>(expected.samples).subspan(n))) << "on the clock of the accepted rate";
    };

    // A refused rate staged again beside the change that makes it valid is taken
    "a refused rate is written back, and staged again beside the change that makes it valid reaches the samples"_test = [] {
        const gr::property_map settings{{"schedule_times_ns", std::vector<std::int64_t>{0LL, 100'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{0.01, 0.01}}, {"sample_rate", 1'000.f}, {"order", 1}, {"bank_size", gr::Size_t{32U}}};
        std::vector<float>     impulse(256UZ, 0.f);
        impulse[100UZ] = 1.f;

        // The first change holds a rate and a direction the block refuses. The second holds the same rate and a good direction.
        auto block = configured<RangeDelay<float>>(settings);
        expect(refusedLive(*block, {{"sample_rate", 500.f}, {"direction", std::string("undo")}}));
        expect(that % (block->sample_rate.value == 1'000.f)) << "the accepted rate is written back to its member";
        expect(!refusedLive(*block, {{"sample_rate", 500.f}, {"direction", std::string("apply")}})) << "the same rate beside a valid direction is taken";
        expect(that % (block->sample_rate.value == 500.f));
        std::vector<float> out(impulse.size());
        std::ignore = block->processBulk(std::span<const float>(impulse), std::span<float>(out));

        // A block configured at 500 Hz from the start puts the impulse where this one must.
        auto               direct = configured<RangeDelay<float>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 100'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{0.01, 0.01}}, {"sample_rate", 500.f}, {"order", 1}, {"bank_size", gr::Size_t{32U}}});
        std::vector<float> expected(impulse.size());
        std::ignore = direct->processBulk(std::span<const float>(impulse), std::span<float>(expected));
        expect(std::ranges::equal(out, expected)) << "the impulse leaves where the 500 Hz line puts it";
        expect(that % (block->currentDelaySamples() == direct->currentDelaySamples())) << "and the reported delay is that of 500 Hz";
    };

    // A refused direction is written back, and a later change of the bias alone is checked against the accepted direction
    "a refused direction is written back, and a later change of another setting is taken"_test = [] {
        auto              block    = configured<RangeDelay<float>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 100'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{0.01, 0.01}}, {"sample_rate", 1'000.f}, {"order", 1}, {"bank_size", gr::Size_t{32U}}});
        const std::string accepted = block->direction.value;
        const std::string refusal  = refusalLive(*block, {{"direction", std::string("undo")}});
        expect(fatal(refusal.contains("'direction'"))) << refusal;
        expect(that % (block->direction.value == accepted)) << "the accepted direction is written back to its member";

        const std::string unrelated = refusalLive(*block, {{"bias_s", 0.001}});
        expect(unrelated.empty()) << std::format("a new bias is taken; the refusal: {}", unrelated);
        expect(that % (block->bias_s.value == 0.001 && block->biasSeconds() == 0.001)) << "the member and the block take the new bias";
    };

    // A change that arms the anchor starts the line on silence, as a tag that arms it does
    "an arm by a settings change starts the line on silence"_test = [] {
        constexpr std::size_t  n     = 300UZ;
        const auto             input = bandLimited(3UZ * n);
        const gr::property_map table{{"schedule_times_ns", std::vector<std::int64_t>{0LL, 100'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{0.01, 0.01}}, {"sample_rate", 1'000.f}, {"order", 1}, {"bank_size", gr::Size_t{32U}}};

        // The reference starts on silence at the input the arm meets. The table holds one delay. The position in time
        // does not matter.
        auto       fresh     = configured<RangeDelay<C>>(table);
        const auto reference = drive<C>(*fresh, std::span<const C>(input).subspan(2UZ * n), std::span<const gr::Tag>(), n);

        auto block  = configured<RangeDelay<C>>(table);
        std::ignore = drive<C>(*block, std::span<const C>(input).first(n), std::span<const gr::Tag>(), n);
        expect(!refusedLive(*block, {{"anchor_source", std::string("first_trigger")}}));
        expect(!block->anchorArmed());
        std::ignore = drive<C>(*block, std::span<const C>(input).subspan(n, n), std::span<const gr::Tag>(), n);
        expect(!refusedLive(*block, {{"anchor_source", std::string("setting")}}));
        expect(block->anchorArmed());
        const auto rearmed = drive<C>(*block, std::span<const C>(input).subspan(2UZ * n), std::span<const gr::Tag>(), n);
        expect(std::ranges::equal(rearmed.samples, reference.samples)) << "the line holds nothing from before the pass-through";
    };

    // A change that moves an armed anchor starts the line on silence, as a tag that moves it does
    "a re-anchor by a settings change starts the line on silence"_test = [] {
        constexpr std::size_t  n     = 300UZ;
        const auto             input = bandLimited(2UZ * n);
        const gr::property_map table{{"schedule_times_ns", std::vector<std::int64_t>{0LL, 100'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{0.01, 0.01}}, {"sample_rate", 1'000.f}, {"order", 1}, {"bank_size", gr::Size_t{32U}}};

        // The table holds one delay. A new anchor changes no delay and starts the line on silence.
        auto       fresh     = configured<RangeDelay<C>>(table);
        const auto reference = drive<C>(*fresh, std::span<const C>(input).subspan(n), std::span<const gr::Tag>(), n);
        auto       steady    = configured<RangeDelay<C>>(table);
        const auto unchanged = drive<C>(*steady, std::span<const C>(input), std::span<const gr::Tag>(), n);

        const std::vector<std::pair<gr::property_map, std::string>> moves{{{{"anchor_ns", std::int64_t{5'000'000'000LL}}}, "a new anchor time"}, {{{"anchor_index", std::uint64_t{100ULL}}}, "a new anchor index"}};
        for (const auto& [change, what] : moves) {
            auto block  = configured<RangeDelay<C>>(table);
            std::ignore = drive<C>(*block, std::span<const C>(input).first(n), std::span<const gr::Tag>(), n);
            expect(!refusedLive(*block, change)) << what;
            expect(block->anchorArmed()) << what;
            const auto moved = drive<C>(*block, std::span<const C>(input).subspan(n), std::span<const gr::Tag>(), n);
            expect(std::ranges::equal(moved.samples, reference.samples)) << what << ": the line holds nothing from before the move";
        }

        // A change that names an anchor setting and leaves the anchor in place keeps the history.
        auto block  = configured<RangeDelay<C>>(table);
        std::ignore = drive<C>(*block, std::span<const C>(input).first(n), std::span<const gr::Tag>(), n);
        expect(!refusedLive(*block, {{"honor_trigger_offset", false}}));
        const auto kept = drive<C>(*block, std::span<const C>(input).subspan(n), std::span<const gr::Tag>(), n);
        expect(std::ranges::equal(kept.samples, tail(unchanged, n))) << "the anchor in place keeps the line's history";
    };

    // An arm after pass-through starts the line on silence, as a block that never ran
    "an arm after a stretch of pass-through starts the line on silence"_test = [] {
        constexpr std::size_t  n     = 300UZ;
        const auto             input = bandLimited(3UZ * n);
        const gr::property_map table{{"schedule_times_ns", std::vector<std::int64_t>{0LL, 100'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{0.01, 0.01}}, {"sample_rate", 1'000.f}, {"order", 1}, {"bank_size", gr::Size_t{32U}}, {"anchor_source", std::string("first_trigger")}};

        // The reference sees one trigger, at sample 2n. Its line takes nothing before that tag.
        auto                       fresh = configured<RangeDelay<C>>(table);
        const std::vector<gr::Tag> late{triggerTag(2UZ * n, 600'000'000ULL)};
        const auto                 reference = drive<C>(*fresh, std::span<const C>(input), std::span<const gr::Tag>(late), n);

        // This block runs armed, is disarmed by a new anchor source, passes samples through, and is armed again at 2n.
        auto                       block = configured<RangeDelay<C>>(table);
        const std::vector<gr::Tag> early{triggerTag(0UZ, 0ULL)};
        std::ignore = drive<C>(*block, std::span<const C>(input).first(n), std::span<const gr::Tag>(early), n);
        expect(block->anchorArmed());
        expect(!refusedLive(*block, {{"anchor_source", std::string("every_trigger")}})) << "a new anchor source is accepted";
        expect(!block->anchorArmed()) << "and waits for its trigger";
        const auto passed = drive<C>(*block, std::span<const C>(input).subspan(n, n), std::span<const gr::Tag>(), n);
        expect(std::ranges::equal(passed.samples, std::span<const C>(input).subspan(n, n))) << "unarmed, the block passes its input through";
        const std::vector<gr::Tag> again{triggerTag(0UZ, 600'000'000ULL)};
        const auto                 rearmed = drive<C>(*block, std::span<const C>(input).subspan(2UZ * n), std::span<const gr::Tag>(again), n);
        expect(block->anchorArmed());
        expect(std::ranges::equal(rearmed.samples, tail(reference, 2UZ * n))) << "the re-armed line holds nothing from before the pass-through";
    };

    // A failed first application leaves start() with the accepted values, which are the defaults
    "start builds the line from the accepted values and never throws"_test = [] {
        auto block  = std::make_unique<RangeDelay<float>>(gr::property_map{{"direction", std::string("undo")}});
        bool failed = false;
        try {
            block->settings().init();
            failed = !block->settings().applyStagedParameters().failedParameters.empty();
        } catch (const std::exception&) {
            failed = true;
        }
        expect(failed) << "an unknown direction is refused";
        expect(nothrow([&] { block->start(); })) << "start never throws";
        expect(that % (block->historySamples() > 0UZ)) << "and the line exists";
        std::vector<float> input(64UZ, 1.f);
        std::vector<float> out(64UZ);
        expect(block->processBulk(std::span<const float>(input), std::span<float>(out)) == gr::work::Status::OK);
    };

    // A table or a rate keeps the history. A new bank drops it.
    "restaging a table keeps the line's history and the bank alone rebuilds it"_test = [] {
        constexpr std::size_t  n = 4'000UZ;
        const gr::property_map settings{{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{0.01, 0.02}}, //
            {"sample_rate", 1.e3f}, {"order", 3}, {"bank_size", gr::Size_t{32U}}};

        const auto           input     = bandLimited(n);
        auto                 straight  = configured<RangeDelay<C>>(settings);
        const std::vector<C> reference = runChunked<RangeDelay<C>, C>(*straight, std::span<const C>(input), n);

        auto           restaged = configured<RangeDelay<C>>(settings);
        std::vector<C> out(n);
        std::ignore = restaged->processBulk(std::span<const C>(input).first(2000UZ), std::span<C>(out.data(), 2000UZ));
        expect(restaged->schedulePosition() == SchedulePosition::After) << "two seconds in, the first table holds its last value";
        // A third knot at ten seconds holds the last value. The new table commands the same delay at every sample of
        // the run. It differs from the held table, and core passes it to settingsChanged().
        expect(!refusedLive(*restaged, {{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL, 10'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{0.01, 0.02, 0.02}}}));
        expect(restaged->schedulePosition() == SchedulePosition::Inside) << "the new table is in force";
        std::ignore = restaged->processBulk(std::span<const C>(input).subspan(2000UZ), std::span<C>(out.data() + 2000UZ, n - 2000UZ));

        expect(std::ranges::equal(out, reference)) << "a table that commands the same delays is the same stream: the history and the read cursor are kept";
        expect(that % (restaged->nBankRebuilds() == 0ULL)) << "and no bank was cut a second time";

        std::ignore = restaged->settings().setStaged({{"bank_size", gr::Size_t{128U}}});
        std::ignore = restaged->settings().applyStagedParameters();
        expect(that % (restaged->nBankRebuilds() == 1ULL)) << "a new bank is a new window, and the history it cost is counted";

        // a deeper table grows the history and keeps the bank
        auto              deeper = configured<RangeDelay<C>>(settings);
        const std::size_t held   = deeper->historySamples();
        std::ignore              = deeper->settings().setStaged({{"schedule_delays_s", std::vector<double>{0.01, 0.5}}});
        std::ignore              = deeper->settings().applyStagedParameters();
        expect(that % (deeper->historySamples() > held)) << "the line reaches as far back as the new table asks";
        expect(that % (deeper->nBankRebuilds() == 0ULL)) << "without the bank changing at all";
    };

    // Before its trigger the block has no time. It neither delays nor feeds its line.
    "an unarmed block passes its input through and reads unarmed"_test = [] {
        constexpr std::size_t n     = 512UZ;
        auto                  block = configured<RangeDelay<float>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{1e-3, 1e-3}}, //
                             {"sample_rate", 1.e6f}, {"anchor_source", std::string("first_trigger")}});
        expect(that % (block->schedulePosition() == SchedulePosition::Unarmed)) << "a trigger-armed block has no position until its trigger";
        expect(!block->anchorArmed());

        const auto input       = bandLimited(n);
        auto       passthrough = configured<RangeDelay<C>>({{"schedule_times_ns", std::vector<std::int64_t>{0LL, 1'000'000'000LL}}, {"schedule_delays_s", std::vector<double>{1e-3, 1e-3}}, //
                  {"sample_rate", 1.e6f}, {"anchor_source", std::string("first_trigger")}});
        const auto out         = runWhole<RangeDelay<C>, C>(*passthrough, std::span<const C>(input));
        expect(std::ranges::equal(out, input)) << "an unarmed block passes its input through unmodified";
        expect(that % (passthrough->currentDelaySeconds() == 0.)) << "and commands no delay, because it has no time to read the table at";
    };

    return 0;
}
