#include <boost/ut.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <print>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Message.hpp>

#include <gnuradio-4.0/basic/RateProbe.hpp>
#include <gnuradio-4.0/basic/Throttle.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::basic::RateProbe;
using gr::blocks::basic::Throttle;
namespace spans = gr::blocks::testing::span;

using Clock = std::chrono::steady_clock;

struct Report {
    double     rate     = 0.0;
    double     rateNow  = 0.0;
    gr::Size_t nSamples = 0U;
};

template<typename T>
void feed(RateProbe<T>& probe, std::span<const T> samples) {
    spans::InputSpan<T> inSpan{samples};
    std::ignore = probe.processBulk(inSpan);
}

[[nodiscard]] std::vector<Report> drain(gr::MsgPortIn& port) {
    std::vector<Report> reports;
    auto                messages = port.streamReader().get();
    for (const gr::Message& message : messages) {
        if (message.endpoint != "rate" || !message.data.has_value()) {
            continue;
        }
        const gr::property_map& map = message.data.value();
        reports.push_back({map.at("rate").value_or(0.0), map.at("rate_now").value_or(0.0), map.at("n_samples").value_or(gr::Size_t{0U})});
    }
    std::ignore = messages.consume(messages.size());
    return reports;
}

[[nodiscard]] double settingDouble(auto& probe, std::string_view key) {
    const auto value = probe.settings().get(std::string(key));
    return value.has_value() ? value->value_or(-1.0) : -1.0;
}

[[nodiscard]] gr::Size_t settingSize(auto& probe, std::string_view key) {
    const auto value = probe.settings().get(std::string(key));
    return value.has_value() ? value->value_or(gr::Size_t{0U}) : gr::Size_t{0U};
}

[[nodiscard]] double seconds(Clock::duration duration) { return std::chrono::duration<double>(duration).count(); }

} // namespace

const boost::ut::suite<"RateProbe"> rateProbeTests = [] {
    using namespace boost::ut;

    "n_samples counts across two work calls, and rate stays 0 before a full interval"_test = []<typename T>() {
        RateProbe<T> probe({{"min_update_ms", 3.6e6f}});
        probe.init(probe.progress);
        probe.start();

        const std::vector<T> first(3UZ);
        const std::vector<T> second(5UZ);
        feed<T>(probe, first);
        expect(eq(probe.n_samples.value, gr::Size_t{3U}));
        feed<T>(probe, second);
        expect(eq(probe.n_samples.value, gr::Size_t{8U})) << "the second call adds to the first";

        expect(eq(probe.rate.value, 0.0)) << "no interval has passed";
        expect(eq(probe.rate_now.value, 0.0));
        expect(eq(settingDouble(probe, "rate"), 0.0)) << "a settings Get reads 0 before the first measurement";
    } | std::tuple<std::uint8_t, std::int16_t, std::int32_t, std::uint32_t, float, double, std::complex<float>, std::complex<double>>{};

    "behind a Throttle, rate is the configured rate within the bound the schedule allows"_test = [] {
        // The test drives the Throttle and the probe in one loop. The Throttle returns a chunk once the chunk's last
        // sample is due. At its clock read t the probe has received R * (t - t0 - d) samples, where t0 is the
        // Throttle's origin and d is how late the call returns, between 0 and the largest lateness W observed. The
        // first interval opens at the probe's first call, whose lateness d_0 lies in the same range. The k-th
        // interval's rate is then R * (1 - (d_k - d_{k-1}) / T_k), with T_k at least min_update_ms. The smoothed rate
        // weighs the first interval by w_1 = (1 - alpha)^(K-1) and the k-th by w_k = alpha * (1 - alpha)^(K-k).
        // Summing by parts, and with alpha at most 1/2 so the weights rise from the second on, the relative error is
        // at most (w_1 - w_2 + w_K) * W / T, plus W * (the largest T_k - T) / T^2 for intervals longer than T.
        constexpr float       kRate       = 20000.f;
        constexpr std::size_t kChunk      = 200UZ;
        constexpr std::size_t kTotal      = 20000UZ;
        constexpr float       kIntervalMs = 100.f;
        constexpr float       kAlpha      = 0.15f;

        Throttle<float> throttle({{"sample_rate", kRate}});
        throttle.settings().init();
        std::ignore = throttle.settings().applyStagedParameters();

        RateProbe<float> probe({{"min_update_ms", kIntervalMs}, {"alpha", kAlpha}});
        probe.init(probe.progress);
        gr::MsgPortIn fromProbe;
        expect(fatal(probe.msgOut.connect(fromProbe).has_value()));

        const std::vector<float> input(kTotal, 1.f);
        std::vector<float>       output(kTotal);

        const Clock::time_point before = Clock::now();
        throttle.restart();
        probe.start();
        double      lateness = 0.0;
        std::size_t paced    = 0UZ;
        while (paced < kTotal) {
            const std::size_t        count = std::min(kChunk, kTotal - paced);
            spans::InputSpan<float>  inSpan{std::span<const float>(input).subspan(paced, count), paced};
            spans::OutputSpan<float> outSpan{std::span<float>(output).subspan(paced, count), paced};
            std::ignore = throttle.processBulk(inSpan, outSpan);
            expect(fatal(eq(inSpan.consumed, count))) << "the Throttle releases the whole chunk";

            feed<float>(probe, std::span<const float>(output).subspan(paced, count));
            paced += count;
            const double due = static_cast<double>(paced) / static_cast<double>(kRate);
            lateness         = std::max(lateness, seconds(Clock::now() - before) - due);
        }

        const std::vector<Report> reports  = drain(fromProbe);
        const std::size_t         nReports = reports.size();
        expect(fatal(ge(nReports, 2UZ))) << "the run spans several intervals";
        expect(eq(probe.n_samples.value, static_cast<gr::Size_t>(kTotal)));

        // the block smooths with its float alpha widened to double; the same value keeps the recomputation exact
        const double alpha    = static_cast<double>(probe.alpha.value);
        double       smoothed = reports.front().rateNow;
        expect(eq(reports.front().rate, smoothed)) << "the first measurement is taken as it is";
        for (std::size_t k = 1UZ; k < nReports; ++k) {
            smoothed = alpha * reports[k].rateNow + (1.0 - alpha) * smoothed;
            expect(le(std::abs(reports[k].rate - smoothed), 1e-9 * smoothed)) << std::format("measurement {} follows the smoothing rule", k);
        }
        expect(eq(probe.rate.value, reports.back().rate));
        expect(eq(settingDouble(probe, "rate"), reports.back().rate)) << "a settings Get reads the last measurement";
        expect(eq(settingDouble(probe, "rate_now"), reports.back().rateNow));
        expect(eq(settingSize(probe, "n_samples"), reports.back().nSamples));

        const double interval = static_cast<double>(kIntervalMs) / 1e3;
        const double weights  = alpha + std::pow(1.0 - alpha, static_cast<double>(nReports) - 2.0) * (1.0 - 2.0 * alpha);
        const double longest  = static_cast<double>(kChunk) / static_cast<double>(kRate) + lateness; // the largest T_k - T
        const double bound    = weights * lateness / interval + lateness * longest / (interval * interval);
        const double error    = std::abs(probe.rate.value / static_cast<double>(kRate) - 1.0);
        std::println("rate {} S/s over {} measurements, relative error {}, bound {} (lateness {} s)", probe.rate.value, nReports, error, bound, lateness);
        expect(le(error, bound)) << std::format("rate {} against {} S/s", probe.rate.value, kRate);
    };

    "min_update_ms bounds the number of measurements"_test = [] {
        constexpr std::size_t    kCalls = 50UZ;
        const std::vector<float> chunk(16UZ, 0.f);

        for (const float intervalMs : {0.f, 20.f}) {
            RateProbe<float> probe({{"min_update_ms", intervalMs}});
            probe.init(probe.progress);
            gr::MsgPortIn fromProbe;
            expect(fatal(probe.msgOut.connect(fromProbe).has_value()));

            const Clock::time_point begin = Clock::now();
            probe.start();
            for (std::size_t call = 0UZ; call < kCalls; ++call) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1)); // every call sees a nonzero interval
                feed<float>(probe, chunk);
            }
            const double elapsed = seconds(Clock::now() - begin);

            const std::vector<Report> reports = drain(fromProbe);
            expect(fatal(!reports.empty())) << std::format("min_update_ms {}: at least one measurement", intervalMs);
            if (intervalMs == 0.f) {
                expect(eq(reports.size(), kCalls - 1UZ)) << "with no minimum, every call after the first measures";
            } else {
                const auto bound = static_cast<std::size_t>(elapsed * 1e3 / static_cast<double>(intervalMs));
                expect(le(reports.size(), bound)) << std::format("{} measurements in {} s", reports.size(), elapsed);
                expect(lt(reports.size(), kCalls)) << "the minimum suppresses measurements";
            }
            for (std::size_t k = 1UZ; k < reports.size(); ++k) {
                expect(gt(reports[k].nSamples, reports[k - 1UZ].nSamples)) << "each measurement counts more samples";
            }
            expect(eq(settingSize(probe, "n_samples"), reports.back().nSamples)) << "a settings Get reads the last measurement";
        }
    };

    "the wait for the first sample stays out of the first interval"_test = [] {
        constexpr std::size_t    kSamples = 1000UZ;
        const std::vector<float> opening(16UZ, 0.f);
        const std::vector<float> chunk(kSamples, 0.f);

        RateProbe<float> probe({{"min_update_ms", 10.f}});
        probe.init(probe.progress);
        gr::MsgPortIn fromProbe;
        expect(fatal(probe.msgOut.connect(fromProbe).has_value()));

        probe.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        const Clock::time_point beforeFirst = Clock::now();
        feed<float>(probe, opening);
        const Clock::time_point afterFirst = Clock::now();
        expect(eq(probe.rate.value, 0.0)) << "the first samples open the interval";
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        const Clock::time_point beforeSecond = Clock::now();
        feed<float>(probe, chunk);
        const Clock::time_point afterSecond = Clock::now();

        const std::vector<Report> reports = drain(fromProbe);
        expect(fatal(eq(reports.size(), 1UZ))) << "one measurement";
        const double samples = static_cast<double>(kSamples);
        expect(ge(reports.front().rateNow, samples / seconds(afterSecond - beforeFirst))) << "the interval excludes the wait";
        expect(le(reports.front().rateNow, samples / seconds(beforeSecond - afterFirst))) << "and spans the two calls";
        expect(eq(reports.front().nSamples, static_cast<gr::Size_t>(opening.size() + kSamples))) << "n_samples counts the opening samples";
    };

    "a setting outside its range is refused"_test = [] {
        const auto make = [](gr::property_map settings) {
            RateProbe<float> probe(std::move(settings));
            probe.settings().init();
            std::ignore = probe.settings().applyStagedParameters();
        };
        expect(throws([&] { make({{"alpha", 0.f}}); })) << "alpha 0";
        expect(throws([&] { make({{"alpha", 1.5f}}); })) << "alpha above 1";
        expect(throws([&] { make({{"min_update_ms", -1.f}}); })) << "negative interval";
        expect(nothrow([&] { make({{"alpha", 1.f}, {"min_update_ms", 0.f}}); })) << "the range ends";
    };
};

int main() { /* not needed for UT */ }
