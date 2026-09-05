#include <gnuradio-4.0/testing/BenchInterleaved.hpp>

#include <algorithm>
#include <cstddef>
#include <format>
#include <functional>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/basic/DataSetToStream.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

// The per-record cost amortizes over the record's length: a one-sample record is all overhead and a million-sample
// record is all copy. Where between them the overhead stops mattering is what these arms measure, against a plain
// span copy as the floor. Signal count and event count are varied because both are per-record work.

namespace {

using gr::blocks::basic::DataSetToStream;
using gr::blocks::testing::bench::Arm;

constexpr std::size_t kTotalSamples = 1UZ << 22;
constexpr std::size_t kMaxRecords   = 1024UZ; ///< a short record would otherwise want millions of live DataSet objects
constexpr std::size_t kRepeats      = 7UZ;
constexpr std::size_t kLengths[]{1UZ, 8UZ, 64UZ, 1024UZ, 65536UZ, 1048576UZ};
constexpr std::size_t kSignalCounts[]{1UZ, 4UZ};
constexpr std::size_t kEventCounts[]{0UZ, 1UZ, 64UZ};

// A minimal span pair: the benchmark measures processBulk, not the scheduler around it.
using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;
using gr::blocks::testing::span::TagView;

[[nodiscard]] gr::DataSet<float> makeRecord(std::size_t nSamples, std::size_t nSignals, std::size_t nEvents) {
    gr::DataSet<float> record;
    record.extents.push_back(static_cast<std::int32_t>(nSamples));
    record.signal_values.resize(nSignals * nSamples);
    for (std::size_t signal = 0UZ; signal < nSignals; ++signal) {
        record.signal_names.push_back(std::format("signal{}", signal));
        record.signal_quantities.emplace_back("voltage");
        record.signal_units.emplace_back("V");
        record.signal_ranges.push_back(gr::Range<float>{-1.f, 1.f});
        record.meta_information.emplace_back();
        record.timing_events.emplace_back();
        for (std::size_t j = 0UZ; j < nSamples; ++j) {
            record.signal_values[signal * nSamples + j] = static_cast<float>(j);
        }
    }
    record.meta_information[0UZ].insert_or_assign(gr::property_map::key_type("sample_rate"), gr::pmt::Value(48000.f));
    for (std::size_t k = 0UZ; k < nEvents && k < nSamples; ++k) {
        record.timing_events[0UZ].emplace_back(static_cast<std::ptrdiff_t>(k * (nSamples / std::max(nEvents, 1UZ))), gr::property_map{{"event", static_cast<gr::Size_t>(k)}});
    }
    return record;
}

[[nodiscard]] DataSetToStream<float> makeBlock() {
    DataSetToStream<float> block{};
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

} // namespace

int main() {
    std::vector<float>              out(kTotalSamples);
    std::vector<float>              floor(kTotalSamples);
    std::vector<gr::DataSet<float>> rejectScratch(1UZ);

    struct Case {
        std::string                     label;
        std::size_t                     length;
        std::vector<gr::DataSet<float>> records;
        DataSetToStream<float>          block;
    };

    std::vector<Case> cases;
    for (const std::size_t length : kLengths) {
        for (const std::size_t signals : kSignalCounts) {
            for (const std::size_t events : kEventCounts) {
                const std::size_t               count = std::clamp(kTotalSamples / length, 1UZ, kMaxRecords);
                std::vector<gr::DataSet<float>> records(count, makeRecord(length, signals, events));
                cases.push_back({std::format("length {:>7}, {} signal(s), {:>2} event(s)", length, signals, events), length, std::move(records), makeBlock()});
            }
        }
    }

    std::vector<Arm> arms;
    arms.emplace_back("plain span copy, the floor", kTotalSamples, [&floor, &out] {
        std::copy_n(out.begin(), kTotalSamples, floor.begin());
        return static_cast<double>(floor[kTotalSamples / 2UZ]);
    });
    for (std::size_t a = 0UZ; a < cases.size(); ++a) {
        const std::size_t samples = cases[a].length * cases[a].records.size();
        arms.emplace_back(cases[a].label, samples, [&cases, &out, &rejectScratch, a] {
            Case&       item    = cases[a];
            std::size_t written = 0UZ;
            for (const gr::DataSet<float>& record : item.records) {
                const std::span<const gr::DataSet<float>> one(&record, 1UZ);
                InputSpan<gr::DataSet<float>>             inSpan(one);
                OutputSpan<float>                         outSpan(std::span<float>(out).subspan(written, item.length));
                OutputSpan<gr::DataSet<float>>            rejectSpan(std::span<gr::DataSet<float>>(rejectScratch), 0UZ, nullptr, false);
                std::ignore = item.block.processBulk(inSpan, outSpan, rejectSpan);
                written += outSpan.count;
            }
            return static_cast<double>(out[written / 2UZ]);
        });
    }

    gr::blocks::testing::bench::report(std::span<Arm>(arms), kRepeats);
}
