#include <boost/ut.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/sdr/SoapySource.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include "SoapyAcquisitionTestDevice.hpp"

using namespace boost::ut;
using CF32 = std::complex<float>;
using Source = gr::blocks::sdr::SoapySource<CF32, 1>;
using Sink = gr::blocks::testing::TagSink<CF32, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>;
using gr::blocks::sdr::ReceiveLease;

namespace {
template<typename T>
T value(const gr::property_map& map, std::string_view key) {
    auto it = map.find(key);
    expect(it != map.end()) << key;
    if (it == map.end()) { return {}; }
    const auto* p = it->second.get_if<T>();
    expect(p != nullptr) << key;
    return p ? *p : T{};
}
std::string text(const gr::property_map& map, std::string_view key) { return std::string(value<std::pmr::string>(map, key)); }
bool waitFor(const auto& predicate) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!predicate()) {
        if (std::chrono::steady_clock::now() >= deadline) { return false; }
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    return true;
}
// Always release a gate before a session is destroyed, even after a failed oracle.
struct OpenGate { DriverGate& gate; ~OpenGate() { gate.release(); } };

gr::property_map epochs(bool restart = false) {
    return {{"epoch_samples", std::uint64_t{31}}, {"configuration_id", std::uint64_t{1}},
        {"epoch_freshness", std::string(restart ? "stream_restart" : "timestamp")}};
}

template<std::size_t N = 1>
struct Session {
    gr::SchedulerWrapper<gr::scheduler::Simple<>> scheduler;
    gr::blocks::sdr::SoapySource<CF32, N>* source;
    std::array<Sink*, N> sinks;
    explicit Session(std::string parameters = {}, gr::property_map extra = {}, std::size_t bufferSize = 65536) {
        static unsigned serial = 0;
        if (!parameters.contains("serial=")) { parameters += std::format(",serial=acquisition-{}-{}", N, ++serial); }
        gr::Graph graph;
        gr::property_map config{{"device", std::string("gr4_test_rx")}, {"device_parameter", parameters},
            {"frequency", std::vector{100e6}}, {"sample_rate", 1e6f}, {"require_fresh_samples", true},
            {"finite_samples", std::uint64_t{31}}, {"max_chunk_size", std::uint32_t{16}},
            {"emit_timing_tags", false}, {"log_device_state", false}};
        if (extra.contains("epoch_samples")) { config.insert_or_assign("finite_samples", std::uint64_t{0}); }
        for (const auto& [k, v] : extra) { config.insert_or_assign(k, v); }
        source = &graph.emplaceBlock<gr::blocks::sdr::SoapySource<CF32, N>>(std::move(config));
        for (std::size_t i = 0; i < N; ++i) {
            sinks[i] = &graph.emplaceBlock<Sink>({{"log_tags", true}, {"log_samples", true}});
            if constexpr (N == 1) { expect(graph.connect<"out", "in">(*source, *sinks[i], {.minBufferSize = bufferSize}).has_value()); }
            else {
                if (i == 0) { expect(graph.connect<"out#0", "in">(*source, *sinks[i], {.minBufferSize = bufferSize}).has_value()); }
                else { expect(graph.connect<"out#1", "in">(*source, *sinks[i], {.minBufferSize = bufferSize}).has_value()); }
            }
        }
        scheduler.setGraph(std::move(graph));
    }
    gr::property_map status() { return value<gr::property_map>(source->settings().get(), "source_status"); }
    void stage(gr::property_map settings) { expect(source->settings().setStaged(settings).empty()); }
    bool paused(std::uint64_t id = 1) {
        return waitFor([&] {
            const auto s = status();
            return s.contains("epoch_completion") && value<std::uint64_t>(value<gr::property_map>(s, "epoch_completion"), "configuration_id") == id;
        });
    }
    void done() {
        expect(waitFor([&] { return scheduler.state() == gr::lifecycle::State::STOPPED || scheduler.state() == gr::lifecycle::State::ERROR; }));
        scheduler.stop();
    }
    void start() { expect(scheduler.start().has_value()); }
};

void excluded(const gr::property_map& status) {
    if constexpr (ReceiveLease::supported) {
        ReceiveLease contender;
        expect(!contender.acquire(text(status, "hardware_identity")).has_value());
    }
}
void released(const gr::property_map& status) {
    expect(value<bool>(status, "source_terminated"));
    expect(value<bool>(status, "ownership_released"));
    if constexpr (ReceiveLease::supported) {
        ReceiveLease contender;
        expect(contender.acquire(text(status, "hardware_identity")).has_value());
    }
}

template<std::size_t N>
void partialReads() {
    Session<N> session("partial=7,fragments=1", {{"channels", N == 1 ? std::vector<gr::Size_t>{1} : std::vector<gr::Size_t>{1, 0}}});
    for (auto* sink : session.sinks) { sink->in.max_samples = 3; }
    session.start();
    session.done();
    const auto status = session.status();
    released(status);
    expect(eq(text(status, "terminal_cause"), std::string("normal_completion")));
    expect(eq(value<std::uint64_t>(status, "published_samples"), std::uint64_t{31}));
    for (std::size_t ch = 0; ch < N; ++ch) {
        auto* sink = session.sinks[ch];
        expect(eq(sink->_samples.size(), 31UZ));
        std::uint64_t accepted = 0;
        for (const auto& tag : sink->_tags) {
            if (!tag.map.contains("soapy_read_result")) { continue; }
            expect(eq(tag.index, static_cast<std::size_t>(accepted)));
            expect(eq(value<std::uint64_t>(tag.map, "sample_index"), accepted));
            expect(value<bool>(tag.map, "samples_admitted"));
            expect(value<bool>(tag.map, "freshness_verified"));
            expect(eq(value<gr::Size_t>(tag.map, "channel"), static_cast<gr::Size_t>(1 - ch)));
            expect(eq(value<float>(tag.map, gr::tag::SAMPLE_RATE.key()), 1e6f));
            expect(eq(value<double>(tag.map, gr::tag::FREQUENCY.key()), 100e6));
            expect(value<std::uint64_t>(tag.map, "read_requested") <= 31 - accepted);
            const auto n = static_cast<std::uint64_t>(value<std::int32_t>(tag.map, "soapy_read_result"));
            expect(eq(n, std::min(std::uint64_t{7}, 31 - accepted)));
            if constexpr (N == 1) { if (accepted == 0) { expect(eq(value<std::int64_t>(tag.map, "device_time_ns"), std::int64_t{0})); } }
            accepted += n;
        }
        expect(eq(accepted, std::uint64_t{31}));
        expect(eq(session.source->output(ch).nTagsDropped, 0UZ));
        expect(eq(session.source->output(ch).nEoSTagsLost, 0UZ));
    }
}

template<std::size_t N>
void backpressure(unsigned mode) {
    DriverGate consumer;
    auto settings = mode == 2 ? epochs() : gr::property_map{{"finite_samples", mode == 0 ? std::uint64_t{0} : std::uint64_t{257}}};
    if (mode == 2) { settings.insert_or_assign("epoch_samples", std::uint64_t{257}); }
    settings.insert_or_assign("require_fresh_samples", mode != 0);
    settings.insert_or_assign("emit_timing_tags", true);
    settings.insert_or_assign("tag_interval", 0.f);
    settings.insert_or_assign("max_chunk_size", std::uint32_t{1});
    Session<N> session("partial=1", settings, 64);
    OpenGate open{consumer};
    std::atomic<bool> entered{false};
    session.sinks[0]->_tagCallback = [&](const gr::Tag&) { if (!entered.exchange(true)) { consumer.enter(); } };
    session.start();
    expect(consumer.wait());
    // The sink holds its first chunk. One merged tag per sample must fill the
    // usable tag store exactly, preserving both provenance and timing metadata.
    expect(waitFor([&] {
        const auto status = session.status();
        return status.contains("waiting_for") && text(status, "waiting_for") == "metadata";
    }));
    const auto blocked = session.status();
    expect(!blocked.contains("terminal_cause"));
    expect(eq(value<std::uint64_t>(blocked, "published_samples"), std::uint64_t{64}));
    if (mode == 0) { session.stage({{"cancel", true}}); }
    consumer.release();
    if (mode == 2) { expect(session.paused()); session.stage({{"finish", true}}); }
    session.done();
    const auto status = session.status();
    released(status);
    const auto count = value<std::uint64_t>(status, "published_samples");
    expect(eq(text(status, "terminal_cause"), std::string(mode == 0 ? "cancelled" : "normal_completion")));
    for (std::size_t ch = 0; ch < N; ++ch) {
        expect(eq(session.sinks[ch]->_samples.size(), static_cast<std::size_t>(count)));
        std::size_t reads = 0;
        for (const auto& tag : session.sinks[ch]->_tags) {
            if (tag.map.contains("soapy_read_result")) {
                expect(eq(tag.index, reads++));
                expect(tag.map.contains(gr::tag::TRIGGER_TIME.key()));
                expect(tag.map.contains(gr::tag::TRIGGER_META_INFO.key()));
            }
        }
        expect(eq(reads, static_cast<std::size_t>(count)));
        expect(eq(session.source->output(ch).nTagsDropped, 0UZ));
        expect(eq(session.source->output(ch).nEoSTagsLost, 0UZ));
    }
}
// Endpoints retain the rejected read's own evidence, distinct from the read
// admitting IQ. Intermediate errors are counted/coalesced without a growing log.
void discontinuityEvidence(const gr::property_map& summary, std::uint64_t count) {
    expect(eq(value<std::uint64_t>(summary, "rejected_read_count"), count));
    expect(eq(value<std::uint64_t>(summary, "overflow_read_count"), count));
    expect(eq(value<std::int32_t>(summary, "flags_or"), std::int32_t{SOAPY_SDR_HAS_TIME}));
    expect(!value<bool>(summary, "count_saturated"));
    expect(eq(value<bool>(summary, "details_coalesced"), count > 2));
    for (const auto* key : {"first_read", "last_read"}) {
        const auto raw = value<gr::property_map>(summary, key);
        expect(eq(value<std::int32_t>(raw, "soapy_read_result"), std::int32_t{SOAPY_SDR_OVERFLOW}));
        expect(eq(value<std::int32_t>(raw, "soapy_read_flags"), std::int32_t{SOAPY_SDR_HAS_TIME}));
        expect(eq(value<std::int64_t>(raw, "soapy_time_ns"), std::int64_t{0}));
        expect(eq(value<std::int64_t>(raw, "device_time_ns"), std::int64_t{0}));
        expect(value<bool>(raw, "has_device_time"));
        expect(value<bool>(raw, "discontinuity"));
        expect(!value<bool>(raw, "samples_admitted"));
        expect(eq(value<std::uint64_t>(raw, "sample_index"), std::uint64_t{0}));
        expect(eq(value<std::uint64_t>(raw, "read_requested"), std::uint64_t{1}));
        expect(eq(value<std::uint64_t>(raw, "read_ordinal"), std::string_view(key) == "first_read" ? std::uint64_t{0} : count - 1));
    }
}

template<std::size_t N>
void minimumInputChunk(unsigned mode, unsigned overflows = 0, bool timingEnabled = true, bool unlimitedOverflow = false) {
    constexpr std::uint64_t samples = 4096;
    auto settings = mode == 2 ? epochs() : gr::property_map{{"finite_samples", mode == 0 ? std::uint64_t{0} : samples}};
    if (mode == 2) { settings.insert_or_assign("epoch_samples", samples); }
    settings.insert_or_assign("require_fresh_samples", mode != 0);
    settings.insert_or_assign("emit_timing_tags", timingEnabled);
    settings.insert_or_assign("emit_meta_info", true);
    settings.insert_or_assign("tag_interval", 0.f);
    settings.insert_or_assign("max_chunk_size", std::uint32_t{1});
    if (unlimitedOverflow) { settings.insert_or_assign("max_overflow_count", gr::Size_t{0}); }
    // Keep the review reproducer's exact existing overflow=1 fixture as well as
    // a prefix longer than the default budget when unlimited recovery is asked.
    const auto fault = overflows == 0 ? "" : overflows == 1 ? ",overflow=1" : std::format(",initial_overflows={}", overflows);
    Session<N> session("partial=1" + std::string(fault), settings, samples);
    for (auto* sink : session.sinks) {
        sink->in.min_samples = samples;
        sink->in.max_samples = samples;
    }
    std::atomic<bool> firstChunk{false};
    if (mode == 0) {
        session.sinks[0]->_tagCallback = [&](const gr::Tag&) {
            if (!firstChunk.exchange(true)) { session.stage({{"cancel", true}}); }
        };
    }
    session.start();
    if (mode == 0) {
        // Success requires the minimum-sized consumer actually to run. A
        // watchdog stop (or done()'s cleanup fallback) cannot satisfy this.
        expect(waitFor([&] { return firstChunk.load(); }));
    }
    if (mode == 2) { expect(session.paused()); session.stage({{"finish", true}}); }
    session.done();
    expect(session.scheduler.state() == gr::lifecycle::State::STOPPED);
    const auto status = session.status();
    released(status);
    expect(eq(text(status, "terminal_cause"), std::string(mode == 0 ? "cancelled" : "normal_completion")));
    expect(eq(value<std::uint64_t>(status, "published_samples"), samples));
    expect(!status.contains("pending_discontinuity"));
    for (std::size_t ch = 0; ch < N; ++ch) {
        expect(eq(session.sinks[ch]->_samples.size(), static_cast<std::size_t>(samples)));
        std::uint64_t reads = 0;
        for (const auto& tag : session.sinks[ch]->_tags) {
            if (!tag.map.contains("soapy_read_result")) { continue; }
            expect(eq(tag.index, static_cast<std::size_t>(reads)));
            expect(eq(value<std::uint64_t>(tag.map, "read_ordinal"), reads + overflows));
            ++reads;
            if (tag.index == 0 && overflows != 0) {
                discontinuityEvidence(value<gr::property_map>(tag.map, "preceding_discontinuity"), overflows);
                expect(value<bool>(tag.map, "discontinuity"));
                expect(value<bool>(tag.map, gr::tag::RX_OVERFLOW.key()));
                expect(eq(value<std::int64_t>(tag.map, "soapy_time_ns"), std::int64_t{0}));
            } else { expect(!tag.map.contains("preceding_discontinuity")); }
            expect(eq(value<std::int32_t>(tag.map, "soapy_read_flags"), std::int32_t{SOAPY_SDR_HAS_TIME}));
            expect(eq(value<std::int32_t>(tag.map, "soapy_read_result"), std::int32_t{1}));
            expect(eq(value<std::uint64_t>(tag.map, "read_requested"), std::uint64_t{1}));
            expect(eq(value<gr::Size_t>(tag.map, "channel"), static_cast<gr::Size_t>(ch)));
            expect(eq(value<float>(tag.map, gr::tag::SAMPLE_RATE.key()), static_cast<float>(value<double>(tag.map, "actual_sample_rate_hz"))));
            expect(eq(value<double>(tag.map, gr::tag::FREQUENCY.key()), value<double>(tag.map, "actual_center_frequency_hz")));
            expect(value<bool>(tag.map, "samples_admitted"));
            expect(eq(value<bool>(tag.map, "freshness_verified"), mode != 0));
            if (!timingEnabled) {
                expect(!tag.map.contains(gr::tag::TRIGGER_TIME.key()));
                expect(!tag.map.contains(gr::tag::TRIGGER_META_INFO.key()));
                continue;
            }
            expect(eq(text(tag.map, gr::tag::TRIGGER_NAME.key()), std::string("SDR_WALLCLOCK")));
            expect(value<std::uint64_t>(tag.map, gr::tag::TRIGGER_TIME.key()) > 0);
            expect(eq(value<float>(tag.map, gr::tag::TRIGGER_OFFSET.key()), 0.f));
            const auto timing = value<gr::property_map>(tag.map, gr::tag::TRIGGER_META_INFO.key());
            expect(eq(text(timing, "trigger_source"), std::string("SoapySource")));
            expect(eq(text(timing, "clock_source"), std::string("wallclock")));
            if (tag.index == 0) {
                expect(eq(value<double>(timing, "requested_sample_rate_hz"), 1e6));
                expect(eq(value<double>(timing, "requested_center_frequency_hz"), 100e6));
            }
        }
        expect(eq(reads, samples));
        expect(eq(session.source->output(ch).nTagsDropped, 0UZ));
        expect(eq(session.source->output(ch).nEoSTagsLost, 0UZ));
    }
}

// Standalone Core lifecycle use reaches destruction while RUNNING/PAUSED;
// scheduler-owned blocks normally transition before their owners destroy them.
std::unique_ptr<Source> standalone(std::string parameters, gr::property_map settings, gr::PortIn<CF32>& receiver) {
    gr::property_map config{{"device", std::string("gr4_test_rx")}, {"device_parameter", std::move(parameters)},
        {"frequency", std::vector{100e6}}, {"sample_rate", 1e6f}, {"require_fresh_samples", true},
        {"log_device_state", false}, {"emit_timing_tags", false}, {"max_chunk_size", std::uint32_t{16}}};
    for (const auto& [key, value] : settings) { config.insert_or_assign(key, value); }
    auto source = std::make_unique<Source>(std::move(config));
    source->init(std::make_shared<gr::Sequence>());
    expect(source->out.connect(receiver).has_value());
    expect(source->changeStateTo(gr::lifecycle::State::RUNNING).has_value());
    return source;
}

gr::property_map terminalFrom(gr::PortIn<CF32>& receiver) {
    auto& reader = receiver.tagReader();
    auto tags = reader.get(reader.available());
    gr::property_map status;
    for (const auto& tag : tags) {
        if (tag.map.contains("source_terminal")) {
            expect(tag.map.contains(gr::tag::END_OF_STREAM.key()));
            status = value<gr::property_map>(tag.map, "source_terminal");
        }
    }
    expect(!status.empty());
    std::ignore = tags.consume(tags.size());
    return status;
}

}

const suite acquisition = [] {
    "finite reads preserve exact accepted partial counts and channel alignment"_test = [] { partialReads<1>(); partialReads<2>(); };
    "metadata backpressure coordinates delayed consumers in every mode and channel path"_test = [] {
        for (unsigned mode = 0; mode < 3; ++mode) { backpressure<1>(mode); backpressure<2>(mode); }
    };
    "minimum-size consumers progress with merged read and timing tags in every mode and channel path"_test = [] {
        for (unsigned mode = 0; mode < 3; ++mode) { minimumInputChunk<1>(mode); minimumInputChunk<2>(mode); }
    };
    "a recoverable overflow cannot block a minimum-size continuous consumer"_test = [] {
        for (bool timing : {false, true}) { minimumInputChunk<1>(0, 1, timing); minimumInputChunk<2>(0, 1, timing); }
        minimumInputChunk<1>(0, 32, false, true);
        minimumInputChunk<2>(0, 32, true, true);
    };
    "bounded acquisition rejects the first overflow even with the continuous overflow budget disabled"_test = [] {
        for (bool persistent : {false, true}) {
            auto settings = persistent ? epochs() : gr::property_map{};
            settings.insert_or_assign("max_overflow_count", gr::Size_t{0});
            Session<2> session("overflow=1,partial=1", settings);
            session.start(); session.done();
            const auto status = session.status();
            released(status);
            expect(session.scheduler.state() == gr::lifecycle::State::ERROR);
            expect(eq(text(status, "terminal_cause"), std::string("stream_discontinuity")));
            expect(eq(value<std::uint64_t>(status, "published_samples"), std::uint64_t{0}));
            expect(!status.contains("pending_discontinuity"));
            const auto raw = value<gr::property_map>(status, "last_read");
            expect(eq(value<std::int32_t>(raw, "soapy_read_result"), std::int32_t{SOAPY_SDR_OVERFLOW}));
            expect(!value<bool>(raw, "samples_admitted"));
            for (std::size_t output = 0; output < 2; ++output) {
                expect(session.sinks[output]->_samples.empty());
                expect(eq(session.source->output(output).nTagsDropped, 0UZ));
                expect(eq(session.source->output(output).nEoSTagsLost, 0UZ));
            }
        }
    };
    "unpublished discontinuities survive cancellation or overflow-budget exhaustion in terminal EOS"_test = [] {
        for (bool cancel : {false, true}) {
            TestRadio::readGate.reset();
            gr::PortIn<CF32> receiver;
            auto source = standalone(cancel ? "serial=pending-cancel,initial_overflows=32,gate_read=1,gate_read_at=33,partial=1" : "serial=pending-budget,initial_overflows=32,partial=1",
                {{"require_fresh_samples", false}, {"max_chunk_size", std::uint32_t{1}},
                 {"max_overflow_count", cancel ? gr::Size_t{0} : gr::Size_t{10}}}, receiver);
            OpenGate open{TestRadio::readGate};
            if (cancel) {
                expect(TestRadio::readGate.wait());
                expect(source->settings().setStaged({{"cancel", true}}).empty());
                TestRadio::readGate.release();
            }
            expect(waitFor([&] { return gr::atomic_ref(source->_ioThreadDone).load_acquire(); }));
            source->stop(); // join before inspecting the final tag store
            const auto status = value<gr::property_map>(source->settings().get(), "source_status");
            expect(eq(receiver.tagReader().available(), 1UZ)); // only terminal EOS, no zero-IQ event tags
            const auto eos = terminalFrom(receiver);
            for (const auto& receipt : {status, eos}) {
                released(receipt);
                expect(eq(text(receipt, "terminal_cause"), std::string(cancel ? "cancelled" : "stream_discontinuity")));
                expect(eq(value<std::uint64_t>(receipt, "published_samples"), std::uint64_t{0}));
                discontinuityEvidence(value<gr::property_map>(receipt, "pending_discontinuity"), cancel ? 32 : 10);
                const auto last = value<gr::property_map>(receipt, "last_read");
                expect(eq(value<std::int32_t>(last, "soapy_read_result"), cancel ? std::int32_t{1} : std::int32_t{SOAPY_SDR_OVERFLOW}));
                expect(!value<bool>(last, "samples_admitted"));
            }
            expect(eq(receiver.streamReader().available(), 0UZ));
            expect(eq(source->out.nTagsDropped, 0UZ));
            expect(eq(source->out.nEoSTagsLost, 0UZ));
        }
    };
    "timing requests and frequency estimates follow each selected output through a non-first-only retune"_test = [] {
        auto settings = epochs();
        settings.insert_or_assign("channels", std::vector<gr::Size_t>{1, 0});
        settings.insert_or_assign("frequency", std::vector{100e6, 200e6});
        settings.insert_or_assign("emit_timing_tags", true);
        settings.insert_or_assign("emit_meta_info", true);
        settings.insert_or_assign("tag_interval", 0.f);
        settings.insert_or_assign("ppm_estimator_cutoff", 1.f);
        settings.insert_or_assign("max_chunk_size", std::uint32_t{1});
        Session<2> session("coerce=1,partial=1", settings);
        session.start();
        expect(session.paused());
        session.stage({{"configuration_id", std::uint64_t{2}}, {"frequency", std::vector{100e6, 201e6}}});
        expect(session.paused(2));
        session.stage({{"finish", true}});
        session.done();
        released(session.status());
        expect(session.scheduler.state() == gr::lifecycle::State::STOPPED);
        expect(eq(text(session.status(), "terminal_cause"), std::string("normal_completion")));
        for (std::size_t output = 0; output < 2; ++output) {
            std::array<unsigned, 2> reads{}, estimates{};
            for (const auto& tag : session.sinks[output]->_tags) {
                if (!tag.map.contains("soapy_read_result")) { continue; }
                const auto id = value<std::uint64_t>(tag.map, "configuration_id");
                expect(id == 1 || id == 2);
                if (id != 1 && id != 2) { continue; }
                ++reads[id - 1];
                const double requested = output == 0 ? 100e6 : id == 1 ? 200e6 : 201e6;
                const auto meta = value<gr::property_map>(tag.map, gr::tag::TRIGGER_META_INFO.key());
                expect(eq(value<double>(meta, "requested_center_frequency_hz"), requested));
                expect(eq(value<gr::Size_t>(meta, "channel"), static_cast<gr::Size_t>(1 - output)));
                expect(eq(value<gr::Size_t>(tag.map, "channel"), static_cast<gr::Size_t>(1 - output)));
                expect(eq(value<double>(tag.map, "actual_center_frequency_hz"), requested + 123.0));
                expect(eq(value<double>(tag.map, gr::tag::FREQUENCY.key()), requested + 123.0));
                expect(eq(value<double>(meta, "requested_sample_rate_hz"), 1e6));
                expect(eq(value<double>(tag.map, "actual_sample_rate_hz"), 999'000.0));
                expect(eq(value<float>(tag.map, gr::tag::SAMPLE_RATE.key()), 999'000.f));
                if (tag.map.contains("estimated_center_frequency_hz")) {
                    ++estimates[id - 1];
                    const double scale = 1.0 + static_cast<double>(value<float>(tag.map, "ppm_error")) * 1e-6;
                    expect(std::abs(value<double>(tag.map, "estimated_center_frequency_hz") / requested - scale) < 1e-12);
                }
            }
            for (std::size_t epoch = 0; epoch < 2; ++epoch) {
                expect(eq(reads[epoch], 31U));
                expect(estimates[epoch] > 0U);
            }
            expect(eq(session.sinks[output]->_samples.size(), 62UZ));
            for (std::size_t i = 0; i < session.sinks[output]->_samples.size(); ++i) {
                const double requested = output == 0 ? 100e6 : i < 31 ? 200e6 : 201e6;
                expect(eq(session.sinks[output]->_samples[i].real(), static_cast<float>(requested + 123.0)));
            }
            expect(eq(session.source->output(output).nTagsDropped, 0UZ));
            expect(eq(session.source->output(output).nEoSTagsLost, 0UZ));
        }
    };
    "paused epochs retain ownership and immutable receipts across a failed later request"_test = [] {
        Session session("fail_retune=1", epochs());
        session.start();
        expect(session.paused());
        const auto completed = value<gr::property_map>(session.status(), "epoch_completion");
        expect(value<bool>(completed, "completed"));
        expect(value<bool>(completed, "quiescent"));
        expect(!value<bool>(completed, "ownership_released"));
        excluded(session.status());
        // Quiescence is an observable phase, with no outstanding driver read.
        expect(eq(text(session.status(), "epoch_state"), std::string("paused")));
        session.stage({{"configuration_id", std::uint64_t{2}}, {"frequency", std::vector{102e6}}});
        session.done();
        const auto failed = session.status();
        expect(value<gr::property_map>(failed, "epoch_completion") == completed);
        expect(eq(value<std::uint64_t>(failed, "configuration_id"), std::uint64_t{2}));
        expect(eq(value<std::uint64_t>(failed, "applied_configuration_id"), std::uint64_t{1}));
        expect(eq(text(failed, "terminal_cause"), std::string("configuration_failure")));
        released(failed);
    };
    "restart close failure during the second configuration keeps the radio excluded"_test = [] {
        Session session("fail_close_at=1", epochs(true));
        session.start();
        expect(session.paused());
        const auto first = value<gr::property_map>(session.status(), "epoch_completion");
        session.stage({{"configuration_id", std::uint64_t{2}}});
        session.done();
        auto status = session.status();
        expect(value<bool>(status, "source_terminated"));
        expect(!value<bool>(status, "ownership_released"));
        expect(value<gr::property_map>(status, "release_errors").contains("close_stream"));
        expect(value<gr::property_map>(status, "epoch_completion") == first);
        excluded(status);
        expect(!session.source->_rxStream.close().has_value());
        session.source->stop();
        expect(!value<bool>(session.status(), "ownership_released"));
        excluded(session.status());
        // No second configure can turn an empty wrapper close into release success.
        bool refused = false;
        try { session.source->start(); } catch (...) { refused = true; }
        expect(refused);
    };
    "intermediate deactivation failure remains quarantined after a later successful deactivate"_test = [] {
        TestRadio::readGate.reset();
        Session session("gate_read=1,fail_deactivate_at=1", {{"finite_samples", std::uint64_t{0}}});
        OpenGate open{TestRadio::readGate};
        session.start();
        expect(TestRadio::readGate.wait());
        session.stage({{"frequency", std::vector{102e6}}, {"configuration_id", std::uint64_t{2}}});
        TestRadio::readGate.release();
        session.done();
        const auto status = session.status();
        expect(eq(text(status, "terminal_cause"), std::string("hardware_release_failure")));
        expect(!value<bool>(status, "ownership_released"));
        expect(value<gr::property_map>(status, "release_errors").contains("deactivate"));
        excluded(status);
    };
    "final release errors preserve an earlier read failure and remain latched"_test = [] {
        Session session("fail_read=2,fail_close=1");
        session.start();
        session.done();
        const auto status = session.status();
        expect(eq(text(status, "terminal_cause"), std::string("driver_failure")));
        expect(value<gr::property_map>(status, "release_errors").contains("close_stream"));
        expect(value<bool>(status, "source_terminated"));
        expect(!value<bool>(status, "ownership_released"));
        excluded(status);
        session.source->stop();
        expect(session.status() == status);
    };
    "failed epoch pause never reports quiescent completion or released ownership"_test = [] {
        Session session("fail_deactivate=1", epochs());
        session.start(); session.done();
        const auto status = session.status();
        expect(!status.contains("epoch_completion"));
        expect(value<bool>(status, "target_reached"));
        expect(!value<bool>(status, "ownership_released"));
        expect(eq(text(status, "terminal_cause"), std::string("hardware_release_failure")));
        const auto diagnostics = value<gr::property_map>(status, "release_errors");
        expect(eq(value<gr::property_map>(diagnostics, "deactivate").size(), 2UZ));
        excluded(status);
    };
    "target reached is independent of final release success"_test = [] {
        Session session("fail_close=1");
        session.start(); session.done();
        const auto status = session.status();
        expect(eq(value<std::uint64_t>(status, "published_samples"), std::uint64_t{31}));
        expect(eq(text(status, "terminal_cause"), std::string("hardware_release_failure")));
        expect(!value<bool>(status, "ownership_released"));
        excluded(status);
    };
    "a shared active TX retains its device reference after RX finishes, including failed RX close"_test = [] {
        namespace soapy = gr::blocks::sdr::soapy;
        for (bool failClose : {false, true}) {
            const auto serial = std::string(failClose ? "shared-tx-failed" : "shared-tx");
            soapy::Kwargs args{{"driver", "gr4_test_rx"}, {"serial", serial}};
            if (failClose) { args.insert_or_assign("fail_close", "1"); }
            const auto destroyed = TestRadio::destroyed.load();
            auto txDevice = soapy::Device::make(args);
            expect(txDevice.has_value());
            if (!txDevice) { continue; }
            auto tx = txDevice->setupStream<CF32, SOAPY_SDR_TX>();
            expect(tx.has_value());
            if (!tx) { continue; }
            expect(tx->activate().has_value());
            Session session("serial=" + serial + (failClose ? ",fail_close=1" : ""));
            session.start(); session.done();
            const auto status = session.status();
            if (failClose) {
                expect(!value<bool>(status, "ownership_released"));
                excluded(status);
            } else { released(status); }
            expect(value<bool>(status, "device_reference_released"));
            expect(!value<bool>(status, "device_handle_released"));
            expect(eq(TestRadio::destroyed.load(), destroyed));
            expect(txDevice->get() != nullptr);
            expect(tx->deactivate().has_value());
            expect(tx->close().has_value());
            auto final = txDevice->close();
            expect(final.has_value() && *final);
            expect(eq(TestRadio::destroyed.load(), destroyed + 1));
            if (failClose) { excluded(status); } // later TX destruction does not revoke quarantine
        }
    };
    "continuous timeouts remain unlimited and long polling intervals remain valid"_test = [] {
        Session session("initial_timeouts=5", {{"finite_samples", std::uint64_t{0}}, {"require_fresh_samples", false},
            {"max_timeout_count", std::uint32_t{2}}, {"max_time_out_us", std::uint32_t{120000}}});
        session.start();
        expect(waitFor([&] { return value<std::uint64_t>(session.status(), "published_samples") > 0; }));
        session.stage({{"cancel", true}});
        session.done();
        expect(eq(text(session.status(), "terminal_cause"), std::string("cancelled")));
        released(session.status());
    };
    "continuous retuning publishes canonical driver readbacks at the new sample boundary"_test = [] {
        TestRadio::readGate.reset();
        Session session("gate_read=1,coerce=1,no_time=1,no_clock=1,no_components=1", {{"finite_samples", std::uint64_t{0}}, {"require_fresh_samples", false}});
        OpenGate open{TestRadio::readGate};
        session.start();
        expect(TestRadio::readGate.wait());
        session.stage({{"configuration_id", std::uint64_t{17}}, {"sample_rate", 2e6f}, {"frequency", std::vector{102e6}}});
        TestRadio::readGate.release();
        expect(waitFor([&] { const auto s = session.status(); return s.contains("applied_configuration_id") && value<std::uint64_t>(s, "applied_configuration_id") == 17 && value<std::uint64_t>(s, "epoch_samples") > 0; }));
        session.stage({{"cancel", true}}); session.done();
        bool old = false, updated = false;
        for (const auto& tag : session.sinks[0]->_tags) {
            if (!tag.map.contains("soapy_read_result")) { continue; }
            const bool next = value<std::uint64_t>(tag.map, "configuration_id") == 17;
            old |= !next; updated |= next;
            expect(eq(value<float>(tag.map, gr::tag::SAMPLE_RATE.key()), next ? 1'999'000.f : 999'000.f));
            expect(eq(value<double>(tag.map, gr::tag::FREQUENCY.key()), next ? 102e6 + 123 : 100e6 + 123));
            expect(eq(value<double>(tag.map, "actual_sample_rate_hz"), next ? 1'999'000. : 999'000.));
        }
        expect(old && updated);
    };
    "unavailable readback is explicit and never fabricates canonical metadata"_test = [] {
        Session session("unavailable_rate=1,unavailable_frequency=1", {{"require_fresh_samples", false}});
        session.start(); session.done();
        for (const auto& tag : session.sinks[0]->_tags) {
            if (!tag.map.contains("soapy_read_result")) { continue; }
            expect(!tag.map.contains(gr::tag::SAMPLE_RATE.key()));
            expect(!tag.map.contains(gr::tag::FREQUENCY.key()));
            expect(!value<bool>(tag.map, "actual_sample_rate_hz_available"));
            expect(!value<bool>(tag.map, "actual_center_frequency_hz_available"));
        }
    };
    "stream recreation refreshes MTU and keeps stream generation independent of acquisition epochs"_test = [] {
        Session session("no_clock=1,no_time=1,restart_buffers=1,changed_mtu=1,partial=16", epochs(true));
        session.start(); expect(session.paused());
        session.stage({{"configuration_id", std::uint64_t{2}}, {"frequency", std::vector{102e6}}});
        expect(session.paused(2));
        session.stage({{"finish", true}}); session.done();
        bool second = false;
        for (const auto& tag : session.sinks[0]->_tags) {
            if (!tag.map.contains("soapy_read_result")) { continue; }
            expect(!value<bool>(tag.map, "freshness_verified"));
            expect(value<bool>(tag.map, "stream_boundary_verified"));
            if (value<std::uint64_t>(tag.map, "configuration_id") == 2) {
                if (!second) { expect(eq(value<std::uint64_t>(tag.map, "read_requested"), std::uint64_t{15})); }
                second = true;
                expect(eq(value<std::uint64_t>(tag.map, "stream_generation"), std::uint64_t{2}));
            }
        }
        expect(second);
        expect(eq(session.sinks[0]->_samples.size(), 62UZ));
        for (std::size_t i = 0; i < session.sinks[0]->_samples.size(); ++i) { expect(eq(session.sinks[0]->_samples[i].real(), i < 31 ? 100e6f : 102e6f)); }
    };
    "same source restarts with distinct run, configuration and stream identities"_test = [] {
        Session session("no_clock=1,no_time=1", epochs(true));
        session.start(); expect(session.paused());
        const auto first = session.status();
        session.stage({{"configuration_id", std::uint64_t{2}}}); expect(session.paused(2));
        session.stage({{"finish", true}}); session.done();
        // SchedulerWrapper restarts the same graph/block objects after joining.

        session.stage({{"configuration_id", std::uint64_t{1}}, {"finish", false}, {"epoch_freshness", std::string("timestamp")},
            {"epoch_samples", std::uint64_t{17}}});
        // A timestamp policy cannot be silently substituted on this driver.
        session.start(); session.done();
        const auto rejected = session.status();
        expect(session.scheduler.state() == gr::lifecycle::State::ERROR);
        expect(eq(text(rejected, "terminal_cause"), std::string("configuration_failure")));
        expect(text(rejected, "terminal_detail").contains("device has no readable hardware clock"));
        expect(!session.source->finish.value);
        expect(eq(session.source->epoch_samples.value, std::uint64_t{17}));
        expect(eq(session.source->epoch_freshness.value, std::string("timestamp")));

        session.stage({{"epoch_freshness", std::string("stream_restart")}, {"channels", std::vector<gr::Size_t>{1}},
            {"stream_args", std::string("test_restart=1")}});
        session.start(); expect(session.paused());
        const auto next = session.status();
        expect(eq(value<std::uint64_t>(value<gr::property_map>(next, "epoch_completion"), "accepted_samples"), std::uint64_t{17}));
        expect(eq(value<gr::Size_t>(value<gr::property_map>(value<gr::property_map>(next, "readback"), "0"), "channel"), gr::Size_t{1}));
        expect(value<std::uint64_t>(next, "run_id") > value<std::uint64_t>(first, "run_id"));
        expect(value<std::uint64_t>(next, "sdr_epoch") > value<std::uint64_t>(first, "sdr_epoch"));
        expect(value<std::uint64_t>(next, "stream_generation") != value<std::uint64_t>(next, "sdr_epoch"));
        session.stage({{"finish", true}}); session.done();
        expect(session.scheduler.state() == gr::lifecycle::State::STOPPED);
        expect(eq(session.sinks[0]->_samples.size(), 17UZ));
        released(session.status());
        for (const auto& tag : session.sinks[0]->_tags) {
            if (tag.map.contains("soapy_read_result")) { expect(eq(value<gr::Size_t>(tag.map, "channel"), gr::Size_t{1})); }
        }
    };
    "cancellation during a read admits no returned samples after the cancellation boundary"_test = [] {
        TestRadio::readGate.reset();
        Session session("gate_read=1", epochs());
        OpenGate open{TestRadio::readGate};
        session.start(); expect(TestRadio::readGate.wait());
        session.stage({{"cancel", true}});
        TestRadio::readGate.release(); session.done();
        expect(eq(value<std::uint64_t>(session.status(), "published_samples"), std::uint64_t{0}));
        expect(eq(text(session.status(), "terminal_cause"), std::string("cancelled")));
        released(session.status());
    };
    "cancellation remains identifiable when final release fails"_test = [] {
        TestRadio::readGate.reset();
        Session session("gate_read=1,fail_close=1", epochs());
        OpenGate open{TestRadio::readGate};
        session.start(); expect(TestRadio::readGate.wait());
        session.stage({{"cancel", true}});
        TestRadio::readGate.release(); session.done();
        const auto status = session.status();
        expect(eq(text(status, "acquisition_cause"), std::string("cancelled")));
        expect(eq(text(status, "terminal_cause"), std::string("hardware_release_failure")));
        expect(!value<bool>(status, "ownership_released"));
        excluded(status);
    };
    "cancellation during configuration and pause uses the normal cleanup path"_test = [] {
        for (bool duringConfiguration : {false, true}) {
            TestRadio::tuneGate.reset();
            Session session(duringConfiguration ? "gate_tune=1" : "", epochs());
            OpenGate open{TestRadio::tuneGate};
            session.start(); expect(session.paused());
            if (duringConfiguration) {
                session.stage({{"configuration_id", std::uint64_t{2}}, {"frequency", std::vector{102e6}}});
                expect(TestRadio::tuneGate.wait());
            }
            session.stage({{"cancel", true}});
            TestRadio::tuneGate.release(); session.done();
            expect(eq(value<std::uint64_t>(session.status(), "published_samples"), std::uint64_t{31}));
            expect(eq(text(session.status(), "terminal_cause"), std::string("cancelled")));
            released(session.status());
        }
    };
    "premature configuration is rejected without mixed epoch samples"_test = [] {
        TestRadio::readGate.reset();
        Session session("gate_read=1", epochs());
        OpenGate open{TestRadio::readGate};
        session.start(); expect(TestRadio::readGate.wait());
        session.stage({{"configuration_id", std::uint64_t{2}}, {"frequency", std::vector{102e6}}});
        TestRadio::readGate.release(); session.done();
        expect(eq(text(session.status(), "terminal_cause"), std::string("configuration_failure")));
        expect(eq(value<std::uint64_t>(session.status(), "applied_configuration_id"), std::uint64_t{1}));
        for (const auto& tag : session.sinks[0]->_tags) {
            if (tag.map.contains("configuration_id")) { expect(eq(value<std::uint64_t>(tag.map, "configuration_id"), std::uint64_t{1})); }
        }
    };
    "cancellation or external stop during initial configuration waits for the configuration owner"_test = [] {
        for (bool external : {false, true}) {
            TestRadio::tuneGate.reset();
            TestRadio::closeCalls.store(0);
            Session session("gate_initial_tune=1");
            OpenGate open{TestRadio::tuneGate};
            session.start(); expect(TestRadio::tuneGate.wait());
            if (external) {
                std::jthread stopping([&] { session.source->stop(); });
                expect(waitFor([&] { return session.source->_ioStopRequested.load(); }));
                expect(eq(TestRadio::closeCalls.load(), 0U));
                TestRadio::tuneGate.release();
                stopping.join();
                session.done();
            } else {
                session.stage({{"cancel", true}});
                TestRadio::tuneGate.release(); session.done();
                expect(eq(text(session.status(), "terminal_cause"), std::string("cancelled")));
            }
            expect(eq(value<std::uint64_t>(session.status(), "published_samples"), std::uint64_t{0}));
            released(session.status());
        }
    };
    "active standalone destruction changes lifecycle before joining an outstanding read"_test = [] {
        TestRadio::readGate.reset(); TestRadio::closeGate.reset();
        TestRadio::closeCalls.store(0);
        gr::PortIn<CF32> receiver;
        auto source = standalone("serial=destroy-reading,gate_read=1,gate_close=1", epochs(), receiver);
        OpenGate openRead{TestRadio::readGate}, openClose{TestRadio::closeGate};
        expect(TestRadio::readGate.wait());
        auto* live = source.get(); // gates keep the derived members and Core base alive below
        const auto before = value<gr::property_map>(live->settings().get(), "source_status");
        std::jthread destroying([&] { source.reset(); });
        expect(waitFor([&] { return live->_ioStopRequested.load(); }));
        expect(live->state() == gr::lifecycle::State::REQUESTED_STOP);
        expect(TestRadio::reading.load());
        expect(eq(TestRadio::closeCalls.load(), 0U));
        excluded(before);
        TestRadio::readGate.release();
        expect(TestRadio::closeGate.wait());
        expect(live->state() == gr::lifecycle::State::REQUESTED_STOP);
        expect(!TestRadio::reading.load());
        excluded(before);
        TestRadio::closeGate.release(); destroying.join();
        const auto status = terminalFrom(receiver);
        released(status);
        expect(eq(value<std::uint64_t>(status, "published_samples"), std::uint64_t{0}));
        expect(eq(receiver.streamReader().available(), 0UZ));
        expect(eq(TestRadio::closeCalls.load(), 1U));
    };
    "internally paused standalone destruction transitions Core before derived cleanup"_test = [] {
        for (bool corePaused : {false, true}) {
            TestRadio::closeGate.reset(); TestRadio::closeCalls.store(0);
            gr::PortIn<CF32> receiver;
            auto source = standalone(corePaused ? "serial=destroy-core-paused,gate_close=1" : "serial=destroy-epoch-paused,gate_close=1", epochs(), receiver);
            OpenGate open{TestRadio::closeGate};
            expect(fatal(waitFor([&] { return value<gr::property_map>(source->settings().get(), "source_status").contains("epoch_completion"); })));
            const auto before = value<gr::property_map>(source->settings().get(), "source_status");
            const auto receipt = value<gr::property_map>(before, "epoch_completion");
            expect(value<bool>(receipt, "quiescent"));
            excluded(before);
            if (corePaused) {
                expect(source->changeStateTo(gr::lifecycle::State::REQUESTED_PAUSE).has_value());
                expect(source->changeStateTo(gr::lifecycle::State::PAUSED).has_value());
            }
            expect(source->state() == (corePaused ? gr::lifecycle::State::PAUSED : gr::lifecycle::State::RUNNING));
            auto* live = source.get();
            std::jthread destroying([&] { source.reset(); });
            expect(TestRadio::closeGate.wait());
            expect(live->state() == gr::lifecycle::State::REQUESTED_STOP);
            excluded(before);
            TestRadio::closeGate.release(); destroying.join();
            const auto status = terminalFrom(receiver);
            released(status);
            expect(value<gr::property_map>(status, "epoch_completion") == receipt);
            expect(eq(value<std::uint64_t>(status, "published_samples"), std::uint64_t{31}));
            expect(eq(receiver.streamReader().available(), 31UZ));
            expect(eq(TestRadio::closeCalls.load(), 1U));
        }
    };
    "native cancellation completes before a pending peer settles, with independent release outcome"_test = [] {
        namespace soapy = gr::blocks::sdr::soapy;
        for (bool failClose : {false, true}) {
            const auto serial = std::string(failClose ? "cancel-pending-failed-close" : "cancel-pending");
            soapy::Kwargs args{{"driver", "gr4_test_rx"}, {"serial", serial}};
            if (failClose) { args.insert_or_assign("fail_close", "1"); }
            soapy::detail::DeviceRegistry::Registration peer(args);
            Session session("serial=" + serial + (failClose ? ",fail_close=1" : ""), epochs());
            TestRadio::readCalls.store(0); TestRadio::activateCalls.store(0);
            session.start();
            expect(fatal(waitFor([&] { return session.scheduler.workerStarted(); })));
            excluded(session.status());
            session.stage({{"cancel", true}});
            expect(fatal(waitFor([&] { return value<bool>(session.status(), "source_terminated"); })));
            session.done(); // peer is still deliberately pending; no stop fallback supplied completion
            const auto status = session.status();
            expect(eq(text(status, "acquisition_cause"), std::string("cancelled")));
            expect(eq(text(status, "terminal_cause"), std::string(failClose ? "hardware_release_failure" : "cancelled")));
            expect(session.scheduler.state() == (failClose ? gr::lifecycle::State::ERROR : gr::lifecycle::State::STOPPED));
            expect(value<bool>(status, "source_terminated"));
            expect(eq(value<bool>(status, "ownership_released"), !failClose));
            expect(!session.source->_ioThreadStarted.load());
            expect(eq(value<std::uint64_t>(status, "published_samples"), std::uint64_t{0}));
            expect(eq(session.sinks[0]->_samples.size(), 0UZ));
            expect(eq(TestRadio::readCalls.load(), 0U));
            expect(eq(TestRadio::activateCalls.load(), 0U));
            expect(eq(session.source->out.nEoSTagsLost, 0UZ));
            if (failClose) { excluded(status); } else { released(status); }
            peer.activate([] {}); // cancelled callback must never resurrect the source
            expect(eq(TestRadio::readCalls.load(), 0U));
            expect(eq(TestRadio::activateCalls.load(), 0U));
            if (!failClose) {
                // Clear cancellation and change mode/target through stopped native
                // staging, without manually applying it on the caller thread.
                session.stage({{"cancel", false}, {"epoch_samples", std::uint64_t{0}}, {"finite_samples", std::uint64_t{13}},
                    {"require_fresh_samples", false}, {"channels", std::vector<gr::Size_t>{1}}});
                session.start(); session.done();
                expect(session.scheduler.state() == gr::lifecycle::State::STOPPED);
                expect(!session.source->cancel.value);
                expect(!session.status().contains("epoch_completion"));
                expect(eq(value<std::uint64_t>(session.status(), "published_samples"), std::uint64_t{13}));
                expect(eq(session.sinks[0]->_samples.size(), 13UZ));
                expect(eq(text(session.status(), "terminal_cause"), std::string("normal_completion")));
                released(session.status());
            }
        }
    };
    "a ready callback extracted by the activation barrier is cancelled before source destruction"_test = [] {
        namespace soapy = gr::blocks::sdr::soapy;
        const soapy::Kwargs args{{"driver", "gr4_test_rx"}, {"serial", "activation-extracted"}};
        soapy::detail::DeviceRegistry::Registration blocker(args); // earlier callback in the ready batch
        DriverGate gate;
        auto session = std::make_unique<Session<>>("serial=activation-extracted", epochs());
        OpenGate open{gate};
        TestRadio::readCalls.store(0);
        session->start();
        expect(fatal(waitFor([&] { return session->scheduler.workerStarted(); })));
        std::jthread ready([&] { blocker.activate([&] { gate.enter(); }); });
        expect(gate.wait()); // source callback is already extracted but has not run
        session->source->stop(); session->done();
        released(session->status());
        session.reset();
        gate.release(); ready.join();
        expect(eq(TestRadio::readCalls.load(), 0U));
    };
    "deferred activation cannot outlive registration cancellation or destroy resources during activation"_test = [] {
        namespace soapy = gr::blocks::sdr::soapy;
        TestRadio::activateGate.reset();
        TestRadio::closeCalls.store(0);
        TestRadio::readCalls.store(0); TestRadio::activateCalls.store(0);
        const soapy::Kwargs args{{"driver", "gr4_test_rx"}, {"serial", "activation-gate"}, {"gate_activate", "1"}};
        soapy::detail::DeviceRegistry::Registration blocker(args);
        Session session("serial=activation-gate,gate_activate=1", epochs());
        OpenGate open{TestRadio::activateGate};
        session.start();
        expect(fatal(waitFor([&] { return session.scheduler.workerStarted(); })));
        std::jthread activating([&] { blocker.activate([] {}); });
        expect(TestRadio::activateGate.wait());
        const auto activatingStatus = session.status();
        session.stage({{"cancel", true}});
        expect(waitFor([&] { return session.source->_ioStopRequested.load(); }));
        expect(eq(TestRadio::closeCalls.load(), 0U));
        expect(session.status() == activatingStatus);
        TestRadio::activateGate.release();
        activating.join(); session.done();
        expect(eq(text(session.status(), "acquisition_cause"), std::string("cancelled")));
        released(session.status());
        expect(eq(value<std::uint64_t>(session.status(), "published_samples"), std::uint64_t{0}));
        expect(eq(TestRadio::readCalls.load(), 0U));
        expect(eq(TestRadio::activateCalls.load(), 1U));
    };
    "external stop joins the I/O owner before closing or releasing ownership"_test = [] {
        TestRadio::readGate.reset(); TestRadio::closeGate.reset();
        TestRadio::closeCalls.store(0);
        Session session("gate_read=1,gate_close=1", epochs());
        OpenGate openRead{TestRadio::readGate}, openClose{TestRadio::closeGate};
        session.start(); expect(TestRadio::readGate.wait());
        std::jthread stopping([&] { session.scheduler.stop(); });
        expect(waitFor([&] { return session.source->_ioStopRequested.load(); }));
        expect(TestRadio::reading.load());
        expect(eq(TestRadio::closeCalls.load(), 0U));
        excluded(session.status());
        TestRadio::readGate.release();
        expect(TestRadio::closeGate.wait());
        expect(!TestRadio::reading.load());
        // The driver has not yet acknowledged close, so no release is reported.
        expect(!value<bool>(session.status(), "ownership_released"));
        excluded(session.status());
        TestRadio::closeGate.release();
        stopping.join();
        released(session.status());
    };
    "startup and second-epoch setup or activation faults release through cleanup"_test = [] {
        for (const auto* fault : {"fail_tune=1", "fail_activate=1", "fail_setup_at=1"}) {
            Session session(fault);
            session.start(); session.done();
            expect(eq(text(session.status(), "terminal_cause"), std::string("configuration_failure")));
            released(session.status());
        }
        for (const auto* fault : {"fail_setup_at=2", "fail_activate_at=2"}) {
            Session session(fault, epochs(true));
            session.start(); expect(session.paused());
            const auto first = value<gr::property_map>(session.status(), "epoch_completion");
            session.stage({{"configuration_id", std::uint64_t{2}}}); session.done();
            expect(value<gr::property_map>(session.status(), "epoch_completion") == first);
            expect(eq(text(session.status(), "terminal_cause"), std::string("configuration_failure")));
            released(session.status());
        }
    };
    "source release precedes downstream processing and graph drain completion"_test = [] {
        DriverGate consumer;
        Session session;
        OpenGate open{consumer};
        std::atomic<bool> entered{false};
        session.sinks[0]->_tagCallback = [&](const gr::Tag&) { if (!entered.exchange(true)) { consumer.enter(); } };
        session.start(); expect(consumer.wait());
        expect(waitFor([&] { return value<bool>(session.status(), "source_terminated"); }));
        released(session.status());
        expect(session.scheduler.state() == gr::lifecycle::State::RUNNING);
        consumer.release(); session.done();
        expect(eq(session.sinks[0]->_samples.size(), 31UZ));
    };
    "bounded failures distinguish missing evidence, timeout, discontinuity and premature END_BURST"_test = [] {
        for (const auto& [fault, cause] : {std::pair{"no_time=1", "freshness_failure"}, {"timeout=1", "timeout"},
                {"overflow=2", "stream_discontinuity"}, {"jump=2", "stream_discontinuity"}, {"initial_stale=10", "freshness_failure"}, {"end_burst=1", "premature_end_burst"}}) {
            Session session(fault, {{"max_timeout_count", std::uint32_t{2}}, {"max_stale_reads", std::uint32_t{2}}});
            session.start(); session.done();
            expect(eq(text(session.status(), "terminal_cause"), std::string(cause)));
            expect(value<std::uint64_t>(session.status(), "published_samples") < 31);
            released(session.status());
        }
    };
    "identity and backend availability are optional unless explicitly required"_test = [] {
        Session session("no_identity=1");
        session.start(); session.done();
        expect(eq(text(session.status(), "ownership"), std::string("unavailable")));
        expect(text(session.status(), "hardware_identity").empty());
        expect(text(session.status(), "source_identity").starts_with("instance:"));
        expect(value<bool>(session.status(), "ownership_released"));
        Session required("no_identity=1", {{"require_receive_ownership", true}});
        required.start(); required.done();
        expect(eq(text(required.status(), "terminal_cause"), std::string("configuration_failure")));
        if constexpr (!ReceiveLease::supported) {
            Session optional;
            optional.start(); optional.done();
            expect(eq(text(optional.status(), "terminal_cause"), std::string("normal_completion")));
            Session unavailable({}, {{"require_receive_ownership", true}});
            unavailable.start(); unavailable.done();
            expect(eq(text(unavailable.status(), "terminal_cause"), std::string("configuration_failure")));
        }
    };
};
int main() {}
