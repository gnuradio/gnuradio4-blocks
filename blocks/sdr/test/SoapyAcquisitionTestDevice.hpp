#ifndef GR_SDR_ACQUISITION_TEST_DEVICE_HPP
#define GR_SDR_ACQUISITION_TEST_DEVICE_HPP
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcpp"
#endif
#include <array>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Registry.hpp>
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <mutex>
#include <cstdint>
#include <stdexcept>
#include <thread>
#include <string>
#include <utility>
#include <vector>

namespace {
using CF32 = std::complex<float>;
// Derived selectively from the RF World fixture. Gates coordinate actual driver
// calls with the test thread; deadlines are only deadlock watchdogs.
class DriverGate {
    std::mutex _mutex;
    std::condition_variable _condition;
    bool _entered = false, _open = false;
public:
    void reset() { auto lock = std::lock_guard(_mutex); _entered = _open = false; }
    void enter() {
        auto lock = std::unique_lock(_mutex);
        _entered = true;
        _condition.notify_all();
        _condition.wait(lock, [&] { return _open; });
    }
    bool wait() {
        auto lock = std::unique_lock(_mutex);
        return _condition.wait_for(lock, std::chrono::seconds(3), [&] { return _entered; });
    }
    void release() { auto lock = std::lock_guard(_mutex); _open = true; _condition.notify_all(); }
};

// No physical modules are needed. Each fault is reproducible and read sizes are
// deliberately unrelated to the consumer's chunk size.
class TestRadio final : public SoapySDR::Device {
    SoapySDR::Kwargs _args;
    bool _active = false;
    std::array<double, 2> _bufferFrequency{}, _frequency{100e6, 100e6};
    double _rate = 1e6, _bandwidth = 500e3, _gain = 0;
    std::int64_t _clock = 0, _next = 0;
    unsigned _tunes = 0, _reads = 0, _stale = 0, _setups = 0, _closes = 0, _deactivations = 0, _activations = 0;
    char _stream{}, _txStream{};
    std::vector<size_t> _channels;
    bool has(const std::string& key) const { return _args.contains(key); }
    unsigned number(const std::string& key, unsigned fallback = 0) const { return has(key) ? static_cast<unsigned>(std::stoul(_args.at(key))) : fallback; }

public:
    inline static std::atomic<unsigned> closeCalls{0}, activateCalls{0};
    inline static std::atomic<unsigned> readCalls{0}, destroyed{0}, maximumReservation{0}, lastReservation{0};
    inline static DriverGate readGate, tuneGate, closeGate, activateGate;
    ~TestRadio() override { ++destroyed; }
    inline static std::atomic<bool> tuning{false}, reading{false}, deactivating{false}, closing{false};
    explicit TestRadio(SoapySDR::Kwargs args) : _args(std::move(args)) { _stale = number("initial_stale"); }
    std::string getDriverKey() const override { return "gr4_test_rx"; }
    std::string getHardwareKey() const override { return "gr4_test_radio"; }
    SoapySDR::Kwargs getHardwareInfo() const override { return has("no_identity") ? SoapySDR::Kwargs{} : SoapySDR::Kwargs{{"serial", has("serial") ? _args.at("serial") : "A"}}; }
    size_t getNumChannels(int) const override { return 2; }
    std::vector<std::string> getStreamFormats(int, size_t) const override { return {SOAPY_SDR_CF32}; }
    bool getFullDuplex(int, size_t) const override { return true; }
    void setSampleRate(int, size_t, double rate) override { _rate = has("coerce") ? rate - 1000.0 : rate; }
    double getSampleRate(int, size_t channel) const override { if (has("unavailable_rate")) { throw std::runtime_error("rate unavailable"); } return _rate + (has("unequal_rates") && channel == 1 ? 1.0 : 0.0); }
    void setBandwidth(int, size_t, double value) override { _bandwidth = value; }
    double getBandwidth(int, size_t) const override { if (has("unavailable_bw")) { throw std::runtime_error("bandwidth unavailable"); } return _bandwidth; }
    void setGain(int, size_t, double value) override { _gain = value; }
    double getGain(int, size_t) const override { return _gain; }
    std::vector<std::string> listGains(int, size_t) const override { return {"RF"}; }
    void setFrequency(int, size_t channel, double value, const SoapySDR::Kwargs&) override {
        if (_active) { throw std::runtime_error("tuning while active"); }
        if (has("fail_tune") || (has("fail_retune") && _tunes > 0)) { throw std::runtime_error("tuning failed"); }
        if (_tunes == 0 && has("gate_initial_tune")) { tuneGate.enter(); }
        if (_tunes > 0 && has("gate_tune")) { tuneGate.enter(); }
        if (_tunes > 0 && has("slow_tune")) {
            tuning.store(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            tuning.store(false);
        }
        _frequency.at(channel) = has("coerce") ? value + 123.0 : value;
        if (_tunes++ > 0) { _clock += 1'000'000; _stale = number("stale"); }
        _next = _clock;
    }
    void setFrequency(int, size_t channel, const std::string&, double value, const SoapySDR::Kwargs&) override { _frequency.at(channel) = value + 12500.0; }
    std::vector<std::string> listFrequencies(int, size_t) const override { return has("no_components") ? std::vector<std::string>{} : std::vector<std::string>{"RF", "BB"}; }
    double getFrequency(int, size_t channel) const override { if (has("unavailable_frequency")) { throw std::runtime_error("frequency unavailable"); } return _frequency.at(channel); }
    std::string getClockSource() const override { return "internal"; }
    std::string getTimeSource() const override { return "internal"; }
    double getMasterClockRate() const override { return 30.72e6; }
    double getReferenceClockRate() const override { return 10e6; }
    bool hasHardwareTime(const std::string&) const override { return !has("no_clock"); }
    long long getHardwareTime(const std::string&) const override { return _clock; }
    SoapySDR::Stream* setupStream(int direction, const std::string&, const std::vector<size_t>& channels, const SoapySDR::Kwargs&) override {
        if (direction == SOAPY_SDR_TX) { return reinterpret_cast<SoapySDR::Stream*>(&_txStream); }
        ++_setups;
        if (number("fail_setup_at") == _setups) { throw std::runtime_error("setup failed"); }
        _channels = channels;
        _bufferFrequency = _frequency;
        return reinterpret_cast<SoapySDR::Stream*>(&_stream);
    }
    void closeStream(SoapySDR::Stream* stream) override {
        if (stream == reinterpret_cast<SoapySDR::Stream*>(&_txStream)) { return; }
        ++_closes;
        ++closeCalls;
        if (has("gate_close")) { closeGate.enter(); }
        if (has("fail_close") || number("fail_close_at") == _closes) { throw std::runtime_error("close failed"); }
    }
    size_t getStreamMTU(SoapySDR::Stream*) const override { return _setups > 1 && has("changed_mtu") ? 5 : 16; }
    int activateStream(SoapySDR::Stream* stream, int, long long, size_t) override {
        if (stream == reinterpret_cast<SoapySDR::Stream*>(&_txStream)) { return 0; }
        ++_activations;
        ++activateCalls;
        if (has("gate_activate")) { activateGate.enter(); }
        if (has("fail_activate") || number("fail_activate_at") == _activations) { return SOAPY_SDR_STREAM_ERROR; }
        _active = true;
        return 0;
    }
    int deactivateStream(SoapySDR::Stream* stream, int, long long) override {
        if (stream == reinterpret_cast<SoapySDR::Stream*>(&_txStream)) { return 0; }
        ++_deactivations;
        _active = false;
        return has("fail_deactivate") || number("fail_deactivate_at") == _deactivations ? SOAPY_SDR_STREAM_ERROR : 0;
    }
    int readStream(SoapySDR::Stream*, void* const* buffers, size_t count, int& flags, long long& timeNs, long timeoutUs) override {
        ++_reads;
        ++readCalls;
        lastReservation.store(static_cast<unsigned>(count));
        maximumReservation.store(std::max(maximumReservation.load(), static_cast<unsigned>(count)));
        if (has("gate_read") && _reads == number("gate_read_at", 1)) { reading.store(true); readGate.enter(); reading.store(false); }
        if (!_active) { throw std::runtime_error("read while inactive"); }
        flags = has("no_time") ? 0 : SOAPY_SDR_HAS_TIME;
        timeNs = _next;
        if (has("timeout") || _reads <= number("initial_timeouts")) { std::this_thread::sleep_for(std::chrono::microseconds(timeoutUs)); return SOAPY_SDR_TIMEOUT; }
        if (has("slow_read")) {
            reading.store(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            reading.store(false);
        }
        if (number("fail_read") == _reads) { return SOAPY_SDR_STREAM_ERROR; }
        if (number("overflow") == _reads || _reads <= number("initial_overflows")) { return SOAPY_SDR_OVERFLOW; }
        if (_stale > 0) { --_stale; timeNs = _clock - 100000; }
        if (has("stale_forever")) { timeNs = _clock - 1; }
        if (number("jump") == _reads) { _next += 10000; timeNs = _next; }
        const auto taken = std::min(count, static_cast<size_t>(number("partial", 7)));
        for (size_t ch = 0; ch < _channels.size(); ++ch) {
            auto* output = static_cast<CF32*>(buffers[ch]);
            for (size_t i = 0; i < taken; ++i) { output[i] = CF32(static_cast<float>(has("restart_buffers") ? _bufferFrequency.at(_channels[ch]) : _frequency.at(_channels[ch])), static_cast<float>(_reads + _channels[ch] * 1000)); }
        }
        if (has("fragments")) { flags |= SOAPY_SDR_MORE_FRAGMENTS; }
        if (number("end_burst") == _reads) { flags |= SOAPY_SDR_END_BURST; }
        _next += static_cast<std::int64_t>(std::llround(static_cast<double>(taken) * 1e9 / _rate));
        return static_cast<int>(has("oversized_read") ? count + 1 : taken);
    }
};
SoapySDR::KwargsList discover(const SoapySDR::Kwargs& args) { return {{{"driver", "gr4_test_rx"}, {"serial", args.contains("serial") ? args.at("serial") : "A"}}}; }
SoapySDR::Device* makeRadio(const SoapySDR::Kwargs& args) { return new TestRadio(args); }
SoapySDR::Registry registration("gr4_test_rx", discover, makeRadio, SOAPY_SDR_ABI_VERSION);

} // namespace
#endif
