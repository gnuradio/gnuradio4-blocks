#include <boost/ut.hpp>

#include <cstddef>
#include <cstdio>
#include <format>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/basic/MessageDebug.hpp>

// not in the anonymous namespace: clang refuses the reflection helper of a block declared there (-Wunneeded-internal-declaration)
namespace qa_message_debug {

/// sends `n_messages` messages on `out`, one per work call, each carrying its index under "index"
struct IndexedMessageSource : gr::Block<IndexedMessageSource> {
    gr::MsgPortOut out;
    gr::Size_t     n_messages = 3U;

    GR_MAKE_REFLECTABLE(IndexedMessageSource, out, n_messages);

    gr::Size_t _sent = 0U;

    gr::work::Status processBulk() {
        if (_sent < n_messages) {
            gr::sendMessage<gr::message::Command::Notify>(out, "", "count", gr::property_map{{"index", _sent}});
            ++_sent;
        }
        return _sent >= n_messages ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

/// sends messages 0 to 2 on `out`, then a settings Set for the block named "debug" on `control`, then message 3.
/// Each step takes one work call.
struct SettingsChangingSource : gr::Block<SettingsChangingSource> {
    gr::MsgPortOut out;

    GR_MAKE_REFLECTABLE(SettingsChangingSource, out);

    gr::MsgPortOut*  control = nullptr;
    gr::property_map change;
    gr::Size_t       _step = 0U;

    gr::work::Status processBulk() {
        if (_step < 3U) {
            gr::sendMessage<gr::message::Command::Notify>(out, "", "count", gr::property_map{{"index", _step}});
        } else if (_step == 3U) {
            gr::sendMessage<gr::message::Command::Set>(*control, "debug", gr::block::property::kSetting, change);
        } else {
            gr::sendMessage<gr::message::Command::Notify>(out, "", "count", gr::property_map{{"index", _step - 1U}});
        }
        ++_step;
        return _step > 4U ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

} // namespace qa_message_debug

namespace {

using gr::blocks::basic::MessageDebug;
using qa_message_debug::IndexedMessageSource;
using qa_message_debug::SettingsChangingSource;

/// the descriptor calls StdoutCapture needs; the CRT spells them with a leading underscore and deprecates the POSIX names
namespace stdout_fd {
#ifdef _WIN32
inline constexpr int kStdout = 1;
inline int           duplicate(int fd) { return ::_dup(fd); }
inline int           duplicateTo(int fd, int target) { return ::_dup2(fd, target); }
inline int           closeFd(int fd) { return ::_close(fd); }
inline int           descriptorOf(std::FILE* file) { return ::_fileno(file); }
#else
inline constexpr int kStdout = STDOUT_FILENO;
inline int           duplicate(int fd) { return ::dup(fd); }
inline int           duplicateTo(int fd, int target) { return ::dup2(fd, target); }
inline int           closeFd(int fd) { return ::close(fd); }
inline int           descriptorOf(std::FILE* file) { return ::fileno(file); }
#endif
} // namespace stdout_fd

/// redirects standard output to a temporary file while it lives, and returns what was written
class StdoutCapture {
    std::FILE* _file  = std::tmpfile();
    int        _saved = -1;

public:
    StdoutCapture() {
        std::fflush(stdout);
        _saved = stdout_fd::duplicate(stdout_fd::kStdout);
        stdout_fd::duplicateTo(stdout_fd::descriptorOf(_file), stdout_fd::kStdout);
    }
    StdoutCapture(const StdoutCapture&)            = delete;
    StdoutCapture& operator=(const StdoutCapture&) = delete;
    ~StdoutCapture() {
        restore();
        std::fclose(_file);
    }

    std::string release() {
        restore();
        std::string text;
        std::rewind(_file);
        for (int c = std::fgetc(_file); c != EOF; c = std::fgetc(_file)) {
            text.push_back(static_cast<char>(c));
        }
        return text;
    }

private:
    void restore() {
        if (_saved >= 0) {
            std::fflush(stdout);
            stdout_fd::duplicateTo(_saved, stdout_fd::kStdout);
            stdout_fd::closeFd(_saved);
            _saved = -1;
        }
    }
};

/// a block driven without a scheduler: one port feeds `messages`, one feeds the built-in input, one reads the replies
struct DirectDrive {
    MessageDebug   block;
    gr::MsgPortOut toMessages;
    gr::MsgPortOut toBlock;
    gr::MsgPortIn  fromBlock;

    explicit DirectDrive(gr::property_map settings) : block(std::move(settings)) {
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        std::ignore = toMessages.connect(block.messages);
        std::ignore = toBlock.connect(block.msgIn);
        std::ignore = block.msgOut.connect(fromBlock);
    }

    void send(gr::property_map data) { gr::sendMessage<gr::message::Command::Set>(toMessages, "sender", "value", std::move(data)); }

    void setSettings(gr::property_map settings) { gr::sendMessage<gr::message::Command::Set>(toBlock, "", gr::block::property::kSetting, std::move(settings)); }

    /// sends a Get on `endpoint` to the block and returns the reply's data, or nothing when no reply arrives
    std::optional<gr::property_map> get(std::string_view endpoint) {
        gr::sendMessage<gr::message::Command::Get>(toBlock, "", endpoint, gr::property_map{}, "query");
        block.processScheduledMessages();
        std::optional<gr::property_map> reply;
        auto                            span = fromBlock.streamReader().get(fromBlock.available());
        for (const gr::Message& message : span) {
            if (message.endpoint == endpoint && message.clientRequestID == "query" && message.data.has_value()) {
                reply = message.data.value();
            }
        }
        std::ignore = span.consume(span.size());
        return reply;
    }
};

template<typename T>
[[nodiscard]] T valueOr(const gr::property_map& map, std::string_view key, T fallback) {
    const auto it = map.find(std::pmr::string(key));
    if (it == map.end()) {
        return fallback;
    }
    const T* value = it->second.template get_if<T>();
    return value != nullptr ? *value : fallback;
}

[[nodiscard]] gr::property_map mapAt(const gr::property_map& map, std::string_view key) { return valueOr(map, key, gr::property_map{}); }

} // namespace

const boost::ut::suite<"MessageDebug"> messageDebugTests = [] {
    using namespace boost::ut;

    "three messages are counted and the last one is read back"_test = [] {
        gr::Graph flow;
        auto&     source = flow.emplaceBlock<IndexedMessageSource>();
        auto&     debug  = flow.emplaceBlock<MessageDebug>({{"name", std::string("debug")}, {"display", false}});
        expect(fatal(flow.connect<"out", "messages">(source, debug).has_value()));

        gr::scheduler::Simple scheduler;
        expect(fatal(scheduler.exchange(std::move(flow)).has_value()));
        expect(fatal(scheduler.runAndWait().has_value()));

        expect(eq(debug.n_messages.value, gr::Size_t{3}));
        expect(eq(valueOr(debug.last.value, "index", gr::Size_t{99}), gr::Size_t{2})) << "last holds the third message's data";

        const gr::property_map active = debug.settings().get();
        expect(eq(valueOr(active, "n_messages", gr::Size_t{0}), gr::Size_t{3})) << "the settings map carries the count";
        expect(eq(valueOr(mapAt(active, "last"), "index", gr::Size_t{99}), gr::Size_t{2})) << "the settings map carries the last data";
    };

    "a settings Get reads the count and the last data"_test = [] {
        DirectDrive drive({{"display", false}});
        drive.send({{"index", 0}});
        drive.send({{"index", 1}});
        drive.block.processScheduledMessages();

        const gr::property_map settings = drive.get(gr::block::property::kSetting).value_or(gr::property_map{});
        expect(eq(valueOr(settings, "n_messages", gr::Size_t{0}), gr::Size_t{2}));
        expect(eq(valueOr(mapAt(settings, "last"), "index", 99), 1));
    };

    "store 2 keeps the last two messages under Stored"_test = [] {
        DirectDrive drive({{"display", false}, {"store", gr::Size_t{2}}});
        drive.send({{"index", 0}});
        drive.send({{"index", 1}});
        drive.send({{"index", 2}});
        drive.block.processScheduledMessages();

        const gr::property_map stored = drive.get(MessageDebug::kStored).value_or(gr::property_map{});
        expect(eq(stored.size(), 2UZ));
        expect(!stored.contains(std::pmr::string("0"))) << "the oldest message is dropped";
        const gr::property_map second = mapAt(stored, "1");
        const gr::property_map third  = mapAt(stored, "2");
        expect(eq(valueOr(mapAt(second, "data"), "index", 99), 1));
        expect(eq(valueOr(mapAt(third, "data"), "index", 99), 2));
        expect(eq(valueOr(third, "cmd", std::pmr::string()), std::pmr::string("Set")));
        expect(eq(valueOr(third, "serviceName", std::pmr::string()), std::pmr::string("sender")));
        expect(eq(valueOr(third, "endpoint", std::pmr::string()), std::pmr::string("value")));
    };

    "store 0 keeps nothing"_test = [] {
        DirectDrive drive({{"display", false}});
        drive.send({{"index", 0}});
        drive.block.processScheduledMessages();
        expect(eq(drive.block.n_messages.value, gr::Size_t{1}));
        const std::optional<gr::property_map> stored = drive.get(MessageDebug::kStored);
        expect(fatal(stored.has_value())) << "the block replies on Stored";
        expect(stored->empty());
    };

    "a display change sent to the block applies before its next messages"_test = [] {
        DirectDrive drive({{"name", std::string("debug")}});
        drive.setSettings({{"display", false}});
        drive.send({{"index", 7}});
        StdoutCapture capture;
        drive.block.processScheduledMessages();
        const std::string printed = capture.release();

        expect(printed.empty()) << printed;
        expect(!drive.block.display.value);
        expect(eq(drive.block.n_messages.value, gr::Size_t{1}));
    };

    auto storeChangeDuringRun = []<typename TScheduler>() {
        gr::Graph flow;
        auto&     source = flow.emplaceBlock<SettingsChangingSource>();
        auto&     debug  = flow.emplaceBlock<MessageDebug>({{"name", std::string("debug")}, {"display", false}, {"store", gr::Size_t{3}}});
        expect(fatal(flow.connect<"out", "messages">(source, debug).has_value()));

        TScheduler scheduler;
        expect(fatal(scheduler.exchange(std::move(flow)).has_value()));
        gr::MsgPortOut toScheduler;
        gr::MsgPortIn  fromScheduler;
        expect(fatal(toScheduler.connect(scheduler.msgIn).has_value()));
        expect(fatal(scheduler.msgOut.connect(fromScheduler).has_value()));
        source.control = &toScheduler;
        source.change  = gr::property_map{{"store", gr::Size_t{1}}};
        expect(fatal(scheduler.runAndWait().has_value()));

        expect(eq(debug.n_messages.value, gr::Size_t{4}));
        expect(eq(debug.store.value, gr::Size_t{1})) << "the change applies during the run";
        gr::Message request;
        request.cmd                            = gr::message::Command::Get;
        const std::optional<gr::Message> reply = debug.propertyCallbackStored(MessageDebug::kStored, request);
        expect(fatal(reply.has_value() && reply->data.has_value()));
        expect(eq(reply->data->size(), 1UZ)) << "store 1 keeps one message";
        expect(eq(valueOr(mapAt(mapAt(*reply->data, "3"), "data"), "index", gr::Size_t{99}), gr::Size_t{3})) << "the kept message is the last one";
    };

    "a store change during a single-threaded run trims the kept messages"_test = [&storeChangeDuringRun] { storeChangeDuringRun.template operator()<gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::singleThreaded>>(); };

    "a store change during a multi-threaded run trims the kept messages"_test = [&storeChangeDuringRun] { storeChangeDuringRun.template operator()<gr::scheduler::Simple<gr::scheduler::ExecutionPolicy::multiThreaded>>(); };

    "display true prints one line per message"_test = [] {
        DirectDrive   drive({{"name", std::string("debug")}});
        StdoutCapture capture;
        drive.send({{"index", 7}});
        drive.block.processScheduledMessages();
        const std::string printed = capture.release();

        expect(eq(printed, std::format("debug: cmd: Set, serviceName: 'sender', endpoint: 'value', data: {}\n", gr::property_map{{"index", 7}})));
    };

    "display false prints nothing"_test = [] {
        DirectDrive   drive({{"display", false}});
        StdoutCapture capture;
        drive.send({{"index", 7}});
        drive.block.processScheduledMessages();
        const std::string printed = capture.release();

        expect(printed.empty()) << printed;
        expect(eq(drive.block.n_messages.value, gr::Size_t{1})) << "the message is still counted";
    };

    "an error message prints its error"_test = [] {
        DirectDrive     drive({{"name", std::string("debug")}});
        const gr::Error error("the sender failed");
        StdoutCapture   capture;
        gr::sendMessage<gr::message::Command::Notify>(drive.toMessages, "sender", "value", error);
        drive.block.processScheduledMessages();
        const std::string printed = capture.release();

        expect(eq(printed, std::format("debug: cmd: Notify, serviceName: 'sender', endpoint: 'value', error: {}\n", error)));
        expect(eq(drive.block.n_messages.value, gr::Size_t{1}));
        expect(drive.block.last.value.empty()) << "an error message carries no data map";
    };
};

int main() { /* not needed for UT */ }
