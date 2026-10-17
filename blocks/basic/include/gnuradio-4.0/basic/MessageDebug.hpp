#ifndef GNURADIO_BASIC_MESSAGEDEBUG_HPP
#define GNURADIO_BASIC_MESSAGEDEBUG_HPP

#include <cstddef>
#include <cstdio>
#include <deque>
#include <format>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::MessageDebug)

struct MessageDebug : Block<MessageDebug> {
    using Description = Doc<R""(
@brief Prints and counts the messages that arrive at its `messages` port.

Each message prints one line to standard output: the block's `name`, the message's `cmd`, `serviceName` and
`endpoint`, then its `data` map or the error it carries. The block counts every message in `n_messages` and keeps the
`data` map of the most recent one in `last`. A settings `Get` reads both. With `store` above 0, the block keeps that
many of the most recent messages. A `Get` on the block's `Stored` property returns them in a map keyed by each
message's index in the order of arrival, starting at 0.

The block has no stream ports. Its work function returns done on the first call. A change to `display` or `store`
applies when the block next handles its messages.
)"">;

    MsgPortIn messages;

    Annotated<bool, "display", Doc<"print each message to standard output">, Visible>   display    = true;
    Annotated<gr::Size_t, "store", Doc<"number of most recent messages kept">, Visible> store      = 0U;
    Annotated<gr::Size_t, "n_messages", Doc<"messages received">>                       n_messages = 0U;
    Annotated<property_map, "last", Doc<"data map of the most recent message">>         last{};

    GR_MAKE_REFLECTABLE(MessageDebug, messages, display, store, n_messages, last);

    std::deque<std::pair<gr::Size_t, Message>> _stored{};

    static constexpr std::string_view kStored = "Stored";

    explicit MessageDebug(property_map initParameters = {}) : Block<MessageDebug>(std::move(initParameters)) { //
        propertyCallbacks[std::string(kStored)] = static_cast<BlockBase::PropertyCallback>(&MessageDebug::propertyCallbackStored);
    }

    /// handles the block's own properties on its built-in port, then applies the settings staged there
    void processMessages(const MsgPortInBuiltin& port, std::span<const Message> requests) {
        BlockBase::processMessages(port, requests);
        this->applyChangedSettings(false);
    }

    void processMessages(MsgPortIn& /*port*/, std::span<const Message> received) {
        for (const Message& message : received) {
            if (display) {
                printMessage(message);
            }
            if (store > 0U) {
                _stored.emplace_back(n_messages.value, message);
                trimStored();
            }
            ++n_messages.value;
            last = message.data.has_value() ? message.data.value() : property_map{};
        }
        if (!received.empty()) {
            this->updateActiveParameters({"n_messages", "last"});
        }
    }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { trimStored(); }

    [[nodiscard]] work::Status processBulk() noexcept { return work::Status::DONE; }

    std::optional<Message> propertyCallbackStored(std::string_view propertyName, Message message) {
        if (message.cmd != message::Command::Get) {
            throw gr::exception(std::format("block {} property {} does not implement command {}", this->unique_name, propertyName, message.cmd));
        }
        property_map stored;
        for (const auto& [index, kept] : _stored) {
            stored.insert_or_assign(std::pmr::string(std::to_string(index)), storedEntry(kept));
        }
        message.data = std::move(stored);
        return message;
    }

private:
    void printMessage(const Message& message) const {
        if (message.data.has_value()) {
            std::println("{}: cmd: {}, serviceName: '{}', endpoint: '{}', data: {}", this->name, message.cmd, message.serviceName, message.endpoint, message.data.value());
        } else {
            std::println("{}: cmd: {}, serviceName: '{}', endpoint: '{}', error: {}", this->name, message.cmd, message.serviceName, message.endpoint, message.data.error());
        }
    }

    void trimStored() {
        while (_stored.size() > static_cast<std::size_t>(store.value)) {
            _stored.pop_front();
        }
    }

    [[nodiscard]] static property_map storedEntry(const Message& message) {
        property_map entry{{"cmd", std::format("{}", message.cmd)}, {"serviceName", message.serviceName}, {"endpoint", message.endpoint}};
        if (message.data.has_value()) {
            entry.insert_or_assign("data", message.data.value());
        } else {
            entry.insert_or_assign("error", std::format("{}", message.data.error()));
        }
        return entry;
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_BASIC_MESSAGEDEBUG_HPP
