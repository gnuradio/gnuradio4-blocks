#ifndef GNURADIO_NETWORK_ZMQTRANSPORT_HPP
#define GNURADIO_NETWORK_ZMQTRANSPORT_HPP

#include <cstdint>
#include <format>
#include <string>
#include <string_view>

#include <gnuradio-4.0/Message.hpp>

/**
 * @brief The socket vocabulary the module's ZeroMQ blocks share: the four patterns, the two names each role
 * accepts, and the one endpoint diagnostic worth stating.
 *
 * It sits in a header of its own because the alternative is a second copy of one table and one piece of
 * user-facing text — the drift these blocks exist to avoid — and because a block carrying a raw sample stream has
 * no business including the packet envelope to learn what 'sub' means.
 */
namespace gr::blocks::network::detail::zmqio {

/// @brief The socket pattern a block runs, resolved from its `pattern` setting once at configure time.
enum class Pattern : std::uint8_t { Pub, Push, Sub, Pull };

/// @brief The one endpoint diagnostic worth stating rather than leaving to be rediscovered.
///
/// libzmq answers an endpoint with no transport prefix with a bare `EINVAL`, and that is the common typo.
[[nodiscard]] inline std::string endpointHint(std::string_view endpoint) { return endpoint.find("://") == std::string_view::npos ? std::format(" — '{}' names no transport; a libzmq endpoint begins with a prefix such as tcp://, ipc:// or inproc://", endpoint) : std::string{}; }

[[nodiscard]] inline Pattern sendPatternFromName(std::string_view name) {
    if (name == "pub") {
        return Pattern::Pub;
    }
    if (name == "push") {
        return Pattern::Push;
    }
    throw gr::exception(std::format("pattern is '{}'; a sink takes 'pub' (fan out, never blocks, drops in mute state) or 'push' (round-robin, blocks in mute state, discards nothing)", name));
}

[[nodiscard]] inline Pattern receivePatternFromName(std::string_view name) {
    if (name == "sub") {
        return Pattern::Sub;
    }
    if (name == "pull") {
        return Pattern::Pull;
    }
    throw gr::exception(std::format("pattern is '{}'; a source takes 'sub' (subscribes to a prefix) or 'pull' (fair-queued)", name));
}

} // namespace gr::blocks::network::detail::zmqio

#endif // GNURADIO_NETWORK_ZMQTRANSPORT_HPP
