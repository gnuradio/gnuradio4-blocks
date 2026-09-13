#pragma once
#include <stdexcept>

namespace gr::blocks::zeromq::detail {
// Only explicit peer-input validation failures use this type. Allocation,
// transport, configuration and publication exceptions must propagate.
class DecodeError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};
} // namespace gr::blocks::zeromq::detail
