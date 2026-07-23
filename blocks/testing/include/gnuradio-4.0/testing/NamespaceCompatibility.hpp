#ifndef GR4_TESTING_NAMESPACE_COMPATIBILITY_HPP
#define GR4_TESTING_NAMESPACE_COMPATIBILITY_HPP

namespace gr::blocks::testing {}

namespace gr::testing {

// DEPRECATED COMPATIBILITY IMPORT: Do not use in new code. Use gr::blocks::testing instead.
// This legacy namespace forwarding is temporary and will be removed in a future release.
using namespace ::gr::blocks::testing;

} // namespace gr::testing

#endif // GR4_TESTING_NAMESPACE_COMPATIBILITY_HPP
