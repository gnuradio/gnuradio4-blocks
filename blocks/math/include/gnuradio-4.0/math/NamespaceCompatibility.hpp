#ifndef GR4_MATH_NAMESPACE_COMPATIBILITY_HPP
#define GR4_MATH_NAMESPACE_COMPATIBILITY_HPP

namespace gr::blocks::math {}

namespace gr::math {

// DEPRECATED COMPATIBILITY IMPORT: Do not use in new code. Use gr::blocks::math instead.
// This interim namespace forwarding is temporary and will be removed in a future release.
using namespace ::gr::blocks::math;

} // namespace gr::math

#endif // GR4_MATH_NAMESPACE_COMPATIBILITY_HPP
