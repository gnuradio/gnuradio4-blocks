#ifndef GR4_MATH_NAMESPACE_COMPATIBILITY_HPP
#define GR4_MATH_NAMESPACE_COMPATIBILITY_HPP

namespace gr::math {}

namespace gr::blocks {

// DEPRECATED COMPATIBILITY ALIAS: Do not use in new code. Use gr::math instead.
// This legacy namespace alias is temporary and will be removed in a future release.
namespace math = ::gr::math;

} // namespace gr::blocks

#endif // GR4_MATH_NAMESPACE_COMPATIBILITY_HPP
