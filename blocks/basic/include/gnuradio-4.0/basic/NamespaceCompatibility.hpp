#ifndef GR4_BASIC_NAMESPACE_COMPATIBILITY_HPP
#define GR4_BASIC_NAMESPACE_COMPATIBILITY_HPP

namespace gr::blocks::basic {}

namespace gr {

// DEPRECATED COMPATIBILITY ALIAS: Do not use in new code. Use gr::blocks::basic instead.
// This interim namespace alias is temporary and will be removed in a future release.
namespace basic = blocks::basic;

} // namespace gr

namespace gr::blocks::type {
// DEPRECATED COMPATIBILITY ALIAS: Do not use in new code. Use gr::blocks::basic instead.
// This legacy namespace alias is temporary and will be removed in a future release.
namespace converter = basic;

} // namespace gr::blocks::type

#endif // GR4_BASIC_NAMESPACE_COMPATIBILITY_HPP
