#ifndef GR4_BASIC_NAMESPACE_COMPATIBILITY_HPP
#define GR4_BASIC_NAMESPACE_COMPATIBILITY_HPP

namespace gr::basic {}

namespace gr::blocks {

// DEPRECATED COMPATIBILITY ALIAS: Do not use in new code. Use gr::basic instead.
// This legacy namespace alias is temporary and will be removed in a future release.
namespace basic = ::gr::basic;

namespace type {

// DEPRECATED COMPATIBILITY ALIAS: Do not use in new code. Use gr::basic instead.
// This legacy namespace alias is temporary and will be removed in a future release.
namespace converter = ::gr::basic;

} // namespace type
} // namespace gr::blocks

#endif // GR4_BASIC_NAMESPACE_COMPATIBILITY_HPP
