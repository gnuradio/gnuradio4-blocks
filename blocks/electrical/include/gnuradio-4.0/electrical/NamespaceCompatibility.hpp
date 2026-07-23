#ifndef GR4_ELECTRICAL_NAMESPACE_COMPATIBILITY_HPP
#define GR4_ELECTRICAL_NAMESPACE_COMPATIBILITY_HPP

namespace gr::blocks::electrical {}

namespace gr {

// DEPRECATED COMPATIBILITY ALIAS: Do not use in new code. Use gr::blocks::electrical instead.
// This legacy namespace alias is temporary and will be removed in a future release.
namespace electrical = blocks::electrical;

} // namespace gr

#endif // GR4_ELECTRICAL_NAMESPACE_COMPATIBILITY_HPP
