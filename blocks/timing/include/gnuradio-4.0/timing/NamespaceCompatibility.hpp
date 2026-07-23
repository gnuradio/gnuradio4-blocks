#ifndef GR4_TIMING_NAMESPACE_COMPATIBILITY_HPP
#define GR4_TIMING_NAMESPACE_COMPATIBILITY_HPP

namespace gr::blocks::timing {}

namespace gr {

// DEPRECATED COMPATIBILITY ALIAS: Do not use in new code. Use gr::blocks::timing instead.
// This legacy namespace alias is temporary and will be removed in a future release.
namespace timing = blocks::timing;

} // namespace gr

#endif // GR4_TIMING_NAMESPACE_COMPATIBILITY_HPP
