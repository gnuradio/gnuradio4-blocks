#ifndef GR4_FILTER_NAMESPACE_COMPATIBILITY_HPP
#define GR4_FILTER_NAMESPACE_COMPATIBILITY_HPP

namespace gr::blocks::filter {}

namespace gr::filter {

// DEPRECATED COMPATIBILITY IMPORT: Do not use in new code. Use gr::blocks::filter instead.
// This legacy namespace forwarding is temporary and will be removed in a future release.
using namespace ::gr::blocks::filter;

} // namespace gr::filter

#endif // GR4_FILTER_NAMESPACE_COMPATIBILITY_HPP
