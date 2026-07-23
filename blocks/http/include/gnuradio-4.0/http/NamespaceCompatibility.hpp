#ifndef GR4_HTTP_NAMESPACE_COMPATIBILITY_HPP
#define GR4_HTTP_NAMESPACE_COMPATIBILITY_HPP

namespace gr::blocks::http {}

namespace gr {

// DEPRECATED COMPATIBILITY ALIAS: Do not use in new code. Use gr::blocks::http instead.
// This legacy namespace alias is temporary and will be removed in a future release.
namespace http = blocks::http;

} // namespace gr

#endif // GR4_HTTP_NAMESPACE_COMPATIBILITY_HPP
