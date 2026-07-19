#ifndef GR4_FILEIO_NAMESPACE_COMPATIBILITY_HPP
#define GR4_FILEIO_NAMESPACE_COMPATIBILITY_HPP

namespace gr::fileio {}

namespace gr::blocks {

// DEPRECATED COMPATIBILITY ALIAS: Do not use in new code. Use gr::fileio instead.
// This legacy namespace alias is temporary and will be removed in a future release.
namespace fileio = ::gr::fileio;

} // namespace gr::blocks

#endif // GR4_FILEIO_NAMESPACE_COMPATIBILITY_HPP
