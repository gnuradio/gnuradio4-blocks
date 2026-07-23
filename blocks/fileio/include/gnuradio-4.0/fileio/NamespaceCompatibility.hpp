#ifndef GR4_FILEIO_NAMESPACE_COMPATIBILITY_HPP
#define GR4_FILEIO_NAMESPACE_COMPATIBILITY_HPP

namespace gr::blocks::fileio {}

namespace gr {

// DEPRECATED COMPATIBILITY ALIAS: Do not use in new code. Use gr::blocks::fileio instead.
// This interim namespace alias is temporary and will be removed in a future release.
namespace fileio = blocks::fileio;

} // namespace gr

#endif // GR4_FILEIO_NAMESPACE_COMPATIBILITY_HPP
