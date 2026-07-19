#ifndef GR4_SDR_NAMESPACE_COMPATIBILITY_HPP
#define GR4_SDR_NAMESPACE_COMPATIBILITY_HPP

namespace gr::sdr {}

namespace gr::blocks {

// DEPRECATED COMPATIBILITY ALIAS: Do not use in new code. Use gr::sdr instead.
// This legacy namespace alias is temporary and will be removed in a future release.
namespace sdr = ::gr::sdr;

} // namespace gr::blocks

#endif // GR4_SDR_NAMESPACE_COMPATIBILITY_HPP
