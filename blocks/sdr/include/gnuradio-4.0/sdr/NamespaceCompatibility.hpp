#ifndef GR4_SDR_NAMESPACE_COMPATIBILITY_HPP
#define GR4_SDR_NAMESPACE_COMPATIBILITY_HPP

namespace gr::blocks::sdr {}

namespace gr {

// DEPRECATED COMPATIBILITY ALIAS: Do not use in new code. Use gr::blocks::sdr instead.
// This interim namespace alias is temporary and will be removed in a future release.
namespace sdr = blocks::sdr;

} // namespace gr

#endif // GR4_SDR_NAMESPACE_COMPATIBILITY_HPP
