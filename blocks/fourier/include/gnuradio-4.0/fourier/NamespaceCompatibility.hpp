#ifndef GR4_FOURIER_NAMESPACE_COMPATIBILITY_HPP
#define GR4_FOURIER_NAMESPACE_COMPATIBILITY_HPP

namespace gr::blocks::fourier {}

namespace gr {

// DEPRECATED COMPATIBILITY ALIAS: Do not use in new code. Use gr::blocks::fourier instead.
// This interim namespace alias is temporary and will be removed in a future release.
namespace fourier = blocks::fourier;

} // namespace gr

namespace gr::blocks {

// DEPRECATED COMPATIBILITY ALIAS: Do not use in new code. Use gr::blocks::fourier instead.
// This legacy namespace alias is temporary and will be removed in a future release.
namespace fft = fourier;

} // namespace gr::blocks

#endif // GR4_FOURIER_NAMESPACE_COMPATIBILITY_HPP
