#ifndef GR4_FOURIER_NAMESPACE_COMPATIBILITY_HPP
#define GR4_FOURIER_NAMESPACE_COMPATIBILITY_HPP

namespace gr::fourier {}

namespace gr::blocks {

// DEPRECATED COMPATIBILITY ALIAS: Do not use in new code. Use gr::fourier instead.
// This legacy namespace alias is temporary and will be removed in a future release.
namespace fft = ::gr::fourier;

} // namespace gr::blocks

#endif // GR4_FOURIER_NAMESPACE_COMPATIBILITY_HPP
