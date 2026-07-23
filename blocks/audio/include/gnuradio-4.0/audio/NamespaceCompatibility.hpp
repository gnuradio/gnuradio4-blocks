#ifndef GR4_AUDIO_NAMESPACE_COMPATIBILITY_HPP
#define GR4_AUDIO_NAMESPACE_COMPATIBILITY_HPP

namespace gr::blocks::audio {}

namespace gr {

// DEPRECATED COMPATIBILITY ALIAS: Do not use in new code. Use gr::blocks::audio instead.
// This legacy namespace alias is temporary and will be removed in a future release.
namespace audio = blocks::audio;

} // namespace gr

#endif // GR4_AUDIO_NAMESPACE_COMPATIBILITY_HPP
