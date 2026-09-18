// The consumer project beside this file is configured and then built. Configuring proves that every target the
// exported interface of gnuradio4::gr-audio names can be defined again by whoever finds the installed package;
// compiling and linking prove that what the interface carries is enough to use the family's headers and to resolve
// what they call. The claims fail in different places: a missing target name ends the configure, a missing include
// directory is not noticed until a translation unit asks for a header, and a missing library not until the link.
// <soundio/soundio.h> is the header that asks, because libsoundio ships no package files and its directory is the one
// an install cannot assume.

#include <gnuradio-4.0/audio/AudioBlocks.hpp>

int main() {
    const gr::blocks::audio::AudioSink<float> sink;
    return sink.sample_rate.value > 0.f ? 0 : 1;
}
