# network

Carrying a packet off the box. The blocks here put a `gr::Packet<T>` on a wire and take one
off again, in the versioned envelope `gr::network::PacketEnvelope` defines: a 32-byte header
that states the item type, item size, lengths, byte order and wire version, a metadata frame
that is the packet's own map serialized by `gr::pmt::yaml`, and the payload.

The envelope is transport-neutral, so this module is named for the family rather than for one
transport: ZeroMQ is the first carrier and a byte-stream transport is meant to land beside it
without a second format.

The family is off by default (`GR4_ENABLE_NETWORK`) and is built only where libzmq and the
cppzmq header are found, because an optional external dependency cannot enter an unconditional
module without becoming unconditional for everything downstream.
