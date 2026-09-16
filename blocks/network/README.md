# network

Carrying a flowgraph off the box. The blocks here put a record, a sample stream or a message on
a wire and take it off again, all in the one versioned envelope `gr::network::PacketEnvelope`
defines: a 32-byte header that states the item type, item size, lengths, byte order and wire
version, a metadata frame serialized by `gr::pmt::yaml`, and the payload.

The envelope is transport-neutral, so this module is named for the family rather than for one
transport: ZeroMQ is the first carrier and a byte-stream transport is meant to land beside it
without a second format.

The family is off by default (`GR4_ENABLE_NETWORK`) and is built only where libzmq and the
cppzmq header are found, because an optional external dependency cannot enter an unconditional
module without becoming unconditional for everything downstream.

Two pairs share that envelope, and each carries a different one of the things a flowgraph is made of.

| pair                                      | what one packet is                | payload             |
| ----------------------------------------- | --------------------------------- | ------------------- |
| `ZmqPacketSink` / `ZmqPacketSource`       | one `gr::Packet<T>`               | its `signal_values` |
| `StreamPacketSink` / `StreamPacketSource` | a chunk of a tagged sample stream | the samples         |

The header is the same for both and the wire version is 1 throughout: a reader written against the envelope reads
either, and what differs is only which reserved keys the metadata frame carries.

## The stream pair, and how a flowgraph continues in another process

`StreamPacketSink` takes a tagged stream of `T`, cuts it into packets of at most `max_items` samples, and publishes
each as one envelope; `StreamPacketSource` takes them and republishes the samples with every carried tag standing at
the sample it stood on before the crossing. Together they are a flowgraph edge with a process boundary in the middle:
everything upstream of the sink runs in one process, everything downstream of the source in another, and neither half
is written differently for it. `blocks/network/examples/` holds the two halves of one such graph.

The one property that governs the design is that **a hole in the stream is announced, never spliced**. A subscriber
loses packets inside libzmq with no error at either end, so every packet states both `sequence` and the absolute
`stream_position`, and the receiver publishes one tag at the next sample rather than concatenating across the hole.

## The reserved metadata keys

The record vocabulary is `blocks/basic`'s and is documented there. These are what the stream pair adds to a metadata
frame, each serving one boundary crossing and consumed by the receiving block, so the vocabulary's one-spelling rule
survives the crossing.

| key               | type         | pair   | meaning                                                                                         |
| ----------------- | ------------ | ------ | ----------------------------------------------------------------------------------------------- |
| `sequence`        | `uint64`     | both   | the packet's ordinal at the sender; a gap in it is loss                                         |
| `sample_rate`     | `float32`    | stream | the rate the stream runs at, stated on every packet                                             |
| `stream_position` | `uint64`     | stream | absolute index of the chunk's first sample, counted by the sink from its start                  |
| `packet_tags`     | list of maps | stream | the tags in the chunk, each `{offset: uint64, map: map}`, offsets from the chunk's first sample |

`stream_position` is deliberately not the record vocabulary's `sample_start`: that key states where a _record_ begins
in a producer's stream, while this one is the sink's own count over the edge it was placed on. On the receiver's gap
tag it reappears, naming the first sample the hole swallowed, beside `n_dropped_samples` — the framework's own
reserved key, so a block that already understands dropped samples needs no new vocabulary — and `packets_lost`. The
missing span is `[stream_position, stream_position + n_dropped_samples)`.

## The settings

All four blocks take the same socket settings, and each side of a pair matches the other's: `push` pairs with `pull`,
`pub` with `sub`.

| setting                         | sinks                                | sources                            |
| ------------------------------- | ------------------------------------ | ---------------------------------- |
| `endpoint`                      | required, no default                 | required, no default               |
| `bind`                          | `true`                               | `false`                            |
| `pattern`                       | `pub` or `push`                      | `sub` or `pull`                    |
| `topic`                         | frame 0, the subscription prefix     | `ZMQ_SUBSCRIBE` prefix, `sub` only |
| `max_message_bytes`             | 16 MiB; a larger envelope is refused | **required, no default**           |
| `queue_messages`, `queue_bytes` | the in-process queue's bounds        | the same                           |
| `send_hwm` / `recv_hwm`         | `ZMQ_SNDHWM`, 16                     | `ZMQ_RCVHWM`, 16                   |
| `linger_ms`                     | `ZMQ_LINGER`, 0                      | `ZMQ_LINGER`, 0                    |
| `overflow`                      | `drop_oldest` or `backpressure`      | —                                  |

`StreamPacketSink` adds two: `max_items`, the samples per packet, where 0 emits one packet per work call; and
`sample_rate`, the rate to state where the stream itself tags none, which a tag in the stream always overrides.

A source's `max_message_bytes` has no default because libzmq's own bound is no limit, so a source that omitted it
would let a peer's claimed length size an allocation before the block saw a byte.

`ZmqStreamSource` is the exception to the envelope, and is here for compatibility alone: it reads the
raw sample stream GNU Radio 3.10's `gr-zeromq` publishes — one message per buffer of items, no header
and no framing — so a stock 3.10 `zmq_pub_sink` or `zmq_push_sink` feeds a GR 4 graph unchanged and a
migration can move one end at a time. A downstream program carrying its own copy of such a block should
instantiate `gr::blocks::network::ZmqStreamSource<T>` by that name and delete the copy; the settings it
takes are `endpoint`, `bind`, `pattern`, `topic`, `max_message_bytes`, `queue_bytes`, `recv_hwm` and
`linger_ms`, and `max_message_bytes` has no default.

`src/` holds the two executables of the packet-link acceptance suite, `packet_link_tx` and
`packet_link_rx`. Each is a flowgraph of stock blocks and nothing else, and `test/qa_PacketLink`
runs them as separate processes over one endpoint: a format is only proved by a peer that did
not build it, and two processes exchanging a known byte vector is the smallest thing that shows
type, length, byte order, version, record metadata and error handling all crossing intact.
