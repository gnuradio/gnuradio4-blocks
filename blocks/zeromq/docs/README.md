# GNU Radio 4 ZeroMQ Blocks

`blocks/zeromq` provides ZeroMQ transport for GR4 typed ports, with selected
GNU Radio 3 stream and PMT interoperability modes. The socket-pattern templates
also accept `T = gr::pmt::Value`; the table below maps socket patterns, not a
promise that every combination of settings interoperates with GR3.

## Socket Patterns

| GNU Radio 3 block        | GR4 block                       |
| ------------------------ | ------------------------------- |
| `zeromq.push_sink`       | `ZmqPushSink<T>`                |
| `zeromq.pull_source`     | `ZmqPullSource<T>`              |
| `zeromq.pub_sink`        | `ZmqPubSink<T>`                 |
| `zeromq.sub_source`      | `ZmqSubSource<T>`               |
| `zeromq.req_source`      | `ZmqReqSource<T>`               |
| `zeromq.rep_sink`        | `ZmqRepSink<T>`                 |
| `zeromq.push_msg_sink`   | `ZmqPushSink<gr::pmt::Value>`   |
| `zeromq.pull_msg_source` | `ZmqPullSource<gr::pmt::Value>` |
| `zeromq.pub_msg_sink`    | `ZmqPubSink<gr::pmt::Value>`    |
| `zeromq.sub_msg_source`  | `ZmqSubSource<gr::pmt::Value>`  |
| `zeromq.req_msg_source`  | `ZmqReqSource<gr::pmt::Value>`  |
| `zeromq.rep_msg_sink`    | `ZmqRepSink<gr::pmt::Value>`    |

The registered stream payload set matches GNU Radio 3's ZMQ stream item sizes:
`uint8_t` maps to `gr.sizeof_char` byte streams, followed by `int16_t`,
`int32_t`, `float`, and `std::complex<float>`. GR4 also registers
`std::vector<float>` and `std::vector<std::complex<float>>` for vector payload
convenience, plus `gr::pmt::Value` for GNU Radio message-domain interop.

Configure with `-DGR4_ENABLE_ZEROMQ=ON` to build the family. Its five example
programs are separate from the library and tests; enable them with
`-DGR4_ENABLE_ZEROMQ_EXAMPLES=ON` as well as the project-wide
`-DENABLE_EXAMPLES=ON`.

The registration set controls block discovery and factory construction; the
underlying templates may still be instantiated directly for other arithmetic or
complex payload types when an application owns both endpoints.

## Common Settings

- `endpoint`: ZeroMQ endpoint, for example `tcp://127.0.0.1:5555`.
- `timeout`: non-negative wait budget in milliseconds. Positive values are
  honored as the total wait budget and are polled internally in bounded slices;
  `0` is nonblocking. Large timeout values may therefore increase graph
  shutdown latency.
- `bind`: `true` makes the GR4 block bind; `false` makes it connect.
- `hwm`: send/receive high-water mark. `-1` keeps the underlying default.
- `linger`: socket linger value in milliseconds.
- `max_message_size`: positive maximum inbound frame size in bytes for Pull,
  Sub, and Req sources (default 64 MiB), and Rep sinks (default 4096 bytes).
  The limit is applied before bind/connect. For TCP and IPC, libzmq rejects an
  oversized frame and disconnects its peer before exposing it to the block.
  REP's limit leaves room for ZMTP handshake metadata, routing identities and
  correlated REQ envelopes; its application body must still be exactly four
  bytes. Four bytes is too small as a general transport limit. Inproc transports
  share already allocated messages and do not enforce this network decoder limit.
- `pass_tags`: enables GNU Radio compatible tag-header framing.
- `pmt_wire_format`: PMT codec selection, `GR4_YAML_V1` (default) or `GR3`.
  `GR4_YAML_V1` preserves native `gr::pmt::Value` types using the YAML format
  described below. Select `GR3` to use GNU Radio 3's legacy binary PMT encoding
  for interoperability. Configure both peers with the same value; there is no
  automatic detection or fallback. This setting affects only
  `gr::pmt::Value` payloads and PMT values inside tag headers, never raw sample
  payloads.

Pub/sub blocks also support:

- `key`: topic string. An empty subscriber key receives all topics.
- `drop_on_hwm`: when `true` (the default), `ZmqPubSink` uses normal PUB
  high-water-mark dropping. When `false`, it keeps PUB and sets
  `ZMQ_XPUB_NODROP`; a full outbound queue then refuses the send and leaves
  the corresponding input unconsumed so the scheduler can retry it. Configuration
  failures propagate; there is no fallback to dropping. This protects established
  subscriptions only: absent or not-yet-subscribed peers have no delivery guarantee.
  PUB inherits this option from XPUB in libzmq 4.3.5, without retaining XPUB's
  subscription notifications.

All source blocks expose `receive_statistics()`, and all sink blocks expose
`send_statistics()`. The snapshots count accepted messages and stream items,
malformed or refused messages, and block-side PUB drops. In normal PUB drop
mode, libzmq reports a successful send even when it drops data for a slow
subscriber, so subscriber-specific HWM drops cannot be counted by the block.

## Execution Model

Socket I/O runs synchronously in `processBulk`; these blocks do not create a
dedicated I/O thread or an intermediate queue. The first receive or send in a
work call may therefore occupy its scheduler worker for up to `timeout`
(100 ms by default). The wait uses polling slices of at most 10 ms so lifecycle
changes can cancel it promptly. Later socket operations in the same work call
share that original deadline and do not each add another full timeout.

This design keeps buffering and backpressure in the scheduler and libzmq. Set
`timeout=0` for nonblocking scheduler calls, or use a smaller positive value
when scheduler-worker latency matters more than waiting for a peer. Deployments
that require socket isolation should assign these blocks suitable scheduler
resources or place the transport behind an application-owned I/O thread and
queue.

## Raw Stream Wire Format

Raw stream messages are byte buffers containing an integer number of stream
items. For scalar `T`, a message contains contiguous `sizeof(T)` items. For
`std::vector<T>`, each received ZMQ message becomes one vector item whose
elements are decoded from the payload bytes.

Source blocks validate that incoming payload sizes are multiples of the
effective item size. Messages larger than the scheduler output span are retained
and flushed across later work calls using an offset into the retained buffer.

## REQ/REP Protocol

`ZmqReqSource<T>` sends a native-endian `uint32_t` request containing the desired
number of output items, matching GNU Radio 3 ZeroMQ blocks. `ZmqRepSink<T>` waits
for a request and replies with up to that many items. `gr::pmt::Value` request/reply
transport sends one PMT per request.

## PMT Payloads

`gr::pmt::Value` payload blocks use the selected `pmt_wire_format`. The
`GR4_YAML_V1` format encodes a PMT in a one-key `{value: ...}` envelope with GNU
Radio 4 core's typed YAML codec, preserving scalar widths such as `uint8_t`,
`float`, and `std::complex<float>`. A four-byte, big-endian YAML byte length
precedes the envelope so tag headers can contain several self-delimiting PMTs.
The versioned name distinguishes this format from a future core binary PMT
format; it is not an alias for such a binary format.

Select `GR3` for GNU Radio 3 interoperability. It uses the legacy GNU Radio 3
PMT binary encoding and may widen some native GR4 scalar types during a
roundtrip. The blocks do not auto-detect or fall back between formats.

The GR3 codec supports null, booleans, signed and unsigned integers,
floating-point and complex scalars, symbols, supported uniform vectors, pairs,
tuples, and dictionaries. Generic `LEGACY_PMT_VECTOR` values are not
supported in GR3 mode and are rejected. Pull, Sub, and Req sources discard a
malformed raw or PMT message, increment `receive_statistics().refused_messages`,
and continue accepting subsequent messages. Their statistics also report total
received messages and accepted output items for the current run.

## Tag Headers

When `pass_tags=true`, the blocks use the GNU Radio `gr-zeromq` tag header
before the payload:

```text
uint16 magic = 0x5FF0
uint8  version = 0x01
uint64 stream offset
uint64 tag count
repeat tag count:
  uint64 tag offset
  serialized PMT key
  serialized PMT value
  serialized PMT srcid
raw payload bytes
```

Tag `key`, `value`, and `srcid` fields use the selected PMT wire format. On
GNU Radio 4 ports, these fields are represented as tag maps with `key`, `value`, and
`srcid` entries.

Only the envelope `{key: <PMT>, value: <PMT>, srcid: <optional PMT>}` is
converted. Both `key` and `value` are required; missing `srcid` becomes null.
Other entries are ignored, and a native map such as `{sample_rate: 48000}` is
not converted. For GR3, the key must be a symbol/string supported by its tag
consumer. This is an envelope contract, not arbitrary native-map preservation.

Multiple tags at the same item offset are preserved. Scalar headers carry offsets
relative to the scalar items in that message. A GR4 vector or PMT is one port
item: its message carries only that item's tags, rebased to offset zero. It does
not provide per-element tags inside a vector. Partial sends retain the remaining
input and its tags for the next call.

## Compatibility boundary

| Payload / peer         | `pmt_wire_format`                                            | `pass_tags` | PUB/SUB `key`           | Item convention                          |
| ---------------------- | ------------------------------------------------------------ | ----------- | ----------------------- | ---------------------------------------- |
| Raw scalars, GR4 ↔ GR4 | either; equal on tagged peers                                | off or on   | empty or matching topic | scalar stream items                      |
| Raw scalars, GR3 ↔ GR4 | `GR3` for tagged traffic; irrelevant without tags            | off or on   | matching GR3 stream key | supported GR3 `vlen=1`                   |
| Vector, GR4 ↔ GR4      | either; equal on tagged peers                                | off or on   | empty or matching topic | one whole message per vector item        |
| PMT, GR4 ↔ GR4         | equal on peers; default `GR4_YAML_V1` preserves native types | off or on   | empty or matching topic | one PMT per message                      |
| PMT, GR3 message peer  | explicitly `GR3`                                             | **off**     | **empty**               | supported legacy PMT representation only |

The GR3 PMT message blocks serialize a PMT directly, without stream tag headers
or a topic frame. Thus `pass_tags=true` on PMT is a supported GR4-to-GR4 mode,
but does not interoperate with GR3 message blocks. GR4's PMT PUB/SUB honors `key`
just like its raw blocks: PUB sends a topic frame before the payload, SUB
subscribes to a prefix and decodes the last frame. Use an empty key with GR3
message peers; filtering serialized PMT bytes is not a portable topic contract.

GR4 has no `vlen` setting. Its vector ports represent message boundaries, not
GR3's fixed-size stream vectors. Untagged bytes may be repacked by applications,
but fixed `vlen>1` tag offsets and REQ item counts are outside this compatibility
contract. No automatic vector-length negotiation or conversion is performed.
The interoperability suite covers scalar `vlen=1`, tagged scalar streams, and
PMT PUSH/PULL; it is not coverage of every entry in the socket-pattern table.

## Examples

The examples are installed as small role-based programs. The raw examples use
untagged samples. `zmq_pmt_payload` explicitly selects `GR3` for both roles;
`zmq_loopback` explicitly selects `GR3` for PMT fields in its stream tag headers.
The block default remains `GR4_YAML_V1`. Start the producing
side first unless noted otherwise.

```bash
zmq_push_pull push tcp://127.0.0.1:5555
zmq_push_pull pull tcp://127.0.0.1:5555
```

```bash
zmq_pub_sub pub tcp://127.0.0.1:5556 demo
zmq_pub_sub sub tcp://127.0.0.1:5556 demo
```

```bash
zmq_req_rep rep tcp://127.0.0.1:5557
zmq_req_rep req tcp://127.0.0.1:5557
```

```bash
zmq_pmt_payload push tcp://127.0.0.1:5558
zmq_pmt_payload pull tcp://127.0.0.1:5558
```

Tagged GR3 stream through GR4 and back to GR3:

```bash
zmq_loopback float tcp://127.0.0.1:5555 tcp://127.0.0.1:5556
```

In another shell with GNU Radio 3 on `PYTHONPATH`:

```bash
python3 blocks/zeromq/examples/gr3_zmq_tag_loopback.py
```

The GR3 flowgraph sends float samples with multiple PMT tags, including two tags
at offset zero, through `zeromq.push_sink(pass_tags=True)`. GR4 receives them
with `ZmqPullSource<float>`, forwards the stream and tags through
`ZmqPushSink<float>`, and GR3 receives them with
`zeromq.pull_source(pass_tags=True)`.

## GNU Radio 3 Interop Recipes

GR3 raw push into GR4 pull:

```python
from gnuradio import blocks, gr, zeromq

tb = gr.top_block()
src = blocks.vector_source_f([1.0, 2.0, 3.0, 4.0], True)
head = blocks.head(gr.sizeof_float, 16)
sink = zeromq.push_sink(gr.sizeof_float, 1, "tcp://127.0.0.1:5555", 100, False, -1, True)
tb.connect(src, head, sink)
tb.run()
```

Run the GR4 side with:

```bash
zmq_push_pull pull tcp://127.0.0.1:5555
```

GR3-format GR4 PMT push into GR3 message pull:

```python
from gnuradio import blocks, gr, zeromq
import time

tb = gr.top_block()
src = zeromq.pull_msg_source("tcp://127.0.0.1:5558", 100, False)
dbg = blocks.message_debug()
tb.msg_connect(src, "out", dbg, "print")
tb.start()
time.sleep(2.0)
tb.stop()
tb.wait()
```

Run the GR4 side with:

```bash
zmq_pmt_payload push tcp://127.0.0.1:5558
```

`qa_ZmqInterop` runs real GR3 peers, including GR4 REQ → GR3 REP with
`REQ_CORRELATE` and `REQ_RELAXED` enabled. It returns CTest's explicit skip code
77 only when optional GR3 modules are missing. Unset `GR4_REQUIRE_GR3_INTEROP`
for optional execution: even setting it to `0` requires GR3. Unexpected probe
errors and interoperability failures fail the test. Native CI keeps the suite
optional except for the Ubuntu 24.04 GCC job, which installs GR3 and requires it.
