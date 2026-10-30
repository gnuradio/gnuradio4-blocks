#!/usr/bin/env python3
"""Writes the graph files under blocks/digital/test/graphs.

The files test digital blocks through rungraph. CMakeLists.txt beside this script
registers them, and neither the build nor ctest runs this script. The lists in the files
stay in the repository; rerun the script after a change to a block's settings, ports or
record layout, and commit the files it writes:

    python3 blocks/digital/test/generate_graphs.py [--output-dir <directory>]

The output directory defaults to graphs/ beside this script. The script uses the
standard library only and runs no block.

Where the lists come from:

- Payload bytes and bits are draws of the xorshift generator the C++ QA files use
  (shifts 13, 7 and 17, seed 0x9e3779b97f4a7c15), reproduced in Rng.
- Every expected list is an oracle's, computed here: a bit-serial CRC in the Rocksoft
  model (crc), checked against the catalog check values of "123456789" before anything
  is written; HDLC bit stuffing (stuff); least significant bit first unpacking
  (bits_lsb_first); the SLIP and PPP escape counts; the frames and sequence numbers the
  framing rules give. No list is a block's captured output.
- The short wires, the delimiters, the slicer inputs and the settings rows are the values
  the C++ cases used.
"""

import argparse
import json
import os
import re
import struct
import textwrap

M64 = (1 << 64) - 1
WIDTH = 116


class U32(int):
    pass


class U64(int):
    pass


class U32s(list):
    pass


class F32s(list):
    pass


def f32(x):
    return struct.unpack("f", struct.pack("f", x))[0]


def fmt_float(x):
    x = f32(x)
    for p in range(1, 10):
        s = "%.*g" % (p, x)
        if f32(float(s)) == x:
            break
    if not any(c in s for c in ".en"):
        s += ".0"
    return s


def fmt_scalar(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, U32):
        return "!!uint32 %d" % v
    if isinstance(v, U64):
        return "!!uint64 %d" % v
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        return "!!float32 " + fmt_float(v)
    if isinstance(v, str):
        if re.fullmatch(r"[A-Za-z][A-Za-z0-9_ ]*", v) and v not in (
            "true",
            "false",
            "null",
            "yes",
            "no",
            "on",
            "off",
        ):
            return v
        return json.dumps(v)
    raise TypeError(v)


def fmt_key(k):
    k = str(k)
    if re.fullmatch(r"[0-9]+|[A-Za-z_][A-Za-z0-9_]*", k):
        return k
    return json.dumps(k)


class Doc:
    """One output document, as a list of lines."""

    def __init__(self):
        self.lines = []

    def add(self, line):
        self.lines.append(line)

    def flow(self, indent, key, values, tag=None, fmt=None):
        fmt = fmt or (fmt_float if isinstance(values, F32s) else str)
        items = [fmt(v) for v in values]
        head = " " * indent + key + ": " + (tag + " " if tag else "")
        one = head + "[" + ", ".join(items) + "]"
        if len(one) <= 80:
            self.add(one)
            return
        pad = " " * (indent + 2)
        rows, cur = [], ""
        for it in items:
            if cur and len(pad) + len(cur) + len(it) + 1 > WIDTH:
                rows.append(cur.rstrip())
                cur = ""
            cur += it + ", "
        rows.append(cur.rstrip().rstrip(","))
        self.add(head + "[")
        for r in rows:
            self.add(pad + r)
        self.add(" " * indent + "]")

    def value(self, indent, key, v):
        key = fmt_key(key)
        if isinstance(v, U32s):
            self.flow(indent, key, v, "!!uint32")
        elif isinstance(v, F32s):
            self.flow(indent, key, v, "!!float32")
        elif isinstance(v, list) and v and all(isinstance(x, (dict, list)) for x in v):
            self.add(" " * indent + key + ":")
            for x in v:
                self.item(indent + 2, x)
        elif isinstance(v, list):
            if v and all(isinstance(x, str) for x in v):
                self.add(
                    " " * indent
                    + key
                    + ": ["
                    + ", ".join(fmt_scalar(x) for x in v)
                    + "]"
                )
            else:
                self.flow(indent, key, v)
        elif isinstance(v, dict):
            if not v:
                self.add(" " * indent + key + ": {}")
            else:
                self.add(" " * indent + key + ":")
                for k2, v2 in v.items():
                    self.value(indent + 2, k2, v2)
        else:
            self.add(" " * indent + key + ": " + fmt_scalar(v))

    def item(self, indent, v):
        """One entry of a block sequence at `indent`."""
        if isinstance(v, dict) and not v:
            self.add(" " * indent + "- {}")
            return
        if not isinstance(v, dict):
            raise TypeError(v)
        start = len(self.lines)
        for k, x in v.items():
            self.value(indent + 2, k, x)
        first = self.lines[start]
        self.lines[start] = first[:indent] + "- " + first[indent + 2 :]


class Graph:
    def __init__(self, file, comment):
        self.file = file
        self.comment = comment
        self.blocks = []
        self.edges = []

    def block(self, bid, name, params=None):
        assert all(b[1] != name for b in self.blocks), name
        self.blocks.append((bid, name, dict(params or {})))
        return name

    def connect(self, a, b, out="out", inp="in"):
        self.edges.append((a, out, b, inp))

    def records_source(self, name, kind, records):
        return self.block(
            f"gr::blocks::basic::DataSetSource<{kind}>", name, {"records": records}
        )

    def records_sink(self, name, kind, records, **params):
        return self.block(
            f"gr::blocks::testing::ExpectDataSetSink<{kind}>",
            name,
            {**params, "records": records},
        )

    def vector_source(self, name, kind, values, **params):
        values = F32s(values) if kind == "float32" else list(values)
        return self.block(
            f"gr::blocks::basic::VectorSource<{kind}>",
            name,
            {"values": values, **params},
        )

    def expect_sink(self, name, kind, expected, **params):
        expected = F32s(expected) if kind == "float32" else list(expected)
        return self.block(
            f"gr::blocks::testing::ExpectSink<{kind}>",
            name,
            {"expected": expected, **params},
        )

    def text(self):
        doc = Doc()
        text = " ".join(self.comment.split()) + " generate_graphs.py writes this file."
        for line in textwrap.wrap(text, WIDTH):
            doc.add("# " + line)
        lines = doc.lines + self.body().lines
        # the graph files are left out of the restylers, whitespace included
        assert not any(line != line.rstrip() for line in lines), self.file
        return "\n".join(lines) + "\n"

    def body(self):
        body = Doc()
        body.add("blocks:")
        for bid, name, params in self.blocks:
            body.add("  - id: " + bid)
            body.add("    parameters:")
            body.add("      name: " + name)
            for key, v in params.items():
                if key == "records":
                    if not v:
                        body.add("      records: {}")
                        continue
                    body.add("      records:")
                    for i, r in enumerate(v):
                        if not r:
                            body.add(f"        {i}: {{}}")
                            continue
                        body.add(f"        {i}:")
                        for k2, v2 in r.items():
                            body.value(10, k2, v2)
                else:
                    body.value(6, key, v)
        body.add("connections:")
        for a, out, b, inp in self.edges:
            body.add(f"  - [{a}, {out}, {b}, {inp}]")
        return body


def record(
    values=None,
    soft=False,
    names=None,
    meta=None,
    extents=None,
    absent=None,
    present=None,
    **fields,
):
    """A record of a source or a sink with the fields given, in the order the sink checks them."""
    r = {}
    if values is not None:
        r["signal_values"] = F32s(values) if soft else list(values)
    if names is not None:
        r["signal_names"] = names
    for key in ("signal_quantities", "signal_units", "axis_names"):
        if key in fields:
            r[key] = fields.pop(key)
    if extents is not None:
        r["extents"] = extents
    if "timestamp" in fields:
        r["timestamp"] = fields.pop("timestamp")
    if meta is not None:
        r["meta_information"] = meta
    if "timing_events" in fields:
        r["timing_events"] = fields.pop("timing_events")
    if absent:
        r["meta_absent"] = absent
    if present:
        r["meta_present"] = present
    assert not fields, fields
    return r


# ---------------------------------------------------------------------------------------------------------------------
# oracles


class Rng:
    """The qa files' xorshift64 generator, seeded as they seed it."""

    def __init__(self, state=0x9E3779B97F4A7C15):
        self.state = state

    def next(self):
        x = self.state
        x ^= (x << 13) & M64
        x ^= x >> 7
        x ^= (x << 17) & M64
        self.state = x
        return x

    def below(self, bound):
        return self.next() % bound

    def bytes(self, n):
        return [self.next() & 0xFF for _ in range(n)]


def reflect(value, width):
    return int(format(value, "0%db" % width)[::-1], 2)


def crc(data, width, poly, init, xorout, refin, refout):
    """The Rocksoft model, bit by bit, MSB-first register."""
    mask, top, reg = (1 << width) - 1, 1 << (width - 1), init & ((1 << width) - 1)
    for byte in data:
        if refin:
            byte = reflect(byte, 8)
        reg ^= byte << (width - 8)
        for _ in range(8):
            reg = ((reg << 1) ^ poly) & mask if reg & top else (reg << 1) & mask
    if refout:
        reg = reflect(reg, width)
    return (reg ^ xorout) & mask


def crc_bytes(value, width, order):
    n = width // 8
    big = [(value >> (8 * (n - 1 - i))) & 0xFF for i in range(n)]
    return big if order == "big" else big[::-1]


def items_of(bits):
    return [1 if c == "1" else 0 for c in bits]


def stuff(bits, k=5):
    out, ones = [], 0
    for b in bits:
        out.append(b)
        if b:
            ones += 1
            if ones == k:
                out.append(0)
                ones = 0
        else:
            ones = 0
    return out


def bits_lsb_first(data):
    return [(byte >> bit) & 1 for byte in data for bit in range(8)]


FLAG = "01111110"


def hdlc_wire(payloads):
    wire = items_of(FLAG)
    for p in payloads:
        wire += stuff(p) + items_of(FLAG)
    return wire


def flip(values, bit):
    out = list(values)
    out[bit // 8] ^= 1 << (bit % 8)
    return out


DIG = "gr::blocks::digital::"

# name, width, poly, init, xorout, refin, refout, catalog check value of "123456789" or None
SETS = [
    ("crc32", 32, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, True, True, 0xCBF43926),
    ("crc16", 16, 0x1021, 0xFFFF, 0x0000, False, False, 0x29B1),
    ("crc8", 8, 0x07, 0xFF, 0x00, False, False, None),
    (
        "crc64",
        64,
        0x42F0E1EBA9EA3693,
        (1 << 64) - 1,
        (1 << 64) - 1,
        True,
        True,
        0x995DC9BBDF1939FA,
    ),
]
X25 = ("x25", 16, 0x1021, 0xFFFF, 0xFFFF, True, True, 0x906E)
CHECK = [ord(c) for c in "123456789"]
for s in SETS + [X25]:
    if s[7] is not None:
        assert crc(CHECK, *s[1:7]) == s[7], s[0]


def crc_of(s, data):
    return crc(data, *s[1:7])


def crc_settings(s, order, skip=None, **more):
    p = {
        "width": U32(s[1]),
        "poly": U64(s[2]),
        "initial_value": U64(s[3]),
        "final_xor": U64(s[4]),
        "input_reflected": s[5],
        "result_reflected": s[6],
        "crc_byte_order": order,
    }
    if skip is not None:
        p["skip_header_bytes"] = U32(skip)
    p.update(more)
    return p


def protect(s, order, data, skip=0):
    return list(data) + crc_bytes(crc_of(s, data[skip:]), s[1], order)


def append_chain(g, prefix, inputs, settings, expected, name=None, **sink):
    src = g.records_source(prefix + "_source", "uint8", inputs)
    blk = g.block(DIG + "CrcAppend", name or prefix + "_append", settings)
    g.connect(src, blk)
    g.connect(blk, g.records_sink(prefix + "_sink", "uint8", expected, **sink))
    return blk


def check_chain(g, prefix, inputs, settings, ok, fail, source=None, name=None):
    """A source, CrcCheck and a sink on each port; `fail` None leaves the fail port unconnected."""
    if source is None:
        source = g.records_source(prefix + "_source", "uint8", inputs)
    blk = g.block(DIG + "CrcCheck", name or prefix + "_check", settings)
    g.connect(source, blk)
    g.connect(blk, g.records_sink(prefix + "_ok", "uint8", ok), "ok")
    if fail is not None:
        g.connect(blk, g.records_sink(prefix + "_fail", "uint8", fail), "fail")
    return blk


# ---------------------------------------------------------------------------------------------------------------------
# CrcAppend and CrcCheck


def crc_round_trip():
    g = Graph(
        "crc-round-trip.yaml",
        """
CrcAppend and CrcCheck in one chain, for each of four parameter sets and both byte orders: CRC-32/ISO-HDLC,
CRC-16/IBM-3740, the CRC-8 of polynomial 0x07 and seed 0xFF, and CRC-64/XZ. Each source sends records of 1, 2 and 9
bytes: as long as the 8-bit and the 16-bit CRC, and longer than the 64-bit one. The passing sink expects each record with its CRC appended, crc_ok true and the CRC it received,
and the failing sink expects no record.
""",
    )
    rng = Rng()
    for s in SETS:
        for order in ("big", "little"):
            payloads = [rng.bytes(n) for n in (1, 2, 9)]
            src = g.records_source(
                f"{s[0]}_{order}_source", "uint8", [record(p) for p in payloads]
            )
            app = g.block(
                DIG + "CrcAppend", f"{s[0]}_{order}_append", crc_settings(s, order)
            )
            g.connect(src, app)
            ok = [
                record(
                    protect(s, order, p),
                    meta={"crc_ok": True, "crc_value": U64(crc_of(s, p))},
                )
                for p in payloads
            ]
            check_chain(
                g, f"{s[0]}_{order}", None, crc_settings(s, order), ok, [], source=app
            )
    return g


def crc_append():
    g = Graph(
        "crc-append.yaml",
        """
CrcAppend alone, for each parameter set of crc-round-trip.yaml and both byte orders. The first record is the
catalog's check input "123456789", whose CRC is the catalog's check value: 0xCBF43926 for CRC-32/ISO-HDLC, 0x29B1
for CRC-16/IBM-3740 and 0x995DC9BBDF1939FA for CRC-64/XZ. The second is 13 bytes. Each sink expects the bytes with
the CRC appended in the byte order set, the extent of the longer record, crc_value and crc_width, and no crc_ok.
""",
    )
    rng = Rng()
    for s in SETS:
        long = rng.bytes(13)
        for order in ("big", "little"):
            expected = []
            for p in (CHECK, long):
                expected.append(
                    record(
                        protect(s, order, p),
                        extents=[len(p) + s[1] // 8],
                        meta={"crc_value": U64(crc_of(s, p)), "crc_width": U64(s[1])},
                        absent=["crc_ok"],
                    )
                )
            append_chain(
                g,
                f"{s[0]}_{order}",
                [record(CHECK), record(long)],
                crc_settings(s, order),
                expected,
            )
    return g


def crc_corruption():
    g = Graph(
        "crc-corruption.yaml",
        """
CrcCheck under each parameter set of crc-round-trip.yaml, fed every single-bit corruption of one protected record:
two payload bytes and the CRC appended big-endian, one record per bit of the whole record, in bit order. The
failing sink expects every record with crc_ok false, and the passing sink expects none.
""",
    )
    rng = Rng()
    for s in SETS:
        good = protect(s, "big", rng.bytes(2))
        inputs = [record(flip(good, bit)) for bit in range(8 * len(good))]
        fail = [record(meta={"crc_ok": False}) for _ in inputs]
        check_chain(g, s[0], inputs, crc_settings(s, "big"), [], fail)
    return g


def crc_guard():
    g = Graph(
        "crc-guard.yaml",
        """
The length guards, for each parameter set of crc-round-trip.yaml with skip_header_bytes 3. CrcCheck receives a
record of every length up to 3 bytes plus the CRC width, which it drops, and then one byte more, which it routes:
its failing sink expects that last record alone. CrcAppend receives records of 0 to 3 bytes, which it drops, and
then a record of 4 bytes, which its sink expects alone with the CRC of its last byte. A last CrcCheck has
skip_header_bytes at the largest value it holds and drops a record of 64 bytes, and both its sinks expect none.
""",
    )
    for s in SETS:
        shortest = 3 + s[1] // 8
        inputs = [record([0x5A] * n) for n in range(shortest + 1)] + [
            record([0x11] * (shortest + 1))
        ]
        check_chain(
            g,
            f"{s[0]}_check",
            inputs,
            crc_settings(s, "big", 3),
            [],
            [record([0x11] * (shortest + 1), meta={"crc_ok": False})],
        )
        inputs = [record([0xA5] * n) for n in range(4)] + [record([0xA5] * 4)]
        append_chain(
            g,
            f"{s[0]}_append",
            inputs,
            crc_settings(s, "big", 3),
            [record(protect(s, "big", [0xA5] * 4, 3))],
        )
    check_chain(
        g,
        "maximum",
        [record([0x11] * 64)],
        crc_settings(SETS[0], "big", 0xFFFFFFFF),
        [],
        [],
    )
    return g


def crc_skip():
    g = Graph(
        "crc-skip.yaml",
        """
skip_header_bytes 5 on both blocks, for each parameter set of crc-round-trip.yaml. A record of 40 bytes passes
CrcAppend and then CrcCheck with discard_crc. The passing sink expects the 40 bytes as they were sent, their extent,
and the CRC of the last 35 bytes, which is the value CrcAppend appended and CrcCheck received.
""",
    )
    rng = Rng()
    for s in SETS:
        whole = rng.bytes(40)
        src = g.records_source(f"{s[0]}_source", "uint8", [record(whole)])
        app = g.block(DIG + "CrcAppend", f"{s[0]}_append", crc_settings(s, "big", 5))
        g.connect(src, app)
        ok = [
            record(
                whole,
                extents=[40],
                meta={"crc_ok": True, "crc_value": U64(crc_of(s, whole[5:]))},
            )
        ]
        check_chain(
            g,
            s[0],
            None,
            crc_settings(s, "big", 5, discard_crc=True),
            ok,
            [],
            source=app,
        )
    return g


def crc_records():
    g = Graph(
        "crc-records.yaml",
        """
CrcAppend and CrcCheck with a record's other fields, under CRC-32/ISO-HDLC. The discard chain sends a protected
record of 24 bytes and a copy with its first bit changed to CrcCheck with discard_crc: each sink expects its record
stripped to 24 bytes. The carried chain sends a record with a timestamp, signal fields, a trigger_name and a timing
event at index 3 to CrcAppend, and its sink expects them all beside crc_value and crc_width, and no crc_ok. The
verdict chain sends that protected record and a copy with bit 63 changed to CrcCheck: the passing sink expects
crc_ok true, the received CRC, the trigger_name and the timestamp, and the failing sink crc_ok false, a crc_value
and the trigger_name. The unconnected chain leaves the fail port unconnected and sends a damaged record, a good one
and the damaged one again: the passing sink expects the good record alone.
""",
    )
    s = SETS[0]
    rng = Rng()
    payload = rng.bytes(24)
    good = protect(s, "big", payload)
    bad = flip(good, 0)
    ok = [record(payload, extents=[24], meta={"crc_ok": True})]
    fail = [record(bad[:24], extents=[24], meta={"crc_ok": False})]
    check_chain(
        g,
        "discard",
        [record(good), record(bad)],
        crc_settings(s, "big", discard_crc=True),
        ok,
        fail,
    )

    payload = rng.bytes(50)
    fields = dict(
        names=["payload"],
        signal_quantities=["bytes"],
        signal_units=[""],
        timestamp=987654321,
        timing_events=[{3: {}}],
    )
    value = U64(crc_of(s, payload))
    original = record(payload, meta={"trigger_name": "access_code"}, **fields)
    expected = record(
        protect(s, "big", payload),
        axis_names=[],
        meta={"trigger_name": "access_code", "crc_value": value, "crc_width": U64(32)},
        absent=["crc_ok"],
        **fields,
    )
    append_chain(g, "carried", [original], crc_settings(s, "big"), [expected])

    appended = protect(s, "big", payload)
    inputs = [
        record(appended, meta={"trigger_name": "access_code"}, **fields),
        record(flip(appended, 63), meta={"trigger_name": "access_code"}, **fields),
    ]
    ok = [
        record(
            meta={"crc_ok": True, "crc_value": value, "trigger_name": "access_code"},
            timestamp=987654321,
        )
    ]
    fail = [
        record(
            meta={"crc_ok": False, "trigger_name": "access_code"}, present=["crc_value"]
        )
    ]
    check_chain(g, "verdict", inputs, crc_settings(s, "big"), ok, fail)

    payload = rng.bytes(16)
    good = protect(s, "big", payload)
    bad = flip(good, 22)
    check_chain(
        g,
        "unconnected",
        [record(bad), record(good), record(bad)],
        crc_settings(s, "big"),
        [record(good)],
        None,
    )
    return g


def crc_settings_graph():
    g = Graph(
        "crc-settings.yaml",
        """
CrcAppend and CrcCheck at their default settings, CRC-32/ISO-HDLC, each with no record to process. The run passes
as written. A case that changes settings through SET either runs to the end with settings the block accepts or ends
with the block's reason for refusing them.
""",
    )
    append_chain(g, "append", [], {}, [], name="append_block")
    check_chain(g, "check", [], {}, [], [], name="check_block")
    return g


def crc_multi_signal():
    g = Graph(
        "crc-multi-signal.yaml",
        """
CrcAppend fed a record of 8 bytes that names two signals. The block refuses to read two signals as one byte
array, and the refusal ends the run with the block's reason. The sink expects the record a single signal of
those bytes would become. A block that stopped refusing would then run to the end, and the exit case of the pair
would fail.
""",
    )
    expected = [record(protect(SETS[0], "big", [0x33] * 8))]
    append_chain(
        g,
        "multi",
        [record([0x33] * 8, names=["payload", "second"])],
        crc_settings(SETS[0], "big"),
        expected,
    )
    return g


# ---------------------------------------------------------------------------------------------------------------------
# DelimiterExtractor


def byte_framing(bound, floor=None):
    p = {
        "end_delimiter": "11000000",
        "bits_per_item": U32(8),
        "max_payload_items": U32(bound),
    }
    if floor is not None:
        p["min_payload_items"] = U32(floor)
    return p


def hdlc_framing(bound, **more):
    return {
        "end_delimiter": FLAG,
        "transparency": "bit_stuffing",
        "max_payload_items": U32(bound),
        **more,
    }


def extractor_chain(
    g,
    prefix,
    stream,
    settings,
    out,
    reject=None,
    kind="uint8",
    source_params=None,
    stages=(),
):
    """A stream source, any stages, DelimiterExtractor and a sink on `out`; `reject` None leaves that port open."""
    prev = g.vector_source(prefix + "_source", kind, stream, **(source_params or {}))
    for bid, suffix, params in stages:
        name = g.block(bid, prefix + "_" + suffix, params)
        g.connect(prev, name)
        prev = name
    ext = g.block(f"{DIG}DelimiterExtractor<{kind}>", prefix + "_extractor", settings)
    g.connect(prev, ext)
    g.connect(ext, g.records_sink(prefix + "_out", kind, out))
    if reject is not None:
        g.connect(ext, g.records_sink(prefix + "_reject", kind, reject), "reject")
    return ext


# anchor A: the flag, 0x3F with one stuffed zero, the flag
ANCHOR_A = FLAG + "111110100" + FLAG


def extractor_records():
    g = Graph(
        "delimiter-extractor-records.yaml",
        """
DelimiterExtractor over HDLC framing, bit stuffing at five ones and a bound of 1024 items. The two-frame chain
carries 0x3F with one stuffed zero and then 0x00 with none. Its sink expects both records with their items, start
index, stuffing count, sequence number, label, signal name and extent, and its reject sink expects none. The soft
chain carries the first frame as floats of 0.75 and -0.25, and its sink expects those values rather than the bits
they slice to. The pack chain assembles the first frame least significant bit first, and its sink expects the one
byte 0x3F, its extent and the one stuffed zero.
""",
    )
    stream = items_of(FLAG + "111110100" + FLAG + "00000000" + FLAG)
    common = {"names": ["payload"], "extents": [8]}
    out = [
        record(
            [1, 1, 1, 1, 1, 1, 0, 0],
            meta={
                "sample_start": U64(8),
                "stuffing_removed": U32(1),
                "sequence": U64(0),
                "trigger_name": "delimiter",
            },
            **common,
        ),
        record(
            [0] * 8,
            meta={
                "sample_start": U64(25),
                "stuffing_removed": U32(0),
                "sequence": U64(1),
            },
            **common,
        ),
    ]
    extractor_chain(g, "two_frames", stream, hdlc_framing(1024), out, [])
    soft = [0.75 if b else -0.25 for b in items_of(ANCHOR_A)]
    extractor_chain(
        g,
        "soft",
        soft,
        hdlc_framing(1024),
        [record([0.75] * 6 + [-0.25] * 2, soft=True)],
        kind="float32",
    )
    pack = hdlc_framing(1024, payload_pack_bits=U32(8), payload_bit_order="LsbFirst")
    extractor_chain(
        g,
        "pack",
        items_of(ANCHOR_A),
        pack,
        [record([0x3F], extents=[1], meta={"stuffing_removed": U32(1)})],
    )
    return g


def extractor_refusals():
    g = Graph(
        "delimiter-extractor-refusals.yaml",
        """
The frames DelimiterExtractor refuses and the regions it does not count as frames, each followed by a frame it
emits. Overrun: 0xC0 delimiters over byte items with a bound of four, a five-item region and then a three-item frame;
the reject sink expects the first four items with the reason overrun. Survive: a hundred such regions in a row
and then a two-item frame, with the reject port unconnected. Undersize: a floor of three and a two-item region,
refused with the reason undersize. Idle: two delimiters back to back before a three-item frame, which keeps sequence
number 0. Unaligned: twenty bits between flags with the pack stage at eight bits, refused unpacked with the reason
unaligned. Each block's line at stop counts what it refused.
""",
    )
    stream = [0xC0, 0x11, 0x22, 0x33, 0x44, 0x55, 0xC0, 0x66, 0x77, 0x88, 0xC0]
    reject = [record([0x11, 0x22, 0x33, 0x44], meta={"discard_reason": "overrun"})]
    extractor_chain(
        g, "overrun", stream, byte_framing(4), [record([0x66, 0x77, 0x88])], reject
    )
    hostile = []
    for _ in range(SURVIVE):
        hostile += [0xC0, 1, 2, 3, 4, 5]
    hostile += [0xC0, 0x99, 0xAA, 0xC0]
    extractor_chain(g, "survive", hostile, byte_framing(4), [record([0x99, 0xAA])])
    reject = [record([0x11, 0x22], meta={"discard_reason": "undersize"})]
    extractor_chain(
        g, "undersize", [0xC0, 0x11, 0x22, 0xC0], byte_framing(16, 3), [], reject
    )
    out = [record([0x11, 0x22, 0x33], meta={"sequence": U64(0)})]
    extractor_chain(
        g, "idle", [0xC0, 0xC0, 0x11, 0x22, 0x33, 0xC0], byte_framing(16, 3), out, []
    )
    payload = "10101010101010101010"
    settings = {
        "end_delimiter": FLAG,
        "max_payload_items": U32(1024),
        "payload_pack_bits": U32(8),
    }
    reject = [record(items_of(payload), meta={"discard_reason": "unaligned"})]
    extractor_chain(
        g, "unaligned", items_of(FLAG + payload + FLAG), settings, [], reject
    )
    return g


SURVIVE = 100


def extractor_recovery():
    g = Graph(
        "delimiter-extractor-recovery.yaml",
        """
DelimiterExtractor abandoning a frame and recovering at the next delimiter. Abort: over HDLC framing a frame broken
by seven ones and then the frame 01100; the sink expects the second frame alone and the reject sink nothing. Patient:
the same wire with abort_ones 0, where the seven ones are payload, and the sink expects two records. Restart: '$'
opens and a line feed closes over byte items, and a second '$' arrives inside the first frame; the sink expects the
frame the second '$' opened. Each block's line at stop counts the aborts and restarts.
""",
    )
    stream = items_of(FLAG + "10100" + "1111111" + FLAG + "01100" + FLAG)
    extractor_chain(
        g, "abort", stream, hdlc_framing(1024), [record([0, 1, 1, 0, 0])], []
    )
    extractor_chain(
        g, "patient", stream, hdlc_framing(1024, abort_ones=U32(0)), [{}, {}], []
    )
    settings = {
        "end_delimiter": "00001010",
        "start_delimiter": "00100100",
        "frame_open": "start",
        "bits_per_item": U32(8),
        "max_payload_items": U32(64),
    }
    extractor_chain(
        g,
        "restart",
        [0x24, 0x41, 0x42, 0x24, 0x43, 0x44, 0x0A],
        settings,
        [record([0x43, 0x44])],
        [],
    )
    return g


SYNC = format(0xACDDA4E2F28C20AC, "064b")
PAYLOADS = [items_of("10110010"), items_of("01001101")]


def extractor_triggers():
    g = Graph(
        "delimiter-extractor-triggers.yaml",
        """
Trigger tags and DelimiterExtractor over HDLC framing. Detector: a 64-bit sync word ahead of two frames, through
AccessCodeCorrelator into the extractor. The sink expects the detector's label and distance on the first record
alone, and the second record labeled by the extractor. Mid-frame: a trigger tag four items into the first of two
frames, which the extractor abandons, and its sink expects the second frame alone. Passive: the same tag with
trigger_resets false, and its sink expects both frames. Each resetting block's line at stop counts one reset.
""",
    )
    wire = hdlc_wire(PAYLOADS)
    stages = [(DIG + "AccessCodeCorrelator<uint8>", "detector", {"access_code": SYNC})]
    out = [
        record(
            PAYLOADS[0], meta={"trigger_name": "access_code", "sync_errors": U32(0)}
        ),
        record(PAYLOADS[1], meta={"trigger_name": "delimiter"}, absent=["sync_errors"]),
    ]
    extractor_chain(
        g, "detector", items_of(SYNC) + wire, hdlc_framing(1024), out, stages=stages
    )
    tags = {"tags": {12: {"gr:trigger_name": "access_code"}}}
    extractor_chain(
        g,
        "mid_frame",
        wire,
        hdlc_framing(1024),
        [record(PAYLOADS[1])],
        source_params=tags,
    )
    passive = hdlc_framing(1024, trigger_resets=False)
    extractor_chain(
        g,
        "passive",
        wire,
        passive,
        [record(PAYLOADS[0]), record(PAYLOADS[1])],
        source_params=tags,
    )
    return g


def extractor_sequence():
    g = Graph(
        "delimiter-extractor-sequence.yaml",
        """
DelimiterExtractor over 0xC0 delimiters and byte items with a bound of 8 and a floor of 2, fed 100 regions: frames
of four items, seven regions of twelve items at frames 3, 16, 29, 42, 55, 68 and 81, and three single items at
frames 7, 40 and 73. The sink expects the 90 frames numbered 0 to 99 without the ten refused ones, and the reject
sink expects those ten, in order, with their reasons.
""",
    )
    rng = Rng()
    undersize, overrun = (7, 40, 73), (3, 16, 29, 42, 55, 68, 81)
    stream, out, reject = [0xC0], [], []
    for which in range(100):
        length = 1 if which in undersize else 12 if which in overrun else 4
        stream += [1 + rng.next() % 0x7E for _ in range(length)] + [0xC0]
        if which in undersize or which in overrun:
            reason = "undersize" if which in undersize else "overrun"
            reject.append(
                record(meta={"sequence": U64(which), "discard_reason": reason})
            )
        else:
            out.append(record(meta={"sequence": U64(which)}))
    extractor_chain(g, "sequence", stream, byte_framing(8, 2), out, reject)
    return g


def extractor_events():
    g = Graph(
        "delimiter-extractor-events.yaml",
        """
Input tags as the timing events of DelimiterExtractor's records, over HDLC framing. Interior: anchor A, 0x3F with
one stuffed zero, with a tag before the first flag and a tag on each of the nine raw payload items. The sink expects
one event per payload item at its index in the decoded record, and two at index 5, where the tag on the stuffed bit
moves to the next item. Edge: a frame of five ones whose last raw item is a stuffed zero carrying a tag, which falls
one past the record's end, and the sink expects the record with no event. Each block's line at stop counts the tags
outside a frame, on a removed item and dropped.
""",
    )
    tags = {0: {"marker": U32(99)}}
    for at in range(8, 17):
        tags[at] = {"marker": U32(at)}
    events = {}
    for at, index in zip(range(8, 17), (0, 1, 2, 3, 4, 5, 5, 6, 7)):
        events.setdefault(index, []).append({"marker": U32(at)})
    events = {k: (v[0] if len(v) == 1 else v) for k, v in events.items()}
    out = [record(timing_events=[events])]
    extractor_chain(
        g,
        "interior",
        items_of(ANCHOR_A),
        hdlc_framing(1024),
        out,
        source_params={"tags": tags},
    )
    trailing = items_of(FLAG + "111110" + FLAG)
    out = [record([1] * 5, timing_events=[{}])]
    extractor_chain(
        g,
        "edge",
        trailing,
        hdlc_framing(1024),
        out,
        source_params={"tags": {13: {"marker": U32(1)}}},
    )
    return g


STUFFED_FRAMES = 44


def extractor_stuffing():
    g = Graph(
        "delimiter-extractor-stuffing.yaml",
        f"""
DelimiterExtractor over HDLC framing with a floor of five items, fed {STUFFED_FRAMES} stuffed frames of 1 to 64 random bits
between shared flags. Each record's extent plus its stuffing count is the raw region between its two flags. The
sink expects each emitted frame's sequence number, extent and stuffing count, and the reject sink the same for the
frames shorter than five bits.
""",
    )
    rng = Rng()
    wire, out, reject = items_of(FLAG), [], []
    for which in range(STUFFED_FRAMES):
        length = 1 + rng.below(64)
        payload = [rng.next() & 1 for _ in range(length)]
        coded = stuff(payload)
        wire += coded + items_of(FLAG)
        r = record(
            extents=[length],
            meta={"sequence": U64(which), "stuffing_removed": U32(len(coded) - length)},
        )
        (reject if length < 5 else out).append(r)
    assert len(reject) == 4, len(reject)
    extractor_chain(
        g, "identity", wire, hdlc_framing(4096, min_payload_items=U32(5)), out, reject
    )
    return g


def extractor_crc():
    g = Graph(
        "delimiter-extractor-crc.yaml",
        """
DelimiterExtractor into CrcCheck, as an HDLC receiver: three frames of three bytes, each followed by its CRC-16/X-25
frame check sequence low byte first, sent least significant bit first and stuffed between shared flags, with one
payload bit of the second frame changed. The extractor packs the bits into bytes least significant bit first and
refuses a frame below 24 bits. The passing sink expects the first and third frames with their check sequences, and
the failing sink the second.
""",
    )
    messages = [[0x41, 0x42, 0x43], [0x44, 0x45, 0x46], [0x47, 0x48, 0x49]]
    frames, sent = [], []
    for which, m in enumerate(messages):
        data = m + crc_bytes(crc_of(X25, m), 16, "little")
        sent.append(data)
        bits = bits_lsb_first(data)
        if which == 1:
            bits[3] ^= 1
        frames.append(bits)
    extractor = hdlc_framing(
        1024,
        payload_pack_bits=U32(8),
        payload_bit_order="LsbFirst",
        min_payload_items=U32(24),
    )
    src = g.vector_source("fcs_source", "uint8", hdlc_wire(frames))
    ext = g.block(DIG + "DelimiterExtractor<uint8>", "fcs_extractor", extractor)
    g.connect(src, ext)
    ok = [
        record(sent[0], meta={"crc_ok": True}),
        record(sent[2], meta={"crc_ok": True}),
    ]
    check_chain(
        g,
        "fcs",
        None,
        crc_settings(X25, "little"),
        ok,
        [record(meta={"crc_ok": False})],
        source=ext,
    )
    return g


def extractor_settings():
    g = Graph(
        "delimiter-extractor-settings.yaml",
        """
DelimiterExtractor with settings it accepts and no item to process: 0xC0 over byte items with a bound of 16, the
flag 01111110 over bit items with a bound of 16, and the flag over float items. The run passes as written. A case
that changes settings through SET checks that the block refuses them, and the refusal ends the run with the block's
reason.
""",
    )
    extractor_chain(g, "byte", [], byte_framing(16), [])
    extractor_chain(
        g, "bit", [], {"end_delimiter": FLAG, "max_payload_items": U32(16)}, []
    )
    extractor_chain(
        g,
        "float",
        [],
        {"end_delimiter": FLAG, "max_payload_items": U32(16)},
        [],
        kind="float32",
    )
    return g


# ---------------------------------------------------------------------------------------------------------------------
# DelimiterFramer

PPP_ORIGINALS = list(range(32)) + [0x7D, 0x7E]


def framer_hdlc(bound):
    return hdlc_framing(bound, payload_pack_bits=U32(8), payload_bit_order="LsbFirst")


def framer_slip(bound):
    return {
        "end_delimiter": "11000000",
        "bits_per_item": U32(8),
        "transparency": "byte_escape",
        "escape_item": U32(0xDB),
        "escape_map": U32s([0xDC, 0xC0, 0xDD, 0xDB]),
        "max_payload_items": U32(bound),
    }


def framer_ppp(bound):
    flat = []
    for original in PPP_ORIGINALS:
        flat += [original ^ 0x20, original]
    return {
        "end_delimiter": FLAG,
        "bits_per_item": U32(8),
        "transparency": "byte_escape",
        "escape_item": U32(0x7D),
        "escape_map": U32s(flat),
        "max_payload_items": U32(bound),
    }


def framer_nmea(bound):
    return {
        "end_delimiter": "0000110100001010",
        "start_delimiter": "00100100",
        "frame_open": "start",
        "bits_per_item": U32(8),
        "max_payload_items": U32(bound),
    }


def hdlc_frame(data):
    """One framed record under framer_hdlc, and the zeros stuffed into it."""
    bits = bits_lsb_first(data)
    coded = stuff(bits)
    return items_of(FLAG) + coded + items_of(FLAG), len(coded) - len(bits)


def payload_record(data, meta=None, **more):
    """The record the qa file's recordOf builds: one signal named payload and an empty unit and quantity."""
    return record(
        data,
        names=["payload"],
        signal_quantities=[""],
        signal_units=[""],
        meta=meta if meta is not None else {},
        **more,
    )


def framer_chain(
    g, prefix, inputs, settings, framed=None, received=None, extractor=None
):
    """A record source and DelimiterFramer, a sink on the framed records, and through DataSetToStream an extractor
    and a sink on what it reads back. `framed` or `received` None leaves that branch out.
    """
    src = g.records_source(prefix + "_source", "uint8", inputs)
    fr = g.block(DIG + "DelimiterFramer", prefix + "_framer", settings)
    g.connect(src, fr)
    if framed is not None:
        g.connect(fr, g.records_sink(prefix + "_framed", "uint8", framed))
    if received is not None:
        wire = g.block(
            "gr::blocks::basic::DataSetToStream<uint8>",
            prefix + "_wire",
            {"boundary_label": ""},
        )
        g.connect(fr, wire)
        ext = g.block(
            DIG + "DelimiterExtractor<uint8>",
            prefix + "_extractor",
            extractor or settings,
        )
        g.connect(wire, ext)
        g.connect(ext, g.records_sink(prefix + "_received", "uint8", received))
    return fr


def framer_hdlc_graph():
    g = Graph(
        "delimiter-framer-hdlc.yaml",
        """
DelimiterFramer over HDLC framing, each byte sent least significant bit first, framing the two bytes 0x7E and 0xFF.
The framed sink expects the flag, 0x7E with a zero stuffed after its fifth one, 0xFF with another, and the flag: 34
bit items and a stuffing count of 2. Through DataSetToStream a DelimiterExtractor with the same framing reads the
frame back, and its sink expects the two bytes and the two zeros it removed. The framer's line at stop counts the
record, the items and the stuffed bits.
""",
    )
    wire = items_of(FLAG + "011111010" + "111110111" + FLAG)
    assert (wire, 2) == hdlc_frame([0x7E, 0xFF])
    framed = [record(wire, extents=[34], meta={"stuffing_inserted": U32(2)})]
    received = [record([0x7E, 0xFF], meta={"stuffing_removed": U32(2)})]
    framer_chain(
        g, "hdlc", [payload_record([0x7E, 0xFF])], framer_hdlc(1024), framed, received
    )
    return g


def framer_loopback():
    g = Graph(
        "delimiter-framer-loopback.yaml",
        """
DelimiterFramer and DelimiterExtractor at the two ends of one link, under HDLC, SLIP, PPP and NMEA 0183 framing.
Each source sends payloads of 1, 2, 7, 33 and 64 random bytes, upper-case letters under NMEA, each with a packet
number. The framed sink expects each packet number and the count of items the framer inserted, and an extent of
both delimiters, the payload items and those insertions. Through DataSetToStream the extractor's sink expects
each payload back and a removed count equal to the inserted one.
""",
    )
    framings = [
        ("hdlc", framer_hdlc(8192), 8, 8, 8),
        ("slip", framer_slip(1024), 1, 1, 1),
        ("ppp", framer_ppp(1024), 1, 1, 1),
        ("nmea", framer_nmea(1024), 1, 1, 2),
    ]
    for name, settings, unit, opening, closing in framings:
        rng = Rng()
        inputs, framed, received = [], [], []
        for packet, length in enumerate((1, 2, 7, 33, 64)):
            data = [
                0x41 + rng.next() % 26 if name == "nmea" else rng.next() & 0xFF
                for _ in range(length)
            ]
            if name == "hdlc":
                inserted = hdlc_frame(data)[1]
            elif name == "slip":
                inserted = sum(1 for b in data if b in (0xC0, 0xDB))
            elif name == "ppp":
                inserted = sum(1 for b in data if b in PPP_ORIGINALS)
            else:
                inserted = 0
            inputs.append(payload_record(data, meta={"packet": U32(packet)}))
            emitted = opening + closing + unit * length + inserted
            framed.append(
                record(
                    extents=[emitted],
                    meta={"packet": U32(packet), "stuffing_inserted": U32(inserted)},
                )
            )
            received.append(record(data, meta={"stuffing_removed": U32(inserted)}))
        framer_chain(g, name, inputs, settings, framed, received)
    return g


def framer_fcs():
    rng = Rng()
    messages = [[0x41, 0x42, 0x43], [0xFF] * 24, rng.bytes(40)]
    frames = [hdlc_frame(m + crc_bytes(crc_of(X25, m), 16, "little")) for m in messages]
    g = Graph(
        "delimiter-framer-fcs.yaml",
        f"""
The HDLC profile with a CRC-16/X-25 frame check sequence, low byte first, over three messages: three bytes, 24 bytes
of 0xFF and 40 random bytes. Clean: CrcAppend, DelimiterFramer, DataSetToStream, DelimiterExtractor and CrcCheck
with discard_crc in one chain. The framed sink expects each frame item by item, the all-ones frame with its
{frames[1][1]} stuffed zeros, and the passing sink each message as it was sent. Flipped: the same three frames on one stream, with
one item of the third frame inverted inside a run of ones short enough that no stuffing decision changes. The
passing sink expects the first two messages, and the failing sink the third.
""",
    )
    crc = crc_settings(X25, "little")
    src = g.records_source(
        "clean_source", "uint8", [payload_record(m) for m in messages]
    )
    app = g.block(DIG + "CrcAppend", "clean_append", crc)
    g.connect(src, app)
    fr = g.block(DIG + "DelimiterFramer", "clean_framer", framer_hdlc(8192))
    g.connect(app, fr)
    framed = [record(f, meta={"stuffing_inserted": U32(n)}) for f, n in frames]
    assert frames[1][1] > 0
    g.connect(fr, g.records_sink("clean_framed", "uint8", framed))
    wire = g.block(
        "gr::blocks::basic::DataSetToStream<uint8>",
        "clean_wire",
        {"boundary_label": ""},
    )
    g.connect(fr, wire)
    ext = g.block(
        DIG + "DelimiterExtractor<uint8>", "clean_extractor", framer_hdlc(8192)
    )
    g.connect(wire, ext)
    ok = [record(m, meta={"crc_ok": True}) for m in messages]
    check_chain(
        g,
        "clean",
        None,
        crc_settings(X25, "little", discard_crc=True),
        ok,
        [],
        source=ext,
    )

    stream = frames[0][0] + frames[1][0] + frames[2][0]
    earlier, target, at = len(frames[0][0]) + len(frames[1][0]), frames[2][0], None
    for i in range(9, len(target) - 9):
        if target[i] == 0:
            continue
        lo = hi = i
        while lo > 0 and target[lo - 1]:
            lo -= 1
        while hi + 1 < len(target) and target[hi + 1]:
            hi += 1
        if hi - lo + 1 <= 4:
            at = earlier + i
            break
    assert at is not None
    stream[at] ^= 1
    src = g.vector_source("flipped_source", "uint8", stream)
    ext = g.block(
        DIG + "DelimiterExtractor<uint8>", "flipped_extractor", framer_hdlc(8192)
    )
    g.connect(src, ext)
    ok = [record(m, meta={"crc_ok": True}) for m in messages[:2]]
    check_chain(
        g,
        "flipped",
        None,
        crc_settings(X25, "little", discard_crc=True),
        ok,
        [record(meta={"crc_ok": False})],
        source=ext,
    )
    return g


def framer_ones():
    g = Graph(
        "delimiter-framer-ones.yaml",
        """
DelimiterFramer and DelimiterExtractor at the two ends of one link under the delimiter 0111111, which ends in ones,
with bit stuffing at five ones and no abort. The source sends 40 payloads of 1 to 64 random bits and then 011111,
which ends in five ones. The received sink expects every payload back, and the framed sink expects one zero stuffed
into the last frame, after its last item.
""",
    )
    settings = {
        "end_delimiter": "0111111",
        "transparency": "bit_stuffing",
        "abort_ones": U32(0),
        "max_payload_items": U32(4096),
    }
    rng = Rng()
    payloads = []
    for _ in range(40):
        length = 1 + rng.below(64)
        payloads.append([rng.next() & 1 for _ in range(length)])
    payloads.append([0, 1, 1, 1, 1, 1])
    framed = [{} for _ in payloads[:-1]] + [record(meta={"stuffing_inserted": U32(1)})]
    framer_chain(
        g,
        "ones",
        [payload_record(p) for p in payloads],
        settings,
        framed,
        [record(p) for p in payloads],
    )
    return g


def framer_records():
    g = Graph(
        "delimiter-framer-records.yaml",
        """
DelimiterFramer with records it drops, records that forge its delimiter, and a record's metadata. Drops: SLIP
framing with a bound of 8, fed an empty record, three bytes, nine bytes and one byte; the sink expects the second and
fourth records framed. Forgery: 0xC0 over byte items with no transparency, fed a payload carrying 0xC0 twice; the
framed sink expects it framed with nothing inserted, and the extractor's sink three records, cut at the two forged
delimiters. Metadata: SLIP framing over three records; the sink expects the first record's keys beside a stuffing
count of 0 and one empty event list where it had an event, and the second record, 0xC0 and 0xDB, with both escaped.
Each framer's line at stop counts what it framed, emitted, escaped, refused or forged.
""",
    )
    inputs = [
        payload_record([]),
        payload_record([0x11, 0x22, 0x33]),
        payload_record([0x44] * 9),
        payload_record([0x55]),
    ]
    framed = [record([0xC0, 0x11, 0x22, 0x33, 0xC0]), record([0xC0, 0x55, 0xC0])]
    framer_chain(g, "drops", inputs, framer_slip(8), framed)

    settings = {
        "end_delimiter": "11000000",
        "bits_per_item": U32(8),
        "max_payload_items": U32(64),
    }
    data = [0x11, 0xC0, 0x22, 0xC0, 0x33]
    framed = [record([0xC0] + data + [0xC0], meta={"stuffing_inserted": U32(0)})]
    received = [record([0x11]), record([0x22]), record([0x33])]
    framer_chain(g, "forgery", [payload_record(data)], settings, framed, received)

    inputs = [
        payload_record(
            [0x11, 0x22],
            meta={"packet": U32(7), "origin": "qa"},
            timing_events=[{1: {"marker": U32(1)}}],
        ),
        payload_record([0xC0, 0xDB], meta={"packet": U32(8)}),
        payload_record([0x33]),
    ]
    framed = [
        record(
            meta={"packet": U32(7), "origin": "qa", "stuffing_inserted": U32(0)},
            timing_events=[{}],
        ),
        record(
            [0xC0, 0xDB, 0xDC, 0xDB, 0xDD, 0xC0],
            meta={"packet": U32(8), "stuffing_inserted": U32(2)},
        ),
        {},
    ]
    framer_chain(g, "metadata", inputs, framer_slip(64), framed)
    return g


def framer_settings():
    g = Graph(
        "delimiter-framer-settings.yaml",
        """
DelimiterFramer with settings it accepts and no record to process: SLIP and PPP framing with a bound of 256, the
flag under bit stuffing with a bound of 16, and 0xC0 over byte items with a bound of 16. The run passes as written. A
case that changes settings through SET checks that the block refuses them, and the refusal ends the run with the
block's reason.
""",
    )
    framer_chain(g, "slip", [], framer_slip(256), [])
    framer_chain(g, "ppp", [], framer_ppp(256), [])
    framer_chain(g, "stuffed", [], hdlc_framing(16), [])
    framer_chain(g, "byte", [], byte_framing(16), [])
    return g


# ---------------------------------------------------------------------------------------------------------------------
# PamSlicer and LevelTracker

SLICER_INPUTS = [-1e30, -3.0, -2.01, -2.0, -1.0, -0.01, 0.0, 1.0, 1.99, 2.0, 3.0, 1e30]
SLICER_RANKS = [0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3]


def pam_slicer():
    g = Graph(
        "pam-slicer.yaml",
        """
PamSlicer over four levels, fed a value inside, at and on each side of every threshold at -2, 0 and +2, and the
two outer extremes. The identity sink expects each value's level rank, the upper region taking a value on its
threshold. The labeled chain maps the ranks to the C4FM labels 3, 2, 0 and 1, and its sink expects those labels.
""",
    )
    src = g.vector_source("slicer_source", "float32", SLICER_INPUTS)
    for name, labels in (("identity", None), ("c4fm", [3, 2, 0, 1])):
        params = {} if labels is None else {"labels": U32s(labels)}
        sl = g.block(DIG + "PamSlicer<float32>", name + "_slicer", params)
        g.connect(src, sl)
        expected = SLICER_RANKS if labels is None else [labels[r] for r in SLICER_RANKS]
        g.connect(sl, g.expect_sink(name + "_sink", "uint8", expected))
    return g


def pam_settings():
    g = Graph(
        "pam-settings.yaml",
        """
PamSlicer and LevelTracker over four levels, each with no symbol to process. The run passes as written. A case that
changes a setting through SET checks that the block refuses it, and the refusal ends the run with the block's
reason.
""",
    )
    for name, block, kind in (
        ("slicer", "PamSlicer<float32>", "uint8"),
        ("tracker", "LevelTracker<float32>", "float32"),
    ):
        src = g.vector_source(name + "_source", "float32", [])
        b = g.block(DIG + block, name + "_block")
        g.connect(src, b)
        g.connect(b, g.expect_sink(name + "_sink", kind, []))
    return g


GRAPHS = [
    crc_round_trip,
    crc_append,
    crc_corruption,
    crc_guard,
    crc_skip,
    crc_records,
    crc_settings_graph,
    crc_multi_signal,
    extractor_records,
    extractor_refusals,
    extractor_recovery,
    extractor_triggers,
    extractor_sequence,
    extractor_events,
    extractor_stuffing,
    extractor_crc,
    extractor_settings,
    framer_hdlc_graph,
    framer_loopback,
    framer_fcs,
    framer_ones,
    framer_records,
    framer_settings,
    pam_slicer,
    pam_settings,
]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--output-dir",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "graphs"),
        help="the directory the graph files are written to",
    )
    args = parser.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    for make in GRAPHS:
        g = make()
        with open(os.path.join(args.output_dir, g.file), "w") as f:
            f.write(g.text())


if __name__ == "__main__":
    main()
