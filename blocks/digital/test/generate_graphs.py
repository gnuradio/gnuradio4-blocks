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


GRAPHS = [
    crc_round_trip,
    crc_append,
    crc_corruption,
    crc_guard,
    crc_skip,
    crc_records,
    crc_settings_graph,
    crc_multi_signal,
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
