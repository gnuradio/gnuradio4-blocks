#!/usr/bin/env python3
"""Writes the stream graph files under blocks/digital/test/graphs.

The files test digital stream blocks through rungraph. CMakeLists.txt beside this script
registers them, and neither the build nor ctest runs this script. The lists in the files
stay in the repository; rerun the script after a change to a block's settings or ports,
and commit the files it writes:

    python3 blocks/digital/test/generate_stream_graphs.py [--output-dir <directory>]

The output directory defaults to graphs/ beside this script. The script uses the
standard library only and runs no block.

Where the lists come from:

- Input bytes, bits and symbols are draws of the xorshift generator the C++ QA files use
  (shifts 13, 7 and 17), reproduced in Rng with the seeds those files use.
- Every expected list is an oracle's, computed here from the rule the block documents.
  Each block's oracle opens its section below. Where a standard publishes a block's
  output, the oracle is checked against it before anything is written. No list is a
  block's captured output.
- The codes, tap sets, seeds, profiles, short streams and settings rows are the values
  the C++ cases used.
"""

import argparse
import json
import math
import os
import re
import struct
import textwrap

M64 = (1 << 64) - 1
WIDTH = 116
SCRIPT = "generate_stream_graphs.py"


class U32(int):
    pass


class U64(int):
    pass


class U32s(list):
    pass


class U16s(list):
    pass


class F32(float):
    pass


class F32s(list):
    pass


class C32s(list):
    pass


def f32(x):
    return struct.unpack("f", struct.pack("f", x))[0]


def fmt_float(x):
    if math.isnan(x):
        return ".nan"
    x = f32(x)
    if x == 0.0:
        return "-0.0" if math.copysign(1.0, x) < 0.0 else "0.0"
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
        if re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", v) and v not in (
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
    if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", k):
        return k
    return json.dumps(k)


def fmt_flow_map(m):
    return (
        "{" + ", ".join(fmt_key(k) + ": " + fmt_inline(v) for k, v in m.items()) + "}"
    )


def fmt_inline(v):
    if isinstance(v, dict):
        return fmt_flow_map(v)
    return fmt_scalar(v)


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
        if len(one) <= WIDTH:
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
        elif isinstance(v, U16s):
            self.flow(indent, key, v, "!!uint16")
        elif isinstance(v, F32s):
            self.flow(indent, key, v, "!!float32")
        elif isinstance(v, C32s):
            self.add(" " * indent + key + ": !!complex32")
            for z in v:
                self.add(
                    " " * (indent + 2)
                    + "- (%s, %s)" % (fmt_float(z.real), fmt_float(z.imag))
                )
        elif isinstance(v, list) and v and all(isinstance(x, dict) for x in v):
            self.add(" " * indent + key + ":")
            for x in v:
                self.add(" " * (indent + 2) + "- " + fmt_flow_map(x))
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

    def chain(self, *names):
        for a, b in zip(names, names[1:]):
            self.connect(a, b)

    def vector_source(self, name, kind, values, tags=None):
        params = {"values": typed(kind, values)}
        if tags:
            params["tags"] = tags
        return self.block(f"gr::blocks::basic::VectorSource<{kind}>", name, params)

    def expect_sink(self, name, kind, expected, **params):
        return self.block(
            f"gr::blocks::testing::ExpectSink<{kind}>",
            name,
            {"expected": typed(kind, expected), **params},
        )

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

    def text(self):
        doc = Doc()
        text = " ".join(self.comment.split()) + f" {SCRIPT} writes this file."
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
                    body.add("      records:")
                    for i, r in enumerate(v):
                        body.add(f"        {i}:")
                        for k2, v2 in r.items():
                            body.value(10, k2, v2)
                else:
                    body.value(6, key, v)
        body.add("connections:")
        for a, out, b, inp in self.edges:
            body.add(f"  - [{a}, {out}, {b}, {inp}]")
        return body


def typed(kind, values):
    values = list(values)
    if kind == "float32":
        return F32s(values)
    if kind == "uint16":
        return U16s(values)
    if kind == "complex<float32>":
        return C32s(values)
    return values


def tags_at(entries):
    """A tags map, keyed by the sample index as a decimal string, from (index, map) pairs."""
    out = {}
    for index, m in sorted(entries, key=lambda e: e[0]):
        out.setdefault(str(index), {}).update(m)
    return out


# ---------------------------------------------------------------------------------------------------------------------
# the generator the C++ QA files draw from


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


def random_items(count, bound, seed=0x243F6A8885A308D3):
    rng = Rng(seed)
    return [rng.next() % bound for _ in range(count)]


DIG = "gr::blocks::digital::"


def u32(n):
    return U32(n)


# ---------------------------------------------------------------------------------------------------------------------
# the differential coding oracle
#
# The rules the blocks document: the encoder emits y[n] = (x[n] + y[n-1]) mod M from y[-1] = initial_state, the
# decoder x[n] = (y[n] - y[n-1]) mod M with the mathematical modulus, each reducing an input at or above M first.
# Under nrzi the encoder emits the XNOR of the input and its previous output, and the decoder the XNOR of two
# consecutive inputs. The phasor emits y[n] = x[n] * conj(x[n-1]) from x[-1] = 0.


def encode(xs, modulus=2, initial=0, coding="differential"):
    out, y = [], initial
    for x in xs:
        y = (~(x ^ y)) & 1 if coding == "nrzi" else (x % modulus + y) % modulus
        out.append(y)
    return out


def decode(ys, modulus=2, initial=0, coding="differential"):
    out, prev = [], initial
    for y in ys:
        if coding == "nrzi":
            out.append((~(y ^ prev)) & 1)
            prev = y & 1
        else:
            out.append((y % modulus - prev) % modulus)
            prev = y % modulus
    return out


def phasor(xs):
    out, prev = [], 0j
    for x in xs:
        out.append(x * prev.conjugate())
        prev = x
    return out


EVERY = [2, 3, 4, 5, 6, 8, 10, 16, 17, 32, 64, 100, 128, 255, 256]
WIDE = [300, 512, 1000, 65535, 65536]


def symbols(count, bound, seed):
    return random_items(count, bound, seed)


def differential_round_trip():
    g = Graph(
        "differential-round-trip.yaml",
        """
DifferentialEncoder into DifferentialDecoder over bytes, for every modulus M of 2, 3, 4, 5, 6, 8, 10, 16, 17, 32,
64, 100, 128, 255 and 256 and both initial states 0 and M/2. Each encoded sink expects (x[n] + y[n-1]) mod M, and
each decoded sink expects the input.
""",
    )
    for modulus in EVERY:
        data = symbols(64, modulus, 0x243F6A8885A308D3 + modulus)
        src = g.vector_source(f"m{modulus}_source", "uint8", data)
        for initial in (0, modulus // 2):
            name = f"m{modulus}_i{initial}"
            settings = {"modulus": u32(modulus), "initial_state": u32(initial)}
            enc = g.block(
                DIG + "DifferentialEncoder<uint8>", name + "_encoder", settings
            )
            dec = g.block(
                DIG + "DifferentialDecoder<uint8>", name + "_decoder", settings
            )
            coded = encode(data, modulus, initial)
            assert decode(coded, modulus, initial) == data
            g.chain(src, enc, dec, g.expect_sink(name + "_sink", "uint8", data))
            g.connect(enc, g.expect_sink(name + "_coded", "uint8", coded))
    return g


def differential_wide():
    g = Graph(
        "differential-wide.yaml",
        """
DifferentialEncoder into DifferentialDecoder over uint16 symbols, for the moduli 300, 512, 1000, 65535 and 65536 that
a byte cannot hold. Each encoded sink expects (x[n] + y[n-1]) mod M, every value below M, and each decoded sink
expects the input.
""",
    )
    for modulus in WIDE:
        data = symbols(64, modulus, 0x13198A2E03707344 + modulus)
        coded = encode(data, modulus)
        assert max(coded) < modulus and decode(coded, modulus) == data
        name = f"m{modulus}"
        settings = {"modulus": u32(modulus)}
        src = g.vector_source(name + "_source", "uint16", data)
        enc = g.block(DIG + "DifferentialEncoder<uint16>", name + "_encoder", settings)
        dec = g.block(DIG + "DifferentialDecoder<uint16>", name + "_decoder", settings)
        g.chain(src, enc, dec, g.expect_sink(name + "_sink", "uint16", data))
        g.connect(enc, g.expect_sink(name + "_coded", "uint16", coded))
    return g


def differential_binary():
    g = Graph(
        "differential-binary.yaml",
        """
The binary codes. Under nrzi the encoder transitions on a zero input bit and holds on a one, and the decoder restores
the input from item 0; the plain mod-2 encoder differs from nrzi on every other item. An input of 0 to 255 at modulus
10 encodes and decodes as its values mod 10 do. Under 'differential' at modulus 2 the pair restores the input; the
coded stream inverted decodes to the input with its first bit flipped, and the coded stream decoded as nrzi gives the
complement of the input. Each sink expects its bits or symbols.
""",
    )
    bits = symbols(64, 2, 0xBE5466CF34E90C6C)
    nrzi = {"coding": "nrzi"}
    coded = encode(bits, coding="nrzi")
    xored = encode(bits)
    assert all(coded[i] == xored[i] ^ (1 - i % 2) for i in range(len(bits)))
    assert all(
        (coded[i] != (coded[i - 1] if i else 0)) == (bits[i] == 0)
        for i in range(len(bits))
    )
    src = g.vector_source("nrzi_source", "uint8", bits)
    enc = g.block(DIG + "DifferentialEncoder<uint8>", "nrzi_encoder", nrzi)
    dec = g.block(DIG + "DifferentialDecoder<uint8>", "nrzi_decoder", nrzi)
    g.chain(src, enc, dec, g.expect_sink("nrzi_sink", "uint8", bits))
    g.connect(enc, g.expect_sink("nrzi_coded", "uint8", coded))
    plain = g.block(
        DIG + "DifferentialEncoder<uint8>", "xor_encoder", {"modulus": u32(2)}
    )
    g.chain(src, plain, g.expect_sink("xor_coded", "uint8", xored))

    wide = list(range(256))
    reduced = [v % 10 for v in wide]
    ten = {"modulus": u32(10)}
    for name, block, expected in (
        ("reduce_encoder", "DifferentialEncoder<uint8>", encode(reduced, 10)),
        ("reduce_decoder", "DifferentialDecoder<uint8>", decode(reduced, 10)),
    ):
        assert expected == (encode if "enc" in name else decode)(wide, 10)
        for label, values in (("wide", wide), ("reduced", reduced)):
            src = g.vector_source(f"{name}_{label}_source", "uint8", values)
            blk = g.block(DIG + block, f"{name}_{label}", ten)
            g.chain(src, blk, g.expect_sink(f"{name}_{label}_sink", "uint8", expected))

    data = symbols(64, 2, 0x9E3779B97F4A7C15)
    differential = {"modulus": u32(2), "coding": "differential"}
    coded = encode(data)
    inverted = [v ^ 1 for v in coded]
    flipped = decode(inverted)
    assert flipped[0] != data[0] and flipped[1:] == data[1:]
    complement = decode(coded, coding="nrzi")
    assert complement == [v ^ 1 for v in data]
    src = g.vector_source("polarity_source", "uint8", data)
    enc = g.block(DIG + "DifferentialEncoder<uint8>", "polarity_encoder", differential)
    dec = g.block(DIG + "DifferentialDecoder<uint8>", "polarity_decoder", differential)
    g.chain(src, enc, dec, g.expect_sink("polarity_sink", "uint8", data))
    as_nrzi = g.block(DIG + "DifferentialDecoder<uint8>", "complement_decoder", nrzi)
    g.chain(enc, as_nrzi, g.expect_sink("complement_sink", "uint8", complement))
    src = g.vector_source("inverted_source", "uint8", inverted)
    dec = g.block(DIG + "DifferentialDecoder<uint8>", "inverted_decoder", differential)
    g.chain(src, dec, g.expect_sink("inverted_sink", "uint8", flipped))
    return g


def c32(z):
    return complex(f32(z.real), f32(z.imag))


def differential_phasor():
    g = Graph(
        "differential-phasor.yaml",
        """
DifferentialPhasor over five samples of magnitudes 1, 2, 0.5, 3 and 1 at angles 0.3, -1.1, 2.4, 0.9 and -2.9. The
first output is exactly zero, and each later one is x[n] * conj(x[n-1]): the magnitudes multiplied and the angles
differenced. The sink accepts a deviation of one part in a million of each value.
""",
    )
    magnitudes = [1.0, 2.0, 0.5, 3.0, 1.0]
    angles = [0.3, -1.1, 2.4, 0.9, -2.9]
    signal = [
        c32(complex(m * math.cos(a), m * math.sin(a)))
        for m, a in zip(magnitudes, angles)
    ]
    expected = [c32(z) for z in phasor(signal)]
    assert expected[0] == 0
    for i in range(1, len(signal)):
        assert abs(abs(expected[i]) - magnitudes[i] * magnitudes[i - 1]) < 1e-6 * 6
    src = g.vector_source("phasor_source", "complex<float32>", signal)
    blk = g.block(DIG + "DifferentialPhasor<float32>", "phasor")
    sink = g.expect_sink(
        "phasor_sink", "complex<float32>", expected, relative_tolerance=1e-6
    )
    g.chain(src, blk, sink)
    return g


MARKERS = [
    (0, {"gr:trigger_name": "alpha"}),
    (1, {"gr:trigger_time": U64(111)}),
    (1, {"gr:trigger_offset": F32(0.5)}),
    (37, {"gr:num_channels": u32(3)}),
    (200, {"gr:frequency": F32(42.0)}),
    (239, {"gr:rx_overflow": True}),
    (120, {"private_key": "carried"}),
]
TAGGED_ITEMS = 240


def differential_tags():
    g = Graph(
        "differential-tags.yaml",
        """
Tags through the three differential blocks. Seven tags, six with reserved keys and one with a private_key, ride a stream
of 240 zeros through DifferentialEncoder and DifferentialDecoder at modulus 4 and through DifferentialPhasor, and each
reaches the sink at its own item. DifferentialPhasor drops freq_est and phase_est from a tag at item 100 and forwards
its private_key. Each sink expects zeros and every tag.
""",
    )
    tags = tags_at(MARKERS)
    for name, block, kind, settings in (
        ("encoder", "DifferentialEncoder<uint8>", "uint8", {"modulus": u32(4)}),
        ("decoder", "DifferentialDecoder<uint8>", "uint8", {"modulus": u32(4)}),
        ("phasor", "DifferentialPhasor<float32>", "complex<float32>", {}),
    ):
        zeros = (
            [0j] * TAGGED_ITEMS if kind == "complex<float32>" else [0] * TAGGED_ITEMS
        )
        src = g.vector_source(name + "_source", kind, zeros, tags)
        blk = g.block(DIG + block, name + "_block", settings)
        sink = g.expect_sink(
            name + "_sink", kind, zeros, expected_tags=tags, tags_exact=True
        )
        g.chain(src, blk, sink)

    estimate = {"freq_est": F32(0.01), "phase_est": F32(0.5), "private_key": "carried"}
    zeros = [0j] * 120
    src = g.vector_source(
        "drop_source", "complex<float32>", zeros, tags_at([(100, estimate)])
    )
    blk = g.block(DIG + "DifferentialPhasor<float32>", "drop_phasor")
    sink = g.expect_sink(
        "drop_sink",
        "complex<float32>",
        zeros,
        expected_tags=tags_at([(100, {"private_key": "carried"})]),
        tags_exact=True,
    )
    g.chain(src, blk, sink)
    return g


def differential_settings():
    g = Graph(
        "differential-settings.yaml",
        """
DifferentialEncoder and DifferentialDecoder over bytes at modulus 4, and DifferentialEncoder over uint16 symbols at
modulus 300, each with no symbol to process. The run passes as written. A case that changes settings through SET
either runs to the end with settings the block accepts or ends with the block's reason for refusing them.
""",
    )
    for name, block, kind, modulus in (
        ("encoder", "DifferentialEncoder<uint8>", "uint8", 4),
        ("decoder", "DifferentialDecoder<uint8>", "uint8", 4),
        ("wide_encoder", "DifferentialEncoder<uint16>", "uint16", 300),
    ):
        src = g.vector_source(name + "_source", kind, [])
        blk = g.block(DIG + block, name + "_block", {"modulus": u32(modulus)})
        g.chain(src, blk, g.expect_sink(name + "_sink", kind, []))
    return g


# ---------------------------------------------------------------------------------------------------------------------
# the scrambler oracle
#
# The rule the blocks document: the feedback at bit k of a stream b is the XOR over the tap delays j of b[k - j]. The
# additive generator runs s[k] = f(s, k) from the seed's bits as s[0 .. degree-1] and emits in[k] XOR s[k]. The
# multiplicative scrambler emits y[k] = in[k] XOR f(y, k) and the descrambler in[k] XOR f(in, k), the seed's bits being
# the degree bits before the stream, oldest first. An item carries bits_per_item bits in its low positions, the first
# in time at the top under MsbFirst and at the bottom under LsbFirst. The forced-transition group emits
# y[k] = v XOR x[k] XOR f(y, k) XOR F[k], with F[k] one where the counter reads N - 1, then compares y[k-p] and y[k-q]
# over the bits transmitted in the epoch and sets the counter to zero if they differ and to (c + 1) mod N otherwise.

BASIC = "gr::blocks::basic::"

PROFILES = {
    "ccsds131": ("additive", "1,3,5,8", "11111111", 8, "MsbFirst"),
    "ccsds131_17": ("additive", "3,17", "00011100011100011", 8, "MsbFirst"),
    "pn9": ("additive", "4,9", "111111111", 8, "LsbFirst"),
    "si4463": ("additive", "4,9", "111100001", 8, "LsbFirst"),
    "g3ruh": ("multiplicative", "12,17", "", 8, "MsbFirst"),
}


def hex_bits(text):
    digits = text.replace(" ", "")
    bits = []
    for i in range(0, len(digits), 2):
        byte = int(digits[i : i + 2], 16)
        bits += [(byte >> (7 - b)) & 1 for b in range(8)]
    return bits


class Scrambler:
    """One scrambler block's settings, and its output over a stream with epoch boundaries."""

    def __init__(
        self,
        mode,
        taps="",
        seed="",
        bits=8,
        order="MsbFirst",
        profile="",
        sequence="",
        repeat=True,
        invert=False,
        force=0,
        monitor="",
        period=0,
    ):
        if profile:
            family, taps, seed, bits, order = PROFILES[profile]
            assert (family == "additive") == (mode == "additive"), profile
        self.mode = mode
        self.bits = bits
        self.order = order
        self.period = period
        self.sequence = hex_bits(sequence) if sequence else None
        self.repeat = repeat
        self.taps = [int(t) for t in taps.split(",")] if taps else []
        self.degree = max(self.taps) if self.taps else 0
        self.seed = [int(c) for c in seed] if seed else [0] * self.degree
        assert len(self.seed) == self.degree or self.sequence
        self.invert = 1 if invert else 0
        self.force = force
        self.monitor = sorted(int(t) for t in monitor.split(",")) if monitor else []
        self.unscrambled = 0
        self.fired = 0
        self.reset()

    def reset(self):
        self.k = 0
        self.history = list(self.seed)
        self.sent = []
        self.counter = 0

    def feedback(self, stream):
        n = len(stream)
        return sum(stream[n - j] for j in self.taps) & 1

    def bit(self, x):
        if self.mode == "additive":
            if self.sequence is not None:
                s = len(self.sequence)
                mask = self.sequence[self.k % s] if self.repeat or self.k < s else 0
            else:
                if self.k >= len(self.history):
                    self.history.append(self.feedback(self.history))
                mask = self.history[self.k]
            self.k += 1
            return x ^ mask
        f = self.feedback(self.history)
        force = 1 if self.force and self.counter == self.force - 1 else 0
        self.fired += force
        result = self.invert ^ x ^ f ^ force
        sent = result if self.mode == "scramble" else x
        if self.force:
            p, q = self.monitor
            n = len(self.sent)
            a = self.sent[n - p] if n >= p else 0
            b = self.sent[n - q] if n >= q else 0
            self.counter = 0 if a ^ b else (self.counter + 1) % self.force
        self.sent.append(sent)
        self.history.append(sent)
        return result

    def item(self, value):
        w = self.bits
        if self.sequence is not None and not self.repeat:
            if self.k >= len(self.sequence):
                self.unscrambled += 1
        positions = range(w - 1, -1, -1) if self.order == "MsbFirst" else range(w)
        out = 0
        for pos in positions:
            out |= self.bit((value >> pos) & 1) << pos
        return out

    def run(self, items, resets=()):
        """The output over items, with the tags at the indices in resets marking an epoch boundary."""
        resets = set(resets)
        since = 0
        out = []
        for i, value in enumerate(items):
            if i in resets or (self.period and since >= self.period):
                self.reset()
                since = 0
            out.append(self.item(value))
            since += 1
        return out


def scramble(items, resets=(), **settings):
    return Scrambler("additive", **settings).run(items, resets)


def mscramble(items, resets=(), **settings):
    return Scrambler("scramble", **settings).run(items, resets)


def descramble(items, resets=(), **settings):
    return Scrambler("descramble", **settings).run(items, resets)


def epochs(fn, items, lengths, **settings):
    out, at = [], 0
    for length in lengths:
        out += fn(items[at : at + length], **settings)
        at += length
    assert at >= len(items)
    return out


CCSDS = {"taps": "1,3,5,8", "seed": "11111111"}
ITU_A = {"taps": "18,23", "seed": "1" * 23}
# the self-synchronizing delay sets the standards name, with the degree beside each
SELF_SYNC = [
    ("itu_a", "18,23", 23),
    ("itu_b", "5,23", 23),
    ("v22", "14,17", 17),
    ("dsss", "4,7", 7),
    ("x64b66b", "39,58", 58),
]

CCSDS_ZEROS = [0xFF, 0x48, 0x0E, 0xC0, 0x9A, 0x0D, 0x70, 0xBC]
CCSDS_RECIPROCAL = [0xFF, 0x1A, 0xAF, 0x66, 0x52, 0x23, 0x1E, 0x10]
PLAIN = [0x47, 0x53, 0x4F, 0x43, 0x21, 0x21, 0x21, 0x21]
SCRAMBLED = [0xB8, 0x1B, 0x41, 0x83, 0xBB, 0x2C, 0x51, 0x9D]
PN9 = [0xFF, 0xE1, 0x1D, 0x9A, 0xED, 0x85, 0x33, 0x24]
PN9 += [0xEA, 0x7A, 0xD2, 0x39, 0x70, 0x97, 0x57, 0x0A]

# the published openings the oracle reproduces before a file is written: CCSDS 131.0-B's first 40 bits, the reciprocal
# reading the kernel records beside it, the CCSDS worked example, the CC11xx / SX12xx whitening sequence, and the
# V-series form B and V.22 openings from an all-ones register
assert scramble([0] * 8, **CCSDS) == CCSDS_ZEROS
assert scramble([0] * 8, taps="3,5,7,8", seed="11111111") == CCSDS_RECIPROCAL
assert scramble(PLAIN, **CCSDS) == SCRAMBLED
assert scramble([0] * 16, profile="pn9") == PN9
assert scramble([0] * 8, taps="5,23", seed="1" * 23) == [
    0xFF,
    0xFF,
    0xFE,
    0x0F,
    0x83,
    0xE3,
    0x07,
    0x3E,
]
assert scramble([0] * 8, taps="14,17", seed="1" * 17) == [
    0xFF,
    0xFF,
    0x80,
    0x01,
    0xC0,
    0x07,
    0xE0,
    0x1C,
]


def tag_frame(label="frame"):
    return {"gr:trigger_name": label}


def scrambler_ccsds():
    g = Graph(
        "scrambler-ccsds.yaml",
        """
AdditiveScrambler with CCSDS 131.0-B's taps 1,3,5,8 and seed 11111111. Eight zero bytes come out as the sequence, FF
48 0E C0 9A 0D 70 BC, whose first 40 bits the standard publishes. The worked example scrambles to its published
bytes and back again, XOR being its own inverse. The reciprocal delay set 3,5,7,8 gives FF 1A AF 66 52 23 1E 10,
which is a different sequence. Each sink expects its bytes.
""",
    )
    for name, values, settings, expected in (
        ("zeros", [0] * 8, CCSDS, CCSDS_ZEROS),
        ("plain", PLAIN, CCSDS, SCRAMBLED),
        ("scrambled", SCRAMBLED, CCSDS, PLAIN),
        ("reciprocal", [0] * 8, {**CCSDS, "taps": "3,5,7,8"}, CCSDS_RECIPROCAL),
    ):
        src = g.vector_source(name + "_source", "uint8", values)
        blk = g.block(DIG + "AdditiveScrambler<uint8>", name + "_scrambler", settings)
        g.chain(src, blk, g.expect_sink(name + "_sink", "uint8", expected))
    return g


def scrambler_additive_resets():
    g = Graph(
        "scrambler-additive-resets.yaml",
        """
AdditiveScrambler with the CCSDS settings and its epoch boundaries. A gr:trigger_name tag at item 37 makes that item the
first of a new epoch. Under reset_tag_value frame a tag labeled frame resets and one labeled preamble does not, and
neither does a tag whose value is a uint32; with reset_tag_value empty the uint32 tag resets, and with reset_tag_key
empty a frame tag does not. A reset_period of 23 over zeros repeats the first 23 bytes of the sequence. Under a period
of 16, a tag at item 40 moves the next boundary to item 56, and a tag at item 32, on a boundary, is one reset. Each sink
expects the epochs the boundaries name.
""",
    )
    data = random_items(64, 256)
    resets = [37]
    rows = [
        ("reset", CCSDS, (37, tag_frame()), resets),
    ]
    narrowed = {**CCSDS, "reset_tag_value": "frame"}
    rows += [
        ("matching", narrowed, (40, tag_frame()), [40]),
        ("other", narrowed, (40, tag_frame("preamble")), []),
        ("typed", narrowed, (40, {"gr:trigger_name": u32(7)}), []),
        ("typed_unnarrowed", CCSDS, (40, {"gr:trigger_name": u32(7)}), [40]),
        ("deaf", {**CCSDS, "reset_tag_key": ""}, (40, tag_frame()), []),
    ]
    for name, settings, tag, boundaries in rows:
        src = g.vector_source(name + "_source", "uint8", data, tags_at([tag]))
        blk = g.block(DIG + "AdditiveScrambler<uint8>", name + "_scrambler", settings)
        expected = epochs(scramble, data, cut(boundaries, len(data)), **CCSDS)
        g.chain(src, blk, g.expect_sink(name + "_sink", "uint8", expected))
    assert epochs(scramble, data, [37, 27], **CCSDS) != scramble(data, **CCSDS)

    zeros = [0] * (23 * 3)
    src = g.vector_source("periodic_source", "uint8", zeros)
    blk = g.block(
        DIG + "AdditiveScrambler<uint8>",
        "periodic_scrambler",
        {**CCSDS, "reset_period": u32(23)},
    )
    expected = scramble([0] * 23, **CCSDS) * 3
    assert expected[0] != expected[22]
    g.chain(src, blk, g.expect_sink("periodic_sink", "uint8", expected))

    data = random_items(96, 256)
    for name, at, lengths in (
        ("off_grid", 40, [16, 16, 8, 16, 16, 16, 8]),
        ("on_grid", 32, [16] * 6),
    ):
        src = g.vector_source(
            name + "_source", "uint8", data, tags_at([(at, tag_frame())])
        )
        blk = g.block(
            DIG + "AdditiveScrambler<uint8>",
            name + "_scrambler",
            {**CCSDS, "reset_period": u32(16)},
        )
        expected = epochs(scramble, data, lengths, **CCSDS)
        g.chain(src, blk, g.expect_sink(name + "_sink", "uint8", expected))
    assert epochs(scramble, data, [16, 16, 8, 16, 16, 16, 8], **CCSDS) != epochs(
        scramble, data, [16, 16, 8, 8, 16, 16, 16], **CCSDS
    )
    return g


def cut(boundaries, total):
    """The epoch lengths that boundaries at the given indices cut a stream of total items into."""
    points = [0] + sorted(boundaries) + [total]
    return [b - a for a, b in zip(points, points[1:]) if b > a]


# items in the streams that run past the additive kernel's 256-item sequence tile
LONG_ITEMS = 600


def scrambler_additive_stream():
    g = Graph(
        "scrambler-additive-stream.yaml",
        """
AdditiveScrambler with the CCSDS settings over a stream. At bits_per_item 3, an input of 0xFF and one of 0x07 give the
same output, every item below 8. Two scramblers in series restore the input, and the private_key tag at item 11 and the
gr:trigger_name tag at item 40 reach the sink at their own items. A gr:sample_rate tag at item 50 of 600 items leaves
with the value it arrived with. Each sink expects its items and its tags.
""",
    )
    expected = scramble([7] * 64, bits=3, **CCSDS)
    assert all(v < 8 for v in expected)
    for name, value in (("high", 0xFF), ("low", 0x07)):
        src = g.vector_source(name + "_source", "uint8", [value] * 64)
        blk = g.block(
            DIG + "AdditiveScrambler<uint8>",
            name + "_scrambler",
            {**CCSDS, "bits_per_item": u32(3)},
        )
        g.chain(src, blk, g.expect_sink(name + "_sink", "uint8", expected))

    stream = random_items(64, 256)
    tags = tags_at([(11, {"private_key": "carried"}), (40, tag_frame())])
    src = g.vector_source("series_source", "uint8", stream, tags)
    first = g.block(DIG + "AdditiveScrambler<uint8>", "series_first", CCSDS)
    second = g.block(DIG + "AdditiveScrambler<uint8>", "series_second", CCSDS)
    sink = g.expect_sink(
        "series_sink", "uint8", stream, expected_tags=tags, tags_exact=True
    )
    g.chain(src, first, second, sink)

    long_stream = random_items(LONG_ITEMS, 256)
    rate = tags_at([(50, {"gr:sample_rate": F32(12345.0)})])
    src = g.vector_source("rate_source", "uint8", long_stream, rate)
    blk = g.block(DIG + "AdditiveScrambler<uint8>", "rate_scrambler", CCSDS)
    sink = g.expect_sink(
        "rate_sink",
        "uint8",
        scramble(long_stream, **CCSDS),
        expected_tags=rate,
        tags_exact=True,
    )
    g.chain(src, blk, sink)
    return g


def scrambler_multiplicative():
    g = Graph(
        "scrambler-multiplicative.yaml",
        """
MultiplicativeScrambler with ITU-T V-series form A, taps 18,23 and a seed of 23 ones. Over 600 items its output differs
from AdditiveScrambler's with the same settings, the feedback reading its own output. An empty seed gives what 23 zeros
give, and an empty seed over zeros emits zeros. A gr:trigger_name tag at item 37 opens a run seeded as a fresh block is.
Under a period of 16, a tag at item 40 re-anchors the period and a tag at item 32 is one reset. The private_key tag at
item 11 and the gr:trigger_name tag at item 40 reach the sink at their own items. Each sink expects its items and its
tags.
""",
    )
    long_data = random_items(LONG_ITEMS, 256)
    for name, block, expected in (
        ("feedback", "MultiplicativeScrambler", mscramble(long_data, **ITU_A)),
        ("additive", "AdditiveScrambler<uint8>", scramble(long_data, **ITU_A)),
    ):
        src = g.vector_source(name + "_source", "uint8", long_data)
        blk = g.block(DIG + block, name + "_scrambler", ITU_A)
        g.chain(src, blk, g.expect_sink(name + "_sink", "uint8", expected))
    assert mscramble(long_data, **ITU_A) != scramble(long_data, **ITU_A)

    data = random_items(64, 256)

    expected = mscramble(data, taps="18,23", seed="0" * 23)
    assert expected == mscramble(data, taps="18,23")
    for name, settings in (
        ("empty_seed", {"taps": "18,23"}),
        ("zero_seed", {"taps": "18,23", "seed": "0" * 23}),
    ):
        src = g.vector_source(name + "_source", "uint8", data)
        blk = g.block(DIG + "MultiplicativeScrambler", name + "_scrambler", settings)
        g.chain(src, blk, g.expect_sink(name + "_sink", "uint8", expected))

    src = g.vector_source("dead_source", "uint8", [0] * 64)
    blk = g.block(DIG + "MultiplicativeScrambler", "dead_scrambler", {"taps": "18,23"})
    g.chain(src, blk, g.expect_sink("dead_sink", "uint8", [0] * 64))

    src = g.vector_source("reset_source", "uint8", data, tags_at([(37, tag_frame())]))
    blk = g.block(DIG + "MultiplicativeScrambler", "reset_scrambler", ITU_A)
    expected = epochs(mscramble, data, [37, 27], **ITU_A)
    assert expected != mscramble(data, **ITU_A)
    g.chain(src, blk, g.expect_sink("reset_sink", "uint8", expected))

    data96 = random_items(96, 256)
    for name, at, lengths in (
        ("off_grid", 40, [16, 16, 8, 16, 16, 16, 8]),
        ("on_grid", 32, [16] * 6),
    ):
        src = g.vector_source(
            name + "_source", "uint8", data96, tags_at([(at, tag_frame())])
        )
        blk = g.block(
            DIG + "MultiplicativeScrambler",
            name + "_scrambler",
            {**ITU_A, "reset_period": u32(16)},
        )
        expected = epochs(mscramble, data96, lengths, **ITU_A)
        g.chain(src, blk, g.expect_sink(name + "_sink", "uint8", expected))

    tags = tags_at([(11, {"private_key": "carried"}), (40, tag_frame())])
    src = g.vector_source("tags_source", "uint8", data, tags)
    blk = g.block(DIG + "MultiplicativeScrambler", "tags_scrambler", ITU_A)
    expected = epochs(mscramble, data, [40, 24], **ITU_A)
    sink = g.expect_sink(
        "tags_sink", "uint8", expected, expected_tags=tags, tags_exact=True
    )
    g.chain(src, blk, sink)
    return g


def self_sync_settings(taps, degree, **more):
    return {"taps": taps, "seed": "1" * degree, **more}


def scrambler_self_sync():
    g = Graph(
        "scrambler-self-sync.yaml",
        """
MultiplicativeScrambler into MultiplicativeDescrambler under the five delay sets the standards name for a
self-synchronizing scrambler: V-series forms A and B, V.22, 802.11 DSSS and 802.3 64B/66B, each with a seed of ones.
The pair restores bytes and single bits from item 0. A descrambler with a wrong seed restores every bit from item
degree on, and one joined 61 bits into the stream with a zero seed restores every bit from degree bits after the
join; a SkipHead in front of each sink drops the bits before that. Under form A with reset_period 40 at both ends
the pair restores the stream, and a descrambler that never resets gets the first 23 bits of each later epoch wrong
and the rest right. Each sink expects its bits.
""",
    )
    data = random_items(64, 256)
    bits = random_items(128, 2)
    for label, taps, degree in SELF_SYNC:
        for kind, values, more in (
            ("bytes", data, {}),
            ("bits", bits, {"bits_per_item": u32(1)}),
        ):
            name = f"{label}_{kind}"
            settings = self_sync_settings(taps, degree, **more)
            src = g.vector_source(name + "_source", "uint8", values)
            tx = g.block(DIG + "MultiplicativeScrambler", name + "_scrambler", settings)
            rx = g.block(
                DIG + "MultiplicativeDescrambler", name + "_descrambler", settings
            )
            g.chain(src, tx, rx, g.expect_sink(name + "_sink", "uint8", values))

    rng = Rng(0x13198A2E03707344)
    for label, taps, degree in SELF_SYNC:
        sent = self_sync_settings(taps, degree, bits_per_item=u32(1))
        channel = mscramble(bits, taps=taps, seed="1" * degree, bits=1)
        for trial in range(2):
            wrong = "".join("1" if rng.next() & 1 else "0" for _ in range(degree))
            assert wrong != "1" * degree
            seen = descramble(channel, taps=taps, seed=wrong, bits=1)
            assert seen[degree:] == bits[degree:]
            name = f"{label}_wrong_{trial}"
            src = g.vector_source(name + "_source", "uint8", bits)
            tx = g.block(DIG + "MultiplicativeScrambler", name + "_scrambler", sent)
            rx = g.block(
                DIG + "MultiplicativeDescrambler",
                name + "_descrambler",
                {**sent, "seed": wrong},
            )
            skip = g.block(
                BASIC + "SkipHead<uint8>", name + "_skip", {"n_skip": u32(degree)}
            )
            g.chain(
                src, tx, rx, skip, g.expect_sink(name + "_sink", "uint8", bits[degree:])
            )

    join = 61
    joined = random_items(200, 2)
    for label, taps, degree in SELF_SYNC:
        sent = self_sync_settings(taps, degree, bits_per_item=u32(1))
        name = f"{label}_join"
        src = g.vector_source(name + "_source", "uint8", joined)
        tx = g.block(DIG + "MultiplicativeScrambler", name + "_scrambler", sent)
        late = g.block(BASIC + "SkipHead<uint8>", name + "_late", {"n_skip": u32(join)})
        rx = g.block(
            DIG + "MultiplicativeDescrambler",
            name + "_descrambler",
            {**sent, "seed": "0" * degree},
        )
        skip = g.block(
            BASIC + "SkipHead<uint8>", name + "_skip", {"n_skip": u32(degree)}
        )
        sink = g.expect_sink(name + "_sink", "uint8", joined[join + degree :])
        g.chain(src, tx, late, rx, skip, sink)

    period, degree = 40, 23
    framed_bits = random_items(period * 4, 2)
    per_bit = {**ITU_A, "bits_per_item": u32(1)}
    framed = {**per_bit, "reset_period": u32(period)}
    channel = mscramble(framed_bits, period=period, bits=1, **ITU_A)
    damaged = descramble(channel, bits=1, **ITU_A)
    wrong = [i for i in range(len(framed_bits)) if damaged[i] != framed_bits[i]]
    assert wrong and all(i % period < degree for i in wrong)
    for name, rx_settings, expected in (
        ("framed", framed, framed_bits),
        ("unframed", per_bit, damaged),
    ):
        src = g.vector_source(name + "_source", "uint8", framed_bits)
        tx = g.block(DIG + "MultiplicativeScrambler", name + "_scrambler", framed)
        rx = g.block(
            DIG + "MultiplicativeDescrambler", name + "_descrambler", rx_settings
        )
        g.chain(src, tx, rx, g.expect_sink(name + "_sink", "uint8", expected))
    return g


def scrambler_descrambler_resets():
    g = Graph(
        "scrambler-descrambler-resets.yaml",
        """
MultiplicativeDescrambler with taps 18,23 and a seed of 23 ones and its epoch boundaries. A gr:trigger_name tag at item
37 opens a run seeded as a fresh block is. Under a period of 16, a tag at item 40 re-anchors the period and a tag at
item 32 is one reset. Each sink expects the epochs the boundaries name.
""",
    )
    data = random_items(64, 256)
    src = g.vector_source("reset_source", "uint8", data, tags_at([(37, tag_frame())]))
    blk = g.block(DIG + "MultiplicativeDescrambler", "reset_descrambler", ITU_A)
    expected = epochs(descramble, data, [37, 27], **ITU_A)
    assert expected != descramble(data, **ITU_A)
    g.chain(src, blk, g.expect_sink("reset_sink", "uint8", expected))

    data96 = random_items(96, 256)
    for name, at, lengths in (
        ("off_grid", 40, [16, 16, 8, 16, 16, 16, 8]),
        ("on_grid", 32, [16] * 6),
    ):
        src = g.vector_source(
            name + "_source", "uint8", data96, tags_at([(at, tag_frame())])
        )
        blk = g.block(
            DIG + "MultiplicativeDescrambler",
            name + "_descrambler",
            {**ITU_A, "reset_period": u32(16)},
        )
        expected = epochs(descramble, data96, lengths, **ITU_A)
        g.chain(src, blk, g.expect_sink(name + "_sink", "uint8", expected))
    return g


def scrambler_family():
    g = Graph(
        "scrambler-family.yaml",
        """
The three scramblers in chains. MultiplicativeScrambler into MultiplicativeDescrambler under form A restores the stream,
and the private_key tag at item 11 and the gr:trigger_name tag at item 40 reach the sink at their own items.
AdditiveScrambler under the CCSDS settings, then the multiplicative pair, carry a gr:sample_rate tag at item 50 with the
value it arrived with. Each sink expects its items and its tags.
""",
    )
    stream = random_items(64, 256)
    tags = tags_at([(11, {"private_key": "carried"}), (40, tag_frame())])
    src = g.vector_source("pair_source", "uint8", stream, tags)
    tx = g.block(DIG + "MultiplicativeScrambler", "pair_scrambler", ITU_A)
    rx = g.block(DIG + "MultiplicativeDescrambler", "pair_descrambler", ITU_A)
    sink = g.expect_sink(
        "pair_sink", "uint8", stream, expected_tags=tags, tags_exact=True
    )
    g.chain(src, tx, rx, sink)

    rate = tags_at([(50, {"gr:sample_rate": F32(12345.0)})])
    src = g.vector_source("rate_source", "uint8", stream, rate)
    additive = g.block(DIG + "AdditiveScrambler<uint8>", "rate_additive", CCSDS)
    tx = g.block(DIG + "MultiplicativeScrambler", "rate_scrambler", ITU_A)
    rx = g.block(DIG + "MultiplicativeDescrambler", "rate_descrambler", ITU_A)
    sink = g.expect_sink(
        "rate_sink",
        "uint8",
        scramble(stream, **CCSDS),
        expected_tags=rate,
        tags_exact=True,
    )
    g.chain(src, additive, tx, rx, sink)
    return g


def scrambler_profiles():
    g = Graph(
        "scrambler-profiles.yaml",
        """
The named profiles. ccsds131 over zeros gives the CCSDS sequence, as the taps and seed spelled out do. pn9 over zeros
gives the CC11xx and SX12xx whitening sequence FF E1 1D 9A ED 85 33 24, read least significant bit first, and does so
with bits_per_item 8 and bit_order MsbFirst staged beside it. The g3ruh pair restores its input, and the g3ruh
scrambler gives what taps 12,17 and a seed of 17 zeros give. Each sink expects its bytes.
""",
    )
    rows = [
        (
            "ccsds",
            "AdditiveScrambler<uint8>",
            {"profile": "ccsds131"},
            [0] * 8,
            CCSDS_ZEROS,
        ),
        ("spelled", "AdditiveScrambler<uint8>", CCSDS, [0] * 8, CCSDS_ZEROS),
        ("pn9", "AdditiveScrambler<uint8>", {"profile": "pn9"}, [0] * 16, PN9),
        (
            "pn9_staged",
            "AdditiveScrambler<uint8>",
            {"profile": "pn9", "bits_per_item": u32(8), "bit_order": "MsbFirst"},
            [0] * 16,
            PN9,
        ),
    ]
    data = random_items(64, 256)
    g3ruh = mscramble(data, profile="g3ruh")
    assert g3ruh == mscramble(data, taps="12,17", seed="0" * 17)
    rows += [
        ("g3ruh", "MultiplicativeScrambler", {"profile": "g3ruh"}, data, g3ruh),
        (
            "g3ruh_spelled",
            "MultiplicativeScrambler",
            {"taps": "12,17", "seed": "0" * 17},
            data,
            g3ruh,
        ),
    ]
    for name, block, settings, values, expected in rows:
        src = g.vector_source(name + "_source", "uint8", values)
        blk = g.block(DIG + block, name + "_scrambler", settings)
        g.chain(src, blk, g.expect_sink(name + "_sink", "uint8", expected))

    src = g.vector_source("pair_source", "uint8", data)
    tx = g.block(
        DIG + "MultiplicativeScrambler", "pair_scrambler", {"profile": "g3ruh"}
    )
    rx = g.block(
        DIG + "MultiplicativeDescrambler", "pair_descrambler", {"profile": "g3ruh"}
    )
    g.chain(src, tx, rx, g.expect_sink("pair_sink", "uint8", data))
    return g


MASK = "FF E1 1D 9A"


def scrambler_sequence():
    g = Graph(
        "scrambler-sequence.yaml",
        """
AdditiveScrambler with the mask FF E1 1D 9A given as a sequence. Repeating, it XORs the mask into 41 bytes, tiling it,
and the spelling ffe11d9a gives the same bytes. Not repeating, it XORs the mask into the first four of eleven bytes,
leaves the other seven as they came and counts them, which the block states when it stops. Each sink expects its
bytes.
""",
    )
    mask = [0xFF, 0xE1, 0x1D, 0x9A]
    data = random_items(41, 256)
    expected = [v ^ mask[i % 4] for i, v in enumerate(data)]
    assert scramble(data, sequence=MASK) == expected
    for name, spelling in (("tiled", MASK), ("spelled", "ffe11d9a")):
        src = g.vector_source(name + "_source", "uint8", data)
        blk = g.block(
            DIG + "AdditiveScrambler<uint8>",
            name + "_scrambler",
            {"sequence": spelling},
        )
        g.chain(src, blk, g.expect_sink(name + "_sink", "uint8", expected))

    data = random_items(11, 256)
    expected = [v ^ mask[i] if i < 4 else v for i, v in enumerate(data)]
    once = Scrambler("additive", sequence=MASK, repeat=False)
    assert once.run(data) == expected and once.unscrambled == 7
    src = g.vector_source("once_source", "uint8", data)
    blk = g.block(
        DIG + "AdditiveScrambler<uint8>",
        "once_scrambler",
        {"sequence": MASK, "sequence_repeat": False},
    )
    g.chain(src, blk, g.expect_sink("once_sink", "uint8", expected))
    return g


def forced(after):
    return {
        "taps": "3,20",
        "seed": "1" * 20,
        "invert_output": True,
        "force_transition_after": u32(after),
        "monitor_delays": "1,2",
    }


def forced_model(after):
    return dict(taps="3,20", seed="1" * 20, invert=True, force=after, monitor="1,2")


STRAIGHT = {"taps": "3,20", "seed": "1" * 20}

# the firings of the forced-transition pair at N = 8 over FORCED_ITEMS bytes, which the two blocks state when they stop
FORCED_ITEMS = 128


def forced_firings():
    data = random_items(FORCED_ITEMS, 256)
    tx = Scrambler("scramble", **forced_model(8))
    channel = tx.run(data)
    rx = Scrambler("descramble", **forced_model(8))
    assert rx.run(channel) == data
    assert tx.fired == rx.fired and tx.fired > 0
    return tx.fired


def scrambler_forced():
    g = Graph(
        "scrambler-forced.yaml",
        f"""
The forced-transition group of the multiplicative pair: taps 3,20, a seed of 20 ones, the output inverted, and a
complement forced at the Nth item over which the transmitted bits one and two items back stay equal. At N = 8 the
pair restores {FORCED_ITEMS} bytes and the channel carries the scrambled bytes, and the two blocks state the same
count of firings when they stop. With force_transition_after 0 the scrambler gives what the plain recursion gives. At
N = 32 the pair restores its input, the scrambler's output differs from the plain one's, and a descrambler without
the inversion and the rule recovers something else. Each sink expects its bytes.
""",
    )
    data = random_items(FORCED_ITEMS, 256)
    channel = mscramble(data, **forced_model(8))
    assert channel != data
    assert forced_firings() == 5
    src = g.vector_source("running_source", "uint8", data)
    tx = g.block(DIG + "MultiplicativeScrambler", "running_scrambler", forced(8))
    rx = g.block(DIG + "MultiplicativeDescrambler", "running_descrambler", forced(8))
    g.chain(src, tx, rx, g.expect_sink("running_sink", "uint8", data))
    g.connect(tx, g.expect_sink("running_channel", "uint8", channel))

    data = random_items(64, 256)
    plain = mscramble(data, **{"taps": "3,20", "seed": "1" * 20})
    for name, settings in (
        ("off", {**STRAIGHT, "force_transition_after": u32(0)}),
        ("plain", STRAIGHT),
    ):
        src = g.vector_source(name + "_source", "uint8", data)
        blk = g.block(DIG + "MultiplicativeScrambler", name + "_scrambler", settings)
        g.chain(src, blk, g.expect_sink(name + "_sink", "uint8", plain))

    inverted = mscramble(data, **forced_model(32))
    assert inverted != plain
    other = descramble(inverted, taps="3,20", seed="1" * 20)
    assert other != data
    src = g.vector_source("inverted_source", "uint8", data)
    tx = g.block(DIG + "MultiplicativeScrambler", "inverted_scrambler", forced(32))
    rx = g.block(DIG + "MultiplicativeDescrambler", "inverted_descrambler", forced(32))
    straight = g.block(
        DIG + "MultiplicativeDescrambler", "straight_descrambler", STRAIGHT
    )
    g.chain(src, tx, rx, g.expect_sink("inverted_sink", "uint8", data))
    g.connect(tx, g.expect_sink("inverted_channel", "uint8", inverted))
    g.chain(tx, straight, g.expect_sink("straight_sink", "uint8", other))
    return g


def scrambler_records():
    g = Graph(
        "scrambler-records.yaml",
        """
AdditiveScrambler on DataSet records, where one record is one epoch. Two records of the same 64 bits, at
bits_per_item 1 under the CCSDS settings, each come out as the stream scrambler's fresh epoch, with the origin meta
key and the signal name crossing. ccsds131 over a record of five zeros gives FF 48 0E C0 9A. reset_period and
reset_tag_key change nothing on a record. Each sink expects its records.
""",
    )
    bits = random_items(64, 2)
    stream = scramble(bits, bits=1, **CCSDS)

    def rec(values):
        return {
            "signal_values": values,
            "signal_names": ["digital"],
            "meta_information": {"origin": "qa"},
        }

    per_bit = {**CCSDS, "bits_per_item": u32(1)}
    rows = [
        ("carrier", per_bit, [rec(bits), rec(bits)], [rec(stream), rec(stream)]),
        (
            "opening",
            {"profile": "ccsds131"},
            [rec([0] * 5)],
            [rec(CCSDS_ZEROS[:5])],
        ),
        ("quiet", per_bit, [rec(bits)], [rec(stream)]),
        (
            "decoyed",
            {**per_bit, "reset_period": u32(3), "reset_tag_key": "some_key"},
            [rec(bits)],
            [rec(stream)],
        ),
    ]
    for name, settings, records, expected in rows:
        src = g.records_source(name + "_source", "uint8", records)
        blk = g.block(
            DIG + "AdditiveScrambler<gr::DataSet<uint8>>", name + "_scrambler", settings
        )
        g.chain(src, blk, g.records_sink(name + "_sink", "uint8", expected))
    return g


def scrambler_settings():
    g = Graph(
        "scrambler-settings.yaml",
        """
The scramblers with no item to process: AdditiveScrambler under the CCSDS settings and under the mask FF E1 1D 9A
without repeating, and MultiplicativeScrambler and MultiplicativeDescrambler under taps 3,20 and a seed of 20 ones.
The run passes as written. A case that changes settings through SET either runs to the end with settings the block
accepts or ends with the block's reason for refusing them.
""",
    )
    for name, block, settings in (
        ("additive", "AdditiveScrambler<uint8>", CCSDS),
        (
            "sequence",
            "AdditiveScrambler<uint8>",
            {"sequence": MASK, "sequence_repeat": False},
        ),
        ("scrambler", "MultiplicativeScrambler", STRAIGHT),
        ("descrambler", "MultiplicativeDescrambler", STRAIGHT),
    ):
        src = g.vector_source(name + "_source", "uint8", [])
        blk = g.block(DIG + block, name + "_block", settings)
        g.chain(src, blk, g.expect_sink(name + "_sink", "uint8", []))
    return g


# ---------------------------------------------------------------------------------------------------------------------
# the access code oracle
#
# The rule the block documents: at item i, once (n-1)*s + 1 items have arrived, the n items i-1-(n-1)*s, ..., i-1 at
# stride s are compared with the code's n symbols of bits_per_item bits each, first item against the code's first
# symbol. A symbol is wrong when any of its bits is, and a distance of at most max_errors tags item i, or under
# code_start emits the code's first item there behind a lag of (n-1)*s + 1 zeros. A byte item at one bit per item is
# one iff non-zero, a soft item one iff it is at least zero, and a byte item of wider symbols carries its low bits.

LABEL = "access_code"
INTEROP = 0xACDDA4E2F28C20FC


def code_string(word, bits):
    return format(word, "0%db" % bits)


def items_of(bits):
    return [1 if c == "1" else 0 for c in bits]


def plant(word, bits, at, total):
    stream = [0] * total
    stream[at : at + bits] = items_of(code_string(word, bits))
    return stream


def plant_strided(word, n, spacing, at, total):
    stream = [0] * total
    for i, c in enumerate(code_string(word, n)):
        stream[at + i * spacing] = int(c)
    return stream


def symbol_of(item, width):
    if isinstance(item, float):
        return 1 if item >= 0.0 else 0
    if width == 1:
        return 1 if item != 0 else 0
    return item & ((1 << width) - 1)


def correlate(stream, code, max_errors=0, width=1, stride=1):
    """The tags the block emits over stream: (index, symbols wrong), in order."""
    symbols = [int(code[k : k + width], 2) for k in range(0, len(code), width)]
    n = len(symbols)
    first = (n - 1) * stride + 1
    found = []
    for i in range(first, len(stream)):
        window = [stream[i - 1 - (n - 1 - k) * stride] for k in range(n)]
        wrong = sum(symbol_of(v, width) != s for v, s in zip(window, symbols))
        if wrong <= max_errors:
            found.append((i, wrong))
    return found


def detection(errors, length, label=LABEL):
    return {
        "gr:trigger_name": label,
        "gr:trigger_offset": F32(0.0),
        "gr:trigger_meta_info": {
            "sync_errors": u32(errors),
            "code_length": u32(length),
        },
    }


def correlator(g, name, kind, stream, settings, tags=None, expected=None, **sink):
    """A source of stream, the correlator and a sink of its output, which is the stream unless expected says."""
    src = g.vector_source(name + "_source", kind, stream, tags)
    blk = g.block(DIG + f"AccessCodeCorrelator<{kind}>", name + "_correlator", settings)
    out = stream if expected is None else expected
    g.chain(src, blk, g.expect_sink(name + "_sink", kind, out, **sink))
    return blk


def indices(found):
    return U32s(i for i, _ in found)


def access_code_detection():
    g = Graph(
        "access-code-detection.yaml",
        """
AccessCodeCorrelator over byte bits, each stream passing through unchanged. The code 1011 in 1011110110 tags items 4 and
9, the item after each match, and in 10110110 tags items 4 and 7, the two matches sharing an item. The stream with the
64-bit interoperability code at item 20 passes through. With the code 1111, the byte 0x02 is a one at one bit per item,
and four of them tag item 16. Each sink expects the stream and the indices of the gr:trigger_name tags.
""",
    )
    for name, stream, code, want in (
        ("worked", items_of("1011110110"), "1011", [4, 9]),
        ("overlapping", items_of("10110110"), "1011", [4, 7]),
        ("sliced", [0] * 12 + [2] * 4 + [0] * 8, "1111", [16]),
    ):
        found = correlate(stream, code)
        assert indices(found) == want, name
        correlator(
            g,
            name,
            "uint8",
            stream,
            {"access_code": code},
            tag_key="gr:trigger_name",
            tag_indices=indices(found),
        )
    correlator(
        g, "passthrough", "uint8", plant(INTEROP, 64, 20, 100), {"access_code": "1011"}
    )
    return g


P25_SYNC = code_string(0x5575F5FF77FF, 48)


def dibits(bits):
    return [int(bits[k : k + 2], 2) for k in range(0, len(bits), 2)]


def access_code_errors():
    g = Graph(
        "access-code-errors.yaml",
        """
AccessCodeCorrelator distances. A 32-bit code with as many bit errors as max_errors, for each bound from 0 to 4, is
tagged at item 72 with the distance as sync_errors in gr:trigger_meta_info, and one error more is not tagged. P25's
48-bit frame sync as 24 dibits at bits_per_item 2 is tagged at item 44; with one dibit wrong in both bits it is not
tagged exactly and is tagged at a bound of one symbol. A 66-bit code at bits_per_item 3, whose first symbol straddles
the two 64-bit limbs, is tagged at item 42; with that symbol wrong in its middle bit or in all three it is not tagged
exactly and is tagged at a bound of one, at a distance of one. Each sink expects the stream and its tags.
""",
    )
    rng = Rng()
    code = rng.next() & 0xFFFFFFFF
    assert bin(code).count("1") > 8
    word = code_string(code, 32)
    for bound in range(5):
        for flips in (bound, bound + 1):
            corrupted = code
            for k in range(flips):
                corrupted ^= 1 << (2 * k)
            stream = plant(corrupted, 32, 40, 80)
            found = correlate(stream, word, bound)
            name = f"bound_{bound}_errors_{flips}"
            settings = {"access_code": word, "max_errors": u32(bound)}
            if flips == bound:
                assert found == [(72, bound)]
                tags = tags_at([(72, detection(bound, 32))])
                correlator(
                    g,
                    name,
                    "uint8",
                    stream,
                    settings,
                    expected_tags=tags,
                    tags_exact=True,
                )
            else:
                assert found == []
                correlator(
                    g, name, "uint8", stream, settings, tag_key="gr:trigger_name"
                )

    stream = [0] * 20 + dibits(P25_SYNC) + [0] * 20
    damaged = list(stream)
    damaged[27] ^= 3
    for name, values, bound, want in (
        ("dibit_exact", stream, 0, [(44, 0)]),
        ("dibit_strict", damaged, 0, []),
        ("dibit_lenient", damaged, 1, [(44, 1)]),
    ):
        found = correlate(values, P25_SYNC, bound, width=2)
        assert found == want, name
        settings = {
            "access_code": P25_SYNC,
            "bits_per_item": u32(2),
            "max_errors": u32(bound),
        }
        tags = tags_at([(i, detection(e, 48)) for i, e in found])
        correlator(
            g, name, "uint8", values, settings, expected_tags=tags, tags_exact=True
        )

    code66 = "".join("101" if i % 2 == 0 else "010" for i in range(22))
    stream = [0] * 20 + [5 if i % 2 == 0 else 2 for i in range(22)] + [0] * 18
    rows = [("limbs_exact", stream, 0, [(42, 0)])]
    for label, item in (("middle", 5 ^ 2), ("whole", 5 ^ 7)):
        damaged = list(stream)
        damaged[20] = item
        rows += [
            (f"limbs_{label}_strict", damaged, 0, []),
            (f"limbs_{label}_lenient", damaged, 1, [(42, 1)]),
        ]
    for name, values, bound, want in rows:
        found = correlate(values, code66, bound, width=3)
        assert found == want, name
        settings = {
            "access_code": code66,
            "bits_per_item": u32(3),
            "max_errors": u32(bound),
        }
        tags = tags_at([(i, detection(e, 66)) for i, e in found])
        correlator(
            g, name, "uint8", values, settings, expected_tags=tags, tags_exact=True
        )
    return g


def access_code_warmup():
    g = Graph(
        "access-code-warmup.yaml",
        """
AccessCodeCorrelator before its register holds a whole code. A code of zeros of length 1, 4, 17, 64, 65 and 128 over
zeros is first tagged at the item equal to its length and then at every item after it. Over soft items, +0.0 and -0.0
slice to one as 1.0 does, so a code of eight ones is tagged from item 8 on over each, and a NaN slices to zero as -1.0
does, so a code of eight zeros is tagged from item 8 on over each. Each sink expects the stream and the indices of the
gr:trigger_name tags.
""",
    )
    for length in (1, 4, 17, 64, 65, 128):
        stream = [0] * (length + 8)
        found = correlate(stream, "0" * length)
        assert indices(found) == list(range(length, length + 8))
        correlator(
            g,
            f"zeros_{length}",
            "uint8",
            stream,
            {"access_code": "0" * length},
            tag_key="gr:trigger_name",
            tag_indices=indices(found),
        )
    for name, value, code in (
        ("plus_zero", 0.0, "1" * 8),
        ("minus_zero", -0.0, "1" * 8),
        ("plus_one", 1.0, "1" * 8),
        ("nan", math.nan, "0" * 8),
        ("minus_one", -1.0, "0" * 8),
    ):
        stream = [value] * 16
        found = correlate(stream, code)
        assert indices(found) == list(range(8, 16)), name
        correlator(
            g,
            name,
            "float32",
            stream,
            {"access_code": code},
            tag_key="gr:trigger_name",
            tag_indices=indices(found),
        )
    return g


def access_code_tags():
    g = Graph(
        "access-code-tags.yaml",
        """
AccessCodeCorrelator with the 64-bit interoperability code planted at item 30, which it tags at item 94. A private_key
tag at items 11 and 110 reaches the sink at both items beside the detection. An upstream gr:trigger_name tag labeled
preamble at item 11 reaches the sink unchanged, and the detection carries the trigger_label access_code. Each sink
expects the stream and every tag.
""",
    )
    stream = plant(INTEROP, 64, 30, 120)
    word = code_string(INTEROP, 64)
    assert correlate(stream, word) == [(94, 0)]
    private = [(11, {"private_key": "carried"}), (110, {"private_key": "carried"})]
    correlator(
        g,
        "private",
        "uint8",
        stream,
        {"access_code": word},
        tags=tags_at(private),
        expected_tags=tags_at(private + [(94, detection(0, 64))]),
        tags_exact=True,
    )
    upstream = [(11, {"gr:trigger_name": "preamble"})]
    correlator(
        g,
        "upstream",
        "uint8",
        stream,
        {"access_code": word, "trigger_label": LABEL},
        tags=tags_at(upstream),
        expected_tags=tags_at(upstream + [(94, detection(0, 64))]),
        tags_exact=True,
    )
    return g


FRAMING_PAYLOAD = 16


def access_code_framing():
    g = Graph(
        "access-code-framing.yaml",
        f"""
AccessCodeCorrelator into StreamToDataSet, a fixed-length framing chain. The 64-bit interoperability code is planted
at items 20, 120 and 220 of zero bits, each followed by {FRAMING_PAYLOAD} random bits. StreamToDataSet, filtering on
the access_code label with no item before the trigger and {FRAMING_PAYLOAD} after it, cuts one record per code. The
sink expects each record to hold the payload that followed its code.
""",
    )
    word = code_string(INTEROP, 64)
    stream = [0] * 330
    payload = random_items(3 * FRAMING_PAYLOAD, 2)
    starts = []
    for k, at in enumerate((20, 120, 220)):
        stream[at : at + 64] = items_of(word)
        start = at + 64
        stream[start : start + FRAMING_PAYLOAD] = payload[
            k * FRAMING_PAYLOAD : (k + 1) * FRAMING_PAYLOAD
        ]
        starts.append(start)
    assert indices(correlate(stream, word)) == starts
    src = g.vector_source("framing_source", "uint8", stream)
    blk = g.block(
        DIG + "AccessCodeCorrelator<uint8>", "framing_correlator", {"access_code": word}
    )
    cutter = g.block(
        BASIC + "StreamToDataSet<uint8>",
        "framing_cutter",
        {"filter": f"[{LABEL}]", "n_pre": u32(0), "n_post": u32(FRAMING_PAYLOAD)},
    )
    records = [{"signal_values": stream[s : s + FRAMING_PAYLOAD]} for s in starts]
    g.chain(src, blk, cutter, g.records_sink("framing_sink", "uint8", records))
    return g


# the AO-40 geometries: a code of n items at stride s is one row of an s by n block interleaver
AO40 = [("ao40_long", 65, 80), ("ao40_short", 52, 51)]


def access_code_strided():
    g = Graph(
        "access-code-strided.yaml",
        """
AccessCodeCorrelator with a code spread over a stride. The AO-40 long form, 65 items at stride 80, and short form, 52
at stride 51, planted at item 11 are each tagged at the item after the code's last. Codes of 4 items at stride 3 and 8
at stride 5, planted at item 7, are tagged at the item after the code's last, and the block
states one detection when it stops; under tag_at code_start the tag index is the same and the output lags the input
by (n-1)*s + 1 items, zeros first. A 32-item code at stride 17 with max_errors 5 is tagged with 0, 1, 3 and 5 items
wrong, at that distance, and not with 6. With max_errors at the code's length of 8, a stream is tagged at every item
from (n-1)*s + 1 on, for strides 1, 3, 5 and 40. Each sink expects its stream and its tags.
""",
    )
    for name, n, spacing in AO40:
        rng = Rng()
        word = "".join("1" if rng.next() & 1 else "0" for _ in range(n))
        lag = (n - 1) * spacing + 1
        at = 11
        stream = [0] * (at + lag + 32)
        for i, c in enumerate(word):
            stream[at + i * spacing] = int(c)
        found = correlate(stream, word, stride=spacing)
        assert indices(found) == [at + lag], name
        correlator(
            g,
            name,
            "uint8",
            stream,
            {"access_code": word, "stride": u32(spacing)},
            tag_key="gr:trigger_name",
            tag_indices=indices(found),
        )

    for n, spacing, word in ((4, 3, 0b1011), (8, 5, 0b10110100)):
        lag = (n - 1) * spacing + 1
        at = 7
        stream = plant_strided(word, n, spacing, at, at + lag + 40)
        code = code_string(word, n)
        found = correlate(stream, code, stride=spacing)
        assert indices(found) == [at + lag]
        settings = {"access_code": code, "stride": u32(spacing)}
        name = f"strided_{n}_{spacing}"
        correlator(
            g,
            name,
            "uint8",
            stream,
            settings,
            tag_key="gr:trigger_name",
            tag_indices=indices(found),
        )
        delayed = [0] * lag + stream[:-lag]
        assert delayed[at + lag] == stream[at]
        correlator(
            g,
            f"delayed_{n}_{spacing}",
            "uint8",
            stream,
            {**settings, "tag_at": "code_start"},
            expected=delayed,
            tag_key="gr:trigger_name",
            tag_indices=indices(found),
        )

    items, spacing, word, tolerated = 32, 17, 0xB4D1CE73, 5
    lag = (items - 1) * spacing + 1
    code = code_string(word, items)
    for wrong in (0, 1, 3, tolerated, tolerated + 1):
        stream = plant_strided(word, items, spacing, 3, 3 + lag + 8)
        for k in range(wrong):
            stream[3 + (2 + 5 * k) * spacing] ^= 1
        found = correlate(stream, code, tolerated, stride=spacing)
        settings = {
            "access_code": code,
            "stride": u32(spacing),
            "max_errors": u32(tolerated),
        }
        name = f"survives_{wrong}"
        if wrong <= tolerated:
            assert found == [(3 + lag, wrong)]
            tags = tags_at([(3 + lag, detection(wrong, items))])
            correlator(
                g, name, "uint8", stream, settings, expected_tags=tags, tags_exact=True
            )
        else:
            assert found == []
            correlator(g, name, "uint8", stream, settings, tag_key="gr:trigger_name")

    code = code_string(0xA5, 8)
    for spacing in (1, 3, 5, 40):
        lag = 7 * spacing + 1
        rng = Rng()
        stream = [rng.next() & 1 for _ in range(lag + 30)]
        found = correlate(stream, code, 8, stride=spacing)
        assert indices(found) == list(range(lag, lag + 30))
        settings = {"access_code": code, "stride": u32(spacing), "max_errors": u32(8)}
        correlator(
            g,
            f"warm_{spacing}",
            "uint8",
            stream,
            settings,
            tag_key="gr:trigger_name",
            tag_indices=indices(found),
        )
    return g


def access_code_settings():
    g = Graph(
        "access-code-settings.yaml",
        """
AccessCodeCorrelator over bytes and over soft items, each with the code 10110100 and no item to process. The run
passes as written. A case that changes settings through SET either runs to the end with settings the block accepts
or ends with the block's reason for refusing them.
""",
    )
    for name, kind in (("byte", "uint8"), ("soft", "float32")):
        src = g.vector_source(name + "_source", kind, [])
        blk = g.block(
            DIG + f"AccessCodeCorrelator<{kind}>",
            name + "_correlator",
            {"access_code": "10110100"},
        )
        g.chain(src, blk, g.expect_sink(name + "_sink", kind, []))
    return g


GRAPHS = [
    differential_round_trip,
    differential_wide,
    differential_binary,
    differential_phasor,
    differential_tags,
    differential_settings,
    scrambler_ccsds,
    scrambler_additive_resets,
    scrambler_additive_stream,
    scrambler_multiplicative,
    scrambler_self_sync,
    scrambler_descrambler_resets,
    scrambler_family,
    scrambler_profiles,
    scrambler_sequence,
    scrambler_forced,
    scrambler_records,
    scrambler_settings,
    access_code_detection,
    access_code_errors,
    access_code_warmup,
    access_code_tags,
    access_code_framing,
    access_code_strided,
    access_code_settings,
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
