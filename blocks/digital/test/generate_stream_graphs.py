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


GRAPHS = [
    differential_round_trip,
    differential_wide,
    differential_binary,
    differential_phasor,
    differential_tags,
    differential_settings,
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
