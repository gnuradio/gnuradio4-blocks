#!/usr/bin/env python3
"""Write the fec family's graph files.

This script writes every graph file under graphs/ beside it. QA does not run it: ctest runs
only the graph files, and no CMake rule names this script. Rerun it by hand after a change to
a block's output or to a case, and commit the graph files it rewrites with that change.

Every input list is pinned in its file. Information bits and symbols are draws of Lcg, a
64-bit linear congruential generator, and each file's header names its seed.

Most expected lists come from an oracle in this script that does not run the block under
test:

- the drawn information, for a round trip
- the published codewords and the pinned Reed-Solomon parity
- a convolutional encoder written from the generator polynomials
- the puncture masks, and the channel noise drawn from its seed

The other lists are a block's own output, captured by running the block through rungraph
with the build tree's plugins (capture()). They are the binary and Reed-Solomon codewords a
damaged input starts from, a decoder's output where it cannot return the information, and
the output that a case compares a second run against. Each file's header names the sinks
that hold a captured list.

After a change to a block, rebuild its plugin and rerun the script. The diff then shows the
change in the lists captured from the blocks. A change to the output an oracle pins fails
that graph's ctest case and changes no file. Captures are cached under --cache-dir, keyed by
the capture graph, the rungraph binary and the plugin libraries, so a rebuilt plugin or core
is captured again. A rerun with nothing changed writes the same files.

    python3 blocks/fec/test/generate_graphs.py --build-dir build

--mutants-dir DIR also writes, for every sink, a copy of its graph with one expected value
changed, and lists the copies in DIR/mutants.txt. Each copy must fail under rungraph, which
shows that the sink checks the list it holds.
"""

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import struct
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
M64 = (1 << 64) - 1
WIDTH = 116


class Options:
    """The paths a run uses, set by main()."""

    output_dir = None
    cache_dir = None
    rungraph = None
    environment = None
    digest = None


# Generators and oracles.


class Lcg:
    """A 64-bit LCG whose output is the state shifted right by 17."""

    def __init__(self, seed):
        self.state = seed

    def next(self):
        self.state = (self.state * 6364136223846793005 + 1442695040888963407) & M64
        return self.state >> 17

    def bits(self, n):
        return [self.next() & 1 for _ in range(n)]

    def symbols(self, n, mask=0x3F):
        return [self.next() & mask for _ in range(n)]


SEED = 0x9E3779B97F4A7C15


def bits_of(word, count):
    return [(word >> (count - 1 - i)) & 1 for i in range(count)]


def f32(x):
    return struct.unpack("f", struct.pack("f", x))[0]


def bch15_codewords(generator, k):
    words = []
    for info in range(1 << k):
        product = 0
        for bit in range(k):
            if (info >> bit) & 1:
                product ^= generator << bit
        words.append(product)
    return words


def flip(values, positions):
    out = list(values)
    for p in positions:
        assert p < len(out)
        out[p] ^= 1
    return out


def error_positions(base, count):
    return [base + 1 + 3 * i for i in range(count)]


# YAML.


class U32(int):
    pass


class U64(int):
    pass


class U32s(list):
    pass


class F32s(list):
    pass


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
        return fmt_float(v)
    if isinstance(v, str):
        plain = re.fullmatch(r"[A-Za-z][A-Za-z0-9_ ]*", v)
        if plain and v not in ("true", "false", "null", "yes", "no"):
            return v
        return json.dumps(v)
    raise TypeError(v)


class Doc:
    """The lines of one file."""

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
        if isinstance(v, U32s):
            self.flow(indent, key, v, "!!uint32")
        elif isinstance(v, F32s):
            self.flow(indent, key, v, "!!float32")
        elif isinstance(v, list):
            if all(isinstance(x, str) for x in v):
                items = ", ".join(fmt_scalar(x) for x in v)
                self.add(" " * indent + key + ": [" + items + "]")
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


def record(values=None, soft=False, names=None, meta="none", extents=None, absent=None):
    """A record of a source or a sink. `meta` "none" lists no meta_information, and
    None lists the field as []."""
    r = {}
    if values is not None:
        r["signal_values"] = F32s(values) if soft else list(values)
    if names is not None:
        r["signal_names"] = names
    if extents is not None:
        r["extents"] = extents
    if meta is None:
        r["meta_information"] = []
    elif meta != "none":
        r["meta_information"] = meta
    if absent:
        r["meta_absent"] = absent
    return r


class Graph:
    def __init__(self, file, comment):
        self.file = file
        self.comment = comment
        self.blocks = []
        self.edges = []
        self.notes = {}

    def note(self, name, text):
        """A comment above the block `name` in the written file."""
        self.notes[name] = text

    def block(self, bid, name, params=None):
        assert all(b[1] != name for b in self.blocks), name
        self.blocks.append((bid, name, dict(params or {})))
        return name

    def connect(self, a, b):
        self.edges.append((a, b))

    def source(self, name, kind, records):
        bid = f"gr::blocks::basic::DataSetSource<{kind}>"
        return self.block(bid, name, {"records": records})

    def sink(self, name, kind, records, **params):
        bid = f"gr::blocks::testing::ExpectDataSetSink<{kind}>"
        return self.block(bid, name, {**params, "records": records})

    def chain(self, prefix, kind_in, inputs, stages, kind_out, expected, **sink_params):
        """A source, the stages in order, and a sink. A stage is (block id, name
        suffix, settings)."""
        prev = self.source(prefix + "_source", kind_in, inputs)
        for bid, suffix, params in stages:
            name = self.block(bid, prefix + "_" + suffix, params)
            self.connect(prev, name)
            prev = name
        sink = self.sink(prefix + "_sink", kind_out, expected, **sink_params)
        self.connect(prev, sink)
        return prev

    def text(self):
        doc = Doc()
        for line in self.comment.strip("\n").split("\n"):
            assert len(line) <= WIDTH, line
            doc.add(("# " + line).rstrip())
        body = Doc()
        body.add("blocks:")
        for bid, name, params in self.blocks:
            if name in self.notes:
                for line in self.notes[name].strip("\n").split("\n"):
                    body.add(("  # " + line).rstrip())
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
        for a, b in self.edges:
            body.add(f"  - [{a}, out, {b}, in]")
        lines = doc.lines + body.lines
        assert all(line == line.rstrip() for line in lines), self.file
        return "\n".join(lines) + "\n"


FEC = "gr::blocks::fec::"


def stage(block, suffix, **params):
    return (FEC + block, suffix, params)


# Captures.


def capture(jobs):
    """Run each job, (input kind, records, stages, output kind), through rungraph
    and return the output of each as one flat list."""
    g = Graph("capture", "capture")
    kinds = []
    for i, (kind_in, records, stages, kind_out) in enumerate(jobs):
        prev = g.source(f"c{i}_source", kind_in, records)
        for bid, suffix, params in stages:
            name = g.block(bid, f"c{i}_{suffix}", params)
            g.connect(prev, name)
            prev = name
        flat = g.block(f"gr::blocks::basic::DataSetToStream<{kind_out}>", f"c{i}_flat")
        g.connect(prev, flat)
        file_sink = f"gr::blocks::fileio::BasicFileSink<{kind_out}>"
        g.connect(flat, g.block(file_sink, f"c{i}_file", {"file_name": f"@{i}"}))
        kinds.append(kind_out)
    text = g.text()
    key = hashlib.sha1((Options.digest + text).encode()).hexdigest()[:16]
    stem = os.path.join(Options.cache_dir, key)
    if os.path.exists(stem + ".json"):
        with open(stem + ".json") as f:
            return json.load(f)
    text = re.sub(
        r"file_name: \"@(\d+)\"",
        lambda m: "file_name: " + json.dumps(f"{stem}_{m.group(1)}.bin"),
        text,
    )
    with open(stem + ".yaml", "w") as f:
        f.write(text)
    run = subprocess.run(
        [Options.rungraph, "--graph", stem + ".yaml"],
        capture_output=True,
        text=True,
        env=Options.environment,
        timeout=60,
    )
    if run.returncode != 0:
        sys.exit(f"capture {stem}.yaml failed:\n{run.stdout}{run.stderr}")
    results = []
    for i, kind_out in enumerate(kinds):
        with open(f"{stem}_{i}.bin", "rb") as f:
            raw = f.read()
        if kind_out == "uint8":
            results.append(list(raw))
        else:
            results.append([v for (v,) in struct.iter_unpack("f", raw)])
    with open(stem + ".json", "w") as f:
        json.dump(results, f)
    return results


def encoded(code, info):
    stages = [stage("FecEncode", "coder", code=code)]
    return capture([("uint8", [record(info)], stages, "uint8")])[0]


def rs_encoded(info, **settings):
    stages = [stage("RsEncode", "coder", **settings)]
    return capture([("uint8", [record(info)], stages, "uint8")])[0]


def settings_graph(file, comment, chains):
    """Chains of a source without records, one block and a sink expecting none, for
    SET cases to change."""
    g = Graph(file, comment)
    for name, kind_in, block, kind_out, params in chains:
        g.chain(name, kind_in, [], [(FEC + block, "block", params)], kind_out, [])
    return g


# The binary codes.

FORMS = [  # code, n, k, t, refuses past the radius
    ("golay24", 24, 12, 3, True),
    ("golay23", 23, 12, 3, False),
    ("golay18", 18, 6, 3, True),
    ("hamming15", 15, 11, 1, False),
    ("hamming10", 10, 6, 1, True),
    ("bch63", 63, 16, 11, True),
]
BCH15 = [  # code, k, t, generator
    ("bch15_11", 11, 1, 0x0013),
    ("bch15_7", 7, 2, 0x01D1),
    ("bch15_5", 5, 3, 0x0537),
]
ALL_FORMS = FORMS + [(c, 15, k, t, None) for c, k, t, _ in BCH15]
ANCHORS = [
    ("golay24", 0x0018EB, 0x001),
    ("golay23", 0x000C75, 0x001),
    ("golay18", 0x0018EB, 0x01),
    ("hamming15", 0x0013, 0x001),
    ("hamming10", 0x001C, 0x01),
    ("bch63", 0xCD930BDD3B2B, 0x0001),
] + [(c, g, 0x1) for c, _, _, g in BCH15]
FORM = {f[0]: f for f in ALL_FORMS}


def meta_counts(corrected, uncorrectable, **more):
    return {
        "corrected_errors": U32(corrected),
        "uncorrectable_errors": U32(uncorrectable),
        **more,
    }


def fec_round_trip():
    g = Graph(
        "fec-binary-round-trip.yaml",
        """
FecEncode into FecDecode, one chain per binary code. Each source record carries four information words of seeded
bits, three for the length-15 BCH codes. The first sink checks that the encoder emits one record of that many
codewords. The second sink expects the information back with no corrected and no uncorrectable codeword.
Written by generate_graphs.py from seed 0x9E3779B97F4A7C15. Oracle: the drawn information.
""",
    )
    rng = Lcg(SEED)
    for code, n, k, _, _ in ALL_FORMS:
        words = 3 if code.startswith("bch15") else 4
        info = rng.bits(words * k)
        src = g.source(code + "_source", "uint8", [record(info)])
        coder = g.block(FEC + "FecEncode", code + "_coder", {"code": code})
        decoder = g.block(FEC + "FecDecode", code + "_decoder", {"code": code})
        coded = g.sink(code + "_coded", "uint8", [record(extents=[words * n])])
        back = record(info, extents=[words * k], meta=meta_counts(0, 0))
        sink = g.sink(code + "_sink", "uint8", [back])
        g.connect(src, coder)
        g.connect(coder, coded)
        g.connect(coder, decoder)
        g.connect(decoder, sink)
    return g


def fec_anchors():
    g = Graph(
        "fec-binary-anchors.yaml",
        """
Known codewords of each binary code against FecEncode and FecDecode. The information word is 1 for every code, the
most significant bit first. The golay24, golay23, golay18, hamming15, hamming10 and bch63 codewords are the values
the kernels' tests pin. For the length-15 BCH codes the codeword is the generator polynomial. Each code has two
chains: the encoder turns the information bits into the codeword's bits, and the decoder turns the codeword's bits
back with nothing corrected.
Written by generate_graphs.py with no seed. Oracle: the published codewords.
""",
    )
    for code, word, info in ANCHORS:
        _, n, k, _, _ = FORM[code]
        cw, iw = bits_of(word, n), bits_of(info, k)
        g.chain(
            code + "_encode",
            "uint8",
            [record(iw)],
            [stage("FecEncode", "coder", code=code)],
            "uint8",
            [record(cw)],
        )
        g.chain(
            code + "_decode",
            "uint8",
            [record(cw)],
            [stage("FecDecode", "decoder", code=code)],
            "uint8",
            [record(iw, meta=meta_counts(0, 0))],
        )
    return g


def fec_errors():
    g = Graph(
        "fec-binary-errors.yaml",
        """
FecDecode on damaged codewords. Each source record is FecEncode's output for seeded information with bits flipped.
The inside chains flip t bits in each of three codewords, at 1, 4, 7 and on from each codeword's start, where t is
the code's radius. Their sinks expect the information back and 3t corrected errors. The beyond chains flip t + 1
bits of the first codeword, and bits 7 and 9 for hamming10. Their sinks expect the information length and one
uncorrectable codeword for the codes that detect it. golay23, hamming15 and bch15_11 are perfect codes and report
none. For bch15_7 and bch15_5 the count follows from the code's full codeword set: the received word is
uncorrectable when no codeword lies within t bits of it. The crossed chain encodes the information words 1 to 8
under bch15_11 and decodes them under hamming15. That sink expects the miscorrected bits the decoder returns, and no
uncorrectable codeword.
Written by generate_graphs.py from seed 0x9E3779B97F4A7C15 ^ 0x1. Oracle: the drawn information. The crossed sink
holds FecDecode's output for its input, captured by the generator.
""",
    )
    rng = Lcg(SEED ^ 0x1)
    for code, n, k, t, _ in ALL_FORMS:
        info = rng.bits(3 * k)
        damaged = encoded(code, info)
        for w in range(3):
            damaged = flip(damaged, error_positions(w * n, t))
        g.chain(
            code + "_inside",
            "uint8",
            [record(damaged)],
            [stage("FecDecode", "decoder", code=code)],
            "uint8",
            [record(info, meta=meta_counts(3 * t, 0))],
        )
    for code, n, k, t, refuses in ALL_FORMS:
        bch15 = code.startswith("bch15")
        words = 3 if bch15 else 2
        info = rng.bits(words * k)
        coded = encoded(code, info)
        positions = [7, 9] if code == "hamming10" else error_positions(0, t + 1)
        damaged = flip(coded, positions)
        if bch15:
            generator = [b[3] for b in BCH15 if b[0] == code][0]
            first = int("".join(str(b) for b in damaged[:15]), 2)
            reachable = any(
                bin(c ^ first).count("1") <= t for c in bch15_codewords(generator, k)
            )
            refuses = not reachable
            assert refuses == (code == "bch15_5"), code
        expected = 1 if refuses else 0
        g.chain(
            code + "_beyond",
            "uint8",
            [record(damaged)],
            [stage("FecDecode", "decoder", code=code)],
            "uint8",
            [record(extents=[words * k], meta={"uncorrectable_errors": U32(expected)})],
        )
    info = [b for w in range(1, 9) for b in bits_of(w, 11)]
    stages = [
        stage("FecEncode", "coder", code="bch15_11"),
        stage("FecDecode", "decoder", code="hamming15"),
    ]
    crossed = capture([("uint8", [record(info)], stages, "uint8")])[0]
    wrong = sum(
        1
        for w in range(8)
        if crossed[11 * w : 11 * w + 11] != info[11 * w : 11 * w + 11]
    )
    assert len(crossed) == 88 and wrong > 0
    g.chain(
        "crossed",
        "uint8",
        [record(info)],
        stages,
        "uint8",
        [record(crossed, meta={"uncorrectable_errors": U32(0)})],
    )
    return g


def fec_records():
    g = Graph(
        "fec-binary-records.yaml",
        """
FecEncode and FecDecode with a record's metadata and with the records they refuse. The carried chain
decodes two golay24 codewords with one flipped bit each. The record already counts 3 corrected errors, so the sink
expects 5, and the other keys and the signal name unchanged. The bare chain decodes a record without a metadata map,
and the sink expects one map holding the two counts. The ordered chains carry three records with a `seq` key. The
encoder emits them in order, and the decoder, given each codeword record with bits 1 and 26 flipped, returns each
record's information with 2 corrected errors. The misaligned chains give each encoder a record that is not a whole
number of information words, a good record and an empty record. The truncated chain gives the decoder a codeword
record one bit short, then the whole record. Each sink expects the good record alone, and the block's line at stop
counts the records refused.
Written by generate_graphs.py from seed 0x9E3779B97F4A7C15 ^ 0x2. Oracle: the drawn information and the counts the
text gives.
""",
    )
    rng = Lcg(SEED ^ 0x2)
    info = rng.bits(24)
    damaged = flip(encoded("golay24", info), [1, 25])
    carried = {
        "corrected_errors": U32(3),
        "provenance": "earlier stage",
        "sample_start": U64(4096),
    }
    counted = meta_counts(5, 0, provenance="earlier stage", sample_start=U64(4096))
    g.chain(
        "carried",
        "uint8",
        [record(damaged, names=["fec"], meta=carried)],
        [stage("FecDecode", "decoder", code="golay24")],
        "uint8",
        [record(info, names=["fec"], meta=counted)],
        meta_exact=True,
    )
    info = rng.bits(11)
    g.chain(
        "bare",
        "uint8",
        [record(encoded("hamming15", info))],
        [stage("FecDecode", "decoder", code="hamming15")],
        "uint8",
        [record(info, meta=meta_counts(0, 0))],
        meta_exact=True,
    )
    infos = [rng.bits(24) for _ in range(3)]
    seq = [{"seq": U32(r)} for r in range(3)]
    g.chain(
        "ordered_encode",
        "uint8",
        [record(infos[r], meta=seq[r]) for r in range(3)],
        [stage("FecEncode", "coder", code="golay24")],
        "uint8",
        [record(extents=[48], meta=seq[r]) for r in range(3)],
    )
    g.chain(
        "ordered_decode",
        "uint8",
        [
            record(flip(encoded("golay24", infos[r]), [1, 26]), meta=seq[r])
            for r in range(3)
        ],
        [stage("FecDecode", "decoder", code="golay24")],
        "uint8",
        [record(infos[r], meta=meta_counts(2, 0, seq=U32(r))) for r in range(3)],
    )
    for code, bad, good, n in (("golay24", 13, 24, 24), ("bch15_11", 4, 22, 15)):
        k = FORM[code][2]
        g.chain(
            "misaligned_" + code,
            "uint8",
            [record(rng.bits(bad)), record(rng.bits(good)), record([])],
            [stage("FecEncode", "coder", code=code)],
            "uint8",
            [record(extents=[good // k * n])],
        )
    info = rng.bits(12)
    coded = encoded("golay24", info)
    g.chain(
        "truncated",
        "uint8",
        [record(coded[:-1]), record(coded)],
        [stage("FecDecode", "decoder", code="golay24")],
        "uint8",
        [record(info)],
    )
    return g


# The Reed-Solomon codes.


def damage_symbols(values, base, count):
    out = list(values)
    for i in range(count):
        p = base + 1 + 3 * i
        out[p] = (out[p] ^ 0x15) & 0x3F
    return out


ARMS = [  # code, E, basis, depth, pad
    ("ccsds_255_223", 16, "conventional", 4, 200),
    ("ccsds_255_223", 16, "dual", 1, 0),
    ("ccsds_255_239", 8, "conventional", 2, 200),
]


def arm_name(code, basis, depth, pad):
    return f"{code[6:]}_{basis}_{depth}_{pad}"


def arm_settings(code, basis, depth, pad):
    return {"code": code, "basis": basis, "interleave": U32(depth), "pad": U32(pad)}


def rs_round_trip():
    g = Graph(
        "fec-rs-round-trip.yaml",
        """
RsEncode into RsDecode. The gf64 chains carry two information words of seeded symbols under each root count, then
at a shortening of 39, 39 and 27 symbols. The first sink of each checks the encoder's record length, the second the
symbols back with nothing corrected. The decoder's line at stop of a shortened chain names no pad corruption. The
ccsds chains do the same under each CCSDS profile: the long code in both bases, at depths 4 and 1 and at a
shortening of 200 or none, and the short code at depth 2. The parity chain encodes twelve symbols under 12 roots and
a shortening of 39. Its sink expects the information unchanged, then the parity the kernel's test pins. The depth
chains encode the same symbols without an interleave setting and at depth 1, and both sinks expect the record the
encoder emits without the setting. The basis chain encodes in the dual basis and decodes in the conventional basis,
and its sink expects the codeword counted uncorrectable.
Written by generate_graphs.py from seed 0x9E3779B97F4A7C15 ^ 0x3. Oracle: the drawn symbols and the pinned parity.
The depth sinks hold RsEncode's output without the interleave setting, captured by the generator.
""",
    )
    rng = Lcg(SEED ^ 0x3)
    for roots, pad in ((8, 0), (12, 0), (16, 0), (8, 39), (12, 39), (16, 27)):
        info = 63 - roots - pad
        symbols = rng.symbols(2 * info)
        name = f"gf64_{roots}_{pad}"
        settings = {"roots": U32(roots)}
        if pad:
            settings["pad"] = U32(pad)
        src = g.source(name + "_source", "uint8", [record(symbols)])
        coder = g.block(FEC + "RsEncode", name + "_coder", settings)
        decoder = g.block(FEC + "RsDecode", name + "_decoder", settings)
        coded = [record(extents=[2 * (63 - pad)])]
        g.connect(src, coder)
        g.connect(coder, g.sink(name + "_coded", "uint8", coded))
        g.connect(coder, decoder)
        back = [record(symbols, meta=meta_counts(0, 0))]
        g.connect(decoder, g.sink(name + "_sink", "uint8", back))
    for code, e, basis, depth, pad in ARMS:
        info = (255 - 2 * e - pad) * depth
        symbols = rng.symbols(info, 0xFF)
        name = "ccsds_" + arm_name(code, basis, depth, pad)
        settings = arm_settings(code, basis, depth, pad)
        src = g.source(name + "_source", "uint8", [record(symbols)])
        coder = g.block(FEC + "RsEncode", name + "_coder", settings)
        decoder = g.block(FEC + "RsDecode", name + "_decoder", settings)
        coded = [record(extents=[(255 - pad) * depth])]
        g.connect(src, coder)
        g.connect(coder, g.sink(name + "_coded", "uint8", coded))
        g.connect(coder, decoder)
        back = [record(symbols, meta=meta_counts(0, 0))]
        g.connect(decoder, g.sink(name + "_sink", "uint8", back))
    parity = [0x1A, 0x3F, 0x3A, 0x3A, 0x0D, 0x03, 0x05, 0x21, 0x1F, 0x1D, 0x17, 0x19]
    symbols = [(7 * i + 5) & 0x3F for i in range(12)]
    g.chain(
        "parity",
        "uint8",
        [record(symbols)],
        [stage("RsEncode", "coder", roots=U32(12), pad=U32(39))],
        "uint8",
        [record(symbols + parity, extents=[24])],
    )
    symbols = rng.symbols(110)
    absent = rs_encoded(symbols, roots=U32(8))
    for name, settings in (
        ("depth_absent", {"roots": U32(8)}),
        ("depth_1", {"roots": U32(8), "interleave": U32(1)}),
    ):
        g.chain(
            name,
            "uint8",
            [record(symbols)],
            [stage("RsEncode", "coder", **settings)],
            "uint8",
            [record(absent)],
        )
    symbols = rng.symbols(223, 0xFF)
    plain = rs_encoded(symbols, code="ccsds_255_223", basis="conventional")
    dual = rs_encoded(symbols, code="ccsds_255_223", basis="dual")
    assert plain != dual and len(plain) == len(dual) == 255
    g.chain(
        "basis",
        "uint8",
        [record(symbols)],
        [
            stage("RsEncode", "coder", code="ccsds_255_223", basis="dual"),
            stage("RsDecode", "decoder", code="ccsds_255_223", basis="conventional"),
        ],
        "uint8",
        [record(meta={"uncorrectable_errors": U32(1)})],
    )
    return g


def rs_errors():
    g = Graph(
        "fec-rs-errors.yaml",
        """
RsDecode on damaged records. Each source record is RsEncode's output for seeded symbols with symbols changed. The
inside chains change t symbols in each of two gf64 codewords, at 1, 4, 7 and on from each codeword's start, by an
exclusive or with 0x15. t is half the root count. Their sinks expect the symbols back and 2t corrected errors. The
beyond chain changes five symbols of one codeword under 8 roots, one past the radius. Its sink expects the
information length and one uncorrectable codeword. The burst chains flip bit 0 of E * I consecutive symbols of a
CCSDS codeblock from symbol 5, where E is the profile's radius and I the depth, so each codeword takes E errors.
Their sinks expect the symbols back and E * I corrected errors. The pad chain decodes the last 24 symbols of a
12-root gf64 codeword whose symbol 38 is 0x2A, under a shortening of 39. Its sink expects one corrected error and
one uncorrectable codeword, and the decoder's line at stop counts the pad corruption. The truncated chain gives the
decoder a codeword record one symbol short, then the whole record, and the decoder's line counts the refusal.
Written by generate_graphs.py from seed 0x9E3779B97F4A7C15 ^ 0x4. Oracle: the drawn symbols and the counts the
text gives.
""",
    )
    rng = Lcg(SEED ^ 0x4)
    for roots in (8, 12, 16):
        info = 63 - roots
        t = roots // 2
        symbols = rng.symbols(2 * info)
        coded = rs_encoded(symbols, roots=U32(roots))
        damaged = damage_symbols(damage_symbols(coded, 0, t), 63, t)
        g.chain(
            f"inside_{roots}",
            "uint8",
            [record(damaged)],
            [stage("RsDecode", "decoder", roots=U32(roots))],
            "uint8",
            [record(symbols, meta=meta_counts(2 * t, 0))],
        )
    symbols = rng.symbols(55)
    g.chain(
        "beyond",
        "uint8",
        [record(damage_symbols(rs_encoded(symbols, roots=U32(8)), 0, 5))],
        [stage("RsDecode", "decoder", roots=U32(8))],
        "uint8",
        [record(extents=[55], meta={"uncorrectable_errors": U32(1)})],
    )
    for code, e, basis, depth, pad in ARMS:
        settings = arm_settings(code, basis, depth, pad)
        symbols = rng.symbols((255 - 2 * e - pad) * depth, 0xFF)
        burst = [5 + j for j in range(e * depth)]
        damaged = flip(rs_encoded(symbols, **settings), burst)
        g.chain(
            "burst_" + arm_name(code, basis, depth, pad),
            "uint8",
            [record(damaged)],
            [stage("RsDecode", "decoder", **settings)],
            "uint8",
            [record(symbols, meta=meta_counts(e * depth, 0))],
        )
    full = [0] * 51
    full[38] = 0x2A
    for i in range(39, 51):
        full[i] = (5 * i + 3) & 0x3F
    word = rs_encoded(full, roots=U32(12))
    assert word[:51] == full and len(word) == 63
    g.chain(
        "pad",
        "uint8",
        [record(word[39:])],
        [stage("RsDecode", "decoder", roots=U32(12), pad=U32(39))],
        "uint8",
        [record(meta=meta_counts(1, 1))],
    )
    symbols = rng.symbols(51)
    coded = rs_encoded(symbols, roots=U32(12))
    g.chain(
        "truncated",
        "uint8",
        [record(coded[:-1]), record(coded)],
        [stage("RsDecode", "decoder", roots=U32(12))],
        "uint8",
        [record(symbols)],
    )
    return g


def fec_settings():
    return settings_graph(
        "fec-settings.yaml",
        """
One chain per binary and Reed-Solomon block, each with settings the block accepts and no record to process. The run
passes as written. A case that changes a setting through SET checks that the block accepts the change or refuses it,
and the refusal ends the run with the block's reason.
Written by generate_graphs.py with no seed and no oracle: the fec chains hold no record.
""",
        [
            ("encoder", "uint8", "FecEncode", "uint8", {"code": "golay24"}),
            ("decoder", "uint8", "FecDecode", "uint8", {"code": "golay24"}),
            ("rs_encoder", "uint8", "RsEncode", "uint8", {"roots": U32(8)}),
            ("rs_decoder", "uint8", "RsDecode", "uint8", {"roots": U32(8)}),
            (
                "ccsds_encoder",
                "uint8",
                "RsEncode",
                "uint8",
                {"code": "ccsds_255_223", "basis": "dual"},
            ),
            (
                "ccsds_decoder",
                "uint8",
                "RsDecode",
                "uint8",
                {"code": "ccsds_255_239", "basis": "conventional"},
            ),
        ],
    )


# The convolutional code.


def conv_encode(info, k, polys, invert=0):
    """Terminated feed-forward encode: K-1 zero tail bits, and bit 0 of the
    register is the current input bit."""
    reg, out = 0, []
    for b in list(info) + [0] * (k - 1):
        reg = ((reg << 1) | b) & ((1 << k) - 1)
        for j, g in enumerate(polys):
            out.append((bin(reg & g).count("1") & 1) ^ ((invert >> j) & 1))
    return out


CLASSIC = (7, [0o171, 0o133])
SHORTEST = (3, [0o7, 0o5])
CCSDS = (7, [0o117, 0o155])


def classic(info, invert=0):
    return conv_encode(info, CLASSIC[0], CLASSIC[1], invert)


def as_soft(bits):
    return [1.0 if b & 1 else -1.0 for b in bits]


def classic_settings(**more):
    return {"constraint_length": U32(7), "polynomials": U32s(CLASSIC[1]), **more}


def shortest_settings():
    return {"constraint_length": U32(3), "polynomials": U32s(SHORTEST[1])}


def conv_encode_graph():
    g = Graph(
        "conv-encode.yaml",
        """
ConvEncode's output order. The impulse chains encode a one followed by K - 1 zeros under the classic K = 7 code
(171, 133 octal) and the K = 3 code (7, 5 octal). The sinks expect each generator polynomial, least significant bit
first, interleaved one bit per polynomial per step, then zeros once the impulse has left the register. The
convention chains encode seeded bits under 'ccsds_uninverted' and 'ccsds', the polynomials 117 and 155 octal. The
'ccsds' sink expects the 'ccsds_uninverted' bits with every second coded bit complemented. The inverted chain sets
invert_outputs to 1 under the classic code. Its first sink expects the classic encoding with every first coded bit
complemented, and its second sink expects the information back from ViterbiDecode under the same setting, with
nothing corrected.
Written by generate_graphs.py from seed 0x9E3779B97F4A7C15 ^ 0x5. Oracle: the generator's convolutional encoder.
""",
    )
    for name, (k, polys) in (("impulse_k7", CLASSIC), ("impulse_k3", SHORTEST)):
        impulse = [1] + [0] * (k - 1)
        expected = []
        for step in range(2 * k - 1):
            expected += [(p >> step) & 1 if step < k else 0 for p in polys]
        assert expected == conv_encode(impulse, k, polys)
        settings = {"constraint_length": U32(k), "polynomials": U32s(polys)}
        g.chain(
            name,
            "uint8",
            [record(impulse)],
            [stage("ConvEncode", "coder", **settings)],
            "uint8",
            [record(expected)],
        )
    rng = Lcg(SEED ^ 0x5)
    info = rng.bits(96)
    uninverted = conv_encode(info, *CCSDS)
    inverted = [b ^ (i & 1) for i, b in enumerate(uninverted)]
    for name, expected in (("ccsds_uninverted", uninverted), ("ccsds", inverted)):
        g.chain(
            name,
            "uint8",
            [record(info)],
            [stage("ConvEncode", "coder", code=name)],
            "uint8",
            [record(expected)],
        )
    info = rng.bits(80)
    flipped = [b ^ (1 - (i & 1)) for i, b in enumerate(classic(info))]
    assert flipped == classic(info, 1)
    settings = classic_settings(invert_outputs=U32(1))
    src = g.source("inverted_source", "uint8", [record(info)])
    coder = g.block(FEC + "ConvEncode", "inverted_coder", settings)
    decoder = g.block(FEC + "ViterbiDecode", "inverted_decoder", settings)
    back = [record(info, meta={"corrected_errors": U32(0)})]
    g.connect(src, coder)
    g.connect(coder, g.sink("inverted_coded", "uint8", [record(flipped)]))
    g.connect(coder, decoder)
    g.connect(decoder, g.sink("inverted_sink", "uint8", back))
    return g


def conv_decode_graph():
    g = Graph(
        "conv-decode.yaml",
        """
ViterbiDecode and ViterbiDecodeSoft under the classic K = 7 code. The hard chain encodes 100 seeded bits. Its first
sink expects the 212 coded bits, two per step over 106 steps, and its second the information back with nothing
corrected and no uncorrectable_errors key. The soft chains decode the coded bits as soft values, a one at +1.0 and a
zero at -1.0. The plain soft chain expects the information back with nothing corrected. The damaged soft chain
negates the values at 11 and 93 and erases 40 to 43 to 0.0, and expects the information back. The open chains
decode under termination 'open', where every step carries an information bit: the sinks expect the 120 information
bits, then the six tail steps as zeros. The step chain decodes the first two coded bits, one step, under 'open'.
Written by generate_graphs.py from seed 0x9E3779B97F4A7C15 ^ 0x6. Oracle: the drawn information and the generator's
convolutional encoder.
""",
    )
    rng = Lcg(SEED ^ 0x6)
    info = rng.bits(100)
    coded = classic(info)
    assert len(coded) == 212
    clean = {"corrected_errors": U32(0)}
    back = [record(info, meta=clean, absent=["uncorrectable_errors"])]
    src = g.source("hard_source", "uint8", [record(info)])
    coder = g.block(FEC + "ConvEncode", "hard_coder", classic_settings())
    decoder = g.block(FEC + "ViterbiDecode", "hard_decoder", classic_settings())
    g.connect(src, coder)
    g.connect(coder, g.sink("hard_coded", "uint8", [record(coded, extents=[212])]))
    g.connect(coder, decoder)
    g.connect(decoder, g.sink("hard_sink", "uint8", back))
    soft = as_soft(coded)
    g.chain(
        "soft",
        "float32",
        [record(soft, soft=True)],
        [stage("ViterbiDecodeSoft", "decoder", **classic_settings())],
        "uint8",
        [record(info, meta=clean)],
    )
    damaged = list(soft)
    damaged[11], damaged[93] = -damaged[11], -damaged[93]
    for p in (40, 41, 42, 43):
        damaged[p] = 0.0
    g.chain(
        "soft_damaged",
        "float32",
        [record(damaged, soft=True)],
        [stage("ViterbiDecodeSoft", "decoder", **classic_settings())],
        "uint8",
        [record(info)],
    )
    info = rng.bits(120)
    open_ = classic_settings(termination="open")
    g.chain(
        "open",
        "uint8",
        [record(info)],
        [
            stage("ConvEncode", "coder", **classic_settings()),
            stage("ViterbiDecode", "decoder", **open_),
        ],
        "uint8",
        [record(info + [0] * 6, extents=[126])],
    )
    g.chain(
        "open_soft",
        "float32",
        [record(as_soft(classic(info)), soft=True)],
        [stage("ViterbiDecodeSoft", "decoder", **open_)],
        "uint8",
        [record(info + [0] * 6, extents=[126])],
    )
    g.chain(
        "open_step",
        "uint8",
        [record(classic(info)[:2])],
        [stage("ViterbiDecode", "decoder", **open_)],
        "uint8",
        [record(extents=[1])],
    )
    return g


def conv_records_graph():
    g = Graph(
        "conv-records.yaml",
        """
ConvEncode, ViterbiDecode and ViterbiDecodeSoft with a record's metadata and with the records they refuse, under the
classic K = 7 code. The carried chain decodes a frame of 100 seeded bits with coded bits 9 and 120 flipped. The
record already counts 3 corrected errors, so the sink expects 5, and the other keys and the signal name unchanged.
The bare chain decodes a frame without a metadata map, and the sink expects one map holding the count. The ordered
chains carry three frames with a `seq` key. The encoder emits them in order, and the decoder, given each coded frame
with bits 5, 61 and 150 flipped, returns each frame's information with 3 corrected errors. The refused chains give
the encoder an empty record, the hard decoder a coded frame one bit short, the 12 zero bits of a tail alone and the
whole frame, the soft decoder a soft frame one value short and the whole frame, and the terminated decoder one step.
Each sink expects the whole frame alone, and the block's line at stop counts the records refused.
Written by generate_graphs.py from seed 0x9E3779B97F4A7C15 ^ 0x7. Oracle: the drawn information and the generator's
convolutional encoder.
""",
    )
    rng = Lcg(SEED ^ 0x7)
    info = rng.bits(100)
    carried = {
        "corrected_errors": U32(3),
        "provenance": "earlier stage",
        "sample_start": U64(4096),
    }
    g.chain(
        "carried",
        "uint8",
        [record(flip(classic(info), [9, 120]), names=["fec"], meta=carried)],
        [stage("ViterbiDecode", "decoder", **classic_settings())],
        "uint8",
        [record(info, names=["fec"], meta={**carried, "corrected_errors": U32(5)})],
        meta_exact=True,
    )
    info = rng.bits(20)
    g.chain(
        "bare",
        "uint8",
        [record(classic(info))],
        [stage("ViterbiDecode", "decoder", **classic_settings())],
        "uint8",
        [record(info, meta={"corrected_errors": U32(0)})],
        meta_exact=True,
    )
    infos = [rng.bits(100) for _ in range(3)]
    seq = [{"seq": U32(r)} for r in range(3)]
    g.chain(
        "ordered_encode",
        "uint8",
        [record(infos[r], meta=seq[r]) for r in range(3)],
        [stage("ConvEncode", "coder", **classic_settings())],
        "uint8",
        [record(extents=[212], meta=seq[r]) for r in range(3)],
    )
    g.chain(
        "ordered_decode",
        "uint8",
        [record(flip(classic(infos[r]), [5, 61, 150]), meta=seq[r]) for r in range(3)],
        [stage("ViterbiDecode", "decoder", **classic_settings())],
        "uint8",
        [
            record(infos[r], meta={"seq": U32(r), "corrected_errors": U32(3)})
            for r in range(3)
        ],
    )
    good = rng.bits(50)
    g.chain(
        "refused_encode",
        "uint8",
        [record([]), record(good)],
        [stage("ConvEncode", "coder", **classic_settings())],
        "uint8",
        [record(classic(good))],
    )
    info = rng.bits(20)
    coded = classic(info)
    g.chain(
        "refused_hard",
        "uint8",
        [record(coded[:-1]), record([0] * 12), record(coded)],
        [stage("ViterbiDecode", "decoder", **classic_settings())],
        "uint8",
        [record(info)],
    )
    info = rng.bits(20)
    soft = as_soft(classic(info))
    g.chain(
        "refused_soft",
        "float32",
        [record(soft[:-1], soft=True), record(soft, soft=True)],
        [stage("ViterbiDecodeSoft", "decoder", **classic_settings())],
        "uint8",
        [record(info)],
    )
    g.chain(
        "refused_step",
        "uint8",
        [record(coded[:2])],
        [stage("ViterbiDecode", "decoder", **classic_settings())],
        "uint8",
        [],
    )
    return g


def conv_settings():
    return settings_graph(
        "conv-settings.yaml",
        """
One chain per convolutional block, each with settings the block accepts and no record to process: the classic
K = 7 code for the encoder and both decoders, and the 'ccsds' convention for a second encoder. The run passes as
written. A case that changes a setting through SET checks that the block accepts the change or refuses it, and the
refusal ends the run with the block's reason.
Written by generate_graphs.py with no seed and no oracle: the convolutional chains hold no record.
""",
        [
            ("encoder", "uint8", "ConvEncode", "uint8", classic_settings()),
            ("decoder", "uint8", "ViterbiDecode", "uint8", classic_settings()),
            (
                "soft_decoder",
                "float32",
                "ViterbiDecodeSoft",
                "uint8",
                classic_settings(),
            ),
            ("ccsds_encoder", "uint8", "ConvEncode", "uint8", {"code": "ccsds"}),
        ],
    )


# Puncturing.

PUNCTURE_SEED = 0xD1B54A32D192ED03


def punctured(coded, mask):
    return [c for i, c in enumerate(coded) if mask[i % len(mask)] == "1"]


def depunctured(kept, mask):
    ones = mask.count("1")
    assert len(kept) % ones == 0
    out, it = [], iter(kept)
    for i in range(len(kept) // ones * len(mask)):
        out.append(next(it) if mask[i % len(mask)] == "1" else 0.0)
    return out


def erased_ones(coded, mask):
    return sum(1 for i, c in enumerate(coded) if mask[i % len(mask)] == "0" and c & 1)


class Awgn:
    """A xorshift and Box-Muller source, which keeps the second deviate of each
    pair."""

    def __init__(self, seed):
        self.state = (seed | 1) & M64
        self.spare = None

    def uniform(self):
        self.state ^= (self.state << 13) & M64
        self.state ^= self.state >> 7
        self.state ^= (self.state << 17) & M64
        return ((self.state >> 11) + 1) * 2.0**-53

    def normal(self):
        if self.spare is not None:
            value, self.spare = self.spare, None
            return value
        radius = math.sqrt(-2.0 * math.log(self.uniform()))
        angle = 2.0 * math.pi * self.uniform()
        self.spare = radius * math.sin(angle)
        return radius * math.cos(angle)


def channel(bits, index, sigma):
    """The soft values of `bits` for record `index` with seeded noise added, and the
    count of values whose sign the noise changed."""
    noise = Awgn((0x2545F4914F6CDD1D + index * 0x9E3779B97F4A7C15) & M64)
    values, flips = [], 0
    for clean in as_soft(bits):
        value = f32(clean + noise.normal() * sigma)
        flips += (clean > 0.0) != (value > 0.0)
        values.append(value)
    return values, flips


def puncture_patterns_graph():
    g = Graph(
        "puncture-patterns.yaml",
        """
Puncture and Depuncture against positions counted by hand. The rate 2/3 chain encodes two seeded bits under the
K = 3 code (7, 5 octal) into eight coded bits. Its first sink expects those bits, and its second the bits at 0, 1,
2, 4, 5 and 6 that '1110' keeps. The rate 3/4 chain encodes four bits into twelve, and '111001' keeps 0, 1, 2, 5,
6, 7, 8 and 11. The phase chain punctures two identical records of ten bits, and both records keep 0, 1, 2, 4, 5,
6, 8 and 9: the pattern starts over at every record. The erasure chain depunctures the six kept soft values of the
rate 2/3 chain, and the sink expects them back in place with 0.0 at 3 and 7.
Written by generate_graphs.py from seed 0xD1B54A32D192ED03. Oracle: the positions counted by hand and the
generator's convolutional encoder.
""",
    )
    rng = Lcg(PUNCTURE_SEED)
    for name, mask, count, kept in (
        ("two_thirds", "1110", 2, [0, 1, 2, 4, 5, 6]),
        ("three_quarters", "111001", 4, [0, 1, 2, 5, 6, 7, 8, 11]),
    ):
        info = rng.bits(count)
        coded = conv_encode(info, *SHORTEST)
        assert punctured(coded, mask) == [coded[p] for p in kept]
        src = g.source(name + "_source", "uint8", [record(info)])
        coder = g.block(FEC + "ConvEncode", name + "_coder", shortest_settings())
        puncture = g.block(FEC + "Puncture", name + "_puncture", {"pattern": mask})
        g.connect(src, coder)
        g.connect(coder, g.sink(name + "_coded", "uint8", [record(coded)]))
        g.connect(coder, puncture)
        kept_bits = [record([coded[p] for p in kept])]
        g.connect(puncture, g.sink(name + "_sink", "uint8", kept_bits))
        if name == "two_thirds":
            erasure_in = as_soft([coded[p] for p in kept])
            erasure_out = [
                0.0 if i % 4 == 3 else v for i, v in enumerate(as_soft(coded))
            ]
            assert depunctured(erasure_in, mask) == erasure_out
    bits = rng.bits(10)
    phase = record([bits[p] for p in (0, 1, 2, 4, 5, 6, 8, 9)])
    g.chain(
        "phase",
        "uint8",
        [record(bits), record(bits)],
        [stage("Puncture", "puncture", pattern="1110")],
        "uint8",
        [phase, phase],
    )
    g.chain(
        "erasure",
        "float32",
        [record(erasure_in, soft=True)],
        [stage("Depuncture", "depuncture", pattern="1110")],
        "float32",
        [record(erasure_out, soft=True)],
    )
    return g


def puncture_chain_graph():
    g = Graph(
        "puncture-chain.yaml",
        """
The punctured chain under the classic K = 7 code, at both published rates. Three frames of 30 seeded bits carry a
`seq` key, and each terminates into 72 coded bits, a whole number of periods of both patterns. The encode chains
run ConvEncode into Puncture, and the sinks expect the bits the pattern keeps. The decode chains start from those
bits as soft values, a one at +1.0 and a zero at -1.0, and run Depuncture into ViterbiDecodeSoft. The sinks expect
each frame's information back in order. The corrected count equals the deleted positions that carried a one, since
an erasure decides to zero. The noisy chain's source holds two rate 2/3 frames, each the punctured coding of 96
seeded bits, as soft values with seeded Gaussian noise of standard deviation 0.4 added. Its sink expects the
information back, and a corrected count of those deleted ones plus the values whose sign the noise changed.
Written by generate_graphs.py from seed 0xD1B54A32D192ED03 ^ 0x1, and ^ 0x3 for the noisy chain's information. The
noise of frame i has the seed 0x2545F4914F6CDD1D plus i times 0x9E3779B97F4A7C15. Oracle: the generator's
convolutional encoder.
""",
    )
    rng = Lcg(PUNCTURE_SEED ^ 0x1)
    for name, mask in (("two_thirds", "1110"), ("three_quarters", "111001")):
        infos = [rng.bits(30) for _ in range(3)]
        coded = [classic(i) for i in infos]
        kept = [punctured(c, mask) for c in coded]
        assert all(len(c) % 12 == 0 and erased_ones(c, mask) > 0 for c in coded)
        seq = [{"seq": U32(f)} for f in range(3)]
        g.chain(
            name + "_encode",
            "uint8",
            [record(infos[f], meta=seq[f]) for f in range(3)],
            [
                stage("ConvEncode", "coder", **classic_settings()),
                stage("Puncture", "puncture", pattern=mask),
            ],
            "uint8",
            [record(kept[f], meta=seq[f]) for f in range(3)],
        )
        g.chain(
            name + "_decode",
            "float32",
            [record(as_soft(kept[f]), soft=True, meta=seq[f]) for f in range(3)],
            [
                stage("Depuncture", "depuncture", pattern=mask),
                stage("ViterbiDecodeSoft", "decoder", **classic_settings()),
            ],
            "uint8",
            [
                record(
                    infos[f],
                    meta={
                        "seq": U32(f),
                        "corrected_errors": U32(erased_ones(coded[f], mask)),
                    },
                )
                for f in range(3)
            ],
        )
    rng = Lcg(PUNCTURE_SEED ^ 0x3)
    infos = [rng.bits(96) for _ in range(2)]
    inputs, expected, flips = [], [], 0
    for f, info in enumerate(infos):
        coded = classic(info)
        values, moved = channel(punctured(coded, "1110"), f, 0.4)
        flips += moved
        corrected = U32(erased_ones(coded, "1110") + moved)
        inputs.append(record(values, soft=True, meta={"seq": U32(f)}))
        expected.append(
            record(info, meta={"seq": U32(f), "corrected_errors": corrected})
        )
    assert flips > 0
    g.chain(
        "noisy",
        "float32",
        inputs,
        [
            stage("Depuncture", "depuncture", pattern="1110"),
            stage("ViterbiDecodeSoft", "decoder", **classic_settings()),
        ],
        "uint8",
        expected,
    )
    return g


def puncture_records_graph():
    g = Graph(
        "puncture-records.yaml",
        """
Puncture and Depuncture with a record's metadata and with the records they refuse. The carried chains run a
record with three meta keys through each block, and the sinks expect exactly those keys and the signal name. The
bare chain punctures a record without a metadata map, and the sink expects one empty map. The refused puncture chain
gives Puncture an empty record and a record of eight bits. The short chain encodes three bits into ten coded bits,
which '1110' punctures to eight. The refused depuncture chain gives Depuncture those eight bits as soft values,
which are not a whole number of the pattern's three kept positions, an empty record and six values of +1.0. Each
sink expects the good record alone, and the block's line at stop counts the records refused.
Written by generate_graphs.py from seed 0xD1B54A32D192ED03 ^ 0x2. Oracle: the drawn information and the pattern's
kept positions.
""",
    )
    rng = Lcg(PUNCTURE_SEED ^ 0x2)
    meta = {
        "corrected_errors": U32(3),
        "provenance": "earlier stage",
        "sample_start": U64(4096),
    }
    bits = rng.bits(8)
    g.chain(
        "carried_puncture",
        "uint8",
        [record(bits, names=["fec"], meta=meta)],
        [stage("Puncture", "puncture", pattern="1110")],
        "uint8",
        [record(punctured(bits, "1110"), names=["fec"], meta=meta)],
        meta_exact=True,
    )
    soft = as_soft(punctured(bits, "1110"))
    g.chain(
        "carried_depuncture",
        "float32",
        [record(soft, soft=True, names=["fec"], meta=meta)],
        [stage("Depuncture", "depuncture", pattern="1110")],
        "float32",
        [record(depunctured(soft, "1110"), soft=True, names=["fec"], meta=meta)],
        meta_exact=True,
    )
    bits = rng.bits(8)
    g.chain(
        "bare",
        "uint8",
        [record(bits)],
        [stage("Puncture", "puncture", pattern="1110")],
        "uint8",
        [record(meta={})],
        meta_exact=True,
    )
    good = rng.bits(8)
    g.chain(
        "refused_puncture",
        "uint8",
        [record([]), record(good)],
        [stage("Puncture", "puncture", pattern="1110")],
        "uint8",
        [record(punctured(good, "1110"))],
    )
    info = rng.bits(3)
    short = conv_encode(info, *SHORTEST)
    assert len(short) == 10 and len(punctured(short, "1110")) == 8
    g.chain(
        "short",
        "uint8",
        [record(info)],
        [
            stage("ConvEncode", "coder", **shortest_settings()),
            stage("Puncture", "puncture", pattern="1110"),
        ],
        "uint8",
        [record(punctured(short, "1110"), extents=[8])],
    )
    whole = [1.0] * 6
    g.chain(
        "refused_depuncture",
        "float32",
        [
            record(as_soft(punctured(short, "1110")), soft=True),
            record([], soft=True),
            record(whole, soft=True),
        ],
        [stage("Depuncture", "depuncture", pattern="1110")],
        "float32",
        [record(depunctured(whole, "1110"), soft=True)],
    )
    return g


def puncture_settings():
    return settings_graph(
        "puncture-settings.yaml",
        """
Puncture under the rate 2/3 pattern '1110' and Depuncture under the rate 3/4 pattern '111001', each with no record
to process. The run passes as written. A case that changes a pattern through SET checks that the block refuses it,
and the refusal ends the run with the block's reason.
Written by generate_graphs.py with no seed and no oracle: the puncture chains hold no record.
""",
        [
            ("puncture", "uint8", "Puncture", "uint8", {"pattern": "1110"}),
            ("depuncture", "float32", "Depuncture", "float32", {"pattern": "111001"}),
        ],
    )


GRAPHS = [
    fec_round_trip,
    fec_anchors,
    fec_errors,
    fec_records,
    rs_round_trip,
    rs_errors,
    fec_settings,
    conv_encode_graph,
    conv_decode_graph,
    conv_records_graph,
    conv_settings,
    puncture_patterns_graph,
    puncture_chain_graph,
    puncture_records_graph,
    puncture_settings,
]


# Mutants and the command line.


def mutants(g):
    """Per sink, the sink's parameters with its first expected value or meta count
    changed, its name and its index in the graph."""
    out = []
    for i, (bid, name, params) in enumerate(g.blocks):
        if "ExpectSink" in bid:
            changed = dict(params)
            first = (params["expected"][0] + 1) & 0xFF
            changed["expected"] = [first] + params["expected"][1:]
            out.append((name, changed, i))
        elif "ExpectDataSetSink" in bid and params["records"]:
            records = [dict(r) for r in params["records"]]
            r = records[0]
            if "signal_values" in r:
                v = list(r["signal_values"])
                if isinstance(r["signal_values"], F32s):
                    v[0] = v[0] + 0.5
                    r["signal_values"] = F32s(v)
                else:
                    v[0] = (v[0] + 1) & 0xFF
                    r["signal_values"] = v
            elif "extents" in r:
                r["extents"] = [r["extents"][0] + 1]
            elif isinstance(r.get("meta_information"), dict) and r["meta_information"]:
                key = sorted(r["meta_information"])[0]
                m = dict(r["meta_information"])
                m[key] = U32(m[key] + 1)
                r["meta_information"] = m
            else:
                records.append({})
            out.append((name, {**params, "records": records}, i))
        elif "ExpectDataSetSink" in bid:
            out.append((name, {**params, "records": [{}]}, i))
    return out


def cached_setting(build_dir, name):
    path = os.path.join(build_dir, "CMakeCache.txt")
    if not os.path.exists(path):
        return None
    with open(path) as f:
        for line in f:
            if line.startswith(name + ":"):
                return line.split("=", 1)[1].strip() or None
    return None


def capture_digest(rungraph, plugin_dirs):
    """A digest of the rungraph binary and of the shared libraries in the plugin
    directories."""
    digest = hashlib.sha1()
    with open(rungraph, "rb") as f:
        digest.update(hashlib.sha1(f.read()).digest())
    for directory in plugin_dirs:
        for entry in sorted(os.listdir(directory)):
            if re.search(r"\.(so|dylib|dll)$", entry):
                digest.update(entry.encode())
                with open(os.path.join(directory, entry), "rb") as f:
                    digest.update(hashlib.sha1(f.read()).digest())
    return digest.hexdigest()


def parse_arguments():
    top = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
    parser = argparse.ArgumentParser(description="Write the fec family's graph files.")
    parser.add_argument(
        "--build-dir",
        default=os.path.join(top, "build"),
        help="the configured and built tree of this repository (default: build)",
    )
    parser.add_argument(
        "--rungraph",
        help="the rungraph binary (default: the one the build tree's gnuradio4 "
        "package or CMAKE_PREFIX_PATH holds, else rungraph on PATH)",
    )
    parser.add_argument(
        "--library-dir",
        help="a directory to prepend to the library path for rungraph (default: the "
        "gnuradio4 package's library directory)",
    )
    parser.add_argument(
        "--plugin-dirs",
        help="the plugin directories, separated by ':' (default: the build tree's "
        "testing, basic, fec and fileio plugins)",
    )
    parser.add_argument(
        "--output-dir",
        default=os.path.join(HERE, "graphs"),
        help="where the graph files go (default: graphs/ beside this script)",
    )
    parser.add_argument(
        "--cache-dir",
        help="where captures are kept (default: blocks/fec/test/graph-captures in "
        "the build tree)",
    )
    parser.add_argument(
        "--mutants-dir",
        help="also write one mutant per sink here, listed in mutants.txt",
    )
    return parser.parse_args()


def find_rungraph(build, package):
    """The bin/rungraph of the gnuradio4 package's four parent directories, then of
    each CMAKE_PREFIX_PATH entry, then on PATH."""
    hints = []
    if package is not None:
        parent = package
        for _ in range(4):
            parent = os.path.dirname(parent)
            hints.append(parent)
    prefixes = cached_setting(build, "CMAKE_PREFIX_PATH")
    if prefixes is not None:
        hints += prefixes.split(";")
    for hint in hints:
        candidate = os.path.join(hint, "bin", "rungraph")
        if os.access(candidate, os.X_OK):
            return candidate
    return shutil.which("rungraph")


def configure(args):
    build = os.path.abspath(args.build_dir)
    package = cached_setting(build, "gnuradio4_DIR")
    rungraph = args.rungraph or find_rungraph(build, package)
    if rungraph is None:
        sys.exit("no rungraph found: pass --rungraph")
    library_dir = args.library_dir
    if library_dir is None and package is not None:
        library_dir = os.path.normpath(os.path.join(package, "..", ".."))
    if args.plugin_dirs:
        plugin_dirs = args.plugin_dirs.split(":")
    else:
        families = ("testing", "basic", "fec", "fileio")
        plugin_dirs = [os.path.join(build, "blocks", f) for f in families]
    for directory in plugin_dirs:
        if not os.path.isdir(directory):
            sys.exit(f"no plugin directory {directory}: build the plugins first")
    environment = dict(os.environ)
    environment["GNURADIO4_PLUGIN_DIRECTORIES"] = ":".join(plugin_dirs)
    variable = "DYLD_LIBRARY_PATH" if sys.platform == "darwin" else "LD_LIBRARY_PATH"
    if library_dir is not None and sys.platform != "win32":
        paths = [library_dir] + [p for p in [environment.get(variable)] if p]
        environment[variable] = ":".join(paths)
    Options.output_dir = os.path.abspath(args.output_dir)
    default_cache = os.path.join(build, "blocks", "fec", "test", "graph-captures")
    Options.cache_dir = os.path.abspath(args.cache_dir or default_cache)
    Options.rungraph = rungraph
    Options.environment = environment
    Options.digest = capture_digest(rungraph, plugin_dirs)
    os.makedirs(Options.output_dir, exist_ok=True)
    os.makedirs(Options.cache_dir, exist_ok=True)


def main():
    args = parse_arguments()
    configure(args)
    listing = []
    for make in GRAPHS:
        g = make()
        with open(os.path.join(Options.output_dir, g.file), "w") as f:
            f.write(g.text())
        print("wrote", g.file)
        if args.mutants_dir is None:
            continue
        os.makedirs(args.mutants_dir, exist_ok=True)
        for name, params, i in mutants(g):
            m = Graph(g.file, g.comment)
            m.blocks = list(g.blocks)
            m.blocks[i] = (g.blocks[i][0], name, params)
            m.edges = g.edges
            stem = g.file.replace(".yaml", "") + "--" + name + ".yaml"
            path = os.path.abspath(os.path.join(args.mutants_dir, stem))
            with open(path, "w") as f:
                f.write(m.text())
            listing.append(f"{path} {name}")
    if args.mutants_dir is not None:
        with open(os.path.join(args.mutants_dir, "mutants.txt"), "w") as f:
            f.write("\n".join(listing) + "\n")


if __name__ == "__main__":
    main()
