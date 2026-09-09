#!/usr/bin/env python3
"""Migrate a saved flow graph across two renames in this repository.

1. The waveform frequency of gr::blocks::basic::SignalGenerator and
   gr::blocks::basic::FunctionGenerator moved off `frequency`, which is a reserved
   stream-tag key naming the stream's center frequency, and is now `tone_frequency`.
   A saved graph that names `frequency` on either block no longer reaches the
   waveform. Only those two blocks are touched: `frequency` on, say, a Soapy source
   is the center frequency and stays as it is.

2. Thirty-three families were registered under an alias that repeated a fixed
   template argument its name already stated, so the alias key carried that
   argument: gr::blocks::math::Add<float32, std::plus<float32>> where a file writes
   Add<float32>. Those alias keys now expand over the value type alone, so every
   argument after the first is dropped:

     gr::blocks::basic::SchmittTrigger                        gr::blocks::math::Add
     gr::blocks::basic::SchmittTriggerBasic                   gr::blocks::math::AddConst
     gr::blocks::basic::SchmittTriggerNoInterpolation         gr::blocks::math::Divide
     gr::blocks::basic::SchmittTriggerPolynomial              gr::blocks::math::DivideConst
     gr::blocks::basic::StreamFilter                          gr::blocks::math::Multiply
     gr::blocks::basic::StreamToDataSet                       gr::blocks::math::MultiplyConst
     gr::blocks::electrical::SinglePhasePowerFactorCalculator gr::blocks::math::Subtract
     gr::blocks::electrical::SinglePhasePowerMetrics          gr::blocks::math::SubtractConst
     gr::blocks::electrical::ThreePhasePowerFactorCalculator  gr::blocks::sdr::SoapyDualSink
     gr::blocks::electrical::ThreePhasePowerMetrics           gr::blocks::sdr::SoapyDualSource
     gr::blocks::electrical::ThreePhaseSystemUnbalanceCalculator gr::blocks::sdr::SoapyQuadSink
     gr::blocks::electrical::TwoPhaseSystemUnbalanceCalculator gr::blocks::sdr::SoapySink
     gr::blocks::filter::FrequencyEstimatorFrequencyDomainDecimating gr::blocks::sdr::SoapySource
     gr::blocks::filter::FrequencyEstimatorTimeDomainDecimating  gr::blocks::testing::ConsoleDebugSink
     gr::blocks::filter::IQDemodulator                        gr::blocks::testing::ImChartMonitor
     gr::blocks::testing::TagMonitor                          gr::blocks::testing::TagSink
     gr::blocks::testing::TagSource

   Nothing about the C++ types changed, so each block's primary key -- the
   expansion an alias resolves to -- is what it was: a graph naming the expansion
   needs no migration. Neither do gr::blocks::filter::BasicFilter,
   BasicDecimatingFilter or the four IirFilterDirectForm* names, which only gained
   keys beside their unchanged expansions.

Usage:
    python3 tools/migrate_generator_settings.py graph.grc              # report only
    python3 tools/migrate_generator_settings.py --in-place graph.grc   # rewrite

Self-test:
    python3 -m unittest discover -s tools -p 'migrate_*.py'
"""

import argparse
import re
import sys
import unittest

GENERATOR_BLOCKS = (
    "gr::blocks::basic::SignalGenerator",
    "gr::blocks::basic::FunctionGenerator",
)

ALIAS_FAMILIES = (
    "gr::blocks::basic::SchmittTrigger",
    "gr::blocks::basic::SchmittTriggerBasic",
    "gr::blocks::basic::SchmittTriggerNoInterpolation",
    "gr::blocks::basic::SchmittTriggerPolynomial",
    "gr::blocks::basic::StreamFilter",
    "gr::blocks::basic::StreamToDataSet",
    "gr::blocks::electrical::SinglePhasePowerFactorCalculator",
    "gr::blocks::electrical::SinglePhasePowerMetrics",
    "gr::blocks::electrical::ThreePhasePowerFactorCalculator",
    "gr::blocks::electrical::ThreePhasePowerMetrics",
    "gr::blocks::electrical::ThreePhaseSystemUnbalanceCalculator",
    "gr::blocks::electrical::TwoPhaseSystemUnbalanceCalculator",
    "gr::blocks::filter::FrequencyEstimatorFrequencyDomainDecimating",
    "gr::blocks::filter::FrequencyEstimatorTimeDomainDecimating",
    "gr::blocks::filter::IQDemodulator",
    "gr::blocks::math::Add",
    "gr::blocks::math::AddConst",
    "gr::blocks::math::Divide",
    "gr::blocks::math::DivideConst",
    "gr::blocks::math::Multiply",
    "gr::blocks::math::MultiplyConst",
    "gr::blocks::math::Subtract",
    "gr::blocks::math::SubtractConst",
    "gr::blocks::sdr::SoapyDualSink",
    "gr::blocks::sdr::SoapyDualSource",
    "gr::blocks::sdr::SoapyQuadSink",
    "gr::blocks::sdr::SoapySink",
    "gr::blocks::sdr::SoapySource",
    "gr::blocks::testing::ConsoleDebugSink",
    "gr::blocks::testing::ImChartMonitor",
    "gr::blocks::testing::TagMonitor",
    "gr::blocks::testing::TagSink",
    "gr::blocks::testing::TagSource",
)

_ID_LINE = re.compile(
    r"^(?P<lead>\s*(?:-\s+)?)id:(?P<gap>\s*)(?P<value>.*?)(?P<trail>\s*)$"
)
_FREQUENCY_LINE = re.compile(r"^(?P<indent>\s*)frequency:(?P<rest>.*)$")


def split_top_level(arguments):
    """Split a template argument list on the commas that are not inside brackets."""
    parts, depth, current = [], 0, ""
    for char in arguments:
        if char in "<([":
            depth += 1
        elif char in ">)]":
            depth -= 1
        if char == "," and depth == 0:
            parts.append(current)
            current = ""
            continue
        current += char
    parts.append(current)
    return [part.strip() for part in parts]


def base_name(block_id):
    """The block id with its template argument list removed."""
    return block_id.split("<", 1)[0].strip()


def migrate_id(block_id):
    """The migrated block id, or None when the id needs no migration."""
    stripped = block_id.strip().strip("'\"")
    if (
        base_name(stripped) not in ALIAS_FAMILIES
        or "<" not in stripped
        or not stripped.endswith(">")
    ):
        return None
    name, _, arguments = stripped.partition("<")
    parts = split_top_level(arguments[:-1])
    if len(parts) < 2:
        return None
    return "{}<{}>".format(name, parts[0])


def migrate(text):
    """Return the migrated text and a list of the changes made, as (line number, description)."""
    lines = text.splitlines(keepends=True)
    changes = []
    generator_indent = (
        None  # indentation of the current generator block's item, None when outside one
    )

    for number, line in enumerate(lines, start=1):
        body = line.rstrip("\r\n")
        indent = len(body) - len(body.lstrip())

        match = _ID_LINE.match(body)
        if match:
            block_id = match.group("value")
            generator_indent = (
                indent
                if base_name(block_id.strip().strip("'\"")) in GENERATOR_BLOCKS
                else None
            )
            migrated = migrate_id(block_id)
            if migrated is not None:
                lines[number - 1] = "{}id:{}{}{}\n".format(
                    match.group("lead"),
                    match.group("gap"),
                    migrated,
                    match.group("trail"),
                )
                changes.append(
                    (number, "id {} -> {}".format(block_id.strip(), migrated))
                )
            continue

        if generator_indent is not None and body.strip() and indent <= generator_indent:
            generator_indent = (
                None  # the generator's own lines are indented past its item
            )

        if generator_indent is not None:
            frequency = _FREQUENCY_LINE.match(body)
            if frequency:
                lines[number - 1] = "{}tone_frequency:{}\n".format(
                    frequency.group("indent"), frequency.group("rest")
                )
                changes.append((number, "frequency -> tone_frequency"))

    return "".join(lines), changes


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Migrate a saved flow graph across the generator setting and alias renames."
    )
    parser.add_argument("graph", help="the saved graph file to migrate")
    parser.add_argument(
        "--in-place",
        action="store_true",
        help="rewrite the file; without it the changes are only reported",
    )
    arguments = parser.parse_args(argv)

    with open(arguments.graph, "r", encoding="utf-8") as handle:
        original = handle.read()
    migrated, changes = migrate(original)

    for number, description in changes:
        print("{}:{}: {}".format(arguments.graph, number, description))
    if not changes:
        print("{}: nothing to migrate".format(arguments.graph))
        return 0
    if not arguments.in_place:
        print(
            "{}: {} change(s) not written; pass --in-place to rewrite".format(
                arguments.graph, len(changes)
            )
        )
        return 0
    with open(arguments.graph, "w", encoding="utf-8") as handle:
        handle.write(migrated)
    print("{}: {} change(s) written".format(arguments.graph, len(changes)))
    return 0


BEFORE = """\
blocks:
  - id: gr::blocks::basic::SignalGenerator<float32>
    parameters:
      name: tone
      frequency: 440
      sample_rate: 48000
  - id: gr::blocks::math::Add<float32, std::plus<float32>>
    parameters:
      name: sum
  - id: gr::blocks::sdr::SoapySource<complex<float32>, 1UZ>
    parameters:
      name: radio
      frequency: [107000000]
  - id: gr::blocks::math::MathOpMultiPortImpl<float32, std::plus<float32>>
    parameters:
      name: primary
  - id: gr::blocks::basic::FunctionGenerator<float32>
    parameters:
      name: ramp
    ctx_parameters:
      - context: one
        parameters:
          frequency: 10
connections:
  - [tone, 0, sum, 0]
"""

AFTER = """\
blocks:
  - id: gr::blocks::basic::SignalGenerator<float32>
    parameters:
      name: tone
      tone_frequency: 440
      sample_rate: 48000
  - id: gr::blocks::math::Add<float32>
    parameters:
      name: sum
  - id: gr::blocks::sdr::SoapySource<complex<float32>>
    parameters:
      name: radio
      frequency: [107000000]
  - id: gr::blocks::math::MathOpMultiPortImpl<float32, std::plus<float32>>
    parameters:
      name: primary
  - id: gr::blocks::basic::FunctionGenerator<float32>
    parameters:
      name: ramp
    ctx_parameters:
      - context: one
        parameters:
          tone_frequency: 10
connections:
  - [tone, 0, sum, 0]
"""


class MigrateTest(unittest.TestCase):
    def test_fixture_graph(self):
        migrated, changes = migrate(BEFORE)
        self.assertEqual(migrated, AFTER)
        self.assertEqual(len(changes), 4)

    def test_migration_is_idempotent(self):
        self.assertEqual(migrate(AFTER), (AFTER, []))

    def test_center_frequency_is_left_alone(self):
        self.assertIn("      frequency: [107000000]", migrate(BEFORE)[0])

    def test_only_the_value_type_survives(self):
        self.assertEqual(
            migrate_id("gr::blocks::math::Add<float32, std::plus<float32>>"),
            "gr::blocks::math::Add<float32>",
        )
        self.assertEqual(
            migrate_id("gr::blocks::sdr::SoapySink<complex<float32>, 1UZ>"),
            "gr::blocks::sdr::SoapySink<complex<float32>>",
        )

    def test_unchanged_ids_report_nothing(self):
        self.assertIsNone(migrate_id("gr::blocks::math::Add<float32>"))
        self.assertIsNone(migrate_id("gr::blocks::filter::BasicFilterProto<float32>"))
        self.assertIsNone(
            migrate_id(
                "gr::blocks::math::MathOpMultiPortImpl<float32, std::plus<float32>>"
            )
        )


if __name__ == "__main__":
    sys.exit(main())
