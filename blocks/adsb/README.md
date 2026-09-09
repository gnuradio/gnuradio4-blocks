# adsb

The 1090 MHz Mode S and ADS-B chain: `PpmFramer` reads a candidate frame's octets out of a magnitude
stream and checks its parity, and the message layer says what an admitted frame _says_. Three blocks;
the two message-layer ones are record adapters over `gr::DataSet<std::uint8_t>`.

The framer is here rather than in `digital` because pulse-position modulation at this preamble, frame
length and parity is Mode S and nothing else. The pulse-position scanner it runs on is the general
kernel and stays in the algorithm layer, as `gr::digital::PpmScanner`.

The blocks stay in `gr::blocks::adsb` and ship no compatibility import: the module is new, so there is
no earlier `gr::adsb` for one to be compatible with.

```
… -> basic::Abs -> adsb::PpmFramer(profile: mode_s)    -> adsb::ModeSDecode -> adsb::AdsbPrinter -> …
                          octets                             typed fields          one text line
```

The framing is ICAO Annex 10 Volume IV, whose extended squitter carries a 56-bit ME field in bits
33 to 88 of a long frame; the message formats inside that field are RTCA DO-260B. Bit numbering
follows the standard throughout: bit 1 is the most significant bit of the first octet.

## What crosses, and in what form

**The payload stays the frame's octets and every decoded field is a metadata key.** So the parity
the framer checked still covers what a record carries, a consumer that wants the frame itself has
it, and a consumer that speaks the protocol needs no parser of its own. No block here puts a
struct or a rendered line where the octets belong.

**Every value is a scalar** — an integer, a double, a bool or a string. A metadata map that leaves
the process is a wire format whether or not anything calls it one, and the encoder the transports
share round-trips scalars and does not round-trip sequences. Nothing here is a list, and a field
whose natural shape is a list would go in the payload instead.

## `ModeSDecode`

One framer record in, the same octets out with the message read into metadata.

A frame is decoded **only where the input record's `crc_ok` is true.** `PpmFramer` writes that key
on long frames alone, so a short-format nomination, which carries no self-checking parity, and a
long frame whose remainder was not zero both cross with their own metadata otherwise untouched, `protocol`
naming which of the two they are. Reading fields out of octets nothing vouched for is how a
decoder reports an aircraft that is not there.

| key                         | type     | on                | meaning                                                    |
| --------------------------- | -------- | ----------------- | ---------------------------------------------------------- |
| `protocol`                  | `string` | every record      | the stream's sub-kind, general to specific (below)         |
| `mode_s_format`             | `uint32` | decoded           | the downlink format, bits 1 to 5                           |
| `mode_s_icao`               | `string` | DF 11, 17, 18     | the announced address, six uppercase hex digits            |
| `adsb_type_code`            | `uint32` | DF 17, 18         | the ME field's type code, bits 33 to 37                    |
| `adsb_message`              | `string` | DF 17, 18         | what that type code names (below)                          |
| `adsb_emitter_category`     | `uint32` | type code 1 to 4  | the category subfield, bits 38 to 40                       |
| `adsb_callsign`             | `string` | type code 1 to 4  | the eight-character identification, its pad removed        |
| `adsb_altitude_ft`          | `int32`  | type code 9 to 18 | barometric altitude, where the code assigns one            |
| `adsb_altitude_q`           | `bool`   | type code 9 to 18 | the Q bit: set, the altitude is to 25 ft; clear, to 100 ft |
| `adsb_cpr_format`           | `uint32` | type code 9 to 18 | 0 for the even half of a position pair, 1 for the odd      |
| `adsb_cpr_latitude`         | `uint32` | type code 9 to 18 | the encoded latitude, seventeen bits                       |
| `adsb_cpr_longitude`        | `uint32` | type code 9 to 18 | the encoded longitude, seventeen bits                      |
| `adsb_latitude`             | `double` | a resolved pair   | degrees north                                              |
| `adsb_longitude`            | `double` | a resolved pair   | degrees east                                               |
| `adsb_velocity_subtype`     | `uint32` | type code 19      | 1 and 2 ground referenced, 3 and 4 air referenced          |
| `adsb_ground_speed_kt`      | `double` | subtypes 1, 2     | knots over the ground                                      |
| `adsb_track_deg`            | `double` | subtypes 1, 2     | degrees true, 0 to 360                                     |
| `adsb_airspeed_kt`          | `double` | subtypes 3, 4     | knots through the air                                      |
| `adsb_airspeed_type`        | `string` | subtypes 3, 4     | `ias` or `tas`                                             |
| `adsb_heading_deg`          | `double` | subtypes 3, 4     | degrees magnetic, where the heading status bit is set      |
| `adsb_vertical_rate_fpm`    | `int32`  | type code 19      | feet a minute, positive up                                 |
| `adsb_vertical_rate_source` | `string` | type code 19      | `barometric` or `geometric`                                |

Every key of the input record crosses beneath these, so what `PpmFramer` reported about the frame
— `sample_start`, `sample_rate`, `sequence`, `crc_remainder`, `preamble_strong`, `preamble_weak`,
`trigger_name` — survives beside what the frame says about itself.

`mode_s_format` is written under the spelling the framer already uses, and is re-asserted here from
the octets rather than copied: the downlink format gets one key and not two, and a consumer needs
no rule about which of two spellings to believe.

`protocol` values, `/`-separated from general to specific:

| value              | when                                                                     |
| ------------------ | ------------------------------------------------------------------------ |
| `mode_s/adsb`      | a decoded extended squitter, downlink format 17 or 18                    |
| `mode_s`           | any other frame the parity vouched for                                   |
| `mode_s/unchecked` | a long frame whose remainder was not zero, published by `emit_unchecked` |
| `mode_s/short`     | a short-format nomination, published by `emit_short`                     |

`adsb_message` values: `no_position`, `identification`, `surface_position`, `airborne_position`,
`airborne_position_gnss`, `airborne_velocity`, `aircraft_status`, `target_state`,
`operational_status`, `reserved`. The surface position, the GNSS-height position and the status
messages are named and not decoded; their type code and their octets are on the record.

### The altitude has two codings and both are in service

Where the AC field's Q bit is set the remaining eleven bits are 25 ft increments offset by
-1000 ft. Where it is clear the field is the 100 ft code of ICAO Annex 10 Volume IV: a count of
500 ft steps in a reflected binary code on the D and A and B pulses, and a 100 ft digit in the
three C pulses, counted the other way on an odd step. A decoder that reads only the Q set case
reports aircraft that are transmitting an altitude as having none, and the 100 ft coding is in
everyday use.

### A position needs two frames

Compact position reporting encodes a coordinate as its offset within a zone, so one frame names a
position only relative to a zone the frame does not state. An even half and an odd half together
name it globally. The halves are held per aircraft in a table of `table_size` addresses, the
address least recently seen giving way when it is full, and a pair whose halves are further apart
in time than `pair_seconds` is refused and counted. A pair straddling a longitude zone boundary
resolves to nothing rather than to something wrong, and the next pair resolves.

A frame whose pair did not resolve still carries its own `adsb_cpr_*` fields, so the position
stays a consumer's to compute if it knows a reference position this block does not.

| setting        | default | meaning                                                  |
| -------------- | ------- | -------------------------------------------------------- |
| `pair_seconds` | 10      | greatest age difference between the two halves of a pair |
| `table_size`   | 256     | aircraft addresses the pairing table holds               |

Counters a consumer may read at any time: `nFrames`, `nDecoded`, `nPositions`, `nPairsExpired`,
`nUnchecked`, `nEvicted`.

## `AdsbPrinter`

One decoded record in, the line that describes it out as the payload of a text record:

```
   29.781  DF17  AB0969  airpos  alt 39000 ft  lat 42.40448  lon -71.34696  [AAL160]
```

The time is from the frame's own `sample_start` and `sample_rate`. The identification in brackets
is remembered per address, in a table of `table_size` entries bounded the same way, because an
aircraft names itself in one message and reports its position in another.

The block **reads only the typed keys `ModeSDecode` wrote**, never the octets, except to show a
frame the decoder would not read — which prints its protocol and its hex instead, that being all
it may be believed about. So the line and a network peer's structured view of the same record
cannot disagree.

The published record's `protocol` is `text/adsb` and its payload carries no trailing newline; the
input record's keys all cross beneath, so a consumer can still filter on the address or the
altitude the line was rendered from. `source_id` is deliberately not written: the record-metadata
vocabulary gives that key to the receiver, as the operator's name for it.

`to_stdout`, off by default, additionally writes each line to standard output, which is what a
headless runner with no consumer attached wants.

## Tests

`test/qa_ModeSDecode` builds every frame field by field from the standard, so a case states
what the standard says and not what one capture happened to hold. The compact position case
encodes a chosen latitude and longitude with its own implementation of the standard's encoder and
asks the block for the position back, which judges the pairing arithmetic against the equations
rather than against itself. One case carries a decoded record through
`basic::DataSetToPacket` and `basic::PacketToDataSet` and through the metadata encoder the
transports share, and checks every field back at its own type. `test/qa_AdsbPrinter` asserts the
lines the pair renders, because the line is the printer's whole contract. `test/qa_PpmFramer` drives the
framer over synthetic magnitude streams built from the standard's preamble and chip timing.
