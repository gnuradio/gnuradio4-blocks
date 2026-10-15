# Example graphs

The files in `graphs/` are complete flowgraphs in the GRC YAML form that
`gr::loadGrc` reads. A graph editor, a host application and `rungraph` load
them without compiling anything. They install to
`share/gnuradio-4.0/examples/`. `ENABLE_EXAMPLES` gates compiled programs:
the families' benchmarks and the ZeroMQ examples. This directory holds graph
files.

- `fm_radio_mono.yaml`: broadcast FM mono, from a wideband front end to
  audio. The chain is a tuner, a decimating channel filter, a discriminator,
  a rational resampler to the audio rate and de-emphasis.

## Running one

Each graph runs with no hardware, no sound device and no display. The
source is a `SignalGenerator` and the sink a `DataSink`. The header comment
of each file names the radio and audio blocks that replace them.

To listen to a station, copy `fm_radio_mono.yaml` and change its
`rf_source` entry to a `gr::blocks::sdr::SoapySource<complex<float32>>`
at the same `sample_rate`. Set `device` to the SoapySDR driver name, for
example `rtlsdr`. `frequency` and `rx_gains` are lists with one entry per
channel, written with the type tag first:

```yaml
- id: gr::blocks::sdr::SoapySource<complex<float32>>
  parameters:
    name: rf_source
    device: rtlsdr
    sample_rate: !!float32 1920000.0
    frequency: !!float64 [98100000.0]
    rx_gains: !!float64 [30.0]
```

Change `audio_sink` to a `gr::blocks::audio::AudioSink<float32>`, and run
the copy:

```sh
rungraph --graph my_fm.yaml --seconds 30
```

`rungraph --set` changes a setting without editing the file, for example
`-s 'tuner.frequency_shift=!!float32 -200000'`.

## Tests

`test/qa_ExampleGraphs.cpp` loads every graph and checks that each block is
constructed under the id its file names. It reads the graphs from a header
the build generates from the files, and from the install layout where the
host file system is readable. It also runs the FM graph headless.
