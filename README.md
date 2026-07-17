[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Main CI](https://github.com/gnuradio/gnuradio4-blocks/actions/workflows/ci-main.yml/badge.svg?branch=main)](https://github.com/gnuradio/gnuradio4-blocks/actions/workflows/ci-main.yml)
[![SDK Image](https://github.com/gnuradio/gnuradio4-blocks/actions/workflows/sdk-image.yml/badge.svg?branch=main)](https://github.com/gnuradio/gnuradio4-blocks/actions/workflows/sdk-image.yml)

# GNU Radio 4.0 Blocks

`gnuradio4-blocks` provides the standard C++23 block libraries for GNU Radio 4.
It builds against installed
[`gnuradio4-core`](https://github.com/gnuradio/gnuradio4-core) and
[`gnuradio4-library`](https://github.com/gnuradio/gnuradio4-library) SDKs and
installs blocks, public headers, runtime block libraries, and CMake targets for
applications and out-of-tree block projects.

This repository is one part of the split GNU Radio 4 source tree:

- [`gnuradio4-core`](https://github.com/gnuradio/gnuradio4-core): runtime,
  scheduler, graph and block model, plugin support, and block-development SDK
- [`gnuradio4-library`](https://github.com/gnuradio/gnuradio4-library): reusable,
  non-block DSP algorithms built on core
- `gnuradio4-blocks`: standard blocks built on core and the algorithm library

## What's Included?

The default build contains:

- basic signal, function, and clock sources; converters; selectors; triggers;
  synchronization, data sinks, and stream-to-data-set blocks
- arithmetic, expression-evaluation, complex rotator
- FIR, IIR, Savitzky-Golay, and SVD filters, plus frequency estimation and
  decimation
- FFT blocks and benchmarks
- File I/O sources and sinks
- HTTP source and sink blocks
- common device utilities and testing, monitoring, delay, and null-source blocks

Audio source/sink blocks and SDR support for RTL2832 devices and SoapySDR are
available as opt-in block modules. The tree also contains native and Emscripten
examples for GPS, RTL2832, and audio-related workflows where their corresponding
modules and toolchains are enabled.

## Building

The project requires CMake 3.27 or newer, a C++23 compiler, and compatible
installations of the `gnuradio4`, `gnuradio4Library`, and `GnuRadioBlockLib`
CMake packages. A default test build also requires Boost.UT and cpp-httplib.
Point `CMAKE_PREFIX_PATH` at the prefix containing the installed core and
library SDKs:

```bash
git clone https://github.com/gnuradio/gnuradio4-blocks.git
cd gnuradio4-blocks

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_PREFIX_PATH=/path/to/gnuradio4-prefix
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Install into the prefix used by the rest of the GNU Radio 4 environment:

```bash
cmake --install build --prefix /path/to/gnuradio4-prefix
```

Useful configuration options are:

- `ENABLE_EXAMPLES` (default: `ON`): build examples and benchmarks
- `ENABLE_TESTING` (default: `ON`): build and register tests
- `GR4_ENABLE_AUDIO` (default: `OFF`): build audio blocks; native builds use
  libsoundio, while Emscripten builds use the Web Audio backend
- `GR4_ENABLE_SDR` (default: `OFF`): build RTL2832 support and, when found,
  SoapySDR source/sink support
- `GR4_ENABLE_HTTP_TESTS` (default: `OFF`): enable HTTP integration tests
- `GR_USE_FETCHCONTENT_DEPS` (default: `OFF`): allow CMake to fetch Boost.UT
- `USE_CCACHE` (default: `ON`): use `ccache` when available
- `GR4_PKG_VERSION` (default: `4.0.0-git`): version recorded in the installed
  `gnuradio4Blocks.pc` metadata

## Using the Blocks

An installation exports the camel-case `gnuradio4Blocks` CMake package and
targets in the `gnuradio4::` namespace. Link the interface target for the block
family whose headers you use; for example:

```cmake
find_package(gnuradio4Blocks CONFIG REQUIRED)

add_executable(my_flowgraph main.cpp)
target_link_libraries(my_flowgraph PRIVATE gnuradio4::gr-basic)
```

Other installed family targets include `gnuradio4::gr-common`,
`gnuradio4::gr-testing`, `gnuradio4::gr-electrical`, `gnuradio4::gr-fileio`,
`gnuradio4::gr-filter`, `gnuradio4::gr-fourier`, `gnuradio4::gr-http`, and
`gnuradio4::gr-math`. Public headers are installed below
`include/gnuradio-4.0`.

## SDK Images

Pushes to `main` publish profile-specific SDK images to the GitHub Container
Registry. They contain compatible core, algorithm-library, and default blocks
installations under `/opt/gnuradio4`:

```text
ghcr.io/gnuradio/gnuradio4-blocks-sdk:<sha-or-main>-<platform>-<compiler>-<profile>
```

Published profiles cover the Ubuntu 24.04 GCC 14 and Clang 20, Ubuntu 26.04
GCC, and Fedora 44 Clang workflow variants. Use a complete tag such as
`main-ubuntu-24.04-gcc14-release`; no generic `main` tag is published. The SDK
image is configured with examples, tests, audio, and SDR disabled.

## License

Unless otherwise noted, this project is licensed under the [MIT License](LICENSE).

Copyright (C) The GNU Radio Authors<br>
Copyright (C) Contributors to the GNU Radio Project<br>
Copyright (C) FAIR - Facility for Antiproton & Ion Research, Darmstadt, Germany

## Helpful Links

- [GNU Radio website](https://gnuradio.org/)
- [GNU Radio wiki](https://wiki.gnuradio.org/)
- [Issue tracker](https://github.com/gnuradio/gnuradio4-blocks/issues)
- [GNU Radio Matrix chat](https://chat.gnuradio.org/)
- [GNU Radio mailing list](https://lists.gnu.org/mailman/listinfo/discuss-gnuradio)
