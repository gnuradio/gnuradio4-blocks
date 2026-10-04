# SDR blocks

The `gr::blocks::sdr` family reaches SDR hardware through
[SoapySDR](https://github.com/pothosware/SoapySDR) (Boost-1.0). The family is
off by default (`GR4_ENABLE_SDR`). Its blocks, the loopback device and the tests
that use them build only where SoapySDR is found.

## Blocks

`SoapySource<T>` receives one or more channels from a SoapySDR device, and
`SoapySink<T>` transmits one or more. Each runs a dedicated IO thread. Each
block opens its device by the driver name merged with its `device_parameter`
arguments. A sink and a source with the same driver and device parameters share
one device handle. The registered sample types are `std::uint8_t`,
`std::int16_t` and `std::complex<float>`.

`DcBlocker.hpp` and `Saturate.hpp` are plain headers and need no SoapySDR.

## Loopback device

`LoopbackDevice.hpp` and `src/SoapyLoopbackModule.cpp` implement a SoapySDR
device with the driver name `loopback`. It routes transmitted samples through a
per-channel model to the receive side. The build produces it twice for the
tests: the static library `gr-sdr-loopback-static`, which tests link, and the
SoapySDR module `gr-sdr-loopback`, which tests load. Neither is installed.

## Tests

- `qa_DcBlocker`, `qa_Saturate`, `qa_SdrHeaderOrder` and
  `qa_apptest_SdrBlockLibrary` open no device.
- `qa_SoapyLoopback` links the static loopback device.
- `qa_SoapyRaiiWrapper` and `qa_SoapyIntegration` load the loopback module.
  ctest runs them with `SOAPY_SDR_PLUGIN_PATH` set to the test directory that
  holds a copy of it.
- `qa_SoapySource` opens attached hardware.

The three loopback tests are skipped where `ldd` finds that the SoapySDR
library links a different C++ standard library from this project's.
