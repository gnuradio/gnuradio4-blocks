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
- `qa_SoapyRaiiWrapper`, `qa_SoapyIntegration` and `qa_SoapySinkBurst` load
  the loopback module. ctest runs them with `SOAPY_SDR_PLUGIN_PATH` set to the
  test directory that holds a copy of it.
- `qa_SoapySource` loads the loopback module and opens attached hardware only
  on request.

The four loopback tests are skipped where `ldd` finds that the SoapySDR
library links a different C++ standard library from this project's.

## Running the SoapySDR tests

`qa_SoapySource` runs against the loopback SoapySDR module built with this
family. Its ctest registration and the program itself set `SOAPY_SDR_ROOT` to an
empty directory and `SOAPY_SDR_PLUGIN_PATH` to the loopback module's directory.
SoapySDR then loads no module of the host and opens no attached radio:

```sh
ctest --test-dir <build-dir> -R qa_SoapySource
```

The hardware cases run against the host's modules and attached radios only
when the binary runs directly with `GR_SDR_TEST_HARDWARE=1`:

```sh
GR_SDR_TEST_HARDWARE=1 <build-dir>/blocks/sdr/test/qa_SoapySource
```

With the variable set, each hardware case runs only when enumeration lists its
driver (`rtlsdr` or `lime`). With `DISABLE_SENSITIVE_TESTS` in the environment,
none of them runs. The parameterized Basic API test always opens the loopback
device. It opens a listed hardware device only when the hardware cases run.
Under ctest the test loads only the loopback module, whatever the variable says.
