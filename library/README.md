# GNU Radio 4 Algorithm

This repository contains the reusable non-block DSP and algorithm libraries split out of GNU Radio 4.

It builds against an installed `gnuradio4-core` SDK and installs the camelCase CMake package `gnuradio4Algorithm`.

## Build model

- configure against the installed core prefix
- use `GR_USE_FETCHCONTENT_DEPS=ON` when test-only dependencies should be fetched automatically
- install into the shared local prefix used by the GNU Radio 4 umbrella workspace

## Package naming

- repository name: `gnuradio4-algorithm`
- CMake package name: `gnuradio4Algorithm`
- exported target namespace: `gnuradio4::`

