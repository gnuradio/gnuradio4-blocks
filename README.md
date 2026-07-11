# GNU Radio 4 Blocks

This repository contains the standard MIT-licensed GNU Radio 4 block libraries.

It builds against installed `gnuradio4-core` and `gnuradio4Library` SDKs and installs the camelCase CMake package `gnuradio4Blocks`.

## Build model

- configure against the shared installed prefix
- use `GR_USE_FETCHCONTENT_DEPS=ON` when test-only dependencies should be fetched automatically
- keep unstable HTTP integration tests disabled by default

## Package naming

- repository name: `gnuradio4-blocks`
- CMake package name: `gnuradio4Blocks`
- exported target namespace: `gnuradio4::`

