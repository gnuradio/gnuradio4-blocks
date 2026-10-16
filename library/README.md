# GNU Radio 4.0 Library

The library holds the reusable signal-processing algorithms that blocks share.
Blocks in this repository build on it, and applications and out-of-tree block
projects can use it directly. It depends on `gnuradio4-core` alone.

The dependency runs one way. Code under `library/` may use core. It never
includes a header or links a target from `blocks/`. A piece of code that a
second block family could use moves into `library/algorithm/`, with its test,
and the block keeps a thin wrapper around it.

## What's Included?

The public C++23 library is header-oriented and includes:

- Fourier transforms and window functions, including SIMD FFT support
- random-number, Gaussian-noise, tone, noise, and signal generators
- filter design, polyphase resamplers, and Savitzky-Golay and SVD filters
- tensor arithmetic, dense linear algebra, and singular value decomposition
- data-set estimation, mathematics, and transformation helpers
- uncertainty-aware numeric values and propagated mathematical operations
- file I/O helpers for native and Emscripten builds
- burst tapering, sample-rate estimation, and Schmitt-trigger utilities
- plotting data models for canvases, charts, and graphs

`third_party/` carries the ExprTk expression library, built as the static
`exprtk` target.

## Building

The library builds as part of this repository; see the top-level `README.md`.
Its tests are registered with the rest of the tree's tests and run under
`ctest`. Its benchmarks build with the other benchmarks when
`ENABLE_EXAMPLES` is on.

Two environment variables decide how a test run treats a figure measured on the
wall clock. `DISABLE_SENSITIVE_TESTS` marks a run whose timing is not
representative, such as continuous integration or a sanitizer build: the tests
tagged `benchmarks` and `visual` do not run, and a test that measures a cost
prints the figure and asserts nothing on it. `ENABLE_BENCHMARK_TESTS` marks a
run on a machine whose CPU placement and clock speed the caller controls: the
longer measurements run, and a cost test also asserts the bound it keeps for
such a run. With neither variable set, a cost test asserts only a bound that
holds on any machine, or prints its figure where it has none. When both are
set, `DISABLE_SENSITIVE_TESTS` decides.

A cost test of a header-only algorithm builds at the build type's optimization
level, not at `GR_QA_OPTIMIZATION_LEVEL`, so its printed figure describes the
algorithm as the build type compiles it. A Debug build compiles it unoptimized,
and its bound must hold there as well.

## Using the Library

An installation of this repository also installs the `gnuradio4Library` CMake
package, with targets in the `gnuradio4::` namespace. A downstream CMake project
can consume the algorithm library with:

```cmake
find_package(gnuradio4Library CONFIG REQUIRED)
target_link_libraries(my_target PRIVATE gnuradio4::gnuradio-algorithm)
```

Public headers are installed under `include/gnuradio-4.0/algorithm`. The
`gnuradio4Library.pc` pkg-config file is installed beside the CMake package.
Most public C++ APIs are in the `gr::algorithm` namespace; established APIs such
as tensor mathematics retain their `gr::math` namespace.

## License

Unless otherwise noted, this project is licensed under the
[MIT License](../LICENSE).
