# PocketFFT

Upstream project: <https://github.com/mreineck/pocketfft>, `cpp` branch.

Vendored from commit `4d865cba24359b05b37da4e75aee24498c3b78e7` of 2026-10-18 on upstream's `cpp` branch. SHA-256 of
`pocketfft_hdronly.h` as vendored:
`518a95c552aeaee352aa30122137cd9e135d23c91e2863d9272aaa758744c86d`. Copied here on 2026-10-23.

The header is taken verbatim and is not reformatted, respelled or otherwise edited; a change here would be an upgrade
to another upstream revision and nothing else. `algorithm/CMakeLists.txt` carries `third_party/include/` as a system
include for the same reason.

The directories above this one mirror the installed layout, so the include path `gnuradio-4.0/third_party/pocketfft/`
is the same in the source tree and in an install. A consumer that finds this project through pkg-config gets only
`-I${includedir}`, and that is all this path needs.

License: BSD-3-Clause, copyright the Max-Planck-Society and Peter Bell. The header's notice names three more holders
for parts of the code, under the same three clauses: Matteo Frigo and the Massachusetts Institute of Technology for
the odd-length DCT-IV path, Tan Ping Liang and Peter Bell for the `prev_good_size` search, and Cris Luengo for the
overflow checks in the `good_size` search. The full notice is in the header. `LICENSE.md` is upstream's license file.

Used by `gnuradio-4.0/algorithm/fourier/fft.hpp` in `gr::algorithm::FFT`. The `PocketFFT` backend holds its own
`pocketfft_c` or `pocketfft_r` plan. The `FourStep` backend calls `pocketfft::c2c` on a two-dimensional shape and builds
its two plans on each call, since the header's plan cache is compiled out. The passes of the `FourStep`
backend run on the header's thread pool. Include the header through `fft.hpp`, which sets `POCKETFFT_PTHREADS`.
