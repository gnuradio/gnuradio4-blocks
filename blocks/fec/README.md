# fec

Record-native adapters over the forward-error-correction kernels of
`gnuradio-4.0/algorithm/fec`: a record of information symbols in, a record of the
codeword out, and back again on the receive side.

| block                          | what it carries                                   |
| ------------------------------ | ------------------------------------------------- |
| `FecEncode` / `FecDecode`      | the block codes: Golay, Hamming and BCH           |
| `RsEncode` / `RsDecode`        | Reed-Solomon, shortened and interleaved           |
| `ConvEncode` / `ViterbiDecode` | convolutional encoding and hard-decision Viterbi  |
| `ViterbiDecodeSoft`            | the same trellis over soft values                 |
| `Puncture` / `Depuncture`      | rate matching against a stated pattern            |
| `Interleave` / `Deinterleave`  | block, convolutional and permutation interleaving |

Every block is an adapter and no block carries arithmetic of its own: the code
lives in the algorithm layer, and what these add is the record boundary, the
settings validation and the counters a graph reads.

The LDPC and Polar pairs are behind `GR4_ENABLE_AFF3CT`, which is off by default;
the module builds whole without it.

`gr::fec` is the algorithm layer's namespace, so the blocks here stay in `gr::blocks::fec` and ship no
compatibility import, which would collide with it.
