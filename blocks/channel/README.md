# channel

Channel models: what a signal meets between a transmitter and a receiver, applied
to a stream so a chain can be measured against a stated impairment rather than
against clean samples.

| block             | what it applies                                                 |
| ----------------- | --------------------------------------------------------------- |
| `AwgnChannel`     | additive white Gaussian noise at a stated signal-to-noise ratio |
| `FadingChannel`   | a multipath delay line with Doppler on each tap                 |
| `FrequencyOffset` | a carrier frequency and phase offset                            |
| `PhaseNoise`      | a phase random walk at a stated level                           |
| `IqImbalance`     | gain and quadrature imbalance between the two rails             |
| `Nonlinearity`    | amplitude and phase compression of a power amplifier            |
| `Quantizer`       | uniform quantization to a stated number of bits                 |

Each model's parameters stay live-settable, so a graph can sweep an impairment
without being rebuilt.

`gr::channel` is the algorithm layer's namespace, so the blocks here stay in `gr::blocks::channel` and ship no
compatibility import, which would collide with it.
