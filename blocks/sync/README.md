# sync

Carrier, frequency and symbol-timing recovery: the tracking loops of a receiver.
Every block here drives one shared second-order recursion,
`gr::sync::ControlLoop`, and differs from its neighbors only in the error
detector it feeds that recursion and in what it emits.

`gr::sync` is the algorithm layer's namespace, so the blocks here stay in `gr::blocks::sync` and ship no
compatibility import, which would collide with it.
